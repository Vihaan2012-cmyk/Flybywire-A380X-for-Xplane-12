use std::collections::HashMap;

use crate::deep::api::Registry;
use crate::deep::electrical::live::board;
use crate::deep::live::{Area, Faults, Truth};

use super::catalog::{self, BreakerDef};
use super::trip::{Breaker, BreakerFaults, SspcStatus, TripCause};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Channel {
    CalibrationDrift,
    ContactWeld,
}

fn status_code(s: SspcStatus) -> f64 {
    match s {
        SspcStatus::Closed => 0.0,
        SspcStatus::OpenCommanded => 1.0,
        SspcStatus::Tripped(TripCause::None) => 1.0,
        SspcStatus::Tripped(TripCause::Thermal) => 2.0,
        SspcStatus::Tripped(TripCause::Magnetic) => 3.0,
        SspcStatus::Tripped(TripCause::ArcFault) => 4.0,
        SspcStatus::LockedOut => 5.0,
    }
}

struct Unit {
    def: &'static BreakerDef,
    breaker: Breaker,
    net_index: Option<usize>,
    drift: f64,
    weld: f64,
    current_a: f64,
    open_var: String,
    status_var: String,
}

pub struct BreakersLive {
    units: Vec<Unit>,
    routed: Vec<(u64, usize, Channel)>,
    welded_mirror: Vec<(usize, u64)>,
    faults_were_armed: bool,
    tripped_count: f64,
    fault_tripped_count: f64,
    locked_out_count: f64,
    unprotected_count: f64,
    misc_routed: Vec<(u64, usize)>,
    cb_monitoring_fault: f64,
    emer_cb_monitoring_fault: f64,
    remote_cb_ctl_on: f64,
    drift_inert_count: f64,
}

pub fn live_system() -> Box<dyn Area> {
    Box::new(BreakersLive::new())
}

impl Default for BreakersLive {
    fn default() -> Self {
        Self::new()
    }
}

impl BreakersLive {
    pub fn new() -> Self {
        let topo = board::topology();
        let mut by_id: HashMap<&'static str, usize> = HashMap::new();
        let mut units: Vec<Unit> = Vec::with_capacity(catalog::all().len());
        for def in catalog::all() {
            by_id.insert(def.id, units.len());
            units.push(Unit {
                def,
                breaker: Breaker::new(def.kind, def.rating_a),
                net_index: topo.breaker_index.get(def.id).copied().or_else(|| topo.breaker_index.get(format!("{}-bkr", def.id).as_str()).copied()),
                drift: 0.0,
                weld: 0.0,
                current_a: 0.0,
                open_var: format!("BKR_{}_OPEN", def.id),
                status_var: format!("BKR_{}_STATUS", def.id),
            });
        }

        let mut reg = Registry::default();
        super::registry::register(&mut reg);
        let mut routed = Vec::with_capacity(reg.failures.len());
        for f in &reg.failures {
            let Some(id) = f.component.strip_prefix("17_breakers.") else { continue };
            let Some(&unit) = by_id.get(id) else { continue };
            let channel = if f.model_field.contains("trip_calibration_drift") {
                Channel::CalibrationDrift
            } else if f.model_field.contains("contact_resistance") {
                Channel::ContactWeld
            } else {
                continue;
            };
            routed.push((f.id, unit, channel));
        }
        let mut misc_routed = Vec::new();
        for f in &reg.failures {
            let Some(suffix) = f.component.strip_prefix("17_breakers.misc.") else { continue };
            let idx = match suffix {
                "cb-monitoring" => 0,
                "emer-cb-monitoring" => 1,
                "remote-cb-ctl" => 2,
                _ => continue,
            };
            misc_routed.push((f.id, idx));
        }
        debug_assert_eq!(routed.len() + misc_routed.len(), reg.failures.len(), "every registered breaker failure must route onto a unit or onto misc_routed");

        let mut elec = Registry::default();
        crate::deep::electrical::registry::register(&mut elec);
        let mut welded_mirror = Vec::new();
        for f in &elec.failures {
            if !f.model_field.ends_with("fails_to_trip") {
                continue;
            }
            let Some(id) = f.component.strip_prefix("24_elec.bkr.") else { continue };
            if let Some(&unit) = by_id.get(id) {
                welded_mirror.push((unit, f.id));
            }
        }

        let unprotected_count = units.iter().filter(|u| u.def.protected_load.is_none()).count() as f64;
        Self {
            units,
            routed,
            welded_mirror,
            faults_were_armed: false,
            tripped_count: 0.0,
            fault_tripped_count: 0.0,
            locked_out_count: 0.0,
            unprotected_count,
            drift_inert_count: 0.0,
            misc_routed,
            cb_monitoring_fault: 0.0,
            emer_cb_monitoring_fault: 0.0,
            remote_cb_ctl_on: 0.0,
        }
    }

    pub fn breaker(&self, id: &str) -> Option<&Breaker> {
        self.units.iter().find(|u| u.def.id == id).map(|u| &u.breaker)
    }

    pub fn breaker_mut(&mut self, id: &str) -> Option<&mut Breaker> {
        self.units.iter_mut().find(|u| u.def.id == id).map(|u| &mut u.breaker)
    }

    pub fn units(&self) -> impl Iterator<Item = (&'static str, &Breaker, f64)> + '_ {
        self.units.iter().map(|u| (u.def.id, &u.breaker, u.current_a))
    }

    pub fn breaker_at_mut(&mut self, index: usize) -> Option<&mut Breaker> {
        self.units.get_mut(index).map(|u| &mut u.breaker)
    }
}

impl Area for BreakersLive {
    fn name(&self) -> &'static str {
        "breakers"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        let dt = truth.dt_s.max(0.0);

        let armed = faults.any();
        if armed || self.faults_were_armed {
            for u in &mut self.units {
                u.drift = 0.0;
                u.weld = 0.0;
            }
            for &(id, unit, channel) in &self.routed {
                let m = faults.get(id);
                if m <= 0.0 {
                    continue;
                }
                let u = &mut self.units[unit];
                match channel {
                    Channel::CalibrationDrift => u.drift = u.drift.max(m),
                    Channel::ContactWeld => u.weld = u.weld.max(m),
                }
            }
            for &(unit, id) in &self.welded_mirror {
                let m = faults.get(id);
                if m > 0.0 {
                    let u = &mut self.units[unit];
                    u.weld = u.weld.max(m);
                }
            }
        }
        self.faults_were_armed = armed;

        let mut misc_state = [0.0f64; 3];
        for &(id, idx) in &self.misc_routed {
            misc_state[idx] = misc_state[idx].max(faults.get(id));
        }
        self.cb_monitoring_fault = misc_state[0];
        self.emer_cb_monitoring_fault = misc_state[1];
        self.remote_cb_ctl_on = misc_state[2].max(if truth.controls.remote_cb_ctl_active { 1.0 } else { 0.0 });

        let ambient_c = truth.environment.sat_c;

        board::with_board(|b| {
            for u in &mut self.units {
                u.current_a = match u.net_index {
                    Some(i) if i < b.breaker_current_a.len() => b.breaker_current_a[i],
                    _ => 0.0,
                };
                u.breaker.step(u.current_a, ambient_c, BreakerFaults { trip_calibration_drift: u.drift, contact_resistance: u.weld }, dt);
            }
        });

        self.tripped_count = self.units.iter().filter(|u| !u.breaker.closed).count() as f64;
        self.fault_tripped_count = self.units.iter().filter(|u| matches!(u.breaker.status(), SspcStatus::Tripped(_) | SspcStatus::LockedOut)).count() as f64;
        self.locked_out_count = self.units.iter().filter(|u| u.breaker.is_locked_out()).count() as f64;
        self.drift_inert_count = self.units.iter().filter(|u| u.current_a < u.breaker.minimum_trip_current_a(ambient_c)).count() as f64;
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        for u in &self.units {
            out(&u.open_var, if u.breaker.closed { 0.0 } else { 1.0 });
            out(&u.status_var, status_code(u.breaker.status()));
        }
        out("BREAKERS_TOTAL", self.units.len() as f64);
        out("BREAKERS_OPEN_COUNT", self.tripped_count);
        out("BREAKERS_TRIPPED_NOT_COMMANDED_COUNT", self.fault_tripped_count);
        out("BREAKERS_LOCKED_OUT_COUNT", self.locked_out_count);
        out("BREAKERS_PROTECTING_NO_MODELLED_LOAD", self.unprotected_count);
        out("BREAKERS_DRIFT_INERT_COUNT", self.drift_inert_count);
        out("BREAKERS_CB_MONITORING_FAULT", if self.cb_monitoring_fault > 0.0 { 1.0 } else { 0.0 });
        out("BREAKERS_EMER_CB_MONITORING_FAULT", if self.emer_cb_monitoring_fault > 0.0 { 1.0 } else { 0.0 });
        out("BREAKERS_REMOTE_CTL_ACTIVE", if self.remote_cb_ctl_on > 0.0 { 1.0 } else { 0.0 });

        board::with_board_mut(|b| {
            let n = board::topology().breaker_count;
            if b.breaker_open_cmd.len() != n {
                b.breaker_open_cmd = vec![0.0; n];
            }
            for v in b.breaker_open_cmd.iter_mut() {
                *v = 0.0;
            }
            for u in &self.units {
                if let Some(i) = u.net_index {
                    if i < n && !u.breaker.closed {
                        b.breaker_open_cmd[i] = 1.0;
                    }
                }
            }
        });
    }

    fn as_breakers(&self) -> Option<&BreakersLive> {
        Some(self)
    }

    fn as_breakers_mut(&mut self) -> Option<&mut BreakersLive> {
        Some(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::electrical::live::ElectricalLive;
    use crate::deep::live::Deep;
    use std::collections::BTreeMap;

    fn flying_truth() -> Truth {
        Truth { dt_s: 1.0 / 30.0, on_ground: false, engine_n1_frac: [0.9; 4], engine_n2_frac: [0.9; 4], engine_n3_frac: [0.9; 4], engine_running: [true; 4], ..Truth::default() }
    }

    fn failure_id_for(component_suffix: &str, field: &str) -> u64 {
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        reg.failures
            .iter()
            .find(|f| f.component == format!("17_breakers.{component_suffix}") && f.model_field.contains(field))
            .unwrap_or_else(|| panic!("no registered {field} failure for {component_suffix}"))
            .id
    }

    #[test]
    fn every_registered_failure_routes_onto_a_real_unit() {
        let live = BreakersLive::new();
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        assert_eq!(live.routed.len() + live.misc_routed.len(), reg.failures.len());
        assert_eq!(live.units.len(), catalog::all().len());
        assert_eq!(live.units.len(), 335, "the catalogue with one breaker per real fuel pump, valve and valve position indicator in place of the 145 numbered fuel placeholders");
    }

    #[test]
    fn exactly_the_known_gap_of_units_has_no_current_source() {
        let live = BreakersLive::new();
        let without: Vec<&str> = live.units.iter().filter(|u| u.net_index.is_none()).map(|u| u.def.id).collect();
        assert_eq!(without.len(), 1, "the known named gap: breakers whose consumer deep::electrical does not model, got {without:?}");
        assert_eq!(without.as_slice(), ["bat-apu"], "bat-1/bat-2 now read their real feeder-breaker current from the network solver (bat-N-bkr); only the unmodelled APU battery remains a gap");
        assert_eq!(live.unprotected_count, 3.0, "protected_load stays honestly None for all three battery-output breakers; this is a separate accounting of modelled downstream load, not of current data");
    }

    #[test]
    fn a_healthy_aircraft_trips_no_breaker_in_any_state_over_a_long_run() {
        use crate::deep::integration::failure_audit::profiles;
        use crate::deep::wiring::live::WiringLive;

        for (dt, seconds) in [(1.0 / 30.0, 120.0), (1.0, 600.0)] {
            for p in profiles() {
                board::clear();
                let mut elec = ElectricalLive::new();
                let mut live = BreakersLive::new();
                let mut wire = WiringLive::new();
                let truth = Truth { dt_s: dt, ..(p.truth)() };
                let faults = Faults::default();
                let frames = (seconds / dt).ceil() as usize;
                for f in 0..frames {
                    elec.tick(&truth, &faults);
                    live.tick(&truth, &faults);
                    wire.tick(&truth, &faults);
                    if let Some(u) = live.units.iter().find(|u| !u.breaker.closed) {
                        panic!(
                            "healthy aircraft: {} tripped ({:?}) in profile {} at t={:.2} s, dt={dt}, carrying {:.2} A on a {:.1} A rating",
                            u.def.id,
                            u.breaker.trip_cause,
                            p.name,
                            f as f64 * dt,
                            u.current_a,
                            u.def.rating_a
                        );
                    }
                    elec.publish(&mut |_, _| {});
                    live.publish(&mut |_, _| {});
                    wire.publish(&mut |_, _| {});
                }
                board::clear();
            }
        }
    }

    #[test]
    fn no_healthy_circuit_settles_above_its_own_breakers_rating() {
        use crate::deep::integration::failure_audit::profiles;
        use crate::deep::wiring::live::WiringLive;

        for p in profiles().into_iter().filter(|p| p.name != "cold_dark") {
            board::clear();
            let mut elec = ElectricalLive::new();
            let mut live = BreakersLive::new();
            let mut wire = WiringLive::new();
            let truth = Truth { dt_s: 0.1, ..(p.truth)() };
            let faults = Faults::default();
            let settle = 600;
            let mut worst = (0.0f64, "");
            for f in 0..settle {
                elec.tick(&truth, &faults);
                live.tick(&truth, &faults);
                wire.tick(&truth, &faults);
                for u in live.units.iter_mut() {
                    u.breaker.maintenance_clear_lockout();
                    let _ = u.breaker.reset();
                }
                if f >= settle - 100 {
                    for u in live.units.iter() {
                        let r = u.current_a / u.def.rating_a.max(1e-9);
                        if r > worst.0 {
                            worst = (r, u.def.id);
                        }
                    }
                }
                elec.publish(&mut |_, _| {});
                live.publish(&mut |_, _| {});
                wire.publish(&mut |_, _| {});
            }
            assert!(worst.0 <= 1.0, "in profile {}, healthy circuit {} settles at {:.3}x its own breaker's rating", p.name, worst.1, worst.0);
            board::clear();
        }
    }

    #[test]
    fn a_healthy_aircraft_leaves_every_breaker_closed() {
        board::clear();
        let mut elec = ElectricalLive::new();
        let mut live = BreakersLive::new();
        let truth = flying_truth();
        let faults = Faults::default();
        let mut published = BTreeMap::new();
        for _ in 0..60 {
            elec.tick(&truth, &faults);
            live.tick(&truth, &faults);
            published.clear();
            elec.publish(&mut |n, v| {
                published.insert(n.to_string(), v);
            });
            live.publish(&mut |n, v| {
                published.insert(n.to_string(), v);
            });
        }
        assert_eq!(published["BREAKERS_OPEN_COUNT"], 0.0, "a healthy aircraft must not trip anything");
        board::clear();
    }

    fn most_drift_sensitive(live: &BreakersLive) -> (&'static str, f64) {
        live.units
            .iter()
            .filter(|u| u.net_index.is_some() && u.current_a > 0.0)
            .map(|u| (u.def.id, u.current_a / (u.def.rating_a * 0.6)))
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
            .expect("some breaker must be carrying current")
    }

    #[test]
    fn a_drifted_trip_point_opens_a_breaker_under_its_own_normal_load_and_kills_that_load() {
        board::clear();
        let truth = flying_truth();
        let mut elec = ElectricalLive::new();
        let mut live = BreakersLive::new();

        let healthy = Faults::default();
        for _ in 0..60 {
            elec.tick(&truth, &healthy);
            live.tick(&truth, &healthy);
            elec.publish(&mut |_, _| {});
            live.publish(&mut |_, _| {});
        }
        let (id, ratio) = most_drift_sensitive(&live);
        assert!(ratio > 1.0, "{id} is the worst case at {ratio:.3}x of a drifted rating -- nothing in the catalogue would trip");
        assert!(live.breaker(id).unwrap().closed);
        let load_var = format!("ELEC_LOAD_{id}_POWERED");
        let open_var = format!("BKR_{id}_OPEN");

        let armed = Faults::from_pairs([(failure_id_for(id, "trip_calibration_drift"), 1.0)]);
        let mut published = BTreeMap::new();
        let slow = Truth { dt_s: 0.05, ..truth.clone() };
        for _ in 0..2_400 {
            elec.tick(&slow, &armed);
            live.tick(&slow, &armed);
            published.clear();
            elec.publish(&mut |n, v| {
                published.insert(n.to_string(), v);
            });
            live.publish(&mut |n, v| {
                published.insert(n.to_string(), v);
            });
            if published.get(&open_var) == Some(&1.0) && published.get(&load_var) == Some(&0.0) {
                break;
            }
        }
        assert_eq!(published[&open_var], 1.0, "a 40%-low trip point must open {id} under its own normal load");
        assert_eq!(published[&load_var], 0.0, "and the load behind it must actually go dead");
        assert_eq!(published[&format!("ELEC_BKR_{id}_CLOSED")], 0.0);
        assert_eq!(published["BREAKERS_OPEN_COUNT"], 1.0, "nothing else may move");
        assert_eq!(published["BREAKERS_TRIPPED_NOT_COMMANDED_COUNT"], 1.0, "a thermal trip is exactly what that aggregate is for");
        board::clear();
    }

    #[test]
    fn a_crew_pulled_breaker_is_open_but_not_tripped() {
        board::clear();
        let truth = flying_truth();
        let mut elec = ElectricalLive::new();
        let mut live = BreakersLive::new();
        let faults = Faults::default();
        let mut published = BTreeMap::new();
        let mut settle = |elec: &mut ElectricalLive, live: &mut BreakersLive, published: &mut BTreeMap<String, f64>| {
            for _ in 0..60 {
                elec.tick(&truth, &faults);
                live.tick(&truth, &faults);
                published.clear();
                elec.publish(&mut |n, v| {
                    published.insert(n.to_string(), v);
                });
                live.publish(&mut |n, v| {
                    published.insert(n.to_string(), v);
                });
            }
        };
        settle(&mut elec, &mut live, &mut published);
        assert_eq!(published["BREAKERS_TRIPPED_NOT_COMMANDED_COUNT"], 0.0);

        let id = live.units.iter().find(|u| u.net_index.is_some()).expect("some breaker protects a modelled load").def.id;
        live.breaker_mut(id).expect("the id came from the unit list").pull();
        settle(&mut elec, &mut live, &mut published);
        assert_eq!(published[&format!("BKR_{id}_OPEN")], 1.0, "a pulled breaker is open");
        assert_eq!(published["BREAKERS_OPEN_COUNT"], 1.0, "and the open count says so");
        assert_eq!(published["BREAKERS_TRIPPED_NOT_COMMANDED_COUNT"], 0.0, "but nothing tripped -- the crew did it");
        board::clear();
    }

    #[test]
    fn a_welded_breaker_does_not_open_on_an_overload_that_would_otherwise_trip_it() {
        let truth = Truth { dt_s: 0.05, on_ground: false, engine_n1_frac: [0.9; 4], engine_n2_frac: [0.9; 4], engine_n3_frac: [0.9; 4], engine_running: [true; 4], ..Truth::default() };

        let run = |id: &str, faults: &Faults| -> bool {
            board::clear();
            let mut elec = ElectricalLive::new();
            let mut live = BreakersLive::new();
            let mut cleared = false;
            for _ in 0..2_400 {
                elec.tick(&truth, faults);
                live.tick(&truth, faults);
                elec.publish(&mut |_, _| {});
                live.publish(&mut |_, _| {});
                if !live.breaker(id).unwrap().closed {
                    cleared = true;
                    break;
                }
            }
            board::clear();
            cleared
        };

        let id = {
            board::clear();
            let mut elec = ElectricalLive::new();
            let mut live = BreakersLive::new();
            for _ in 0..60 {
                elec.tick(&truth, &Faults::default());
                live.tick(&truth, &Faults::default());
                elec.publish(&mut |_, _| {});
                live.publish(&mut |_, _| {});
            }
            most_drift_sensitive(&live).0
        };
        let drift = failure_id_for(id, "trip_calibration_drift");
        assert!(run(id, &Faults::from_pairs([(drift, 1.0)])), "a drifted trip point alone must open {id} within 120 s");

        let weld_here = failure_id_for(id, "contact_resistance");
        assert!(!run(id, &Faults::from_pairs([(drift, 1.0), (weld_here, 0.999)])), "welded contacts must not open on that same overload");

        let weld_in_electrical = {
            let mut reg = Registry::default();
            crate::deep::electrical::registry::register(&mut reg);
            reg.failures
                .iter()
                .find(|f| f.component == format!("24_elec.bkr.{id}") && f.model_field.ends_with("fails_to_trip"))
                .expect("the same breaker's fails-to-trip must be registered in deep::electrical too")
                .id
        };
        assert!(
            !run(id, &Faults::from_pairs([(drift, 1.0), (weld_in_electrical, 0.999)])),
            "arming the other area's id for the same physical channel has to weld the one real device, not just that area's copy of it"
        );
    }

    #[test]
    fn it_plugs_into_deep_alongside_the_electrical_area() {
        board::clear();
        let mut deep = Deep::new().with_area(crate::deep::electrical::live::live_system()).with_area(live_system());
        let mut published = BTreeMap::new();
        deep.tick(flying_truth(), &Faults::default(), &mut |n, v| {
            published.insert(n.to_string(), v);
        });
        assert_eq!(deep.area_names(), vec!["electrical", "breakers"]);
        assert_eq!(published["BREAKERS_TOTAL"], 335.0);
        assert!(published.contains_key("BKR_cab-fan-1_STATUS"));
        board::clear();
    }

    #[test]
    fn nothing_produces_a_nan_at_rest_or_at_zero_dt() {
        board::clear();
        let mut live = BreakersLive::new();
        let truth = Truth { dt_s: 0.0, ..Truth::default() };
        for _ in 0..3 {
            live.tick(&truth, &Faults::default());
        }
        live.publish(&mut |name, v| assert!(v.is_finite(), "{name} is {v}"));
    }

    #[test]
    fn how_many_drift_failures_are_inert_by_construction() {
        use crate::deep::integration::failure_audit::profiles;

        let mut best: HashMap<&'static str, f64> = HashMap::new();
        for p in profiles() {
            board::clear();
            let mut elec = ElectricalLive::new();
            let mut live = BreakersLive::new();
            let truth = Truth { dt_s: 0.1, ..(p.truth)() };
            let faults = Faults::default();
            for f in 0..600 {
                elec.tick(&truth, &faults);
                live.tick(&truth, &faults);
                for u in live.units.iter_mut() {
                    u.breaker.maintenance_clear_lockout();
                    let _ = u.breaker.reset();
                }
                elec.publish(&mut |_, _| {});
                live.publish(&mut |_, _| {});
                if f >= 100 {
                    for u in live.units.iter() {
                        let floor = u.breaker.minimum_trip_current_a(71.0).max(1e-9);
                        let r = u.current_a / floor;
                        let e = best.entry(u.def.id).or_insert(0.0);
                        if r > *e {
                            *e = r;
                        }
                    }
                }
            }
            board::clear();
        }

        let live = BreakersLive::new();
        let protecting_a_load = live.units.iter().filter(|u| u.net_index.is_some()).count();
        let mut inert: Vec<(&'static str, f64)> = live
            .units
            .iter()
            .filter(|u| u.net_index.is_some())
            .map(|u| (u.def.id, best.get(u.def.id).copied().unwrap_or(0.0)))
            .filter(|&(_, r)| r < 1.0)
            .collect();
        inert.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        let named: Vec<String> = inert.iter().map(|(id, r)| format!("{id} {r:.3}x")).collect();

        if let Some(&(closest, ratio)) = inert.first() {
            let armed = Faults::from_pairs([(failure_id_for(closest, "trip_calibration_drift"), 1.0)]);
            for p in profiles() {
                board::clear();
                let truth = Truth { dt_s: 0.1, ..(p.truth)() };
                let mut elec = ElectricalLive::new();
                let mut brk = BreakersLive::new();
                for _ in 0..1_200 {
                    elec.tick(&truth, &armed);
                    brk.tick(&truth, &armed);
                    elec.publish(&mut |_, _| {});
                    brk.publish(&mut |_, _| {});
                }
                assert!(
                    brk.breaker(closest).expect("the id came from the unit list").closed,
                    "{closest} is the inert unit closest to its drifted trip point ({ratio:.3}x), so full drift must not open it -- and it opened in profile {}",
                    p.name
                );
                board::clear();
            }
        }

        assert_eq!(
            inert.len(),
            DRIFT_INERT_IN_EVERY_STATE,
            "{} of the {protecting_a_load} breakers protecting a modelled load never reach their own fully-drifted trip point in any profile, \
             so that many registered trip_calibration_drift failures are inert by construction: {named:?}",
            inert.len()
        );
    }

    const DRIFT_INERT_IN_EVERY_STATE: usize = 40;

    #[test]
    fn the_live_drift_inert_count_is_published_and_is_a_real_number() {
        board::clear();
        let mut elec = ElectricalLive::new();
        let mut live = BreakersLive::new();
        let truth = flying_truth();
        let mut published = BTreeMap::new();
        for _ in 0..60 {
            elec.tick(&truth, &Faults::default());
            live.tick(&truth, &Faults::default());
            published.clear();
            elec.publish(&mut |n, v| {
                published.insert(n.to_string(), v);
            });
            live.publish(&mut |n, v| {
                published.insert(n.to_string(), v);
            });
        }
        let inert = published["BREAKERS_DRIFT_INERT_COUNT"];
        assert!(inert > 0.0, "a flying aircraft has circuits that are not switched on; none of their drift failures can act");
        assert!(inert < published["BREAKERS_TOTAL"], "and it cannot be all of them: {inert}");
        board::clear();
    }

}

#[cfg(test)]
mod cost {
    use super::*;
    use crate::deep::electrical::live::ElectricalLive;
    use crate::deep::wiring::live::WiringLive;

    #[test]
    #[ignore = "timing"]
    fn per_tick_cost() {
        board::clear();
        let truth = Truth { dt_s: 1.0 / 30.0, on_ground: false, engine_n1_frac: [0.9; 4], engine_n2_frac: [0.9; 4], engine_n3_frac: [0.9; 4], engine_running: [true; 4], ..Truth::default() };
        let faults = Faults::default();
        let t0 = std::time::Instant::now();
        let mut elec = ElectricalLive::new();
        let mut brk = BreakersLive::new();
        let mut wire = WiringLive::new();
        let build = t0.elapsed();

        for _ in 0..30 {
            elec.tick(&truth, &faults);
            brk.tick(&truth, &faults);
            wire.tick(&truth, &faults);
            elec.publish(&mut |_, _| {});
            brk.publish(&mut |_, _| {});
            wire.publish(&mut |_, _| {});
        }

        let n = 300;
        let mut vars = 0usize;
        let t = std::time::Instant::now();
        for _ in 0..n {
            elec.tick(&truth, &faults);
            brk.tick(&truth, &faults);
            wire.tick(&truth, &faults);
            vars = 0;
            elec.publish(&mut |_, _| vars += 1);
            brk.publish(&mut |_, _| vars += 1);
            wire.publish(&mut |_, _| vars += 1);
        }
        let per = t.elapsed() / n;
        println!("construction {build:?}; per tick+publish {per:?}; {vars} variables published per frame");
        board::clear();
    }
}
