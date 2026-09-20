//! The live breaker system: the 399 real trip units of the A380's
//! Electrical Load Management System, actually running.
//!
//! [`super::catalog`] is the table and [`super::trip`] is the physics; until
//! now nothing owned a single [`super::trip::Breaker`]. This module owns all
//! 399 of them and drives each from the current its own protected circuit
//! genuinely draws.
//!
//! ## Where the current comes from
//!
//! This area deliberately does **not** solve a network. `deep::electrical`
//! owns the one solve in the crate (see its own `live.rs` doc comment for
//! why), and each frame it publishes the current every feeder carried.
//! This area reads that back through `deep::electrical::live::board` and
//! steps its own, richer trip curve against it: ambient-derated I^2t, a
//! magnetic instantaneous element, an SSPC arc-fault channel and the
//! repeated-trip lockout, none of which the network's own contact-level
//! element models. When a unit here opens, it publishes `BKR_<id>_OPEN`,
//! and `deep::electrical` pulls the matching contacts on the next frame.
//! That is one frame of lag on a device whose fastest element (magnetic)
//! is specified in tens of milliseconds and whose slowest (thermal) is
//! specified in seconds.
//!
//! The two areas are in series, which is exactly what a real circuit is:
//! the load's own feeder protection and the power centre's SSPC both sit on
//! the same wire, and whichever reaches its threshold first clears the
//! fault. Because this catalogue's ratings are rounded **up** to the
//! AS39019 standard series while `deep::electrical`'s are the raw margined
//! current, the network's own element is the lower-rated of the two and
//! normally clears a plain overload first. What this area adds on top is
//! everything that curve cannot see: a drifted trip point below the
//! nominal rating, an arcing fault's current signature, a bay running hot,
//! and a circuit that has already tripped three times in five minutes.
//!
//! ## The 3 breakers that still protect nothing modelled
//!
//! A gap-closing pass gave 125 of the original 128 "protects nothing
//! modelled" breakers a real load in `deep::electrical::loads.rs`, so they
//! now see the current their own protected circuit genuinely draws like
//! every group-1 breaker. Only ATA24's three battery-output breakers
//! (BATTERY 1, BATTERY 2, APU BATTERY) remain: a battery's own output
//! breaker protects a *source's* output current, not a consumer's demand,
//! and `network::Load` (the only thing this area's current comes from)
//! models demand only -- see `catalog::ata24_power_sources`'s own comment.
//! `integration_test::breakers_protecting_no_modelled_load_are_a_known_named_gap`
//! pins the remaining three. Those three units are constructed and stepped
//! here like every other, but the current they see is genuinely zero,
//! because no model in this crate draws through them. Their two failures
//! are therefore registered and armable but cannot produce an effect --
//! honestly inert rather than fed a fabricated current.

use std::collections::HashMap;

use crate::deep::api::Registry;
use crate::deep::electrical::live::board;
use crate::deep::live::{Area, Faults, Truth};

use super::catalog::{self, BreakerDef};
use super::trip::{Breaker, BreakerFaults, SspcStatus, TripCause};

/// Which of a unit's two health channels a registered failure drives.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Channel {
    /// `trip::BreakerFaults::trip_calibration_drift`.
    CalibrationDrift,
    /// `trip::BreakerFaults::contact_resistance`.
    ContactWeld,
}

/// Published `BKR_<id>_STATUS` encoding -- one number carrying the whole of
/// [`SspcStatus`], for a CDS/OIT-style CB page that wants to draw the real
/// state rather than just open/closed.
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
    /// This breaker's index in `deep::electrical`'s solved network, when it
    /// protects a load that area actually models; `None` for the 3 that
    /// protect real equipment with no load model in this crate.
    net_index: Option<usize>,
    drift: f64,
    weld: f64,
    current_a: f64,
    open_var: String,
    status_var: String,
}

pub struct BreakersLive {
    units: Vec<Unit>,
    /// `(failure id, unit index, channel)` for all 798 registered failures.
    routed: Vec<(u64, usize, Channel)>,
    /// The `deep::electrical` failure id that describes the same physical
    /// welded-contact channel as this area's own `contact_resistance`
    /// (see `deep::electrical::live`'s own note on the duplication).
    welded_mirror: Vec<(usize, u64)>,
    faults_were_armed: bool,
    tripped_count: f64,
    locked_out_count: f64,
    unprotected_count: f64,
}

/// This area's live system, constructed cold: every catalogue breaker
/// closed, cold and healthy.
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
                net_index: def.protected_load.and_then(|_| topo.breaker_index.get(def.id).copied()),
                drift: 0.0,
                weld: 0.0,
                current_a: 0.0,
                open_var: format!("BKR_{}_OPEN", def.id),
                status_var: format!("BKR_{}_STATUS", def.id),
            });
        }

        // Route this area's own registered failures onto their units.
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
        debug_assert_eq!(routed.len(), reg.failures.len(), "every registered breaker failure must route onto a unit");

        // The mirror of the same physical channel in `deep::electrical`.
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
        Self { units, routed, welded_mirror, faults_were_armed: false, tripped_count: 0.0, locked_out_count: 0.0, unprotected_count }
    }

    /// Read-only access to one unit's live trip state, for tests and for
    /// whatever Study/CB page draws the real panel.
    pub fn breaker(&self, id: &str) -> Option<&Breaker> {
        self.units.iter().find(|u| u.def.id == id).map(|u| &u.breaker)
    }

    pub fn breaker_mut(&mut self, id: &str) -> Option<&mut Breaker> {
        self.units.iter_mut().find(|u| u.def.id == id).map(|u| &mut u.breaker)
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

        // No `Truth` field carries an equipment-bay/power-centre air
        // temperature, and a thermal breaker's trip point genuinely depends
        // on one (`trip::thermal_ambient_derate`). Static air temperature is
        // the only ambient available -- see this area's report.
        let ambient_c = truth.environment.sat_c;

        board::with_board(|b| {
            for u in &mut self.units {
                u.current_a = match u.net_index {
                    Some(i) if i < b.breaker_current_a.len() => b.breaker_current_a[i],
                    // Either the frame before `deep::electrical` has ever
                    // published, or one of the 3 breakers whose consumer
                    // this crate does not model: no current exists to read,
                    // and inventing one would be a fabricated input.
                    _ => 0.0,
                };
                u.breaker.step(u.current_a, ambient_c, BreakerFaults { trip_calibration_drift: u.drift, contact_resistance: u.weld }, dt);
            }
        });

        self.tripped_count = self.units.iter().filter(|u| !u.breaker.closed).count() as f64;
        self.locked_out_count = self.units.iter().filter(|u| u.breaker.is_locked_out()).count() as f64;
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        for u in &self.units {
            out(&u.open_var, if u.breaker.closed { 0.0 } else { 1.0 });
            out(&u.status_var, status_code(u.breaker.status()));
        }
        out("BREAKERS_TOTAL", self.units.len() as f64);
        out("BREAKERS_OPEN_COUNT", self.tripped_count);
        out("BREAKERS_LOCKED_OUT_COUNT", self.locked_out_count);
        out("BREAKERS_PROTECTING_NO_MODELLED_LOAD", self.unprotected_count);

        // The typed side of `BKR_<id>_OPEN`, which `deep::electrical` reads
        // back next frame to open the matching contacts.
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
        assert_eq!(live.routed.len(), reg.failures.len());
        assert_eq!(live.units.len(), catalog::all().len());
        assert_eq!(live.units.len(), 399, "the catalogue is the 399-breaker table this area's COUNTS.md documents");
    }

    #[test]
    fn exactly_the_known_gap_of_units_has_no_current_source() {
        let live = BreakersLive::new();
        let without: Vec<&str> = live.units.iter().filter(|u| u.net_index.is_none()).map(|u| u.def.id).collect();
        assert_eq!(without.len(), 3, "the known named gap: breakers whose consumer deep::electrical does not model, got {without:?}");
        let mut sorted = without.clone();
        sorted.sort_unstable();
        assert_eq!(sorted.as_slice(), ["bat-1", "bat-2", "bat-apu"], "the gap must be exactly the three battery-output breakers, nothing else");
        assert_eq!(live.unprotected_count, 3.0);
    }

    #[test]
    #[ignore = "diagnostic"]
    fn diagnose() {
        board::clear();
        let mut elec = ElectricalLive::new();
        let mut live = BreakersLive::new();
        let truth = flying_truth();
        let faults = Faults::default();
        for _ in 0..120 {
            elec.tick(&truth, &faults);
            live.tick(&truth, &faults);
            elec.publish(&mut |_, _| {});
            live.publish(&mut |_, _| {});
        }
        let open: Vec<String> = live
            .units
            .iter()
            .filter(|u| !u.breaker.closed)
            .map(|u| format!("{} I={:.2} rated={:.1} kind={:?} cause={:?}", u.def.id, u.current_a, u.def.rating_a, u.def.kind, u.breaker.trip_cause))
            .collect();
        println!("open {}:", open.len());
        for o in &open {
            println!("  {o}");
        }
        board::clear();
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

    /// Picks the breaker whose own steady load sits closest to what a 40%
    /// drifted trip point would leave it, so the test is about the model
    /// rather than about one hand-picked id. Returns `(id, ratio)`.
    fn most_drift_sensitive(live: &BreakersLive) -> (&'static str, f64) {
        live.units
            .iter()
            .filter(|u| u.net_index.is_some() && u.current_a > 0.0)
            .map(|u| (u.def.id, u.current_a / (u.def.rating_a * 0.6)))
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
            .expect("some breaker must be carrying current")
    }

    /// The registered effect of `trip_calibration_drift` is: "<name> opens
    /// under a load it should carry, de-energising <its load>". Arm it on a
    /// breaker carrying its own real steady load and it must do exactly
    /// that -- and the load behind it must actually go dead, which is the
    /// whole point of the two areas being coupled.
    #[test]
    fn a_drifted_trip_point_opens_a_breaker_under_its_own_normal_load_and_kills_that_load() {
        board::clear();
        let truth = flying_truth();
        let mut elec = ElectricalLive::new();
        let mut live = BreakersLive::new();

        // Settle healthy first so the electrical area has published a real
        // current for every breaker.
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
        // The I^2t element needs real time to integrate; step up to 120 s.
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
        board::clear();
    }

    /// The registered effect of `contact_resistance`: "<name> does not open
    /// on a genuine overload/short ... downstream of a breaker that should
    /// have isolated it". The overload used here is the *other* registered
    /// failure of the same breaker (a drifted trip point, which puts its own
    /// normal load past its effective rating): with the contacts welded, the
    /// breaker that would otherwise have opened stays shut.
    ///
    /// Run three times, because the same physical weld is catalogued twice
    /// -- once here as `17_breakers.<id>`'s `contact_resistance` and once in
    /// `deep::electrical` as `24_elec.bkr.<id>`'s `fails_to_trip` -- and
    /// arming *either* id has to weld the one real device.
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
        assert_eq!(published["BREAKERS_TOTAL"], 399.0);
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
}

#[cfg(test)]
mod cost {
    use super::*;
    use crate::deep::electrical::live::ElectricalLive;
    use crate::deep::wiring::live::WiringLive;

    /// Frame cost of the three coupled areas together, measured rather than
    /// asserted (`cargo test -- --ignored --nocapture deep::breakers::live::cost`).
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
