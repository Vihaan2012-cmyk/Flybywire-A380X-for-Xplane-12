use std::collections::HashMap;

use crate::deep::api::Registry;
use crate::deep::electrical::live::board;
use crate::deep::live::{Area, Faults, Truth};

use super::arc::{arc_current_a, arc_heat_w};
use super::bundle::WireBundleNetwork;
use super::faults::{
    chafe_effect, connector_corrosion_effect, maintenance_damage_effect, open_wire_effect, rodent_damage_effect, water_ingress_effect,
    zone_overheat_effects, CircuitEffect, ARC_VOLTAGE_DROP_V,
};
use super::gauge::Awg;
use super::routing::build_generic_a380_network;
use super::zones::{Zone, ALL_ZONES};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    Chafe,
    BundleOverheat,
    ConnectorCorrosion,
    WaterIngress,
    RodentDamage,
    MaintenanceDamage,
    OpenWire,
}

const KINDS: [Kind; 7] =
    [Kind::Chafe, Kind::BundleOverheat, Kind::ConnectorCorrosion, Kind::WaterIngress, Kind::RodentDamage, Kind::MaintenanceDamage, Kind::OpenWire];

impl Kind {
    fn from_param(p: &str) -> Option<Kind> {
        Some(match p {
            "chafe_wear" => Kind::Chafe,
            "overheat_damage" => Kind::BundleOverheat,
            "connector_corrosion" => Kind::ConnectorCorrosion,
            "water_ingress" => Kind::WaterIngress,
            "rodent_damage" => Kind::RodentDamage,
            "maintenance_damage" => Kind::MaintenanceDamage,
            "open_wire_fatigue" => Kind::OpenWire,
            _ => return None,
        })
    }

    fn is_shunt(self) -> bool {
        matches!(self, Kind::Chafe | Kind::RodentDamage | Kind::MaintenanceDamage | Kind::WaterIngress)
    }

    fn is_arcing(self) -> bool {
        matches!(self, Kind::Chafe | Kind::RodentDamage | Kind::MaintenanceDamage)
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct CircuitDamage {
    open: f64,
    fault_current_a: f64,
    arc_heat_w: f64,
    series_ohm: f64,
}

struct Circuit {
    #[allow(dead_code)]
    id: &'static str,
    load_index: Option<usize>,
    bus_index: Option<usize>,
    rated_current_a: Option<f64>,
    rated_power_w: Option<f64>,
    feeder_ohm: f64,
    awg: Awg,
    arc_current_var: String,
    arc_heat_var: String,
}

pub struct WiringLive {
    net: WireBundleNetwork,
    zones: Vec<Zone>,
    severity: Vec<[f64; 7]>,
    routed: Vec<(u64, usize, usize)>,
    circuits: Vec<Circuit>,
    circuit_index: HashMap<&'static str, usize>,
    damage: Vec<CircuitDamage>,
    zone_arc_heat_w: Vec<f64>,
    faults_were_armed: bool,

    zone_overheat_var: Vec<String>,
    zone_arc_heat_var: Vec<String>,
    zone_worst_var: Vec<String>,
    segment_var: Vec<(String, usize)>,
    total_arc_heat_w: f64,
}

pub fn live_system() -> Box<dyn Area> {
    Box::new(WiringLive::new())
}

impl Default for WiringLive {
    fn default() -> Self {
        Self::new()
    }
}

fn bus_index_for_label(label: &str) -> Option<usize> {
    const LABELS: [&str; 17] = [
        "AC1", "AC2", "AC3", "AC4", "AC_ESS", "AC_ESS_SHED", "AC_EMER", "AC_GND_FLT_SVC", "DC1", "DC2", "DC_ESS", "DC_ESS_SHED", "DC_BAT", "DC_HOT1",
        "DC_HOT2", "DC_APU", "DC_GND_FLT_SVC",
    ];
    LABELS.iter().position(|&l| l == label)
}

fn nominal_voltage_for_bus_index(i: usize) -> f64 {
    if i < 8 {
        115.0
    } else {
        28.0
    }
}

impl WiringLive {
    pub fn new() -> Self {
        let net = build_generic_a380_network();

        let mut reg = Registry::default();
        super::registry::register(&mut reg);
        let mut zones: Vec<Zone> = Vec::new();
        let mut zone_of_component: HashMap<String, usize> = HashMap::new();
        for comp in &reg.components {
            let Some(tag) = comp.id.strip_prefix("91_wiring.harness_") else { continue };
            let Some(&zone) = ALL_ZONES.iter().find(|z| z.name().to_lowercase() == tag) else { continue };
            zone_of_component.insert(comp.id.clone(), zones.len());
            zones.push(zone);
        }

        let mut routed = Vec::with_capacity(reg.failures.len());
        for f in &reg.failures {
            let Some(&zi) = zone_of_component.get(&f.component) else { continue };
            let Some(param) = f.model_field.rsplit_once("[component param ").map(|(_, rest)| rest.trim_end_matches(']')) else { continue };
            let Some(kind) = Kind::from_param(param) else { continue };
            let ki = KINDS.iter().position(|&k| k == kind).expect("KINDS covers every Kind");
            routed.push((f.id, zi, ki));
        }
        debug_assert_eq!(routed.len(), reg.failures.len(), "every registered wiring failure must route onto a zone and kind");

        let topo = board::topology();
        let breaker_rating: HashMap<&'static str, (f64, f64)> =
            crate::deep::breakers::catalog::all().iter().map(|d| (d.id, (d.rating_a, d.rated_power_w))).collect();
        let bus_of_circuit = super::routing::circuit_buses();

        let mut ids: Vec<&'static str> = net.segments().iter().flat_map(|s| s.circuits.iter().map(|c| c.circuit)).collect();
        ids.sort_unstable();
        ids.dedup();

        let mut circuits = Vec::with_capacity(ids.len());
        let mut circuit_index = HashMap::new();
        for id in ids {
            let bus_index = bus_of_circuit.get(id).and_then(|l| bus_index_for_label(l));
            let feeder_ohm = match bus_index {
                Some(i) if i < 8 => 0.08,
                _ => 0.03,
            };
            let awg = net
                .segments_for_circuit(id)
                .iter()
                .filter_map(|s| s.wire_of(id))
                .map(|w| w.awg)
                .max_by(|a, b| a.resistance_per_m_at_20c().partial_cmp(&b.resistance_per_m_at_20c()).unwrap_or(std::cmp::Ordering::Equal))
                .unwrap_or(Awg::Size(20));
            let rating = breaker_rating.get(id).copied().or_else(|| breaker_rating.get(format!("{id}-normal-bkr").as_str()).copied());
            circuit_index.insert(id, circuits.len());
            circuits.push(Circuit {
                id,
                load_index: topo.load_index.get(id).copied(),
                bus_index,
                rated_current_a: rating.map(|(a, _)| a),
                rated_power_w: rating.map(|(_, w)| w),
                feeder_ohm,
                awg,
                arc_current_var: format!("WIRING_CIRCUIT_{id}_ARC_CURRENT_A"),
                arc_heat_var: format!("WIRING_CIRCUIT_{id}_ARC_HEAT_W"),
            });
        }

        let zone_overheat_var = zones.iter().map(|z| format!("WIRING_ZONE_{}_OVERHEAT_SEVERITY", z.name())).collect();
        let zone_arc_heat_var = zones.iter().map(|z| format!("WIRING_ZONE_{}_ARC_HEAT_W", z.name())).collect();
        let zone_worst_var = zones.iter().map(|z| format!("WIRING_ZONE_{}_WORST_FAULT_SEVERITY", z.name())).collect();
        let segment_var = net
            .segments()
            .iter()
            .filter_map(|s| zones.iter().position(|&z| z == s.zone).map(|zi| (format!("WIRING_SEGMENT_{}_FAULT_SEVERITY", s.id), zi)))
            .collect();

        let n_zones = zones.len();
        let n_circuits = circuits.len();
        Self {
            net,
            zones,
            severity: vec![[0.0; 7]; n_zones],
            routed,
            circuits,
            circuit_index,
            damage: vec![CircuitDamage::default(); n_circuits],
            zone_arc_heat_w: vec![0.0; n_zones],
            faults_were_armed: false,
            zone_overheat_var,
            zone_arc_heat_var,
            zone_worst_var,
            segment_var,
            total_arc_heat_w: 0.0,
        }
    }

    pub fn bundle_network(&self) -> &WireBundleNetwork {
        &self.net
    }

    fn source_voltage(&self, c: &Circuit, bus_voltage: &[f64; 17]) -> f64 {
        match c.bus_index {
            Some(i) => {
                let v = bus_voltage[i];
                if v > 0.0 {
                    v
                } else {
                    0.0
                }
            }
            None => 0.0,
        }
    }

    fn accumulate(d: &mut CircuitDamage, kind: Kind, effect: CircuitEffect, route_ohm: f64, source_v: f64) {
        match effect {
            CircuitEffect::Open => d.open = 1.0,
            CircuitEffect::ShortToStructure | CircuitEffect::CrosstalkShort { .. } => {
                if route_ohm > 0.0 {
                    d.fault_current_a = d.fault_current_a.max(source_v / route_ohm);
                }
            }
            CircuitEffect::HighResistance(r) => {
                if kind.is_shunt() {
                    let i = if kind.is_arcing() {
                        arc_current_a(source_v, route_ohm + r, ARC_VOLTAGE_DROP_V)
                    } else if route_ohm + r > 0.0 {
                        source_v / (route_ohm + r)
                    } else {
                        0.0
                    };
                    if i > d.fault_current_a {
                        d.fault_current_a = i;
                        d.arc_heat_w = if kind.is_arcing() { arc_heat_w(i, ARC_VOLTAGE_DROP_V) } else { i * i * r };
                    }
                } else {
                    d.series_ohm += r;
                }
            }
        }
    }
}

impl Area for WiringLive {
    fn name(&self) -> &'static str {
        "wiring"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        let armed = faults.any();
        if !armed && !self.faults_were_armed {
            return;
        }
        self.faults_were_armed = armed;

        for s in &mut self.severity {
            *s = [0.0; 7];
        }
        for &(id, zi, ki) in &self.routed {
            let m = faults.get(id);
            if m > 0.0 {
                self.severity[zi][ki] = self.severity[zi][ki].max(m);
            }
        }

        for d in &mut self.damage {
            *d = CircuitDamage::default();
        }
        for h in &mut self.zone_arc_heat_w {
            *h = 0.0;
        }

        let bus_voltage = board::with_board(|b| b.bus_voltage);
        let ambient_c = truth.environment.sat_c;
        let ambient = |_z: Zone| ambient_c;

        for zi in 0..self.zones.len() {
            let zone = self.zones[zi];
            let sev = self.severity[zi];
            if sev.iter().all(|&m| m <= 0.0) {
                continue;
            }

            let overheat = sev[1];
            if overheat > 0.0 {
                for (circuit, effect) in zone_overheat_effects(&self.net, zone, overheat) {
                    let Some(&ci) = self.circuit_index.get(circuit) else { continue };
                    let route_ohm = self.net.circuit_resistance_ohm(circuit, &ambient);
                    let source_v = self.source_voltage(&self.circuits[ci], &bus_voltage);
                    let mut d = self.damage[ci];
                    Self::accumulate(&mut d, Kind::Chafe, effect, route_ohm, source_v);
                    self.damage[ci] = d;
                }
            }

            let here = self.net.circuits_in_zone(zone);
            for circuit in here {
                let Some(&ci) = self.circuit_index.get(circuit) else { continue };
                let route_ohm = self.net.circuit_resistance_ohm(circuit, &ambient);
                let source_v = self.source_voltage(&self.circuits[ci], &bus_voltage);
                let mate = self
                    .net
                    .segments_in_zone(zone)
                    .iter()
                    .find(|s| s.wire_of(circuit).is_some())
                    .and_then(|s| s.circuits.iter().map(|c| c.circuit).find(|&other| other != circuit));

                let mut d = self.damage[ci];
                for (ki, &kind) in KINDS.iter().enumerate() {
                    let m = sev[ki];
                    if m <= 0.0 || kind == Kind::BundleOverheat {
                        continue;
                    }
                    let rated = self.circuits[ci].rated_current_a;
                    let effect = match kind {
                        Kind::Chafe => rated.and_then(|a| chafe_effect(m, a, mate)),
                        Kind::RodentDamage => rated.and_then(|a| rodent_damage_effect(m, self.circuits[ci].awg, a)),
                        Kind::MaintenanceDamage => rated.and_then(|a| maintenance_damage_effect(m, a, mate)),
                        Kind::ConnectorCorrosion => connector_corrosion_effect(m),
                        Kind::WaterIngress => water_ingress_effect(m),
                        Kind::OpenWire => open_wire_effect(m, route_ohm),
                        Kind::BundleOverheat => None,
                    };
                    if let Some(effect) = effect {
                        Self::accumulate(&mut d, kind, effect, route_ohm, source_v);
                    }
                }
                self.damage[ci] = d;
            }
        }

        self.total_arc_heat_w = 0.0;
        for zi in 0..self.zones.len() {
            let mut w = 0.0;
            for circuit in self.net.circuits_in_zone(self.zones[zi]) {
                if let Some(&ci) = self.circuit_index.get(circuit) {
                    w += self.damage[ci].arc_heat_w;
                }
            }
            self.zone_arc_heat_w[zi] = w;
            self.total_arc_heat_w += w;
        }
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        for zi in 0..self.zones.len() {
            out(&self.zone_overheat_var[zi], self.severity[zi][1]);
            out(&self.zone_arc_heat_var[zi], self.zone_arc_heat_w[zi]);
            out(&self.zone_worst_var[zi], self.severity[zi].iter().copied().fold(0.0, f64::max));
        }
        for (name, zi) in &self.segment_var {
            out(name, self.severity[*zi].iter().copied().fold(0.0, f64::max));
        }
        for (ci, c) in self.circuits.iter().enumerate() {
            out(&c.arc_current_var, self.damage[ci].fault_current_a);
            out(&c.arc_heat_var, self.damage[ci].arc_heat_w);
        }
        out("WIRING_TOTAL_ARC_HEAT_W", self.total_arc_heat_w);

        board::with_board_mut(|b| {
            let n = board::topology().load_count;
            if b.load_open.len() != n {
                b.load_open = vec![0.0; n];
                b.load_short = vec![0.0; n];
                b.load_high_resistance = vec![0.0; n];
            }
            for v in b.load_open.iter_mut() {
                *v = 0.0;
            }
            for v in b.load_short.iter_mut() {
                *v = 0.0;
            }
            for v in b.load_high_resistance.iter_mut() {
                *v = 0.0;
            }
            for (ci, c) in self.circuits.iter().enumerate() {
                let Some(li) = c.load_index else { continue };
                if li >= n {
                    continue;
                }
                let d = self.damage[ci];
                b.load_open[li] = d.open;
                let v = match c.bus_index {
                    Some(i) => {
                        let bv = b.bus_voltage[i];
                        if bv > 1.0 {
                            bv
                        } else {
                            nominal_voltage_for_bus_index(i)
                        }
                    }
                    None => continue,
                };
                if d.fault_current_a > 0.0 {
                    b.load_short[li] = (d.fault_current_a * c.feeder_ohm / v).clamp(0.0, 1.0);
                }
                if d.series_ohm > 0.0 {
                    if let Some(p) = c.rated_power_w {
                        let extra = p * d.series_ohm / (v * v);
                        b.load_high_resistance[li] = (extra / 0.5).clamp(0.0, 1.0);
                    }
                }
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::live::Deep;
    use std::collections::BTreeMap;

    fn failure_id_for(zone: Zone, param: &str) -> u64 {
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        let comp = format!("91_wiring.harness_{}", zone.name().to_lowercase());
        reg.failures
            .iter()
            .find(|f| f.component == comp && f.model_field.contains(&format!("[component param {param}]")))
            .unwrap_or_else(|| panic!("no registered {param} failure for {comp}"))
            .id
    }

    fn run(live: &mut WiringLive, truth: &Truth, faults: &Faults, frames: usize) -> BTreeMap<String, f64> {
        let mut published = BTreeMap::new();
        for _ in 0..frames {
            live.tick(truth, faults);
            published.clear();
            live.publish(&mut |n, v| {
                published.insert(n.to_string(), v);
            });
        }
        published
    }

    #[test]
    fn every_registered_failure_routes_onto_a_zone_and_a_kind() {
        let live = WiringLive::new();
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        assert_eq!(live.routed.len(), reg.failures.len());
        assert_eq!(live.zones.len(), 13);
        assert_eq!(live.routed.len(), 13 * 7);
        assert!(!live.circuits.is_empty());
    }

    #[test]
    fn arming_an_engine_1_bundle_overheat_moves_the_variable_its_own_ecam_trigger_reads() {
        board::clear();
        let mut live = WiringLive::new();
        let truth = Truth::default();
        let quiet = run(&mut live, &truth, &Faults::default(), 2);
        assert_eq!(quiet["WIRING_ZONE_ENGINE_1_OVERHEAT_SEVERITY"], 0.0);

        let id = failure_id_for(Zone::Engine(1), "overheat_damage");
        let hot = run(&mut live, &truth, &Faults::from_pairs([(id, 0.9)]), 2);
        assert!(hot["WIRING_ZONE_ENGINE_1_OVERHEAT_SEVERITY"] > 0.5, "the ENGINE_1 WIRE OVHT alert can only ever fire above 0.5");
        assert_eq!(hot["WIRING_ZONE_ENGINE_2_OVERHEAT_SEVERITY"], 0.0, "a zone fault must stay in its own zone");
        board::clear();
    }

    #[test]
    fn a_full_severity_avionics_bay_fire_opens_the_loads_routed_through_it() {
        use crate::deep::electrical::live::ElectricalLive;
        board::clear();
        let truth = Truth { dt_s: 1.0 / 30.0, on_ground: false, engine_n1_frac: [0.9; 4], engine_n2_frac: [0.9; 4], engine_n3_frac: [0.9; 4], engine_running: [true; 4], ..Truth::default() };

        let mut elec = ElectricalLive::new();
        let mut wire = WiringLive::new();
        let healthy = Faults::default();
        let mut published = BTreeMap::new();
        for _ in 0..20 {
            elec.tick(&truth, &healthy);
            wire.tick(&truth, &healthy);
            published.clear();
            elec.publish(&mut |n, v| {
                published.insert(n.to_string(), v);
            });
            wire.publish(&mut |n, v| {
                published.insert(n.to_string(), v);
            });
        }
        assert_eq!(published["ELEC_LOAD_prim-1_POWERED"], 1.0);

        let id = failure_id_for(Zone::MainAvionics, "overheat_damage");
        let burning = Faults::from_pairs([(id, 1.0)]);
        for _ in 0..20 {
            elec.tick(&truth, &burning);
            wire.tick(&truth, &burning);
            published.clear();
            elec.publish(&mut |n, v| {
                published.insert(n.to_string(), v);
            });
            wire.publish(&mut |n, v| {
                published.insert(n.to_string(), v);
            });
        }
        assert_eq!(published["WIRING_ZONE_MAIN_AVIONICS_OVERHEAT_SEVERITY"], 1.0);
        assert_eq!(published["ELEC_LOAD_prim-1_POWERED"], 0.0, "a burnt-through conductor cannot carry its load's current");
        board::clear();
    }

    #[test]
    fn a_deep_chafe_puts_real_arc_current_and_heat_onto_the_circuit_it_damages() {
        use crate::deep::electrical::live::ElectricalLive;
        board::clear();
        let truth = Truth { dt_s: 1.0 / 30.0, on_ground: false, engine_n1_frac: [0.9; 4], engine_n2_frac: [0.9; 4], engine_n3_frac: [0.9; 4], engine_running: [true; 4], ..Truth::default() };
        let mut elec = ElectricalLive::new();
        let mut wire = WiringLive::new();

        let id = failure_id_for(Zone::MainAvionics, "chafe_wear");
        let chafed = Faults::from_pairs([(id, 0.95)]);
        let mut published = BTreeMap::new();
        for _ in 0..20 {
            elec.tick(&truth, &chafed);
            wire.tick(&truth, &chafed);
            published.clear();
            elec.publish(&mut |n, v| {
                published.insert(n.to_string(), v);
            });
            wire.publish(&mut |n, v| {
                published.insert(n.to_string(), v);
            });
        }
        let i = published["WIRING_CIRCUIT_egpwc_ARC_CURRENT_A"];
        assert!(i > 0.0, "a 95% chafe on a live 115 V circuit must draw a real arc current, got {i} A");
        assert!(published["WIRING_CIRCUIT_egpwc_ARC_HEAT_W"] > 0.0);
        assert_eq!(published["WIRING_CIRCUIT_prim-1_ARC_CURRENT_A"], 0.0, "the same chafe on a 28 V circuit cannot sustain a 30 V arc");
        assert!(published["WIRING_ZONE_MAIN_AVIONICS_ARC_HEAT_W"] > 0.0);
        assert!(published["WIRING_TOTAL_ARC_HEAT_W"] > 0.0);
        board::clear();
    }

    #[test]
    fn it_plugs_into_deep_and_publishes_the_variable_every_one_of_its_alerts_triggers_on() {
        board::clear();
        let mut deep = Deep::new().with_area(live_system());
        let mut published = BTreeMap::new();
        deep.tick(Truth::default(), &Faults::default(), &mut |n, v| {
            published.insert(n.to_string(), v);
        });
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        assert_eq!(reg.alerts.len(), 13);
        for zone in &live_zones() {
            let name = format!("WIRING_ZONE_{}_OVERHEAT_SEVERITY", zone.name());
            assert!(published.contains_key(&name), "{name} is read by an ECAM trigger but nobody publishes it");
        }
        board::clear();
    }

    fn live_zones() -> Vec<Zone> {
        WiringLive::new().zones
    }

    #[test]
    fn nothing_produces_a_nan_at_rest_or_at_zero_dt() {
        board::clear();
        let mut live = WiringLive::new();
        let truth = Truth { dt_s: 0.0, ..Truth::default() };
        let published = run(&mut live, &truth, &Faults::from_pairs([(failure_id_for(Zone::Apu, "water_ingress"), 1.0)]), 3);
        for (name, v) in &published {
            assert!(v.is_finite(), "{name} is {v}");
        }
        board::clear();
    }
}
