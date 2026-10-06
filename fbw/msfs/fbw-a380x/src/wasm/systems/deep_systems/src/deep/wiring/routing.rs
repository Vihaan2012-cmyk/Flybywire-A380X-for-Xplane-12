use super::bundle::{CircuitWire, Segment, WireBundleNetwork};
use super::gauge::{Awg, Insulation};
use super::zones::Zone;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Side {
    Side1,
    Side2,
    Ess,
    Apu,
    Ground,
}

pub fn side_of_bus(bus: &str) -> Side {
    match bus {
        "AC1" | "AC2" | "DC1" => Side::Side1,
        "AC3" | "AC4" | "DC2" => Side::Side2,
        "AC_ESS" | "AC_ESS_SHED" | "AC_247XP" | "DC_ESS" | "DC_HOT1" | "DC_HOT2" | "DC_HOT3" | "DC_HOT4" | "AC_STAT_INV" | "DC_247PP" => Side::Ess,
        "APU_GEN" | "309PP" | "DC_309PP" => Side::Apu,
        "AC_GND_FLT_SVC" | "DC_GND_FLT_SVC" => Side::Ground,
        _ => Side::Ess,
    }
}

pub fn side_of_engine(n: u8) -> Side {
    if n <= 2 {
        Side::Side1
    } else {
        Side::Side2
    }
}

const LRU_120C_ETFE: WireSpec2 = WireSpec2 { awg: Awg::Size(20), insulation: Insulation::Etfe150 };
const FEEDER_150C: WireSpec2 = WireSpec2 { awg: Awg::Size(4), insulation: Insulation::Ptfe200 };
const HEAVY_FEEDER: WireSpec2 = WireSpec2 { awg: Awg::Aught(1), insulation: Insulation::Ptfe200 };
const SENSOR_260C: WireSpec2 = WireSpec2 { awg: Awg::Size(22), insulation: Insulation::Ptfe260 };
const VALVE_ACTUATOR_WIRE: WireSpec2 = WireSpec2 { awg: Awg::Size(16), insulation: Insulation::Etfe150 };

type WireSpec2 = super::gauge::WireSpec;

struct RouteEntry {
    circuit: &'static str,
    bus: &'static str,
    path: &'static [Zone],
    wire: WireSpec2,
}

fn catalogue() -> Vec<RouteEntry> {
    use Zone::*;
    vec![
        RouteEntry { circuit: "tr-1", bus: "AC1", path: &[MainAvionics], wire: FEEDER_150C },
        RouteEntry { circuit: "tr-2", bus: "AC2", path: &[MainAvionics], wire: FEEDER_150C },
        RouteEntry { circuit: "tr-ess", bus: "AC_ESS", path: &[MainAvionics], wire: FEEDER_150C },
        RouteEntry { circuit: "tr-apu", bus: "309PP", path: &[MainAvionics], wire: FEEDER_150C },
        RouteEntry { circuit: "static-inv", bus: "AC_STAT_INV", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "gen-1", bus: "AC1", path: &[Engine(1), WingRoot, MainAvionics], wire: HEAVY_FEEDER },
        RouteEntry { circuit: "gen-2", bus: "AC2", path: &[Engine(2), WingRoot, MainAvionics], wire: HEAVY_FEEDER },
        RouteEntry { circuit: "gen-3", bus: "AC3", path: &[Engine(3), WingRoot, MainAvionics], wire: HEAVY_FEEDER },
        RouteEntry { circuit: "gen-4", bus: "AC4", path: &[Engine(4), WingRoot, MainAvionics], wire: HEAVY_FEEDER },
        RouteEntry { circuit: "apu-gen-1", bus: "APU_GEN", path: &[Apu, TailCone, WingRoot, MainAvionics], wire: HEAVY_FEEDER },
        RouteEntry { circuit: "apu-gen-2", bus: "APU_GEN", path: &[Apu, TailCone, WingRoot, MainAvionics], wire: HEAVY_FEEDER },
        RouteEntry { circuit: "lgciu-1", bus: "DC_ESS", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "lgciu-2", bus: "DC2", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "prox-uplock-gear-nose-1", bus: "DC_ESS", path: &[MainAvionics, WingRoot, NoseGearBay], wire: SENSOR_260C },
        RouteEntry { circuit: "prox-downlock-gear-nose-2", bus: "DC_ESS", path: &[MainAvionics, WingRoot, NoseGearBay], wire: SENSOR_260C },
        RouteEntry { circuit: "prox-uplock-gear-left-1", bus: "DC_ESS", path: &[MainAvionics, WingRoot, MainGearBay], wire: SENSOR_260C },
        RouteEntry { circuit: "prox-downlock-gear-left-2", bus: "DC_ESS", path: &[MainAvionics, WingRoot, MainGearBay], wire: SENSOR_260C },
        RouteEntry { circuit: "prox-uplock-gear-right-1", bus: "DC_ESS", path: &[MainAvionics, WingRoot, MainGearBay], wire: SENSOR_260C },
        RouteEntry { circuit: "prox-downlock-gear-right-2", bus: "DC_ESS", path: &[MainAvionics, WingRoot, MainGearBay], wire: SENSOR_260C },
        RouteEntry { circuit: "gear-actuator-nose", bus: "DC_ESS", path: &[MainAvionics, WingRoot, NoseGearBay], wire: FEEDER_150C },
        RouteEntry { circuit: "gear-actuator-left", bus: "DC_ESS", path: &[MainAvionics, WingRoot, MainGearBay], wire: FEEDER_150C },
        RouteEntry { circuit: "gear-actuator-right", bus: "DC_ESS", path: &[MainAvionics, WingRoot, MainGearBay], wire: FEEDER_150C },
        RouteEntry { circuit: "hyd-epump-ga", bus: "AC2", path: &[MainAvionics, WingRoot], wire: FEEDER_150C },
        RouteEntry { circuit: "hyd-epump-gb", bus: "AC3", path: &[MainAvionics, WingRoot], wire: FEEDER_150C },
        RouteEntry { circuit: "hyd-epump-ya", bus: "AC4", path: &[MainAvionics, WingRoot], wire: FEEDER_150C },
        RouteEntry { circuit: "hyd-epump-yb", bus: "AC1", path: &[MainAvionics, WingRoot], wire: FEEDER_150C },
        RouteEntry { circuit: "fire-loop-eng-1-a", bus: "DC_ESS", path: &[MainAvionics, WingRoot, Engine(1)], wire: SENSOR_260C },
        RouteEntry { circuit: "fire-loop-eng-1-b", bus: "DC_ESS", path: &[MainAvionics, WingRoot, Engine(1)], wire: SENSOR_260C },
        RouteEntry { circuit: "fire-loop-eng-2-a", bus: "DC_ESS", path: &[MainAvionics, WingRoot, Engine(2)], wire: SENSOR_260C },
        RouteEntry { circuit: "fire-loop-eng-2-b", bus: "DC_ESS", path: &[MainAvionics, WingRoot, Engine(2)], wire: SENSOR_260C },
        RouteEntry { circuit: "fire-loop-apu-a", bus: "DC_ESS", path: &[MainAvionics, TailCone, Apu], wire: SENSOR_260C },
        RouteEntry { circuit: "fire-loop-apu-b", bus: "DC_ESS", path: &[MainAvionics, TailCone, Apu], wire: SENSOR_260C },
        RouteEntry { circuit: "fire-loop-mlg-bay-a", bus: "DC_ESS", path: &[MainAvionics, WingRoot, MainGearBay], wire: SENSOR_260C },
        RouteEntry { circuit: "fire-loop-mlg-bay-b", bus: "DC_ESS", path: &[MainAvionics, WingRoot, MainGearBay], wire: SENSOR_260C },
        RouteEntry { circuit: "ra-sys-a", bus: "AC1", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "ra-sys-b", bus: "AC2", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "ra-sys-c", bus: "AC_ESS", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "egpwc", bus: "AC_ESS", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "prim-1", bus: "DC_ESS", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "prim-2", bus: "DC2", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "prim-3", bus: "DC_ESS", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "sec-1", bus: "DC2", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "sec-2", bus: "DC_ESS", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "sec-3", bus: "DC2", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "fcdc-1", bus: "DC_ESS", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "fcdc-2", bus: "DC2", path: &[MainAvionics], wire: LRU_120C_ETFE },
        RouteEntry { circuit: "bleed-eng-1", bus: "DC1", path: &[MainAvionics, WingRoot, Engine(1)], wire: VALVE_ACTUATOR_WIRE },
        RouteEntry { circuit: "bleed-eng-2", bus: "DC1", path: &[MainAvionics, WingRoot, Engine(2)], wire: VALVE_ACTUATOR_WIRE },
        RouteEntry { circuit: "bleed-eng-3", bus: "DC2", path: &[MainAvionics, WingRoot, Engine(3)], wire: VALVE_ACTUATOR_WIRE },
        RouteEntry { circuit: "bleed-eng-4", bus: "DC2", path: &[MainAvionics, WingRoot, Engine(4)], wire: VALVE_ACTUATOR_WIRE },
        RouteEntry { circuit: "cab-fan-1", bus: "AC1", path: &[MainAvionics, Cockpit], wire: FEEDER_150C },
        RouteEntry { circuit: "cab-fan-2", bus: "AC2", path: &[MainAvionics, Cockpit], wire: FEEDER_150C },
        RouteEntry { circuit: "fwd-isol-valve", bus: "DC1", path: &[MainAvionics, CargoFwd], wire: VALVE_ACTUATOR_WIRE },
        RouteEntry { circuit: "fwd-extract-fan", bus: "AC1", path: &[MainAvionics, CargoFwd], wire: FEEDER_150C },
        RouteEntry { circuit: "bulk-isol-valve", bus: "DC2", path: &[MainAvionics, CargoAft], wire: VALVE_ACTUATOR_WIRE },
        RouteEntry { circuit: "bulk-extract-fan", bus: "AC4", path: &[MainAvionics, CargoAft], wire: FEEDER_150C },
        RouteEntry { circuit: "cargo-heater", bus: "AC2", path: &[MainAvionics, CargoAft], wire: FEEDER_150C },
    ]
}

fn hop_length_m(from: Zone, to: Zone) -> f64 {
    use Zone::*;
    match (from, to) {
        (a, b) if a == b => 3.0,
        (MainAvionics, WingRoot) | (WingRoot, MainAvionics) => 25.0,
        (WingRoot, Engine(_)) | (Engine(_), WingRoot) => 20.0,
        (WingRoot, MainGearBay) | (MainGearBay, WingRoot) => 10.0,
        (MainAvionics, NoseGearBay) | (NoseGearBay, MainAvionics) => 15.0,
        (WingRoot, NoseGearBay) | (NoseGearBay, WingRoot) => 12.0,
        (MainAvionics, TailCone) | (TailCone, MainAvionics) => 45.0,
        (TailCone, Apu) | (Apu, TailCone) => 5.0,
        (MainAvionics, CargoFwd) | (CargoFwd, MainAvionics) => 8.0,
        (MainAvionics, CargoAft) | (CargoAft, MainAvionics) => 30.0,
        (MainAvionics, Cockpit) | (Cockpit, MainAvionics) => 6.0,
        _ => 15.0,
    }
}

pub fn build_generic_a380_network() -> WireBundleNetwork {
    let mut net = WireBundleNetwork::new();
    let entries = catalogue();

    use std::collections::BTreeMap;
    let mut segments: BTreeMap<&'static str, (Zone, f64, Vec<CircuitWire>)> = BTreeMap::new();
    let mut routes: Vec<(&'static str, Vec<&'static str>)> = Vec::new();

    for e in &entries {
        let side = side_of_bus(e.bus);
        let mut seg_ids = Vec::new();
        for &zone in e.path {
            let seg_id = segment_id(zone, side, e.circuit);
            let len = hop_length_m(zone, zone_before(e.path, zone).unwrap_or(zone));
            let entry = segments.entry(seg_id).or_insert_with(|| (zone, len, Vec::new()));
            if !entry.2.iter().any(|c: &CircuitWire| c.circuit == e.circuit) {
                entry.2.push(CircuitWire { circuit: e.circuit, wire: e.wire });
            }
            seg_ids.push(seg_id);
        }
        routes.push((e.circuit, seg_ids));
    }

    for (id, (zone, length_m, circuits)) in segments {
        net.add_segment(Segment { id, zone, length_m, circuits });
    }
    for (circuit, seg_ids) in routes {
        net.set_route(circuit, seg_ids);
    }
    net
}

fn zone_before(path: &[Zone], zone: Zone) -> Option<Zone> {
    let i = path.iter().position(|&z| z == zone)?;
    if i == 0 {
        Some(zone)
    } else {
        Some(path[i - 1])
    }
}

fn redundant_computer_lane(circuit: &'static str) -> Option<u8> {
    match circuit {
        "prim-1" | "sec-1" | "fcdc-1" => Some(1),
        "prim-2" | "sec-2" | "fcdc-2" => Some(2),
        "prim-3" | "sec-3" => Some(3),
        _ => None,
    }
}

fn lane_segment_override(zone: Zone, circuit: &'static str) -> Option<&'static str> {
    let lane = redundant_computer_lane(circuit)?;
    Some(Box::leak(format!("seg-lane{}-{}", lane, zone.name().to_lowercase()).into_boxed_str()))
}

fn segment_id(zone: Zone, side: Side, circuit: &'static str) -> &'static str {
    if let Some(loop_id) = fire_loop_segment_override(zone, circuit) {
        return loop_id;
    }
    if let Some(lane_id) = lane_segment_override(zone, circuit) {
        return lane_id;
    }
    let side_tag = match side {
        Side::Side1 => "side1",
        Side::Side2 => "side2",
        Side::Ess => "ess",
        Side::Apu => "apu",
        Side::Ground => "gnd",
    };
    Box::leak(format!("seg-{}-{}", zone.name().to_lowercase(), side_tag).into_boxed_str())
}

fn fire_loop_segment_override(zone: Zone, circuit: &'static str) -> Option<&'static str> {
    if !circuit.starts_with("fire-loop-") {
        return None;
    }
    let loop_tag = if circuit.ends_with("-a") { "a" } else { "b" };
    Some(Box::leak(format!("seg-fireloop-{}-{}", loop_tag, zone.name().to_lowercase()).into_boxed_str()))
}

pub fn segregation_violations(net: &WireBundleNetwork) -> Vec<String> {
    let mut out = Vec::new();
    for seg in net.segments() {
        let sides: Vec<Side> = seg.circuits.iter().map(|c| circuit_side(c.circuit)).collect();
        let has_side1 = sides.contains(&Side::Side1);
        let has_side2 = sides.contains(&Side::Side2);
        if has_side1 && has_side2 {
            out.push(format!("segment {} mixes Side1 and Side2 circuits: {:?}", seg.id, seg.circuits.iter().map(|c| c.circuit).collect::<Vec<_>>()));
        }
    }
    out
}

pub fn circuit_buses() -> std::collections::HashMap<&'static str, &'static str> {
    catalogue().into_iter().map(|e| (e.circuit, e.bus)).collect()
}

fn circuit_side(circuit: &str) -> Side {
    catalogue().into_iter().find(|e| e.circuit == circuit).map(|e| side_of_bus(e.bus)).unwrap_or(Side::Ess)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn side_of_bus_splits_the_two_normal_generation_channels_and_keeps_ess_apart() {
        assert_eq!(side_of_bus("AC1"), Side::Side1);
        assert_eq!(side_of_bus("AC3"), Side::Side2);
        assert_eq!(side_of_bus("AC_ESS"), Side::Ess);
        assert_eq!(side_of_bus("APU_GEN"), Side::Apu);
        assert_eq!(side_of_bus("nonsense"), Side::Ess, "unknown bus defaults to the conservative Ess assumption");
    }

    #[test]
    fn the_generic_network_builds_with_no_segregation_violation() {
        let net = build_generic_a380_network();
        let violations = segregation_violations(&net);
        assert!(violations.is_empty(), "{violations:?}");
    }

    #[test]
    fn gen_1_and_gen_3_cross_wing_root_in_different_segments() {
        let net = build_generic_a380_network();
        let r1 = net.route_of("gen-1");
        let r3 = net.route_of("gen-3");
        let wr1 = r1.iter().find(|s| s.contains("wing_root")).copied();
        let wr3 = r3.iter().find(|s| s.contains("wing_root")).copied();
        assert!(wr1.is_some() && wr3.is_some());
        assert_ne!(wr1, wr3, "side1 and side2 generator feeders must not share a wing-root segment");
    }

    #[test]
    fn fire_loop_a_and_b_share_a_zone_but_never_a_segment() {
        let net = build_generic_a380_network();
        let a = net.route_of("fire-loop-eng-1-a");
        let b = net.route_of("fire-loop-eng-1-b");
        for (sa, sb) in a.iter().zip(b.iter()) {
            assert_ne!(sa, sb, "loop A and loop B must never land on the same physical bundle");
        }
        assert!(net.segments_for_circuit("fire-loop-eng-1-a").iter().any(|s| s.zone == Zone::Engine(1)));
        assert!(net.segments_for_circuit("fire-loop-eng-1-b").iter().any(|s| s.zone == Zone::Engine(1)));
    }

    #[test]
    fn every_catalogued_circuit_gets_a_nonempty_route() {
        let net = build_generic_a380_network();
        for e in catalogue() {
            assert!(!net.route_of(e.circuit).is_empty(), "{} has no route", e.circuit);
        }
    }

    #[test]
    fn prim_sec_and_fcdc_each_land_on_a_distinct_segment_from_their_own_set() {
        let net = build_generic_a380_network();
        for set in [["prim-1", "prim-2", "prim-3"], ["sec-1", "sec-2", "sec-3"]] {
            let segs: Vec<&str> = set.iter().map(|&c| net.route_of(c)[0]).collect();
            assert_eq!(segs.len(), segs.iter().collect::<std::collections::HashSet<_>>().len(), "{set:?} must not share a bundle: {segs:?}");
        }
        let fcdc_segs: Vec<&str> = ["fcdc-1", "fcdc-2"].iter().map(|&c| net.route_of(c)[0]).collect();
        assert_ne!(fcdc_segs[0], fcdc_segs[1], "fcdc-1 and fcdc-2 must not share a bundle");
    }

    #[test]
    fn cargo_ventilation_routes_cite_their_real_bus() {
        let entries = catalogue();
        let bus_of = |id: &str| entries.iter().find(|e| e.circuit == id).unwrap_or_else(|| panic!("no route entry {id}")).bus;
        assert_eq!(bus_of("fwd-isol-valve"), "DC1");
        assert_eq!(bus_of("bulk-isol-valve"), "DC2");
        assert_eq!(bus_of("fwd-extract-fan"), "AC1");
        assert_eq!(bus_of("bulk-extract-fan"), "AC4");
        assert_eq!(side_of_bus(bus_of("fwd-isol-valve")), side_of_bus(bus_of("fwd-extract-fan")), "the fwd valve and fwd fan should now sit on the same segregation side");
        assert_eq!(side_of_bus(bus_of("bulk-isol-valve")), side_of_bus(bus_of("bulk-extract-fan")), "the bulk valve and bulk fan should now sit on the same segregation side");
    }
}
