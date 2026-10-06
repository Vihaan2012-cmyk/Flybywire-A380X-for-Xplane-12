use super::bundle::{Segment, WireBundleNetwork};
use super::gauge::Awg;
use super::zones::Zone;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CircuitEffect {
    Open,
    ShortToStructure,
    HighResistance(f64),
    CrosstalkShort { with: &'static str },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FaultKind {
    Chafe,
    BundleOverheat,
    ConnectorCorrosion,
    WaterIngress,
    RodentDamage,
    MaintenanceDamage,
    OpenWire,
}

pub const ARC_VOLTAGE_DROP_V: f64 = 30.0;

pub fn chafe_effect(magnitude: f64, rated_current_a: f64, other: Option<&'static str>) -> Option<CircuitEffect> {
    let m = magnitude.clamp(0.0, 1.0);
    if m <= 0.0 {
        return None;
    }
    if m >= 1.0 {
        return Some(match other {
            Some(o) => CircuitEffect::CrosstalkShort { with: o },
            None => CircuitEffect::ShortToStructure,
        });
    }
    let arc_floor_ohm = ARC_VOLTAGE_DROP_V / rated_current_a.max(0.01);
    Some(CircuitEffect::HighResistance(arc_floor_ohm / m))
}

pub fn connector_corrosion_effect(magnitude: f64) -> Option<CircuitEffect> {
    let m = magnitude.clamp(0.0, 1.0);
    if m <= 0.0 {
        None
    } else {
        Some(CircuitEffect::HighResistance(100.0 * m))
    }
}

pub fn water_ingress_effect(magnitude: f64) -> Option<CircuitEffect> {
    let m = magnitude.clamp(0.0, 1.0);
    if m <= 0.0 {
        None
    } else {
        Some(CircuitEffect::HighResistance(2_000.0 / m))
    }
}

pub fn rodent_damage_effect(magnitude: f64, awg: Awg, rated_current_a: f64) -> Option<CircuitEffect> {
    let m = magnitude.clamp(0.0, 1.0);
    if m <= 0.0 {
        return None;
    }
    if m >= 1.0 {
        let thin = matches!(awg, Awg::Size(n) if n >= 18);
        return Some(if thin { CircuitEffect::Open } else { CircuitEffect::ShortToStructure });
    }
    chafe_effect(m, rated_current_a, None)
}

pub fn maintenance_damage_effect(magnitude: f64, rated_current_a: f64, other: Option<&'static str>) -> Option<CircuitEffect> {
    chafe_effect(magnitude, rated_current_a, other)
}

pub fn open_wire_effect(magnitude: f64, healthy_resistance_ohm: f64) -> Option<CircuitEffect> {
    let m = magnitude.clamp(0.0, 1.0);
    if m <= 0.0 {
        None
    } else if m >= 0.999 {
        Some(CircuitEffect::Open)
    } else {
        Some(CircuitEffect::HighResistance(healthy_resistance_ohm * (1.0 / (1.0 - m) - 1.0)))
    }
}

pub const BUNDLE_FIRE_MAX_TEMP_C: f64 = 400.0;

pub fn zone_overheat_effects(net: &WireBundleNetwork, zone: Zone, magnitude: f64) -> Vec<(&'static str, CircuitEffect)> {
    let severity = magnitude.clamp(0.0, 1.0);
    if severity <= 0.0 {
        return Vec::new();
    }
    let fire_temp_c = BUNDLE_FIRE_MAX_TEMP_C * severity;
    let mut out = Vec::new();
    for seg in net.segments_in_zone(zone) {
        out.extend(segment_overheat_effects(seg, fire_temp_c));
    }
    out
}

pub fn bundle_overheat_effects(net: &WireBundleNetwork, segment_id: &str, magnitude: f64) -> Vec<(&'static str, CircuitEffect)> {
    let severity = magnitude.clamp(0.0, 1.0);
    if severity <= 0.0 {
        return Vec::new();
    }
    let fire_temp_c = BUNDLE_FIRE_MAX_TEMP_C * severity;
    match net.segment(segment_id) {
        Some(seg) => segment_overheat_effects(seg, fire_temp_c),
        None => Vec::new(),
    }
}

fn segment_overheat_effects(seg: &Segment, fire_temp_c: f64) -> Vec<(&'static str, CircuitEffect)> {
    let mut out = Vec::new();
    for cw in &seg.circuits {
        let max_c = cw.wire.insulation.max_temp_c();
        if fire_temp_c <= max_c {
            continue;
        }
        let damage = ((fire_temp_c - max_c) / (BUNDLE_FIRE_MAX_TEMP_C - max_c).max(1.0)).clamp(0.0, 1.0);
        let effect = if damage >= 1.0 {
            CircuitEffect::Open
        } else if damage >= 0.5 {
            match seg.circuits.iter().map(|c| c.circuit).find(|&id| id != cw.circuit) {
                Some(other) => CircuitEffect::CrosstalkShort { with: other },
                None => CircuitEffect::HighResistance(1_000.0 * (1.0 - damage).max(0.01)),
            }
        } else {
            CircuitEffect::HighResistance(1_000.0 * (1.0 - damage))
        };
        out.push((cw.circuit, effect));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::super::bundle::{CircuitWire, Segment};
    use super::super::gauge::{Insulation, WireSpec};
    use super::super::routing::build_generic_a380_network;
    use super::*;

    #[test]
    fn a_healthy_chafe_produces_no_effect() {
        assert_eq!(chafe_effect(0.0, 5.0, None), None);
    }

    #[test]
    fn chafe_resistance_falls_toward_the_arc_floor_as_severity_rises() {
        let low = match chafe_effect(0.1, 5.0, None).unwrap() {
            CircuitEffect::HighResistance(r) => r,
            _ => panic!("expected HighResistance"),
        };
        let high = match chafe_effect(0.9, 5.0, None).unwrap() {
            CircuitEffect::HighResistance(r) => r,
            _ => panic!("expected HighResistance"),
        };
        assert!(high < low, "deeper chafe {high} must resist less than shallow chafe {low}");
        assert!(high > ARC_VOLTAGE_DROP_V / 5.0 - 1e-9, "must not undercut the arc's own floor resistance");
    }

    #[test]
    fn a_full_severity_chafe_with_a_neighbour_shorts_to_that_neighbour_not_structure() {
        assert_eq!(chafe_effect(1.0, 5.0, Some("other-circuit")), Some(CircuitEffect::CrosstalkShort { with: "other-circuit" }));
        assert_eq!(chafe_effect(1.0, 5.0, None), Some(CircuitEffect::ShortToStructure));
    }

    #[test]
    fn rodent_damage_opens_thin_wire_but_shorts_heavy_feeder_at_full_severity() {
        let thin = rodent_damage_effect(1.0, Awg::Size(22), 2.0);
        let heavy = rodent_damage_effect(1.0, Awg::Aught(1), 200.0);
        assert_eq!(thin, Some(CircuitEffect::Open));
        assert_eq!(heavy, Some(CircuitEffect::ShortToStructure));
    }

    #[test]
    fn open_wire_resistance_diverges_as_the_crack_completes_then_snaps_open() {
        let r0 = 1.0;
        let mild = open_wire_effect(0.5, r0).unwrap();
        let severe = open_wire_effect(0.99, r0).unwrap();
        let full = open_wire_effect(1.0, r0).unwrap();
        let extract = |e: CircuitEffect| match e {
            CircuitEffect::HighResistance(r) => r,
            _ => panic!(),
        };
        assert!(extract(severe) > extract(mild));
        assert_eq!(full, CircuitEffect::Open);
    }

    #[test]
    fn higher_rated_insulation_survives_a_fire_that_opens_a_lower_rated_neighbour() {
        let mut net = super::super::bundle::WireBundleNetwork::new();
        net.add_segment(Segment {
            id: "mixed",
            zone: Zone::Engine(1),
            length_m: 5.0,
            circuits: vec![
                CircuitWire { circuit: "low-rated", wire: WireSpec { awg: Awg::Size(20), insulation: Insulation::Etfe150 } },
                CircuitWire { circuit: "high-rated", wire: WireSpec { awg: Awg::Size(20), insulation: Insulation::Ptfe260 } },
            ],
        });
        net.set_route("low-rated", vec!["mixed"]);
        net.set_route("high-rated", vec!["mixed"]);

        let effects = zone_overheat_effects(&net, Zone::Engine(1), 0.5);
        let low = effects.iter().find(|(id, _)| *id == "low-rated");
        let high = effects.iter().find(|(id, _)| *id == "high-rated");
        assert!(low.is_some(), "the 150 C circuit must be damaged by a 200 C event");
        assert!(high.is_none(), "the 260 C circuit must survive the same 200 C event");
    }

    #[test]
    fn a_full_severity_zone_fire_opens_every_circuit_in_every_segment_of_that_zone_including_both_fire_loops() {
        let net = build_generic_a380_network();
        let effects = zone_overheat_effects(&net, Zone::Engine(1), 1.0);
        let ids: Vec<&str> = effects.iter().map(|(id, _)| *id).collect();
        assert!(ids.contains(&"fire-loop-eng-1-a"));
        assert!(ids.contains(&"fire-loop-eng-1-b"), "a zone-wide fire, unlike a segment chafe, must reach both A and B loops");
        assert!(effects.iter().all(|(_, e)| *e == CircuitEffect::Open), "full-severity fire opens every conductor it reaches");
    }

    #[test]
    fn a_zone_with_no_severity_damages_nothing() {
        let net = build_generic_a380_network();
        assert!(zone_overheat_effects(&net, Zone::Engine(1), 0.0).is_empty());
    }

    #[test]
    fn a_bundle_overheat_on_one_fire_loop_segment_never_reaches_its_partner_loop() {
        let net = build_generic_a380_network();
        let seg_a = net.route_of("fire-loop-eng-1-a")[net.route_of("fire-loop-eng-1-a").len() - 1];
        let effects = bundle_overheat_effects(&net, seg_a, 1.0);
        let ids: Vec<&str> = effects.iter().map(|(id, _)| *id).collect();
        assert!(ids.contains(&"fire-loop-eng-1-a"));
        assert!(!ids.contains(&"fire-loop-eng-1-b"), "a bundle fault, unlike a zone fire, must stay on its own physical route");
    }

    #[test]
    fn a_bundle_overheat_on_prim_1s_own_segment_leaves_prim_2_untouched() {
        let net = build_generic_a380_network();
        let prim1_seg = net.route_of("prim-1")[0];
        let prim2_seg = net.route_of("prim-2")[0];
        assert_ne!(prim1_seg, prim2_seg);
        let effects = bundle_overheat_effects(&net, prim1_seg, 1.0);
        let ids: Vec<&str> = effects.iter().map(|(id, _)| *id).collect();
        assert!(!ids.contains(&"prim-2"), "prim-1's own bundle overheating must not touch prim-2's bundle");
    }
}
