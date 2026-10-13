//! Wiring faults, each mapped to a per-circuit effect: chafe (an arcing
//! short between two specific circuits, or to structure), bundle
//! overheat/fire damage in a zone (damages every circuit passing through),
//! connector corrosion/high resistance, water ingress, rodent/maintenance
//! damage, and a plain open wire.
//!
//! Every fault carries a continuous severity `0.0 = healthy .. 1.0 = fully
//! failed` (this push's convention). The mapping from severity to
//! [`CircuitEffect`] is a real, continuous physical relationship in each
//! case (cited per function below), not a lookup table of scripted
//! symptoms.

use super::bundle::{Segment, WireBundleNetwork};
use super::gauge::Awg;
use super::zones::Zone;

/// What a fault does to one circuit's own conductor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CircuitEffect {
    /// Full discontinuity: the circuit carries no current at all.
    Open,
    /// A low-impedance path to airframe structure (return/ground).
    ShortToStructure,
    /// Extra series/leakage resistance introduced by the fault, ohms
    /// (added on top of the wire's own healthy resistance).
    HighResistance(f64),
    /// Shorted to a specific other circuit sharing the same segment.
    CrosstalkShort { with: &'static str },
}

/// The seven fault mechanisms this module models.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FaultKind {
    /// Insulation worn through by rubbing against structure or another
    /// bundle at one point.
    Chafe,
    /// Localized heat/fire damage across a whole zone.
    BundleOverheat,
    /// Oxide film raising a connector pin's contact resistance.
    ConnectorCorrosion,
    /// Moisture ingress providing a leakage path.
    WaterIngress,
    /// Insulation/conductor damage from rodent activity.
    RodentDamage,
    /// Mechanical damage during maintenance (pinch, crush, cut).
    MaintenanceDamage,
    /// A plain mechanical/fatigue break developing through the conductor.
    OpenWire,
}

/// Typical series-arc voltage drop, V: publicly reported arc-fault
/// literature (e.g. Underwriters Laboratories UL 1699 arc-fault-circuit-
/// interrupter test characterization, and IEEE papers on series arcing
/// faults in aircraft wiring) finds a series arc's own voltage drop stays
/// roughly constant across a wide current range, commonly cited in the
/// 25-40 V band; 30 V is the representative midpoint used here. This
/// constancy (not a fixed resistance) is exactly why an arcing fault's
/// effective resistance falls as the fault deepens and more current flows
/// -- `V_arc` stays put, `R_eff = V_arc / I` falls -- reproduced below.
pub const ARC_VOLTAGE_DROP_V: f64 = 30.0;

/// A chafe's effect at severity `magnitude` on `circuit`, whose own rated
/// current is `rated_current_a` (used to size the arc's own effective
/// resistance once contact is metallic) and which shares its segment with
/// `other` (`Some` picks a specific neighbour for a wire-to-wire short,
/// `None` means the exposed conductor contacts bare structure instead).
/// Below full severity, chafing has broken through *some* insulation but
/// the contact is intermittent/high-impedance (arcing, not yet a bolted
/// fault) -- the contact resistance falls as the insulation wears through,
/// approaching the arc's own characteristic `V_arc/I` floor as `magnitude`
/// approaches 1. At `magnitude >= 1` the fault is a bolted short (structure
/// or the neighbour), the real end state of a chafe that has fully
/// breached both conductors.
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
    // Contact resistance scales down from "no contact" toward the arc
    // floor as m -> 1: R(m) = floor / m, an intermediate high-resistance
    // arcing contact for any partial chafe.
    Some(CircuitEffect::HighResistance(arc_floor_ohm / m))
}

/// Corroded/oxidized connector pin contact resistance, ohms. Oxide-film
/// contact resistance rising with corrosion progress is a well-documented
/// aerospace-connector maintenance phenomenon (FAA AC 43.13-1B's own
/// connector/terminal maintenance guidance discusses cleaning corroded
/// contacts specifically because oxide films raise contact resistance);
/// the exact magnitude-vs-ohms curve has no public figure, so the ceiling
/// (100 ohm at `magnitude == 1`, GENERIC: an order of magnitude above a
/// healthy gold/tin-plated contact's milliohm-class resistance, enough to
/// matter to a low-current avionics load without being an open circuit)
/// scales linearly with severity -- corrosion is a gradual, continuous
/// process, not a threshold effect.
pub fn connector_corrosion_effect(magnitude: f64) -> Option<CircuitEffect> {
    let m = magnitude.clamp(0.0, 1.0);
    if m <= 0.0 {
        None
    } else {
        Some(CircuitEffect::HighResistance(100.0 * m))
    }
}

/// Water-ingress leakage resistance, ohms. Pure water is a poor conductor;
/// real leakage paths in a contaminated connector or splice (galley/lav
/// water, de-icing fluid, condensation carrying airframe/cleaning residue)
/// conduct through dissolved ionic contamination, whose resistivity has no
/// single public figure -- the 2 kOhm ceiling at `magnitude == 1` is
/// GENERIC, chosen as a plausible contaminated-water leakage path (orders
/// of magnitude below a dry connector's effectively infinite leakage
/// resistance, orders of magnitude above a metallic short), falling
/// further as ingress worsens (more contamination, a fuller path).
pub fn water_ingress_effect(magnitude: f64) -> Option<CircuitEffect> {
    let m = magnitude.clamp(0.0, 1.0);
    if m <= 0.0 {
        None
    } else {
        Some(CircuitEffect::HighResistance(2_000.0 / m))
    }
}

/// Rodent damage: a small animal chewing through insulation typically
/// strips it before it can sever a heavier conductor, so a thin signal
/// wire (`AWG` 18 or smaller-diameter, i.e. numerically 18 or above) is
/// modelled as eventually bitten fully through (`Open` at full severity),
/// while a heavier power feeder more plausibly ends up with bare,
/// exposed conductor contacting structure (`ShortToStructure`) -- the same
/// distinction FAA wildlife/rodent wiring-damage guidance and maintenance
/// literature draws between "chewed through" fine wire and "insulation
/// stripped" heavy cable. Below full severity, treated as a developing
/// chafe-style contact (reuses [`chafe_effect`]'s own curve) since the
/// physical mechanism -- insulation progressively worn away -- is the
/// same until the conductor itself finally gives way.
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

/// Maintenance damage: a pinch, crush or slip of a tool during servicing.
/// Modelled as tending toward a short (crushed against structure, or
/// against a neighbour if the segment bundles more than one circuit)
/// rather than an open -- a crush event deforms and shorts far more often
/// in practice than it cleanly severs a conductor (standard wiring-
/// maintenance-damage description, e.g. FAA AC 43.13-1B's wiring
/// installation/damage-avoidance guidance).
pub fn maintenance_damage_effect(magnitude: f64, rated_current_a: f64, other: Option<&'static str>) -> Option<CircuitEffect> {
    chafe_effect(magnitude, rated_current_a, other)
}

/// A plain fatigue/mechanical crack propagating through a solid conductor's
/// cross-section: remaining conductive area shrinks as `(1 - magnitude)`,
/// and resistance is inversely proportional to area (textbook `R = rho*L/A`),
/// so resistance scales as `1/(1-magnitude)` relative to the healthy value
/// until the crack fully separates the conductor (`Open` at `magnitude >= 1`).
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

/// GENERIC representative temperature a fully severe (`magnitude == 1.0`)
/// localized bundle overheat/fire event reaches, deg C: well above any
/// insulation's continuous rating (see `gauge::Insulation`) but below an
/// open-flame certification test temperature (the 14 CFR/FAA flame test
/// for wire insulation applies roughly a 1,100 C flame, a public figure
/// from AC 20-135/AC 25-16 flammability testing) -- representing a serious
/// electrical overheat/smouldering event, not an open flame.
pub const BUNDLE_FIRE_MAX_TEMP_C: f64 = 400.0;

/// Zone-wide overheat/fire damage: every circuit in every segment whose
/// zone is `zone` is checked against its *own* insulation's temperature
/// margin -- higher-rated insulation (`Ptfe260`) only starts taking damage
/// once the fire's own temperature exceeds 260 C, needing a more severe
/// event than `Etfe150` insulation right next to it in the very same
/// bundle, a real, continuous difference this function derives rather than
/// assumes. Below half of a circuit's own damage fraction the developing
/// char/tracking is modelled as a high-resistance leakage path (charred
/// insulation is measurably, if poorly, conductive -- textbook carbon-
/// tracking behaviour); at or above half, tracking risk is modelled as a
/// crosstalk short to another circuit sharing the same segment where one
/// exists (arcing between adjacent conductors through a carbonized path is
/// the real, documented aircraft-wiring-fire propagation risk, e.g. FAA AC
/// 25-16 "Electrical Fault and Fire Prevention and Protection"); full
/// damage (`>= 1.0`) is the conductor destroyed (`Open`).
pub fn zone_overheat_effects(net: &WireBundleNetwork, zone: Zone, magnitude: f64) -> Vec<(&'static str, CircuitEffect)> {
    let severity = magnitude.clamp(0.0, 1.0);
    if severity <= 0.0 {
        return Vec::new();
    }
    // Fire temperature scales linearly with severity (0 C rise at magnitude
    // 0, `BUNDLE_FIRE_MAX_TEMP_C` at full severity) -- a plain, continuous
    // severity-to-temperature map, not a scripted threshold.
    let fire_temp_c = BUNDLE_FIRE_MAX_TEMP_C * severity;
    let mut out = Vec::new();
    for seg in net.segments_in_zone(zone) {
        out.extend(segment_overheat_effects(seg, fire_temp_c));
    }
    out
}

fn segment_overheat_effects(seg: &Segment, fire_temp_c: f64) -> Vec<(&'static str, CircuitEffect)> {
    let mut out = Vec::new();
    for cw in &seg.circuits {
        let max_c = cw.wire.insulation.max_temp_c();
        if fire_temp_c <= max_c {
            continue; // this circuit's insulation class survives this event
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
        // Build a two-circuit segment by hand: one Etfe150 (150 C), one
        // Ptfe260 (260 C), in the same bundle -- exactly the "insulation
        // temperature rating" differentiation the backlog asks for.
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

        // Severity 0.5 -> fire_temp_c = 200 C: exceeds 150 C (low-rated
        // damaged) but not 260 C (high-rated untouched).
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
}
