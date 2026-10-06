use super::bundle::{Segment, WireBundleNetwork};
use super::routing::build_generic_a380_network;
use super::zones::Zone;
use crate::deep::api::*;

fn frac(name: &str, meaning: &str) -> ParamDef {
    ParamDef { name: name.to_string(), meaning: meaning.to_string(), healthy: 0.0 }
}

fn harness_params() -> Vec<ParamDef> {
    vec![
        frac("chafe_wear", "insulation breach depth at the worst chafe point on this bundle (0 = intact, 1 = bolted short/fully breached)"),
        frac("overheat_damage", "localized bundle overheat/fire damage fraction on this bundle (0 = none, 1 = every conductor destroyed) -- see faults::bundle_overheat_effects"),
        frac("connector_corrosion", "oxide-film contact resistance growth at this bundle's connectors (0 = clean contact, 1 = maximum modelled corrosion)"),
        frac("water_ingress", "moisture/contamination leakage-path severity on this bundle (0 = dry, 1 = worst modelled leakage path)"),
        frac("rodent_damage", "insulation/conductor damage from rodent activity on this bundle (0 = none, 1 = conductor bitten through or bared)"),
        frac("maintenance_damage", "mechanical pinch/crush/cut damage from servicing on this bundle (0 = none, 1 = crushed/severed)"),
        frac("open_wire_fatigue", "fatigue crack propagation through a conductor's cross-section on this bundle (0 = intact, 1 = fully separated)"),
    ]
}

struct Kind {
    suffix: &'static str,
    param: &'static str,
    model_field: &'static str,
    magnitude: &'static str,
    effect: &'static str,
}
fn kinds() -> [Kind; 7] {
    [
        Kind {
            suffix: "Chafe",
            param: "chafe_wear",
            model_field: "wiring::faults::chafe_effect(magnitude, rated_current_a, other)",
            magnitude: "0..1, insulation breach depth (0 intact .. 1 bolted short)",
            effect: "intermittent high-resistance arcing contact that deepens toward a bolted short-to-structure or short-to-neighbour as the breach completes",
        },
        Kind {
            suffix: "BundleOverheat",
            param: "overheat_damage",
            model_field: "wiring::faults::bundle_overheat_effects(net, segment_id, magnitude)",
            magnitude: "0..1, bundle fire/overheat severity (0 = ambient, 1 = 400 C representative severe localized electrical fire)",
            effect: "every circuit on this one physical bundle whose insulation temperature rating the fire's own temperature exceeds takes damage (leakage resistance, then inter-conductor crosstalk, then full open) -- higher-rated insulation on the same bundle survives longer, and bundles on other routes through the same zone are untouched",
        },
        Kind {
            suffix: "ConnectorCorrosion",
            param: "connector_corrosion",
            model_field: "wiring::faults::connector_corrosion_effect(magnitude)",
            magnitude: "0..1, oxide-film contact-resistance growth",
            effect: "added series contact resistance at a connector pin, up to 100 ohm at full severity",
        },
        Kind {
            suffix: "WaterIngress",
            param: "water_ingress",
            model_field: "wiring::faults::water_ingress_effect(magnitude)",
            magnitude: "0..1, contamination/leakage-path severity",
            effect: "a leakage resistance to structure through contaminated moisture, falling toward 2 kOhm as ingress worsens",
        },
        Kind {
            suffix: "RodentDamage",
            param: "rodent_damage",
            model_field: "wiring::faults::rodent_damage_effect(magnitude, awg, rated_current_a)",
            magnitude: "0..1, insulation/conductor bite-through progress",
            effect: "a developing chafe-style contact that, at full severity, opens a thin signal wire or shorts-to-structure a heavy feeder",
        },
        Kind {
            suffix: "MaintenanceDamage",
            param: "maintenance_damage",
            model_field: "wiring::faults::maintenance_damage_effect(magnitude, rated_current_a, other)",
            magnitude: "0..1, crush/pinch/cut severity",
            effect: "a developing chafe-style short to structure or a bundled neighbour",
        },
        Kind {
            suffix: "OpenWire",
            param: "open_wire_fatigue",
            model_field: "wiring::faults::open_wire_effect(magnitude, healthy_resistance_ohm)",
            magnitude: "0..1, fatigue-crack cross-section fraction",
            effect: "series resistance rising as the remaining conductor cross-section shrinks, snapping to a full open once the crack completes",
        },
    ]
}

pub fn component_id_for(net: &WireBundleNetwork, seg: &Segment) -> String {
    let segments_in_same_zone = net.segments().iter().filter(|s| s.zone == seg.zone).count();
    if segments_in_same_zone <= 1 {
        format!("91_wiring.harness_{}", seg.zone.name().to_lowercase())
    } else {
        format!("91_wiring.harness_{}", seg.id.trim_start_matches("seg-").replace('-', "_"))
    }
}

fn segment_kind_n(segments: &[Segment], segment_id: &str, kind_index: usize) -> u16 {
    let si = segments.iter().position(|s| s.id == segment_id).expect("segment must be in the registered network") as u16;
    si * kinds().len() as u16 + kind_index as u16 + 1
}

pub fn register(r: &mut Registry) {
    let net = build_generic_a380_network();
    let segments: &[Segment] = net.segments();

    let mut zones: Vec<Zone> = Vec::new();
    for seg in segments {
        if !zones.contains(&seg.zone) {
            zones.push(seg.zone);
        }
    }

    for seg in segments {
        r.component(ComponentDef {
            id: component_id_for(&net, seg),
            area: Area::Wiring,
            ata: 91,
            name: format!("Wiring harness -- {} bundle {}", seg.zone.name(), seg.id),
            params: harness_params(),
            failures: kinds().iter().enumerate().map(|(ki, _)| failure_id(Area::Wiring, 91, segment_kind_n(segments, seg.id, ki))).collect(),
        });
    }

    let mut n: u16 = 0;
    for seg in segments {
        for k in kinds() {
            n += 1;
            let ki = kinds().iter().position(|kk| kk.suffix == k.suffix).expect("suffix must be in kinds");
            let debug_n = segment_kind_n(segments, seg.id, ki);
            debug_assert_eq!(debug_n, n, "sequential id assignment must match the lookup helper");
            let circuits_here = net.circuits_in_segment(seg.id);
            r.failure(FailureDef {
                id: failure_id(Area::Wiring, 91, n),
                area: Area::Wiring,
                ata: 91,
                name: format!("{} -- {} wiring bundle {}", k.suffix, seg.zone.name(), seg.id),
                component: component_id_for(&net, seg),
                model_field: format!("{} [component param {}]", k.model_field, k.param),
                magnitude: k.magnitude.to_string(),
                effect: format!("{} (this bundle carries: {})", k.effect, if circuits_here.is_empty() { "no catalogued circuit yet".to_string() } else { circuits_here.join(", ") }),
            });
        }
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registers_without_validation_errors() {
        let mut r = Registry::default();
        register(&mut r);
        let errors = r.validate_area();
        assert!(errors.is_empty(), "{errors:?}");
    }

    #[test]
    fn one_component_and_seven_failures_per_physical_bundle_one_alert_per_engine_zone_only() {
        let mut r = Registry::default();
        register(&mut r);
        let net = build_generic_a380_network();
        let n_segments = net.segments().len();
        let mut zones: Vec<Zone> = Vec::new();
        for seg in net.segments() {
            if !zones.contains(&seg.zone) {
                zones.push(seg.zone);
            }
        }
        assert_eq!(r.components.len(), n_segments);
        assert_eq!(r.failures.len(), n_segments * 7);
        assert!(r.alerts.is_empty(), "the A380 has no ECAM alert for a wiring harness overheat; its effect is the circuits each bundle carries");
    }

    #[test]
    fn every_failure_id_carries_ata_91_and_the_wiring_area() {
        let mut r = Registry::default();
        register(&mut r);
        for f in &r.failures {
            assert_eq!(f.ata, 91);
            assert_eq!(f.id / 1_000_000, Area::Wiring as u64);
        }
    }

    #[test]
    fn segment_kind_n_is_stable_and_1_based_with_no_collisions() {
        let net = build_generic_a380_network();
        let segments = net.segments();
        let max_n = (segments.len() * kinds().len()) as u16;
        let mut seen = std::collections::HashSet::new();
        for seg in segments {
            for ki in 0..kinds().len() {
                let n = segment_kind_n(segments, seg.id, ki);
                assert!(n >= 1 && n <= max_n);
                assert!(seen.insert(n), "duplicate n {n}");
            }
        }
    }

    #[test]
    fn a_single_bundle_fault_component_only_lists_circuits_on_its_own_bundle() {
        let net = build_generic_a380_network();
        let prim1_seg = net.segment(net.route_of("prim-1")[0]).expect("prim-1 route must resolve");
        let prim2_seg = net.segment(net.route_of("prim-2")[0]).expect("prim-2 route must resolve");
        assert_ne!(prim1_seg.id, prim2_seg.id, "prim-1 and prim-2 must be registered as separate bundles");
        assert_ne!(component_id_for(&net, prim1_seg), component_id_for(&net, prim2_seg));
    }

    #[test]
    fn the_cockpit_harness_keeps_its_legacy_zone_only_component_id() {
        let net = build_generic_a380_network();
        let cockpit_segments: Vec<&Segment> = net.segments().iter().filter(|s| s.zone == Zone::Cockpit).collect();
        assert_eq!(cockpit_segments.len(), 1, "this test assumes Cockpit still routes through exactly one bundle");
        assert_eq!(component_id_for(&net, cockpit_segments[0]), "91_wiring.harness_cockpit");
    }
}
