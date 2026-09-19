//! Registers `wiring`'s failures, components and ECAM alerts into
//! `crate::deep::api::Registry` (`Area::Wiring`, area code 12). ATA 91 is
//! used throughout ("Wiring Diagrams" -- ATA100/iSpec 2200's own chapter
//! number for wiring diagram manuals across most transport types, a real,
//! standard ATA chapter, not GENERIC).
//!
//! **Model-field/Var note** (same convention `thermal_zones::registry` and
//! `environment::registry` already established for this push's other
//! self-contained engines): `bundle`/`faults`/`routing`/`arc` take every
//! input as a plain argument (an ambient-temperature closure, a magnitude,
//! a duty cycle) and publish nothing themselves. The `WIRING_ZONE_<ZONE>_
//! OVERHEAT_SEVERITY` and `WIRING_SEGMENT_<ID>_FAULT_SEVERITY` variable
//! names an `EcamAlert::trigger` below reads are **not yet published** --
//! whoever integrates `routing::build_generic_a380_network()` into the
//! running simulation (feeding real per-segment fault severities in, and
//! `faults::zone_overheat_effects`/`arc::arc_heat_w`'s outputs to the
//! thermal-zone and electrical-network models) needs to publish them each
//! tick. Recorded in `PROGRESS.md`.

use super::routing::build_generic_a380_network;
use super::zones::Zone;
use crate::deep::api::*;

fn frac(name: &str, meaning: &str) -> ParamDef {
    ParamDef { name: name.to_string(), meaning: meaning.to_string(), healthy: 0.0 }
}

/// The seven fault-kind health parameters every wiring-harness component
/// carries, matching `faults::FaultKind` exactly (one param per kind, all
/// "0 = healthy .. 1 = fully failed").
fn harness_params() -> Vec<ParamDef> {
    vec![
        frac("chafe_wear", "insulation breach depth at the worst chafe point in this zone's harness (0 = intact, 1 = bolted short/fully breached)"),
        frac("overheat_damage", "localized bundle overheat/fire damage fraction (0 = none, 1 = every conductor destroyed) -- see faults::zone_overheat_effects"),
        frac("connector_corrosion", "oxide-film contact resistance growth at this zone's connectors (0 = clean contact, 1 = maximum modelled corrosion)"),
        frac("water_ingress", "moisture/contamination leakage-path severity (0 = dry, 1 = worst modelled leakage path)"),
        frac("rodent_damage", "insulation/conductor damage from rodent activity (0 = none, 1 = conductor bitten through or bared)"),
        frac("maintenance_damage", "mechanical pinch/crush/cut damage from servicing (0 = none, 1 = crushed/severed)"),
        frac("open_wire_fatigue", "fatigue crack propagation through a conductor's cross-section (0 = intact, 1 = fully separated)"),
    ]
}

struct ZoneEntry {
    zone: Zone,
    component_id: &'static str,
}

/// Every zone this module's own routing catalogue (`routing::catalogue`)
/// actually threads a real circuit through -- registering a harness
/// component for a zone with no wiring in it would dangle, so
/// `UpperAvionics` (routed nowhere yet in the catalogue) is left out until
/// the catalogue covers it. 13 zones, checked by this file's own test.
fn harness_zones() -> Vec<ZoneEntry> {
    vec![
        ZoneEntry { zone: Zone::MainAvionics, component_id: "91_wiring.harness_main_avionics" },
        ZoneEntry { zone: Zone::WingRoot, component_id: "91_wiring.harness_wing_root" },
        ZoneEntry { zone: Zone::Engine(1), component_id: "91_wiring.harness_engine_1" },
        ZoneEntry { zone: Zone::Engine(2), component_id: "91_wiring.harness_engine_2" },
        ZoneEntry { zone: Zone::Engine(3), component_id: "91_wiring.harness_engine_3" },
        ZoneEntry { zone: Zone::Engine(4), component_id: "91_wiring.harness_engine_4" },
        ZoneEntry { zone: Zone::Apu, component_id: "91_wiring.harness_apu" },
        ZoneEntry { zone: Zone::TailCone, component_id: "91_wiring.harness_tail_cone" },
        ZoneEntry { zone: Zone::MainGearBay, component_id: "91_wiring.harness_main_gear_bay" },
        ZoneEntry { zone: Zone::NoseGearBay, component_id: "91_wiring.harness_nose_gear_bay" },
        ZoneEntry { zone: Zone::CargoFwd, component_id: "91_wiring.harness_cargo_fwd" },
        ZoneEntry { zone: Zone::CargoAft, component_id: "91_wiring.harness_cargo_aft" },
        ZoneEntry { zone: Zone::Cockpit, component_id: "91_wiring.harness_cockpit" },
    ]
}

/// One fault kind's registration data: its failure-name suffix, the model
/// function/field it drives, its magnitude meaning and its physical effect
/// (all mirroring `faults::FaultKind`/the function of the same purpose in
/// `faults.rs`).
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
            model_field: "wiring::faults::zone_overheat_effects(net, zone, magnitude)",
            magnitude: "0..1, zone fire/overheat severity (0 = ambient, 1 = 400 C representative severe localized electrical fire)",
            effect: "every circuit in the zone whose insulation temperature rating the fire's own temperature exceeds takes damage (leakage resistance, then inter-conductor crosstalk, then full open) -- higher-rated insulation in the same bundle survives longer",
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

pub fn register(r: &mut Registry) {
    let net = build_generic_a380_network();
    let zones = harness_zones();
    let mut n: u16 = 0;
    for z in &zones {
        r.component(ComponentDef {
            id: z.component_id.to_string(),
            area: Area::Wiring,
            ata: 91,
            name: format!("Wiring harness -- {}", z.zone.name()),
            params: harness_params(),
            failures: kinds().iter().map(|k| failure_id(Area::Wiring, 91, zone_kind_n(&zones, z.zone, k.suffix))).collect(),
        });
    }
    for z in &zones {
        for k in kinds() {
            n += 1;
            let debug_n = zone_kind_n(&zones, z.zone, k.suffix);
            debug_assert_eq!(debug_n, n, "sequential id assignment must match the lookup helper");
            let circuits_here = net.circuits_in_zone(z.zone);
            r.failure(FailureDef {
                id: failure_id(Area::Wiring, 91, n),
                area: Area::Wiring,
                ata: 91,
                name: format!("{} -- {} wiring harness", k.suffix, z.zone.name()),
                component: z.component_id.to_string(),
                model_field: format!("{} [component param {}]", k.model_field, k.param),
                magnitude: k.magnitude.to_string(),
                effect: format!("{} (this zone currently routes: {})", k.effect, if circuits_here.is_empty() { "no catalogued circuit yet".to_string() } else { circuits_here.join(", ") }),
            });
        }
    }
    register_ecam_alerts(r, &zones);
}

/// Stable, deterministic id for `(zone, fault-kind suffix)`: the zone's
/// position in `harness_zones()` times 7 kinds, plus the kind's own
/// position in `kinds()`, 1-based -- computed independently of the
/// registration loop's own running counter so `register`'s
/// `debug_assert_eq!` above is a real cross-check, not a tautology.
fn zone_kind_n(zones: &[ZoneEntry], zone: Zone, suffix: &str) -> u16 {
    let zi = zones.iter().position(|z| z.zone == zone).expect("zone must be in harness_zones") as u16;
    let ki = kinds().iter().position(|k| k.suffix == suffix).expect("suffix must be in kinds") as u16;
    zi * kinds().len() as u16 + ki + 1
}

/// ECAM: the one genuinely wiring-specific, safety-relevant event a crew
/// would see annunciated is a developing bundle overheat/fire -- every
/// other fault kind here (chafe, corrosion, ingress, rodent/maintenance
/// damage, a fatigue-opened wire) surfaces to the crew only through
/// whatever *downstream* system loses power or gives a bad reading (that
/// system's own already-registered ECAM alert, not a new one here, since
/// duplicating it under a second key would misrepresent one physical event
/// as two). Titles below are GENERIC representative Airbus-style ECAM
/// wording (short caps mnemonic, ATA-appropriate) -- this push's `docs/deep/
/// BRIEF.md` rule 3 GENERIC-labelling applied to alert text the same way
/// every physics module here labels a non-A380-sourced constant, not a
/// claim that this exact wording appears in a real A380 FCOM/QRH (no public
/// A380-specific wiring-fire ECAM message text exists to cite).
fn register_ecam_alerts(r: &mut Registry, zones: &[ZoneEntry]) {
    for z in zones {
        let overheat_failure_id = failure_id(Area::Wiring, 91, zone_kind_n(zones, z.zone, "BundleOverheat"));
        let zone_tag = z.zone.name();
        let var_name = format!("WIRING_ZONE_{zone_tag}_OVERHEAT_SEVERITY");
        r.alert(
            EcamAlert::new(&format!("WIRING_OVHT_{zone_tag}"), 91, &format!("{zone_tag} WIRE OVHT"), Level::Caution, var(&var_name).gt(0.5))
                .confirm(5.0)
                .status_line(&format!("{zone_tag} WIRING -- OVHT"))
                .inop_sys(&format!("{zone_tag} WIRING HARNESS"))
                .raised_by(&[overheat_failure_id]),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registers_without_validation_errors() {
        let mut r = Registry::default();
        register(&mut r);
        let errors = r.validate();
        assert!(errors.is_empty(), "{errors:?}");
    }

    #[test]
    fn ninety_one_failures_thirteen_components_thirteen_alerts() {
        let mut r = Registry::default();
        register(&mut r);
        assert_eq!(r.components.len(), 13);
        assert_eq!(r.failures.len(), 13 * 7);
        assert_eq!(r.alerts.len(), 13);
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
    fn zone_kind_n_is_stable_and_1_based_with_no_collisions() {
        let zones = harness_zones();
        let mut seen = std::collections::HashSet::new();
        for z in &zones {
            for k in kinds() {
                let n = zone_kind_n(&zones, z.zone, k.suffix);
                assert!(n >= 1 && n <= 91);
                assert!(seen.insert(n), "duplicate n {n}");
            }
        }
    }
}
