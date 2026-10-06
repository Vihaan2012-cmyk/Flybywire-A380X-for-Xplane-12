//! Registers every [`super::catalog`] breaker as a component (two health
//! params: `trip_calibration_drift`, `contact_resistance`) with its own pair
//! of failures (nuisance trip, fails to trip) through `deep::api::Registry`,
//! per `docs/deep/BRIEF.md`'s "Registering failures, components and ECAM
//! alerts". `n` is assigned sequentially per ATA chapter as each breaker is
//! walked in `catalog::all()`'s own fixed order, so re-running `register`
//! against an unchanged catalogue always reproduces the same ids.
//!
//! No ECAM alerts are registered here. A real A380 breaker trip/fails-to-
//! trip fault has no ECAM message of its own -- the *consuming system's*
//! own loss-of-power (or, for a fails-to-trip overload, that system's own
//! overheat/smoke) alert is what actually annunciates, and that alert
//! belongs to the area modelling that system (`deep::electrical`,
//! `deep::fire_ice`, ...), not to this one. Inventing a breaker-specific
//! alert here would be exactly the padding/renaming `docs/deep/BRIEF.md`
//! tells every area not to do.

use std::collections::HashMap;

use super::catalog::{self, BreakerDef};
use crate::deep::api::*;

pub fn register(r: &mut Registry) {
    let mut next_n: HashMap<u16, u16> = HashMap::new();
    for def in catalog::all() {
        register_one(r, def, &mut next_n);
    }
    register_misc(r, &mut next_n);
}

/// Three armable conditions `E:/fbw-debug/ecam/E-ELEC-DESIGN.md`/
/// `BRIEF-phase2-FCOM.md`'s Task B pass add: the two monitoring-*path*
/// health verdicts for `240800020`/`054` (the computer that watches all 399
/// trip units, and the emergency-configuration-specific channel, each
/// independent of whether any individual breaker has actually tripped), and
/// `240800073`'s maintenance-panel switch left ON. Unlike every other entry
/// `register_one` above adds, none of these three is one of `catalog::all()`'s
/// 399 real trip units, so they are registered directly here instead.
fn register_misc(r: &mut Registry, next_n: &mut HashMap<u16, u16>) {
    let entries: [(&str, &str, &str, &str); 3] = [
        ("cb-monitoring", "C/B MONITORING computer", "C/B MONITORING FAULT", "the breaker-monitoring computer itself, independent of any individual breaker's real state (240800020)"),
        ("emer-cb-monitoring", "EMER C/B MONITORING computer", "EMER C/B MONITORING FAULT", "the emergency-configuration-specific monitoring channel, active only once ELEC_EMER_CONFIG_ACTIVE is on (240800054)"),
        (
            "remote-cb-ctl",
            "REMOTE C/B CTL maintenance panel pb",
            "REMOTE C/B CTL LEFT ON",
            "FCOM p.4954: maintenance personnel left the REMOTE C/B CTL pb (maintenance panel) set to ON; boolean switch position, not a failure (240800073)",
        ),
    ];
    for (suffix, comp_name, failure_name, meaning) in entries {
        let comp_id = format!("17_breakers.misc.{suffix}");
        let n = take_n(next_n, 24);
        let id = failure_id(Area::Breakers, 24, n);
        r.component(ComponentDef {
            id: comp_id.clone(),
            area: Area::Breakers,
            ata: 24,
            name: comp_name.to_string(),
            params: vec![ParamDef { name: "fault".into(), meaning: meaning.into(), healthy: 0.0 }],
            failures: vec![id],
        });
        r.failure(FailureDef {
            id,
            area: Area::Breakers,
            ata: 24,
            name: failure_name.to_string(),
            component: comp_id,
            model_field: format!("deep::breakers::live::BreakersLive.misc.{suffix}"),
            magnitude: "boolean: 0 healthy/normal, >0 faulted/mis-set".into(),
            effect: meaning.to_string(),
        });
    }
}

fn take_n(next_n: &mut HashMap<u16, u16>, ata: u16) -> u16 {
    let n = next_n.entry(ata).or_insert(1);
    let v = *n;
    *n += 1;
    v
}

fn register_one(r: &mut Registry, def: &BreakerDef, next_n: &mut HashMap<u16, u16>) {
    let comp_id = format!("17_breakers.{}", def.id);
    let n_nuisance = take_n(next_n, def.ata);
    let n_fails = take_n(next_n, def.ata);
    let nuisance_id = failure_id(Area::Breakers, def.ata, n_nuisance);
    let fails_id = failure_id(Area::Breakers, def.ata, n_fails);

    r.component(ComponentDef {
        id: comp_id.clone(),
        area: Area::Breakers,
        ata: def.ata,
        name: format!("{} breaker", def.name),
        params: vec![
            ParamDef {
                name: "trip_calibration_drift".into(),
                meaning: "0 = correctly calibrated .. 1 = fully drifted low: nuisance-trips below the breaker's true rated current (bimetal spring aging fatigue, or an SSPC's own reference/firmware drift)".into(),
                healthy: 0.0,
            },
            ParamDef {
                name: "contact_resistance".into(),
                meaning: "0 = clean contacts .. 1 = welded/fused by repeated arcing: raises the I2t heat (and magnetic instantaneous multiple) the mechanism needs before it can still force the contacts open, up to effectively never".into(),
                healthy: 0.0,
            },
        ],
        failures: vec![nuisance_id, fails_id],
    });

    let protects = def.protected_load.map(|l| format!("deep::electrical load \"{l}\" ({})", def.consumer)).unwrap_or_else(|| format!("{} (real A380 equipment, no modelled load in this codebase yet)", def.consumer));

    r.failure(FailureDef {
        id: nuisance_id,
        area: Area::Breakers,
        ata: def.ata,
        name: format!("{} nuisance trip (calibration drift)", def.name),
        component: comp_id.clone(),
        model_field: "deep::breakers::trip::Breaker.step's BreakerFaults.trip_calibration_drift".into(),
        magnitude: "0..1: lowers this breaker's effective trip threshold by up to 40% of its true rated current (trip.rs's own effective_rated_a term)".into(),
        effect: format!("{} opens under a load it should carry, de-energising {}", def.name, protects),
    });
    r.failure(FailureDef {
        id: fails_id,
        area: Area::Breakers,
        ata: def.ata,
        name: format!("{} fails to trip (contact weld)", def.name),
        component: comp_id,
        model_field: "deep::breakers::trip::Breaker.step's BreakerFaults.contact_resistance".into(),
        magnitude: "0..1: raises the I2t heat threshold and the magnetic instantaneous multiple by 1/(1-contact_resistance) (diverging as it approaches 1.0, not a fixed multiple) before the mechanism can still force welded contacts open".into(),
        effect: format!("{} does not open on a genuine overload/short, {} keeps drawing fault current downstream of a breaker that should have isolated it", def.name, protects),
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_whole_catalogue_registers_with_no_dangling_references_or_collisions() {
        let mut r = Registry::default();
        register(&mut r);
        let errors = r.validate_area();
        assert!(errors.is_empty(), "registry errors: {errors:?}");
    }

    #[test]
    fn every_breaker_gets_its_own_component_and_exactly_two_failures() {
        let mut r = Registry::default();
        register(&mut r);
        // `register_misc`'s 3 computer-health/switch-position entries
        // (`240800020`/`054`/`073`) are not one of the 399 real trip units,
        // and carry one failure each rather than two.
        const MISC_COMPONENTS: usize = 3;
        const MISC_FAILURES: usize = 3;
        assert_eq!(r.components.len(), catalog::all().len() + MISC_COMPONENTS);
        assert_eq!(r.failures.len(), catalog::all().len() * 2 + MISC_FAILURES);
    }

    #[test]
    fn no_ata_chapter_overflows_the_failure_id_encoding() {
        // failure_id = area*1_000_000 + ata*1_000 + n; n must stay below
        // 1000 or it would corrupt the ata digit of the next chapter up.
        let mut counts: HashMap<u16, u32> = HashMap::new();
        for def in catalog::all() {
            *counts.entry(def.ata).or_insert(0) += 2;
        }
        for (ata, count) in counts {
            assert!(count < 1000, "ATA {ata} has {count} breaker failures, would overflow the id encoding");
        }
    }

    #[test]
    fn component_ids_are_unique_and_match_every_failures_own_component_field() {
        let mut r = Registry::default();
        register(&mut r);
        let mut ids: Vec<&str> = r.components.iter().map(|c| c.id.as_str()).collect();
        let before = ids.len();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), before);
        for f in &r.failures {
            assert!(r.components.iter().any(|c| c.id == f.component), "failure {} names unknown component {}", f.id, f.component);
        }
    }
}
