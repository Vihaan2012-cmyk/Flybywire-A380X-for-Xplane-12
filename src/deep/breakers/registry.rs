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
        assert_eq!(r.components.len(), catalog::all().len());
        assert_eq!(r.failures.len(), catalog::all().len() * 2);
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
