#[allow(unused_imports)]
use super::{failure_on, Provocation, Provoked};
use crate::deep::api::{FailureDef, Registry};

fn is_weld(f: &FailureDef) -> bool {
    f.component.starts_with("17_breakers.") && f.model_field.contains("contact_resistance")
}

fn is_drift(f: &FailureDef) -> bool {
    f.component.starts_with("17_breakers.") && f.model_field.contains("trip_calibration_drift")
}

fn unit_id(f: &FailureDef) -> &str {
    f.component.strip_prefix("17_breakers.").unwrap_or(f.component.as_str())
}

fn protected_load_id(id: &str) -> Option<&'static str> {
    crate::deep::breakers::catalog::all().into_iter().find(|d| d.id == id).and_then(|d| d.protected_load)
}

fn load_short_to_ground(registry: &Registry, load_id: &str) -> Option<u64> {
    let suffix = format!("_elec.{load_id}");
    registry.failures.iter().find(|rf| rf.component.ends_with(&suffix) && rf.model_field.ends_with("short_to_ground")).map(|rf| rf.id)
}

fn load_high_resistance(registry: &Registry, load_id: &str) -> Option<u64> {
    let suffix = format!("_elec.{load_id}");
    registry.failures.iter().find(|rf| rf.component.ends_with(&suffix) && rf.model_field.ends_with("high_resistance")).map(|rf| rf.id)
}

pub(super) fn provocations() -> Vec<Provocation> {
    vec![
        Provocation {
            applies: is_weld,
            provoke: |f, registry| {
                let id = unit_id(f);
                let mut companions = Vec::new();
                if let Some(load_id) = protected_load_id(id) {
                    if let Some(short) = load_short_to_ground(registry, load_id) {
                        companions.push((short, 1.0));
                    }
                }
                Provoked { later: None, profile: "cruise_soak", truth: None, companions, frames: 60 }
            },
            why: "a welded breaker's real trigger is a short on its own protected load: a healthy breaker clears it (magnetic or thermal trip, load de-energised); with the weld armed the same short leaves the breaker closed and the fault current keeps flowing onto the bus/feeder",
        },
        Provocation {
            applies: is_drift,
            provoke: |f, registry| {
                let id = unit_id(f);
                let mut companions = Vec::new();
                if let Some(load_id) = protected_load_id(id) {
                    if let Some(hr) = load_high_resistance(registry, load_id) {
                        companions.push((hr, 1.0));
                    }
                }
                Provoked { later: None, profile: "cruise_soak", truth: None, companions, frames: 200 }
            },
            why: "a drifted trip point only lowers the threshold by up to 40% of true rated current (trip.rs MAX_CALIBRATION_DRIFT): a breaker sized with the normal margin over its load's steady current never reaches that threshold no matter how long it runs, so the real trigger pairs the drift with the load's own high_resistance fault (an aged winding/connection drawing extra current for the same output) pushing the combined effective current past the lowered threshold; a unit whose steady current already sits close to its rated current can trip from drift plus this companion alone, a unit with large design margin may still legitimately stay latent even then",
        },
    ]
}
