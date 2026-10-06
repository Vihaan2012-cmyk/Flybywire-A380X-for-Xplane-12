#[allow(unused_imports)]
use super::{failure_on, Provocation, Provoked};
#[allow(unused_imports)]
use crate::deep::api::{FailureDef, Registry};
use crate::deep::live::Truth;

fn load_id_for_breaker(breaker_id: &str) -> &str {
    for suffix in ["-normal-bkr", "-2nd-bkr", "-3rd-bkr"] {
        if let Some(stripped) = breaker_id.strip_suffix(suffix) {
            return stripped;
        }
    }
    breaker_id
}

fn failure_on_load(registry: &Registry, load_id: &str, name_part: &str) -> Option<u64> {
    let suffix = format!("_elec.{load_id}");
    registry.failures.iter().find(|f| f.component.ends_with(&suffix) && f.name.contains(name_part)).map(|f| f.id)
}

fn provocations_breaker_fails_to_trip() -> Provocation {
    Provocation {
        applies: |f| f.component.starts_with("24_elec.bkr.") && f.name.contains("fails to trip"),
        provoke: |f, registry| {
            let breaker_id = f.component.strip_prefix("24_elec.bkr.").unwrap_or(f.component.as_str());
            let load_id = load_id_for_breaker(breaker_id);
            let companions = match failure_on_load(registry, load_id, "short to ground") {
                Some(id) => vec![(id, 1.0)],
                None => Vec::new(),
            };
            Provoked { later: None, profile: "cruise", truth: None, companions, frames: 40 }
        },
        why: "a breaker that fails to trip is silent until its own load shorts: the real trigger is that load's own short_to_ground fault, which a healthy breaker clears and a jammed one lets ride",
    }
}

fn provocations_gear_actuator_load_faults() -> Provocation {
    Provocation {
        applies: |f| {
            f.model_field.starts_with("deep::electrical::network::Load.faults")
                && (f.component.contains(".gear-actuator-") || f.component.contains(".gear-door-actuator-"))
        },
        provoke: |_f, _registry| Provoked { later: None,
            profile: "gear_cycle",
            truth: Some(Box::new(|t: &mut Truth| t.controls.gear_door_commanded_open = [0.5; 3])),
            companions: Vec::new(),
            frames: 40,
        },
        why: "a gear/gear-door actuator only draws current while the gear is travelling: the doors held mid-travel, so an open circuit, short, high-resistance or intermittent fault has real current to act on",
    }
}

fn tr_line_truth(id: &'static str) -> Option<Box<dyn Fn(&mut Truth)>> {
    match id {
        "tr-1-line" => Some(Box::new(|t: &mut Truth| {
            t.engine_running[0] = false;
            t.controls.engine_master_on[0] = false;
        })),
        "tr-2-line" => Some(Box::new(|t: &mut Truth| {
            t.engine_running[1] = false;
            t.controls.engine_master_on[1] = false;
        })),
        "tr-ess-line" => Some(Box::new(|t: &mut Truth| {
            t.engine_running[0] = false;
            t.controls.engine_master_on[0] = false;
            t.engine_running[3] = false;
            t.controls.engine_master_on[3] = false;
        })),
        _ => None,
    }
}

fn provocations_tr_line_welded_closed() -> Provocation {
    Provocation {
        applies: |f| {
            f.component.starts_with("24_elec.contactor.")
                && f.name.contains("welded closed")
                && (f.component.ends_with("tr-1-line")
                    || f.component.ends_with("tr-2-line")
                    || f.component.ends_with("tr-ess-line")
                    || f.component.ends_with("tr-apu-line"))
        },
        provoke: |f, _registry| {
            let id: &'static str = if f.component.ends_with("tr-1-line") {
                "tr-1-line"
            } else if f.component.ends_with("tr-2-line") {
                "tr-2-line"
            } else if f.component.ends_with("tr-ess-line") {
                "tr-ess-line"
            } else {
                "tr-apu-line"
            };
            Provoked { later: None, profile: "cruise", truth: tr_line_truth(id), companions: Vec::new(), frames: 16 }
        },
        why: "a TR-line contactor's welded_closed only matters once its own TR stops producing (that TR's generator lost) and the contactor is commanded open; a healthy one isolates the dead TR, a welded one leaves it connected",
    }
}

fn provocations_rat_line_fails_to_close() -> Provocation {
    Provocation {
        applies: |f| f.component.ends_with("24_elec.contactor.rat-line") && f.name.contains("fails to close"),
        provoke: |_f, _registry| Provoked { later: None,
            profile: "gear_cycle",
            truth: Some(Box::new(|t: &mut Truth| {
                t.engine_running = [false; 4];
                t.controls.engine_master_on = [false; 4];
                t.apu_running = false;
                t.gpu_plugged_in = false;
            })),
            companions: Vec::new(),
            frames: 40,
        },
        why: "rat-line's fails_to_close only matters once all main generation is lost in flight: the RAT deploys and should power AC EMER, and a healthy contactor makes that connection while a failed one never does",
    }
}

pub(super) fn provocations() -> Vec<Provocation> {
    vec![
        provocations_breaker_fails_to_trip(),
        provocations_gear_actuator_load_faults(),
        provocations_tr_line_welded_closed(),
        provocations_rat_line_fails_to_close(),
    ]
}
