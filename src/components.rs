//! Damageable components and their physical parameters: the failure model.
//!
//! A failure is not a switch with a scripted effect. Every physical model
//! registers the components it simulates and, for each, the quantities that
//! can physically change (an efficiency lost to blade damage, a leak area, a
//! valve's stuck position, extra bearing friction, a sensor bias), each with
//! its unit, healthy value and range. The model reads the current value every
//! tick and its physics produces the consequence.
//!
//! Anything can move any parameter:
//! - a catalogued failure (`failures.rs`) is a named set of perturbations,
//!   scaled by its magnitude ([`define_failure`]);
//! - the Failures tab can set any component's parameter directly
//!   ([`set_direct`]), so the space of failures is every combination of every
//!   parameter, not a list;
//! - a perturbation can worsen over time ([`set_progression`]), as wear does;
//! - damage-spreading events (rotor burst fragments, fire) will set them too.
//!
//! When several sources act on one parameter they combine by the parameter's
//! own [`Combine`] rule: independent fractional losses compound
//! (`1 - prod(1 - loss)`), additive quantities (leak area, friction) sum, and
//! for a position the most displaced value wins.
//!
//! Values are read through a [`Handle`] a model takes at construction, so the
//! per-tick read is an index into a vector, not a name lookup.

use std::collections::BTreeMap;
use std::sync::{Mutex, RwLock};

/// How several sources acting on one parameter combine.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Combine {
    /// Independent fractional losses in 0..1: `1 - prod(1 - x)`.
    CompoundLoss,
    /// Quantities that add: leak area, extra friction, bias.
    Sum,
    /// A position or level: the value furthest from healthy wins.
    MostDisplaced,
}

/// One physical quantity of a component.
#[derive(Clone, Debug, serde::Serialize)]
pub struct ParamSpec {
    pub name: &'static str,
    pub unit: &'static str,
    pub healthy: f64,
    pub min: f64,
    pub max: f64,
    pub combine: Combine,
    pub description: &'static str,
}

/// What a model holds to read one parameter each tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Handle(usize);

/// One contribution to a parameter, from a named source.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Perturbation {
    pub component: String,
    pub param: String,
    /// The parameter's value this source asks for (at full magnitude, for a
    /// failure's; the failure's magnitude scales the displacement from
    /// healthy).
    pub value: f64,
}

/// A catalogued failure: the perturbations it applies at full magnitude.
#[derive(Clone, Debug)]
pub struct FailureDef {
    pub id: u64,
    pub perturbations: Vec<Perturbation>,
}

/// A direct setting from the Failures tab (or a spreading event), persisted,
/// optionally worsening at `rate_per_hour` (in the parameter's own unit)
/// toward its limit.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Direct {
    pub component: String,
    pub param: String,
    pub value: f64,
    #[serde(default)]
    pub rate_per_hour: f64,
}

struct Slot {
    component: String,
    spec: ParamSpec,
    /// The combined value, recomputed whenever a source changes.
    value: f64,
}

#[derive(Default)]
struct Registry {
    slots: Vec<Slot>,
    by_key: BTreeMap<(String, String), usize>,
    failures: BTreeMap<u64, FailureDef>,
    direct: BTreeMap<(String, String), Direct>,
    /// Failures whose whole effect is one component's `loss`
    /// (`failures::register_component_catalogue`): failure id -> slot. Their
    /// combined value goes back to `failures` as the failure's level.
    mirrors: BTreeMap<u64, usize>,
}

static REGISTRY: RwLock<Option<Registry>> = RwLock::new(None);
/// Serialises writers so a read of `REGISTRY` never sees half an update.
static WRITE: Mutex<()> = Mutex::new(());

fn with_mut<R>(f: impl FnOnce(&mut Registry) -> R) -> R {
    let _guard = WRITE.lock().unwrap_or_else(|e| e.into_inner());
    let mut reg = REGISTRY.write().unwrap_or_else(|e| e.into_inner());
    f(reg.get_or_insert_with(Registry::default))
}

/// Register `component`'s parameters (idempotent: registering again returns
/// the same handles). Returns one handle per spec, in order.
pub fn register(component: &str, specs: &[ParamSpec]) -> Vec<Handle> {
    with_mut(|r| {
        let handles = specs
            .iter()
            .map(|spec| {
                let key = (component.to_owned(), spec.name.to_owned());
                if let Some(&i) = r.by_key.get(&key) {
                    return Handle(i);
                }
                let i = r.slots.len();
                r.slots.push(Slot { component: component.to_owned(), spec: spec.clone(), value: spec.healthy });
                r.by_key.insert(key, i);
                Handle(i)
            })
            .collect();
        recompute(r);
        handles
    })
}

/// Register catalogued failures as components, each with one parameter
/// the failure drives to its maximum at full magnitude, in one pass.
pub fn register_failure_mirrors(specs: &[(u64, String, ParamSpec)]) {
    with_mut(|r| {
        for (id, component, spec) in specs {
            let key = (component.clone(), spec.name.to_owned());
            let i = match r.by_key.get(&key) {
                Some(&i) => i,
                None => {
                    let i = r.slots.len();
                    r.slots.push(Slot { component: component.clone(), spec: spec.clone(), value: spec.healthy });
                    r.by_key.insert(key, i);
                    i
                }
            };
            r.failures.insert(
                *id,
                FailureDef { id: *id, perturbations: vec![Perturbation { component: component.clone(), param: spec.name.to_owned(), value: spec.max }] },
            );
            r.mirrors.insert(*id, i);
        }
        recompute(r);
    });
}

/// A parameter's current combined value (its healthy value if nothing acts
/// on it, or if the handle is unknown).
pub fn value(handle: Handle) -> f64 {
    REGISTRY.read().ok().and_then(|r| r.as_ref().and_then(|r| r.slots.get(handle.0).map(|s| s.value))).unwrap_or(0.0)
}

/// Declare a catalogued failure's perturbations (at full magnitude).
pub fn define_failure(def: FailureDef) {
    with_mut(|r| {
        r.failures.insert(def.id, def);
        recompute(r);
    });
}

/// Set a parameter directly (the Failures tab, a spreading event). A value
/// equal to healthy with no progression removes the setting.
pub fn set_direct(component: &str, param: &str, value: f64, rate_per_hour: f64) -> Result<(), String> {
    with_mut(|r| {
        let key = (component.to_owned(), param.to_owned());
        let Some(&i) = r.by_key.get(&key) else {
            return Err(format!("no parameter {param} on {component}"));
        };
        let spec = &r.slots[i].spec;
        let value = value.clamp(spec.min, spec.max);
        if value == spec.healthy && rate_per_hour == 0.0 {
            r.direct.remove(&key);
        } else {
            r.direct.insert(key, Direct { component: component.to_owned(), param: param.to_owned(), value, rate_per_hour });
        }
        recompute(r);
        Ok(())
    })
}

/// Make a direct setting worsen over time (wear).
pub fn set_progression(component: &str, param: &str, rate_per_hour: f64) -> Result<(), String> {
    let current = with_mut(|r| {
        r.direct
            .get(&(component.to_owned(), param.to_owned()))
            .map(|d| d.value)
            .or_else(|| r.by_key.get(&(component.to_owned(), param.to_owned())).map(|&i| r.slots[i].spec.healthy))
    });
    match current {
        Some(v) => set_direct(component, param, v, rate_per_hour),
        None => Err(format!("no parameter {param} on {component}")),
    }
}

/// Advance progressing settings by `dt_hours` and recombine with the active
/// failures' magnitudes. Call once per tick (real, unpaused time).
pub fn tick(dt_hours: f64) {
    with_mut(|r| {
        if dt_hours > 0.0 {
            let limits: BTreeMap<(String, String), (f64, f64)> = r
                .direct
                .keys()
                .filter_map(|k| r.by_key.get(k).map(|&i| (k.clone(), (r.slots[i].spec.min, r.slots[i].spec.max))))
                .collect();
            for (key, d) in r.direct.iter_mut() {
                if d.rate_per_hour != 0.0 {
                    let (lo, hi) = limits.get(key).copied().unwrap_or((f64::MIN, f64::MAX));
                    d.value = (d.value + d.rate_per_hour * dt_hours).clamp(lo, hi);
                }
            }
        }
        recompute(r);
    });
}

/// Every source acting on each parameter, combined.
fn recompute(r: &mut Registry) {
    let mut sources: Vec<Vec<f64>> = vec![Vec::new(); r.slots.len()];
    for def in r.failures.values() {
        // Armed magnitude only: the failure's effective magnitude includes
        // this very recompute's result for mirrored failures.
        let m = crate::failures::armed_magnitude(def.id);
        if m <= 0.0 {
            continue;
        }
        for p in &def.perturbations {
            if let Some(&i) = r.by_key.get(&(p.component.clone(), p.param.clone())) {
                let h = r.slots[i].spec.healthy;
                sources[i].push(h + (p.value - h) * m.clamp(0.0, 1.0));
            }
        }
    }
    for (key, d) in &r.direct {
        if let Some(&i) = r.by_key.get(key) {
            sources[i].push(d.value);
        }
    }
    for (slot, src) in r.slots.iter_mut().zip(sources) {
        let h = slot.spec.healthy;
        let combined = if src.is_empty() {
            h
        } else {
            match slot.spec.combine {
                Combine::CompoundLoss => 1.0 - src.iter().map(|v| 1.0 - v.clamp(0.0, 1.0)).product::<f64>(),
                Combine::Sum => h + src.iter().map(|v| v - h).sum::<f64>(),
                Combine::MostDisplaced => src.iter().copied().fold(h, |best, v| if (v - h).abs() > (best - h).abs() { v } else { best }),
            }
        };
        slot.value = combined.clamp(slot.spec.min, slot.spec.max);
    }
    if !r.mirrors.is_empty() {
        let levels = r
            .mirrors
            .iter()
            .filter_map(|(&id, &i)| {
                let s = &r.slots[i];
                let level = ((s.value - s.spec.healthy) / (s.spec.max - s.spec.healthy)).clamp(0.0, 1.0);
                (level > 0.0).then_some((id, level))
            })
            .collect();
        crate::failures::set_component_levels(levels);
    }
}

/// The direct settings, for persistence.
pub fn snapshot_direct() -> Vec<Direct> {
    REGISTRY.read().ok().and_then(|r| r.as_ref().map(|r| r.direct.values().cloned().collect())).unwrap_or_default()
}

/// Restore persisted direct settings (after the models have registered).
pub fn restore_direct(settings: Vec<Direct>) {
    for d in settings {
        let _ = set_direct(&d.component, &d.param, d.value, d.rate_per_hour);
    }
}

/// Every registered component parameter, for the Failures tab.
#[derive(Clone, Debug, serde::Serialize)]
pub struct Listed {
    pub component: String,
    pub spec: ParamSpec,
    pub value: f64,
    pub direct: Option<f64>,
    pub rate_per_hour: f64,
}

pub fn list() -> Vec<Listed> {
    REGISTRY
        .read()
        .ok()
        .and_then(|r| {
            r.as_ref().map(|r| {
                r.slots
                    .iter()
                    .map(|s| {
                        let d = r.direct.get(&(s.component.clone(), s.spec.name.to_owned()));
                        Listed {
                            component: s.component.clone(),
                            spec: s.spec.clone(),
                            value: s.value,
                            direct: d.map(|d| d.value),
                            rate_per_hour: d.map_or(0.0, |d| d.rate_per_hour),
                        }
                    })
                    .collect()
            })
        })
        .unwrap_or_default()
}

/// Test-only: clear everything (process-global, like `failures::reset_all`).
pub fn reset_all() {
    let _guard = WRITE.lock().unwrap_or_else(|e| e.into_inner());
    if let Ok(mut r) = REGISTRY.write() {
        *r = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const LOSS: ParamSpec = ParamSpec {
        name: "efficiency_loss",
        unit: "fraction",
        healthy: 0.0,
        min: 0.0,
        max: 1.0,
        combine: Combine::CompoundLoss,
        description: "",
    };
    const LEAK: ParamSpec =
        ParamSpec { name: "leak_area", unit: "m2", healthy: 0.0, min: 0.0, max: 1.0, combine: Combine::Sum, description: "" };

    #[test]
    fn two_sources_combine_by_the_parameters_own_rule() {
        let _g = crate::failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        reset_all();
        let _f = crate::failures::Failures::new();
        let h = register("test.a", &[LOSS, LEAK]);
        set_direct("test.a", "efficiency_loss", 0.5, 0.0).unwrap();
        define_failure(FailureDef {
            id: 72_012,
            perturbations: vec![Perturbation { component: "test.a".into(), param: "efficiency_loss".into(), value: 0.5 }],
        });
        crate::failures::set_magnitude(72_012, 1.0);
        tick(0.0);
        // Independent 50% losses compound to 75%, not 100%.
        assert!((value(h[0]) - 0.75).abs() < 1e-9, "{}", value(h[0]));
        set_direct("test.a", "leak_area", 0.01, 0.0).unwrap();
        assert!((value(h[1]) - 0.01).abs() < 1e-12);
        crate::failures::set_magnitude(72_012, 0.0);
        reset_all();
    }

    #[test]
    fn a_failures_magnitude_scales_its_displacement_from_healthy() {
        let _g = crate::failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        reset_all();
        let _f = crate::failures::Failures::new();
        let h = register("test.b", &[LOSS]);
        define_failure(FailureDef {
            id: 72_013,
            perturbations: vec![Perturbation { component: "test.b".into(), param: "efficiency_loss".into(), value: 0.8 }],
        });
        crate::failures::set_magnitude(72_013, 0.5);
        tick(0.0);
        assert!((value(h[0]) - 0.4).abs() < 1e-9, "{}", value(h[0]));
        crate::failures::set_magnitude(72_013, 0.0);
        reset_all();
    }

    #[test]
    fn a_progressing_setting_worsens_to_its_limit() {
        let _g = crate::failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        reset_all();
        let h = register("test.c", &[LOSS]);
        set_direct("test.c", "efficiency_loss", 0.1, 0.2).unwrap();
        tick(2.0);
        assert!((value(h[0]) - 0.5).abs() < 1e-9, "{}", value(h[0]));
        tick(10.0);
        assert!((value(h[0]) - 1.0).abs() < 1e-9, "clamped at its max: {}", value(h[0]));
        reset_all();
    }

    #[test]
    fn a_degraded_component_drives_its_failure_like_arming_it() {
        reset_all();
        crate::failures::reset_for_tests();
        crate::failures::register_component_catalogue();
        let prim = list().into_iter().find(|c| c.component.ends_with(".prim_1")).expect("PRIM 1 is a component");
        assert!(!crate::failures::is_active(27_000));
        set_direct(&prim.component, "loss", 0.4, 0.0).unwrap();
        assert!(crate::failures::is_active(27_000), "a degraded PRIM 1 is a failed PRIM 1");
        assert!((crate::failures::magnitude(27_000) - 0.4).abs() < 1e-9);
        assert!(crate::failures::active_ids().contains(&27_000));
        set_direct(&prim.component, "loss", 0.0, 0.0).unwrap();
        assert!(!crate::failures::is_active(27_000), "restored to healthy");
        reset_all();
        crate::failures::reset_for_tests();
    }
}
