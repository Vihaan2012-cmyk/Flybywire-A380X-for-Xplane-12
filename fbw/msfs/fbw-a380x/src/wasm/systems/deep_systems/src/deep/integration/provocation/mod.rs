use crate::deep::api::{FailureDef, Registry};
use crate::deep::live::{Faults, Truth};

use super::failure_audit::{baseline_phased, first_diff_phased, profiles, MAGNITUDES, SENTINEL_ID};

mod breakers;
mod electrical;
mod fire_gear_apu;
mod sensors;
mod systems;

pub struct Provocation {
    pub applies: fn(&FailureDef) -> bool,
    pub provoke: fn(&FailureDef, &Registry) -> Provoked,
    pub why: &'static str,
}

pub struct Provoked {
    pub profile: &'static str,
    pub truth: Option<Box<dyn Fn(&mut Truth)>>,
    pub companions: Vec<(u64, f64)>,
    pub frames: usize,
    pub later: Option<(usize, Box<dyn Fn(&mut Truth)>)>,
}

impl Provoked {
    pub fn truth_at(&self, truth: Truth) -> impl Fn(usize) -> Truth {
        let later = self.later.as_ref().map(|(frame, change)| {
            let mut t = truth.clone();
            change(&mut t);
            (*frame, t)
        });
        move |frame| match &later {
            Some((from, t)) if frame >= *from => t.clone(),
            _ => truth.clone(),
        }
    }
}

pub fn all() -> Vec<Provocation> {
    let mut v = Vec::new();
    v.extend(breakers::provocations());
    v.extend(electrical::provocations());
    v.extend(sensors::provocations());
    v.extend(fire_gear_apu::provocations());
    v.extend(systems::provocations());
    v
}

pub fn for_failure(f: &FailureDef) -> Option<Provocation> {
    all().into_iter().find(|p| (p.applies)(f))
}

#[derive(Clone, Debug)]
pub enum ProvokedVerdict {
    NoProvocation,
    UnknownProfile(&'static str),
    Live { why: &'static str, magnitude: f64, moved: Vec<String>, moved_count: usize },
    StillDead { why: &'static str },
}

pub fn provoked_verdict(f: &FailureDef, registry: &Registry) -> ProvokedVerdict {
    let Some(p) = for_failure(f) else {
        return ProvokedVerdict::NoProvocation;
    };
    let provoked = (p.provoke)(f, registry);
    let Some(profile) = profiles().into_iter().find(|x| x.name == provoked.profile) else {
        return ProvokedVerdict::UnknownProfile(provoked.profile);
    };
    let mut truth = (profile.truth)();
    if let Some(change) = &provoked.truth {
        change(&mut truth);
    }
    let mut reference: Vec<(u64, f64)> = vec![(SENTINEL_ID, 1.0)];
    reference.extend(provoked.companions.iter().copied().filter(|&(id, _)| id != f.id));
    let truth_at = provoked.truth_at(truth);
    let base = baseline_phased(&truth_at, &Faults::from_pairs(reference.clone()), provoked.frames);
    for m in MAGNITUDES {
        let mut armed = reference.clone();
        armed.push((f.id, m));
        let d = first_diff_phased(&base, &truth_at, &Faults::from_pairs(armed));
        if !d.is_empty() {
            let mut moved: Vec<String> = d.changed.iter().take(6).map(|&i| base.names[i].clone()).collect();
            if d.shape_changed {
                moved.push("<published name set changed>".into());
            }
            return ProvokedVerdict::Live { why: p.why, magnitude: m, moved, moved_count: d.changed.len() };
        }
    }
    ProvokedVerdict::StillDead { why: p.why }
}

pub fn failure_on(registry: &Registry, component: &str, name_part: &str) -> Option<u64> {
    registry.failures.iter().find(|f| f.component == component && f.name.contains(name_part)).map(|f| f.id)
}
