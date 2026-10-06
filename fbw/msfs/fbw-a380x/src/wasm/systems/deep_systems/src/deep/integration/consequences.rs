use std::collections::{BTreeMap, HashMap};

use super::failure_audit::{fresh_areas, profiles, Profile, SENTINEL_ID};
use super::provocation;
use crate::deep::api::{Cond, FailureDef, Registry};
use crate::deep::electrical::live::board::{with_board_mut, Board};
use crate::deep::live::{Deep, Faults, Truth};

const BREAKER_PROFILES: [&str; 3] = ["cruise", "ground_apu", "all_commands_exercised"];

const TAIL_FRAMES: usize = 40;

const EXAMPLES: usize = 3;

const VALUES: usize = 16;

const MAX_TRACE_SIM_S: f64 = 3600.0;

#[derive(Clone, Debug)]
pub struct AreaEffect {
    pub area: &'static str,
    pub moved: usize,
    pub examples: Vec<String>,
}

#[derive(Clone, Debug)]
pub struct FbwEffect {
    pub fbw_id: u64,
    pub deep_component: &'static str,
    pub reason: &'static str,
}

#[derive(Clone, Debug)]
pub struct Consequences {
    pub profile: &'static str,
    pub stages: Vec<Vec<AreaEffect>>,
    pub alerts: Vec<String>,
    pub fbw: Vec<FbwEffect>,
    pub develops_over_s: Option<f64>,
    pub values: Vec<(String, f64, f64)>,
}

struct AlertTrigger {
    key: String,
    trigger: Cond,
    confirm_s: f64,
}

struct Recorded {
    names: Vec<String>,
    area_of: Vec<usize>,
    areas: Vec<&'static str>,
    values: Vec<Vec<f64>>,
    alerts: Vec<Vec<bool>>,
    derived: Vec<Vec<(u64, f64)>>,
}

fn same(a: f64, b: f64) -> bool {
    a == b || (a.is_nan() && b.is_nan())
}

fn alert_states(deep: &Deep, alerts: &[AlertTrigger]) -> Vec<bool> {
    let frame = deep.last_published();
    let read = |name: &str| frame.get(name).unwrap_or(0.0);
    alerts.iter().map(|a| a.trigger.eval(&read)).collect()
}

fn record(truth: &Truth, faults: &Faults, frames: usize, alerts: &[AlertTrigger]) -> Recorded {
    let mut deep = fresh_areas();
    let mut rec = Recorded { names: Vec::new(), area_of: Vec::new(), areas: Vec::new(), values: Vec::with_capacity(frames), alerts: Vec::with_capacity(frames), derived: Vec::with_capacity(frames) };
    for frame in 0..frames {
        let first = frame == 0;
        let mut values = Vec::with_capacity(rec.names.len());
        let names = &mut rec.names;
        deep.tick(truth.clone(), faults, &mut |name, value| {
            if first {
                names.push(name.to_owned());
            }
            values.push(value);
        });
        if first {
            for (i, (area, count)) in deep.published_counts_by_area().into_iter().enumerate() {
                rec.areas.push(area);
                rec.area_of.extend(std::iter::repeat(i).take(count));
            }
            if rec.area_of.len() != rec.names.len() {
                rec.area_of = vec![usize::MAX; rec.names.len()];
            }
        }
        rec.values.push(values);
        rec.alerts.push(alert_states(&deep, alerts));
        rec.derived.push(deep.derived_failures().iter().map(|d| (d.fbw_id, d.magnitude)).collect());
    }
    rec
}

trait BaseRun {
    fn frames(&self) -> usize;
    fn advance(&mut self, frame: usize);
    fn names(&self) -> &[String];
    fn area_of(&self) -> &[usize];
    fn areas(&self) -> &[&'static str];
    fn values(&self, frame: usize) -> &[f64];
    fn alerts(&self, frame: usize) -> &[bool];
    fn derived(&self, frame: usize) -> &[(u64, f64)];
}

struct Replay<'a>(&'a Recorded);

impl BaseRun for Replay<'_> {
    fn frames(&self) -> usize {
        self.0.values.len()
    }
    fn advance(&mut self, _frame: usize) {}
    fn names(&self) -> &[String] {
        &self.0.names
    }
    fn area_of(&self) -> &[usize] {
        &self.0.area_of
    }
    fn areas(&self) -> &[&'static str] {
        &self.0.areas
    }
    fn values(&self, frame: usize) -> &[f64] {
        &self.0.values[frame]
    }
    fn alerts(&self, frame: usize) -> &[bool] {
        &self.0.alerts[frame]
    }
    fn derived(&self, frame: usize) -> &[(u64, f64)] {
        &self.0.derived[frame]
    }
}

struct Stepped<'a> {
    deep: Deep,
    board: Board,
    truth_at: &'a dyn Fn(usize) -> Truth,
    faults: Faults,
    frames: usize,
    alert_defs: &'a [AlertTrigger],
    names: Vec<String>,
    area_of: Vec<usize>,
    areas: Vec<&'static str>,
    values: Vec<f64>,
    alerts: Vec<bool>,
    derived: Vec<(u64, f64)>,
}

impl<'a> Stepped<'a> {
    fn new(truth_at: &'a dyn Fn(usize) -> Truth, faults: Faults, frames: usize, alert_defs: &'a [AlertTrigger]) -> Self {
        let deep = fresh_areas();
        let board = with_board_mut(std::mem::take);
        Self { deep, board, truth_at, faults, frames, alert_defs, names: Vec::new(), area_of: Vec::new(), areas: Vec::new(), values: Vec::new(), alerts: Vec::new(), derived: Vec::new() }
    }
}

impl BaseRun for Stepped<'_> {
    fn frames(&self) -> usize {
        self.frames
    }
    fn advance(&mut self, frame: usize) {
        with_board_mut(|b| std::mem::swap(b, &mut self.board));
        let first = frame == 0;
        let (names, values) = (&mut self.names, &mut self.values);
        values.clear();
        self.deep.tick((self.truth_at)(frame), &self.faults, &mut |name, value| {
            if first {
                names.push(name.to_owned());
            }
            values.push(value);
        });
        with_board_mut(|b| std::mem::swap(b, &mut self.board));
        if first {
            for (i, (area, count)) in self.deep.published_counts_by_area().into_iter().enumerate() {
                self.areas.push(area);
                self.area_of.extend(std::iter::repeat(i).take(count));
            }
            if self.area_of.len() != self.names.len() {
                self.area_of = vec![usize::MAX; self.names.len()];
            }
        }
        self.alerts = alert_states(&self.deep, self.alert_defs);
        self.derived = self.deep.derived_failures().iter().map(|d| (d.fbw_id, d.magnitude)).collect();
    }
    fn names(&self) -> &[String] {
        &self.names
    }
    fn area_of(&self) -> &[usize] {
        &self.area_of
    }
    fn areas(&self) -> &[&'static str] {
        &self.areas
    }
    fn values(&self, _frame: usize) -> &[f64] {
        &self.values
    }
    fn alerts(&self, _frame: usize) -> &[bool] {
        &self.alerts
    }
    fn derived(&self, _frame: usize) -> &[(u64, f64)] {
        &self.derived
    }
}

fn trace(base: &mut dyn BaseRun, truth_at: &dyn Fn(usize) -> Truth, faults: &Faults, profile: &'static str, alerts: &[AlertTrigger], setup: &dyn Fn(&mut Deep) -> Option<()>) -> Option<Consequences> {
    let mut deep = fresh_areas();
    setup(&mut deep)?;
    let dt = truth_at(0).dt_s.max(0.0);
    let mut n = 0usize;
    let mut first_moved: Vec<Option<usize>> = Vec::new();
    let mut by_name: Option<HashMap<String, usize>> = None;
    let mut first_any: Option<usize> = None;
    let mut in_place: Vec<bool> = Vec::new();
    let mut held_s = vec![0.0_f64; alerts.len()];
    let mut raised: Vec<usize> = Vec::new();
    let mut fbw: BTreeMap<u64, FbwEffect> = BTreeMap::new();
    let mut faulted_now: Vec<f64> = Vec::new();
    let mut last_frame = 0usize;

    for frame in 0..base.frames() {
        if first_any.is_some_and(|f| frame > f + TAIL_FRAMES) {
            break;
        }
        base.advance(frame);
        if frame == 0 {
            n = base.names().len();
            first_moved = vec![None; n];
            in_place = vec![false; n];
            faulted_now = vec![f64::NAN; n];
        }
        last_frame = frame;
        let names = base.names();
        let expected = base.values(frame);
        let mut i = 0usize;
        let mut off_position: Vec<(String, f64)> = Vec::new();
        in_place.fill(false);
        deep.tick(truth_at(frame), faults, &mut |name, value| {
            if i < n && names[i] == name {
                in_place[i] = true;
                faulted_now[i] = value;
                if first_moved[i].is_none() && !same(expected[i], value) {
                    first_moved[i] = Some(frame);
                }
            } else {
                off_position.push((name.to_owned(), value));
            }
            i += 1;
        });
        if !off_position.is_empty() || i != n {
            let index = by_name.get_or_insert_with(|| names.iter().enumerate().map(|(k, s)| (s.clone(), k)).collect());
            let mut seen = in_place.clone();
            for (name, value) in &off_position {
                if let Some(&k) = index.get(name.as_str()) {
                    seen[k] = true;
                    faulted_now[k] = *value;
                    if first_moved[k].is_none() && !same(expected[k], *value) {
                        first_moved[k] = Some(frame);
                    }
                }
            }
            for (k, slot) in seen.iter().enumerate() {
                if !slot && first_moved[k].is_none() {
                    first_moved[k] = Some(frame);
                }
            }
        }

        let states = alert_states(&deep, alerts);
        for (j, (&now, &healthy)) in states.iter().zip(base.alerts(frame)).enumerate() {
            if now && !healthy {
                held_s[j] += dt;
                if held_s[j] + 1e-9 >= alerts[j].confirm_s && !raised.contains(&j) {
                    raised.push(j);
                }
            } else {
                held_s[j] = 0.0;
            }
        }
        for d in deep.derived_failures() {
            let healthy = base.derived(frame).iter().find(|(id, _)| *id == d.fbw_id).map_or(0.0, |(_, m)| *m);
            if d.magnitude > healthy + 1e-9 {
                fbw.entry(d.fbw_id).or_insert(FbwEffect { fbw_id: d.fbw_id, deep_component: d.deep_component, reason: d.reason });
            }
        }

        if first_any.is_none() && (first_moved.iter().any(Option::is_some) || !raised.is_empty() || !fbw.is_empty()) {
            first_any = Some(frame);
        }
    }

    first_any?;

    let (names, area_of, areas) = (base.names(), base.area_of(), base.areas());
    let mut per_area: BTreeMap<usize, (usize, Vec<(usize, usize)>)> = BTreeMap::new();
    for (k, moved) in first_moved.iter().enumerate() {
        let Some(frame) = *moved else { continue };
        let area = area_of.get(k).copied().unwrap_or(usize::MAX);
        let entry = per_area.entry(area).or_insert((frame, Vec::new()));
        entry.0 = entry.0.min(frame);
        entry.1.push((frame, k));
    }
    let mut frames: Vec<usize> = per_area.values().map(|(f, _)| *f).collect();
    frames.sort_unstable();
    frames.dedup();
    let mut stages: Vec<Vec<AreaEffect>> = vec![Vec::new(); frames.len()];
    for (area, (frame, mut moved)) in per_area {
        moved.sort_unstable();
        let stage = frames.binary_search(&frame).unwrap_or(0);
        stages[stage].push(AreaEffect {
            area: areas.get(area).copied().unwrap_or("unattributed"),
            moved: moved.len(),
            examples: moved.iter().take(EXAMPLES).map(|&(_, k)| names[k].clone()).collect(),
        });
    }
    for stage in &mut stages {
        stage.sort_by(|a, b| b.moved.cmp(&a.moved).then(a.area.cmp(b.area)));
    }

    let healthy_last = base.values(last_frame).to_vec();
    let mut moved_first: Vec<(usize, usize)> = first_moved.iter().enumerate().filter_map(|(k, f)| f.map(|f| (f, k))).collect();
    moved_first.sort_unstable();
    let values = moved_first
        .iter()
        .take(VALUES)
        .map(|&(_, k)| (names[k].clone(), healthy_last.get(k).copied().unwrap_or(f64::NAN), faulted_now.get(k).copied().unwrap_or(f64::NAN)))
        .collect();

    Some(Consequences { profile, stages, alerts: raised.into_iter().map(|j| alerts[j].key.clone()).collect(), fbw: fbw.into_values().collect(), develops_over_s: None, values })
}

#[doc(hidden)]
pub fn stepped_self_check(truth: &Truth, frames: usize) -> Option<Consequences> {
    let faults = Faults::from_pairs([(SENTINEL_ID, 1.0)]);
    let alerts: Vec<AlertTrigger> = Vec::new();
    let recorded = record(truth, &faults, frames, &alerts);
    if let Some(c) = trace(&mut Replay(&recorded), &|_| truth.clone(), &faults, "self_check_replayed", &alerts, &|_| Some(())) {
        return Some(c);
    }
    let truth_at = |_: usize| truth.clone();
    let mut base = Stepped::new(&truth_at, faults.clone(), frames, &alerts);
    trace(&mut base, &truth_at, &faults, "self_check_stepped", &alerts, &|_| Some(()))
}

pub struct Tracer {
    alerts: Vec<AlertTrigger>,
    references: Vec<(Profile, Truth, Recorded)>,
}

impl Tracer {
    pub fn new(registry: &Registry) -> Self {
        let alerts: Vec<AlertTrigger> = registry
            .alerts
            .iter()
            .map(|a| AlertTrigger { key: a.key.clone(), trigger: a.trigger.clone(), confirm_s: a.confirm_s })
            .chain(crate::deep::ecam::fbw::wirings().into_iter().map(|p| AlertTrigger {
                key: format!("{} {}", p.id, p.title),
                trigger: p.trigger,
                confirm_s: p.confirm_s,
            }))
            .collect();
        let reference = Faults::from_pairs([(SENTINEL_ID, 1.0)]);
        let references = profiles()
            .into_iter()
            .map(|p| {
                let truth = (p.truth)();
                let rec = record(&truth, &reference, p.frames, &alerts);
                (p, truth, rec)
            })
            .collect();
        Self { alerts, references }
    }

    pub fn failure(&self, f: &FailureDef, registry: &Registry, dead_in_profiles: bool) -> Option<Consequences> {
        let provocation = provocation::for_failure(f);
        if !(dead_in_profiles && provocation.is_some()) {
            let armed = Faults::from_pairs([(SENTINEL_ID, 1.0), (f.id, 1.0)]);
            for (p, truth, rec) in &self.references {
                if let Some(c) = trace(&mut Replay(rec), &|_| truth.clone(), &armed, p.name, &self.alerts, &|_| Some(())) {
                    return Some(c);
                }
            }
        }
        let provoked = (provocation?.provoke)(f, registry);
        let profile = profiles().into_iter().find(|x| x.name == provoked.profile)?;
        let mut truth = (profile.truth)();
        if let Some(change) = &provoked.truth {
            change(&mut truth);
        }
        let simulated_s = provoked.frames as f64 * truth.dt_s.max(0.0);
        if simulated_s > MAX_TRACE_SIM_S {
            return Some(Consequences { profile: provoked.profile, stages: Vec::new(), alerts: Vec::new(), fbw: Vec::new(), develops_over_s: Some(simulated_s), values: Vec::new() });
        }
        let mut reference: Vec<(u64, f64)> = vec![(SENTINEL_ID, 1.0)];
        reference.extend(provoked.companions.iter().copied().filter(|&(id, _)| id != f.id));
        let truth_at = provoked.truth_at(truth);
        let mut base = Stepped::new(&truth_at, Faults::from_pairs(reference.clone()), provoked.frames, &self.alerts);
        reference.push((f.id, 1.0));
        trace(&mut base, &truth_at, &Faults::from_pairs(reference), provoked.profile, &self.alerts, &|_| Some(()))
    }

    pub fn breaker(&self, id: &str) -> Option<Consequences> {
        let reference = Faults::from_pairs([(SENTINEL_ID, 1.0)]);
        for (p, truth, rec) in self.references.iter().filter(|(p, _, _)| BREAKER_PROFILES.contains(&p.name)) {
            let pull = |deep: &mut Deep| {
                deep.breakers_mut()?.breaker_mut(id)?.pull();
                Some(())
            };
            if let Some(c) = trace(&mut Replay(rec), &|_| truth.clone(), &reference, p.name, &self.alerts, &pull) {
                return Some(c);
            }
        }
        None
    }
}
