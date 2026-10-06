use std::collections::{BTreeMap, BTreeSet, HashMap, VecDeque};
use std::io::Write as _;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;
use std::time::Instant;

use systems::simulation::test::{ReadByName, SimulationTestBed, TestBed};
use systems::simulation::{Aircraft, StartState, VariableIdentifier};

use super::tests_cpp_host::CppHost;
use super::tests_flight_state::{apply_flight_state, write_named};
use crate::A380;

const FLIGHT_STATE: &str = "runway.FLT";
const FRAME_S: f64 = 0.1;
const FRAMES_PER_S: usize = 10;
const WARMUP_S: usize = 10;
const MAX_S: usize = 180;
const MIN_FAILURE_S: usize = 10;
const MIN_BREAKER_S: usize = 5;
const MIN_DEAD_S: usize = 30;
const SETTLE_WINDOW_S: usize = 10;
const SETUP_FRAMES: usize = 3;
const REL_TOL: f64 = 1e-3;
const ABS_TOL: f64 = 1e-6;
const RUNAWAY: f64 = 1e9;
const HUB_SHARE: f64 = 0.05;
const RUN_WALL_LIMIT_S: f64 = 300.0;
const OUT_DIR: &str = "D:/A380/msfs-a380/reports/suite";

#[derive(Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Debug)]
enum Item {
    Failure(u64),
    Breaker(usize),
}

fn fbw_table() -> &'static HashMap<u64, systems::failures::FailureType> {
    static TABLE: std::sync::OnceLock<HashMap<u64, systems::failures::FailureType>> = std::sync::OnceLock::new();
    TABLE.get_or_init(|| crate::fbw_failures().into_iter().collect())
}

fn deep_ids() -> &'static BTreeSet<u64> {
    static IDS: std::sync::OnceLock<BTreeSet<u64>> = std::sync::OnceLock::new();
    IDS.get_or_init(|| deep_systems::deep::registry().failures.iter().map(|f| f.id).collect())
}

#[derive(Clone, Copy, Debug)]
struct Air {
    alt_ft: f64,
    cas_kt: f64,
    mach: f64,
    vs_fpm: f64,
    fmgc_phase: f64,
    pitch_deg: f64,
    seed_n1: f64,
    seed_n2: f64,
    tla_deg: f64,
}

#[derive(Clone, Copy, Debug)]
struct Stage {
    name: &'static str,
    flt: &'static str,
    start: StartState,
    air: Option<Air>,
}

const STAGES: [Stage; 6] = [
    Stage { name: "apron", flt: "apron.FLT", start: StartState::Apron, air: None },
    Stage { name: "runway", flt: "runway.FLT", start: StartState::Runway, air: None },
    Stage { name: "climb", flt: "Climb.flt", start: StartState::Climb, air: Some(Air { alt_ft: 15_000., cas_kt: 300., mach: 0., vs_fpm: 2_000., fmgc_phase: 2., pitch_deg: 7., seed_n1: 88., seed_n2: 94., tla_deg: 25. }) },
    Stage { name: "cruise", flt: "cruise.FLT", start: StartState::Cruise, air: Some(Air { alt_ft: 35_000., cas_kt: 0., mach: 0.85, vs_fpm: 0., fmgc_phase: 3., pitch_deg: 2.5, seed_n1: 86., seed_n2: 92., tla_deg: 25. }) },
    Stage { name: "approach", flt: "approach.FLT", start: StartState::Approach, air: Some(Air { alt_ft: 3_000., cas_kt: 200., mach: 0., vs_fpm: -800., fmgc_phase: 5., pitch_deg: 2., seed_n1: 45., seed_n2: 78., tla_deg: 12. }) },
    Stage { name: "final", flt: "final.FLT", start: StartState::Final, air: Some(Air { alt_ft: 1_000., cas_kt: 140., mach: 0., vs_fpm: -700., fmgc_phase: 5., pitch_deg: 1., seed_n1: 45., seed_n2: 78., tla_deg: 12. }) },
];

struct AirData {
    sat_c: f64,
    pressure_hpa: f64,
    density: f64,
    tas_kt: f64,
    cas_kt: f64,
    mach: f64,
}

fn air_data(air: &Air) -> AirData {
    use systems::shared::InternationalStandardAtmosphere as Isa;
    use uom::si::{f64::Length, length::foot, pressure::hectopascal, thermodynamic_temperature::degree_celsius};
    let alt = Length::new::<foot>(air.alt_ft);
    let p = Isa::pressure_at_altitude(alt).get::<hectopascal>();
    let t = Isa::temperature_at_altitude(alt).get::<degree_celsius>();
    let a_kt = 38.967_854 * (t + 273.15).sqrt();
    let (p0, a0) = (1013.25, 661.478_6);
    let (mach, cas) = if air.mach > 0. {
        let qc = p * ((1. + 0.2 * air.mach * air.mach).powf(3.5) - 1.);
        (air.mach, a0 * (5. * ((qc / p0 + 1.).powf(2. / 7.) - 1.)).sqrt())
    } else {
        let qc = p0 * ((1. + 0.2 * (air.cas_kt / a0).powi(2)).powf(3.5) - 1.);
        ((5. * ((qc / p + 1.).powf(2. / 7.) - 1.)).sqrt(), air.cas_kt)
    };
    AirData { sat_c: t, pressure_hpa: p, density: p * 100. / (287.05 * (t + 273.15)), tas_kt: mach * a_kt, cas_kt: cas, mach }
}

fn apply_air(bench: &mut SimulationTestBed<A380>, host: Option<&mut CppHost>, air: &Air) {
    use super::tests_cpp_host::Family;
    use uom::si::{
        f64::{Length, MassDensity, Pressure, ThermodynamicTemperature, Velocity},
        length::foot,
        mass_density::kilogram_per_cubic_meter,
        pressure::hectopascal,
        thermodynamic_temperature::degree_celsius,
        velocity::{foot_per_minute, knot},
    };
    let d = air_data(air);
    bench.set_on_ground(false);
    bench.set_pressure_altitude(Length::new::<foot>(air.alt_ft));
    bench.set_ambient_pressure(Pressure::new::<hectopascal>(d.pressure_hpa));
    bench.set_ambient_temperature(ThermodynamicTemperature::new::<degree_celsius>(d.sat_c));
    bench.set_ambient_air_density(MassDensity::new::<kilogram_per_cubic_meter>(d.density));
    bench.set_true_airspeed(Velocity::new::<knot>(d.tas_kt));
    bench.set_indicated_airspeed(Velocity::new::<knot>(d.cas_kt));
    bench.set_vertical_speed(Velocity::new::<foot_per_minute>(air.vs_fpm));
    write_named(bench, "FMGC_FLIGHT_PHASE", air.fmgc_phase);
    if let Some(host) = host {
        let tat = (d.sat_c + 273.15) * (1. + 0.2 * d.mach * d.mach) - 273.15;
        host.set_flight_profile(&[
            ("SIM ON GROUND", Family::Plain, 0.),
            ("PLANE ALTITUDE", Family::Length, air.alt_ft),
            ("INDICATED ALTITUDE", Family::Length, air.alt_ft),
            ("PRESSURE ALTITUDE", Family::Length, air.alt_ft),
            ("PLANE ALT ABOVE GROUND", Family::Length, air.alt_ft),
            ("PLANE ALT ABOVE GROUND MINUS CG", Family::Length, air.alt_ft),
            ("RADIO HEIGHT", Family::Length, air.alt_ft),
            ("AIRSPEED INDICATED", Family::Speed, d.cas_kt),
            ("AIRSPEED TRUE", Family::Speed, d.tas_kt),
            ("GROUND VELOCITY", Family::Speed, d.tas_kt),
            ("AIRSPEED MACH", Family::Plain, d.mach),
            ("MACH", Family::Plain, d.mach),
            ("VERTICAL SPEED", Family::VerticalSpeed, air.vs_fpm),
            ("AMBIENT TEMPERATURE", Family::Temperature, d.sat_c),
            ("STANDARD ATM TEMPERATURE", Family::Temperature, d.sat_c),
            ("TOTAL AIR TEMPERATURE", Family::Temperature, tat),
            ("AMBIENT PRESSURE", Family::Pressure, d.pressure_hpa),
            ("AMBIENT DENSITY", Family::Density, d.density),
            ("PLANE PITCH DEGREES", Family::Angle, -air.pitch_deg),
            ("INCIDENCE ALPHA", Family::Angle, air.pitch_deg),
            ("G FORCE", Family::Plain, 1.),
        ]);
    }
}

struct Rig {
    bench: SimulationTestBed<A380>,
    host: CppHost,
    fbw_active: Vec<u64>,
}

impl Rig {
    fn spawn() -> Self {
        Self::spawn_stage(&STAGES[1])
    }

    fn spawn_stage(stage: &Stage) -> Self {
        let mut bench = SimulationTestBed::new_with_start_state(stage.start, A380::new);
        apply_flight_state(&mut bench, stage.flt);
        if let Some(air) = &stage.air {
            apply_air(&mut bench, None, air);
            for n in 1..=4 {
                write_named(&mut bench, &format!("TURB ENG CORRECTED N1:{n}"), air.seed_n1);
                write_named(&mut bench, &format!("TURB ENG CORRECTED N2:{n}"), air.seed_n2);
            }
        }
        let mut host = CppHost::new(&mut bench);
        if let Some(air) = &stage.air {
            apply_air(&mut bench, Some(&mut host), air);
        }
        let mut rig = Self { bench, host, fbw_active: Vec::new() };
        if let Some(air) = &stage.air {
            rig.set_thrust_lever_angle(air.tla_deg);
        }
        rig.frames(WARMUP_S * FRAMES_PER_S);
        rig
    }

    fn frame(&mut self) {
        for id in self.bench.query(|a| a.derived_failure_ids()) {
            if let Some(failure_type) = fbw_table().get(&id) {
                self.bench.fail(*failure_type);
            }
        }
        self.bench.run_with_delta(std::time::Duration::from_secs_f64(FRAME_S));
        self.host.frame(&mut self.bench, FRAME_S);
    }

    fn frames(&mut self, n: usize) {
        for _ in 0..n {
            self.frame();
        }
    }

    fn tla(&self) -> f64 {
        ["AUTOTHRUST_TLA:1", "A32NX_AUTOTHRUST_TLA:1"]
            .iter()
            .find_map(|n| self.bench.known_variable_identifier(n).and_then(|id| self.bench.read_identifier(&id)))
            .unwrap_or(f64::NAN)
    }

    fn set_thrust_lever_angle(&mut self, target_deg: f64) {
        static AXIS: std::sync::OnceLock<Mutex<HashMap<i64, i32>>> = std::sync::OnceLock::new();
        let cache = AXIS.get_or_init(Default::default);
        let key = (target_deg * 10.).round() as i64;
        let known = cache.lock().unwrap_or_else(|e| e.into_inner()).get(&key).copied();
        if let Some(axis) = known {
            self.host.inject_key_event("THROTTLE_AXIS_SET_EX1", axis);
            self.frame();
            return;
        }
        let (mut lo, mut hi) = (-16384i32, 16384i32);
        for _ in 0..16 {
            let mid = lo + (hi - lo) / 2;
            self.host.inject_key_event("THROTTLE_AXIS_SET_EX1", mid);
            self.frame();
            let tla = self.tla();
            if !tla.is_finite() {
                return;
            }
            if tla < target_deg {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        self.host.inject_key_event("THROTTLE_AXIS_SET_EX1", hi);
        self.frame();
        if (self.tla() - target_deg).abs() < 1. {
            cache.lock().unwrap_or_else(|e| e.into_inner()).insert(key, hi);
        }
    }

    fn arm(&mut self, id: u64) -> f64 {
        if !deep_ids().contains(&id) {
            if let Some(failure_type) = fbw_table().get(&id) {
                self.bench.fail(*failure_type);
            }
            self.fbw_active.push(id);
            let payload = format!("[{}]", self.fbw_active.iter().map(|i| i.to_string()).collect::<Vec<_>>().join(","));
            self.host.send_commbus("FBW_FAILURE_UPDATE", &payload);
        }
        write_named(&mut self.bench, "DEEP_FAILURE_CMD_MAGNITUDE", 1.);
        write_named(&mut self.bench, "DEEP_FAILURE_CMD_ID", id as f64);
        self.frame();
        let result: f64 = self.bench.read_by_name("DEEP_FAILURE_CMD_RESULT");
        if deep_ids().contains(&id) || result == 1. {
            result
        } else {
            1.
        }
    }
}

struct Sampler {
    ids: Vec<Option<VariableIdentifier>>,
}

impl Sampler {
    fn new(names: &[String]) -> Self {
        Self { ids: vec![None; names.len()] }
    }

    fn sample(&mut self, bench: &SimulationTestBed<A380>, names: &[String]) -> Vec<f64> {
        self.ids
            .iter_mut()
            .zip(names)
            .map(|(id, name)| {
                if id.is_none() {
                    *id = bench.known_variable_identifier(name);
                }
                id.as_ref().and_then(|i| bench.read_identifier(i)).unwrap_or(f64::NAN)
            })
            .collect()
    }
}

struct Ctx {
    stage: &'static Stage,
    units: Vec<&'static str>,
    names: Vec<String>,
    index: HashMap<String, usize>,
    healthy: Vec<Vec<f64>>,
    noisy: Vec<bool>,
    failure_names: HashMap<u64, String>,
}

impl Ctx {
    fn label(&self, item: Item) -> String {
        match item {
            Item::Failure(id) => format!("F{id} {}", self.failure_names.get(&id).map(String::as_str).unwrap_or("?")),
            Item::Breaker(i) => format!("CB {}", self.units[i]),
        }
    }
}

#[derive(Default, Clone)]
struct Outcome {
    items: Vec<Item>,
    sim_s: usize,
    wall_s: f64,
    ever: BTreeSet<usize>,
    end: BTreeMap<usize, f64>,
    trend: BTreeMap<usize, i8>,
    extra: BTreeMap<String, f64>,
    flags: BTreeMap<String, f64>,
    first_deviation_s: Option<usize>,
    settled: bool,
    arm_results: Vec<f64>,
    non_finite: Vec<String>,
    runaway: Vec<String>,
    panic: Option<String>,
    cpp_trap: Vec<String>,
    fbw_failures: Vec<u64>,
}

impl Outcome {
    fn live(&self) -> bool {
        !self.ever.is_empty() || !self.extra.is_empty()
    }

    fn definite_bug(&self) -> bool {
        self.panic.is_some() || !self.non_finite.is_empty() || !self.runaway.is_empty() || !self.cpp_trap.is_empty()
    }
}

fn trend_of(healthy: f64, value: f64) -> i8 {
    if !healthy.is_finite() || !value.is_finite() {
        return 0;
    }
    let d = value - healthy;
    if d.abs() <= 1e-9 * healthy.abs().max(1.) {
        0
    } else if d > 0. {
        1
    } else {
        -1
    }
}

fn differs(healthy: f64, value: f64) -> bool {
    if healthy.is_nan() || value.is_nan() {
        return healthy.is_nan() != value.is_nan();
    }
    let d = (value - healthy).abs();
    d > ABS_TOL && d > REL_TOL * healthy.abs().max(value.abs())
}

fn healthy_trajectory(names: &[String], stage: &Stage) -> Vec<Vec<f64>> {
    let mut rig = Rig::spawn_stage(stage);
    rig.frames(SETUP_FRAMES);
    let mut sampler = Sampler::new(names);
    let mut out = Vec::with_capacity(MAX_S + 1);
    out.push(sampler.sample(&rig.bench, names));
    for _ in 1..=MAX_S {
        rig.frames(FRAMES_PER_S);
        out.push(sampler.sample(&rig.bench, names));
    }
    out
}

fn run_items(ctx: &Ctx, items: &[Item], horizon_s: usize) -> Outcome {
    let started = Instant::now();
    let mut out = Outcome { items: items.to_vec(), ..Default::default() };
    let only_breakers = items.iter().all(|i| matches!(i, Item::Breaker(_)));
    let min_s = if only_breakers { MIN_BREAKER_S } else { MIN_FAILURE_S };

    let mut rig = Rig::spawn_stage(ctx.stage);
    for item in items {
        if let Item::Breaker(i) = item {
            write_named(&mut rig.bench, &format!("BKR_{}_CMD", deep_systems::lvar_key(ctx.units[*i])), 1.);
        }
    }
    let failures: Vec<u64> = items.iter().filter_map(|i| if let Item::Failure(id) = i { Some(*id) } else { None }).collect();
    for k in 0..SETUP_FRAMES {
        match failures.get(k) {
            Some(id) => out.arm_results.push(rig.arm(*id)),
            None => rig.frame(),
        }
    }

    let mut sampler = Sampler::new(&ctx.names);
    let mut window: VecDeque<BTreeMap<usize, f64>> = VecDeque::with_capacity(SETTLE_WINDOW_S + 1);
    let mut settled_for = 0usize;
    let mut last: Option<(usize, Vec<f64>)> = None;
    for s in 1..=horizon_s.min(MAX_S) {
        if started.elapsed().as_secs_f64() > RUN_WALL_LIMIT_S {
            out.panic = Some(format!("run exceeded {RUN_WALL_LIMIT_S} s of wall time at {s} s simulated"));
            break;
        }
        rig.frames(FRAMES_PER_S);
        let now = sampler.sample(&rig.bench, &ctx.names);
        let healthy = &ctx.healthy[s];
        let mut dev: BTreeMap<usize, f64> = BTreeMap::new();
        for (i, &value) in now.iter().enumerate() {
            if ctx.noisy[i] {
                continue;
            }
            if !value.is_finite() && healthy[i].is_finite() && !value.is_nan() {
                out.non_finite.push(ctx.names[i].clone());
            }
            if value.is_nan() && !healthy[i].is_nan() {
                continue;
            }
            let arinc_word = |v: f64| v >= 0. && v.fract() == 0. && v < 17_179_869_184.;
            if value.abs() > RUNAWAY && healthy[i].abs() < RUNAWAY / 1e3 && !arinc_word(value) {
                out.runaway.push(ctx.names[i].clone());
            }
            if differs(healthy[i], value) {
                dev.insert(i, value);
            }
        }
        if out.first_deviation_s.is_none() && !dev.is_empty() {
            out.first_deviation_s = Some(s);
        }
        out.ever.extend(dev.keys().copied());
        let stable = window.len() == SETTLE_WINDOW_S
            && window.front().is_some_and(|then| then.len() == dev.len() && then.iter().all(|(i, v)| dev.get(i).is_some_and(|w| !differs(*v, *w))));
        settled_for = if stable { settled_for + 1 } else { 0 };
        if window.len() == SETTLE_WINDOW_S {
            window.pop_front();
        }
        window.push_back(dev.clone());
        out.end = dev;
        out.sim_s = s;
        last = Some((s, now));
        let dead = out.ever.is_empty();
        if (dead && s >= MIN_DEAD_S) || (!dead && s >= min_s && settled_for >= 1) {
            out.settled = true;
            break;
        }
    }
    if let Some((s, now)) = &last {
        let healthy = &ctx.healthy[*s];
        out.trend = now
            .iter()
            .enumerate()
            .filter(|(i, _)| !ctx.noisy[*i])
            .filter_map(|(i, v)| match trend_of(healthy[i], *v) {
                0 => None,
                t => Some((i, t)),
            })
            .collect();
    }
    out.non_finite.sort();
    out.non_finite.dedup();
    out.runaway.sort();
    out.runaway.dedup();
    for (name, id) in rig.bench.variable_identifiers() {
        let failure_flag = name.starts_with("DEEP_FAILURE_") && name.ends_with("_ACTIVE");
        if !ctx.index.contains_key(&name) || failure_flag {
            if let Some(v) = rig.bench.read_identifier(&id) {
                if failure_flag {
                    if v > 0. {
                        out.flags.insert(name, v);
                    }
                } else {
                    out.extra.insert(name, v);
                }
            }
        }
    }
    out.cpp_trap = rig.host.failures();
    out.fbw_failures = rig.bench.query(|a| a.derived_failure_ids());
    out.fbw_failures.extend(rig.fbw_active.iter().copied());
    out.fbw_failures.sort();
    out.fbw_failures.dedup();
    out.wall_s = started.elapsed().as_secs_f64();
    out
}

fn run_guarded(ctx: &Ctx, items: &[Item], horizon_s: usize) -> Outcome {
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| run_items(ctx, items, horizon_s))) {
        Ok(o) => o,
        Err(e) => {
            let msg = e.downcast_ref::<String>().cloned().or_else(|| e.downcast_ref::<&str>().map(|s| s.to_string())).unwrap_or_else(|| "panic".into());
            Outcome { items: items.to_vec(), panic: Some(msg), ..Default::default() }
        }
    }
}

fn run_all(ctx: &Ctx, jobs: &[(Vec<Item>, usize)], threads: usize, what: &str) -> Vec<Outcome> {
    let started = Instant::now();
    let next = AtomicUsize::new(0);
    let finished = AtomicUsize::new(0);
    let done = Mutex::new(Vec::<(usize, Outcome)>::with_capacity(jobs.len()));
    std::thread::scope(|s| {
        for _ in 0..threads {
            s.spawn(|| loop {
                let i = next.fetch_add(1, Ordering::Relaxed);
                let Some((job, horizon)) = jobs.get(i) else { break };
                let o = run_guarded(ctx, job, *horizon);
                done.lock().expect("suite results").push((i, o));
                let n = finished.fetch_add(1, Ordering::Relaxed) + 1;
                if n % 500 == 0 || n == jobs.len() {
                    println!("{what}: {n}/{} after {:.0} s", jobs.len(), started.elapsed().as_secs_f64());
                }
            });
        }
    });
    let mut done = done.into_inner().expect("suite results");
    done.sort_by_key(|(i, _)| *i);
    done.into_iter().map(|(_, o)| o).collect()
}

fn family_of(label: &str) -> Option<(&'static str, u8)> {
    const FAMILIES: [&str; 9] = ["PRIM", "SEC", "FCDC", "FCU", "LGCIU", "ADIRU", "ADR", "IR", "FMS"];
    let upper = label.to_uppercase().replace(['-', '_'], " ");
    let words: Vec<&str> = upper.split_whitespace().collect();
    for w in words.windows(2) {
        if let Some(f) = FAMILIES.iter().find(|f| **f == w[0]) {
            if let Ok(n) = w[1].parse::<u8>() {
                if (1..=3).contains(&n) {
                    return Some((f, n));
                }
            }
        }
    }
    None
}

fn msfs_name(name: &str) -> Vec<String> {
    if name.contains(' ') {
        return vec![name.to_string()];
    }
    if name.starts_with("A32NX_") || name.starts_with("A380X_") || name.starts_with("XMLVAR") {
        return vec![format!("L:{name}")];
    }
    vec![format!("L:A32NX_{name}"), format!("L:{name}")]
}

#[derive(Default)]
struct ComboStats {
    candidates: usize,
    run: usize,
    with_emergent: usize,
    with_masked: usize,
    bugs: usize,
}

fn compare_combo(single: &HashMap<Item, &Outcome>, combo: &Outcome, hubs: &BTreeSet<usize>) -> (Vec<usize>, Vec<usize>) {
    let mut predicted: BTreeSet<usize> = BTreeSet::new();
    let mut ended: BTreeSet<usize> = BTreeSet::new();
    for item in &combo.items {
        if let Some(o) = single.get(item) {
            predicted.extend(o.ever.iter().copied());
            ended.extend(o.end.keys().copied());
        }
    }
    let parts: Vec<&Outcome> = combo.items.iter().filter_map(|item| single.get(item).copied()).collect();
    let nudged = |i: usize, sign: i8| parts.iter().any(|o| o.trend.get(&i).is_some_and(|t| sign == 0 || *t == sign));
    let cancelled = |i: usize| parts.iter().any(|o| o.trend.get(&i) == Some(&1)) && parts.iter().any(|o| o.trend.get(&i) == Some(&-1));
    let emergent: Vec<usize> = combo
        .ever
        .iter()
        .filter(|i| !predicted.contains(i) && !hubs.contains(i))
        .filter(|i| !nudged(**i, combo.trend.get(i).copied().unwrap_or(0)))
        .copied()
        .collect();
    let masked: Vec<usize> = ended.iter().filter(|i| !combo.ever.contains(i) && !hubs.contains(i) && !cancelled(**i)).copied().collect();
    (emergent, masked)
}

fn env_usize(name: &str, default: usize) -> usize {
    std::env::var(name).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

#[test]
#[ignore]
fn failure_and_breaker_suite() {
    deep_systems::set_log_quiet(true);
    systems::shared::use_nominal_values(true);
    let started = Instant::now();
    let threads = env_usize("SUITE_THREADS", std::thread::available_parallelism().map(|n| n.get()).unwrap_or(4));
    let pair_budget = env_usize("SUITE_PAIR_BUDGET", 2000);
    let limit = env_usize("SUITE_LIMIT", usize::MAX);
    let plan_only = std::env::var("SUITE_PLAN_ONLY").is_ok();
    std::fs::create_dir_all(format!("{OUT_DIR}/fws")).ok();

    let probe = Rig::spawn();
    let units: Vec<&'static str> = probe.bench.query(|a| a.deep_systems.deep.unit_ids());
    let names: Vec<String> = {
        let mut n: Vec<String> = probe.bench.variable_identifiers().into_iter().map(|(n, _)| n).collect();
        n.sort();
        n
    };
    drop(probe);
    let registry = deep_systems::deep::registry();
    let catalogue: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string("D:/A380/msfs-a380/install/out/EFB/catalogue.json").expect("the EFB catalogue is built")).expect("catalogue json");
    let mut failure_names: HashMap<u64, String> = registry.failures.iter().map(|f| (f.id, f.name.clone())).collect();
    let mut failures: Vec<u64> = Vec::new();
    for f in catalogue["failures"].as_array().into_iter().flatten() {
        let Some(id) = f["id"].as_u64() else { continue };
        if deep_systems::msfs_excluded::is_msfs_excluded(id) {
            continue;
        }
        failure_names.entry(id).or_insert_with(|| f["name"].as_str().unwrap_or_default().to_string());
        failures.push(id);
    }
    failures.sort();
    failures.dedup();

    let (healthy, other) = std::thread::scope(|s| {
        let a = s.spawn(|| healthy_trajectory(&names, &STAGES[1]));
        let b = s.spawn(|| healthy_trajectory(&names, &STAGES[1]));
        (a.join().expect("healthy run"), b.join().expect("healthy run"))
    });
    let bookkeeping = |n: &str| n.starts_with("DEEP_FAILURE") || n.ends_with("_CMD") || n.contains("_ARMED_") || n == "DEEP_SYSTEMS_TICK_US";
    let noisy: Vec<bool> = (0..names.len())
        .map(|i| {
            bookkeeping(&names[i]) || healthy.iter().zip(&other).any(|(x, y)| x[i].to_bits() != y[i].to_bits() && !(x[i].is_nan() && y[i].is_nan()))
        })
        .collect();
    let index: HashMap<String, usize> = names.iter().enumerate().map(|(i, n)| (n.clone(), i)).collect();
    println!(
        "healthy reference ({FLIGHT_STATE}, Rust systems + deep + fbw/fadec/extra-backend wasm): {} variables, {} noisy, {:.0} s",
        names.len(),
        noisy.iter().filter(|n| **n).count(),
        started.elapsed().as_secs_f64()
    );
    {
        let mut lines = String::new();
        for (i, n) in names.iter().enumerate().filter(|(i, _)| noisy[*i]) {
            let first = healthy.iter().zip(&other).position(|(x, y)| x[i].to_bits() != y[i].to_bits()).unwrap_or(0);
            lines.push_str(&format!("{n}	first differs at {first} s: {} vs {}
", healthy[first][i], other[first][i]));
        }
        std::fs::write(format!("{OUT_DIR}/noisy.txt"), lines).ok();
    }
    let ctx = Ctx { stage: &STAGES[1], units, names, index, healthy, noisy, failure_names };

    let mut single_items: Vec<Item> = failures.iter().map(|&id| Item::Failure(id)).collect();
    single_items.extend((0..ctx.units.len()).map(Item::Breaker));
    if let Ok(only) = std::env::var("SUITE_ONLY") {
        let wanted: BTreeSet<String> = only.split(',').map(|s| s.trim().to_string()).collect();
        single_items.retain(|i| match i {
            Item::Failure(id) => wanted.contains(&id.to_string()),
            Item::Breaker(b) => wanted.contains(ctx.units[*b]),
        });
    }
    single_items.truncate(limit);
    let n_f = single_items.iter().filter(|i| matches!(i, Item::Failure(_))).count();
    let n_b = single_items.len() - n_f;

    let mut family: BTreeMap<&'static str, BTreeMap<u8, Vec<Item>>> = BTreeMap::new();
    for item in &single_items {
        let label = match item {
            Item::Failure(id) => ctx.failure_names.get(id).cloned().unwrap_or_default(),
            Item::Breaker(i) => ctx.units[*i].to_string(),
        };
        let Some((fam, n)) = family_of(&label) else { continue };
        let main_failure = matches!(item, Item::Failure(_)) && label.trim().eq_ignore_ascii_case(&format!("{fam} {n}"));
        let normal_breaker = matches!(item, Item::Breaker(_)) && !label.contains("2nd");
        if main_failure || normal_breaker {
            family.entry(fam).or_default().entry(n).or_default().push(*item);
        }
    }
    let mut dependent_jobs: Vec<Vec<Item>> = Vec::new();
    for members in family.values() {
        let idx: Vec<&Vec<Item>> = members.values().collect();
        if idx.len() == 3 {
            for x in idx[0] {
                for y in idx[1] {
                    for z in idx[2] {
                        let items = vec![*x, *y, *z];
                        dependent_jobs.push(items);
                    }
                }
            }
        } else if idx.len() == 2 {
            for x in idx[0] {
                for y in idx[1] {
                    let items = vec![*x, *y];
                    dependent_jobs.push(items);
                }
            }
        }
    }

    let calibration: Vec<(Vec<Item>, usize)> = single_items.iter().step_by((single_items.len() / 96).max(1)).map(|i| (vec![*i], MAX_S)).collect();
    if plan_only {
        let t = Instant::now();
        let sample = run_all(&ctx, &calibration, threads, "calibration");
        let wall = t.elapsed().as_secs_f64();
        let per_run_thread = sample.iter().map(|o| o.wall_s).sum::<f64>() / sample.len().max(1) as f64;
        let avg_sim = sample.iter().map(|o| o.sim_s as f64).sum::<f64>() / sample.len().max(1) as f64;
        let live = sample.iter().filter(|o| o.live()).count() as f64 / sample.len().max(1) as f64;
        let throughput = sample.len() as f64 / wall;
        let full_pairs = (n_f * n_b) as f64 + (n_f * n_f.saturating_sub(1) / 2) as f64 + (n_b * n_b.saturating_sub(1) / 2) as f64;
        let mut plan = format!(
            "# Suite plan (nothing but a {}-run calibration was executed)\n\n\
             - stack: FBW Rust systems + deep model + shipped fbw.wasm / fadec-a380x.wasm / extra-backend-a380x.wasm, spawned from {FLIGHT_STATE} with StartState::Runway, {WARMUP_S} s warm-up, 100 ms frames, {threads} threads\n\
             - variables compared each second: {} ({} excluded: non-deterministic or arming bookkeeping)\n\
             - singles: {n_f} failures + {n_b} breakers = {}\n\
             - calibration: {:.1} s simulated per run on average (pruned when settled), {:.2} s wall per run per thread, {:.1} runs/s on {threads} threads, {:.0}% of sampled items had an effect\n\
             - singles estimate: {:.0} min\n\
             - full pair space: {n_f}x{n_b} = {} (1F+1CB), C({n_f},2) = {} (2F), C({n_b},2) = {} (2CB), total {:.0}; brute force would take {:.0} days\n\
             - pairs actually run: only those whose single-run footprints overlap (outside hub variables), most-coupled first, up to SUITE_PAIR_BUDGET = {pair_budget} per category; pair horizon = slower single's settle time + {SETTLE_WINDOW_S} s\n\
             - pair estimate at that budget: {:.0} min (3 x {pair_budget} runs)\n",
            sample.len(),
            ctx.names.len(),
            ctx.noisy.iter().filter(|n| **n).count(),
            single_items.len(),
            avg_sim,
            per_run_thread,
            throughput,
            live * 100.,
            single_items.len() as f64 / throughput / 60.,
            n_f * n_b,
            n_f * n_f.saturating_sub(1) / 2,
            n_b * n_b.saturating_sub(1) / 2,
            full_pairs,
            full_pairs / throughput / 86_400.,
            3. * pair_budget as f64 / throughput / 60.,
        );
        let mut family_count = 0usize;
        let mut fam: BTreeMap<&'static str, BTreeMap<u8, usize>> = BTreeMap::new();
        for item in &single_items {
            let label = match item {
                Item::Failure(id) => ctx.failure_names.get(id).cloned().unwrap_or_default(),
                Item::Breaker(i) => ctx.units[*i].to_string(),
            };
            if let Some((f, n)) = family_of(&label) {
                family_count += 1;
                *fam.entry(f).or_default().entry(n).or_default() += 1;
            }
        }
        let full_dependent: usize = fam.values().filter(|m| m.len() >= 2).map(|m| m.values().product::<usize>()).sum();
        plan.push_str(&format!(
            "- redundant families (members per unit): {:?}\n- dependent runs, each computer's own failure + its normal-supply breaker: {} ({:.1} min)\n- dependent runs, every failure mode of every unit combined across units: {} ({:.0} min)\n",
            fam,
            dependent_jobs.len(),
            dependent_jobs.len() as f64 / throughput / 60.,
            full_dependent,
            full_dependent as f64 / throughput / 60.
        ));
        let _ = family_count;
        std::fs::write(format!("{OUT_DIR}/PLAN.md"), &plan).ok();
        println!("{plan}");
        return;
    }

    let single_jobs: Vec<(Vec<Item>, usize)> = single_items.iter().map(|i| (vec![*i], MAX_S)).collect();
    let singles = run_all(&ctx, &single_jobs, threads, "singles");
    let single: HashMap<Item, &Outcome> = singles.iter().map(|o| (o.items[0], o)).collect();

    let live: Vec<&Outcome> = singles.iter().filter(|o| o.live()).collect();
    let mut var_count: HashMap<usize, usize> = HashMap::new();
    for o in &live {
        for i in &o.ever {
            *var_count.entry(*i).or_default() += 1;
        }
    }
    let hub_threshold = ((live.len() as f64) * HUB_SHARE).ceil().max(2.) as usize;
    let hubs: BTreeSet<usize> = var_count.iter().filter(|(_, c)| **c >= hub_threshold).map(|(i, _)| *i).collect();
    let sig: HashMap<Item, BTreeSet<usize>> = live.iter().map(|o| (o.items[0], o.ever.iter().filter(|i| !hubs.contains(i)).copied().collect())).collect();
    let live_f: Vec<Item> = live.iter().map(|o| o.items[0]).filter(|i| matches!(i, Item::Failure(_))).collect();
    let live_b: Vec<Item> = live.iter().map(|o| o.items[0]).filter(|i| matches!(i, Item::Breaker(_))).collect();
    let mut by_var: HashMap<usize, Vec<Item>> = HashMap::new();
    for (item, s) in &sig {
        for v in s {
            by_var.entry(*v).or_default().push(*item);
        }
    }
    let overlapping = |a: &[Item], b: &[Item], same: bool| -> Vec<(Item, Item, usize)> {
        let b_set: BTreeSet<Item> = b.iter().copied().collect();
        let mut out: BTreeMap<(Item, Item), usize> = BTreeMap::new();
        for x in a {
            let Some(sx) = sig.get(x) else { continue };
            for v in sx {
                for y in by_var.get(v).into_iter().flatten() {
                    if !b_set.contains(y) || x == y || (same && y < x) {
                        continue;
                    }
                    *out.entry((*x, *y)).or_default() += 1;
                }
            }
        }
        let mut v: Vec<(Item, Item, usize)> = out.into_iter().map(|((x, y), c)| (x, y, c)).collect();
        v.sort_by(|p, q| q.2.cmp(&p.2).then(p.0.cmp(&q.0)).then(p.1.cmp(&q.1)));
        v
    };
    let horizon = |items: &[Item]| -> usize {
        let slowest = items.iter().filter_map(|i| single.get(i)).map(|o| o.sim_s).max().unwrap_or(MAX_S);
        (slowest + SETTLE_WINDOW_S).min(MAX_S)
    };
    let categories: [(&str, Vec<(Item, Item, usize)>, usize); 3] = [
        ("1 failure + 1 breaker", overlapping(&live_f, &live_b, false), n_f * n_b),
        ("2 failures", overlapping(&live_f, &live_f, true), n_f * n_f.saturating_sub(1) / 2),
        ("2 breakers", overlapping(&live_b, &live_b, true), n_b * n_b.saturating_sub(1) / 2),
    ];

    let mut combos: Vec<(String, Outcome, Vec<usize>, Vec<usize>)> = Vec::new();
    let mut stats: Vec<(String, ComboStats, usize)> = Vec::new();
    for (name, candidates, space) in categories {
        let take = candidates.len().min(pair_budget);
        let jobs: Vec<(Vec<Item>, usize)> = candidates.iter().take(take).map(|(a, b, _)| (vec![*a, *b], horizon(&[*a, *b]))).collect();
        let outcomes = run_all(&ctx, &jobs, threads, name);
        let mut st = ComboStats { candidates: candidates.len(), run: outcomes.len(), ..Default::default() };
        for o in outcomes {
            let (emergent, masked) = compare_combo(&single, &o, &hubs);
            st.with_emergent += usize::from(!emergent.is_empty());
            st.with_masked += usize::from(!masked.is_empty());
            st.bugs += usize::from(o.definite_bug());
            combos.push((name.to_string(), o, emergent, masked));
        }
        if take < candidates.len() {
            println!("{name}: ran the {take} most-coupled of {} interacting pairs; {} left unrun", candidates.len(), candidates.len() - take);
        }
        stats.push((name.to_string(), st, space));
    }
    let dependent_jobs: Vec<(Vec<Item>, usize)> = dependent_jobs.into_iter().map(|items| { let h = horizon(&items); (items, h) }).collect();
    let dependent = run_all(&ctx, &dependent_jobs, threads, "redundant families");
    let mut dep_st = ComboStats { candidates: dependent_jobs.len(), run: dependent.len(), ..Default::default() };
    for o in dependent {
        let (emergent, masked) = compare_combo(&single, &o, &hubs);
        dep_st.with_emergent += usize::from(!emergent.is_empty());
        dep_st.with_masked += usize::from(!masked.is_empty());
        dep_st.bugs += usize::from(o.definite_bug());
        combos.push(("redundant family".to_string(), o, emergent, masked));
    }

    let describe = |o: &Outcome| o.items.iter().map(|i| ctx.label(*i)).collect::<Vec<_>>().join(" + ");
    {
        let reference_s = 30usize.min(MAX_S);
        let mut base = serde_json::Map::new();
        for (i, name) in ctx.names.iter().enumerate() {
            let v = ctx.healthy[reference_s][i];
            if v.is_finite() {
                for m in msfs_name(name) {
                    base.insert(m, serde_json::json!(v));
                }
            }
        }
        std::fs::write(format!("{OUT_DIR}/fws/base.json"), serde_json::to_string(&base).unwrap_or_default()).ok();
        let mut f = std::io::BufWriter::new(std::fs::File::create(format!("{OUT_DIR}/fws/runs.jsonl")).expect("runs.jsonl"));
        for o in singles.iter().chain(combos.iter().map(|c| &c.1)) {
            let mut end = serde_json::Map::new();
            for (i, v) in &o.end {
                if v.is_finite() {
                    for m in msfs_name(&ctx.names[*i]) {
                        end.insert(m, serde_json::json!(v));
                    }
                }
            }
            for (name, v) in &o.extra {
                if v.is_finite() {
                    for m in msfs_name(name) {
                        end.insert(m, serde_json::json!(v));
                    }
                }
            }
            let line = serde_json::json!({ "label": describe(o), "fbw": o.fbw_failures, "end": end });
            let _ = writeln!(f, "{line}");
        }
    }

    let total_runs = singles.len() + stats.iter().map(|s| s.1.run).sum::<usize>() + dep_st.run;
    let avg_sim = singles.iter().map(|o| o.sim_s as f64).sum::<f64>() / singles.len().max(1) as f64;
    let avg_wall = singles.iter().map(|o| o.wall_s).sum::<f64>() / singles.len().max(1) as f64;
    let full_3min = singles.iter().filter(|o| o.sim_s >= MAX_S).count();
    let dead_f = singles.iter().filter(|o| matches!(o.items[0], Item::Failure(_)) && !o.live()).count();
    let dead_b = singles.iter().filter(|o| matches!(o.items[0], Item::Breaker(_)) && !o.live()).count();
    let rejected: Vec<&Outcome> = singles.iter().filter(|o| o.arm_results.iter().any(|r| *r != 1.)).collect();
    let single_bugs: Vec<&Outcome> = singles.iter().filter(|o| o.definite_bug()).collect();
    let var_list = |v: &[usize], n: usize| v.iter().take(n).map(|i| ctx.names[*i].as_str()).collect::<Vec<_>>().join(", ");

    let mut report = format!(
        "# Failure and breaker suite\n\nHeadless: FBW Rust systems + deep model + shipped fbw.wasm / fadec-a380x.wasm / extra-backend-a380x.wasm, spawned from {FLIGHT_STATE} (StartState::Runway), 100 ms frames, {threads} threads. ECAM and displays: see FWS.md (replay of these end states through FwsCore).\n\n\
         Total wall time {:.0} s. {total_runs} runs.\n\n\
         ## Singles\n\n- {n_f} failures, {n_b} breakers\n- live: {} failures, {} breakers; no effect within {MIN_DEAD_S} s: {dead_f} failures, {dead_b} breakers\n- not armable on the bench: {}\n- average simulated time {avg_sim:.1} s ({full_3min} needed the full {MAX_S} s), average wall time {avg_wall:.2} s per run per thread\n- definite bugs (panic, C++ trap, non-finite, runaway): {}\n- hub variables excluded from coupling (moved by >= {hub_threshold} singles): {}\n\n",
        started.elapsed().as_secs_f64(),
        live_f.len(),
        live_b.len(),
        rejected.len(),
        single_bugs.len(),
        hubs.len()
    );
    report.push_str("## Combinations\n\n| category | full space | interacting | run | emergent | masked | definite bugs | predicted by superposition |\n|---|---|---|---|---|---|---|---|\n");
    for (name, st, space) in &stats {
        report.push_str(&format!("| {name} | {space} | {} | {} | {} | {} | {} | {} |\n", st.candidates, st.run, st.with_emergent, st.with_masked, st.bugs, space.saturating_sub(st.candidates)));
    }
    report.push_str(&format!(
        "| redundant families | {} | {} | {} | {} | {} | {} | - |\n\n",
        dep_st.candidates, dep_st.candidates, dep_st.run, dep_st.with_emergent, dep_st.with_masked, dep_st.bugs
    ));
    for (name, st, _) in &stats {
        if st.run > 0 && st.candidates > st.run {
            let rest = st.candidates - st.run;
            report.push_str(&format!(
                "- {name}: {rest} interacting pairs unrun; at the sampled rates about {:.0} would show emergent effects and {:.0} definite bugs\n",
                rest as f64 * st.with_emergent as f64 / st.run as f64,
                rest as f64 * st.bugs as f64 / st.run as f64
            ));
        }
    }
    report.push_str("\n## Definite bugs\n\n");
    for o in single_bugs.iter().copied().chain(combos.iter().map(|c| &c.1).filter(|o| o.definite_bug())) {
        report.push_str(&format!(
            "- {}: panic {:?}; C++ {:?}; non-finite {:?}; runaway {:?}\n",
            describe(o),
            o.panic,
            o.cpp_trap,
            o.non_finite.iter().take(6).collect::<Vec<_>>(),
            o.runaway.iter().take(6).collect::<Vec<_>>()
        ));
    }
    report.push_str("\n## Not armable on the bench\n\n");
    for o in &rejected {
        report.push_str(&format!("- {}: result {:?}\n", describe(o), o.arm_results));
    }
    report.push_str("\n## Redundant families\n\n");
    for (_, o, emergent, masked) in combos.iter().filter(|c| c.0 == "redundant family") {
        report.push_str(&format!("- {} ({} s): {} emergent [{}], {} masked [{}]\n", describe(o), o.sim_s, emergent.len(), var_list(emergent, 8), masked.len(), var_list(masked, 5)));
    }
    let mut ranked: Vec<&(String, Outcome, Vec<usize>, Vec<usize>)> = combos.iter().filter(|c| c.0 != "redundant family" && (!c.2.is_empty() || !c.3.is_empty())).collect();
    ranked.sort_by(|p, q| (q.2.len() + q.3.len()).cmp(&(p.2.len() + p.3.len())));
    report.push_str("\n## Combinations that are not the sum of their parts (most first)\n\n");
    for (cat, o, emergent, masked) in ranked.iter().take(300).copied() {
        report.push_str(&format!("- [{cat}] {} ({} s): {} emergent [{}], {} masked [{}]\n", describe(o), o.sim_s, emergent.len(), var_list(emergent, 6), masked.len(), var_list(masked, 4)));
    }

    let mut singles_json = serde_json::Map::new();
    for o in &singles {
        singles_json.insert(
            describe(o),
            serde_json::json!({
                "sim_s": o.sim_s,
                "wall_s": o.wall_s,
                "first_deviation_s": o.first_deviation_s,
                "settled": o.settled,
                "arm": o.arm_results,
                "moved": o.ever.iter().map(|i| ctx.names[*i].clone()).collect::<Vec<_>>(),
                "new_vars": o.extra.keys().collect::<Vec<_>>(),
                "fbw_failures": o.fbw_failures,
                "panic": o.panic,
                "cpp": o.cpp_trap,
                "non_finite": o.non_finite,
                "runaway": o.runaway,
            }),
        );
    }
    std::fs::write(format!("{OUT_DIR}/suite-singles.json"), serde_json::to_string(&singles_json).unwrap_or_default()).ok();
    std::fs::write(format!("{OUT_DIR}/SUITE.md"), &report).ok();
    println!("{}", report.lines().take(40).collect::<Vec<_>>().join("\n"));
}

#[test]
#[ignore]
fn suite_speed_profile() {
    deep_systems::set_log_quiet(true);
    systems::shared::use_nominal_values(true);
    let frames = 300;
    let t = Instant::now();
    let mut rig = Rig::spawn();
    let spawn_s = t.elapsed().as_secs_f64();
    let (mut rust, mut cpp, mut derived, mut deep) = (0f64, 0f64, 0f64, 0f64);
    for _ in 0..frames {
        let a = Instant::now();
        for id in rig.bench.query(|a| a.derived_failure_ids()) {
            if let Some(failure_type) = fbw_table().get(&id) {
                rig.bench.fail(*failure_type);
            }
        }
        let b = Instant::now();
        rig.bench.run_with_delta(std::time::Duration::from_secs_f64(FRAME_S));
        let c = Instant::now();
        rig.host.frame(&mut rig.bench, FRAME_S);
        let d = Instant::now();
        derived += (b - a).as_secs_f64();
        let tick_us: f64 = rig.bench.read_by_name("DEEP_SYSTEMS_TICK_US");
        deep += tick_us / 1e6;
        rust += (c - b).as_secs_f64();
        cpp += (d - c).as_secs_f64();
    }
    let names: Vec<String> = rig.bench.variable_identifiers().into_iter().map(|(n, _)| n).collect();
    let mut sampler = Sampler::new(&names);
    let s = Instant::now();
    for _ in 0..30 {
        let _ = sampler.sample(&rig.bench, &names);
    }
    let sample_ms = s.elapsed().as_secs_f64() * 1e3 / 30.;
    println!(
        "single thread: spawn+{WARMUP_S}s warm-up {:.2} s; per frame: rust systems+deep {:.2} ms (deep host {:.2} ms), C++ wasm {:.2} ms, derived failures {:.3} ms; sample {:.2} ms",
        spawn_s,
        rust * 1e3 / frames as f64,
        deep * 1e3 / frames as f64,
        cpp * 1e3 / frames as f64,
        derived * 1e3 / frames as f64,
        sample_ms
    );
    let t = Instant::now();
    let bare = SimulationTestBed::new_with_start_state(StartState::Runway, A380::new);
    println!("aircraft construction alone {:.3} s", t.elapsed().as_secs_f64());
    drop(bare);
    let t = Instant::now();
    let mut bench = SimulationTestBed::new_with_start_state(StartState::Runway, A380::new);
    apply_flight_state(&mut bench, FLIGHT_STATE);
    let built = t.elapsed().as_secs_f64();
    let _host = CppHost::new(&mut bench);
    println!("construction+FLT {:.3} s, C++ instantiate {:.3} s", built, t.elapsed().as_secs_f64() - built);

    for threads in [1usize, 8, 16, 24, 32] {
        let t = Instant::now();
        std::thread::scope(|s| {
            for _ in 0..threads {
                s.spawn(|| {
                    let mut rig = Rig::spawn();
                    rig.frames(200);
                });
            }
        });
        let wall = t.elapsed().as_secs_f64();
        let frames_total = threads * (200 + WARMUP_S * FRAMES_PER_S);
        println!("{threads:>2} threads: {:.0} frames/s total, {:.2} ms per frame per thread", frames_total as f64 / wall, wall * 1e3 / (200 + WARMUP_S * FRAMES_PER_S) as f64);
    }
}

#[test]
#[ignore]
fn deep_area_profile() {
    deep_systems::set_log_quiet(true);
    systems::shared::use_nominal_values(true);
    let rig = Rig::spawn();
    let truth = rig.bench.query(|a| a.deep_systems.last_truth.clone()).expect("the host has ticked");
    let mut deep = deep_systems::deep::integration::failure_audit::fresh_areas();
    let faults = deep_systems::deep::integration::failure_audit::reference_faults();
    for _ in 0..20 {
        deep.tick(truth.clone(), &faults, &mut |_, _| {});
    }
    let frames = 200;
    let mut totals: BTreeMap<&'static str, (f64, f64)> = BTreeMap::new();
    let mut published = 0usize;
    let t = Instant::now();
    for _ in 0..frames {
        for (name, tick, publish) in deep.tick_timed(truth.clone(), &faults, &mut |_, _| published += 1) {
            let e = totals.entry(name).or_default();
            e.0 += tick.as_secs_f64();
            e.1 += publish.as_secs_f64();
        }
    }
    let wall = t.elapsed().as_secs_f64() * 1e3 / frames as f64;
    let clone = {
        let t = Instant::now();
        for _ in 0..frames {
            std::hint::black_box(truth.clone());
        }
        t.elapsed().as_secs_f64() * 1e3 / frames as f64
    };
    let mut rows: Vec<(&str, f64, f64)> = totals.into_iter().map(|(n, (a, b))| (n, a * 1e3 / frames as f64, b * 1e3 / frames as f64)).collect();
    rows.sort_by(|a, b| (b.1 + b.2).total_cmp(&(a.1 + a.2)));
    println!("deep areas alone: {wall:.3} ms per frame, {} values published per frame, Truth clone {clone:.3} ms", published / frames);
    for (n, a, b) in rows {
        println!("  {n:<28} tick {a:.3} ms  publish {b:.3} ms");
    }
}

#[test]
#[ignore]
fn failure_magnitude_kinds() {
    let reg = deep_systems::deep::registry();
    let live: Vec<_> = reg.failures.iter().filter(|f| !deep_systems::msfs_excluded::is_msfs_excluded(f.id)).collect();
    let binary = live
        .iter()
        .filter(|f| {
            let m = f.magnitude.to_lowercase();
            m.is_empty() || m.contains("bool") || m.contains("on/off") || m.contains("binary") || m.starts_with("1 ") || m == "1" || m.contains("present") && !m.contains("0..1")
        })
        .count();
    let continuous = live.iter().filter(|f| f.magnitude.contains("0..1") || f.magnitude.contains("0 ..") || f.magnitude.contains("0-1")).count();
    println!("deep failures offered: {}, continuous 0..1: {continuous}, binary-looking: {binary}, other: {}", live.len(), live.len() - continuous - binary);
}

struct CampaignDir;

impl std::fmt::Display for CampaignDir {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        static DIR: std::sync::OnceLock<String> = std::sync::OnceLock::new();
        f.write_str(DIR.get_or_init(|| std::env::var("CAMPAIGN_DIR").unwrap_or_else(|_| "E:/test1results".to_string())))
    }
}

const CAMPAIGN_DIR: CampaignDir = CampaignDir;
const PILOT_PER_GROUP: usize = 2401;

fn lock<T>(m: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

fn guarded<R>(what: &str, board: &Mutex<Board>, f: impl FnOnce() -> R) -> Option<R> {
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)) {
        Ok(r) => Some(r),
        Err(e) => {
            let msg = e.downcast_ref::<String>().cloned().or_else(|| e.downcast_ref::<&str>().map(|s| s.to_string())).unwrap_or_else(|| "panic".into());
            let mut b = lock(board);
            b.harness_errors += 1;
            b.finding(format!("HARNESS [{what}] {msg}"));
            None
        }
    }
}

fn prepare(stage: &'static Stage) -> (Ctx, Vec<Item>) {
    let probe = Rig::spawn_stage(stage);
    let units: Vec<&'static str> = probe.bench.query(|a| a.deep_systems.deep.unit_ids());
    let mut names: Vec<String> = probe.bench.variable_identifiers().into_iter().map(|(n, _)| n).collect();
    names.sort();
    drop(probe);
    let registry = deep_systems::deep::registry();
    let catalogue: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string("D:/A380/msfs-a380/install/out/EFB/catalogue.json").expect("the EFB catalogue is built")).expect("catalogue json");
    let mut failure_names: HashMap<u64, String> = registry.failures.iter().map(|f| (f.id, f.name.clone())).collect();
    let mut failures: Vec<u64> = Vec::new();
    for f in catalogue["failures"].as_array().into_iter().flatten() {
        let Some(id) = f["id"].as_u64() else { continue };
        if deep_systems::msfs_excluded::is_msfs_excluded(id) {
            continue;
        }
        failure_names.entry(id).or_insert_with(|| f["name"].as_str().unwrap_or_default().to_string());
        failures.push(id);
    }
    failures.sort();
    failures.dedup();
    let (healthy, other) = std::thread::scope(|s| {
        let a = s.spawn(|| healthy_trajectory(&names, stage));
        let b = s.spawn(|| healthy_trajectory(&names, stage));
        (a.join().expect("healthy run"), b.join().expect("healthy run"))
    });
    let bookkeeping = |n: &str| n.starts_with("DEEP_FAILURE") || n.ends_with("_CMD") || n.contains("_ARMED_") || n == "DEEP_SYSTEMS_TICK_US";
    let noisy: Vec<bool> = (0..names.len())
        .map(|i| bookkeeping(&names[i]) || healthy.iter().zip(&other).any(|(x, y)| x[i].to_bits() != y[i].to_bits() && !(x[i].is_nan() && y[i].is_nan())))
        .collect();
    let index: HashMap<String, usize> = names.iter().enumerate().map(|(i, n)| (n.clone(), i)).collect();
    let mut items: Vec<Item> = failures.iter().map(|&id| Item::Failure(id)).collect();
    items.extend((0..units.len()).map(Item::Breaker));
    (Ctx { stage, units, names, index, healthy, noisy, failure_names }, items)
}

fn splitmix(mut x: u64) -> u64 {
    x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = x;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

fn wilson(k: usize, n: usize) -> (f64, f64, f64) {
    if n == 0 {
        return (0., 0., 1.);
    }
    let z = 1.959_963_984_540_054_f64;
    let (k, n) = (k as f64, n as f64);
    let p = k / n;
    let d = 1. + z * z / n;
    let c = (p + z * z / (2. * n)) / d;
    let h = z * ((p * (1. - p) / n) + z * z / (4. * n * n)).sqrt() / d;
    (p, (c - h).max(0.), (c + h).min(1.))
}

#[derive(Default)]
struct Board {
    phase: String,
    phase_done: usize,
    phase_total: usize,
    phases: Vec<serde_json::Value>,
    runs: usize,
    bugs: usize,
    emergent: usize,
    masked: usize,
    singles_live: usize,
    singles_dead: usize,
    harness_errors: usize,
    recent: VecDeque<String>,
    groups: Vec<serde_json::Value>,
    note: String,
}

impl Board {
    fn start_phase(&mut self, name: &str, total: usize) {
        if !self.phase.is_empty() {
            let (p, d, t) = (self.phase.clone(), self.phase_done, self.phase_total);
            self.phases.push(serde_json::json!({ "name": p, "done": d, "total": t }));
        }
        self.phase = name.to_string();
        self.phase_done = 0;
        self.phase_total = total;
    }

    fn finding(&mut self, line: String) {
        if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(format!("{CAMPAIGN_DIR}/findings.log")) {
            let _ = writeln!(f, "{line}");
        }
        let short: String = line.chars().take(400).collect();
        self.recent.push_front(short);
        self.recent.truncate(40);
    }

    fn summary(&self, elapsed_s: f64) -> String {
        let mut s = String::from("# Failure campaign summary\n\n");
        s.push_str(&format!(
            "Elapsed {:.2} h, {} runs. Singles live {} / no effect {}. Definite bugs {}, combinations with emergent effects {}, with masked effects {}, harness errors {}.\n\n",
            elapsed_s / 3600.,
            self.runs,
            self.singles_live,
            self.singles_dead,
            self.bugs,
            self.emergent,
            self.masked,
            self.harness_errors
        ));
        for p in &self.phases {
            s.push_str(&format!("- {}: {}/{}\n", p["name"].as_str().unwrap_or(""), p["done"], p["total"]));
        }
        s.push_str(&format!("- {} (current): {}/{}\n\n", self.phase, self.phase_done, self.phase_total));
        if !self.note.is_empty() {
            s.push_str(&format!("{}\n\n", self.note));
        }
        if !self.groups.is_empty() {
            s.push_str("| group | population | pilot (random) | pilot anomaly rate (95%) | run | anomalies found | unrun | expected anomalies left (95% upper) |\n|---|---|---|---|---|---|---|---|\n");
            for g in &self.groups {
                s.push_str(&format!(
                    "| {} | {} | {} | {:.3}% ({:.3}-{:.3}%) | {} | {} | {} | {:.0} ({:.0}) |\n",
                    g["name"].as_str().unwrap_or(""),
                    g["population"],
                    g["pilot"],
                    g["rate"].as_f64().unwrap_or(0.) * 100.,
                    g["rate_lo"].as_f64().unwrap_or(0.) * 100.,
                    g["rate_hi"].as_f64().unwrap_or(0.) * 100.,
                    g["run"],
                    g["hits"],
                    g["unrun"],
                    g["expected_left"].as_f64().unwrap_or(0.),
                    g["expected_left_hi"].as_f64().unwrap_or(0.)
                ));
            }
        }
        s.push_str("\n## Latest findings\n\n");
        for f in &self.recent {
            s.push_str(&format!("- {f}\n"));
        }
        s
    }
}

struct Sink {
    file: Mutex<Option<std::io::BufWriter<std::fs::File>>>,
}

impl Sink {
    fn new(name: &str) -> Self {
        let f = std::fs::OpenOptions::new().create(true).append(true).open(format!("{CAMPAIGN_DIR}/{name}")).ok().map(std::io::BufWriter::new);
        Self { file: Mutex::new(f) }
    }

    fn line(&self, v: &serde_json::Value) {
        if let Some(f) = lock(&self.file).as_mut() {
            let _ = writeln!(f, "{v}");
            let _ = f.flush();
        }
    }
}

fn stopped(deadline: Instant) -> bool {
    Instant::now() >= deadline || std::path::Path::new(&format!("{CAMPAIGN_DIR}/STOP")).exists()
}

fn execute(
    ctx: &Ctx,
    total: usize,
    job_at: &(dyn Fn(usize) -> Option<(Vec<Item>, usize)> + Sync),
    threads: usize,
    deadline: Instant,
    board: &Mutex<Board>,
    sink: &(dyn Fn(Outcome) + Sync),
) -> usize {
    let next = AtomicUsize::new(0);
    let done = AtomicUsize::new(0);
    std::thread::scope(|s| {
        for _ in 0..threads {
            s.spawn(|| loop {
                if stopped(deadline) {
                    break;
                }
                let i = next.fetch_add(1, Ordering::Relaxed);
                if i >= total {
                    break;
                }
                let Some(Some((items, horizon))) = guarded("job", board, || job_at(i)) else { continue };
                let o = run_guarded(ctx, &items, horizon);
                guarded("record", board, || sink(o));
                done.fetch_add(1, Ordering::Relaxed);
                let mut b = lock(board);
                b.phase_done += 1;
                b.runs += 1;
            });
        }
    });
    done.into_inner()
}

fn bug_detail(o: &Outcome) -> String {
    let mut parts = Vec::new();
    if let Some(p) = &o.panic {
        parts.push(format!("panic: {p}"));
    }
    if !o.cpp_trap.is_empty() {
        parts.push(format!("C++ module trapped: {}", o.cpp_trap.join("; ")));
    }
    if !o.non_finite.is_empty() {
        parts.push(format!("non-finite: {}", o.non_finite.iter().take(8).cloned().collect::<Vec<_>>().join(", ")));
    }
    if !o.runaway.is_empty() {
        parts.push(format!("runaway: {}", o.runaway.iter().take(8).cloned().collect::<Vec<_>>().join(", ")));
    }
    parts.join(" | ")
}

fn outcome_json(ctx: &Ctx, o: &Outcome) -> serde_json::Value {
    serde_json::json!({
        "label": o.items.iter().map(|i| ctx.label(*i)).collect::<Vec<_>>().join(" + "),
        "sim_s": o.sim_s,
        "wall_s": (o.wall_s * 100.).round() / 100.,
        "moved": o.ever.len(),
        "moved_sample": o.ever.iter().take(25).map(|i| ctx.names[*i].clone()).collect::<Vec<_>>(),
        "new_vars": o.extra.len(),
        "settled": o.settled,
        "arm": o.arm_results,
        "fbw_failures": o.fbw_failures,
        "panic": o.panic,
        "cpp": o.cpp_trap,
        "non_finite": o.non_finite,
        "runaway": o.runaway,
    })
}

fn replay_json(ctx: &Ctx, o: &Outcome) -> serde_json::Value {
    let mut end = serde_json::Map::new();
    for (i, v) in &o.end {
        if v.is_finite() {
            for m in msfs_name(&ctx.names[*i]) {
                end.insert(m, serde_json::json!(v));
            }
        }
    }
    for (name, v) in o.extra.iter().chain(&o.flags) {
        if v.is_finite() {
            for m in msfs_name(name) {
                end.insert(m, serde_json::json!(v));
            }
        }
    }
    serde_json::json!({ "label": o.items.iter().map(|i| ctx.label(*i)).collect::<Vec<_>>().join(" + "), "fbw": o.fbw_failures, "end": end })
}

fn write_base(ctx: &Ctx, dir: &str) {
    std::fs::create_dir_all(dir).ok();
    let mut base = serde_json::Map::new();
    for (i, name) in ctx.names.iter().enumerate() {
        let v = ctx.healthy[30.min(MAX_S)][i];
        if v.is_finite() {
            for m in msfs_name(name) {
                base.insert(m, serde_json::json!(v));
            }
        }
    }
    let _ = std::fs::write(format!("{dir}/base.json"), serde_json::to_string(&base).unwrap_or_default());
}

fn item_key(ctx: &Ctx, item: Item) -> String {
    match item {
        Item::Failure(id) => format!("F{id}"),
        Item::Breaker(i) => format!("CB:{}", ctx.units[i]),
    }
}

fn parse_item(ctx: &Ctx, key: &str) -> Option<Item> {
    if let Some(unit) = key.strip_prefix("CB:") {
        return ctx.units.iter().position(|u| *u == unit).map(Item::Breaker);
    }
    key.strip_prefix('F')?.parse().ok().map(Item::Failure)
}

fn outcome_full(ctx: &Ctx, o: &Outcome) -> serde_json::Value {
    let finite = |m: &BTreeMap<usize, f64>| -> serde_json::Map<String, serde_json::Value> {
        m.iter().filter(|(_, v)| v.is_finite()).map(|(i, v)| (ctx.names[*i].clone(), serde_json::json!(v))).collect()
    };
    serde_json::json!({
        "items": o.items.iter().map(|i| item_key(ctx, *i)).collect::<Vec<_>>(),
        "sim_s": o.sim_s,
        "wall_s": o.wall_s,
        "ever": o.ever.iter().map(|i| ctx.names[*i].clone()).collect::<Vec<_>>(),
        "end": finite(&o.end),
        "trend": o.trend.iter().map(|(i, t)| (ctx.names[*i].clone(), serde_json::json!(t))).collect::<serde_json::Map<_, _>>(),
        "extra": o.extra.iter().filter(|(_, v)| v.is_finite()).map(|(k, v)| (k.clone(), serde_json::json!(v))).collect::<serde_json::Map<_, _>>(),
        "first": o.first_deviation_s,
        "settled": o.settled,
        "arm": o.arm_results,
        "fbw": o.fbw_failures,
        "panic": o.panic,
        "cpp": o.cpp_trap,
        "non_finite": o.non_finite,
        "runaway": o.runaway,
    })
}

fn outcome_from(ctx: &Ctx, v: &serde_json::Value) -> Option<Outcome> {
    let strings = |k: &str| -> Vec<String> { v[k].as_array().into_iter().flatten().filter_map(|x| x.as_str().map(str::to_string)).collect() };
    let items: Vec<Item> = strings("items").iter().map(|k| parse_item(ctx, k)).collect::<Option<Vec<_>>>()?;
    if items.is_empty() {
        return None;
    }
    Some(Outcome {
        flags: BTreeMap::new(),
        items,
        sim_s: v["sim_s"].as_u64().unwrap_or(0) as usize,
        wall_s: v["wall_s"].as_f64().unwrap_or(0.),
        ever: strings("ever").iter().filter_map(|n| ctx.index.get(n).copied()).collect(),
        end: v["end"].as_object().into_iter().flatten().filter_map(|(n, x)| Some((*ctx.index.get(n)?, x.as_f64()?))).collect(),
        trend: v["trend"].as_object().into_iter().flatten().filter_map(|(n, x)| Some((*ctx.index.get(n)?, x.as_i64()? as i8))).collect(),
        extra: v["extra"].as_object().into_iter().flatten().filter_map(|(n, x)| Some((n.clone(), x.as_f64()?))).collect(),
        first_deviation_s: v["first"].as_u64().map(|x| x as usize),
        settled: v["settled"].as_bool().unwrap_or(false),
        arm_results: v["arm"].as_array().into_iter().flatten().filter_map(|x| x.as_f64()).collect(),
        non_finite: strings("non_finite"),
        runaway: strings("runaway"),
        panic: v["panic"].as_str().map(str::to_string),
        cpp_trap: strings("cpp"),
        fbw_failures: v["fbw"].as_array().into_iter().flatten().filter_map(|x| x.as_u64()).collect(),
    })
}

fn load_jsonl(name: &str) -> Vec<serde_json::Value> {
    std::fs::read_to_string(format!("{CAMPAIGN_DIR}/{name}"))
        .map(|t| t.lines().filter_map(|l| serde_json::from_str(l).ok()).collect())
        .unwrap_or_default()
}

fn run_singles(ctx: &Ctx, items: &[Item], threads: usize, deadline: Instant, board: &Mutex<Board>, tag: &str) -> Vec<Outcome> {
    let previous: Vec<Outcome> = load_jsonl(&format!("outcomes-{tag}.jsonl")).iter().filter_map(|v| outcome_from(ctx, v)).filter(|o| o.items.len() == 1).collect();
    let done: std::collections::HashSet<Item> = previous.iter().map(|o| o.items[0]).collect();
    {
        let mut b = lock(board);
        b.phase_done += previous.len();
        b.runs += previous.len();
        for o in &previous {
            if o.live() {
                b.singles_live += 1;
            } else {
                b.singles_dead += 1;
            }
            b.bugs += usize::from(o.definite_bug());
        }
        if !previous.is_empty() {
            b.finding(format!("RESUMED [{tag}] {} singles loaded from the previous run", previous.len()));
        }
    }
    let todo: Vec<Item> = items.iter().copied().filter(|i| !done.contains(i)).collect();
    let results = Mutex::new(previous);
    let sink = Sink::new(&format!("singles-{tag}.jsonl"));
    let full = Sink::new(&format!("outcomes-{tag}.jsonl"));
    let no_effect = Sink::new(&format!("no-effect-{tag}.txt"));
    let replay = Sink::new(&format!("fws-{tag}/runs.jsonl"));
    let job = |i: usize| todo.get(i).map(|it| (vec![*it], MAX_S));
    execute(ctx, todo.len(), &job, threads, deadline, board, &|o: Outcome| {
        full.line(&outcome_full(ctx, &o));
        sink.line(&outcome_json(ctx, &o));
        replay.line(&replay_json(ctx, &o));
        {
            let mut b = lock(board);
            if o.live() {
                b.singles_live += 1;
            } else {
                b.singles_dead += 1;
                if let Some(f) = lock(&no_effect.file).as_mut() {
                    let _ = writeln!(f, "{}", ctx.label(o.items[0]));
                    let _ = f.flush();
                }
            }
            if o.definite_bug() {
                b.bugs += 1;
                let l = format!("BUG [{tag}] {}: {}", ctx.label(o.items[0]), bug_detail(&o));
                b.finding(l);
            }
        }
        lock(&results).push(o);
    });
    results.into_inner().unwrap_or_else(|e| e.into_inner())
}

struct Group {
    name: String,
    cat: usize,
    none: bool,
    pairs: Vec<(u32, u32)>,
    population: usize,
    pilot_n: usize,
    pilot_hits: usize,
    run: usize,
    hits: usize,
}

fn publish_groups(groups: &[Group], board: &Mutex<Board>) {
    let v: Vec<serde_json::Value> = groups
        .iter()
        .map(|g| {
            let (p, lo, hi) = wilson(g.pilot_hits, g.pilot_n);
            let unrun = g.population.saturating_sub(g.run);
            serde_json::json!({
                "name": g.name, "population": g.population, "pilot": g.pilot_n, "pilot_hits": g.pilot_hits,
                "rate": p, "rate_lo": lo, "rate_hi": hi, "run": g.run, "hits": g.hits, "unrun": unrun,
                "expected_left": p * unrun as f64, "expected_left_hi": hi * unrun as f64,
            })
        })
        .collect();
    lock(board).groups = v;
}

#[test]
#[ignore]
fn failure_campaign() {
    deep_systems::set_log_quiet(true);
    systems::shared::use_nominal_values(true);
    let hours: f64 = std::env::var("SUITE_HOURS").ok().and_then(|v| v.parse().ok()).unwrap_or(11.5);
    let threads = env_usize("SUITE_THREADS", std::thread::available_parallelism().map(|n| n.get()).unwrap_or(4));
    let started = Instant::now();
    let deadline = started + std::time::Duration::from_secs_f64(hours * 3600.);
    for d in ["", "/fws-runway", "/fws-apron"] {
        std::fs::create_dir_all(format!("{CAMPAIGN_DIR}{d}")).ok();
    }
    let _ = std::fs::remove_file(format!("{CAMPAIGN_DIR}/STOP"));
    let board = Mutex::new(Board::default());
    let finished = std::sync::atomic::AtomicBool::new(false);

    std::thread::scope(|scope| {
        scope.spawn(|| {
            let mut history: VecDeque<(Instant, usize)> = VecDeque::new();
            let mut last_summary = Instant::now();
            loop {
                let done = finished.load(Ordering::Relaxed);
                let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    let (json, summary) = {
                        let b = lock(&board);
                        let now = Instant::now();
                        history.push_back((now, b.runs));
                        while history.len() > 2 && history.front().is_some_and(|f| now.duration_since(f.0).as_secs_f64() > 120.) {
                            history.pop_front();
                        }
                        let (t0, r0) = history.front().copied().unwrap_or((now, b.runs));
                        let rate = if now > t0 { (b.runs.saturating_sub(r0)) as f64 / now.duration_since(t0).as_secs_f64() } else { 0. };
                        let json = serde_json::json!({
                            "finished": done,
                            "elapsed_s": started.elapsed().as_secs_f64(),
                            "deadline_s": hours * 3600.,
                            "phase": b.phase,
                            "phase_done": b.phase_done,
                            "phase_total": b.phase_total,
                            "phases": b.phases,
                            "runs": b.runs,
                            "runs_per_s": rate,
                            "bugs": b.bugs,
                            "emergent": b.emergent,
                            "masked": b.masked,
                            "singles_live": b.singles_live,
                            "singles_dead": b.singles_dead,
                            "harness_errors": b.harness_errors,
                            "recent": b.recent,
                            "groups": b.groups,
                            "note": b.note,
                        });
                        let summary = (done || last_summary.elapsed().as_secs() >= 300).then(|| b.summary(started.elapsed().as_secs_f64()));
                        (json, summary)
                    };
                    let tmp = format!("{CAMPAIGN_DIR}/progress.tmp");
                    if std::fs::write(&tmp, json.to_string()).is_ok() {
                        let _ = std::fs::rename(&tmp, format!("{CAMPAIGN_DIR}/progress.json"));
                    }
                    if let Some(s) = summary {
                        let _ = std::fs::write(format!("{CAMPAIGN_DIR}/SUMMARY.md"), s);
                        last_summary = Instant::now();
                    }
                }));
                if done {
                    break;
                }
                std::thread::sleep(std::time::Duration::from_secs(2));
            }
        });

        let _ = guarded("campaign", &board, || campaign_body(&board, threads, deadline));
        finished.store(true, Ordering::Relaxed);
    });
}

fn campaign_body(board: &Mutex<Board>, threads: usize, deadline: Instant) {
    lock(board).start_phase("healthy reference (runway)", 2);
    let Some((ctx, mut items)) = guarded("prepare runway", board, || prepare(&STAGES[1])) else { return };
    let smoke = env_usize("CAMPAIGN_SMOKE", 0);
    if smoke > 0 {
        let f: Vec<Item> = items.iter().copied().filter(|i| matches!(i, Item::Failure(_))).take(smoke).collect();
        let b: Vec<Item> = items.iter().copied().filter(|i| matches!(i, Item::Breaker(_))).take(smoke).collect();
        items = f.into_iter().chain(b).collect();
    }
    write_base(&ctx, &format!("{CAMPAIGN_DIR}/fws-runway"));
    lock(board).start_phase("singles (runway)", items.len());
    let singles = guarded("singles runway", board, || run_singles(&ctx, &items, threads, deadline, board, "runway")).unwrap_or_default();
    let single: HashMap<Item, &Outcome> = singles.iter().map(|o| (o.items[0], o)).collect();

    if !stopped(deadline) {
        lock(board).start_phase("healthy reference (apron)", 2);
        if let Some((apron_ctx, mut apron_items)) = guarded("prepare apron", board, || prepare(&STAGES[0])) {
            if smoke > 0 {
                apron_items.retain(|i| items.contains(i));
            }
            write_base(&apron_ctx, &format!("{CAMPAIGN_DIR}/fws-apron"));
            lock(board).start_phase("singles (apron)", apron_items.len());
            let _ = guarded("singles apron", board, || run_singles(&apron_ctx, &apron_items, threads, deadline, board, "apron"));
        }
    }

    let live: Vec<&Outcome> = singles.iter().filter(|o| o.live()).collect();
    let live_items: std::collections::HashSet<Item> = live.iter().map(|o| o.items[0]).collect();
    let mut var_count: HashMap<usize, usize> = HashMap::new();
    for o in &live {
        for i in &o.ever {
            *var_count.entry(*i).or_default() += 1;
        }
    }
    let hub_threshold = ((live.len() as f64) * HUB_SHARE).ceil().max(2.) as usize;
    let electrical_flow = |n: &str| n.ends_with("_POTENTIAL") || n.ends_with("_CURRENT") || n.ends_with("_CURRENT_A");
    let hubs: BTreeSet<usize> = var_count
        .iter()
        .filter(|(_, c)| **c >= hub_threshold)
        .map(|(i, _)| *i)
        .chain(ctx.names.iter().enumerate().filter(|(_, n)| electrical_flow(n)).map(|(i, _)| i))
        .collect();
    let horizon = |its: &[Item]| -> usize { (its.iter().filter_map(|i| single.get(i)).map(|o| o.sim_s).max().unwrap_or(MAX_S) + SETTLE_WINDOW_S).min(MAX_S) };
    lock(board).note = format!("Combinations use only the {} of {} items that had an effect on the runway.", live_items.len(), items.len());
    let pos: HashMap<Item, u32> = items.iter().enumerate().map(|(i, it)| (*it, i as u32)).collect();
    let seen: Mutex<std::collections::HashSet<(u32, u32)>> = Mutex::new(std::collections::HashSet::new());
    let reduce = |its: &[Item]| -> Option<Vec<Item>> {
        let mut v: Vec<Item> = its.iter().copied().filter(|i| live_items.contains(i)).collect();
        v.sort();
        v.dedup();
        (v.len() >= 2).then_some(v)
    };
    let pair_key = |v: &[Item]| -> Option<(u32, u32)> {
        if v.len() != 2 {
            return None;
        }
        let (a, b) = (*pos.get(&v[0])?, *pos.get(&v[1])?);
        Some(if a < b { (a, b) } else { (b, a) })
    };

    let previous_combos = load_jsonl("pairs.jsonl");
    let done_combos: std::collections::HashSet<Vec<Item>> = previous_combos
        .iter()
        .filter_map(|v| {
            let mut its: Vec<Item> = v["items"].as_array()?.iter().map(|k| parse_item(&ctx, k.as_str()?)).collect::<Option<Vec<_>>>()?;
            its.sort();
            Some(its)
        })
        .collect();
    let previous_stats: HashMap<(String, String), (usize, usize)> = {
        let mut m: HashMap<(String, String), (usize, usize)> = HashMap::new();
        let mut b = lock(board);
        for v in &previous_combos {
            let bug = v["panic"].is_string() || v["cpp"].as_array().is_some_and(|a| !a.is_empty()) || v["non_finite"].as_array().is_some_and(|a| !a.is_empty()) || v["runaway"].as_array().is_some_and(|a| !a.is_empty());
            let emergent = v["emergent"].as_u64().unwrap_or(0) > 0;
            let masked = v["masked"].as_u64().unwrap_or(0) > 0;
            let e = m.entry((v["phase"].as_str().unwrap_or("").to_string(), v["group"].as_str().unwrap_or("").to_string())).or_default();
            e.0 += 1;
            e.1 += usize::from(bug || emergent || masked);
            b.runs += 1;
            b.bugs += usize::from(bug);
            b.emergent += usize::from(emergent);
            b.masked += usize::from(masked);
        }
        if !previous_combos.is_empty() {
            b.finding(format!("RESUMED {} combinations loaded from the previous run", previous_combos.len()));
        }
        m
    };
    let is_done = |its: &[Item]| -> bool {
        let mut v = its.to_vec();
        v.sort();
        done_combos.contains(&v)
    };
    let pairs_sink = Sink::new("pairs.jsonl");
    let replay_pairs = Sink::new("fws-runway/runs.jsonl");
    let record = |phase: &str, group: &str, o: Outcome| -> bool {
        let (emergent, masked) = compare_combo(&single, &o, &hubs);
        let bug = o.definite_bug();
        let anomaly = bug || !emergent.is_empty() || !masked.is_empty();
        let mut v = outcome_json(&ctx, &o);
        v["items"] = serde_json::json!(o.items.iter().map(|i| item_key(&ctx, *i)).collect::<Vec<_>>());
        v["phase"] = serde_json::json!(phase);
        v["group"] = serde_json::json!(group);
        v["emergent"] = serde_json::json!(emergent.len());
        v["emergent_sample"] = serde_json::json!(emergent.iter().take(15).map(|i| ctx.names[*i].clone()).collect::<Vec<_>>());
        v["masked"] = serde_json::json!(masked.len());
        v["masked_sample"] = serde_json::json!(masked.iter().take(10).map(|i| ctx.names[*i].clone()).collect::<Vec<_>>());
        pairs_sink.line(&v);
        if anomaly {
            replay_pairs.line(&replay_json(&ctx, &o));
            let mut b = lock(board);
            b.bugs += usize::from(bug);
            b.emergent += usize::from(!emergent.is_empty());
            b.masked += usize::from(!masked.is_empty());
            let label = o.items.iter().map(|i| ctx.label(*i)).collect::<Vec<_>>().join(" + ");
            let kind = if bug { "BUG" } else if !masked.is_empty() { "MASKED" } else { "EMERGENT" };
            let mut detail = Vec::new();
            if bug {
                detail.push(bug_detail(&o));
            }
            if !emergent.is_empty() {
                detail.push(format!(
                    "{} moved that neither part moves alone ({})",
                    emergent.len(),
                    emergent.iter().take(5).map(|i| ctx.names[*i].clone()).collect::<Vec<_>>().join(", ")
                ));
            }
            if !masked.is_empty() {
                detail.push(format!(
                    "{} that a part moves alone stayed put ({})",
                    masked.len(),
                    masked.iter().take(5).map(|i| ctx.names[*i].clone()).collect::<Vec<_>>().join(", ")
                ));
            }
            b.finding(format!("{kind} [{group}] {label}: {}", detail.join(" | ")));
        }
        anomaly
    };

    let mut family: BTreeMap<&'static str, BTreeMap<u8, Vec<Item>>> = BTreeMap::new();
    for item in &items {
        let label = match item {
            Item::Failure(id) => ctx.failure_names.get(id).cloned().unwrap_or_default(),
            Item::Breaker(i) => ctx.units[*i].to_string(),
        };
        if let Some((fam, n)) = family_of(&label) {
            family.entry(fam).or_default().entry(n).or_default().push(*item);
        }
    }
    let mut family_jobs: Vec<Vec<Item>> = Vec::new();
    for members in family.values() {
        let idx: Vec<&Vec<Item>> = members.values().collect();
        match idx.len() {
            3 => {
                for x in idx[0] {
                    for y in idx[1] {
                        for z in idx[2] {
                            family_jobs.push(vec![*x, *y, *z]);
                        }
                    }
                }
            }
            2 => {
                for x in idx[0] {
                    for y in idx[1] {
                        family_jobs.push(vec![*x, *y]);
                    }
                }
            }
            _ => {}
        }
    }
    let raw_family_jobs = family_jobs.len();
    let mut unique: BTreeSet<Vec<Item>> = BTreeSet::new();
    let family_jobs: Vec<Vec<Item>> = family_jobs.into_iter().filter_map(|j| reduce(&j)).filter(|j| unique.insert(j.clone())).collect();
    for j in &family_jobs {
        if let Some(k) = pair_key(j) {
            lock(&seen).insert(k);
        }
    }
    {
        let mut b = lock(board);
        let triples = family_jobs.iter().filter(|j| j.len() == 3).count();
        b.note = format!(
            "{} Redundant computers: {} raw combinations reduced to {} ({} triples, {} demoted or native pairs) after dropping inert members.",
            b.note,
            raw_family_jobs,
            family_jobs.len(),
            triples,
            family_jobs.len() - triples
        );
    }
    if !stopped(deadline) {
        lock(board).start_phase("redundant computers (all failure modes)", family_jobs.len());
        let job = |i: usize| family_jobs.get(i).filter(|j| !is_done(j)).map(|j| (j.clone(), horizon(j)));
        execute(&ctx, family_jobs.len(), &job, threads, deadline, board, &|o| {
            record("families", "redundant computers", o);
        });
    }

    let breakers: Vec<Item> = items.iter().copied().filter(|i| matches!(i, Item::Breaker(_)) && live_items.contains(i)).collect();
    let failures_only: Vec<Item> = items.iter().copied().filter(|i| matches!(i, Item::Failure(_)) && live_items.contains(i)).collect();
    let nb = breakers.len();
    let cb_pairs = nb * nb.saturating_sub(1) / 2;
    let run_breaker_pairs = || {
        if stopped(deadline) || nb < 2 {
            return;
        }
        lock(board).start_phase("all breaker pairs", cb_pairs);
        let pair_of = |k: usize| -> (usize, usize) {
            let mut i = 0usize;
            let mut rem = k;
            while i + 1 < nb && rem >= nb - 1 - i {
                rem -= nb - 1 - i;
                i += 1;
            }
            (i, (i + 1 + rem).min(nb - 1))
        };
        let job = |k: usize| {
            let (i, j) = pair_of(k);
            if i == j {
                return None;
            }
            let its = vec![breakers[i], breakers[j]];
            if !lock(&seen).insert(pair_key(&its)?) || is_done(&its) {
                return None;
            }
            let h = horizon(&its);
            Some((its, h))
        };
        execute(&ctx, cb_pairs, &job, threads, deadline, board, &|o| {
            record("breaker pairs", "2CB (all)", o);
        });
    };
    if failures_only.len() < 2 || nb == 0 {
        run_breaker_pairs();
        return;
    }

    let mut by_var: HashMap<usize, Vec<u32>> = HashMap::new();
    for o in &live {
        for v in o.ever.iter().filter(|v| !hubs.contains(v)) {
            if let Some(p) = pos.get(&o.items[0]) {
                by_var.entry(*v).or_default().push(*p);
            }
        }
    }
    let mut coupling: HashMap<(u32, u32), u16> = HashMap::new();
    for members in by_var.values() {
        for (a, &x) in members.iter().enumerate() {
            for &y in &members[a + 1..] {
                let (lo, hi) = if x < y { (x, y) } else { (y, x) };
                if lo == hi || (matches!(items[lo as usize], Item::Breaker(_)) && matches!(items[hi as usize], Item::Breaker(_))) {
                    continue;
                }
                let e = coupling.entry((lo, hi)).or_default();
                *e = e.saturating_add(1);
            }
        }
    }
    let category = |p: (u32, u32)| -> usize { usize::from(matches!(items[p.0 as usize], Item::Breaker(_)) || matches!(items[p.1 as usize], Item::Breaker(_))) };
    let cat_names = ["2F", "1F+1CB"];
    let cat_population = [failures_only.len() * failures_only.len().saturating_sub(1) / 2, failures_only.len() * nb];
    let mut groups: Vec<Group> = Vec::new();
    for cat in 0..2 {
        let mut list: Vec<((u32, u32), u16)> = coupling.iter().filter(|(p, _)| category(**p) == cat).map(|(p, c)| (*p, *c)).collect();
        list.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
        let n = list.len();
        let (t_hi, t_lo) = (n / 3, 2 * n / 3);
        let high: Vec<(u32, u32)> = list[..t_hi].iter().map(|x| x.0).collect();
        let mid: Vec<(u32, u32)> = list[t_hi..t_lo].iter().map(|x| x.0).collect();
        let low: Vec<(u32, u32)> = list[t_lo..].iter().map(|x| x.0).collect();
        for (label, pairs) in [("high overlap", high), ("medium overlap", mid), ("low overlap", low)] {
            let population = pairs.len();
            groups.push(Group { name: format!("{} {label}", cat_names[cat]), cat, none: false, pairs, population, pilot_n: 0, pilot_hits: 0, run: 0, hits: 0 });
        }
        groups.push(Group { name: format!("{} no overlap", cat_names[cat]), cat, none: true, pairs: Vec::new(), population: cat_population[cat].saturating_sub(n), pilot_n: 0, pilot_hits: 0, run: 0, hits: 0 });
    }
    publish_groups(&groups, board);
    let random_none = |cat: usize, seed: u64| -> Option<(u32, u32)> {
        for attempt in 0..64u64 {
            let r1 = splitmix(seed.wrapping_mul(131).wrapping_add(attempt));
            let r2 = splitmix(r1);
            let a = failures_only[(r1 % failures_only.len() as u64) as usize];
            let b = if cat == 0 { failures_only[(r2 % failures_only.len() as u64) as usize] } else { breakers[(r2 % nb as u64) as usize] };
            let (Some(&a), Some(&b)) = (pos.get(&a), pos.get(&b)) else { continue };
            if a == b {
                continue;
            }
            let p = if a < b { (a, b) } else { (b, a) };
            if coupling.contains_key(&p) {
                continue;
            }
            if lock(&seen).insert(p) {
                return Some(p);
            }
        }
        None
    };

    let pilot_per_group = env_usize("CAMPAIGN_PILOT", PILOT_PER_GROUP);
    let mut pilot_jobs: Vec<(usize, (u32, u32))> = Vec::new();
    for (gi, g) in groups.iter().enumerate() {
        if g.none {
            for k in 0..pilot_per_group {
                if let Some(p) = random_none(g.cat, splitmix(gi as u64 * 1_000_003 + k as u64)) {
                    pilot_jobs.push((gi, p));
                }
            }
        } else {
            let mut idx: Vec<usize> = (0..g.pairs.len()).collect();
            let take = idx.len().min(pilot_per_group);
            for k in 0..take {
                let j = k + (splitmix(gi as u64 * 7_919 + k as u64) % (idx.len() - k) as u64) as usize;
                idx.swap(k, j);
            }
            for &k in idx.iter().take(take) {
                lock(&seen).insert(g.pairs[k]);
                pilot_jobs.push((gi, g.pairs[k]));
            }
        }
    }
    if !stopped(deadline) {
        lock(board).start_phase("pilot: random pairs per group", pilot_jobs.len());
        let stats: Mutex<Vec<(usize, usize)>> = Mutex::new(vec![(0, 0); groups.len()]);
        let gi_of: HashMap<Vec<Item>, usize> = pilot_jobs.iter().map(|(gi, (a, b))| (vec![items[*a as usize], items[*b as usize]], *gi)).collect();
        let names: Vec<String> = groups.iter().map(|g| g.name.clone()).collect();
        let job = |i: usize| {
            let (_, (a, b)) = pilot_jobs.get(i)?;
            let its = vec![items[*a as usize], items[*b as usize]];
            if is_done(&its) {
                return None;
            }
            let h = horizon(&its);
            Some((its, h))
        };
        execute(&ctx, pilot_jobs.len(), &job, threads, deadline, board, &|o| {
            let gi = gi_of.get(&o.items).copied().unwrap_or(0);
            let hit = record("pilot", &names[gi], o);
            let mut s = lock(&stats);
            s[gi].0 += 1;
            s[gi].1 += usize::from(hit);
        });
        for (gi, s) in lock(&stats).iter().enumerate() {
            let prev = previous_stats.get(&("pilot".to_string(), groups[gi].name.clone())).copied().unwrap_or((0, 0));
            groups[gi].pilot_n = s.0 + prev.0;
            groups[gi].pilot_hits = s.1 + prev.1;
            groups[gi].run = s.0 + prev.0;
            groups[gi].hits = s.1 + prev.1;
        }
        publish_groups(&groups, board);
    }

    run_breaker_pairs();

    let mut order: Vec<usize> = (0..groups.len()).collect();
    order.sort_by(|a, b| wilson(groups[*b].pilot_hits, groups[*b].pilot_n).0.total_cmp(&wilson(groups[*a].pilot_hits, groups[*a].pilot_n).0));
    lock(board).note = format!("Combinations use only the {} items with an effect on the runway. Greedy order (by pilot anomaly rate): {}", live_items.len(), order.iter().map(|g| groups[*g].name.clone()).collect::<Vec<_>>().join(" > "));
    for gi in order {
        if stopped(deadline) {
            break;
        }
        let name = groups[gi].name.clone();
        let cat = groups[gi].cat;
        let none_group = groups[gi].none;
        let remaining: Vec<(u32, u32)> = {
            let s = lock(&seen);
            groups[gi].pairs.iter().copied().filter(|p| !s.contains(p)).collect()
        };
        let total = if none_group { groups[gi].population.saturating_sub(groups[gi].run) } else { remaining.len() };
        lock(board).start_phase(&format!("exhaust: {name}"), total);
        let counts = Mutex::new((0usize, 0usize));
        let job = |k: usize| {
            let p = if none_group { random_none(cat, splitmix(0xC0FFEE ^ (gi as u64 * 2_000_003 + k as u64)))? } else { *remaining.get(k)? };
            let its = vec![items[p.0 as usize], items[p.1 as usize]];
            if is_done(&its) {
                return None;
            }
            let h = horizon(&its);
            Some((its, h))
        };
        execute(&ctx, total, &job, threads, deadline, board, &|o| {
            let hit = record("exhaust", &name, o);
            let mut c = lock(&counts);
            c.0 += 1;
            c.1 += usize::from(hit);
        });
        let (n, h) = counts.into_inner().unwrap_or_else(|e| e.into_inner());
        let prev = previous_stats.get(&("exhaust".to_string(), name.clone())).copied().unwrap_or((0, 0));
        groups[gi].run += n + prev.0;
        groups[gi].hits += h + prev.1;
        publish_groups(&groups, board);
    }
    if !stopped(deadline) {
        let _ = std::fs::write(format!("{CAMPAIGN_DIR}/COMPLETE"), "every phase finished");
    }
}


#[test]
#[ignore]
fn export_failure_registry() {
    let registry = deep_systems::deep::registry();
    let rows: Vec<serde_json::Value> = registry
        .failures
        .iter()
        .map(|f| serde_json::json!({"id": f.id, "ata": f.ata, "name": f.name, "component": f.component, "effect": f.effect}))
        .collect();
    let path = std::env::var("EXPORT_PATH").unwrap_or_else(|_| "E:/registry-failures.json".to_string());
    std::fs::write(&path, serde_json::to_string(&rows).expect("registry json")).expect("write registry export");
    println!("{} failures written to {path}", rows.len());
}

#[test]
#[ignore]
fn stage_singles() {
    deep_systems::set_log_quiet(true);
    systems::shared::use_nominal_values(true);
    let stage_name = std::env::var("STAGE").unwrap_or_else(|_| "cruise".to_string());
    let tag = std::env::var("TAG").unwrap_or_else(|_| stage_name.clone());
    let threads = env_usize("SUITE_THREADS", std::thread::available_parallelism().map(|n| n.get()).unwrap_or(4));
    let stage: &'static Stage = match stage_name.as_str() {
        "apron" => &STAGES[0],
        "runway" => &STAGES[1],
        "climb" => &STAGES[2],
        "cruise" => &STAGES[3],
        "approach" => &STAGES[4],
        "final" => &STAGES[5],
        other => panic!("unknown STAGE {other}"),
    };
    std::fs::create_dir_all(format!("{CAMPAIGN_DIR}/fws-{tag}")).ok();
    let (ctx, all) = prepare(stage);
    let items: Vec<Item> = match std::env::var("ITEMS_FILE") {
        Ok(path) => {
            let wanted: Vec<Item> = std::fs::read_to_string(&path)
                .expect("ITEMS_FILE")
                .lines()
                .filter_map(|l| {
                    let first = l.split_whitespace().next()?;
                    let key = if first == "CB" { format!("CB:{}", l.split_whitespace().nth(1)?) } else { first.to_string() };
                    parse_item(&ctx, &key)
                })
                .collect();
            all.into_iter().filter(|i| wanted.contains(i)).collect()
        }
        Err(_) => all,
    };
    write_base(&ctx, &format!("{CAMPAIGN_DIR}/fws-{tag}"));
    let board = Mutex::new(Board::default());
    lock(&board).start_phase(&format!("singles ({tag})"), items.len());
    let deadline = Instant::now() + std::time::Duration::from_secs(86_400);
    let outcomes = run_singles(&ctx, &items, threads, deadline, &board, &tag);
    let live = outcomes.iter().filter(|o| o.live()).count();
    let bugs = outcomes.iter().filter(|o| o.definite_bug()).count();
    println!("stage {stage_name} tag {tag}: {} singles, {live} with an effect, {bugs} definite bugs", outcomes.len());
}

#[test]
#[ignore]
fn stage_health() {
    deep_systems::set_log_quiet(true);
    systems::shared::use_nominal_values(true);
    let watch = [
        "PRIM_1_HEALTHY",
        "SEC_1_HEALTHY",
        "ENG_1_PHYS_LIT",
        "ENG_1_PHYS_N1",
        "ENGINE_STATE:1",
        "AUTOTHRUST_N1_COMMANDED:1",
        "AUTOTHRUST_TLA:1",
        "HYD_GREEN_SYSTEM_1_SECTION_PRESSURE",
        "ELEC_AC_1_BUS_IS_POWERED",
        "PRESS_CABIN_ALTITUDE",
        "GEAR_HANDLE_POSITION",
        "GEAR_CENTER_POSITION",
        "LEFT_FLAPS_POSITION_PERCENT",
        "APU_N",
    ];
    let only = std::env::var("STAGE").ok();
    for stage in STAGES.iter().filter(|s| only.as_deref().is_none_or(|o| o == s.name)) {
        let t = Instant::now();
        let mut rig = Rig::spawn_stage(stage);
        rig.frames(600);
        let snap = rig.bench.query(|a| a.deep_systems.snapshot());
        let line: Vec<String> = watch
            .iter()
            .map(|n| {
                let v = rig.bench.known_variable_identifier(n).and_then(|id| rig.bench.read_identifier(&id)).or_else(|| snap.get(*n).copied());
                format!("{n}={}", v.map_or("-".into(), |v| format!("{v:.1}")))
            })
            .collect();
        let alerts: Vec<String> = snap.iter().filter(|(k, v)| k.contains("ECAM") && **v > 0.5 && !k.contains("COUNT")).map(|(k, _)| k.clone()).take(15).collect();
        println!("== {} ({:.1} s wall)", stage.name, t.elapsed().as_secs_f64());
        println!("   {}", line.join(" "));
        println!("   C++: {:?}", rig.host.failures());
        println!("   deep ECAM: {:?}", alerts);
    }
}


#[test]
#[ignore]
fn spawn_scaling() {
    deep_systems::set_log_quiet(true);
    systems::shared::use_nominal_values(true);
    let threads = env_usize("SUITE_THREADS", 30);
    let per_thread = env_usize("SPAWNS", 6);
    let _ = Rig::spawn();
    for legacy in [true, false] {
        super::tests_cpp_host::LEGACY_LINKER.store(legacy, Ordering::Relaxed);
        let t = Instant::now();
        std::thread::scope(|s| {
            for _ in 0..threads {
                s.spawn(|| {
                    for _ in 0..per_thread {
                        let mut rig = Rig::spawn();
                        rig.frames(100);
                    }
                });
            }
        });
        let wall = t.elapsed().as_secs_f64();
        let rigs = (threads * per_thread) as f64;
        println!(
            "{}: {} rigs x 200 frames on {threads} threads in {wall:.1} s = {:.1} rigs/s, {:.0} frames/s",
            if legacy { "per-spawn linker" } else { "shared InstancePre" },
            rigs,
            rigs / wall,
            rigs * ((WARMUP_S * FRAMES_PER_S) as f64 + 100.) / wall
        );
    }
}

#[test]
#[ignore]
fn burn_in() {
    deep_systems::set_log_quiet(true);
    systems::shared::use_nominal_values(true);
    let threads = env_usize("SUITE_THREADS", 20);
    let seconds = env_usize("BURN_S", 150) as f64;
    let (ctx, items) = prepare(&STAGES[1]);
    let jobs: Vec<(Vec<Item>, usize)> = items.iter().step_by(3).map(|i| (vec![*i], MAX_S)).collect();
    let t = Instant::now();
    let deadline = t + std::time::Duration::from_secs_f64(seconds);
    let next = AtomicUsize::new(0);
    let done = AtomicUsize::new(0);
    let wall = Mutex::new(0f64);
    std::thread::scope(|s| {
        for _ in 0..threads {
            s.spawn(|| loop {
                if Instant::now() >= deadline {
                    break;
                }
                let i = next.fetch_add(1, Ordering::Relaxed);
                let Some((job, h)) = jobs.get(i) else { break };
                let o = run_guarded(&ctx, job, *h);
                *wall.lock().unwrap() += o.wall_s;
                done.fetch_add(1, Ordering::Relaxed);
            });
        }
    });
    let n = done.into_inner();
    println!("BURN: {n} runs on {threads} threads in {:.0} s = {:.2} runs/s, mean {:.2} s wall per run", t.elapsed().as_secs_f64(), n as f64 / t.elapsed().as_secs_f64(), wall.into_inner().unwrap() / n.max(1) as f64);
}
