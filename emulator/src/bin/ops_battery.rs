//! Operations batteries: procedures (startup, shutdown, turnaround,
//! run-up) and wear (long runs that age the aircraft).
//!
//!     ops_battery --procedures [--workers N] [--dump-dir DIR]
//!     ops_battery --wear [--wear-cases N] [--wear-hours H] [--workers N] [--dump-dir DIR]
//!
//! **Procedures**: 6 procedures x 6 environments (cold/standard/hot day at
//! sea level and 5,000 ft) x (no failure + every catalogued failure at half
//! magnitude, applied as the procedure starts). With no failure every phase
//! must reach its state (engines On, buses powered, hydraulics at pressure;
//! after shutdown everything off) or the case fails. With a failure, missed
//! phases are recorded as findings; the case fails only on a crash, a NaN
//! or an invariant breach.
//!
//! **Wear**: per case, hours of engine power blocks at random lever settings
//! (with long TOGA holds) and 1-3 damageable component parameters worsening
//! at random rates. Checks: engine hours, cycles and creep never decrease;
//! no NaN; no invariant breach. Every 10 simulated minutes the engines'
//! fuel flow and EGT, their wear and the worsening parameters are recorded.
//!
//! Each case runs in a worker process (the plugin's state is process-wide),
//! on a thread with a timeout. Results: one JSON line per case in
//! `worker_NNN.jsonl`, and SUMMARY.txt.

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::io::Write as _;
use std::path::PathBuf;
use std::sync::mpsc;
use std::time::Duration;

use fbw_a380_emulator::{presets, Emulator};

// ---- seeded randomness (same generator as the battery) ----
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn f64(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.f64()
    }
    fn usize(&mut self, lo: usize, hi: usize) -> usize {
        lo + (self.next() as usize) % (hi - lo).max(1)
    }
}

#[derive(Clone)]
struct Config {
    procedures: bool,
    workers: usize,
    dump_dir: PathBuf,
    timeout_secs: u64,
    wear_cases: usize,
    wear_hours: f64,
    seed: u64,
    worker: bool,
    shard_id: usize,
    start: usize,
    end: usize,
}

fn parse() -> Config {
    let args: Vec<String> = std::env::args().collect();
    let get = |f: &str| args.iter().position(|a| a == f).and_then(|i| args.get(i + 1)).cloned();
    let procedures = !args.iter().any(|a| a == "--wear");
    let stamp = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
    Config {
        procedures,
        workers: get("--workers").and_then(|s| s.parse().ok()).unwrap_or(32),
        dump_dir: get("--dump-dir").map(PathBuf::from).unwrap_or_else(|| {
            PathBuf::from(format!(r"E:\fbw-test-dumps\{}_{stamp}", if procedures { "procedures" } else { "wear" }))
        }),
        timeout_secs: get("--timeout-secs").and_then(|s| s.parse().ok()).unwrap_or(if procedures { 180 } else { 900 }),
        wear_cases: get("--wear-cases").and_then(|s| s.parse().ok()).unwrap_or(2000),
        wear_hours: get("--wear-hours").and_then(|s| s.parse().ok()).unwrap_or(4.0),
        seed: get("--seed").and_then(|s| s.parse().ok()).unwrap_or(stamp),
        worker: args.iter().any(|a| a == "--worker"),
        shard_id: get("--shard-id").and_then(|s| s.parse().ok()).unwrap_or(0),
        start: get("--shard-start").and_then(|s| s.parse().ok()).unwrap_or(0),
        end: get("--shard-end").and_then(|s| s.parse().ok()).unwrap_or(0),
    }
}

// ---- cases ----
const PROCEDURES: [&str; 6] = ["startup_ext_apu", "startup_apu_only", "startup_order_1234", "shutdown", "turnaround", "run_up"];
/// (OAT C, pressure altitude ft).
const ENVIRONMENTS: [(f64, f64); 6] = [(-30.0, 0.0), (15.0, 0.0), (45.0, 0.0), (-30.0, 5000.0), (15.0, 5000.0), (40.0, 5000.0)];

#[derive(Clone, Debug)]
enum Case {
    Procedure { proc: usize, env: usize, failure: Option<u64> },
    Wear { seed: u64 },
}

fn cases(cfg: &Config) -> Vec<Case> {
    if cfg.procedures {
        let failures = Emulator::new(systems::simulation::StartState::Apron).list_failures();
        let mut v = Vec::new();
        for proc in 0..PROCEDURES.len() {
            for env in 0..ENVIRONMENTS.len() {
                v.push(Case::Procedure { proc, env, failure: None });
                for &f in &failures {
                    v.push(Case::Procedure { proc, env, failure: Some(f) });
                }
            }
        }
        v
    } else {
        (0..cfg.wear_cases).map(|i| Case::Wear { seed: cfg.seed.wrapping_mul(0x9E37).wrapping_add(i as u64) }).collect()
    }
}

#[derive(Default)]
struct Outcome {
    desc: String,
    /// Failing findings: any makes the case fail.
    issues: Vec<String>,
    /// Non-failing findings (phases a failure stopped, for the report).
    notes: Vec<String>,
    metrics: Vec<(String, f64)>,
}

impl Outcome {
    fn passed(&self) -> bool {
        self.issues.is_empty()
    }
}

/// NaN anywhere, or an invariant breach so far.
fn health(e: &mut Emulator, out: &mut Outcome, when: &str) {
    if e.values().iter().any(|v| !v.is_finite()) {
        let name = e.snapshot_all().into_iter().find(|(_, v)| !v.is_finite()).map(|(n, _)| n).unwrap_or_default();
        out.issues.push(format!("nan at {when}: {name}"));
    }
    for v in e.invariant_report() {
        let issue = format!("invariant: {}", v.name);
        if !out.issues.contains(&issue) {
            out.issues.push(issue);
        }
    }
}

fn wait(e: &mut Emulator, secs: f64, done: impl FnMut(&mut Emulator) -> bool) -> bool {
    presets::run_until(e, secs, done)
}

fn set_env(e: &mut Emulator, env: usize) {
    let (oat, alt) = ENVIRONMENTS[env];
    e.set_oat_c(oat);
    e.set_pressure_altitude_ft(alt);
}

fn bus(e: &mut Emulator, n: u8) -> bool {
    e.get_var(&format!("A32NX_ELEC_AC_{n}_BUS_IS_POWERED")) > 0.5
}

// ---- procedures ----
fn run_procedure(proc: usize, env: usize, failure: Option<u64>) -> Outcome {
    let mut out = Outcome { desc: format!("{} env {env} failure {failure:?}", PROCEDURES[proc]), ..Default::default() };
    let strict = failure.is_none();
    // A phase that did not reach its state: a failure without a fault, a
    // finding with one.
    let mut phase = |out: &mut Outcome, ok: bool, what: &str| {
        if !ok {
            if strict {
                out.issues.push(format!("phase not reached: {what}"));
            } else {
                out.notes.push(format!("phase not reached: {what}"));
            }
        }
    };
    let apply = |e: &mut Emulator| {
        if let Some(f) = failure {
            e.set_failure_magnitude(f, 0.5);
        }
    };
    let start_engines = |e: &mut Emulator, order: [u8; 4], out: &mut Outcome, phase: &mut dyn FnMut(&mut Outcome, bool, &str)| {
        for n in 1..=4 {
            e.set_var(&format!("TURB ENG IGNITION SWITCH EX1:{n}"), 2.0);
        }
        e.run(presets::PRESET_DT, 10);
        for n in order {
            e.set_var(&format!("GENERAL ENG STARTER:{n}"), 1.0);
            let ok = wait(e, 180.0, |e| presets::engine_running(e, n));
            phase(out, ok, &format!("engine {n} started"));
        }
        for n in 1..=4 {
            e.set_var(&format!("TURB ENG IGNITION SWITCH EX1:{n}"), 1.0);
        }
    };
    let check_running = |e: &mut Emulator, out: &mut Outcome, phase: &mut dyn FnMut(&mut Outcome, bool, &str)| {
        wait(e, 20.0, |_| false);
        for n in 1..=4u8 {
            let ok = presets::engine_running(e, n);
            phase(out, ok, &format!("engine {n} running"));
            let ok = bus(e, n);
            phase(out, ok, &format!("AC bus {n} powered"));
        }
        for sys in ["GREEN", "YELLOW"] {
            let p = e.get_var(&format!("A32NX_HYD_{sys}_SYSTEM_1_SECTION_PRESSURE"));
            phase(out, p > 4500.0, &format!("{sys} hydraulics at pressure ({p:.0} psi)"));
        }
    };

    let mut e = match proc {
        3 | 4 | 5 => presets::engines_running(),
        _ => presets::cold_and_dark(),
    };
    set_env(&mut e, env);
    apply(&mut e);
    let wear_before: Vec<_> = (1..=4).map(|n| e.engine_wear(n)).collect();
    let t_before = e.elapsed_s();
    match proc {
        0 | 1 | 2 => {
            for bat in ["BAT_1", "BAT_2", "BAT_ESS", "BAT_APU"] {
                e.set_var(&format!("A32NX_OVHD_ELEC_{bat}_PB_IS_AUTO"), 1.0);
            }
            e.run(presets::PRESET_DT, 20);
            if proc != 1 {
                e.set_var("EXT_PWR_AVAIL:1", 1.0);
                e.set_var("A32NX_OVHD_ELEC_EXT_PWR_1_PB_IS_ON", 1.0);
                let ok = wait(&mut e, 10.0, |e| bus(e, 1));
                phase(&mut out, ok, "ground power on AC 1");
            }
            e.set_var("A32NX_OVHD_APU_MASTER_SW_PB_IS_ON", 1.0);
            e.run(presets::PRESET_DT, 20);
            e.set_var("A32NX_OVHD_APU_START_PB_IS_ON", 1.0);
            let ok = wait(&mut e, 180.0, |e| e.get_var("A32NX_OVHD_APU_START_PB_IS_AVAILABLE") > 0.5);
            phase(&mut out, ok, "APU available");
            e.set_var("A32NX_OVHD_ELEC_APU_GEN_1_PB_IS_ON", 1.0);
            e.set_var("A32NX_OVHD_ELEC_APU_GEN_2_PB_IS_ON", 1.0);
            e.set_var("A32NX_OVHD_APU_BLEED_PB_IS_ON", 1.0);
            e.run(presets::PRESET_DT, 30);
            let order = if proc == 2 { [1, 2, 3, 4] } else { [4, 3, 2, 1] };
            start_engines(&mut e, order, &mut out, &mut phase);
            if proc != 1 {
                e.set_var("A32NX_OVHD_ELEC_EXT_PWR_1_PB_IS_ON", 0.0);
            }
            check_running(&mut e, &mut out, &mut phase);
        }
        3 => {
            for n in 1..=4 {
                e.set_var(&format!("GENERAL ENG STARTER:{n}"), 0.0);
            }
            let ok = wait(&mut e, 240.0, |e| (1..=4u8).all(|n| e.get_var(&format!("ENGINE_N3:{n}")) < 5.0));
            phase(&mut out, ok, "engines spooled down");
            e.set_var("A32NX_OVHD_APU_BLEED_PB_IS_ON", 0.0);
            e.set_var("A32NX_OVHD_APU_MASTER_SW_PB_IS_ON", 0.0);
            let ok = wait(&mut e, 240.0, |e| e.get_var("A32NX_OVHD_APU_START_PB_IS_AVAILABLE") < 0.5);
            phase(&mut out, ok, "APU shut down");
            for bat in ["BAT_1", "BAT_2", "BAT_ESS", "BAT_APU"] {
                e.set_var(&format!("A32NX_OVHD_ELEC_{bat}_PB_IS_AUTO"), 0.0);
            }
            wait(&mut e, 10.0, |_| false);
            for n in 1..=4u8 {
                let s = e.get_var(&format!("ENGINE_STATE:{n}"));
                phase(&mut out, s.round() == 0.0, &format!("engine {n} off (state {s})"));
                let ok = !bus(&mut e, n);
                phase(&mut out, ok, &format!("AC bus {n} de-powered"));
            }
        }
        4 => {
            for n in 1..=4 {
                e.set_var(&format!("GENERAL ENG STARTER:{n}"), 0.0);
            }
            let ok = wait(&mut e, 240.0, |e| (1..=4u8).all(|n| e.get_var(&format!("ENGINE_STATE:{n}")).round() == 0.0));
            phase(&mut out, ok, "engines off");
            start_engines(&mut e, [4, 3, 2, 1], &mut out, &mut phase);
            check_running(&mut e, &mut out, &mut phase);
        }
        _ => {
            let idle: Vec<f64> = (1..=4).map(|n| e.get_var(&format!("ENGINE_N1:{n}"))).collect();
            for n in 1..=4 {
                e.set_thrust_lever(n, 0.6);
            }
            wait(&mut e, 60.0, |_| false);
            for n in 1..=4usize {
                let n1 = e.get_var(&format!("ENGINE_N1:{n}"));
                phase(&mut out, n1 > idle[n - 1] + 10.0, &format!("engine {n} spooled up (N1 {n1:.1} from {:.1})", idle[n - 1]));
                out.metrics.push((format!("n1_runup_{n}"), n1));
            }
            for n in 1..=4 {
                e.set_thrust_lever(n, 0.0);
            }
            wait(&mut e, 60.0, |_| false);
            for n in 1..=4usize {
                let n1 = e.get_var(&format!("ENGINE_N1:{n}"));
                phase(&mut out, n1 < idle[n - 1] + 5.0, &format!("engine {n} back to idle (N1 {n1:.1})"));
            }
        }
    }
    // Wear: never backwards; with no fault, exact start cycles and hours
    // that follow the engines' running time.
    let elapsed_h = (e.elapsed_s() - t_before) / 3600.0;
    let starts = match proc {
        0 | 1 | 2 | 4 => 1,
        _ => 0,
    };
    for n in 1..=4usize {
        let (a, b) = (wear_before[n - 1], e.engine_wear(n));
        out.metrics.push((format!("e{n}_cycles"), (b.cycles - a.cycles.min(b.cycles)) as f64));
        out.metrics.push((format!("e{n}_hours_s"), (b.hours - a.hours) * 3600.0));
        out.metrics.push((format!("e{n}_creep"), b.creep_life_fraction - a.creep_life_fraction));
        out.metrics.push((format!("e{n}_oil_pct"), b.oil_quantity_pct));
        if b.hours + 1e-9 < a.hours || b.cycles < a.cycles || b.creep_life_fraction + 1e-12 < a.creep_life_fraction {
            out.issues.push(format!("engine {n} wear went backwards"));
        }
        if b.oil_quantity_pct > a.oil_quantity_pct + 1e-9 {
            out.issues.push(format!("engine {n} oil rose without servicing"));
        }
        if b.hours - a.hours > elapsed_h + 1e-6 {
            out.issues.push(format!("engine {n} hours outran the clock"));
        }
        if strict {
            if b.cycles - a.cycles != starts {
                out.issues.push(format!("engine {n} wear: {} start cycles, expected {starts}", b.cycles - a.cycles));
            }
            if (b.oil_quantity_pct - a.oil_quantity_pct).abs() > 1e-9 {
                out.issues.push(format!("engine {n} wear: oil changed without a leak"));
            }
            if proc == 5 && (b.hours - a.hours) < 0.9 * elapsed_h {
                out.issues.push(format!("engine {n} wear: hours not accrued while running"));
            }
        }
    }
    health(&mut e, &mut out, "end");
    out
}

// ---- wear ----
fn run_wear(seed: u64, hours: f64) -> Outcome {
    let mut rng = Rng(seed);
    let mut out = Outcome { desc: format!("wear seed {seed} {hours} h"), ..Default::default() };
    let mut e = presets::engines_running();
    set_env(&mut e, rng.usize(0, ENVIRONMENTS.len()));
    // 1-3 damageable parameters worsening at random rates.
    let params = e.list_component_params();
    let mut worsening = Vec::new();
    for _ in 0..rng.usize(1, 4) {
        if params.is_empty() {
            break;
        }
        let (c, p, _) = params[rng.usize(0, params.len())].clone();
        let rate = rng.range(0.01, 0.15); // of the parameter's unit per hour
        let _ = e.set_component_param(&c, &p, 0.0, rate);
        worsening.push((c, p, rate));
    }
    out.desc.push_str(&format!(" worsening {:?}", worsening.iter().map(|(c, p, r)| format!("{c}.{p}+{r:.3}/h")).collect::<Vec<_>>()));

    const DT: f64 = 0.5;
    let blocks = (hours * 6.0).round() as usize; // 10-minute blocks
    let mut last = [(0.0f64, 0u32, 0.0f64); 4];
    for block in 0..blocks {
        // A power setting per block; now and then TOGA held for the block.
        let lever = if rng.f64() < 0.1 { 1.0 } else { rng.range(0.1, 0.85) };
        for n in 1..=4 {
            e.set_thrust_lever(n, lever);
        }
        e.run(DT, (600.0 / DT) as u32);
        let t = (block + 1) as f64 / 6.0;
        for n in 1..=4usize {
            let w = e.engine_wear(n);
            let (h, c, creep) = last[n - 1];
            if w.hours + 1e-9 < h || w.cycles < c || w.creep_life_fraction + 1e-12 < creep {
                out.issues.push(format!("engine {n} wear went backwards at {t:.2} h"));
            }
            last[n - 1] = (w.hours, w.cycles, w.creep_life_fraction);
            out.metrics.push((format!("t{t:.2}_e{n}_ff"), e.get_var(&format!("ENGINE_FF:{n}"))));
            out.metrics.push((format!("t{t:.2}_e{n}_egt"), e.get_var(&format!("ENGINE_EGT:{n}"))));
            out.metrics.push((format!("t{t:.2}_e{n}_creep"), w.creep_life_fraction));
        }
        out.metrics.push((format!("t{t:.2}_lever"), lever));
        for (c, p, _) in &worsening {
            let v = e.list_component_params().into_iter().find(|(cc, pp, _)| cc == c && pp == p).map_or(f64::NAN, |x| x.2);
            out.metrics.push((format!("t{t:.2}_{c}.{p}"), v));
        }
        health(&mut e, &mut out, &format!("{t:.2} h"));
        if !out.issues.is_empty() {
            break;
        }
    }
    out
}

// ---- worker / orchestrator ----
fn json_str(s: &str) -> String {
    let mut o = String::from("\"");
    for c in s.chars() {
        match c {
            '"' => o.push_str("\\\""),
            '\\' => o.push_str("\\\\"),
            c if (c as u32) < 0x20 => {
                let _ = write!(o, "\\u{:04x}", c as u32);
            }
            c => o.push(c),
        }
    }
    o.push('"');
    o
}

fn worker(cfg: Config) {
    std::fs::create_dir_all(&cfg.dump_dir).expect("dump dir");
    let all = cases(&cfg);
    let mut w = std::io::BufWriter::new(std::fs::File::create(cfg.dump_dir.join(format!("worker_{:03}.jsonl", cfg.shard_id))).expect("worker file"));
    let (mut pass, mut fail, mut hung) = (0usize, 0usize, 0usize);
    let mut findings: BTreeMap<String, usize> = BTreeMap::new();
    for i in cfg.start..cfg.end.min(all.len()) {
        let case = all[i].clone();
        let hours = cfg.wear_hours;
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| match case {
                Case::Procedure { proc, env, failure } => run_procedure(proc, env, failure),
                Case::Wear { seed } => run_wear(seed, hours),
            }));
            let out = r.unwrap_or_else(|p| {
                let msg = p.downcast_ref::<String>().cloned().or_else(|| p.downcast_ref::<&str>().map(|s| s.to_string())).unwrap_or_default();
                Outcome { desc: "panic".into(), issues: vec![format!("panic: {msg}")], ..Default::default() }
            });
            let _ = tx.send(out);
        });
        let out = match rx.recv_timeout(Duration::from_secs(cfg.timeout_secs)) {
            Ok(o) => o,
            Err(_) => {
                hung += 1;
                Outcome { desc: format!("{:?}", all[i]), issues: vec![format!("hung > {} s", cfg.timeout_secs)], ..Default::default() }
            }
        };
        if out.passed() {
            pass += 1;
        } else {
            fail += 1;
        }
        for f in out.issues.iter().chain(out.notes.iter()) {
            let key = f.split(" (").next().unwrap_or(f).to_owned();
            *findings.entry(key).or_insert(0) += 1;
        }
        let issues: Vec<String> = out.issues.iter().map(|s| json_str(s)).collect();
        let notes: Vec<String> = out.notes.iter().map(|s| json_str(s)).collect();
        let metrics: Vec<String> = out.metrics.iter().map(|(k, v)| format!("{}:{}", json_str(k), if v.is_finite() { v.to_string() } else { "null".into() })).collect();
        let _ = writeln!(
            w,
            "{{\"case\":{i},\"desc\":{},\"passed\":{},\"issues\":[{}],\"notes\":[{}],\"metrics\":{{{}}}}}",
            json_str(&out.desc),
            out.passed(),
            issues.join(","),
            notes.join(","),
            metrics.join(",")
        );
        let _ = w.flush();
    }
    let mut s = format!("shard {} [{},{}) pass={pass} fail={fail} hung={hung}\n", cfg.shard_id, cfg.start, cfg.end);
    let mut ranked: Vec<_> = findings.into_iter().collect();
    ranked.sort_by(|a, b| b.1.cmp(&a.1));
    for (k, n) in ranked.iter().take(60) {
        let _ = writeln!(s, "  {n:>6}  {k}");
    }
    let _ = std::fs::write(cfg.dump_dir.join(format!("worker_{:03}_summary.txt", cfg.shard_id)), s);
}

fn orchestrate(cfg: Config) {
    std::fs::create_dir_all(&cfg.dump_dir).expect("dump dir");
    let total = cases(&cfg).len();
    println!("{}: {total} cases, {} workers, {}", if cfg.procedures { "procedures" } else { "wear" }, cfg.workers, cfg.dump_dir.display());
    let exe = std::env::current_exe().expect("exe");
    let shard = total.div_ceil(cfg.workers.max(1));
    let mut kids = Vec::new();
    for wkr in 0..cfg.workers {
        let start = wkr * shard;
        if start >= total {
            break;
        }
        let mut c = std::process::Command::new(&exe);
        c.arg("--worker")
            .arg(if cfg.procedures { "--procedures" } else { "--wear" })
            .arg("--dump-dir").arg(&cfg.dump_dir)
            .arg("--timeout-secs").arg(cfg.timeout_secs.to_string())
            .arg("--wear-cases").arg(cfg.wear_cases.to_string())
            .arg("--wear-hours").arg(cfg.wear_hours.to_string())
            .arg("--seed").arg(cfg.seed.to_string())
            .arg("--shard-id").arg(wkr.to_string())
            .arg("--shard-start").arg(start.to_string())
            .arg("--shard-end").arg((start + shard).min(total).to_string());
        kids.push(c.spawn().expect("spawn worker"));
    }
    for mut k in kids {
        let _ = k.wait();
    }
    let (mut pass, mut fail, mut hung) = (0usize, 0usize, 0usize);
    let mut findings: BTreeMap<String, usize> = BTreeMap::new();
    for entry in std::fs::read_dir(&cfg.dump_dir).into_iter().flatten().flatten() {
        let name = entry.file_name().to_string_lossy().into_owned();
        if !name.ends_with("_summary.txt") {
            continue;
        }
        let text = std::fs::read_to_string(entry.path()).unwrap_or_default();
        let mut lines = text.lines();
        if let Some(head) = lines.next() {
            for tok in head.split_whitespace() {
                let mut kv = tok.splitn(2, '=');
                match (kv.next(), kv.next().and_then(|v| v.parse::<usize>().ok())) {
                    (Some("pass"), Some(v)) => pass += v,
                    (Some("fail"), Some(v)) => fail += v,
                    (Some("hung"), Some(v)) => hung += v,
                    _ => {}
                }
            }
        }
        for l in lines {
            let l = l.trim();
            if let Some((n, k)) = l.split_once("  ") {
                if let Ok(n) = n.trim().parse::<usize>() {
                    *findings.entry(k.trim().to_owned()).or_insert(0) += n;
                }
            }
        }
    }
    let mut s = format!("{} cases: pass={pass} fail={fail} hung={hung}\n\nfindings (failures without a fault; phases a fault stopped):\n", pass + fail);
    let mut ranked: Vec<_> = findings.into_iter().collect();
    ranked.sort_by(|a, b| b.1.cmp(&a.1));
    for (k, n) in ranked.iter().take(200) {
        let _ = writeln!(s, "  {n:>7}  {k}");
    }
    let _ = std::fs::write(cfg.dump_dir.join("SUMMARY.txt"), &s);
    print!("{s}");
}

fn main() {
    let cfg = parse();
    if cfg.worker {
        worker(cfg);
    } else {
        orchestrate(cfg);
    }
}
