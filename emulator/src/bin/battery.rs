//! `battery`: a seeded, combinatorial fuzz battery for the offline
//! emulator. Not a `#[test]` -- a standalone binary meant to run for a
//! long time, across every CPU core, hammering [`fbw_a380_emulator::Emulator`]
//! with thousands of cases and dumping every datapoint it touches.
//!
//! ## Why child *processes*, not just threads
//!
//! `fbw_a380_systems_xp`'s own `scenarios.rs` documents it directly: the
//! failure/breaker/circuit/invariant/wear state this crate exercises lives
//! in process-global `static ... Mutex<...>` items, not per-`Emulator`
//! state (`Emulator::new` calls `scenarios::reset_global_state()` etc. so
//! *sequential* cases in one process are always clean, but two `Emulator`s
//! ticking concurrently on two threads of the *same* process would tear
//! each other's failure/breaker/invariant state -- there is no thread-local
//! isolation, only process isolation). So "all CPU cores at once" here
//! means a worker pool of `--workers` **OS processes** (this same binary,
//! re-invoked with `--worker`), each assigned a contiguous slice of the
//! case-index space and running its cases one at a time, serialised,
//! inside itself -- the second option the task brief allows, combined with
//! the first (separate processes) for the actual cross-core parallelism.
//!
//! ## Hang handling
//!
//! A case (see the brief's "PW980 APU start may hang") runs on a helper
//! thread inside the worker process; the worker's main loop waits on it
//! with a bounded `recv_timeout`. On timeout the case is logged as HUNG
//! and the loop moves on to the next case index *without* joining the
//! stuck thread (there is no safe way to kill a Rust thread) -- it is
//! deliberately abandoned as an orphan. This is safe for correctness (the
//! next case's `Emulator::new()` only ever *briefly* locks the same global
//! mutexes to reset them, so an orphan spinning inside FBW's own systems
//! tick does not deadlock it), at the cost of leaking one CPU worth of
//! spin per hang for the rest of that worker's run -- an acceptable
//! trade-off given the brief names exactly one known hang source.

use std::collections::{BTreeMap, HashSet};
use std::fmt::Write as _;
use std::io::Write as _;
use std::path::PathBuf;
use std::sync::mpsc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use fbw_a380_emulator::{presets, Emulator};
use systems::simulation::StartState;

// =====================================================================
// A tiny in-crate PRNG (splitmix64). No new dependency: the task asks for
// exactly this if `rand` is not already in this crate's own dependency
// tree, which it is not (see emulator/Cargo.toml).
// =====================================================================

#[derive(Clone)]
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        // Avoid the all-zero fixed point.
        Self(seed ^ 0x9E3779B97F4A7C15)
    }
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }
    fn range_f64(&mut self, lo: f64, hi: f64) -> f64 {
        lo + self.next_f64() * (hi - lo)
    }
    /// `[lo, hi)`.
    fn range_usize(&mut self, lo: usize, hi: usize) -> usize {
        if hi <= lo {
            return lo;
        }
        lo + (self.next_u64() as usize) % (hi - lo)
    }
    fn bool_p(&mut self, p: f64) -> bool {
        self.next_f64() < p
    }
}

/// Derives a per-case seed from the master seed and a global case index --
/// deterministic and independent of run order/worker split, so any case
/// index is exactly reproducible standalone (`--only-case`).
fn derive_seed(master: u64, idx: u64) -> u64 {
    let mut s = master ^ idx.wrapping_mul(0xD1B5_4A32_D192_ED03) ^ 0x2545_F491_4F6C_DD1D;
    s = s.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = s;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

// =====================================================================
// Config / CLI
// =====================================================================

#[derive(Clone)]
struct Config {
    master_seed: u64,
    cases: usize,
    workers: usize,
    dump_dir: PathBuf,
    max_gb: f64,
    max_ticks: usize,
    snapshot_every: usize,
    timeout_secs: u64,
    /// Tiered mode: singles, then overlapping pairs, then triples (see
    /// `tiered_main`).
    tiered: bool,
    /// Worker in tiered mode: the tier's plan (one `ComboSpec` per line).
    plan_file: Option<PathBuf>,
    /// Most cases a tier above the first may run (default for each below).
    tier_max: usize,
    /// Per-tier caps: pairs, triples, quads, and the rare random tier.
    t2_max: usize,
    t3_max: usize,
    t4_max: usize,
    t5_max: usize,
    /// Ticks each tiered case runs after its start state.
    tier_ticks: usize,
    /// Tiered: continue the run in `dump_dir`, reusing each tier's plan and
    /// skipping the cases its reach files already hold.
    resume: bool,
    // worker-only
    /// The case indices to work through (default: the shard's range).
    todo_file: Option<PathBuf>,
    /// The shared work queue's markers (`fbw_a380_emulator::work`).
    claim_dir: Option<PathBuf>,
    chunk: usize,
    worker: bool,
    shard_id: usize,
    shard_start: usize,
    shard_end: usize,
}

fn parse_args() -> Config {
    let args: Vec<String> = std::env::args().collect();
    let get = |flag: &str| -> Option<String> {
        args.iter().position(|a| a == flag).and_then(|i| args.get(i + 1)).cloned()
    };
    let default_seed = || -> u64 {
        SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_nanos() as u64).unwrap_or(0xC0FF_EE12_3456_789A)
    };
    let master_seed = get("--master-seed").and_then(|s| s.parse().ok()).unwrap_or_else(default_seed);
    let cases = get("--cases").and_then(|s| s.parse().ok()).unwrap_or(10_000usize);
    let workers =
        get("--workers").and_then(|s| s.parse().ok()).unwrap_or_else(|| std::thread::available_parallelism().map(|n| n.get()).unwrap_or(4));
    let dump_dir = get("--dump-dir").map(PathBuf::from).unwrap_or_else(|| default_dump_dir(master_seed));
    let max_gb = get("--max-gb").and_then(|s| s.parse().ok()).unwrap_or(20.0);
    let max_ticks = get("--max-ticks").and_then(|s| s.parse().ok()).unwrap_or(2_000usize);
    let snapshot_every = get("--snapshot-every").and_then(|s| s.parse().ok()).unwrap_or(10usize).max(1);
    let timeout_secs = get("--timeout-secs").and_then(|s| s.parse().ok()).unwrap_or(20u64);
    let worker = args.iter().any(|a| a == "--worker");
    let tiered = args.iter().any(|a| a == "--tiered");
    let plan_file = get("--plan-file").map(PathBuf::from);
    let tier_max = get("--tier-max").and_then(|s| s.parse().ok()).unwrap_or(20_000usize);
    let t2_max = get("--t2-max").and_then(|s| s.parse().ok()).unwrap_or(tier_max);
    let t3_max = get("--t3-max").and_then(|s| s.parse().ok()).unwrap_or(tier_max);
    let t4_max = get("--t4-max").and_then(|s| s.parse().ok()).unwrap_or(tier_max);
    let t5_max = get("--t5-max").and_then(|s| s.parse().ok()).unwrap_or((tier_max / 20).max(1));
    // At TIER_DT = 0.1 s: 20 s of simulated time after the start state.
    let tier_ticks = get("--tier-ticks").and_then(|s| s.parse().ok()).unwrap_or(200usize);
    let shard_id = get("--shard-id").and_then(|s| s.parse().ok()).unwrap_or(0);
    let shard_start = get("--shard-start").and_then(|s| s.parse().ok()).unwrap_or(0);
    let shard_end = get("--shard-end").and_then(|s| s.parse().ok()).unwrap_or(0);
    let resume = args.iter().any(|a| a == "--resume");
    let todo_file = get("--todo-file").map(PathBuf::from);
    let claim_dir = get("--claim-dir").map(PathBuf::from);
    let chunk = get("--chunk").and_then(|s| s.parse().ok()).unwrap_or(16usize);
    Config {
        resume,
        todo_file,
        claim_dir,
        chunk,
        master_seed,
        cases,
        workers,
        dump_dir,
        max_gb,
        max_ticks,
        snapshot_every,
        timeout_secs,
        tiered,
        plan_file,
        tier_max,
        t2_max,
        t3_max,
        t4_max,
        t5_max,
        tier_ticks,
        worker,
        shard_id,
        shard_start,
        shard_end,
    }
}

/// `days_from_civil` (Howard Hinnant's algorithm) -- avoids adding a time/
/// date crate just to name the dump directory.
fn default_dump_dir(seed: u64) -> PathBuf {
    let secs = SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0) as i64;
    let days = secs.div_euclid(86_400);
    let tod = secs.rem_euclid(86_400);
    let (h, m, s) = (tod / 3600, (tod % 3600) / 60, tod % 60);
    let z = days + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = (z - era * 146_097) as u64;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146_096) / 365;
    let y = yoe as i64 + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m_ = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if m_ <= 2 { y + 1 } else { y };
    let ts = format!("{y:04}{m_:02}{d:02}_{h:02}{m:02}{s:02}");
    PathBuf::from(format!("D:\\fbw-test-dumps\\{ts}_seed{seed}"))
}

// =====================================================================
// Case plan: structured (enumerable) + random (continuous). Both are
// computed identically in every process (orchestrator to size shards,
// each worker to resolve its own slice) purely from static catalogue
// data, so no case data needs to cross a process boundary.
// =====================================================================

#[derive(Clone, Debug)]
enum CaseKind {
    /// Every failure id alone, at several magnitudes.
    FailureAlone { id: u64, magnitude: f64 },
    /// Every breaker pulled alone.
    BreakerAlone { id: &'static str },
    /// All pairs of failures within the same ATA chapter (failure ids are
    /// `ata*1000 + n`, e.g. `24_004` -> ATA 24; see `failures.rs`).
    FailurePair { a: u64, b: u64, ata: u16 },
    /// The continuous, seeded, combinatorial space.
    Random { index: usize },
    /// A normal-operations procedure/flight-phase scenario, driven through
    /// real cockpit-control inputs where the emulator can reach them (see
    /// [`run_procedure`]'s doc comment for exactly what is and is not
    /// causally reachable offline). `combo` indexes the fixed
    /// expedited x environment x mid-procedure-failure grid built by
    /// [`ProcedureSpec::structured`].
    Procedure { combo: usize },
    /// Tiered mode: faults applied together on one start state.
    Combo(ComboSpec),
}

/// Tiered mode's unit: a start state (`base_preset`'s index) and the faults
/// applied on it together, failures at full magnitude and breakers pulled.
/// One per line in a tier's plan file: `preset|f1,f2|b1,b2`.
#[derive(Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
struct ComboSpec {
    preset: usize,
    failures: Vec<u64>,
    breakers: Vec<String>,
}

impl ComboSpec {
    fn to_line(&self) -> String {
        let f: Vec<String> = self.failures.iter().map(u64::to_string).collect();
        format!("{}|{}|{}", self.preset, f.join(","), self.breakers.join(","))
    }

    fn from_line(line: &str) -> Option<Self> {
        let mut parts = line.trim_end().splitn(3, '|');
        let preset = parts.next()?.parse().ok()?;
        let failures = parts.next()?.split(',').filter(|x| !x.is_empty()).filter_map(|x| x.parse().ok()).collect();
        let breakers = parts.next().unwrap_or("").split(',').filter(|x| !x.is_empty()).map(str::to_owned).collect();
        Some(Self { preset, failures, breakers })
    }

    /// The faults as element keys ("f:ID" / "b:ID").
    fn elements(&self) -> Vec<String> {
        self.failures.iter().map(|f| format!("f:{f}")).chain(self.breakers.iter().map(|b| format!("b:{b}"))).collect()
    }

    fn with(&self, element: &str) -> Self {
        let mut next = self.clone();
        if let Some(f) = element.strip_prefix("f:") {
            next.failures.push(f.parse().unwrap_or(0));
            next.failures.sort_unstable();
        } else if let Some(b) = element.strip_prefix("b:") {
            next.breakers.push(b.to_owned());
            next.breakers.sort();
        }
        next
    }
}

const MAGNITUDES: [f64; 5] = [0.1, 0.25, 0.5, 0.75, 1.0];

fn build_structured_plan() -> Vec<CaseKind> {
    // A throwaway instance purely to read the plugin's own catalogues
    // (`list_failures`/`list_breaker_ids`) -- never ticked or reused; its
    // `Emulator::new` already resets the global state it touched.
    let e = Emulator::new(StartState::Apron);
    let failure_ids: Vec<u64> = e.list_failures();
    let breaker_ids: Vec<&'static str> = e.list_breaker_ids();
    drop(e);

    let mut plan = Vec::new();
    for &id in &failure_ids {
        for &m in &MAGNITUDES {
            plan.push(CaseKind::FailureAlone { id, magnitude: m });
        }
    }
    for &id in &breaker_ids {
        plan.push(CaseKind::BreakerAlone { id });
    }
    let mut by_ata: BTreeMap<u16, Vec<u64>> = BTreeMap::new();
    for &id in &failure_ids {
        by_ata.entry((id / 1000) as u16).or_default().push(id);
    }
    for (ata, ids) in &by_ata {
        for i in 0..ids.len() {
            for j in (i + 1)..ids.len() {
                plan.push(CaseKind::FailurePair { a: ids[i], b: ids[j], ata: *ata });
            }
        }
    }
    // expedited{false,true} x environment{normal,hot/high-alt,cold} x
    // mid-procedure-failure{no,yes} = 12 canonical procedure scenarios.
    for combo in 0..12 {
        plan.push(CaseKind::Procedure { combo });
    }
    plan
}

fn resolve_case(index: usize, structured: &[CaseKind]) -> CaseKind {
    if index < structured.len() {
        structured[index].clone()
    } else {
        CaseKind::Random { index: index - structured.len() }
    }
}

// =====================================================================
// Per-case diagnostics and outcome.
// =====================================================================

#[derive(Default)]
struct CaseOutcome {
    /// Tiered mode: variables this case moved away from the fault-free
    /// baseline of the same start state and duration.
    reach: Vec<String>,
    index: usize,
    seed: u64,
    kind_desc: String,
    ticks_completed: usize,
    ticks_planned: usize,
    panicked: bool,
    panic_msg: Option<String>,
    nan_detected: bool,
    first_nan_var: Option<String>,
    first_nan_tick: Option<usize>,
    invariant_counts: BTreeMap<String, u64>,
    sanity_issues: Vec<String>,
    bytes_written: u64,
}

impl CaseOutcome {
    fn passed(&self) -> bool {
        // `not_covered:` notes record what this bench cannot drive offline;
        // they are tallied in the summary but are not failures.
        !self.panicked
            && !self.nan_detected
            && self.invariant_counts.is_empty()
            && self.sanity_issues.iter().all(|s| s.starts_with("not_covered:"))
    }
}

fn json_escape(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 4);
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            _ => out.push(c),
        }
    }
    out
}

/// One "snapshot+check" pass: writes a `tick` JSONL line (if under the
/// byte budget) and folds NaN/invariant/sanity findings into `out`.
/// Returns bytes written (0 if the case is under a budget hold-back but
/// still checked -- checks always run; only the *dump* is gated).
/// The variable names and the index lists the name-based checks use,
/// refreshed only when the variable set grows. Per thread: each case runs
/// on its own thread.
#[derive(Default)]
struct NameCache {
    names: Vec<String>,
    battery_potential: Vec<usize>,
    battery_charge: Vec<usize>,
    fuel_quantity: Vec<usize>,
}

thread_local! {
    static NAMES: std::cell::RefCell<NameCache> = std::cell::RefCell::new(NameCache::default());
}

fn index_where(names: &[String], pred: impl Fn(&str) -> bool) -> Vec<usize> {
    names.iter().enumerate().filter(|(_, n)| pred(n)).map(|(i, _)| i).collect()
}

#[allow(clippy::too_many_arguments)]
fn snapshot_and_check(
    e: &mut Emulator,
    out: &mut CaseOutcome,
    case_index: usize,
    tick: usize,
    dt: f64,
    writer: &mut impl std::io::Write,
    remaining_budget: &mut i64,
    prev_fuel_sum: &mut Option<f64>,
    write: bool,
) -> u64 {
    let was_nan = out.nan_detected;
    // Values only, every tick; the names are fetched once and again only
    // when the set of variables grows (it settles within the first ticks),
    // along with the index lists the name-based checks below need. Copying
    // all ~5,300 names every tick cost about half a physics tick.
    let values = e.values();
    NAMES.with(|cell| {
        let mut cache = cell.borrow_mut();
        if cache.names.len() != values.len() {
            let snap = e.snapshot_all();
            cache.names = snap.into_iter().map(|(n, _)| n).collect();
            cache.battery_potential = index_where(&cache.names, |n| n.contains("BAT") && n.contains("POTENTIAL"));
            cache.battery_charge = index_where(&cache.names, |n| n.contains("BAT") && (n.contains("CHARGE") || n.contains("SOC")));
            cache.fuel_quantity = index_where(&cache.names, |n| n.contains("FUEL") && (n.contains("QUANTITY") || n.contains("QTY")));
        }
    });
    let name_of = |i: usize| NAMES.with(|c| c.borrow().names.get(i).cloned().unwrap_or_default());
    if !out.nan_detected {
        if let Some(i) = values.iter().position(|v| !v.is_finite()) {
            out.nan_detected = true;
            out.first_nan_var = Some(name_of(i));
            out.first_nan_tick = Some(tick);
        }
    }

    for v in e.invariant_report() {
        let counter = out.invariant_counts.entry(v.name.clone()).or_insert(0);
        *counter = (*counter).max(v.total);
    }

    // Battery charge within bounds (heuristic: name contains BAT).
    let (bat_v, bat_c, fuel_q) =
        NAMES.with(|c| {
            let c = c.borrow();
            (c.battery_potential.clone(), c.battery_charge.clone(), c.fuel_quantity.clone())
        });
    for &i in &bat_v {
        let value = values[i];
        if value < -5.0 || value > 36.0 {
            out.sanity_issues.push(format!("battery_potential_out_of_bounds:{}={value:.2}", name_of(i)));
        }
    }
    for &i in &bat_c {
        let value = values[i];
        if value < -1.0 || value > 101.0 {
            out.sanity_issues.push(format!("battery_charge_out_of_bounds:{}={value:.2}", name_of(i)));
        }
    }

    // Breaker open => no current through it. (Its supply bus staying
    // powered is normal: a pulled breaker de-powers what it feeds, and a
    // pulled bus feeder reroutes through the tie.)
    for bs in e.breaker_states() {
        if !bs.closed {
            if bs.current_a.abs() > 0.5 {
                out.sanity_issues.push(format!("breaker_open_but_current_flowing:{}={:.2}", bs.id, bs.current_a));
            }
        }
    }

    // Fuel mass heuristic: total of anything that looks like a fuel
    // quantity/mass should not jump up between snapshots (no refuel
    // modelled by this battery).
    let fuel_sum: f64 = fuel_q.iter().map(|&i| values[i]).sum();
    if let Some(prev) = *prev_fuel_sum {
        if fuel_sum > prev + 1.0 {
            out.sanity_issues.push(format!("fuel_mass_increased_without_refuel:{prev:.1}->{fuel_sum:.1}"));
        }
    }
    *prev_fuel_sum = Some(fuel_sum);

    // Checked every tick, so the same anomaly repeats: keep its first
    // occurrence per kind (the text before '=').
    let mut seen = std::collections::HashSet::new();
    out.sanity_issues.retain(|s| seen.insert(s.split('=').next().unwrap_or(s).to_owned()));

    // Written only on the snapshot schedule, plus the tick a NaN first
    // appears so its dump holds the moment it went bad.
    let first_nan_now = !was_nan && out.nan_detected;
    if !(write || first_nan_now) || *remaining_budget <= 0 {
        return 0;
    }

    let snap = e.snapshot_all();
    let mut line = String::with_capacity(4096);
    let _ = write!(line, "{{\"type\":\"tick\",\"case\":{case_index},\"tick\":{tick},\"time_s\":{:.3},\"dt\":{dt},\"vars\":{{", e.elapsed_s());
    for (i, (name, value)) in snap.iter().enumerate() {
        if i > 0 {
            line.push(',');
        }
        let v = if value.is_finite() { format!("{value}") } else if value.is_nan() { "\"NaN\"".to_owned() } else { "\"Inf\"".to_owned() };
        let _ = write!(line, "\"{}\":{}", json_escape(name), v);
    }
    line.push_str("}}\n");
    let bytes = line.as_bytes().len() as u64;
    let _ = writer.write_all(line.as_bytes());
    *remaining_budget -= bytes as i64;
    bytes
}

// =====================================================================
// Tiered mode runner.
// =====================================================================

/// The fault-free end state of each start state after `tier_ticks`, per
/// worker process (deterministic, so every worker computes the same one).
static BASELINES: std::sync::Mutex<BTreeMap<(usize, usize), std::sync::Arc<Vec<(String, f64)>>>> =
    std::sync::Mutex::new(BTreeMap::new());

/// Time step of tiered cases: 0.1 s, twice the rest of the battery's. The
/// engine and APU models split large steps internally, and the start states
/// are already built at this step (`presets::PRESET_DT`), so each tick covers
/// twice the simulated time for the same cost.
const TIER_DT: f64 = 0.1;

/// Empty (fault-free) cases per start state in tier 1, measuring its noise.
const NOISE_CASES: usize = 8;

/// Most elements in one tier-2 group case.
const GROUP_MAX: usize = 10;

fn baseline(preset: usize, ticks: usize) -> std::sync::Arc<Vec<(String, f64)>> {
    if let Some(b) = BASELINES.lock().ok().and_then(|m| m.get(&(preset, ticks)).cloned()) {
        return b;
    }
    let (mut e, _) = base_preset(preset);
    e.run(TIER_DT, ticks as u32);
    let snap = std::sync::Arc::new(e.snapshot_all());
    if let Ok(mut m) = BASELINES.lock() {
        m.insert((preset, ticks), snap.clone());
    }
    snap
}

/// Whether a value moved from its baseline: beyond a relative tolerance
/// (with a small absolute floor), or went non-finite.
fn moved(value: f64, base: f64) -> bool {
    if !value.is_finite() || !base.is_finite() {
        return value.is_finite() != base.is_finite();
    }
    (value - base).abs() > 1e-3 * base.abs().max(1.0)
}

fn run_combo(spec: &ComboSpec, index: usize, cfg: &Config, writer: &mut impl std::io::Write, remaining_budget: &mut i64) -> CaseOutcome {
    let seed = derive_seed(cfg.master_seed, index as u64);
    let (mut e, preset) = base_preset(spec.preset);
    let mut out = CaseOutcome { index, seed, kind_desc: format!("{} from {preset}", spec.to_line()), ..Default::default() };
    check_preset_reached(&mut e, &mut out, preset);
    for &f in &spec.failures {
        e.set_failure_magnitude(f, 1.0);
    }
    for b in &spec.breakers {
        if let Some(id) = e.list_breaker_ids().into_iter().find(|x| x == b) {
            e.pull_breaker(id);
        }
    }
    let ticks = cfg.tier_ticks;
    out.ticks_planned = ticks;
    let mut prev_fuel = None;
    for tick in 0..ticks {
        e.tick(TIER_DT);
        out.ticks_completed = tick + 1;
        // Tiered runs are large: a snapshot is kept only of a case that has
        // failed, at its last tick (and the tick a NaN first appears, which
        // `snapshot_and_check` always keeps). A passing case keeps just its
        // reach record.
        let write = tick + 1 == ticks && !out.passed();
        let b = snapshot_and_check(&mut e, &mut out, index, tick, TIER_DT, writer, remaining_budget, &mut prev_fuel, write);
        out.bytes_written += b;
    }
    let base = baseline(spec.preset, ticks);
    let base_by_name: BTreeMap<&str, f64> = base.iter().map(|(n, v)| (n.as_str(), *v)).collect();
    out.reach = e
        .snapshot_all()
        .into_iter()
        .filter(|(n, v)| base_by_name.get(n.as_str()).map_or(true, |b| moved(*v, *b)))
        .map(|(n, _)| n)
        .collect();
    out
}

// =====================================================================
// Structured case runners.
// =====================================================================

fn run_structured(kind: &CaseKind, index: usize, cfg: &Config, writer: &mut impl std::io::Write, remaining_budget: &mut i64) -> CaseOutcome {
    let seed = derive_seed(cfg.master_seed, index as u64);
    let (mut e, preset) = base_preset(index);
    let mut out = CaseOutcome { index, seed, kind_desc: format!("{kind:?} from {preset}"), ..Default::default() };
    check_preset_reached(&mut e, &mut out, preset);
    let dt = 0.05;
    let ticks = 300usize.min(cfg.max_ticks);
    out.ticks_planned = ticks;

    match kind {
        CaseKind::FailureAlone { id, magnitude } => e.set_failure_magnitude(*id, *magnitude),
        CaseKind::BreakerAlone { id } => e.pull_breaker(id),
        CaseKind::FailurePair { a, b, .. } => {
            e.set_failure_magnitude(*a, 0.5);
            e.set_failure_magnitude(*b, 0.5);
        }
        CaseKind::Random { .. } => unreachable!("run_structured called with a Random case"),
        CaseKind::Procedure { .. } => unreachable!("run_structured called with a Procedure case"),
        CaseKind::Combo(_) => unreachable!("run_structured called with a Combo case"),
    }

    let mut prev_fuel = None;
    for tick in 0..ticks {
        e.tick(dt);
        out.ticks_completed = tick + 1;
        let write = tick % cfg.snapshot_every == 0 || tick + 1 == ticks;
        let b = snapshot_and_check(&mut e, &mut out, index, tick, dt, writer, remaining_budget, &mut prev_fuel, write);
        out.bytes_written += b;
    }
    out
}

// =====================================================================
// Procedure/normal-operations runner: cold-and-dark -> battery -> EXT PWR
// -> APU start -> APU bleed -> (engine starts: not reachable, see below)
// -> electrical reconfig -> hydraulic pump selection -> fuel panel (real
// cockpit controls only) -> bleed/pack/anti-ice -> mid-procedure partial
// failure -> gear/flaps cycles -> breaker pulls -> taxi/takeoff/climb/
// cruise/descent/landing envelope -> shutdown.
//
// What is and is not causally reachable offline, and why:
// - Battery-on (`A32NX_OVHD_ELEC_BAT_<id>_PB_IS_AUTO`) and APU
//   master/start (`A32NX_OVHD_APU_MASTER_SW_PB_IS_ON`/`_START_PB_IS_ON`)
//   are confirmed causal: they are the exact two inputs
//   `emulator/src/presets.rs`'s own `powered()` preset uses, and that
//   preset is proven (by its own doc comment and the coverage test) to
//   bring the aircraft up for real.
// - EXT PWR/GEN/bus-tie/pack/hydraulic-pump input names are grepped
//   verbatim out of `a380_systems`'s own `write_by_name`/component-name
//   calls (electrical/mod.rs, air_conditioning/mod.rs, hydraulic/mod.rs),
//   so they are real names, but this pass did not independently prove
//   each one is the read side the same string binds to -- effects are
//   only asserted through the separately-confirmed
//   `A32NX_ELEC_AC_1_BUS_IS_POWERED` observation, never through echoing
//   the input back.
// - Engine start (`engine_commands.rs`) and fuel pump/crossfeed/transfer
//   (`fuel.rs`) are explicitly out of scope per `emulator/README.md` --
//   both are `Xplm`-bound in the live plugin. This runner does not
//   pretend to drive them: engine start is logged as `not_covered`
//   outright, and the fuel panel is only touched through real
//   `fbw/cockpit/*` controls (`Emulator::list_controls`/`set_dataref`),
//   never a guessed simvar name -- if `cockpit_bindings.txt` is not
//   present (see that module's doc comment) or has no `*FUEL*` control,
//   that is logged as `not_covered` too, not silently skipped.
// - "Takeoff/climb/cruise/descent" here is a flight-*envelope*
//   progression (on-ground flag, pressure altitude, IAS -- the same
//   inputs `presets::cruise()` itself uses), not a causally modelled
//   takeoff roll/climb performance, which would need engine thrust
//   through the same unreachable `engine_commands.rs` coupling.

#[derive(Clone, Debug)]
struct ProcedureSpec {
    expedited: bool,
    oat_c: f64,
    alt_ft: f64,
    mid_failure: Option<(u64, f64)>,
}

impl ProcedureSpec {
    /// The 12-entry structured grid: expedited{false,true} x
    /// environment{normal,hot/high-alt,cold} x mid-failure{no,yes}.
    fn structured(combo: usize, failure_ids: &[u64], rng: &mut Rng) -> Self {
        let expedited = combo % 2 == 1;
        let env = (combo / 2) % 3;
        let with_failure = (combo / 6) % 2 == 1;
        let (oat_c, alt_ft) = match env {
            0 => (15.0, 1500.0),
            1 => (48.0, 41_000.0),
            _ => (-55.0, 39_000.0),
        };
        let mid_failure =
            if with_failure && !failure_ids.is_empty() { Some((failure_ids[rng.range_usize(0, failure_ids.len())], rng.range_f64(0.3, 1.0))) } else { None };
        Self { expedited, oat_c, alt_ft, mid_failure }
    }

    /// The continuous, seeded version: same axes, sampled.
    fn random(rng: &mut Rng, failure_ids: &[u64]) -> Self {
        let expedited = rng.bool_p(0.5);
        let (oat_c, alt_ft) = match rng.range_usize(0, 3) {
            0 => (rng.range_f64(0.0, 25.0), rng.range_f64(0.0, 10_000.0)),
            1 => (rng.range_f64(40.0, 55.0), rng.range_f64(30_000.0, 43_000.0)),
            _ => (rng.range_f64(-60.0, -30.0), rng.range_f64(25_000.0, 41_000.0)),
        };
        let mid_failure = if rng.bool_p(0.5) && !failure_ids.is_empty() { Some((failure_ids[rng.range_usize(0, failure_ids.len())], rng.range_f64(0.0, 1.0))) } else { None };
        Self { expedited, oat_c, alt_ft, mid_failure }
    }
}

/// Staged tick count, or a shortened "expedited" one.
fn scaled(staged: usize, expedited: bool) -> usize {
    if expedited {
        (staged / 3).max(5)
    } else {
        staged
    }
}

/// Ticks `e` by `wanted` steps of `dt`, clamped to what remains of the
/// case's hard tick cap (`budget`), and advances `global`/`out` -- purely
/// mechanical bookkeeping factored out so each phase below is one line.
macro_rules! phase_ticks {
    ($e:expr, $dt:expr, $budget:expr, $global:expr, $out:expr, $wanted:expr, $index:expr, $writer:expr, $remaining:expr, $prev_fuel:expr) => {{
        let t = ($wanted).min($budget);
        for i in 0..t {
            $e.tick($dt);
            let b = snapshot_and_check(&mut $e, &mut $out, $index, $global + i, $dt, $writer, $remaining, &mut $prev_fuel, false);
            $out.bytes_written += b;
        }
        $global += t;
        $budget = $budget.saturating_sub(t);
        $out.ticks_completed = $global;
    }};
}

/// The start state a case asked for, built by the real procedure
/// (`presets`): whether it actually got there is itself a result.
fn base_preset(index: usize) -> (Emulator, &'static str) {
    match index % 4 {
        0 => (presets::cold_and_dark(), "cold_and_dark"),
        1 => (presets::ground_power(), "ground_power"),
        2 => (presets::powered(), "powered"),
        _ => (presets::engines_running(), "engines_running"),
    }
}

fn check_preset_reached(e: &mut Emulator, out: &mut CaseOutcome, preset: &str) {
    match preset {
        "ground_power" | "powered" if e.get_var("A32NX_ELEC_AC_1_BUS_IS_POWERED") < 0.5 => {
            out.sanity_issues.push(format!("preset_not_reached:{preset}:ac_bus_1_unpowered"));
        }
        "powered" if e.get_var("A32NX_OVHD_APU_START_PB_IS_AVAILABLE") < 0.5 => {
            out.sanity_issues.push(format!("preset_not_reached:{preset}:apu_not_avail"));
        }
        "engines_running" => {
            for n in 1..=4u8 {
                if !presets::engine_running(e, n) {
                    let n3 = e.get_var(&format!("ENGINE_N3:{n}"));
                    out.sanity_issues.push(format!("preset_not_reached:{preset}:engine_{n}_not_running=n3 {n3:.1}"));
                }
            }
        }
        _ => {}
    }
}

fn check_bus_powered(e: &mut Emulator, out: &mut CaseOutcome, phase: &str) {
    let name = "A32NX_ELEC_AC_1_BUS_IS_POWERED";
    let snap = e.snapshot_all();
    match snap.iter().find(|(n, _)| n == name) {
        Some((_, v)) => {
            if *v < 0.5 {
                out.sanity_issues.push(format!("phase_bus_not_powered:{phase}:{name}={v}"));
            }
        }
        None => out.sanity_issues.push(format!("not_covered:bus_powered_var({name} not registered in this build)")),
    }
}

fn check_apu_available(e: &mut Emulator, out: &mut CaseOutcome) {
    // FBW's own AVAIL signal: the APU START pushbutton's AVAIL light.
    let candidates = ["A32NX_OVHD_APU_START_PB_IS_AVAILABLE"];
    let snap = e.snapshot_all();
    match candidates.iter().find(|c| snap.iter().any(|(n, _)| n == *c)) {
        Some(&name) => {
            let v = snap.iter().find(|(n, _)| n == name).map(|(_, v)| *v).unwrap_or(0.0);
            // fuel.rs (X-Plane-bound) is the only writer of the APU feed
            // pressure; with nothing feeding it the ECB correctly refuses to
            // start, which is a gap in this bench, not in the aircraft.
            let feed_psi = e.get_var("A32NX_APU_FUEL_FEED_PRESSURE_PSI");
            if v < 0.5 && feed_psi <= 0.0 {
                out.sanity_issues.push("not_covered:apu_start (no offline fuel system feeds APU_FUEL_FEED_PRESSURE_PSI)".to_owned());
            } else if v < 0.5 {
                out.sanity_issues.push(format!("phase_apu_not_available_within_timeout:{name}={v}"));
            }
        }
        None => out.sanity_issues.push("not_covered:apu_available_var (none of the candidate names are registered in this build)".to_owned()),
    }
}

#[allow(clippy::too_many_arguments)]
fn run_procedure(
    global_index: usize,
    seed: u64,
    cfg: &Config,
    failure_ids: &[u64],
    breaker_ids: &[&'static str],
    controls: &[fbw_a380_emulator::controls::Control],
    writer: &mut impl std::io::Write,
    remaining_budget: &mut i64,
    spec: ProcedureSpec,
) -> CaseOutcome {
    let mut rng = Rng::new(seed ^ 0x1357_9BDF_2468_ACE0);
    let mut out = CaseOutcome { index: global_index, seed, ..Default::default() };
    let dt = 0.05;
    let mut tick_budget = cfg.max_ticks;
    let mut global_tick = 0usize;
    let mut prev_fuel = None;

    {
        let mut hdr = String::new();
        let mf = match spec.mid_failure {
            Some((id, m)) => format!("[{id},{m:.3}]"),
            None => "null".to_owned(),
        };
        let _ = write!(
            hdr,
            "{{\"type\":\"case_header\",\"case\":{global_index},\"seed\":{seed},\"kind\":\"procedure\",\"expedited\":{},\"oat_c\":{:.2},\"alt_ft\":{:.1},\"mid_failure\":{mf}}}\n",
            spec.expedited, spec.oat_c, spec.alt_ft
        );
        let bytes = hdr.as_bytes().len() as u64;
        let _ = writer.write_all(hdr.as_bytes());
        out.bytes_written += bytes;
        *remaining_budget -= bytes as i64;
    }

    let mut e = presets::cold_and_dark();
    e.set_oat_c(spec.oat_c);
    e.set_pressure_altitude_ft(spec.alt_ft.max(0.0));

    // Battery on (confirmed causal -- see the module doc comment above).
    for bat in ["BAT_1", "BAT_2", "BAT_ESS", "BAT_APU"] {
        e.set_var(&format!("A32NX_OVHD_ELEC_{bat}_PB_IS_AUTO"), 1.0);
    }
    phase_ticks!(e, dt, tick_budget, global_tick, out, scaled(40, spec.expedited), global_index, writer, remaining_budget, prev_fuel);
    {
        let b = snapshot_and_check(&mut e, &mut out, global_index, global_tick, dt, writer, remaining_budget, &mut prev_fuel, true);
        out.bytes_written += b;
    }

    // Ground power cart connected, then EXT PWR on.
    e.set_var("EXT_PWR_AVAIL:1", 1.0);
    e.set_var("A32NX_OVHD_ELEC_EXT_PWR_1_PB_IS_ON", 1.0);
    phase_ticks!(e, dt, tick_budget, global_tick, out, scaled(20, spec.expedited), global_index, writer, remaining_budget, prev_fuel);
    check_bus_powered(&mut e, &mut out, "after_ext_pwr");

    // APU start (confirmed causal).
    e.set_var("A32NX_OVHD_APU_MASTER_SW_PB_IS_ON", 1.0);
    phase_ticks!(e, dt, tick_budget, global_tick, out, scaled(20, spec.expedited), global_index, writer, remaining_budget, prev_fuel);
    e.set_var("A32NX_OVHD_APU_START_PB_IS_ON", 1.0);
    // The PW980 takes ~49 s to AVAIL (FBW apu tests' APPROXIMATE_STARTUP_TIME);
    // that is engine time, so it is not shortened for expedited runs. 70 s.
    phase_ticks!(e, dt, tick_budget, global_tick, out, 1_400, global_index, writer, remaining_budget, prev_fuel);
    check_apu_available(&mut e, &mut out);
    check_bus_powered(&mut e, &mut out, "after_apu_start");

    // APU bleed (best-effort input name).
    e.set_var("A32NX_OVHD_APU_BLEED_PB_IS_ON", 1.0);
    phase_ticks!(e, dt, tick_budget, global_tick, out, scaled(20, spec.expedited), global_index, writer, remaining_budget, prev_fuel);

    // Engine starts with the real controls: IGN/START, then each master,
    // waiting on the FADEC's own ENGINE_STATE (engine time, so not shortened
    // for expedited runs; up to 120 s each, within the case's tick budget).
    for n in 1..=4 {
        e.set_var(&format!("TURB ENG IGNITION SWITCH EX1:{n}"), 2.0);
    }
    for n in [4u8, 3, 2, 1] {
        e.set_var(&format!("GENERAL ENG STARTER:{n}"), 1.0);
        let mut waited = 0usize;
        while !presets::engine_running(&mut e, n) && waited < 2_400 && tick_budget > 0 {
            phase_ticks!(e, dt, tick_budget, global_tick, out, 20, global_index, writer, remaining_budget, prev_fuel);
            waited += 20;
        }
        if !presets::engine_running(&mut e, n) {
            let n3 = e.get_var(&format!("ENGINE_N3:{n}"));
            out.sanity_issues.push(format!("phase_engine_not_started:engine_{n}=n3 {n3:.1}"));
        }
    }
    for n in 1..=4 {
        e.set_var(&format!("TURB ENG IGNITION SWITCH EX1:{n}"), 1.0);
    }

    // Electrical reconfiguration: GEN/APU GEN/EXT PWR/bus tie, random
    // order, random on/off.
    let mut elec_actions: Vec<&str> =
        vec!["A32NX_OVHD_ELEC_APU_GEN_1_PB_IS_ON", "A32NX_OVHD_ELEC_APU_GEN_2_PB_IS_ON", "A32NX_OVHD_ELEC_EXT_PWR_1_PB_IS_ON", "A32NX_OVHD_ELEC_BUS_TIE_PB_IS_AUTO"];
    for i in (1..elec_actions.len()).rev() {
        let j = rng.range_usize(0, i + 1);
        elec_actions.swap(i, j);
    }
    let mut any_source_on = false;
    for name in &elec_actions {
        let on = rng.bool_p(0.5);
        // An APU GEN only supplies once the APU itself is AVAIL.
        let supplies = if name.contains("APU_GEN") { e.get_var("A32NX_OVHD_APU_START_PB_IS_AVAILABLE") > 0.5 } else { name.contains("EXT_PWR") };
        if on && supplies {
            any_source_on = true;
        }
        e.set_var(name, if on { 1.0 } else { 0.0 });
        phase_ticks!(e, dt, tick_budget, global_tick, out, scaled(10, spec.expedited), global_index, writer, remaining_budget, prev_fuel);
    }
    // With EXT PWR off and no APU GEN that can supply, there is no source: AC 1 unpowered
    // is then the right answer, not an anomaly.
    if any_source_on {
        check_bus_powered(&mut e, &mut out, "after_elec_reconfig");
    }

    // Hydraulic pump selections (confirmed input names).
    let mut hyd_actions: Vec<&str> = vec![
        "A32NX_OVHD_HYD_EPUMPYA_ON_PB_IS_AUTO",
        "A32NX_OVHD_HYD_EPUMPYB_ON_PB_IS_AUTO",
        "A32NX_OVHD_HYD_EPUMPGA_ON_PB_IS_AUTO",
        "A32NX_OVHD_HYD_EPUMPGB_ON_PB_IS_AUTO",
        "A32NX_OVHD_HYD_ENG_1A_PUMP_PB_IS_AUTO",
        "A32NX_OVHD_HYD_ENG_3A_PUMP_PB_IS_AUTO",
    ];
    for i in (1..hyd_actions.len()).rev() {
        let j = rng.range_usize(0, i + 1);
        hyd_actions.swap(i, j);
    }
    for name in &hyd_actions {
        e.set_var(name, if rng.bool_p(0.5) { 1.0 } else { 0.0 });
        phase_ticks!(e, dt, tick_budget, global_tick, out, scaled(10, spec.expedited), global_index, writer, remaining_budget, prev_fuel);
    }
    {
        let b = snapshot_and_check(&mut e, &mut out, global_index, global_tick, dt, writer, remaining_budget, &mut prev_fuel, true);
        out.bytes_written += b;
    }

    // Fuel pump/crossfeed/transfer panel: real cockpit controls only (fuel.rs
    // itself runs in the emulator's engine chain).
    let fuel_controls: Vec<&fbw_a380_emulator::controls::Control> = controls.iter().filter(|c| c.dataref.to_uppercase().contains("FUEL")).collect();
    if fuel_controls.is_empty() {
        out.sanity_issues.push(
            "not_covered:fuel_pump_crossfeed_transfer (no fbw/cockpit/*FUEL* control found -- cockpit_bindings.txt absent or empty)"
                .to_owned(),
        );
    } else {
        for c in fuel_controls.iter().take(6) {
            e.set_dataref(&c.dataref, if rng.bool_p(0.5) { 1.0 } else { 0.0 });
            phase_ticks!(e, dt, tick_budget, global_tick, out, scaled(5, spec.expedited), global_index, writer, remaining_budget, prev_fuel);
        }
    }

    // Bleed/pack/anti-ice selections.
    e.set_var("A32NX_OVHD_COND_PACK_1_PB_IS_ON", if rng.bool_p(0.7) { 1.0 } else { 0.0 });
    e.set_var("A32NX_OVHD_COND_PACK_2_PB_IS_ON", if rng.bool_p(0.7) { 1.0 } else { 0.0 });
    phase_ticks!(e, dt, tick_budget, global_tick, out, scaled(10, spec.expedited), global_index, writer, remaining_budget, prev_fuel);
    let ice_controls: Vec<&fbw_a380_emulator::controls::Control> =
        controls.iter().filter(|c| { let u = c.dataref.to_uppercase(); u.contains("ICE") || u.contains("ANTI") }).collect();
    if ice_controls.is_empty() {
        out.sanity_issues.push("not_covered:anti_ice (no fbw/cockpit/*ICE*/*ANTI* control found)".to_owned());
    } else {
        for c in ice_controls.iter().take(4) {
            e.set_dataref(&c.dataref, if rng.bool_p(0.5) { 1.0 } else { 0.0 });
            phase_ticks!(e, dt, tick_budget, global_tick, out, scaled(5, spec.expedited), global_index, writer, remaining_budget, prev_fuel);
        }
    }
    {
        let b = snapshot_and_check(&mut e, &mut out, global_index, global_tick, dt, writer, remaining_budget, &mut prev_fuel, true);
        out.bytes_written += b;
    }

    // Mid-procedure partial failure injection.
    if let Some((id, mag)) = spec.mid_failure {
        e.set_failure_magnitude(id, mag);
        phase_ticks!(e, dt, tick_budget, global_tick, out, scaled(20, spec.expedited), global_index, writer, remaining_budget, prev_fuel);
        let b = snapshot_and_check(&mut e, &mut out, global_index, global_tick, dt, writer, remaining_budget, &mut prev_fuel, true);
        out.bytes_written += b;
    }

    // Gear and flaps/slats cycles (confirmed input names; the A380's
    // single lever drives flaps and slats together, so this exercises
    // both).
    e.set_var("GEAR_LEVER_POSITION_REQUEST", 0.0);
    phase_ticks!(e, dt, tick_budget, global_tick, out, scaled(10, spec.expedited), global_index, writer, remaining_budget, prev_fuel);
    for step in [1.0, 2.0, 3.0, 4.0, 0.0] {
        e.set_var("FLAPS_HANDLE_INDEX", step);
        phase_ticks!(e, dt, tick_budget, global_tick, out, scaled(8, spec.expedited), global_index, writer, remaining_budget, prev_fuel);
    }
    e.set_var("GEAR_LEVER_POSITION_REQUEST", 1.0);
    phase_ticks!(e, dt, tick_budget, global_tick, out, scaled(10, spec.expedited), global_index, writer, remaining_budget, prev_fuel);
    {
        let b = snapshot_and_check(&mut e, &mut out, global_index, global_tick, dt, writer, remaining_budget, &mut prev_fuel, true);
        out.bytes_written += b;
    }

    // Random breaker pulls/resets mid-procedure.
    if !breaker_ids.is_empty() {
        for _ in 0..rng.range_usize(0, 4) {
            let id = breaker_ids[rng.range_usize(0, breaker_ids.len())];
            if rng.bool_p(0.5) {
                e.pull_breaker(id);
            } else {
                e.reset_breaker(id);
            }
            phase_ticks!(e, dt, tick_budget, global_tick, out, scaled(5, spec.expedited), global_index, writer, remaining_budget, prev_fuel);
        }
    }

    // Taxi/takeoff/climb/cruise/descent/landing envelope progression,
    // with a random real cockpit-control change per phase and the
    // environment extremes carried through.
    let phases: [(&str, bool, f64, f64); 6] = [
        ("taxi", true, spec.alt_ft.max(0.0).min(1000.0), 15.0),
        ("takeoff", false, spec.alt_ft.max(0.0).min(1000.0) + 200.0, 160.0),
        ("climb", false, (spec.alt_ft.max(1000.0) * 0.6).min(41_000.0), 280.0),
        ("cruise", false, spec.alt_ft.max(10_000.0).min(43_000.0), 480.0),
        ("descent", false, (spec.alt_ft.max(1000.0) * 0.3).min(20_000.0), 250.0),
        ("landing", true, spec.alt_ft.max(0.0).min(500.0), 140.0),
    ];
    for (name, on_ground, alt, ias) in phases {
        e.set_on_ground(on_ground);
        e.set_pressure_altitude_ft(alt);
        e.set_indicated_airspeed_kt(ias);
        if !controls.is_empty() && rng.bool_p(0.6) {
            let c = &controls[rng.range_usize(0, controls.len())];
            e.set_dataref(&c.dataref, if rng.bool_p(0.5) { 1.0 } else { 0.0 });
        }
        phase_ticks!(e, dt, tick_budget, global_tick, out, scaled(15, spec.expedited), global_index, writer, remaining_budget, prev_fuel);
        let b = snapshot_and_check(&mut e, &mut out, global_index, global_tick, dt, writer, remaining_budget, &mut prev_fuel, true);
        out.bytes_written += b;
        let _ = name;
    }

    // Shutdown.
    e.set_var("A32NX_OVHD_APU_MASTER_SW_PB_IS_ON", 0.0);
    for bat in ["BAT_1", "BAT_2", "BAT_ESS", "BAT_APU"] {
        e.set_var(&format!("A32NX_OVHD_ELEC_{bat}_PB_IS_AUTO"), 0.0);
    }
    phase_ticks!(e, dt, tick_budget, global_tick, out, scaled(20, spec.expedited), global_index, writer, remaining_budget, prev_fuel);
    {
        let b = snapshot_and_check(&mut e, &mut out, global_index, global_tick, dt, writer, remaining_budget, &mut prev_fuel, true);
        out.bytes_written += b;
    }

    out.kind_desc = format!("procedure expedited={} oat={:.1} alt={:.0} mid_failure={:?} ticks={}", spec.expedited, spec.oat_c, spec.alt_ft, spec.mid_failure, out.ticks_completed);
    out.ticks_planned = out.ticks_completed;
    out
}

// =====================================================================
// Random case runner: samples every axis in the brief from a seeded Rng.
// =====================================================================

#[derive(Clone, Copy)]
enum TimelineEvent {
    PullBreaker,
    ResetBreaker,
    SetFailureMagnitude,
    ClearFailure,
    SetControl,
    SetOat,
    SetIcing,
}

fn run_random(
    global_index: usize,
    struct_len: usize,
    cfg: &Config,
    failure_ids: &[u64],
    breaker_ids: &[&'static str],
    controls: &[fbw_a380_emulator::controls::Control],
    writer: &mut impl std::io::Write,
    remaining_budget: &mut i64,
) -> CaseOutcome {
    let seed = derive_seed(cfg.master_seed, global_index as u64);
    let mut rng = Rng::new(seed);
    let mut out = CaseOutcome { index: global_index, seed, ..Default::default() };

    // --- starting preset ---
    let preset_roll = rng.next_f64();
    let (mut e, preset_name) = if preset_roll < 0.15 {
        (presets::cold_and_dark(), "cold_and_dark")
    } else if preset_roll < 0.30 {
        (presets::ground_power(), "ground_power")
    } else if preset_roll < 0.50 {
        (presets::powered(), "powered")
    } else if preset_roll < 0.80 {
        (presets::engines_running(), "engines_running")
    } else {
        let alt = rng.range_f64(1_000.0, 41_000.0);
        let mach = rng.range_f64(0.3, 0.89);
        (presets::cruise(alt, mach), "cruise")
    };
    check_preset_reached(&mut e, &mut out, preset_name);

    // --- environment ---
    let oat = rng.range_f64(-65.0, 50.0);
    let alt = rng.range_f64(-1_000.0, 41_000.0);
    let wind = (rng.range_f64(-40.0, 40.0), rng.range_f64(-40.0, 40.0), rng.range_f64(-40.0, 40.0));
    let icing = if rng.bool_p(0.3) { rng.range_f64(0.0, 1.0) } else { 0.0 };
    let day_night = if rng.bool_p(0.5) { "day" } else { "night" };
    e.set_oat_c(oat);
    e.set_pressure_altitude_ft(alt);
    e.set_wind_ms(wind.0, wind.1, wind.2);
    e.set_structural_icing_fraction(icing);

    // --- weights (payload stations; tank fuel is fuel.rs's own default load,
    // TOTAL WEIGHT set alongside) ---
    let mut payload_sum = 0.0;
    for station in 1..=18u32 {
        let lb = rng.range_f64(0.0, 60_000.0);
        e.set_payload_station_lb(station, lb);
        payload_sum += lb;
    }
    let fuel_guess = rng.range_f64(20_000.0, 600_000.0);
    let structure_lb = 620_000.0;
    e.set_total_weight_lb(structure_lb + payload_sum + fuel_guess);

    // --- initial failures: a random subset, continuous magnitude,
    // many multi-failure combinations ---
    let n_failures = { let r = rng.next_f64(); (r * r * 8.0) as usize }; // biased toward fewer
    let mut chosen: HashSet<u64> = HashSet::new();
    for _ in 0..n_failures {
        if failure_ids.is_empty() {
            break;
        }
        let id = failure_ids[rng.range_usize(0, failure_ids.len())];
        chosen.insert(id);
    }
    for &id in &chosen {
        e.set_failure_magnitude(id, rng.range_f64(0.0, 1.0));
    }

    // --- initial breaker pulls ---
    let n_breakers = { let r = rng.next_f64(); (r * r * 8.0) as usize };
    let mut pulled: HashSet<&str> = HashSet::new();
    for _ in 0..n_breakers {
        if breaker_ids.is_empty() {
            break;
        }
        let id = breaker_ids[rng.range_usize(0, breaker_ids.len())];
        pulled.insert(id);
        e.pull_breaker(id);
    }

    // --- initial cockpit control changes ---
    let n_controls = { let r = rng.next_f64(); (r * r * 6.0) as usize };
    for _ in 0..n_controls {
        if controls.is_empty() {
            break;
        }
        let c = &controls[rng.range_usize(0, controls.len())];
        e.set_dataref(&c.dataref, if rng.bool_p(0.5) { 1.0 } else { 0.0 });
    }

    // --- duration / dt ---
    let dt = *[0.02, 0.05, 0.1].get(rng.range_usize(0, 3)).unwrap();
    let ticks = rng.range_usize(50, cfg.max_ticks.max(51));
    out.ticks_planned = ticks;

    // --- scripted timeline: random events at random ticks ---
    let n_events = rng.range_usize(0, 8);
    let mut timeline: Vec<(usize, TimelineEvent)> = Vec::with_capacity(n_events);
    for _ in 0..n_events {
        let t = rng.range_usize(0, ticks);
        let kind = match rng.range_usize(0, 7) {
            0 => TimelineEvent::PullBreaker,
            1 => TimelineEvent::ResetBreaker,
            2 => TimelineEvent::SetFailureMagnitude,
            3 => TimelineEvent::ClearFailure,
            4 => TimelineEvent::SetControl,
            5 => TimelineEvent::SetOat,
            _ => TimelineEvent::SetIcing,
        };
        timeline.push((t, kind));
    }
    timeline.sort_by_key(|(t, _)| *t);

    out.kind_desc = format!(
        "random preset={preset_name} oat={oat:.1} alt={alt:.0} icing={icing:.2} day_night={day_night} n_fail={} n_brk={} n_ctrl={n_controls} dt={dt} ticks={ticks} n_events={n_events}",
        chosen.len(),
        pulled.len()
    );

    // Header line: full case parameters, always written regardless of
    // budget (small, and needed to reproduce the case from its seed).
    {
        let mut hdr = String::new();
        let _ = write!(
            hdr,
            "{{\"type\":\"case_header\",\"case\":{global_index},\"seed\":{seed},\"kind\":\"random\",\"preset\":\"{preset_name}\",\"oat_c\":{oat:.2},\"alt_ft\":{alt:.1},\"wind\":[{:.1},{:.1},{:.1}],\"icing\":{icing:.3},\"day_night\":\"{day_night}\",\"payload_lb\":{payload_sum:.1},\"fuel_guess_lb\":{fuel_guess:.1},\"failures\":[{}],\"breakers_pulled\":[{}],\"dt\":{dt},\"ticks_planned\":{ticks},\"n_events\":{n_events}}}\n",
            wind.0, wind.1, wind.2,
            chosen.iter().map(|id| id.to_string()).collect::<Vec<_>>().join(","),
            pulled.iter().map(|id| format!("\"{id}\"")).collect::<Vec<_>>().join(",")
        );
        let bytes = hdr.as_bytes().len() as u64;
        let _ = writer.write_all(hdr.as_bytes());
        out.bytes_written += bytes;
        *remaining_budget -= bytes as i64;
    }

    let mut ev_idx = 0usize;
    let mut prev_fuel = None;
    for tick in 0..ticks {
        while ev_idx < timeline.len() && timeline[ev_idx].0 == tick {
            match timeline[ev_idx].1 {
                TimelineEvent::PullBreaker if !breaker_ids.is_empty() => e.pull_breaker(breaker_ids[rng.range_usize(0, breaker_ids.len())]),
                TimelineEvent::ResetBreaker if !breaker_ids.is_empty() => e.reset_breaker(breaker_ids[rng.range_usize(0, breaker_ids.len())]),
                TimelineEvent::SetFailureMagnitude if !failure_ids.is_empty() => {
                    e.set_failure_magnitude(failure_ids[rng.range_usize(0, failure_ids.len())], rng.range_f64(0.0, 1.0))
                }
                TimelineEvent::ClearFailure if !failure_ids.is_empty() => {
                    e.set_failure_magnitude(failure_ids[rng.range_usize(0, failure_ids.len())], 0.0)
                }
                TimelineEvent::SetControl if !controls.is_empty() => {
                    let c = &controls[rng.range_usize(0, controls.len())];
                    e.set_dataref(&c.dataref, if rng.bool_p(0.5) { 1.0 } else { 0.0 });
                }
                TimelineEvent::SetOat => e.set_oat_c(rng.range_f64(-65.0, 50.0)),
                TimelineEvent::SetIcing => e.set_structural_icing_fraction(rng.range_f64(0.0, 1.0)),
                _ => {}
            }
            ev_idx += 1;
        }
        e.tick(dt);
        out.ticks_completed = tick + 1;
        let write = tick % cfg.snapshot_every == 0 || tick + 1 == ticks;
        let b = snapshot_and_check(&mut e, &mut out, global_index, tick, dt, writer, remaining_budget, &mut prev_fuel, write);
        out.bytes_written += b;
    }
    let _ = struct_len;
    out
}

// =====================================================================
// Worker: runs a contiguous [start, end) slice of the global case-index
// space, one case at a time, each on a helper thread with a wall-clock
// timeout (hang protection) and a caught panic (crash protection).
// =====================================================================

fn worker_main(cfg: Config) {
    std::fs::create_dir_all(&cfg.dump_dir).expect("create dump dir");
    // Tiered mode: the cases come from the tier's plan file instead.
    let plan: Option<Vec<ComboSpec>> = cfg.plan_file.as_ref().map(|path| {
        std::fs::read_to_string(path).expect("read plan file").lines().filter_map(ComboSpec::from_line).collect()
    });
    let structured = if plan.is_some() { Vec::new() } else { build_structured_plan() };
    let mut reach_writer = plan.as_ref().map(|_| {
        let path = cfg.dump_dir.join(format!("reach_{:03}.txt", cfg.shard_id));
        std::io::BufWriter::new(std::fs::File::create(path).expect("create reach file"))
    });
    let e = Emulator::new(StartState::Apron);
    let failure_ids: Vec<u64> = e.list_failures();
    let breaker_ids: Vec<&'static str> = e.list_breaker_ids();
    let controls = e.list_controls();
    drop(e);

    let log_path = cfg.dump_dir.join(format!("worker_{:03}.jsonl", cfg.shard_id));
    let summary_path = cfg.dump_dir.join(format!("worker_{:03}_summary.txt", cfg.shard_id));
    let file = std::fs::File::create(&log_path).expect("create worker log");
    let mut writer = std::io::BufWriter::with_capacity(1 << 20, file);

    let per_worker_budget_bytes = ((cfg.max_gb * 1024.0 * 1024.0 * 1024.0) / cfg.workers as f64) as i64;
    let mut remaining_budget = per_worker_budget_bytes;

    let mut cases_run = 0u64;
    let mut passes = 0u64;
    let mut fails = 0u64;
    let mut panics = 0u64;
    let mut hangs = 0u64;
    let mut nans = 0u64;
    let mut cap_hit = false;
    let mut invariant_totals: BTreeMap<String, u64> = BTreeMap::new();
    let mut sanity_totals: BTreeMap<String, u64> = BTreeMap::new();
    let mut sample_failures: Vec<String> = Vec::new();

    // The cases: the todo list or the shard's range, claimed through the
    // shared queue when there is one.
    let items: Vec<usize> = match &cfg.todo_file {
        Some(path) => std::fs::read_to_string(path).expect("read todo file").lines().filter_map(|l| l.trim().parse().ok()).collect(),
        None => (cfg.shard_start..cfg.shard_end).collect(),
    };
    let last_item = items.last().copied();
    let cases: Box<dyn Iterator<Item = usize>> = match &cfg.claim_dir {
        Some(dir) => Box::new(fbw_a380_emulator::work::Claims::new(dir, items, cfg.chunk)),
        None => Box::new(items.into_iter()),
    };
    for index in cases {
        if remaining_budget <= 0 {
            cap_hit = true;
            break;
        }
        let kind = match &plan {
            Some(p) => CaseKind::Combo(p[index].clone()),
            None => resolve_case(index, &structured),
        };
        let kind_for_log = kind.clone();
        let seed_preview = derive_seed(cfg.master_seed, index as u64);

        let (tx, rx) = mpsc::channel::<CaseOutcome>();
        let cfg2 = cfg.clone();
        let structured2 = structured.clone();
        let failure_ids2 = failure_ids.clone();
        let breaker_ids2 = breaker_ids.clone();
        let controls2 = controls.clone();
        let log_path2 = log_path.clone();
        let struct_len = structured.len();

        // Flush the shared writer before handing bytes-budget bookkeeping
        // to a fresh append handle on the thread (keeps the worker's main
        // writer as the single source of truth for "did we finish", while
        // the case thread appends its own lines independently so a hung
        // thread never blocks the main writer).
        let _ = writer.flush();
        let budget_for_case = remaining_budget;

        let handle = std::thread::spawn(move || {
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                let file = std::fs::OpenOptions::new().append(true).open(&log_path2).expect("reopen worker log for append");
                let mut w = std::io::BufWriter::with_capacity(1 << 16, file);
                let mut budget = budget_for_case;
                let out = match &kind {
                    CaseKind::Random { index: ri } => {
                        let global = struct_len + ri;
                        let seed = derive_seed(cfg2.master_seed, global as u64);
                        // ~40% of the continuous random space is normal-
                        // operations procedures (this coin flip, seeded
                        // off the case's own seed so it's reproducible),
                        // the rest is free-form field sampling.
                        let mut probe = Rng::new(seed ^ 0xABCD);
                        if probe.bool_p(0.4) {
                            let spec = ProcedureSpec::random(&mut probe, &failure_ids2);
                            run_procedure(global, seed, &cfg2, &failure_ids2, &breaker_ids2, &controls2, &mut w, &mut budget, spec)
                        } else {
                            run_random(global, struct_len, &cfg2, &failure_ids2, &breaker_ids2, &controls2, &mut w, &mut budget)
                        }
                    }
                    CaseKind::Procedure { combo } => {
                        let seed = derive_seed(cfg2.master_seed, index as u64);
                        let mut rng0 = Rng::new(seed ^ 0x2468);
                        let spec = ProcedureSpec::structured(*combo, &failure_ids2, &mut rng0);
                        run_procedure(index, seed, &cfg2, &failure_ids2, &breaker_ids2, &controls2, &mut w, &mut budget, spec)
                    }
                    CaseKind::Combo(spec) => run_combo(spec, index, &cfg2, &mut w, &mut budget),
                    other => run_structured(other, index, &cfg2, &mut w, &mut budget),
                };
                let _ = w.flush();
                out
            }));
            let outcome = match result {
                Ok(out) => out,
                Err(payload) => {
                    let msg = if let Some(s) = payload.downcast_ref::<&str>() {
                        s.to_string()
                    } else if let Some(s) = payload.downcast_ref::<String>() {
                        s.clone()
                    } else {
                        "panic (non-string payload)".to_owned()
                    };
                    CaseOutcome { index, seed: seed_preview, kind_desc: format!("{kind:?}"), panicked: true, panic_msg: Some(msg), ..Default::default() }
                }
            };
            let _ = tx.send(outcome);
        });

        match rx.recv_timeout(Duration::from_secs(cfg.timeout_secs)) {
            Ok(outcome) => {
                let _ = handle.join();
                cases_run += 1;
                if let Some(rw) = reach_writer.as_mut() {
                    let _ = writeln!(rw, "{index}\t{}\t{}", outcome.passed() && !outcome.panicked, outcome.reach.join(","));
                }
                remaining_budget -= outcome.bytes_written as i64;
                if outcome.panicked {
                    panics += 1;
                    fails += 1;
                    if sample_failures.len() < 50 {
                        sample_failures.push(format!("case {index} seed {} PANIC: {}", outcome.seed, outcome.panic_msg.as_deref().unwrap_or("?")));
                    }
                } else if outcome.passed() {
                    passes += 1;
                } else {
                    fails += 1;
                    if outcome.nan_detected {
                        nans += 1;
                    }
                    if sample_failures.len() < 50 {
                        sample_failures.push(format!(
                            "case {index} seed {} FAIL: nan={} first_nan={:?}@{:?} invariants={} sanity={}",
                            outcome.seed,
                            outcome.nan_detected,
                            outcome.first_nan_var,
                            outcome.first_nan_tick,
                            outcome.invariant_counts.len(),
                            outcome.sanity_issues.len()
                        ));
                    }
                }
                for (name, count) in &outcome.invariant_counts {
                    *invariant_totals.entry(name.clone()).or_insert(0) += count;
                }
                for issue in &outcome.sanity_issues {
                    // Check name plus its first detail (e.g. the phase), not the value.
                    let key = issue.split('=').next().unwrap_or(issue).splitn(3, ':').take(2).collect::<Vec<_>>().join(":");
                    let key = key.split(" (").next().unwrap_or(&key).to_owned();
                    *sanity_totals.entry(key).or_insert(0) += 1;
                }
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {
                // Deliberately abandoned -- see the module doc comment.
                if let Some(rw) = reach_writer.as_mut() {
                    let _ = writeln!(rw, "{index}\tfalse\t");
                }
                hangs += 1;
                cases_run += 1;
                fails += 1;
                if sample_failures.len() < 50 {
                    sample_failures.push(format!("case {index} seed {seed_preview} HUNG (> {}s): {kind_for_log:?}", cfg.timeout_secs));
                }
            }
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                fails += 1;
                cases_run += 1;
            }
        }

        // Incremental summary so the run can be inspected mid-flight.
        if cases_run % 10 == 0 || Some(index) == last_item {
            write_summary(
                &summary_path,
                cfg.shard_id,
                cfg.shard_start,
                cfg.shard_end,
                cases_run,
                passes,
                fails,
                panics,
                hangs,
                nans,
                cap_hit,
                &invariant_totals,
                &sanity_totals,
                &sample_failures,
            );
        }
    }
    let _ = writer.flush();
    if let Some(rw) = reach_writer.as_mut() {
        let _ = rw.flush();
    }
    write_summary(
        &summary_path,
        cfg.shard_id,
        cfg.shard_start,
        cfg.shard_end,
        cases_run,
        passes,
        fails,
        panics,
        hangs,
        nans,
        cap_hit,
        &invariant_totals,
        &sanity_totals,
        &sample_failures,
    );
}

#[allow(clippy::too_many_arguments)]
fn write_summary(
    path: &PathBuf,
    shard_id: usize,
    start: usize,
    end: usize,
    cases_run: u64,
    passes: u64,
    fails: u64,
    panics: u64,
    hangs: u64,
    nans: u64,
    cap_hit: bool,
    invariant_totals: &BTreeMap<String, u64>,
    sanity_totals: &BTreeMap<String, u64>,
    sample_failures: &[String],
) {
    let mut s = String::new();
    let _ = writeln!(s, "shard {shard_id} range [{start},{end}) cases_run={cases_run} pass={passes} fail={fails} panics={panics} hangs={hangs} nans={nans} cap_hit={cap_hit}");
    let mut ranked: Vec<(&String, &u64)> = invariant_totals.iter().collect();
    ranked.sort_by(|a, b| b.1.cmp(a.1));
    let _ = writeln!(s, "-- invariant violations, ranked by count --");
    for (name, count) in ranked.iter().take(50) {
        let _ = writeln!(s, "  {count:>10}  {name}");
    }
    let mut sranked: Vec<(&String, &u64)> = sanity_totals.iter().collect();
    sranked.sort_by(|a, b| b.1.cmp(a.1));
    let _ = writeln!(s, "-- sanity-check anomalies, ranked by count --");
    for (name, count) in sranked.iter().take(50) {
        let _ = writeln!(s, "  {count:>10}  {name}");
    }
    let _ = writeln!(s, "-- sample failures (first {}) --", sample_failures.len());
    for line in sample_failures {
        let _ = writeln!(s, "  {line}");
    }
    let _ = std::fs::write(path, s);
}

// =====================================================================
// Orchestrator: sizes the case space, spawns `--workers` worker
// processes each covering a contiguous slice, waits for all of them, and
// writes a merged top-level SUMMARY.txt.
// =====================================================================

fn orchestrator_main(cfg: Config) {
    std::fs::create_dir_all(&cfg.dump_dir).expect("create dump dir");
    let structured = build_structured_plan();
    let total = structured.len() + cfg.cases;
    println!("battery: master-seed={} structured_cases={} random_cases={} total_cases={}", cfg.master_seed, structured.len(), cfg.cases, total);
    println!("battery: workers={} max_gb={} dump_dir={}", cfg.workers, cfg.max_gb, cfg.dump_dir.display());

    let exe = std::env::current_exe().expect("current_exe");
    let shard_len = (total + cfg.workers - 1) / cfg.workers;
    let mut children = Vec::new();
    for w in 0..cfg.workers {
        let start = w * shard_len;
        if start >= total {
            break;
        }
        let end = (start + shard_len).min(total);
        let child = std::process::Command::new(&exe)
            .arg("--worker")
            .arg("--master-seed").arg(cfg.master_seed.to_string())
            .arg("--cases").arg(cfg.cases.to_string())
            .arg("--workers").arg(cfg.workers.to_string())
            .arg("--dump-dir").arg(&cfg.dump_dir)
            .arg("--max-gb").arg(cfg.max_gb.to_string())
            .arg("--max-ticks").arg(cfg.max_ticks.to_string())
            .arg("--snapshot-every").arg(cfg.snapshot_every.to_string())
            .arg("--timeout-secs").arg(cfg.timeout_secs.to_string())
            .arg("--shard-id").arg(w.to_string())
            .arg("--shard-start").arg(start.to_string())
            .arg("--shard-end").arg(end.to_string())
            .spawn()
            .expect("spawn worker process");
        children.push(child);
    }
    println!("battery: spawned {} worker processes", children.len());

    for mut c in children {
        let _ = c.wait();
    }

    // Merge shard summaries into one top-level report.
    let mut merged = String::new();
    let mut total_run = 0u64;
    let mut total_pass = 0u64;
    let mut total_fail = 0u64;
    let mut total_panics = 0u64;
    let mut total_hangs = 0u64;
    let mut total_nans = 0u64;
    for entry in std::fs::read_dir(&cfg.dump_dir).into_iter().flatten().flatten() {
        let path = entry.path();
        if path.file_name().and_then(|n| n.to_str()).is_some_and(|n| n.ends_with("_summary.txt")) {
            if let Ok(text) = std::fs::read_to_string(&path) {
                if let Some(first) = text.lines().next() {
                    for tok in first.split_whitespace() {
                        if let Some(v) = tok.strip_prefix("cases_run=") {
                            total_run += v.parse().unwrap_or(0);
                        } else if let Some(v) = tok.strip_prefix("pass=") {
                            total_pass += v.parse().unwrap_or(0);
                        } else if let Some(v) = tok.strip_prefix("fail=") {
                            total_fail += v.parse().unwrap_or(0);
                        } else if let Some(v) = tok.strip_prefix("panics=") {
                            total_panics += v.parse().unwrap_or(0);
                        } else if let Some(v) = tok.strip_prefix("hangs=") {
                            total_hangs += v.parse().unwrap_or(0);
                        } else if let Some(v) = tok.strip_prefix("nans=") {
                            total_nans += v.parse().unwrap_or(0);
                        }
                    }
                }
                merged.push_str(&text);
                merged.push('\n');
            }
        }
    }
    let header = format!(
        "BATTERY SUMMARY\nmaster_seed={}\nstructured_cases={}\nrandom_cases={}\ntotal_cases={}\ncases_run={}\npass={}\nfail={}\npanics={}\nhangs={}\nnans={}\n\n",
        cfg.master_seed, structured.len(), cfg.cases, total, total_run, total_pass, total_fail, total_panics, total_hangs, total_nans
    );
    let _ = std::fs::write(cfg.dump_dir.join("SUMMARY.txt"), format!("{header}{merged}"));
    println!("battery: done. cases_run={total_run} pass={total_pass} fail={total_fail} panics={total_panics} hangs={total_hangs} nans={total_nans}");
    println!("battery: summary at {}", cfg.dump_dir.join("SUMMARY.txt").display());
}

// =====================================================================
// Tiered mode (orchestrator side).
//
// Tier 1: every failure (at full magnitude) and every breaker alone, on each
// of the four start states, recording which variables each moved from the
// fault-free baseline ("reach"). Tier 2: pairs whose reaches overlap, ranked
// by how specific the shared variables are (a variable most singles move
// says little; one only two move says a lot), capped at `--tier-max`; plus
// a sample of non-overlapping pairs, whose outcome is predicted from their
// singles and checked. Tier 3: each pair that interacted, plus one more
// element overlapping it. An interaction is a combination that fails where
// its parts each passed, or that moves variables none of its parts moved.
// =====================================================================

struct TierResult {
    passed: bool,
    reach: std::collections::BTreeSet<String>,
}

/// The reach files' finished cases: (index, passed, reach). A file cut off
/// mid-line (a killed worker) loses its unfinished last line.
fn read_reach(dir: &std::path::Path) -> Vec<(usize, bool, Vec<String>)> {
    let mut out = Vec::new();
    for entry in std::fs::read_dir(dir).into_iter().flatten().flatten() {
        let name = entry.file_name().to_string_lossy().into_owned();
        if !(name.starts_with("reach_") && name.ends_with(".txt")) {
            continue;
        }
        let text = std::fs::read_to_string(entry.path()).unwrap_or_default();
        let complete = match text.rfind('\n') {
            Some(end) => &text[..end],
            None => "",
        };
        for line in complete.lines() {
            let mut f = line.splitn(3, '\t');
            let (Some(i), Some(ok)) = (f.next().and_then(|x| x.parse::<usize>().ok()), f.next()) else { continue };
            let reach = f.next().unwrap_or("").split(',').filter(|x| !x.is_empty()).map(str::to_owned).collect();
            out.push((i, ok == "true", reach));
        }
    }
    out
}

fn run_tier(cfg: &Config, tier: usize, specs: &mut Vec<ComboSpec>) -> Vec<Option<TierResult>> {
    run_tier_named(cfg, &format!("tier_{tier}"), specs)
}

fn run_tier_named(cfg: &Config, name: &str, specs: &mut Vec<ComboSpec>) -> Vec<Option<TierResult>> {
    let dir = cfg.dump_dir.join(name);
    std::fs::create_dir_all(&dir).expect("create tier dir");
    let plan = dir.join("plan.txt");
    // Resuming: the tier's own plan, exactly as it was drawn.
    let reused = cfg.resume && plan.exists();
    if reused {
        *specs = std::fs::read_to_string(&plan).expect("read plan").lines().filter_map(ComboSpec::from_line).collect();
    } else {
        let text: String = specs.iter().map(|s| s.to_line() + "\n").collect();
        std::fs::write(&plan, text).expect("write plan");
    }
    let exe = std::env::current_exe().expect("current_exe");
    let total = specs.len();
    let done: std::collections::BTreeSet<usize> = if reused { read_reach(&dir).into_iter().map(|(i, ..)| i).collect() } else { Default::default() };
    let todo: Vec<usize> = (0..total).filter(|i| !done.contains(i)).collect();
    if reused {
        println!("{name}: resuming, {} of {total} done, {} to run", done.len(), todo.len());
    }
    // New workers never overwrite an earlier run's files.
    let first_id = std::fs::read_dir(&dir)
        .into_iter()
        .flatten()
        .flatten()
        .filter(|e| e.file_name().to_string_lossy().starts_with("reach_"))
        .count();
    let todo_file = dir.join(format!("todo_{first_id}.txt"));
    std::fs::write(&todo_file, todo.iter().map(|i| format!("{i}\n")).collect::<String>()).expect("write todo");
    let claim_dir = dir.join(format!("claims_{first_id}"));
    let chunk = fbw_a380_emulator::work::Claims::chunk_for(todo.len(), cfg.workers);
    progress_run(&dir, total, done.len(), first_id);
    let mut children = Vec::new();
    if !todo.is_empty() {
        for w in 0..cfg.workers.min(todo.len()) {
            children.push(
                std::process::Command::new(&exe)
                    .arg("--worker")
                    .arg("--master-seed").arg(cfg.master_seed.to_string())
                    .arg("--workers").arg(cfg.workers.to_string())
                    .arg("--dump-dir").arg(&dir)
                    .arg("--max-gb").arg(cfg.max_gb.to_string())
                    .arg("--snapshot-every").arg(cfg.snapshot_every.to_string())
                    .arg("--timeout-secs").arg(cfg.timeout_secs.to_string())
                    .arg("--tier-ticks").arg(cfg.tier_ticks.to_string())
                    .arg("--plan-file").arg(&plan)
                    .arg("--todo-file").arg(&todo_file)
                    .arg("--claim-dir").arg(&claim_dir)
                    .arg("--chunk").arg(chunk.to_string())
                    .arg("--shard-id").arg((first_id + w).to_string())
                    .spawn()
                    .expect("spawn worker"),
            );
        }
    }
    for mut c in children {
        let _ = c.wait();
    }
    let _ = std::fs::remove_dir_all(&claim_dir);
    progress_run(&dir, total, total, usize::MAX);
    let mut results: Vec<Option<TierResult>> = (0..total).map(|_| None).collect();
    for (i, passed, reach) in read_reach(&dir) {
        if i < total {
            results[i] = Some(TierResult { passed, reach: reach.into_iter().collect() });
        }
    }
    results
}

/// Predicts a combination's outcome from what the tiers learned: it fails
/// if any of its elements failed alone on that start state, or if it
/// contains a combination the tiers found fails only together; otherwise it
/// is predicted to pass. Works for a combination of any size.
struct Predictor {
    fails_alone: std::collections::BTreeSet<(usize, String)>,
    fails_together: Vec<(usize, std::collections::BTreeSet<String>)>,
}

impl Predictor {
    fn from_tiers(single: &BTreeMap<(usize, String), &TierResult>, graph: &[(usize, ComboSpec, bool, bool)]) -> Self {
        Self {
            fails_alone: single.iter().filter(|(_, r)| !r.passed).map(|(k, _)| k.clone()).collect(),
            fails_together: graph
                .iter()
                .filter(|(tier, _, ef, _)| *ef && *tier < 5)
                .map(|(_, spec, _, _)| (spec.preset, spec.elements().into_iter().collect()))
                .collect(),
        }
    }

    /// From a finished run's `interactions.json` (the `--predict` mode).
    fn from_json(text: &str) -> Self {
        let mut fails_alone = std::collections::BTreeSet::new();
        let mut fails_together = Vec::new();
        // nodes: {"id":"f:72012","passedAlone":[true,false,null,true],...}
        for chunk in text.split("{\"id\":\"").skip(1) {
            let id = chunk.split('"').next().unwrap_or("").to_owned();
            if let Some(list) = chunk.split("\"passedAlone\":[").nth(1).and_then(|r| r.split(']').next()) {
                for (preset, v) in list.split(',').enumerate() {
                    if v.trim() == "false" {
                        fails_alone.insert((preset, id.clone()));
                    }
                }
            }
        }
        // combos: {"tier":2,"preset":1,"elements":["f:1","b:X"],"failsTogether":true,...}
        for chunk in text.split("{\"tier\":").skip(1) {
            let tier: usize = chunk.split(',').next().and_then(|t| t.trim().parse().ok()).unwrap_or(0);
            let preset: usize = chunk.split("\"preset\":").nth(1).and_then(|r| r.split(',').next()).and_then(|t| t.trim().parse().ok()).unwrap_or(0);
            let together = chunk.contains("\"failsTogether\":true");
            let elements: std::collections::BTreeSet<String> = chunk
                .split("\"elements\":[")
                .nth(1)
                .and_then(|r| r.split(']').next())
                .map(|l| l.split(',').map(|e| e.trim().trim_matches('"').to_owned()).filter(|e| !e.is_empty()).collect())
                .unwrap_or_default();
            if together && tier < 5 {
                fails_together.push((preset, elements));
            }
        }
        Self { fails_alone, fails_together }
    }

    fn predict_fail(&self, spec: &ComboSpec) -> bool {
        let els: std::collections::BTreeSet<String> = spec.elements().into_iter().collect();
        els.iter().any(|e| self.fails_alone.contains(&(spec.preset, e.clone())))
            || self.fails_together.iter().any(|(p, set)| *p == spec.preset && set.is_subset(&els))
    }

    /// Why: the elements that fail alone and the known combinations inside it.
    fn explain(&self, spec: &ComboSpec) -> Vec<String> {
        let els: std::collections::BTreeSet<String> = spec.elements().into_iter().collect();
        let mut why: Vec<String> = els
            .iter()
            .filter(|e| self.fails_alone.contains(&(spec.preset, (*e).clone())))
            .map(|e| format!("{e} fails alone"))
            .collect();
        for (p, set) in &self.fails_together {
            if *p == spec.preset && set.is_subset(&els) {
                why.push(format!("{} fail together", set.iter().cloned().collect::<Vec<_>>().join(" + ")));
            }
        }
        why
    }
}

/// `--predict <dump dir> "preset|f1,f2,...|b1,..."`: the predicted outcome
/// of any combination, from a finished tiered run, without running it.
fn predict_main(args: &[String]) {
    let dir = args.get(2).map(PathBuf::from).expect("usage: battery --predict <dump dir> \"preset|failures|breakers\"");
    let spec = args.get(3).and_then(|l| ComboSpec::from_line(l)).expect("combination as preset|f1,f2|b1,b2");
    let text = std::fs::read_to_string(dir.join("interactions.json")).expect("read interactions.json");
    let p = Predictor::from_json(&text);
    let why = p.explain(&spec);
    if p.predict_fail(&spec) {
        println!("predicted: FAILS");
        for w in why {
            println!("  because {w}");
        }
    } else {
        println!("predicted: passes (no element fails alone and no known failing combination is inside it)");
    }
}

fn tiered_main(cfg: Config) {
    std::fs::create_dir_all(&cfg.dump_dir).expect("create dump dir");
    start_progress_server(cfg.workers);
    let e = Emulator::new(StartState::Apron);
    let failure_ids = e.list_failures();
    let breaker_ids: Vec<String> = e.list_breaker_ids().into_iter().map(str::to_owned).collect();
    drop(e);
    let mut report = String::new();
    let started = std::time::Instant::now();

    // ---- tier 1: singles on every start state ----
    let mut singles = Vec::new();
    for preset in 0..4 {
        for &f in &failure_ids {
            singles.push(ComboSpec { preset, failures: vec![f], breakers: vec![] });
        }
        for b in &breaker_ids {
            singles.push(ComboSpec { preset, failures: vec![], breakers: vec![b.clone()] });
        }
    }
    // Empty cases: no fault at all. What they move against their worker's
    // baseline run is the start state's own run-to-run noise (the models are
    // not bit-reproducible in some 20-70 variables).
    for preset in 0..4 {
        for _ in 0..NOISE_CASES {
            singles.push(ComboSpec { preset, failures: vec![], breakers: vec![] });
        }
    }
    println!("tier 1: {} singles (with {} empty noise cases)", singles.len(), 4 * NOISE_CASES);
    progress_tier("tier 1: singles");
    let r1 = run_tier(&cfg, 1, &mut singles);
    // Per start state: element -> its single's result.
    let mut single: BTreeMap<(usize, String), &TierResult> = BTreeMap::new();
    let mut noise: Vec<std::collections::BTreeSet<String>> = vec![Default::default(); 4];
    for (spec, r) in singles.iter().zip(&r1) {
        let Some(r) = r else { continue };
        match spec.elements().first() {
            Some(e) => {
                single.insert((spec.preset, e.clone()), r);
            }
            None => noise[spec.preset].extend(r.reach.iter().cloned()),
        }
    }
    // A failure group's shared bookkeeping flag is never a physical effect.
    let is_hook = |v: &str| v.starts_with("A32NX_FAIL_") && v.ends_with("_HOOK");
    let noise_any: std::collections::BTreeSet<String> = noise.iter().flatten().cloned().collect();
    // What an element really does on its start state: its reach without
    // the noise and the hooks.
    let effect: BTreeMap<(usize, String), std::collections::BTreeSet<String>> = single
        .iter()
        .map(|((p, k), r)| ((*p, k.clone()), r.reach.iter().filter(|v| !noise[*p].contains(*v) && !is_hook(v)).cloned().collect()))
        .collect();
    let _ = writeln!(
        report,
        "noise: {} variables move with no fault at all ({} cold, {} ground power, {} APU, {} engines)",
        noise_any.len(),
        noise[0].len(),
        noise[1].len(),
        noise[2].len(),
        noise[3].len()
    );
    let t1_fail = r1.iter().filter(|r| r.as_ref().is_some_and(|r| !r.passed)).count();
    let _ = writeln!(report, "tier 1: {} singles, {} failed, {} missing, {:.0} s", singles.len(), t1_fail, r1.iter().filter(|r| r.is_none()).count(), started.elapsed().as_secs_f64());

    // How specific each variable is: ln(singles / singles that move it).
    let mut df: BTreeMap<&str, usize> = BTreeMap::new();
    for r in r1.iter().flatten() {
        for v in &r.reach {
            *df.entry(v.as_str()).or_insert(0) += 1;
        }
    }
    let n1 = r1.iter().flatten().count().max(1) as f64;
    let idf = |v: &str| (n1 / *df.get(v).unwrap_or(&1) as f64).ln();
    // Never evidence of a shared path: the measured noise and the hooks.
    let hub = |v: &str| noise_any.contains(v) || is_hook(v);
    let overlap_score = |a: &TierResult, b: &std::collections::BTreeSet<String>| -> f64 {
        a.reach.intersection(b).filter(|v| !hub(v)).map(|v| idf(v)).sum()
    };

    // ---- tier 2: every pair, exactly ----
    // Two elements that move a variable in common are run as a pair. Every
    // other pair is covered by group cases: up to GROUP_MAX elements, no two
    // of which share an effect, so the group's result must be exactly its
    // members' effects together. A group that is not (fails where its
    // members pass, moves something none of them moves, or loses one's
    // effect) is split into its pairs and those are run.
    let shares = |preset: usize, a: &str, b: &str| -> bool {
        match (effect.get(&(preset, a.to_owned())), effect.get(&(preset, b.to_owned()))) {
            (Some(x), Some(y)) => !x.is_disjoint(y),
            _ => false,
        }
    };
    let mut t2: Vec<ComboSpec> = Vec::new();
    let mut group_pairs = 0usize;
    for preset in 0..4 {
        let elems: Vec<&String> = effect.keys().filter(|(p, _)| *p == preset).map(|(_, k)| k).collect();
        let n = elems.len();
        let words = n.div_ceil(64);
        let bit = |set: &[u64], j: usize| (set[j / 64] >> (j % 64)) & 1 == 1;
        let mut conflict = vec![vec![0u64; words]; n];
        let mut by_var: std::collections::HashMap<&str, Vec<usize>> = std::collections::HashMap::new();
        for (i, k) in elems.iter().enumerate() {
            for v in &effect[&(preset, (*k).clone())] {
                by_var.entry(v.as_str()).or_default().push(i);
            }
        }
        for list in by_var.values() {
            for &x in list {
                for &y in list {
                    if x != y {
                        conflict[x][y / 64] |= 1 << (y % 64);
                    }
                }
            }
        }
        let mut uncovered = vec![vec![0u64; words]; n];
        for i in 0..n {
            for j in 0..n {
                if i == j {
                    continue;
                }
                if bit(&conflict[i], j) {
                    if i < j {
                        t2.push(ComboSpec { preset, failures: vec![], breakers: vec![] }.with(elems[i]).with(elems[j]));
                    }
                } else {
                    uncovered[i][j / 64] |= 1 << (j % 64);
                }
            }
        }
        let count = |set: &[u64]| set.iter().map(|w| w.count_ones() as usize).sum::<usize>();
        loop {
            let Some(seed) = (0..n).filter(|&i| count(&uncovered[i]) > 0).max_by_key(|&i| count(&uncovered[i])) else { break };
            let mut group = vec![seed];
            let mut compat: Vec<u64> = (0..words).map(|w| !conflict[seed][w]).collect();
            compat[seed / 64] &= !(1 << (seed % 64));
            while group.len() < GROUP_MAX {
                let mut best: Option<(usize, usize, usize)> = None;
                for b in 0..n {
                    if !bit(&compat, b) {
                        continue;
                    }
                    let gain = group.iter().filter(|&&g| bit(&uncovered[g], b)).count();
                    if gain == 0 {
                        continue;
                    }
                    let look: usize = uncovered[b].iter().zip(&compat).map(|(x, y)| (x & y).count_ones() as usize).sum();
                    if best.is_none_or(|(_, g, l)| (gain, look) > (g, l)) {
                        best = Some((b, gain, look));
                    }
                }
                let Some((b, ..)) = best else { break };
                group.push(b);
                for w in 0..words {
                    compat[w] &= !conflict[b][w];
                }
                compat[b / 64] &= !(1 << (b % 64));
            }
            for &x in &group {
                for &y in &group {
                    if x != y && bit(&uncovered[x], y) {
                        uncovered[x][y / 64] &= !(1 << (y % 64));
                        if x < y {
                            group_pairs += 1;
                        }
                    }
                }
            }
            let mut spec = ComboSpec { preset, failures: vec![], breakers: vec![] };
            for &i in &group {
                spec = spec.with(elems[i]);
            }
            t2.push(spec);
        }
    }
    progress_report(&report);
    progress_tier("tier 2: pairs and groups");
    let r2 = run_tier(&cfg, 2, &mut t2);
    // A pair run on its own: two elements that share an effect.
    let is_shared_pair = |spec: &ComboSpec| {
        let e = spec.elements();
        e.len() == 2 && shares(spec.preset, &e[0], &e[1])
    };
    let consistent = |spec: &ComboSpec, r: &TierResult| -> bool {
        let members = spec.elements();
        let all_pass = members.iter().all(|k| single.get(&(spec.preset, k.clone())).is_some_and(|p| p.passed));
        if all_pass && !r.passed {
            return false;
        }
        let got: std::collections::BTreeSet<&String> = r.reach.iter().filter(|v| !noise[spec.preset].contains(*v) && !is_hook(v)).collect();
        let union: std::collections::BTreeSet<&String> = members.iter().filter_map(|k| effect.get(&(spec.preset, k.clone()))).flatten().collect();
        got == union
    };
    let (mut n_shared, mut n_groups, mut n_flagged) = (0usize, 0usize, 0usize);
    let mut pairs: Vec<ComboSpec> = Vec::new();
    let mut r2p: Vec<Option<TierResult>> = Vec::new();
    let mut followups: std::collections::BTreeSet<ComboSpec> = std::collections::BTreeSet::new();
    for (spec, r) in t2.iter().zip(r2) {
        if is_shared_pair(spec) {
            n_shared += 1;
            pairs.push(spec.clone());
            r2p.push(r);
            continue;
        }
        n_groups += 1;
        let Some(r) = r else { continue };
        if consistent(spec, &r) {
            continue;
        }
        n_flagged += 1;
        let e = spec.elements();
        if e.len() == 2 {
            pairs.push(spec.clone());
            r2p.push(Some(r));
            continue;
        }
        for i in 0..e.len() {
            for j in (i + 1)..e.len() {
                followups.insert(ComboSpec { preset: spec.preset, failures: vec![], breakers: vec![] }.with(&e[i]).with(&e[j]));
            }
        }
    }
    let mut followups: Vec<ComboSpec> = followups.into_iter().collect();
    let n_followups = followups.len();
    if !followups.is_empty() {
        progress_tier("tier 2: pairs from flagged groups");
        let rf = run_tier_named(&cfg, "tier_2_split", &mut followups);
        pairs.extend(followups);
        r2p.extend(rf);
    }
    let r2 = r2p;
    let n_overlap_run = pairs.len();
    let _ = writeln!(
        report,
        "tier 2 plan: {n_shared} pairs that share an effect run one by one; {group_pairs} other pairs covered by {n_groups} group cases (up to {GROUP_MAX} each); {n_flagged} groups not exactly their members' effects, split into {n_followups} pairs"
    );
    let mut score_tier = |tier: usize, specs: &[ComboSpec], results: &[Option<TierResult>], graph: &[(usize, ComboSpec, bool, bool)], report: &mut String| {
        // Predicted before running, from what the tiers below had found.
        let predictor = Predictor::from_tiers(&single, &graph.iter().filter(|(t, ..)| *t < tier).cloned().collect::<Vec<_>>());
        let (mut tp, mut tn, mut fp, mut fn_) = (0usize, 0usize, 0usize, 0usize);
        for (spec, r) in specs.iter().zip(results) {
            let Some(r) = r else { continue };
            match (predictor.predict_fail(spec), !r.passed) {
                (true, true) => tp += 1,
                (false, false) => tn += 1,
                (true, false) => fp += 1,
                (false, true) => fn_ += 1,
            }
        }
        let n = (tp + tn + fp + fn_).max(1);
        let _ = writeln!(
            report,
            "tier {tier} predicted before running: {:.2}% right ({tp} failures foreseen, {tn} passes foreseen, {fn_} failures missed, {fp} false alarms)",
            100.0 * (tp + tn) as f64 / n as f64
        );
    };

    let parts_of = |spec: &ComboSpec| -> Vec<&TierResult> {
        spec.elements().iter().filter_map(|k| single.get(&(spec.preset, k.clone())).copied()).collect()
    };
    let classify = |spec: &ComboSpec, r: &TierResult, parts: &[&TierResult]| -> (bool, bool) {
        let all_parts_pass = parts.iter().all(|p| p.passed);
        let union: std::collections::BTreeSet<&String> = parts.iter().flat_map(|p| p.reach.iter()).collect();
        let new_reach = r.reach.iter().any(|v| !hub(v) && !union.contains(v));
        let _ = spec;
        (all_parts_pass && !r.passed, new_reach)
    };
    // The composition rule, checked on every combination: its result must
    // contain what each single part does alone. A part that failed alone
    // must still fail it; every variable a part moved must still move. What
    // is missing is returned (empty = consistent). Genuine physical masking
    // (one fault removing the conditions another needs) shows up here too,
    // listed, for a person to judge.
    let lost = |spec: &ComboSpec, r: &TierResult| -> Vec<String> {
        let mut missing = Vec::new();
        for e in spec.elements() {
            let Some(part) = single.get(&(spec.preset, e.clone())) else { continue };
            if !part.passed && r.passed {
                missing.push(format!("{e}: failed alone, passes combined"));
            }
            let gone: Vec<&String> = part.reach.difference(&r.reach).filter(|v| !hub(v)).collect();
            if !gone.is_empty() {
                let shown: Vec<&str> = gone.iter().take(3).map(|v| v.as_str()).collect();
                missing.push(format!("{e}: {} of its effects gone ({}{})", gone.len(), shown.join(", "), if gone.len() > 3 { ", ..." } else { "" }));
            }
        }
        missing
    };
    let mut lost_total = 0usize;
    let mut lost_lines: Vec<String> = Vec::new();
    let mut interacting: Vec<(ComboSpec, std::collections::BTreeSet<String>)> = Vec::new();
    // Every interacting combination, all tiers: (tier, spec, fails only
    // together, moves new variables) -- for interactions.json and the graph.
    let mut graph: Vec<(usize, ComboSpec, bool, bool)> = Vec::new();
    let (mut emergent_fail, mut emergent_reach) = (0, 0);
    let mut interaction_lines = Vec::new();
    for (k, (spec, r)) in pairs.iter().zip(&r2).enumerate() {
        let Some(r) = r else { continue };
        let parts = parts_of(spec);
        let (ef, er) = classify(spec, r, &parts);
        let missing = lost(spec, r);
        if !missing.is_empty() {
            lost_total += 1;
            if lost_lines.len() < 300 {
                lost_lines.push(format!("  {}  -- {}", spec.to_line(), missing.join("; ")));
            }
        }
        let _ = k;
        if ef {
            emergent_fail += 1;
        }
        if er {
            emergent_reach += 1;
        }
        if ef || er {
            graph.push((2, spec.clone(), ef, er));
            if interaction_lines.len() < 200 {
                interaction_lines.push(format!("  {} {}", if ef { "FAILS" } else { "reach" }, spec.to_line()));
            }
            interacting.push((spec.clone(), r.reach.clone()));
        }
    }
    let _ = writeln!(
        report,
        "tier 2: {n_overlap_run} pairs run, {emergent_fail} fail only together, {emergent_reach} move new variables; {:.0} s",
        started.elapsed().as_secs_f64()
    );
    score_tier(2, &pairs, &r2, &graph, &mut report);

    // ---- tiers 3 and 4: each interacting combination + one more element
    // overlapping it; a tier stops the climb when it finds nothing new ----
    let mut known: BTreeMap<ComboSpec, TierResult> = BTreeMap::new();
    for (spec, r) in pairs.iter().zip(r2) {
        if let Some(r) = r {
            known.insert(spec.clone(), r);
        }
    }
    for tier in 3..=4usize {
        let cap = if tier == 3 { cfg.t3_max } else { cfg.t4_max };
        if interacting.is_empty() {
            let _ = writeln!(report, "tier {tier}: skipped (tier {} found no interactions)", tier - 1);
            break;
        }
        let mut next: std::collections::BTreeSet<ComboSpec> = std::collections::BTreeSet::new();
        let per_parent = (cap / interacting.len().max(1)).clamp(1, 64);
        for (spec, reach) in &interacting {
            let have = spec.elements();
            let mut cands: Vec<(f64, &String)> = single
                .iter()
                .filter(|((p, k), _)| *p == spec.preset && !have.contains(k))
                .map(|((_, k), r)| (overlap_score(r, reach), k))
                .filter(|(s, _)| *s > 0.0)
                .collect();
            cands.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
            for (_, k) in cands.into_iter().take(per_parent) {
                next.insert(spec.with(k));
            }
            if next.len() >= cap {
                break;
            }
        }
        let mut specs: Vec<ComboSpec> = next.into_iter().take(cap).collect();
        println!("tier {tier}: {} combinations", specs.len());
        progress_report(&report);
        progress_tier(&format!("tier {tier}: {} combinations", specs.len()));
        let results = run_tier(&cfg, tier, &mut specs);
        score_tier(tier, &specs, &results, &graph, &mut report);
        let (mut ef_n, mut er_n) = (0, 0);
        let mut found = Vec::new();
        for (spec, r) in specs.iter().zip(results) {
            let Some(r) = r else { continue };
            // Parts: every sub-combination one element smaller that has been
            // run, plus the singles.
            let elems = spec.elements();
            let mut subs: Vec<ComboSpec> = Vec::new();
            for skip in 0..elems.len() {
                let mut sub = ComboSpec { preset: spec.preset, failures: vec![], breakers: vec![] };
                for (i, k) in elems.iter().enumerate() {
                    if i != skip {
                        sub = sub.with(k);
                    }
                }
                subs.push(sub);
            }
            let mut parts: Vec<&TierResult> = subs.iter().filter_map(|s| known.get(s)).collect();
            parts.extend(parts_of(spec));
            let (ef, er) = classify(spec, &r, &parts);
            let missing = lost(spec, &r);
            if !missing.is_empty() {
                lost_total += 1;
                if lost_lines.len() < 300 {
                    lost_lines.push(format!("  {}  -- {}", spec.to_line(), missing.join("; ")));
                }
            }
            ef_n += ef as usize;
            er_n += er as usize;
            if (ef || er) && interaction_lines.len() < 600 {
                interaction_lines.push(format!("  {} {}", if ef { "FAILS" } else { "reach" }, spec.to_line()));
            }
            if ef || er {
                found.push((spec.clone(), r.reach.clone()));
                graph.push((tier, spec.clone(), ef, er));
            }
            known.insert(spec.clone(), r);
        }
        let _ = writeln!(
            report,
            "tier {tier}: {} combinations, {ef_n} fail only together, {er_n} move new variables; {:.0} s",
            specs.len(),
            started.elapsed().as_secs_f64()
        );
        interacting = found;
    }
    // ---- tier 5: anything and everything. Random combinations of 5-12
    // failures and breakers drawn from the whole catalogue (not guided by
    // the graph), each on a random start state. Checks what the guided tiers
    // might have missed: any failure here the tiers below did not predict.
    let elements: Vec<String> = single.keys().filter(|(p, _)| *p == 0).map(|(_, k)| k.clone()).collect();
    let mut chaos: std::collections::BTreeSet<ComboSpec> = std::collections::BTreeSet::new();
    let mut crng = Rng::new(cfg.master_seed ^ 0x5CA05);
    let mut attempts = 0usize;
    // The rare tier: a small sample (a twentieth of the other tiers' cap),
    // each case predicted from tiers 1-4 before it runs.
    let chaos_n = cfg.t5_max;
    while chaos.len() < chaos_n && attempts < chaos_n * 4 && !elements.is_empty() {
        attempts += 1;
        let preset = crng.range_usize(0, 4);
        let n = crng.range_usize(5, 13);
        let mut spec = ComboSpec { preset, failures: vec![], breakers: vec![] };
        for _ in 0..n {
            let k = &elements[crng.range_usize(0, elements.len())];
            if !spec.elements().contains(k) {
                spec = spec.with(k);
            }
        }
        chaos.insert(spec);
    }
    let mut chaos: Vec<ComboSpec> = chaos.into_iter().collect();
    println!("tier 5: {} random combinations", chaos.len());
    progress_report(&report);
    progress_tier("tier 5: random combinations");
    let r5 = run_tier(&cfg, 5, &mut chaos);
    let predictor = Predictor::from_tiers(&single, &graph);
    let (mut f5, mut surprise5) = (0usize, 0usize);
    let (mut tp, mut tn, mut fp, mut fn_) = (0usize, 0usize, 0usize, 0usize);
    for (spec, r) in chaos.iter().zip(&r5) {
        let Some(r) = r else { continue };
        let missing = lost(spec, r);
        if !missing.is_empty() {
            lost_total += 1;
            if lost_lines.len() < 300 {
                lost_lines.push(format!("  {}  -- {}", spec.to_line(), missing.join("; ")));
            }
        }
        match (predictor.predict_fail(spec), !r.passed) {
            (true, true) => tp += 1,
            (false, false) => tn += 1,
            (true, false) => fp += 1,
            (false, true) => fn_ += 1,
        }
        if !r.passed {
            f5 += 1;
            // A surprise: it failed though every one of its parts passed alone.
            let parts = parts_of(spec);
            if parts.iter().all(|p| p.passed) {
                surprise5 += 1;
                graph.push((5, spec.clone(), true, false));
                if interaction_lines.len() < 800 {
                    interaction_lines.push(format!("  FAILS {}", spec.to_line()));
                }
            }
        }
    }
    let _ = writeln!(
        report,
        "tier 5: {} random combinations of 5-12, {f5} failed, {surprise5} of those with every part passing alone; {:.0} s",
        chaos.len(),
        started.elapsed().as_secs_f64()
    );
    let scored = (tp + tn + fp + fn_).max(1);
    let _ = writeln!(
        report,
        "tier 5 predicted from tiers 1-4 before running: {:.1}% right ({tp} failures foreseen, {tn} passes foreseen, {fn_} failures missed, {fp} false alarms)",
        100.0 * (tp + tn) as f64 / scored as f64
    );

    // ---- interactions.json: nodes (every failure and breaker, with how it
    // did alone per start state) and every interacting combination ----
    let mut json = String::from("{\"nodes\":[");
    let mut first = true;
    for k in &elements {
        let passes: Vec<String> =
            (0..4).map(|p| single.get(&(p, k.clone())).map_or("null".to_owned(), |r| r.passed.to_string())).collect();
        let reach = single.get(&(0, k.clone())).map_or(0, |r| r.reach.len());
        if !first {
            json.push(',');
        }
        first = false;
        let _ = write!(json, "{{\"id\":\"{}\",\"passedAlone\":[{}],\"reach\":{reach}}}", json_escape(k), passes.join(","));
    }
    json.push_str("],\"combos\":[");
    for (i, (tier, spec, ef, er)) in graph.iter().enumerate() {
        if i > 0 {
            json.push(',');
        }
        let els: Vec<String> = spec.elements().iter().map(|e| format!("\"{}\"", json_escape(e))).collect();
        let _ = write!(json, "{{\"tier\":{tier},\"preset\":{},\"elements\":[{}],\"failsTogether\":{ef},\"newReach\":{er}}}", spec.preset, els.join(","));
    }
    json.push_str("]}");
    let _ = std::fs::write(cfg.dump_dir.join("interactions.json"), json);

    let _ = writeln!(
        report,
        "\ncomposition check: {lost_total} combinations are missing a part's failure or effects (FAILED by the composition rule; each listed below -- physical masking or a bug)"
    );
    for l in &lost_lines {
        let _ = writeln!(report, "{l}");
    }
    let _ = writeln!(report, "\ninteractions (preset|failures|breakers; FAILS = fails only in combination):");
    for l in &interaction_lines {
        let _ = writeln!(report, "{l}");
    }
    let _ = std::fs::write(cfg.dump_dir.join("TIERS.txt"), &report);
    print!("{report}");
}

fn main() {
    let raw: Vec<String> = std::env::args().collect();
    if raw.get(1).is_some_and(|a| a == "--predict") {
        predict_main(&raw);
        return;
    }
    let cfg = parse_args();
    if cfg.worker {
        worker_main(cfg);
    } else if cfg.tiered {
        tiered_main(cfg);
    } else {
        orchestrator_main(cfg);
    }
}


// =====================================================================
// Progress page: http://127.0.0.1:8790 while a tiered run is going.
// =====================================================================

struct Progress {
    label: String,
    dir: PathBuf,
    total: usize,
    done_before: usize,
    first_id: usize,
    tier_started: Option<std::time::Instant>,
    run_started: Option<std::time::Instant>,
    report: String,
    finished: Vec<String>,
    samples: std::collections::VecDeque<(f64, usize)>,
}

static PROGRESS: std::sync::Mutex<Progress> = std::sync::Mutex::new(Progress {
    label: String::new(),
    dir: PathBuf::new(),
    total: 0,
    done_before: 0,
    first_id: usize::MAX,
    tier_started: None,
    run_started: None,
    report: String::new(),
    finished: Vec::new(),
    samples: std::collections::VecDeque::new(),
});

fn progress_tier(label: &str) {
    if let Ok(mut p) = PROGRESS.lock() {
        p.label = label.to_owned();
    }
}

fn progress_report(report: &str) {
    if let Ok(mut p) = PROGRESS.lock() {
        p.report = report.to_owned();
    }
}

/// A tier's run starting (`first_id` its first worker) or, with
/// `first_id == usize::MAX`, finished.
fn progress_run(dir: &std::path::Path, total: usize, done_before: usize, first_id: usize) {
    if let Ok(mut p) = PROGRESS.lock() {
        if first_id == usize::MAX {
            if let Some(t) = p.tier_started {
                let line = format!("{}: {total} cases in {:.0} s", p.label, t.elapsed().as_secs_f64());
                p.finished.push(line);
            }
            p.total = total;
            p.done_before = total;
            p.first_id = usize::MAX;
        } else {
            p.dir = dir.to_owned();
            p.total = total;
            p.done_before = done_before;
            p.first_id = first_id;
            p.tier_started = Some(std::time::Instant::now());
            p.samples.clear();
        }
    }
}

/// Every running worker of the current tier: (id, cases, passed, failed,
/// hung, panicked), from the summaries they rewrite every few cases.
fn worker_counts(dir: &std::path::Path, first_id: usize) -> Vec<[u64; 6]> {
    let mut out = Vec::new();
    if first_id == usize::MAX {
        return out;
    }
    for entry in std::fs::read_dir(dir).into_iter().flatten().flatten() {
        let name = entry.file_name().to_string_lossy().into_owned();
        let Some(id) = name.strip_prefix("worker_").and_then(|n| n.strip_suffix("_summary.txt")).and_then(|n| n.parse::<usize>().ok()) else { continue };
        if id < first_id {
            continue;
        }
        let Ok(text) = std::fs::read_to_string(entry.path()) else { continue };
        let head = text.lines().next().unwrap_or("");
        let field = |k: &str| head.split_whitespace().find_map(|t| t.strip_prefix(k)).and_then(|v| v.parse::<u64>().ok()).unwrap_or(0);
        out.push([(id - first_id) as u64, field("cases_run="), field("pass="), field("fail="), field("hangs="), field("panics=")]);
    }
    out.sort_by_key(|w| w[0]);
    out
}

fn json_quoted(s: &str) -> String {
    let mut o = String::with_capacity(s.len() + 2);
    o.push('"');
    for c in s.chars() {
        match c {
            '"' => o.push_str("\\\""),
            '\\' => o.push_str("\\\\"),
            '\n' => o.push_str("\\n"),
            c if (c as u32) < 0x20 => o.push(' '),
            c => o.push(c),
        }
    }
    o.push('"');
    o
}

fn progress_json(workers_cfg: usize) -> String {
    let (label, dir, total, done_before, first_id, tier_started, run_started, report, finished) = match PROGRESS.lock() {
        Ok(p) => (p.label.clone(), p.dir.clone(), p.total, p.done_before, p.first_id, p.tier_started, p.run_started, p.report.clone(), p.finished.clone()),
        Err(_) => return "{}".into(),
    };
    let workers = worker_counts(&dir, first_id);
    let run_now: u64 = workers.iter().map(|w| w[1]).sum();
    let done = (done_before as u64 + run_now).min(total as u64);
    let (pass, fail, hung, panics) = workers.iter().fold((0, 0, 0, 0), |a, w| (a.0 + w[2], a.1 + w[3], a.2 + w[4], a.3 + w[5]));
    let now = run_started.map_or(0.0, |t| t.elapsed().as_secs_f64());
    // Rate over the last minute of samples.
    let rate = if let Ok(mut p) = PROGRESS.lock() {
        p.samples.push_back((now, run_now as usize));
        while p.samples.len() > 2 && now - p.samples[0].0 > 60.0 {
            p.samples.pop_front();
        }
        let (t0, d0) = p.samples[0];
        if now - t0 > 1.0 { (run_now as f64 - d0 as f64) / (now - t0) } else { 0.0 }
    } else {
        0.0
    };
    let eta = if rate > 0.0 { (total as f64 - done as f64) / rate } else { -1.0 };
    let worker_json: Vec<String> = workers.iter().map(|w| format!("[{},{},{},{},{},{}]", w[0], w[1], w[2], w[3], w[4], w[5])).collect();
    let finished_json: Vec<String> = finished.iter().map(|f| json_quoted(f)).collect();
    format!(
        "{{\"label\":{},\"total\":{total},\"done\":{done},\"pass\":{pass},\"fail\":{fail},\"hung\":{hung},\"panics\":{panics},\"rate\":{rate:.2},\"eta\":{eta:.0},\"elapsed\":{now:.0},\"tierElapsed\":{:.0},\"workersConfigured\":{workers_cfg},\"workers\":[{}],\"finished\":[{}],\"report\":{}}}",
        json_quoted(&label),
        tier_started.map_or(0.0, |t| t.elapsed().as_secs_f64()),
        worker_json.join(","),
        finished_json.join(","),
        json_quoted(&report)
    )
}

const PROGRESS_PAGE: &str = r##"<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Battery progress</title>
<style>
:root{--bg:#15171a;--card:#1f2226;--line:#33373d;--text:#e6e6e6;--dim:#9aa0a6;--ok:#3ec46d;--bad:#e5484d;--warn:#f0b429;--accent:#4aa3ff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:14px/1.45 "Segoe UI",system-ui,sans-serif;padding:24px 16px}
main{max-width:1100px;margin:0 auto}h1{font-size:20px;margin:0 0 4px}.dim{color:var(--dim)}
.card{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:16px;margin:14px 0}
.bar{height:14px;background:var(--line);border-radius:7px;overflow:hidden}.bar>div{height:100%;background:var(--accent);transition:width .6s}
.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:10px;margin-top:12px}
.stat .v{font-size:22px;font-variant-numeric:tabular-nums}.stat .k{color:var(--dim);font-size:12px;text-transform:uppercase;letter-spacing:.5px}
.workers{display:grid;grid-template-columns:repeat(auto-fill,minmax(92px,1fr));gap:6px}
.w{border:1px solid var(--line);border-radius:6px;padding:6px 8px;font-variant-numeric:tabular-nums;font-size:12px}.w b{display:block;font-size:15px}
pre{white-space:pre-wrap;margin:0;font:12px/1.5 ui-monospace,Consolas,monospace;color:var(--dim)}
.ok{color:var(--ok)}.bad{color:var(--bad)}.warn{color:var(--warn)}
</style></head><body><main>
<h1>Battery progress</h1><div class="dim" id="label">waiting…</div>
<div class="card"><div class="bar"><div id="bar" style="width:0"></div></div>
<div class="stats">
<div class="stat"><div class="v" id="done">–</div><div class="k">cases this tier</div></div>
<div class="stat"><div class="v" id="pct">–</div><div class="k">complete</div></div>
<div class="stat"><div class="v" id="rate">–</div><div class="k">cases / s</div></div>
<div class="stat"><div class="v" id="eta">–</div><div class="k">tier ETA</div></div>
<div class="stat"><div class="v ok" id="pass">–</div><div class="k">passed</div></div>
<div class="stat"><div class="v bad" id="fail">–</div><div class="k">failed</div></div>
<div class="stat"><div class="v warn" id="hung">–</div><div class="k">hung / panicked</div></div>
<div class="stat"><div class="v" id="elapsed">–</div><div class="k">run elapsed</div></div>
</div></div>
<div class="card"><div class="k dim" style="margin-bottom:8px">Workers (cases, failures)</div><div class="workers" id="workers"></div></div>
<div class="card"><div class="k dim" style="margin-bottom:8px">Finished</div><pre id="finished">–</pre></div>
<div class="card"><div class="k dim" style="margin-bottom:8px">Report so far</div><pre id="report">–</pre></div>
</main><script>
const $=id=>document.getElementById(id);
const t=s=>{if(s<0)return'–';s=Math.round(s);const h=Math.floor(s/3600),m=Math.floor(s%3600/60),x=s%60;return h?`${h}h ${m}m`:m?`${m}m ${x}s`:`${x}s`};
async function tick(){try{const d=await(await fetch('/progress')).json();
$('label').textContent=d.label||'starting…';const pct=d.total?100*d.done/d.total:0;$('bar').style.width=pct.toFixed(2)+'%';
$('done').textContent=`${d.done.toLocaleString()} / ${d.total.toLocaleString()}`;$('pct').textContent=pct.toFixed(1)+'%';
$('rate').textContent=d.rate.toFixed(1);$('eta').textContent=t(d.eta);$('pass').textContent=d.pass.toLocaleString();$('fail').textContent=d.fail.toLocaleString();
$('hung').textContent=`${d.hung} / ${d.panics}`;$('elapsed').textContent=t(d.elapsed);
$('workers').replaceChildren(...d.workers.map(w=>{const e=document.createElement('div');e.className='w';e.innerHTML=`<span class="dim">#${w[0]}</span><b>${w[1]}</b><span class="${w[3]?'bad':'dim'}">${w[3]} failed</span>`;return e}));
$('finished').textContent=d.finished.length?d.finished.join('\n'):'–';$('report').textContent=d.report||'–';
}catch(e){$('label').textContent='battery not reachable (finished or stopped)'}}
tick();setInterval(tick,2000);
</script></body></html>"##;

fn start_progress_server(workers: usize) {
    if let Ok(mut p) = PROGRESS.lock() {
        p.run_started = Some(std::time::Instant::now());
    }
    let Ok(listener) = std::net::TcpListener::bind(("127.0.0.1", 8790)) else {
        println!("progress page: port 8790 busy, no page this run");
        return;
    };
    println!("progress page: http://127.0.0.1:8790");
    std::thread::spawn(move || {
        use std::io::{Read, Write};
        for stream in listener.incoming().flatten() {
            let mut stream = stream;
            let mut buf = [0u8; 2048];
            let n = stream.read(&mut buf).unwrap_or(0);
            let req = String::from_utf8_lossy(&buf[..n]);
            let path = req.split_whitespace().nth(1).unwrap_or("/");
            let (kind, body) = if path.starts_with("/progress") {
                ("application/json", progress_json(workers))
            } else {
                ("text/html; charset=utf-8", PROGRESS_PAGE.to_owned())
            };
            let _ = write!(stream, "HTTP/1.1 200 OK\r\nContent-Type: {kind}\r\nContent-Length: {}\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n{body}", body.len());
        }
    });
}
