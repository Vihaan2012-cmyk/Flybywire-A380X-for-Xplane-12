//! Physical-bounds guard for the shared quantity substrate: `Vars`'s
//! `read`/`write`/`write_from_xplane` (`src/lib.rs`), and a couple of
//! solver-output points in `fuel_network.rs`.
//!
//! Nine systems (fuel, hydraulics, engines, electrical, air/bleed, gear,
//! avionics, ...) are being built in parallel and coupled together, with
//! continuous partial failures flowing between them. One bad coupling can
//! turn into a NaN or a negative pressure/mass that then cascades through
//! every system reading that quantity. This module is the net: it never
//! changes *what* a system computes, only catches the physically
//! impossible values crossing the substrate, clamps the ones with a known
//! floor/range, and logs every one so the log becomes a punch list of
//! which coupling is wrong.
//!
//! Performance: the bound for a named quantity is looked up by substring
//! match on the name (see [`bound_for`]), but that lookup happens exactly
//! once, when the variable is first registered (`Vars::add`). The
//! per-write check in the hot path ([`check`]) is then just a non-finite
//! test plus, in the rare case a bound exists, one or two comparisons --
//! no string matching per write/per tick.

use std::collections::HashMap;
use std::sync::Mutex;
use std::sync::atomic::{AtomicU64, Ordering};

/// A physical bound for a quantity. Computed once per slot at
/// registration (see `Vars::add` in `lib.rs`) from the variable's name,
/// then reused on every write.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Bound {
    /// No known floor or ceiling for this name (e.g. it is signed and
    /// legitimately can go either way: differential pressure, vertical
    /// speed, current direction, attitude, trim...). Still checked for
    /// non-finite values.
    None,
    /// Cannot go negative: mass, absolute pressure, quantity, volume,
    /// capacity, density, RPM/N1/N2/N3, flow rate.
    NonNegative,
    /// Absolute temperature: cannot go below absolute zero. The floor is
    /// -273.15 because FBW/X-Plane's temperature simvars in `Vars` are
    /// Celsius; a Kelvin-scale quantity would use a 0.0 floor instead (see
    /// the direct calls from `physics/`, which pass their own bound).
    TemperatureFloor(f64),
    /// A closed range, e.g. a 0..1 fraction/ratio.
    Range(f64, f64),
}

impl Bound {
    /// `None` if `value` is within bound (or non-finite -- that is handled
    /// separately, unconditionally, before this is reached), `Some(clamped)`
    /// otherwise.
    #[inline]
    fn violation(self, value: f64) -> Option<f64> {
        match self {
            Bound::None => None,
            Bound::NonNegative => (value < 0.).then_some(0.),
            Bound::TemperatureFloor(floor) => (value < floor).then_some(floor),
            Bound::Range(lo, hi) => {
                if value < lo {
                    Some(lo)
                } else if value > hi {
                    Some(hi)
                } else {
                    None
                }
            }
        }
    }
}

/// FBW's ARINC429-style "no data" sentinel: several of FlyByWire's own
/// simple (non-word-encoded) simvars and aircraft variables use -1 to mean
/// "not applicable / no computed data" (distinct from the full
/// `Arinc429Word` SSM encoding used internally, which is not exposed as a
/// single dataref value). A bare -1 must never be floor-clamped to 0 or it
/// silently turns "no data" into "definitely zero", which is worse than
/// the sentinel.
const ARINC_NO_DATA_SENTINEL: f64 = -1.0;

/// Table of bounds keyed by a case-insensitive substring of the variable
/// name. Order matters: the signed-quantity exemptions are checked first
/// so they win over a generic keyword that would otherwise also match
/// (e.g. "differential pressure" contains "pressure").
///
/// Every entry below is a physical floor/range, not a guess: see the
/// per-group comments for the justification.
pub fn bound_for(name: &str) -> Bound {
    let n = name.to_ascii_uppercase();

    // Signed quantities that legitimately go negative. Never clamp these;
    // only the universal non-finite guard applies.
    const SIGNED_KEYWORDS: &[&str] = &[
        "DELTA", "DIFFERENTIAL", "DIFF_PRESS", "DIFF PRESS", "VVI", "VERTICAL SPEED", "VS_",
        "CURRENT", "AMPERE", "AMP_", "HEADING", "TRACK", "LATITUDE", "LONGITUDE", "PITCH",
        "ROLL", "YAW", "BANK", "SIDESLIP", "AOA", "RATE OF TURN", "ACCEL", "G FORCE", "WIND",
        "SLIP", "TORQUE", "ERROR", "OFFSET", "TRIM", "DEVIATION", "DEV_", "IMBALANCE",
        // Altitudes and heights are signed: pressure altitude is negative on
        // a high-pressure day, and elevations below sea level exist.
        "ALTITUDE", "ALT_", "ELEVATION", "HEIGHT",
        // Pneumatic transducers report gauge pressure (container minus
        // ambient, FBW `PressureTransducer::update`): below ambient is real.
        "TRANSDUCER_PRESSURE",
    ];
    if SIGNED_KEYWORDS.iter().any(|k| n.contains(k)) {
        return Bound::None;
    }

    // Absolute temperature cannot go below absolute zero. FBW/X-Plane
    // temperature simvars reaching `Vars` are Celsius (-273.15 floor);
    // solver-internal Kelvin state uses its own 0.0 floor directly (see
    // fuel_network.rs / physics::engine, not this table).
    if n.contains("TEMP") {
        return Bound::TemperatureFloor(-273.15);
    }

    // A normalized 0..1 fraction, so MSFS's 0..100-scaled percent simvars
    // are not caught here by accident.
    // Only explicit fractions: a "ratio" is not bounded by 1 in general
    // (pressure, bypass and expansion ratios all exceed it).
    if n.contains("FRACTION") {
        return Bound::Range(0.0, 1.0);
    }

    // Continuous component-aging/degradation magnitudes (electrical-sources
    // workstream: engine_generator.rs, transformer_rectifier.rs,
    // battery.rs, static_inverter.rs): 0 = new/undegraded, 1 = maximum
    // modelled degradation. Checked before the generic "GROWTH"/"CAPACITY"
    // NonNegative keywords below so these get the tighter closed range
    // instead.
    const DEGRADATION_KEYWORDS: &[&str] =
        &["DEGRADATION", "RESISTANCE_GROWTH", "CAPACITY_FADE"];
    if DEGRADATION_KEYWORDS.iter().any(|k| n.contains(k)) {
        return Bound::Range(0.0, 1.0);
    }

    // GCU voltage-regulator drift magnitude (engine_generator.rs): signed,
    // -1 (full low drift) .. +1 (full high drift), 0 = perfectly trimmed.
    if n.contains("REGULATOR_DRIFT") {
        return Bound::Range(-1.0, 1.0);
    }

    // Cannot be negative: absolute pressure (a vacuum is 0 Pa, not
    // negative -- only *differential* pressure, excluded above, can be
    // signed), mass, fuel/hydraulic quantity, weight, volume, tank/line
    // capacity, fluid density, spool/rotor speed (RPM, N1/N2/N3) and flow
    // rate.
    const NONNEGATIVE_KEYWORDS: &[&str] = &[
        "PRESSURE",
        "MASS",
        "QUANTITY",
        "WEIGHT",
        "VOLUME",
        "CAPACITY",
        "DENSITY",
        "FUEL FLOW",
        "FUEL_FLOW",
        "FLOW RATE",
        "RPM",
        " N1",
        " N2",
        " N3",
        "N1:",
        "N2:",
        "N3:",
    ];
    if NONNEGATIVE_KEYWORDS.iter().any(|k| n.contains(k)) {
        return Bound::NonNegative;
    }

    Bound::None
}

/// How often (in simulated seconds) a repeat violation of the same
/// quantity is re-logged. The count keeps accumulating between log lines;
/// nothing is lost, the log is just not flooded every tick.
const DEDUP_WINDOW_SECS: f64 = 5.0;

struct Entry {
    /// Simulated seconds (see [`advance_tick`]) this quantity was last
    /// written to the X-Plane log.
    last_logged_at: f64,
    /// Tick this quantity was last written to the log.
    last_logged_tick: u64,
    /// Violations of this quantity since the last log line.
    since_log: u64,
    /// Violations of this quantity since the process started.
    total: u64,
    last_value: f64,
    bound: Bound,
}

/// A snapshot row for [`report`]: what a test or the Study panel's
/// scenario debugger can read back about one quantity that hit a bound.
#[derive(Clone, Debug, PartialEq)]
pub struct Violation {
    pub name: String,
    pub total: u64,
    pub last_value: f64,
    pub bound: Bound,
    pub last_tick: u64,
}

static STATE: Mutex<Option<HashMap<String, Entry>>> = Mutex::new(None);
static TICK: AtomicU64 = AtomicU64::new(0);
static ELAPSED_SECS: Mutex<f64> = Mutex::new(0.0);
/// Non-finite or out-of-bound writes caught since the process started, for
/// test/debug-build visibility without panicking mid-tick (other systems
/// are deliberately injecting partial failures; a hard panic here would
/// take the whole simulation down for a fault this module exists to
/// contain).
static VIOLATION_COUNT: AtomicU64 = AtomicU64::new(0);

/// Called once per plugin tick (`Plugin::tick`) with the frame's delta, so
/// dedup windows are measured in simulated seconds rather than ticks
/// (ticks can vary in wall-clock length).
pub fn advance_tick(delta_secs: f64) {
    TICK.fetch_add(1, Ordering::Relaxed);
    let mut secs = ELAPSED_SECS.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
    *secs += delta_secs.max(0.0);
}

fn now_tick() -> u64 {
    TICK.load(Ordering::Relaxed)
}

fn now_secs() -> f64 {
    *ELAPSED_SECS.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
}

/// Check and, where a bound applies, clamp `value` for the named quantity.
/// `hint` is a short source label (e.g. `"Vars::write"`,
/// `"fuel_network tank"`) that lands in the log line to help pin down
/// which accessor saw the bad value.
///
/// Always returns a finite value: a non-finite input is replaced (0.0, or
/// the bound's floor/midpoint if one applies) and logged regardless of
/// `bound`, since NaN/inf must never propagate no matter the quantity.
pub fn check(name: &str, value: f64, bound: Bound, hint: &str) -> f64 {
    if value.is_finite() {
        if let Some(clamped) = bound.violation(value) {
            record(name, value, clamped, bound, hint);
            return clamped;
        }
        return value;
    }
    // Non-finite: fall back to the bound's floor when it has one (a NaN
    // pressure is more safely 0 than left alone), otherwise 0.
    let fallback = match bound {
        Bound::NonNegative => 0.0,
        Bound::TemperatureFloor(floor) => floor,
        Bound::Range(lo, _) => lo,
        Bound::None => 0.0,
    };
    record(name, value, fallback, bound, hint);
    fallback
}

/// The ARINC "no data" sentinel is exempt from clamping (but a NaN/inf is
/// still caught even for a sentinel-carrying variable -- NaN is never a
/// valid encoding of "no data").
#[inline]
pub fn is_arinc_sentinel(value: f64) -> bool {
    value.is_finite() && value == ARINC_NO_DATA_SENTINEL
}

fn record(name: &str, attempted: f64, clamped: f64, bound: Bound, hint: &str) {
    VIOLATION_COUNT.fetch_add(1, Ordering::Relaxed);
    // "Fail loudly in debug/test builds": the test-visible counter above
    // (and `report()`) is the primary signal, checked explicitly by every
    // test in this module. A hard `debug_assert` panic is also available,
    // opt-in via FBW_INVARIANTS_PANIC=1, but is NOT the default even in
    // debug builds: nine systems are deliberately injecting continuous
    // partial failures into these same couplings, so an unconditional
    // panic here would turn an expected, now-safely-clamped fault into an
    // unrelated crash of the whole simulation -- the opposite of this
    // module's job.
    debug_assert!(
        cfg!(test) || std::env::var_os("FBW_INVARIANTS_PANIC").is_none(),
        "invariant: {name} = {attempted} out of bounds ({bound:?}), clamped to {clamped} [{hint}]"
    );

    let tick = now_tick();
    let secs = now_secs();
    let mut guard = STATE.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
    let map = guard.get_or_insert_with(HashMap::new);
    let entry = map.entry(name.to_string()).or_insert(Entry {
        last_logged_at: f64::NEG_INFINITY,
        last_logged_tick: 0,
        since_log: 0,
        total: 0,
        last_value: attempted,
        bound,
    });
    entry.total += 1;
    entry.since_log += 1;
    entry.last_value = attempted;
    entry.bound = bound;

    if secs - entry.last_logged_at >= DEDUP_WINDOW_SECS {
        let repeats = entry.since_log;
        entry.last_logged_at = secs;
        entry.last_logged_tick = tick;
        entry.since_log = 0;
        let total = entry.total;
        drop(guard);
        crate::log(&format!(
            "invariant: {name} = {attempted} out of bounds ({bound:?}), clamped to {clamped} \
             [{hint}] (x{repeats} since last log, {total} total, tick {tick})"
        ));
    }
}

/// Snapshot of every quantity that has hit a bound since the process (or
/// last [`reset`]) started, for tests and the Study panel's scenario
/// debugger. Sorted by name for a stable, diffable order.
pub fn report() -> Vec<Violation> {
    let guard = STATE.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
    let mut rows: Vec<Violation> = guard
        .iter()
        .flat_map(|m| m.iter())
        .map(|(name, e)| Violation {
            name: name.clone(),
            total: e.total,
            last_value: e.last_value,
            bound: e.bound,
            last_tick: e.last_logged_tick,
        })
        .collect();
    rows.sort_by(|a, b| a.name.cmp(&b.name));
    rows
}

/// Total violations caught since start/last [`reset`], including ones
/// folded into a dedup window and not yet (or never) logged.
pub fn violation_count() -> u64 {
    VIOLATION_COUNT.load(Ordering::Relaxed)
}

/// Clears all recorded state. Tests must call this at setup: the counters
/// are process-global (this runs on every write, on every tick, across
/// the whole plugin, so it cannot be instance-scoped without either a
/// `RefCell` threaded through every system or a lookup keyed by something
/// heavier than a slot index) and would otherwise leak counts between
/// tests.
pub fn reset() {
    *STATE.lock().unwrap_or_else(std::sync::PoisonError::into_inner) = None;
    TICK.store(0, Ordering::Relaxed);
    *ELAPSED_SECS.lock().unwrap_or_else(std::sync::PoisonError::into_inner) = 0.0;
    VIOLATION_COUNT.store(0, Ordering::Relaxed);
}

#[cfg(test)]
mod tests {
    use super::*;

    // The module under test is process-global state (see `reset`'s doc
    // comment), but `cargo test` runs tests in parallel threads within one
    // process. Without serializing, two of these tests interleave and
    // corrupt each other's counts. This lock (held for a whole test, not
    // just around `reset`) keeps this module's tests -- and only this
    // module's tests -- to one at a time.
    static TEST_LOCK: Mutex<()> = Mutex::new(());

    fn setup() -> std::sync::MutexGuard<'static, ()> {
        let guard = TEST_LOCK.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
        reset();
        guard
    }

    #[test]
    fn nan_is_replaced_and_logged() {
        let _guard = setup();
        let out = check("FUEL QUANTITY:1", f64::NAN, Bound::NonNegative, "test");
        assert_eq!(out, 0.0);
        assert_eq!(violation_count(), 1);
        let rows = report();
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].name, "FUEL QUANTITY:1");
    }

    #[test]
    fn infinity_is_replaced_and_logged() {
        let _guard = setup();
        let out = check("HYD PRESSURE:1", f64::INFINITY, Bound::NonNegative, "test");
        assert_eq!(out, 0.0);
        assert_eq!(violation_count(), 1);
        let out2 = check("ENGINE TEMPERATURE:1", f64::NEG_INFINITY, Bound::TemperatureFloor(-273.15), "test");
        assert_eq!(out2, -273.15);
    }

    #[test]
    fn negative_mass_is_clamped_and_logged() {
        let _guard = setup();
        let out = check("FUEL TANK MASS:1", -12.5, Bound::NonNegative, "test");
        assert_eq!(out, 0.0);
        let rows = report();
        assert_eq!(rows[0].last_value, -12.5);
        assert_eq!(rows[0].total, 1);
    }

    #[test]
    fn negative_pressure_is_clamped_and_logged() {
        let _guard = setup();
        let out = check("HYD SYSTEM PRESSURE:1", -500.0, Bound::NonNegative, "test");
        assert_eq!(out, 0.0);
        assert_eq!(violation_count(), 1);
    }

    #[test]
    fn signed_quantity_passes_untouched() {
        let _guard = setup();
        // Vertical speed and differential pressure legitimately go
        // negative; bound_for must return None for them, and check() must
        // not alter a finite in-range value.
        assert_eq!(bound_for("VERTICAL SPEED"), Bound::None);
        assert_eq!(bound_for("HYD DIFFERENTIAL PRESSURE:1"), Bound::None);
        assert_eq!(bound_for("ELEC BAT CURRENT:1"), Bound::None);
        let out = check("VERTICAL SPEED", -1200.0, bound_for("VERTICAL SPEED"), "test");
        assert_eq!(out, -1200.0);
        assert_eq!(violation_count(), 0);
    }

    #[test]
    fn arinc_sentinel_is_whitelisted() {
        let _guard = setup();
        assert!(is_arinc_sentinel(-1.0));
        assert!(!is_arinc_sentinel(-1.5));
        assert!(!is_arinc_sentinel(f64::NAN));
    }

    #[test]
    fn dedup_folds_repeats_within_the_window_but_keeps_counting() {
        let _guard = setup();
        // Three violations within the same simulated instant: only the
        // Entry's `total` should reflect all three, exercised via check().
        check("APU FUEL QUANTITY", -1.0, Bound::NonNegative, "test");
        check("APU FUEL QUANTITY", -2.0, Bound::NonNegative, "test");
        check("APU FUEL QUANTITY", -3.0, Bound::NonNegative, "test");
        let rows = report();
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].total, 3);
        assert_eq!(rows[0].last_value, -3.0);
    }

    #[test]
    fn bound_table_covers_documented_examples() {
        assert_eq!(bound_for("HYD SYSTEM 1 PRESSURE"), Bound::NonNegative);
        assert_eq!(bound_for("FUEL TANK 1 QUANTITY"), Bound::NonNegative);
        assert_eq!(bound_for("FUEL TANK 1 CAPACITY"), Bound::NonNegative);
        assert_eq!(bound_for("AMBIENT TEMPERATURE"), Bound::TemperatureFloor(-273.15));
        assert_eq!(bound_for("APU EGT"), Bound::None); // not in the table: unmatched name, no false positive
        assert_eq!(bound_for("VALVE OPEN FRACTION"), Bound::Range(0.0, 1.0));
        assert_eq!(bound_for("COMPRESSOR PRESSURE RATIO"), Bound::NonNegative);
        assert_eq!(bound_for("PRESSURE ALTITUDE"), Bound::None);
    }

    #[test]
    fn reset_clears_state() {
        let _guard = setup();
        check("X", -1.0, Bound::NonNegative, "test");
        assert_eq!(violation_count(), 1);
        reset();
        assert_eq!(violation_count(), 0);
        assert!(report().is_empty());
    }
}
