//! Every variable of every system, live in `Log.txt`, as it changes.
//!
//! The per-area diagnostics this plugin grew (`FBW_HYD_STATS`,
//! `FBW_FCTL_STATS`, `FBW_ENG_STATS`, `FBW_NAV_STATS`, `FBW_SCREEN_STATS`)
//! each answer one question, and each was written *after* the question was
//! already being asked -- so a fault reproduced once, without the right one
//! compiled in, had to be reproduced again. This has no such gap: every
//! system is covered at once, because FlyByWire's own systems, this port's
//! physics and the deep-systems areas all register through one variable
//! registry, and `Plugin::take_snapshot` already copies the whole of it
//! every tick for the panels. This reads that same copy. Nothing new is
//! gathered, and no area has code here -- an area added later is included
//! without a line being written for it.
//!
//! **Changes only, after the first sample.** Logging two thousand-odd
//! variables every interval would be unreadable and would cost frames in
//! exactly the way a file dump does. In the steady state almost nothing
//! moves, so a sample is a handful of names; the interesting moments are
//! precisely the ones where it is not.
//!
//! Names are batched into long lines rather than logged one at a time: each
//! `crate::log` is a call into X-Plane's own logging, and a thousand of them
//! in one frame is the cost this is meant to avoid.
//!
//! Entries are separated by [`SEPARATOR`], not by a space: MSFS's own
//! variable names have spaces in them (`FUELSYSTEM TANK QUANTITY:3`,
//! `PLANE HEADING DEGREES TRUE`), so a space-separated line cannot be split
//! back into name/value pairs by anything reading it afterwards -- which
//! was found the first time one of these lines had to be picked apart.
//!
//! ```text
//! FBW_LOG_ALL=1                       every second, whatever changed
//! FBW_LOG_ALL=0.2                     five times a second
//! FBW_LOG_ALL=1 FBW_LOG_ALL_FILTER=HYD,ENGINE   only those names
//! FBW_LOG_ALL=1 FBW_LOG_ALL_ALL=1     every variable every time, not just changes
//! ```
//!
//! `FBW_LOG_ALL` only shows *changes*, so a variable that moves once and
//! then holds -- DC ESS staying unpowered for tens of seconds (W222), a
//! flight phase that never advances -- leaves no trace between its edges
//! in a long session; a session that only sampled the edges cannot tell a
//! held state from one that flickered unseen in between. [`KeySnapshot`]
//! answers "what is the state right now" instead, for a fixed, small list
//! of the variables a sim-test session is asked about most, on its own
//! clock, regardless of whether anything moved:
//!
//! ```text
//! FBW_LOG_KEY=1                       state, once a second
//! FBW_LOG_KEY=10                      state, every 10 s (typical)
//! ```

/// Roughly one terminal-width-independent line. Long enough that a busy
/// sample is a few lines rather than hundreds, short enough to stay
/// greppable.
use std::collections::HashMap;

const LINE_CHARS: usize = 1600;

/// Between entries. Not a space: MSFS's variable names contain spaces, and
/// a space-separated line cannot be split back into pairs. No registered
/// name contains a semicolon.
const SEPARATOR: &str = "; ";

/// Values this close together are the same number for logging purposes.
///
/// Without it, every filtered or integrated quantity in the aeroplane --
/// spool speeds, pressures, temperatures, filter states -- differs in its
/// last bit every single tick, and "what changed" becomes "everything",
/// which is the unreadable full dump this exists to avoid. A relative
/// tolerance rather than an absolute one, so it means the same thing for a
/// pressure in pascals as for a ratio between zero and one.
const RELATIVE_TOLERANCE: f64 = 1e-4;

fn same(a: f64, b: f64) -> bool {
    if a == b {
        return true;
    }
    if a.is_nan() || b.is_nan() {
        // A NaN appearing or clearing is a change worth seeing; two NaNs in
        // a row are not.
        return a.is_nan() && b.is_nan();
    }
    let scale = a.abs().max(b.abs()).max(1.0);
    (a - b).abs() <= RELATIVE_TOLERANCE * scale
}

pub struct LiveLog {
    interval: std::time::Duration,
    filters: Vec<String>,
    /// `FBW_LOG_ALL_ALL`: log every variable every sample, not only what
    /// moved. For the rare case where the question is "what was the state",
    /// not "what happened".
    everything: bool,
    at: Option<std::time::Instant>,
    /// Last logged value per variable, by name rather than snapshot
    /// position. `Plugin::take_snapshot` rebuilds `names` as `[SIMULATOR
    /// ++ NAMED]` whenever either block grows, so a registration can shift
    /// every `NAMED` index without touching the variable itself -- a
    /// position is not stable across that, but a name is. Keying on the
    /// name also means a generation bump no longer has to mark everything
    /// "unknown": only the names actually new to the registry are absent
    /// from the map, so only they get logged, instead of the whole
    /// ~10,000-variable registry re-dumping in the one frame the
    /// generation bumped.
    last: HashMap<String, f64>,
}

impl LiveLog {
    /// `None` unless `FBW_LOG_ALL` asks for it.
    pub fn from_env() -> Option<Self> {
        let raw = std::env::var("FBW_LOG_ALL").ok()?;
        let raw = raw.trim();
        if raw.is_empty() || raw == "0" || raw.eq_ignore_ascii_case("off") {
            return None;
        }
        // Sub-second is allowed deliberately: a transient that takes an
        // engine out lives for a few frames, and a five-second sample would
        // step straight over it.
        let seconds = raw.parse::<f64>().ok().filter(|s| *s > 0.).unwrap_or(1.0);
        let filters = std::env::var("FBW_LOG_ALL_FILTER")
            .unwrap_or_default()
            .split(',')
            .map(|s| s.trim().to_ascii_uppercase())
            .filter(|s| !s.is_empty())
            .collect::<Vec<_>>();
        let everything = std::env::var("FBW_LOG_ALL_ALL").is_ok_and(|v| v.trim() != "0" && !v.trim().is_empty());
        crate::log(&format!(
            "log-all: every {seconds:.2} s to Log.txt ({}, filter {})",
            if everything { "every variable" } else { "whatever changed" },
            if filters.is_empty() { "none".to_owned() } else { filters.join("+") },
        ));
        Some(Self {
            interval: std::time::Duration::from_secs_f64(seconds),
            filters,
            everything,
            at: None,
            last: HashMap::new(),
        })
    }

    fn wants(&self, name: &str) -> bool {
        self.filters.is_empty() || self.filters.iter().any(|f| name.to_ascii_uppercase().contains(f.as_str()))
    }

    /// Call once a tick, after everything else has written its variables, so
    /// a sample is the state at the end of the tick rather than halfway
    /// through it.
    pub fn tick(&mut self) {
        let now = std::time::Instant::now();
        if self.at.is_some_and(|t| now - t < self.interval) {
            return;
        }
        self.at = Some(now);

        // Everything is collected inside the lock and logged outside it: the
        // panel thread reads this same snapshot, and `crate::log` goes into
        // X-Plane.
        let (time, first, changes) = {
            let Ok(s) = crate::snapshot().lock() else { return };
            let (first, changes) = self.changed(&s.names, &s.values, &s.sources);
            (s.time, first, changes)
        };

        if changes.is_empty() {
            return;
        }
        let what = if first { "all" } else { "changed" };
        let mut line = String::new();
        let mut count = 0usize;
        for change in &changes {
            if line.len() + change.len() + SEPARATOR.len() > LINE_CHARS && !line.is_empty() {
                crate::log(&format!("state {time:.1}s ({what} {count}/{}):{line}", changes.len()));
                line.clear();
            }
            line.push_str(SEPARATOR);
            line.push_str(change);
            count += 1;
        }
        if !line.is_empty() {
            crate::log(&format!("state {time:.1}s ({what} {count}/{}):{line}", changes.len()));
        }
    }

    /// The names that moved since they were last logged, given this tick's
    /// sample. Pure -- no lock, no clock -- so a generation bump (a name
    /// landing at a snapshot position it was never at before) is testable
    /// without the global snapshot mutex. `first` is true only for the very
    /// first sample this `LiveLog` has ever taken (`last` starts empty),
    /// which is also the only time the whole registry is expected to show
    /// up as "changed"; a later generation bump only adds the names that
    /// are new to `last`, not the ones merely shifted to a new position.
    fn changed(&mut self, names: &[String], values: &[f64], sources: &[u8]) -> (bool, Vec<String>) {
        let first = self.last.is_empty();
        let mut changes: Vec<String> = Vec::new();
        for (i, &value) in values.iter().enumerate() {
            let name = names.get(i).map_or("", String::as_str);
            if !self.wants(name) {
                continue;
            }
            let previous = self.last.get(name).copied();
            if !self.everything {
                if let Some(previous) = previous {
                    if same(previous, value) {
                        continue;
                    }
                }
            }
            self.last.insert(name.to_owned(), value);
            // The source matters as much as the value: a variable
            // reading zero because the physics says zero, and one
            // reading zero because nothing in the build ever writes it,
            // are the same number and a different bug.
            let source = match sources.get(i).copied().unwrap_or(0) {
                crate::FROM_XPLANE => "~",
                crate::FROM_SYSTEMS => "",
                _ => "?",
            };
            changes.push(format!("{name}{source}={value:.4}"));
        }
        (first, changes)
    }
}

/// The fixed variable list [`KeySnapshot`] logs, grouped the way the log
/// line prints them. FlyByWire's own name where this port keeps one
/// (`Snapshot::find` tries both with and without the `A32NX_` prefix), this
/// port's own aspect name otherwise (`EXT_PWR_AVAIL:n`, `ENGINE_N1:n`, the
/// `FUEL_TANK_QUANTITY_n` this plugin publishes alongside MSFS's own
/// `FUELSYSTEM TANK QUANTITY:n`, see `fuel.rs`). `fctl_law` is all three
/// PRIMs (`prim.rs`'s `update_prim_fg_shim` picks whichever is the
/// "master" per bit 21 of its own status word, cpp:1776-1786 -- logging
/// just one would hide a law disagreement between them, exactly the kind
/// of thing this exists to catch). About 40 names in total: enough to
/// answer "what was the aeroplane doing" without becoming the full dump
/// this module exists to avoid.
const KEY_GROUPS: &[(&str, &[&str])] = &[
    ("phase", &["FMGC_FLIGHT_PHASE"]),
    (
        "buses",
        &[
            // `deep/electrical/live.rs`'s `BusId`/`bus_tag`: there is no
            // `Dc3`/`Dc4` on this aeroplane, only `Dc1`, `Dc2`, `DcEss` --
            // AC does run 1-4 plus `AcEss`.
            "ELEC_DC_ESS_BUS_IS_POWERED",
            "ELEC_DC_1_BUS_IS_POWERED",
            "ELEC_DC_2_BUS_IS_POWERED",
            "ELEC_AC_ESS_BUS_IS_POWERED",
            "ELEC_AC_1_BUS_IS_POWERED",
            "ELEC_AC_2_BUS_IS_POWERED",
            "ELEC_AC_3_BUS_IS_POWERED",
            "ELEC_AC_4_BUS_IS_POWERED",
        ],
    ),
    ("hyd", &["HYD_GREEN_SYSTEM_1_SECTION_PRESSURE", "HYD_YELLOW_SYSTEM_1_SECTION_PRESSURE"]),
    (
        "fctl_law",
        &[
            "A32NX_PRIM_1_FCTL_LAW_STATUS_WORD",
            "A32NX_PRIM_2_FCTL_LAW_STATUS_WORD",
            "A32NX_PRIM_3_FCTL_LAW_STATUS_WORD",
        ],
    ),
    ("ap_athr", &["AUTOPILOT_ACTIVE", "AUTOTHRUST_STATUS"]),
    (
        "fuel",
        &[
            "FUEL_TANK_QUANTITY_1",
            "FUEL_TANK_QUANTITY_2",
            "FUEL_TANK_QUANTITY_3",
            "FUEL_TANK_QUANTITY_4",
            "FUEL_TANK_QUANTITY_5",
            "FUEL_TANK_QUANTITY_6",
            "FUEL_TANK_QUANTITY_7",
            "FUEL_TANK_QUANTITY_8",
            "FUEL_TANK_QUANTITY_9",
            "FUEL_TANK_QUANTITY_10",
            "FUEL_TANK_QUANTITY_11",
        ],
    ),
    (
        "eng",
        &[
            "ENGINE_N1:1",
            "ENGINE_N1:2",
            "ENGINE_N1:3",
            "ENGINE_N1:4",
            "ENGINE_N2:1",
            "ENGINE_N2:2",
            "ENGINE_N2:3",
            "ENGINE_N2:4",
            "ENGINE_EGT:1",
            "ENGINE_EGT:2",
            "ENGINE_EGT:3",
            "ENGINE_EGT:4",
        ],
    ),
    ("cg", &["AIRFRAME_GW_CG_PERCENT_MAC"]),
    ("stationary", &["IS_STATIONARY"]),
    ("gpu", &["EXT_PWR_AVAIL:1", "OVHD_ELEC_EXT_PWR_1_PB_IS_ON"]),
];

/// A cheap, bounded, fixed-clock snapshot of [`KEY_GROUPS`], independent of
/// `LiveLog`'s "changed" filter -- so a variable that moves once and then
/// holds for a long time (DC ESS staying unpowered, W222) still shows up
/// on the next tick this fires, not only on the edge.
pub struct KeySnapshot {
    interval: std::time::Duration,
    at: Option<std::time::Instant>,
    /// One resolved index per name in [`KEY_GROUPS`], flattened in the same
    /// order `key_lines` walks it; `None` for a name that has never
    /// registered. Rebuilt only when `resolved_generation` no longer
    /// matches the snapshot's own `generation` -- the same reason
    /// `Snapshot::index` itself is generation-gated in `lib.rs`'s
    /// `take_snapshot` -- so a tick that fires costs one `Vec` read per
    /// name, not a lookup, unless the registry just grew.
    indices: Vec<Option<usize>>,
    resolved_generation: u64,
}

impl KeySnapshot {
    /// `None` unless `FBW_LOG_KEY` asks for it.
    pub fn from_env() -> Option<Self> {
        let raw = std::env::var("FBW_LOG_KEY").ok()?;
        let raw = raw.trim();
        if raw.is_empty() || raw == "0" || raw.eq_ignore_ascii_case("off") {
            return None;
        }
        // Ten seconds by default: frequent enough that a tens-of-seconds
        // outage (W222) shows up in several samples, cheap enough to leave
        // running for a whole session.
        let seconds = raw.parse::<f64>().ok().filter(|s| *s > 0.).unwrap_or(10.0);
        let total: usize = KEY_GROUPS.iter().map(|(_, names)| names.len()).sum();
        crate::log(&format!("log-key: {total} key variables every {seconds:.2} s to Log.txt"));
        Some(Self {
            interval: std::time::Duration::from_secs_f64(seconds),
            at: None,
            indices: Vec::new(),
            resolved_generation: u64::MAX,
        })
    }

    /// Call once a tick, after everything else has written its variables --
    /// same placement as `LiveLog::tick`, so a sample is the state at the
    /// end of the tick.
    pub fn tick(&mut self) {
        let now = std::time::Instant::now();
        if self.at.is_some_and(|t| now - t < self.interval) {
            return;
        }
        self.at = Some(now);

        // Same discipline as `LiveLog::tick`: collect every line inside the
        // lock, log them after releasing it, so this never holds the
        // snapshot mutex -- which the panel thread also reads -- across
        // X-Plane's own logging call.
        let (time, lines) = {
            let Ok(s) = crate::snapshot().lock() else { return };
            if s.generation != self.resolved_generation {
                // The registry grew since these were last resolved
                // (`Plugin::take_snapshot` renumbers on any growth) --
                // re-resolve the ~40 names here, not the whole registry:
                // `Snapshot::find` is one hashmap lookup per name.
                self.indices = resolve_indices(|name| s.find(name));
                self.resolved_generation = s.generation;
            }
            (s.time, key_lines(&self.indices, &s.values))
        };
        for line in lines {
            crate::log(&format!("key {time:.1}s {line}"));
        }
    }
}

/// [`KEY_GROUPS`]'s names, flattened in order, each resolved through
/// `find` (a `Snapshot::find`-shaped lookup). A free function so it is
/// testable without a real `Snapshot`.
fn resolve_indices(mut find: impl FnMut(&str) -> Option<usize>) -> Vec<Option<usize>> {
    KEY_GROUPS.iter().flat_map(|(_, names)| names.iter()).map(|name| find(name)).collect()
}

/// The only three names [`KEY_GROUPS`] tracks that are genuinely packed
/// ARINC 429 words (`prim.rs`'s `update_prim_fg_shim`/W81's own SSM-gated
/// decode pattern): `A32NX_PRIM_{1,2,3}_FCTL_LAW_STATUS_WORD`. Decoded by
/// name, not by guessing from a value's bit pattern -- the crate-wide
/// `arinc()` heuristic W63 originally added here was dropped (per
/// coordinator/W120 review: "mis-decodes nearly every ordinary value"), so
/// this checks the one thing it is actually true for instead of every
/// value this snapshot logs.
fn is_fctl_law_status_word(name: &str) -> bool {
    matches!(name, "A32NX_PRIM_1_FCTL_LAW_STATUS_WORD" | "A32NX_PRIM_2_FCTL_LAW_STATUS_WORD" | "A32NX_PRIM_3_FCTL_LAW_STATUS_WORD")
}

/// One `group:name=value; ...` line per entry in [`KEY_GROUPS`], given
/// already-resolved indices into a snapshot's `values` (see
/// `resolve_indices`). A missing index -- the variable has never
/// registered -- logs `=?` rather than being silently skipped, since on a
/// fixed clock a gap has to be visible as a gap. [`is_fctl_law_status_word`]
/// names decode as an SSM-gated ARINC word (the same shape `prim.rs`/W81
/// use), everything else logs its plain value -- unlike `LiveLog::changed`
/// (which dropped its own generic decode with W63), this list is fixed and
/// small enough to name the packed ones explicitly rather than guess.
/// Pure -- no lock, no clock -- so this is testable without the global
/// snapshot mutex.
fn key_lines(indices: &[Option<usize>], values: &[f64]) -> Vec<String> {
    let mut lines = Vec::with_capacity(KEY_GROUPS.len());
    let mut i = 0;
    for (group, names) in KEY_GROUPS {
        let mut line = String::new();
        for name in *names {
            let value = indices[i].and_then(|idx| values.get(idx).copied());
            i += 1;
            if !line.is_empty() {
                line.push_str(SEPARATOR);
            }
            line.push_str(&match value {
                None => format!("{name}=?"),
                Some(v) if is_fctl_law_status_word(name) => {
                    let word = crate::prim::from_simvar(v);
                    format!("{name}=SSM:{} {:.4}", word.SSM, word.Data)
                }
                Some(v) => format!("{name}={v:.4}"),
            });
        }
        lines.push(format!("{group}:{line}"));
    }
    lines
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_last_bit_wobble_is_not_a_change_but_a_real_move_is() {
        // Spool speeds and pressures differ in their last bit every tick;
        // treating those as changes would log the whole aeroplane forever.
        assert!(same(5142.0, 5142.3));
        assert!(!same(5142.0, 5100.0));
        // The tolerance is relative, so it means the same for a ratio.
        assert!(same(0.5, 0.500_01));
        assert!(!same(0.5, 0.6));
        // ... and absolute near zero, or nothing near zero would ever
        // register as having moved off it.
        assert!(!same(0.0, 0.5));
        assert!(same(0.0, 0.000_01));
    }

    #[test]
    fn a_nan_appearing_is_a_change_and_two_in_a_row_are_not() {
        assert!(!same(f64::NAN, 1.0));
        assert!(!same(1.0, f64::NAN));
        assert!(same(f64::NAN, f64::NAN));
    }

    fn with_filters(filters: &[&str]) -> LiveLog {
        LiveLog {
            interval: std::time::Duration::from_secs(1),
            filters: filters.iter().map(|f| f.to_ascii_uppercase()).collect(),
            everything: false,
            at: None,
            last: HashMap::new(),
        }
    }

    #[test]
    fn no_registered_name_could_collide_with_the_separator() {
        // The separator has to be something a name cannot contain, or a
        // line cannot be split back into pairs -- which is exactly what a
        // space turned out not to be, since MSFS's own names have spaces.
        for name in [
            "FUELSYSTEM TANK QUANTITY:3",
            "PLANE HEADING DEGREES TRUE",
            "A32NX_HYD_GREEN_SYSTEM_1_SECTION_PRESSURE",
            "ENGINE_N3:1",
        ] {
            assert!(!name.contains(SEPARATOR.trim()), "{name} contains the separator");
        }
    }

    #[test]
    fn a_generation_bump_only_logs_the_names_new_to_the_registry() {
        let mut l = with_filters(&[]);
        let names = |v: &[&str]| v.iter().map(|s| s.to_string()).collect::<Vec<_>>();
        let sources = |n: usize| vec![crate::FROM_SYSTEMS; n];

        // First sample: the whole (tiny, here) registry is new.
        let (first, changes) = l.changed(&names(&["A", "B", "C"]), &[1.0, 2.0, 3.0], &sources(3));
        assert!(first);
        assert_eq!(changes.len(), 3);

        // Nothing moved: no repeat, and it is no longer the first sample.
        let (first, changes) = l.changed(&names(&["A", "B", "C"]), &[1.0, 2.0, 3.0], &sources(3));
        assert!(!first);
        assert!(changes.is_empty());

        // A registration inserts D in the middle, renumbering B and C --
        // what `Plugin::take_snapshot` does when the `SIMULATOR`-kind block
        // grows and shifts the `NAMED` block that follows it in the flat
        // `names` array. Only D, the name new to the registry, gets
        // logged: the old bug (indexing `last` by position and resetting
        // it to all-NaN on any generation bump) would have re-logged all
        // four here.
        let (first, changes) = l.changed(&names(&["A", "D", "B", "C"]), &[1.0, 4.0, 2.0, 3.0], &sources(4));
        assert!(!first);
        assert_eq!(changes, vec!["D=4.0000".to_string()]);
    }

    #[test]
    fn about_forty_key_variables_are_tracked() {
        let total: usize = KEY_GROUPS.iter().map(|(_, names)| names.len()).sum();
        assert!((30..=50).contains(&total), "expected roughly 40 key variables, found {total}");
    }

    #[test]
    fn resolve_indices_looks_up_every_key_name_once_in_order() {
        let flat: Vec<&str> = KEY_GROUPS.iter().flat_map(|(_, names)| names.iter().copied()).collect();
        let mut calls: Vec<String> = Vec::new();
        let indices = resolve_indices(|name| {
            calls.push(name.to_owned());
            None
        });
        assert_eq!(indices, vec![None; flat.len()]);
        assert_eq!(calls, flat.iter().map(|s| s.to_string()).collect::<Vec<_>>());
    }

    #[test]
    fn key_lines_report_a_found_value_a_missing_one_and_an_arinc_word() {
        let flat: Vec<&str> = KEY_GROUPS.iter().flat_map(|(_, names)| names.iter().copied()).collect();
        let total = flat.len();
        let mut indices = vec![None; total];
        let mut values = Vec::new();

        // The first name in the flattened list resolves to a plain value.
        indices[0] = Some(values.len());
        values.push(2.0);

        // The second name is never registered -- `?`, not a stale value or
        // a panic. (indices[1] stays `None`.)

        // One of the three `fctl_law` names resolves to a packed ARINC
        // word, decoded the same SSM-gated way `prim.rs`/W81 decode one --
        // `key_lines` only decodes these three names by name, not any
        // value that merely looks packed (the crate-wide `arinc()`
        // heuristic W63 dropped).
        let fctl_index = flat.iter().position(|&n| n == "A32NX_PRIM_1_FCTL_LAW_STATUS_WORD").expect("fctl_law name present");
        indices[fctl_index] = Some(values.len());
        values.push(crate::prim::to_simvar(crate::fbw_types::BaseArinc429 { SSM: crate::prim::SSM_NO, Data: 3.25 }));

        let lines = key_lines(&indices, &values);
        assert_eq!(lines.len(), KEY_GROUPS.len(), "one line per group, even ones with an unresolved name");

        let joined = lines.join(" | ");
        assert!(joined.contains(&format!("{}=2.0000", flat[0])), "{joined}");
        assert!(joined.contains(&format!("{}=?", flat[1])), "{joined}");
        assert!(
            joined.contains(&format!("{}=SSM:{} 3.2500", flat[fctl_index], crate::prim::SSM_NO)),
            "{joined}"
        );
    }

    #[test]
    fn a_filter_matches_anywhere_in_the_name_without_regard_to_case() {
        let l = with_filters(&["hyd"]);
        assert!(l.wants("A32NX_HYD_GREEN_PRESSURE"));
        assert!(l.wants("deep_hyd_thermal"));
        assert!(!l.wants("ENGINE_N3:1"));
    }

    #[test]
    fn no_filter_takes_every_system() {
        let l = with_filters(&[]);
        assert!(l.wants("anything at all"));
        assert!(l.wants("ENGINE_N3:1"));
    }
}
