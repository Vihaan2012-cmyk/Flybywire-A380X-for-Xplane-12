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

/// Roughly one terminal-width-independent line. Long enough that a busy
/// sample is a few lines rather than hundreds, short enough to stay
/// greppable.
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
    /// Last logged value per variable, in snapshot order.
    last: Vec<f64>,
    /// The snapshot's `generation` when `last` was built. A registration
    /// renumbers the snapshot, so anything indexed by position has to be
    /// rebuilt rather than compared across the change.
    generation: u64,
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
            last: Vec::new(),
            generation: u64::MAX,
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
            let first = s.generation != self.generation;
            if first {
                self.generation = s.generation;
                self.last = vec![f64::NAN; s.values.len()];
            } else if self.last.len() != s.values.len() {
                // Same generation, different length: not expected, but
                // indexing past the end would panic, so treat it as new.
                self.last = vec![f64::NAN; s.values.len()];
            }
            let mut changes: Vec<String> = Vec::new();
            for (i, &value) in s.values.iter().enumerate() {
                let name = s.names.get(i).map_or("", String::as_str);
                if !self.wants(name) {
                    continue;
                }
                if !self.everything && same(self.last[i], value) {
                    continue;
                }
                self.last[i] = value;
                // The source matters as much as the value: a variable
                // reading zero because the physics says zero, and one
                // reading zero because nothing in the build ever writes it,
                // are the same number and a different bug.
                let source = match s.sources.get(i).copied().unwrap_or(0) {
                    crate::FROM_XPLANE => "~",
                    crate::FROM_SYSTEMS => "",
                    _ => "?",
                };
                changes.push(format!("{name}{source}={value:.4}"));
            }
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
            last: Vec::new(),
            generation: u64::MAX,
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
