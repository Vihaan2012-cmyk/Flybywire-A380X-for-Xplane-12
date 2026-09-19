//! Per-view slot state: taking a consistent snapshot of the plugin's
//! `SlotTable` under its seqlock (rule 1), and the read-your-writes overlay
//! that keeps a just-written value visible until the plugin has republished
//! twice (rule 3). Both are plain data structures with no CEF or shared-memory
//! dependency, so they are unit tested directly (docs/briefs/xphfbw-js-bridge.md).

use std::collections::HashMap;
#[cfg(test)]
use std::cell::RefCell;

/// A copy of every currently-registered slot's value, taken under the
/// plugin's seqlock, plus the frame it was published at and how many slots
/// were resolved as of that frame (rule 2: a slot at or past `resolved`
/// reads as 0, the same as the QuickJS worker's first read).
pub struct Snapshot {
    pub frame: u64,
    pub resolved: u32,
    pub values: Vec<f64>,
}

impl Snapshot {
    /// The slot's raw value from this snapshot (before the overlay), 0 for
    /// an unresolved or out-of-range slot.
    pub fn raw(&self, slot: u32) -> f64 {
        if slot >= self.resolved {
            return 0.0;
        }
        self.values.get(slot as usize).copied().unwrap_or(0.0)
    }
}

/// Retries a copy under a seqlock: `frame()` must be even and unchanged
/// across the copy, otherwise the writer raced us and we copy again
/// (rule 1). `copy` fills in the snapshotted values on success.
///
/// Generic over closures so this can be unit tested without any shared
/// memory: a fake `frame` sequence can simulate a writer that is mid-publish
/// (odd) or that finishes a publish while we are copying (changed).
pub fn seqlock_read<T>(mut frame: impl FnMut() -> u64, mut copy: impl FnMut() -> T) -> (u64, T) {
    loop {
        let before = frame();
        if before & 1 == 1 {
            continue;
        }
        let value = copy();
        let after = frame();
        if after == before {
            return (before, value);
        }
    }
}

/// A view's pending writes: `setVar` updates are visible to that view's own
/// `getVar` at once, and keep overriding the published value until the
/// snapshot is at least two full publishes (four frame counts, since the
/// plugin's seqlock bumps `frame` by 2 per publish) past the write (rule 3).
#[derive(Default)]
pub struct Overlay {
    writes: HashMap<u32, (f64, u64)>,
}

impl Overlay {
    pub fn new() -> Self {
        Self::default()
    }

    /// Record a write this view just made, effective as of `frame_at_write`
    /// (the latest frame this view had observed when it wrote).
    pub fn set(&mut self, slot: u32, value: f64, frame_at_write: u64) {
        self.writes.insert(slot, (value, frame_at_write));
    }

    /// The value `getVar` should return for `slot`: the overlaid write while
    /// it is still fresher than `snapshot_frame`, otherwise `published` (and
    /// the overlay entry is dropped, since the plugin has caught up).
    pub fn resolve(&mut self, slot: u32, published: f64, snapshot_frame: u64) -> f64 {
        match self.writes.get(&slot) {
            Some(&(value, frame_at_write)) if snapshot_frame < frame_at_write + 4 => value,
            Some(_) => {
                self.writes.remove(&slot);
                published
            }
            None => published,
        }
    }

    /// Drop overlay entries the plugin has already republished past, so the
    /// map does not grow without bound across a long-running view. Safe to
    /// call any time; `resolve` alone is already correct without it.
    pub fn prune(&mut self, snapshot_frame: u64) {
        self.writes.retain(|_, &mut (_, frame_at_write)| snapshot_frame < frame_at_write + 4);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;

    #[test]
    fn seqlock_read_succeeds_immediately_on_a_stable_even_frame() {
        let calls = Cell::new(0);
        let (frame, value) = seqlock_read(|| 42, || {
            calls.set(calls.get() + 1);
            "copied"
        });
        assert_eq!(frame, 42);
        assert_eq!(value, "copied");
        assert_eq!(calls.get(), 1, "no retry needed");
    }

    #[test]
    fn seqlock_read_retries_while_the_writer_is_mid_publish() {
        // Odd frame (writer mid-publish) on the first read, then a stable
        // even frame once it finishes.
        let reads = RefCell::new(vec![7u64, 8, 8].into_iter());
        let (frame, value) = seqlock_read(|| reads.borrow_mut().next().unwrap(), || "ok");
        assert_eq!(frame, 8);
        assert_eq!(value, "ok");
    }

    #[test]
    fn seqlock_read_retries_when_the_frame_changes_during_the_copy() {
        // before=10, then the writer republishes (12) while we copy, so we
        // must retry; the second attempt is stable at 12.
        let reads = RefCell::new(vec![10u64, 12, 12, 12].into_iter());
        let (frame, value) = seqlock_read(|| reads.borrow_mut().next().unwrap(), || "ok");
        assert_eq!(frame, 12);
        assert_eq!(value, "ok");
    }

    #[test]
    fn snapshot_raw_is_zero_past_resolved_even_if_the_backing_array_has_a_value() {
        let snap = Snapshot { frame: 4, resolved: 2, values: vec![1.0, 2.0, 999.0] };
        assert_eq!(snap.raw(0), 1.0);
        assert_eq!(snap.raw(1), 2.0);
        assert_eq!(snap.raw(2), 0.0, "slot >= resolved reads 0, rule 2");
        assert_eq!(snap.raw(50), 0.0, "out of range reads 0 too");
    }

    #[test]
    fn overlay_returns_the_written_value_until_two_publishes_later() {
        let mut overlay = Overlay::new();
        overlay.set(3, 1.5, 100);
        // Same publish, and the very next one: still overlaid.
        assert_eq!(overlay.resolve(3, 0.0, 100), 1.5);
        assert_eq!(overlay.resolve(3, 0.0, 102), 1.5);
        // Two full publishes later (frame >= frame_at_write + 4): the
        // published value wins and the overlay entry is gone.
        assert_eq!(overlay.resolve(3, 9.0, 104), 9.0);
        assert_eq!(overlay.resolve(3, 9.5, 106), 9.5, "overlay entry was dropped");
    }

    #[test]
    fn overlay_is_per_slot_and_a_later_write_replaces_the_frame_reference() {
        let mut overlay = Overlay::new();
        overlay.set(1, 10.0, 50);
        overlay.set(2, 20.0, 50);
        overlay.set(1, 11.0, 60); // A second write to slot 1 resets its clock.
        assert_eq!(overlay.resolve(1, 0.0, 62), 11.0, "still within 4 of the second write");
        assert_eq!(overlay.resolve(2, 0.0, 62), 0.0, "slot 2's window (50+4=54) has passed");
    }

    #[test]
    fn overlay_untouched_slots_pass_the_published_value_through() {
        let mut overlay = Overlay::new();
        assert_eq!(overlay.resolve(9, 3.25, 1000), 3.25);
    }

    #[test]
    fn overlay_prune_drops_only_entries_the_plugin_has_republished_past() {
        let mut overlay = Overlay::new();
        overlay.set(1, 1.0, 10);
        overlay.set(2, 2.0, 100);
        overlay.prune(20); // 20 >= 10+4, but 20 < 100+4
        assert_eq!(overlay.resolve(1, 42.0, 20), 42.0, "slot 1 was pruned and reads through");
        assert_eq!(overlay.resolve(2, 42.0, 20), 2.0, "slot 2 is still within its window");
    }
}
