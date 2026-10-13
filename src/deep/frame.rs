//! The three values every deep area exchanges with the frame loop:
//! [`PublishedFrame`] (what the areas published last frame), [`Faults`]
//! (what is armed) and [`DerivedFailure`] (an area's verdict on one of
//! FlyByWire's own components).
//!
//! They live in their own file, re-exported from `deep::live`, so the
//! `deep_electrical` crate (`crates/deep_electrical`) can compile this same
//! file for MSFS: one definition for both simulators.

use std::collections::{BTreeMap, HashMap};

/// What every area published on the previous frame, by variable name.
///
/// This is how one area reads another's output. The areas are deliberately
/// independent -- none may call into another, so that each stays separately
/// testable -- but the aircraft is not: a pneumatic duct's overheat loop
/// watches the temperature of the bay it runs through, and that bay is the
/// thermal area's to compute. Passing the previous frame's published values
/// back in keeps the areas decoupled while letting the physics join up.
///
/// A name nobody published reads as `None`, never as zero: an area must be
/// able to tell "the bay is at 0 C" from "nothing models that bay".
///
/// Stored in publish order (names and values side by side) with a name
/// index beside it. The layer publishes the same ~4,900 names in the same
/// order every frame, so [`Self::begin`]/[`Self::set`]/[`Self::finish`]
/// update each value in place with one string comparison and rebuild the
/// index only on a frame whose order changed. A sorted map of freshly
/// allocated names, rebuilt every frame, was 0.9 ms of the layer's 2 ms.
/// A name published twice in one frame reads as its last value, as it did.
#[derive(Clone, Debug, Default)]
pub struct PublishedFrame {
    names: Vec<String>,
    values: Vec<f64>,
    index: HashMap<String, usize>,
    cursor: usize,
    reordered: bool,
}

impl PublishedFrame {
    /// What another area published for `name` last frame, if anything did.
    pub fn get(&self, name: &str) -> Option<f64> {
        self.index.get(name).map(|&i| self.values[i])
    }

    /// `get`, with a caller-chosen stand-in for "nobody models this yet".
    /// The fallback belongs to the caller because only it knows what a
    /// physically sane substitute is -- ambient for a duct pressure, say,
    /// never zero.
    pub fn get_or(&self, name: &str, fallback: f64) -> f64 {
        self.get(name).unwrap_or(fallback)
    }

    pub fn is_empty(&self) -> bool {
        self.index.is_empty()
    }

    /// How many distinct names this frame holds.
    pub fn len(&self) -> usize {
        self.index.len()
    }

    /// Every distinct name and its value, in no particular order.
    pub fn iter(&self) -> impl Iterator<Item = (&str, f64)> + '_ {
        self.index.iter().map(|(name, &i)| (name.as_str(), self.values[i]))
    }

    /// Set one value outside a [`Self::begin`]..[`Self::finish`] frame (a
    /// test building the frame another area would have published).
    pub fn insert(&mut self, name: impl Into<String>, value: f64) {
        let name = name.into();
        if let Some(&i) = self.index.get(&name) {
            self.values[i] = value;
        } else {
            self.index.insert(name.clone(), self.names.len());
            self.names.push(name);
            self.values.push(value);
        }
    }

    /// Start writing a new frame over this one, in the order the last one
    /// was written.
    pub fn begin(&mut self) {
        self.cursor = 0;
        self.reordered = false;
    }

    /// The next published value of the frame [`Self::begin`] started.
    pub fn set(&mut self, name: &str, value: f64) {
        let i = self.cursor;
        if i < self.names.len() && self.names[i] == name {
            self.values[i] = value;
        } else {
            self.names.truncate(i);
            self.values.truncate(i);
            self.names.push(name.to_owned());
            self.values.push(value);
            self.reordered = true;
        }
        self.cursor += 1;
    }

    /// End the frame: drop whatever the last frame published and this one
    /// did not, and re-index if the order changed.
    pub fn finish(&mut self) {
        if self.cursor < self.names.len() {
            self.names.truncate(self.cursor);
            self.values.truncate(self.cursor);
            self.reordered = true;
        }
        if self.reordered {
            self.index.clear();
            for (i, name) in self.names.iter().enumerate() {
                self.index.insert(name.clone(), i);
            }
        }
    }
}

impl From<BTreeMap<String, f64>> for PublishedFrame {
    fn from(map: BTreeMap<String, f64>) -> Self {
        map.into_iter().collect()
    }
}

impl FromIterator<(String, f64)> for PublishedFrame {
    fn from_iter<I: IntoIterator<Item = (String, f64)>>(pairs: I) -> Self {
        let mut frame = Self::default();
        for (name, value) in pairs {
            frame.insert(name, value);
        }
        frame
    }
}

/// How badly each failure is armed, by `deep::api` failure id.
///
/// Absent means healthy. Every magnitude is clamped to 0..1 on the way in,
/// so an area can use it as a fraction without checking.
#[derive(Clone, Debug, Default)]
pub struct Faults(BTreeMap<u64, f64>);

impl Faults {
    pub fn from_pairs(pairs: impl IntoIterator<Item = (u64, f64)>) -> Self {
        Self(pairs.into_iter().map(|(id, m)| (id, m.clamp(0.0, 1.0))).collect())
    }

    /// This failure's magnitude, 0 if it is not armed at all.
    pub fn get(&self, id: u64) -> f64 {
        self.0.get(&id).copied().unwrap_or(0.0)
    }

    /// Whether anything at all is armed -- areas with an expensive
    /// healthy-case shortcut can check this first.
    pub fn any(&self) -> bool {
        self.0.values().any(|&m| m > 0.0)
    }
}

/// One of FlyByWire's own failures, as a deep area's verdict on a
/// component the two models share.
///
/// This is level 2 of `docs/deep/authority.md`: the deep area has
/// concluded that a physical component `a380_systems` also models is no
/// longer working, and says so in the only vocabulary `a380_systems`
/// accepts -- its own failure ids, the ones `crate::failures`'
/// `a380_failures()`/`extra::extra_failures()` catalogue. The plugin feeds
/// these into `crate::failures` beside the ones the crew armed, so a
/// derived failure and an armed one reach `a380_systems` by the same path.
///
/// Every field but `magnitude` is fixed at build time, because a derived
/// failure the crew cannot clear is only tolerable if the crew can see
/// *why*: [`deep_component`](Self::deep_component) and
/// [`reason`](Self::reason) are what the Study page shows next to it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DerivedFailure {
    /// The id in `crate::failures`' catalogue -- FlyByWire's own
    /// (`a380_failures()`) or this port's extra one, both of which
    /// `failures::set_magnitude` accepts.
    pub fbw_id: u64,
    /// How far gone, 0 healthy .. 1 fully failed. An area emits every
    /// coupling it owns every frame, healthy ones at `0.0`; [`Deep::tick`]
    /// keeps only those above zero. Ids whose FlyByWire side is binary
    /// (most of them) get `1.0` or `0.0` and nothing in between -- the
    /// granularity limit `authority.md` names.
    pub magnitude: f64,
    /// The deep component that concluded it, by its registry id, e.g.
    /// `"24_elec.vfg-1"`.
    pub deep_component: &'static str,
    /// Why, in one phrase a pilot can read: "own overload element tripped
    /// it off line", "feeder breaker open".
    pub reason: &'static str,
}
