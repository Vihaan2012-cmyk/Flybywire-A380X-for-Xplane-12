//! The variable bridge: what the deep layer reads from a host simulator, and
//! what it writes back.
//!
//! Written against a trait rather than against either simulator, for two
//! reasons. It can be tested without a sim at all -- the tests below drive it
//! through a plain map -- and the same mapping serves X-Plane's datarefs and
//! MSFS's LVars without being written twice.
//!
//! # What it costs, measured
//!
//! In MSFS (`msfs/lvar-bench`, run in the sim): **11-12 ns** to write a
//! variable through a handle held from construction, **14-16 ns** to read
//! one, and dead flat from 50 variables to 4,228 -- linearity 1.01x. Three
//! consequences, and the design follows all three:
//!
//! * **Resolve every handle once.** Looking a name up per operation costs
//!   another 58 ns, six times the write itself.
//! * **Do not bother detecting change.** Writing a value that did not change
//!   costs the same as writing one that did (measured saving: 0.7%). Skipping
//!   is ours to do in Rust if we ever want it; the sim gives nothing for it.
//! * **Publishing everything is affordable.** All 4,228 published values cost
//!   0.048 ms a frame. For scale, the deep layer already spends 2.3 ms
//!   thinking. The boundary is not the constraint anyone expected it to be.
//!
//! # What crosses, and what does not
//!
//! Most of those 4,228 are the *internal* cross-area bus -- how one area
//! reads another a frame behind, which `Deep` keeps in memory. They only
//! need to become host variables where something outside reads them: the
//! authority couplings, and whatever the EFB shows. This bridge publishes
//! whatever it is given a handle for, so the caller decides; `publish_all`
//! is the simple choice and the measurement above says it is affordable.

use std::collections::BTreeMap;

/// A resolved variable, opaque to this module. The host decides what it is:
/// an index into its own table, an X-Plane `VariableIdentifier`, an MSFS
/// `NamedVariable`'s slot.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Handle(pub usize);

/// The host simulator's variable table.
///
/// `resolve` is called once per name when the bridge is built and never
/// again; `set` and `get` run every frame. An implementation should make
/// `resolve` do the expensive work (the string lookup, the registration) so
/// the per-frame calls stay at the 11 ns the benchmark measured.
pub trait VarStore {
    /// The handle for `name`, registering it if the host has not seen it.
    fn resolve(&mut self, name: &str) -> Handle;
    /// Write, every frame, without checking whether the value changed.
    fn set(&mut self, handle: Handle, value: f64);
    /// Read. Returns the host's value, or 0.0 where it has none.
    fn get(&mut self, handle: Handle) -> f64;
}

/// Every published name, resolved once, in the order `Deep` publishes them.
///
/// Positional rather than keyed: `Deep::publish` yields values in a stable
/// order, so a `Vec` indexed by position avoids a hash lookup per variable
/// per frame. The names are kept alongside so a mismatch can be caught
/// rather than silently writing a value to the wrong variable.
pub struct Published {
    names: Vec<String>,
    handles: Vec<Handle>,
}

impl Published {
    /// Resolve every name the deep layer publishes.
    pub fn resolve<S: VarStore>(store: &mut S, names: &[String]) -> Self {
        let handles = names.iter().map(|n| store.resolve(n)).collect();
        Self { names: names.to_vec(), handles }
    }

    pub fn len(&self) -> usize {
        self.handles.len()
    }

    pub fn is_empty(&self) -> bool {
        self.handles.is_empty()
    }

    /// Write one frame's published values.
    ///
    /// `frame` is what the areas just published, keyed by name. Anything the
    /// bridge does not hold a handle for is ignored rather than resolved on
    /// the spot: resolving inside the frame loop is the one thing the
    /// benchmark says not to do, and a name appearing mid-session means the
    /// area set changed, which is a bug worth noticing rather than papering
    /// over.
    pub fn write<S: VarStore>(&self, store: &mut S, frame: &BTreeMap<String, f64>) -> usize {
        let mut written = 0;
        for (name, handle) in self.names.iter().zip(&self.handles) {
            if let Some(value) = frame.get(name) {
                store.set(*handle, *value);
                written += 1;
            }
        }
        written
    }

    /// Names the host has a handle for but this frame did not publish.
    ///
    /// Empty in normal running. A non-empty answer means an area stopped
    /// publishing something it used to, which leaves a stale value sitting in
    /// the host where a reader will believe it.
    pub fn missing(&self, frame: &BTreeMap<String, f64>) -> Vec<&str> {
        self.names.iter().filter(|n| !frame.contains_key(*n)).map(String::as_str).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A host that is just a map, so the bridge can be tested with no
    /// simulator of any kind.
    #[derive(Default)]
    struct MapStore {
        names: Vec<String>,
        values: Vec<f64>,
        resolves: usize,
        sets: usize,
    }

    impl VarStore for MapStore {
        fn resolve(&mut self, name: &str) -> Handle {
            self.resolves += 1;
            if let Some(i) = self.names.iter().position(|n| n == name) {
                return Handle(i);
            }
            self.names.push(name.to_owned());
            self.values.push(0.0);
            Handle(self.names.len() - 1)
        }
        fn set(&mut self, handle: Handle, value: f64) {
            self.sets += 1;
            self.values[handle.0] = value;
        }
        fn get(&mut self, handle: Handle) -> f64 {
            self.values[handle.0]
        }
    }

    impl MapStore {
        fn value(&self, name: &str) -> Option<f64> {
            self.names.iter().position(|n| n == name).map(|i| self.values[i])
        }
    }

    fn frame(pairs: &[(&str, f64)]) -> BTreeMap<String, f64> {
        pairs.iter().map(|(n, v)| ((*n).to_owned(), *v)).collect()
    }

    #[test]
    fn a_name_is_resolved_once_however_many_frames_run() {
        // The whole point of holding handles: the 58 ns name lookup happens
        // at construction, not 60 times a second for the rest of the flight.
        let mut store = MapStore::default();
        let names: Vec<String> = ["A", "B", "C"].iter().map(|s| (*s).to_owned()).collect();
        let published = Published::resolve(&mut store, &names);
        assert_eq!(store.resolves, 3);

        let f = frame(&[("A", 1.0), ("B", 2.0), ("C", 3.0)]);
        for _ in 0..100 {
            published.write(&mut store, &f);
        }
        assert_eq!(store.resolves, 3, "resolving in the frame loop is the one thing not to do");
        assert_eq!(store.sets, 300);
    }

    #[test]
    fn every_published_value_reaches_the_variable_of_its_own_name() {
        let mut store = MapStore::default();
        let names: Vec<String> =
            ["ELEC_AC_1_BUS_POTENTIAL", "HYD_GREEN_PRESSURE_PA", "DEEP_ENG_1_TGT_SENSED_C"]
                .iter()
                .map(|s| (*s).to_owned())
                .collect();
        let published = Published::resolve(&mut store, &names);

        let written = published.write(
            &mut store,
            &frame(&[
                ("ELEC_AC_1_BUS_POTENTIAL", 115.0),
                ("HYD_GREEN_PRESSURE_PA", 34_474_000.0),
                ("DEEP_ENG_1_TGT_SENSED_C", 612.0),
            ]),
        );

        assert_eq!(written, 3);
        assert_eq!(store.value("ELEC_AC_1_BUS_POTENTIAL"), Some(115.0));
        assert_eq!(store.value("HYD_GREEN_PRESSURE_PA"), Some(34_474_000.0));
        assert_eq!(store.value("DEEP_ENG_1_TGT_SENSED_C"), Some(612.0));
    }

    #[test]
    fn an_unchanged_value_is_written_again_rather_than_skipped() {
        // Deliberate. Measured in MSFS: writing an unchanged value saves 0.7%
        // -- nothing. Skipping would cost a comparison and a branch per
        // variable to save less than the branch costs.
        let mut store = MapStore::default();
        let names = vec!["STEADY".to_owned()];
        let published = Published::resolve(&mut store, &names);
        let f = frame(&[("STEADY", 42.0)]);

        published.write(&mut store, &f);
        published.write(&mut store, &f);
        published.write(&mut store, &f);

        assert_eq!(store.sets, 3);
    }

    #[test]
    fn a_name_that_stops_being_published_is_reported_not_left_stale() {
        // A stale value is worse than a missing one: a reader cannot tell it
        // is old, and will act on it.
        let mut store = MapStore::default();
        let names: Vec<String> = ["A", "B"].iter().map(|s| (*s).to_owned()).collect();
        let published = Published::resolve(&mut store, &names);

        published.write(&mut store, &frame(&[("A", 1.0), ("B", 2.0)]));
        let only_a = frame(&[("A", 9.0)]);
        let written = published.write(&mut store, &only_a);

        assert_eq!(written, 1);
        assert_eq!(published.missing(&only_a), vec!["B"]);
        assert_eq!(store.value("B"), Some(2.0), "the host keeps the last write; that is why it is reported");
    }

    #[test]
    fn a_value_the_bridge_has_no_handle_for_is_ignored_not_resolved_mid_frame() {
        let mut store = MapStore::default();
        let names = vec!["KNOWN".to_owned()];
        let published = Published::resolve(&mut store, &names);
        assert_eq!(store.resolves, 1);

        let written = published.write(&mut store, &frame(&[("KNOWN", 1.0), ("SURPRISE", 2.0)]));

        assert_eq!(written, 1);
        assert_eq!(store.resolves, 1, "a new name mid-session is a bug to notice, not to absorb");
        assert_eq!(store.value("SURPRISE"), None);
    }

    #[test]
    fn the_real_published_set_resolves_and_writes_end_to_end() {
        // Against the actual deep layer rather than invented names: every
        // name it publishes gets a handle, and one tick's worth of values
        // lands without a single mid-frame resolve.
        let deep = crate::deep::live::all_areas();
        let names = deep.published_names();
        assert!(names.len() > 1_000, "the deep layer publishes {} names", names.len());

        let mut store = MapStore::default();
        let published = Published::resolve(&mut store, &names);
        assert_eq!(published.len(), names.len());
        let after_resolve = store.resolves;

        let f: BTreeMap<String, f64> = names.iter().map(|n| (n.clone(), 1.0)).collect();
        let written = published.write(&mut store, &f);

        assert_eq!(written, names.len());
        assert_eq!(store.resolves, after_resolve);
        assert!(published.missing(&f).is_empty());
    }
}
