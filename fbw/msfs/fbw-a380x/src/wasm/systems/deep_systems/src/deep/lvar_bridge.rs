use std::collections::BTreeMap;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Handle(pub usize);

pub trait VarStore {
    fn resolve(&mut self, name: &str) -> Handle;
    fn set(&mut self, handle: Handle, value: f64);
    fn get(&mut self, handle: Handle) -> f64;
}

pub struct Published {
    names: Vec<String>,
    handles: Vec<Handle>,
}

impl Published {
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

    pub fn missing(&self, frame: &BTreeMap<String, f64>) -> Vec<&str> {
        self.names.iter().filter(|n| !frame.contains_key(*n)).map(String::as_str).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
