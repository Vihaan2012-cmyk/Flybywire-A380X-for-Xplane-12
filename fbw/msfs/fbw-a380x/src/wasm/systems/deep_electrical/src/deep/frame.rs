use std::collections::{BTreeMap, HashMap};

#[derive(Clone, Debug, Default)]
pub struct PublishedFrame {
    names: Vec<String>,
    values: Vec<f64>,
    index: HashMap<String, usize>,
    cursor: usize,
    reordered: bool,
}

impl PublishedFrame {
    pub fn get(&self, name: &str) -> Option<f64> {
        self.index.get(name).map(|&i| self.values[i])
    }

    pub fn get_or(&self, name: &str, fallback: f64) -> f64 {
        self.get(name).unwrap_or(fallback)
    }

    pub fn is_empty(&self) -> bool {
        self.index.is_empty()
    }

    pub fn len(&self) -> usize {
        self.index.len()
    }

    pub fn iter(&self) -> impl Iterator<Item = (&str, f64)> + '_ {
        self.index.iter().map(|(name, &i)| (name.as_str(), self.values[i]))
    }

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

    pub fn begin(&mut self) {
        self.cursor = 0;
        self.reordered = false;
    }

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

#[derive(Clone, Debug, Default)]
pub struct Faults(BTreeMap<u64, f64>);

impl Faults {
    pub fn from_pairs(pairs: impl IntoIterator<Item = (u64, f64)>) -> Self {
        Self(pairs.into_iter().map(|(id, m)| (id, m.clamp(0.0, 1.0))).collect())
    }

    pub fn get(&self, id: u64) -> f64 {
        self.0.get(&id).copied().unwrap_or(0.0)
    }

    pub fn any(&self) -> bool {
        self.0.values().any(|&m| m > 0.0)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DerivedFailure {
    pub fbw_id: u64,
    pub magnitude: f64,
    pub deep_component: &'static str,
    pub reason: &'static str,
}
