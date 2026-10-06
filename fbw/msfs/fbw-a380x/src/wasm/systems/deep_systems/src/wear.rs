use std::collections::BTreeMap;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Wear {
    pub hot_hours: f64,
    pub cycles: u32,
    pub thermal_stress_integral: f64,
    pub degradation_fraction: f64,
}

impl Default for Wear {
    fn default() -> Self {
        Self { hot_hours: 0.0, cycles: 0, thermal_stress_integral: 0.0, degradation_fraction: 0.0 }
    }
}

#[derive(Clone, Debug, Default)]
pub struct WearStore(BTreeMap<String, Wear>);

impl WearStore {
    pub fn get(&self, id: &str) -> Wear {
        self.0.get(id).copied().unwrap_or_default()
    }

    pub fn ids(&self) -> impl Iterator<Item = &str> {
        self.0.keys().map(String::as_str)
    }

    pub fn accumulate(&mut self, id: &str, hot_hours: f64, cycles: u32, thermal_stress: f64, degradation_delta: f64) {
        let w = self.0.entry(id.to_owned()).or_default();
        w.hot_hours += hot_hours;
        w.cycles += cycles;
        w.thermal_stress_integral += thermal_stress;
        w.degradation_fraction += degradation_delta;
    }

    pub fn set(&mut self, id: &str, wear: Wear) {
        self.0.insert(id.to_owned(), wear);
    }

    pub fn reset_component(&mut self, id: &str) {
        self.0.remove(id);
    }
}

thread_local! {
    static LATEST: std::cell::RefCell<Option<WearStore>> = const { std::cell::RefCell::new(None) };
}

pub fn publish(store: WearStore) {
    LATEST.with(|w| *w.borrow_mut() = Some(store));
}

pub fn snapshot() -> WearStore {
    LATEST.with(|w| w.borrow().clone()).unwrap_or_default()
}

pub fn reset_all() {
    LATEST.with(|w| *w.borrow_mut() = None);
}
