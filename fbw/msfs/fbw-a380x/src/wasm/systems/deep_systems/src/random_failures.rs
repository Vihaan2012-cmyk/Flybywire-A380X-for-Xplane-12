use std::collections::BTreeSet;

use crate::deep::api::Area;

#[derive(Clone, Copy, Debug)]
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self(if seed == 0 { 0x9E3779B97F4A7C15 } else { seed })
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    pub fn seed(&self) -> u64 {
        self.0
    }
}

struct Component {
    id: u64,
    mtbf_hours: f64,
}

fn mtbf_hours_for(area: Area) -> Option<f64> {
    match area {
        Area::Electrical | Area::Sensors | Area::AvionicsNetwork | Area::Communications | Area::AutoFlight | Area::Environment => Some(25_000.0),
        Area::Hydraulics | Area::Fuel | Area::PneumaticDucts => Some(6_000.0),
        Area::EngineAccessories | Area::EngineCore => Some(20_000.0),
        Area::GearStructure | Area::ThermalZones | Area::FireIce | Area::Apu | Area::Oxygen | Area::Cabin | Area::FlightControls | Area::Breakers | Area::Wiring => Some(15_000.0),
        Area::FlightModel | Area::Integration => None,
    }
}

const EXCLUDE_KEYWORDS: &[&str] = &["touchdown", "overweight landing", "exceedance", "creep", "wear-out", "tyre burst", "overspeed", "vmo", "mmo"];

fn excluded(name: &str, effect: &str) -> bool {
    let hay = format!("{} {}", name.to_ascii_lowercase(), effect.to_ascii_lowercase());
    EXCLUDE_KEYWORDS.iter().any(|k| hay.contains(k))
}

fn components() -> Vec<Component> {
    let mut seen: BTreeSet<String> = BTreeSet::new();
    let mut v = Vec::new();
    for f in &crate::deep::registry().failures {
        let Some(mtbf_hours) = mtbf_hours_for(f.area) else { continue };
        if excluded(&f.name, &f.effect) {
            continue;
        }
        if !seen.insert(f.component.clone()) {
            continue;
        }
        v.push(Component { id: f.id, mtbf_hours });
    }
    v
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Config {
    pub enabled: bool,
    pub rate_multiplier: f64,
}

impl Default for Config {
    fn default() -> Self {
        Self { enabled: false, rate_multiplier: 1.0 }
    }
}

pub struct RandomFailures {
    pub config: Config,
    rng: Rng,
    components: Vec<Component>,
}

impl RandomFailures {
    pub fn new(seed: u64) -> Self {
        Self { config: Config::default(), rng: Rng::new(seed), components: components() }
    }

    pub fn restore(&mut self, config: Config, rng_seed: u64) {
        self.config = config;
        self.rng = Rng::new(rng_seed);
    }

    pub fn snapshot(&self) -> (Config, u64) {
        (self.config, self.rng.seed())
    }

    pub fn component_count(&self) -> usize {
        self.components.len()
    }

    pub fn update(&mut self, delta_hours: f64, already_active: &BTreeSet<u64>) -> Vec<u64> {
        if !self.config.enabled || self.config.rate_multiplier <= 0.0 || delta_hours <= 0.0 {
            return Vec::new();
        }
        let mut triggered = Vec::new();
        for c in &self.components {
            if already_active.contains(&c.id) {
                continue;
            }
            let lambda = self.config.rate_multiplier / c.mtbf_hours;
            let p = 1.0 - (-lambda * delta_hours).exp();
            if self.rng.next_f64() < p {
                triggered.push(c.id);
            }
        }
        triggered
    }
}
