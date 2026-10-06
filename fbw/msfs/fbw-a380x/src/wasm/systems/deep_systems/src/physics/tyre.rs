use crate::invariants;
use crate::physics::damage;
use crate::physics::gas;

pub const N2_MOLAR_MASS_KG_MOL: f64 = 0.0280134;
pub const N2_SPECIFIC_GAS_CONSTANT: f64 = gas::R_UNIVERSAL / N2_MOLAR_MASS_KG_MOL;

pub const COLD_PRESSURE_PA: f64 = 1_550_000.0;
pub const COLD_TEMP_K: f64 = 288.15;

pub const SOAK_TAU_S: f64 = 900.0;
pub const COOL_TAU_S: f64 = 1200.0;
pub const SOAK_RATE_PER_S: f64 = 1.0 / SOAK_TAU_S;
pub const COOL_RATE_PER_S: f64 = 1.0 / COOL_TAU_S;

pub const ROLL_HEAT_COEFF_C_PER_MS_PER_S: f64 = 0.02;

pub const LEAK_RATE_FRACTION_PER_S_AT_FULL_MAGNITUDE: f64 = 0.0005;

pub const NEW_TREAD_DEPTH_MM: f64 = 6.0;
pub const WEAR_RATE_MM_PER_S_AT_FULL_MAGNITUDE: f64 = 0.002;

pub const LEG_FAILURE_IDS: [u64; 4] = [32_101, 32_102, 32_103, 32_104];

pub const LEG_WHEEL_INDICES: [[usize; 4]; 4] = [
    [0, 1, 4, 5],
    [2, 3, 6, 7],
    [8, 9, 12, 13],
    [10, 11, 14, 15],
];

pub const BRAKED_WHEELS: usize = 16;

pub const WHEELS: usize = 22;

pub const WHEEL_NAMES: [&str; WHEELS] = [
    "L wing 1", "L wing 2", "R wing 1", "R wing 2", "L wing 3", "L wing 4", "R wing 3", "R wing 4", "L body 1", "L body 2",
    "R body 1", "R body 2", "L body 3", "L body 4", "R body 3", "R body 4", "Nose 1", "Nose 2", "L body 5", "L body 6",
    "R body 5", "R body 6",
];

pub const NOSE_FAILURE_ID: u64 = 32_100;
pub const UNBRAKED_WHEELS: [(usize, u64); WHEELS - BRAKED_WHEELS] = [
    (16, NOSE_FAILURE_ID),
    (17, NOSE_FAILURE_ID),
    (18, LEG_FAILURE_IDS[2]),
    (19, LEG_FAILURE_IDS[2]),
    (20, LEG_FAILURE_IDS[3]),
    (21, LEG_FAILURE_IDS[3]),
];

pub fn leg_of_wheel(wheel: usize) -> usize {
    LEG_WHEEL_INDICES.iter().position(|indices| indices.contains(&wheel)).expect("every wheel 0..16 is in exactly one leg")
}

#[derive(Clone, Copy, Debug)]
pub struct TyreWheel {
    pub temp_c: f64,
    pub leaked_fraction: f64,
    pub tread_mm: f64,
    pub fuse_plug_melted: bool,
}

impl Default for TyreWheel {
    fn default() -> Self {
        Self { temp_c: 15.0, leaked_fraction: 0.0, tread_mm: NEW_TREAD_DEPTH_MM, fuse_plug_melted: false }
    }
}

impl TyreWheel {
    pub fn pressure_pa(&self) -> f64 {
        let temp_k = self.temp_c + 273.15;
        COLD_PRESSURE_PA * (temp_k / COLD_TEMP_K) * (1.0 - self.leaked_fraction)
    }

    pub fn step(&mut self, brake_temp_c: f64, ambient_c: f64, groundspeed_ms: f64, magnitude: f64, delta: f64) -> bool {
        use crate::invariants::{self, Bound};

        let leak_rate = LEAK_RATE_FRACTION_PER_S_AT_FULL_MAGNITUDE * magnitude;
        let leaked_raw = self.leaked_fraction + leak_rate * delta;
        self.leaked_fraction = invariants::check("TYRE_LEAKED_FRACTION", leaked_raw, Bound::Range(0.0, 1.0), "TyreWheel::step (leak)");

        let pressure_ratio = 1.0 - self.leaked_fraction;
        let flex_heat_raw = ROLL_HEAT_COEFF_C_PER_MS_PER_S * (1.0 / pressure_ratio) * groundspeed_ms.max(0.0);
        let flex_heat = invariants::check("TYRE_FLEX_HEAT_RATE_C_S", flex_heat_raw, Bound::NonNegative, "TyreWheel::step (flex heat)");
        let d_temp = SOAK_RATE_PER_S * (brake_temp_c - self.temp_c) + flex_heat - COOL_RATE_PER_S * (self.temp_c - ambient_c);
        let temp_raw = self.temp_c + d_temp * delta;
        self.temp_c = invariants::check("TYRE_TEMPERATURE_C", temp_raw, Bound::TemperatureFloor(-273.15), "TyreWheel::step (temp)");

        let wear_rate = WEAR_RATE_MM_PER_S_AT_FULL_MAGNITUDE * magnitude;
        let tread_raw = self.tread_mm - wear_rate * delta;
        self.tread_mm = invariants::check("TYRE_TREAD_MM", tread_raw, Bound::NonNegative, "TyreWheel::step (tread)");

        if !self.fuse_plug_melted && self.temp_c > damage::FUSE_PLUG_MELT_C {
            self.fuse_plug_melted = true;
            self.leaked_fraction = 1.0;
            return true;
        }
        false
    }

    pub fn step_unbraked(&mut self, ambient_c: f64, groundspeed_ms: f64, magnitude: f64, delta: f64) -> bool {
        let own_temp = self.temp_c;
        self.step(own_temp, ambient_c, groundspeed_ms, magnitude, delta)
    }
}
