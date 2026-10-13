//! Float-type liquid level transmitter: a float riding the liquid surface,
//! coupled through a lever arm to a potentiometer (or magnetically to a
//! reed-switch string), used here for hydraulic reservoir quantity
//! indication -- deliberately its own model rather than reusing
//! [`super::discrete::PressureTransducer`] or
//! [`super::discrete::temperature_sensor_reading_c`] under a wrong label
//! (a float sender is neither): a hydraulic reservoir's indicated quantity
//! is a *level*, not a pressure or a temperature, even though temperature
//! is exactly what makes that level move without any real fluid loss (see
//! below).
//!
//! ## Thermal expansion: level changes without a leak
//! Aircraft hydraulic fluid (phosphate-ester type, e.g. Skydrol-class
//! fluids widely used in transport hydraulics) expands measurably with
//! temperature; published fluid-property data sheets for this fluid class
//! commonly cite a volumetric thermal expansion coefficient on the order
//! of several parts in ten-thousand per degree C -- 7.5e-4 /C is used here
//! as a representative mid-value for this fluid class (GENERIC: the exact
//! A380 fluid specification's data sheet is not in scope). This is why
//! real hydraulic reservoir quantity gauges are commonly marked with a
//! "cold"/"hot" range or read differently on the ground before/after a
//! flight: the same *mass* of fluid occupies a larger *volume* hot, and the
//! float only ever senses volume (surface height), not mass.
//!
//! ## Float dynamics and faults
//! The float is deliberately damped (a light float in an undamped linkage
//! would bounce with every acceleration/slosh cycle in turbulence or
//! manoeuvring), giving it a first-order lag rather than an instantaneous
//! reading -- GENERIC time constant, sized to reject short-period slosh
//! while still tracking a real, sustained level change. Faults:
//! - **binding** (corrosion, fouling): the float sticks, exactly like the
//!   `pitot_blocked`-style latching this directory uses elsewhere -- a
//!   heavily bound float simply stops moving.
//! - **sender bias**: potentiometer wiper wear/miscalibration, a signed
//!   offset.
//! - **open circuit**: wiring/potentiometer open, and the indicating
//!   system defaults to a fail-safe low reading (`0`) rather than
//!   extrapolating -- prompting an inspection rather than a false
//!   assurance of a full reservoir, the same fail-safe convention this
//!   directory's other capacitance/resistance sensors use.

/// Representative phosphate-ester hydraulic fluid volumetric thermal
/// expansion coefficient, per degree C. GENERIC (see module docs).
const THERMAL_EXPANSION_PER_C: f64 = 7.5e-4;
/// Reference temperature the reservoir's "full" mark is calibrated at, C.
/// GENERIC (a typical ambient ground-fill reference).
const REFERENCE_TEMP_C: f64 = 15.0;
/// Float mechanical response time constant, seconds. GENERIC.
const FLOAT_TAU_S: f64 = 3.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct FloatLevelFaults {
    /// Float mechanically bound (corrosion/fouling): `0.0` free .. `1.0`
    /// (>=0.98) fully seized.
    pub float_stuck: f64,
    /// Signed sender calibration bias, as a fraction of full scale.
    pub sender_bias: f64,
    /// Open circuit: `1.0` (>=0.98) fully open.
    pub open_circuit: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct FloatLevelOutput {
    /// Indicated level, `0.0..~1.05` (thermal expansion can push a
    /// near-full reservoir slightly past its cold-fill "full" mark).
    pub indicated_frac: f64,
    pub stuck: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct FloatLevelSensor {
    indicated_frac: f64,
}

impl FloatLevelSensor {
    pub fn new(initial_frac: f64) -> Self {
        Self { indicated_frac: initial_frac.clamp(0.0, 1.2) }
    }

    /// `true_volume_frac`: the reservoir's true liquid volume as a fraction
    /// of its rated capacity (already reflects any real leak -- that is
    /// this sensor's *input*, not something it models). `fluid_temp_c`:
    /// the fluid's own temperature, for the thermal-expansion effect.
    pub fn step(&mut self, true_volume_frac: f64, fluid_temp_c: f64, faults: &FloatLevelFaults, dt_s: f64) -> FloatLevelOutput {
        let dt = dt_s.max(0.0);
        if faults.open_circuit.clamp(0.0, 1.0) >= 0.98 {
            return FloatLevelOutput { indicated_frac: 0.0, stuck: false };
        }

        let apparent_frac = true_volume_frac.max(0.0) * (1.0 + THERMAL_EXPANSION_PER_C * (fluid_temp_c - REFERENCE_TEMP_C));

        let stuck = faults.float_stuck.clamp(0.0, 1.0) >= 0.98;
        if !stuck {
            // First-order float lag: an exact exponential step in `dt`
            // (not a fixed per-call blend fraction -- see `static_port.rs`'s
            // fixed-blend bug and fix for why this matters).
            let k = (-dt / FLOAT_TAU_S).exp();
            self.indicated_frac = apparent_frac + (self.indicated_frac - apparent_frac) * k;
        }

        let biased = (self.indicated_frac + faults.sender_bias).clamp(0.0, 1.2);
        FloatLevelOutput { indicated_frac: biased, stuck }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn settle(sensor: &mut FloatLevelSensor, volume: f64, temp_c: f64, faults: &FloatLevelFaults, dt: f64, n: u32) -> FloatLevelOutput {
        let mut out = FloatLevelOutput::default();
        for _ in 0..n {
            out = sensor.step(volume, temp_c, faults, dt);
        }
        out
    }

    #[test]
    fn healthy_sensor_settles_on_the_true_volume_at_reference_temperature() {
        let mut s = FloatLevelSensor::new(0.5);
        let out = settle(&mut s, 0.8, REFERENCE_TEMP_C, &FloatLevelFaults::default(), 0.5, 200);
        assert!((out.indicated_frac - 0.8).abs() < 0.01, "{}", out.indicated_frac);
    }

    #[test]
    fn heating_the_fluid_raises_the_indicated_level_with_no_real_volume_change() {
        let mut cold = FloatLevelSensor::new(0.6);
        let mut hot = FloatLevelSensor::new(0.6);
        let cold_out = settle(&mut cold, 0.6, 15.0, &FloatLevelFaults::default(), 0.5, 200);
        let hot_out = settle(&mut hot, 0.6, 90.0, &FloatLevelFaults::default(), 0.5, 200);
        assert!(hot_out.indicated_frac > cold_out.indicated_frac, "cold {} hot {}", cold_out.indicated_frac, hot_out.indicated_frac);
    }

    #[test]
    fn a_real_leak_still_lowers_the_indicated_level_despite_temperature() {
        let mut leaking_hot = FloatLevelSensor::new(0.9);
        let out = settle(&mut leaking_hot, 0.3, 90.0, &FloatLevelFaults::default(), 0.5, 200);
        // Even heated, a true volume of 0.3 must read well below a healthy
        // full reservoir -- heat cannot mask a real, large leak.
        assert!(out.indicated_frac < 0.5, "{}", out.indicated_frac);
    }

    #[test]
    fn stuck_float_freezes_regardless_of_true_volume_changes() {
        let mut s = FloatLevelSensor::new(0.7);
        let faults = FloatLevelFaults { float_stuck: 1.0, ..Default::default() };
        let frozen = s.step(0.7, REFERENCE_TEMP_C, &faults, 0.5).indicated_frac;
        let out = settle(&mut s, 0.1, REFERENCE_TEMP_C, &faults, 0.5, 200);
        assert!((out.indicated_frac - frozen).abs() < 1e-6, "frozen {} now {}", frozen, out.indicated_frac);
        assert!(out.stuck);
    }

    #[test]
    fn open_circuit_reads_a_conservative_zero() {
        let mut s = FloatLevelSensor::new(0.9);
        let faults = FloatLevelFaults { open_circuit: 1.0, ..Default::default() };
        let out = s.step(0.9, REFERENCE_TEMP_C, &faults, 0.5);
        assert_eq!(out.indicated_frac, 0.0);
    }

    #[test]
    fn sender_bias_offsets_the_reading() {
        let mut s = FloatLevelSensor::new(0.5);
        let faults = FloatLevelFaults { sender_bias: 0.1, ..Default::default() };
        let out = settle(&mut s, 0.5, REFERENCE_TEMP_C, &faults, 0.5, 200);
        assert!((out.indicated_frac - 0.6).abs() < 0.01, "{}", out.indicated_frac);
    }

    #[test]
    fn no_nan_at_zero_dt_or_rest() {
        let mut s = FloatLevelSensor::new(0.0);
        let out = s.step(0.0, 0.0, &FloatLevelFaults::default(), 0.0);
        assert!(out.indicated_frac.is_finite());
    }
}
