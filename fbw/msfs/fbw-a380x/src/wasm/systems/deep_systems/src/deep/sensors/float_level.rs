const THERMAL_EXPANSION_PER_C: f64 = 7.5e-4;
const REFERENCE_TEMP_C: f64 = 15.0;
const FLOAT_TAU_S: f64 = 3.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct FloatLevelFaults {
    pub float_stuck: f64,
    pub sender_bias: f64,
    pub open_circuit: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct FloatLevelOutput {
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

    pub fn step(&mut self, true_volume_frac: f64, fluid_temp_c: f64, faults: &FloatLevelFaults, dt_s: f64) -> FloatLevelOutput {
        let dt = dt_s.max(0.0);
        if faults.open_circuit.clamp(0.0, 1.0) >= 0.98 {
            return FloatLevelOutput { indicated_frac: 0.0, stuck: false };
        }

        let apparent_frac = true_volume_frac.max(0.0) * (1.0 + THERMAL_EXPANSION_PER_C * (fluid_temp_c - REFERENCE_TEMP_C));

        let stuck = faults.float_stuck.clamp(0.0, 1.0) >= 0.98;
        if !stuck {
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
