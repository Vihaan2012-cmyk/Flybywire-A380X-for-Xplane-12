fn fractional_frequency_drop(added_mass_kg: f64, effective_probe_mass_kg: f64) -> f64 {
    0.5 * added_mass_kg / effective_probe_mass_kg.max(1e-9)
}

const PROBE_EFFECTIVE_MASS_KG: f64 = 0.003;
const DETECTION_MASS_FRACTION: f64 = 0.05;
const ACCUM_KG_S_PER_LWC_TAS: f64 = 4e-7;
const DEICE_SECONDS: f64 = 3.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct IceDetectorFaults {
    pub heater_failure: f64,
    pub frequency_sensor_bias: f64,
    pub probe_damage_bias: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct IceDetectorOutput {
    pub ice_detected: bool,
    pub ice_kg: f64,
    pub deicing: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct IceDetector {
    ice_kg: f64,
    deice_remaining_s: f64,
}

impl IceDetector {
    pub fn new() -> Self {
        Self { ice_kg: 0.0, deice_remaining_s: 0.0 }
    }

    pub fn step(&mut self, sat_c: f64, tas_ms: f64, lwc_gm3: f64, faults: &IceDetectorFaults, dt_s: f64) -> IceDetectorOutput {
        let dt = dt_s.max(0.0);

        if self.deice_remaining_s > 0.0 {
            let clear_frac = (dt / DEICE_SECONDS.max(1e-6)).min(1.0);
            self.ice_kg = (self.ice_kg * (1.0 - clear_frac)).max(0.0);
            self.deice_remaining_s = (self.deice_remaining_s - dt).max(0.0);
        } else if sat_c < 0.0 && lwc_gm3 > 0.0 {
            self.ice_kg += ACCUM_KG_S_PER_LWC_TAS * lwc_gm3.max(0.0) * tas_ms.max(0.0) * dt;
        }

        let true_shift = fractional_frequency_drop(self.ice_kg, PROBE_EFFECTIVE_MASS_KG);
        let sensed_shift = true_shift + faults.frequency_sensor_bias + faults.probe_damage_bias;
        let ice_detected = sensed_shift >= DETECTION_MASS_FRACTION * 0.5;

        if ice_detected && self.deice_remaining_s <= 0.0 {
            let heater_ok = faults.heater_failure.clamp(0.0, 1.0) < 0.98;
            if heater_ok {
                self.deice_remaining_s = DEICE_SECONDS;
            }
        }

        IceDetectorOutput { ice_detected, ice_kg: self.ice_kg, deicing: self.deice_remaining_s > 0.0 }
    }
}

impl Default for IceDetector {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_icing_conditions_never_trips() {
        let mut d = IceDetector::new();
        let mut out = IceDetectorOutput::default();
        for _ in 0..1000 {
            out = d.step(15.0, 200.0, 0.0, &IceDetectorFaults::default(), 0.5);
        }
        assert!(!out.ice_detected);
        assert_eq!(out.ice_kg, 0.0);
    }

    #[test]
    fn healthy_detector_cycles_detect_then_deice() {
        let mut d = IceDetector::new();
        let faults = IceDetectorFaults::default();
        let mut tripped = false;
        let mut deiced_after_trip = false;
        for _ in 0..2000 {
            let out = d.step(-20.0, 200.0, 0.8, &faults, 0.1);
            if out.ice_detected {
                tripped = true;
            }
            if tripped && out.deicing {
                deiced_after_trip = true;
            }
        }
        assert!(tripped, "expected the detector to trip in heavy icing");
        assert!(deiced_after_trip, "expected the healthy detector to run its deice cycle after tripping");
    }

    #[test]
    fn failed_heater_latches_ice_detected_instead_of_cycling() {
        let mut d = IceDetector::new();
        let faults = IceDetectorFaults { heater_failure: 1.0, ..Default::default() };
        let mut out = IceDetectorOutput::default();
        for _ in 0..2000 {
            out = d.step(-20.0, 200.0, 0.8, &faults, 0.1);
        }
        assert!(out.ice_detected);
        assert!(!out.deicing, "a fully failed heater never starts a deice cycle");
        assert!(out.ice_kg > 0.0);
    }

    #[test]
    fn frequency_sensor_negative_bias_can_mask_real_icing() {
        let mut masked = IceDetector::new();
        let mut healthy = IceDetector::new();
        let masking_faults = IceDetectorFaults { frequency_sensor_bias: -1.0, ..Default::default() };
        let mut masked_ever = false;
        let mut healthy_ever = false;
        for _ in 0..50 {
            masked_ever |= masked.step(-20.0, 200.0, 0.8, &masking_faults, 0.1).ice_detected;
            healthy_ever |= healthy.step(-20.0, 200.0, 0.8, &IceDetectorFaults::default(), 0.1).ice_detected;
        }
        assert!(healthy_ever, "the healthy detector should find this icing");
        assert!(!masked_ever, "the biased detector should never report it");
        assert!(masked.step(-20.0, 200.0, 0.8, &masking_faults, 0.1).ice_kg > 0.0);
    }

    #[test]
    fn probe_damage_bias_can_cause_a_false_positive_with_no_ice() {
        let mut d = IceDetector::new();
        let faults = IceDetectorFaults { probe_damage_bias: 1.0, ..Default::default() };
        let out = d.step(20.0, 200.0, 0.0, &faults, 0.1);
        assert!(out.ice_detected);
        assert_eq!(out.ice_kg, 0.0);
    }

    #[test]
    fn no_nan_at_zero_dt_or_rest() {
        let mut d = IceDetector::new();
        let out = d.step(0.0, 0.0, 0.0, &IceDetectorFaults::default(), 0.0);
        assert!(out.ice_kg.is_finite());
    }
}
