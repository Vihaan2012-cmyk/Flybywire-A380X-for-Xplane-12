use super::rng::Rng;

const UERE_M: f64 = 4.0;
const MIN_SATELLITES_FOR_FIX: u32 = 4;
const REFERENCE_SATELLITES: f64 = 8.0;
const REFERENCE_HDOP: f64 = 1.2;
const MAX_SPOOF_WALKOFF_MS: f64 = 1.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct GpsFaults {
    pub receiver_fault: f64,
    pub antenna_fault: f64,
    pub antenna_degradation: f64,
    pub jamming: f64,
    pub spoof_target_offset_m: [f64; 2],
}

#[derive(Clone, Copy, Debug, Default)]
pub struct GpsOutput {
    pub position_error_1sigma_m: f64,
    pub applied_spoof_offset_m: [f64; 2],
    pub position_offset_m: [f64; 2],
    pub valid: bool,
    pub effective_satellites: f64,
}

pub struct GpsReceiver {
    rng: Rng,
    applied_spoof_offset_m: [f64; 2],
}

impl GpsReceiver {
    pub fn new(seed: u64) -> Self {
        Self { rng: Rng::new(seed), applied_spoof_offset_m: [0.0, 0.0] }
    }

    pub fn step(&mut self, satellites_visible: u32, faults: &GpsFaults, dt_s: f64) -> GpsOutput {
        let dt = dt_s.max(0.0);
        if faults.receiver_fault.clamp(0.0, 1.0) >= 0.98 || faults.antenna_fault.clamp(0.0, 1.0) >= 0.98 {
            return GpsOutput { valid: false, ..Default::default() };
        }

        let jam = (faults.jamming.clamp(0.0, 1.0) + faults.antenna_degradation.max(0.0)).min(1.0);
        let effective_satellites = (satellites_visible as f64 * (1.0 - jam)).max(0.0);

        if effective_satellites < MIN_SATELLITES_FOR_FIX as f64 {
            return GpsOutput { effective_satellites, valid: false, ..Default::default() };
        }

        let hdop = REFERENCE_HDOP * (REFERENCE_SATELLITES / effective_satellites.max(1.0));
        let sigma_m = hdop * UERE_M;
        let noise_n = self.rng.gaussian() * sigma_m / std::f64::consts::SQRT_2;
        let noise_e = self.rng.gaussian() * sigma_m / std::f64::consts::SQRT_2;

        for i in 0..2 {
            let target = faults.spoof_target_offset_m[i];
            let delta = target - self.applied_spoof_offset_m[i];
            let max_step = MAX_SPOOF_WALKOFF_MS * dt;
            self.applied_spoof_offset_m[i] += delta.clamp(-max_step, max_step);
        }

        GpsOutput {
            position_error_1sigma_m: sigma_m,
            applied_spoof_offset_m: self.applied_spoof_offset_m,
            position_offset_m: [noise_n + self.applied_spoof_offset_m[0], noise_e + self.applied_spoof_offset_m[1]],
            valid: true,
            effective_satellites,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn more_satellites_gives_lower_position_error() {
        let mut few = GpsReceiver::new(1);
        let mut many = GpsReceiver::new(1);
        let few_out = few.step(5, &GpsFaults::default(), 1.0);
        let many_out = many.step(12, &GpsFaults::default(), 1.0);
        assert!(few_out.valid && many_out.valid);
        assert!(few_out.position_error_1sigma_m > many_out.position_error_1sigma_m);
    }

    #[test]
    fn fewer_than_four_satellites_has_no_fix() {
        let mut gps = GpsReceiver::new(1);
        let out = gps.step(3, &GpsFaults::default(), 1.0);
        assert!(!out.valid);
    }

    #[test]
    fn heavy_jamming_reduces_effective_satellites_below_the_fix_minimum() {
        let mut gps = GpsReceiver::new(1);
        let faults = GpsFaults { jamming: 0.9, ..Default::default() };
        let out = gps.step(8, &faults, 1.0);
        assert!(out.effective_satellites < MIN_SATELLITES_FOR_FIX as f64);
        assert!(!out.valid);
    }

    #[test]
    fn light_jamming_still_gives_a_fix_with_more_noise() {
        let mut clean = GpsReceiver::new(2);
        let mut jammed = GpsReceiver::new(2);
        let clean_out = clean.step(10, &GpsFaults::default(), 1.0);
        let jammed_out = jammed.step(10, &GpsFaults { jamming: 0.4, ..Default::default() }, 1.0);
        assert!(jammed_out.valid);
        assert!(jammed_out.position_error_1sigma_m > clean_out.position_error_1sigma_m);
    }

    #[test]
    fn receiver_fault_gives_no_fix_regardless_of_satellites() {
        let mut gps = GpsReceiver::new(1);
        let faults = GpsFaults { receiver_fault: 1.0, ..Default::default() };
        let out = gps.step(12, &faults, 1.0);
        assert!(!out.valid);
    }

    #[test]
    fn antenna_fault_gives_no_fix_regardless_of_satellites() {
        let mut gps = GpsReceiver::new(1);
        let faults = GpsFaults { antenna_fault: 1.0, ..Default::default() };
        let out = gps.step(12, &faults, 1.0);
        assert!(!out.valid);
    }

    #[test]
    fn degraded_antenna_acts_like_jamming_on_the_effective_satellite_count() {
        let mut healthy = GpsReceiver::new(1);
        let mut degraded = GpsReceiver::new(1);
        let faults = GpsFaults { antenna_degradation: 0.5, ..Default::default() };
        let healthy_out = healthy.step(10, &GpsFaults::default(), 1.0);
        let degraded_out = degraded.step(10, &faults, 1.0);
        assert!(degraded_out.valid);
        assert!(degraded_out.effective_satellites < healthy_out.effective_satellites);
        assert!(degraded_out.position_error_1sigma_m > healthy_out.position_error_1sigma_m);
    }

    #[test]
    fn spoofing_ramps_gradually_rather_than_jumping() {
        let mut gps = GpsReceiver::new(1);
        let faults = GpsFaults { spoof_target_offset_m: [500.0, 0.0], ..Default::default() };
        let out = gps.step(10, &faults, 1.0);
        assert!(out.applied_spoof_offset_m[0] < 5.0, "{}", out.applied_spoof_offset_m[0]);
        assert!(out.applied_spoof_offset_m[0] > 0.0);
    }

    #[test]
    fn spoofing_eventually_reaches_its_target_given_enough_time() {
        let mut gps = GpsReceiver::new(1);
        let faults = GpsFaults { spoof_target_offset_m: [50.0, -20.0], ..Default::default() };
        let mut out = GpsOutput::default();
        for _ in 0..1000 {
            out = gps.step(10, &faults, 1.0);
        }
        assert!((out.applied_spoof_offset_m[0] - 50.0).abs() < 1.0);
        assert!((out.applied_spoof_offset_m[1] - (-20.0)).abs() < 1.0);
    }

    #[test]
    fn no_nan_at_zero_dt_or_zero_satellites() {
        let mut gps = GpsReceiver::new(1);
        let out = gps.step(0, &GpsFaults::default(), 0.0);
        assert!(!out.valid);
        assert!(out.effective_satellites.is_finite());
    }
}
