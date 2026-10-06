use super::rng::Rng;

pub const MAX_RANGE_FT: f64 = 2500.0;
const MULTIPATH_REFERENCE_HEIGHT_FT: f64 = 50.0;
const MULTIPATH_BASE_SIGMA_FT: f64 = 1.5;
const SPECULAR_TERRAIN_MULTIPLIER: f64 = 3.0;
const DIRECT_COUPLING_ERRONEOUS_FT: f64 = 8.0;
const DIRECT_COUPLING_NOISE_SIGMA_FT: f64 = 3.0;
const DIRECT_COUPLING_MONITOR_THRESHOLD_FT: f64 = 100.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct RadioAltimeterFaults {
    pub transceiver_fault: f64,
    pub tx_antenna_fault: f64,
    pub rx_antenna_fault: f64,
    pub rx_antenna_degradation: f64,
    pub false_offset_ft: f64,
    pub tracking_loop_degradation: f64,
    pub direct_coupling_fault: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct RadioAltimeterOutput {
    pub agl_ft: f64,
    pub valid: bool,
}

pub struct RadioAltimeter {
    rng: Rng,
}

impl RadioAltimeter {
    pub fn new(seed: u64) -> Self {
        Self { rng: Rng::new(seed) }
    }

    pub fn step(&mut self, true_agl_ft: f64, over_water_or_snow: bool, faults: &RadioAltimeterFaults) -> RadioAltimeterOutput {
        let any_part_fully_failed = faults.transceiver_fault.clamp(0.0, 1.0) >= 0.98
            || faults.tx_antenna_fault.clamp(0.0, 1.0) >= 0.98
            || faults.rx_antenna_fault.clamp(0.0, 1.0) >= 0.98;
        if any_part_fully_failed {
            return RadioAltimeterOutput { agl_ft: 0.0, valid: false };
        }

        let direct_coupling = faults.direct_coupling_fault.clamp(0.0, 1.0);
        if direct_coupling >= 0.5 {
            let noise_ft = self.rng.gaussian() * DIRECT_COUPLING_NOISE_SIGMA_FT;
            let erroneous_ft = (DIRECT_COUPLING_ERRONEOUS_FT + noise_ft).max(0.0);
            let monitor_flags_it = true_agl_ft > DIRECT_COUPLING_MONITOR_THRESHOLD_FT;
            return RadioAltimeterOutput { agl_ft: erroneous_ft, valid: !monitor_flags_it };
        }

        if !(0.0..=MAX_RANGE_FT).contains(&true_agl_ft) {
            return RadioAltimeterOutput { agl_ft: 0.0, valid: false };
        }

        let terrain_multiplier = if over_water_or_snow { SPECULAR_TERRAIN_MULTIPLIER } else { 1.0 };
        let height_factor = if true_agl_ft <= MULTIPATH_REFERENCE_HEIGHT_FT {
            true_agl_ft.max(0.0) / MULTIPATH_REFERENCE_HEIGHT_FT
        } else {
            MULTIPATH_REFERENCE_HEIGHT_FT / true_agl_ft
        };
        let severity = 1.0 + faults.tracking_loop_degradation.max(0.0) + faults.rx_antenna_degradation.max(0.0);
        let sigma_ft = MULTIPATH_BASE_SIGMA_FT * terrain_multiplier * height_factor * severity;
        let noise_ft = self.rng.gaussian() * sigma_ft;

        let agl_ft = (true_agl_ft + faults.false_offset_ft + noise_ft).max(-50.0);
        RadioAltimeterOutput { agl_ft, valid: true }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn healthy_unit_reads_close_to_true_agl_away_from_multipath_terrain() {
        let mut ra = RadioAltimeter::new(1);
        let out = ra.step(1000.0, false, &RadioAltimeterFaults::default());
        assert!(out.valid);
        assert!((out.agl_ft - 1000.0).abs() < 5.0, "{}", out.agl_ft);
    }

    #[test]
    fn above_range_is_invalid_not_clamped() {
        let mut ra = RadioAltimeter::new(1);
        let out = ra.step(MAX_RANGE_FT + 500.0, false, &RadioAltimeterFaults::default());
        assert!(!out.valid);
    }

    #[test]
    fn transceiver_fault_reports_invalid_even_at_a_normal_height() {
        let mut ra = RadioAltimeter::new(1);
        let faults = RadioAltimeterFaults { transceiver_fault: 1.0, ..Default::default() };
        let out = ra.step(500.0, false, &faults);
        assert!(!out.valid);
    }

    #[test]
    fn either_antenna_failing_reports_invalid_even_with_a_healthy_transceiver() {
        let mut tx_failed = RadioAltimeter::new(1);
        let tx_faults = RadioAltimeterFaults { tx_antenna_fault: 1.0, ..Default::default() };
        assert!(!tx_failed.step(500.0, false, &tx_faults).valid);

        let mut rx_failed = RadioAltimeter::new(1);
        let rx_faults = RadioAltimeterFaults { rx_antenna_fault: 1.0, ..Default::default() };
        assert!(!rx_failed.step(500.0, false, &rx_faults).valid);
    }

    #[test]
    fn degraded_receive_antenna_adds_noise_like_tracking_loop_degradation_but_stays_valid() {
        let mut healthy = RadioAltimeter::new(4);
        let mut degraded_antenna = RadioAltimeter::new(4);
        let faults = RadioAltimeterFaults { rx_antenna_degradation: 2.0, ..Default::default() };
        let n = 2000;
        let (mut healthy_sq, mut degraded_sq) = (0.0, 0.0);
        for _ in 0..n {
            let h = healthy.step(500.0, false, &RadioAltimeterFaults::default());
            let d = degraded_antenna.step(500.0, false, &faults);
            assert!(d.valid, "a merely degraded (not failed) antenna must still return valid data");
            healthy_sq += (h.agl_ft - 500.0).powi(2);
            degraded_sq += (d.agl_ft - 500.0).powi(2);
        }
        assert!(degraded_sq > healthy_sq, "healthy {healthy_sq} degraded {degraded_sq}");
    }

    #[test]
    fn false_offset_reproduces_a_small_negative_ground_reading() {
        let mut ra = RadioAltimeter::new(1);
        let faults = RadioAltimeterFaults { false_offset_ft: -6.0, ..Default::default() };
        let mut sum = 0.0;
        let n = 200;
        for _ in 0..n {
            sum += ra.step(0.0, false, &faults).agl_ft;
        }
        assert!((sum / n as f64 - (-6.0)).abs() < 2.0, "{}", sum / n as f64);
    }

    #[test]
    fn multipath_noise_is_worse_over_water_than_over_varied_terrain() {
        let mut over_land = RadioAltimeter::new(2);
        let mut over_water = RadioAltimeter::new(2);
        let n = 2000;
        let (mut land_sq, mut water_sq) = (0.0, 0.0);
        for _ in 0..n {
            let l = over_land.step(30.0, false, &RadioAltimeterFaults::default());
            let w = over_water.step(30.0, true, &RadioAltimeterFaults::default());
            land_sq += (l.agl_ft - 30.0).powi(2);
            water_sq += (w.agl_ft - 30.0).powi(2);
        }
        assert!(water_sq > land_sq, "land {land_sq} water {water_sq}");
    }

    #[test]
    fn multipath_noise_is_worse_close_to_the_ground_than_high_up() {
        let mut low = RadioAltimeter::new(3);
        let mut high = RadioAltimeter::new(3);
        let n = 2000;
        let (mut low_sq, mut high_sq) = (0.0, 0.0);
        for _ in 0..n {
            let l = low.step(20.0, false, &RadioAltimeterFaults::default());
            let h = high.step(2000.0, false, &RadioAltimeterFaults::default());
            low_sq += (l.agl_ft - 20.0).powi(2);
            high_sq += (h.agl_ft - 2000.0).powi(2);
        }
        assert!(low_sq > high_sq, "low {low_sq} high {high_sq}");
    }

    #[test]
    fn direct_coupling_reports_an_erroneous_low_height_at_cruise_and_the_monitor_flags_it() {
        let mut ra = RadioAltimeter::new(1);
        let faults = RadioAltimeterFaults { direct_coupling_fault: 1.0, ..Default::default() };
        let out = ra.step(20_000.0, false, &faults);
        assert!(out.agl_ft < 50.0, "{}", out.agl_ft);
        assert!(!out.valid, "the RA's own monitor must flag a height that cannot match any sane descent rate");
    }

    #[test]
    fn direct_coupling_near_the_ground_is_the_hard_to_detect_case() {
        let mut ra = RadioAltimeter::new(1);
        let faults = RadioAltimeterFaults { direct_coupling_fault: 1.0, ..Default::default() };
        let out = ra.step(10.0, false, &faults);
        assert!(out.valid, "close to the ground the leakage path height is plausible and the monitor does not catch it");
    }

    #[test]
    fn negative_agl_glitch_is_invalid() {
        let mut ra = RadioAltimeter::new(1);
        let out = ra.step(-5.0, false, &RadioAltimeterFaults::default());
        assert!(!out.valid);
    }

    #[test]
    fn no_nan_at_zero_height() {
        let mut ra = RadioAltimeter::new(1);
        let out = ra.step(0.0, false, &RadioAltimeterFaults::default());
        assert!(out.agl_ft.is_finite());
    }

    #[test]
    fn parked_on_a_flat_ramp_reads_within_about_a_foot_of_true_height() {
        let mut ra = RadioAltimeter::new(5);
        for _ in 0..500 {
            let out = ra.step(0.0, false, &RadioAltimeterFaults::default());
            assert!(out.valid);
            assert!(out.agl_ft.abs() < 1.0, "{}", out.agl_ft);
        }
    }

    #[test]
    fn multipath_is_worst_near_the_reference_height_not_at_touchdown() {
        let mut at_touchdown = RadioAltimeter::new(6);
        let mut at_reference = RadioAltimeter::new(6);
        let n = 2000;
        let (mut touchdown_sq, mut reference_sq) = (0.0, 0.0);
        for _ in 0..n {
            let t = at_touchdown.step(0.0, false, &RadioAltimeterFaults::default());
            let r = at_reference.step(MULTIPATH_REFERENCE_HEIGHT_FT, false, &RadioAltimeterFaults::default());
            touchdown_sq += (t.agl_ft - 0.0).powi(2);
            reference_sq += (r.agl_ft - MULTIPATH_REFERENCE_HEIGHT_FT).powi(2);
        }
        assert!(reference_sq > touchdown_sq, "touchdown {touchdown_sq} reference {reference_sq}");
    }
}
