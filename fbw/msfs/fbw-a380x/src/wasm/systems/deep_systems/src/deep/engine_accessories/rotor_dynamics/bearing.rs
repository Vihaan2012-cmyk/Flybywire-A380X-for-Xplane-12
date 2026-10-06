const ROLLING_ELEMENT_COUNT: f64 = 16.0;
const DIAMETER_RATIO_D_OVER_D: f64 = 0.18;
const CONTACT_ANGLE_COS: f64 = 1.0;
const REFERENCE_AMPLITUDE_MM_S: f64 = 8.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct BearingFaults {
    pub outer_race_spall: f64,
    pub inner_race_spall: f64,
    pub rolling_element_spall: f64,
    pub cage_wear: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BearingSignature {
    pub outer_race: (f64, f64),
    pub inner_race: (f64, f64),
    pub ball_spin: (f64, f64),
    pub cage: (f64, f64),
}

pub fn defect_frequencies_hz(fr_hz: f64) -> (f64, f64, f64, f64) {
    let fr = fr_hz.max(0.0);
    let ratio = DIAMETER_RATIO_D_OVER_D * CONTACT_ANGLE_COS;
    let bpfo = (ROLLING_ELEMENT_COUNT / 2.0) * fr * (1.0 - ratio);
    let bpfi = (ROLLING_ELEMENT_COUNT / 2.0) * fr * (1.0 + ratio);
    let bsf = (1.0 / (2.0 * DIAMETER_RATIO_D_OVER_D)) * fr * (1.0 - ratio * ratio);
    let ftf = (fr / 2.0) * (1.0 - ratio);
    (bpfo, bpfi, bsf, ftf)
}

pub fn signature(omega_rad_s: f64, design_omega_rad_s: f64, faults: &BearingFaults) -> BearingSignature {
    let fr_hz = omega_rad_s.max(0.0) / (2.0 * std::f64::consts::PI);
    let (bpfo, bpfi, bsf, ftf) = defect_frequencies_hz(fr_hz);
    let speed_frac = (omega_rad_s.max(0.0) / design_omega_rad_s.max(1e-6)).min(1.5);
    let amp = |severity: f64| REFERENCE_AMPLITUDE_MM_S * severity.clamp(0.0, 1.0) * speed_frac;
    BearingSignature {
        outer_race: (bpfo, amp(faults.outer_race_spall)),
        inner_race: (bpfi, amp(faults.inner_race_spall)),
        ball_spin: (bsf, amp(faults.rolling_element_spall)),
        cage: (ftf, amp(faults.cage_wear)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DESIGN_OMEGA: f64 = 1000.0;

    #[test]
    fn a_healthy_bearing_has_zero_amplitude_at_every_defect_frequency_no_nan() {
        let s = signature(DESIGN_OMEGA, DESIGN_OMEGA, &BearingFaults::default());
        assert_eq!(s.outer_race.1, 0.0);
        assert_eq!(s.inner_race.1, 0.0);
        assert_eq!(s.ball_spin.1, 0.0);
        assert_eq!(s.cage.1, 0.0);
        assert!(!s.outer_race.0.is_nan());
    }

    #[test]
    fn zero_speed_gives_zero_frequencies_and_zero_amplitude() {
        let s = signature(0.0, DESIGN_OMEGA, &BearingFaults { outer_race_spall: 1.0, ..Default::default() });
        assert_eq!(s.outer_race.0, 0.0);
        assert_eq!(s.outer_race.1, 0.0);
    }

    #[test]
    fn each_defect_type_rings_only_at_its_own_frequency() {
        let s = signature(DESIGN_OMEGA, DESIGN_OMEGA, &BearingFaults { outer_race_spall: 1.0, ..Default::default() });
        assert!(s.outer_race.1 > 0.0);
        assert_eq!(s.inner_race.1, 0.0);
        assert_eq!(s.ball_spin.1, 0.0);
        assert_eq!(s.cage.1, 0.0);
        let freqs = [s.outer_race.0, s.inner_race.0, s.ball_spin.0, s.cage.0];
        for i in 0..freqs.len() {
            for j in (i + 1)..freqs.len() {
                assert!((freqs[i] - freqs[j]).abs() > 1e-6, "{freqs:?}");
            }
        }
    }

    #[test]
    fn inner_race_frequency_exceeds_outer_race_frequency_for_this_geometry() {
        let (bpfo, bpfi, _, _) = defect_frequencies_hz(100.0);
        assert!(bpfi > bpfo);
    }

    #[test]
    fn severity_scales_the_amplitude_linearly() {
        let half = signature(DESIGN_OMEGA, DESIGN_OMEGA, &BearingFaults { inner_race_spall: 0.5, ..Default::default() });
        let full = signature(DESIGN_OMEGA, DESIGN_OMEGA, &BearingFaults { inner_race_spall: 1.0, ..Default::default() });
        assert!((full.inner_race.1 - 2.0 * half.inner_race.1).abs() < 1e-9);
    }

    #[test]
    fn higher_shaft_speed_gives_more_amplitude_for_the_same_severity() {
        let slow = signature(DESIGN_OMEGA * 0.5, DESIGN_OMEGA, &BearingFaults { rolling_element_spall: 1.0, ..Default::default() });
        let fast = signature(DESIGN_OMEGA, DESIGN_OMEGA, &BearingFaults { rolling_element_spall: 1.0, ..Default::default() });
        assert!(fast.ball_spin.1 > slow.ball_spin.1);
        assert!(fast.ball_spin.0 > slow.ball_spin.0, "the defect frequency itself also rises with shaft speed");
    }
}
