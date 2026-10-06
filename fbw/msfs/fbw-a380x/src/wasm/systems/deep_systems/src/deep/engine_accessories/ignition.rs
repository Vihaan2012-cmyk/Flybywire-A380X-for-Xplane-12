pub const V_BUS_V: f64 = 115.0;
pub const CAPACITOR_ENERGY_J: f64 = 18.0;
const CHARGE_TIME_CONSTANT_S: f64 = 0.15;
const THRESHOLD_FRAC: f64 = 0.95;
const BREAKDOWN_NEW_V: f64 = 10_000.0;
const BREAKDOWN_ERODED_V: f64 = 22_000.0;
const EXCITER_PEAK_OUTPUT_V: f64 = 20_000.0;

pub fn breakdown_voltage_v(erosion: f64) -> f64 {
    let e = erosion.clamp(0.0, 1.0);
    BREAKDOWN_NEW_V + e * (BREAKDOWN_ERODED_V - BREAKDOWN_NEW_V)
}

pub fn can_fire(erosion: f64) -> bool {
    breakdown_voltage_v(erosion) <= EXCITER_PEAK_OUTPUT_V
}

pub fn spark_rate_hz(powered: bool, erosion: f64, exciter_failure: f64) -> f64 {
    let health = 1.0 - exciter_failure.clamp(0.0, 1.0);
    if !powered || health <= 1e-6 || !can_fire(erosion) {
        return 0.0;
    }
    let tau = CHARGE_TIME_CONSTANT_S / health;
    let period_s = -tau * (1.0 - THRESHOLD_FRAC).ln();
    1.0 / period_s
}

#[derive(Clone, Copy, Debug, Default)]
pub struct IgnitionFaults {
    pub exciter_a_failure: f64,
    pub exciter_b_failure: f64,
    pub igniter_a_erosion: f64,
    pub igniter_b_erosion: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct IgnitionState {
    pub chain_a_hz: f64,
    pub chain_b_hz: f64,
    pub chain_a_firing: bool,
    pub chain_b_firing: bool,
    pub no_ignition_available: bool,
}

pub fn step(powered: bool, faults: &IgnitionFaults) -> IgnitionState {
    let a_hz = spark_rate_hz(powered, faults.igniter_a_erosion, faults.exciter_a_failure);
    let b_hz = spark_rate_hz(powered, faults.igniter_b_erosion, faults.exciter_b_failure);
    IgnitionState {
        chain_a_hz: a_hz,
        chain_b_hz: b_hz,
        chain_a_firing: a_hz > 0.0,
        chain_b_firing: b_hz > 0.0,
        no_ignition_available: powered && a_hz <= 0.0 && b_hz <= 0.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unpowered_gives_no_sparks_and_no_nan() {
        let s = step(false, &IgnitionFaults::default());
        assert_eq!(s.chain_a_hz, 0.0);
        assert!(!s.no_ignition_available);
    }

    #[test]
    fn a_healthy_new_plug_sparks_at_a_positive_rate_when_powered() {
        let s = step(true, &IgnitionFaults::default());
        assert!(s.chain_a_hz > 0.0 && s.chain_b_hz > 0.0);
        assert!(s.chain_a_firing && s.chain_b_firing);
    }

    #[test]
    fn a_failed_exciter_stops_its_own_chain_but_not_the_other() {
        let s = step(true, &IgnitionFaults { exciter_a_failure: 1.0, ..Default::default() });
        assert_eq!(s.chain_a_hz, 0.0);
        assert!(s.chain_b_hz > 0.0);
        assert!(!s.no_ignition_available, "the B chain alone still gives ignition");
    }

    #[test]
    fn both_exciters_failed_means_no_ignition_available() {
        let s = step(true, &IgnitionFaults { exciter_a_failure: 1.0, exciter_b_failure: 1.0, ..Default::default() });
        assert!(s.no_ignition_available);
    }

    #[test]
    fn a_badly_eroded_plug_cannot_break_down_and_stops_firing() {
        assert!(can_fire(0.0));
        assert!(!can_fire(1.0));
        let s = step(true, &IgnitionFaults { igniter_a_erosion: 1.0, ..Default::default() });
        assert_eq!(s.chain_a_hz, 0.0);
        assert!(s.chain_b_hz > 0.0);
    }

    #[test]
    fn a_partially_failing_exciter_sparks_slower_not_just_less_energetically() {
        let healthy = spark_rate_hz(true, 0.0, 0.0);
        let degraded = spark_rate_hz(true, 0.0, 0.6);
        assert!(degraded > 0.0 && degraded < healthy);
    }

    #[test]
    fn breakdown_voltage_rises_monotonically_with_erosion() {
        assert!(breakdown_voltage_v(1.0) > breakdown_voltage_v(0.5));
        assert!(breakdown_voltage_v(0.5) > breakdown_voltage_v(0.0));
    }
}
