use super::weather_truth::EnvironmentTruth;
use crate::deep::thermal_zones::network::OutsideAir;

pub fn outside_air(truth: &EnvironmentTruth) -> OutsideAir {
    OutsideAir { static_temp_c: truth.sat_c, mach: truth.mach(), true_airspeed_m_s: truth.tas_ms }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ground_static_conditions_give_zero_mach_and_matching_temperature() {
        let truth = EnvironmentTruth { sat_c: 22.0, tas_ms: 0.0, ..Default::default() };
        let air = outside_air(&truth);
        assert_eq!(air.static_temp_c, 22.0);
        assert_eq!(air.mach, 0.0);
        assert_eq!(air.true_airspeed_m_s, 0.0);
    }

    #[test]
    fn cruise_conditions_give_a_nonzero_mach_consistent_with_tas_and_sat() {
        let truth = EnvironmentTruth { sat_c: -56.5, tas_ms: 250.0, ..Default::default() };
        let air = outside_air(&truth);
        assert!(air.mach > 0.7 && air.mach < 0.9, "{}", air.mach);
        assert_eq!(air.true_airspeed_m_s, 250.0);
    }
}
