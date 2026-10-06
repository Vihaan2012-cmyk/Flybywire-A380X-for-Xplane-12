pub const R_AIR: f64 = 287.05;
pub const CP_AIR: f64 = 1005.0;
pub const GAMMA_AIR: f64 = 1.4;

pub const CP_GAS: f64 = 1150.0;
pub const GAMMA_GAS: f64 = 1.333;
pub const R_GAS: f64 = R_AIR;

pub fn pressure_ratio_from_temperature_ratio(temperature_ratio: f64, gamma: f64) -> f64 {
    temperature_ratio.max(1e-6).powf(gamma / (gamma - 1.0))
}

pub fn temperature_ratio_from_pressure_ratio(pressure_ratio: f64, gamma: f64) -> f64 {
    pressure_ratio.max(1e-6).powf((gamma - 1.0) / gamma)
}

pub fn total_temperature(static_temp_k: f64, mach: f64, gamma: f64) -> f64 {
    static_temp_k * (1.0 + 0.5 * (gamma - 1.0) * mach * mach)
}

pub fn total_pressure(static_pressure_pa: f64, mach: f64, gamma: f64) -> f64 {
    static_pressure_pa * pressure_ratio_from_temperature_ratio(1.0 + 0.5 * (gamma - 1.0) * mach * mach, gamma)
}

pub fn critical_pressure_ratio(gamma: f64) -> f64 {
    ((gamma + 1.0) / 2.0).powf(gamma / (gamma - 1.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn total_conditions_match_static_at_zero_mach() {
        assert!((total_temperature(288.15, 0.0, GAMMA_AIR) - 288.15).abs() < 1e-9);
        assert!((total_pressure(101325.0, 0.0, GAMMA_AIR) - 101325.0).abs() < 1e-6);
    }

    #[test]
    fn total_temperature_rises_with_mach() {
        let low = total_temperature(288.15, 0.2, GAMMA_AIR);
        let high = total_temperature(288.15, 0.85, GAMMA_AIR);
        assert!(high > low);
    }

    #[test]
    fn critical_pressure_ratio_is_about_1_89_for_air() {
        assert!((critical_pressure_ratio(GAMMA_AIR) - 1.8929).abs() < 0.001);
    }
}
