//! Shared gas-property constants and isentropic relations. Standard
//! textbook values (e.g. Cohen, Rogers & Saravanamuttoo, *Gas Turbine
//! Theory*; Mattingly, *Elements of Gas Turbine Propulsion*), not specific
//! to this engine.

/// Specific gas constant, dry air, J/(kg·K).
pub const R_AIR: f64 = 287.05;
/// Specific heat at constant pressure, cold section (air), J/(kg·K).
pub const CP_AIR: f64 = 1005.0;
/// Ratio of specific heats, cold section.
pub const GAMMA_AIR: f64 = 1.4;

/// Specific heat at constant pressure, hot section (combustion products),
/// J/(kg·K). Combustion gas cp rises with temperature; a single typical
/// value in the range quoted for turbine-inlet-temperature gas (~1150-1244
/// J/(kg·K)) is used rather than a temperature-dependent curve, for a
/// tractable real-time model. Documented simplification.
pub const CP_GAS: f64 = 1150.0;
/// Ratio of specific heats, hot section.
pub const GAMMA_GAS: f64 = 1.333;
/// Specific gas constant, combustion products, J/(kg·K). Close enough to
/// dry air's for a kerosene/air mixture at typical fuel/air ratios that
/// using the same value is a standard simplification.
pub const R_GAS: f64 = R_AIR;

/// Total pressure from total temperature ratio and gamma (isentropic
/// relation): `Pt2/Pt1 = (Tt2/Tt1)^(gamma/(gamma-1))`.
pub fn pressure_ratio_from_temperature_ratio(temperature_ratio: f64, gamma: f64) -> f64 {
    temperature_ratio.max(1e-6).powf(gamma / (gamma - 1.0))
}

/// The inverse: temperature ratio from a pressure ratio.
pub fn temperature_ratio_from_pressure_ratio(pressure_ratio: f64, gamma: f64) -> f64 {
    pressure_ratio.max(1e-6).powf((gamma - 1.0) / gamma)
}

/// Freestream total temperature from static temperature and Mach
/// (isentropic, calorically perfect gas): `Tt = T*(1 + (gamma-1)/2 * M^2)`.
pub fn total_temperature(static_temp_k: f64, mach: f64, gamma: f64) -> f64 {
    static_temp_k * (1.0 + 0.5 * (gamma - 1.0) * mach * mach)
}

/// Freestream total pressure from static pressure and Mach (isentropic).
pub fn total_pressure(static_pressure_pa: f64, mach: f64, gamma: f64) -> f64 {
    static_pressure_pa * pressure_ratio_from_temperature_ratio(1.0 + 0.5 * (gamma - 1.0) * mach * mach, gamma)
}

/// The critical pressure ratio (Pt/P) at which a convergent nozzle chokes.
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
