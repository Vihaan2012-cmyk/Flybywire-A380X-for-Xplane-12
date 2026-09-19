//! Shared gas-property constants and isentropic relations for the APU deep
//! model.
//!
//! Standard textbook values (Cohen, Rogers & Saravanamuttoo, *Gas Turbine
//! Theory*; Mattingly, *Elements of Gas Turbine Propulsion*) -- generic gas
//! turbine thermodynamics, not specific to the PW980A (no public gas
//! property data exists, or would need to exist, for one particular
//! machine's working fluid). This is this directory's own copy, not shared
//! with `physics/engine`'s equivalent file: the brief requires every deep
//! system directory to be self-contained (its `mod.rs` declares its own
//! submodules and nothing outside the directory may be depended on), so the
//! same public textbook constants are independently restated here rather
//! than imported from a sibling module that might change shape without
//! this one knowing.

/// Specific gas constant, dry air, J/(kg*K).
pub const R_AIR_J_KG_K: f64 = 287.05;
/// Specific heat at constant pressure, cold section (air), J/(kg*K).
pub const CP_AIR_J_KG_K: f64 = 1005.0;
/// Ratio of specific heats, cold section.
pub const GAMMA_AIR: f64 = 1.4;

/// Specific heat at constant pressure, hot section (combustion products),
/// J/(kg*K). Real combustion-gas cp rises with temperature; one typical
/// value in the range quoted for turbine-inlet-temperature gas
/// (~1150-1244 J/(kg*K)) is used instead of a temperature-dependent curve,
/// the standard simplification for a tractable real-time cycle model.
pub const CP_GAS_J_KG_K: f64 = 1150.0;
/// Ratio of specific heats, hot section.
pub const GAMMA_GAS: f64 = 1.333;

/// ICAO standard atmosphere reference conditions, used for "corrected"
/// (density-normalised) speeds and mass flows throughout this directory.
pub const T_REF_K: f64 = 288.15;
pub const P_REF_PA: f64 = 101_325.0;

/// Total pressure ratio from a total temperature ratio and gamma (the
/// isentropic relation): `P2/P1 = (T2/T1)^(gamma/(gamma-1))`.
pub fn pressure_ratio_from_temperature_ratio(temperature_ratio: f64, gamma: f64) -> f64 {
    temperature_ratio.max(1e-6).powf(gamma / (gamma - 1.0))
}

/// The inverse: temperature ratio from a pressure ratio.
pub fn temperature_ratio_from_pressure_ratio(pressure_ratio: f64, gamma: f64) -> f64 {
    pressure_ratio.max(1e-6).powf((gamma - 1.0) / gamma)
}

/// Corrected ("referred") mass flow: `mdot * sqrt(T/Tref) / (P/Pref)`. The
/// standard turbomachinery normalisation that makes a single map usable
/// across the whole range of inlet density a machine sees.
pub fn corrected_flow_kg_s(mdot_kg_s: f64, t_k: f64, p_pa: f64) -> f64 {
    if p_pa <= 1.0 {
        return 0.0;
    }
    mdot_kg_s.max(0.0) * (t_k.max(1.0) / T_REF_K).sqrt() / (p_pa / P_REF_PA)
}

/// Inverse of `corrected_flow_kg_s`: actual mass flow from a corrected flow
/// at given inlet conditions.
pub fn actual_flow_kg_s(mdot_corrected_kg_s: f64, t_k: f64, p_pa: f64) -> f64 {
    mdot_corrected_kg_s.max(0.0) * (p_pa.max(0.0) / P_REF_PA) / (t_k.max(1.0) / T_REF_K).sqrt()
}

/// Freestream/ram total temperature from static temperature and Mach
/// (isentropic, calorically perfect gas): `Tt = T*(1 + (gamma-1)/2 * M^2)`.
/// Used by `start_envelope.rs` for the ram temperature rise the APU's inlet
/// scoop sees at airspeed.
pub fn total_temperature_k(static_temp_k: f64, mach: f64, gamma: f64) -> f64 {
    static_temp_k.max(0.0) * (1.0 + 0.5 * (gamma - 1.0) * mach * mach)
}

/// Freestream/ram total pressure from static pressure and Mach (isentropic).
pub fn total_pressure_pa(static_pressure_pa: f64, mach: f64, gamma: f64) -> f64 {
    static_pressure_pa.max(0.0)
        * pressure_ratio_from_temperature_ratio(1.0 + 0.5 * (gamma - 1.0) * mach * mach, gamma)
}

/// Ideal-gas density ratio to the ICAO standard-day sea-level reference,
/// from static pressure and temperature: `rho/rho_ref = (P/Pref)/(T/Tref)`
/// (same gas, so the specific gas constant cancels). Used by
/// `start_envelope.rs` for density-driven relight-envelope and dynamic-
/// pressure effects.
pub fn density_ratio(pressure_pa: f64, temperature_k: f64) -> f64 {
    let p_ratio = pressure_pa.max(0.0) / P_REF_PA;
    let t_ratio = temperature_k.max(1.0) / T_REF_K;
    p_ratio / t_ratio
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pressure_and_temperature_ratio_relations_are_mutual_inverses() {
        let pr = 4.2;
        let tr = temperature_ratio_from_pressure_ratio(pr, GAMMA_AIR);
        let pr_back = pressure_ratio_from_temperature_ratio(tr, GAMMA_AIR);
        assert!((pr_back - pr).abs() < 1e-9, "{pr_back}");
    }

    #[test]
    fn corrected_flow_round_trips_through_actual_flow() {
        let mdot = 3.7;
        let t = 310.0;
        let p = 150_000.0;
        let corrected = corrected_flow_kg_s(mdot, t, p);
        let back = actual_flow_kg_s(corrected, t, p);
        assert!((back - mdot).abs() < 1e-9, "{back}");
    }

    #[test]
    fn corrected_flow_at_reference_conditions_equals_actual_flow() {
        let mdot = 2.5;
        assert!((corrected_flow_kg_s(mdot, T_REF_K, P_REF_PA) - mdot).abs() < 1e-9);
    }

    #[test]
    fn zero_or_negative_pressure_never_produces_nan_or_infinity() {
        let c = corrected_flow_kg_s(5.0, 300.0, 0.0);
        assert!(c.is_finite() && c == 0.0);
        let a = actual_flow_kg_s(5.0, 300.0, -1.0);
        assert!(a.is_finite());
    }

    #[test]
    fn ram_conditions_match_static_at_zero_mach() {
        assert!((total_temperature_k(288.15, 0.0, GAMMA_AIR) - 288.15).abs() < 1e-9);
        assert!((total_pressure_pa(101_325.0, 0.0, GAMMA_AIR) - 101_325.0).abs() < 1e-6);
    }

    #[test]
    fn ram_temperature_and_pressure_rise_with_mach() {
        let low = total_temperature_k(288.15, 0.1, GAMMA_AIR);
        let high = total_temperature_k(288.15, 0.5, GAMMA_AIR);
        assert!(high > low);
        let low_p = total_pressure_pa(101_325.0, 0.1, GAMMA_AIR);
        let high_p = total_pressure_pa(101_325.0, 0.5, GAMMA_AIR);
        assert!(high_p > low_p);
    }

    #[test]
    fn density_ratio_is_one_at_reference_conditions_and_falls_with_altitude() {
        assert!((density_ratio(P_REF_PA, T_REF_K) - 1.0).abs() < 1e-9);
        // A representative ~20,000 ft ISA point: ~46.6 kPa, ~248.5 K.
        let high_altitude = density_ratio(46_600.0, 248.5);
        assert!(high_altitude < 0.7 && high_altitude > 0.4, "{high_altitude}");
    }
}
