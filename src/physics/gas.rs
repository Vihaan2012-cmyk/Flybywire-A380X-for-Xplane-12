//! Gas-law physics for `oxygen.rs`: ideal and van der Waals real-gas
//! pressure from mass, volume and temperature, and the diluter-demand
//! mixing an aviation regulator performs. Pure functions, unit-tested
//! without an X-Plane/FBW context.
//!
//! Sources are repeated in `docs/physics/fluids.md`.

/// Universal gas constant, J/(mol*K) (CODATA 2018, exact by SI definition).
pub const R_UNIVERSAL: f64 = 8.31446261815324;

/// Oxygen's molar mass, kg/mol (O2, standard atomic weights: 2 x 15.999 g/mol).
pub const O2_MOLAR_MASS_KG_MOL: f64 = 0.0319988;

/// Oxygen's specific gas constant, J/(kg*K): `R / M`.
pub const O2_SPECIFIC_GAS_CONSTANT: f64 = R_UNIVERSAL / O2_MOLAR_MASS_KG_MOL; // ~259.8 J/(kg K)

/// Air's specific gas constant, J/(kg*K), for cabin-air dilution mixing.
pub const AIR_SPECIFIC_GAS_CONSTANT: f64 = 287.058;

/// Ideal gas law pressure: `P = m * R_specific * T / V`. Kelvin, kilograms,
/// cubic metres in; pascals out.
pub fn ideal_gas_pressure_pa(mass_kg: f64, volume_m3: f64, temp_k: f64) -> f64 {
    if volume_m3 <= 0. {
        return 0.;
    }
    mass_kg.max(0.) * O2_SPECIFIC_GAS_CONSTANT * temp_k / volume_m3
}

/// Ideal gas law mass for a target pressure: `m = P * V / (R_specific * T)`.
pub fn ideal_gas_mass_kg(pressure_pa: f64, volume_m3: f64, temp_k: f64) -> f64 {
    if temp_k <= 0. {
        return 0.;
    }
    (pressure_pa.max(0.) * volume_m3 / (O2_SPECIFIC_GAS_CONSTANT * temp_k)).max(0.)
}

/// Van der Waals constants for O2 (standard tabulated values, e.g. any
/// physical chemistry reference table): `a` in Pa*m^6/mol^2, `b` in m^3/mol.
/// a = 1.382 L^2*bar/mol^2 = 0.1382 Pa*m^6/mol^2; b = 0.03186 L/mol =
/// 3.186e-5 m^3/mol.
const O2_VDW_A: f64 = 0.1382;
const O2_VDW_B: f64 = 3.186e-5;

/// Van der Waals real-gas pressure: `P = nRT/(V - n*b) - a*n^2/V^2`, with `n`
/// the amount of substance (mol) derived from `mass_kg`. At crew-bottle
/// densities (~165-200 kg/m^3 -- a full 1850 psi/127 bar charge) this is
/// *not* a small correction: the attraction term `a*n^2/V^2` and the
/// excluded-volume term `n*b` both matter at this density, and the two
/// together pull the real pressure about 10-12% below the ideal-gas figure
/// (verified numerically in this module's own test and in
/// `oxygen.rs`'s `ideal_and_real_gas_pressure_agree_closely_for_the_crew_bottle`
/// -- both were previously (and wrongly) asserted to agree within 5%,
/// contradicting the equation actually implemented here). The brief calls for
/// "ideal or real gas"; [`ideal_gas_pressure_pa`] is used for the crew
/// bottle's displayed pressure (matching the sourced 1850 psi *nominal*
/// full-charge figure, itself a round nominal number rather than a real-gas
/// lab reading), and this function is kept as the real-gas cross-check.
pub fn van_der_waals_pressure_pa(mass_kg: f64, volume_m3: f64, temp_k: f64) -> f64 {
    if volume_m3 <= 0. {
        return 0.;
    }
    let n = mass_kg.max(0.) / O2_MOLAR_MASS_KG_MOL; // mol
    let free_volume = (volume_m3 - n * O2_VDW_B).max(1e-9);
    (n * R_UNIVERSAL * temp_k) / free_volume - O2_VDW_A * n * n / (volume_m3 * volume_m3)
}

/// Mass flow (kg/s) corresponding to a regulator delivering
/// `standard_liters_per_min` of oxygen measured at "standard" conditions
/// (0 C, 1 atm, the usual convention for medical/aviation oxygen flow
/// meters), using the ideal gas law at those reference conditions to convert
/// volume flow to mass flow.
pub fn o2_mass_flow_kg_s(standard_liters_per_min: f64) -> f64 {
    const STANDARD_TEMP_K: f64 = 273.15;
    const STANDARD_PRESSURE_PA: f64 = 101_325.;
    let density = STANDARD_PRESSURE_PA / (O2_SPECIFIC_GAS_CONSTANT * STANDARD_TEMP_K); // kg/m3
    let m3_per_s = standard_liters_per_min.max(0.) / 1000. / 60.;
    density * m3_per_s
}

/// Ambient (dilution) oxygen partial-pressure fraction available to a
/// diluter-demand regulator at `cabin_altitude_ft`, from the International
/// Standard Atmosphere pressure ratio: as cabin altitude rises, atmospheric
/// pressure falls and a diluter-demand regulator must blend in progressively
/// more pure oxygen (and, above roughly FL340, deliver 100% / pressure-
/// breathing) to hold the same inspired oxygen partial pressure as at sea
/// level. Returns the fraction of pure oxygen (0-1) a standard
/// diluter-demand schedule would blend in, using the widely published
/// aviation physiology approximation that 100% oxygen is required from
/// about 34,000 ft (`FULL_O2_ALTITUDE_FT`) up to the cabin/pressure-breathing
/// changeover, linearly ramped from 21% (ambient air, no dilution needed) at
/// sea level. This is the standard qualitative diluter-demand schedule
/// taught in aeromedical references (e.g. FAA Airman's Information Manual/
/// AC 61-107, "physiological training"), not a specific A380 regulator
/// calibration curve (proprietary), flagged as such.
const FULL_O2_ALTITUDE_FT: f64 = 34_000.;
pub fn diluter_demand_o2_fraction(cabin_altitude_ft: f64) -> f64 {
    const SEA_LEVEL_O2_FRACTION: f64 = 0.21;
    if cabin_altitude_ft <= 0. {
        return SEA_LEVEL_O2_FRACTION;
    }
    let ratio = (cabin_altitude_ft / FULL_O2_ALTITUDE_FT).clamp(0., 1.);
    SEA_LEVEL_O2_FRACTION + (1. - SEA_LEVEL_O2_FRACTION) * ratio
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ideal_gas_round_trips_mass_and_pressure() {
        let mass = 5.0;
        let volume = 0.025;
        let temp = 288.15;
        let p = ideal_gas_pressure_pa(mass, volume, temp);
        let back = ideal_gas_mass_kg(p, volume, temp);
        assert!((back - mass).abs() < 1e-9);
    }

    #[test]
    fn ideal_gas_pressure_matches_hand_calculation() {
        // 1 kg of O2 in 1 m^3 at 0 C: P = R_specific * T = 259.8 * 273.15.
        let p = ideal_gas_pressure_pa(1.0, 1.0, 273.15);
        assert!((p - O2_SPECIFIC_GAS_CONSTANT * 273.15).abs() < 1e-6);
    }

    #[test]
    fn van_der_waals_deviates_from_ideal_by_the_expected_amount_at_bottle_density() {
        // 5 kg in 25 L is 200 kg/m^3, close to a charged crew bottle's own
        // density -- not "moderate pressure" for O2 (see this module's doc):
        // real behaviour is measurably non-ideal here, roughly 10-12% low,
        // not the few-percent deviation once (wrongly) assumed.
        let mass = 5.0;
        let volume = 0.025;
        let temp = 288.15;
        let ideal = ideal_gas_pressure_pa(mass, volume, temp);
        let real = van_der_waals_pressure_pa(mass, volume, temp);
        let rel_dev = (real - ideal).abs() / ideal;
        assert!(rel_dev > 0.08 && rel_dev < 0.15, "{rel_dev}");
    }

    #[test]
    fn mass_flow_scales_linearly_with_flow_rate() {
        let f1 = o2_mass_flow_kg_s(1.0);
        let f2 = o2_mass_flow_kg_s(2.0);
        assert!((f2 / f1 - 2.0).abs() < 1e-9);
        assert_eq!(o2_mass_flow_kg_s(0.), 0.);
        assert_eq!(o2_mass_flow_kg_s(-1.), 0.);
    }

    #[test]
    fn diluter_demand_ramps_from_ambient_to_pure_oxygen() {
        assert!((diluter_demand_o2_fraction(0.) - 0.21).abs() < 1e-9);
        assert_eq!(diluter_demand_o2_fraction(34_000.), 1.0);
        assert_eq!(diluter_demand_o2_fraction(40_000.), 1.0); // clamped
        let mid = diluter_demand_o2_fraction(17_000.);
        assert!(mid > 0.21 && mid < 1.0);
    }
}
