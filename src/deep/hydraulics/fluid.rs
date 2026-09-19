//! Hydraulic fluid properties for the A380's green/yellow 5000 psi systems.
//!
//! Airbus large-aircraft hydraulics use a fire-resistant phosphate-ester
//! fluid (Skydrol-type, e.g. Skydrol LD-4 / Solutia-Eastman), never a mineral
//! oil, because a mineral-oil leak sprayed near a hot engine/APU bleed duct
//! at 5000 psi is a fire hazard (the reason FAA/EASA transport-category
//! hydraulics use fire-resistant fluid at all, see FAA AC 20-43). No A380
//! fluid grade is published; Skydrol LD-4's own technical bulletin (Eastman
//! Pub. No. 7249153C, "LD-4 / 500B-4 Technical Bulletin", the fluid family
//! this class of aircraft uses) gives the reference numbers below, used as
//! representative of the type rather than a confirmed A380 spec.

/// Density at the datasheet's reference temperature (25 C): relative density
/// 1.004-1.014 (Skydrol LD-4 technical bulletin); midpoint used, kg/m^3.
pub const FLUID_DENSITY_KG_M3_AT_25C: f64 = 1009.0;
const FLUID_DENSITY_REF_TEMP_C: f64 = 25.0;

/// Thermal expansion coefficient: no published curve for this fluid family;
/// GENERIC, taken by analogy with the same order of magnitude as other
/// petroleum/ester-based aviation fluids (this plugin's own Jet A figure,
/// `physics::fluids::JET_A_THERMAL_EXPANSION_PER_K`, is 9.0e-4/K), scaled
/// down slightly for a denser ester base stock.
pub const FLUID_THERMAL_EXPANSION_PER_K: f64 = 7.0e-4;

/// Fluid density at `temp_c`, from the reference density and linear thermal
/// expansion: `rho(T) = rho_ref / (1 + beta*(T - T_ref))`.
pub fn density_kg_m3(temp_c: f64) -> f64 {
    FLUID_DENSITY_KG_M3_AT_25C / (1.0 + FLUID_THERMAL_EXPANSION_PER_K * (temp_c - FLUID_DENSITY_REF_TEMP_C))
}

/// Two published kinematic viscosity points from the same Skydrol LD-4
/// bulletin: 11.15 mm^2/s (cSt) at 38 C and 3.83 cSt at 99 C. Fitted with the
/// same Walther (ASTM D341) log-log form already used in this crate
/// (`physics::engine::oil::viscosity_cst`, Mobil Jet Oil II) rather than
/// re-deriving a different method, since a two-point Walther fit is the
/// standard method for this kind of fluid.
const VISC_REF1_TEMP_K: f64 = 311.15; // 38 C
const VISC_REF1_CST: f64 = 11.15;
const VISC_REF2_TEMP_K: f64 = 372.15; // 99 C
const VISC_REF2_CST: f64 = 3.83;

/// Kinematic viscosity (cSt) at `temp_c`.
pub fn viscosity_cst(temp_c: f64) -> f64 {
    let t = (temp_c + 273.15).clamp(180.0, 450.0);
    let z = |v: f64| (v + 0.7).log10().log10();
    let (z1, z2) = (z(VISC_REF1_CST), z(VISC_REF2_CST));
    let (t1, t2) = (VISC_REF1_TEMP_K.log10(), VISC_REF2_TEMP_K.log10());
    let b = (z1 - z2) / (t2 - t1);
    let a = z1 + b * t1;
    let zt = a - b * t.log10();
    (10f64.powf(10f64.powf(zt))) - 0.7
}

/// Dynamic viscosity, Pa*s, from kinematic viscosity and density
/// (`mu = nu * rho`, `nu` converted from cSt = mm^2/s = 1e-6 m^2/s).
pub fn dynamic_viscosity_pa_s(temp_c: f64) -> f64 {
    viscosity_cst(temp_c) * 1.0e-6 * density_kg_m3(temp_c)
}

/// Isothermal secant bulk modulus at the datasheet's reference condition:
/// 221,000 psi (Skydrol LD-4 technical bulletin), converted to Pa. No public
/// temperature curve exists for this fluid family; the base modulus is held
/// constant with temperature here (a secondary effect next to the entrained
/// air correction below, which is the dominant source of "spongy" low
/// pressure behaviour this model needs to reproduce).
pub const PSI_PA: f64 = 6894.757;
pub const FLUID_BULK_MODULUS_PA: f64 = 221_000.0 * PSI_PA;

/// Standard atmosphere, Pa: the absolute-pressure reference the entrained
/// air correction and the network solver's gauge-to-absolute conversion
/// both use.
pub const ATM_PA: f64 = 101_325.0;

/// Effective bulk modulus of the fluid with undissolved (entrained/free) air
/// mixed in, the classic two-phase mixture relation (Merritt, "Hydraulic
/// Control Systems", 1967, ch. 2; also e.g. Manring, "Hydraulic Control
/// Systems"): a gas/liquid mixture's compliance is the volume-weighted sum
/// of each phase's own compliance,
/// `1/Beta_eff = (1-x)/Beta_fluid + x/(n*P)`,
/// where `x` is the *local* entrained air volume fraction at pressure `p_pa`
/// and `n` the air's polytropic exponent for the fast compression/expansion
/// a pressure transient imposes (n=1.0 isothermal .. 1.4 adiabatic for air;
/// 1.2 used as a mid-range value, matching the accumulator model's own
/// default in `accumulator.rs`).
///
/// `air_fraction_at_1atm` is the volume of free air per volume of fluid at
/// atmospheric pressure (an entrained-air *content*, not itself pressure
/// dependent); the air actually occupies less volume at working pressure
/// (Boyle's law, `x(P) = air_fraction_at_1atm * P_atm / P`), which is why a
/// healthy system with a small air content still runs stiff at 5000 psi but
/// turns spongy the moment pressure drops (a reservoir running low draws in
/// air at the pump inlet, `reservoir.rs`'s cavitation/air-ingestion fault).
pub fn effective_bulk_modulus_pa(pressure_pa: f64, air_fraction_at_1atm: f64) -> f64 {
    const POLYTROPIC_N: f64 = 1.2;
    let p = pressure_pa.max(ATM_PA * 0.05); // never fall to a non-physical near-zero absolute pressure
    let x = (air_fraction_at_1atm.max(0.0) * ATM_PA / p).min(0.9);
    let beta_fluid = FLUID_BULK_MODULUS_PA;
    let compliance = (1.0 - x) / beta_fluid + x / (POLYTROPIC_N * p);
    if compliance <= 0.0 {
        beta_fluid
    } else {
        1.0 / compliance
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn density_falls_with_temperature() {
        let cold = density_kg_m3(-20.0);
        let warm = density_kg_m3(80.0);
        assert!(cold > warm);
        assert!((density_kg_m3(25.0) - FLUID_DENSITY_KG_M3_AT_25C).abs() < 1e-9);
    }

    #[test]
    fn viscosity_matches_its_two_datasheet_points() {
        assert!((viscosity_cst(38.0) - VISC_REF1_CST).abs() < 1e-6);
        assert!((viscosity_cst(99.0) - VISC_REF2_CST).abs() < 1e-6);
    }

    #[test]
    fn viscosity_stays_under_the_datasheet_low_temperature_ceiling() {
        // Datasheet: "< 2000 cSt at -54 C".
        assert!(viscosity_cst(-54.0) < 2000.0);
        assert!(viscosity_cst(-54.0) > viscosity_cst(38.0));
    }

    #[test]
    fn dynamic_viscosity_is_positive_and_scales_with_density() {
        let mu = dynamic_viscosity_pa_s(38.0);
        assert!(mu > 0.0);
        assert!((mu - viscosity_cst(38.0) * 1e-6 * density_kg_m3(38.0)).abs() < 1e-12);
    }

    #[test]
    fn bulk_modulus_matches_the_datasheet_with_no_entrained_air() {
        let beta = effective_bulk_modulus_pa(5000.0 * PSI_PA, 0.0);
        assert!((beta - FLUID_BULK_MODULUS_PA).abs() < 1.0);
    }

    #[test]
    fn entrained_air_softens_the_fluid_much_more_at_low_pressure() {
        let air = 0.01; // 1% free air at atmospheric, a contaminated/aerated system
        let beta_low = effective_bulk_modulus_pa(50.0 * PSI_PA, air);
        let beta_high = effective_bulk_modulus_pa(5000.0 * PSI_PA, air);
        let beta_clean_high = effective_bulk_modulus_pa(5000.0 * PSI_PA, 0.0);
        assert!(beta_low < beta_high, "low pressure should be far spongier with air entrained");
        // At 5000 psi even 1% free-at-1atm air has compressed to a tiny
        // fraction, so it barely dents the clean-fluid modulus.
        assert!(beta_high > 0.9 * beta_clean_high);
        // But at 50 psi the same air content is a large fraction of volume,
        // and dominates the mixture's compliance.
        assert!(beta_low < 0.5 * beta_clean_high);
    }

    #[test]
    fn bulk_modulus_never_zero_or_negative_at_extreme_inputs() {
        let beta = effective_bulk_modulus_pa(0.0, 5.0);
        assert!(beta > 0.0 && beta.is_finite());
    }
}
