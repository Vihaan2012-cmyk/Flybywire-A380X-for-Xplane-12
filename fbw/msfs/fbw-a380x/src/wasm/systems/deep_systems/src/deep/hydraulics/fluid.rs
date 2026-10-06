pub const FLUID_DENSITY_KG_M3_AT_25C: f64 = 1009.0;
const FLUID_DENSITY_REF_TEMP_C: f64 = 25.0;

pub const FLUID_THERMAL_EXPANSION_PER_K: f64 = 7.0e-4;

pub fn density_kg_m3(temp_c: f64) -> f64 {
    FLUID_DENSITY_KG_M3_AT_25C / (1.0 + FLUID_THERMAL_EXPANSION_PER_K * (temp_c - FLUID_DENSITY_REF_TEMP_C))
}

const VISC_REF1_TEMP_K: f64 = 311.15;
const VISC_REF1_CST: f64 = 11.15;
const VISC_REF2_TEMP_K: f64 = 372.15;
const VISC_REF2_CST: f64 = 3.83;

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

pub fn dynamic_viscosity_pa_s(temp_c: f64) -> f64 {
    viscosity_cst(temp_c) * 1.0e-6 * density_kg_m3(temp_c)
}

pub const PSI_PA: f64 = 6894.757;
pub const FLUID_BULK_MODULUS_PA: f64 = 221_000.0 * PSI_PA;

pub const ATM_PA: f64 = 101_325.0;

pub fn effective_bulk_modulus_pa(pressure_pa: f64, air_fraction_at_1atm: f64) -> f64 {
    const POLYTROPIC_N: f64 = 1.2;
    let p = pressure_pa.max(ATM_PA * 0.05);
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
        let air = 0.01;
        let beta_low = effective_bulk_modulus_pa(50.0 * PSI_PA, air);
        let beta_high = effective_bulk_modulus_pa(5000.0 * PSI_PA, air);
        let beta_clean_high = effective_bulk_modulus_pa(5000.0 * PSI_PA, 0.0);
        assert!(beta_low < beta_high, "low pressure should be far spongier with air entrained");
        assert!(beta_high > 0.9 * beta_clean_high);
        assert!(beta_low < 0.5 * beta_clean_high);
    }

    #[test]
    fn bulk_modulus_never_zero_or_negative_at_extreme_inputs() {
        let beta = effective_bulk_modulus_pa(0.0, 5.0);
        assert!(beta > 0.0 && beta.is_finite());
    }
}
