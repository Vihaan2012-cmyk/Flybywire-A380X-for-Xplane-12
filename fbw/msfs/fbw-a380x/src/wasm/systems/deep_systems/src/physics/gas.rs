pub const R_UNIVERSAL: f64 = 8.31446261815324;

pub const O2_MOLAR_MASS_KG_MOL: f64 = 0.0319988;

pub const O2_SPECIFIC_GAS_CONSTANT: f64 = R_UNIVERSAL / O2_MOLAR_MASS_KG_MOL;

pub const AIR_SPECIFIC_GAS_CONSTANT: f64 = 287.058;

pub fn ideal_gas_pressure_pa(mass_kg: f64, volume_m3: f64, temp_k: f64) -> f64 {
    if volume_m3 <= 0. {
        return 0.;
    }
    mass_kg.max(0.) * O2_SPECIFIC_GAS_CONSTANT * temp_k / volume_m3
}

pub fn ideal_gas_mass_kg(pressure_pa: f64, volume_m3: f64, temp_k: f64) -> f64 {
    if temp_k <= 0. {
        return 0.;
    }
    (pressure_pa.max(0.) * volume_m3 / (O2_SPECIFIC_GAS_CONSTANT * temp_k)).max(0.)
}

const O2_VDW_A: f64 = 0.1382;
const O2_VDW_B: f64 = 3.186e-5;

pub fn van_der_waals_pressure_pa(mass_kg: f64, volume_m3: f64, temp_k: f64) -> f64 {
    if volume_m3 <= 0. {
        return 0.;
    }
    let n = mass_kg.max(0.) / O2_MOLAR_MASS_KG_MOL;
    let free_volume = (volume_m3 - n * O2_VDW_B).max(1e-9);
    (n * R_UNIVERSAL * temp_k) / free_volume - O2_VDW_A * n * n / (volume_m3 * volume_m3)
}

pub fn o2_mass_flow_kg_s(standard_liters_per_min: f64) -> f64 {
    const STANDARD_TEMP_K: f64 = 273.15;
    const STANDARD_PRESSURE_PA: f64 = 101_325.;
    let density = STANDARD_PRESSURE_PA / (O2_SPECIFIC_GAS_CONSTANT * STANDARD_TEMP_K);
    let m3_per_s = standard_liters_per_min.max(0.) / 1000. / 60.;
    density * m3_per_s
}

const FULL_O2_ALTITUDE_FT: f64 = 34_000.;

fn full_o2_ramp_ratio(cabin_altitude_ft: f64) -> f64 {
    (cabin_altitude_ft.max(0.) / FULL_O2_ALTITUDE_FT).clamp(0., 1.)
}

pub fn diluter_demand_o2_fraction(cabin_altitude_ft: f64) -> f64 {
    const SEA_LEVEL_O2_FRACTION: f64 = 0.21;
    if cabin_altitude_ft <= 0. {
        return SEA_LEVEL_O2_FRACTION;
    }
    SEA_LEVEL_O2_FRACTION + (1. - SEA_LEVEL_O2_FRACTION) * full_o2_ramp_ratio(cabin_altitude_ft)
}

pub fn diluter_demand_protection_fraction(cabin_altitude_ft: f64) -> f64 {
    full_o2_ramp_ratio(cabin_altitude_ft)
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
        let p = ideal_gas_pressure_pa(1.0, 1.0, 273.15);
        assert!((p - O2_SPECIFIC_GAS_CONSTANT * 273.15).abs() < 1e-6);
    }

    #[test]
    fn van_der_waals_deviates_from_ideal_by_the_expected_amount_at_bottle_density() {
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
        assert_eq!(diluter_demand_o2_fraction(40_000.), 1.0);
        let mid = diluter_demand_o2_fraction(17_000.);
        assert!(mid > 0.21 && mid < 1.0);
    }

    #[test]
    fn diluter_demand_protection_ramps_from_none_to_full() {
        assert_eq!(diluter_demand_protection_fraction(0.), 0.);
        assert_eq!(diluter_demand_protection_fraction(34_000.), 1.0);
        assert_eq!(diluter_demand_protection_fraction(40_000.), 1.0);
        let mid = diluter_demand_protection_fraction(17_000.);
        assert!(mid > 0. && mid < 1.0);
    }
}
