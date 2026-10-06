pub const FUEL_DENSITY_KG_M3: f64 = 800.0;

pub const FUEL_CP_J_KGK: f64 = 2010.0;

pub fn viscosity_cst(temp_k: f64) -> f64 {
    const WALTHER_A: f64 = 13.253_827_9;
    const WALTHER_B: f64 = 5.525_940_0;
    let t = temp_k.clamp(200.0, 400.0);
    10f64.powf(10f64.powf(WALTHER_A - WALTHER_B * t.log10())) - 0.7
}

pub fn vapour_pressure_pa(temp_k: f64) -> f64 {
    const T_REF_K: f64 = 293.15;
    const P_REF_PA: f64 = 300.0;
    const K_PER_K: f64 = 0.06;
    let t = temp_k.clamp(200.0, 400.0);
    P_REF_PA * (K_PER_K * (t - T_REF_K)).exp()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn viscosity_matches_its_two_anchor_points() {
        assert!((viscosity_cst(253.15) - 8.0).abs() < 0.01, "{}", viscosity_cst(253.15));
        assert!((viscosity_cst(313.15) - 1.25).abs() < 0.01, "{}", viscosity_cst(313.15));
    }

    #[test]
    fn viscosity_rises_as_fuel_cools() {
        assert!(viscosity_cst(233.15) > viscosity_cst(293.15));
    }

    #[test]
    fn vapour_pressure_rises_with_temperature_and_stays_positive() {
        assert!(vapour_pressure_pa(200.0) > 0.0);
        assert!(vapour_pressure_pa(330.0) > vapour_pressure_pa(290.0));
    }
}
