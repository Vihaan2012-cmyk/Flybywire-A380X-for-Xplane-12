pub fn fcoc_temperature_rise_k(fcoc_heat_w: f64, dt_s: f64, tank_fuel_mass_kg: f64, specific_heat_j_kgk: f64) -> f64 {
    if tank_fuel_mass_kg <= 1e-3 || specific_heat_j_kgk <= 0.0 {
        return 0.0;
    }
    (fcoc_heat_w.max(0.0) * dt_s.max(0.0)) / (tank_fuel_mass_kg * specific_heat_j_kgk)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FuelType {
    JetA,
    JetA1,
    Jp8,
}
impl FuelType {
    pub fn freeze_point_c(self) -> f64 {
        match self {
            FuelType::JetA => -40.0,
            FuelType::JetA1 => -47.0,
            FuelType::Jp8 => -47.0,
        }
    }
}

pub const WAX_ONSET_MARGIN_K: f64 = 10.0;
pub fn wax_fraction(temp_c: f64, fuel: FuelType) -> f64 {
    let freeze = fuel.freeze_point_c();
    let cloud = freeze + WAX_ONSET_MARGIN_K;
    if temp_c >= cloud {
        return 0.0;
    }
    ((cloud - temp_c) / WAX_ONSET_MARGIN_K).clamp(0.0, 1.0)
}

pub const ICE_FULL_BLOCKAGE_MARGIN_K: f64 = 5.0;
pub fn filter_ice_blockage_fraction(temp_c: f64, free_water_fraction: f64, anti_ice_heater_on: bool) -> f64 {
    if anti_ice_heater_on || temp_c >= 0.0 {
        return 0.0;
    }
    let subcool = (0.0 - temp_c).min(ICE_FULL_BLOCKAGE_MARGIN_K);
    free_water_fraction.clamp(0.0, 1.0) * (subcool / ICE_FULL_BLOCKAGE_MARGIN_K)
}

pub fn filter_blockage_fraction(wax: f64, ice: f64) -> f64 {
    (wax.clamp(0.0, 1.0) + ice.clamp(0.0, 1.0)).min(1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fcoc_heat_raises_temperature_and_needs_no_mass_to_avoid_nan() {
        let dt = fcoc_temperature_rise_k(50_000.0, 1.0, 2000.0, 2000.0);
        assert!(dt > 0.0);
        assert_eq!(fcoc_temperature_rise_k(50_000.0, 1.0, 0.0, 2000.0), 0.0);
        assert_eq!(fcoc_temperature_rise_k(50_000.0, 0.0, 2000.0, 2000.0), 0.0);
    }

    #[test]
    fn more_heat_or_more_time_raises_temperature_more() {
        let a = fcoc_temperature_rise_k(10_000.0, 1.0, 5000.0, 2000.0);
        let b = fcoc_temperature_rise_k(20_000.0, 1.0, 5000.0, 2000.0);
        assert!(b > a);
        let c = fcoc_temperature_rise_k(10_000.0, 2.0, 5000.0, 2000.0);
        assert!(c > a);
    }

    #[test]
    fn jet_a1_and_jp8_freeze_colder_than_jet_a() {
        assert_eq!(FuelType::JetA.freeze_point_c(), -40.0);
        assert_eq!(FuelType::JetA1.freeze_point_c(), -47.0);
        assert_eq!(FuelType::Jp8.freeze_point_c(), -47.0);
        assert!(FuelType::JetA1.freeze_point_c() < FuelType::JetA.freeze_point_c());
    }

    #[test]
    fn wax_forms_above_the_freeze_point_and_saturates_at_it() {
        let fuel = FuelType::JetA;
        assert_eq!(wax_fraction(0.0, fuel), 0.0, "well above cloud point");
        let partial = wax_fraction(fuel.freeze_point_c() + 5.0, fuel);
        assert!(partial > 0.0 && partial < 1.0);
        assert_eq!(wax_fraction(fuel.freeze_point_c(), fuel), 1.0);
        assert_eq!(wax_fraction(fuel.freeze_point_c() - 20.0, fuel), 1.0, "never exceeds 1");
    }

    #[test]
    fn colder_fuel_types_delay_wax_formation_at_the_same_temperature() {
        let t = -42.0;
        assert!(wax_fraction(t, FuelType::JetA) > wax_fraction(t, FuelType::JetA1));
    }

    #[test]
    fn no_ice_above_freezing_or_with_the_heater_on_or_with_no_water() {
        assert_eq!(filter_ice_blockage_fraction(5.0, 0.5, false), 0.0);
        assert_eq!(filter_ice_blockage_fraction(-10.0, 0.5, true), 0.0);
        assert_eq!(filter_ice_blockage_fraction(-10.0, 0.0, false), 0.0);
    }

    #[test]
    fn ice_blockage_grows_with_subcooling_and_water_content_and_saturates() {
        let shallow = filter_ice_blockage_fraction(-1.0, 0.5, false);
        let deep = filter_ice_blockage_fraction(-10.0, 0.5, false);
        assert!(deep > shallow);
        assert!((deep - 0.5).abs() < 1e-9, "fully sub-cooled reaches the full water fraction");
        let less_water = filter_ice_blockage_fraction(-10.0, 0.1, false);
        assert!(less_water < deep);
    }

    #[test]
    fn combined_blockage_adds_and_caps_at_one() {
        assert!((filter_blockage_fraction(0.3, 0.3) - 0.6).abs() < 1e-9);
        assert_eq!(filter_blockage_fraction(0.8, 0.8), 1.0);
        assert_eq!(filter_blockage_fraction(0.0, 0.0), 0.0);
    }
}
