pub const RATED_SHAFT_POWER_W: f64 = 1_342_000.0;

pub const N_DESIGN_RPM: f64 = 60_000.0;

pub const ROTOR_INERTIA_KG_M2: f64 = 0.028;

pub const GOVERNED_N_PERCENT: f64 = 100.0;

pub const OVERSPEED_TRIP_PERCENT: f64 = 105.0;

pub const LIGHT_OFF_N_PERCENT: f64 = 12.0;

pub const SELF_SUSTAINING_N_PERCENT: f64 = 55.0;

pub const CORE_PRESSURE_RATIO_DESIGN: f64 = 4.2;
pub const CORE_COMPRESSOR_EFFICIENCY_DESIGN: f64 = 0.78;
pub const CORE_MDOT_DESIGN_KG_S: f64 = 4.0;
pub const CORE_COMPRESSOR_EFFICIENCY_FALLOFF: f64 = 0.55;
pub const CORE_SURGE_MARGIN_DESIGN_FRAC: f64 = 0.12;
pub const CORE_SURGE_LINE_FLATNESS: f64 = 0.5;
pub const CORE_CHOKE_FLOW_MULTIPLE: f64 = 1.3;

pub const LOAD_PRESSURE_RATIO_DESIGN: f64 = 3.0;
pub const LOAD_COMPRESSOR_EFFICIENCY_DESIGN: f64 = 0.75;
pub const LOAD_MDOT_DESIGN_KG_S: f64 = 1.2;
pub const LOAD_COMPRESSOR_EFFICIENCY_FALLOFF: f64 = 0.6;
pub const LOAD_SURGE_MARGIN_DESIGN_FRAC: f64 = 0.18;
pub const LOAD_SURGE_LINE_FLATNESS: f64 = 0.5;
pub const LOAD_CHOKE_FLOW_MULTIPLE: f64 = 1.3;

pub const IGV_FULL_TRAVEL_RATE_PER_S: f64 = 0.5;
pub const SCV_FULL_TRAVEL_RATE_PER_S: f64 = 1.0;
pub const SCV_SURGE_SAFETY_MARGIN_FRAC: f64 = 0.25;

pub const COMBUSTOR_EFFICIENCY: f64 = 0.98;
pub const COMBUSTOR_PRESSURE_LOSS_FRAC: f64 = 0.04;
pub const LHV_JET_A_J_KG: f64 = 43.1e6;

pub const T4_DESIGN_K: f64 = 1150.15;
pub const TURBINE_EFFICIENCY_DESIGN: f64 = 0.82;
pub const TURBINE_EFFICIENCY_FALLOFF: f64 = 0.45;
pub fn turbine_pressure_ratio_design() -> f64 {
    CORE_PRESSURE_RATIO_DESIGN * (1.0 - COMBUSTOR_PRESSURE_LOSS_FRAC)
}

pub const MAX_FUEL_FLOW_MARGIN: f64 = 1.6;

pub const EGT_START_LIMIT_C: f64 = 900.0;
pub const EGT_RUNNING_LIMIT_C: f64 = 750.0;
pub const EGT_TRIP_C: f64 = 950.0;

pub const EGT_RUNNING_WARNING_C: f64 = 900.0;
pub const EGT_START_WARNING_BELOW_FL250_C: f64 = 900.0;
pub const EGT_START_WARNING_AT_OR_ABOVE_FL250_C: f64 = 982.0;
pub const EGT_WARNING_TO_CAUTION_DIFFERENCE_C: f64 = 33.0;

pub const EGT_START_WARNING_ALTITUDE_SWITCH_FT: f64 = 25_000.0;

pub fn egt_warning_c(starting: bool, pressure_altitude_ft: f64) -> f64 {
    if starting && pressure_altitude_ft >= EGT_START_WARNING_ALTITUDE_SWITCH_FT {
        EGT_START_WARNING_AT_OR_ABOVE_FL250_C
    } else if starting {
        EGT_START_WARNING_BELOW_FL250_C
    } else {
        EGT_RUNNING_WARNING_C
    }
}

pub fn egt_caution_c(starting: bool, pressure_altitude_ft: f64) -> f64 {
    egt_warning_c(starting, pressure_altitude_ft) - EGT_WARNING_TO_CAUTION_DIFFERENCE_C
}

pub const OIL_REGULATED_PRESSURE_PSI: f64 = 60.0;
pub const OIL_REGULATION_N_PERCENT: f64 = 50.0;
pub const OIL_PRESSURE_TRIP_PSI: f64 = 15.0;
pub const OIL_PRESSURE_TRIP_DEBOUNCE_S: f64 = 5.0;
pub const OIL_HEAT_CAPACITY_J_K: f64 = 9_000.0;
pub const OIL_TIME_CONSTANT_S: f64 = 150.0;
pub const OIL_TANK_CAPACITY_L: f64 = 8.0;
pub const OIL_LOW_LEVEL_TRIP_L: f64 = 2.0;

pub const BATTERY_NOMINAL_OPEN_CIRCUIT_V: f64 = 24.0;
pub const BATTERY_INTERNAL_RESISTANCE_OHM: f64 = 0.018;
pub const STARTER_ARMATURE_RESISTANCE_OHM: f64 = 0.022;
pub const STARTER_KE_KT: f64 = 0.016;

pub const GENERATOR_RATED_APPARENT_VA: f64 = 120_000.0;
pub const GENERATOR_RATED_POWER_FACTOR: f64 = 0.8;
pub const GENERATOR_EFFICIENCY_DESIGN: f64 = 0.88;

pub const FUEL_METERING_VALVE_RATE_PER_S: f64 = 2.0;
pub const INLET_DOOR_RATE_PER_S: f64 = 0.1;

pub const FIXED_ACCESSORY_POWER_W: f64 = 3_000.0;
pub const N_SENSOR_MAX_BIAS_PERCENT: f64 = 20.0;
pub const EGT_SENSOR_MAX_BIAS_C: f64 = 150.0;

pub const MIN_RELIGHT_DENSITY_RATIO: f64 = 0.53;
pub const SCOOP_RAM_COUPLING_FRAC: f64 = 0.3;
pub const WINDMILL_TORQUE_COEFF_NM_PER_PA: f64 = 0.02;

pub const OIL_VISCOSITY_WALTHER_A: f64 = 9.3116;
pub const OIL_VISCOSITY_WALTHER_B: f64 = 3.6661;
pub fn omega_rated_rad_s() -> f64 {
    N_DESIGN_RPM * std::f64::consts::TAU / 60.0
}

pub fn oil_viscous_drag_torque_hot_rated_nm() -> f64 {
    FIXED_ACCESSORY_POWER_W / omega_rated_rad_s()
}

pub const OIL_VISCOUS_DRAG_EXPONENT: f64 = 2.0 / 3.0;
pub const OIL_HOT_REFERENCE_K: f64 = 353.15;

pub const COMPRESSOR_WEAR_PER_HOUR: f64 = 0.00006;
pub const COMPRESSOR_WEAR_PER_START: f64 = 0.00015;
pub const TURBINE_WEAR_PER_HOUR: f64 = 0.00008;
pub const CREEP_RATE_PER_HOUR_AT_DESIGN_EGT: f64 = 0.00004;
pub const CREEP_REFERENCE_EGT_C: f64 = 600.0;
pub const CREEP_DOUBLING_C: f64 = 25.0;
pub const CREEP_THRESHOLD_EGT_C: f64 = 500.0;

pub const STARTER_DUTY_LIMIT_S: f64 = 60.0;
pub const STARTER_COOLDOWN_TIME_CONSTANT_S: f64 = 180.0;

pub const OIL_PRESSURE_SENSOR_MAX_BIAS_PSI: f64 = 20.0;
pub const ECB_OVERSPEED_DEBOUNCE_S: f64 = 0.2;
pub const ECB_EGT_DEBOUNCE_S: f64 = 1.0;

pub const IGV_LOAD_SHED_START_FRAC: f64 = 0.7;
pub const IGV_LOAD_SHED_FULL_FRAC: f64 = 1.1;
pub const IGV_LOAD_SHED_FLOOR: f64 = 0.3;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn turbine_pressure_ratio_design_is_below_the_compressor_pressure_ratio() {
        assert!(turbine_pressure_ratio_design() < CORE_PRESSURE_RATIO_DESIGN);
        assert!(turbine_pressure_ratio_design() > 1.0);
    }

    #[test]
    fn rated_power_is_in_a_physically_plausible_band_for_this_spool_speed() {
        let omega_rated = N_DESIGN_RPM * std::f64::consts::TAU / 60.0;
        let torque_at_rated_power = RATED_SHAFT_POWER_W / omega_rated;
        assert!(torque_at_rated_power > 10.0 && torque_at_rated_power < 5000.0);
    }
}

#[cfg(test)]
mod egt_limit_tests {
    use super::*;

    #[test]
    fn the_three_kinds_of_egt_limit_are_ordered_the_way_the_ecb_orders_them() {
        assert!(EGT_RUNNING_LIMIT_C < egt_caution_c(false, 0.0));
        assert!(egt_caution_c(false, 0.0) < EGT_RUNNING_WARNING_C);
        assert!(EGT_RUNNING_WARNING_C < EGT_TRIP_C);
        assert!(EGT_START_LIMIT_C <= EGT_START_WARNING_BELOW_FL250_C);
        assert!(EGT_START_LIMIT_C < EGT_TRIP_C);
    }

    #[test]
    fn the_start_warning_rises_above_fl250_and_only_while_starting() {
        assert_eq!(egt_warning_c(true, 0.0), EGT_START_WARNING_BELOW_FL250_C);
        assert_eq!(egt_warning_c(true, 24_999.0), EGT_START_WARNING_BELOW_FL250_C);
        assert_eq!(egt_warning_c(true, 25_000.0), EGT_START_WARNING_AT_OR_ABOVE_FL250_C);
        assert_eq!(egt_warning_c(true, 39_000.0), EGT_START_WARNING_AT_OR_ABOVE_FL250_C);
        assert_eq!(egt_warning_c(false, 0.0), EGT_RUNNING_WARNING_C);
        assert_eq!(egt_warning_c(false, 39_000.0), EGT_RUNNING_WARNING_C);
    }

    #[test]
    fn caution_tracks_warning_by_the_ecbs_own_fixed_difference() {
        for &(starting, alt) in &[(true, 0.0), (true, 30_000.0), (false, 0.0), (false, 30_000.0)] {
            let gap = egt_warning_c(starting, alt) - egt_caution_c(starting, alt);
            assert!((gap - EGT_WARNING_TO_CAUTION_DIFFERENCE_C).abs() < 1e-12);
        }
        assert!((egt_caution_c(false, 0.0) - 867.0).abs() < 1e-12);
    }

    #[test]
    fn the_pw980a_running_warning_is_not_the_a320_figure() {
        assert!(EGT_RUNNING_WARNING_C > 682.0 + 100.0);
    }
}
