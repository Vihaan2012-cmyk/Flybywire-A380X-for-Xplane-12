use super::gas;

pub const ALVEOLAR_WATER_VAPOUR_PA: f64 = 6266.2;
pub const ALVEOLAR_CO2_PA: f64 = 5332.9;
pub const RESPIRATORY_QUOTIENT: f64 = 0.8;
pub const BODY_TEMP_K: f64 = 310.15;

pub const RESTING_MINUTE_VOLUME_L_PER_MIN: f64 = 8.0;

pub const EMERGENCY_SEAL_LEAK_L_PER_MIN: f64 = 5.0;

pub const DISTRIBUTION_SETPOINT_GAUGE_PA: f64 = 85.0 * gas::PSI_TO_PA;

pub const REGULATOR_DROPOUT_MARGIN_PA: f64 = 5.0 * gas::PSI_TO_PA;

pub const REGULATOR_DROOP_PA_PER_KG_S: f64 = 1.5e8;

pub const LOW_PRESSURE_RELIEF_MULTIPLE: f64 = 1.5;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum MaskMode {
    #[default]
    Normal,
    Pure,
    Emergency,
}

pub fn sea_level_alveolar_po2_pa() -> f64 {
    (101_325.0 - ALVEOLAR_WATER_VAPOUR_PA) * gas::AIR_O2_MOLE_FRACTION - ALVEOLAR_CO2_PA / RESPIRATORY_QUOTIENT
}

pub fn diluter_demand_o2_fraction(cabin_pressure_pa: f64) -> f64 {
    let dry = cabin_pressure_pa - ALVEOLAR_WATER_VAPOUR_PA;
    if dry <= 0.0 {
        return 1.0;
    }
    let required = (sea_level_alveolar_po2_pa() + ALVEOLAR_CO2_PA / RESPIRATORY_QUOTIENT) / dry;
    required.clamp(gas::AIR_O2_MOLE_FRACTION, 1.0)
}

pub fn bottle_draw_fraction(o2_fraction: f64) -> f64 {
    let f_air = gas::AIR_O2_MOLE_FRACTION;
    ((o2_fraction.clamp(0.0, 1.0) - f_air) / (1.0 - f_air)).clamp(0.0, 1.0)
}

pub fn delivered_o2_fraction(mode: MaskMode, cabin_pressure_pa: f64, dilution_stuck_ambient: f64) -> f64 {
    let healthy = match mode {
        MaskMode::Normal => diluter_demand_o2_fraction(cabin_pressure_pa),
        MaskMode::Pure | MaskMode::Emergency => 1.0,
    };
    let stuck = dilution_stuck_ambient.clamp(0.0, 1.0);
    healthy * (1.0 - stuck) + gas::AIR_O2_MOLE_FRACTION * stuck
}

pub fn mask_demand_kg_s(mode: MaskMode, cabin_pressure_pa: f64, delivered_fraction: f64, minute_volume_l_per_min: f64) -> f64 {
    if !(cabin_pressure_pa > 0.0) {
        return 0.0;
    }
    let seal_leak = if mode == MaskMode::Emergency { EMERGENCY_SEAL_LEAK_L_PER_MIN } else { 0.0 };
    let volume_m3_s = (minute_volume_l_per_min.max(0.0) + seal_leak) * 1e-3 / 60.0;
    let density = gas::delivered_density_kg_m3(cabin_pressure_pa, BODY_TEMP_K);
    let draw = bottle_draw_fraction(delivered_fraction);
    let share = match mode {
        MaskMode::Normal => draw,
        MaskMode::Pure | MaskMode::Emergency => delivered_fraction.clamp(0.0, 1.0),
    };
    volume_m3_s * density * share
}

#[derive(Clone, Copy, Debug)]
pub struct PressureRegulator {
    pub setpoint_gauge_pa: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RegulatorFaults {
    pub setpoint_shift: f64,
    pub seat_leak: f64,
}

pub const SEAT_LEAK_FULL_SCALE_M2: f64 = 1e-8;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RegulatorOutputs {
    pub outlet_gauge_pa: f64,
    pub relief_lifted: bool,
    pub dropped_out: bool,
}

impl PressureRegulator {
    pub fn new(setpoint_gauge_pa: f64) -> Self {
        Self { setpoint_gauge_pa }
    }

    pub fn step(&self, inlet_gauge_pa: f64, flow_kg_s: f64, faults: RegulatorFaults) -> RegulatorOutputs {
        let shifted = self.setpoint_gauge_pa * (1.0 + faults.setpoint_shift.clamp(-1.0, 1.0));
        let droop = REGULATOR_DROOP_PA_PER_KG_S * flow_kg_s.max(0.0);
        let commanded = (shifted - droop).max(0.0);
        let ceiling = (inlet_gauge_pa - REGULATOR_DROPOUT_MARGIN_PA).max(0.0);
        let dropped_out = commanded > ceiling;
        let mut outlet = commanded.min(ceiling);
        let relief_at = self.setpoint_gauge_pa * LOW_PRESSURE_RELIEF_MULTIPLE;
        let relief_lifted = outlet > relief_at;
        if relief_lifted {
            outlet = relief_at;
        }
        RegulatorOutputs { outlet_gauge_pa: outlet, relief_lifted, dropped_out }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_alveolar_target_agrees_with_the_textbook_figure() {
        let quoted = 103.0 * 133.322_387_415;
        let ours = sea_level_alveolar_po2_pa();
        assert!((ours - quoted).abs() < 800.0, "computed {ours} Pa vs quoted {quoted} Pa");
    }

    #[test]
    fn the_pure_oxygen_crossover_lands_where_the_published_figure_is() {
        let mut crossover_ft = 0.0;
        for ft in 20_000..45_000 {
            let p = 101_325.0 * (1.0 - ft as f64 * 0.3048 / 44_330.77).powf(1.0 / 0.190_263_1);
            if diluter_demand_o2_fraction(p) >= 1.0 {
                crossover_ft = ft as f64;
                break;
            }
        }
        assert!(crossover_ft > 31_000.0 && crossover_ft < 36_000.0, "crossover at {crossover_ft} ft");
    }

    #[test]
    fn a_diluter_demand_mask_draws_nothing_at_all_on_the_ground() {
        let f = diluter_demand_o2_fraction(101_325.0);
        assert!((f - gas::AIR_O2_MOLE_FRACTION).abs() < 1e-12, "{f}");
        assert!(bottle_draw_fraction(f) < 1e-12);
        assert!(mask_demand_kg_s(MaskMode::Normal, 101_325.0, f, RESTING_MINUTE_VOLUME_L_PER_MIN) < 1e-15);
        assert!(mask_demand_kg_s(MaskMode::Pure, 101_325.0, 1.0, RESTING_MINUTE_VOLUME_L_PER_MIN) > 0.0);
    }

    #[test]
    fn the_draw_rises_with_cabin_altitude_and_tops_out_at_pure_oxygen() {
        let mut last = -1.0;
        for p in [101_325.0, 75_262.0, 57_182.0, 46_563.0, 35_600.0, 26_000.0] {
            let f = diluter_demand_o2_fraction(p);
            let d = mask_demand_kg_s(MaskMode::Normal, p, f, RESTING_MINUTE_VOLUME_L_PER_MIN);
            assert!(f >= gas::AIR_O2_MOLE_FRACTION && f <= 1.0);
            assert!(d > last, "draw should climb with cabin altitude: {d} after {last} at {p} Pa");
            last = d;
        }
        assert_eq!(diluter_demand_o2_fraction(10_000.0), 1.0);
        let very_high = mask_demand_kg_s(MaskMode::Normal, 18_750.0, 1.0, RESTING_MINUTE_VOLUME_L_PER_MIN);
        assert!(very_high < last, "{very_high} vs {last}");
    }

    #[test]
    fn a_crew_bottle_lasts_a_realistic_number_of_hours_at_a_depressurised_cruise() {
        let p = 23_842.0;
        let f = diluter_demand_o2_fraction(p);
        let per_mask = mask_demand_kg_s(MaskMode::Normal, p, f, RESTING_MINUTE_VOLUME_L_PER_MIN);
        let total = 4.0 * per_mask;
        let supply_kg = gas::mass_from_free_air_kg(6520.0, 294.15);
        let hours = supply_kg / total / 3600.0;
        assert!(hours > 5.0 && hours < 40.0, "{hours} hours");
    }

    #[test]
    fn a_stuck_diluter_delivers_cabin_air_at_any_altitude() {
        let healthy = delivered_o2_fraction(MaskMode::Normal, 18_750.0, 0.0);
        let stuck = delivered_o2_fraction(MaskMode::Normal, 18_750.0, 1.0);
        assert_eq!(healthy, 1.0);
        assert!((stuck - gas::AIR_O2_MOLE_FRACTION).abs() < 1e-12, "{stuck}");
        assert_eq!(mask_demand_kg_s(MaskMode::Normal, 18_750.0, stuck, RESTING_MINUTE_VOLUME_L_PER_MIN), 0.0);
        let half = delivered_o2_fraction(MaskMode::Pure, 18_750.0, 0.5);
        assert!(half > gas::AIR_O2_MOLE_FRACTION && half < 1.0, "{half}");
    }

    #[test]
    fn emergency_costs_more_than_pure_which_costs_more_than_normal() {
        let p = 57_182.0;
        let f = diluter_demand_o2_fraction(p);
        let normal = mask_demand_kg_s(MaskMode::Normal, p, f, RESTING_MINUTE_VOLUME_L_PER_MIN);
        let pure = mask_demand_kg_s(MaskMode::Pure, p, 1.0, RESTING_MINUTE_VOLUME_L_PER_MIN);
        let emer = mask_demand_kg_s(MaskMode::Emergency, p, 1.0, RESTING_MINUTE_VOLUME_L_PER_MIN);
        assert!(normal < pure && pure < emer, "{normal} {pure} {emer}");
    }

    #[test]
    fn the_reducer_holds_its_setpoint_until_the_bottle_nearly_gives_out() {
        let r = PressureRegulator::new(DISTRIBUTION_SETPOINT_GAUGE_PA);
        let full = r.step(1850.0 * gas::PSI_TO_PA, 0.0, RegulatorFaults::default());
        assert!((full.outlet_gauge_pa - DISTRIBUTION_SETPOINT_GAUGE_PA).abs() < 1.0);
        assert!(!full.dropped_out && !full.relief_lifted);
        let low = r.step(100.0 * gas::PSI_TO_PA, 0.0, RegulatorFaults::default());
        assert!((low.outlet_gauge_pa - DISTRIBUTION_SETPOINT_GAUGE_PA).abs() < 1.0);
        let empty = r.step(50.0 * gas::PSI_TO_PA, 0.0, RegulatorFaults::default());
        assert!(empty.dropped_out);
        assert!(empty.outlet_gauge_pa < DISTRIBUTION_SETPOINT_GAUGE_PA);
        assert!(empty.outlet_gauge_pa > 0.0);
    }

    #[test]
    fn a_shifted_setpoint_starves_the_masks_or_lifts_the_relief() {
        let r = PressureRegulator::new(DISTRIBUTION_SETPOINT_GAUGE_PA);
        let inlet = 1850.0 * gas::PSI_TO_PA;
        let low = r.step(inlet, 0.0, RegulatorFaults { setpoint_shift: -0.8, ..Default::default() });
        assert!(low.outlet_gauge_pa < 0.3 * DISTRIBUTION_SETPOINT_GAUGE_PA, "{}", low.outlet_gauge_pa);
        let high = r.step(inlet, 0.0, RegulatorFaults { setpoint_shift: 1.0, ..Default::default() });
        assert!(high.relief_lifted);
        assert!((high.outlet_gauge_pa - DISTRIBUTION_SETPOINT_GAUGE_PA * LOW_PRESSURE_RELIEF_MULTIPLE).abs() < 1.0);
    }

    #[test]
    fn nothing_here_divides_by_zero() {
        assert!(diluter_demand_o2_fraction(0.0).is_finite());
        assert_eq!(mask_demand_kg_s(MaskMode::Normal, 0.0, 1.0, 8.0), 0.0);
        let r = PressureRegulator::new(DISTRIBUTION_SETPOINT_GAUGE_PA);
        let out = r.step(0.0, 0.0, RegulatorFaults::default());
        assert_eq!(out.outlet_gauge_pa, 0.0);
        assert!(out.dropped_out);
    }
}
