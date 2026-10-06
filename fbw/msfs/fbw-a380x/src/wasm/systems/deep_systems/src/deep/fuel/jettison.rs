use super::geometry::G;

pub fn orifice_flow_m3_s(cda_m2: f64, delta_pressure_pa: f64, density_kg_m3: f64) -> f64 {
    if delta_pressure_pa <= 0.0 || density_kg_m3 <= 0.0 || cda_m2 <= 0.0 {
        return 0.0;
    }
    cda_m2 * (2.0 * delta_pressure_pa / density_kg_m3).sqrt()
}

pub fn head_pressure_pa(liquid_depth_m: f64, density_kg_m3: f64) -> f64 {
    (liquid_depth_m.max(0.0)) * density_kg_m3.max(0.0) * G
}

#[derive(Clone, Copy, Debug, Default)]
pub struct JettisonValve {
    pub position: f64,
    stuck_at: Option<f64>,
    was_stuck: bool,
}
impl JettisonValve {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn step(&mut self, commanded_open: bool, opening_time_s: f64, stuck_fraction: f64, dt_s: f64) {
        let stuck_fraction = stuck_fraction.clamp(0.0, 1.0);
        let is_stuck = stuck_fraction > 0.0;
        if is_stuck && !self.was_stuck {
            self.stuck_at = Some(self.position);
        }
        if !is_stuck {
            self.stuck_at = None;
        }
        self.was_stuck = is_stuck;
        let commanded_target = if commanded_open { 1.0 } else { 0.0 };
        let target = match self.stuck_at {
            Some(stuck) => stuck + (commanded_target - stuck) * (1.0 - stuck_fraction),
            None => commanded_target,
        };
        if opening_time_s <= 0.0 {
            self.position = target.clamp(0.0, 1.0);
            return;
        }
        let max_step = (1.0 / opening_time_s) * dt_s.max(0.0);
        let diff = (target - self.position).clamp(-max_step, max_step);
        self.position = (self.position + diff).clamp(0.0, 1.0);
    }
}

pub fn effective_cda_m2(nominal_cda_m2: f64, blockage_fraction: f64, valve_position: f64) -> f64 {
    (nominal_cda_m2.max(0.0) * (1.0 - blockage_fraction.clamp(0.0, 1.0)) * valve_position.clamp(0.0, 1.0)).max(0.0)
}

pub fn jettison_mass_flow_kg_s(
    cda_m2: f64,
    liquid_depth_m: f64,
    boost_pump_pressure_pa: f64,
    ullage_pressure_pa: f64,
    nozzle_exit_pressure_pa: f64,
    density_kg_m3: f64,
) -> f64 {
    let drive_pressure = ullage_pressure_pa.max(0.0) + head_pressure_pa(liquid_depth_m, density_kg_m3) + boost_pump_pressure_pa.max(0.0);
    let delta_p = drive_pressure - nozzle_exit_pressure_pa.max(0.0);
    orifice_flow_m3_s(cda_m2, delta_p, density_kg_m3) * density_kg_m3.max(0.0)
}

pub const NOMINAL_NOZZLE_CDA_M2: f64 = 3.2e-3;
pub const NOMINAL_JETTISON_PUMP_RISE_PA: f64 = 50_000.0;

pub const REFERENCE_JETTISON_RATE_PER_SIDE_KG_S: f64 = 2_000.0 / 60.0;

#[cfg(test)]
mod tests {
    use super::*;

    const RHO: f64 = 800.0;
    const SEA_LEVEL_PA: f64 = 101_325.0;
    const CRUISE_PA: f64 = 22_600.0;

    #[test]
    fn orifice_flow_is_zero_with_no_favourable_pressure_difference() {
        assert_eq!(orifice_flow_m3_s(0.001, 0.0, RHO), 0.0);
        assert_eq!(orifice_flow_m3_s(0.001, -100.0, RHO), 0.0);
        assert_eq!(orifice_flow_m3_s(0.0, 1000.0, RHO), 0.0);
    }

    #[test]
    fn orifice_flow_grows_with_area_and_pressure_and_never_nans() {
        let base = orifice_flow_m3_s(0.001, 50_000.0, RHO);
        assert!(base > 0.0 && base.is_finite());
        assert!(orifice_flow_m3_s(0.002, 50_000.0, RHO) > base);
        assert!(orifice_flow_m3_s(0.001, 100_000.0, RHO) > base);
    }

    #[test]
    fn a_valve_commanded_open_ramps_fully_open_over_its_opening_time() {
        let mut v = JettisonValve::new();
        v.step(true, 2.0, 0.0, 1.0);
        assert!((v.position - 0.5).abs() < 1e-9);
        v.step(true, 2.0, 0.0, 1.0);
        assert!((v.position - 1.0).abs() < 1e-9);
    }

    #[test]
    fn a_valve_commanded_closed_ramps_back_shut() {
        let mut v = JettisonValve { position: 1.0, ..Default::default() };
        v.step(false, 1.0, 0.0, 1.0);
        assert!((v.position - 0.0).abs() < 1e-9);
    }

    #[test]
    fn a_stuck_valve_freezes_its_reachable_range_at_the_position_it_seized_at() {
        let mut v = JettisonValve::new();
        v.step(true, 1.0, 0.0, 0.5);
        assert!((v.position - 0.5).abs() < 1e-9);
        v.step(true, 1.0, 1.0, 5.0);
        assert!((v.position - 0.5).abs() < 1e-9);
    }

    #[test]
    fn a_partially_stuck_valve_has_partial_authority_to_keep_moving() {
        let mut v = JettisonValve::new();
        v.step(true, 1.0, 0.0, 0.5);
        v.step(true, 10.0, 0.5, 100.0);
        assert!(v.position > 0.5 && v.position < 1.0, "{}", v.position);
    }

    #[test]
    fn blockage_and_a_shut_valve_both_reduce_effective_area_to_zero_or_less() {
        assert_eq!(effective_cda_m2(0.002, 1.0, 1.0), 0.0);
        assert_eq!(effective_cda_m2(0.002, 0.0, 0.0), 0.0);
        let half = effective_cda_m2(0.002, 0.5, 1.0);
        assert!((half - 0.001).abs() < 1e-9);
    }

    #[test]
    fn jettison_flow_falls_as_the_tank_drains() {
        let cda = 0.0015;
        let full = jettison_mass_flow_kg_s(cda, 3.0, 0.0, SEA_LEVEL_PA, SEA_LEVEL_PA, RHO);
        let half = jettison_mass_flow_kg_s(cda, 1.5, 0.0, SEA_LEVEL_PA, SEA_LEVEL_PA, RHO);
        let empty = jettison_mass_flow_kg_s(cda, 0.0, 0.0, SEA_LEVEL_PA, SEA_LEVEL_PA, RHO);
        assert!(full > half);
        assert!(half > empty);
        assert_eq!(empty, 0.0);
        assert!((full / half - std::f64::consts::SQRT_2).abs() < 1e-9, "{full} / {half}");
    }

    #[test]
    fn jettison_flow_is_set_by_the_pressure_across_the_nozzle_not_by_altitude() {
        let cda = 0.0015;
        let sea_level = jettison_mass_flow_kg_s(cda, 2.0, 0.0, SEA_LEVEL_PA, SEA_LEVEL_PA, RHO);
        let cruise = jettison_mass_flow_kg_s(cda, 2.0, 0.0, CRUISE_PA, CRUISE_PA, RHO);
        assert!(sea_level > 0.0);
        assert!((cruise - sea_level).abs() < 1e-9, "{cruise} vs {sea_level}");
        let with_suction = jettison_mass_flow_kg_s(cda, 2.0, 0.0, CRUISE_PA, CRUISE_PA - 5_000.0, RHO);
        assert!((with_suction / cruise - 1.148).abs() < 0.002, "{}", with_suction / cruise);
    }

    #[test]
    fn a_boost_pump_assist_adds_to_the_driving_pressure() {
        let cda = 0.0015;
        let gravity_only = jettison_mass_flow_kg_s(cda, 1.0, 0.0, SEA_LEVEL_PA, SEA_LEVEL_PA, RHO);
        let pump_assisted = jettison_mass_flow_kg_s(cda, 1.0, 50_000.0, SEA_LEVEL_PA, SEA_LEVEL_PA, RHO);
        assert!(pump_assisted > gravity_only);
        assert!((pump_assisted / gravity_only - 2.714).abs() < 0.005, "{}", pump_assisted / gravity_only);
    }

    #[test]
    fn the_nominal_nozzle_dumps_about_two_thousand_kg_per_minute_per_side() {
        let kg_s = jettison_mass_flow_kg_s(
            NOMINAL_NOZZLE_CDA_M2,
            2.0,
            NOMINAL_JETTISON_PUMP_RISE_PA,
            SEA_LEVEL_PA,
            SEA_LEVEL_PA,
            RHO,
        );
        let kg_min = kg_s * 60.0;
        assert!((kg_min - REFERENCE_JETTISON_RATE_PER_SIDE_KG_S * 60.0).abs() < 100.0, "{kg_min} kg/min");
    }

    #[test]
    fn the_modelled_rate_sits_inside_the_band_of_publicly_quoted_figures() {
        let per_side_kg_min = jettison_mass_flow_kg_s(
            NOMINAL_NOZZLE_CDA_M2,
            2.0,
            NOMINAL_JETTISON_PUMP_RISE_PA,
            SEA_LEVEL_PA,
            SEA_LEVEL_PA,
            RHO,
        ) * 60.0;
        let both_sides_kg_min = 2.0 * per_side_kg_min;
        assert!(
            both_sides_kg_min > 3_300.0,
            "peak total jettison {both_sides_kg_min} kg/min cannot average the highest quoted figure"
        );
        assert!(
            both_sides_kg_min < 2.0 * 3_300.0,
            "peak total jettison {both_sides_kg_min} kg/min is more than twice the highest quoted figure"
        );
    }
}
