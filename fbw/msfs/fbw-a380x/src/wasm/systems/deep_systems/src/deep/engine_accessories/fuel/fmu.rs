use super::common::FUEL_DENSITY_KG_M3;
use super::hp_pump::DESIGN_FLOW_M3_S as HP_PUMP_DESIGN_FLOW_M3_S;

const DP_REGULATED_PA: f64 = 1.5e6;
const CD: f64 = 0.62;
const AREA_MAX_M2: f64 = 1.7e-4;
const SLEW_RATE_M2_S: f64 = AREA_MAX_M2 / 1.5;

#[derive(Clone, Copy, Debug, Default)]
pub struct FmuFaults {
    pub valve_sticking: f64,
    pub spill_stuck_open: f64,
    pub spill_stuck_closed: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct FmuState {
    pub metered_kg_s: f64,
    pub valve_area_m2: f64,
    pub differential_pa: f64,
    pub spill_m3_s: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct FuelMeteringUnit {
    valve_area_m2: f64,
}

impl FuelMeteringUnit {
    pub fn new() -> Self {
        Self { valve_area_m2: 0.0 }
    }

    pub fn prime(&mut self, wf_kg_s: f64) {
        let flow_coeff = CD * (2.0 * DP_REGULATED_PA / FUEL_DENSITY_KG_M3).sqrt();
        self.valve_area_m2 = (wf_kg_s.max(0.0) / FUEL_DENSITY_KG_M3 / flow_coeff).min(AREA_MAX_M2);
    }

    pub fn step(&mut self, wf_command_kg_s: f64, hp_pump_supply_pa: f64, hp_pump_flow_m3_s: f64, faults: &FmuFaults, dt_s: f64) -> FmuState {
        let dt = dt_s.max(0.0);
        let sticking = faults.valve_sticking.clamp(0.0, 1.0);

        let stuck_closed = faults.spill_stuck_closed.clamp(0.0, 1.0);
        let stuck_open = faults.spill_stuck_open.clamp(0.0, 1.0);
        let inflated = DP_REGULATED_PA + stuck_closed * (hp_pump_supply_pa.max(DP_REGULATED_PA) - DP_REGULATED_PA);
        let differential_pa = (inflated * (1.0 - stuck_open)).max(0.0);

        let flow_coeff = CD * (2.0 * DP_REGULATED_PA / FUEL_DENSITY_KG_M3).max(0.0).sqrt();
        let target_area = if flow_coeff > 1e-9 { (wf_command_kg_s.max(0.0) / FUEL_DENSITY_KG_M3 / flow_coeff).min(AREA_MAX_M2) } else { 0.0 };

        let max_step = SLEW_RATE_M2_S * (1.0 - sticking) * dt;
        let error = target_area - self.valve_area_m2;
        self.valve_area_m2 += error.clamp(-max_step, max_step);
        self.valve_area_m2 = self.valve_area_m2.clamp(0.0, AREA_MAX_M2);

        let actual_coeff = CD * (2.0 * differential_pa / FUEL_DENSITY_KG_M3).max(0.0).sqrt();
        let wanted_m3_s = self.valve_area_m2 * actual_coeff;
        let valve_flow_m3_s = wanted_m3_s.min(hp_pump_flow_m3_s.max(0.0));
        let spill_m3_s = (hp_pump_flow_m3_s.max(0.0) - valve_flow_m3_s).max(0.0);

        FmuState {
            metered_kg_s: valve_flow_m3_s * FUEL_DENSITY_KG_M3,
            valve_area_m2: self.valve_area_m2,
            differential_pa,
            spill_m3_s,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn settle(fmu: &mut FuelMeteringUnit, wf: f64, faults: &FmuFaults, seconds: f64) -> FmuState {
        let dt = 0.02;
        let mut out = FmuState::default();
        for _ in 0..(seconds / dt) as usize {
            out = fmu.step(wf, 6.0e6, HP_PUMP_DESIGN_FLOW_M3_S, faults, dt);
        }
        out
    }

    #[test]
    fn zero_command_settles_to_zero_flow_no_nan() {
        let mut fmu = FuelMeteringUnit::new();
        let s = settle(&mut fmu, 0.0, &FmuFaults::default(), 5.0);
        assert!(s.metered_kg_s.abs() < 1e-6);
        assert!(!s.metered_kg_s.is_nan());
    }

    #[test]
    fn a_healthy_fmu_tracks_its_commanded_flow() {
        let mut fmu = FuelMeteringUnit::new();
        let s = settle(&mut fmu, 2.48, &FmuFaults::default(), 5.0);
        assert!((s.metered_kg_s - 2.48).abs() < 0.05, "{}", s.metered_kg_s);
    }

    #[test]
    fn a_fully_stuck_valve_never_moves_from_its_starting_position() {
        let mut fmu = FuelMeteringUnit::new();
        let s = settle(&mut fmu, 2.48, &FmuFaults { valve_sticking: 1.0, ..Default::default() }, 5.0);
        assert_eq!(s.valve_area_m2, 0.0);
        assert_eq!(s.metered_kg_s, 0.0);
    }

    #[test]
    fn a_spill_valve_stuck_open_starves_metered_flow() {
        let mut fmu = FuelMeteringUnit::new();
        let healthy = settle(&mut fmu, 2.48, &FmuFaults::default(), 5.0);
        let mut fmu2 = FuelMeteringUnit::new();
        let starved = settle(&mut fmu2, 2.48, &FmuFaults { spill_stuck_open: 0.95, ..Default::default() }, 5.0);
        assert!(starved.metered_kg_s < 0.3 * healthy.metered_kg_s);
    }

    #[test]
    fn a_spill_valve_stuck_closed_over_meters_for_the_same_command() {
        let mut fmu = FuelMeteringUnit::new();
        let healthy = settle(&mut fmu, 2.0, &FmuFaults::default(), 5.0);
        let mut fmu2 = FuelMeteringUnit::new();
        let over = settle(&mut fmu2, 2.0, &FmuFaults { spill_stuck_closed: 1.0, ..Default::default() }, 5.0);
        assert!(over.metered_kg_s > healthy.metered_kg_s);
    }
}
