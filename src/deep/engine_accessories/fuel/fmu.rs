//! Fuel metering unit: the metering valve and its pressure-drop (spill)
//! valve that together turn a commanded fuel flow into an actual one,
//! upstream of the HP shut-off valve (`shutoff_valve.rs`) and the burner
//! manifold (`manifold.rs`).
//!
//! A real FMU doesn't meter flow by controlling pressure -- it meters by
//! area, and uses a spill valve to hold the pressure drop *across* the
//! metering valve constant (a "constant-differential-pressure" governor),
//! so that flow through an orifice of area `A` is `Q = Cd * A * sqrt(2*dP/rho)`
//! with `dP` pinned near a fixed set point: area alone then sets flow
//! linearly, which is what lets a simple valve position be the fuel-flow
//! command. The HP gear pump (`hp_pump.rs`) is sized to always deliver more
//! than the combustor needs; whatever the metering valve does not pass, the
//! spill valve returns to the pump's inlet.
//!
//! Two independent things can go wrong: the metering valve itself can stick
//! (a seized or fouled spool moves slower than commanded, in the limit not
//! at all -- position lags or freezes, it does not teleport to a wrong
//! value), and the spill valve can fail to hold its set differential (stuck
//! open collapses the differential toward zero, starving the combustor of
//! metered flow even with the valve wide open; stuck closed lets the full,
//! unregulated HP pump pressure appear across the metering valve, over-
//! metering flow for whatever area is commanded).
//!
//! No Trent-900 FMU figures are public. The regulated differential and the
//! valve's discharge coefficient/area range are **GENERIC**, chosen so the
//! metering valve's full-open flow at the regulated differential exceeds
//! both the lead's gas path's ~2.48 kg/s SLS design-point fuel flow
//! (`physics::engine::gas_path`) and `hp_pump::DESIGN_FLOW_M3_S`, with
//! margin for the spill valve to always have something to spill.

use super::common::FUEL_DENSITY_KG_M3;
use super::hp_pump::DESIGN_FLOW_M3_S as HP_PUMP_DESIGN_FLOW_M3_S;

/// The spill valve's regulated differential across the metering valve, Pa
/// (**GENERIC**, typical of a fuel-metering constant-dP governor).
const DP_REGULATED_PA: f64 = 1.5e6;
/// Metering valve discharge coefficient (sharp-edged orifice, standard
/// textbook value) and full-open area, sized so `Cd * A_MAX *
/// sqrt(2*DP_REGULATED_PA/rho)` (~5.2 kg/s) exceeds both the design fuel
/// flow and the HP pump's own design delivery.
const CD: f64 = 0.62;
const AREA_MAX_M2: f64 = 1.7e-4;
/// Valve actuator slew rate at full authority, m^2/s (**GENERIC**: reaches
/// full travel in a couple of seconds, comparable to a torque-motor-driven
/// metering valve).
const SLEW_RATE_M2_S: f64 = AREA_MAX_M2 / 1.5;

/// Faults the FMU can carry, 0 (healthy) .. 1 (fully failed).
#[derive(Clone, Copy, Debug, Default)]
pub struct FmuFaults {
    /// Metering valve sticking (fouling/seizure): scales the actuator's
    /// slew rate down to zero at 1.0, so the valve moves ever more slowly
    /// toward whatever is commanded and, fully stuck, freezes wherever it
    /// happens to be -- not a scripted "wrong value", an emergent one.
    pub valve_sticking: f64,
    /// Spill valve stuck open: collapses the regulated differential toward
    /// zero, starving metered flow even at full valve area.
    pub spill_stuck_open: f64,
    /// Spill valve stuck closed: lets the differential rise toward the raw
    /// (unregulated) HP pump discharge pressure, over-metering flow for a
    /// given valve area.
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

    /// One step. `wf_command_kg_s` is the governor's commanded fuel mass
    /// flow, `hp_pump_supply_pa`/`hp_pump_flow_m3_s` the HP pump's discharge
    /// pressure and delivered flow this frame.
    pub fn step(&mut self, wf_command_kg_s: f64, hp_pump_supply_pa: f64, hp_pump_flow_m3_s: f64, faults: &FmuFaults, dt_s: f64) -> FmuState {
        let dt = dt_s.max(0.0);
        let sticking = faults.valve_sticking.clamp(0.0, 1.0);

        // Differential: stuck-closed inflates it toward the raw supply
        // pressure, stuck-open collapses it toward zero. Both can be
        // partial and are applied independently.
        let stuck_closed = faults.spill_stuck_closed.clamp(0.0, 1.0);
        let stuck_open = faults.spill_stuck_open.clamp(0.0, 1.0);
        let inflated = DP_REGULATED_PA + stuck_closed * (hp_pump_supply_pa.max(DP_REGULATED_PA) - DP_REGULATED_PA);
        let differential_pa = (inflated * (1.0 - stuck_open)).max(0.0);

        // Commanded area from the target flow at the *regulated* set point
        // (the valve is sized against the nominal differential; a real
        // torque motor does not know the spill valve has failed).
        let flow_coeff = CD * (2.0 * DP_REGULATED_PA / FUEL_DENSITY_KG_M3).max(0.0).sqrt();
        let target_area = if flow_coeff > 1e-9 { (wf_command_kg_s.max(0.0) / FUEL_DENSITY_KG_M3 / flow_coeff).min(AREA_MAX_M2) } else { 0.0 };

        // Rate-limited actuator, slower the more it is sticking; fully
        // stuck (1.0) cannot move at all.
        let max_step = SLEW_RATE_M2_S * (1.0 - sticking) * dt;
        let error = target_area - self.valve_area_m2;
        self.valve_area_m2 += error.clamp(-max_step, max_step);
        self.valve_area_m2 = self.valve_area_m2.clamp(0.0, AREA_MAX_M2);

        // Actual flow at the *actual* differential (which may differ from
        // the regulated set point if the spill valve has failed).
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
