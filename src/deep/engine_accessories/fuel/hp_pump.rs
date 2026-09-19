//! HP fuel pump: the positive-displacement gear stage of the engine-driven
//! fuel pump unit, on the same accessory-gearbox shaft as the LP centrifugal
//! stage (`lp_pump.rs`). A gear pump's theoretical delivery is a fixed
//! displacement per revolution, so flow is directly proportional to shaft
//! speed (N3) regardless of downstream pressure; real delivery falls a
//! little short of that because some fuel slips back past the gear teeth
//! through the internal clearances, and that slip fraction grows with the
//! pressure the pump is working against and with wear opening the
//! clearances up -- exactly how a worn gear pump's real volumetric
//! efficiency degrades, not a scripted derate.
//!
//! No Trent-900 HP pump displacement is public. `DISPLACEMENT_M3_PER_REV` is
//! **GENERIC**, sized so `DESIGN_FLOW_M3_S` at `N3_DESIGN_RPM` comfortably
//! exceeds the fuel flow the design point needs -- the lead's rebuilt gas
//! path (`physics::engine::gas_path`) calibrates a design-point fuel flow
//! of about 2.48 kg/s at the sea-level-static design point; sized at
//! roughly 1.6x that (~4.1 kg/s / 800 kg/m^3 ~= 5.1e-3 m^3/s) gives the
//! margin a real fuel system carries so the FMU (`fmu.rs`) always has
//! excess flow to spill.

use super::common::FUEL_DENSITY_KG_M3;

/// HP spool design speed, RPM (matches `physics::engine::params::N3_DESIGN_RPM`;
/// restated here so this module stays self-contained, per this directory's
/// isolation rule).
pub const N3_DESIGN_RPM: f64 = 12200.0;
/// Displacement per revolution, m^3 (**GENERIC**, see module docs).
pub const DISPLACEMENT_M3_PER_REV: f64 = 2.5e-5;
pub const DESIGN_FLOW_M3_S: f64 = DISPLACEMENT_M3_PER_REV * N3_DESIGN_RPM / 60.0;
/// Internal slip flow at the design pressure rise, as a fraction of the
/// theoretical (displacement) flow -- a healthy gear pump's volumetric
/// efficiency is typically 90-97%; 5% is a **GENERIC** mid-range figure.
const DESIGN_SLIP_FRACTION: f64 = 0.05;
/// The pressure rise the slip fraction above is quoted at, Pa (**GENERIC**,
/// a representative HP pump discharge pressure for a large turbofan FMU).
const DESIGN_RISE_PA: f64 = 4.0e6;

/// Faults the HP pump can carry, 0 (healthy) .. 1 (fully failed).
#[derive(Clone, Copy, Debug, Default)]
pub struct HpPumpFaults {
    /// Gear/bearing wear opening the internal clearances: slip flow grows
    /// as `1 / (1 - wear)^2` at a given pressure rise (the same
    /// clearance-area-squared relation the oil and fuel filters use for a
    /// growing leak path).
    pub wear: f64,
    /// Cavitation/starvation at the pump inlet (fed forward from
    /// `lp_pump::LpPumpState::cavitating`/flow shortfall, not recomputed
    /// here): a fraction of the theoretical flow that simply never arrives
    /// because there is not enough fuel at the inlet to fill the gear
    /// pockets.
    pub inlet_starvation: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct HpPumpState {
    pub delivered_m3_s: f64,
    pub delivered_kg_s: f64,
    pub slip_m3_s: f64,
}

/// One step. `n3_frac` is the HP spool speed (0..~1.2); `discharge_rise_pa`
/// the pressure rise the pump is working against (from the FMU's spill-valve
/// regulation, `fmu.rs`).
pub fn step(n3_frac: f64, discharge_rise_pa: f64, faults: &HpPumpFaults) -> HpPumpState {
    let n = n3_frac.max(0.0);
    let theoretical_m3_s = DISPLACEMENT_M3_PER_REV * (n * N3_DESIGN_RPM) / 60.0;
    let wear = faults.wear.clamp(0.0, 0.98);
    let rise_ratio = (discharge_rise_pa.max(0.0) / DESIGN_RISE_PA).max(0.0);
    let slip_m3_s = theoretical_m3_s * DESIGN_SLIP_FRACTION * rise_ratio / (1.0 - wear).powi(2);
    let after_slip = (theoretical_m3_s - slip_m3_s).max(0.0);
    let starvation = faults.inlet_starvation.clamp(0.0, 1.0);
    let delivered_m3_s = after_slip * (1.0 - starvation);
    HpPumpState { delivered_m3_s, delivered_kg_s: delivered_m3_s * FUEL_DENSITY_KG_M3, slip_m3_s }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_stopped_shaft_delivers_nothing_with_no_nan() {
        let s = step(0.0, 0.0, &HpPumpFaults::default());
        assert_eq!(s.delivered_m3_s, 0.0);
        assert!(!s.delivered_kg_s.is_nan());
    }

    #[test]
    fn delivery_scales_with_speed_at_a_fixed_pressure() {
        let half = step(0.5, DESIGN_RISE_PA, &HpPumpFaults::default());
        let full = step(1.0, DESIGN_RISE_PA, &HpPumpFaults::default());
        assert!((full.delivered_m3_s - 2.0 * half.delivered_m3_s).abs() / full.delivered_m3_s < 0.02);
    }

    #[test]
    fn design_flow_exceeds_the_engines_design_point_fuel_demand() {
        let s = step(1.0, DESIGN_RISE_PA, &HpPumpFaults::default());
        // ~2.48 kg/s design fuel flow (physics::engine::gas_path's
        // calibrated SLS design point); the HP pump must deliver comfortably
        // more so the FMU always has excess to spill.
        assert!(s.delivered_kg_s > 2.48 * 1.3, "{}", s.delivered_kg_s);
    }

    #[test]
    fn wear_increases_slip_and_reduces_delivery_at_pressure() {
        let healthy = step(1.0, DESIGN_RISE_PA, &HpPumpFaults::default());
        let worn = step(1.0, DESIGN_RISE_PA, &HpPumpFaults { wear: 0.7, ..Default::default() });
        assert!(worn.slip_m3_s > healthy.slip_m3_s);
        assert!(worn.delivered_m3_s < healthy.delivered_m3_s);
    }

    #[test]
    fn inlet_starvation_caps_delivery_regardless_of_slip() {
        let s = step(1.0, 0.0, &HpPumpFaults { inlet_starvation: 0.6, ..Default::default() });
        let full = step(1.0, 0.0, &HpPumpFaults::default());
        assert!((s.delivered_m3_s - 0.4 * full.delivered_m3_s).abs() < 1e-9);
    }
}
