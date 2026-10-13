//! LP (boost) fuel pump: the centrifugal first stage of the engine-driven
//! fuel pump unit, on the same accessory-gearbox shaft as the HP gear stage
//! (`hp_pump.rs`). Its job is to raise the fuel pressure enough above its
//! vapour pressure that the HP stage's own suction does not cavitate; a
//! centrifugal stage does that by imparting velocity to the fuel and
//! recovering it as pressure, so its delivery pressure rises with the
//! square of its speed and falls off with flow the way any centrifugal
//! pump's H-Q curve does (affinity laws: head ~ N^2 at fixed flow ratio,
//! flow ~ N at fixed head ratio).
//!
//! No Trent-900 LP pump design-point figures are public. The design
//! pressure rise and flow are **GENERIC**, sized so the design point sits
//! comfortably above the HP pump's own peak demand (`hp_pump::DESIGN_FLOW_M3_S`)
//! with margin. The general two-stage (centrifugal LP + gear HP) engine fuel
//! pump architecture on a common gearbox-driven shaft is standard across
//! large turbofans (Rolls-Royce and Honeywell public fuel-system training
//! summaries), not itself a Trent-972-specific figure.

use super::common::vapour_pressure_pa;

/// Design-point delivery pressure rise at 100% N3 and the flow that rise is
/// quoted at, Pa / m^3/s (**GENERIC**: ~7 bar, comfortably above the HP
/// pump's peak displacement flow).
pub const DESIGN_RISE_PA: f64 = 7.0e5;
pub const DESIGN_FLOW_M3_S: f64 = 6.0e-3;
/// The pump curve's shutoff flow (zero delivery) as a multiple of the design
/// flow -- a typical centrifugal pump's curve reaches zero head somewhat
/// past its best-efficiency flow (**GENERIC**, standard centrifugal-pump
/// curve shape).
const SHUTOFF_FLOW_MULTIPLE: f64 = 1.6;
/// Required net positive suction head, expressed directly as a pressure
/// margin over vapour pressure, at the design flow, Pa (**GENERIC**).
const NPSH_REQUIRED_DESIGN_PA: f64 = 1.5e4;

/// Faults the LP pump can carry, each 0 (healthy) .. 1 (fully failed).
#[derive(Clone, Copy, Debug, Default)]
pub struct LpPumpFaults {
    /// Impeller/bearing wear: derates the design pressure-rise coefficient.
    pub wear: f64,
    /// Inlet strainer icing or debris: throttles the suction pressure the
    /// pump sees before the impeller, feeding straight into the cavitation
    /// check below rather than being a separate scripted outcome.
    pub inlet_restriction: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct LpPumpState {
    pub outlet_pa: f64,
    pub flow_m3_s: f64,
    pub cavitating: bool,
}

/// One step. `n3_frac` is the HP spool speed (0..~1.2) the pump shaft turns
/// with; `inlet_pa` the tank/feed pressure arriving at the pump; `fuel_k`
/// the fuel temperature (sets vapour pressure); `demand_m3_s` the flow the
/// HP pump downstream is trying to draw through it.
pub fn step(n3_frac: f64, inlet_pa: f64, fuel_k: f64, demand_m3_s: f64, faults: &LpPumpFaults) -> LpPumpState {
    let n = n3_frac.max(0.0);
    let wear = faults.wear.clamp(0.0, 1.0);
    let restriction = faults.inlet_restriction.clamp(0.0, 1.0);
    let effective_inlet_pa = inlet_pa - restriction * (inlet_pa - vapour_pressure_pa(fuel_k)).max(0.0);
    let q = demand_m3_s.max(0.0);

    // Affinity-law pump curve: rise(Q) = (1-wear) * n^2 * RISE0 * (1 -
    // (Q/(n*Q0*SHUTOFF))^2); a stopped shaft (n ~ 0) delivers nothing, not a
    // divide-by-zero.
    let rise = if n > 1e-4 {
        let q_ratio = q / (n * DESIGN_FLOW_M3_S * SHUTOFF_FLOW_MULTIPLE);
        (1.0 - wear) * n * n * DESIGN_RISE_PA * (1.0 - q_ratio * q_ratio).max(0.0)
    } else {
        0.0
    };

    // Cavitation: available NPSH (as a pressure margin over vapour
    // pressure) against what the current flow demands; a shortfall derates
    // both the pressure achieved and the flow actually passed, rather than
    // just flagging a boolean with no consequence.
    let npsh_available = (effective_inlet_pa - vapour_pressure_pa(fuel_k)).max(0.0);
    let npsh_required = NPSH_REQUIRED_DESIGN_PA * (q / DESIGN_FLOW_M3_S).powi(2);
    let cavitation_factor = if npsh_required > 1e-6 { (npsh_available / npsh_required).clamp(0.0, 1.0) } else { 1.0 };
    let cavitating = cavitation_factor < 0.999 && q > 1e-9;

    LpPumpState {
        outlet_pa: effective_inlet_pa + rise * cavitation_factor,
        flow_m3_s: q * cavitation_factor,
        cavitating,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_stopped_shaft_delivers_no_rise_and_no_nan() {
        let s = step(0.0, 2.0e5, 288.0, 0.0, &LpPumpFaults::default());
        assert_eq!(s.outlet_pa, 2.0e5);
        assert!(!s.outlet_pa.is_nan() && !s.flow_m3_s.is_nan());
    }

    #[test]
    fn a_healthy_pump_at_speed_raises_pressure_above_inlet() {
        let s = step(0.97, 2.0e5, 288.0, DESIGN_FLOW_M3_S, &LpPumpFaults::default());
        assert!(s.outlet_pa > 2.0e5 + 3.0e5, "{}", s.outlet_pa);
        assert!(!s.cavitating);
    }

    #[test]
    fn wear_derates_the_pressure_rise() {
        let healthy = step(0.97, 2.0e5, 288.0, DESIGN_FLOW_M3_S, &LpPumpFaults::default());
        let worn = step(0.97, 2.0e5, 288.0, DESIGN_FLOW_M3_S, &LpPumpFaults { wear: 0.6, ..Default::default() });
        assert!(worn.outlet_pa < healthy.outlet_pa);
    }

    #[test]
    fn a_starved_inlet_at_hot_fuel_cavitates_and_loses_flow() {
        let healthy = step(0.97, 1.5e5, 330.0, DESIGN_FLOW_M3_S, &LpPumpFaults::default());
        let starved = step(0.97, 1.5e5, 330.0, DESIGN_FLOW_M3_S, &LpPumpFaults { inlet_restriction: 0.95, ..Default::default() });
        assert!(starved.cavitating);
        assert!(starved.flow_m3_s < healthy.flow_m3_s);
    }
}
