use super::common::vapour_pressure_pa;

pub const DESIGN_RISE_PA: f64 = 7.0e5;
pub const DESIGN_FLOW_M3_S: f64 = 6.0e-3;
const SHUTOFF_FLOW_MULTIPLE: f64 = 1.6;
const NPSH_REQUIRED_DESIGN_PA: f64 = 1.5e4;

#[derive(Clone, Copy, Debug, Default)]
pub struct LpPumpFaults {
    pub wear: f64,
    pub inlet_restriction: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct LpPumpState {
    pub outlet_pa: f64,
    pub flow_m3_s: f64,
    pub cavitating: bool,
}

pub fn step(n3_frac: f64, inlet_pa: f64, fuel_k: f64, demand_m3_s: f64, faults: &LpPumpFaults) -> LpPumpState {
    let n = n3_frac.max(0.0);
    let wear = faults.wear.clamp(0.0, 1.0);
    let restriction = faults.inlet_restriction.clamp(0.0, 1.0);
    let effective_inlet_pa = inlet_pa - restriction * (inlet_pa - vapour_pressure_pa(fuel_k)).max(0.0);
    let q = demand_m3_s.max(0.0);

    let rise = if n > 1e-4 {
        let q_ratio = q / (n * DESIGN_FLOW_M3_S * SHUTOFF_FLOW_MULTIPLE);
        (1.0 - wear) * n * n * DESIGN_RISE_PA * (1.0 - q_ratio * q_ratio).max(0.0)
    } else {
        0.0
    };

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
