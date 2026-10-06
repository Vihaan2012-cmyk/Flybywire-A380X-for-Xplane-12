const RAM_COEFF_KG_S_PER_PA: f64 = 4.0e-4;
const EDUCTOR_ENTRAINMENT_RATIO: f64 = 3.0;
const EDUCTOR_MOTIVE_KG_S: f64 = 0.05;
const MIN_SAFE_FLOW_KG_S: f64 = 0.1;

#[derive(Clone, Copy, Debug, Default)]
pub struct VentilationFaults {
    pub scoop_blockage: f64,
    pub eductor_blockage: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct VentilationState {
    pub flow_kg_s: f64,
    pub vapour_accumulation_risk: bool,
}

pub fn step(dynamic_pressure_pa: f64, engine_running: bool, faults: &VentilationFaults) -> VentilationState {
    let scoop_open = 1.0 - faults.scoop_blockage.clamp(0.0, 1.0);
    let eductor_open = 1.0 - faults.eductor_blockage.clamp(0.0, 1.0);

    let ram_kg_s = RAM_COEFF_KG_S_PER_PA * dynamic_pressure_pa.max(0.0) * scoop_open;
    let eductor_kg_s = if engine_running { EDUCTOR_MOTIVE_KG_S * EDUCTOR_ENTRAINMENT_RATIO * eductor_open } else { 0.0 };
    let flow_kg_s = ram_kg_s + eductor_kg_s;

    VentilationState { flow_kg_s, vapour_accumulation_risk: flow_kg_s < MIN_SAFE_FLOW_KG_S }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stationary_engine_off_gives_no_flow_and_flags_risk_no_nan() {
        let s = step(0.0, false, &VentilationFaults::default());
        assert_eq!(s.flow_kg_s, 0.0);
        assert!(s.vapour_accumulation_risk);
        assert!(!s.flow_kg_s.is_nan());
    }

    #[test]
    fn engine_running_on_the_ground_clears_the_minimum_via_the_eductor_alone() {
        let s = step(0.0, true, &VentilationFaults::default());
        assert!(s.flow_kg_s >= MIN_SAFE_FLOW_KG_S);
        assert!(!s.vapour_accumulation_risk);
    }

    #[test]
    fn cruise_dynamic_pressure_clears_the_minimum_via_ram_alone() {
        let s = step(5_000.0, false, &VentilationFaults::default());
        assert!(s.flow_kg_s >= MIN_SAFE_FLOW_KG_S);
    }

    #[test]
    fn a_fully_blocked_eductor_on_the_ground_reintroduces_the_risk() {
        let s = step(0.0, true, &VentilationFaults { eductor_blockage: 1.0, ..Default::default() });
        assert!(s.vapour_accumulation_risk);
        assert_eq!(s.flow_kg_s, 0.0);
    }

    #[test]
    fn a_fully_blocked_scoop_in_cruise_still_gets_eductor_flow_if_running() {
        let s = step(5_000.0, true, &VentilationFaults { scoop_blockage: 1.0, ..Default::default() });
        assert!(s.flow_kg_s > 0.0);
        assert!(s.flow_kg_s < step(5_000.0, true, &VentilationFaults::default()).flow_kg_s);
    }
}
