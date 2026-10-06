use super::common::viscosity_cst;

const PSI_PA: f64 = 6894.757;
const BYPASS_CRACK_PA: f64 = 12.0 * PSI_PA;

const DESIGN_DROP_PA: f64 = 0.2 * PSI_PA;
pub const DESIGN_FLOW_M3_S: f64 = 6.0e-3;
const REFERENCE_TEMP_K: f64 = 333.15;
const IMPENDING_BYPASS_FRACTION: f64 = 0.7;

#[derive(Clone, Copy, Debug, Default)]
pub struct StrainerFaults {
    pub clog: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct StrainerState {
    pub outlet_pa: f64,
    pub differential_pa: f64,
    pub bypassed: bool,
    pub impending_bypass: bool,
}

pub fn step(inlet_pa: f64, flow_m3_s: f64, fuel_k: f64, faults: &StrainerFaults) -> StrainerState {
    let clog = faults.clog.clamp(0.0, 0.999);
    let viscosity_ratio = viscosity_cst(fuel_k) / viscosity_cst(REFERENCE_TEMP_K);
    let q = flow_m3_s.max(0.0);
    let element_drop = DESIGN_DROP_PA * viscosity_ratio * (q / DESIGN_FLOW_M3_S) / (1.0 - clog).powi(2);
    let differential_pa = element_drop.min(BYPASS_CRACK_PA);
    StrainerState {
        outlet_pa: inlet_pa - differential_pa,
        differential_pa,
        bypassed: element_drop >= BYPASS_CRACK_PA,
        impending_bypass: element_drop >= BYPASS_CRACK_PA * IMPENDING_BYPASS_FRACTION,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_clean_strainer_at_zero_flow_drops_nothing_and_is_nan_free() {
        let s = step(3.0e5, 0.0, 288.0, &StrainerFaults::default());
        assert_eq!(s.differential_pa, 0.0);
        assert!(!s.bypassed);
        assert!(!s.outlet_pa.is_nan());
    }

    #[test]
    fn a_badly_clogged_strainer_opens_its_bypass() {
        let s = step(3.0e5, DESIGN_FLOW_M3_S, REFERENCE_TEMP_K, &StrainerFaults { clog: 0.95 });
        assert!(s.bypassed);
        assert_eq!(s.differential_pa, BYPASS_CRACK_PA);
    }

    #[test]
    fn a_healthy_strainer_stays_well_clear_of_its_own_bypass() {
        let s = step(3.0e5, DESIGN_FLOW_M3_S, REFERENCE_TEMP_K, &StrainerFaults::default());
        assert!(!s.bypassed && !s.impending_bypass);
    }
}
