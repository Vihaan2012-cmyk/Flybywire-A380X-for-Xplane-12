use super::common::viscosity_cst;

const PSI_PA: f64 = 6894.757;

const DESIGN_DROP_PA: f64 = 0.5 * PSI_PA;
pub const DESIGN_FLOW_M3_S: f64 = 6.0e-3;
const REFERENCE_TEMP_K: f64 = 333.15;
pub(super) const BYPASS_CRACK_PA: f64 = 35.0 * PSI_PA;
const IMPENDING_BYPASS_FRACTION: f64 = 0.7;

#[derive(Clone, Copy, Debug, Default)]
pub struct FilterFaults {
    pub clog: f64,
    pub monitor_fault: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct FilterState {
    pub outlet_pa: f64,
    pub differential_pa: f64,
    pub bypassed: bool,
    pub impending_bypass: bool,
    pub monitor_fault: bool,
}

pub fn step(inlet_pa: f64, flow_m3_s: f64, fuel_k: f64, faults: &FilterFaults) -> FilterState {
    let clog = faults.clog.clamp(0.0, 0.999);
    let viscosity_ratio = viscosity_cst(fuel_k) / viscosity_cst(REFERENCE_TEMP_K);
    let q = flow_m3_s.max(0.0);
    let element_drop = DESIGN_DROP_PA * viscosity_ratio * (q / DESIGN_FLOW_M3_S) / (1.0 - clog).powi(2);
    let differential_pa = element_drop.min(BYPASS_CRACK_PA);
    FilterState {
        outlet_pa: inlet_pa - differential_pa,
        differential_pa,
        bypassed: element_drop >= BYPASS_CRACK_PA,
        impending_bypass: element_drop >= BYPASS_CRACK_PA * IMPENDING_BYPASS_FRACTION,
        monitor_fault: faults.monitor_fault > 0.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_clean_filter_at_zero_flow_drops_nothing_and_is_nan_free() {
        let s = step(3.0e5, 0.0, 288.0, &FilterFaults::default());
        assert_eq!(s.differential_pa, 0.0);
        assert!(!s.bypassed);
        assert!(!s.outlet_pa.is_nan());
    }

    #[test]
    fn a_clean_filter_at_design_flow_drops_a_modest_amount() {
        let s = step(3.0e5, DESIGN_FLOW_M3_S, REFERENCE_TEMP_K, &FilterFaults::default());
        assert!(s.differential_pa > 0.0 && s.differential_pa < BYPASS_CRACK_PA * 0.3);
        assert!(!s.bypassed && !s.impending_bypass);
    }

    #[test]
    fn a_badly_clogged_filter_opens_its_bypass() {
        let s = step(3.0e5, DESIGN_FLOW_M3_S, REFERENCE_TEMP_K, &FilterFaults { clog: 0.9, ..Default::default() });
        assert!(s.bypassed);
        assert_eq!(s.differential_pa, BYPASS_CRACK_PA);
    }

    #[test]
    fn a_partially_clogged_filter_warns_before_it_bypasses() {
        let s = step(3.0e5, DESIGN_FLOW_M3_S, REFERENCE_TEMP_K, &FilterFaults { clog: 0.87, ..Default::default() });
        assert!(s.impending_bypass);
        assert!(!s.bypassed, "must warn *before* it bypasses, not at the same time");
        assert!(
            (s.differential_pa / PSI_PA - 29.586).abs() < 0.01,
            "{}",
            s.differential_pa / PSI_PA
        );
    }

    #[test]
    fn cold_fuel_drops_more_than_hot_fuel_at_the_same_flow() {
        let cold = step(3.0e5, DESIGN_FLOW_M3_S, 240.0, &FilterFaults::default());
        let hot = step(3.0e5, DESIGN_FLOW_M3_S, REFERENCE_TEMP_K, &FilterFaults::default());
        assert!(cold.differential_pa > hot.differential_pa);
    }

    #[test]
    fn the_monitor_fault_is_independent_of_the_real_clog_state() {
        let dead_monitor = step(3.0e5, DESIGN_FLOW_M3_S, REFERENCE_TEMP_K, &FilterFaults { clog: 0.0, monitor_fault: 1.0 });
        assert!(dead_monitor.monitor_fault);
        assert!(!dead_monitor.bypassed && !dead_monitor.impending_bypass, "the element itself is clean");

        let healthy_monitor = step(3.0e5, DESIGN_FLOW_M3_S, REFERENCE_TEMP_K, &FilterFaults { clog: 0.9, monitor_fault: 0.0 });
        assert!(!healthy_monitor.monitor_fault);
        assert!(healthy_monitor.bypassed);
    }
}
