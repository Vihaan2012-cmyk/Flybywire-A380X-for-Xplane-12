//! Main fuel filter, between the LP and HP pump stages, with its bypass
//! valve. Modelled the same way `physics::engine::oil`'s filter is: a clean
//! element resists viscously (pressure drop proportional to flow and
//! viscosity), a clogging element's resistance grows as the flow area
//! shrinks (`1 / (1 - clog)^2`, the orifice-area-squared relation for a
//! viscous element losing open area), and a spring-loaded bypass valve
//! cracks open once the differential across the element reaches its set
//! point, so a badly clogged filter cannot starve the HP pump -- it instead
//! passes unfiltered fuel, which is exactly the real failure mode a bypass
//! valve exists to trade against (an impending-bypass indication is common
//! on real aircraft fuel filters for this reason).
//!
//! No Trent-900 fuel filter figures are public; the design-point pressure
//! drop and the bypass cracking differential are **GENERIC**, typical of an
//! engine fuel filter (a few psi clean, opening its bypass in the tens of
//! psi).

use super::common::viscosity_cst;

const PSI_PA: f64 = 6894.757;

/// Clean-element pressure drop at the design flow and reference (hot) fuel
/// temperature, Pa (**GENERIC**).
const DESIGN_DROP_PA: f64 = 0.5 * PSI_PA;
pub const DESIGN_FLOW_M3_S: f64 = 6.0e-3;
/// Reference temperature the design drop is quoted at, K.
const REFERENCE_TEMP_K: f64 = 333.15;
/// Bypass valve cracking differential, Pa (**GENERIC**).
const BYPASS_CRACK_PA: f64 = 35.0 * PSI_PA;
/// The impending-bypass warning threshold, as a fraction of the cracking
/// differential (real filter bypass indicators typically warn well before
/// the valve actually opens).
const IMPENDING_BYPASS_FRACTION: f64 = 0.7;

/// Faults the filter can carry, 0 (healthy) .. 1 (fully failed).
#[derive(Clone, Copy, Debug, Default)]
pub struct FilterFaults {
    /// Element blocked with debris/wax (cold-fuel wax formation, or debris
    /// ingested from the tank): resistance grows as `1 / (1 - clog)^2`.
    pub clog: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct FilterState {
    pub outlet_pa: f64,
    pub differential_pa: f64,
    pub bypassed: bool,
    pub impending_bypass: bool,
}

/// One step. `inlet_pa` is the LP pump's delivery, `flow_m3_s` the flow
/// actually passing through (the HP pump's suction demand), `fuel_k` the
/// fuel temperature (colder fuel is more viscous and drops more pressure
/// for the same flow, exactly like the oil filter).
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
        let s = step(3.0e5, DESIGN_FLOW_M3_S, REFERENCE_TEMP_K, &FilterFaults { clog: 0.9 });
        assert!(s.bypassed);
        assert_eq!(s.differential_pa, BYPASS_CRACK_PA);
    }

    #[test]
    fn a_partially_clogged_filter_warns_before_it_bypasses() {
        // Solve for a clog fraction that lands the element drop just under
        // the crack pressure but over the warning fraction.
        let s = step(3.0e5, DESIGN_FLOW_M3_S, REFERENCE_TEMP_K, &FilterFaults { clog: 0.85 });
        assert!(s.impending_bypass);
    }

    #[test]
    fn cold_fuel_drops_more_than_hot_fuel_at_the_same_flow() {
        let cold = step(3.0e5, DESIGN_FLOW_M3_S, 240.0, &FilterFaults::default());
        let hot = step(3.0e5, DESIGN_FLOW_M3_S, REFERENCE_TEMP_K, &FilterFaults::default());
        assert!(cold.differential_pa > hot.differential_pa);
    }
}
