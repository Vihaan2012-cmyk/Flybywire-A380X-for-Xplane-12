//! LP (boost) pump inlet fuel strainer, upstream of the filter modelled in
//! `fuel::filter`. Structurally identical to that module (a clean element
//! resists viscously, a clogging element's resistance grows as the flow
//! area shrinks, a spring-loaded bypass valve cracks open once the
//! differential reaches its set point) -- the real reason large transport
//! fuel systems put a coarse strainer ahead of the fine filter: it catches
//! the debris the filter would otherwise have to, with its own independent
//! bypass so a badly clogged strainer cannot starve the pump behind it.
//!
//! Revision 3 left the bypass-crack differential unsourced -- no Trent-900
//! fuel-strainer figure is public, and `TRENT900-LIMITS.md` is a
//! rotor-speed/temperature/oil/fuel-condition document, not a hydraulic-
//! component spec (`E-ENG-DESIGN.md` Pattern 21). **Phase 2 (2026-09-27):**
//! the A380 FCOM itself gives the real number directly for this exact
//! alert -- `PRO-ABN-ECAM-10-70`, "ENG 1(2)(3)(4) FUEL STRAINER CLOGGED"
//! (FCOM p.5787): "The pressure drop across the fuel strainer is higher
//! than 12 PSI." That supersedes the earlier approximation (reusing
//! `fuel::filter::BYPASS_CRACK_PA`, the fine filter's own 35 psi figure) --
//! a real, alert-specific source is now available, so the analogue is no
//! longer needed.

use super::common::viscosity_cst;

const PSI_PA: f64 = 6894.757;
/// Bypass valve cracking differential, Pa. FCOM PRO-ABN-ECAM p.5787, "ENG
/// 1(2)(3)(4) FUEL STRAINER CLOGGED": "The pressure drop across the fuel
/// strainer is higher than 12 PSI."
const BYPASS_CRACK_PA: f64 = 12.0 * PSI_PA;

/// Clean-element pressure drop at the design flow and reference (hot) fuel
/// temperature, Pa (**GENERIC**, the same order of magnitude as the fine
/// filter's own design drop -- a coarse strainer's clean-element loss is
/// typically smaller, not larger).
const DESIGN_DROP_PA: f64 = 0.2 * PSI_PA;
pub const DESIGN_FLOW_M3_S: f64 = 6.0e-3;
const REFERENCE_TEMP_K: f64 = 333.15;
/// The impending-bypass warning threshold, as a fraction of the cracking
/// differential -- the same fraction `fuel::filter` uses for the same
/// reason (a real warning fires before the valve actually opens).
const IMPENDING_BYPASS_FRACTION: f64 = 0.7;

#[derive(Clone, Copy, Debug, Default)]
pub struct StrainerFaults {
    /// Element blocked with debris, 0 clean .. 1 blocked; resistance grows
    /// as `1 / (1 - clog)^2`, the same relation `fuel::filter` uses.
    pub clog: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct StrainerState {
    pub outlet_pa: f64,
    pub differential_pa: f64,
    pub bypassed: bool,
    pub impending_bypass: bool,
}

/// One step, structurally identical to `fuel::filter::step`.
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
