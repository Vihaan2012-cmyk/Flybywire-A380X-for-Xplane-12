//! Turbine flow capacity (Stodola's ellipse/cone law) and expansion.
//!
//! A multi-row turbine's mass flow capacity as a function of pressure ratio
//! follows, to a good approximation over its normal working range,
//! Stodola's "ellipse law" (A. Stodola, *Steam and Gas Turbines*, 1927;
//! restated in Cohen, Rogers & Saravanamuttoo, *Gas Turbine Theory*, as the
//! standard way to represent a turbine's flow characteristic without a full
//! nonlinear nozzle/blade-row map):
//!
//!   `mdot * sqrt(Tt_in) / Pt_in = C * sqrt(1 - (Pt_out/Pt_in)^2)`
//!
//! `C`, the flow-capacity coefficient, is a function only of the turbine's
//! geometry (throat areas), not of pressure ratio or speed -- which is what
//! makes this law a useful *generic* stand-in for a real flow map: the
//! turbine's whole characteristic reduces to the one number `C`, which
//! `power_section.rs` calibrates against this file's own design point
//! (chosen turbine-inlet temperature and pressure ratio, `params.rs`)
//! instead of measuring it.
//!
//! Because the coefficient is fixed by geometry, this file also gives the
//! *inverse* relation in closed form: given the mass flow that continuity
//! with the compressor upstream has already fixed, solve directly for the
//! pressure ratio the turbine must run at to pass exactly that flow, with no
//! iteration needed (the ellipse law is a simple quadratic in `1/PR`).

use super::gas;

/// One turbine's design point and generic map shape (an efficiency island,
/// same generic shape convention as `compressor_map.rs`).
#[derive(Clone, Copy, Debug)]
pub struct Spec {
    pub eta_design: f64,
    pub efficiency_falloff: f64,
}

impl Spec {
    /// Applies a turbine-damage fault (blade erosion, FOD, cracked
    /// shrouds -- 0 healthy .. 1 fully degraded) as a fractional isentropic
    /// efficiency loss, floored so a "fully degraded" turbine still turns,
    /// just very inefficiently, rather than becoming a singularity.
    pub fn degraded(&self, efficiency_loss_frac: f64) -> Spec {
        let loss = efficiency_loss_frac.clamp(0.0, 1.0);
        Spec {
            eta_design: (self.eta_design * (1.0 - loss)).max(0.3 * self.eta_design),
            ..*self
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Expansion {
    pub tt_out_k: f64,
    pub pt_out_pa: f64,
    pub delta_tt_k: f64,
    pub shaft_power_w: f64,
    pub eta: f64,
}

/// Stodola's ellipse law, forward direction: the corrected flow this
/// turbine passes at a given pressure ratio.
pub fn corrected_flow_kg_s(pressure_ratio: f64, capacity_coefficient: f64) -> f64 {
    let pr = pressure_ratio.max(1.0);
    let x = (1.0 / pr).clamp(0.0, 1.0);
    (capacity_coefficient.max(0.0) * (1.0 - x * x).max(0.0).sqrt()).max(0.0)
}

/// Stodola's ellipse law, inverse direction: the pressure ratio needed to
/// pass exactly `mdot_corrected_kg_s` through a turbine of this flow
/// capacity. Solved in closed form (`flow/C = sqrt(1-x^2)`, `x = 1/PR`), no
/// iteration -- monotonic and always well-defined for any non-negative
/// requested flow up to the capacity coefficient itself (flows at or above
/// `C` are physically impossible through this throat area and are capped at
/// the fully-open, near-infinite-pressure-ratio limit).
pub fn pressure_ratio_for_flow(mdot_corrected_kg_s: f64, capacity_coefficient: f64) -> f64 {
    if capacity_coefficient <= 1e-9 || mdot_corrected_kg_s <= 1e-9 {
        return 1.0;
    }
    let ratio = (mdot_corrected_kg_s / capacity_coefficient).clamp(0.0, 1.0 - 1e-9);
    let x = (1.0 - ratio * ratio).max(1e-9).sqrt();
    (1.0 / x).max(1.0)
}

/// Calibrates the capacity coefficient so that, at a chosen design corrected
/// flow and design pressure ratio, the ellipse law is satisfied exactly --
/// this task's own free scale parameter, fixed by the design point chosen
/// in `params.rs` rather than an unexplained magic number.
pub fn calibrate_capacity_coefficient(
    design_corrected_flow_kg_s: f64,
    design_pressure_ratio: f64,
) -> f64 {
    let pr = design_pressure_ratio.max(1.0 + 1e-6);
    let x = 1.0 / pr;
    let denom = (1.0 - x * x).max(1e-9).sqrt();
    design_corrected_flow_kg_s.max(0.0) / denom
}

/// Public so `governor.rs`'s EGT-limit fuel schedule can predict the
/// efficiency this turbine will run at without duplicating the formula.
pub fn efficiency(spec: &Spec, pr_frac_of_design: f64) -> f64 {
    let x = pr_frac_of_design.max(0.0);
    (spec.eta_design * (1.0 - spec.efficiency_falloff.clamp(0.0, 1.0) * (1.0 - x).powi(2)))
        .clamp(0.35 * spec.eta_design, spec.eta_design)
}

/// Expands `mdot_gas_kg_s` of gas at `tt_in_k`/`pt_in_pa` through this
/// turbine at the given pressure ratio (already fixed by continuity via
/// `pressure_ratio_for_flow`), returning the temperature/pressure drop and
/// the shaft power that expansion delivers.
pub fn expand(
    spec: &Spec,
    tt_in_k: f64,
    pt_in_pa: f64,
    mdot_gas_kg_s: f64,
    pressure_ratio: f64,
    pr_frac_of_design: f64,
) -> Expansion {
    let eta = efficiency(spec, pr_frac_of_design);
    if mdot_gas_kg_s <= 1e-6 || tt_in_k <= 1.0 || pressure_ratio <= 1.0 + 1e-9 {
        return Expansion {
            tt_out_k: tt_in_k,
            pt_out_pa: pt_in_pa,
            delta_tt_k: 0.0,
            shaft_power_w: 0.0,
            eta,
        };
    }
    let temp_ratio_isentropic =
        gas::temperature_ratio_from_pressure_ratio(1.0 / pressure_ratio, gas::GAMMA_GAS);
    let delta_tt_isentropic = tt_in_k * (1.0 - temp_ratio_isentropic);
    let delta_tt = (delta_tt_isentropic * eta).max(0.0);
    let tt_out = tt_in_k - delta_tt;
    let pt_out = pt_in_pa / pressure_ratio;
    let shaft_power = mdot_gas_kg_s * gas::CP_GAS_J_KG_K * delta_tt;
    Expansion {
        tt_out_k: tt_out,
        pt_out_pa: pt_out,
        delta_tt_k: delta_tt,
        shaft_power_w: shaft_power,
        eta,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec() -> Spec {
        Spec {
            eta_design: 0.82,
            efficiency_falloff: 0.45,
        }
    }

    #[test]
    fn forward_and_inverse_ellipse_law_are_mutual_inverses() {
        let c = 5.0;
        let pr = 3.2;
        let flow = corrected_flow_kg_s(pr, c);
        let pr_back = pressure_ratio_for_flow(flow, c);
        assert!((pr_back - pr).abs() < 1e-6, "{pr_back}");
    }

    #[test]
    fn calibration_reproduces_the_chosen_design_point_exactly() {
        let design_flow = 4.03;
        let design_pr = 4.032;
        let c = calibrate_capacity_coefficient(design_flow, design_pr);
        let flow_back = corrected_flow_kg_s(design_pr, c);
        assert!((flow_back - design_flow).abs() < 1e-6, "{flow_back}");
    }

    #[test]
    fn more_flow_through_a_fixed_throat_needs_a_higher_pressure_ratio() {
        let c = 5.0;
        let low_pr = pressure_ratio_for_flow(2.0, c);
        let high_pr = pressure_ratio_for_flow(4.0, c);
        assert!(high_pr > low_pr);
    }

    #[test]
    fn zero_flow_or_zero_pressure_ratio_never_produces_nan() {
        assert!(pressure_ratio_for_flow(0.0, 5.0).is_finite());
        assert!(pressure_ratio_for_flow(5.0, 0.0).is_finite());
        let e = expand(&spec(), 1150.0, 400_000.0, 0.0, 1.0, 1.0);
        assert!(e.tt_out_k.is_finite() && e.shaft_power_w == 0.0);
    }

    #[test]
    fn higher_pressure_ratio_extracts_more_shaft_power() {
        let low = expand(&spec(), 1150.0, 400_000.0, 4.0, 2.0, 0.5);
        let high = expand(&spec(), 1150.0, 400_000.0, 4.0, 4.0, 1.0);
        assert!(high.shaft_power_w > low.shaft_power_w);
        assert!(high.tt_out_k < low.tt_out_k);
    }

    #[test]
    fn damage_reduces_shaft_power_for_the_same_expansion() {
        let healthy = expand(&spec(), 1150.0, 400_000.0, 4.0, 3.5, 1.0);
        let damaged = expand(&spec().degraded(0.3), 1150.0, 400_000.0, 4.0, 3.5, 1.0);
        assert!(damaged.shaft_power_w < healthy.shaft_power_w);
        // Less work extracted from the same temperature/pressure drop means
        // more of the gas's enthalpy survives to the exit -- a damaged
        // turbine runs a hotter EGT for the same expansion, the real
        // causal signature of turbine damage.
        assert!(damaged.tt_out_k > healthy.tt_out_k);
    }
}
