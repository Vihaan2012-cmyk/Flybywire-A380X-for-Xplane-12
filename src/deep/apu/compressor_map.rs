//! A generic scaled compressor map, shared by the gas-generator's
//! power-section compressor and the load (bleed) compressor
//! (`power_section.rs`, `load_compressor.rs`) -- each with its own design
//! point (`Spec`).
//!
//! No public component map exists for either machine. The brief allows a
//! "generic scaled compressor map (corrected speed lines, surge line)" when
//! no real one is public; this follows the standard, machine-independent
//! turbomachinery similarity relationships used for exactly that purpose
//! (Cohen, Rogers & Saravanamuttoo, *Gas Turbine Theory*, ch. 5's generic
//! single-stage centrifugal compressor characteristic):
//!
//! - Specific work on a given corrected-speed line is set by the Euler
//!   turbomachinery equation (`w ~ U^2 ~ N^2` for a fixed blade-speed
//!   velocity triangle); a speed line is therefore close to a fixed pressure
//!   ratio, only weakly dependent on where along it the compressor
//!   operates, which is what makes real corrected-speed lines on a
//!   centrifugal compressor map run nearly vertical away from their surge
//!   and choke ends. `physics/engine/compressor.rs` documents the identical
//!   N^1.8 shape (a little gentler than the idealised N^2, matching real
//!   maps' low-speed behaviour); the exponent is restated independently
//!   here per this directory's self-containment rule, not imported.
//! - Each speed line has a *range* of corrected flow it can sustain: a
//!   surge boundary (minimum stable corrected flow -- below it the stage
//!   stalls and the through-flow collapses/reverses rather than following
//!   the map continuously) and a choke boundary (maximum, nozzle-limited,
//!   corrected flow). Published generic maps for this class of machine show
//!   the surge line's slope flatter than the working lines' own N^1 scaling,
//!   narrowing the available margin at part speed -- the shape this file
//!   uses (`Spec::surge_line_flatness`).
//! - Efficiency peaks on the nominal operating line and falls off toward
//!   either boundary (the generic "efficiency island" shape every published
//!   compressor map shows), and collapses further once the stage is
//!   actually surging or choked (both are, physically, loss-dominated
//!   off-design states, not smooth continuations of the healthy map).

use super::gas;

/// One compressor stage's design point and generic map shape.
#[derive(Clone, Copy, Debug)]
pub struct Spec {
    pub pr_design: f64,
    pub eta_design: f64,
    pub mdot_corrected_design_kg_s: f64,
    /// How sharply efficiency falls away from the design speed (higher =
    /// sharper). Generic efficiency-island curvature.
    pub efficiency_falloff: f64,
    /// Minimum stable corrected flow at *design* speed, as a fraction of
    /// `mdot_corrected_design_kg_s`.
    pub surge_margin_design_frac: f64,
    /// How much flatter than the nominal N^1 operating line the surge line
    /// runs; 0 = parallel (constant fractional margin at every speed), 1 =
    /// the surge line barely falls with speed at all (margin collapses
    /// toward zero as speed drops).
    pub surge_line_flatness: f64,
    /// Choke corrected flow as a multiple of the design corrected flow.
    pub choke_flow_multiple: f64,
    /// Erosion/damage state, 0 (clean, as designed) .. 1 (fully degraded).
    /// Applied by [`Spec::degraded`]; healthy specs leave it at 0.
    ///
    /// It is kept *separate* from `eta_design` on purpose. `eta_design` is
    /// the clean machine's design-point efficiency, and it is what sets the
    /// *work* the blading puts into the flow (see `evaluate`): eroded
    /// blading still spins at the same speed and still does the same Euler
    /// work on the air. What erosion changes is how much of that work comes
    /// back out as useful pressure rise. Folding the erosion into
    /// `eta_design` itself would (through `w_design_per_kelvin`'s `1/eta`)
    /// silently raise the work input by exactly the factor it lowered the
    /// efficiency by, leaving the delivered pressure rise unchanged -- an
    /// eroded compressor that costs more shaft power but suffers no
    /// pressure-ratio penalty at all, which is not what erosion does.
    pub erosion_efficiency_loss: f64,
}

impl Spec {
    /// Applies a compressor-erosion/damage fault (0 healthy .. 1 fully
    /// degraded) as a fractional loss of isentropic efficiency, floored so a
    /// "fully degraded" compressor is a badly worn machine (30% of its clean
    /// efficiency), not a mathematical singularity.
    pub fn degraded(&self, efficiency_loss_frac: f64) -> Spec {
        Spec {
            erosion_efficiency_loss: (self.erosion_efficiency_loss
                + efficiency_loss_frac.clamp(0.0, 1.0))
            .clamp(0.0, 0.7),
            ..*self
        }
    }
}

/// One evaluation of the map at a given corrected speed and requested
/// corrected flow.
#[derive(Clone, Copy, Debug, Default)]
pub struct Point {
    pub pr: f64,
    pub eta: f64,
    pub mdot_corrected_kg_s: f64,
    pub mdot_kg_s: f64,
    pub tt_out_k: f64,
    pub pt_out_pa: f64,
    pub specific_work_j_kg: f64,
    pub power_w: f64,
    /// `(requested - surge) / surge` at the current speed; negative means
    /// the requested flow is past the surge line.
    pub surge_margin: f64,
    pub in_surge: bool,
    pub choked: bool,
}

const SPEED_EXPONENT: f64 = 1.8;

/// The corrected flow, at the given corrected speed fraction, below which
/// this stage cannot sustain steady flow (the surge line).
pub fn surge_corrected_flow_kg_s(spec: &Spec, n_corrected_frac: f64) -> f64 {
    let n = n_corrected_frac.max(0.0);
    let design_surge = spec.mdot_corrected_design_kg_s * spec.surge_margin_design_frac.clamp(0.01, 0.9);
    let exponent = (1.0 - spec.surge_line_flatness.clamp(0.0, 0.95)).max(0.05);
    design_surge * n.powf(exponent)
}

/// The corrected flow above which this stage is choked.
pub fn choke_corrected_flow_kg_s(spec: &Spec, n_corrected_frac: f64) -> f64 {
    let n = n_corrected_frac.max(0.0);
    spec.mdot_corrected_design_kg_s * spec.choke_flow_multiple.max(1.01) * n
}

/// Runs the stage at corrected speed `n_corrected_frac` (0 at rest, 1 at
/// design, can exceed 1 under overspeed), being asked to pass
/// `requested_mdot_corrected_kg_s` of corrected flow (what continuity with
/// the rest of the gas path, or a downstream valve/IGV, is demanding of
/// it). Inlet conditions `tt_in_k`/`pt_in_pa` are actual (not corrected).
pub fn evaluate(
    spec: &Spec,
    tt_in_k: f64,
    pt_in_pa: f64,
    n_corrected_frac: f64,
    requested_mdot_corrected_kg_s: f64,
) -> Point {
    let n = n_corrected_frac.max(0.0);
    let tt_in = tt_in_k.max(1.0);
    let pt_in = pt_in_pa.max(1.0);

    let isentropic_temp_ratio_design =
        gas::temperature_ratio_from_pressure_ratio(spec.pr_design, gas::GAMMA_AIR);
    let w_design_per_kelvin =
        gas::CP_AIR_J_KG_K * (isentropic_temp_ratio_design - 1.0) / spec.eta_design.max(0.05);
    let specific_work_ideal_shape = w_design_per_kelvin * tt_in * n.powf(SPEED_EXPONENT);

    let surge_flow = surge_corrected_flow_kg_s(spec, n);
    let choke_flow = choke_corrected_flow_kg_s(spec, n);
    let requested = requested_mdot_corrected_kg_s.max(0.0);
    // Below a small speed floor the stage is essentially windmilling/at
    // rest: nothing to call "surge" (no rotating stall exists with no
    // rotation), it is simply near-zero flow and near-zero work.
    let in_surge = n > 0.05 && requested < surge_flow;
    let choked = requested > choke_flow;

    // Off-design efficiency island, then the erosion penalty on top of it.
    // Note `specific_work_ideal_shape` above deliberately uses the *clean*
    // `eta_design`: the work the blading puts in is set by blade speed
    // (Euler), not by how efficiently that work is recovered, so erosion
    // shows up purely as a lower delivered pressure ratio for the same
    // temperature rise and the same shaft power.
    let eta_clean =
        (spec.eta_design * (1.0 - spec.efficiency_falloff.clamp(0.0, 1.0) * (1.0 - n).powi(2)))
            .clamp(0.35 * spec.eta_design, spec.eta_design);
    let eta_nominal = eta_clean * (1.0 - spec.erosion_efficiency_loss.clamp(0.0, 0.7));

    let (mdot_corrected, specific_work, eta) = if in_surge {
        // A real single-stage surge is a violent breakdown of steady flow
        // (rotating stall / flow reversal), not a smooth continuation of
        // the map below the surge line: the stage cannot hold the pressure
        // rise its blade speed would otherwise deliver, so both the
        // pressure rise and the through-flow it actually manages collapse
        // toward a small fraction of what was requested, until conditions
        // (a control valve reopening, speed changing) move the operating
        // point back above the surge line.
        (requested * 0.15, specific_work_ideal_shape * 0.15, eta_nominal * 0.3)
    } else if choked {
        (choke_flow, specific_work_ideal_shape, eta_nominal * 0.7)
    } else {
        (requested, specific_work_ideal_shape, eta_nominal)
    };

    let ideal_work = specific_work * eta;
    let tt_out = tt_in + specific_work / gas::CP_AIR_J_KG_K;
    let pt_out = pt_in
        * gas::pressure_ratio_from_temperature_ratio(
            1.0 + ideal_work / (gas::CP_AIR_J_KG_K * tt_in),
            gas::GAMMA_AIR,
        );

    let mdot = gas::actual_flow_kg_s(mdot_corrected, tt_in, pt_in);
    let power = mdot * specific_work;

    Point {
        pr: pt_out / pt_in,
        eta,
        mdot_corrected_kg_s: mdot_corrected,
        mdot_kg_s: mdot,
        tt_out_k: tt_out,
        pt_out_pa: pt_out,
        specific_work_j_kg: specific_work,
        power_w: power,
        surge_margin: (requested - surge_flow) / surge_flow.max(1e-6),
        in_surge,
        choked,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec() -> Spec {
        Spec {
            pr_design: 4.2,
            eta_design: 0.78,
            mdot_corrected_design_kg_s: 4.0,
            efficiency_falloff: 0.55,
            surge_margin_design_frac: 0.12,
            surge_line_flatness: 0.5,
            choke_flow_multiple: 1.3,
            erosion_efficiency_loss: 0.0,
        }
    }

    #[test]
    fn design_speed_and_flow_gives_close_to_the_design_pressure_ratio() {
        let s = spec();
        let p = evaluate(&s, 288.15, 101_325.0, 1.0, s.mdot_corrected_design_kg_s);
        assert!(!p.in_surge && !p.choked);
        assert!((p.pr - 4.2).abs() < 0.2, "{}", p.pr);
    }

    #[test]
    fn zero_speed_gives_zero_flow_and_work_with_no_nan() {
        let p = evaluate(&spec(), 288.15, 101_325.0, 0.0, 0.0);
        assert!(p.mdot_kg_s.abs() < 1e-9);
        assert!(p.power_w.abs() < 1e-9);
        assert!(p.pr.is_finite() && p.tt_out_k.is_finite());
        assert!(!p.in_surge);
    }

    #[test]
    fn requesting_flow_below_the_surge_line_triggers_surge_and_collapses_pressure_rise() {
        let s = spec();
        let healthy = evaluate(&s, 288.15, 101_325.0, 1.0, s.mdot_corrected_design_kg_s);
        let surge_flow = surge_corrected_flow_kg_s(&s, 1.0);
        let starved = evaluate(&s, 288.15, 101_325.0, 1.0, surge_flow * 0.5);
        assert!(starved.in_surge);
        assert!(starved.pr < healthy.pr);
        assert!(starved.eta < healthy.eta);
    }

    #[test]
    fn requesting_flow_above_choke_is_capped_at_the_choke_flow() {
        let s = spec();
        let choke_flow = choke_corrected_flow_kg_s(&s, 1.0);
        let p = evaluate(&s, 288.15, 101_325.0, 1.0, choke_flow * 2.0);
        assert!(p.choked);
        assert!((p.mdot_corrected_kg_s - choke_flow).abs() < 1e-6);
    }

    #[test]
    fn erosion_lowers_pressure_ratio_for_the_same_speed_and_flow_without_changing_temperature_rise() {
        let s = spec();
        let healthy = evaluate(&s, 288.15, 101_325.0, 1.0, s.mdot_corrected_design_kg_s);
        let eroded = evaluate(
            &s.degraded(0.3),
            288.15,
            101_325.0,
            1.0,
            s.mdot_corrected_design_kg_s,
        );
        // Euler work in is set by blade speed (N), not by efficiency, so
        // the actual temperature rise the flow receives is unchanged...
        assert!((eroded.tt_out_k - healthy.tt_out_k).abs() < 1e-6);
        // ...but less of that work shows up as useful pressure rise.
        assert!(eroded.pr < healthy.pr);
        assert!(eroded.eta < healthy.eta);
    }

    #[test]
    fn surge_margin_narrows_at_part_speed() {
        let s = spec();
        // Nominal operating line flow scales with N; check the *ratio* of
        // nominal-line flow to surge-line flow shrinks as speed drops
        // (flatter surge line than the N^1 working line), the documented
        // generic map shape.
        let nominal_full = s.mdot_corrected_design_kg_s * 1.0;
        let nominal_half = s.mdot_corrected_design_kg_s * 0.5;
        let surge_full = surge_corrected_flow_kg_s(&s, 1.0);
        let surge_half = surge_corrected_flow_kg_s(&s, 0.5);
        let margin_full = nominal_full / surge_full;
        let margin_half = nominal_half / surge_half;
        assert!(margin_half < margin_full, "{margin_half} {margin_full}");
    }
}
