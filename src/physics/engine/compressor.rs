//! A generic compressor characteristic, used for the fan, IP compressor and
//! HP compressor alike (each with its own design-point parameters).
//!
//! No public Trent 900 component map exists, so this uses the standard
//! turbomachinery similarity/affinity relationships instead of a measured
//! map, as the brief allows for "generic scaled maps" when a real one is
//! not public:
//! - Specific work (energy per unit mass) scales close to the square of
//!   corrected speed (Euler's turbomachinery equation gives `w ∝
//!   N_corrected^2` for a fixed velocity-triangle shape, blade speed U ∝
//!   N); `thermo` below uses a slightly gentler N^1.8, documented at its
//!   own definition, so a real cited low-speed operating point (idle)
//!   still produces net-positive compression once inlet losses are
//!   accounted for.
//! - Corrected mass flow scales linearly with corrected speed away from the
//!   choke/surge boundaries: `mdot_corrected ∝ N_corrected` (the common
//!   first-order approximation for the working range of a compressor map,
//!   e.g. as used in generic/GasTurb-style maps when no real one is
//!   available).
//! - Isentropic efficiency peaks at the design point and falls off
//!   off-design, modelled as a parabola in corrected speed (the generic
//!   "efficiency island" shape common to compressor maps).
//!
//! Power then follows as `mdot * w ∝ N_corrected^2.8`, close to the
//! standard turbomachinery affinity law's cubic scaling, which is what
//! gives the spool its nonlinear, stabilising torque-speed relationship in
//! `spool.rs`.

use super::gas::{self, CP_AIR, GAMMA_AIR};

/// One compressor's design point.
#[derive(Clone, Copy, Debug)]
pub struct Spec {
    /// Design-point pressure ratio.
    pub pr_design: f64,
    /// Design-point isentropic efficiency.
    pub eta_design: f64,
    /// Design-point corrected mass flow, kg/s (`mdot * sqrt(Tt_in/Tref) /
    /// (Pt_in/Pref)`), at whatever inlet conditions the caller passes.
    pub mdot_corrected_design_kg_s: f64,
    /// How sharply efficiency falls away from the design speed (higher =
    /// sharper). A generic efficiency-island curvature.
    pub efficiency_falloff: f64,
    /// Damage: the fraction of isentropic efficiency lost (0 healthy, 1
    /// none left), e.g. eroded, bent or missing blades. It lowers the
    /// pressure rise the blades get for their work; the work itself is set
    /// by blade speed (Euler) and does not change with it.
    pub efficiency_loss_fraction: f64,
}

/// A compression stage's result, given its inlet conditions and how fast it
/// is turning (as a fraction of its design corrected speed).
#[derive(Clone, Copy, Debug, Default)]
pub struct Stage {
    pub tt_out_k: f64,
    pub pt_out_pa: f64,
    pub specific_work_j_kg: f64,
    pub mdot_kg_s: f64,
    pub power_w: f64,
}

/// The thermodynamic part shared by both entry points below: temperature
/// rise, exit pressure and specific work, independent of how the mass flow
/// through the stage is determined.
fn thermo(spec: &Spec, tt_in_k: f64, pt_in_pa: f64, n_corrected_frac: f64) -> (f64, f64, f64) {
    let n = n_corrected_frac.max(0.0);

    // Specific work vs. corrected speed: Euler's turbomachinery equation
    // gives specific work ∝ U^2 ∝ N^2 for a fixed velocity-triangle shape,
    // the exponent used in `mod.rs`'s design-point torque-balance
    // simplification (`power ∝ N^3`, so this must stay consistent with
    // that). Measured generic compressor characteristics for real
    // machines are usually a little less steep than the idealised N^2 at
    // low corrected speed (blade angles/incidence are not truly fixed
    // across the whole speed range), so this uses N^1.8 as the generic map
    // shape — closer to the idealised Euler exponent than to a flat
    // linear approximation, but not so steep that a real engine's cited
    // low-speed idle setting (`params::IDLE_N1_PCT`, 15% of design) would
    // fail to produce a net-positive fan pressure ratio once inlet ram
    // recovery loss is accounted for (see `docs/physics/engine.md`'s idle
    // thrust validation).
    const SPEED_EXPONENT: f64 = 1.8;
    let isentropic_temp_ratio_design = gas::temperature_ratio_from_pressure_ratio(spec.pr_design, GAMMA_AIR);
    // Design-point specific work at a *reference* inlet temperature of
    // T_ref (288.15 K); work scales linearly with inlet temperature at
    // fixed corrected speed, applied via `tt_in_k` below.
    let w_design_per_kelvin = CP_AIR * (isentropic_temp_ratio_design - 1.0) / spec.eta_design;

    let specific_work = w_design_per_kelvin * tt_in_k * n.powf(SPEED_EXPONENT);
    let tt_out = tt_in_k + specific_work / CP_AIR;

    // Efficiency island: peaks at n=1, falls off a fraction of design value
    // away from it either side. Floored so the maths never divides by ~0.
    let eta = (spec.eta_design * (1.0 - spec.efficiency_falloff * (1.0 - n).powi(2))).clamp(0.35, spec.eta_design)
        * (1.0 - spec.efficiency_loss_fraction.clamp(0.0, 1.0));
    let ideal_work = specific_work * eta; // work actually usable isentropically
    let pt_out = pt_in_pa * gas::pressure_ratio_from_temperature_ratio(1.0 + ideal_work / (CP_AIR * tt_in_k), GAMMA_AIR);

    (specific_work, tt_out, pt_out)
}

/// Runs one compression stage that *defines* the mass flow through it (the
/// fan, which sets the whole engine's airflow). `n_corrected_frac` is this
/// spool's corrected speed as a fraction of its own design corrected speed
/// (0 at rest, 1 at the design point; can exceed 1 briefly under
/// overspeed). Mass flow is derived from the spec's own design corrected
/// flow via the corrected-flow relation (see module docs).
pub fn stage(spec: &Spec, tt_in_k: f64, pt_in_pa: f64, n_corrected_frac: f64) -> Stage {
    let n = n_corrected_frac.max(0.0);
    let (specific_work, tt_out, pt_out) = thermo(spec, tt_in_k, pt_in_pa, n);

    // Corrected flow ∝ N (see module docs), descaled to actual mass flow by
    // the usual corrected-flow relation.
    let mdot_corrected = spec.mdot_corrected_design_kg_s * n;
    let mdot = mdot_corrected * (pt_in_pa / super::params::P_REF_PA) / (tt_in_k / super::params::T_REF_K).sqrt();

    Stage { tt_out_k: tt_out, pt_out_pa: pt_out, specific_work_j_kg: specific_work, mdot_kg_s: mdot, power_w: mdot * specific_work }
}

/// Runs a downstream compression stage (IP or HP compressor) whose mass
/// flow is *given*, not derived from its own corrected-flow relation: it is
/// whatever the upstream fan already sent into the core. This is what
/// keeps mass exactly conserved along the core instead of letting each
/// stage's independent generic map disagree with the others (the "engine
/// matching" problem a full nonlinear model would solve iteratively;
/// fixing the flow at each downstream stage sidesteps needing that solve).
pub fn stage_fixed_flow(spec: &Spec, tt_in_k: f64, pt_in_pa: f64, n_corrected_frac: f64, mdot_kg_s: f64) -> Stage {
    let (specific_work, tt_out, pt_out) = thermo(spec, tt_in_k, pt_in_pa, n_corrected_frac.max(0.0));
    let mdot = mdot_kg_s.max(0.0);
    Stage { tt_out_k: tt_out, pt_out_pa: pt_out, specific_work_j_kg: specific_work, mdot_kg_s: mdot, power_w: mdot * specific_work }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec() -> Spec {
        Spec { pr_design: 1.6, eta_design: 0.9, mdot_corrected_design_kg_s: 1298.0, efficiency_falloff: 0.6, efficiency_loss_fraction: 0.0 }
    }

    #[test]
    fn design_speed_gives_the_design_pressure_ratio() {
        let s = stage(&spec(), 288.15, 101325.0, 1.0);
        let pr = s.pt_out_pa / 101325.0;
        // Efficiency < 1 means actual work exceeds the ideal isentropic
        // work at the same PR, so PR at the design *specific work* should
        // land close to, not above, the nameplate PR.
        assert!((pr - 1.6).abs() < 0.05, "{pr}");
    }

    #[test]
    fn lower_speed_gives_lower_pressure_ratio_and_flow() {
        let full = stage(&spec(), 288.15, 101325.0, 1.0);
        let half = stage(&spec(), 288.15, 101325.0, 0.5);
        assert!(half.pt_out_pa < full.pt_out_pa);
        assert!(half.mdot_kg_s < full.mdot_kg_s);
        assert!(half.power_w < full.power_w);
    }

    #[test]
    fn zero_speed_gives_zero_flow_and_work() {
        let s = stage(&spec(), 288.15, 101325.0, 0.0);
        assert!(s.mdot_kg_s.abs() < 1e-9);
        assert!(s.power_w.abs() < 1e-9);
    }

    #[test]
    fn fixed_flow_stages_use_exactly_the_given_mass_flow() {
        let s = stage_fixed_flow(&spec(), 300.0, 150_000.0, 0.9, 42.0);
        assert!((s.mdot_kg_s - 42.0).abs() < 1e-9);
        assert!(s.power_w > 0.0);
    }

    #[test]
    fn power_scales_roughly_with_the_cube_of_speed() {
        let full = stage(&spec(), 288.15, 101325.0, 1.0);
        let half = stage(&spec(), 288.15, 101325.0, 0.5);
        let ratio = full.power_w / half.power_w;
        // Not exactly 8x because efficiency also varies with speed, but it
        // should be in the right neighbourhood.
        assert!((4.0..14.0).contains(&ratio), "{ratio}");
    }
}
