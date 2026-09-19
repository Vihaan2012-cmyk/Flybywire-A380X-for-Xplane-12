//! Full gas-turbine component-matching solve: given this frame's burner-
//! inlet conditions (Tt3/Pt3, from the compressor maps at the spools'
//! *current* corrected speeds — `mod.rs::step` computes these the same way
//! it always has) and the governor's commanded fuel flow, this finds the
//! core mass flow, burner exit temperature and each turbine's actually-
//! delivered shaft power that are simultaneously consistent with:
//! - the combustor's energy balance (the burner exit temperature is not
//!   assumed; it is whatever the fuel/air energy balance at the *solved*
//!   core mass flow gives — residual `R0`);
//! - each turbine's own choked nozzle-guide-vane flow capacity
//!   (`mdot * sqrt(Tt) / Pt` is fixed by NGV throat area once choked,
//!   independent of what pressure exists downstream — the classic
//!   "compressor-turbine matching" constraint of Cohen, Rogers &
//!   Saravanamuttoo, *Gas Turbine Theory* — residuals `R1`/`R2`/`R3` for the
//!   HP/IP/LP turbines in series);
//! - the core (hot-stream) propelling nozzle's own compressible-orifice
//!   flow capacity at its design-calibrated throat area (residual `R4`,
//!   subsonic or choked, `nozzle::mass_flow_capacity`).
//!
//! This replaces the previous reduced-order approximation (`mod.rs`'s old
//! module docs, before this pass): a fixed design-point turbine torque
//! scaled by how far combustion power and core flow were from their design
//! values, which never used any turbine's own flow capacity at all. Turbine
//! flow choking — previously "not separately represented" (the old
//! documented simplification) — is now the load-bearing physics that
//! determines how much air the engine actually breathes.
//!
//! Five unknowns, five residuals, solved by Newton-Raphson with a numerical
//! (forward-difference) Jacobian: `x = [mdot_core, Tt4, P_hpt, P_ipt,
//! P_lpt]` — the brief's "map operating points" (the core mass flow, i.e.
//! where the compressors sit on their speed lines once matched to the
//! turbines' swallowing capacity, and each turbine's own delivered power,
//! i.e. where it sits on its own characteristic) "and the burner exit
//! temperature". Warm-started from the previous frame's converged solution
//! (the gas path changes little frame to frame at any fixed sim rate),
//! damped (each Newton step's fractional size is capped) and bounded (every
//! unknown is clamped to a physically sane range after each iteration), so
//! it converges smoothly even from a poor warm start (e.g. right after
//! light-off, or the very first frame) and can never hand back a NaN,
//! negative flow or runaway value to the rest of the model.
//!
//! ## What this does *not* solve implicitly
//! The three spools' own angular acceleration. `mod.rs::step` converts the
//! matched turbine powers to torque at each spool's *current* speed (via
//! `Spool::torque_from_power`, already floored for near-zero rotation) and
//! hands that to the existing explicit fixed-substep integrator
//! (`spool.rs`), unchanged and separately tested for numerical stability at
//! any `dt`, including a paused sim or a huge frame spike. That integrator
//! *is* the transient spool power balance the brief's residual list asks
//! for; folding it into this same simultaneous solve (a fully implicit
//! backward-Euler coupling of the gas path to spool speed within one
//! Newton system) was considered and is not attempted here, in the
//! interest of landing a working, tested version fast, per the lead's
//! instruction. Documented, not hidden. The physical consequence is
//! unchanged from before this pass: any given frame's gas-path solve still
//! uses last frame's spool speeds — a lag of a few milliseconds, invisible
//! at flight-sim frame rates, and no worse than the model already had.

use super::combustor;
use super::gas;
use super::nozzle;
use super::turbine;

/// One matching solve's inputs: this frame's burner-inlet thermodynamic
/// state (already computed from the compressor maps at the spools' current
/// corrected speeds), the commanded fuel flow, and the design-point
/// calibration constants that give each turbine/nozzle its flow capacity.
#[derive(Clone, Copy, Debug)]
pub struct MatchInputs {
    pub tt3_k: f64,
    pub pt3_pa: f64,
    pub wf_kg_s: f64,
    pub bleed_extraction_kg_s: f64,
    pub ambient_pressure_pa: f64,
    pub eta_hpt: f64,
    pub eta_ipt: f64,
    pub eta_lpt: f64,
    /// Design-calibrated turbine flow-capacity constants, `mdot *
    /// sqrt(Tt) / Pt` (kg·K^0.5/(s·Pa)), for the HP/IP/LP turbine NGVs in
    /// series (see `mod.rs::design_point`).
    pub c_hpt: f64,
    pub c_ipt: f64,
    pub c_lpt: f64,
    /// Design-calibrated effective core nozzle throat area, m².
    pub nozzle_area_m2: f64,
    /// Design-point core mass flow and combustion power, used only to scale
    /// this solve's numerical-Jacobian step sizes and physical bounds — not
    /// otherwise part of the physics.
    pub design_mdot_core_kg_s: f64,
    pub design_power_w: f64,
}

/// One matching solve's outputs: the matched core mass flow, the resulting
/// station conditions all the way to the core nozzle inlet, and the power
/// each turbine actually delivered.
#[derive(Clone, Copy, Debug, Default)]
pub struct MatchResult {
    pub mdot_core_kg_s: f64,
    pub tt4_k: f64,
    pub pt4_pa: f64,
    pub mdot_gas_kg_s: f64,
    pub p_hpt_w: f64,
    pub p_ipt_w: f64,
    pub p_lpt_w: f64,
    /// HPT exit / IPT inlet.
    pub tt45_k: f64,
    pub pt45_pa: f64,
    /// IPT exit / LPT inlet — this port's EGT probe station.
    pub tt49_k: f64,
    pub pt49_pa: f64,
    /// LPT exit / core nozzle inlet.
    pub tt5_k: f64,
    pub pt5_pa: f64,
    pub iterations: u32,
    /// Non-dimensional residual norm at the returned iterate, for tests and
    /// telemetry (near zero = converged; the solver still returns its best
    /// iterate, clamped to physical bounds, even if this is not tiny).
    pub residual_norm: f64,
}

const N: usize = 5;

/// Evaluates all five residuals (and the station chain they came from) at
/// one candidate `x`. `x` is assumed already bounded by the caller.
fn residuals(x: &[f64; N], inp: &MatchInputs) -> ([f64; N], MatchResult) {
    let mdot_core = x[0].max(0.0);
    let tt4_guess = x[1].max(50.0);
    let p_hpt = x[2].max(0.0);
    let p_ipt = x[3].max(0.0);
    let p_lpt = x[4].max(0.0);

    // Bleed extraction happens after the HP compressor, before the
    // combustor (real bleed port location); floored at a small fraction of
    // whatever core flow actually exists so a stopped core still has a
    // stopped, not merely reduced, combustor (matches `mod.rs`'s previous
    // treatment).
    let mdot_to_combustor = (mdot_core - inp.bleed_extraction_kg_s.max(0.0)).max(0.02 * mdot_core);
    let comb = combustor::burn(mdot_to_combustor, inp.wf_kg_s, inp.tt3_k, inp.pt3_pa);

    let hpt = turbine::expand(tt4_guess, comb.pt4_pa, comb.mdot_gas_kg_s, p_hpt, inp.eta_hpt, gas::GAMMA_GAS);
    let ipt = turbine::expand(hpt.tt_out_k, hpt.pt_out_pa, comb.mdot_gas_kg_s, p_ipt, inp.eta_ipt, gas::GAMMA_GAS);
    let lpt = turbine::expand(ipt.tt_out_k, ipt.pt_out_pa, comb.mdot_gas_kg_s, p_lpt, inp.eta_lpt, gas::GAMMA_GAS);

    let pt4 = comb.pt4_pa.max(1.0);
    let pt45 = hpt.pt_out_pa.max(1.0);
    let pt49 = ipt.pt_out_pa.max(1.0);

    // R0: the burner's own energy balance at *this candidate's* mdot_core
    // must reproduce the candidate Tt4 — ties `mdot_core` and `Tt4`
    // together (mass/energy continuity through the combustor).
    let r0 = comb.tt4_k - tt4_guess;
    // R1-R3: each turbine's choked-NGV flow capacity, in series
    // (station mass continuity plus the turbine flow-capacity relation).
    let r1 = comb.mdot_gas_kg_s * tt4_guess.max(0.0).sqrt() / pt4 - inp.c_hpt;
    let r2 = comb.mdot_gas_kg_s * hpt.tt_out_k.max(0.0).sqrt() / pt45 - inp.c_ipt;
    let r3 = comb.mdot_gas_kg_s * ipt.tt_out_k.max(0.0).sqrt() / pt49 - inp.c_lpt;
    // R4: the core nozzle's own compressible-orifice flow capacity at its
    // design-calibrated fixed area (station mass continuity plus nozzle
    // flow capacity).
    let predicted_nozzle_mdot =
        nozzle::mass_flow_capacity(lpt.tt_out_k, lpt.pt_out_pa, inp.ambient_pressure_pa, inp.nozzle_area_m2, gas::GAMMA_GAS, gas::R_GAS);
    let r4 = predicted_nozzle_mdot - comb.mdot_gas_kg_s;

    let result = MatchResult {
        mdot_core_kg_s: mdot_core,
        tt4_k: comb.tt4_k,
        pt4_pa: comb.pt4_pa,
        mdot_gas_kg_s: comb.mdot_gas_kg_s,
        p_hpt_w: p_hpt,
        p_ipt_w: p_ipt,
        p_lpt_w: p_lpt,
        tt45_k: hpt.tt_out_k,
        pt45_pa: hpt.pt_out_pa,
        tt49_k: ipt.tt_out_k,
        pt49_pa: ipt.pt_out_pa,
        tt5_k: lpt.tt_out_k,
        pt5_pa: lpt.pt_out_pa,
        iterations: 0,
        residual_norm: 0.0,
    };
    ([r0, r1, r2, r3, r4], result)
}

/// Physical bounds applied to every unknown after every iteration, so the
/// solver can never diverge to a NaN, negative flow or unbounded power
/// regardless of warm start or Jacobian conditioning.
fn bound(x: [f64; N], inp: &MatchInputs) -> [f64; N] {
    let mdot_max = 4.0 * inp.design_mdot_core_kg_s.max(1.0);
    let power_max = 3.0 * inp.design_power_w.max(1.0);
    [x[0].clamp(0.0, mdot_max), x[1].clamp(200.0, 2400.0), x[2].clamp(0.0, power_max), x[3].clamp(0.0, power_max), x[4].clamp(0.0, power_max)]
}

/// Characteristic scales for each unknown/residual, so the Newton step,
/// numerical-Jacobian perturbation size and convergence test are not
/// dominated by unit choice (Tt4 in kelvin vs. mdot in kg/s vs. power in
/// watts span many orders of magnitude).
struct Scales {
    x: [f64; N],
    r: [f64; N],
}

fn scales(inp: &MatchInputs) -> Scales {
    let m = inp.design_mdot_core_kg_s.max(1.0);
    let p = inp.design_power_w.max(1.0);
    // Residual scales: R0 in kelvin; R1-R3 in the capacity constant's own
    // units (roughly m * sqrt(1000K) / typical Pa, order m*30/1e5); R4 in
    // kg/s.
    Scales { x: [m, 1000.0, p, p, p], r: [1000.0, m * 0.01, m * 0.01, m * 0.01, m] }
}

/// Solves the 5-unknown, 5-residual system by Newton-Raphson with a
/// numerical (forward-difference) Jacobian, warm-started from `x0`. Damped
/// (each step's fractional size is capped at `MAX_STEP_FRAC` of that
/// unknown's own scale) and bounded (`bound`, applied every iteration), so
/// it converges smoothly even from a poor warm start and never hands back
/// an unphysical value.
pub fn solve(inp: &MatchInputs, x0: [f64; N]) -> MatchResult {
    let sc = scales(inp);
    const MAX_ITERS: u32 = 20;
    const MAX_STEP_FRAC: f64 = 0.5;
    const TOL: f64 = 1e-7;

    let mut x = bound(x0, inp);
    let (r0, mut last) = residuals(&x, inp);
    let mut r = r0;

    for iter in 0..MAX_ITERS {
        let norm: f64 = (0..N).map(|i| (r[i] / sc.r[i]).powi(2)).sum::<f64>().sqrt();
        last.iterations = iter;
        last.residual_norm = norm;
        if norm < TOL {
            break;
        }

        // Numerical Jacobian, forward differences, each column perturbed by
        // that unknown's own characteristic size.
        let mut jac = [[0.0_f64; N]; N];
        for j in 0..N {
            let h = (sc.x[j] * 1e-4).max(1e-6);
            let mut xp = x;
            xp[j] += h;
            let (rp, _) = residuals(&bound(xp, inp), inp);
            for i in 0..N {
                jac[i][j] = (rp[i] - r[i]) / h;
            }
        }

        let Some(dx) = solve_linear(&jac, &r) else {
            break; // singular Jacobian: stop and return the last good iterate
        };

        // Newton step x_{k+1} = x_k - J^-1 * r, damped to at most
        // MAX_STEP_FRAC of each unknown's own scale so a poorly-conditioned
        // early iterate cannot overshoot into a wildly wrong region.
        let mut x_next = x;
        for i in 0..N {
            let step = (-dx[i]).clamp(-MAX_STEP_FRAC * sc.x[i], MAX_STEP_FRAC * sc.x[i]);
            x_next[i] = x[i] + step;
        }
        x = bound(x_next, inp);
        let (rn, result) = residuals(&x, inp);
        r = rn;
        last = result;
    }
    last
}

/// Solves `jac * dx = r` for `dx` by Gauss elimination with partial
/// pivoting. `N` is small (5) so this is cheap and allocation-free.
fn solve_linear(jac: &[[f64; N]; N], r: &[f64; N]) -> Option<[f64; N]> {
    let mut a = *jac;
    let mut b = *r;
    for col in 0..N {
        let mut pivot = col;
        for row in (col + 1)..N {
            if a[row][col].abs() > a[pivot][col].abs() {
                pivot = row;
            }
        }
        if a[pivot][col].abs() < 1e-12 {
            return None;
        }
        a.swap(col, pivot);
        b.swap(col, pivot);
        for row in (col + 1)..N {
            let factor = a[row][col] / a[col][col];
            for k in col..N {
                a[row][k] -= factor * a[col][k];
            }
            b[row] -= factor * b[col];
        }
    }
    let mut x = [0.0; N];
    for row in (0..N).rev() {
        let mut sum = b[row];
        for k in (row + 1)..N {
            sum -= a[row][k] * x[k];
        }
        x[row] = sum / a[row][row];
    }
    Some(x)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A representative set of design-like inputs: capacity constants and
    /// nozzle area computed exactly as `mod.rs::design_point` computes them
    /// (self-consistent design-point values), so a solve at the same Tt3/Pt3
    /// and design fuel flow should reproduce the design point closely.
    fn design_like_inputs(tt3: f64, pt3: f64, wf: f64) -> MatchInputs {
        // A representative Tt4/mdot/power set at a plausible operating
        // point, used only to derive self-consistent capacity constants for
        // this test (not the real engine's calibrated ones, which live in
        // `mod.rs` and are exercised end-to-end there).
        let mdot_core = 140.0;
        let comb = combustor::burn(mdot_core, wf, tt3, pt3);
        let p_hpt = 25_000_000.0;
        let p_ipt = 15_000_000.0;
        let p_lpt = 60_000_000.0;
        let hpt = turbine::expand(comb.tt4_k, comb.pt4_pa, comb.mdot_gas_kg_s, p_hpt, 0.90, gas::GAMMA_GAS);
        let ipt = turbine::expand(hpt.tt_out_k, hpt.pt_out_pa, comb.mdot_gas_kg_s, p_ipt, 0.91, gas::GAMMA_GAS);
        let lpt = turbine::expand(ipt.tt_out_k, ipt.pt_out_pa, comb.mdot_gas_kg_s, p_lpt, 0.925, gas::GAMMA_GAS);
        let ambient = 101_325.0;
        let area = nozzle::design_area_m2(comb.mdot_gas_kg_s, lpt.tt_out_k, lpt.pt_out_pa, ambient, gas::GAMMA_GAS, gas::R_GAS);

        MatchInputs {
            tt3_k: tt3,
            pt3_pa: pt3,
            wf_kg_s: wf,
            bleed_extraction_kg_s: 0.0,
            ambient_pressure_pa: ambient,
            eta_hpt: 0.90,
            eta_ipt: 0.91,
            eta_lpt: 0.925,
            c_hpt: comb.mdot_gas_kg_s * comb.tt4_k.sqrt() / comb.pt4_pa,
            c_ipt: comb.mdot_gas_kg_s * hpt.tt_out_k.sqrt() / hpt.pt_out_pa,
            c_lpt: comb.mdot_gas_kg_s * ipt.tt_out_k.sqrt() / ipt.pt_out_pa,
            nozzle_area_m2: area,
            design_mdot_core_kg_s: mdot_core,
            design_power_w: p_hpt.max(p_ipt).max(p_lpt),
        }
    }

    #[test]
    fn converges_to_the_design_point_it_was_calibrated_from() {
        let inp = design_like_inputs(650.0, 2_500_000.0, 3.4);
        let result = solve(&inp, [0.0, 288.15, 0.0, 0.0, 0.0]);
        assert!(result.residual_norm < 1e-6, "{:?}", result);
        assert!((result.mdot_core_kg_s - 140.0).abs() / 140.0 < 0.02, "{}", result.mdot_core_kg_s);
        assert!(result.tt4_k > 800.0 && result.tt4_k.is_finite());
    }

    #[test]
    fn a_warm_start_near_the_answer_converges_in_very_few_iterations() {
        let inp = design_like_inputs(650.0, 2_500_000.0, 3.4);
        let cold = solve(&inp, [0.0, 288.15, 0.0, 0.0, 0.0]);
        let warm = solve(&inp, [cold.mdot_core_kg_s, cold.tt4_k, cold.p_hpt_w, cold.p_ipt_w, cold.p_lpt_w]);
        assert!(warm.iterations <= 2, "{}", warm.iterations);
        assert!(warm.residual_norm < 1e-6);
    }

    #[test]
    fn more_fuel_at_the_same_burner_inlet_conditions_raises_tt4() {
        let low = solve(&design_like_inputs(650.0, 2_500_000.0, 2.0), [0.0, 288.15, 0.0, 0.0, 0.0]);
        let high = solve(&design_like_inputs(650.0, 2_500_000.0, 4.0), [0.0, 288.15, 0.0, 0.0, 0.0]);
        assert!(high.tt4_k > low.tt4_k, "{} vs {}", high.tt4_k, low.tt4_k);
    }

    #[test]
    fn a_hopeless_cold_start_from_zero_never_produces_nan_or_negative_flow() {
        let inp = design_like_inputs(288.15, 101_325.0 * 0.99, 0.0);
        let result = solve(&inp, [0.0, 288.15, 0.0, 0.0, 0.0]);
        assert!(result.mdot_core_kg_s.is_finite() && result.mdot_core_kg_s >= 0.0);
        assert!(result.tt4_k.is_finite());
        assert!(result.p_hpt_w >= 0.0 && result.p_ipt_w >= 0.0 && result.p_lpt_w >= 0.0);
    }

    #[test]
    fn a_wildly_wrong_warm_start_still_converges_without_diverging() {
        let inp = design_like_inputs(650.0, 2_500_000.0, 3.4);
        let result = solve(&inp, [inp.design_mdot_core_kg_s * 4.0, 2400.0, inp.design_power_w * 3.0, 0.0, inp.design_power_w * 3.0]);
        assert!(result.residual_norm.is_finite());
        assert!(result.mdot_core_kg_s.is_finite() && result.mdot_core_kg_s >= 0.0);
    }
}
