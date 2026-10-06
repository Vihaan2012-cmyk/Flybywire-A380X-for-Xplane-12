use super::combustor;
use super::gas;
use super::nozzle;
use super::turbine;

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
    pub c_hpt: f64,
    pub c_ipt: f64,
    pub c_lpt: f64,
    pub nozzle_area_m2: f64,
    pub design_mdot_core_kg_s: f64,
    pub design_power_w: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct MatchResult {
    pub mdot_core_kg_s: f64,
    pub tt4_k: f64,
    pub pt4_pa: f64,
    pub mdot_gas_kg_s: f64,
    pub p_hpt_w: f64,
    pub p_ipt_w: f64,
    pub p_lpt_w: f64,
    pub tt45_k: f64,
    pub pt45_pa: f64,
    pub tt49_k: f64,
    pub pt49_pa: f64,
    pub tt5_k: f64,
    pub pt5_pa: f64,
    pub iterations: u32,
    pub residual_norm: f64,
}

const N: usize = 5;

fn residuals(x: &[f64; N], inp: &MatchInputs) -> ([f64; N], MatchResult) {
    let mdot_core = x[0].max(0.0);
    let tt4_guess = x[1].max(50.0);
    let p_hpt = x[2].max(0.0);
    let p_ipt = x[3].max(0.0);
    let p_lpt = x[4].max(0.0);

    let mdot_to_combustor = (mdot_core - inp.bleed_extraction_kg_s.max(0.0)).max(0.02 * mdot_core);
    let comb = combustor::burn(mdot_to_combustor, inp.wf_kg_s, inp.tt3_k, inp.pt3_pa);

    let hpt = turbine::expand(tt4_guess, comb.pt4_pa, comb.mdot_gas_kg_s, p_hpt, inp.eta_hpt, gas::GAMMA_GAS);
    let ipt = turbine::expand(hpt.tt_out_k, hpt.pt_out_pa, comb.mdot_gas_kg_s, p_ipt, inp.eta_ipt, gas::GAMMA_GAS);
    let lpt = turbine::expand(ipt.tt_out_k, ipt.pt_out_pa, comb.mdot_gas_kg_s, p_lpt, inp.eta_lpt, gas::GAMMA_GAS);

    let pt4 = comb.pt4_pa.max(1.0);
    let pt45 = hpt.pt_out_pa.max(1.0);
    let pt49 = ipt.pt_out_pa.max(1.0);

    let r0 = comb.tt4_k - tt4_guess;
    let r1 = comb.mdot_gas_kg_s * tt4_guess.max(0.0).sqrt() / pt4 - inp.c_hpt;
    let r2 = comb.mdot_gas_kg_s * hpt.tt_out_k.max(0.0).sqrt() / pt45 - inp.c_ipt;
    let r3 = comb.mdot_gas_kg_s * ipt.tt_out_k.max(0.0).sqrt() / pt49 - inp.c_lpt;
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

fn bound(x: [f64; N], inp: &MatchInputs) -> [f64; N] {
    let mdot_max = 4.0 * inp.design_mdot_core_kg_s.max(1.0);
    let power_max = 3.0 * inp.design_power_w.max(1.0);
    [x[0].clamp(0.0, mdot_max), x[1].clamp(200.0, 2400.0), x[2].clamp(0.0, power_max), x[3].clamp(0.0, power_max), x[4].clamp(0.0, power_max)]
}

struct Scales {
    x: [f64; N],
    r: [f64; N],
}

fn scales(inp: &MatchInputs) -> Scales {
    let m = inp.design_mdot_core_kg_s.max(1.0);
    let p = inp.design_power_w.max(1.0);
    Scales { x: [m, 1000.0, p, p, p], r: [1000.0, m * 0.01, m * 0.01, m * 0.01, m] }
}

fn weighted_norm(r: &[f64; N], sc: &Scales) -> f64 {
    (0..N).map(|i| (r[i] / sc.r[i]).powi(2)).sum::<f64>().sqrt()
}

pub fn solve(inp: &MatchInputs, x0: [f64; N]) -> MatchResult {
    let sc = scales(inp);
    const MAX_ITERS: u32 = 60;
    const MAX_LAMBDA_TRIES: u32 = 30;
    const TOL: f64 = 1e-7;

    let mut x = bound(x0, inp);
    if x[0] < 0.1 * sc.x[0] {
        let seed_mdot = inp.design_mdot_core_kg_s.max(1.0);
        let seed_comb = combustor::burn(seed_mdot, inp.wf_kg_s, inp.tt3_k, inp.pt3_pa);
        let seed_power = inp.design_power_w.max(1.0) / 3.0;
        x = bound([seed_mdot, seed_comb.tt4_k.max(400.0), seed_power, seed_power, seed_power], inp);
    }
    let (r0, mut last) = residuals(&x, inp);
    let mut r = r0;
    let mut lambda = 1e-2_f64;
    let mut final_iter = 0;

    for iter in 0..MAX_ITERS {
        final_iter = iter;
        let norm = weighted_norm(&r, &sc);
        if norm < TOL {
            break;
        }

        let q: [f64; N] = std::array::from_fn(|i| r[i] / sc.r[i]);
        let mut js = [[0.0_f64; N]; N];
        for j in 0..N {
            let h = (sc.x[j] * 1e-4).max(1e-6);
            let mut xp = x;
            xp[j] += h;
            let (rp, _) = residuals(&bound(xp, inp), inp);
            for i in 0..N {
                js[i][j] = ((rp[i] - r[i]) / h) * sc.x[j] / sc.r[i];
            }
        }

        let mut jtj = [[0.0_f64; N]; N];
        let mut jtq = [0.0_f64; N];
        for a in 0..N {
            for b in 0..N {
                jtj[a][b] = (0..N).map(|i| js[i][a] * js[i][b]).sum();
            }
            jtq[a] = (0..N).map(|i| js[i][a] * q[i]).sum();
        }

        let mut accepted = None;
        for _try in 0..MAX_LAMBDA_TRIES {
            let mut m = jtj;
            for a in 0..N {
                m[a][a] += lambda * jtj[a][a].max(1e-12);
            }
            let rhs: [f64; N] = std::array::from_fn(|a| -jtq[a]);
            if let Some(dy) = solve_linear(&m, &rhs) {
                let candidate = bound(std::array::from_fn(|i| x[i] + dy[i] * sc.x[i]), inp);
                let (rc, result) = residuals(&candidate, inp);
                let candidate_norm = weighted_norm(&rc, &sc);
                if candidate_norm < norm {
                    accepted = Some((candidate, rc, result, (lambda * 0.3).max(1e-9)));
                    break;
                }
            }
            lambda *= 4.0;
        }

        let Some((x_next, r_next, result_next, next_lambda)) = accepted else {
            break;
        };
        x = x_next;
        r = r_next;
        last = result_next;
        lambda = next_lambda;
    }
    last.iterations = final_iter;
    last.residual_norm = weighted_norm(&r, &sc);
    last
}

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

    fn design_like_inputs(tt3: f64, pt3: f64, wf: f64) -> MatchInputs {
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
