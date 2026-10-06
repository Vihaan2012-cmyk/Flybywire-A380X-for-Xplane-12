use super::gas::{self, CP_AIR, GAMMA_AIR};

#[derive(Clone, Copy, Debug)]
pub struct Spec {
    pub pr_design: f64,
    pub eta_design: f64,
    pub mdot_corrected_design_kg_s: f64,
    pub efficiency_falloff: f64,
    pub efficiency_loss_fraction: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Stage {
    pub tt_out_k: f64,
    pub pt_out_pa: f64,
    pub specific_work_j_kg: f64,
    pub mdot_kg_s: f64,
    pub power_w: f64,
}

fn thermo(spec: &Spec, tt_in_k: f64, pt_in_pa: f64, n_corrected_frac: f64) -> (f64, f64, f64) {
    let n = n_corrected_frac.max(0.0);

    const SPEED_EXPONENT: f64 = 1.8;
    let isentropic_temp_ratio_design = gas::temperature_ratio_from_pressure_ratio(spec.pr_design, GAMMA_AIR);
    let w_design_per_kelvin = CP_AIR * (isentropic_temp_ratio_design - 1.0) / spec.eta_design;

    let specific_work = w_design_per_kelvin * tt_in_k * n.powf(SPEED_EXPONENT);
    let tt_out = tt_in_k + specific_work / CP_AIR;

    let eta = (spec.eta_design * (1.0 - spec.efficiency_falloff * (1.0 - n).powi(2))).clamp(0.35, spec.eta_design)
        * (1.0 - spec.efficiency_loss_fraction.clamp(0.0, 1.0));
    let ideal_work = specific_work * eta;
    let pt_out = pt_in_pa * gas::pressure_ratio_from_temperature_ratio(1.0 + ideal_work / (CP_AIR * tt_in_k), GAMMA_AIR);

    (specific_work, tt_out, pt_out)
}

pub fn stage(spec: &Spec, tt_in_k: f64, pt_in_pa: f64, n_corrected_frac: f64) -> Stage {
    let n = n_corrected_frac.max(0.0);
    let (specific_work, tt_out, pt_out) = thermo(spec, tt_in_k, pt_in_pa, n);

    let mdot_corrected = spec.mdot_corrected_design_kg_s * n;
    let mdot = mdot_corrected * (pt_in_pa / super::params::P_REF_PA) / (tt_in_k / super::params::T_REF_K).sqrt();

    Stage { tt_out_k: tt_out, pt_out_pa: pt_out, specific_work_j_kg: specific_work, mdot_kg_s: mdot, power_w: mdot * specific_work }
}

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
        assert!((4.0..14.0).contains(&ratio), "{ratio}");
    }
}
