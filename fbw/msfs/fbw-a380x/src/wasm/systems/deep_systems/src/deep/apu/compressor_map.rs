use super::gas;

#[derive(Clone, Copy, Debug)]
pub struct Spec {
    pub pr_design: f64,
    pub eta_design: f64,
    pub mdot_corrected_design_kg_s: f64,
    pub efficiency_falloff: f64,
    pub surge_margin_design_frac: f64,
    pub surge_line_flatness: f64,
    pub choke_flow_multiple: f64,
    pub erosion_efficiency_loss: f64,
}

impl Spec {
    pub fn degraded(&self, efficiency_loss_frac: f64) -> Spec {
        Spec {
            erosion_efficiency_loss: (self.erosion_efficiency_loss
                + efficiency_loss_frac.clamp(0.0, 1.0))
            .clamp(0.0, 0.7),
            ..*self
        }
    }
}

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
    pub surge_margin: f64,
    pub in_surge: bool,
    pub choked: bool,
}

const SPEED_EXPONENT: f64 = 1.8;

pub fn surge_corrected_flow_kg_s(spec: &Spec, n_corrected_frac: f64) -> f64 {
    let n = n_corrected_frac.max(0.0);
    let design_surge = spec.mdot_corrected_design_kg_s * spec.surge_margin_design_frac.clamp(0.01, 0.9);
    let exponent = (1.0 - spec.surge_line_flatness.clamp(0.0, 0.95)).max(0.05);
    design_surge * n.powf(exponent)
}

pub fn choke_corrected_flow_kg_s(spec: &Spec, n_corrected_frac: f64) -> f64 {
    let n = n_corrected_frac.max(0.0);
    spec.mdot_corrected_design_kg_s * spec.choke_flow_multiple.max(1.01) * n
}

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
    let in_surge = n > 0.05 && requested < surge_flow;
    let choked = requested > choke_flow;

    let eta_clean =
        (spec.eta_design * (1.0 - spec.efficiency_falloff.clamp(0.0, 1.0) * (1.0 - n).powi(2)))
            .clamp(0.35 * spec.eta_design, spec.eta_design);
    let eta_nominal = eta_clean * (1.0 - spec.erosion_efficiency_loss.clamp(0.0, 0.7));

    let (mdot_corrected, specific_work, eta) = if in_surge {
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
        assert!((eroded.tt_out_k - healthy.tt_out_k).abs() < 1e-6);
        assert!(eroded.pr < healthy.pr);
        assert!(eroded.eta < healthy.eta);
    }

    #[test]
    fn surge_margin_narrows_at_part_speed() {
        let s = spec();
        let nominal_full = s.mdot_corrected_design_kg_s * 1.0;
        let nominal_half = s.mdot_corrected_design_kg_s * 0.5;
        let surge_full = surge_corrected_flow_kg_s(&s, 1.0);
        let surge_half = surge_corrected_flow_kg_s(&s, 0.5);
        let margin_full = nominal_full / surge_full;
        let margin_half = nominal_half / surge_half;
        assert!(margin_half < margin_full, "{margin_half} {margin_full}");
    }
}
