use super::gas;

#[derive(Clone, Copy, Debug, Default)]
pub struct Expansion {
    pub tt_out_k: f64,
    pub pt_out_pa: f64,
    pub delta_tt_k: f64,
}

pub fn max_power_w(tt_in_k: f64, pt_in_pa: f64, mdot_gas_kg_s: f64, back_pressure_pa: f64, eta: f64, gamma: f64) -> f64 {
    if mdot_gas_kg_s <= 1e-6 || tt_in_k <= 1.0 || pt_in_pa <= back_pressure_pa.max(1.0) {
        return 0.0;
    }
    let pressure_ratio = (back_pressure_pa.max(1.0) / pt_in_pa).clamp(1e-6, 1.0);
    let delta_tt_isentropic = tt_in_k * (1.0 - pressure_ratio.powf((gamma - 1.0) / gamma));
    mdot_gas_kg_s * gas::CP_GAS * eta.max(0.3) * delta_tt_isentropic
}

pub fn expand(tt_in_k: f64, pt_in_pa: f64, mdot_gas_kg_s: f64, power_w: f64, eta: f64, gamma: f64) -> Expansion {
    if mdot_gas_kg_s <= 1e-6 || tt_in_k <= 1.0 {
        return Expansion { tt_out_k: tt_in_k, pt_out_pa: pt_in_pa, delta_tt_k: 0.0 };
    }
    let delta_tt = power_w / (mdot_gas_kg_s * super::gas::CP_GAS);
    let delta_tt_isentropic = delta_tt / eta.max(0.3);
    let temp_ratio = (1.0 - delta_tt_isentropic / tt_in_k).clamp(0.05, 1.0);
    let pressure_ratio_out_over_in = gas::pressure_ratio_from_temperature_ratio(temp_ratio, gamma);
    let bounded_delta_tt = delta_tt.min((1.0 - temp_ratio) * eta.max(0.3) * tt_in_k).max(0.0);
    Expansion { tt_out_k: tt_in_k - bounded_delta_tt, pt_out_pa: pt_in_pa * pressure_ratio_out_over_in, delta_tt_k: bounded_delta_tt }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::gas::GAMMA_GAS;

    #[test]
    fn zero_power_leaves_conditions_unchanged() {
        let e = expand(1400.0, 2_000_000.0, 100.0, 0.0, 0.9, GAMMA_GAS);
        assert!((e.tt_out_k - 1400.0).abs() < 1e-6);
        assert!((e.pt_out_pa - 2_000_000.0).abs() < 1.0);
    }

    #[test]
    fn more_power_extracted_drops_temperature_and_pressure_more() {
        let low = expand(1400.0, 2_000_000.0, 100.0, 5_000_000.0, 0.9, GAMMA_GAS);
        let high = expand(1400.0, 2_000_000.0, 100.0, 15_000_000.0, 0.9, GAMMA_GAS);
        assert!(high.tt_out_k < low.tt_out_k);
        assert!(high.pt_out_pa < low.pt_out_pa);
    }

    #[test]
    fn lower_efficiency_needs_a_bigger_pressure_drop_for_the_same_work() {
        let good = expand(1400.0, 2_000_000.0, 100.0, 10_000_000.0, 0.95, GAMMA_GAS);
        let poor = expand(1400.0, 2_000_000.0, 100.0, 10_000_000.0, 0.6, GAMMA_GAS);
        assert!((good.tt_out_k - poor.tt_out_k).abs() < 1e-6);
        assert!(poor.pt_out_pa < good.pt_out_pa);
    }

    #[test]
    fn the_extractable_power_is_bounded_by_the_pressure_left_to_expand_through() {
        let high = max_power_w(1400.0, 2_000_000.0, 100.0, 101_325.0, 0.9, GAMMA_GAS);
        let low = max_power_w(1400.0, 1_000_000.0, 100.0, 101_325.0, 0.9, GAMMA_GAS);
        assert!(high > low && low > 0.0, "{high} {low}");
        assert_eq!(max_power_w(1400.0, 101_325.0, 100.0, 101_325.0, 0.9, GAMMA_GAS), 0.0);
        let e = expand(1400.0, 2_000_000.0, 100.0, high, 0.9, GAMMA_GAS);
        assert!((e.pt_out_pa - 101_325.0).abs() < 5_000.0, "{}", e.pt_out_pa);
    }
}
