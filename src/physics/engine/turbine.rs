//! Turbine expansion: given the power the shaft actually needs to deliver
//! (see `spool.rs`/`mod.rs` for where that power comes from) and the gas
//! flow passing through, this finds the temperature and pressure drop from
//! energy conservation and the isentropic-efficiency relation, with no
//! separate turbine map needed:
//!
//! - `ΔTt = P / (mdot_gas * cp_gas)` (energy conservation: the power
//!   delivered to the shaft came from the gas cooling by exactly that
//!   much).
//! - `ΔTt_isentropic = ΔTt / eta` (definition of isentropic efficiency for
//!   an expansion).
//! - `Pt_out/Pt_in = (1 - ΔTt_isentropic/Tt_in)^(gamma/(gamma-1))` (the
//!   isentropic relation, solved for the pressure ratio that would give
//!   that isentropic temperature drop).
//!
//! This sidesteps needing an explicit turbine flow-capacity map (turbine
//! nozzle guide vanes are choked across almost the whole flight envelope in
//! a real engine, which is what a full nonlinear "engine matching" model
//! would use instead); the energy route gives the same station pressures
//! without an iterative solve, at the cost of not separately representing
//! choking. Documented simplification, appropriate for a real-time model.

use super::gas;

#[derive(Clone, Copy, Debug, Default)]
pub struct Expansion {
    pub tt_out_k: f64,
    pub pt_out_pa: f64,
    pub delta_tt_k: f64,
}

/// The most shaft power this turbine can take from the gas, given what it
/// is fed and the back pressure it must still exhaust against: expansion
/// can only go down to that pressure, so the isentropic temperature drop
/// (and with it the shaft work) is bounded. A turbine handed more power
/// than this in a reduced-order model would be extracting energy the gas
/// does not have left, which is what lets a downstream spool (the fan, last
/// in the chain) take too large a share at low power settings.
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
    // `pressure_ratio_from_temperature_ratio` returns Pout/Pin given
    // Tout/Tin (see `gas.rs`); temp_ratio < 1 here (an expansion), so this
    // is correctly < 1 and *multiplies* pt_in down, unlike the compressor
    // stages (`compressor.rs`), which also multiply but with a temp_ratio
    // > 1.
    let pressure_ratio_out_over_in = gas::pressure_ratio_from_temperature_ratio(temp_ratio, gamma);
    Expansion { tt_out_k: tt_in_k - delta_tt, pt_out_pa: pt_in_pa * pressure_ratio_out_over_in, delta_tt_k: delta_tt }
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
        // Same temperature drop (same power/energy balance)...
        assert!((good.tt_out_k - poor.tt_out_k).abs() < 1e-6);
        // ...but a less efficient turbine needs more pressure ratio to
        // achieve it, so it ends up at a lower exit pressure.
        assert!(poor.pt_out_pa < good.pt_out_pa);
    }

    #[test]
    fn the_extractable_power_is_bounded_by_the_pressure_left_to_expand_through() {
        // Same gas, same flow: half the inlet pressure, less work available.
        let high = max_power_w(1400.0, 2_000_000.0, 100.0, 101_325.0, 0.9, GAMMA_GAS);
        let low = max_power_w(1400.0, 1_000_000.0, 100.0, 101_325.0, 0.9, GAMMA_GAS);
        assert!(high > low && low > 0.0, "{high} {low}");
        // Nothing left to expand through: no work.
        assert_eq!(max_power_w(1400.0, 101_325.0, 100.0, 101_325.0, 0.9, GAMMA_GAS), 0.0);
        // The bound is what `expand` itself would need to reach that back
        // pressure: expanding at exactly this power lands at it (within a
        // little, the two share the same isentropic relation).
        let e = expand(1400.0, 2_000_000.0, 100.0, high, 0.9, GAMMA_GAS);
        assert!((e.pt_out_pa - 101_325.0).abs() < 5_000.0, "{}", e.pt_out_pa);
    }
}
