//! Combustor: a real energy balance from fuel flow and lower heating value,
//! not a lookup table -- this is what makes EGT, hot starts and hung starts
//! emerge from the model instead of being scripted, satisfying the brief's
//! "consequences come out of the model" rule.
//!
//! Standard combustor energy balance (basic thermodynamics, not specific to
//! any one engine): `mdot_fuel * LHV * eta_b + mdot_air * cp * Tt3 =
//! (mdot_air + mdot_fuel) * cp * Tt4`, solved for `Tt4`. One heat capacity
//! (air's) is used on both sides rather than switching to the hot-gas value
//! partway through the same balance, so `Tt4 == Tt3` at exactly zero fuel
//! flow (an energy balance that used two different heat capacities across
//! itself would not have that property, which would be unphysical). The
//! turbine expansion downstream (`turbine_flow.rs`) is where the higher
//! hot-gas heat capacity is used, once the flow genuinely is combustion
//! product.

use super::gas::CP_AIR_J_KG_K;
use super::params::{COMBUSTOR_EFFICIENCY, COMBUSTOR_PRESSURE_LOSS_FRAC, LHV_JET_A_J_KG};

#[derive(Clone, Copy, Debug, Default)]
pub struct Combustion {
    pub tt4_k: f64,
    pub pt4_pa: f64,
    /// Total gas mass flow leaving the combustor (air + fuel), kg/s.
    pub mdot_gas_kg_s: f64,
}

/// `mdot_air_kg_s` is the core air mass flow actually reaching the
/// combustor (after the load/bleed compressor has already taken its share
/// off a separate path -- see `power_section.rs`). `mdot_fuel_kg_s` is what
/// the fuel control unit is actually metering this tick, not a commanded
/// value.
pub fn burn(mdot_air_kg_s: f64, mdot_fuel_kg_s: f64, tt3_k: f64, pt3_pa: f64) -> Combustion {
    let mdot_air = mdot_air_kg_s.max(0.0);
    let mdot_fuel = mdot_fuel_kg_s.max(0.0);
    let mdot_gas = mdot_air + mdot_fuel;
    let tt4 = if mdot_gas <= 1e-6 {
        tt3_k
    } else {
        let heat_released = mdot_fuel * LHV_JET_A_J_KG * COMBUSTOR_EFFICIENCY;
        let heat_in_air = mdot_air * CP_AIR_J_KG_K * tt3_k;
        (heat_released + heat_in_air) / (mdot_gas * CP_AIR_J_KG_K)
    };
    Combustion {
        tt4_k: tt4,
        pt4_pa: (pt3_pa.max(0.0)) * (1.0 - COMBUSTOR_PRESSURE_LOSS_FRAC),
        mdot_gas_kg_s: mdot_gas,
    }
}

/// Solves `burn` in reverse for the fuel flow that reaches a target `tt4_k`
/// at the given air flow and inlet temperature -- used once, at start-up, to
/// calibrate this model's own design-point fuel flow
/// (`power_section::design_point`) against the design turbine-inlet
/// temperature chosen in `params.rs`, rather than asserting a fuel-flow
/// number with no derivation behind it.
pub fn fuel_flow_for_target_tt4_kg_s(mdot_air_kg_s: f64, tt3_k: f64, target_tt4_k: f64) -> f64 {
    let mdot_air = mdot_air_kg_s.max(0.0);
    let denominator = LHV_JET_A_J_KG * COMBUSTOR_EFFICIENCY - target_tt4_k * CP_AIR_J_KG_K;
    if denominator <= 0.0 {
        return 0.0;
    }
    (mdot_air * CP_AIR_J_KG_K * (target_tt4_k - tt3_k) / denominator).max(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_fuel_leaves_temperature_unchanged() {
        let c = burn(4.0, 0.0, 500.0, 400_000.0);
        assert!((c.tt4_k - 500.0).abs() < 1e-6);
    }

    #[test]
    fn more_fuel_at_fixed_airflow_raises_temperature() {
        let low = burn(4.0, 0.02, 500.0, 400_000.0);
        let high = burn(4.0, 0.05, 500.0, 400_000.0);
        assert!(high.tt4_k > low.tt4_k);
    }

    #[test]
    fn the_same_fuel_at_less_airflow_runs_hotter() {
        let plenty_of_air = burn(4.0, 0.05, 400.0, 400_000.0);
        let starved = burn(1.0, 0.05, 400.0, 400_000.0);
        assert!(starved.tt4_k > plenty_of_air.tt4_k);
    }

    #[test]
    fn pressure_drops_by_the_documented_loss_fraction() {
        let c = burn(4.0, 0.03, 500.0, 400_000.0);
        assert!((c.pt4_pa - 400_000.0 * (1.0 - COMBUSTOR_PRESSURE_LOSS_FRAC)).abs() < 1.0);
    }

    #[test]
    fn energy_is_conserved() {
        let mdot_air = 4.0;
        let mdot_fuel = 0.03;
        let tt3 = 480.0;
        let c = burn(mdot_air, mdot_fuel, tt3, 400_000.0);
        let energy_in =
            mdot_fuel * LHV_JET_A_J_KG * COMBUSTOR_EFFICIENCY + mdot_air * CP_AIR_J_KG_K * tt3;
        let energy_out = c.mdot_gas_kg_s * CP_AIR_J_KG_K * c.tt4_k;
        assert!((energy_in - energy_out).abs() / energy_in < 1e-9);
    }

    #[test]
    fn fuel_flow_for_target_solves_burn_exactly() {
        let mdot_air = 4.0;
        let tt3 = 480.0;
        let target = 1150.0;
        let fuel = fuel_flow_for_target_tt4_kg_s(mdot_air, tt3, target);
        let c = burn(mdot_air, fuel, tt3, 400_000.0);
        assert!((c.tt4_k - target).abs() < 1e-6, "{}", c.tt4_k);
    }

    #[test]
    fn no_airflow_never_produces_nan() {
        let c = burn(0.0, 0.0, 288.0, 100_000.0);
        assert!(c.tt4_k.is_finite());
        let fuel = fuel_flow_for_target_tt4_kg_s(0.0, 288.0, 1150.0);
        assert!(fuel.is_finite() && fuel >= 0.0);
    }
}
