//! Combustor: a real energy balance from fuel flow and lower heating value,
//! not a lookup table. This is what makes EGT (and hot/hung starts) emerge
//! from the model instead of being scripted.

use super::params::{COMBUSTOR_EFFICIENCY, COMBUSTOR_PRESSURE_LOSS_FRAC, FAR_STOICHIOMETRIC, LHV_JET_A1_J_KG};
use super::gas::CP_AIR;

#[derive(Clone, Copy, Debug, Default)]
pub struct Combustion {
    pub tt4_k: f64,
    pub pt4_pa: f64,
    /// Total gas mass flow leaving the combustor (air + fuel), kg/s.
    pub mdot_gas_kg_s: f64,
}

/// Energy balance: `mdot_fuel * LHV * eta_b + (mdot_air + mdot_fuel) *
/// cp_air * Tt3 = (mdot_air + mdot_fuel) * cp_air * Tt4`, solved for Tt4 —
/// equivalently `Tt4 = Tt3 + Q / (mdot_gas * cp)`.
///
/// The fuel stream carries its own sensible enthalpy in, at the combustor
/// inlet total temperature: it has come through the fuel/oil heat
/// exchanger and the burner feed arms in the hot casing, so it arrives far
/// nearer `Tt3` than absolute zero. Omitting that term — the form this
/// balance had before — is equivalent to injecting the fuel at 0 K. That is
/// invisible at a normal fuel/air ratio (0.025 of the mass flow), but it
/// sends `Tt4` to *zero kelvin* the moment the airflow does: with
/// `mdot_air = 0` and fuel still flowing there is no oxygen, so
/// `heat_released` is zero, and the balance then divides no enthalpy at all
/// by the fuel's own mass. A combustor with no air passing through it sits
/// at the temperature of what is in it, and those 0 K station temperatures
/// were propagating through the turbines into the gas path's plenums as
/// NaN. Both sides use
/// air's specific heat, not the hot-gas value the downstream turbines use
/// (`turbine.rs`'s `CP_GAS`): using two different heat capacities across
/// the same energy balance would leave `Tt4 != Tt3` at zero fuel flow, an
/// unphysical result this model must not produce (see the
/// `no_fuel_leaves_temperature_unchanged` test). Real combustion gas's
/// higher heat capacity only really matters once the flow *is* hot
/// combustion product, which is what the turbine expansions downstream
/// model; treating the combustor's own balance with one consistent heat
/// capacity is the standard simplification for a reduced-order cycle
/// model. `mdot_air_kg_s` is the core air mass flow actually reaching the
/// combustor (after bleed extraction has already been taken out
/// upstream). Pressure drops by the combustor's fractional total-pressure
/// loss.
pub fn burn(mdot_air_kg_s: f64, mdot_fuel_kg_s: f64, tt3_k: f64, pt3_pa: f64) -> Combustion {
    let mdot_air = mdot_air_kg_s.max(0.0);
    let mdot_fuel = mdot_fuel_kg_s.max(0.0);
    let mdot_gas = mdot_air + mdot_fuel;
    // Only the fuel the air can actually burn releases any heat. Kerosene
    // needs about 14.7 kg of air per kg of fuel, so once the fuel-air ratio
    // reaches 1/14.7 there is no oxygen left: the rest of the fuel leaves
    // the combustor unburnt, carrying its mass out but no heat with it.
    //
    // Without this limit, fuel going into a core whose airflow has
    // collapsed -- a stalled compressor, a seized spool, a start with the
    // air shut off -- divides a finite heat release by an airflow heading
    // for zero and hands the turbines a combustor exit in the tens of
    // thousands of kelvin. That is not a hot start; it is a division by
    // nothing. The real limit is chemical, and the real consequence of far
    // too much fuel for the air is a rich flame-out with the unburnt fuel
    // lighting further downstream, not an unbounded temperature.
    let burnt_fuel = mdot_fuel.min(mdot_air * FAR_STOICHIOMETRIC);
    let tt4 = if mdot_gas <= 1e-6 {
        tt3_k
    } else {
        let heat_released = burnt_fuel * LHV_JET_A1_J_KG * COMBUSTOR_EFFICIENCY;
        let heat_in = mdot_gas * CP_AIR * tt3_k;
        (heat_released + heat_in) / (mdot_gas * CP_AIR)
    };
    Combustion { tt4_k: tt4, pt4_pa: pt3_pa * (1.0 - COMBUSTOR_PRESSURE_LOSS_FRAC), mdot_gas_kg_s: mdot_gas }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_fuel_leaves_temperature_unchanged() {
        let c = burn(100.0, 0.0, 500.0, 1_000_000.0);
        assert!((c.tt4_k - 500.0).abs() < 1e-6);
    }

    #[test]
    fn more_fuel_at_fixed_airflow_raises_temperature() {
        let low = burn(100.0, 1.0, 500.0, 1_000_000.0);
        let high = burn(100.0, 2.0, 500.0, 1_000_000.0);
        assert!(high.tt4_k > low.tt4_k);
    }

    #[test]
    fn the_same_fuel_at_less_airflow_runs_hotter() {
        // The physical basis of a hot/hung start: too much fuel for the
        // available airflow at low spool speed drives EGT up on its own,
        // with no scripted "hot start" branch anywhere.
        let plenty_of_air = burn(150.0, 3.0, 400.0, 1_000_000.0);
        let starved = burn(15.0, 3.0, 400.0, 1_000_000.0);
        assert!(starved.tt4_k > plenty_of_air.tt4_k);
    }

    #[test]
    fn pressure_drops_by_the_documented_loss_fraction() {
        let c = burn(100.0, 1.0, 500.0, 1_000_000.0);
        assert!((c.pt4_pa - 950_000.0).abs() < 1.0);
    }

    #[test]
    fn energy_is_conserved() {
        let mdot_air = 120.0;
        let mdot_fuel = 2.0;
        let tt3 = 650.0;
        let c = burn(mdot_air, mdot_fuel, tt3, 1_000_000.0);
        // Both streams enter at Tt3: the air, and the fuel with its own
        // sensible enthalpy (see `burn`'s docs — the fuel is not injected
        // at 0 K).
        let energy_in = mdot_fuel * LHV_JET_A1_J_KG * COMBUSTOR_EFFICIENCY + (mdot_air + mdot_fuel) * CP_AIR * tt3;
        let energy_out = c.mdot_gas_kg_s * CP_AIR * c.tt4_k;
        assert!((energy_in - energy_out).abs() / energy_in < 1e-9);
    }

    #[test]
    fn with_no_air_at_all_the_combustor_sits_at_its_inlet_temperature() {
        // Nothing to burn the fuel in, so no heat release — and nothing
        // that can take the gas to 0 K either.
        let c = burn(0.0, 0.5, 700.0, 400_000.0);
        assert!((c.tt4_k - 700.0).abs() < 1e-9, "{}", c.tt4_k);
    }
}
