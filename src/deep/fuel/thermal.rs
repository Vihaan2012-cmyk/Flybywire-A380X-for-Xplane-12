//! Fuel temperature extensions and water/wax contamination (backlog item 4):
//! FCOC (Fuel-Cooled Oil Cooler) heat into the feed tanks, cold-soak
//! freezing point by fuel type, wax formation approaching the freeze point,
//! and free water settling and icing in the feed filters.
//!
//! `src/fuel.rs` already has a real per-tank heat-transfer model (module doc
//! there, "FUEL-004"): internal/external convective coefficients, wetted
//! area from live tank capacity, and -- as of the hydraulics/IDG workstream
//! -- heat rejected into the engine feed tanks from the hydraulic pumps and
//! generators via the HHX (`fuel.rs`'s own `HYD_PUMP_LOSS_FRACTION`/
//! `IDG_LOSS_FRACTION`, citing "Hydraulics onboard the A380", Power &
//! Motion Technology). What it does not consume, despite the plugin already
//! publishing it, is `ENGINE_FCOC_HEAT_W:n` (`src/engine_commands.rs:263`,
//! fed by `src/physics/engine/oil.rs`'s own FCOC model, `oil.rs:83,197`,
//! `out.fuel_heat_w`) -- the heat the *engine oil cooler* rejects into fuel,
//! which is a different, larger heat source than the hydraulic HHX and is
//! simply never added to a tank's energy balance today. Nor does `fuel.rs`
//! model fuel type (it hard-codes Jet A, `JET_A_LBS_PER_GAL`), wax formation,
//! or water/ice in the filters at all. This module adds all four as
//! self-contained functions ready to be folded into that existing energy
//! balance once the lead wires this directory in, without re-deriving the
//! convective heat-transfer coefficients `fuel.rs` already has right.

/// One tank's FCOC-driven temperature rise this tick from the engine oil
/// cooler's own rejected heat (`ENGINE_FCOC_HEAT_W:n`), by a first-order
/// energy balance over the tank's current fuel mass:
/// `dT = Q * dt / (m * c_p)` (elementary calorimetry, the same functional
/// form `fuel.rs`'s own convective terms use before integrating). Guarded
/// against a momentarily near-empty tank (returns 0 rather than dividing by
/// a vanishing mass, matching this push's "no NaN/div-by-zero at rest"
/// rule).
pub fn fcoc_temperature_rise_k(fcoc_heat_w: f64, dt_s: f64, tank_fuel_mass_kg: f64, specific_heat_j_kgk: f64) -> f64 {
    if tank_fuel_mass_kg <= 1e-3 || specific_heat_j_kgk <= 0.0 {
        return 0.0;
    }
    (fcoc_heat_w.max(0.0) * dt_s.max(0.0)) / (tank_fuel_mass_kg * specific_heat_j_kgk)
}

/// The fuel types this model distinguishes, with their specification
/// maximum freeze points (ASTM D1655 for Jet A/Jet A-1; MIL-DTL-83133 for
/// JP-8, which is essentially Jet A-1 plus additives and shares its freeze
/// spec). `src/fuel.rs`'s own `FUEL_FREEZE_POINT_C` constant already cites
/// Jet A's -40 C figure for the one fuel type it models; this enum adds the
/// colder types real long-haul operators (including many A380 routes) often
/// actually uplift.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FuelType {
    /// ASTM D1655: freeze point -40 C max.
    JetA,
    /// ASTM D1655: freeze point -47 C max (the colder, more common
    /// international kerosene grade).
    JetA1,
    /// MIL-DTL-83133: freeze point -47 C max.
    Jp8,
}
impl FuelType {
    pub fn freeze_point_c(self) -> f64 {
        match self {
            FuelType::JetA => -40.0,
            FuelType::JetA1 => -47.0,
            FuelType::Jp8 => -47.0,
        }
    }
}

/// Wax (paraffin crystal) formation fraction, 0 (clear fuel) .. 1 (at or
/// below the specification freeze point, where standardised freeze-point
/// test methods define fuel as no longer pourable). Real kerosene begins
/// precipitating wax crystals at its *cloud point*, several degrees above
/// the freeze point, well before it stops flowing; `WAX_ONSET_MARGIN_K`
/// (`GENERIC`, no public cloud-point figure for FlyByWire's modelled fuel)
/// sets that margin, and the fraction ramps linearly from the cloud point
/// down to the freeze point.
pub const WAX_ONSET_MARGIN_K: f64 = 10.0;
pub fn wax_fraction(temp_c: f64, fuel: FuelType) -> f64 {
    let freeze = fuel.freeze_point_c();
    let cloud = freeze + WAX_ONSET_MARGIN_K;
    if temp_c >= cloud {
        return 0.0;
    }
    ((cloud - temp_c) / WAX_ONSET_MARGIN_K).clamp(0.0, 1.0)
}

/// Free water in a tank: denser than kerosene, it settles to the tank's
/// lowest point -- exactly where the boost-pump/filter inlet sits -- so it
/// is the first thing drawn in as the fuel is used. `free_water_fraction` is
/// the fault input, 0 (dry) .. 1 (a `GENERIC` ceiling volume fraction judged
/// a severe but plausible undrained contamination level, not a spec limit);
/// below 0 C any of that water that reaches the filter freezes into ice
/// crystals that block its mesh (a well-documented real hazard, the reason
/// aircraft fuel systems carry FSII additive or filter heaters). Blockage
/// ramps from 0 at 0 C to the full `free_water_fraction` at
/// `ICE_FULL_BLOCKAGE_MARGIN_K` below freezing, `GENERIC` since no public
/// figure gives the exact ice-formation-rate-vs-subcooling curve for this
/// aircraft's filters.
pub const ICE_FULL_BLOCKAGE_MARGIN_K: f64 = 5.0;
pub fn filter_ice_blockage_fraction(temp_c: f64, free_water_fraction: f64, anti_ice_heater_on: bool) -> f64 {
    if anti_ice_heater_on || temp_c >= 0.0 {
        return 0.0;
    }
    let subcool = (0.0 - temp_c).min(ICE_FULL_BLOCKAGE_MARGIN_K);
    free_water_fraction.clamp(0.0, 1.0) * (subcool / ICE_FULL_BLOCKAGE_MARGIN_K)
}

/// Combined filter differential-pressure multiplier from wax and ice
/// together (both narrow the same mesh, so their blocked-area fractions add,
/// capped at fully blocked): `1 / (1 - blocked)^2`-style rise would overstate
/// a single narrow filter without a real orifice model of the filter itself,
/// so this returns the blocked-area fraction directly (0 clear .. 1 fully
/// blocked) for a caller to apply to its own pressure-drop or flow-capacity
/// model, rather than inventing a second orifice law here.
pub fn filter_blockage_fraction(wax: f64, ice: f64) -> f64 {
    (wax.clamp(0.0, 1.0) + ice.clamp(0.0, 1.0)).min(1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fcoc_heat_raises_temperature_and_needs_no_mass_to_avoid_nan() {
        let dt = fcoc_temperature_rise_k(50_000.0, 1.0, 2000.0, 2000.0);
        assert!(dt > 0.0);
        assert_eq!(fcoc_temperature_rise_k(50_000.0, 1.0, 0.0, 2000.0), 0.0);
        assert_eq!(fcoc_temperature_rise_k(50_000.0, 0.0, 2000.0, 2000.0), 0.0);
    }

    #[test]
    fn more_heat_or_more_time_raises_temperature_more() {
        let a = fcoc_temperature_rise_k(10_000.0, 1.0, 5000.0, 2000.0);
        let b = fcoc_temperature_rise_k(20_000.0, 1.0, 5000.0, 2000.0);
        assert!(b > a);
        let c = fcoc_temperature_rise_k(10_000.0, 2.0, 5000.0, 2000.0);
        assert!(c > a);
    }

    #[test]
    fn jet_a1_and_jp8_freeze_colder_than_jet_a() {
        assert_eq!(FuelType::JetA.freeze_point_c(), -40.0);
        assert_eq!(FuelType::JetA1.freeze_point_c(), -47.0);
        assert_eq!(FuelType::Jp8.freeze_point_c(), -47.0);
        assert!(FuelType::JetA1.freeze_point_c() < FuelType::JetA.freeze_point_c());
    }

    #[test]
    fn wax_forms_above_the_freeze_point_and_saturates_at_it() {
        let fuel = FuelType::JetA;
        assert_eq!(wax_fraction(0.0, fuel), 0.0, "well above cloud point");
        let partial = wax_fraction(fuel.freeze_point_c() + 5.0, fuel);
        assert!(partial > 0.0 && partial < 1.0);
        assert_eq!(wax_fraction(fuel.freeze_point_c(), fuel), 1.0);
        assert_eq!(wax_fraction(fuel.freeze_point_c() - 20.0, fuel), 1.0, "never exceeds 1");
    }

    #[test]
    fn colder_fuel_types_delay_wax_formation_at_the_same_temperature() {
        let t = -42.0;
        assert!(wax_fraction(t, FuelType::JetA) > wax_fraction(t, FuelType::JetA1));
    }

    #[test]
    fn no_ice_above_freezing_or_with_the_heater_on_or_with_no_water() {
        assert_eq!(filter_ice_blockage_fraction(5.0, 0.5, false), 0.0);
        assert_eq!(filter_ice_blockage_fraction(-10.0, 0.5, true), 0.0);
        assert_eq!(filter_ice_blockage_fraction(-10.0, 0.0, false), 0.0);
    }

    #[test]
    fn ice_blockage_grows_with_subcooling_and_water_content_and_saturates() {
        let shallow = filter_ice_blockage_fraction(-1.0, 0.5, false);
        let deep = filter_ice_blockage_fraction(-10.0, 0.5, false);
        assert!(deep > shallow);
        assert!((deep - 0.5).abs() < 1e-9, "fully sub-cooled reaches the full water fraction");
        let less_water = filter_ice_blockage_fraction(-10.0, 0.1, false);
        assert!(less_water < deep);
    }

    #[test]
    fn combined_blockage_adds_and_caps_at_one() {
        assert!((filter_blockage_fraction(0.3, 0.3) - 0.6).abs() < 1e-9);
        assert_eq!(filter_blockage_fraction(0.8, 0.8), 1.0);
        assert_eq!(filter_blockage_fraction(0.0, 0.0), 0.0);
    }
}
