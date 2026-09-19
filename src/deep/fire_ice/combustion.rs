//! Per-zone combustion: a leaking flammable fluid (fuel, oil or hydraulic
//! fluid) burning in a protected zone, its heat release limited by both the
//! fuel available and the oxygen the zone's ventilation air can supply, and
//! its consequent zone temperature rise -- then spread to a neighbouring
//! zone through a real conductive heat-transfer interface, exactly the way
//! `physics::bays.rs`'s `WING_ROOT_TO_MAIN_AVIONICS_LINK_W_PER_K` couples
//! two bays (read as precedent, not depended on -- BRIEF rule 2, this
//! directory re-derives its own version). A neighbour only ignites when
//! its *own* leak/fuel source and the heat crossing the link push its *own*
//! temperature past its fluid's autoignition point -- an emergent
//! consequence of the energy balance, never a scripted "fire spreads"
//! flag.
//!
//! ## Sources
//! - Jet A-1 net heating value ~=43.1 MJ/kg (ASTM D1655 public
//!   specification for aviation turbine fuel; ~43 MJ/kg is the commonly
//!   cited public figure).
//! - MIL-PRF-23699 turbine oil (Mobil Jet Oil II data sheet, the same
//!   product `physics::engine::oil.rs` cites): heating value ~=42 MJ/kg
//!   (typical for a synthetic ester lubricant, public product literature),
//!   autoignition ~=344 C (Mobil Jet Oil II MSDS, public).
//! - Phosphate-ester hydraulic fluid (Skydrol-class, the fire-resistant
//!   fluid used in transport-category hydraulics including 5000 psi
//!   systems): heating value ~=27 MJ/kg (lower than hydrocarbons --
//!   phosphate esters are deliberately fire-resistant), autoignition
//!   ~=468 C (Skydrol LD-4 safety data sheet, public). Both figures are
//!   the reason phosphate-ester fluid, not a mineral oil, is mandatory in
//!   transport hydraulics.
//! - Stoichiometric air-fuel mass ratio ~=14.7 for a generic aviation
//!   hydrocarbon (standard combustion-engineering figure, e.g. any
//!   combustion textbook's Jet-A/kerosene value); used here for all three
//!   fluids as a GENERIC single figure (the exact stoichiometric ratio for
//!   oil/hydraulic fluid differs slightly but no public per-fluid figure
//!   for combustion in an aircraft-zone-fire context exists; kept in the
//!   same order of magnitude, GENERIC).

use super::util::clamp01;

/// A flammable fluid's combustion properties.
#[derive(Clone, Copy, Debug)]
pub struct Fluid {
    pub heating_value_j_kg: f64,
    pub autoignition_c: f64,
}

pub const JET_FUEL: Fluid = Fluid { heating_value_j_kg: 43.1e6, autoignition_c: 210.0 };
pub const TURBINE_OIL: Fluid = Fluid { heating_value_j_kg: 42.0e6, autoignition_c: 344.0 };
pub const HYDRAULIC_FLUID: Fluid = Fluid { heating_value_j_kg: 27.0e6, autoignition_c: 468.0 };

/// GENERIC: representative stoichiometric air:fuel mass ratio for a
/// generic aviation hydrocarbon/ester (module doc).
const STOICH_AFR: f64 = 14.7;

/// Once established, a flame does not require the temperature to still be
/// above the autoignition point every instant -- its own heat release
/// keeps it going as long as fuel and air continue, the same self-
/// sustaining behaviour real fires show (a flame is only put out by
/// removing fuel, removing air, or cooling/diluting it below what its own
/// heat release can overcome, e.g. a suppression agent). This quenching
/// margin is how far below the autoignition point the zone must cool
/// before an established flame is considered to have gone out on its own
/// (GENERIC: representative of typical flame quenching temperature
/// margins in combustion engineering, not the same thing as the ignition
/// threshold).
const QUENCH_MARGIN_C: f64 = 80.0;

/// Fuel and air availability driving one zone's combustion this tick.
#[derive(Clone, Copy, Debug, Default)]
pub struct ZoneSupply {
    /// Leaking flammable fluid available to burn, kg/s (0 if the leak has
    /// been isolated, e.g. by closing the fuel shutoff valve on fire
    /// pushbutton release).
    pub fuel_available_kg_s: f64,
    /// Ventilation/leakage air reaching the zone, kg/s -- the oxidiser
    /// supply. A sealed/starved zone (e.g. bleed shutoff after a fire
    /// pushbutton pull cuts nacelle ventilation air too) drives this
    /// toward zero.
    pub air_available_kg_s: f64,
    /// An external ignition source is present this tick (hot turbine case,
    /// electrical arc, the test pushbutton) -- without one, fuel below its
    /// autoignition temperature simply pools rather than igniting.
    pub ignition_source: bool,
    /// Fraction of the zone's suppression-agent design concentration
    /// present (0 none .. 1 or more, fully suppressing): see
    /// `extinguishing.rs`'s zone concentration output. Combustion's heat
    /// release scales down linearly with this and is fully quenched at or
    /// above 1.0, matching how a halogenated agent chemically interrupts
    /// the combustion radical chain once its design concentration is
    /// reached (public halon/clean-agent total-flooding design principle,
    /// e.g. NFPA 12A).
    pub suppression_fraction: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct ZoneCombustion {
    fluid: Fluid,
    temp_c: f64,
    thermal_mass_j_k: f64,
    /// Conductance from the zone's contents to the ambient/nacelle air it
    /// is ventilated by or conducts heat to, W/K.
    ambient_conductance_w_k: f64,
    burning: bool,
}

/// What burning in this zone did this tick.
#[derive(Clone, Copy, Debug, Default)]
pub struct CombustionState {
    pub temp_c: f64,
    pub burning: bool,
    pub heat_release_w: f64,
    pub burn_rate_kg_s: f64,
}

impl ZoneCombustion {
    pub fn new(fluid: Fluid, ambient_c: f64, thermal_mass_j_k: f64, ambient_conductance_w_k: f64) -> Self {
        Self { fluid, temp_c: ambient_c, thermal_mass_j_k, ambient_conductance_w_k, burning: false }
    }

    pub fn temp_c(&self) -> f64 {
        self.temp_c
    }

    pub fn is_burning(&self) -> bool {
        self.burning
    }

    /// One tick. `extra_heat_w` is heat arriving from outside this zone's
    /// own combustion (a neighbour's fire crossing an inter-zone link, see
    /// [`conductive_link_w`]); it can raise this zone's temperature enough
    /// to ignite its own fuel even with no `ignition_source` of its own,
    /// which is the actual physical mechanism of fire spread.
    pub fn step(&mut self, supply: &ZoneSupply, ambient_c: f64, extra_heat_w: f64, dt_s: f64) -> CombustionState {
        let dt = dt_s.max(0.0);
        let fuel = supply.fuel_available_kg_s.max(0.0);
        let air = supply.air_available_kg_s.max(0.0);

        // Ignition: needs fuel, needs air (a fire cannot start in a fully
        // inert/starved zone), and needs either an external ignition
        // source or the zone's own temperature already at or above the
        // fluid's autoignition point (heat crossing an inter-zone link can
        // provide this without any local ignition source).
        let hot_enough_to_ignite = self.temp_c >= self.fluid.autoignition_c;
        let can_ignite = fuel > 0.0 && air > 0.0 && (supply.ignition_source || hot_enough_to_ignite);

        // Self-sustaining: once burning, only quenches when fuel or air run
        // out, the zone cools well below the autoignition point (flame
        // quenching), or the suppression agent reaches its design
        // concentration.
        let quenched_by_suppression = clamp01(supply.suppression_fraction) >= 1.0;
        let starved = fuel <= 0.0 || air <= 0.0;
        let cooled_out = self.temp_c < self.fluid.autoignition_c - QUENCH_MARGIN_C;

        self.burning = if self.burning {
            !(starved || cooled_out || quenched_by_suppression)
        } else {
            can_ignite && !quenched_by_suppression
        };

        let (burn_rate_kg_s, heat_release_w) = if self.burning {
            let air_limited_fuel = air / STOICH_AFR;
            let burn_rate = fuel.min(air_limited_fuel);
            let suppression_derate = 1.0 - clamp01(supply.suppression_fraction);
            (burn_rate, burn_rate * self.fluid.heating_value_j_kg * suppression_derate)
        } else {
            (0.0, 0.0)
        };

        let ambient_loss_w = self.ambient_conductance_w_k * (self.temp_c - ambient_c);
        let net_w = heat_release_w + extra_heat_w - ambient_loss_w;
        self.temp_c += net_w / self.thermal_mass_j_k.max(1e-6) * dt;

        CombustionState { temp_c: self.temp_c, burning: self.burning, heat_release_w, burn_rate_kg_s }
    }
}

/// Conductive heat crossing an inter-zone link this tick, W (positive:
/// flows from `hot_c` to `cold_c`'s zone). The one deliberate literal
/// "wire" in a two-zone spread model, matching `physics::bays.rs`'s own
/// `heat_link_enabled` precedent: severing it (e.g. a firewall holding, or
/// the physical bay structure between two zones) is the controlled
/// variable a spread test cuts to prove the link -- not the local
/// combustion physics -- is what carries the fire onward.
pub fn conductive_link_w(conductance_w_k: f64, hot_zone_c: f64, cold_zone_c: f64) -> f64 {
    conductance_w_k.max(0.0) * (hot_zone_c - cold_zone_c)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(zone: &mut ZoneCombustion, supply: &ZoneSupply, ambient_c: f64, extra_heat_w: f64, dt_s: f64, ticks: u32) -> CombustionState {
        let mut out = CombustionState::default();
        for _ in 0..ticks {
            out = zone.step(supply, ambient_c, extra_heat_w, dt_s);
        }
        out
    }

    fn leak_and_air() -> ZoneSupply {
        ZoneSupply { fuel_available_kg_s: 0.02, air_available_kg_s: 2.0, ignition_source: true, suppression_fraction: 0.0 }
    }

    #[test]
    fn a_fuel_leak_with_air_and_an_ignition_source_ignites_and_heats_the_zone() {
        let mut zone = ZoneCombustion::new(HYDRAULIC_FLUID, 20.0, 50_000.0, 30.0);
        let out = run(&mut zone, &leak_and_air(), 20.0, 0.0, 1.0, 600);
        assert!(out.burning);
        assert!(out.temp_c > 200.0, "zone should heat well above ambient once burning, got {}", out.temp_c);
    }

    #[test]
    fn with_no_ignition_source_and_a_cold_zone_fuel_just_pools_unburnt() {
        let mut zone = ZoneCombustion::new(HYDRAULIC_FLUID, 20.0, 50_000.0, 30.0);
        let supply = ZoneSupply { ignition_source: false, ..leak_and_air() };
        let out = run(&mut zone, &supply, 20.0, 0.0, 1.0, 600);
        assert!(!out.burning);
        assert!((out.temp_c - 20.0).abs() < 5.0);
    }

    #[test]
    fn without_air_a_leak_cannot_sustain_combustion_even_with_ignition() {
        let mut zone = ZoneCombustion::new(JET_FUEL, 20.0, 50_000.0, 30.0);
        let starved = ZoneSupply { air_available_kg_s: 0.0, ..leak_and_air() };
        let out = run(&mut zone, &starved, 20.0, 0.0, 1.0, 600);
        assert!(!out.burning, "no oxidiser available, nothing can burn");
    }

    #[test]
    fn isolating_the_fuel_after_a_fire_starts_lets_it_burn_out_and_cool() {
        let mut zone = ZoneCombustion::new(JET_FUEL, 20.0, 50_000.0, 30.0);
        let burning_state = run(&mut zone, &leak_and_air(), 20.0, 0.0, 1.0, 300);
        assert!(burning_state.burning);

        // Fire pushbutton pulled: fuel shutoff valve closes (matches the
        // real interlock SetOnFireModule's own doc describes in
        // fire_and_smoke_protection.rs -- read as context, not depended
        // on).
        let isolated = ZoneSupply { fuel_available_kg_s: 0.0, ignition_source: false, ..leak_and_air() };
        let out = run(&mut zone, &isolated, 20.0, 0.0, 1.0, 1200);
        assert!(!out.burning, "starved of fuel the flame must go out");
        assert!(out.temp_c < burning_state.temp_c, "the zone must cool once the fire is out");
    }

    #[test]
    fn suppression_agent_at_design_concentration_extinguishes_the_fire() {
        let mut zone = ZoneCombustion::new(JET_FUEL, 20.0, 50_000.0, 30.0);
        let burning_state = run(&mut zone, &leak_and_air(), 20.0, 0.0, 1.0, 300);
        assert!(burning_state.burning);

        let suppressed = ZoneSupply { suppression_fraction: 1.0, ..leak_and_air() };
        let out = zone.step(&suppressed, 20.0, 0.0, 1.0);
        assert!(!out.burning);
        assert_eq!(out.heat_release_w, 0.0);
    }

    #[test]
    fn heat_crossing_a_link_can_ignite_a_neighbours_own_fuel_without_a_local_ignition_source() {
        // Zone 1 is already burning; zone 2 has its own leak (autoignition
        // 468 C for the fire-resistant hydraulic fluid) but no ignition
        // source of its own -- only the heat conducted across the link
        // from zone 1's fire should ever ignite it.
        let mut zone1 = ZoneCombustion::new(JET_FUEL, 20.0, 50_000.0, 30.0);
        let mut zone2 = ZoneCombustion::new(HYDRAULIC_FLUID, 20.0, 20_000.0, 20.0);
        let supply2 = ZoneSupply { ignition_source: false, ..leak_and_air() };

        for _ in 0..3000 {
            zone1.step(&leak_and_air(), 20.0, 0.0, 1.0);
            let link_w = conductive_link_w(200.0, zone1.temp_c(), zone2.temp_c());
            zone2.step(&supply2, 20.0, link_w, 1.0);
        }

        assert!(zone1.is_burning());
        assert!(zone2.is_burning(), "sustained heat from the neighbouring fire should have crossed the link and ignited zone 2's own leak, zone2 temp {}", zone2.temp_c());
    }

    #[test]
    fn cutting_the_link_prevents_the_same_spread() {
        let mut zone1 = ZoneCombustion::new(JET_FUEL, 20.0, 50_000.0, 30.0);
        let mut zone2 = ZoneCombustion::new(HYDRAULIC_FLUID, 20.0, 20_000.0, 20.0);
        let supply2 = ZoneSupply { ignition_source: false, ..leak_and_air() };

        for _ in 0..3000 {
            zone1.step(&leak_and_air(), 20.0, 0.0, 1.0);
            // Link severed: no conductance between the zones.
            let link_w = conductive_link_w(0.0, zone1.temp_c(), zone2.temp_c());
            zone2.step(&supply2, 20.0, link_w, 1.0);
        }

        assert!(zone1.is_burning());
        assert!(!zone2.is_burning(), "with the link cut, zone 2 must not ignite from zone 1's fire");
    }

    #[test]
    fn no_nan_at_rest_with_zero_dt_and_zero_supply() {
        let mut zone = ZoneCombustion::new(JET_FUEL, 20.0, 50_000.0, 30.0);
        let out = zone.step(&ZoneSupply::default(), 20.0, 0.0, 0.0);
        assert!(!out.temp_c.is_nan());
        assert_eq!(out.heat_release_w, 0.0);
    }
}
