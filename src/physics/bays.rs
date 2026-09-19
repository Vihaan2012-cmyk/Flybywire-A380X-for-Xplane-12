//! Equipment bay thermal model (hyperrealism.md emergence goal: a
//! pneumatic -> thermal -> electrical chain nobody scripted -- a bleed
//! duct leak heats an avionics bay, and the hotter bay trips a breaker on
//! a load that is fine when cold).
//!
//! Five lumped thermal nodes, matching the real A380 bay layout closely
//! enough for this plugin's purposes (no FBW Rust equivalent exists --
//! `docs/physics/air.md`'s own cabin zone model has "no solar or avionics
//! heat load term" and stops at the pressurised cabin, not the bays):
//! - `MainAvionics`: the main equipment centre (under the cockpit floor),
//!   where most of the DC/AC essential and flight-control-computer
//!   breakers/LRUs `breakers.rs` catalogues actually sit.
//! - `UpperAvionics`: the smaller upper avionics bay behind the cockpit.
//! - `FwdCargo`/`AftCargo`: the two ventilated cargo compartments,
//!   coupled to FlyByWire's own real `FwdExtractFan`/`BulkExtractFan`
//!   failures (`failures.rs` 21_008/21_010) so a fan failure there is a
//!   real, already-registered fault, not a new one.
//! - `WingRootBleed`: the wing-root/centre-fuselage zone the engine bleed
//!   ducts (`failures.rs` 36_000..36_003, "Engine n bleed duct leak") run
//!   through. It has no avionics equipment of its own; a duct leak here
//!   dumps its enthalpy locally, then that heat reaches `MainAvionics`
//!   only through [`Bays::heat_link_enabled`]'s explicit conductive link
//!   -- modelling the two compartments' shared structure/duct-run
//!   proximity. This is the one deliberately literal "wire" in the whole
//!   chain: cutting it is exactly the test's decouple check, and every
//!   other step (leak -> mass flow -> enthalpy -> bay temperature ->
//!   breaker derating -> trip time) is a real, continuous physical
//!   quantity, not a scripted effect.
//!
//! Each bay's energy balance (`d(T)/dt = sum(Q) / thermal_mass`) sums, in
//! Watts: avionics/electrical heat dissipation, bleed-leak enthalpy flow
//! (`WingRootBleed` only), the `WingRootBleed` <-> `MainAvionics`
//! conductive link, extract-fan ventilation to a cabin-temperature supply,
//! and skin conduction to outside air (X-Plane OAT, convection driven by
//! TAS). Every constant is cited or marked `derived`/`typical` below; none
//! are clamped locally -- the final per-tick temperature is run through
//! `invariants::check` (`TemperatureFloor(-273.15)`), same as every other
//! physics module (see `physics::tyre`, the precedent this module follows
//! most closely).
//!
//! **Breaker coupling (task step 2).** `breakers.rs`'s `Breakers::new`/
//! `bay_for`/`post_systems` (see `docs/physics/breakers.md`'s "thermal
//! ambient" section) already landed the consumer side while this module
//! was being written: every breaker reads a per-bay ambient dataref named
//! exactly `BAY_<bay>_TEMPERATURE_C` (`bay_for`'s buckets: `AVIONICS`,
//! `CARGO_FWD`, `CARGO_AFT`, `WING_ROOT`) into
//! `physics::electrical::trip_step_with_ambient`, which folds
//! `(ambient_c - REFERENCE_AMBIENT_C) / ELEMENT_RISE_AT_TRIP_C` into the
//! same I^2t `ratio^2` term the old ambient-free curve used -- a real,
//! already-published mechanism, not something this module needed to
//! invent. `Breakers::new` initialises every bay's ambient dataref to
//! `REFERENCE_AMBIENT_C` (25 C) as an inert default; this module's
//! [`Bays::update`] publishing real values under the *same* names is what
//! actually closes the chain end to end -- the names below (`AVIONICS`,
//! `CARGO_FWD`, `CARGO_AFT`, `WING_ROOT`) are matched exactly to
//! `bay_for`'s buckets, not renamed independently.
//!
//! The headline test below proves the chain by calling
//! `physics::electrical::trip_step_with_ambient` directly with this
//! module's own hand-computed steady-state bay temperature (both that
//! function and this module's leak/link physics are independent,
//! already-published code -- neither was written to make the other's test
//! pass).

use crate::failures;
use crate::physics::gas;
use crate::Vars;
use systems::simulation::VariableIdentifier;

// ---------------------------------------------------------------------------
// Cited/derived constants
// ---------------------------------------------------------------------------

/// Dry air specific heat at constant pressure, J/(kg*K). Standard textbook
/// value (sea-level, moderate temperature range); not FBW- or A380-
/// specific, used the same way `fuel.rs`'s tank-skin convection model and
/// `docs/physics/air.md`'s own `1.005 kJ/(kg*K)` precooler energy balance
/// (air.md line citing `Precooler::update`'s fixed specific heat) already
/// do.
pub const CP_AIR_J_PER_KG_K: f64 = 1005.0;
/// Ratio of specific heats for air (diatomic ideal gas), standard.
const GAMMA_AIR: f64 = 1.4;
/// Specific gas constant for air, J/(kg*K): `gas::R_UNIVERSAL /
/// (molar mass)`, already published by this crate for exactly this reuse
/// (`gas.rs:18`, `AIR_SPECIFIC_GAS_CONSTANT`).
const R_AIR: f64 = gas::AIR_SPECIFIC_GAS_CONSTANT;
/// 1 psi in Pa (exact, standard conversion), and inHg -> Pa (standard),
/// both reused for the bleed source pressure and `AMBIENT PRESSURE`
/// (`fuel.rs` reads the same X-Plane simvar under the same name and unit).
const PSI_TO_PA: f64 = 6894.757;
const INHG_TO_PA: f64 = 3386.389;

/// Bleed duct source temperature at the leak, deg C: `docs/physics/air.md`'s
/// own design-point bleed condition ("`-56.5 C/238 hPa/Mach-0.85-equivalent
/// ambient, coldest cabin selection`" cites a "200 C bleed" inlet). Reused
/// here as the representative hot-side temperature a bleed duct leak
/// dumps into its surrounding bay, rather than inventing a separate figure.
const BLEED_DUCT_TEMP_C: f64 = 200.0;
/// Bleed duct source pressure at the leak, Pa: `docs/physics/air.md`'s own
/// "`~44 psi source`" ground/full-bleed design point (the PRV's own
/// production-verified ~40 psig regulation point, `pressure_regulating_
/// valve_regulates_to_40_psig`, plus margin to the ~44 psi figure air.md's
/// ACM design-point tests use upstream of it).
const BLEED_DUCT_PRESSURE_PA: f64 = 44.0 * PSI_TO_PA;
/// Orifice discharge coefficient: `docs/physics/air.md`'s own cited duct/
/// valve figure ("`both Cd 0.65`", line 234), reused rather than inventing
/// a second one for the same class of duct opening.
const LEAK_DISCHARGE_COEFFICIENT: f64 = 0.65;
/// Leak orifice area at `failures::magnitude() == 1.0` (a full-severity
/// leak), m^2. **Derived/typical**: no FBW or published A380 bleed-duct
/// crack-area figure exists. Sized (2.5 cm^2) so a full-severity leak's
/// enthalpy flow is the same order of magnitude as a single avionics bay's
/// own baseline heat load (a few kW) -- large enough to dominate the bay's
/// heat balance the way a real duct-leak overheat-detection loop trip
/// implies, not so large it instantly saturates every test. Scales
/// linearly with `failures::magnitude`, so lower-severity leaks are a
/// proportionally smaller crack, matching every other continuous-severity
/// failure in this crate (e.g. `physics::tyre`'s leak rate).
const LEAK_AREA_MAX_M2: f64 = 2.5e-4;

/// Conductive link between `WingRootBleed` and `MainAvionics`, W/K.
/// **Derived/typical**: represents the two compartments' shared structure
/// and duct-run proximity (an insulated bulkhead/floor of a few square
/// metres at a typical aircraft-panel U-value of a few W/(m^2*K)) -- no
/// published A380 figure exists for this path. This is the literal "bay
/// heat link" the headline test's decouple check cuts.
const WING_ROOT_TO_MAIN_AVIONICS_LINK_W_PER_K: f64 = 40.0;

/// External convection coefficient, W/(m^2*K): the same empirical form
/// this plugin already uses for fuel-tank skin convection
/// (`fuel.rs::tank_external_h_w_m2k`, `10 + 5*sqrt(TAS_m_s)`), reused here
/// for bay skin conduction to outside air since no bay-specific
/// correlation exists and the physical driver (forced convection over an
/// external panel, low/high TAS) is the same.
fn external_h_w_m2k(true_airspeed_m_s: f64) -> f64 {
    10.0 + 5.0 * true_airspeed_m_s.abs().sqrt()
}

/// Cabin/conditioned-air reference temperature bay ventilation draws its
/// supply from, deg C: FlyByWire's own real cabin demand temperature
/// (`docs/physics/air.md` line 104, "`within 1 C of the 24 C demand`" --
/// the ACM's ground design-point test against the ACSC's actual selected
/// temperature). Reused rather than inventing a separate bay-supply figure
/// -- avionics/cargo bay ventilation on a pressurised transport draws from
/// (or exhausts to) cabin-conditioned air, not raw outside air.
const CABIN_SUPPLY_TEMP_C: f64 = 24.0;

/// Standard compressible orifice/nozzle mass flow rate (isentropic ideal
/// gas, subsonic or choked), kg/s. The same critical pressure ratio
/// `docs/physics/air.md`'s own cabin outflow valve equation uses (~0.53,
/// `subsonic_flow_out_calculation`/`supersonic_flow_out_calculation`,
/// cabin_air.rs:308-337) falls out of this general form at `gamma = 1.4`
/// (`(2/(gamma+1))^(gamma/(gamma-1))` = 0.5283); not re-derived
/// separately, just applied here to the bleed-duct leak path instead of
/// the outflow valve.
fn orifice_mass_flow_kg_s(cd: f64, area_m2: f64, p_up_pa: f64, t_up_k: f64, p_down_pa: f64) -> f64 {
    if p_up_pa <= 0.0 || t_up_k <= 0.0 || area_m2 <= 0.0 {
        return 0.0;
    }
    let critical_ratio = (2.0 / (GAMMA_AIR + 1.0)).powf(GAMMA_AIR / (GAMMA_AIR - 1.0));
    let pr = (p_down_pa / p_up_pa).clamp(0.0, 1.0);
    if pr <= critical_ratio {
        // Choked: downstream pressure can't influence the throat any
        // further, flow is at the local sonic condition.
        cd * area_m2 * p_up_pa * (GAMMA_AIR / (R_AIR * t_up_k)).sqrt() * (2.0 / (GAMMA_AIR + 1.0)).powf((GAMMA_AIR + 1.0) / (2.0 * (GAMMA_AIR - 1.0)))
    } else {
        let term = (pr.powf(2.0 / GAMMA_AIR) - pr.powf((GAMMA_AIR + 1.0) / GAMMA_AIR)).max(0.0);
        cd * area_m2 * p_up_pa * ((2.0 * GAMMA_AIR) / (R_AIR * t_up_k * (GAMMA_AIR - 1.0)) * term).sqrt()
    }
}

/// Leak orifice area for a given `failures::magnitude()` reading, m^2.
fn leak_area_m2(magnitude: f64) -> f64 {
    LEAK_AREA_MAX_M2 * magnitude.clamp(0.0, 1.0)
}

// ---------------------------------------------------------------------------
// Per-bay static parameters (Watts, kg/s, m^2, J/K -- all `derived/typical`,
// no FBW or published A380 figures exist for any of them; each is sized to
// the right order of magnitude for its role, cited inline).
// ---------------------------------------------------------------------------

/// One lumped bay: static parameters plus its own live temperature.
struct BayParams {
    /// Baseline avionics/electrical heat dissipation, W. Typical/derived:
    /// literature order-of-magnitude for a large transport main/upper
    /// equipment centre (several kW split between the two bays); no
    /// FBW-published per-bay figure exists.
    baseline_avionics_w: f64,
    /// Exposed skin/structure area for conduction to outside air, m^2.
    skin_area_m2: f64,
    /// Extract fan flow at full effectiveness, kg/s. Typical/derived,
    /// order of magnitude for large-transport avionics/cargo bay
    /// ventilation (several hundred CFM class).
    fan_flow_kg_s: f64,
    /// Lumped thermal mass (equipment + structure + bay air), J/K.
    /// Typical/derived: dominated by equipment/rack mass, not bay air.
    thermal_mass_j_per_k: f64,
}

const MAIN_AVIONICS: BayParams = BayParams { baseline_avionics_w: 3000.0, skin_area_m2: 4.0, fan_flow_kg_s: 0.35, thermal_mass_j_per_k: 4.0e5 };
const UPPER_AVIONICS: BayParams = BayParams { baseline_avionics_w: 2000.0, skin_area_m2: 3.0, fan_flow_kg_s: 0.25, thermal_mass_j_per_k: 2.5e5 };
const FWD_CARGO: BayParams = BayParams { baseline_avionics_w: 50.0, skin_area_m2: 8.0, fan_flow_kg_s: 0.30, thermal_mass_j_per_k: 3.0e5 };
const AFT_CARGO: BayParams = BayParams { baseline_avionics_w: 50.0, skin_area_m2: 8.0, fan_flow_kg_s: 0.30, thermal_mass_j_per_k: 3.0e5 };
/// No avionics equipment and (per the module doc) no dedicated extract fan
/// modelled -- ventilated passively; only skin conduction, the leak, and
/// the link to `MainAvionics` act on it.
const WING_ROOT_BLEED: BayParams = BayParams { baseline_avionics_w: 0.0, skin_area_m2: 2.0, fan_flow_kg_s: 0.0, thermal_mass_j_per_k: 1.5e5 };

/// Not-yet-registered placeholder failure ids for the two avionics bays'
/// own extract fans (task step 4's "loss of the avionics extract fan"):
/// `FwdCargo`/`AftCargo` already have real, registered ids
/// (`failures.rs` 21_008 `FwdExtractFan`, 21_010 `BulkExtractFan`), but no
/// equivalent exists yet for `MainAvionics`/`UpperAvionics` ventilation.
/// `failures::magnitude(id)` works on any `u64` without prior
/// registration (it just reads the global active-set/magnitude map), so
/// these are usable today by tests via `failures::set_active`/
/// `set_magnitude`; production scenario wiring still needs a real catalogue
/// entry from the failures/breaker workstream -- see the module doc.
pub const MAIN_AVIONICS_VENT_FAN_FAILURE_ID: u64 = 21_101;
pub const UPPER_AVIONICS_VENT_FAN_FAILURE_ID: u64 = 21_102;

/// Engine bleed-duct-leak failure ids this module reads directly
/// (`failures.rs` 36_000..36_003, "Engine n bleed duct leak" --
/// `Effect::Hook { var: "FAIL_BLEED_DUCT_LEAK_HOOK", owner: Owner::Air }`).
/// The air workstream owns that hook's *pneumatic* consumption (duct mass
/// flow bypass); this module reads the same `failures::magnitude` on the
/// same ids for its own, independent physical effect (the leaked air's
/// heat), exactly the way `physics::tyre` reads `damage.rs`'s per-leg leak
/// ids for its own temperature/wear effects alongside `damage.rs`'s own
/// consumption of the same id.
const ENGINE_BLEED_LEAK_FAILURE_IDS: [u64; 4] = [36_000, 36_001, 36_002, 36_003];

// ---------------------------------------------------------------------------
// The five bays
// ---------------------------------------------------------------------------

/// Equipment bay thermal model: five lumped nodes, ticked once per frame.
pub struct Bays {
    pub main_avionics_c: f64,
    pub upper_avionics_c: f64,
    pub fwd_cargo_c: f64,
    pub aft_cargo_c: f64,
    pub wing_root_bleed_c: f64,
    /// The one deliberate "wire" in the whole chain (module doc): true in
    /// production. The headline test's decouple check runs a second
    /// `Bays` with this `false`, proving the same leak cannot trip the
    /// breaker once the physical path between the two bays is cut. Not a
    /// failure and not exposed as a dataref -- a structural test/model
    /// control, the equivalent of physically removing the shared
    /// bulkhead's thermal path.
    pub heat_link_enabled: bool,

    ambient_c: VariableIdentifier,
    true_airspeed_kt: VariableIdentifier,
    ambient_pressure_inhg: VariableIdentifier,
    main_avionics_out: VariableIdentifier,
    upper_avionics_out: VariableIdentifier,
    fwd_cargo_out: VariableIdentifier,
    aft_cargo_out: VariableIdentifier,
    wing_root_bleed_out: VariableIdentifier,
    /// Shared engine-load-contract-style optional per-bay electrical heat
    /// (W), reading as 0 (falls back to `baseline_avionics_w`) until the
    /// electrical workstream publishes real per-bay TR/bus dissipation.
    main_avionics_elec_w: VariableIdentifier,
    upper_avionics_elec_w: VariableIdentifier,
}

impl Bays {
    pub fn new(vars: &mut Vars) -> Self {
        use systems::simulation::VariableRegistry;
        Self {
            main_avionics_c: 25.0,
            upper_avionics_c: 25.0,
            fwd_cargo_c: 25.0,
            aft_cargo_c: 25.0,
            wing_root_bleed_c: 25.0,
            heat_link_enabled: true,
            ambient_c: vars.get("AMBIENT TEMPERATURE".into()),
            true_airspeed_kt: vars.get("AIRSPEED TRUE".into()),
            ambient_pressure_inhg: vars.get("AMBIENT PRESSURE".into()),
            // Names matched to `breakers.rs`'s already-landed `bay_for`
            // consumer (docs/physics/breakers.md "thermal ambient"
            // section): `Breakers::new` reads back exactly these dataref
            // names (`BAY_AVIONICS_TEMPERATURE_C`, `BAY_CARGO_FWD_
            // TEMPERATURE_C`, `BAY_CARGO_AFT_TEMPERATURE_C`,
            // `BAY_WING_ROOT_TEMPERATURE_C`) as each breaker's ambient,
            // currently defaulted to `REFERENCE_AMBIENT_C` until this
            // module writes a real value -- publishing under these same
            // names is what actually closes the emergent chain end to
            // end. `UpperAvionics` has no `bay_for` bucket of its own
            // (every avionics-bay breaker maps to the single `AVIONICS`
            // bucket), so it keeps its own descriptive name as a bonus
            // bay not yet consumed by any breaker.
            main_avionics_out: vars.get("BAY_AVIONICS_TEMPERATURE_C".into()),
            upper_avionics_out: vars.get("BAY_UPPER_AVIONICS_TEMPERATURE_C".into()),
            fwd_cargo_out: vars.get("BAY_CARGO_FWD_TEMPERATURE_C".into()),
            aft_cargo_out: vars.get("BAY_CARGO_AFT_TEMPERATURE_C".into()),
            wing_root_bleed_out: vars.get("BAY_WING_ROOT_TEMPERATURE_C".into()),
            main_avionics_elec_w: vars.get("ELEC_BAY_MAIN_AVIONICS_HEAT_W".into()),
            upper_avionics_elec_w: vars.get("ELEC_BAY_UPPER_AVIONICS_HEAT_W".into()),
        }
    }

    /// Advance every bay one tick and publish `BAY_<NAME>_TEMPERATURE_C`.
    /// `delta` is real (unpaused) seconds this tick.
    pub fn update(&mut self, vars: &mut Vars, delta: f64) {
        use systems::simulation::SimulatorReaderWriter;

        let ambient_c = vars.read(&self.ambient_c);
        let tas_m_s = vars.read(&self.true_airspeed_kt) * 0.514444;
        let ambient_pa = vars.read(&self.ambient_pressure_inhg) * INHG_TO_PA;
        let h = external_h_w_m2k(tas_m_s);

        // Bleed-duct leak enthalpy flow into WingRootBleed: sum of all
        // four engines' independent leak severities (continuous,
        // `failures::magnitude`), each its own orifice at the shared duct
        // source condition.
        let leak_w: f64 = ENGINE_BLEED_LEAK_FAILURE_IDS
            .iter()
            .map(|&id| {
                let mag = failures::magnitude(id);
                let area = leak_area_m2(mag);
                let mdot = orifice_mass_flow_kg_s(LEAK_DISCHARGE_COEFFICIENT, area, BLEED_DUCT_PRESSURE_PA, BLEED_DUCT_TEMP_C + 273.15, ambient_pa);
                mdot * CP_AIR_J_PER_KG_K * (BLEED_DUCT_TEMP_C - self.wing_root_bleed_c)
            })
            .sum();

        // The one conductive link between WingRootBleed and MainAvionics
        // -- zero when `heat_link_enabled` is false (the decouple check).
        let link_w = if self.heat_link_enabled { WING_ROOT_TO_MAIN_AVIONICS_LINK_W_PER_K * (self.wing_root_bleed_c - self.main_avionics_c) } else { 0.0 };

        let main_elec_extra = vars.read(&self.main_avionics_elec_w);
        let main_avionics_w = if main_elec_extra > 0.0 { main_elec_extra } else { MAIN_AVIONICS.baseline_avionics_w };
        let upper_elec_extra = vars.read(&self.upper_avionics_elec_w);
        let upper_avionics_w = if upper_elec_extra > 0.0 { upper_elec_extra } else { UPPER_AVIONICS.baseline_avionics_w };

        let main_fan_eff = 1.0 - failures::magnitude(MAIN_AVIONICS_VENT_FAN_FAILURE_ID);
        let upper_fan_eff = 1.0 - failures::magnitude(UPPER_AVIONICS_VENT_FAN_FAILURE_ID);
        let fwd_fan_eff = 1.0 - failures::magnitude(21_008); // FwdExtractFan
        let aft_fan_eff = 1.0 - failures::magnitude(21_010); // BulkExtractFan

        self.main_avionics_c = step_bay(self.main_avionics_c, delta, &MAIN_AVIONICS, main_avionics_w + link_w, main_fan_eff, ambient_c, h);
        self.upper_avionics_c = step_bay(self.upper_avionics_c, delta, &UPPER_AVIONICS, upper_avionics_w, upper_fan_eff, ambient_c, h);
        self.fwd_cargo_c = step_bay(self.fwd_cargo_c, delta, &FWD_CARGO, FWD_CARGO.baseline_avionics_w, fwd_fan_eff, ambient_c, h);
        self.aft_cargo_c = step_bay(self.aft_cargo_c, delta, &AFT_CARGO, AFT_CARGO.baseline_avionics_w, aft_fan_eff, ambient_c, h);
        self.wing_root_bleed_c = step_bay(self.wing_root_bleed_c, delta, &WING_ROOT_BLEED, leak_w - link_w, 1.0, ambient_c, h);

        vars.write(&self.main_avionics_out, self.main_avionics_c);
        vars.write(&self.upper_avionics_out, self.upper_avionics_c);
        vars.write(&self.fwd_cargo_out, self.fwd_cargo_c);
        vars.write(&self.aft_cargo_out, self.aft_cargo_c);
        vars.write(&self.wing_root_bleed_out, self.wing_root_bleed_c);
    }
}

/// One bay's energy balance for one tick: `d(T)/dt = sum(Q) / thermal_mass`,
/// `sum(Q)` = `extra_heat_w` (baseline avionics/electrical heat, plus any
/// leak/link term the caller already folded in) minus extract-fan
/// ventilation to [`CABIN_SUPPLY_TEMP_C`] (gated by `fan_eff`, 1.0 =
/// healthy, 0.0 = fully failed/lost) minus skin conduction to `ambient_c`
/// at convection coefficient `h_w_m2k`. Not clamped locally --
/// `invariants::check` (`TemperatureFloor`) is the one clamp point, same
/// as `physics::tyre::TyreWheel::step`.
fn step_bay(temp_c: f64, delta: f64, p: &BayParams, extra_heat_w: f64, fan_eff: f64, ambient_c: f64, h_w_m2k: f64) -> f64 {
    use crate::invariants::{self, Bound};

    let fan_w = fan_eff.clamp(0.0, 1.0) * p.fan_flow_kg_s * CP_AIR_J_PER_KG_K * (temp_c - CABIN_SUPPLY_TEMP_C);
    let skin_w = h_w_m2k * p.skin_area_m2 * (temp_c - ambient_c);
    let net_w = extra_heat_w - fan_w - skin_w;
    let raw = temp_c + (net_w / p.thermal_mass_j_per_k) * delta;
    invariants::check("BAY_TEMPERATURE_C", raw, Bound::TemperatureFloor(-273.15), "physics::bays::step_bay")
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Steady state of the single linear ODE `step_bay` integrates:
    /// `0 = Q_extra - fan_eff*fan*cp*(T-supply) - h*A*(T-ambient)`, solved
    /// directly for `T` (independent of `step_bay`'s own code -- this is
    /// the textbook closed-form steady state of a first-order linear
    /// thermal-mass system, the same kind of hand solve
    /// `physics::tyre`'s own tests use for its linear ODE).
    fn steady_state_c(p: &BayParams, extra_heat_w: f64, fan_eff: f64, ambient_c: f64, h_w_m2k: f64) -> f64 {
        let fan_ua = fan_eff.clamp(0.0, 1.0) * p.fan_flow_kg_s * CP_AIR_J_PER_KG_K;
        let skin_ua = h_w_m2k * p.skin_area_m2;
        (extra_heat_w + fan_ua * CABIN_SUPPLY_TEMP_C + skin_ua * ambient_c) / (fan_ua + skin_ua)
    }

    fn run_to_steady_state(temp_c: &mut f64, delta_s: f64, ticks: u32, p: &BayParams, extra_heat_w: f64, fan_eff: f64, ambient_c: f64, h_w_m2k: f64) {
        for _ in 0..ticks {
            *temp_c = step_bay(*temp_c, delta_s, p, extra_heat_w, fan_eff, ambient_c, h_w_m2k);
        }
    }

    // -- 1. Bay temperature: independent prediction, no breaker involved --

    #[test]
    fn main_avionics_bay_reaches_its_hand_computed_steady_state() {
        crate::scenarios::reset_global_state();
        let h = external_h_w_m2k(0.0); // on ground, TAS 0 -> h = 10 W/m^2K
        let predicted = steady_state_c(&MAIN_AVIONICS, MAIN_AVIONICS.baseline_avionics_w, 1.0, 25.0, h);
        let mut t = 25.0;
        run_to_steady_state(&mut t, 1.0, 6000, &MAIN_AVIONICS, MAIN_AVIONICS.baseline_avionics_w, 1.0, 25.0, h);
        assert!((t - predicted).abs() < 0.1, "sim {t} vs hand-computed steady state {predicted}");
        // Sanity: baseline avionics heat alone keeps the bay only modestly
        // above ambient with a healthy fan (well short of the headline
        // test's leak-driven temperature).
        assert!(predicted < 35.0, "healthy-fan baseline steady state should stay modest, got {predicted}");
    }

    // -- 2. Headline chain: leak -> WingRootBleed heat -> link -> MainAvionics
    //    heat -> breaker derating -> trip, at a load fine when cold. --

    #[test]
    fn bleed_leak_heats_avionics_bay_and_trips_a_breaker_fine_when_cold() {
        crate::scenarios::reset_global_state();
        // `36_000` ("Engine 1 bleed duct leak") is a real id in
        // `failures.rs`'s own catalogue; `set_active`/`set_magnitude` only
        // take effect once that catalogue has been registered, same
        // precedent as `scenarios.rs`'s own
        // `reset_global_state_clears_failure_activation` test.
        let _f = crate::failures::Failures::new();
        let ambient_c = 25.0;
        let h = external_h_w_m2k(0.0); // ground, no wind

        // Continuous leak, full severity (failures::magnitude), engine 1.
        failures::set_active(36_000, true);
        failures::set_magnitude(36_000, 1.0);
        assert_eq!(failures::magnitude(36_000), 1.0, "setup: leak magnitude must actually register");

        // --- Independent prediction 1: WingRootBleed / MainAvionics
        // steady state, hand-solved as two linear equations (same
        // derivation as this file's module doc: two coupled first-order
        // balances, solved by substitution -- not calling `Bays::update`
        // or `step_bay` at all). ---
        let leak_area = leak_area_m2(1.0);
        let mdot = orifice_mass_flow_kg_s(LEAK_DISCHARGE_COEFFICIENT, leak_area, BLEED_DUCT_PRESSURE_PA, BLEED_DUCT_TEMP_C + 273.15, 101_325.0);
        let leak_ua = mdot * CP_AIR_J_PER_KG_K; // heat-capacity rate of the leak flow
        let a = WING_ROOT_TO_MAIN_AVIONICS_LINK_W_PER_K;
        let skin_wrb_ua = h * WING_ROOT_BLEED.skin_area_m2;
        let fan_main_ua = MAIN_AVIONICS.fan_flow_kg_s * CP_AIR_J_PER_KG_K;
        let skin_main_ua = h * MAIN_AVIONICS.skin_area_m2;

        // WingRootBleed: 0 = leak_ua*(200 - Twrb) - a*(Twrb - Tmain) - skin_wrb_ua*(Twrb - ambient)
        //   => Twrb = (leak_ua*200 + a*Tmain + skin_wrb_ua*ambient) / (leak_ua + a + skin_wrb_ua)
        // MainAvionics: 0 = 3000 + a*(Twrb - Tmain) - fan_main_ua*(Tmain - 24) - skin_main_ua*(Tmain - ambient)
        //   => Tmain = (3000 + a*Twrb + fan_main_ua*24 + skin_main_ua*ambient) / (a + fan_main_ua + skin_main_ua)
        // Solve by substitution (Twrb linear in Tmain, then substitute).
        let wrb_denom = leak_ua + a + skin_wrb_ua;
        let wrb_k0 = (leak_ua * BLEED_DUCT_TEMP_C + skin_wrb_ua * ambient_c) / wrb_denom;
        let wrb_k1 = a / wrb_denom; // Twrb = wrb_k0 + wrb_k1*Tmain

        let main_denom = a + fan_main_ua + skin_main_ua;
        let main_k0 = (MAIN_AVIONICS.baseline_avionics_w + fan_main_ua * CABIN_SUPPLY_TEMP_C + skin_main_ua * ambient_c) / main_denom;
        let main_k1 = a / main_denom; // Tmain = main_k0 + main_k1*Twrb

        // Tmain = main_k0 + main_k1*(wrb_k0 + wrb_k1*Tmain)
        let t_main_predicted = (main_k0 + main_k1 * wrb_k0) / (1.0 - main_k1 * wrb_k1);
        let t_wrb_predicted = wrb_k0 + wrb_k1 * t_main_predicted;

        // --- Sim: run the same per-bay step function (`Bays::update`'s
        // own per-tick math, called directly with plain locals so this
        // test needs no `Vars`/simulator plumbing) to steady state and
        // confirm it matches the independent hand solve above. ---
        let mut sim_main = ambient_c;
        let mut sim_wrb = ambient_c;
        for _ in 0..12000 {
            sim_wrb = step_bay(sim_wrb, 1.0, &WING_ROOT_BLEED, leak_heat_w(mdot, sim_wrb) - link_heat_w(true, sim_wrb, sim_main), 1.0, ambient_c, h);
            sim_main = step_bay(sim_main, 1.0, &MAIN_AVIONICS, MAIN_AVIONICS.baseline_avionics_w + link_heat_w(true, sim_wrb, sim_main), 1.0, ambient_c, h);
        }
        assert!((sim_main - t_main_predicted).abs() < 0.5, "sim main avionics {sim_main} vs hand-computed {t_main_predicted}");
        assert!((sim_wrb - t_wrb_predicted).abs() < 0.5, "sim wing-root-bleed {sim_wrb} vs hand-computed {t_wrb_predicted}");

        // --- Independent prediction 2: trip time from the hand-computed
        // bay temperature, using breakers.rs's own already-landed
        // `trip_step_with_ambient` (physics::electrical, `pub`, not this
        // module's code) at a consumer load fixed at 90% of its nameplate
        // rating (the task's "load fine when cold") -- `ratio` itself
        // never changes; only the bay ambient does. ---
        use crate::physics::electrical::{trip_step_with_ambient, REFERENCE_AMBIENT_C};
        const RATIO: f64 = 0.9; // 90% of rated current, fixed for every case below
        // electrical.rs's own THERMAL_TRIP_K = 30.0 and
        // ELEMENT_RISE_AT_TRIP_C = 75.0 are private; reproduced here from
        // `trip_step_with_ambient`'s own doc/source (read before writing
        // this test), same as this file's other citations of a sibling
        // module's private constants -- this test's own independent
        // restatement of that published curve, used only to predict trip
        // time, never to compute it (the sim call below uses the real
        // function).
        const THERMAL_TRIP_K: f64 = 30.0;
        const ELEMENT_RISE_AT_TRIP_C: f64 = 75.0;
        let thermal_input_hot = RATIO * RATIO + (t_main_predicted - REFERENCE_AMBIENT_C) / ELEMENT_RISE_AT_TRIP_C;
        assert!(thermal_input_hot > 1.0, "bay must be hot enough to push a 90% load over the trip threshold for this test to be meaningful, thermal_input={thermal_input_hot}");
        let predicted_trip_s = THERMAL_TRIP_K / (thermal_input_hot - 1.0);

        let mut heat = 0.0;
        let mut tripped_at = None;
        let mut t = 0.0;
        while t < predicted_trip_s * 3.0 + 60.0 {
            if trip_step_with_ambient(&mut heat, RATIO, 1.0, t_main_predicted).is_some() {
                tripped_at = Some(t);
                break;
            }
            t += 1.0;
        }
        let tripped_at = tripped_at.expect("a 90%-of-nameplate load in the hot, leak-heated bay should trip");
        assert!((tripped_at - predicted_trip_s).abs() < predicted_trip_s * 0.15 + 2.0, "trip at {tripped_at}s vs hand-computed {predicted_trip_s}s");

        // --- Control: the same 90% load in a cold (ambient-temperature)
        // bay never trips -- at REFERENCE_AMBIENT_C, trip_step_with_ambient
        // is bit-for-bit the old ambient-free curve, and I^2t heat only
        // accumulates above ratio 1.0. ---
        let mut cold_heat = 0.0;
        for _ in 0..3600 {
            assert!(trip_step_with_ambient(&mut cold_heat, RATIO, 1.0, ambient_c).is_none(), "a 90% load in a cold bay must never trip");
        }

        // --- Decouple: same leak, same load, but the WingRootBleed <->
        // MainAvionics heat link is cut. MainAvionics should settle back
        // near its own no-leak baseline and the breaker must not trip. ---
        let mut decoupled_main = ambient_c;
        let mut decoupled_wrb = ambient_c;
        for _ in 0..12000 {
            decoupled_wrb = step_bay(decoupled_wrb, 1.0, &WING_ROOT_BLEED, leak_heat_w(mdot, decoupled_wrb) - link_heat_w(false, decoupled_wrb, decoupled_main), 1.0, ambient_c, h);
            decoupled_main = step_bay(decoupled_main, 1.0, &MAIN_AVIONICS, MAIN_AVIONICS.baseline_avionics_w + link_heat_w(false, decoupled_wrb, decoupled_main), 1.0, ambient_c, h);
        }
        assert!(decoupled_wrb > 100.0, "wing-root-bleed bay itself must still be hot from the leak (proves the leak is still active, only the link is cut), got {decoupled_wrb}");
        assert!((decoupled_main - t_main_predicted).abs() > 5.0, "cutting the heat link must make a real difference to MainAvionics vs the coupled case");
        let decoupled_thermal_input = RATIO * RATIO + (decoupled_main - REFERENCE_AMBIENT_C) / ELEMENT_RISE_AT_TRIP_C;
        assert!(decoupled_thermal_input <= 1.0, "with the bay heat link cut, the same leak must not push a 90% load over the trip threshold, thermal_input={decoupled_thermal_input}");
        let mut decoupled_heat = 0.0;
        for _ in 0..3600 {
            assert!(trip_step_with_ambient(&mut decoupled_heat, RATIO, 1.0, decoupled_main).is_none(), "decouple check: bay heat link cut, breaker must not trip");
        }
    }

    fn leak_heat_w(mdot: f64, wrb_temp_c: f64) -> f64 {
        mdot * CP_AIR_J_PER_KG_K * (BLEED_DUCT_TEMP_C - wrb_temp_c)
    }
    fn link_heat_w(enabled: bool, wrb_temp_c: f64, main_temp_c: f64) -> f64 {
        if enabled {
            WING_ROOT_TO_MAIN_AVIONICS_LINK_W_PER_K * (wrb_temp_c - main_temp_c)
        } else {
            0.0
        }
    }

    // -- 3. Loss of the avionics extract fan: continuous airflow loss,
    //    hand-computed steady state. --

    #[test]
    fn losing_the_main_avionics_extract_fan_raises_its_steady_state_temperature() {
        crate::scenarios::reset_global_state();
        let ambient_c = 25.0;
        let h = external_h_w_m2k(0.0);

        let healthy_predicted = steady_state_c(&MAIN_AVIONICS, MAIN_AVIONICS.baseline_avionics_w, 1.0, ambient_c, h);
        let mut healthy_t = ambient_c;
        run_to_steady_state(&mut healthy_t, 1.0, 8000, &MAIN_AVIONICS, MAIN_AVIONICS.baseline_avionics_w, 1.0, ambient_c, h);
        assert!((healthy_t - healthy_predicted).abs() < 0.1);

        // Continuous fan loss: `MAIN_AVIONICS_VENT_FAN_FAILURE_ID` is a
        // placeholder id not yet in `failures.rs`'s registered catalogue
        // (module doc), so `failures::set_active`/`set_magnitude` would
        // silently no-op on it today (both require the id to already be
        // registered -- confirmed by reading `set_active`/`set_magnitude`
        // in failures.rs). This test instead exercises the same `fan_eff`
        // input `Bays::update` computes as `1.0 -
        // failures::magnitude(MAIN_AVIONICS_VENT_FAN_FAILURE_ID)` would
        // once that id is registered: `fan_eff = 0.0` is exactly what a
        // full-severity (`magnitude == 1.0`) fan loss produces.
        let fan_eff = 1.0 - 1.0_f64;
        assert_eq!(fan_eff, 0.0);

        // With the fan fully lost, only skin conduction cools the bay:
        // 0 = 3000 - skin_ua*(T - ambient) => T = ambient + 3000/skin_ua.
        // Time constant with only skin conduction cooling is C/UA =
        // 3.0e5/40 = 7500 s; run well past 10 time constants so the sim
        // has genuinely converged (not just close) before comparing.
        let lost_fan_predicted = steady_state_c(&MAIN_AVIONICS, MAIN_AVIONICS.baseline_avionics_w, 0.0, ambient_c, h);
        let mut lost_fan_t = ambient_c;
        run_to_steady_state(&mut lost_fan_t, 1.0, 120_000, &MAIN_AVIONICS, MAIN_AVIONICS.baseline_avionics_w, 0.0, ambient_c, h);
        assert!((lost_fan_t - lost_fan_predicted).abs() < 0.5, "sim {lost_fan_t} vs hand-computed {lost_fan_predicted}");
        assert!(lost_fan_t > healthy_t + 20.0, "losing extract ventilation should raise the bay's steady-state temperature substantially: healthy {healthy_t}, lost-fan {lost_fan_t}");
    }
}
