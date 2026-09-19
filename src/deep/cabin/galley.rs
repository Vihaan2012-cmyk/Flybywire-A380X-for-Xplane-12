//! Galleys: ovens, chillers and water boilers as electrical loads with real
//! thermal state, plus a galley bus feed fault, chiller failure and oven
//! overheat/smoke.
//!
//! No FlyByWire source exists to port beyond the electrical shed *flag*
//! (`a380_systems/src/electrical/galley.rs`'s `MainGalley`/`SecondaryGalley`,
//! a boolean with no thermal model at all); this module gives that flag
//! something physical to shed. Native addition, one galley per cabin zone
//! (`Zone`), each with one oven, one chiller and one water boiler — a
//! simplification (a real A380 galley has several of each) that still lets
//! every fault the backlog asks for act on a real heat balance.
//!
//! **Sourcing:**
//! - Aircraft galley convection ovens are commonly documented (cabin-crew
//!   training material, catering-equipment trade literature) as electric,
//!   thermostatically controlled, with typical cook/reheat temperatures in
//!   the 350-450 F (177-232 C) class; 200 C is used here as a `GENERIC`
//!   representative setpoint.
//! - Galley chillers keep stored catering trolleys cold (commonly cited
//!   target on the order of 4-8 C, standard commercial refrigeration
//!   practice for food safety); 5 C is `GENERIC` here.
//! - Galley water boilers heat water for hot beverages to just under
//!   boiling; 95 C is `GENERIC` here (below standard atmospheric boiling
//!   point, matching how catering water boilers are commonly described as
//!   not full rolling boil).
//! - All ratings (oven 2200 W, chiller compressor 300 W / cooling capacity
//!   250 W, boiler 1500 W) and thermal masses/loss coefficients are
//!   `GENERIC`: no public A380 AMM galley-equipment figures exist. Sized
//!   only for the right qualitative order (an oven reaches cooking
//!   temperature in minutes, a chiller pulls a warm compartment down over
//!   tens of minutes), following this crate's existing convention for
//!   unsourced equipment figures (e.g. `physics::engine::oil`'s pump/
//!   cooler sizes).

use super::Zone;

/// GENERIC oven element rating, W, and cavity thermostat setpoint, C
/// (module doc).
const OVEN_RATED_W: f64 = 2200.0;
const OVEN_SETPOINT_C: f64 = 200.0;
const OVEN_BAND_C: f64 = 10.0;
/// GENERIC oven cavity thermal mass (J/K) and loss to the surrounding
/// galley air (W/K).
const OVEN_CAPACITY_J_K: f64 = 4000.0;
const OVEN_LOSS_W_K: f64 = 8.0;
/// A thermostat stuck on (rather than cycling) lets the cavity run past
/// its setpoint; smoke is reported above this. GENERIC.
const OVEN_SMOKE_TEMP_C: f64 = 260.0;

/// GENERIC chiller compartment target temperature, C, compressor electrical
/// rating and net cooling capacity, W (module doc).
const CHILLER_SETPOINT_C: f64 = 5.0;
const CHILLER_COMPRESSOR_W: f64 = 300.0;
const CHILLER_CAPACITY_REMOVED_W: f64 = 250.0;
/// GENERIC compartment thermal mass (loaded trolleys) and heat leaking in
/// from the surrounding cabin air.
const CHILLER_MASS_J_K: f64 = 30_000.0;
const CHILLER_LEAK_W_K: f64 = 4.0;
const CABIN_AMBIENT_C: f64 = 24.0;

/// GENERIC water boiler element rating, W, reservoir mass, kg, and
/// thermostat setpoint, C (module doc).
const BOILER_RATED_W: f64 = 1500.0;
const BOILER_WATER_KG: f64 = 2.0;
const BOILER_SETPOINT_C: f64 = 95.0;
const BOILER_BAND_C: f64 = 5.0;
const BOILER_LOSS_W_K: f64 = 3.0;
const WATER_SPECIFIC_HEAT_J_KGK: f64 = 4186.0; // standard, matches water.rs

/// Faults this system carries, each 0.0 (healthy) .. 1.0 (fully failed),
/// one set per galley zone.
#[derive(Clone, Copy, Debug, Default)]
pub struct GalleyFaults {
    /// The galley's own bus feed (a wiring/contactor fault, distinct from
    /// the aircraft-wide shed input below): at 1.0 the whole galley is dead.
    pub bus_fault: [f64; Zone::COUNT],
    /// The oven thermostat stuck closed (element never cycles off): drives
    /// the cavity toward the smoke threshold instead of holding setpoint.
    pub oven_overheat: [f64; Zone::COUNT],
    /// The chiller compressor.
    pub chiller_fault: [f64; Zone::COUNT],
    pub boiler_fault: [f64; Zone::COUNT],
}

#[derive(Clone, Copy, Debug)]
pub struct GalleyInputs {
    /// The aircraft-wide galley bus feed (FlyByWire's own shed condition:
    /// non-essential bus unpowered, single-generator ops, or the overhead
    /// COMMERCIAL/GALLEY pushbutton off) — ANDed with each galley's own
    /// `bus_fault` above.
    pub commercial_power_available: bool,
    pub oven_commanded: [bool; Zone::COUNT],
    pub chiller_commanded: [bool; Zone::COUNT],
    pub boiler_commanded: [bool; Zone::COUNT],
}

impl Default for GalleyInputs {
    fn default() -> Self {
        Self {
            commercial_power_available: true,
            oven_commanded: [false; Zone::COUNT],
            chiller_commanded: [true; Zone::COUNT],
            boiler_commanded: [true; Zone::COUNT],
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct GalleyOutputs {
    pub oven_temp_c: [f64; Zone::COUNT],
    pub oven_power_w: [f64; Zone::COUNT],
    pub oven_smoke: [bool; Zone::COUNT],
    pub chiller_temp_c: [f64; Zone::COUNT],
    pub chiller_power_w: [f64; Zone::COUNT],
    pub chiller_failed_warm: [bool; Zone::COUNT],
    pub boiler_temp_c: [f64; Zone::COUNT],
    pub boiler_power_w: [f64; Zone::COUNT],
    pub total_power_w: f64,
}

#[derive(Clone, Copy, Debug)]
struct GalleyZoneState {
    oven_c: f64,
    chiller_c: f64,
    boiler_c: f64,
}

pub struct GalleySystem {
    zones: [GalleyZoneState; Zone::COUNT],
}

impl GalleySystem {
    pub fn new() -> Self {
        Self { zones: [GalleyZoneState { oven_c: CABIN_AMBIENT_C, chiller_c: CABIN_AMBIENT_C, boiler_c: CABIN_AMBIENT_C }; Zone::COUNT] }
    }

    pub fn step(&mut self, inputs: &GalleyInputs, faults: &GalleyFaults, dt: f64) -> GalleyOutputs {
        let dt = dt.max(0.0);
        let mut out = GalleyOutputs::default();

        for i in 0..Zone::COUNT {
            let state = &mut self.zones[i];
            let galley_live = inputs.commercial_power_available && faults.bus_fault[i] < 1.0;

            // Oven: thermostat holds setpoint by cycling; a stuck-closed
            // thermostat (oven_overheat fault) instead keeps the element on
            // past setpoint, driving the cavity toward the smoke threshold.
            let oven_stuck = faults.oven_overheat[i] > 0.0;
            let oven_on = galley_live && inputs.oven_commanded[i] && (oven_stuck || state.oven_c < OVEN_SETPOINT_C + OVEN_BAND_C);
            let oven_target_setpoint = if oven_stuck { OVEN_SMOKE_TEMP_C + 100.0 } else { OVEN_SETPOINT_C };
            let oven_power_w = if oven_on { OVEN_RATED_W } else { 0.0 };
            let oven_target_c = CABIN_AMBIENT_C + oven_power_w / OVEN_LOSS_W_K;
            let oven_tau_s = (OVEN_CAPACITY_J_K / OVEN_LOSS_W_K).max(1e-6);
            // A healthy thermostat cannot overshoot past its own setpoint
            // band because `oven_on` above stops commanding heat once the
            // cavity reaches it; only the stuck case's higher implicit
            // target keeps driving the cavity further.
            let capped_target_c = if oven_stuck { oven_target_c.min(oven_target_setpoint + 200.0) } else { oven_target_c.min(OVEN_SETPOINT_C + OVEN_BAND_C) };
            state.oven_c += (capped_target_c - state.oven_c) * (1.0 - (-dt / oven_tau_s).exp());
            out.oven_temp_c[i] = state.oven_c;
            out.oven_power_w[i] = oven_power_w;
            out.oven_smoke[i] = state.oven_c >= OVEN_SMOKE_TEMP_C;

            // Chiller: a heat-balance pulldown against cabin-ambient leak-in,
            // with fixed cooling capacity while the compressor runs.
            let chiller_health = 1.0 - faults.chiller_fault[i].clamp(0.0, 1.0);
            let chiller_on = galley_live && inputs.chiller_commanded[i] && chiller_health > 0.0 && state.chiller_c > CHILLER_SETPOINT_C - 1.0;
            let cooling_w = if chiller_on { CHILLER_CAPACITY_REMOVED_W * chiller_health } else { 0.0 };
            let leak_in_w = CHILLER_LEAK_W_K * (CABIN_AMBIENT_C - state.chiller_c);
            // Net heat flow into the compartment: leak in minus active
            // cooling; convert to a temperature-rate via the compartment's
            // thermal mass (an explicit energy balance step, not a lag
            // toward a precomputed target, since the compressor's on/off
            // condition itself depends on the running temperature).
            let net_w = leak_in_w - cooling_w;
            state.chiller_c += net_w * dt / CHILLER_MASS_J_K;
            state.chiller_c = state.chiller_c.min(CABIN_AMBIENT_C).max(-40.0);
            out.chiller_temp_c[i] = state.chiller_c;
            out.chiller_power_w[i] = if chiller_on { CHILLER_COMPRESSOR_W * chiller_health } else { 0.0 };
            out.chiller_failed_warm[i] = state.chiller_c > CHILLER_SETPOINT_C + 5.0;

            // Boiler: thermostatic first-order lag, same treatment as
            // `water.rs`'s point-of-use heaters.
            let boiler_health = 1.0 - faults.boiler_fault[i].clamp(0.0, 1.0);
            let boiler_on = galley_live && inputs.boiler_commanded[i] && state.boiler_c < BOILER_SETPOINT_C + BOILER_BAND_C && boiler_health > 0.0;
            let boiler_power_w = if boiler_on { BOILER_RATED_W * boiler_health } else { 0.0 };
            let boiler_capacity_j_k = BOILER_WATER_KG * WATER_SPECIFIC_HEAT_J_KGK;
            let boiler_target_c = CABIN_AMBIENT_C + boiler_power_w / BOILER_LOSS_W_K;
            let boiler_tau_s = (boiler_capacity_j_k / BOILER_LOSS_W_K).max(1e-6);
            state.boiler_c += (boiler_target_c - state.boiler_c) * (1.0 - (-dt / boiler_tau_s).exp());
            out.boiler_temp_c[i] = state.boiler_c;
            out.boiler_power_w[i] = boiler_power_w;
        }

        out.total_power_w = (0..Zone::COUNT).map(|i| out.oven_power_w[i] + out.chiller_power_w[i] + out.boiler_power_w[i]).sum();
        out
    }
}

impl Default for GalleySystem {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn commanded_all_on() -> GalleyInputs {
        GalleyInputs { oven_commanded: [true; Zone::COUNT], ..Default::default() }
    }

    #[test]
    fn a_healthy_oven_settles_at_its_thermostat_setpoint_not_higher() {
        let mut g = GalleySystem::new();
        let inputs = commanded_all_on();
        let mut out = GalleyOutputs::default();
        for _ in 0..7200 {
            out = g.step(&inputs, &GalleyFaults::default(), 1.0);
        }
        assert!((out.oven_temp_c[0] - OVEN_SETPOINT_C).abs() < OVEN_BAND_C + 1.0, "{}", out.oven_temp_c[0]);
        assert!(!out.oven_smoke[0]);
    }

    #[test]
    fn a_stuck_thermostat_overheats_the_oven_to_smoke() {
        let mut g = GalleySystem::new();
        let inputs = commanded_all_on();
        let mut faults = GalleyFaults::default();
        faults.oven_overheat[0] = 1.0;
        let mut out = GalleyOutputs::default();
        for _ in 0..7200 {
            out = g.step(&inputs, &faults, 1.0);
        }
        assert!(out.oven_smoke[0], "temp={}", out.oven_temp_c[0]);
        assert!(!out.oven_smoke[1], "other zones unaffected");
    }

    #[test]
    fn a_healthy_chiller_pulls_the_compartment_down_towards_its_setpoint() {
        let mut g = GalleySystem::new();
        let inputs = GalleyInputs::default();
        let mut out = GalleyOutputs::default();
        for _ in 0..36000 {
            out = g.step(&inputs, &GalleyFaults::default(), 1.0);
        }
        assert!(out.chiller_temp_c[0] < CABIN_AMBIENT_C - 10.0, "{}", out.chiller_temp_c[0]);
        assert!(!out.chiller_failed_warm[0]);
    }

    #[test]
    fn a_failed_chiller_drifts_back_to_cabin_ambient() {
        let mut healthy = GalleySystem::new();
        let mut failed = GalleySystem::new();
        let inputs = GalleyInputs::default();
        let mut faults = GalleyFaults::default();
        faults.chiller_fault[0] = 1.0;
        for _ in 0..36000 {
            healthy.step(&inputs, &GalleyFaults::default(), 1.0);
            failed.step(&inputs, &faults, 1.0);
        }
        assert!(failed.zones[0].chiller_c > healthy.zones[0].chiller_c, "a failed chiller should run warmer than a healthy one");
        assert!(failed.zones[0].chiller_c > CHILLER_SETPOINT_C + 5.0);
    }

    #[test]
    fn a_galley_bus_fault_kills_all_three_appliances_in_that_galley_only() {
        let mut g = GalleySystem::new();
        let inputs = commanded_all_on();
        let mut faults = GalleyFaults::default();
        faults.bus_fault[0] = 1.0;
        let out = g.step(&inputs, &faults, 1.0);
        assert_eq!(out.oven_power_w[0], 0.0);
        assert_eq!(out.chiller_power_w[0], 0.0);
        assert_eq!(out.boiler_power_w[0], 0.0);
        assert!(out.oven_power_w[1] > 0.0 || out.boiler_power_w[1] > 0.0, "other galleys unaffected");
    }

    #[test]
    fn shedding_the_commercial_bus_kills_every_galley() {
        let mut g = GalleySystem::new();
        let inputs = GalleyInputs { commercial_power_available: false, ..commanded_all_on() };
        let out = g.step(&inputs, &GalleyFaults::default(), 1.0);
        assert_eq!(out.total_power_w, 0.0);
    }

    #[test]
    fn a_boiler_reaches_its_thermostat_band_and_holds_there() {
        let mut g = GalleySystem::new();
        let inputs = GalleyInputs::default();
        let mut out = GalleyOutputs::default();
        for _ in 0..3600 {
            out = g.step(&inputs, &GalleyFaults::default(), 1.0);
        }
        assert!((out.boiler_temp_c[0] - BOILER_SETPOINT_C).abs() < BOILER_BAND_C + 1.0, "{}", out.boiler_temp_c[0]);
    }

    #[test]
    fn no_nan_at_rest_or_dt_zero() {
        let mut g = GalleySystem::new();
        let out = g.step(&GalleyInputs::default(), &GalleyFaults::default(), 0.0);
        assert!(out.oven_temp_c.iter().all(|t| !t.is_nan()));
        assert!(out.chiller_temp_c.iter().all(|t| !t.is_nan()));
        assert!(out.boiler_temp_c.iter().all(|t| !t.is_nan()));
    }
}
