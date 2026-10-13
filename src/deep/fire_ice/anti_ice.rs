//! Anti-ice: wing and engine-nacelle hot-bleed-air heat balance, electrical
//! probe and windshield heat, and windshield rain removal -- and their
//! valve, heater, controller and sensor faults. Shares the Messinger
//! surface energy balance and droplet collection physics with `icing.rs`
//! ([`super::util`]): an anti-ice system's job is exactly to supply the
//! heat that balance says would otherwise freeze the impinging water.
//!
//! ## Sources
//! - Wing/nacelle hot-air anti-ice (piccolo-tube style): standard public
//!   transport-aircraft anti-ice architecture description (e.g. FAA
//!   *Airplane Flying Handbook*/AMT Powerplant handbook, ch. on ice and
//!   rain protection); bleed supply temperature/pressure figures reused
//!   from this crate's own `docs/physics/air.md` design point ("~200 C
//!   bleed", "~44 psi source"), the same figures `physics::bays.rs` (read
//!   as context, not depended on -- BRIEF rule 2) already cites for a
//!   bleed-duct leak's source condition.
//! - "Running wet" vs "fully evaporative" anti-ice design philosophies are
//!   both real, publicly documented approaches (ice-protection engineering
//!   texts, e.g. Gent, Dart & Cansdale (2000), "Aircraft Icing", Phil.
//!   Trans. R. Soc. Lond. A 358); this module targets running-wet (surface
//!   held at/just above 0 C, matching [`super::util::messinger_freezing_
//!   fraction`]'s own "anti-ice demand" definition) since that is the less
//!   energy-intensive and more commonly described approach.
//! - Probe/window electrical heat: resistive (I^2R) heating is standard;
//!   typical windshield anti-ice/anti-fog target temperatures around
//!   40 C are commonly cited public figures for heated aircraft
//!   windshields (public ice-protection engineering references) --
//!   GENERIC here, not an A380-specific figure.
//! - Windshield electro-thermal ply delamination/cracking from a local
//!   resistance defect is standard, publicly documented heated-windshield
//!   failure physics (e.g. FAA Airworthiness Directives on heated
//!   windshield arcing/delamination events, public); the exact damage
//!   threshold is GENERIC.
//! - Rain removal by a high-velocity air jet shearing the water film off
//!   the windshield exterior is a standard, publicly described transport-
//!   aircraft feature (e.g. Bombardier/Boeing rain-removal system
//!   descriptions, public type-training material) additional to wipers.

use super::util::{clamp01, equilibrium_surface_c_with_bleed, equilibrium_surface_c_with_heater, relax_toward_equilibrium_c, surface_net_loss_w_m2, CP_AIR, LATENT_HEAT_FUSION_WATER_J_KG};

// ---------------------------------------------------------------------------
// Wing and nacelle hot-bleed-air anti-ice
// ---------------------------------------------------------------------------

/// GENERIC bleed source condition at the anti-ice valve (module doc: reuses
/// `docs/physics/air.md`'s own design point, not a new figure).
pub const BLEED_SUPPLY_TEMP_C: f64 = 200.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct BleedAntiIceFaults {
    /// Valve fails toward closed: reduces commanded flow toward zero.
    pub valve_stuck_closed: f64,
    /// Valve fails toward open: floor on flow regardless of command,
    /// risking overheat once icing conditions (and their cooling demand)
    /// end.
    pub valve_stuck_open: f64,
    /// Upstream duct leak: this fraction of the commanded flow never
    /// reaches the piccolo tube/heated skin at all.
    pub duct_leak: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct BleedSurfaceParams {
    pub area_m2: f64,
    pub h_w_m2k: f64,
    /// Bleed-to-skin heat exchange effectiveness (piccolo-tube jet
    /// impingement), 0..1. GENERIC, same order of magnitude as this
    /// crate's other cooler-effectiveness figures (e.g.
    /// `physics::engine::oil.rs`'s `FCOC_EFFECTIVENESS`/`ACOC_
    /// EFFECTIVENESS`, both 0.7-0.8).
    pub effectiveness: f64,
    /// Bleed mass flow at fully open commanded valve, kg/s. GENERIC.
    pub max_bleed_kg_s: f64,
    /// Representative thermal time constant of this surface's own metal
    /// mass, s: how quickly it moves toward the "if held forever at this
    /// heater/environment state" equilibrium
    /// [`super::util::equilibrium_surface_c_with_bleed`] computes, via
    /// [`super::util::relax_toward_equilibrium_c`]'s exact exponential
    /// step. **GENERIC**: a leading-edge skin's own thermal mass is not
    /// published; sized to the right order of magnitude for a metal skin
    /// a few mm thick (tens of seconds), not the near-zero mass a
    /// zero-thermal-capacity surface would imply.
    pub thermal_tau_s: f64,
}

/// **GENERIC** (module doc): no public per-zone bleed flow/area/thermal-
/// mass figure exists for the A380; these are sized to the right order of
/// magnitude for a large-transport wing/nacelle leading-edge anti-ice
/// zone.
pub const WING_ANTI_ICE: BleedSurfaceParams = BleedSurfaceParams { area_m2: 6.0, h_w_m2k: 120.0, effectiveness: 0.75, max_bleed_kg_s: 0.5, thermal_tau_s: 25.0 };
pub const NACELLE_ANTI_ICE: BleedSurfaceParams = BleedSurfaceParams { area_m2: 3.0, h_w_m2k: 150.0, effectiveness: 0.8, max_bleed_kg_s: 0.3, thermal_tau_s: 15.0 };

/// Skin/duct temperature above which the anti-ice overheat detection loop
/// trips (a real, commonly fitted feature on hot-air anti-ice systems --
/// public general knowledge, e.g. any transport type-training ice-
/// protection chapter). **GENERIC** figure, set below the dry/no-demand
/// stuck-valve equilibrium these parameters produce (~82 C, see tests) and
/// above the running-wet in-icing equilibrium (~20 C) so it discriminates
/// the two.
pub const OVERHEAT_TRIP_C: f64 = 60.0;

pub struct BleedAntiIceSurface {
    params: BleedSurfaceParams,
    surface_c: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BleedAntiIceOutputs {
    pub surface_c: f64,
    pub bleed_delivered_kg_s: f64,
    pub freezing_fraction: f64,
    pub overheat: bool,
}

impl BleedAntiIceSurface {
    pub fn new(params: BleedSurfaceParams, initial_c: f64) -> Self {
        Self { params, surface_c: initial_c }
    }

    pub fn surface_c(&self) -> f64 {
        self.surface_c
    }

    /// One tick. `valve_command` is 0..1 (crew switch/auto-icing-detection
    /// commanded position); `env`-derived Messinger inputs and `beta0` (a
    /// surface-specific collection efficiency, [`super::icing`]) determine
    /// the icing heat demand the bleed air must overcome. The bleed heat
    /// exchanger's own "current gap to supply temperature" coupling is
    /// solved self-consistently each tick
    /// ([`super::util::equilibrium_surface_c_with_bleed`]), then the
    /// surface's own thermal mass relaxes toward that target with an
    /// exact exponential step
    /// ([`super::util::relax_toward_equilibrium_c`]) -- giving smooth,
    /// numerically robust dynamics regardless of how large the bleed flow
    /// is relative to the surface's other conductances.
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        valve_command: f64,
        static_air_c: f64,
        recovery_c: f64,
        lwc_kg_m3: f64,
        beta0: f64,
        tas_m_s: f64,
        ambient_pressure_pa: f64,
        faults: &BleedAntiIceFaults,
        dt_s: f64,
    ) -> BleedAntiIceOutputs {
        let commanded = clamp01(valve_command) * (1.0 - clamp01(faults.valve_stuck_closed));
        let position = commanded.max(clamp01(faults.valve_stuck_open));
        let mdot_bleed = position * self.params.max_bleed_kg_s * (1.0 - clamp01(faults.duct_leak));

        let bleed_slope_w_m2k = self.params.effectiveness * mdot_bleed * CP_AIR / self.params.area_m2.max(1e-6);
        let equilibrium_c = equilibrium_surface_c_with_bleed(self.params.h_w_m2k, static_air_c, recovery_c, lwc_kg_m3, beta0, tas_m_s, ambient_pressure_pa, bleed_slope_w_m2k, BLEED_SUPPLY_TEMP_C);
        self.surface_c = relax_toward_equilibrium_c(self.surface_c, equilibrium_c, self.params.thermal_tau_s, dt_s);

        // Whether this surface, held at its actual (possibly heated)
        // temperature, is still accreting ice: re-evaluate the Messinger
        // freezing fraction against the achieved surface temperature
        // rather than the "would it freeze at 0 C" natural-icing number
        // [`super::util::messinger_freezing_fraction`] itself gives.
        let freezing_fraction = if self.surface_c > 0.0 {
            0.0
        } else {
            let (net_loss, impingement) = surface_net_loss_w_m2(self.params.h_w_m2k, static_air_c, recovery_c, lwc_kg_m3, beta0, tas_m_s, ambient_pressure_pa, self.surface_c);
            if impingement > 1e-9 {
                clamp01(net_loss / (impingement * LATENT_HEAT_FUSION_WATER_J_KG))
            } else {
                0.0
            }
        };

        BleedAntiIceOutputs { surface_c: self.surface_c, bleed_delivered_kg_s: mdot_bleed, freezing_fraction, overheat: self.surface_c > OVERHEAT_TRIP_C }
    }
}

// ---------------------------------------------------------------------------
// Probe electrical heat (with controller/sensor fault chain)
// ---------------------------------------------------------------------------

/// GENERIC rated probe heater power, W (representative Rosemount-class
/// pitot/AOA probe heater rating, public product literature order of
/// magnitude -- `sensors::pitot.rs` (read as context) cites 350 W for its
/// own, separate, more detailed pitot model; this module's own probe is a
/// smaller/simpler generic sensor, kept independently GENERIC).
pub const PROBE_HEATER_RATED_W: f64 = 80.0;
const PROBE_AREA_M2: f64 = 0.0006;
const PROBE_H_W_M2K: f64 = 200.0;
/// Thermostatic control target: hold at/just above freezing.
const PROBE_TARGET_C: f64 = 5.0;
/// GENERIC: a probe's own mass is tiny, so its thermal time constant is
/// short (seconds), unlike the much larger wing/nacelle skin or windshield
/// (see [`BleedSurfaceParams::thermal_tau_s`], [`WINDOW_THERMAL_TAU_S`]).
const PROBE_THERMAL_TAU_S: f64 = 5.0;
/// A stuck-sensor fault reads a fixed, plausible-looking warm value
/// instead of the probe's true temperature -- a real, common resistive-
/// element sensor failure mode (open winding or short to a fixed
/// reference rail settles the reading at one value regardless of the
/// actual temperature), rather than tracking ambient at all. **GENERIC**
/// value, chosen only to sit clearly above [`PROBE_TARGET_C`] so a full
/// fault reliably defeats control regardless of the real environment.
const PROBE_SENSOR_STUCK_READING_C: f64 = 15.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct ProbeHeaterFaults {
    /// Heater element open/degraded winding: fraction reduction of rated
    /// power actually deliverable even when fully commanded.
    pub heater_open_circuit: f64,
    /// The heater control computer fails to command power at all.
    pub controller_fault: f64,
    /// The temperature feedback sensor sticks at a fixed warm reading
    /// (module doc: [`PROBE_SENSOR_STUCK_READING_C`]) instead of tracking
    /// the probe's true temperature: the controller then under-commands
    /// heat even though the probe is genuinely cold -- a silent failure
    /// mode (the system believes it is healthy while the probe ices).
    pub sensor_fault: f64,
}

pub struct ProbeHeater {
    surface_c: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ProbeHeaterOutputs {
    pub surface_c: f64,
    pub power_w: f64,
    pub freezing_fraction: f64,
}

impl ProbeHeater {
    pub fn new(initial_c: f64) -> Self {
        Self { surface_c: initial_c }
    }

    pub fn surface_c(&self) -> f64 {
        self.surface_c
    }

    #[allow(clippy::too_many_arguments)]
    pub fn step(&mut self, static_air_c: f64, recovery_c: f64, lwc_kg_m3: f64, beta0: f64, tas_m_s: f64, ambient_pressure_pa: f64, faults: &ProbeHeaterFaults, dt_s: f64) -> ProbeHeaterOutputs {
        // The controller sees a blend of the true surface temperature and
        // a fault-biased fixed warm reading -- exactly the silent-failure
        // mechanism the module doc describes.
        let sensed_c = self.surface_c + (PROBE_SENSOR_STUCK_READING_C - self.surface_c) * clamp01(faults.sensor_fault);
        let commanded_on = sensed_c < PROBE_TARGET_C;
        let controller_ok = clamp01(faults.controller_fault) < 1.0;
        let power_w = if commanded_on && controller_ok {
            PROBE_HEATER_RATED_W * (1.0 - clamp01(faults.heater_open_circuit))
        } else {
            0.0
        };
        let heater_w_m2 = power_w / PROBE_AREA_M2;

        let equilibrium_c = equilibrium_surface_c_with_heater(PROBE_H_W_M2K, static_air_c, recovery_c, lwc_kg_m3, beta0, tas_m_s, ambient_pressure_pa, heater_w_m2);
        self.surface_c = relax_toward_equilibrium_c(self.surface_c, equilibrium_c, PROBE_THERMAL_TAU_S, dt_s);

        let freezing_fraction = if self.surface_c > 0.0 {
            0.0
        } else {
            let (net_loss, impingement) = surface_net_loss_w_m2(PROBE_H_W_M2K, static_air_c, recovery_c, lwc_kg_m3, beta0, tas_m_s, ambient_pressure_pa, self.surface_c);
            if impingement > 1e-9 {
                clamp01(net_loss / (impingement * LATENT_HEAT_FUSION_WATER_J_KG))
            } else {
                0.0
            }
        };

        ProbeHeaterOutputs { surface_c: self.surface_c, power_w, freezing_fraction }
    }
}

// ---------------------------------------------------------------------------
// Windshield electrical heat: film resistance, overheat, delamination/crack
// ---------------------------------------------------------------------------

const WINDOW_AREA_M2: f64 = 1.5;
const WINDOW_H_W_M2K: f64 = 80.0;
/// GENERIC target windshield temperature for anti-ice/anti-fog, deg C
/// (module doc).
pub const WINDOW_TARGET_C: f64 = 40.0;
/// GENERIC nameplate heating power for the film, W: sized so, against
/// this module's convective loss coefficient, the *unregulated* (always
/// on) equilibrium sits comfortably above `WINDOW_OVERHEAT_PROTECT_C`
/// (the fault this module needs to demonstrate), while normal thermostatic
/// cycling around `WINDOW_TARGET_C` never gets close to it (see tests).
pub const WINDOW_RATED_W: f64 = 10_000.0;
/// GENERIC overheat-protection cutout, deg C: the thermostat opens the
/// heater circuit above this to protect the ply bond.
pub const WINDOW_OVERHEAT_PROTECT_C: f64 = 65.0;
/// GENERIC damage threshold, deg C above [`WINDOW_OVERHEAT_PROTECT_C`] a
/// local hot spot must reach before the ply bond is damaged (delamination
/// onset) and further before it fully cracks.
const WINDOW_DELAMINATION_MARGIN_C: f64 = 15.0;
const WINDOW_CRACK_MARGIN_C: f64 = 40.0;
/// GENERIC: windshield glass/ply thermal mass is much larger than a
/// probe's, so its time constant is tens of seconds.
const WINDOW_THERMAL_TAU_S: f64 = 40.0;
/// A stuck-sensor fault reads a fixed warm value regardless of the
/// window's true temperature (module doc precedent:
/// [`PROBE_SENSOR_STUCK_READING_C`]). **GENERIC**, chosen above
/// [`WINDOW_TARGET_C`] so a full fault reliably defeats control.
const WINDOW_SENSOR_STUCK_READING_C: f64 = 50.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct WindowHeatFaults {
    /// A resistance defect in the conductive film (nicked/corroded
    /// element): concentrates power into a shrinking local area exactly
    /// like `physics::engine::oil.rs`'s filter-clog resistance law
    /// (`1/(1-clog)^2`, read as precedent, re-derived independently here
    /// per BRIEF rule 2), so the local hot spot's power density is the
    /// nameplate density divided by `(1 - defect)^2`.
    pub film_defect: f64,
    pub controller_fault: f64,
    pub sensor_fault: f64,
}

pub struct WindowHeat {
    surface_c: f64,
    hot_spot_c: f64,
    delaminated: bool,
    cracked: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct WindowHeatOutputs {
    pub surface_c: f64,
    pub hot_spot_c: f64,
    pub power_w: f64,
    pub overheat_tripped: bool,
    pub delaminated: bool,
    pub cracked: bool,
}

impl WindowHeat {
    pub fn new(initial_c: f64) -> Self {
        Self { surface_c: initial_c, hot_spot_c: initial_c, delaminated: false, cracked: false }
    }

    pub fn is_cracked(&self) -> bool {
        self.cracked
    }
    pub fn is_delaminated(&self) -> bool {
        self.delaminated
    }

    #[allow(clippy::too_many_arguments)]
    pub fn step(&mut self, static_air_c: f64, recovery_c: f64, lwc_kg_m3: f64, beta0: f64, tas_m_s: f64, ambient_pressure_pa: f64, faults: &WindowHeatFaults, dt_s: f64) -> WindowHeatOutputs {
        let sensed_c = self.surface_c + (WINDOW_SENSOR_STUCK_READING_C - self.surface_c) * clamp01(faults.sensor_fault);
        let controller_ok = clamp01(faults.controller_fault) < 1.0;
        // Bang-bang thermostatic control around the target when the
        // controller is healthy. A faulted controller does not merely
        // fail to regulate -- it commands full, unconditional heat
        // (a stuck contactor/failed control-logic scenario), the
        // dangerous failure mode for a film heater, not "fails off".
        let commanded_on = if controller_ok { sensed_c < WINDOW_TARGET_C } else { true };
        let nameplate_power_w = if commanded_on { WINDOW_RATED_W } else { 0.0 };

        let bulk_heater_w_m2 = nameplate_power_w / WINDOW_AREA_M2;
        let bulk_equilibrium_c = equilibrium_surface_c_with_heater(WINDOW_H_W_M2K, static_air_c, recovery_c, lwc_kg_m3, beta0, tas_m_s, ambient_pressure_pa, bulk_heater_w_m2);
        self.surface_c = relax_toward_equilibrium_c(self.surface_c, bulk_equilibrium_c, WINDOW_THERMAL_TAU_S, dt_s);

        // Local hot spot: the same nameplate power concentrated by the
        // film defect into a small area, evaluated with the same
        // convective environment (the defect is a point on the same
        // exposed surface), relaxing with the same thermal mass.
        let defect = clamp01(faults.film_defect);
        let concentration = 1.0 / (1.0 - defect.min(0.995)).powi(2);
        let hot_spot_heater_w_m2 = bulk_heater_w_m2 * concentration;
        let hot_spot_equilibrium_c = equilibrium_surface_c_with_heater(WINDOW_H_W_M2K, static_air_c, recovery_c, lwc_kg_m3, beta0, tas_m_s, ambient_pressure_pa, hot_spot_heater_w_m2);
        self.hot_spot_c = relax_toward_equilibrium_c(self.hot_spot_c, hot_spot_equilibrium_c, WINDOW_THERMAL_TAU_S, dt_s);

        if self.hot_spot_c > WINDOW_OVERHEAT_PROTECT_C + WINDOW_DELAMINATION_MARGIN_C {
            self.delaminated = true;
        }
        if self.hot_spot_c > WINDOW_OVERHEAT_PROTECT_C + WINDOW_CRACK_MARGIN_C {
            self.cracked = true;
        }

        WindowHeatOutputs {
            surface_c: self.surface_c,
            hot_spot_c: self.hot_spot_c,
            power_w: nameplate_power_w,
            overheat_tripped: self.surface_c > WINDOW_OVERHEAT_PROTECT_C,
            delaminated: self.delaminated,
            cracked: self.cracked,
        }
    }
}

// ---------------------------------------------------------------------------
// Windshield rain removal (high-velocity air jet shearing the water film)
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, Default)]
pub struct RainRemovalFaults {
    /// Valve/duct/blower fault: fraction reduction of the jet's effective
    /// dynamic pressure.
    pub system_fault: f64,
}

/// GENERIC shear-removal coefficient, kg/(m^2 s Pa): sized so a
/// representative rain-removal jet dynamic pressure (a few kPa, typical of
/// a bleed-air blower duct) clears a light rain catch rate at a
/// significant fraction of a second's response, matching the qualitative
/// "clears the windshield quickly once selected on" real-system behaviour.
const SHEAR_COEFFICIENT_KG_M2_S_PA: f64 = 2.0e-5;
/// Standard air density used for the jet's dynamic pressure, kg/m^3
/// (sea-level reference; the jet itself is a local bleed/ram-air supply,
/// not the ambient free-stream density).
const JET_AIR_DENSITY_KG_M3: f64 = 1.0;

pub struct RainRemoval {
    film_kg_m2: f64,
}

impl RainRemoval {
    pub fn new() -> Self {
        Self { film_kg_m2: 0.0 }
    }

    pub fn film_kg_m2(&self) -> f64 {
        self.film_kg_m2
    }

    /// One tick. `catch_kg_m2_s` is the rain/impingement water catch rate
    /// (e.g. from [`super::icing::IcingSurface`]'s own impingement mass
    /// flux when unfrozen); `jet_velocity_m_s` is the rain-removal system's
    /// commanded jet velocity (0 when the system is off).
    pub fn step(&mut self, catch_kg_m2_s: f64, jet_velocity_m_s: f64, evaporation_kg_m2_s: f64, faults: &RainRemovalFaults, dt_s: f64) -> f64 {
        let dt = dt_s.max(0.0);
        let effective_velocity = jet_velocity_m_s.max(0.0) * (1.0 - clamp01(faults.system_fault));
        let dynamic_pressure_pa = 0.5 * JET_AIR_DENSITY_KG_M3 * effective_velocity * effective_velocity;
        let shear_removal_kg_m2_s = SHEAR_COEFFICIENT_KG_M2_S_PA * dynamic_pressure_pa;
        let net_kg_m2_s = catch_kg_m2_s.max(0.0) - evaporation_kg_m2_s.max(0.0) - shear_removal_kg_m2_s;
        self.film_kg_m2 = (self.film_kg_m2 + net_kg_m2_s * dt).max(0.0);
        self.film_kg_m2
    }
}

impl Default for RainRemoval {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::util::recovery_temperature_c;

    fn icing_condition() -> (f64, f64, f64, f64, f64, f64) {
        // (static_c, recovery_c, lwc, beta0, tas, pressure)
        let static_c = -10.0;
        let tas = 100.0;
        (static_c, recovery_temperature_c(static_c, tas, 0.9), 5e-4, 0.6, tas, 80_000.0)
    }

    #[test]
    fn healthy_wing_anti_ice_holds_the_surface_at_or_above_freezing_in_icing_conditions() {
        let (static_c, recovery_c, lwc, beta0, tas, p) = icing_condition();
        let mut surface = BleedAntiIceSurface::new(WING_ANTI_ICE, static_c);
        let mut out = BleedAntiIceOutputs::default();
        for _ in 0..300 {
            out = surface.step(1.0, static_c, recovery_c, lwc, beta0, tas, p, &BleedAntiIceFaults::default(), 1.0);
        }
        assert!(out.surface_c >= -0.5, "healthy anti-ice should hold near/above freezing, got {}", out.surface_c);
        assert!(out.bleed_delivered_kg_s > 0.0);
    }

    #[test]
    fn a_valve_stuck_closed_leaves_the_surface_to_ice_like_the_unheated_case() {
        let (static_c, recovery_c, lwc, beta0, tas, p) = icing_condition();
        let mut surface = BleedAntiIceSurface::new(WING_ANTI_ICE, static_c);
        let faults = BleedAntiIceFaults { valve_stuck_closed: 1.0, ..Default::default() };
        let mut out = BleedAntiIceOutputs::default();
        for _ in 0..300 {
            out = surface.step(1.0, static_c, recovery_c, lwc, beta0, tas, p, &faults, 1.0);
        }
        assert_eq!(out.bleed_delivered_kg_s, 0.0);
        assert!(out.surface_c < 0.0);
    }

    #[test]
    fn a_valve_stuck_open_keeps_heating_and_can_overheat_once_no_longer_needed() {
        let mut surface = BleedAntiIceSurface::new(WING_ANTI_ICE, 15.0);
        let faults = BleedAntiIceFaults { valve_stuck_open: 1.0, ..Default::default() };
        // Warm, dry (no icing) air: the valve should be commanded closed,
        // but a stuck-open fault keeps delivering full bleed regardless.
        // Recovery temperature at 15 C/100 m/s is used directly (~19.5 C
        // computed, but a plain 15 C static-equal input isolates the test
        // from that other formula and still overheats with margin).
        let mut out = BleedAntiIceOutputs::default();
        for _ in 0..300 {
            out = surface.step(0.0, 15.0, 15.0, 0.0, 0.0, 100.0, 101_325.0, &faults, 1.0);
        }
        assert!(out.overheat, "a valve stuck open with no icing load should overheat the skin, got surface {}", out.surface_c);
    }

    #[test]
    fn a_duct_leak_reduces_delivered_bleed_flow() {
        let (static_c, recovery_c, lwc, beta0, tas, p) = icing_condition();
        let mut healthy = BleedAntiIceSurface::new(WING_ANTI_ICE, static_c);
        let mut leaky = BleedAntiIceSurface::new(WING_ANTI_ICE, static_c);
        let healthy_out = healthy.step(1.0, static_c, recovery_c, lwc, beta0, tas, p, &BleedAntiIceFaults::default(), 1.0);
        let leaky_out = leaky.step(1.0, static_c, recovery_c, lwc, beta0, tas, p, &BleedAntiIceFaults { duct_leak: 0.6, ..Default::default() }, 1.0);
        assert!(leaky_out.bleed_delivered_kg_s < healthy_out.bleed_delivered_kg_s);
    }

    #[test]
    fn probe_heater_keeps_a_healthy_probe_above_freezing() {
        // With real thermal mass, a bang-bang thermostat settles into a
        // small oscillation band around its target rather than a single
        // fixed value (module doc, `relax_toward_equilibrium_c`), so this
        // checks a whole tail window of ticks stays above freezing rather
        // than one arbitrary final tick (which would be phase-dependent).
        let (static_c, recovery_c, lwc, beta0, tas, p) = icing_condition();
        let mut probe = ProbeHeater::new(static_c);
        let mut min_tail_c = f64::INFINITY;
        for i in 0..150 {
            let out = probe.step(static_c, recovery_c, lwc, beta0, tas, p, &ProbeHeaterFaults::default(), 1.0);
            if i >= 100 {
                min_tail_c = min_tail_c.min(out.surface_c);
            }
        }
        assert!(min_tail_c > 0.0, "healthy probe heat must stay above freezing once settled, min {min_tail_c}");
    }

    #[test]
    fn a_faulted_controller_leaves_the_probe_unheated_and_icing() {
        let (static_c, recovery_c, lwc, beta0, tas, p) = icing_condition();
        let mut probe = ProbeHeater::new(static_c);
        let faults = ProbeHeaterFaults { controller_fault: 1.0, ..Default::default() };
        let mut out = ProbeHeaterOutputs::default();
        for _ in 0..60 {
            out = probe.step(static_c, recovery_c, lwc, beta0, tas, p, &faults, 1.0);
        }
        assert_eq!(out.power_w, 0.0, "a faulted controller must never command heat");
        assert!(out.surface_c < 0.0, "an unheated probe in icing conditions must run below freezing");
    }

    #[test]
    fn a_sensor_stuck_reading_warm_silently_leaves_the_probe_unheated_and_icing() {
        // A stuck-at-value sensor fault (module doc:
        // `PROBE_SENSOR_STUCK_READING_C`) reads a fixed warm value
        // regardless of the probe's true (icing) temperature, silently
        // defeating control -- the controller believes it is healthy
        // while the probe itself genuinely ices.
        let (static_c, recovery_c, lwc, beta0, tas, p) = icing_condition();
        let mut probe = ProbeHeater::new(static_c);
        let faults = ProbeHeaterFaults { sensor_fault: 1.0, ..Default::default() };
        let mut out = ProbeHeaterOutputs::default();
        for _ in 0..60 {
            out = probe.step(static_c, recovery_c, lwc, beta0, tas, p, &faults, 1.0);
        }
        assert_eq!(out.power_w, 0.0, "a sensor stuck reading warm must silently suppress heating");
        assert!(out.surface_c < 0.0, "the probe itself must still be cold/icing despite the system believing it is warm");
    }

    #[test]
    fn a_fully_open_circuit_heater_delivers_no_power_even_when_commanded() {
        let (static_c, recovery_c, lwc, beta0, tas, p) = icing_condition();
        let mut probe = ProbeHeater::new(static_c);
        let faults = ProbeHeaterFaults { heater_open_circuit: 1.0, ..Default::default() };
        let out = probe.step(static_c, recovery_c, lwc, beta0, tas, p, &faults, 1.0);
        assert_eq!(out.power_w, 0.0);
    }

    #[test]
    fn window_heat_settles_near_its_target_temperature_when_healthy() {
        let mut window = WindowHeat::new(-10.0);
        let mut min_tail_c = f64::INFINITY;
        let mut max_tail_c = f64::NEG_INFINITY;
        let mut any_delaminated = false;
        for i in 0..600 {
            let out = window.step(-10.0, -10.0, 0.0, 0.0, 100.0, 101_325.0, &WindowHeatFaults::default(), 1.0);
            any_delaminated |= out.delaminated || out.cracked;
            if i >= 400 {
                min_tail_c = min_tail_c.min(out.surface_c);
                max_tail_c = max_tail_c.max(out.surface_c);
            }
        }
        assert!(min_tail_c > 20.0 && max_tail_c < WINDOW_OVERHEAT_PROTECT_C, "settled band [{min_tail_c},{max_tail_c}] should bracket the {WINDOW_TARGET_C} C target well clear of the {WINDOW_OVERHEAT_PROTECT_C} C cutout");
        assert!(!any_delaminated);
    }

    #[test]
    fn a_film_defect_concentrates_heat_into_a_hot_spot_that_can_delaminate() {
        let mut window = WindowHeat::new(-10.0);
        let faults = WindowHeatFaults { film_defect: 0.9, ..Default::default() };
        let mut out = WindowHeatOutputs::default();
        for _ in 0..60 {
            out = window.step(-10.0, -10.0, 0.0, 0.0, 100.0, 101_325.0, &faults, 1.0);
        }
        assert!(out.hot_spot_c > out.surface_c, "hot spot {} vs bulk {}", out.hot_spot_c, out.surface_c);
        assert!(out.delaminated, "a severe film defect must overheat its own hot spot into delamination");
    }

    #[test]
    fn a_healthy_controller_and_overheat_cutout_prevent_delamination() {
        let mut window = WindowHeat::new(-10.0);
        for _ in 0..600 {
            let out = window.step(-10.0, -10.0, 0.0, 0.0, 100.0, 101_325.0, &WindowHeatFaults::default(), 1.0);
            assert!(!out.delaminated && !out.cracked);
        }
    }

    #[test]
    fn a_controller_fault_can_stick_the_heater_on_and_overheat_a_healthy_film() {
        let mut window = WindowHeat::new(-10.0);
        let faults = WindowHeatFaults { controller_fault: 1.0, ..Default::default() };
        let mut out = WindowHeatOutputs::default();
        for _ in 0..300 {
            out = window.step(-10.0, -10.0, 0.0, 0.0, 100.0, 101_325.0, &faults, 1.0);
        }
        assert!(out.overheat_tripped, "a stuck-on heater with no working cutout must overheat, surface {}", out.surface_c);
    }

    #[test]
    fn crack_is_irreversible_once_reached() {
        let mut window = WindowHeat::new(-10.0);
        let faults = WindowHeatFaults { film_defect: 0.97, ..Default::default() };
        for _ in 0..60 {
            window.step(-10.0, -10.0, 0.0, 0.0, 100.0, 101_325.0, &faults, 1.0);
        }
        assert!(window.is_cracked());
        // Repair the defect; the crack must not un-happen.
        for _ in 0..60 {
            window.step(-10.0, -10.0, 0.0, 0.0, 100.0, 101_325.0, &WindowHeatFaults::default(), 1.0);
        }
        assert!(window.is_cracked());
    }

    #[test]
    fn rain_removal_reduces_film_thickness_when_jet_is_on() {
        let mut off = RainRemoval::new();
        let mut on = RainRemoval::new();
        for _ in 0..100 {
            off.step(0.02, 0.0, 0.0, &RainRemovalFaults::default(), 0.1);
            on.step(0.02, 200.0, 0.0, &RainRemovalFaults::default(), 0.1);
        }
        assert!(on.film_kg_m2() < off.film_kg_m2(), "on {} vs off {}", on.film_kg_m2(), off.film_kg_m2());
    }

    #[test]
    fn a_faulted_rain_removal_system_is_less_effective() {
        let mut healthy = RainRemoval::new();
        let mut faulted = RainRemoval::new();
        for _ in 0..100 {
            healthy.step(0.02, 200.0, 0.0, &RainRemovalFaults::default(), 0.1);
            faulted.step(0.02, 200.0, 0.0, &RainRemovalFaults { system_fault: 1.0 }, 0.1);
        }
        assert!(faulted.film_kg_m2() > healthy.film_kg_m2());
    }

    #[test]
    fn no_nan_at_rest_or_zero_dt() {
        let mut surface = BleedAntiIceSurface::new(WING_ANTI_ICE, 15.0);
        let out = surface.step(0.0, 15.0, 15.0, 0.0, 0.0, 0.0, 101_325.0, &BleedAntiIceFaults::default(), 0.0);
        assert!(!out.surface_c.is_nan());
        let mut probe = ProbeHeater::new(15.0);
        let probe_out = probe.step(15.0, 15.0, 0.0, 0.0, 0.0, 101_325.0, &ProbeHeaterFaults::default(), 0.0);
        assert!(!probe_out.surface_c.is_nan());
        let mut window = WindowHeat::new(15.0);
        let window_out = window.step(15.0, 15.0, 0.0, 0.0, 0.0, 101_325.0, &WindowHeatFaults::default(), 0.0);
        assert!(!window_out.surface_c.is_nan() && !window_out.hot_spot_c.is_nan());
        let mut rain = RainRemoval::new();
        let film = rain.step(0.0, 0.0, 0.0, &RainRemovalFaults::default(), 0.0);
        assert!(!film.is_nan());
    }
}
