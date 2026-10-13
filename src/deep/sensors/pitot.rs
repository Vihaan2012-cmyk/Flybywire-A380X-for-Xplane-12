//! Pitot probe: the total-pressure pickup, its anti-ice heater and heat
//! balance, drain hole, ice accretion/blockage, non-ice (insect/tape)
//! blockage, and the pneumatic lag of the line back to the ADR.
//!
//! ## Why the drain hole matters
//! A pitot tube is a forward-facing hole feeding a `-2..3 mm` drain hole near
//! its base, there so any water that gets in can weep out rather than
//! collect and freeze. Its state changes how a blocked tube fails, which is
//! standard, publicly documented pitot-static system knowledge (FAA
//! *Airplane Flying Handbook* (FAA-H-8083-3), ch. 4, "Pitot-Static System
//! and Instruments -- Blockage"; also FAA AC 25-1419-1B, "flight in icing
//! conditions"):
//! - Tube blocked, drain clear: trapped air behind the blockage leaks out
//!   through the drain, so the sensed total pressure decays toward the
//!   (still-good) static pressure -- indicated airspeed sags toward zero
//!   regardless of what the aircraft actually does.
//! - Tube *and* drain both blocked: the trapped pressure has nowhere to go.
//!   It stays frozen at whatever it was the instant of blockage, so the
//!   computed airspeed instead starts tracking (inversely) *altitude*
//!   changes, because it is really `frozen_total - falling_static` doing the
//!   work of an altimeter, not an airspeed indicator. This is the physical
//!   mechanism behind the historical "unreliable airspeed" pitot-icing
//!   upsets: a climb after blockage-with-drain-closed reads an *increasing*
//!   speed (falling static widens the frozen delta) even though the true
//!   airspeed may be falling.
//!
//! ## Sources
//! - Pneumatic lag: Gracey, W., *Measurement of Aircraft Speed and Altitude*,
//!   NASA Reference Publication 1046 (1980), discusses the first-order lag of
//!   pitot-static tubing/orifices -- used here as the standard model for an
//!   unrestricted line's response time.
//! - Heater/heat-balance correlations: cylinder-in-crossflow convection
//!   (Zukauskas correlation, Incropera & DeWitt, *Fundamentals of Heat and
//!   Mass Transfer*, Table 7.4) and a Messinger-style surface energy balance
//!   (Messinger, "Equilibrium Temperature of an Unheated Icing Surface as a
//!   Function of Air Speed", J. Aeronautical Sciences, 1953) -- the same
//!   published method `src/physics/adirs.rs` cites; re-derived independently
//!   here per this brief's no-cross-module-dependency rule, not copied.
//! - Probe geometry (13 mm dia, 150 mm exposed) and heater rating (350 W):
//!   representative Rosemount-class pitot probe figures from public product
//!   literature, not A380-specific -- GENERIC.
//! - Ice-block mass (the amount of ice that closes the ~2-3 mm orifice):
//!   nothing public gives an exact figure for this orifice; GENERIC, sized
//!   so continuous unheated icing blocks the tube within about a minute,
//!   consistent with why probe heat is mandatory equipment.

use std::f64::consts::PI;

/// Probe geometry: outer diameter and aerodynamically exposed length,
/// metres. GENERIC (see module docs).
const PROBE_DIAMETER_M: f64 = 0.013;
const PROBE_EXPOSED_LENGTH_M: f64 = 0.15;
/// Rated heater power, W. GENERIC.
pub const RATED_HEATER_W: f64 = 350.0;
/// Target surface temperature the heater holds against convective/water-catch
/// cooling: the certification-relevant floor, just above freezing.
const TARGET_SURFACE_C: f64 = 0.0;

/// Air thermal conductivity, kinematic viscosity and Prandtl number at a
/// representative cold (~-20 C) icing-altitude condition (Incropera & DeWitt,
/// Table A.4), held constant rather than temperature-interpolated -- a
/// second-order refinement next to the heater-power/LWC effects this module
/// targets.
const AIR_K_W_MK: f64 = 0.0206;
const AIR_NU_M2_S: f64 = 1.13e-5;
const AIR_PR: f64 = 0.72;
const WATER_CP_J_KGK: f64 = 4186.0;
const WATER_LF_J_KG: f64 = 334_000.0;
/// A representative "high LWC" figure, at the upper end of the FAA/CS-25
/// Appendix C continuous-maximum-icing envelope (14 CFR Part 25 Appendix C).
const REFERENCE_LWC_GM3: f64 = 0.6;

/// Ice mass that closes the pitot orifice, kg. GENERIC (see module docs):
/// sized to block an unheated probe within roughly a minute of continuous
/// icing at [`REFERENCE_LWC_GM3`] and typical approach speed.
const ICE_BLOCK_MASS_KG: f64 = 0.0012;

/// Baseline pneumatic time constant of an unrestricted pitot line, seconds.
/// GENERIC, in line with the order-of-magnitude lag Gracey (NASA RP-1046)
/// describes for typical transport pitot-static installations (a few tenths
/// of a second to about a second, growing with altitude/reduced density and
/// line length).
const PNEUMATIC_TAU_S: f64 = 0.25;
/// Time constant for a blocked tube with a clear drain to bleed down toward
/// static pressure, seconds. GENERIC: the drain orifice is much smaller than
/// the main tube, so this is slower than the healthy pneumatic lag but still
/// fast enough that airspeed visibly decays over several seconds, matching
/// the documented "airspeed unwinds toward zero" behaviour.
const DRAIN_LEAK_TAU_S: f64 = 4.0;
/// Below this fraction of the tube's nominal open area, the probe is treated
/// as blocked (drain-hole logic takes over) rather than merely restricted.
const BLOCKED_OPEN_FRACTION: f64 = 0.03;

/// Convective heat-transfer coefficient for crossflow over a cylinder
/// (Zukauskas correlation, Nu = C*Re^m*Pr^0.37).
fn convective_h_w_m2k(tas_ms: f64) -> f64 {
    let v = tas_ms.max(0.0);
    let re = v * PROBE_DIAMETER_M / AIR_NU_M2_S;
    if re <= 1.0 {
        return 0.0;
    }
    let (c, m) = if re <= 2.0e5 { (0.26, 0.6) } else { (0.076, 0.7) };
    let nu = c * re.powf(m) * AIR_PR.powf(0.37);
    nu * AIR_K_W_MK / PROBE_DIAMETER_M
}

/// Heat, W, the heater must supply at this SAT/TAS/LWC to hold
/// [`TARGET_SURFACE_C`]: dry-air convection plus warming/keeping-liquid the
/// water the frontal area catches. Returns `(required_w, catch_kg_s)`.
fn heat_required_w(sat_c: f64, tas_ms: f64, lwc_gm3: f64) -> (f64, f64) {
    let delta_t = (TARGET_SURFACE_C - sat_c).max(0.0);
    let h = convective_h_w_m2k(tas_ms);
    let conv_area_m2 = PI * PROBE_DIAMETER_M * PROBE_EXPOSED_LENGTH_M;
    let q_conv = h * conv_area_m2 * delta_t;

    let frontal_area_m2 = PROBE_DIAMETER_M * PROBE_EXPOSED_LENGTH_M;
    let lwc_kg_m3 = lwc_gm3.max(0.0) * 1e-3;
    let catch_kg_s = lwc_kg_m3 * tas_ms.max(0.0) * frontal_area_m2;
    let q_water = catch_kg_s * (WATER_CP_J_KGK * delta_t + WATER_LF_J_KG);

    (q_conv + q_water, catch_kg_s)
}

/// Fault inputs, each a fraction `0.0` (healthy) `..1.0` (fully failed).
#[derive(Clone, Copy, Debug, Default)]
pub struct PitotFaults {
    /// Heater element/circuit power loss.
    pub heater_failure: f64,
    /// Fixed mechanical restriction independent of icing: insects, a
    /// forgotten pitot cover's tape residue, corrosion. Reduces the tube's
    /// effective open area.
    pub insect_or_tape_blockage: f64,
    /// The tube is bent/crushed (ramp equipment strike, bird strike):
    /// reduces effective open area, same mechanism as
    /// `insect_or_tape_blockage` but a separate cause for reporting.
    pub mechanical_damage: f64,
    /// The drain hole itself is blocked (corrosion, its own ice, foreign
    /// object). `0.0` = fully clear, `1.0` = fully sealed.
    pub drain_blocked: f64,
}

/// What the probe senses this tick, before the ADR inverts it into
/// CAS/Mach/altitude (that inversion is [`super::adr`]'s job).
#[derive(Clone, Copy, Debug, Default)]
pub struct PitotOutput {
    /// Pressure the ADR actually receives at its pitot input, Pa.
    pub sensed_total_pressure_pa: f64,
    /// True (tip) total pressure is unreachable because the tube is
    /// substantially closed.
    pub blocked: bool,
    /// Accreted ice mass at the orifice, kg (Study/diagnostic use).
    pub ice_kg: f64,
    /// Heater power actually delivered, W (electrical load reporting).
    pub heater_power_w: f64,
}

/// One pitot probe's state.
#[derive(Clone, Copy, Debug)]
pub struct PitotProbe {
    ice_kg: f64,
    /// The pressure the ADR reads: tracks true total pressure when healthy,
    /// decays to static or freezes when blocked (see module docs).
    sensed_pa: f64,
}

impl PitotProbe {
    /// `initial_total_pa`: the true total pressure at spawn, so the probe
    /// starts agreeing with reality rather than lagging in from zero.
    pub fn new(initial_total_pa: f64) -> Self {
        Self { ice_kg: 0.0, sensed_pa: initial_total_pa }
    }

    /// Advances the probe by `dt_s`.
    ///
    /// - `true_total_pa`, `true_static_pa`: the pressures a perfect probe
    ///   would see at the tip this instant.
    /// - `tas_ms`, `sat_c`, `lwc_gm3`: for the heat balance.
    /// - `powered`: the heater's electrical bus is live (a dead bus can
    ///   never heat the probe regardless of `faults.heater_failure`).
    pub fn step(
        &mut self,
        true_total_pa: f64,
        true_static_pa: f64,
        tas_ms: f64,
        sat_c: f64,
        lwc_gm3: f64,
        powered: bool,
        faults: &PitotFaults,
        dt_s: f64,
    ) -> PitotOutput {
        let dt = dt_s.max(0.0);

        // ---- Heater and ice heat balance (see module docs).
        let rated_w = if powered { RATED_HEATER_W * (1.0 - faults.heater_failure.clamp(0.0, 1.0)) } else { 0.0 };
        let (required_w, catch_kg_s) = heat_required_w(sat_c, tas_ms, lwc_gm3);
        let deficit_w = (required_w - rated_w).max(0.0);
        let surplus_w = (rated_w - required_w).max(0.0);
        let accretion_kg_s = if required_w > 0.0 { catch_kg_s * (deficit_w / required_w).min(1.0) } else { 0.0 };
        let melt_kg_s = surplus_w / WATER_LF_J_KG;
        self.ice_kg = (self.ice_kg + (accretion_kg_s - melt_kg_s) * dt).max(0.0);

        // ---- Open-area fraction: ice plus the two fixed mechanical causes
        // all narrow the same one orifice, so their effects multiply (each
        // is an independent fractional restriction of what's left).
        let ice_open = (1.0 - (self.ice_kg / ICE_BLOCK_MASS_KG).min(1.0)).max(0.0);
        let mech_open = (1.0 - faults.insect_or_tape_blockage.clamp(0.0, 1.0))
            * (1.0 - faults.mechanical_damage.clamp(0.0, 1.0));
        let open_fraction = (ice_open * mech_open).clamp(0.0, 1.0);
        let blocked = open_fraction < BLOCKED_OPEN_FRACTION;

        if blocked {
            // On the tick blockage begins, `self.sensed_pa` already holds
            // the last free-flowing reading -- that becomes the "trapped"
            // pressure the branches below decay from or freeze at.
            let drain_clear = 1.0 - faults.drain_blocked.clamp(0.0, 1.0);
            if drain_clear > 0.02 {
                // Leaks toward static pressure; a more-open drain leaks
                // faster (shorter effective time constant).
                let tau = (DRAIN_LEAK_TAU_S / drain_clear.max(0.02)).max(0.1);
                let k = (-dt / tau).exp();
                self.sensed_pa = true_static_pa + (self.sensed_pa - true_static_pa) * k;
            }
            // Drain (substantially) blocked too: trapped, `sensed_pa`
            // simply does not move -- this is the frozen-differential
            // behaviour the module docs describe.
        } else {
            // Healthy or merely restricted: first-order pneumatic lag
            // toward the true total pressure. A partly closed orifice adds
            // resistance; for small-signal orifice flow, resistance scales
            // roughly as the inverse square of the open area (Q ~ A*sqrt(dP),
            // so the RC time constant ~ 1/A^2 for a fixed downstream volume).
            let tau = PNEUMATIC_TAU_S / (open_fraction * open_fraction).max(1e-3);
            let k = (-dt / tau.max(1e-6)).exp();
            self.sensed_pa = true_total_pa + (self.sensed_pa - true_total_pa) * k;
        }

        PitotOutput {
            sensed_total_pressure_pa: self.sensed_pa,
            blocked,
            ice_kg: self.ice_kg,
            heater_power_w: rated_w,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn step_n(probe: &mut PitotProbe, total: f64, static_p: f64, tas: f64, sat_c: f64, lwc: f64, powered: bool, faults: &PitotFaults, dt: f64, n: u32) -> PitotOutput {
        let mut out = PitotOutput::default();
        for _ in 0..n {
            out = probe.step(total, static_p, tas, sat_c, lwc, powered, faults, dt);
        }
        out
    }

    #[test]
    fn healthy_probe_tracks_a_pressure_step_within_a_few_time_constants() {
        let mut probe = PitotProbe::new(101_325.0);
        let out = step_n(&mut probe, 108_000.0, 95_000.0, 200.0, 15.0, 0.0, true, &PitotFaults::default(), 0.05, 200);
        assert!((out.sensed_total_pressure_pa - 108_000.0).abs() < 10.0, "{}", out.sensed_total_pressure_pa);
        assert!(!out.blocked);
    }

    #[test]
    fn no_nan_at_zero_dt_or_rest() {
        let mut probe = PitotProbe::new(0.0);
        let out = probe.step(0.0, 0.0, 0.0, 15.0, 0.0, false, &PitotFaults::default(), 0.0);
        assert!(out.sensed_total_pressure_pa.is_finite());
        assert!(!out.sensed_total_pressure_pa.is_nan());
    }

    #[test]
    fn unheated_icing_eventually_blocks_the_tube() {
        let mut probe = PitotProbe::new(101_325.0);
        let faults = PitotFaults::default();
        // Cold, wet, fast, unpowered heater: heavy icing.
        let out = step_n(&mut probe, 108_000.0, 95_000.0, 220.0, -20.0, 0.6, false, &faults, 0.5, 400);
        assert!(out.blocked, "ice_kg={}", out.ice_kg);
        assert!(out.ice_kg > 0.0);
    }

    #[test]
    fn heater_prevents_blockage_in_the_same_icing_conditions() {
        let mut probe = PitotProbe::new(101_325.0);
        let faults = PitotFaults::default();
        let out = step_n(&mut probe, 108_000.0, 95_000.0, 220.0, -20.0, 0.6, true, &faults, 0.5, 400);
        assert!(!out.blocked);
        assert_eq!(out.ice_kg, 0.0);
    }

    #[test]
    fn blocked_tube_with_clear_drain_decays_toward_static_pressure() {
        let mut probe = PitotProbe::new(101_325.0);
        // Force a hard mechanical blockage (bird strike) with the drain
        // clear, rather than waiting on the icing timer.
        let faults = PitotFaults { mechanical_damage: 1.0, ..Default::default() };
        let true_total = 108_000.0;
        let true_static = 95_000.0;
        let out = step_n(&mut probe, true_total, true_static, 200.0, 15.0, 0.0, true, &faults, 0.5, 60);
        assert!(out.blocked);
        assert!((out.sensed_total_pressure_pa - true_static).abs() < 500.0, "{}", out.sensed_total_pressure_pa);
    }

    #[test]
    fn blocked_tube_with_blocked_drain_freezes_regardless_of_true_pressure_changes() {
        let mut probe = PitotProbe::new(101_325.0);
        let faults = PitotFaults { mechanical_damage: 1.0, drain_blocked: 1.0, ..Default::default() };
        // One tick to record the frozen value at (true_total=108000).
        let frozen = probe.step(108_000.0, 95_000.0, 200.0, 15.0, 0.0, true, &faults, 0.5).sensed_total_pressure_pa;
        // Now the aircraft climbs a lot: static and total pressure both
        // fall a great deal. The frozen reading must not follow.
        let out = step_n(&mut probe, 40_000.0, 30_000.0, 250.0, -40.0, 0.0, true, &faults, 0.5, 200);
        assert!((out.sensed_total_pressure_pa - frozen).abs() < 1.0, "frozen {} now {}", frozen, out.sensed_total_pressure_pa);
    }

    #[test]
    fn partial_insect_blockage_slows_the_pneumatic_response_without_declaring_blocked() {
        let mut clean = PitotProbe::new(101_325.0);
        let mut restricted = PitotProbe::new(101_325.0);
        let restricted_faults = PitotFaults { insect_or_tape_blockage: 0.5, ..Default::default() };
        let clean_out = clean.step(108_000.0, 95_000.0, 200.0, 15.0, 0.0, true, &PitotFaults::default(), 0.2);
        let restricted_out = restricted.step(108_000.0, 95_000.0, 200.0, 15.0, 0.0, true, &restricted_faults, 0.2);
        assert!(!restricted_out.blocked);
        // The restricted probe should have moved less far toward the true
        // value in the same single tick.
        let clean_progress = (clean_out.sensed_total_pressure_pa - 101_325.0).abs();
        let restricted_progress = (restricted_out.sensed_total_pressure_pa - 101_325.0).abs();
        assert!(restricted_progress < clean_progress, "clean {clean_progress}, restricted {restricted_progress}");
    }

    #[test]
    fn severe_insect_blockage_is_treated_as_blocked() {
        let mut probe = PitotProbe::new(101_325.0);
        let faults = PitotFaults { insect_or_tape_blockage: 0.99, ..Default::default() };
        let out = probe.step(108_000.0, 95_000.0, 200.0, 15.0, 0.0, true, &faults, 0.1);
        assert!(out.blocked);
    }

    #[test]
    fn heater_power_reports_zero_when_unpowered_even_if_the_element_is_healthy() {
        let mut probe = PitotProbe::new(101_325.0);
        let out = probe.step(108_000.0, 95_000.0, 200.0, -20.0, 0.6, false, &PitotFaults::default(), 0.1);
        assert_eq!(out.heater_power_w, 0.0);
    }
}
