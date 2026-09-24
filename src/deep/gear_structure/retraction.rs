//! Retraction/extension: the hydraulic actuator that swings the leg, its
//! mechanical uplock and (spring-loaded) downlock, gravity/free-fall
//! extension, gear-door sequencing, and the proximity-sensor indications a
//! cockpit would read (which can be made to lie, independent of the true
//! mechanical state).
//!
//! # Sequencing
//! Real gear systems interlock doors and gear travel: the doors must be
//! (mostly) open before the leg is allowed to move, and close again once
//! the leg reaches its target and locks (FlyByWire's own A380 LGCIU state
//! machine, `fbw-common/.../landing_gear/mod.rs`'s
//! `GearSystemState::{Retracting,Extending}` gating `should_open_doors`/
//! `should_extend_gears` on `all_fully_opened()`/`all_down_and_locked()`, is
//! the same general shape). This module keeps that shape as a small local
//! state machine (`Phase`) rather than importing FlyByWire's, since this
//! crate must stay self-contained: `Locked -> DoorsOpening -> Traveling ->
//! DoorsClosing -> Locked`. Whether doors re-close once the gear is down
//! varies by aircraft type and is not publicly documented for the A380 at
//! this level of detail, so this module applies the same universal sequence
//! to both directions -- a documented, generic simplification.
//!
//! # Uplock release
//! An uplock is a spring-loaded hook; releasing it against gear weight and
//! air/spring preload normally takes hydraulic pressure. A gravity/free-fall
//! extension system exists precisely so a jammed *hydraulic* release path
//! can still be defeated by a separate, more direct mechanical or pneumatic
//! release cable/charge -- so a moderate uplock jam only defeats the normal
//! hydraulic release, while a severe jam (the hook itself fouled) defeats
//! even that. This is standard, publicly documented transport-category
//! landing gear design philosophy (the reason gravity extension exists at
//! all), not A380-specific data.
//!
//! # Downlocks
//! Downlocks are spring-loaded (an over-centre linkage or a hook), so they
//! engage from spring force alone once the leg reaches full extension,
//! regardless of whether hydraulic or gravity extension got it there. A
//! `downlock_fail` fault represents that spring/linkage failing to fully
//! seat: the leg still reaches the geometrically "down" position (so the
//! door sequence and position sensors show "down"), but the true
//! `downlocked` flag this module exposes -- which `strut::Strut::step`'s
//! `locked_down` input reads -- stays false, so any real touchdown load then
//! folds the leg (see `strut.rs`'s collapse condition). This is the
//! textbook "gear down but not locked" hazard.
//!
//! # Sensor lies
//! `sensor_lies` models a stuck/miscalibrated proximity sensor: above a
//! threshold, the *indicated* (sensed) lock state is the opposite of the
//! true one, independent of it, so a crew reading only the sensed value has
//! no way to tell the gear's true state from the indication alone -- the
//! physical hazard this fault represents.

use super::LegKind;

/// Full gear travel time at nominal hydraulic pressure, s (GENERIC,
/// order-of-magnitude for a large transport's main gear cycle).
const GEAR_NOMINAL_TRAVEL_S: f64 = 8.0;
/// Full door travel time (GENERIC: doors are lighter and faster than the
/// leg itself).
const DOOR_NOMINAL_TRAVEL_S: f64 = 4.0;
/// Gravity/free-fall extension travel time, s.
///
/// **Derived from a sourced figure**: the FCOM states that "landing gear
/// gravity extension takes approximately 70 s" (repeated throughout its
/// abnormal procedures, e.g. p.4846). That 70 s is the crew-facing,
/// end-to-end duration -- lever pulled to gear down and locked -- whereas
/// this constant is only the leg's own travel, which this model runs *after*
/// a door phase costing `DOOR_NOMINAL_TRAVEL_S`. So the travel time that
/// makes the whole sequence match the aircraft is 70 - 4 = 66 s, for a main
/// leg at `kind_rate_scale` 1.0.
///
/// The nose leg finishes sooner, as it does everywhere else in this model
/// (`kind_rate_scale`); the 70 s is governed by the slowest leg, which is
/// what the crew is waiting on.
const GRAVITY_EXTEND_TRAVEL_S: f64 = 66.0;
/// The end-to-end gravity extension duration the FCOM gives, s -- what
/// `GRAVITY_EXTEND_TRAVEL_S` is derived from, kept so the test can assert
/// the sequence against the sourced figure rather than the derived one.
const GRAVITY_EXTEND_TOTAL_S: f64 = 70.0;

/// Doors are considered clear of the gear's path above this position.
const DOOR_OPEN_THRESHOLD: f64 = 0.98;
/// Doors are considered fully shut below this position.
const DOOR_CLOSED_THRESHOLD: f64 = 0.02;
/// Gear positions counted as "reached" the up/down end stop.
const GEAR_UP_THRESHOLD: f64 = 0.02;
const GEAR_DOWN_THRESHOLD: f64 = 0.98;

/// Minimum system pressure fraction to release the uplock hydraulically at
/// all (GENERIC).
const MIN_RELEASE_PRESSURE_FRACTION: f64 = 0.3;
/// Above this jam severity the normal hydraulic uplock release can no
/// longer overcome the hook (GENERIC).
const UPLOCK_JAM_DEFEATS_HYDRAULIC: f64 = 0.5;
/// Above this jam severity even gravity extension's separate mechanical/
/// pneumatic release cannot free the hook (GENERIC: the hook itself, not
/// just its normal actuator, is fouled).
const UPLOCK_JAM_DEFEATS_EMERGENCY: f64 = 0.95;
/// Above this fault level the downlock spring/linkage fails to fully seat
/// (GENERIC).
const DOWNLOCK_ENGAGE_FAULT_THRESHOLD: f64 = 0.5;
/// Above this severity a jammed door is fully seized (GENERIC); below it,
/// the door still moves but proportionally slower.
const DOOR_JAM_FREEZE_THRESHOLD: f64 = 0.5;
/// A sensed lock indication is inverted from the true state above this
/// fault severity (GENERIC: below it, the sensor is considered to be
/// reading correctly).
const SENSOR_LIE_THRESHOLD: f64 = 0.5;

fn kind_rate_scale(kind: LegKind) -> f64 {
    match kind {
        // The nose gear/doors are smaller and lighter than the main gear's
        // (GENERIC scaling, not a cited A380 figure).
        LegKind::Nose => 1.3,
        LegKind::Wing | LegKind::Body => 1.0,
    }
}

/// Faults this leg's retraction system carries, each a fraction 0 (healthy)
/// .. 1 (fully failed).
#[derive(Clone, Copy, Debug, Default)]
pub struct RetractionFaults {
    /// Internal leak in the gear actuator: reduces available extend/retract
    /// speed and force.
    pub actuator_leak: f64,
    /// Uplock hook resists release (see module doc for the hydraulic vs.
    /// emergency-release thresholds this drives).
    pub uplock_jam: f64,
    /// Downlock spring/linkage fails to fully seat at full extension.
    pub downlock_fail: f64,
    /// Door actuator/track jam: reduces door speed, freezing it above
    /// `DOOR_JAM_FREEZE_THRESHOLD`.
    pub door_jam: f64,
    /// Proximity sensor(s) report the opposite of the true lock state above
    /// `SENSOR_LIE_THRESHOLD`.
    pub sensor_lies: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    Locked,
    DoorsOpening,
    Traveling,
    DoorsClosing,
}

#[derive(Clone, Copy, Debug)]
pub struct RetractionInputs {
    /// True = lever/handle commands gear down.
    pub gear_lever_down: bool,
    /// The alternate (free-fall) extension system has been selected.
    pub gravity_extend_commanded: bool,
    /// This leg's actuator hydraulic system pressure, as a fraction of
    /// nominal (0..1+).
    pub hydraulic_pressure_fraction: f64,
    pub dt_s: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct RetractionOutputs {
    /// True mechanical position, 0 (fully up) .. 1 (fully down).
    pub gear_position: f64,
    /// True door position, 0 (closed) .. 1 (open).
    pub door_position: f64,
    pub uplocked: bool,
    pub downlocked: bool,
    /// Possibly-lying proximity-sensor indications.
    pub sensed_uplocked: bool,
    pub sensed_downlocked: bool,
    pub phase: Phase,
    /// The leg is commanded to move but mechanically cannot even start
    /// (the uplock release failed): a genuine "gear won't extend" hazard,
    /// distinct from merely being slow.
    pub stuck_locked: bool,
}

pub struct Retraction {
    kind: LegKind,
    phase: Phase,
    /// Which end of travel the leg is currently sitting at (position, not
    /// lock truth -- see `step`'s doc comment on why these must be tracked
    /// separately: a failed downlock still reaches the down *position*).
    position_down: bool,
    target_down: bool,
    gravity_mode: bool,
    gear_position: f64,
    door_position: f64,
    uplocked: bool,
    downlocked: bool,
    stuck_locked: bool,
}

impl Retraction {
    pub fn new(kind: LegKind) -> Self {
        Self {
            kind,
            phase: Phase::Locked,
            position_down: true,
            target_down: true,
            gravity_mode: false,
            gear_position: 1.0,
            door_position: 0.0,
            uplocked: false,
            downlocked: true,
            stuck_locked: false,
        }
    }

    pub fn step(&mut self, inputs: &RetractionInputs, faults: &RetractionFaults) -> RetractionOutputs {
        let dt = inputs.dt_s.max(0.0);
        let commanded_down = inputs.gear_lever_down || inputs.gravity_extend_commanded;
        let scale = kind_rate_scale(self.kind);

        if self.phase == Phase::Locked {
            self.stuck_locked = false;
            // Driven by *position*, not lock truth: a leg that reached the
            // down position with a failed downlock (still `Phase::Locked`,
            // `downlocked == false`) must still be recognised as "at the
            // down end" so a later retract command is not silently ignored.
            if self.position_down != commanded_down {
                if !self.position_down && commanded_down {
                    // Currently up-locked, commanded to extend: the uplock
                    // must release first.
                    let can_release = if inputs.gravity_extend_commanded {
                        faults.uplock_jam < UPLOCK_JAM_DEFEATS_EMERGENCY
                    } else {
                        inputs.hydraulic_pressure_fraction >= MIN_RELEASE_PRESSURE_FRACTION && faults.uplock_jam < UPLOCK_JAM_DEFEATS_HYDRAULIC
                    };
                    if can_release {
                        self.uplocked = false;
                        self.target_down = true;
                        self.gravity_mode = inputs.gravity_extend_commanded;
                        self.phase = Phase::DoorsOpening;
                    } else {
                        self.stuck_locked = true;
                    }
                } else {
                    // Currently down-locked, commanded to retract: no
                    // release gate modelled (see module doc) -- the
                    // downlock is simply overcome by the retract actuator.
                    self.downlocked = false;
                    self.target_down = false;
                    self.gravity_mode = false;
                    self.phase = Phase::DoorsOpening;
                }
            }
        }

        // Door rate: frozen solid above the jam-freeze threshold, otherwise
        // proportionally slowed.
        let jam = faults.door_jam.clamp(0.0, 1.0);
        let door_rate = if jam >= DOOR_JAM_FREEZE_THRESHOLD { 0.0 } else { scale * (1.0 - jam / DOOR_JAM_FREEZE_THRESHOLD) / DOOR_NOMINAL_TRAVEL_S };

        match self.phase {
            Phase::Locked => {}
            Phase::DoorsOpening => {
                self.door_position = (self.door_position + door_rate * dt).min(1.0);
                if self.door_position >= DOOR_OPEN_THRESHOLD {
                    self.phase = Phase::Traveling;
                }
            }
            Phase::Traveling => {
                let leak_factor = (1.0 - faults.actuator_leak.clamp(0.0, 1.0)).max(0.0);
                let gear_rate = if self.gravity_mode {
                    scale / GRAVITY_EXTEND_TRAVEL_S
                } else {
                    scale * inputs.hydraulic_pressure_fraction.clamp(0.0, 2.0) * leak_factor / GEAR_NOMINAL_TRAVEL_S
                };
                if self.target_down {
                    self.gear_position = (self.gear_position + gear_rate * dt).min(1.0);
                    if self.gear_position >= GEAR_DOWN_THRESHOLD {
                        self.downlocked = faults.downlock_fail < DOWNLOCK_ENGAGE_FAULT_THRESHOLD;
                        self.position_down = true;
                        self.phase = Phase::DoorsClosing;
                    }
                } else {
                    self.gear_position = (self.gear_position - gear_rate * dt).max(0.0);
                    if self.gear_position <= GEAR_UP_THRESHOLD {
                        self.uplocked = true;
                        self.position_down = false;
                        self.phase = Phase::DoorsClosing;
                    }
                }
            }
            Phase::DoorsClosing => {
                self.door_position = (self.door_position - door_rate * dt).max(0.0);
                if self.door_position <= DOOR_CLOSED_THRESHOLD {
                    self.phase = Phase::Locked;
                }
            }
        }

        let sensed_invert = faults.sensor_lies >= SENSOR_LIE_THRESHOLD;
        RetractionOutputs {
            gear_position: self.gear_position,
            door_position: self.door_position,
            uplocked: self.uplocked,
            downlocked: self.downlocked,
            sensed_uplocked: self.uplocked != sensed_invert,
            sensed_downlocked: self.downlocked != sensed_invert,
            phase: self.phase,
            stuck_locked: self.stuck_locked,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn healthy() -> RetractionFaults {
        RetractionFaults::default()
    }

    fn run_to_locked(r: &mut Retraction, inputs: &RetractionInputs, faults: &RetractionFaults, max_ticks: u32) -> RetractionOutputs {
        let mut out = r.step(inputs, faults);
        for _ in 0..max_ticks {
            if out.phase == Phase::Locked {
                break;
            }
            out = r.step(inputs, faults);
        }
        out
    }

    #[test]
    fn starts_down_and_locked() {
        let mut r = Retraction::new(LegKind::Wing);
        let inputs = RetractionInputs { gear_lever_down: true, gravity_extend_commanded: false, hydraulic_pressure_fraction: 1.0, dt_s: 0.1 };
        let out = r.step(&inputs, &healthy());
        assert_eq!(out.phase, Phase::Locked);
        assert!(out.downlocked);
        assert!((out.gear_position - 1.0).abs() < 1e-9);
    }

    /// The FCOM's own figure: gravity extension takes approximately 70 s,
    /// lever to down-and-locked. That is the whole sequence, doors included,
    /// which is why `GRAVITY_EXTEND_TRAVEL_S` is not itself 70.
    #[test]
    fn gravity_extension_takes_the_seventy_seconds_the_fcom_gives() {
        // Start from up and locked.
        let mut r = Retraction::new(LegKind::Wing);
        let faults = healthy();
        let up = RetractionInputs { gear_lever_down: false, gravity_extend_commanded: false, hydraulic_pressure_fraction: 1.0, dt_s: 0.1 };
        let out = run_to_locked(&mut r, &up, &faults, 4_000);
        assert!(out.uplocked);

        // Free-fall it down with no hydraulics at all.
        let dt = 0.1;
        let drop = RetractionInputs { gear_lever_down: false, gravity_extend_commanded: true, hydraulic_pressure_fraction: 0.0, dt_s: dt };
        let mut seconds = 0.0;
        for _ in 0..4_000 {
            let out = r.step(&drop, &faults);
            seconds += dt;
            if out.phase == Phase::Locked && out.gear_position > GEAR_DOWN_THRESHOLD {
                break;
            }
        }
        assert!(
            (seconds - GRAVITY_EXTEND_TOTAL_S).abs() < 2.0,
            "gravity extension should take about {GRAVITY_EXTEND_TOTAL_S} s, took {seconds}"
        );
        // And it must be far slower than the powered cycle, which is the
        // whole point of the manoeuvre.
        assert!(seconds > 4.0 * (GEAR_NOMINAL_TRAVEL_S + DOOR_NOMINAL_TRAVEL_S));
    }

    #[test]
    fn a_full_retract_then_extend_cycle_locks_at_both_ends() {
        let mut r = Retraction::new(LegKind::Wing);
        let faults = healthy();
        let up_inputs = RetractionInputs { gear_lever_down: false, gravity_extend_commanded: false, hydraulic_pressure_fraction: 1.0, dt_s: 0.1 };
        let out = run_to_locked(&mut r, &up_inputs, &faults, 2_000);
        assert_eq!(out.phase, Phase::Locked);
        assert!(out.uplocked);
        assert!(out.gear_position < 0.05);
        assert!(out.door_position < 0.05, "doors must close again once up-locked");

        let down_inputs = RetractionInputs { gear_lever_down: true, gravity_extend_commanded: false, hydraulic_pressure_fraction: 1.0, dt_s: 0.1 };
        let out = run_to_locked(&mut r, &down_inputs, &faults, 2_000);
        assert_eq!(out.phase, Phase::Locked);
        assert!(out.downlocked);
        assert!(out.gear_position > 0.95);
    }

    #[test]
    fn a_jammed_uplock_prevents_ever_leaving_the_uplocked_state_hydraulically() {
        let mut r = Retraction::new(LegKind::Wing);
        let faults = RetractionFaults { uplock_jam: 0.8, ..Default::default() };
        // First get it up-locked with a healthy retraction.
        let up_inputs = RetractionInputs { gear_lever_down: false, gravity_extend_commanded: false, hydraulic_pressure_fraction: 1.0, dt_s: 0.1 };
        let out = run_to_locked(&mut r, &up_inputs, &RetractionFaults::default(), 2_000);
        assert!(out.uplocked);

        // Now the uplock has jammed: a normal (hydraulic) gear-down
        // selection cannot release it.
        let down_inputs = RetractionInputs { gear_lever_down: true, gravity_extend_commanded: false, hydraulic_pressure_fraction: 1.0, dt_s: 0.1 };
        let mut out = r.step(&down_inputs, &faults);
        for _ in 0..49 {
            out = r.step(&down_inputs, &faults);
        }
        assert!(out.stuck_locked, "a jammed uplock must prevent the normal release, not just slow it");
        assert_eq!(out.phase, Phase::Locked);
        assert!(out.gear_position < 0.05, "the gear must not have moved at all");

        // But gravity extension's separate mechanical release still works
        // at this jam severity.
        let gravity_inputs = RetractionInputs { gear_lever_down: true, gravity_extend_commanded: true, hydraulic_pressure_fraction: 0.0, dt_s: 0.1 };
        let out = run_to_locked(&mut r, &gravity_inputs, &faults, 5_000);
        assert!(out.downlocked, "gravity extension must still succeed through a moderate uplock jam");
        assert!(out.gear_position > 0.95);
    }

    #[test]
    fn a_severe_uplock_jam_defeats_even_gravity_extension() {
        let mut r = Retraction::new(LegKind::Nose);
        let faults = RetractionFaults { uplock_jam: 0.99, ..Default::default() };
        let up_inputs = RetractionInputs { gear_lever_down: false, gravity_extend_commanded: false, hydraulic_pressure_fraction: 1.0, dt_s: 0.1 };
        let out = run_to_locked(&mut r, &up_inputs, &RetractionFaults::default(), 2_000);
        assert!(out.uplocked);

        let gravity_inputs = RetractionInputs { gear_lever_down: true, gravity_extend_commanded: true, hydraulic_pressure_fraction: 0.0, dt_s: 0.1 };
        let mut out = r.step(&gravity_inputs, &faults);
        for _ in 0..50 {
            out = r.step(&gravity_inputs, &faults);
        }
        assert!(out.stuck_locked, "a severe enough jam must defeat gravity extension too");
    }

    #[test]
    fn a_failed_downlock_reaches_the_down_position_but_never_reports_locked() {
        let mut r = Retraction::new(LegKind::Body);
        let faults = RetractionFaults { downlock_fail: 0.9, ..Default::default() };
        let inputs = RetractionInputs { gear_lever_down: true, gravity_extend_commanded: false, hydraulic_pressure_fraction: 1.0, dt_s: 0.1 };
        // Force it up first with a healthy cycle, then extend with the fault.
        let up_inputs = RetractionInputs { gear_lever_down: false, ..inputs };
        run_to_locked(&mut r, &up_inputs, &RetractionFaults::default(), 2_000);
        let out = run_to_locked(&mut r, &inputs, &faults, 2_000);
        assert!(out.gear_position > 0.95, "the leg still reaches the down position geometrically");
        assert!(!out.downlocked, "but the downlock never truly engages");
    }

    #[test]
    fn a_lying_sensor_inverts_only_the_indication_not_the_truth() {
        let mut r = Retraction::new(LegKind::Wing);
        let faults = RetractionFaults { sensor_lies: 1.0, ..Default::default() };
        let inputs = RetractionInputs { gear_lever_down: true, gravity_extend_commanded: false, hydraulic_pressure_fraction: 1.0, dt_s: 0.1 };
        let out = r.step(&inputs, &faults);
        assert!(out.downlocked, "truth: still genuinely down-locked at start");
        assert!(!out.sensed_downlocked, "indication: lying sensor shows the opposite");
    }

    #[test]
    fn a_frozen_door_jam_prevents_the_gear_from_ever_starting_to_move() {
        let mut r = Retraction::new(LegKind::Wing);
        let faults = RetractionFaults { door_jam: 0.9, ..Default::default() };
        let up_inputs = RetractionInputs { gear_lever_down: false, gravity_extend_commanded: false, hydraulic_pressure_fraction: 1.0, dt_s: 0.1 };
        let mut out = r.step(&up_inputs, &faults);
        for _ in 0..500 {
            out = r.step(&up_inputs, &faults);
        }
        assert_eq!(out.phase, Phase::DoorsOpening, "a fully seized door must block the sequence indefinitely");
        assert!(out.gear_position > 0.95, "the gear itself must not have moved while the doors are stuck");
    }

    #[test]
    fn numerically_safe_at_rest_and_dt_zero() {
        let mut r = Retraction::new(LegKind::Nose);
        let inputs = RetractionInputs { gear_lever_down: true, gravity_extend_commanded: false, hydraulic_pressure_fraction: 1.0, dt_s: 0.0 };
        let out = r.step(&inputs, &healthy());
        assert!(out.gear_position.is_finite() && out.door_position.is_finite());
    }
}
