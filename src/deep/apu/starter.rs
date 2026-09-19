//! The APU start sequence and starter motor circuit -- item 3's "start
//! sequence with starter motor from batteries (battery voltage affects
//! start), light-off".
//!
//! **Starter motor**: a series-wound DC motor fed from the aircraft's main
//! battery (a real APU start is commonly battery-only, which is why a weak
//! or depleted battery visibly slows a real start). The battery itself is
//! `interfaces::BatteryInput` -- a plain Thevenin-equivalent source (open-
//! circuit voltage and internal resistance) that `deep::electrical`'s own
//! battery model is meant to supply (this file does not model *why* a
//! battery is weak or cold-soaked, only reacts to whatever V/R it is
//! handed), so its *terminal* voltage sags under the starter's own inrush
//! current -- a real, measurable effect of any loaded aircraft battery,
//! emerging from solving the series circuit rather than being scripted:
//!
//!   `V_bus = V_open_circuit - I * R_battery`
//!   `I = (V_bus - Ke*omega) / R_armature` (series motor back-EMF)
//!   `torque = Kt * I` (`Kt = Ke` in SI units)
//!
//! Substituting the first into the second and solving for `I` avoids an
//! inner iteration: `I = (V_open_circuit - Ke*omega) / (R_battery +
//! R_armature)`.
//!
//! **Start sequence**: crank (starter only) -> light-off (fuel/ignition
//! introduced once cranking speed clears an effective light-off threshold,
//! and only within the certified relight envelope,
//! `start_envelope::relight_permitted`) -> self-sustaining (the starter is
//! no longer needed and disengages) -> governed. Every transition is driven
//! by the spool's own measured speed, never a timer, so a weak battery/
//! starter, a failed igniter, a duty-cycle lockout
//! (`life::StarterDutyCycle`) or an out-of-envelope relight attempt
//! genuinely hangs the start (the core sits cranking, never reaching
//! light-off) rather than the sequence being fast-forwarded through
//! regardless.

use super::interfaces::BatteryInput;
use super::params;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StartPhase {
    Idle,
    Cranking,
    Accelerating,
    SelfSustaining,
}

impl Default for StartPhase {
    fn default() -> Self {
        StartPhase::Idle
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct StarterFaults {
    /// Starter motor wear/failure: weakens the shared back-EMF/torque
    /// constant (0 healthy .. 1 -- a badly worn brushed motor still turns,
    /// just with more current for less torque, the real symptom of brush/
    /// commutator wear, not simply "no torque at all").
    pub starter_degradation: f64,
    /// Igniter failure: 0 healthy .. 1. Raises the effective speed the core
    /// must reach before light-off can occur; at 1.0 that threshold is
    /// pushed to (and past) the self-sustaining speed, which the starter
    /// alone cannot reach -- a hung start with no fuel ever lit, the real
    /// consequence of a fully failed igniter, falling out of the same
    /// threshold logic rather than a separate scripted "hung start" branch.
    pub igniter_failure: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct Starter {
    phase: StartPhase,
}

pub struct Inputs {
    pub n_percent: f64,
    pub master_on: bool,
    pub start_selected: bool,
    pub battery: BatteryInput,
    /// The starter motor's own duty-cycle protection has locked it out
    /// pending cool-down (`life::StarterDutyCycle`) -- refuses to engage
    /// even with everything else ready, same as a real starter's thermal
    /// cutout.
    pub duty_cycle_locked_out: bool,
    /// Whether the current altitude/temperature puts this attempt within
    /// the certified relight envelope (`start_envelope::relight_permitted`).
    pub relight_permitted: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct Outputs {
    pub phase: StartPhase,
    pub starter_engaged: bool,
    pub starter_torque_nm: f64,
    pub starter_current_a: f64,
    pub battery_terminal_v: f64,
    /// Fuel valve may open and ignition is active this tick (the governor's
    /// `running` input, `governor.rs`).
    pub fuel_and_ignition_on: bool,
}

impl Starter {
    pub fn new() -> Self {
        Self { phase: StartPhase::Idle }
    }

    pub fn phase(&self) -> StartPhase {
        self.phase
    }

    /// `omega_rad_s` is the gas-generator spool's actual angular velocity
    /// this tick (`power_section::PowerSection`, previous tick's value --
    /// same one-tick-lag pattern the rest of this directory uses).
    pub fn step(&mut self, inputs: &Inputs, faults: &StarterFaults, omega_rad_s: f64) -> Outputs {
        let effective_light_off = (params::LIGHT_OFF_N_PERCENT
            + faults.igniter_failure.clamp(0.0, 1.0)
                * (params::SELF_SUSTAINING_N_PERCENT - params::LIGHT_OFF_N_PERCENT + 5.0))
            .min(200.0);

        self.phase = if !inputs.master_on {
            StartPhase::Idle
        } else {
            match self.phase {
                StartPhase::Idle if inputs.start_selected => StartPhase::Cranking,
                StartPhase::Cranking
                    if inputs.n_percent >= effective_light_off && inputs.relight_permitted =>
                {
                    StartPhase::Accelerating
                }
                StartPhase::Accelerating if inputs.n_percent >= params::SELF_SUSTAINING_N_PERCENT => {
                    StartPhase::SelfSustaining
                }
                other => other,
            }
        };

        let starter_wants_to_engage = inputs.master_on
            && matches!(self.phase, StartPhase::Cranking | StartPhase::Accelerating)
            && inputs.n_percent < params::SELF_SUSTAINING_N_PERCENT
            && !inputs.duty_cycle_locked_out;
        let starter_engaged = starter_wants_to_engage && inputs.battery.available;

        let ke_kt = (params::STARTER_KE_KT * (1.0 - 0.5 * faults.starter_degradation.clamp(0.0, 1.0)))
            .max(0.2 * params::STARTER_KE_KT);

        let (current, terminal_v, torque) = if starter_engaged {
            let v_oc = inputs.battery.open_circuit_v.max(0.0);
            let r_battery = inputs.battery.internal_resistance_ohm.max(0.0);
            let back_emf = ke_kt * omega_rad_s.max(0.0);
            let total_resistance = r_battery + params::STARTER_ARMATURE_RESISTANCE_OHM;
            let i = ((v_oc - back_emf) / total_resistance).max(0.0);
            let v_term = (v_oc - i * r_battery).max(0.0);
            (i, v_term, ke_kt * i)
        } else {
            (0.0, inputs.battery.open_circuit_v.max(0.0), 0.0)
        };

        let fuel_and_ignition_on =
            inputs.master_on && matches!(self.phase, StartPhase::Accelerating | StartPhase::SelfSustaining);

        Outputs {
            phase: self.phase,
            starter_engaged,
            starter_torque_nm: torque,
            starter_current_a: current,
            battery_terminal_v: terminal_v,
            fuel_and_ignition_on,
        }
    }
}

impl Default for Starter {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn inputs(n_percent: f64, start_selected: bool) -> Inputs {
        Inputs {
            n_percent,
            master_on: true,
            start_selected,
            battery: BatteryInput::healthy(),
            duty_cycle_locked_out: false,
            relight_permitted: true,
        }
    }

    #[test]
    fn master_off_stays_idle_and_never_engages() {
        let mut s = Starter::new();
        let out = s.step(
            &Inputs { n_percent: 0.0, master_on: false, ..inputs(0.0, true) },
            &StarterFaults::default(),
            0.0,
        );
        assert_eq!(out.phase, StartPhase::Idle);
        assert!(!out.starter_engaged);
    }

    #[test]
    fn selecting_start_engages_the_starter_and_draws_current_at_rest() {
        let mut s = Starter::new();
        let out = s.step(&inputs(0.0, true), &StarterFaults::default(), 0.0);
        assert_eq!(out.phase, StartPhase::Cranking);
        assert!(out.starter_engaged);
        assert!(out.starter_current_a > 0.0);
        assert!(out.starter_torque_nm > 0.0);
        assert!(!out.fuel_and_ignition_on);
    }

    #[test]
    fn current_falls_and_battery_recovers_as_speed_rises_toward_the_starter_alone_limit() {
        let mut low = Starter::new();
        let low_out = low.step(&inputs(5.0, true), &StarterFaults::default(), 300.0);
        let mut high = Starter::new();
        let high_out = high.step(&inputs(5.0, true), &StarterFaults::default(), 2000.0);
        assert!(high_out.starter_current_a < low_out.starter_current_a);
        assert!(high_out.battery_terminal_v > low_out.battery_terminal_v);
    }

    #[test]
    fn a_weaker_battery_sags_more_under_the_same_starter_load() {
        let mut strong = Starter::new();
        let strong_out = strong.step(
            &Inputs {
                battery: BatteryInput { open_circuit_v: 24.0, ..BatteryInput::healthy() },
                ..inputs(0.0, true)
            },
            &StarterFaults::default(),
            300.0,
        );
        let mut weak = Starter::new();
        let weak_out = weak.step(
            &Inputs {
                battery: BatteryInput { open_circuit_v: 18.0, ..BatteryInput::healthy() },
                ..inputs(0.0, true)
            },
            &StarterFaults::default(),
            300.0,
        );
        let strong_sag = 24.0 - strong_out.battery_terminal_v;
        let weak_sag = 18.0 - weak_out.battery_terminal_v;
        // Lower open-circuit voltage means less back-EMF headroom, so more
        // current flows for the same speed, and that larger current sags
        // the (already weaker) battery even further.
        assert!(weak_out.starter_current_a > strong_out.starter_current_a);
        assert!(weak_sag > strong_sag);
    }

    #[test]
    fn starter_degradation_draws_more_current_for_less_torque_at_the_same_speed() {
        let omega = 300.0;
        let mut healthy = Starter::new();
        let healthy_out = healthy.step(&inputs(5.0, true), &StarterFaults::default(), omega);
        let mut degraded = Starter::new();
        let degraded_out = degraded.step(
            &inputs(5.0, true),
            &StarterFaults { starter_degradation: 0.7, igniter_failure: 0.0 },
            omega,
        );
        assert!(degraded_out.starter_current_a > healthy_out.starter_current_a);
        assert!(degraded_out.starter_torque_nm < healthy_out.starter_torque_nm);
    }

    #[test]
    fn a_fully_failed_igniter_never_reaches_fuel_and_ignition_however_fast_it_cranks() {
        let mut s = Starter::new();
        s.step(&inputs(0.0, true), &StarterFaults { igniter_failure: 1.0, ..Default::default() }, 0.0);
        // Cranked all the way to (just under) self-sustaining speed: still
        // never got fuel/ignition, because the effective light-off
        // threshold with a fully failed igniter sits at/above that speed.
        let out = s.step(
            &inputs(params::SELF_SUSTAINING_N_PERCENT - 0.5, true),
            &StarterFaults { igniter_failure: 1.0, ..Default::default() },
            2000.0,
        );
        assert!(!out.fuel_and_ignition_on);
        assert_eq!(out.phase, StartPhase::Cranking);
    }

    #[test]
    fn reaching_light_off_speed_moves_to_accelerating_with_fuel_and_ignition_on() {
        let mut s = Starter::new();
        s.step(&inputs(0.0, true), &StarterFaults::default(), 0.0);
        let out = s.step(&inputs(params::LIGHT_OFF_N_PERCENT + 1.0, true), &StarterFaults::default(), 1500.0);
        assert_eq!(out.phase, StartPhase::Accelerating);
        assert!(out.fuel_and_ignition_on);
        assert!(out.starter_engaged, "still assisting below self-sustaining speed");
    }

    #[test]
    fn reaching_self_sustaining_speed_disengages_the_starter() {
        let mut s = Starter::new();
        s.step(&inputs(0.0, true), &StarterFaults::default(), 0.0);
        s.step(&inputs(params::LIGHT_OFF_N_PERCENT + 1.0, true), &StarterFaults::default(), 1500.0);
        let out = s.step(
            &inputs(params::SELF_SUSTAINING_N_PERCENT + 1.0, true),
            &StarterFaults::default(),
            5000.0,
        );
        assert_eq!(out.phase, StartPhase::SelfSustaining);
        assert!(!out.starter_engaged);
        assert_eq!(out.starter_torque_nm, 0.0);
    }

    #[test]
    fn no_battery_available_never_engages_even_with_start_selected() {
        let mut s = Starter::new();
        let out = s.step(
            &Inputs {
                battery: BatteryInput { available: false, ..BatteryInput::healthy() },
                ..inputs(0.0, true)
            },
            &StarterFaults::default(),
            0.0,
        );
        assert!(!out.starter_engaged);
        assert_eq!(out.starter_torque_nm, 0.0);
    }

    #[test]
    fn a_duty_cycle_lockout_refuses_to_engage_even_with_everything_else_ready() {
        let mut s = Starter::new();
        let out = s.step(
            &Inputs { duty_cycle_locked_out: true, ..inputs(0.0, true) },
            &StarterFaults::default(),
            0.0,
        );
        assert!(!out.starter_engaged);
        assert_eq!(out.starter_torque_nm, 0.0);
    }

    #[test]
    fn outside_the_relight_envelope_it_cranks_forever_but_never_lights() {
        let mut s = Starter::new();
        s.step(&Inputs { relight_permitted: false, ..inputs(0.0, true) }, &StarterFaults::default(), 0.0);
        let out = s.step(
            &Inputs { relight_permitted: false, ..inputs(params::LIGHT_OFF_N_PERCENT + 5.0, true) },
            &StarterFaults::default(),
            1500.0,
        );
        assert_eq!(out.phase, StartPhase::Cranking);
        assert!(!out.fuel_and_ignition_on);
        assert!(out.starter_engaged, "still cranking, just never lighting");
    }
}
