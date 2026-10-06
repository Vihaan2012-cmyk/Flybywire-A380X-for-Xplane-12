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
    pub starter_degradation: f64,
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
    pub duty_cycle_locked_out: bool,
    pub relight_permitted: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct Outputs {
    pub phase: StartPhase,
    pub starter_engaged: bool,
    pub starter_torque_nm: f64,
    pub starter_current_a: f64,
    pub battery_terminal_v: f64,
    pub fuel_and_ignition_on: bool,
}

impl Starter {
    pub fn new() -> Self {
        Self { phase: StartPhase::Idle }
    }

    pub fn phase(&self) -> StartPhase {
        self.phase
    }

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
        let omega = 300.0;
        let mut strong = Starter::new();
        let strong_out = strong.step(
            &Inputs { battery: BatteryInput::healthy(), ..inputs(0.0, true) },
            &StarterFaults::default(),
            omega,
        );
        let mut weak = Starter::new();
        let weak_out = weak.step(
            &Inputs {
                battery: BatteryInput { internal_resistance_ohm: 0.090, ..BatteryInput::healthy() },
                ..inputs(0.0, true)
            },
            &StarterFaults::default(),
            omega,
        );
        let voc = params::BATTERY_NOMINAL_OPEN_CIRCUIT_V;
        let strong_sag = voc - strong_out.battery_terminal_v;
        let weak_sag = voc - weak_out.battery_terminal_v;
        assert!(weak_sag > strong_sag, "weak {weak_sag} strong {strong_sag}");
        assert!((strong_sag - 8.64).abs() < 0.01, "{strong_sag}");
        assert!((weak_sag - 15.43).abs() < 0.01, "{weak_sag}");
        assert!(weak_out.starter_current_a < strong_out.starter_current_a);
        assert!(weak_out.starter_torque_nm < strong_out.starter_torque_nm);
    }

    #[test]
    fn a_flatter_battery_cranks_with_less_torque_at_the_same_speed() {
        let omega = 300.0;
        let mut full = Starter::new();
        let full_out = full.step(
            &Inputs {
                battery: BatteryInput { open_circuit_v: 24.0, ..BatteryInput::healthy() },
                ..inputs(0.0, true)
            },
            &StarterFaults::default(),
            omega,
        );
        let mut flat = Starter::new();
        let flat_out = flat.step(
            &Inputs {
                battery: BatteryInput { open_circuit_v: 18.0, ..BatteryInput::healthy() },
                ..inputs(0.0, true)
            },
            &StarterFaults::default(),
            omega,
        );
        assert!(flat_out.starter_current_a < full_out.starter_current_a);
        assert!(flat_out.starter_torque_nm < full_out.starter_torque_nm);
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
