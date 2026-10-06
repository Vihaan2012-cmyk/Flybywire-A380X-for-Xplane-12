use super::ecb::{self, Ecb};
use super::fire::{self, FireInterface};
use super::fuel_control::FuelControl;
use super::generators::Generators;
use super::governor::{self, Governor};
use super::inlet_door::InletDoor;
use super::interfaces::{BatteryInput, BleedOutput, GeneratorOutput};
use super::life::{CoreLife, StarterDutyCycle};
use super::load_compressor::{self, LoadCompressor};
use super::oil::OilSystem;
use super::params;
use super::power_section::{self, PowerSection, PowerSectionFaults};
use super::start_envelope::{self, FlightCondition};
use super::starter::{self, StartPhase, Starter};

#[derive(Clone, Copy, Debug)]
pub struct Inputs {
    pub dt_s: f64,
    pub ambient_pressure_pa: f64,
    pub ambient_temperature_k: f64,
    pub true_airspeed_mps: f64,
    pub master_on: bool,
    pub start_selected: bool,
    pub battery: BatteryInput,
    pub bleed_demand_kg_s: f64,
    pub gen1_used: bool,
    pub gen2_used: bool,
    pub gen1_electrical_load_w: f64,
    pub gen2_electrical_load_w: f64,
    pub fire_loop_detected: bool,
    pub fire_button_pushed: bool,
    pub fuel_shutoff_valve_powered: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Outputs {
    pub n_percent: f64,
    pub egt_indicated_c: f64,
    pub egt_true_c: f64,
    pub oil_pressure_psi: f64,
    pub oil_temperature_c: f64,
    pub bleed_output: BleedOutput,
    pub gen1_output: GeneratorOutput,
    pub gen2_output: GeneratorOutput,
    pub igv_position_frac: f64,
    pub scv_position_frac: f64,
    pub load_compressor_in_surge: bool,
    pub inlet_door_open_frac: f64,
    pub starter_current_a: f64,
    pub battery_terminal_v: f64,
    pub start_phase: StartPhase,
    pub fire_confirmed: bool,
    pub fire_bottle_pressure_frac: f64,
    pub overspeed_tripped: bool,
    pub egt_hard_tripped: bool,
    pub oil_low_pressure_tripped: bool,
    pub ecb_dual_channel_speed_loss: bool,
    pub ecb_any_trip: bool,
    pub relight_permitted: bool,
    pub operating_hours: f64,
    pub compressor_wear_frac: f64,
    pub turbine_wear_frac: f64,
    pub starter_duty_heat_frac: f64,
    pub available: bool,
}

pub struct Apu {
    power_section: PowerSection,
    load_compressor: LoadCompressor,
    governor: Governor,
    egt_limiter: governor::EgtLimiter,
    starter: Starter,
    generators: Generators,
    oil: OilSystem,
    fuel_control: FuelControl,
    inlet_door: InletDoor,
    fire: FireInterface,
    ecb: Ecb,
    core_life: CoreLife,
    starter_duty: StarterDutyCycle,
    last_oil_pressure_psi: f64,
}

impl Apu {
    pub fn new(ambient_temperature_k: f64) -> Self {
        let power_section = PowerSection::new(ambient_temperature_k);
        let max_fuel_flow = power_section.fuel_flow_design_kg_s() * params::MAX_FUEL_FLOW_MARGIN;
        Self {
            power_section,
            load_compressor: LoadCompressor::new(),
            governor: Governor::new(max_fuel_flow),
            egt_limiter: governor::EgtLimiter::new(max_fuel_flow),
            starter: Starter::new(),
            generators: Generators::new(),
            oil: OilSystem::new(ambient_temperature_k),
            fuel_control: FuelControl::new(max_fuel_flow),
            inlet_door: InletDoor::new(),
            fire: FireInterface::new(),
            ecb: Ecb::new(),
            core_life: CoreLife::new(),
            starter_duty: StarterDutyCycle::new(),
            last_oil_pressure_psi: 0.0,
        }
    }

    pub fn n_percent(&self) -> f64 {
        self.power_section.n_percent()
    }

    const MAX_CONTROL_SUBSTEP_S: f64 = 0.05;

    pub fn step(&mut self, inputs: &Inputs, faults: &super::faults::ApuFaults) -> Outputs {
        let total_dt = inputs.dt_s.max(0.0);
        let substeps = (total_dt / Self::MAX_CONTROL_SUBSTEP_S).ceil().max(1.0) as u32;
        let mut sub = *inputs;
        sub.dt_s = total_dt / substeps as f64;
        let mut out = Outputs::default();
        for _ in 0..substeps {
            out = self.step_once(&sub, faults);
        }
        out
    }

    fn step_once(&mut self, inputs: &Inputs, faults: &super::faults::ApuFaults) -> Outputs {
        let dt = inputs.dt_s.max(0.0);
        let n_percent = self.power_section.n_percent();
        let omega = self.power_section.omega_rad_s();

        let fc = FlightCondition {
            ambient_pressure_pa: inputs.ambient_pressure_pa,
            ambient_temperature_k: inputs.ambient_temperature_k,
            true_airspeed_mps: inputs.true_airspeed_mps,
        };
        let (inlet_t_k, inlet_p_pa) = fc.inlet_total_conditions();
        let relight_permitted = start_envelope::relight_permitted(&fc);
        let windmill_torque_nm = start_envelope::windmill_torque_nm(&fc, (n_percent / 100.0).max(0.0));

        let door_open_frac = self.inlet_door.step(inputs.master_on, &faults.inlet_door, dt);
        let inlet_loss = InletDoor::pressure_loss_frac(door_open_frac);
        let door_ready = self.inlet_door.is_fully_open();

        let duty_locked_out = self.starter_duty.locked_out();
        let phase_before = self.starter.phase();
        let starter_out = self.starter.step(
            &starter::Inputs {
                n_percent,
                master_on: inputs.master_on,
                start_selected: inputs.start_selected && door_ready,
                battery: inputs.battery,
                duty_cycle_locked_out: duty_locked_out,
                relight_permitted,
            },
            &faults.starter,
            omega,
        );
        if phase_before == StartPhase::Idle && starter_out.phase == StartPhase::Cranking {
            self.core_life.record_start();
        }
        self.starter_duty.step(starter_out.starter_engaged, faults.starter_duty_model_fault, dt);

        let fire_out = self.fire.step(
            &fire::Inputs {
                fire_loop_detected: inputs.fire_loop_detected,
                fire_button_pushed: inputs.fire_button_pushed,
                dt_s: dt,
            },
            &faults.fire,
        );

        let ecb_run_input =
            starter_out.fuel_and_ignition_on && !fire_out.fuel_shutoff_commanded && inputs.fuel_shutoff_valve_powered;
        let ecb_out = self.ecb.step(
            &ecb::TrueSignals {
                n_percent,
                egt_true_c: self.power_section.egt_c(),
                oil_pressure_psi: self.last_oil_pressure_psi,
                fire_confirmed: fire_out.fire_confirmed,
                start_selected: inputs.start_selected,
                inlet_door_ready: door_ready,
            },
            ecb_run_input,
            dt,
            &faults.ecb,
        );

        let running = ecb_run_input && !ecb_out.any_trip;
        let solenoid_open = running;
        let n_for_governor = ecb_out.n_for_governor.unwrap_or(n_percent);

        let egt_limit_c = governor::egt_limit_c(n_for_governor);
        let egt_limit_fuel = governor::egt_limit_fuel_flow_kg_s(
            self.power_section.calibration(),
            n_for_governor,
            inlet_t_k,
            inlet_p_pa * (1.0 - inlet_loss.clamp(0.0, 0.5)),
            egt_limit_c,
        );
        let measured_egt_c = ecb_out.egt_indicated_c.unwrap_or(self.power_section.egt_c());
        let measured_ceiling = self.egt_limiter.step(measured_egt_c, egt_limit_c, running, dt);
        let commanded_fuel =
            self.governor.step(n_for_governor, running, egt_limit_fuel.min(measured_ceiling), dt);
        let actual_fuel = self.fuel_control.step(commanded_fuel, solenoid_open, &faults.fuel_control, dt);

        let gen1_load = if inputs.gen1_used { inputs.gen1_electrical_load_w } else { 0.0 };
        let gen2_load = if inputs.gen2_used { inputs.gen2_electrical_load_w } else { 0.0 };
        let total_rated = self.generators.gen1.rated_real_power_w() + self.generators.gen2.rated_real_power_w();
        let electrical_load_frac = if total_rated > 1e-6 { (gen1_load + gen2_load) / total_rated } else { 0.0 };

        let bleed_demand = if fire_out.bleed_valve_close_commanded { 0.0 } else { inputs.bleed_demand_kg_s };
        let load_out = self.load_compressor.step(
            &load_compressor::Inputs {
                n_frac: n_percent / 100.0,
                ambient_pressure_pa: inputs.ambient_pressure_pa,
                ambient_temperature_k: inputs.ambient_temperature_k,
                bleed_demand_kg_s: bleed_demand,
                electrical_load_frac,
                dt_s: dt,
            },
            &faults.load_compressor,
        );

        let gens_out = self.generators.step(gen1_load, gen2_load, &faults.gen1, &faults.gen2);
        let (gen1_shaft_w, gen1_overloaded) = self.generators.gen1.shaft_power_w(gen1_load, &faults.gen1);
        let (gen2_shaft_w, gen2_overloaded) = self.generators.gen2.shaft_power_w(gen2_load, &faults.gen2);

        let cold_drag_nm = self.oil.cold_drag_torque_nm(omega);
        let fixed_accessory_torque_nm =
            params::FIXED_ACCESSORY_POWER_W / self.power_section.omega_rated_rad_s().max(1.0);
        let load_compressor_torque_nm = if omega > 1.0 { load_out.shaft_power_w / omega } else { 0.0 };
        let generator_torque_nm = if omega > 1.0 { gens_out.total_shaft_power_w / omega } else { 0.0 };
        let accessory_torque_nm =
            load_compressor_torque_nm + generator_torque_nm + fixed_accessory_torque_nm + cold_drag_nm;
        let windmill_torque_nm = windmill_torque_nm * door_open_frac.clamp(0.0, 1.0);
        let driving_torque_nm = starter_out.starter_torque_nm + windmill_torque_nm;

        let compressor_efficiency_loss =
            faults.power_section.compressor_efficiency_loss.max(self.core_life.compressor_wear_frac());
        let turbine_efficiency_loss =
            faults.power_section.turbine_efficiency_loss.max(self.core_life.turbine_wear_frac());

        let ps_out = self.power_section.step(
            &power_section::Inputs {
                ambient_pressure_pa: inlet_p_pa,
                ambient_temperature_k: inlet_t_k,
                inlet_pressure_loss_frac: inlet_loss,
                fuel_flow_kg_s: actual_fuel,
                starter_torque_nm: driving_torque_nm,
                accessory_torque_nm,
                dt_s: dt,
            },
            &PowerSectionFaults { compressor_efficiency_loss, turbine_efficiency_loss },
        );
        self.core_life.accumulate(running, ps_out.egt_c, dt);

        let friction_heat_w = (fixed_accessory_torque_nm + cold_drag_nm).max(0.0) * omega.max(0.0);
        let oil_out = self.oil.step(
            n_percent,
            running,
            inputs.ambient_temperature_k,
            friction_heat_w,
            &faults.oil,
            dt,
        );
        self.last_oil_pressure_psi = oil_out.pressure_psi;

        let egt_true_c = ps_out.egt_c;
        let egt_indicated_c = ecb_out.egt_indicated_c.unwrap_or(egt_true_c);

        Outputs {
            n_percent: ps_out.n_percent,
            egt_indicated_c,
            egt_true_c,
            oil_pressure_psi: oil_out.pressure_psi,
            oil_temperature_c: oil_out.temp_c,
            bleed_output: BleedOutput {
                mass_flow_kg_s: load_out.delivered_bleed_kg_s,
                pressure_pa: load_out.delivered_pressure_pa,
                temperature_k: load_out.delivered_temperature_k,
            },
            gen1_output: GeneratorOutput { real_power_w: gen1_load, shaft_power_w: gen1_shaft_w, overloaded: gen1_overloaded },
            gen2_output: GeneratorOutput { real_power_w: gen2_load, shaft_power_w: gen2_shaft_w, overloaded: gen2_overloaded },
            igv_position_frac: load_out.igv_position_frac,
            scv_position_frac: load_out.scv_position_frac,
            load_compressor_in_surge: load_out.in_surge,
            inlet_door_open_frac: door_open_frac,
            starter_current_a: starter_out.starter_current_a,
            battery_terminal_v: starter_out.battery_terminal_v,
            start_phase: starter_out.phase,
            fire_confirmed: fire_out.fire_confirmed,
            fire_bottle_pressure_frac: fire_out.bottle_pressure_frac,
            overspeed_tripped: self.power_section.overspeed_tripped(),
            egt_hard_tripped: self.power_section.egt_over_hard_trip(),
            oil_low_pressure_tripped: oil_out.low_pressure_tripped,
            ecb_dual_channel_speed_loss: ecb_out.dual_channel_speed_loss,
            ecb_any_trip: ecb_out.any_trip,
            relight_permitted,
            operating_hours: self.core_life.operating_hours,
            compressor_wear_frac: self.core_life.compressor_wear_frac(),
            turbine_wear_frac: self.core_life.turbine_wear_frac(),
            starter_duty_heat_frac: self.starter_duty.heat_fraction(),
            available: matches!(starter_out.phase, StartPhase::SelfSustaining)
                && ps_out.n_percent > params::GOVERNED_N_PERCENT - 5.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::ecb::{ChannelFaults, EcbFaults, SensorFault};
    use super::super::faults::ApuFaults;

    fn base_inputs() -> Inputs {
        Inputs {
            dt_s: 0.5,
            ambient_pressure_pa: 101_325.0,
            ambient_temperature_k: 288.15,
            true_airspeed_mps: 0.0,
            master_on: true,
            start_selected: false,
            battery: BatteryInput::healthy(),
            bleed_demand_kg_s: 0.0,
            gen1_used: false,
            gen2_used: false,
            gen1_electrical_load_w: 0.0,
            gen2_electrical_load_w: 0.0,
            fire_loop_detected: false,
            fire_button_pushed: false,
            fuel_shutoff_valve_powered: true,
        }
    }

    #[test]
    fn a_full_healthy_start_reaches_available_with_no_nan_anywhere() {
        let mut apu = Apu::new(288.15);
        let faults = ApuFaults::default();
        let mut out = Outputs::default();

        for _ in 0..200 {
            out = apu.step(&base_inputs(), &faults);
        }

        let mut inputs = base_inputs();
        inputs.start_selected = true;
        const MAX_TICKS: u32 = 4000;
        let mut ticks = 0;
        while !out.available && ticks < MAX_TICKS {
            out = apu.step(&inputs, &faults);
            assert!(out.n_percent.is_finite() && out.egt_true_c.is_finite());
            assert!(!out.n_percent.is_nan() && !out.egt_indicated_c.is_nan());
            ticks += 1;
        }
        assert!(out.available, "did not reach available within {MAX_TICKS} ticks (n={:.1}%)", out.n_percent);
        assert!(!out.overspeed_tripped && !out.egt_hard_tripped);
        assert!(out.operating_hours >= 0.0);
    }

    #[test]
    fn no_battery_never_starts() {
        let mut apu = Apu::new(288.15);
        let mut inputs = base_inputs();
        inputs.battery = BatteryInput { available: false, ..BatteryInput::healthy() };
        let mut out = Outputs::default();
        for _ in 0..200 {
            out = apu.step(&inputs, &ApuFaults::default());
        }
        inputs.start_selected = true;
        for _ in 0..500 {
            out = apu.step(&inputs, &ApuFaults::default());
        }
        assert!(out.n_percent < 1.0, "{}", out.n_percent);
        assert!(!out.available);
    }

    #[test]
    fn above_the_relight_ceiling_it_cranks_but_never_becomes_available() {
        let mut apu = Apu::new(218.8);
        let mut inputs = base_inputs();
        inputs.ambient_pressure_pa = 23_800.0;
        inputs.ambient_temperature_k = 218.8;
        for _ in 0..200 {
            apu.step(&inputs, &ApuFaults::default());
        }
        inputs.start_selected = true;
        let mut out = Outputs::default();
        for _ in 0..500 {
            out = apu.step(&inputs, &ApuFaults::default());
        }
        assert!(!out.relight_permitted);
        assert!(!out.available);
    }

    #[test]
    fn a_healthy_apu_stays_at_zero_n_in_cruise_with_the_master_switch_off() {
        let mut apu = Apu::new(218.8);
        let mut inputs = base_inputs();
        inputs.true_airspeed_mps = 230.0;
        inputs.ambient_pressure_pa = 23_800.0;
        inputs.ambient_temperature_k = 218.8;
        inputs.master_on = false;
        inputs.start_selected = false;
        let mut out = Outputs::default();
        for _ in 0..3600 {
            out = apu.step(&inputs, &ApuFaults::default());
            assert_eq!(out.n_percent, 0.0, "N moved off zero at tick with the master switch off and the inlet door never commanded open");
        }
        assert_eq!(out.n_percent, 0.0);
        assert_eq!(out.inlet_door_open_frac, 0.0, "the door was never commanded open either");
    }

    #[test]
    fn a_fire_confirmed_mid_run_commands_shutoff_and_the_bleed_valve_closes() {
        let mut apu = Apu::new(288.15);
        let faults = ApuFaults::default();
        let mut inputs = base_inputs();
        for _ in 0..200 {
            apu.step(&inputs, &faults);
        }
        inputs.start_selected = true;
        inputs.bleed_demand_kg_s = 1.0;
        let mut out = Outputs::default();
        for _ in 0..4000 {
            out = apu.step(&inputs, &faults);
            if out.available {
                break;
            }
        }
        assert!(out.available);

        inputs.fire_loop_detected = true;
        inputs.fire_button_pushed = true;
        for _ in 0..10 {
            out = apu.step(&inputs, &faults);
        }
        assert!(out.fire_confirmed);
        assert_eq!(out.bleed_output.mass_flow_kg_s, 0.0);
        assert!(out.fire_bottle_pressure_frac < 1.0);
    }

    #[test]
    fn a_generator_carrying_a_real_load_past_its_rating_is_overloaded() {
        let mut apu = Apu::new(288.15);
        let mut inputs = base_inputs();
        inputs.gen1_used = true;
        inputs.gen1_electrical_load_w = 3.0 * (params::GENERATOR_RATED_APPARENT_VA * params::GENERATOR_RATED_POWER_FACTOR);
        let out = apu.step(&inputs, &ApuFaults::default());
        assert!(out.gen1_output.overloaded, "a generator asked for 3x its rated power must report overloaded");
        assert!(!out.gen2_output.overloaded, "generator 2 was never used and must stay healthy");

        let mut apu2 = Apu::new(288.15);
        let mut light = base_inputs();
        light.gen1_used = true;
        light.gen1_electrical_load_w = 0.2 * (params::GENERATOR_RATED_APPARENT_VA * params::GENERATOR_RATED_POWER_FACTOR);
        let out2 = apu2.step(&light, &ApuFaults::default());
        assert!(!out2.gen1_output.overloaded);
    }

    #[test]
    fn a_dual_channel_speed_sensor_loss_is_a_protective_condition_not_a_silent_freeze() {
        let mut apu = Apu::new(288.15);
        let faults = ApuFaults {
            ecb: EcbFaults {
                channel_a: ChannelFaults {
                    speed_sensor: SensorFault { bias: 0.0, failed: true },
                    ..Default::default()
                },
                channel_b: ChannelFaults {
                    speed_sensor: SensorFault { bias: 0.0, failed: true },
                    ..Default::default()
                },
            },
            ..ApuFaults::default()
        };
        let mut inputs = base_inputs();
        inputs.start_selected = true;
        let mut out = Outputs::default();
        for _ in 0..300 {
            out = apu.step(&inputs, &faults);
        }
        assert!(out.ecb_dual_channel_speed_loss);
        assert!(out.ecb_any_trip);
        assert!(!out.available, "must not silently keep running with no speed signal at all");
    }

    fn started(apu: &mut Apu, inputs: &Inputs, faults: &ApuFaults) -> Outputs {
        let mut out = Outputs::default();
        let mut idle = *inputs;
        idle.start_selected = false;
        for _ in 0..200 {
            out = apu.step(&idle, faults);
        }
        for _ in 0..4000 {
            out = apu.step(inputs, faults);
            if out.available {
                break;
            }
        }
        assert!(out.available, "did not start (n={:.1}%)", out.n_percent);
        out
    }

    #[test]
    fn a_healthy_apu_at_design_load_runs_an_hour_without_a_protective_trip() {
        for ambient_k in [288.15, 318.15] {
            let mut apu = Apu::new(ambient_k);
            let faults = ApuFaults::default();
            let mut inputs = base_inputs();
            inputs.ambient_temperature_k = ambient_k;
            inputs.start_selected = true;
            started(&mut apu, &inputs, &faults);
            inputs.bleed_demand_kg_s = params::LOAD_MDOT_DESIGN_KG_S;
            inputs.gen1_used = true;
            inputs.gen2_used = true;
            let rated_w = params::GENERATOR_RATED_APPARENT_VA * params::GENERATOR_RATED_POWER_FACTOR;
            inputs.gen1_electrical_load_w = rated_w;
            inputs.gen2_electrical_load_w = rated_w;
            for tick in 0..(3600.0 / inputs.dt_s) as usize {
                let out = apu.step(&inputs, &faults);
                assert!(
                    !out.ecb_any_trip && out.available,
                    "{ambient_k} K, t={:.0} s: tripped (oil {}, overspeed {}, EGT {}) at N {:.1}%, EGT {:.0} C, oil {:.1} psi",
                    tick as f64 * inputs.dt_s,
                    out.oil_low_pressure_tripped,
                    out.overspeed_tripped,
                    out.egt_hard_tripped,
                    out.n_percent,
                    out.egt_true_c,
                    out.oil_pressure_psi
                );
            }
        }
    }

    #[test]
    fn an_oil_leak_trips_the_running_apu_on_low_oil_pressure() {
        let mut apu = Apu::new(288.15);
        let mut inputs = base_inputs();
        inputs.start_selected = true;
        started(&mut apu, &inputs, &ApuFaults::default());
        let leaking = ApuFaults { oil: super::super::oil::OilFaults { leak: 1.0 }, ..ApuFaults::default() };
        let mut tripped_at = None;
        for tick in 0..(3600.0 / inputs.dt_s) as usize {
            let out = apu.step(&inputs, &leaking);
            if out.ecb_any_trip {
                assert!(
                    !out.overspeed_tripped && !out.egt_hard_tripped && out.oil_pressure_psi < params::OIL_PRESSURE_TRIP_PSI,
                    "it must be the oil trip: overspeed {}, EGT {}, oil {:.1} psi",
                    out.overspeed_tripped,
                    out.egt_hard_tripped,
                    out.oil_pressure_psi
                );
                tripped_at = Some(tick as f64 * inputs.dt_s);
                break;
            }
        }
        assert!(tripped_at.is_some(), "a full leak never tripped the APU within an hour");
    }
}
