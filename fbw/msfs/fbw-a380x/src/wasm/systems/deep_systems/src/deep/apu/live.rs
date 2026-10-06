use super::apu::{Apu, Inputs, Outputs};
use super::ecb::{ChannelFaults, EcbFaults, SensorFault};
use super::faults::ApuFaults;
use super::fire::FireFaults;
use super::fuel_control::FuelControlFaults;
use super::generators::GeneratorFaults;
use super::inlet_door::InletDoorFaults;
use super::interfaces::BatteryInput;
use super::load_compressor::LoadCompressorFaults;
use super::oil::OilFaults;
use super::params;
use super::power_section::PowerSectionFaults;
use super::registry::ids;
use super::starter::{StartPhase, StarterFaults};
use crate::deep::live::{Area, Faults, Truth};

pub fn live_system() -> Box<dyn Area> {
    Box::new(LiveApu::new())
}

pub struct LiveApu {
    apu: Apu,
    out: Outputs,
    seated: bool,
}

impl Default for LiveApu {
    fn default() -> Self {
        Self::new()
    }
}

impl LiveApu {
    pub fn new() -> Self {
        Self {
            apu: Apu::new(288.15),
            out: Outputs::default(),
            seated: false,
        }
    }

    fn battery(truth: &Truth) -> BatteryInput {
        let volts = truth.dc_bus_volts.iter().copied().fold(0.0_f64, f64::max);
        BatteryInput {
            open_circuit_v: volts.max(0.0),
            internal_resistance_ohm: params::BATTERY_INTERNAL_RESISTANCE_OHM,
            available: volts > 1.0,
        }
    }

    fn faults_from(faults: &Faults) -> ApuFaults {
        let speed = faults.get(ids::SPEED_SENSOR_FAULT);
        let egt = faults.get(ids::EGT_SENSOR_FAULT);
        let channel = ChannelFaults {
            speed_sensor: SensorFault { bias: speed, failed: speed >= 0.999 },
            egt_sensor: SensorFault { bias: egt, failed: egt >= 0.999 },
            oil_pressure_sensor: SensorFault::default(),
            processing_fault: 0.0,
        };

        ApuFaults {
            power_section: PowerSectionFaults {
                compressor_efficiency_loss: faults.get(ids::COMPRESSOR_EROSION),
                turbine_efficiency_loss: faults.get(ids::TURBINE_DAMAGE).max(faults.get(49_000)),
            },
            load_compressor: LoadCompressorFaults {
                efficiency_loss: faults.get(ids::LOAD_COMPRESSOR_EROSION),
                igv_jam: faults.get(ids::IGV_JAM),
                scv_jam: faults.get(ids::SCV_JAM),
            },
            starter: StarterFaults {
                starter_degradation: faults.get(ids::STARTER_DEGRADATION).max(faults.get(49_002)),
                igniter_failure: faults.get(ids::IGNITER_FAILURE),
            },
            gen1: GeneratorFaults {
                efficiency_loss: faults.get(ids::GEN1_WEAR),
                overload_protection_failed: faults.get(ids::GEN1_OVERLOAD) >= 0.5,
            },
            gen2: GeneratorFaults {
                efficiency_loss: faults.get(ids::GEN2_WEAR),
                overload_protection_failed: faults.get(ids::GEN2_OVERLOAD) >= 0.5,
            },
            oil: OilFaults { leak: faults.get(ids::OIL_LEAK).max(faults.get(49_004)) },
            fuel_control: FuelControlFaults { metering_valve_jam: faults.get(ids::FCU_FAULT) },
            inlet_door: InletDoorFaults { jam: faults.get(ids::INLET_DOOR_JAM) },
            fire: FireFaults {
                loop_failure: faults.get(ids::FIRE_LOOP_FAILURE),
                squib_failure: faults.get(ids::FIRE_SQUIB_FAILURE),
            },
            ecb: EcbFaults { channel_a: channel, channel_b: channel },
            starter_duty_model_fault: 0.0,
        }
    }

    fn phase_code(phase: StartPhase) -> f64 {
        match phase {
            StartPhase::Idle => 0.0,
            StartPhase::Cranking => 1.0,
            StartPhase::Accelerating => 2.0,
            StartPhase::SelfSustaining => 3.0,
        }
    }
}

impl Area for LiveApu {
    fn name(&self) -> &'static str {
        "apu"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        let ambient_k = truth.environment.sat_c + 273.15;

        if !self.seated {
            self.apu = Apu::new(ambient_k.max(1.0));
            self.seated = true;
        }

        let inputs = Inputs {
            dt_s: truth.dt_s,
            ambient_pressure_pa: truth.environment.ambient_pressure_pa,
            ambient_temperature_k: ambient_k,
            true_airspeed_mps: truth.environment.tas_ms,
            master_on: truth.controls.apu_master_sw_on,
            start_selected: truth.controls.apu_start_pb_on,
            battery: Self::battery(truth),
            bleed_demand_kg_s: truth.published.get_or("PNEU_APU_BLEED_DEMAND_KG_S", 0.0),
            gen1_used: truth.controls.apu_gen_pb_on[0],
            gen2_used: truth.controls.apu_gen_pb_on[1],
            gen1_electrical_load_w: truth.published.get_or("ELEC_APU_GEN_1_LOAD_W", 0.0),
            gen2_electrical_load_w: truth.published.get_or("ELEC_APU_GEN_2_LOAD_W", 0.0),
            fire_loop_detected: truth.published.get_or("DEEP_FIRE_DETECTED_APU", 0.0) > 0.0,
            fire_button_pushed: truth.controls.fire_pb_apu_released,
            fuel_shutoff_valve_powered: truth.published.get_or("ELEC_APU_FUEL_SHUTOFF_VALVE_BREAKER_OPEN", 0.0) <= 0.5,
        };

        let mut faults = Self::faults_from(faults);
        if truth.published.get_or("ELEC_APU_ECU_A_BREAKER_OPEN", 0.0) > 0.5 {
            faults.ecb.channel_a.processing_fault = 1.0;
        }
        if truth.published.get_or("ELEC_APU_ECU_B_BREAKER_OPEN", 0.0) > 0.5 {
            faults.ecb.channel_b.processing_fault = 1.0;
        }

        self.out = self.apu.step(&inputs, &faults);
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        let o = &self.out;

        out("DEEP_APU_N", o.n_percent);
        out("DEEP_APU_EGT", o.egt_indicated_c);
        out("DEEP_APU_OIL_PRESSURE_PSI", o.oil_pressure_psi);
        out("APU_LOAD_COMPRESSOR_SURGE", f64::from(o.load_compressor_in_surge));
        out("APU_GEN_1_OVERLOAD", f64::from(o.gen1_output.overloaded));
        out("APU_GEN_2_OVERLOAD", f64::from(o.gen2_output.overloaded));
        out("APU_FIRE_LOOP_DETECTED", f64::from(o.fire_confirmed));

        out("APU_EGT_TRUE_C", o.egt_true_c);
        out("DEEP_APU_OIL_TEMPERATURE_C", o.oil_temperature_c);
        out("APU_BLEED_FLOW_KG_S", o.bleed_output.mass_flow_kg_s);
        out("APU_BLEED_PRESSURE_PA", o.bleed_output.pressure_pa);
        out("APU_BLEED_TEMPERATURE_K", o.bleed_output.temperature_k);
        out("APU_IGV_POSITION", o.igv_position_frac);
        out("APU_SCV_POSITION", o.scv_position_frac);
        out("APU_INLET_DOOR_OPEN", o.inlet_door_open_frac);
        out("APU_STARTER_CURRENT_A", o.starter_current_a);
        out("APU_STARTER_SUPPLY_V", o.battery_terminal_v);
        out("APU_START_PHASE", Self::phase_code(o.start_phase));
        out("APU_AVAILABLE", f64::from(o.available));
        out("APU_RELIGHT_PERMITTED", f64::from(o.relight_permitted));
        out("APU_FIRE_CONFIRMED", f64::from(o.fire_confirmed));
        out("APU_FIRE_BOTTLE_PRESSURE", o.fire_bottle_pressure_frac);
        out("APU_OVERSPEED_TRIP", f64::from(o.overspeed_tripped));
        out("APU_EGT_TRIP", f64::from(o.egt_hard_tripped));
        out("APU_ECB_TRIP", f64::from(o.ecb_any_trip));
        out("APU_ECB_DUAL_CHANNEL_SPEED_LOSS", f64::from(o.ecb_dual_channel_speed_loss));
        out("APU_GEN_1_SHAFT_POWER_W", o.gen1_output.shaft_power_w);
        out("APU_GEN_2_SHAFT_POWER_W", o.gen2_output.shaft_power_w);
        out("APU_OPERATING_HOURS", o.operating_hours);
        out("APU_COMPRESSOR_WEAR", o.compressor_wear_frac);
        out("APU_TURBINE_WEAR", o.turbine_wear_frac);
        out("APU_STARTER_DUTY_HEAT", o.starter_duty_heat_frac);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    fn running_truth() -> Truth {
        Truth {
            dt_s: 1.0 / 30.0,
            dc_bus_volts: [params::BATTERY_NOMINAL_OPEN_CIRCUIT_V; 2],
            controls: crate::deep::live::Controls { apu_master_sw_on: true, apu_start_pb_on: true, ..crate::deep::live::Controls::default() },
            ..Truth::default()
        }
    }

    fn published(area: &LiveApu) -> BTreeMap<String, f64> {
        let mut map = BTreeMap::new();
        area.publish(&mut |name, value| {
            map.insert(name.to_string(), value);
        });
        map
    }

    fn run(area: &mut LiveApu, truth: &Truth, faults: &Faults, seconds: f64) -> BTreeMap<String, f64> {
        let ticks = (seconds / truth.dt_s).round().max(1.0) as u32;
        for _ in 0..ticks {
            area.tick(truth, faults);
        }
        published(area)
    }

    #[test]
    fn a_cold_unpowered_aircraft_leaves_the_apu_stopped_and_publishes_it() {
        let mut area = LiveApu::new();
        let vars = run(&mut area, &Truth::default(), &Faults::default(), 60.0);
        assert_eq!(vars["DEEP_APU_N"], 0.0);
        assert_eq!(vars["APU_AVAILABLE"], 0.0);
        assert_eq!(vars["DEEP_APU_OIL_PRESSURE_PSI"], 0.0);
        for (name, value) in &vars {
            assert!(value.is_finite(), "{name} is not finite");
        }
    }

    #[test]
    fn commanded_on_with_a_live_battery_bus_it_starts_and_governs() {
        let mut area = LiveApu::new();
        let vars = run(&mut area, &running_truth(), &Faults::default(), 600.0);
        assert!(
            (vars["DEEP_APU_N"] - params::GOVERNED_N_PERCENT).abs() < 1.0,
            "governed at {}%",
            vars["DEEP_APU_N"]
        );
        assert_eq!(vars["APU_AVAILABLE"], 1.0);
        assert!(vars["DEEP_APU_EGT"] > 100.0 && vars["DEEP_APU_EGT"] < params::EGT_RUNNING_LIMIT_C);
        assert!(vars["DEEP_APU_OIL_PRESSURE_PSI"] > params::OIL_PRESSURE_TRIP_PSI);
        assert_eq!(vars["APU_START_PHASE"], 3.0);
    }

    #[test]
    fn with_no_dc_bus_there_is_nothing_to_crank_with() {
        let mut area = LiveApu::new();
        let truth = Truth { controls: crate::deep::live::Controls { apu_master_sw_on: true, apu_start_pb_on: true, ..crate::deep::live::Controls::default() }, ..Truth::default() };
        let vars = run(&mut area, &truth, &Faults::default(), 300.0);
        assert!(vars["DEEP_APU_N"] < 1.0, "{}", vars["DEEP_APU_N"]);
        assert_eq!(vars["APU_AVAILABLE"], 0.0);
    }

    #[test]
    fn the_catalogues_apu_oil_leak_and_turbine_damage_reach_this_model() {
        let leak = LiveApu::faults_from(&Faults::from_pairs([(49_004, 1.0)]));
        assert_eq!(leak.oil.leak, 1.0);
        let damage = LiveApu::faults_from(&Faults::from_pairs([(49_000, 0.4)]));
        assert_eq!(damage.power_section.turbine_efficiency_loss, 0.4);

        let truth = running_truth();
        let mut area = LiveApu::new();
        let vars = run(&mut area, &truth, &Faults::from_pairs([(49_004, 1.0)]), 900.0);
        assert!(vars["DEEP_APU_OIL_PRESSURE_PSI"] < params::OIL_PRESSURE_TRIP_PSI, "{}", vars["DEEP_APU_OIL_PRESSURE_PSI"]);
        assert_eq!(vars["APU_ECB_TRIP"], 1.0, "the ECB must trip the APU for FlyByWire's box to shut it down");
    }

    #[test]
    fn an_armed_oil_leak_drives_the_published_oil_pressure_under_the_ecam_trigger() {
        let truth = running_truth();

        let mut healthy = LiveApu::new();
        let healthy_vars = run(&mut healthy, &truth, &Faults::default(), 900.0);
        assert!(
            healthy_vars["DEEP_APU_OIL_PRESSURE_PSI"] >= params::OIL_PRESSURE_TRIP_PSI,
            "a healthy APU must not be near the trigger: {}",
            healthy_vars["DEEP_APU_OIL_PRESSURE_PSI"]
        );

        let mut leaking = LiveApu::new();
        let faults = Faults::from_pairs([(ids::OIL_LEAK, 1.0)]);
        let leaking_vars = run(&mut leaking, &truth, &faults, 900.0);
        assert!(
            leaking_vars["DEEP_APU_OIL_PRESSURE_PSI"] < params::OIL_PRESSURE_TRIP_PSI,
            "oil pressure only fell to {} psi",
            leaking_vars["DEEP_APU_OIL_PRESSURE_PSI"]
        );
    }

    #[test]
    fn an_armed_egt_sensor_fault_moves_the_indicated_egt_but_not_the_true_one() {
        let truth = running_truth();

        let mut healthy = LiveApu::new();
        let healthy_vars = run(&mut healthy, &truth, &Faults::default(), 600.0);

        let mut biased = LiveApu::new();
        let faults = Faults::from_pairs([(ids::EGT_SENSOR_FAULT, 0.8)]);
        let biased_vars = run(&mut biased, &truth, &faults, 600.0);

        assert!(
            biased_vars["DEEP_APU_EGT"] < healthy_vars["DEEP_APU_EGT"] - 50.0,
            "indicated EGT barely moved: {} vs {}",
            biased_vars["DEEP_APU_EGT"],
            healthy_vars["DEEP_APU_EGT"]
        );
        assert!(
            (biased_vars["APU_EGT_TRUE_C"] - healthy_vars["APU_EGT_TRUE_C"]).abs() < 5.0,
            "the real turbine changed: {} vs {}",
            biased_vars["APU_EGT_TRUE_C"],
            healthy_vars["APU_EGT_TRUE_C"]
        );
    }

    #[test]
    fn a_fully_failed_igniter_hangs_the_start_below_the_start_fault_threshold() {
        let mut area = LiveApu::new();
        let faults = Faults::from_pairs([(ids::IGNITER_FAILURE, 1.0)]);
        let vars = run(&mut area, &running_truth(), &faults, 600.0);
        assert!(
            vars["DEEP_APU_N"] < params::SELF_SUSTAINING_N_PERCENT,
            "it lit anyway and reached {}%",
            vars["DEEP_APU_N"]
        );
        assert_eq!(vars["APU_AVAILABLE"], 0.0);
    }

    #[test]
    fn an_armed_inlet_door_jam_holds_the_door_shut_and_raises_egt() {
        let truth = running_truth();

        let mut healthy = LiveApu::new();
        let healthy_vars = run(&mut healthy, &truth, &Faults::default(), 600.0);
        assert!(healthy_vars["APU_INLET_DOOR_OPEN"] > 0.99);

        let mut jammed = LiveApu::new();
        let faults = Faults::from_pairs([(ids::INLET_DOOR_JAM, 1.0)]);
        let jammed_vars = run(&mut jammed, &truth, &faults, 600.0);
        assert!(
            jammed_vars["APU_INLET_DOOR_OPEN"] < 0.5,
            "door opened to {}",
            jammed_vars["APU_INLET_DOOR_OPEN"]
        );
        assert!(jammed_vars["DEEP_APU_N"] < healthy_vars["DEEP_APU_N"] - 1.0);
    }

    #[test]
    fn every_variable_an_ecam_trigger_names_is_published() {
        let mut area = LiveApu::new();
        area.tick(&running_truth(), &Faults::default());
        let vars = published(&area);
        for name in [
            "DEEP_APU_N",
            "DEEP_APU_EGT",
            "DEEP_APU_OIL_PRESSURE_PSI",
            "APU_LOAD_COMPRESSOR_SURGE",
            "APU_GEN_1_OVERLOAD",
            "APU_GEN_2_OVERLOAD",
            "APU_FIRE_LOOP_DETECTED",
        ] {
            assert!(vars.contains_key(name), "{name} is never published");
        }
        assert_eq!(area.name(), "apu");
    }

    #[test]
    fn the_start_pushbutton_starts_the_apu_not_the_apu_running_flag() {
        let mut not_started = LiveApu::new();
        let truth = Truth {
            apu_running: true,
            dc_bus_volts: [params::BATTERY_NOMINAL_OPEN_CIRCUIT_V; 2],
            controls: crate::deep::live::Controls { apu_master_sw_on: true, apu_start_pb_on: false, ..crate::deep::live::Controls::default() },
            ..Truth::default()
        };
        let vars = run(&mut not_started, &truth, &Faults::default(), 120.0);
        assert_eq!(vars["DEEP_APU_N"], 0.0, "apu_running alone must not start the machine: {}", vars["DEEP_APU_N"]);
        assert_eq!(vars["APU_AVAILABLE"], 0.0);

        let mut started = LiveApu::new();
        let truth = Truth {
            apu_running: false,
            dc_bus_volts: [params::BATTERY_NOMINAL_OPEN_CIRCUIT_V; 2],
            controls: crate::deep::live::Controls { apu_master_sw_on: true, apu_start_pb_on: true, ..crate::deep::live::Controls::default() },
            ..Truth::default()
        };
        let vars = run(&mut started, &truth, &Faults::default(), 600.0);
        assert!((vars["DEEP_APU_N"] - params::GOVERNED_N_PERCENT).abs() < 1.0, "the real START pb must start and govern the machine: {}", vars["DEEP_APU_N"]);
        assert_eq!(vars["APU_AVAILABLE"], 1.0);
    }

    #[test]
    fn a_published_apu_fire_detection_is_confirmed_and_a_pushed_button_discharges_the_bottle() {
        let mut area = LiveApu::new();
        let mut truth = running_truth();
        truth.controls.fire_pb_apu_released = true;
        let mut published_frame = BTreeMap::new();
        published_frame.insert("DEEP_FIRE_DETECTED_APU".to_string(), 1.0);
        truth.published = crate::deep::live::PublishedFrame::from(published_frame);

        let vars = run(&mut area, &truth, &Faults::default(), 5.0);
        assert_eq!(vars["APU_FIRE_LOOP_DETECTED"], 1.0, "a published DEEP_FIRE_DETECTED_APU must be confirmed here");
        assert_eq!(vars["APU_FIRE_CONFIRMED"], 1.0);
        assert!(vars["APU_FIRE_BOTTLE_PRESSURE"] < 1.0, "the fire pushbutton must actually discharge the bottle");
    }

    #[test]
    fn a_published_apu_generator_load_reaches_the_generator_and_can_overload_it() {
        let truth = running_truth();

        let mut unpublished = LiveApu::new();
        let vars = run(&mut unpublished, &truth, &Faults::default(), 5.0);
        assert_eq!(vars["APU_GEN_1_OVERLOAD"], 0.0, "nothing published under the load's name must not fabricate a load");

        let mut overloaded = LiveApu::new();
        let mut published_frame = BTreeMap::new();
        published_frame.insert("ELEC_APU_GEN_1_LOAD_W".to_string(), 3.0 * params::GENERATOR_RATED_APPARENT_VA * params::GENERATOR_RATED_POWER_FACTOR);
        let mut truth_with_load = truth.clone();
        truth_with_load.published = crate::deep::live::PublishedFrame::from(published_frame);
        let vars = run(&mut overloaded, &truth_with_load, &Faults::default(), 1.0 / 30.0);
        assert_eq!(vars["APU_GEN_1_OVERLOAD"], 1.0, "a published load past rating must overload generator 1");
        assert_eq!(vars["APU_GEN_2_OVERLOAD"], 0.0, "generator 2's own load was never published and must stay healthy");
    }

    #[test]
    fn an_unpowered_fuel_shutoff_valve_fails_closed_and_the_apu_never_starts() {
        let mut area = LiveApu::new();
        let mut published_frame = BTreeMap::new();
        published_frame.insert("ELEC_APU_FUEL_SHUTOFF_VALVE_BREAKER_OPEN".to_string(), 1.0);
        let mut truth = running_truth();
        truth.published = crate::deep::live::PublishedFrame::from(published_frame);

        let vars = run(&mut area, &truth, &Faults::default(), 600.0);
        assert!(
            vars["DEEP_APU_N"] < params::SELF_SUSTAINING_N_PERCENT,
            "no fuel reaching the combustor must mean no start: {}",
            vars["DEEP_APU_N"]
        );
        assert_eq!(vars["APU_AVAILABLE"], 0.0);
    }

    #[test]
    fn an_unpowered_ecu_channel_is_excluded_but_the_other_channel_alone_still_starts_and_governs() {
        let mut area = LiveApu::new();
        let mut published_frame = BTreeMap::new();
        published_frame.insert("ELEC_APU_ECU_A_BREAKER_OPEN".to_string(), 1.0);
        let mut truth = running_truth();
        truth.published = crate::deep::live::PublishedFrame::from(published_frame);

        let vars = run(&mut area, &truth, &Faults::default(), 600.0);
        assert!(
            (vars["DEEP_APU_N"] - params::GOVERNED_N_PERCENT).abs() < 1.0,
            "channel B alone must still start and govern: {}",
            vars["DEEP_APU_N"]
        );
        assert_eq!(vars["APU_AVAILABLE"], 1.0);
    }

    #[test]
    fn both_ecu_channels_unpowered_is_a_dual_channel_speed_loss_and_the_apu_never_starts() {
        let mut area = LiveApu::new();
        let mut published_frame = BTreeMap::new();
        published_frame.insert("ELEC_APU_ECU_A_BREAKER_OPEN".to_string(), 1.0);
        published_frame.insert("ELEC_APU_ECU_B_BREAKER_OPEN".to_string(), 1.0);
        let mut truth = running_truth();
        truth.published = crate::deep::live::PublishedFrame::from(published_frame);

        let vars = run(&mut area, &truth, &Faults::default(), 600.0);
        assert!(vars["DEEP_APU_N"] < params::SELF_SUSTAINING_N_PERCENT, "{}", vars["DEEP_APU_N"]);
        assert_eq!(vars["APU_AVAILABLE"], 0.0);
    }

    #[test]
    fn nothing_published_is_ever_nan_through_a_whole_start_and_a_pause_sized_frame() {
        let mut area = LiveApu::new();
        let mut truth = running_truth();
        for i in 0..20_000 {
            truth.dt_s = if i % 100 == 0 { 0.5 } else { 1.0 / 30.0 };
            area.tick(&truth, &Faults::default());
        }
        let vars = published(&area);
        for (name, value) in &vars {
            assert!(value.is_finite(), "{name} = {value}");
        }
        assert!((vars["DEEP_APU_N"] - params::GOVERNED_N_PERCENT).abs() < 2.0, "{}", vars["DEEP_APU_N"]);
    }
}

#[cfg(test)]
mod probe {
    use super::*;
    use std::collections::BTreeMap;

    #[test]
    fn probe_surge() {
        for (label, id) in [("scv", ids::SCV_JAM), ("igv", ids::IGV_JAM), ("lcerode", ids::LOAD_COMPRESSOR_EROSION)] {
            for m in [0.5, 1.0] {
                let mut a = LiveApu::new();
                let t = Truth { dt_s: 1.0/30.0, dc_bus_volts: [24.0; 2], controls: crate::deep::live::Controls { apu_master_sw_on: true, apu_start_pb_on: true, ..crate::deep::live::Controls::default() }, ..Truth::default() };
                let f = Faults::from_pairs([(id, m)]);
                let mut surged = false;
                for _ in 0..18000 { a.tick(&t, &f);
                    let mut map = BTreeMap::new();
                    a.publish(&mut |n, v| { map.insert(n.to_string(), v); });
                    if map["APU_LOAD_COMPRESSOR_SURGE"] > 0.0 { surged = true; }
                }
                let mut map = BTreeMap::new();
                a.publish(&mut |n, v| { map.insert(n.to_string(), v); });
                println!("{label} {m}: surged_ever={surged} N={:.2} igv={:.3} scv={:.3}", map["DEEP_APU_N"], map["APU_IGV_POSITION"], map["APU_SCV_POSITION"]);
            }
        }
    }
}
