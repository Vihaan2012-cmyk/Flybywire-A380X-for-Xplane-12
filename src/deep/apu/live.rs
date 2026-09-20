//! The live APU: one PW980A-class machine, owned and stepped every frame.
//!
//! `apu.rs` already assembles the whole machine -- power section, load
//! compressor, ECB, governor and EGT limiter, oil system, starter, inlet
//! door, generators, fire interface, life tracking -- into one
//! [`Apu::step`]. What was missing is an owner: nothing in the running
//! plugin held an `Apu`, so none of that physics ever ran. This module is
//! that owner, and the translation between `deep::live::Truth`/`Faults` and
//! this directory's own input and fault structs.
//!
//! ## What drives it
//!
//! | `apu::Inputs` field | from |
//! |---|---|
//! | `dt_s`, ambient pressure/temperature, TAS | `Truth` directly |
//! | `battery` | `Truth::dc_bus_volts` (see [`LiveApu::battery`]) |
//! | `master_on` | `Truth::controls.apu_master_sw_on` |
//! | `start_selected` | `Truth::controls.apu_start_pb_on` |
//! | `fire_loop_detected` | `Truth::published`'s `FIRE_DETECTED_APU` |
//!
//! `master_on`/`start_selected` used to both be taken from `Truth::
//! apu_running` -- the plugin's own "the APU is commanded to run" flag --
//! which meant the crew pressing START and the ECB's own decision that the
//! start failed were the same bit. `Truth::controls.apu_master_sw_on`/
//! `apu_start_pb_on` are the real MASTER SW and START pushbuttons now, named
//! for exactly this module's own long-standing request, so they separate
//! the two: the machine only *starts* on a real START pb press, and can now
//! genuinely fail a start (hung/aborted) while the crew's own selection
//! stays visible as a distinct signal.
//!
//! Fire detection is `deep::fire_ice`'s own loop, not modelled again here;
//! this directory only confirms and acts on it. That area publishes the
//! APU bay's own confirmed detection as `FIRE_DETECTED_APU` (`fire_ice::
//! live`'s own `Names::fire_detected`), read here one frame behind through
//! `Truth::published` -- the documented inter-area mechanism -- rather than
//! the permanent `false` this module used to hold `fire_loop_detected` at.
//! `fire_button_pushed` (the APU fire pushbutton itself, which commands the
//! ground/automatic bottle discharge `fire_ice` already models) is
//! `Truth::controls.fire_pb_apu_released` for the same reason.
//!
//! Two inputs the machine genuinely needs are still not in `Truth` (or
//! published by any area) and nothing here invents them
//! (`docs/deep/BRIEF.md` hard rule 3):
//!
//! * **Bleed demand.** The load compressor's customer demand comes from the
//!   pneumatic ducts, which is another area; areas can read another area's
//!   *published* output (`Truth::published`), but `deep::pneumatic_ducts`
//!   does not publish a bleed demand figure today, so this stays zero. Zero
//!   is a real operating state -- APU BLEED off, IGVs closed, surge valve
//!   open -- not a placeholder, but it is not the only one, and either a
//!   published pneumatic demand or `Truth::apu_bleed_demand_kg_s` would make
//!   the other ones reachable.
//! * **Generator electrical load.** `Truth::ac_bus_volts` says which AC
//!   buses are alive but not how many watts the APU's two generators are
//!   carrying, so both are modelled unloaded and neither can be driven into
//!   its overload. `deep::electrical` computes this every tick internally
//!   (`ElectricalLive::measured_apu_gen_load_w`) but does not publish it;
//!   publishing it (or a `Truth::apu_generator_load_w: [f64; 2]`) would make
//!   the APU GEN 1/2 FAULT alerts' overload path reachable.

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

/// The live system for this area.
pub fn live_system() -> Box<dyn Area> {
    Box::new(LiveApu::new())
}

pub struct LiveApu {
    apu: Apu,
    out: Outputs,
    /// Whether the machine has been seated against real ambient
    /// conditions yet, see [`LiveApu::tick`].
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
            // ISA sea level: the cold state `Truth::default()` describes.
            // The first real frame re-seats it against actual ambient (see
            // `tick`).
            apu: Apu::new(288.15),
            out: Outputs::default(),
            seated: false,
        }
    }

    /// What the starter is fed from. `Truth` publishes FlyByWire's own DC
    /// bus voltages, which is a real measured terminal voltage of whatever
    /// is powering the bus (battery, TR, or external power), so it is used
    /// as the source voltage directly; the higher of the two buses wins,
    /// because the APU start contactor draws from whichever battery bus is
    /// alive. `Truth` carries no source *resistance*, so the model's own
    /// nominal battery internal resistance is used -- this is the one place
    /// the starter's supply is not fully real, and a
    /// `Truth::apu_start_source_resistance_ohm` (or a Thevenin pair) would
    /// close it.
    fn battery(truth: &Truth) -> BatteryInput {
        let volts = truth.dc_bus_volts.iter().copied().fold(0.0_f64, f64::max);
        BatteryInput {
            open_circuit_v: volts.max(0.0),
            internal_resistance_ohm: params::BATTERY_INTERNAL_RESISTANCE_OHM,
            // Below a volt there is no bus: nothing to crank with.
            available: volts > 1.0,
        }
    }

    /// Every failure `registry.rs` registers for this area, read by id and
    /// placed in the exact model field that entry's `model_field` names.
    fn faults_from(faults: &Faults) -> ApuFaults {
        // A single id covers both ECB channels for each sensor
        // (`registry.rs`: `model_field = "...channel_a/b..."`), and that is
        // what makes its registered effect reachable: a fault on one
        // channel alone is masked by the other channel's vote, which is
        // exactly what `ecb.rs`'s voting exists to do. Arming the catalogue
        // failure therefore means the common-mode case -- both pickups
        // biased the same way, so the governor really does see a wrong
        // speed and over-fuels toward overspeed.
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
                turbine_efficiency_loss: faults.get(ids::TURBINE_DAMAGE),
            },
            load_compressor: LoadCompressorFaults {
                efficiency_loss: faults.get(ids::LOAD_COMPRESSOR_EROSION),
                igv_jam: faults.get(ids::IGV_JAM),
                scv_jam: faults.get(ids::SCV_JAM),
            },
            starter: StarterFaults {
                starter_degradation: faults.get(ids::STARTER_DEGRADATION),
                igniter_failure: faults.get(ids::IGNITER_FAILURE),
            },
            gen1: GeneratorFaults {
                efficiency_loss: faults.get(ids::GEN1_WEAR),
                // Registered as "boolean, represented as 0/1".
                overload_protection_failed: faults.get(ids::GEN1_OVERLOAD) >= 0.5,
            },
            gen2: GeneratorFaults {
                efficiency_loss: faults.get(ids::GEN2_WEAR),
                overload_protection_failed: faults.get(ids::GEN2_OVERLOAD) >= 0.5,
            },
            oil: OilFaults { leak: faults.get(ids::OIL_LEAK) },
            fuel_control: FuelControlFaults { metering_valve_jam: faults.get(ids::FCU_FAULT) },
            inlet_door: InletDoorFaults { jam: faults.get(ids::INLET_DOOR_JAM) },
            fire: FireFaults {
                loop_failure: faults.get(ids::FIRE_LOOP_FAILURE),
                squib_failure: faults.get(ids::FIRE_SQUIB_FAILURE),
            },
            ecb: EcbFaults { channel_a: channel, channel_b: channel },
            // `life::StarterDutyCycle`'s own model fault is not in the
            // catalogue: it degrades an *estimate* the ECB keeps, not a
            // physical part, so there is no component for it to live on.
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

        // `new` had to assume ISA sea level, because no frame had been
        // filled yet. The first real frame re-seats the machine's oil and
        // metal temperatures at the ambient the aircraft is actually
        // sitting in -- once only: from then on those temperatures are its
        // own state, and throwing them away every frame would mean a
        // shut-down APU never cooled down or warmed up at all.
        if !self.seated {
            self.apu = Apu::new(ambient_k.max(1.0));
            self.seated = true;
        }

        let inputs = Inputs {
            dt_s: truth.dt_s,
            ambient_pressure_pa: truth.environment.ambient_pressure_pa,
            ambient_temperature_k: ambient_k,
            true_airspeed_mps: truth.environment.tas_ms,
            // The real MASTER SW and START pushbuttons, not `apu_running`:
            // see the module doc. The crew's own selection and the ECB's
            // decision that a start failed are no longer the same bit.
            master_on: truth.controls.apu_master_sw_on,
            start_selected: truth.controls.apu_start_pb_on,
            battery: Self::battery(truth),
            // See the module docs: still no `Truth`/published bleed demand
            // or generator load.
            bleed_demand_kg_s: 0.0,
            gen1_used: false,
            gen2_used: false,
            gen1_electrical_load_w: 0.0,
            gen2_electrical_load_w: 0.0,
            // `deep::fire_ice`'s own confirmed APU bay detection, one frame
            // behind through the documented published-frame mechanism (see
            // the module doc). Absent (nothing published yet, e.g. the
            // first frame) reads as no fire, never as a fabricated one.
            fire_loop_detected: truth.published.get_or("FIRE_DETECTED_APU", 0.0) > 0.0,
            fire_button_pushed: truth.controls.fire_pb_apu_released,
        };

        self.out = self.apu.step(&inputs, &Self::faults_from(faults));
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        let o = &self.out;

        // --- the variables `registry.rs` triggers ECAM alerts on ---------
        out("APU_N", o.n_percent);
        // Indicated, not true: an EGT thermocouple fault is registered
        // precisely because what the crew and the ECAM see can differ from
        // what the turbine is actually doing.
        out("APU_EGT", o.egt_indicated_c);
        out("APU_OIL_PRESSURE_PSI", o.oil_pressure_psi);
        out("APU_LOAD_COMPRESSOR_SURGE", f64::from(o.load_compressor_in_surge));
        out("APU_GEN_1_OVERLOAD", f64::from(o.gen1_output.overloaded));
        out("APU_GEN_2_OVERLOAD", f64::from(o.gen2_output.overloaded));
        // The APU's own contribution to the APU FIRE warning the fire
        // protection area owns. `fire.rs` confirms a detected fire only
        // through a healthy loop, so this is the *confirmed* detection, not
        // the raw input.
        out("APU_FIRE_LOOP_DETECTED", f64::from(o.fire_confirmed));

        // --- the rest of the machine, for the EFB Study pages ------------
        out("APU_EGT_TRUE_C", o.egt_true_c);
        out("APU_OIL_TEMPERATURE_C", o.oil_temperature_c);
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

    /// A ground truth with the battery bus alive and the crew holding
    /// MASTER SW and START on -- everything this area can actually be
    /// given today. Deliberately does not set `Truth::apu_running` (the
    /// plugin's own separate "is the APU actually running" flag, which
    /// this live system no longer reads): the whole point of this pass is
    /// that the crew's own selection and the machine's own state are two
    /// different things now.
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

    /// Runs `seconds` of simulated time and returns what the area
    /// publishes at the end.
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
        assert_eq!(vars["APU_N"], 0.0);
        assert_eq!(vars["APU_AVAILABLE"], 0.0);
        assert_eq!(vars["APU_OIL_PRESSURE_PSI"], 0.0);
        for (name, value) in &vars {
            assert!(value.is_finite(), "{name} is not finite");
        }
    }

    #[test]
    fn commanded_on_with_a_live_battery_bus_it_starts_and_governs() {
        let mut area = LiveApu::new();
        let vars = run(&mut area, &running_truth(), &Faults::default(), 600.0);
        assert!(
            (vars["APU_N"] - params::GOVERNED_N_PERCENT).abs() < 1.0,
            "governed at {}%",
            vars["APU_N"]
        );
        assert_eq!(vars["APU_AVAILABLE"], 1.0);
        assert!(vars["APU_EGT"] > 100.0 && vars["APU_EGT"] < params::EGT_RUNNING_LIMIT_C);
        assert!(vars["APU_OIL_PRESSURE_PSI"] > params::OIL_PRESSURE_TRIP_PSI);
        assert_eq!(vars["APU_START_PHASE"], 3.0);
    }

    #[test]
    fn with_no_dc_bus_there_is_nothing_to_crank_with() {
        let mut area = LiveApu::new();
        let truth = Truth { controls: crate::deep::live::Controls { apu_master_sw_on: true, apu_start_pb_on: true, ..crate::deep::live::Controls::default() }, ..Truth::default() };
        let vars = run(&mut area, &truth, &Faults::default(), 300.0);
        assert!(vars["APU_N"] < 1.0, "{}", vars["APU_N"]);
        assert_eq!(vars["APU_AVAILABLE"], 0.0);
    }

    /// `registry.rs`'s oil leak: "Tank level falls; the pump progressively
    /// starves ... pressure falls, and sustained low pressure while running
    /// trips low oil pressure protection." `APU_OIL_LO_PR` triggers on
    /// `APU_OIL_PRESSURE_PSI < 15`, so the published variable has to get
    /// there on its own.
    #[test]
    fn an_armed_oil_leak_drives_the_published_oil_pressure_under_the_ecam_trigger() {
        let truth = running_truth();

        let mut healthy = LiveApu::new();
        let healthy_vars = run(&mut healthy, &truth, &Faults::default(), 900.0);
        assert!(
            healthy_vars["APU_OIL_PRESSURE_PSI"] >= params::OIL_PRESSURE_TRIP_PSI,
            "a healthy APU must not be near the trigger: {}",
            healthy_vars["APU_OIL_PRESSURE_PSI"]
        );

        let mut leaking = LiveApu::new();
        let faults = Faults::from_pairs([(ids::OIL_LEAK, 1.0)]);
        let leaking_vars = run(&mut leaking, &truth, &faults, 900.0);
        assert!(
            leaking_vars["APU_OIL_PRESSURE_PSI"] < params::OIL_PRESSURE_TRIP_PSI,
            "oil pressure only fell to {} psi",
            leaking_vars["APU_OIL_PRESSURE_PSI"]
        );
    }

    /// `registry.rs`'s EGT thermocouple fault: "Cockpit-indicated EGT reads
    /// low relative to the true turbine-exit temperature ... the true
    /// physics and the hard protective trip are unaffected." `APU_EGT` is
    /// the indicated one, so it must move and `APU_EGT_TRUE_C` must not.
    #[test]
    fn an_armed_egt_sensor_fault_moves_the_indicated_egt_but_not_the_true_one() {
        let truth = running_truth();

        let mut healthy = LiveApu::new();
        let healthy_vars = run(&mut healthy, &truth, &Faults::default(), 600.0);

        let mut biased = LiveApu::new();
        let faults = Faults::from_pairs([(ids::EGT_SENSOR_FAULT, 0.8)]);
        let biased_vars = run(&mut biased, &truth, &faults, 600.0);

        assert!(
            biased_vars["APU_EGT"] < healthy_vars["APU_EGT"] - 50.0,
            "indicated EGT barely moved: {} vs {}",
            biased_vars["APU_EGT"],
            healthy_vars["APU_EGT"]
        );
        assert!(
            (biased_vars["APU_EGT_TRUE_C"] - healthy_vars["APU_EGT_TRUE_C"]).abs() < 5.0,
            "the real turbine changed: {} vs {}",
            biased_vars["APU_EGT_TRUE_C"],
            healthy_vars["APU_EGT_TRUE_C"]
        );
    }

    /// `registry.rs`'s igniter failure: "At full failure the threshold sits
    /// at/above self-sustaining speed ... a hung start with fuel never
    /// lit." That is what `APU_START_FAULT` (START pb on, `APU_N < 55`)
    /// exists to catch.
    #[test]
    fn a_fully_failed_igniter_hangs_the_start_below_the_start_fault_threshold() {
        let mut area = LiveApu::new();
        let faults = Faults::from_pairs([(ids::IGNITER_FAILURE, 1.0)]);
        let vars = run(&mut area, &running_truth(), &faults, 600.0);
        assert!(
            vars["APU_N"] < params::SELF_SUSTAINING_N_PERCENT,
            "it lit anyway and reached {}%",
            vars["APU_N"]
        );
        assert_eq!(vars["APU_AVAILABLE"], 0.0);
    }

    /// `registry.rs`'s inlet door jam: "Door fails to reach fully open,
    /// imposing a continuing inlet total-pressure loss that reduces
    /// available power and raises EGT for the same demand."
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
        assert!(jammed_vars["APU_N"] < healthy_vars["APU_N"] - 1.0);
    }

    #[test]
    fn every_variable_an_ecam_trigger_names_is_published() {
        let mut area = LiveApu::new();
        area.tick(&running_truth(), &Faults::default());
        let vars = published(&area);
        for name in [
            "APU_N",
            "APU_EGT",
            "APU_OIL_PRESSURE_PSI",
            "APU_LOAD_COMPRESSOR_SURGE",
            "APU_GEN_1_OVERLOAD",
            "APU_GEN_2_OVERLOAD",
            "APU_FIRE_LOOP_DETECTED",
        ] {
            assert!(vars.contains_key(name), "{name} is never published");
        }
        assert_eq!(area.name(), "apu");
    }

    /// The whole point of this pass: `truth.controls.apu_start_pb_on` is
    /// what starts the machine now, not `Truth::apu_running` (the plugin's
    /// own, separate "the APU is actually running" flag, which used to
    /// double as the start command too).
    #[test]
    fn the_start_pushbutton_starts_the_apu_not_the_apu_running_flag() {
        // `apu_running` claims the machine is already running, but nobody
        // pressed START: master on, start pb off must leave it cold.
        let mut not_started = LiveApu::new();
        let truth = Truth {
            apu_running: true,
            dc_bus_volts: [params::BATTERY_NOMINAL_OPEN_CIRCUIT_V; 2],
            controls: crate::deep::live::Controls { apu_master_sw_on: true, apu_start_pb_on: false, ..crate::deep::live::Controls::default() },
            ..Truth::default()
        };
        let vars = run(&mut not_started, &truth, &Faults::default(), 120.0);
        assert_eq!(vars["APU_N"], 0.0, "apu_running alone must not start the machine: {}", vars["APU_N"]);
        assert_eq!(vars["APU_AVAILABLE"], 0.0);

        // The reverse: `apu_running` says the machine is not running, but a
        // real START pb press (with the battery bus alive) must still spin
        // it up and bring it to governed speed.
        let mut started = LiveApu::new();
        let truth = Truth {
            apu_running: false,
            dc_bus_volts: [params::BATTERY_NOMINAL_OPEN_CIRCUIT_V; 2],
            controls: crate::deep::live::Controls { apu_master_sw_on: true, apu_start_pb_on: true, ..crate::deep::live::Controls::default() },
            ..Truth::default()
        };
        let vars = run(&mut started, &truth, &Faults::default(), 600.0);
        assert!((vars["APU_N"] - params::GOVERNED_N_PERCENT).abs() < 1.0, "the real START pb must start and govern the machine: {}", vars["APU_N"]);
        assert_eq!(vars["APU_AVAILABLE"], 1.0);
    }

    /// `deep::fire_ice`'s own confirmed APU bay detection now reaches this
    /// area through `Truth::published`, replacing the permanent `false`
    /// this module used to hold `fire_loop_detected` at.
    #[test]
    fn a_published_apu_fire_detection_is_confirmed_and_a_pushed_button_discharges_the_bottle() {
        let mut area = LiveApu::new();
        let mut truth = running_truth();
        truth.controls.fire_pb_apu_released = true;
        // `Truth::published` is one frame behind; seed it directly the way
        // `Deep::tick` would after `deep::fire_ice` published a confirmed
        // detection last frame.
        let mut published_frame = BTreeMap::new();
        published_frame.insert("FIRE_DETECTED_APU".to_string(), 1.0);
        truth.published = crate::deep::live::PublishedFrame(published_frame);

        let vars = run(&mut area, &truth, &Faults::default(), 5.0);
        assert_eq!(vars["APU_FIRE_LOOP_DETECTED"], 1.0, "a published FIRE_DETECTED_APU must be confirmed here");
        assert_eq!(vars["APU_FIRE_CONFIRMED"], 1.0);
        assert!(vars["APU_FIRE_BOTTLE_PRESSURE"] < 1.0, "the fire pushbutton must actually discharge the bottle");
    }

    #[test]
    fn nothing_published_is_ever_nan_through_a_whole_start_and_a_pause_sized_frame() {
        let mut area = LiveApu::new();
        let mut truth = running_truth();
        for i in 0..20_000 {
            // Every hundredth frame is a half-second one, the size X-Plane
            // hands out after a pause.
            truth.dt_s = if i % 100 == 0 { 0.5 } else { 1.0 / 30.0 };
            area.tick(&truth, &Faults::default());
        }
        let vars = published(&area);
        for (name, value) in &vars {
            assert!(value.is_finite(), "{name} = {value}");
        }
        assert!((vars["APU_N"] - params::GOVERNED_N_PERCENT).abs() < 2.0, "{}", vars["APU_N"]);
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
                println!("{label} {m}: surged_ever={surged} N={:.2} igv={:.3} scv={:.3}", map["APU_N"], map["APU_IGV_POSITION"], map["APU_SCV_POSITION"]);
            }
        }
    }
}
