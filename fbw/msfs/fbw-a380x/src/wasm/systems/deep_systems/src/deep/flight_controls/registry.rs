use crate::deep::api::*;

const ATA: u16 = 27;

struct Ids {
    next: u16,
}
impl Ids {
    fn new() -> Self {
        Self { next: 1 }
    }
    fn next(&mut self) -> u16 {
        let n = self.next;
        self.next += 1;
        n
    }
}

struct FieldSpec {
    param: String,
    meaning: String,
    title: String,
    model_field: String,
    magnitude: String,
    effect: String,
}

fn field(param: &str, meaning: &str, title: &str, model_field: String, magnitude: &str, effect: &str) -> FieldSpec {
    FieldSpec { param: param.to_string(), meaning: meaning.to_string(), title: title.to_string(), model_field, magnitude: magnitude.to_string(), effect: effect.to_string() }
}

fn register_component(r: &mut Registry, ids: &mut Ids, comp_id: &str, comp_name: &str, fields: Vec<FieldSpec>) -> Vec<u64> {
    let mut fids = Vec::new();
    let mut params = Vec::new();
    for f in &fields {
        fids.push(failure_id(Area::FlightControls, ATA, ids.next()));
        params.push(ParamDef { name: f.param.clone(), meaning: f.meaning.clone(), healthy: 0.0 });
    }
    r.component(ComponentDef {
        id: comp_id.to_string(),
        area: Area::FlightControls,
        ata: ATA,
        name: comp_name.to_string(),
        params,
        failures: fids.clone(),
    });
    for (f, id) in fields.iter().zip(fids.iter()) {
        r.failure(FailureDef {
            id: *id,
            area: Area::FlightControls,
            ata: ATA,
            name: format!("{comp_name} {}", f.title),
            component: comp_id.to_string(),
            model_field: f.model_field.clone(),
            magnitude: f.magnitude.clone(),
            effect: f.effect.clone(),
        });
    }
    fids
}

fn surface_fields(model_path: &str) -> Vec<FieldSpec> {
    vec![
        field(
            "jam",
            "0 free .. 1 mechanically seized",
            "actuator jam",
            format!("{model_path}::ActuatorFaults.jam"),
            "0 free .. 1 fully seized: the actuator pins at the angle it jammed",
            "servo authority falls toward zero and a strong resistive spring pins the surface near its jam angle, breakable only by an external torque exceeding the jam's own resistance",
        ),
        field(
            "runaway",
            "0 none .. 1 full-authority servo hardover",
            "servo hardover",
            format!("{model_path}::ActuatorFaults.runaway/.runaway_sign"),
            "0 none .. 1 full-rate drive in runaway_sign's direction regardless of command",
            "the surface drives toward one stop unless the other actuators on the same surface, or the aerodynamic hinge moment, have enough authority to override it",
        ),
        field(
            "supply_loss",
            "0 full hydraulic/electrical supply .. 1 none",
            "hydraulic/electrical supply loss",
            format!("{model_path}::ActuatorFaults.supply_loss"),
            "0 full supply pressure .. 1 none",
            "force ceiling falls linearly and rate ceiling as sqrt(supply); at 1.0 the actuator can only be moved by the other actuators on the surface or the airload",
        ),
        field(
            "transducer_fault",
            "0 healthy .. 1 position feedback frozen/biased",
            "position transducer fault",
            format!("{model_path}::ActuatorFaults.transducer_frozen/.transducer_bias_rad"),
            "0 healthy .. 1 feedback frozen at the angle seen when the fault began (or biased, for a partial fault)",
            "the servo's own position loop chases a stale/wrong reading, driving the true surface away from its commanded position even while the loop reads \"on target\"",
        ),
        field(
            "disconnect",
            "0 linked .. 1 mechanical link to the surface sheared",
            "surface disconnect",
            format!("{model_path}: surface::SurfaceFaults.disconnected"),
            "boolean in practice: 0 linked, 1 sheared",
            "zero actuator torque reaches the surface; it free-floats, weathervaning under its own aerodynamic restoring moment and structural damping alone",
        ),
        field(
            "flutter_damper_loss",
            "0 healthy .. 1 dedicated flutter/gust damper failed",
            "flutter damper loss",
            format!("{model_path}: surface::SurfaceFaults.flutter_damper_loss"),
            "0 healthy .. 1 dedicated damper failed",
            "net hinge damping can go negative at high dynamic pressure, letting a disturbance grow instead of decay (a reduced-order flutter proxy)",
        ),
        field(
            "valve_leakage",
            "0 healthy .. 1 servo valve fully worn (continuous internal bypass)",
            "servo valve internal leakage",
            format!("{model_path}::ActuatorFaults.valve_leakage"),
            "0 healthy .. 1 fully worn",
            "rate ceiling falls (bypassed flow does no work on the piston) and the actuator's holding stiffness softens, so it creeps/droops under a sustained external load between position-loop corrections",
        ),
        field(
            "piston_seal_wear",
            "0 healthy .. 1 seal fully worn (continuous bore-to-rod-side bypass)",
            "piston seal wear",
            format!("{model_path}::ActuatorFaults.piston_seal_wear"),
            "0 healthy .. 1 fully worn",
            "force ceiling falls (some delivered flow recirculates across the piston instead of displacing it against load) and, like valve leakage, softens holding stiffness",
        ),
        field(
            "position_transducer_drift",
            "0 healthy .. 1 drifting at the modelled maximum rate",
            "position transducer drift",
            format!("sensors::PositionTransducer feeding {model_path}: TransducerFaults.drift"),
            "0 healthy .. 1 drifting at the modelled maximum rate",
            "the computer's own position monitoring (separate from the actuator's internal servo feedback) slowly diverges from truth, eventually tripping a dual-channel disagreement",
        ),
        field(
            "position_transducer_open",
            "0 healthy .. 1 fully open circuit (no signal)",
            "position transducer open circuit",
            format!("sensors::PositionTransducer feeding {model_path}: TransducerFaults.open_circuit"),
            "0 healthy .. 1 fully open",
            "that channel's signal is lost (rails to zero, not a frozen value); the computer must rely on the other channel alone",
        ),
        field(
            "position_transducer_intermittent",
            "0 none .. 1 signal dropped essentially all the time",
            "position transducer intermittent connection",
            format!("sensors::PositionTransducer feeding {model_path}: TransducerFaults.intermittent"),
            "0 none .. 1 signal dropped essentially all the time",
            "the channel's signal cuts in and out, which a monitor sees as repeated brief losses rather than one clean failure",
        ),
    ]
}

fn fault_var(comp_id: &str) -> String {
    let bare = comp_id.split('.').next_back().unwrap_or(comp_id);
    format!("FCTL_{}_FAULT", bare.to_uppercase())
}

pub fn register(r: &mut Registry) {
    let mut ids = Ids::new();

    let ailerons = [
        ("27_fctl.ail_l1", "Left outward aileron"),
        ("27_fctl.ail_l2", "Left middle aileron"),
        ("27_fctl.ail_l3", "Left inward aileron"),
        ("27_fctl.ail_r1", "Right outward aileron"),
        ("27_fctl.ail_r2", "Right middle aileron"),
        ("27_fctl.ail_r3", "Right inward aileron"),
    ];
    let mut aileron_ids = Vec::new();
    for (id, name) in ailerons {
        aileron_ids.extend(register_component(r, &mut ids, id, name, surface_fields("flight_controls::surface::ControlSurface<2> (aileron)")));
    }

    let elevators = [
        ("27_fctl.elev_l_inbd", "Left inboard elevator"),
        ("27_fctl.elev_l_outbd", "Left outboard elevator"),
        ("27_fctl.elev_r_inbd", "Right inboard elevator"),
        ("27_fctl.elev_r_outbd", "Right outboard elevator"),
    ];
    let mut elevator_ids = Vec::new();
    for (id, name) in elevators {
        elevator_ids.extend(register_component(r, &mut ids, id, name, surface_fields("flight_controls::surface::ControlSurface<1> (elevator)")));
    }

    let rudders = [("27_fctl.rud_upper", "Upper rudder"), ("27_fctl.rud_lower", "Lower rudder")];
    let mut rudder_ids = Vec::new();
    for (id, name) in rudders {
        rudder_ids.extend(register_component(r, &mut ids, id, name, surface_fields("flight_controls::surface::ControlSurface<2> (rudder)")));
    }

    let mut spoiler_ids = Vec::new();
    let mut spoiler_comp_ids = Vec::new();
    for side in ["L", "R"] {
        let side_name = if side == "L" { "Left" } else { "Right" };
        for n in 1..=8u16 {
            let id = format!("27_fctl.splr_{}{n}", side.to_lowercase());
            let name = format!("{side_name} spoiler {n}");
            spoiler_ids.extend(register_component(r, &mut ids, &id, &name, surface_fields("flight_controls::surface::ControlSurface<1> (spoiler)")));
            spoiler_comp_ids.push(id);
        }
    }

    let ths_fields = vec![
        field(
            "motor_green_supply_loss",
            "0 full green hydraulic supply to the green THS motor .. 1 none",
            "green motor failure",
            "flight_controls::ths::ThsFaults.motor_green (actuator::ActuatorFaults.supply_loss)".to_string(),
            "0 healthy .. 1 no hydraulic drive from the green motor",
            "trim rate roughly halves; the yellow motor alone still trims and, with a healthy no-back, still holds",
        ),
        field(
            "motor_yellow_supply_loss",
            "0 full yellow hydraulic supply to the yellow THS motor .. 1 none",
            "yellow motor failure",
            "flight_controls::ths::ThsFaults.motor_yellow (actuator::ActuatorFaults.supply_loss)".to_string(),
            "0 healthy .. 1 no hydraulic drive from the yellow motor",
            "trim rate roughly halves; the green motor alone still trims and, with a healthy no-back, still holds",
        ),
        field(
            "no_back_failure",
            "0 healthy (holds against airload between trim inputs) .. 1 no holding torque left",
            "no-back brake failure",
            "flight_controls::ths::ThsFaults.no_back_failure".to_string(),
            "0 healthy .. 1 no holding torque left",
            "between trim inputs the stabiliser can be back-driven by its own aerodynamic hinge moment, drifting off the commanded trim -- the failure mode behind the FAA's AD 2000-15-15 jackscrew/nut inspections on another type's THS",
        ),
        field(
            "ballscrew_jam",
            "0 free .. 1 screw/nut seized",
            "ballscrew jam",
            "flight_controls::ths::ThsFaults.ballscrew_jam".to_string(),
            "0 free .. 1 fully seized at the angle it jammed",
            "trim freezes at the jam angle regardless of motor command, resistible only by a large enough external torque",
        ),
        field(
            "position_transducer_drift",
            "0 healthy .. 1 drifting at the modelled maximum rate",
            "position transducer drift",
            "sensors::PositionTransducer feeding flight_controls::ths::TrimmableHorizontalStabilizer: TransducerFaults.drift".to_string(),
            "0 healthy .. 1 drifting at the modelled maximum rate",
            "trim position monitoring slowly diverges from truth -- particularly consequential for THS, since runaway detection depends on trustworthy position feedback",
        ),
        field(
            "position_transducer_open",
            "0 healthy .. 1 fully open circuit",
            "position transducer open circuit",
            "sensors::PositionTransducer feeding flight_controls::ths::TrimmableHorizontalStabilizer: TransducerFaults.open_circuit".to_string(),
            "0 healthy .. 1 fully open",
            "that channel's signal is lost; the computer must rely on the other channel alone",
        ),
    ];
    let ths_ids = register_component(r, &mut ids, "27_fctl.ths", "Trimmable horizontal stabiliser", ths_fields);

    let rudder_trim_fields = vec![
        field(
            "motor_failure",
            "0 healthy .. 1 electric motor-pump dead",
            "motor failure",
            "flight_controls::ths::RudderTrimActuator (actuator::ElectricPumpFaults.motor_failure)".to_string(),
            "0 healthy .. 1 dead",
            "rudder trim can no longer be commanded; it holds its last value",
        ),
        field(
            "jam",
            "0 free .. 1 seized",
            "jam",
            "flight_controls::ths::RudderTrimActuator (actuator::ActuatorFaults.jam)".to_string(),
            "0 free .. 1 fully seized",
            "rudder trim freezes at its jammed value",
        ),
    ];
    let rudder_trim_ids = register_component(r, &mut ids, "27_fctl.rudder_trim", "Rudder trim actuator", rudder_trim_fields);

    fn high_lift_fields(model_path: &str) -> Vec<FieldSpec> {
        vec![
            field("pcu_jam", "0 free .. 1 PCU seized", "PCU jam", format!("{model_path}::HighLiftFaults.pcu (actuator::ActuatorFaults.jam)"), "0 free .. 1 seized", "the drive line freezes at the PCU's own shaft angle"),
            field("pcu_runaway", "0 none .. 1 servo hardover", "PCU hardover", format!("{model_path}::HighLiftFaults.pcu (actuator::ActuatorFaults.runaway)"), "0 none .. 1 full-rate uncommanded drive", "uncommanded motion until the wingtip brake, a shaft break, or the torque limiter's own authority stops it"),
            field("pcu_supply_loss", "0 full supply .. 1 none", "PCU supply loss", format!("{model_path}::HighLiftFaults.pcu (actuator::ActuatorFaults.supply_loss)"), "0 full supply .. 1 none", "drive torque and rate both fall toward zero"),
            field("limiter_bypass", "0 trips at its design threshold .. 1 seized/bypassed, never trips", "torque limiter failure", format!("{model_path}::HighLiftFaults.limiter_bypass"), "0 healthy .. 1 bypassed", "a downstream jam or overload transmits full PCU torque straight into the shaft the limiter exists to protect, instead of being capped"),
            field("inboard_shaft_break", "0 intact .. 1 PCU-to-inboard segment severed", "inboard shaft break", format!("{model_path}::HighLiftFaults.inboard_shaft_break"), "0 intact .. 1 severed", "the inboard and outboard stations both lose drive, free-floating under airload"),
            field("outboard_shaft_break", "0 intact .. 1 inboard-to-outboard segment severed", "outboard shaft break", format!("{model_path}::HighLiftFaults.outboard_shaft_break"), "0 intact .. 1 severed", "only the outboard station loses drive; the inboard station still tracks"),
            field("wingtip_brake_fail", "0 healthy .. 1 no holding torque when commanded", "wingtip brake failure", format!("{model_path}::HighLiftFaults.wingtip_brake_fail"), "0 healthy .. 1 no holding torque", "an asymmetry or overspeed condition can no longer be contained mechanically"),
        ]
    }
    let high_lift = [
        ("27_fctl.flap_l", "Left flap drive line"),
        ("27_fctl.flap_r", "Right flap drive line"),
        ("27_fctl.slat_l", "Left outboard slat drive line"),
        ("27_fctl.slat_r", "Right outboard slat drive line"),
        ("27_fctl.droop_l", "Left inboard droop-nose drive line"),
        ("27_fctl.droop_r", "Right inboard droop-nose drive line"),
    ];
    let mut high_lift_ids = Vec::new();
    for (id, name) in high_lift {
        let model_path = if id.contains("flap") {
            "flight_controls::high_lift::HighLiftSystem (flap)"
        } else if id.contains("slat") {
            "flight_controls::high_lift::HighLiftSystem (slat)"
        } else {
            "flight_controls::high_lift::HighLiftSystem (droop nose)"
        };
        high_lift_ids.extend(register_component(r, &mut ids, id, name, high_lift_fields(model_path)));
    }

    let ground_spoiler_fields = vec![
        field(
            "fails_to_deploy",
            "0 healthy .. 1 never commands deployment even though armed and touchdown/spin-up conditions are met",
            "logic fails to deploy",
            "flight_controls::spoiler::GroundSpoilerLogicFaults.fails_to_deploy".to_string(),
            "0 healthy .. 1 never deploys",
            "loss of lift dump and reduced wheel braking effectiveness on landing",
        ),
        field(
            "fails_to_retract",
            "0 healthy .. 1 never commands retraction on a go-around",
            "logic fails to retract",
            "flight_controls::spoiler::GroundSpoilerLogicFaults.fails_to_retract".to_string(),
            "0 healthy .. 1 stuck deployed",
            "reduced lift and increased drag through a go-around, exactly when climb performance matters most",
        ),
    ];
    let ground_spoiler_ids = register_component(r, &mut ids, "27_fctl.gnd_splr_logic", "Ground spoiler deploy/retract logic", ground_spoiler_fields);

    for (n, letter) in [(1, "A"), (2, "B"), (3, "C")] {
        let prim_fields = vec![field(
            "ra_link_fault",
            "0 healthy (uses all three RA systems) .. 1 (drops its own RA link)",
            &format!("PRIM {n} RA {letter} link fault"),
            format!("flight_controls::prim::Prim{n}.ra_link_fault"),
            "boolean: 0 healthy, >0 faulted -- a PRIM either has a working link to that RA system or it does not",
            &format!("PRIM {n} stops using RA system {letter}, per 340800056/057/058"),
        )];
        register_component(r, &mut ids, &format!("27_fctl.prim_{n}"), &format!("PRIM {n} (RA link health only, no control-law model)"), prim_fields);
    }
    fn input_sensor_fields(model_path: &str) -> Vec<FieldSpec> {
        vec![
            field(
                "chan_a_open",
                "0 healthy .. 1 fully open circuit (no signal)",
                "channel A open circuit",
                format!("sensors::PositionTransducer feeding {model_path}: TransducerFaults.open_circuit (channel A)"),
                "0 healthy .. 1 fully open",
                "channel A's signal is lost (rails to zero, not a frozen value); the computer must rely on channel B alone",
            ),
            field(
                "chan_a_drift",
                "0 healthy .. 1 drifting at the modelled maximum rate",
                "channel A drift",
                format!("sensors::PositionTransducer feeding {model_path}: TransducerFaults.drift (channel A)"),
                "0 healthy .. 1 drifting at the modelled maximum rate",
                "channel A slowly diverges from channel B, eventually tripping the dual-channel disagreement monitor",
            ),
            field(
                "chan_b_open",
                "0 healthy .. 1 fully open circuit (no signal)",
                "channel B open circuit",
                format!("sensors::PositionTransducer feeding {model_path}: TransducerFaults.open_circuit (channel B)"),
                "0 healthy .. 1 fully open",
                "channel B's signal is lost; the computer must rely on channel A alone",
            ),
            field(
                "chan_b_drift",
                "0 healthy .. 1 drifting at the modelled maximum rate",
                "channel B drift",
                format!("sensors::PositionTransducer feeding {model_path}: TransducerFaults.drift (channel B)"),
                "0 healthy .. 1 drifting at the modelled maximum rate",
                "channel B slowly diverges from channel A, eventually tripping the dual-channel disagreement monitor",
            ),
        ]
    }
    let input_sensors = [
        ("27_fctl.l_sidestick_pitch", "Captain's sidestick pitch transducer"),
        ("27_fctl.l_sidestick_roll", "Captain's sidestick roll transducer"),
        ("27_fctl.rudder_pedal", "Rudder pedal position transducer"),
    ];
    for (id, name) in input_sensors {
        register_component(r, &mut ids, id, name, input_sensor_fields("flight_controls::live::FlightControlsLive (cockpit input transducer)"));
    }

    let pin_prog_fields = |unit: &str| {
        vec![field(
            "mismatch",
            "0 all three units' configured identity tags agree .. 1 one unit's overwritten",
            "configuration identity mismatch",
            format!("flight_controls::live::FlightControlsLive (new {unit} pin-programming component, E-FCTL ECAM completeness pass)"),
            "boolean in practice: 0 agree, 1 disagree",
            "a maintenance-style fault (a unit swapped in with the wrong software standard or pin strap), not a flight-control-law degradation -- this port models no in-flight effect from a version/pin-prog mismatch alone, matching real Airbus practice (a despatch/maintenance item)",
        )]
    };
    let prim_pin_prog_ids = register_component(r, &mut ids, "27_fctl.prim_pin_prog", "PRIM software/pin-programming identity", pin_prog_fields("PRIM"));
    let sec_pin_prog_ids = register_component(r, &mut ids, "27_fctl.sec_pin_prog", "SEC software/pin-programming identity", pin_prog_fields("SEC"));
    debug_assert_eq!(prim_pin_prog_ids.len(), 1);
    debug_assert_eq!(sec_pin_prog_ids.len(), 1);

    let rate_gyros = [
        ("27_fctl.rate_gyro_pitch", "Pitch rate gyro pair feeding the flight control laws"),
        ("27_fctl.rate_gyro_roll", "Roll rate gyro pair feeding the flight control laws"),
        ("27_fctl.rate_gyro_yaw", "Yaw rate gyro pair feeding the flight control laws"),
    ];
    for (id, name) in rate_gyros {
        register_component(r, &mut ids, id, name, input_sensor_fields("flight_controls::live::FlightControlsLive (rate gyro pair, fed from physics::adirs's real sensed body rate)"));
    }

    let r_sidesticks = [
        ("27_fctl.r_sidestick_pitch", "F.O.'s sidestick pitch transducer"),
        ("27_fctl.r_sidestick_roll", "F.O.'s sidestick roll transducer"),
    ];
    for (id, name) in r_sidesticks {
        register_component(r, &mut ids, id, name, input_sensor_fields("flight_controls::live::FlightControlsLive (cockpit input transducer, F.O. side, held at a fixed neutral position)"));
    }

    fn prim_channel_field(kind: &str) -> Vec<FieldSpec> {
        vec![field(
            "channel_fault",
            "0 healthy .. 1 that PRIM's own command/monitoring channel failed",
            &format!("{kind} channel fault"),
            format!("flight_controls::live::FlightControlsLive (new per-PRIM {kind} channel component, E-FCTL coordinator follow-up)"),
            "boolean in practice: 0 healthy, 1 failed",
            "that PRIM alone loses this one function while its overall health (and every other PRIM) is unaffected -- gated on Truth::prim_healthy so the whole-unit failure is not double counted",
        )]
    }
    for n in 1..=3u16 {
        register_component(r, &mut ids, &format!("27_fctl.prim_{n}_elevator_channel"), &format!("PRIM {n} elevator command channel"), prim_channel_field("elevator command"));
        register_component(r, &mut ids, &format!("27_fctl.prim_{n}_rudder_channel"), &format!("PRIM {n} rudder command channel"), prim_channel_field("rudder command"));
        register_component(r, &mut ids, &format!("27_fctl.prim_{n}_sidestick_monitor"), &format!("PRIM {n} sidestick-sensor monitor"), prim_channel_field("sidestick-sensor monitor"));
    }

    let laf_fields = vec![
        field("left_accel_1_fail", "0 healthy .. 1 failed", "left wing accelerometer 1 failure", "flight_controls::live::FlightControlsLive (new load-alleviation-function component, E-FCTL coordinator follow-up)".to_string(), "boolean in practice: 0 healthy, 1 failed", "one of the left wing's three LAF accelerometers reads invalid; the 2-of-3 vote still passes alone"),
        field("left_accel_2_fail", "0 healthy .. 1 failed", "left wing accelerometer 2 failure", "flight_controls::live::FlightControlsLive (same component)".to_string(), "boolean in practice: 0 healthy, 1 failed", "same as accelerometer 1, left wing"),
        field("left_accel_3_fail", "0 healthy .. 1 failed", "left wing accelerometer 3 failure", "flight_controls::live::FlightControlsLive (same component)".to_string(), "boolean in practice: 0 healthy, 1 failed", "same as accelerometer 1, left wing"),
        field("right_accel_1_fail", "0 healthy .. 1 failed", "right wing accelerometer 1 failure", "flight_controls::live::FlightControlsLive (same component)".to_string(), "boolean in practice: 0 healthy, 1 failed", "one of the right wing's three LAF accelerometers reads invalid; the 2-of-3 vote still passes alone"),
        field("right_accel_2_fail", "0 healthy .. 1 failed", "right wing accelerometer 2 failure", "flight_controls::live::FlightControlsLive (same component)".to_string(), "boolean in practice: 0 healthy, 1 failed", "same as accelerometer 1, right wing"),
        field("right_accel_3_fail", "0 healthy .. 1 failed", "right wing accelerometer 3 failure", "flight_controls::live::FlightControlsLive (same component)".to_string(), "boolean in practice: 0 healthy, 1 failed", "same as accelerometer 1, right wing"),
    ];
    register_component(r, &mut ids, "27_fctl.load_alleviation", "Load alleviation function accelerometers", laf_fields);

    let flap_lever_fields = vec![
        field("chan_1_comm_lost", "0 healthy .. 1 communication with SFCC 1 lost", "SFCC 1 communication loss", "flight_controls::live::FlightControlsLive (new flap-lever-CSU component, E-FCTL)".to_string(), "boolean in practice: 0 healthy, 1 lost", "SFCC 1 can no longer read the flap lever's own position"),
        field("chan_2_comm_lost", "0 healthy .. 1 communication with SFCC 2 lost", "SFCC 2 communication loss", "flight_controls::live::FlightControlsLive (same component)".to_string(), "boolean in practice: 0 healthy, 1 lost", "SFCC 2 can no longer read the flap lever's own position"),
    ];
    register_component(r, &mut ids, "27_fctl.flap_lever_csu", "Flap lever CSU communication", flap_lever_fields);

    fn sec_direct_law_channel_fields(n: u16) -> Vec<FieldSpec> {
        vec![field(
            "channel_fault",
            "0 healthy .. 1 that SEC's own direct-law command channel failed",
            "direct-law channel fault",
            format!("flight_controls::live::FlightControlsLive (new SEC {n} direct-law channel component, E-FCTL coordinator follow-up)"),
            "boolean in practice: 0 healthy, 1 failed",
            "that SEC alone loses the Direct law it computes for ailerons, spoilers, elevators, THS and rudders (FCOM DSC-27-10-10 FLIGHT CONTROL SYSTEM - SYSTEM DESCRIPTION, SEC)",
        )]
    }
    for n in 1..=3u16 {
        register_component(r, &mut ids, &format!("27_fctl.sec_{n}_direct_law_channel"), &format!("SEC {n} direct-law command channel"), sec_direct_law_channel_fields(n));
    }

    let fcdc_fields = |n: u16| {
        vec![field(
            "comm_fault",
            "0 healthy .. 1 that FCDC stops acquiring PRIM/SEC data",
            "data concentrator fault",
            format!("flight_controls::live::FlightControlsLive (new FCDC {n} data-concentrator component, E-FCTL coordinator follow-up)"),
            "boolean in practice: 0 healthy, 1 failed",
            "that FCDC alone stops acquiring PRIM/SEC data for the F/CTL SD page, the Flight Warning System, the on-board maintenance system and the flight data recorders (FCOM DSC-27-10-10 FLIGHT CONTROL SYSTEM - SYSTEM DESCRIPTION, FLIGHT CONTROL DATA CONCENTRATOR); it hosts no control law, so no other aircraft system is affected",
        )]
    };
    for n in 1..=2u16 {
        register_component(r, &mut ids, &format!("27_fctl.fcdc_{n}"), &format!("FCDC {n} (data concentrator, no control-law model)"), fcdc_fields(n));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registering_everything_produces_no_validation_errors() {
        let mut r = Registry::default();
        register(&mut r);
        let errors = r.validate_area();
        assert!(errors.is_empty(), "{errors:?}");
    }

    #[test]
    fn every_a380_surface_family_is_represented_individually() {
        let mut r = Registry::default();
        register(&mut r);
        let ids: Vec<&str> = r.components.iter().map(|c| c.id.as_str()).collect();
        for expected in [
            "27_fctl.ail_l1",
            "27_fctl.ail_r3",
            "27_fctl.elev_l_outbd",
            "27_fctl.rud_upper",
            "27_fctl.rud_lower",
            "27_fctl.splr_l1",
            "27_fctl.splr_r8",
            "27_fctl.ths",
            "27_fctl.rudder_trim",
            "27_fctl.flap_l",
            "27_fctl.slat_r",
            "27_fctl.droop_l",
            "27_fctl.droop_r",
            "27_fctl.gnd_splr_logic",
            "27_fctl.l_sidestick_pitch",
            "27_fctl.l_sidestick_roll",
            "27_fctl.rudder_pedal",
            "27_fctl.prim_pin_prog",
            "27_fctl.sec_pin_prog",
            "27_fctl.rate_gyro_pitch",
            "27_fctl.rate_gyro_roll",
            "27_fctl.rate_gyro_yaw",
            "27_fctl.r_sidestick_pitch",
            "27_fctl.r_sidestick_roll",
            "27_fctl.prim_1_elevator_channel",
            "27_fctl.prim_3_rudder_channel",
            "27_fctl.prim_2_sidestick_monitor",
            "27_fctl.load_alleviation",
            "27_fctl.flap_lever_csu",
            "27_fctl.sec_1_direct_law_channel",
            "27_fctl.sec_3_direct_law_channel",
            "27_fctl.fcdc_1",
            "27_fctl.fcdc_2",
        ] {
            assert!(ids.contains(&expected), "missing component {expected}");
        }
        assert_eq!(r.components.len(), 66);
    }

    #[test]
    fn ids_are_unique_and_carry_the_flight_controls_area_and_ata() {
        let mut r = Registry::default();
        register(&mut r);
        let mut seen = std::collections::HashSet::new();
        for f in &r.failures {
            assert!(seen.insert(f.id), "duplicate id {}", f.id);
            assert_eq!(f.ata, 27);
            assert_eq!(f.id / 1_000_000, Area::FlightControls as u64);
        }
    }
}
