//! Registers every flight-control-surface component, failure and ECAM alert
//! from this directory into the shared `crate::deep::api::Registry`, per
//! `docs/deep/BRIEF.md`'s "Registering failures, components and ECAM
//! alerts". Area `FlightControls`, ATA 27, every physical surface
//! registered individually (six ailerons, four elevators, two rudders,
//! sixteen spoilers, the THS, the rudder trim actuator, and the four
//! flap/slat high-lift drive lines) rather than lumped into one generic
//! entry per surface type.
//!
//! Trigger variables: this area does not yet publish live health variables
//! to the plugin's variable registry (this directory is still
//! self-contained, per the shared brief's hard rule 2), so each component
//! is given one planned aggregate variable, `FCTL_<COMPONENT>_FAULT` (0..1,
//! the worst of that component's active fault magnitudes), documented here
//! and in `PROGRESS.md` as the new variable this area must publish once
//! wired into the plugin proper.

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

/// One failure field this crate's fault structs carry: the health
/// parameter's own name/meaning (for the component), and the failure's
/// title suffix/model field/magnitude/effect (for the failure entry).
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

/// Registers one component and all of its failure fields, returning the
/// failure ids in the same order as `fields`.
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

/// The six failure fields every servo-hydraulic surface in `actuator.rs` /
/// `surface.rs` carries (`actuator::ActuatorFaults` plus
/// `surface::SurfaceFaults`), parameterised by the concrete model path so
/// the same list serves ailerons, elevators, rudders and spoilers alike.
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

    // ---- Ailerons: 3 panels x 2 sides, each panel one `ControlSurface<2>`
    // (an outward-mounted always-hydraulic actuator plus an inward-mounted
    // one that is EHA-capable on the middle/inward panels,
    // a380_systems/src/hydraulic/mod.rs:394-441, 499-501).
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

    // ---- Elevators: 2 panels x 2 sides, one actuator each.
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

    // ---- Rudders: upper/lower, each two EBHA-capable actuators.
    let rudders = [("27_fctl.rud_upper", "Upper rudder"), ("27_fctl.rud_lower", "Lower rudder")];
    let mut rudder_ids = Vec::new();
    for (id, name) in rudders {
        rudder_ids.extend(register_component(r, &mut ids, id, name, surface_fields("flight_controls::surface::ControlSurface<2> (rudder)")));
    }

    // ---- Spoilers: 8 per side, one actuator each (spoiler 6 each side is
    // EBHA-capable, a380_systems/src/hydraulic/mod.rs:574,608-618,665-666).
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

    // ---- THS: two hydraulic motors, a no-back brake, a ballscrew.
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

    // ---- Rudder trim actuator.
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

    // ---- High-lift: flap, slat and droop-nose drive lines, one per wing
    // per system. The A380 uses conventional slotted slats outboard and a
    // "droop nose" (a single-hinge rotating leading edge, no track or slot)
    // inboard instead of a slat there -- a public Airbus/A380 design
    // choice, not modelled anywhere in `a380_systems` (that crate only has
    // the FBW A320-family-style slat everywhere); `high_lift::HighLiftSystem::{new_flap,
    // new_slat, new_droop_nose}` give each its own GENERIC drive sizing.
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

    // ---- Ground spoiler auto-deploy/retract logic (shared across all
    // ground-spoiler-capable panels, not per panel: this is the arming/
    // sequencing electronics, not any one actuator).
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

    // ---- E-ELEC Phase 2 (2026-09-27): `340800056`-`058 NAV RA SYS A(B)(C)
    // LOST BY PRIM` -- a minimal PRIM computer-health flag, no control-law
    // model. The PRIMs are real, redundant consumers of the three radio
    // altimeter systems; this area models surfaces/actuators/jams but has
    // no PRIM component at all today, which is exactly the gap
    // `E-ELEC-DESIGN.md`'s Group E designs: three new failure ids, boolean
    // (no numeric threshold -- a computer is using a given RA system or it
    // is not), one PRIM own health fault dropping that PRIM's own use of
    // one RA system (PRIM 1 <-> RA A, PRIM 2 <-> RA B, PRIM 3 <-> RA C --
    // the simplest sourced mapping without inventing a fuller
    // redundancy-management matrix this port does not otherwise model).
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
    // ---- Cockpit input transducers (E-FCTL, ECAM completeness pass,
    // `E-FCTL-DESIGN.md` section 3.2): the captain's sidestick pitch/roll
    // axis and the rudder pedal axis, each modelled as the same
    // `sensors::DualTransducer` two-channel shape as every surface's own
    // position monitor above, applied to the raw command input instead of a
    // surface position. Unlike a surface's monitor (registry.rs's existing
    // `surface_fields`, which only arms channel A -- channel B is always
    // healthy there), both channels here get their own failure fields,
    // because "the whole stick/pedal is lost" needs both channels failable
    // independently. There is no F.O.-side sidestick component: this port
    // has no live, independently-driven F.O. stick axis to model a
    // transducer against (`Truth::capt_sidestick_pitch_raw`'s own doc), so
    // `271800026`/`271800028` stay unsourced rather than watching a value
    // nothing ever moves.
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

    // ---- PRIM/SEC software/pin-programming identity (E-FCTL, ECAM
    // completeness pass, `E-FCTL-DESIGN.md` section 3.3): a discrete
    // agree/disagree check across the three configured units, not a
    // continuous quantity, so "never invent a threshold" does not apply --
    // the comparison is binary. Genuinely new modelling (this port's own
    // compiled PRIM/SEC logic carries no per-unit version/pin-prog identity
    // to disagree, confirmed by grepping every `A380{Prim,Sec}Computer*_
    // types.h` for "version"/"standard"/"part_number"/"identific": no hits),
    // built on the same kind of config-pin concept this port already uses
    // elsewhere (`BaseFcuDiscreteInputs::pin_prog_qfe_avail`).
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

    // ---- Rate gyros (E-FCTL, coordinator follow-up 2026-09-27,
    // `271800018` F/CTL TWO GYROMETERs FAULT): the coordinator asked
    // whether a real body-rate quantity already exists in this port rather
    // than leaving this unsourced for lack of a live gyro input. It does:
    // `src/physics/adirs.rs` runs a real strapdown-IRS simulation per
    // ADIRU, with its own gyro bias/failure model, and writes its *sensed*
    // (not X-Plane-truth) body rate back over the native `BODY_ROTATION_
    // RATE_X/Y/Z` datarefs (`adirs.rs:1502-1504`) -- the same datarefs
    // `SimReadings::body_rotation_velocity_rad_s` reads
    // (`prim.rs:64-90`'s own doc), which is in turn what the real compiled
    // PRIM/SEC flight-control laws consume as their own rate input. So
    // `Truth::body_rate_{pitch,roll,yaw}_raw` (fed from that same read, see
    // `prim.rs`'s new publish) is a real, already-modelled, already-
    // failable quantity, not an invented one. FlyByWire's own compiled
    // Fctl bus additionally carries six discrete `rate_gyro_*_bus` fields
    // (`A380PrimComputerFctl_types.h:781-786`) that would need extending
    // `BasePrimFctlLogicOutputs`'s exact C ABI layout for a purely cosmetic
    // gain (the value is already the same one this model uses); this pass
    // reads the underlying real quantity through the sensors this port
    // already runs instead.
    let rate_gyros = [
        ("27_fctl.rate_gyro_pitch", "Pitch rate gyro pair feeding the flight control laws"),
        ("27_fctl.rate_gyro_roll", "Roll rate gyro pair feeding the flight control laws"),
        ("27_fctl.rate_gyro_yaw", "Yaw rate gyro pair feeding the flight control laws"),
    ];
    for (id, name) in rate_gyros {
        register_component(r, &mut ids, id, name, input_sensor_fields("flight_controls::live::FlightControlsLive (rate gyro pair, fed from physics::adirs's real sensed body rate)"));
    }

    // ---- F.O. sidestick (E-FCTL, coordinator follow-up, `271800026`/
    // `271800028`): the coordinator's point stands -- the F.O. sidestick is
    // a real physical transducer pair in the real aircraft even though this
    // port's cockpit has no independent input device moving it. Modelling
    // it exactly like the captain's (section 3.2) but against a fixed
    // neutral "true angle" is honest: the FCOM's own triggering text for
    // both the generic and the captain's-side procedure ("The left (right)
    // sidestick is failed" / "Two, or three PRIMs detect that one sidestick
    // sensor is failed", `E-FCTL-FCOM.json` ids 271800026/271800028) never
    // requires the stick to be *moved*, only that its transducer(s) are
    // failed or disagree -- exactly what a transducer sitting at a real,
    // physical rest position can still do.
    let r_sidesticks = [
        ("27_fctl.r_sidestick_pitch", "F.O.'s sidestick pitch transducer"),
        ("27_fctl.r_sidestick_roll", "F.O.'s sidestick roll transducer"),
    ];
    for (id, name) in r_sidesticks {
        register_component(r, &mut ids, id, name, input_sensor_fields("flight_controls::live::FlightControlsLive (cockpit input transducer, F.O. side, held at a fixed neutral position)"));
    }

    // ---- Per-PRIM elevator/rudder command channel and sidestick-sensor
    // monitor (E-FCTL, coordinator follow-up, `271800033`-`271800035`/
    // `271800039`-`271800041`/`271800042`-`271800044`). Reading the FCOM's
    // own triggering text (`E-FCTL-FCOM.json`): "PRIM 1(2)(3) has lost the
    // capacity to control an elevator/rudder actuator" and "PRIM 1(2)(3)
    // detects that one sidestick sensor is failed" -- both describe a
    // fault *internal to that one PRIM's own command or monitoring
    // circuitry*, not "which hydraulic system backs the actuator" (the
    // quantity `A380PrimComputerFctl.cpp`'s `elevator_n_avail`/rudder-mode-
    // avail bits actually carry, confirmed the first pass through this
    // chapter -- see `ata27.rs`'s module doc). FlyByWire's compiled model
    // does not expose a sub-unit "this PRIM's own elevator/rudder output
    // stage" or "this PRIM's own sidestick input monitor" health bit
    // (grepped `A380PrimComputer*_types.h` again for anything narrower than
    // whole-unit `prim_healthy`/the actuator-wide avail bits: none), so this
    // is genuinely new modelling -- a real, plausible LRU failure mode (an
    // internal output driver or input-monitoring stage failing while the
    // rest of the unit stays healthy) built as its own component per the
    // user's "model missing causes as real components" rule, gated on that
    // PRIM's own overall health (`Truth::prim_healthy`) so the whole-unit
    // failure (already covered by FlyByWire's own wired `271800036`-`038`)
    // is never double-counted.
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

    // ---- Load alleviation function (E-FCTL, coordinator follow-up,
    // `271800029`): the FCOM's own triggering text (`E-FCTL-FCOM.json`)
    // gives a real, sourced, discrete condition -- "The load alleviation
    // function (LAF) is failed... Two out of three accelerometers used for
    // the LAF are failed in one wing" -- which this pass models literally:
    // three accelerometers per wing (six total), 2-of-3 voting per wing.
    // This does not require modelling the aerodynamic envelope the function
    // itself operates in (the earlier objection): the FCOM's own FAULT
    // condition is the accelerometer-voting failure, not "the function
    // failed to engage when it should have".
    let laf_fields = vec![
        field("left_accel_1_fail", "0 healthy .. 1 failed", "left wing accelerometer 1 failure", "flight_controls::live::FlightControlsLive (new load-alleviation-function component, E-FCTL coordinator follow-up)".to_string(), "boolean in practice: 0 healthy, 1 failed", "one of the left wing's three LAF accelerometers reads invalid; the 2-of-3 vote still passes alone"),
        field("left_accel_2_fail", "0 healthy .. 1 failed", "left wing accelerometer 2 failure", "flight_controls::live::FlightControlsLive (same component)".to_string(), "boolean in practice: 0 healthy, 1 failed", "same as accelerometer 1, left wing"),
        field("left_accel_3_fail", "0 healthy .. 1 failed", "left wing accelerometer 3 failure", "flight_controls::live::FlightControlsLive (same component)".to_string(), "boolean in practice: 0 healthy, 1 failed", "same as accelerometer 1, left wing"),
        field("right_accel_1_fail", "0 healthy .. 1 failed", "right wing accelerometer 1 failure", "flight_controls::live::FlightControlsLive (same component)".to_string(), "boolean in practice: 0 healthy, 1 failed", "one of the right wing's three LAF accelerometers reads invalid; the 2-of-3 vote still passes alone"),
        field("right_accel_2_fail", "0 healthy .. 1 failed", "right wing accelerometer 2 failure", "flight_controls::live::FlightControlsLive (same component)".to_string(), "boolean in practice: 0 healthy, 1 failed", "same as accelerometer 1, right wing"),
        field("right_accel_3_fail", "0 healthy .. 1 failed", "right wing accelerometer 3 failure", "flight_controls::live::FlightControlsLive (same component)".to_string(), "boolean in practice: 0 healthy, 1 failed", "same as accelerometer 1, right wing"),
    ];
    register_component(r, &mut ids, "27_fctl.load_alleviation", "Load alleviation function accelerometers", laf_fields);

    // ---- Flap lever CSU communication (E-FCTL, `272800014`/`272800015`
    // F/CTL FLAPS LEVER SYS 1/2 FAULT): FCOM PRO-ABN-ECAM p.5112
    // (`E-FCTL-FCOM.json`) gives the real condition literally --
    // "Communication between the FLAPS lever and SFCC 1(2) is lost" -- a
    // discrete per-channel comm-loss fault, not a position-accuracy one.
    // `272800013` (FLAPS LEVER OUT OF DETENT) is deliberately not built on
    // this component: FlyByWire's own `0.18°`/`6.69°` SFCC thresholds
    // (section 3.4) apply to a continuous lever *angle* this port's cockpit
    // input does not have (`Truth::flap_lever_handle_index` is a discrete
    // detent index, always exactly on a detent, by this port's own design)
    // -- see the UNSOURCED note against that id rather than inventing an
    // index-to-degree conversion factor nothing sources.
    let flap_lever_fields = vec![
        field("chan_1_comm_lost", "0 healthy .. 1 communication with SFCC 1 lost", "SFCC 1 communication loss", "flight_controls::live::FlightControlsLive (new flap-lever-CSU component, E-FCTL)".to_string(), "boolean in practice: 0 healthy, 1 lost", "SFCC 1 can no longer read the flap lever's own position"),
        field("chan_2_comm_lost", "0 healthy .. 1 communication with SFCC 2 lost", "SFCC 2 communication loss", "flight_controls::live::FlightControlsLive (same component)".to_string(), "boolean in practice: 0 healthy, 1 lost", "SFCC 2 can no longer read the flap lever's own position"),
    ];
    register_component(r, &mut ids, "27_fctl.flap_lever_csu", "Flap lever CSU communication", flap_lever_fields);

    register_ecam(
        r,
        &ailerons,
        &aileron_ids,
        &elevators,
        &elevator_ids,
        &rudders,
        &rudder_ids,
        &spoiler_comp_ids,
        &spoiler_ids,
        &ths_ids,
        &rudder_trim_ids,
        &high_lift,
        &high_lift_ids,
        &ground_spoiler_ids,
    );
}

#[allow(clippy::too_many_arguments)]
fn register_ecam(
    r: &mut Registry,
    ailerons: &[(&str, &str)],
    aileron_ids: &[u64],
    elevators: &[(&str, &str)],
    elevator_ids: &[u64],
    rudders: &[(&str, &str)],
    rudder_ids: &[u64],
    spoiler_comp_ids: &[String],
    spoiler_ids: &[u64],
    ths_ids: &[u64],
    rudder_trim_ids: &[u64],
    high_lift: &[(&str, &str)],
    high_lift_ids: &[u64],
    ground_spoiler_ids: &[u64],
) {
    let ail_comp_ids: Vec<&str> = ailerons.iter().map(|(id, _)| *id).collect();
    let elev_comp_ids: Vec<&str> = elevators.iter().map(|(id, _)| *id).collect();
    let rud_comp_ids: Vec<&str> = rudders.iter().map(|(id, _)| *id).collect();
    let hl_comp_ids: Vec<&str> = high_lift.iter().map(|(id, _)| *id).collect();
    let splr_comp_ids: Vec<&str> = spoiler_comp_ids.iter().map(|s| s.as_str()).collect();

    let any_fault = |comp_ids: &[&str]| any(comp_ids.iter().map(|&id| var(&fault_var(id)).gt(0.5)).collect());

    r.alert(
        EcamAlert::new("FCTL_AIL_FAULT", ATA, "F/CTL AIL FAULT", Level::Caution, any_fault(&ail_comp_ids))
            .confirm(1.0)
            .inhibit(&[Phase::LiftOff, Phase::Touchdown])
            .step(line("MONITOR ROLL CONTROL", "").colour("white"))
            .status_line("F/CTL AIL SYS FAULT")
            .raised_by(aileron_ids),
    );

    r.alert(
        EcamAlert::new("FCTL_SPLR_FAULT", ATA, "F/CTL SPLR FAULT", Level::Caution, any_fault(&splr_comp_ids))
            .confirm(1.0)
            .inhibit(&[Phase::LiftOff, Phase::Touchdown])
            .step(line("SPEED BRAKE", "DO NOT USE").colour("cyan"))
            .status_line("F/CTL SPLR SYS FAULT")
            .raised_by(spoiler_ids),
    );

    r.alert(
        EcamAlert::new("FCTL_ELEV_FAULT", ATA, "F/CTL ELEV FAULT", Level::Caution, any_fault(&elev_comp_ids))
            .confirm(1.0)
            .inhibit(&[Phase::LiftOff, Phase::Touchdown])
            .step(line("MONITOR PITCH CONTROL", "").colour("white"))
            .status_line("F/CTL ELEV SYS FAULT")
            .raised_by(elevator_ids),
    );

    r.alert(
        EcamAlert::new("FCTL_RUD_FAULT", ATA, "F/CTL RUD FAULT", Level::Caution, any_fault(&rud_comp_ids))
            .confirm(1.0)
            .inhibit(&[Phase::LiftOff, Phase::Touchdown])
            .step(line("MONITOR YAW CONTROL", "").colour("white"))
            .status_line("F/CTL RUD SYS FAULT")
            .raised_by(rudder_ids),
    );

    r.alert(
        EcamAlert::new(
            "FCTL_THS_RUNAWAY",
            ATA,
            "F/CTL THS RUNAWAY",
            Level::Warning,
            any(vec![var(&fault_var("27_fctl.ths")).gt(0.5)]),
        )
        .confirm(0.5)
        .inhibit(&[Phase::LiftOff, Phase::Touchdown])
        .step(line("PITCH TRIM (MAN/ELEC)", "DO NOT USE").colour("cyan"))
        .status_line("F/CTL THS FAULT")
        .inop_sys("PITCH TRIM")
        .raised_by(ths_ids),
    );

    r.alert(
        EcamAlert::new("FCTL_RUD_TRIM_FAULT", ATA, "F/CTL RUD TRIM FAULT", Level::Advisory, any(vec![var(&fault_var("27_fctl.rudder_trim")).gt(0.5)]))
            .confirm(1.0)
            .step(line("RUDDER TRIM", "CHECK").colour("cyan"))
            .raised_by(rudder_trim_ids),
    );

    // high_lift order is [flap_l, flap_r, slat_l, slat_r, droop_l, droop_r],
    // 7 fields each: flap 0..14, slat 14..28, droop nose 28..42.
    r.alert(
        EcamAlert::new("FCTL_FLAP_FAULT", ATA, "F/CTL FLAP FAULT", Level::Caution, any_fault(&[hl_comp_ids[0], hl_comp_ids[1]]))
            .confirm(1.0)
            .inhibit(&[Phase::LiftOff, Phase::Touchdown])
            .step(line("MONITOR FLAP POSITION", "").colour("white"))
            .status_line("F/CTL FLAP SYS FAULT")
            .raised_by(&high_lift_ids[0..14]),
    );

    r.alert(
        EcamAlert::new("FCTL_SLAT_FAULT", ATA, "F/CTL SLAT FAULT", Level::Caution, any_fault(&[hl_comp_ids[2], hl_comp_ids[3]]))
            .confirm(1.0)
            .inhibit(&[Phase::LiftOff, Phase::Touchdown])
            .step(line("MONITOR SLAT POSITION", "").colour("white"))
            .status_line("F/CTL SLAT SYS FAULT")
            .raised_by(&high_lift_ids[14..28]),
    );

    r.alert(
        EcamAlert::new("FCTL_DROOP_NOSE_FAULT", ATA, "F/CTL DROOP NOSE FAULT", Level::Caution, any_fault(&[hl_comp_ids[4], hl_comp_ids[5]]))
            .confirm(1.0)
            .inhibit(&[Phase::LiftOff, Phase::Touchdown])
            .step(line("MONITOR SLAT POSITION", "").colour("white"))
            .status_line("F/CTL DROOP NOSE SYS FAULT")
            .raised_by(&high_lift_ids[28..42]),
    );

    r.alert(
        EcamAlert::new(
            "FCTL_GND_SPLR_FAULT",
            ATA,
            "F/CTL GND SPLR SYS FAULT",
            Level::Caution,
            any(vec![var(&fault_var("27_fctl.gnd_splr_logic")).gt(0.5)]),
        )
        .confirm(1.0)
        .inhibit(&[Phase::LiftOff, Phase::Touchdown])
        .step(line("LDG DIST", "AFFECTED").colour("white"))
        .status_line("F/CTL GND SPLR FAULT")
        .raised_by(ground_spoiler_ids),
    );
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
            // E-FCTL, ECAM completeness pass (sections 3.2/3.3).
            "27_fctl.l_sidestick_pitch",
            "27_fctl.l_sidestick_roll",
            "27_fctl.rudder_pedal",
            "27_fctl.prim_pin_prog",
            "27_fctl.sec_pin_prog",
            // Coordinator follow-up, 2026-09-27.
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
        ] {
            assert!(ids.contains(&expected), "missing component {expected}");
        }
        // 6 ailerons + 4 elevators + 2 rudders + 16 spoilers + 1 THS + 1
        // rudder trim + 6 high-lift lines (flap/slat/droop nose x2 sides)
        // 37 real surface/actuator components, plus E-ELEC Phase 2's 3 PRIM
        // RA-link components (`27_fctl.prim_n`), plus E-FCTL's 21 (cockpit input
        // transducers, rate gyros, per-PRIM channels, load alleviation, flap
        // lever CSU) = 61.
        assert_eq!(r.components.len(), 61);
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
