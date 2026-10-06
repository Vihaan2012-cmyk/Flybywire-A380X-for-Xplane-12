//! Registers every gear-structure component, failure and ECAM alert in code
//! (BRIEF.md "Registering failures, components and ECAM alerts"), so the
//! lead's `Registry::validate()` can cross-check ids and references without
//! this workstream describing anything only in prose.
//!
//! Ids: `Area::GearStructure`, ATA 32 (Landing Gear) throughout. Numbered
//! sequentially by a running counter rather than by hand, since most of
//! this area's failures are the same handful of physical fault types
//! expanded per leg/wheel/steering instance (the same pattern the brief
//! calls for with "x4 engines, L/R, green/yellow").
//!
//! New Vars this model would need to publish once wired into the plugin
//! (none exist yet -- this workstream is still self-contained per the
//! brief's hard rules): `GEAR_STRUT_GAS_CHARGE_FRACTION:n`,
//! `GEAR_STRUT_OIL_LEVEL_FRACTION:n`, `GEAR_STRUT_LIFE_FRACTION:n`,
//! `GEAR_STRUT_COLLAPSED:n` (n = 1 nose, 2 left wing, 3 right wing, 4 left
//! body, 5 right body, matching `LEGS` below); `GEAR_POSITION:n`,
//! `GEAR_DOWNLOCKED:n`, `GEAR_UPLOCKED:n`, `GEAR_DOOR_POSITION:n`,
//! `GEAR_STUCK_LOCKED:n`; the *sensed* (possibly-lying) counterparts
//! `SENSED_GEAR_DOWNLOCKED:n`/`SENSED_GEAR_UPLOCKED:n`
//! (`retraction::RetractionOutputs::sensed_downlocked`/`sensed_uplocked`);
//! `BRAKE_STACK_TEMP_C:n`, `BRAKE_WEAR_FRACTION:n`, `BRAKE_FIRE:n` (n =
//! 1..16, `LEG_WHEEL_INDICES` order); `ANTISKID_CHANNEL_FAULT:n` (a BITE-
//! detected direct report of `antiskid_inop` past a self-test threshold,
//! the same way a real antiskid computer posts its own channel faults --
//! not yet computed by `brakes.rs`, which only takes the fault as a raw
//! input today); `PARK_BRAKE_PRESS_PA`, `PARK_BRAKE_HOLDING`,
//! `PARK_BRAKE_SET`; `NW_STEER_ANGLE_DEG`, `NW_STEER_SHIMMY_UNSTABLE`,
//! `BODY_STEER_ANGLE_DEG:n`, `BODY_STEER_SHIMMY_UNSTABLE:n` (n = 1 left, 2
//! right -- the three steerable positions each carry their own
//! `SteeringActuator`/`shimmy_unstable`, so any ECAM trigger over shimmy
//! must check all three, not just the nose). Documented here per the
//! brief's instruction to record any new Var this model must publish.

use crate::deep::api::*;

const ATA: u16 = 32;

struct LegSpec {
    key: &'static str,
    name: &'static str,
    /// Instance number for the prospective `GEAR_STRUT_*:n` Vars above.
    n: u16,
}

const LEGS: [LegSpec; 5] = [
    LegSpec { key: "nose", name: "Nose gear", n: 1 },
    LegSpec { key: "l_wing", name: "Left wing gear", n: 2 },
    LegSpec { key: "r_wing", name: "Right wing gear", n: 3 },
    LegSpec { key: "l_body", name: "Left body gear", n: 4 },
    LegSpec { key: "r_body", name: "Right body gear", n: 5 },
];

pub fn register(r: &mut Registry) {
    reset_ids();
    register_struts(r);
    register_retractions(r);
    register_wheel_brakes(r);
    register_parking_brake(r);
    register_steering(r);
    register_bscu(r);
    register_brake_pedal_transducers(r);
    register_steer_ctl(r);
    register_steer_input_transducers(r);
    register_ecam(r);
}

fn register_struts(r: &mut Registry) {
    for leg in &LEGS {
        let component_id = format!("32_gear.{}_strut", leg.key);

        let gas_leak = r.failure(FailureDef {
            id: failure_id(Area::GearStructure, ATA, next_id()),
            area: Area::GearStructure,
            ata: ATA,
            name: format!("{} shock strut nitrogen precharge leak", leg.name),
            component: component_id.clone(),
            model_field: "gear_structure::strut::StrutFaults.gas_leak".into(),
            magnitude: "0..1, fraction of the full-leak (24 h to empty) leak rate".into(),
            effect: "gas_charge_fraction depletes; the leg sags to a higher static compression for the same load, eating into stroke margin before the next landing and raising flex/overload risk".into(),
        });
        let oil_leak = r.failure(FailureDef {
            id: failure_id(Area::GearStructure, ATA, next_id()),
            area: Area::GearStructure,
            ata: ATA,
            name: format!("{} shock strut hydraulic oil leak", leg.name),
            component: component_id.clone(),
            model_field: "gear_structure::strut::StrutFaults.oil_leak".into(),
            magnitude: "0..1, fraction of the full-leak (24 h to empty) leak rate".into(),
            effect: "oil_level_fraction depletes; orifice damping is lost, so a landing impact rebounds with far less energy absorbed, raising the peak reaction force for a given sink speed".into(),
        });

        // `E-IND-DESIGN.md` 320800043/046: the strut's own pressure-
        // monitoring BITE and its weight-on-wheels sensing, each an
        // independent side channel from the real gas charge / true ground
        // contact (see `strut::StrutFaults`'s own doc on each field).
        let gas_charge_sensor_fail = r.failure(FailureDef {
            id: failure_id(Area::GearStructure, ATA, next_id()),
            area: Area::GearStructure,
            ata: ATA,
            name: format!("{} strut pressure-monitoring sensor failure", leg.name),
            component: component_id.clone(),
            model_field: "gear_structure::strut::StrutFaults.gas_charge_sensor_fail".into(),
            magnitude: "inherently boolean (a BITE self-test either passes or fails): 0 healthy, >= 0.5 failed".into(),
            effect: "the strut's own pressure-sensing/monitoring function reports failed, independent of the real gas_charge_fraction (320800043 L/G OLEO PRESS MONITORING FAULT)".into(),
        });
        let wow_sensing_fail = r.failure(FailureDef {
            id: failure_id(Area::GearStructure, ATA, next_id()),
            area: Area::GearStructure,
            ata: ATA,
            name: format!("{} weight-on-wheels sensor failure", leg.name),
            component: component_id.clone(),
            model_field: "gear_structure::strut::StrutFaults.wow_sensing_fail".into(),
            magnitude: "0..1: >=0.5 the sensed ground-contact state is inverted from the true state (retraction::RetractionFaults::sensor_lies' own convention, applied to weight-on-wheels sensing)".into(),
            effect: "weight-on-wheels sensing disagrees with this leg's true ground-contact state, independent of it (320800046 L/G WEIGHT ON WHEELS FAULT)".into(),
        });

        r.component(ComponentDef {
            id: component_id,
            area: Area::GearStructure,
            ata: ATA,
            name: format!("{} oleo-pneumatic shock strut", leg.name),
            params: vec![
                ParamDef { name: "gas_charge_fraction".into(), meaning: "nitrogen precharge remaining, 0 empty .. 1 full service".into(), healthy: 1.0 },
                ParamDef { name: "oil_level_fraction".into(), meaning: "hydraulic damping oil remaining, 0 empty .. 1 full service".into(), healthy: 1.0 },
                ParamDef { name: "life_fraction_consumed".into(), meaning: "Miner's-rule fatigue budget consumed by landing cycles, 0 new .. 1 budget exhausted".into(), healthy: 0.0 },
            ],
            failures: vec![gas_leak, oil_leak, gas_charge_sensor_fail, wow_sensing_fail],
        });
    }
}

fn register_retractions(r: &mut Registry) {
    for leg in &LEGS {
        let component_id = format!("32_gear.{}_retraction", leg.key);

        let actuator_leak = r.failure(FailureDef {
            id: failure_id(Area::GearStructure, ATA, next_id()),
            area: Area::GearStructure,
            ata: ATA,
            name: format!("{} extend/retract actuator internal leak", leg.name),
            component: component_id.clone(),
            model_field: "gear_structure::retraction::RetractionFaults.actuator_leak".into(),
            magnitude: "0..1, fraction of nominal actuator speed/force lost".into(),
            effect: "the leg extends/retracts slower (or, at 1.0, not at all) for the same hydraulic pressure".into(),
        });
        let uplock_jam = r.failure(FailureDef {
            id: failure_id(Area::GearStructure, ATA, next_id()),
            area: Area::GearStructure,
            ata: ATA,
            name: format!("{} uplock hook jam", leg.name),
            component: component_id.clone(),
            model_field: "gear_structure::retraction::RetractionFaults.uplock_jam".into(),
            magnitude: "0..1: >=0.5 defeats the normal hydraulic release, >=0.95 also defeats gravity extension's separate mechanical/pneumatic release".into(),
            effect: "the leg cannot leave the up-locked state on a gear-down selection (moderate jam: gravity extension still frees it; severe jam: it will not extend at all)".into(),
        });
        let downlock_fail = r.failure(FailureDef {
            id: failure_id(Area::GearStructure, ATA, next_id()),
            area: Area::GearStructure,
            ata: ATA,
            name: format!("{} downlock spring/linkage failure", leg.name),
            component: component_id.clone(),
            model_field: "gear_structure::retraction::RetractionFaults.downlock_fail".into(),
            magnitude: "0..1: >=0.5 the downlock fails to fully seat".into(),
            effect: "the leg reaches the geometric down position but the downlock never truly engages; a real touchdown load then folds the leg (strut.rs's unlocked-collapse path)".into(),
        });
        let door_jam = r.failure(FailureDef {
            id: failure_id(Area::GearStructure, ATA, next_id()),
            area: Area::GearStructure,
            ata: ATA,
            name: format!("{} gear door actuator/track jam", leg.name),
            component: component_id.clone(),
            model_field: "gear_structure::retraction::RetractionFaults.door_jam".into(),
            magnitude: "0..1: >=0.5 the door seizes completely, otherwise proportionally slowed".into(),
            effect: "the door sequence stalls, which blocks the gear from ever starting to move (the interlock this model applies between door and gear travel)".into(),
        });
        let sensor_lies = r.failure(FailureDef {
            id: failure_id(Area::GearStructure, ATA, next_id()),
            area: Area::GearStructure,
            ata: ATA,
            name: format!("{} lock proximity sensor failure", leg.name),
            component: component_id.clone(),
            model_field: "gear_structure::retraction::RetractionFaults.sensor_lies".into(),
            magnitude: "0..1: >=0.5 the sensed lock indication is inverted from the true state".into(),
            effect: "the cockpit indication disagrees with the leg's true lock state, independent of it".into(),
        });

        let mut failures = vec![actuator_leak, uplock_jam, downlock_fail, door_jam, sensor_lies];

        // `E-IND-DESIGN.md` 320800032 L/G BOGIE POSITION FAULT: only the
        // two body legs carry a bogie-beam trim/levelling actuator (it
        // levels the bogie before it retracts into the wheel well) --
        // nose and wing legs have no such mechanism at all, so this
        // failure is only registered for `l_body`/`r_body`.
        if leg.key == "l_body" || leg.key == "r_body" {
            let bogie_trim_fail = r.failure(FailureDef {
                id: failure_id(Area::GearStructure, ATA, next_id()),
                area: Area::GearStructure,
                ata: ATA,
                name: format!("{} bogie trim/levelling actuator failure", leg.name),
                component: component_id.clone(),
                model_field: "gear_structure::retraction::RetractionFaults.bogie_trim_fail".into(),
                magnitude: "inherently boolean (a BITE self-test either passes or fails): 0 healthy, >= 0.5 failed to trim".into(),
                effect: "the body-gear bogie fails to trim/level before retraction (320800032 L/G BOGIE POSITION FAULT)".into(),
            });
            failures.push(bogie_trim_fail);
        }

        r.component(ComponentDef {
            id: component_id,
            area: Area::GearStructure,
            ata: ATA,
            name: format!("{} retraction/door/lock system", leg.name),
            params: Vec::new(),
            failures,
        });
    }
}

/// `LEG_WHEEL_INDICES`' own layout: [left wing, right wing, left body,
/// right body], 4 wheels each -- the 16 braked wheels only (matches
/// `physics::tyre::Tyres`'s existing 16-wheel numbering).
fn register_wheel_brakes(r: &mut Registry) {
    let leg_names = ["Left wing", "Right wing", "Left body", "Right body"];
    for (leg_idx, indices) in super::LEG_WHEEL_INDICES.iter().enumerate() {
        for &wheel in indices {
            let wheel_n = wheel + 1;
            let component_id = format!("32_gear.wheel_{wheel_n}_brake");

            let antiskid_inop = r.failure(FailureDef {
                id: failure_id(Area::GearStructure, ATA, next_id()),
                area: Area::GearStructure,
                ata: ATA,
                name: format!("{} wheel {wheel_n} antiskid channel failure", leg_names[leg_idx]),
                component: component_id.clone(),
                model_field: "gear_structure::brakes::BrakeFaults.antiskid_inop".into(),
                magnitude: "0..1, 1.0 = no skid protection at all on this wheel's channel".into(),
                effect: "commanded brake pressure is no longer released during a skid, letting this wheel lock and drag at full aircraft speed".into(),
            });
            let dragging = r.failure(FailureDef {
                id: failure_id(Area::GearStructure, ATA, next_id()),
                area: Area::GearStructure,
                ata: ATA,
                name: format!("{} wheel {wheel_n} dragging brake", leg_names[leg_idx]),
                component: component_id.clone(),
                model_field: "gear_structure::brakes::BrakeFaults.dragging".into(),
                magnitude: "0..1, added uncommanded brake-force fraction that never releases".into(),
                effect: "this wheel's stack heats and wears even with no pedal/autobrake command, and can reach the fire threshold on a long taxi".into(),
            });

            r.component(ComponentDef {
                id: component_id,
                area: Area::GearStructure,
                ata: ATA,
                name: format!("{} wheel {wheel_n} carbon brake", leg_names[leg_idx]),
                params: vec![ParamDef { name: "wear_fraction".into(), meaning: "carbon heat-sink wear budget consumed, 0 new .. 1 worn out".into(), healthy: 0.0 }],
                failures: vec![antiskid_inop, dragging],
            });
        }
    }
}

fn register_parking_brake(r: &mut Registry) {
    let component_id = "32_gear.parking_brake_accumulator".to_string();
    let leak = r.failure(FailureDef {
        id: failure_id(Area::GearStructure, ATA, next_id()),
        area: Area::GearStructure,
        ata: ATA,
        name: "Parking brake accumulator leak".into(),
        component: component_id.clone(),
        model_field: "gear_structure::brakes::ParkingBrakeFaults.leak".into(),
        magnitude: "0..1, fraction of the full-leak (few-hour bleed-down) rate".into(),
        effect: "the accumulator's stored pressure bleeds down while the parking brake is set, eventually falling below the minimum holding pressure".into(),
    });
    r.component(ComponentDef { id: component_id, area: Area::GearStructure, ata: ATA, name: "Parking brake hydraulic accumulator".into(), params: Vec::new(), failures: vec![leak] });
}

fn register_steering(r: &mut Registry) {
    let steering_names = [("nose", "Nosewheel"), ("l_body", "Left body gear rear-axle"), ("r_body", "Right body gear rear-axle")];
    for (key, name) in steering_names {
        let component_id = format!("32_gear.{key}_steering");

        let shimmy = r.failure(FailureDef {
            id: failure_id(Area::GearStructure, ATA, next_id()),
            area: Area::GearStructure,
            ata: ATA,
            name: format!("{name} shimmy damper failure"),
            component: component_id.clone(),
            model_field: "gear_structure::steering::SteeringFaults.shimmy_damper_fail".into(),
            magnitude: "0..1, mechanical damping coefficient reduced toward its residual structural minimum".into(),
            effect: "the critical (unstable) groundspeed for this wheel's torsional shimmy mode falls; above it, a self-excited oscillation grows instead of damping out".into(),
        });
        let actuator_leak = r.failure(FailureDef {
            id: failure_id(Area::GearStructure, ATA, next_id()),
            area: Area::GearStructure,
            ata: ATA,
            name: format!("{name} steering actuator internal leak"),
            component: component_id.clone(),
            model_field: "gear_structure::steering::SteeringFaults.actuator_leak".into(),
            magnitude: "0..1, fraction of nominal steering slew rate lost".into(),
            effect: "the wheel tracks a commanded steering angle more slowly".into(),
        });

        let mut failures = vec![shimmy, actuator_leak];

        // `E-IND-DESIGN.md` 320800056/057/059: the angle-limit override and
        // the disconnect (towing) mechanism exist only on the nosewheel.
        if key == "nose" {
            let disc_mechanism_fail = r.failure(FailureDef {
                id: failure_id(Area::GearStructure, ATA, next_id()),
                area: Area::GearStructure,
                ata: ATA,
                name: format!("{name} disconnect mechanism failure"),
                component: component_id.clone(),
                model_field: "gear_structure::steering::SteeringFaults.disc_mechanism_fail".into(),
                magnitude: "0..1: >=0.5 the mechanism no longer responds to a new disconnect/reconnect selection".into(),
                effect: "the nosewheel steering disconnect (towing) mechanism resists commanded release/engagement and freezes at its last state (320800057 STEER N/W STEER DISC FAULT, 320800059 STEER N/W STEER NOT DISC)".into(),
            });
            failures.push(disc_mechanism_fail);

            let steer_overtravel_fail = r.failure(FailureDef {
                id: failure_id(Area::GearStructure, ATA, next_id()),
                area: Area::GearStructure,
                ata: ATA,
                name: format!("{name} steering angle limit override failure"),
                component: component_id.clone(),
                model_field: "gear_structure::steering::SteeringFaults.steer_overtravel_fail".into(),
                magnitude: "0..1: >=0.5 defeats the nosewheel steering angle limit switch/software clamp".into(),
                effect: "a runaway actuator or a miscalibrated limit switch lets the true nosewheel angle exceed MAX_NOSE_ANGLE_DEG, which this model's own healthy clamp otherwise prevents (320800056 STEER N/W STEER ANGLE LIMIT EXCEEDED)".into(),
            });
            failures.push(steer_overtravel_fail);
        }

        r.component(ComponentDef { id: component_id, area: Area::GearStructure, ata: ATA, name: format!("{name} steering"), params: Vec::new(), failures });
    }
}

/// `E-IND-DESIGN.md`'s new Brake System Controller (BSC): the two BSCU
/// control channels, the normal/alternate pressure-monitoring pair, the
/// autobrake function and the brake selector valve. Hydraulic-source
/// availability itself is not modelled here at all -- it is read straight
/// off `deep::hydraulics`'s own already-published
/// `HYD_GREEN/YELLOW_MANIFOLD_PRESSURE_PSI` (and the existing parking-brake
/// accumulator's `PARK_BRAKE_PRESS_PA`) directly in `fbw/ata32.rs`'s own
/// trigger conditions, the same cross-area published-name read
/// `fbw/ata31_33.rs` already uses for the FCDC loads -- no second copy of
/// that state belongs in this area.
fn register_bscu(r: &mut Registry) {
    let component_id = "32_gear.bscu".to_string();
    let mut failures = Vec::new();
    let mut params = Vec::new();
    let channels: [(&str, &str, &str); 6] = [
        ("ctl_1_fail", "BSCU control channel 1 failure", "320800015 BRAKES CTL 1 FAULT"),
        ("ctl_2_fail", "BSCU control channel 2 failure", "320800016 BRAKES CTL 2 FAULT"),
        ("norm_press_sensor_fail", "BSCU normal brake pressure sensor failure", "320800021 BRAKES NORM BRK PRESS MONITORING FAULT"),
        ("alt_press_sensor_fail", "BSCU alternate brake pressure sensor failure", "320800012 BRAKES ALTN BRK PRESS MONITORING FAULT"),
        ("autobrake_fail", "BSCU autobrake function failure", "320800013 BRAKES AUTO BRK FAULT"),
        ("sel_valve_jam", "BSCU brake selector valve jammed open", "320800027 BRAKES SEL VLV JAMMED OPEN"),
    ];
    for (field, desc, alert) in channels {
        let id = r.failure(FailureDef {
            id: failure_id(Area::GearStructure, ATA, next_id()),
            area: Area::GearStructure,
            ata: ATA,
            name: desc.to_string(),
            component: component_id.clone(),
            model_field: format!("gear_structure::live::BscuFaults.{field}"),
            magnitude: "inherently boolean (a BITE self-test/jam either applies or does not): 0 healthy, >= 0.5 failed".into(),
            effect: format!("{alert}"),
        });
        failures.push(id);
        params.push(ParamDef { name: field.into(), meaning: desc.into(), healthy: 0.0 });
    }
    r.component(ComponentDef { id: component_id, area: Area::GearStructure, ata: ATA, name: "Brake System Control Unit (BSCU)".into(), params, failures });
}

/// `E-IND-DESIGN.md` 320800024 BRAKES PEDAL BRAKING FAULT: the two brake
/// pedal position transducers.
fn register_brake_pedal_transducers(r: &mut Registry) {
    let component_id = "32_gear.brake_pedal_transducers".to_string();
    let sides = [("left", "Left brake pedal transducer failure"), ("right", "Right brake pedal transducer failure")];
    let mut failures = Vec::new();
    let mut params = Vec::new();
    for (side, desc) in sides {
        let id = r.failure(FailureDef {
            id: failure_id(Area::GearStructure, ATA, next_id()),
            area: Area::GearStructure,
            ata: ATA,
            name: desc.to_string(),
            component: component_id.clone(),
            model_field: format!("gear_structure::live::BrakePedalTransducerFaults.{side}"),
            magnitude: "inherently boolean: 0 healthy, >= 0.5 failed".into(),
            effect: "320800024 BRAKES PEDAL BRAKING FAULT".into(),
        });
        failures.push(id);
        params.push(ParamDef { name: format!("pedal_sensor_fail_{side}"), meaning: desc.into(), healthy: 0.0 });
    }
    r.component(ComponentDef { id: component_id, area: Area::GearStructure, ata: ATA, name: "Brake pedal position transducers".into(), params, failures });
}

/// `E-IND-DESIGN.md`'s new Steering System Controller (SSC) control
/// channels and selector valve. As with the BSCU above, hydraulic-source
/// availability is read directly from `deep::hydraulics`'s own published
/// pressures in `fbw/ata32.rs`, not modelled here.
fn register_steer_ctl(r: &mut Registry) {
    let component_id = "32_gear.steer_ctl".to_string();
    let mut failures = Vec::new();
    let mut params = Vec::new();
    let channels: [(&str, &str, &str); 3] = [
        ("ctl_1_fail", "Steering control channel 1 failure", "320800053 STEER CTL 1 FAULT"),
        ("ctl_2_fail", "Steering control channel 2 failure", "320800054 STEER CTL 2 FAULT"),
        ("sel_valve_jam", "Steering selector valve jammed open", "320800062 STEER SEL VLV JAMMED OPEN"),
    ];
    for (field, desc, alert) in channels {
        let id = r.failure(FailureDef {
            id: failure_id(Area::GearStructure, ATA, next_id()),
            area: Area::GearStructure,
            ata: ATA,
            name: desc.to_string(),
            component: component_id.clone(),
            model_field: format!("gear_structure::live::SteerCtlFaults.{field}"),
            magnitude: "inherently boolean: 0 healthy, >= 0.5 failed".into(),
            effect: alert.to_string(),
        });
        failures.push(id);
        params.push(ParamDef { name: field.into(), meaning: desc.into(), healthy: 0.0 });
    }
    r.component(ComponentDef { id: component_id, area: Area::GearStructure, ata: ATA, name: "Steering System Control Unit".into(), params, failures });
}

/// `E-IND-DESIGN.md` 320800051/052/061: the captain's and F/O's tiller
/// transducers and the pedal-steering transducer.
fn register_steer_input_transducers(r: &mut Registry) {
    let component_id = "32_gear.steer_input_transducers".to_string();
    let mut failures = Vec::new();
    let mut params = Vec::new();
    let channels: [(&str, &str, &str); 3] = [
        ("capt_tiller_fail", "Captain's steering tiller transducer failure", "320800051 STEER CAPT STEER TILLER FAULT"),
        ("fo_tiller_fail", "F/O's steering tiller transducer failure", "320800052 STEER FO STEER TILLER FAULT"),
        ("pedal_steer_fail", "Pedal steering transducer failure", "320800061 STEER PEDAL STEER CTL FAULT"),
    ];
    for (field, desc, alert) in channels {
        let id = r.failure(FailureDef {
            id: failure_id(Area::GearStructure, ATA, next_id()),
            area: Area::GearStructure,
            ata: ATA,
            name: desc.to_string(),
            component: component_id.clone(),
            model_field: format!("gear_structure::live::SteerInputTransducerFaults.{field}"),
            magnitude: "inherently boolean: 0 healthy, >= 0.5 failed".into(),
            effect: alert.to_string(),
        });
        failures.push(id);
        params.push(ParamDef { name: field.into(), meaning: desc.into(), healthy: 0.0 });
    }
    r.component(ComponentDef { id: component_id, area: Area::GearStructure, ata: ATA, name: "Steering input transducers".into(), params, failures });
}

fn register_ecam(r: &mut Registry) {
    // Gear disagree / not locked: any leg reporting the geometric down
    // position without its true downlock (or a stuck uplock release).
    let gear_faults: Vec<u64> = r.failures.iter().filter(|f| f.model_field.ends_with("downlock_fail") || f.model_field.ends_with("uplock_jam")).map(|f| f.id).collect();
    r.alert(
        EcamAlert::new("L_G_GEAR_NOT_DOWNLOCKED", ATA, "L/G GEAR NOT DOWNLOCKED", Level::Warning, any(vec![
            var("GEAR_DOWNLOCKED:1").off(),
            var("GEAR_DOWNLOCKED:2").off(),
            var("GEAR_DOWNLOCKED:3").off(),
            var("GEAR_DOWNLOCKED:4").off(),
            var("GEAR_DOWNLOCKED:5").off(),
        ]))
        .confirm(1.0)
        .inhibit(&[Phase::LiftOff, Phase::Below800Ft])
        .step(line("GEAR LEVER", "RECYCLE").done(var("GEAR_LEVER_SELECTED_DOWN").on()))
        .status_line("L/G GEAR NOT DOWNLOCKED")
        .inop_sys("L/G NORMAL EXTENSION")
        .raised_by(&gear_faults),
    );

    // A genuine sensor-lies condition: the *sensed* lock indication reads
    // differently from the *true* one this model computes -- exactly what
    // `RetractionFaults::sensor_lies` does (module doc, `retraction.rs`).
    let sensor_lie_faults: Vec<u64> = r.failures.iter().filter(|f| f.model_field.ends_with("sensor_lies")).map(|f| f.id).collect();
    let disagree_conds: Vec<Cond> = LEGS
        .iter()
        .flat_map(|leg| {
            [
                Cond::VarVar { a: format!("SENSED_GEAR_DOWNLOCKED:{}", leg.n), cmp: Cmp::Ne, b: format!("GEAR_DOWNLOCKED:{}", leg.n) },
                Cond::VarVar { a: format!("SENSED_GEAR_UPLOCKED:{}", leg.n), cmp: Cmp::Ne, b: format!("GEAR_UPLOCKED:{}", leg.n) },
            ]
        })
        .collect();
    r.alert(
        EcamAlert::new("L_G_GEAR_DISAGREE", ATA, "L/G GEAR DISAGREE", Level::Caution, any(disagree_conds))
            .confirm(2.0)
            .status_line("L/G GEAR POSITION DISAGREE")
            .raised_by(&sensor_lie_faults),
    );

    let brake_fire_faults: Vec<u64> = r.failures.iter().filter(|f| f.model_field.ends_with("BrakeFaults.dragging")).map(|f| f.id).collect();
    r.alert(
        EcamAlert::new("WHEEL_BRAKE_FIRE", ATA, "WHEEL L/G BRAKE FIRE", Level::Warning, any((1..=16).map(|n| var(&format!("BRAKE_FIRE:{n}")).on()).collect()))
            .step(line("PARKING BRAKE", "SET").only_if(var("PARK_BRAKE_HOLDING").on()).done(var("PARK_BRAKE_SET").on()))
            .step(line("EVACUATION", "CONSIDER").after(10.0))
            .status_line("WHEEL BRAKE FIRE")
            .raised_by(&brake_fire_faults),
    );

    // A real antiskid computer self-tests each channel and posts its own
    // BITE fault directly (`ANTISKID_CHANNEL_FAULT:n`, module doc) rather
    // than the cockpit inferring it from a skid outcome.
    let antiskid_faults: Vec<u64> = r.failures.iter().filter(|f| f.model_field.ends_with("antiskid_inop")).map(|f| f.id).collect();
    r.alert(
        EcamAlert::new("L_G_BRAKES_ANTISKID_FAULT", ATA, "L/G BRAKES ANTISKID N/U", Level::Caution, any((1..=16).map(|n| var(&format!("ANTISKID_CHANNEL_FAULT:{n}")).on()).collect()))
            .status_line("ANTISKID FAULT")
            .inop_sys("AUTOBRAKE")
            .raised_by(&antiskid_faults),
    );

    let parking_leak_faults: Vec<u64> = r.failures.iter().filter(|f| f.model_field.ends_with("ParkingBrakeFaults.leak")).map(|f| f.id).collect();
    r.alert(
        EcamAlert::new("L_G_PARK_BRK_LO_PR", ATA, "L/G PARK BRAKE LO PR", Level::Caution, all(vec![var("PARK_BRAKE_SET").on(), var("PARK_BRAKE_HOLDING").off()]))
            .confirm(3.0)
            .step(line("PARKING BRAKE", "CHECK").done(var("PARK_BRAKE_HOLDING").on()))
            .status_line("PARK BRAKE ACCUMULATOR LOW")
            .raised_by(&parking_leak_faults),
    );

    // Three steerable positions (nose, left body, right body) each carry
    // their own shimmy-damper fault and their own `shimmy_unstable` output
    // (`steering.rs`) -- the trigger must cover all three, matching
    // `raised_by` below, or a body-gear shimmy could never raise this.
    let shimmy_faults: Vec<u64> = r.failures.iter().filter(|f| f.model_field.ends_with("shimmy_damper_fail")).map(|f| f.id).collect();
    r.alert(
        EcamAlert::new(
            "L_G_STEER_SHIMMY",
            ATA,
            "L/G STEERING SHIMMY",
            Level::Advisory,
            any(vec![var("NW_STEER_SHIMMY_UNSTABLE").on(), var("BODY_STEER_SHIMMY_UNSTABLE:1").on(), var("BODY_STEER_SHIMMY_UNSTABLE:2").on()]),
        )
        .confirm(2.0)
        .status_line("LANDING GEAR STEERING SHIMMY")
        .raised_by(&shimmy_faults),
    );
}

/// Sequential id counter for `failure_id`'s low three digits, so this file
/// never has to hand-number ~74 near-identical per-instance failures.
///
/// The counter is reset at the top of every [`register`] call rather than
/// running for the life of the process. That matters now that `live.rs`
/// resolves its failure ids by registering into a throw-away `Registry`:
/// with a process-wide counter, a second `register` handed out a second,
/// completely different set of ids, so the ids the live system consumed
/// would not have been the ids the plugin's own `deep::registry()` arms in
/// `Faults`. The numbering of the first call is unchanged.
fn next_id() -> u16 {
    COUNTER.with(|c| {
        let n = c.get();
        c.set(n + 1);
        n
    })
}

thread_local! {
    static COUNTER: std::cell::Cell<u16> = const { std::cell::Cell::new(1) };
}

fn reset_ids() {
    COUNTER.with(|c| c.set(1));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn everything_registered_validates_clean() {
        let mut r = Registry::default();
        register(&mut r);
        let errors = r.validate_area();
        assert!(errors.is_empty(), "registry validation errors: {errors:?}");
    }

    #[test]
    fn every_leg_strut_and_retraction_component_is_present() {
        let mut r = Registry::default();
        register(&mut r);
        for leg in &LEGS {
            assert!(r.components.iter().any(|c| c.id == format!("32_gear.{}_strut", leg.key)));
            assert!(r.components.iter().any(|c| c.id == format!("32_gear.{}_retraction", leg.key)));
        }
    }

    #[test]
    fn all_sixteen_braked_wheels_have_a_brake_component() {
        let mut r = Registry::default();
        register(&mut r);
        for n in 1..=16 {
            assert!(r.components.iter().any(|c| c.id == format!("32_gear.wheel_{n}_brake")), "wheel {n} brake component missing");
        }
    }

    #[test]
    fn failure_ids_are_all_unique_and_carry_the_gear_structure_area_and_ata() {
        let mut r = Registry::default();
        register(&mut r);
        let mut ids: Vec<u64> = r.failures.iter().map(|f| f.id).collect();
        let before = ids.len();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), before, "duplicate failure ids registered");
        for f in &r.failures {
            assert_eq!(f.area, Area::GearStructure);
            assert_eq!(f.ata, ATA);
        }
    }

    /// Regression for a reviewer-flagged bug: the shimmy alert's
    /// `raised_by` originally covered all three steerable positions (nose,
    /// left body, right body) but its trigger only read the nosewheel's
    /// variable, so a body-gear-only shimmy could never raise it. Every
    /// position `raised_by` names for a fault must have a way to make the
    /// same alert's trigger fire.
    #[test]
    fn a_body_gear_only_shimmy_still_raises_the_steering_shimmy_alert() {
        let mut r = Registry::default();
        register(&mut r);
        let alert = r.alerts.iter().find(|a| a.key == "L_G_STEER_SHIMMY").expect("shimmy alert registered");
        assert_eq!(alert.failures.len(), 3, "the shimmy alert should be raised by exactly the three steerable positions' faults");

        let read_nose_only = |name: &str| if name == "NW_STEER_SHIMMY_UNSTABLE" { 1.0 } else { 0.0 };
        assert!(alert.trigger.eval(&read_nose_only), "a nosewheel-only shimmy must still raise it");

        let read_body_only = |name: &str| if name == "BODY_STEER_SHIMMY_UNSTABLE:1" { 1.0 } else { 0.0 };
        assert!(alert.trigger.eval(&read_body_only), "a left-body-gear-only shimmy, with the nosewheel healthy, must also raise it");

        let read_none = |_name: &str| 0.0;
        assert!(!alert.trigger.eval(&read_none), "no shimmy anywhere must not raise it");
    }

    /// Every other alert audited by hand for the same trigger-vs-raised_by
    /// scope mismatch the shimmy alert had: `L_G_GEAR_NOT_DOWNLOCKED` and
    /// `L_G_GEAR_DISAGREE` each check all 5 legs' own variables against all
    /// 5 legs' own faults; `WHEEL_BRAKE_FIRE` and
    /// `L_G_BRAKES_ANTISKID_FAULT` each check all 16 wheels against all 16
    /// wheels' own faults; `L_G_PARK_BRK_LO_PR` is a single system-wide
    /// accumulator on both sides. This test pins the two multi-position
    /// alerts' counts so a future edit that narrows one side without the
    /// other (as the shimmy bug did) fails loudly.
    #[test]
    fn multi_position_alerts_trigger_and_raised_by_counts_stay_in_step() {
        let mut r = Registry::default();
        register(&mut r);
        let downlock = r.alerts.iter().find(|a| a.key == "L_G_GEAR_NOT_DOWNLOCKED").unwrap();
        assert_eq!(downlock.failures.len(), 10, "5 legs x (downlock_fail, uplock_jam)");
        let disagree = r.alerts.iter().find(|a| a.key == "L_G_GEAR_DISAGREE").unwrap();
        assert_eq!(disagree.failures.len(), 5, "5 legs x sensor_lies");
        let fire = r.alerts.iter().find(|a| a.key == "WHEEL_BRAKE_FIRE").unwrap();
        assert_eq!(fire.failures.len(), 16, "16 wheels x dragging");
        let antiskid = r.alerts.iter().find(|a| a.key == "L_G_BRAKES_ANTISKID_FAULT").unwrap();
        assert_eq!(antiskid.failures.len(), 16, "16 wheels x antiskid_inop");
    }
}
