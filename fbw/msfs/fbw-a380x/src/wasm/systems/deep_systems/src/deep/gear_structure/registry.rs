use crate::deep::api::*;

const ATA: u16 = 32;

struct LegSpec {
    key: &'static str,
    name: &'static str,
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
}

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
}
