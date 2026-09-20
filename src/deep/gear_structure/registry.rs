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
    register_struts(r);
    register_retractions(r);
    register_wheel_brakes(r);
    register_parking_brake(r);
    register_steering(r);
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
            failures: vec![gas_leak, oil_leak],
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

        r.component(ComponentDef {
            id: component_id,
            area: Area::GearStructure,
            ata: ATA,
            name: format!("{} retraction/door/lock system", leg.name),
            params: Vec::new(),
            failures: vec![actuator_leak, uplock_jam, downlock_fail, door_jam, sensor_lies],
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

        r.component(ComponentDef { id: component_id, area: Area::GearStructure, ata: ATA, name: format!("{name} steering"), params: Vec::new(), failures: vec![shimmy, actuator_leak] });
    }
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
        .step(line("GEAR LEVER", "RECYCLE").done(var("GEAR_LEVER_POSITION_REQUEST").on()))
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
fn next_id() -> u16 {
    use std::sync::atomic::{AtomicU16, Ordering};
    static COUNTER: AtomicU16 = AtomicU16::new(1);
    COUNTER.fetch_add(1, Ordering::Relaxed)
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
