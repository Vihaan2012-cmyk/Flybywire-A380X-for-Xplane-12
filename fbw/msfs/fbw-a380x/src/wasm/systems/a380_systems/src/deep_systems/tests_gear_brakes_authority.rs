#[allow(unused_imports)]
use std::time::Duration;

#[allow(unused_imports)]
use systems::simulation::test::{ReadByName, SimulationTestBed, TestBed, WriteByName};

#[allow(unused_imports)]
use super::tests::{aircraft, arm, failure_id, run};
#[allow(unused_imports)]
use crate::A380;

fn roll_wheel_one(test_bed: &mut SimulationTestBed<A380>) {
    test_bed.write_by_name("WHEEL RPM:1", 300.);
}

#[test]
fn a_dragging_brake_heats_flybywires_own_brake_continuously_with_severity() {
    let dragging_id = failure_id("32_gear.wheel_1_brake", "dragging brake");

    let mut healthy = aircraft();
    roll_wheel_one(&mut healthy);
    run(&mut healthy, 100);
    let healthy_temp: f64 = healthy.read_by_name("BRAKE_TEMPERATURE_1");

    let mut mild = aircraft();
    arm(&mut mild, dragging_id as f64, 0.3);
    roll_wheel_one(&mut mild);
    run(&mut mild, 100);
    let mild_temp: f64 = mild.read_by_name("BRAKE_TEMPERATURE_1");

    let mut severe = aircraft();
    arm(&mut severe, dragging_id as f64, 1.0);
    roll_wheel_one(&mut severe);
    run(&mut severe, 100);
    let severe_temp: f64 = severe.read_by_name("BRAKE_TEMPERATURE_1");

    let mild_dragging: f64 = mild.read_by_name("BRAKE_DEEP_DRAGGING_1");
    let severe_dragging: f64 = severe.read_by_name("BRAKE_DEEP_DRAGGING_1");
    assert!(
        (mild_dragging - 0.3).abs() < 0.01,
        "a 0.3 dragging arm must publish back as 0.3, not a rounded/binary value: {mild_dragging}"
    );
    assert!(
        (severe_dragging - 1.0).abs() < 0.01,
        "a 1.0 dragging arm must publish back as 1.0: {severe_dragging}"
    );

    assert!(
        mild_temp > healthy_temp + 0.5,
        "a 0.3 dragging brake must measurably heat FlyByWire's own BRAKE_TEMPERATURE_1 above the healthy baseline: healthy {healthy_temp} C, mild {mild_temp} C"
    );
    assert!(
        severe_temp > mild_temp,
        "a fully dragging brake must heat FlyByWire's own brake more than a partly dragging one, proving the authority scales with severity rather than tripping one fixed effect: mild {mild_temp} C, severe {severe_temp} C (healthy {healthy_temp} C)"
    );

    let severe_right_temp: f64 = severe.read_by_name("BRAKE_TEMPERATURE_3");
    let healthy_right_temp: f64 = healthy.read_by_name("BRAKE_TEMPERATURE_3");
    assert!(
        (severe_right_temp - healthy_right_temp).abs() < 0.5,
        "a left-wing-only dragging fault must not heat the right wing group's brakes: healthy {healthy_right_temp} C, severe (should be unaffected) {severe_right_temp} C"
    );
}

#[test]
fn a_healthy_aircraft_is_unchanged_by_the_gear_brakes_authority() {
    let mut test_bed = aircraft();
    roll_wheel_one(&mut test_bed);
    run(&mut test_bed, 50);

    for n in 1..=16 {
        let dragging: f64 = test_bed.read_by_name(&format!("BRAKE_DEEP_DRAGGING_{n}"));
        let antiskid_inop: f64 = test_bed.read_by_name(&format!("BRAKE_DEEP_ANTISKID_INOP_{n}"));
        assert_eq!(dragging, 0.0, "BRAKE_DEEP_DRAGGING:{n} must publish exactly 0 on a healthy aircraft");
        assert_eq!(
            antiskid_inop, 0.0,
            "BRAKE_DEEP_ANTISKID_INOP:{n} must publish exactly 0 on a healthy aircraft"
        );
    }

    let temp_early: f64 = test_bed.read_by_name("BRAKE_TEMPERATURE_1");
    run(&mut test_bed, 200);
    let temp_late: f64 = test_bed.read_by_name("BRAKE_TEMPERATURE_1");
    assert!(
        (temp_late - temp_early).abs() < 5.0,
        "a healthy aircraft's brake temperature must stay stable with the gear/brakes authority wired in: {temp_early} -> {temp_late} C"
    );
}

#[test]
fn a_jammed_nosewheel_disconnect_keeps_steering_disconnected_after_towing() {
    for jammed in [false, true] {
        let mut test_bed = aircraft();
        test_bed.write_by_name("PUSHBACK STATE", 3.);
        run(&mut test_bed, 200);
        test_bed.write_by_name("EXTERNAL_BYPASS_PIN_INSERTED", true);
        run(&mut test_bed, 20);
        let towing: bool = test_bed.read_by_name("HYD_NW_STRG_DISC_ECAM_MEMO");
        assert!(towing, "the pin is in for towing");
        if jammed {
            assert_eq!(arm(&mut test_bed, failure_id("32_gear.nose_steering", "disconnect mechanism") as f64, 1.), 1.);
        }
        run(&mut test_bed, 20);
        test_bed.write_by_name("EXTERNAL_BYPASS_PIN_INSERTED", false);
        run(&mut test_bed, 50);
        let disconnected: bool = test_bed.read_by_name("HYD_NW_STRG_DISC_ECAM_MEMO");
        assert_eq!(disconnected, jammed, "jammed {jammed}: steering disconnected after the pin is pulled");
    }
}

#[test]
fn a_failed_downlock_spring_leaves_the_leg_unlocked_after_a_gear_cycle() {
    for failed in [false, true] {
        let mut test_bed = aircraft();
        test_bed.set_on_ground(false);
        let hold = |test_bed: &mut SimulationTestBed<A380>, down: f64, frames: usize| {
            for _ in 0..frames {
                super::tests_ecam::engines_running(test_bed, 85., 95., 97.);
                test_bed.write_by_name("GEAR_HANDLE_POSITION", down);
                run(test_bed, 1);
            }
        };
        hold(&mut test_bed, 1., 300);
        hold(&mut test_bed, 0., 600);
        if failed {
            assert_eq!(arm(&mut test_bed, failure_id("32_gear.l_wing_retraction", "downlock") as f64, 1.), 1.);
        }
        hold(&mut test_bed, 1., 600);
        let left_wing: f64 = test_bed.read_by_name("GEAR_DOWNLOCKED_2");
        let nose: f64 = test_bed.read_by_name("GEAR_DOWNLOCKED_1");
        assert_eq!(nose, 1., "the healthy nose leg locks down");
        assert_eq!(left_wing, if failed { 0. } else { 1. }, "failed {failed}: left wing gear downlocked");
        let fault: f64 = test_bed.read_by_name("GEAR_DOWNLOCK_FAULT_2");
        assert_eq!(fault, if failed { 1. } else { 0. }, "failed {failed}: the downlock fault reaches FlyByWire's gear system");
    }
}
