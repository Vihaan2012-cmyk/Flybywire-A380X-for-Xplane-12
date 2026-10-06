#[allow(unused_imports)]
use std::time::Duration;

#[allow(unused_imports)]
use systems::simulation::test::{ReadByName, SimulationTestBed, TestBed, WriteByName};

#[allow(unused_imports)]
use super::tests::{aircraft, arm, computers_healthy, failure_id, run};
#[allow(unused_imports)]
use crate::A380;

fn spin_engines(test_bed: &mut SimulationTestBed<A380>, n1: f64, n2: f64, n3: f64) {
    for n in 1..=4 {
        test_bed.write_by_name(&format!("ENGINE_STATE:{n}"), 1.);
        test_bed.write_by_name(&format!("ENGINE_N1:{n}"), n1);
        test_bed.write_by_name(&format!("TURB ENG CORRECTED N1:{n}"), n1);
        test_bed.write_by_name(&format!("TURB ENG CORRECTED N2:{n}"), n2);
        test_bed.write_by_name(&format!("ENGINE_N2:{n}"), n2);
        test_bed.write_by_name(&format!("ENGINE_N2_HEALTHY:{n}"), n2);
        test_bed.write_by_name(&format!("ENGINE_N3:{n}"), n3);
        test_bed.write_by_name(&format!("ENGINE_N3_HEALTHY:{n}"), n3);
    }
}

#[test]
fn a_partial_edp_fault_sags_flybywires_own_green_pressure_without_tripping() {
    let healthy_id = failure_id("29_hyd.green_edp_1a", "displacement loss");
    let mut healthy = aircraft();
    spin_engines(&mut healthy, 85., 95., 97.);
    run(&mut healthy, 150);
    let healthy_pressure: f64 = healthy.read_by_name("HYD_GREEN_SYSTEM_1_SECTION_PRESSURE");

    let mut degraded = aircraft();
    spin_engines(&mut degraded, 85., 95., 97.);
    arm(&mut degraded, healthy_id as f64, 0.3);
    run(&mut degraded, 150);
    let degraded_pressure: f64 = degraded.read_by_name("HYD_GREEN_SYSTEM_1_SECTION_PRESSURE");

    assert!(healthy_pressure > 2500.0, "setup: green should reach a real service pressure on four EDPs: {healthy_pressure} psi");
    assert!(degraded_pressure > 1000.0, "one pump losing 30% of its displacement out of four must not collapse the whole circuit: {degraded_pressure} psi");

    let degraded_loss: f64 = degraded.read_by_name("DEEP_HYD_EDP_1A_CAPABILITY_LOSS");
    let healthy_loss: f64 = healthy.read_by_name("DEEP_HYD_EDP_1A_CAPABILITY_LOSS");
    assert_eq!(healthy_loss, 0.0, "a healthy EDP 1A carries no capability loss");
    assert!(
        degraded_loss > 0.15 && degraded_loss < 0.6,
        "a 0.3 displacement loss must reach FlyByWire's EDP 1A as a partial capability loss: {degraded_loss}"
    );

    let healthy_active: f64 = healthy.read_by_name("HYD_1A_EDPUMP_ACTIVE");
    let still_active: f64 = degraded.read_by_name("HYD_1A_EDPUMP_ACTIVE");
    assert_eq!(still_active, healthy_active, "0.3 displacement loss must not change the pump's own active state");
}

#[test]
fn a_deep_reservoir_leak_drains_flybywires_own_reservoir_by_magnitude() {
    let leak_id = failure_id("29_hyd.green_line_gear", "leak");

    let mut healthy = aircraft();
    run(&mut healthy, 50);
    let start_level: f64 = healthy.read_by_name("HYD_GREEN_RESERVOIR_LEVEL");
    run(&mut healthy, 300);
    let healthy_level: f64 = healthy.read_by_name("HYD_GREEN_RESERVOIR_LEVEL");
    assert!((start_level - healthy_level).abs() < 0.05, "setup: a healthy reservoir should not measurably drain in 30 s: {start_level} -> {healthy_level}");

    let mut mild = aircraft();
    arm(&mut mild, leak_id as f64, 0.3);
    run(&mut mild, 300);
    let mild_level: f64 = mild.read_by_name("HYD_GREEN_RESERVOIR_LEVEL");

    let mut severe = aircraft();
    arm(&mut severe, leak_id as f64, 1.0);
    run(&mut severe, 300);
    let severe_level: f64 = severe.read_by_name("HYD_GREEN_RESERVOIR_LEVEL");

    assert!(mild_level < start_level - 0.01, "a 0.3 line leak must measurably drain FlyByWire's own reservoir: {start_level} -> {mild_level}");
    assert!(
        severe_level < mild_level,
        "a full leak must drain faster than a mild one over the same time, proving the input scales with magnitude rather than tripping one fixed rate: mild {mild_level}, severe {severe_level} (start {start_level})"
    );

    let yellow_mild: f64 = mild.read_by_name("HYD_YELLOW_RESERVOIR_LEVEL");
    let yellow_healthy: f64 = healthy.read_by_name("HYD_YELLOW_RESERVOIR_LEVEL");
    assert!((yellow_mild - yellow_healthy).abs() < 0.05, "a green leak must not drain yellow's reservoir");
}

#[test]
fn a_healthy_aircraft_hydraulics_are_unchanged() {
    let mut test_bed = aircraft();
    spin_engines(&mut test_bed, 85., 95., 97.);
    run(&mut test_bed, 50);
    let pressure_early: f64 = test_bed.read_by_name("HYD_GREEN_SYSTEM_1_SECTION_PRESSURE");
    let reservoir_early: f64 = test_bed.read_by_name("HYD_GREEN_RESERVOIR_LEVEL");
    run(&mut test_bed, 200);
    let pressure_late: f64 = test_bed.read_by_name("HYD_GREEN_SYSTEM_1_SECTION_PRESSURE");
    let reservoir_late: f64 = test_bed.read_by_name("HYD_GREEN_RESERVOIR_LEVEL");

    assert!(pressure_early > 2500.0, "setup: green should reach service pressure: {pressure_early} psi");
    assert!((pressure_late - pressure_early).abs() < 50.0, "a healthy aircraft's green pressure must stay stable: {pressure_early} -> {pressure_late} psi");
    assert!((reservoir_late - reservoir_early).abs() < 0.1, "a healthy aircraft's reservoir must not drain: {reservoir_early} -> {reservoir_late}");

    for id in [
        "DEEP_HYD_EDP_1A_CAPABILITY_LOSS",
        "DEEP_HYD_ELEC_PUMP_GREEN_A_CAPABILITY_LOSS",
        "DEEP_HYD_GREEN_RESERVOIR_LEAK_M3_S",
        "DEEP_HYD_GREEN_ACCUMULATOR_PRECHARGE_LOSS",
        "DEEP_HYD_GREEN_PRIORITY_VALVE_STUCK",
    ] {
        let v: f64 = test_bed.read_by_name(id);
        assert_eq!(v, 0.0, "{id} must publish exactly 0 on a healthy aircraft");
    }
}

fn open_fwd_cargo_door_and_run(test_bed: &mut SimulationTestBed<A380>, frames: usize) -> f64 {
    test_bed.write_by_name("FWD_DOOR_CARGO_OPEN_REQ", 1.);
    run(test_bed, frames);
    test_bed.read_by_name("FWD_DOOR_CARGO_POSITION")
}

#[test]
fn a_healthy_aircraft_is_unchanged_by_the_cabin_authority() {
    let mut test_bed = aircraft();
    let position = open_fwd_cargo_door_and_run(&mut test_bed, 450);

    assert!(position > 0.85, "setup: a healthy door should open most of the way in 45 s: {position}");

    for id in ["CABIN_CARGO_DOOR_JAM_FRACTION:1", "CABIN_CARGO_DOOR_HYDRAULIC_LOSS_FRACTION:1"] {
        let v: f64 = test_bed.read_by_name(id);
        assert_eq!(v, 0.0, "{id} must publish exactly 0 on a healthy aircraft");
    }
}

#[test]
fn a_deep_cargo_door_jam_caps_flybywires_own_door_travel_by_magnitude() {
    let jam_id = failure_id("52_dr.cargo_actuator", "jam");

    let mut healthy = aircraft();
    let healthy_position = open_fwd_cargo_door_and_run(&mut healthy, 450);
    assert!(healthy_position > 0.85, "setup: {healthy_position}");

    let mut mild = aircraft();
    arm(&mut mild, jam_id as f64, 0.5);
    let mild_position = open_fwd_cargo_door_and_run(&mut mild, 450);

    let mut severe = aircraft();
    arm(&mut severe, jam_id as f64, 1.0);
    let severe_position = open_fwd_cargo_door_and_run(&mut severe, 450);

    assert!(
        mild_position < healthy_position - 0.1,
        "a 0.5 cargo-door jam must measurably cap FlyByWire's own door travel: healthy {healthy_position}, jammed {mild_position}"
    );
    assert!(mild_position > 0.3, "a 0.5 jam caps travel near 50%, not near 0%: {mild_position}");
    assert!(
        severe_position < mild_position,
        "a fully seized actuator must cap travel lower than a half jam: mild {mild_position}, severe {severe_position}"
    );
    assert!(severe_position < 0.05, "a fully seized (jam = 1.0) actuator must not move the door at all: {severe_position}");
}

#[test]
fn a_deep_cargo_door_hydraulic_loss_slows_flybywires_own_door_without_fully_stopping_a_mild_one() {
    let hyd_id = failure_id("52_dr.cargo_actuator", "hydraulic");

    let mut healthy = aircraft();
    let healthy_position = open_fwd_cargo_door_and_run(&mut healthy, 150);
    assert!(healthy_position > 0.1, "setup: a healthy door should have started moving by 15 s: {healthy_position}");

    let mut starved = aircraft();
    arm(&mut starved, hyd_id as f64, 0.8);
    let starved_position = open_fwd_cargo_door_and_run(&mut starved, 150);

    let mut dead = aircraft();
    arm(&mut dead, hyd_id as f64, 1.0);
    let dead_position = open_fwd_cargo_door_and_run(&mut dead, 150);

    assert!(
        starved_position < healthy_position,
        "an 80% hydraulic-circuit loss must slow FlyByWire's own door actuator: healthy {healthy_position}, starved {starved_position}"
    );
    assert_eq!(dead_position, 0.0, "a fully lost actuator hydraulic circuit must not move the door at all: {dead_position}");
}
