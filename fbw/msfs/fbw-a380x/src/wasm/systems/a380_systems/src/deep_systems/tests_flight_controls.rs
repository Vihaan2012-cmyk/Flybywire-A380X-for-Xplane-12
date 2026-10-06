#[allow(unused_imports)]
use std::time::Duration;

#[allow(unused_imports)]
use systems::simulation::test::{ReadByName, SimulationTestBed, TestBed, WriteByName};

#[allow(unused_imports)]
use super::tests::{aircraft, arm, computers_healthy, failure_id, run};
#[allow(unused_imports)]
use crate::A380;

#[test]
fn a_healthy_aircraft_shows_no_flight_controls_faults() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 50);

    let flaps_jammed: bool = test_bed.read_by_name("FLAPS_JAMMED");
    let slats_jammed: bool = test_bed.read_by_name("SLATS_JAMMED");
    assert!(!flaps_jammed, "a healthy aircraft's flaps must not be reported jammed");
    assert!(!slats_jammed, "a healthy aircraft's slats must not be reported jammed");

    for gate in [
        "ROLLOUT_BREAKER_OPEN",
        "PRIM_1_BREAKER_OPEN",
        "PRIM_2_BREAKER_OPEN",
        "PRIM_3_BREAKER_OPEN",
        "SEC_1_BREAKER_OPEN",
        "SEC_2_BREAKER_OPEN",
        "SEC_3_BREAKER_OPEN",
        "FCDC_1_BREAKER_OPEN",
        "FCDC_2_BREAKER_OPEN",
    ] {
        let open: f64 = test_bed.read_by_name(gate);
        assert_eq!(open, 0., "{gate} must read closed on a healthy aircraft");
    }

    let rudder_trim: f64 = test_bed.read_by_name("A32NX_SEC_1_RUDDER_ACTUAL_POSITION");
    assert!(!rudder_trim.is_nan(), "rudder trim readback must be a number");
}

#[test]
fn a_deep_flap_pcu_jam_reaches_flybywires_own_flap_drive() {
    let mut test_bed = aircraft();

    let id = failure_id("27_fctl.flap_l", "PCU jam") as f64;
    arm(&mut test_bed, id, 1.0);
    run(&mut test_bed, 5);

    let flaps_jammed: bool = test_bed.read_by_name("FLAPS_JAMMED");
    assert!(flaps_jammed, "an armed flap PCU jam must reach FlyByWire's own FLAPS_JAMMED input");
    let slats_jammed: bool = test_bed.read_by_name("SLATS_JAMMED");
    assert!(!slats_jammed, "a flap-only jam must not report the slats jammed");

    arm(&mut test_bed, id, 0.0);
    run(&mut test_bed, 5);
    let flaps_jammed_after_clear: bool = test_bed.read_by_name("FLAPS_JAMMED");
    assert!(!flaps_jammed_after_clear, "clearing the armed jam must clear FLAPS_JAMMED");
}

#[test]
fn both_prim_1_feed_breakers_open_drives_its_gate() {
    let mut test_bed = aircraft();

    let gate_before: f64 = test_bed.read_by_name("PRIM_1_BREAKER_OPEN");
    assert_eq!(gate_before, 0.);

    test_bed.write_by_name("BKR_PRIM_1_NORMAL_BKR_CMD", 1.);
    run(&mut test_bed, 1);
    let gate_one_open: f64 = test_bed.read_by_name("PRIM_1_BREAKER_OPEN");
    assert_eq!(gate_one_open, 0., "one feed alone must not open the gate");

    test_bed.write_by_name("BKR_PRIM_1_2ND_BKR_CMD", 1.);
    run(&mut test_bed, 1);
    let gate_both_open: f64 = test_bed.read_by_name("PRIM_1_BREAKER_OPEN");
    assert_eq!(gate_both_open, 1., "both feeds open must open PRIM_1_BREAKER_OPEN");

    let sec_gate: f64 = test_bed.read_by_name("SEC_1_BREAKER_OPEN");
    assert_eq!(sec_gate, 0.);
}
