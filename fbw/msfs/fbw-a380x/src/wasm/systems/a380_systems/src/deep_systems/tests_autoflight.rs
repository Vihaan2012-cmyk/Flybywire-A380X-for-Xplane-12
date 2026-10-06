#[allow(unused_imports)]
use std::time::Duration;

#[allow(unused_imports)]
use systems::simulation::test::{ReadByName, SimulationTestBed, TestBed, WriteByName};

#[allow(unused_imports)]
use super::tests::{aircraft, arm, computers_healthy, failure_id, run};
#[allow(unused_imports)]
use crate::A380;

fn let_the_authority_react(test_bed: &mut SimulationTestBed<A380>) {
    run(test_bed, 1);
}

#[test]
fn a_healthy_aircraft_is_unchanged_by_the_autoflight_authority() {
    let mut test_bed = aircraft();
    test_bed.write_by_name("AUTOPILOT_1_ACTIVE", 1.0);
    test_bed.write_by_name("AUTOPILOT_2_ACTIVE", 0.0);
    run(&mut test_bed, 50);

    let ap1: f64 = test_bed.read_by_name("AUTOPILOT_1_ACTIVE");
    let ap2: f64 = test_bed.read_by_name("AUTOPILOT_2_ACTIVE");
    assert_eq!(ap1, 1.0, "a healthy FCU must leave FlyByWire's own engaged autopilot untouched");
    assert_eq!(ap2, 0.0, "a healthy FCU must leave FlyByWire's own disengaged autopilot untouched");

    for id in [
        "DEEP_AUTOFLT_FCU_FAULT",
        "DEEP_AUTOFLT_FCU_SWITCHED_OFF",
        "DEEP_AUTOFLT_CAPT_FCU_BKUP_FAULT",
        "DEEP_AUTOFLT_FO_FCU_BKUP_FAULT",
    ] {
        let v: f64 = test_bed.read_by_name(id);
        assert_eq!(v, 0.0, "{id} must publish exactly 0 on a healthy aircraft");
    }
}

#[test]
fn a_deep_fcu_fault_sags_flybywires_own_autopilot_output_continuously_with_severity() {
    let fault_id = failure_id("22_afs.fcu", "fault");

    let mut mild = aircraft();
    mild.write_by_name("AUTOPILOT_1_ACTIVE", 1.0);
    mild.write_by_name("AUTOPILOT_2_ACTIVE", 1.0);
    arm(&mut mild, fault_id as f64, 0.3);
    let_the_authority_react(&mut mild);
    let mild_ap1: f64 = mild.read_by_name("AUTOPILOT_1_ACTIVE");
    let mild_ap2: f64 = mild.read_by_name("AUTOPILOT_2_ACTIVE");

    let mut severe = aircraft();
    severe.write_by_name("AUTOPILOT_1_ACTIVE", 1.0);
    severe.write_by_name("AUTOPILOT_2_ACTIVE", 1.0);
    arm(&mut severe, fault_id as f64, 0.8);
    let_the_authority_react(&mut severe);
    let severe_ap1: f64 = severe.read_by_name("AUTOPILOT_1_ACTIVE");

    let mut full = aircraft();
    full.write_by_name("AUTOPILOT_1_ACTIVE", 1.0);
    full.write_by_name("AUTOPILOT_2_ACTIVE", 1.0);
    arm(&mut full, fault_id as f64, 1.0);
    let_the_authority_react(&mut full);
    let full_ap1: f64 = full.read_by_name("AUTOPILOT_1_ACTIVE");
    let full_ap2: f64 = full.read_by_name("AUTOPILOT_2_ACTIVE");

    assert!(
        mild_ap1 < 1.0 && mild_ap1 > 0.0,
        "a 0.3 FCU fault must measurably sag FlyByWire's own autopilot output without zeroing it: {mild_ap1}"
    );
    assert_eq!(mild_ap1, mild_ap2, "both autopilots share the one FCU and must sag together");
    assert!(
        severe_ap1 < mild_ap1,
        "a worse FCU fault must sag FlyByWire's own autopilot output further than a milder one: mild {mild_ap1}, severe {severe_ap1}"
    );
    assert_eq!(full_ap1, 0.0, "a fully faulted FCU must drop FlyByWire's own autopilot 1 output to zero");
    assert_eq!(full_ap2, 0.0, "a fully faulted FCU must drop FlyByWire's own autopilot 2 output to zero");
}

#[test]
fn fcu_switched_off_also_drives_the_same_autopilot_output() {
    let switched_off_id = failure_id("22_afs.fcu", "switched off");

    let mut test_bed = aircraft();
    test_bed.write_by_name("AUTOPILOT_1_ACTIVE", 1.0);
    arm(&mut test_bed, switched_off_id as f64, 1.0);
    let_the_authority_react(&mut test_bed);

    let ap1: f64 = test_bed.read_by_name("AUTOPILOT_1_ACTIVE");
    assert_eq!(ap1, 0.0, "an FCU switched fully off must drop FlyByWire's own autopilot output to zero exactly like a full fault");
}
