use systems::simulation::test::{ReadByName, SimulationTestBed, TestBed, WriteByName};

use super::tests::{aircraft, arm, failure_id, run};
use crate::A380;

#[test]
fn arming_a_failure_logs_one_event_with_the_right_id_and_time() {
    let mut test_bed = aircraft();
    test_bed.write_by_name("ZULU TIME", 43_200.);
    let count_before: f64 = test_bed.read_by_name("DEEP_MAINT_LOG_COUNT");
    assert_eq!(count_before, 0., "healthy aircraft: nothing logged yet");

    let id = failure_id("91_wiring.harness_cockpit", "Chafe") as f64;
    let result = arm(&mut test_bed, id, 1.0);
    assert_eq!(result, 1., "the chafe failure should arm");
    run(&mut test_bed, 5);

    let count: f64 = test_bed.read_by_name("DEEP_MAINT_LOG_COUNT");
    let mut armed: Vec<(f64, f64)> = Vec::new();
    for k in 0..count as usize {
        let kind: f64 = test_bed.read_by_name(&format!("DEEP_MAINT_LOG_{k}_KIND"));
        if kind == 1. {
            let logged_id: f64 = test_bed.read_by_name(&format!("DEEP_MAINT_LOG_{k}_ID"));
            let time: f64 = test_bed.read_by_name(&format!("DEEP_MAINT_LOG_{k}_TIME"));
            armed.push((logged_id, time));
        }
    }
    assert_eq!(armed.len(), 1, "exactly one armed event");
    assert_eq!(armed[0].0, id, "the logged id is the armed failure's own id");
    assert_eq!(armed[0].1, 43_200., "stamped with the sim's zulu time");
}

#[test]
fn a_breaker_trip_logs_an_event() {
    let mut test_bed = aircraft();
    test_bed.write_by_name("ZULU TIME", 43_200.);
    let seq_before: f64 = test_bed.read_by_name("DEEP_MAINT_LOG_SEQ");
    assert_eq!(seq_before, 0.);

    let id = failure_id("91_wiring.harness_cockpit", "Chafe") as f64;
    let result = arm(&mut test_bed, id, 1.0);
    assert_eq!(result, 1.);
    run(&mut test_bed, 100);

    let tripped: f64 = test_bed.read_by_name("BREAKERS_TRIPPED_NOT_COMMANDED_COUNT");
    assert!(tripped >= 1., "the short should trip at least one unit on its own (precondition)");

    let seq_after: f64 = test_bed.read_by_name("DEEP_MAINT_LOG_SEQ");
    assert!(seq_after > seq_before, "the maintenance log should have grown");

    let found_trip = (0..64).any(|k| {
        let kind: f64 = test_bed.read_by_name(&format!("DEEP_MAINT_LOG_{k}_KIND"));
        kind == 5.
    });
    assert!(found_trip, "a KIND_BREAKER_TRIPPED (5.0) event should be in the log");
}

#[test]
fn a_healthy_flight_logs_nothing() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 200);

    let count: f64 = test_bed.read_by_name("DEEP_MAINT_LOG_COUNT");
    let seq: f64 = test_bed.read_by_name("DEEP_MAINT_LOG_SEQ");
    assert_eq!(count, 0., "a healthy aircraft logs no maintenance events");
    assert_eq!(seq, 0.);
}
