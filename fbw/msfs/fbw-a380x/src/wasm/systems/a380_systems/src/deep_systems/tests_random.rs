use deep_systems::scripted_failures::ArmCondition;
use systems::simulation::test::{ReadByName, TestBed, WriteByName};

use super::tests::{aircraft, failure_id, run};

#[test]
fn defaults_arm_nothing() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 200);
    let armed: f64 = test_bed.read_by_name("DEEP_FAILURES_ARMED_COUNT");
    assert_eq!(armed, 0., "the random engine is off by default and nothing was scripted");
}

#[test]
fn a_random_failure_fires_under_a_forced_rate() {
    let mut test_bed = aircraft();
    test_bed.write_by_name("DEEP_RANDOM_FAILURES_ENABLED", true);
    test_bed.write_by_name("DEEP_RANDOM_FAILURES_RATE_MULTIPLIER", 1.0e9);
    run(&mut test_bed, 20);
    let armed: f64 = test_bed.read_by_name("DEEP_FAILURES_ARMED_COUNT");
    assert!(armed > 0., "a component should have failed under a forced, huge hazard rate");
}

#[test]
fn a_scripted_failure_fires_at_its_trigger() {
    let mut test_bed = aircraft();
    let id = failure_id("49_apu.core_compressor", "erosion");
    let already: f64 = test_bed.read_by_name("DEEP_AIRFRAME_HOURS");
    let threshold = already + 0.002;
    test_bed.command(|a| a.deep_systems.schedule_scripted(id, ArmCondition::ElapsedHours(threshold)));

    run(&mut test_bed, 1);
    assert!(!test_bed.query(|a| a.deep_systems.armed_failures().contains_key(&id)), "should not fire before its elapsed-hours threshold");

    run(&mut test_bed, 100);
    assert!(test_bed.query(|a| a.deep_systems.armed_failures().contains_key(&id)), "should fire once elapsed real flight hours pass the threshold");
}

#[test]
fn wear_accumulates_a_flight_cycle() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 5);
    let cycles_before: f64 = test_bed.read_by_name("DEEP_WEAR_AIRFRAME_CYCLES");
    assert_eq!(cycles_before, 0., "no cycle yet on a ground start");

    test_bed.set_on_ground(false);
    run(&mut test_bed, 5);
    test_bed.set_on_ground(true);
    run(&mut test_bed, 5);

    let cycles_after: f64 = test_bed.read_by_name("DEEP_WEAR_AIRFRAME_CYCLES");
    assert_eq!(cycles_after, 1., "one airborne-then-touchdown transition counts one cycle");
}

fn schedule(test_bed: &mut systems::simulation::test::SimulationTestBed<crate::A380>, id: f64, kind: f64, value: f64, magnitude: f64, external: bool) {
    test_bed.write_by_name("DEEP_SCRIPTED_CMD_EXTERNAL", if external { 1. } else { 0. });
    test_bed.write_by_name("DEEP_SCRIPTED_CMD_MAGNITUDE", magnitude);
    test_bed.write_by_name("DEEP_SCRIPTED_CMD_VALUE", value);
    test_bed.write_by_name("DEEP_SCRIPTED_CMD_KIND", kind);
    test_bed.write_by_name("DEEP_SCRIPTED_CMD_ID", id);
}

#[test]
fn a_failure_scheduled_from_the_cockpit_shows_as_pending_then_arms_at_its_own_severity() {
    let mut test_bed = aircraft();
    let id = failure_id("49_apu.core_compressor", "erosion");
    schedule(&mut test_bed, id as f64, 5., 3., 0.4, false);
    run(&mut test_bed, 1);
    let result: f64 = test_bed.read_by_name("DEEP_SCRIPTED_CMD_RESULT");
    assert_eq!(result, 1.);
    let pending_id: f64 = test_bed.read_by_name("DEEP_SCRIPTED_PENDING_0_ID");
    let pending_kind: f64 = test_bed.read_by_name("DEEP_SCRIPTED_PENDING_0_KIND");
    assert_eq!(pending_id, id as f64);
    assert_eq!(pending_kind, 5.);
    assert!(!test_bed.query(|a| a.deep_systems.armed_failures().contains_key(&id)));

    run(&mut test_bed, 40);
    assert_eq!(test_bed.query(|a| a.deep_systems.armed_failures().get(&id).copied()), Some(0.4));
    let pending: f64 = test_bed.read_by_name("DEEP_SCRIPTED_FAILURES_PENDING_COUNT");
    assert_eq!(pending, 0.);
}

#[test]
fn a_liftoff_trigger_waits_on_the_ground_and_fires_once_airborne() {
    let mut test_bed = aircraft();
    test_bed.set_on_ground(true);
    run(&mut test_bed, 2);
    let id = failure_id("49_apu.core_compressor", "erosion");
    schedule(&mut test_bed, id as f64, 4., 1., 1., false);
    run(&mut test_bed, 50);
    assert!(!test_bed.query(|a| a.deep_systems.armed_failures().contains_key(&id)), "five seconds on the ground is not after liftoff");

    test_bed.set_on_ground(false);
    run(&mut test_bed, 20);
    assert!(test_bed.query(|a| a.deep_systems.armed_failures().contains_key(&id)));
}

#[test]
fn a_flybywire_failure_is_handed_to_the_efb_until_it_acknowledges() {
    let mut test_bed = aircraft();
    schedule(&mut test_bed, 26_002., 5., 0., 1., true);
    run(&mut test_bed, 2);
    let fire: f64 = test_bed.read_by_name("DEEP_SCRIPTED_EXTERNAL_FIRE_ID");
    assert_eq!(fire, 26_002.);
    run(&mut test_bed, 5);
    let still: f64 = test_bed.read_by_name("DEEP_SCRIPTED_EXTERNAL_FIRE_ID");
    assert_eq!(still, 26_002., "held until the EFB says it activated it");

    test_bed.write_by_name("DEEP_SCRIPTED_EXTERNAL_FIRE_ACK", 26_002.);
    run(&mut test_bed, 2);
    let after: f64 = test_bed.read_by_name("DEEP_SCRIPTED_EXTERNAL_FIRE_ID");
    let ack: f64 = test_bed.read_by_name("DEEP_SCRIPTED_EXTERNAL_FIRE_ACK");
    assert_eq!(after, 0.);
    assert_eq!(ack, 0.);
}

#[test]
fn an_unmodelled_id_is_rejected_and_a_cancel_removes_a_pending_failure() {
    let mut test_bed = aircraft();
    schedule(&mut test_bed, 999_999_999., 1., 140., 1., false);
    run(&mut test_bed, 1);
    let result: f64 = test_bed.read_by_name("DEEP_SCRIPTED_CMD_RESULT");
    assert_eq!(result, 3.);

    let id = failure_id("49_apu.core_compressor", "erosion");
    schedule(&mut test_bed, id as f64, 1., 140., 1., false);
    run(&mut test_bed, 1);
    schedule(&mut test_bed, id as f64, 0., 0., 0., false);
    run(&mut test_bed, 1);
    let pending: f64 = test_bed.read_by_name("DEEP_SCRIPTED_FAILURES_PENDING_COUNT");
    assert_eq!(pending, 0.);
}
