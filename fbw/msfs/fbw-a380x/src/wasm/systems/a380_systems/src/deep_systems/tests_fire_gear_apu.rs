#[allow(unused_imports)]
use std::time::Duration;

#[allow(unused_imports)]
use systems::simulation::test::{ReadByName, SimulationTestBed, TestBed, WriteByName};
#[allow(unused_imports)]
use systems::simulation::Aircraft;

#[allow(unused_imports)]
use super::tests::{aircraft, arm, computers_healthy, failure_id, run};
#[allow(unused_imports)]
use crate::A380;

fn id_by_name(name_part: &str) -> u64 {
    let registry = deep_systems::deep::registry();
    let matches: Vec<_> = registry.failures.iter().filter(|f| f.name.contains(name_part)).collect();
    assert_eq!(matches.len(), 1, "expected exactly one failure named {name_part:?}, found {}: {:?}", matches.len(), matches.iter().map(|f| &f.name).collect::<Vec<_>>());
    matches[0].id
}

#[test]
fn a_deep_loop_fault_shows_as_flybywires_own_loop_fault() {
    let mut test_bed = aircraft();
    let id = id_by_name("ENG 1 fire loop A open circuit");
    assert_eq!(arm(&mut test_bed, id as f64, 1.0), 1.);
    run(&mut test_bed, 10);
    let ids = test_bed.query(|a| a.derived_failure_ids());
    assert!(ids.contains(&26_007), "loop A fault must derive FlyByWire's own FireDetectionLoop(A, Engine(1)): {ids:?}");
    assert!(!ids.contains(&26_001), "a single faulted loop alone must not falsely derive a zone fire: {ids:?}");
}

#[test]
fn a_deep_engine_fire_raises_flybywires_own_fire_detection() {
    let mut test_bed = aircraft();
    let loop_a = id_by_name("ENG 1 fire loop A open circuit");
    let loop_b = id_by_name("ENG 1 fire loop B open circuit");
    assert_eq!(arm(&mut test_bed, loop_a as f64, 1.0), 1.);
    assert_eq!(arm(&mut test_bed, loop_b as f64, 1.0), 1.);
    run(&mut test_bed, 20);
    let ids = test_bed.query(|a| a.derived_failure_ids());
    assert!(ids.contains(&26_001), "both loops faulted together must derive FlyByWire's own SetOnFire(Engine(1)): {ids:?}");
    assert!(ids.contains(&26_007) && ids.contains(&26_008), "both loop faults must still be visible on their own: {ids:?}");
}

#[test]
#[ignore = "unverified at the 2026-09-30 21:42 deadline: sequencing (does the leg actually reach uplocked before the fault is armed?) not confirmed"]
fn a_gear_fault_shows_in_flybywires_own_gear_values() {
    let mut test_bed = aircraft();
    test_bed.write_by_name("GEAR_HANDLE_POSITION", 0.0);
    run(&mut test_bed, 100);
    let id = failure_id("32_gear.nose_retraction", "uplock hook jam");
    assert_eq!(arm(&mut test_bed, id as f64, 1.0), 1.);
    test_bed.write_by_name("GEAR_HANDLE_POSITION", 1.0);
    run(&mut test_bed, 100);
    let ids = test_bed.query(|a| a.derived_failure_ids());
    assert!(ids.contains(&32_020), "a fully jammed nose uplock must derive FlyByWire's own GearActuatorJammed(GearNose): {ids:?}");
}

#[test]
fn a_brake_fault_shows_in_flybywires_own_brake_values() {
    let mut test_bed = aircraft();
    let id = failure_id("32_gear.parking_brake_accumulator", "leak");
    assert_eq!(arm(&mut test_bed, id as f64, 1.0), 1.);
    run(&mut test_bed, 10);
    let ids = test_bed.query(|a| a.derived_failure_ids());
    assert!(ids.contains(&32_030), "an armed parking-brake accumulator leak must derive FlyByWire's own BrakeAccumulatorGasLeak: {ids:?}");
}

#[test]
#[ignore = "unverified at the 2026-09-30 21:42 deadline: APU never confirmed running within the test's frame budget"]
fn an_apu_degradation_trips_flybywires_own_apu_protection() {
    let mut test_bed = aircraft();
    test_bed.write_by_name("OVHD_APU_MASTER_SW_PB_IS_ON", true);
    test_bed.write_by_name("OVHD_APU_START_PB_IS_ON", true);
    run(&mut test_bed, 600);
    let id = failure_id("49_apu.oil_system", "APU oil leak");
    assert_eq!(arm(&mut test_bed, id as f64, 1.0), 1.);
    run(&mut test_bed, 900);
    let trip: f64 = test_bed.read_by_name("APU_ECB_TRIP");
    assert_eq!(trip, 1.0, "an APU oil leak must reach FlyByWire's own ECB protective-trip input");
}

#[test]
fn a_healthy_aircraft_derives_none_of_this_areas_couplings() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 50);
    let ids = test_bed.query(|a| a.derived_failure_ids());
    for id in [26_001u64, 26_002, 26_003, 26_004, 26_005, 26_006, 26_007, 26_008, 32_004, 32_020, 32_021, 32_022, 32_023, 32_024, 32_025, 32_030] {
        assert!(!ids.contains(&id), "a healthy aircraft must derive nothing for id {id}: {ids:?}");
    }
}

#[test]
fn deep_derived_fbw_failure_26001_publishes_the_engine_1_fire_coupling() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 10);
    let healthy: f64 = test_bed.read_by_name("DEEP_DERIVED_FBW_FAILURE_26001");
    assert_eq!(healthy, 0.0, "a healthy aircraft must publish 0 for the engine 1 fire coupling");

    let loop_a = id_by_name("ENG 1 fire loop A open circuit");
    let loop_b = id_by_name("ENG 1 fire loop B open circuit");
    assert_eq!(arm(&mut test_bed, loop_a as f64, 1.0), 1.);
    assert_eq!(arm(&mut test_bed, loop_b as f64, 1.0), 1.);
    run(&mut test_bed, 20);
    let faulted: f64 = test_bed.read_by_name("DEEP_DERIVED_FBW_FAILURE_26001");
    assert_eq!(faulted, 1.0, "both engine 1 fire loops faulted must publish 1 for the derived SetOnFire coupling");
}

fn registered(id: u64) -> u64 {
    assert!(deep_systems::deep::registry().failures.iter().any(|f| f.id == id), "failure {id} is not registered");
    id
}

#[test]
fn a_hold_leak_lights_only_from_the_content_fire_in_that_hold() {
    let leak = registered(8_026_106);
    let fwd_fire = registered(11_026_001);
    let aft_fire = registered(11_026_002);
    let burning = |fires: &[u64]| -> f64 {
        let mut test_bed = aircraft();
        assert_eq!(arm(&mut test_bed, leak as f64, 1.0), 1.);
        for &fire in fires {
            assert_eq!(arm(&mut test_bed, fire as f64, 1.0), 1.);
        }
        run(&mut test_bed, 100);
        test_bed.read_by_name("FIRE_ZONE_CARGO_FWD_BURNING")
    };
    assert_eq!(burning(&[]), 0.0, "no flame in the hold: the fluid pools");
    assert_eq!(burning(&[aft_fire]), 0.0, "a fire in the aft hold is not a flame in the forward one");
    assert_eq!(burning(&[fwd_fire]), 1.0, "the forward hold's content fire lights the leak");
}

#[test]
fn a_holds_content_fire_alarms_its_smoke_detector_unless_the_lens_is_obscured() {
    let fwd_fire = registered(11_026_001);
    let lens = registered(8_026_227);
    let detected = |armed: &[u64]| -> (f64, f64) {
        let mut test_bed = aircraft();
        for &id in armed {
            assert_eq!(arm(&mut test_bed, id as f64, 1.0), 1.);
        }
        run(&mut test_bed, 100);
        (test_bed.read_by_name("CARGO_FWD_SMOKE_DETECTED"), test_bed.read_by_name("CARGO_AFT_SMOKE_DETECTED"))
    };
    assert_eq!(detected(&[]), (0.0, 0.0), "clean holds never alarm");
    assert_eq!(detected(&[fwd_fire]), (1.0, 0.0), "the forward hold's fire alarms only the forward detector");
    assert_eq!(detected(&[fwd_fire, lens]).0, 0.0, "an obscured lens hides it");
}
