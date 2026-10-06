#[allow(unused_imports)]
use std::time::Duration;

#[allow(unused_imports)]
use systems::simulation::test::{ReadByName, SimulationTestBed, TestBed, WriteByName};

#[allow(unused_imports)]
use super::tests::{aircraft, computers_healthy, failure_id, run};
#[allow(unused_imports)]
use crate::A380;

const RESULT_APPLIED: f64 = 1.;

fn item_for(fbw_id: u64) -> usize {
    deep_systems::mel_catalog::MEL_FAILURES
        .iter()
        .position(|(_, ids)| ids.contains(&fbw_id))
        .unwrap_or_else(|| panic!("{fbw_id} is not MEL-catalogued"))
}

fn mel_cmd(test_bed: &mut SimulationTestBed<A380>, item: usize, action: f64) -> f64 {
    test_bed.write_by_name("DEEP_MEL_CMD_ITEM", item as f64);
    test_bed.write_by_name("DEEP_MEL_CMD", action);
    run(test_bed, 1);
    let consumed: f64 = test_bed.read_by_name("DEEP_MEL_CMD");
    assert_eq!(consumed, 0., "the command is consumed");
    test_bed.read_by_name("DEEP_MEL_CMD_RESULT")
}

#[test]
fn every_mel_catalogued_item_can_be_deferred_and_released() {
    let _serial = super::mel::serial_for_tests();
    let mut test_bed = aircraft();
    for item in 0..deep_systems::mel_catalog::MEL_FAILURES.len() {
        assert_eq!(mel_cmd(&mut test_bed, item, 1.), RESULT_APPLIED, "item {item} must be deferrable");
        assert_eq!(mel_cmd(&mut test_bed, item, 2.), RESULT_APPLIED, "item {item} must be releasable");
    }
}

#[test]
fn deferring_a_mel_item_from_the_efb_arms_it_and_marks_it_inop_releasing_clears_both() {
    let _serial = super::mel::serial_for_tests();
    let mut test_bed = aircraft();
    let item = item_for(78_001);

    let mel_inop: f64 = test_bed.read_by_name("ENG_2_REV_MEL_INOP");
    assert_eq!(mel_inop, 0., "healthy aircraft: nothing is MEL-deferred");
    assert!(!test_bed.query(|a| a.deep_systems.mel_is_deferred(78_001)));
    assert!(!test_bed.query(|a| a.deep_systems.derived_failure_ids().contains(&78_001)));

    assert_eq!(mel_cmd(&mut test_bed, item, 1.), RESULT_APPLIED, "the item is deferrable");
    run(&mut test_bed, 3);
    assert!(test_bed.query(|a| a.deep_systems.mel_is_deferred(78_001)), "deferring the item defers every fbw id it covers");
    assert!(test_bed.query(|a| a.deep_systems.derived_failure_ids().contains(&78_001)), "78_001 is activated through derived_failure_ids");
    let mel_inop: f64 = test_bed.read_by_name("ENG_2_REV_MEL_INOP");
    assert_eq!(mel_inop, 1., "the reverser's own area reads deferred_state and placards it INOP");

    assert_eq!(mel_cmd(&mut test_bed, item, 2.), RESULT_APPLIED, "releasing repairs it");
    run(&mut test_bed, 3);
    assert!(!test_bed.query(|a| a.deep_systems.mel_is_deferred(78_001)), "releasing it clears the deferral");
    assert!(!test_bed.query(|a| a.deep_systems.derived_failure_ids().contains(&78_001)));
    let mel_inop: f64 = test_bed.read_by_name("ENG_2_REV_MEL_INOP");
    assert_eq!(mel_inop, 0., "and the placard with it");
}

#[test]
fn a_mel_free_aircraft_has_zero_deferrals() {
    let _serial = super::mel::serial_for_tests();
    let mut test_bed = aircraft();
    run(&mut test_bed, 50);
    for &(_, ids) in deep_systems::mel_catalog::MEL_FAILURES {
        for &id in ids {
            assert!(!test_bed.query(|a| a.deep_systems.mel_is_deferred(id)), "{id} is deferred on a MEL-free aircraft");
        }
    }
    for eng in [2, 3] {
        let mel_inop: f64 = test_bed.read_by_name(&format!("ENG_{eng}_REV_MEL_INOP"));
        assert_eq!(mel_inop, 0., "engine {eng} reverser MEL INOP on a healthy, MEL-free aircraft");
    }
}

#[test]
fn a_mel_free_aircraft_has_zero_mel_effects_through_a_whole_flight() {
    let _serial = super::mel::serial_for_tests();
    use std::time::Duration;
    use uom::si::{f64::*, length::foot, velocity::knot};

    fn fly_at(test_bed: &mut SimulationTestBed<A380>, altitude: Length, tas_kt: f64) {
        test_bed.set_pressure_altitude(altitude);
        test_bed.set_ambient_pressure(systems::shared::InternationalStandardAtmosphere::pressure_at_altitude(altitude));
        test_bed.set_ambient_temperature(systems::shared::InternationalStandardAtmosphere::temperature_at_altitude(altitude));
        test_bed.set_true_airspeed(Velocity::new::<knot>(tas_kt));
        test_bed.set_indicated_airspeed(Velocity::new::<knot>(tas_kt.min(300.)));
    }
    fn engines(test_bed: &mut SimulationTestBed<A380>, n1: f64, n2: f64, n3: f64) {
        for n in 1..=4 {
            test_bed.write_by_name(&format!("ENGINE_STATE:{n}"), 1.);
            test_bed.write_by_name(&format!("TURB ENG CORRECTED N1:{n}"), n1);
            test_bed.write_by_name(&format!("TURB ENG CORRECTED N2:{n}"), n2);
            test_bed.write_by_name(&format!("ENGINE_N2:{n}"), n2);
            test_bed.write_by_name(&format!("ENGINE_N2_HEALTHY:{n}"), n2);
            test_bed.write_by_name(&format!("ENGINE_N3:{n}"), n3);
            test_bed.write_by_name(&format!("ENGINE_N3_HEALTHY:{n}"), n3);
        }
    }
    fn assert_mel_free(test_bed: &mut SimulationTestBed<A380>, phase: &str) {
        for &(_, ids) in deep_systems::mel_catalog::MEL_FAILURES {
            for &id in ids {
                assert!(!test_bed.query(|a| a.deep_systems.mel_is_deferred(id)), "{phase}: {id} reads as MEL-deferred with nothing deferred");
            }
        }
        for eng in [2, 3] {
            let mel_inop: f64 = test_bed.read_by_name(&format!("ENG_{eng}_REV_MEL_INOP"));
            assert_eq!(mel_inop, 0., "{phase}: engine {eng} reverser MEL INOP with nothing deferred");
        }
    }

    let mut test_bed = SimulationTestBed::new(A380::new);
    computers_healthy(&mut test_bed);
    test_bed.set_on_ground(true);
    for id in 1..=4 {
        test_bed.write_by_name(&format!("OVHD_ELEC_BAT_{id}_PB_IS_AUTO"), true);
    }
    test_bed.run_multiple_frames(Duration::from_secs(60));
    assert_mel_free(&mut test_bed, "cold and dark");

    for i in 1..=4 {
        test_bed.write_by_name(&format!("EXT_PWR_AVAIL:{i}"), true);
        test_bed.write_by_name(&format!("OVHD_ELEC_EXT_PWR_{i}_PB_IS_ON"), true);
    }
    test_bed.write_by_name("CONFIG_ADIRS_IR_ALIGN_TIME", 1.);
    for n in 1..=3 {
        test_bed.write_by_name(&format!("OVHD_ADIRS_IR_{n}_MODE_SELECTOR_KNOB"), 1.);
    }
    test_bed.run_multiple_frames(Duration::from_secs(600));
    assert_mel_free(&mut test_bed, "external power");

    engines(&mut test_bed, 20., 65., 70.);
    for i in 1..=4 {
        test_bed.write_by_name(&format!("OVHD_ELEC_EXT_PWR_{i}_PB_IS_ON"), false);
        test_bed.write_by_name(&format!("EXT_PWR_AVAIL:{i}"), false);
    }
    test_bed.run_multiple_frames(Duration::from_secs(600));
    assert_mel_free(&mut test_bed, "engines at idle");

    test_bed.set_on_ground(false);
    engines(&mut test_bed, 85., 95., 97.);
    for minute in 1..=20 {
        fly_at(&mut test_bed, Length::new::<foot>(1750. * minute as f64), 250. + 10. * minute as f64);
        test_bed.run_multiple_frames(Duration::from_secs(60));
    }
    fly_at(&mut test_bed, Length::new::<foot>(35000.), 490.);
    test_bed.run_multiple_frames(Duration::from_secs(1200));
    assert_mel_free(&mut test_bed, "cruise FL350");
}
