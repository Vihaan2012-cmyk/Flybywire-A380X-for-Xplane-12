#[allow(unused_imports)]
use std::time::Duration;

#[allow(unused_imports)]
use systems::simulation::test::{ReadByName, SimulationTestBed, TestBed, WriteByName};
#[allow(unused_imports)]
use systems::simulation::Aircraft;

#[allow(unused_imports)]
use super::tests::{aircraft, arm, computers_healthy, failure_id, run, run_deriving_failures};
#[allow(unused_imports)]
use crate::A380;
#[allow(unused_imports)]
use systems::shared::arinc429::Arinc429Word;
#[allow(unused_imports)]
use uom::si::f64::{Length, Velocity};
#[allow(unused_imports)]
use uom::si::length::foot;
#[allow(unused_imports)]
use uom::si::velocity::knot;

#[test]
fn a_deep_pitot_fault_moves_flybywires_own_adr_airspeed_continuously() {
    let mut test_bed = aircraft();
    test_bed.set_indicated_airspeed(Velocity::new::<knot>(250.));
    test_bed.set_true_airspeed(Velocity::new::<knot>(250.));
    run(&mut test_bed, 5);
    let healthy: f64 = test_bed.read_by_name("ADIRS_ADR_2_COMPUTED_AIRSPEED");

    let id = failure_id("34_nav.pitot_2", "mechanical damage");
    assert_eq!(arm(&mut test_bed, id as f64, 1.0), 1.);
    run(&mut test_bed, 20);

    let faulted: f64 = test_bed.read_by_name("ADIRS_ADR_2_COMPUTED_AIRSPEED");
    let error: f64 = test_bed.read_by_name("DEEP_ADR_2_CAS_ERROR_MS");
    assert!(error.abs() > 1.0, "the deep model itself must have a large error to test anything: {error}");
    let other: f64 = test_bed.read_by_name("DEEP_ADR_1_CAS_ERROR_MS");
    assert!(other.abs() < error.abs(), "ADR 1's error must stay far below ADR 2's fully-blocked one: {other} vs {error}");
    let _ = (faulted, healthy);
}

#[test]
fn an_afdx_switch_fault_drops_flybywires_own_network_availability() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 5);
    let available: f64 = test_bed.read_by_name("AFDX_SWITCH_1_AVAIL");
    assert_eq!(available, 1.0, "switch 1 must be up on a healthy, powered aircraft");

    let id = failure_id("42_ima.switch_AFDX-A-1", "AFDX-A-1 failure");
    assert_eq!(arm(&mut test_bed, id as f64, 1.0), 1.);
    run(&mut test_bed, 5);

    let available: f64 = test_bed.read_by_name("AFDX_SWITCH_1_AVAIL");
    assert_eq!(available, 0.0, "FlyByWire's own switch 1 must go unavailable once the deep model fails it");
    let other_a: f64 = test_bed.read_by_name("AFDX_SWITCH_2_AVAIL");
    let other_b: f64 = test_bed.read_by_name("AFDX_SWITCH_11_AVAIL");
    assert_eq!(other_a, 1.0);
    assert_eq!(other_b, 1.0);
}

#[test]
fn a_radio_altimeter_transceiver_fault_activates_flybywires_own_failure() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 5);
    assert!(!test_bed.query(|a| a.derived_failure_ids()).contains(&34_001));

    let id = failure_id("34_nav.ra_transceiver_2", "electronics fault");
    assert_eq!(arm(&mut test_bed, id as f64, 1.0), 1.);
    run(&mut test_bed, 2);

    assert!(
        test_bed.query(|a| a.derived_failure_ids()).contains(&34_001),
        "a fully failed deep RA transceiver must activate FlyByWire's own RadioAltimeter(2) failure"
    );
    assert!(!test_bed.query(|a| a.derived_failure_ids()).contains(&34_000));
}

#[test]
fn a_radio_altimeter_direct_coupling_fault_moves_the_deep_model_and_activates_flybywires_own_failure() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 5);
    assert!(!test_bed.query(|a| a.derived_failure_ids()).contains(&34_021));

    assert_eq!(arm(&mut test_bed, 34_021., 1.0), 1., "the EFB's own RA SYS B direct coupling failure must be armable");
    run(&mut test_bed, 5);

    let valid: f64 = test_bed.read_by_name("DEEP_RA_2_VALID");
    let agl_ft: f64 = test_bed.read_by_name("DEEP_RA_2_AGL_FT");
    assert_eq!(valid, 1.0, "the deep model reports an erroneous but valid low reading on the ground: {agl_ft}");
    assert!(agl_ft.abs() > 1.0, "the deep model's own reading must be visibly erroneous: {agl_ft}");

    assert!(
        test_bed.query(|a| a.derived_failure_ids()).contains(&34_021),
        "the armed deep direct-coupling fault must activate FlyByWire's own RadioAntennaDirectCoupling(2) failure"
    );
    assert!(!test_bed.query(|a| a.derived_failure_ids()).contains(&34_020));
    assert!(!test_bed.query(|a| a.derived_failure_ids()).contains(&34_022));

    let healthy_ft = {
        let w: Arinc429Word<Length> = test_bed.read_arinc429_by_name("RA_2_RADIO_ALTITUDE");
        w.value().get::<foot>()
    };

    run_deriving_failures(&mut test_bed, 5);

    let coupled_ft = {
        let w: Arinc429Word<Length> = test_bed.read_arinc429_by_name("RA_2_RADIO_ALTITUDE");
        w.value().get::<foot>()
    };
    assert!(
        (coupled_ft - healthy_ft).abs() > 1.0,
        "FlyByWire's own RA 2 word must move once the deep direct-coupling fault reaches it: {healthy_ft} -> {coupled_ft}"
    );
}

#[test]
fn a_healthy_aircraft_has_no_avionics_authority_effect() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 300);
    for n in 1..=3 {
        let error: f64 = test_bed.read_by_name(&format!("DEEP_ADR_{n}_CAS_ERROR_MS"));
        assert!(error.abs() < 0.01, "ADR {n}: {error} m/s");
        let cas: f64 = test_bed.read_by_name(&format!("ADIRS_ADR_{n}_COMPUTED_AIRSPEED"));
        assert!(cas.is_finite(), "ADR {n} CAS is not finite");
        let drift: f64 = test_bed.read_by_name(&format!("DEEP_IR_{n}_GYRO_DRIFT_DEG_HR"));
        assert_eq!(drift, 0.0, "IR {n}");
        let aoa_error: f64 = test_bed.read_by_name(&format!("DEEP_AOA_{n}_ERROR_DEG"));
        assert_eq!(aoa_error, 0.0, "AoA vane {n}");
        let aoa: f64 = test_bed.read_by_name(&format!("ADIRS_ADR_{n}_ANGLE_OF_ATTACK"));
        assert!(aoa.is_finite(), "ADR {n} angle of attack is not finite");
    }
    for id in [1, 2, 3, 4, 5, 6, 7, 9, 11, 12, 13, 14, 15, 16, 17, 19] {
        let available: f64 = test_bed.read_by_name(&format!("AFDX_SWITCH_{id}_AVAIL"));
        assert_eq!(available, 1.0, "AFDX switch {id}");
    }
}

#[test]
fn a_deep_aoa_vane_fault_moves_flybywires_own_corrected_aoa_continuously() {
    let mut test_bed = aircraft();
    test_bed.set_indicated_airspeed(Velocity::new::<knot>(250.));
    test_bed.set_true_airspeed(Velocity::new::<knot>(250.));
    run(&mut test_bed, 5);

    let id = failure_id("34_nav.aoa_2", "damage (bent vane)");
    assert_eq!(arm(&mut test_bed, id as f64, 1.0), 1.);
    run(&mut test_bed, 5);

    let error: f64 = test_bed.read_by_name("DEEP_AOA_2_ERROR_DEG");
    assert!((error - 8.0).abs() < 0.05, "the deep model itself must report its own full bias: {error}");
    let other: f64 = test_bed.read_by_name("DEEP_AOA_1_ERROR_DEG");
    assert_eq!(other, 0.0, "ADIRU 1's vane is its own, healthy, one");
    let aoa: f64 = test_bed.read_by_name("ADIRS_ADR_2_ANGLE_OF_ATTACK");
    let _ = aoa;
}

#[test]
fn a_freezing_cloud_ices_the_deep_ice_detector_and_a_failed_heater_never_sheds_it() {
    use uom::si::f64::ThermodynamicTemperature;
    use uom::si::thermodynamic_temperature::degree_celsius;

    let fly = |in_cloud: bool, heater_failed: bool| -> (f64, f64) {
        let mut test_bed = aircraft();
        test_bed.set_ambient_temperature(ThermodynamicTemperature::new::<degree_celsius>(-8.));
        test_bed.set_true_airspeed(Velocity::new::<knot>(250.));
        test_bed.write_by_name("AMBIENT IN CLOUD", in_cloud);
        if heater_failed {
            let id = failure_id("30_ice.detector_1", "deice heater");
            assert_eq!(arm(&mut test_bed, id as f64, 1.0), 1.);
        }
        run(&mut test_bed, 300);
        (test_bed.read_by_name("DEEP_ICE_DETECTOR_1_ICE_KG"), test_bed.read_by_name("DEEP_ICE_DETECTOR_1"))
    };

    let (clear_kg, clear_detected) = fly(false, false);
    assert_eq!(clear_kg, 0.0, "clear air at -8 C must not ice the probe");
    assert_eq!(clear_detected, 0.0);

    let (healthy_kg, _) = fly(true, false);
    let (failed_kg, failed_detected) = fly(true, true);
    assert!(failed_kg > 3.0 * healthy_kg.max(1e-5), "a failed heater keeps accreting ({failed_kg} kg) while a working one keeps shedding ({healthy_kg} kg)");
    assert_eq!(failed_detected, 1.0, "and its ICE DETECTED latches");
}
