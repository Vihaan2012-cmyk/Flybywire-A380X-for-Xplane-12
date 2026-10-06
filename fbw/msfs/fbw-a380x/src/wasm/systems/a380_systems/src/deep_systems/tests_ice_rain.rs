#[allow(unused_imports)]
use systems::simulation::test::{ReadByName, SimulationTestBed, TestBed, WriteByName};

#[allow(unused_imports)]
use super::tests::{aircraft, arm, failure_id, run};
#[allow(unused_imports)]
use crate::A380;

#[test]
fn a_healthy_aircraft_is_unchanged_by_the_ice_rain_authority() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 50);

    let wing_fault: f64 = test_bed.read_by_name("OVHD_ANTI_ICE_WING_PB_HAS_FAULT");
    let probe_window_fault: f64 =
        test_bed.read_by_name("OVHD_ANTI_ICE_PROBE_WINDOW_HEAT_PB_HAS_FAULT");
    assert_eq!(wing_fault, 0.0, "no deep ice/rain failure is armed; the WING ANTI ICE legend must stay dark");
    assert_eq!(
        probe_window_fault, 0.0,
        "no deep ice/rain failure is armed; the PROBE/WINDOW HEAT legend must stay dark"
    );
}

#[test]
fn a_deep_wing_anti_ice_valve_fault_lights_flybywires_own_ohp_legend_with_severity() {
    let mut healthy = aircraft();
    run(&mut healthy, 500);
    let healthy_fault: f64 = healthy.read_by_name("OVHD_ANTI_ICE_WING_PB_HAS_FAULT");
    assert_eq!(healthy_fault, 0.0, "baseline: no failure armed");

    let mut faulted = aircraft();
    let id = failure_id("30_ice.wing_l_anti_ice_valve", "stuck open") as f64;
    arm(&mut faulted, id, 1.0);
    run(&mut faulted, 5000);
    let deep_overheat_before: f64 = faulted.read_by_name("ANTI_ICE_WING_L_OVERHEAT");
    run(&mut faulted, 20);
    let faulted_fault: f64 = faulted.read_by_name("OVHD_ANTI_ICE_WING_PB_HAS_FAULT");
    assert_eq!(deep_overheat_before, 1.0, "setup: the stuck-open valve must overheat the leading edge within 500 s");
    let deep_overheat: f64 = faulted.read_by_name("ANTI_ICE_WING_L_OVERHEAT");

    assert!(
        faulted_fault >= healthy_fault,
        "a stuck-open wing anti-ice valve must never leave the OHP legend dimmer than healthy"
    );
    assert_eq!(
        faulted_fault, deep_overheat,
        "the OHP legend must track the deep model's own overheat verdict exactly -- it is \
         the authority, not a shadow (got legend {faulted_fault}, deep verdict {deep_overheat})"
    );
}

#[test]
fn a_rain_removal_failure_reaches_the_wiper_authority() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 5);
    let healthy: f64 = test_bed.read_by_name("DEEP_FAILURE_8030049_ACTIVE");
    assert_eq!(healthy, 0.);
    assert_eq!(arm(&mut test_bed, 8030049., 1.), 1.);
    run(&mut test_bed, 5);
    let left: f64 = test_bed.read_by_name("DEEP_FAILURE_8030049_ACTIVE");
    let right: f64 = test_bed.read_by_name("DEEP_FAILURE_8030050_ACTIVE");
    assert_eq!((left, right), (1., 0.), "only the left wiper is stopped");
}
