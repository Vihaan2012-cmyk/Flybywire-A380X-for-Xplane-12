#[allow(unused_imports)]
use systems::simulation::test::{ReadByName, SimulationTestBed, TestBed};

#[allow(unused_imports)]
use super::tests::{aircraft, arm, failure_id, run};
#[allow(unused_imports)]
use crate::A380;

#[test]
fn a_healthy_aircraft_is_unchanged_by_the_oxygen_authority() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 50);

    let crew_pressure: f64 = test_bed.read_by_name("OXYGEN_CREW_PRESSURE_PSI");
    let crew_supply_available: f64 = test_bed.read_by_name("OXYGEN_CREW_SUPPLY_AVAILABLE");
    let pax_masks_deployed: f64 = test_bed.read_by_name("OXYGEN_PAX_MASKS_DEPLOYED");

    assert!(
        (crew_pressure - 1850.0).abs() < 5.0,
        "a healthy, serviced crew bottle must read its full charge: {crew_pressure} psi"
    );
    assert_eq!(crew_supply_available, 1.0, "a healthy aircraft's crew oxygen supply must be available");
    assert_eq!(pax_masks_deployed, 0.0, "a healthy aircraft must not show the passenger masks deployed");
}

#[test]
fn a_deep_crew_cylinder_leak_sags_flybywires_own_oxygen_pressure_with_severity() {
    let leak_id = failure_id("35_oxy.crew_cylinder", "Crew oxygen cylinder leak");

    let mut healthy = aircraft();
    run(&mut healthy, 300);
    let healthy_pressure: f64 = healthy.read_by_name("OXYGEN_CREW_PRESSURE_PSI");

    let mut mild = aircraft();
    arm(&mut mild, leak_id as f64, 0.3);
    run(&mut mild, 300);
    let mild_pressure: f64 = mild.read_by_name("OXYGEN_CREW_PRESSURE_PSI");

    let mut severe = aircraft();
    arm(&mut severe, leak_id as f64, 1.0);
    run(&mut severe, 300);
    let severe_pressure: f64 = severe.read_by_name("OXYGEN_CREW_PRESSURE_PSI");

    assert!(
        (healthy_pressure - 1850.0).abs() < 5.0,
        "setup: a healthy crew bottle should hold its full charge over 30 s: {healthy_pressure} psi"
    );
    assert!(
        mild_pressure < healthy_pressure - 1.0,
        "a 0.3 cylinder leak must measurably sag FlyByWire's own crew oxygen pressure: healthy {healthy_pressure} psi, mild {mild_pressure} psi"
    );
    assert!(
        severe_pressure < mild_pressure,
        "a full leak must drain the bottle faster than a mild one over the same time, proving the bridge scales with magnitude rather than tripping one fixed value: mild {mild_pressure} psi, severe {severe_pressure} psi"
    );
    assert!(severe_pressure >= 0.0, "pressure must not go negative: {severe_pressure} psi");
}
