#[allow(unused_imports)]
use std::time::Duration;

#[allow(unused_imports)]
use systems::shared::InternationalStandardAtmosphere;
#[allow(unused_imports)]
use systems::simulation::test::{ReadByName, SimulationTestBed, TestBed, WriteByName};
#[allow(unused_imports)]
use uom::si::{f64::*, length::foot, velocity::knot};

#[allow(unused_imports)]
use super::tests::{aircraft, arm, computers_healthy, failure_id, run};
#[allow(unused_imports)]
use crate::A380;

fn electrical(name: &str) -> bool {
    name.starts_with("ELEC_")
        || name.starts_with("BKR_")
        || name.starts_with("BREAKERS_")
        || name.starts_with("WIRING_")
        || name.starts_with("ELMS_")
}

#[test]
fn feed_tank_1_main_pump_breaker_raises_its_gate() {
    let mut test_bed = aircraft();
    let gate: f64 = test_bed.read_by_name("ELEC_FUEL_PUMP_FEED1_MAIN_BREAKER_OPEN");
    assert_eq!(gate, 0., "healthy aircraft: the gate starts closed");

    test_bed.write_by_name("BKR_FUEL_PUMP_FEED1_MAIN_CMD", 1.);
    run(&mut test_bed, 4);
    let gate: f64 = test_bed.read_by_name("ELEC_FUEL_PUMP_FEED1_MAIN_BREAKER_OPEN");
    let current: f64 = test_bed.read_by_name("BKR_FUEL_PUMP_FEED1_MAIN_CURRENT_A");
    assert_eq!(gate, 1., "opening the unit should raise its gate");
    assert_eq!(current, 0., "an open unit carries no current");

    test_bed.write_by_name("BKR_FUEL_PUMP_FEED1_MAIN_CMD", 2.);
    run(&mut test_bed, 4);
    let gate: f64 = test_bed.read_by_name("ELEC_FUEL_PUMP_FEED1_MAIN_BREAKER_OPEN");
    assert_eq!(gate, 0., "closing the unit should drop the gate again");
}

#[test]
fn capt_pfd_du_breaker_gate_reflects_the_unit() {
    let mut test_bed = aircraft();
    let gate: f64 = test_bed.read_by_name("ELEC_CAPT_PFD_DU_BREAKER_OPEN");
    assert_eq!(gate, 0.);

    test_bed.write_by_name("BKR_CAPT_PFD_DU_CMD", 1.);
    run(&mut test_bed, 4);
    let gate: f64 = test_bed.read_by_name("ELEC_CAPT_PFD_DU_BREAKER_OPEN");
    assert_eq!(gate, 1., "CdsDisplayUnit.tsx reads exactly this variable to blank the captain's PFD");
}

#[test]
fn capt_nd_du_needs_both_feeds_open_before_its_gate_trips() {
    let mut test_bed = aircraft();
    test_bed.write_by_name("BKR_CAPT_ND_DU_NORMAL_BKR_CMD", 1.);
    run(&mut test_bed, 4);
    let gate: f64 = test_bed.read_by_name("ELEC_CAPT_ND_DU_BREAKER_OPEN");
    assert_eq!(gate, 0., "the ND still has its second feed");

    test_bed.write_by_name("BKR_CAPT_ND_DU_2ND_BKR_CMD", 1.);
    run(&mut test_bed, 4);
    let gate: f64 = test_bed.read_by_name("ELEC_CAPT_ND_DU_BREAKER_OPEN");
    assert_eq!(gate, 1., "both feeds are now open");
}

#[test]
fn fms_breakers_need_both_feeds_open_before_their_gate_trips_and_cuts_power() {
    for (n, fmc) in [(1, "A"), (2, "B"), (3, "C")] {
        let mut test_bed = aircraft();
        let load = format!("fms-{n}");
        let gate_name = format!("ELEC_FMS_{n}_BREAKER_OPEN");
        let powered_name = format!("ELEC_LOAD_{load}_POWERED");

        let gate: f64 = test_bed.read_by_name(&gate_name);
        assert_eq!(gate, 0.);

        test_bed.write_by_name(&format!("BKR_FMS_{n}_NORMAL_BKR_CMD"), 1.);
        run(&mut test_bed, 4);
        let gate: f64 = test_bed.read_by_name(&gate_name);
        assert_eq!(gate, 0., "FMC-{fmc} still has its second (DC ESS) feed");

        test_bed.write_by_name(&format!("BKR_FMS_{n}_2ND_BKR_CMD"), 1.);
        run(&mut test_bed, 4);
        let gate: f64 = test_bed.read_by_name(&gate_name);
        assert_eq!(gate, 1., "FlightManagementComputer.ts/DummyFlightManagementComputer.ts read exactly this variable to set L:A32NX_FMC_{fmc}_IS_HEALTHY false");
        let powered: f64 = test_bed.read_by_name(&powered_name);
        assert_eq!(powered, 0., "FMC-{fmc} must show no power once both feeds are open");
    }
}

#[test]
fn adirs_breakers_need_both_feeds_open_before_their_gate_trips_and_cuts_power() {
    for n in 1..=3 {
        let mut test_bed = aircraft();
        let load = format!("adirs-{n}");
        let gate_name = format!("ELEC_ADIRS_{n}_BREAKER_OPEN");
        let powered_name = format!("ELEC_LOAD_{load}_POWERED");

        let gate: f64 = test_bed.read_by_name(&gate_name);
        assert_eq!(gate, 0.);

        test_bed.write_by_name(&format!("BKR_ADIRS_{n}_NORMAL_BKR_CMD"), 1.);
        run(&mut test_bed, 4);
        let gate: f64 = test_bed.read_by_name(&gate_name);
        assert_eq!(gate, 0., "ADIRU {n} still has its second (DC ESS) feed");

        test_bed.write_by_name(&format!("BKR_ADIRS_{n}_2ND_BKR_CMD"), 1.);
        run(&mut test_bed, 4);
        let gate: f64 = test_bed.read_by_name(&gate_name);
        assert_eq!(gate, 1., "FwsCore.ts reads exactly this variable as adiru{n}Unpowered to fault its ADR/IR {n}");
        let powered: f64 = test_bed.read_by_name(&powered_name);
        assert_eq!(powered, 0., "ADIRU {n} must show no power once both feeds are open");
    }
}

#[test]
fn tcas_breaker_needs_both_feeds_open_before_its_gate_trips_and_cuts_power() {
    let mut test_bed = aircraft();
    let gate: f64 = test_bed.read_by_name("ELEC_TCAS_BREAKER_OPEN");
    assert_eq!(gate, 0.);

    test_bed.write_by_name("BKR_TCAS_NORMAL_BKR_CMD", 1.);
    run(&mut test_bed, 4);
    let gate: f64 = test_bed.read_by_name("ELEC_TCAS_BREAKER_OPEN");
    assert_eq!(gate, 0., "TCAS still has its second (DC 2) feed");

    test_bed.write_by_name("BKR_TCAS_2ND_BKR_CMD", 1.);
    run(&mut test_bed, 4);
    let gate: f64 = test_bed.read_by_name("ELEC_TCAS_BREAKER_OPEN");
    assert_eq!(gate, 1., "LegacyTcasComputer.ts reads exactly this variable to clear tcasPower and set L:A32NX_TCAS_FAULT");
    let powered: f64 = test_bed.read_by_name("ELEC_LOAD_tcas_POWERED");
    assert_eq!(powered, 0., "TCAS must show no power once both feeds are open");
}

#[test]
fn avionics_fan_1_is_a_deep_only_consumer_not_yet_gated_to_flybywire() {
    let mut test_bed = aircraft();
    let before = test_bed.query(|a| a.deep_systems.snapshot());
    test_bed.write_by_name("BKR_AVIONICS_FAN_1_CMD", 1.);
    run(&mut test_bed, 30);
    let after = test_bed.query(|a| a.deep_systems.snapshot());

    let fbw_visible = after
        .iter()
        .any(|(k, v)| (k.starts_with("GATE ") || k.starts_with("FBW FAILURE ")) && before.get(k) != Some(v));
    assert!(!fbw_visible, "avionics-fan-1 has no FlyByWire gate or failure yet");

    let deep_changed = after
        .iter()
        .any(|(k, v)| !k.starts_with("GATE ") && !k.starts_with("FBW FAILURE ") && !electrical(k) && before.get(k) != Some(v));
    assert!(deep_changed, "opening avionics-fan-1 should still move some deep-model output");
}

#[test]
fn a_wiring_short_trips_a_breaker_and_drops_its_flybywire_gate() {
    let mut test_bed = aircraft();
    let tripped_before: f64 = test_bed.read_by_name("BREAKERS_TRIPPED_NOT_COMMANDED_COUNT");
    assert_eq!(tripped_before, 0., "healthy aircraft: nothing tripped on its own");
    let before = test_bed.query(|a| a.deep_systems.snapshot());

    let id = failure_id("91_wiring.harness_cockpit", "Chafe") as f64;
    let result = arm(&mut test_bed, id, 1.0);
    assert_eq!(result, 1., "the chafe failure should arm");
    run(&mut test_bed, 100);

    let worst: f64 = test_bed.read_by_name("WIRING_ZONE_COCKPIT_WORST_FAULT_SEVERITY");
    assert!(worst > 0.9, "the wiring model should publish the full-severity fault back: {worst}");

    let tripped_after: f64 = test_bed.read_by_name("BREAKERS_TRIPPED_NOT_COMMANDED_COUNT");
    assert!(tripped_after >= 1., "a bolted short to structure should trip at least one unit on its own");

    let after = test_bed.query(|a| a.deep_systems.snapshot());
    let newly_open_gates: Vec<&String> = after
        .iter()
        .filter(|(k, v)| k.starts_with("GATE ") && **v == 1. && before.get(*k) != Some(v))
        .map(|(k, _)| k)
        .collect();
    assert!(!newly_open_gates.is_empty(), "the short should cut at least one FlyByWire-gated consumer: none did");

    arm(&mut test_bed, -1., 0.);
}

#[test]
fn a_whole_flight_trips_no_breaker_and_keeps_readings_sane() {
    let mut test_bed = aircraft();
    test_bed.set_on_ground(true);

    let sample_currents = ["BKR_CAB_FAN_1_CURRENT_A", "BKR_FUEL_PUMP_1_CURRENT_A", "BKR_HYD_EPUMP_GA_CURRENT_A"];
    let sample_wiring_severity = [
        "WIRING_ZONE_COCKPIT_WORST_FAULT_SEVERITY",
        "WIRING_ZONE_ENGINE_1_WORST_FAULT_SEVERITY",
        "WIRING_ZONE_MAIN_AVIONICS_WORST_FAULT_SEVERITY",
    ];

    let check = |test_bed: &mut SimulationTestBed<A380>, name: &str| {
        let tripped: f64 = test_bed.read_by_name("BREAKERS_TRIPPED_NOT_COMMANDED_COUNT");
        assert_eq!(tripped, 0., "{name}: a breaker tripped with nothing wrong");
        for var in sample_currents {
            let a: f64 = test_bed.read_by_name(var);
            assert!(a.is_finite() && (0. ..2000.).contains(&a), "{name}: {var} = {a}");
        }
        for var in sample_wiring_severity {
            let s: f64 = test_bed.read_by_name(var);
            assert_eq!(s, 0., "{name}: {var} should stay healthy on a healthy airframe, read {s}");
        }
    };

    for id in 1..=4 {
        test_bed.write_by_name(&format!("OVHD_ELEC_BAT_{id}_PB_IS_AUTO"), true);
    }
    run(&mut test_bed, 100);
    check(&mut test_bed, "cold and dark");

    for id in 1..=4 {
        test_bed.write_by_name(&format!("EXT_PWR_AVAIL:{id}"), true);
        test_bed.write_by_name(&format!("OVHD_ELEC_EXT_PWR_{id}_PB_IS_ON"), true);
    }
    test_bed.write_by_name("CONFIG_ADIRS_IR_ALIGN_TIME", 1.);
    for n in 1..=3 {
        test_bed.write_by_name(&format!("OVHD_ADIRS_IR_{n}_MODE_SELECTOR_KNOB"), 1.);
    }
    run(&mut test_bed, 200);
    check(&mut test_bed, "ground power");

    for n in 1..=4 {
        test_bed.write_by_name(&format!("ENGINE_STATE:{n}"), 1.);
        test_bed.write_by_name(&format!("TURB ENG CORRECTED N1:{n}"), 20.);
        test_bed.write_by_name(&format!("TURB ENG CORRECTED N2:{n}"), 65.);
        test_bed.write_by_name(&format!("ENGINE_N2:{n}"), 65.);
        test_bed.write_by_name(&format!("ENGINE_N2_HEALTHY:{n}"), 65.);
        test_bed.write_by_name(&format!("ENGINE_N3:{n}"), 70.);
        test_bed.write_by_name(&format!("ENGINE_N3_HEALTHY:{n}"), 70.);
    }
    for id in 1..=4 {
        test_bed.write_by_name(&format!("OVHD_ELEC_EXT_PWR_{id}_PB_IS_ON"), false);
        test_bed.write_by_name(&format!("EXT_PWR_AVAIL:{id}"), false);
    }
    run(&mut test_bed, 200);
    check(&mut test_bed, "engine start / taxi");

    test_bed.set_on_ground(false);
    for n in 1..=4 {
        test_bed.write_by_name(&format!("TURB ENG CORRECTED N1:{n}"), 85.);
        test_bed.write_by_name(&format!("TURB ENG CORRECTED N2:{n}"), 95.);
        test_bed.write_by_name(&format!("ENGINE_N2:{n}"), 95.);
        test_bed.write_by_name(&format!("ENGINE_N2_HEALTHY:{n}"), 95.);
        test_bed.write_by_name(&format!("ENGINE_N3:{n}"), 97.);
        test_bed.write_by_name(&format!("ENGINE_N3_HEALTHY:{n}"), 97.);
    }
    for step in 1..=5 {
        let altitude = Length::new::<foot>(7000. * step as f64);
        test_bed.set_pressure_altitude(altitude);
        test_bed.set_ambient_pressure(InternationalStandardAtmosphere::pressure_at_altitude(altitude));
        test_bed.set_ambient_temperature(InternationalStandardAtmosphere::temperature_at_altitude(altitude));
        test_bed.set_true_airspeed(Velocity::new::<knot>(250. + 20. * step as f64));
        test_bed.set_indicated_airspeed(Velocity::new::<knot>(280.));
        run(&mut test_bed, 60);
    }
    check(&mut test_bed, "climb");

    let cruise = Length::new::<foot>(35000.);
    test_bed.set_pressure_altitude(cruise);
    test_bed.set_ambient_pressure(InternationalStandardAtmosphere::pressure_at_altitude(cruise));
    test_bed.set_ambient_temperature(InternationalStandardAtmosphere::temperature_at_altitude(cruise));
    test_bed.set_true_airspeed(Velocity::new::<knot>(490.));
    test_bed.set_indicated_airspeed(Velocity::new::<knot>(290.));
    run(&mut test_bed, 300);
    check(&mut test_bed, "cruise FL350");

    for step in (0..=5).rev() {
        let altitude = Length::new::<foot>(7000. * step as f64);
        test_bed.set_pressure_altitude(altitude);
        test_bed.set_ambient_pressure(InternationalStandardAtmosphere::pressure_at_altitude(altitude));
        test_bed.set_ambient_temperature(InternationalStandardAtmosphere::temperature_at_altitude(altitude));
        test_bed.set_true_airspeed(Velocity::new::<knot>(180. + 10. * step as f64));
        test_bed.set_indicated_airspeed(Velocity::new::<knot>(180.));
        run(&mut test_bed, 60);
    }

    test_bed.set_on_ground(true);
    test_bed.set_true_airspeed(Velocity::new::<knot>(0.));
    test_bed.set_indicated_airspeed(Velocity::new::<knot>(0.));
    for n in 1..=4 {
        test_bed.write_by_name(&format!("TURB ENG CORRECTED N1:{n}"), 20.);
        test_bed.write_by_name(&format!("TURB ENG CORRECTED N2:{n}"), 65.);
        test_bed.write_by_name(&format!("ENGINE_N2:{n}"), 65.);
        test_bed.write_by_name(&format!("ENGINE_N2_HEALTHY:{n}"), 65.);
        test_bed.write_by_name(&format!("ENGINE_N3:{n}"), 70.);
        test_bed.write_by_name(&format!("ENGINE_N3_HEALTHY:{n}"), 70.);
    }
    run(&mut test_bed, 200);
    check(&mut test_bed, "landing");
}
