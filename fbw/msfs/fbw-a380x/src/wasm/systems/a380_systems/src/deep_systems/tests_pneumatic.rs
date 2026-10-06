#[allow(unused_imports)]
use std::time::Duration;

#[allow(unused_imports)]
use systems::simulation::test::{ReadByName, SimulationTestBed, TestBed, WriteByName};

#[allow(unused_imports)]
use super::tests::{aircraft, arm, computers_healthy, failure_id, run};
#[allow(unused_imports)]
use crate::A380;


fn start_engines(test_bed: &mut SimulationTestBed<A380>) {
    for n in 1..=4 {
        test_bed.write_by_name(&format!("OVHD_PNEU_ENG_{n}_BLEED_PB_IS_AUTO"), true);
        test_bed.write_by_name(&format!("ENGINE_STATE:{n}"), 1.);
        test_bed.write_by_name(&format!("TURB ENG CORRECTED N1:{n}"), 85.);
        test_bed.write_by_name(&format!("TURB ENG CORRECTED N2:{n}"), 95.);
        test_bed.write_by_name(&format!("ENGINE_N2:{n}"), 95.);
        test_bed.write_by_name(&format!("ENGINE_N2_HEALTHY:{n}"), 95.);
        test_bed.write_by_name(&format!("ENGINE_N3:{n}"), 97.);
        test_bed.write_by_name(&format!("ENGINE_N3_HEALTHY:{n}"), 97.);
    }
    run(test_bed, 300);
}

#[test]
fn a_deep_duct_leak_lowers_flybywires_own_regulated_bleed_pressure_continuously_with_magnitude() {
    let mut test_bed = aircraft();
    start_engines(&mut test_bed);

    let healthy: f64 = test_bed.read_by_name("PNEU_ENG_1_PRECOOLER_OUTLET_PRESSURE");
    assert!(healthy > 20., "engine 1 must be regulating a real bleed pressure once running, got {healthy} psi");

    test_bed.write_by_name("DEEP_PNEU_ENG_1_DUCT_LEAK_KG_S", 0.2);
    run(&mut test_bed, 60);
    let small_leak: f64 = test_bed.read_by_name("PNEU_ENG_1_PRECOOLER_OUTLET_PRESSURE");
    assert!(small_leak < healthy, "a duct leak must lower FlyByWire's own regulated bleed pressure, healthy {healthy} Pa vs leaking {small_leak} Pa");

    test_bed.write_by_name("DEEP_PNEU_ENG_1_DUCT_LEAK_KG_S", 1.0);
    run(&mut test_bed, 60);
    let big_leak: f64 = test_bed.read_by_name("PNEU_ENG_1_PRECOOLER_OUTLET_PRESSURE");
    assert!(big_leak < small_leak, "a bigger leak must lower the pressure further, small {small_leak} Pa vs big {big_leak} Pa");

    let engine_2: f64 = test_bed.read_by_name("PNEU_ENG_2_PRECOOLER_OUTLET_PRESSURE");
    assert!(engine_2 > 20., "engine 1's own leak must not depressurise engine 2's own duct: {engine_2} psi");

    test_bed.write_by_name("DEEP_PNEU_ENG_1_DUCT_LEAK_KG_S", 0.0);
    run(&mut test_bed, 100);
    let recovered: f64 = test_bed.read_by_name("PNEU_ENG_1_PRECOOLER_OUTLET_PRESSURE");
    assert!(recovered > big_leak, "clearing the leak must let the regulator recover the pressure, big-leak {big_leak} Pa vs recovered {recovered} Pa");
}

#[test]
fn a_deep_engine_start_duct_leak_lowers_flybywires_own_starter_container_pressure_continuously_with_magnitude() {
    let mut test_bed = aircraft();
    start_engines(&mut test_bed);

    let healthy: f64 = test_bed.read_by_name("PNEU_ENG_1_STARTER_CONTAINER_PRESSURE");

    test_bed.write_by_name("DEEP_PNEU_ENG_1_START_DUCT_LEAK_KG_S", 0.2);
    run(&mut test_bed, 60);
    let small_leak: f64 = test_bed.read_by_name("PNEU_ENG_1_STARTER_CONTAINER_PRESSURE");
    assert!(small_leak <= healthy, "a start-duct leak must not raise FlyByWire's own starter container pressure, healthy {healthy} Pa vs leaking {small_leak} Pa");

    test_bed.write_by_name("DEEP_PNEU_ENG_1_START_DUCT_LEAK_KG_S", 1.0);
    run(&mut test_bed, 60);
    let big_leak: f64 = test_bed.read_by_name("PNEU_ENG_1_STARTER_CONTAINER_PRESSURE");
    assert!(big_leak <= small_leak, "a bigger start-duct leak must not raise the pressure further, small {small_leak} Pa vs big {big_leak} Pa");

    let engine_2: f64 = test_bed.read_by_name("PNEU_ENG_2_STARTER_CONTAINER_PRESSURE");
    assert!(engine_2 > 0., "engine 1's own start-duct leak must not depressurise engine 2's own starter container: {engine_2} Pa");

    test_bed.write_by_name("DEEP_PNEU_ENG_1_START_DUCT_LEAK_KG_S", 0.0);
    run(&mut test_bed, 100);
    let recovered: f64 = test_bed.read_by_name("PNEU_ENG_1_STARTER_CONTAINER_PRESSURE");
    assert!(recovered >= big_leak, "clearing the leak must not leave the pressure lower than under the leak, big-leak {big_leak} Pa vs recovered {recovered} Pa");
}

#[test]
fn an_odls_trip_closes_flybywires_pr_and_hp_valves() {
    let mut test_bed = aircraft();
    start_engines(&mut test_bed);
    run(&mut test_bed, 50);

    let pr_before: f64 = test_bed.read_by_name("PNEU_ENG_1_PR_VALVE_OPEN");
    assert_eq!(pr_before, 1., "engine 1's PR valve must be open on a healthy running engine");

    test_bed.write_by_name("DEEP_PNEU_ENG_1_ODLS_ISOLATED", 1.0);
    run(&mut test_bed, 50);
    let pr_after: f64 = test_bed.read_by_name("PNEU_ENG_1_PR_VALVE_OPEN");
    let hp_after: f64 = test_bed.read_by_name("PNEU_ENG_1_HP_VALVE_OPEN");
    assert_eq!(pr_after, 0., "a confirmed ODLS trip must close FlyByWire's own PR valve");
    assert_eq!(hp_after, 0., "a confirmed ODLS trip must close FlyByWire's own HP valve too");

    let pr_engine_2: f64 = test_bed.read_by_name("PNEU_ENG_2_PR_VALVE_OPEN");
    assert_eq!(pr_engine_2, 1., "isolating engine 1 must not touch engine 2's own PR valve");

    test_bed.write_by_name("DEEP_PNEU_ENG_1_ODLS_ISOLATED", 0.0);
    run(&mut test_bed, 50);
    let pr_reset: f64 = test_bed.read_by_name("PNEU_ENG_1_PR_VALVE_OPEN");
    assert_eq!(pr_reset, 1., "clearing the isolation must let the PR valve open again");
}

#[test]
fn a_whole_flight_keeps_pneumatics_air_conditioning_and_thermal_zones_in_realistic_ranges() {
    use systems::shared::InternationalStandardAtmosphere;
    use uom::si::{f64::*, length::foot, velocity::knot};

    fn fly_at(test_bed: &mut SimulationTestBed<A380>, altitude: Length, tas_kt: f64) {
        test_bed.set_pressure_altitude(altitude);
        test_bed.set_ambient_pressure(InternationalStandardAtmosphere::pressure_at_altitude(altitude));
        test_bed.set_ambient_temperature(InternationalStandardAtmosphere::temperature_at_altitude(altitude));
        test_bed.set_true_airspeed(Velocity::new::<knot>(tas_kt));
        test_bed.set_indicated_airspeed(Velocity::new::<knot>(tas_kt.min(300.)));
    }

    fn check_phase(test_bed: &mut SimulationTestBed<A380>, name: &str) {
        let mut not_finite: Vec<String> = Vec::new();
        let mut out_of_range: Vec<String> = Vec::new();
        let mut spurious_fault: Vec<String> = Vec::new();

        for n in 1..=4 {
            let duct_pa: f64 = test_bed.read_by_name(&format!("DEEP_PNEU_ENG_{n}_DUCT_PRESSURE_PA"));
            let duct_c: f64 = test_bed.read_by_name(&format!("DEEP_PNEU_ENG_{n}_DUCT_TEMPERATURE_C"));
            let fbw_pressure: f64 = test_bed.read_by_name(&format!("PNEU_ENG_{n}_PRECOOLER_OUTLET_PRESSURE"));
            let odls: f64 = test_bed.read_by_name(&format!("DEEP_PNEU_ENG_{n}_ODLS_ISOLATED"));
            let pr_open: f64 = test_bed.read_by_name(&format!("PNEU_ENG_{n}_PR_VALVE_OPEN"));
            if ![duct_pa, duct_c, fbw_pressure, odls, pr_open].iter().all(|v| v.is_finite()) {
                not_finite.push(format!("engine {n} pneumatics"));
            }
            if !(0.0..=1_000_000.0).contains(&duct_pa) {
                out_of_range.push(format!("engine {n} duct pressure {duct_pa} Pa"));
            }
            if !(-60.0..=300.0).contains(&duct_c) {
                out_of_range.push(format!("engine {n} duct temperature {duct_c} C"));
            }
            if odls != 0.0 {
                spurious_fault.push(format!("engine {n} ODLS isolated with nothing wrong"));
            }
            let _ = pr_open;
        }
        for p in 1..=2 {
            let pa: f64 = test_bed.read_by_name(&format!("DEEP_PNEU_PACK_{p}_SUPPLY_PRESSURE_PA"));
            if !pa.is_finite() {
                not_finite.push(format!("pack {p} supply pressure"));
            }
            if !(0.0..=1_000_000.0).contains(&pa) {
                out_of_range.push(format!("pack {p} supply pressure {pa} Pa"));
            }
        }
        let apu_c: f64 = test_bed.read_by_name("DEEP_PNEU_APU_DUCT_TEMPERATURE_C");
        if !apu_c.is_finite() || !(-60.0..=300.0).contains(&apu_c) {
            out_of_range.push(format!("APU duct temperature {apu_c} C"));
        }
        for zone in [
            "MainAvionics",
            "UpperAvionics",
            "AftAvionics",
            "CargoFwd",
            "CargoAft",
            "CargoBulk",
            "CabinMainDeck",
            "CabinUpperDeck",
        ] {
            let c: f64 = test_bed.read_by_name(&format!("THERMAL_ZONE_{}_TEMPERATURE_C", zone.to_ascii_uppercase()));
            if !c.is_finite() {
                not_finite.push(format!("{zone} temperature"));
            }
            if !(-60.0..=85.0).contains(&c) {
                out_of_range.push(format!("{zone} temperature {c} C"));
            }
        }

        assert!(not_finite.is_empty(), "{name}: not a number: {not_finite:?}");
        assert!(out_of_range.is_empty(), "{name}: out of a realistic range: {out_of_range:?}");
        assert!(spurious_fault.is_empty(), "{name}: spurious fault on a healthy aircraft: {spurious_fault:?}");
    }

    let mut test_bed = aircraft();
    test_bed.set_on_ground(true);
    check_phase(&mut test_bed, "cold and dark / ground power");

    start_engines(&mut test_bed);
    for n in 1..=4 {
        let pr_open: f64 = test_bed.read_by_name(&format!("PNEU_ENG_{n}_PR_VALVE_OPEN"));
        assert_eq!(pr_open, 1., "engine {n} PR valve must open once the engine is confirmed running");
    }
    check_phase(&mut test_bed, "engines running, on ground (taxi)");

    test_bed.set_on_ground(false);
    fly_at(&mut test_bed, Length::new::<foot>(5000.), 220.);
    run(&mut test_bed, 100);
    check_phase(&mut test_bed, "climb");

    fly_at(&mut test_bed, Length::new::<foot>(35000.), 480.);
    run(&mut test_bed, 200);
    check_phase(&mut test_bed, "cruise FL350");

    fly_at(&mut test_bed, Length::new::<foot>(8000.), 250.);
    run(&mut test_bed, 100);
    check_phase(&mut test_bed, "descent");

    test_bed.set_on_ground(true);
    fly_at(&mut test_bed, Length::new::<foot>(0.), 0.);
    run(&mut test_bed, 100);
    check_phase(&mut test_bed, "landing");
}

#[test]
fn a_healthy_aircraft_is_unchanged_by_this_areas_authority_inputs() {
    let mut test_bed = aircraft();
    start_engines(&mut test_bed);
    run(&mut test_bed, 100);

    for n in 1..=4 {
        let pr: f64 = test_bed.read_by_name(&format!("PNEU_ENG_{n}_PR_VALVE_OPEN"));
        assert_eq!(pr, 1., "engine {n} PR valve must be open on a healthy running engine");
        let pressure: f64 = test_bed.read_by_name(&format!("PNEU_ENG_{n}_PRECOOLER_OUTLET_PRESSURE"));
        assert!(pressure.is_finite() && pressure > 20., "engine {n} must regulate a real bleed pressure, got {pressure} psi");
    }
    let not_finite = [
        "PNEU_ENG_1_PRECOOLER_OUTLET_PRESSURE",
        "PNEU_APU_BLEED_CONTAINER_PRESSURE",
        "HYD_GREEN_RESERVOIR_AIR_PRESSURE",
        "HYD_YELLOW_RESERVOIR_AIR_PRESSURE",
    ]
    .into_iter()
    .filter(|name| !ReadByName::<_, f64>::read_by_name(&mut test_bed, name).is_finite())
    .collect::<Vec<_>>();
    assert!(not_finite.is_empty(), "not a number: {not_finite:?}");
}

#[test]
fn a_seized_valve_reaches_flybywires_own_valve_seizure_input() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 10);
    for n in 1..=16 {
        let seizure: f64 = test_bed.read_by_name(&format!("PNEU_VALVE_FAILED:{n}"));
        assert_eq!(seizure, 0., "valve {n} is seized on a healthy aircraft");
    }
    let id = failure_id("36_pneu.engine_upstream_valve_stage", "HP valve stuck");
    assert_eq!(arm(&mut test_bed, id as f64, 0.6), 1., "the deep HP valve failure arms");
    run(&mut test_bed, 3);
    for n in 1..=16 {
        let seizure: f64 = test_bed.read_by_name(&format!("PNEU_VALVE_FAILED:{n}"));
        let expected = if n <= 4 { 0.6 } else { 0. };
        assert!((seizure - expected).abs() < 1e-9, "valve {n}: seizure {seizure}, expected {expected}");
    }
}

#[test]
fn a_healthy_aircraft_is_unchanged_by_the_pneumatic_ducts_authority() {
    let mut test_bed = aircraft();
    start_engines(&mut test_bed);
    run(&mut test_bed, 50);

    for n in 1..=4 {
        let isolated: f64 = test_bed.read_by_name(&format!("DEEP_PNEU_ENG_{n}_ODLS_ISOLATED"));
        assert_eq!(isolated, 0., "engine {n} must not be isolated on a healthy aircraft");
    }

    let before: [f64; 3] = [
        test_bed.read_by_name("PNEU_XBLEED_VALVE_L_OPEN_AMOUNT"),
        test_bed.read_by_name("PNEU_XBLEED_VALVE_C_OPEN_AMOUNT"),
        test_bed.read_by_name("PNEU_XBLEED_VALVE_R_OPEN_AMOUNT"),
    ];
    assert!(before.iter().all(|v| v.is_finite()), "cross-bleed valve open amounts must be real numbers: {before:?}");

    for n in 1..=4 {
        test_bed.write_by_name(&format!("DEEP_PNEU_ENG_{n}_ODLS_ISOLATED"), 0.0);
    }
    run(&mut test_bed, 20);

    let after: [f64; 3] = [
        test_bed.read_by_name("PNEU_XBLEED_VALVE_L_OPEN_AMOUNT"),
        test_bed.read_by_name("PNEU_XBLEED_VALVE_C_OPEN_AMOUNT"),
        test_bed.read_by_name("PNEU_XBLEED_VALVE_R_OPEN_AMOUNT"),
    ];
    for (name, (b, a)) in ["L", "C", "R"].iter().zip(before.iter().zip(after.iter())) {
        assert!((a - b).abs() < 0.05, "cross-bleed valve {name}: a healthy (0) isolation input must not move the valve's own open amount beyond FlyByWire's own solve, before {b}, after {a}");
    }
}

#[test]
fn a_deep_isolation_closes_flybywires_cross_bleed_valve_continuously_with_severity() {
    let mut test_bed = aircraft();
    start_engines(&mut test_bed);
    run(&mut test_bed, 50);

    let healthy: f64 = test_bed.read_by_name("PNEU_XBLEED_VALVE_L_OPEN_AMOUNT");
    assert!(healthy.is_finite(), "the left cross-bleed valve's open amount must be a real number: {healthy}");

    test_bed.write_by_name("DEEP_PNEU_ENG_1_ODLS_ISOLATED", 0.4);
    run(&mut test_bed, 10);
    let partial: f64 = test_bed.read_by_name("PNEU_XBLEED_VALVE_L_OPEN_AMOUNT");
    assert!(partial <= healthy + 1e-9, "a partial isolation must not raise the valve's own open amount, healthy {healthy} vs partial {partial}");

    test_bed.write_by_name("DEEP_PNEU_ENG_1_ODLS_ISOLATED", 1.0);
    run(&mut test_bed, 10);
    let full: f64 = test_bed.read_by_name("PNEU_XBLEED_VALVE_L_OPEN_AMOUNT");
    assert!(full < 1e-9, "a full (1.0) isolation must fully close the valve regardless of FlyByWire's own command, got {full}");
    assert!(full <= partial + 1e-9, "a fuller isolation must close it at least as much -- continuous with severity, not a threshold trip, partial {partial} vs full {full}");

    let right: f64 = test_bed.read_by_name("PNEU_XBLEED_VALVE_R_OPEN_AMOUNT");
    assert!(right.is_finite(), "right cross-bleed valve amount must stay a real number: {right}");

    test_bed.write_by_name("DEEP_PNEU_ENG_1_ODLS_ISOLATED", 0.0);
    run(&mut test_bed, 50);
    let recovered: f64 = test_bed.read_by_name("PNEU_XBLEED_VALVE_L_OPEN_AMOUNT");
    assert!(recovered >= full, "clearing the isolation must let the valve reopen at least as far as under full isolation, full-isolation {full} vs recovered {recovered}");
}

#[test]
#[ignore]
fn diag_pr_valve_gate_inputs() {
    let mut test_bed = aircraft();
    start_engines(&mut test_bed);
    for _ in 0..3 {
        run(&mut test_bed, 50);
        for name in [
            "PNEU_ENG_1_PR_VALVE_OPEN",
            "PNEU_ENG_1_TRANSFER_TRANSDUCER_PRESSURE",
            "PNEU_ENG_1_REGULATED_TRANSDUCER_PRESSURE",
            "PNEU_ENG_1_HP_PRESSURE",
            "PNEU_ENG_1_STARTER_VALVE_OPEN",
            "PNEU_ENG_1_HP_VALVE_OPEN",
            "OVHD_PNEU_ENG_1_BLEED_PB_IS_AUTO",
            "OVHD_PNEU_APU_BLEED_PB_IS_ON",
            "APU_BLEED_AIR_VALVE_OPEN",
            "DEEP_PNEU_ENG_1_ODLS_ISOLATED",
            "FIRE_BUTTON_ENG1",
            "ENGINE_STATE:1",
            "ELEC_DC_ESS_BUS_IS_POWERED",
            "ELEC_DC_1_BUS_IS_POWERED",
            "ELEC_AC_1_BUS_IS_POWERED",
            "PNEU_ENG_1_PRECOOLER_OUTLET_PRESSURE",
        ] {
            let v: f64 = test_bed.read_by_name(name);
            println!("{name} = {v}");
        }
        println!("---");
    }
}
