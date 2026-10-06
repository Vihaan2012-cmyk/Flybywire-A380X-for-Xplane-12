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

fn spin_engines(test_bed: &mut SimulationTestBed<A380>, n1: f64, n2: f64, n3: f64) {
    for n in 1..=4 {
        test_bed.write_by_name(&format!("ENGINE_STATE:{n}"), 1.);
        test_bed.write_by_name(&format!("ENGINE_N1:{n}"), n1);
        test_bed.write_by_name(&format!("TURB ENG CORRECTED N1:{n}"), n1);
        test_bed.write_by_name(&format!("TURB ENG CORRECTED N2:{n}"), n2);
        test_bed.write_by_name(&format!("ENGINE_N2:{n}"), n2);
        test_bed.write_by_name(&format!("ENGINE_N2_HEALTHY:{n}"), n2);
        test_bed.write_by_name(&format!("ENGINE_N3:{n}"), n3);
        test_bed.write_by_name(&format!("ENGINE_N3_HEALTHY:{n}"), n3);
    }
}

#[test]
fn a_partial_vfg_winding_degradation_sags_potential_without_tripping() {
    let mut test_bed = aircraft();
    spin_engines(&mut test_bed, 85., 95., 97.);
    run(&mut test_bed, 100);

    let healthy: f64 = test_bed.read_by_name("ELEC_ENG_GEN_1_POTENTIAL");
    assert!((100. ..=120.).contains(&healthy), "healthy GEN 1 potential out of range: {healthy}");
    let baseline_degradation: f64 = test_bed.read_by_name("ELEC_ENG_GEN_1_IMPEDANCE_DEGRADATION");
    assert_eq!(baseline_degradation, 0.);

    let id = failure_id("24_elec.vfg-1", "winding degradation");
    assert_eq!(arm(&mut test_bed, id as f64, 0.3), 1.);
    run(&mut test_bed, 50);

    let echoed: f64 = test_bed.read_by_name("ELEC_ENG_GEN_1_IMPEDANCE_DEGRADATION");
    assert!((echoed - 0.3).abs() < 1e-6, "expected 0.3 echoed onto FlyByWire's own input, got {echoed}");

    let degraded: f64 = test_bed.read_by_name("ELEC_ENG_GEN_1_POTENTIAL");
    assert!(degraded < healthy, "a partial winding degradation should sag FlyByWire's own generator potential: {healthy} -> {degraded}");
    assert!(degraded > 10., "a partial (0.3) degradation must not cut output to near zero: {degraded}");

    assert!(
        !test_bed.query(|a| a.derived_failure_ids()).contains(&24_020),
        "0.3 must not trip the binary level-2 coupling"
    );
}

#[test]
fn a_full_vfg_winding_degradation_still_trips_as_before() {
    let mut test_bed = aircraft();
    spin_engines(&mut test_bed, 85., 95., 97.);
    run(&mut test_bed, 100);

    let id = failure_id("24_elec.vfg-1", "winding degradation");
    assert_eq!(arm(&mut test_bed, id as f64, 1.0), 1.);
    run(&mut test_bed, 50);

    assert!(
        test_bed.query(|a| a.derived_failure_ids()).contains(&24_020),
        "a full winding degradation must still trip the unchanged level-2 coupling"
    );

    run_deriving_failures(&mut test_bed, 20);

    let potential: f64 = test_bed.read_by_name("ELEC_ENG_GEN_1_POTENTIAL");
    assert_eq!(potential, 0., "a tripped generator provides no output at all");
}

#[test]
fn a_tr_degradation_shows_in_flybywires_own_resistance_degradation_input() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 20);
    let baseline: f64 = test_bed.read_by_name("ELEC_TR_1_RESISTANCE_DEGRADATION");
    assert_eq!(baseline, 0.);

    let id = failure_id("24_elec.tr-1", "winding degradation");
    assert_eq!(arm(&mut test_bed, id as f64, 0.4), 1.);
    run(&mut test_bed, 20);

    let echoed: f64 = test_bed.read_by_name("ELEC_TR_1_RESISTANCE_DEGRADATION");
    assert!((echoed - 0.4).abs() < 1e-6, "expected 0.4, got {echoed}");
    assert!(
        !test_bed.query(|a| a.derived_failure_ids()).contains(&24_000),
        "0.4 must not trip TR 1's own binary coupling (DEGRADED_BEYOND_HALF = 0.5)"
    );
}

#[test]
fn a_battery_degradation_shows_in_flybywires_own_values() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 20);
    let baseline_r: f64 = test_bed.read_by_name("ELEC_BAT_1_RESISTANCE_GROWTH");
    let baseline_c: f64 = test_bed.read_by_name("ELEC_BAT_1_CAPACITY_FADE");
    assert_eq!(baseline_r, 0.);
    assert_eq!(baseline_c, 0.);

    let id = failure_id("24_elec.bat-1", "resistance growth");
    assert_eq!(arm(&mut test_bed, id as f64, 0.5), 1.);
    run(&mut test_bed, 20);

    let echoed: f64 = test_bed.read_by_name("ELEC_BAT_1_RESISTANCE_GROWTH");
    assert!((echoed - 0.5).abs() < 1e-6, "expected 0.5, got {echoed}");
}

#[test]
fn a_ground_cart_degradation_reaches_external_power_source_continuously() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 20);
    let baseline: f64 = test_bed.read_by_name("ELEC_EXT_PWR_1_REGULATION_DEGRADATION");
    assert_eq!(baseline, 0.);

    let id = failure_id("24_elec.gpu", "weak cart");
    assert_eq!(arm(&mut test_bed, id as f64, 0.4), 1.);
    run(&mut test_bed, 20);

    for n in 1..=4 {
        let echoed: f64 = test_bed.read_by_name(&format!("ELEC_EXT_PWR_{n}_REGULATION_DEGRADATION"));
        assert!((echoed - 0.4).abs() < 1e-6, "receptacle {n}: expected 0.4, got {echoed}");
    }
}

#[test]
fn a_rat_jam_reaches_the_emergency_generator_continuously() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 20);
    let baseline: f64 = test_bed.read_by_name("ELEC_EMER_GEN_OUTPUT_CAPABILITY_LOSS");
    assert_eq!(baseline, 0.);

    let id = failure_id("24_elec.rat", "jammed");
    assert_eq!(arm(&mut test_bed, id as f64, 0.4), 1.);
    run(&mut test_bed, 20);

    let echoed: f64 = test_bed.read_by_name("ELEC_EMER_GEN_OUTPUT_CAPABILITY_LOSS");
    assert!((echoed - 0.4).abs() < 1e-6, "expected 0.4, got {echoed}");
}

#[test]
fn a_healthy_aircraft_is_unchanged_by_the_new_continuous_inputs() {
    let mut test_bed = aircraft();
    spin_engines(&mut test_bed, 85., 95., 97.);
    run(&mut test_bed, 50);

    for n in 1..=4 {
        let g: f64 = test_bed.read_by_name(&format!("ELEC_ENG_GEN_{n}_IMPEDANCE_DEGRADATION"));
        let d: f64 = test_bed.read_by_name(&format!("ELEC_ENG_GEN_{n}_REGULATOR_DRIFT"));
        let t: f64 = test_bed.read_by_name(&format!("ELEC_TR_{n}_RESISTANCE_DEGRADATION"));
        let e: f64 = test_bed.read_by_name(&format!("ELEC_EXT_PWR_{n}_REGULATION_DEGRADATION"));
        assert_eq!(g, 0., "GEN {n} impedance degradation");
        assert_eq!(d, 0., "GEN {n} regulator drift");
        assert_eq!(t, 0., "TR {n} resistance degradation");
        assert_eq!(e, 0., "EXT PWR {n} regulation degradation");
    }
    for n in 1..=2 {
        let r: f64 = test_bed.read_by_name(&format!("ELEC_BAT_{n}_RESISTANCE_GROWTH"));
        let c: f64 = test_bed.read_by_name(&format!("ELEC_BAT_{n}_CAPACITY_FADE"));
        assert_eq!(r, 0., "BAT {n} resistance growth");
        assert_eq!(c, 0., "BAT {n} capacity fade");
    }
    let si: f64 = test_bed.read_by_name("ELEC_STAT_INV_EFFICIENCY_DEGRADATION");
    let emer: f64 = test_bed.read_by_name("ELEC_EMER_GEN_OUTPUT_CAPABILITY_LOSS");
    assert_eq!(si, 0.);
    assert_eq!(emer, 0.);
    assert!(test_bed.query(|a| a.derived_failure_ids()).is_empty(), "a healthy aircraft must derive no FlyByWire failures");
}

fn run_gear_down(test_bed: &mut SimulationTestBed<A380>, frames: usize) {
    for _ in 0..frames {
        test_bed.write_by_name("GEAR_HANDLE_POSITION", 1.0);
        test_bed.run_with_delta(Duration::from_millis(100));
    }
}

#[test]
fn a_whole_flight_keeps_electrical_values_realistic_and_quiet() {
    let mut test_bed = SimulationTestBed::new(A380::new);
    computers_healthy(&mut test_bed);
    test_bed.set_on_ground(true);
    test_bed.write_by_name("GEAR_HANDLE_POSITION", 1.0);
    for id in 1..=4 {
        test_bed.write_by_name(&format!("OVHD_ELEC_BAT_{id}_PB_IS_AUTO"), true);
    }
    run_gear_down(&mut test_bed, 200);

    for i in 1..=4 {
        test_bed.write_by_name(&format!("EXT_PWR_AVAIL:{i}"), true);
        test_bed.write_by_name(&format!("OVHD_ELEC_EXT_PWR_{i}_PB_IS_ON"), true);
    }
    run_gear_down(&mut test_bed, 300);

    spin_engines(&mut test_bed, 20., 65., 70.);
    run_gear_down(&mut test_bed, 300);
    for i in 1..=4 {
        test_bed.write_by_name(&format!("OVHD_ELEC_EXT_PWR_{i}_PB_IS_ON"), false);
        test_bed.write_by_name(&format!("EXT_PWR_AVAIL:{i}"), false);
    }
    run_gear_down(&mut test_bed, 150);

    test_bed.set_on_ground(false);
    spin_engines(&mut test_bed, 85., 95., 97.);
    run_gear_down(&mut test_bed, 300);

    run_gear_down(&mut test_bed, 400);

    spin_engines(&mut test_bed, 40., 75., 80.);
    run_gear_down(&mut test_bed, 300);

    test_bed.set_on_ground(true);
    spin_engines(&mut test_bed, 20., 60., 65.);
    run_gear_down(&mut test_bed, 150);

    for name in [
        "ELEC_AC_1_BUS_POTENTIAL",
        "ELEC_AC_2_BUS_POTENTIAL",
        "ELEC_AC_3_BUS_POTENTIAL",
        "ELEC_AC_4_BUS_POTENTIAL",
        "ELEC_DC_1_BUS_POTENTIAL",
        "ELEC_DC_2_BUS_POTENTIAL",
        "ELEC_ENG_GEN_1_POTENTIAL",
        "ELEC_ENG_GEN_1_FREQUENCY",
        "ELEC_BAT_1_CHARGE_FRACTION",
        "ELEC_BAT_2_CHARGE_FRACTION",
    ] {
        let v: f64 = test_bed.read_by_name(name);
        assert!(v.is_finite(), "{name} is not a number: {v}");
    }
    let bat1: f64 = test_bed.read_by_name("ELEC_BAT_1_CHARGE_FRACTION");
    let bat2: f64 = test_bed.read_by_name("ELEC_BAT_2_CHARGE_FRACTION");
    assert!((0.0..=1.0).contains(&bat1), "battery 1 charge fraction out of range: {bat1}");
    assert!((0.0..=1.0).contains(&bat2), "battery 2 charge fraction out of range: {bat2}");
    let gen1_v: f64 = test_bed.read_by_name("ELEC_ENG_GEN_1_POTENTIAL");
    assert!(gen1_v > 100., "GEN 1 should be online and healthy on final approach: {gen1_v} V");

    for n in 1..=4 {
        let d: f64 = test_bed.read_by_name(&format!("ELEC_ENG_GEN_{n}_IMPEDANCE_DEGRADATION"));
        assert_eq!(d, 0., "GEN {n} impedance degradation should be 0 on a healthy flight");
    }
    let derived = test_bed.query(|a| a.derived_failure_ids());
    assert!(derived.is_empty(), "a healthy whole flight must derive no FlyByWire failures: {derived:?}");
}

fn contactor_closed(test_bed: &mut SimulationTestBed<A380>, id: &str) -> bool {
    test_bed.read_by_name(&format!("ELEC_CONTACTOR_{id}_IS_CLOSED"))
}

#[test]
fn a_gpu_line_contactor_that_fails_to_close_keeps_external_power_off_the_aircraft() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 50);
    assert!(contactor_closed(&mut test_bed, "990XG1"), "healthy: external power 1 is connected");
    let ac1_before: bool = test_bed.read_by_name("ELEC_AC_1_BUS_IS_POWERED");
    assert!(ac1_before, "healthy: AC 1 is powered from external power 1");

    assert_eq!(arm(&mut test_bed, failure_id("24_elec.contactor.gpu-1-line", "fails to close") as f64, 1.), 1.);
    run(&mut test_bed, 50);
    assert!(!contactor_closed(&mut test_bed, "990XG1"), "the failed line contactor must hold FlyByWire's 990XG1 open");
    let ac1: bool = test_bed.read_by_name("ELEC_AC_1_BUS_IS_POWERED");
    assert!(!ac1, "with external power 1 the only source, AC 1 loses power");
}

#[test]
fn a_tr_1_breaker_trip_opens_flybywires_tr_1_line() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 50);
    assert!(contactor_closed(&mut test_bed, "990PU1"), "healthy: TR 1 is connected");

    assert_eq!(arm(&mut test_bed, failure_id("24_elec.bkr.tr-1-bkr", "nuisance trip") as f64, 1.), 1.);
    run(&mut test_bed, 50);
    let fails: f64 = test_bed.read_by_name("ELEC_CONTACTOR_990PU1_FAILS_TO_CLOSE");
    assert_eq!(fails, 1., "the tripped TR 1 breaker must hold FlyByWire's TR 1 line open");
    assert!(!contactor_closed(&mut test_bed, "990PU1"));
}

#[test]
fn a_generator_breaker_trip_takes_that_generator_off_line_and_the_bus_stays_fed() {
    let mut test_bed = aircraft();
    computers_healthy(&mut test_bed);
    spin_engines(&mut test_bed, 20., 65., 70.);
    test_bed.write_by_name("OVHD_ELEC_EXT_PWR_1_PB_IS_ON", false);
    test_bed.write_by_name("EXT_PWR_AVAIL:1", false);
    for _ in 0..30 {
        spin_engines(&mut test_bed, 20., 65., 70.);
        run(&mut test_bed, 10);
    }
    assert!(contactor_closed(&mut test_bed, "990XU1"), "healthy: generator 1 is on line");

    assert_eq!(arm(&mut test_bed, failure_id("24_elec.bkr.gen-1-bkr", "nuisance trip") as f64, 1.), 1.);
    for _ in 0..10 {
        spin_engines(&mut test_bed, 20., 65., 70.);
        run_deriving_failures(&mut test_bed, 10);
    }
    assert!(!contactor_closed(&mut test_bed, "990XU1"), "the tripped generator 1 breaker must take FlyByWire's generator 1 off line");
    for n in 2..=4 {
        assert!(contactor_closed(&mut test_bed, &format!("990XU{n}")), "generator {n} stays on line");
    }
}

#[test]
fn a_healthy_aircraft_never_forces_a_source_line() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 50);
    for id in ["990PU1", "990PU2", "6PE", "7PU", "990PB1", "990PB2", "990XU1", "990XU4", "990XS1", "990XS2", "990XG1", "990XG4", "5XE", "7XB"] {
        let fails: f64 = test_bed.read_by_name(&format!("ELEC_CONTACTOR_{id}_FAILS_TO_CLOSE"));
        let welded: f64 = test_bed.read_by_name(&format!("ELEC_CONTACTOR_{id}_WELDED_CLOSED"));
        assert_eq!((fails, welded), (0., 0.), "{id} forced on a healthy aircraft");
    }
}

#[test]
fn an_ac_emer_bus_short_fails_flybywires_own_ac_emer_bus() {
    let mut test_bed = aircraft();
    run_deriving_failures(&mut test_bed, 50);
    let before: bool = test_bed.read_by_name("ELEC_AC_ESS_BUS_IS_POWERED");
    assert!(before, "healthy: AC EMER is powered on ground power");

    assert_eq!(arm(&mut test_bed, failure_id("24_elec.bus.AC_EMER", "short to ground") as f64, 1.), 1.);
    run_deriving_failures(&mut test_bed, 50);
    let ids = test_bed.query(|a| a.derived_failure_ids());
    let deep_v: f64 = test_bed.read_by_name("ELEC_AC_EMER_BUS_POTENTIAL");
    let fbw_v: f64 = test_bed.read_by_name("ELEC_AC_ESS_BUS_POTENTIAL");
    let after: bool = test_bed.read_by_name("ELEC_AC_ESS_BUS_IS_POWERED");
    assert!(!after, "the shorted AC EMER bus must be dead in FlyByWire's network too: derived {ids:?}, deep AC EMER {deep_v} V, FBW {fbw_v} V");
}

#[test]
fn a_generator_breaker_that_fails_to_trip_costs_its_bus_when_the_generator_faults() {
    for jammed in [false, true] {
        let mut test_bed = aircraft();
        computers_healthy(&mut test_bed);
        spin_engines(&mut test_bed, 20., 65., 70.);
        test_bed.write_by_name("OVHD_ELEC_EXT_PWR_1_PB_IS_ON", false);
        test_bed.write_by_name("EXT_PWR_AVAIL:1", false);
        for _ in 0..30 {
            spin_engines(&mut test_bed, 20., 65., 70.);
            run(&mut test_bed, 10);
        }
        if jammed {
            assert_eq!(arm(&mut test_bed, failure_id("24_elec.bkr.gen-1-bkr", "fails to trip") as f64, 1.), 1.);
        }
        assert_eq!(arm(&mut test_bed, failure_id("24_elec.vfg-1", "winding degradation") as f64, 1.), 1.);
        for _ in 0..10 {
            spin_engines(&mut test_bed, 20., 65., 70.);
            run_deriving_failures(&mut test_bed, 10);
        }
        let line_closed: f64 = test_bed.read_by_name("ELEC_BKR_GEN_1_BKR_CLOSED");
        let ac1: bool = test_bed.read_by_name("ELEC_AC_1_BUS_IS_POWERED");
        let ids = test_bed.query(|a| a.derived_failure_ids());
        if jammed {
            assert_eq!(line_closed, 1., "a jammed breaker stays closed under its generator's fault");
            assert!(!ac1, "the faulted generator still on AC 1: AC 1 is isolated: derived {ids:?}");
        } else {
            assert_eq!(line_closed, 0., "the generator's protection opens its line breaker");
            assert!(ac1, "AC 1 is fed through the bus ties once generator 1 is isolated: derived {ids:?}");
        }
    }
}

#[test]
fn a_failed_battery_is_disconnected_unless_its_breaker_fails_to_trip() {
    for jammed in [false, true] {
        let mut test_bed = aircraft();
        run_deriving_failures(&mut test_bed, 20);
        if jammed {
            assert_eq!(arm(&mut test_bed, failure_id("24_elec.bkr.bat-1-bkr", "fails to trip") as f64, 1.), 1.);
        }
        assert_eq!(arm(&mut test_bed, failure_id("24_elec.bat-1", "resistance growth") as f64, 1.), 1.);
        run_deriving_failures(&mut test_bed, 50);
        let closed: f64 = test_bed.read_by_name("ELEC_BKR_BAT_1_BKR_CLOSED");
        let ids = test_bed.query(|a| a.derived_failure_ids());
        let hot_buses: Vec<u64> = ids.iter().copied().filter(|&id| (24_113..=24_116).contains(&id) || id == 24_111 || id == 24_112).collect();
        if jammed {
            assert_eq!(closed, 1., "a jammed battery breaker stays closed");
            assert!(!hot_buses.is_empty(), "the failed battery left on its bus costs that bus: derived {ids:?}");
        } else {
            assert_eq!(closed, 0., "the failed battery is disconnected");
            let fails: f64 = test_bed.read_by_name("ELEC_CONTACTOR_990PB1_FAILS_TO_CLOSE");
            assert_eq!(fails, 1., "and FlyByWire's battery 1 contactor with it");
            assert!(hot_buses.is_empty(), "no bus is lost when the battery is isolated: derived {ids:?}");
        }
    }
}

#[test]
fn an_open_battery_cross_feed_diode_loses_dc_hot_2_in_flybywire_too() {
    let mut test_bed = aircraft();
    run_deriving_failures(&mut test_bed, 20);
    assert!(!test_bed.query(|a| a.derived_failure_ids()).contains(&24_114), "healthy: DC HOT 2 is not failed");
    assert_eq!(arm(&mut test_bed, failure_id("24_elec.diode.bat-cross-feed-diode", "open") as f64, 1.), 1.);
    run_deriving_failures(&mut test_bed, 50);
    let ids = test_bed.query(|a| a.derived_failure_ids());
    assert!(ids.contains(&24_114), "the open diode costs DC HOT 2: derived {ids:?}");
}

#[test]
fn a_high_resistance_load_fault_trips_only_the_breakers_it_really_overloads() {
    let run_case = |component: &str, unit: &str, magnitude: f64| -> (f64, f64, f64) {
        let mut test_bed = aircraft();
        run(&mut test_bed, 50);
        if magnitude > 0.0 {
            assert_eq!(arm(&mut test_bed, failure_id(component, "high resistance") as f64, magnitude), 1.);
        }
        run(&mut test_bed, 3000);
        let key = unit.to_uppercase().replace('-', "_");
        let status: f64 = test_bed.read_by_name(&format!("BKR_{key}_STATUS"));
        let cut: f64 = test_bed.read_by_name(&format!("ELEC_LOAD_{key}_CUT"));
        let network_closed: f64 = test_bed.read_by_name(&format!("ELEC_BKR_{key}_CLOSED"));
        (status, cut, network_closed)
    };
    for (component, unit, trips) in [
        ("31_elec.dfdr", "dfdr", false),
        ("31_elec.cvr", "cvr", false),
        ("31_elec.dfdau", "dfdau", false),
        ("31_elec.fo-efis-bkup-ctl", "fo-efis-bkup-ctl", false),
        ("31_elec.fo-efis-ctl-panel", "fo-efis-ctl-panel", false),
        ("31_elec.cds-mailbox-capt", "cds-mailbox-capt", true),
        ("31_elec.recorder-accelerometer", "recorder-accelerometer", true),
    ] {
        let (status, cut, network_closed) = run_case(component, unit, 1.0);
        if trips {
            assert!(status != 0. && cut == 1., "{unit}: full severity overloads its 1 A breaker, which trips and cuts the load (status {status}, cut {cut}, network closed {network_closed})");
        } else {
            assert!(status == 0. && cut == 0. && network_closed == 1., "{unit}: full severity stays under its breaker, nothing trips (status {status}, cut {cut}, network closed {network_closed})");
        }
        let (status, cut, _) = run_case(component, unit, 0.3);
        assert!(status == 0. && cut == 0., "{unit}: partial severity trips nothing (status {status}, cut {cut})");
    }
}

#[test]
fn a_dead_short_on_dc_ess_trips_its_feeder_and_takes_flybywires_dc_ess_bus_down() {
    use super::tests::{aircraft, arm, run, run_deriving_failures};
    let mut test_bed = aircraft();
    for id in 1..=4 {
        test_bed.write_by_name(&format!("OVHD_ELEC_BAT_{id}_PB_IS_AUTO"), true);
        test_bed.write_by_name(&format!("EXT_PWR_AVAIL:{id}"), true);
        test_bed.write_by_name(&format!("OVHD_ELEC_EXT_PWR_{id}_PB_IS_ON"), true);
    }
    run(&mut test_bed, 50);
    let powered = |t: &mut SimulationTestBed<A380>, n: &str| -> f64 { t.read_by_name(n) };
    assert_eq!(powered(&mut test_bed, "ELEC_DC_ESS_BUS_IS_POWERED"), 1.);
    assert_eq!(arm(&mut test_bed, 1024914.0, 1.0), 1.);
    run_deriving_failures(&mut test_bed, 20);
    assert_eq!(powered(&mut test_bed, "ELEC_DC_ESS_BUS_IS_POWERED"), 0., "a dead short must take FlyByWire's DC ESS bus down");
    assert_eq!(powered(&mut test_bed, "ELEC_DC_1_BUS_IS_POWERED"), 1., "the feeder trip isolates the short, DC 1 stays up");
}
