#[allow(unused_imports)]
use systems::simulation::test::{ReadByName, SimulationTestBed, TestBed, WriteByName};

#[allow(unused_imports)]
use super::tests::{aircraft, arm, failure_id, run};
#[allow(unused_imports)]
use crate::A380;
#[allow(unused_imports)]
use systems::shared::arinc429::Arinc429Word;
#[allow(unused_imports)]
use uom::si::{f64::ThermodynamicTemperature, thermodynamic_temperature::degree_celsius};

fn start_apu_for_real(test_bed: &mut SimulationTestBed<A380>) {
    let feed_four_gal = 5000. / systems::fuel::FUEL_GALLONS_TO_KG;
    test_bed.write_by_name("FUEL_TANK_QUANTITY_9", feed_four_gal);
    test_bed.write_by_name("FUELSYSTEM TANK QUANTITY:9", feed_four_gal);
    test_bed.write_by_name("OVHD_APU_MASTER_SW_PB_IS_ON", true);
    test_bed.write_by_name("OVHD_APU_START_PB_IS_ON", true);
    for _ in 0..60 {
        run(test_bed, 50);
        let n: f64 = test_bed.read_by_name("APU_N_RAW");
        if n > 99.0 {
            return;
        }
    }
    let n: f64 = test_bed.read_by_name("APU_N_RAW");
    panic!("setup: APU_N did not reach a governed running state within the test's frame budget: {n}");
}

#[test]
fn a_healthy_aircraft_is_unchanged_by_the_apu_authority() {
    let mut test_bed = aircraft();
    start_apu_for_real(&mut test_bed);
    run(&mut test_bed, 50);

    let n: f64 = test_bed.read_by_name("APU_N_RAW");
    let egt = { let w: Arinc429Word<ThermodynamicTemperature> = test_bed.read_arinc429_by_name("APU_EGT"); w.value().get::<degree_celsius>() };
    let egt_warning = { let w: Arinc429Word<ThermodynamicTemperature> = test_bed.read_arinc429_by_name("APU_EGT_WARNING"); w.value().get::<degree_celsius>() };
    let bleed: f64 = test_bed.read_by_name("APU_BLEED_AIR_PRESSURE");

    assert!(
        (n - 100.0).abs() < 1.0,
        "a healthy APU must still hold its normal governed N with the authority bridge wired in: {n}"
    );
    assert!(
        egt < egt_warning,
        "a healthy running APU's EGT must stay well under its own warning band (no spurious bias injected): egt {egt}, warning {egt_warning}"
    );
    assert!(
        bleed > 0.0,
        "a healthy running APU must still produce positive bleed air pressure: {bleed}"
    );
}

#[test]
fn an_armed_apu_oil_leak_eventually_pulls_down_flybywires_own_apu_n() {
    let id = failure_id("49_apu.oil_system", "APU oil leak");

    let mut healthy = aircraft();
    start_apu_for_real(&mut healthy);
    run(&mut healthy, 6_000);
    let healthy_n: f64 = healthy.read_by_name("APU_N_RAW");

    let mut leaking = aircraft();
    start_apu_for_real(&mut leaking);
    assert_eq!(arm(&mut leaking, id as f64, 1.0), 1.);
    run(&mut leaking, 6_000);
    let leaking_n: f64 = leaking.read_by_name("APU_N_RAW");

    assert!(
        healthy_n > 99.0,
        "setup: a healthy APU must hold its governed N over 600 s with nothing armed: {healthy_n}"
    );
    assert!(
        leaking_n < healthy_n - 1.0,
        "a full-magnitude APU oil leak must measurably pull FlyByWire's own APU_N down from its \
         healthy governed value within 600 s: healthy {healthy_n}, leaking {leaking_n}"
    );
}

#[test]
fn a_stuck_apu_feed_valve_starves_flybywires_apu() {
    let mut test_bed = aircraft();
    assert_eq!(arm(&mut test_bed, failure_id("28_fuel.valve.apu_feed", "sticks") as f64, 1.), 1.);
    let feed_four_gal = 5000. / systems::fuel::FUEL_GALLONS_TO_KG;
    test_bed.write_by_name("FUEL_TANK_QUANTITY_9", feed_four_gal);
    test_bed.write_by_name("FUELSYSTEM TANK QUANTITY:9", feed_four_gal);
    test_bed.write_by_name("OVHD_APU_MASTER_SW_PB_IS_ON", true);
    test_bed.write_by_name("OVHD_APU_START_PB_IS_ON", true);
    let mut max_n = 0.0f64;
    for _ in 0..60 {
        run(&mut test_bed, 50);
        let n: f64 = test_bed.read_by_name("APU_N_RAW");
        max_n = max_n.max(n);
    }
    let fault: f64 = test_bed.read_by_name("FUEL_APU_FEED_VALVE_FAULT");
    assert_eq!(fault, 1., "the feed valve stays shut against its command");
    assert!(max_n < 50., "and the APU never gets going: N peaked at {max_n}");

    let mut healthy = aircraft();
    start_apu_for_real(&mut healthy);
    let low_pressure: bool = healthy.read_by_name("APU_LOW_FUEL_PRESSURE_FAULT");
    assert!(!low_pressure);
}

#[test]
fn a_failed_apu_generator_overload_protection_lets_the_winding_burn_out() {
    use deep_systems::deep::electrical::sources::{ApuGenerator, ApuGeneratorFaults, ApuGeneratorInputs};
    assert_eq!(failure_id("49_apu.generator_1", "overload protection"), deep_systems::deep::electrical::live::APU_GEN_OVERLOAD_PROTECTION_FAILED[0]);
    assert_eq!(failure_id("49_apu.generator_2", "overload protection"), deep_systems::deep::electrical::live::APU_GEN_OVERLOAD_PROTECTION_FAILED[1]);

    for failed in [false, true] {
        let mut generator = ApuGenerator::new();
        let faults = || ApuGeneratorFaults { winding_degradation: 0., regulator_drift: 0., overload_protection_failed: if failed { 1. } else { 0. } };
        let mut tripped_at = None;
        let mut volts = 0.;
        for tick in 0..2000 {
            let out = generator.step(ApuGeneratorInputs { apu_speed_fraction: 1., measured_load_w: 180_000. }, faults(), 0.1);
            if out.overload_tripped && tripped_at.is_none() {
                tripped_at = Some(tick);
            }
            volts = out.open_circuit_v;
        }
        if failed {
            assert_eq!(tripped_at, None, "nothing trips an unprotected machine");
            assert!(generator.burnt_out() && volts == 0., "its winding burns out: {volts} V");
        } else {
            assert!(tripped_at.is_some(), "a protected machine trips on the overload");
            assert!(!generator.burnt_out() && volts > 100., "and keeps its winding: {volts} V");
        }
    }
}
