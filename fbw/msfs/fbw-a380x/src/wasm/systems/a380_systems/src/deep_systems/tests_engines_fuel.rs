#[allow(unused_imports)]
use std::time::Duration;

#[allow(unused_imports)]
use systems::simulation::test::{ReadByName, SimulationTestBed, TestBed, WriteByName};

#[allow(unused_imports)]
use super::tests::{aircraft, arm, computers_healthy, failure_id, run};
#[allow(unused_imports)]
use crate::A380;

#[test]
fn a_healthy_aircraft_is_unchanged_by_deep_fuel_authority() {
    let mut test_bed = aircraft();
    let gal = 5000. / systems::fuel::FUEL_GALLONS_TO_KG;
    test_bed.write_by_name("FUEL_TANK_QUANTITY_2", gal);
    test_bed.write_by_name("FUELSYSTEM PUMP ACTIVE:19", true);
    run(&mut test_bed, 50);
    let qty: f64 = test_bed.read_by_name("FUEL_TANK_QUANTITY_2");
    assert!((qty - gal).abs() < 1e-6, "no leak armed: tank quantity must not move");
    let pressure: f64 = test_bed.read_by_name("FUEL_PUMP_PRESSURE_NORM:19");
    assert!((pressure - 1.0).abs() < 1e-9, "no pump fault armed: pressure must stay nominal");
}

fn seeded_aircraft(start_gal: f64) -> SimulationTestBed<A380> {
    let mut test_bed = SimulationTestBed::new(A380::new);
    computers_healthy(&mut test_bed);
    test_bed.write_by_name("EXT_PWR_AVAIL:1", true);
    test_bed.write_by_name("OVHD_ELEC_EXT_PWR_1_PB_IS_ON", true);
    test_bed.write_by_name("FUEL_TANK_QUANTITY_2", start_gal);
    test_bed.write_by_name("FUELSYSTEM TANK QUANTITY:2", start_gal);
    run(&mut test_bed, 20);
    test_bed
}

#[test]
fn a_deep_fuel_leak_drains_flybywires_own_tank_scaling_with_magnitude() {
    let id = failure_id("28_fuel.tank_wall.feed_1", "structural fuel leak");
    let start_gal = 8000. / systems::fuel::FUEL_GALLONS_TO_KG;

    let mut small = seeded_aircraft(start_gal);
    assert_eq!(arm(&mut small, id as f64, 0.2), 1.);
    run(&mut small, 300);
    let after_small: f64 = small.read_by_name("FUEL_TANK_QUANTITY_2");
    let lost_small = start_gal - after_small;
    assert!(lost_small > 0., "a 0.2 leak must remove real fuel from FlyByWire's own tank");

    let mut big = seeded_aircraft(start_gal);
    assert_eq!(arm(&mut big, id as f64, 1.0), 1.);
    run(&mut big, 300);
    let after_big: f64 = big.read_by_name("FUEL_TANK_QUANTITY_2");
    let lost_big = start_gal - after_big;
    assert!(lost_big > lost_small, "a bigger leak magnitude must lose more fuel over the same time: {lost_big} vs {lost_small}");
}

#[test]
fn a_pump_fault_shows_in_flybywires_own_pump_pressure() {
    let mut test_bed = aircraft();
    test_bed.write_by_name("FUELSYSTEM PUMP ACTIVE:19", true);
    run(&mut test_bed, 5);
    let healthy: f64 = test_bed.read_by_name("FUEL_PUMP_PRESSURE_NORM:19");
    assert!((healthy - 1.0).abs() < 1e-9, "healthy running pump: nominal pressure");

    let id = failure_id("28_fuel.pump.trim_left", "pump degradation");
    assert_eq!(arm(&mut test_bed, id as f64, 0.4), 1.);
    run(&mut test_bed, 10);
    let degraded: f64 = test_bed.read_by_name("FUEL_PUMP_PRESSURE_NORM:19");
    assert!((degraded - 0.6).abs() < 0.05, "a 0.4 pump fault must show as reduced FlyByWire pump pressure: {degraded}");
}

#[test]
fn a_healthy_aircraft_is_unchanged_by_the_fuel_ledger_quantity_authority() {
    let mut test_bed = aircraft();
    let gal = 5000. / systems::fuel::FUEL_GALLONS_TO_KG;
    test_bed.write_by_name("FUEL_TANK_QUANTITY_2", gal);
    run(&mut test_bed, 50);
    let qty: f64 = test_bed.read_by_name("FUEL_TANK_QUANTITY_2");
    assert!(
        (qty - gal).abs() < 1e-6,
        "an absent/implausible ledger true-mass value must never move FlyByWire's own tank: {qty} vs {gal}"
    );
}

#[test]
fn the_fuel_ledger_quantity_authority_closes_a_small_divergence_gradually_and_ignores_a_large_one() {
    let start_kg = 5000.;
    let start_gal = start_kg / systems::fuel::FUEL_GALLONS_TO_KG;

    let mut small = aircraft();
    small.write_by_name("FUEL_TANK_QUANTITY_2", start_gal);
    small.write_by_name("FUELSYSTEM TANK QUANTITY:2", start_gal);
    run(&mut small, 20);
    let diverged_gal = (start_kg + 100.0) / systems::fuel::FUEL_GALLONS_TO_KG;
    small.write_by_name("FUEL_TANK_QUANTITY_2", diverged_gal);
    run(&mut small, 60);
    let after_small_gal: f64 = small.read_by_name("FUEL_TANK_QUANTITY_2");
    let after_small_kg = after_small_gal * systems::fuel::FUEL_GALLONS_TO_KG;
    assert!(
        after_small_kg < start_kg + 100.0 - 1.0,
        "a small, plausible divergence must be closed, moving the real tank toward the ledger's true mass: {after_small_kg}"
    );
    assert!(
        after_small_kg > start_kg,
        "the correction must be rate-limited, not an instant jump to the ledger's value: {after_small_kg}"
    );

    let mut big = aircraft();
    big.write_by_name("FUEL_TANK_QUANTITY_2", start_gal);
    big.write_by_name("FUELSYSTEM TANK QUANTITY:2", start_gal);
    run(&mut big, 20);
    let far_gal = (start_kg + 5000.0) / systems::fuel::FUEL_GALLONS_TO_KG;
    big.write_by_name("FUEL_TANK_QUANTITY_2", far_gal);
    run(&mut big, 60);
    let after_big_gal: f64 = big.read_by_name("FUEL_TANK_QUANTITY_2");
    assert!(
        (after_big_gal - far_gal).abs() < 1e-6,
        "a divergence beyond the authority's sanity bound must be treated as invalid and never move the tank: {after_big_gal} vs {far_gal}"
    );
}

#[test]
fn a_healthy_running_engine_publishes_exactly_zero_gas_path_deltas() {
    let mut test_bed = aircraft();
    test_bed.write_by_name("ENGINE_N1:1", 20.0);
    test_bed.write_by_name("ENGINE_N2:1", 60.0);
    test_bed.write_by_name("ENGINE_N2_HEALTHY:1", 60.0);
    test_bed.write_by_name("ENGINE_N3:1", 70.0);
    test_bed.write_by_name("ENGINE_N3_HEALTHY:1", 70.0);
    test_bed.write_by_name("ENGINE_STATE:1", 1.0);
    test_bed.write_by_name("ENGINE_FF:1", 2.48 * 3600.0);
    run(&mut test_bed, 50);
    let egt_delta: f64 = test_bed.read_by_name("A32NX_ENG_1_EGT_DELTA_C");
    let oil_delta: f64 = test_bed.read_by_name("A32NX_ENG_1_OIL_TEMP_DELTA_C");
    let surge: f64 = test_bed.read_by_name("A32NX_ENG_1_GASPATH_SURGE");
    assert_eq!(egt_delta, 0.0, "nothing armed: EGT delta must be exactly 0");
    assert_eq!(oil_delta, 0.0, "nothing armed: oil temp delta must be exactly 0");
    assert_eq!(surge, 0.0, "a healthy engine must never read as surged");
}

fn accelerate_engine_1(test_bed: &mut SimulationTestBed<A380>) {
    for step in 1..=200 {
        let f = step as f64 / 200.0;
        test_bed.write_by_name("ENGINE_N1:1", 20.0 + 65.0 * f);
        test_bed.write_by_name("ENGINE_N2:1", 60.0 + 35.0 * f);
        test_bed.write_by_name("ENGINE_N2_HEALTHY:1", 60.0 + 35.0 * f);
        test_bed.write_by_name("ENGINE_N3:1", 70.0 + 27.0 * f);
        test_bed.write_by_name("ENGINE_N3_HEALTHY:1", 70.0 + 27.0 * f);
        test_bed.write_by_name("ENGINE_FF:1", (2.48 + 5.5 * f) * 3600.0);
        run(test_bed, 1);
    }
}

#[test]
fn a_vsv_jam_moves_the_egt_and_n3_deltas_continuously_with_magnitude() {
    let id = failure_id("72_air.vsv_1", "actuator jam");
    let run_with = |magnitude: f64| {
        let mut test_bed = aircraft();
        test_bed.write_by_name("ENGINE_N1:1", 20.0);
        test_bed.write_by_name("ENGINE_N2:1", 60.0);
        test_bed.write_by_name("ENGINE_N2_HEALTHY:1", 60.0);
        test_bed.write_by_name("ENGINE_N3:1", 70.0);
        test_bed.write_by_name("ENGINE_N3_HEALTHY:1", 70.0);
        test_bed.write_by_name("ENGINE_STATE:1", 1.0);
        test_bed.write_by_name("ENGINE_FF:1", 2.48 * 3600.0);
        run(&mut test_bed, 50);
        if magnitude > 0.0 {
            assert_eq!(arm(&mut test_bed, id as f64, magnitude), 1.);
        }
        run(&mut test_bed, 50);
        accelerate_engine_1(&mut test_bed);
        run(&mut test_bed, 50);
        let published = test_bed.query(|a| a.deep_systems.snapshot());
        let value = |name: &str| *published.get(name).unwrap_or_else(|| panic!("{name} is not published"));
        (
            value("A32NX_ENG_1_VSV_STALL_MARGIN_DELTA_PCT"),
            value("A32NX_ENG_1_N3_CAPABILITY_LOSS_PCT"),
            value("A32NX_ENG_1_GASPATH_SURGE"),
        )
    };
    let (healthy_margin, healthy_n3, healthy_surge) = run_with(0.0);
    let (small_margin, small_n3, _) = run_with(0.3);
    let (big_margin, big_n3, _) = run_with(1.0);

    assert_eq!(healthy_margin, 0.0, "a healthy engine's vanes follow their schedule");
    assert_eq!(healthy_n3, 0.0, "nothing armed: no N3 capability loss");
    assert_eq!(healthy_surge, 0.0, "a healthy engine accelerating normally must not read as surging");
    assert!(small_margin < 0.0, "a jammed VSV must cost stall margin once the spool accelerates: {small_margin}");
    assert!(small_n3 > 0.0, "the lost margin must cost N3 capability: {small_n3}");
    assert!(
        big_margin <= small_margin && big_n3 >= small_n3,
        "a worse jam never costs less: small {small_margin}/{small_n3}, big {big_margin}/{big_n3}"
    );
}

fn engines_at_ground_idle() -> SimulationTestBed<A380> {
    let mut test_bed = aircraft();
    for n in 1..=4 {
        test_bed.write_by_name(&format!("ENGINE_N1:{n}"), 18.5);
        test_bed.write_by_name(&format!("ENGINE_N2:{n}"), 68.7);
        test_bed.write_by_name(&format!("ENGINE_N2_HEALTHY:{n}"), 68.7);
        test_bed.write_by_name(&format!("ENGINE_N3:{n}"), 68.0);
        test_bed.write_by_name(&format!("ENGINE_N3_HEALTHY:{n}"), 68.0);
        test_bed.write_by_name(&format!("ENGINE_STATE:{n}"), 1.0);
        test_bed.write_by_name(&format!("ENGINE_FF:{n}"), 2.48 * 3600.0);
        test_bed.write_by_name(&format!("ENGINE_EGT:{n}"), 380.0);
    }
    run(&mut test_bed, 50);
    test_bed
}

fn fadec_frame(test_bed: &mut SimulationTestBed<A380>, eng: usize) -> f64 {
    let p = test_bed.query(|a| a.deep_systems.snapshot());
    let n2_loss = p[&format!("A32NX_ENG_{eng}_N2_CAPABILITY_LOSS_PCT")].clamp(0.0, 100.0) / 100.0;
    let n3_loss = p[&format!("A32NX_ENG_{eng}_N3_CAPABILITY_LOSS_PCT")].clamp(0.0, 100.0) / 100.0;
    let n3 = 68.0 * (1.0 - n3_loss);
    test_bed.write_by_name(&format!("ENGINE_N3:{eng}"), n3);
    test_bed.write_by_name(&format!("ENGINE_N2:{eng}"), (n3 + 0.7) * (1.0 - n2_loss));
    run(test_bed, 1);
    n3_loss * 100.0
}

#[test]
fn the_engine_model_reads_the_healthy_spool_so_its_own_n3_loss_cannot_feed_back() {
    let n3_loss = |test_bed: &mut SimulationTestBed<A380>, eng: usize| {
        test_bed.query(|a| a.deep_systems.snapshot())[&format!("A32NX_ENG_{eng}_N3_CAPABILITY_LOSS_PCT")]
    };
    for (id, eng) in [(72012.0, 1), (72005.0, 2)] {
        let range = |closed_loop: bool| {
            let mut test_bed = engines_at_ground_idle();
            assert_eq!(arm(&mut test_bed, id, 1.0), 1.);
            let mut frames = Vec::new();
            for frame in 0..360 {
                if closed_loop {
                    fadec_frame(&mut test_bed, eng);
                } else {
                    run(&mut test_bed, 1);
                }
                if frame >= 300 {
                    frames.push(n3_loss(&mut test_bed, eng));
                }
            }
            frames.iter().fold((f64::MAX, f64::MIN), |(lo, hi), l| (lo.min(*l), hi.max(*l)))
        };
        let (open_lo, open_hi) = range(false);
        let (lo, hi) = range(true);
        assert!(lo > 30.0, "{id}: the damaged core must keep losing N3 capability while the indicated N3 shows it: {lo}");
        assert!(
            hi - lo <= open_hi - open_lo + 3.0,
            "{id}: showing the loss on the indicated N3 must not make it swing more than the model does on its own: {lo}..{hi} vs {open_lo}..{open_hi}"
        );
    }
}

#[test]
fn a_destroyed_hp_compressor_leaves_the_core_below_self_sustaining_speed() {
    let mut healthy = engines_at_ground_idle();
    run(&mut healthy, 300);
    let p = healthy.query(|a| a.deep_systems.snapshot());
    assert_eq!(p["A32NX_ENG_1_N3_CAPABILITY_LOSS_PCT"], 0.0, "a healthy engine loses no N3 capability");

    let mut test_bed = engines_at_ground_idle();
    assert_eq!(arm(&mut test_bed, 72012.0, 1.0), 1.);
    run(&mut test_bed, 300);
    let p = test_bed.query(|a| a.deep_systems.snapshot());
    let achievable_n3 = 68.0 * (1.0 - p["A32NX_ENG_1_N3_CAPABILITY_LOSS_PCT"] / 100.0);
    assert!(achievable_n3 < 50.0, "a destroyed HP compressor must leave the core unable to sustain itself: {achievable_n3}");
    assert_eq!(p["A32NX_ENG_2_N3_CAPABILITY_LOSS_PCT"], 0.0, "the other engines are untouched");
}


