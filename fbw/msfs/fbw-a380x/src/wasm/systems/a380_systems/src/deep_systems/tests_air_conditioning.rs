#[allow(unused_imports)]
use std::time::Duration;

#[allow(unused_imports)]
use systems::simulation::test::{ReadByName, SimulationTestBed, TestBed, WriteByName};

#[allow(unused_imports)]
use super::tests::{aircraft, arm, computers_healthy, failure_id, run, run_deriving_failures};
#[allow(unused_imports)]
use crate::A380;
#[allow(unused_imports)]
use systems::shared::arinc429::Arinc429Word;
#[allow(unused_imports)]
use uom::si::{f64::Length, length::foot};

#[test]
fn a_healthy_aircraft_is_unchanged_by_the_air_conditioning_authority() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 50);

    let pack_1_regul: f64 = test_bed.read_by_name("DEEP_PNEU_PACK_1_REGUL_FAULT");
    let pack_1_fcv_1: f64 = test_bed.read_by_name("DEEP_PNEU_PACK_1_FCV_1_FAULT");
    let pack_1_fcv_2: f64 = test_bed.read_by_name("DEEP_PNEU_PACK_1_FCV_2_FAULT");
    let pack_2_regul: f64 = test_bed.read_by_name("DEEP_PNEU_PACK_2_REGUL_FAULT");
    let cpc_1_fault: f64 = test_bed.read_by_name("DEEP_CPC_1_SENSOR_FAULT");
    let cpc_2_fault: f64 = test_bed.read_by_name("DEEP_CPC_2_SENSOR_FAULT");
    assert_eq!(pack_1_regul, 0.0, "setup: a healthy aircraft must publish zero pack 1 regulation fault");
    assert_eq!(pack_1_fcv_1, 0.0, "setup: a healthy aircraft must publish zero pack 1 FCV 1 fault");
    assert_eq!(pack_1_fcv_2, 0.0, "setup: a healthy aircraft must publish zero pack 1 FCV 2 fault");
    assert_eq!(pack_2_regul, 0.0, "setup: a healthy aircraft must publish zero pack 2 regulation fault");
    assert_eq!(cpc_1_fault, 0.0, "setup: a healthy aircraft must publish zero CPC 1 sensor fault");
    assert_eq!(cpc_2_fault, 0.0, "setup: a healthy aircraft must publish zero CPC 2 sensor fault");

    let pressure_ratio: f64 = test_bed.read_by_name("COND_PACK_1_ACM_PRESSURE_RATIO");
    assert!(
        pressure_ratio.is_finite() && pressure_ratio >= 1.0,
        "a healthy pack must still produce a real compressor pressure ratio: {pressure_ratio}"
    );
    let cabin_altitude_ft: f64 = { let w: Arinc429Word<Length> = test_bed.read_arinc429_by_name("PRESS_CABIN_ALTITUDE_B1"); w.value().get::<foot>() };
    assert!(
        cabin_altitude_ft.is_finite(),
        "a healthy CPC must still produce a real cabin altitude reading: {cabin_altitude_ft}"
    );

    test_bed.write_by_name("DEEP_CPC_1_SENSOR_FAULT", 0.0);
    run(&mut test_bed, 10);
    let cabin_altitude_ft_after_explicit_zero: f64 = { let w: Arinc429Word<Length> = test_bed.read_arinc429_by_name("PRESS_CABIN_ALTITUDE_B1"); w.value().get::<foot>() };
    assert!(
        (cabin_altitude_ft_after_explicit_zero - cabin_altitude_ft).abs() < 100.0,
        "writing an explicit 0 CPC fault must read back close to the same cabin altitude as the default: {cabin_altitude_ft} vs {cabin_altitude_ft_after_explicit_zero}"
    );
}

#[test]
fn a_deep_cpc_sensor_fault_moves_flybywires_own_cabin_altitude_continuously_with_severity() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 20);
    let healthy_alt_ft: f64 = { let w: Arinc429Word<Length> = test_bed.read_arinc429_by_name("PRESS_CABIN_ALTITUDE_B1"); w.value().get::<foot>() };

    test_bed.write_by_name("DEEP_CPC_1_SENSOR_FAULT", 0.3);
    run(&mut test_bed, 20);
    let mild_alt_ft: f64 = { let w: Arinc429Word<Length> = test_bed.read_arinc429_by_name("PRESS_CABIN_ALTITUDE_B1"); w.value().get::<foot>() };
    let mild_bias_ft = mild_alt_ft - healthy_alt_ft;
    assert!(
        mild_bias_ft.abs() > 300.0,
        "a 0.3 CPC transducer fault must measurably bias FlyByWire's own cabin altitude reading: healthy {healthy_alt_ft} ft, mild {mild_alt_ft} ft"
    );

    test_bed.write_by_name("DEEP_CPC_1_SENSOR_FAULT", 1.0);
    run(&mut test_bed, 20);
    let severe_alt_ft: f64 = { let w: Arinc429Word<Length> = test_bed.read_arinc429_by_name("PRESS_CABIN_ALTITUDE_B1"); w.value().get::<foot>() };
    let severe_bias_ft = severe_alt_ft - healthy_alt_ft;
    assert!(
        severe_bias_ft.abs() > mild_bias_ft.abs(),
        "a bigger fault (1.0) must bias the altitude further than a smaller one (0.3): mild bias {mild_bias_ft} ft, severe bias {severe_bias_ft} ft"
    );
    assert!(
        (severe_bias_ft.abs() - 4000.0).abs() < 500.0,
        "a fully-faulted (1.0) CPC transducer must bias the altitude by close to the documented ceiling: got {severe_bias_ft} ft"
    );

    let b2_alt_ft: f64 = { let w: Arinc429Word<Length> = test_bed.read_arinc429_by_name("PRESS_CABIN_ALTITUDE_B2"); w.value().get::<foot>() };
    assert!(
        b2_alt_ft.is_finite(),
        "CPIOM-B2 must still produce a real reading: {b2_alt_ft}"
    );

    test_bed.write_by_name("DEEP_CPC_1_SENSOR_FAULT", 0.0);
    run(&mut test_bed, 20);
    let recovered_alt_ft: f64 = { let w: Arinc429Word<Length> = test_bed.read_arinc429_by_name("PRESS_CABIN_ALTITUDE_B1"); w.value().get::<foot>() };
    assert!(
        (recovered_alt_ft - healthy_alt_ft).abs() < 500.0,
        "clearing the CPC fault must recover close to the original reading: healthy {healthy_alt_ft} ft, recovered {recovered_alt_ft} ft"
    );
}

fn start_apu_and_hot_air_for_real(test_bed: &mut SimulationTestBed<A380>) {
    let feed_four_gal = 5000. / systems::fuel::FUEL_GALLONS_TO_KG;
    test_bed.write_by_name("FUEL_TANK_QUANTITY_9", feed_four_gal);
    test_bed.write_by_name("FUELSYSTEM TANK QUANTITY:9", feed_four_gal);
    test_bed.write_by_name("OVHD_APU_MASTER_SW_PB_IS_ON", true);
    test_bed.write_by_name("OVHD_APU_START_PB_IS_ON", true);
    for _ in 0..60 {
        run(test_bed, 50);
        let n: f64 = test_bed.read_by_name("APU_N_RAW");
        if n > 99.0 {
            break;
        }
    }
    let n: f64 = test_bed.read_by_name("APU_N_RAW");
    assert!(n > 99.0, "setup: APU_N did not reach a governed running state within the test's frame budget: {n}");

    test_bed.write_by_name("OVHD_APU_BLEED_PB_IS_ON", true);
    test_bed.write_by_name("OVHD_COND_PACK_1_PB_IS_ON", true);
    test_bed.write_by_name("OVHD_COND_PACK_2_PB_IS_ON", true);
    test_bed.write_by_name("OVHD_COND_HOT_AIR_1_PB_IS_ON", true);
    test_bed.write_by_name("OVHD_COND_HOT_AIR_2_PB_IS_ON", true);

    for _ in 0..60 {
        run(test_bed, 50);
        let tcs: Arinc429Word<u32> = test_bed.read_arinc429_by_name("COND_CPIOM_B1_TCS_DISCRETE_WORD");
        if tcs.get_bit(15) {
            return;
        }
    }
    let tcs: Arinc429Word<u32> = test_bed.read_arinc429_by_name("COND_CPIOM_B1_TCS_DISCRETE_WORD");
    panic!(
        "setup: the hot-air valve 1 did not indicate open within the test's frame budget: disagree 1 {}, disagree 2 {}, open 1 {}, open 2 {}",
        tcs.get_bit(13), tcs.get_bit(14), tcs.get_bit(15), tcs.get_bit(16)
    );
}

#[test]
fn losing_only_the_hot_air_valve_1_position_indication_feed_does_not_jam_the_valve_but_the_tcs_sees_a_disagree() {
    let mut test_bed = aircraft();
    start_apu_and_hot_air_for_real(&mut test_bed);

    let pos_ind_open_circuit = failure_id("21_elec.hotair-1-pos-ind", "open circuit") as f64;

    assert_eq!(arm(&mut test_bed, pos_ind_open_circuit, 1.0), 1.0);

    run_deriving_failures(&mut test_bed, 400);

    let tcs: Arinc429Word<u32> = test_bed.read_arinc429_by_name("COND_CPIOM_B1_TCS_DISCRETE_WORD");
    assert!(
        tcs.get_bit(13),
        "losing the hot-air valve 1 position-indication feed must make the TCS discrete word see a hot air 1 disagree (COND HOT AIR 1 FAULT: the hot-air valve is failed closed)"
    );
    assert!(
        !tcs.get_bit(15),
        "the corrupted position-indication feed must show hot air 1 as closed on the TCS word"
    );
    assert!(
        !tcs.get_bit(14),
        "the hot-air valve 2 feed is untouched by the hot-air valve 1 position-indication failure, so it must not show a disagree"
    );
}
