#[allow(unused_imports)]
use systems::simulation::test::{ReadByName, SimulationTestBed, TestBed, WriteByName};

#[allow(unused_imports)]
use super::tests::{aircraft, arm, failure_id, run};
#[allow(unused_imports)]
use crate::A380;

#[test]
fn a_healthy_aircraft_is_unchanged_by_the_fire_authority() {
    let mut test_bed = aircraft();
    run(&mut test_bed, 100);

    for id in ["1_ENG_1", "2_ENG_1", "1_ENG_2", "2_ENG_3", "1_ENG_4", "1_APU_1"] {
        let remaining: f64 = test_bed.read_by_name(&format!("FIRE_BOTTLE_{id}_AGENT_REMAINING_FRACTION"));
        assert_eq!(remaining, 1.0, "FIRE_BOTTLE_{id}_AGENT_REMAINING_FRACTION must read a full bottle on a healthy aircraft, got {remaining}");

        let deep_loss: f64 = test_bed.read_by_name(&format!("DEEP_FIRE_BOTTLE_{id}_CHARGE_LOSS_FRACTION"));
        assert_eq!(deep_loss, 0.0, "DEEP_FIRE_BOTTLE_{id}_CHARGE_LOSS_FRACTION must publish exactly 0 on a healthy aircraft");

        let discharged: f64 = test_bed.read_by_name(&format!("FIRE_SQUIB_{id}_IS_DISCHARGED"));
        assert_eq!(discharged, 0.0, "FIRE_SQUIB_{id}_IS_DISCHARGED must stay false with nothing armed and no command");
    }
}

#[test]
fn a_deep_bottle_leak_moves_flybywires_own_agent_quantity_continuously_with_severity() {
    let leak_id = failure_id("26_fire.eng1_bottle1", "leak");

    let mut healthy = aircraft();
    run(&mut healthy, 300);
    let healthy_remaining: f64 =
        healthy.read_by_name("FIRE_BOTTLE_1_ENG_1_AGENT_REMAINING_FRACTION");
    assert_eq!(healthy_remaining, 1.0, "setup: a healthy bottle must stay full");

    let mut mild = aircraft();
    arm(&mut mild, leak_id as f64, 0.3);
    run(&mut mild, 300);
    let mild_remaining: f64 = mild.read_by_name("FIRE_BOTTLE_1_ENG_1_AGENT_REMAINING_FRACTION");
    let mild_loss: f64 = mild.read_by_name("DEEP_FIRE_BOTTLE_1_ENG_1_CHARGE_LOSS_FRACTION");

    let mut severe = aircraft();
    arm(&mut severe, leak_id as f64, 1.0);
    run(&mut severe, 300);
    let severe_remaining: f64 = severe.read_by_name("FIRE_BOTTLE_1_ENG_1_AGENT_REMAINING_FRACTION");
    let severe_loss: f64 = severe.read_by_name("DEEP_FIRE_BOTTLE_1_ENG_1_CHARGE_LOSS_FRACTION");

    assert!(
        mild_loss > 0.0,
        "a 0.3 bottle leak must measurably move FlyByWire's own deep input above 0 within 30 s: got {mild_loss}"
    );
    assert!(
        mild_remaining < healthy_remaining,
        "a 0.3 bottle leak must move FlyByWire's own agent-remaining reading below full: {healthy_remaining} -> {mild_remaining}"
    );
    assert!(
        severe_loss > mild_loss,
        "a full-severity leak must drain faster than a 0.3 leak over the same time, proving the input scales with magnitude rather than tripping one fixed rate: mild {mild_loss}, severe {severe_loss}"
    );
    assert!(
        severe_remaining < mild_remaining,
        "the full-severity bottle must read a lower agent quantity than the mild one over the same time: mild {mild_remaining}, severe {severe_remaining}"
    );

    let eng2_remaining: f64 = severe.read_by_name("FIRE_BOTTLE_1_ENG_2_AGENT_REMAINING_FRACTION");
    assert_eq!(eng2_remaining, 1.0, "a leak on engine 1's bottle must not drain engine 2's bottle");

    let severe_empty: f64 = severe.read_by_name("DEEP_FIRE_BOTTLE_1_ENG_1_EMPTY");
    assert_eq!(severe_empty, 0.0, "setup: 30 s of even a full-severity leak must not already empty the bottle outright");
}

#[test]
fn a_degraded_fusible_link_lets_a_lavatory_bin_fire_burn_on() {
    for degraded in [false, true] {
        let mut test_bed = aircraft();
        run(&mut test_bed, 20);
        if degraded {
            assert_eq!(arm(&mut test_bed, failure_id("26_fire.lavatory_extinguisher", "fusible link") as f64, 1.), 1.);
        }
        assert_eq!(arm(&mut test_bed, failure_id("26_fire.lavatory_bin", "waste bin fire") as f64, 0.5), 1.);
        run(&mut test_bed, 6000);
        let discharged: f64 = test_bed.read_by_name("LAVATORY_EXTINGUISHER_DISCHARGED");
        let burning: f64 = test_bed.read_by_name("LAVATORY_1_BIN_FIRE");
        let alarm: f64 = test_bed.read_by_name("DEEP_SMOKE_LAV_1_ALARM");
        let bin_c: f64 = test_bed.read_by_name("LAVATORY_1_BIN_TEMP_C");
        if degraded {
            assert_eq!((discharged, burning, alarm), (0., 1., 1.), "degraded link: never discharged, still burning, smoke alarm on (bin {bin_c} C)");
        } else {
            assert_eq!((discharged, burning), (1., 0.), "healthy link: discharged and the fire is out (bin {bin_c} C)");
        }
    }
}
