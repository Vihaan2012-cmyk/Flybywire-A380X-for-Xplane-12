use fbw_a380_emulator::presets;

fn fdu_bit(e: &mut fbw_a380_emulator::Emulator, bit: u32) -> bool {
    let raw = e.get_var("A32NX_FIRE_FDU_DISCRETE_WORD") as u64;
    let word = f32::from_bits((raw & 0xffff_ffff) as u32) as u32;
    word & (1 << (bit - 1)) != 0
}

#[test]
fn pulling_a_loop_breaker_faults_that_loop_and_pulling_both_is_not_a_fire() {
    let mut e = presets::powered();
    presets::run_until(&mut e, 5.0, |_| false);
    assert!(!fdu_bit(&mut e, 18) && !fdu_bit(&mut e, 19), "ENG 1 loops start healthy");

    e.pull_breaker("fire-loop-eng-1-A");
    presets::run_until(&mut e, 2.0, |_| false);
    assert_eq!(e.get_var("A32NX_ELEC_FIRE_LOOP_A_1_BREAKER_OPEN"), 1.0, "the breaker reaches FlyByWire");
    assert!(fdu_bit(&mut e, 18), "ENG 1 LOOP A reads failed with its breaker out");
    assert!(!fdu_bit(&mut e, 19), "LOOP B is untouched");

    e.pull_breaker("fire-loop-eng-1-B");
    presets::run_until(&mut e, 6.0, |_| false);
    assert!(fdu_bit(&mut e, 18) && fdu_bit(&mut e, 19), "both ENG 1 loops read failed");
    assert_eq!(e.get_var("A32NX_FIRE_DETECTED_ENG1"), 0.0, "two dead breakers are a detection fault, not a fire");
    assert!(!fdu_bit(&mut e, 11), "no ENG 1 FIRE in the discrete word");

    e.reset_all_breakers();
    presets::run_until(&mut e, 2.0, |_| false);
    assert!(!fdu_bit(&mut e, 18) && !fdu_bit(&mut e, 19), "resetting the breakers clears both faults");
}
