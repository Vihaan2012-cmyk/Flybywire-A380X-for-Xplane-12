//! A seized pneumatic valve (failures.rs PNEUMATIC_VALVES, FlyByWire's
//! pneumatic model patched with `ValveSeizure`) ignores its controller: with
//! engine 1's bleed valve seized open, switching ENG 1 BLEED off leaves it
//! open, while engine 2's, switched off at the same time, closes.

use fbw_a380_emulator::{presets, Emulator};

fn pr_open(e: &mut Emulator, n: u8) -> f64 {
    e.get_var(&format!("PNEU_ENG_{n}_PR_VALVE_OPEN"))
}

#[test]
fn a_seized_bleed_valve_stays_open_when_its_bleed_is_switched_off() {
    let mut e = presets::engines_running();
    e.run(presets::PRESET_DT, 100);
    assert!(pr_open(&mut e, 1) > 0.5 && pr_open(&mut e, 2) > 0.5, "both bleed valves open with engines running");

    e.set_failure_magnitude(36_012, 1.0); // Engine 1 bleed valve stuck
    e.run(presets::PRESET_DT, 5);
    for n in [1, 2] {
        e.set_var(&format!("OVHD_PNEU_ENG_{n}_BLEED_PB_IS_AUTO"), 0.0);
    }
    e.run(presets::PRESET_DT, 100);
    assert!(pr_open(&mut e, 2) < 0.5, "engine 2's healthy valve closes when its bleed is switched off");
    assert!(pr_open(&mut e, 1) > 0.5, "engine 1's seized valve stays open");

    // Repaired: it answers its switch again.
    e.set_failure_magnitude(36_012, 0.0);
    e.run(presets::PRESET_DT, 100);
    assert!(pr_open(&mut e, 1) < 0.5, "repaired, engine 1's valve closes");
}
