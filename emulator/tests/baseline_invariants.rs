//! Every start state, left alone for 20 s, breaches no physical invariant:
//! the tiered battery counts any invariant as a failure, so a baseline
//! breach (FlyByWire's oil pressure curve fit writing -0.89 psi at rest,
//! once) would fail every case of a run.

use fbw_a380_emulator::{presets, Emulator};
#[test]
fn baseline_presets_breach_no_invariant() {
    let builders: [(&str, fn() -> Emulator); 4] = [("cold", presets::cold_and_dark), ("gpu", presets::ground_power), ("powered", presets::powered), ("running", presets::engines_running)];
    for (name, build) in builders {
        let out = std::thread::Builder::new().stack_size(64 << 20).spawn(move || {
            let mut e = build();
            e.run(0.1, 200);
            e.invariant_report().iter().map(|v| (v.name.clone(), v.last_value)).collect::<Vec<_>>()
        }).unwrap().join().unwrap();
        assert!(out.is_empty(), "{name}: {out:?}");
    }
}
