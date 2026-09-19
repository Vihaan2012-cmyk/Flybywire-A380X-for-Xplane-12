use fbw_a380_emulator::{presets, Emulator};
fn run(build: fn() -> Emulator) -> Vec<(String, f64)> {
    std::thread::Builder::new().stack_size(64 << 20).spawn(move || { let mut e = build(); e.run(0.1, 200); e.snapshot_all() }).unwrap().join().unwrap()
}
#[test]
fn probe() {
    let builders: [(&str, fn() -> Emulator); 4] = [("cold", presets::cold_and_dark), ("gpu", presets::ground_power), ("powered", presets::powered), ("running", presets::engines_running)];
    for (name, b) in builders {
        let a = run(b);
        let mut diff = std::collections::BTreeSet::new();
        for _ in 0..5 {
            let m: std::collections::HashMap<_, _> = run(b).into_iter().collect();
            diff.extend(a.iter().filter(|(n, v)| m.get(n).map_or(true, |w| (v - w).abs() > 1e-3 * v.abs().max(1.0) || v.is_nan() != w.is_nan())).map(|(n, _)| n.clone()));
        }
        println!("NOISE {name} {} {:?}", diff.len(), diff);
    }
}
