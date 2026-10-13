use fbw_a380_emulator::presets;

#[test]
#[ignore]
fn engine_starts_across_hot_and_high_conditions() {
    for (oat_c, alt_ft) in [(15.0, 0.0), (40.0, 0.0), (50.0, 0.0), (30.0, 8000.0), (40.0, 5000.0), (-20.0, 0.0)] {
        let mut e = presets::powered();
        e.set_oat_c(oat_c);
        e.set_pressure_altitude_ft(alt_ft);
        presets::run_until(&mut e, 30.0, |_| false);
        for n in 1..=4 {
            e.set_var(&format!("TURB ENG IGNITION SWITCH EX1:{n}"), 2.0);
        }
        presets::run_until(&mut e, 1.0, |_| false);
        let mut line = Vec::new();
        for n in [4u8, 3, 2, 1] {
            e.set_var(&format!("GENERAL ENG STARTER:{n}"), 1.0);
            let mut peak = f64::MIN;
            let mut t = 0.0;
            let started = presets::run_until(&mut e, 180.0, |e| {
                t += 1.0 / 30.0;
                peak = peak.max(e.get_var(&format!("A32NX_ENGINE_EGT:{n}")));
                presets::engine_running(e, n)
            });
            line.push(format!("eng{n} {} peak EGT {peak:.0} C", if started { "started" } else { "DID NOT START" }));
        }
        eprintln!("OAT {oat_c:>5.1} C, {alt_ft:>5.0} ft: {}", line.join(", "));
    }
}
