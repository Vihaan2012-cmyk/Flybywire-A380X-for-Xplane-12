use fbw_a380_emulator::presets;

fn flex(e: &mut fbw_a380_emulator::Emulator) -> [f64; 4] {
    ["INBOARD", "INBOARD_MID", "OUTBOARD_MID", "OUTBOARD"].map(|s| e.get_var(&format!("A32NX_WING_FLEX_LEFT_{s}")))
}

#[test]
fn parked_wings_droop_slightly_instead_of_standing_up_like_flight() {
    let mut e = presets::engines_started();
    e.set_agl_ft(0.0);
    for (i, c) in [58.0, 61.0, 61.0, 56.0, 56.0].into_iter().enumerate() {
        let name = if i == 0 { "CONTACT POINT COMPRESSION".to_owned() } else { format!("CONTACT POINT COMPRESSION:{i}") };
        e.set_var(&name, c);
    }
    presets::run_until(&mut e, 20.0, |_| false);
    let parked = flex(&mut e);
    assert!(parked.iter().all(|a| *a < 0.5), "parked wing flex angles {parked:?} deg should be a slight droop, not a flight bend");
}
