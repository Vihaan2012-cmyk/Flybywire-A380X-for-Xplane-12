use fbw_a380_emulator::presets;

fn arinc(raw: f64) -> (f32, u64) {
    let bits = raw as u64;
    (f32::from_bits((bits & 0xffff_ffff) as u32), (bits >> 32) & 0b11)
}

fn ir_gs(e: &mut fbw_a380_emulator::Emulator, n: u8) -> f32 {
    arinc(e.get_var(&format!("A32NX_ADIRS_IR_{n}_GROUND_SPEED"))).0
}

#[test]
#[ignore]
fn ir_ground_speed_in_cruise_and_parked() {
    let mut e = presets::cruise(37_000.0, 0.85);
    for i in 0..10 {
        presets::run_until(&mut e, 60.0, |_| false);
        eprintln!("cruise t+{:>3}s IR1 GS {:.1} kt (true GS {:.1} kt)", (i + 1) * 60, ir_gs(&mut e, 1), e.get_dataref("sim/flightmodel/position/groundspeed") * 1.943_844);
    }
    let mut p = presets::engines_started();
    for i in 0..10 {
        presets::run_until(&mut p, 60.0, |_| false);
        eprintln!("parked t+{:>3}s IR1 GS {:.1} kt IR2 {:.1} IR3 {:.1}", (i + 1) * 60, ir_gs(&mut p, 1), ir_gs(&mut p, 2), ir_gs(&mut p, 3));
    }
}

#[test]
#[ignore]
fn batteries_only_power() {
    let mut e = presets::cold_and_dark();
    for bat in ["BAT_1", "BAT_2", "BAT_ESS", "BAT_APU"] {
        e.set_var(&format!("A32NX_OVHD_ELEC_{bat}_PB_IS_AUTO"), 1.0);
    }
    presets::run_until(&mut e, 30.0, |_| false);
    let mut powered = Vec::new();
    for bus in [
        "DC_1", "DC_2", "DC_ESS", "DC_ESS_IN_FLIGHT", "DC_GND_FLT_SVC", "DC_BAT", "DC_HOT_1", "DC_HOT_2", "DC_HOT_3", "DC_HOT_4", "AC_1", "AC_2", "AC_3", "AC_4", "AC_ESS", "AC_EMER",
        "AC_ESS_SHED", "DC_ESS_SHED", "108PH", "247PP", "309PP", "AC_GND_FLT_SVC", "AC_STAT_INV",
    ] {
        powered.push(format!("{bus}={}", e.get_var(&format!("A32NX_ELEC_{bus}_BUS_IS_POWERED"))));
    }
    eprintln!("batteries only: {}", powered.join(" "));
    for n in ["1", "2", "ESS", "APU"] {
        eprintln!("BAT {n}: contactor {} potential {:.1} V", e.get_var(&format!("A32NX_ELEC_CONTACTOR_990PB{n}_IS_CLOSED")), e.get_var(&format!("A32NX_ELEC_BAT_{n}_POTENTIAL")));
    }
}

#[test]
#[ignore]
fn breakers_by_name() {
    let e = presets::cold_and_dark();
    for b in e.list_breakers() {
        let id = b.id.to_ascii_lowercase();
        if ["pfd", "nd", "mfd", "ewd", "sd", "cdu", "fire", "loop", "bat", "display", "du"].iter().any(|k| id.contains(k)) {
            eprintln!("{:<32} {:<40} consumers {:?} failures {:?} var {:?}", b.id, b.name, b.consumers, b.failures, b.plugin_var);
        }
    }
}
