use fbw_a380_emulator::presets;

fn arinc(raw: f64) -> (f32, u64) {
    let bits = raw as u64;
    (f32::from_bits((bits & 0xffff_ffff) as u32), (bits >> 32) & 0b11)
}

#[test]
#[ignore]
fn ext_power_reconnects_a_dark_aircraft() {
    let mut e = presets::powered();
    for n in 1..=4 {
        e.set_var(&format!("EXT_PWR_AVAIL:{n}"), 1.0);
    }
    presets::run_until(&mut e, 2.0, |_| false);
    let buses = |e: &mut fbw_a380_emulator::Emulator| -> String {
        (1..=4).map(|n| format!("{}", e.get_var(&format!("A32NX_ELEC_AC_{n}_BUS_IS_POWERED")) as i32)).collect::<String>()
            + &format!(" dcess {}", e.get_var("A32NX_ELEC_DC_ESS_BUS_IS_POWERED") as i32)
    };
    eprintln!("on APU: AC {}", buses(&mut e));
    for n in 1..=4 {
        e.set_var(&format!("A32NX_OVHD_ELEC_EXT_PWR_{n}_PB_IS_ON"), 0.0);
    }
    e.set_var("A32NX_OVHD_APU_MASTER_SW_PB_IS_ON", 0.0);
    e.set_var("A32NX_OVHD_ELEC_APU_GEN_1_PB_IS_ON", 0.0);
    e.set_var("A32NX_OVHD_ELEC_APU_GEN_2_PB_IS_ON", 0.0);
    presets::run_until(&mut e, 120.0, |_| false);
    eprintln!("APU off, EXT PWR off: AC {} apu_n {:?}", buses(&mut e), arinc(e.get_var("A32NX_APU_N")));
    for n in 1..=4 {
        e.set_var(&format!("A32NX_OVHD_ELEC_EXT_PWR_{n}_PB_IS_ON"), 1.0);
    }
    presets::run_until(&mut e, 10.0, |_| false);
    eprintln!("EXT PWR back on: AC {} avail {}", buses(&mut e), (1..=4).map(|n| format!("{}", e.get_var(&format!("EXT_PWR_AVAIL:{n}")) as i32)).collect::<String>());
}

#[test]
#[ignore]
fn apu_n_word_during_a_start() {
    let mut e = presets::cold_and_dark();
    for bat in ["BAT_1", "BAT_2", "BAT_ESS", "BAT_APU"] {
        e.set_var(&format!("A32NX_OVHD_ELEC_{bat}_PB_IS_AUTO"), 1.0);
    }
    presets::run_until(&mut e, 2.0, |_| false);
    e.set_var("A32NX_OVHD_APU_MASTER_SW_PB_IS_ON", 1.0);
    for i in 0..12 {
        if i == 1 {
            e.set_var("A32NX_OVHD_APU_START_PB_IS_ON", 1.0);
        }
        presets::run_until(&mut e, 5.0, |_| false);
        let raw = e.get_var("A32NX_APU_N");
        eprintln!("t+{:>3}s APU_N raw {raw} -> {:?}", (i + 1) * 5, arinc(raw));
    }
}

#[test]
#[ignore]
fn engine_start_timeline() {
    let mut e = presets::powered();
    for n in 1..=4 {
        e.set_var(&format!("TURB ENG IGNITION SWITCH EX1:{n}"), 2.0);
    }
    presets::run_until(&mut e, 1.0, |_| false);
    e.set_var("GENERAL ENG STARTER:1", 1.0);
    for i in 0..40 {
        presets::run_until(&mut e, 2.0, |_| false);
        eprintln!(
            "t+{:>3}s state {} N1 {:.1} N3 {:.1} EGT {:.0} FF {:.0}",
            (i + 1) * 2,
            e.get_var("ENGINE_STATE:1"),
            e.get_var("A32NX_ENGINE_N1:1"),
            e.get_var("A32NX_ENGINE_N3:1"),
            e.get_var("A32NX_ENGINE_EGT:1"),
            e.get_var("A32NX_ENGINE_FF:1"),
        );
    }
    e.set_var("TURB ENG IGNITION SWITCH EX1:1", 1.0);
    e.set_thrust_lever(1, 1.0);
    for i in 0..10 {
        presets::run_until(&mut e, 2.0, |_| false);
        eprintln!(
            "lever full t+{:>2}s state {} N1 {:.1} TLA_N1 {:.1} idle {:.1} toga {:.1} EGT {:.0}",
            (i + 1) * 2,
            e.get_var("ENGINE_STATE:1"),
            e.get_var("A32NX_ENGINE_N1:1"),
            e.get_var("A32NX_AUTOTHRUST_TLA_N1:1"),
            e.get_var("A32NX_AUTOTHRUST_THRUST_LIMIT_IDLE"),
            e.get_var("A32NX_AUTOTHRUST_THRUST_LIMIT_TOGA"),
            e.get_var("A32NX_ENGINE_EGT:1"),
        );
    }
}
