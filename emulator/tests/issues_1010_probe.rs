use fbw_a380_emulator::presets;

fn buses(e: &mut fbw_a380_emulator::Emulator) -> String {
    ["AC_1", "AC_2", "AC_3", "AC_4", "AC_ESS", "DC_1", "DC_2", "DC_ESS"]
        .iter()
        .map(|b| format!("{b}={}", e.get_var(&format!("A32NX_ELEC_{b}_BUS_IS_POWERED"))))
        .collect::<Vec<_>>()
        .join(" ")
}

#[test]
#[ignore]
fn each_external_power_unit_alone() {
    for unit in 1..=4 {
        let mut e = presets::cold_and_dark();
        for bat in ["BAT_1", "BAT_2", "BAT_ESS", "BAT_APU"] {
            e.set_var(&format!("A32NX_OVHD_ELEC_{bat}_PB_IS_AUTO"), 1.0);
        }
        presets::run_until(&mut e, 2.0, |_| false);
        e.set_var(&format!("EXT_PWR_AVAIL:{unit}"), 1.0);
        presets::run_until(&mut e, 1.0, |_| false);
        let avail = e.get_var(&format!("A32NX_OVHD_ELEC_EXT_PWR_{unit}_PB_IS_AVAILABLE"));
        e.set_var(&format!("A32NX_OVHD_ELEC_EXT_PWR_{unit}_PB_IS_ON"), 1.0);
        presets::run_until(&mut e, 3.0, |_| false);
        eprintln!(
            "EXT PWR {unit}: pb available {avail} on {} on line {} | {}",
            e.get_var(&format!("A32NX_OVHD_ELEC_EXT_PWR_{unit}_PB_IS_ON")),
            e.get_var(&format!("A32NX_ELEC_EXT_PWR_{unit}_ON_LINE")),
            buses(&mut e)
        );
    }
}

#[test]
#[ignore]
fn parking_brake_and_brake_accumulator_after_a_cold_spawn() {
    let mut e = presets::cold_and_dark();
    let show = |e: &mut fbw_a380_emulator::Emulator, when: &str| {
        eprintln!(
            "{when}: park {} | altn acc {:.0} psi altn L/R {:.0}/{:.0} norm L/R {:.0}/{:.0} | yellow {:.0} green {:.0}",
            e.get_var("A32NX_PARK_BRAKE_LEVER_POS"),
            e.get_var("A32NX_HYD_BRAKE_ALTN_ACC_PRESS"),
            e.get_var("A32NX_HYD_BRAKE_ALTN_LEFT_PRESS"),
            e.get_var("A32NX_HYD_BRAKE_ALTN_RIGHT_PRESS"),
            e.get_var("A32NX_HYD_BRAKE_NORM_LEFT_PRESS"),
            e.get_var("A32NX_HYD_BRAKE_NORM_RIGHT_PRESS"),
            e.get_var("A32NX_HYD_YELLOW_SYSTEM_1_SECTION_PRESSURE"),
            e.get_var("A32NX_HYD_GREEN_SYSTEM_1_SECTION_PRESSURE"),
        )
    };
    show(&mut e, "spawn");
    e.set_var("A32NX_PARK_BRAKE_LEVER_POS", 1.0);
    presets::run_until(&mut e, 5.0, |_| false);
    show(&mut e, "park set, cold");
    for bat in ["BAT_1", "BAT_2", "BAT_ESS", "BAT_APU"] {
        e.set_var(&format!("A32NX_OVHD_ELEC_{bat}_PB_IS_AUTO"), 1.0);
    }
    e.set_var("EXT_PWR_AVAIL:1", 1.0);
    e.set_var("A32NX_OVHD_ELEC_EXT_PWR_1_PB_IS_ON", 1.0);
    presets::run_until(&mut e, 5.0, |_| false);
    show(&mut e, "ext pwr on");
    for pump in ["YA", "YB", "GA", "GB"] {
        e.set_var(&format!("A32NX_OVHD_HYD_EPUMP{pump}_ON_PB_IS_AUTO"), 0.0);
    }
    for t in 1..=6 {
        presets::run_until(&mut e, 10.0, |_| false);
        show(&mut e, &format!("pumps on +{}s", t * 10));
    }
}
