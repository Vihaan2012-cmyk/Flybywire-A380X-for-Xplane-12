use super::{phase, proc, sd_page, FbwProc};
use crate::deep::api::{all, any, var, Cond, Level};

fn network_alive() -> Cond {
    any(vec![
        var("ELEC_AC_1_BUS_IS_POWERED").on(),
        var("ELEC_AC_2_BUS_IS_POWERED").on(),
        var("ELEC_AC_3_BUS_IS_POWERED").on(),
        var("ELEC_AC_4_BUS_IS_POWERED").on(),
    ])
}

fn ac_dead(tag: &str) -> Cond {
    all(vec![var(&format!("ELEC_{tag}_BUS_POTENTIAL")).lt(90.0), var(&format!("ELEC_{tag}_BUS_IS_POWERED")).off()])
}

fn dc_dead(tag: &str) -> Cond {
    var(&format!("ELEC_{tag}_BUS_IS_POWERED")).off()
}

fn bus_fault(id: u64, title: &'static str, parts: Vec<Cond>, note: &'static str) -> FbwProc {
    let mut conds = parts;
    conds.push(network_alive());
    proc(id, title, Level::Caution, sd_page::ELEC_AC, all(conds), note).confirm(0.5).inhibit(phase::TAKEOFF_AND_LANDING_ROLL)
}

pub fn wire(v: &mut Vec<FbwProc>) {
    v.push(
        bus_fault(240_800_004, "ELEC AC BUS 2 FAULT", vec![ac_dead("AC_2")], "AC bus 2 unpowered and below 90 V while the AC network is live")
            .suppressed_by(&[240_800_005, 240_800_006])
            .inhibit(&[4, 5, 10])
            .items(8, Vec::new()),
    );
    v.push(
        bus_fault(240_800_007, "ELEC AC BUS 3 FAULT", vec![ac_dead("AC_3")], "AC bus 3 unpowered and below 90 V while the AC network is live")
            .suppressed_by(&[240_800_005, 240_800_008])
            .inhibit(&[4, 5, 10])
            .items(12, Vec::new()),
    );
    v.push(
        bus_fault(240_800_009, "ELEC AC BUS 4 FAULT", vec![ac_dead("AC_4")], "AC bus 4 unpowered and below 90 V while the AC network is live")
            .suppressed_by(&[240_800_006, 240_800_008])
            .inhibit(&[4, 5, 10])
            .items(9, Vec::new()),
    );
    v.push(
        bus_fault(240_800_010, "ELEC AC EMER BUS FAULT", vec![ac_dead("AC_ESS")], "the AC emergency bus is unpowered and below 90 V while the AC network is live").items(4, Vec::new()),
    );
    v.push(
        bus_fault(240_800_012, "ELEC AC ESS BUS FAULT", vec![ac_dead("AC_ESS_SHED")], "the AC essential bus is unpowered and below 90 V while the AC network is live").inhibit(&[4, 5, 10]).items(11, Vec::new()),
    );

    v.push(
        bus_fault(240_800_006, "ELEC AC BUS 2+4 FAULT", vec![ac_dead("AC_2"), ac_dead("AC_4")], "AC buses 2 and 4 both dead while 1 or 3 still carries the network")
            .items(28, Vec::new()),
    );
    v.push(
        bus_fault(
            240_800_005,
            "ELEC AC BUS 2+3 & DC BUS 1+2 FAULT",
            vec![ac_dead("AC_2"), ac_dead("AC_3"), dc_dead("DC_1"), dc_dead("DC_2")],
            "AC buses 2 and 3 and both main DC buses dead together -- the two TRs they feed lose their source with them",
        )
        .items(41, Vec::new()),
    );
    v.push(
        bus_fault(
            240_800_008,
            "ELEC AC BUS 3+4 & DC BUS 2 FAULT",
            vec![ac_dead("AC_3"), ac_dead("AC_4"), dc_dead("DC_2")],
            "AC buses 3 and 4 dead together with the DC bus 2 they feed through TR 2",
        )
        .items(15, Vec::new()),
    );

    v.push(
        bus_fault(240_800_026, "ELEC DC BUS 1 FAULT", vec![dc_dead("DC_1")], "DC bus 1 unpowered while the AC network is live")
            .suppressed_by(&[240_800_005, 240_800_027, 240_800_028])
            .items(11, Vec::new()),
    );
    v.push(
        bus_fault(240_800_029, "ELEC DC BUS 2 FAULT", vec![dc_dead("DC_2")], "DC bus 2 unpowered while the AC network is live")
            .suppressed_by(&[240_800_005, 240_800_008, 240_800_027])
            .items(12, Vec::new()),
    );
    v.push(
        bus_fault(240_800_027, "ELEC DC BUS 1+2 FAULT", vec![dc_dead("DC_1"), dc_dead("DC_2")], "both main DC buses unpowered while the AC network is live")
            .items(22, Vec::new()),
    );
    v.push(
        bus_fault(240_800_030, "ELEC DC ESS BUS FAULT", vec![dc_dead("DC_ESS")], "the DC essential bus is unpowered while the AC network is live")
            .suppressed_by(&[240_800_028])
            .items(20, Vec::new()),
    );
    v.push(
        bus_fault(240_800_028, "ELEC DC BUS 1+ESS FAULT", vec![dc_dead("DC_1"), dc_dead("DC_ESS")], "DC bus 1 and the DC essential bus unpowered together")
            .items(21, Vec::new()),
    );
    v.push(
        bus_fault(
            240_800_031,
            "ELEC DC ESS BUS PART FAULT",
            vec![var("ELEC_DC_ESS_BUS_IS_POWERED").on(), var("ELEC_DC_ESS_SHED_BUS_IS_POWERED").off()],
            "the DC essential bus is still live but its shed section is not -- part of the bus, which is what PART FAULT names",
        )
        .suppressed_by(&[240_800_030])
        .items(16, Vec::new()),
    );

    v.push(
        proc(240_800_016, "ELEC APU TR FAULT", Level::Caution, sd_page::ELEC_DC, var("ELEC_TR_APU_FAULT").on(), "deep::electrical's own verdict on the APU transformer-rectifier")
            .confirm(1.0)
            .items(2, Vec::new()),
    );
    v.push(
        proc(240_800_018, "ELEC BAT 2 (ESS) FAULT", Level::Caution, sd_page::ELEC_DC, var("ELEC_BAT_2_FAULT").on(), "deep::electrical's own verdict on battery 2; battery 1's own procedure is already raised by our ELEC BAT 1 FAULT")
            .confirm(1.0)
            .items(1, Vec::new()),
    );
    v.push(
        proc(240_800_017, "ELEC BAT 1 (ESS) FAULT", Level::Caution, sd_page::ELEC_DC, var("ELEC_BAT_1_FAULT").on(), "FCOM: BAT 1 is failed; deep::electrical's own verdict on battery 1")
            .confirm(2.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10])
            .items(1, Vec::new()),
    );
    v.push(
        proc(
            240_800_021,
            "ELEC C/B TRIPPED",
            Level::Caution,
            sd_page::CB,
            var("BREAKERS_TRIPPED_NOT_COMMANDED_COUNT").ge(1.0),
            "at least one of deep::breakers' 399 trip units was opened by its own thermal, magnetic, arc-fault or lockout element rather than by a crew command",
        )
        .confirm(1.0)
        .items(0, Vec::new()),
    );

    v.push(proc(240_800_082, "ELEC TR 2 FAULT", Level::Caution, sd_page::ELEC_DC, var("ELEC_TR_2_FAULT").on(), "deep::electrical's own verdict on TR 2; TR 1's procedure is already raised by our ELEC TR 1 FAULT").confirm(1.0));
    v.push(proc(240_800_083, "ELEC TR ESS FAULT", Level::Caution, sd_page::ELEC_DC, var("ELEC_TR_ESS_FAULT").on(), "deep::electrical's own verdict on the essential TR").confirm(1.0));

    v.push(
        proc(
            240_800_001,
            "ELEC ABNORMAL FLIGHT OPS SUPPLY",
            Level::Advisory,
            sd_page::ELEC_AC,
            all(vec![dc_dead("DC_2"), var("ELEC_DC_ESS_BUS_IS_POWERED").on()]),
            "FCOM PRO-ABN-ECAM p.4831: the OITs' normal DC 2 source is dead while they are still fed via DC ESS",
        )
        .confirm(1.0)
        .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
        .items(0, Vec::new()),
    );

    v.push(
        proc(240_800_011, "ELEC AC ESS BUS ALTN", Level::Caution, sd_page::ELEC_AC, var("ELEC_AC_ESS_FED_BY_ALTN").on(), "FCOM p.4866; deep::electrical's own NORM/ALTN feeder-contactor readout")
            .confirm(1.0)
            .inhibit(&[4, 5, 6, 7, 8, 9, 10])
            .items(0, Vec::new()),
    );

    v.push(
        proc(240_800_013, "ELEC APU BAT FAULT", Level::Caution, sd_page::ELEC_DC, var("ELEC_APU_BAT_FAULT").on(), "FCOM p.4870; a third instance of the existing Battery/BatteryFaults model, dedicated to the APU start/standby battery")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10])
            .items(1, Vec::new()),
    );

    v.push(
        proc(240_800_014, "ELEC APU GEN A FAULT", Level::Caution, sd_page::ELEC_AC, var("ELEC_APU_GEN_1_FAULT").on(), "FCOM p.4872; deep::electrical's own per-channel APU generator verdict, channel A")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10])
            .items(1, Vec::new()),
    );
    v.push(
        proc(240_800_015, "ELEC APU GEN B FAULT", Level::Caution, sd_page::ELEC_AC, var("ELEC_APU_GEN_2_FAULT").on(), "FCOM p.4872; deep::electrical's own per-channel APU generator verdict, channel B")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10])
            .items(1, Vec::new()),
    );

    v.push(
        proc(240_800_019, "ELEC BUS TIE OFF", Level::Caution, sd_page::ELEC_AC, var("ELEC_BUS_TIE_OFF").on(), "FCOM p.4879: the BUS TIE pb-sw is abnormally set to OFF")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10])
            .items(0, Vec::new()),
    );

    v.push(
        proc(240_800_020, "ELEC C/B MONITORING FAULT", Level::Advisory, sd_page::CB, var("BREAKERS_CB_MONITORING_FAULT").on(), "FCOM p.4880: the C/B monitoring function is failed; STATUS-only (INOP SYS), no aural or master light")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );
    v.push(
        proc(
            240_800_054,
            "ELEC EMER C/B MONITORING FAULT",
            Level::Advisory,
            sd_page::CB,
            all(vec![var("BREAKERS_EMER_CB_MONITORING_FAULT").on(), var("ELEC_EMER_CONFIG_ACTIVE").on()]),
            "FCOM p.4928: the emergency part of the C/B monitoring function is failed, gated by ELEC_EMER_CONFIG_ACTIVE since the emergency path is only in circuit then",
        )
        .confirm(1.0)
        .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
        .items(0, Vec::new()),
    );

    v.push(
        proc(240_800_022, "ELEC CABIN L SUPPLY CENTER OVHT", Level::Caution, sd_page::ELEC_AC, var("ELEC_CABIN_L_SUPPLY_CENTER_OVHT").on(), "FCOM p.4882: the overheat detectors have detected an overheat in the left cabin supply center")
            .confirm(3.0)
            .inhibit(&[4, 5, 6, 7, 9, 10])
            .items(1, Vec::new()),
    );
    v.push(
        proc(240_800_023, "ELEC CABIN R SUPPLY CENTER OVHT", Level::Caution, sd_page::ELEC_AC, var("ELEC_CABIN_R_SUPPLY_CENTER_OVHT").on(), "FCOM p.4882: same as 240800022, right cabin supply center")
            .confirm(3.0)
            .inhibit(&[4, 5, 6, 7, 9, 10])
            .items(1, Vec::new()),
    );
    v.push(
        proc(240_800_024, "ELEC CABIN L SUPPLY CENTER OVHT DET FAULT", Level::Advisory, sd_page::ELEC_AC, var("ELEC_CABIN_L_SUPPLY_CENTER_OVHT_DET_FAULT").on(), "FCOM p.4883: the left cabin supply center's own overheat detector has failed; STATUS-only")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );
    v.push(
        proc(240_800_025, "ELEC CABIN R SUPPLY CENTER OVHT DET FAULT", Level::Advisory, sd_page::ELEC_AC, var("ELEC_CABIN_R_SUPPLY_CENTER_OVHT_DET_FAULT").on(), "FCOM p.4883: same as 240800024, right cabin supply center")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );

    for n in 1..=4u64 {
        let (id, title) = match n {
            1 => (240_800_032, "ELEC DRIVE 1 DISC FAULT"),
            2 => (240_800_033, "ELEC DRIVE 2 DISC FAULT"),
            3 => (240_800_034, "ELEC DRIVE 3 DISC FAULT"),
            _ => (240_800_035, "ELEC DRIVE 4 DISC FAULT"),
        };
        v.push(
            proc(id, title, Level::Caution, sd_page::ELEC_AC, var(&format!("ELEC_DRIVE_{n}_DISC_FAULT")).on(), "FCOM p.4917: the disconnection function of that engine generator's drive failed")
                .confirm(1.0)
                .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
                .items(0, Vec::new()),
        );
    }

    for n in 1..=4u64 {
        let (id, title) = match n {
            1 => (240_800_036, "ELEC DRIVE 1 DISCONNECTED"),
            2 => (240_800_037, "ELEC DRIVE 2 DISCONNECTED"),
            3 => (240_800_038, "ELEC DRIVE 3 DISCONNECTED"),
            _ => (240_800_039, "ELEC DRIVE 4 DISCONNECTED"),
        };
        v.push(
            proc(
                id,
                title,
                Level::Caution,
                sd_page::ELEC_AC,
                var(&format!("ELEC_GEN_{n}_FAULT")).on(),
                "FCOM p.4918: the generator disconnects from its engine while operating -- modelled as that generator's own already-sourced FAULT verdict sustained long enough (5 s) to represent escalation to a physical disconnect",
            )
            .confirm(5.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10])
            .items(1, Vec::new()),
        );
    }

    const OIL_LEVEL_LO_FRAC: f64 = 0.25;
    for n in 1..=4u64 {
        let (id, title) = match n {
            1 => (240_800_040, "ELEC DRIVE 1 OIL LEVEL LO"),
            2 => (240_800_041, "ELEC DRIVE 2 OIL LEVEL LO"),
            3 => (240_800_042, "ELEC DRIVE 3 OIL LEVEL LO"),
            _ => (240_800_043, "ELEC DRIVE 4 OIL LEVEL LO"),
        };
        v.push(
            proc(
                id,
                title,
                Level::Caution,
                sd_page::ELEC_AC,
                var(&format!("ELEC_DRIVE_{n}_OIL_LEVEL_FRAC")).lt(OIL_LEVEL_LO_FRAC),
                "FCOM p.4920: the oil level of the engine driven generator is low; threshold reused from apu::oil.rs's own OIL_LOW_LEVEL_TRIP_L/OIL_TANK_CAPACITY_L",
            )
            .confirm(5.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(1, Vec::new()),
        );
    }

    for n in 1..=4u64 {
        let (id, title) = match n {
            1 => (240_800_044, "ELEC DRIVE 1 OIL OVHT"),
            2 => (240_800_045, "ELEC DRIVE 2 OIL OVHT"),
            3 => (240_800_046, "ELEC DRIVE 3 OIL OVHT"),
            _ => (240_800_047, "ELEC DRIVE 4 OIL OVHT"),
        };
        v.push(
            proc(id, title, Level::Caution, sd_page::ELEC_AC, var(&format!("ELEC_DRIVE_{n}_OIL_TEMP_C")).gt(200.0), "FCOM p.4920: the oil temperature is higher than 200 degC")
                .confirm(5.0)
                .inhibit(&[1, 3, 4, 5, 6, 7, 9, 10, 12])
                .items(2, Vec::new()),
        );
    }

    for n in 1..=4u64 {
        let (id, title) = match n {
            1 => (240_800_048, "ELEC DRIVE 1 OIL PRESS LO"),
            2 => (240_800_049, "ELEC DRIVE 2 OIL PRESS LO"),
            3 => (240_800_050, "ELEC DRIVE 3 OIL PRESS LO"),
            _ => (240_800_051, "ELEC DRIVE 4 OIL PRESS LO"),
        };
        v.push(
            proc(id, title, Level::Caution, sd_page::ELEC_AC, var(&format!("ELEC_DRIVE_{n}_OIL_PRESSURE_PSI")).lt(35.0), "FCOM p.4948: the oil pressure is abnormally low (lower than 35 PSI)")
                .confirm(1.0)
                .inhibit(&[1, 4, 5, 6, 7, 9, 10, 12])
                .items(2, Vec::new()),
        );
    }

    v.push(
        proc(240_800_052, "ELEC ELEC NETWORK MANAGEMENT 1 FAULT", Level::Caution, sd_page::ELEC_AC, var("ELEC_ENMU_1_FAULT").on(), "FCOM p.4926: ENMU 1 is failed")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10])
            .items(1, Vec::new()),
    );
    v.push(
        proc(240_800_053, "ELEC ELEC NETWORK MANAGEMENT 2 FAULT", Level::Caution, sd_page::ELEC_AC, var("ELEC_ENMU_2_FAULT").on(), "FCOM p.4926: ENMU 2 is failed")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10])
            .items(1, Vec::new()),
    );

    for n in 1..=4u64 {
        let (id, title) = match n {
            1 => (240_800_056, "ELEC EXT PWR 1 FAULT"),
            2 => (240_800_057, "ELEC EXT PWR 2 FAULT"),
            3 => (240_800_058, "ELEC EXT PWR 3 FAULT"),
            _ => (240_800_059, "ELEC EXT PWR 4 FAULT"),
        };
        v.push(
            proc(
                id,
                title,
                Level::Advisory,
                sd_page::ELEC_AC,
                all(vec![var(&format!("ELEC_EXT_PWR_{n}_ON_LINE")).on(), var(&format!("ELEC_EXT_PWR_{n}_FAULT")).on()]),
                "FCOM p.4943: the external power unit, or its associated GGPCU, is failed; gated by that receptacle being on line",
            )
            .confirm(0.5)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10])
            .items(1, Vec::new()),
        );
    }

    v.push(
        proc(240_800_060, "ELEC  F/CTL ACTUATOR PWR SUPPLY FAULT", Level::Advisory, sd_page::FCTL, var("ELEC_FCTL_ACTUATOR_PWR_FAULT").on(), "the F/CTL EHA/EBHA power-conditioning unit's own health, independent of the AC/DC bus it draws from")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );

    for n in 1..=4u64 {
        let (id, title) = match n {
            1 => (240_800_065, "ELEC GEN 1 OFF"),
            2 => (240_800_066, "ELEC GEN 2 OFF"),
            3 => (240_800_067, "ELEC GEN 3 OFF"),
            _ => (240_800_068, "ELEC GEN 4 OFF"),
        };
        v.push(
            proc(id, title, Level::Caution, sd_page::ELEC_AC, var(&format!("ELEC_GEN_{n}_PB_ON")).off(), "FCOM p.4947: the GEN n pb-sw is abnormally set to OFF (deep::electrical's own publish of the same pushbutton state command_contactors reads)")
                .confirm(1.0)
                .inhibit(&[1, 3, 4, 5, 6, 7, 9, 10, 12])
                .items(0, Vec::new()),
        );
    }

    v.push(
        proc(240_800_069, "ELEC LOAD MANAGEMENT FAULT", Level::Caution, sd_page::ELEC_AC, var("ELEC_ELMU_FAULT").on(), "FCOM p.4949: the ELMU is failed")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(1, Vec::new()),
    );

    v.push(
        proc(240_800_070, "ELEC PRIMARY SUPPLY CENTER 1 FAULT", Level::Advisory, sd_page::ELEC_AC, var("ELEC_PSC_1_FAULT").on(), "FCOM p.4950: some loads are abnormally disconnected from PSC1")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );
    v.push(
        proc(240_800_071, "ELEC PRIMARY SUPPLY CENTER 2 FAULT", Level::Advisory, sd_page::ELEC_AC, var("ELEC_PSC_2_FAULT").on(), "FCOM p.4950: same as 240800070, PSC2")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(240_800_072, "ELEC RAT FAULT", Level::Advisory, sd_page::ELEC_AC, var("ELEC_RAT_FAULT").on(), "FCOM p.4952: the RAT's own standing health verdict (failed/stowed-not-locked/heater failed/electrical fault)")
            .confirm(2.0)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(240_800_073, "ELEC REMOTE C/B CTL ACTIVE", Level::Caution, sd_page::CB, var("BREAKERS_REMOTE_CTL_ACTIVE").on(), "FCOM p.4954: maintenance personnel left the REMOTE C/B CTL pb set to ON")
            .confirm(1.0)
            .inhibit(&[1, 3, 4, 5, 6, 7, 9, 10, 12])
            .items(1, Vec::new()),
    );

    v.push(
        proc(240_800_074, "ELEC SECONDARY SUPPLY CENTER 1 DEGRADED", Level::Advisory, sd_page::ELEC_AC, var("ELEC_SSC_1_DEGRADED").on(), "FCOM p.4955: communication is degraded between SSC1 and CPIOM E / other aircraft systems")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );
    v.push(
        proc(240_800_075, "ELEC SECONDARY SUPPLY CENTER 2 DEGRADED", Level::Advisory, sd_page::ELEC_AC, var("ELEC_SSC_2_DEGRADED").on(), "FCOM p.4955: same as 240800074, SSC2")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );
    v.push(
        proc(240_800_076, "ELEC SECONDARY SUPPLY CENTER 1 FAULT", Level::Advisory, sd_page::ELEC_AC, var("ELEC_SSC_1_FAULT").on(), "FCOM p.4956: some systems are no longer supplied by SSC1")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );
    v.push(
        proc(240_800_077, "ELEC SECONDARY SUPPLY CENTER 2 FAULT", Level::Advisory, sd_page::ELEC_AC, var("ELEC_SSC_2_FAULT").on(), "FCOM p.4956: same as 240800076, SSC2")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );
    v.push(
        proc(240_800_078, "ELEC SECONDARY SUPPLY CENTER 1 REDUND LOST", Level::Advisory, sd_page::ELEC_AC, var("ELEC_SSC_1_REDUND_LOST").on(), "FCOM p.4958: the redundancy of some system electrical supply from SSC1 is lost, no operational impact")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );
    v.push(
        proc(240_800_079, "ELEC SECONDARY SUPPLY CENTER 2 REDUND LOST", Level::Advisory, sd_page::ELEC_AC, var("ELEC_SSC_2_REDUND_LOST").on(), "FCOM p.4958: same as 240800078, SSC2")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(240_800_080, "ELEC STATIC INV FAULT", Level::Advisory, sd_page::ELEC_AC, var("ELEC_STATIC_INV_FAULT").on(), "FCOM p.4959; deep::electrical's own static-inverter degradation verdict (FBW_STATIC_INVERTER/24_004)")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10])
            .items(0, Vec::new()),
    );

    v.push(
        proc(240_800_084, "ELEC TR 1 MONITORING FAULT", Level::Advisory, sd_page::ELEC_DC, var("ELEC_TR_1_MONITORING_FAULT").on(), "FCOM p.4963: the monitoring function of TR 1 is failed")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );
    v.push(
        proc(240_800_085, "ELEC TR 2 MONITORING FAULT", Level::Advisory, sd_page::ELEC_DC, var("ELEC_TR_2_MONITORING_FAULT").on(), "FCOM p.4963: same as 240800084, TR 2")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );
    v.push(
        proc(240_800_086, "ELEC TR ESS MONITORING FAULT", Level::Advisory, sd_page::ELEC_DC, var("ELEC_TR_ESS_MONITORING_FAULT").on(), "FCOM p.4963: same as 240800084, TR ESS")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );
}
