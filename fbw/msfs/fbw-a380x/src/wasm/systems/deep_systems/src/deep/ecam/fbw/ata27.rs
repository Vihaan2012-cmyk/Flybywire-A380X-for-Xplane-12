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

const FCDC_SIBLING_CONFIRM_S: f64 = 0.6;

pub fn wire(v: &mut Vec<FbwProc>) {
    v.push(
        proc(
            271_800_001,
            "CONFIG L SIDESTICK FAULT (BY TAKE-OVER)",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_L_SIDESTICK_DISABLED_BY_TAKEOVER").on(), network_alive()]),
            "the F.O.'s priority-takeover pushbutton has locked the captain's stick out -- FCOM PRO-ABN-ECAM p.5015; A380PrimComputerFctl.cpp:1770-1790 via prim.rs's A32NX_PRIM_1_LEFT_SIDESTICK_DISABLED",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[5, 6, 7, 8, 9, 10, 12])
        .items(0, Vec::new()),
    );
    v.push(
        proc(
            271_800_002,
            "CONFIG R SIDESTICK FAULT (BY TAKE-OVER)",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_R_SIDESTICK_DISABLED_BY_TAKEOVER").on(), network_alive()]),
            "the captain's priority-takeover pushbutton has locked the F.O.'s stick out -- FCOM PRO-ABN-ECAM p.5015; A380PrimComputerFctl.cpp:1770-1790 via prim.rs's A32NX_PRIM_1_RIGHT_SIDESTICK_DISABLED",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[5, 6, 7, 8, 9, 10, 12])
        .items(0, Vec::new()),
    );

    v.push(
        proc(
            271_800_025,
            "F/CTL L SIDESTICK FAULT",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_L_SIDESTICK_PITCH_FAULT").on(), var("FCTL_L_SIDESTICK_ROLL_FAULT").on(), network_alive()]),
            "both channels of the captain's sidestick pitch AND roll transducers invalid -- deep::flight_controls's new sensors::DualTransducer on the real captain's stick axis (prim.rs:1124-1125); FCOM PRO-ABN-ECAM p.5043",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[])
        .items(0, Vec::new()),
    );
    v.push(
        proc(
            271_800_027,
            "F/CTL L SIDESTICK SENSOR FAULT",
            Level::Caution,
            sd_page::FCTL,
            all(vec![any(vec![var("FCTL_L_SIDESTICK_PITCH_SENSOR_FAULT").on(), var("FCTL_L_SIDESTICK_ROLL_SENSOR_FAULT").on()]), network_alive()]),
            "the captain's sidestick pitch or roll channels disagree by more than TRANSDUCER_DISAGREE_RAD for TRANSDUCER_DISAGREE_TIMER_S (live.rs:156-157) while at least one channel still reads -- narrower than 271800025's full loss; FCOM PRO-ABN-ECAM p.5077 (correction, coordinator follow-up 2026-09-27: the previous pass wrongly cited p.5043 -- the generic SIDESTICK SENSOR FAULT procedure this id and 271800028 share is on p.5077, which does have a decoded phase bar, unlike p.5043)",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
        .items(0, Vec::new()),
    );

    v.push(
        proc(
            271_800_050,
            "F/CTL RUDDER PEDAL FAULT",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_RUDDER_PEDAL_FAULT").on(), network_alive()]),
            "both channels of the rudder pedal position transducer invalid -- deep::flight_controls's new sensors::DualTransducer on the real rudder pedal axis; FCOM PRO-ABN-ECAM p.5062",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[4, 5, 6, 7])
        .items(0, Vec::new()),
    );
    v.push(
        proc(
            271_800_052,
            "F/CTL RUDDER PEDAL SENSOR FAULT",
            Level::Advisory,
            sd_page::FCTL,
            all(vec![var("FCTL_RUDDER_PEDAL_SENSOR_FAULT").on(), network_alive()]),
            "the rudder pedal transducer's two channels disagree while at least one still reads -- FCOM PRO-ABN-ECAM p.5067",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
        .items(0, Vec::new()),
    );

    v.push(
        proc(
            271_800_045,
            "F/CTL PRIM VERSIONS DISAGREE",
            Level::Advisory,
            sd_page::FCTL,
            all(vec![var("FCTL_PRIM_VERSIONS_DISAGREE").on(), network_alive()]),
            "the three PRIMs' configured identity tags do not all agree -- new deep::flight_controls component 27_fctl.prim_pin_prog (E-FCTL-DESIGN.md section 3.3); FCOM PRO-ABN-ECAM p.5058",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12])
        .items(0, Vec::new()),
    );
    v.push(
        proc(
            271_800_046,
            "F/CTL SEC VERSIONS DISAGREE",
            Level::Advisory,
            sd_page::FCTL,
            all(vec![var("FCTL_SEC_VERSIONS_DISAGREE").on(), network_alive()]),
            "the three SECs' configured identity tags do not all agree -- new deep::flight_controls component 27_fctl.sec_pin_prog; FCOM PRO-ABN-ECAM p.5058",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12])
        .items(0, Vec::new()),
    );
    v.push(
        proc(
            271_800_047,
            "F/CTL PRIMs PIN PROG DISAGREE",
            Level::Advisory,
            sd_page::FCTL,
            all(vec![var("FCTL_PRIM_PIN_PROG_DISAGREE").on(), network_alive()]),
            "the three PRIMs' hardware pin-programming does not all agree -- same underlying check as 271800045, published under FlyByWire's second title; FCOM PRO-ABN-ECAM p.5059",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12])
        .items(0, Vec::new()),
    );

    v.push(
        proc(
            271_800_018,
            "F/CTL TWO GYROMETERs FAULT",
            Level::Caution,
            sd_page::FCTL,
            all(vec![
                any(vec![var("FCTL_RATE_GYRO_PITCH_FAULT").on(), var("FCTL_RATE_GYRO_ROLL_FAULT").on(), var("FCTL_RATE_GYRO_YAW_FAULT").on()]),
                network_alive(),
            ]),
            "both channels of one axis's rate-gyro pair invalid -- real sensed body rate from physics::adirs's own strapdown-IRS model, not an invented input; FCOM PRO-ABN-ECAM p.5092",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
        .items(0, Vec::new()),
    );

    v.push(
        proc(
            271_800_026,
            "F/CTL R SIDESTICK FAULT",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_R_SIDESTICK_PITCH_FAULT").on(), var("FCTL_R_SIDESTICK_ROLL_FAULT").on(), network_alive()]),
            "both channels of the F.O.'s sidestick pitch AND roll transducers invalid -- modelled at the same fixed neutral this port's cockpit holds that stick at; FCOM PRO-ABN-ECAM p.5043",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[])
        .items(0, Vec::new()),
    );
    v.push(
        proc(
            271_800_028,
            "F/CTL R SIDESTICK SENSOR FAULT",
            Level::Caution,
            sd_page::FCTL,
            all(vec![any(vec![var("FCTL_R_SIDESTICK_PITCH_SENSOR_FAULT").on(), var("FCTL_R_SIDESTICK_ROLL_SENSOR_FAULT").on()]), network_alive()]),
            "the F.O.'s sidestick pitch or roll channels disagree while at least one still reads -- a channel drifting off a shared neutral still trips the disagreement monitor; FCOM PRO-ABN-ECAM p.5077",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
        .items(0, Vec::new()),
    );

    v.push(
        proc(
            271_800_029,
            "F/CTL LOAD ALLEVIATION FAULT",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_LOAD_ALLEVIATION_FAULT").on(), network_alive()]),
            "two of the three accelerometers used by the load alleviation function have failed in one wing -- FCOM PRO-ABN-ECAM p.5040's own literal 2-of-3 vote, new deep::flight_controls component 27_fctl.load_alleviation",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10])
        .items(0, Vec::new()),
    );

    for n in 1..=3u64 {
        v.push(
            proc(
                271_800_032 + n,
                match n {
                    1 => "F/CTL PRIM 1 ELEVATOR ACTUATOR FAULT",
                    2 => "F/CTL PRIM 2 ELEVATOR ACTUATOR FAULT",
                    _ => "F/CTL PRIM 3 ELEVATOR ACTUATOR FAULT",
                },
                Level::Caution,
                sd_page::FCTL,
                all(vec![var(&format!("FCTL_PRIM_{n}_ELEVATOR_CHANNEL_FAULT")).on(), network_alive()]),
                "that PRIM's own elevator command channel has failed while the PRIM itself stays healthy -- new deep::flight_controls per-PRIM channel component; FCOM PRO-ABN-ECAM p.5055",
            )
            .confirm(FCDC_SIBLING_CONFIRM_S)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
        );
    }
    for n in 1..=3u64 {
        v.push(
            proc(
                271_800_038 + n,
                match n {
                    1 => "F/CTL PRIM 1 RUDDER ACTUATOR FAULT",
                    2 => "F/CTL PRIM 2 RUDDER ACTUATOR FAULT",
                    _ => "F/CTL PRIM 3 RUDDER ACTUATOR FAULT",
                },
                Level::Caution,
                sd_page::FCTL,
                all(vec![var(&format!("FCTL_PRIM_{n}_RUDDER_CHANNEL_FAULT")).on(), network_alive()]),
                "that PRIM's own rudder command channel has failed while the PRIM itself stays healthy -- new deep::flight_controls per-PRIM channel component; FCOM PRO-ABN-ECAM p.5056",
            )
            .confirm(FCDC_SIBLING_CONFIRM_S)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
        );
    }
    for n in 1..=3u64 {
        v.push(
            proc(
                271_800_041 + n,
                match n {
                    1 => "F/CTL PRIM 1 SIDESTICK SENSOR FAULT",
                    2 => "F/CTL PRIM 2 SIDESTICK SENSOR FAULT",
                    _ => "F/CTL PRIM 3 SIDESTICK SENSOR FAULT",
                },
                Level::Caution,
                sd_page::FCTL,
                all(vec![var(&format!("FCTL_PRIM_{n}_SIDESTICK_MONITOR_FAULT")).on(), network_alive()]),
                "that PRIM's own sidestick-sensor monitoring circuit has failed while the PRIM itself stays healthy -- new deep::flight_controls per-PRIM channel component; FCOM PRO-ABN-ECAM p.5057",
            )
            .confirm(FCDC_SIBLING_CONFIRM_S)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
        );
    }

    v.push(
        proc(
            272_800_014,
            "F/CTL FLAPS LEVER SYS 1 FAULT",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_FLAPS_LEVER_SYS_1_FAULT").on(), network_alive()]),
            "communication between the flaps lever and SFCC 1 is lost -- FCOM PRO-ABN-ECAM p.5112's own literal wording, new deep::flight_controls component 27_fctl.flap_lever_csu",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(0, Vec::new()),
    );
    v.push(
        proc(
            272_800_015,
            "F/CTL FLAPS LEVER SYS 2 FAULT",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_FLAPS_LEVER_SYS_2_FAULT").on(), network_alive()]),
            "communication between the flaps lever and SFCC 2 is lost -- FCOM PRO-ABN-ECAM p.5112's own literal wording, new deep::flight_controls component 27_fctl.flap_lever_csu",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(0, Vec::new()),
    );

    v.push(
        proc(
            272_800_016,
            "F/CTL FLAPS LOCKED",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_FLAP_WINGTIP_BRAKE_ON").on(), network_alive()]),
            "the flap wingtip brake has engaged to stop an asymmetry/runaway/overspeed condition -- FCOM PRO-ABN-ECAM p.5114; already-published high_lift.rs signal, no new component",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[4, 5, 6, 7, 10])
        .items(0, Vec::new()),
    );
    v.push(
        proc(
            272_800_024,
            "F/CTL SLATS LOCKED",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_SLAT_WINGTIP_BRAKE_ON").on(), network_alive()]),
            "the slat wingtip brake has engaged to stop an asymmetry/runaway/overspeed condition -- FCOM PRO-ABN-ECAM p.5134; already-published high_lift.rs signal, no new component",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[4, 5, 6, 7, 10])
        .items(0, Vec::new()),
    );

    for (id, varname) in [
        (271_800_019u64, "FCTL_AIL_L3_FAULT"),
        (271_800_020u64, "FCTL_AIL_R3_FAULT"),
        (271_800_021u64, "FCTL_AIL_L2_FAULT"),
        (271_800_022u64, "FCTL_AIL_R2_FAULT"),
        (271_800_023u64, "FCTL_AIL_L1_FAULT"),
        (271_800_024u64, "FCTL_AIL_R1_FAULT"),
    ] {
        v.push(
            proc(
                id,
                match id {
                    271_800_019 => "F/CTL L INR AILERON FAULT",
                    271_800_020 => "F/CTL R INR AILERON FAULT",
                    271_800_021 => "F/CTL L MID AILERON FAULT",
                    271_800_022 => "F/CTL R MID AILERON FAULT",
                    271_800_023 => "F/CTL L OUTR AILERON FAULT",
                    _ => "F/CTL R OUTR AILERON FAULT",
                },
                Level::Caution,
                sd_page::FCTL,
                all(vec![var(varname).gt(0.5), network_alive()]),
                "FCOM PRO-ABN-ECAM p.5041: that aileron's own drive line has failed -- deep::flight_controls per-surface component",
            )
            .confirm(FCDC_SIBLING_CONFIRM_S)
            .inhibit(&[4, 5, 6, 7, 10])
            .items(0, Vec::new()),
        );
    }

    v.push(
        proc(
            271_800_011,
            "F/CTL ELEVATOR ACTUATOR FAULT",
            Level::Caution,
            sd_page::FCTL,
            all(vec![
                any(vec![
                    var("FCTL_ELEV_L_INBD_FAULT").gt(0.5),
                    var("FCTL_ELEV_L_OUTBD_FAULT").gt(0.5),
                    var("FCTL_ELEV_R_INBD_FAULT").gt(0.5),
                    var("FCTL_ELEV_R_OUTBD_FAULT").gt(0.5),
                ]),
                network_alive(),
            ]),
            "FCOM PRO-ABN-ECAM p.5031: one actuator of one elevator failed -- deep::flight_controls per-surface component",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
        .items(0, Vec::new()),
    );

    v.push(
        proc(
            271_800_030,
            "F/CTL PART SPLRs FAULT",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_SPLR_FAILED_COUNT").ge(1.0), var("FCTL_SPLR_FAILED_COUNT").lt(10.0), network_alive()]),
            "FCOM PRO-ABN-ECAM p.5044: between one and eight spoilers failed; nine, which the FCOM leaves to neither title, stays here",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(0, Vec::new()),
    );

    v.push(
        proc(
            271_800_031,
            "F/CTL MOST SPLRs FAULT",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_SPLR_FAILED_COUNT").ge(10.0), network_alive()]),
            "FCOM PRO-ABN-ECAM p.5044: more than nine spoilers failed",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(0, Vec::new()),
    );

    v.push(
        proc(
            271_800_068,
            "F/CTL STABILIZER FAULT",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_THS_FAULT").gt(0.5), network_alive()]),
            "FCOM PRO-ABN-ECAM p.5089: the stabilizer failed, pitch trim locked -- deep::flight_controls 27_fctl.ths, moved from the registry's own invented FCTL_THS_RUNAWAY",
        )
        .confirm(0.5)
        .inhibit(&[4, 5, 6])
        .items(0, Vec::new()),
    );
}

#[cfg(test)]
mod tests {
    use super::wire;
    use crate::deep::registry;

    #[test]
    fn surface_alerts_stay_with_the_flybywire_procedures() {
        let reg = registry();
        let keys: Vec<&str> = reg.alerts.iter().map(|a| a.key.as_str()).collect();
        for gone in ["FCTL_AIL_FAULT", "FCTL_ELEV_FAULT", "FCTL_RUD_FAULT", "FCTL_SPLR_FAULT", "FCTL_FLAP_FAULT", "FCTL_SLAT_FAULT", "FCTL_RUD_TRIM_FAULT", "FCTL_THS_RUNAWAY", "FCTL_DROOP_NOSE_FAULT", "FCTL_GND_SPLR_FAULT"] {
            assert!(!keys.contains(&gone), "{gone} must stay replaced by the FlyByWire procedure for the same failure, not reappear in the registry");
        }
    }

    #[test]
    fn wire_pushes_exactly_the_thirty_six_wired_ids() {
        let mut v = Vec::new();
        wire(&mut v);
        let mut ids: Vec<u64> = v.iter().map(|p| p.id).collect();
        ids.sort_unstable();
        assert_eq!(
            ids,
            vec![
                271_800_001,
                271_800_002,
                271_800_011,
                271_800_018,
                271_800_019,
                271_800_020,
                271_800_021,
                271_800_022,
                271_800_023,
                271_800_024,
                271_800_025,
                271_800_026,
                271_800_027,
                271_800_028,
                271_800_029,
                271_800_030,
                271_800_031,
                271_800_033,
                271_800_034,
                271_800_035,
                271_800_039,
                271_800_040,
                271_800_041,
                271_800_042,
                271_800_043,
                271_800_044,
                271_800_045,
                271_800_046,
                271_800_047,
                271_800_050,
                271_800_052,
                271_800_068,
                272_800_014,
                272_800_015,
                272_800_016,
                272_800_024,
            ],
            "ata27::wire's id set drifted from this module's own addendum -- update both together"
        );
    }

    #[test]
    fn every_phase_2_entry_cites_an_fcom_page() {
        let mut v = Vec::new();
        wire(&mut v);
        for p in &v {
            assert!(p.note.contains("FCOM PRO-ABN-ECAM p."), "{}: note does not cite an FCOM page: {:?}", p.id, p.note);
        }
    }
}
