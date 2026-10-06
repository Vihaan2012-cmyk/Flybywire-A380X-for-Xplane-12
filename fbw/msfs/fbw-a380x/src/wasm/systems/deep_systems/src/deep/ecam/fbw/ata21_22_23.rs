use super::{proc, sd_page, FbwProc};
use crate::deep::api::{all, any, var, Cond, Level};

fn network_alive() -> Cond {
    any(vec![
        var("ELEC_AC_1_BUS_IS_POWERED").on(),
        var("ELEC_AC_2_BUS_IS_POWERED").on(),
        var("ELEC_AC_3_BUS_IS_POWERED").on(),
        var("ELEC_AC_4_BUS_IS_POWERED").on(),
    ])
}

fn gated(c: Cond) -> Cond {
    all(vec![c, network_alive()])
}

fn binary(id: u64, title: &'static str, level: Level, sd: i32, var_name: &'static str, inhibit: &'static [u32], item_count: usize, note: &'static str) -> FbwProc {
    proc(id, title, level, sd, gated(var(var_name).gt(0.0)), note).inhibit(inhibit).items(item_count, Vec::new())
}

pub fn wire(v: &mut Vec<FbwProc>) {
    v.push(
        proc(
            211_800_013,
            "AIR PACK 1 OVHT",
            Level::Caution,
            sd_page::BLEED,
            gated(var("DEEP_PNEU_PACK_1_ACM_OUTLET_TEMPERATURE_C").gt(95.0)),
            "FCOM PRO-ABN-ECAM p.4653: \"the pack outlet temperature is above 95 C\" -- the real, cited trip, applied to pneumatic_ducts' own modelled ACM outlet temperature",
        )
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(1, Vec::new()),
    );
    v.push(
        proc(
            211_800_014,
            "AIR PACK 2 OVHT",
            Level::Caution,
            sd_page::BLEED,
            gated(var("DEEP_PNEU_PACK_2_ACM_OUTLET_TEMPERATURE_C").gt(95.0)),
            "mirror of 211800013 for pack 2",
        )
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(1, Vec::new()),
    );

    v.push(binary(
        211_800_015,
        "AIR PACK 1 REGUL FAULT",
        Level::Caution,
        sd_page::BLEED,
        "DEEP_PNEU_PACK_1_REGUL_FAULT",
        &[3, 4, 5, 6, 7, 9, 10],
        5,
        "pneumatic_ducts's own pack 1 regulation-train component (bypass valve/water extractor/ram-air-door interlock); distinct from the FDAC BothChannelsFault already claimed by the wired 211800009",
    ));
    v.push(binary(
        211_800_016,
        "AIR PACK 2 REGUL FAULT",
        Level::Caution,
        sd_page::BLEED,
        "DEEP_PNEU_PACK_2_REGUL_FAULT",
        &[3, 4, 5, 6, 7, 9, 10],
        5,
        "mirror of 211800015 for pack 2",
    ));

    for (id, title, var_name) in [
        (211_800_017u64, "AIR PACK 1 VLV 1 FAULT", "DEEP_PNEU_PACK_1_FCV_1_FAULT"),
        (211_800_018, "AIR PACK 1 VLV 2 FAULT", "DEEP_PNEU_PACK_1_FCV_2_FAULT"),
        (211_800_019, "AIR PACK 2 VLV 1 FAULT", "DEEP_PNEU_PACK_2_FCV_1_FAULT"),
        (211_800_020, "AIR PACK 2 VLV 2 FAULT", "DEEP_PNEU_PACK_2_FCV_2_FAULT"),
    ] {
        v.push(binary(id, title, Level::Advisory, sd_page::BLEED, var_name, &[3, 4, 5, 6, 7, 9, 10, 11], 0, "pneumatic_ducts's own real FCV component; see that area's doc on why this is not the FDAC-bridge the design sheet first planned"));
    }

    v.push(binary(
        211_800_022,
        "AIR PACK 1+2 REGUL REDUNDANCY FAULT",
        Level::Caution,
        sd_page::BLEED,
        "DEEP_PNEU_PACK_REGUL_REDUNDANCY_FAULT",
        &[3, 4, 5, 6, 7, 9, 10],
        0,
        "FCOM PRO-ABN-ECAM p.4662 (\"AIR PACK 1+2 REGUL REDUNDANCY LOST\"): at least one of several redundant components failed, on each pack together",
    ));

    v.push(
        proc(
            211_800_024,
            "COND BULK CARGO DUCT OVHT",
            Level::Caution,
            sd_page::COND,
            gated(var("DEEP_THERM_BULK_CARGO_DUCT_TEMPERATURE_C").gt(70.0)),
            "FCOM PRO-ABN-ECAM p.4669: \"there is a bulk cargo duct overheat when the air temperature inside the duct exceeds 70 C\"",
        )
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(1, Vec::new()),
    );
    v.push(
        proc(
            211_800_028,
            "COND DUCT OVHT",
            Level::Caution,
            sd_page::COND,
            gated(var("DEEP_THERM_TRIM_AIR_DUCT_TEMPERATURE_C").gt(70.0)),
            "FCOM PRO-ABN-ECAM p.4675: \"there is a duct overheat when the air temperature inside the applicable duct exceeds 70 C\"",
        )
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(0, Vec::new()),
    );

    v.push(binary(
        211_800_030,
        "COND FWD CARGO TEMP REGUL FAULT",
        Level::Advisory,
        sd_page::COND,
        "DEEP_THERM_FWD_CARGO_TRV_FAULT",
        &[3, 4, 5, 6, 7, 9, 10, 11],
        2,
        "thermal_zones's own FWD cargo zone trim-air valve component; distinct from the already-wired aircraft-wide HOT AIR valves 1/2 (211800032/033)",
    ));

    v.push(binary(
        211_800_034,
        "COND MIXER PRESS REGUL FAULT",
        Level::Advisory,
        sd_page::BLEED,
        "DEEP_PNEU_MIXER_PRESS_REGUL_FAULT",
        &[3, 4, 5, 6, 7, 9, 10, 11],
        0,
        "a380_systems acknowledges this exact real failure mode as an unimplemented TODO (full_digital_agu_controller.rs:287); pneumatic_ducts's own mixer-unit component fills it",
    ));

    v.push(binary(
        211_800_036,
        "COND PURSER TEMP SEL FAULT",
        Level::Caution,
        sd_page::COND,
        "DEEP_CABIN_PURSER_TEMP_SEL_FAULT",
        &[3, 4, 5, 6, 7, 9, 10, 11],
        0,
        "cabin's own purser temperature selector panel component (a real crew control FlyByWire already reads, cpiom_b.rs:861, purs_sel_temp_id)",
    ));

    v.push(binary(
        211_800_037,
        "COND RAM AIR 1 FAULT",
        Level::Advisory,
        sd_page::COND,
        "DEEP_PNEU_RAM_AIR_1_FAULT",
        &[2, 3, 4, 5, 6, 7, 8, 9, 10, 11],
        0,
        "pneumatic_ducts's own ram-air door 1 stuck failure, on the real, already-published door position (COND_PACK_1_RAM_AIR_DOOR_POSITION)",
    ));
    v.push(binary(
        211_800_038,
        "COND RAM AIR 2 FAULT",
        Level::Advisory,
        sd_page::COND,
        "DEEP_PNEU_RAM_AIR_2_FAULT",
        &[2, 3, 4, 5, 6, 7, 8, 9, 10, 11],
        0,
        "mirror of 211800037 for pack 2",
    ));

    v.push(
        proc(
            211_800_045,
            "AIR PACK REGUL DEGRADED",
            Level::Caution,
            sd_page::BLEED,
            gated(var("DEEP_PNEU_PACK_FLOW_INSUFFICIENT_FWD_CRG").on()),
            "pending FlyByWire write (E-AIR-FBW-WRITES.md): the real PackFlow::pack_flow_demand vs FWD_CARGO_ZONE_VOLUME_CUBIC_METER comparison already computed inside cpiom_b.rs, not yet published",
        )
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(2, Vec::new()),
    );

    v.push(
        proc(
            212_800_012,
            "COND PART SECONDARY CABIN FANS FAULT",
            Level::Advisory,
            sd_page::COND,
            gated(all(vec![var("DEEP_CABIN_SECONDARY_FANS_FAILED_COUNT").ge(1.0), var("DEEP_CABIN_SECONDARY_FANS_FAILED_COUNT").lt(4.0)])),
            "cabin's own model of the 4 real secondary cabin fans a380_systems already computes per fan (cabin_fan_has_failed, mod.rs:732-734) but does not publish per-fan; 1..3 of 4 failed",
        )
        .inhibit(&[2, 3, 4, 5, 6, 7, 9, 10, 11])
        .suppressed_by(&[212_800_013])
        .items(0, Vec::new()),
    );
    v.push(
        proc(
            212_800_013,
            "COND SECONDARY CABIN FANS FAULT",
            Level::Advisory,
            sd_page::COND,
            gated(var("DEEP_CABIN_SECONDARY_FANS_FAILED_COUNT").ge(4.0)),
            "all 4 secondary cabin fans failed",
        )
        .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
        .items(0, Vec::new()),
    );

    v.push(binary(
        212_800_019,
        "VENT COOLG SYS 1 OVHT",
        Level::Caution,
        sd_page::COND,
        "DEEP_AVNCS_COOLG_1_OVHT",
        &[3, 4, 5, 6, 7, 9, 10],
        3,
        "avionics_network's own supplemental cooling system 1 component; FCOM p.4731 confirms the real LRU (\"there is an overheat on the system 1(2) of the supplemental cooling system\")",
    ));
    v.push(binary(
        212_800_020,
        "VENT COOLG SYS 2 OVHT",
        Level::Caution,
        sd_page::COND,
        "DEEP_AVNCS_COOLG_2_OVHT",
        &[3, 4, 5, 6, 7, 9, 10],
        3,
        "mirror of 212800019 for system 2",
    ));

    v.push(binary(
        212_800_021,
        "VENT COOLG SYS PROT FAULT",
        Level::Advisory,
        sd_page::COND,
        "DEEP_AVNCS_COOLG_PROT_FAULT",
        &[3, 4, 5, 6, 7, 8, 9, 10, 11],
        0,
        "FCOM p.4733: \"the overheat detection system of the supplemental cooling system 1(2) is lost\" -- distinct from the overheat itself",
    ));

    v.push(binary(
        212_800_022,
        "VENT IFE BAY ISOL FAULT",
        Level::Advisory,
        sd_page::COND,
        "DEEP_CABIN_IFE_BAY_ISOL_FAULT",
        &[3, 4, 5, 6, 7, 8, 9, 10, 11],
        0,
        "cabin's own IFE bay isolation valve component",
    ));
    v.push(binary(
        212_800_023,
        "VENT IFE BAY VENT FAULT",
        Level::Caution,
        sd_page::COND,
        "DEEP_CABIN_IFE_BAY_VENT_FAULT",
        &[3, 4, 5, 6, 7, 9, 10],
        2,
        "cabin's own IFE bay extraction fan component",
    ));

    v.push(binary(
        212_800_024,
        "VENT LAV & GALLEYS EXTRACT FAULT",
        Level::Advisory,
        sd_page::COND,
        "DEEP_CABIN_LAV_GALLEY_EXTRACT_FAULT",
        &[3, 4, 5, 6, 7, 9, 10, 11],
        1,
        "cabin's own lav & galley extraction fan component, distinct from the FWD/BULK cargo extraction fans a380_systems' VCM models",
    ));

    v.push(binary(
        212_800_028,
        "VENT THS BAY VENT FAULT",
        Level::Advisory,
        sd_page::COND,
        "DEEP_THERM_THS_BAY_VENT_FAULT",
        &[3, 4, 5, 6, 7, 8, 9, 10, 11],
        0,
        "thermal_zones's own THS bay ventilation fan component -- a flag, not a temperature, since no THS bay thermal zone exists anywhere to invent a threshold from",
    ));

    v.push(
        proc(
            213_800_003,
            "CAB PRESS EXCESS NEGATIVE DIFF PRESS",
            Level::Caution,
            sd_page::PRESS,
            gated(var("DEEP_CPC_1_DIFF_PRESSURE_SENSED_PA").lt(-4964.0)),
            "FCOM PRO-ABN-ECAM p.4757: cabin differential pressure lower than -0.72 PSI (-4964 Pa, 0.72*6894.76); FlyByWire's own real, already-computed FWC signal (cpiom_b.rs:862, FWC_EXCESSIVE_NEGATIVE_DIFF_PRESSURE) via DEEP_CPC_1_DIFF_PRESSURE_SENSED_PA. Wired Caution, not Warning: FlyByWire's own title colour for this id is amber, and no_entry_is_louder_than_flybywires_own_title_colour keeps the quieter of the two (E-AIR-FCOM.json)",
        )
        .inhibit(&[1, 2, 3, 4, 5, 6, 7, 10, 11, 12])
        .items(3, Vec::new()),
    );

    v.push(
        proc(
            213_800_007,
            "CAB PRESS DIFF PRESS HI",
            Level::Caution,
            sd_page::PRESS,
            gated(all(vec![var("DEEP_CPC_1_DIFF_PRESSURE_SENSED_PA").gt(61_494.0), var("DEEP_CPC_1_DIFF_PRESSURE_SENSED_PA").lt(63_432.0)])),
            "FlyByWire's own cpiom_b.rs:859-860, FWC_DIFF_PRESS_HI_LOWER_LIMIT=8.92 PSI (61 494 Pa) / _UPPER_LIMIT=9.2 PSI (63 432 Pa); bounded below the already-wired 9.65 PSI EXCESS DIFF PRESS limit",
        )
        .inhibit(&[1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12])
        .items(3, Vec::new()),
    );

    v.push(binary(
        213_800_010,
        "CAB PRESS MAN CTL FAULT",
        Level::Caution,
        sd_page::PRESS,
        "DEEP_PNEU_PRESS_MAN_CTL_FAULT",
        &[3, 4, 5, 6, 7, 9, 10, 11],
        1,
        "pneumatic_ducts's own manual pressurisation control path component, distinct from the already-wired 213800005 AUTO CTL FAULT",
    ));

    v.push(binary(
        213_800_015,
        "CAB PRESS OUTFLW VLV CTL FAULT",
        Level::Caution,
        sd_page::PRESS,
        "DEEP_PNEU_OUTFLW_VLV_CTL_FAULT_ALL",
        &[3, 4, 5, 6, 7, 9, 10, 11],
        0,
        "all 4 OCSMs' own BothChannelsFault together (real FlyByWire per-channel discretes); no FCOM procedure exists for this exact combination, so phase/level follow 213800019-028's own family",
    ));

    v.push(
        proc(
            213_800_016,
            "CAB PRESS SENSORS FAULT",
            Level::Advisory,
            sd_page::PRESS,
            gated(any(vec![var("DEEP_CPC_1_SENSOR_FAULT").on(), var("DEEP_CPC_2_SENSOR_FAULT").on()])),
            "sensors's own CPC transducer stuck verdict (already-registered `stuck` field reaching its own documented 1.0 \"fully frozen\" ceiling); distinct from cpcs_has_fault (213800005/029-042) and from adirs_data_is_valid",
        )
        .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
        .items(0, Vec::new()),
    );

    v.push(binary(
        213_800_018,
        "COND CABIN AIR EXTRACT VLV FAULT",
        Level::Advisory,
        sd_page::PRESS,
        "DEEP_PNEU_CABIN_AIR_EXTRACT_VLV_FAULT",
        &[2, 3, 4, 5, 6, 7, 8, 9, 10, 11],
        0,
        "pneumatic_ducts's own cabin air extract valve component (the overhead CABIN AIR EXTRACT pushbutton's own valve)",
    ));

    v.push(
        proc(
            213_800_008,
            "CAB PRESS DIFF PRESS LO",
            Level::Caution,
            sd_page::PRESS,
            gated(all(vec![
                var("DEEP_CPC_1_DIFF_PRESSURE_SENSED_PA").lt(9_997.0),
                var("DEEP_CPC_1_CABIN_ALT_ABOVE_LANDING_ELEV_FT").gt(1_500.0),
                var("DEEP_ADIRS_VERTICAL_SPEED_FPM").lt(-500.0),
            ])),
            "FCOM PRO-ABN-ECAM p.4751: \"during descent, and if the aircraft altitude is at least 1500 ft above the landing field elevation, the cabin differential pressure is almost at 0 PSI\" -- cpiom_b.rs's own 1.45 PSI (9997 Pa) LOW_DIFFERENTIAL_PRESSURE_WARNING, FlyByWire's own real FMS landing elevation, and X-Plane's own real vertical speed",
        )
        .inhibit(&[1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12])
        .items(2, Vec::new()),
    );

    v.push(
        proc(
            211_800_056,
            "AIR ABNORM BLEED CONFIG",
            Level::Advisory,
            sd_page::BLEED,
            gated(all(vec![
                any(vec![
                    var("DEEP_PNEU_ENG_1_ISOLATION_OPEN").off(),
                    var("DEEP_PNEU_ENG_2_ISOLATION_OPEN").off(),
                    var("DEEP_PNEU_ENG_3_ISOLATION_OPEN").off(),
                    var("DEEP_PNEU_ENG_4_ISOLATION_OPEN").off(),
                ]),
                var("DEEP_PNEU_XBLEED_L_OPEN").off(),
                var("DEEP_PNEU_XBLEED_C_OPEN").off(),
                var("DEEP_PNEU_XBLEED_R_OPEN").off(),
            ])),
            "FCOM PRO-ABN-ECAM p.5661: an engine bleed isolated with no crossbleed path open, pneumatic_ducts' own already-published DEEP_PNEU_ENG_n_ISOLATION_OPEN / DEEP_PNEU_XBLEED_{L,C,R}_OPEN",
        )
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(15, Vec::new()),
    );

    v.push(
        proc(
            220_800_002,
            "AUTOLAND",
            Level::Caution,
            sd_page::STATUS,
            gated(all(vec![
                var("DEEP_AUTOFLT_DUAL_AP_ENGAGED").on(),
                var("DEEP_AUTOFLT_APPROACH_CAPABILITY_DOWNGRADED").off(),
                var("DEEP_AUTOFLT_RADIO_HEIGHT_FT").lt(200.0),
            ])),
            "dual AP engaged (real L:A32NX_AUTOPILOT_{1,2}_ACTIVE) with approach capability not downgraded, below 200 ft RA -- FCOM DSC-22-FG-90-10's own real 200 ft AUTOLAND-light gate",
        )
        .inhibit(&[4, 5, 6, 10])
        .items(0, Vec::new()),
    );

    v.push(binary(
        220_800_005,
        "AUTO FLT AFS CTL PNL FAULT",
        Level::Caution,
        sd_page::STATUS,
        "DEEP_AUTOFLT_FCU_FAULT",
        &[4, 5, 6, 10],
        3,
        "FCOM PRO-ABN-ECAM p.4785: \"the AFS control panel is failed\"; autoflight's own FCU component",
    ));

    v.push(binary(
        220_800_006,
        "AUTO FLT APPROACH CAPABILITY DOWNGRADED",
        Level::Caution,
        sd_page::STATUS,
        "DEEP_AUTOFLT_APPROACH_CAPABILITY_DOWNGRADED",
        &[1, 3, 4, 5, 6, 7, 10, 11, 12],
        0,
        "FCOM PRO-ABN-ECAM p.4789's own OR-condition, approximated here by any PRIM unhealthy (Truth::prim_healthy, real and already published) -- see autoflight::registry's own doc for why this port cannot see which PRIM-internal lane failed",
    ));

    v.push(
        proc(
            220_800_007,
            "AUTO FLT AFS CTL PNL+CAPT BKUP CTL FAULT",
            Level::Caution,
            sd_page::STATUS,
            gated(all(vec![var("DEEP_AUTOFLT_FCU_FAULT").gt(0.0), var("DEEP_AUTOFLT_CAPT_FCU_BKUP_FAULT").gt(0.0)])),
            "FCOM PRO-ABN-ECAM p.4791: \"the AFS control panel is failed, and the CAPT (F/O) AFS page on the FCU backup is failed\" -- the exact compound condition",
        )
        .inhibit(&[4, 5, 6, 10])
        .items(1, Vec::new()),
    );
    v.push(
        proc(
            220_800_008,
            "AUTO FLT AFS CTL PNL+F/O BKUP CTL FAULT",
            Level::Caution,
            sd_page::STATUS,
            gated(all(vec![var("DEEP_AUTOFLT_FCU_FAULT").gt(0.0), var("DEEP_AUTOFLT_FO_FCU_BKUP_FAULT").gt(0.0)])),
            "mirror of 220800007 for the F/O backup page, same FCOM procedure",
        )
        .inhibit(&[4, 5, 6, 10])
        .items(1, Vec::new()),
    );

    for (id, title, eng) in [
        (220_800_009u64, "AUTO FLT ENG 1 A/THR OFF", 1u16),
        (220_800_010, "AUTO FLT ENG 2 A/THR OFF", 2),
        (220_800_011, "AUTO FLT ENG 3 A/THR OFF", 3),
        (220_800_012, "AUTO FLT ENG 4 A/THR OFF", 4),
    ] {
        v.push(
            proc(
                id,
                title,
                Level::Caution,
                sd_page::STATUS,
                gated(all(vec![var("DEEP_AUTOFLT_ATHR_ARMED_OR_ACTIVE").on(), var(&format!("DEEP_AUTOFLT_ENG_{eng}_ATHR_FAULT")).on()])),
                "FCOM PRO-ABN-ECAM p.4792: \"the A/THR is armed or active, but failed on the indicated engine\" -- the aircraft-wide status is real (L:A32NX_AUTOTHRUST_STATUS); the per-engine fault is a pending FlyByWire write (E-AIR-FBW-WRITES.md)",
            )
            .inhibit(&[2, 3, 4, 5, 6, 10, 11])
            .items(1, Vec::new()),
        );
    }

    v.push(binary(
        220_800_014,
        "AUTO FLT TCAS MODE FAULT",
        Level::Caution,
        sd_page::STATUS,
        "DEEP_AUTOFLT_TCAS_MODE_FAULT",
        &[3, 4, 5, 6, 9, 10, 11],
        3,
        "FCOM PRO-ABN-ECAM p.4796: \"the AP/FD TCAS mode is failed\"; autoflight's own arbitration-logic component",
    ));

    v.push(binary(
        220_800_015,
        "CDS & AUTO FLT FCU SWITCHED OFF",
        Level::Caution,
        sd_page::STATUS,
        "DEEP_AUTOFLT_FCU_SWITCHED_OFF",
        &[2, 3, 4, 5, 6, 7, 9, 10, 11],
        0,
        "FCOM PRO-ABN-ECAM p.4797: \"the FCU is switched off: the EFIS CPs and the AFS CP are electrically shutoff\"",
    ));

    v.push(
        proc(
            230_800_001,
            "CAB COM CIDS 1+2+3 FAULT",
            Level::Caution,
            sd_page::STATUS,
            gated(all(vec![var("DEEP_COM_CIDS_1_FAULT").gt(0.0), var("DEEP_COM_CIDS_2_FAULT").gt(0.0), var("DEEP_COM_CIDS_3_FAULT").gt(0.0)])),
            "communications's own 3 CIDS computer components, all 3 ANDed",
        )
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(0, Vec::new()),
    );
    let cids_channel_names = ["DEEP_COM_CIDS_PA_UPPER_MAGNITUDE", "DEEP_COM_CIDS_PA_MAIN_MAGNITUDE", "DEEP_COM_CIDS_PA_LOWER_MAGNITUDE", "DEEP_COM_CIDS_INTERPHONE_MAGNITUDE"];
    v.push(
        proc(
            230_800_002,
            "CAB COM CIDS CABIN COM FAULT",
            Level::Caution,
            sd_page::STATUS,
            gated(any(cids_channel_names.iter().map(|n| var(n).ge(1.0)).collect())),
            "communications's own 4 CIDS channel components, any one fully faulted",
        )
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(4, Vec::new()),
    );
    v.push(
        proc(
            230_800_003,
            "CAB COM COM DEGRADED",
            Level::Advisory,
            sd_page::STATUS,
            gated(any(cids_channel_names.iter().map(|n| all(vec![var(n).gt(0.0), var(n).lt(1.0)])).collect())),
            "communications's own 4 CIDS channel components, any one degraded (not fully faulted) -- a lesser severity tier of 230800002 on the same components",
        )
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
        .items(2, Vec::new()),
    );

    for (id, title, var_name) in [
        (230_800_004u64, "COM CAPT PTT STUCK", "DEEP_COM_CAPT_PTT_STUCK"),
        (230_800_005, "COM F/O PTT STUCK", "DEEP_COM_FO_PTT_STUCK"),
        (230_800_006, "COM THIRD OCCUPANT PTT STUCK", "DEEP_COM_THIRD_PTT_STUCK"),
    ] {
        v.push(
            proc(id, title, Level::Caution, sd_page::STATUS, gated(var(var_name).gt(0.0)), "FCOM PRO-ABN-ECAM p.4816: \"stuck in the transmit position for more than 40 s, and no transmission key is selected\"")
                .confirm(40.0)
                .inhibit(&[3, 4, 5, 6, 7, 9, 10])
                .items(0, Vec::new()),
        );
    }

    v.push(binary(
        230_800_007,
        "COM DATALINK FAULT",
        Level::Caution,
        sd_page::STATUS,
        "DEEP_COM_DATALINK_FAULT",
        &[3, 4, 5, 6, 7, 9, 10],
        2,
        "communications's own ATSU/datalink router component",
    ));

    v.push(binary(230_800_008, "COM HF 1 DATALINK FAULT", Level::Advisory, sd_page::STATUS, "DEEP_COM_HF1_DATALINK_FAULT", &[3, 4, 5, 6, 7, 9, 10], 0, "communications's own HF 1 transceiver component"));
    v.push(binary(230_800_009, "COM HF 2 DATALINK FAULT", Level::Advisory, sd_page::STATUS, "DEEP_COM_HF2_DATALINK_FAULT", &[3, 4, 5, 6, 7, 9, 10], 0, "communications's own HF 2 transceiver component"));
    v.push(binary(230_800_010, "COM HF 1 EMITTING", Level::Caution, sd_page::STATUS, "DEEP_COM_HF1_EMITTING", &[3, 4, 5, 6, 7, 9, 10], 0, "communications's own HF 1 transceiver, stuck-emitting sub-fault"));
    v.push(binary(230_800_011, "COM HF 2 EMITTING", Level::Caution, sd_page::STATUS, "DEEP_COM_HF2_EMITTING", &[3, 4, 5, 6, 7, 9, 10], 0, "communications's own HF 2 transceiver, stuck-emitting sub-fault"));

    v.push(binary(230_800_019, "COM SATCOM DATALINK FAULT", Level::Advisory, sd_page::STATUS, "DEEP_COM_SATCOM_DATALINK_FAULT", &[3, 4, 5, 6, 7, 9, 10], 0, "communications's own SATCOM transceiver, datalink sub-fault"));
    v.push(binary(230_800_020, "COM SATCOM FAULT", Level::Advisory, sd_page::STATUS, "DEEP_COM_SATCOM_FAULT", &[3, 4, 5, 6, 7, 9, 10, 11], 0, "communications's own SATCOM transceiver, main-unit sub-fault"));
    v.push(binary(230_800_021, "COM SATCOM VOICE FAULT", Level::Advisory, sd_page::STATUS, "DEEP_COM_SATCOM_VOICE_FAULT", &[3, 4, 5, 6, 7, 9, 10, 11], 0, "communications's own SATCOM transceiver, voice-channel sub-fault"));

    for (id, title, var_name) in [
        (230_800_022u64, "COM VHF 1 EMITTING", "DEEP_COM_VHF1_EMITTING"),
        (230_800_023, "COM VHF 2 EMITTING", "DEEP_COM_VHF2_EMITTING"),
        (230_800_024, "COM VHF 3 EMITTING", "DEEP_COM_VHF3_EMITTING"),
    ] {
        v.push(
            proc(id, title, Level::Caution, sd_page::STATUS, gated(var(var_name).gt(0.0)), "communications's own VHF stuck-emitting component, extending (in doc comment only) radios.rs's real VHF tuning model; 60 s RMP TX KEY DESELECT is the catalogue's own figure")
                .confirm(60.0)
                .inhibit(&[3, 4, 5, 6, 7, 9, 10])
                .items(1, Vec::new()),
        );
    }
    v.push(binary(230_800_025, "COM VHF 3 DATALINK FAULT", Level::Advisory, sd_page::STATUS, "DEEP_COM_VHF3_DATALINK_FAULT", &[3, 4, 5, 6, 7, 9, 10], 0, "communications's own VHF 3 component, datalink sub-fault (VHF 3 is the dedicated ACARS-over-VHF datalink radio on the real aircraft)"));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wires_sixty_three_of_the_seventy_three() {
        let mut v = Vec::new();
        wire(&mut v);
        assert_eq!(v.len(), 63, "see this module's own doc comment for the 10 deliberately left unwired");
    }

    #[test]
    fn the_pending_write_triggers_light_up_once_their_variable_carries_a_real_value() {
        let mut v = Vec::new();
        wire(&mut v);
        let find = |id: u64| v.iter().find(|p| p.id == id).unwrap().clone();

        let network_alive_map = |extra: &[(&str, f64)]| -> std::collections::BTreeMap<String, f64> {
            let mut m = std::collections::BTreeMap::new();
            m.insert("ELEC_AC_1_BUS_IS_POWERED".to_owned(), 1.0);
            for (k, val) in extra {
                m.insert((*k).to_owned(), *val);
            }
            m
        };

        let p = find(211_800_045);
        assert!(!p.trigger.eval(&|n| *network_alive_map(&[]).get(n).unwrap_or(&0.0)));
        assert!(p.trigger.eval(&|n| *network_alive_map(&[("DEEP_PNEU_PACK_FLOW_INSUFFICIENT_FWD_CRG", 1.0)]).get(n).unwrap_or(&0.0)));

        let p = find(220_800_009);
        let armed_only = network_alive_map(&[("DEEP_AUTOFLT_ATHR_ARMED_OR_ACTIVE", 1.0)]);
        assert!(!p.trigger.eval(&|n| *armed_only.get(n).unwrap_or(&0.0)), "armed alone, no per-engine fault yet, must stay quiet");
        let armed_and_faulted = network_alive_map(&[("DEEP_AUTOFLT_ATHR_ARMED_OR_ACTIVE", 1.0), ("DEEP_AUTOFLT_ENG_1_ATHR_FAULT", 1.0)]);
        assert!(p.trigger.eval(&|n| *armed_and_faulted.get(n).unwrap_or(&0.0)), "once FlyByWire writes the per-engine fault, this must fire");
        let p2 = find(220_800_010);
        assert!(!p2.trigger.eval(&|n| *armed_and_faulted.get(n).unwrap_or(&0.0)));
    }
}
