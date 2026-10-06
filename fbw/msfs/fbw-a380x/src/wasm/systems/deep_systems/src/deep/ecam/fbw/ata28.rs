use super::{phase, proc, sd_page, FbwProc};
use crate::deep::api::{all, any, not, var, Cond, Level};

fn network_alive() -> Cond {
    any(vec![var("ELEC_AC_1_BUS_IS_POWERED").on(), var("ELEC_AC_2_BUS_IS_POWERED").on(), var("ELEC_AC_3_BUS_IS_POWERED").on(), var("ELEC_AC_4_BUS_IS_POWERED").on()])
}

fn gated(c: Cond) -> Cond {
    all(vec![c, network_alive()])
}

const PUMP_FAILED_FRACTION: f64 = 0.5;

fn pump_failed(n: u32) -> Cond {
    var(&format!("FUEL_TRIM_PUMP_DEGRADATION:{n}")).ge(PUMP_FAILED_FRACTION)
}

fn on(name: &str) -> Cond {
    var(name).on()
}

fn title(s: String) -> &'static str {
    Box::leak(s.into_boxed_str())
}

fn one(id: u64) -> &'static [u64] {
    Box::leak(vec![id].into_boxed_slice())
}

pub fn wire(v: &mut Vec<FbwProc>) {
    v.push(
        proc(281_800_086, "FUEL TEMP LO", Level::Caution, sd_page::FUEL, gated(on("FUEL_FOB_LO_TEMP")), "FCOM PRO-ABN-ECAM p.5258")
            .confirm(3.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11, 12])
            .items(0, Vec::new()),
    );
    v.push(
        proc(281_800_054, "FUEL JETTISON FAULT", Level::Caution, sd_page::FUEL, gated(any(vec![on("FUEL_JETTISON_L_VALVE_FAULT"), on("FUEL_JETTISON_R_VALVE_FAULT")])), "FCOM PRO-ABN-ECAM p.5203")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(0, Vec::new()),
    );
    v.push(
        proc(281_800_089, "FUEL TRIM TK L PMP FAULT", Level::Caution, sd_page::FUEL, all(vec![pump_failed(1), var("FUEL_TRIM_PUMP_DEGRADATION:2").lt(PUMP_FAILED_FRACTION)]), "the left trim pump has lost at least half its delivery while the right one has not")
            .confirm(5.0)
            .inhibit(phase::TAKEOFF_AND_LANDING)
            .suppressed_by(&[281_800_091]),
    );
    v.push(
        proc(281_800_090, "FUEL TRIM TK R PMP FAULT", Level::Caution, sd_page::FUEL, all(vec![pump_failed(2), var("FUEL_TRIM_PUMP_DEGRADATION:1").lt(PUMP_FAILED_FRACTION)]), "the right trim pump has lost at least half its delivery while the left one has not")
            .confirm(5.0)
            .inhibit(phase::TAKEOFF_AND_LANDING)
            .suppressed_by(&[281_800_091]),
    );
    v.push(
        proc(281_800_091, "FUEL TRIM TK L+R PMPs FAULT", Level::Caution, sd_page::FUEL, all(vec![pump_failed(1), pump_failed(2)]), "both trim pumps have lost at least half their delivery, which is when the trim transfer itself stops")
            .confirm(5.0)
            .inhibit(phase::TAKEOFF_AND_LANDING),
    );

    v.push(
        proc(
            281_800_052,
            "FUEL JETTISON",
            Level::Advisory,
            sd_page::FUEL,
            any(vec![var("FUEL_JETTISON_FLOW_KG_S:1").gt(0.0), var("FUEL_JETTISON_FLOW_KG_S:2").gt(0.0)]),
            "fuel jettison is in progress -- FUEL_JETTISON_FLOW_KG_S:1/:2 (live.rs) are the real nozzle flow, driven by the real valve position and nozzle CdA; a status memo, not a fault, so no cold-and-dark gate (a jettison can only ever be in progress with the aircraft powered and flying)",
        )
        .confirm(0.0)
        .inhibit(phase::NONE),
    );

    v.push(
        proc(
            281_800_003,
            "FUEL APU FEED FAULT",
            Level::Caution,
            sd_page::FUEL,
            gated(any(vec![on("FUEL_APU_FEED_PUMP_FAULT"), all(vec![on("FUEL_APU_FEED_VALVE_FAULT"), not(on("FUEL_APU_FEED_VALVE_NOT_CLOSED"))])])),
            "FCOM PRO-ABN-ECAM p.5146: the APU feed pump has degraded, or the isolation/LP valve is abnormally closed (a mismatch that is not the abnormally-open case 281800004 already covers)",
        )
        .confirm(5.0)
        .inhibit(&[3, 4, 5, 6, 7, 9, 10]),
    );
    v.push(
        proc(281_800_004, "FUEL APU FEED VLV NOT CLOSED", Level::Caution, sd_page::FUEL, gated(on("FUEL_APU_FEED_VALVE_NOT_CLOSED")), "FCOM PRO-ABN-ECAM p.5147: the APU isolation/LP valve is abnormally open (has not returned closed once the APU stopped drawing fuel)")
            .confirm(5.0)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11]),
    );

    for n in 1..=4u64 {
        v.push(
            proc(281_800_017 + n, title(format!("FUEL ENG {n} LP VLV FAULT")), Level::Caution, sd_page::FUEL, gated(on(&format!("FUEL_ENG_LP_VALVE_FAULT:{n}"))), "FCOM PRO-ABN-ECAM p.5162: that engine's LP (fire) shutoff valve is abnormally closed on ground during start, or abnormally open on ground during shutdown -- a commanded-vs-actual-position mismatch")
                .confirm(5.0)
                .inhibit(&[4, 5, 6, 7, 8, 9, 10]),
        );
    }

    for n in 1..=4u64 {
        let main_id = 281_800_030 + n;
        let stby_id = 281_800_034 + n;
        let combined_id = 281_800_026 + n;
        v.push(
            proc(main_id, title(format!("FUEL FEED TK {n} MAIN PMP FAULT")), Level::Caution, sd_page::FUEL, gated(on(&format!("FUEL_FEED_PUMP_FAULT:main_{n}"))), "FCOM PRO-ABN-ECAM p.5176: feed tank n's main pump is at low pressure, abnormally not running, or off -- the same PUMP_FAILED_FRACTION convention the trim pumps use")
                .confirm(5.0)
                .inhibit(&[3, 4, 5, 6, 7, 9, 10])
                .suppressed_by(one(combined_id)),
        );
        v.push(
            proc(stby_id, title(format!("FUEL FEED TK {n} STBY PMP FAULT")), Level::Caution, sd_page::FUEL, gated(on(&format!("FUEL_FEED_PUMP_FAULT:stby_{n}"))), "FCOM PRO-ABN-ECAM p.5178: feed tank n's standby pump is at low pressure, abnormally not running, or off")
                .confirm(5.0)
                .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10])
                .suppressed_by(one(combined_id)),
        );
        v.push(
            proc(
                combined_id,
                title(format!("FUEL FEED TK {n} MAIN + STBY PMPS FAULT")),
                Level::Caution,
                sd_page::FUEL,
                gated(all(vec![on(&format!("FUEL_FEED_PUMP_FAULT:main_{n}")), on(&format!("FUEL_FEED_PUMP_FAULT:stby_{n}"))])),
                "FCOM PRO-ABN-ECAM p.5169: both of feed tank n's pumps are faulted at once, which is when the feed tank has no boosted supply left",
            )
            .confirm(5.0)
            .inhibit(&[4, 5, 6, 7, 9, 10]),
        );
    }

    let sides = [("L", "left"), ("R", "right")];
    for (letter, side) in sides {
        let outer_id = if side == "left" { 281_800_068 } else { 281_800_069 };
        let wing_id = if side == "left" { 281_800_070 } else { 281_800_071 };
        v.push(
            proc(outer_id, title(format!("FUEL {letter} OUTR TK PMP FAULT")), Level::Caution, sd_page::FUEL, gated(on(&format!("FUEL_WING_PUMP_FAULT:outer_{side}"))), "FCOM PRO-ABN-ECAM p.5214: that side's outer tank pump is failed, running dry, or off")
                .confirm(5.0)
                .inhibit(&[3, 4, 5, 6, 7, 9, 10])
                .suppressed_by(one(wing_id)),
        );

        let (inner_fwd_id, inner_aft_id, inner_combined_id) = if side == "left" { (281_800_063, 281_800_058, 281_800_056) } else { (281_800_065, 281_800_060, 281_800_057) };
        v.push(
            proc(inner_fwd_id, title(format!("FUEL {letter} INR TK FWD PMP FAULT")), Level::Caution, sd_page::FUEL, gated(on(&format!("FUEL_WING_PUMP_FAULT:inner_fwd_{side}"))), "FCOM PRO-ABN-ECAM p.5210: that side's inner-tank forward transfer pump is failed, running dry, or off")
                .confirm(5.0)
                .inhibit(&[3, 4, 5, 6, 7, 9, 10])
                .suppressed_by(one(inner_combined_id)),
        );
        v.push(
            proc(inner_aft_id, title(format!("FUEL {letter} INR TK AFT PMP FAULT")), Level::Caution, sd_page::FUEL, gated(on(&format!("FUEL_WING_PUMP_FAULT:inner_aft_{side}"))), "FCOM PRO-ABN-ECAM p.5208: that side's inner-tank aft transfer pump is failed, running dry, or off")
                .confirm(5.0)
                .inhibit(&[3, 4, 5, 6, 7, 9, 10])
                .suppressed_by(one(inner_combined_id)),
        );
        v.push(
            proc(
                inner_combined_id,
                title(format!("FUEL {letter} INR TK FWD+AFT PMPs FAULT")),
                Level::Caution,
                sd_page::FUEL,
                gated(all(vec![on(&format!("FUEL_WING_PUMP_FAULT:inner_fwd_{side}")), on(&format!("FUEL_WING_PUMP_FAULT:inner_aft_{side}"))])),
                "FCOM PRO-ABN-ECAM p.5206: both of that side's inner-tank transfer pumps are faulted at once",
            )
            .confirm(5.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10])
            .suppressed_by(one(wing_id)),
        );

        let (mid_fwd_id, mid_aft_id, mid_combined_id) = if side == "left" { (281_800_062, 281_800_059, 281_800_066) } else { (281_800_064, 281_800_061, 281_800_067) };
        v.push(
            proc(mid_fwd_id, title(format!("FUEL {letter} MID TK FWD PMP FAULT")), Level::Caution, sd_page::FUEL, gated(on(&format!("FUEL_WING_PUMP_FAULT:mid_fwd_{side}"))), "FCOM PRO-ABN-ECAM p.5210: that side's mid-tank forward transfer pump is failed, running dry, or off")
                .confirm(5.0)
                .inhibit(&[3, 4, 5, 6, 7, 9, 10])
                .suppressed_by(one(mid_combined_id)),
        );
        v.push(
            proc(mid_aft_id, title(format!("FUEL {letter} MID TK AFT PMP FAULT")), Level::Caution, sd_page::FUEL, gated(on(&format!("FUEL_WING_PUMP_FAULT:mid_aft_{side}"))), "FCOM PRO-ABN-ECAM p.5208: that side's mid-tank aft transfer pump is failed, running dry, or off")
                .confirm(5.0)
                .inhibit(&[3, 4, 5, 6, 7, 9, 10])
                .suppressed_by(one(mid_combined_id)),
        );
        v.push(
            proc(
                mid_combined_id,
                title(format!("FUEL {letter} MID TK FWD+AFT PMPs FAULT")),
                Level::Caution,
                sd_page::FUEL,
                gated(all(vec![on(&format!("FUEL_WING_PUMP_FAULT:mid_fwd_{side}")), on(&format!("FUEL_WING_PUMP_FAULT:mid_aft_{side}"))])),
                "FCOM PRO-ABN-ECAM p.5212: both of that side's mid-tank transfer pumps are faulted at once",
            )
            .confirm(5.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10])
            .suppressed_by(one(wing_id)),
        );

        v.push(
            proc(
                wing_id,
                title(format!("FUEL {letter} WING FEED PMPS FAULT")),
                Level::Caution,
                sd_page::FUEL,
                gated(any(vec![
                    on(&format!("FUEL_WING_PUMP_FAULT:outer_{side}")),
                    on(&format!("FUEL_WING_PUMP_FAULT:mid_fwd_{side}")),
                    on(&format!("FUEL_WING_PUMP_FAULT:mid_aft_{side}")),
                    on(&format!("FUEL_WING_PUMP_FAULT:inner_fwd_{side}")),
                    on(&format!("FUEL_WING_PUMP_FAULT:inner_aft_{side}")),
                ])),
                "FCOM PRO-ABN-ECAM p.5216: all of that side's wing feed tank pumps are faulted -- an inferred OR of the whole side's own five pumps (E-FUEL-DESIGN.md D4), FlyByWire's catalogue carries no items to check membership against",
            )
            .confirm(5.0)
            .inhibit(&[4, 5, 6, 7, 9, 10]),
        );
    }

    v.push(
        proc(
            281_800_055,
            "FUEL JETTISON VLV NOT CLOSED",
            Level::Caution,
            sd_page::FUEL,
            gated(any(vec![on("FUEL_JETTISON_VALVE_NOT_CLOSED:1"), on("FUEL_JETTISON_VALVE_NOT_CLOSED:2")])),
            "FCOM PRO-ABN-ECAM p.5205: either jettison nozzle valve is abnormally open (has not returned closed when jettison was deselected)",
        )
        .confirm(5.0)
        .inhibit(&[4, 5, 6, 7, 9, 10]),
    );

    v.push(
        proc(
            281_800_072,
            "FUEL LEAK DET FAULT",
            Level::Advisory,
            sd_page::FUEL,
            gated(on("FUEL_LEAK_DETECTOR_FAULT")),
            "FCOM PRO-ABN-ECAM p.5222: the fuel-leak-detection function itself has failed (distinct from a real leak, 281800073, already raised under deep::fuel's own FUEL_LEAK id)",
        )
        .confirm(1.0)
        .inhibit(&[1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12]),
    );

    v.push(
        proc(281_800_079, "FUEL OUTR TK XFR FAULT", Level::Caution, sd_page::FUEL, gated(on("FUEL_OUTER_TRANSFER_FAULT")), "FCOM PRO-ABN-ECAM p.5249: the automatic or manual transfer from either outer tank has failed -- the two outer-transfer valves cg_transfer.rs/registry.rs already model and register")
            .confirm(5.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]),
    );

    v.push(
        proc(
            281_800_098,
            "FUEL WINGS BALANCED",
            Level::Caution,
            sd_page::FUEL,
            gated(all(vec![on("FUEL_WING_IMBALANCE_KNOWN"), on("FUEL_WING_IMBALANCE_EVER_EXCEEDED"), var("FUEL_WING_IMBALANCE_KG").lt(3000.0)])),
            "FCOM PRO-ABN-ECAM p.5289: following a WINGS NOT BALANCED alert, the corrective actions are successful. Threshold: deep::fuel's own WING_IMBALANCE_LIMIT_KG (live.rs), which the FCOM's own 6600 lb symmetric-feed-tank-pair figure (p.5294) corroborates (6600 lb = 2993.7 kg, within 0.2% of 3000.0); kept at the crossfeed-physics constant rather than the FCOM's own slightly different number so the alert and the automatic crossfeed correction agree on the same threshold. Silent, not falsely 'balanced', while the real bridge has not published a reading yet. t04-ecam-healthy: also requires FUEL_WING_IMBALANCE_EVER_EXCEEDED -- without it this fired continuously on every healthy, never-imbalanced aircraft the instant fuel quantity became known, because 'imbalance < 3000 kg' is also true of a perfectly balanced tank set. The message can only mean 'corrected' once a real imbalance actually happened.",
        )
        .confirm(5.0)
        .inhibit(&[3, 4, 5, 6, 7, 9, 10]),
    );
    v.push(
        proc(
            281_800_100,
            "FUEL WINGS NOT BALANCED",
            Level::Caution,
            sd_page::FUEL,
            gated(all(vec![on("FUEL_WING_IMBALANCE_KNOWN"), var("FUEL_WING_IMBALANCE_KG").ge(3000.0)])),
            "FCOM PRO-ABN-ECAM p.5294: an imbalance beyond the limit is detected (the FCOM's own real check is per symmetric feed-tank/inner/mid/outer pair, 2600-6600 lb depending on the pair; this design keeps the existing whole-side-sum simplification, flagged in E-FUEL-DESIGN.md D9)",
        )
        .confirm(5.0)
        .inhibit(&[3, 4, 5, 6, 7, 9, 10]),
    );

    for n in 1..=2u64 {
        v.push(
            proc(281_800_042 + n, title(format!("FUEL FQDC {n} FAULT")), Level::Advisory, sd_page::FUEL, gated(on(&format!("FUEL_FQDC_FAULT:{n}"))), "FCOM PRO-ABN-ECAM p.5183: FQDC channel n is failed -- a discrete computer-health verdict, the same shape ata24.rs's ELEC_TR_APU_FAULT already uses")
                .confirm(1.0)
                .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11]),
        );
        v.push(
            proc(281_800_045 + n, title(format!("FUEL FQMS {n} FAULT")), Level::Advisory, sd_page::FUEL, gated(on(&format!("FUEL_FQMS_FAULT:{n}"))), "FCOM PRO-ABN-ECAM p.5190: FQMS channel n is failed -- the higher-level function FQDC feeds, modelled as its own discrete")
                .confirm(1.0)
                .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11]),
        );
    }
    v.push(
        proc(
            281_800_048,
            "FUEL FQMS 1+2 FAULT",
            Level::Caution,
            sd_page::FUEL,
            gated(any(vec![all(vec![on("FUEL_FQMS_FAULT:1"), on("FUEL_FQMS_FAULT:2")]), all(vec![on("FUEL_FQDC_FAULT:1"), on("FUEL_FQDC_FAULT:2")])])),
            "FCOM PRO-ABN-ECAM p.5192: both FQMS are failed, or both FQDCs are failed (the AGP-comparison clause this FCOM also lists has no modelled AGP channel to check)",
        )
        .confirm(1.0)
        .inhibit(&[4, 5, 6, 7, 9, 10]),
    );
    v.push(
        proc(281_800_078, "FUEL NORM XFR FAULT", Level::Caution, sd_page::FUEL, gated(on("FUEL_TRANSFER_SEQUENCER_FAULT:norm")), "FCOM PRO-ABN-ECAM p.5246: normal (primary) fuel transfers are lost")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .suppressed_by(&[281_800_077]),
    );
    v.push(
        proc(
            281_800_077,
            "FUEL NORM + ALTN XFR FAULT",
            Level::Caution,
            sd_page::FUEL,
            gated(all(vec![on("FUEL_TRANSFER_SEQUENCER_FAULT:norm"), on("FUEL_TRANSFER_SEQUENCER_FAULT:altn")])),
            "FCOM PRO-ABN-ECAM p.5243: a combination of equipment failures (both the normal and the alternate transfer-sequencing channel) cannot be solved with alternate fuel transfers",
        )
        .confirm(1.0)
        .inhibit(&[1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12]),
    );

    for n in 1..=4u64 {
        v.push(
            proc(281_800_013 + n, title(format!("FUEL CROSSFEED VLV {n} FAULT")), Level::Caution, sd_page::FUEL, gated(on(&format!("FUEL_CROSSFEED_VALVE_FAULT:{n}"))), "FCOM PRO-ABN-ECAM p.5160: crossfeed valve n is abnormally closed or abnormally open")
                .confirm(2.0)
                .inhibit(&[4, 5, 6, 7, 9, 10]),
        );
    }

    v.push(
        proc(281_800_008, "FUEL CG DATA DISAGREE", Level::Caution, sd_page::FUEL, gated(on("A32NX_AIRFRAME_ZFW_CG_DISAGREE")), "FCOM PRO-ABN-ECAM p.5154 sources the real alert (FQMS vs WBBC CG, >8%); this model has no separate WBBC channel, so the trigger is a380_systems::airframe::AirframeCgAlerts's crew-entered-vs-computed ZFW CG %MAC check instead (a real, different, cockpit-meaningful comparison, not the FCOM's own one), using the FCOM's own 8% figure")
            .confirm(5.0)
            .inhibit(&[1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12]),
    );
    v.push(
        proc(281_800_096, "FUEL WEIGHT DATA DISAGREE", Level::Caution, sd_page::FUEL, gated(on("A32NX_AIRFRAME_WEIGHT_DISAGREE")), "FCOM PRO-ABN-ECAM p.5283 sources the real alert (FQMS vs WBBC GW, diff >198,400 lb); this model has no separate WBBC channel, so the trigger is a380_systems::airframe::AirframeCgAlerts's crew-entered-vs-computed GW check instead, using the FCOM's own 198,400 lb figure converted to kg")
            .confirm(5.0)
            .inhibit(&[1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12]),
    );

    v.push(
        proc(281_800_095, "FUEL WEIGHT & BALANCE BKUP FAULT", Level::Advisory, sd_page::FUEL, gated(on("FUEL_WB_BACKUP_FAULT")), "FCOM PRO-ABN-ECAM p.5282: WBBC 1 and WBBC 2 are both failed -- a discrete backup-computer health verdict")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11]),
    );

    v.push(
        proc(
            281_800_009,
            "FUEL CG OUT OF RANGE",
            Level::Caution,
            sd_page::FUEL,
            gated(on("A32NX_AIRFRAME_CG_OUT_OF_RANGE")),
            "FCOM PRO-ABN-ECAM p.5156: the CG computed by the FQMS exceeds the certified takeoff limits at engine start, or the alternate gauging system's CG exceeds 54% -- this model has no ENG MODE-selector/alternate-gauging input, so the trigger is a380_systems::airframe::AirframeCgAlerts checking the actual TO CG %MAC against the real forward/aft edges of airframe.json5's own flight envelope (28%..35% forward break by weight, 44% aft)",
        )
        .confirm(5.0)
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11, 12]),
    );
    v.push(
        proc(
            281_800_007,
            "FUEL CG AT FWD LIMIT",
            Level::Caution,
            sd_page::FUEL,
            gated(on("A32NX_AIRFRAME_CG_AT_FWD_LIMIT")),
            "FCOM PRO-ABN-ECAM p.5151 names the real limit (\"Refer to LIM-11 Center of Gravity Limits\", not in this crate's refs) without a number this pass could cite directly; the trigger is a380_systems::airframe::AirframeCgAlerts's 0.5% MAC GENERIC early-annunciation margin before airframe.json5's own real forward envelope edge",
        )
        .confirm(5.0)
        .inhibit(&[1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12]),
    );
    v.push(
        proc(
            281_800_022,
            "FUEL EXCESS AFT CG",
            Level::Caution,
            sd_page::FUEL,
            gated(on("A32NX_AIRFRAME_CG_EXCESS_AFT")),
            "FCOM PRO-ABN-ECAM p.5164: the CG computed by the WBBC exceeds 50% while the fuel system is in automatic mode -- this model has no separate WBBC channel or auto/manual fuel-system-mode discrete, so the trigger is a380_systems::airframe::AirframeCgAlerts comparing the real gross-weight CG %MAC against the FCOM's own 50% number directly, without the mode gate (a real, cited threshold, not a margin against the 43% structural limit an earlier revision used)",
        )
        .confirm(5.0)
        .inhibit(&[1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12]),
    );
    v.push(
        proc(
            281_800_081,
            "FUEL PREDICTED CG OUT OF T.O RANGE",
            Level::Caution,
            sd_page::FUEL,
            gated(on("A32NX_AIRFRAME_TO_CG_OUT_OF_RANGE")),
            "FCOM PRO-ABN-ECAM p.5252: the FQMS detects that, after refuel, the CG will exceed the certified takeoff limits. No published FMS-predicted-TOW channel exists, so a380_systems::airframe::AirframeCgAlerts checks the crew-entered target TO CG %MAC (AIRFRAME_TO_CG_PERCENT_MAC_DESIRED, derived from the EFB target payload and AIRFRAME_GW_DESIRED) against airframe.json5's own flight envelope edges, the closest real proxy for the predicted take-off CG",
        )
        .confirm(5.0)
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11, 12]),
    );

    for (id, n) in [(281_800_039, 2u32), (281_800_040, 5), (281_800_041, 6), (281_800_042, 9)] {
        let feed_n = match n {
            2 => 1,
            5 => 2,
            6 => 3,
            _ => 4,
        };
        v.push(
            proc(id, title(format!("FUEL FEED TK {feed_n} TEMP HI")), Level::Caution, sd_page::FUEL, gated(var(&format!("FUEL_TANK_TEMP_C:{n}")).gt(53.0)), "FCOM PRO-ABN-ECAM p.5179: the fuel temperature in feed tank n is above 53 C -- the FCOM's own direct number, replacing an earlier revision's EASA TCDS wing-tank-at-pylon 50 C proxy")
                .confirm(5.0)
                .inhibit(&[3, 4, 5, 6, 7, 9, 10]),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    fn wiring(id: u64) -> FbwProc {
        wire_all().into_iter().find(|p| p.id == id).unwrap_or_else(|| panic!("{id} is not wired"))
    }

    fn wire_all() -> Vec<FbwProc> {
        let mut v = Vec::new();
        wire(&mut v);
        v
    }

    fn powered() -> BTreeMap<String, f64> {
        [("ELEC_AC_1_BUS_IS_POWERED".to_string(), 1.0)].into_iter().collect()
    }

    fn with(mut m: BTreeMap<String, f64>, k: &str, v: f64) -> BTreeMap<String, f64> {
        m.insert(k.to_string(), v);
        m
    }

    fn holds(c: &Cond, published: &BTreeMap<String, f64>) -> bool {
        c.eval(&|n: &str| *published.get(n).unwrap_or(&0.0))
    }

    #[test]
    fn every_new_alert_is_quiet_cold_and_dark_even_with_its_own_cause_set() {
        let cold: BTreeMap<String, f64> = [
            ("FUEL_APU_FEED_PUMP_FAULT", 1.0),
            ("FUEL_ENG_LP_VALVE_FAULT:2", 1.0),
            ("FUEL_FEED_PUMP_FAULT:main_1", 1.0),
            ("FUEL_FEED_PUMP_FAULT:stby_1", 1.0),
            ("FUEL_WING_PUMP_FAULT:outer_left", 1.0),
            ("FUEL_JETTISON_VALVE_NOT_CLOSED:1", 1.0),
            ("FUEL_LEAK_DETECTOR_FAULT", 1.0),
            ("FUEL_OUTER_TRANSFER_FAULT", 1.0),
            ("FUEL_CROSSFEED_VALVE_FAULT:1", 1.0),
            ("FUEL_FQDC_FAULT:1", 1.0),
            ("FUEL_WB_BACKUP_FAULT", 1.0),
        ]
        .into_iter()
        .map(|(k, v)| (k.to_string(), v))
        .collect();
        for id in [281_800_003, 281_800_018 + 1, 281_800_031, 281_800_068, 281_800_055, 281_800_072, 281_800_079, 281_800_014, 281_800_043, 281_800_095] {
            assert!(!holds(&wiring(id).trigger, &cold), "{id} fired with no AC bus powered");
        }
    }

    #[test]
    fn apu_feed_fault_and_valve_not_closed_are_the_two_real_directions() {
        let fault = wiring(281_800_003);
        let not_closed = wiring(281_800_004);
        let healthy = powered();
        assert!(!holds(&fault.trigger, &healthy) && !holds(&not_closed.trigger, &healthy));

        let stuck_closed = with(with(powered(), "FUEL_APU_FEED_PUMP_FAULT", 0.0), "FUEL_APU_FEED_VALVE_FAULT", 1.0);
        assert!(holds(&fault.trigger, &stuck_closed), "a valve mismatch that is not the not-closed case must raise FAULT");
        assert!(!holds(&not_closed.trigger, &stuck_closed));

        let stuck_open = with(with(with(powered(), "FUEL_APU_FEED_VALVE_FAULT", 1.0), "FUEL_APU_FEED_VALVE_NOT_CLOSED", 1.0), "FUEL_APU_FEED_PUMP_FAULT", 0.0);
        assert!(holds(&not_closed.trigger, &stuck_open));
        assert!(!holds(&fault.trigger, &stuck_open), "the not-closed direction must not also raise the general FAULT id");

        let pump_only = with(powered(), "FUEL_APU_FEED_PUMP_FAULT", 1.0);
        assert!(holds(&fault.trigger, &pump_only));
    }

    #[test]
    fn feed_pump_faults_suppress_under_their_combined_id() {
        let main = wiring(281_800_031);
        let stby = wiring(281_800_035);
        let combined = wiring(281_800_027);
        let m = with(powered(), "FUEL_FEED_PUMP_FAULT:main_1", 1.0);
        assert!(holds(&main.trigger, &m) && !holds(&stby.trigger, &m) && !holds(&combined.trigger, &m));
        assert_eq!(main.suppressed_by, &[281_800_027]);
        assert_eq!(stby.suppressed_by, &[281_800_027]);

        let both = with(with(powered(), "FUEL_FEED_PUMP_FAULT:main_1", 1.0), "FUEL_FEED_PUMP_FAULT:stby_1", 1.0);
        assert!(holds(&combined.trigger, &both), "both pumps faulted must raise the combined id");
    }

    #[test]
    fn wing_pump_fault_hierarchy_suppresses_up_through_the_wing_level() {
        let single = wiring(281_800_065);
        let combined = wiring(281_800_057);
        let wing = wiring(281_800_071);
        assert_eq!(single.suppressed_by, &[281_800_057]);
        assert_eq!(combined.suppressed_by, &[281_800_071]);
        assert_eq!(wing.suppressed_by, &[] as &[u64]);

        let one = with(powered(), "FUEL_WING_PUMP_FAULT:inner_fwd_right", 1.0);
        assert!(holds(&single.trigger, &one), "the single pump's own id must fire");
        assert!(!holds(&combined.trigger, &one), "the fwd+aft combined id needs both pumps");
        assert!(holds(&wing.trigger, &one), "the wing-level OR includes this pump");

        let both_inner = with(with(powered(), "FUEL_WING_PUMP_FAULT:inner_fwd_right", 1.0), "FUEL_WING_PUMP_FAULT:inner_aft_right", 1.0);
        assert!(holds(&combined.trigger, &both_inner), "both of that side's inner pumps together must raise the combined id");
        assert!(holds(&wing.trigger, &both_inner), "and the wing-level OR, since the inner pair is part of it");
    }

    #[test]
    fn jettison_valve_not_closed_ors_both_sides() {
        let p = wiring(281_800_055);
        assert!(!holds(&p.trigger, &powered()));
        assert!(holds(&p.trigger, &with(powered(), "FUEL_JETTISON_VALVE_NOT_CLOSED:1", 1.0)));
        assert!(holds(&p.trigger, &with(powered(), "FUEL_JETTISON_VALVE_NOT_CLOSED:2", 1.0)));
    }

    #[test]
    fn leak_det_fault_and_outer_xfer_fault_are_plain_direct_readings() {
        let leak_det = wiring(281_800_072);
        assert_eq!(leak_det.level, Level::Advisory, "FCOM p.5222's blank indications box: no aural, no master light");
        assert!(holds(&leak_det.trigger, &with(powered(), "FUEL_LEAK_DETECTOR_FAULT", 1.0)));
        assert!(!holds(&leak_det.trigger, &powered()));

        let outer = wiring(281_800_079);
        assert!(holds(&outer.trigger, &with(powered(), "FUEL_OUTER_TRANSFER_FAULT", 1.0)));
        assert!(!holds(&outer.trigger, &powered()));
    }

    #[test]
    fn wings_balanced_and_not_balanced_are_mutually_exclusive_and_silent_when_unknown() {
        let balanced = wiring(281_800_098);
        let not_balanced = wiring(281_800_100);
        let unknown = with(powered(), "FUEL_WING_IMBALANCE_KNOWN", 0.0);
        assert!(!holds(&balanced.trigger, &unknown) && !holds(&not_balanced.trigger, &unknown), "the real bridge has not published a reading yet: both must stay silent");

        let level = with(with(with(powered(), "FUEL_WING_IMBALANCE_KNOWN", 1.0), "FUEL_WING_IMBALANCE_EVER_EXCEEDED", 1.0), "FUEL_WING_IMBALANCE_KG", 100.0);
        assert!(holds(&balanced.trigger, &level) && !holds(&not_balanced.trigger, &level), "FCOM p.5289: FUEL WINGS BALANCED is the recovery message following a prior FUEL WINGS NOT BALANCED, so it needs FUEL_WING_IMBALANCE_EVER_EXCEEDED set, not just KNOWN");

        let imbalanced = with(with(powered(), "FUEL_WING_IMBALANCE_KNOWN", 1.0), "FUEL_WING_IMBALANCE_KG", 4000.0);
        assert!(!holds(&balanced.trigger, &imbalanced) && holds(&not_balanced.trigger, &imbalanced));
    }

    #[test]
    fn fqdc_and_fqms_singles_are_advisory_and_the_combined_ids_are_caution() {
        assert_eq!(wiring(281_800_043).level, Level::Advisory, "FCOM p.5183: SD page only, no aural, no master light");
        assert_eq!(wiring(281_800_046).level, Level::Advisory, "FCOM p.5190: SD page only");
        assert_eq!(wiring(281_800_048).level, Level::Caution, "FCOM p.5192: Audio + Master Light");

        let combined = wiring(281_800_048);
        let via_fqms = with(with(powered(), "FUEL_FQMS_FAULT:1", 1.0), "FUEL_FQMS_FAULT:2", 1.0);
        assert!(holds(&combined.trigger, &via_fqms));
        let via_fqdc = with(with(powered(), "FUEL_FQDC_FAULT:1", 1.0), "FUEL_FQDC_FAULT:2", 1.0);
        assert!(holds(&combined.trigger, &via_fqdc));
        let only_one = with(powered(), "FUEL_FQMS_FAULT:1", 1.0);
        assert!(!holds(&combined.trigger, &only_one));
    }

    #[test]
    fn norm_xfr_fault_is_suppressed_by_norm_plus_altn() {
        let norm = wiring(281_800_078);
        let both = wiring(281_800_077);
        assert_eq!(norm.suppressed_by, &[281_800_077]);
        let norm_only = with(powered(), "FUEL_TRANSFER_SEQUENCER_FAULT:norm", 1.0);
        assert!(holds(&norm.trigger, &norm_only) && !holds(&both.trigger, &norm_only));
        let both_faulted = with(with(powered(), "FUEL_TRANSFER_SEQUENCER_FAULT:norm", 1.0), "FUEL_TRANSFER_SEQUENCER_FAULT:altn", 1.0);
        assert!(holds(&both.trigger, &both_faulted));
    }

    #[test]
    fn crossfeed_valves_are_wired_individually_with_no_cross_suppression() {
        for n in 1..=4u64 {
            let p = wiring(281_800_013 + n);
            assert_eq!(p.suppressed_by, &[] as &[u64], "FlyByWire does not pair the crossfeed valves the way it pairs the trim pumps");
            let m = with(powered(), &format!("FUEL_CROSSFEED_VALVE_FAULT:{n}"), 1.0);
            assert!(holds(&p.trigger, &m));
            assert!(!holds(&p.trigger, &powered()));
        }
    }

    #[test]
    fn wb_backup_fault_is_advisory_and_a_plain_direct_reading() {
        let p = wiring(281_800_095);
        assert_eq!(p.level, Level::Advisory, "FCOM p.5282's blank indications box");
        assert!(holds(&p.trigger, &with(powered(), "FUEL_WB_BACKUP_FAULT", 1.0)));
        assert!(!holds(&p.trigger, &powered()));
    }

    #[test]
    fn cg_and_weight_disagree_read_the_efb_owned_names() {
        let cg = wiring(281_800_008);
        let weight = wiring(281_800_096);
        assert!(holds(&cg.trigger, &with(powered(), "A32NX_AIRFRAME_ZFW_CG_DISAGREE", 1.0)));
        assert!(!holds(&cg.trigger, &powered()));
        assert!(holds(&weight.trigger, &with(powered(), "A32NX_AIRFRAME_WEIGHT_DISAGREE", 1.0)));
        assert!(!holds(&weight.trigger, &powered()));
    }

    #[test]
    fn the_cg_envelope_ids_each_read_their_own_efb_owned_discrete() {
        let cases = [(281_800_009, "A32NX_AIRFRAME_CG_OUT_OF_RANGE"), (281_800_007, "A32NX_AIRFRAME_CG_AT_FWD_LIMIT"), (281_800_022, "A32NX_AIRFRAME_CG_EXCESS_AFT"), (281_800_081, "A32NX_AIRFRAME_TO_CG_OUT_OF_RANGE")];
        for (id, name) in cases {
            let p = wiring(id);
            assert!(holds(&p.trigger, &with(powered(), name, 1.0)), "{id} must fire on its own {name}");
            assert!(!holds(&p.trigger, &powered()), "{id} must be quiet when {name} is false");
            assert_eq!(p.level, Level::Caution, "FCOM shows Audio and/or Master Light for every one of these four");
        }
    }

    #[test]
    fn feed_tank_temp_hi_uses_the_fcom_s_own_53c_and_resolves_to_the_right_engine() {
        let cases = [(281_800_039, 2u32, 1u32), (281_800_040, 5, 2), (281_800_041, 6, 3), (281_800_042, 9, 4)];
        for (id, n, feed_n) in cases {
            let p = wiring(id);
            assert!(p.title.contains(&format!("FEED TK {feed_n} TEMP HI")));
            let at_limit = with(powered(), &format!("FUEL_TANK_TEMP_C:{n}"), 53.0);
            assert!(!holds(&p.trigger, &at_limit), "FCOM p.5179: strictly above 53 C, not at it");
            let above = with(powered(), &format!("FUEL_TANK_TEMP_C:{n}"), 53.1);
            assert!(holds(&p.trigger, &above));
        }
    }
}
