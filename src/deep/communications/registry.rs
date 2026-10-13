//! Registers `deep::communications`'s components and failures. See this
//! directory's `mod.rs` for scope. Every id is
//! `failure_id(Area::Communications, ATA, n)`, `n` sequential in the order
//! this function assigns them.

use crate::deep::api::*;

const ATA: u16 = 23;

fn simple(r: &mut Registry, comp: &str, name: &str, n: u16, model_field: &str, effect: &str) -> u64 {
    let fid = failure_id(Area::Communications, ATA, n);
    r.component(ComponentDef {
        id: comp.into(),
        area: Area::Communications,
        ata: ATA,
        name: name.into(),
        params: vec![ParamDef { name: "fault".into(), meaning: "0 healthy .. 1 fully faulted".into(), healthy: 0.0 }],
        failures: vec![fid],
    });
    r.failure(FailureDef {
        id: fid,
        area: Area::Communications,
        ata: ATA,
        name: format!("{name} fault"),
        component: comp.into(),
        model_field: model_field.into(),
        magnitude: "0 healthy .. 1 fully faulted -- a component-broken flag, the same shape as ELEC_GEN_n_FAULT in fbw/ata24.rs".into(),
        effect: effect.into(),
    });
    fid
}

pub fn register(r: &mut Registry) {
    // ---- 230800001/002/003: CIDS 1/2/3 computers and their 4 cabin-com
    // channels.
    simple(r, "23_com.cids_1", "CIDS 1 computer", 1, "communications::live::CommunicationsLive.cids_fault", "one third of FlyByWire's own 230800001 CAB COM CIDS 1+2+3 FAULT (all 3 ANDed)");
    simple(r, "23_com.cids_2", "CIDS 2 computer", 2, "communications::live::CommunicationsLive.cids_fault", "one third of FlyByWire's own 230800001 CAB COM CIDS 1+2+3 FAULT (all 3 ANDed)");
    simple(r, "23_com.cids_3", "CIDS 3 computer", 3, "communications::live::CommunicationsLive.cids_fault", "one third of FlyByWire's own 230800001 CAB COM CIDS 1+2+3 FAULT (all 3 ANDed)");

    for (n, channel) in [(4u16, "PA_UPPER"), (5, "PA_MAIN"), (6, "PA_LOWER"), (7, "INTERPHONE")] {
        let comp = format!("23_com.cids_channel_{}", channel.to_lowercase());
        let fid = failure_id(Area::Communications, ATA, n);
        r.component(ComponentDef {
            id: comp.clone(),
            area: Area::Communications,
            ata: ATA,
            name: format!("CIDS {channel} channel"),
            params: vec![ParamDef { name: "fault".into(), meaning: "0 healthy .. <1 degraded .. 1 fully faulted, matching the catalogue's own PA/CABIN INTERPHONE DEGRADED vs FAULT item pair".into(), healthy: 0.0 }],
            failures: vec![fid],
        });
        r.failure(FailureDef {
            id: fid,
            area: Area::Communications,
            ata: ATA,
            name: format!("CIDS {channel} channel fault"),
            component: comp,
            model_field: "communications::live::CommunicationsLive.cids_channel".into(),
            magnitude: "0 healthy .. <1 degraded .. 1 fully faulted".into(),
            effect: "feeds FlyByWire's own 230800002 CAB COM CIDS CABIN COM FAULT (any channel at 1.0, not all 3 computers) and 230800003 CAB COM COM DEGRADED (any channel in (0,1))".into(),
        });
    }

    // ---- 230800004/005/006: cockpit PTT switches. Confirm 40 s is the
    // FCOM's own real figure (FCOM p.4816, E-AIR-FCOM.json 230800004),
    // applied in fbw/ata21_22_23.rs, not baked in here.
    simple(r, "23_com.capt_ptt", "Captain's PTT switch", 8, "communications::live::CommunicationsLive.capt_ptt_stuck", "FlyByWire's own 230800004 COM CAPT PTT STUCK");
    simple(r, "23_com.fo_ptt", "First Officer's PTT switch", 9, "communications::live::CommunicationsLive.fo_ptt_stuck", "FlyByWire's own 230800005 COM F/O PTT STUCK");
    simple(r, "23_com.third_occ_ptt", "Third occupant's PTT switch", 10, "communications::live::CommunicationsLive.third_ptt_stuck", "FlyByWire's own 230800006 COM THIRD OCCUPANT PTT STUCK");

    // ---- 230800007: ATSU/datalink router.
    simple(r, "23_com.atsu_datalink_router", "ATSU/datalink router", 11, "communications::live::CommunicationsLive.datalink_fault", "FlyByWire's own 230800007 COM DATALINK FAULT");

    // ---- 230800008-011: HF 1/2 transceivers, two independent sub-faults
    // each (datalink / stuck emitting), one component per radio.
    two_fault_component(
        r,
        "23_com.hf1",
        "HF 1 transceiver",
        (12, "hf1_datalink_fault", "FlyByWire's own 230800008 COM HF 1 DATALINK FAULT"),
        (13, "hf1_stuck_emitting", "FlyByWire's own 230800010 COM HF 1 EMITTING"),
    );
    two_fault_component(
        r,
        "23_com.hf2",
        "HF 2 transceiver",
        (14, "hf2_datalink_fault", "FlyByWire's own 230800009 COM HF 2 DATALINK FAULT"),
        (15, "hf2_stuck_emitting", "FlyByWire's own 230800011 COM HF 2 EMITTING"),
    );

    // ---- 230800019-021: SATCOM transceiver, three independent sub-faults
    // (main unit / datalink / voice channel), one component.
    let satcom_main = failure_id(Area::Communications, ATA, 16);
    let satcom_datalink = failure_id(Area::Communications, ATA, 17);
    let satcom_voice = failure_id(Area::Communications, ATA, 18);
    r.component(ComponentDef {
        id: "23_com.satcom".into(),
        area: Area::Communications,
        ata: ATA,
        name: "SATCOM transceiver".into(),
        params: vec![
            ParamDef { name: "main_fault".into(), meaning: "0 healthy .. 1 fully faulted".into(), healthy: 0.0 },
            ParamDef { name: "datalink_fault".into(), meaning: "0 healthy .. 1 fully faulted".into(), healthy: 0.0 },
            ParamDef { name: "voice_fault".into(), meaning: "0 healthy .. 1 fully faulted".into(), healthy: 0.0 },
        ],
        failures: vec![satcom_main, satcom_datalink, satcom_voice],
    });
    r.failure(FailureDef { id: satcom_main, area: Area::Communications, ata: ATA, name: "SATCOM main unit fault".into(), component: "23_com.satcom".into(), model_field: "communications::live::CommunicationsLive.satcom_fault".into(), magnitude: "0 healthy .. 1 fully faulted".into(), effect: "FlyByWire's own 230800020 COM SATCOM FAULT".into() });
    r.failure(FailureDef { id: satcom_datalink, area: Area::Communications, ata: ATA, name: "SATCOM datalink fault".into(), component: "23_com.satcom".into(), model_field: "communications::live::CommunicationsLive.satcom_datalink_fault".into(), magnitude: "0 healthy .. 1 fully faulted".into(), effect: "FlyByWire's own 230800019 COM SATCOM DATALINK FAULT".into() });
    r.failure(FailureDef { id: satcom_voice, area: Area::Communications, ata: ATA, name: "SATCOM voice fault".into(), component: "23_com.satcom".into(), model_field: "communications::live::CommunicationsLive.satcom_voice_fault".into(), magnitude: "0 healthy .. 1 fully faulted".into(), effect: "FlyByWire's own 230800021 COM SATCOM VOICE FAULT".into() });

    // ---- 230800022-025: VHF 1/2/3 stuck-emitting, VHF 3 datalink. Extends
    // (in doc comment only, not in code) `radios.rs`'s existing real VHF
    // tuning model with the stuck-transmitting/datalink LRU faults that
    // model does not carry. Confirm 60 s is the catalogue-sourced figure
    // the design sheet cites (RMP TX KEY DESELECT after 60 s of emitting),
    // applied in fbw/ata21_22_23.rs.
    simple(r, "23_com.vhf1", "VHF 1 (stuck emitting)", 19, "communications::live::CommunicationsLive.vhf1_stuck_emitting", "FlyByWire's own 230800022 COM VHF 1 EMITTING");
    simple(r, "23_com.vhf2", "VHF 2 (stuck emitting)", 20, "communications::live::CommunicationsLive.vhf2_stuck_emitting", "FlyByWire's own 230800023 COM VHF 2 EMITTING");
    two_fault_component(
        r,
        "23_com.vhf3",
        "VHF 3 (extends radios.rs's real VHF tuning model with the LRU faults it does not carry)",
        (21, "vhf3_stuck_emitting", "FlyByWire's own 230800024 COM VHF 3 EMITTING"),
        (22, "vhf3_datalink_fault", "FlyByWire's own 230800025 COM VHF 3 DATALINK FAULT"),
    );
}

/// A component with exactly two independent binary sub-faults, the shape
/// HF 1/2 and VHF 3 each need.
fn two_fault_component(r: &mut Registry, comp: &str, name: &str, (n_a, field_a, effect_a): (u16, &str, &str), (n_b, field_b, effect_b): (u16, &str, &str)) {
    let fid_a = failure_id(Area::Communications, ATA, n_a);
    let fid_b = failure_id(Area::Communications, ATA, n_b);
    r.component(ComponentDef {
        id: comp.into(),
        area: Area::Communications,
        ata: ATA,
        name: name.into(),
        params: vec![
            ParamDef { name: field_a.into(), meaning: "0 healthy .. 1 fully faulted".into(), healthy: 0.0 },
            ParamDef { name: field_b.into(), meaning: "0 healthy .. 1 fully faulted".into(), healthy: 0.0 },
        ],
        failures: vec![fid_a, fid_b],
    });
    r.failure(FailureDef {
        id: fid_a,
        area: Area::Communications,
        ata: ATA,
        name: format!("{name} {field_a}"),
        component: comp.into(),
        model_field: format!("communications::live::CommunicationsLive.{field_a}"),
        magnitude: "0 healthy .. 1 fully faulted".into(),
        effect: effect_a.into(),
    });
    r.failure(FailureDef {
        id: fid_b,
        area: Area::Communications,
        ata: ATA,
        name: format!("{name} {field_b}"),
        component: comp.into(),
        model_field: format!("communications::live::CommunicationsLive.{field_b}"),
        magnitude: "0 healthy .. 1 fully faulted".into(),
        effect: effect_b.into(),
    });
}
