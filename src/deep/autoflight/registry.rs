//! Registers `deep::autoflight`'s components and failures. See this
//! directory's `mod.rs` for why the area exists and what it deliberately
//! does not model.
//!
//! Every id is `failure_id(Area::AutoFlight, ATA, n)`, `n` sequential in the
//! order this function assigns them.

use crate::deep::api::*;

const ATA: u16 = 22;

pub fn register(r: &mut Registry) {
    // ---- 220800005/007/008/015: the FCU (AFS control panel) and its two
    // MFD backup pages.
    let fcu = "22_afs.fcu";
    let fcu_fault = failure_id(Area::AutoFlight, ATA, 1);
    let fcu_switched_off = failure_id(Area::AutoFlight, ATA, 2);
    r.component(ComponentDef {
        id: fcu.into(),
        area: Area::AutoFlight,
        ata: ATA,
        name: "FCU (AFS control panel)".into(),
        params: vec![
            ParamDef { name: "fault".into(), meaning: "the panel itself broken, 0 healthy .. 1 fully faulted".into(), healthy: 0.0 },
            ParamDef { name: "switched_off".into(), meaning: "the panel deliberately shut off (a discrete state, not a fault), 0 on .. 1 off".into(), healthy: 0.0 },
        ],
        failures: vec![fcu_fault, fcu_switched_off],
    });
    r.failure(FailureDef {
        id: fcu_fault,
        area: Area::AutoFlight,
        ata: ATA,
        name: "FCU fault".into(),
        component: fcu.into(),
        model_field: "autoflight::live::AutoFlightLive.fcu_fault".into(),
        magnitude: "0 healthy .. 1 fully faulted -- a component-broken flag, the same shape as ELEC_GEN_n_FAULT in fbw/ata24.rs".into(),
        effect: "FlyByWire's own 220800005 AUTO FLT AFS CTL PNL FAULT; FCOM confirms this exact real trigger (\"the AFS control panel is failed\", FCOM p.4785, E-AIR-FCOM.json)".into(),
    });
    r.failure(FailureDef {
        id: fcu_switched_off,
        area: Area::AutoFlight,
        ata: ATA,
        name: "FCU switched off".into(),
        component: fcu.into(),
        model_field: "autoflight::live::AutoFlightLive.fcu_switched_off".into(),
        magnitude: "0 on .. 1 off -- a discrete state (crew/CDS-power action), not a fault".into(),
        effect: "FlyByWire's own 220800015 CDS & AUTO FLT FCU SWITCHED OFF; FCOM confirms (\"the FCU is switched off: the EFIS CPs and the AFS CP are electrically shutoff\", FCOM p.4797, E-AIR-FCOM.json)".into(),
    });

    let capt_bkup = "22_afs.capt_mfd_fcu_backup";
    let capt_bkup_fault = failure_id(Area::AutoFlight, ATA, 3);
    r.component(ComponentDef {
        id: capt_bkup.into(),
        area: Area::AutoFlight,
        ata: ATA,
        name: "Captain's MFD FCU backup page".into(),
        params: vec![ParamDef { name: "fault".into(), meaning: "the backup page itself broken, 0 healthy .. 1 fully faulted".into(), healthy: 0.0 }],
        failures: vec![capt_bkup_fault],
    });
    r.failure(FailureDef {
        id: capt_bkup_fault,
        area: Area::AutoFlight,
        ata: ATA,
        name: "Captain MFD FCU backup fault".into(),
        component: capt_bkup.into(),
        model_field: "autoflight::live::AutoFlightLive.capt_bkup_fault".into(),
        magnitude: "0 healthy .. 1 fully faulted".into(),
        effect: "half of FlyByWire's own 220800007 AUTO FLT AFS CTL PNL+CAPT BKUP CTL FAULT (the other half is fcu_fault); FCOM confirms the exact compound condition (\"the AFS control panel is failed, and the CAPT (F/O) AFS page on the FCU backup is failed\", FCOM p.4791, E-AIR-FCOM.json)".into(),
    });

    let fo_bkup = "22_afs.fo_mfd_fcu_backup";
    let fo_bkup_fault = failure_id(Area::AutoFlight, ATA, 4);
    r.component(ComponentDef {
        id: fo_bkup.into(),
        area: Area::AutoFlight,
        ata: ATA,
        name: "First Officer's MFD FCU backup page".into(),
        params: vec![ParamDef { name: "fault".into(), meaning: "the backup page itself broken, 0 healthy .. 1 fully faulted".into(), healthy: 0.0 }],
        failures: vec![fo_bkup_fault],
    });
    r.failure(FailureDef {
        id: fo_bkup_fault,
        area: Area::AutoFlight,
        ata: ATA,
        name: "F/O MFD FCU backup fault".into(),
        component: fo_bkup.into(),
        model_field: "autoflight::live::AutoFlightLive.fo_bkup_fault".into(),
        magnitude: "0 healthy .. 1 fully faulted".into(),
        effect: "half of FlyByWire's own 220800008 AUTO FLT AFS CTL PNL+F/O BKUP CTL FAULT (the other half is fcu_fault); same FCOM procedure as 220800007, mirrored for the F/O page".into(),
    });

    // ---- 220800014: TCAS/AP mode arbitration.
    let tcas_mode = "22_afs.tcas_ap_mode_arbitration";
    let tcas_mode_fault = failure_id(Area::AutoFlight, ATA, 5);
    r.component(ComponentDef {
        id: tcas_mode.into(),
        area: Area::AutoFlight,
        ata: ATA,
        name: "TCAS/AP mode arbitration logic".into(),
        params: vec![ParamDef { name: "fault".into(), meaning: "the arbitration logic itself broken, 0 healthy .. 1 fully faulted".into(), healthy: 0.0 }],
        failures: vec![tcas_mode_fault],
    });
    r.failure(FailureDef {
        id: tcas_mode_fault,
        area: Area::AutoFlight,
        ata: ATA,
        name: "TCAS/AP mode arbitration fault".into(),
        component: tcas_mode.into(),
        model_field: "autoflight::live::AutoFlightLive.tcas_mode_fault".into(),
        magnitude: "0 healthy .. 1 fully faulted -- an arbitration-logic LRU fault, no threshold invented".into(),
        effect: "FlyByWire's own 220800014 AUTO FLT TCAS MODE FAULT; FCOM confirms (\"the AP/FD TCAS mode is failed\", FCOM p.4796, E-AIR-FCOM.json)".into(),
    });

    // ---- 220800006: approach capability downgraded. No new failure --
    // this is a combinational read of `Truth::prim_healthy`, already real
    // and published (`deep/plugin.rs`'s own table). The FCOM's own real
    // trigger (p.4789, E-AIR-FCOM.json) ORs several PRIM-internal sub-
    // sensor losses (gyrometer, vertical accelerometer, AOA/sideslip/IRS
    // voting) this port does not model at that resolution -- only the
    // PRIM's own overall health. Recorded here as a documented
    // simplification, not an invented threshold: an unhealthy PRIM
    // genuinely can and does downgrade approach capability on the real
    // aircraft, this port just cannot see which of its internal lanes
    // caused it.
    r.component(ComponentDef {
        id: "22_afs.approach_capability_monitor".into(),
        area: Area::AutoFlight,
        ata: ATA,
        name: "Approach capability computation".into(),
        params: vec![],
        failures: vec![],
    });
}
