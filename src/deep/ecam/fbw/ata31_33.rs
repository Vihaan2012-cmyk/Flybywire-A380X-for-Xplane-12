//! ATA 31 -- indicating/recording, and ATA 33 -- lights. Both share
//! `AbnormalSensed/ata31-32-33.ts` with ATA 32, which already had its own
//! full pass (`ata32.rs`) and is left alone here.
//!
//! The file defines 35 procedures across ATA 31 (`311800001`..`319800004`)
//! and exactly **one** ATA 33 entry (`334800101` CABIN EMER EXIT LT FAULT --
//! this airframe's lighting chapter has no other abnormal-sensed procedure
//! in this file at all, so "ATA 33 is thin" is not a guess).
//!
//! # What is wired, and from where
//!
//! `deep::avionics_network` publishes `AVNCS_MODULE_CPIOM_C1_PARTITION_
//! FWS_AVAILABLE`: this port models a single CPIOM-C module (`CPIOM_C1`),
//! not the dual-module split the real FWS hosts its two channels on, so this
//! one ARINC 653 partition variable is this port's whole verdict on "is the
//! FWS function up" -- it cannot separately say FWS 1 vs FWS 2. That flag
//! going false is therefore FlyByWire's `314800004` FWS 1+2 FAULT (the total
//! loss this port can actually see), never `314800008`/`314800009` FWS 1/2
//! FAULT alone (nothing here can say which channel).
//!
//! `314800003` FWS 1+2 & FCDC 1+2 FAULT adds the two flight-control data
//! concentrators: `deep::electrical` publishes `ELEC_LOAD_fcdc-1_POWERED`
//! and `ELEC_LOAD_fcdc-2_POWERED` from its own dual-fed FCDC loads
//! (`electrical/loads.rs:353-354`, ATA 27), and losing power to an LRU is a
//! sound (if not exhaustive) subset of "faulted" -- the same reasoning
//! `ata24.rs`'s bus and TR/GEN procedures already use. `314800003`'s trigger
//! is a strict superset of `314800004`'s own condition (it requires the FWS
//! partition down *and* both FCDCs unpowered), so `314800004` names it as
//! its suppressor: when the combined procedure is up, the plainer one stays
//! down, matching FlyByWire's own `notActiveWhenItemActive` convention.
//!
//! `319800002`/`319800003` RECORDER CVR/DFDR FAULT read
//! `ELEC_LOAD_cvr_POWERED`/`ELEC_LOAD_dfdr_POWERED` (`electrical/loads.rs:
//! 713-716`, ATA 31's own mandatory-recorder loads) the same way.
//! `319800001` ACCELMTR FAULT and `319800004` SYS FAULT stay unwired: no
//! area publishes an independent accelerometer state, and "SYS FAULT" names
//! the recorder system as a whole, which is not one load this port can
//! point at without guessing which failure mode it means.
//!
//! `334800101` CABIN EMER EXIT LT FAULT reads
//! `ELEC_LOAD_emer-lighting-charger-1_POWERED` and `...-2_POWERED`
//! (`electrical/loads.rs:849-853`, ATA 33's own emergency-lighting-battery
//! charger loads -- the circuit that keeps the exit-light battery packs
//! charged). Both unpowered together is what "FAULT" names: one charger
//! down still leaves the other battery pack serviced.
//!
//! # The "network alive" gate, again
//!
//! Every trigger above is gated on [`network_alive`], repeated from
//! `ata24.rs` rather than made `pub` there for one caller: without it, a
//! cold and dark aircraft -- FWS partition unpowered, both FCDCs unpowered,
//! both recorders unpowered, both lighting chargers unpowered -- would light
//! every one of these five procedures at the gate before the first engine
//! ever turns. The gate is the FWS's own real precondition (it needs power
//! to say anything, including about itself), written down rather than
//! invented for this file.
//!
//! # `E-IND-DESIGN.md`'s Phase 2 pass (2026-09-27)
//!
//! `E:/fbw-debug/ecam/E-IND-DESIGN.md` designed and this pass wires the rest
//! of ATA 31's CDS/KCCU/HUD/video/recorder family and the two FWS-database
//! ids, each from a new small `deep::electrical` avionics load (the LRU-
//! power pattern this module's own doc above already uses for FCDC/CVR/
//! DFDR/charger) or a new `deep::avionics_network`/`Truth::controls`
//! reading:
//!
//! * `311800001` CDS & AUTO FLT FCU SWITCHED OFF: a direct mirror of
//!   `Truth::controls.fcu_switch_off`, published `CDS_FCU_SWITCH_OFF` by
//!   `deep::avionics_network` (no real dataref found for this port yet;
//!   defaults to the normal, guard-down position).
//! * `311800002`..`311800011` (EFIS backup/control panels, PFD/ND/EWD DU
//!   monitoring): each reads a new `deep::electrical` avionics load's own
//!   `ELEC_LOAD_*_POWERED` (`electrical/loads.rs`'s `ata31_ind_group`),
//!   losing power being the same sound (if not exhaustive) subset of
//!   "faulted"/"not monitored" this module's own doc above already reasons
//!   for its other CDS peripherals.
//! * `313800003`/`313800004` (whole-unit KCCU fault) and `313800007` (CDS
//!   mailbox): the same LRU-power pattern.
//! * `314800001`/`314800005` (FWS database rejected): `deep::
//!   avionics_network`'s own two new BITE flags on the CPIOM-C1 FWS
//!   partition, `AVNCS_MODULE_CPIOM_C1_{CUSTOMIZATION,ATQC}_DB_REJECTED`,
//!   gated on that partition being available the same way
//!   `fws_unavailable` already gates `314800004`/`314800003` above.
//! * `316800001` (HUD) and `318800001` (video multiplexer): the same
//!   LRU-power pattern.
//! * `319800001`/`319800004` (recorder accelerometer / DFDAU): two more
//!   ATA-31-mandatory-equipment-class loads joining `319800002`/`003`'s own
//!   `ata31_recorders` group.
//!
//! # Wired after the FCOM pass
//!
//! * `311800012` CDS DISPLAY DISAGREE: FCOM p.5393 -- not an air-data
//!   comparison, but the CDS's own check of what a display unit shows
//!   against the copy it sends a second unit to monitor. Each modelled DU
//!   carries a `display_monitor_disagree` failure (`deep::sensors`,
//!   extending the DU's `31_elec` component).
//! * `313800001`/`002`/`005`/`006` CURSOR CTL / KEYBOARD FAULT: FCOM
//!   p.5377/5383. The KCCU's cursor control device and keyboard fail
//!   independently (FCOM DSC-31-30-10); each part's BITE failure extends
//!   the KCCU's `31_elec` component from `deep::sensors`, so
//!   `deep::electrical`'s shared load model is untouched.
//! * `316800002` HUD FPV DISAGREE: FCOM p.5416, "a discrepancy between the
//!   IRS sources of the flight path angle"; DSC-31-60-20 gives the
//!   threshold, CHECK FPV when the two selected IRS differ by more than
//!   1 deg. `deep::sensors` compares the CAPT and F/O side IRs' own
//!   published flight path angles.
//!
//! # What is still deliberately left unwired
//!
//! * The other four `314800xxx` FWS ids (`314800006` AUDIO FUNCTION LOST,
//!   `314800007` ECP FAULT, `314800008`/`314800009` FWS 1/2 FAULT
//!   individually): each names a specific FWS sub-function or a single
//!   channel this port's one partition flag cannot distinguish.

use super::{phase, proc, sd_page, FbwProc};
use crate::deep::api::{all, var, any, Cond, Level};

/// FlyByWire's own inhibit window for this file's default-inhibit entries
/// (`phase::TAKEOFF_AND_LANDING`) is kept for every new entry below too,
/// unless noted: none of these CDS/KCCU/HUD/recorder cautions are the kind
/// FlyByWire itself lifts the standard takeoff/landing inhibit for (its
/// sibling `311800xxx`/`313800xxx`/`316800xxx`/`318800xxx`/`319800xxx`
/// entries in `FwsAbnormalSensed.ts` all keep the same window their ATA
/// chapter defaults to).

/// At least one main AC bus is live, so the electrical network is running
/// and an unpowered load is that load's own problem rather than a cold
/// aeroplane. Identical to `ata24::network_alive`, which is private to that
/// module; repeated here rather than exported for this one caller.
fn network_alive() -> Cond {
    any(vec![
        var("ELEC_AC_1_BUS_IS_POWERED").on(),
        var("ELEC_AC_2_BUS_IS_POWERED").on(),
        var("ELEC_AC_3_BUS_IS_POWERED").on(),
        var("ELEC_AC_4_BUS_IS_POWERED").on(),
    ])
}

/// `deep::avionics_network`'s whole verdict on "is the FWS function up" --
/// see this module's doc comment for why one flag stands for both channels.
fn fws_unavailable() -> Cond {
    var("AVNCS_MODULE_CPIOM_C1_PARTITION_FWS_AVAILABLE").off()
}

fn fcdc_dead(n: u32) -> Cond {
    var(&format!("ELEC_LOAD_fcdc-{n}_POWERED")).off()
}

pub fn wire(v: &mut Vec<FbwProc>) {
    // ---- FWS 1+2 FAULT.
    v.push(
        proc(
            314_800_004,
            "FWS 1+2 FAULT",
            Level::Caution,
            sd_page::STATUS,
            all(vec![fws_unavailable(), network_alive()]),
            "deep::avionics_network's single CPIOM-C1 FWS partition flag going unavailable while the network is powered -- this port models one CPIOM-C module, so this is its whole verdict on the FWS function",
        )
        .confirm(1.0)
        // `E-IND-FCOM.json` (Phase 2 FCOM pass): FCOM PRO-ABN-ECAM p.5405
        // shows this procedure with an empty inhibited-phase list -- never
        // inhibited, unlike this file's own default. A real FWS failure has
        // to stay visible through every phase, including the ones its own
        // failure would otherwise silence other alerts in.
        .inhibit(phase::NONE)
        .suppressed_by(&[314_800_003])
        .items(9, Vec::new()),
    );

    // ---- FWS 1+2 & FCDC 1+2 FAULT. Strict superset of the above: the FWS
    // partition down as well as both FCDC loads unpowered.
    v.push(
        proc(
            314_800_003,
            "FWS 1+2 & FCDC 1+2 FAULT",
            Level::Caution,
            sd_page::STATUS,
            all(vec![fws_unavailable(), fcdc_dead(1), fcdc_dead(2), network_alive()]),
            "the FWS partition down together with both flight-control data concentrators' own ELEC_LOAD_fcdc-n_POWERED reading unpowered",
        )
        .confirm(1.0)
        .items(10, Vec::new()),
    );

    // ---- RECORDER CVR/DFDR FAULT. `deep::electrical`'s own mandatory
    // flight-recorder loads, unpowered while the network is live.
    v.push(
        proc(
            319_800_002,
            "RECORDER CVR FAULT",
            Level::Advisory,
            sd_page::STATUS,
            all(vec![var("ELEC_LOAD_cvr_POWERED").off(), network_alive()]),
            "deep::electrical's own CVR load reading unpowered while the network is live",
        )
        // FCOM PRO-ABN-ECAM p.5418: advisory (no aural, no master light) with this phase bar.
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
        .confirm(1.0),
    );
    v.push(
        proc(
            319_800_003,
            "RECORDER DFDR FAULT",
            Level::Advisory,
            sd_page::STATUS,
            all(vec![var("ELEC_LOAD_dfdr_POWERED").off(), network_alive()]),
            "deep::electrical's own DFDR load reading unpowered while the network is live",
        )
        // FCOM PRO-ABN-ECAM p.5419: advisory (no aural, no master light) with this phase bar.
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
        .confirm(1.0),
    );

    // ---- CABIN EMER EXIT LT FAULT. Both emergency-lighting-battery charger
    // circuits unpowered together while the network is live -- one charger
    // down alone still leaves that side's battery pack serviced.
    v.push(
        proc(
            334_800_101,
            "CABIN EMER EXIT LT FAULT",
            Level::Advisory,
            sd_page::STATUS,
            all(vec![
                var("ELEC_LOAD_emer-lighting-charger-1_POWERED").off(),
                var("ELEC_LOAD_emer-lighting-charger-2_POWERED").off(),
                network_alive(),
            ]),
            "deep::electrical's own pair of emergency-lighting-battery charger loads both reading unpowered while the network is live",
        )
        // FCOM PRO-ABN-ECAM p.5541: advisory (no aural, no master light) with this phase bar.
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10])
        .confirm(1.0),
    );

    // =========================================================================
    // `E-IND-DESIGN.md`'s Phase 2 pass, continued: the rest of the CDS/KCCU/
    // HUD/video/recorder family and the two FWS-database ids. Levels and
    // inhibits below cite `E-IND-FCOM.json` (E:/fbw-debug/ecam/E-IND-FCOM.json),
    // this group's Task A mapping against the A380 FCOM's own
    // PRO-ABN-ECAM chapter. Every entry's title is amber (`\x1b<4m`) in
    // FlyByWire's own catalogue, so `Level::Advisory` or `Level::Caution` are
    // both within `no_entry_is_louder_than_flybywires_own_title_colour`'s
    // ceiling (only `Level::Warning` would not be); the FCOM's own audio/
    // master-light indications (present = SC + MASTER CAUT = Caution; absent,
    // SD-page/status only = Advisory, FlyByWire's `Level::Advisory` = "amber,
    // no aural, no master light") pick between the two.
    // =========================================================================

    // ---- 311800001 CDS & AUTO FLT FCU SWITCHED OFF. `deep::
    // avionics_network` publishes `CDS_FCU_SWITCH_OFF` as a direct mirror of
    // `Truth::controls.fcu_switch_off` (no real dataref found for this port
    // yet). FCOM PRO-ABN-ECAM p.4797/5375: Audio+Master Light present ->
    // Caution; phases [2,3,4,5,6,7,9,10,11].
    v.push(
        proc(311_800_001, "CDS & AUTO FLT FCU SWITCHED OFF", Level::Caution, sd_page::STATUS, all(vec![var("CDS_FCU_SWITCH_OFF").on(), network_alive()]), "deep::avionics_network's direct mirror of Truth::controls.fcu_switch_off, FCOM PRO-ABN-ECAM p.4797")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 9, 10, 11]),
    );

    // ---- 311800002/003 CDS CAPT/F/O EFIS BKUP CTL FAULT, 311800004/005 CDS
    // CAPT/F/O EFIS CTL PNL FAULT, 311800006 the combined procedure. No FCOM
    // title match was found for the four single-side ids (the 2011 revision
    // may simply not carry them; `status: absent` in the mapping); the
    // combined `311800006` matches at p.5391 with phases [4,5,6,7,10] and no
    // Audio/Master Light listed... note: the combined one *does* show
    // Audio+Master Light per E-IND-FCOM.json's own render (Caution). The
    // four singles reuse the combined procedure's own phase window (same
    // physical family, no sourced figure of their own) and, following this
    // chapter's own observed pattern -- an isolated single-channel/backup
    // fault stays Advisory while the *combined*, both-channels-gone
    // condition is what actually reaches Caution (confirmed directly by the
    // BRAKES/STEER CTL-1(2)-vs-combined pairs later in this file's sibling
    // `ata32.rs`) -- are wired Advisory.
    let capt_efis_bkup_ctl_dead = || var("ELEC_LOAD_capt-efis-bkup-ctl_POWERED").off();
    let fo_efis_bkup_ctl_dead = || var("ELEC_LOAD_fo-efis-bkup-ctl_POWERED").off();
    let capt_efis_ctl_panel_dead = || var("ELEC_LOAD_capt-efis-ctl-panel_POWERED").off();
    let fo_efis_ctl_panel_dead = || var("ELEC_LOAD_fo-efis-ctl-panel_POWERED").off();
    v.push(
        proc(311_800_002, "CDS CAPT EFIS BKUP CTL FAULT", Level::Advisory, sd_page::STATUS, all(vec![capt_efis_bkup_ctl_dead(), network_alive()]), "deep::electrical's new capt-efis-bkup-ctl load unpowered; no FCOM title match, wired Advisory as an isolated single-side fault (E-IND-FCOM.json: absent)")
            .confirm(1.0)
            .inhibit(&[4, 5, 6, 7, 10]),
    );
    v.push(
        proc(311_800_003, "CDS F/O EFIS BKUP CTL FAULT", Level::Advisory, sd_page::STATUS, all(vec![fo_efis_bkup_ctl_dead(), network_alive()]), "as 311800002, F/O side")
            .confirm(1.0)
            .inhibit(&[4, 5, 6, 7, 10]),
    );
    v.push(
        proc(311_800_004, "CDS CAPT EFIS CTL PNL FAULT", Level::Advisory, sd_page::STATUS, all(vec![capt_efis_ctl_panel_dead(), network_alive()]), "deep::electrical's new capt-efis-ctl-panel load unpowered; no FCOM title match, wired Advisory as an isolated single-side fault")
            .confirm(1.0)
            .inhibit(&[4, 5, 6, 7, 10])
            .suppressed_by(&[311_800_006]),
    );
    v.push(
        proc(311_800_005, "CDS F/O EFIS CTL PNL FAULT", Level::Advisory, sd_page::STATUS, all(vec![fo_efis_ctl_panel_dead(), network_alive()]), "as 311800004, F/O side")
            .confirm(1.0)
            .inhibit(&[4, 5, 6, 7, 10])
            .suppressed_by(&[311_800_006]),
    );
    v.push(
        proc(
            311_800_006,
            "CDS CAPT+F/O EFIS CTL PNLs FAULT",
            Level::Caution,
            sd_page::STATUS,
            all(vec![capt_efis_ctl_panel_dead(), fo_efis_ctl_panel_dead(), network_alive()]),
            "both EFIS control panel loads unpowered together; FCOM PRO-ABN-ECAM p.5391, phases [4,5,6,7,10]",
        )
        .confirm(1.0)
        .inhibit(&[4, 5, 6, 7, 10]),
    );

    // ---- 311800007..311800011 CDS DU NOT MONITORED family. No FCOM title
    // match for any of the five ids (E-IND-FCOM.json: absent); the module's
    // own default inhibit is kept since no sourced figure exists to replace
    // it with, and the same single-vs-combined pattern as the EFIS panels
    // above sets Advisory for the three singles and Caution for the two
    // combined ids.
    let capt_pfd_du_dead = || var("ELEC_LOAD_capt-pfd-du_POWERED").off();
    let fo_pfd_du_dead = || var("ELEC_LOAD_fo-pfd-du_POWERED").off();
    let capt_nd_du_dead = || var("ELEC_LOAD_capt-nd-du_POWERED").off();
    let fo_nd_du_dead = || var("ELEC_LOAD_fo-nd-du_POWERED").off();
    v.push(
        proc(311_800_007, "CDS CAPT PFD DU NOT MONITORED", Level::Advisory, sd_page::STATUS, all(vec![capt_pfd_du_dead(), network_alive()]), "deep::electrical's new capt-pfd-du load unpowered; power loss is a sound subset of 'not monitored' -- no FCOM title match, wired Advisory as an isolated single-side condition")
            .confirm(1.0),
    );
    v.push(
        proc(311_800_008, "CDS CAPT ND DU NOT MONITORED", Level::Advisory, sd_page::STATUS, all(vec![capt_nd_du_dead(), network_alive()]), "as 311800007, ND")
            .confirm(1.0),
    );
    v.push(
        proc(311_800_009, "CDS CAPT EWD DU NOT MONITORED", Level::Advisory, sd_page::STATUS, all(vec![var("ELEC_LOAD_capt-ewd-du_POWERED").off(), network_alive()]), "deep::electrical's new capt-ewd-du load unpowered; as 311800007")
            .confirm(1.0),
    );
    v.push(
        proc(
            311_800_010,
            "CDS CAPT F/O PFD DU NOT MONITORED",
            Level::Caution,
            sd_page::STATUS,
            all(vec![capt_pfd_du_dead(), fo_pfd_du_dead(), network_alive()]),
            "both PFD DU loads unpowered together; combined condition, wired Caution matching this chapter's own single-vs-combined pattern",
        )
        .confirm(1.0),
    );
    v.push(
        proc(
            311_800_011,
            "CDS CAPT F/O ND DU NOT MONITORED",
            Level::Caution,
            sd_page::STATUS,
            all(vec![capt_nd_du_dead(), fo_nd_du_dead(), network_alive()]),
            "both ND DU loads unpowered together; as 311800010",
        )
        .confirm(1.0),
    );

    // ---- 313800003/004 CDS CAPT/F/O CURSOR CTL+KEYBOARD FAULT (whole-unit
    // KCCU loss) and 313800007 CDS CAPT MAILBOX ACCESS FAULT. No FCOM title
    // match for any of the seven `313800xxx` ids; the two whole-unit KCCU
    // faults are wired Caution (a combined, both-functions-gone condition,
    // matching this chapter's own pattern), the single mailbox peripheral
    // Advisory (an isolated single-LRU condition).
    //
    // `311800012` CDS DISPLAY DISAGREE, FCOM p.5393: Audio and MASTER CAUT,
    // default phases. Any modelled display unit whose image disagrees with
    // its monitor copy while it is powered.
    v.push(
        proc(
            311_800_012,
            "CDS DISPLAY DISAGREE",
            Level::Caution,
            sd_page::STATUS,
            all(vec![
                any(crate::deep::sensors::live::CDS_MONITORED_DUS
                    .iter()
                    .map(|du| all(vec![var(&format!("ELEC_LOAD_{du}_POWERED")).on(), var(&crate::deep::sensors::live::cds_monitor_var(du)).on()]))
                    .collect()),
                network_alive(),
            ]),
            "FCOM PRO-ABN-ECAM p.5393: the CDS finds a discrepancy between a display unit and the unit monitoring it",
        )
        .confirm(1.0),
    );

    // KCCU, FCOM p.5377/5378/5383: no aural, no master light (Level 1),
    // default phases. A part's own BITE fires its single alert while the
    // unit is powered; the unit lost, or both parts failed, is the
    // combined CURSOR CTL+KEYBOARD alert, which supersedes the singles.
    for (side, ccd_id, keyboard_id, both_id) in [("capt", 313_800_001, 313_800_005, 313_800_003), ("fo", 313_800_002, 313_800_006, 313_800_004)] {
        let powered = || var(&format!("ELEC_LOAD_kccu-{side}_POWERED")).on();
        let part = |p: &str| var(&format!("DEEP_KCCU_{}_{p}_FAILED", side.to_ascii_uppercase()));
        let (ccd_title, keyboard_title, both_title) = if side == "capt" {
            ("CDS CAPT CURSOR CTL FAULT", "CDS CAPT KEYBOARD FAULT", "CDS CAPT CURSOR CTL+KEYBOARD FAULT")
        } else {
            ("CDS F/O CURSOR CTL FAULT", "CDS F/O KEYBOARD FAULT", "CDS F/O CURSOR CTL+KEYBOARD FAULT")
        };
        let supersede: &'static [u64] = if side == "capt" { &[313_800_003] } else { &[313_800_004] };
        v.push(
            proc(ccd_id, ccd_title, Level::Advisory, sd_page::STATUS, all(vec![powered(), part("CCD").on(), network_alive()]), "FCOM PRO-ABN-ECAM p.5377: the Cursor Control Device is failed (the KCCU's own BITE, deep::sensors)")
                .confirm(1.0)
                .suppressed_by(supersede),
        );
        v.push(
            proc(keyboard_id, keyboard_title, Level::Advisory, sd_page::STATUS, all(vec![powered(), part("KEYBOARD").on(), network_alive()]), "FCOM PRO-ABN-ECAM p.5383: the Keyboard is failed (the KCCU's own BITE, deep::sensors)")
                .confirm(1.0)
                .suppressed_by(supersede),
        );
        v.push(
            proc(
                both_id,
                both_title,
                Level::Advisory,
                sd_page::STATUS,
                all(vec![any(vec![var(&format!("ELEC_LOAD_kccu-{side}_POWERED")).off(), all(vec![part("CCD").on(), part("KEYBOARD").on()])]), network_alive()]),
                "FCOM PRO-ABN-ECAM p.5378: the KCCU is failed -- deep::electrical's kccu load unpowered, or both of its parts failed",
            )
            .confirm(1.0),
        );
    }
    v.push(
        proc(313_800_007, "CDS CAPT MAILBOX ACCESS FAULT", Level::Advisory, sd_page::STATUS, all(vec![var("ELEC_LOAD_cds-mailbox-capt_POWERED").off(), network_alive()]), "deep::electrical's new cds-mailbox-capt load unpowered; no FCOM title match, wired Advisory as an isolated single-peripheral condition")
            .confirm(1.0),
    );

    // ---- 314800001/005 FWS AIRLINE CUSTOMIZATION / ATQC DATABASE REJECTED.
    // No FCOM title match for either id, but every one of this family's own
    // FCOM-confirmed siblings in this file (314800004 FWS 1+2 FAULT,
    // p.5405; 314800007 ECP FAULT, p.5410; 314800008/009 FWS 1/2 FAULT,
    // p.5412 -- all `E-IND-FCOM.json: matched`) shows no Audio/Master Light
    // at all (SD-page/status only, Level::Advisory) and, for 314800004
    // specifically, an *empty* inhibited-phase list -- never inhibited. Both
    // new database-check ids are wired the same way as their whole family.
    v.push(
        proc(
            314_800_001,
            "FWS AIRLINE CUSTOMIZATION REJECTED",
            Level::Advisory,
            sd_page::STATUS,
            all(vec![var("AVNCS_MODULE_CPIOM_C1_CUSTOMIZATION_DB_REJECTED").on(), network_alive()]),
            "deep::avionics_network's new customization_db_rejected BITE flag on the CPIOM-C1 FWS partition; no FCOM title match, level/inhibit taken from this family's own FCOM-confirmed siblings (314800004/007/008/009)",
        )
        .confirm(1.0)
        .inhibit(phase::NONE),
    );
    v.push(
        proc(
            314_800_005,
            "FWS ATQC DATABASE REJECTED",
            Level::Advisory,
            sd_page::STATUS,
            all(vec![var("AVNCS_MODULE_CPIOM_C1_ATQC_DB_REJECTED").on(), network_alive()]),
            "as 314800001, the ATQC database check",
        )
        .confirm(1.0)
        .inhibit(phase::NONE),
    );

    // ---- 316800001 NAV HUD FAULT. FCOM PRO-ABN-ECAM p.5415: SD-page/status
    // only (Advisory), phases [3,4,5,6,7,10].
    v.push(
        proc(316_800_001, "NAV HUD FAULT", Level::Advisory, sd_page::STATUS, all(vec![var("ELEC_LOAD_hud_POWERED").off(), network_alive()]), "deep::electrical's new hud load unpowered; FCOM PRO-ABN-ECAM p.5415")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 10]),
    );
    // ---- 316800002 NAV HUD FPV DISAGREE. FCOM p.5416: MASTER CAUT, phases
    // [3,4,5,6,7,10]; the threshold is DSC-31-60-20's CHECK FPV, the two
    // selected IRS' flight path angles more than 1 deg apart.
    v.push(
        proc(
            316_800_002,
            "NAV HUD FPV DISAGREE",
            Level::Caution,
            sd_page::STATUS,
            all(vec![var("ELEC_LOAD_hud_POWERED").on(), var("DEEP_IR_CAPT_FO_FPA_DIFF_DEG").gt(1.0), network_alive()]),
            "FCOM PRO-ABN-ECAM p.5416 / DSC-31-60-20: the CAPT and F/O side IRs' flight path angles differ by more than 1 deg",
        )
        .confirm(1.0)
        .inhibit(&[3, 4, 5, 6, 7, 10]),
    );

    // ---- 318800001 VIDEO MULTIPLEXER FAULT. FCOM p.5421: SD-page/status
    // only (Advisory), phases [3,4,5,6,7,9,10].
    v.push(
        proc(318_800_001, "VIDEO MULTIPLEXER FAULT", Level::Advisory, sd_page::STATUS, all(vec![var("ELEC_LOAD_video-multiplexer_POWERED").off(), network_alive()]), "deep::electrical's new video-multiplexer load unpowered; FCOM PRO-ABN-ECAM p.5421")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]),
    );

    // ---- 319800001 RECORDER ACCELMTR FAULT, 319800004 RECORDER SYS FAULT
    // (the DFDAU). FCOM p.5417/5420: SD-page/status only (Advisory), phases
    // [2,3,4,5,6,7,8,9,10,11] / [3,4,5,6,7,8,9,10,11] respectively -- the
    // same family as the already-wired 319800002/003 above.
    v.push(
        proc(319_800_001, "RECORDER ACCELMTR FAULT", Level::Advisory, sd_page::STATUS, all(vec![var("ELEC_LOAD_recorder-accelerometer_POWERED").off(), network_alive()]), "deep::electrical's new recorder-accelerometer load unpowered, joining ata31_recorders; FCOM PRO-ABN-ECAM p.5417")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11]),
    );
    v.push(
        proc(
            319_800_004,
            "RECORDER SYS FAULT",
            Level::Advisory,
            sd_page::STATUS,
            all(vec![var("ELEC_LOAD_dfdau_POWERED").off(), network_alive()]),
            "deep::electrical's new dfdau load unpowered -- the real, distinct DFDAU LRU this id names, not the recorder system 'as a whole'; FCOM PRO-ABN-ECAM p.5420",
        )
        .confirm(1.0)
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11]),
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::ecam::fbw;
    use crate::deep::integration::failure_audit::bare;
    use crate::deep::live::{Faults, Truth};

    fn run(truth: Truth, faults: &Faults, frames: usize) -> std::collections::BTreeMap<String, f64> {
        let mut deep = crate::deep::integration::failure_audit::fresh_areas();
        let mut out = std::collections::BTreeMap::new();
        for _ in 0..frames {
            out.clear();
            deep.tick(truth.clone(), faults, &mut |n, v| {
                out.insert(bare(n).to_owned(), v);
            });
        }
        out
    }

    fn holds(c: &Cond, published: &std::collections::BTreeMap<String, f64>) -> bool {
        c.eval(&|n: &str| *published.get(bare(n)).unwrap_or(&0.0))
    }

    fn wiring(id: u64) -> FbwProc {
        fbw::wirings().into_iter().find(|p| p.id == id).unwrap_or_else(|| panic!("{id} is not wired"))
    }

    /// A failure id from the combined registry, found by a predicate over
    /// its component and model field rather than hard-coded, so this fails
    /// loudly if an area renames one instead of silently arming nothing.
    fn failure_id(pred: impl Fn(&crate::deep::api::FailureDef) -> bool, what: &str) -> u64 {
        let r = crate::deep::registry();
        let hits: Vec<u64> = r.failures.iter().filter(|f| pred(f)).map(|f| f.id).collect();
        assert!(!hits.is_empty(), "no registered failure matches {what}");
        hits[0]
    }

    fn flying() -> Truth {
        // `deep::avionics_network` does not read its module power from
        // `deep::electrical`'s own simulated bus network -- its own doc
        // comment (`avionics_network/live.rs:45-54`) says `Truth`'s
        // `ac_bus_volts`/`dc_bus_volts` are a still-missing load-allocation
        // seam, so CPIOM-C1 (and every other module) reads its power
        // straight off those two raw `Truth` fields. They default to 0 V,
        // so without setting them here the FWS partition reads permanently
        // unavailable even on a running aircraft, healthy or not. Set to
        // nominal so this fixture is unambiguously "the aircraft has power".
        Truth {
            dt_s: 0.05,
            on_ground: false,
            engine_running: [true; 4],
            engine_n1_frac: [0.9; 4],
            engine_n2_frac: [0.9; 4],
            engine_n3_frac: [0.9; 4],
            ac_bus_volts: [115.0; 4],
            dc_bus_volts: [28.0; 2],
            ..Truth::default()
        }
    }

    #[test]
    fn fws_and_fcdc_faults_need_the_network_to_be_alive() {
        // Cold and dark: the FWS partition and both FCDCs are unpowered
        // simply because nothing is generating, and none of that is a
        // fault. The network-alive gate is what keeps both procedures off
        // the flight deck at the gate.
        let cold = run(Truth { dt_s: 0.1, ..Truth::default() }, &Faults::default(), 30);
        assert!(!holds(&wiring(314_800_004).trigger, &cold), "FWS 1+2 FAULT fired on a cold and dark aircraft");
        assert!(!holds(&wiring(314_800_003).trigger, &cold), "FWS 1+2 & FCDC 1+2 FAULT fired on a cold and dark aircraft");
    }

    #[test]
    fn fws_1plus2_fault_fires_when_the_cpiom_c1_fws_partition_dies() {
        let index = crate::deep::avionics_network::live::FaultIndex::build();
        let id = index.id("CPIOM-C1 partition FWS failure");
        assert_ne!(id, 0, "deep::avionics_network registers a CPIOM-C1 FWS partition failure");

        let healthy = run(flying(), &Faults::default(), 20);
        let armed = run(flying(), &Faults::from_pairs([(id, 1.0)]), 20);

        let fws = wiring(314_800_004);
        assert!(!holds(&fws.trigger, &healthy), "FWS 1+2 FAULT must be quiet with the partition healthy");
        assert!(holds(&fws.trigger, &armed), "FWS 1+2 FAULT must fire when the CPIOM-C1 FWS partition dies");
        // The FCDCs are untouched, so the combined procedure must stay down
        // and the plain one must not be suppressed by it.
        assert!(!holds(&wiring(314_800_003).trigger, &armed), "FWS 1+2 & FCDC 1+2 FAULT must stay quiet when only the FWS partition died");
    }

    #[test]
    fn fws_and_fcdc_combined_fault_needs_both_fcdcs_dead_too() {
        // Each FCDC is dual-fed (`electrical/loads.rs:353-354`, two feed
        // breakers OR-ed together): tripping one of its two *breakers* --
        // `17_breakers.fcdc-1-normal-bkr`/`-2nd-bkr` -- proves nothing, since
        // the surviving feed just keeps the load powered, and even arming
        // both is not reliable through the breakers' own I^2t drift element
        // (it only opens a breaker that is already carrying most of its own
        // rated current, which is a fact about that specific breaker's
        // sizing, not a lever this test can pull). What genuinely represents
        // "this LRU has no power" regardless of which feed it would have
        // used is the *load's own* `open_circuit` fault
        // (`deep::electrical::network::Load.faults.open_circuit`, `network.rs`
        // `health()`/`step()`: at 1.0 the load's own delivered power is
        // forced to zero, so `powered = energised && p > 0.0` reads false on
        // every feed at once) -- registered once per catalogue load as
        // `deep::electrical::registry.rs`'s `LOAD_CHANNELS`, component
        // `27_elec.fcdc-1`/`27_elec.fcdc-2`.
        let index = crate::deep::avionics_network::live::FaultIndex::build();
        let fws_id = index.id("CPIOM-C1 partition FWS failure");
        assert_ne!(fws_id, 0);
        let fcdc1 = failure_id(|f| f.component == "27_elec.fcdc-1" && f.model_field.contains("open_circuit"), "FCDC 1's own load open-circuit fault");
        let fcdc2 = failure_id(|f| f.component == "27_elec.fcdc-2" && f.model_field.contains("open_circuit"), "FCDC 2's own load open-circuit fault");

        let both_dead = run(flying(), &Faults::from_pairs([(fws_id, 1.0), (fcdc1, 1.0), (fcdc2, 1.0)]), 30);
        let only_fws = run(flying(), &Faults::from_pairs([(fws_id, 1.0)]), 30);

        assert!(holds(&wiring(314_800_003).trigger, &both_dead), "FWS 1+2 & FCDC 1+2 FAULT must fire once the FWS partition and both FCDCs are down");
        assert!(!holds(&wiring(314_800_003).trigger, &only_fws), "and must stay quiet with the FCDCs still powered");
    }

    #[test]
    fn recorder_faults_fire_on_their_own_load_and_not_on_a_cold_aircraft() {
        // Same reasoning as the FCDC test above: CVR/DFDR are single-fed at
        // a comfortable current margin under their own breaker's rating
        // (`deep::breakers::catalog`'s `standard_size` rounds up), so a
        // `trip_calibration_drift` fault on `17_breakers.cvr`/`dfdr` never
        // actually opens either breaker -- confirmed by instrumenting the
        // live current, which never moved past ~1.75 A against a 3.0 A
        // rating even fully drifted for 120 s. The load's own
        // `open_circuit` fault is what genuinely and immediately de-powers
        // it regardless of breaker headroom.
        let cvr = failure_id(|f| f.component == "31_elec.cvr" && f.model_field.contains("open_circuit"), "the CVR load's own open-circuit fault");
        let dfdr = failure_id(|f| f.component == "31_elec.dfdr" && f.model_field.contains("open_circuit"), "the DFDR load's own open-circuit fault");

        let cold = run(Truth { dt_s: 0.1, ..Truth::default() }, &Faults::default(), 30);
        assert!(!holds(&wiring(319_800_002).trigger, &cold), "RECORDER CVR FAULT fired on a cold and dark aircraft");
        assert!(!holds(&wiring(319_800_003).trigger, &cold), "RECORDER DFDR FAULT fired on a cold and dark aircraft");

        let healthy = run(flying(), &Faults::default(), 30);
        let cvr_dead = run(flying(), &Faults::from_pairs([(cvr, 1.0)]), 30);
        let dfdr_dead = run(flying(), &Faults::from_pairs([(dfdr, 1.0)]), 30);

        assert!(!holds(&wiring(319_800_002).trigger, &healthy) && !holds(&wiring(319_800_003).trigger, &healthy), "both recorder procedures must be quiet with both recorders healthy");
        assert!(holds(&wiring(319_800_002).trigger, &cvr_dead), "RECORDER CVR FAULT must fire when the CVR's own load loses power");
        assert!(!holds(&wiring(319_800_003).trigger, &cvr_dead), "and DFDR must stay quiet when only the CVR died");
        assert!(holds(&wiring(319_800_003).trigger, &dfdr_dead), "RECORDER DFDR FAULT must fire when the DFDR's own load loses power");
        assert!(!holds(&wiring(319_800_002).trigger, &dfdr_dead), "and CVR must stay quiet when only the DFDR died");
    }

    #[test]
    fn cabin_emer_exit_lt_fault_needs_both_chargers_dead_and_not_a_cold_aircraft() {
        let c1 = failure_id(|f| f.component == "17_breakers.emer-lighting-charger-1" && f.model_field.contains("trip_calibration_drift"), "emergency lighting charger 1's breaker calibration drift");
        let c2 = failure_id(|f| f.component == "17_breakers.emer-lighting-charger-2" && f.model_field.contains("trip_calibration_drift"), "emergency lighting charger 2's breaker calibration drift");

        let cold = run(Truth { dt_s: 0.1, ..Truth::default() }, &Faults::default(), 30);
        assert!(!holds(&wiring(334_800_101).trigger, &cold), "CABIN EMER EXIT LT FAULT fired on a cold and dark aircraft");

        let healthy = run(flying(), &Faults::default(), 20);
        let one_dead = run(flying(), &Faults::from_pairs([(c1, 1.0)]), 2_400);
        let both_dead = run(flying(), &Faults::from_pairs([(c1, 1.0), (c2, 1.0)]), 2_400);

        assert!(!holds(&wiring(334_800_101).trigger, &healthy), "must be quiet with both chargers healthy");
        assert!(!holds(&wiring(334_800_101).trigger, &one_dead), "one charger down must still leave the other pack serviced");
        assert!(holds(&wiring(334_800_101).trigger, &both_dead), "must fire once both chargers are dead");
    }

    /// Each wired id's inhibit window: the default where the FCOM agrees,
    /// the FCOM's own phases where it does not.
    #[test]
    fn wired_ids_keep_the_default_takeoff_and_landing_inhibit() {
        assert_eq!(wiring(314_800_003).inhibit, phase::TAKEOFF_AND_LANDING);
        // CVR / DFDR FAULT: FCOM PRO-ABN-ECAM p.5418 / p.5419 inhibit
        // phases 3 to 11. EMER EXIT LT: p.5541 inhibits 3 to 10.
        for id in [319_800_002, 319_800_003] {
            assert_eq!(wiring(id).inhibit, &[3, 4, 5, 6, 7, 8, 9, 10, 11]);
        }
        assert_eq!(wiring(334_800_101).inhibit, &[3, 4, 5, 6, 7, 8, 9, 10]);
        // `314_800_004` moved to `phase::NONE` in the Phase 2 FCOM pass:
        // FCOM PRO-ABN-ECAM p.5405 shows this procedure with an empty
        // inhibited-phase list -- never inhibited.
        assert_eq!(wiring(314_800_004).inhibit, phase::NONE);
    }

    // -------------------------------------------------------------------
    // `E-IND-DESIGN.md`'s Phase 2 pass: the rest of ATA 31.
    // -------------------------------------------------------------------

    fn load_open_circuit(component: &str) -> u64 {
        failure_id(|f| f.component == component && f.model_field.contains("open_circuit"), component)
    }

    #[test]
    fn every_new_ata31_id_is_silent_cold_and_dark() {
        let cold = run(Truth { dt_s: 0.1, ..Truth::default() }, &Faults::default(), 30);
        for id in [
            311_800_001, 311_800_002, 311_800_003, 311_800_004, 311_800_005, 311_800_006, 311_800_007, 311_800_008, 311_800_009, 311_800_010, 311_800_011, 311_800_012, 313_800_001, 313_800_002,
            313_800_003, 313_800_004, 313_800_005, 313_800_006, 313_800_007, 314_800_001, 314_800_005, 316_800_001, 316_800_002, 318_800_001, 319_800_001, 319_800_004,
        ] {
            assert!(!holds(&wiring(id).trigger, &cold), "{id} fired on a cold and dark aircraft");
        }
    }

    #[test]
    fn cds_fcu_switch_off_fires_only_when_the_switch_is_off_and_the_network_is_alive() {
        let mut off = flying();
        off.controls.fcu_switch_off = true;
        let on = run(flying(), &Faults::default(), 5);
        let switched_off = run(off, &Faults::default(), 5);
        assert!(!holds(&wiring(311_800_001).trigger, &on));
        assert!(holds(&wiring(311_800_001).trigger, &switched_off));
    }

    #[test]
    fn efis_backup_and_ctl_panel_faults_fire_singly_and_combine() {
        let capt_bkup = load_open_circuit("31_elec.capt-efis-bkup-ctl");
        let fo_bkup = load_open_circuit("31_elec.fo-efis-bkup-ctl");
        let capt_panel = load_open_circuit("31_elec.capt-efis-ctl-panel");
        let fo_panel = load_open_circuit("31_elec.fo-efis-ctl-panel");

        assert!(holds(&wiring(311_800_002).trigger, &run(flying(), &Faults::from_pairs([(capt_bkup, 1.0)]), 5)));
        assert!(holds(&wiring(311_800_003).trigger, &run(flying(), &Faults::from_pairs([(fo_bkup, 1.0)]), 5)));

        let capt_only = run(flying(), &Faults::from_pairs([(capt_panel, 1.0)]), 5);
        assert!(holds(&wiring(311_800_004).trigger, &capt_only));
        assert!(!holds(&wiring(311_800_006).trigger, &capt_only), "one panel alone must not raise the combined procedure");

        let both = run(flying(), &Faults::from_pairs([(capt_panel, 1.0), (fo_panel, 1.0)]), 5);
        assert!(holds(&wiring(311_800_006).trigger, &both));
    }

    #[test]
    fn du_not_monitored_faults_fire_singly_and_combine() {
        let capt_pfd = load_open_circuit("31_elec.capt-pfd-du");
        let fo_pfd = load_open_circuit("31_elec.fo-pfd-du");
        let capt_nd = load_open_circuit("31_elec.capt-nd-du");
        let capt_ewd = load_open_circuit("31_elec.capt-ewd-du");

        assert!(holds(&wiring(311_800_007).trigger, &run(flying(), &Faults::from_pairs([(capt_pfd, 1.0)]), 5)));
        assert!(holds(&wiring(311_800_008).trigger, &run(flying(), &Faults::from_pairs([(capt_nd, 1.0)]), 5)));
        assert!(holds(&wiring(311_800_009).trigger, &run(flying(), &Faults::from_pairs([(capt_ewd, 1.0)]), 5)));

        let capt_only = run(flying(), &Faults::from_pairs([(capt_pfd, 1.0)]), 5);
        assert!(!holds(&wiring(311_800_010).trigger, &capt_only), "one PFD DU alone must not raise the combined procedure");
        let both = run(flying(), &Faults::from_pairs([(capt_pfd, 1.0), (fo_pfd, 1.0)]), 5);
        assert!(holds(&wiring(311_800_010).trigger, &both));
    }

    #[test]
    fn kccu_whole_unit_and_mailbox_faults_fire_on_their_own_load() {
        let kccu_capt = load_open_circuit("31_elec.kccu-capt");
        let kccu_fo = load_open_circuit("31_elec.kccu-fo");
        let mailbox = load_open_circuit("31_elec.cds-mailbox-capt");
        assert!(holds(&wiring(313_800_003).trigger, &run(flying(), &Faults::from_pairs([(kccu_capt, 1.0)]), 5)));
        assert!(holds(&wiring(313_800_004).trigger, &run(flying(), &Faults::from_pairs([(kccu_fo, 1.0)]), 5)));
        assert!(holds(&wiring(313_800_007).trigger, &run(flying(), &Faults::from_pairs([(mailbox, 1.0)]), 5)));
    }

    fn part_failure(component: &str, field: &str) -> u64 {
        failure_id(|f| f.component == component && f.model_field.ends_with(&format!("(faults.{field})")), component)
    }

    #[test]
    fn a_kccu_part_fires_its_own_alert_and_both_parts_fire_the_combined_one() {
        let healthy = run(flying(), &Faults::default(), 5);
        let ccd = part_failure("31_elec.kccu-capt", "ccd_failed");
        let keyboard = part_failure("31_elec.kccu-fo", "keyboard_failed");
        let ccd_only = run(flying(), &Faults::from_pairs([(ccd, 1.0)]), 5);
        let keyboard_only = run(flying(), &Faults::from_pairs([(keyboard, 1.0)]), 5);
        for id in [313_800_001, 313_800_002, 313_800_003, 313_800_004, 313_800_005, 313_800_006] {
            assert!(!holds(&wiring(id).trigger, &healthy), "{id} fired on a healthy aircraft");
        }
        assert!(holds(&wiring(313_800_001).trigger, &ccd_only));
        assert!(!holds(&wiring(313_800_005).trigger, &ccd_only), "the keyboard is independent of the cursor device");
        assert!(!holds(&wiring(313_800_003).trigger, &ccd_only));
        assert!(holds(&wiring(313_800_006).trigger, &keyboard_only));
        assert!(!holds(&wiring(313_800_002).trigger, &keyboard_only));

        let capt_keyboard = part_failure("31_elec.kccu-capt", "keyboard_failed");
        let both = run(flying(), &Faults::from_pairs([(ccd, 1.0), (capt_keyboard, 1.0)]), 5);
        assert!(holds(&wiring(313_800_003).trigger, &both));
        assert_eq!(wiring(313_800_001).suppressed_by, &[313_800_003]);
        assert_eq!(wiring(313_800_005).suppressed_by, &[313_800_003]);

        // An unpowered KCCU is the whole-unit alert, not a part's.
        let unpowered = run(flying(), &Faults::from_pairs([(ccd, 1.0), (load_open_circuit("31_elec.kccu-capt"), 1.0)]), 5);
        assert!(!holds(&wiring(313_800_001).trigger, &unpowered));
        assert!(holds(&wiring(313_800_003).trigger, &unpowered));
    }

    #[test]
    fn every_monitored_display_unit_raises_display_disagree() {
        assert!(!holds(&wiring(311_800_012).trigger, &run(flying(), &Faults::default(), 5)));
        for du in crate::deep::sensors::live::CDS_MONITORED_DUS {
            let id = part_failure(&format!("31_elec.{du}"), "display_monitor_disagree");
            assert!(holds(&wiring(311_800_012).trigger, &run(flying(), &Faults::from_pairs([(id, 1.0)]), 5)), "{du}");
        }
    }

    /// FCOM DSC-31-60-20: CHECK FPV past 1 deg between the two selected
    /// IRS -- IR 1 and IR 2, or IR 3 in place of one with the ATT HDG knob.
    #[test]
    fn hud_fpv_disagree_compares_the_two_selected_irs_flight_path_angles() {
        use crate::deep::live::IrOutputs;
        let with_fpa = |fpa: [Option<f64>; 3], knob: f64| {
            let mut t = flying();
            for (ir, f) in t.ir.iter_mut().zip(fpa) {
                *ir = IrOutputs { pitch_deg: Some(2.0), roll_deg: Some(0.0), true_heading_deg: Some(90.0), flight_path_angle_deg: f };
            }
            t.att_hdg_switching_knob = knob;
            run(t, &Faults::default(), 5)
        };
        let fires = |p| holds(&wiring(316_800_002).trigger, &p);
        assert!(!fires(with_fpa([Some(3.0), Some(2.4), Some(3.0)], 1.0)), "0.6 deg is inside CHECK FPV's 1 deg");
        assert!(fires(with_fpa([Some(3.0), Some(1.5), Some(3.0)], 1.0)), "1.5 deg between IR 1 and IR 2");
        assert!(!fires(with_fpa([Some(3.0), None, Some(3.0)], 1.0)), "an invalid IR flags its FPA instead");
        assert!(!fires(with_fpa([Some(0.0), Some(3.0), Some(3.0)], 0.0)), "CAPT ON 3 compares IR 3 with IR 2");
        assert!(fires(with_fpa([Some(3.0), Some(3.0), Some(0.0)], 2.0)), "F/O ON 3 compares IR 1 with IR 3");
    }

    /// The same selected pair drives NAV CAPT AND F/O ATT DISAGREE, FCOM
    /// p.5570's pitch or roll past 5 deg.
    #[test]
    fn att_disagree_fires_on_pitch_or_roll_between_the_selected_irs() {
        use crate::deep::live::IrOutputs;
        let with = |capt: (f64, f64), fo: (f64, f64)| {
            let mut t = flying();
            let ir = |(pitch, roll): (f64, f64)| IrOutputs { pitch_deg: Some(pitch), roll_deg: Some(roll), true_heading_deg: Some(90.0), flight_path_angle_deg: Some(0.0) };
            t.ir = [ir(capt), ir(fo), ir(fo)];
            run(t, &Faults::default(), 5)
        };
        let fires = |p| holds(&wiring(340_800_017).trigger, &p);
        assert!(!fires(with((2.0, 0.0), (2.5, 1.0))));
        assert!(fires(with((2.0, 0.0), (8.0, 0.0))), "pitch");
        assert!(fires(with((2.0, 178.0), (2.0, -176.0))), "roll, across the +-180 wrap");
        assert!(!fires(with((2.0, 179.0), (2.0, -179.0))), "2 deg across the wrap is not 358");
    }

    #[test]
    fn hud_and_video_multiplexer_and_recorder_faults_fire_on_their_own_load() {
        let hud = load_open_circuit("31_elec.hud");
        let video = load_open_circuit("31_elec.video-multiplexer");
        let accel = load_open_circuit("31_elec.recorder-accelerometer");
        let dfdau = load_open_circuit("31_elec.dfdau");
        assert!(holds(&wiring(316_800_001).trigger, &run(flying(), &Faults::from_pairs([(hud, 1.0)]), 5)));
        assert!(holds(&wiring(318_800_001).trigger, &run(flying(), &Faults::from_pairs([(video, 1.0)]), 5)));
        assert!(holds(&wiring(319_800_001).trigger, &run(flying(), &Faults::from_pairs([(accel, 1.0)]), 5)));
        assert!(holds(&wiring(319_800_004).trigger, &run(flying(), &Faults::from_pairs([(dfdau, 1.0)]), 5)));
    }

    #[test]
    fn fws_database_rejected_ids_fire_on_their_own_bite_flag() {
        let index = crate::deep::avionics_network::live::FaultIndex::build();
        let custom_id = index.id("CPIOM-C1 partition FWS customization database rejected");
        let atqc_id = index.id("CPIOM-C1 partition FWS ATQC database rejected");
        assert_ne!(custom_id, 0);
        assert_ne!(atqc_id, 0);

        let custom = run(flying(), &Faults::from_pairs([(custom_id, 1.0)]), 5);
        assert!(holds(&wiring(314_800_001).trigger, &custom));
        assert!(!holds(&wiring(314_800_005).trigger, &custom));

        let atqc = run(flying(), &Faults::from_pairs([(atqc_id, 1.0)]), 5);
        assert!(holds(&wiring(314_800_005).trigger, &atqc));
        assert!(!holds(&wiring(314_800_001).trigger, &atqc));

        // Never inhibited, per FCOM PRO-ABN-ECAM p.5405's own empty phase
        // list for this family's confirmed sibling, 314800004.
        assert_eq!(wiring(314_800_001).inhibit, phase::NONE);
        assert_eq!(wiring(314_800_005).inhibit, phase::NONE);
    }
}
