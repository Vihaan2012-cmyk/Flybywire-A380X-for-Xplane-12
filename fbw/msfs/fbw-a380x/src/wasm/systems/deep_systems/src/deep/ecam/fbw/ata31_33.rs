use super::{phase, proc, sd_page, FbwProc};
use crate::deep::api::{all, var, any, Cond, Level};

fn network_alive() -> Cond {
    any(vec![
        var("ELEC_AC_1_BUS_IS_POWERED").on(),
        var("ELEC_AC_2_BUS_IS_POWERED").on(),
        var("ELEC_AC_3_BUS_IS_POWERED").on(),
        var("ELEC_AC_4_BUS_IS_POWERED").on(),
    ])
}

fn fws_unavailable() -> Cond {
    var("AVNCS_MODULE_CPIOM_C1_PARTITION_FWS_AVAILABLE").off()
}

fn fcdc_dead(n: u32) -> Cond {
    var(&format!("ELEC_LOAD_fcdc-{n}_POWERED")).off()
}

pub fn wire(v: &mut Vec<FbwProc>) {
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
        .inhibit(phase::NONE)
        .suppressed_by(&[314_800_003])
        .items(9, Vec::new()),
    );

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

    v.push(
        proc(
            319_800_002,
            "RECORDER CVR FAULT",
            Level::Advisory,
            sd_page::STATUS,
            all(vec![var("ELEC_LOAD_cvr_POWERED").off(), network_alive()]),
            "deep::electrical's own CVR load reading unpowered while the network is live",
        )
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
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
        .confirm(1.0),
    );

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
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10])
        .confirm(1.0),
    );

    v.push(
        proc(311_800_001, "CDS & AUTO FLT FCU SWITCHED OFF", Level::Caution, sd_page::STATUS, all(vec![var("CDS_FCU_SWITCH_OFF").on(), network_alive()]), "deep::avionics_network's direct mirror of Truth::controls.fcu_switch_off, FCOM PRO-ABN-ECAM p.4797")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 9, 10, 11]),
    );

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

    v.push(
        proc(316_800_001, "NAV HUD FAULT", Level::Advisory, sd_page::STATUS, all(vec![var("ELEC_LOAD_hud_POWERED").off(), network_alive()]), "deep::electrical's new hud load unpowered; FCOM PRO-ABN-ECAM p.5415")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 10]),
    );
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

    v.push(
        proc(318_800_001, "VIDEO MULTIPLEXER FAULT", Level::Advisory, sd_page::STATUS, all(vec![var("ELEC_LOAD_video-multiplexer_POWERED").off(), network_alive()]), "deep::electrical's new video-multiplexer load unpowered; FCOM PRO-ABN-ECAM p.5421")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10]),
    );

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

    fn failure_id(pred: impl Fn(&crate::deep::api::FailureDef) -> bool, what: &str) -> u64 {
        let r = crate::deep::registry();
        let hits: Vec<u64> = r.failures.iter().filter(|f| pred(f)).map(|f| f.id).collect();
        assert!(!hits.is_empty(), "no registered failure matches {what}");
        hits[0]
    }

    fn flying() -> Truth {
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
        assert!(!holds(&wiring(314_800_003).trigger, &armed), "FWS 1+2 & FCDC 1+2 FAULT must stay quiet when only the FWS partition died");
    }

    #[test]
    fn fws_and_fcdc_combined_fault_needs_both_fcdcs_dead_too() {
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

    #[test]
    fn wired_ids_keep_the_default_takeoff_and_landing_inhibit() {
        assert_eq!(wiring(314_800_003).inhibit, phase::TAKEOFF_AND_LANDING);
        for id in [319_800_002, 319_800_003] {
            assert_eq!(wiring(id).inhibit, &[3, 4, 5, 6, 7, 8, 9, 10, 11]);
        }
        assert_eq!(wiring(334_800_101).inhibit, &[3, 4, 5, 6, 7, 8, 9, 10]);
        assert_eq!(wiring(314_800_004).inhibit, phase::NONE);
    }

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

        assert_eq!(wiring(314_800_001).inhibit, phase::NONE);
        assert_eq!(wiring(314_800_005).inhibit, phase::NONE);
    }
}
