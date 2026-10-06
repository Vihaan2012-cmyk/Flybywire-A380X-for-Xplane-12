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

fn lav_alarms(first: u32) -> Cond {
    any((first..first + 4).map(|n| var(&format!("DEEP_SMOKE_LAV_{n}_ALARM")).on()).collect())
}

fn lav_faults(first: u32) -> Cond {
    any((first..first + 4).map(|n| var(&format!("DEEP_SMOKE_LAV_{n}_FAULT")).on()).collect())
}

pub fn wire(v: &mut Vec<FbwProc>) {
    v.push(
        proc(
            260_800_033,
            "SMOKE IFE BAY SMOKE",
            Level::Warning,
            -1,
            any(vec![var("CABIN_IFE_ZONE_SMOKE:1").on(), var("CABIN_IFE_ZONE_SMOKE:2").on(), var("CABIN_IFE_ZONE_SMOKE:3").on()]),
            "FCOM PRO-ABN-ECAM p.4991",
        )
        .inhibit(&[4, 5, 6, 9, 10])
        .items(3, Vec::new()),
    );


    for (id, title, first, note) in [
        (
            260_800_065u64,
            "SMOKE MAIN DECK LAVATORY SMOKE",
            1u32,
            "any of the four main-deck lavatory smoke detectors in alarm; they sample THERMAL_ZONE_CABINMAINDECK_SMOKE_CONCENTRATION, which deep::thermal_zones' main-deck lavatory waste-bin fire now drives",
        ),
        (
            260_800_066,
            "SMOKE UPPER DECK LAVATORY SMOKE",
            5,
            "any of the four upper-deck lavatory smoke detectors in alarm, from the upper-deck waste-bin fire source",
        ),
    ] {
        v.push(proc(id, title, Level::Warning, sd_page::COND, lav_alarms(first), note).confirm(5.0).inhibit(&[4, 5, 6, 7, 9, 10]).items(1, Vec::new()));
    }

    v.push(
        proc(260_800_029, "SMOKE AFT AVNCS DET FAULT", Level::Advisory, sd_page::COND, var("DEEP_SMOKE_AVNCS_AFT_FAULT").on(), "FCOM PRO-ABN-ECAM p.4982")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(260_800_030, "SMOKE AFT AVNCS SMOKE", Level::Warning, sd_page::COND, var("DEEP_SMOKE_AVNCS_AFT_ALARM").on(), "FCOM PRO-ABN-ECAM p.4983")
            .confirm(5.0)
            .inhibit(&[4, 5, 6, 9, 10])
            .items(5, Vec::new()),
    );

    v.push(
        proc(260_800_031, "SMOKE DET FAULT", Level::Caution, sd_page::COND, var("DEEP_SMOKE_ANY_DETECTOR_FAULT").on(), "FCOM PRO-ABN-ECAM p.4988")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10])
            .items(6, Vec::new()),
    );

    v.push(
        proc(260_800_032, "SMOKE IFE BAY DET FAULT", Level::Advisory, sd_page::COND, any(vec![var("CABIN_IFE_ZONE_SMOKE_FAULT:1").on(), var("CABIN_IFE_ZONE_SMOKE_FAULT:2").on(), var("CABIN_IFE_ZONE_SMOKE_FAULT:3").on()]), "FCOM PRO-ABN-ECAM p.4990")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(1, Vec::new()),
    );

    v.push(
        proc(260_800_034, "SMOKE L MAIN AVNCS DET FAULT", Level::Advisory, sd_page::COND, var("DEEP_SMOKE_AVNCS_MAIN_L_FAULT").on(), "FCOM PRO-ABN-ECAM p.4992 -- FCOM folds L/R Main/Upper into one procedure")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(260_800_035, "SMOKE R MAIN AVNCS DET FAULT", Level::Advisory, sd_page::COND, var("DEEP_SMOKE_AVNCS_MAIN_R_FAULT").on(), "FCOM PRO-ABN-ECAM p.4992 -- FCOM folds L/R Main/Upper into one procedure")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(260_800_036, "SMOKE L UPPER AVNCS DET FAULT", Level::Advisory, sd_page::COND, var("DEEP_SMOKE_AVNCS_UPPER_L_FAULT").on(), "FCOM PRO-ABN-ECAM p.4992 -- FCOM folds L/R Main/Upper into one procedure")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(260_800_037, "SMOKE R UPPER AVNCS DET FAULT", Level::Advisory, sd_page::COND, var("DEEP_SMOKE_AVNCS_UPPER_R_FAULT").on(), "FCOM PRO-ABN-ECAM p.4992 -- FCOM folds L/R Main/Upper into one procedure")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(260_800_038, "SMOKE L MAIN AVNCS SMOKE", Level::Warning, sd_page::COND, var("DEEP_SMOKE_AVNCS_MAIN_L_ALARM").on(), "FCOM PRO-ABN-ECAM p.4993 -- FCOM folds L/R into one procedure")
            .confirm(5.0)
            .inhibit(&[4, 5, 6, 9, 10])
            .items(6, Vec::new()),
    );

    v.push(
        proc(260_800_039, "SMOKE R MAIN AVNCS SMOKE", Level::Warning, sd_page::COND, var("DEEP_SMOKE_AVNCS_MAIN_R_ALARM").on(), "FCOM PRO-ABN-ECAM p.4993 -- FCOM folds L/R into one procedure")
            .confirm(5.0)
            .inhibit(&[4, 5, 6, 9, 10])
            .items(6, Vec::new()),
    );

    v.push(
        proc(260_800_040, "SMOKE L UPPER AVNCS SMOKE", Level::Warning, sd_page::COND, var("DEEP_SMOKE_AVNCS_UPPER_L_ALARM").on(), "FCOM PRO-ABN-ECAM p.4995 -- FCOM folds L/R into one procedure")
            .confirm(5.0)
            .inhibit(&[4, 5, 6, 9, 10])
            .items(8, Vec::new()),
    );

    v.push(
        proc(260_800_041, "SMOKE R UPPER AVNCS SMOKE", Level::Warning, sd_page::COND, var("DEEP_SMOKE_AVNCS_UPPER_R_ALARM").on(), "FCOM PRO-ABN-ECAM p.4995 -- FCOM folds L/R into one procedure")
            .confirm(5.0)
            .inhibit(&[4, 5, 6, 9, 10])
            .items(8, Vec::new()),
    );

    v.push(
        proc(260_800_042, "SMOKE FACILITIES DET FAULT", Level::Caution, sd_page::COND, var("DEEP_SDF_CONFIGURATION_FAULT").on(), "FCOM PRO-ABN-ECAM p.4997 -- un-UNSOURCED: FCOM sources a real trigger (SDF cabin-configuration mismatch); modelled as a new aggregate SDF discrete, DEEP_SDF_CONFIGURATION_FAULT")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(2, Vec::new()),
    );

    v.push(
        proc(260_800_043, "SMOKE FWD CARGO BOTTLES FAULT", Level::Caution, sd_page::COND, var("FIRE_CARGO_FWD_DISTRIBUTION_FAULT").on(), "FCOM PRO-ABN-ECAM p.5000 -- FCOM folds FWD/AFT into one procedure")
            .confirm(1.0)
            .inhibit(&[4, 5, 6, 7, 9, 10])
            .items(0, Vec::new()),
    );

    v.push(
        proc(260_800_044, "SMOKE AFT CARGO BOTTLES FAULT", Level::Caution, sd_page::COND, var("FIRE_CARGO_AFT_DISTRIBUTION_FAULT").on(), "FCOM PRO-ABN-ECAM p.5000 -- FCOM folds FWD/AFT into one procedure")
            .confirm(1.0)
            .inhibit(&[4, 5, 6, 7, 9, 10])
            .items(0, Vec::new()),
    );

    v.push(
        proc(260_800_048, "SMOKE FWD CARGO DET FAULT", Level::Advisory, sd_page::COND, any(vec![var("DEEP_SMOKE_FWD_CARGO_A_FAULT").on(), var("DEEP_SMOKE_FWD_CARGO_B_FAULT").on()]), "FCOM PRO-ABN-ECAM p.5004 -- FCOM folds FWD/AFT/BULK into one procedure")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(2, Vec::new()),
    );

    v.push(
        proc(260_800_049, "SMOKE AFT CARGO DET FAULT", Level::Advisory, sd_page::COND, any(vec![var("DEEP_SMOKE_AFT_CARGO_A_FAULT").on(), var("DEEP_SMOKE_AFT_CARGO_B_FAULT").on()]), "FCOM PRO-ABN-ECAM p.5004 -- FCOM folds FWD/AFT/BULK into one procedure")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(3, Vec::new()),
    );

    v.push(
        proc(260_800_050, "SMOKE BULK CARGO DET FAULT", Level::Advisory, sd_page::COND, var("DEEP_SMOKE_CARGO_BULK_FAULT").on(), "FCOM PRO-ABN-ECAM p.5004 -- FCOM folds FWD/AFT/BULK into one procedure")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(3, Vec::new()),
    );

    v.push(
        proc(260_800_051, "SMOKE FWD+AFT CARGO BOTTLES FAULT", Level::Caution, sd_page::COND, all(vec![var("FIRE_CARGO_FWD_DISTRIBUTION_FAULT").on(), var("FIRE_CARGO_AFT_DISTRIBUTION_FAULT").on()]), "FCOM PRO-ABN-ECAM p.5006")
            .confirm(1.0)
            .inhibit(&[4, 5, 6, 7, 9, 10])
            .items(0, Vec::new()),
    );

    v.push(
        proc(260_800_052, "SMOKE FWD+AFT CARGO BTL 1 FAULT", Level::Caution, sd_page::COND, any(vec![var("FIRE_CARGO_FWD_KNOCKDOWN_SQUIB_FAULT").on(), var("FIRE_CARGO_AFT_KNOCKDOWN_SQUIB_FAULT").on()]), "FCOM PRO-ABN-ECAM p.5007 -- FCOM folds BTL 1/2 into one procedure")
            .confirm(1.0)
            .inhibit(&[4, 5, 6, 7, 9, 10])
            .items(0, Vec::new()),
    );

    v.push(
        proc(260_800_053, "SMOKE FWD+AFT CARGO BTL 2 FAULT", Level::Caution, sd_page::COND, any(vec![var("FIRE_CARGO_FWD_EXTENDED_SQUIB_FAULT").on(), var("FIRE_CARGO_AFT_EXTENDED_SQUIB_FAULT").on()]), "FCOM PRO-ABN-ECAM p.5007 -- FCOM folds BTL 1/2 into one procedure")
            .confirm(1.0)
            .inhibit(&[4, 5, 6, 7, 9, 10])
            .items(0, Vec::new()),
    );

    v.push(
        proc(260_800_054, "SMOKE FWD LWR CAB REST BTL 1 FAULT", Level::Advisory, sd_page::COND, var("FIRE_LDCR_BTL_1_SQUIB_FAULT").on(), "FCOM PRO-ABN-ECAM p.4985 -- FCOM only models an AFT LDCR module; ours is FWD -- same architecture, sourced by analogy, page cited for the pattern only")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(260_800_055, "SMOKE FWD LWR CAB REST BTL 2 FAULT", Level::Advisory, sd_page::COND, var("FIRE_LDCR_BTL_2_SQUIB_FAULT").on(), "FCOM PRO-ABN-ECAM p.4985 -- FCOM only models an AFT LDCR module; ours is FWD -- same architecture, sourced by analogy, page cited for the pattern only")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(260_800_056, "SMOKE FWD LWR CAB REST DET FAULT", Level::Advisory, sd_page::COND, var("DEEP_SMOKE_FWDLOWERCREWREST_FAULT").on(), "FCOM PRO-ABN-ECAM p.4986 -- FCOM only models an AFT LDCR module; ours is FWD -- same architecture, sourced by analogy, page cited for the pattern only")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(260_800_057, "SMOKE FWD LWR CAB REST SMOKE", Level::Warning, sd_page::COND, var("DEEP_SMOKE_FWDLOWERCREWREST_ALARM").on(), "FCOM PRO-ABN-ECAM p.4987 -- FCOM only models an AFT LDCR module; ours is FWD -- same architecture, sourced by analogy, page cited for the pattern only")
            .confirm(5.0)
            .inhibit(&[4, 5, 6, 9, 10])
            .items(4, Vec::new()),
    );

    v.push(
        proc(260_800_058, "SMOKE MAIN 5L FLT REST DET FAULT", Level::Advisory, sd_page::COND, var("DEEP_SMOKE_MAIN5L_FLTREST_FAULT").on(), "FCOM PRO-ABN-ECAM p.4998")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(260_800_059, "SMOKE MAIN 5L CAB REST DET FAULT", Level::Advisory, sd_page::COND, var("DEEP_SMOKE_MAIN5L_CABREST_FAULT").on(), "FCOM PRO-ABN-ECAM p.4998 -- FCOM does not distinguish FLT REST from CAB REST by name; same procedure text covers flight crew rest compartment 1 or 2")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(260_800_060, "SMOKE MAIN 5L FLT REST SMOKE", Level::Warning, sd_page::COND, var("DEEP_SMOKE_MAIN5L_FLTREST_ALARM").on(), "FCOM PRO-ABN-ECAM p.4999")
            .confirm(5.0)
            .inhibit(&[4, 5, 6, 9, 10])
            .items(2, Vec::new()),
    );

    v.push(
        proc(260_800_061, "SMOKE MAIN 5L CAB REST SMOKE", Level::Warning, sd_page::COND, var("DEEP_SMOKE_MAIN5L_CABREST_ALARM").on(), "FCOM PRO-ABN-ECAM p.4999 -- FCOM does not distinguish FLT REST from CAB REST by name; same procedure text covers flight crew rest compartment 1 or 2")
            .confirm(5.0)
            .inhibit(&[4, 5, 6, 9, 10])
            .items(2, Vec::new()),
    );

    v.push(
        proc(260_800_062, "SMOKE MAIN DECK LAVATORY DET FAULT", Level::Advisory, sd_page::COND, lav_faults(1), "FCOM PRO-ABN-ECAM p.5008")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(260_800_063, "SMOKE UPPER DECK LAVATORY DET FAULT", Level::Advisory, sd_page::COND, lav_faults(5), "FCOM PRO-ABN-ECAM p.5008")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(260_800_064, "SMOKE LOWER DECK LAVATORY DET FAULT", Level::Advisory, sd_page::COND, var("DEEP_SMOKE_FWDLOWERCREWREST_FAULT").on(), "FCOM PRO-ABN-ECAM p.5008")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(260_800_067, "SMOKE LOWER DECK LAVATORY SMOKE", Level::Caution, sd_page::COND, var("DEEP_SMOKE_FWDLOWERCREWREST_ALARM").on(), "FCOM PRO-ABN-ECAM p.5009")
            .confirm(5.0)
            .inhibit(&[4, 5, 6, 7, 9, 10])
            .items(2, Vec::new()),
    );

    v.push(
        proc(260_800_068, "SMOKE MAIN 1L CWS DET FAULT", Level::Advisory, sd_page::COND, var("DEEP_SMOKE_MAIN_1L_CWS_FAULT").on(), "FCOM PRO-ABN-ECAM p.5010 -- FCOM XX=1L, CWS, main deck; folds all door positions/decks into one procedure")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(2, Vec::new()),
    );

    v.push(
        proc(260_800_069, "SMOKE MAIN 1L RCC DET FAULT", Level::Advisory, sd_page::COND, var("DEEP_SMOKE_MAIN_1L_RCC_FAULT").on(), "FCOM PRO-ABN-ECAM p.5010 -- FCOM XX=1L, RCC, main deck; folds all door positions/decks into one procedure")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(2, Vec::new()),
    );

    v.push(
        proc(260_800_070, "SMOKE UPPER 1L CWS DET FAULT", Level::Advisory, sd_page::COND, var("DEEP_SMOKE_UPPER_1L_CWS_FAULT").on(), "FCOM PRO-ABN-ECAM p.5010 -- FCOM XX=1L, CWS, upper deck; folds all door positions/decks into one procedure")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(2, Vec::new()),
    );

    v.push(
        proc(260_800_071, "SMOKE UPPER 1L RCC DET FAULT", Level::Advisory, sd_page::COND, var("DEEP_SMOKE_UPPER_1L_RCC_FAULT").on(), "FCOM PRO-ABN-ECAM p.5010 -- FCOM XX=1L, RCC, upper deck; folds all door positions/decks into one procedure")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(2, Vec::new()),
    );

    v.push(
        proc(260_800_072, "SMOKE MAIN 2L CWS DET FAULT", Level::Advisory, sd_page::COND, var("DEEP_SMOKE_MAIN_2L_CWS_FAULT").on(), "FCOM PRO-ABN-ECAM p.5010 -- FCOM XX=2L, CWS, main deck; folds all door positions/decks into one procedure")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(2, Vec::new()),
    );

    v.push(
        proc(260_800_073, "SMOKE MAIN 2L RCC DET FAULT", Level::Advisory, sd_page::COND, var("DEEP_SMOKE_MAIN_2L_RCC_FAULT").on(), "FCOM PRO-ABN-ECAM p.5010 -- FCOM XX=2L, RCC, main deck; folds all door positions/decks into one procedure")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(2, Vec::new()),
    );

    v.push(
        proc(260_800_074, "SMOKE UPPER 2L CWS DET FAULT", Level::Advisory, sd_page::COND, var("DEEP_SMOKE_UPPER_2L_CWS_FAULT").on(), "FCOM PRO-ABN-ECAM p.5010 -- FCOM XX=2L, CWS, upper deck; folds all door positions/decks into one procedure")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(2, Vec::new()),
    );

    v.push(
        proc(260_800_075, "SMOKE UPPER 2L RCC DET FAULT", Level::Advisory, sd_page::COND, var("DEEP_SMOKE_UPPER_2L_RCC_FAULT").on(), "FCOM PRO-ABN-ECAM p.5010 -- FCOM XX=2L, RCC, upper deck; folds all door positions/decks into one procedure")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(2, Vec::new()),
    );

    v.push(
        proc(260_800_076, "SMOKE MAIN 3R CWS DET FAULT", Level::Advisory, sd_page::COND, var("DEEP_SMOKE_MAIN_3R_CWS_FAULT").on(), "FCOM PRO-ABN-ECAM p.5010 -- FCOM XX=3R, CWS, main deck; folds all door positions/decks into one procedure")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(2, Vec::new()),
    );

    v.push(
        proc(260_800_077, "SMOKE MAIN 3R RCC DET FAULT", Level::Advisory, sd_page::COND, var("DEEP_SMOKE_MAIN_3R_RCC_FAULT").on(), "FCOM PRO-ABN-ECAM p.5010 -- FCOM XX=3R, RCC, main deck; folds all door positions/decks into one procedure")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(2, Vec::new()),
    );

    v.push(
        proc(260_800_078, "SMOKE UPPER 3R CWS DET FAULT", Level::Advisory, sd_page::COND, var("DEEP_SMOKE_UPPER_3R_CWS_FAULT").on(), "FCOM PRO-ABN-ECAM p.5010 -- FCOM XX=3R, CWS, upper deck; folds all door positions/decks into one procedure")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(2, Vec::new()),
    );

    v.push(
        proc(260_800_079, "SMOKE UPPER 3R RCC DET FAULT", Level::Advisory, sd_page::COND, var("DEEP_SMOKE_UPPER_3R_RCC_FAULT").on(), "FCOM PRO-ABN-ECAM p.5010 -- FCOM XX=3R, RCC, upper deck; folds all door positions/decks into one procedure")
            .confirm(1.0)
            .inhibit(&[3, 4, 5, 6, 7, 9, 10, 11])
            .items(2, Vec::new()),
    );

    v.push(
        proc(260_800_080, "SMOKE MAIN 1L CWS SMOKE", Level::Warning, sd_page::COND, var("DEEP_SMOKE_MAIN_1L_CWS_ALARM").on(), "FCOM PRO-ABN-ECAM p.5011 -- FCOM p.5011 Triggering Conditions text is a copy of p.5010 DET FAULT wording (smoke detection is failed) -- an apparent FCOM extraction duplication; the true SMOKE trigger (smoke is detected in the corresponding CWS/RCC) is inferred from the family pattern and confirmed by the page's own CRC/MASTER WARN icons (rendered), not from this mismatched text")
            .confirm(5.0)
            .inhibit(&[4, 5, 6, 9, 10])
            .items(3, Vec::new()),
    );

    v.push(
        proc(260_800_081, "SMOKE MAIN 1L RCC SMOKE", Level::Warning, sd_page::COND, var("DEEP_SMOKE_MAIN_1L_RCC_ALARM").on(), "FCOM PRO-ABN-ECAM p.5011 -- same CWS/RCC SMOKE family text-extraction caveat as 260800080")
            .confirm(5.0)
            .inhibit(&[4, 5, 6, 9, 10])
            .items(3, Vec::new()),
    );

    v.push(
        proc(260_800_082, "SMOKE UPPER 1L CWS SMOKE", Level::Warning, sd_page::COND, var("DEEP_SMOKE_UPPER_1L_CWS_ALARM").on(), "FCOM PRO-ABN-ECAM p.5011 -- same CWS/RCC SMOKE family text-extraction caveat as 260800080")
            .confirm(5.0)
            .inhibit(&[4, 5, 6, 9, 10])
            .items(3, Vec::new()),
    );

    v.push(
        proc(260_800_083, "SMOKE UPPER 1L RCC SMOKE", Level::Warning, sd_page::COND, var("DEEP_SMOKE_UPPER_1L_RCC_ALARM").on(), "FCOM PRO-ABN-ECAM p.5011 -- same CWS/RCC SMOKE family text-extraction caveat as 260800080")
            .confirm(5.0)
            .inhibit(&[4, 5, 6, 9, 10])
            .items(3, Vec::new()),
    );

    v.push(
        proc(260_800_084, "SMOKE MAIN 2L CWS SMOKE", Level::Warning, sd_page::COND, var("DEEP_SMOKE_MAIN_2L_CWS_ALARM").on(), "FCOM PRO-ABN-ECAM p.5011 -- same CWS/RCC SMOKE family text-extraction caveat as 260800080")
            .confirm(5.0)
            .inhibit(&[4, 5, 6, 9, 10])
            .items(3, Vec::new()),
    );

    v.push(
        proc(260_800_085, "SMOKE MAIN 2L RCC SMOKE", Level::Warning, sd_page::COND, var("DEEP_SMOKE_MAIN_2L_RCC_ALARM").on(), "FCOM PRO-ABN-ECAM p.5011 -- same CWS/RCC SMOKE family text-extraction caveat as 260800080")
            .confirm(5.0)
            .inhibit(&[4, 5, 6, 9, 10])
            .items(3, Vec::new()),
    );

    v.push(
        proc(260_800_086, "SMOKE UPPER 2L CWS SMOKE", Level::Warning, sd_page::COND, var("DEEP_SMOKE_UPPER_2L_CWS_ALARM").on(), "FCOM PRO-ABN-ECAM p.5011 -- same CWS/RCC SMOKE family text-extraction caveat as 260800080")
            .confirm(5.0)
            .inhibit(&[4, 5, 6, 9, 10])
            .items(3, Vec::new()),
    );

    v.push(
        proc(260_800_087, "SMOKE UPPER 2L RCC SMOKE", Level::Warning, sd_page::COND, var("DEEP_SMOKE_UPPER_2L_RCC_ALARM").on(), "FCOM PRO-ABN-ECAM p.5011 -- same CWS/RCC SMOKE family text-extraction caveat as 260800080")
            .confirm(5.0)
            .inhibit(&[4, 5, 6, 9, 10])
            .items(3, Vec::new()),
    );

    v.push(
        proc(260_800_088, "SMOKE MAIN 3R CWS SMOKE", Level::Warning, sd_page::COND, var("DEEP_SMOKE_MAIN_3R_CWS_ALARM").on(), "FCOM PRO-ABN-ECAM p.5011 -- same CWS/RCC SMOKE family text-extraction caveat as 260800080")
            .confirm(5.0)
            .inhibit(&[4, 5, 6, 9, 10])
            .items(3, Vec::new()),
    );

    v.push(
        proc(260_800_089, "SMOKE MAIN 3R RCC SMOKE", Level::Warning, sd_page::COND, var("DEEP_SMOKE_MAIN_3R_RCC_ALARM").on(), "FCOM PRO-ABN-ECAM p.5011 -- same CWS/RCC SMOKE family text-extraction caveat as 260800080")
            .confirm(5.0)
            .inhibit(&[4, 5, 6, 9, 10])
            .items(3, Vec::new()),
    );

    v.push(
        proc(260_800_090, "SMOKE UPPER 3R CWS SMOKE", Level::Warning, sd_page::COND, var("DEEP_SMOKE_UPPER_3R_CWS_ALARM").on(), "FCOM PRO-ABN-ECAM p.5011 -- same CWS/RCC SMOKE family text-extraction caveat as 260800080")
            .confirm(5.0)
            .inhibit(&[4, 5, 6, 9, 10])
            .items(3, Vec::new()),
    );

    v.push(
        proc(260_800_091, "SMOKE UPPER 3R RCC SMOKE", Level::Warning, sd_page::COND, var("DEEP_SMOKE_UPPER_3R_RCC_ALARM").on(), "FCOM PRO-ABN-ECAM p.5011 -- same CWS/RCC SMOKE family text-extraction caveat as 260800080")
            .confirm(5.0)
            .inhibit(&[4, 5, 6, 9, 10])
            .items(3, Vec::new()),
    );

    v.push(
        proc(260_800_092, "SMOKE SAFETY TEST REQUIRED", Level::Advisory, sd_page::COND, var("DEEP_SDF_SAFETY_TEST_OVERDUE").on(), "FCOM PRO-ABN-ECAM p.5012 -- un-UNSOURCED: FCOM sources a real trigger (50h since last automatic safety test, tested every 10h on ground); modelled as a BITE pass-through discrete (DEEP_SDF_SAFETY_TEST_OVERDUE), not a literal elapsed-hours clock")
            .confirm(1.0)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
    );

    v.push(
        proc(260_800_093, "SMOKE UPPER 1L SHOWER DET FAULT", Level::Advisory, sd_page::COND, var("DEEP_SMOKE_UPPER_1L_SHOWER_FAULT").on(), "no FCOM entry; this 2011 FCOM revision may not itemise showers separately from CWS/RCC -- DESIGN.md sibling choice (Caution, phase::NONE) kept")
            .confirm(1.0)
            .inhibit(phase::NONE)
            .items(0, Vec::new()),
    );

    v.push(
        proc(260_800_094, "SMOKE UPPER 1R SHOWER DET FAULT", Level::Advisory, sd_page::COND, var("DEEP_SMOKE_UPPER_1R_SHOWER_FAULT").on(), "no FCOM entry -- DESIGN.md sibling choice kept")
            .confirm(1.0)
            .inhibit(phase::NONE)
            .items(0, Vec::new()),
    );

    v.push(
        proc(260_800_095, "SMOKE UPPER 1L SHOWER SMOKE", Level::Warning, sd_page::COND, var("DEEP_SMOKE_UPPER_1L_SHOWER_ALARM").on(), "no FCOM entry -- DESIGN.md sibling choice kept, Warning matching FlyByWire's own red title")
            .confirm(5.0)
            .inhibit(phase::NONE)
            .items(2, Vec::new()),
    );

    v.push(
        proc(260_800_096, "SMOKE UPPER 1R SHOWER SMOKE", Level::Warning, sd_page::COND, var("DEEP_SMOKE_UPPER_1R_SHOWER_ALARM").on(), "no FCOM entry -- DESIGN.md sibling choice kept")
            .confirm(5.0)
            .inhibit(phase::NONE)
            .items(2, Vec::new()),
    );
}
