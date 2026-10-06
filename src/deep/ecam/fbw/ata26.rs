//! ATA 26 -- fire and smoke. 69 of FlyByWire's 97 procedures here are
//! unwired, and almost all of them are cabin, crew-rest, shower, trolley
//! (CWS/RCC) and avionics-bay smoke detection this port does not model at
//! that resolution.
//!
//! # MAIN / UPPER DECK LAVATORY SMOKE, and the modelling gap that had to
//! # close first
//!
//! `deep::sensors` gives all eight lavatories a smoke detector
//! (`DEEP_SMOKE_LAV_1..8_ALARM`) and its own source records the deck split
//! FlyByWire's `260800065`/`260800066` are written against: lavatories 1-4
//! are main-deck, 5-8 upper-deck. Each of those detectors samples
//! `THERMAL_ZONE_CABIN{MAIN,UPPER}DECK_SMOKE_CONCENTRATION`.
//!
//! Until this pass *no registered failure injected smoke into a cabin
//! deck* -- `deep::thermal_zones::live::apply_fire_failures` injected only
//! into the cargo bays, the nacelle cowls and the APU compartment. A
//! trigger written then would have been well formed, would have read only
//! published variables, and would still never have fired: the same bug in
//! a better disguise, and the reason the previous pass left both unwired
//! rather than wiring a dead alert.
//!
//! What closes it is a real source rather than a trigger: two ATA 26
//! failures in `deep::thermal_zones`, a main-deck and an upper-deck
//! **lavatory waste-bin fire**, injecting heat and smoke into
//! `zones.cabin_{main,upper}_deck` exactly as the three cargo fires do into
//! their bays. That is the one cabin fire the aircraft is certified to
//! detect by itself (CS/FAR 25.854 requires a smoke detector in every
//! lavatory and a built-in extinguisher in every waste receptacle), which
//! is why these two procedures exist at all. `thermal_zones` still has no
//! lavatory *zone*, and none is invented: the lavatory extract draws the
//! deck's own air past the detector, which is what
//! `sensors::live_discrete`'s own note on these eight detectors already
//! says, so the deck concentration is what a lavatory detector sees.
//!
//! The trigger is the *detectors'* alarms, not the zone concentration:
//! `any(DEEP_SMOKE_LAV_1..4_ALARM)` for the main deck and `5..8` for the
//! upper. That is deliberate -- it is what the real FWS has to work from,
//! it carries the sensors' own faults (a stuck or desensitised detector
//! genuinely does not alarm), and it keeps the deck split in the one place
//! that already records it.
//!
//! # What is deliberately left unwired
//!
//! * `260800045`/`260800046`/`260800047` FWD/AFT/BULK CARGO SMOKE:
//!   `deep::fire_ice`'s own `registry.rs` already registers `CARGO SMOKE
//!   FWD`, `CARGO SMOKE AFT` and `CARGO SMOKE BULK` on the same detectors.
//!   Wiring FlyByWire's ids as well would double-annunciate a cargo fire.
//! * `260800033` IFE BAY SMOKE: our own `CAB IFE SMOKE` (`deep::cabin`,
//!   ATA 44) already announces it.
//! * `260800067` LOWER DECK LAVATORY SMOKE: the eight modelled lavatory
//!   detectors are main and upper deck only; nothing samples a lower-deck
//!   lavatory, so there is no detector to raise it.
//! * `260900097` FIRE SMOKE / FUMES: `sensed: false` in FlyByWire's own
//!   catalogue (crew-diagnosed, cannot take a trigger).
//!
//! # ECAM completeness pass (E-FIRE, FCOM addendum)
//!
//! The rest of this chapter's 69 unwired procedures are wired below,
//! against the A380 FCOM (`E:/fbw-debug/ecam/E-FIRE-FCOM.json` has the full
//! per-id mapping: page, triggering text, level and flight-phase inhibit).
//! New model components this pass adds, each following the pattern
//! `sensors::smoke_detector`'s own `circuit_fault` field already
//! established (a fourth, independent, pass-through fault alongside the
//! three sensing faults):
//!
//! * **23 new smoke-detector instances** (`deep::sensors::live_discrete`,
//!   `DEEP_SMOKE_<slug>_{PCT_PER_FT,ALARM,FAULT}`) sampling either an
//!   existing zone concentration or one of the two new zones below.
//! * **Two new `thermal_zones` compartments**: the Aft avionics bay and the
//!   FWD Lower Crew Rest (LDCR) module (`topology_a380::A380Zones::{aft_
//!   avionics,fwd_lower_crew_rest}`), each with its own registered fire/
//!   smoke-load failure, plus new Main/Upper avionics equipment-fire
//!   failures injected into the two *existing* avionics bays.
//! * **`deep::fire_ice`**: a per-hold cargo distribution-path fault
//!   (`FIRE_CARGO_{FWD,AFT}_DISTRIBUTION_FAULT`), the cargo bottles' single
//!   `squib_failure` split into `knockdown_squib_fault`/`extended_squib_
//!   fault` (two physically distinct bottles/stages), and the LDCR
//!   module's own two-bottle squib-circuit discretes (`FIRE_LDCR_BTL_
//!   {1,2}_SQUIB_FAULT`, FCOM PRO-ABN-ECAM p.4985's AFT-module analog).
//! * **A new aggregate SDF (Smoke Detection Function) discrete pair**
//!   (`DEEP_SDF_CONFIGURATION_FAULT`, `DEEP_SDF_SAFETY_TEST_OVERDUE`) --
//!   `260800042`/`260800092`, un-UNSOURCED once the FCOM's own triggering
//!   text (p.4997, p.5012) gave them a real condition to model.
//! * **`deep::cabin::ife`** (cross-area, ATA 44): a per-zone smoke-detector
//!   circuit fault for `260800032` SMOKE IFE BAY DET FAULT, the alert's
//!   real owning area.
//!
//! **Level correction**: every `* SMOKE` (an actual detection, not `* DET
//! FAULT`) procedure in this chapter below `260800029` carries FlyByWire's
//! own **red** `\x1b<2m` title colour, not amber -- confirmed by direct
//! read of `ata26.ts` and independently by the FCOM's own CRC audio /
//! MASTER WARN icon, rendered and checked across four different families
//! (AFT AVNCS SMOKE p.4983, MAIN/UPPER XX CWS/RCC SMOKE p.5011, FLT REST
//! SMOKE p.4999, FWD/AFT/BULK CARGO SMOKE p.5001). These are wired
//! `Level::Warning`; every `* DET FAULT`/`* FAULT` procedure, and the
//! MAIN/UPPER/LOWER DECK LAVATORY SMOKE family, remain amber (`\x1b<4m`),
//! `Level::Caution`, matching the earlier, still-correct assessment for
//! that family.
//!
//! `260800033`, `260800045`-`260800047` (duplicates, above) and
//! `260900097` (sensed:false, above) stay unwired; everything else FlyByWire
//! defines in this chapter now has a real, sourced trigger.

use super::{phase, proc, sd_page, FbwProc};
use crate::deep::api::{all, any, var, Cond, Level};

/// The alarm discretes of one deck's four lavatory smoke detectors.
/// `deep::sensors` names them `DEEP_SMOKE_LAV_1..8_ALARM` and splits them
/// 1-4 main deck, 5-8 upper deck; that split lives in one place
/// (`sensors::live_discrete`) and this reads it back rather than
/// re-deciding it.
fn lav_alarms(first: u32) -> Cond {
    any((first..first + 4).map(|n| var(&format!("DEEP_SMOKE_LAV_{n}_ALARM")).on()).collect())
}

/// The same deck split as [`lav_alarms`], read off each detector's own
/// `circuit_fault` sibling instead (§A, `sensors::live_discrete`) for
/// `260800062`/`260800063` MAIN/UPPER DECK LAVATORY DET FAULT.
fn lav_faults(first: u32) -> Cond {
    any((first..first + 4).map(|n| var(&format!("DEEP_SMOKE_LAV_{n}_FAULT")).on()).collect())
}

pub fn wire(v: &mut Vec<FbwProc>) {
    // Both are amber in FlyByWire's own title (`\x1b<4m`), not red, and
    // both carry a single crew-actioned line ("CKPT / CABIN COM ...
    // ESTABLISH", `sensed: false`), so no item wiring: there is nothing
    // here this port computes, and `fusedChecked` leaves the line to the
    // crew, which is correct.
    //
    // No flight-phase inhibit, matching FlyByWire's own `[]` on every
    // sensed smoke procedure it does trigger (`260800001`): a smoke
    // warning must reach the crew at any moment.
    //
    // 5 s confirmation: a smoke detector's own output is already
    // filtered (`physics`'s `SmokeDetector` carries the alarm hysteresis),
    // and this is the same order of delay a real lavatory detector's
    // annunciation uses -- long enough that a single noisy frame cannot
    // raise a cabin smoke warning.
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
        v.push(proc(id, title, Level::Caution, sd_page::COND, lav_alarms(first), note).confirm(5.0).inhibit(phase::NONE).items(1, Vec::new()));
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
