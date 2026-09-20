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
//! * `260800030`/`260800038`..`260800041` AVNCS SMOKE (aft, L/R main,
//!   L/R upper): `deep::fire_ice` models one avionics zone, not five, so
//!   there is no way to tell which of the five FlyByWire procedures a
//!   detection belongs to. Announcing the wrong bay is worse than
//!   announcing none.
//! * Every `... DET FAULT` and `... BOTTLES FAULT` procedure in this
//!   chapter (33 of the 69): `deep::sensors`' smoke detectors publish an
//!   alarm and a reading, not a detector-fault discrete, and the cabin
//!   extinguisher bottles are not modelled outside the engine/APU/cargo
//!   ones `deep::fire_ice` already carries.
//! * `260800092` SMOKE SAFETY TEST REQUIRED, `260900097` FIRE SMOKE /
//!   FUMES: a maintenance state and a crew-diagnosed (non-sensed)
//!   procedure; neither is a sensed aircraft condition.

use super::{phase, proc, sd_page, FbwProc};
use crate::deep::api::{any, var, Cond, Level};

/// The alarm discretes of one deck's four lavatory smoke detectors.
/// `deep::sensors` names them `DEEP_SMOKE_LAV_1..8_ALARM` and splits them
/// 1-4 main deck, 5-8 upper deck; that split lives in one place
/// (`sensors::live_discrete`) and this reads it back rather than
/// re-deciding it.
fn lav_alarms(first: u32) -> Cond {
    any((first..first + 4).map(|n| var(&format!("DEEP_SMOKE_LAV_{n}_ALARM")).on()).collect())
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
}
