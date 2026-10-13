//! Triggers for FlyByWire's *own* abnormal-sensed procedures that nothing of
//! FlyByWire's triggers.
//!
//! # The finding
//!
//! `fbw-a380x/src/systems/instruments/src/MsfsAvionicsCommon/EcamMessages/
//! AbnormalSensed/ata*.ts` defines **1004** abnormal-sensed procedures, each
//! with a title, a list of checklist items and a nine-digit id.
//! `systems-host/CpiomC/FlightWarningSystem/FwsAbnormalSensed.ts`'s
//! `ewdAbnormalSensed` map gives **273** of those ids an `EwdAbnormalItem`,
//! and that object's `simVarIsActive` is the *only* thing that can raise a
//! procedure. The remaining **732** are text with nothing to raise them.
//! (Counted from the two sources, not taken on trust; `tests.rs`'s
//! `the_defined_and_wired_counts_are_what_this_module_was_built_against`
//! re-counts them from FlyByWire's own tree.)
//!
//! That is not only 732 missing EWD warnings. The ECL's ABN PROC page
//! (`instruments/src/EWD/elements/WdAbnormalSensedProcedures.tsx`) renders
//! the `fws_abn_sensed_procedures` bus topic, which `FwsAbnormalSensed`
//! publishes from `FwsCore.presentedAbnormalProceduresList` -- and the only
//! way into that map is `FwsCore.ts:5561`'s loop over `this.ewdAbnormal`.
//! So an unwired id is also an electronic checklist the crew can never be
//! shown.
//!
//! # What this module does, and what it deliberately does not
//!
//! It adds an entry to `ewdAbnormal`/`allSuppressableItems`/
//! `abnormalSensed.ewdAbnormalSensed` **for FlyByWire's own id**, so
//! FlyByWire's own definition fires. It never registers a `deep::api::
//! EcamAlert` for any of these: that would mint a second, ten-digit id with
//! a second copy of the title and the items, and the crew would get the same
//! warning twice.
//!
//! It also does not touch `EcamInopSys`/`FwsInopSys`: on this airframe the
//! INOP SYS list is its own dict of independently-triggered items
//! (`FwsInopSys.ts`), not something an abnormal procedure carries, and the
//! `inopSysAllPhases` hook on `EwdAbnormalItem` is marked `@deprecated` in
//! FlyByWire's own source. Inventing an INOP SYS list per procedure would be
//! exactly the kind of unsourced guess `docs/deep/BRIEF.md` rule 3 forbids.
//!
//! # The rule every entry here obeys
//!
//! **Never invent a trigger.** Every [`FbwProc::trigger`] below is a `Cond`
//! over variables a deep area actually publishes (`Deep::published_names`),
//! and `tests.rs` asserts that as a standing check. Where this port does not
//! model the condition a procedure annunciates, the procedure is left
//! unwired and recorded in `docs/deep/fbw_unwired.md` with the reason --
//! an alert that fires on an approximation is worse than one that does not
//! fire, because a crew would act on it.
//!
//! **Never duplicate an alert we already raise.** `deep::registry()` carries
//! 304 alerts of our own. Where one of those already announces what a
//! FlyByWire procedure announces, the FlyByWire id is left unwired and
//! recorded, rather than wired into a double warning.
//!
//! # Per-item wiring
//!
//! A procedure's items are FlyByWire's. Each [`FbwItem`] names one by index
//! and may give it
//!
//! * `show` -- shown only while this holds (`whichItemsToShow`), which is
//!   how FlyByWire's own entries express "only above FL100", "only on the
//!   ground";
//! * `checked` -- a *sensed* line that ticks itself when this holds
//!   (`whichItemsChecked`).
//!
//! A line with no `checked` condition stays crew-actioned, which is what
//! FlyByWire's own `fusedChecked` (`FwsCore.ts:5650`) then leaves entirely
//! to the crew. That is the correct representation of a line this port does
//! not compute -- not a gap, and certainly not a reason to tick it from a
//! value we did not calculate.
//!
//! [`FbwProc::item_count`] records the procedure's own item count as read
//! from FlyByWire's source; `tests.rs` re-parses `AbnormalSensed/ata*.ts`
//! and fails if any of them has drifted, and the JS side sizes its vectors
//! from the live procedure regardless, so a mismatch can never reach
//! `FwsCore`'s size warning.

use crate::deep::api::{Cond, Level};

pub mod ata21_22_23;
pub mod ata24;
pub mod ata26;
pub mod ata27;
pub mod ata46_49_52_56;
pub mod ata28;
pub mod ata29;
pub mod ata31_33;
pub mod ata32;
pub mod ata34;
pub mod ata70;
pub mod ata_cheap_wins;

/// FlyByWire's `SdPages` (`fbw-a380x/src/systems/shared/src/
/// EcamSystemPages.ts`), by the numbers its own comment says nothing may
/// reorder.
pub mod sd_page {
    pub const ENG: i32 = 0;
    pub const APU: i32 = 1;
    pub const BLEED: i32 = 2;
    pub const COND: i32 = 3;
    pub const PRESS: i32 = 4;
    pub const DOOR: i32 = 5;
    pub const ELEC_AC: i32 = 6;
    pub const ELEC_DC: i32 = 7;
    pub const FUEL: i32 = 8;
    pub const WHEEL: i32 = 9;
    pub const HYD: i32 = 10;
    pub const FCTL: i32 = 11;
    pub const CB: i32 = 12;
    pub const CRZ: i32 = 13;
    pub const STATUS: i32 = 14;
}

/// FlyByWire's `FwcFlightPhase` (`FwsFlightPhases.ts:12-25`), used verbatim
/// in [`FbwProc::inhibit`] -- these entries are FlyByWire's own, so they
/// speak FlyByWire's own phase vocabulary rather than going through
/// `deep::api::Phase`, whose ten variants cannot express all twelve.
pub mod phase {
    pub const ELEC_PWR: u32 = 1;
    pub const FIRST_ENG_STARTED: u32 = 2;
    pub const SECOND_ENG_TO_POWER: u32 = 3;
    pub const AT_OR_ABOVE_80_KT: u32 = 4;
    pub const AT_OR_ABOVE_V1: u32 = 5;
    pub const LIFT_OFF: u32 = 6;
    pub const AT_OR_ABOVE_400_FT: u32 = 7;
    pub const AT_OR_ABOVE_1500_FT: u32 = 8;
    pub const AT_OR_BELOW_800_FT: u32 = 9;
    pub const TOUCH_DOWN: u32 = 10;
    pub const AT_OR_BELOW_80_KT: u32 = 11;
    pub const ENGINES_SHUTDOWN: u32 = 12;

    /// The window FlyByWire inhibits almost every ATA 21/24/26/28/29/70
    /// caution in: from take-off power through 1500 ft, and again from
    /// 800 ft through the landing roll. Copied from the phase list its own
    /// wired neighbours in the same chapter carry (e.g. `290800001`,
    /// `211800021`), so this is FlyByWire's convention and not a guess.
    pub const TAKEOFF_AND_LANDING: &[u32] = &[3, 4, 5, 6, 7, 9, 10];

    /// The tighter window used for warnings that must still be shown in the
    /// climb (`211800021` PACK 1+2 FAULT's own list).
    pub const TAKEOFF_AND_LANDING_ROLL: &[u32] = &[4, 5, 6, 7, 9, 10];

    /// Nothing inhibited: for warnings a crew must have at any moment
    /// (fire, smoke), matching FlyByWire's own `[]` on `260800001`.
    pub const NONE: &[u32] = &[];

    /// FlyByWire's own `FwsCore.phase56Inhibition` (`FwsCore.ts:2010`,
    /// `[5, 6]`), which every one of its wired ATA 70 entries uses
    /// (`701800029`..`701800032` ENG n FAIL, `701800109`.. ENG n SHUTDOWN).
    /// Engine alerts are inhibited from V1 to lift-off and at no other
    /// time, because that is the one stretch where a crew must not be
    /// distracted and cannot act.
    pub const ENG_56: &[u32] = &[5, 6];
}

/// One of FlyByWire's own checklist items, wired to what this port models.
#[derive(Clone, Debug)]
pub struct FbwItem {
    /// Index into the procedure's own `items[]`, in FlyByWire's order.
    pub index: usize,
    /// Shown only while this holds. `None` = always shown, FlyByWire's own
    /// default when `whichItemsToShow` is absent (`FwsCore.ts:5577`).
    pub show: Option<Cond>,
    /// The line ticks itself when this holds. `None` leaves it
    /// crew-actioned.
    pub checked: Option<Cond>,
}

/// `item(3).checked(var("X").on())`
pub fn item(index: usize) -> FbwItem {
    FbwItem { index, show: None, checked: None }
}

impl FbwItem {
    pub fn checked(mut self, c: Cond) -> Self {
        self.checked = Some(c);
        self
    }
    pub fn shown_if(mut self, c: Cond) -> Self {
        self.show = Some(c);
        self
    }
}

/// One of FlyByWire's own abnormal-sensed procedures, given the trigger it
/// never had.
#[derive(Clone, Debug)]
pub struct FbwProc {
    /// FlyByWire's own nine-digit id, exactly as `AbnormalSensed/ata*.ts`
    /// and `EcamAbnormalSensedProcedures` key it.
    pub id: u64,
    /// FlyByWire's own title, with its `\x1b` colour codes stripped. Never
    /// emitted to JS -- the title on the flight deck is FlyByWire's own, out
    /// of its own table. It is here so a reader (and `tests.rs`) can see at
    /// a glance which procedure an entry belongs to.
    pub title: &'static str,
    pub level: Level,
    pub sys_page: i32,
    /// FlyByWire's own `FwcFlightPhase` numbers; see [`phase`].
    pub inhibit: &'static [u32],
    /// How long the trigger must hold. FlyByWire's own `monitorConfirmTime`
    /// is set to 0 for these entries so this is the whole delay.
    pub confirm_s: f64,
    pub trigger: Cond,
    /// Other procedure ids that suppress this one while they are active
    /// (FlyByWire's `notActiveWhenItemActive`).
    pub suppressed_by: &'static [u64],
    pub items: Vec<FbwItem>,
    /// The procedure's own `items.len()` in FlyByWire's source, as a
    /// cross-check; see this module's doc comment.
    pub item_count: usize,
    /// Why this trigger is the right one for this procedure, and where the
    /// variables come from.
    pub note: &'static str,
}

/// A procedure with no crew items and nothing to sense per line -- the shape
/// most of these take (`items: []` in FlyByWire's own catalogue).
pub fn proc(id: u64, title: &'static str, level: Level, sys_page: i32, trigger: Cond, note: &'static str) -> FbwProc {
    FbwProc {
        id,
        title,
        level,
        sys_page,
        inhibit: phase::TAKEOFF_AND_LANDING,
        confirm_s: 0.0,
        trigger,
        suppressed_by: &[],
        items: Vec::new(),
        item_count: 0,
        note,
    }
}

impl FbwProc {
    pub fn confirm(mut self, s: f64) -> Self {
        self.confirm_s = s;
        self
    }
    pub fn inhibit(mut self, phases: &'static [u32]) -> Self {
        self.inhibit = phases;
        self
    }
    pub fn suppressed_by(mut self, ids: &'static [u64]) -> Self {
        self.suppressed_by = ids;
        self
    }
    /// The procedure's own item count, and the per-item wiring for the ones
    /// this port can sense.
    pub fn items(mut self, count: usize, items: Vec<FbwItem>) -> Self {
        self.item_count = count;
        self.items = items;
        self
    }
}

/// Every FlyByWire procedure this port wires, in id order.
pub fn wirings() -> Vec<FbwProc> {
    let mut v = Vec::new();
    ata21_22_23::wire(&mut v);
    ata24::wire(&mut v);
    ata26::wire(&mut v);
    ata27::wire(&mut v);
    ata46_49_52_56::wire(&mut v);
    ata28::wire(&mut v);
    ata29::wire(&mut v);
    ata31_33::wire(&mut v);
    ata32::wire(&mut v);
    ata34::wire(&mut v);
    ata70::wire(&mut v);
    ata_cheap_wins::wire(&mut v);
    v.sort_by_key(|p| p.id);
    v
}
