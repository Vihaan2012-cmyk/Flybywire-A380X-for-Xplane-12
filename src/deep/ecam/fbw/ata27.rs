//! ATA 27 -- flight controls. FlyByWire defines 100 abnormal-sensed
//! procedures in `AbnormalSensed/ata27.ts` and wires 32 of them itself
//! (`grep -c '^    27' FwsAbnormalSensed.ts` restricted to the 27xxxxxxx
//! ids: 32). The other **68** are text nothing can raise, and this module
//! wires **zero** of them -- but not for the single reason the previous
//! pass recorded.
//!
//! # Testing the inherited conclusion
//!
//! The previous pass's note says "every unwired one names a computer (PRIM,
//! SEC, FCDC) or a control-law degradation." That is true of roughly half
//! the 68, but reading FlyByWire's own titles for all of them (not just a
//! sample) turns up a second, distinct family: plain surface/actuator
//! titles that name no computer at all --
//! `271800006` AILERON ACTUATOR FAULT, `271800011` ELEVATOR ACTUATOR FAULT,
//! `271800016` GND SPLRs FAULT, `271800048` RUDDER ACTUATOR FAULT,
//! `271800066` STABILIZER ACTUATOR FAULT, `271800030`/`271800031` PART/MOST
//! SPLRs FAULT, `272800004`/`272800005` FLAP CTL 1/2 FAULT,
//! `272800019`/`272800020` SLAT CTL 1/2 FAULT, and the six per-position
//! aileron ids (`271800019`-`271800024` L/R INR/MID/OUTR AILERON FAULT).
//! These are exactly the kind of "surface, servo or hydraulic supply"
//! procedure this pass was asked to look for, and `deep::flight_controls`
//! does model the surfaces and actuators they name (`actuator.rs`,
//! `surface.rs`, `ths.rs`, `live.rs`).
//!
//! So the finding this pass adds: **none of the 68 fail for lack of a
//! modelled surface.** They fail for one of two independent reasons, and
//! every one of the 68 falls into one or the other --
//!
//! ## Reason 1: the title names a computer, control law or input device we
//! do not model
//!
//! `271800001`/`271800002`/`271800025`-`271800028` (L/R SIDESTICK
//! FAULT/SENSOR FAULT -- the sidestick itself, an input device, not a
//! surface), `271800018` (TWO GYROMETERs FAULT -- rate gyros feeding the
//! flight control laws), `271800029` (LOAD ALLEVIATION FAULT -- a PRIM
//! control-law function), `271800033`-`271800035`/`271800039`-`271800041`
//! (PRIM n ELEVATOR/RUDDER ACTUATOR FAULT -- named by *which PRIM* commands
//! the actuator, not by the actuator), `271800042`-`271800044` (PRIM n
//! SIDESTICK SENSOR FAULT), `271800045`-`271800047` (PRIM/SEC VERSIONS
//! DISAGREE, PRIMs PIN PROG DISAGREE -- software identity), `271800050`/
//! `271800052`/`271800053` (RUDDER PEDAL FAULT/SENSOR FAULT, RUDDER
//! PRESSURE SENSOR FAULT -- pedal transducers, not the rudder), and
//! `272800013`-`272800015` (FLAPS LEVER OUT OF DETENT, FLAPS LEVER SYS 1/2
//! FAULT -- the cockpit lever and its two sensing channels). None of these
//! has a published variable in `deep::flight_controls` or anywhere else:
//! `deep_published_vars.txt` has no PRIM/SEC/FCDC identity, no sidestick,
//! no rudder pedal transducer, no lever-detent sensor. Left unwired.
//!
//! `270900001`-`270900005` (RUDDER PEDAL JAMMED, RUDDER TRIM RUNAWAY, SPEED
//! BRAKES LEVER JAMMED, LDG WITH FLAPS LEVER JAMMED, LDG WITH NO SLATS NO
//! FLAPS) are `sensed: false` in FlyByWire's own source -- they carry no
//! `simVarIsActive` for *any* port to raise, by FlyByWire's own design.
//! Structurally ineligible, not merely unmodelled.
//!
//! `272800009`-`272800012`/`272800025`-`272800027` (FLAP/SLAT n SAFETY TEST
//! REQUIRED, SLATS/FLAPS TIP BRK TEST REQUIRED) and `272800016`/
//! `272800024` (FLAPS/SLATS LOCKED) are maintenance-state flags with no
//! corresponding published variable either -- `deep::flight_controls`
//! publishes wingtip-brake *state*
//! (`FCTL_FLAP_WINGTIP_BRAKE_ON`/`FCTL_SLAT_WINGTIP_BRAKE_ON`) but nothing
//! for "a post-event safety test is now required" or "the surface is
//! mechanically locked", and inventing either from the brake-on bit would
//! be exactly the unsourced guess the module-level rule forbids. Left
//! unwired.
//!
//! ## Reason 2: the surface or actuator is modelled, but our own registry
//! already announces the same condition
//!
//! `deep::flight_controls::registry` already carries a system-level alert
//! for every surface family this chapter's remaining ids describe, each
//! built as `any_fault` over that family's own per-position published
//! faults: `FCTL_AIL_FAULT` (over `FCTL_AIL_{L,R}{1,2,3}_FAULT`),
//! `FCTL_ELEV_FAULT` (`FCTL_ELEV_{L,R}_{INBD,OUTBD}_FAULT`), `FCTL_RUD_FAULT`
//! (`FCTL_RUD_{LOWER,UPPER}_FAULT`), `FCTL_SPLR_FAULT` (all sixteen
//! `FCTL_SPLR_{L,R}{1..8}_FAULT`), `FCTL_GND_SPLR_FAULT`, `FCTL_FLAP_FAULT`
//! (`FCTL_FLAP_{L,R}_FAULT`), `FCTL_SLAT_FAULT` (`FCTL_SLAT_{L,R}_FAULT`),
//! `FCTL_RUD_TRIM_FAULT` and `FCTL_THS_RUNAWAY` (status-lines as
//! `F/CTL THS FAULT`, built on the published `FCTL_THS_FAULT`, itself the
//! union of the THS's green motor, yellow motor, no-back brake and
//! ballscrew-jam faults). Every one of those already fires the instant any
//! actuator in its family does.
//!
//! That makes the remaining ids duplicates rather than gaps:
//! `271800006`/`271800007` (AILERON ACTUATOR/ELEC ACTUATOR FAULT),
//! `271800010`/`271800011`/`271800012`/`271800061` (DOUBLE/SINGLE ELEVATOR
//! FAULT, ELEVATOR ACTUATOR/ELEC ACTUATOR FAULT), `271800019`-`271800024`
//! (the six per-position aileron faults), `271800016` (GND SPLRs FAULT),
//! `271800030`/`271800031` (PART/MOST SPLRs FAULT -- a count on top of the
//! same per-spoiler faults `FCTL_SPLR_FAULT` already unions),
//! `271800048`/`271800049` (RUDDER ACTUATOR/ELEC ACTUATOR FAULT),
//! `271800054`-`271800056` (RUDDER TRIM 1/2/FAULT -- `FCTL_RUD_TRIM_FAULT`
//! already covers the rudder trim actuator), `271800063` (SPD BRKs FAULT --
//! the spoilers used as speedbrakes are the same `FCTL_SPLR_*` surfaces),
//! `271800066`-`271800068` (STABILIZER ACTUATOR/ELEC ACTUATOR/FAULT -- the
//! THS), and `272800004`/`272800005` (FLAP CTL 1/2 FAULT) and
//! `272800019`/`272800020` (SLAT CTL 1/2 FAULT) -- these last four *do* name
//! a computer channel (the SFCC-equivalent), so they also partly overlap
//! Reason 1, but even a channel-resolved trigger would fire alongside
//! `FCTL_FLAP_FAULT`/`FCTL_SLAT_FAULT` on the same physical event, which is
//! the duplicate warning the module-level rule at `mod.rs` forbids
//! regardless of which reason gets there first.
//!
//! Wiring any of these would put the same failure on the EWD twice, which
//! is worse than leaving FlyByWire's own text unraised: this port already
//! tells the crew the aeroplane has an aileron/elevator/rudder/spoiler/
//! flap/slat/THS fault the moment it is true.
//!
//! # What this leaves for a later pass
//!
//! If `deep::flight_controls` ever publishes per-position identity in a
//! form `FwsCore`'s own `notActiveWhenItemActive` could suppress our
//! coarser alert with (so only one of the two ever reaches the crew), the
//! per-position ids above stop being duplicates. Until then, zero wired is
//! the correct answer for this chapter, and it is a different 68-way split
//! than the previous pass recorded, not the same conclusion inherited
//! unread.

use super::FbwProc;

/// This module wires nothing, for the two reasons in its module doc. If a
/// later pass adds an entry here, add that entry's own reachability test
/// to the `tests` module below rather than deleting the guards there.
pub fn wire(v: &mut Vec<FbwProc>) {
    let _ = v;
}

#[cfg(test)]
mod tests {
    use super::wire;
    use crate::deep::registry;

    /// Guards the "Reason 2" claim against drift: every system-level alert
    /// this module's doc cites as already covering a chapter-27 surface
    /// family must actually exist in our own registry, with the key named
    /// in the doc comment. If one of these is ever renamed or removed, the
    /// duplicate-warning argument above needs re-checking before this
    /// module can be trusted to still wire zero for the right reason.
    #[test]
    fn every_surface_family_this_module_defers_to_has_a_live_own_alert() {
        let reg = registry();
        let keys: Vec<&str> = reg.alerts.iter().map(|a| a.key.as_str()).collect();
        for expected in [
            "FCTL_AIL_FAULT",
            "FCTL_ELEV_FAULT",
            "FCTL_RUD_FAULT",
            "FCTL_SPLR_FAULT",
            "FCTL_GND_SPLR_FAULT",
            "FCTL_FLAP_FAULT",
            "FCTL_SLAT_FAULT",
            "FCTL_RUD_TRIM_FAULT",
            "FCTL_THS_RUNAWAY",
        ] {
            assert!(keys.contains(&expected), "expected our own registry to still carry {expected}, which this module's doc relies on to justify wiring zero ATA 27 ids; if it is gone, re-examine the 68 unwired ids before assuming they are still duplicates");
        }
    }

    /// `wire` really does add nothing, so `mod.rs`'s aggregation cannot
    /// silently pick up an id from this module that the doc above does not
    /// account for.
    #[test]
    fn wire_adds_nothing() {
        let mut v = Vec::new();
        wire(&mut v);
        assert!(v.is_empty(), "ata27::wire pushed an entry but the module doc claims zero wirings -- update the doc (and this test) together with any real addition");
    }
}
