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
//!
//! # Phase 2 (E-FCTL, ECAM completeness pass, 2026-09-27)
//!
//! `E-FCTL-DESIGN.md` re-examined the 40 ids the note above leaves
//! unwired for "Reason 1" and modelled 35 of them, on the finding that
//! `src/prim.rs` already exposes (or, with a small shim extension, can
//! expose) real, live signals a PRIM/SEC otherwise hides: the priority-
//! takeover-driven sidestick-disable bits, and this port's own
//! `sensors::DualTransducer` machinery applied to the cockpit sidestick/
//! pedal inputs. **Implementing that design surfaced three further
//! corrections**, each checked against the real compiled source or this
//! port's own input wiring rather than assumed, which move 12 of those 35
//! back to unsourced (on top of the one, `271800053`, the design already
//! flagged):
//!
//! 1. **The L/R take-over mapping was inverted.** The design's own
//!    "meaning" text (captain's take-over locks the F.O.'s stick) does not
//!    match the id it names (`271800001` is titled *L* SIDESTICK FAULT).
//!    The FCOM's own triggering text settles it (`E-FCTL-FCOM.json`,
//!    p.5015): "the L(R) sidestick is deactivated... because the take-over
//!    pb on the *opposite* sidestick has been pressed" -- so `271800001`
//!    (L) is the F.O.'s take-over disabling the captain's own stick, and
//!    `271800002` (R) is the captain's take-over disabling the F.O.'s.
//!    Wired the corrected way below.
//! 2. **There is no live F.O.-side sidestick input in this port.**
//!    `prim.rs` only ever assigns `capt_pitch_stick_pos`/`capt_roll_stick_
//!    pos` from a real X-Plane axis (`prim.rs:1124-1125,1672-1673`);
//!    `fbw_types.rs`'s `fo_*_stick_pos` fields are written nowhere from a
//!    live input (grepped: only ever read, never assigned). A
//!    `DualTransducer` watching a permanently-idle value cannot be
//!    meaningfully faulted or tested, which is the same category of gap
//!    `271800018`'s own rate-gyro finding below describes. `271800026`/
//!    `271800028` (R SIDESTICK FAULT/SENSOR FAULT) move to unsourced for
//!    this reason; `271800025`/`271800027` (L side) keep the real captain's
//!    axis and are wired.
//! 3. **The compiled `fctl_logic` avail bits are not per-PRIM channels.**
//!    Reading `A380PrimComputerFctl.cpp:789-930` (the block computing
//!    `elevator1Avail`/`elevator2Avail`/the rudder-mode-avail bits) shows
//!    each is "is this physical actuator's hydraulic supply available",
//!    computed from `is_unit_1`/`_2`/`_3` branches that select *which
//!    hydraulic system* backs that actuator -- not "this PRIM's own command
//!    channel". Two different PRIMs asking for `elevator_1_avail` get the
//!    same real-world fact, not two different channels. Wiring
//!    `271800033`-`271800035`/`271800039`-`271800041` (PRIM n ELEVATOR/
//!    RUDDER ACTUATOR FAULT) as "this PRIM healthy AND its own channel
//!    unavailable" would therefore not represent what the title claims
//!    (a fault specific to *that* PRIM's command path). The design's own
//!    flagged uncertainty here ("Phase 2 must confirm... not assumed") is
//!    resolved by this reading, not confirmed by it; these six move to
//!    unsourced. The identical reasoning applies to `271800042`-
//!    `271800044` (PRIM n SIDESTICK SENSOR FAULT): grepping `Prim.cpp`/
//!    `A380PrimComputerFctl.cpp` for a per-PRIM sidestick-channel routing
//!    table (as the design's own note says it tried) finds none, and the
//!    design's proposed odd/even-PRIM assignment is this design's own
//!    invention, not a real routing -- also moved to unsourced. And
//!    `271800029` (LOAD ALLEVIATION FAULT): `aileron_droop_active`/
//!    `aileron_antidroop_active` are gated on the real aerodynamic
//!    condition (speed/load) the function needs, which this port does not
//!    model, so "not active" cannot be distinguished from "not currently
//!    needed" without inventing that envelope -- unsourced.
//!
//! What Phase 2 does wire, all built on real, live, tested signals:
//! `271800001`/`271800002` (sidestick disabled by take-over -- a straight
//! pass-through of `Truth::prim_left_sidestick_disabled`/`_right_...`,
//! themselves `src/prim.rs`'s real compiled-PRIM output), `271800025`/
//! `271800027` (L SIDESTICK FAULT/SENSOR FAULT, the captain's real stick
//! axis through a new `sensors::DualTransducer`), `271800050`/`271800052`
//! (RUDDER PEDAL FAULT/SENSOR FAULT, the same shape on the real rudder
//! pedal axis -- `prim.rs:1143,1688`), and `271800045`-`271800047` (PRIM/
//! SEC VERSIONS DISAGREE, PRIMs PIN PROG DISAGREE -- a genuinely new,
//! simple pin-programming identity component, `E-FCTL-DESIGN.md` section
//! 3.3, since this port's compiled PRIM/SEC carries no per-unit version
//! identity to compare). `deep::flight_controls::registry.rs`'s
//! `27_fctl.l_sidestick_pitch`/`_roll`, `27_fctl.rudder_pedal`,
//! `27_fctl.prim_pin_prog` and `27_fctl.sec_pin_prog` are the five new
//! components; `live.rs` steps and publishes them.
//!
//! # Coordinator follow-up (2026-09-27): re-examining MODEL -> UNSOURCED
//!
//! The user's rule is that a missing cause gets modelled, not dropped. Re-
//! reading the FCOM's own triggering text for the 13 ids the first Phase 2
//! pass moved to unsourced (`E-FCTL-FCOM.json`) found a real, modellable
//! condition for 13 of them, corrected and wired above:
//!
//! * `271800018` (TWO GYROMETERs FAULT): the objection was "the rate-gyro
//!   input is a permanently-zeroed constant, like the missing F.O. stick".
//!   That was wrong -- `physics/adirs.rs` runs a real strapdown-IRS
//!   simulation with its own gyro sensor model and writes its *sensed*
//!   value back over the same native datarefs `SimReadings::body_rotation_
//!   velocity_rad_s` reads, which is a real, already-failable quantity this
//!   pass threads into `Truth` and models as a two-channel gyro pair.
//! * `271800026`/`271800028` (R SIDESTICK FAULT/SENSOR FAULT): the
//!   objection was "no live F.O. stick axis to watch". Correct that the
//!   axis is inert, wrong that a transducer needs a moving input to be
//!   faultable: the FCOM's own triggering text never requires deflection
//!   ("The left (right) sidestick is failed" / a channel disagreement), so
//!   a `DualTransducer` held at a fixed neutral position is an honest model
//!   of a real physical transducer pair this port's cockpit never moves.
//! * `271800029` (LOAD ALLEVIATION FAULT): the objection was "droop/
//!   antidroop engagement depends on an aero envelope this port does not
//!   model". Reading the FCOM's own trigger (p.5040) shows the alert's real
//!   condition is narrower and fully modellable without that envelope: "Two
//!   out of three accelerometers used for the LAF are failed in one wing" --
//!   a literal 2-of-3 vote over six new accelerometer failures.
//! * `271800033`-`271800035`/`271800039`-`271800041` (PRIM n ELEVATOR/
//!   RUDDER ACTUATOR FAULT) and `271800042`-`271800044` (PRIM n SIDESTICK
//!   SENSOR FAULT): the objection that FlyByWire's compiled `elevator_n_
//!   avail`/rudder-mode-avail bits are per-*actuator*, not per-*PRIM*,
//!   still holds (confirmed again this pass) -- but the FCOM's own trigger
//!   for these nine ids ("PRIM n has lost the capacity to control an
//!   actuator" / "PRIM n detects a sensor is failed") describes a fault
//!   internal to that one PRIM's own command/monitoring circuitry, which
//!   FlyByWire's compiled model does not expose at all (confirmed:
//!   `A380PrimComputer*_types.h` has no sub-unit health bit narrower than
//!   whole-PRIM `prim_healthy`). Per the user's rule this is modelled as a
//!   new, real component instead -- nine new one-failure-each per-PRIM
//!   channel components in `deep::flight_controls`, each gated on that
//!   PRIM's own health so FlyByWire's already-wired whole-unit `271800036`-
//!   `038` is never double counted.
//!
//! `271800053` (RUDDER PRESSURE SENSOR FAULT) stays unsourced: its
//! quantity (a pedal feel-unit hydraulic pressure) is genuinely not
//! published or modelled anywhere in this port, and the coordinator's
//! message did not ask this one be re-examined (a force/pressure reading,
//! not a position one -- the pattern the other 13 fit does not apply).
//!
//! ## The deferred SFCC lever/wingtip-brake group (`272800009`-`272800027`)
//!
//! Of the 12, 4 are wired above: `272800014`/`272800015` (a new flap-lever-
//! CSU communication-loss component, FCOM PRO-ABN-ECAM p.5112's own literal
//! wording) and `272800016`/`272800024` (reusing the already-published
//! wingtip-brake-commanded state, `high_lift.rs`'s own asymmetry monitor --
//! no new component, since this port's brake achieves full holding torque
//! the instant it is commanded unless independently faulted).
//!
//! The other 8 stay unsourced, for a reason distinct from "no signal
//! exists" -- reading the FCOM's own triggering text this pass (not
//! assumed from the earlier design) found the *real* trigger is a
//! maintenance-hours/days clock this port has no sourced way to model
//! without inventing something:
//!
//! * `272800009`-`272800012` (FLAP/SLAT n SAFETY TEST REQUIRED): FCOM PRO-
//!   ABN-ECAM p.5109's actual text is "An automatic internal safety test
//!   has not been performed in the previous 150 operating hours" -- not the
//!   post-fault-event latch the original design assumed. Modelling only the
//!   accumulator (hours since last test) without a sourced "test completed"
//!   reset condition would either never reset (misrepresenting every
//!   sufficiently long *healthy* session as faulted, since the model has no
//!   way to complete a test on its own) or invent a reset trigger nothing
//!   sources -- both forbidden. Left unwired rather than guessed.
//! * `272800025`-`272800027` (SLATS/FLAPS TIP BRK TEST REQUIRED): FCOM PRO-
//!   ABN-ECAM p.5139 is the same shape, on a 5/10-*day* clock instead of
//!   hours. Same reasoning, same conclusion.
//! * `272800013` (FLAPS LEVER OUT OF DETENT): FCOM PRO-ABN-ECAM p.5111's
//!   trigger ("The FLAPS lever is between two detents") needs a continuous
//!   lever *angle*, and FlyByWire's own sourced thresholds for it
//!   (`TARGET_THRESHOLD_DEGREE = 0.18°`) are in degrees -- but this port's
//!   only flap-lever signal, `Truth::flap_lever_handle_index`, is a
//!   discrete detent index with no sourced index-to-degree conversion.
//!   Applying the degree threshold without one would be inventing the
//!   conversion factor, not the threshold itself, which the same "never
//!   fake a value" rule still forbids.

use super::{phase, proc, sd_page, FbwProc};
use crate::deep::api::{all, any, var, Cond, Level};

/// At least one main AC bus is live: the same "network alive" gate every
/// other chapter's bridge uses (`ata24::network_alive`'s own doc), so none
/// of these fire on a cold-and-dark aircraft merely because every published
/// input reads its own zero/healthy default.
fn network_alive() -> Cond {
    any(vec![
        var("ELEC_AC_1_BUS_IS_POWERED").on(),
        var("ELEC_AC_2_BUS_IS_POWERED").on(),
        var("ELEC_AC_3_BUS_IS_POWERED").on(),
        var("ELEC_AC_4_BUS_IS_POWERED").on(),
    ])
}

/// FCDC-sibling inhibit/confirm (`271800013`/`014` FCDC FAULT,
/// `FwsAbnormalSensed.ts:2589-2603`: `flightPhaseInhib: [3,4,5,6,7,9,10,11]`,
/// `failure: 2`, no `monitorConfirmTime` override -> the documented 0.6 s
/// default), which `E-FCTL-DESIGN.md` section 4 uses for every computer/
/// config-identity id in this chapter -- except where the FCOM's own decoded
/// phase bar disagrees, in which case the FCOM wins
/// (`BRIEF-phase2-FCOM.md`) and the FCOM's own array is used instead, cited
/// by page in the `proc()` call.
const FCDC_SIBLING_CONFIRM_S: f64 = 0.6;

pub fn wire(v: &mut Vec<FbwProc>) {
    // ---- 271800001/271800002: sidestick disabled by the opposite side's
    // priority take-over pushbutton. FCOM PRO-ABN-ECAM p.5015 (`E-FCTL-
    // FCOM.json`): "the L(R) sidestick is deactivated... because the take
    // over pb on the opposite sidestick has been pressed for more than
    // 30 s" -- inhibit [5,6,7,8,9,10,12], the FCOM's own bar, which differs
    // from the FCDC-sibling default and so wins per the FCOM addendum.
    // FlyByWire's own title colour is amber/caution (`ata27.ts:29,34`)
    // though the FCOM shows this as a red/CRC warning (images `54437`/
    // `54438`); kept the quieter caution
    // (`no_entry_is_louder_than_flybywires_own_title_colour`).
    v.push(
        proc(
            271_800_001,
            "CONFIG L SIDESTICK FAULT (BY TAKE-OVER)",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_L_SIDESTICK_DISABLED_BY_TAKEOVER").on(), network_alive()]),
            "the F.O.'s priority-takeover pushbutton has locked the captain's stick out -- FCOM PRO-ABN-ECAM p.5015; A380PrimComputerFctl.cpp:1770-1790 via prim.rs's A32NX_PRIM_1_LEFT_SIDESTICK_DISABLED",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[5, 6, 7, 8, 9, 10, 12])
        .items(0, Vec::new()),
    );
    v.push(
        proc(
            271_800_002,
            "CONFIG R SIDESTICK FAULT (BY TAKE-OVER)",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_R_SIDESTICK_DISABLED_BY_TAKEOVER").on(), network_alive()]),
            "the captain's priority-takeover pushbutton has locked the F.O.'s stick out -- FCOM PRO-ABN-ECAM p.5015; A380PrimComputerFctl.cpp:1770-1790 via prim.rs's A32NX_PRIM_1_RIGHT_SIDESTICK_DISABLED",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[5, 6, 7, 8, 9, 10, 12])
        .items(0, Vec::new()),
    );

    // ---- 271800025/271800027: the captain's own sidestick, both channels
    // lost outright vs. the two channels merely disagreeing. FCOM PRO-ABN-
    // ECAM p.5043 (`F/CTL L(R) SIDESTICK FAULT`) shows no phase bar
    // (`fcom_alerts.json`'s `phases: []` for this page) -- so the FCOM gives
    // no inhibit to prefer, and the FCDC-sibling default stands, per the
    // addendum's "if the FCOM has no entry, keep the DESIGN.md's sibling
    // choice".
    v.push(
        proc(
            271_800_025,
            "F/CTL L SIDESTICK FAULT",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_L_SIDESTICK_PITCH_FAULT").on(), var("FCTL_L_SIDESTICK_ROLL_FAULT").on(), network_alive()]),
            "both channels of the captain's sidestick pitch AND roll transducers invalid -- deep::flight_controls's new sensors::DualTransducer on the real captain's stick axis (prim.rs:1124-1125); FCOM PRO-ABN-ECAM p.5043",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(phase::TAKEOFF_AND_LANDING)
        .items(0, Vec::new()),
    );
    v.push(
        proc(
            271_800_027,
            "F/CTL L SIDESTICK SENSOR FAULT",
            Level::Caution,
            sd_page::FCTL,
            all(vec![any(vec![var("FCTL_L_SIDESTICK_PITCH_SENSOR_FAULT").on(), var("FCTL_L_SIDESTICK_ROLL_SENSOR_FAULT").on()]), network_alive()]),
            "the captain's sidestick pitch or roll channels disagree by more than TRANSDUCER_DISAGREE_RAD for TRANSDUCER_DISAGREE_TIMER_S (live.rs:156-157) while at least one channel still reads -- narrower than 271800025's full loss; FCOM PRO-ABN-ECAM p.5077 (correction, coordinator follow-up 2026-09-27: the previous pass wrongly cited p.5043 -- the generic SIDESTICK SENSOR FAULT procedure this id and 271800028 share is on p.5077, which does have a decoded phase bar, unlike p.5043)",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
        .items(0, Vec::new()),
    );

    // ---- 271800050/271800052: the rudder pedal transducer, same shape as
    // the sidestick pair above but on the real rudder pedal axis
    // (prim.rs:1143,1688). FCOM PRO-ABN-ECAM p.5062 gives phases
    // [4,5,6,7] for the plain FAULT; p.5067 (SENSOR FAULT) shows no icons
    // (an unannunciated "crew awareness" FCOM entry, `E-FCTL-FCOM.json`'s
    // own note against `271800052`), so that one keeps the FCDC-sibling
    // inhibit per the addendum.
    v.push(
        proc(
            271_800_050,
            "F/CTL RUDDER PEDAL FAULT",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_RUDDER_PEDAL_FAULT").on(), network_alive()]),
            "both channels of the rudder pedal position transducer invalid -- deep::flight_controls's new sensors::DualTransducer on the real rudder pedal axis; FCOM PRO-ABN-ECAM p.5062",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[4, 5, 6, 7])
        .items(0, Vec::new()),
    );
    v.push(
        proc(
            271_800_052,
            "F/CTL RUDDER PEDAL SENSOR FAULT",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_RUDDER_PEDAL_SENSOR_FAULT").on(), network_alive()]),
            "the rudder pedal transducer's two channels disagree while at least one still reads -- FCOM PRO-ABN-ECAM p.5067 (no FCOM-decoded phase bar; kept the FCDC-sibling inhibit per the addendum)",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(phase::TAKEOFF_AND_LANDING)
        .items(0, Vec::new()),
    );

    // ---- 271800045/271800046/271800047: PRIM/SEC software or pin-
    // programming identity disagreement -- a genuinely new component
    // (`E-FCTL-DESIGN.md` section 3.3), since this port's compiled PRIM/SEC
    // carries no per-unit version identity to compare (grepped every
    // A380{Prim,Sec}Computer*_types.h for "version"/"standard"/
    // "part_number"/"identific": no hits). `271800045`/`271800047` are the
    // *same* underlying check (`FCTL_PRIM_PIN_PROG_DISAGREE` ==
    // `FCTL_PRIM_VERSIONS_DISAGREE`, `live.rs`'s own publish) under
    // FlyByWire's two separate titles. FCOM PRO-ABN-ECAM p.5058/p.5059 show
    // no icons (unannunciated "crew awareness" entries); FCDC-sibling
    // inhibit/confirm stands per the addendum.
    v.push(
        proc(
            271_800_045,
            "F/CTL PRIM VERSIONS DISAGREE",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_PRIM_VERSIONS_DISAGREE").on(), network_alive()]),
            "the three PRIMs' configured identity tags do not all agree -- new deep::flight_controls component 27_fctl.prim_pin_prog (E-FCTL-DESIGN.md section 3.3); FCOM PRO-ABN-ECAM p.5058",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(phase::TAKEOFF_AND_LANDING)
        .items(0, Vec::new()),
    );
    v.push(
        proc(
            271_800_046,
            "F/CTL SEC VERSIONS DISAGREE",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_SEC_VERSIONS_DISAGREE").on(), network_alive()]),
            "the three SECs' configured identity tags do not all agree -- new deep::flight_controls component 27_fctl.sec_pin_prog; FCOM PRO-ABN-ECAM p.5058",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(phase::TAKEOFF_AND_LANDING)
        .items(0, Vec::new()),
    );
    v.push(
        proc(
            271_800_047,
            "F/CTL PRIMs PIN PROG DISAGREE",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_PRIM_PIN_PROG_DISAGREE").on(), network_alive()]),
            "the three PRIMs' hardware pin-programming does not all agree -- same underlying check as 271800045, published under FlyByWire's second title; FCOM PRO-ABN-ECAM p.5059",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(phase::TAKEOFF_AND_LANDING)
        .items(0, Vec::new()),
    );

    // =========================================================================
    // Coordinator follow-up (2026-09-27): re-examining the 13 ids the first
    // pass moved MODEL -> UNSOURCED against the user's rule that a missing
    // cause gets modelled, not dropped. See `registry.rs`'s own doc comments
    // on each new component for the full sourcing; summarised per id below.
    // =========================================================================

    // ---- 271800018: the rate gyro pair feeding the flight control laws IS
    // a real, already-modelled, already-failable quantity in this port
    // (`physics::adirs`'s own strapdown-IRS sensor model, threaded through
    // `Truth::body_rate_*_raw` -- `registry.rs`'s own doc on `27_fctl.
    // rate_gyro_*`). The FCOM's own text ("Two gyrometers... are failed")
    // does not say which axis pair, and this chapter's `UNWIRED.md` count
    // has exactly one id here, so this fires on any axis's pair failing
    // outright, not three separate ids. FCOM PRO-ABN-ECAM p.5092: no icons
    // (unannunciated "crew awareness"); FCDC-sibling level/confirm stands,
    // its own decoded phase bar used for inhibit.
    v.push(
        proc(
            271_800_018,
            "F/CTL TWO GYROMETERs FAULT",
            Level::Caution,
            sd_page::FCTL,
            all(vec![
                any(vec![var("FCTL_RATE_GYRO_PITCH_FAULT").on(), var("FCTL_RATE_GYRO_ROLL_FAULT").on(), var("FCTL_RATE_GYRO_YAW_FAULT").on()]),
                network_alive(),
            ]),
            "both channels of one axis's rate-gyro pair invalid -- real sensed body rate from physics::adirs's own strapdown-IRS model, not an invented input; FCOM PRO-ABN-ECAM p.5092",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
        .items(0, Vec::new()),
    );

    // ---- 271800026/271800028: the F.O. sidestick, modelled at a fixed
    // neutral position (`registry.rs`'s own doc: a real transducer pair
    // this port's cockpit never moves, but the FCOM's own trigger for
    // neither id requires deflection). FCOM PRO-ABN-ECAM p.5043 (FAULT,
    // caution, no decoded bar -- FCDC-sibling inhibit) / p.5077 (SENSOR
    // FAULT, shared with 271800027/271800028's own generic procedure, a
    // real decoded bar).
    v.push(
        proc(
            271_800_026,
            "F/CTL R SIDESTICK FAULT",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_R_SIDESTICK_PITCH_FAULT").on(), var("FCTL_R_SIDESTICK_ROLL_FAULT").on(), network_alive()]),
            "both channels of the F.O.'s sidestick pitch AND roll transducers invalid -- modelled at the same fixed neutral this port's cockpit holds that stick at; FCOM PRO-ABN-ECAM p.5043",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(phase::TAKEOFF_AND_LANDING)
        .items(0, Vec::new()),
    );
    v.push(
        proc(
            271_800_028,
            "F/CTL R SIDESTICK SENSOR FAULT",
            Level::Caution,
            sd_page::FCTL,
            all(vec![any(vec![var("FCTL_R_SIDESTICK_PITCH_SENSOR_FAULT").on(), var("FCTL_R_SIDESTICK_ROLL_SENSOR_FAULT").on()]), network_alive()]),
            "the F.O.'s sidestick pitch or roll channels disagree while at least one still reads -- a channel drifting off a shared neutral still trips the disagreement monitor; FCOM PRO-ABN-ECAM p.5077",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
        .items(0, Vec::new()),
    );

    // ---- 271800029: the FCOM's own condition is a literal 2-of-3
    // accelerometer vote per wing (`registry.rs`'s `27_fctl.
    // load_alleviation`), not the aerodynamic envelope the earlier pass
    // objected to modelling -- the FAULT is the accelerometer voting
    // failure itself. FCOM PRO-ABN-ECAM p.5040: no icons; FCDC-sibling
    // level/confirm, its own decoded bar for inhibit.
    v.push(
        proc(
            271_800_029,
            "F/CTL LOAD ALLEVIATION FAULT",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_LOAD_ALLEVIATION_FAULT").on(), network_alive()]),
            "two of the three accelerometers used by the load alleviation function have failed in one wing -- FCOM PRO-ABN-ECAM p.5040's own literal 2-of-3 vote, new deep::flight_controls component 27_fctl.load_alleviation",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10])
        .items(0, Vec::new()),
    );

    // ---- 271800033-271800035 / 271800039-271800041 / 271800042-271800044:
    // the FCOM's own triggering text ("PRIM n has lost the capacity to
    // control an elevator/rudder actuator" / "PRIM n detects that one
    // sidestick sensor is failed") describes a fault internal to that one
    // PRIM's own command or monitoring circuitry -- not "which hydraulic
    // system backs the actuator" (`A380PrimComputerFctl.cpp`'s real
    // `elevator_n_avail` semantics, the first pass's correct objection to
    // the original design). New per-PRIM components
    // (`registry.rs`: `27_fctl.prim_{1,2,3}_{elevator,rudder}_channel`,
    // `27_fctl.prim_{1,2,3}_sidestick_monitor`), each gated on that PRIM's
    // own health so FlyByWire's already-wired whole-unit `271800036`-`038`
    // is never double counted. FCOM PRO-ABN-ECAM p.5055 (elevator)/p.5056
    // (rudder)/p.5057 (sidestick sensor): no icons; FCDC-sibling level/
    // confirm, each page's own decoded bar for inhibit.
    for n in 1..=3u64 {
        v.push(
            proc(
                271_800_032 + n,
                match n {
                    1 => "F/CTL PRIM 1 ELEVATOR ACTUATOR FAULT",
                    2 => "F/CTL PRIM 2 ELEVATOR ACTUATOR FAULT",
                    _ => "F/CTL PRIM 3 ELEVATOR ACTUATOR FAULT",
                },
                Level::Caution,
                sd_page::FCTL,
                all(vec![var(&format!("FCTL_PRIM_{n}_ELEVATOR_CHANNEL_FAULT")).on(), network_alive()]),
                "that PRIM's own elevator command channel has failed while the PRIM itself stays healthy -- new deep::flight_controls per-PRIM channel component; FCOM PRO-ABN-ECAM p.5055",
            )
            .confirm(FCDC_SIBLING_CONFIRM_S)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
        );
    }
    for n in 1..=3u64 {
        v.push(
            proc(
                271_800_038 + n,
                match n {
                    1 => "F/CTL PRIM 1 RUDDER ACTUATOR FAULT",
                    2 => "F/CTL PRIM 2 RUDDER ACTUATOR FAULT",
                    _ => "F/CTL PRIM 3 RUDDER ACTUATOR FAULT",
                },
                Level::Caution,
                sd_page::FCTL,
                all(vec![var(&format!("FCTL_PRIM_{n}_RUDDER_CHANNEL_FAULT")).on(), network_alive()]),
                "that PRIM's own rudder command channel has failed while the PRIM itself stays healthy -- new deep::flight_controls per-PRIM channel component; FCOM PRO-ABN-ECAM p.5056",
            )
            .confirm(FCDC_SIBLING_CONFIRM_S)
            .inhibit(&[3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
        );
    }
    for n in 1..=3u64 {
        v.push(
            proc(
                271_800_041 + n,
                match n {
                    1 => "F/CTL PRIM 1 SIDESTICK SENSOR FAULT",
                    2 => "F/CTL PRIM 2 SIDESTICK SENSOR FAULT",
                    _ => "F/CTL PRIM 3 SIDESTICK SENSOR FAULT",
                },
                Level::Caution,
                sd_page::FCTL,
                all(vec![var(&format!("FCTL_PRIM_{n}_SIDESTICK_MONITOR_FAULT")).on(), network_alive()]),
                "that PRIM's own sidestick-sensor monitoring circuit has failed while the PRIM itself stays healthy -- new deep::flight_controls per-PRIM channel component; FCOM PRO-ABN-ECAM p.5057",
            )
            .confirm(FCDC_SIBLING_CONFIRM_S)
            .inhibit(&[2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            .items(0, Vec::new()),
        );
    }

    // =========================================================================
    // The deferred 272800009-272800027 SFCC lever / wingtip-brake group
    // (section 3.4). Of the 12, 4 are honestly wirable; the other 8
    // (272800009-272800012, 272800013, 272800025-272800027) are UNSOURCED --
    // see the module doc addendum below for why each one specifically.
    // =========================================================================

    // ---- 272800014/272800015: FCOM PRO-ABN-ECAM p.5112 gives the real
    // condition literally -- "Communication between the FLAPS lever and
    // SFCC 1(2) is lost" -- a discrete per-channel comm-loss fault (new
    // component `27_fctl.flap_lever_csu`), not the position-accuracy
    // problem `272800013` runs into (see the UNSOURCED note there).
    v.push(
        proc(
            272_800_014,
            "F/CTL FLAPS LEVER SYS 1 FAULT",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_FLAPS_LEVER_SYS_1_FAULT").on(), network_alive()]),
            "communication between the flaps lever and SFCC 1 is lost -- FCOM PRO-ABN-ECAM p.5112's own literal wording, new deep::flight_controls component 27_fctl.flap_lever_csu",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(0, Vec::new()),
    );
    v.push(
        proc(
            272_800_015,
            "F/CTL FLAPS LEVER SYS 2 FAULT",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_FLAPS_LEVER_SYS_2_FAULT").on(), network_alive()]),
            "communication between the flaps lever and SFCC 2 is lost -- FCOM PRO-ABN-ECAM p.5112's own literal wording, new deep::flight_controls component 27_fctl.flap_lever_csu",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[3, 4, 5, 6, 7, 9, 10])
        .items(0, Vec::new()),
    );

    // ---- 272800016/272800024: FCOM PRO-ABN-ECAM p.5114/p.5134 -- "The
    // wing-tip brakes have locked the flaps/slats". This is exactly the
    // already-published wingtip-brake-commanded state
    // (`FCTL_{FLAP,SLAT}_WINGTIP_BRAKE_ON`, `high_lift.rs`'s own
    // `HighLiftPair` asymmetry monitor) -- no new component needed, and no
    // separate "_HOLDING" alias invented: this port's brake model achieves
    // full holding torque the instant it is commanded unless a
    // `wingtip_brake_fail` fault is also armed, so "commanded" and "holding"
    // are the same observable state here.
    v.push(
        proc(
            272_800_016,
            "F/CTL FLAPS LOCKED",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_FLAP_WINGTIP_BRAKE_ON").on(), network_alive()]),
            "the flap wingtip brake has engaged to stop an asymmetry/runaway/overspeed condition -- FCOM PRO-ABN-ECAM p.5114; already-published high_lift.rs signal, no new component",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[4, 5, 6, 7, 10])
        .items(0, Vec::new()),
    );
    v.push(
        proc(
            272_800_024,
            "F/CTL SLATS LOCKED",
            Level::Caution,
            sd_page::FCTL,
            all(vec![var("FCTL_SLAT_WINGTIP_BRAKE_ON").on(), network_alive()]),
            "the slat wingtip brake has engaged to stop an asymmetry/runaway/overspeed condition -- FCOM PRO-ABN-ECAM p.5134; already-published high_lift.rs signal, no new component",
        )
        .confirm(FCDC_SIBLING_CONFIRM_S)
        .inhibit(&[4, 5, 6, 7, 10])
        .items(0, Vec::new()),
    );
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

    /// Phase 2 plus the coordinator follow-up (E-FCTL) wires exactly the 26
    /// ids this module's addendum names -- not more (an accidental extra
    /// push) and not fewer (a forgotten one). Per-alert behaviour (healthy
    /// silent / cause raises / cold-dark silent) is in `fbw_tests.rs`, which
    /// has the `run`/`holds`/`wiring` helpers this module does not.
    #[test]
    fn wire_pushes_exactly_the_twenty_six_wired_ids() {
        let mut v = Vec::new();
        wire(&mut v);
        let mut ids: Vec<u64> = v.iter().map(|p| p.id).collect();
        ids.sort_unstable();
        assert_eq!(
            ids,
            vec![
                271_800_001,
                271_800_002,
                271_800_018,
                271_800_025,
                271_800_026,
                271_800_027,
                271_800_028,
                271_800_029,
                271_800_033,
                271_800_034,
                271_800_035,
                271_800_039,
                271_800_040,
                271_800_041,
                271_800_042,
                271_800_043,
                271_800_044,
                271_800_045,
                271_800_046,
                271_800_047,
                271_800_050,
                271_800_052,
                272_800_014,
                272_800_015,
                272_800_016,
                272_800_024,
            ],
            "ata27::wire's id set drifted from this module's own addendum -- update both together"
        );
    }

    /// Every id this module pushes has a non-empty note citing its FCOM
    /// page (Task B's own rule: every threshold/mapping cites a source).
    #[test]
    fn every_phase_2_entry_cites_an_fcom_page() {
        let mut v = Vec::new();
        wire(&mut v);
        for p in &v {
            assert!(p.note.contains("FCOM PRO-ABN-ECAM p."), "{}: note does not cite an FCOM page: {:?}", p.id, p.note);
        }
    }
}
