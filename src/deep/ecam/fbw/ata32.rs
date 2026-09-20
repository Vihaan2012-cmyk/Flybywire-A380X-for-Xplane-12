//! ATA 32 -- landing gear, brakes, steering and wheels. 62 of the 93
//! unwired procedures in `ata31-32-33.ts` are in this chapter.
//!
//! `deep::gear_structure` publishes a real retraction model -- per-leg
//! `GEAR_POSITION`, `GEAR_DOOR_POSITION`, `GEAR_DOWNLOCKED`,
//! `GEAR_UPLOCKED` and their separately-modelled proximity-sensed twins
//! `SENSED_GEAR_*`, plus the lever position the legs are actually obeying
//! (`GEAR_LEVER_POSITION_REQUEST`) -- and `physics::tyre` gives every wheel
//! its own nitrogen pressure through `DEEP_TYRE_PRESSURE_SENSED_PA:n`.
//!
//! # What is deliberately left unwired
//!
//! * `320800039` GEAR NOT LOCKED DOWN, `320800045` SYSTEM DISAGREE,
//!   `320800001`..`320800007` A-SKID FAULT, `320800023` PARK BRK PRESS LO,
//!   `320900006` TIRE DAMAGE SUSPECTED: `deep::gear_structure`'s own
//!   `registry.rs` already registers `L/G GEAR NOT DOWNLOCKED`, `L/G GEAR
//!   DISAGREE`, `L/G BRAKES ANTISKID N/U`, `L/G PARK BRAKE LO PR` and
//!   `NOSE L/G BIRD DAMAGE` on the same variables.
//! * The 30 brake and steering control procedures (`320800010`..`320800029`,
//!   `320800047`..`320800062`): these annunciate the *braking and steering
//!   control system* -- normal/alternate/emergency brake selection, the two
//!   BSCU channels, the selector valves, the tiller and pedal transducers --
//!   and `deep::gear_structure` models the wheels, the antiskid channels and
//!   the shimmy, not the BSCU. Nothing published can say which channel or
//!   which valve.
//! * `320800031`/`320800043` ABNORM OLEO PRESS and its monitoring:
//!   `GEAR_STRUT_GAS_CHARGE_FRACTION:n` is published, but it is a charge
//!   *fraction* with no sourced servicing band to compare against, and the
//!   A380's own oleo pressure check is against a strut-extension-versus-
//!   weight table this port has no equivalent of.
//! * `320800042` GRVTY EXTN FAULT, `320800046` WEIGHT ON WHEELS FAULT,
//!   `320800033`..`320800035` L/G CTL FAULT: the free-fall extension system,
//!   the WOW voting and the two LGCIS channels are not modelled.

use super::{item, phase, proc, sd_page, FbwProc};
use crate::deep::api::{all, any, var, Cond, Level};

/// The five legs `deep::gear_structure` publishes, `:1`..`:5` (nose, left
/// wing, right wing, left body, right body -- the area's own index order).
const LEGS: [u32; 5] = [1, 2, 3, 4, 5];

/// The gear lever is selected up. `GEAR_LEVER_POSITION_REQUEST` is
/// published as 1 for lever down and 0 for lever up
/// (`deep/gear_structure/live.rs:434`, `b(self.gear_lever_down)`).
fn lever_up() -> Cond {
    var("GEAR_LEVER_POSITION_REQUEST").off()
}

fn lever_down() -> Cond {
    var("GEAR_LEVER_POSITION_REQUEST").on()
}

/// How far below its cold service pressure a tyre has to fall to be called
/// low. **GENERIC**, 90 % of `physics::tyre::COLD_PRESSURE_PA`
/// (1 550 000 Pa, itself flagged generic there as the widely-published
/// ~15.5 bar figure for A380 main-gear tyres): the industry servicing
/// criterion is that a tyre more than 10 % below its reference has lost gas
/// and must be attended to. Conservative in the right direction -- brake
/// heat raises a tyre's pressure, never lowers it, so a hot tyre cannot
/// reach this by temperature alone.
const TYRE_LOW_PA: f64 = 0.9 * crate::physics::tyre::COLD_PRESSURE_PA;

/// Every tyre pressure transducer the areas actually publish, by name.
///
/// Read from `Deep::published_names()` rather than written down as a count:
/// how many wheels carry a transducer is `deep::sensors`' business, it
/// changed twice while this module was being written, and a hard-coded
/// range is wrong in both directions -- too long and the trigger reads a
/// variable nobody publishes (which `Cond::eval` reads as 0 Pa, an
/// instantly-true "tyre flat"), too short and a wheel added later is
/// silently outside the alert. Building the condition from the published
/// names themselves can be neither.
///
/// This makes [`super::wirings`] depend on constructing the areas once. It
/// is called once at start-up, from `patches::source_patches`, and once per
/// test; `Area::publish` is a pure read of an area's own state, so the
/// areas built here are discarded with no effect.
fn tyre_pressure_vars() -> Vec<String> {
    let mut v: Vec<String> = crate::deep::live::all_areas().published_names().into_iter().filter(|n| n.contains("DEEP_TYRE_PRESSURE_SENSED_PA:")).collect();
    v.sort();
    v.dedup();
    v
}

pub fn wire(v: &mut Vec<FbwProc>) {
    // ---- L/G GEAR NOT LOCKED UP.
    //
    // The lever is up and a leg is not up-locked. 30 s confirmation, which
    // is comfortably beyond this port's own modelled travel (
    // `gear_structure::retraction`'s `GEAR_NOMINAL_TRAVEL_S` = 8 s plus
    // `DOOR_NOMINAL_TRAVEL_S` = 4 s at each end), so a normal retraction
    // never reaches it. **GENERIC** in that the real aircraft's own
    // retraction timer is not published anywhere; derived from the port's
    // own travel times with better than a factor of two in hand.
    v.push(
        proc(
            320_800_040,
            "L/G GEAR NOT LOCKED UP",
            Level::Caution,
            sd_page::WHEEL,
            all(vec![lever_up(), any(LEGS.iter().map(|n| var(&format!("GEAR_UPLOCKED:{n}")).off()).collect())]),
            "the lever is up and at least one of the five legs has not up-locked well after the modelled retraction time",
        )
        .confirm(30.0)
        .inhibit(phase::TAKEOFF_AND_LANDING)
        .items(
            7,
            // [1] "L/G LEVER ... RECYCLE" is a cycle, which a lever
            // *position* cannot show; left crew-actioned. [2] "L/G LEVER
            // ... DOWN" is a position, and this is the same lever variable
            // `deep::gear_structure::registry`'s own `L/G GEAR NOT
            // DOWNLOCKED` procedure reads back.
            vec![item(2).checked(lever_down())],
        ),
    );

    // ---- L/G DOORS NOT CLOSED.
    //
    // The lever is up and every leg has up-locked -- so the retraction
    // sequence is finished and the doors should be shut -- but a door is
    // still off its stop. 20 s, four times the modelled door travel.
    v.push(
        proc(
            320_800_036,
            "L/G DOORS NOT CLOSED",
            Level::Caution,
            sd_page::WHEEL,
            all(vec![
                lever_up(),
                all(LEGS.iter().map(|n| var(&format!("GEAR_UPLOCKED:{n}")).on()).collect()),
                any(LEGS.iter().map(|n| var(&format!("GEAR_DOOR_POSITION:{n}")).gt(0.05)).collect()),
            ]),
            "retraction is complete on all five legs but a gear door is still open by more than 5 % of its travel",
        )
        .confirm(20.0)
        .inhibit(phase::TAKEOFF_AND_LANDING)
        .items(12, vec![item(7).checked(lever_down()), item(9).checked(lever_up())]),
    );

    // ---- WHEEL TIRE PRESS LO. No items in FlyByWire's own catalogue.
    v.push(
        proc(
            320_800_063,
            "WHEEL TIRE PRESS LO",
            Level::Caution,
            sd_page::WHEEL,
            any(tyre_pressure_vars().iter().map(|n| var(n).lt(TYRE_LOW_PA)).collect()),
            "any wheel's own pressure transducer reading more than 10 % below the cold service pressure physics::tyre models",
        )
        // 10 s: a transducer reading is already filtered by
        // `deep::sensors`, and a tyre cannot lose a tenth of its gas in a
        // transient.
        .confirm(10.0)
        .inhibit(phase::TAKEOFF_AND_LANDING),
    );
}
