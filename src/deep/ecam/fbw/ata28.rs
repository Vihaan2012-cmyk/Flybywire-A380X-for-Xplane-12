//! ATA 28 -- fuel. 97 of FlyByWire's 106 procedures here are unwired.
//!
//! `deep::fuel` publishes a great deal (per-tank quantity, true quantity,
//! temperature, leak rate, FQMS confidence, crossfeed and jettison valve
//! state), but almost all of the unwired ATA 28 procedures are about
//! *pumps and valves the area does not publish individually*: the four
//! feed-tank main and standby pumps, the inner/mid/outer tank fwd and aft
//! pumps, the four crossfeed valves, the four engine LP valves and the APU
//! feed valve are all modelled as transfer paths rather than as named
//! units, so there is nothing that can say which one failed.
//!
//! The exception is the trim tank, and it is an exception `deep::fuel`
//! made deliberately: `FUEL_TRIM_PUMP_DEGRADATION:1`/`:2` exist precisely
//! because the trim transfer path's own fault detector takes the *healthier*
//! of the two mutually redundant pumps and so can never show one pump's
//! failure, and its own source comment says why that per-pump reading is
//! published anyway -- "a real aircraft still shows a LO PRESS caution on
//! the specific failed pump even though its redundant twin keeps the
//! transfer going" (`deep/fuel/live.rs:398-409`). That is exactly what
//! FlyByWire's `281800089`/`281800090`/`281800091` are.
//!
//! # What is deliberately left unwired
//!
//! * `281800073` FUEL LEAK DETECTED, `281800054` JETTISON FAULT,
//!   `281800094` TRIM TK XFR FAULT, `281800014`..`281800017` CROSSFEED VLV
//!   FAULT, `281800045`/`281800049` FQI/GAUGING FAULT, `281800086` FUEL
//!   TEMP LO, `281800006` AUTO GND XFR FAULT: `deep::fuel`'s own
//!   `registry.rs` already registers `FUEL LEAK`, `FUEL JETTISON FAULT`,
//!   `FUEL TRIM TK TRANSFER FAULT`, `FUEL WING XFEED FAULT`, `FUEL QTY
//!   INDICATION FAULT`, `FUEL FOB LO TEMP` and `FUEL AUTO CG XFR FAULT` on
//!   the same published variables.
//! * `281800039`..`281800042` FEED TK n TEMP HI: `FUEL_TANK_TEMP_C:n` is
//!   published for all eleven tanks, but nothing in this repository records
//!   which of the eleven indices are the four feed tanks, and no
//!   high-temperature limit for a feed tank is sourced anywhere. Both would
//!   have to be guessed.
//! * `281800097`/`281800093` WING/TRIM TK OVERFLOW, `281800010`..`281800013`
//!   COLLECTOR CELL n NOT FULL, `281800051` INR TKs QTY LO: these compare a
//!   quantity against a tank capacity, and no per-tank capacity table is
//!   published or recorded here.
//! * The ~40 transfer, refuel, CG and weight-and-balance procedures
//!   (`281800005`, `281800007`..`281800009`, `281800022`, `281800050`,
//!   `281800074`..`281800085`, `281800095`..`281800100`): these annunciate
//!   FQMS *modes and computations* (auto ground transfer completed,
//!   predicted CG out of take-off range, refuel data vs FMS disagree) that
//!   this port has no fuel management computer to produce.

use super::{phase, proc, sd_page, FbwProc};
use crate::deep::api::{all, var, Cond, Level};

/// How far a trim pump's own degradation has to go before it is a failed
/// pump rather than a worn one. **GENERIC**: `deep::fuel` publishes this as
/// a 0..1 fraction of lost delivery with no threshold of its own, and half
/// its rated delivery is the point at which the pump can no longer carry
/// the transfer on its own -- which is the condition a LO PRESS caution on
/// that pump annunciates. Deliberately well clear of the wear a healthy
/// pump accumulates, so this is a failure and not a maintenance trend.
const PUMP_FAILED_FRACTION: f64 = 0.5;

fn pump_failed(n: u32) -> Cond {
    var(&format!("FUEL_TRIM_PUMP_DEGRADATION:{n}")).ge(PUMP_FAILED_FRACTION)
}

pub fn wire(v: &mut Vec<FbwProc>) {
    // Trim pump 1 is the left pump and 2 the right, in the same order
    // `deep::fuel::live`'s own `trim_pump_degradation: [f64; 2]` and its
    // `registry.rs` failure ids use.
    //
    // The single-pump procedures are suppressed while the both-pumps one is
    // active, the way FlyByWire's own paired entries are.
    v.push(
        proc(281_800_089, "FUEL TRIM TK L PMP FAULT", Level::Caution, sd_page::FUEL, all(vec![pump_failed(1), var("FUEL_TRIM_PUMP_DEGRADATION:2").lt(PUMP_FAILED_FRACTION)]), "the left trim pump has lost at least half its delivery while the right one has not")
            .confirm(5.0)
            .inhibit(phase::TAKEOFF_AND_LANDING)
            .suppressed_by(&[281_800_091]),
    );
    v.push(
        proc(281_800_090, "FUEL TRIM TK R PMP FAULT", Level::Caution, sd_page::FUEL, all(vec![pump_failed(2), var("FUEL_TRIM_PUMP_DEGRADATION:1").lt(PUMP_FAILED_FRACTION)]), "the right trim pump has lost at least half its delivery while the left one has not")
            .confirm(5.0)
            .inhibit(phase::TAKEOFF_AND_LANDING)
            .suppressed_by(&[281_800_091]),
    );
    v.push(
        proc(281_800_091, "FUEL TRIM TK L+R PMPs FAULT", Level::Caution, sd_page::FUEL, all(vec![pump_failed(1), pump_failed(2)]), "both trim pumps have lost at least half their delivery, which is when the trim transfer itself stops")
            .confirm(5.0)
            .inhibit(phase::TAKEOFF_AND_LANDING),
    );
}
