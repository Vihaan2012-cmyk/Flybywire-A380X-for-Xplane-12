//! Cheap wins: FlyByWire abnormal-sensed procedures that were left unwired
//! only because a small addition to a deep area was missing (a value that
//! the underlying physics already computes but that no `deep` area
//! published), per `docs/deep/fbw_unwired.md` category (c).
//!
//! # ENG n OIL FILTER CLOGGED (`701800081`..`701800084`)
//!
//! `physics::engine::oil` already models the oil filter and its bypass
//! valve (`OilState::filter_bypassed`, cracked at `FILTER_BYPASS_PSI` once
//! `filter_clog` has raised the element's viscous drop that far) but,
//! before this file, nothing carried that bit out of the physics layer.
//! It now does: `EngineOutputs::oil_filter_bypassed` (already existed) is
//! read back from the `ENGINE_OIL_FILTER_BYPASS:n` dataref
//! `engine_commands.rs` was already writing, into a new
//! `Truth::engine_oil_filter_bypassed` field (`deep/live.rs`,
//! `deep/plugin.rs`), and `deep::engine_accessories` now publishes it as
//! `A32NX_ENG_n_OIL_FILTER_BYPASSED` (`engine_accessories/live.rs`). A
//! filter that has clogged enough to crack its own bypass valve is exactly
//! what FlyByWire's OIL FILTER CLOGGED procedure announces, so the
//! published bit is the trigger with nothing invented or approximated.
//!
//! # What is not here
//!
//! FEED TK n TEMP HI (`281800039`..`281800042`), the tank-capacity
//! comparisons (`281800097`/`281800093`/`281800010`..`281800013`/
//! `281800051`) and L/G ABNORM OLEO PRESS (`320800031`) were re-examined
//! for this pass and are still correctly unwired:
//!
//! * A feed-tank index table already exists in effect --
//!   `deep::fuel::geometry::Tank::{Feed1,Feed2,Feed3,Feed4}` carry the same
//!   discriminants (`2,5,6,9`) that `deep::fuel::live`'s own
//!   `FUEL_TANK_TEMP_C:n` publishes under -- but no high-temperature limit
//!   for an A380 feed tank is sourced anywhere in either repository.
//!   FlyByWire's own `ata28.ts` carries no numeric limit for `281800039`,
//!   and this port has no A380 fuel SD page with a coloured band to read
//!   one off, unlike the Trent 900 oil temperature FlyByWire's own ENGINE
//!   page draws amber above 177 C. Inventing one is exactly what the
//!   no-guessing rule forbids, so this stays unwired.
//! * The tank-capacity comparisons need a per-tank capacity table, which
//!   `deep::fuel::geometry` does have (`TankShape`'s volumes, sourced from
//!   FlyByWire's own `flight_model.cfg`) -- but WING/TRIM TK OVERFLOW,
//!   COLLECTOR CELL n NOT FULL and INR TKs QTY LO each also need a
//!   *servicing/collector-cell target*, not just a maximum, that neither
//!   repository states, so wiring them off "quantity vs. maximum capacity"
//!   would fire (or never fire) on the wrong band.
//! * `GEAR_STRUT_GAS_CHARGE_FRACTION:n` is a charge fraction with no
//!   sourced servicing band (a real oleo's abnormal-pressure limits are
//!   temperature- and load-dependent maintenance-manual figures, not a
//!   fixed fraction), so `320800031` stays unwired for the same reason.
//!
//! # THRUST LOSS / `240800003`, re-examined and left as recorded
//!
//! * `701800133`..`701800136` ENG n THRUST LOSS: confirmed still
//!   unwirable. `engine_accessories/live.rs`'s own `thrust_abnormal` flag
//!   is `error > THRUST_ABNORMAL_TOLERANCE` where `error` is the identical
//!   `metering_error_fraction()` that `fmu_fault` (`ENG n FADEC FUEL
//!   METERING FAULT`) is raised from at a lower tolerance
//!   (`METERING_TOLERANCE`), so `thrust_abnormal` is a strict subset of
//!   `fmu_fault` and can never be true on its own -- it names a metering
//!   fault a second time, not an independent thrust loss.
//! * `240800003` ELEC AC BUS 1+2 & DC BUS 1 FAULT: confirmed still
//!   unwirable. It is a compound condition that includes AC BUS 1 lost,
//!   and `deep::electrical`'s own `ELEC AC BUS 1 FAULT` already announces
//!   that on its own with nothing here able to suppress it, so wiring
//!   `240800003` would double-annunciate AC BUS 1 the moment it is the
//!   only one of the three actually down. Left unwired until that alert is
//!   withdrawn or given a suppression hook, as the doc already says.

use super::{phase, proc, sd_page, FbwProc};
use crate::deep::api::{var, Level};

pub fn wire(v: &mut Vec<FbwProc>) {
    for eng in 1..=4u32 {
        let i = u64::from(eng) - 1;
        v.push(
            proc(
                701_800_081 + i,
                "ENG n OIL FILTER CLOGGED",
                // Amber in FlyByWire's own title (`\x1b<4m`).
                Level::Caution,
                sd_page::ENG,
                var(&format!("A32NX_ENG_{eng}_OIL_FILTER_BYPASSED")).on(),
                "the oil filter's own bypass valve has cracked, which physics::engine::oil only does once filter_clog has raised the element's viscous drop past FILTER_BYPASS_PSI -- a clogged filter, not an approximation of one",
            )
            // 5 s: long enough that a single frame of a cold, thick-oil
            // start (which also opens the bypass, briefly, on a healthy
            // filter) is not a caution.
            .confirm(5.0)
            .inhibit(phase::ENG_56),
        );
    }
}

#[cfg(test)]
mod tests {
    use crate::deep::api::Cond;
    use crate::deep::engine_accessories::live::live_system;
    use crate::deep::live::{Area, Faults, Truth};
    use std::collections::BTreeMap;

    /// Arms the real oil-filter-clog condition (`Truth::
    /// engine_oil_filter_bypassed`, exactly what `physics::engine::oil`
    /// sets `OilState::filter_bypassed` from), ticks the real
    /// `deep::engine_accessories` area, and asserts the published bit --
    /// and therefore this procedure's own trigger -- flips.
    #[test]
    fn a_clogging_oil_filter_reaches_the_published_bypass_bit_and_the_procedure_trigger() {
        let wirings = super::super::wirings();
        let proc = wirings.iter().find(|p| p.id == 701_800_081).expect("701800081 wired");

        let mut truth = Truth { dt_s: 0.02, engine_running: [true; 4], ..Truth::default() };
        let mut area = live_system();
        let clean = tick(&mut *area, &truth);
        assert_eq!(clean.get("A32NX_ENG_1_OIL_FILTER_BYPASSED"), Some(&0.0));
        assert!(!eval(&proc.trigger, &clean), "must not be armed on a clean engine");

        truth.engine_oil_filter_bypassed[0] = true;
        let clogged = tick(&mut *area, &truth);
        assert_eq!(clogged.get("A32NX_ENG_1_OIL_FILTER_BYPASSED"), Some(&1.0), "the bypass bit must reach the published var");
        assert!(eval(&proc.trigger, &clogged), "the procedure's own trigger must flip once the published bit is set");
        // Engine 2 was never armed: its own bit, and its own procedure,
        // must stay clear.
        assert_eq!(clogged.get("A32NX_ENG_2_OIL_FILTER_BYPASSED"), Some(&0.0));
    }

    fn tick(area: &mut dyn Area, truth: &Truth) -> BTreeMap<String, f64> {
        area.tick(truth, &Faults::default());
        let mut out = BTreeMap::new();
        area.publish(&mut |k: &str, v: f64| {
            out.insert(k.to_string(), v);
        });
        out
    }

    fn eval(c: &Cond, vars: &BTreeMap<String, f64>) -> bool {
        c.eval(&|name| vars.get(name).copied().unwrap_or(0.0))
    }
}
