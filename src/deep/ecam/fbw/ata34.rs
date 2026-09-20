//! ATA 34 -- navigation. FlyByWire defines 58 unwired abnormal-sensed
//! procedures in `AbnormalSensed/ata34.ts`; the first pass wrote off the
//! whole chapter as "navigation receivers we don't model" without opening
//! them id by id. That call was too broad in one direction and (on a
//! second look) too narrow in another: `deep::sensors` publishes per-ADIRU
//! AoA vane jam/heater state, per-unit GPS validity and per-system
//! static-port degradation that nothing else announces, so those are real
//! wins. It also turned out that `FwsAbnormalSensed.ts` already wires the
//! whole ADR (`340800001`-`006`) and most of the radio-altimeter
//! (`340800053`-`062`) groups itself -- checked directly against its
//! source rather than against the stale "0 wired for ATA 34" total the
//! first pass's chapter count implied -- so those ids are correctly left
//! alone: FlyByWire already gives them a trigger, and adding a second one
//! here would be exactly the double-trigger `no_entry_takes_an_id_
//! flybywire_already_triggers` exists to catch.
//!
//! # What this module wires
//!
//! * **AOA n FAULT** (`340800011`/`012`/`013`) from
//!   `DEEP_AOA_n_{JAMMED,HEATER_FAILED}` per unit -- `deep::sensors::
//!   registry`'s own `NAV AOA DISAGREE` only carries the OR of all three,
//!   so it cannot distinguish which vane, and these per-unit ids are new
//!   information, not a duplicate. `FwsAbnormalSensed.ts` does not wire
//!   any of the three (verified directly against its source).
//! * **GPS 2 FAULT** (`340800035`) and **GPS 1+2 FAULT** (`340800036`) from
//!   `DEEP_GPS_2_VALID`/`DEEP_GPS_1_VALID`. `340800034` GPS 1 FAULT is
//!   already `deep::sensors::registry`'s own `NAV GPS 1 FAULT` on the same
//!   `DEEP_GPS_1_VALID == 0` and stays unwired; `035`/`036` are not in
//!   `FwsAbnormalSensed.ts` either.
//! * **STATIC PROBE FAULT** (`340800067`) from
//!   `DEEP_STATIC_{1,2,3,4}_DEGRADED` -- not announced anywhere else and
//!   not in `FwsAbnormalSensed.ts`.
//!
//! # What stays unwired, and why
//!
//! * `340800001`-`340800010` every ADR n FAULT, pairwise combo, the triple
//!   DATA DEGRADED/FAULT and both AIR DATA DISAGREE procedures:
//!   `FwsAbnormalSensed.ts` already keys all of `340800001`-`006` into its
//!   own `ewdAbnormalSensed` map (confirmed by reading the file directly,
//!   not by trusting the chapter's unwired count), so those six already
//!   have a trigger. `007`-`010` were never candidates: `deep::sensors::
//!   live`'s voter publishes a per-channel outlier flag, not a three-way
//!   "all disagree" reading, so a triple-fault condition would have to be
//!   invented.
//! * `340800014` AOA DISAGREE: already `deep::sensors::registry`'s own
//!   `NAV AOA DISAGREE`, the OR of all three vanes' jam/heater flags.
//! * `340800053`-`340800062` RA SYS A/B/C FAULT, the LOST BY PRIM variants
//!   and every pair/triple combo: `340800053`-`055` and `059`-`062` are
//!   already keyed in `FwsAbnormalSensed.ts` (confirmed directly); `056`-
//!   `058` LOST BY PRIM need a PRIM flight-control computer's own health,
//!   not modelled here.
//! * `340800016`-`340800020` CAPT/F.O ALT/ATT/BARO/HDG DISAGREE,
//!   `340800025`/`026` FM/GPS and FM/IR POS DISAGREE, `340800040`-`045`
//!   IR n FAULT and combos, `340800045` IR NOT ALIGNED: no `DEEP_IR_*`
//!   variable exists -- `deep::sensors` publishes air-data and AoA/GPS/RA,
//!   not an inertial reference's attitude or heading solution.
//! * `340800022`-`340800032` FLS/GLS/ILS capability and fault procedures,
//!   `340800037`-`340800039` ILS n FAULT, `340800046`-`340800049` LS n
//!   FAULT/TUNING DISAGREE, `340800015` ARPT NAV FAULT, `340800021` EXTREME
//!   LATITUDE, `340800033` GNSS SIGNAL DEGRADED: navigation receivers and
//!   FM functions this port does not model, exactly the first pass's
//!   finding -- still true for this half of the chapter.
//! * `340800050`/`051` OAT PROBE 1/2 FAULT: no `DEEP_OAT_*` variable;
//!   `deep::sensors` publishes TAT (`DEEP_TAT_n_*`), not a separate OAT
//!   probe, and reusing TAT here would be exactly the kind of unsourced
//!   substitution `docs/deep/BRIEF.md` rule 3 forbids.
//! * `340800052` RA DEGRADED: FlyByWire's own source marks this "error
//!   model not implemented", and `deep::sensors` has no accuracy-degraded
//!   state for the radio altimeter distinct from `IN_RANGE`/`VALID`.
//! * `340800063` RESIDUAL AIR SPEED, `340800071` UNRELIABLE AIR SPEED
//!   INDICATION, `340900003` its WIP twin: these are FWS-computed
//!   verdicts over the ADR voter's own output (residual groundspeed at
//!   touchdown, cross-checked airspeed sources), not a single published
//!   flag -- reproducing the FWS's own logic here would be a guess at
//!   its thresholds.
//! * `340800064`-`066` SIDESLIP PROBE 1/2/3 FAULT: no `DEEP_SIDESLIP_*`
//!   variable is published.
//! * `340800068`-`070` TAT PROBE 1/2/3 FAULT: TAT probe 1 and 2 are
//!   already inside `deep::sensors::registry`'s own combined
//!   `NAV TAT PROBE FAULT` (`DEEP_TAT_1_HEATER_FAILED` OR
//!   `DEEP_TAT_2_HEATER_FAILED`); wiring either individually would put a
//!   second, unit-specific title on the EWD for the same heater failure.
//!   TAT probe 3 has no published variable at all (`deep::sensors` models
//!   two TAT probes, not three).
//! * `340900001`/`340900002` IR ALIGNMENT IN ATT MODE / FLUCTUATING
//!   VERTICAL SPEED: marked `(WIP)` in FlyByWire's own catalogue.
//! * `341800015`-`341800028` ROW/ROP, TCAS, XPDR, TERR SYS, TAWS, GPWS and
//!   `340700001`-`340700003` LDG ELEVN/AIR DATA SELECTION: surveillance and
//!   MEMO/INFO entries outside the abnormal-sensed id range this pass
//!   covers (`3418xxxxx`/`3407xxxxx`, not `3408xxxxx`), and not modelled.

use super::{proc, sd_page, FbwProc};
use crate::deep::api::{all, any, var};
use crate::deep::api::Level;

/// One AOA vane's own fault: jammed (mechanically stuck, or heater failure
/// leading to icing and jam per `deep::sensors::registry`'s own
/// `register_aoa_vane` note) or its heater failed outright. Either is a
/// fault of that specific vane.
fn aoa_fault(n: u32) -> crate::deep::api::Cond {
    any(vec![var(&format!("DEEP_AOA_{n}_JAMMED")).on(), var(&format!("DEEP_AOA_{n}_HEATER_FAILED")).on()])
}

/// One GPS/MMR receiver's own validity flag gone false.
fn gps_invalid(n: u32) -> crate::deep::api::Cond {
    var(&format!("DEEP_GPS_{n}_VALID")).eq(0.0)
}

pub fn wire(v: &mut Vec<FbwProc>) {
    // ---- AOA. Per-vane fault; `deep::sensors::registry`'s own
    // `NAV AOA DISAGREE` only carries the OR of all three, so these are new
    // information, not a duplicate.
    v.push(proc(340_800_011, "NAV AOA 1 FAULT", Level::Caution, sd_page::STATUS, aoa_fault(1), "AoA vane 1 jammed or its heater failed (deep::sensors DEEP_AOA_1_JAMMED / DEEP_AOA_1_HEATER_FAILED)").confirm(2.0).items(0, Vec::new()));
    v.push(proc(340_800_012, "NAV AOA 2 FAULT", Level::Caution, sd_page::STATUS, aoa_fault(2), "AoA vane 2 jammed or its heater failed").confirm(2.0).items(0, Vec::new()));
    v.push(proc(340_800_013, "NAV AOA 3 FAULT", Level::Caution, sd_page::STATUS, aoa_fault(3), "AoA vane 3 jammed or its heater failed").confirm(2.0).items(0, Vec::new()));

    // ---- GPS. GPS 1 FAULT is already deep::sensors::registry's own
    // `NAV GPS 1 FAULT` on the same `DEEP_GPS_1_VALID == 0` and is left
    // unwired; the combo below is new.
    v.push(
        proc(340_800_035, "NAV GPS 2 FAULT", Level::Caution, sd_page::STATUS, gps_invalid(2), "GPS/MMR receiver 2's own validity flag gone false (deep::sensors DEEP_GPS_2_VALID)")
            .confirm(5.0)
            .suppressed_by(&[340_800_036])
            .items(0, Vec::new()),
    );
    v.push(
        proc(340_800_036, "NAV GPS 1+2 FAULT", Level::Caution, sd_page::STATUS, all(vec![gps_invalid(1), gps_invalid(2)]), "both GPS/MMR receivers' validity flags false together")
            .confirm(5.0)
            .items(0, Vec::new()),
    );

    // ---- Static ports. Not announced anywhere else.
    v.push(
        proc(
            340_800_067,
            "NAV STATIC PROBE FAULT",
            Level::Caution,
            sd_page::STATUS,
            any(vec![
                var("DEEP_STATIC_1_DEGRADED").on(),
                var("DEEP_STATIC_2_DEGRADED").on(),
                var("DEEP_STATIC_3_DEGRADED").on(),
                var("DEEP_STATIC_4_DEGRADED").on(),
            ]),
            "at least one system's static-port pair reads degraded (deep::sensors DEEP_STATIC_n_DEGRADED, average_pair's own verdict when one port of a pair is blocked)",
        )
        .confirm(3.0)
        .items(0, Vec::new()),
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::api::Cond;
    use crate::deep::integration::failure_audit::fresh_areas;
    use crate::deep::live::{Faults, Truth};
    use crate::deep::sensors::live::FaultIndex;
    use std::collections::BTreeMap;

    /// A powered aircraft in the air, so the air-data and nav sensors this
    /// chapter reads are not simply unpowered (which a cold-and-dark
    /// default would report as invalid for reasons that have nothing to do
    /// with the faults under test).
    fn flying() -> Truth {
        Truth { dt_s: 0.1, on_ground: false, altitude_ft: 20_000.0, ac_bus_volts: [115.0; 4], dc_bus_volts: [28.0; 2], engine_running: [true; 4], engine_n1_frac: [0.85; 4], ..Truth::default() }
    }

    fn run(truth: Truth, faults: &Faults, frames: usize) -> BTreeMap<String, f64> {
        let mut deep = fresh_areas();
        let mut out = BTreeMap::new();
        for _ in 0..frames {
            out.clear();
            deep.tick(truth.clone(), faults, &mut |n, v| {
                out.insert(n.to_string(), v);
            });
        }
        out
    }

    fn holds(c: &Cond, published: &BTreeMap<String, f64>) -> bool {
        c.eval(&|n: &str| *published.get(n).unwrap_or(&0.0))
    }

    fn wiring(id: u64) -> FbwProc {
        let mut v = Vec::new();
        wire(&mut v);
        v.into_iter().find(|p| p.id == id).unwrap_or_else(|| panic!("{id} is not wired by ata34::wire"))
    }

    /// Every trigger this module writes must read only names
    /// `deep::live::all_areas()` actually publishes -- the same standing
    /// guard `fbw_tests.rs` runs over the whole aggregation, checked here
    /// too so this file fails on its own if it ever drifts.
    #[test]
    fn every_trigger_reads_a_published_variable() {
        use crate::deep::integration::failure_audit::{bare, cond_vars};
        use crate::deep::live::all_areas;
        let published: std::collections::BTreeSet<String> = all_areas().published_names().iter().map(|n| bare(n).to_owned()).collect();
        let mut v = Vec::new();
        wire(&mut v);
        for p in &v {
            let mut names = Vec::new();
            cond_vars(&p.trigger, &mut names);
            for n in names {
                let n = bare(&n);
                assert!(published.contains(n), "{} ({}) reads {n}, which nothing publishes", p.id, p.title);
            }
        }
    }

    /// AOA family: seize vane 2's hinge outright and check the fault lands
    /// on unit 2 alone.
    #[test]
    fn aoa_2_fault_fires_only_on_the_seized_vane() {
        let index = FaultIndex::build();
        let id = index.id("34_nav.aoa_2", "mechanically_stuck");
        assert_ne!(id, 0, "the AoA seizure fault id could not be resolved");

        let healthy = run(flying(), &Faults::default(), 10);
        let armed = run(flying(), &Faults::from_pairs([(id, 1.0)]), 30);

        let aoa1 = wiring(340_800_011);
        let aoa2 = wiring(340_800_012);
        assert!(!holds(&aoa2.trigger, &healthy), "NAV AOA 2 FAULT must be quiet with every vane free");
        assert!(holds(&aoa2.trigger, &armed), "NAV AOA 2 FAULT must fire once vane 2 seizes");
        assert!(!holds(&aoa1.trigger, &armed), "vane 1 was untouched, so its own procedure must stay quiet");
    }

    /// GPS family: fail receiver 2's electronics outright and check the
    /// single procedure and the 1+2 combo behave -- the combo needs both.
    #[test]
    fn gps_2_and_the_1_plus_2_combo_need_the_right_receivers_down() {
        let index = FaultIndex::build();
        let id2 = index.id("34_nav.gps_receiver_2", "receiver_fault");
        assert_ne!(id2, 0, "the GPS 2 receiver fault id could not be resolved");

        let healthy = run(flying(), &Faults::default(), 10);
        let one_down = run(flying(), &Faults::from_pairs([(id2, 1.0)]), 10);

        let gps2 = wiring(340_800_035);
        let combo = wiring(340_800_036);
        assert!(!holds(&gps2.trigger, &healthy), "NAV GPS 2 FAULT must be quiet with both receivers healthy");
        assert!(holds(&gps2.trigger, &one_down), "NAV GPS 2 FAULT must fire once receiver 2 fails outright");
        assert!(!holds(&combo.trigger, &one_down), "receiver 1 is still healthy, so the 1+2 combo must not fire on receiver 2 alone");
    }

    /// Static-probe family: block one port of one system's pair -- the
    /// averaging line falls back to the healthy port, which is exactly
    /// what its own `degraded` flag exists to catch -- and check the
    /// procedure fires on that alone.
    #[test]
    fn static_probe_fault_fires_when_one_port_of_a_pair_is_blocked() {
        let index = FaultIndex::build();
        let id = index.id("34_nav.static_1_1", "blocked");
        assert_ne!(id, 0, "the static port fault id could not be resolved");

        let healthy = run(flying(), &Faults::default(), 20);
        let armed = run(flying(), &Faults::from_pairs([(id, 1.0)]), 60);

        let p = wiring(340_800_067);
        assert!(!holds(&p.trigger, &healthy), "NAV STATIC PROBE FAULT must be quiet with every port clear");
        assert!(holds(&p.trigger, &armed), "NAV STATIC PROBE FAULT must fire once one port of a pair is blocked");
    }
}
