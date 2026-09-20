//! ATA 31 -- indicating/recording, and ATA 33 -- lights. Both share
//! `AbnormalSensed/ata31-32-33.ts` with ATA 32, which already had its own
//! full pass (`ata32.rs`) and is left alone here.
//!
//! The file defines 35 procedures across ATA 31 (`311800001`..`319800004`)
//! and exactly **one** ATA 33 entry (`334800101` CABIN EMER EXIT LT FAULT --
//! this airframe's lighting chapter has no other abnormal-sensed procedure
//! in this file at all, so "ATA 33 is thin" is not a guess).
//!
//! # What is wired, and from where
//!
//! `deep::avionics_network` publishes `AVNCS_MODULE_CPIOM_C1_PARTITION_
//! FWS_AVAILABLE`: this port models a single CPIOM-C module (`CPIOM_C1`),
//! not the dual-module split the real FWS hosts its two channels on, so this
//! one ARINC 653 partition variable is this port's whole verdict on "is the
//! FWS function up" -- it cannot separately say FWS 1 vs FWS 2. That flag
//! going false is therefore FlyByWire's `314800004` FWS 1+2 FAULT (the total
//! loss this port can actually see), never `314800008`/`314800009` FWS 1/2
//! FAULT alone (nothing here can say which channel).
//!
//! `314800003` FWS 1+2 & FCDC 1+2 FAULT adds the two flight-control data
//! concentrators: `deep::electrical` publishes `ELEC_LOAD_fcdc-1_POWERED`
//! and `ELEC_LOAD_fcdc-2_POWERED` from its own dual-fed FCDC loads
//! (`electrical/loads.rs:353-354`, ATA 27), and losing power to an LRU is a
//! sound (if not exhaustive) subset of "faulted" -- the same reasoning
//! `ata24.rs`'s bus and TR/GEN procedures already use. `314800003`'s trigger
//! is a strict superset of `314800004`'s own condition (it requires the FWS
//! partition down *and* both FCDCs unpowered), so `314800004` names it as
//! its suppressor: when the combined procedure is up, the plainer one stays
//! down, matching FlyByWire's own `notActiveWhenItemActive` convention.
//!
//! `319800002`/`319800003` RECORDER CVR/DFDR FAULT read
//! `ELEC_LOAD_cvr_POWERED`/`ELEC_LOAD_dfdr_POWERED` (`electrical/loads.rs:
//! 713-716`, ATA 31's own mandatory-recorder loads) the same way.
//! `319800001` ACCELMTR FAULT and `319800004` SYS FAULT stay unwired: no
//! area publishes an independent accelerometer state, and "SYS FAULT" names
//! the recorder system as a whole, which is not one load this port can
//! point at without guessing which failure mode it means.
//!
//! `334800101` CABIN EMER EXIT LT FAULT reads
//! `ELEC_LOAD_emer-lighting-charger-1_POWERED` and `...-2_POWERED`
//! (`electrical/loads.rs:849-853`, ATA 33's own emergency-lighting-battery
//! charger loads -- the circuit that keeps the exit-light battery packs
//! charged). Both unpowered together is what "FAULT" names: one charger
//! down still leaves the other battery pack serviced.
//!
//! # The "network alive" gate, again
//!
//! Every trigger above is gated on [`network_alive`], repeated from
//! `ata24.rs` rather than made `pub` there for one caller: without it, a
//! cold and dark aircraft -- FWS partition unpowered, both FCDCs unpowered,
//! both recorders unpowered, both lighting chargers unpowered -- would light
//! every one of these five procedures at the gate before the first engine
//! ever turns. The gate is the FWS's own real precondition (it needs power
//! to say anything, including about itself), written down rather than
//! invented for this file.
//!
//! # What else was checked and left alone
//!
//! * ATA 31's CDS family (`311800001`..`311800012`, display units, EFIS
//!   control panels), the cursor/keyboard family (`313800001`..`313800007`),
//!   HUD (`316800001`/`316800002`) and the video multiplexer (`318800001`):
//!   no area publishes per-display-unit, per-EFIS-panel, cursor/keyboard or
//!   HUD state. `deep::avionics_network`'s ~200 `AVNCS_*` names cover the
//!   CPIOM/IOM network fabric (modules, cables, switches, virtual links),
//!   not the CDS display units themselves, which are a separate, unmodelled
//!   LRU family.
//! * The other six `314800xxx` FWS ids (`314800001` AIRLINE CUSTOMIZATION
//!   REJECTED, `314800005` ATQC DATABASE REJECTED, `314800006` AUDIO
//!   FUNCTION LOST, `314800007` ECP FAULT, `314800008`/`314800009` FWS 1/2
//!   FAULT individually): each names a specific FWS sub-function or a single
//!   channel this port's one partition flag cannot distinguish.

use super::{phase, proc, sd_page, FbwProc};
use crate::deep::api::{all, var, any, Cond, Level};

/// At least one main AC bus is live, so the electrical network is running
/// and an unpowered load is that load's own problem rather than a cold
/// aeroplane. Identical to `ata24::network_alive`, which is private to that
/// module; repeated here rather than exported for this one caller.
fn network_alive() -> Cond {
    any(vec![
        var("ELEC_AC_1_BUS_IS_POWERED").on(),
        var("ELEC_AC_2_BUS_IS_POWERED").on(),
        var("ELEC_AC_3_BUS_IS_POWERED").on(),
        var("ELEC_AC_4_BUS_IS_POWERED").on(),
    ])
}

/// `deep::avionics_network`'s whole verdict on "is the FWS function up" --
/// see this module's doc comment for why one flag stands for both channels.
fn fws_unavailable() -> Cond {
    var("AVNCS_MODULE_CPIOM_C1_PARTITION_FWS_AVAILABLE").off()
}

fn fcdc_dead(n: u32) -> Cond {
    var(&format!("ELEC_LOAD_fcdc-{n}_POWERED")).off()
}

pub fn wire(v: &mut Vec<FbwProc>) {
    // ---- FWS 1+2 FAULT.
    v.push(
        proc(
            314_800_004,
            "FWS 1+2 FAULT",
            Level::Caution,
            sd_page::STATUS,
            all(vec![fws_unavailable(), network_alive()]),
            "deep::avionics_network's single CPIOM-C1 FWS partition flag going unavailable while the network is powered -- this port models one CPIOM-C module, so this is its whole verdict on the FWS function",
        )
        .confirm(1.0)
        .suppressed_by(&[314_800_003])
        .items(9, Vec::new()),
    );

    // ---- FWS 1+2 & FCDC 1+2 FAULT. Strict superset of the above: the FWS
    // partition down as well as both FCDC loads unpowered.
    v.push(
        proc(
            314_800_003,
            "FWS 1+2 & FCDC 1+2 FAULT",
            Level::Caution,
            sd_page::STATUS,
            all(vec![fws_unavailable(), fcdc_dead(1), fcdc_dead(2), network_alive()]),
            "the FWS partition down together with both flight-control data concentrators' own ELEC_LOAD_fcdc-n_POWERED reading unpowered",
        )
        .confirm(1.0)
        .items(10, Vec::new()),
    );

    // ---- RECORDER CVR/DFDR FAULT. `deep::electrical`'s own mandatory
    // flight-recorder loads, unpowered while the network is live.
    v.push(
        proc(
            319_800_002,
            "RECORDER CVR FAULT",
            Level::Caution,
            sd_page::STATUS,
            all(vec![var("ELEC_LOAD_cvr_POWERED").off(), network_alive()]),
            "deep::electrical's own CVR load reading unpowered while the network is live",
        )
        .confirm(1.0),
    );
    v.push(
        proc(
            319_800_003,
            "RECORDER DFDR FAULT",
            Level::Caution,
            sd_page::STATUS,
            all(vec![var("ELEC_LOAD_dfdr_POWERED").off(), network_alive()]),
            "deep::electrical's own DFDR load reading unpowered while the network is live",
        )
        .confirm(1.0),
    );

    // ---- CABIN EMER EXIT LT FAULT. Both emergency-lighting-battery charger
    // circuits unpowered together while the network is live -- one charger
    // down alone still leaves that side's battery pack serviced.
    v.push(
        proc(
            334_800_101,
            "CABIN EMER EXIT LT FAULT",
            Level::Caution,
            sd_page::STATUS,
            all(vec![
                var("ELEC_LOAD_emer-lighting-charger-1_POWERED").off(),
                var("ELEC_LOAD_emer-lighting-charger-2_POWERED").off(),
                network_alive(),
            ]),
            "deep::electrical's own pair of emergency-lighting-battery charger loads both reading unpowered while the network is live",
        )
        .confirm(1.0),
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::ecam::fbw;
    use crate::deep::integration::failure_audit::bare;
    use crate::deep::live::{Faults, Truth};

    fn run(truth: Truth, faults: &Faults, frames: usize) -> std::collections::BTreeMap<String, f64> {
        let mut deep = crate::deep::integration::failure_audit::fresh_areas();
        let mut out = std::collections::BTreeMap::new();
        for _ in 0..frames {
            out.clear();
            deep.tick(truth.clone(), faults, &mut |n, v| {
                out.insert(bare(n).to_owned(), v);
            });
        }
        out
    }

    fn holds(c: &Cond, published: &std::collections::BTreeMap<String, f64>) -> bool {
        c.eval(&|n: &str| *published.get(bare(n)).unwrap_or(&0.0))
    }

    fn wiring(id: u64) -> FbwProc {
        fbw::wirings().into_iter().find(|p| p.id == id).unwrap_or_else(|| panic!("{id} is not wired"))
    }

    /// A failure id from the combined registry, found by a predicate over
    /// its component and model field rather than hard-coded, so this fails
    /// loudly if an area renames one instead of silently arming nothing.
    fn failure_id(pred: impl Fn(&crate::deep::api::FailureDef) -> bool, what: &str) -> u64 {
        let r = crate::deep::registry();
        let hits: Vec<u64> = r.failures.iter().filter(|f| pred(f)).map(|f| f.id).collect();
        assert!(!hits.is_empty(), "no registered failure matches {what}");
        hits[0]
    }

    fn flying() -> Truth {
        // `deep::avionics_network` does not read its module power from
        // `deep::electrical`'s own simulated bus network -- its own doc
        // comment (`avionics_network/live.rs:45-54`) says `Truth`'s
        // `ac_bus_volts`/`dc_bus_volts` are a still-missing load-allocation
        // seam, so CPIOM-C1 (and every other module) reads its power
        // straight off those two raw `Truth` fields. They default to 0 V,
        // so without setting them here the FWS partition reads permanently
        // unavailable even on a running aircraft, healthy or not. Set to
        // nominal so this fixture is unambiguously "the aircraft has power".
        Truth {
            dt_s: 0.05,
            on_ground: false,
            engine_running: [true; 4],
            engine_n1_frac: [0.9; 4],
            engine_n2_frac: [0.9; 4],
            engine_n3_frac: [0.9; 4],
            ac_bus_volts: [115.0; 4],
            dc_bus_volts: [28.0; 2],
            ..Truth::default()
        }
    }

    #[test]
    fn fws_and_fcdc_faults_need_the_network_to_be_alive() {
        // Cold and dark: the FWS partition and both FCDCs are unpowered
        // simply because nothing is generating, and none of that is a
        // fault. The network-alive gate is what keeps both procedures off
        // the flight deck at the gate.
        let cold = run(Truth { dt_s: 0.1, ..Truth::default() }, &Faults::default(), 30);
        assert!(!holds(&wiring(314_800_004).trigger, &cold), "FWS 1+2 FAULT fired on a cold and dark aircraft");
        assert!(!holds(&wiring(314_800_003).trigger, &cold), "FWS 1+2 & FCDC 1+2 FAULT fired on a cold and dark aircraft");
    }

    #[test]
    fn fws_1plus2_fault_fires_when_the_cpiom_c1_fws_partition_dies() {
        let index = crate::deep::avionics_network::live::FaultIndex::build();
        let id = index.id("CPIOM-C1 partition FWS failure");
        assert_ne!(id, 0, "deep::avionics_network registers a CPIOM-C1 FWS partition failure");

        let healthy = run(flying(), &Faults::default(), 20);
        let armed = run(flying(), &Faults::from_pairs([(id, 1.0)]), 20);

        let fws = wiring(314_800_004);
        assert!(!holds(&fws.trigger, &healthy), "FWS 1+2 FAULT must be quiet with the partition healthy");
        assert!(holds(&fws.trigger, &armed), "FWS 1+2 FAULT must fire when the CPIOM-C1 FWS partition dies");
        // The FCDCs are untouched, so the combined procedure must stay down
        // and the plain one must not be suppressed by it.
        assert!(!holds(&wiring(314_800_003).trigger, &armed), "FWS 1+2 & FCDC 1+2 FAULT must stay quiet when only the FWS partition died");
    }

    #[test]
    fn fws_and_fcdc_combined_fault_needs_both_fcdcs_dead_too() {
        // Each FCDC is dual-fed (`electrical/loads.rs:353-354`, two feed
        // breakers OR-ed together): tripping one of its two *breakers* --
        // `17_breakers.fcdc-1-normal-bkr`/`-2nd-bkr` -- proves nothing, since
        // the surviving feed just keeps the load powered, and even arming
        // both is not reliable through the breakers' own I^2t drift element
        // (it only opens a breaker that is already carrying most of its own
        // rated current, which is a fact about that specific breaker's
        // sizing, not a lever this test can pull). What genuinely represents
        // "this LRU has no power" regardless of which feed it would have
        // used is the *load's own* `open_circuit` fault
        // (`deep::electrical::network::Load.faults.open_circuit`, `network.rs`
        // `health()`/`step()`: at 1.0 the load's own delivered power is
        // forced to zero, so `powered = energised && p > 0.0` reads false on
        // every feed at once) -- registered once per catalogue load as
        // `deep::electrical::registry.rs`'s `LOAD_CHANNELS`, component
        // `27_elec.fcdc-1`/`27_elec.fcdc-2`.
        let index = crate::deep::avionics_network::live::FaultIndex::build();
        let fws_id = index.id("CPIOM-C1 partition FWS failure");
        assert_ne!(fws_id, 0);
        let fcdc1 = failure_id(|f| f.component == "27_elec.fcdc-1" && f.model_field.contains("open_circuit"), "FCDC 1's own load open-circuit fault");
        let fcdc2 = failure_id(|f| f.component == "27_elec.fcdc-2" && f.model_field.contains("open_circuit"), "FCDC 2's own load open-circuit fault");

        let both_dead = run(flying(), &Faults::from_pairs([(fws_id, 1.0), (fcdc1, 1.0), (fcdc2, 1.0)]), 30);
        let only_fws = run(flying(), &Faults::from_pairs([(fws_id, 1.0)]), 30);

        assert!(holds(&wiring(314_800_003).trigger, &both_dead), "FWS 1+2 & FCDC 1+2 FAULT must fire once the FWS partition and both FCDCs are down");
        assert!(!holds(&wiring(314_800_003).trigger, &only_fws), "and must stay quiet with the FCDCs still powered");
    }

    #[test]
    fn recorder_faults_fire_on_their_own_load_and_not_on_a_cold_aircraft() {
        // Same reasoning as the FCDC test above: CVR/DFDR are single-fed at
        // a comfortable current margin under their own breaker's rating
        // (`deep::breakers::catalog`'s `standard_size` rounds up), so a
        // `trip_calibration_drift` fault on `17_breakers.cvr`/`dfdr` never
        // actually opens either breaker -- confirmed by instrumenting the
        // live current, which never moved past ~1.75 A against a 3.0 A
        // rating even fully drifted for 120 s. The load's own
        // `open_circuit` fault is what genuinely and immediately de-powers
        // it regardless of breaker headroom.
        let cvr = failure_id(|f| f.component == "31_elec.cvr" && f.model_field.contains("open_circuit"), "the CVR load's own open-circuit fault");
        let dfdr = failure_id(|f| f.component == "31_elec.dfdr" && f.model_field.contains("open_circuit"), "the DFDR load's own open-circuit fault");

        let cold = run(Truth { dt_s: 0.1, ..Truth::default() }, &Faults::default(), 30);
        assert!(!holds(&wiring(319_800_002).trigger, &cold), "RECORDER CVR FAULT fired on a cold and dark aircraft");
        assert!(!holds(&wiring(319_800_003).trigger, &cold), "RECORDER DFDR FAULT fired on a cold and dark aircraft");

        let healthy = run(flying(), &Faults::default(), 30);
        let cvr_dead = run(flying(), &Faults::from_pairs([(cvr, 1.0)]), 30);
        let dfdr_dead = run(flying(), &Faults::from_pairs([(dfdr, 1.0)]), 30);

        assert!(!holds(&wiring(319_800_002).trigger, &healthy) && !holds(&wiring(319_800_003).trigger, &healthy), "both recorder procedures must be quiet with both recorders healthy");
        assert!(holds(&wiring(319_800_002).trigger, &cvr_dead), "RECORDER CVR FAULT must fire when the CVR's own load loses power");
        assert!(!holds(&wiring(319_800_003).trigger, &cvr_dead), "and DFDR must stay quiet when only the CVR died");
        assert!(holds(&wiring(319_800_003).trigger, &dfdr_dead), "RECORDER DFDR FAULT must fire when the DFDR's own load loses power");
        assert!(!holds(&wiring(319_800_002).trigger, &dfdr_dead), "and CVR must stay quiet when only the DFDR died");
    }

    #[test]
    fn cabin_emer_exit_lt_fault_needs_both_chargers_dead_and_not_a_cold_aircraft() {
        let c1 = failure_id(|f| f.component == "17_breakers.emer-lighting-charger-1" && f.model_field.contains("trip_calibration_drift"), "emergency lighting charger 1's breaker calibration drift");
        let c2 = failure_id(|f| f.component == "17_breakers.emer-lighting-charger-2" && f.model_field.contains("trip_calibration_drift"), "emergency lighting charger 2's breaker calibration drift");

        let cold = run(Truth { dt_s: 0.1, ..Truth::default() }, &Faults::default(), 30);
        assert!(!holds(&wiring(334_800_101).trigger, &cold), "CABIN EMER EXIT LT FAULT fired on a cold and dark aircraft");

        let healthy = run(flying(), &Faults::default(), 20);
        let one_dead = run(flying(), &Faults::from_pairs([(c1, 1.0)]), 2_400);
        let both_dead = run(flying(), &Faults::from_pairs([(c1, 1.0), (c2, 1.0)]), 2_400);

        assert!(!holds(&wiring(334_800_101).trigger, &healthy), "must be quiet with both chargers healthy");
        assert!(!holds(&wiring(334_800_101).trigger, &one_dead), "one charger down must still leave the other pack serviced");
        assert!(holds(&wiring(334_800_101).trigger, &both_dead), "must fire once both chargers are dead");
    }

    /// `phase` is re-exported for parity with the other `ataNN` modules even
    /// though this file's entries all keep the default inhibit window; this
    /// guards against an unused-import warning turning into a build failure
    /// if that ever changes without touching this line.
    #[test]
    fn wired_ids_keep_the_default_takeoff_and_landing_inhibit() {
        for id in [314_800_004, 314_800_003, 319_800_002, 319_800_003, 334_800_101] {
            assert_eq!(wiring(id).inhibit, phase::TAKEOFF_AND_LANDING);
        }
    }
}
