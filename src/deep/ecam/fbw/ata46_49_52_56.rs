//! ATA 46-49-52-56 -- doors (E-ELEC's third chapter). FlyByWire defines six
//! unwired abnormal-sensed procedures for the upper-deck passenger doors,
//! `520800027`-`032 DOOR UPPER 1L/1R/2L/2R/3L/3R NOT CLOSED`
//! (`AbnormalSensed/ata46-49-52-56.ts`). The ten main-deck door ids in the
//! same catalogue file already carry FlyByWire's own `flightPhaseInhib`
//! (`FwsAbnormalSensed.ts`) and are correctly left alone here.
//!
//! # The source
//!
//! `E:/fbw-debug/ecam/E-ELEC-FCOM.json` (Task A) confirms all six against
//! FCOM PRO-ABN-ECAM p.5746: one combined procedure, "DOOR MAIN 1(2)(3)(4)
//! (5)L(R) / UPPER 1(2)(3)L(R) NOT CLOSED" -- "The door is not closed and is
//! not locked" -- covering every main and upper passenger door under one
//! trigger, Level::Warning, and the same decoded inhibit bar
//! (`[1, 4, 5, 6, 7, 9, 10, 12]`) for all of them.
//!
//! # The signal
//!
//! `deep::live::DOOR_NAMES`/`DOOR_POINTS` previously carried only one of the
//! six upper doors (`U1L`); this pass (`E:/fbw-debug/ecam/
//! E-ELEC-DESIGN.md`) extends both to all six, so `Truth::door_open_fraction`
//! now gives every upper door's own real interactive-point travel from
//! `src/doors.rs`'s aircraft model, not a value this port would otherwise
//! have to invent. `deep::cabin::live::CabinLive` reads that directly and
//! publishes `CABIN_DOOR_UPPER_<pos>_OPEN_PERCENT`, plus that door's own
//! not-latched-sensor fault as `CABIN_DOOR_UPPER_<pos>_LATCH_SENSOR_FAULT`
//! (the same per-door proximity-sensor fault class `deep::cabin::registry`'s
//! existing `f_latch` already registers for the one generic door, one
//! instance per upper door here instead of the shared generic one).
//!
//! "Closed" is exactly 0% open (`deep::cabin::live`'s own doc, "and every
//! door shut" is the cold-and-dark default) -- a definitional boundary, not
//! an invented physical threshold. A latch-sensor fault also raises the
//! alert on its own: a proximity sensor stuck reporting NOT CLOSED is
//! exactly the disagreement the real alert exists to catch, independent of
//! the door's real position.

use super::{phase, proc, sd_page, FbwProc};
use crate::deep::api::{any, var, Level};

pub fn wire(v: &mut Vec<FbwProc>) {
    let doors: [(u64, &str, &str); 6] = [
        (520_800_027, "DOOR UPPER 1L NOT CLOSED", "1L"),
        (520_800_028, "DOOR UPPER 1R NOT CLOSED", "1R"),
        (520_800_029, "DOOR UPPER 2L NOT CLOSED", "2L"),
        (520_800_030, "DOOR UPPER 2R NOT CLOSED", "2R"),
        (520_800_031, "DOOR UPPER 3L NOT CLOSED", "3L"),
        (520_800_032, "DOOR UPPER 3R NOT CLOSED", "3R"),
    ];
    for (id, title, pos) in doors {
        v.push(
            proc(
                id,
                title,
                Level::Warning,
                sd_page::DOOR,
                any(vec![
                    var(&format!("CABIN_DOOR_UPPER_{pos}_OPEN_PERCENT")).gt(0.0),
                    var(&format!("CABIN_DOOR_UPPER_{pos}_LATCH_SENSOR_FAULT")).on(),
                ]),
                "FCOM PRO-ABN-ECAM p.5746: the door is not closed and is not locked -- deep::cabin's own real door-position readout (deep::live::DOOR_NAMES, extended this pass) or its own latch-sensor disagreement",
            )
            // 1.0 s: avoid flashing the procedure up for a single frame
            // while the door is mid-travel, matching the confirm scale
            // `ata24.rs`'s own bus-fault family uses for the same reason.
            .confirm(1.0)
            .inhibit(phase::NONE)
            .items(6, Vec::new()),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::api::Cond;
    use crate::deep::integration::failure_audit::fresh_areas;
    use crate::deep::live::{Faults, Truth};
    use std::collections::BTreeMap;

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
        v.into_iter().find(|p| p.id == id).unwrap_or_else(|| panic!("{id} is not wired by ata46_49_52_56::wire"))
    }

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

    /// Every door starts shut cold and dark (`deep::cabin::live`'s own
    /// documented default), and opening one upper door fires only that
    /// door's own procedure.
    #[test]
    fn upper_1l_not_closed_fires_only_on_its_own_door() {
        let cold_shut = run(Truth { dt_s: 0.1, ..Truth::default() }, &Faults::default(), 5);
        let mut open_1l = Truth { dt_s: 0.1, ..Truth::default() };
        let idx = crate::deep::live::DOOR_NAMES.iter().position(|&n| n == "U1L").expect("U1L must be in DOOR_NAMES");
        open_1l.door_open_fraction[idx] = 0.5;
        let opened = run(open_1l, &Faults::default(), 5);

        let p_1l = wiring(520_800_027);
        let p_1r = wiring(520_800_028);
        assert!(!holds(&p_1l.trigger, &cold_shut), "DOOR UPPER 1L NOT CLOSED must be quiet with every door shut");
        assert!(holds(&p_1l.trigger, &opened), "DOOR UPPER 1L NOT CLOSED must fire once U1L is open");
        assert!(!holds(&p_1r.trigger, &opened), "door 1R was untouched, so its own procedure must stay quiet");
    }

    /// A latch-sensor fault on a shut door raises the alert on its own,
    /// independent of the door's real (shut) position.
    #[test]
    fn upper_2l_latch_sensor_fault_fires_even_with_the_door_really_shut() {
        let r = crate::deep::registry();
        let id = r
            .failures
            .iter()
            .find(|f| f.component == "52_dr.door_upper_2L_latch_sensor")
            .unwrap_or_else(|| panic!("no registered failure on 52_dr.door_upper_2L_latch_sensor"))
            .id;

        let healthy = run(Truth { dt_s: 0.1, ..Truth::default() }, &Faults::default(), 5);
        let stuck = run(Truth { dt_s: 0.1, ..Truth::default() }, &Faults::from_pairs([(id, 1.0)]), 5);

        let p = wiring(520_800_029);
        assert!(!holds(&p.trigger, &healthy));
        assert!(holds(&p.trigger, &stuck), "DOOR UPPER 2L NOT CLOSED must fire on its own latch-sensor fault alone");
    }
}
