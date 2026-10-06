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
