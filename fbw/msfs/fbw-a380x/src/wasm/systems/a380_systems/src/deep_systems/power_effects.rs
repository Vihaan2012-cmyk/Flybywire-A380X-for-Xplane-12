use std::collections::BTreeMap;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum When {
    Always,
    OnGround,
    InFlight,
    EngineRunning(usize),
    EngineNotRunning(usize),
}

pub(super) struct LoadEffect {
    pub load: &'static str,
    pub deep: &'static [u64],
    pub fbw: &'static [u64],
    pub when: When,
}

pub(super) struct AliasEffect {
    pub failure: u64,
    pub deep: &'static [u64],
    pub fbw: &'static [u64],
    pub when: When,
}

include!("power_effects_table.rs");

#[derive(Clone, Copy, Debug, Default)]
pub(super) struct Conditions {
    pub on_ground: bool,
    pub engine_running: [bool; 4],
}

impl Conditions {
    pub fn of(truth: &deep_systems::Truth) -> Self {
        Self { on_ground: truth.on_ground, engine_running: truth.engine_running }
    }

    fn hold(&self, when: When) -> bool {
        let engine = |n: usize| n >= 1 && n <= 4 && self.engine_running[n - 1];
        match when {
            When::Always => true,
            When::OnGround => self.on_ground,
            When::InFlight => !self.on_ground,
            When::EngineRunning(n) => engine(n),
            When::EngineNotRunning(n) => n >= 1 && n <= 4 && !engine(n),
        }
    }
}

#[derive(Default, Debug, PartialEq)]
pub(super) struct Active {
    pub deep: Vec<(u64, f64)>,
    pub fbw: Vec<u64>,
}

pub(super) fn active(conditions: &Conditions, load_cut: &[bool], armed: &BTreeMap<u64, f64>) -> Active {
    let mut deep: BTreeMap<u64, f64> = BTreeMap::new();
    let mut fbw: Vec<u64> = Vec::new();
    let mut add = |ids: &[u64], fbw_ids: &[u64], severity: f64| {
        for &id in ids {
            let s = deep.entry(id).or_insert(0.);
            *s = s.max(severity);
        }
        fbw.extend_from_slice(fbw_ids);
    };
    for (row, &cut) in LOAD_EFFECTS.iter().zip(load_cut) {
        if cut && conditions.hold(row.when) {
            add(row.deep, row.fbw, 1.);
        }
    }
    for row in ALIAS_EFFECTS {
        if let Some(&severity) = armed.get(&row.failure) {
            if severity > 0. && conditions.hold(row.when) {
                add(row.deep, row.fbw, severity);
            }
        }
    }
    fbw.sort_unstable();
    fbw.dedup();
    Active { deep: deep.into_iter().collect(), fbw }
}

pub(super) fn alias_sources() -> impl Iterator<Item = u64> {
    ALIAS_EFFECTS.iter().map(|row| row.failure)
}

pub(super) fn cut_name(load: &str) -> String {
    format!("ELEC_LOAD_{load}_CUT")
}

pub(super) fn load_of_unit<'a>(unit: &'a str, exists: impl Fn(&str) -> bool) -> Option<&'a str> {
    let mut id = unit;
    loop {
        if exists(&cut_name(id)) {
            return Some(id);
        }
        id = &id[..id.rfind('-')?];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn conditions_hold_only_when_the_aircraft_is_in_that_state() {
        let mut c = Conditions { on_ground: true, engine_running: [true, false, false, false] };
        assert!(c.hold(When::Always));
        assert!(c.hold(When::OnGround) && !c.hold(When::InFlight));
        assert!(c.hold(When::EngineRunning(1)) && !c.hold(When::EngineNotRunning(1)));
        assert!(c.hold(When::EngineNotRunning(2)) && !c.hold(When::EngineRunning(2)));
        assert!(!c.hold(When::EngineRunning(5)) && !c.hold(When::EngineNotRunning(0)));
        c.on_ground = false;
        assert!(c.hold(When::InFlight));
    }

    #[test]
    fn a_feed_suffix_comes_off_until_a_published_load_is_found() {
        let loads = ["ELEC_LOAD_adirs-1_CUT", "ELEC_LOAD_cab-fan-1_CUT"];
        let exists = |n: &str| loads.contains(&n);
        assert_eq!(load_of_unit("adirs-1-normal-bkr", exists), Some("adirs-1"));
        assert_eq!(load_of_unit("cab-fan-1", exists), Some("cab-fan-1"));
        assert_eq!(load_of_unit("bat-1", exists), None);
    }

    #[test]
    fn a_cut_load_activates_its_row_and_a_healthy_one_does_not() {
        let c = Conditions { on_ground: true, engine_running: [false; 4] };
        let healthy = active(&c, &vec![false; LOAD_EFFECTS.len()], &BTreeMap::new());
        assert_eq!(healthy, Active::default());
        if let Some((k, row)) = LOAD_EFFECTS.iter().enumerate().find(|(_, r)| r.when == When::Always) {
            let mut cut = vec![false; LOAD_EFFECTS.len()];
            cut[k] = true;
            let on = active(&c, &cut, &BTreeMap::new());
            for id in row.deep {
                assert!(on.deep.iter().any(|&(d, s)| d == *id && s == 1.), "{} cut should activate {id}", row.load);
            }
            for id in row.fbw {
                assert!(on.fbw.contains(id), "{} cut should activate FBW {id}", row.load);
            }
        }
    }

    #[test]
    fn cutting_a_real_fuel_pump_breaker_arms_that_pumps_own_deep_failure_not_a_generic_placeholder() {
        let c = Conditions { on_ground: true, engine_running: [false; 4] };
        let (k, row) = LOAD_EFFECTS.iter().enumerate().find(|(_, r)| r.load == "fuel-pump-trim-left").expect("fuel-pump-trim-left must be a real wired load, not the old generic fuel-pump-N placeholder");
        assert_eq!(row.deep, &[19028045], "fuel-pump-trim-left must arm the real TrimTankPumpLeft degradation failure, not an unmapped id");
        let mut cut = vec![false; LOAD_EFFECTS.len()];
        cut[k] = true;
        let on = active(&c, &cut, &BTreeMap::new());
        assert!(on.deep.iter().any(|&(d, s)| d == 19028045 && s == 1.), "pulling the trim-left pump breaker must stop that pump, not just move electrical bookkeeping");

        let (k2, row2) = LOAD_EFFECTS.iter().enumerate().find(|(_, r)| r.load == "fuel-valve-crossfeed-1").expect("fuel-valve-crossfeed-1 must be a real wired load, not the old generic fuel-valve-N placeholder");
        assert_eq!(row2.deep, &[19028057]);
        let mut cut2 = vec![false; LOAD_EFFECTS.len()];
        cut2[k2] = true;
        let on2 = active(&c, &cut2, &BTreeMap::new());
        assert!(on2.deep.iter().any(|&(d, s)| d == 19028057 && s == 1.), "pulling the crossfeed 1 valve breaker must stop that valve");
    }

    #[test]
    fn an_armed_alias_carries_its_severity_to_its_targets() {
        let c = Conditions { on_ground: false, engine_running: [true; 4] };
        if let Some(row) = ALIAS_EFFECTS.iter().find(|r| r.when == When::Always && !r.deep.is_empty()) {
            let armed = BTreeMap::from([(row.failure, 0.4)]);
            let on = active(&c, &vec![false; LOAD_EFFECTS.len()], &armed);
            for id in row.deep {
                assert!(on.deep.iter().any(|&(d, s)| d == *id && (s - 0.4).abs() < 1e-9));
            }
        }
    }
}
