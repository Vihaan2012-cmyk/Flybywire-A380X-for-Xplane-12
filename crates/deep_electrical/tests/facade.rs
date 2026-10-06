use std::collections::BTreeSet;

use deep_electrical::*;

#[test]
fn keys_are_the_efbs_lvar_keys() {
    assert_eq!(lvar_key("fms-1-normal-bkr"), "FMS_1_NORMAL_BKR");
    assert_eq!(lvar_key("fire-loop-eng1-A"), "FIRE_LOOP_ENG1_A");
    assert_eq!(lvar_key("BKR_fms-1-normal-bkr_OPEN"), "BKR_FMS_1_NORMAL_BKR_OPEN");
    assert_eq!(lvar_key("ELEC_X:1"), "ELEC_X_1");
}

#[test]
fn every_published_name_keeps_a_distinct_key() {
    let deep = DeepElectrical::new();
    let names = deep.published_names();
    let keys: BTreeSet<String> = names.iter().map(|n| lvar_key(n)).collect();
    assert_eq!(keys.len(), names.len(), "two published names collapse to one simulator variable");
}

#[test]
fn every_unit_keeps_a_distinct_key() {
    let deep = DeepElectrical::new();
    let ids = deep.unit_ids();
    let keys: BTreeSet<String> = ids.iter().map(|n| lvar_key(n)).collect();
    assert_eq!(keys.len(), ids.len());
    assert!(ids.len() >= 399, "{} units", ids.len());
}

#[test]
fn published_names_cover_what_a_tick_publishes() {
    let mut deep = DeepElectrical::new();
    let names: BTreeSet<String> = deep.published_names().into_iter().collect();
    let inputs = golden::inputs(250);
    for _ in 0..3 {
        deep.tick(&inputs, &Faults::default(), &mut |k, _| assert!(names.contains(k), "{k} is published but was not listed"));
    }
}

#[test]
fn every_gate_names_real_units() {
    let deep = DeepElectrical::new();
    let ids = deep.unit_ids();
    let mut vars = BTreeSet::new();
    for g in GATES {
        assert!(vars.insert(g.variable), "{} twice", g.variable);
        assert!(g.variable.ends_with("_BREAKER_OPEN"), "{}: open polarity only, so an unwritten gate is closed", g.variable);
        assert!(!g.units.is_empty());
        for u in g.units {
            assert!(ids.contains(u), "{} names {u}, which is not a unit", g.variable);
        }
    }
}

#[test]
fn a_unit_opened_by_the_crew_stays_open_until_closed_and_drops_its_load() {
    let mut deep = DeepElectrical::new();
    let i = deep.unit_index("fms-1-normal-bkr").unwrap();
    let inputs = golden::inputs(250);
    let mut sink = |_: &str, _: f64| {};
    for _ in 0..20 {
        deep.tick(&inputs, &Faults::default(), &mut sink);
    }
    assert!(deep.current_a_at(i) > 0.0, "the FMS draws current on a powered aircraft");
    assert!(deep.command_at(i, BreakerCommand::Open));
    for _ in 0..20 {
        deep.tick(&inputs, &Faults::default(), &mut sink);
    }
    assert!(deep.is_open_at(i));
    assert_eq!(deep.current_a_at(i), 0.0, "an open unit carries no current");
    assert!(deep.command_at(i, BreakerCommand::Close));
    for _ in 0..20 {
        deep.tick(&inputs, &Faults::default(), &mut sink);
    }
    assert!(!deep.is_open_at(i));
    assert!(deep.current_a_at(i) > 0.0);
}

#[test]
fn a_thermal_breaker_is_pulled_and_pushed_in_by_hand() {
    let mut deep = DeepElectrical::new();
    use deep_electrical::deep::breakers::{catalog, trip::BreakerKind};
    let thermal = catalog::all().iter().position(|d| d.kind == BreakerKind::Thermal).expect("the catalogue has thermal breakers");
    assert_eq!(deep.unit_ids()[thermal], catalog::all()[thermal].id, "units are in catalogue order");
    assert!(deep.command_at(thermal, BreakerCommand::Open));
    assert!(deep.is_open_at(thermal));
    assert!(deep.command_at(thermal, BreakerCommand::Close));
    assert!(!deep.is_open_at(thermal));
}
