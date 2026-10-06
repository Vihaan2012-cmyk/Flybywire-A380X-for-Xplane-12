use std::collections::{BTreeMap, BTreeSet};

use systems::simulation::test::{TestBed, WriteByName};

use super::breaker_failures::BREAKER_FAILURES;
use super::power_effects::{self, When, ALIAS_EFFECTS, LOAD_EFFECTS, NO_COCKPIT_EFFECT};
use super::tests::{aircraft, run};
use super::tyres::Tyres;
use deep_systems::{all_gates, lvar_key, DeepSystems};

const CATALOGUE: &str = "D:/A380/msfs-a380/install/out/EFB/catalogue.json";
const WIRING_OUT: &str = "D:/A380/msfs-a380/install/out/EFB/wiring.json";

fn flybywire_rust_ids() -> BTreeSet<u64> {
    let src = include_str!("../failures.rs");
    let start = src.find("fn fbw_failures").expect("a380_systems lists its FlyByWire failures");
    src[start..]
        .split('(')
        .filter_map(|chunk| {
            let (id, rest) = chunk.split_once(',')?;
            rest.trim_start().starts_with("FailureType").then(|| id.trim().replace('_', "").parse().ok())?
        })
        .collect()
}

fn flybywire_own_ids() -> BTreeSet<u64> {
    let src = include_str!("../../../../../systems/failures/src/a380.ts");
    let start = src.find("A380Failure = Object.freeze({").expect("a380.ts lists its failures");
    let body = &src[start..start + src[start..].find("});").unwrap()];
    body.lines()
        .filter_map(|l| l.split(':').nth(1))
        .filter_map(|v| v.trim().trim_end_matches(',').parse().ok())
        .collect()
}

fn published() -> BTreeSet<String> {
    DeepSystems::new().published_names().into_iter().collect()
}

fn catalogue() -> serde_json::Value {
    serde_json::from_str(&std::fs::read_to_string(CATALOGUE).expect("the EFB catalogue is built")).unwrap()
}

#[test]
fn every_wiring_row_names_a_real_load_and_failures_a_model_carries_out() {
    let published = published();
    let deep: BTreeSet<u64> = deep_systems::failure_ids();
    let fbw = flybywire_rust_ids();
    assert!(fbw.len() > 100, "parsed only {} FlyByWire Rust failure ids", fbw.len());
    let alias: BTreeSet<u64> = power_effects::alias_sources().collect();
    let mut bad = Vec::new();
    for row in LOAD_EFFECTS {
        if !published.contains(&power_effects::cut_name(row.load)) {
            bad.push(format!("load row {}: the network publishes no such load", row.load));
        }
        if row.deep.is_empty() && row.fbw.is_empty() {
            bad.push(format!("load row {}: no targets", row.load));
        }
    }
    let rows = LOAD_EFFECTS
        .iter()
        .map(|r| (format!("load {}", r.load), r.deep, r.fbw, r.when))
        .chain(ALIAS_EFFECTS.iter().map(|r| (format!("alias {}", r.failure), r.deep, r.fbw, r.when)));
    for (what, deep_ids, fbw_ids, when) in rows {
        for id in deep_ids {
            if !deep.contains(id) {
                bad.push(format!("{what}: deep target {id} is not a registered failure"));
            }
            if alias.contains(id) {
                bad.push(format!("{what}: target {id} is itself only an alias"));
            }
        }
        for id in fbw_ids {
            if !fbw.contains(id) {
                bad.push(format!("{what}: FlyByWire target {id} is not one its Rust systems carry out"));
            }
        }
        if let When::EngineRunning(n) | When::EngineNotRunning(n) = when {
            if !(1..=4).contains(&n) {
                bad.push(format!("{what}: engine {n}"));
            }
        }
    }
    assert!(bad.is_empty(), "{} bad wiring rows:\n{}", bad.len(), bad.join("\n"));
}

fn cond_vars(c: &deep_systems::deep::api::Cond, out: &mut Vec<String>) {
    use deep_systems::deep::api::Cond;
    match c {
        Cond::Var { name, .. } => out.push(name.clone()),
        Cond::VarVar { a, b, .. } => out.extend([a.clone(), b.clone()]),
        Cond::And(v) | Cond::Or(v) => v.iter().for_each(|x| cond_vars(x, out)),
        Cond::Not(x) => cond_vars(x, out),
        Cond::Always => {}
    }
}

fn ecam_by_var() -> BTreeMap<String, Vec<String>> {
    let mut out: BTreeMap<String, Vec<String>> = BTreeMap::new();
    let registry = deep_systems::deep::registry();
    let triggers = registry
        .alerts
        .iter()
        .map(|a| (a.title.clone(), a.trigger.clone()))
        .chain(deep_systems::deep::ecam::fbw::wirings().into_iter().map(|p| (p.title.to_owned(), p.trigger)));
    for (title, trigger) in triggers {
        let mut vars = Vec::new();
        cond_vars(&trigger, &mut vars);
        for v in vars {
            let list = out.entry(v).or_default();
            if !list.contains(&title) {
                list.push(title.clone());
            }
        }
    }
    out
}

#[allow(clippy::type_complexity)]
fn unit_wiring() -> BTreeMap<&'static str, (Vec<&'static str>, Vec<u64>, Vec<String>, Option<&'static str>)> {
    let ecam = ecam_by_var();
    let published = published();
    let exists = |n: &str| published.contains(n);
    let mut out = BTreeMap::new();
    for unit in DeepSystems::new().unit_ids() {
        let gates: Vec<&'static str> = all_gates().filter(|g| g.units.contains(&unit)).map(|g| g.variable).collect();
        let mut failures: Vec<u64> = BREAKER_FAILURES
            .iter()
            .filter(|(units, _)| units.contains(&unit))
            .flat_map(|(_, ids)| ids.iter().copied())
            .collect();
        let load = power_effects::load_of_unit(unit, exists);
        if let Some(load) = load {
            for row in LOAD_EFFECTS.iter().filter(|r| r.load == load) {
                failures.extend(row.deep.iter().chain(row.fbw).copied());
            }
        }
        failures.sort_unstable();
        failures.dedup();
        let alerts = load.and_then(|l| ecam.get(&power_effects::cut_name(l)).cloned()).unwrap_or_default();
        let none = NO_COCKPIT_EFFECT
            .iter()
            .find(|(subject, _)| *subject == unit || Some(*subject) == load)
            .map(|(_, why)| *why);
        out.insert(unit, (gates, failures, alerts, none));
    }
    out
}

#[test]
fn every_protection_unit_reaches_the_cockpit_or_says_why_not() {
    let unwired: Vec<&str> = unit_wiring()
        .into_iter()
        .filter(|(_, (gates, failures, alerts, none))| gates.is_empty() && failures.is_empty() && alerts.is_empty() && none.is_none())
        .map(|(unit, _)| unit)
        .collect();
    assert!(unwired.is_empty(), "{} protection units reach nothing in the cockpit: {}", unwired.len(), unwired.join(", "));
}

#[test]
fn every_failure_the_efb_lists_can_be_armed() {
    let catalogue = catalogue();
    let armable: BTreeSet<u64> = deep_systems::failure_ids()
        .into_iter()
        .chain(Tyres::failure_ids())
        .chain(power_effects::alias_sources())
        .chain(flybywire_own_ids())
        .chain(super::consumer_failures::CONSUMER_FAILURES.iter().copied())
        .collect();
    let rejected: Vec<String> = catalogue["failures"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|f| !armable.contains(&f["id"].as_u64().unwrap()))
        .map(|f| format!("{} {}", f["id"], f["name"].as_str().unwrap_or("")))
        .collect();
    assert!(rejected.is_empty(), "{} failures the EFB lists would be rejected:\n{}", rejected.len(), rejected.join("\n"));
}

#[test]
fn opening_a_wired_breaker_activates_its_consumers_failures() {
    let published = published();
    let exists = |n: &str| published.contains(n);
    let units = DeepSystems::new().unit_ids();
    let Some((unit, row)) = LOAD_EFFECTS
        .iter()
        .filter(|r| r.when == When::Always && !r.deep.is_empty())
        .find_map(|r| units.iter().find(|u| power_effects::load_of_unit(u, exists) == Some(r.load)).map(|u| (*u, r)))
    else {
        return;
    };
    let mut test_bed = aircraft();
    run(&mut test_bed, 4);
    let before = test_bed.query(|a| a.deep_systems.effects.deep.clone());
    assert!(row.deep.iter().all(|id| !before.iter().any(|(d, _)| d == id)), "{unit}: healthy aircraft already shows its consumer failed");

    test_bed.write_by_name(&format!("BKR_{}_CMD", lvar_key(unit)), 1.);
    run(&mut test_bed, 8);
    let after = test_bed.query(|a| a.deep_systems.effects.deep.clone());
    for id in row.deep {
        assert!(after.iter().any(|(d, s)| d == id && *s == 1.), "opening {unit} should fail {id} (load {})", row.load);
    }

    test_bed.write_by_name(&format!("BKR_{}_CMD", lvar_key(unit)), 2.);
    run(&mut test_bed, 8);
    let closed = test_bed.query(|a| a.deep_systems.effects.deep.clone());
    assert!(row.deep.iter().all(|id| !closed.iter().any(|(d, _)| d == id)), "closing {unit} should restore its consumer");
}

#[test]
#[ignore]
fn export_wiring() {
    let units: serde_json::Map<String, serde_json::Value> = unit_wiring()
        .into_iter()
        .map(|(unit, (gates, failures, alerts, none))| {
            let mut v = serde_json::json!({ "failures": failures, "gates": gates, "ecam": alerts });
            if let Some(why) = none {
                v["none"] = serde_json::Value::from(why);
            }
            (unit.to_owned(), v)
        })
        .collect();
    let out = serde_json::json!({ "schemaVersion": 1, "units": units });
    std::fs::write(WIRING_OUT, serde_json::to_string(&out).unwrap()).unwrap();
    println!("wiring.json: {} units", units_len(&out));
}

fn units_len(v: &serde_json::Value) -> usize {
    v["units"].as_object().map_or(0, |m| m.len())
}

#[test]
#[ignore]
fn export_ecam_patches_msfs() {
    let registry = deep_systems::deep::registry();
    let published: BTreeSet<String> = published();
    let rename = |js: &str| -> String {
        let mut out = String::with_capacity(js.len());
        let mut rest = js;
        while let Some(i) = rest.find("'L:") {
            out.push_str(&rest[..i + 3]);
            rest = &rest[i + 3..];
            let end = rest.find('\'').unwrap_or(rest.len());
            let name = &rest[..end];
            if published.contains(name) {
                let key = lvar_key(name);
                out.push_str("A32NX_");
                out.push_str(key.strip_prefix("A32NX_").unwrap_or(&key));
            } else {
                out.push_str(name);
            }
            rest = &rest[end..];
        }
        out.push_str(rest);
        out
    };
    let quarantined: BTreeSet<String> = std::fs::read_to_string("D:/A380/msfs-a380/wiring/flight-alarms.json")
        .ok()
        .and_then(|s| serde_json::from_str::<BTreeMap<String, serde_json::Value>>(&s).ok())
        .map(|m| m.into_keys().collect())
        .unwrap_or_default();
    let fcom: BTreeMap<String, serde_json::Value> = std::fs::read_to_string("D:/A380/msfs-a380/wiring/registry-fcom.json")
        .ok()
        .and_then(|s| serde_json::from_str(&s).ok())
        .unwrap_or_default();
    let alerts: Vec<_> = registry
        .alerts
        .iter()
        .filter(|a| !quarantined.contains(&format!("{} {}", a.key, a.title)))
        .cloned()
        .map(|mut a| {
            if let Some(f) = fcom.get(&a.title) {
                a.fcom_phases = f["phases"].as_array().map(|v| v.iter().filter_map(|p| p.as_u64().map(|p| p as u32)).collect());
                a.suppressed_by = f["suppressed_by"].as_array().map(|v| v.iter().filter_map(|p| p.as_u64()).collect()).unwrap_or_default();
                match f["level"].as_u64() {
                    Some(3) => a.level = deep_systems::deep::api::Level::Warning,
                    Some(2) => a.level = deep_systems::deep::api::Level::Caution,
                    Some(1) => a.level = deep_systems::deep::api::Level::Advisory,
                    _ => {}
                }
            }
            a
        })
        .collect();
    let fbw: Vec<_> = deep_systems::deep::ecam::fbw::wirings()
        .into_iter()
        .filter(|p| !quarantined.contains(&format!("{} {}", p.id, p.title)))
        .collect();
    println!("quarantined {} of {} triggers", registry.alerts.len() + deep_systems::deep::ecam::fbw::wirings().len() - alerts.len() - fbw.len(), registry.alerts.len() + deep_systems::deep::ecam::fbw::wirings().len());
    let patches: Vec<serde_json::Value> = deep_systems::deep::ecam::patches::source_patches_with(&alerts, &fbw)
        .into_iter()
        .map(|p| serde_json::json!({ "path": p.path, "find": p.find, "replace": rename(&p.replace), "reason": p.reason }))
        .collect();
    let fbw = deep_systems::deep::ecam::fbw::wirings().len();
    std::fs::write("D:/A380/msfs-a380/wiring/ecam-patches-live.json", serde_json::to_string_pretty(&patches).unwrap()).unwrap();
    println!("ecam patches: {} alerts, {fbw} FlyByWire procedures", registry.alerts.len());
}

#[test]
#[ignore]
fn diag_fails_to_trip_hotair_1() {
    use deep_systems::deep::integration::failure_audit::profiles;
    let registry = deep_systems::deep::registry();
    let id = |comp: &str, part: &str| registry.failures.iter().find(|f| f.component == comp && f.name.contains(part)).map(|f| f.id).unwrap();
    let short = id("21_elec.hotair-1", "short to ground");
    let jam = id("24_elec.bkr.hotair-1", "fails to trip");
    let truth = (profiles().into_iter().find(|p| p.name == "cruise").unwrap().truth)();
    for (label, faults) in [("healthy breaker", vec![(short, 1.0)]), ("jammed breaker", vec![(short, 1.0), (jam, 1.0)])] {
        let mut deep = DeepSystems::new();
        let faults = deep_systems::Faults::from_pairs(faults);
        for frame in 0..12 {
            let mut seen: BTreeMap<String, f64> = BTreeMap::new();
            deep.tick(truth.clone(), &faults, &mut |n, v| {
                if (n.contains("hotair-1") && !n.contains("pos-ind")) || n.starts_with("ELEC_AC_ESS") {
                    seen.insert(n.to_owned(), v);
                }
            });
            println!("{label} frame {frame}: {seen:?}");
        }
    }
}

#[test]
#[ignore]
fn no_wired_trigger_fires_on_a_healthy_aircraft() {
    use deep_systems::deep::integration::failure_audit::profiles;
    let registry = deep_systems::deep::registry();
    let mut triggers: Vec<(String, deep_systems::deep::api::Cond)> =
        deep_systems::deep::ecam::fbw::generated::procs().into_iter().map(|p| (p.id.to_string(), p.trigger)).collect();
    triggers.extend(registry.alerts.iter().filter(|a| a.key.starts_with("WIRED_")).map(|a| (a.key.clone(), a.trigger.clone())));
    let mut fired: BTreeMap<String, Vec<String>> = BTreeMap::new();
    let healthy = deep_systems::Faults::from_pairs([]);
    for p in profiles() {
        let truth = (p.truth)();
        let mut deep = DeepSystems::new();
        let mut values: std::collections::HashMap<String, f64> = std::collections::HashMap::new();
        for frame in 0..p.frames {
            deep.tick(truth.clone(), &healthy, &mut |n, v| {
                values.insert(n.to_owned(), v);
            });
            if frame < 5 {
                continue;
            }
            let read = |n: &str| values.get(n).copied().unwrap_or(0.0);
            for (key, trigger) in &triggers {
                if trigger.eval(&read) {
                    let at = format!("{} frame {frame}", p.name);
                    let list = fired.entry(key.clone()).or_default();
                    if list.len() < 3 {
                        list.push(at);
                    }
                }
            }
        }
    }
    std::fs::write("D:/A380/msfs-a380/wiring/false-alarms.json", serde_json::to_string_pretty(&fired).unwrap()).unwrap();
    assert!(fired.is_empty(), "{} generated triggers fire on a healthy aircraft: {:?}", fired.len(), fired.keys().collect::<Vec<_>>());
}

#[test]
#[ignore]
fn report_every_trigger_on_a_healthy_aircraft() {
    use deep_systems::deep::integration::failure_audit::profiles;
    let registry = deep_systems::deep::registry();
    let mut triggers: Vec<(String, deep_systems::deep::api::Cond)> = deep_systems::deep::ecam::fbw::wirings()
        .into_iter()
        .map(|p| (format!("{} {}", p.id, p.title), p.trigger))
        .collect();
    triggers.extend(registry.alerts.iter().map(|a| (format!("{} {}", a.key, a.title), a.trigger.clone())));
    let mut fired: BTreeMap<String, Vec<String>> = BTreeMap::new();
    let healthy = deep_systems::Faults::from_pairs([]);
    for p in profiles() {
        let truth = (p.truth)();
        let mut deep = DeepSystems::new();
        let mut values: std::collections::HashMap<String, f64> = std::collections::HashMap::new();
        for frame in 0..p.frames {
            deep.tick(truth.clone(), &healthy, &mut |n, v| {
                values.insert(n.to_owned(), v);
            });
            if frame < 5 {
                continue;
            }
            let read = |n: &str| values.get(n).copied().unwrap_or(0.0);
            for (key, trigger) in &triggers {
                if trigger.eval(&read) {
                    let list = fired.entry(key.clone()).or_default();
                    if list.len() < 4 && !list.iter().any(|s| s.starts_with(p.name)) {
                        list.push(format!("{} frame {frame}", p.name));
                    }
                }
            }
        }
    }
    std::fs::write("D:/A380/msfs-a380/wiring/healthy-alarms-all.json", serde_json::to_string_pretty(&fired).unwrap()).unwrap();
    println!("healthy aircraft: {} of {} triggers fire somewhere", fired.len(), triggers.len());
}

#[test]
fn the_deep_registry_builds_without_errors_or_duplicate_ids() {
    let registry = deep_systems::deep::registry();
    assert!(registry.errors.is_empty(), "registry errors:
{}", registry.errors.join("
"));
}
