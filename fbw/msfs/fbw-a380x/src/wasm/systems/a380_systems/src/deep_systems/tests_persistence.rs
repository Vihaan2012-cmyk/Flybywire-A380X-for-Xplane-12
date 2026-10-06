use std::collections::BTreeMap;
use std::sync::atomic::{AtomicU64, Ordering};

use deep_systems::random_failures::{Config as RandomConfig, RandomFailures};
use deep_systems::scripted_failures::{ArmCondition, Scripted};
use deep_systems::wear::{Wear, WearStore};

use super::persistence::Persistence;

fn scratch_dir(tag: &str) -> std::path::PathBuf {
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    let n = COUNTER.fetch_add(1, Ordering::Relaxed);
    let dir = std::env::temp_dir().join(format!("a380x_deep_persistence_test_{tag}_{}_{n}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    dir
}

#[test]
fn save_then_load_into_a_fresh_aircraft_gives_the_same_health() {
    let dir = scratch_dir("roundtrip");
    let mut p = Persistence::new(&dir);
    p.state.airframe_hours = 123.25;

    let mut armed = BTreeMap::new();
    armed.insert(1_000_042u64, 0.6);
    let mut random = RandomFailures::new(0xDEAD_BEEF_1234_5678);
    random.restore(RandomConfig { enabled: true, rate_multiplier: 2.5 }, 0xDEAD_BEEF_1234_5678);
    let mut scripted = Scripted::new();
    scripted.schedule(1_000_099, ArmCondition::AboveAltitudeFt(10_000.0));
    let mut wear = WearStore::default();
    wear.accumulate("engine-2", 3.5, 4, 0.75, 0.02);

    p.capture_from(&armed, &random, &scripted, &wear, &std::collections::BTreeSet::new());
    p.save().expect("save against a writable scratch dir should succeed");

    let fresh = Persistence::new(&dir);
    let mut fresh_random = RandomFailures::new(0);
    let mut fresh_scripted = Scripted::new();
    let mut fresh_wear = WearStore::default();
    let loaded_armed = fresh.apply_to(&mut fresh_random, &mut fresh_scripted, &mut fresh_wear);

    assert_eq!(loaded_armed, armed, "armed durable failures must round-trip exactly");
    assert_eq!(fresh.state.airframe_hours, 123.25);
    assert_eq!(fresh_random.snapshot(), random.snapshot(), "random engine config + RNG seed must round-trip");
    assert_eq!(fresh_wear.get("engine-2"), wear.get("engine-2"), "wear must round-trip");
    assert_eq!(fresh_scripted.list().len(), 1);
    assert_eq!(fresh_scripted.list()[0].id, 1_000_099);
    assert_eq!(fresh_scripted.list()[0].condition, ArmCondition::AboveAltitudeFt(10_000.0));

    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn a_corrupt_file_gives_a_healthy_aircraft() {
    let dir = scratch_dir("corrupt");
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join(super::persistence::FILE_NAME), "{ this is not valid toml at all @@@ not even close").unwrap();

    let p = Persistence::new(&dir);
    let mut random = RandomFailures::new(0);
    let mut scripted = Scripted::new();
    let mut wear = WearStore::default();
    let armed = p.apply_to(&mut random, &mut scripted, &mut wear);

    assert!(armed.is_empty(), "a corrupt file must never seed a failure as armed");
    assert_eq!(p.state.airframe_hours, 0.0);
    assert!(!random.snapshot().0.enabled, "random failures must default to off");
    assert!(scripted.list().is_empty());
    assert_eq!(wear.get("engine-2"), Wear::default(), "a corrupt file must never seed damage");

    let missing_dir = scratch_dir("missing");
    let missing = Persistence::new(&missing_dir);
    assert!(missing.state.armed_failures.is_empty());
    assert_eq!(missing.state.airframe_hours, 0.0);

    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn a_transient_trip_is_not_saved() {
    let dir = scratch_dir("transient");
    let mut p = Persistence::new(&dir);
    let random = RandomFailures::new(0);
    let scripted = Scripted::new();
    let wear = WearStore::default();

    p.capture_from(&BTreeMap::new(), &random, &scripted, &wear, &std::collections::BTreeSet::new());
    p.save().unwrap();

    let after_transient = Persistence::new(&dir);
    assert!(after_transient.state.armed_failures.is_empty(), "a trip that was never durably armed must not survive a reload");

    let mut durable = BTreeMap::new();
    durable.insert(1_000_007u64, 1.0);
    p.capture_from(&durable, &random, &scripted, &wear, &std::collections::BTreeSet::new());
    p.save().unwrap();
    let after_durable = Persistence::new(&dir);
    assert_eq!(after_durable.state.armed_failures, durable, "a durably armed failure must survive a reload");

    let _ = std::fs::remove_dir_all(&dir);
}
