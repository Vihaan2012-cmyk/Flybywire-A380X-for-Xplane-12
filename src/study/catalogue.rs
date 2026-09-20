//! Build-time export of the *static* half of the failure catalogue: every
//! failure, component, ECAM alert and circuit breaker (with its panel
//! position) the `deep` areas and the legacy/extra catalogues register,
//! every ATA chapter name any of them uses, and the MEL-to-failure
//! cross-reference table (`mel_catalog::MEL_FAILURES`) -- as one JSON
//! document with no live state in it anywhere.
//!
//! This is the other half of [`super::web`]'s `/study/components` and
//! `/study/breakers`: those additionally publish *live* values (a measured
//! wear parameter, a breaker's present current, which failures are
//! currently armed) read from the running simulation's snapshot, so they
//! cannot run without X-Plane and a `Truth`. Everything in this file is
//! definitional -- registered once in Rust, unaffected by whether a flight
//! is even loaded -- which is exactly what an MSFS EFB bundle wants to ship
//! *inside itself* rather than cross the LVar boundary for on every frame:
//! thousands of values that never change, instead of the hundreds that do
//! (`docs/msfs-port.md` section 0a).
//!
//! Deliberately **not** a rework of `web.rs`'s own JSON shapes (those are
//! frozen by their own tests and, potentially, existing HTTP clients) --
//! this reuses the same underlying accessor functions those endpoints read
//! from (`crate::deep::registry()`, `crate::failures::all_ids()`,
//! `super::failures::{ata_of, chapter}`, `crate::deep::breakers::catalog`,
//! `crate::mel_catalog::MEL_FAILURES`), so the static catalogue and the
//! live Study endpoints can never describe two different universes of
//! failures or components, but it is an additive document, not a patch to
//! either endpoint.
//!
//! `catalogue_json` takes its generation timestamp as a parameter instead
//! of reading the clock itself, so it stays a pure function of the
//! registries: two calls with the same timestamp must be byte-identical
//! (asserted by this module's own test), which is the entire point of
//! shipping this as a version-controlled build artifact rather than
//! fetching it at runtime. `tests::dump_catalogue` is the real build step
//! (mirrors `web.rs`'s own `dump_pages`): it reads the real clock once and
//! writes to `CATALOGUE_OUT`; see `docs/catalogue-export.md`.

use std::collections::BTreeSet;

use serde_json::{json, Value};

use super::failures as study_failures;

/// This schema's version. Bump it -- and say why in
/// `docs/catalogue-export.md` -- whenever a field's *meaning* changes, not
/// merely whenever a count changes: the counts are expected to grow as the
/// `deep` areas do (two other workstreams are actively adding to
/// `src/deep/electrical` as this module is written).
pub(crate) const SCHEMA_VERSION: u32 = 1;

/// The whole `deep` registry, built once for this process. Both
/// [`components_json`] and [`alerts_json`] read from it, so it is cached
/// here rather than called once per caller the way `web.rs`'s
/// `deep_components` caches only the `components` half.
fn registry() -> &'static crate::deep::api::Registry {
    static REGISTRY: std::sync::OnceLock<crate::deep::api::Registry> = std::sync::OnceLock::new();
    REGISTRY.get_or_init(crate::deep::registry)
}

// ---------------------------------------------------------------------
// Failures: every id `crate::failures::all_ids()` knows, from any of the
// four catalogues it merges (FlyByWire's own, the flight control
// computers', this project's "extra" catalogue, and the `deep` areas').

fn failure_json(id: u64) -> Value {
    let ata = study_failures::ata_of(id);
    let component = crate::failures::affected_components(id).into_iter().next().expect("affected_components always names at least one component");
    json!({
        "id": id,
        "name": crate::failures::any_failure_name(id),
        "ataChapterNumber": ata,
        "ataChapterName": study_failures::chapter(ata),
        "component": component,
        "cause": crate::failures::cause_description(id),
        // Only the `deep` catalogue states what its 0..1 magnitude means
        // physically ("leak orifice area, 0..20 mm2", "actuator seizure
        // fraction"); the legacy and extra catalogues are a plain 0..1
        // "how failed" fraction with no further physical mapping to add,
        // so this is left out for them rather than filled with an invented
        // "fraction, 0..1" placeholder that would look like real data.
        "magnitudeSemantics": crate::failures::deep_failure(id).map(|f| f.magnitude.clone()),
    })
}

fn failures_json() -> Vec<Value> {
    let mut ids = crate::failures::all_ids();
    ids.sort_unstable();
    ids.dedup();
    ids.into_iter().map(failure_json).collect()
}

// ---------------------------------------------------------------------
// Components: only the `deep` areas' own registry. `crate::components`'s
// registry (the one `web.rs::components_json`'s first half reads) is
// populated by physics models registering themselves as they are
// constructed -- there is no such registration without building a `Truth`,
// so it has nothing to offer a sim-free build step (see
// `docs/catalogue-export.md`'s notes on what this deliberately leaves out).

/// A component id's trailing instance number, when its final `_`/`-`/`.`
/// separated token is a plain integer (`"49_apu.generator_1"` -> `1`,
/// `"24_elec.vfg-1"` -> `1`). `ComponentDef` carries no explicit instance
/// field, only whatever an area's author encoded into the id string, and
/// that encoding is not consistent across areas -- a colour word
/// (`"29_hyd.green_accumulator"`), a letter suffix
/// (`"29_hyd.green_edp_1a"`), or a plain trailing number all appear. Rather
/// than guess at the inconsistent cases, this recognises only the
/// unambiguous one and leaves the rest `None` (`docs/catalogue-export.md`
/// flags this as worth standardising).
fn trailing_instance(id: &str) -> Option<u32> {
    let bytes = id.as_bytes();
    let mut i = id.len();
    while i > 0 && bytes[i - 1].is_ascii_digit() {
        i -= 1;
    }
    if i == 0 || i == id.len() {
        return None;
    }
    if matches!(bytes[i - 1], b'_' | b'-' | b'.') {
        id[i..].parse().ok()
    } else {
        None
    }
}

fn component_json(c: &crate::deep::api::ComponentDef) -> Value {
    let ata = u64::from(c.ata);
    let mut failures = c.failures.clone();
    failures.sort_unstable();
    json!({
        "id": c.id,
        "name": c.name,
        "ataChapterNumber": ata,
        "ataChapterName": study_failures::chapter(ata),
        "instance": trailing_instance(&c.id),
        "parameters": c.params.iter().map(|p| json!({
            "name": p.name,
            "meaning": p.meaning,
            "healthyValue": p.healthy,
        })).collect::<Vec<_>>(),
        "failures": failures,
    })
}

fn components_json() -> Vec<Value> {
    let mut components: Vec<&crate::deep::api::ComponentDef> = registry().components.iter().collect();
    components.sort_by(|a, b| a.id.cmp(&b.id));
    components.into_iter().map(component_json).collect()
}

// ---------------------------------------------------------------------
// ECAM alerts.

fn level_str(l: crate::deep::api::Level) -> &'static str {
    use crate::deep::api::Level;
    match l {
        Level::Warning => "warning",
        Level::Caution => "caution",
        Level::Advisory => "advisory",
        Level::Memo => "memo",
    }
}

fn aural_str(a: &crate::deep::api::Aural) -> String {
    use crate::deep::api::Aural;
    match a {
        Aural::ContinuousRepetitiveChime => "continuousRepetitiveChime".to_string(),
        Aural::SingleChime => "singleChime".to_string(),
        Aural::Cavalry => "cavalryCharge".to_string(),
        Aural::Named(name) => format!("named:{name}"),
        Aural::None => "none".to_string(),
    }
}

fn master_light_str(m: crate::deep::api::MasterLight) -> &'static str {
    use crate::deep::api::MasterLight;
    match m {
        MasterLight::Warning => "warning",
        MasterLight::Caution => "caution",
        MasterLight::None => "none",
    }
}

fn phase_str(p: crate::deep::api::Phase) -> &'static str {
    use crate::deep::api::Phase;
    match p {
        Phase::ElecPower => "electricalPowerAvailable",
        Phase::FirstEngineStarted => "firstEngineStarted",
        Phase::FirstEngineTakeoffPower => "firstEngineTakeoffPower",
        Phase::Above80Kt => "above80Knots",
        Phase::LiftOff => "liftOff",
        Phase::Above1500Ft => "above1500Feet",
        Phase::Below800Ft => "below800Feet",
        Phase::Touchdown => "touchdown",
        Phase::Below80Kt => "below80Knots",
        Phase::SecondEngineShutdown => "secondEngineShutdown",
    }
}

fn alert_json(a: &crate::deep::api::EcamAlert) -> Value {
    let ata = u64::from(a.ata);
    let mut failures = a.failures.clone();
    failures.sort_unstable();
    json!({
        "key": a.key,
        "ataChapterNumber": ata,
        "ataChapterName": study_failures::chapter(ata),
        "title": a.title,
        "level": level_str(a.level),
        "aural": aural_str(&a.aural),
        "masterLight": master_light_str(a.master),
        "confirmSeconds": a.confirm_s,
        "inhibitedInPhases": a.inhibited_in.iter().copied().map(phase_str).collect::<Vec<_>>(),
        "statusPageLines": a.status,
        "inoperativeSystemsListEntries": a.inop,
        "failures": failures,
        // The procedure's own step-by-step text is not exported: each
        // line's "applies while"/"done when" condition
        // (`deep::api::Cond`) is written over *live* variables (an ADIRU
        // dataref, a switch position), so the procedure only means
        // something once evaluated against a running simulation -- unlike
        // every other field here, it is not purely static. Left as a count
        // rather than fabricated as flat, condition-free text.
        "procedureLineCount": a.procedure.len(),
    })
}

fn alerts_json() -> Vec<Value> {
    let mut alerts: Vec<&crate::deep::api::EcamAlert> = registry().alerts.iter().collect();
    alerts.sort_by(|a, b| a.key.cmp(&b.key));
    alerts.into_iter().map(alert_json).collect()
}

// ---------------------------------------------------------------------
// Circuit breakers: only `deep::breakers::catalog`'s 399-entry ELMS set,
// which is the one that carries a real panel/row/column position -- the
// legacy `crate::breakers::catalog()` set (absorbed `systems.cfg` circuits
// and FlyByWire power-path gates, `web.rs`'s `"source":"catalogue"` rows)
// has no panel layout at all and is really about *how* a breaker gates a
// live power path, which is a live-Truth question, not a catalogue one; it
// is deliberately left out of this static export (documented in
// `docs/catalogue-export.md`, not silently dropped).

fn panel_str(p: crate::deep::breakers::catalog::Panel) -> &'static str {
    use crate::deep::breakers::catalog::Panel;
    match p {
        Panel::OverheadFwd => "overheadForward",
        Panel::OverheadAft => "overheadAft",
        Panel::AvionicsBay => "avionicsBay",
        Panel::PrimaryPowerCentre1 => "primaryPowerCentre1",
        Panel::PrimaryPowerCentre2 => "primaryPowerCentre2",
        Panel::PrimaryPowerCentre3 => "primaryPowerCentre3",
        Panel::PrimaryPowerCentre4 => "primaryPowerCentre4",
        Panel::SecondaryPowerCentreFwd => "secondaryPowerCentreForward",
        Panel::SecondaryPowerCentreAft => "secondaryPowerCentreAft",
    }
}

fn breaker_kind_str(k: crate::deep::breakers::trip::BreakerKind) -> &'static str {
    use crate::deep::breakers::trip::BreakerKind;
    match k {
        BreakerKind::Thermal => "thermal",
        BreakerKind::Sspc => "solidStatePowerController",
    }
}

fn breaker_json(def: &crate::deep::breakers::catalog::BreakerDef) -> Value {
    let ata = u64::from(def.ata);
    json!({
        "id": def.id,
        "name": def.name,
        "ataChapterNumber": ata,
        "ataChapterName": study_failures::chapter(ata),
        "bus": def.bus.label(),
        "ratingAmperes": def.rating_a,
        "kind": breaker_kind_str(def.kind),
        "consumer": def.consumer,
        "basis": def.basis,
        "panel": panel_str(def.panel),
        "row": def.position.row,
        "column": def.position.column,
        "label": def.position.label,
        "protectsModelledLoad": def.protected_load.is_some(),
    })
}

fn breakers_json() -> Vec<Value> {
    let mut breakers: Vec<&crate::deep::breakers::catalog::BreakerDef> = crate::deep::breakers::catalog::all().iter().collect();
    breakers.sort_by(|a, b| a.id.cmp(b.id));
    breakers.into_iter().map(breaker_json).collect()
}

// ---------------------------------------------------------------------
// MEL cross-reference: not the operator's own MEL text (`mel_catalog`'s
// `catalog()`), which is the user's own copyrighted PDF, parsed onto their
// own disk and never bundled -- there is nothing there to export at build
// time, and none exists at all until the user supplies one. What *is*
// static and ours to ship is `MEL_FAILURES`: which MEL item reference
// covers which of our own failure ids, one entry per unit of redundant
// equipment.

fn mel_ata_chapter(mel_reference: &str) -> Option<u64> {
    mel_reference.get(..2)?.parse().ok()
}

fn mel_entry_json(mel_reference: &str, ids: &[u64]) -> Value {
    let mut failures = ids.to_vec();
    failures.sort_unstable();
    let ata = mel_ata_chapter(mel_reference);
    json!({
        "melReference": mel_reference,
        "ataChapterNumber": ata,
        "ataChapterName": ata.map(study_failures::chapter),
        "failures": failures,
    })
}

fn mel_entries_json() -> Vec<Value> {
    let mut entries: Vec<(&str, &[u64])> = crate::mel_catalog::MEL_FAILURES.to_vec();
    entries.sort_by_key(|(r, _)| *r);
    entries.into_iter().map(|(r, ids)| mel_entry_json(r, ids)).collect()
}

// ---------------------------------------------------------------------
// ATA chapters: not a hand-kept list of its own (which could silently drift
// from what the four sections above actually reference) -- the set of
// chapter numbers really used by this build's failures, components,
// alerts and breakers, read back from the JSON already built for them, each
// paired with its name via the one chapter-name table
// (`super::failures::chapter`) every section above already draws its own
// `ataChapterName` from.

fn chapters_json(sections: &[&[Value]]) -> Vec<Value> {
    let mut numbers: BTreeSet<u64> = BTreeSet::new();
    for section in sections {
        for entry in *section {
            if let Some(n) = entry.get("ataChapterNumber").and_then(Value::as_u64) {
                numbers.insert(n);
            }
        }
    }
    numbers.into_iter().map(|n| json!({ "number": n, "name": study_failures::chapter(n) })).collect()
}

// ---------------------------------------------------------------------

/// The whole static catalogue as one JSON document. Pure given
/// `generated_at_unix_seconds`: calling this twice with the same argument
/// produces byte-identical output (`tests::catalogue_json_is_deterministic`).
/// `serde_json`'s `Map` here is a `BTreeMap` (this crate does not enable
/// `serde_json`'s `preserve_order` feature), so every object's keys are
/// already emitted in a fixed, sorted order; every array below is sorted by
/// its own stable key before being handed to `json!`, so nothing here
/// depends on registration order either.
pub(crate) fn catalogue_json(generated_at_unix_seconds: u64) -> String {
    let failures = failures_json();
    let components = components_json();
    let alerts = alerts_json();
    let breakers = breakers_json();
    let mel_entries = mel_entries_json();
    let chapters = chapters_json(&[failures.as_slice(), components.as_slice(), alerts.as_slice(), breakers.as_slice()]);
    json!({
        "schemaVersion": SCHEMA_VERSION,
        "generatedAtUnixSeconds": generated_at_unix_seconds,
        "chapters": chapters,
        "failures": failures,
        "components": components,
        "alerts": alerts,
        "breakers": breakers,
        "melEntries": mel_entries,
    })
    .to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The real build step: `CATALOGUE_OUT=path/to/catalogue.json cargo
    /// test --lib -- --ignored study::catalogue::tests::dump_catalogue`
    /// (see `docs/catalogue-export.md`). Reads the real clock once, unlike
    /// `catalogue_json` itself, which stays a pure function of its
    /// argument so the determinism test below needs no clock at all.
    #[test]
    #[ignore]
    fn dump_catalogue() {
        if let Ok(path) = std::env::var("CATALOGUE_OUT") {
            let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
            std::fs::write(path, catalogue_json(now)).unwrap();
        }
    }

    #[test]
    fn catalogue_json_is_byte_identical_for_the_same_input() {
        let a = catalogue_json(1_700_000_000);
        let b = catalogue_json(1_700_000_000);
        assert_eq!(a.as_bytes(), b.as_bytes(), "same registries, same timestamp, must be byte-identical");
    }

    /// A different timestamp changes exactly the one field it should, and
    /// nothing about the ordering or content of anything else.
    #[test]
    fn only_the_timestamp_field_moves_with_the_timestamp_argument() {
        let a: Value = serde_json::from_str(&catalogue_json(1)).unwrap();
        let b: Value = serde_json::from_str(&catalogue_json(2)).unwrap();
        assert_eq!(a["generatedAtUnixSeconds"], 1);
        assert_eq!(b["generatedAtUnixSeconds"], 2);
        let mut a = a;
        let mut b = b;
        a["generatedAtUnixSeconds"] = json!(0);
        b["generatedAtUnixSeconds"] = json!(0);
        assert_eq!(a, b);
    }

    /// Every collection's count against the same registry function the
    /// Study endpoints themselves read from, so this cannot silently lose
    /// half the catalogue without a live registry change also failing this
    /// test -- not a hard-coded number, since the `deep` areas (this
    /// crate's own `deep::electrical` chief among them) are still growing.
    #[test]
    fn every_section_carries_its_full_count() {
        let v: Value = serde_json::from_str(&catalogue_json(0)).unwrap();
        let failures = v["failures"].as_array().unwrap();
        let components = v["components"].as_array().unwrap();
        let alerts = v["alerts"].as_array().unwrap();
        let breakers = v["breakers"].as_array().unwrap();
        let mel_entries = v["melEntries"].as_array().unwrap();
        let chapters = v["chapters"].as_array().unwrap();

        let expect_failures = crate::failures::all_ids().len();
        assert_eq!(failures.len(), expect_failures);
        assert!(failures.len() > 5_000, "the full catalogue should be here: {}", failures.len());

        let expect_components = crate::deep::registry().components.len();
        assert_eq!(components.len(), expect_components);
        assert!(components.len() > 1_900, "the deep component catalogue should be here: {}", components.len());

        let expect_alerts = crate::deep::registry().alerts.len();
        assert_eq!(alerts.len(), expect_alerts);
        assert!(alerts.len() > 250, "the ECAM alert catalogue should be here: {}", alerts.len());

        assert_eq!(breakers.len(), crate::deep::breakers::catalog::all().len());
        assert!(breakers.len() > 300, "the ELMS breaker catalogue should be here: {}", breakers.len());

        assert_eq!(mel_entries.len(), crate::mel_catalog::MEL_FAILURES.len());

        assert!(chapters.len() > 15 && chapters.len() < 45, "an implausible chapter count: {}", chapters.len());
    }

    /// Every collection is sorted by its own documented key -- required for
    /// the byte-identical determinism guarantee above to mean anything (an
    /// unsorted collection would only agree with itself, not across runs
    /// with registration order shuffled).
    #[test]
    fn every_collection_is_sorted_by_its_stable_key() {
        let v: Value = serde_json::from_str(&catalogue_json(0)).unwrap();
        let ids: Vec<u64> = v["failures"].as_array().unwrap().iter().map(|f| f["id"].as_u64().unwrap()).collect();
        assert!(ids.windows(2).all(|w| w[0] < w[1]), "failures must be sorted, deduplicated by id");

        let component_ids: Vec<&str> = v["components"].as_array().unwrap().iter().map(|c| c["id"].as_str().unwrap()).collect();
        assert!(component_ids.windows(2).all(|w| w[0] < w[1]), "components must be sorted by id");

        let alert_keys: Vec<&str> = v["alerts"].as_array().unwrap().iter().map(|a| a["key"].as_str().unwrap()).collect();
        assert!(alert_keys.windows(2).all(|w| w[0] < w[1]), "alerts must be sorted by key");

        let breaker_ids: Vec<&str> = v["breakers"].as_array().unwrap().iter().map(|b| b["id"].as_str().unwrap()).collect();
        assert!(breaker_ids.windows(2).all(|w| w[0] < w[1]), "breakers must be sorted by id");

        let mel_refs: Vec<&str> = v["melEntries"].as_array().unwrap().iter().map(|m| m["melReference"].as_str().unwrap()).collect();
        assert!(mel_refs.windows(2).all(|w| w[0] < w[1]), "MEL entries must be sorted by reference");

        let chapter_numbers: Vec<u64> = v["chapters"].as_array().unwrap().iter().map(|c| c["number"].as_u64().unwrap()).collect();
        assert!(chapter_numbers.windows(2).all(|w| w[0] < w[1]), "chapters must be sorted by number");
    }

    /// No entry anywhere carries a fabricated placeholder: every name/title
    /// is real text, every chapter resolves to a real name, and the
    /// magnitude-semantics field is present exactly where the registry
    /// actually states one (never a made-up "0..1 fraction" filler for the
    /// catalogues that do not).
    #[test]
    fn nothing_is_fabricated() {
        let v: Value = serde_json::from_str(&catalogue_json(0)).unwrap();
        for f in v["failures"].as_array().unwrap() {
            assert!(f["name"].as_str().is_some_and(|s| !s.is_empty()));
            assert!(f["cause"].as_str().is_some_and(|s| !s.is_empty()));
            assert_ne!(f["ataChapterName"], "Other", "every catalogued failure has a real chapter: {f}");
            assert!(f["component"].as_str().is_some_and(|s| !s.is_empty()));
        }
        for c in v["components"].as_array().unwrap() {
            assert!(c["name"].as_str().is_some_and(|s| !s.is_empty()));
            assert_ne!(c["ataChapterName"], "Other", "every deep component has a real chapter: {c}");
        }
        for b in v["breakers"].as_array().unwrap() {
            assert!(b["name"].as_str().is_some_and(|s| !s.is_empty()));
            assert!(b["label"].as_str().is_some_and(|s| !s.is_empty()));
            assert!(b["protectsModelledLoad"].is_boolean());
        }
        // A deep failure (id >= 1_000_000) always states its magnitude's
        // physical meaning; the legacy/extra catalogues never fabricate one.
        let deep_failure = v["failures"].as_array().unwrap().iter().find(|f| f["id"].as_u64().unwrap() >= 1_000_000).expect("at least one deep failure");
        assert!(deep_failure["magnitudeSemantics"].is_string(), "{deep_failure}");
        let legacy_failure = v["failures"].as_array().unwrap().iter().find(|f| f["id"].as_u64().unwrap() == 24_020).expect("FlyByWire's own generator failure");
        assert!(legacy_failure["magnitudeSemantics"].is_null(), "{legacy_failure}");
    }

    /// The instance-number heuristic recognises a plain trailing digit and
    /// honestly declines a colour-coded one rather than guessing.
    #[test]
    fn component_instance_is_read_only_from_an_unambiguous_trailing_number() {
        assert_eq!(trailing_instance("49_apu.generator_1"), Some(1));
        assert_eq!(trailing_instance("49_apu.generator_2"), Some(2));
        assert_eq!(trailing_instance("29_hyd.green_accumulator"), None);
        assert_eq!(trailing_instance("29_hyd.green_edp_1a"), None);
        assert_eq!(trailing_instance("24_elec.vfg-1"), Some(1));

        let v: Value = serde_json::from_str(&catalogue_json(0)).unwrap();
        let components = v["components"].as_array().unwrap();
        let gen1 = components.iter().find(|c| c["id"] == "49_apu.generator_1").expect("APU generator 1 is a real component");
        assert_eq!(gen1["instance"], 1);
    }

    /// The legacy gating-breaker catalogue (`crate::breakers::catalog()`)
    /// is deliberately not in here: it carries no panel position at all,
    /// unlike every entry this export does carry one for.
    #[test]
    fn only_the_panel_positioned_breaker_catalogue_is_exported() {
        let v: Value = serde_json::from_str(&catalogue_json(0)).unwrap();
        let breakers = v["breakers"].as_array().unwrap();
        assert_eq!(breakers.len(), crate::deep::breakers::catalog::all().len());
        for b in breakers {
            assert!(b["row"].as_u64().is_some());
            assert!(b["column"].as_u64().is_some());
        }
    }
}
