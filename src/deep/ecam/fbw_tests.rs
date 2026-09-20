//! Standing guards for [`crate::deep::ecam::fbw`], the entries that give
//! FlyByWire's own abnormal-sensed procedures the trigger they never had.
//!
//! Three of these are the ones that matter most, and they are here rather
//! than in a doc because a doc cannot fail:
//!
//! * [`no_wired_trigger_reads_a_variable_nobody_publishes`] -- the exact bug
//!   this whole task exists to fix (104 of our own alerts had it). A trigger
//!   over a variable no area publishes reads 0 for ever, so the alert either
//!   never fires or is stuck on from load.
//! * [`no_entry_takes_an_id_flybywire_already_triggers`] -- a procedure with
//!   two triggers is worse than one with none, and FlyByWire adds entries to
//!   `FwsAbnormalSensed.ts` release by release. This re-reads its source
//!   every run.
//! * [`every_item_vector_matches_the_procedures_own_item_count`] -- the ECL
//!   renders these; an entry whose `whichItemsChecked()` is the wrong length
//!   for its procedure is what `FwsCore.ts:5594-5607` warns about, and would
//!   leave the checklist page operating against a mis-sized state vector.
//!
//! The reachability tests below then do what a well-formed trigger still
//! cannot prove on its own: arm the real failure (or set the real aircraft
//! state), tick the real areas, and check the condition actually goes true.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

use crate::deep::api::{Cond, Level};
use crate::deep::ecam::fbw::{self, FbwProc};
use crate::deep::integration::failure_audit::{bare, cond_vars, reachability, Tri};
use crate::deep::live::{all_areas, Controls, Faults, Truth};

// ---------------------------------------------------------------------------
// FlyByWire's own source, read at test time.
// ---------------------------------------------------------------------------

const FBW_SRC: &str = r"D:\fbw-aircraft\fbw-a380x\src\systems";

fn abnormal_sensed_dir() -> PathBuf {
    Path::new(FBW_SRC).join("instruments/src/MsfsAvionicsCommon/EcamMessages/AbnormalSensed")
}

fn fws_abnormal_sensed() -> PathBuf {
    Path::new(FBW_SRC).join("systems-host/CpiomC/FlightWarningSystem/FwsAbnormalSensed.ts")
}

/// FlyByWire's tree is a read-only reference that is present on a
/// development machine and absent on a bare checkout. Every test that reads
/// it skips rather than fails when it is missing, the same way
/// `patches.rs`'s own build-tree test does.
fn have_fbw_source() -> bool {
    fws_abnormal_sensed().is_file() && abnormal_sensed_dir().is_dir()
}

/// Strips `//` and `/* */` comments without touching string literals, so a
/// comment cannot hide a brace and an apostrophe inside a comment cannot be
/// mistaken for a quote.
fn strip_comments(src: &str) -> String {
    let b: Vec<char> = src.chars().collect();
    let mut out = String::with_capacity(src.len());
    let mut i = 0;
    while i < b.len() {
        let c = b[i];
        if c == '"' || c == '\'' || c == '`' {
            out.push(c);
            i += 1;
            while i < b.len() {
                out.push(b[i]);
                if b[i] == '\\' {
                    i += 1;
                    if i < b.len() {
                        out.push(b[i]);
                        i += 1;
                    }
                    continue;
                }
                if b[i] == c {
                    i += 1;
                    break;
                }
                i += 1;
            }
            continue;
        }
        if c == '/' && i + 1 < b.len() && b[i + 1] == '/' {
            while i < b.len() && b[i] != '\n' {
                i += 1;
            }
            continue;
        }
        if c == '/' && i + 1 < b.len() && b[i + 1] == '*' {
            i += 2;
            while i + 1 < b.len() && !(b[i] == '*' && b[i + 1] == '/') {
                i += 1;
            }
            i += 2;
            continue;
        }
        out.push(c);
        i += 1;
    }
    out
}

/// Walks a `[...]`/`{...}` block from `start` (which must sit on the opening
/// bracket) and returns the byte index just past its match, skipping string
/// literals. Byte-indexed throughout: these files open with a UTF-8 BOM, so
/// a char index and a byte index are not the same thing, and mixing them is
/// how a parser reads the wrong procedure without saying so.
fn match_block(s: &[u8], start: usize, open: u8, close: u8) -> Option<usize> {
    let mut depth = 0i32;
    let mut i = start;
    while i < s.len() {
        let c = s[i];
        if c == b'"' || c == b'\'' || c == b'`' {
            i += 1;
            while i < s.len() {
                if s[i] == b'\\' {
                    i += 2;
                    continue;
                }
                if s[i] == c {
                    break;
                }
                i += 1;
            }
            i += 1;
            continue;
        }
        if c == open {
            depth += 1;
        } else if c == close {
            depth -= 1;
            if depth == 0 {
                return Some(i + 1);
            }
        }
        i += 1;
    }
    None
}

/// The number of top-level elements in an array body -- one per checklist
/// item, counting a spread element (`{ ...FMS_PRED_UNRELIABLE_CHECKLIST_ITEM }`)
/// as the single item it becomes.
fn count_top_level(body: &[u8]) -> usize {
    let mut depth = 0i32;
    let mut n = 0usize;
    let mut saw_content = false;
    let mut i = 0;
    while i < body.len() {
        let c = body[i];
        if c == b'"' || c == b'\'' || c == b'`' {
            saw_content = true;
            i += 1;
            while i < body.len() {
                if body[i] == b'\\' {
                    i += 2;
                    continue;
                }
                if body[i] == c {
                    break;
                }
                i += 1;
            }
            i += 1;
            continue;
        }
        match c {
            b'{' | b'[' | b'(' => {
                depth += 1;
                saw_content = true;
            }
            b'}' | b']' | b')' => depth -= 1,
            b',' if depth == 0 => {
                if saw_content {
                    n += 1;
                }
                saw_content = false;
            }
            c if !c.is_ascii_whitespace() => saw_content = true,
            _ => {}
        }
        i += 1;
    }
    if saw_content {
        n += 1;
    }
    n
}

/// Every nine-digit id at a given indent in a file, with the byte offset of
/// the `{` that opens its body.
fn entries_at_indent(raw: &str, indent: usize) -> Vec<(u64, usize)> {
    let bytes = raw.as_bytes();
    let needle = format!("\n{}", " ".repeat(indent));
    let mut out = Vec::new();
    let mut i = 0usize;
    while let Some(rel) = raw[i..].find(&needle) {
        let at = i + rel + needle.len();
        i = at;
        if at + 10 > bytes.len() {
            break;
        }
        let digits = &bytes[at..at + 9];
        if !digits.iter().all(u8::is_ascii_digit) || bytes[at + 9] != b':' {
            continue;
        }
        // Not a longer number that merely starts with nine digits.
        if bytes.get(at.wrapping_sub(1)).is_some_and(u8::is_ascii_digit) {
            continue;
        }
        let id: u64 = std::str::from_utf8(digits).expect("ascii").parse().expect("nine digits");
        match bytes[at + 10..].iter().position(|&c| c == b'{') {
            Some(p) => out.push((id, at + 10 + p)),
            None => break,
        }
    }
    out
}

/// One abnormal procedure as FlyByWire's own catalogue defines it.
struct Defined {
    items: usize,
    /// The first `\x1b<Nm` colour code in the title, FlyByWire's own
    /// statement of the alert's level: 2 = red, 3 = green, 4 = amber.
    colour: Option<char>,
}

fn defined_procedures() -> std::collections::BTreeMap<u64, Defined> {
    let mut out = std::collections::BTreeMap::new();
    let mut files: Vec<PathBuf> = std::fs::read_dir(abnormal_sensed_dir())
        .expect("FlyByWire's AbnormalSensed directory")
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "ts"))
        .collect();
    files.sort();
    for path in files {
        let raw = strip_comments(&std::fs::read_to_string(&path).expect("read"));
        let bytes = raw.as_bytes();
        for (id, brace) in entries_at_indent(&raw, 2) {
            let Some(end) = match_block(bytes, brace, b'{', b'}') else { continue };
            let body = &bytes[brace + 1..end - 1];
            let body_s = String::from_utf8_lossy(body).into_owned();
            let items = match body_s.find("items:").and_then(|p| body[p..].iter().position(|&c| c == b'[').map(|q| p + q)) {
                Some(open) => match match_block(body, open, b'[', b']') {
                    Some(close) => count_top_level(&body[open + 1..close - 1]),
                    None => 0,
                },
                None => 0,
            };
            let colour = body_s.find("title:").and_then(|p| body_s[p..].find("\x1b<").map(|q| p + q + 5)).and_then(|p| body_s[p..].chars().next());
            out.insert(id, Defined { items, colour });
        }
    }
    out
}

/// Every id FlyByWire's own `ewdAbnormalSensed` map triggers. Its entries
/// sit at four-space indent; `ewdDeferredProcs` in the same file uses the
/// same shape, which is why the caller intersects this with the defined set
/// rather than trusting its size alone.
fn flybywire_wired() -> BTreeSet<u64> {
    let raw = strip_comments(&std::fs::read_to_string(fws_abnormal_sensed()).expect("read FwsAbnormalSensed.ts"));
    entries_at_indent(&raw, 4).into_iter().map(|(id, _)| id).collect()
}

// ---------------------------------------------------------------------------
// Shared helpers.
// ---------------------------------------------------------------------------

fn published_set() -> BTreeSet<String> {
    all_areas().published_names().iter().map(|n| bare(n).to_owned()).collect()
}

/// Variables a per-item `checked`/`show` condition may read although no deep
/// area publishes them: cockpit controls the *plugin* owns, each verified to
/// be written somewhere in this repository. This is the same allowance
/// `failure_audit::PLUGIN_OWNED_TRIGGER_VARS` makes, and it applies to
/// procedure lines only -- a line reads back the control the crew moved,
/// which is by definition not a deep area's output. A *trigger* gets no such
/// allowance (see [`no_wired_trigger_reads_a_variable_nobody_publishes`]).
const PLUGIN_OWNED_ITEM_VARS: &[&str] = &[
    // The overhead engine-generator pushbuttons: `src/aspects.rs:612-618`
    // copies MSFS's `GENERAL ENG MASTER ALTERNATOR:n` into each of these
    // every frame.
    "OVHD_ELEC_ENG_GEN_1_PB_IS_ON",
    "OVHD_ELEC_ENG_GEN_2_PB_IS_ON",
    "OVHD_ELEC_ENG_GEN_3_PB_IS_ON",
    "OVHD_ELEC_ENG_GEN_4_PB_IS_ON",
];

fn vars_of(c: &Cond) -> Vec<String> {
    let mut v = Vec::new();
    cond_vars(c, &mut v);
    v.iter().map(|n| bare(n).to_owned()).collect()
}

/// Steps the real areas from a given state and returns a reader over
/// everything they published, for evaluating a trigger exactly as the JS
/// shim would.
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

/// A failure id from the combined registry, found by a predicate over its
/// component and name rather than hard-coded, so this fails loudly if an
/// area renames one instead of silently arming nothing.
fn failure_id(pred: impl Fn(&crate::deep::api::FailureDef) -> bool, what: &str) -> u64 {
    let r = crate::deep::registry();
    let hits: Vec<u64> = r.failures.iter().filter(|f| pred(f)).map(|f| f.id).collect();
    assert!(!hits.is_empty(), "no registered failure matches {what}");
    hits[0]
}

// ---------------------------------------------------------------------------
// The standing guards.
// ---------------------------------------------------------------------------

#[test]
fn no_wired_trigger_reads_a_variable_nobody_publishes() {
    // `Cond::eval` (and the JS shim's `evalCond`, through `SimVar`) reads a
    // variable nothing publishes as 0. So a trigger naming one is not
    // "probably fine": it is a fixed comparison against zero, which is
    // either permanently false (an alert that can never appear) or
    // permanently true (an alert that is on from the moment the aircraft
    // loads). Both are the bug this module was written to stop repeating.
    let published = published_set();
    let mut bad = Vec::new();
    for p in fbw::wirings() {
        for v in vars_of(&p.trigger) {
            if !published.contains(&v) {
                bad.push(format!("{} ({}) reads unpublished {v}", p.id, p.title));
            }
        }
    }
    assert!(bad.is_empty(), "{} wired triggers read a variable nobody publishes:\n{}", bad.len(), bad.join("\n"));
}

#[test]
fn no_wired_trigger_is_permanently_true_or_permanently_false() {
    // The other face of the same bug: a trigger every one of whose
    // variables is published can still be stuck if it was written wrong.
    // `failure_audit::reachability` is the same three-valued check our own
    // 304 alerts are held to.
    let published = published_set();
    let known = |n: &str| published.contains(n);
    for p in fbw::wirings() {
        assert_eq!(reachability(&p.trigger, &known), Tri::Reachable, "{} ({}) is not reachable: its trigger has a fixed answer", p.id, p.title);
    }
}

#[test]
fn every_item_condition_reads_a_published_or_a_known_plugin_owned_variable() {
    let published = published_set();
    let mut bad = Vec::new();
    for p in fbw::wirings() {
        for it in &p.items {
            for c in [it.show.as_ref(), it.checked.as_ref()].into_iter().flatten() {
                for v in vars_of(c) {
                    if !published.contains(&v) && !PLUGIN_OWNED_ITEM_VARS.contains(&v.as_str()) {
                        bad.push(format!("{} ({}) item {} reads {v}", p.id, p.title, it.index));
                    }
                }
            }
        }
    }
    assert!(bad.is_empty(), "items reading variables nothing writes:\n{}", bad.join("\n"));
}

#[test]
fn every_entry_has_a_distinct_nine_digit_flybywire_id() {
    let procs = fbw::wirings();
    let ids: BTreeSet<u64> = procs.iter().map(|p| p.id).collect();
    assert_eq!(ids.len(), procs.len(), "two entries claim the same procedure id");
    for p in &procs {
        assert!((100_000_000..1_000_000_000).contains(&p.id), "{} is not a nine-digit FlyByWire id", p.id);
    }
    // Sorted, so the generated JS array is stable run to run.
    assert!(procs.windows(2).all(|w| w[0].id < w[1].id));
}

#[test]
fn every_suppressor_is_itself_a_defined_procedure() {
    // `notActiveWhenItemActive` names ids FlyByWire looks up in
    // `allSuppressableItems`; naming one that does not exist is a silently
    // dead suppression (`FwsCore.ts:5507`'s `if (val && this...[val])`).
    let procs = fbw::wirings();
    let wired: BTreeSet<u64> = procs.iter().map(|p| p.id).collect();
    for p in &procs {
        for s in p.suppressed_by {
            assert!(wired.contains(s), "{} is suppressed by {s}, which nothing triggers, so the suppression can never happen", p.id);
        }
    }
}

#[test]
fn the_defined_and_wired_counts_are_what_this_module_was_built_against() {
    if !have_fbw_source() {
        return;
    }
    let defined = defined_procedures();
    let wired = flybywire_wired();
    let unwired = defined.keys().filter(|id| !wired.contains(id)).count();
    println!("FBW abnormal sensed: defined {} | FlyByWire triggers {} | unwired {} | this port triggers {}", defined.len(), wired.len(), unwired, fbw::wirings().len());
    assert_eq!(defined.len(), 1004, "FlyByWire's catalogue changed size; re-read docs/deep/fbw_unwired.md before trusting its counts");
    assert_eq!(wired.iter().filter(|id| defined.contains_key(id)).count(), 272, "FlyByWire wired or unwired a procedure; re-check which of ours are still needed");
}

#[test]
fn no_entry_takes_an_id_flybywire_already_triggers() {
    if !have_fbw_source() {
        return;
    }
    let wired = flybywire_wired();
    for p in fbw::wirings() {
        assert!(!wired.contains(&p.id), "{} ({}) is already triggered by FwsAbnormalSensed.ts -- two triggers on one procedure is worse than none", p.id, p.title);
    }
}

#[test]
fn every_entry_names_a_procedure_flybywire_actually_defines() {
    if !have_fbw_source() {
        return;
    }
    let defined = defined_procedures();
    for p in fbw::wirings() {
        assert!(defined.contains_key(&p.id), "{} ({}) is not in FlyByWire's own catalogue, so there is no title or checklist to raise", p.id, p.title);
    }
}

#[test]
fn every_item_vector_matches_the_procedures_own_item_count() {
    if !have_fbw_source() {
        return;
    }
    let defined = defined_procedures();
    for p in fbw::wirings() {
        let real = defined[&p.id].items;
        assert_eq!(p.item_count, real, "{} ({}) records {} items; FlyByWire's own definition has {real}", p.id, p.title, p.item_count);
        for it in &p.items {
            assert!(it.index < real.max(1), "{} ({}) wires item {} of a {real}-item procedure", p.id, p.title, it.index);
        }
    }
}

#[test]
fn no_entry_is_louder_than_flybywires_own_title_colour() {
    // FlyByWire states the alert's level in the title itself: `\x1b<2m` is
    // red, `\x1b<4m` amber, `\x1b<3m` green. Giving a procedure a master
    // warning its own title says is amber would put a red line and a CRC on
    // the flight deck for an alert Airbus made amber.
    if !have_fbw_source() {
        return;
    }
    let defined = defined_procedures();
    for p in fbw::wirings() {
        let Some(colour) = defined[&p.id].colour else { continue };
        match colour {
            '2' => {}
            '4' => assert_ne!(p.level, Level::Warning, "{} ({}) is amber in FlyByWire's own title but wired as a red warning", p.id, p.title),
            _ => assert!(matches!(p.level, Level::Advisory | Level::Memo), "{} ({}) is not amber or red in FlyByWire's own title but is wired above advisory", p.id, p.title),
        }
    }
}

// ---------------------------------------------------------------------------
// Reachability: the real failure, the real areas, the real condition.
// ---------------------------------------------------------------------------

fn engines_running() -> Truth {
    Truth {
        dt_s: 0.1,
        on_ground: false,
        engine_running: [true; 4],
        engine_n2_frac: [0.9; 4],
        engine_n3_frac: [0.95; 4],
        engine_oil_pressure_pa: [520_000.0; 4],
        engine_oil_temp_c: [90.0; 4],
        ..Truth::default()
    }
}

#[test]
fn eng_oil_press_lo_fires_when_the_oil_pressure_falls_with_the_core_turning() {
    let healthy = run(engines_running(), &Faults::default(), 20);
    let low = run(Truth { engine_oil_pressure_pa: [520_000.0, 100_000.0, 520_000.0, 520_000.0], ..engines_running() }, &Faults::default(), 20);
    for (eng, id) in (1..=4u64).map(|e| (e, 701_800_084 + e)) {
        let p = wiring(id);
        assert!(!holds(&p.trigger, &healthy), "ENG {eng} OIL PRESS LO must be quiet on a healthy engine");
        assert_eq!(holds(&p.trigger, &low), eng == 2, "only engine 2's oil pressure was dropped, but ENG {eng} OIL PRESS LO read {}", holds(&p.trigger, &low));
    }
}

#[test]
fn eng_oil_press_lo_stays_quiet_on_a_shut_down_engine() {
    // A cold engine has no oil pressure and that is not a fault. Without
    // the core-running gate every one of these four would be on from the
    // moment the aircraft loaded.
    let cold = run(Truth { dt_s: 0.1, ..Truth::default() }, &Faults::default(), 20);
    for e in 1..=4u64 {
        assert!(!holds(&wiring(701_800_084 + e).trigger, &cold), "ENG {e} OIL PRESS LO fired on a shut-down engine");
    }
}

#[test]
fn eng_ign_a_fault_fires_when_that_chains_exciter_dies_and_leaves_the_other_alone() {
    // Ignition is only energised while a start or continuous ignition is
    // selected, which is what `starter_engaged` drives here.
    let starting = Truth {
        dt_s: 0.1,
        // `step_ignition` energises both exciters while the starter is
        // engaged and the engine's own AC bus is live
        // (`engine_accessories/live.rs:1124-1127`).
        ac_bus_volts: [115.0; 4],
        controls: Controls { starter_engaged: [true; 4], ..Controls::default() },
        engine_n2_frac: [0.2; 4],
        engine_n3_frac: [0.25; 4],
        ..Truth::default()
    };
    let healthy = run(starting.clone(), &Faults::default(), 20);
    let a2 = failure_id(|f| f.name.contains("Engine 2 ignition exciter A failure"), "engine 2 exciter A");
    let armed = run(starting, &Faults::from_pairs([(a2, 1.0)]), 20);

    let ign_a2 = wiring(701_800_058);
    let ign_b2 = wiring(701_800_062);
    assert!(!holds(&ign_a2.trigger, &healthy), "ENG 2 IGN A FAULT must be quiet with both exciters healthy");
    assert!(holds(&ign_a2.trigger, &armed), "ENG 2 IGN A FAULT must fire when exciter A is dead and the igniters are energised");
    assert!(!holds(&ign_b2.trigger, &armed), "chain B is untouched, so ENG 2 IGN B FAULT must stay quiet");
    assert!(!holds(&wiring(701_800_057).trigger, &armed), "engine 1 is untouched");
}

#[test]
fn eng_ign_fault_stays_quiet_when_the_igniters_are_not_energised() {
    // A dead exciter is invisible, and correctly so, while nothing is
    // asking it to spark: the fault is annunciated when ignition is
    // selected, not from the gate on a cold aeroplane.
    let a2 = failure_id(|f| f.name.contains("Engine 2 ignition exciter A failure"), "engine 2 exciter A");
    let quiet = run(Truth { dt_s: 0.1, ac_bus_volts: [115.0; 4], ..Truth::default() }, &Faults::from_pairs([(a2, 1.0)]), 20);
    assert!(!holds(&wiring(701_800_058).trigger, &quiet));
}

#[test]
fn elec_gen_fault_fires_when_that_generator_fails() {
    let flying = engines_running();
    let healthy = run(flying.clone(), &Faults::default(), 30);
    let vfg2 = failure_id(|f| f.component == "24_elec.vfg-2" && f.model_field.contains("regulator_drift"), "the number 2 generator's regulator drift");
    let armed = run(flying, &Faults::from_pairs([(vfg2, 1.0)]), 30);

    let gen2 = wiring(240_800_062);
    assert!(!holds(&gen2.trigger, &healthy), "ELEC GEN 2 FAULT must be quiet with every generator healthy");
    assert!(holds(&gen2.trigger, &armed), "ELEC GEN 2 FAULT must fire when generator 2 fails");
    assert!(!holds(&wiring(240_800_063).trigger, &armed), "generator 3 is untouched, so its own procedure must stay quiet");
}

#[test]
fn an_elec_bus_fault_needs_the_network_to_be_alive() {
    // Cold and dark, every bus is unpowered and none of it is a fault. The
    // network-alive gate is what keeps all eight bus procedures off the
    // flight deck at the gate.
    let cold = run(Truth { dt_s: 0.1, ..Truth::default() }, &Faults::default(), 30);
    for id in [240_800_004, 240_800_007, 240_800_009, 240_800_010, 240_800_012, 240_800_026, 240_800_029, 240_800_030] {
        let p = wiring(id);
        assert!(!holds(&p.trigger, &cold), "{} ({}) fired on a cold and dark aircraft", p.id, p.title);
    }
}

#[test]
fn lavatory_smoke_fires_on_the_deck_that_is_burning_and_not_on_the_other() {
    // The whole point of the cabin-deck smoke source added for these two:
    // before it, `deep::thermal_zones` injected smoke only into the cargo
    // bays, the nacelles and the APU bay, so the eight lavatory detectors
    // could never alarm and a trigger over them would have been a
    // well-formed alert that never fired. This arms the real failure and
    // checks it actually reaches the detectors -- and only the four on the
    // deck that is burning.
    let cruise = Truth { dt_s: 1.0, on_ground: false, engine_running: [true; 4], ..Truth::default() };
    let main = failure_id(|f| f.component == "26_thermal.cabin_main_deck_lavatory_fire_load", "the main-deck lavatory fire");
    let upper = failure_id(|f| f.component == "26_thermal.cabin_upper_deck_lavatory_fire_load", "the upper-deck lavatory fire");

    let quiet = run(cruise.clone(), &Faults::default(), 300);
    // Long enough for the smoke to build in a 700 m^3 deck, which is what
    // makes this a fire and not an instant discrete.
    let burning = run(cruise.clone(), &Faults::from_pairs([(main, 1.0)]), 300);
    let burning_upper = run(cruise, &Faults::from_pairs([(upper, 1.0)]), 300);

    let main_deck = wiring(260_800_065);
    let upper_deck = wiring(260_800_066);
    assert!(!holds(&main_deck.trigger, &quiet) && !holds(&upper_deck.trigger, &quiet), "no fire, no lavatory smoke warning");
    assert!(holds(&main_deck.trigger, &burning), "a main-deck lavatory waste-bin fire must reach the four main-deck detectors");
    assert!(!holds(&upper_deck.trigger, &burning), "and not the upper-deck procedure");
    assert!(holds(&upper_deck.trigger, &burning_upper), "and the upper deck the same way round");
    assert!(!holds(&main_deck.trigger, &burning_upper), "and not the main-deck procedure");
}

#[test]
fn elec_cb_tripped_separates_a_protection_trip_from_a_breaker_the_crew_pulled() {
    // `BREAKERS_OPEN_COUNT` could not carry this procedure because it
    // counts a crew-pulled breaker too. The aggregate this wiring reads is
    // built from `trip::Breaker::status()`, so it must be zero on a
    // healthy aircraft and rise only when a protection element opens
    // something. The unit-level half of the distinction (pull one by hand,
    // the count stays at zero) is
    // `deep::breakers::live::tests::a_crew_pulled_breaker_is_open_but_not_tripped`;
    // this end is that the trigger is reachable at all from a real failure.
    let flying = Truth { dt_s: 0.05, on_ground: false, engine_running: [true; 4], engine_n1_frac: [0.9; 4], engine_n2_frac: [0.9; 4], engine_n3_frac: [0.9; 4], ..Truth::default() };
    let p = wiring(240_800_021);
    let healthy = run(flying.clone(), &Faults::default(), 200);
    assert!(!holds(&p.trigger, &healthy), "a healthy aircraft trips no breaker, so ELEC C/B TRIPPED must be silent");

    // A drifted trip point opens a breaker under its own normal load --
    // the registered effect of `trip_calibration_drift`, and a genuine
    // protection trip rather than a command. Armed across the whole
    // catalogue rather than on one hand-picked id: at 40 % low a breaker
    // only trips if it is actually carrying most of its own design load,
    // and which units those are is `deep::electrical`'s business, not this
    // test's. `deep::breakers`' own
    // `a_drifted_trip_point_opens_a_breaker_under_its_own_normal_load...`
    // is the test that says *which* one and that its load goes dead; this
    // one only has to show the aggregate reaches the trigger.
    let drift: Vec<(u64, f64)> = crate::deep::registry()
        .failures
        .iter()
        .filter(|f| f.component.starts_with("17_breakers.") && f.model_field.contains("trip_calibration_drift"))
        .map(|f| (f.id, 1.0))
        .collect();
    assert!(!drift.is_empty(), "deep::breakers registers a trip-calibration drift per unit");
    // The I^2t element integrates in real time; 120 s at 0.05 s frames is
    // the same window `deep::breakers`' own test uses.
    let armed = run(flying, &Faults::from_pairs(drift), 2_400);
    assert!(holds(&p.trigger, &armed), "a breaker opened by its own thermal element must raise ELEC C/B TRIPPED");
}

#[test]
fn eng_oil_temp_hi_fires_on_hot_oil_with_the_core_turning_and_not_on_a_cold_aircraft() {
    let healthy = run(engines_running(), &Faults::default(), 20);
    let hot = run(Truth { engine_oil_temp_c: [90.0, 90.0, 200.0, 90.0], ..engines_running() }, &Faults::default(), 20);
    // A transducer pegged high on a parked aeroplane: the core-running
    // gate is what keeps that off the flight deck.
    let cold = run(Truth { dt_s: 0.1, engine_oil_temp_c: [250.0; 4], ..Truth::default() }, &Faults::default(), 20);
    for (eng, id) in (1..=4u64).map(|e| (e, 701_800_092 + e)) {
        let p = wiring(id);
        assert!(!holds(&p.trigger, &healthy), "ENG {eng} OIL TEMP HI must be quiet at a normal 90 C");
        assert_eq!(holds(&p.trigger, &hot), eng == 3, "only engine 3's oil was taken past 177 C, but ENG {eng} OIL TEMP HI read {}", holds(&p.trigger, &hot));
        assert!(!holds(&p.trigger, &cold), "ENG {eng} OIL TEMP HI fired on a shut-down engine");
    }
}

#[test]
fn wheel_tire_press_lo_fires_on_a_soft_tyre_and_not_on_a_serviced_one() {
    let mut soft = Truth { dt_s: 0.1, on_ground: true, ..Truth::default() };
    let serviced = run(Truth { dt_s: 0.1, on_ground: true, ..Truth::default() }, &Faults::default(), 20);
    soft.tyre_pressure_pa[3] = 0.8 * crate::physics::tyre::COLD_PRESSURE_PA;
    let flat = run(soft, &Faults::default(), 20);

    let p = wiring(320_800_063);
    assert!(!holds(&p.trigger, &serviced), "every tyre at its service pressure must not raise WHEEL TIRE PRESS LO");
    assert!(holds(&p.trigger, &flat), "a tyre 20 % down must raise WHEEL TIRE PRESS LO");
}

#[test]
fn fuel_trim_pump_fault_separates_one_failed_pump_from_both() {
    let cruise = Truth { dt_s: 0.5, on_ground: false, engine_running: [true; 4], ..Truth::default() };
    let left = failure_id(|f| f.component == "28_fuel.pump.trim_left", "the left trim tank pump");
    let right = failure_id(|f| f.component == "28_fuel.pump.trim_right", "the right trim tank pump");

    let healthy = run(cruise.clone(), &Faults::default(), 20);
    let one = run(cruise.clone(), &Faults::from_pairs([(left, 1.0)]), 20);
    let both = run(cruise, &Faults::from_pairs([(left, 1.0), (right, 1.0)]), 20);

    let l = wiring(281_800_089);
    let r = wiring(281_800_090);
    let lr = wiring(281_800_091);
    assert!(!holds(&l.trigger, &healthy) && !holds(&r.trigger, &healthy) && !holds(&lr.trigger, &healthy));
    assert!(holds(&l.trigger, &one), "the left pump's own procedure must fire when only it has failed");
    assert!(!holds(&r.trigger, &one) && !holds(&lr.trigger, &one), "and neither of the other two");
    assert!(holds(&lr.trigger, &both), "both pumps failed must raise the L+R procedure");
    assert!(!holds(&l.trigger, &both) && !holds(&r.trigger, &both), "and neither single-pump procedure, which the L+R one also suppresses");
}

#[test]
fn lg_gear_not_locked_up_and_doors_not_closed_separate_a_jam_from_a_clean_retraction() {
    // A clean retraction: lever up, airborne, run well past the modelled
    // travel (`retraction::GEAR_NOMINAL_TRAVEL_S` 8 s + `DOOR_NOMINAL_TRAVEL_S`
    // 4 s at each end). Both procedures must be silent.
    let airborne = Truth {
        dt_s: 0.1,
        on_ground: false,
        altitude_ft: 3_000.0,
        hydraulic_pressure_pa: [5000.0 * 6894.757; 2],
        controls: Controls { gear_lever_down: false, ..Controls::default() },
        ..Truth::default()
    };
    let clean = run(airborne.clone(), &Faults::default(), 600);
    let not_up = wiring(320_800_040);
    let doors = wiring(320_800_036);
    assert!(!holds(&not_up.trigger, &clean), "a clean retraction must not raise L/G GEAR NOT LOCKED UP");
    assert!(!holds(&doors.trigger, &clean), "a clean retraction must not raise L/G DOORS NOT CLOSED");

    // A leg that cannot retract: the uplock condition is what separates
    // this from the clean case, and it is read from the same per-leg
    // variables `deep::gear_structure` publishes for its own alerts.
    let jam = failure_id(|f| f.component.ends_with("_retraction") && f.model_field.contains("RetractionFaults.actuator_leak"), "a gear retraction actuator internal leak");
    let jammed = run(airborne, &Faults::from_pairs([(jam, 1.0)]), 600);
    assert!(holds(&not_up.trigger, &jammed), "a jammed retraction must raise L/G GEAR NOT LOCKED UP");
    assert!(!holds(&doors.trigger, &jammed), "and not the doors procedure, whose trigger needs every leg up-locked first");
}

#[test]
fn the_per_instance_counts_these_triggers_enumerate_are_still_what_the_areas_publish() {
    // Three triggers iterate a fixed range of instances -- sixteen tyre
    // pressure transducers, five landing gear legs and their doors. If an
    // area adds a sixteenth wheel or a sixth leg, the range would silently
    // stop covering it: the alert would still fire, just never for the new
    // one, which is the quietest kind of wrong. `..._reads_a_variable_nobody
    // _publishes` catches a range that grew too long; this catches one that
    // is now too short.
    let published = published_set();
    let count = |prefix: &str| published.iter().filter(|n| n.starts_with(prefix)).count();
    // The tyre transducers are not listed here: `ata32::tyre_pressure_vars`
    // builds that trigger from the published names themselves, so it cannot
    // drift. What it must not be is *empty*, which would make the procedure
    // unraisable -- `no_wired_trigger_is_permanently_true_or_permanently_false`
    // is what catches that.
    assert!(count("DEEP_TYRE_PRESSURE_SENSED_PA:") > 0, "nothing publishes a tyre pressure transducer any more");
    assert_eq!(count("GEAR_UPLOCKED:"), 5, "the two L/G procedures enumerate 5 legs (ata32::LEGS)");
    assert_eq!(count("GEAR_DOOR_POSITION:"), 5, "L/G DOORS NOT CLOSED enumerates 5 doors (ata32::LEGS)");
    assert_eq!(count("FUEL_TRIM_PUMP_DEGRADATION:"), 2, "the trim pump procedures name both pumps");
    assert_eq!(count("DEEP_SMOKE_LAV_"), 16, "eight lavatory detectors, each with a reading and an alarm");
}
