//! One-off dump of every `SourcePatch` the plugin would splice into
//! FlyByWire's compiled JS, plus the deep-registered `EcamAlert`s' variable
//! names, to JSON files under `D:/A380/fbw-build/wasm-fs2020/ecam-msfs/` —
//! input for the X-Plane -> MSFS ECAM-bridge port (report only, no product
//! code).
//!
//! A **binary**, not a `#[test]`, and deliberately so: `cargo test --lib`
//! (the originally intended approach; see git history of this file for the
//! `#[cfg(test)]`-module version) forces `--cfg test` on the whole crate,
//! which pulls in `src/deep/electrical_crate_parity.rs`
//! (`#[cfg(test)] mod electrical_crate_parity;`, `deep/mod.rs:24`) and
//! therefore the `[dev-dependencies]` crate `deep_electrical`
//! (`Cargo.toml`) — which fails to build on its own, independent of
//! anything in this file (`crates/deep_electrical/src/deep/live.rs`'s own
//! copy of `trait Area` is missing the `as_breakers`/`as_breakers_mut`
//! default methods that `src/deep/breakers/live.rs`'s `impl Area for
//! BreakersLive` now provides — a pre-existing sync gap between that
//! crate's hand-maintained trait copy and this one, unrelated to the ECAM
//! bridge and out of this task's scope to fix). A `[[bin]]` target links
//! the library crate the same way `src/bin/fbw_a380_systems_server.rs`
//! already does — through `[dependencies]` only, with `--cfg test` never
//! set — so `electrical_crate_parity`/`deep_electrical` are never touched
//! and the existing build breakage is sidestepped entirely rather than
//! fixed or worked around in place.
//!
//! Because this runs as a *separate* crate, only items `pub` all the way
//! from the crate root are reachable — `mod ecam_patches`/`mod oans` are
//! each private, so their `source_patches()` cannot be called directly from
//! here. `ecam_patches`'s output (which itself folds in the private `ecl`
//! submodule) is already re-exported as data through the crate's own `pub
//! fn static_source_patches()`. `oans::plugin::source_patches()`'s two
//! patches are reconstructed here byte-for-byte from the same
//! `OLD_LINE`/`NEW_LINE` substitution `oans/plugin.rs`'s own
//! `source_patches()` uses (copied verbatim below; this file does not
//! define new behaviour, only restates existing constant data it cannot
//! reach any other way).
//!
//! Run with `cargo +stable-x86_64-pc-windows-gnu run --release --features
//! js --bin dump_patches`.

use fbw_a380_systems::deep;
use fbw_a380_systems::deep::ecam::cond_json::js_var_name;
use serde::Serialize;
use std::collections::BTreeSet;
use std::path::Path;

#[derive(Serialize)]
struct DumpedPatch {
    path: String,
    find: String,
    replace: String,
    reason: String,
    source: String,
}

#[derive(Serialize)]
struct DumpedAlert {
    key: String,
    ata: u16,
    title: String,
    level: String,
    confirm_s: f64,
    vars: Vec<String>,
    status: Vec<String>,
    inop: Vec<String>,
}

fn cond_vars(c: &deep::api::Cond, out: &mut BTreeSet<String>) {
    use deep::api::Cond;
    match c {
        Cond::Always => {}
        Cond::Var { name, .. } => {
            out.insert(js_var_name(name));
        }
        Cond::VarVar { a, b, .. } => {
            out.insert(js_var_name(a));
            out.insert(js_var_name(b));
        }
        Cond::And(v) | Cond::Or(v) => {
            for c in v {
                cond_vars(c, out);
            }
        }
        Cond::Not(c) => cond_vars(c, out),
    }
}

/// Verbatim copy of `oans/plugin.rs::source_patches()`'s logic (that
/// module is private and unreachable from this separate crate — see file
/// doc comment). Any future edit to the real function must be mirrored
/// here by hand, the same way this codebase already hand-syncs the
/// LegacyFuel/GPUManagement inline patches between `lib.rs` and
/// `js_bridge.rs`.
fn oans_patches() -> Vec<DumpedPatch> {
    const OLD_LINE: &str = "const response = await navigraphRequest.get(`https://amdb.api.navigraph.com/v1/${query}`);";
    const NEW_LINE: &str =
        "const response = { data: await (await fetch(`https://amdb.api.navigraph.com/v1/${query}`)).json() };";
    const NDJS: &str = "/Pages/VCockpit/Instruments/A380X/ND/nd.js";
    let patch = |find: &str, why: &str| -> DumpedPatch {
        DumpedPatch {
            path: NDJS.to_string(),
            find: find.to_string(),
            replace: find.replacen(OLD_LINE, NEW_LINE, 1),
            reason: format!(
                "{why}: navigraphRequest needs XMLHttpRequest, which nothing here defines (axios's \
                 getDefaultAdapter only tries `typeof XMLHttpRequest !== \"undefined\"`), so it is \
                 sent through fetch() instead, which src/oans/plugin.rs answers locally"
            ),
            source: "oans/plugin.rs".to_string(),
        }
    };
    vec![
        patch(
            "    let query = \"search\";\n    query += `?q=${queryString}`;\n    navigraphAuth;\n    const response = await navigraphRequest.get(`https://amdb.api.navigraph.com/v1/${query}`);",
            "amdb.ts searchAmdbAirports",
        ),
        patch(
            "    query += `&include=${includeString}`;\n    navigraphAuth;\n    const response = await navigraphRequest.get(`https://amdb.api.navigraph.com/v1/${query}`);",
            "amdb.ts getAmdbData",
        ),
    ]
}

/// Reasons that identify the two `ecl.rs` patches inside
/// `static_source_patches()`'s flat output (that submodule is private to
/// `ecam_patches.rs` and not reachable from here directly).
const ECL_REASON_MARKERS: &[&str] =
    &["ECAM control panel buttons only carry", "AFTER START checklist's sensed RUDDER TRIM"];

/// Reasons that identify the two hand-carried inline patches
/// (`static_source_patches()`'s own first two entries).
const INLINE_REASON_MARKERS: &[&str] =
    &["LegacyFuel is left out", "GPUManagement is left out"];

/// Reasons that identify the five deep-ECAM-bridge patches
/// (`deep/ecam/patches.rs::source_patches`'s own `reason` text, shared by
/// all five).
const DEEP_ECAM_REASON_MARKER: &str = "deep ECAM bridge:";

fn tag(p: fbw_a380_systems::source_patch::SourcePatch) -> DumpedPatch {
    let source = if INLINE_REASON_MARKERS.iter().any(|m| p.reason.contains(m)) {
        "lib.rs/js_bridge.rs inline (native instrument replacement)".to_string()
    } else if ECL_REASON_MARKERS.iter().any(|m| p.reason.contains(m)) {
        "ecam_patches/ecl.rs (Electronic Checklist / ECP)".to_string()
    } else if p.reason.contains(DEEP_ECAM_REASON_MARKER) {
        "deep/ecam/patches.rs".to_string()
    } else {
        "ecam_patches.rs".to_string()
    };
    DumpedPatch { path: p.path, find: p.find, replace: p.replace, reason: p.reason, source }
}

fn main() {
    let out_dir = Path::new("D:/A380/fbw-build/wasm-fs2020/ecam-msfs");
    std::fs::create_dir_all(out_dir).expect("create out dir");

    let registry = deep::registry();

    // `static_source_patches()` = the 2 inline patches + `ecam_patches`
    // (incl. `ecl`, 14 total) + the 5 deep-ECAM-bridge patches. `oans`'s 2
    // are reconstructed separately (see file doc comment).
    let mut patches: Vec<DumpedPatch> =
        fbw_a380_systems::static_source_patches().into_iter().map(tag).collect();
    patches.extend(oans_patches());

    let json = serde_json::to_string_pretty(&patches).expect("serialize patches");
    std::fs::write(out_dir.join("patches.json"), json).expect("write patches.json");

    let alerts: Vec<DumpedAlert> = registry
        .alerts
        .iter()
        .map(|a| {
            let mut vars = BTreeSet::new();
            cond_vars(&a.trigger, &mut vars);
            for line in &a.procedure {
                cond_vars(&line.applies_if, &mut vars);
                if let Some(c) = &line.done_when {
                    cond_vars(c, &mut vars);
                }
            }
            DumpedAlert {
                key: a.key.clone(),
                ata: a.ata,
                title: a.title.clone(),
                level: format!("{:?}", a.level),
                confirm_s: a.confirm_s,
                vars: vars.into_iter().collect(),
                status: a.status.clone(),
                inop: a.inop.clone(),
            }
        })
        .collect();
    let alerts_json = serde_json::to_string_pretty(&alerts).expect("serialize alerts");
    std::fs::write(out_dir.join("alerts.json"), alerts_json).expect("write alerts.json");

    println!("dumped {} patches and {} deep ECAM alerts to {}", patches.len(), alerts.len(), out_dir.display());
}
