//! `SourcePatch`es that splice registered `EcamAlert`s into FlyByWire's own
//! compiled JS (the exact mechanism `ecam_patches.rs`, `wxr/mod.rs` and
//! `oans/plugin.rs` already use, assembled by `js_bridge.rs::native_ports`).
//! See `docs/deep/ecam_bridge.md` for the full design and the one-line
//! `js_bridge.rs` change (not made here -- an existing file this directory
//! may not edit) that would wire `source_patches` in alongside those three.
//!
//! Every `find` below is copied verbatim from a fresh FlyByWire development
//! build (`D:\fbw-aircraft\fbw-a380x\out\flybywire-aircraft-a380-842\
//! html_ui`, the same tree `ecam_patches.rs` and `js_bridge.rs::native_ports`
//! already target) and confirmed to occur exactly once in the file it
//! patches (`docs/deep/ecam_bridge.md` records the exact `grep`/`rg`
//! command and count for each).

use crate::deep::api::EcamAlert;
use crate::deep::ecam::codegen;
use crate::deep::ecam::ids;
use crate::js::msfs::SourcePatch;

const EWD: &str = "/Pages/VCockpit/Instruments/A380X/EWD/ewd.js";
const SYSTEMS_HOST: &str = "/Pages/VCockpit/Instruments/A380X/SystemsHost/SystemsHost.js";

/// Both `EWD.js` and `SystemsHost.js` hold their own compiled copy of
/// `EcamAbnormalSensedProcedures` (each bundle inlines the shared TS
/// module it imports it from independently -- confirmed byte-identical at
/// `ewd.js:68967` and `SystemsHost.js:166034`), and `EcamAbnormalProcedures`
/// is a bare alias of it in both (`var EcamAbnormalProcedures =
/// EcamAbnormalSensedProcedures;`), so one `Object.assign` right after this
/// line reaches every consumer in that bundle, including the alias.
const PROC_SPREAD_FIND: &str = "  var EcamAbnormalSensedProcedures = __spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues({}, EcamAbnormalSensedAta212223), EcamAbnormalSensedAta24), EcamAbnormalSensedAta26), EcamAbnormalSensedAta27), EcamAbnormalSensedAta28), EcamAbnormalSensedAta2930), EcamAbnormalSensedAta313233), EcamAbnormalSensedAta34), EcamAbnormalSensedAta353642), EcamAbnormalSensedAta46495256), EcamAbnormalSensedAta70), EcamAbnormalSensedAta80Rest), EcamAbnormalSecondaryFailures);";

/// The end of `SystemsHost.js`'s `EcamInopSys` dict (`var EcamInopSys =
/// {...}`, plain id->text, read only while `FwsCore` resolves an
/// `inopSysAllPhases()` key to display text before publishing it -- not
/// needed in `EWD.js`/`SDv2.js`, neither of which reference `EcamInopSys`
/// at all in this build).
const INOP_END_FIND: &str = "    700300003: \"\\x1B<4mENG 2+3 REVERSERs\"\n  };";

/// The end of `SystemsHost.js`'s `EcamMemos` dict, same reasoning as
/// `INOP_END_FIND` for STATUS-page "INFO" lines (`.status_line(...)`,
/// `EwdAbnormalItem.info`, `FwsCore.ts:5714`).
const MEMOS_END_FIND: &str = "    \"709000001\": \"\\x1B<3mIGNITION\"\n  };";

/// The very first line of `FwsCore.update(_deltaTime)`
/// (`fwsUpdateThrottler` is declared and used only on `FwsCore`, so this
/// text cannot match any other class's `update` method in the same file).
/// Everything this bridge evaluates every tick (trigger, confirm delay,
/// per-line shown/checked state) is installed and stepped from here, once
/// per `FwsCore` instance (there are two, FWS1/FWS2, sharing this compiled
/// text) -- see `deep_ecam_bridge.js`'s file doc comment for why this is
/// early enough (it runs after both `FwsCore`'s and `FwsAbnormalSensed`'s
/// constructors, which is all that matters).
const UPDATE_START_FIND: &str = "      const deltaTime = this.fwsUpdateThrottler.canUpdate(_deltaTime);";

const SHIM_JS: &str = include_str!("deep_ecam_bridge.js");

fn proc_patch(path: &str, replace: String) -> SourcePatch {
    SourcePatch {
        path: path.to_string(),
        find: PROC_SPREAD_FIND.to_string(),
        replace,
        reason: "deep ECAM bridge: merge injected alerts' title/items text into EcamAbnormalSensedProcedures (docs/deep/ecam_bridge.md)".to_string(),
    }
}

/// Builds every `SourcePatch` for one set of registered alerts: one for
/// `EWD.js`'s copy of `EcamAbnormalSensedProcedures`, and four for
/// `SystemsHost.js` (the same procedures merge, `EcamInopSys`, `EcamMemos`,
/// and the `FwsCore.update()` install/step). Empty input still returns all
/// five, each merging in an empty object (`Object.assign(x, {})`) or
/// installing an empty alert array, which is a harmless no-op -- so wiring
/// this in costs nothing before any area has registered an alert.
pub fn source_patches(alerts: &[EcamAlert]) -> Vec<SourcePatch> {
    let assigned = ids::assign(alerts);
    let procedures_merge = codegen::procedures_merge_js(&assigned);
    let inop_merge = codegen::inop_merge_js(&assigned);
    let memos_merge = codegen::info_merge_js(&assigned);
    let alerts_array = codegen::alerts_js_array(&assigned);

    let proc_replace = format!("{PROC_SPREAD_FIND}\n  Object.assign(EcamAbnormalSensedProcedures, {procedures_merge});");

    let inop_replace = format!("{INOP_END_FIND}\n  Object.assign(EcamInopSys, {inop_merge});");

    let memos_replace = format!("{MEMOS_END_FIND}\n  Object.assign(EcamMemos, {memos_merge});");

    // Guarded to run once per FwsCore instance: `SHIM_JS` defines
    // `installDeepEcam` (idempotent to redefine, but there is no reason to
    // pay for it every frame) and `__deepEcamAlerts` is only needed to
    // build the closures `installDeepEcam` returns, so both sit inside the
    // guard; `__deepEcamTick` (per FwsCore instance, since each has its own
    // `ewdAbnormalSensed`/`ewdAbnormal`/`allSuppressableItems`) is stepped
    // every tick after that, guard or not.
    let update_replace = format!(
        "{UPDATE_START_FIND}\n      if (!this.__deepEcamInstalled) {{\n        this.__deepEcamInstalled = true;\n{SHIM_JS}\n        var __deepEcamAlerts = {alerts_array};\n        this.__deepEcamTick = globalThis.installDeepEcam(this, __deepEcamAlerts);\n      }}\n      if (this.__deepEcamTick) {{ this.__deepEcamTick(); }}"
    );

    vec![
        proc_patch(EWD, proc_replace.clone()),
        proc_patch(SYSTEMS_HOST, proc_replace),
        SourcePatch {
            path: SYSTEMS_HOST.to_string(),
            find: INOP_END_FIND.to_string(),
            replace: inop_replace,
            reason: "deep ECAM bridge: merge injected alerts' INOP SYS text into EcamInopSys (docs/deep/ecam_bridge.md)".to_string(),
        },
        SourcePatch {
            path: SYSTEMS_HOST.to_string(),
            find: MEMOS_END_FIND.to_string(),
            replace: memos_replace,
            reason: "deep ECAM bridge: merge injected alerts' STATUS text into EcamMemos (docs/deep/ecam_bridge.md)".to_string(),
        },
        SourcePatch {
            path: SYSTEMS_HOST.to_string(),
            find: UPDATE_START_FIND.to_string(),
            replace: update_replace,
            reason: "deep ECAM bridge: install and step injected alerts' EwdAbnormalItem trigger/line state every FwsCore tick (docs/deep/ecam_bridge.md)".to_string(),
        },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::api::{var, EcamAlert, Level};

    fn sample_alerts() -> Vec<EcamAlert> {
        vec![EcamAlert::new("SAMPLE", 21, "SAMPLE TITLE", Level::Caution, var("A32NX_SAMPLE_FAULT").on()).confirm(1.0)]
    }

    #[test]
    fn five_patches_are_produced_each_matching_its_own_file() {
        let patches = source_patches(&sample_alerts());
        assert_eq!(patches.len(), 5);
        assert_eq!(patches.iter().filter(|p| p.path == EWD).count(), 1);
        assert_eq!(patches.iter().filter(|p| p.path == SYSTEMS_HOST).count(), 4);
    }

    #[test]
    fn every_replace_still_contains_the_original_find_text_verbatim() {
        // A SourcePatch must not lose the anchor text it replaces -- every
        // replace here extends the original line/block rather than
        // rewriting it, so a second, unrelated area's patch (were there
        // one at the same anchor) would still find its own anchor text
        // untouched textually, and the diff stays reviewable.
        for p in source_patches(&sample_alerts()) {
            assert!(p.replace.starts_with(&p.find), "patch for {} must extend, not replace, its anchor", p.path);
        }
    }

    #[test]
    fn empty_input_still_produces_well_formed_no_op_merges() {
        let patches = source_patches(&[]);
        let update_find = UPDATE_START_FIND;
        for p in &patches {
            if p.find == update_find {
                assert!(p.replace.contains("var __deepEcamAlerts = [];"), "an empty alert list must install an empty array: {}", p.replace);
            } else {
                assert!(p.replace.ends_with("Object.assign(EcamAbnormalSensedProcedures, {});") || p.replace.ends_with("Object.assign(EcamInopSys, {});") || p.replace.ends_with("Object.assign(EcamMemos, {});"), "an empty alert list must merge an empty object: {}", p.replace);
            }
        }
    }

    #[test]
    fn the_generated_data_is_embedded_in_the_update_anchor_patch() {
        let patches = source_patches(&sample_alerts());
        let update_patch = patches.iter().find(|p| p.find == UPDATE_START_FIND).expect("the update() patch exists");
        assert!(update_patch.replace.contains("installDeepEcam"));
        assert!(update_patch.replace.contains("__deepEcamInstalled"));
        assert!(!update_patch.replace.contains("'A32NX_SAMPLE_FAULT'"), "the bare name must be L:-prefixed, not used as-is");
        assert!(update_patch.replace.contains("L:A32NX_SAMPLE_FAULT"));
    }

    #[test]
    fn the_title_patch_carries_the_alert_title_in_both_bundles() {
        let patches = source_patches(&sample_alerts());
        for path in [EWD, SYSTEMS_HOST] {
            let p = patches.iter().find(|p| p.path == path && p.find == PROC_SPREAD_FIND).unwrap();
            assert!(p.replace.contains("SAMPLE TITLE"), "{path} must carry the alert's title");
        }
    }
}
