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
/// The STATUS page (`panel.cfg` VCockpit04 `htmlgauge01`,
/// `A380X/SDv2/sdv2.html`). This, not `SystemsHost.js`, is where an INOP
/// SYS or STATUS "INFO" id becomes text: `FwsCore` publishes only the
/// *keys* over the bus (`inopSysAllPhasesKeys`/`informationKeys`,
/// `SystemsHost.js:179593-179610`) and `sdv2.js` resolves each one against
/// its own `EcamInopSys`/`EcamInfos` (`sdv2.js:68632`, `:68650`, `:68653`,
/// `:68677`). `SystemsHost.js` carries its own copies of both dicts but
/// never reads either (`grep -c 'EcamInopSys' SystemsHost.js` = 1, the
/// declaration), so merging into them there would display nothing -- see
/// this module's `a_status_id_is_merged_where_it_is_actually_resolved`
/// test and the note in `docs/deep/ecam_bridge.md`'s section 3, which
/// this contradicts.
const SDV2: &str = "/Pages/VCockpit/Instruments/A380X/SDv2/sdv2.js";

/// Both `EWD.js` and `SystemsHost.js` hold their own compiled copy of
/// `EcamAbnormalSensedProcedures` (each bundle inlines the shared TS
/// module it imports it from independently -- confirmed byte-identical at
/// `ewd.js:68967` and `SystemsHost.js:166034`), and `EcamAbnormalProcedures`
/// is a bare alias of it in both (`var EcamAbnormalProcedures =
/// EcamAbnormalSensedProcedures;`), so one `Object.assign` right after this
/// line reaches every consumer in that bundle, including the alias.
const PROC_SPREAD_FIND: &str = "  var EcamAbnormalSensedProcedures = __spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues({}, EcamAbnormalSensedAta212223), EcamAbnormalSensedAta24), EcamAbnormalSensedAta26), EcamAbnormalSensedAta27), EcamAbnormalSensedAta28), EcamAbnormalSensedAta2930), EcamAbnormalSensedAta313233), EcamAbnormalSensedAta34), EcamAbnormalSensedAta353642), EcamAbnormalSensedAta46495256), EcamAbnormalSensedAta70), EcamAbnormalSensedAta80Rest), EcamAbnormalSecondaryFailures);";

/// The end of `sdv2.js`'s `EcamInopSys` dict (`var EcamInopSys = {...}`,
/// plain id->text): what the STATUS page's INOP SYS columns resolve each
/// key the FWS publishes against (`sdv2.js:68650`/`:68653`/`:68677`).
/// Occurs exactly once in `sdv2.js` (and once in `SystemsHost.js`, whose
/// copy nothing reads -- see [`SDV2`]).
const INOP_END_FIND: &str = "    700300003: \"\\x1B<4mENG 2+3 REVERSERs\"\n  };";

/// The end of `sdv2.js`'s `EcamInfos` dict: what the STATUS page's "INFO"
/// block resolves each `informationKeys` entry against (`sdv2.js:68632`),
/// which is where an alert's `.status_line(...)` (`EwdAbnormalItem.info`)
/// ends up. **Not** `EcamMemos`: that dict is the EWD/PFD memo list
/// (`ewd.js:69005`, `pfd.js`), a different thing entirely, and `sdv2.js`
/// does not reference it at all.
const INFOS_END_FIND: &str = "    800200005: \"\\x1B<3mNO BRAKED PIVOT TURN\"\n  };";

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

/// Builds every `SourcePatch` for one set of registered alerts: the
/// procedures merge for each of the two bundles that hold their own copy
/// of `EcamAbnormalSensedProcedures` (`EWD.js` renders the procedure,
/// `SystemsHost.js` reads `items[i].sensed` while masking
/// `whichItemsChecked`), the STATUS page's two text dicts in `sdv2.js`
/// (`EcamInopSys`, `EcamInfos`), and the `FwsCore.update()` install/step
/// in `SystemsHost.js`. Empty input still returns all five, each merging
/// in an empty object (`Object.assign(x, {})`) or installing an empty
/// alert array, which is a harmless no-op -- so wiring this in costs
/// nothing before any area has registered an alert.
pub fn source_patches(alerts: &[EcamAlert]) -> Vec<SourcePatch> {
    let assigned = ids::assign(alerts);
    let procedures_merge = codegen::procedures_merge_js(&assigned);
    let inop_merge = codegen::inop_merge_js(&assigned);
    let infos_merge = codegen::info_merge_js(&assigned);
    let alerts_array = codegen::alerts_js_array(&assigned);

    let proc_replace = format!("{PROC_SPREAD_FIND}\n  Object.assign(EcamAbnormalSensedProcedures, {procedures_merge});");

    let inop_replace = format!("{INOP_END_FIND}\n  Object.assign(EcamInopSys, {inop_merge});");

    let infos_replace = format!("{INFOS_END_FIND}\n  Object.assign(EcamInfos, {infos_merge});");

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
            path: SDV2.to_string(),
            find: INOP_END_FIND.to_string(),
            replace: inop_replace,
            reason: "deep ECAM bridge: merge injected alerts' INOP SYS text into the STATUS page's EcamInopSys (docs/deep/ecam_bridge.md)".to_string(),
        },
        SourcePatch {
            path: SDV2.to_string(),
            find: INFOS_END_FIND.to_string(),
            replace: infos_replace,
            reason: "deep ECAM bridge: merge injected alerts' STATUS INFO text into the STATUS page's EcamInfos (docs/deep/ecam_bridge.md)".to_string(),
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
        assert_eq!(patches.iter().filter(|p| p.path == SYSTEMS_HOST).count(), 2);
        assert_eq!(patches.iter().filter(|p| p.path == SDV2).count(), 2);
    }

    /// The development build every `find` above was copied from, the same
    /// tree `ecam_patches.rs` targets. Absent on a machine that has not
    /// built FlyByWire, which is why the test below skips rather than
    /// fails there (`display/tests.rs`'s `have_package` does the same).
    const BUILT_HTML_UI: &str = r"D:\fbw-aircraft\fbw-a380x\out\flybywire-aircraft-a380-842\html_ui";

    #[test]
    fn every_anchor_occurs_exactly_once_in_the_file_it_patches() {
        // `js/msfs/mod.rs`'s `read_file` applies a patch only when its
        // `find` matches exactly once, and logs an error and runs the file
        // unchanged otherwise -- a silent no-op on the aircraft. Checking
        // it here is the difference between finding that out in a test and
        // finding it out by an alert never appearing.
        let root = std::path::Path::new(BUILT_HTML_UI);
        if !root.join("Pages/VCockpit/Instruments/A380X/SystemsHost/SystemsHost.js").is_file() {
            return;
        }
        for p in source_patches(&sample_alerts()) {
            let file = root.join(p.path.trim_start_matches('/'));
            let text = std::fs::read_to_string(&file).unwrap_or_else(|e| panic!("{}: {e}", file.display()));
            assert_eq!(text.matches(p.find.as_str()).count(), 1, "{} ({}): anchor must occur exactly once", p.path, p.reason);
        }
    }

    #[test]
    fn a_status_id_is_merged_where_it_is_actually_resolved() {
        // `FwsCore` publishes INOP SYS and STATUS "INFO" as bare ids and
        // the STATUS page (sdv2.js) is the only bundle in this build that
        // turns one into text -- `SystemsHost.js` holds its own copies of
        // `EcamInopSys`/`EcamInfos` and never reads either. Merging into
        // the wrong bundle costs nothing at load and shows nothing on the
        // aircraft, which is exactly the kind of silence a test has to
        // catch.
        let patches = source_patches(&sample_alerts());
        let inop = patches.iter().find(|p| p.find == INOP_END_FIND).expect("the INOP SYS merge exists");
        assert_eq!(inop.path, SDV2);
        assert!(inop.replace.contains("Object.assign(EcamInopSys,"));
        let infos = patches.iter().find(|p| p.find == INFOS_END_FIND).expect("the STATUS INFO merge exists");
        assert_eq!(infos.path, SDV2);
        assert!(infos.replace.contains("Object.assign(EcamInfos,"));
        // EcamMemos is the EWD/PFD memo list, not the STATUS page: nothing
        // this bridge emits may touch it.
        assert!(patches.iter().all(|p| !p.replace.contains("EcamMemos")), "the STATUS lines are EcamInfos, not EcamMemos");
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
                assert!(p.replace.ends_with("Object.assign(EcamAbnormalSensedProcedures, {});") || p.replace.ends_with("Object.assign(EcamInopSys, {});") || p.replace.ends_with("Object.assign(EcamInfos, {});"), "an empty alert list must merge an empty object: {}", p.replace);
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
