use crate::deep::api::EcamAlert;
use crate::deep::ecam::codegen;
use crate::deep::ecam::ids;
use crate::source_patch::SourcePatch;

const EWD: &str = "/Pages/VCockpit/Instruments/A380X/EWD/ewd.js";
const SYSTEMS_HOST: &str = "/Pages/VCockpit/Instruments/A380X/SystemsHost/SystemsHost.js";
const SDV2: &str = "/Pages/VCockpit/Instruments/A380X/SDv2/sdv2.js";

const PROC_SPREAD_FIND: &str = "  var EcamAbnormalSensedProcedures = __spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues(__spreadValues({}, EcamAbnormalSensedAta212223), EcamAbnormalSensedAta24), EcamAbnormalSensedAta26), EcamAbnormalSensedAta27), EcamAbnormalSensedAta28), EcamAbnormalSensedAta2930), EcamAbnormalSensedAta313233), EcamAbnormalSensedAta34), EcamAbnormalSensedAta353642), EcamAbnormalSensedAta46495256), EcamAbnormalSensedAta70), EcamAbnormalSensedAta80Rest), EcamAbnormalSecondaryFailures);";

const INOP_END_FIND: &str = "    700300003: \"\\x1B<4mENG 2+3 REVERSERs\"\n  };";

const INFOS_END_FIND: &str = "    800200005: \"\\x1B<3mNO BRAKED PIVOT TURN\"\n  };";

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

pub fn source_patches(alerts: &[EcamAlert]) -> Vec<SourcePatch> {
    source_patches_with(alerts, &crate::deep::ecam::fbw::wirings())
}

pub fn source_patches_with(alerts: &[EcamAlert], fbw: &[crate::deep::ecam::fbw::FbwProc]) -> Vec<SourcePatch> {
    let assigned = ids::assign(alerts);
    let procedures_merge = codegen::procedures_merge_js(&assigned);
    let inop_merge = codegen::inop_merge_js(&assigned);
    let infos_merge = codegen::info_merge_js(&assigned);
    let alerts_array = codegen::alerts_js_array(&assigned);
    let fbw_array = crate::deep::ecam::fbw_codegen::fbw_alerts_js_array(fbw);

    let proc_replace = format!("{PROC_SPREAD_FIND}\n  Object.assign(EcamAbnormalSensedProcedures, {procedures_merge});");

    let inop_replace = format!("{INOP_END_FIND}\n  Object.assign(EcamInopSys, {inop_merge});");

    let infos_replace = format!("{INFOS_END_FIND}\n  Object.assign(EcamInfos, {infos_merge});");

    let update_replace = format!(
        "{UPDATE_START_FIND}\n      if (!this.__deepEcamInstalled) {{\n        this.__deepEcamInstalled = true;\n{SHIM_JS}\n        var __deepEcamAlerts = {alerts_array};\n        this.__deepEcamTick = globalThis.installDeepEcam(this, __deepEcamAlerts);\n        var __deepFbwAlerts = {fbw_array};\n        this.__deepFbwTick = globalThis.installDeepEcamFbw(this, __deepFbwAlerts, EcamAbnormalProcedures);\n      }}\n      if (this.__deepEcamTick) {{ this.__deepEcamTick(); }}\n      if (this.__deepFbwTick) {{ this.__deepFbwTick(); }}"
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

    const BUILT_HTML_UI: &str = r"D:\fbw-aircraft\fbw-a380x\out\flybywire-aircraft-a380-842\html_ui";

    #[test]
    fn every_anchor_occurs_exactly_once_in_the_file_it_patches() {
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
        let patches = source_patches(&sample_alerts());
        let inop = patches.iter().find(|p| p.find == INOP_END_FIND).expect("the INOP SYS merge exists");
        assert_eq!(inop.path, SDV2);
        assert!(inop.replace.contains("Object.assign(EcamInopSys,"));
        let infos = patches.iter().find(|p| p.find == INFOS_END_FIND).expect("the STATUS INFO merge exists");
        assert_eq!(infos.path, SDV2);
        assert!(infos.replace.contains("Object.assign(EcamInfos,"));
        assert!(patches.iter().all(|p| !p.replace.contains("EcamMemos")), "the STATUS lines are EcamInfos, not EcamMemos");
    }

    #[test]
    fn every_replace_still_contains_the_original_find_text_verbatim() {
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
