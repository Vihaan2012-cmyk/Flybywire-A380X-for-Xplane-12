//! The operator Minimum Equipment List, read from the user's own parsed copy.
//!
//! An A380 MEL is Airbus/operator copyright, so none is bundled: the user
//! runs `tools/parse_mel.py` over their own PDF and puts the result at
//! `<X-Plane>/Output/preferences/fbw_a380x_mel.json`. Without that file the
//! MEL page says so and deferral falls back to `mel.rs`'s generic per-failure
//! category.
//!
//! Each sub-item carries what the MEL table gives: repair-interval category
//! (the MEL's own preamble: B 3, C 10, D 120 calendar days; A as the item
//! states), number installed/required, placard, the dispatch conditions
//! ("(o)" operational, "(m)" maintenance) and the operational procedure text.
//!
//! Matching a failure to MEL items is a suggestion for the user to choose
//! from, never automatic: same ATA chapter (a failure id's thousands, as
//! `study/web.rs`'s `failures_json` reads it), ranked by the words the
//! failure's name shares with the item's title.

use std::path::PathBuf;
use std::sync::OnceLock;

use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct SubItem {
    pub id: String,
    pub category: Option<String>,
    pub installed: Option<String>,
    pub required: Option<String>,
    pub placard: Option<String>,
    pub repair_interval_days: Option<f64>,
    #[serde(default)]
    pub conditions: String,
    #[serde(default)]
    pub references: Vec<String>,
    #[serde(default)]
    pub ops_procedure_required: bool,
    #[serde(default)]
    pub maintenance_procedure_required: bool,
    #[serde(default)]
    pub amm: Vec<String>,
    pub ops_procedure: Option<String>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Item {
    pub ata: String,
    pub title: String,
    pub subitems: Vec<SubItem>,
}

#[derive(Deserialize)]
struct MelFile {
    items: Vec<Item>,
}

/// X-Plane's folder when this runs outside X-Plane (the XPHFBW app, which
/// is told it on its command line or finds it itself).
static XPLANE_ROOT: OnceLock<PathBuf> = OnceLock::new();

pub fn set_xplane_root(root: PathBuf) {
    let _ = XPLANE_ROOT.set(root);
}

pub fn path() -> Option<PathBuf> {
    XPLANE_ROOT
        .get()
        .cloned()
        .or_else(crate::xp::system_path)
        .map(|root| root.join("Output").join("preferences").join("fbw_a380x_mel.json"))
}

pub fn parse(text: &str) -> Result<Vec<Item>, String> {
    serde_json::from_str::<MelFile>(text).map(|f| f.items).map_err(|e| e.to_string())
}

static CATALOG: OnceLock<Option<Vec<Item>>> = OnceLock::new();

/// The loaded MEL, read once; `None` when the user has not provided one.
pub fn catalog() -> Option<&'static [Item]> {
    CATALOG
        .get_or_init(|| {
            let path = path()?;
            let text = std::fs::read_to_string(&path).ok()?;
            match parse(&text) {
                Ok(items) => Some(items),
                Err(e) => {
                    crate::log(&format!("MEL at {} could not be read: {e}", path.display()));
                    None
                }
            }
        })
        .as_deref()
}

pub fn find_in<'a>(items: &'a [Item], sub_id: &str) -> Option<(&'a Item, &'a SubItem)> {
    items.iter().find_map(|it| it.subitems.iter().find(|s| s.id == sub_id).map(|s| (it, s)))
}

pub fn sub_item(sub_id: &str) -> Option<(&'static Item, &'static SubItem)> {
    find_in(catalog()?, sub_id)
}

/// The repair interval in hours for a sub-item, when the MEL gives one in
/// days (B/C/D); category A's interval is in its own conditions text.
pub fn interval_hours(sub: &SubItem) -> Option<f64> {
    sub.repair_interval_days.map(|d| d * 24.0)
}

/// Operator MEL items (their ATA reference, every sub-item of it) and the
/// catalogued failures that are the same equipment, one per unit: deferring
/// the item under the MEL means that unit really is lost in the simulation.
/// Only exact equipment matches are listed; an item whose loss the
/// simulation cannot produce (or only a broader or different loss of) is
/// left out, and the MEL page does not offer it.
pub const MEL_FAILURES: &[(&str, &[u64])] = &[
    // 21 Air conditioning
    ("21-21-01", &[21_001, 21_002, 21_003, 21_004]),
    ("21-28-02", &[21_009]),
    ("21-28-06", &[21_007]),
    ("21-60-04", &[21_005, 21_006]),
    ("21-60-11", &[21_011]),
    // 22 Auto flight: the AFS control panel with each FCU channel.
    ("22-80-02", &[22_002]),
    ("22-80-03", &[22_003]),
    // 24 Electrical power
    ("24-21-01", &[24_020, 24_021, 24_022, 24_023]),
    ("24-23-01", &[24_030, 24_031]),
    ("24-32-02", &[24_003]),
    // 26 Fire protection: detection loops
    ("26-10-01", &[26_017, 26_018]),
    ("26-10-02", &[26_007, 26_008, 26_009, 26_010, 26_011, 26_012, 26_013, 26_014]),
    ("26-10-03", &[26_015, 26_016]),
    // 27 Flight controls
    ("27-93-01", &[27_000]),
    ("27-93-02", &[27_001]),
    ("27-93-03", &[27_002]),
    ("27-94-01", &[27_003]),
    ("27-94-02", &[27_004]),
    ("27-94-03", &[27_005]),
    ("27-96-01", &[27_007]),
    ("27-99-01", &[27_007]),
    // 28 Fuel: feed pumps (pump A the main, pump B the standby), jettison,
    // and the trim tank's automatic transfer (its transfer pump).
    ("28-26-06", &[28_000]),
    ("28-26-07", &[28_001]),
    ("28-26-10", &[28_002]),
    ("28-26-13", &[28_003]),
    ("28-26-12", &[28_004]),
    ("28-26-11", &[28_005]),
    ("28-26-14", &[28_006]),
    ("28-26-15", &[28_007]),
    ("28-25-03", &[28_011]),
    ("28-27-01", &[28_008]),
    ("28-25-01", &[28_120, 28_121]),
    ("28-25-02", &[28_122]),
    ("28-25-04", &[28_123, 28_126]),
    ("28-25-05", &[28_127, 28_130]),
    ("28-25-06", &[28_124, 28_125]),
    ("28-25-07", &[28_128, 28_129]),
    ("28-25-08", &[28_131, 28_132]),
    ("28-25-09", &[28_133, 28_134]),
    ("28-25-10", &[28_135, 28_136]),
    ("28-25-11", &[28_137, 28_138]),
    ("28-25-12", &[28_139, 28_140]),
    ("28-25-13", &[28_141, 28_142]),
    ("28-25-14", &[28_143]),
    ("28-25-19", &[28_144, 28_145]),
    ("28-26-01", &[28_100, 28_101]),
    // 29 Hydraulic power: the electric pumps A/B of each system.
    ("29-21-01", &[29_006, 29_007]),
    ("29-21-02", &[29_008, 29_009]),
    // 30 Ice and rain: the ADIRUs' probe heating.
    ("34-11-05", &[30_000, 30_001, 30_002]),
    // 34 Navigation
    ("34-11-01", &[34_100, 34_101, 34_102]),
    ("34-11-02", &[34_103, 34_104, 34_105]),
    ("34-42-01", &[34_000, 34_001, 34_002]),
    // 36 Pneumatic
    ("36-11-05", &[36_008, 36_009, 36_010, 36_011]),
    // The engine bleed system: its pressure regulating (shut-off) valve.
    ("36-11-01", &[36_012, 36_013, 36_014, 36_015]),
    ("36-13-01", &[36_017]),
    ("36-13-02", &[36_016, 36_018]),
    // 21 Air conditioning: each pack's two flow control valves.
    ("21-50-02", &[21_050, 21_051]),
    ("21-50-03", &[21_052, 21_053]),
    // 49 APU: an APU that cannot be started.
    ("49-10-01", &[49_002]),
    ("49-20-01", &[28_110]),
    // 74/78/80 Engines
    ("74-31-01", &[74_000, 74_001, 74_002, 74_003]),
    ("74-31-02", &[74_000, 74_001, 74_002, 74_003]),
    ("78-30-04", &[78_000, 78_001, 78_002, 78_003]),
    ("80-11-01", &[80_000, 80_001, 80_002, 80_003]),
];

/// The failures (one per unit) an MEL sub-item or item reference covers;
/// empty when the simulation does not model that equipment.
pub fn failures_for(mel_ref: &str) -> &'static [u64] {
    let item = mel_ref.get(..8).unwrap_or(mel_ref);
    MEL_FAILURES.iter().find(|(r, _)| *r == item).map_or(&[], |(_, f)| *f)
}

/// MEL sub-items covering failure `failure_id`, from the table above.
pub fn candidates_in<'a>(items: &'a [Item], failure_id: u64, limit: usize) -> Vec<(&'a Item, &'a SubItem)> {
    items
        .iter()
        .filter(|it| failures_for(&it.ata).contains(&failure_id))
        .flat_map(|it| it.subitems.iter().map(move |s| (it, s)))
        .take(limit)
        .collect()
}

pub fn candidates(failure_id: u64, limit: usize) -> Vec<(&'static Item, &'static SubItem)> {
    catalog().map(|items| candidates_in(items, failure_id, limit)).unwrap_or_default()
}

/// Free-text search over ATA reference, title and conditions.
pub fn search_in<'a>(items: &'a [Item], query: &str, limit: usize) -> Vec<(&'a Item, &'a SubItem)> {
    let q = query.trim().to_ascii_lowercase();
    if q.is_empty() {
        return Vec::new();
    }
    items
        .iter()
        .flat_map(|it| it.subitems.iter().map(move |s| (it, s)))
        .filter(|(it, s)| {
            s.id.to_ascii_lowercase().starts_with(&q)
                || it.title.to_ascii_lowercase().contains(&q)
                || s.conditions.to_ascii_lowercase().contains(&q)
        })
        .take(limit)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    // Two items shaped exactly as tools/parse_mel.py writes them (values
    // from the A380 MEL's 49-10 APU pages).
    const SAMPLE: &str = r#"{"items":[
      {"ata":"49-10-01","title":"APU","subitems":[
        {"id":"49-10-01A","category":"C","installed":"1","required":"0","placard":"Yes","repair_interval_days":10,
         "conditions":"(o) May be inoperative provided that the APU MASTER SW pb-sw is set to Off.",
         "references":["(o) Refer to OpsProc 49-10-01A"],"ops_procedure_required":true,
         "maintenance_procedure_required":false,"amm":[],"ops_procedure":"APU MASTER SW ... OFF"}]},
      {"ata":"49-10-02","title":"APU Flap","subitems":[
        {"id":"49-10-02A","category":"C","installed":"1","required":"0","placard":"Yes","repair_interval_days":10,
         "conditions":"(o) May be inoperative in open position.","references":[],"ops_procedure_required":true,
         "maintenance_procedure_required":false,"amm":[],"ops_procedure":null}]},
      {"ata":"21-01-01","title":"PACK Pb-Sw FAULT Light","subitems":[
        {"id":"21-01-01A","category":"A","installed":"2","required":"0","placard":"Yes","repair_interval_days":null,
         "conditions":"May be inoperative.","references":[],"ops_procedure_required":false,
         "maintenance_procedure_required":false,"amm":[],"ops_procedure":null}]}
    ]}"#;

    #[test]
    fn candidates_come_only_from_the_explicit_table() {
        let items = parse(SAMPLE).unwrap();
        // 49-10-01 (APU) covers the APU starter failure; the APU flap and the
        // PACK light cover nothing the simulation models.
        let c = candidates_in(&items, 49_002, 5);
        assert!(!c.is_empty() && c.iter().all(|(it, _)| it.ata == "49-10-01"));
        assert!(candidates_in(&items, 49_003, 5).is_empty(), "no loose title match");
        assert_eq!(failures_for("49-10-01A"), &[49_002]);
        assert!(failures_for("49-10-02A").is_empty());
    }

    #[test]
    fn day_categories_convert_and_category_a_has_no_fixed_interval() {
        let items = parse(SAMPLE).unwrap();
        assert_eq!(interval_hours(find_in(&items, "49-10-01A").unwrap().1), Some(240.0));
        assert_eq!(interval_hours(find_in(&items, "21-01-01A").unwrap().1), None);
    }

    #[test]
    fn search_matches_reference_title_and_conditions() {
        let items = parse(SAMPLE).unwrap();
        assert_eq!(search_in(&items, "49-10-02", 10).len(), 1);
        assert_eq!(search_in(&items, "open position", 10)[0].1.id, "49-10-02A");
    }
}
