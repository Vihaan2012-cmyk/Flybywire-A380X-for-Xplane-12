//! The failures page: every failure the systems and computers know, by ATA
//! chapter, each a button that arms or clears it (failures.rs, the same set
//! `fbw/failure/<id>` exposes).

use std::collections::BTreeMap;
use std::ffi::c_int;

use super::canvas::{palette as p, Action, Canvas};
use crate::xp::FONT_BASIC;

/// ATA chapter names for the failure ids' thousands (FlyByWire's EFB groups
/// failures the same way, by `id / 1000`); also the web Study tab's grouping.
pub(super) fn chapter(ata: u64) -> &'static str {
    match ata {
        21 => "21 Air Conditioning and Pressurisation",
        22 => "22 Auto Flight",
        24 => "24 Electrical Power",
        26 => "26 Fire Protection",
        27 => "27 Flight Controls",
        28 => "28 Fuel",
        29 => "29 Hydraulic Power",
        31 => "31 Indicating and Recording",
        32 => "32 Landing Gear",
        34 => "34 Navigation",
        36 => "36 Pneumatic",
        49 => "49 Airborne Auxiliary Power",
        70..=80 => "70 Engines",
        _ => "Other",
    }
}

/// Draw the page; returns how far it scrolls.
pub fn draw(cv: &mut Canvas, scroll: c_int) -> c_int {
    let (l, t, r, b) = cv.clip;
    cv.fill_px(l, t, r, b, p::PERIWINKLE);
    let lh = cv.line_h;
    let active = crate::failures::active_ids();
    let ids = crate::failures::all_ids();
    let mut chapters: BTreeMap<&str, Vec<u64>> = BTreeMap::new();
    for id in ids {
        chapters.entry(chapter(id / 1000)).or_default().push(id);
    }

    // Header band: how many are armed, and a button clearing them all.
    let band_b = t - (lh + 14);
    cv.fill_px(l, t, r, band_b, p::BAND);
    let summary = format!("{} failures armed", active.len());
    cv.text_px(l + 10, t - lh - 2, p::INK_LIGHT, FONT_BASIC, &summary);

    let columns = 3;
    let gap = 8;
    let col_w = (r - l - 20 - gap * (columns - 1)) / columns;
    let row_h = lh + 10;
    let mut y = band_b - 8 + scroll;
    let top_limit = band_b - 4;
    let mut height = 0;
    for (name, list) in &chapters {
        if y - lh <= top_limit && y > b {
            cv.text_px(l + 10, y - lh, p::INK, FONT_BASIC, name);
        }
        y -= lh + 6;
        height += lh + 6;
        for (i, id) in list.iter().enumerate() {
            let col = (i % columns as usize) as c_int;
            let x = l + 10 + col * (col_w + gap);
            if y <= top_limit && y - row_h > b {
                let armed = active.contains(id);
                let label = format!("{} {}", id, crate::failures::any_failure_name(*id));
                cv.button_px(x, y, x + col_w, y - row_h + 2, &label, armed, Action::ToggleFailure(*id));
            }
            if col == columns - 1 || i + 1 == list.len() {
                y -= row_h;
                height += row_h;
            }
        }
        y -= 6;
        height += 6;
    }
    (height - (band_b - b)).max(0)
}
