//! The Walkaround page: every exterior preflight item (`walkaround.rs`),
//! grouped probes/engines/gear pins/chocks, each a button toggling its own
//! `fbw/walkaround/<id>_toggle` command, lit while installed; "Remove all"/
//! "Install all" act on every item at once. A line at the top names any
//! item whose presence is currently causing a failure (the pitot/static/
//! AoA covers only -- gear pins and chocks have no failure id, see
//! `walkaround.rs`'s module doc for why).

use std::ffi::c_int;

use super::canvas::{palette as p, Action, Canvas};
use crate::xp::FONT_BASIC;

/// One group's heading and the item indices it covers. `pub(super)`: also
/// built into the web Study tab's JSON for this page (`web.rs`), so the two
/// front ends can never drift apart.
pub(super) const GROUPS: [(&str, &[usize]); 4] = [
    ("Probes", &crate::walkaround::GROUP_PROBES),
    ("Engines", &crate::walkaround::GROUP_ENGINES),
    ("Gear pins", &crate::walkaround::GROUP_GEAR_PINS),
    ("Chocks", &crate::walkaround::GROUP_CHOCKS),
];

/// `fbw/walkaround/<id>_toggle`, precomputed as `'static` literals (not
/// `format!`ed per frame into `Action::Command(&'static str)`, which would
/// need to leak a new allocation every redraw): one entry per
/// `crate::walkaround::ITEMS`, same order.
pub(super) const TOGGLE_COMMANDS: [&str; crate::walkaround::N] = [
    "fbw/walkaround/pitot_cover_1_toggle",
    "fbw/walkaround/pitot_cover_2_toggle",
    "fbw/walkaround/pitot_cover_3_toggle",
    "fbw/walkaround/static_covers_toggle",
    "fbw/walkaround/aoa_cover_1_toggle",
    "fbw/walkaround/aoa_cover_2_toggle",
    "fbw/walkaround/aoa_cover_3_toggle",
    "fbw/walkaround/eng_inlet_cover_1_toggle",
    "fbw/walkaround/eng_inlet_cover_2_toggle",
    "fbw/walkaround/eng_inlet_cover_3_toggle",
    "fbw/walkaround/eng_inlet_cover_4_toggle",
    "fbw/walkaround/eng_exhaust_cover_1_toggle",
    "fbw/walkaround/eng_exhaust_cover_2_toggle",
    "fbw/walkaround/eng_exhaust_cover_3_toggle",
    "fbw/walkaround/eng_exhaust_cover_4_toggle",
    "fbw/walkaround/gear_pin_nose_toggle",
    "fbw/walkaround/gear_pin_lwing_toggle",
    "fbw/walkaround/gear_pin_rwing_toggle",
    "fbw/walkaround/gear_pin_lbody_toggle",
    "fbw/walkaround/gear_pin_rbody_toggle",
    "fbw/walkaround/chocks_nose_toggle",
    "fbw/walkaround/chocks_lwing_toggle",
    "fbw/walkaround/chocks_rwing_toggle",
    "fbw/walkaround/chocks_lbody_toggle",
    "fbw/walkaround/chocks_rbody_toggle",
];

/// A short label for an item id ("pitot_cover_1" -> "Pitot cover 1").
pub(super) fn label_for(id: &str) -> String {
    let mut s = id.replace('_', " ");
    if let Some(c) = s.get_mut(0..1) {
        c.make_ascii_uppercase();
    }
    s
}

/// The failure ids a probe cover can cause, with a short cause name, so the
/// summary line can name exactly which covers are currently active
/// failures (`walkaround.rs`'s `failure_levels_from`, mirrored here by id
/// rather than re-deriving it from dataref state, since the ids are the
/// stable, documented interface between the two).
pub(super) const PROBE_FAILURE_IDS: [(u64, &str); 9] = [
    (34_100, "pitot_cover_1"),
    (34_101, "pitot_cover_2"),
    (34_102, "pitot_cover_3"),
    (34_103, "static_covers"),
    (34_104, "static_covers"),
    (34_105, "static_covers"),
    (34_106, "aoa_cover_1"),
    (34_107, "aoa_cover_2"),
    (34_108, "aoa_cover_3"),
];

/// Draw the page; returns how far it scrolls.
pub fn draw(cv: &mut Canvas, scroll: c_int) -> c_int {
    let (l, t, r, b) = cv.clip;
    cv.fill_px(l, t, r, b, p::PERIWINKLE);
    let lh = cv.line_h;

    let installed_count = crate::walkaround::ITEMS
        .iter()
        .filter(|id| cv.value(&format!("fbw/walkaround/{id}")) == Some(1.))
        .count();
    let mut causing: Vec<&str> = PROBE_FAILURE_IDS.iter().filter(|(id, _)| crate::failures::is_active(*id)).map(|(_, name)| *name).collect();
    causing.sort_unstable();
    causing.dedup();

    let band_b = t - (2 * lh + 20);
    cv.fill_px(l, t, r, band_b, p::BAND);
    cv.text_px(l + 10, t - lh - 2, p::INK_LIGHT, FONT_BASIC, &format!("{installed_count}/{} items installed", crate::walkaround::N));
    let causing_line = if causing.is_empty() {
        "No item is currently causing a failure.".to_owned()
    } else {
        format!("Causing a failure now: {}", causing.join(", "))
    };
    cv.text_px(l + 10, t - 2 * lh - 6, p::INK_LIGHT, FONT_BASIC, &causing_line);

    let button_w = 22 * cv.char_w;
    let row_h = lh + 10;
    cv.button_px(l + 10, band_b - 6, l + 10 + button_w, band_b - 6 - row_h + 4, "Remove all", false, Action::Command("fbw/walkaround/remove_all"));
    cv.button_px(l + 10 + button_w + 8, band_b - 6, l + 10 + 2 * button_w + 8, band_b - 6 - row_h + 4, "Install all", false, Action::Command("fbw/walkaround/install_all"));

    let columns = 3;
    let gap = 8;
    let col_w = (r - l - 20 - gap * (columns - 1)) / columns;
    let top_limit = band_b - row_h - 12;
    let mut y = top_limit + scroll;
    let mut height = 0;
    for (name, items) in GROUPS {
        if y - lh <= top_limit && y > b {
            cv.text_px(l + 10, y - lh, p::INK, FONT_BASIC, name);
        }
        y -= lh + 6;
        height += lh + 6;
        for (i, &idx) in items.iter().enumerate() {
            let col = (i % columns as usize) as c_int;
            let x = l + 10 + col * (col_w + gap);
            if y <= top_limit && y - row_h > b {
                let id = crate::walkaround::ITEMS[idx];
                let installed = cv.value(&format!("fbw/walkaround/{id}")) == Some(1.);
                cv.button_px(x, y, x + col_w, y - row_h + 2, &label_for(id), installed, Action::Command(TOGGLE_COMMANDS[idx]));
            }
            if col == columns - 1 || i + 1 == items.len() {
                y -= row_h;
                height += row_h;
            }
        }
        y -= 6;
        height += 6;
    }
    (height - (top_limit - b)).max(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Guards `TOGGLE_COMMANDS` against drifting out of sync with
    /// `crate::walkaround::ITEMS` (it is a literal list, not derived from
    /// it, because `Action::Command` needs a `'static str` this module
    /// cannot build per-frame without leaking).
    #[test]
    fn toggle_commands_match_the_items_list_in_order() {
        for (i, id) in crate::walkaround::ITEMS.iter().enumerate() {
            assert_eq!(TOGGLE_COMMANDS[i], format!("fbw/walkaround/{id}_toggle"));
        }
    }

    #[test]
    fn every_group_covers_every_item_exactly_once() {
        let mut seen = [0u32; crate::walkaround::N];
        for (_, items) in GROUPS {
            for &i in items {
                seen[i] += 1;
            }
        }
        assert!(seen.iter().all(|&c| c == 1));
    }
}
