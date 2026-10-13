//! The failures page: every failure the systems and computers know, by ATA
//! chapter, each a button that arms or clears it (failures.rs, the same set
//! `fbw/failure/<id>` exposes).

use std::collections::BTreeMap;
use std::ffi::c_int;

use super::canvas::{palette as p, Action, Canvas};
use crate::xp::FONT_BASIC;

/// ATA chapter names for the failure ids' thousands (FlyByWire's EFB groups
/// failures the same way); also the web Study tab's grouping. See
/// [`ata_of`] for why a deep id's chapter is not simply `id / 1000`.
pub(super) fn chapter(ata: u64) -> &'static str {
    match ata {
        20 => "20 Standard Practices",
        21 => "21 Air Conditioning and Pressurisation",
        22 => "22 Auto Flight",
        23 => "23 Communications",
        24 => "24 Electrical Power",
        25 => "25 Equipment and Furnishings",
        26 => "26 Fire Protection",
        27 => "27 Flight Controls",
        28 => "28 Fuel",
        29 => "29 Hydraulic Power",
        30 => "30 Ice and Rain Protection",
        31 => "31 Indicating and Recording",
        32 => "32 Landing Gear",
        33 => "33 Lights",
        34 => "34 Navigation",
        35 => "35 Oxygen",
        36 => "36 Pneumatic",
        38 => "38 Water and Waste",
        42 => "42 Integrated Modular Avionics",
        44 => "44 Cabin Systems",
        45 => "45 Onboard Maintenance",
        46 => "46 Information Systems",
        49 => "49 Airborne Auxiliary Power",
        52 => "52 Doors",
        53 => "53 Fuselage",
        56 => "56 Windows",
        57 => "57 Wings",
        71 => "71 Power Plant",
        72 => "72 Engine",
        73 => "73 Engine Fuel and Control",
        74 => "74 Ignition",
        75 => "75 Engine Air",
        76 => "76 Engine Controls",
        77 => "77 Engine Indicating",
        78 => "78 Exhaust",
        79 => "79 Engine Oil",
        80 => "80 Starting",
        91 => "91 Charts",
        92 => "92 Electrical Installation",
        70 => "70 Engines",
        _ => "Other",
    }
}

/// The ATA chapter an id belongs to, across both id schemes.
///
/// FlyByWire's own ids and this crate's extra catalogue put the chapter in
/// the thousands (`24_000` is ATA 24). `deep::api` ids are area-coded --
/// `area * 1_000_000 + ata * 1_000 + n`, which `Registry::failure` checks
/// on registration -- so their chapter is the *middle* three digits.
/// Reading a deep id the old way gives `11_026` for a thermal-zone fire
/// failure, which is no chapter at all, and every one of the several
/// thousand deep failures lands in "Other".
pub(crate) fn ata_of(id: u64) -> u64 {
    if id >= 1_000_000 {
        id / 1_000 % 1_000
    } else {
        id / 1_000
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
        chapters.entry(chapter(ata_of(id))).or_default().push(id);
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

#[cfg(test)]
mod tests {
    use super::*;

    /// Every catalogued failure lands in a real ATA chapter.
    ///
    /// The deep catalogue's several thousand failures are area-coded, so
    /// reading their chapter the old way (`id / 1000`) put every one of
    /// them in "Other" -- a Study page with one enormous unnamed group.
    #[test]
    fn every_catalogued_failure_falls_in_a_named_chapter() {
        let ids = crate::failures::all_ids();
        assert!(ids.len() > 4_000, "the deep catalogue should be in here: {}", ids.len());

        let mut other: Vec<u64> = Vec::new();
        for id in &ids {
            if chapter(ata_of(*id)) == "Other" {
                other.push(*id);
            }
        }
        assert!(other.is_empty(), "{} failures have no named chapter, e.g. {:?}", other.len(), &other[..other.len().min(10)]);
    }

    /// The two id schemes put the chapter in different places.
    #[test]
    fn a_deep_ids_chapter_is_its_middle_digits_and_a_legacy_ids_is_its_thousands() {
        // FlyByWire's own: ATA 24, electrical.
        assert_eq!(ata_of(24_000), 24);
        assert_eq!(chapter(ata_of(24_000)), "24 Electrical Power");
        // Deep: area 11 (thermal zones), ATA 26, eighth failure.
        assert_eq!(ata_of(11_026_008), 26);
        assert_eq!(chapter(ata_of(11_026_008)), "26 Fire Protection");
        // Read the old way that id gives 11_026, which is not a chapter.
        assert_eq!(chapter(11_026_008 / 1000), "Other");
    }
}
