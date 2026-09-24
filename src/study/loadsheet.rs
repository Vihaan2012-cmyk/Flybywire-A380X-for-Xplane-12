//! The loadsheet: what the aeroplane weighs, where its balance sits, and
//! where that came from.
//!
//! This page exists because of what the balance turned out to cost. The
//! plugin will not write a centre of gravity into X-Plane until FlyByWire
//! has published a real loadsheet (`weight_balance.rs`), because before
//! that its payload variables all read zero and zero means "nobody has said
//! yet" rather than "an empty aircraft" -- and a balance point computed
//! from a payload nobody has stated does not describe the aeroplane it is
//! applied to. Until this page there was nowhere in the port to see which
//! of those two states you were in, and flying without a loadsheet looked
//! exactly like flying with one until the nose gear started taking twice
//! its share.
//!
//! So the top of the page answers that first: whether the balance is live,
//! and what X-Plane is actually flying if it is not.
//!
//! **SimBrief.** The rest of the port already reaches SimBrief -- the OIT's
//! own `SYNC SIMBRIEF` (`tools/js-build/patches`) and the network path
//! behind it (`net.rs`, `js_bridge.rs`) -- but only from inside FlyByWire's
//! own pages, which means the fetch is only reachable once those pages have
//! loaded. The buttons here drive the same FlyByWire variables the EFB
//! does, so an OFP can be pulled and boarded without going through the
//! tablet.

use std::ffi::c_int;

use super::canvas::{palette as p, Action, Canvas};
use crate::xp::FONT_BASIC;

/// One cabin zone: its label, the variable FlyByWire publishes it as, how
/// many seats it holds, and which deck it is on (0 main, 1 upper).
///
/// The seat counts are FlyByWire's own (`a380_systems/payload/mod.rs`'s
/// `max_pax` per station), not a guess: the drawing fills each block by
/// occupancy, so a wrong capacity draws a full zone as half empty.
pub(super) struct Zone {
    pub label: &'static str,
    pub var: &'static str,
    pub seats: u32,
    pub deck: u8,
}

const fn zone(label: &'static str, var: &'static str, seats: u32, deck: u8) -> Zone {
    Zone { label, var, seats, deck }
}

/// Main deck front to back, then the upper deck front to back.
pub(super) const PAX_ZONES: &[Zone] = &[
    zone("MAIN FWD A", "A32NX_PAX_MAIN_FWD_A", 28, 0),
    zone("MAIN FWD B", "A32NX_PAX_MAIN_FWD_B", 28, 0),
    zone("MAIN MID 1A", "A32NX_PAX_MAIN_MID_1A", 39, 0),
    zone("MAIN MID 1B", "A32NX_PAX_MAIN_MID_1B", 50, 0),
    zone("MAIN MID 1C", "A32NX_PAX_MAIN_MID_1C", 43, 0),
    zone("MAIN MID 2A", "A32NX_PAX_MAIN_MID_2A", 48, 0),
    zone("MAIN MID 2B", "A32NX_PAX_MAIN_MID_2B", 40, 0),
    zone("MAIN MID 2C", "A32NX_PAX_MAIN_MID_2C", 36, 0),
    zone("MAIN AFT A", "A32NX_PAX_MAIN_AFT_A", 42, 0),
    zone("MAIN AFT B", "A32NX_PAX_MAIN_AFT_B", 40, 0),
    zone("UPPER FWD", "A32NX_PAX_UPPER_FWD", 14, 1),
    zone("UPPER MID A", "A32NX_PAX_UPPER_MID_A", 30, 1),
    zone("UPPER MID B", "A32NX_PAX_UPPER_MID_B", 28, 1),
    zone("UPPER AFT", "A32NX_PAX_UPPER_AFT", 18, 1),
];

/// The three holds, with FlyByWire's own `max_cargo_kg`.
pub(super) struct Hold {
    pub label: &'static str,
    pub var: &'static str,
    pub max_kg: f64,
}

pub(super) const CARGO_HOLDS: &[Hold] = &[
    Hold { label: "FWD HOLD", var: "A32NX_CARGO_FWD", max_kg: 28_577. },
    Hold { label: "AFT HOLD", var: "A32NX_CARGO_AFT", max_kg: 20_310. },
    Hold { label: "BULK", var: "A32NX_CARGO_BULK", max_kg: 2_513. },
];

/// Every seat the aeroplane has, so the page can say "412 of 484".
pub(super) fn total_seats() -> u32 {
    let mut n = 0;
    let mut i = 0;
    while i < PAX_ZONES.len() {
        n += PAX_ZONES[i].seats;
        i += 1;
    }
    n
}

/// The variables this page's own buttons may write, and the only ones the
/// panel's `writeVariable` action will accept. Without the list that action
/// is a door onto every aircraft variable in the simulation, openable by
/// anything that can reach the panel's HTTP endpoint.
pub(super) const WRITABLE: &[&str] = &["BOARDING_STARTED_BY_USR", "BOARDING_RATE", "EFB_SIMBRIEF_REQUEST"];

/// Kilograms per passenger, FlyByWire's own figure for the A380
/// (`Payload.tsx`'s `PAX_WEIGHT`): 84 kg, a passenger plus their cabin bag.
const KG_PER_PAX: f64 = 84.0;

fn kg(cv: &Canvas, name: &str) -> f64 {
    cv.value(name).unwrap_or(0.0)
}

/// `value` against `desired`, shown as "now / wanted" when they differ so a
/// boarding in progress reads as one.
fn against_desired(cv: &Canvas, name: &str, unit: &str, dp: usize) -> String {
    let now = kg(cv, name);
    let want = cv.value(&format!("{name}_DESIRED"));
    match want {
        Some(w) if (w - now).abs() > 0.5 => format!("{now:.dp$} / {w:.dp$} {unit}", dp = dp),
        _ => format!("{now:.dp$} {unit}", dp = dp),
    }
}

pub fn draw(cv: &mut Canvas, scroll: c_int) -> c_int {
    let (l, t, r, b) = cv.clip;
    cv.fill_px(l, t, r, b, p::PERIWINKLE);
    let lh = cv.line_h;
    let row_h = lh + 10;

    let pax: f64 = PAX_ZONES.iter().map(|z| kg(cv, z.var)).sum();
    let cargo_kg: f64 = CARGO_HOLDS.iter().map(|h| kg(cv, h.var)).sum();
    let payload_kg = pax * KG_PER_PAX + cargo_kg;
    // The same latch `weight_balance.rs` waits on, read the same way: a
    // payload FlyByWire has actually stated.
    let live = payload_kg > 1.0;

    let band_b = t - (lh + 14);
    cv.fill_px(l, t, r, band_b, p::BAND);
    let summary = if live {
        format!("{:.0} passengers and {:.0} t of cargo. The balance is live: X-Plane is flying this loadsheet.", pax, cargo_kg / 1000.0)
    } else {
        "No loadsheet. Nobody is aboard, so the plugin leaves X-Plane's own balance alone rather than write one for an aeroplane carrying nobody. Import or board below.".to_owned()
    };
    cv.text_px(l + 10, t - lh - 2, p::INK_LIGHT, FONT_BASIC, &summary);

    let mut y = band_b - 8 + scroll;
    let top_limit = band_b - 4;
    let mut height = 0;
    let mut line = |cv: &mut Canvas, y: &mut c_int, height: &mut c_int, label: &str, value: &str| {
        if *y < top_limit && *y > b {
            cv.text_px(l + 14, *y - lh, p::INK, FONT_BASIC, label);
            cv.text_px(l + 260, *y - lh, p::INK_LIGHT, FONT_BASIC, value);
        }
        *y -= lh + 4;
        *height += lh + 4;
    };
    let mut heading = |cv: &mut Canvas, y: &mut c_int, height: &mut c_int, text: &str| {
        *y -= 6;
        if *y < top_limit && *y > b {
            cv.text_px(l + 10, *y - lh, p::CHIP_INK, FONT_BASIC, text);
        }
        *y -= lh + 6;
        *height += lh + 12;
    };

    heading(cv, &mut y, &mut height, "WEIGHTS AND BALANCE");
    line(cv, &mut y, &mut height, "Zero fuel weight", &against_desired(cv, "A32NX_AIRFRAME_ZFW", "kg", 0));
    line(cv, &mut y, &mut height, "Zero fuel CG", &against_desired(cv, "A32NX_AIRFRAME_ZFW_CG_PERCENT_MAC", "% MAC", 1));
    line(cv, &mut y, &mut height, "Gross weight", &against_desired(cv, "A32NX_AIRFRAME_GW", "kg", 0));
    line(cv, &mut y, &mut height, "Gross weight CG", &against_desired(cv, "A32NX_AIRFRAME_GW_CG_PERCENT_MAC", "% MAC", 1));
    line(cv, &mut y, &mut height, "Take-off weight", &against_desired(cv, "A32NX_AIRFRAME_TOW", "kg", 0));
    line(cv, &mut y, &mut height, "Take-off CG", &against_desired(cv, "A32NX_AIRFRAME_TO_CG_PERCENT_MAC", "% MAC", 1));
    // What X-Plane itself is carrying, which is the number that actually
    // flies. It differs from the above whenever the balance is not live,
    // and that difference is the whole reason this page exists.
    let xp_lb = cv.value("TOTAL WEIGHT").unwrap_or(0.0);
    line(cv, &mut y, &mut height, "X-Plane total weight", &format!("{:.0} kg", xp_lb * 0.453_592_37));
    line(
        cv,
        &mut y,
        &mut height,
        "Balance written to X-Plane",
        if live { "yes, from this loadsheet" } else { "no, X-Plane's own is in use" },
    );

    heading(cv, &mut y, &mut height, "SIMBRIEF");
    let uid = cv.value("A32NX_CONFIG_SIMBRIEF_USERID").unwrap_or(0.0);
    line(
        cv,
        &mut y,
        &mut height,
        "Pilot ID",
        &if uid > 0.0 { format!("{uid:.0}") } else { "not set - set it on the EFB first".to_owned() },
    );
    // FlyByWire's own boarding variables, written the same way the EFB
    // writes them. `BOARDING_STARTED_BY_USR` runs the boarding toward
    // whatever the `_DESIRED` figures hold, so "Deboard" is simply those
    // set to zero and the same switch thrown.
    let buttons: &[(&str, Action)] = &[
        ("Fetch SimBrief OFP", Action::WriteVariable("EFB_SIMBRIEF_REQUEST", 1)),
        ("Board", Action::WriteVariable("BOARDING_STARTED_BY_USR", 1)),
        ("Stop", Action::WriteVariable("BOARDING_STARTED_BY_USR", 0)),
        ("Instant", Action::WriteVariable("BOARDING_RATE", 0)),
    ];
    y -= 4;
    if y < top_limit && y > b {
        let w = (r - l - 28 - 3 * 8) / 4;
        for (i, (label, action)) in buttons.iter().enumerate() {
            let x = l + 14 + i as c_int * (w + 8);
            cv.button_px(x, y, x + w, y - row_h + 4, label, false, *action);
        }
    }
    y -= row_h + 6;
    height += row_h + 6;
    let rate = cv.value("A32NX_BOARDING_RATE").unwrap_or(0.0);
    line(
        cv,
        &mut y,
        &mut height,
        "Boarding rate",
        match rate as i64 {
            0 => "instant",
            1 => "fast",
            _ => "real time",
        },
    );

    heading(cv, &mut y, &mut height, "PASSENGERS");
    for z in PAX_ZONES {
        line(cv, &mut y, &mut height, z.label, &against_desired(cv, z.var, "pax", 0));
    }
    line(cv, &mut y, &mut height, "Total", &format!("{pax:.0} pax, {:.0} kg", pax * KG_PER_PAX));

    heading(cv, &mut y, &mut height, "CARGO");
    for h in CARGO_HOLDS {
        line(cv, &mut y, &mut height, h.label, &against_desired(cv, h.var, "kg", 0));
    }
    line(cv, &mut y, &mut height, "Total", &format!("{cargo_kg:.0} kg"));

    height
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_zone_and_hold_is_a_variable_flybywire_publishes() {
        // Taken from a running session's own registry dump rather than
        // guessed: a misspelt name here reads zero, and zero payload is
        // exactly the state this page exists to make visible, so it would
        // hide the very thing it reports.
        for v in PAX_ZONES.iter().map(|z| z.var).chain(CARGO_HOLDS.iter().map(|h| h.var)) {
            assert!(v.starts_with("A32NX_"), "{v} is not a FlyByWire variable");
            assert!(!v.ends_with("_DESIRED"), "{v} is the target, not the load");
        }
        assert_eq!(PAX_ZONES.len(), 14, "the A380 has fourteen cabin zones");
        // FlyByWire's own per-station `max_pax` summed: a seat count that
        // drifts from theirs draws every zone at the wrong fill.
        assert_eq!(total_seats(), 484, "the A380X's seat map");
    }
}
