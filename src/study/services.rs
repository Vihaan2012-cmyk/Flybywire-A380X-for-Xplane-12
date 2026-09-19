//! Circuit breakers and ground services.
//!
//! - **Circuit Breakers:** every systems.cfg circuit (circuits.rs), grouped
//!   by type. Each is a button that pulls or resets its breaker, lit while
//!   the breaker is closed; a red mark shows a closed breaker whose circuit
//!   has no power.
//! - **Ground Services:** the flyPad ground services (efb.rs commands), the
//!   tug, and consumables servicing (oxygen, oxygen.rs), with the live
//!   quantities: oxygen, fuel temperatures, jettison.

use std::collections::BTreeMap;
use std::ffi::c_int;

use super::canvas::{palette as p, Action, Canvas};
use crate::xp::FONT_BASIC;

/// Draw the circuit breaker page; returns how far it scrolls.
pub fn breakers(cv: &mut Canvas, scroll: c_int) -> c_int {
    let (l, t, r, b) = cv.clip;
    cv.fill_px(l, t, r, b, p::PERIWINKLE);
    let lh = cv.line_h;
    let circuits = crate::circuits::parse_circuits(crate::circuits::SYSTEMS_CFG);
    let mut groups: BTreeMap<String, Vec<&crate::circuits::CircuitDef>> = BTreeMap::new();
    for c in &circuits {
        groups.entry(c.type_name.clone()).or_default().push(c);
    }
    let open = circuits.iter().filter(|c| cv.value(&format!("CIRCUIT BREAKER CLOSED:{}", c.number)) == Some(0.)).count();

    let band_b = t - (lh + 14);
    cv.fill_px(l, t, r, band_b, p::BAND);
    let summary = format!("{} circuits, {} breakers pulled. Click a breaker to pull or reset it.", circuits.len(), open);
    cv.text_px(l + 10, t - lh - 2, p::INK_LIGHT, FONT_BASIC, &summary);

    let columns = 3;
    let gap = 8;
    let col_w = (r - l - 20 - gap * (columns - 1)) / columns;
    let row_h = lh + 10;
    let top_limit = band_b - 4;
    let mut y = band_b - 8 + scroll;
    let mut height = 0;
    for (kind, list) in &groups {
        if y - lh <= top_limit && y > b {
            cv.text_px(l + 10, y - lh, p::INK, FONT_BASIC, kind);
        }
        y -= lh + 6;
        height += lh + 6;
        for (i, c) in list.iter().enumerate() {
            let col = (i % columns as usize) as c_int;
            let x = l + 10 + col * (col_w + gap);
            if y <= top_limit && y - row_h > b {
                let closed = cv.value(&format!("CIRCUIT BREAKER CLOSED:{}", c.number)) != Some(0.);
                // Live current and why it tripped (physics/electrical.rs):
                // 1 thermal (I squared t), 2 magnetic (instant).
                let current = cv.value(&format!("CIRCUIT CURRENT:{}", c.number));
                let cause = match cv.value(&format!("CIRCUIT TRIP CAUSE:{}", c.number)) {
                    Some(v) if v == 1. => " TRIP-THERMAL",
                    Some(v) if v == 2. => " TRIP-MAGNETIC",
                    _ => "",
                };
                let amps = current.map_or(String::new(), |a| format!(" {a:.1}A"));
                let label = format!("{} {}{amps}{cause}", c.number, c.name.as_deref().unwrap_or(""));
                cv.button_px(x, y, x + col_w, y - row_h + 2, &label, closed, Action::ToggleBreaker(c.number));
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

/// Draw the ground services page.
pub fn ground(cv: &mut Canvas) {
    let (l, t, r, b) = cv.clip;
    cv.fill_px(l, t, r, b, p::PERIWINKLE);
    let lh = cv.line_h;
    let row_h = lh + 12;
    let w = 22 * cv.char_w;

    let mut y = t - 10;
    cv.text_px(l + 10, y - lh, p::INK, FONT_BASIC, "Ground services (flyPad)");
    y -= lh + 8;
    let buttons: [(&str, Action); 6] = [
        ("Ground power unit", Action::Command("fbw/efb/ground/gpu")),
        ("Jet bridge", Action::Command("fbw/efb/ground/jetway")),
        ("Stairs", Action::Command("fbw/efb/ground/stairs")),
        ("Fuel truck", Action::Command("fbw/efb/ground/fuel_truck")),
        ("Baggage", Action::Command("fbw/efb/ground/baggage")),
        ("Catering", Action::Command("fbw/efb/ground/catering")),
    ];
    for (i, (label, action)) in buttons.iter().enumerate() {
        let x = l + 10 + (i as c_int % 3) * (w + 8);
        let yy = y - (i as c_int / 3) * row_h;
        cv.button_px(x, yy, x + w, yy - row_h + 4, label, false, *action);
    }
    y -= 2 * row_h + 10;
    let gpu = cv.value("A32NX_EFB_GROUND_GPU_CONNECTED").or_else(|| cv.value("EXTERNAL POWER AVAILABLE:1"));
    cv.text_px(l + 10, y - lh, p::INK, FONT_BASIC, &format!("GPU connected: {}", on_off(gpu)));
    y -= lh + 14;

    cv.text_px(l + 10, y - lh, p::INK, FONT_BASIC, "Pushback");
    y -= lh + 8;
    let tug: [(&str, Action); 3] = [
        ("Call tug", Action::Command("fbw/efb/pushback/call_tug")),
        ("Release tug", Action::Command("fbw/efb/pushback/release_tug")),
        ("Stop", Action::Command("fbw/efb/pushback/stop")),
    ];
    for (i, (label, action)) in tug.iter().enumerate() {
        let x = l + 10 + i as c_int * (w + 8);
        cv.button_px(x, y, x + w, y - row_h + 4, label, false, *action);
    }
    y -= row_h + 10;

    cv.text_px(l + 10, y - lh, p::INK, FONT_BASIC, "Servicing");
    y -= lh + 8;
    cv.button_px(l + 10, y, l + 10 + w, y - row_h + 4, "Service oxygen", false, Action::ServiceOxygen);
    y -= row_h + 6;
    let pct = |name: &str| cv.value(name).map_or("---".to_string(), |v| format!("{v:.0}%"));
    let lines = [
        format!("Crew oxygen: {}  {} psi  low pressure: {}", pct("OXYGEN_CREW_QUANTITY_PERCENT"), cv.value("OXYGEN_CREW_PRESSURE_PSI").map_or("---".into(), |v| format!("{v:.0}")), on_off(cv.value("OXYGEN_CREW_LOW_PRESSURE"))),
        format!("Passenger oxygen: {}  masks deployed: {}", pct("OXYGEN_PAX_QUANTITY_PERCENT"), on_off(cv.value("OXYGEN_PAX_MASKS_DEPLOYED"))),
    ];
    for line in &lines {
        cv.text_px(l + 10, y - lh, p::INK, FONT_BASIC, line);
        y -= lh + 4;
    }
    y -= 10;

    cv.text_px(l + 10, y - lh, p::INK, FONT_BASIC, "Fuel");
    y -= lh + 6;
    let temps: Vec<String> = (1..=11).map(|i| cv.value(&format!("FUEL_TEMP_{i}")).map_or("---".into(), |v| format!("{v:.0}"))).collect();
    cv.text_px(l + 10, y - lh, p::INK, FONT_BASIC, &format!("Tank temperatures (C): {}", temps.join(" ")));
    y -= lh + 4;
    let freeze = cv.value("FUEL_TEMP_FREEZE_POINT").map_or("---".into(), |v| format!("{v:.0} C"));
    cv.text_px(
        l + 10,
        y - lh,
        p::INK,
        FONT_BASIC,
        &format!("Freeze point: {freeze}  FOB LO TEMP: {}  Jettison: {}", on_off(cv.value("FUEL_FOB_LO_TEMP")), on_off(cv.value("FUEL JETTISON SWITCH"))),
    );
    let _ = r;
}

/// The ground services buttons: label and the `fbw/efb/ground/*` command
/// each presses (the same commands [`ground`] draws as buttons, and the web
/// Study tab's `/study/action` posts by name).
pub(super) fn ground_buttons() -> Vec<(&'static str, &'static str)> {
    vec![
        ("Ground power unit", "fbw/efb/ground/gpu"),
        ("Jet bridge", "fbw/efb/ground/jetway"),
        ("Stairs", "fbw/efb/ground/stairs"),
        ("Fuel truck", "fbw/efb/ground/fuel_truck"),
        ("Baggage", "fbw/efb/ground/baggage"),
        ("Catering", "fbw/efb/ground/catering"),
    ]
}

/// The pushback buttons.
pub(super) fn tug_buttons() -> Vec<(&'static str, &'static str)> {
    vec![("Call tug", "fbw/efb/pushback/call_tug"), ("Release tug", "fbw/efb/pushback/release_tug"), ("Stop", "fbw/efb/pushback/stop")]
}

/// The live quantities the ground services page shows below its buttons.
pub(super) fn ground_groups() -> Vec<super::canvas::Group> {
    use super::canvas::{group, lamp, num, palette as p};
    let mut fuel: Vec<_> = (1..=11).map(|i| num(&format!("Tank {i} temp"), format!("FUEL_TEMP_{i}"), "C", 0)).collect();
    fuel.push(num("Freeze point", "FUEL_TEMP_FREEZE_POINT", "C", 0));
    fuel.push(lamp("FOB LO TEMP", "FUEL_FOB_LO_TEMP"));
    fuel.push(lamp("Jettison", "FUEL JETTISON SWITCH"));
    vec![
        group("Status", p::STATION, vec![lamp("GPU connected", "A32NX_EFB_GROUND_GPU_CONNECTED")]),
        group(
            "Oxygen",
            p::CYAN,
            vec![
                num("Crew quantity", "OXYGEN_CREW_QUANTITY_PERCENT", "%", 0),
                num("Crew pressure", "OXYGEN_CREW_PRESSURE_PSI", "psi", 0),
                lamp("Crew low pressure", "OXYGEN_CREW_LOW_PRESSURE"),
                num("Passenger quantity", "OXYGEN_PAX_QUANTITY_PERCENT", "%", 0),
                lamp("Passenger masks deployed", "OXYGEN_PAX_MASKS_DEPLOYED"),
            ],
        ),
        group("Fuel", p::ORANGE, fuel),
    ]
}

fn on_off(v: Option<f64>) -> &'static str {
    match v {
        None => "---",
        Some(v) if v != 0. => "YES",
        Some(_) => "no",
    }
}
