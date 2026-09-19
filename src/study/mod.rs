//! Study pages: FlyByWire's systems, in X-Plane's own windows.
//!
//! The aircraft menu holds a Study tree, laid out the way study aeroplanes
//! lay theirs out. Each entry opens a window: an engine cutaway with its
//! station boxes, the electrical and hydraulic networks drawn as the
//! aircraft's own synoptic pages draw them, the fuel tanks across the wing,
//! and boxes of fields for the rest. Hover any reading to see where it comes
//! from; click it to follow that variable and watch its history.
//!
//! Every figure comes from the running simulation. Nothing is invented.

mod canvas;
mod depth;
mod elec;
mod engine;
mod failures;
mod hyd;
mod pages;
mod services;
/// JSON for the XPHFBW app's Study tab: the same page/group/topology data
/// [`build_menu`]'s windows draw from, plus the failures/breakers catalogues
/// and the action queues a click applies to. `panel.rs` routes to these.
pub(crate) mod web;

/// An ATA chapter's name ("28 Fuel"), as the Failures tab groups by it.
pub(crate) fn chapter_name(ata: u64) -> &'static str {
    failures::chapter(ata)
}

use std::collections::HashMap;
use std::ffi::{c_int, c_void};

use crate::xp::{MenuId, WindowId, Xplm, CURSOR_ARROW, FONT_BASIC, FONT_PROPORTIONAL, MOUSE_DOWN};
use canvas::{format_value, palette as p, Action, Canvas, Hit, Show};
use pages::Trace;

/// The pages the Study tree opens.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum PageKind {
    Engine(usize),
    Apu,
    Electrical,
    Hydraulics,
    FlightControls,
    Fuel,
    Bleed,
    AirConditioning,
    Pressurisation,
    GearBrakes,
    AirData,
    Fire,
    Radios,
    Failures,
    Breakers,
    GroundServices,
    All,
}

/// Every page. A menu item's refcon is its place in here.
pub(crate) const ITEMS: &[(&str, PageKind)] = &[
    ("Engine 1 State", PageKind::Engine(1)),
    ("Engine 2 State", PageKind::Engine(2)),
    ("Engine 3 State", PageKind::Engine(3)),
    ("Engine 4 State", PageKind::Engine(4)),
    ("APU", PageKind::Apu),
    ("Electrical Network", PageKind::Electrical),
    ("Hydraulic Network", PageKind::Hydraulics),
    ("Flight Controls", PageKind::FlightControls),
    ("Fuel System", PageKind::Fuel),
    ("Bleed System", PageKind::Bleed),
    ("Air Conditioning", PageKind::AirConditioning),
    ("Cabin Pressurisation", PageKind::Pressurisation),
    ("Landing Gear and Brakes", PageKind::GearBrakes),
    ("Air Data and Inertial", PageKind::AirData),
    ("Fire Protection", PageKind::Fire),
    ("Radios", PageKind::Radios),
    ("Failures", PageKind::Failures),
    ("Circuit Breakers", PageKind::Breakers),
    ("Ground Services", PageKind::GroundServices),
    ("All Variables", PageKind::All),
];

pub(crate) fn item_of(kind: PageKind) -> usize {
    ITEMS.iter().position(|i| i.1 == kind).unwrap_or(0)
}

/// What a window shows.
enum View {
    Page(usize),
    Variable { name: String, from: usize },
}

struct Win {
    id: WindowId,
    view: View,
    /// Pixels scrolled off the top, on pages taller than the window.
    scroll: c_int,
    /// How far the page runs past the foot, so scrolling stops there.
    overflow: c_int,
    mouse: (c_int, c_int),
    hits: Vec<Hit>,
    /// Schematic pages: showing the physics models' quantities instead.
    physics: bool,
}

/// The height of the title band across the top of every window.
const BAND_H: c_int = 50;

static mut MENU: MenuId = std::ptr::null_mut();
static mut WINDOWS: Vec<Win> = Vec::new();
static mut TRACES: Option<HashMap<String, Trace>> = None;

/// The windows, by the index a window's refcon carries. Entries are never
/// removed, so an index stays good for as long as the plugin runs. Only the
/// main thread touches them: X-Plane calls every window callback there.
#[allow(static_mut_refs)]
unsafe fn windows() -> &'static mut Vec<Win> {
    &mut *(&raw mut WINDOWS)
}

#[allow(static_mut_refs)]
unsafe fn traces() -> &'static mut HashMap<String, Trace> {
    let traces = &mut *(&raw mut TRACES);
    traces.get_or_insert_with(HashMap::new)
}

/// Build the aircraft menu and its Study tree.
pub fn build_menu(xplm: &Xplm) {
    unsafe {
        let root = xplm.menu("FlyByWire A380X", menu_handler);
        if root.is_null() {
            crate::log("no study menu: X-Plane gave the aircraft no menu to hang it on");
            return;
        }
        let study = xplm.submenu(root, "Study", menu_handler);
        let add = |menu: MenuId, kind: PageKind| {
            let at = item_of(kind);
            xplm.menu_item(menu, &format!("{}...", ITEMS[at].0), at as *mut c_void);
        };

        add(study, PageKind::All);
        add(study, PageKind::AirData);
        add(study, PageKind::Electrical);

        let engines = xplm.submenu(study, "Engines", menu_handler);
        for n in 1..=4 {
            add(engines, PageKind::Engine(n));
        }
        xplm.menu_separator(engines);
        add(engines, PageKind::Apu);
        xplm.menu_separator(engines);
        add(engines, PageKind::Bleed);
        add(engines, PageKind::Fuel);

        let environmental = xplm.submenu(study, "Environmental", menu_handler);
        add(environmental, PageKind::AirConditioning);
        add(environmental, PageKind::Pressurisation);

        let hydraulics = xplm.submenu(study, "Hydraulics", menu_handler);
        add(hydraulics, PageKind::Hydraulics);
        add(hydraulics, PageKind::FlightControls);

        let landing_gear = xplm.submenu(study, "Landing Gear", menu_handler);
        add(landing_gear, PageKind::GearBrakes);

        let fire = xplm.submenu(study, "Fire", menu_handler);
        add(fire, PageKind::Fire);

        add(study, PageKind::Radios);
        add(study, PageKind::Failures);
        add(study, PageKind::Breakers);
        add(study, PageKind::GroundServices);

        let menu = &raw mut MENU;
        *menu = root;
        crate::log(&format!("study menu ready with {} pages", ITEMS.len()));
    }
}

pub fn destroy(xplm: &Xplm) {
    unsafe {
        for win in windows().drain(..) {
            xplm.destroy_window(win.id);
        }
        let menu = &raw mut MENU;
        xplm.destroy_menu(*menu);
        *menu = std::ptr::null_mut();
        *(&raw mut TRACES) = None;
    }
}

unsafe extern "C" fn menu_handler(_menu: *mut c_void, item: *mut c_void) {
    let at = item as usize;
    if at >= ITEMS.len() {
        return;
    }
    let xplm = &raw const crate::XPLM;
    let Some(xplm) = (*xplm).as_ref() else { return };
    // A page already open comes back to the front rather than opening twice.
    if let Some(win) = windows().iter_mut().find(|w| matches!(w.view, View::Page(p) if p == at)) {
        win.scroll = 0;
        xplm.show(win.id, true);
        return;
    }
    let index = windows().len();
    let id = xplm.window(1180, 760, draw, click, wheel, cursor, index as *mut c_void);
    xplm.set_title(id, ITEMS[at].0);
    windows().push(Win {
        id,
        view: View::Page(at),
        scroll: 0,
        overflow: 0,
        mouse: (-1, -1),
        hits: Vec::new(),
        physics: false,
    });
}

unsafe extern "C" fn click(_window: WindowId, x: c_int, y: c_int, status: c_int, refcon: *mut c_void) -> c_int {
    if status != MOUSE_DOWN {
        return 1;
    }
    let xplm = &raw const crate::XPLM;
    let Some(xplm) = (*xplm).as_ref() else { return 1 };
    let Some(win) = windows().get_mut(refcon as usize) else { return 1 };
    let Some(action) = win
        .hits
        .iter()
        .rev()
        .find(|h| x >= h.left && x <= h.right && y <= h.top && y >= h.bottom)
        .map(|h| h.action)
    else {
        return 1;
    };
    let here = match win.view {
        View::Page(at) => at,
        View::Variable { from, .. } => from,
    };
    match action {
        Action::Follow(index) => {
            // Indices move when a variable is registered, so the name is what
            // the window keeps.
            let name = crate::snapshot().lock().ok().and_then(|s| s.names.get(index).cloned());
            if let Some(name) = name {
                xplm.set_title(win.id, &name);
                win.view = View::Variable { name, from: here };
            }
        }
        Action::Back => {
            xplm.set_title(win.id, ITEMS[here].0);
            win.view = View::Page(here);
        }
        Action::Page(at) => {
            xplm.set_title(win.id, ITEMS[at].0);
            win.view = View::Page(at);
        }
        Action::Press(command) => {
            crate::published::press(command);
            return 1;
        }
        Action::ToggleFailure(id) => {
            crate::failures::toggle(id);
            return 1;
        }
        Action::ToggleBreaker(number) => {
            crate::circuits::request_toggle(number);
            return 1;
        }
        Action::Command(name) => {
            crate::xp::command_once(name);
            return 1;
        }
        Action::ServiceOxygen => {
            crate::oxygen::request_service();
            return 1;
        }
        Action::TogglePhysics => {
            win.physics = !win.physics;
        }
    }
    win.scroll = 0;
    win.hits.clear();
    1
}

unsafe extern "C" fn wheel(
    _window: WindowId,
    _x: c_int,
    _y: c_int,
    _axis: c_int,
    clicks: c_int,
    refcon: *mut c_void,
) -> c_int {
    if let Some(win) = windows().get_mut(refcon as usize) {
        win.scroll = (win.scroll - clicks * 48).clamp(0, win.overflow.max(0));
    }
    1
}

unsafe extern "C" fn cursor(_window: WindowId, x: c_int, y: c_int, refcon: *mut c_void) -> c_int {
    if let Some(win) = windows().get_mut(refcon as usize) {
        win.mouse = (x, y);
    }
    CURSOR_ARROW
}

unsafe extern "C" fn draw(window: WindowId, refcon: *mut c_void) {
    let xplm = &raw const crate::XPLM;
    let Some(xplm) = (*xplm).as_ref() else { return };
    let (left, top, right, bottom) = xplm.geometry(window);
    let Some(win) = windows().get_mut(refcon as usize) else { return };
    win.hits.clear();
    let Ok(snap) = crate::snapshot().lock() else { return };

    // The window's ground and title band.
    xplm.fill(left, top, right, bottom, p::WINDOW);
    let band = top - BAND_H;
    xplm.fill(left, top, right, band, p::BAND);
    xplm.line(left, band, right, band, p::EDGE, 1.);
    let (char_w, font_h) = xplm.font_size(FONT_BASIC);

    let (page_at, title) = match &win.view {
        View::Page(at) => (*at, ITEMS[*at].0.to_string()),
        View::Variable { name, from } => (*from, name.clone()),
    };
    xplm.text(left + 14, top - 22, p::INK_LIGHT, FONT_PROPORTIONAL, &title);
    let state = format!(
        "{}   {:.0} s   {} ticks   {} variables",
        if snap.ticks > 0 { "SIMULATION RUNNING" } else { "SIMULATION NOT STARTED" },
        snap.time,
        snap.ticks,
        snap.values.len()
    );
    let state_ink = if snap.ticks > 0 { [0.80, 0.85, 0.95] } else { p::ECAM_AMBER };
    xplm.text(left + 14, top - 40, state_ink, FONT_BASIC, &state);

    // Followed variables keep recording while shown.
    let trace = if let View::Variable { name, .. } = &win.view {
        snap.find(name).map(|i| {
            let value = snap.values[i];
            let trace = traces().entry(name.clone()).or_insert_with(|| Trace::new(value, snap.time));
            trace.record(value, snap.time);
            &*trace
        })
    } else {
        None
    };

    let clip = (left + 8, band - 8, right - 8, bottom + 8);
    let mut overflow = 0;
    let tip;
    {
        let mut cv = Canvas::new(xplm, &snap, clip, win.mouse, &mut win.hits);
        let physics_kind = match &win.view {
            View::Page(at) if win.physics && depth::has_physics(ITEMS[*at].1) => Some(ITEMS[*at].1),
            _ => None,
        };
        if let Some(kind) = physics_kind {
            overflow = pages::flow(&mut cv, &depth::extra(kind), win.scroll);
        } else {
        match &win.view {
            View::Page(at) => match ITEMS[*at].1 {
                PageKind::Engine(n) => engine::draw(&mut cv, n),
                PageKind::Electrical => elec::draw(&mut cv),
                PageKind::Hydraulics => hyd::draw(&mut cv),
                PageKind::Fuel => pages::fuel(&mut cv),
                PageKind::FlightControls => pages::flight_controls(&mut cv),
                PageKind::All => overflow = pages::all(&mut cv, win.scroll),
                PageKind::Radios => overflow = pages::radios(&mut cv, win.scroll),
                PageKind::Failures => overflow = failures::draw(&mut cv, win.scroll),
                PageKind::Breakers => overflow = services::breakers(&mut cv, win.scroll),
                PageKind::GroundServices => services::ground(&mut cv),
                other => overflow = pages::flow(&mut cv, &pages::groups(other), win.scroll),
            },
            View::Variable { name, from } => pages::variable(&mut cv, name, ITEMS[*from].0, trace),
        }
        }
        tip = cv.tip;
    }
    win.overflow = overflow;
    win.scroll = win.scroll.min(overflow);

    // Band controls: the note, and the engine selector on engine pages.
    let lh = font_h + 4;
    let mut note_left = right - 14;
    if let (View::Page(_), PageKind::Engine(current)) = (&win.view, ITEMS[page_at].1) {
        let w = 7 * char_w + 12;
        let mut cv = Canvas::new(xplm, &snap, (left, top, right, band), win.mouse, &mut win.hits);
        for n in (1..=4).rev() {
            let r = note_left;
            let l = r - w;
            cv.button_px(l, top - 12, r, top - 14 - lh - 6, &format!("ENG {n}"), n == current, Action::Page(item_of(PageKind::Engine(n))));
            note_left = l - 6;
        }
        note_left -= 8;
    }
    // Schematic pages: the schematic, or the physics behind it.
    if let View::Page(_) = win.view {
        if depth::has_physics(ITEMS[page_at].1) {
            let w = 10 * char_w + 12;
            let mut cv = Canvas::new(xplm, &snap, (left, top, right, band), win.mouse, &mut win.hits);
            let label = if win.physics { "SCHEMATIC" } else { "PHYSICS" };
            cv.button_px(note_left - w, top - 12, note_left, top - 14 - lh - 6, label, win.physics, Action::TogglePhysics);
            note_left -= w + 14;
        }
    }
    let note = match win.view {
        View::Page(_) if overflow > 0 => "hover a reading for its source, click to follow it, wheel to scroll",
        View::Page(_) => "hover a reading for its source, click to follow it",
        View::Variable { .. } => "the page's own reading, source and history",
    };
    let note_w = note.chars().count() as c_int * char_w + 20;
    let note_l = note_left - note_w;
    if note_l > left + 14 + title.chars().count() as c_int * char_w + 20 {
        xplm.fill(note_l, top - 12, note_left, top - 14 - lh - 6, p::NOTE.body);
        xplm.frame(note_l, top - 12, note_left, top - 14 - lh - 6, p::EDGE, 1.);
        xplm.text(note_l + 10, top - 14 - lh + 1, p::INK, FONT_BASIC, note);
    }

    if let Some(index) = tip {
        tooltip(xplm, &snap, index, win.mouse, (left, top, right, bottom), char_w, lh);
    }
}

/// The box under the mouse: which variable, what it reads, where from.
fn tooltip(
    xplm: &Xplm,
    snap: &crate::Snapshot,
    index: usize,
    (mx, my): (c_int, c_int),
    (left, top, right, bottom): (c_int, c_int, c_int, c_int),
    char_w: c_int,
    lh: c_int,
) {
    let Some(name) = snap.names.get(index) else { return };
    let value = snap.values.get(index).copied().unwrap_or(f64::NAN);
    let source = snap.sources.get(index).copied().unwrap_or(0);
    let reading = if canvas::looks_packed(value) {
        format!("{}  (packed ARINC 429 word)", format_value(value, Show::Arinc(2), ""))
    } else {
        canvas::number(value)
    };
    let mut lines = vec![name.clone(), format!("reads {reading}")];
    lines.extend(pages::wrap(&pages::source_text(name, source), 60));
    if let Some(dataref) = snap.datarefs.get(index).filter(|d| !d.is_empty()) {
        lines.push(format!("dataref {dataref}"));
    }
    lines.push("click to follow".into());

    let w = lines.iter().map(|l| l.chars().count()).max().unwrap_or(10) as c_int * char_w + 20;
    let h = lines.len() as c_int * lh + 12;
    // Beside the mouse, kept inside the window.
    let mut l = mx + 18;
    let mut t = my - 18;
    if l + w > right - 4 {
        l = (mx - 18 - w).max(left + 4);
    }
    if t - h < bottom + 4 {
        t = (my + 18 + h).min(top - 4);
    }
    xplm.fill(l + 3, t - 3, l + w + 3, t - h - 3, [0., 0., 0., 0.35]);
    xplm.fill(l, t, l + w, t - h, [0.99, 0.98, 0.88, 1.]);
    xplm.frame(l, t, l + w, t - h, p::EDGE, 1.);
    let mut y = t - lh;
    for (i, line) in lines.iter().enumerate() {
        let ink = if i == 0 { p::INK } else if source == 0 && i >= 2 { [0.55, 0.32, 0.02] } else { p::INK_DIM };
        xplm.text(l + 10, y, ink, FONT_BASIC, line);
        y -= lh;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_page_has_one_menu_item() {
        for (i, (_, kind)) in ITEMS.iter().enumerate() {
            assert_eq!(item_of(*kind), i, "{kind:?} is listed twice");
        }
    }

    #[test]
    fn engine_items_are_found_by_engine() {
        assert_eq!(ITEMS[item_of(PageKind::Engine(3))].0, "Engine 3 State");
    }
}
