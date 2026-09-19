//! The electrical network, drawn the way the aircraft's own ELEC page draws
//! it: sources along the edges, buses between them, and wires that light
//! green when a live source is feeding a powered bus.
//!
//! The wiring follows FlyByWire's A380 electrical code, not a guess:
//! generators, external power and the APU generators feed AC buses 1 to 4;
//! AC 1 feeds AC ESS normally and AC 4 in alternate; the emergency generator
//! backs AC ESS; transformer 1 takes AC 2, transformer 2 takes AC 3, the ESS
//! transformer takes AC ESS and the APU transformer takes AC 4. Batteries 1,
//! 2, ESS and APU are its batteries 1 to 4, each with its hot bus.

use std::ffi::c_int;

use super::canvas::{palette as p, Action, Canvas, Reading, DESIGN_H, DESIGN_W};
use crate::xp::{FONT_BASIC, FONT_PROPORTIONAL};

/// A box as drawn, in pixels.
#[derive(Clone, Copy, Debug)]
pub struct Rect {
    pub l: c_int,
    pub t: c_int,
    pub r: c_int,
    pub b: c_int,
}

impl Rect {
    pub fn cx(&self) -> c_int {
        (self.l + self.r) / 2
    }
}

/// One line inside a source box.
pub enum Line {
    /// A number with its unit. `normal` names the flag FlyByWire sets when
    /// the reading is within limits; a live reading outside them is amber.
    Value { name: String, unit: &'static str, decimals: usize, normal: Option<String> },
    /// A word shown lit when the variable is set: green when that is good,
    /// amber when it is a warning.
    Flag { name: String, text: &'static str, warning: bool },
}

pub fn value(name: impl Into<String>, unit: &'static str, decimals: usize, normal: Option<String>) -> Line {
    Line::Value { name: name.into(), unit, decimals, normal }
}

pub fn flag(name: impl Into<String>, text: &'static str) -> Line {
    Line::Flag { name: name.into(), text, warning: false }
}

pub fn warning(name: impl Into<String>, text: &'static str) -> Line {
    Line::Flag { name: name.into(), text, warning: true }
}

/// A wire between boxes, through the given pixel points.
pub fn wire(cv: &Canvas, points: &[(c_int, c_int)], live: bool) {
    let (colour, width) = if live { (p::ECAM_LIVE, 2.5) } else { (p::ECAM_DEAD, 1.5) };
    for pair in points.windows(2) {
        cv.line_px(pair[0].0, pair[0].1, pair[1].0, pair[1].1, colour, width);
    }
}

/// A fluid line: blue when there is fluid behind it.
pub fn pipe(cv: &Canvas, points: &[(c_int, c_int)], full: bool) {
    let (colour, width) = if full { (p::ECAM_FLUID, 2.5) } else { (p::ECAM_DEAD, 1.5) };
    for pair in points.windows(2) {
        cv.line_px(pair[0].0, pair[0].1, pair[1].0, pair[1].1, colour, width);
    }
}

/// A source: a titled box of readings, black with a white edge when any
/// reading is live and grey when all are dead.
pub fn source(cv: &mut Canvas, x: f32, y: f32, w: f32, title: &str, lines: &[Line]) -> Rect {
    let (l, t, r) = (cv.x(x), cv.y(y), cv.x(x + w));
    let lh = cv.line_h;
    let b = t - (lh + 4) - lines.len() as c_int * lh - 5;
    let live = lines.iter().any(|line| match line {
        Line::Value { name, .. } => cv.value(name).is_some_and(|v| v.abs() > 0.5),
        Line::Flag { name, warning, .. } => !warning && cv.on(name),
    });

    cv.fill_px(l, t, r, b, p::ECAM_GROUND);
    cv.frame_px(l, t, r, b, if live { p::ECAM_EDGE } else { p::ECAM_DEAD }, 1.);
    let title_ink = if live { p::ECAM_WHITE } else { p::ECAM_GREY };
    cv.text_centred_px((l + r) / 2, t - lh + 1, title_ink, FONT_PROPORTIONAL, title);

    let unit_w = 4 * cv.char_w;
    let mut top = t - lh - 4;
    for line in lines {
        let bottom = top - lh;
        let base = bottom + 4;
        let name = match line {
            Line::Value { name, .. } | Line::Flag { name, .. } => name,
        };
        match cv.read(name) {
            Reading::Missing => {
                cv.text_centred_px((l + r) / 2, base, p::ECAM_GREY, FONT_BASIC, "not modelled");
            }
            Reading::Value { index, value, source } => {
                if cv.hovering(l, top, r, bottom) {
                    cv.fill_px(l + 1, top, r - 1, bottom, [1., 1., 1., 0.12]);
                    cv.tip = Some(index);
                }
                match line {
                    Line::Value { unit, decimals, normal, .. } => {
                        let abnormal = normal.as_deref().is_some_and(|n| {
                            let flag = cv.read(n);
                            matches!(flag, Reading::Value { .. }) && !flag.on()
                        });
                        let ink = if !value.is_finite() || source == 0 || value.abs() < 0.5 {
                            p::ECAM_GREY
                        } else if abnormal {
                            p::ECAM_AMBER
                        } else {
                            p::ECAM_GREEN
                        };
                        let s = if value.is_finite() { format!("{value:.decimals$}") } else { "--".into() };
                        cv.text_px(r - unit_w - 8 - cv.text_width(&s), base, ink, FONT_BASIC, &s);
                        cv.text_px(r - unit_w - 2, base, p::ECAM_CYAN, FONT_BASIC, unit);
                    }
                    Line::Flag { text, warning, .. } => {
                        let set = value.is_finite() && value != 0.;
                        let ink = match (set, warning) {
                            (false, _) => p::ECAM_GREY,
                            (true, false) => p::ECAM_GREEN,
                            (true, true) => p::ECAM_AMBER,
                        };
                        cv.text_centred_px((l + r) / 2, base, ink, FONT_BASIC, text);
                    }
                }
                if source == 0 {
                    cv.fill_px(l + 2, top - 2, l + 4, bottom + 2, p::UNFED);
                }
                cv.hit_px(l, top, r, bottom, Action::Follow(index));
            }
        }
        top = bottom;
    }
    Rect { l, t, r, b }
}

/// A bus: green when powered, amber when not, grey when the simulation does
/// not model it. Returns the box and whether it is powered.
pub fn bus(cv: &mut Canvas, x: f32, y: f32, w: f32, h: f32, title: &str, name: &str) -> (Rect, bool) {
    let (l, t, r, b) = (cv.x(x), cv.y(y), cv.x(x + w), cv.y(y + h));
    let reading = cv.read(name);
    let powered = reading.on();
    let (edge, ink, ground) = match reading {
        Reading::Missing => (p::ECAM_DEAD, p::ECAM_GREY, p::ECAM_GROUND),
        _ if powered => (p::ECAM_LIVE, p::ECAM_GREEN, [0.04, 0.19, 0.08, 1.]),
        _ => ([1., 0.66, 0.12, 1.], p::ECAM_AMBER, p::ECAM_GROUND),
    };
    cv.fill_px(l, t, r, b, ground);
    cv.frame_px(l, t, r, b, edge, 1.5);
    let label = match reading {
        Reading::Missing => format!("{title}: not modelled"),
        _ => title.to_string(),
    };
    let base = b + (t - b - cv.line_h) / 2 + 4;
    cv.text_centred_px((l + r) / 2, base, ink, FONT_PROPORTIONAL, &label);
    if let Reading::Value { index, source, .. } = reading {
        if cv.hovering(l, t, r, b) {
            cv.fill_px(l, t, r, b, [1., 1., 1., 0.10]);
            cv.tip = Some(index);
        }
        if source == 0 {
            cv.fill_px(l + 2, t - 2, l + 4, b + 2, p::UNFED);
        }
        cv.hit_px(l, t, r, b, Action::Follow(index));
    }
    (Rect { l, t, r, b }, powered)
}

/// The centres of the four main columns: AC and DC 1, 2, 3, 4 sides.
const AC_CENTRES: [f32; 4] = [100., 290., 710., 900.];

pub fn draw(cv: &mut Canvas) {
    cv.fill(0., 0., DESIGN_W, DESIGN_H, p::ECAM_GROUND);
    cv.text(6., 2., p::ECAM_CYAN, FONT_PROPORTIONAL, "AC");
    cv.line(0., 302., DESIGN_W, 302., p::ECAM_DEAD, 1.);
    cv.text(6., 306., p::ECAM_CYAN, FONT_PROPORTIONAL, "DC");

    // External power along the top, the APU generators between.
    let mut ext = Vec::with_capacity(4);
    for (i, cx) in AC_CENTRES.iter().enumerate() {
        let n = i + 1;
        ext.push(source(
            cv,
            cx - 60.,
            6.,
            120.,
            &format!("EXT {n}"),
            &[
                flag(format!("A32NX_EXT_PWR_AVAIL:{n}"), "AVAIL"),
                flag(format!("A32NX_OVHD_ELEC_EXT_PWR_{n}_PB_IS_ON"), "ON"),
            ],
        ));
    }
    let apu: Vec<Rect> = [("APU GEN A", 1, 395.), ("APU GEN B", 2, 505.)]
        .iter()
        .map(|&(title, n, x)| {
            let g = format!("A32NX_ELEC_APU_GEN_{n}");
            source(
                cv,
                x,
                6.,
                100.,
                title,
                &[
                    value(format!("{g}_POTENTIAL"), "V", 0, Some(format!("{g}_POTENTIAL_NORMAL"))),
                    value(format!("{g}_FREQUENCY"), "HZ", 0, Some(format!("{g}_FREQUENCY_NORMAL"))),
                    value(format!("{g}_LOAD"), "%", 0, Some(format!("{g}_LOAD_NORMAL"))),
                ],
            )
        })
        .collect();

    // The AC buses.
    let mut ac = Vec::with_capacity(4);
    for (i, cx) in AC_CENTRES.iter().enumerate() {
        let n = i + 1;
        ac.push(bus(cv, cx - 70., 112., 140., 26., &format!("AC {n}"), &format!("A32NX_ELEC_AC_{n}_BUS_IS_POWERED")));
    }
    let (ess, ess_on) = bus(cv, 430., 112., 140., 26., "AC ESS", "A32NX_ELEC_AC_ESS_BUS_IS_POWERED");
    let (shed, shed_on) = bus(cv, 430., 176., 140., 24., "AC ESS SHED", "A32NX_ELEC_AC_ESS_SHED_BUS_IS_POWERED");

    // Generators below their buses, the emergency generator below AC ESS.
    let mut gens = Vec::with_capacity(4);
    for (i, cx) in AC_CENTRES.iter().enumerate() {
        let n = i + 1;
        let g = format!("A32NX_ELEC_ENG_GEN_{n}");
        gens.push(source(
            cv,
            cx - 65.,
            214.,
            130.,
            &format!("GEN {n}"),
            &[
                value(format!("{g}_POTENTIAL"), "V", 0, Some(format!("{g}_POTENTIAL_NORMAL"))),
                value(format!("{g}_FREQUENCY"), "HZ", 0, Some(format!("{g}_FREQUENCY_NORMAL"))),
                value(format!("{g}_LOAD"), "%", 0, Some(format!("{g}_LOAD_NORMAL"))),
            ],
        ));
    }
    let emer = source(
        cv,
        440.,
        214.,
        120.,
        "EMER GEN",
        &[
            value("A32NX_ELEC_EMER_GEN_POTENTIAL", "V", 0, None),
            value("A32NX_ELEC_EMER_GEN_FREQUENCY", "HZ", 0, None),
        ],
    );

    // AC wiring.
    let tie_y = cv.y(94.);
    let apu_live = cv.on("A32NX_ELEC_APU_GEN_1_POTENTIAL_NORMAL") || cv.on("A32NX_ELEC_APU_GEN_2_POTENTIAL_NORMAL");
    let apu_normal = [
        cv.on("A32NX_ELEC_APU_GEN_1_POTENTIAL_NORMAL"),
        cv.on("A32NX_ELEC_APU_GEN_2_POTENTIAL_NORMAL"),
    ];
    for (i, gen) in apu.iter().enumerate() {
        wire(cv, &[(gen.cx(), gen.b), (gen.cx(), tie_y)], apu_normal[i]);
    }
    wire(cv, &[(cv.x(130.), tie_y), (cv.x(930.), tie_y)], apu_live);
    cv.text_centred_px(cv.x(200.), tie_y + 3, p::ECAM_GREY, FONT_BASIC, "BUS TIE");
    for i in 0..4 {
        let (bus, powered) = ac[i];
        let cx = cv.x(AC_CENTRES[i]);
        let offset = cv.px(30.);
        let n = i + 1;
        let ext_live = cv.on(&format!("A32NX_EXT_PWR_AVAIL:{n}"))
            && cv.on(&format!("A32NX_OVHD_ELEC_EXT_PWR_{n}_PB_IS_ON"))
            && powered;
        wire(cv, &[(cx - offset, ext[i].b), (cx - offset, bus.t)], ext_live);
        wire(cv, &[(cx + offset, tie_y), (cx + offset, bus.t)], apu_live && powered);
        let gen_live = cv.on(&format!("A32NX_ELEC_ENG_GEN_{n}_POTENTIAL_NORMAL")) && powered;
        wire(cv, &[(cx, gens[i].t), (cx, bus.b)], gen_live);
    }
    let (ac1, ac1_on) = ac[0];
    let (ac4, ac4_on) = ac[3];
    let norm_y = cv.y(150.);
    let altn_y = cv.y(160.);
    let (x_norm, x_altn) = (cv.x(470.), cv.x(530.));
    wire(
        cv,
        &[(cv.x(140.), ac1.b), (cv.x(140.), norm_y), (x_norm, norm_y), (x_norm, ess.b)],
        ac1_on && ess_on,
    );
    cv.text_px(cv.x(300.), norm_y + 3, p::ECAM_GREY, FONT_BASIC, "NORM");
    wire(
        cv,
        &[(cv.x(860.), ac4.b), (cv.x(860.), altn_y), (x_altn, altn_y), (x_altn, ess.b)],
        ac4_on && ess_on && !ac1_on,
    );
    cv.text_px(cv.x(640.), altn_y + 3, p::ECAM_GREY, FONT_BASIC, "ALTN");
    wire(cv, &[(ess.cx(), ess.b), (ess.cx(), shed.t)], ess_on && shed_on);
    let emer_live = cv.value("A32NX_ELEC_EMER_GEN_POTENTIAL").is_some_and(|v| v > 50.) && ess_on;
    wire(cv, &[(emer.cx(), emer.t), (emer.cx(), shed.b)], emer_live);

    // Transformer rectifiers, each with the AC bus it draws from.
    let ac_on = [ac[0].1, ac[1].1, ac[2].1, ac[3].1];
    let trs = [
        ("TR 1", 1, 210., "AC 2", ac_on[1]),
        ("ESS TR", 3, 500., "AC ESS", ess_on),
        ("TR 2", 2, 700., "AC 3", ac_on[2]),
        ("APU TR", 4, 900., "AC 4", ac_on[3]),
    ];
    let mut tr_rects = Vec::with_capacity(4);
    for &(title, n, cx, from, from_on) in &trs {
        let t = format!("A32NX_ELEC_TR_{n}");
        let rect = source(
            cv,
            cx - 60.,
            324.,
            120.,
            title,
            &[
                value(format!("{t}_POTENTIAL"), "V", 1, Some(format!("{t}_POTENTIAL_NORMAL"))),
                value(format!("{t}_CURRENT"), "A", 0, Some(format!("{t}_CURRENT_NORMAL"))),
            ],
        );
        let stub_top = cv.y(310.);
        wire(cv, &[(rect.cx(), stub_top), (rect.cx(), rect.t)], from_on);
        cv.text_px(rect.cx() + 6, stub_top - cv.line_h + 4, if from_on { p::ECAM_GREEN } else { p::ECAM_GREY }, FONT_BASIC, from);
        tr_rects.push(rect);
    }

    // DC buses.
    let dc = [
        bus(cv, 130., 404., 160., 26., "DC 1", "A32NX_ELEC_DC_1_BUS_IS_POWERED"),
        bus(cv, 420., 404., 160., 26., "DC ESS", "A32NX_ELEC_DC_ESS_BUS_IS_POWERED"),
        bus(cv, 620., 404., 160., 26., "DC 2", "A32NX_ELEC_DC_2_BUS_IS_POWERED"),
        bus(cv, 820., 404., 160., 26., "DC APU", "A32NX_ELEC_DC_APU_BUS_IS_POWERED"),
    ];
    for (i, rect) in tr_rects.iter().enumerate() {
        let n = trs[i].1;
        let tr_live = cv.value(&format!("A32NX_ELEC_TR_{n}_POTENTIAL")).is_some_and(|v| v > 20.);
        let (bus, powered) = dc[i];
        wire(cv, &[(rect.cx(), rect.b), (rect.cx(), bus.t)], tr_live && powered);
    }

    // Batteries under their buses, each over its hot bus.
    let bats = [("BAT 1", 1, 130.), ("BAT ESS", 3, 420.), ("BAT 2", 2, 620.), ("BAT APU", 4, 820.)];
    let hots = [("HOT 1", 1), ("HOT ESS", 3), ("HOT 2", 2), ("HOT APU", 4)];
    for (i, &(title, n, x)) in bats.iter().enumerate() {
        let b = format!("A32NX_ELEC_BAT_{n}");
        let rect = source(
            cv,
            x,
            454.,
            160.,
            title,
            &[
                value(format!("{b}_POTENTIAL"), "V", 1, Some(format!("{b}_POTENTIAL_NORMAL"))),
                value(format!("{b}_CURRENT"), "A", 0, Some(format!("{b}_CURRENT_NORMAL"))),
            ],
        );
        let (bus_rect, bus_on) = dc[i];
        let flowing = cv.value(&format!("{b}_CURRENT")).is_some_and(|a| a.abs() > 0.5);
        wire(cv, &[(rect.cx(), rect.t), (rect.cx(), bus_rect.b)], flowing && bus_on);

        let (hot_title, hot_n) = hots[i];
        let (hot, hot_on) = bus(cv, x + 20., 532., 120., 22., hot_title, &format!("A32NX_ELEC_DC_HOT_{hot_n}_BUS_IS_POWERED"));
        wire(cv, &[(rect.cx(), rect.b), (rect.cx(), hot.t)], hot_on);
    }

    // The static inverter, fed from the essential side.
    let inv = source(
        cv,
        300.,
        454.,
        110.,
        "STAT INV",
        &[
            value("A32NX_ELEC_STAT_INV_POTENTIAL", "V", 0, None),
            value("A32NX_ELEC_STAT_INV_FREQUENCY", "HZ", 0, None),
        ],
    );
    let inv_live = cv.value("A32NX_ELEC_STAT_INV_POTENTIAL").is_some_and(|v| v > 20.);
    let (dc_ess, _) = dc[1];
    let turn = cv.y(442.);
    wire(
        cv,
        &[(cv.x(440.), dc_ess.b), (cv.x(440.), turn), (inv.cx(), turn), (inv.cx(), inv.t)],
        inv_live,
    );

    // The key, and the two network-wide switches.
    let key_y = 578.;
    cv.text(10., key_y, p::ECAM_GREY, FONT_BASIC, "green wire: a live source feeding a powered bus");
    let flags = [
        ("BUS TIE AUTO", "A32NX_OVHD_ELEC_BUS_TIE_PB_IS_AUTO", false),
        ("GALLEY SHED", "A32NX_ELEC_GALLEY_IS_SHED", true),
    ];
    for (i, (text, name, warn)) in flags.iter().enumerate() {
        let x = 620. + i as f32 * 190.;
        let (l, t) = (cv.x(x), cv.y(key_y));
        let r = l + cv.text_width(text) + 16;
        let b = t - cv.line_h;
        match cv.read(name) {
            Reading::Missing => cv.text_px(l, b + 4, p::ECAM_GREY, FONT_BASIC, &format!("{text}: not modelled")),
            Reading::Value { index, value, .. } => {
                let set = value.is_finite() && value != 0.;
                let ink = match (set, warn) {
                    (false, _) => p::ECAM_GREY,
                    (true, false) => p::ECAM_GREEN,
                    (true, true) => p::ECAM_AMBER,
                };
                if cv.hovering(l, t, r, b) {
                    cv.tip = Some(index);
                }
                cv.frame_px(l - 4, t, r, b, if set { p::ECAM_EDGE } else { p::ECAM_DEAD }, 1.);
                cv.text_px(l + 4, b + 4, ink, FONT_BASIC, text);
                cv.hit_px(l - 4, t, r, b, Action::Follow(index));
            }
        }
    }
}

// ---------------------------------------------------------------------
// Topology data for the web Study tab: the same network [`draw`] draws to
// an XPLM window, described as nodes and links on the design sheet instead
// of painted pixels, so a browser can lay the synoptic out and colour it
// itself from `/vars`. Positions and the "live" conditions are read from
// the same numbers and variable names `draw` uses; only the exact pixel
// sizing (which depends on X-Plane's own font metrics) is replaced by
// [`box_h`]'s design-unit approximation.

/// One reading or flag shown in a topology node's box.
pub(super) struct TopoField {
    /// A left-hand label, for a box drawn the canvas (Study page) way: a
    /// label on the left, the reading on the right. `None` for the
    /// aircraft's own ECAM-style boxes, whose rows carry no separate label
    /// ([`source`]'s value rows are right-aligned bare readings; its flag
    /// rows are the flag's own text, centred).
    pub label: Option<String>,
    pub name: String,
    pub unit: &'static str,
    pub decimals: usize,
    /// "num" (a plain reading), "lamp" (canvas-style on/off), or "flag"
    /// (ECAM-style: `flag_text` lit when the variable is set).
    pub kind: &'static str,
    pub flag_text: &'static str,
}

/// An ECAM-style bare reading (no label, right-aligned, as [`source`] draws).
pub(super) fn tfield(name: impl Into<String>, unit: &'static str, decimals: usize) -> TopoField {
    TopoField { label: None, name: name.into(), unit, decimals, kind: "num", flag_text: "" }
}

/// An ECAM-style flag (no label, `text` shown centred and lit when set).
pub(super) fn tflag(name: impl Into<String>, text: &'static str) -> TopoField {
    TopoField { label: None, name: name.into(), unit: "", decimals: 0, kind: "flag", flag_text: text }
}

/// A canvas-style labelled reading, as the Study pages' own boxes draw.
pub(super) fn lfield(label: &str, name: impl Into<String>, unit: &'static str, decimals: usize) -> TopoField {
    TopoField { label: Some(label.into()), name: name.into(), unit, decimals, kind: "num", flag_text: "" }
}

/// A canvas-style labelled lamp.
pub(super) fn llamp(label: &str, name: impl Into<String>) -> TopoField {
    TopoField { label: Some(label.into()), name: name.into(), unit: "", decimals: 0, kind: "lamp", flag_text: "" }
}

pub(super) struct TopoNode {
    pub id: String,
    pub title: String,
    pub x: f32,
    pub y: f32,
    pub w: f32,
    /// "source" (a black box with a white edge when live) or "bus" (a green
    /// or amber powered/unpowered box), the same two shapes [`source`] and
    /// [`bus`] draw.
    pub kind: &'static str,
    pub fields: Vec<TopoField>,
}

/// Whether a link is live: every gate in `All` must hold, any one in `Any`.
pub(super) enum Gate {
    /// The variable reads as set (non-zero).
    On(String),
    /// The variable reads as clear (zero, or missing).
    Off(String),
    /// The variable's value is above a threshold.
    Gt(String, f64),
    /// The variable's magnitude is above a threshold (a current either way).
    AbsGt(String, f64),
    All(Vec<Gate>),
    Any(Vec<Gate>),
}

pub(super) struct TopoLink {
    pub points: Vec<(f32, f32)>,
    pub gate: Gate,
    /// "wire" (electrical) or "pipe" (fluid), [`wire`] and [`pipe`]'s colours.
    pub kind: &'static str,
}

pub(super) struct Topology {
    pub design_w: f32,
    pub design_h: f32,
    pub nodes: Vec<TopoNode>,
    pub links: Vec<TopoLink>,
}

/// A node's height in design units at this many field lines: a title line
/// plus one line per field, close enough to how tall [`source`] draws at
/// X-Plane's own font size that a link lands on the box's edge.
pub(super) fn box_h(lines: usize) -> f32 {
    22. + lines as f32 * 16.
}

/// Add a node and return its centre x, top y and bottom y in design units.
pub(super) fn node(nodes: &mut Vec<TopoNode>, id: &str, title: &str, x: f32, y: f32, w: f32, kind: &'static str, fields: Vec<TopoField>) -> (f32, f32, f32) {
    let bottom = y + box_h(fields.len());
    nodes.push(TopoNode { id: id.into(), title: title.into(), x, y, w, kind, fields });
    (x + w / 2., y, bottom)
}

pub(super) fn wlink(points: Vec<(f32, f32)>, gate: Gate) -> TopoLink {
    TopoLink { points, gate, kind: "wire" }
}

pub(super) fn plink(points: Vec<(f32, f32)>, gate: Gate) -> TopoLink {
    TopoLink { points, gate, kind: "pipe" }
}

/// The electrical network as data: the same sources, buses and wiring
/// [`draw`] paints, generators and external power feeding the AC buses,
/// AC 1 to AC ESS normally and AC 4 in alternate, the emergency generator
/// backing AC ESS, the four transformer rectifiers, the DC buses, batteries
/// and hot buses, and the static inverter.
pub(super) fn topology() -> Topology {
    let mut nodes = Vec::new();
    let mut links = Vec::new();
    let apu_normal_names = ["A32NX_ELEC_APU_GEN_1_POTENTIAL_NORMAL", "A32NX_ELEC_APU_GEN_2_POTENTIAL_NORMAL"];
    let apu_live_gate = || Gate::Any(apu_normal_names.iter().map(|n| Gate::On((*n).into())).collect());

    // External power and the APU generators, along the top.
    let mut ext = Vec::with_capacity(4);
    for (i, &cx) in AC_CENTRES.iter().enumerate() {
        let n = i + 1;
        ext.push(node(
            &mut nodes,
            &format!("ext{n}"),
            &format!("EXT {n}"),
            cx - 60.,
            6.,
            120.,
            "source",
            vec![tflag(format!("A32NX_EXT_PWR_AVAIL:{n}"), "AVAIL"), tflag(format!("A32NX_OVHD_ELEC_EXT_PWR_{n}_PB_IS_ON"), "ON")],
        ));
    }
    let apu: Vec<(f32, f32, f32)> = [("APU GEN A", 1, 395.), ("APU GEN B", 2, 505.)]
        .iter()
        .map(|&(title, n, x)| {
            let g = format!("A32NX_ELEC_APU_GEN_{n}");
            node(
                &mut nodes,
                &format!("apu_gen_{n}"),
                title,
                x,
                6.,
                100.,
                "source",
                vec![tfield(format!("{g}_POTENTIAL"), "V", 0), tfield(format!("{g}_FREQUENCY"), "Hz", 0), tfield(format!("{g}_LOAD"), "%", 0)],
            )
        })
        .collect();

    // The AC buses.
    let mut ac = Vec::with_capacity(4);
    for (i, &cx) in AC_CENTRES.iter().enumerate() {
        let n = i + 1;
        ac.push(node(&mut nodes, &format!("ac{n}"), &format!("AC {n}"), cx - 70., 112., 140., "bus", Vec::new()));
    }
    let ess = node(&mut nodes, "ac_ess", "AC ESS", 430., 112., 140., "bus", Vec::new());
    let shed = node(&mut nodes, "ac_ess_shed", "AC ESS SHED", 430., 176., 140., "bus", Vec::new());

    // Generators below their buses, the emergency generator below AC ESS.
    let mut gens = Vec::with_capacity(4);
    for (i, &cx) in AC_CENTRES.iter().enumerate() {
        let n = i + 1;
        let g = format!("A32NX_ELEC_ENG_GEN_{n}");
        gens.push(node(
            &mut nodes,
            &format!("gen{n}"),
            &format!("GEN {n}"),
            cx - 65.,
            214.,
            130.,
            "source",
            vec![tfield(format!("{g}_POTENTIAL"), "V", 0), tfield(format!("{g}_FREQUENCY"), "Hz", 0), tfield(format!("{g}_LOAD"), "%", 0)],
        ));
    }
    let emer = node(
        &mut nodes,
        "emer_gen",
        "EMER GEN",
        440.,
        214.,
        120.,
        "source",
        vec![tfield("A32NX_ELEC_EMER_GEN_POTENTIAL", "V", 0), tfield("A32NX_ELEC_EMER_GEN_FREQUENCY", "Hz", 0)],
    );

    // AC wiring.
    let tie_y = 94.;
    for (i, apu_gen) in apu.iter().enumerate() {
        links.push(wlink(vec![(apu_gen.0, apu_gen.2), (apu_gen.0, tie_y)], Gate::On(apu_normal_names[i].into())));
    }
    links.push(wlink(vec![(130., tie_y), (930., tie_y)], apu_live_gate()));
    for i in 0..4 {
        let n = i + 1;
        let bus_on = || Gate::On(format!("A32NX_ELEC_AC_{n}_BUS_IS_POWERED"));
        let (cx, top, _) = ac[i];
        let offset = 30.;
        links.push(wlink(
            vec![(cx - offset, ext[i].2), (cx - offset, top)],
            Gate::All(vec![Gate::On(format!("A32NX_EXT_PWR_AVAIL:{n}")), Gate::On(format!("A32NX_OVHD_ELEC_EXT_PWR_{n}_PB_IS_ON")), bus_on()]),
        ));
        links.push(wlink(vec![(cx + offset, tie_y), (cx + offset, top)], Gate::All(vec![apu_live_gate(), bus_on()])));
        links.push(wlink(vec![(cx, gens[i].1), (cx, ac[i].2)], Gate::All(vec![Gate::On(format!("A32NX_ELEC_ENG_GEN_{n}_POTENTIAL_NORMAL")), bus_on()])));
    }
    let ac1_on = Gate::On("A32NX_ELEC_AC_1_BUS_IS_POWERED".into());
    let ac4_on = Gate::On("A32NX_ELEC_AC_4_BUS_IS_POWERED".into());
    let ess_on = || Gate::On("A32NX_ELEC_AC_ESS_BUS_IS_POWERED".into());
    links.push(wlink(vec![(140., ac[0].2), (140., 150.), (470., 150.), (470., ess.2)], Gate::All(vec![ac1_on, ess_on()])));
    // The alternate feed only lights when AC 1 is not already feeding AC ESS.
    links.push(wlink(
        vec![(860., ac[3].2), (860., 160.), (530., 160.), (530., ess.2)],
        Gate::All(vec![ac4_on, ess_on(), Gate::Off("A32NX_ELEC_AC_1_BUS_IS_POWERED".into())]),
    ));
    links.push(wlink(vec![(ess.0, ess.2), (ess.0, shed.1)], Gate::All(vec![ess_on(), Gate::On("A32NX_ELEC_AC_ESS_SHED_BUS_IS_POWERED".into())])));
    links.push(wlink(vec![(emer.0, emer.1), (emer.0, shed.2)], Gate::All(vec![Gate::Gt("A32NX_ELEC_EMER_GEN_POTENTIAL".into(), 50.), ess_on()])));

    // Transformer rectifiers, each with the AC bus it draws from.
    let trs = [("TR 1", 1, 210., "A32NX_ELEC_AC_2_BUS_IS_POWERED"), ("ESS TR", 3, 500., "A32NX_ELEC_AC_ESS_BUS_IS_POWERED"), ("TR 2", 2, 700., "A32NX_ELEC_AC_3_BUS_IS_POWERED"), ("APU TR", 4, 900., "A32NX_ELEC_AC_4_BUS_IS_POWERED")];
    let mut tr_rects = Vec::with_capacity(4);
    for &(title, n, cx, from) in &trs {
        let t = format!("A32NX_ELEC_TR_{n}");
        let rect = node(
            &mut nodes,
            &format!("tr{n}"),
            title,
            cx - 60.,
            324.,
            120.,
            "source",
            vec![tfield(format!("{t}_POTENTIAL"), "V", 1), tfield(format!("{t}_CURRENT"), "A", 0)],
        );
        links.push(wlink(vec![(rect.0, 310.), (rect.0, rect.1)], Gate::On(from.into())));
        tr_rects.push((n, rect));
    }

    // DC buses.
    let dc = [
        node(&mut nodes, "dc1", "DC 1", 130., 404., 160., "bus", Vec::new()),
        node(&mut nodes, "dc_ess", "DC ESS", 420., 404., 160., "bus", Vec::new()),
        node(&mut nodes, "dc2", "DC 2", 620., 404., 160., "bus", Vec::new()),
        node(&mut nodes, "dc_apu", "DC APU", 820., 404., 160., "bus", Vec::new()),
    ];
    for (i, &(n, rect)) in tr_rects.iter().enumerate() {
        links.push(wlink(
            vec![(rect.0, rect.2), (rect.0, dc[i].1)],
            Gate::All(vec![Gate::Gt(format!("A32NX_ELEC_TR_{n}_POTENTIAL"), 20.), Gate::On(format!("A32NX_ELEC_DC_{}_BUS_IS_POWERED", dc_suffix(i)))]),
        ));
    }

    // Batteries under their buses, each over its hot bus.
    let bats = [("BAT 1", 1, 130.), ("BAT ESS", 3, 420.), ("BAT 2", 2, 620.), ("BAT APU", 4, 820.)];
    let hots = [("HOT 1", 1), ("HOT ESS", 3), ("HOT 2", 2), ("HOT APU", 4)];
    for (i, &(title, n, x)) in bats.iter().enumerate() {
        let b = format!("A32NX_ELEC_BAT_{n}");
        let rect = node(
            &mut nodes,
            &format!("bat{n}"),
            title,
            x,
            454.,
            160.,
            "source",
            vec![tfield(format!("{b}_POTENTIAL"), "V", 2), tfield(format!("{b}_CURRENT"), "A", 1)],
        );
        links.push(wlink(
            vec![(rect.0, rect.1), (rect.0, dc[i].2)],
            Gate::All(vec![Gate::AbsGt(format!("{b}_CURRENT"), 0.5), Gate::On(format!("A32NX_ELEC_DC_{}_BUS_IS_POWERED", dc_suffix(i)))]),
        ));
        let (hot_title, hot_n) = hots[i];
        let hot = node(&mut nodes, &format!("hot{hot_n}"), hot_title, x + 20., 532., 120., "bus", Vec::new());
        links.push(wlink(vec![(rect.0, rect.2), (rect.0, hot.1)], Gate::On(format!("A32NX_ELEC_DC_HOT_{hot_n}_BUS_IS_POWERED"))));
    }

    // The static inverter, fed from the essential side.
    let inv = node(
        &mut nodes,
        "stat_inv",
        "STAT INV",
        300.,
        454.,
        110.,
        "source",
        vec![tfield("A32NX_ELEC_STAT_INV_POTENTIAL", "V", 0), tfield("A32NX_ELEC_STAT_INV_FREQUENCY", "Hz", 0)],
    );
    links.push(wlink(
        vec![(440., dc[1].2), (440., 442.), (inv.0, 442.), (inv.0, inv.1)],
        Gate::Gt("A32NX_ELEC_STAT_INV_POTENTIAL".into(), 20.),
    ));

    // The two network-wide switches, as a small key node.
    nodes.push(TopoNode {
        id: "elec_key".into(),
        title: "OVERHEAD".into(),
        x: 620.,
        y: 566.,
        w: 380.,
        kind: "source",
        fields: vec![tflag("A32NX_OVHD_ELEC_BUS_TIE_PB_IS_AUTO", "BUS TIE AUTO"), tflag("A32NX_ELEC_GALLEY_IS_SHED", "GALLEY SHED")],
    });

    Topology { design_w: DESIGN_W, design_h: DESIGN_H, nodes, links }
}

/// The DC bus name suffix at a battery's own index (1, ESS, 2, APU).
fn dc_suffix(i: usize) -> &'static str {
    ["1", "ESS", "2", "APU"][i]
}

/// A canvas [`Field`] as a topology field, so a page with no hand-drawn
/// synoptic (bleed, air conditioning, pressurisation, gear and brakes, air
/// data, fire, radios) can still show as a live diagram: one node per box,
/// laid out in a flow, values updated the same way [`draw`]'s nodes are.
pub(super) fn field_to_topo(f: &super::canvas::Field) -> TopoField {
    match f.show {
        super::canvas::Show::Lamp => llamp(&f.label, f.name.clone()),
        super::canvas::Show::Num(d) => lfield(&f.label, f.name.clone(), f.unit, d),
        super::canvas::Show::Arinc(d) => TopoField { label: Some(f.label.clone()), name: f.name.clone(), unit: f.unit, decimals: d, kind: "arinc", flag_text: "" },
    }
}

/// A live diagram for a page that only has boxes of fields: each box
/// becomes a node, flowing left to right and wrapping, the same way
/// [`super::pages::flow`] lays boxes out (no links — there is no hand-picked
/// wiring for these pages, only the fields FlyByWire publishes).
pub(super) fn auto_topology(groups: &[super::canvas::Group]) -> Topology {
    let (gap, width) = (14., 230.);
    let (mut x, mut y, mut row_h): (f32, f32, f32) = (gap, gap, 0.);
    let mut nodes = Vec::new();
    for (i, g) in groups.iter().enumerate() {
        let fields: Vec<TopoField> = g.fields.iter().map(field_to_topo).collect();
        if fields.is_empty() {
            continue;
        }
        let h = box_h(fields.len());
        if x + width > DESIGN_W - gap && x > gap {
            x = gap;
            y += row_h + gap;
            row_h = 0.;
        }
        nodes.push(TopoNode { id: format!("g{i}"), title: g.title.clone(), x, y, w: width, kind: "source", fields });
        x += width + gap;
        row_h = row_h.max(h);
    }
    let design_h = (y + row_h + gap).max(300.);
    Topology { design_w: DESIGN_W, design_h, nodes, links: Vec::new() }
}
