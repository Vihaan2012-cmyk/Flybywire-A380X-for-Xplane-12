//! Drawing for the study pages.
//!
//! A page is laid out on a fixed design sheet, a thousand units wide and six
//! hundred tall, and scaled into whatever window it opens in. Text stays at
//! X-Plane's own font size, so boxes that hold text size themselves in pixels
//! from the lines they carry. X-Plane does not clip what a window draws, so
//! everything here clips itself to the page area by hand.
//!
//! Readings come from the snapshot the flight loop leaves behind. A field
//! whose variable the simulation does not hold says "not modelled"; a
//! variable nothing has ever written carries an amber tick, so a zero is
//! never passed off as a measurement.

use std::ffi::c_int;

use crate::xp::{Xplm, FONT_BASIC, FONT_PROPORTIONAL};
use crate::Snapshot;

pub type Rgba = [f32; 4];
pub type Ink = [f32; 3];

/// The design sheet every page is laid out on.
pub const DESIGN_W: f32 = 1000.;
pub const DESIGN_H: f32 = 600.;

/// What a click on something drawn does.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Action {
    /// Follow the variable at this place in the snapshot.
    Follow(usize),
    /// Back out of a followed variable.
    Back,
    /// Show another page in the same window.
    Page(usize),
    /// Press one of the plugin's own commands.
    Press(crate::published::Command),
    /// Arm or clear a failure by id.
    ToggleFailure(u64),
    /// Pull or reset a circuit breaker by circuit number.
    ToggleBreaker(usize),
    /// Run an X-Plane command by name.
    Command(&'static str),
    /// Refill the oxygen bottles and stow the masks.
    ServiceOxygen,
    /// Switch a schematic page between its schematic and its physics.
    TogglePhysics,
    /// Write one of FlyByWire's own variables, for the loadsheet page's
    /// boarding and SimBrief buttons. The name carries no aircraft prefix,
    /// the same as `study::web`'s queued writes.
    WriteVariable(&'static str, i32),
}

/// A clickable area the last frame drew.
pub struct Hit {
    pub left: c_int,
    pub top: c_int,
    pub right: c_int,
    pub bottom: c_int,
    pub action: Action,
}

/// How a field's reading is shown.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Show {
    /// A number to this many decimals.
    Num(usize),
    /// A lamp: lit when the variable is not zero.
    Lamp,
    /// An ARINC 429 word as FlyByWire packs it: the value, and whether the
    /// computer sending it vouches for it.
    Arinc(usize),
}

/// One data field: its label, the variable behind it, and its unit.
pub struct Field {
    pub label: String,
    pub name: String,
    pub unit: &'static str,
    pub show: Show,
}

pub fn num(label: &str, name: impl Into<String>, unit: &'static str, decimals: usize) -> Field {
    Field { label: label.into(), name: name.into(), unit, show: Show::Num(decimals) }
}

pub fn lamp(label: &str, name: impl Into<String>) -> Field {
    Field { label: label.into(), name: name.into(), unit: "", show: Show::Lamp }
}

pub fn arinc(label: &str, name: impl Into<String>, unit: &'static str, decimals: usize) -> Field {
    Field { label: label.into(), name: name.into(), unit, show: Show::Arinc(decimals) }
}

/// A titled box of fields.
pub struct Group {
    pub title: String,
    pub theme: Theme,
    pub fields: Vec<Field>,
    /// Leave out the fields the simulation does not hold, for lists whose
    /// length depends on the aircraft (brake temperatures, tanks).
    pub hide_missing: bool,
}

pub fn group(title: &str, theme: Theme, fields: Vec<Field>) -> Group {
    Group { title: title.into(), theme, fields, hide_missing: false }
}

/// The colours of one kind of box.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Theme {
    pub title: Rgba,
    pub body: Rgba,
    pub edge: Rgba,
    pub ink: Ink,
    pub value: Ink,
}

/// The panel's palette: the slate, lavender and pastel boxes of a study
/// panel, and the black, green and amber of the aircraft's own synoptics.
pub mod palette {
    use super::{Ink, Rgba, Theme};

    pub const WINDOW: Rgba = [0.17, 0.19, 0.24, 1.];
    pub const BAND: Rgba = [0.33, 0.37, 0.46, 1.];
    pub const SLATE: Rgba = [0.55, 0.60, 0.70, 1.];
    pub const PERIWINKLE: Rgba = [0.69, 0.68, 0.97, 1.];
    pub const EDGE: Rgba = [0.12, 0.12, 0.19, 1.];
    pub const HOVER: Rgba = [1., 1., 1., 0.38];
    pub const CHIP: Rgba = [0.78, 0.77, 0.80, 1.];
    pub const CHIP_INK: Ink = [0.34, 0.34, 0.40];
    pub const UNFED: Rgba = [0.95, 0.62, 0.12, 1.];
    pub const LAMP_ON: Rgba = [0.20, 0.86, 0.32, 1.];
    pub const LAMP_OFF: Rgba = [0.40, 0.42, 0.48, 1.];
    pub const LEADER: Rgba = [0.24, 0.26, 0.33, 1.];
    pub const INK: Ink = [0.03, 0.03, 0.08];
    pub const INK_LIGHT: Ink = [0.95, 0.96, 0.99];
    pub const INK_DIM: Ink = [0.36, 0.37, 0.44];

    const fn theme(title: Rgba, body: Rgba) -> Theme {
        Theme { title, body, edge: EDGE, ink: INK, value: INK }
    }

    pub const STATION: Theme = theme([0.74, 0.74, 0.89, 1.], [0.87, 0.87, 0.96, 1.]);
    pub const MAGENTA: Theme = theme([0.88, 0.40, 0.88, 1.], [0.93, 0.71, 0.93, 1.]);
    pub const GREEN: Theme = theme([0.38, 0.84, 0.43, 1.], [0.64, 0.91, 0.66, 1.]);
    pub const BLUE: Theme = theme([0.44, 0.67, 0.93, 1.], [0.69, 0.82, 0.96, 1.]);
    pub const YELLOW: Theme = theme([0.97, 0.92, 0.28, 1.], [0.98, 0.96, 0.64, 1.]);
    pub const ORANGE: Theme = theme([0.98, 0.60, 0.27, 1.], [0.99, 0.79, 0.57, 1.]);
    pub const CYAN: Theme = theme([0.36, 0.87, 0.94, 1.], [0.65, 0.93, 0.96, 1.]);
    pub const RED: Theme = theme([0.92, 0.34, 0.34, 1.], [0.96, 0.65, 0.65, 1.]);
    pub const BUTTON: Theme = theme([0.74, 0.88, 0.50, 1.], [0.74, 0.88, 0.50, 1.]);
    pub const BUTTON_ON: Theme = theme([0.46, 0.72, 0.28, 1.], [0.46, 0.72, 0.28, 1.]);
    pub const NOTE: Theme = theme([0.83, 0.86, 0.97, 1.], [0.83, 0.86, 0.97, 1.]);

    // The aircraft's own synoptic colours.
    pub const ECAM_GROUND: Rgba = [0.02, 0.03, 0.05, 1.];
    pub const ECAM_EDGE: Rgba = [0.78, 0.80, 0.86, 1.];
    pub const ECAM_LIVE: Rgba = [0.16, 0.92, 0.36, 1.];
    pub const ECAM_DEAD: Rgba = [0.34, 0.36, 0.42, 1.];
    pub const ECAM_FLUID: Rgba = [0.30, 0.74, 1.00, 1.];
    pub const ECAM_WHITE: Ink = [0.90, 0.92, 0.95];
    pub const ECAM_GREEN: Ink = [0.20, 0.95, 0.40];
    pub const ECAM_AMBER: Ink = [1.00, 0.66, 0.12];
    pub const ECAM_CYAN: Ink = [0.32, 0.85, 1.00];
    pub const ECAM_GREY: Ink = [0.50, 0.52, 0.58];
    pub const ECAM_YELLOW: Ink = [1.00, 0.93, 0.25];
    pub const ECAM_BOX: Theme = Theme {
        title: [0.07, 0.08, 0.11, 1.],
        body: [0.03, 0.04, 0.06, 1.],
        edge: ECAM_EDGE,
        ink: ECAM_WHITE,
        value: ECAM_GREEN,
    };
}

use palette as p;

/// A theme's semantic name, for anything (like the web Study tab) that draws
/// its own boxes and wants the same colour language as the XPLM windows
/// without carrying raw RGBA around. Falls back to "station" for a theme
/// that is not one of the named palette constants.
pub(super) fn theme_name(theme: &Theme) -> &'static str {
    match *theme {
        t if t == p::STATION => "station",
        t if t == p::MAGENTA => "magenta",
        t if t == p::GREEN => "green",
        t if t == p::BLUE => "blue",
        t if t == p::YELLOW => "yellow",
        t if t == p::ORANGE => "orange",
        t if t == p::CYAN => "cyan",
        t if t == p::RED => "red",
        t if t == p::NOTE => "note",
        t if t == p::ECAM_BOX => "ecam",
        _ => "station",
    }
}

/// A variable's reading this frame.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Reading {
    /// The simulation does not hold this variable.
    Missing,
    Value { index: usize, value: f64, source: u8 },
}

impl Reading {
    /// The number, when there is one to use.
    pub fn live(&self) -> Option<f64> {
        match *self {
            Reading::Value { value, .. } if value.is_finite() => Some(value),
            _ => None,
        }
    }

    /// Whether the variable reads as set. A missing or unreadable one does not.
    pub fn on(&self) -> bool {
        self.live().is_some_and(|v| v != 0.)
    }
}

/// An ARINC 429 word unpacked: FlyByWire stores the value's float bits in the
/// low half and the sign/status matrix in the high half.
pub fn unpack_arinc(packed: f64) -> (f32, u32) {
    let bits = packed as u64;
    (f32::from_bits(bits as u32), ((bits >> 32) & 0b11) as u32)
}

/// What an ARINC status says about the value, as its abbreviation.
pub fn arinc_status(ssm: u32) -> Option<&'static str> {
    match ssm {
        0 => Some("FW"),
        1 => Some("NCD"),
        2 => Some("FT"),
        _ => None,
    }
}

/// Whether a number is an ARINC 429 word as FlyByWire packs it: a whole
/// number holding a 32 bit float's bits, with the status in the two bits
/// above. A float of magnitude two or more has bits from 0x40000000 up, so
/// its word is at least about a billion even when the status bits are zero
/// (failure warning); no reading this aircraft carries is a whole number that
/// large. FlyByWire sends many readings this way, so a field is decoded
/// whenever its number is one, whatever the page expected.
pub fn looks_packed(value: f64) -> bool {
    value.is_finite() && (1.0e9..17_179_869_184.).contains(&value) && value == value.trunc()
}

/// A reading written out: whole where it is whole, never in exponent form.
/// An ARINC word shows its value and, when the sender does not vouch for
/// it, the reason: FW failure warning, NCD no computed data, FT test.
pub fn format_value(value: f64, show: Show, unit: &str) -> String {
    if !value.is_finite() {
        return "no reading".into();
    }
    let with_unit = |n: String| if unit.is_empty() { n } else { format!("{n} {unit}") };
    let show = match show {
        Show::Num(decimals) if looks_packed(value) => Show::Arinc(decimals),
        other => other,
    };
    match show {
        Show::Lamp => (if value != 0. { "ON" } else { "OFF" }).into(),
        Show::Num(decimals) => {
            if value.abs() >= 1e7 {
                with_unit(format!("{value:.0}"))
            } else {
                with_unit(format!("{value:.decimals$}"))
            }
        }
        Show::Arinc(decimals) => {
            let (v, ssm) = unpack_arinc(value);
            let shown = with_unit(format!("{v:.decimals$}"));
            match arinc_status(ssm) {
                Some(status) => format!("{shown} {status}"),
                None => shown,
            }
        }
    }
}

/// A number readable at a glance.
pub fn number(value: f64) -> String {
    if !value.is_finite() {
        return "no reading".into();
    }
    let size = value.abs();
    if value == value.trunc() && size < 1e9 {
        format!("{value:.0}")
    } else if size >= 1000. {
        format!("{value:.0}")
    } else if size >= 1. {
        format!("{value:.2}")
    } else {
        format!("{value:.4}")
    }
}

/// Clip a segment to a box, Cohen and Sutherland's way. The box is
/// `(left, top, right, bottom)` with y growing upwards.
pub fn clip_line(
    mut x1: f32,
    mut y1: f32,
    mut x2: f32,
    mut y2: f32,
    (l, t, r, b): (f32, f32, f32, f32),
) -> Option<(f32, f32, f32, f32)> {
    let code = |x: f32, y: f32| -> u8 {
        let mut c = 0;
        if x < l {
            c |= 1;
        }
        if x > r {
            c |= 2;
        }
        if y < b {
            c |= 4;
        }
        if y > t {
            c |= 8;
        }
        c
    };
    let (mut c1, mut c2) = (code(x1, y1), code(x2, y2));
    for _ in 0..8 {
        if c1 | c2 == 0 {
            return Some((x1, y1, x2, y2));
        }
        if c1 & c2 != 0 {
            return None;
        }
        let out = if c1 != 0 { c1 } else { c2 };
        let (x, y) = if out & 8 != 0 {
            (x1 + (x2 - x1) * (t - y1) / (y2 - y1), t)
        } else if out & 4 != 0 {
            (x1 + (x2 - x1) * (b - y1) / (y2 - y1), b)
        } else if out & 2 != 0 {
            (r, y1 + (y2 - y1) * (r - x1) / (x2 - x1))
        } else {
            (l, y1 + (y2 - y1) * (l - x1) / (x2 - x1))
        };
        if out == c1 {
            (x1, y1) = (x, y);
            c1 = code(x1, y1);
        } else {
            (x2, y2) = (x, y);
            c2 = code(x2, y2);
        }
    }
    None
}

fn dim(ink: Ink) -> Ink {
    [ink[0] * 0.55 + 0.2, ink[1] * 0.55 + 0.2, ink[2] * 0.55 + 0.22]
}

/// One frame's drawing into one page area.
pub struct Canvas<'a> {
    pub xp: &'a Xplm,
    pub snap: &'a Snapshot,
    /// The page area, `(left, top, right, bottom)`: nothing is drawn outside.
    pub clip: (c_int, c_int, c_int, c_int),
    scale: f32,
    ox: f32,
    oy: f32,
    pub char_w: c_int,
    pub line_h: c_int,
    pub mouse: (c_int, c_int),
    pub hits: &'a mut Vec<Hit>,
    /// The variable under the mouse, for the tooltip drawn over the page.
    pub tip: Option<usize>,
}

impl<'a> Canvas<'a> {
    pub fn new(
        xp: &'a Xplm,
        snap: &'a Snapshot,
        clip: (c_int, c_int, c_int, c_int),
        mouse: (c_int, c_int),
        hits: &'a mut Vec<Hit>,
    ) -> Self {
        let (l, t, r, b) = clip;
        let (w, h) = ((r - l).max(1) as f32, (t - b).max(1) as f32);
        let scale = (w / DESIGN_W).min(h / DESIGN_H).max(0.05);
        let ox = l as f32 + (w - DESIGN_W * scale) / 2.;
        let oy = t as f32 - (h - DESIGN_H * scale) / 2.;
        let (char_w, font_h) = xp.font_size(FONT_BASIC);
        Self {
            xp,
            snap,
            clip,
            scale,
            ox,
            oy,
            char_w,
            line_h: font_h + 4,
            mouse,
            hits,
            tip: None,
        }
    }

    // Design sheet to pixels.

    pub fn x(&self, x: f32) -> c_int {
        (self.ox + x * self.scale).round() as c_int
    }

    pub fn y(&self, y: f32) -> c_int {
        (self.oy - y * self.scale).round() as c_int
    }

    pub fn px(&self, d: f32) -> c_int {
        (d * self.scale).round() as c_int
    }

    // Readings.

    pub fn read(&self, name: &str) -> Reading {
        match self.snap.find(name) {
            None => Reading::Missing,
            Some(index) => Reading::Value {
                index,
                value: self.snap.values.get(index).copied().unwrap_or(f64::NAN),
                source: self.snap.sources.get(index).copied().unwrap_or(0),
            },
        }
    }

    pub fn on(&self, name: &str) -> bool {
        self.read(name).on()
    }

    pub fn value(&self, name: &str) -> Option<f64> {
        self.read(name).live()
    }

    // Pixel primitives, all clipped to the page.

    fn clip_box(&self, l: c_int, t: c_int, r: c_int, b: c_int) -> Option<(c_int, c_int, c_int, c_int)> {
        let (cl, ct, cr, cb) = self.clip;
        let (l, t, r, b) = (l.max(cl), t.min(ct), r.min(cr), b.max(cb));
        (l < r && b < t).then_some((l, t, r, b))
    }

    pub fn fill_px(&self, l: c_int, t: c_int, r: c_int, b: c_int, colour: Rgba) {
        if let Some((l, t, r, b)) = self.clip_box(l, t, r, b) {
            self.xp.fill(l, t, r, b, colour);
        }
    }

    pub fn line_px(&self, x1: c_int, y1: c_int, x2: c_int, y2: c_int, colour: Rgba, width: f32) {
        let (l, t, r, b) = self.clip;
        let bounds = (l as f32, t as f32, r as f32, b as f32);
        if let Some((a, b1, c, d)) = clip_line(x1 as f32, y1 as f32, x2 as f32, y2 as f32, bounds) {
            self.xp.line(a as c_int, b1 as c_int, c as c_int, d as c_int, colour, width);
        }
    }

    pub fn frame_px(&self, l: c_int, t: c_int, r: c_int, b: c_int, colour: Rgba, width: f32) {
        self.line_px(l, t, r, t, colour, width);
        self.line_px(r, t, r, b, colour, width);
        self.line_px(r, b, l, b, colour, width);
        self.line_px(l, b, l, t, colour, width);
    }

    /// Text with its baseline at `y`, cut to what fits before the page edge.
    pub fn text_px(&self, x: c_int, y: c_int, ink: Ink, font: c_int, s: &str) {
        let (cl, ct, cr, cb) = self.clip;
        if y < cb + 1 || y + self.line_h - 4 > ct || x < cl || x >= cr {
            return;
        }
        let room = ((cr - x) / self.char_w.max(1)).max(0) as usize;
        if room == 0 {
            return;
        }
        if s.chars().count() > room {
            let cut: String = s.chars().take(room).collect();
            self.xp.text(x, y, ink, font, &cut);
        } else {
            self.xp.text(x, y, ink, font, s);
        }
    }

    pub fn text_width(&self, s: &str) -> c_int {
        s.chars().count() as c_int * self.char_w
    }

    /// Text centred on `x`.
    pub fn text_centred_px(&self, x: c_int, y: c_int, ink: Ink, font: c_int, s: &str) {
        self.text_px(x - self.text_width(s) / 2, y, ink, font, s);
    }

    pub fn inside(&self, (x, y): (c_int, c_int)) -> bool {
        let (l, t, r, b) = self.clip;
        x >= l && x <= r && y >= b && y <= t
    }

    pub fn hovering(&self, l: c_int, t: c_int, r: c_int, b: c_int) -> bool {
        let (x, y) = self.mouse;
        self.inside(self.mouse) && x >= l && x <= r && y <= t && y >= b
    }

    /// Register a clickable area; the part outside the page cannot be clicked.
    pub fn hit_px(&mut self, l: c_int, t: c_int, r: c_int, b: c_int, action: Action) {
        if let Some((left, top, right, bottom)) = self.clip_box(l, t, r, b) {
            self.hits.push(Hit { left, top, right, bottom, action });
        }
    }

    // Design-sheet primitives.

    pub fn fill(&self, x: f32, y: f32, w: f32, h: f32, colour: Rgba) {
        self.fill_px(self.x(x), self.y(y), self.x(x + w), self.y(y + h), colour);
    }

    pub fn frame(&self, x: f32, y: f32, w: f32, h: f32, colour: Rgba, width: f32) {
        self.frame_px(self.x(x), self.y(y), self.x(x + w), self.y(y + h), colour, width);
    }

    pub fn line(&self, x1: f32, y1: f32, x2: f32, y2: f32, colour: Rgba, width: f32) {
        self.line_px(self.x(x1), self.y(y1), self.x(x2), self.y(y2), colour, width);
    }

    /// A filled convex shape, drawn only when the whole of it is on the page:
    /// a shape cannot be cut by hand the way a line or a box can.
    pub fn poly(&self, points: &[(f32, f32)], colour: Rgba) {
        let px: Vec<(f32, f32)> = points
            .iter()
            .map(|&(x, y)| (self.ox + x * self.scale, self.oy - y * self.scale))
            .collect();
        let (l, t, r, b) = self.clip;
        let within = px
            .iter()
            .all(|&(x, y)| x >= l as f32 && x <= r as f32 && y >= b as f32 && y <= t as f32);
        if within {
            self.xp.poly(&px, colour);
        }
    }

    /// A shape's outline.
    pub fn outline(&self, points: &[(f32, f32)], colour: Rgba, width: f32) {
        for i in 0..points.len() {
            let a = points[i];
            let b = points[(i + 1) % points.len()];
            self.line(a.0, a.1, b.0, b.1, colour, width);
        }
    }

    /// Text whose top sits at design height `y`.
    pub fn text(&self, x: f32, y: f32, ink: Ink, font: c_int, s: &str) {
        self.text_px(self.x(x), self.y(y) - self.line_h + 4, ink, font, s);
    }

    /// A round marker with a label in it, as station numbers are drawn.
    pub fn marker(&self, x: f32, y: f32, label: &str) {
        let radius = (self.text_width(label) / 2 + 6).max(9) as f32 / self.scale;
        let ring: Vec<(f32, f32)> = (0..16)
            .map(|i| {
                let a = i as f32 / 16. * std::f32::consts::TAU;
                (x + a.cos() * radius, y + a.sin() * radius)
            })
            .collect();
        self.poly(&ring, [0.84, 0.84, 0.90, 1.]);
        self.outline(&ring, p::EDGE, 1.);
        self.text_centred_px(self.x(x), self.y(y) - self.line_h / 2 + 3, p::INK, FONT_BASIC, label);
    }

    /// An arrow pointing right, for air leaving the engine.
    pub fn arrow(&self, x: f32, y: f32, len: f32, half: f32, colour: Rgba) {
        let head = half * 1.6;
        self.poly(
            &[(x, y - half * 0.5), (x + len - head, y - half * 0.5), (x + len - head, y + half * 0.5), (x, y + half * 0.5)],
            colour,
        );
        self.poly(&[(x + len - head, y - half), (x + len, y), (x + len - head, y + half)], colour);
        self.outline(
            &[
                (x, y - half * 0.5),
                (x + len - head, y - half * 0.5),
                (x + len - head, y - half),
                (x + len, y),
                (x + len - head, y + half),
                (x + len - head, y + half * 0.5),
                (x, y + half * 0.5),
            ],
            p::EDGE,
            1.,
        );
    }

    // Widgets.

    /// Height in pixels of a titled box holding this many rows.
    pub fn box_height(&self, rows: usize) -> c_int {
        (self.line_h + 6) + rows as c_int * self.line_h + 6
    }

    /// One data field on one row of a box. The row's top is `top`.
    pub fn field_row(&mut self, field: &Field, l: c_int, r: c_int, top: c_int, theme: &Theme) {
        let lh = self.line_h;
        let bottom = top - lh;
        let base = bottom + 4;
        let reading = self.read(&field.name);

        if let Reading::Value { index, .. } = reading {
            if self.hovering(l, top, r, bottom) {
                self.fill_px(l + 1, top, r - 1, bottom, p::HOVER);
                self.tip = Some(index);
            }
        }
        // The reading is never cut; a label too long to sit beside it is.
        let reading_chars = match reading {
            Reading::Missing => "not modelled".len(),
            Reading::Value { value, .. } => match field.show {
                Show::Lamp => 2,
                show => format_value(value, show, field.unit).chars().count(),
            },
        } as c_int;
        let label_room = ((r - l - 22) / self.char_w.max(1) - reading_chars - 1).max(0) as usize;
        if field.label.chars().count() > label_room {
            let cut: String = field.label.chars().take(label_room.saturating_sub(2)).collect();
            self.text_px(l + 8, base, theme.ink, FONT_BASIC, &format!("{cut}.."));
        } else {
            self.text_px(l + 8, base, theme.ink, FONT_BASIC, &field.label);
        }

        match reading {
            Reading::Missing => {
                let s = "not modelled";
                let w = self.text_width(s);
                self.fill_px(r - w - 12, top - 2, r - 4, bottom + 2, p::CHIP);
                self.text_px(r - w - 8, base, p::CHIP_INK, FONT_BASIC, s);
            }
            Reading::Value { index, value, source } => {
                if source == 0 {
                    self.fill_px(l + 2, top - 2, l + 5, bottom + 2, p::UNFED);
                }
                match field.show {
                    Show::Lamp => {
                        let size = (lh - 7).max(6);
                        let (lt, lr) = (top - (lh - size) / 2, r - 8);
                        let colour = if value.is_finite() && value != 0. { p::LAMP_ON } else { p::LAMP_OFF };
                        self.fill_px(lr - size, lt, lr, lt - size, colour);
                        self.frame_px(lr - size, lt, lr, lt - size, theme.edge, 1.);
                    }
                    Show::Num(_) | Show::Arinc(_) => {
                        let s = format_value(value, field.show, field.unit);
                        let w = self.text_width(&s);
                        let packed = matches!(field.show, Show::Arinc(_)) || looks_packed(value);
                        let flagged = !value.is_finite()
                            || source == 0
                            || packed && arinc_status(unpack_arinc(value).1).is_some();
                        let ink = if flagged { dim(theme.value) } else { theme.value };
                        self.text_px(r - w - 8, base, ink, FONT_BASIC, &s);
                    }
                }
                self.hit_px(l, top, r, bottom, Action::Follow(index));
            }
        }
    }

    /// A titled box of fields at a pixel position; returns its bottom.
    pub fn box_px(&mut self, l: c_int, top: c_int, width: c_int, title: &str, theme: &Theme, fields: &[&Field]) -> c_int {
        let r = l + width;
        let bottom = top - self.box_height(fields.len());
        let title_bottom = top - self.line_h - 6;
        self.fill_px(l, top, r, bottom, theme.body);
        self.fill_px(l, top, r, title_bottom, theme.title);
        self.line_px(l, title_bottom, r, title_bottom, theme.edge, 1.);
        self.frame_px(l, top, r, bottom, theme.edge, 1.);
        self.text_px(l + 8, title_bottom + 5, theme.ink, FONT_PROPORTIONAL, title);
        let mut y = title_bottom - 3;
        for field in fields {
            self.field_row(field, l, r, y, theme);
            y -= self.line_h;
        }
        bottom
    }

    /// A titled box at a design position, `w` design units wide.
    pub fn station(&mut self, x: f32, y: f32, w: f32, title: &str, theme: &Theme, fields: &[Field]) -> c_int {
        let refs: Vec<&Field> = fields.iter().collect();
        let (l, t, width) = (self.x(x), self.y(y), self.px(w));
        self.box_px(l, t, width, title, theme, &refs)
    }

    /// A box's width in pixels to fit its title and fields.
    pub fn fit_width(&self, title: &str, fields: &[&Field]) -> c_int {
        let widest = fields
            .iter()
            .map(|f| f.label.chars().count() + 3 + value_chars(f))
            .chain(std::iter::once(title.chars().count() + 3))
            .max()
            .unwrap_or(20) as c_int;
        widest * self.char_w + 18
    }

    /// A flat button; `lit` marks the one in use.
    pub fn button_px(&mut self, l: c_int, t: c_int, r: c_int, b: c_int, label: &str, lit: bool, action: Action) {
        let theme = if lit { p::BUTTON_ON } else { p::BUTTON };
        self.fill_px(l, t, r, b, theme.body);
        if self.hovering(l, t, r, b) {
            self.fill_px(l, t, r, b, p::HOVER);
        }
        self.frame_px(l, t, r, b, p::EDGE, 1.);
        let base = b + (t - b - self.line_h) / 2 + 4;
        self.text_centred_px((l + r) / 2, base, p::INK, FONT_BASIC, label);
        self.hit_px(l, t, r, b, action);
    }

    /// A horizontal gauge: the reading as a bar between two limits.
    #[allow(clippy::too_many_arguments)]
    pub fn gauge_px(&mut self, l: c_int, r: c_int, top: c_int, label: &str, name: &str, low: f64, high: f64, unit: &'static str) {
        let lh = self.line_h;
        let bottom = top - lh;
        let base = bottom + 4;
        let label_w = 18 * self.char_w;
        let value_w = 12 * self.char_w;
        let (bar_l, bar_r) = (l + label_w, r - value_w - 6);
        let reading = self.read(name);
        self.text_px(l + 6, base, p::ECAM_WHITE, FONT_BASIC, label);
        match reading {
            Reading::Missing => self.text_px(bar_l, base, p::ECAM_GREY, FONT_BASIC, "not modelled"),
            Reading::Value { index, value, source } => {
                if self.hovering(l, top, r, bottom) {
                    self.fill_px(l, top, r, bottom, [1., 1., 1., 0.10]);
                    self.tip = Some(index);
                }
                self.fill_px(bar_l, top - 4, bar_r, bottom + 4, [0.12, 0.13, 0.17, 1.]);
                self.frame_px(bar_l, top - 4, bar_r, bottom + 4, p::ECAM_DEAD, 1.);
                if value.is_finite() && bar_r > bar_l && high > low {
                    let at = ((value - low) / (high - low)).clamp(0., 1.);
                    let zero = ((0f64 - low) / (high - low)).clamp(0., 1.);
                    let span = (bar_r - bar_l) as f64;
                    let (a, b) = (bar_l + (zero * span) as c_int, bar_l + (at * span) as c_int);
                    let colour = if source == 0 { p::ECAM_DEAD } else { p::ECAM_LIVE };
                    self.fill_px(a.min(b), top - 6, a.max(b).max(a.min(b) + 2), bottom + 6, colour);
                }
                let s = format_value(value, Show::Num(2), unit);
                let ink = if source == 0 { p::ECAM_GREY } else { p::ECAM_GREEN };
                self.text_px(r - self.text_width(&s) - 4, base, ink, FONT_BASIC, &s);
                self.hit_px(l, top, r, bottom, Action::Follow(index));
            }
        }
    }
}

/// Characters a field's reading usually needs, to size its box.
fn value_chars(field: &Field) -> usize {
    match field.show {
        Show::Lamp => 3,
        Show::Num(d) => 7 + d + field.unit.len(),
        // Room for the status after the value.
        Show::Arinc(d) => 11 + d + field.unit.len(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const BOX: (f32, f32, f32, f32) = (0., 100., 100., 0.);

    #[test]
    fn a_line_inside_is_left_alone() {
        assert_eq!(clip_line(10., 10., 90., 90., BOX), Some((10., 10., 90., 90.)));
    }

    #[test]
    fn a_line_outside_is_dropped() {
        assert_eq!(clip_line(-50., 150., -10., 120., BOX), None);
    }

    #[test]
    fn a_line_crossing_the_edge_stops_at_it() {
        let (x1, y1, x2, y2) = clip_line(-50., 50., 50., 50., BOX).unwrap();
        assert_eq!((x1, y1, x2, y2), (0., 50., 50., 50.));
    }

    #[test]
    fn arinc_words_unpack_the_way_flybywire_packs_them() {
        // Normal operation carrying 250.5.
        let packed = (((0b11u64) << 32) | 250.5f32.to_bits() as u64) as f64;
        assert_eq!(unpack_arinc(packed), (250.5, 3));
        assert_eq!(format_value(packed, Show::Arinc(1), "kt"), "250.5 kt");
        // No computed data, as the idle air data computers send.
        assert_eq!(format_value(4_294_967_296., Show::Arinc(0), ""), "0 NCD");
        // A packed word in a plain number field is still decoded.
        let egt = (((0b11u64) << 32) | 540.0f32.to_bits() as u64) as f64;
        assert_eq!(format_value(egt, Show::Num(0), "C"), "540 C");
        assert!(looks_packed(egt));
        assert!(!looks_packed(1_091_586.5));
        // 1091586861 is about 9 C with the failure warning status, whose bits
        // are zero: the APU EGT a cold aircraft sends.
        let (v, ssm) = unpack_arinc(1_091_586_861.);
        assert!((v - 9.0).abs() < 0.05 && ssm == 0);
        assert_eq!(format_value(1_091_586_861., Show::Num(0), "C"), "9 C FW");
        // The largest plain reading, the moment of inertia, stays a number.
        assert!(!looks_packed(110_895_003.));
    }

    #[test]
    fn readings_never_use_exponents() {
        assert_eq!(format_value(0.000_012, Show::Num(2), "psi"), "0.00 psi");
        assert_eq!(format_value(31_133_584.7, Show::Num(0), ""), "31133585");
        assert_eq!(format_value(f64::NAN, Show::Num(1), "C"), "no reading");
        assert_eq!(number(123_456.7), "123457");
    }

    #[test]
    fn lamps_read_on_and_off() {
        assert_eq!(format_value(1., Show::Lamp, ""), "ON");
        assert_eq!(format_value(0., Show::Lamp, ""), "OFF");
    }
}
