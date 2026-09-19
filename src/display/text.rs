//! The instrument fonts: finding a face for a CSS family, laying text out,
//! and a glyph atlas the screens draw text from.
//!
//! Fonts are FlyByWire's own files, read at run time from the aircraft's
//! `html_ui/Fonts/fbw-a380x` folder (copied there from the MSFS package's
//! folder of the same name, docs/screens.md), never built into the plugin.
//! [`FACES`] says which file each CSS family is, and on which screens,
//! because FlyByWire's instruments reuse family names for different files:
//! `Ecam` is the EIS font on the PFD but the ISIS font on the standby
//! instrument, and `Digital` is one font on the FCU and another on the
//! battery display.
//!
//! Layout is advance widths plus the font's `kern` table. FlyByWire's
//! instrument fonts carry no GPOS kerning or Latin ligatures (checked with
//! ttf-parser: the EIS fonts' only layout feature is the optional `zero`),
//! so this is what a browser lays out too. `measure` and drawing share the
//! same layout, so what the instruments measure is what they get.

use std::collections::HashMap;
use std::path::Path;

use ab_glyph::{point, Font, FontVec, GlyphId, OutlineCurve, PxScale};

use super::path::{Affine, Path as Outline};
use super::stream::{Align, Baseline};

/// One font file, as offered to some screens under one family name.
pub struct Face {
    family: String,
    /// Lower-case screen ids, or empty for every screen.
    screens: Vec<String>,
    weight: u16,
    italic: bool,
    font: FontVec,
    units_per_em: f32,
    ascent: f32,
    descent: f32,
}

/// A laid-out run of text, in CSS pixels from the pen's start.
pub struct Run {
    pub face: usize,
    /// Glyph and the x of its origin.
    pub glyphs: Vec<(GlyphId, f64)>,
    pub width: f64,
}

/// One `@font-face` of FlyByWire's A380X instruments.
pub struct FaceDef {
    /// Screens it is declared for; empty for every screen.
    pub screens: &'static [&'static str],
    pub family: &'static str,
    pub weight: u16,
    /// Under `html_ui/Fonts/fbw-a380x`.
    pub file: &'static str,
}

/// The `@font-face` rules of fbw-a380x/src/systems/instruments/src (all
/// `font-style: normal`). Left out: the RMP's `A1000`
/// (`/Fonts/A1000/a1000.ttf`, RMP/style.scss:37), which is MSFS's own and
/// not in FlyByWire's package, and the EFB and OIT fonts.
pub static FACES: &[FaceDef] = &[
    // Common/definitions.scss:1, PFD/style.scss, ND/style.scss:8,
    // EWD/style.scss:25, SDv2/style.scss, MFD/pages/common/style.scss:4.
    FaceDef { screens: &[], family: "Ecam", weight: 400, file: "FBW-Display-EIS-A380.ttf" },
    // ND/style.scss:16, SDv2/style.scss, MFD/pages/common/style.scss:12.
    FaceDef { screens: &[], family: "FBW-Display-EIS-A380-SlashedZero", weight: 400, file: "FBW-Display-EIS-A380-SlashedZero.ttf" },
    // ND/style.scss:24.
    FaceDef { screens: &[], family: "NDChrono", weight: 400, file: "NDChrono.ttf" },
    // ISISlegacy/style.scss:3.
    FaceDef { screens: &["SCREEN_ISIS_1"], family: "Ecam", weight: 400, file: "ISISFontTemporary.ttf" },
    // FCU/definitions.scss:8 and :15.
    FaceDef { screens: &["FCU"], family: "Poppins-SemiBold", weight: 400, file: "Poppins-SemiBold.ttf" },
    FaceDef { screens: &["FCU"], family: "Digital", weight: 900, file: "A380X_FCU.ttf" },
    // BAT/style.scss:3.
    FaceDef { screens: &["BAT"], family: "Digital", weight: 900, file: "AirbusBAT.ttf" },
    // RTPI/style.scss:3.
    FaceDef { screens: &["RTPI"], family: "AirbusRTPI", weight: 100, file: "AirbusRTPI.ttf" },
    // fbw-common Clock/style.scss:1, with Clock/Clock.scss:4's path.
    FaceDef { screens: &["Clock"], family: "AirbusChronometer", weight: 400, file: "AirbusChronometer.ttf" },
    // RMP/style.scss:6-35.
    FaceDef { screens: &RMPS, family: "RMP-10", weight: 400, file: "FBW-Display-RMP-10.ttf" },
    FaceDef { screens: &RMPS, family: "RMP-11", weight: 400, file: "FBW-Display-RMP-11.ttf" },
    FaceDef { screens: &RMPS, family: "RMP-13", weight: 400, file: "FBW-Display-RMP-13.ttf" },
    FaceDef { screens: &RMPS, family: "RMP-16", weight: 400, file: "FBW-Display-RMP-16.ttf" },
    FaceDef { screens: &RMPS, family: "RMP-19", weight: 400, file: "FBW-Display-RMP-19.ttf" },
];

const RMPS: [&str; 3] = ["SCREEN_DU_RMP_1", "SCREEN_DU_RMP_2", "SCREEN_DU_RMP_3"];

/// Families CSS names generically, which are not reported missing.
const GENERIC: [&str; 5] = ["monospace", "sans-serif", "serif", "ui-monospace", "system-ui"];

#[derive(Default)]
pub struct Fonts {
    faces: Vec<Face>,
    /// Families asked for that nothing matched, so each is logged once.
    pub missing: Vec<String>,
}

impl Fonts {
    /// Every face in [`FACES`], from `dir`. Returns what went wrong along
    /// the way, for the log.
    pub fn load(dir: &Path) -> (Self, Vec<String>) {
        let mut fonts = Fonts::default();
        let mut problems = Vec::new();
        for face in FACES {
            let screens = face.screens.iter().map(|s| screen_key(s)).collect();
            match std::fs::read(dir.join(face.file)) {
                Ok(data) => {
                    if let Err(e) = fonts.add(face.family, screens, face.weight, false, data) {
                        problems.push(format!("{}: {e}", face.file));
                    }
                }
                Err(e) => problems.push(format!("{}: {e}", dir.join(face.file).display())),
            }
        }
        (fonts, problems)
    }

    pub fn add(&mut self, family: &str, screens: Vec<String>, weight: u16, italic: bool, data: Vec<u8>) -> Result<(), String> {
        let font = FontVec::try_from_vec(data).map_err(|e| e.to_string())?;
        let units_per_em = font.units_per_em().unwrap_or(1000.);
        let ascent = font.ascent_unscaled();
        let descent = -font.descent_unscaled();
        self.faces.push(Face { family: family.to_ascii_lowercase(), screens, weight, italic, font, units_per_em, ascent, descent });
        Ok(())
    }

    pub fn is_empty(&self) -> bool {
        self.faces.is_empty()
    }

    /// The face a CSS font family list resolves to on a screen: the first
    /// family with a face, then the face nearest in style and weight, as
    /// CSS font matching picks it.
    pub fn select(&mut self, screen: &str, families: &str, weight: u16, italic: bool) -> Option<usize> {
        let screen = screen_key(screen);
        for family in families.split(',') {
            let family = family.trim().trim_matches(|c| c == '"' || c == '\'').trim().to_ascii_lowercase();
            if family.is_empty() {
                continue;
            }
            let candidates = self
                .faces
                .iter()
                .enumerate()
                .filter(|(_, f)| f.family == family && (screen.is_empty() || f.screens.is_empty() || f.screens.contains(&screen)));
            // A face for this screen beats one offered to every screen; with
            // no screen named, the one offered to every screen wins.
            let best = candidates.min_by_key(|(_, f)| {
                (
                    f.screens.is_empty() != screen.is_empty(),
                    f.italic != italic,
                    weight_distance(weight, f.weight),
                )
            });
            if let Some((i, _)) = best {
                return Some(i);
            }
        }
        let first = families.split(',').next().unwrap_or("").trim().to_string();
        if !self.missing.contains(&first) && !GENERIC.contains(&first.as_str()) {
            self.missing.push(first);
        }
        None
    }

    pub fn face(&self, i: usize) -> &Face {
        &self.faces[i]
    }

    /// Ascent and descent at a size, both positive, in CSS pixels.
    pub fn metrics(&self, face: usize, size: f64) -> (f64, f64) {
        let f = &self.faces[face];
        let s = size / f.units_per_em as f64;
        (f.ascent as f64 * s, f.descent as f64 * s)
    }

    pub fn layout(&self, face: usize, size: f64, text: &str) -> Run {
        let f = &self.faces[face];
        let s = size / f.units_per_em as f64;
        let mut glyphs = Vec::with_capacity(text.len());
        let mut x = 0.;
        let mut previous: Option<GlyphId> = None;
        for c in text.chars() {
            let id = f.font.glyph_id(c);
            if let Some(p) = previous {
                x += f.font.kern_unscaled(p, id) as f64 * s;
            }
            glyphs.push((id, x));
            x += f.font.h_advance_unscaled(id) as f64 * s;
            previous = Some(id);
        }
        Run { face, glyphs, width: x }
    }

    /// Where a run's pen starts for an alignment and baseline, relative to
    /// the anchor point.
    pub fn anchor_offset(&self, run: &Run, size: f64, align: Align, baseline: Baseline) -> (f64, f64) {
        let dx = match align {
            Align::Left => 0.,
            Align::Center => -run.width / 2.,
            Align::Right => -run.width,
        };
        let (ascent, descent) = self.metrics(run.face, size);
        let dy = match baseline {
            Baseline::Alphabetic => 0.,
            Baseline::Top => ascent,
            Baseline::Bottom => -descent,
            Baseline::Middle => (ascent - descent) / 2.,
            // The hanging baseline, where fonts give none: 80% of the ascent,
            // as browsers take it.
            Baseline::Hanging => ascent * 0.8,
        };
        (dx, dy)
    }

    /// A glyph's outline as a path, through `m`, with the glyph's origin at
    /// the user-space point `origin`.
    pub fn outline(&self, face: usize, glyph: GlyphId, size: f64, m: &Affine, origin: (f64, f64), path: &mut Outline) {
        let f = &self.faces[face];
        let Some(outline) = f.font.outline(glyph) else { return };
        let s = size / f.units_per_em as f64;
        let to_user = |p: ab_glyph::Point| (origin.0 + p.x as f64 * s, origin.1 - p.y as f64 * s);
        let mut last: Option<ab_glyph::Point> = None;
        for curve in &outline.curves {
            let (start, end) = match curve {
                OutlineCurve::Line(a, b) | OutlineCurve::Quad(a, _, b) | OutlineCurve::Cubic(a, _, _, b) => (*a, *b),
            };
            if last != Some(start) {
                path.close();
                let (x, y) = to_user(start);
                path.move_to(m.apply(x, y));
            }
            match curve {
                OutlineCurve::Line(_, b) => {
                    let (x, y) = to_user(*b);
                    path.line_to(m.apply(x, y));
                }
                OutlineCurve::Quad(_, c, b) => {
                    let (cx, cy) = to_user(*c);
                    let (x, y) = to_user(*b);
                    path.quad_to(m, [cx, cy, x, y]);
                }
                OutlineCurve::Cubic(_, c1, c2, b) => {
                    let (c1x, c1y) = to_user(*c1);
                    let (c2x, c2y) = to_user(*c2);
                    let (x, y) = to_user(*b);
                    path.cubic_to(m, [c1x, c1y, c2x, c2y, x, y]);
                }
            }
            last = Some(end);
        }
        path.close();
    }
}

/// How far a face's weight is from the one asked for, as CSS orders
/// fallbacks: for 400 and 500 the nearer of the two first, then lighter,
/// then heavier; lighter weights prefer lighter faces, heavier ones heavier.
fn weight_distance(want: u16, have: u16) -> u32 {
    let (want, have) = (want as i32, have as i32);
    let d = (want - have).unsigned_abs();
    let wrong_way = if want < 400 {
        have > want
    } else if want > 500 {
        have < want
    } else {
        have > 500
    };
    d + if wrong_way { 1000 } else { 0 }
}

/// Screen ids compare without case and without the `$` panel.cfg puts on
/// some of them.
pub fn screen_key(id: &str) -> String {
    id.trim().trim_start_matches('$').to_ascii_lowercase()
}

/// Glyph bitmaps sit this far apart in the atlas, so linear filtering never
/// reads a neighbour.
const PADDING: u32 = 1;
/// Horizontal sub-pixel positions a glyph is rasterised at.
pub const SUBPIXEL: u32 = 4;
pub const ATLAS_START: u32 = 1024;
/// Side of the opaque block flat colours are drawn with, so that paths and
/// text share one texture and batch together.
const SOLID: u32 = 4;
pub const ATLAS_MAX: u32 = 4096;

/// Where a glyph's bitmap is in the atlas, and how it sits on the pen.
#[derive(Clone, Copy, Debug)]
pub struct AtlasGlyph {
    pub x: u32,
    pub y: u32,
    pub w: u32,
    pub h: u32,
    /// Bitmap's top-left relative to the pen, in device pixels, y down.
    pub left: f32,
    pub top: f32,
}

#[derive(Hash, PartialEq, Eq, Clone, Copy)]
struct GlyphKey {
    face: usize,
    glyph: u16,
    /// Pixel size in eighths of a pixel.
    size: u32,
    subpixel: u32,
}

/// One coverage texture holding every glyph every screen draws. When it
/// fills up it is emptied and its generation counts up, and every screen
/// tessellated against the old one is tessellated again.
pub struct Atlas {
    pub size: u32,
    pub pixels: Vec<u8>,
    pub generation: u64,
    glyphs: HashMap<GlyphKey, Option<AtlasGlyph>>,
    shelf_x: u32,
    shelf_y: u32,
    shelf_h: u32,
    /// Rows changed since the last upload, [first, last).
    pub dirty: Option<(u32, u32)>,
}

impl Default for Atlas {
    fn default() -> Self {
        Self::with_size(ATLAS_START)
    }
}

impl Atlas {
    pub fn with_size(size: u32) -> Self {
        Self {
            size,
            pixels: vec![0; (size * size) as usize],
            generation: 1,
            glyphs: HashMap::new(),
            shelf_x: SOLID + PADDING,
            shelf_y: 0,
            shelf_h: SOLID,
            dirty: Some((0, size)),
        }
        .with_solid()
    }

    /// The opaque block flat colours sample, in the top-left corner.
    fn with_solid(mut self) -> Self {
        for y in 0..SOLID {
            for x in 0..SOLID {
                self.pixels[(y * self.size + x) as usize] = 255;
            }
        }
        self
    }

    /// Texture coordinates inside the opaque block, far enough from its
    /// edges that linear filtering reads nothing else.
    pub fn solid_uv(&self) -> (f32, f32) {
        let c = SOLID as f32 / 2. / self.size as f32;
        (c, c)
    }

    /// A glyph rasterised at `px` pixels per em, placed at a pen whose x has
    /// the fraction `subpixel / SUBPIXEL`. `None` for a glyph with no ink,
    /// or `Err` when the atlas is full and has been started again, which
    /// means everything tessellated against it is stale.
    pub fn glyph(&mut self, fonts: &Fonts, face: usize, glyph: GlyphId, px: f64, subpixel: u32) -> Result<Option<AtlasGlyph>, ()> {
        let key = GlyphKey { face, glyph: glyph.0, size: (px * 8.).round().max(1.) as u32, subpixel };
        if let Some(g) = self.glyphs.get(&key) {
            return Ok(*g);
        }
        let f = fonts.face(face);
        let px = key.size as f32 / 8.;
        let scale = PxScale::from(px * f.font.height_unscaled() / f.units_per_em);
        let positioned = glyph.with_scale_and_position(scale, point(subpixel as f32 / SUBPIXEL as f32, 0.));
        let Some(outlined) = f.font.outline_glyph(positioned) else {
            self.glyphs.insert(key, None);
            return Ok(None);
        };
        let bounds = outlined.px_bounds();
        let (w, h) = (bounds.width() as u32, bounds.height() as u32);
        if w == 0 || h == 0 {
            self.glyphs.insert(key, None);
            return Ok(None);
        }
        let Some((x, y)) = self.place(w, h) else {
            let size = (self.size * 2).min(ATLAS_MAX);
            let generation = self.generation + 1;
            *self = Self::with_size(size);
            self.generation = generation;
            return Err(());
        };
        let stride = self.size as usize;
        outlined.draw(|gx, gy, c| {
            let i = (y + gy) as usize * stride + (x + gx) as usize;
            self.pixels[i] = (c.clamp(0., 1.) * 255. + 0.5) as u8;
        });
        let entry = AtlasGlyph { x, y, w, h, left: bounds.min.x, top: bounds.min.y };
        self.dirty = Some(match self.dirty {
            Some((a, b)) => (a.min(y), b.max(y + h)),
            None => (y, y + h),
        });
        self.glyphs.insert(key, Some(entry));
        Ok(Some(entry))
    }

    fn place(&mut self, w: u32, h: u32) -> Option<(u32, u32)> {
        if w + 2 * PADDING > self.size {
            return None;
        }
        if self.shelf_x + w + PADDING > self.size {
            self.shelf_y += self.shelf_h + PADDING;
            self.shelf_x = PADDING;
            self.shelf_h = 0;
        }
        if self.shelf_y + h + PADDING > self.size {
            return None;
        }
        let at = (self.shelf_x, self.shelf_y);
        self.shelf_x += w + PADDING;
        self.shelf_h = self.shelf_h.max(h);
        Some(at)
    }
}

#[cfg(test)]
pub mod tests {
    use super::*;

    /// FlyByWire's EIS font from the MSFS package, when it is on this machine.
    pub fn eis_font() -> Option<Vec<u8>> {
        let package = r"D:\Microsoft Flight Simulator 2020\Microsoft Flight Simulator 2020 Packages\Community\flybywire-aircraft-a380-842\html_ui\Fonts\fbw-a380x";
        std::fs::read(Path::new(package).join("FBW-Display-EIS-A380.ttf")).ok()
    }

    #[test]
    fn families_resolve_per_screen_and_by_weight() {
        let Some(data) = eis_font() else { return };
        let mut fonts = Fonts::default();
        fonts.add("Ecam", Vec::new(), 400, false, data.clone()).unwrap();
        fonts.add("Ecam", vec![screen_key("SCREEN_ISIS_1")], 400, false, data.clone()).unwrap();
        fonts.add("Digital", Vec::new(), 900, false, data).unwrap();
        assert_eq!(fonts.select("SCREEN_DU_PFDL", "\"Ecam\", monospace", 400, false), Some(0));
        assert_eq!(fonts.select("$screen_isis_1", "Ecam", 400, false), Some(1));
        assert_eq!(fonts.select("FCU", "Nope, 'Digital'", 400, false), Some(2));
        assert_eq!(fonts.select("FCU", "Nope", 400, false), None);
        assert_eq!(fonts.missing, vec!["Nope".to_string()]);
        assert!(weight_distance(700, 900) < weight_distance(700, 400));
        assert!(weight_distance(300, 100) < weight_distance(300, 400));
    }

    #[test]
    fn measured_width_is_the_sum_of_advances() {
        let Some(data) = eis_font() else { return };
        let mut fonts = Fonts::default();
        fonts.add("Ecam", Vec::new(), 400, false, data).unwrap();
        let one = fonts.layout(0, 22., "0").width;
        let run = fonts.layout(0, 22., "0000");
        assert!((run.width - 4. * one).abs() < 1e-9);
        assert!((run.glyphs[3].1 - 3. * one).abs() < 1e-9);
        let (ascent, descent) = fonts.metrics(0, 4096.);
        assert_eq!((ascent, descent), (3276., 820.));
    }

    #[test]
    fn the_atlas_grows_when_full() {
        let Some(data) = eis_font() else { return };
        let mut fonts = Fonts::default();
        fonts.add("Ecam", Vec::new(), 400, false, data).unwrap();
        let mut atlas = Atlas::with_size(64);
        let a = fonts.face(0).font.glyph_id('A');
        assert!(atlas.glyph(&fonts, 0, a, 20., 0).unwrap().is_some());
        let mut grew = false;
        for size in 21..200 {
            if atlas.glyph(&fonts, 0, a, size as f64, 0).is_err() {
                grew = true;
                break;
            }
        }
        assert!(grew);
        assert_eq!((atlas.size, atlas.generation), (128, 2));
    }
}
