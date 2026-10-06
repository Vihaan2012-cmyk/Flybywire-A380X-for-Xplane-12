//! A Plane Maker .acf for the converted aircraft.
//!
//! A real X-Plane airliner's .acf is the template, so every one of its tens
//! of thousands of properties holds a working value and the result loads.
//! Over it goes what the MSFS package states, and what can be measured from
//! its 3D model:
//!
//! - Wing, tailplane, fin, wingtip fences and pylons: sliced from the model
//!   every foot of span (leading and trailing edge, chord, height), then
//!   fitted as X-Plane surfaces: root position at the quarter chord, sweep
//!   along the quarter-chord line, segment length along that line.
//! - Fuselage, belly fairing and nacelles: rebuilt ring by ring from the
//!   model, 18 points per ring as Plane Maker stores them.
//! - Weights, CG and its limits, fuel tanks, engines and thrust, gear
//!   contact points, V-speeds, flap detents and control travel: from the
//!   package's flight_model.cfg, engines.cfg and aircraft.cfg.
//! - Misc objects: the converted OBJ files.
//!
//! Units: .acf lengths are feet with x right, y up and z aft of the
//! reference point, which here is the MSFS model origin, so the objects sit
//! at 0,0,0. MSFS cfg positions are (longitudinal forward, lateral right,
//! vertical up) feet from its datum.

use std::collections::{BTreeMap, HashMap};
use std::path::Path;

use anyhow::{bail, Context};
use crate::model::Model;

/// What a converted object is, which picks the template misc object whose
/// Plane Maker settings it takes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObjKind {
    /// Outside geometry (the template's wing object).
    Exterior,
    /// Cockpit and cabin geometry (the template's first cockpit object).
    Cabin,
    /// The click spots (the template's clickable 3D cockpit object).
    Cockpit,
    /// Lights only (the template's lights object).
    Lights,
    /// Blended outside surfaces: windows, light covers (glass lighting).
    ExteriorGlass,
    /// Blended cockpit and cabin surfaces: windscreen, display glass (glass
    /// lighting, inside); X-Plane lets daylight in through them.
    CabinGlass,
    /// Blended outside geometry that is not glass (lettering decals): an
    /// ordinary exterior object marked translucent.
    ExteriorBlend,
    /// Blended inside geometry that is not glass (placards, legends): an
    /// ordinary cabin object marked translucent.
    CabinBlend,
    /// The passenger cabin proper (the LOD01 graft: seats, sidewalls,
    /// ceiling, stairs) -- geometry, not [`Self::Cabin`]'s broader "ordinary
    /// non-clicked interior object" (which also covers plain cockpit decor
    /// and demoted cockpit objects that lost X-Plane's one click-tested
    /// slot). Unlike every other interior kind, this one is drawn from
    /// outside the aircraft too (not `_v10_is_internal`), so an open
    /// passenger door does not show a hollow fuselage -- capped by its own
    /// `ATTR_LOD` (`--cabin-lod-far`) so that only costs triangles close up.
    PaxCabin,
}

/// The .acf object flag Plane Maker sets for an object with translucent
/// geometry: X-Plane 12 draws it after the opaque pass, with blending.
/// Laminar's own glass and "transparent" objects all carry it.
const OBJ_FLAG_TRANSLUCENT: i64 = 0x2000;

/// The .acf object flag for casting shadows inside the cockpit: every one of
/// Laminar's cockpit objects carries it (13, 29) and their exterior ones do
/// too (24 = this and the outside-shadow bit, 0x10). Clearing it on every
/// object turned the whole flight deck sun-lit (2026-09-28's lighting test).
const OBJ_FLAG_INTERIOR_SHADOW: i64 = 0x8;

/// How far from the pilot's eye an exterior object must reach to be able to
/// shade the flight deck, and how far below the eye its floor is.
const FLIGHT_DECK_REACH_M: f64 = 3.0;
const FLIGHT_DECK_FLOOR_BELOW_EYE_M: f64 = 1.0;

/// Whether an exterior object can throw a shadow into the flight deck: its
/// box comes within [`FLIGHT_DECK_REACH_M`] of the pilot's eye and rises
/// above the flight-deck floor. X-Plane draws every object carrying
/// [`OBJ_FLAG_INTERIOR_SHADOW`] again in each cockpit shadow pass; on the
/// A380 that was the whole aircraft -- wings, engines, the aft fuselage and
/// 868,000 triangles of tyres under the floor -- 2.5 million triangles per
/// pass, none of which can reach the flight deck (2026-09-28, CPU-bound at
/// 58 ms a frame). Such an object keeps its outside shadow.
fn casts_into_flight_deck(bounds: ([f64; 3], [f64; 3]), eye: [f64; 3]) -> bool {
    let (lo, hi) = bounds;
    let near = (0..3).all(|k| hi[k] >= eye[k] - FLIGHT_DECK_REACH_M && lo[k] <= eye[k] + FLIGHT_DECK_REACH_M);
    near && hi[1] > eye[1] - FLIGHT_DECK_FLOOR_BELOW_EYE_M
}

/// The template object's flags with the translucent flag added.
fn translucent_flags(template: Option<&str>) -> i64 {
    template.and_then(|f| f.trim().parse::<i64>().ok()).unwrap_or(0) | OBJ_FLAG_TRANSLUCENT
}

/// Whether X-Plane should treat this object as interior-only
/// (`_v10_is_internal`): every cockpit and cabin kind except
/// [`ObjKind::PaxCabin`], the one interior kind meant to draw from outside
/// the aircraft too (an open passenger door should not show a hollow
/// fuselage) -- see its own doc.
fn is_internal(kind: ObjKind) -> bool {
    matches!(kind, ObjKind::Cockpit | ObjKind::Cabin | ObjKind::CabinGlass | ObjKind::CabinBlend)
}

const FT: f64 = 3.280_84;

/// An .acf: its lines, with added properties kept apart so line indices
/// stay valid.
pub struct Acf {
    lines: Vec<Option<String>>,
    index: HashMap<String, usize>,
    end: usize,
    added: Vec<(String, String)>,
    added_index: HashMap<String, usize>,
}

impl Acf {
    pub fn parse(text: &str) -> anyhow::Result<Acf> {
        let mut lines = Vec::new();
        let mut index = HashMap::new();
        let mut end = None;
        for (i, l) in text.lines().enumerate() {
            if let Some(rest) = l.strip_prefix("P ") {
                let key = rest.split_once(' ').map_or(rest, |(k, _)| k);
                index.insert(key.to_string(), i);
            }
            if l.trim() == "PROPERTIES_END" {
                end = Some(i);
            }
            lines.push(Some(l.to_string()));
        }
        let end = end.context("no PROPERTIES_END: not a Plane Maker .acf")?;
        Ok(Acf {
            lines,
            index,
            end,
            added: Vec::new(),
            added_index: HashMap::new(),
        })
    }

    pub fn get(&self, key: &str) -> Option<&str> {
        if let Some(&j) = self.added_index.get(key) {
            return Some(&self.added[j].1);
        }
        let l = self.lines[*self.index.get(key)?].as_deref()?;
        l.strip_prefix("P ")?.split_once(' ').map(|(_, v)| v)
    }

    pub fn getf(&self, key: &str) -> Option<f64> {
        self.get(key)?.trim().parse().ok()
    }

    /// Set a property, adding it when the template lacks it.
    pub fn set(&mut self, key: &str, value: impl std::fmt::Display) {
        let v = value.to_string();
        if let Some(&i) = self.index.get(key) {
            if self.lines[i].is_some() {
                self.lines[i] = Some(format!("P {key} {v}"));
                return;
            }
        }
        if let Some(&j) = self.added_index.get(key) {
            self.added[j].1 = v;
            return;
        }
        self.added_index.insert(key.to_string(), self.added.len());
        self.added.push((key.to_string(), v));
    }

    pub fn setf(&mut self, key: &str, value: f64) {
        self.set(key, format!("{value:.9}"));
    }

    /// Every property under a prefix, as (key, value).
    pub fn with_prefix(&self, prefix: &str) -> Vec<(String, String)> {
        let mut out: Vec<(String, String)> = self
            .lines
            .iter()
            .flatten()
            .filter_map(|l| l.strip_prefix("P "))
            .filter(|r| r.starts_with(prefix))
            .filter_map(|r| r.split_once(' ').map(|(k, v)| (k.to_string(), v.to_string())))
            .collect();
        out.extend(self.added.iter().filter(|(k, _)| k.starts_with(prefix)).cloned());
        out
    }

    pub fn remove_prefix(&mut self, prefix: &str) {
        for l in self.lines.iter_mut() {
            if l.as_deref().and_then(|l| l.strip_prefix("P ")).is_some_and(|r| r.starts_with(prefix)) {
                *l = None;
            }
        }
        self.added.retain(|(k, _)| !k.starts_with(prefix));
        self.added_index = self.added.iter().enumerate().map(|(i, (k, _))| (k.clone(), i)).collect();
    }

    /// The .acf text: the header and properties, then this aircraft's own
    /// (empty) 2D and 3D panels. The template's panels are never copied:
    /// they hold its instruments (the A330's ECAM, PFD, ND and radio panels,
    /// wired to Laminar's Airbus FMGS), and X-Plane refuses to load an
    /// aircraft whose panel needs an FMGS without the matching MCDU.
    pub fn to_text(&self) -> String {
        let mut out = String::new();
        for l in self.lines.iter().take(self.end).flatten() {
            out.push_str(l);
            out.push('\n');
        }
        for (k, v) in &self.added {
            out.push_str(&format!("P {k} {v}\n"));
        }
        out.push_str("PROPERTIES_END\nPANEL_2D_BEGIN\nPANEL_2D_END\nPANEL_3D_BEGIN\nPANEL_3D_END\n");
        out
    }
}

/// An MSFS .cfg: section (upper case) -> key (lower case) -> value.
type Cfg = HashMap<String, BTreeMap<String, String>>;

fn read_cfg(path: &Path) -> Cfg {
    parse_cfg(&std::fs::read_to_string(path).unwrap_or_default())
}

fn parse_cfg(text: &str) -> Cfg {
    let mut out: Cfg = HashMap::new();
    let mut sect = String::new();
    for l in text.lines() {
        let l = l.split(';').next().unwrap_or("").trim();
        if l.starts_with('[') {
            sect = l.trim_matches(['[', ']']).to_ascii_uppercase();
        } else if let Some((k, v)) = l.split_once('=') {
            out.entry(sect.clone()).or_default().insert(k.trim().to_ascii_lowercase(), v.trim().trim_matches('"').to_string());
        }
    }
    out
}

fn num(cfg: &Cfg, sect: &str, key: &str) -> Option<f64> {
    cfg.get(sect)?.get(key)?.split(',').next()?.trim().parse().ok()
}

fn nums(s: &str) -> Vec<f64> {
    s.split(',').filter_map(|x| x.trim().parse().ok()).collect()
}

/// The offline calibration check XP-003 asks for: the stall speed (KCAS at
/// sea level/ISA) a weight, wing area and CLmax predict from the textbook
/// 1g lift equation (`L = W` at the stall: `Vs = sqrt(2W / (rho S CLmax))`).
/// Nothing here feeds the .acf — X-Plane's blade-element model derives its
/// own stall speed from the converted wing's geometry and airfoils, it does
/// not accept one as an input — this is only a check that the conversion's
/// wing area (flight_model.cfg's own `wing_area`) and CLmax
/// (`lift_curve_peak`'s reading of `lift_coef_aoa_table`) are in the right
/// place, documented against flight_model.cfg's own reference stall speeds
/// in docs/flight-model.md.
fn stall_speed_kt(weight_lbf: f64, wing_area_ft2: f64, cl_max: f64) -> f64 {
    const RHO_SL_SLUG_FT3: f64 = 0.0023769;
    const FT_PER_S_TO_KT: f64 = 0.5924838;
    (2.0 * weight_lbf / (RHO_SL_SLUG_FT3 * wing_area_ft2 * cl_max)).sqrt() * FT_PER_S_TO_KT
}

/// A point one MSFS cfg names, in .acf coordinates.
fn acf_point(lon: f64, lat: f64, vert: f64, datum: [f64; 3]) -> [f64; 3] {
    [lat + datum[1], vert + datum[2], -(lon + datum[0])]
}

/// Model vertices of meshes whose base texture names one of `include` (all
/// meshes when empty) and none of `exclude`, in .acf feet.
fn verts(model: &Model, include: &[&str], exclude: &[&str]) -> Vec<[f64; 3]> {
    let mut out = Vec::new();
    for m in &model.meshes {
        let tex = model.materials.get(m.material).and_then(|x| x.base_color.clone()).unwrap_or_default().to_ascii_uppercase();
        if !include.is_empty() && !include.iter().any(|k| tex.contains(k)) {
            continue;
        }
        if exclude.iter().any(|k| tex.contains(k)) {
            continue;
        }
        out.extend(m.vertices.iter().map(|v| [-v.pos[0] as f64 * FT, v.pos[1] as f64 * FT, -v.pos[2] as f64 * FT]));
    }
    out
}

/// A triangle in .acf feet.
type Tri = [[f64; 3]; 3];

/// Model triangles of meshes whose base texture names one of `include`
/// (all meshes when empty) and none of `exclude`, in .acf feet.
fn tris(model: &Model, include: &[&str], exclude: &[&str]) -> Vec<Tri> {
    let mut out = Vec::new();
    for m in &model.meshes {
        let tex = model.materials.get(m.material).and_then(|x| x.base_color.clone()).unwrap_or_default().to_ascii_uppercase();
        if !include.is_empty() && !include.iter().any(|k| tex.contains(k)) {
            continue;
        }
        if exclude.iter().any(|k| tex.contains(k)) {
            continue;
        }
        let p = |i: u32| {
            let v = m.vertices[i as usize].pos;
            [-v[0] as f64 * FT, v[1] as f64 * FT, -v[2] as f64 * FT]
        };
        out.extend(m.indices.chunks_exact(3).map(|t| [p(t[0]), p(t[1]), p(t[2])]));
    }
    out
}

fn centroid(t: &Tri) -> [f64; 3] {
    [(t[0][0] + t[1][0] + t[2][0]) / 3.0, (t[0][1] + t[1][1] + t[2][1]) / 3.0, (t[0][2] + t[1][2] + t[2][2]) / 3.0]
}

/// One slice across a lifting surface: leading and trailing edge (z), the
/// height of its chord line and its lowest point.
#[derive(Debug, Clone, Copy)]
struct Station {
    s: f64,
    le: f64,
    te: f64,
    h: f64,
    low: f64,
}

impl Station {
    fn chord(&self) -> f64 {
        self.te - self.le
    }
    fn qc(&self) -> f64 {
        self.le + 0.25 * self.chord()
    }
}

/// Cut triangles with a plane every `step` feet along `span`: each station
/// holds the exact cross-section where the surface crosses it, however far
/// apart the mesh's vertices are. `height` is the coordinate across the
/// chord plane (y for wings, x for vertical surfaces).
fn slice(tris: &[Tri], span: impl Fn(&[f64; 3]) -> f64, height: impl Fn(&[f64; 3]) -> f64, step: f64) -> Vec<Station> {
    let mut cuts: BTreeMap<i64, Vec<[f64; 3]>> = BTreeMap::new();
    for t in tris {
        let sp = [span(&t[0]), span(&t[1]), span(&t[2])];
        let (a, b) = (sp[0].min(sp[1]).min(sp[2]), sp[0].max(sp[1]).max(sp[2]));
        for k in (a / step).ceil() as i64..=(b / step).floor() as i64 {
            let s0 = k as f64 * step;
            for (i, j) in [(0, 1), (1, 2), (2, 0)] {
                let (u, v) = (sp[i] - s0, sp[j] - s0);
                if (u < 0.0) == (v < 0.0) || (u - v).abs() < 1e-12 {
                    continue;
                }
                let f = (u / (u - v)).clamp(0.0, 1.0);
                let q = [
                    t[i][0] + (t[j][0] - t[i][0]) * f,
                    t[i][1] + (t[j][1] - t[i][1]) * f,
                    t[i][2] + (t[j][2] - t[i][2]) * f,
                ];
                cuts.entry(k).or_default().push(q);
            }
        }
    }
    cuts.into_iter()
        .filter(|(_, v)| v.len() >= 4)
        .map(|(k, v)| {
            let le = *v.iter().min_by(|a, b| a[2].total_cmp(&b[2])).unwrap();
            // The trailing edge on the chord plane: flap-track fairings
            // reach further aft but hang well below it. The window grows
            // with the chord, since a thick root's trailing edge sits a few
            // feet below its leading edge.
            let hl = height(&le);
            let raw = v.iter().map(|p| p[2]).fold(f64::MIN, f64::max) - le[2];
            let window = (0.15 * raw).max(3.0);
            let te = v
                .iter()
                .filter(|p| (height(p) - hl).abs() < window)
                .max_by(|a, b| a[2].total_cmp(&b[2]))
                .copied()
                .unwrap_or(le);
            Station {
                s: k as f64 * step,
                le: le[2],
                te: te[2],
                h: (hl + height(&te)) / 2.0,
                low: v.iter().map(|p| p[1]).fold(f64::MAX, f64::min),
            }
        })
        .collect()
}

/// The station at `s`, interpolated between the nearest slices.
fn at(st: &[Station], s: f64) -> Station {
    if s <= st[0].s {
        return Station { s, ..st[0] };
    }
    if s >= st[st.len() - 1].s {
        return Station { s, ..st[st.len() - 1] };
    }
    let i = st.iter().position(|x| x.s >= s).unwrap_or(st.len() - 1);
    let (a, b) = (st[i - 1], st[i]);
    let t = (s - a.s) / (b.s - a.s);
    let l = |x: f64, y: f64| x + (y - x) * t;
    Station {
        s,
        le: l(a.le, b.le),
        te: l(a.te, b.te),
        h: l(a.h, b.h),
        low: l(a.low, b.low),
    }
}

/// With MSFS2XP_ACF_DEBUG set, print a surface's stations to stderr.
fn debug_stations(what: &str, st: &[Station]) {
    if std::env::var_os("MSFS2XP_ACF_DEBUG").is_none() {
        return;
    }
    eprintln!("{what}: {} stations (s, le, te, chord, h)", st.len());
    for s in st.iter().step_by(3) {
        eprintln!("  {:7.1} {:8.1} {:8.1} {:6.1} {:6.1}", s.s, s.le, s.te, s.chord(), s.h);
    }
}

/// Stations with each edge median-filtered over eleven slices, which removes
/// spikes up to five slices wide (a flap-track fairing, an engine exhaust)
/// without moving the edges elsewhere.
fn smooth(st: Vec<Station>) -> Vec<Station> {
    let med = |i: usize, f: &dyn Fn(&Station) -> f64| {
        let mut w: Vec<f64> = st[i.saturating_sub(5)..(i + 6).min(st.len())].iter().map(f).collect();
        w.sort_by(|a, b| a.total_cmp(b));
        w[w.len() / 2]
    };
    (0..st.len())
        .map(|i| Station {
            s: st[i].s,
            le: med(i, &|s| s.le),
            te: med(i, &|s| s.te),
            h: med(i, &|s| s.h),
            low: st[i].low,
        })
        .collect()
}

/// A straight line through `f` over the stations as (value at s = 0,
/// slope), by Theil-Sen: the median of pairwise slopes, so outlying slices
/// (a fillet, a gap) do not tilt it.
fn theil_sen(pts: &[&Station], f: impl Fn(&Station) -> f64) -> (f64, f64) {
    if pts.len() < 2 {
        return (pts.first().map_or(0.0, |s| f(s)), 0.0);
    }
    let mut slopes = Vec::new();
    for i in 0..pts.len() {
        for j in i + 1..pts.len() {
            let ds = pts[j].s - pts[i].s;
            if ds.abs() > 1e-9 {
                slopes.push((f(pts[j]) - f(pts[i])) / ds);
            }
        }
    }
    slopes.sort_by(|a, b| a.total_cmp(b));
    let slope = slopes.get(slopes.len() / 2).copied().unwrap_or(0.0);
    let mut icepts: Vec<f64> = pts.iter().map(|s| f(s) - slope * s.s).collect();
    icepts.sort_by(|a, b| a.total_cmp(b));
    (icepts[icepts.len() / 2], slope)
}

/// Where the robust line through `f` over the stations meets s = 0.
fn intercept(pts: &[&Station], f: impl Fn(&Station) -> f64) -> f64 {
    theil_sen(pts, f).0
}

/// A single-panel surface fitted to its stations: straight leading and
/// trailing edges and chord-line height, from `from` to `to` (the
/// surface's true tip, which lies up to a slice beyond the last station).
fn fitted(st: &[Station], from: f64, to: f64) -> Option<(Station, Station)> {
    let last = *st.last()?;
    let refs: Vec<&Station> = st.iter().collect();
    let (le0, le1) = theil_sen(&refs, |s| s.le);
    let (te0, te1) = theil_sen(&refs, |s| s.te);
    let (h0, h1) = theil_sen(&refs, |s| s.h);
    let make = |s: f64| Station {
        s,
        le: le0 + le1 * s,
        te: te0 + te1 * s,
        h: h0 + h1 * s,
        low: last.low,
    };
    Some((make(from), make(to.max(last.s))))
}

/// A small vertical surface (a fence, a pylon) as one panel of its mean
/// chord (area over height), from its lowest to its highest station, with
/// the leading edge along a robust line. Their shapes (a diamond fence, a
/// swept fairing) do not taper from root to tip, so end chords mislead.
fn mean_panel(st: &[Station]) -> Option<(Station, Station)> {
    let (first, last) = (*st.first()?, *st.last()?);
    let height = last.s - first.s;
    if height <= 0.0 {
        return None;
    }
    let area: f64 = st.windows(2).map(|w| (w[0].chord() + w[1].chord()) / 2.0 * (w[1].s - w[0].s)).sum();
    let chord = area / height;
    let refs: Vec<&Station> = st.iter().collect();
    let (l0, l1) = theil_sen(&refs, |s| s.le);
    let mut hs: Vec<f64> = st.iter().map(|s| s.h).collect();
    hs.sort_by(|a, b| a.total_cmp(b));
    let h = hs[hs.len() / 2];
    let make = |s: f64| Station {
        s,
        le: l0 + l1 * s,
        te: l0 + l1 * s + chord,
        h,
        low: first.low,
    };
    Some((make(first.s), make(last.s)))
}

/// One X-Plane surface between two stations.
struct Surface {
    root: [f64; 3],
    croot: f64,
    ctip: f64,
    semilen: f64,
    sweep: f64,
    dihed: f64,
}

/// A horizontal surface from `a` to `b` on the right side (x = s).
fn horizontal(a: &Station, a_x: f64, b: &Station) -> Surface {
    let (p0, p1) = ([a_x, a.h, a.qc()], [b.s, b.h, b.qc()]);
    let (dx, dy, dz) = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
    Surface {
        root: p0,
        croot: a.chord(),
        ctip: b.chord(),
        semilen: (dx * dx + dy * dy + dz * dz).sqrt(),
        sweep: dz.atan2((dx * dx + dy * dy).sqrt()).to_degrees(),
        dihed: dy.atan2(dx).to_degrees(),
    }
}

/// A vertical surface rising from `a` to `b` (s is the height, h the x).
fn vertical(a: &Station, b: &Station) -> Surface {
    let (dy, dz) = (b.s - a.s, b.qc() - a.qc());
    Surface {
        root: [a.h, a.s, a.qc()],
        croot: a.chord(),
        ctip: b.chord(),
        semilen: (dy * dy + dz * dz).sqrt(),
        sweep: dz.atan2(dy).to_degrees(),
        dihed: 90.0,
    }
}

fn write_surface(acf: &mut Acf, i: usize, s: &Surface, right: bool, incidence: &dyn Fn(f64) -> f64) {
    let p = format!("_wing/{i}/");
    let x = if right { s.root[0].abs() } else { -s.root[0].abs() };
    acf.setf(&format!("{p}_part_x"), x);
    acf.setf(&format!("{p}_part_y"), s.root[1]);
    acf.setf(&format!("{p}_part_z"), s.root[2]);
    acf.setf(&format!("{p}_Croot"), s.croot);
    acf.setf(&format!("{p}_Ctip"), s.ctip);
    acf.setf(&format!("{p}_semilen_SEG"), s.semilen);
    acf.setf(&format!("{p}_sweep_design"), s.sweep);
    acf.setf(&format!("{p}_dihed_design"), s.dihed);
    acf.setf(&format!("{p}_is_right_mult"), if right { 1.0 } else { -1.0 });
    let els = acf.getf(&format!("{p}_els")).unwrap_or(10.0).max(1.0) as usize;
    for e in 0..els.min(10) {
        acf.setf(&format!("{p}_incidence/{e}"), incidence((e as f64 + 0.5) / els as f64));
    }
}

/// X-Plane's own wing flex, kept short of putting the engines on the runway.
/// X-Plane bends the whole joined wing into an even curve by the airframe's
/// g, and the parts hung from it follow. At the A330 template's 1.5 deg of
/// mid-span dihedral per g, the A380's longer wing lowers its outer nacelles
/// about 0.9 m per g against 1.6 m of clearance, so a jolt on top of the
/// full-tank droop put them and the outer wing on the runway behind the CG.
/// The contact raised the g, the g bent the wing further, and the aircraft
/// tipped onto its nose while the wing curled under it (X-Plane's
/// flight-model cycle dump, 30 Sep: wing elements 14 m below the CG). At
/// 0.5 deg per g they drop about 0.3 m per g. The wing drawn bends with
/// FlyByWire's own flex model, as in MSFS; this is X-Plane's physics only.
const WING_DIHEDRAL_PER_G: f64 = 0.5;

fn wing_flex(acf: &mut Acf) {
    acf.setf("acf/_wing_mid_dihed_per_g", WING_DIHEDRAL_PER_G);
}

/// 18-point rings (top, down the right side to the bottom, up the left) of
/// a body at stations `zs`, relative to `part`.
fn body_rings(points: &[[f64; 3]], zs: &[f64], part: [f64; 3]) -> Vec<[[f64; 3]; 18]> {
    let mut out = Vec::new();
    for (i, &z) in zs.iter().enumerate() {
        let gap = zs
            .get(i + 1)
            .map(|n| n - z)
            .into_iter()
            .chain(i.checked_sub(1).map(|j| z - zs[j]))
            .fold(f64::MAX, f64::min)
            .clamp(0.8, 6.0)
            / 2.0;
        let slab: Vec<&[f64; 3]> = points.iter().filter(|p| (p[2] - z).abs() <= gap).collect();
        let mut ring = [[part[0], part[1], z]; 18];
        if slab.is_empty() {
            out.push(ring.map(|p| [p[0] - part[0], p[1] - part[1], p[2] - part[2]]));
            continue;
        }
        let (x0, x1) = slab.iter().fold((f64::MAX, f64::MIN), |a, p| (a.0.min(p[0]), a.1.max(p[0])));
        let (y0, y1) = slab.iter().fold((f64::MAX, f64::MIN), |a, p| (a.0.min(p[1]), a.1.max(p[1])));
        let (cx, cy) = ((x0 + x1) / 2.0, (y0 + y1) / 2.0);
        let mut r = [None::<f64>; 18];
        for (j, rj) in r.iter_mut().enumerate() {
            let theta = if j <= 8 { j as f64 * 22.5 } else { 180.0 + (j - 9) as f64 * 22.5 };
            *rj = slab
                .iter()
                .filter(|p| {
                    let a = (p[0] - cx).atan2(p[1] - cy).to_degrees().rem_euclid(360.0);
                    let d = (a - theta).rem_euclid(360.0);
                    d.min(360.0 - d) <= 11.25
                })
                .map(|p| (p[0] - cx).hypot(p[1] - cy))
                .reduce(f64::max);
        }
        // Directions with no points take their neighbours' radius.
        let known: Vec<f64> = r.iter().flatten().copied().collect();
        let fallback = known.iter().sum::<f64>() / known.len().max(1) as f64;
        for j in 0..18 {
            let theta = if j <= 8 { j as f64 * 22.5 } else { 180.0 + (j - 9) as f64 * 22.5 };
            let rad = r[j].or_else(|| r[(j + 1) % 18].or(r[(j + 17) % 18])).unwrap_or(fallback);
            let t = theta.to_radians();
            ring[j] = [cx + rad * t.sin(), cy + rad * t.cos(), z];
        }
        out.push(ring.map(|p| [p[0] - part[0], p[1] - part[1], p[2] - part[2]]));
    }
    out
}

fn write_body(acf: &mut Acf, i: usize, part: [f64; 3], rings: &[[[f64; 3]; 18]]) {
    let p = format!("_body/{i}/");
    acf.setf(&format!("{p}_part_x"), part[0]);
    acf.setf(&format!("{p}_part_y"), part[1]);
    acf.setf(&format!("{p}_part_z"), part[2]);
    for (s, ring) in rings.iter().enumerate() {
        for (j, pt) in ring.iter().enumerate() {
            for (k, v) in pt.iter().enumerate() {
                acf.setf(&format!("{p}_geo_xyz/{s},{j},{k}"), *v);
            }
        }
    }
    acf.setf(&format!("{p}_part_rad"), body_radius(rings));
}

/// A body's `_part_rad`: its cross-section radius, the farthest any ring
/// point lies from its own ring's centre -- what Plane Maker writes (the
/// 777-F's fuselage 10.30 ft on 10.38 ft rings, its nacelles 6.69 on 6.50;
/// the A330's nacelles 6.40 on 5.49; the 737's 4.30 on 4.01). X-Plane sizes
/// an engine by its nacelle's radius and stands the aircraft on it: this
/// used to be the body's bounding sphere, 26.7-28.1 ft for nacelles 7.7 ft
/// in radius, so each A380 engine reached 8 m below the wing and X-Plane
/// placed the aircraft on its engines, 8 m in the air (2026-09-30).
fn body_radius(rings: &[[[f64; 3]; 18]]) -> f64 {
    rings
        .iter()
        .map(|ring| {
            let (cx, cy) = (ring.iter().map(|p| p[0]).sum::<f64>() / 18., ring.iter().map(|p| p[1]).sum::<f64>() / 18.);
            ring.iter().map(|p| (p[0] - cx).hypot(p[1] - cy)).fold(0.0, f64::max)
        })
        .fold(0.0, f64::max)
}

/// Station positions (feet along the body, from its first station) of a
/// template body, as fractions of its length.
fn station_fractions(acf: &Acf, body: usize) -> Vec<f64> {
    let n = acf.getf(&format!("_body/{body}/_s_dim")).unwrap_or(20.0) as usize;
    let zs: Vec<f64> = (0..n).map(|s| acf.getf(&format!("_body/{body}/_geo_xyz/{s},0,2")).unwrap_or(0.0)).collect();
    let (z0, z1) = (zs[0], zs.iter().copied().fold(f64::MIN, f64::max));
    zs.iter().map(|z| ((z - z0) / (z1 - z0).max(1e-6)).clamp(0.0, 1.0)).collect()
}

fn span_of(points: &[[f64; 3]], axis: usize) -> (f64, f64) {
    points.iter().fold((f64::MAX, f64::MIN), |a, p| (a.0.min(p[axis]), a.1.max(p[axis])))
}

/// Horizontal stabiliser trim travel. MSFS's elevator_trim_up_limit and
/// elevator_trim_down_limit are the trimmable stabiliser's nose-up and
/// nose-down travel (FlyByWire's A380X: 10 up, 2 down, the THS's +10/-2 deg
/// in a380_systems); X-Plane's are `_stab_trim_up` and `_stab_trim_dn`
/// (DataRefs.txt: acf_hstb_trim_up/dn, "maximum degrees deflection" up and
/// down of a stabiliser that moves in trim). The A330 template's are 8 each.
fn stab_trim(fm: &Cfg, acf: &mut Acf) -> Option<String> {
    let geo = "AIRPLANE_GEOMETRY";
    let up = num(fm, geo, "elevator_trim_up_limit");
    let dn = num(fm, geo, "elevator_trim_down_limit");
    if up.is_none() && dn.is_none() {
        return None;
    }
    let was = (acf.getf("acf/_stab_trim_up"), acf.getf("acf/_stab_trim_dn"));
    if let Some(v) = up {
        acf.setf("acf/_stab_trim_up", v.abs());
    }
    if let Some(v) = dn {
        acf.setf("acf/_stab_trim_dn", v.abs());
    }
    let f = |v: Option<f64>| v.map_or("-".to_string(), |v| format!("{v}"));
    Some(format!(
        "stabiliser trim: {} deg nose up, {} deg nose down (flight_model.cfg; the template's: {} up, {} down)",
        f(up.map(f64::abs)),
        f(dn.map(f64::abs)),
        f(was.0),
        f(was.1)
    ))
}

/// Rudder trim authority as a fraction of full rudder throw: X-Plane's
/// `acf/_hdng_acft_lf/rt_trim_rat` is a ratio of `_rudd1_lf/rt` (Plane
/// Maker's Control Geometry page), not an absolute angle, so it must be
/// derived from flight_model.cfg's `rudder_trim_limit` over `rudder_limit`
/// rather than copied from either alone (FlyByWire's A380X: 25.5 deg trim /
/// 30 deg full = 0.85, on the A330 template's 0.83).
fn rudder_trim_ratio(fm: &Cfg, acf: &mut Acf) -> Option<String> {
    let geo = "AIRPLANE_GEOMETRY";
    let trim = num(fm, geo, "rudder_trim_limit")?;
    let full = num(fm, geo, "rudder_limit")?;
    if full == 0.0 {
        return None;
    }
    let was = (acf.getf("acf/_hdng_acft_lf_trim_rat"), acf.getf("acf/_hdng_acft_rt_trim_rat"));
    let ratio = (trim.abs() / full.abs()).clamp(0.0, 1.0);
    acf.setf("acf/_hdng_acft_lf_trim_rat", ratio);
    acf.setf("acf/_hdng_acft_rt_trim_rat", ratio);
    let f = |v: Option<f64>| v.map_or("-".to_string(), |v| format!("{v:.3}"));
    Some(format!(
        "rudder trim ratio: {ratio:.3} ({trim} deg trim / {full} deg full travel, flight_model.cfg; the template's: {}/{})",
        f(was.0),
        f(was.1)
    ))
}

pub struct Inputs<'a> {
    pub template: &'a Path,
    pub cfg_dir: &'a Path,
    pub exterior: &'a Model,
    /// Object files (relative to objects/) and what each is.
    pub objects: &'a [(String, ObjKind)],
    /// Each object's extent by file name (metres, the OBJ's own frame:
    /// x right, y up, z aft of the reference point it is attached at), where
    /// known -- see [`casts_into_flight_deck`].
    pub object_bounds: &'a std::collections::HashMap<String, ([f64; 3], [f64; 3])>,
    pub name: &'a str,
    /// Name of the default livery in X-Plane's livery menu.
    pub livery: &'a str,
    /// Maximum operating speed (kt) and Mach, which MSFS cfgs do not hold
    /// (their max_indicated_speed and max_mach are overspeed damage limits).
    pub vmo: Option<f64>,
    pub mmo: Option<f64>,
    /// Leading edge of the mean aerodynamic chord (feet, MSFS longitudinal
    /// from the datum) and the chord's length, from the aircraft's loadsheet.
    pub lemac: Option<f64>,
    pub mac: Option<f64>,
    /// Longitudinal centre of gravity in X-Plane's frame (feet, negative
    /// forward), replacing the one derived from the cfg's empty-weight
    /// figure. See `--cg-z` in `main.rs` for why the A380 needs one and how
    /// its value was derived from the certificated nose-gear load.
    pub cg_z: Option<f64>,
    /// Trailing-edge flap and leading-edge slat angles per handle detent
    /// (degrees), from the systems when the cfg's differ (FlyByWire's A380X:
    /// flaps 0/0/8/17/26/33 and slats 0/20/20/20/23/23, a380_systems
    /// hydraulic/mod.rs FPPU-to-surface tables and sfcc channels).
    pub flap_degrees: Option<Vec<f64>>,
    pub slat_degrees: Option<Vec<f64>>,
    /// Steering limits (degrees) of the nose gear and of the steered main
    /// (body) gear, from the systems when the cfg's differ.
    pub nose_steering: Option<f64>,
    pub body_steering: Option<f64>,
}

/// A wheel's steering limit: the contact point's (degrees), or the systems'
/// nose or steered-main-gear limit when given. Unsteered wheels stay fixed.
fn steering_limit(nose_gear: bool, cfg_deg: f64, nose: Option<f64>, body: Option<f64>) -> f64 {
    match (nose_gear, cfg_deg > 0.0) {
        (true, _) => nose.unwrap_or(cfg_deg),
        (false, true) => body.unwrap_or(cfg_deg),
        (false, false) => cfg_deg,
    }
}

/// A gear leg's length: from its fuselage or wing attach point down to its
/// own wheel, when that attach point is at least a foot above the wheel, or
/// the template's leg length when it is not (no attach point found, or one
/// so close to the wheel the leg would be a foot long or shorter). Also
/// reports which case it was: `_gear_y` is set to `bottom + radius + leg`
/// regardless of the branch, so the wheel's own rest height always matches
/// its flight_model.cfg contact point exactly either way -- but a leg that
/// fell back is no longer the length the model's own structure would give
/// it, which is worth knowing about.
fn gear_leg_length(attach: Option<f64>, bottom: f64, radius: f64, template_leg: f64) -> (f64, bool) {
    match attach.map(|a| a - bottom - radius).filter(|l| *l > 1.0) {
        Some(l) => (l, false),
        None => (template_leg, true),
    }
}

/// A main gear leg's `_strut_preload_def` (feet), crediting it for its own
/// ground-contact height's offset from the mains' mean: a leg installed
/// lower than the mean touches down, and starts compressing under load,
/// before a leg installed higher does, so it is given that much extra free
/// travel before its own preload force starts; a leg above the mean gives
/// some back. Clamped so the correction can never make a leg's preload
/// negative or send it past its own full travel.
fn gear_preload_def(base_def: f64, travel: f64, bottom: f64, mean_bottom: f64) -> f64 {
    (base_def - (bottom - mean_bottom)).clamp(0.0, travel)
}

/// X-Plane's ground-service and sensor points and the model part each is
/// taken from: the boarding doors jetways and stairs dock to, the catering
/// doors, the cargo doors the baggage loaders drive to, the external power
/// receptacle, the refuel point and the radio altimeter antenna. The
/// template's own are the A330's, which on the A380 put the baggage loaders
/// and the aft catering truck behind its tail and the radio altimeter 15 ft
/// past its tail cone. The A380 model has no node called a refuel coupling;
/// the meshes FlyByWire named `FUEL PUMP 1`, under the left wing root, are
/// the nearest thing to one.
const SERVICE_POINTS: &[(&str, &str)] = &[
    ("acf/_board_1", "PAX_DOOR_M1L"),
    ("acf/_board_2", "PAX_DOOR_M2L"),
    ("acf/_food_1", "PAX_DOOR_M1R"),
    ("acf/_food_2", "PAX_DOOR_M5R"),
    ("acf/_bagg_1", "FWD_CARGO_DOOR1"),
    ("acf/_bagg_2", "AFT_CARGO_DOOR"),
    ("acf/_bagg_3", "BULK_CARGO_DOOR"),
    ("acf/_ground_pwr", "EXT_PWR_PANELS"),
    ("acf/_fuel_1", "FUEL PUMP 1"),
    ("acf/_rad_alt_sens_xyz", "RADIO_ALTIMETER"),
];

/// Where a named model part meets the outside, in .acf feet: its lowest
/// point (a door's sill, an antenna's face), its middle fore and aft, and
/// across, its outer skin on the side it sits (its middle if it straddles
/// the centreline). Every mesh hanging from that node or below it counts.
/// `None` when there is no such part.
fn part_point(model: &Model, node_name: &str) -> Option<[f64; 3]> {
    let root = model.nodes.iter().position(|n| n.name == node_name)?;
    let under = |mut i: usize| loop {
        if i == root {
            return true;
        }
        match model.nodes[i].parent {
            Some(p) => i = p,
            None => return false,
        }
    };
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for m in model.meshes.iter().filter(|m| m.node.is_some_and(|n| under(n))) {
        for v in &m.vertices {
            let p = [-v.pos[0] as f64 * FT, v.pos[1] as f64 * FT, -v.pos[2] as f64 * FT];
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
    }
    if !lo[0].is_finite() {
        return None;
    }
    let mid = |k: usize| (lo[k] + hi[k]) / 2.0;
    let x = if mid(0) < -1.0 { lo[0] } else if mid(0) > 1.0 { hi[0] } else { mid(0) };
    Some([x, lo[1], mid(2)])
}

/// An engine's position, written everywhere Plane Maker writes it: the engine
/// (`_engn`) and its propeller or fan disc (`_blad`), which Plane Maker keeps
/// identical (the A330 template and FlightFactor's 777-F both do). Only the
/// `_engn` copy used to move, so the A380 kept the A330's two fan discs where
/// the A330's engines are -- x +/-30.8, y -9.3, z 92 ft, which on the A380 is
/// under the rear fuselage, one of them on the wrong side -- and its other two
/// engines had none.
fn place_engine(acf: &mut Acf, ei: usize, e: &[f64; 3]) {
    for fam in ["_engn", "_blad"] {
        for (axis, v) in ["x", "y", "z"].iter().zip(e) {
            acf.setf(&format!("{fam}/{ei}/_part_{axis}"), *v);
        }
    }
}

/// The template's own static leg loads at its maximum weight (lb): its nose
/// leg (`_gear/0`) and each of its main legs, from moments about its tyre
/// contacts. A leg's tyre is at the foot of the extended leg, which a
/// negative `_lonE` rakes aft (Plane Maker: positive angles lean the gear
/// forward). `None` when the template has no nose leg and mains to share.
fn template_leg_loads(acf: &Acf) -> Option<(f64, f64)> {
    let foot_z = |g: usize| {
        let z = acf.getf(&format!("_gear/{g}/_gear_z"))?;
        let leg = acf.getf(&format!("_gear/{g}/_leg_len")).unwrap_or(0.0);
        let lon = acf.getf(&format!("_gear/{g}/_lonE")).unwrap_or(0.0);
        Some(z - leg * lon.to_radians().sin())
    };
    let mains: Vec<f64> =
        (1..10).filter(|g| acf.getf(&format!("_gear/{g}/_gear_type")).is_some_and(|t| t > 0.0)).filter_map(foot_z).collect();
    let (nose_z, weight, cg) = (foot_z(0)?, acf.getf("acf/_m_max")?, acf.getf("acf/_cgZ")?);
    if mains.is_empty() || weight <= 0.0 {
        return None;
    }
    let main_z = mains.iter().sum::<f64>() / mains.len() as f64;
    if (main_z - nose_z).abs() < 1.0 {
        return None;
    }
    let nose = weight * ((main_z - cg) / (main_z - nose_z)).clamp(0.0, 1.0);
    Some((nose, (weight - nose) / mains.len() as f64))
}

/// A strut's `_damp` (lb per ft/s) that keeps the template leg's damping
/// ratio. Critical damping is 2*sqrt(k*m), so the ratio holds when the
/// damping scales as the square root of the spring rate *and* of the load
/// the leg carries. Scaling by the rate alone left the A380's nose leg --
/// three times the A330 template nose's load -- at a damping ratio of 0.23
/// against the template's 0.40 (Laminar's 777-F: 1.1), and the converted
/// aircraft chattered on it standing still: 140-500 deg/s^2 of pitch
/// acceleration at 1 deg/s of pitch rate, its tyres carrying 70-83% of the
/// weight, until the nose strut hit its stop and threw it into the air
/// (2026-09-26 and 09-29). The mains, at the template's own load, barely
/// move. Loads of zero fall back to the rate alone.
fn gear_damping(template_damp: f64, template_rate: f64, template_load_lb: f64, rate: f64, load_lb: f64) -> f64 {
    let load_ratio = if template_load_lb > 0.0 && load_lb > 0.0 { load_lb / template_load_lb } else { 1.0 };
    template_damp * (rate / template_rate.max(1.0) * load_ratio).sqrt()
}

/// Positions (degrees, airspeed limit) of a flight_model.cfg flap section.
fn flap_positions(fm: &Cfg, sect: &str) -> Vec<(f64, f64)> {
    let Some(fl) = fm.get(sect) else { return Vec::new() };
    (0..16)
        .filter_map(|i| fl.get(&format!("flaps-position.{i}")).map(|v| nums(v)))
        .filter(|v| v.len() >= 2)
        .map(|v| (v[0], v[1]))
        .collect()
}

/// The flap detents (trailing-edge flaps from the first `type = 1` section,
/// their last-detent speed) and the slat schedule per detent (the `type = 2`
/// leading-edge section, as a fraction of its largest angle), each overridden
/// by the systems' own angles when given. X-Plane keeps a detent angle per
/// handle position in `_flap1_dn`/`_flap2_dn` and the slats' deployment
/// ratio per position in `_slat1_dn`/`_slat2_dn` with their full travel in
/// `_slat{1,2}_dn_max_deg`; entries past the last detent are cleared (the
/// A330 template's sixth slat entry, 0.435, otherwise stays as FULL).
fn flap_schedule(fm: &Cfg, acf: &mut Acf, flaps: Option<&[f64]>, slats: Option<&[f64]>) -> Vec<String> {
    let mut report = Vec::new();
    let section = |ty: &str| {
        let mut names: Vec<&String> = fm.keys().filter(|k| k.starts_with("FLAPS.")).collect();
        names.sort();
        names.into_iter().find(|k| fm[*k].get("type").is_some_and(|t| t.trim() == ty)).cloned()
    };
    let te = section("1").map(|s| flap_positions(fm, &s)).unwrap_or_default();
    let mut deg: Vec<f64> = te.iter().map(|p| p.0).collect();
    let mut flap_src = "flight_model.cfg";
    if let Some(f) = flaps.filter(|f| f.len() >= 2) {
        // The systems' angles; a detent that repeats the one before keeps the
        // cfg's own (FlyByWire's CONF 1 is 0.01 there: X-Plane's detents must
        // rise).
        deg = f.iter().enumerate().map(|(i, &v)| if i > 0 && v <= f[i - 1] { te.get(i).map_or(v, |p| p.0.max(v)) } else { v }).collect();
        flap_src = "the systems";
    }
    if deg.len() >= 2 {
        for set in ["acf/_flap1_dn", "acf/_flap2_dn"] {
            let n = acf.getf(&format!("{set}/count")).unwrap_or(12.0) as usize;
            for i in 0..n {
                acf.setf(&format!("{set}/{i}"), deg.get(i).copied().unwrap_or(0.0));
            }
        }
        acf.set("acf/_flap_detents", deg.len() - 1);
        if let Some(&(_, v)) = te.last().filter(|p| p.1 > 0.0) {
            acf.setf("acf/_Vfem_kts", v);
        }
        report.push(format!("flaps: {} detents {:?} deg ({flap_src})", deg.len() - 1, deg));
    }
    let le: Vec<f64> = match slats.filter(|s| s.len() >= 2) {
        Some(s) => s.to_vec(),
        None => section("2").map(|s| flap_positions(fm, &s).iter().map(|p| p.0).collect()).unwrap_or_default(),
    };
    let max = le.iter().copied().fold(0.0, f64::max);
    if le.len() >= 2 && max > 0.0 {
        for set in ["acf/_slat1_dn", "acf/_slat2_dn"] {
            let n = acf.getf(&format!("{set}/count")).unwrap_or(12.0) as usize;
            for i in 0..n {
                acf.setf(&format!("{set}/{i}"), le.get(i).map_or(0.0, |d| (d / max).clamp(0.0, 1.0)));
            }
            acf.setf(&format!("{set}_max_deg"), max);
        }
        report.push(format!(
            "slats: {:?} deg per detent, {max} deg full travel ({})",
            le,
            if slats.is_some() { "the systems" } else { "flight_model.cfg leading-edge flaps" }
        ));
    }
    report
}

/// The highest-CL breakpoint of a flight_model.cfg `lift_coef_aoa_table`
/// ([AERODYNAMICS]): FlyByWire's own clean-wing CLmax, and the angle of
/// attack, in degrees, it occurs at. The table's angles are radians (its own
/// -3.15..3.15 range, near +/-180 degrees, gives that away).
fn lift_curve_peak(table: &str) -> Option<(f64, f64)> {
    table
        .split(',')
        .filter_map(|pair| {
            let (a, cl) = pair.trim().split_once(':')?;
            Some((a.trim().parse::<f64>().ok()?, cl.trim().parse::<f64>().ok()?))
        })
        .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(aoa_rad, cl)| (aoa_rad.to_degrees(), cl))
}

/// [AERODYNAMICS]/[STALL PROTECTION]/[FLAPS.*] tuning X-Plane's blade-element
/// model can take directly (XP-003): the flap family's lift/drag increment
/// (`_flap1_cl`/`_flap1_cd`; flight_model.cfg gives one combined figure, not
/// split leading/trailing edge, so the slats' own `_flap2_cl`/`_flap2_cd`
/// stay the template's), the stall/alpha-protection angle
/// (`_stall_warn_aoa`), and flap transit time. [FLIGHT_TUNING]'s
/// induced_drag_scalar/parasite_drag_scalar/flap_induced_drag_scalar have no
/// Plane Maker equivalent to set directly (X-Plane's induced and parasite
/// drag come from the wing's own geometry and airfoil polars, not a global
/// scalar); they are checked offline instead, against flight_model.cfg's own
/// reference stall speeds (docs/flight-model.md).
fn aero_tuning(fm: &Cfg, acf: &mut Acf) -> Vec<String> {
    let mut report = Vec::new();
    let aero = "AERODYNAMICS";
    if let (Some(cl), Some(cd)) = (num(fm, aero, "lift_coef_flaps"), num(fm, aero, "drag_coef_flaps")) {
        acf.setf("acf/_flap1_cl", cl);
        acf.setf("acf/_flap1_cd", cd);
        report.push(format!(
            "flap aero: trailing-edge Cl +{cl} Cd +{cd} (flight_model.cfg AERODYNAMICS lift_coef_flaps/drag_coef_flaps; \
             no separate leading-edge figure, so the slats' own _flap2_cl/_flap2_cd stay the template's)"
        ));
    }

    let peak = fm.get(aero).and_then(|s| s.get("lift_coef_aoa_table")).and_then(|t| lift_curve_peak(t));
    let on_limit = num(fm, "STALL PROTECTION", "on_limit");
    if let Some(aoa) = on_limit.or(peak.map(|(aoa, _)| aoa)) {
        acf.setf("acf/_stall_warn_aoa", aoa);
        report.push(match (on_limit, peak) {
            (Some(on), Some((paoa, cl))) => format!(
                "stall AoA: {on} deg ([STALL PROTECTION] on_limit, alpha protection's own trigger; \
                 the clean wing's own CLmax {cl:.2} peaks at {paoa:.1} deg in AERODYNAMICS' lift_coef_aoa_table, consistent with it)"
            ),
            (Some(on), None) => format!("stall AoA: {on} deg ([STALL PROTECTION] on_limit)"),
            _ => format!("stall AoA: {aoa:.1} deg (AERODYNAMICS lift_coef_aoa_table's own CLmax; no [STALL PROTECTION] on_limit given)"),
        });
    }

    let ext = fm.keys().filter(|k| k.starts_with("FLAPS.")).filter_map(|k| num(fm, k, "extending-time")).fold(0.0_f64, f64::max);
    if ext > 0.0 {
        acf.setf("acf/_flap_ext_time", ext);
        // FLAPS.* gives only one transit time (extending-time); using it for
        // retraction too is a documented approximation, not a fabricated
        // number, absent a separate retract figure.
        acf.setf("acf/_flap_ret_time", ext);
        report.push(format!("flap transit: {ext} sec extend and retract (flight_model.cfg FLAPS.* extending-time, slowest section)"));
    }

    // Offline calibration check (docs/flight-model.md): X-Plane derives its
    // own stall speed from the converted wing's geometry, not from a cfg
    // value, so the only way to catch a bad conversion is to predict what
    // that geometry implies (the textbook 1g lift equation) and compare it
    // against flight_model.cfg's own reference stall speeds.
    if let (Some(weight), Some(area), Some((_, clean_clmax))) = (
        num(fm, "WEIGHT_AND_BALANCE", "max_gross_weight"),
        num(fm, "AIRPLANE_GEOMETRY", "wing_area"),
        peak,
    ) {
        let flap_cl = num(fm, aero, "lift_coef_flaps").unwrap_or(0.0);
        let predicted_landing = stall_speed_kt(weight, area, clean_clmax + flap_cl);
        let predicted_clean = stall_speed_kt(weight, area, clean_clmax);
        let reference_landing = num(fm, "REFERENCE SPEEDS", "full_flaps_stall_speed");
        let reference_clean = num(fm, "REFERENCE SPEEDS", "flaps_up_stall_speed");
        let fmt_ref = |r: Option<f64>| r.map_or("-".to_string(), |v| format!("{v:.0}"));
        report.push(format!(
            "stall speed check: {predicted_landing:.0}/{predicted_clean:.0} kt predicted landing/clean from MTOW {weight:.0} lb, \
             wing area {area:.0} sq ft and CLmax {clean_clmax:.2}(+{flap_cl:.2} flaps), against flight_model.cfg's own \
             {}/{} kt reference (REFERENCE SPEEDS)",
            fmt_ref(reference_landing),
            fmt_ref(reference_clean)
        ));
        for (predicted, reference, label) in [(predicted_landing, reference_landing, "landing"), (predicted_clean, reference_clean, "clean")] {
            if let Some(reference) = reference.filter(|&r| r > 0.0) {
                if (predicted - reference).abs() > reference * 0.25 {
                    report.push(format!(
                        "WARNING: {label} stall speed check is {predicted:.0} kt against a {reference:.0} kt reference, over 25% off; \
                         check the converted wing's area and airfoils"
                    ));
                }
            }
        }
    }
    report
}

/// The real fuel tanks (`[FUEL_SYSTEM]`'s `tank.*`, dropping the sub-gallon
/// plumbing tanks MSFS uses for crossfeed and gravity feed), merged into
/// X-Plane's nine slots by combining the smallest tank into its nearest
/// neighbour on the same side until nine are left. Returns the report line,
/// or `None` when the cfg has no `[FUEL_SYSTEM]`.
fn fuel_tanks(fm: &Cfg, acf: &mut Acf, datum: [f64; 3]) -> Option<String> {
    let fs = fm.get("FUEL_SYSTEM")?;
    let mut tanks: Vec<(String, f64, [f64; 3])> = fs
        .iter()
        .filter(|(k, _)| k.starts_with("tank."))
        .filter_map(|(_, v)| {
            let field = |name: &str| v.split('#').find_map(|f| f.strip_prefix(&format!("{name}:")).map(str::to_string));
            let cap: f64 = field("Capacity")?.parse().ok()?;
            let pos = nums(&field("Position")?);
            (cap > 1.0 && pos.len() >= 3).then(|| (field("Title").unwrap_or_default(), cap, acf_point(pos[0], pos[1], pos[2], datum)))
        })
        .collect();
    while tanks.len() > 9 {
        // Merge the smallest tank into the nearest one on its side.
        let (i, _) = tanks.iter().enumerate().min_by(|a, b| a.1 .1.total_cmp(&b.1 .1)).unwrap();
        let t = tanks.remove(i);
        let d = |p: &[f64; 3]| (p[0] - t.2[0]).hypot(p[2] - t.2[2]);
        let j = tanks
            .iter()
            .enumerate()
            .filter(|(_, o)| (o.2[0] >= 0.0) == (t.2[0] >= 0.0))
            .min_by(|a, b| d(&a.1 .2).total_cmp(&d(&b.1 .2)))
            .map_or(0, |(j, _)| j);
        let o = &mut tanks[j];
        let cap = o.1 + t.1;
        for k in 0..3 {
            o.2[k] = (o.2[k] * o.1 + t.2[k] * t.1) / cap;
        }
        o.0 = format!("{} + {}", o.0, t.0);
        o.1 = cap;
    }
    let total: f64 = tanks.iter().map(|t| t.1).sum();
    // Jet fuel: 6.7 lb per US gallon.
    acf.setf("acf/_m_fuel_max_tot", total * 6.7);
    for i in 0..9 {
        match tanks.get(i) {
            Some((name, cap, p)) => {
                acf.set(&format!("acf/_tank_name/{i}"), name);
                acf.setf(&format!("acf/_tank_rat/{i}"), cap / total);
                // The template's own per-slot ramp-start fill level
                // (_tank_rat_def) is keyed to the A330's nine tanks (e.g.
                // its centre tank empty by default, its mains 80%);
                // flight_model.cfg's FUEL_SYSTEM carries no equivalent
                // default-fill figure to re-key it to this aircraft's own
                // eleven tanks merged into nine, so a slot's old default now
                // names a fuel state that belongs to a different, unrelated
                // tank (previously left untouched: a feed tank inherited the
                // A330's "0%", which starves it before its pumps ever run).
                // Full is the only default X-Plane can show consistently
                // once the identity behind each slot has changed.
                acf.setf(&format!("acf/_tank_rat_def/{i}"), 1.0);
                for (k, v) in p.iter().enumerate() {
                    acf.setf(&format!("acf/_tank_xyz/{i},{k}"), *v);
                    acf.setf(&format!("acf/_tank_xyz_full/{i},{k}"), *v);
                }
            }
            None => {
                acf.set(&format!("acf/_tank_name/{i}"), "");
                acf.setf(&format!("acf/_tank_rat/{i}"), 0.0);
                acf.setf(&format!("acf/_tank_rat_def/{i}"), 0.0);
            }
        }
    }
    Some(format!("fuel: {:.0} US gal ({:.0} lb) in {} tanks (flight_model.cfg)", total, total * 6.7, tanks.len()))
}

/// The .acf text and a report of what came from where.
pub fn build(inp: &Inputs) -> anyhow::Result<(String, Vec<String>)> {
    let mut acf = Acf::parse(&std::fs::read_to_string(inp.template).with_context(|| format!("reading {}", inp.template.display()))?)?;
    let fm = read_cfg(&inp.cfg_dir.join("flight_model.cfg"));
    let en = read_cfg(&inp.cfg_dir.join("engines.cfg"));
    let ac = read_cfg(&inp.cfg_dir.join("aircraft.cfg"));
    if fm.is_empty() {
        bail!("no flight_model.cfg in {}", inp.cfg_dir.display());
    }
    let mut report = Vec::new();
    let wb = "WEIGHT_AND_BALANCE";
    let geo = "AIRPLANE_GEOMETRY";
    let datum = fm.get(wb).and_then(|s| s.get("reference_datum_position")).map(|v| nums(v)).unwrap_or_default();
    let datum = [datum.first().copied().unwrap_or(0.0), datum.get(1).copied().unwrap_or(0.0), datum.get(2).copied().unwrap_or(0.0)];

    // Identity.
    let icao = ac.get("GENERAL").and_then(|s| s.get("icao_type_designator")).cloned().unwrap_or_default();
    acf.set("acf/_name", inp.name);
    if !icao.is_empty() {
        acf.set("acf/_ICAO", &icao);
    }
    acf.set("acf/_author", "FlyByWire Simulations (GPL-3.0); converted by msfs2xp-aircraft");
    acf.set("acf/_descrip", format!("{} converted from MSFS", inp.name));
    // The template's own paint name ("Lufthansa D-AIKN") would label the
    // default livery.
    acf.set("acf/_default_livery_name", inp.livery);
    acf.set("acf/_manufacturer", "Airbus");
    // Systems and avionics: X-Plane's generic behaviour, not the template's
    // Laminar Airbus setup. Its Airbus autopilot logic (12) and fly-by-wire
    // run Laminar's FMGS, which needs the template's MCDU; X-Plane refuses to
    // load the aircraft without one. These are the values Laminar's other
    // airliners (757, 767, 747, Citation X) use; the FBW systems port will
    // bring its own flight laws.
    for (k, v) in [
        // X-Plane's built-in airliner autopilot and FMS; for an Airbus it is
        // the FMGS, which demands an MCDU in the cockpit.
        ("acf/_custom_autopilot", 0),
        ("acf/_autopilot_logic", 0),
        ("acf/_airbus_FBW", 0),
        ("acf/_dual_control_type", 0),
        ("acf/_EFIS2_on_avio", 1),
        ("acf/_integrated_approach_navigation", 0),
        ("acf/_fdir_needed_to_engage_servo", 0),
        ("acf/_AP_overspeed_prot", 1),
        ("acf/_hnav_loc_only", 0),
        ("acf/_require_gps_capture", 0),
        ("acf/_hdng_acft_trim_mode", 0),
        ("acf/_ptch_acft_trim_mode", 0),
        ("acf/_roll_acft_trim_mode", 0),
        ("acf/_starter_requires_mode_sel", 0),
        ("acf/_vor_mode_has_dead_reckon", 0),
    ] {
        acf.set(k, v);
    }
    report.push("systems: X-Plane generic autopilot and avionics (no Laminar Airbus FMGS or fly-by-wire)".into());

    // The pilot's eye: MSFS's eyepoint (cameras.cfg [VIEWS], feet from the
    // datum) plus its pilot camera's offset (metres; x right, y up, z
    // forward). The copilot sits mirrored.
    let cams = read_cfg(&inp.cfg_dir.join("cameras.cfg"));
    // The eye in metres, the objects' own frame (see `casts_into_flight_deck`).
    let mut eye_m: Option<[f64; 3]> = None;
    let clean = |v: &String| nums(v.split(';').next().unwrap_or(""));
    if let Some(eye) = cams.get("VIEWS").and_then(|s| s.get("eyepoint")).map(clean).filter(|v| v.len() >= 3) {
        let off = cams
            .values()
            .find(|s| s.get("title").is_some_and(|t| t.trim().trim_matches('"').eq_ignore_ascii_case("pilot")))
            .and_then(|s| s.get("initialxyz"))
            .map(clean)
            .filter(|v| v.len() >= 3)
            .unwrap_or_else(|| vec![0.0; 3]);
        const M_FT: f64 = 3.280_839_895;
        let p = acf_point(eye[0] + off[2] * M_FT, eye[1] + off[0] * M_FT, eye[2] + off[1] * M_FT, datum);
        eye_m = Some([p[0] / M_FT, p[1] / M_FT, p[2] / M_FT]);
        for (k, v) in p.iter().enumerate() {
            acf.setf(&format!("acf/_pe_xyz/{k}"), *v);
            acf.setf(&format!("acf/_pe_xyz_copilot/{k}"), if k == 0 { -*v } else { *v });
        }
        report.push(format!(
            "pilot eye: x {:.1} y {:.1} z {:.1} ft (cameras.cfg eyepoint and pilot camera)",
            p[0], p[1], p[2]
        ));
    }

    // Fuselage first: the wing root and gear attach to it.
    // Cabin sections set the width; the tail (FUSE4) is trimmed below and
    // the wing-body fairing (FUSE5) is a body of its own.
    let fuse_all = verts(inp.exterior, &["FUSE"], &["FUSE4", "FUSE5"]);
    let (fz0, fz1) = span_of(&fuse_all, 2);
    let mid_top = fuse_all
        .iter()
        .filter(|p| p[2] > fz0 + 0.2 * (fz1 - fz0) && p[2] < fz1 - 0.2 * (fz1 - fz0))
        .map(|p| p[1])
        .fold(f64::MIN, f64::max);
    let half_width = fuse_all.iter().map(|p| p[0].abs()).fold(0.0, f64::max);
    let tail = verts(inp.exterior, &["FUSE4"], &[]);
    let fuse: Vec<[f64; 3]> = fuse_all
        .iter()
        .chain(tail.iter().filter(|p| p[0].abs() <= half_width && p[1] <= mid_top))
        .copied()
        .collect();
    let (nose, tail_z) = span_of(&fuse, 2);
    let fus_rings = if fuse.is_empty() {
        None
    } else {
        let zs: Vec<f64> = station_fractions(&acf, 0).iter().map(|f| nose + f * (tail_z - nose)).collect();
        let rings = body_rings(&fuse, &zs, [0.0; 3]);
        write_body(&mut acf, 0, [0.0; 3], &rings);
        report.push(format!(
            "fuselage: {:.1} ft long, {:.1} ft wide, {:.1} ft tall (from the model; MSFS says {} ft long)",
            tail_z - nose,
            2.0 * half_width,
            mid_top - fuse.iter().map(|p| p[1]).fold(f64::MAX, f64::min),
            num(&fm, geo, "fuselage_length").unwrap_or(0.0)
        ));
        Some((zs, rings))
    };
    // The bottom of the fuselage at a station (for gear attach points).
    let belly = |z: f64| -> Option<f64> {
        let (zs, rings) = fus_rings.as_ref()?;
        let i = zs.iter().enumerate().min_by(|a, b| (a.1 - z).abs().total_cmp(&(b.1 - z).abs()))?.0;
        Some(rings[i][8][1])
    };
    let top_at = |z: f64| -> Option<f64> {
        let (zs, rings) = fus_rings.as_ref()?;
        let i = zs.iter().enumerate().min_by(|a, b| (a.1 - z).abs().total_cmp(&(b.1 - z).abs()))?.0;
        Some(rings[i][0][1])
    };
    // The fuselage's half width at a station.
    let half_at = |z: f64| -> Option<f64> {
        let (zs, rings) = fus_rings.as_ref()?;
        let i = zs.iter().enumerate().min_by(|a, b| (a.1 - z).abs().total_cmp(&(b.1 - z).abs()))?.0;
        Some(rings[i].iter().map(|p| p[0].abs()).fold(0.0, f64::max))
    };
    let fairing = verts(inp.exterior, &["FUSE5"], &[]);
    if !fairing.is_empty() {
        let (a, b) = span_of(&fairing, 2);
        let zs: Vec<f64> = station_fractions(&acf, 1).iter().map(|f| a + f * (b - a)).collect();
        write_body(&mut acf, 1, [0.0; 3], &body_rings(&fairing, &zs, [0.0; 3]));
    }

    // Main wing: four segments per side at the template's span fractions.
    let wing = tris(inp.exterior, &["WING"], &["FENCE"]);
    let mut wing_info = None;
    if !wing.is_empty() {
        let st = smooth(slice(&wing, |p| p[0].abs(), |p| p[1], 1.0));
        // Outboard of the wing-body junction, whose glove and fillet are not
        // the wing's planform.
        let st: Vec<Station> = st.into_iter().filter(|s| s.s >= half_width + 3.0).collect();
        debug_stations("wing", &st);
        // The true tip: up to a slice beyond the last station.
        let tip = wing.iter().flatten().map(|p| p[0].abs()).fold(0.0, f64::max);
        let inc0 = num(&fm, geo, "wing_incidence").unwrap_or(0.0);
        let twist = num(&fm, geo, "wing_twist").unwrap_or(0.0);
        // The template's segment breaks, as fractions of its semispan.
        let tx: Vec<f64> = [2usize, 4, 6].iter().map(|i| acf.getf(&format!("_wing/{i}/_part_x")).unwrap_or(0.0).abs()).collect();
        let t_tip = {
            let w6 = |k: &str| acf.getf(&format!("_wing/6/{k}")).unwrap_or(0.0);
            tx[2] + w6("_semilen_SEG") * w6("_sweep_design").to_radians().cos() * w6("_dihed_design").to_radians().cos()
        };
        let breaks = [0.0, tx[0] / t_tip * tip, tx[1] / t_tip * tip, tx[2] / t_tip * tip, tip];
        // The root at the centreline: leading and trailing edges carried
        // inboard along their lines over the inner wing, which is the
        // reference planform both MSFS's wing area and X-Plane's wing use.
        let inner: Vec<&Station> = st.iter().filter(|s| s.s <= breaks[1]).collect();
        let root = Station {
            s: 0.0,
            le: intercept(&inner, |s| s.le),
            te: intercept(&inner, |s| s.te),
            h: intercept(&inner, |s| s.h),
            low: inner.first().map_or(0.0, |s| s.low),
        };
        // Dihedral: the published figure (flight_model.cfg) when there is
        // one, since the model sits in its drooped ground shape; the wing
        // keeps the height the model gives it at the fuselage.
        let dihedral = num(&fm, geo, "wing_dihedral");
        let (root, st) = match dihedral {
            Some(d) if !st.is_empty() => {
                let t = d.to_radians().tan();
                let h0 = st[0].h - st[0].s * t;
                let st: Vec<Station> = st.iter().map(|s| Station { h: h0 + s.s * t, ..*s }).collect();
                (Station { h: h0, ..root }, st)
            }
            _ => (root, st),
        };
        if let Some(d) = dihedral {
            report.push(format!("wing dihedral: {d} deg on every segment (flight_model.cfg; the model rests in its drooped ground shape)"));
        }
        for seg in 0..4 {
            let a = if seg == 0 { root } else { at(&st, breaks[seg]) };
            let b = at(&st, breaks[seg + 1]);
            let surf = horizontal(&a, breaks[seg], &b);
            let (ea, eb) = (breaks[seg] / tip, breaks[seg + 1] / tip);
            let inc = move |f: f64| inc0 + twist * (ea + (eb - ea) * f);
            for i in [seg * 2, seg * 2 + 1] {
                let right = acf.getf(&format!("_wing/{i}/_is_right_mult")).unwrap_or(1.0) > 0.0;
                write_surface(&mut acf, i, &surf, right, &inc);
            }
        }
        // Area, mean aerodynamic chord and where it starts.
        let full: Vec<Station> = std::iter::once(root).chain(st.iter().copied()).collect();
        let (mut s_half, mut c2, mut cle) = (0.0, 0.0, 0.0);
        for w in full.windows(2) {
            let ds = w[1].s - w[0].s;
            let c = (w[0].chord() + w[1].chord()) / 2.0;
            s_half += c * ds;
            c2 += c * c * ds;
            cle += c * (w[0].le + w[1].le) / 2.0 * ds;
        }
        let mac = c2 / s_half;
        let le_mac = cle / s_half;
        report.push(format!(
            "wing: span {:.1} ft (MSFS {}), area {:.0} sq ft (MSFS {}), root chord {:.1} ft (MSFS {}), MAC {:.1} ft",
            2.0 * tip,
            num(&fm, geo, "wing_span").unwrap_or(0.0),
            2.0 * s_half,
            num(&fm, geo, "wing_area").unwrap_or(0.0),
            root.chord(),
            num(&fm, geo, "wing_root_chord").unwrap_or(0.0),
            mac
        ));
        wing_info = Some((st, mac, le_mac));
    } else {
        report.push("wing: no wing meshes found; the template's wing is left as it is".into());
    }
    wing_flex(&mut acf);
    report.push(format!("wing flex: {WING_DIHEDRAL_PER_G} deg of mid-span dihedral per g (the A330 template's 1.5 put the outer nacelles on the runway)"));

    // Tailplane: points near the MSFS tail position, outside the tail cone.
    let all = tris(inp.exterior, &[], &["WING", "FENCE", "PYLON", "ENG", "FAN", "BLUR"]);
    let htz = -num(&fm, geo, "htail_pos_lon").unwrap_or(-100.0) - datum[0];
    let hty = num(&fm, geo, "htail_pos_vert").unwrap_or(15.0) + datum[2];
    // Points outside the tail cone at their own station, so the cone's sides
    // do not count as tailplane.
    let stab: Vec<Tri> = all
        .iter()
        .filter(|t| {
            let p = centroid(t);
            (p[2] - htz).abs() < 50.0 && (p[1] - hty).abs() < 10.0 && p[0].abs() > half_at(p[2]).unwrap_or(6.0) + 1.0
        })
        .copied()
        .collect();
    if !stab.is_empty() {
        let st = smooth(slice(&stab, |p| p[0].abs(), |p| p[1], 1.0));
        debug_stations("tailplane", &st);
        // Straight edges fitted over the exposed panel and carried to the
        // centreline, as for the wing's reference planform.
        let tip = stab.iter().flatten().map(|p| p[0].abs()).fold(0.0, f64::max);
        if let Some((root, b)) = fitted(&st, 0.0, tip) {
            let b = &b;
            let surf = horizontal(&root, 0.0, b);
            let inc = num(&fm, geo, "htail_incidence").unwrap_or(0.0);
            for i in [8usize, 9] {
                let right = acf.getf(&format!("_wing/{i}/_is_right_mult")).unwrap_or(1.0) > 0.0;
                write_surface(&mut acf, i, &surf, right, &|_| inc);
            }
            let area = (root.chord() + b.chord()) / 2.0 * b.s * 2.0;
            report.push(format!(
                "tailplane: span {:.1} ft (MSFS {}), area {:.0} sq ft (MSFS {}), sweep {:.1} (MSFS {})",
                2.0 * b.s,
                num(&fm, geo, "htail_span").unwrap_or(0.0),
                area,
                num(&fm, geo, "htail_area").unwrap_or(0.0),
                surf.sweep,
                num(&fm, geo, "htail_sweep").unwrap_or(0.0)
            ));
        }
    }

    // Fin: thin points above the fuselage near the MSFS fin position.
    let vtz = -num(&fm, geo, "vtail_pos_lon").unwrap_or(-90.0) - datum[0];
    let base = top_at(vtz).unwrap_or(mid_top);
    // Thin points above the fuselage crown at their own station, so the
    // crown ahead of the fin does not count as fin.
    let fin: Vec<Tri> = all
        .iter()
        .filter(|t| {
            let p = centroid(t);
            p[0].abs() < 3.0 && (p[2] - vtz).abs() < 60.0 && p[1] > base && p[1] > top_at(p[2]).unwrap_or(base) + 1.0
        })
        .copied()
        .collect();
    if !fin.is_empty() {
        // Clear of the crown, fillet and tail cone: 3 ft above the fuselage.
        let st: Vec<Station> = smooth(slice(&fin, |p| p[1], |p| p[0], 1.0)).into_iter().filter(|s| s.s >= base + 3.0).collect();
        debug_stations("fin", &st);
        let top = fin.iter().flatten().map(|p| p[1]).fold(f64::MIN, f64::max);
        if let Some((a, b)) = st.first().and_then(|first| fitted(&st, first.s, top)) {
            let (a, b) = (&a, &b);
            let mut surf = vertical(a, b);
            // The fin stands on the centreline, whatever its thickness.
            surf.root[0] = 0.0;
            write_surface(&mut acf, 10, &surf, true, &|_| 0.0);
            let area = (a.chord() + b.chord()) / 2.0 * (b.s - a.s);
            report.push(format!(
                "fin: height {:.1} ft (MSFS {}), area {:.0} sq ft (MSFS {}), sweep {:.1} (MSFS {})",
                b.s - a.s,
                num(&fm, geo, "vtail_span").unwrap_or(0.0),
                area,
                num(&fm, geo, "vtail_area").unwrap_or(0.0),
                surf.sweep,
                num(&fm, geo, "vtail_sweep").unwrap_or(0.0)
            ));
        }
    }

    // Wingtip fences in the template's winglet slots.
    let fence = tris(inp.exterior, &["FENCE"], &[]);
    for (i, right) in [(12usize, true), (13, false)] {
        let pts: Vec<Tri> = fence.iter().filter(|t| (centroid(t)[0] > 0.0) == right).copied().collect();
        let st = slice(&pts, |p| p[1], |p| p[0], 0.5);
        if let Some((a, b)) = mean_panel(&st) {
            write_surface(&mut acf, i, &vertical(&a, &b), right, &|_| 0.0);
        }
    }

    // Engines, their nacelles and pylons.
    let gen = en.get("GENERALENGINEDATA");
    let engines: Vec<[f64; 3]> = (0..8)
        .filter_map(|i| gen?.get(&format!("engine.{i}")).map(|v| nums(v)))
        .filter(|v| v.len() >= 3)
        .map(|v| acf_point(v[0], v[1], v[2], datum))
        .collect();
    if !engines.is_empty() {
        let thrust = num(&en, "TURBINEENGINEDATA", "static_thrust").unwrap_or(0.0) * num(&en, "JET_ENGINE", "thrust_scalar").unwrap_or(1.0);
        // The template's left and right engine, nacelle and pylon.
        let right_of = |acf: &Acf, fam: &str, i: usize| acf.getf(&format!("{fam}/{i}/_part_x")).unwrap_or(0.0) > 0.0;
        let (le, re) = if right_of(&acf, "_engn", 1) { (0, 1) } else { (1, 0) };
        let (lb, rb) = if right_of(&acf, "_body", 22) { (21, 22) } else { (22, 21) };
        let (lp, rp) = if acf.getf("_wing/33/_is_right_mult").unwrap_or(-1.0) > 0.0 { (32, 33) } else { (33, 32) };
        let snap = |acf: &Acf, fam: &str, i: usize| acf.with_prefix(&format!("{fam}/{i}/"));
        let (se_l, se_r) = (snap(&acf, "_engn", le), snap(&acf, "_engn", re));
        let (sp_l, sp_r) = (snap(&acf, "_prop", le), snap(&acf, "_prop", re));
        let (sd_l, sd_r) = (snap(&acf, "_blad", le), snap(&acf, "_blad", re));
        let (sb_l, sb_r) = (snap(&acf, "_body", lb), snap(&acf, "_body", rb));
        let (sw_l, sw_r) = (snap(&acf, "_wing", lp), snap(&acf, "_wing", rp));
        let restore = |acf: &mut Acf, snap: &[(String, String)], fam: &str, from: usize, to: usize| {
            let src = format!("{fam}/{from}/");
            for (k, v) in snap {
                acf.set(&format!("{fam}/{to}/{}", &k[src.len()..]), v);
            }
        };
        let nacelle = verts(inp.exterior, &["ENG_LH", "ENG_RH"], &["CORE"]);
        let pylon = tris(inp.exterior, &["PYLON"], &[]);
        let nearest = |p: &[f64; 3]| {
            engines
                .iter()
                .enumerate()
                .min_by(|a, b| (a.1[0] - p[0]).abs().total_cmp(&(b.1[0] - p[0]).abs()))
                .map_or(0, |(i, _)| i)
        };
        let n_fracs = station_fractions(&acf, lb);
        for (i, e) in engines.iter().enumerate() {
            let right = e[0] > 0.0;
            let (se, sp, sd, sb, sw) =
                if right { (&se_r, &sp_r, &sd_r, &sb_r, &sw_r) } else { (&se_l, &sp_l, &sd_l, &sb_l, &sw_l) };
            let (ei, bi) = (i, 21 + i);
            restore(&mut acf, se, "_engn", if right { re } else { le }, ei);
            restore(&mut acf, sp, "_prop", if right { re } else { le }, ei);
            restore(&mut acf, sd, "_blad", if right { re } else { le }, ei);
            restore(&mut acf, sb, "_body", if right { rb } else { lb }, bi);
            place_engine(&mut acf, ei, e);
            if thrust > 0.0 {
                acf.setf(&format!("_engn/{ei}/_thrust_max_limit"), thrust);
            }
            acf.set(&format!("_body/{bi}/_engn_for_body"), ei);
            let pts: Vec<[f64; 3]> = nacelle.iter().filter(|p| nearest(p) == i).copied().collect();
            if !pts.is_empty() {
                let (a, b) = span_of(&pts, 2);
                let zs: Vec<f64> = n_fracs.iter().map(|f| a + f * (b - a)).collect();
                write_body(&mut acf, bi, *e, &body_rings(&pts, &zs, *e));
            }
            // Pylons in wing slots 32 upwards.
            let wi = 32 + i;
            restore(&mut acf, sw, "_wing", if right { rp } else { lp }, wi);
            let pts: Vec<Tri> = pylon.iter().filter(|t| nearest(&centroid(t)) == i).copied().collect();
            let st = slice(&pts, |p| p[1], |p| p[0], 0.5);
            if let Some((a, mut b)) = mean_panel(&st) {
                // Up to the wing's underside above the engine: the pylon's
                // triangles run on into its fairing over the wing, and their
                // full height made a 7 ft fin above each engine.
                if let Some((wst, _, _)) = wing_info.as_ref() {
                    let under = at(wst, a.h.abs()).low;
                    if under > a.s + 1.0 && under < b.s {
                        b.s = under;
                    }
                }
                write_surface(&mut acf, wi, &vertical(&a, &b), right, &|_| 0.0);
            }
        }
        acf.set("acf/_num_engn", engines.len());
        report.push(format!(
            "engines: {} at the engines.cfg positions, {:.0} lbf static thrust each (engines.cfg)",
            engines.len(),
            thrust
        ));
    }

    // Gear: every wheel contact point.
    if let Some(cp) = fm.get("CONTACT_POINTS") {
        let wheels: Vec<Vec<f64>> = (0..40)
            .filter_map(|i| cp.get(&format!("point.{i}")).map(|v| nums(v)))
            .filter(|v| v.len() >= 8 && v[0] == 1.0)
            .collect();
        let t_nose = acf.with_prefix("_gear/0/");
        let t_main = acf.with_prefix("_gear/1/");
        // Read before the legs below overwrite the template's gear (and
        // before the weights section replaces its CG and maximum weight).
        let template_loads = template_leg_loads(&acf);
        let mains = wheels.iter().filter(|w| w[7] < 30.0).count().max(1) as f64;
        let mtow = num(&fm, wb, "max_gross_weight").unwrap_or(acf.getf("acf/_m_max").unwrap_or(0.0));
        // How the weight divides between the nose leg and the mains standing
        // still: moments about the main gear line, with the CG and the nose
        // leg both ahead of it. MSFS measures z forward from the datum, and
        // the contact points and the CG share that datum, so the raw cfg
        // numbers subtract directly.
        let main_z = wheels.iter().filter(|w| w[7] < 30.0).map(|w| w[1]).sum::<f64>() / mains;
        let nose_z = wheels.iter().find(|w| w[7] >= 30.0).map_or(main_z, |w| w[1]);
        let cg_z = fm
            .get(wb)
            .and_then(|s| s.get("empty_weight_cg_position"))
            .map(|v| nums(v))
            .and_then(|v| v.first().copied())
            .unwrap_or(main_z);
        let nose_share = if (nose_z - main_z).abs() > 1.0 {
            ((cg_z - main_z) / (nose_z - main_z)).clamp(0.02, 0.30)
        } else {
            0.10
        };
        // Field 9 of a contact point is the maximum compression in feet when
        // the section asks for it, and a multiple of field 8 otherwise.
        let max_comp_ft = cp.get("set_max_compression").map(|v| v.trim().starts_with('1')).unwrap_or(false);
        // Pass 1: the mains' own mean ground-contact height, so pass 2 below
        // (inside the loop) can credit each main leg a preload correction
        // for its offset from it. The A380's own contact points put the
        // body mains 0.12 ft lower than the wing mains, and every main
        // shares one spring rate (below), which without this correction
        // lets the lower legs take the load alone until the higher ones
        // catch up.
        let main_bottom_mean = {
            let bottoms: Vec<f64> = wheels.iter().filter(|w| w[7] < 30.0).map(|w| acf_point(w[1], w[2], w[3], datum)[1]).collect();
            if bottoms.is_empty() { 0.0 } else { bottoms.iter().sum::<f64>() / bottoms.len() as f64 }
        };
        let mut rows = Vec::new();
        let mut fallback_legs = Vec::new();
        for (gi, w) in wheels.iter().enumerate() {
            let nose_gear = w[7] >= 30.0;
            let src = if nose_gear { &t_nose } else { &t_main };
            let from = if nose_gear { "_gear/0/" } else { "_gear/1/" };
            for (k, v) in src {
                acf.set(&format!("_gear/{gi}/{}", &k[from.len()..]), v);
            }
            let p = acf_point(w[1], w[2], w[3], datum);
            let radius = w[6];
            let bottom = p[1];
            // Attach at the fuselage belly, or under the wing for wing gear.
            let attach = if p[0].abs() > half_width {
                wing_info.as_ref().map(|(st, _, _)| at(st, p[0].abs()).low)
            } else {
                belly(p[2])
            };
            let template_leg = acf.getf(&format!("_gear/{gi}/_leg_len")).unwrap_or(5.0);
            let (leg, used_fallback) = gear_leg_length(attach, bottom, radius, template_leg);
            if used_fallback {
                fallback_legs.push(format!("{}{:.0}/{:.0}", if nose_gear { "nose " } else { "" }, p[0], p[2]));
            }
            acf.setf(&format!("_gear/{gi}/_gear_x"), p[0]);
            acf.setf(&format!("_gear/{gi}/_gear_y"), bottom + radius + leg);
            acf.setf(&format!("_gear/{gi}/_gear_z"), p[2]);
            acf.setf(&format!("_gear/{gi}/_leg_len"), leg);
            acf.setf(&format!("_gear/{gi}/_tire_radius"), radius);
            // Steering: the contact point's, or the systems' own actuator
            // limits when given (FlyByWire's A380X steers the nose 75 deg and
            // the body gear 15 deg, a380_systems hydraulic/mod.rs:1792-1814,
            // where flight_model.cfg says 70 and 8).
            let steer = steering_limit(nose_gear, w[7], inp.nose_steering, inp.body_steering);
            acf.setf(&format!("_gear/{gi}/_steerdeg_lospeed"), steer);
            // The strut's spring curve, from the contact point's own
            // numbers: field 8 is how far the leg compresses under its share
            // of the weight standing still, field 9 how far it can travel.
            // A leg's rate is therefore its share of maximum weight over its
            // static compression, and the force at full travel follows.
            //
            // Only the forces used to be rescaled; the deflections stayed as
            // the A330 template left them, which on the A380 put the nose's
            // full force at 0.70 ft of travel and the mains' preload ramp at
            // 0.80 ft. The mains could not take any load until the nose had
            // already bottomed out, so the aircraft stood on its nose leg --
            // 1.47 MN of a 3.09 MN aircraft on one strut, 37 kN on each main
            // -- sank, and X-Plane called it a crash on the runway. MSFS's
            // own figures say the opposite of the template's: the A380's
            // nose strut is the soft one (static 1.29 ft, travel 1.20 ft)
            // and the mains are stiff (0.95 ft, 2.40 ft).
            let share = if nose_gear { nose_share } else { (1.0 - nose_share) / mains };
            let stat_c = w.get(8).copied().filter(|c| *c > 0.01);
            let max_c = w
                .get(9)
                .copied()
                .filter(|c| *c > 0.01)
                .map(|c| if max_comp_ft { c } else { c * stat_c.unwrap_or(0.0) })
                .filter(|c| *c > 0.01);
            if let (Some(stat_c), Some(max_c)) = (stat_c, max_c) {
                let rate = share * mtow / stat_c;
                // A leg needs room above where it sits: travel of at least
                // twice its static compression, so settling onto it has
                // somewhere to go. The A380's nose contact point does not
                // give that -- MSFS's own numbers have it compressing 1.29
                // ft under its share of maximum weight with 1.20 ft of
                // travel, 107% of the leg, bottomed before it has even been
                // loaded -- so field 9 cannot be taken literally there.
                // Taken literally it was: the nose reached 94% of its travel
                // settling onto an empty runway and X-Plane called it a
                // crash. The mains are unaffected (40% of theirs).
                //
                // XP-### (tyres sinking into the runway): that "mains
                // unaffected" is no longer true against the A380's current
                // flight_model.cfg -- the mains' own field 9 ratio is 1.25
                // here (point.1-4), the same shape of problem the nose had
                // (1.2048), just milder, so `stat_c / 0.5` (needing 2x
                // static) now wins over `max_c` for every leg, not only the
                // nose: nose travel becomes 2.59 ft (0.789 m) against a raw
                // max_c of 1.56 ft (0.476 m); body 2.76 ft (0.841 m) against
                // 1.72 ft (0.525 m); wing 2.63 ft (0.801 m) against 1.64 ft
                // (0.501 m) -- roughly 1.6x the contact point's own number on
                // every leg. rig.rs's `exterior()` measures each leg's real
                // compression clip independently (`gear_travel_m`) and gets
                // almost exactly these same raw, un-doubled figures (the
                // nose's clip moves its wheel node ~0.48 m, not 0.79 m) --
                // the model's own artwork was built to the contact points as
                // MSFS wrote them, not to this floor. No animation can show
                // compression past its own last keyframe, so once real
                // deflection (`tire_vertical_deflection_mtr`) passes that
                // clip's travel the wheel mesh stops rising while this floor
                // keeps letting the fuselage settle further -- the tyre
                // renders sunk into the runway by the shortfall. rig.rs's
                // `comp()` now divides by each leg's own measured travel
                // instead of a flat 0.5 m, which removes the small (~5%)
                // per-leg mismatch that was always there, but cannot close
                // this much larger (~60%) one: physics is allowed to
                // compress the strut well past anything the model can draw.
                //
                // Not changed here: capping `travel` at `max_c` would very
                // likely fix the render (and was this aircraft's own,
                // odds-on-correct number before this floor existed to save
                // the nose), but this exact floor is also the fix for a
                // real crash-on-load bug (above), and there is no way to
                // tell from the cfg alone whether today's mains, at today's
                // weights, ever get close enough to `max_c` to reproduce
                // it -- that needs a runtime check, not another guess:
                // log `sim/flightmodel2/gear/tire_vertical_deflection_mtr[0..4]`
                // and `.../tire_vertical_force_n_mtr[0..4]` at rest and
                // during a firm touchdown, and compare each leg's deflection
                // against its own `_strut_max_wgt_def` (this `travel`, in
                // feet) both with this floor and with it capped at `max_c`,
                // before loosening a fix that was put here for a documented
                // crash.
                let travel = max_c.max(stat_c / 0.5);
                let force = rate * travel;
                let old_rate = acf
                    .getf(&format!("_gear/{gi}/_strut_max_wgt_frc"))
                    .zip(acf.getf(&format!("_gear/{gi}/_strut_max_wgt_def")))
                    .map(|(f, d)| f / d.max(0.01));
                acf.setf(&format!("_gear/{gi}/_strut_max_wgt_def"), travel);
                acf.setf(&format!("_gear/{gi}/_strut_max_wgt_frc"), force);
                // Preload on the same straight line through the origin: a
                // strut that carries nothing until it is already a third of
                // the way down cannot hold an aircraft up.
                //
                // For a main leg, that preload also credits back its own
                // height offset from the mains' mean computed in pass 1,
                // above (see gear_preload_def). The nose has no such peers
                // to level against, so it keeps the plain rule.
                let preload_def = if nose_gear { 0.1 * travel } else { gear_preload_def(0.1 * travel, travel, bottom, main_bottom_mean) };
                acf.setf(&format!("_gear/{gi}/_strut_preload_def"), preload_def);
                acf.setf(&format!("_gear/{gi}/_strut_preload_frc"), preload_def * rate);
                // Damping keeps the template's damping *ratio*, which
                // goes as the square root of the spring rate (critical
                // damping is 2*sqrt(k*m)), not as the rate itself. Scaling
                // it linearly softened the A380's nose damper to 29% of the
                // template's when its rate fell to 29%, when it wanted 54%:
                // under-damped by 1.85 times, the nose then overshot far
                // enough while settling to bottom out on a flat runway.
                // The leg's load is the other half of that square root
                // (see gear_damping).
                let load = share * mtow;
                let template_load = template_loads.map_or(load, |(nose, main)| if nose_gear { nose } else { main });
                if let Some(d) = acf.getf(&format!("_gear/{gi}/_damp")).zip(old_rate).map(|(d, o)| gear_damping(d, o, template_load, rate, load)) {
                    acf.setf(&format!("_gear/{gi}/_damp"), d);
                }
            }
            rows.push(format!("{}{:.0}/{:.0}", if nose_gear { "nose " } else { "" }, p[0], p[2]));
        }
        report.push(format!(
            "gear: {} legs from the MSFS contact points, {:.0}% of the weight on the nose leg (x/z ft: {})",
            wheels.len(),
            100.0 * nose_share,
            rows.join(", ")
        ));
        if !fallback_legs.is_empty() {
            report.push(format!(
                "gear: {} leg(s) had no fuselage/wing attach point more than 1 ft above the wheel (x/z ft: {}); using the template's own leg length there instead of the model's geometry, so that leg is no longer the length the airframe would give it",
                fallback_legs.len(),
                fallback_legs.join(", ")
            ));
        }
    }

    // Ground services and the radio altimeter, from the model's own parts.
    // A point the model has no part for is cleared (all zero, as X-Plane
    // leaves an unused one) rather than left where the template had it.
    let mut placed = Vec::new();
    for (key, node) in SERVICE_POINTS {
        let p = part_point(inp.exterior, node);
        for (k, v) in p.unwrap_or([0.0; 3]).iter().enumerate() {
            acf.setf(&format!("{key}/{k}"), *v);
        }
        placed.push(match p {
            Some(p) => format!("{} {:.0}/{:.0}/{:.0}", &key[5..], p[0], p[1], p[2]),
            None => format!("{} cleared (no {node})", &key[5..]),
        });
    }
    report.push(format!("service points (x/y/z ft): {}", placed.join(", ")));

    // Weights, CG and its limits.
    if let Some(m) = num(&fm, wb, "empty_weight") {
        acf.setf("acf/_m_empty", m);
    }
    if let Some(m) = num(&fm, wb, "max_gross_weight") {
        acf.setf("acf/_m_max", m);
    }
    if let Some(cg) = fm.get(wb).and_then(|s| s.get("empty_weight_cg_position")).map(|v| nums(v)).filter(|v| v.len() >= 3) {
        let p = acf_point(cg[0], cg[1], cg[2], datum);
        acf.setf("acf/_cgY", p[1]);
        acf.setf("acf/_cgZ", inp.cg_z.unwrap_or(p[2]));
        // LEMAC and MAC as the aircraft's loadsheet gives them, otherwise as
        // measured from the wing.
        let given = inp.lemac.zip(inp.mac).map(|(l, m)| (-(l + datum[0]), m));
        let measured = wing_info.as_ref().map(|(_, m, l)| (*l, *m));
        if let Some((le, mac)) = given.or(measured) {
            let (f, a) = (num(&fm, wb, "cg_forward_limit").unwrap_or(0.1), num(&fm, wb, "cg_aft_limit").unwrap_or(0.4));
            acf.setf("acf/_cgZ_fwd", le + f * mac);
            acf.setf("acf/_cgZ_aft", le + a * mac);
            report.push(format!(
                "CG: empty at {:.1}% MAC; limits {:.0}-{:.0}% MAC (flight_model.cfg) = z {:.1} to {:.1} ft; MAC {:.1} ft from z {:.1} ({})",
                (p[2] - le) / mac * 100.0,
                f * 100.0,
                a * 100.0,
                le + f * mac,
                le + a * mac,
                mac,
                le,
                if given.is_some() { "loadsheet" } else { "measured from the wing" }
            ));
        }
    }

    // Payload stations, grouped into X-Plane's nine as the systems plugin
    // groups them when it writes their weights.
    let loads: Vec<(usize, &str)> = fm
        .get(wb)
        .map(|s| {
            s.iter()
                .filter_map(|(k, v)| Some((k.strip_prefix("station_load.")?.parse().ok()?, v.as_str())))
                .collect()
        })
        .unwrap_or_default();
    let stations = crate::stations::parse(&loads);
    if !stations.is_empty() {
        let groups = crate::stations::group(&stations);
        let mut rows = Vec::new();
        for i in 0..crate::stations::XPLANE_STATIONS {
            let (name, max, p) = match groups.get(i) {
                Some(g) => {
                    let a = crate::stations::arm(&stations, g);
                    let name = g.iter().map(|&s| stations[s].name.as_str()).collect::<Vec<_>>().join(" + ");
                    (name, g.iter().map(|&s| stations[s].max_lb).sum::<f64>(), acf_point(a[0], a[1], a[2], datum))
                }
                None => (String::new(), 0.0, [0.0; 3]),
            };
            acf.set(&format!("acf/_fixed_name/{i}"), &name);
            acf.setf(&format!("acf/_fixed_max/{i}"), max);
            for (k, v) in p.iter().enumerate() {
                acf.setf(&format!("acf/_fixed_ref/{i},{k}"), *v);
            }
            if !name.is_empty() {
                rows.push(format!("{name} {max:.0} lb at z {:.1}", p[2]));
            }
        }
        report.push(format!("payload: {} stations in {} (flight_model.cfg): {}", stations.len(), groups.len(), rows.join("; ")));
    }

    // Fuel: the real tanks, merged into X-Plane's nine slots if needed.
    if let Some(line) = fuel_tanks(&fm, &mut acf, datum) {
        report.push(line);
    }

    // Speeds.
    let rs = "REFERENCE SPEEDS";
    let speed = |k: &str| num(&fm, rs, k);
    if let Some(v) = speed("full_flaps_stall_speed") {
        acf.setf("acf/_Vso_kts", v);
    }
    if let Some(v) = speed("flaps_up_stall_speed") {
        acf.setf("acf/_Vs_kts", v);
    }
    if let Some(v) = speed("max_flaps_extended") {
        acf.setf("acf/_Vfe1_kts", v);
    }
    if let Some(v) = speed("max_gear_extended") {
        acf.setf("acf/_Vle_kts", v);
    }
    // Vno/Vne/Mmo deliberately do NOT come from [REFERENCE SPEEDS]
    // max_indicated_speed/max_mach: per `Inputs::vmo`/`mmo`'s own doc
    // comment, those are MSFS's overspeed-damage thresholds, not the normal
    // operating limit (barber pole) VMO/MMO represent (FlyByWire's A380X:
    // max_indicated_speed 390/max_mach 0.97 vs the real aircraft's published
    // VMO/MMO 330 kt/0.89 Mach, which is what `inp.vmo`/`inp.mmo` carry).
    if let Some(v) = inp.vmo {
        acf.setf("acf/_Vno_kts", v);
        acf.setf("acf/_Vne_kts", v);
    }
    if let Some(m) = inp.mmo {
        acf.setf("acf/_Mmo", m);
    }
    report.push(format!(
        "speeds: Vso {} Vs {} Vfe {} Vle {} kt (flight_model.cfg), Vmo {} kt Mmo {} (--vmo/--mmo, or FlyByWire's A380X published default if neither was given; not flight_model.cfg's own overspeed-damage max_indicated_speed/max_mach)",
        speed("full_flaps_stall_speed").unwrap_or(0.0),
        speed("flaps_up_stall_speed").unwrap_or(0.0),
        speed("max_flaps_extended").unwrap_or(0.0),
        speed("max_gear_extended").unwrap_or(0.0),
        inp.vmo.map_or("-".into(), |v| v.to_string()),
        inp.mmo.map_or("-".into(), |v| v.to_string())
    ));

    // Flap and slat schedule per detent.
    report.extend(flap_schedule(&fm, &mut acf, inp.flap_degrees.as_deref(), inp.slat_degrees.as_deref()));

    // Aerodynamic tuning: [AERODYNAMICS] flap lift/drag, the clean wing's
    // CLmax angle of attack, [STALL PROTECTION]'s own alpha, and the flap
    // sections' own extend/retract time (XP-003).
    report.extend(aero_tuning(&fm, &mut acf));

    // Control travel.
    for (k, key) in [
        ("elevator_up_limit", "acf/_elev1_up"),
        ("elevator_down_limit", "acf/_elev1_dn"),
        ("aileron_up_limit", "acf/_ailn1_up"),
        ("aileron_down_limit", "acf/_ailn1_dn"),
        ("rudder_limit", "acf/_rudd1_lf"),
        ("rudder_limit", "acf/_rudd1_rt"),
    ] {
        if let Some(v) = num(&fm, geo, k) {
            acf.setf(key, v);
        }
    }
    if let Some(line) = stab_trim(&fm, &mut acf) {
        report.push(line);
    }
    if let Some(line) = rudder_trim_ratio(&fm, &mut acf) {
        report.push(line);
    }

    // Resting height and pitch. X-Plane starts a flight at them, and Plane
    // Maker works them out when it saves, so a built file keeps the
    // template's: the A330's put this aircraft 1.5 m into the ground and its
    // struts threw it up (the crash X-Plane reported). Worked out here from
    // the gear, the struts at three quarters of maximum weight (a twelfth on
    // the nose) and the CG.
    {
        let mtow = acf.getf("acf/_m_max").unwrap_or(0.0);
        let k = |g: usize, s: &str| acf.getf(&format!("_gear/{g}/{s}")).unwrap_or(0.0);
        let all: Vec<usize> = (0..10).filter(|&g| k(g, "_leg_len") > 0.0).collect();
        let noses: Vec<usize> = all.iter().copied().filter(|&g| k(g, "_steerdeg_lospeed") >= 30.0).collect();
        let mains: Vec<usize> = all.iter().copied().filter(|g| !noses.contains(g)).collect();
        if !noses.is_empty() && !mains.is_empty() && mtow > 0.0 {
            let w = 0.75 * mtow;
            let ground_y = |g: usize, load: f64| {
                let (pd, pf, md, mf) = (k(g, "_strut_preload_def"), k(g, "_strut_preload_frc"), k(g, "_strut_max_wgt_def"), k(g, "_strut_max_wgt_frc"));
                let defl = if mf > pf { pd + ((load - pf) / (mf - pf)).clamp(0.0, 1.0) * (md - pd) } else { pd };
                k(g, "_gear_y") - k(g, "_leg_len") - k(g, "_tire_radius") + defl
            };
            let avg = |v: &[usize], f: &dyn Fn(usize) -> f64| v.iter().map(|&g| f(g)).sum::<f64>() / v.len() as f64;
            let (nose_load, main_load) = (w / 12.0 / noses.len() as f64, w * 11.0 / 12.0 / mains.len() as f64);
            let (zn, yn) = (avg(&noses, &|g| k(g, "_gear_z")), avg(&noses, &|g| ground_y(g, nose_load)));
            let (zm, ym) = (avg(&mains, &|g| k(g, "_gear_z")), avg(&mains, &|g| ground_y(g, main_load)));
            let the = -((yn - ym) / (zm - zn)).atan();
            let (cgz, cgy) = (acf.getf("acf/_cgZ").unwrap_or(0.0), acf.getf("acf/_cgY").unwrap_or(0.0));
            let yg = ym + (yn - ym) * (cgz - zm) / (zn - zm);
            let h = (cgy - yg) * the.cos();
            acf.setf("acf/_h_eqlbm", h);
            acf.setf("acf/_the_eqlbm", the.to_degrees());
            report.push(format!(
                "resting on the gear: CG {h:.1} ft above the ground, pitch {:.2} deg (the template's: 13.4 ft, -0.64 deg)",
                the.to_degrees()
            ));
        }
    }

    // Misc objects: the converted OBJs, exterior and cabin.
    let pick = |needle: &str| -> Vec<(String, String)> {
        let n = acf.getf("_obja/count").unwrap_or(0.0) as usize;
        (0..n)
            .find(|i| acf.get(&format!("_obja/{i}/_v10_att_file_stl")).is_some_and(|f| f.to_ascii_lowercase().contains(needle)))
            .map(|i| acf.with_prefix(&format!("_obja/{i}/")))
            .unwrap_or_default()
    };
    let suffix = |k: &str| k.splitn(3, '/').nth(2).unwrap_or("").to_string();
    let strip = |v: Vec<(String, String)>| -> Vec<(String, String)> { v.iter().map(|(k, v)| (suffix(k), v.clone())).collect() };
    let outside = strip(pick("wing"));
    let inside = strip(pick("cockpit/"));
    // The clickable cockpit object, and the lights-only object.
    let clicks = strip(pick("cockpit/a330_cockpit.obj"));
    let lights = strip(pick("lights.obj"));
    // Glass objects, outside and inside ("Glass (Outside)" and "Glass
    // (Inside)" lighting).
    let glass_out = strip(pick("a330_exterior_glass.obj"));
    let glass_in = strip(pick("cockpit/a330_glass_interior.obj"));
    acf.remove_prefix("_obja/");
    for (i, (file, kind)) in inp.objects.iter().enumerate() {
        let src = match kind {
            ObjKind::Exterior => &outside,
            ObjKind::Cabin | ObjKind::PaxCabin => &inside,
            ObjKind::Cockpit if !clicks.is_empty() => &clicks,
            ObjKind::Cockpit => &inside,
            ObjKind::Lights if !lights.is_empty() => &lights,
            ObjKind::Lights => &outside,
            ObjKind::ExteriorGlass if !glass_out.is_empty() => &glass_out,
            ObjKind::ExteriorGlass => &outside,
            ObjKind::CabinGlass if !glass_in.is_empty() => &glass_in,
            ObjKind::CabinGlass => &inside,
            ObjKind::ExteriorBlend => &outside,
            ObjKind::CabinBlend => &inside,
        };
        for (k, v) in src {
            acf.set(&format!("_obja/{i}/{k}"), v);
        }
        // Whether X-Plane treats this object as interior. The field was
        // never set, so each object kept whatever the template's own slot
        // happened to hold -- on the A330 template's unused slots that is
        // uninitialised junk (625279001 on 87 of the A380's 96 cockpit
        // objects). X-Plane uses it to decide which objects an exterior view
        // draws, so with junk in it the whole cockpit, 3.3 million
        // triangles, is drawn from outside the aircraft as well as inside.
        acf.set(&format!("_obja/{i}/_v10_is_internal"), i64::from(is_internal(*kind)));
        if matches!(kind, ObjKind::ExteriorBlend | ObjKind::CabinBlend) {
            let flags = translucent_flags(acf.get(&format!("_obja/{i}/_obj_flags")));
            acf.set(&format!("_obja/{i}/_obj_flags"), flags);
        }
        if matches!(kind, ObjKind::Exterior | ObjKind::ExteriorBlend | ObjKind::ExteriorGlass) {
            if let (Some(eye), Some(&bounds)) = (eye_m, inp.object_bounds.get(file)) {
                if !casts_into_flight_deck(bounds, eye) {
                    let flags = acf.get(&format!("_obja/{i}/_obj_flags")).and_then(|f| f.trim().parse::<i64>().ok()).unwrap_or(0);
                    acf.set(&format!("_obja/{i}/_obj_flags"), flags & !OBJ_FLAG_INTERIOR_SHADOW);
                }
            }
        }
        acf.set(&format!("_obja/{i}/_v10_att_file_stl"), file);
        for k in ["_v10_att_x_acf_prt_ref", "_v10_att_y_acf_prt_ref", "_v10_att_z_acf_prt_ref"] {
            acf.setf(&format!("_obja/{i}/{k}"), 0.0);
        }
        for k in ["_v10_att_body", "_v10_att_wing", "_v10_att_gear"] {
            acf.set(&format!("_obja/{i}/{k}"), -1);
        }
    }
    acf.set("_obja/count", inp.objects.len());
    report.push(format!("objects: {} attached at the reference point", inp.objects.len()));

    Ok((acf.to_text(), report))
}

#[cfg(test)]
mod tests {
    use super::*;

    const T: &str = "I\n1200 Version\nACF\n\nPROPERTIES_BEGIN\nP acf/_m_empty 100.0\nP _wing/0/_Croot 1.0\nP _wing/0/_Ctip 2.0\nP _obja/0/_v10_att_file_stl a.obj\nP _obja/count 1\nPROPERTIES_END\nPANEL_2D_BEGIN\nbut_DC_fdir_mode x\nPANEL_2D_END\nPANEL_3D_BEGIN\nGROUP ECAMS\nEND_GROUP\nPANEL_3D_END\n";

    #[test]
    fn only_objects_around_the_flight_deck_cast_shadows_into_it() {
        // The A380's own numbers: the pilot's eye, and three of its
        // exterior objects' extents (x right, y up, z aft, metres).
        let eye = [-0.52, 3.16, -32.07];
        let nose_fuselage = ([-3.6, -1.5, -35.2], [3.6, 7.0, -14.8]);
        let tyres = ([-6.0, -2.22, -33.9], [6.0, 0.96, 5.9]);
        let wing = ([-40.0, -2.0, -17.1], [-3.0, 3.0, 16.6]);
        assert!(casts_into_flight_deck(nose_fuselage, eye), "the skin around the windows shades the flight deck");
        assert!(!casts_into_flight_deck(tyres, eye), "the tyres are all under the flight-deck floor");
        assert!(!casts_into_flight_deck(wing, eye), "the wing is fifteen metres aft");
    }

    #[test]
    fn blended_decal_objects_are_marked_translucent() {
        // Laminar's cockpit objects carry 13; their transparent ones 8195/8219.
        assert_eq!(translucent_flags(Some("13")), 8205);
        assert_eq!(translucent_flags(Some("8195")), 8195);
        assert_eq!(translucent_flags(None), 8192);
    }

    /// `PaxCabin` is the one interior kind that must draw from outside the
    /// aircraft too (an open passenger door should not show a hollow
    /// fuselage): every other cockpit/cabin kind stays internal-only, as
    /// before this kind existed.
    #[test]
    fn only_pax_cabin_is_not_marked_internal() {
        assert!(!is_internal(ObjKind::PaxCabin), "the one kind X-Plane must draw from outside too");
        assert!(is_internal(ObjKind::Cockpit));
        assert!(is_internal(ObjKind::Cabin), "plain non-clicked interior objects (cockpit decor, demoted duplicates) stay internal");
        assert!(is_internal(ObjKind::CabinGlass));
        assert!(is_internal(ObjKind::CabinBlend));
        assert!(!is_internal(ObjKind::Exterior));
        assert!(!is_internal(ObjKind::ExteriorGlass));
        assert!(!is_internal(ObjKind::ExteriorBlend));
        assert!(!is_internal(ObjKind::Lights));
    }

    #[test]
    fn properties_are_set_added_and_removed() {
        let mut a = Acf::parse(T).unwrap();
        a.setf("acf/_m_empty", 5.5);
        a.set("acf/_new", "x");
        a.set("_wing/3/_Ctip", 2.0);
        a.remove_prefix("_obja/");
        let t = a.to_text();
        assert!(t.contains("P acf/_m_empty 5.500000000\n"), "set in place");
        let (new, end) = (t.find("P acf/_new x\n").unwrap(), t.find("PROPERTIES_END").unwrap());
        assert!(new < end && t.contains("P _wing/3/_Ctip 2\n"), "added keys go before the end marker");
        assert!(!t.contains("_obja"));
        assert!(t.ends_with("PROPERTIES_END\nPANEL_2D_BEGIN\nPANEL_2D_END\nPANEL_3D_BEGIN\nPANEL_3D_END\n"), "{t}");
        assert_eq!(a.getf("_wing/3/_Ctip"), Some(2.0));
        assert_eq!(a.getf("_wing/0/_Croot"), Some(1.0));
    }

    #[test]
    fn stabiliser_trim_travel_comes_from_flight_model_cfg() {
        // FlyByWire's A380X flight_model.cfg:619-620, on the A330 template's
        // 8 deg each way.
        let fm = parse_cfg(
            "[AIRPLANE_GEOMETRY]
             elevator_trim_up_limit = 10 ; Elevator trim max angle (absolute value) (DEGREES)
             elevator_trim_down_limit = 2 ; Elevator trim max angle nose down direction (absolute value) (DEGREES)
",
        );
        let mut a = Acf::parse(&T.replace("PROPERTIES_END", "P acf/_stab_trim_dn 8.000000000
P acf/_stab_trim_up 8.000000000
PROPERTIES_END")).unwrap();
        let line = stab_trim(&fm, &mut a).unwrap();
        assert_eq!((a.getf("acf/_stab_trim_up"), a.getf("acf/_stab_trim_dn")), (Some(10.0), Some(2.0)), "{line}");
        let t = a.to_text();
        assert!(t.contains("P acf/_stab_trim_up 10.000000000
") && t.contains("P acf/_stab_trim_dn 2.000000000
"), "{t}");
        assert!(stab_trim(&parse_cfg("[AIRPLANE_GEOMETRY]
elevator_up_limit = 30
"), &mut a).is_none());
    }

    #[test]
    fn rudder_trim_ratio_comes_from_flight_model_cfg() {
        // FlyByWire's A380X flight_model.cfg:617-618: rudder_limit 30 deg,
        // rudder_trim_limit 25.5 deg, on the A330 template's ratio 0.83.
        let fm = parse_cfg(
            "[AIRPLANE_GEOMETRY]
             rudder_limit = 30 ; Rudder max deflection angle (absolute value) (DEGREES)
             rudder_trim_limit = 25.5 ; Rudder trim max deflection angle (absolute value) (DEGREES)
",
        );
        let mut a = Acf::parse(&T.replace(
            "PROPERTIES_END",
            "P acf/_hdng_acft_lf_trim_rat 0.829999983\nP acf/_hdng_acft_rt_trim_rat 0.829999983\nPROPERTIES_END",
        ))
        .unwrap();
        let line = rudder_trim_ratio(&fm, &mut a).unwrap();
        assert!((a.getf("acf/_hdng_acft_lf_trim_rat").unwrap() - 0.85).abs() < 1e-9, "{line}");
        assert!((a.getf("acf/_hdng_acft_rt_trim_rat").unwrap() - 0.85).abs() < 1e-9, "{line}");
        // No rudder_limit at all: nothing to divide by, nothing reported.
        assert!(rudder_trim_ratio(&parse_cfg("[AIRPLANE_GEOMETRY]\nrudder_trim_limit = 25.5\n"), &mut a).is_none());
    }

    #[test]
    fn steering_limits_come_from_the_systems_when_given() {
        // flight_model.cfg point.0 (nose, 70), point.1 (body, 8), point.3
        // (wing, 0); a380_systems hydraulic/mod.rs:1792-1814: 75 and 15.
        assert_eq!(steering_limit(true, 70.0, Some(75.0), Some(15.0)), 75.0);
        assert_eq!(steering_limit(false, 8.0, Some(75.0), Some(15.0)), 15.0);
        assert_eq!(steering_limit(false, 0.0, Some(75.0), Some(15.0)), 0.0);
        assert_eq!(steering_limit(false, 8.0, None, None), 8.0);
    }

    #[test]
    fn gear_leg_length_uses_the_attach_point_when_it_gives_a_real_leg() {
        // A380 wing main: bottom -15.63, radius 2.356, a wing underside
        // (attach) at -1.80 gives an 11.47 ft leg -- comfortably over the 1
        // ft floor, so the model's own geometry is used, not the template.
        let (leg, fell_back) = gear_leg_length(Some(-1.80), -15.63, 2.356, 5.0);
        assert!((leg - 11.474).abs() < 1e-3);
        assert!(!fell_back);
    }

    #[test]
    fn gear_leg_length_falls_back_when_there_is_no_attach_point() {
        let (leg, fell_back) = gear_leg_length(None, -15.75, 2.356, 5.0);
        assert_eq!(leg, 5.0);
        assert!(fell_back);
    }

    #[test]
    fn gear_leg_length_falls_back_at_and_below_the_one_foot_floor() {
        // attach - bottom - radius == 1.0 exactly: the filter is `> 1.0`,
        // so this still falls back, not a boundary that silently accepts it.
        let (leg, fell_back) = gear_leg_length(Some(-11.75), -15.75, 3.0, 5.0);
        assert_eq!(leg, 5.0);
        assert!(fell_back);
        // A hair over 1.0 does use the computed leg.
        let (leg2, fell_back2) = gear_leg_length(Some(-11.749), -15.75, 3.0, 5.0);
        assert!((leg2 - 1.001).abs() < 1e-6);
        assert!(!fell_back2);
    }

    #[test]
    fn main_gear_preload_credits_a_leg_for_sitting_below_the_mains_mean() {
        // Two synthetic main legs on the A380's own numbers: 2.40 ft travel,
        // contact-point verticals -15.75 (body) and -15.63 (wing) --
        // flight_model.cfg point.1-4, 0.12 ft apart, mean -15.69.
        let travel = 2.40;
        let base = 0.1 * travel; // the un-credited preload the existing rule gives every leg
        let mean = -15.69;
        let low_leg = gear_preload_def(base, travel, -15.75, mean); // body: below the mean, touches down first
        let high_leg = gear_preload_def(base, travel, -15.63, mean); // wing: above the mean
        assert!(low_leg > base, "a leg below the mains' mean is credited extra free travel: {low_leg} vs base {base}");
        assert!(high_leg < base, "a leg above the mean gives some back: {high_leg} vs base {base}");
        assert!(((low_leg - base) - (base - high_leg)).abs() < 1e-9, "the credit and the debit are symmetric about the mean");
        // Clamped: an offset bigger than the base preload doesn't go
        // negative, and one bigger than the remaining travel doesn't exceed it.
        assert_eq!(gear_preload_def(base, travel, mean + 10.0, mean), 0.0);
        assert_eq!(gear_preload_def(base, travel, mean - 10.0, mean), travel);
        // No offset from the mean (every main at the same height) reproduces
        // today's uncorrected rule exactly.
        assert_eq!(gear_preload_def(base, travel, mean, mean), base);
    }

    #[test]
    fn a_body_radius_is_its_cross_section_not_its_length() {
        // A 26 ft nacelle of 7.7 ft rings, its origin at the aft end the way the
        // converter places them, and its axis 2 ft off the part's own origin.
        let ring = |z: f64, r: f64| -> [[f64; 3]; 18] {
            std::array::from_fn(|j| {
                let t = (j as f64 * 20.0).to_radians();
                [r * t.sin(), 2.0 + r * t.cos(), z]
            })
        };
        let rings = [ring(-26.0, 6.0), ring(-13.0, 7.7), ring(0.0, 5.0)];
        assert!((body_radius(&rings) - 7.7).abs() < 1e-9, "{}", body_radius(&rings));
        assert_eq!(body_radius(&[]), 0.0);
    }

    #[test]
    fn a_service_point_is_where_its_part_meets_the_outside() {
        use crate::model::glb::{Mesh, Node, Vertex};
        let node = |name: &str, parent: Option<usize>| Node {
            name: name.into(),
            parent,
            translation: [0.0; 3],
            rotation: [0.0, 0.0, 0.0, 1.0],
            scale: [1.0; 3],
            world: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        };
        let mesh = |on: usize, pts: &[[f32; 3]]| Mesh {
            node: Some(on),
            vertices: pts.iter().map(|&pos| Vertex { pos, ..Default::default() }).collect(),
            ..Default::default()
        };
        // glTF metres: +x is the aircraft's left, +z forward (acf x = -x, z = -z).
        let model = Model {
            nodes: vec![node("ROOT", None), node("PAX_DOOR_M1L", Some(0)), node("PAX_DOOR_M1L_HANDLE", Some(1)), node("EXT_PWR_PANELS", Some(0))],
            meshes: vec![
                mesh(1, &[[2.5, 1.0, 28.2], [3.05, 3.14, 29.5]]),
                mesh(2, &[[3.2, 2.0, 28.8]]),
                mesh(3, &[[-0.6, -1.45, 29.0], [0.6, -1.4, 29.4]]),
                mesh(0, &[[-50.0, -9.0, 0.0]]),
            ],
            ..Default::default()
        };
        // The left door: its outer skin (the handle, a child, stands proud of
        // it), its sill, its middle fore and aft.
        let d = part_point(&model, "PAX_DOOR_M1L").unwrap();
        assert!((d[0] + 3.2 * FT).abs() < 1e-6 && (d[1] - 1.0 * FT).abs() < 1e-6, "{d:?}");
        assert!((d[2] + (28.2 + 29.5) / 2.0 * FT).abs() < 1e-5, "{d:?}");
        // A part across the centreline stays on it; its lowest point.
        let p = part_point(&model, "EXT_PWR_PANELS").unwrap();
        assert!(p[0].abs() < 1e-6 && (p[1] + 1.45 * FT).abs() < 1e-5, "{p:?}");
        assert_eq!(part_point(&model, "REFUEL_COUPLING"), None);
    }

    #[test]
    fn an_engine_and_its_fan_disc_move_together() {
        // The A330 template's left engine and fan disc, where the A330 has them.
        let template = T.replace(
            "PROPERTIES_END",
            "P _engn/0/_part_x -30.77\nP _engn/0/_part_y -9.27\nP _engn/0/_part_z 92\n\
             P _blad/0/_part_x -30.77\nP _blad/0/_part_y -9.27\nP _blad/0/_part_z 92\nPROPERTIES_END",
        );
        let mut a = Acf::parse(&template).unwrap();
        // The A380's left inner engine; and one the template never had.
        place_engine(&mut a, 0, &[-47.5, -4.0, -15.0]);
        place_engine(&mut a, 3, &[84.0, -1.5, 10.0]);
        for (ei, e) in [(0, [-47.5, -4.0, -15.0]), (3, [84.0, -1.5, 10.0])] {
            for fam in ["_engn", "_blad"] {
                let at: Vec<f64> = ["x", "y", "z"].iter().map(|k| a.getf(&format!("{fam}/{ei}/_part_{k}")).unwrap()).collect();
                assert_eq!(at, e, "{fam}/{ei}");
            }
        }
    }

    #[test]
    fn gear_damping_keeps_the_template_legs_damping_ratio() {
        // The A330 template's own gear, CG and maximum weight.
        let template = T.replace(
            "PROPERTIES_END",
            "P acf/_m_max 533519\nP acf/_cgZ 97\n\
             P _gear/0/_gear_type 3\nP _gear/0/_gear_z 21.86\nP _gear/0/_leg_len 7\nP _gear/0/_lonE 0\n\
             P _gear/1/_gear_type 5\nP _gear/1/_gear_z 103.81\nP _gear/1/_leg_len 13.1\nP _gear/1/_lonE -10\n\
             P _gear/2/_gear_type 5\nP _gear/2/_gear_z 103.81\nP _gear/2/_leg_len 13.1\nP _gear/2/_lonE -10\n\
             P _gear/3/_gear_type 0\nPROPERTIES_END",
        );
        let a = Acf::parse(&template).unwrap();
        let (nose, main) = template_leg_loads(&a).unwrap();
        // The mains' feet 13.1 sin 10 = 2.27 ft aft of 103.81: 10.8% on the nose.
        assert!((nose / 533_519. - 0.1079).abs() < 0.0005, "{nose}");
        assert!((nose + 2. * main - 533_519.).abs() < 1e-6);
        // The installed A380 nose: 362 878 lb at 2.5895 ft, carrying
        // 181 440 lb, from the template's 11 489 at 80 424 lb / 0.70 ft.
        // Rate alone gave 12 689 (damping ratio 0.23); with the load, 0.40.
        let d = gear_damping(11_489.092, 80_423.64 / 0.7, nose, 362_878.06 / 2.5895, 181_440.);
        assert!((d - 22_530.).abs() < 150., "{d}");
        // At the template's own rate and load: the template's own damping.
        assert!((gear_damping(10_686., 106_862., main, 106_862., main) - 10_686.).abs() < 1e-6);
        // Four times the load at the same rate: twice the damping.
        assert!((gear_damping(10_000., 1e5, 1e5, 1e5, 4e5) - 20_000.).abs() < 1e-6);
        // No load known: the rate alone, as before.
        assert!((gear_damping(10_000., 1e5, 0., 4e5, 0.) - 20_000.).abs() < 1e-6);
        // No nose leg in the template: nothing to scale by.
        assert_eq!(template_leg_loads(&Acf::parse(T).unwrap()), None);
    }

    #[test]
    fn flap_and_slat_schedules_follow_the_detents() {
        // FlyByWire's A380X flight_model.cfg:861-919 (trailing edge FLAPS.0,
        // leading edge FLAPS.2), on the A330 template's tables.
        let fm = parse_cfg(
            "[FLAPS.0]\ntype = 1\nflaps-position.0 = 0.00, -1\nflaps-position.1 = 0.01, -1\nflaps-position.2 = 8.00, 222\n\
             flaps-position.3 = 17.00, 220\nflaps-position.4 = 26.00, 196\nflaps-position.5 = 32.00, 182\n\
             [FLAPS.1]\ntype = 1\nflaps-position.0 = 0.00, -1\nflaps-position.1 = 5.00, -1\n\
             [FLAPS.2]\ntype = 2\nflaps-position.0 = 0.00, -1\nflaps-position.1 = 20.00, 263\nflaps-position.2 = 20.01, 222\n\
             flaps-position.3 = 20.02, 220\nflaps-position.4 = 23.00, 196\nflaps-position.5 = 23.01, 182\n",
        );
        let mut tables = String::new();
        for set in ["_flap1_dn", "_slat1_dn"] {
            for (i, v) in [0.0, 8.0, 14.0, 22.0, 32.0].iter().enumerate() {
                tables.push_str(&format!("P acf/{set}/{i} {v}\n"));
            }
            tables.push_str(&format!("P acf/{set}/5 0.434782594\nP acf/{set}/count 6\n"));
        }
        let template = T.replace("PROPERTIES_END", &format!("{tables}PROPERTIES_END"));
        // From the cfg alone: slats per detent from the leading-edge section.
        let mut a = Acf::parse(&template).unwrap();
        flap_schedule(&fm, &mut a, None, None);
        assert_eq!(a.getf("acf/_flap1_dn/5"), Some(32.0));
        assert_eq!(a.getf("acf/_slat1_dn_max_deg"), Some(23.01));
        assert!((a.getf("acf/_slat1_dn/5").unwrap() - 1.0).abs() < 1e-9 && (a.getf("acf/_slat1_dn/1").unwrap() - 20.0 / 23.01).abs() < 1e-6);
        // With the systems' angles (a380_systems FPPU tables).
        let mut a = Acf::parse(&template).unwrap();
        let lines = flap_schedule(&fm, &mut a, Some(&[0.0, 0.0, 8.0, 17.0, 26.0, 33.0]), Some(&[0.0, 20.0, 20.0, 20.0, 23.0, 23.0]));
        let flaps: Vec<f64> = (0..6).map(|i| a.getf(&format!("acf/_flap1_dn/{i}")).unwrap()).collect();
        assert_eq!(flaps, vec![0.0, 0.01, 8.0, 17.0, 26.0, 33.0], "{lines:?}");
        let slats: Vec<f64> = (0..6).map(|i| a.getf(&format!("acf/_slat1_dn/{i}")).unwrap()).collect();
        assert!(slats.iter().zip([0.0, 20.0 / 23.0, 20.0 / 23.0, 20.0 / 23.0, 1.0, 1.0]).all(|(a, b)| (a - b).abs() < 1e-9), "{slats:?}");
        assert_eq!((a.getf("acf/_slat1_dn_max_deg"), a.getf("acf/_slat2_dn_max_deg")), (Some(23.0), Some(23.0)));
        assert_eq!(a.getf("acf/_flap_detents"), Some(5.0));
    }

    #[test]
    fn fuel_tanks_merge_into_nine_slots_and_clear_the_templates_stale_default_fill() {
        // A cut-down version of FlyByWire's A380X FUEL_SYSTEM (flight_model.cfg
        // TANK.1-11, plus its sub-gallon EXTRA plumbing tanks that must not
        // count as real tanks): eleven tanks under a wing tip each, which must
        // merge down to nine.
        let fm = parse_cfg(
            "[FUEL_SYSTEM]
Tank.1 = Name:LeftOuter#Title:LEFT OUTER#Capacity:2731.5#Position:-25.0,-100.0,8.5
Tank.2 = Name:Feed1#Title:FEED ONE#Capacity:7299.6#Position:-7.45,-71.0,7.3
Tank.3 = Name:LeftMid#Title:LEFT MID#Capacity:9632#Position:7.1,-45.9,5.9
Tank.4 = Name:LeftInner#Title:LEFT INNER#Capacity:12189.4#Position:16.5,-24.7,3.2
Tank.5 = Name:Feed2#Title:FEED TWO#Capacity:7753.2#Position:27.3,-18.4,1.0
Tank.6 = Name:Feed3#Title:FEED THREE#Capacity:7753.2#Position:27.3,18.4,1.0
Tank.7 = Name:RightInner#Title:RIGHT INNER#Capacity:12189.4#Position:16.5,24.7,3.2
Tank.8 = Name:RightMid#Title:RIGHT MID#Capacity:9632#Position:7.1,45.9,5.9
Tank.9 = Name:Feed4#Title:FEED FOUR#Capacity:7299.6#Position:-7.45,71,7.3
Tank.10 = Name:RightOuter#Title:RIGHT OUTER#Capacity:2731.5#Position:-25.0,100,8.5
Tank.11 = Name:Trim#Title:TRIM#Capacity:6260.3#Position:-87.14,0,12.1
Tank.12 = Name:Extra1#Title:EXTRA ONE#Capacity:1#Position:-7.45,-71.0,7.3
",
        );
        // The A330 template's own nine tanks: the centre tank starts empty,
        // the mains 80% (its own real default ratios), unrelated to which of
        // the new aircraft's tanks end up in each slot.
        let mut tables = String::new();
        for i in 0..9 {
            let def = if i % 3 == 1 { 0.0 } else { 0.8 };
            tables.push_str(&format!("P acf/_tank_rat_def/{i} {def}\n"));
        }
        let template = T.replace("PROPERTIES_END", &format!("{tables}PROPERTIES_END"));
        let mut a = Acf::parse(&template).unwrap();
        let line = fuel_tanks(&fm, &mut a, [0.0; 3]).unwrap();
        // Eleven tanks merged into nine (the two smallest, the outers, each
        // merge into their nearest same-side neighbour: FEED ONE/FOUR).
        let names: Vec<String> = (0..9).map(|i| a.get(&format!("acf/_tank_name/{i}")).unwrap().to_string()).collect();
        assert!(names.iter().any(|n| n.contains("FEED ONE") && n.contains("LEFT OUTER")), "{names:?}");
        assert!(names.iter().any(|n| n.contains("FEED FOUR") && n.contains("RIGHT OUTER")), "{names:?}");
        assert!(names.iter().all(|n| !n.is_empty()), "all nine slots hold a real tank: {names:?}");
        // Every real tank's default fill replaces the template's stale,
        // now-meaningless per-slot value (mixed 0.8/0.0) with a single
        // consistent default; no slot is left starved at the template's 0%.
        for i in 0..9 {
            assert_eq!(a.getf(&format!("acf/_tank_rat_def/{i}")), Some(1.0), "slot {i}: {line}");
        }
        let ratios: f64 = (0..9).map(|i| a.getf(&format!("acf/_tank_rat/{i}")).unwrap()).sum();
        assert!((ratios - 1.0).abs() < 1e-6, "tank ratios sum to the whole: {ratios}");
    }

    #[test]
    fn aero_tuning_reads_aerodynamics_and_stall_protection() {
        // FlyByWire's A380X flight_model.cfg:643-664 (AERODYNAMICS),
        // 848-855 (STALL PROTECTION), 858-875 (FLAPS.0's extending-time).
        let fm = parse_cfg(
            "[AERODYNAMICS]
lift_coef_flaps = 1.2694
drag_coef_flaps = 0.270
lift_coef_aoa_table = -3.15:0, 0:0.095, 0.139:0.95, 0.2:1.24, 0.314:1.60, 0.36:1.70, 0.5:1.58, 3.15:0
[STALL PROTECTION]
stall_protection = 0
on_limit = 20
[FLAPS.0]
type = 1
extending-time = 25
[FLAPS.1]
type = 1
extending-time = 20
",
        );
        let mut a = Acf::parse(T).unwrap();
        let lines = aero_tuning(&fm, &mut a);
        assert_eq!(a.getf("acf/_flap1_cl"), Some(1.2694));
        assert_eq!(a.getf("acf/_flap1_cd"), Some(0.270));
        // on_limit (already degrees) wins over the lift table's own CLmax
        // breakpoint, which is only reported as a cross-check.
        assert_eq!(a.getf("acf/_stall_warn_aoa"), Some(20.0));
        assert_eq!(a.getf("acf/_flap_ext_time"), Some(25.0));
        assert_eq!(a.getf("acf/_flap_ret_time"), Some(25.0));
        assert!(lines.iter().any(|l| l.contains("20.6")), "{lines:?}");
    }

    #[test]
    fn lift_curve_peak_finds_clmax_in_degrees() {
        let (aoa, cl) = lift_curve_peak("-3.15:0, 0:0.095, 0.139:0.95, 0.2:1.24, 0.314:1.60, 0.36:1.70, 0.5:1.58, 3.15:0").unwrap();
        assert_eq!(cl, 1.70);
        assert!((aoa - 20.63).abs() < 0.01, "{aoa}");
    }

    #[test]
    fn stall_speed_calibration_matches_flybywires_reference_speeds() {
        // flight_model.cfg: max_gross_weight 1,124,355 lb (WEIGHT_AND_BALANCE
        // :16), wing_area 9096 sq ft (GEOMETRY:578), clean CLmax 1.70
        // (AERODYNAMICS:651 lift_coef_aoa_table), lift_coef_flaps 1.2694
        // (AERODYNAMICS:643), full_flaps_stall_speed 115 / flaps_up_stall_speed
        // 171 kt (REFERENCE SPEEDS:782-783). X-Plane derives stall speed from
        // the converted wing itself; this only checks the geometry and CLmax
        // that conversion used land in the right place.
        let (weight, area, clean_clmax) = (1_124_355.0, 9096.0, 1.70);
        let landing = stall_speed_kt(weight, area, clean_clmax + 1.2694);
        let clean = stall_speed_kt(weight, area, clean_clmax);
        // Landing config lines up closely; clean is a cruder approximation
        // (flight_model.cfg's flaps-up reference speed carries more margin
        // than the pure 1g stall the textbook formula gives) but still the
        // right order of magnitude, both documented in docs/flight-model.md.
        assert!((landing - 115.0).abs() < 115.0 * 0.10, "{landing}");
        assert!((clean - 171.0).abs() < 171.0 * 0.20, "{clean}");
    }

    #[test]
    fn the_wing_flexes_less_than_the_a330_template() {
        // The A330 template's flex: 1.5 deg of mid-span dihedral per g.
        let mut a = Acf::parse(&T.replace("PROPERTIES_END", "P acf/_wing_mid_dihed_per_g 1.500000000\nPROPERTIES_END")).unwrap();
        wing_flex(&mut a);
        let flex = a.getf("acf/_wing_mid_dihed_per_g").unwrap();
        assert!(flex > 0.0 && flex < 1.5, "{flex}");
    }

    #[test]
    fn a_swept_dihedral_segment_matches_x_plane_geometry() {
        // The A330 template's first wing segment: 31.46 ft along a 27.4 deg
        // quarter-chord line with 8 deg dihedral ends 27.66 ft out, 3.89 ft
        // up and 14.48 ft aft, where its next segment starts.
        let (sw, di, len) = (27.4f64.to_radians(), 8f64.to_radians(), 31.46);
        let end = [len * sw.cos() * di.cos(), len * sw.cos() * di.sin(), len * sw.sin()];
        let a = Station { s: 0.0, le: -10.0, te: 30.0, h: 0.0, low: 0.0 };
        let b = Station { s: end[0], le: end[2] - 5.0, te: end[2] + 15.0, h: end[1], low: 0.0 };
        let s = horizontal(&a, 0.0, &b);
        assert!((s.semilen - len).abs() < 1e-6 && (s.sweep - 27.4).abs() < 1e-6 && (s.dihed - 8.0).abs() < 1e-6, "{} {} {}", s.semilen, s.sweep, s.dihed);
        assert!((end[0] - 27.66).abs() < 0.01 && (end[1] - 3.89).abs() < 0.01 && (end[2] - 14.48).abs() < 0.01);
    }

    #[test]
    fn triangles_section_exactly_between_sparse_vertices() {
        // A flat panel 20 ft along x, leading edge at z = 0, trailing edge
        // at z = 10, made of just two triangles: no vertex lies between
        // x = 0 and x = 20, yet every station gets both edges.
        let t: Vec<Tri> = vec![
            [[0.0, 0.0, 0.0], [20.0, 0.0, 0.0], [20.0, 0.0, 10.0]],
            [[0.0, 0.0, 0.0], [20.0, 0.0, 10.0], [0.0, 0.0, 10.0]],
        ];
        let st = slice(&t, |p| p[0], |p| p[1], 1.0);
        assert!(st.len() >= 18, "{}", st.len());
        for s in &st {
            assert!((s.le - 0.0).abs() < 1e-9 && (s.te - 10.0).abs() < 1e-9, "{s:?}");
        }
    }

    #[test]
    fn rings_follow_plane_maker_order() {
        // A circle of radius 5 around (0, 2) at z = 10.
        let pts: Vec<[f64; 3]> = (0..360).map(|a| {
            let t = (a as f64).to_radians();
            [5.0 * t.sin(), 2.0 + 5.0 * t.cos(), 10.0]
        }).collect();
        let r = body_rings(&pts, &[10.0], [0.0; 3]);
        let ring = r[0];
        assert!((ring[0][1] - 7.0).abs() < 0.1 && ring[0][0].abs() < 0.1, "top first: {:?}", ring[0]);
        assert!((ring[4][0] - 5.0).abs() < 0.1, "then the right side: {:?}", ring[4]);
        assert!((ring[8][1] + 3.0).abs() < 0.1 && (ring[9][1] + 3.0).abs() < 0.1, "bottom twice");
        assert!((ring[13][0] + 5.0).abs() < 0.1, "then the left: {:?}", ring[13]);
    }
}
