//! Golden streams drawn with the software renderer (the same tessellation
//! and plan the GL renderer carries out), written as PNG snapshots to
//! `SNAPSHOTS`, checked against tiny-skia drawing the same commands on its
//! own, and timed.

use std::path::{Path, PathBuf};
use std::time::Instant;

use tiny_skia as sk;

use super::path::arc_sweep;
use super::plan::{self, Step};
use super::screens::{self, SCREENS};
use super::stream::{self, opcode::*, Cap, FillRule, Join, Op};
use super::tessellate::{add_dimming, Images, Mesh, Resources, Tessellator};
use super::text::{Atlas, Fonts};
use super::{soft, Displays, Tess};

const SNAPSHOTS: &str = r"D:\fbw-build\display-snapshots";
const PACKAGE_HTML_UI: &str =
    r"D:\Microsoft Flight Simulator 2020\Microsoft Flight Simulator 2020 Packages\Community\flybywire-aircraft-a380-842\html_ui";

fn resources() -> Resources {
    let root = PathBuf::from(PACKAGE_HTML_UI);
    let (fonts, _) = Fonts::load(&root.join(super::FONTS_DIR));
    Resources { fonts, atlas: Atlas::default(), images: Images::new(root) }
}

fn have_package() -> bool {
    Path::new(PACKAGE_HTML_UI).join(super::FONTS_DIR).join("FBW-Display-EIS-A380.ttf").is_file()
}

/// A stream as the DOM side would build it.
#[derive(Default)]
struct S {
    ops: Vec<f64>,
    strings: Vec<String>,
}

impl S {
    fn op(&mut self, code: u32, args: &[f64]) -> &mut Self {
        self.ops.push(code as f64);
        self.ops.extend_from_slice(args);
        self
    }

    fn string(&mut self, s: &str) -> f64 {
        self.strings.push(s.to_string());
        (self.strings.len() - 1) as f64
    }

    fn fill(&mut self, c: [f64; 4], evenodd: bool) -> &mut Self {
        self.op(FILL, &[c[0], c[1], c[2], c[3], evenodd as u8 as f64])
    }

    #[allow(clippy::too_many_arguments)]
    fn stroke(&mut self, c: [f64; 4], width: f64, cap: u8, join: u8, miter: f64, dashes: &[f64], offset: f64) -> &mut Self {
        let mut args = vec![c[0], c[1], c[2], c[3], width, cap as f64, join as f64, miter, dashes.len() as f64];
        args.extend_from_slice(dashes);
        args.push(offset);
        self.op(STROKE, &args)
    }

    #[allow(clippy::too_many_arguments)]
    fn text(&mut self, text: &str, font: &str, size: f64, x: f64, y: f64, align: u8, baseline: u8, c: [f64; 4], stroke: f64) -> &mut Self {
        let (t, f) = (self.string(text), self.string(font));
        let args = [t, f, size, 400., 0., x, y, align as f64, baseline as f64, c[0], c[1], c[2], c[3], stroke, 1., 1., 1., 1.];
        self.op(TEXT, &args)
    }

    fn parsed(&self) -> Vec<Op> {
        stream::parse(&self.ops, self.strings.len()).expect("a valid stream")
    }
}

const WHITE: [f64; 4] = [1., 1., 1., 1.];
const GREEN: [f64; 4] = [0., 1., 0., 1.];
const CYAN: [f64; 4] = [0., 1., 1., 1.];
const AMBER: [f64; 4] = [1., 0.6, 0., 1.];
const MAGENTA: [f64; 4] = [1., 0., 1., 1.];

fn tessellate(s: &S, screen: &str, size: (u32, u32), res: &mut Resources) -> Mesh {
    let ops = s.parsed();
    let mut t = Tessellator::default();
    for _ in 0..4 {
        if let Ok(mesh) = t.run(&ops, &s.strings, screen, size, (1., 1.), res) {
            assert!(mesh.problems.is_empty(), "{:?}", mesh.problems);
            return mesh;
        }
    }
    panic!("the atlas never settled");
}

fn rgba_of(target: &soft::Target) -> Vec<u8> {
    target.resolve()
}

fn save(name: &str, width: u32, height: u32, rgba: Vec<u8>) {
    let _ = std::fs::create_dir_all(SNAPSHOTS);
    let pixmap = sk::Pixmap::from_vec(rgba, sk::IntSize::from_wh(width, height).unwrap()).unwrap();
    pixmap.save_png(Path::new(SNAPSHOTS).join(format!("{name}.png"))).unwrap();
}

fn pixel(rgba: &[u8], width: u32, x: u32, y: u32) -> [u8; 4] {
    let i = ((y * width + x) * 4) as usize;
    [rgba[i], rgba[i + 1], rgba[i + 2], rgba[i + 3]]
}

/// Geometry every canvas feature the contract has, except text and images.
fn shapes() -> S {
    let mut s = S::default();
    // Fill rules: a five-pointed star, nonzero (solid) and evenodd (hole).
    for (i, evenodd) in [(0, false), (1, true)] {
        let (cx, cy) = (60. + 120. * i as f64, 60.);
        s.op(BEGIN_PATH, &[]);
        for k in 0..5 {
            let a = -std::f64::consts::FRAC_PI_2 + k as f64 * 4. * std::f64::consts::PI / 5.;
            s.op(if k == 0 { MOVE_TO } else { LINE_TO }, &[cx + 50. * a.cos(), cy + 50. * a.sin()]);
        }
        s.op(CLOSE_PATH, &[]).fill(GREEN, evenodd);
    }
    // Caps and joins on a zigzag, butt/miter, round/round, square/bevel.
    for (i, (cap, join)) in [(0u8, 0u8), (1, 1), (2, 2)].into_iter().enumerate() {
        let y = 150. + i as f64 * 45.;
        s.op(BEGIN_PATH, &[]).op(MOVE_TO, &[20., y + 20.]).op(LINE_TO, &[50., y]).op(LINE_TO, &[80., y + 20.]).op(LINE_TO, &[110., y]);
        s.stroke(WHITE, 9., cap, join, 10., &[], 0.);
    }
    // A dashed arc and a dashed bézier.
    s.op(BEGIN_PATH, &[]).op(ARC, &[330., 70., 50., 0.3, 5.5, 0.]).stroke(CYAN, 3., 0, 0, 10., &[10., 5.], 2.);
    s.op(BEGIN_PATH, &[]).op(MOVE_TO, &[150., 180.]).op(CUBIC_TO, &[190., 120., 250., 260., 300., 170.]);
    s.op(QUAD_TO, &[340., 120., 380., 190.]).stroke(AMBER, 4., 1, 1, 10., &[18., 6., 3., 6.], 0.);
    // A rotated, scaled rectangle under save/restore, translucent.
    s.op(SAVE, &[]).op(TRANSFORM, &[1., 0., 0., 1., 320., 250.]).op(TRANSFORM, &[0.866, 0.5, -0.5, 0.866, 0., 0.]);
    s.op(GLOBAL_ALPHA, &[0.6]).op(BEGIN_PATH, &[]).op(RECT, &[-40., -20., 80., 40.]).fill(MAGENTA, false);
    s.op(RESTORE, &[]);
    // A translucent stroke crossing itself: painted once where it overlaps.
    s.op(BEGIN_PATH, &[]).op(MOVE_TO, &[140., 230.]).op(LINE_TO, &[240., 290.]).op(LINE_TO, &[240., 230.]).op(LINE_TO, &[140., 290.]);
    s.stroke([1., 1., 1., 0.5], 12., 0, 0, 10., &[], 0.);
    // A clip rect, then a clip path (a circle) inside it.
    s.op(SAVE, &[]).op(CLIP_RECT, &[20., 285., 90., 10.]).op(BEGIN_PATH, &[]).op(RECT, &[0., 280., 400., 20.]).fill(AMBER, false);
    s.op(RESTORE, &[]);
    s.op(SAVE, &[]).op(BEGIN_PATH, &[]).op(ARC, &[70., 110., 22., 0., 6.2832, 0.]).op(CLIP_PATH, &[0.]);
    s.op(BEGIN_PATH, &[]).op(RECT, &[40., 80., 60., 60.]).fill(CYAN, false);
    s.op(RESTORE, &[]);
    s
}

#[test]
fn shapes_match_tiny_skia() {
    let s = shapes();
    let (w, h) = (400, 300);
    let mut res = Resources { fonts: Fonts::default(), atlas: Atlas::default(), images: Images::new(PathBuf::new()) };
    let mesh = tessellate(&s, "BAT", (w, h), &mut res);
    let ours = rgba_of(&soft::render(&mesh, &res, 4, &[], &[]));
    let theirs = skia(&s.parsed(), w, h);
    save("shapes", w, h, ours.clone());
    save("shapes-tiny-skia", w, h, theirs.data().to_vec());
    let (mut off, mut sum) = (0usize, 0u64);
    let mut diff = Vec::with_capacity(ours.len());
    for (a, b) in ours.chunks(4).zip(theirs.data().chunks(4)) {
        let d = (0..3).map(|i| (a[i] as i32 - b[i] as i32).unsigned_abs()).max().unwrap();
        sum += d as u64;
        if d > 96 {
            off += 1;
        }
        diff.extend([d as u8, d as u8, d as u8, 255]);
    }
    save("shapes-diff", w, h, diff);
    let pixels = (w * h) as usize;
    let mean = sum as f64 / pixels as f64;
    println!("shapes vs tiny-skia: {off} pixels off by more than 96, mean difference {mean:.2}");
    assert!(off * 1000 < pixels * 3, "{off} of {pixels} pixels differ");
    // What more samples would buy, against tiny-skia's analytic coverage
    // (docs/screens.md, anti-aliasing).
    for samples in [1, 4, 8, 16] {
        let img = rgba_of(&soft::render(&mesh, &res, samples, &[], &[]));
        let total: u64 = img
            .chunks(4)
            .zip(theirs.data().chunks(4))
            .map(|(a, b)| (0..3).map(|i| (a[i] as i32 - b[i] as i32).unsigned_abs()).max().unwrap() as u64)
            .sum();
        println!("{samples}x: mean difference from tiny-skia {:.2}", total as f64 / pixels as f64);
    }
    assert!(mean < 2.5, "mean difference {mean}");

    // Spot checks: the evenodd star has its hole, nonzero has none; the
    // clip path keeps the square round; the translucent cross is 50% where
    // it overlaps.
    assert_eq!(pixel(&ours, w, 60, 62)[1], 255);
    assert_eq!(pixel(&ours, w, 180, 62)[1], 0);
    assert_eq!(pixel(&ours, w, 97, 137), [0, 0, 0, 255]);
    assert_eq!(pixel(&ours, w, 70, 110), [0, 255, 255, 255]);
    let centre = pixel(&ours, w, 190, 260);
    assert!((126..=129).contains(&centre[0]), "{centre:?}");
    // Outside the clip rect nothing of the amber band is drawn.
    assert_eq!(pixel(&ours, w, 200, 290), [0, 0, 0, 255]);
}

/// The same commands through tiny-skia: paths built in user space and
/// drawn through the transform, clips as masks, dashes by tiny-skia.
fn skia(ops: &[Op], w: u32, h: u32) -> sk::Pixmap {
    #[derive(Clone)]
    struct St {
        t: sk::Transform,
        alpha: f64,
        mask: Option<sk::Mask>,
    }
    let mut pixmap = sk::Pixmap::new(w, h).unwrap();
    pixmap.fill(sk::Color::BLACK);
    let mut st = St { t: sk::Transform::identity(), alpha: 1., mask: None };
    let mut stack = Vec::new();
    let mut pb = sk::PathBuilder::new();
    let mut current: Option<(f32, f32)> = None;
    let paint = |c: [f32; 4], alpha: f64| {
        let mut p = sk::Paint::default();
        p.set_color(sk::Color::from_rgba(c[0], c[1], c[2], c[3] * alpha as f32).unwrap());
        p.anti_alias = true;
        p
    };
    let clip = |st: &mut St, path: &sk::Path, rule: sk::FillRule| match &mut st.mask {
        Some(m) => m.intersect_path(path, rule, true, st.t),
        None => {
            let mut m = sk::Mask::new(w, h).unwrap();
            m.fill_path(path, rule, true, st.t);
            st.mask = Some(m);
        }
    };
    let rule = |r: FillRule| if r == FillRule::EvenOdd { sk::FillRule::EvenOdd } else { sk::FillRule::Winding };
    for op in ops {
        match op {
            Op::Save => stack.push(st.clone()),
            Op::Restore => st = stack.pop().unwrap(),
            Op::Transform([a, b, c, d, e, f]) => {
                st.t = st.t.pre_concat(sk::Transform::from_row(*a as f32, *b as f32, *c as f32, *d as f32, *e as f32, *f as f32))
            }
            Op::GlobalAlpha(a) => st.alpha *= a,
            Op::ClipRect([x, y, rw, rh]) => {
                let path = sk::PathBuilder::from_rect(sk::Rect::from_xywh(*x as f32, *y as f32, *rw as f32, *rh as f32).unwrap());
                clip(&mut st, &path, sk::FillRule::Winding);
            }
            Op::BeginPath => {
                pb = sk::PathBuilder::new();
                current = None;
            }
            Op::MoveTo(x, y) => {
                pb.move_to(*x as f32, *y as f32);
                current = Some((*x as f32, *y as f32));
            }
            Op::LineTo(x, y) => {
                pb.line_to(*x as f32, *y as f32);
                current = Some((*x as f32, *y as f32));
            }
            Op::QuadTo([cx, cy, x, y]) => {
                pb.quad_to(*cx as f32, *cy as f32, *x as f32, *y as f32);
                current = Some((*x as f32, *y as f32));
            }
            Op::CubicTo([a, b, c, d, x, y]) => {
                pb.cubic_to(*a as f32, *b as f32, *c as f32, *d as f32, *x as f32, *y as f32);
                current = Some((*x as f32, *y as f32));
            }
            Op::Arc { cx, cy, r, start, end, ccw } => {
                // Cubic segments of at most a quarter turn each.
                let sweep = arc_sweep(*start, *end, *ccw);
                let n = (sweep.abs() / std::f64::consts::FRAC_PI_2).ceil().max(1.) as usize;
                let step = sweep / n as f64;
                let k = 4. / 3. * (step / 4.).tan();
                let p = |t: f64| (cx + r * t.cos(), cy + r * t.sin());
                let (sx, sy) = p(*start);
                if current.is_some() {
                    pb.line_to(sx as f32, sy as f32);
                } else {
                    pb.move_to(sx as f32, sy as f32);
                }
                for i in 0..n {
                    let (a0, a1) = (start + step * i as f64, start + step * (i + 1) as f64);
                    let (x0, y0) = p(a0);
                    let (x1, y1) = p(a1);
                    let c1 = (x0 - k * r * a0.sin(), y0 + k * r * a0.cos());
                    let c2 = (x1 + k * r * a1.sin(), y1 - k * r * a1.cos());
                    pb.cubic_to(c1.0 as f32, c1.1 as f32, c2.0 as f32, c2.1 as f32, x1 as f32, y1 as f32);
                    current = Some((x1 as f32, y1 as f32));
                }
            }
            Op::Rect([x, y, rw, rh]) => {
                pb.push_rect(sk::Rect::from_xywh(*x as f32, *y as f32, *rw as f32, *rh as f32).unwrap());
                current = Some((*x as f32, *y as f32));
            }
            Op::ClosePath => pb.close(),
            Op::Fill(c, r) => {
                if let Some(path) = pb.clone().finish() {
                    pixmap.fill_path(&path, &paint(*c, st.alpha), rule(*r), st.t, st.mask.as_ref());
                }
            }
            Op::Stroke(s) => {
                if let Some(path) = pb.clone().finish() {
                    let stroke = sk::Stroke {
                        width: s.width as f32,
                        miter_limit: s.miter_limit as f32,
                        line_cap: match s.cap {
                            Cap::Butt => sk::LineCap::Butt,
                            Cap::Round => sk::LineCap::Round,
                            Cap::Square => sk::LineCap::Square,
                        },
                        line_join: match s.join {
                            Join::Miter => sk::LineJoin::Miter,
                            Join::Round => sk::LineJoin::Round,
                            Join::Bevel => sk::LineJoin::Bevel,
                        },
                        dash: sk::StrokeDash::new(s.dashes.iter().map(|d| *d as f32).collect(), s.dash_offset as f32),
                    };
                    pixmap.stroke_path(&path, &paint(s.colour, st.alpha), &stroke, st.t, st.mask.as_ref());
                }
            }
            Op::ClipPath(r) => {
                if let Some(path) = pb.clone().finish() {
                    clip(&mut st, &path, rule(*r));
                }
            }
            other => panic!("the tiny-skia reference does not draw {other:?}"),
        }
    }
    pixmap
}

#[test]
fn text_is_drawn_where_it_is_measured() {
    if !have_package() {
        return println!("the MSFS package is not on this machine; skipped");
    }
    let mut res = resources();
    let (w, h) = (768, 400);
    let mut s = S::default();
    let mut displays = Tess::new(PathBuf::from(PACKAGE_HTML_UI));
    let lines = [("FL350  SPD 250", 30.), ("QNH 1013  STD", 22.), ("V1 142  VR 145  V2 150", 18.), ("NAV  ALT CST  CLB", 14.)];
    let mut y = 50.;
    for (text, size) in lines {
        let width = displays.measure_text(Some("SCREEN_DU_PFDL"), "Ecam", size, text);
        assert!(width > 0.);
        // A left-aligned run from x, and a green bar under exactly its
        // measured width; the same run right-aligned at x + width must land
        // on the same pixels.
        s.text(text, "Ecam", size, 20., y, 0, 0, WHITE, 0.);
        s.op(BEGIN_PATH, &[]).op(RECT, &[20., y + 3., width, 2.]).fill(GREEN, false);
        s.text(text, "Ecam", size, 400. + width, y, 2, 0, CYAN, 0.);
        y += size * 1.8;
    }
    // Baselines against a guide, centred, stroked, rotated, and the ISIS's
    // own "Ecam", the FCU's "Digital".
    s.op(BEGIN_PATH, &[]).op(MOVE_TO, &[20., 300.]).op(LINE_TO, &[748., 300.]).stroke(MAGENTA, 1., 0, 0, 10., &[], 0.);
    for (i, baseline) in [0u8, 1, 2, 3, 4].into_iter().enumerate() {
        s.text("H1", "Ecam", 28., 40. + i as f64 * 60., 300., 0, baseline, AMBER, 0.);
    }
    s.text("CENTRE", "Ecam", 24., 500., 280., 1, 0, WHITE, 0.);
    s.text("STROKE", "Ecam", 34., 440., 340., 0, 0, [0., 0., 0., 1.], 1.5);
    s.op(SAVE, &[]).op(TRANSFORM, &[0.94, 0.34, -0.34, 0.94, 640., 330.]).text("ROLL 20", "Ecam", 20., 0., 0., 1, 1, GREEN, 0.);
    s.op(RESTORE, &[]);
    s.text("123.5", "Digital", 40., 360., 390., 0, 0, AMBER, 0.);
    let mesh = tessellate(&s, "FCU", (w, h), &mut res);
    let rgba = rgba_of(&soft::render(&mesh, &res, 4, &[], &[]));
    save("text", w, h, rgba);

    // The left run and the right-aligned run are the same glyph quads,
    // shifted by 380 px, to the atlas's quarter-pixel placement.
    let glyph_batches: Vec<_> = mesh.batches.iter().filter(|b| b.paint == super::tessellate::Paint::Atlas).collect();
    assert!(!glyph_batches.is_empty());
    let mut s2 = S::default();
    let mut s3 = S::default();
    let width = displays.measure_text(Some("SCREEN_DU_PFDL"), "Ecam", 22., "QNH 1013  STD");
    s2.text("QNH 1013  STD", "Ecam", 22., 20., 50., 0, 0, WHITE, 0.);
    s3.text("QNH 1013  STD", "Ecam", 22., 400. + width, 50., 2, 0, WHITE, 0.);
    let a = tessellate(&s2, "SCREEN_DU_PFDL", (w, h), &mut res);
    let b = tessellate(&s3, "SCREEN_DU_PFDL", (w, h), &mut res);
    assert_eq!(a.vertices.len(), b.vertices.len());
    for (va, vb) in a.vertices.iter().zip(&b.vertices) {
        assert!((vb.x - va.x - 380.).abs() <= 0.25, "{} vs {}", va.x, vb.x);
        assert_eq!((va.y, va.u, va.v), (vb.y, vb.u, vb.v));
    }
    // The last glyph's advance ends at the measured width.
    let face = res.fonts.select("SCREEN_DU_PFDL", "Ecam", 400, false).unwrap();
    let run = res.fonts.layout(face, 22., "QNH 1013  STD");
    assert!((run.width - width).abs() < 1e-9);
    // The ISIS and the FCU get their own fonts under shared family names.
    let pfd = res.fonts.select("SCREEN_DU_PFDL", "Ecam", 400, false);
    let isis = res.fonts.select("SCREEN_ISIS_1", "Ecam", 400, false);
    let fcu = res.fonts.select("FCU", "Digital", 900, false);
    let bat = res.fonts.select("BAT", "Digital", 900, false);
    assert!(pfd != isis && fcu != bat && fcu.is_some() && bat.is_some());
    // Measured without a screen, a shared family is the common face.
    assert_eq!(res.fonts.select("", "Ecam", 400, false), pfd);
}

#[test]
fn every_font_face_loads() {
    if !have_package() {
        return println!("the MSFS package is not on this machine; skipped");
    }
    let (fonts, problems) = Fonts::load(&Path::new(PACKAGE_HTML_UI).join(super::FONTS_DIR));
    assert!(problems.is_empty(), "{problems:?}");
    assert!(!fonts.is_empty());
}

#[test]
fn images_and_gradients_draw() {
    if !have_package() {
        return println!("the MSFS package is not on this machine; skipped");
    }
    let mut res = resources();
    let url = "/Images/fbw-a380x/TRIM_INDICATOR.png";
    let index = res.images.get(url).expect("the image loads");
    let (iw, ih) = (res.images.pictures[index].width as f64, res.images.pictures[index].height as f64);
    let (w, h) = (512, 512);
    let mut s = S::default();
    let u = s.string(url);
    s.op(IMAGE, &[u, 0., 0., iw, ih, 20., 20., iw, ih]);
    // Scaled down, which takes the mipmaps, and half transparent.
    s.op(SAVE, &[]).op(GLOBAL_ALPHA, &[0.5]).op(IMAGE, &[u, 0., 0., iw, ih, 300., 20., iw / 3., ih / 3.]).op(RESTORE, &[]);
    s.op(BEGIN_PATH, &[]).op(RECT, &[20., 400., 472., 80.]);
    // Stops: offset, r, g, b, a.
    let stops = [0., 0., 0., 1., 1., 0.5, 0., 1., 0., 1., 1., 1., 0., 0., 1.];
    let mut args = vec![20., 0., 492., 0., 3.];
    args.extend(stops);
    args.push(0.);
    s.op(LINEAR_GRADIENT_FILL, &args);
    let mesh = tessellate(&s, "SCREEN_DU_SD", (w, h), &mut res);
    let rgba = rgba_of(&soft::render(&mesh, &res, 4, &[], &[]));
    save("images", w, h, rgba.clone());
    // The gradient runs blue, green, red.
    let left = pixel(&rgba, w, 22, 440);
    let middle = pixel(&rgba, w, 256, 440);
    let right = pixel(&rgba, w, 489, 440);
    assert!(left[2] > 240 && left[0] < 15, "{left:?}");
    assert!(middle[1] > 240 && middle[0] < 15, "{middle:?}");
    assert!(right[0] > 240 && right[2] < 15, "{right:?}");
}

#[test]
fn the_mfd_sides_dim_with_their_own_knobs() {
    let def = &SCREENS[screens::find("SCREEN_DU_MFD").unwrap()];
    let (w, h) = (def.width, def.height);
    let mut res = Resources { fonts: Fonts::default(), atlas: Atlas::default(), images: Images::new(PathBuf::new()) };
    let mut s = S::default();
    s.op(BEGIN_PATH, &[]).op(RECT, &[0., 0., w as f64, h as f64]).fill(WHITE, false);
    let mut mesh = tessellate(&s, def.id, (w, h), &mut res);
    let regions: Vec<[f32; 4]> = def.dimming.iter().map(|d| [d.region[0], d.region[1], d.region[0] + d.region[2], d.region[1] + d.region[3]].map(|v| v as f32)).collect();
    add_dimming(&mut mesh, &regions);
    // Knobs as the model has them: captain's at 100%, first officer's at
    // 25%, through the same variables the plugin reads.
    let mut displays = Displays::new(None, PathBuf::new());
    displays.update_brightness(|name| match name {
        "LIGHT POTENTIOMETER:98" => Some(1.),
        "LIGHT POTENTIOMETER:99" => Some(0.25),
        "A32NX_ELEC_DC_1_BUS_IS_POWERED" => Some(1.),
        _ => None,
    });
    let brightness = displays.screens[screens::find("SCREEN_DU_MFD").unwrap()].brightness.clone();
    assert_eq!(brightness, vec![1., 0.25]);
    let rgba = rgba_of(&soft::render(&mesh, &res, 1, &brightness, &[]));
    save("mfd-dimming", w, h, rgba.clone());
    assert_eq!(pixel(&rgba, w, 300, 500), [255, 255, 255, 255]);
    assert_eq!(pixel(&rgba, w, 1200, 500)[0], 64);
    // The gap between the MFDs is on neither mesh and stays as drawn.
    assert_eq!(pixel(&rgba, w, 820, 500), [255, 255, 255, 255]);
    // Without power the EWD region goes dark whatever its knob says.
    displays.update_brightness(|name| (name == "LIGHT POTENTIOMETER:92").then_some(1.));
    assert_eq!(displays.screens[screens::find("SCREEN_DU_EWD").unwrap()].brightness, vec![0.]);
}

/// A primary-flight-display-sized stream: tapes of ticks and labels inside
/// clip rectangles, an attitude sphere inside a clip path under rotation,
/// dashed and translucent strokes.
fn pfd_like(scale_marks: usize) -> S {
    let mut s = S::default();
    s.op(BEGIN_PATH, &[]).op(RECT, &[0., 0., 768., 1024.]).fill([0.05, 0.05, 0.08, 1.], false);
    // The attitude sphere, rolled and clipped to a rounded window.
    s.op(SAVE, &[]).op(BEGIN_PATH, &[]).op(MOVE_TO, &[200., 250.]).op(LINE_TO, &[560., 250.]);
    s.op(QUAD_TO, &[600., 250., 600., 290.]).op(LINE_TO, &[600., 610.]).op(QUAD_TO, &[600., 650., 560., 650.]);
    s.op(LINE_TO, &[200., 650.]).op(QUAD_TO, &[160., 650., 160., 610.]).op(LINE_TO, &[160., 290.]).op(QUAD_TO, &[160., 250., 200., 250.]);
    s.op(CLIP_PATH, &[0.]);
    s.op(TRANSFORM, &[0.966, -0.259, 0.259, 0.966, 380., 450.]);
    s.op(BEGIN_PATH, &[]).op(RECT, &[-600., -600., 1200., 600.]).fill([0.05, 0.6, 1., 1.], false);
    s.op(BEGIN_PATH, &[]).op(RECT, &[-600., 0., 1200., 600.]).fill([0.6, 0.35, 0.1, 1.], false);
    for i in -8..=8 {
        let y = i as f64 * 20.;
        let half = if i % 2 == 0 { 60. } else { 25. };
        s.op(BEGIN_PATH, &[]).op(MOVE_TO, &[-half, y]).op(LINE_TO, &[half, y]).stroke(WHITE, 2., 0, 0, 10., &[], 0.);
        if i % 2 == 0 && i != 0 {
            s.text(&format!("{}", (i * 5_i32).abs()), "Ecam", 20., -half - 8., y, 2, 1, WHITE, 0.);
        }
    }
    s.op(RESTORE, &[]);
    // Speed and altitude tapes.
    for (x, labels) in [(40., 30), (650., 500)] {
        s.op(SAVE, &[]).op(CLIP_RECT, &[x - 30., 250., 110., 400.]);
        s.op(BEGIN_PATH, &[]).op(RECT, &[x - 30., 250., 110., 400.]).fill([0.2, 0.2, 0.2, 1.], false);
        for k in 0..scale_marks {
            let y = 250. + k as f64 * (400. / scale_marks as f64) * 4.;
            s.op(BEGIN_PATH, &[]).op(MOVE_TO, &[x + 50., y]).op(LINE_TO, &[x + 70., y]).stroke(WHITE, 2., 0, 0, 10., &[], 0.);
            if k % 2 == 0 {
                s.text(&format!("{}", labels * k), "Ecam", 22., x + 45., y, 2, 1, WHITE, 0.);
            }
        }
        s.op(RESTORE, &[]);
    }
    // Flight mode annunciator boxes, dashed, and the heading arc.
    for i in 0..5 {
        s.op(BEGIN_PATH, &[]).op(RECT, &[10. + i as f64 * 150., 10., 140., 90.]).stroke(WHITE, 2., 0, 0, 10., &[6., 4.], 0.);
        s.text("CLB", "Ecam", 26., 80. + i as f64 * 150., 40., 1, 1, GREEN, 0.);
        s.text("NAV", "Ecam", 26., 80. + i as f64 * 150., 75., 1, 1, CYAN, 0.);
    }
    s.op(BEGIN_PATH, &[]).op(ARC, &[384., 1400., 600., 4.1, 5.3, 0.]).stroke(WHITE, 3., 1, 1, 10., &[], 0.);
    s.op(BEGIN_PATH, &[]).op(ARC, &[384., 450., 150., 0., 6.2832, 0.]).stroke([1., 1., 0., 0.5], 8., 0, 0, 10., &[], 0.);
    s
}

#[test]
fn a_pfd_like_stream_draws_in_few_calls() {
    if !have_package() {
        return println!("the MSFS package is not on this machine; skipped");
    }
    let mut res = resources();
    let s = pfd_like(40);
    let started = Instant::now();
    let mesh = tessellate(&s, "SCREEN_DU_PFDL", (768, 1024), &mut res);
    let ms = started.elapsed().as_secs_f64() * 1000.;
    let steps = plan::plan(&mesh);
    let calls = plan::draw_calls(&steps);
    println!(
        "pfd-like: {} numbers, {} vertices, {} batches, {} draw calls, {} stencil clears, tessellated in {ms:.2} ms",
        s.ops.len(),
        mesh.vertices.len(),
        mesh.batches.len(),
        calls,
        steps.iter().filter(|x| matches!(x, Step::ClearStencil)).count()
    );
    let rgba = rgba_of(&soft::render(&mesh, &res, 4, &[], &[]));
    save("pfd-like", 768, 1024, rgba);
    // Paths and text share the atlas, so consecutive draws in one clip are
    // one batch: each clip region costs a call or two, whatever its number
    // of ticks and labels.
    assert!(calls <= 8, "{calls} draw calls");
    let more = tessellate(&pfd_like(400), "SCREEN_DU_PFDL", (768, 1024), &mut res);
    assert_eq!(plan::draw_calls(&plan::plan(&more)), calls);
}

#[test]
fn large_streams_tessellate_quickly() {
    if !have_package() {
        return println!("the MSFS package is not on this machine; skipped");
    }
    let mut res = resources();
    // A navigation-display-sized worst case: 4000 dashed arcs and polylines,
    // 3000 labels, 2000 filled symbols.
    let mut s = S::default();
    for i in 0..4000 {
        let (x, y) = ((i * 37 % 768) as f64, (i * 91 % 1024) as f64);
        if i % 2 == 0 {
            s.op(BEGIN_PATH, &[]).op(ARC, &[x, y, 20. + (i % 50) as f64, 0., 2.5, 0.]).stroke(CYAN, 2., 0, 0, 10., &[8., 4.], 0.);
        } else {
            s.op(BEGIN_PATH, &[]).op(MOVE_TO, &[x, y]).op(LINE_TO, &[x + 30., y + 10.]).op(LINE_TO, &[x + 50., y - 20.]);
            s.stroke(MAGENTA, 3., 1, 1, 10., &[], 0.);
        }
    }
    for i in 0..3000 {
        s.text(&format!("WPT{:03}", i % 1000), "Ecam", 18. + (i % 3) as f64, (i * 53 % 700) as f64, (i * 29 % 1000) as f64, 0, 0, WHITE, 0.);
    }
    for i in 0..2000 {
        let (x, y) = ((i * 13 % 760) as f64, (i * 71 % 1020) as f64);
        s.op(BEGIN_PATH, &[]).op(MOVE_TO, &[x, y - 6.]).op(LINE_TO, &[x + 6., y]).op(LINE_TO, &[x, y + 6.]).op(LINE_TO, &[x - 6., y]);
        s.op(CLOSE_PATH, &[]).fill(GREEN, false);
    }
    let parsed_at = Instant::now();
    let ops = s.parsed();
    let parse_ms = parsed_at.elapsed().as_secs_f64() * 1000.;
    // Warm: the atlas holds the glyphs, as it does after the first frame.
    let _ = tessellate(&s, "SCREEN_DU_NDL", (768, 1024), &mut res);
    let mut t = Tessellator::default();
    let started = Instant::now();
    let mesh = t.run(&ops, &s.strings, "SCREEN_DU_NDL", (768, 1024), (1., 1.), &mut res).ok().unwrap();
    let ms = started.elapsed().as_secs_f64() * 1000.;
    let calls = plan::draw_calls(&plan::plan(&mesh));
    println!(
        "large: {} numbers parsed in {parse_ms:.2} ms, {} vertices in {ms:.2} ms, {} draw calls",
        s.ops.len(),
        mesh.vertices.len(),
        calls
    );
    let rgba = rgba_of(&soft::render(&mesh, &res, 4, &[], &[]));
    save("large", 768, 1024, rgba);
    assert!(ms < 250., "{ms} ms");
    assert!(calls <= 8, "{calls} draw calls");
}

#[test]
fn touches_come_back_in_css_pixels() {
    let mut d = Displays::new(None, PathBuf::new());
    let mfd = screens::find("SCREEN_DU_MFD").unwrap();
    // X-Plane's texel (1000, 23), origin bottom-left, on the 1646x1024 MFD.
    d.mouse(mfd, 1000, 23, crate::xp::MOUSE_DOWN, 0);
    d.cursor(mfd, 1001, 23);
    d.mouse(mfd, 1001, 23, crate::xp::MOUSE_UP, 0);
    d.push_event(mfd, "wheel", 10, 1013, 0, -100.);
    let events: Vec<_> = d.events.drain(..).collect();
    assert_eq!(events.len(), 3, "a hover while pressed is not a move: {events:?}");
    assert_eq!((events[0].screen, events[0].kind, events[0].x, events[0].y), ("SCREEN_DU_MFD", "down", 1000.5, 1000.5));
    assert_eq!(events[1].kind, "up");
    assert_eq!((events[2].kind, events[2].x, events[2].y, events[2].delta), ("wheel", 10.5, 10.5, -100.));
}

#[test]
fn native_images_draw_under_the_nd_with_transform_clip_and_alpha() {
    use crate::mapdata::terrain::terronnd::NativeImage;
    use std::sync::Arc;
    // A 4x2 image: red, green, blue, white / black, black, black, black
    // (straight RGBA, row 0 at the top).
    let mut rgba = Vec::new();
    for p in [[255, 0, 0, 255], [0, 255, 0, 255], [0, 0, 255, 255], [255, 255, 255, 255]] {
        rgba.extend(p);
    }
    rgba.extend([0, 0, 0, 255].repeat(4));
    let image = Arc::new(NativeImage { width: 4, height: 2, generation: 1, rgba: rgba.into() });
    let (w, h) = (400, 200);
    let mut res = Resources { fonts: Fonts::default(), atlas: Atlas::default(), images: Images::new(PathBuf::new()) };
    let mut s = S::default();
    let id = s.string("TERRONND_L");
    // As a composed ND stream: the terrain image first, the ND over it.
    s.op(NATIVE_IMAGE, &[id, 0., 0., 400., 200.]);
    s.op(BEGIN_PATH, &[]).op(RECT, &[180., 90., 40., 20.]).fill(MAGENTA, false);
    // The same image again (one texture), clipped, rotated half a turn and
    // at half alpha.
    s.op(SAVE, &[]).op(CLIP_RECT, &[0., 150., 100., 50.]).op(GLOBAL_ALPHA, &[0.5]);
    s.op(TRANSFORM, &[-1., 0., 0., -1., 400., 200.]).op(NATIVE_IMAGE, &[id, 0., 0., 400., 200.]).op(RESTORE, &[]);
    let mesh = tessellate(&s, "SCREEN_DU_NDL", (w, h), &mut res);
    assert_eq!(mesh.natives, vec!["TERRONND_L".to_string()]);

    let drawn = rgba_of(&soft::render(&mesh, &res, 4, &[], &[Some(image)]));
    save("native-image", w, h, drawn.clone());
    // Sampled where linear filtering reads one texel (edges clamp).
    assert_eq!(pixel(&drawn, w, 49, 49), [255, 0, 0, 255]);
    assert_eq!(pixel(&drawn, w, 350, 49), [255, 255, 255, 255]);
    assert_eq!(pixel(&drawn, w, 350, 150), [0, 0, 0, 255]);
    assert_eq!(pixel(&drawn, w, 200, 100), [255, 0, 255, 255]);
    // Rotated: the bottom-left corner shows the image's top-right (white)
    // at half alpha over the black bottom row; outside the clip, nothing.
    let corner = pixel(&drawn, w, 50, 175);
    assert!(corner[..3].iter().all(|c| (126..=129).contains(c)), "{corner:?}");
    assert_eq!(pixel(&drawn, w, 150, 175), [0, 0, 0, 255]);

    // Before the terrain worker has drawn: nothing but the ND.
    let empty = rgba_of(&soft::render(&mesh, &res, 4, &[], &[None]));
    assert_eq!(pixel(&empty, w, 50, 50), [0, 0, 0, 255]);
    assert_eq!(pixel(&empty, w, 200, 100), [255, 0, 255, 255]);
}

/// A regression check against the display-bug report ("fills and strokes
/// produce nothing, text is sheared"): the exact op sequence captured in
/// src/js/dom/tests/golden/pfd.txt's third submit (attitude sphere shown,
/// speed and FMA box), a real stream FlyByWire's PFD component emits, fed
/// through the real tessellator and software rasteriser. Investigating the
/// bug report, this and the two tests below are how it was established
/// that fill/stroke geometry and text placement are both correct for real
/// captured streams; see the handback report for what was and was not
/// found.
#[test]
fn real_pfd_capture_draws_its_vector_geometry() {
    if !have_package() {
        return println!("the MSFS package is not on this machine; skipped");
    }
    let mut res = resources();
    let (w, h) = (768, 1024);
    let mut s = S::default();
    s.op(BEGIN_PATH, &[]).op(RECT, &[0., 0., 768., 1024.]).fill([0., 0., 0., 1.], false);
    s.op(SAVE, &[]);
    s.op(CLIP_RECT, &[0., 0., 768., 1024.]);
    s.op(TRANSFORM, &[4.838, 0., 0., 4.838, 0., 0.161]);
    s.op(BEGIN_PATH, &[]);
    s.op(MOVE_TO, &[105.64, 62.887]);
    s.op(LINE_TO, &[107.212, 62.086]);
    s.op(MOVE_TO, &[105.64, 61.303]);
    s.op(LINE_TO, &[107.212, 60.502]);
    s.stroke([0., 1., 0., 1.], 0.378, 1, 0, 4., &[], 0.);
    s.op(BEGIN_PATH, &[]);
    s.op(MOVE_TO, &[90.858, 44.839]);
    s.op(ELLIPSE, &[68.906, 80.823, 42.133, 42.158, 0., -1.023, -2.119, 1.]);
    s.stroke([1., 1., 1., 1.], 0.378, 1, 0, 4., &[], 0.);
    s.op(BEGIN_PATH, &[]);
    s.op(MOVE_TO, &[68.906, 38.65]);
    s.op(LINE_TO, &[66.388, 34.95]);
    s.op(LINE_TO, &[71.424, 34.95]);
    s.op(LINE_TO, &[68.906, 38.65]);
    s.stroke([1., 1., 0., 1.], 0.605, 1, 1, 4., &[], 0.);
    s.op(SAVE, &[]);
    s.op(TRANSFORM, &[1., 0., 0., 1., 3.25, 0.]);
    s.text("SPEED", "Ecam, monospace", 6., 9.282, 7.128, 0, 0, [0., 1., 0., 1.], 0.);
    s.op(BEGIN_PATH, &[]);
    s.op(MOVE_TO, &[0.706, 1.814]);
    s.op(LINE_TO, &[31.633, 1.814]);
    s.op(LINE_TO, &[31.633, 7.862]);
    s.op(LINE_TO, &[0.706, 7.862]);
    s.op(CLOSE_PATH, &[]);
    s.stroke([0.902, 0.502, 0., 1.], 0.605, 1, 0, 4., &[], 0.);
    s.op(RESTORE, &[]);
    s.text("250", "Ecam, monospace", 6., 15.5, 100., 0, 0, [0., 1., 0., 1.], 0.);
    s.op(RESTORE, &[]);

    let mesh = tessellate(&s, "SCREEN_DU_PFDL", (w, h), &mut res);
    let rgba = rgba_of(&soft::render(&mesh, &res, 4, &[], &[]));
    save("real-pfd-capture", w, h, rgba.clone());
    // The device-space bounding box of the stroked horizon/attitude
    // geometry: scale 4.838 maps user coords ~34..107 to device ~164..518,
    // y ~34..82 to ~164..397. If FILL/STROKE stopped reaching the vertex
    // buffer (the reported fault), this box would be all background.
    let mut lit = 0usize;
    for y in 150..420 {
        for x in 150..530 {
            if pixel(&rgba, w, x, y) != [0, 0, 0, 255] {
                lit += 1;
            }
        }
    }
    assert!(lit > 500, "only {lit} non-background pixels in the stroked geometry's bounding box");
    // The FMA "SPEED" box is a stroked rect at device x 19.1..168.8, y
    // 8.9..38.2 (scale 4.838 composed with a translate(3.25, 0), applied to
    // its user-space corners). Its amber outline must be present near its
    // left edge.
    assert_eq!(pixel(&rgba, w, 19, 20), [230, 128, 0, 255], "the amber FMA box outline is missing or the wrong colour");
}

/// The same kind of check as [`real_pfd_capture_draws_its_vector_geometry`],
/// against src/js/dom/tests/golden/ewd.txt's first submit (EGT gauge:
/// CLIP_PATH nested inside a CLIP_RECT, three stroked ellipses, a rotated
/// filled triangle under a real 45-degree rotation matrix).
#[test]
fn real_ewd_capture_draws_its_vector_geometry() {
    if !have_package() {
        return println!("the MSFS package is not on this machine; skipped");
    }
    let mut res = resources();
    let (w, h) = (768, 1024);
    let mut s = S::default();
    s.op(SAVE, &[]);
    s.op(CLIP_RECT, &[0., 0., 768., 1024.]);
    s.text("620", "sans-serif", 23., 98.5, 111.7, 0, 0, [0., 1., 0., 1.], 0.);
    s.op(BEGIN_PATH, &[]);
    s.op(MOVE_TO, &[160., 100.]);
    s.op(ELLIPSE, &[100., 100., 60., 60., 0., 0., -3.491, 1.]);
    s.stroke([1., 1., 1., 1.], 2., 0, 0, 4., &[], 0.);
    s.op(BEGIN_PATH, &[]);
    s.op(MOVE_TO, &[158., 100.]);
    s.op(ELLIPSE, &[100., 100., 58., 58., 0., 0., -0.349, 1.]);
    s.stroke([1., 0., 0., 1.], 8., 0, 0, 4., &[], 0.);
    s.op(BEGIN_PATH, &[]);
    s.op(MOVE_TO, &[42., 100.]);
    s.op(ELLIPSE, &[100., 100., 58., 58., 0., 3.142, 2.793, 1.]);
    s.stroke([0., 1., 0., 1.], 3., 1, 0, 4., &[], 0.);
    s.op(SAVE, &[]);
    s.op(BEGIN_PATH, &[]);
    s.op(MOVE_TO, &[0., 1000.]);
    s.op(LINE_TO, &[0., 800.]);
    s.op(LINE_TO, &[130., 800.]);
    s.op(LINE_TO, &[130., 1000.]);
    s.op(CLOSE_PATH, &[]);
    s.op(CLIP_PATH, &[0.]);
    s.op(BEGIN_PATH, &[]);
    s.op(MOVE_TO, &[105., 800.]);
    s.op(LINE_TO, &[105., 1000.]);
    s.stroke([1., 1., 1., 1.], 2., 0, 0, 4., &[], 0.);
    s.op(SAVE, &[]);
    s.op(TRANSFORM, &[0.707, 0.707, -0.707, 0.707, 667.15, 189.358]);
    s.op(BEGIN_PATH, &[]);
    s.op(MOVE_TO, &[100., 900.]);
    s.op(LINE_TO, &[110., 895.]);
    s.op(LINE_TO, &[110., 905.]);
    s.op(CLOSE_PATH, &[]);
    s.fill([0., 1., 1., 1.], false);
    s.op(RESTORE, &[]);
    s.op(RESTORE, &[]);
    s.op(RESTORE, &[]);

    let mesh = tessellate(&s, "SCREEN_DU_EWD", (w, h), &mut res);
    // Two levels of clip in one mesh: the whole-screen CLIP_RECT (a
    // scissor) and the nested CLIP_PATH (a stencil path).
    assert_eq!(mesh.clips.len(), 3);
    assert!(mesh.clips[2].paths.len() == 1, "the nested CLIP_PATH did not become a stencil clip: {:?}", mesh.clips);
    let rgba = rgba_of(&soft::render(&mesh, &res, 4, &[], &[]));
    save("real-ewd-capture", w, h, rgba.clone());
    let mut lit = 0usize;
    for y in 20..1024 {
        for x in 0..768 {
            if pixel(&rgba, w, x, y) != [0, 0, 0, 255] {
                lit += 1;
            }
        }
    }
    assert!(lit > 300, "only {lit} non-background pixels for the whole EGT gauge");
    // The gauge's outer white ring passes through device (100, 40): user
    // (100, 40) on the ellipse cx=100,cy=100,r=60 at angle -90 degrees (top).
    assert_ne!(pixel(&rgba, w, 100, 40), [0, 0, 0, 255], "the gauge ring is missing");
    // The rotated cyan triangle fill, well inside its bounds.
    assert_eq!(pixel(&rgba, w, 105, 900), [0, 255, 255, 255]);
}

/// Two TEXT ops at the same y, different x (as "IDLE" and "+0.0" would be
/// on an MFD line) under an anisotropic device scale (scale.0 != scale.1,
/// as a screen whose device texture is not exactly proportional to its CSS
/// size would have): checks that alone does not produce the
/// y-grows-with-x skew described in the bug report (it does not: an
/// anisotropic scale keeps b == c == 0, so [`super::tessellate::Tessellator::glyph_quads`]'s
/// non-upright branch, taken here since `m.a != m.d`, still places every
/// glyph through the same shear-free affine map).
#[test]
fn anisotropic_scale_does_not_skew_text() {
    if !have_package() {
        return println!("the MSFS package is not on this machine; skipped");
    }
    let mut res = resources();
    let (w, h) = (400, 100);
    let mut s = S::default();
    s.text("IDLE", "Ecam", 20., 20., 50., 0, 0, WHITE, 0.);
    s.text("+0.0", "Ecam", 20., 150., 50., 0, 0, WHITE, 0.);
    let ops = s.parsed();
    let mut t = Tessellator::default();
    // A noticeably anisotropic device scale: 2x horizontally, 1x vertically.
    let mesh = t.run(&ops, &s.strings, "SCREEN_DU_MFD", (w * 2, h), (2.0, 1.0), &mut res).ok().unwrap();
    assert!(mesh.problems.is_empty(), "{:?}", mesh.problems);
    let ys: Vec<f32> = mesh.vertices.iter().map(|v| v.y).collect();
    let (min_y, max_y) = (ys.iter().cloned().fold(f32::INFINITY, f32::min), ys.iter().cloned().fold(f32::NEG_INFINITY, f32::max));
    let rgba = rgba_of(&soft::render(&mesh, &res, 4, &[], &[]));
    save("anisotropic-text", w * 2, h, rgba);
    // With no rotation, both runs share the same baseline y regardless of x:
    // the vertical spread across every glyph vertex should be small (within
    // one glyph's own ascent/descent shape, not a whole line height).
    assert!((max_y - min_y) < 30., "vertices span {} device px vertically: {:?}", max_y - min_y, ys);
}
