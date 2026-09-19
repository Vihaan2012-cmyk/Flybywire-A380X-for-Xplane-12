//! A stream's commands turned into triangles, once per stream.
//!
//! Everything a screen draws becomes one vertex list and a short list of
//! batches, each a run of triangles sharing a paint (flat colour, glyph
//! atlas, an image, a gradient) and a clip. Consecutive draws with the same
//! state share a batch, so a whole PFD is a handful of draw calls. The GL
//! renderer and the software renderer the tests use both draw this.
//!
//! Colours and alpha are baked into the vertices; the transform is baked into
//! their positions, in device pixels with y down.

use std::collections::HashMap;
use std::path::PathBuf;

use lyon_tessellation::math::point;
use lyon_tessellation::path::Path as LyonPath;
use lyon_tessellation::{
    BuffersBuilder, FillOptions, FillTessellator, FillVertex, LineCap, LineJoin, StrokeOptions, StrokeTessellator, StrokeVertex,
    VertexBuffers,
};

use super::image::{self, Picture};
use super::path::{dash, Affine, Path, Subpath, TOLERANCE};
use super::stream::{Cap, Colour, FillRule, Gradient, Join, Op, Stroke, Text};
use super::text::{Atlas, Fonts, SUBPIXEL};

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vertex {
    pub x: f32,
    pub y: f32,
    pub u: f32,
    pub v: f32,
    pub rgba: [u8; 4],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Paint {
    /// The glyph atlas, coloured by the vertices: glyph coverage for text,
    /// and its opaque block ([`Mesh::solid_uv`]) for flat colour, so that
    /// paths and text drawn in turn stay one batch.
    Atlas,
    /// An image, by its place in [`Images`].
    Image(usize),
    /// A native image, by its place in [`Mesh::natives`]; looked up when
    /// drawn, since it changes without a new stream.
    Native(usize),
    /// The mesh's gradient ramps, one texture row each; a vertex's v is its
    /// ramp's row plus a half, and the renderer scales v by the row count.
    Gradients,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Batch {
    pub paint: Paint,
    pub clip: usize,
    pub first: u32,
    pub count: u32,
    /// Translucent strokes: each pixel is painted once, however many of the
    /// stroke's triangles overlap there, as a canvas strokes a whole path.
    pub once: bool,
}

/// A clip: a pixel rectangle and the clip paths inside it, all intersected.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Clip {
    /// x0, y0, x1, y1 in device pixels, y down; `None` for the whole screen.
    pub scissor: Option<[i32; 4]>,
    /// Vertex ranges (first, count) of each clip path's triangles.
    pub paths: Vec<(u32, u32)>,
}

/// Width of a gradient ramp texture.
pub const RAMP: usize = 256;

/// Clip paths the stencil buffer can nest: seven bits, the eighth is the
/// translucent stroke mark.
pub const MAX_CLIP_DEPTH: usize = 127;

#[derive(Default)]
pub struct Mesh {
    pub width: u32,
    pub height: u32,
    pub vertices: Vec<Vertex>,
    pub batches: Vec<Batch>,
    /// Index 0 is no clip.
    pub clips: Vec<Clip>,
    /// RGBA ramps, `RAMP` texels each.
    pub gradients: Vec<Vec<u8>>,
    /// The atlas generation the glyph coordinates refer to.
    pub atlas_generation: u64,
    /// Where flat colour samples the atlas.
    pub solid_uv: (f32, f32),
    /// Ids of the native images the mesh draws (NATIVE_IMAGE).
    pub natives: Vec<String>,
    /// Vertex ranges (first, count) of the screen's dimming regions, drawn
    /// over everything else (see [`add_dimming`]).
    pub dims: Vec<(u32, u32)>,
    /// What went wrong while tessellating, for the log.
    pub problems: Vec<String>,
}

/// Quads over a screen's dimming regions (device pixels, y down: x0, y0,
/// x1, y1), drawn last in black at one minus the region's brightness, so
/// each region's pixels come out multiplied by its brightness.
pub fn add_dimming(mesh: &mut Mesh, regions: &[[f32; 4]]) {
    for &[x0, y0, x1, y1] in regions {
        let first = mesh.vertices.len() as u32;
        let v = |x, y| Vertex { x, y, u: 0., v: 0., rgba: [0, 0, 0, 255] };
        mesh.vertices.extend([v(x0, y0), v(x1, y0), v(x1, y1), v(x0, y0), v(x1, y1), v(x0, y1)]);
        mesh.dims.push((first, 6));
    }
}

/// The images every screen draws from, loaded on first use.
pub struct Images {
    pub root: PathBuf,
    pub pictures: Vec<std::sync::Arc<Picture>>,
    by_url: HashMap<String, Result<usize, String>>,
}

impl Images {
    pub fn new(root: PathBuf) -> Self {
        Self { root, pictures: Vec::new(), by_url: HashMap::new() }
    }

    pub fn get(&mut self, url: &str) -> Result<usize, String> {
        if let Some(r) = self.by_url.get(url) {
            return r.clone();
        }
        let result = image::resolve(&self.root, url).and_then(|s| image::load(&s)).map(|p| {
            self.pictures.push(std::sync::Arc::new(p));
            self.pictures.len() - 1
        });
        self.by_url.insert(url.to_string(), result.clone());
        result
    }
}

/// What tessellation draws from: fonts, the glyph atlas and images.
pub struct Resources {
    pub fonts: Fonts,
    pub atlas: Atlas,
    pub images: Images,
}

/// Glyphs larger than this are filled from their outlines rather than
/// taken from the atlas.
const ATLAS_GLYPH_LIMIT: f64 = 160.;

#[derive(Clone, Copy)]
struct State {
    m: Affine,
    alpha: f64,
    clip: usize,
}

pub struct Tessellator {
    fill: FillTessellator,
    stroke: StrokeTessellator,
    buffers: VertexBuffers<[f32; 2], u32>,
}

/// The atlas filled up while tessellating and was started again; the
/// stream has to be tessellated again against the new one.
pub struct Stale;

impl Default for Tessellator {
    fn default() -> Self {
        Self { fill: FillTessellator::new(), stroke: StrokeTessellator::new(), buffers: VertexBuffers::new() }
    }
}

fn rgba(c: Colour, alpha: f64) -> [u8; 4] {
    let q = |v: f32| (v.clamp(0., 1.) * 255. + 0.5) as u8;
    [q(c[0]), q(c[1]), q(c[2]), q((c[3] as f64 * alpha) as f32)]
}

fn lyon_path(subpaths: &[Subpath], map: impl Fn([f64; 2]) -> [f64; 2]) -> LyonPath {
    let mut b = LyonPath::builder();
    for s in subpaths {
        if s.points.len() < 2 {
            continue;
        }
        let p = map(s.points[0]);
        b.begin(point(p[0] as f32, p[1] as f32));
        for q in &s.points[1..] {
            let q = map(*q);
            b.line_to(point(q[0] as f32, q[1] as f32));
        }
        b.end(s.closed);
    }
    b.build()
}

impl Tessellator {
    /// Tessellate a stream for a screen `width` by `height` device pixels,
    /// whose CSS pixels are `scale` device pixels.
    pub fn run(
        &mut self,
        ops: &[Op],
        strings: &[String],
        screen: &str,
        (width, height): (u32, u32),
        scale: (f64, f64),
        res: &mut Resources,
    ) -> Result<Mesh, Stale> {
        let mut mesh = Mesh { width, height, clips: vec![Clip::default()], atlas_generation: res.atlas.generation, solid_uv: res.atlas.solid_uv(), ..Default::default() };
        let base = Affine::scale(scale.0, scale.1);
        let mut state = State { m: base, alpha: 1., clip: 0 };
        let mut stack: Vec<State> = Vec::new();
        let mut path = Path::default();
        for op in ops {
            match op {
                Op::Save => stack.push(state),
                Op::Restore => {
                    if let Some(s) = stack.pop() {
                        state = s;
                    }
                }
                Op::Transform(t) => state.m = state.m.then(&Affine::new(*t)),
                Op::SetTransform(t) => state.m = base.then(&Affine::new(*t)),
                Op::GlobalAlpha(a) => {
                    if a.is_finite() {
                        state.alpha *= a.clamp(0., 1.);
                    }
                }
                Op::ClipRect([x, y, w, h]) => {
                    let mut rect = Path::default();
                    rect.rect(&state.m, [*x, *y, *w, *h]);
                    state.clip = self.clip(&mut mesh, state, &rect, FillRule::NonZero, state.m.is_axis_aligned());
                }
                Op::BeginPath => path.clear(),
                Op::MoveTo(x, y) => path.move_to(state.m.apply(*x, *y)),
                Op::LineTo(x, y) => path.line_to(state.m.apply(*x, *y)),
                Op::QuadTo(q) => path.quad_to(&state.m, *q),
                Op::CubicTo(c) => path.cubic_to(&state.m, *c),
                Op::Arc { cx, cy, r, start, end, ccw } => path.ellipse(&state.m, *cx, *cy, *r, *r, 0., *start, *end, *ccw),
                Op::Ellipse { cx, cy, rx, ry, rotation, start, end, ccw } => {
                    path.ellipse(&state.m, *cx, *cy, *rx, *ry, *rotation, *start, *end, *ccw)
                }
                Op::Rect(r) => path.rect(&state.m, *r),
                Op::ClosePath => path.close(),
                Op::Fill(colour, rule) => {
                    let c = rgba(*colour, state.alpha);
                    if c[3] > 0 && self.visible(&mesh, state) {
                        let (u, v) = mesh.solid_uv;
                        self.fill(&mut mesh, state, &path.subpaths, *rule, Paint::Atlas, |_| (c, u, v));
                    }
                }
                Op::Stroke(stroke) => self.stroke_path(&mut mesh, state, &path.subpaths, stroke),
                Op::ClipPath(rule) => state.clip = self.clip(&mut mesh, state, &path, *rule, false),
                Op::Text(text) => self.text(&mut mesh, state, text, strings, screen, res)?,
                Op::Image(img) => self.image(&mut mesh, state, img, strings, res),
                Op::NativeImage(img) => self.native_image(&mut mesh, state, img, strings),
                Op::LinearGradientFill(g) => self.gradient(&mut mesh, state, &path.subpaths, g),
            }
        }
        Ok(mesh)
    }

    fn visible(&self, mesh: &Mesh, state: State) -> bool {
        !matches!(mesh.clips[state.clip].scissor, Some([x0, y0, x1, y1]) if x0 >= x1 || y0 >= y1)
    }

    /// A clip inside the current one. A rectangle that stays axis-aligned on
    /// screen narrows the scissor; anything else is a stencil clip path.
    fn clip(&mut self, mesh: &mut Mesh, state: State, path: &Path, rule: FillRule, as_scissor: bool) -> usize {
        let parent = mesh.clips[state.clip].clone();
        let full = [0, 0, mesh.width as i32, mesh.height as i32];
        let within = parent.scissor.unwrap_or(full);
        let mut clip = parent;
        if as_scissor {
            let pts: Vec<[f64; 2]> = path.subpaths.iter().flat_map(|s| s.points.iter().copied()).collect();
            let rect = if pts.is_empty() {
                [0, 0, 0, 0]
            } else {
                let min = |i: usize| pts.iter().map(|p| p[i]).fold(f64::INFINITY, f64::min).round() as i32;
                let max = |i: usize| pts.iter().map(|p| p[i]).fold(f64::NEG_INFINITY, f64::max).round() as i32;
                [min(0), min(1), max(0), max(1)]
            };
            clip.scissor = Some([rect[0].max(within[0]), rect[1].max(within[1]), rect[2].min(within[2]), rect[3].min(within[3])]);
        } else if path.is_empty() {
            clip.scissor = Some([0, 0, 0, 0]);
        } else if clip.paths.len() >= MAX_CLIP_DEPTH {
            mesh.problems.push(format!("clip paths nest deeper than {MAX_CLIP_DEPTH}; the deeper ones are ignored"));
        } else {
            let first = mesh.vertices.len() as u32;
            self.fill_triangles(mesh, &path.subpaths, rule, |_| ([255; 4], 0., 0.));
            let count = mesh.vertices.len() as u32 - first;
            if count == 0 {
                clip.scissor = Some([0, 0, 0, 0]);
            } else {
                clip.paths.push((first, count));
            }
        }
        mesh.clips.push(clip);
        mesh.clips.len() - 1
    }

    fn push_batch(mesh: &mut Mesh, paint: Paint, clip: usize, once: bool, first: u32) {
        let count = mesh.vertices.len() as u32 - first;
        if count == 0 {
            return;
        }
        if let Some(last) = mesh.batches.last_mut() {
            if !once && !last.once && last.paint == paint && last.clip == clip && last.first + last.count == first {
                last.count += count;
                return;
            }
        }
        mesh.batches.push(Batch { paint, clip, first, count, once });
    }

    /// Fill triangles for device-space subpaths, straight into the vertex
    /// list; `shade` gives each vertex its colour and texture coordinates.
    fn fill_triangles(&mut self, mesh: &mut Mesh, subpaths: &[Subpath], rule: FillRule, shade: impl Fn([f32; 2]) -> ([u8; 4], f32, f32)) {
        if subpaths.iter().all(|s| s.points.len() < 3) {
            return;
        }
        let lyon = lyon_path(subpaths, |p| p);
        let options = FillOptions::tolerance(TOLERANCE as f32).with_fill_rule(match rule {
            FillRule::NonZero => lyon_tessellation::FillRule::NonZero,
            FillRule::EvenOdd => lyon_tessellation::FillRule::EvenOdd,
        });
        self.buffers.vertices.clear();
        self.buffers.indices.clear();
        let result = self
            .fill
            .tessellate_path(&lyon, &options, &mut BuffersBuilder::new(&mut self.buffers, |v: FillVertex| v.position().to_array()));
        if let Err(e) = result {
            mesh.problems.push(format!("a fill could not be tessellated: {e:?}"));
            return;
        }
        let VertexBuffers { vertices, indices } = &self.buffers;
        mesh.vertices.extend(indices.iter().map(|&i| {
            let [x, y] = vertices[i as usize];
            let (rgba, u, v) = shade([x, y]);
            Vertex { x, y, u, v, rgba }
        }));
    }

    fn fill(&mut self, mesh: &mut Mesh, state: State, subpaths: &[Subpath], rule: FillRule, paint: Paint, shade: impl Fn([f32; 2]) -> ([u8; 4], f32, f32)) {
        let first = mesh.vertices.len() as u32;
        self.fill_triangles(mesh, subpaths, rule, shade);
        Self::push_batch(mesh, paint, state.clip, false, first);
    }

    fn stroke_path(&mut self, mesh: &mut Mesh, state: State, subpaths: &[Subpath], stroke: &Stroke) {
        let c = rgba(stroke.colour, state.alpha);
        if c[3] == 0 || !(stroke.width.is_finite() && stroke.width > 0.) || !self.visible(mesh, state) {
            return;
        }
        // Stroke in user space, where the width is, then map back: a
        // stretched transform makes a stretched line, as on a canvas.
        let Some(inverse) = state.m.inverse() else { return };
        let user: Vec<Subpath> = subpaths
            .iter()
            .map(|s| Subpath { points: s.points.iter().map(|p| inverse.apply(p[0], p[1])).collect(), closed: s.closed })
            .collect();
        let dashed;
        let user = match dash(&user, &stroke.dashes, stroke.dash_offset) {
            Some(d) => {
                dashed = d;
                &dashed
            }
            None => &user,
        };
        if user.iter().all(|s| s.points.len() < 2) {
            return;
        }
        let lyon = lyon_path(user, |p| p);
        let options = StrokeOptions::tolerance((TOLERANCE / state.m.max_scale().max(1e-6)) as f32)
            .with_line_width(stroke.width as f32)
            .with_line_cap(match stroke.cap {
                Cap::Butt => LineCap::Butt,
                Cap::Round => LineCap::Round,
                Cap::Square => LineCap::Square,
            })
            .with_line_join(match stroke.join {
                Join::Miter => LineJoin::Miter,
                Join::Round => LineJoin::Round,
                Join::Bevel => LineJoin::Bevel,
            })
            .with_miter_limit(if stroke.miter_limit.is_finite() { stroke.miter_limit.max(1.) as f32 } else { 10. });
        self.buffers.vertices.clear();
        self.buffers.indices.clear();
        let result = self
            .stroke
            .tessellate_path(&lyon, &options, &mut BuffersBuilder::new(&mut self.buffers, |v: StrokeVertex| v.position().to_array()));
        if let Err(e) = result {
            mesh.problems.push(format!("a stroke could not be tessellated: {e:?}"));
            return;
        }
        let first = mesh.vertices.len() as u32;
        let m = state.m;
        let (u, v) = mesh.solid_uv;
        let VertexBuffers { vertices, indices } = &self.buffers;
        mesh.vertices.extend(indices.iter().map(|&i| {
            let [x, y] = vertices[i as usize];
            let [dx, dy] = m.apply(x as f64, y as f64);
            Vertex { x: dx as f32, y: dy as f32, u, v, rgba: c }
        }));
        Self::push_batch(mesh, Paint::Atlas, state.clip, c[3] < 255, first);
    }

    fn gradient(&mut self, mesh: &mut Mesh, state: State, subpaths: &[Subpath], g: &Gradient) {
        let (dx, dy) = (g.to[0] - g.from[0], g.to[1] - g.from[1]);
        let length2 = dx * dx + dy * dy;
        // A gradient from a point to itself paints nothing.
        if !(length2 > 0.) || g.stops.is_empty() || !self.visible(mesh, state) {
            return;
        }
        let Some(inverse) = state.m.inverse() else { return };
        let mut stops = g.stops.clone();
        stops.retain(|s| s.0.is_finite());
        for s in &mut stops {
            s.0 = s.0.clamp(0., 1.);
        }
        stops.sort_by(|a, b| a.0.total_cmp(&b.0));
        if stops.is_empty() {
            return;
        }
        let mut ramp = Vec::with_capacity(RAMP * 4);
        for i in 0..RAMP {
            let t = (i as f64 + 0.5) / RAMP as f64;
            let c = match stops.iter().position(|s| s.0 > t) {
                None => stops[stops.len() - 1].1,
                Some(0) => stops[0].1,
                Some(k) => {
                    let (a, b) = (stops[k - 1], stops[k]);
                    let f = if b.0 > a.0 { ((t - a.0) / (b.0 - a.0)) as f32 } else { 1. };
                    [0, 1, 2, 3].map(|j| a.1[j] + (b.1[j] - a.1[j]) * f)
                }
            };
            ramp.extend(rgba(c, 1.));
        }
        mesh.gradients.push(ramp);
        let index = mesh.gradients.len() - 1;
        let alpha = rgba([1., 1., 1., 1.], state.alpha);
        let (fx, fy) = (g.from[0], g.from[1]);
        // A texel's centre sits at its ramp position, so the ends of the ramp
        // are its first and last texel.
        let texel = 1. / RAMP as f64;
        let row = index as f32 + 0.5;
        self.fill(mesh, state, subpaths, g.rule, Paint::Gradients, |[x, y]| {
            let [ux, uy] = inverse.apply(x as f64, y as f64);
            let t = ((ux - fx) * dx + (uy - fy) * dy) / length2;
            let u = texel / 2. + t * (1. - texel);
            (alpha, u as f32, row)
        });
    }

    fn native_image(&mut self, mesh: &mut Mesh, state: State, img: &super::stream::NativeImage, strings: &[String]) {
        let [x, y, w, h] = img.dst;
        let c = rgba([1., 1., 1., 1.], state.alpha);
        if ![x, y, w, h].iter().all(|v| v.is_finite()) || w == 0. || h == 0. || c[3] == 0 || !self.visible(mesh, state) {
            return;
        }
        let id = &strings[img.id as usize];
        let index = match mesh.natives.iter().position(|n| n == id) {
            Some(i) => i,
            None => {
                mesh.natives.push(id.clone());
                mesh.natives.len() - 1
            }
        };
        let first = mesh.vertices.len() as u32;
        let corner = |fx: f64, fy: f64| {
            let [dx, dy] = state.m.apply(x + w * fx, y + h * fy);
            Vertex { x: dx as f32, y: dy as f32, u: fx as f32, v: fy as f32, rgba: c }
        };
        let (a, b, cc, d) = (corner(0., 0.), corner(1., 0.), corner(1., 1.), corner(0., 1.));
        mesh.vertices.extend([a, b, cc, a, cc, d]);
        Self::push_batch(mesh, Paint::Native(index), state.clip, false, first);
    }

    fn image(&mut self, mesh: &mut Mesh, state: State, img: &super::stream::Image, strings: &[String], res: &mut Resources) {
        let url = &strings[img.url as usize];
        let [sx, sy, sw, sh] = img.src;
        let [x, y, w, h] = img.dst;
        if ![sx, sy, sw, sh, x, y, w, h].iter().all(|v| v.is_finite()) || sw == 0. || sh == 0. || w == 0. || h == 0. {
            return;
        }
        let index = match res.images.get(url) {
            Ok(i) => i,
            Err(e) => {
                mesh.problems.push(format!("image {url}: {e}"));
                return;
            }
        };
        if !self.visible(mesh, state) {
            return;
        }
        let picture = &res.images.pictures[index];
        let (pw, ph) = (picture.width as f64, picture.height as f64);
        let c = rgba([1., 1., 1., 1.], state.alpha);
        if c[3] == 0 {
            return;
        }
        let first = mesh.vertices.len() as u32;
        let corner = |fx: f64, fy: f64| {
            let [dx, dy] = state.m.apply(x + w * fx, y + h * fy);
            Vertex { x: dx as f32, y: dy as f32, u: ((sx + sw * fx) / pw) as f32, v: ((sy + sh * fy) / ph) as f32, rgba: c }
        };
        let (a, b, cc, d) = (corner(0., 0.), corner(1., 0.), corner(1., 1.), corner(0., 1.));
        mesh.vertices.extend([a, b, cc, a, cc, d]);
        Self::push_batch(mesh, Paint::Image(index), state.clip, false, first);
    }

    fn text(&mut self, mesh: &mut Mesh, state: State, text: &Text, strings: &[String], screen: &str, res: &mut Resources) -> Result<(), Stale> {
        let content = &strings[text.text as usize];
        if content.is_empty() || !(text.size.is_finite() && text.size > 0.) || !self.visible(mesh, state) {
            return Ok(());
        }
        let family = &strings[text.font as usize];
        let Some(face) = res.fonts.select(screen, family, text.weight, text.italic) else { return Ok(()) };
        let run = res.fonts.layout(face, text.size, content);
        let (ox, oy) = res.fonts.anchor_offset(&run, text.size, text.align, text.baseline);
        let (x0, y0) = (text.x + ox, text.y + oy);
        let m = state.m;
        let fill = rgba(text.fill, state.alpha);
        let device_px = text.size * m.mean_scale();
        if fill[3] > 0 && device_px > 0. {
            if device_px > ATLAS_GLYPH_LIMIT {
                let mut outlines = Path::default();
                for &(glyph, gx) in &run.glyphs {
                    res.fonts.outline(face, glyph, text.size, &m, (x0 + gx, y0), &mut outlines);
                }
                let (u, v) = mesh.solid_uv;
                self.fill(mesh, state, &outlines.subpaths, FillRule::NonZero, Paint::Atlas, |_| (fill, u, v));
            } else {
                self.glyph_quads(mesh, state, &run, (x0, y0), text.size, fill, res)?;
            }
        }
        let stroke = rgba(text.stroke, state.alpha);
        if text.stroke_width > 0. && stroke[3] > 0 {
            let mut outlines = Path::default();
            for &(glyph, gx) in &run.glyphs {
                res.fonts.outline(face, glyph, text.size, &m, (x0 + gx, y0), &mut outlines);
            }
            let style = Stroke {
                colour: text.stroke,
                width: text.stroke_width,
                cap: Cap::Butt,
                join: Join::Miter,
                miter_limit: 4.,
                dashes: Vec::new(),
                dash_offset: 0.,
            };
            self.stroke_path(mesh, state, &outlines.subpaths, &style);
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn glyph_quads(
        &mut self,
        mesh: &mut Mesh,
        state: State,
        run: &super::text::Run,
        (x0, y0): (f64, f64),
        size: f64,
        colour: [u8; 4],
        res: &mut Resources,
    ) -> Result<(), Stale> {
        let m = state.m;
        let first = mesh.vertices.len() as u32;
        let atlas_size = res.atlas.size as f32;
        // Upright, unflipped text at a uniform scale lands on whole pixels
        // vertically and quarter pixels horizontally, which keeps it crisp.
        let upright = m.b.abs() < 1e-9 && m.c.abs() < 1e-9 && m.a > 0. && (m.a - m.d).abs() < 1e-9;
        let px = size * m.mean_scale();
        let generation = res.atlas.generation;
        for &(glyph, gx) in &run.glyphs {
            let (entry, quad) = if upright {
                let [dx, dy] = m.apply(x0 + gx, y0);
                let quarters = (dx * SUBPIXEL as f64).round();
                let whole = (quarters / SUBPIXEL as f64).floor();
                let subpixel = (quarters - whole * SUBPIXEL as f64) as u32;
                let Some(e) = res.atlas.glyph(&res.fonts, run.face, glyph, px, subpixel).map_err(|_| Stale)? else { continue };
                let (left, top) = (whole as f32 + e.left, dy.round() as f32 + e.top);
                let (w, h) = (e.w as f32, e.h as f32);
                (e, [[left, top], [left + w, top], [left + w, top + h], [left, top + h]])
            } else {
                let Some(e) = res.atlas.glyph(&res.fonts, run.face, glyph, px, 0).map_err(|_| Stale)? else { continue };
                let s = m.mean_scale();
                let corner = |fx: f32, fy: f32| {
                    let [dx, dy] = m.apply(x0 + gx + fx as f64 / s, y0 + fy as f64 / s);
                    [dx as f32, dy as f32]
                };
                let (l, t, w, h) = (e.left, e.top, e.w as f32, e.h as f32);
                (e, [corner(l, t), corner(l + w, t), corner(l + w, t + h), corner(l, t + h)])
            };
            debug_assert_eq!(generation, res.atlas.generation);
            let (u0, v0) = (entry.x as f32 / atlas_size, entry.y as f32 / atlas_size);
            let (u1, v1) = ((entry.x + entry.w) as f32 / atlas_size, (entry.y + entry.h) as f32 / atlas_size);
            let uv = [[u0, v0], [u1, v0], [u1, v1], [u0, v1]];
            let v = |i: usize| Vertex { x: quad[i][0], y: quad[i][1], u: uv[i][0], v: uv[i][1], rgba: colour };
            mesh.vertices.extend([v(0), v(1), v(2), v(0), v(2), v(3)]);
        }
        Self::push_batch(mesh, Paint::Atlas, state.clip, false, first);
        Ok(())
    }
}
