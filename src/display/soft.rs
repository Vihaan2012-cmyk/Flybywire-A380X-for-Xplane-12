//! A software rasteriser for the tests: carries out a mesh's plan the way
//! OpenGL does into a multisampled target with a stencil buffer, then
//! resolves it. Coverage is sampled at the standard multisample positions,
//! colour is shaded once per pixel at its centre (as OpenGL shades
//! multisampled triangles without sample shading), textures are filtered
//! as the GL renderer sets them up, and blending is the same.

use std::sync::Arc;

use super::plan::{plan, Step, StencilTest, MARK};
use super::tessellate::{Mesh, Paint, Resources, Vertex, RAMP};
use crate::mapdata::terrain::terronnd::NativeImage;

/// Sample offsets from the pixel centre, in sixteenths of a pixel: the
/// standard 4x and 8x patterns.
fn pattern(samples: usize) -> Vec<(f32, f32)> {
    let raw: &[(i32, i32)] = match samples {
        1 => &[(0, 0)],
        4 => &[(-2, -6), (6, -2), (-6, 2), (2, 6)],
        8 => &[(1, -3), (-1, 3), (5, 1), (-3, -5), (-5, 5), (-7, -1), (3, 7), (7, -7)],
        16 => &[
            (1, 1), (-1, -3), (-3, 2), (4, -1), (-5, -2), (2, 5), (5, 3), (3, -5),
            (-2, 6), (0, -7), (-4, -6), (-6, 4), (-8, 0), (7, -4), (6, 7), (-7, -8),
        ],
        _ => panic!("no {samples}x pattern"),
    };
    raw.iter().map(|&(x, y)| (x as f32 / 16., y as f32 / 16.)).collect()
}

pub struct Target {
    pub width: usize,
    pub height: usize,
    samples: usize,
    offsets: Vec<(f32, f32)>,
    colour: Vec<[f32; 4]>,
    stencil: Vec<u8>,
}

impl Target {
    pub fn new(width: usize, height: usize, samples: usize) -> Self {
        let n = width * height * samples;
        // Cleared to opaque black, as the renderer clears.
        Self { width, height, samples, offsets: pattern(samples), colour: vec![[0., 0., 0., 1.]; n], stencil: vec![0; n] }
    }

    /// The resolved image, RGBA8.
    pub fn resolve(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.width * self.height * 4);
        for p in 0..self.width * self.height {
            let mut sum = [0f32; 4];
            for s in 0..self.samples {
                let c = self.colour[p * self.samples + s];
                for i in 0..4 {
                    sum[i] += c[i];
                }
            }
            out.extend(sum.map(|v| (v / self.samples as f32 * 255. + 0.5).clamp(0., 255.) as u8));
        }
        out
    }
}

struct Texture<'a> {
    levels: Vec<(usize, usize, &'a [u8])>,
    channels: usize,
    mipmapped: bool,
}

impl Texture<'_> {
    fn texel(&self, level: usize, x: i64, y: i64) -> [f32; 4] {
        let (w, h, data) = self.levels[level];
        let x = x.clamp(0, w as i64 - 1) as usize;
        let y = y.clamp(0, h as i64 - 1) as usize;
        let i = (y * w + x) * self.channels;
        if self.channels == 1 {
            [1., 1., 1., data[i] as f32 / 255.]
        } else {
            [0, 1, 2, 3].map(|c| data[i + c] as f32 / 255.)
        }
    }

    fn bilinear(&self, level: usize, u: f32, v: f32) -> [f32; 4] {
        let (w, h, _) = self.levels[level];
        let (x, y) = (u * w as f32 - 0.5, v * h as f32 - 0.5);
        let (x0, y0) = (x.floor(), y.floor());
        let (fx, fy) = (x - x0, y - y0);
        let (x0, y0) = (x0 as i64, y0 as i64);
        let (a, b, c, d) = (self.texel(level, x0, y0), self.texel(level, x0 + 1, y0), self.texel(level, x0, y0 + 1), self.texel(level, x0 + 1, y0 + 1));
        [0, 1, 2, 3].map(|i| (a[i] * (1. - fx) + b[i] * fx) * (1. - fy) + (c[i] * (1. - fx) + d[i] * fx) * fy)
    }

    /// Trilinear when mipmapped, as GL_LINEAR_MIPMAP_LINEAR; `lod` is the
    /// log2 of texels per pixel.
    fn sample(&self, u: f32, v: f32, lod: f32) -> [f32; 4] {
        if !self.mipmapped || lod <= 0. {
            return self.bilinear(0, u, v);
        }
        let top = (self.levels.len() - 1) as f32;
        let lod = lod.min(top);
        let l0 = lod.floor() as usize;
        let l1 = (l0 + 1).min(self.levels.len() - 1);
        let f = lod - l0 as f32;
        let (a, b) = (self.bilinear(l0, u, v), self.bilinear(l1, u, v));
        [0, 1, 2, 3].map(|i| a[i] * (1. - f) + b[i] * f)
    }
}

/// `brightness` is each dimming region's, and `natives` each of the mesh's
/// native images, as the renderer is given them.
pub fn render(mesh: &Mesh, res: &Resources, samples: usize, brightness: &[f32], natives: &[Option<Arc<NativeImage>>]) -> Target {
    let mut target = Target::new(mesh.width as usize, mesh.height as usize, samples);
    let mut scissor: Option<[i32; 4]> = None;
    for step in plan(mesh) {
        match step {
            Step::Scissor(s) => scissor = s,
            Step::ClearStencil => target.stencil.iter_mut().for_each(|s| *s = 0),
            Step::ClearMark => {
                let [x0, y0, x1, y1] = scissor.unwrap_or([0, 0, target.width as i32, target.height as i32]);
                for y in y0.max(0)..y1.min(target.height as i32) {
                    for x in x0.max(0)..x1.min(target.width as i32) {
                        let p = (y as usize * target.width + x as usize) * samples;
                        for s in &mut target.stencil[p..p + samples] {
                            *s &= !MARK;
                        }
                    }
                }
            }
            Step::ClipPath { level, first, count } => {
                let tris = &mesh.vertices[first as usize..(first + count) as usize];
                for tri in tris.chunks_exact(3) {
                    raster(&mut target, tri, scissor, None, |t, i, _| {
                        if t.stencil[i] & super::plan::LEVELS == level {
                            t.stencil[i] = t.stencil[i].wrapping_add(1);
                        }
                    });
                }
            }
            Step::Dim { region, first, count } => {
                let b = brightness.get(region).copied().unwrap_or(1.).clamp(0., 1.);
                if b >= 1. {
                    continue;
                }
                let tris = &mesh.vertices[first as usize..(first + count) as usize];
                for tri in tris.chunks_exact(3) {
                    raster(&mut target, tri, scissor, None, |t, i, _| {
                        // glColor4f(0, 0, 0, 1 - b) with the same blending.
                        let dst = &mut t.colour[i];
                        let a = 1. - b;
                        for c in 0..3 {
                            dst[c] *= 1. - a;
                        }
                        dst[3] = a + dst[3] * (1. - a);
                    });
                }
            }
            Step::Draw { paint, first, count, stencil } => {
                let owned_ramp;
                let texture = match paint {
                    Paint::Atlas => {
                        let size = res.atlas.size as usize;
                        Some(Texture { levels: vec![(size, size, &res.atlas.pixels[..])], channels: 1, mipmapped: false })
                    }
                    Paint::Image(i) => {
                        let p = &res.images.pictures[i];
                        Some(Texture { levels: p.levels.iter().map(|(w, h, d)| (*w as usize, *h as usize, &d[..])).collect(), channels: 4, mipmapped: true })
                    }
                    Paint::Native(i) => match natives.get(i).and_then(|n| n.as_ref()) {
                        Some(n) => Some(Texture { levels: vec![(n.width as usize, n.height as usize, &n.rgba[..])], channels: 4, mipmapped: false }),
                        None => continue,
                    },
                    Paint::Gradients => {
                        owned_ramp = mesh.gradients.concat();
                        Some(Texture { levels: vec![(RAMP, mesh.gradients.len(), &owned_ramp[..])], channels: 4, mipmapped: false })
                    }
                };
                let tris = &mesh.vertices[first as usize..(first + count) as usize];
                for tri in tris.chunks_exact(3) {
                    let lod = texture.as_ref().map_or(0., |t| lod(tri, t.levels[0].0, t.levels[0].1));
                    raster(&mut target, tri, scissor, Some(()), |t, i, shade: Shade| {
                        match stencil {
                            StencilTest::Off => {}
                            StencilTest::Equal { reference, mask, mark } => {
                                if t.stencil[i] & mask != reference & mask {
                                    return;
                                }
                                if mark {
                                    t.stencil[i] ^= MARK;
                                }
                            }
                        }
                        let mut src = shade.colour;
                        if let Some(tex) = &texture {
                            // The texture matrix the renderer sets for gradient rows.
                            let v = if paint == Paint::Gradients { shade.v / mesh.gradients.len() as f32 } else { shade.v };
                            let c = tex.sample(shade.u, v, lod);
                            src = [src[0] * c[0], src[1] * c[1], src[2] * c[2], src[3] * c[3]];
                        }
                        let dst = &mut t.colour[i];
                        let a = src[3];
                        // glBlendFuncSeparate(SRC_ALPHA, ONE_MINUS_SRC_ALPHA, ONE, ONE_MINUS_SRC_ALPHA)
                        for c in 0..3 {
                            dst[c] = src[c] * a + dst[c] * (1. - a);
                        }
                        dst[3] = a + dst[3] * (1. - a);
                    });
                }
            }
        }
    }
    target
}

#[derive(Clone, Copy)]
struct Shade {
    colour: [f32; 4],
    u: f32,
    v: f32,
}

/// Texture level of detail for a triangle: its texture coordinates change
/// linearly across it, so one value holds for the whole triangle.
fn lod(tri: &[Vertex], w: usize, h: usize) -> f32 {
    let (a, b, c) = (tri[0], tri[1], tri[2]);
    let det = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
    if det.abs() < 1e-12 {
        return 0.;
    }
    let grad = |fa: f32, fb: f32, fc: f32| {
        let dx = ((fb - fa) * (c.y - a.y) - (fc - fa) * (b.y - a.y)) / det;
        let dy = ((fc - fa) * (b.x - a.x) - (fb - fa) * (c.x - a.x)) / det;
        (dx, dy)
    };
    let (dudx, dudy) = grad(a.u * w as f32, b.u * w as f32, c.u * w as f32);
    let (dvdx, dvdy) = grad(a.v * h as f32, b.v * h as f32, c.v * h as f32);
    let rho = ((dudx * dudx + dvdx * dvdx).sqrt()).max((dudy * dudy + dvdy * dvdy).sqrt());
    rho.max(1e-9).log2()
}

/// Rasterise one triangle: `per_sample` runs for every covered sample inside
/// the scissor, with the pixel's shading when `shading` is asked for.
fn raster(target: &mut Target, tri: &[Vertex], scissor: Option<[i32; 4]>, shading: Option<()>, mut per_sample: impl FnMut(&mut Target, usize, Shade)) {
    let (a, b, c) = (tri[0], tri[1], tri[2]);
    let area = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
    if area.abs() < 1e-12 || !area.is_finite() {
        return;
    }
    let [sx0, sy0, sx1, sy1] = scissor.unwrap_or([0, 0, target.width as i32, target.height as i32]);
    let x0 = (a.x.min(b.x).min(c.x).floor() as i32).max(sx0).max(0);
    let y0 = (a.y.min(b.y).min(c.y).floor() as i32).max(sy0).max(0);
    let x1 = (a.x.max(b.x).max(c.x).ceil() as i32 + 1).min(sx1).min(target.width as i32);
    let y1 = (a.y.max(b.y).max(c.y).ceil() as i32 + 1).min(sy1).min(target.height as i32);
    let sign = area.signum();
    // Edge functions, positive inside for either winding. A sample exactly
    // on a shared edge belongs to one side only (the top-left rule).
    let edge = |p: Vertex, q: Vertex, x: f32, y: f32| sign * ((q.x - p.x) * (y - p.y) - (q.y - p.y) * (x - p.x));
    let top_left = |p: Vertex, q: Vertex| {
        let (dx, dy) = (sign * (q.x - p.x), sign * (q.y - p.y));
        dy < 0. || (dy == 0. && dx > 0.)
    };
    let inside = |w: f32, tl: bool| w > 0. || (w == 0. && tl);
    let tl = [top_left(b, c), top_left(c, a), top_left(a, b)];
    let offsets = target.offsets.clone();
    let samples = target.samples;
    for y in y0..y1 {
        for x in x0..x1 {
            let (cx, cy) = (x as f32 + 0.5, y as f32 + 0.5);
            let mut shade = Shade { colour: [0.; 4], u: 0., v: 0. };
            let mut shaded = false;
            for (s, &(ox, oy)) in offsets.iter().enumerate() {
                let (px, py) = (cx + ox, cy + oy);
                if !(inside(edge(b, c, px, py), tl[0]) && inside(edge(c, a, px, py), tl[1]) && inside(edge(a, b, px, py), tl[2])) {
                    continue;
                }
                if shading.is_some() && !shaded {
                    let wa = edge(b, c, cx, cy) / (sign * area);
                    let wb = edge(c, a, cx, cy) / (sign * area);
                    let wc = 1. - wa - wb;
                    let lerp = |fa: f32, fb: f32, fc: f32| fa * wa + fb * wb + fc * wc;
                    shade = Shade {
                        colour: [0, 1, 2, 3].map(|i| lerp(a.rgba[i] as f32, b.rgba[i] as f32, c.rgba[i] as f32) / 255.),
                        u: lerp(a.u, b.u, c.u),
                        v: lerp(a.v, b.v, c.v),
                    };
                    shaded = true;
                }
                let i = (y as usize * target.width + x as usize) * samples + s;
                per_sample(target, i, shade);
            }
        }
    }
}
