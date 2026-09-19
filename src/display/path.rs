//! Paths as a canvas builds them: points taken through the transform in force
//! when they are added, curves and arcs flattened into line segments there
//! and then, to a tolerance in device pixels.

use std::f64::consts::{PI, TAU};

/// A 2-D affine transform, in canvas order: x' = a x + c y + e,
/// y' = b x + d y + f.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Affine {
    pub a: f64,
    pub b: f64,
    pub c: f64,
    pub d: f64,
    pub e: f64,
    pub f: f64,
}

impl Affine {
    #[cfg(test)]
    pub const IDENTITY: Affine = Affine { a: 1., b: 0., c: 0., d: 1., e: 0., f: 0. };

    pub fn new([a, b, c, d, e, f]: [f64; 6]) -> Self {
        Self { a, b, c, d, e, f }
    }

    pub fn scale(sx: f64, sy: f64) -> Self {
        Self { a: sx, b: 0., c: 0., d: sy, e: 0., f: 0. }
    }

    #[cfg(test)]
    pub fn translate(x: f64, y: f64) -> Self {
        Self { e: x, f: y, ..Self::IDENTITY }
    }

    /// `self` then `m` applied first: the canvas `transform` call.
    pub fn then(&self, m: &Affine) -> Affine {
        Affine {
            a: self.a * m.a + self.c * m.b,
            b: self.b * m.a + self.d * m.b,
            c: self.a * m.c + self.c * m.d,
            d: self.b * m.c + self.d * m.d,
            e: self.a * m.e + self.c * m.f + self.e,
            f: self.b * m.e + self.d * m.f + self.f,
        }
    }

    pub fn apply(&self, x: f64, y: f64) -> [f64; 2] {
        [self.a * x + self.c * y + self.e, self.b * x + self.d * y + self.f]
    }

    pub fn det(&self) -> f64 {
        self.a * self.d - self.b * self.c
    }

    pub fn inverse(&self) -> Option<Affine> {
        let det = self.det();
        if !det.is_finite() || det.abs() < 1e-12 {
            return None;
        }
        let (a, b, c, d) = (self.d / det, -self.b / det, -self.c / det, self.a / det);
        Some(Affine { a, b, c, d, e: -(a * self.e + c * self.f), f: -(b * self.e + d * self.f) })
    }

    /// How much the transform scales lengths, taken as the square root of its
    /// area scale: what a line width or a glyph grows by.
    pub fn mean_scale(&self) -> f64 {
        self.det().abs().sqrt()
    }

    /// The most it stretches any direction, for flattening tolerances.
    pub fn max_scale(&self) -> f64 {
        let (a, b, c, d) = (self.a, self.b, self.c, self.d);
        let s = a * a + b * b + c * c + d * d;
        let det = self.det();
        ((s + (s * s - 4. * det * det).max(0.).sqrt()) / 2.).sqrt()
    }

    /// Whether it keeps rectangles axis-aligned (scaling, flips, translation,
    /// quarter turns).
    pub fn is_axis_aligned(&self) -> bool {
        const E: f64 = 1e-9;
        (self.b.abs() < E && self.c.abs() < E) || (self.a.abs() < E && self.d.abs() < E)
    }
}

/// One subpath: its points, in device pixels, and whether it was closed.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Subpath {
    pub points: Vec<[f64; 2]>,
    pub closed: bool,
}

/// The current path of a canvas.
#[derive(Clone, Debug, Default)]
pub struct Path {
    pub subpaths: Vec<Subpath>,
    /// Where the next subpath starts after `close`, until something moves.
    pending_start: Option<[f64; 2]>,
}

/// Flattening tolerance, in device pixels: how far a segment may stray from
/// the curve it stands for.
pub const TOLERANCE: f64 = 0.1;

impl Path {
    pub fn clear(&mut self) {
        self.subpaths.clear();
        self.pending_start = None;
    }

    pub fn is_empty(&self) -> bool {
        self.subpaths.iter().all(|s| s.points.len() < 2)
    }

    fn open(&mut self) -> Option<&mut Subpath> {
        match self.subpaths.last() {
            Some(s) if !s.closed => self.subpaths.last_mut(),
            _ => None,
        }
    }

    fn last_point(&self) -> Option<[f64; 2]> {
        match self.subpaths.last() {
            Some(s) if !s.closed => s.points.last().copied(),
            _ => self.pending_start,
        }
    }

    pub fn move_to(&mut self, p: [f64; 2]) {
        if !p[0].is_finite() || !p[1].is_finite() {
            return;
        }
        self.pending_start = None;
        match self.subpaths.last_mut() {
            // A subpath with a single point draws nothing: reuse it.
            Some(s) if !s.closed && s.points.len() == 1 => s.points[0] = p,
            _ => self.subpaths.push(Subpath { points: vec![p], closed: false }),
        }
    }

    pub fn line_to(&mut self, p: [f64; 2]) {
        if !p[0].is_finite() || !p[1].is_finite() {
            return;
        }
        if self.open().is_none() {
            match self.pending_start.take() {
                Some(start) => self.subpaths.push(Subpath { points: vec![start], closed: false }),
                // Canvas: a line with no current point starts one.
                None => return self.move_to(p),
            }
        }
        let s = self.open().expect("an open subpath");
        if s.points.last() != Some(&p) {
            s.points.push(p);
        }
    }

    pub fn close(&mut self) {
        if let Some(s) = self.open() {
            let start = s.points[0];
            s.closed = true;
            self.pending_start = Some(start);
        }
    }

    pub fn quad_to(&mut self, m: &Affine, [cx, cy, x, y]: [f64; 4]) {
        let p1 = m.apply(cx, cy);
        let p2 = m.apply(x, y);
        let Some(p0) = self.last_point() else {
            self.move_to(p1);
            return self.line_to(p2);
        };
        // Wang's formula for a quadratic.
        let dd = ((p0[0] - 2. * p1[0] + p2[0]).powi(2) + (p0[1] - 2. * p1[1] + p2[1]).powi(2)).sqrt();
        let n = segments((dd / (4. * TOLERANCE)).sqrt());
        for i in 1..=n {
            let t = i as f64 / n as f64;
            let u = 1. - t;
            self.line_to([
                u * u * p0[0] + 2. * u * t * p1[0] + t * t * p2[0],
                u * u * p0[1] + 2. * u * t * p1[1] + t * t * p2[1],
            ]);
        }
    }

    pub fn cubic_to(&mut self, m: &Affine, [c1x, c1y, c2x, c2y, x, y]: [f64; 6]) {
        let p1 = m.apply(c1x, c1y);
        let p2 = m.apply(c2x, c2y);
        let p3 = m.apply(x, y);
        let Some(p0) = self.last_point() else {
            self.move_to(p1);
            return self.cubic_to(m, [c1x, c1y, c2x, c2y, x, y]);
        };
        let d = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| ((a[0] - 2. * b[0] + c[0]).powi(2) + (a[1] - 2. * b[1] + c[1]).powi(2)).sqrt();
        let dd = d(p0, p1, p2).max(d(p1, p2, p3));
        // Wang's formula for a cubic: n = sqrt(3 * 2 / 8 * dd / tolerance).
        let n = segments((0.75 * dd / TOLERANCE).sqrt());
        for i in 1..=n {
            let t = i as f64 / n as f64;
            let u = 1. - t;
            let (a, b, c, e) = (u * u * u, 3. * u * u * t, 3. * u * t * t, t * t * t);
            self.line_to([
                a * p0[0] + b * p1[0] + c * p2[0] + e * p3[0],
                a * p0[1] + b * p1[1] + c * p2[1] + e * p3[1],
            ]);
        }
    }

    pub fn rect(&mut self, m: &Affine, [x, y, w, h]: [f64; 4]) {
        if ![x, y, w, h].iter().all(|v| v.is_finite()) {
            return;
        }
        self.move_to(m.apply(x, y));
        self.line_to(m.apply(x + w, y));
        self.line_to(m.apply(x + w, y + h));
        self.line_to(m.apply(x, y + h));
        self.close();
    }

    /// The canvas `ellipse` (and `arc`, with equal radii): a line from the
    /// current point to the arc's start, then the arc.
    #[allow(clippy::too_many_arguments)]
    pub fn ellipse(&mut self, m: &Affine, cx: f64, cy: f64, rx: f64, ry: f64, rotation: f64, start: f64, end: f64, ccw: bool) {
        if ![cx, cy, rx, ry, rotation, start, end].iter().all(|v| v.is_finite()) || rx < 0. || ry < 0. {
            return;
        }
        let sweep = arc_sweep(start, end, ccw);
        let (sin_r, cos_r) = rotation.sin_cos();
        let point = |t: f64| {
            let (px, py) = (rx * t.cos(), ry * t.sin());
            m.apply(cx + px * cos_r - py * sin_r, cy + px * sin_r + py * cos_r)
        };
        let radius = rx.max(ry) * m.max_scale();
        let step = if radius > TOLERANCE { 2. * (1. - TOLERANCE / radius).clamp(-1., 1.).acos() } else { PI / 2. };
        let n = segments(sweep.abs() / step.max(1e-3));
        let first = point(start);
        if self.last_point().is_some() {
            self.line_to(first);
        } else {
            self.move_to(first);
        }
        for i in 1..=n {
            self.line_to(point(start + sweep * i as f64 / n as f64));
        }
    }
}

/// Segment count for a flattened curve, kept sane for absurd inputs.
fn segments(n: f64) -> usize {
    if n.is_finite() {
        (n.ceil() as usize).clamp(1, 4096)
    } else {
        1
    }
}

/// The signed sweep of a canvas arc, as the canvas specification works it
/// out: a full turn when the angles are a turn or more apart in the drawing
/// direction, otherwise the difference brought into one turn.
pub fn arc_sweep(start: f64, end: f64, ccw: bool) -> f64 {
    if !ccw {
        if end - start >= TAU {
            TAU
        } else {
            (end - start).rem_euclid(TAU)
        }
    } else if start - end >= TAU {
        -TAU
    } else {
        -(start - end).rem_euclid(TAU)
    }
}

/// Split polylines into dashes, as a canvas dashes a stroke: the pattern
/// runs on along each subpath from `offset` and starts again for the next.
/// A pattern of odd length is repeated to make it even; one with a negative
/// or non-finite entry, or with nothing but zeros, means a solid line.
pub fn dash(subpaths: &[Subpath], pattern: &[f64], offset: f64) -> Option<Vec<Subpath>> {
    if pattern.is_empty() || pattern.iter().any(|d| !d.is_finite() || *d < 0.) {
        return None;
    }
    let mut pattern = pattern.to_vec();
    if pattern.len() % 2 == 1 {
        pattern.extend_from_within(..);
    }
    let total: f64 = pattern.iter().sum();
    if total <= 0. {
        return None;
    }
    let mut out = Vec::new();
    for sub in subpaths {
        if sub.points.len() < 2 {
            continue;
        }
        let mut points = sub.points.clone();
        if sub.closed {
            points.push(points[0]);
        }
        // Where in the pattern the subpath starts.
        let mut phase = offset.rem_euclid(total);
        let mut index = 0;
        while phase >= pattern[index] {
            phase -= pattern[index];
            index = (index + 1) % pattern.len();
        }
        let mut left = pattern[index] - phase;
        let mut on = index % 2 == 0;
        let mut current: Option<Subpath> = on.then(|| Subpath { points: vec![points[0]], closed: false });
        for w in points.windows(2) {
            let (a, b) = (w[0], w[1]);
            let length = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
            let mut done = 0.;
            while length - done > left {
                done += left;
                let t = done / length;
                let p = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
                if on {
                    let mut dash = current.take().expect("a dash in progress");
                    dash.points.push(p);
                    out.push(dash);
                } else {
                    current = Some(Subpath { points: vec![p], closed: false });
                }
                on = !on;
                index = (index + 1) % pattern.len();
                left = pattern[index];
            }
            left -= length - done;
            if let Some(dash) = current.as_mut() {
                if dash.points.last() != Some(&b) {
                    dash.points.push(b);
                }
            }
        }
        if let Some(dash) = current.take() {
            out.push(dash);
        }
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-6
    }

    #[test]
    fn transforms_compose_in_canvas_order() {
        // translate(10, 0) then scale(2): a point at 1 lands at 12.
        let m = Affine::translate(10., 0.).then(&Affine::scale(2., 2.));
        assert_eq!(m.apply(1., 0.), [12., 0.]);
        let inv = m.inverse().unwrap();
        let p = inv.apply(12., 0.);
        assert!(close(p[0], 1.) && close(p[1], 0.));
        assert!(Affine::new([0., 1., -1., 0., 0., 0.]).is_axis_aligned());
        assert!(!Affine::new([0.7, 0.7, -0.7, 0.7, 0., 0.]).is_axis_aligned());
    }

    #[test]
    fn arcs_sweep_as_a_canvas_does() {
        assert!(close(arc_sweep(0., PI, false), PI));
        assert!(close(arc_sweep(0., PI, true), -PI));
        assert!(close(arc_sweep(0., 3. * TAU, false), TAU));
        assert!(close(arc_sweep(0., -PI / 2., false), 1.5 * PI));
        assert!(close(arc_sweep(0., 0., false), 0.));
    }

    #[test]
    fn a_circle_flattens_within_tolerance() {
        let mut p = Path::default();
        p.ellipse(&Affine::IDENTITY, 50., 50., 40., 40., 0., 0., TAU, false);
        let pts = &p.subpaths[0].points;
        assert!(pts.len() > 16);
        for w in pts.windows(2) {
            let mid = [(w[0][0] + w[1][0]) / 2., (w[0][1] + w[1][1]) / 2.];
            let r = ((mid[0] - 50.).powi(2) + (mid[1] - 50.).powi(2)).sqrt();
            assert!(40. - r <= TOLERANCE + 1e-9, "chord strays {}", 40. - r);
        }
    }

    #[test]
    fn a_line_after_close_starts_at_the_closed_subpath_start() {
        let mut p = Path::default();
        p.move_to([0., 0.]);
        p.line_to([10., 0.]);
        p.line_to([10., 10.]);
        p.close();
        p.line_to([0., 10.]);
        assert_eq!(p.subpaths.len(), 2);
        assert!(p.subpaths[0].closed);
        assert_eq!(p.subpaths[1].points, vec![[0., 0.], [0., 10.]]);
    }

    #[test]
    fn dashes_follow_the_pattern_across_corners() {
        let line = Subpath { points: vec![[0., 0.], [10., 0.], [10., 10.]], closed: false };
        let dashes = dash(&[line], &[4., 2.], 1.).unwrap();
        // Offset 1: 3 on, 2 off, 4 on, 2 off (round the corner), 4 on, 2 off, 3 on.
        let lengths: Vec<f64> = dashes
            .iter()
            .map(|d| d.points.windows(2).map(|w| ((w[1][0] - w[0][0]).powi(2) + (w[1][1] - w[0][1]).powi(2)).sqrt()).sum())
            .collect();
        assert_eq!(lengths.len(), 4);
        for (got, want) in lengths.iter().zip([3., 4., 4., 3.]) {
            assert!(close(*got, want), "{lengths:?}");
        }
        assert_eq!(dashes[1].points, vec![[5., 0.], [9., 0.]]);
        assert_eq!(dashes[2].points, vec![[10., 1.], [10., 5.]]);
        // Nothing but zeros is a solid line.
        assert!(dash(&[], &[0., 0.], 0.).is_none());
    }
}
