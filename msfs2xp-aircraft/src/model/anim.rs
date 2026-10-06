//! Node animations as X-Plane OBJ8 keyframe animations.
//!
//! Geometry is written in its rest pose (the glTF nodes' own transforms). A
//! node that a clip moves is animated by its change from that rest pose. For
//! node n with rest parent world P0 = [A | b] and rest local transform
//! (t0, r0), at a clip time with local (t, r) every vertex v below n moves to
//!
//! ```text
//!   v' = Dw (v - p0) + p0 + A (t - t0),     Dw = qp (r r0^-1) qp^-1
//! ```
//!
//! where p0 is n's rest world origin and qp the rotation of A. Nested nodes
//! compose outermost first, the order X-Plane applies the transforms listed
//! in one ANIM_begin block, so a mesh's block lists its ancestors' changes
//! and then its own node's.
//!
//! A clip plays over its dataref's 0..1: MSFS stretches a clip's key times
//! over its variable's range (an aileron clip keyed -100..100 frames sits at
//! neutral in the middle of the range, and a gear clip keyed over 20 frames
//! still runs over the whole 0..100 of its variable).
//!
//! Axes: glTF (x, y, z) is X-Plane (-x, y, -z), a turn about the vertical, so
//! rotation axes turn the same way and angles keep their sign.

use std::cell::RefCell;
use std::collections::HashMap;
use std::fmt::Write as _;

use super::glb::{trs_matrix, AnimPath, Channel, Model, IDENTITY, M4};

pub type V3 = [f64; 3];
/// Quaternion, x y z w.
pub type Q = [f64; 4];

pub fn qmul(a: Q, b: Q) -> Q {
    [
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    ]
}

pub fn qconj(q: Q) -> Q {
    [-q[0], -q[1], -q[2], q[3]]
}

pub fn qnorm(q: Q) -> Q {
    let n = q.iter().map(|x| x * x).sum::<f64>().sqrt();
    if n < 1e-12 {
        [0.0, 0.0, 0.0, 1.0]
    } else {
        q.map(|x| x / n)
    }
}

fn qdot(a: Q, b: Q) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
}

fn slerp(a: Q, b: Q, u: f64) -> Q {
    let mut b = b;
    let mut d = qdot(a, b);
    if d < 0.0 {
        b = b.map(|x| -x);
        d = -d;
    }
    if d > 0.9995 {
        return qnorm([0, 1, 2, 3].map(|k| a[k] + (b[k] - a[k]) * u));
    }
    let th = d.clamp(-1.0, 1.0).acos();
    let s = th.sin();
    let (wa, wb) = (((1.0 - u) * th).sin() / s, (u * th).sin() / s);
    qnorm([0, 1, 2, 3].map(|k| wa * a[k] + wb * b[k]))
}

pub fn qrot(q: Q, v: V3) -> V3 {
    let p = qmul(qmul(q, [v[0], v[1], v[2], 0.0]), qconj(q));
    [p[0], p[1], p[2]]
}

/// Rotation angle of a unit quaternion, radians in 0..pi.
pub fn qangle(q: Q) -> f64 {
    2.0 * norm([q[0], q[1], q[2]]).atan2(q[3].abs())
}

pub fn norm(v: V3) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

pub fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// glTF axes to X-Plane object axes.
pub fn xp(v: V3) -> V3 {
    [-v[0], v[1], -v[2]]
}

fn q_xp(q: Q) -> Q {
    [-q[0], q[1], -q[2], q[3]]
}

/// The rotation part of a (possibly scaled) matrix, from its normalised columns.
pub fn quat_of(m: &M4) -> Q {
    let col = |c: usize| {
        let v = [m[c * 4], m[c * 4 + 1], m[c * 4 + 2]];
        let n = norm(v);
        if n < 1e-12 {
            v
        } else {
            v.map(|x| x / n)
        }
    };
    let cols = [col(0), col(1), col(2)];
    let r = |i: usize, j: usize| cols[j][i];
    let tr = r(0, 0) + r(1, 1) + r(2, 2);
    let q = if tr > 0.0 {
        let s = (tr + 1.0).sqrt() * 2.0;
        [(r(2, 1) - r(1, 2)) / s, (r(0, 2) - r(2, 0)) / s, (r(1, 0) - r(0, 1)) / s, 0.25 * s]
    } else if r(0, 0) > r(1, 1) && r(0, 0) > r(2, 2) {
        let s = (1.0 + r(0, 0) - r(1, 1) - r(2, 2)).sqrt() * 2.0;
        [0.25 * s, (r(0, 1) + r(1, 0)) / s, (r(0, 2) + r(2, 0)) / s, (r(2, 1) - r(1, 2)) / s]
    } else if r(1, 1) > r(2, 2) {
        let s = (1.0 + r(1, 1) - r(0, 0) - r(2, 2)).sqrt() * 2.0;
        [(r(0, 1) + r(1, 0)) / s, 0.25 * s, (r(1, 2) + r(2, 1)) / s, (r(0, 2) - r(2, 0)) / s]
    } else {
        let s = (1.0 + r(2, 2) - r(0, 0) - r(1, 1)).sqrt() * 2.0;
        [(r(0, 2) + r(2, 0)) / s, (r(1, 2) + r(2, 1)) / s, 0.25 * s, (r(1, 0) - r(0, 1)) / s]
    };
    qnorm(q)
}

/// Apply a column-major matrix to a point.
pub fn apply(m: &M4, p: V3) -> V3 {
    [
        m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12],
        m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13],
        m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14],
    ]
}

/// Inverse of an affine column-major matrix.
pub fn inverse(m: &M4) -> M4 {
    let a = [[m[0], m[4], m[8]], [m[1], m[5], m[9]], [m[2], m[6], m[10]]];
    let det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if det.abs() < 1e-15 {
        return IDENTITY;
    }
    let inv = [
        [
            (a[1][1] * a[2][2] - a[1][2] * a[2][1]) / det,
            (a[0][2] * a[2][1] - a[0][1] * a[2][2]) / det,
            (a[0][1] * a[1][2] - a[0][2] * a[1][1]) / det,
        ],
        [
            (a[1][2] * a[2][0] - a[1][0] * a[2][2]) / det,
            (a[0][0] * a[2][2] - a[0][2] * a[2][0]) / det,
            (a[0][2] * a[1][0] - a[0][0] * a[1][2]) / det,
        ],
        [
            (a[1][0] * a[2][1] - a[1][1] * a[2][0]) / det,
            (a[0][1] * a[2][0] - a[0][0] * a[2][1]) / det,
            (a[0][0] * a[1][1] - a[0][1] * a[1][0]) / det,
        ],
    ];
    let t = [m[12], m[13], m[14]];
    let it = [0, 1, 2].map(|r| -(inv[r][0] * t[0] + inv[r][1] * t[1] + inv[r][2] * t[2]));
    [
        inv[0][0], inv[1][0], inv[2][0], 0.0, inv[0][1], inv[1][1], inv[2][1], 0.0, inv[0][2], inv[1][2], inv[2][2], 0.0, it[0], it[1],
        it[2], 1.0,
    ]
}

fn span(ch: &Channel, t: f64) -> (usize, usize, f64) {
    let ts = &ch.times;
    let n = ts.len();
    if n == 0 || t <= ts[0] as f64 {
        return (0, 0, 0.0);
    }
    if t >= ts[n - 1] as f64 {
        return (n - 1, n - 1, 0.0);
    }
    let b = ts.partition_point(|&x| (x as f64) <= t).clamp(1, n - 1);
    let a = b - 1;
    if ch.step {
        return (a, a, 0.0);
    }
    let dt = ts[b] as f64 - ts[a] as f64;
    let u = if dt > 0.0 { (t - ts[a] as f64) / dt } else { 0.0 };
    (a, b, u)
}

pub fn sample_v(ch: &Channel, t: f64) -> V3 {
    if ch.values.is_empty() {
        return [0.0; 3];
    }
    let (a, b, u) = span(ch, t);
    let (va, vb) = (ch.values[a], ch.values[b]);
    [0, 1, 2].map(|k| va[k] as f64 + (vb[k] as f64 - va[k] as f64) * u)
}

pub fn sample_q(ch: &Channel, t: f64) -> Q {
    if ch.values.is_empty() {
        return [0.0, 0.0, 0.0, 1.0];
    }
    let (a, b, u) = span(ch, t);
    let q = |i: usize| qnorm(ch.values[i].map(|x| x as f64));
    slerp(q(a), q(b), u)
}

/// Which channels move which node, and each clip's time range.
pub struct ClipIndex {
    /// Per node: (clip, channel index) of every channel targeting it.
    by_node: HashMap<usize, Vec<(usize, usize)>>,
    /// Per clip: first and last key time.
    ranges: Vec<(f64, f64)>,
}

impl ClipIndex {
    pub fn new(model: &Model) -> Self {
        let mut by_node: HashMap<usize, Vec<(usize, usize)>> = HashMap::new();
        let mut ranges = Vec::with_capacity(model.clips.len());
        for (ci, clip) in model.clips.iter().enumerate() {
            let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
            for (k, ch) in clip.channels.iter().enumerate() {
                if ch.node < model.nodes.len() {
                    by_node.entry(ch.node).or_default().push((ci, k));
                }
                for &t in &ch.times {
                    lo = lo.min(t as f64);
                    hi = hi.max(t as f64);
                }
            }
            ranges.push(if lo <= hi { (lo, hi) } else { (0.0, 0.0) });
        }
        ClipIndex { by_node, ranges }
    }

    pub fn frac(&self, clip: usize, t: f64) -> f64 {
        let (a, b) = self.ranges[clip];
        if b > a {
            ((t - a) / (b - a)).clamp(0.0, 1.0)
        } else {
            0.0
        }
    }

    pub fn time(&self, clip: usize, f: f64) -> f64 {
        let (a, b) = self.ranges[clip];
        a + (b - a) * f
    }

    fn channels<'s>(&'s self, model: &'s Model, node: usize) -> impl Iterator<Item = (usize, &'s Channel)> + 's {
        self.by_node
            .get(&node)
            .into_iter()
            .flatten()
            .map(move |&(c, k)| (c, &model.clips[c].channels[k]))
    }

    /// Does this channel ever leave its node's rest value?
    pub fn moves(model: &Model, ch: &Channel) -> bool {
        let n = &model.nodes[ch.node];
        match ch.path {
            AnimPath::Translation => ch
                .values
                .iter()
                .any(|v| norm(sub([v[0] as f64, v[1] as f64, v[2] as f64], n.translation)) > 1e-6),
            AnimPath::Rotation => ch
                .values
                .iter()
                .any(|v| 1.0 - qdot(qnorm(v.map(|x| x as f64)), qnorm(n.rotation)).abs() > 1e-9),
            AnimPath::Scale => ch
                .values
                .iter()
                .any(|v| norm(sub([v[0] as f64, v[1] as f64, v[2] as f64], n.scale)) > 1e-6),
        }
    }

    /// Nodes a clip moves.
    pub fn nodes_of(&self, model: &Model, clip: usize) -> Vec<usize> {
        let mut out: Vec<usize> = model.clips[clip]
            .channels
            .iter()
            .filter(|ch| ch.node < model.nodes.len() && Self::moves(model, ch))
            .map(|ch| ch.node)
            .collect();
        out.sort_unstable();
        out.dedup();
        out
    }

    /// Distinct key fractions of a clip (all its moving channels).
    pub fn key_fractions(&self, model: &Model, clip: usize) -> Vec<f64> {
        let mut fs: Vec<f64> = model.clips[clip]
            .channels
            .iter()
            .filter(|ch| ch.node < model.nodes.len() && Self::moves(model, ch))
            .flat_map(|ch| ch.times.iter().map(|&t| self.frac(clip, t as f64)))
            .collect();
        fs.sort_by(f64::total_cmp);
        fs.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
        fs
    }

    /// The node's local transform with the given clips at the given fractions.
    fn local(&self, model: &Model, node: usize, values: &HashMap<usize, f64>) -> M4 {
        let n = &model.nodes[node];
        let (mut t, mut r, mut s) = (n.translation, n.rotation, n.scale);
        for (c, ch) in self.channels(model, node) {
            let Some(&f) = values.get(&c) else { continue };
            let time = self.time(c, f);
            match ch.path {
                AnimPath::Translation => t = sample_v(ch, time),
                AnimPath::Rotation => r = sample_q(ch, time),
                AnimPath::Scale => s = sample_v(ch, time),
            }
        }
        trs_matrix(t, r, s)
    }

    /// A node's world transform with the given clips at the given fractions
    /// (every other node at rest).
    pub fn world_at(&self, model: &Model, node: usize, values: &HashMap<usize, f64>) -> M4 {
        let mut chain = vec![node];
        while let Some(p) = model.nodes[*chain.last().unwrap()].parent {
            if chain.len() > 256 {
                break;
            }
            chain.push(p);
        }
        let mut w = IDENTITY;
        for &n in chain.iter().rev() {
            w = super::glb::mul(&w, &self.local(model, n, values));
        }
        w
    }

    /// Where a point (glTF world, rest pose) is with the given clip fractions,
    /// carried by `node`.
    pub fn carry(&self, model: &Model, node: usize, p: V3, values: &HashMap<usize, f64>) -> V3 {
        let rest = inverse(&model.nodes[node].world);
        apply(&self.world_at(model, node, values), apply(&rest, p))
    }

    /// The fraction at which the clip leaves its nodes closest to their rest
    /// pose, so a dataref starting there draws the model as built.
    pub fn rest_fraction(&self, model: &Model, clip: usize) -> f64 {
        let mut cands = self.key_fractions(model, clip);
        cands.push(0.0);
        cands.push(1.0);
        let chans: Vec<&Channel> = model.clips[clip]
            .channels
            .iter()
            .filter(|ch| ch.node < model.nodes.len() && Self::moves(model, ch))
            .collect();
        let dist = |f: f64| -> f64 {
            let t = self.time(clip, f);
            chans
                .iter()
                .map(|ch| {
                    let n = &model.nodes[ch.node];
                    match ch.path {
                        AnimPath::Translation => norm(sub(sample_v(ch, t), n.translation)),
                        AnimPath::Rotation => qangle(qmul(sample_q(ch, t), qconj(qnorm(n.rotation)))),
                        AnimPath::Scale => norm(sub(sample_v(ch, t), n.scale)),
                    }
                })
                .sum()
        };
        cands
            .into_iter()
            .map(|f| (dist(f), f))
            .min_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)))
            .map_or(0.0, |x| x.1)
    }
}

/// Compact number for OBJ text.
pub fn num(x: f64) -> String {
    if !x.is_finite() || x.abs() < 5e-7 {
        return "0".into();
    }
    let s = format!("{x:.6}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s == "-0" {
        "0".into()
    } else {
        s.to_string()
    }
}

/// Keep the keys linear interpolation cannot reproduce within `tol`.
fn reduce(f: &[f64], vals: &[Vec<f64>], tol: f64) -> Vec<usize> {
    let n = f.len();
    if n <= 2 {
        return (0..n).collect();
    }
    let mut keep = vec![0];
    let mut a = 0;
    for i in 1..n - 1 {
        let b = i + 1;
        let ok = (a + 1..b).all(|j| {
            let u = if f[b] > f[a] { (f[j] - f[a]) / (f[b] - f[a]) } else { 0.0 };
            vals[j]
                .iter()
                .zip(&vals[a])
                .zip(&vals[b])
                .all(|((v, va), vb)| (va + (vb - va) * u - v).abs() <= tol)
        });
        if !ok {
            keep.push(i);
            a = i;
        }
    }
    keep.push(n - 1);
    keep
}

/// Unwrap an angle sequence (degrees) so neighbours differ by at most 180.
fn unwrap(a: &mut [f64]) {
    for i in 1..a.len() {
        while a[i] - a[i - 1] > 180.0 {
            a[i] -= 360.0;
        }
        while a[i] - a[i - 1] < -180.0 {
            a[i] += 360.0;
        }
    }
}

fn euler_xyz(q: Q) -> [f64; 3] {
    // R = Rx(a) Ry(b) Rz(c), rows of the rotation matrix of q.
    let [x, y, z, w] = q;
    let r02 = 2.0 * (x * z + y * w);
    let r12 = 2.0 * (y * z - x * w);
    let r22 = 1.0 - 2.0 * (x * x + y * y);
    let r01 = 2.0 * (x * y - z * w);
    let r00 = 1.0 - 2.0 * (y * y + z * z);
    let b = r02.clamp(-1.0, 1.0).asin();
    let a = (-r12).atan2(r22);
    let c = (-r01).atan2(r00);
    [a.to_degrees(), b.to_degrees(), c.to_degrees()]
}

/// Writes the animation commands for geometry, by the node it hangs from.
pub struct Animator<'a> {
    model: &'a Model,
    index: &'a ClipIndex,
    /// The dataref playing each animated clip (0..1 over the clip).
    drefs: HashMap<usize, String>,
    /// Extra commands (ANIM_hide/ANIM_show) per node, for all geometry below it.
    vis: HashMap<usize, Vec<String>>,
    /// Per node, the bound clips that move it.
    moved_by: HashMap<usize, Vec<usize>>,
    cache: RefCell<HashMap<usize, String>>,
}

impl<'a> Animator<'a> {
    pub fn new(model: &'a Model, index: &'a ClipIndex, drefs: HashMap<usize, String>, vis: HashMap<usize, Vec<String>>) -> Self {
        let mut moved_by: HashMap<usize, Vec<usize>> = HashMap::new();
        for (node, list) in &index.by_node {
            for &(c, k) in list {
                if drefs.contains_key(&c) && ClipIndex::moves(model, &model.clips[c].channels[k]) {
                    let v = moved_by.entry(*node).or_default();
                    if !v.contains(&c) {
                        v.push(c);
                    }
                }
            }
        }
        for v in moved_by.values_mut() {
            v.sort_unstable();
        }
        Animator { model, index, drefs, vis, moved_by, cache: RefCell::new(HashMap::new()) }
    }

    pub fn dataref(&self, clip: usize) -> Option<&str> {
        self.drefs.get(&clip).map(String::as_str)
    }

    /// Root-first chain of nodes down to `node`.
    pub fn chain(&self, node: usize) -> Vec<usize> {
        let mut c = vec![node];
        while let Some(p) = self.model.nodes[*c.last().unwrap()].parent {
            if c.len() > 256 {
                break;
            }
            c.push(p);
        }
        c.reverse();
        c
    }

    /// Bound clips moving a node itself.
    pub fn clips_at(&self, node: usize) -> &[usize] {
        self.moved_by.get(&node).map_or(&[], Vec::as_slice)
    }

    /// Commands (inside ANIM_begin .. ANIM_end) for geometry hanging from
    /// `node`; empty when it never moves or hides.
    pub fn commands(&self, node: Option<usize>) -> String {
        let Some(node) = node.filter(|&n| n < self.model.nodes.len()) else {
            return String::new();
        };
        if let Some(s) = self.cache.borrow().get(&node) {
            return s.clone();
        }
        let mut out = String::new();
        let chain = self.chain(node);
        for n in &chain {
            for line in self.vis.get(n).into_iter().flatten() {
                out.push_str(line);
                out.push('\n');
            }
        }
        for n in chain {
            for &c in self.clips_at(n) {
                self.block(n, c, &mut out);
            }
        }
        self.cache.borrow_mut().insert(node, out.clone());
        out
    }

    fn block(&self, node: usize, clip: usize, out: &mut String) {
        let m = self.model;
        let nd = &m.nodes[node];
        let dr = &self.drefs[&clip];
        let chans: Vec<&Channel> = self
            .index
            .channels(m, node)
            .filter(|(c, ch)| *c == clip && ClipIndex::moves(m, ch))
            .map(|(_, ch)| ch)
            .collect();
        let find = |p: AnimPath| chans.iter().copied().find(|c| c.path == p);
        let (tch, rch, sch) = (find(AnimPath::Translation), find(AnimPath::Rotation), find(AnimPath::Scale));
        let mut times: Vec<f64> = chans.iter().flat_map(|c| c.times.iter().map(|&t| t as f64)).collect();
        times.sort_by(f64::total_cmp);
        times.dedup_by(|a, b| (*a - *b).abs() < 1e-7);
        if times.is_empty() {
            return;
        }
        let parent = nd.parent.map(|p| m.nodes[p].world).unwrap_or(IDENTITY);
        let qp = quat_of(&parent);
        let lin = |v: V3| {
            [
                parent[0] * v[0] + parent[4] * v[1] + parent[8] * v[2],
                parent[1] * v[0] + parent[5] * v[1] + parent[9] * v[2],
                parent[2] * v[0] + parent[6] * v[1] + parent[10] * v[2],
            ]
        };
        let (t0, r0) = (nd.translation, qnorm(nd.rotation));
        let pivot = xp([nd.world[12], nd.world[13], nd.world[14]]);
        let mut fr: Vec<f64> = Vec::new();
        let mut trans: Vec<V3> = Vec::new();
        let mut rots: Vec<Q> = Vec::new();
        let mut hidden: Vec<bool> = Vec::new();
        for &t in &times {
            let f = self.index.frac(clip, t);
            if fr.last().is_some_and(|&l| (f - l).abs() < 1e-7) {
                continue;
            }
            let tv = tch.map(|c| sample_v(c, t)).unwrap_or(t0);
            let rv = rch.map(|c| sample_q(c, t)).unwrap_or(r0);
            fr.push(f);
            trans.push(xp(lin(sub(tv, t0))));
            rots.push(q_xp(qnorm(qmul(qmul(qp, qmul(rv, qconj(r0))), qconj(qp)))));
            hidden.push(sch.is_some_and(|c| sample_v(c, t).iter().any(|s| s.abs() < 0.01)));
        }

        // Scale to nothing hides the part.
        let mut i = 0;
        while i < hidden.len() {
            if hidden[i] {
                let j = (i..hidden.len()).take_while(|&k| hidden[k]).last().unwrap_or(i);
                let lo = if i == 0 { -1e6 } else { fr[i] - 1e-4 };
                let hi = if j == hidden.len() - 1 { 1e6 } else { fr[j] + 1e-4 };
                let _ = writeln!(out, "ANIM_hide {} {} {dr}", num(lo), num(hi));
                i = j + 1;
            } else {
                i += 1;
            }
        }

        if trans.iter().any(|d| norm(*d) > 1e-5) {
            let vals: Vec<Vec<f64>> = trans.iter().map(|d| d.to_vec()).collect();
            let keep = reduce(&fr, &vals, 0.0005);
            if keep.len() < 2 {
                let d = trans[keep[0]];
                let _ = writeln!(out, "ANIM_trans {0} {1} {2} {0} {1} {2}", num(d[0]), num(d[1]), num(d[2]));
            } else {
                let _ = writeln!(out, "ANIM_trans_begin {dr}");
                for k in keep {
                    let d = trans[k];
                    let _ = writeln!(out, "ANIM_trans_key {} {} {} {}", num(fr[k]), num(d[0]), num(d[1]), num(d[2]));
                }
                out.push_str("ANIM_trans_end\n");
            }
        }

        let big = rots.iter().copied().fold(0.0f64, |a, q| a.max(qangle(q)));
        if big < 1e-5 {
            return;
        }
        let _ = writeln!(out, "ANIM_trans {0} {1} {2} {0} {1} {2}", num(pivot[0]), num(pivot[1]), num(pivot[2]));
        let refq = rots.iter().copied().max_by(|a, b| qangle(*a).total_cmp(&qangle(*b))).unwrap();
        let v = [refq[0], refq[1], refq[2]];
        let s = if refq[3] < 0.0 { -1.0 } else { 1.0 };
        let axis = v.map(|x| x * s / norm(v));
        let single = rots.iter().all(|q| {
            let v = [q[0], q[1], q[2]];
            let along = dot(v, axis);
            norm(sub(v, axis.map(|a| a * along))) < 2e-3
        });
        let emit = |axis: V3, mut deg: Vec<f64>, out: &mut String| {
            unwrap(&mut deg);
            if deg.iter().all(|a| a.abs() < 1e-4) {
                return;
            }
            let vals: Vec<Vec<f64>> = deg.iter().map(|a| vec![*a]).collect();
            let keep = reduce(&fr, &vals, 0.02);
            if keep.len() < 2 {
                let a = num(deg[keep[0]]);
                let _ = writeln!(out, "ANIM_rotate {} {} {} {a} {a}", num(axis[0]), num(axis[1]), num(axis[2]));
            } else {
                let _ = writeln!(out, "ANIM_rotate_begin {} {} {} {dr}", num(axis[0]), num(axis[1]), num(axis[2]));
                for k in keep {
                    let _ = writeln!(out, "ANIM_rotate_key {} {}", num(fr[k]), num(deg[k]));
                }
                out.push_str("ANIM_rotate_end\n");
            }
        };
        if single {
            let deg = rots.iter().map(|q| (2.0 * dot([q[0], q[1], q[2]], axis).atan2(q[3])).to_degrees()).collect();
            emit(axis, deg, out);
        } else {
            let e: Vec<[f64; 3]> = rots.iter().map(|q| euler_xyz(*q)).collect();
            for (k, ax) in [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]].into_iter().enumerate() {
                emit(ax, e.iter().map(|a| a[k]).collect(), out);
            }
        }
        let _ = writeln!(out, "ANIM_trans {0} {1} {2} {0} {1} {2}", num(-pivot[0]), num(-pivot[1]), num(-pivot[2]));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::glb::{Clip, Node};

    fn node(name: &str, parent: Option<usize>, t: V3, r: Q) -> Node {
        Node {
            name: name.into(),
            parent,
            translation: t,
            rotation: r,
            scale: [1.0; 3],
            world: trs_matrix(t, r, [1.0; 3]),
        }
    }

    fn rot_y(deg: f64) -> Q {
        let h = deg.to_radians() / 2.0;
        [0.0, h.sin(), 0.0, h.cos()]
    }

    #[test]
    fn a_hinge_turns_about_its_rest_pivot() {
        // A door hinged at x = 2 swings 90 degrees about +Y over the clip.
        let mut model = Model::default();
        model.nodes.push(node("hinge", None, [2.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]));
        model.clips.push(Clip {
            name: "door".into(),
            channels: vec![Channel {
                node: 0,
                path: AnimPath::Rotation,
                times: vec![0.0, 1.0],
                values: vec![[0.0, 0.0, 0.0, 1.0], rot_y(90.0).map(|x| x as f32)],
                step: false,
            }],
        });
        let index = ClipIndex::new(&model);
        let drefs = HashMap::from([(0usize, "fbw/anim/door".to_string())]);
        let a = Animator::new(&model, &index, drefs, HashMap::new());
        let s = a.commands(Some(0));
        // Pivot in X-Plane axes is (-2, 0, 0); the axis stays +Y.
        assert!(s.contains("ANIM_trans -2 0 0 -2 0 0\n"), "{s}");
        assert!(s.contains("ANIM_rotate_begin 0 1 0 fbw/anim/door\nANIM_rotate_key 0 0\nANIM_rotate_key 1 90\n"), "{s}");
        assert!(s.ends_with("ANIM_trans 2 0 0 2 0 0\n"), "{s}");
        // A point 1 m out from the hinge (+X) ends up at -Z after +90 about Y.
        let p = index.carry(&model, 0, [3.0, 0.0, 0.0], &HashMap::from([(0usize, 1.0)]));
        assert!((p[0] - 2.0).abs() < 1e-9 && (p[2] + 1.0).abs() < 1e-9, "{p:?}");
    }

    #[test]
    fn a_node_at_rest_in_every_key_is_left_alone_and_the_rest_fraction_found() {
        let mut model = Model::default();
        model.nodes.push(node("a", None, [0.0; 3], [0.0, 0.0, 0.0, 1.0]));
        model.nodes.push(node("b", None, [0.0; 3], rot_y(10.0)));
        model.clips.push(Clip {
            name: "c".into(),
            channels: vec![
                Channel { node: 0, path: AnimPath::Translation, times: vec![0.0, 2.0], values: vec![[0.0; 4]; 2], step: false },
                Channel {
                    node: 1,
                    path: AnimPath::Rotation,
                    times: vec![-1.0, 0.0, 1.0],
                    values: vec![rot_y(0.0).map(|x| x as f32), rot_y(10.0).map(|x| x as f32), rot_y(20.0).map(|x| x as f32)],
                    step: false,
                },
            ],
        });
        let index = ClipIndex::new(&model);
        let a = Animator::new(&model, &index, HashMap::from([(0usize, "d".to_string())]), HashMap::new());
        assert!(a.commands(Some(0)).is_empty());
        assert_eq!(index.nodes_of(&model, 0), vec![1]);
        // Clip runs -1..2 s; node b is at rest at t = 0, a third of the way.
        assert!((index.rest_fraction(&model, 0) - 1.0 / 3.0).abs() < 1e-6);
    }

    #[test]
    fn numbers_are_compact() {
        assert_eq!(num(1.5), "1.5");
        assert_eq!(num(-0.0000001), "0");
        assert_eq!(num(2.0), "2");
        assert_eq!(num(-3.25), "-3.25");
    }
}
