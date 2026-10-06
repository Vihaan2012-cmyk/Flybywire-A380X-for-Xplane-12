#!/usr/bin/env python3
"""Generate the CL650-style exterior walkaround OBJ8 object for the FBW A380X
X-Plane port: pitot/AoA/static covers, engine inlet/exhaust covers, gear pins
and chocks, positioned from the real MSFS glTF geometry.

Usage:
    python make_walkaround.py --gltf <path to a380_exterior.gltf> --out <output dir>
    python make_walkaround.py --gltf ... --out ... --acf <path to .acf to append an _obja entry to>

See README.md in this folder for the sourcing of every item's position and
size, and for the click-test finding this script's --acf mode depends on.

Everything this script reads is under D:/Microsoft Flight Simulator 2020 or
D:/Steam Games (read-only references). Everything it writes goes under
E:/fbw-int/msfs2xp-aircraft/tools/walkaround/ (this file, README.md) or the
--out directory (intended: D:/A380/fbw-build/walkaround-stage/). It never
touches the plugin, the Rust converter, or Converter itself.
"""
from __future__ import annotations

import argparse
import json
import math
import struct
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

# ---------------------------------------------------------------------------
# glTF reading (mirrors the conventions in src/model/glb.rs, reimplemented in
# plain Python/numpy here since this tool must not touch the Rust source).
# ---------------------------------------------------------------------------

# glTF componentType -> (numpy dtype, byte size)
NPDTYPE = {5120: "i1", 5121: "u1", 5122: "i2", 5123: "u2", 5125: "u4", 5126: "f4"}
CSIZE = {5120: 1, 5121: 1, 5122: 2, 5123: 2, 5125: 4, 5126: 4}
NCOMP = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}


def load_gltf(gltf_path: Path):
    gltf_path = Path(gltf_path)
    with open(gltf_path, "rb") as f:
        j = json.load(f)
    bin_name = j["buffers"][0]["uri"]
    bin_path = gltf_path.parent / bin_name
    with open(bin_path, "rb") as f:
        buf = f.read()
    return j, buf


def accessor_np(j, buf, idx) -> np.ndarray:
    """Decodes a POSITION-like accessor to float64 (N, ncomp). Handles both
    tightly packed and interleaved bufferViews. Does not need the MSFS
    half-float-in-a-SHORT quirk noted for TEXCOORD: positions are plain
    componentType 5126 (FLOAT) in this asset, verified below at load time."""
    a = j["accessors"][idx]
    bv = j["bufferViews"][a["bufferView"]]
    csize = CSIZE[a["componentType"]]
    n = NCOMP[a["type"]]
    count = a["count"]
    tight = csize * n
    stride = bv.get("byteStride", tight)
    base = bv.get("byteOffset", 0) + a.get("byteOffset", 0)
    dt = np.dtype(NPDTYPE[a["componentType"]])
    if stride == tight:
        raw = np.frombuffer(buf, dtype=dt, count=count * n, offset=base)
        out = raw.reshape(count, n).astype(np.float64)
    else:
        u8 = np.frombuffer(buf, dtype=np.uint8)
        out = np.zeros((count, n), dtype=np.float64)
        for i in range(count):
            off = base + i * stride
            out[i] = np.frombuffer(u8, dtype=dt, count=n, offset=off)
    if a["componentType"] in (5121, 5123) and a.get("normalized"):
        out = out / (255.0 if a["componentType"] == 5121 else 65535.0)
    return out


def _quat_rot(q):
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def node_local_matrix(node) -> np.ndarray:
    if "matrix" in node:
        return np.array(node["matrix"], dtype=np.float64).reshape(4, 4).T
    t = node.get("translation", [0, 0, 0])
    q = node.get("rotation", [0, 0, 0, 1])
    s = node.get("scale", [1, 1, 1])
    r = _quat_rot(q)
    m = np.eye(4)
    m[:3, 0] = r[:, 0] * s[0]
    m[:3, 1] = r[:, 1] * s[1]
    m[:3, 2] = r[:, 2] * s[2]
    m[:3, 3] = t
    return m


def build_world_matrices(j) -> list[np.ndarray]:
    nodes = j["nodes"]
    world = [None] * len(nodes)

    def visit(idx, parent_world):
        m = parent_world @ node_local_matrix(nodes[idx])
        world[idx] = m
        for c in nodes[idx].get("children", []):
            visit(c, m)

    for scene in j.get("scenes", []):
        for root in scene["nodes"]:
            visit(root, np.eye(4))
    for i in range(len(nodes)):
        if world[i] is None:
            world[i] = np.eye(4)
    return world


def transform_points(m, pts) -> np.ndarray:
    ones = np.ones((pts.shape[0], 1))
    h = np.hstack([pts, ones])
    return (m @ h.T).T[:, :3]


def transform_dir(m, d) -> np.ndarray:
    """Direction vector through the upper 3x3 (no translation)."""
    return (m[:3, :3] @ np.asarray(d, dtype=np.float64))


def mesh_world_vertices(j, buf, world, node_idx) -> np.ndarray:
    node = j["nodes"][node_idx]
    if "mesh" not in node:
        return np.zeros((0, 3))
    mesh = j["meshes"][node["mesh"]]
    m = world[node_idx]
    allv = []
    for prim in mesh["primitives"]:
        pidx = prim["attributes"].get("POSITION")
        if pidx is None:
            continue
        v = accessor_np(j, buf, pidx)
        allv.append(transform_points(m, v))
    return np.vstack(allv) if allv else np.zeros((0, 3))


def node_by_name(names, name):
    try:
        return names.index(name)
    except ValueError:
        return None


def material_index(j, name):
    for i, m in enumerate(j["materials"]):
        if m.get("name") == name:
            return i
    return None


def collect_material_vertices(j, buf, world, mat_indices, mesh_to_nodes):
    """All world-space vertices whose primitive uses one of `mat_indices`,
    tagged with (material index, node index) so callers can re-group them."""
    pts, mats, node_ids = [], [], []
    for mesh_idx, mesh in enumerate(j["meshes"]):
        for prim in mesh["primitives"]:
            mat = prim.get("material")
            if mat not in mat_indices:
                continue
            pidx = prim["attributes"].get("POSITION")
            if pidx is None:
                continue
            v_local = accessor_np(j, buf, pidx)
            for node_idx in mesh_to_nodes.get(mesh_idx, []):
                v = transform_points(world[node_idx], v_local)
                pts.append(v)
                mats.append(np.full(len(v), mat))
                node_ids.append(np.full(len(v), node_idx))
    if not pts:
        return np.zeros((0, 3)), np.zeros((0,), int), np.zeros((0,), int)
    return np.vstack(pts), np.concatenate(mats), np.concatenate(node_ids)


def principal_axis(v: np.ndarray):
    """Centroid, unit elongation axis (largest-variance direction), the
    extent of the cluster along that axis, and its typical radius
    perpendicular to it -- used to size and orient small probe geometry
    directly from its own mesh vertices instead of guessing an axis
    convention."""
    c = v.mean(axis=0)
    d = v - c
    cov = d.T @ d / max(len(v), 1)
    vals, vecs = np.linalg.eigh(cov)
    axis = vecs[:, -1]
    proj = d @ axis
    extent = proj.max() - proj.min()
    perp = d - np.outer(proj, axis)
    radius = np.linalg.norm(perp, axis=1).mean()
    return c, axis, extent, radius


def ring_fit(v: np.ndarray):
    """Centre and mean radius of a point cluster taken to lie on a ring in
    the XY plane (used for the engine inlet/exhaust rings, whose axis is the
    aircraft's longitudinal axis per ThrustAnglesPitchHeading = 0,0 in
    engines.cfg)."""
    c = v.mean(axis=0)
    r = np.linalg.norm(v[:, :2] - c[:2], axis=1).mean()
    return c, r


def angular_profile(pts: np.ndarray, center_xy, n_bins: int, agg: str = "mean"):
    """Buckets `pts` into `n_bins` angle bins around `center_xy` (in the XY
    plane -- the engine axis is Z, per ring_fit) and returns the radius per
    bin (mean, or the max with `agg="max"`), circularly interpolating any
    empty bins. Used to sample the real (non-circular) lip/cowl cross-section
    instead of assuming a perfect circle. `agg="max"` picks the outer-skin
    envelope out of a band that also contains smaller-radius points (e.g. the
    inner duct wall near an inlet lip), instead of averaging the two
    together. Returns None if there is no usable data at all."""
    if len(pts) == 0:
        return None
    ang = np.arctan2(pts[:, 1] - center_xy[1], pts[:, 0] - center_xy[0])
    r = np.hypot(pts[:, 0] - center_xy[0], pts[:, 1] - center_xy[1])
    bin_idx = np.clip(((ang + math.pi) / (2 * math.pi) * n_bins).astype(int), 0, n_bins - 1)
    prof = np.full(n_bins, np.nan)
    for b in range(n_bins):
        sel = bin_idx == b
        if np.any(sel):
            prof[b] = r[sel].max() if agg == "max" else r[sel].mean()
    nanidx = np.isnan(prof)
    if nanidx.all():
        return None
    if nanidx.any():
        idxs = np.arange(n_bins)
        good = ~nanidx
        xs, ys = idxs[good], prof[good]
        xs_ext = np.concatenate([xs - n_bins, xs, xs + n_bins])
        ys_ext = np.concatenate([ys, ys, ys])
        prof[nanidx] = np.interp(idxs[nanidx], xs_ext, ys_ext)
    return prof


def min_clearance(cover_pts: np.ndarray, ref_pts: np.ndarray, sample_ref=4000, seed=0):
    """Minimum distance from any cover vertex to the real nacelle/core
    surface it is fitted over -- a direct clipping check, not a guess."""
    if len(cover_pts) == 0 or len(ref_pts) == 0:
        return float("nan")
    if len(ref_pts) > sample_ref:
        rng = np.random.default_rng(seed)
        ref_pts = ref_pts[rng.choice(len(ref_pts), sample_ref, replace=False)]
    mind = np.inf
    for i in range(0, len(cover_pts), 400):
        chunk = cover_pts[i : i + 400]
        d = np.linalg.norm(chunk[:, None, :] - ref_pts[None, :, :], axis=2)
        mind = min(mind, float(d.min()))
    return mind


def build_wrap_cover(engine_pts: np.ndarray, lip_center_xy, lip_z: float, skirt_dir: float,
                      n_bins: int = 40, dome_bulge: float = 0.05, dome_standoff: float = 0.03,
                      skirt_depth: float = 0.25, clearance: float = 0.016,
                      band_width: float = 0.035, band_bump: float = 0.014, n_wrap_rings: int = 4):
    """Builds one fitted fabric cover -- dome cap, wrap-over, snug skirt and
    strap band -- as a single continuous triangle mesh, in glTF/world space,
    with local (u,v) in [0,1] (mapped into an atlas region by the caller).
    `skirt_dir` is the direction (in Z) the skirt runs away from the lip: -1
    for an inlet cover (skirt runs aft over the outer cowl), +1 for an
    exhaust plug (skirt runs forward over the nozzle/core). Returns (verts,
    idx, confident, cover_pts, attach_pts) where `confident` says whether a
    real cowl cross-section was sampled for the skirt's far edge (vs. a
    lip-profile fallback), and `cover_pts` is every vertex position (for the
    caller's clearance check)."""
    cx, cy = float(lip_center_xy[0]), float(lip_center_xy[1])
    cap_out_dir = -skirt_dir  # direction the cap bulges, away from the engine

    # The true leading-edge curve, per angle: the most-exposed point in the
    # cap_out_dir sense (not assumed flat at lip_z -- a scarfed/angled lip
    # edge is common), sampled from a band wide enough to cover the whole
    # rounded lip cross-section either side of it.
    edge_band = engine_pts[np.abs(engine_pts[:, 2] - lip_z) < skirt_depth * 0.6]
    if len(edge_band) < n_bins // 2:
        edge_band = engine_pts
    ang = np.arctan2(edge_band[:, 1] - cy, edge_band[:, 0] - cx)
    bin_idx = np.clip(((ang + math.pi) / (2 * math.pi) * n_bins).astype(int), 0, n_bins - 1)
    proj = cap_out_dir * edge_band[:, 2]
    edge_proj = np.full(n_bins, np.nan)
    for b in range(n_bins):
        sel = bin_idx == b
        if np.any(sel):
            edge_proj[b] = proj[sel].max()
    nanidx = np.isnan(edge_proj)
    if nanidx.all():
        edge_proj[:] = cap_out_dir * lip_z
    elif nanidx.any():
        idxs = np.arange(n_bins)
        good = ~nanidx
        xs, ys = idxs[good], edge_proj[good]
        xs_ext = np.concatenate([xs - n_bins, xs, xs + n_bins])
        ys_ext = np.concatenate([ys, ys, ys])
        edge_proj[nanidx] = np.interp(idxs[nanidx], xs_ext, ys_ext)
    edge_z = cap_out_dir * edge_proj  # real z of the leading edge, per angle

    # Outer lip radius per angle: the MAX radius in the edge band, so the
    # outer cowl skin is picked over the smaller-radius inner duct wall the
    # same band also contains close to the tip (a mean of the two is what
    # recessed the cap inside the lip before this fix).
    outer_prof = angular_profile(edge_band, (cx, cy), n_bins, agg="max")
    if outer_prof is None:
        outer_prof = np.full(n_bins, float(np.hypot(engine_pts[:, 0] - cx, engine_pts[:, 1] - cy).max()))

    cowl_z = lip_z + skirt_dir * skirt_depth
    cowl_band = engine_pts[np.abs(engine_pts[:, 2] - cowl_z) < 0.03]
    confident = len(cowl_band) >= n_bins // 2
    cowl_prof = angular_profile(cowl_band, (cx, cy), n_bins, agg="max") if confident else None
    if cowl_prof is None:
        cowl_prof = outer_prof * 0.97
        confident = False

    thetas = np.array([2 * math.pi * b / n_bins - math.pi for b in range(n_bins)])
    dirs_xy = np.stack([np.cos(thetas), np.sin(thetas)], axis=1)

    verts = []  # (pos, uv_local)
    idx = []

    def add_ring(ring_pts, uv_v):
        base = len(verts)
        for b, p in enumerate(ring_pts):
            verts.append((p, (b / n_bins, uv_v)))
        return base

    def quad_strip(a_base, b_base):
        for b in range(n_bins):
            b2 = (b + 1) % n_bins
            a0, a1, b0, b1 = a_base + b, a_base + b2, b_base + b, b_base + b2
            idx.extend([a0, a1, b1, a0, b1, b0])

    # Dome cap: 3 concentric rings from the centre out to the rim. The rim
    # sits at the full outer-lip radius (+ clearance), standing
    # `dome_standoff` proud of the real leading-edge curve -- a few cm in
    # front of the lip's most-forward point, spanning the outer lip
    # diameter, not recessed inside it. Extra sag on the lower half (gravity,
    # -Y) reads as slack fabric rather than a rigid shell.
    rim_r = outer_prof + clearance
    rim_z = edge_z + cap_out_dir * dome_standoff
    # The apex is a synthetic point (no mesh vertex sits at r=0), so it must
    # clear both the rim's own most-exposed angle bin and `lip_z` itself
    # (the caller's true extreme point) by `dome_bulge` -- a plain mean of
    # the rim left the apex short of a pointed/rounded tip whose true apex
    # sits further out than any rim sample (an exhaust centre-body cone).
    apex_proj = max(float(np.max(cap_out_dir * rim_z)), cap_out_dir * lip_z) + dome_bulge
    apex_z = cap_out_dir * apex_proj
    n_cap_rings = 3
    cap_bases = []
    for ring_i in range(1, n_cap_rings + 1):
        frac = ring_i / n_cap_rings
        ring_pts = []
        for b in range(n_bins):
            r = rim_r[b] * frac
            sag = 0.4 * dome_bulge * max(0.0, -dirs_xy[b, 1]) * frac
            z = apex_z + (rim_z[b] - apex_z) * frac + cap_out_dir * sag
            ring_pts.append(np.array([cx + dirs_xy[b, 0] * r, cy + dirs_xy[b, 1] * r, z]))
        cap_bases.append(add_ring(ring_pts, 0.03 + 0.17 * frac))
    c_idx = len(verts)
    verts.append((np.array([cx, cy, apex_z]), (0.5, 0.0)))
    for b in range(n_bins):
        b2 = (b + 1) % n_bins
        idx.extend([c_idx, cap_bases[0] + b, cap_bases[0] + b2])
    for i in range(len(cap_bases) - 1):
        quad_strip(cap_bases[i], cap_bases[i + 1])

    # Wrap-over + skirt: rings rolling continuously from the cap's rim
    # (forward, at the outer-lip radius) down across the leading edge and
    # back along the outer cowl to `cowl_z`. Each ring is sampled from the
    # nacelle's own geometry at that depth (angular_profile, max radius per
    # angle), not a straight lerp between the lip and cowl profiles -- a
    # real lip curves non-linearly, and a lerp clipped inside it partway
    # back (the exposed grey band seen before this fix).
    skirt_bases = [cap_bases[-1]]
    slice_half_width = max(0.02, skirt_depth / n_wrap_rings * 0.6)
    for ring_i in range(1, n_wrap_rings + 1):
        frac = ring_i / n_wrap_rings
        z_c = lip_z + skirt_dir * skirt_depth * frac
        band = engine_pts[np.abs(engine_pts[:, 2] - z_c) < slice_half_width]
        prof = angular_profile(band, (cx, cy), n_bins, agg="max") if len(band) >= n_bins // 2 else None
        if prof is None:
            prof = outer_prof * (1 - frac) + cowl_prof * frac
        ring_pts = []
        for b in range(n_bins):
            r = prof[b] + clearance
            ring_pts.append(np.array([cx + dirs_xy[b, 0] * r, cy + dirs_xy[b, 1] * r, z_c]))
        skirt_bases.append(add_ring(ring_pts, 0.22 + 0.5 * frac))
    for i in range(len(skirt_bases) - 1):
        quad_strip(skirt_bases[i], skirt_bases[i + 1])

    # Strap/drawstring band at the skirt's far edge: 3 rings bumped a little
    # further out than the cowl, tapering back in -- reads as a cinched band.
    band_rings = []
    for frac, extra, uv_v in ((0.0, band_bump, 0.68), (0.5, band_bump * 1.3, 0.78), (1.0, band_bump * 0.4, 0.88)):
        z = cowl_z + skirt_dir * band_width * frac
        ring_pts = []
        for b in range(n_bins):
            r = cowl_prof[b] + clearance + extra
            ring_pts.append(np.array([cx + dirs_xy[b, 0] * r, cy + dirs_xy[b, 1] * r, z]))
        band_rings.append(add_ring(ring_pts, uv_v))
    chain = [skirt_bases[-1]] + band_rings
    for i in range(len(chain) - 1):
        quad_strip(chain[i], chain[i + 1])

    # Face-averaged per-vertex normals (flat-ish shading is fine for fabric).
    positions = [v[0] for v in verts]
    normals = [np.zeros(3) for _ in verts]
    for t in range(0, len(idx), 3):
        i0, i1, i2 = idx[t], idx[t + 1], idx[t + 2]
        p0, p1, p2 = positions[i0], positions[i1], positions[i2]
        fn = np.cross(p1 - p0, p2 - p0)
        normals[i0] = normals[i0] + fn
        normals[i1] = normals[i1] + fn
        normals[i2] = normals[i2] + fn
    out_verts = []
    for (p, uv), n in zip(verts, normals):
        nl = np.linalg.norm(n)
        nn = n / nl if nl > 1e-9 else np.array([0.0, 0.0, 1.0])
        out_verts.append((p, nn, uv))

    cover_pts = np.array(positions)
    # attach points for streamers: 3 points along the band's lower arc
    # (around theta = -90 deg, i.e. -Y / down), in world space.
    lower_bin = int(np.argmin(np.abs(thetas + math.pi / 2)))
    attach_bins = [(lower_bin - 4) % n_bins, lower_bin, (lower_bin + 4) % n_bins]
    attach_pts = []
    for b in attach_bins:
        r = cowl_prof[b] + clearance + band_bump
        z = cowl_z + skirt_dir * band_width * 0.5
        attach_pts.append(np.array([cx + dirs_xy[b, 0] * r, cy + dirs_xy[b, 1] * r, z]))

    return out_verts, idx, confident, cover_pts, attach_pts


def gen_flat_label(center, out_dir, half_w, half_h, uv):
    n = out_dir / (np.linalg.norm(out_dir) + 1e-12)
    r, u_ = _basis_from_axis(n)
    u0, v0, u1, v1 = uv
    p00 = center - r * half_w - u_ * half_h
    p10 = center + r * half_w - u_ * half_h
    p11 = center + r * half_w + u_ * half_h
    p01 = center - r * half_w + u_ * half_h
    verts = [(p00, n, (u0, v1)), (p10, n, (u1, v1)), (p11, n, (u1, v0)), (p01, n, (u0, v0))]
    idx = [0, 1, 2, 0, 2, 3]
    return verts, idx


class UnionFind:
    def __init__(self, n):
        self.p = list(range(n))

    def find(self, x):
        while self.p[x] != x:
            x = self.p[x]
        return x

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.p[ra] = rb


# ---------------------------------------------------------------------------
# Extraction: everything below reads positions/sizes only from the loaded
# glTF (plus flight_model.cfg / engines.cfg for cross-checks noted in the
# report), never invented.
# ---------------------------------------------------------------------------


class Item:
    """One walkaround item: its OBJ8 dataref id, the pieces of geometry that
    make it up (already positioned in world/glTF space -- the OBJ-frame flip
    happens once, at write time), and where its position came from."""

    def __init__(self, item_id, tooltip, pieces, source, feature, distance, ground_gear=None):
        self.id = item_id
        self.tooltip = tooltip
        self.pieces = pieces  # list of dict: kind + params, all in gltf/world space
        self.source = source
        self.feature = feature
        self.distance = distance
        # X-Plane gear index whose tyre deflection this item rides up with
        # (chocks sit on the ground, and on the ground a strut compresses:
        # the ground rises toward the airframe by exactly that deflection).
        self.ground_gear = ground_gear


def extract_probes(j, buf, world, names):
    items = []
    notes = []

    def mesh_of(name):
        idx = node_by_name(names, name)
        if idx is None:
            return None, None
        return idx, mesh_world_vertices(j, buf, world, idx)

    # Multi-function probes MFP1/2/3: on the real A380 each MFP combines the
    # pitot sensing tube with a small AoA vane on one flank, replacing the
    # separate pitot heads + AoA vanes of earlier Airbus types. The asset
    # names the two parts of each unit as "<probe>_BASE" (the tube/body --
    # used for pitot_cover_N) and "<probe>_FIN" (the flat vane -- used for
    # aoa_cover_N); "_SCREWS" are mounting hardware, not covered.
    for n in (1, 2, 3):
        base_idx, base_v = mesh_of(f"MFP{n}_BASE")
        fin_idx, fin_v = mesh_of(f"MFP{n}_FIN")
        if base_idx is None or len(base_v) == 0:
            notes.append(f"MFP{n}_BASE not found -- pitot_cover_{n} left out")
            continue
        c, axis, extent, radius = principal_axis(base_v)
        length = max(extent, 0.12)
        radius = max(radius, 0.02)
        pieces = [{"kind": "sleeve", "center": c, "axis": axis, "length": length, "radius": radius * 1.15}]
        pieces.append(_streamer_piece(c - axis * (length * 0.5)))
        items.append(
            Item(
                f"pitot_cover_{n}",
                f"Remove pitot cover {n}",
                pieces,
                f"node MFP{n}_BASE (material A380X_ACCESSORIES), PCA of its own vertices",
                f"multi-function probe {n} body (pitot sensing tube)",
                0.0,
            )
        )
        if fin_idx is not None and len(fin_v) > 0:
            fc, faxis, fextent, fradius = principal_axis(fin_v)
            pieces_f = [{"kind": "pad", "center": fc, "axis": faxis, "length": max(fextent, 0.05), "width": max(fradius * 2, 0.04), "thick": 0.015}]
            pieces_f.append(_streamer_piece(fc - np.array([0, 0.06, 0])))
            items.append(
                Item(
                    f"aoa_cover_{n}",
                    f"Remove AoA vane cover {n}",
                    pieces_f,
                    f"node MFP{n}_FIN (material A380X_ACCESSORIES), PCA of its own vertices",
                    f"multi-function probe {n} vane (AoA sensing fin, same physical unit as pitot_cover_{n})",
                    float(np.linalg.norm(fc - c)),
                )
            )
        else:
            notes.append(f"MFP{n}_FIN not found -- aoa_cover_{n} left out")

    # Standby pitot: found (STBY_PITOT), but the interface has no 4th pitot
    # slot -- the 3 MFPs already fill pitot_cover_1..3. Reported, not placed.
    stby_idx, stby_v = mesh_of("STBY_PITOT")
    if stby_idx is not None and len(stby_v):
        c = stby_v.mean(axis=0)
        notes.append(
            f"STBY_PITOT found at {np.round(c, 2).tolist()} (glTF frame) -- a real 4th pitot probe "
            "(feeds the ISIS), but the fixed interface only defines pitot_cover_1..3 (matched to the "
            "3 MFPs). Left uncovered; no interface slot to put it on."
        )

    # Static ports: STATIC_PORT and STBY_STATIC each bundle a mirrored L/R
    # pair in one mesh (split here by the sign of X). All four pads go under
    # the single `static_covers` item, matching the one dataref the interface
    # gives for the whole static-port set.
    static_pieces = []
    for name in ("STATIC_PORT", "STBY_STATIC"):
        idx, v = mesh_of(name)
        if idx is None or len(v) == 0:
            notes.append(f"{name} not found -- its pads left out of static_covers")
            continue
        for side_sign, side in ((1, "left"), (-1, "right")):
            sel = v[np.sign(v[:, 0]) == side_sign] if side_sign > 0 else v[v[:, 0] <= 0]
            if len(sel) == 0:
                continue
            c, axis, extent, radius = principal_axis(sel)
            static_pieces.append({"kind": "pad", "center": c, "axis": axis, "length": max(extent, 0.1), "width": max(radius * 1.6, 0.08), "thick": 0.01})
            static_pieces.append(_streamer_piece(c - np.array([0, 0.05, 0]), length=0.3))
    if static_pieces:
        items.append(
            Item(
                "static_covers",
                "Remove static port covers",
                static_pieces,
                "nodes STATIC_PORT + STBY_STATIC (material A380X_ACCESSORIES), split left/right by sign(x)",
                "primary + standby static ports (both sides)",
                0.0,
            )
        )
    else:
        notes.append("no static port geometry found at all -- static_covers omitted")

    return items, notes


def _streamer_piece(attach_point, length=0.5, width=0.06):
    return {"kind": "streamer", "attach": attach_point, "length": length, "width": width}


def extract_engines(j, buf, world, names, mesh_to_nodes):
    items = []
    notes = []
    mats = {
        "ENG_CORES_LH": material_index(j, "A380_EXTERIOR_ENG_CORES_LH"),
        "ENG_LH": material_index(j, "A380_EXTERIOR_ENG_LH"),
        "ENG_CORES_RH": material_index(j, "A380_EXTERIOR_ENG_CORES_RH"),
        "ENG_RH": material_index(j, "A380_EXTERIOR_ENG_RH"),
        "REVERSER_CORE": material_index(j, "A380_EXTERIOR_REVERSER_CORE"),
    }
    mat_ids = {v for v in mats.values() if v is not None}
    allv, allm, alln = collect_material_vertices(j, buf, world, mat_ids, mesh_to_nodes)
    if len(allv) == 0:
        notes.append("no engine nacelle materials found at all -- no engine covers placed")
        return items, notes, []

    # Engine number -> reference point, from VFX_Contrails_Engine_N (numbered
    # explicitly 1..4 in the asset). Cross-checked against engines.cfg:
    # Engine.0..3 lateral offsets are -84,-47.5,+47.5,+84 ft, i.e. left
    # outboard, left inboard, right inboard, right outboard in that order --
    # exactly the interface's 1..4 numbering -- and the contrail nodes'
    # |x| ordering (largest,mid,mid,largest with matching sign pattern)
    # confirms +X is the left side of this aircraft in the glTF frame.
    refs = {}
    for n in (1, 2, 3, 4):
        idx = node_by_name(names, f"VFX_Contrails_Engine_{n}")
        if idx is None:
            notes.append(f"VFX_Contrails_Engine_{n} not found -- engine {n} covers left out")
            continue
        refs[n] = world[idx][:3, 3]
    if len(refs) < 4:
        return items, notes, []

    xz = allv[:, [0, 2]]
    ref_xz = np.array([refs[n][[0, 2]] for n in (1, 2, 3, 4)])
    d = np.linalg.norm(xz[:, None, :] - ref_xz[None, :, :], axis=2)
    assign = d.argmin(axis=1) + 1

    cover_stats = []

    def _fitted_cover_item(item_id, tooltip, feature, engine_pts, lip_center_xy, lip_z, skirt_dir, ref_pts, src, skirt_depth=0.25):
        """Builds one fitted fabric cover (dome + skirt + band) plus 3
        streamers as a single Item; falls back to the old flat disc if the
        fit raises, so the object always builds even if a rare profile is
        degenerate."""
        try:
            verts, idx, confident, cover_pts, attach_pts = build_wrap_cover(
                engine_pts, lip_center_xy, lip_z, skirt_dir, skirt_depth=skirt_depth
            )
            clearance = min_clearance(cover_pts, ref_pts)
            pieces = [{"kind": "wrap_mesh", "verts": verts, "idx": idx, "region": "fabric_plain"}]
            avg_r = float(np.linalg.norm(cover_pts[:, :2] - np.array(lip_center_xy), axis=1).mean())
            cap_out_dir = np.array([0.0, 0.0, -skirt_dir])
            label_center = np.array([lip_center_xy[0], lip_center_xy[1], lip_z]) + cap_out_dir * 0.06
            pieces.append({"kind": "label", "center": label_center, "out_dir": cap_out_dir, "half_w": avg_r * 0.55, "half_h": avg_r * 0.24, "region": "fabric_print"})
            for ap in attach_pts:
                pieces.append(_streamer_piece(ap, length=1.0, width=0.07))
            cover_stats.append({"id": item_id, "clearance_m": clearance, "confident_cowl_band": confident, "n_tris": len(idx) // 3})
            fit_note = "fitted cover (dome+skirt+band)" if confident else "fitted cover (dome+skirt+band; cowl band under-sampled, skirt taper reused the lip profile scaled 0.97 -- reduced confidence)"
            return Item(item_id, tooltip, pieces, f"{src}; {fit_note}, min clearance {clearance*100:.1f} cm", feature, 0.0)
        except Exception as exc:  # pragma: no cover -- keep the object buildable
            notes.append(f"{item_id}: fitted-cover build failed ({exc}) -- fell back to a flat disc")
            c, r = ring_fit(engine_pts)
            normal = np.array([0.0, 0.0, -skirt_dir])
            return Item(item_id, tooltip, [{"kind": "disc", "center": c - normal * -0.06, "normal": normal, "radius": r * 0.97, "band": True}], src + "; DISC FALLBACK", feature, 0.0)

    for n in (1, 2, 3, 4):
        sel = assign == n
        matset = set(allm[sel])
        nacelle_mat = mats["ENG_LH"] if mats["ENG_LH"] in matset else mats["ENG_RH"]
        if nacelle_mat is None or nacelle_mat not in matset:
            notes.append(f"engine {n}: no exterior nacelle geometry found -- inlet/exhaust covers left out")
            continue
        v_nac = allv[sel & (allm == nacelle_mat)]
        zmax = v_nac[:, 2].max()
        zr = v_nac[:, 2].max() - v_nac[:, 2].min()
        front = v_nac[v_nac[:, 2] > zmax - 0.03 * zr]
        cf, rf = ring_fit(front)
        lip_z = float(cf[2])
        items.append(
            _fitted_cover_item(
                f"eng_inlet_cover_{n}",
                f"Remove engine {n} inlet cover",
                f"engine {n} inlet lip",
                v_nac,
                cf[:2],
                lip_z,
                -1.0,  # skirt runs aft over the outer cowl
                v_nac,
                f"nacelle lip ring, engine {n} (material {'A380_EXTERIOR_ENG_LH' if nacelle_mat==mats['ENG_LH'] else 'A380_EXTERIOR_ENG_RH'}, forward 3% z-slice, angular profile fit)",
            )
        )

        # The visible exhaust plug/centre-body is the rounded nose on the
        # engine core cowl (A380_EXTERIOR_ENG_CORES_*): its own aft-most tip
        # is already fairly round (radius ~0.5 m right at the tip in this
        # asset, not a point), so it is anchored the same way the inlet lip
        # is -- a thin slice at the cluster's own most-exposed z, not an
        # arbitrary window partway along the duct (which used to land the
        # cover's anchor a half-metre short of the real tip, leaving it
        # floating clear of the geometry it was meant to cover). The
        # reverser cowl (when present) isn't the plug itself -- it's the
        # surrounding fixed/translating cowl further forward -- so it is
        # folded into the clearance reference only, not used to place the
        # cover.
        # The visible centre-body is the nacelle's own tail-cone taper
        # (same material as the inlet lip, A380_EXTERIOR_ENG_*): it runs the
        # nacelle's own aft-most point (a near-point tip, mirroring the
        # inlet's forward-most point) out to where its radius settles near
        # the engine core/reverser cowl's own diameter. A fixed 0.2-0.3 m
        # skirt (right for the inlet's blunt lip) stays needle-thin this
        # close to a pointed tip, so the skirt instead runs until the
        # nacelle's own profile has widened out to the core cowl's radius --
        # long enough to actually enclose the centre-body, not just graze
        # its very point.
        core_mat = mats["ENG_CORES_LH"] if mats["ENG_CORES_LH"] in matset else mats["ENG_CORES_RH"]
        rev_mat = mats["REVERSER_CORE"]
        has_rev = rev_mat is not None and rev_mat in matset
        has_core = core_mat is not None and core_mat in matset
        zmin_n = float(v_nac[:, 2].min())
        zr_n = float(v_nac[:, 2].max() - zmin_n)
        tip_n = v_nac[v_nac[:, 2] < zmin_n + 0.02 * zr_n]
        if len(tip_n) < 20:
            tip_n = v_nac[v_nac[:, 2] < zmin_n + 0.05 * zr_n]
        ce, _ = ring_fit(tip_n)
        exh_center_xy, exh_z = ce[:2], zmin_n
        target_r = None
        if has_core:
            v_core = allv[sel & (allm == core_mat)]
            target_r = float(np.hypot(v_core[:, 0] - exh_center_xy[0], v_core[:, 1] - exh_center_xy[1]).max())
        elif has_rev:
            v_rev = allv[sel & (allm == rev_mat)]
            target_r = float(np.hypot(v_rev[:, 0] - exh_center_xy[0], v_rev[:, 1] - exh_center_xy[1]).max())
        exh_skirt_depth = 0.25
        if target_r is not None:
            # walk out from the tip until the nacelle's own radius reaches
            # the surrounding core/reverser cowl's radius (its natural
            # "cinch" point), clamped to a sane range.
            for frac in np.linspace(0.02, 0.6, 30):
                band = v_nac[v_nac[:, 2] < zmin_n + frac * zr_n]
                if len(band) < 20:
                    continue
                r = np.hypot(band[:, 0] - exh_center_xy[0], band[:, 1] - exh_center_xy[1]).max()
                if r >= target_r * 0.9:
                    exh_skirt_depth = max(0.25, frac * zr_n)
                    break
            else:
                exh_skirt_depth = max(0.25, 0.6 * zr_n)
        ref_pts_exh = [v_nac]
        if has_core:
            ref_pts_exh.append(v_core)
        if has_rev:
            ref_pts_exh.append(allv[sel & (allm == rev_mat)])
        ref_pts_exh = np.concatenate(ref_pts_exh)
        nacelle_name = "A380_EXTERIOR_ENG_LH" if nacelle_mat == mats["ENG_LH"] else "A380_EXTERIOR_ENG_RH"
        src = (f"nacelle tail-cone tip, engine {n} (material {nacelle_name}, aft-most 2% z-slice, angular profile "
               f"fit, skirt run out {exh_skirt_depth:.2f} m to the core/reverser cowl's own radius)")
        if not has_core and not has_rev:
            src += " -- no separate core/reverser geometry on this engine; clearance checked against the nacelle only"
        items.append(
            _fitted_cover_item(
                f"eng_exhaust_cover_{n}",
                f"Remove engine {n} exhaust plug",
                f"engine {n} exhaust nozzle",
                v_nac,
                exh_center_xy,
                exh_z,
                1.0,  # skirt runs forward, over the nozzle/core, into the engine
                ref_pts_exh,
                src,
                skirt_depth=exh_skirt_depth,
            )
        )
    return items, notes, cover_stats


FT = 0.3048
# Generic chock size (no sourced A380 chock drawing): a wedge whose sloped
# face the tyre rests against, toe tucked under the tyre.
CHOCK_LENGTH_M = 0.35
CHOCK_HEIGHT_M = 0.22
# How far ahead of/behind an axle's ground contact the chock's toe sits, as
# a fraction of the tyre radius: close enough to read as touching the tyre,
# far enough that the slope stays below the tyre's curve.
CHOCK_TOE_OFFSET_R = 0.35
# A downlock pin sits on the leg well above the tyres (no brace geometry is
# identifiable by name in the asset); this much above the tyre top.
PIN_ABOVE_TYRE_TOP_M = 0.6
# Each wheel set's footprint, measured from this asset's own tyre nodes
# (A380_EXTERIOR_TYRES). Only distances between a bogie's own tyres are used:
# they are the same in any gear pose, whereas the tyres' positions are not --
# the glTF's rest pose has every leg retracted (the wing gear swung sideways
# toward the fuselage and its bogie tilted ~23 degrees), which is why the
# first version of this tool, sizing from rest-pose bounding boxes, gave the
# wing gear the body gear's length.
#   nose: two tyres (NLG_RH_Wheel.001/.002) 1.18 m apart centre to centre on
#         one axle, plus one tyre's width.
#   body: axle pairs at z -1.75 and -3.39, the rear (steering) axle's two
#         tyres centred at -5.04 -- 1.645 m apart, 3.29 m front to rear;
#         each axle pair node is 1.92 m wide across its tyres.
#   wing: two axle-pair nodes whose centres are 1.73 m apart; its pairs are
#         tilted in the rest pose, so their width is taken from the body
#         gear's untilted pair (the same main-gear tyre).
# (width across the tyres, front-to-rear axle span), metres.
WHEEL_SETS = {
    "nose": (1.18 + 0.46, 0.0),
    "body": (1.92, 3.29),
    "wing": (1.92, 1.73),
}


def read_acf_gear(acf_path: Path):
    """X-Plane's own gear, from the .acf: per gear index, the contact point
    (x, z) and the uncompressed tyre bottom (y), in metres in X-Plane's
    aircraft frame (x right, y up, z aft), plus the tyre radius.

    The glTF's own tyre vertices cannot give this: its rest pose has the
    gear *retracted* (the nose leg's own `c_gear` animation rotates it 108
    degrees down from there), so chocks placed on it floated metres up
    inside the wheel wells. The extended nose wheel, evaluated through that
    animation, lands at z -30.07 m, bottom -4.67 m; the .acf's nose gear
    contact is z -30.22 m, bottom -4.62 m -- the physics and the drawn gear
    agree once the gear is down, so the physics is the ground truth here."""
    props = {}
    for line in acf_path.read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith("P _gear/"):
            parts = line.split()
            if len(parts) >= 3:
                props[parts[1]] = parts[2]
    gear = {}
    for n in range(10):
        key = f"_gear/{n}/"
        if int(float(props.get(key + "_gear_type", "0"))) == 0:
            continue
        try:
            x = float(props[key + "_gear_x"]) * FT
            y = float(props[key + "_gear_y"]) * FT
            z = float(props[key + "_gear_z"]) * FT
            leg = float(props[key + "_leg_len"]) * FT
            r = float(props[key + "_tire_radius"]) * FT
        except KeyError:
            continue
        # Legs extend vertically (`_lonE`/`_latE` 0 on every A380 leg): the
        # attach point, down the leg to the axle, down the tyre to the ground.
        gear[n] = {"x": x, "z": z, "ground_y": y - leg - r, "r": r}
    return gear


def extract_gear(j, buf, world, names, mesh_to_nodes, acf_gear):
    items = []
    notes = []
    if not acf_gear:
        notes.append("no --gear-acf gear data -- no gear pins/chocks placed")
        return items, notes
    # X-Plane gear index -> interface ids: 0 nose; x < 0 is left; of the
    # mains, the pair nearer the centreline is the body gear.
    mains = [n for n in acf_gear if n != 0]
    body_x = min(abs(acf_gear[m]["x"]) for m in mains) if mains else 0.0
    legs = []
    if 0 in acf_gear:
        legs.append((0, "gear_pin_nose", "chocks_nose", "nose"))
    for n in sorted(mains):
        g = acf_gear[n]
        side = "l" if g["x"] < 0 else "r"
        kind = "body" if abs(abs(g["x"]) - body_x) < 1e-6 else "wing"
        label = f"{'left' if side == 'l' else 'right'} {kind}"
        legs.append((n, f"gear_pin_{side}{kind}", f"chocks_{side}{kind}", label))

    for n, pin_id, chock_id, label in legs:
        g = acf_gear[n]
        # X-Plane aircraft frame -> this tool's glTF frame (x left, z forward).
        gx, gz, ground_y, r = -g["x"], -g["z"], g["ground_y"], g["r"]
        set_width, axle_span = WHEEL_SETS[label.split()[-1]]
        width = set_width + 0.2
        size_src = f"wheel set measured from the glTF's own tyre nodes ({set_width:.2f} m across, axles {axle_span:.2f} m front to rear)"
        # Front and rear axles' ground contact, the bogie centred on the
        # .acf contact point.
        half_axles = axle_span / 2.0
        toe = CHOCK_TOE_OFFSET_R * r
        front_toe = gz + half_axles + toe
        rear_toe = gz - half_axles - toe
        pin_pos = np.array([gx, ground_y + 2 * r + PIN_ABOVE_TYRE_TOP_M, gz])
        items.append(
            Item(
                pin_id,
                f"Toggle {label} gear pin",
                [{"kind": "pin", "center": pin_pos}, _streamer_piece(pin_pos - np.array([0, 0.02, 0]), length=0.6)],
                f".acf _gear/{n} contact point (x, z); height from the .acf tyre bottom",
                f"{label} gear leg, above the tyres (no downlock/drag-brace geometry identifiable by name)",
                0.0,
            )
        )
        items.append(
            Item(
                chock_id,
                f"Toggle {label} gear chocks",
                [
                    {"kind": "wedge", "center": np.array([gx, ground_y, front_toe + CHOCK_LENGTH_M / 2]), "ground_y": ground_y, "facing": -1.0, "width": width},
                    {"kind": "wedge", "center": np.array([gx, ground_y, rear_toe - CHOCK_LENGTH_M / 2]), "ground_y": ground_y, "facing": 1.0, "width": width},
                ],
                f".acf _gear/{n}: contact point and uncompressed tyre bottom; {size_src}",
                f"{label} gear, on the ground fore and aft of the wheels",
                0.0,
                ground_gear=n,
            )
        )
    return items, notes


def build_mesh_to_nodes(j):
    out = {}
    for i, n in enumerate(j["nodes"]):
        if "mesh" in n:
            out.setdefault(n["mesh"], []).append(i)
    return out


def extract_all(j, buf, acf_gear=None):
    world = build_world_matrices(j)
    names = [n.get("name", "") for n in j["nodes"]]
    mesh_to_nodes = build_mesh_to_nodes(j)
    items, notes, cover_stats = [], [], []
    it, nt = extract_probes(j, buf, world, names)
    items.extend(it)
    notes.extend(nt)
    it, nt, cs = extract_engines(j, buf, world, names, mesh_to_nodes)
    items.extend(it)
    notes.extend(nt)
    cover_stats.extend(cs)
    it, nt = extract_gear(j, buf, world, names, mesh_to_nodes, acf_gear)
    items.extend(it)
    notes.extend(nt)
    return items, notes, cover_stats


# ---------------------------------------------------------------------------
# Geometry builders. Everything here works in glTF/world space (X, Y up, Z
# forward); the single axis flip to X-Plane's OBJ8 frame (x=-x, z=-z) is
# applied once, when vertices are written out.
# ---------------------------------------------------------------------------

Vertex = tuple  # (pos[3], normal[3], uv[2])


def _basis_from_axis(axis):
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    up = np.array([0.0, 1.0, 0.0])
    if abs(np.dot(axis, up)) > 0.95:
        up = np.array([1.0, 0.0, 0.0])
    b1 = np.cross(up, axis)
    b1 /= np.linalg.norm(b1) + 1e-12
    b2 = np.cross(axis, b1)
    return b1, b2


def gen_cylinder(center, axis, length, radius, uv, segments=8, caps=True):
    verts, idx = [], []
    b1, b2 = _basis_from_axis(axis)
    half = axis * (length / 2.0)
    ring0 = center - half
    ring1 = center + half
    u0, v0, u1, v1 = uv
    base = 0
    for ring_pos, vcoord in ((ring0, v0), (ring1, v1)):
        for s in range(segments + 1):
            ang = 2 * math.pi * s / segments
            off = b1 * math.cos(ang) * radius + b2 * math.sin(ang) * radius
            n = off / (radius + 1e-12)
            verts.append((ring_pos + off, n, (u0 + (u1 - u0) * s / segments, vcoord)))
    for s in range(segments):
        a, b, c, d = base + s, base + s + 1, base + segments + 1 + s + 1, base + segments + 1 + s
        idx += [a, b, c, a, c, d]
    if caps:
        c0i = len(verts)
        verts.append((ring0, -axis, (0.5, 0.5)))
        for s in range(segments + 1):
            ang = 2 * math.pi * s / segments
            off = b1 * math.cos(ang) * radius + b2 * math.sin(ang) * radius
            verts.append((ring0 + off, -axis, (0.5 + 0.5 * math.cos(ang), 0.5 + 0.5 * math.sin(ang))))
        for s in range(segments):
            idx += [c0i, c0i + 1 + s + 1, c0i + 1 + s]
        c1i = len(verts)
        verts.append((ring1, axis, (0.5, 0.5)))
        for s in range(segments + 1):
            ang = 2 * math.pi * s / segments
            off = b1 * math.cos(ang) * radius + b2 * math.sin(ang) * radius
            verts.append((ring1 + off, axis, (0.5 + 0.5 * math.cos(ang), 0.5 + 0.5 * math.sin(ang))))
        for s in range(segments):
            idx += [c1i, c1i + 1 + s, c1i + 1 + s + 1]
    return verts, idx


def gen_disc(center, normal, radius, uv, segments=20, inner_radius=0.0):
    verts, idx = [], []
    n = normal / (np.linalg.norm(normal) + 1e-12)
    b1, b2 = _basis_from_axis(n)
    u0, v0, u1, v1 = uv
    ring = []
    for s in range(segments):
        ang = 2 * math.pi * s / segments
        off = b1 * math.cos(ang) * radius + b2 * math.sin(ang) * radius
        p = center + off
        uv_p = (u0 + (u1 - u0) * (0.5 + 0.5 * math.cos(ang)), v0 + (v1 - v0) * (0.5 + 0.5 * math.sin(ang)))
        ring.append(len(verts))
        verts.append((p, n, uv_p))
    if inner_radius > 0:
        # annulus: also generate an inner ring and skip the centre
        inner = []
        for s in range(segments):
            ang = 2 * math.pi * s / segments
            off = b1 * math.cos(ang) * inner_radius + b2 * math.sin(ang) * inner_radius
            p = center + off
            inner.append(len(verts))
            verts.append((p, n, (0.5, 0.5)))
        for s in range(segments):
            a, b = ring[s], ring[(s + 1) % segments]
            c, d = inner[(s + 1) % segments], inner[s]
            idx += [a, b, c, a, c, d]
    else:
        ci = len(verts)
        verts.append((center, n, (0.5, 0.5)))
        for s in range(segments):
            idx += [ci, ring[s], ring[(s + 1) % segments]]
    return verts, idx


def gen_box(center, axis_x, axis_y, axis_z, half, uv, double_sided_top=False):
    """A box centred at `center`; `half` = (hx,hy,hz) half-extents along the
    given orthonormal-ish axes. Each face gets the full `uv` rect."""
    ax, ay, az = axis_x, axis_y, axis_z
    hx, hy, hz = half
    corners = {}
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                corners[(sx, sy, sz)] = center + ax * sx * hx + ay * sy * hy + az * sz * hz
    u0, v0, u1, v1 = uv
    verts, idx = [], []

    def face(n_axis, corner_list):
        base = len(verts)
        n = n_axis / (np.linalg.norm(n_axis) + 1e-12)
        uvs = [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]
        for c, uvc in zip(corner_list, uvs):
            verts.append((c, n, uvc))
        idx.extend([base, base + 1, base + 2, base, base + 2, base + 3])

    face(az, [corners[(-1, -1, 1)], corners[(1, -1, 1)], corners[(1, 1, 1)], corners[(-1, 1, 1)]])
    face(-az, [corners[(1, -1, -1)], corners[(-1, -1, -1)], corners[(-1, 1, -1)], corners[(1, 1, -1)]])
    face(ay, [corners[(-1, 1, -1)], corners[(-1, 1, 1)], corners[(1, 1, 1)], corners[(1, 1, -1)]])
    face(-ay, [corners[(-1, -1, 1)], corners[(-1, -1, -1)], corners[(1, -1, -1)], corners[(1, -1, 1)]])
    face(ax, [corners[(1, -1, -1)], corners[(1, -1, 1)], corners[(1, 1, 1)], corners[(1, 1, -1)]])
    face(-ax, [corners[(-1, -1, 1)], corners[(-1, -1, -1)], corners[(-1, 1, -1)], corners[(-1, 1, 1)]])
    return verts, idx


def gen_ribbon(attach, length, width, uv):
    """A thin double-sided quad hanging straight down from `attach` (world
    -Y), textured with the streamer texture running along its length."""
    down = np.array([0.0, -1.0, 0.0])
    side = np.array([1.0, 0.0, 0.0])
    hw = width / 2.0
    top_l = attach - side * hw
    top_r = attach + side * hw
    bot_l = top_l + down * length
    bot_r = top_r + down * length
    n = np.array([0.0, 0.0, 1.0])
    u0, v0, u1, v1 = uv
    verts = [
        (top_l, n, (u0, v0)),
        (top_r, n, (u1, v0)),
        (bot_r, n, (u1, v1)),
        (bot_l, n, (u0, v1)),
    ]
    idx = [0, 1, 2, 0, 2, 3]
    # back face (flipped normal) so the streamer reads from both sides
    verts2 = [
        (top_l, -n, (u0, v0)),
        (bot_l, -n, (u0, v1)),
        (bot_r, -n, (u1, v1)),
        (top_r, -n, (u1, v0)),
    ]
    base = len(verts)
    verts.extend(verts2)
    idx += [base, base + 1, base + 2, base, base + 2, base + 3]
    return verts, idx


def gen_wedge(center, ground_y, facing, length, width, height, uv):
    """A chock: a right-triangular prism. `facing` = +1 slopes up toward +Z
    (glTF forward), -1 toward -Z, so the pair of wedges leans against a
    wheel from the front and from behind."""
    hw = width / 2.0
    toe = center + np.array([0, 0, facing * length / 2.0])
    heel = center - np.array([0, 0, facing * length / 2.0])
    p_toe_l = np.array([center[0] - hw, ground_y, toe[2]])
    p_toe_r = np.array([center[0] + hw, ground_y, toe[2]])
    p_heel_l = np.array([center[0] - hw, ground_y, heel[2]])
    p_heel_r = np.array([center[0] + hw, ground_y, heel[2]])
    p_top_l = np.array([center[0] - hw, ground_y + height, heel[2]])
    p_top_r = np.array([center[0] + hw, ground_y + height, heel[2]])
    u0, v0, u1, v1 = uv
    verts, idx = [], []

    def tri(pts, n):
        base = len(verts)
        for p in pts:
            verts.append((p, n, (u0, v0)))
        idx.extend([base, base + 1, base + 2])

    def quad(pts, n):
        base = len(verts)
        uvs = [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]
        for p, uvc in zip(pts, uvs):
            verts.append((p, n, uvc))
        idx.extend([base, base + 1, base + 2, base, base + 2, base + 3])

    side_n = np.array([0.0, 0.0, facing])
    slope_dir = np.array([0.0, height, -facing * length]) if length > 0 else np.array([0, 1, 0])
    slope_n = np.cross(np.array([1.0, 0.0, 0.0]), slope_dir)
    slope_n = slope_n / (np.linalg.norm(slope_n) + 1e-12) * facing
    quad([p_toe_l, p_toe_r, p_heel_r, p_heel_l], np.array([0.0, -1.0, 0.0]))  # bottom
    quad([p_heel_l, p_heel_r, p_top_r, p_top_l], -side_n)  # vertical back face
    quad([p_toe_r, p_toe_l, p_top_l, p_top_r], slope_n)  # sloped face (toe -> top edge)
    # two triangular ends
    tri([p_toe_l, p_heel_l, p_top_l], np.array([-1.0, 0.0, 0.0]))
    tri([p_toe_r, p_top_r, p_heel_r], np.array([1.0, 0.0, 0.0]))
    return verts, idx


def to_obj_frame(p):
    return np.array([-p[0], p[1], -p[2]])


def piece_to_geometry(piece, uv_regions):
    kind = piece["kind"]
    if kind == "sleeve":
        return gen_cylinder(piece["center"], piece["axis"], piece["length"], piece["radius"], uv_regions["cover_red"], segments=8)
    if kind == "pad":
        c, axis = piece["center"], piece["axis"]
        b1, b2 = _basis_from_axis(axis)
        return gen_box(c, axis, b2, b1, (piece["length"] / 2, piece["thick"] / 2, piece["width"] / 2), uv_regions["cover_red_text"])
    if kind == "pin":
        return gen_cylinder(piece["center"], np.array([0.0, 1.0, 0.0]), 0.22, 0.018, uv_regions["cover_red"], segments=6)
    if kind == "disc":
        verts, idx = gen_disc(piece["center"], piece["normal"], piece["radius"], uv_regions["cover_red_text"], segments=24)
        return verts, idx
    if kind == "streamer":
        return gen_ribbon(piece["attach"], piece["length"], piece["width"], uv_regions["streamer"])
    if kind == "wedge":
        return gen_wedge(piece["center"], piece["ground_y"], piece["facing"], CHOCK_LENGTH_M, piece.get("width", 0.22), CHOCK_HEIGHT_M, uv_regions["chock"])
    if kind == "wrap_mesh":
        u0, v0, u1, v1 = uv_regions[piece["region"]]
        verts = [(p, n, (u0 + (u1 - u0) * lu, v0 + (v1 - v0) * lv)) for (p, n, (lu, lv)) in piece["verts"]]
        return verts, piece["idx"]
    if kind == "label":
        return gen_flat_label(piece["center"], piece["out_dir"], piece["half_w"], piece["half_h"], uv_regions[piece["region"]])
    raise ValueError(f"unknown piece kind {kind}")


# ---------------------------------------------------------------------------
# Texture atlas
# ---------------------------------------------------------------------------


def build_atlas(out_png: Path):
    """Builds the atlas as four independent 512x512 tiles, each drawn (and
    clipped) on its own Image and then pasted into the corresponding quadrant
    -- so a diagonal chock stripe or a wide line of text can never bleed
    across into a neighbouring tile the way drawing directly on one shared
    canvas would."""
    tile = 512
    size_x, size_y = tile * 3, tile * 2
    atlas = Image.new("RGBA", (size_x, size_y), (0, 0, 0, 255))
    red = (178, 22, 22, 255)
    white = (245, 245, 245, 255)
    yellow = (235, 190, 20, 255)
    black = (20, 20, 20, 255)
    fabric_base = (96, 16, 16, 255)  # dark red woven fabric for the engine covers
    try:
        font_big = ImageFont.truetype("C:/Windows/Fonts/arialbd.ttf", 40)
        font_small = ImageFont.truetype("C:/Windows/Fonts/arialbd.ttf", 26)
    except Exception:
        font_big = font_small = ImageFont.load_default()
    text = "REMOVE BEFORE FLIGHT"

    def fitted_font(txt, max_w, start_size):
        s = start_size
        while s > 8:
            try:
                f = ImageFont.truetype("C:/Windows/Fonts/arialbd.ttf", s)
            except Exception:
                return ImageFont.load_default()
            tmp = ImageDraw.Draw(Image.new("RGB", (1, 1)))
            if tmp.textlength(txt, font=f) <= max_w:
                return f
            s -= 2
        return ImageFont.load_default()

    # cover_red: plain red fabric-ish tile.
    t_plain = Image.new("RGBA", (tile, tile), red)
    d = ImageDraw.Draw(t_plain)
    for i in range(0, tile, 37):
        d.line([(0, i), (tile - 1, i)], fill=(160, 12, 12, 255), width=1)
    atlas.paste(t_plain, (0, 0))

    # cover_red_text: red with white REMOVE BEFORE FLIGHT, fitted to the tile.
    t_text = Image.new("RGBA", (tile, tile), red)
    d = ImageDraw.Draw(t_text)
    f = fitted_font(text, tile - 20, 40)
    tw = d.textlength(text, font=f)
    d.text(((tile - tw) / 2, tile * 0.22), text, font=f, fill=white)
    d.text(((tile - tw) / 2, tile * 0.70), text, font=f, fill=white)
    atlas.paste(t_text, (tile, 0))

    # streamer: red ribbon with repeated, smaller text (it is a narrow strip
    # in practice) plus white end-bands.
    t_streamer = Image.new("RGBA", (tile, tile), red)
    d = ImageDraw.Draw(t_streamer)
    fs = fitted_font(text, tile - 16, 26)
    tws = d.textlength(text, font=fs)
    for row in range(4):
        y = 60 + row * 115
        d.text(((tile - tws) / 2, y), text, font=fs, fill=white)
    d.rectangle([0, 0, tile - 1, 14], fill=white)
    d.rectangle([0, tile - 15, tile - 1, tile - 1], fill=white)
    atlas.paste(t_streamer, (0, tile))

    # chock: yellow with black hazard stripes, drawn on its own tile so the
    # diagonals are clipped to it.
    t_chock = Image.new("RGBA", (tile, tile), yellow)
    d = ImageDraw.Draw(t_chock)
    for i in range(-tile, tile, 48):
        d.line([(i, tile - 1), (i + tile, 0)], fill=black, width=16)
    atlas.paste(t_chock, (tile, tile))

    # fabric_plain / fabric_print: woven dark-red fabric for the engine
    # inlet/exhaust covers -- a fine crosshatch weave plus a seam cross,
    # baked straight into the albedo (no separate normal map in this pass;
    # the weave and seam lines are the shading cue, per the task's own
    # albedo-bake fallback).
    def weave(base):
        img = Image.new("RGBA", (tile, tile), base)
        dd = ImageDraw.Draw(img)
        dark = tuple(max(0, c - 30) for c in base[:3]) + (255,)
        light = tuple(min(255, c + 22) for c in base[:3]) + (255,)
        for i in range(0, tile, 9):
            dd.line([(i, 0), (i, tile - 1)], fill=dark, width=1)
        for jj in range(0, tile, 9):
            dd.line([(0, jj), (tile - 1, jj)], fill=light, width=1)
        # a couple of stitched seam lines (as on a real fitted sewn cover)
        dd.line([(tile * 0.5, 0), (tile * 0.5, tile - 1)], fill=(15, 15, 15, 255), width=3)
        dd.line([(0, tile * 0.85), (tile - 1, tile * 0.85)], fill=(15, 15, 15, 255), width=3)
        return img, dd

    t_fab_plain, _ = weave(fabric_base)
    atlas.paste(t_fab_plain, (2 * tile, 0))

    t_fab_print, dp = weave(fabric_base)
    fp = fitted_font(text, tile - 24, 34)
    twp = dp.textlength(text, font=fp)
    dp.text(((tile - twp) / 2, tile * 0.40), text, font=fp, fill=white)
    atlas.paste(t_fab_print, (2 * tile, tile))

    atlas.save(out_png)
    regions = {
        "cover_red": (0.0, 0.0, 1 / 3, 0.5),
        "cover_red_text": (1 / 3, 0.0, 2 / 3, 0.5),
        "streamer": (0.0, 0.5, 1 / 3, 1.0),
        "chock": (1 / 3, 0.5, 2 / 3, 1.0),
        "fabric_plain": (2 / 3, 0.0, 1.0, 0.5),
        "fabric_print": (2 / 3, 0.5, 1.0, 1.0),
    }
    return regions


# ---------------------------------------------------------------------------
# OBJ8 writing
# ---------------------------------------------------------------------------


def write_obj8(items, uv_regions, out_path: Path, texture_name: str):
    vt_lines = []
    idx_all = []
    body = []
    point_count = 0

    for item in items:
        anim_verts = []
        anim_idx = []
        for piece in item.pieces:
            verts, idx = piece_to_geometry(piece, uv_regions)
            base = len(anim_verts)
            anim_verts.extend(verts)
            anim_idx.extend([i + base for i in idx])
        if not anim_verts:
            continue
        first_index = len(idx_all)
        base_vertex = point_count
        for pos, normal, uv in anim_verts:
            p = to_obj_frame(pos)
            n = to_obj_frame(normal)
            nlen = math.sqrt(n[0] ** 2 + n[1] ** 2 + n[2] ** 2) or 1.0
            n = n / nlen
            vt_lines.append(f"VT {p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {n[0]:.4f} {n[1]:.4f} {n[2]:.4f} {uv[0]:.5f} {uv[1]:.5f}")
            point_count += 1
        idx_all.extend([i + base_vertex for i in anim_idx])
        n_idx = len(anim_idx)

        body.append("ANIM_begin")
        # Geometry starts out visible and ANIM_show only ever shows it, so a
        # removed item has to be hidden explicitly (0 = removed).
        body.append(f"ANIM_hide -0.5 0.5 fbw/walkaround/{item.id}")
        if item.ground_gear is not None:
            body.append(f"ANIM_trans_begin sim/flightmodel2/gear/tire_vertical_deflection_mtr[{item.ground_gear}]")
            body.append("ANIM_trans_key 0 0 0 0")
            body.append("ANIM_trans_key 1 0 1 0")
            body.append("ANIM_trans_end")
        body.append(f"ATTR_manip_command hand fbw/walkaround/{item.id}_toggle {item.tooltip}")
        body.append(f"TRIS {first_index} {n_idx}")
        body.append("ATTR_manip_none")
        body.append("ANIM_end")

    out = []
    out.append("I")
    out.append("800")
    out.append("OBJ")
    out.append("")
    out.append(f"TEXTURE {texture_name}")
    out.append("ATTR_no_cull")
    out.append("ATTR_shadow")
    out.append("")
    out.append(f"POINT_COUNTS {point_count} 0 0 {len(idx_all)}")
    out.append("")
    out.extend(vt_lines)
    out.append("")
    i = 0
    while i + 10 <= len(idx_all):
        chunk = idx_all[i : i + 10]
        out.append("IDX10 " + " ".join(str(x) for x in chunk))
        i += 10
    for x in idx_all[i:]:
        out.append(f"IDX {x}")
    out.append("")
    out.extend(body)
    out.append("")

    out_path.write_text("\n".join(out), encoding="utf-8")
    return point_count, len(idx_all)


# ---------------------------------------------------------------------------
# Verification: parse the OBJ back, sanity-check it, and render top/side PNGs.
# ---------------------------------------------------------------------------


def verify_obj(obj_path: Path, expected_points, expected_indices):
    text = obj_path.read_text(encoding="utf-8")
    lines = text.splitlines()
    report = []
    ok = True

    pc_line = next((l for l in lines if l.startswith("POINT_COUNTS")), None)
    if pc_line is None:
        report.append("FAIL: no POINT_COUNTS line")
        ok = False
    else:
        parts = pc_line.split()
        pts, idxs = int(parts[1]), int(parts[4])
        if pts != expected_points or idxs != expected_indices:
            report.append(f"FAIL: POINT_COUNTS says {pts} pts / {idxs} idx, writer produced {expected_points}/{expected_indices}")
            ok = False
        else:
            report.append(f"OK: POINT_COUNTS matches writer output ({pts} pts, {idxs} idx)")
        vt_count = sum(1 for l in lines if l.startswith("VT "))
        if vt_count != pts:
            report.append(f"FAIL: {vt_count} VT lines but POINT_COUNTS claims {pts}")
            ok = False
        else:
            report.append(f"OK: {vt_count} VT lines match POINT_COUNTS")

    begins = sum(1 for l in lines if l == "ANIM_begin")
    ends = sum(1 for l in lines if l == "ANIM_end")
    if begins != ends:
        report.append(f"FAIL: {begins} ANIM_begin vs {ends} ANIM_end")
        ok = False
    else:
        report.append(f"OK: {begins} ANIM_begin/ANIM_end pairs balance")

    max_idx = -1
    n_idx_seen = 0
    for l in lines:
        if l.startswith("IDX10 "):
            vals = [int(x) for x in l.split()[1:]]
            n_idx_seen += len(vals)
            max_idx = max(max_idx, max(vals))
        elif l.startswith("IDX "):
            v = int(l.split()[1])
            n_idx_seen += 1
            max_idx = max(max_idx, v)
    if n_idx_seen != expected_indices:
        report.append(f"FAIL: read back {n_idx_seen} indices, expected {expected_indices}")
        ok = False
    if max_idx >= expected_points:
        report.append(f"FAIL: max index {max_idx} out of range for {expected_points} points")
        ok = False
    else:
        report.append(f"OK: all indices in range (max {max_idx} < {expected_points} points)")

    return ok, report


def render_views(items, aircraft_outline, out_dir: Path):
    """Draws a simple top-view and side-view PNG: the aircraft's outline
    (from a coarse hull of the exterior vertices, in OBJ frame) plus a
    coloured box per item at its position, for a by-eye sanity check."""
    colors = {
        "pitot": (255, 60, 60),
        "aoa": (255, 150, 0),
        "static": (255, 220, 0),
        "eng_inlet": (60, 160, 255),
        "eng_exhaust": (120, 60, 220),
        "gear_pin": (0, 200, 0),
        "chocks": (230, 190, 20),
    }

    def color_for(item_id):
        for key, c in colors.items():
            if item_id.startswith(key):
                return c
        return (200, 200, 200)

    try:
        label_font = ImageFont.truetype("C:/Windows/Fonts/arial.ttf", 13)
    except Exception:
        label_font = ImageFont.load_default()

    def make_view(axes, fname, flip_y=False):
        pad = 60
        w, h = 2200, 850
        img = Image.new("RGB", (w, h), (18, 18, 22))
        draw = ImageDraw.Draw(img)
        pts = [to_obj_frame(p) for p in aircraft_outline]
        xs = [p[axes[0]] for p in pts] + [to_obj_frame(pc["center"])[axes[0]] for it in items for pc in it.pieces if "center" in pc]
        ys = [p[axes[1]] for p in pts] + [to_obj_frame(pc["center"])[axes[1]] for it in items for pc in it.pieces if "center" in pc]
        if not xs:
            return
        xmin, xmax = min(xs) - 1, max(xs) + 1
        ymin, ymax = min(ys) - 1, max(ys) + 1
        sx = (w - 2 * pad) / max(xmax - xmin, 1e-6)
        sy = (h - 2 * pad) / max(ymax - ymin, 1e-6)
        s = min(sx, sy)

        def to_px(p):
            x = pad + (p[axes[0]] - xmin) * s
            y = pad + (p[axes[1]] - ymin) * s
            if flip_y:
                y = h - y
            return x, y

        for p in pts:
            x, y = to_px(p)
            draw.ellipse([x - 1, y - 1, x + 1, y + 1], fill=(90, 90, 100))

        placed_label_rows = {}
        for it in sorted(items, key=lambda x: x.id):
            c = color_for(it.id)
            for piece in it.pieces:
                if "center" not in piece:
                    continue
                p = to_obj_frame(piece["center"])
                x, y = to_px(p)
                r = 6
                draw.rectangle([x - r, y - r, x + r, y + r], outline=c, width=2)
            p0 = to_obj_frame(it.pieces[0]["center"]) if "center" in it.pieces[0] else None
            if p0 is not None:
                x, y = to_px(p0)
                # stagger labels stacked at nearly the same (x,y) so they stay readable
                key = (round(x / 40), round(y / 40))
                row = placed_label_rows.get(key, 0)
                placed_label_rows[key] = row + 1
                draw.text((x + 8, y - 6 + row * 12), it.id, fill=c, font=label_font)
        legend_y = 10
        for key, c in colors.items():
            draw.rectangle([10, legend_y, 22, legend_y + 12], fill=c)
            draw.text((28, legend_y), key, fill=(220, 220, 220), font=label_font)
            legend_y += 16
        img.save(out_dir / fname)

    make_view((0, 2), "walkaround_top.png", flip_y=True)  # X vs Z, nose toward image top
    make_view((2, 1), "walkaround_side.png", flip_y=True)  # Z vs Y, nose to the right, profile


# ---------------------------------------------------------------------------
# --acf mode
# ---------------------------------------------------------------------------


def append_obja(acf_path: Path, obj_rel_path: str):
    """Appends one exterior `_obja` entry (X-Plane's indexed-array .acf
    format: `P _obja/<N>/<key> <value>`, plus a `P _obja/count <N>` line
    that X-Plane uses to size the array). Verified against a real, installed
    FlyByWire A380X.acf: the object list is a block of `P _obja/<N>/...`
    lines terminated by `P _obja/count <N>`, immediately before a
    `PROPERTIES_END` marker -- this reproduces that shape exactly rather
    than just tacking lines onto the end of the file, which would leave the
    entry after `PROPERTIES_END`/inside the panel sections and, going by
    that same real file, past where X-Plane's count says the array ends.
    """
    import re

    text = acf_path.read_text(encoding="utf-8", errors="replace")
    m = re.search(r"^P _obja/count (\d+)\s*$", text, flags=re.MULTILINE)
    if not m:
        raise ValueError("no 'P _obja/count N' line found in this .acf -- format not recognised, refusing to guess")
    idx = int(m.group(1))
    flags = 16 | 0x800  # outside-shadows-only (16) | clickable (0x800) = 2064
    entry_lines = [
        f"P _obja/{idx}/_obj_flags {flags}",
        f"P _obja/{idx}/_v10_att_body -1",
        f"P _obja/{idx}/_v10_att_file_stl {obj_rel_path}",
        f"P _obja/{idx}/_v10_att_gear -1",
        f"P _obja/{idx}/_v10_att_phi_ref 0.000000000",
        f"P _obja/{idx}/_v10_att_psi_ref 0.000000000",
        f"P _obja/{idx}/_v10_att_the_ref 0.000000000",
        f"P _obja/{idx}/_v10_att_wing -1",
        f"P _obja/{idx}/_v10_att_x_acf_prt_ref 0.000000000",
        f"P _obja/{idx}/_v10_att_y_acf_prt_ref 0.000000000",
        f"P _obja/{idx}/_v10_att_z_acf_prt_ref 0.000000000",
        f"P _obja/{idx}/_v10_is_internal 0",
        f"P _obja/{idx}/_v10_steers_with_gear 0",
    ]
    replacement = "\n".join(entry_lines) + f"\nP _obja/count {idx + 1}"
    new_text = text[: m.start()] + replacement + text[m.end() :]
    return new_text, idx


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------


def gather_outline(j, buf, world, target_points=45000):
    """A coarse point cloud of the whole exterior, used only to draw an
    outline in the review PNGs -- not used for any placement decision. Takes
    a small, even sample from every sizeable mesh so the silhouette covers
    wings/tail/engines too, not just whichever meshes happen to come first."""
    names = [n.get("name", "") for n in j["nodes"]]
    skip_words = ("FAN_BLUR", "CONTRAIL", "VFX", "FX_", "SOUND_", "LIGHT_")
    mesh_to_nodes = build_mesh_to_nodes(j)

    candidates = []  # (mesh_idx, pidx, node_idx)
    for mesh_idx, mesh in enumerate(j["meshes"]):
        for prim in mesh["primitives"]:
            pidx = prim["attributes"].get("POSITION")
            if pidx is None:
                continue
            a = j["accessors"][pidx]
            if a["count"] < 40:
                continue
            for node_idx in mesh_to_nodes.get(mesh_idx, [])[:1]:
                name = names[node_idx]
                if any(w in name.upper() for w in skip_words):
                    continue
                candidates.append((mesh_idx, pidx, node_idx))
            break  # one primitive per mesh is enough for a silhouette
    if not candidates:
        return np.zeros((0, 3))
    per_mesh = max(4, target_points // max(len(candidates), 1))
    pts = []
    for mesh_idx, pidx, node_idx in candidates:
        v_local = accessor_np(j, buf, pidx)
        step = max(1, len(v_local) // per_mesh)
        sample = v_local[::step]
        pts.append(transform_points(world[node_idx], sample))
    return np.vstack(pts) if pts else np.zeros((0, 3))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--gltf", required=True, type=Path)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--acf", type=Path, default=None, help="Append an _obja entry for the walkaround object to this .acf (off by default).")
    ap.add_argument("--gear-acf", type=Path, required=True, help="Read the gear contact points from this .acf (read only).")
    args = ap.parse_args()

    out_dir = Path(args.out)
    (out_dir / "objects").mkdir(parents=True, exist_ok=True)

    print(f"Loading {args.gltf} ...")
    j, buf = load_gltf(args.gltf)
    print(f"  {len(j['nodes'])} nodes, {len(j['meshes'])} meshes, {len(j['materials'])} materials")

    acf_gear = read_acf_gear(args.gear_acf)
    print(f"  .acf gear: {sorted(acf_gear)}")
    items, notes, cover_stats = extract_all(j, buf, acf_gear)
    print(f"Extracted {len(items)} geometry items ({sum(len(i.pieces) for i in items)} pieces)")

    tex_png = out_dir / "objects" / "a380_walkaround.png"
    uv_regions = build_atlas(tex_png)

    obj_path = out_dir / "objects" / "a380_walkaround.obj"
    n_pts, n_idx = write_obj8(items, uv_regions, obj_path, "a380_walkaround.png")
    print(f"Wrote {obj_path} ({n_pts} points, {n_idx} indices)")

    ok, report = verify_obj(obj_path, n_pts, n_idx)
    for line in report:
        print(" ", line)
    if not ok:
        print("VERIFICATION FAILED -- see above")

    print("Gathering exterior outline for review PNGs ...")
    world = build_world_matrices(j)
    outline = gather_outline(j, buf, world)
    render_views(items, outline, out_dir)
    print(f"Wrote {out_dir/'walkaround_top.png'} and {out_dir/'walkaround_side.png'}")

    print("\nItem table (glTF/world frame position -> feature, source):")
    for it in sorted(items, key=lambda x: x.id):
        c = None
        for p in it.pieces:
            if "center" in p:
                c = p["center"]
                break
        if c is None and it.pieces and "attach" in it.pieces[0]:
            c = it.pieces[0]["attach"]
        cs = np.round(c, 3).tolist() if c is not None else "n/a"
        print(f"  {it.id:22s} pos={cs}  feature={it.feature}")
        print(f"      source: {it.source}")

    if notes:
        print("\nItems/notes that could not be placed or need review:")
        for n in notes:
            print("  -", n)

    if cover_stats:
        print("\nEngine cover fit (fitted dome+skirt+band, min clearance to the real nacelle/core surface):")
        for cs_ in cover_stats:
            conf = "confident" if cs_["confident_cowl_band"] else "LOW CONFIDENCE (cowl band under-sampled)"
            print(f"  {cs_['id']:22s} min clearance {cs_['clearance_m']*100:5.1f} cm  tris {cs_['n_tris']:5d}  {conf}")

    if args.acf:
        print(f"\n--acf given: appending an _obja entry to {args.acf}")
        text, idx = append_obja(args.acf, "a380_walkaround.obj")
        args.acf.write_text(text, encoding="utf-8")
        print(f"  appended _obja/{idx}/* (flags=2064: outside-shadows-only | clickable)")

    print("\nDone.")


if __name__ == "__main__":
    main()
