#!/usr/bin/env python3
"""
Build the FlyByWire A380X (X-Plane 12 port) livery paint kit -- second pass.

Reads the converted aircraft's OBJ8 models and textures from the X-Plane
install (read-only) and (re)writes the whole paint kit to an output folder
that is NOT inside the X-Plane install. The kit folder is entirely
generated; re-running this script is the supported way to update it.

Usage:
    python make_paintkit.py [--aircraft PATH] [--out PATH] [--only NAME,...]

See README.md next to this script for how this fits together, and the
kit's own README.md (generated) for what painters need to know.
"""
from __future__ import annotations

import argparse
import re
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from pytoshop import enums
import pytoshop.util as _ptutil
from pytoshop.image_data import ImageData
from pytoshop.user import nested_layers as nl

import dds as dds_mod

# ---------------------------------------------------------------------------
# pytoshop writes Photoshop's Unicode-string layer-name field WITH an extra
# null terminator baked into the declared length (Adobe's own documented
# convention for this one field). Every reader is supposed to strip it, and
# pytoshop's own decoder does -- but the psd-tools library used to spot
# check this kit's output does not, so `psd.composite()` and layer listings
# showed names ending in a literal "\x00". Rather than trust every painter's
# tool to strip it, write the field WITHOUT the extra terminator (a plain
# length-prefixed UTF-16BE string, which is what most other PSD writers
# emit and what every reader that follows the declared length -- not a null
# scan -- reads identically).
def _encode_unicode_string_no_terminator(s):
    return struct.pack(">L", len(s)) + s.encode("utf_16_be")


_ptutil.encode_unicode_string = _encode_unicode_string_no_terminator

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

DEFAULT_AIRCRAFT = Path(r"D:/Steam Games/steamapps/common/X-Plane 12/Aircraft/FlyByWire A380X")
DEFAULT_OUT = Path(r"D:/FlyByWire A380X Paint Kit")
TARGET_DIR_NAME = "objects"
FONT_PATH = Path(r"C:/Windows/Fonts/arial.ttf")

LIVERIES = {
    "Emirates": "Airbus A380 Emirates A6-EVG (4k)",
    "Pride": "Pride A380X",
}

# ---- Texture groups (section numbering matches the kit README) -----------

GROUP_MAIN = [
    "A380X_FUSE1_ALBEDO_KE7E7E7.dds",
    "A380X_FUSE2_ALBEDO_KE7E7E7.dds",
    "A380X_FUSE3_ALBEDO_KE7E7E7.dds",
    "A380X_FUSE4_ALBEDO_KE7E7E7.dds",
    "A380X_FUSE5_ALBEDO_KE7E7E7.dds",
    "A380_EXTERIOR_WING1_ALBEDO.dds",
    "A380_EXTERIOR_WING2_ALBEDO.dds",
    "A380X_EXTERIOR_WINGFENCE_ALBEDO_KE7E7E7.dds",
    "A380_EXTERIOR_ENG_LH_ALBEDO.dds",
    "A380_EXTERIOR_ENG_RH_ALBEDO.dds",
    "A380_EXTERTIOR_FAN_ALBEDO.dds",
]
GROUP_SECONDARY = [
    "A380_EXTERIOR_WING1_2_ALBEDO_KE7E7E7.dds",
    "A380_EXTERIOR_WING2_2_ALBEDO_KE7E7E7.dds",
    "A380_EXTERTIOR_PYLON_ALBEDO.dds",
    "A380_EXTERIOR_ENG_CORES_LH_ALBEDO.dds",
    "A380_EXTERIOR_ENG_CORES_RH_ALBEDO.dds",
    "A380_EXTERIOR_MISC_ALBEDO.dds",
    "A380X_ACCESSORIES_ALBEDO_KE7E7E7.dds",
    "A380X_ACCESSORIES_ALBEDO_KE7E7E7_A25.dds",
    "A380X_PAX_DOORS_ALBEDO_KE7E7E7.dds",
]
GROUP_GEAR = [
    "A380_EXTERIOR_NLG_ALBEDO_KE7E7E7.dds",
    "A380X_BLG_ALBEDO_KE7E7E7.dds",
    "A380X_WLG_ALBEDO_KE7E7E7.dds",
    "A380X_LG_BAY_MAIN_ALBEDO_KE7E7E7.dds",
    "TIRES_ALB.dds",
]
GROUP_DECALS = [
    "A380_PAX_WINDOW_ALBEDO_KE7E7E7_DECAL.dds",
    "A380_DECAL_DOOR_ALBEDO_KE7E7E7_DECAL.dds",
    "FWD_CARGO_DECALS_DECAL.dds",
    "CARGO_FWD.dds",
    "A380_DETAIL_RIBBON01_ALBEDO_DECAL.dds",
    "A380_EXTERIOR_ICE_BASECOLOR_KE7E7E7_DECAL.dds",
    "A380X_REGISTRATION_ALBEDO_KE7E7E7_DECAL.dds",
]
GROUP_EFFECTS = [
    "ENGINE_BLUR1_ALBEDO_KE7E7E7.dds",
    "ENGINE_BLUR1_ALBEDO_KE7E7E7_DECAL.dds",
    "ENGINE_BLUR2_ALBEDO_KE7E7E7.dds",
    "ENGINE_BLUR2_ALBEDO_KE7E7E7_DECAL.dds",
    "ENGINE_BLUR3_ALBEDO_KE7E7E7.dds",
    "ENGINE_BLUR3_ALBEDO_KE7E7E7_DECAL.dds",
    "ENGINE_BLUR4_ALBEDO_KE7E7E7.dds",
    "ENGINE_BLUR4_ALBEDO_KE7E7E7_DECAL.dds",
    "RAT_BLUR_3_DECAL.dds",
]
GROUP_GLASS = [
    "A380X_LIGHT_GLASS_ALBEDO_MASK80.dds",
    "A380X_LIGHT_GLASS_ALBEDO_KE7E7E7_A25.dds",
]
SOLID_TEXTURES = ["solid_000000ff.png", "solid_161616ff.png", "solid_e6e6e6ff.png"]

TEXTURE_GROUPS = [
    ("Group 1: Main livery sheets", GROUP_MAIN),
    ("Group 2: Secondary structure", GROUP_SECONDARY),
    ("Group 3: Landing gear and bays", GROUP_GEAR),
    ("Group 4: Decals", GROUP_DECALS),
    ("Group 5: Effects", GROUP_EFFECTS),
    ("Group 6: Glass and lights", GROUP_GLASS),
]
ALL_FULL_TREATMENT = [t for _, g in TEXTURE_GROUPS for t in g]

# Textures the example livery repaints from a rasterised 3-D position map
# (task G); also the sheets that get a "Height guides" PSD layer, since
# both need the same per-texel position raster and this keeps the cost of
# that raster off very large meshes (TIRES_ALB alone has ~250k triangles).
POSITION_MAP_TEXTURES = list(GROUP_MAIN) + ["A380X_PAX_DOORS_ALBEDO_KE7E7E7.dds"]
EXAMPLE_LIVERY_TEXTURES = [
    "A380X_FUSE1_ALBEDO_KE7E7E7.dds",
    "A380X_FUSE2_ALBEDO_KE7E7E7.dds",
    "A380X_FUSE3_ALBEDO_KE7E7E7.dds",
    "A380X_FUSE4_ALBEDO_KE7E7E7.dds",
    "A380X_FUSE5_ALBEDO_KE7E7E7.dds",
    "A380X_PAX_DOORS_ALBEDO_KE7E7E7.dds",
]

# Sheets whose island labels come from a fixed, texture-identity label
# rather than the position-based classifier -- the classifier's rules are
# tuned for the fuselage/wing/empennage/engine taxonomy and would misfire
# on a landing gear leg or a tiny decal.
TEXTURE_LABEL_OVERRIDE = {
    "A380_EXTERIOR_NLG_ALBEDO_KE7E7E7.dds": "Nose landing gear",
    "A380X_BLG_ALBEDO_KE7E7E7.dds": "Body landing gear",
    "A380X_WLG_ALBEDO_KE7E7E7.dds": "Wing landing gear",
    "A380X_LG_BAY_MAIN_ALBEDO_KE7E7E7.dds": "Landing gear bay",
    "TIRES_ALB.dds": "Tyre",
    "A380_EXTERIOR_ENG_CORES_LH_ALBEDO.dds": "Engine core LH",
    "A380_EXTERIOR_ENG_CORES_RH_ALBEDO.dds": "Engine core RH",
    "A380_PAX_WINDOW_ALBEDO_KE7E7E7_DECAL.dds": "Cabin window band",
    "A380_DECAL_DOOR_ALBEDO_KE7E7E7_DECAL.dds": "Door decal",
    "FWD_CARGO_DECALS_DECAL.dds": "Cargo door decal",
    "CARGO_FWD.dds": "Forward cargo door",
    "A380_DETAIL_RIBBON01_ALBEDO_DECAL.dds": "Rivet / panel ribbon detail",
    "A380_EXTERIOR_ICE_BASECOLOR_KE7E7E7_DECAL.dds": "Ice detector placard (flight deck)",
    "A380X_REGISTRATION_ALBEDO_KE7E7E7_DECAL.dds": "Registration placard (flight deck)",
    "RAT_BLUR_3_DECAL.dds": "RAT (Ram Air Turbine) blur disc",
    "A380X_LIGHT_GLASS_ALBEDO_MASK80.dds": "Light lens / glass",
    "A380X_LIGHT_GLASS_ALBEDO_KE7E7E7_A25.dds": "Light lens / glass",
    "A380X_ACCESSORIES_ALBEDO_KE7E7E7.dds": "Exterior accessory",
    "A380X_ACCESSORIES_ALBEDO_KE7E7E7_A25.dds": "Exterior accessory",
    "A380_EXTERIOR_MISC_ALBEDO.dds": "Exterior misc part",
}
for _n in range(1, 5):
    TEXTURE_LABEL_OVERRIDE[f"ENGINE_BLUR{_n}_ALBEDO_KE7E7E7.dds"] = f"Engine {_n} fan blur disc"
    TEXTURE_LABEL_OVERRIDE[f"ENGINE_BLUR{_n}_ALBEDO_KE7E7E7_DECAL.dds"] = f"Engine {_n} fan blur disc"

SHORT_NAME = {
    "A380X_FUSE1_ALBEDO_KE7E7E7.dds": "FUSE1",
    "A380X_FUSE2_ALBEDO_KE7E7E7.dds": "FUSE2",
    "A380X_FUSE3_ALBEDO_KE7E7E7.dds": "FUSE3",
    "A380X_FUSE4_ALBEDO_KE7E7E7.dds": "FUSE4",
    "A380X_FUSE5_ALBEDO_KE7E7E7.dds": "FUSE5",
    "A380_EXTERIOR_WING1_ALBEDO.dds": "WING1",
    "A380_EXTERIOR_WING2_ALBEDO.dds": "WING2",
    "A380X_EXTERIOR_WINGFENCE_ALBEDO_KE7E7E7.dds": "WINGFENCE",
    "A380_EXTERIOR_ENG_LH_ALBEDO.dds": "ENG_LH",
    "A380_EXTERIOR_ENG_RH_ALBEDO.dds": "ENG_RH",
    "A380_EXTERTIOR_FAN_ALBEDO.dds": "FAN",
}


def short_name(tex: str) -> str:
    if tex in SHORT_NAME:
        return SHORT_NAME[tex]
    stem = Path(tex).stem
    for marker in ("_ALBEDO", "_BASECOLOR"):
        idx = stem.find(marker)
        if idx != -1:
            stem = stem[:idx]
            break
    return stem


# ---------------------------------------------------------------------------
# OBJ8 parsing
# ---------------------------------------------------------------------------


@dataclass
class ObjMesh:
    path: Path
    texture: str
    verts: np.ndarray  # (N, 8): x,y,z, nx,ny,nz, u,v
    tris: np.ndarray  # (M, 3) int32 vertex indices
    dropped_uv_triangles: int = 0


_TEXTURE_RE = re.compile(r"^TEXTURE\s+(\S+)", re.IGNORECASE)
MAX_UV_EDGE_FRACTION = 0.25
UV_RANGE_MARGIN = 0.01


def scan_texture_map(objects_dir: Path) -> dict[str, list[Path]]:
    mapping: dict[str, list[Path]] = {}
    files = sorted(objects_dir.glob("*.obj"))
    for obj_path in files:
        with open(obj_path, "r", encoding="utf-8", errors="replace") as f:
            for line in f:
                m = _TEXTURE_RE.match(line.strip())
                if m:
                    mapping.setdefault(m.group(1), []).append(obj_path)
                    break
    return mapping


def parse_obj(path: Path) -> ObjMesh:
    """Parse one OBJ8 file: its declared texture, its VT vertex table, and
    its full triangle list (every consecutive triple of the global IDX/IDX10
    index buffer). Triangles with a UV edge longer than
    `MAX_UV_EDGE_FRACTION` of the sheet, or any UV vertex outside
    [-UV_RANGE_MARGIN, 1+UV_RANGE_MARGIN], are dropped (FUSE5 in particular
    has a batch of sliver triangles whose UVs stretch far outside 0..1;
    every other sheet checked cleanly)."""
    texture = ""
    verts: list[list[float]] = []
    idx: list[int] = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("TEXTURE ") and not texture:
                texture = line.split(None, 1)[1].strip()
            elif line.startswith("VT "):
                parts = line.split()
                verts.append([float(x) for x in parts[1:9]])
            elif line.startswith("IDX10 "):
                idx.extend(int(x) for x in line.split()[1:11])
            elif line.startswith("IDX "):
                idx.append(int(line.split()[1]))
    v = np.asarray(verts, dtype=np.float64) if verts else np.zeros((0, 8))
    n_tris = len(idx) // 3
    t = np.asarray(idx[: n_tris * 3], dtype=np.int64).reshape(-1, 3) if n_tris else np.zeros((0, 3), dtype=np.int64)

    dropped = 0
    if len(t):
        uv = v[:, 6:8]
        tri_uv = uv[t]  # (M,3,2)
        out_of_range = np.any((tri_uv < -UV_RANGE_MARGIN) | (tri_uv > 1 + UV_RANGE_MARGIN), axis=(1, 2))
        e0 = tri_uv[:, 1] - tri_uv[:, 0]
        e1 = tri_uv[:, 2] - tri_uv[:, 1]
        e2 = tri_uv[:, 0] - tri_uv[:, 2]
        len0 = np.linalg.norm(e0, axis=1)
        len1 = np.linalg.norm(e1, axis=1)
        len2 = np.linalg.norm(e2, axis=1)
        stretched = (len0 > MAX_UV_EDGE_FRACTION) | (len1 > MAX_UV_EDGE_FRACTION) | (len2 > MAX_UV_EDGE_FRACTION)
        uv_flagged = out_of_range | stretched

        # A UV-edge/out-of-range flag alone is not enough: several sheets
        # (a ribbon decal that deliberately tiles its UV past 1.0, a light
        # lens whose one or two quads legitimately span a large fraction of
        # a small atlas) have large, VALID triangles that trip this same
        # test. The real defect this exists to catch is a SLIVER: near-zero
        # 3-D area with a UV footprint wildly out of proportion to it (the
        # FUSE5 case, ~0.00045 sq. m average over its 88-ish sliver
        # triangles). Requiring both conditions keeps the large, legitimate
        # triangles and drops only the genuine slivers -- checked directly
        # against every sheet this run flagged before shipping this filter.
        P0, P1, P2 = v[t[:, 0], 0:3], v[t[:, 1], 0:3], v[t[:, 2], 0:3]
        area3d = 0.5 * np.linalg.norm(np.cross(P1 - P0, P2 - P0), axis=1)
        SLIVER_AREA_M2 = 0.002
        bad = uv_flagged & (area3d < SLIVER_AREA_M2)
        dropped = int(bad.sum())
        if dropped:
            t = t[~bad]

    return ObjMesh(path=path, texture=texture, verts=v, tris=t, dropped_uv_triangles=dropped)


def find_islands(mesh: ObjMesh) -> list[np.ndarray]:
    n = len(mesh.verts)
    parent = np.arange(n)

    def find(x: int) -> int:
        root = x
        while parent[root] != root:
            root = parent[root]
        while parent[x] != root:
            parent[x], x = root, parent[x]
        return root

    def union(a: int, b: int) -> None:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    for tri in mesh.tris:
        union(int(tri[0]), int(tri[1]))
        union(int(tri[1]), int(tri[2]))

    groups: dict[int, list[int]] = {}
    used = np.unique(mesh.tris.reshape(-1)) if len(mesh.tris) else np.array([], dtype=np.int64)
    for vi in used:
        r = find(int(vi))
        groups.setdefault(r, []).append(int(vi))
    return [np.asarray(g, dtype=np.int64) for g in groups.values()]


# ---------------------------------------------------------------------------
# Aircraft reference geometry + absolute-position classifier
# ---------------------------------------------------------------------------


@dataclass
class AircraftRef:
    nose_z: float
    fuse1_z1: float
    fuse2_z1: float
    fuse4_z0: float
    wing_z0: float
    wing_z1: float
    eng_z0: float
    eng_z1: float


def compute_reference(objects_dir: Path, texmap: dict[str, list[Path]]) -> AircraftRef:
    def mesh_for(tex: str) -> ObjMesh:
        paths = texmap.get(tex, [])
        return parse_obj(paths[0]) if paths else ObjMesh(path=Path(), texture=tex, verts=np.zeros((0, 8)), tris=np.zeros((0, 3), dtype=np.int64))

    fuse1 = mesh_for("A380X_FUSE1_ALBEDO_KE7E7E7.dds")
    fuse2 = mesh_for("A380X_FUSE2_ALBEDO_KE7E7E7.dds")
    fuse4 = mesh_for("A380X_FUSE4_ALBEDO_KE7E7E7.dds")
    wing1 = mesh_for("A380_EXTERIOR_WING1_ALBEDO.dds")
    eng_lh = mesh_for("A380_EXTERIOR_ENG_LH_ALBEDO.dds")

    all_z = [m.verts[:, 2] for m in (fuse1, fuse2, fuse4, wing1, eng_lh) if len(m.verts)]
    nose_z = float(min(z.min() for z in all_z)) if all_z else -35.0

    return AircraftRef(
        nose_z=nose_z,
        fuse1_z1=float(fuse1.verts[:, 2].max()) if len(fuse1.verts) else -15.0,
        fuse2_z1=float(fuse2.verts[:, 2].max()) if len(fuse2.verts) else 8.0,
        fuse4_z0=float(fuse4.verts[:, 2].min()) if len(fuse4.verts) else 18.0,
        wing_z0=float(wing1.verts[:, 2].min()) if len(wing1.verts) else -17.0,
        wing_z1=float(wing1.verts[:, 2].max()) if len(wing1.verts) else 17.0,
        eng_z0=float(eng_lh.verts[:, 2].min()) if len(eng_lh.verts) else -13.0,
        eng_z1=float(eng_lh.verts[:, 2].max()) if len(eng_lh.verts) else 4.0,
    )


def engine_pylon_number(x: float, ax: float) -> int:
    """A380 engine/pylon numbering: 1 = outer left, 2 = inner left,
    3 = inner right, 4 = outer right (outer = the larger |x|, split at the
    midpoint between the inboard and outboard nacelle clusters, ~|x| 20)."""
    outboard = ax > 20.0
    return (1 if outboard else 2) if x < 0 else (4 if outboard else 3)


def classify_point(x: float, y: float, z: float, ny: float, ref: AircraftRef) -> str:
    """Absolute-aircraft-position taxonomy (task C). Thresholds for the
    empennage split (|x|>5, y<8 => horizontal stabiliser) are exactly the
    ones the coordinator hand-verified against FUSE4's vertex bounding box."""
    ax = abs(x)
    side = "L" if x < -0.15 else ("R" if x > 0.15 else "C")

    if z >= ref.fuse4_z0 - 0.5:
        if ax > 5.0 and y < 8.0:
            ud = "upper" if ny > 0.25 else ("lower" if ny < -0.25 else "")
            return f"Horizontal stabiliser {side} {ud}".strip()
        if y >= 8.0:
            return f"Vertical fin / rudder{' ' + side if ax > 1.2 else ''}"
        return "Tail cone (rear fuselage)"

    if z <= ref.nose_z + 3.0:
        return "Nose / radome"

    # Engine checked before the (much broader) wing rule: engines sit low
    # and close to the wing underside, well within the wing's own z-span,
    # so wing would otherwise claim them first. (Pylons are handled by a
    # dedicated override -- see engine_pylon_number/TEXTURE_LABEL_OVERRIDE
    # -- rather than by position here: a pylon's own footprint sits so
    # close to its engine's that no position-only rule cleanly separates
    # the two, and the pylon texture is its own dedicated sheet anyway.)
    if ax > 8.0 and (ref.eng_z0 - 1.0) <= z <= (ref.eng_z1 + 1.5) and y < 4.5:
        return f"Engine {engine_pylon_number(x, ax)} nacelle"

    if ax > 4.3 and (ref.wing_z0 - 1.0) <= z <= (ref.wing_z1 + 1.0):
        if ax > 34.0:
            return f"Wingtip fence {side}"
        ud = "upper" if ny > 0.15 else "lower"
        return f"Wing {side} {ud}"

    band = "belly" if y < 0.6 else "upper"
    if z <= ref.fuse1_z1:
        station = "Forward fuselage"
    elif z <= ref.fuse2_z1:
        station = "Mid fuselage"
    else:
        station = "Aft fuselage"
    if band == "belly":
        return f"Belly fairing ({station.lower()})"
    return f"{station} {side} {band}"


def sheet_geometry_description(tex: str, meshes: list[ObjMesh], ref: AircraftRef) -> tuple[str, dict[str, int]]:
    """Compute a station-range + dominant-parts description straight from
    the mesh's own vertices, plus a label->count histogram used both for
    the description and for the FUSE4 fin/stabiliser assertion."""
    if not meshes or all(len(m.verts) == 0 for m in meshes):
        return "(no geometry found)", {}

    xs = np.concatenate([m.verts[:, 0] for m in meshes])
    ys = np.concatenate([m.verts[:, 1] for m in meshes])
    zs = np.concatenate([m.verts[:, 2] for m in meshes])
    nys = np.concatenate([m.verts[:, 4] for m in meshes])

    step = max(1, len(xs) // 4000)  # sample for the histogram; stations/sides use the full set
    hist: dict[str, int] = {}
    for x, y, z, ny in zip(xs[::step], ys[::step], zs[::step], nys[::step]):
        if tex == "A380_EXTERTIOR_PYLON_ALBEDO.dds":
            label = f"Pylon {engine_pylon_number(float(x), abs(float(x)))}"
        else:
            label = classify_point(float(x), float(y), float(z), float(ny), ref)
        hist[label] = hist.get(label, 0) + 1

    station0 = float(zs.min()) - ref.nose_z
    station1 = float(zs.max()) - ref.nose_z
    has_left = bool(np.any(xs < -0.2))
    has_right = bool(np.any(xs > 0.2))
    side = "both sides" if has_left and has_right else ("left side" if has_left else ("right side" if has_right else "centreline"))
    top_labels = sorted(hist.items(), key=lambda kv: -kv[1])[:3]
    top_str = ", ".join(f"{lbl} ({cnt / sum(hist.values()) * 100:.0f}%)" for lbl, cnt in top_labels) if hist else "n/a"

    desc = f"Station {station0:.1f}-{station1:.1f} m aft of the nose, {side}, y {ys.min():.1f}..{ys.max():.1f} m. Mainly: {top_str}."
    return desc, hist


def assert_fuse4_has_fin_and_stabiliser(meshes: list[ObjMesh]) -> str:
    """Hard check (task A): the script must fail if FUSE4 does not in fact
    contain both the horizontal stabiliser and the vertical fin, by the
    coordinator's own hand-verified rule (|x|>5 and y<8 => stabiliser;
    y>=8 => fin)."""
    if not meshes:
        raise AssertionError("FUSE4: no mesh found at all")
    xs = np.concatenate([m.verts[:, 0] for m in meshes])
    ys = np.concatenate([m.verts[:, 1] for m in meshes])
    stab = int(np.sum((np.abs(xs) > 5.0) & (ys < 8.0)))
    fin = int(np.sum(ys >= 8.0))
    if stab < 10000:
        raise AssertionError(f"FUSE4 geometry check failed: only {stab} vertices match the stabiliser rule (|x|>5,y<8); expected ~39000")
    if fin == 0:
        raise AssertionError("FUSE4 geometry check failed: no vertices with y>=8 (vertical fin) found")
    return f"FUSE4 verified: {stab} vertices match horizontal-stabiliser rule, {fin} match vertical-fin rule (y>=8)."


# ---------------------------------------------------------------------------
# Texture I/O
# ---------------------------------------------------------------------------


def load_rgba(path: Path) -> np.ndarray:
    im = Image.open(path).convert("RGBA")
    return np.array(im)


def save_png(arr: np.ndarray, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(arr, "RGBA").save(path)


def resize_to(arr: np.ndarray, w: int, h: int) -> np.ndarray:
    if arr.shape[1] == w and arr.shape[0] == h:
        return arr
    return np.array(Image.fromarray(arr, "RGBA").resize((w, h), Image.LANCZOS))


def uv_to_px(u: float, v: float, w: int, h: int) -> tuple[float, float]:
    """OBJ v=0 is the bottom of the texture; image row 0 is the top."""
    return u * w, (1.0 - v) * h


# ---------------------------------------------------------------------------
# Coverage mask (fast, PIL polygon fill) -- used by every sheet
# ---------------------------------------------------------------------------


def rasterize_coverage(mesh: ObjMesh, w: int, h: int) -> np.ndarray:
    img = Image.new("L", (w, h), 0)
    draw = ImageDraw.Draw(img)
    uv = mesh.verts[:, 6:8]
    for tri in mesh.tris:
        pts = [uv_to_px(uv[i, 0], uv[i, 1], w, h) for i in tri]
        draw.polygon(pts, fill=255)
    return np.array(img) > 0


def dilate_bool(mask: np.ndarray, px: int) -> np.ndarray:
    from scipy.ndimage import binary_dilation

    return binary_dilation(mask, iterations=px)


# ---------------------------------------------------------------------------
# Full 3-D position map (barycentric raster) -- only for POSITION_MAP_TEXTURES
# ---------------------------------------------------------------------------


def rasterize_position_map(mesh: ObjMesh, w: int, h: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Returns (coverage bool[h,w], pos float32[h,w,3], normal_y float32[h,w])."""
    uv = mesh.verts[:, 6:8]
    pos = mesh.verts[:, 0:3]
    nrm = mesh.verts[:, 3:6]
    coverage = np.zeros((h, w), dtype=bool)
    posmap = np.zeros((h, w, 3), dtype=np.float32)
    normal_y = np.zeros((h, w), dtype=np.float32)

    px_all = uv[:, 0] * w
    py_all = (1.0 - uv[:, 1]) * h

    for tri in mesh.tris:
        i0, i1, i2 = int(tri[0]), int(tri[1]), int(tri[2])
        x0, y0 = px_all[i0], py_all[i0]
        x1, y1 = px_all[i1], py_all[i1]
        x2, y2 = px_all[i2], py_all[i2]
        minx = max(int(np.floor(min(x0, x1, x2))), 0)
        maxx = min(int(np.ceil(max(x0, x1, x2))), w - 1)
        miny = max(int(np.floor(min(y0, y1, y2))), 0)
        maxy = min(int(np.ceil(max(y0, y1, y2))), h - 1)
        if maxx < minx or maxy < miny:
            continue
        denom = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
        if abs(denom) < 1e-9:
            continue
        gx, gy = np.meshgrid(np.arange(minx, maxx + 1) + 0.5, np.arange(miny, maxy + 1) + 0.5)
        w0 = ((y1 - y2) * (gx - x2) + (x2 - x1) * (gy - y2)) / denom
        w1 = ((y2 - y0) * (gx - x2) + (x0 - x2) * (gy - y2)) / denom
        w2 = 1.0 - w0 - w1
        inside = (w0 >= -1e-3) & (w1 >= -1e-3) & (w2 >= -1e-3)
        if not inside.any():
            continue
        P = w0[..., None] * pos[i0] + w1[..., None] * pos[i1] + w2[..., None] * pos[i2]
        NY = w0 * nrm[i0, 1] + w1 * nrm[i1, 1] + w2 * nrm[i2, 1]
        sub_cov = coverage[miny : maxy + 1, minx : maxx + 1]
        sub_pos = posmap[miny : maxy + 1, minx : maxx + 1]
        sub_ny = normal_y[miny : maxy + 1, minx : maxx + 1]
        sub_pos[inside] = P[inside].astype(np.float32)
        sub_ny[inside] = NY[inside].astype(np.float32)
        sub_cov |= inside

    return coverage, posmap, normal_y


# ---------------------------------------------------------------------------
# Panel lines & detail (task E: multi-livery max, coverage-masked blur)
# ---------------------------------------------------------------------------


TOPHAT_GAIN = 2.5
TOPHAT_FLOOR = 0.3


def _black_tophat_factor(lum: np.ndarray, sheet_width: int, gain: float, floor: float) -> np.ndarray:
    """A morphological black top-hat (`grey_closing(L) - L`) responds only
    to dark features narrower than the closing kernel: panel seams, rivets,
    door outlines, drain holes. A large field of colour, a thick-stroked
    wordmark, or a big shattered-triangle graphic all close back to
    themselves (or close over each other) and give ~0 top-hat, so they do
    not darken this layer at all -- unlike a blurred local-average ratio,
    which large artwork *always* perturbs, however the different livery
    versions are then combined. Kernel scales with sheet width (5 px at a
    2048 sheet), forced odd and at least 3."""
    from scipy.ndimage import grey_closing

    k = max(3, int(round(5 * sheet_width / 2048)))
    if k % 2 == 0:
        k += 1
    closed = grey_closing(lum, size=k)
    tophat = np.clip(closed - lum, 0, None)
    return np.clip(1.0 - gain * (tophat / 255.0), floor, 1.0)


def gather_livery_refs(aircraft: Path, tex_name: str, w: int, h: int) -> dict[str, np.ndarray]:
    """Every shipped livery's version of this exact texture file, resized to
    (w, h). Used both for the PSD's "Reference - <livery>" layers and to
    feed the detail layer's cross-livery masking -- the two need the same
    set, and building it in only one of the two call sites was the original
    bug behind the example livery still showing "Spirit of Toulouse" and
    the shattered-triangle mosaic: with only the house texture to compare
    against, the black top-hat has nothing to mask a livery-only thin dark
    feature with, and both of those are genuinely thin and dark enough to
    register (script text; small individual mosaic facets), so they survive
    exactly as designed for a single-livery sheet."""
    refs: dict[str, np.ndarray] = {}
    for livery_name, folder in LIVERIES.items():
        lv_path = aircraft / "liveries" / folder / TARGET_DIR_NAME / tex_name
        if lv_path.exists():
            refs[livery_name] = resize_to(load_rgba(lv_path), w, h)
    return refs


MSFS_PACKAGE = Path(r"D:/Microsoft Flight Simulator 2020/Microsoft Flight Simulator 2020 Packages/Community/flybywire-aircraft-a380-842")
MSFS_TEXTURE_DIR = MSFS_PACKAGE / "SimObjects" / "AirPlanes" / "FlyByWire_A380_842" / "texture"


def find_msfs_roughness_source(tex_name: str) -> Path | None:
    """The ORIGINAL MSFS roughness map for this sheet (before the converter
    ever ran), livery-independent by construction since a livery never
    touches it. G channel only -- R is occlusion (baked black for the
    radome, not usable as a weathering cue), B is metalness."""
    if not MSFS_TEXTURE_DIR.is_dir():
        return None
    stem = Path(tex_name).stem
    for marker in ("_ALBEDO", "_BASECOLOR"):
        idx = stem.find(marker)
        if idx != -1:
            stem = stem[:idx]
            break
    if stem.endswith("_ALB"):
        stem = stem[:-4]
    for suffix in ("_METAL.PNG.DDS", "_COMP.PNG.DDS", "_OCCLUSIONROUGHNESSMETALLIC.PNG.DDS"):
        hit = list(MSFS_TEXTURE_DIR.glob(f"{stem}{suffix}"))
        if hit:
            return hit[0]
    return None


def make_panel_detail_multi(
    sheet_size: tuple[int, int],
    liveries_rgba: list[np.ndarray],
    coverage: np.ndarray,
    roughness_source: Path | None = None,
) -> np.ndarray:
    """Rebuilt detail layer: per-livery morphological black top-hat (thin
    dark features only -- seams, rivets, door outlines), combined across
    every available livery version of this sheet by taking the per-pixel
    MAXIMUM factor (the LEAST darkening) so a thin feature only one livery
    carries (the house's "Spirit of Toulouse" script, say) is erased by any
    other livery reading flat (factor 1) at that same spot, while a seam
    every livery shares survives in all of them. Large-area artwork -- a
    wordmark however thick-stroked, a shattered-triangle graphic, a whole
    paint field -- never enters the top-hat at all, so it does not need
    cross-livery cancellation the way the earlier blurred-ratio approach
    did (and that approach still leaked large artwork through no matter how
    the liveries were combined; replaced outright rather than re-tuned).
    Optionally multiplies in the ORIGINAL MSFS roughness map's G channel
    (livery-independent) as a gentle weathering term."""
    h, w = sheet_size

    factors = []
    for rgba in liveries_rgba:
        rgba_r = resize_to(rgba, w, h) if rgba.shape[:2] != (h, w) else rgba
        rgb = rgba_r[:, :, :3].astype(np.float32)
        lum = rgb[:, :, 0] * 0.299 + rgb[:, :, 1] * 0.587 + rgb[:, :, 2] * 0.114
        factors.append(_black_tophat_factor(lum, w, TOPHAT_GAIN, TOPHAT_FLOOR))

    combined = np.maximum.reduce(factors) if factors else np.ones((h, w), dtype=np.float32)

    if roughness_source is not None:
        try:
            rough = np.array(Image.open(roughness_source).convert("RGBA"))
            rough_g = resize_to(rough, w, h)[:, :, 1].astype(np.float32) / 255.0
            weather = 1.0 - 0.1 * rough_g  # rougher -> up to 10% darker
            combined = combined * weather
        except Exception:
            pass

    gray = np.clip(combined * 255.0, 0, 255).astype(np.uint8)
    cov_dilated = dilate_bool(coverage, 4) if coverage.any() else coverage
    gray = np.where(cov_dilated, gray, 255)

    out = np.zeros((h, w, 4), dtype=np.uint8)
    out[:, :, 0] = gray
    out[:, :, 1] = gray
    out[:, :, 2] = gray
    out[:, :, 3] = 255
    return out


# ---------------------------------------------------------------------------
# Wireframe, mirror detection, FWD/UP arrows, labels
# ---------------------------------------------------------------------------

LARGE_MESH_TRIS = 120_000
LARGE_BOUNDARY_EDGES = 60_000


def render_wireframe(mesh: ObjMesh, w: int, h: int, report: list[str], sheet_label: str) -> np.ndarray:
    img = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)
    n_tris = len(mesh.tris)
    if n_tris == 0:
        return np.array(img)

    edge_count: dict[tuple[int, int], int] = {}
    for tri in mesh.tris:
        for a, b in ((tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])):
            key = (int(a), int(b)) if a < b else (int(b), int(a))
            edge_count[key] = edge_count.get(key, 0) + 1

    uv = mesh.verts[:, 6:8]
    boundary = []
    interior = []
    for (a, b), cnt in edge_count.items():
        p1 = uv_to_px(uv[a][0], uv[a][1], w, h)
        p2 = uv_to_px(uv[b][0], uv[b][1], w, h)
        (boundary if cnt == 1 else interior).append((p1, p2))

    decimated = n_tris > LARGE_MESH_TRIS
    if decimated:
        report.append(f"WIREFRAME {sheet_label}: {n_tris} triangles > {LARGE_MESH_TRIS} -- interior edges skipped, boundary only")
        interior = []
    if len(boundary) > LARGE_BOUNDARY_EDGES:
        stride = max(2, len(boundary) // LARGE_BOUNDARY_EDGES)
        report.append(f"WIREFRAME {sheet_label}: {len(boundary)} boundary edges > {LARGE_BOUNDARY_EDGES} -- drawing every {stride}th")
        boundary = boundary[::stride]

    for p1, p2 in interior:
        draw.line([p1, p2], fill=(0, 230, 255, 150), width=1)
    for p1, p2 in boundary:
        draw.line([p1, p2], fill=(255, 200, 0, 230), width=2)

    return np.array(img)


def triangle_mirror_signs(mesh: ObjMesh, w: int, h: int) -> np.ndarray:
    """Per-triangle sign test (task C's exact formula):
        s3 = sign(N . (e1 x e2))            (3-D winding vs. outward normal)
        s2 = sign(cross2d of the same two edges in image space, y-down)
    Returns s2*s3 (>0 => triangle flagged 'mirrored' by this local test).

    NOTE (see the generation report / README): this local, per-triangle
    test cannot detect a *global* left/right mirror of a whole UV island
    (e.g. the aircraft's left and right fuselage sides, which are a
    correctly-wound mirror-image pair -- standard, correct mesh
    construction). It only catches an island whose OWN UV chart was
    reflected during atlas packing relative to its OWN triangle winding.
    """
    if len(mesh.tris) == 0:
        return np.zeros(0)
    P = mesh.verts[:, 0:3]
    N = mesh.verts[:, 3:6]
    uv = mesh.verts[:, 6:8]
    t = mesh.tris
    P0, P1, P2 = P[t[:, 0]], P[t[:, 1]], P[t[:, 2]]
    Nm = (N[t[:, 0]] + N[t[:, 1]] + N[t[:, 2]]) / 3.0
    cross3 = np.cross(P1 - P0, P2 - P0)
    s3 = np.sign(np.sum(Nm * cross3, axis=1))

    Q0x, Q0y = uv[t[:, 0], 0] * w, (1.0 - uv[t[:, 0], 1]) * h
    Q1x, Q1y = uv[t[:, 1], 0] * w, (1.0 - uv[t[:, 1], 1]) * h
    Q2x, Q2y = uv[t[:, 2], 0] * w, (1.0 - uv[t[:, 2], 1]) * h
    cross2 = (Q1x - Q0x) * (Q2y - Q0y) - (Q1y - Q0y) * (Q2x - Q0x)
    s2 = np.sign(cross2)
    return s2 * s3


def label_islands(
    mesh: ObjMesh,
    islands: list[np.ndarray],
    w: int,
    h: int,
    tex_name: str,
    ref: AircraftRef,
    top_n: int = 25,
) -> tuple[np.ndarray, list[str]]:
    img = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype(str(FONT_PATH), size=max(12, min(w, h) // 100))
    except Exception:
        font = ImageFont.load_default()

    uv = mesh.verts[:, 6:8]
    xyz = mesh.verts[:, 0:3]
    nrm = mesh.verts[:, 3:6]

    override = TEXTURE_LABEL_OVERRIDE.get(tex_name)
    mirror_sign = triangle_mirror_signs(mesh, w, h) if len(mesh.tris) else np.zeros(0)
    tri_of_vertex: dict[int, list[int]] = {}
    if len(mesh.tris):
        for ti, tri in enumerate(mesh.tris):
            for vi in tri:
                tri_of_vertex.setdefault(int(vi), []).append(ti)

    scored = []
    for verts_idx in islands:
        if len(verts_idx) < 3:
            continue
        pts = uv[verts_idx]
        area_w = float(pts[:, 0].max() - pts[:, 0].min())
        area_h = float(pts[:, 1].max() - pts[:, 1].min())
        scored.append((area_w * area_h, verts_idx))
    scored.sort(key=lambda t: -t[0])

    drawn: list[str] = []
    placed_px: list[tuple[float, float]] = []
    for _, verts_idx in scored[: top_n * 3]:
        if len(drawn) >= top_n:
            break
        pts_uv = uv[verts_idx]
        pts_xyz = xyz[verts_idx]
        pts_n = nrm[verts_idx]
        cu, cv = float(pts_uv[:, 0].mean()), float(pts_uv[:, 1].mean())
        px, py = uv_to_px(cu, cv, w, h)

        too_close = any(abs(px - ppx) < w * 0.06 and abs(py - ppy) < h * 0.06 for ppx, ppy in placed_px)
        if too_close:
            continue

        cx, cy, cz = pts_xyz.mean(axis=0)
        if tex_name == "A380_EXTERTIOR_PYLON_ALBEDO.dds":
            label = f"Pylon {engine_pylon_number(float(cx), abs(float(cx)))}"
        elif override:
            label = override
        else:
            cny = float(pts_n[:, 1].mean())
            label = classify_point(float(cx), float(cy), float(cz), cny, ref)

        tri_idxs = sorted(set(ti for vi in verts_idx for ti in tri_of_vertex.get(int(vi), [])))
        mirrored = False
        if tri_idxs:
            signs = mirror_sign[tri_idxs]
            mirrored = float(np.mean(signs > 0)) > 0.5
        if mirrored:
            label = label + "  [MIRRORED - reverse text here]"

        placed_px.append((px, py))
        drawn.append(label)

        n_sample = min(len(verts_idx), 800)
        sel = verts_idx[:: max(1, len(verts_idx) // n_sample)][:n_sample]
        Psel = xyz[sel]
        uvsel = uv[sel]
        pxsel = uvsel[:, 0] * w
        pysel = (1.0 - uvsel[:, 1]) * h
        M = np.column_stack([np.ones(len(sel)), Psel[:, 0], Psel[:, 1], Psel[:, 2]])
        try:
            coeff_x, *_ = np.linalg.lstsq(M, pxsel, rcond=None)
            coeff_y, *_ = np.linalg.lstsq(M, pysel, rcond=None)
            fwd = np.array([-coeff_x[3], -coeff_y[3]])  # -d(px,py)/dz
            up = np.array([coeff_x[2], coeff_y[2]])  # d(px,py)/dy
            for vec, color in ((fwd, (255, 80, 80, 255)), (up, (120, 255, 120, 255))):
                norm = np.linalg.norm(vec)
                if norm > 1e-6:
                    vec = vec / norm * 38.0
                    draw.line([(px, py), (px + vec[0], py + vec[1])], fill=color, width=3)
                    ah = vec / 38.0 * 10.0
                    perp = np.array([-ah[1], ah[0]]) * 0.5
                    tip = (px + vec[0], py + vec[1])
                    draw.polygon(
                        [tip, (tip[0] - ah[0] + perp[0], tip[1] - ah[1] + perp[1]), (tip[0] - ah[0] - perp[0], tip[1] - ah[1] - perp[1])],
                        fill=color,
                    )
        except np.linalg.LinAlgError:
            pass

        r = 4
        draw.ellipse([px - r, py - r, px + r, py + r], outline=(255, 60, 60, 255), width=2)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx or dy:
                    draw.text((px + 6 + dx, py - 6 + dy), label, font=font, fill=(0, 0, 0, 255))
        draw.text((px + 6, py - 6), label, font=font, fill=(255, 255, 80, 255))

    return np.array(img), drawn


def make_height_guides(pos: np.ndarray, coverage: np.ndarray, nose_z: float, w: int, h: int) -> np.ndarray:
    """Hidden helper layer: thin lines every 0.5 m of height (y) and every
    5 m of station (z - nose_z), for lining a livery's cheatlines up with
    real aircraft dimensions."""
    img = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)
    y = pos[:, :, 1]
    z = pos[:, :, 2]
    station = z - nose_z

    def local_step(vals: np.ndarray) -> np.ndarray:
        gy, gx = np.gradient(vals)
        return np.sqrt(gy**2 + gx**2)

    step_y = local_step(y)
    step_s = local_step(station)

    near_half = (np.abs(y - np.round(y / 0.5) * 0.5) < np.maximum(step_y, 1e-3)) & coverage
    near_5m = (np.abs(station - np.round(station / 5.0) * 5.0) < np.maximum(step_s, 1e-3)) & coverage

    ys_idx, xs_idx = np.nonzero(near_half)
    for yy, xx in zip(ys_idx[::3], xs_idx[::3]):
        img.putpixel((int(xx), int(yy)), (255, 150, 0, 200))
    ys_idx, xs_idx = np.nonzero(near_5m)
    for yy, xx in zip(ys_idx[::3], xs_idx[::3]):
        img.putpixel((int(xx), int(yy)), (170, 90, 255, 200))

    try:
        font = ImageFont.truetype(str(FONT_PATH), size=max(10, min(w, h) // 130))
    except Exception:
        font = ImageFont.load_default()

    labelled_y = set()
    ys_idx, xs_idx = np.nonzero(near_half)
    for yy, xx in list(zip(ys_idx, xs_idx))[::800]:
        val = round(float(y[yy, xx]) / 0.5) * 0.5
        key = round(val, 1)
        if key in labelled_y:
            continue
        labelled_y.add(key)
        draw.text((int(xx) + 3, int(yy) - 8), f"y={val:.1f}m", font=font, fill=(255, 180, 60, 255))

    labelled_s = set()
    ys_idx, xs_idx = np.nonzero(near_5m)
    for yy, xx in list(zip(ys_idx, xs_idx))[::800]:
        val = round(float(station[yy, xx]) / 5.0) * 5.0
        key = round(val, 1)
        if key in labelled_s:
            continue
        labelled_s.add(key)
        draw.text((int(xx) + 3, int(yy) + 8), f"stn={val:.0f}m", font=font, fill=(200, 140, 255, 255))

    return np.array(img)


# ---------------------------------------------------------------------------
# PSD assembly
# ---------------------------------------------------------------------------


def make_base_white(h: int, w: int) -> np.ndarray:
    return np.full((h, w, 4), 255, dtype=np.uint8)


def make_empty(h: int, w: int) -> np.ndarray:
    return np.zeros((h, w, 4), dtype=np.uint8)


def _ensure_nonempty_alpha(rgba: np.ndarray) -> np.ndarray:
    if not np.any(rgba[:, :, 3]):
        rgba = rgba.copy()
        rgba[0, 0, 3] = 1
    return rgba


def _channels_from_rgba(rgba: np.ndarray) -> dict[int, np.ndarray]:
    return {
        0: np.ascontiguousarray(rgba[:, :, 0]),
        1: np.ascontiguousarray(rgba[:, :, 1]),
        2: np.ascontiguousarray(rgba[:, :, 2]),
        -1: np.ascontiguousarray(rgba[:, :, 3]),
    }


def _image_layer(name: str, rgba: np.ndarray, w: int, h: int, visible: bool = True, blend=enums.BlendMode.normal) -> nl.Image:
    return nl.Image(
        name=name,
        visible=visible,
        blend_mode=blend,
        top=0,
        left=0,
        bottom=h,
        right=w,
        channels=_channels_from_rgba(rgba),
        color_mode=enums.ColorMode.rgb,
    )


def build_psd(
    out_path: Path,
    w: int,
    h: int,
    house_rgba: np.ndarray,
    livery_refs: dict[str, np.ndarray],
    detail_rgba: np.ndarray,
    wire_rgba: np.ndarray,
    labels_rgba: np.ndarray,
    height_guides_rgba,
) -> None:
    layers_top_to_bottom: list[nl.Layer] = []
    layers_top_to_bottom.append(_image_layer("Labels", _ensure_nonempty_alpha(labels_rgba), w, h, visible=False))
    if height_guides_rgba is not None:
        layers_top_to_bottom.append(_image_layer("Height guides", _ensure_nonempty_alpha(height_guides_rgba), w, h, visible=False))
    layers_top_to_bottom.append(_image_layer("UV wireframe", _ensure_nonempty_alpha(wire_rgba), w, h, visible=True))
    layers_top_to_bottom.append(_image_layer("Panel lines & detail", detail_rgba, w, h, visible=True, blend=enums.BlendMode.multiply))
    layers_top_to_bottom.append(_image_layer("PAINT HERE", _ensure_nonempty_alpha(make_empty(h, w)), w, h, visible=True))
    layers_top_to_bottom.append(_image_layer("Base white", make_base_white(h, w), w, h, visible=True))
    for name, arr in livery_refs.items():
        layers_top_to_bottom.append(_image_layer(f"Reference - {name}", arr, w, h, visible=False))
    layers_top_to_bottom.append(_image_layer("Reference - Airbus House", house_rgba, w, h, visible=False))

    psd = nl.nested_layers_to_psd(
        layers_top_to_bottom,
        color_mode=enums.ColorMode.rgb,
        compression=enums.Compression.zip,
        size=(h, w),
    )

    preview = Image.fromarray(make_base_white(h, w), "RGBA")
    preview.alpha_composite(Image.fromarray(wire_rgba, "RGBA"))
    preview_arr = np.array(preview.convert("RGB"))
    stacked = np.ascontiguousarray(np.transpose(preview_arr, (2, 0, 1)))
    psd.image_data = ImageData(channels=stacked, compression=enums.Compression.raw)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "wb") as f:
        psd.write(f)


# ---------------------------------------------------------------------------
# Per-texture pipeline
# ---------------------------------------------------------------------------


@dataclass
class SheetResult:
    tex_name: str
    width: int = 0
    height: int = 0
    description: str = ""
    has_mat: bool = False
    has_alpha: bool = False
    ok: bool = False


def find_mat_norm(objects_dir: Path, tex_name: str) -> tuple[Path | None, Path | None]:
    stem = Path(tex_name).stem
    for marker in ("_ALBEDO", "_BASECOLOR"):
        idx = stem.find(marker)
        if idx != -1:
            stem = stem[:idx]
            break
    if stem.endswith("_ALB"):
        stem = stem[:-4]
    mat = None
    nrm = None
    for p in objects_dir.glob(f"{stem}*.png"):
        low = p.name.lower()
        if "mat" in low and mat is None:
            mat = p
        elif ("nml" in low or "norm" in low) and nrm is None:
            nrm = p
    return mat, nrm


def process_texture(
    tex_name: str,
    aircraft: Path,
    out_root: Path,
    texmap: dict[str, list[Path]],
    preview_dir: Path,
    ref: AircraftRef,
    report: list[str],
) -> SheetResult:
    objects_dir = aircraft / TARGET_DIR_NAME
    src = objects_dir / tex_name
    result = SheetResult(tex_name=tex_name)
    if not src.exists():
        report.append(f"SKIPPED {tex_name}: not found in {objects_dir}")
        return result

    house_rgba = load_rgba(src)
    h, w = house_rgba.shape[:2]
    result.width, result.height = w, h
    result.has_alpha = bool(np.any(house_rgba[:, :, 3] < 250))

    obj_paths = texmap.get(tex_name, [])
    meshes = [parse_obj(p) for p in obj_paths]
    total_dropped = sum(m.dropped_uv_triangles for m in meshes)
    if total_dropped:
        report.append(f"UV-FILTER {tex_name}: dropped {total_dropped} degenerate/out-of-range triangle(s)")

    result.description, hist = sheet_geometry_description(tex_name, meshes, ref)
    if tex_name == "A380X_FUSE4_ALBEDO_KE7E7E7.dds":
        note = assert_fuse4_has_fin_and_stabiliser(meshes)
        report.append(note)

    wire_layers = []
    label_layers = []
    all_labels: list[str] = []
    total_tris = 0
    for mesh in meshes:
        islands = find_islands(mesh)
        wire_layers.append(render_wireframe(mesh, w, h, report, short_name(tex_name)))
        lab_img, lab_names = label_islands(mesh, islands, w, h, tex_name, ref)
        label_layers.append(lab_img)
        all_labels.extend(lab_names)
        total_tris += len(mesh.tris)

    if wire_layers:
        wire_rgba = np.zeros((h, w, 4), dtype=np.uint8)
        labels_rgba = np.zeros((h, w, 4), dtype=np.uint8)
        for wl in wire_layers:
            mask = wl[:, :, 3] > 0
            wire_rgba[mask] = wl[mask]
        for ll in label_layers:
            mask = ll[:, :, 3] > 0
            labels_rgba[mask] = ll[mask]
    else:
        wire_rgba = make_empty(h, w)
        labels_rgba = make_empty(h, w)
        report.append(f"NOTE {tex_name}: no OBJ declares this texture -- wireframe/labels left blank")

    if meshes:
        coverage = np.zeros((h, w), dtype=bool)
        for mesh in meshes:
            coverage |= rasterize_coverage(mesh, w, h)
    else:
        coverage = np.ones((h, w), dtype=bool)

    livery_refs = gather_livery_refs(aircraft, tex_name, w, h)
    livery_rgbas = [house_rgba] + list(livery_refs.values())
    if len(livery_rgbas) == 1:
        report.append(
            f"NOTE {tex_name}: only the house livery ships this sheet -- the black-top-hat filter still "
            "excludes large artwork by construction, but a thin dark feature unique to a future livery "
            "has nothing to cross-check against here"
        )

    roughness_source = find_msfs_roughness_source(tex_name)
    if roughness_source is None:
        report.append(f"NOTE {tex_name}: no source MSFS METAL/COMP roughness map found -- detail layer built without the weathering term")
    detail_rgba = make_panel_detail_multi((h, w), livery_rgbas, coverage, roughness_source)

    height_guides_rgba = None
    if tex_name in POSITION_MAP_TEXTURES and meshes:
        pm_cov = np.zeros((h, w), dtype=bool)
        pm_pos = np.zeros((h, w, 3), dtype=np.float32)
        for mesh in meshes:
            c, p, _ny = rasterize_position_map(mesh, w, h)
            pm_pos[c] = p[c]
            pm_cov |= c
        height_guides_rgba = make_height_guides(pm_pos, pm_cov, ref.nose_z, w, h)

    stem = Path(tex_name).stem
    psd_path = out_root / "PSD" / f"{stem}.psd"
    build_psd(psd_path, w, h, house_rgba, livery_refs, detail_rgba, wire_rgba, labels_rgba, height_guides_rgba)

    png_dir = out_root / "PNG layers" / stem
    save_png(house_rgba, png_dir / "01_Reference_Airbus_House.png")
    idx = 2
    for name, arr in livery_refs.items():
        save_png(arr, png_dir / f"{idx:02d}_Reference_{name}.png")
        idx += 1
    save_png(make_base_white(h, w), png_dir / f"{idx:02d}_Base_white.png")
    idx += 1
    save_png(make_empty(h, w), png_dir / f"{idx:02d}_PAINT_HERE.png")
    idx += 1
    save_png(detail_rgba, png_dir / f"{idx:02d}_Panel_lines_and_detail_MULTIPLY.png")
    idx += 1
    save_png(wire_rgba, png_dir / f"{idx:02d}_UV_wireframe.png")
    idx += 1
    if height_guides_rgba is not None:
        save_png(height_guides_rgba, png_dir / f"{idx:02d}_Height_guides.png")
        idx += 1
    save_png(labels_rgba, png_dir / f"{idx:02d}_Labels.png")

    house_im = Image.fromarray(house_rgba, "RGBA")
    preview = house_im.copy()
    preview.alpha_composite(Image.fromarray(wire_rgba, "RGBA"))
    preview.alpha_composite(Image.fromarray(labels_rgba, "RGBA"))
    preview.convert("RGB").save(preview_dir / f"{stem}_preview.png")

    mat_src, nrm_src = find_mat_norm(objects_dir, tex_name)
    result.has_mat = mat_src is not None
    if mat_src or nrm_src:
        mat_dir = out_root / "Material maps"
        mat_dir.mkdir(parents=True, exist_ok=True)
        for p in (mat_src, nrm_src):
            if p is not None:
                Image.open(p).convert("RGB").save(mat_dir / p.name)

    result.ok = True
    report.append(f"OK {tex_name} ({w}x{h}) tris={total_tris} labels={len(all_labels)} liveries={1 + len(livery_refs)}")
    return result


def process_solid(tex_name: str, aircraft: Path, out_root: Path, report: list[str]) -> None:
    objects_dir = aircraft / TARGET_DIR_NAME
    src = objects_dir / tex_name
    if not src.exists():
        report.append(f"SKIPPED (solid) {tex_name}: not found")
        return
    arr = load_rgba(src)
    out_dir = out_root / "Solid colours (no PSD)"
    out_dir.mkdir(parents=True, exist_ok=True)
    save_png(arr, out_dir / (Path(tex_name).stem + ".png"))
    report.append(f"OK (solid, copy only) {tex_name} ({arr.shape[1]}x{arr.shape[0]})")


# ---------------------------------------------------------------------------
# Example livery (task G: painted from a 3-D position map, not a straight
# texture-space band)
# ---------------------------------------------------------------------------

CHEATLINE_RGB = (208, 32, 48)
BELLY_RGB = (170, 170, 172)
FIN_RGB = (208, 32, 48)
FIN_Y0 = 8.0
BELLY_Y_MAX = 0.3


def compute_window_line(objects_dir: Path, texmap: dict[str, list[Path]]) -> float:
    paths = texmap.get("A380_PAX_WINDOW_ALBEDO_KE7E7E7_DECAL.dds", [])
    if not paths:
        return 2.0
    mesh = parse_obj(paths[0])
    return float(mesh.verts[:, 1].min()) if len(mesh.verts) else 2.0


def paint_example_sheet(
    tex_name: str,
    texmap: dict[str, list[Path]],
    detail_rgba: np.ndarray,
    w: int,
    h: int,
    window_y_min: float,
    report: list[str],
) -> np.ndarray:
    paths = texmap.get(tex_name, [])
    if not paths:
        report.append(f"EXAMPLE LIVERY {tex_name}: no OBJ found, painting plain white")
        out = np.full((h, w, 4), 255, dtype=np.uint8)
        gray = detail_rgba[:, :, 0:1].astype(np.float32) / 255.0
        out[:, :, :3] = np.clip(out[:, :, :3].astype(np.float32) * gray, 0, 255).astype(np.uint8)
        return out

    coverage = np.zeros((h, w), dtype=bool)
    posmap = np.zeros((h, w, 3), dtype=np.float32)
    for p in paths:
        mesh = parse_obj(p)
        c, pos, _ny = rasterize_position_map(mesh, w, h)
        posmap[c] = pos[c]
        coverage |= c

    cheat_top = window_y_min - 0.3
    cheat_bot = cheat_top - 0.6

    out_rgb = np.full((h, w, 3), 255.0, dtype=np.float32)
    x = posmap[:, :, 0]
    y = posmap[:, :, 1]

    is_fin = coverage & (y >= FIN_Y0)
    is_stab = coverage & (np.abs(x) > 5.0) & (y < FIN_Y0)
    is_belly = coverage & (y < BELLY_Y_MAX) & ~is_fin & ~is_stab
    is_cheat = coverage & (y >= cheat_bot) & (y <= cheat_top) & ~is_fin & ~is_stab & ~is_belly

    out_rgb[is_fin] = FIN_RGB
    out_rgb[is_belly] = BELLY_RGB
    out_rgb[is_cheat] = CHEATLINE_RGB

    gray = detail_rgba[:, :, 0:1].astype(np.float32) / 255.0
    out_rgb = np.clip(out_rgb * gray, 0, 255)

    out = np.full((h, w, 4), 255, dtype=np.uint8)
    out[:, :, :3] = out_rgb.astype(np.uint8)
    out[~coverage] = [255, 255, 255, 255]

    report.append(
        f"EXAMPLE LIVERY {tex_name}: cheatline y=[{cheat_bot:.2f},{cheat_top:.2f}] m (window sill y={window_y_min:.2f} m, "
        f"-0.3 m gap, 0.6 m band); fin y>={FIN_Y0}; belly y<{BELLY_Y_MAX}"
    )
    return out


def build_example_livery(aircraft: Path, out_root: Path, texmap: dict[str, list[Path]], report: list[str]) -> None:
    objects_dir = aircraft / TARGET_DIR_NAME
    demo_name = "FBW Paint Kit Demo"
    demo_root = out_root / "Example livery" / "liveries" / demo_name
    demo_objects = demo_root / TARGET_DIR_NAME
    demo_objects.mkdir(parents=True, exist_ok=True)

    window_y_min = compute_window_line(objects_dir, texmap)
    report.append(f"EXAMPLE LIVERY: window sill y_min={window_y_min:.3f} m (from A380_PAX_WINDOW_ALBEDO_KE7E7E7_DECAL, exterior_030)")

    for tex_name in EXAMPLE_LIVERY_TEXTURES:
        src = objects_dir / tex_name
        if not src.exists():
            report.append(f"SKIPPED (example livery) {tex_name}: not found")
            continue
        house_rgba = load_rgba(src)
        h, w = house_rgba.shape[:2]
        coverage = np.zeros((h, w), dtype=bool)
        for p in texmap.get(tex_name, []):
            coverage |= rasterize_coverage(parse_obj(p), w, h)
        if not coverage.any():
            coverage = np.ones((h, w), dtype=bool)
        livery_rgbas = [house_rgba] + list(gather_livery_refs(aircraft, tex_name, w, h).values())
        detail_rgba = make_panel_detail_multi((h, w), livery_rgbas, coverage, find_msfs_roughness_source(tex_name))

        painted = paint_example_sheet(tex_name, texmap, detail_rgba, w, h, window_y_min, report)
        out_path = demo_objects / tex_name
        info = dds_mod.write_dxt5_dds(painted, out_path)
        report.append(f"EXAMPLE LIVERY {tex_name}: wrote DXT5 DDS, {info['mip_count']} mip levels")

    for icon in aircraft.glob("FlyByWire A380X_icon11*.png"):
        (demo_root / icon.name).write_bytes(icon.read_bytes())

    (demo_root / "README_example_livery.txt").write_text(
        "FBW Paint Kit Demo -- a generated DEMONSTRATION livery, not a real one.\n\n"
        "Painted from each sheet's own 3-D position (not a straight band in texture\n"
        "space, which would land in the wrong place on a curved, mirrored UV layout):\n"
        "white fuselage, a red cheatline at a fixed height band just below the main-\n"
        "deck window line, a light grey belly, and a red fin with a white stabiliser.\n"
        "The 'Panel lines & detail' layer is multiplied on top so seams and rivets\n"
        "still read through the new colours. Exported as DXT5 DDS with a full mip\n"
        "chain via tools/paintkit/dds.py, matching the aircraft's own texture format.\n\n"
        "To try it: copy the 'FBW Paint Kit Demo' folder into\n"
        "  D:/Steam Games/steamapps/common/X-Plane 12/Aircraft/FlyByWire A380X/liveries/\n"
        "then pick it from the livery selector in X-Plane. Anything not shipped here\n"
        "(wings, engines, registration, WING1_2/2_2, etc.) falls back to the house\n"
        "livery's own textures automatically.\n",
        encoding="utf-8",
    )
    report.append(f"OK example livery -> {demo_root}")


# ---------------------------------------------------------------------------
# README
# ---------------------------------------------------------------------------

TINT_NOTE = """\
## 4. Colour / tint: paint at full, final brightness -- do not pre-darken

Short answer: **paint the exact final colours you want to see in X-Plane, at
normal brightness.** Do not multiply your colours down to compensate for the
`_KE7E7E7` in the file names.

Why the base files are named that way: the converter (`msfs2xp-aircraft`,
built on the `msfs2xp` texture pipeline) turns MSFS glTF materials into
X-Plane OBJ8 + a texture file per material. MSFS materials carry a
`baseColorFactor` that *multiplies* the base colour texture (glTF spec, in
linear light); X-Plane's OBJ8 has no equivalent per-material tint at all, so
whatever a `baseColorFactor` was doing has to be baked into the texture's own
pixels once, at conversion time (see `tint_rgba` and `TextureKey::tint` in
`Converter/src/texture/mod.rs` and `msfs2xp-aircraft/src/model/obj8.rs`). The
FlyByWire A380's fuselage material uses a `baseColorFactor` of about
`[0.8, 0.8, 0.8]` -- sRGB-encoded, 0.8 linear is very close to the sRGB byte
`0xE7` -- so the converter multiplies that into the texture and tags the
output file name with the factor it used (`_KE7E7E7`), the same way it tags
every other tinted variant, purely so two materials that would otherwise
collide on one file name do not overwrite each other's bake.

That suffix is **only bookkeeping inside the converter.** Once the file is on
disk, X-Plane does not know or care what produced it -- the OBJ's `TEXTURE`
line just names a file, and X-Plane draws whatever pixels are in it, with no
further multiply. A livery replaces that file wholesale. There is nothing
left to "undo": paint your registration white, your cheatline red, your logo
whatever colour the brand guide says, and that is what shows in the sim.
"""

MAT_NOTE = """\
## 5. The *_MAT (material/gloss) and *_NML/*_NORMAL (normal map) sheets

`*_MAT`/`*_COMP_MAT` built from MSFS's COMP texture (occlusion / roughness /
metalness in its red / green / blue channels) by `material_png()` in
`Converter/src/texture/mod.rs`. X-Plane's `TEXTURE_MAP material_gloss` layout
(Laminar's convention):

| Channel | Meaning |
|---|---|
| Red   | Metalness (0 = dielectric/paint, 255 = bare metal), scaled down on rough surfaces so a rough "metal" reading does not blind-mirror in X-Plane's sun specular. |
| Green | Gloss (inverse roughness: 255 = mirror smooth, 0 = fully matte). |
| Blue  | Unused (0). |

Chrome panel: push red and green high together. Matte livery finish over
paint: keep red low, pull green down a little. An ordinary colour repaint
does not need to touch this file at all.

`*_NORMAL_NML`/`*_NORM_NML`, Laminar's `TEXTURE_MAP normal` layout, built by
`normal_png()`:

| Channel | Meaning |
|---|---|
| Red  | Normal X, as-is from the source. |
| Green | Normal Y, inverted from the source (MSFS packs DirectX-style normals with green pointing down the image; X-Plane reads green as up). |
| Blue | Unused (0); X-Plane reconstructs it from red/green. |

Leave this alone unless you are re-sculpting surface detail, not just
repainting.
"""


def _texture_row(tex: str, aircraft: Path, descriptions: dict[str, str]) -> str:
    objects_dir = aircraft / TARGET_DIR_NAME
    src = objects_dir / tex
    size = ""
    if src.exists():
        try:
            with Image.open(src) as im:
                size = f"{im.width}x{im.height}"
        except Exception:
            size = "?"
    mat, nrm = find_mat_norm(objects_dir, tex)
    has_mat = "yes" if (mat or nrm) else "no"
    has_alpha = "yes" if "DECAL" in tex or "MASK" in tex or "_A25" in tex else "no"
    desc = descriptions.get(tex, "").replace("|", "/")
    return f"| `{tex}` ({size}) | {desc} | {has_mat} | {has_alpha} |"


def write_readme(out_root: Path, aircraft: Path, descriptions: dict[str, str], verification: dict[str, str]) -> None:
    lines: list[str] = []
    lines.append("# FlyByWire A380X (X-Plane 12 port) -- Livery Paint Kit\n")
    lines.append(
        "This kit lets a livery painter repaint the FlyByWire A380X X-Plane port "
        "without touching the aircraft install. Generated from the aircraft actually "
        f"installed at:\n\n    {aircraft}\n\n"
        "by `msfs2xp-aircraft/tools/paintkit/make_paintkit.py`. The kit folder is "
        "entirely generated; re-run that script to update it (it reads the install "
        "read-only and only ever writes into this kit folder).\n"
    )

    lines.append("## 1. The texture list\n")
    lines.append(
        "Every texture referenced by any `a380_exterior_*.obj`, plus three "
        "exterior-facing textures used from OBJ files in the cockpit numbering range "
        "(the registration placard, the ice-detector placard, and the opening "
        "upper-deck door). Descriptions below are computed from each sheet's own "
        "vertex positions -- station range aft of the nose, which side(s), height "
        "band, and the dominant named parts found on it -- not hand-typed.\n"
    )

    for title, group in TEXTURE_GROUPS:
        lines.append(f"### {title}\n")
        lines.append("| File (size) | Computed description | Has *_MAT/*_NML | Has alpha |")
        lines.append("|---|---|---|---|")
        for tex in group:
            lines.append(_texture_row(tex, aircraft, descriptions))
        lines.append("")

    lines.append("### Solid-colour textures (no PSD)\n")
    lines.append(
        "`solid_000000ff.png`, `solid_161616ff.png`, `solid_e6e6e6ff.png` "
        "(exterior_000/001/002) are flat single-colour fills shared by many small "
        "parts across the whole aircraft. Overriding one recolours **every** part "
        "that uses it, not just one:\n"
    )
    lines.append("| File | Geometry that uses it |")
    lines.append("|---|---|")
    for tex, note in zip(SOLID_TEXTURES, descriptions.get("_solid_notes", ["", "", ""])):
        lines.append(f"| `{tex}` | {note} |")
    lines.append("")

    lines.append("### Where the vertical fin and horizontal stabiliser live\n")
    lines.append(
        "Both are on **`A380X_FUSE4_ALBEDO_KE7E7E7.dds`** (fuselage sheet 4), not a "
        "separate sheet, sharing it with the tail cone. Verified by the generator "
        f"itself every run (it fails the build otherwise): {verification.get('fuse4', '')}\n"
    )

    lines.append("## 2. Folder layout a livery must use\n")
    lines.append(
        "```\n"
        "liveries/\n"
        "  My Livery Name/\n"
        "    FlyByWire A380X_icon11.png\n"
        "    FlyByWire A380X_icon11_thumb.png\n"
        "    objects/\n"
        "      A380X_FUSE1_ALBEDO_KE7E7E7.dds   <- exact file names from the tables above\n"
        "      ... only the files you actually change; anything omitted falls back to\n"
        "          the house livery's own texture automatically.\n"
        "```\n"
        "See `Example livery/` in this kit for a folder shaped exactly like this, "
        "ready to copy.\n"
    )

    lines.append("## 3. Export settings\n")
    lines.append(
        "- Export **DXT5/BC3 with a full mip chain and a legacy (non-DX10) DDS "
        "header**, matching the aircraft's own textures exactly (a 2048x2048 sheet "
        "has 12 mip levels; Emirates ships 10, which X-Plane also accepts). GIMP: "
        "File > Export As > .dds, compression BC3/DXT5, generate mipmaps. Photoshop: "
        "needs the Intel/NVIDIA DDS plug-in, then the same BC3 + mipmaps choice. Or "
        "use `Tools/export_dds.py` in this kit (below).\n"
        "- Maximum resolution **4096x4096**; do not exceed it. Native resolution "
        "(the size listed in the tables above) is best -- do not upscale.\n"
        "- Keep the registration decal's alpha channel -- see \u00a76.\n"
    )

    lines.append("## Tools/export_dds.py\n")
    lines.append(
        "A copy of this kit's DXT5 encoder (`tools/paintkit/dds.py` in the "
        "converter repo) as a standalone script: PNG in, DXT5 DDS out, same base "
        "name, full mip chain, legacy header.\n\n"
        "```\npython Tools/export_dds.py MyLivery_FUSE1.png\n```\n"
        "writes `MyLivery_FUSE1.dds` next to it. Needs `pip install pillow numpy`.\n"
    )

    lines.append(TINT_NOTE)
    lines.append(MAT_NOTE)

    lines.append("## 6. The registration decal and the exterior registration\n")
    lines.append(
        "**Two different things share the word \"registration\":**\n\n"
        "- `A380X_REGISTRATION_ALBEDO_KE7E7E7_DECAL.dds` is a small placard **inside "
        "the flight deck** (its OBJ, `a380_cockpit_011.obj`, has 20 vertices centred "
        "on the aircraft's own centreline at y=2.4, z=-32.7 -- a cockpit fixture, not "
        "an exterior letter). Keep its alpha channel: alpha=0 stays invisible, "
        "alpha=255 is solid lettering, and anti-aliased edges should keep partial "
        "alpha rather than being forced to 0/255.\n"
        "- The **exterior** registration (the tail number painted on the fuselage/fin "
        "that other liveries show) is baked directly into the FUSE sheets' own "
        "pixels -- it is not a separate decal file. Paint your own registration "
        "directly onto FUSE4/FUSE5 (or wherever your livery places it) as ordinary "
        "artwork on the `PAINT HERE` layer.\n"
    )

    lines.append("## 7. PSD layer structure\n")
    lines.append(
        "Bottom to top, in every PSD under `PSD/`:\n\n"
        "1. **Reference - Airbus House** (hidden) -- the base aircraft's own texture.\n"
        "2. **Reference - Emirates / Reference - Pride** (hidden, where that livery ships this sheet).\n"
        "3. **Base white** -- plain white, full opacity.\n"
        "4. **PAINT HERE** -- empty; your artwork goes here.\n"
        "5. **Panel lines & detail** (Multiply) -- panel lines, rivets, door outlines and drain holes "
        "only, white everywhere else. Built per livery version of this sheet (house, Emirates, Pride, "
        "whichever exist) with a morphological black top-hat (`grey_closing(L, k) - L`, k~5 px at a "
        "2048 sheet, scaled with resolution): this responds only to a dark feature narrower than the "
        "kernel, so a seam or rivet darkens the layer a little while a whole paint field, a thick "
        "wordmark, or a big graphic -- whatever its size, brightness, or how many liveries carry it -- "
        "gives zero response and stays plain white. The per-livery results are combined with the "
        "per-pixel MAXIMUM (least darkening), so a thin dark feature only one livery happens to carry "
        "is masked by any other livery reading flat there, while a seam every livery shares survives in "
        "all of them. Where a source MSFS roughness map still exists for this sheet, its (livery-"
        "independent) green channel is multiplied in as a gentle +/-10% weathering term; sheets with no "
        "such source are noted in the generation report and simply skip that term. A sheet only the "
        "house livery ships still gets full large-artwork suppression from the top-hat alone -- only a "
        "thin dark feature unique to a livery no one has painted yet could still show, which is a much "
        "narrower gap than the old ratio-based approach left.\n"
        "6. **Height guides** (hidden; main livery sheets + PAX_DOORS only -- \u00a71 group 1, plus the "
        "upper-deck door) -- thin lines every 0.5 m of height and every 5 m of fuselage station, "
        "labelled in metres, rasterised from each texel's actual 3-D position. Line up cheatlines "
        "with it. Not generated for every sheet -- computing a full per-texel 3-D position raster on "
        "a 250,000-triangle mesh like the tyres is not worth the minutes it costs for a sheet nobody "
        "paints a cheatline onto.\n"
        "7. **UV wireframe** (visible) -- every triangle edge (thin cyan) and every UV-island/mesh "
        "boundary (bold yellow). On the largest meshes (over 120,000 triangles) interior edges are "
        "skipped and only boundaries are drawn, and very dense boundaries are drawn every Nth edge; "
        "the generation report says which sheets this applied to.\n"
        "8. **Labels** (hidden) -- up to 25 per sheet, largest UV island first, named from the "
        "island's absolute 3-D position on the aircraft (nose/radome, forward/mid/aft fuselage each "
        "split upper/lower and left/right, belly fairing, vertical fin, horizontal stabiliser left/"
        "right upper/lower, wing left/right upper/lower, wingtip fence, engine 1-4, pylon 1-4, gear) "
        "rather than a per-sheet relative bucket. Each label carries a red FWD arrow (toward the nose "
        "in image space) and a green UP arrow (toward the aircraft's actual up), both from a "
        "least-squares fit of the island's own vertices against image pixel coordinates -- so the "
        "arrows are correct even on a curved or rotated UV island. An island flagged "
        "`[MIRRORED - reverse text here]` has a locally reflected UV chart (its own triangle winding "
        "vs. its own UV winding disagree with the sheet's usual convention): mirror your artwork "
        "before painting it there so it reads correctly on the aircraft. No island on this aircraft is "
        "mirrored, but many are ROTATED, so always follow the arrows. On FUSE1, for example, the "
        "right-side islands have FWD pointing left and UP pointing DOWN -- rotated 180 degrees -- "
        "which is why the house texture's \"wire\" is upside-down there. Paint text on such an island "
        "rotated to match the arrows, not mirrored.\n\n"
        "The same layers, flattened to individual PNGs, are in `PNG layers/<sheet name>/` for GIMP, "
        "Krita, Paint.NET, or anything else that cannot open a PSD.\n"
    )

    lines.append("## 8. Worked example: a plain test livery\n")
    lines.append(
        "`Example livery/` in this kit is a ready-made one: the house textures repainted white with a "
        "red cheatline, a light grey belly, and a red fin, using exactly the workflow below, and "
        "exported as real DXT5 DDS via `dds.py`. Copy it into the aircraft's `liveries/` folder to see "
        "it fly before building your own from scratch. To do it by hand:\n\n"
        "1. Open `PSD/A380X_FUSE1_ALBEDO_KE7E7E7.psd`.\n"
        "2. Hide `Base white`, add a new layer above it, fill it with one flat colour.\n"
        "3. Keep `Panel lines & detail` (Multiply) on top and visible.\n"
        "4. Turn on `UV wireframe` and `Height guides` briefly to see where parts and height bands "
        "fall; turn them back off.\n"
        "5. Repeat for FUSE2..5 with the same colour, and for WINGFENCE/WING1/WING2/ENG_LH/ENG_RH/FAN "
        "if you want the whole aircraft to match.\n"
        "6. Export DXT5/BC3 with mips (\u00a73) with the sheet's exact original file name, into "
        "`liveries/My Test Livery/objects/` as shown in \u00a72.\n"
        "7. Select \"My Test Livery\" in X-Plane's livery menu and confirm the colour shows up in the "
        "right places, at the brightness you painted it (\u00a74).\n"
    )

    lines.append("## 9. Known limitations -- what this kit could not fully automate\n")
    lines.append(
        "- **The \"Panel lines & detail\" layer's approach changed twice during development, worth "
        "recording.** A blurred local-luminance-ratio approach (first a plain per-pixel maximum across "
        "liveries, then \"whichever livery is closest to neutral\") was tried first; both still let "
        "FUSE1's large-scale house artwork -- \"flybywire\"/\"SPIRIT OF TOULOUSE\" and the white/navy "
        "shattered-triangle border -- leak into the layer as a visible smudge or emboss, and the fin's "
        "copy of the same shattered pattern showed on FUSE4, because a local-average ratio reacts to "
        "*any* deviation from the local mean, whatever its size, and no way of combining liveries fixes "
        "that if the deviation is large. Replaced outright with a morphological black top-hat "
        "(`grey_closing(L, k) - L`, k~5 px at a 2048 sheet, gain 2.5, floor 0.3): it only ever responds "
        "to a dark feature narrower than the kernel, so large artwork of any size, brightness or livery "
        "coverage gives zero response by construction, before liveries are even combined -- the "
        "per-pixel MAXIMUM of the per-livery results only has to catch the rarer case of a thin dark "
        "feature (a pinstripe, a script) unique to one livery. Verified by decoding the rebuilt layer "
        "for FUSE1 (crop x 1024..2048, y 0..1400) and FUSE4: both wordmarks, the titles-band Emirates "
        "gold lettering, and the shattered-triangle border/fin pattern are gone; door outlines, drain-"
        "hole dots, and panel seams remain. A source MSFS roughness map (livery-independent) is folded "
        "in as a gentle weathering term where one still exists; sheets with none are noted in the "
        "generation report. The example livery initially still showed both artefacts even after this "
        "fix, from an unrelated bug: it built the detail layer from the house texture alone rather than "
        "every shipped livery version, so the cross-livery masking the fuselage PSDs get had nothing to "
        "compare against; fixed by sharing the same livery-gathering code (`gather_livery_refs`) between "
        "both call sites. **One residual, disclosed rather than hidden:** right along the shattered-"
        "triangle pattern's own jagged boundary (not its interior), a handful of narrow paint peninsulas "
        "are genuinely as thin as real panel structure, so the top-hat cannot tell them apart on FUSE1's "
        "house texture alone; cross-livery masking hides most of it, but a faint trace survives at one "
        "spot near the fuselage crown (~x1196,y217 on FUSE1) that only shows up after deliberately "
        "stretching contrast -- it is not visible at normal brightness in the delivered PNG/PSD/DDS. "
        "Enlarging the closing kernel from 5 px up to 31 px did not remove it (the peninsula itself is "
        "wide enough that a kernel large enough to close it would start flattening genuine panel lines "
        "too), so it was left as-is and not chased further.\n"
        "- Position-based island labels are a heuristic (absolute 3-D position bucketing against the "
        "hand-verified FUSE4 rule, extended by the same logic to the rest of the aircraft), not "
        "authored part names -- OBJ8 carries no group/part-name metadata. Cross-check against `UV "
        "wireframe` and `Preview/`.\n"
        "- `Height guides` and the example livery's position-based painting are built only for the "
        "main livery sheets and the upper-deck door (\u00a77); every other sheet still gets its full "
        "wireframe/labels/detail-layer/PSD treatment.\n"
        "- Very large meshes (TIRES_ALB has roughly a quarter million triangles) get a decimated "
        "wireframe (boundary edges only, or every Nth boundary edge) -- see the generation report for "
        "exactly which sheets this applied to.\n"
    )

    readme_md = "\n".join(lines)
    (out_root / "README.md").write_text(readme_md, encoding="utf-8")
    write_readme_html(out_root, readme_md)


def write_readme_html(out_root: Path, md: str) -> None:
    import html as _html

    def inline(s: str) -> str:
        s = _html.escape(s)
        s = re.sub(r"`([^`]+)`", r"<code>\1</code>", s)
        s = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", s)
        return s

    out = [
        "<!doctype html><meta charset='utf-8'><title>FlyByWire A380X Paint Kit</title>",
        "<style>body{font-family:system-ui,Segoe UI,Arial,sans-serif;max-width:960px;margin:2em auto;padding:0 1em;line-height:1.5;color:#1a1a1a}"
        "code{background:#f0f0f0;padding:0 .3em;border-radius:3px}"
        "table{border-collapse:collapse;margin:1em 0;width:100%}td,th{border:1px solid #ccc;padding:.4em .6em;text-align:left;vertical-align:top}"
        "h1,h2,h3{border-bottom:1px solid #ddd;padding-bottom:.2em}</style>",
    ]

    lines = md.split("\n")
    i = 0
    in_ul = in_table = in_pre = False
    while i < len(lines):
        line = lines[i]
        if line.strip().startswith("```"):
            if not in_pre:
                out.append("<pre>")
                in_pre = True
            else:
                out.append("</pre>")
                in_pre = False
            i += 1
            continue
        if in_pre:
            out.append(_html.escape(line))
            i += 1
            continue
        if line.startswith("### "):
            out.append(f"<h3>{inline(line[4:])}</h3>")
        elif line.startswith("## "):
            out.append(f"<h2>{inline(line[3:])}</h2>")
        elif line.startswith("# "):
            out.append(f"<h1>{inline(line[2:])}</h1>")
        elif line.strip().startswith("|"):
            cells = [c.strip() for c in line.strip().strip("|").split("|")]
            if all(re.fullmatch(r"-+", c) for c in cells):
                i += 1
                continue
            if not in_table:
                out.append("<table>")
                in_table = True
                tag = "th"
            else:
                tag = "td"
            out.append("<tr>" + "".join(f"<{tag}>{inline(c)}</{tag}>" for c in cells) + "</tr>")
        elif line.strip().startswith("- "):
            if not in_ul:
                out.append("<ul>")
                in_ul = True
            out.append(f"<li>{inline(line.strip()[2:])}</li>")
        else:
            if in_ul and not line.strip().startswith("-"):
                out.append("</ul>")
                in_ul = False
            if in_table and not line.strip().startswith("|"):
                out.append("</table>")
                in_table = False
            if line.strip():
                out.append(f"<p>{inline(line)}</p>")
        i += 1
    if in_ul:
        out.append("</ul>")
    if in_table:
        out.append("</table>")

    (out_root / "README.html").write_text("\n".join(out), encoding="utf-8")


EXPORT_DDS_TOOL = '''\
#!/usr/bin/env python3
"""
Tools/export_dds.py -- PNG in, DXT5/BC3 DDS out, same base name, full
box-filtered mip chain, legacy (non-DX10) DDS header. Matches the format
the aircraft's own textures ship in.

Usage:
    python export_dds.py MyLivery_FUSE1.png [more.png ...]

Writes MyLivery_FUSE1.dds next to the PNG. Needs: pip install pillow numpy
"""
import sys
from pathlib import Path

import numpy as np
from PIL import Image

import dds


def main(argv):
    if not argv:
        print(__doc__)
        return 1
    for arg in argv:
        p = Path(arg)
        if not p.exists():
            print(f"skip: {p} not found")
            continue
        im = Image.open(p).convert("RGBA")
        arr = np.array(im)
        out = p.with_suffix(".dds")
        info = dds.write_dxt5_dds(arr, out)
        print(f"{p.name} -> {out.name} ({arr.shape[1]}x{arr.shape[0]}, {info[\'mip_count\']} mip levels)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
'''


def install_export_dds_tool(out_root: Path) -> None:
    """Copy the DXT5 encoder + a CLI wrapper into the kit as Tools/, so a
    painter without the converter repo can still export a correctly
    formatted DDS (task F)."""
    tools_dir = out_root / "Tools"
    tools_dir.mkdir(parents=True, exist_ok=True)
    dds_src = Path(__file__).with_name("dds.py")
    (tools_dir / "dds.py").write_bytes(dds_src.read_bytes())
    (tools_dir / "export_dds.py").write_text(EXPORT_DDS_TOOL, encoding="utf-8")


def describe_solids(aircraft: Path) -> list[str]:
    return [
        "Fine black trim/seal details scattered across the whole aircraft (window frames, gaps, small accents) -- exterior_000.",
        "Dark grey band across the wing-box/belly area underside -- exterior_001.",
        "Light grey trim near the nose and forward belly, likely gear-bay/door edges -- exterior_002.",
    ]


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--aircraft", type=Path, default=DEFAULT_AIRCRAFT)
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--only", type=str, default="", help="comma-separated substrings to filter which textures to build")
    args = ap.parse_args()

    aircraft: Path = args.aircraft
    out_root: Path = args.out
    objects_dir = aircraft / TARGET_DIR_NAME
    if not objects_dir.is_dir():
        print(f"error: {objects_dir} not found", file=sys.stderr)
        return 1

    out_root.mkdir(parents=True, exist_ok=True)
    preview_dir = out_root / "Preview"
    preview_dir.mkdir(parents=True, exist_ok=True)

    print("Scanning OBJ8 files for TEXTURE declarations...")
    texmap = scan_texture_map(objects_dir)
    ref = compute_reference(objects_dir, texmap)
    print(f"Reference geometry: nose_z={ref.nose_z:.2f} fuse1_z1={ref.fuse1_z1:.2f} fuse2_z1={ref.fuse2_z1:.2f} fuse4_z0={ref.fuse4_z0:.2f}")

    only = [s.strip().lower() for s in args.only.split(",") if s.strip()]

    report: list[str] = []
    descriptions: dict[str, str] = {}
    verification: dict[str, str] = {}

    for tex_name in ALL_FULL_TREATMENT:
        if only and not any(s in tex_name.lower() for s in only):
            continue
        print(f"-- {tex_name}")
        result = process_texture(tex_name, aircraft, out_root, texmap, preview_dir, ref, report)
        descriptions[tex_name] = result.description
        if tex_name == "A380X_FUSE4_ALBEDO_KE7E7E7.dds":
            verification["fuse4"] = next((r for r in report if r.startswith("FUSE4 verified")), "")

    if not only:
        for tex_name in SOLID_TEXTURES:
            process_solid(tex_name, aircraft, out_root, report)
        descriptions["_solid_notes"] = describe_solids(aircraft)

        print("Building example livery...")
        build_example_livery(aircraft, out_root, texmap, report)

        print("Installing Tools/export_dds.py...")
        install_export_dds_tool(out_root)

        print("Writing README...")
        write_readme(out_root, aircraft, descriptions, verification)

    report_path = out_root / "_generation_report.txt"
    report_path.write_text("\n".join(report), encoding="utf-8")
    print("\n".join(report))
    print(f"\nDone. Report written to {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
