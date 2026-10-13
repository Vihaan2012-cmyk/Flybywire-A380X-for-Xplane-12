"""Inlet lip geometry of the converted A380 for engine_cover_blender.py.

    python measure_inlets.py <aircraft objects dir> <a380_walkaround.obj> <out.json>

Each engine's axis centre comes from the walkaround object's own inlet
cover (fitted to the lip by make_walkaround.py); the lip's forward plane,
its highlight radius and the outer cowl radius just aft of it come from the
nacelle object (textures A380_EXTERIOR_ENG_LH/RH).
"""
import glob
import json
import os
import re
import sys

import numpy as np


def walkaround_centres(path):
    lines = open(path, encoding="latin-1").read().splitlines()
    vt = np.array([[float(x) for x in l.split()[1:4]] for l in lines if l.startswith("VT ")])
    idx = []
    for l in lines:
        if l.startswith("IDX10 "):
            idx += [int(x) for x in l.split()[1:]]
        elif l.startswith("IDX "):
            idx.append(int(l.split()[1]))
    centres, cur = {}, None
    for l in lines:
        m = re.match(r"ANIM_hide -0.5 0.5 fbw/walkaround/(eng_inlet_cover_\d)", l)
        if m:
            cur = m.group(1)
            continue
        m = re.match(r"TRIS (\d+) (\d+)", l)
        if m and cur:
            ids = sorted(set(idx[int(m.group(1)):int(m.group(1)) + int(m.group(2))]))
            centres[cur] = vt[ids].mean(axis=0)
            cur = None
    return centres


def nacelle_points(objects_dir):
    pts = []
    for f in glob.glob(os.path.join(objects_dir, "a380_exterior_*.obj")):
        with open(f, encoding="latin-1") as fh:
            head = fh.read(4000)
            if not re.search(r"TEXTURE A380_EXTERIOR_ENG_(LH|RH)_ALBEDO", head):
                continue
            fh.seek(0)
            pts += [[float(x) for x in l.split()[1:4]] for l in fh if l.startswith("VT ")]
    return np.array(pts)


def main(objects_dir, walkaround, out):
    pts = nacelle_points(objects_dir)
    res = {}
    for name, c in sorted(walkaround_centres(walkaround).items()):
        cx, cy = c[0], c[1]
        for _ in range(3):
            r = np.hypot(pts[:, 0] - cx, pts[:, 1] - cy)
            near = pts[(r > 0.9) & (r < 2.6) & (np.abs(pts[:, 2] - c[2]) < 3)]
            ang = np.arctan2(near[:, 1] - cy, near[:, 0] - cx)
            sector = ((ang + np.pi) / (2 * np.pi) * 48).astype(int) % 48
            hl, cowl = [], []
            steps = [0.02 + 0.04 * i for i in range(26)]
            prof = [[] for _ in steps]
            for k in range(48):
                s_pts = near[sector == k]
                if len(s_pts) == 0:
                    continue
                front = s_pts[np.argmin(s_pts[:, 2])]
                hl.append(front)
                rs = np.hypot(s_pts[:, 0] - cx, s_pts[:, 1] - cy)
                band = rs[(s_pts[:, 2] > front[2] + 0.25) & (s_pts[:, 2] < front[2] + 0.40)]
                if len(band):
                    cowl.append(band.max())
                for i, dz in enumerate(steps):
                    b = rs[np.abs(s_pts[:, 2] - front[2] - dz) < 0.02]
                    if len(b):
                        prof[i].append(b.max())
            hl = np.array(hl)
            A = np.c_[2 * hl[:, 0], 2 * hl[:, 1], np.ones(len(hl))]
            b = hl[:, 0] ** 2 + hl[:, 1] ** 2
            cx, cy, k0 = np.linalg.lstsq(A, b, rcond=None)[0]
        r_hl = np.hypot(hl[:, 0] - cx, hl[:, 1] - cy)
        centre = np.array([cx, cy, hl[:, 2].mean()])
        _, _, v = np.linalg.svd(hl - centre)
        normal = v[2] if v[2][2] < 0 else -v[2]
        res[name] = {
            "centre": [round(float(x), 4) for x in centre],
            "normal": [round(float(x), 4) for x in normal],
            "r_lip": round(float(np.median(r_hl)), 4),
            "r_lip_spread": round(float(r_hl.max() - r_hl.min()), 4),
            "z_span": round(float(hl[:, 2].max() - hl[:, 2].min()), 4),
            "r_outer": round(float(np.median(cowl)), 4),
            "sectors": len(hl),
            "profile": [[round(dz, 3), round(float(np.median(v)), 4)] for dz, v in zip(steps, prof) if v],
        }
        print(name, res[name])
    json.dump(res, open(out, "w"), indent=1)


if __name__ == "__main__":
    main(*sys.argv[1:4])
