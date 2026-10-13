import collections
import json
import os
import pickle
import re
import subprocess
import sys

import numpy as np
from PIL import Image

import ddsmips
from objparse import triangles

TEXPIPE = r'D:/A380/fbw-build/target/release/examples/texpipe.exe'
MSFS_TEX = r'D:/Microsoft Flight Simulator 2020/Microsoft Flight Simulator 2020 Packages/Community/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380_842/TEXTURE'
DECAL_SRC = 'A380_COCKPIT_DECAL_ALBEDO'
EMIS_SRC = 'A380_COCKPIT_DECAL_EMIS'
DECALS = {
    'a380_cockpit_020': None,
    'a380_cockpit_021': '000000',
    'a380_cockpit_022': 'E7E7E7',
    'a380_cockpit_024': 'E7E7E7',
    'a380_cockpit_025': 'FILL707070',
}
NOT_PANELS = ('a380_cockpit_020', 'a380_cockpit_021', 'a380_cockpit_022', 'a380_cockpit_023', 'a380_cockpit_024', 'a380_cockpit_025')
SKIP_TEXTURE_PREFIX = ('A380_COCKPIT_DECAL', 'A380_COCKPIT_DISPLAY', 'A380X_LIGHT_GLASS', 'solid_')
CELL = 0.03
MIN_DOT = 0.4
MAX_GAP = 0.006
RES = 4096
OVERLAP_RES = 2048
UPSCALE_4K = {'MIP_KNOBS_02_4K_ALBEDO'}


def light_canon(main_lua):
    canon = {}
    for m in re.finditer(r'light\(\{([^}]*)\}', open(main_lua, encoding='utf-8', errors='replace').read()):
        refs = re.findall(r'"([^"]+)"', m.group(1))
        lights = sorted(r for r in refs if r.split('/')[-1].startswith('LIGHTS_'))
        head = lights[0] if lights else refs[0]
        for r in refs:
            canon.setdefault(r, head)
    return canon


CANON = {}


def bake_group(light):
    if light is None:
        return '(unlit)'
    return CANON.get(light, light)


def pair_groups(tris):
    owner = {}
    out = list(range(len(tris)))
    for t, tri in enumerate(tris):
        keys = [tuple(np.round(v, 5)) for v in tri]
        e = max(range(3), key=lambda j: np.linalg.norm(tri[j] - tri[(j + 1) % 3]))
        edge = tuple(sorted((keys[e], keys[(e + 1) % 3])))
        if edge in owner:
            out[t] = owner[edge]
        else:
            owner[edge] = t
    return out


def edge_groups(tris):
    parent = list(range(len(tris)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    owner = {}
    for t, tri in enumerate(tris):
        keys = [tuple(np.round(v, 5)) for v in tri]
        for e in range(3):
            edge = tuple(sorted((keys[e], keys[(e + 1) % 3])))
            if edge in owner:
                parent[find(t)] = find(owner[edge])
            else:
                owner[edge] = t
    return [find(t) for t in range(len(tris))]


def bary3(p, a, b, c):
    v0, v1, v2 = b - a, c - a, p - a
    d00, d01, d11 = v0 @ v0, v0 @ v1, v1 @ v1
    d20, d21 = v2 @ v0, v2 @ v1
    den = d00 * d11 - d01 * d01
    if abs(den) < 1e-18:
        return None
    v = (d11 * d20 - d01 * d21) / den
    w = (d00 * d21 - d01 * d20) / den
    return np.array([1 - v - w, v, w])


def bary2(pts, tri):
    a, b, c = tri
    v0, v1 = b - a, c - a
    den = v0[0] * v1[1] - v1[0] * v0[1]
    if abs(den) < 1e-12:
        return None
    d = pts - a
    v = (d[:, 0] * v1[1] - v1[0] * d[:, 1]) / den
    w = (v0[0] * d[:, 1] - d[:, 0] * v0[1]) / den
    return np.stack([1 - v - w, v, w], axis=1)


def area2(t):
    return abs((t[1][0] - t[0][0]) * (t[2][1] - t[0][1]) - (t[2][0] - t[0][0]) * (t[1][1] - t[0][1]))


def signed_area(p):
    x, y = p[:, 0], p[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(np.roll(x, -1), y))


def ccw(p):
    return p if signed_area(p) >= 0 else p[::-1]


def clip_area(subject, tri):
    poly = list(subject)
    for e in range(3):
        a, b = tri[e], tri[(e + 1) % 3]
        if not poly:
            return 0.0
        side = lambda p: (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        out = []
        for j in range(len(poly)):
            p, q = poly[j], poly[(j + 1) % len(poly)]
            sp, sq = side(p), side(q)
            if sp >= 0:
                out.append(p)
            if (sp >= 0) != (sq >= 0):
                t = sp / (sp - sq)
                out.append(p + t * (q - p))
        poly = out
    return signed_area(np.array(poly)) if len(poly) >= 3 else 0.0


def basis(n):
    a = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(n, a)
    u /= np.linalg.norm(u)
    return np.stack([u, np.cross(n, u)])


def sample(img, xy):
    h, w = img.shape[:2]
    x = np.clip(xy[:, 0] - 0.5, 0, w - 1.001)
    y = np.clip(xy[:, 1] - 0.5, 0, h - 1.001)
    x0, y0 = x.astype(np.int64), y.astype(np.int64)
    fx, fy = (x - x0)[:, None], (y - y0)[:, None]
    return (img[y0, x0] * (1 - fx) * (1 - fy) + img[y0, x0 + 1] * fx * (1 - fy)
            + img[y0 + 1, x0] * (1 - fx) * fy + img[y0 + 1, x0 + 1] * fx * fy)


def uv_px(uv, res, rows=None):
    return np.stack([uv[..., 0] * res, (1.0 - uv[..., 1]) * (rows or res)], axis=-1)


def obj_path(objdir, name):
    o = os.environ.get('BAKE_OVERRIDE')
    if o and os.path.exists(os.path.join(o, name)):
        return os.path.join(o, name)
    return os.path.join(objdir, name)


def apply_copies(img, tex, fill):
    p = os.environ.get('BAKE_COPIES')
    if not p:
        return img
    info = json.load(open(p)).get(tex)
    if not info:
        return img
    from unshare import extend_and_copy
    return extend_and_copy(img, info, fill)


def lin(x):
    return np.where(x <= 0.04045, x / 12.92, ((x + 0.055) / 1.055) ** 2.4)


def srgb(x):
    x = np.clip(x, 0, 1)
    return np.where(x <= 0.0031308, x * 12.92, 1.055 * x ** (1 / 2.4) - 0.055)


def run(*a):
    subprocess.run([TEXPIPE, *a], check=True, capture_output=True)


def decode_installed(path):
    head = open(path, 'rb').read(88)[84:88]
    gen = ddsmips.mips(path) if head == b'DXT1' else ddsmips.mips_bc3(path)
    return next(img for l, w, h, img in gen if l == 0)


def overlap_count(tris_uv):
    r = OVERLAP_RES
    count = np.zeros(r * r, np.int64)
    lo = np.clip(np.floor(tris_uv.min(axis=1)).astype(np.int64), 0, r - 1)
    hi = np.clip(np.ceil(tris_uv.max(axis=1)).astype(np.int64), 0, r)
    size = (hi - lo).max(axis=1)
    a = tris_uv[:, 0]
    v0, v1 = tris_uv[:, 1] - a, tris_uv[:, 2] - a
    den = v0[:, 0] * v1[:, 1] - v1[:, 0] * v0[:, 1]
    ok = (np.abs(den) > 1e-12) & ((hi - lo).min(axis=1) > 0)
    for win in (2, 4, 8, 16, 32):
        sel = np.flatnonzero(ok & (size <= win) & (size > win // 2 if win > 2 else True))
        if not len(sel):
            continue
        gy, gx = np.mgrid[0:win, 0:win]
        off = np.stack([gx.ravel(), gy.ravel()], axis=1)
        step = max(1, 4_000_000 // (win * win))
        for c in range(0, len(sel), step):
            t = sel[c:c + step]
            P = lo[t][:, None, :] + off[None]
            d = P + 0.5 - a[t][:, None, :]
            v = (d[..., 0] * v1[t, None, 1] - v1[t, None, 0] * d[..., 1]) / den[t, None]
            w = (v0[t, None, 0] * d[..., 1] - d[..., 0] * v0[t, None, 1]) / den[t, None]
            inside = (v > 1e-6) & (w > 1e-6) & (1 - v - w > 1e-6) & (P[..., 0] < hi[t, None, 0]) & (P[..., 1] < hi[t, None, 1])
            count += np.bincount((P[..., 1] * r + P[..., 0])[inside], minlength=r * r)
    for t in np.flatnonzero(ok & (size > 32)):
        ys, xs = np.mgrid[lo[t, 1]:hi[t, 1], lo[t, 0]:hi[t, 0]]
        l = bary2(np.stack([xs.ravel() + 0.5, ys.ravel() + 0.5], axis=1), tris_uv[t])
        if l is None:
            continue
        inside = (l > 1e-6).all(axis=1)
        count[(ys.ravel() * r + xs.ravel())[inside]] += 1
    return count.reshape(r, r)


_GRIDS = {}


def grid(n):
    if n not in _GRIDS:
        i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1), indexing='ij')
        keep = i + j <= n
        i, j = i[keep] / n, j[keep] / n
        _GRIDS[n] = np.stack([i, j, 1 - i - j], axis=1)
    return _GRIDS[n]


def find_jobs(loaded, alpha):
    by_texture = collections.defaultdict(list)
    for n, (h, _) in loaded.items():
        if h['texture']:
            by_texture[h['texture']].append(n)

    sig_ids = {(): 0}
    P_pos, P_uv, P_ref, P_sig = [], [], [], []
    for n, (h, batches) in loaded.items():
        tex = h['texture'] or ''
        num = int(n[13:])
        if n in NOT_PANELS or tex.startswith(SKIP_TEXTURE_PREFIX) or num >= 97 or num in (4, 8, 9, 10):
            continue
        for b in batches:
            s = sig_ids.setdefault(b['sig'] if (b['animated'] or b['conditional']) else (), len(sig_ids))
            P_pos.append(b['pos'])
            P_uv.append(b['uv'])
            P_ref.extend((n, b['batch'], k) for k in range(len(b['pos'])))
            P_sig.append(np.full(len(b['pos']), s, np.int64))
    P_pos = np.concatenate(P_pos)
    P_uv = np.concatenate(P_uv)
    P_sig = np.concatenate(P_sig)
    P_n = np.cross(P_pos[:, 1] - P_pos[:, 0], P_pos[:, 2] - P_pos[:, 0])
    P_n /= np.maximum(np.linalg.norm(P_n, axis=1, keepdims=True), 1e-12)
    P_tex = [loaded[r[0]][0]['texture'] for r in P_ref]
    cells = collections.defaultdict(list)
    lo_c = np.floor((P_pos.min(axis=1) - 0.007) / CELL).astype(np.int64)
    hi_c = np.floor((P_pos.max(axis=1) + 0.007) / CELL).astype(np.int64)
    for i in range(len(P_pos)):
        l, h = lo_c[i], hi_c[i]
        for x in range(l[0], h[0] + 1):
            for y in range(l[1], h[1] + 1):
                for z in range(l[2], h[2] + 1):
                    cells[(x, y, z)].append(i)
    cells = {k: np.asarray(v, np.int64) for k, v in cells.items()}
    print(f'static panel triangles: {len(P_pos)} in {len(cells)} cells', flush=True)

    overlaps = {}
    jobs = collections.defaultdict(list)
    baked = collections.defaultdict(list)
    glow_objects = collections.defaultdict(dict)
    report = collections.Counter()
    offedge = []
    accepted = []
    shared_rejects = []
    only = json.loads(os.environ['BAKE_ONLY']) if os.environ.get('BAKE_ONLY') else None
    for dn in DECALS:
        for b in loaded[dn][1]:
            if only is not None and [dn, b['batch']] not in only:
                continue
            group = bake_group(b['light'])
            if group is None:
                report[f'{dn} kept: own light'] += len(b['pos'])
                continue
            moving = b['animated'] or b['conditional']
            sid = sig_ids.get(b['sig'] if moving else ())
            if sid is None:
                report[f'{dn} kept: moving, no part moves the same'] += len(b['pos'])
                continue
            tag = ' (moving)' if moving else ''
            grp = edge_groups(b['pos'])
            for k in range(len(b['pos'])):
                D, Duv, nd = b['pos'][k], b['uv'][k], b['normal'][k]
                if not np.isfinite(nd).all() or np.linalg.norm(nd) < 0.5:
                    report[f'{dn}{tag} kept: degenerate'] += 1
                    continue
                lo = np.floor(D.min(axis=0) / CELL).astype(np.int64)
                hi = np.floor(D.max(axis=0) / CELL).astype(np.int64)
                found = [cells[c] for c in ((x, y, z) for x in range(lo[0], hi[0] + 1) for y in range(lo[1], hi[1] + 1) for z in range(lo[2], hi[2] + 1)) if c in cells]
                if not found:
                    report[f'{dn}{tag} kept: no panel'] += 1
                    continue
                cand = np.unique(np.concatenate(found))
                cand = cand[(P_n[cand] @ nd >= MIN_DOT) & (P_sig[cand] == sid)]
                B = basis(nd)
                d2raw = D @ B.T
                d2 = ccw(d2raw)
                d_area = signed_area(d2)
                if d_area <= 1e-12:
                    report[f'{dn}{tag} kept: degenerate'] += 1
                    continue
                c2 = P_pos[cand] @ B.T
                dlo, dhi = d2.min(axis=0), d2.max(axis=0)
                cand = cand[((c2.max(axis=1) >= dlo) & (c2.min(axis=1) <= dhi)).all(axis=1)]
                near = [i for i in cand.tolist() if clip_area(d2, ccw(P_pos[i] @ B.T)) > 1e-6 * d_area]
                if not near:
                    report[f'{dn}{tag} kept: no panel'] += 1
                    continue
                n_div = int(np.clip(np.ceil(np.linalg.norm(D - np.roll(D, 1, axis=0), axis=1).max() / 0.0001), 24, 240))
                g = grid(n_div)
                pts = g @ d2raw
                pts3 = g @ D
                ink = sample(alpha, g @ uv_px(Duv, alpha.shape[1]))[:, 0] > 0.03
                if not ink.any():
                    baked[dn].append([b['batch'], k, []])
                    report[f'{dn}{tag} removed: no ink'] += 1
                    continue
                best = np.full(len(pts), np.inf)
                owner = np.full(len(pts), -1)
                for i in near:
                    l = bary2(pts, ccw(P_pos[i] @ B.T))
                    if l is None:
                        continue
                    t = np.abs((P_pos[i][0] - pts3) @ P_n[i] / float(nd @ P_n[i]))
                    upd = (l >= -1e-6).all(axis=1) & (t <= MAX_GAP) & (t < best)
                    best[upd] = t[upd]
                    owner[upd] = i
                loose = ink & (owner < 0)
                if loose.any():
                    frac = 1 - loose.sum() / ink.sum()
                    report[f'{dn}{tag} kept: off panel edge, {int(frac * 10) * 10}%+ on panel'] += 1
                    offedge.append([dn, b['batch'], k, D.tolist(), nd.tolist(), float(frac)])
                    continue
                pairs = sorted(set(owner[ink].tolist()))
                if any(float(P_n[i] @ nd) < 0.9 for i in pairs):
                    report[f'{dn}{tag} wraps onto a bevel'] += 1
                shared = False
                maps = collections.defaultdict(list)
                for i in pairs:
                    tex = P_tex[i]
                    if tex not in overlaps:
                        overlaps[tex] = overlap_count(np.concatenate([uv_px(bt['uv'], OVERLAP_RES) for n in by_texture[tex] for bt in loaded[n][1]]))
                    ov = overlaps[tex]
                    a3, b3, c3 = P_pos[i]
                    lam = [bary3(p + ((a3 - p) @ P_n[i]) / float(nd @ P_n[i]) * nd, a3, b3, c3) for p in D]
                    if any(l is None for l in lam):
                        shared = True
                        break
                    lam = np.array(lam)
                    pp = uv_px(P_uv[i], OVERLAP_RES)
                    dp = lam @ pp
                    lo = np.clip(np.floor(np.maximum(dp.min(axis=0), pp.min(axis=0))).astype(int) - 1, 0, OVERLAP_RES - 1)
                    hi = np.clip(np.ceil(np.minimum(dp.max(axis=0), pp.max(axis=0))).astype(int) + 1, 0, OVERLAP_RES)
                    if (hi - lo).min() > 0:
                        ys, xs = np.mgrid[lo[1]:hi[1], lo[0]:hi[0]]
                        c = np.stack([xs.ravel() + 0.5, ys.ravel() + 0.5], axis=1)
                        lP, lD = bary2(c, pp), bary2(c, dp)
                        if lP is not None and lD is not None:
                            opp = np.linalg.norm(np.roll(pp, -1, axis=0) - np.roll(pp, -2, axis=0), axis=1)
                            near_tex = (lP * area2(pp) / np.maximum(opp, 1e-9) >= -1.5).all(axis=1) & (lD >= -0.05).all(axis=1)
                            if near_tex.any():
                                a_at = sample(alpha, lD[near_tex] @ uv_px(Duv, alpha.shape[1]))[:, 0]
                                hot = near_tex.copy()
                                hot[near_tex] = a_at > 0.03
                                if (ov[ys.ravel()[hot], xs.ravel()[hot]] > 1).any():
                                    shared = True
                                    break
                    maps[tex].append((i, lam))
                if shared:
                    report[f'{dn}{tag} kept: shared texture area'] += 1
                    shared_rejects.append(dict(dn=dn, batch=b['batch'], k=k, faces=[list(P_ref[i]) + [P_tex[i]] for i in pairs]))
                    continue
                accepted.append(dict(Duv=Duv, dn=dn, batch=b['batch'], k=k, group=group, maps=dict(maps), grp=(dn, b['batch'], grp[k]), tag=tag))
    while True:
        on_tri = collections.defaultdict(collections.Counter)
        for a in accepted:
            if a['group'] != '(unlit)':
                for m in a['maps'].values():
                    for i, _ in m:
                        on_tri[i][a['group']] += 1
        losers = set()
        for i, c in on_tri.items():
            if len(c) > 1:
                win = c.most_common(1)[0][0]
                losers.update((i, g) for g in c if g != win)
        if not losers:
            break
        keep = []
        for a in accepted:
            if any((i, a['group']) in losers for m in a['maps'].values() for i, _ in m):
                report[f"{a['dn']}{a['tag']} kept: another light on the same face"] += 1
            else:
                keep.append(a)
        accepted = keep
    for a in accepted:
        for tex, m in a['maps'].items():
            jobs[tex].append(dict(Duv=a['Duv'], dn=a['dn'], group=a['group'], maps=m, grp=a['grp']))
            if a['group'] != '(unlit)':
                for i, _ in m:
                    glow_objects[P_ref[i][0]][(P_ref[i][1], P_ref[i][2])] = (a['group'], tex)
        baked[a['dn']].append([a['batch'], a['k'], sorted(a['maps'])])
        report[f"{a['dn']} baked{a['tag']}" + (' across textures' if len(a['maps']) > 1 else '')] += 1
    for k_, v in sorted(report.items()):
        print(f'  {k_}: {v}', flush=True)
    json.dump(offedge, open(os.path.join(os.environ.get('BAKE_WORK', '.'), 'offedge.json'), 'w'))
    json.dump(shared_rejects, open(os.path.join(os.environ.get('BAKE_WORK', '.'), 'shared.json'), 'w'))
    return jobs, baked, glow_objects, P_uv


def load_atlases(work):
    atlases = {}
    for tint in sorted({t or '-' for t in DECALS.values()} - {t for t in DECALS.values() if t and t.startswith('FILL')}):
        png = os.path.join(work, f'decal_{tint}.png')
        if not os.path.exists(png):
            run('albedo', os.path.join(MSFS_TEX, DECAL_SRC + '.PNG.DDS'), '-', tint, str(RES), png)
        a = np.asarray(Image.open(png).convert('RGBA')).astype(np.float32) / 255.0
        a[..., :3] *= a[..., 3:4]
        atlases[tint] = a
    for fill in {t for t in DECALS.values() if t and t.startswith('FILL')}:
        a = atlases['-'].copy()
        a[..., :3] = np.array([int(fill[4 + 2 * j:6 + 2 * j], 16) for j in range(3)], np.float32) / 255.0 * a[..., 3:4]
        atlases[fill] = a
    png = os.path.join(work, 'decal_emis.png')
    if not os.path.exists(png):
        run('decode', os.path.join(MSFS_TEX, EMIS_SRC + '.PNG.DDS'), str(RES), png)
    emis = np.asarray(Image.open(png).convert('RGB')).astype(np.float32) / 255.0
    return atlases, emis


def check_decal_parity(objdir, loaded, atlases):
    for dn, tint in DECALS.items():
        if tint and tint.startswith('FILL'):
            continue
        got = decode_installed(os.path.join(objdir, loaded[dn][0]['texture'])).astype(np.float32) / 255.0
        ours = atlases[tint or '-']
        if got.shape[:2] != ours.shape[:2]:
            print(f'decal parity {dn}: size differs', flush=True)
            continue
        m = (got[..., 3] > 0.5) & (ours[..., 3] > 0.5)
        col = ours[..., :3][m] / ours[..., 3:4][m]
        print(f'decal parity {dn}: alpha diff {np.abs(got[..., 3] - ours[..., 3]).mean() * 255:.2f}, colour diff where opaque {np.abs(got[..., :3][m] - col).mean() * 255:.2f}', flush=True)


def rebuild_base(objdir, work, tex):
    stem = os.path.splitext(tex)[0]
    tint = re.search(r'_K([0-9A-F]{6})$', stem)
    tint = tint.group(1) if tint else '-'
    src_stem = re.sub(r'_K[0-9A-F]{6}$', '', stem)
    src = os.path.join(MSFS_TEX, src_stem + '.PNG.DDS')
    comp = os.path.join(MSFS_TEX, re.sub(r'_ALBEDO$', '_COMP', src_stem) + '.PNG.DDS')
    inst = decode_installed(os.path.join(objdir, tex))[..., :3].astype(np.float32)
    best = None
    for use_comp in (False, True):
        if use_comp and not os.path.exists(comp):
            continue
        probe_png = os.path.join(work, f'{stem}_probe{int(use_comp)}.png')
        run('albedo', src, comp if use_comp else '-', tint, str(inst.shape[1]), probe_png)
        a = np.asarray(Image.open(probe_png).convert('RGB')).astype(np.float32)
        if a.shape != inst.shape:
            continue
        d = float(np.abs(a - inst).mean())
        if best is None or d < best[0]:
            best = (d, use_comp)
    if best is None:
        return None, None
    full_png = os.path.join(work, f'{stem}_src.png')
    run('albedo', src, comp if best[1] else '-', tint, str(RES), full_png)
    base = np.asarray(Image.open(full_png).convert('RGBA')).astype(np.float32) / 255.0
    if os.environ.get('BAKE_CLEAN'):
        from clean import clean_rgb
        base[..., :3] = clean_rgb(base[..., :3], base.shape[1] / 4096)[0]
    base = apply_copies(base, tex, (0.0, 0.0, 0.0, 1.0))
    if stem.replace('A380_COCKPIT_', '') in UPSCALE_4K and base.shape[1] < RES:
        base = upscale(base, RES)
    return base, best


def upscale(base, res):
    h = base.shape[0] * res // base.shape[1]
    rgb = lin(np.clip(base[..., :3], 0, 1))
    chans = [np.asarray(Image.fromarray(rgb[..., c].astype(np.float32), 'F').resize((res, h), Image.BICUBIC)) for c in range(3)]
    alpha = np.asarray(Image.fromarray(base[..., 3].astype(np.float32), 'F').resize((res, h), Image.BICUBIC))
    return np.dstack([srgb(np.clip(np.dstack(chans), 0, 1)), np.clip(alpha, 0, 1)]).astype(np.float32)


def composite(base, items, P_uv, atlases, emis):
    H, W = base.shape[:2]
    layer = np.zeros((H * W, 4), np.float32)
    glow = np.zeros((H * W, 3), np.float32)
    groups = collections.OrderedDict()
    for it in items:
        groups.setdefault(it.get('grp', id(it)), []).append(it)
    for members in groups.values():
        SS = 3
        for it in members:
            at_area = max(area2(uv_px(it['Duv'], atlases[DECALS[it['dn']] or '-'].shape[1])), 1e-9)
            for i, lam in it['maps']:
                Dpx = lam @ uv_px(P_uv[i], W, H)
                if np.isfinite(Dpx).all():
                    SS = max(SS, int(np.clip(np.ceil(1.5 / max(np.sqrt(area2(Dpx) / at_area), 1e-3)), 3, 8)))
        n_sub = SS * SS
        offs = (np.arange(SS) + 0.5) / SS
        ox, oy = np.meshgrid(offs, offs)
        sub = np.stack([ox.ravel(), oy.ravel()], axis=1)
        sids, cols, ems = [], [], []
        for it in members:
            A = atlases[DECALS[it['dn']] or '-']
            dpx_atlas = uv_px(it['Duv'], A.shape[1])
            for i, lam in it['maps']:
                Ppx = uv_px(P_uv[i], W, H)
                Dpx = lam @ Ppx
                if not np.isfinite(Dpx).all():
                    continue
                lo = np.clip(np.floor(np.maximum(Ppx.min(axis=0), Dpx.min(axis=0))).astype(int), 0, [W - 1, H - 1])
                hi = np.clip(np.ceil(np.minimum(Ppx.max(axis=0), Dpx.max(axis=0))).astype(int), 0, [W, H])
                if (hi - lo).min() <= 0:
                    continue
                ys, xs = np.mgrid[lo[1]:hi[1], lo[0]:hi[0]]
                tex_idx = (ys.ravel() * W + xs.ravel()).astype(np.int64)
                pts = (np.stack([xs.ravel(), ys.ravel()], axis=1)[:, None, :] + sub[None]).reshape(-1, 2)
                lP, lD = bary2(pts, Ppx), bary2(pts, Dpx)
                if lP is None or lD is None:
                    continue
                inside = (lP >= 0).all(axis=1) & (lD >= 0).all(axis=1)
                if not inside.any():
                    continue
                sid = (np.repeat(tex_idx, n_sub) * 64 + np.tile(np.arange(n_sub), len(tex_idx)))[inside]
                uv = lD[inside] @ dpx_atlas
                c = sample(A, uv)
                sids.append(sid)
                cols.append(c)
                ems.append(sample(emis, uv) * c[:, 3:4] if it['group'] != '(unlit)' else np.zeros((len(uv), 3), np.float32))
        if not sids:
            continue
        sids = np.concatenate(sids)
        cols = np.concatenate(cols)
        ems = np.concatenate(ems)
        _, first = np.unique(sids, return_index=True)
        sids, cols, ems = sids[first], cols[first], ems[first]
        ut, inv = np.unique(sids // 64, return_inverse=True)
        acc = np.zeros((len(ut), 4), np.float32)
        np.add.at(acc, inv, cols / n_sub)
        eacc = np.zeros((len(ut), 3), np.float32)
        np.add.at(eacc, inv, ems / n_sub)
        a_ = np.clip(acc[:, 3:4], 0, 1)
        layer[ut, :3] = acc[:, :3] + layer[ut, :3] * (1 - a_)
        layer[ut, 3:4] = a_ + layer[ut, 3:4] * (1 - a_)
        glow[ut] = eacc + glow[ut] * (1 - a_)
    layer = layer.reshape(H, W, 4)
    a_ = layer[..., 3:4]
    dec = np.clip(layer[..., :3] / np.maximum(a_, 1e-6), 0, 1)
    out = srgb(lin(base[..., :3]) * (1 - a_) + lin(dec) * a_)
    return np.concatenate([out, base[..., 3:4]], axis=2), glow.reshape(H, W, 3), a_


def main(objdir, acf_path, work, stage, only=None):
    os.makedirs(work, exist_ok=True)
    os.makedirs(stage, exist_ok=True)
    acf = open(acf_path, encoding='utf-8', errors='replace').read()
    names = [os.path.splitext(f)[0] for f in re.findall(r'_v10_att_file_stl (\S+)', acf)]
    cockpit = [n for n in names if n.startswith('a380_cockpit_') and n[13:].isdigit()]
    loaded = {n: triangles(obj_path(objdir, n + '.obj'), n) for n in cockpit}
    print(f'loaded {len(loaded)} cockpit objects', flush=True)
    CANON.update(light_canon(os.path.join(objdir, '..', 'plugins', 'sasl', 'data', 'modules', 'main.lua')))
    atlases, emis = load_atlases(work)
    cache = os.path.join(work, 'jobs.pkl')
    if os.environ.get('BAKE_REUSE') and os.path.exists(cache):
        jobs, baked, glow_objects, P_uv = pickle.load(open(cache, 'rb'))
    else:
        jobs, baked, glow_objects, P_uv = find_jobs(loaded, atlases['-'][..., 3:4])
        pickle.dump((jobs, baked, glow_objects, P_uv), open(cache, 'wb'))
    check_decal_parity(objdir, loaded, atlases)

    results = {}
    for tex, items in sorted(jobs.items(), key=lambda kv: -len(kv[1])):
        if only and only.lower() not in tex.lower():
            continue
        stem = os.path.splitext(tex)[0]
        base, best = rebuild_base(objdir, work, tex)
        if base is None:
            print(f'{tex}: no recipe matched the install, skipped', flush=True)
            continue
        rgba, glow, cover = composite(base, items, P_uv, atlases, emis)
        out_png = os.path.join(work, f'{stem}_baked.png')
        Image.fromarray((rgba * 255 + 0.5).astype(np.uint8), 'RGBA').save(out_png)
        run('dds', out_png, os.path.join(stage, tex))
        lit = None
        if glow.max() > 1e-3:
            lit = f'{stem}_TXTLIT.dds'
            lit_png = os.path.join(work, f'{stem}_TXTLIT.png')
            Image.fromarray((np.clip(glow, 0, 1) * 255 + 0.5).astype(np.uint8), 'RGB').save(lit_png)
            run('dds', lit_png, os.path.join(stage, lit))
        results[tex] = dict(lit=lit, ao=best[1], parity=round(best[0], 2), decals=len(items))
        print(f'{tex}: {len(items)} decal triangles, recipe {"AO" if best[1] else "plain"} (parity {best[0]:.2f}), glow {bool(lit)}', flush=True)

    glow = {o: [[b, k, g, t_] for (b, k), (g, t_) in sorted(t.items())] for o, t in glow_objects.items()}
    json.dump(dict(results=results, baked=baked, glow_tris=glow), open(os.path.join(work, 'bake_result.json'), 'w'), indent=1)


if __name__ == '__main__':
    main(*sys.argv[1:6])
