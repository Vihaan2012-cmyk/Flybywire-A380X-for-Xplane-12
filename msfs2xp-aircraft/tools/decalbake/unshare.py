import collections
import json
import os
import subprocess
import sys

import numpy as np
from PIL import Image
from scipy.ndimage import binary_dilation

import bake
from bake import TEXPIPE, decode_installed, uv_px
from objparse import triangles

G = 4096
CELLPX = 8
EXTEND = {'A380_COCKPIT_KEYBOARD_ALBEDO.dds', 'A380_COCKPIT_PEDESTAL_KNOBS01_4K_ALBEDO.dds'}


def header(path):
    h = {}
    for l in open(path, encoding='utf-8', errors='replace'):
        t = l.split()
        if not t:
            continue
        if t[0] == 'POINT_COUNTS':
            break
        if t[0] in ('TEXTURE', 'TEXTURE_LIT'):
            h[t[0]] = t[1]
        elif t[0] == 'TEXTURE_MAP':
            h[t[1]] = t[2]
    return h


def occupancy(tris_uv):
    old = bake.OVERLAP_RES
    bake.OVERLAP_RES = G
    occ = bake.overlap_count(uv_px(tris_uv, G)) > 0
    bake.OVERLAP_RES = old
    for pts in (tris_uv, (tris_uv + np.roll(tris_uv, 1, axis=1)) / 2):
        px = np.clip(np.floor(uv_px(pts, G)).astype(int).reshape(-1, 2), 0, G - 1)
        occ[px[:, 1], px[:, 0]] = True
    occ = binary_dilation(occ, iterations=3)
    n = G // CELLPX
    return occ.reshape(n, CELLPX, n, CELLPX).any(axis=(1, 3))


def place(cells, w, h):
    rows, cols = cells.shape
    if h > rows or w > cols:
        return None
    S = np.zeros((rows + 1, cols + 1), np.int32)
    S[1:, 1:] = cells.astype(np.int32).cumsum(0).cumsum(1)
    win = S[h:, w:] - S[:-h, w:] - S[h:, :-w] + S[:-h, :-w]
    ys, xs = np.nonzero(win == 0)
    if not len(ys):
        return None
    j = np.lexsort((xs, ys))[0]
    return int(ys[j]), int(xs[j])


def islands(tris):
    parent = list(range(len(tris)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    own = {}
    for i, (p, uv) in enumerate(tris):
        keys = [tuple(np.round(np.r_[p[j], uv[j]], 5)) for j in range(3)]
        for e in range(3):
            edge = tuple(sorted((keys[e], keys[(e + 1) % 3])))
            if edge in own:
                parent[find(i)] = find(own[edge])
            else:
                own[edge] = i
    out = collections.defaultdict(list)
    for i in range(len(tris)):
        out[find(i)].append(i)
    return list(out.values())


def extend_and_copy(img, info, fill=None):
    s = img.shape[1] / G
    if info['ext'] > 1:
        ext = np.zeros((img.shape[0] * info['ext'],) + img.shape[1:], img.dtype)
        if fill is not None:
            ext[...] = np.array(fill, img.dtype)[:img.shape[2]] if img.ndim == 3 else fill[0]
        ext[:img.shape[0]] = img
    else:
        ext = img.copy()
    src = ext.copy()
    for oy, ox, h, w, ny, nx in info['rects']:
        a = [int(round(v * s)) for v in (oy, ox, h, w, ny, nx)]
        ext[a[4]:a[4] + a[2], a[5]:a[5] + a[3]] = src[a[0]:a[0] + a[2], a[1]:a[1] + a[3]]
    return ext


def main(objdir, shared_json, out_dir, work):
    os.makedirs(out_dir, exist_ok=True)
    shared = json.load(open(shared_json))
    faces = collections.defaultdict(set)
    for r in shared:
        for o, b, k, t in r['faces']:
            faces[t].add((o, b, k))
    users = collections.defaultdict(list)
    heads = {}
    for f in os.listdir(objdir):
        if f.endswith('.obj') and f.startswith('a380_cockpit_'):
            h = header(os.path.join(objdir, f))
            heads[f[:-4]] = h
            if 'TEXTURE' in h:
                users[h['TEXTURE']].append(f[:-4])
    parsed = {}

    def tri(o):
        if o not in parsed:
            parsed[o] = {b['batch']: b for b in triangles(os.path.join(objdir, o + '.obj'), o)[1]}
        return parsed[o]

    moves = collections.defaultdict(dict)
    info = {}
    report = collections.Counter()
    for tex, fs in sorted(faces.items(), key=lambda kv: -len(kv[1])):
        ext = 2 if tex in EXTEND else 1
        cells = occupancy(np.concatenate([b['uv'] for n in users[tex] for b in tri(n).values()]))
        if ext > 1:
            cells = np.vstack([cells, np.zeros_like(cells)])
        info[tex] = dict(ext=ext, rects=[])
        groups = collections.defaultdict(list)
        for o, b, k in sorted(fs):
            groups[(o, b)].append(k)
        jobs = []
        n = G // CELLPX
        for (o, b), ks in groups.items():
            bt = tri(o)[b]
            for isl in islands([(bt['pos'][k], bt['uv'][k]) for k in ks]):
                kk = [ks[i] for i in isl]
                px = uv_px(bt['uv'][kk], G).reshape(-1, 2)
                lo = np.clip(np.floor((px.min(axis=0) - CELLPX) / CELLPX).astype(int), 0, n - 1)
                hi = np.clip(np.ceil((px.max(axis=0) + CELLPX) / CELLPX).astype(int), lo + 1, n)
                jobs.append((o, b, kk, lo, hi))
        jobs.sort(key=lambda j: -np.prod(j[4] - j[3]))
        for o, b, kk, lo, hi in jobs:
            w, h = (hi - lo)
            at = place(cells, w, h)
            if at is None:
                report[f'{tex}: no room'] += len(kk)
                continue
            ny, nx = at
            cells[ny:ny + h, nx:nx + w] = True
            oy, ox = lo[1], lo[0]
            du = (nx - ox) * CELLPX / G
            dv = -(ny - oy) * CELLPX / (G * ext)
            for k in kk:
                moves[o][(b, k)] = (du, dv)
            info[tex]['rects'].append([int(oy * CELLPX), int(ox * CELLPX), int(h * CELLPX), int(w * CELLPX), int(ny * CELLPX), int(nx * CELLPX)])
            report[f'{tex}: moved'] += len(kk)
    for k, v in sorted(report.items()):
        print(f'  {k}: {v}', flush=True)

    touched = set(moves)
    for tex, inf in info.items():
        if inf['ext'] > 1:
            touched.update(users[tex])
    for o in sorted(touched):
        ext = info.get(heads[o].get('TEXTURE'), {}).get('ext', 1)
        mv = moves.get(o, {})
        path = os.path.join(objdir, o + '.obj')
        lines = open(path, encoding='utf-8').read().splitlines()
        vt_at = [i for i, l in enumerate(lines) if l.startswith('VT')]
        idx_lines = [i for i, l in enumerate(lines) if l.startswith('IDX')]
        idx = []
        for i in idx_lines:
            idx.extend(int(x) for x in lines[i].split()[1:])
        vts = []
        for i in vt_at:
            t = lines[i].split()
            if ext > 1:
                t[8] = f'{1 - (1 - float(t[8])) / ext:.6f}'
            vts.append(t)
        new_vt = []
        made = {}
        batch = 0
        for l in lines:
            if not l.startswith('TRIS'):
                continue
            off, cnt = int(l.split()[1]), int(l.split()[2])
            for k in range(cnt // 3):
                d = mv.get((batch, k))
                if d is None:
                    continue
                for j in range(3):
                    vi = idx[off + 3 * k + j]
                    key = (vi, d)
                    if key not in made:
                        t = vts[vi]
                        new_vt.append(t[:7] + [f'{float(t[7]) + d[0]:.6f}', f'{float(t[8]) + d[1]:.6f}'])
                        made[key] = len(vts) + len(new_vt) - 1
                    idx[off + 3 * k + j] = made[key]
            batch += 1
        pc = next(i for i, l in enumerate(lines) if l.startswith('POINT_COUNTS'))
        c = lines[pc].split()
        out = lines[:pc] + [f'POINT_COUNTS {len(vts) + len(new_vt)} {c[2]} {c[3]} {len(idx)}']
        out += [' '.join(t) for t in vts + new_vt]
        tens = len(idx) // 10 * 10
        out += ['IDX10 ' + ' '.join(map(str, idx[i:i + 10])) for i in range(0, tens, 10)]
        out += [f'IDX {v}' for v in idx[tens:]]
        skip = set(vt_at) | set(idx_lines) | {pc}
        out += [l for i, l in enumerate(lines) if i > pc and i not in skip]
        open(os.path.join(out_dir, o + '.obj'), 'w', encoding='utf-8', newline='\n').write('\n'.join(out) + '\n')
        before = tri(o)
        after = {b['batch']: b for b in triangles(os.path.join(out_dir, o + '.obj'), o)[1]}
        bad = 0
        for b, bt in before.items():
            at_ = after[b]
            if not np.allclose(bt['pos'], at_['pos'], atol=1e-6):
                bad += 1
            want = bt['uv'].copy()
            want[..., 1] = 1 - (1 - want[..., 1]) / ext
            for k in range(len(want)):
                d = mv.get((b, k), (0.0, 0.0))
                if not np.allclose(at_['uv'][k], want[k] + np.array(d), atol=2e-6):
                    bad += 1
        print(f'{o}: {len(mv)} faces moved, {len(new_vt)} new vertices, uv x{ext}, {"OK" if not bad else f"{bad} MISMATCHES"}', flush=True)

    maps = collections.defaultdict(set)
    for o in touched:
        h = heads[o]
        for key in ('normal', 'material_gloss', 'TEXTURE_LIT'):
            if key in h:
                maps[h['TEXTURE']].add((key, h[key]))
    for tex, files in maps.items():
        for key, f in sorted(files):
            src = os.path.join(objdir, f)
            fill = (128, 128, 255, 255) if key == 'normal' else None
            if f.lower().endswith('.png'):
                img = np.asarray(Image.open(src))
                Image.fromarray(extend_and_copy(img, info[tex], fill)).save(os.path.join(out_dir, f))
            else:
                img = decode_installed(src)
                png = os.path.join(work, os.path.splitext(f)[0] + '_unshared.png')
                Image.fromarray(extend_and_copy(img, info[tex], fill)).save(png)
                subprocess.run([TEXPIPE, 'dds', png, os.path.join(out_dir, f)], check=True, capture_output=True)
            print(f'{f}: {len(info[tex]["rects"])} regions copied, canvas x{info[tex]["ext"]}', flush=True)
    json.dump(info, open(os.path.join(work, 'copies.json'), 'w'), indent=1)


if __name__ == '__main__':
    main(*sys.argv[1:5])
