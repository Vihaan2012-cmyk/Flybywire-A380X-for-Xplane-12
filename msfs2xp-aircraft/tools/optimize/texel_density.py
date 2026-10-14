import os
import re
import struct
import sys
from collections import defaultdict

import numpy as np

root = sys.argv[1]
obj_dir = os.path.join(root, 'objects')
EYE = np.array([-0.524, 3.158, -32.072])
PX_PER_RAD = 720 / np.tan(np.radians(39.5 / 2))
MARGIN = 1.5
acf = open(next(os.path.join(root, f) for f in os.listdir(root) if f.endswith('.acf')), encoding='utf-8', errors='replace').read()
loaded = sorted(set(re.findall(r'^P _obja/\d+/_v10_att_file_stl (\S+)', acf, re.M)))


def tex_size(path):
    with open(path, 'rb') as f:
        head = f.read(32)
    if head[:4] == b'DDS ':
        h, w = struct.unpack('<II', head[12:20])
        return w, h, 'dds'
    w, h = struct.unpack('>II', head[16:24])
    return w, h, 'png'


def vram(w, h, kind, name):
    if kind == 'png':
        bpp = 8
    else:
        with open(os.path.join(obj_dir, name), 'rb') as f:
            f.seek(84)
            bpp = 4 if f.read(4) == b'DXT1' else 8
    return w * h * bpp / 8 * 4 / 3


ratios = defaultdict(list)
for name in loaded:
    if not name.startswith('a380_cockpit'):
        continue
    path = os.path.join(obj_dir, name)
    vt, idx, tris, tex = [], [], [], []
    for line in open(path, encoding='utf-8', errors='replace'):
        p = line.split()
        if not p:
            continue
        if p[0] == 'VT':
            vt.append([float(x) for x in (p[1], p[2], p[3], p[7], p[8])])
        elif p[0] in ('IDX', 'IDX10'):
            idx.extend(int(x) for x in p[1:])
        elif p[0] == 'TRIS':
            tris.append((int(p[1]), int(p[2])))
        elif p[0].startswith('TEXTURE') and p[0] != 'TEXTURE_LIT_NOWRAP':
            tex.append(p[-1])
    if not tris or not tex:
        continue
    vt = np.array(vt)
    idx = np.array(idx)
    sel = np.concatenate([idx[s:s + n] for s, n in tris]).reshape(-1, 3)
    a, b, c = vt[sel[:, 0]], vt[sel[:, 1]], vt[sel[:, 2]]
    world = 0.5 * np.linalg.norm(np.cross(b[:, :3] - a[:, :3], c[:, :3] - a[:, :3]), axis=1)
    uv = 0.5 * np.abs((b[:, 3] - a[:, 3]) * (c[:, 4] - a[:, 4]) - (c[:, 3] - a[:, 3]) * (b[:, 4] - a[:, 4]))
    dist = np.linalg.norm((a[:, :3] + b[:, :3] + c[:, :3]) / 3 - EYE, axis=1).clip(0.25)
    ok = (world > 1e-7) & (uv > 1e-10)
    for t in tex:
        p = os.path.join(obj_dir, t)
        if not os.path.exists(p):
            continue
        w, h, kind = tex_size(p)
        density = np.sqrt(uv[ok] * w * h / world[ok])
        need = PX_PER_RAD / dist[ok] * MARGIN
        r = density / need
        ratios[t].append((r, world[ok]))

total = saved = 0
rows = []
for t, parts in ratios.items():
    r = np.concatenate([p[0] for p in parts])
    wgt = np.concatenate([p[1] for p in parts])
    order = np.argsort(r)
    cum = np.cumsum(wgt[order]) / wgt.sum()
    low = r[order][np.searchsorted(cum, 0.05)]
    w, h, kind = tex_size(os.path.join(obj_dir, t))
    mb = vram(w, h, kind, t) / 2**20
    total += mb
    halvings = 0
    while low >= 2 ** (halvings + 1) and min(w, h) >> (halvings + 1) >= 512:
        halvings += 1
    gain = mb * (1 - 4 ** -halvings)
    saved += gain
    rows.append((gain, t, w, h, kind, low, halvings, mb))
for gain, t, w, h, kind, low, halvings, mb in sorted(rows, reverse=True)[:40]:
    print(f'{gain:6.1f} MB  {t:58} {w}x{h} {kind}  density/need {low:5.2f}  halve x{halvings}  ({mb:.1f} MB)')
print(f'cockpit textures {total:.0f} MB, saving {saved:.0f} MB by sizing to what the eye resolves')
