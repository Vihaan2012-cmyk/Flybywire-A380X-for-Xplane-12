import os
import subprocess
import sys

import numpy as np
from scipy import ndimage

obj_dir, out_dir, tool = sys.argv[1], sys.argv[2], sys.argv[3]
OBJECTS = ['a380_cockpit_020.obj', 'a380_cockpit_022.obj', 'a380_cockpit_023.obj', 'a380_cockpit_024.obj', 'a380_cockpit_025.obj']
TEXTURES = ['A380_COCKPIT_DECAL_ALBEDO_DECAL.dds', 'A380_COCKPIT_DECAL_ALBEDO_KE7E7E7_DECAL.dds', 'A380_COCKPIT_DECAL_ALBEDO_K000000_DECAL_A90.dds', 'A380_COCKPIT_DECAL_EMIS_LITA.dds']
SIZE = 4096
TILE = 8
G = SIZE // TILE
os.makedirs(out_dir, exist_ok=True)


def read_obj(path):
    lines = open(path, encoding='utf-8', errors='replace').read().split('\n')
    vt_lines, idx, tris = [], [], []
    for i, line in enumerate(lines):
        p = line.split()
        if not p:
            continue
        if p[0] == 'VT':
            vt_lines.append(i)
        elif p[0] in ('IDX', 'IDX10'):
            idx.extend(int(x) for x in p[1:])
        elif p[0] == 'TRIS':
            tris.append((int(p[1]), int(p[2])))
    uv = np.array([[float(lines[i].split()[7]), float(lines[i].split()[8])] for i in vt_lines])
    return lines, vt_lines, idx, tris, uv


def px(uv):
    return uv[:, 0] * SIZE, (1 - uv[:, 1]) * SIZE


objs = {n: read_obj(os.path.join(obj_dir, n)) for n in OBJECTS}
mask = np.zeros((G, G), bool)
tri_boxes = {}
for n, (lines, vt_lines, idx, tris, uv) in objs.items():
    boxes = []
    for s, c in tris:
        for k in range(s, s + c, 3):
            vs = [idx[k], idx[k + 1], idx[k + 2]]
            x, y = px(uv[vs])
            tx0, tx1 = int(np.clip(np.floor(x.min() / TILE), 0, G - 1)), int(np.clip(np.floor(x.max() / TILE), 0, G - 1))
            ty0, ty1 = int(np.clip(np.floor(y.min() / TILE), 0, G - 1)), int(np.clip(np.floor(y.max() / TILE), 0, G - 1))
            mask[ty0:ty1 + 1, tx0:tx1 + 1] = True
            boxes.append((vs, tx0, ty0))
    tri_boxes[n] = boxes
grown = ndimage.binary_dilation(mask, structure=np.ones((3, 3), bool))
labels, count = ndimage.label(grown, structure=np.ones((3, 3), int))
slices = ndimage.find_objects(labels)
rects = []
for lab, sl in enumerate(slices, start=1):
    ty, tx = sl
    rects.append((lab, tx.start * TILE, ty.start * TILE, (tx.stop - tx.start) * TILE, (ty.stop - ty.start) * TILE))
print(f'{count} patches, {sum(r[3] * r[4] for r in rects) / SIZE**2 * 100:.1f}% of the sheet')


def pack(width):
    order = sorted(rects, key=lambda r: -r[4])
    place, x, y, shelf = {}, 0, 0, 0
    for lab, rx, ry, w, h in order:
        if w > width:
            return None
        if x + w > width:
            x, y, shelf = 0, y + shelf, 0
        place[lab] = (x, y)
        x += w
        shelf = max(shelf, h)
    height = 1
    while height < y + shelf:
        height *= 2
    return place, height


best = None
for width in (512, 1024, 2048, 4096):
    r = pack(width)
    if r and (best is None or width * r[1] < best[0] * best[1][1]):
        best = (width, r)
W, (place, H) = best
print(f'new sheet {W}x{H} (was {SIZE}x{SIZE})')
offset = {lab: (place[lab][0] - rx, place[lab][1] - ry) for lab, rx, ry, w, h in rects}

for n, (lines, vt_lines, idx, tris, uv) in objs.items():
    shift = {}
    for vs, tx0, ty0 in tri_boxes[n]:
        lab = labels[ty0, tx0]
        for v in vs:
            if v in shift and shift[v] != lab:
                raise SystemExit(f'{n}: vertex {v} sits in two patches')
            shift[v] = lab
    new_uv = uv.copy()
    for v, lab in shift.items():
        dx, dy = offset[lab]
        x, y = uv[v, 0] * SIZE + dx, (1 - uv[v, 1]) * SIZE + dy
        new_uv[v] = (x / W, 1 - y / H)
    for i, line_no in enumerate(vt_lines):
        p = lines[line_no].split()
        p[7], p[8] = f'{new_uv[i, 0]:.7f}', f'{new_uv[i, 1]:.7f}'
        lines[line_no] = '\t'.join(p) if '\t' in lines[line_no] else ' '.join(p)
    objs[n] = (lines, vt_lines, idx, tris, uv, new_uv)


def sample(img, size_w, size_h, uv):
    x = np.clip((uv[:, 0] * size_w).astype(int), 0, size_w - 1)
    y = np.clip(((1 - uv[:, 1]) * size_h).astype(int), 0, size_h - 1)
    return img[y, x]


worst = 0
for t in TEXTURES:
    raw = os.path.join(out_dir, t + '.rgba')
    info = subprocess.run([tool, 'decode', os.path.join(obj_dir, t), raw], capture_output=True, text=True, check=True).stdout.split()
    old = np.fromfile(raw, np.uint8).reshape(SIZE, SIZE, 4)
    new = np.zeros((H, W, 4), np.uint8)
    for lab, rx, ry, w, h in rects:
        nx, ny = place[lab]
        new[ny:ny + h, nx:nx + w] = old[ry:ry + h, rx:rx + w]
    for n, (lines, vt_lines, idx, tris, uv, new_uv) in objs.items():
        used = sorted({idx[k] for s, c in tris for k in range(s, s + c)})
        cent_old, cent_new = [], []
        for s, c in tris:
            for k in range(s, s + c, 3):
                vs = [idx[k], idx[k + 1], idx[k + 2]]
                cent_old.append(uv[vs].mean(axis=0))
                cent_new.append(new_uv[vs].mean(axis=0))
        if cent_old:
            a = sample(old, SIZE, SIZE, np.array(cent_old)).astype(int)
            b = sample(new, W, H, np.array(cent_new)).astype(int)
            worst = max(worst, int(np.abs(a - b).max()))
    rgba = os.path.join(out_dir, t + '.new.rgba')
    new.tofile(rgba)
    print(t, info, '->', subprocess.run([tool, 'encode', rgba, str(W), str(H), os.path.join(out_dir, t)], capture_output=True, text=True, check=True).stdout.strip())
    os.remove(raw)
    os.remove(rgba)
print('largest texel difference at triangle centres, old vs new sheet:', worst)
for n, (lines, *_rest) in objs.items():
    with open(os.path.join(out_dir, n), 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(lines))
print('written to', out_dir)
