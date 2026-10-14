import os
import re
import sys

import numpy as np

obj_dir = sys.argv[1]
names = sys.argv[2:]
INBOARD_MIN, OUTBOARD_MIN = 12.5, 20.25
ENGINE_TEXTURE = re.compile(r'_ENG_|_FAN_|_PYLON_|ENGINE_BLUR')


def engine_of(x):
    a = abs(x)
    if a < INBOARD_MIN:
        return 0
    if x < 0:
        return 1 if a >= OUTBOARD_MIN else 2
    return 4 if a >= OUTBOARD_MIN else 3


def hidden(engine, first, count):
    return ['ANIM_begin', f'ANIM_hide 5.5 6.5 sim/operation/failures/rel_engsep{engine - 1}', f'TRIS {first} {count}', 'ANIM_end']


def parse(lines):
    vt = np.array([[float(v) for v in l.split()[1:4]] for l in lines if l.startswith('VT')]).reshape(-1, 3)
    idx = np.array([int(v) for l in lines if l.startswith(('IDX10', 'IDX ')) for v in l.split()[1:]], dtype=np.int64)
    tris = [(int(l.split()[1]), int(l.split()[2])) for l in lines if l.startswith('TRIS ')]
    centres = np.concatenate([vt[idx[s:s + c]].reshape(-1, 3, 3).mean(axis=1) for s, c in tris]) if tris else np.zeros((0, 3))
    return vt, idx, centres


files = {n: open(os.path.join(obj_dir, n), encoding='utf-8', errors='replace').read().split('\n') for n in names}
engine_objs = [n for n, lines in files.items() if ENGINE_TEXTURE.search(' '.join(l for l in lines if l.startswith('TEXTURE')))]
centres = {n: parse(lines)[2] for n, lines in files.items()}
boxes = []
if engine_objs:
    allc = np.concatenate([centres[n] for n in engine_objs])
    for e in (1, 2, 3, 4):
        c = allc[np.array([engine_of(x) for x in allc[:, 0]]) == e]
        if len(c):
            boxes.append((c.min(axis=0) - 0.05, c.max(axis=0) + 0.05))


def inside_engines(c):
    out = c[np.abs(c[:, 0]) >= INBOARD_MIN]
    if not len(out) or not boxes:
        return False
    return all(any(((p >= lo) & (p <= hi)).all() for lo, hi in boxes) for p in out)


for name, lines in files.items():
    path = os.path.join(obj_dir, name)
    if any('rel_engsep' in l for l in lines):
        print(name, 'already split')
        continue
    if name not in engine_objs and not inside_engines(centres[name]):
        continue
    vt, idx, _ = parse(lines)
    xs = vt[:, 0]
    out, reordered, wrapped = [], False, 0
    for line in lines:
        p = line.split()
        if not p or p[0] != 'TRIS':
            out.append(line)
            continue
        first, count = int(p[1]), int(p[2])
        tri = idx[first:first + count].reshape(-1, 3)
        engines = np.array([engine_of(x) for x in xs[tri].mean(axis=1)])
        groups = [e for e in (0, 1, 2, 3, 4) if (engines == e).any()]
        if groups == [0]:
            out.append(line)
            continue
        order = np.concatenate([np.flatnonzero(engines == e) for e in groups])
        if (order != np.arange(len(order))).any():
            idx[first:first + count] = tri[order].ravel()
            reordered = True
        at = first
        for e in groups:
            n = int((engines == e).sum()) * 3
            out.extend([f'TRIS {at} {n}'] if e == 0 else hidden(e, at, n))
            wrapped += e != 0
            at += n
    if not wrapped:
        print(name, 'has no engine geometry')
        continue
    if reordered:
        body = [l for l in out if not l.startswith(('IDX10', 'IDX '))]
        at = next(i for i, l in enumerate(body) if l.startswith('VT')) + int((xs.size))
        rows = [f'IDX10 {" ".join(map(str, idx[i:i + 10]))}' for i in range(0, len(idx) - len(idx) % 10, 10)]
        rows += [f'IDX {v}' for v in idx[len(idx) - len(idx) % 10:]]
        out = body[:at] + rows + body[at:]
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(out))
    print(name, 'split,', wrapped, 'engine batches' + (', indices reordered' if reordered else ''))
