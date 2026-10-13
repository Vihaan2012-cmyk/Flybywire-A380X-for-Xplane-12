import collections
import json
import os
import sys

import numpy as np

from objparse import triangles

DECALS = ['a380_cockpit_020', 'a380_cockpit_021', 'a380_cockpit_022', 'a380_cockpit_023', 'a380_cockpit_024', 'a380_cockpit_025']
SKIP_TEXTURE_PREFIX = ('A380_COCKPIT_DECAL', 'A380_COCKPIT_DISPLAY', 'A380X_LIGHT_GLASS', 'solid_')
CELL = 0.03


def panel_objects(objdir, acf_objects):
    out = []
    for name in acf_objects:
        if name in DECALS or not name.startswith('a380_cockpit_') or name.endswith('_lt'):
            continue
        num = name[len('a380_cockpit_'):]
        if not num.isdigit() or int(num) >= 97 or int(num) in (4, 8, 9, 10):
            continue
        out.append(name)
    return out


def load(objdir, names):
    data = {}
    for n in names:
        header, tris = triangles(os.path.join(objdir, n + '.obj'), n)
        data[n] = (header, tris)
    return data


def flatten(data, static_only):
    rows = []
    for n, (header, batches) in data.items():
        for b in batches:
            if static_only and (b['animated'] or b['conditional']):
                continue
            for k in range(len(b['pos'])):
                rows.append((n, b['batch'], k, header['texture']))
    return rows


def main(objdir, acf_path, out_json):
    acf = open(acf_path, encoding='utf-8', errors='replace').read()
    import re
    acf_objects = [os.path.splitext(f)[0] for f in re.findall(r'_v10_att_file_stl (\S+)', acf)]
    panels = load(objdir, panel_objects(objdir, acf_objects))
    decals = load(objdir, DECALS)

    P_pos, P_n, P_ref = [], [], []
    for n, (header, batches) in panels.items():
        tex = header['texture'] or ''
        if tex.startswith(SKIP_TEXTURE_PREFIX):
            continue
        for b in batches:
            if b['animated'] or b['conditional']:
                continue
            P_pos.append(b['pos'])
            P_n.append(b['normal'])
            P_ref.extend((n, b['batch'], k) for k in range(len(b['pos'])))
    P_pos = np.concatenate(P_pos)
    P_n = np.concatenate(P_n)
    cent = P_pos.mean(axis=1)
    cells = collections.defaultdict(list)
    for i, c in enumerate(np.floor(cent / CELL).astype(np.int64)):
        cells[tuple(c)].append(i)
    print(f'panel triangles (static): {len(P_pos)}', flush=True)

    stats = collections.Counter()
    by_target = collections.defaultdict(collections.Counter)
    mapping = {}
    for dn, (header, batches) in decals.items():
        for b in batches:
            group = b['light'] or '(unlit)'
            for k in range(len(b['pos'])):
                key = f'{dn}:{b["batch"]}:{k}'
                if b['animated']:
                    stats[(dn, 'animated')] += 1
                    continue
                if b['conditional']:
                    stats[(dn, 'conditional')] += 1
                    continue
                tri = b['pos'][k]
                c = tri.mean(axis=0)
                nd = b['normal'][k]
                ci = np.floor(c / CELL).astype(np.int64)
                cand = []
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        for dz in (-1, 0, 1):
                            cand.extend(cells.get((ci[0] + dx, ci[1] + dy, ci[2] + dz), ()))
                best = None
                for i in cand:
                    pn = P_n[i]
                    if abs(float(pn @ nd)) < 0.9:
                        continue
                    p0 = P_pos[i][0]
                    dist = float(pn @ (c - p0))
                    if abs(dist) > 0.006:
                        continue
                    q = c - dist * pn
                    a, bb, cc = P_pos[i]
                    v0, v1, v2 = bb - a, cc - a, q - a
                    d00, d01, d11 = v0 @ v0, v0 @ v1, v1 @ v1
                    d20, d21 = v2 @ v0, v2 @ v1
                    den = d00 * d11 - d01 * d01
                    if abs(den) < 1e-18:
                        continue
                    v = (d11 * d20 - d01 * d21) / den
                    w = (d00 * d21 - d01 * d20) / den
                    if v < -0.02 or w < -0.02 or v + w > 1.02:
                        continue
                    if best is None or abs(dist) < abs(best[1]):
                        best = (i, dist)
                if best is None:
                    stats[(dn, 'unmatched')] += 1
                    continue
                tn, tb, tk = P_ref[best[0]]
                ttex = panels[tn][0]['texture']
                stats[(dn, 'matched')] += 1
                by_target[f'{tn} {ttex}'][f'{dn} {group}'] += 1
                mapping[key] = [tn, tb, tk, round(best[1] * 1000, 2)]
    print('decal triangles:')
    for (dn, kind), v in sorted(stats.items()):
        print(f'  {dn} {kind}: {v}')
    print('targets:')
    for t, groups in sorted(by_target.items(), key=lambda kv: -sum(kv[1].values())):
        top = ', '.join(f'{g} {c}' for g, c in groups.most_common(6))
        print(f'  {t}: {sum(groups.values())} -> {top}')
    json.dump(mapping, open(out_json, 'w'))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3])
