import collections, json, os, re, sys
import numpy as np
from scipy.spatial import cKDTree
from objparse import triangles
objdir, acf_path, offedge = sys.argv[1:4]
acf = open(acf_path, encoding='utf-8', errors='replace').read()
names = [os.path.splitext(f)[0] for f in re.findall(r'_v10_att_file_stl (\S+)', acf)]
cockpit = [n for n in names if n.startswith('a380_cockpit_') and n[13:].isdigit() and int(n[13:]) < 97]
off = json.load(open(offedge))
C = np.array([np.mean(o[3], axis=0) for o in off])
N = np.array([o[4] for o in off])
tree = cKDTree(C)
best = [None] * len(off)
for n in cockpit:
    if n in ('a380_cockpit_020', 'a380_cockpit_021', 'a380_cockpit_022', 'a380_cockpit_023', 'a380_cockpit_024', 'a380_cockpit_025'):
        continue
    h, bs = triangles(os.path.join(objdir, n + '.obj'), n)
    for b in bs:
        P = b['pos']
        cen = P.mean(axis=1)
        rad = np.linalg.norm(P - cen[:, None], axis=2).max(axis=1) + 0.01
        for t, js in enumerate(tree.query_ball_point(cen, rad)):
            if not js:
                continue
            a, bb, cc = P[t]
            nrm = b['normal'][t]
            for j in js:
                d = float((C[j] - a) @ nrm)
                if abs(d) > 0.01:
                    continue
                q = C[j] - d * nrm
                v0, v1, v2 = bb - a, cc - a, q - a
                d00, d01, d11, d20, d21 = v0 @ v0, v0 @ v1, v1 @ v1, v2 @ v0, v2 @ v1
                den = d00 * d11 - d01 * d01
                if abs(den) < 1e-18:
                    continue
                v = (d11 * d20 - d01 * d21) / den
                w = (d00 * d21 - d01 * d20) / den
                if v >= -0.01 and w >= -0.01 and v + w <= 1.01:
                    cand = (abs(d), n, h['texture'], float(nrm @ N[j]), bool(b['animated'] or b['conditional']))
                    if best[j] is None or cand < best[j]:
                        best[j] = cand
why = collections.Counter()
ex = {}
for j, hit in enumerate(best):
    if hit is None:
        key = 'nothing within 10 mm under the centre'
    else:
        d, n, tex, dot, mov = hit
        key = f'{n} {tex} dot {"<0.9" if dot < 0.9 else ">=0.9"} dist {"<=6mm" if d <= 0.006 else ">6mm"} {"moving" if mov else "static"}'
    why[key] += 1
    ex.setdefault(key, (off[j][0], off[j][1], off[j][2], round(off[j][5], 2), [round(x, 3) for x in C[j]]))
for k, v in why.most_common(30):
    print(v, k, ex.get(k))
