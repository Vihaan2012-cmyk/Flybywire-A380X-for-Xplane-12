import collections, json, os, sys
import numpy as np
from objparse import triangles
from bake import obj_path
objdir, stage, work = sys.argv[1:4]
res = json.load(open(os.path.join(work, 'bake_result.json')))
def bag(path, name):
    h, bs = triangles(path, name)
    c = collections.Counter()
    lit = collections.Counter()
    for b in bs:
        for p, uv in zip(b['pos'], b['uv']):
            key = (tuple(np.round(p, 6).ravel()), tuple(np.round(uv, 6).ravel()), b['sig'])
            c[key] += 1
            lit[(key, b['light'])] += 1
    return h, c, lit
bad = 0
for f in sorted(os.listdir(stage)):
    if not f.endswith('.obj'):
        continue
    n = f[:-4]
    h0, a, la = bag(obj_path(objdir, f), n)
    h1, b, lb = bag(os.path.join(stage, f), n)
    gone = a - b
    added = b - a
    exp_gone = sum(1 for e in res['baked'].get(n, []))
    ok = not added and (sum(gone.values()) == exp_gone if n in res['baked'] else not gone)
    tex_ok = h0['texture'] == h1['texture'] and all(h0['maps'].get(k) == v for k, v in h1['maps'].items())
    moved_light = sum(v for (k, l), v in (lb - la).items())
    print(f'{n}: {sum(a.values())} -> {sum(b.values())} tris, removed {sum(gone.values())} (expected {exp_gone if n in res["baked"] else 0}), added {sum(added.values())}, lit {h1["lit"]}, relit {moved_light}, {"OK" if ok and tex_ok else "MISMATCH"}')
    bad += not (ok and tex_ok)
    for t in (h1['lit'],):
        if t and not (os.path.exists(os.path.join(stage, t)) or os.path.exists(obj_path(objdir, t))):
            print('  missing lit texture', t); bad += 1
sys.exit(1 if bad else 0)
