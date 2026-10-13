import collections, json, os, sys
import numpy as np
from objparse import triangles
from density import area2, area3
objdir, map_json = sys.argv[1], sys.argv[2]
m = json.load(open(map_json))
cache = {}
def tri(obj, batch, k):
    if obj not in cache:
        h, bs = triangles(os.path.join(objdir, obj + '.obj'), obj)
        cache[obj] = (h, {b['batch']: b for b in bs})
    h, bs = cache[obj]
    b = bs[batch]
    return h['texture'], b['pos'][k], b['uv'][k]
dd = collections.defaultdict(list); pd = collections.defaultdict(list); hts = collections.defaultdict(list)
for key, (tn, tb, tk, _) in m.items():
    dn, db, dk = key.split(':')
    _, dp, duv = tri(dn, int(db), int(dk))
    ptex, pp, puv = tri(tn, tb, tk)
    da, pa = area3(dp), area3(pp)
    if da < 1e-10 or pa < 1e-10 or area2(duv) <= 0: continue
    dd[ptex].append(np.sqrt(area2(duv) * 4096 ** 2 / da) / 1000)
    pd[ptex].append(np.sqrt(area2(puv) * 4096 ** 2 / pa) / 1000)
    hts[ptex].append(np.sqrt(da) * 1000)
for t in sorted(dd, key=lambda t: -len(dd[t])):
    print(f'{t}: decal {np.median(dd[t]):.1f} px/mm, panel@4096 {np.median(pd[t]):.2f} px/mm (p10 {np.percentile(pd[t],10):.2f}), decal tri size ~{np.median(hts[t]):.1f} mm')
