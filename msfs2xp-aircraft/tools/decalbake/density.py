import collections
import json
import os
import sys

import numpy as np

from objparse import triangles


def area3(p):
    return np.linalg.norm(np.cross(p[..., 1, :] - p[..., 0, :], p[..., 2, :] - p[..., 0, :]), axis=-1) / 2


def area2(uv):
    a, b, c = uv[..., 0, :], uv[..., 1, :], uv[..., 2, :]
    return np.abs((b[..., 0] - a[..., 0]) * (c[..., 1] - a[..., 1]) - (c[..., 0] - a[..., 0]) * (b[..., 1] - a[..., 1])) / 2


def main(objdir, map_json):
    m = json.load(open(map_json))
    cache = {}
    def tri(obj, batch, k):
        if obj not in cache:
            h, bs = triangles(os.path.join(objdir, obj + '.obj'), obj)
            cache[obj] = (h, {b['batch']: b for b in bs})
        h, bs = cache[obj]
        b = bs[batch]
        return h['texture'], b['pos'][k], b['uv'][k]
    ratios = collections.defaultdict(list)
    for key, (tn, tb, tk, _) in m.items():
        dn, db, dk = key.split(':')
        _, dp, duv = tri(dn, int(db), int(dk))
        ptex, pp, puv = tri(tn, tb, tk)
        da, pa = area3(dp), area3(pp)
        if da < 1e-10 or pa < 1e-10:
            continue
        ddens = area2(duv) * 4096 ** 2 / da
        pdens = area2(puv) * 4096 ** 2 / pa
        if ddens <= 0:
            continue
        ratios[ptex].append(np.sqrt(pdens / ddens))
    for t, r in sorted(ratios.items(), key=lambda kv: -len(kv[1])):
        r = np.array(r)
        print(f'{t}: n={len(r)} panel/decal texel ratio at 4096 median {np.median(r):.2f} p10 {np.percentile(r, 10):.2f} p90 {np.percentile(r, 90):.2f}')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
