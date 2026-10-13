import os, sys
import numpy as np
from PIL import Image
from objparse import triangles
import bake
objdir, dn, bi, k = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
h, bs = triangles(os.path.join(objdir, dn + '.obj'), dn)
b = bs[bi]
D, Duv, nd = b['pos'][k], b['uv'][k], b['normal'][k]
print('decal', D.round(4).tolist(), 'normal', nd.round(3), 'uv', Duv.round(4).tolist(), 'edge mm', (np.linalg.norm(D - np.roll(D, 1, 0), axis=1) * 1000).round(1))
alpha = np.asarray(Image.open('E:/fbw-gse/decalbake/work/decal_-.png').convert('RGBA'))[..., 3:4].astype(np.float32) / 255
B = bake.basis(nd)
d2raw = D @ B.T
g = bake.grid(60)
ink = bake.sample(alpha, g @ bake.uv_px(Duv, alpha.shape[1]))[:, 0] > 0.03
pts3 = g @ D
print('ink points', ink.sum(), 'of', len(g))
for n in ('a380_cockpit_050', 'a380_cockpit_051', 'a380_cockpit_052'):
    hh, pb = triangles(os.path.join(objdir, n + '.obj'), n)
    for p in pb:
        P = p['pos']
        cen = P.mean(axis=1)
        close = np.flatnonzero(np.linalg.norm(cen - D.mean(axis=0), axis=1) < 0.03)
        for t in close:
            a = P[t][0]
            nrm = p['normal'][t]
            dist = (D - a) @ nrm
            l = bake.bary2(pts3 @ B.T, bake.ccw(P[t] @ B.T))
            if l is None:
                continue
            cov = (l >= -1e-6).all(axis=1)
            if (cov & ink).any():
                print(n, 'batch', p['batch'], 'tri', t, 'dot %.3f' % (nrm @ nd), 'dist mm', (dist * 1000).round(2), 'covers ink', int((cov & ink).sum()), 'moving' if (p['animated'] or p['conditional']) else 'static')
