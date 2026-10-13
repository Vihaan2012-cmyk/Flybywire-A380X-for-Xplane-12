import collections, os, re, sys, pickle
import numpy as np
from objparse import triangles
objdir, acf_path, work = sys.argv[1:4]
jobs, baked, glow, P_uv = pickle.load(open(os.path.join(work, 'jobs.pkl'), 'rb'))
acf = open(acf_path, encoding='utf-8', errors='replace').read()
names = [os.path.splitext(f)[0] for f in re.findall(r'_v10_att_file_stl (\S+)', acf)]
cockpit = [n for n in names if n.startswith('a380_cockpit_') and n[13:].isdigit()]
users = collections.defaultdict(list)
for n in cockpit:
    for l in open(os.path.join(objdir, n + '.obj'), encoding='utf-8', errors='replace'):
        if l.startswith('TEXTURE '):
            users[l.split()[1]].append(n)
            break
        if l.startswith('POINT_COUNTS'):
            break
for tex, items in sorted(jobs.items(), key=lambda kv: -len(kv[1])):
    dens = []
    for n in users[tex]:
        h, bs = triangles(os.path.join(objdir, n + '.obj'), n)
        for b in bs:
            p, uv = b['pos'], b['uv']
            a3 = np.linalg.norm(np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0]), axis=1) / 2
            e1, e2 = uv[:, 1] - uv[:, 0], uv[:, 2] - uv[:, 0]
            a2 = np.abs(e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0]) / 2
            m = a3 > 1e-9
            dens.append(np.sqrt(a2[m] * 4096 ** 2 / a3[m]) / 1000)
    d = np.concatenate(dens) if dens else np.array([0])
    print(f'{tex}: labels {len(items)}, texture median {np.median(d):.2f} px/mm at 4096')
