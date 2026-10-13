import math

import numpy as np


class Tri:
    __slots__ = ('obj', 'batch', 'k', 'pos', 'uv', 'normal', 'light', 'animated', 'conditional', 'lod')


def _rot(axis, deg):
    ax = np.asarray(axis, dtype=np.float64)
    n = np.linalg.norm(ax)
    if n == 0:
        return np.eye(4)
    x, y, z = ax / n
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    t = 1 - c
    m = np.eye(4)
    m[:3, :3] = [[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                 [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                 [t * x * z - s * y, t * y * z + s * x, t * z * z + c]]
    return m


def _trans(v):
    m = np.eye(4)
    m[:3, 3] = v
    return m


def parse(path):
    lines = open(path, encoding='utf-8', errors='replace').read().splitlines()
    vt, idx = [], []
    header = {'texture': None, 'lit': None, 'maps': {}}
    commands = []
    for line in lines:
        t = line.split()
        if not t:
            continue
        k = t[0]
        if k == 'VT':
            vt.append([float(x) for x in t[1:9]])
        elif k in ('IDX', 'IDX10'):
            idx.extend(int(x) for x in t[1:])
        elif k == 'TEXTURE':
            header['texture'] = t[1] if len(t) > 1 else None
        elif k == 'TEXTURE_LIT':
            header['lit'] = t[1]
        elif k == 'TEXTURE_MAP':
            header['maps'][t[1]] = t[2]
        else:
            commands.append(t)
    return np.asarray(vt, dtype=np.float64).reshape(-1, 8), np.asarray(idx, dtype=np.int64), header, commands


def triangles(path, name):
    vt, idx, header, commands = parse(path)
    stack = []
    m = np.eye(4)
    animated = False
    conditional = False
    sig = ()
    light = None
    lod = None
    tris = []
    key_block = None
    batch = 0
    for t in commands:
        k = t[0]
        if k == 'ANIM_begin':
            stack.append((m.copy(), animated, conditional, sig))
        elif k == 'ANIM_end':
            m, animated, conditional, sig = stack.pop()
        elif k == 'ANIM_trans':
            a = np.array([float(x) for x in t[1:4]])
            b = np.array([float(x) for x in t[4:7]])
            if not np.allclose(a, b, atol=1e-6):
                animated = True
                sig = sig + (' '.join(t[7:]) + ' ' + ' '.join(f'{x:.5f}' for x in b - a),)
            m = m @ _trans(a)
        elif k == 'ANIM_rotate':
            axis = [float(x) for x in t[1:4]]
            r1, r2 = float(t[4]), float(t[5])
            if abs(r1 - r2) > 1e-6:
                animated = True
                sig = sig + (' '.join(t[1:]),)
            m = m @ _rot(axis, r1)
        elif k == 'ANIM_trans_begin':
            key_block = ('trans', [], t[1])
        elif k == 'ANIM_trans_key':
            key_block[1].append([float(x) for x in t[1:5]])
        elif k == 'ANIM_trans_end':
            keys = np.array(key_block[1]).reshape(-1, 4) if key_block[1] else np.zeros((1, 4))
            if not np.allclose(keys[:, 1:], keys[0, 1:], atol=1e-6):
                animated = True
                rel = keys[:, 1:] - keys[0, 1:]
                sig = sig + ('trans ' + key_block[2] + ' ' + ' '.join(f'{v:.4f}:{x:.5f},{y:.5f},{z:.5f}' for v, (x, y, z) in zip(keys[:, 0], rel)),)
            m = m @ _trans(keys[0, 1:])
            key_block = None
        elif k == 'ANIM_rotate_begin':
            key_block = ('rot', [float(x) for x in t[1:4]], [], t[4] if len(t) > 4 else '')
        elif k == 'ANIM_rotate_key':
            key_block[2].append((float(t[1]), float(t[2])))
        elif k == 'ANIM_rotate_end':
            keys = key_block[2]
            angles = [a for _, a in keys]
            if angles and max(angles) - min(angles) > 1e-6:
                animated = True
                sig = sig + ('rot ' + key_block[3] + ' ' + ' '.join(f'{x:.4f}' for x in key_block[1]) + ' ' + ' '.join(f'{v:.4f}:{a:.3f}' for v, a in keys),)
            m = m @ _rot(key_block[1], angles[0] if angles else 0.0)
            key_block = None
        elif k in ('ANIM_hide', 'ANIM_show'):
            conditional = True
            sig = sig + (' '.join(t),)
        elif k == 'ATTR_light_level':
            light = t[3] if len(t) > 3 else t[-1]
        elif k == 'ATTR_light_level_reset':
            light = None
        elif k == 'ATTR_LOD':
            lod = (float(t[1]), float(t[2]))
        elif k == 'TRIS':
            off, cnt = int(t[1]), int(t[2])
            ids = idx[off:off + cnt].reshape(-1, 3)
            p = vt[ids][..., :3]
            p = np.einsum('ij,tkj->tki', m[:3, :3], p) + m[:3, 3]
            uv = vt[ids][..., 6:8]
            nrm = np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0])
            ln = np.linalg.norm(nrm, axis=1, keepdims=True)
            nrm = np.divide(nrm, ln, out=np.zeros_like(nrm), where=ln > 1e-12)
            tris.append(dict(obj=name, batch=batch, offset=off, count=cnt, pos=p, uv=uv, normal=nrm, light=light, animated=animated, conditional=conditional, lod=lod, sig=sig))
            batch += 1
    return header, tris
