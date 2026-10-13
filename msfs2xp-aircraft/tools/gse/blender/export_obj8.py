import json
import os

import bmesh
import bpy


def xp(v):
    return (v.x, v.z, -v.y)


def _fmt(x):
    s = f'{x:.5f}'.rstrip('0').rstrip('.')
    return '0' if s in ('-0', '') else s


def _anim_lines(steps, indent):
    out = []
    for st in steps:
        k = st['kind']
        dr = st['dataref']
        if k == 'trans':
            a, b = st['offset0'], st['offset1']
            out.append(f"{indent}ANIM_trans {' '.join(_fmt(c) for c in xp_vec(a))} {' '.join(_fmt(c) for c in xp_vec(b))} {_fmt(st['v0'])} {_fmt(st['v1'])} {dr}")
        elif k == 'rotate':
            pivot = xp_vec(st['pivot'])
            out.append(f"{indent}ANIM_trans {' '.join(_fmt(c) for c in pivot)} {' '.join(_fmt(c) for c in pivot)} 0 0 none")
            out.append(f"{indent}ANIM_rotate {' '.join(_fmt(c) for c in xp_vec(st['axis']))} {_fmt(st['deg0'])} {_fmt(st['deg1'])} {_fmt(st['v0'])} {_fmt(st['v1'])} {dr}")
            out.append(f"{indent}ANIM_trans {' '.join(_fmt(-c) for c in pivot)} {' '.join(_fmt(-c) for c in pivot)} 0 0 none")
        elif k in ('hide', 'show'):
            out.append(f"{indent}ANIM_{k} {_fmt(st['v0'])} {_fmt(st['v1'])} {dr}")
        else:
            raise ValueError(f'unknown anim kind {k}')
    return out


def xp_vec(v):
    return (v[0], v[2], -v[1])


def export(target, parts, out_dir, name, textures, variants=None, lods=None):
    me = target.data
    me.calc_loop_triangles() if hasattr(me, 'calc_loop_triangles') else None
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.triangulate(bm, faces=bm.faces)
    uv = bm.loops.layers.uv.active
    part = bm.faces.layers.int.get('gse_part')
    smooth_normals = {}
    groups = {}
    for f in bm.faces:
        groups.setdefault(f[part] if part else 0, []).append(f)
    vt, index, keyed = [], [], {}
    batches = []
    order = sorted(groups, key=lambda i: (bool(parts[i].get('gse_anim')), i))
    for i in order:
        start = len(index)
        for f in groups[i]:
            corners = []
            for loop in f.loops:
                n = loop.vert.normal if f.smooth else f.normal
                key = (tuple(round(c, 5) for c in loop.vert.co), tuple(round(c, 3) for c in n), tuple(round(c, 5) for c in loop[uv].uv))
                idx = keyed.get(key)
                if idx is None:
                    idx = len(vt)
                    keyed[key] = idx
                    vt.append((xp(loop.vert.co), xp(n), loop[uv].uv))
                corners.append(idx)
            index.extend(corners)
        batches.append((i, start, len(index) - start))
    bm.free()

    body = []
    for (x, y, z), (nx, ny, nz), (s, t) in vt:
        body.append(f'VT {_fmt(x)} {_fmt(y)} {_fmt(z)} {_fmt(nx)} {_fmt(ny)} {_fmt(nz)} {_fmt(s)} {_fmt(t)}')
    full, rest = divmod(len(index), 10)
    for k in range(full):
        body.append('IDX10 ' + ' '.join(str(v) for v in index[k * 10:k * 10 + 10]))
    for v in index[full * 10:]:
        body.append(f'IDX {v}')
    body.append('')
    static = [(i, s, c) for i, s, c in batches if not parts[i].get('gse_anim')]
    if static:
        s0 = static[0][1]
        body.append(f'TRIS {s0} {sum(c for _, _, c in static)}')
    for i, s, c in batches:
        steps = parts[i].get('gse_anim')
        if not steps:
            continue
        steps = json.loads(steps)
        body.append('ANIM_begin')
        body.extend(_anim_lines(steps, '\t'))
        body.append(f'\tTRIS {s} {c}')
        body.append('ANIM_end')

    written = []
    for variant, albedo in [(None, textures['albedo'])] + sorted((variants or {}).items()):
        head = ['I', '800', 'OBJ', '', f"TEXTURE {os.path.basename(albedo)}"]
        if textures.get('normal'):
            head.append(f"TEXTURE_MAP normal {os.path.basename(textures['normal'])}")
        if textures.get('material'):
            head.append(f"TEXTURE_MAP material_gloss {os.path.basename(textures['material'])}")
        head += ['GLOBAL_specular 1.0', f'POINT_COUNTS {len(vt)} 0 0 {len(index)}', '']
        path = os.path.join(out_dir, f'{name}.obj' if variant is None else f'{name}_{variant}.obj')
        with open(path, 'w', encoding='utf-8', newline='\n') as fh:
            fh.write('\n'.join(head + body) + '\n')
        written.append(path)
    return written, len(vt), len(index) // 3
