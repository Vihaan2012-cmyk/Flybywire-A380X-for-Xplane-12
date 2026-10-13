import collections
import json
import os
import re
import subprocess
import sys

import numpy as np
from PIL import Image, ImageFilter

import ddsmips
from bake import TEXPIPE, decode_installed, light_canon, obj_path


def rewrite(lines, drop=None, glow=None, lit=None):
    drop = drop or {}
    glow = glow or {}
    pc_at = next(i for i, l in enumerate(lines) if l.startswith('POINT_COUNTS'))
    header = list(lines[:pc_at])
    if lit:
        header = [l for l in header if not l.startswith('TEXTURE_LIT')]
        tex_at = next(i for i, l in enumerate(header) if l.startswith('TEXTURE '))
        header.insert(tex_at + 1, f'TEXTURE_LIT {lit}')
    vts, idx, cmds = [], [], []
    for l in lines[pc_at + 1:]:
        if l.startswith('VT'):
            vts.append(l)
        elif l.startswith('IDX'):
            idx.extend(int(x) for x in l.split()[1:])
        else:
            cmds.append(l)
    new_idx = []
    body = []
    batch = 0
    state = None
    for l in cmds:
        t = l.split()
        k = t[0] if t else ''
        if k == 'ATTR_light_level':
            state = l
        elif k == 'ATTR_light_level_reset':
            state = None
        if k != 'TRIS':
            body.append(l)
            continue
        off, cnt = int(t[1]), int(t[2])
        tri = np.asarray(idx[off:off + cnt]).reshape(-1, 3)
        gone = drop.get(batch, set())
        groups = glow.get(batch, {})
        plain = [i for i in range(len(tri)) if i not in gone and i not in groups]
        if plain:
            body.append(f'TRIS {len(new_idx)} {len(plain) * 3}')
            new_idx.extend(tri[plain].ravel().tolist())
        by_group = collections.defaultdict(list)
        for i, g in groups.items():
            if i not in gone:
                by_group[g].append(i)
        for g, ids in sorted(by_group.items()):
            body.append(f'ATTR_light_level 0 1 {g}')
            body.append(f'TRIS {len(new_idx)} {len(ids) * 3}')
            new_idx.extend(tri[sorted(ids)].ravel().tolist())
        if by_group:
            body.append(state if state else 'ATTR_light_level_reset')
        batch += 1
    c = lines[pc_at].split()[1:]
    out = header + [f'POINT_COUNTS {c[0]} {c[1]} {c[2]} {len(new_idx)}'] + vts
    tens = len(new_idx) // 10 * 10
    out += ['IDX10 ' + ' '.join(map(str, new_idx[i:i + 10])) for i in range(0, tens, 10)]
    out += [f'IDX {v}' for v in new_idx[tens:]]
    return out + body


def main(objdir, acf_path, work, stage, main_lua):
    res = json.load(open(os.path.join(work, 'bake_result.json')))
    done = set(res['results'])
    canon = light_canon(main_lua)
    acf = open(acf_path, encoding='utf-8', errors='replace').read()
    in_acf = {os.path.splitext(f)[0] for f in re.findall(r'_v10_att_file_stl (\S+)', acf)}

    for dn, entries in res['baked'].items():
        drop = collections.defaultdict(set)
        for b, k, texs in entries:
            if all(t in done for t in texs):
                drop[b].add(k)
        if not drop:
            continue
        path = obj_path(objdir, dn + '.obj')
        lines = open(path, encoding='utf-8').read().splitlines()
        out = rewrite(lines, drop=drop)
        open(os.path.join(stage, dn + '.obj'), 'w', encoding='utf-8', newline='\n').write('\n'.join(out) + '\n')
        print(f'{dn}: removed {sum(len(v) for v in drop.values())} baked triangles', flush=True)

    for obj, entries in res['glow_tris'].items():
        if obj not in in_acf:
            continue
        glow = collections.defaultdict(dict)
        for b, k, g, tex in entries:
            if tex in done:
                glow[b][k] = canon.get(g, g)
        if not glow:
            continue
        path = obj_path(objdir, obj + '.obj')
        lines = open(path, encoding='utf-8').read().splitlines()
        tex = next(l.split()[1] for l in lines if l.startswith('TEXTURE '))
        stem = os.path.splitext(tex)[0]
        glow_png = os.path.join(work, f'{stem}_TXTLIT.png')
        if not res['results'].get(tex, {}).get('lit'):
            print(f'{obj}: no glow in {tex}, left alone', flush=True)
            continue
        old_lit = next((l.split()[1] for l in lines if l.startswith('TEXTURE_LIT')), None)
        if old_lit:
            lit = os.path.splitext(old_lit)[0] + '_TXT.dds'
            if not os.path.exists(os.path.join(stage, lit)):
                g = np.asarray(Image.open(glow_png).convert('RGB')).astype(np.float32)
                e = decode_installed(obj_path(objdir, old_lit))[..., :3]
                e = np.asarray(Image.fromarray(e).resize((g.shape[1], g.shape[0]), Image.BICUBIC)).astype(np.float32)
                lamps = ((e.max(axis=2) - e.min(axis=2)) > 40) & (e.max(axis=2) > 60)
                lamps = np.asarray(Image.fromarray((lamps * 255).astype(np.uint8)).filter(ImageFilter.MaxFilter(5))) > 0
                e[lamps] = 0
                merged = np.clip(np.maximum(e, g), 0, 255).astype(np.uint8)
                mp = os.path.join(work, os.path.splitext(lit)[0] + '.png')
                Image.fromarray(merged, 'RGB').save(mp)
                subprocess.run([TEXPIPE, 'dds', mp, os.path.join(stage, lit)], check=True, capture_output=True)
        else:
            lit = res['results'][tex]['lit']
        out = rewrite(lines, glow=glow, lit=lit)
        open(os.path.join(stage, obj + '.obj'), 'w', encoding='utf-8', newline='\n').write('\n'.join(out) + '\n')
        groups = collections.Counter(g for b in glow.values() for g in b.values())
        print(f'{obj}: lit {lit}, glow triangles {dict(groups)}', flush=True)


if __name__ == '__main__':
    main(*sys.argv[1:6])
