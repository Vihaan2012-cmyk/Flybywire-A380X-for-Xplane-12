import os
import sys

src_dir, out_dir = sys.argv[1], sys.argv[2]
names = sys.argv[3:]
os.makedirs(out_dir, exist_ok=True)
CABIN = 'fbw/options/cabin_visible'

for name in names:
    lines = open(os.path.join(src_dir, name), encoding='utf-8', errors='replace').read().split('\n')
    header, vts, idx, body = [], [], [], []
    for line in lines:
        p = line.split()
        if p and p[0] == 'VT':
            vts.append(line)
        elif p and p[0] in ('IDX', 'IDX10'):
            idx.extend(int(x) for x in p[1:])
        elif p and p[0] in ('VLINE', 'VLIGHT', 'LINES', 'LIGHTS'):
            raise SystemExit(f'{name}: has {p[0]}, not handled')
        elif not vts and not idx and not body and (not p or p[0] not in ('ATTR_LOD',) and not p[0].startswith(('ATTR_', 'ANIM_', 'TRIS'))):
            header.append(line)
        else:
            body.append(line)

    stack, keep = [], []
    for line in body:
        p = line.split()
        if not p:
            keep.append(('line', line))
            continue
        k = p[0]
        if k == 'ANIM_begin':
            stack.append(stack[-1] if stack else False)
        elif k == 'ANIM_end':
            stack.pop()
        elif k in ('ANIM_show', 'ANIM_hide') and p[-1] == CABIN and stack:
            stack[-1] = True
        if k == 'TRIS':
            if stack and stack[-1]:
                continue
            keep.append(('tris', int(p[1]), int(p[2])))
        else:
            keep.append(('line', line))

    remap, new_vts, new_idx, out_body = {}, [], [], []
    for item in keep:
        if item[0] == 'line':
            out_body.append(item[1])
            continue
        _, start, count = item
        offset = len(new_idx)
        for i in idx[start:start + count]:
            if i not in remap:
                remap[i] = len(new_vts)
                new_vts.append(vts[i])
            new_idx.append(remap[i])
        out_body.append(f'TRIS {offset} {count}')

    if not new_idx:
        with open(os.path.join(out_dir, name), 'w', encoding='utf-8', newline=chr(10)) as f:
            f.write(chr(10).join(['I', '800', 'OBJ', '', 'TEXTURE solid_00000000.png', 'POINT_COUNTS 0 0 0 0', '']))
        print(f'{name:24} {len(vts):>9,} -> empty object')
        continue
    out = []
    for line in header:
        if line.startswith('POINT_COUNTS'):
            out.append(f'POINT_COUNTS {len(new_vts)} 0 0 {len(new_idx)}')
        else:
            out.append(line)
    out.extend(new_vts)
    for i in range(0, len(new_idx) - len(new_idx) % 10, 10):
        out.append('IDX10 ' + ' '.join(str(x) for x in new_idx[i:i + 10]))
    for x in new_idx[len(new_idx) - len(new_idx) % 10:]:
        out.append(f'IDX {x}')
    out.extend(out_body)
    with open(os.path.join(out_dir, name), 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(out))
    print(f'{name:24} {len(vts):>9,} -> {len(new_vts):>7,} vertices, {len(idx) // 3:>9,} -> {len(new_idx) // 3:>7,} triangles')
