import os
import sys

obj_dir = sys.argv[1]
names = sys.argv[2:]
HIDE = ['ANIM_begin', 'ANIM_hide 0.5 1.5 fbw/anim/eye_in_flight_deck']

for name in names:
    path = os.path.join(obj_dir, name)
    lines = open(path, encoding='utf-8', errors='replace').read().split('\n')
    if any('fbw/anim/eye_in_flight_deck' in l for l in lines):
        print(name, 'already wrapped')
        continue
    if any(l.lstrip().startswith(('LIGHT', 'VLIGHT')) for l in lines):
        print(name, 'has lights, skipped')
        continue
    last_data = max(i for i, l in enumerate(lines) if l.startswith(('VT', 'IDX', 'POINT_COUNTS')))
    out = lines[:last_data + 1]
    opened = False
    for line in lines[last_data + 1:]:
        word = line.split()[0] if line.split() else ''
        if word == 'ATTR_LOD':
            if opened:
                out.append('ANIM_end')
            out.append(line)
            out.extend(HIDE)
            opened = True
            continue
        if word and not opened:
            out.extend(HIDE)
            opened = True
        out.append(line)
    while out and out[-1].strip() == '':
        out.pop()
    if opened:
        out.append('ANIM_end')
    out.append('')
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(out))
    print(name, 'wrapped')
