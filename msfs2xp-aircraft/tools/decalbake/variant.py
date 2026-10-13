import hashlib
import os
import re
import shutil
import sys

from unshare import header

GRAIN = ('LEATHER', 'CARPET', 'SEAT', 'FLOOR', 'CABLES', 'STAIRS', 'TEXTILE')
ACF_NAME = 'FlyByWire A380X Clean Cockpit.acf'


def clean_name(f):
    stem, ext = os.path.splitext(f)
    return f'{stem}_CLEAN{ext}'


def grain(f):
    return any(g in f.upper() for g in GRAIN)


def sha(p):
    return hashlib.sha1(open(p, 'rb').read()).hexdigest()


def main(aircraft, stage_clean, stage_baked, baked_list, out):
    objdir = os.path.join(aircraft, 'objects')
    os.makedirs(os.path.join(out, 'objects'), exist_ok=True)
    baked = set(open(baked_list).read().split())
    rename = {}
    for f in os.listdir(stage_clean):
        if not grain(f):
            rename[f] = os.path.join(stage_clean, f)
    for f in baked:
        rename[f] = os.path.join(stage_baked, f)
    for f, src in rename.items():
        shutil.copyfile(src, os.path.join(out, 'objects', clean_name(f)))

    for f in os.listdir(stage_baked):
        if f.endswith('.obj') and sha(os.path.join(stage_baked, f)) != sha(os.path.join(objdir, f)):
            print(f'WARNING {f}: the clean bake wrote a different OBJ than the installed one', flush=True)

    acf_path = os.path.join(aircraft, 'FlyByWire A380X.acf')
    raw = open(acf_path, 'rb').read()
    names = [os.path.splitext(f.decode())[0] for f in re.findall(rb'_v10_att_file_stl (\S+)', raw)]
    swapped = 0
    for n in names:
        if not n.startswith('a380_cockpit_'):
            continue
        path = os.path.join(objdir, n + '.obj')
        h = header(path)
        if h.get('TEXTURE') not in rename and h.get('material_gloss') not in rename:
            continue
        lines = open(path, encoding='utf-8').read().split('\n')
        for i, l in enumerate(lines):
            t = l.split()
            if not t:
                continue
            if t[0] == 'POINT_COUNTS':
                break
            if t[0] == 'TEXTURE' and t[1] in rename:
                lines[i] = f'TEXTURE {clean_name(t[1])}'
            elif t[0] == 'TEXTURE_MAP' and t[1] == 'material_gloss' and t[2] in rename:
                lines[i] = f'TEXTURE_MAP material_gloss {clean_name(t[2])}'
        open(os.path.join(out, 'objects', n + '_clean.obj'), 'w', encoding='utf-8', newline='\n').write('\n'.join(lines))
        raw = raw.replace(f'_v10_att_file_stl {n}.obj'.encode(), f'_v10_att_file_stl {n}_clean.obj'.encode())
        swapped += 1
    raw = re.sub(rb'P acf/_name [^\r\n]*', b'P acf/_name FlyByWire A380X (clean cockpit)', raw)
    raw = re.sub(rb'P acf/_descrip [^\r\n]*', b'P acf/_descrip FlyByWire A380X converted from MSFS, cockpit without wear', raw)
    open(os.path.join(out, ACF_NAME), 'wb').write(raw)
    print(f'{len(rename)} clean textures, {swapped} cockpit objects swapped into {ACF_NAME}', flush=True)


if __name__ == '__main__':
    main(*sys.argv[1:6])
