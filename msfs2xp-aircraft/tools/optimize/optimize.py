"""Post-conversion optimisation of a converted A380: decal sheet repack,
cabin-free object variants, small-part mesh simplification, texture sizing to
what the pilot's eye resolves, exterior objects hidden from the flight deck, and
each engine hidden when it separates.

optimize.py <aircraft dir> [--dry-run]
Each step records itself in objects/optimize_state.json and runs once.
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
TARGET = os.environ.get('CARGO_TARGET_DIR', 'D:/A380/fbw-build/target')
DDSTOOL = os.path.join(TARGET, 'release', 'ddstool.exe')
OBJDECIMATE = os.path.join(TARGET, 'release', 'objdecimate.exe')
CABIN = 'fbw/options/cabin_visible'
SMALL_PARTS = re.compile(r'KNOB|SCREW|KEYBOARD')
LETTERING = re.compile(r'KNOB|PUSHBUTTON|STICKER|PUSH_TEXT|_TXT')
GEAR = re.compile(r'TIRES|_NLG_|_WLG_|_BLG_|LG_BAY')
FT = 0.3048

aircraft = sys.argv[1]
dry = '--dry-run' in sys.argv
objects = os.path.join(aircraft, 'objects')
state_path = os.path.join(objects, 'optimize_state.json')
state = json.load(open(state_path)) if os.path.exists(state_path) else {}
acf_text = open(next(os.path.join(aircraft, f) for f in os.listdir(aircraft) if f.endswith('.acf')), encoding='utf-8', errors='replace').read()
acf = {}
for m in re.finditer(r'^P _obja/(\d+)/(\S+) (.*)$', acf_text, re.M):
    acf.setdefault(int(m.group(1)), {})[m.group(2)] = m.group(3).strip()
loaded = {o['_v10_att_file_stl']: int(o.get('_obj_flags', '0') or 0) for o in acf.values() if o.get('_v10_att_file_stl')}
eye = [float(re.search(rf'^P acf/_pe_xyz/{i} (\S+)', acf_text, re.M).group(1)) * FT for i in range(3)]


def run(*args):
    print('  $', ' '.join(os.path.basename(a) if os.path.isabs(a) else a for a in args))
    if not dry:
        subprocess.run([sys.executable, '-I', *args] if args[0].endswith('.py') else list(args), check=True)


def done(step):
    if not dry:
        state[step] = True
        json.dump(state, open(state_path, 'w'), indent=1)


def text(name):
    return open(os.path.join(objects, name), encoding='utf-8', errors='replace').read()


def build_tools():
    if dry:
        return
    env = dict(os.environ, CARGO_TARGET_DIR=TARGET)
    for d in ('optimize/ddstool', 'objdecimate'):
        subprocess.run(['cargo', 'build', '--release', '--manifest-path', os.path.join(REPO, 'tools', d, 'Cargo.toml')], check=True, env=env)


build_tools()

if not state.get('decals'):
    print('decal sheets')
    work = tempfile.mkdtemp()
    run(os.path.join(HERE, 'decal_repack.py'), objects, work, DDSTOOL)
    if not dry:
        for f in os.listdir(work):
            shutil.move(os.path.join(work, f), os.path.join(objects, f))
        lines = text('a380_cockpit_021.obj').replace('TEXTURE A380_COCKPIT_DECAL_ALBEDO_K000000_DECAL.dds', 'TEXTURE solid_00000000.png')
        open(os.path.join(objects, 'a380_cockpit_021.obj'), 'w', encoding='utf-8', newline='\n').write(lines)
    shutil.rmtree(work, ignore_errors=True)
    done('decals')

if not state.get('cabin'):
    cabin = [n for n in loaded if n.startswith('a380_cockpit') and CABIN in text(n)]
    print('cabin-free variants:', len(cabin))
    run(os.path.join(HERE, 'cabin_strip.py'), objects, os.path.join(objects, 'cabin_off'), *cabin)
    done('cabin')

if not state.get('small_parts'):
    parts = [n for n in loaded if n.startswith('a380_cockpit') and SMALL_PARTS.search(' '.join(re.findall(r'^TEXTURE\S* (\S+)', text(n), re.M)))]
    print('small parts simplified:', len(parts))
    for n in parts:
        path = os.path.join(objects, n)
        run(OBJDECIMATE, path, path, '0:1000:0.00015:keep')
    done('small_parts')

if not state.get('exterior_hide'):
    hide = []
    for n, flags in sorted(loaded.items()):
        if not n.startswith('a380_exterior') or flags & 0x8:
            continue
        vts = [l.split() for l in text(n).splitlines() if l.startswith('VT')]
        if not vts:
            continue
        ymax = max(float(v[2]) for v in vts)
        zmin = min(float(v[3]) for v in vts)
        gear = GEAR.search(' '.join(re.findall(r'^TEXTURE\S* (\S+)', text(n), re.M)))
        if zmin > eye[2] + 2.0 or ymax < eye[1] - 2.5 or gear:
            hide.append(n)
    print('exterior hidden from the flight deck:', len(hide))
    run(os.path.join(HERE, 'hide_exterior.py'), objects, *hide)
    done('exterior_hide')

def full_cabin_dir():
    stored = os.path.join(objects, 'cabin_on')
    return stored if os.path.isdir(stored) and any(f.endswith('.obj') for f in os.listdir(stored)) else objects


def add_objects(names, internal):
    for f in os.listdir(aircraft):
        if not f.endswith('.acf'):
            continue
        path = os.path.join(aircraft, f)
        lines = open(path, encoding='latin-1').read().split('\n')
        listed = {l.split()[2] for l in lines if re.match(r'^P _obja/\d+/_v10_att_file_stl ', l)}
        count_at = next(i for i, l in enumerate(lines) if l.startswith('P _obja/count '))
        count = int(lines[count_at].split()[2])
        last = max(i for i, l in enumerate(lines) if l.startswith('P _obja/'))
        new = []
        for name in names:
            if name in listed:
                continue
            fields = {'_obj_flags': '13', '_v10_att_body': '-1', '_v10_att_file_stl': name, '_v10_att_gear': '-1', '_v10_att_phi_ref': '0.000000000', '_v10_att_psi_ref': '0.000000000', '_v10_att_the_ref': '0.000000000', '_v10_att_wing': '-1', '_v10_att_x_acf_prt_ref': '0.000000000', '_v10_att_y_acf_prt_ref': '0.000000000', '_v10_att_z_acf_prt_ref': '0.000000000', '_v10_is_internal': '1' if internal else '0', '_v10_steers_with_gear': '0'}
            new += [f'P _obja/{count}/{k} {v}' for k, v in fields.items()]
            count += 1
        if new and not dry:
            lines[count_at] = f'P _obja/count {count}'
            lines[last + 1:last + 1] = new
            open(path, 'w', encoding='latin-1', newline='').write('\n'.join(lines))
        print(f'  {f}: {len(new) // 13} objects added')


if not state.get('cabin_ife'):
    print('cabin IFE screens')
    run(os.path.join(HERE, 'ife_screens.py'), full_cabin_dir(), objects)
    add_objects(sorted(f for f in os.listdir(objects) if f.startswith('a380_cabin_ife_') and f.endswith('.obj')), True)
    done('cabin_ife')

if not state.get('cabin_mood'):
    print('cabin mood lighting')
    run(os.path.join(HERE, 'cabin_mood.py'), full_cabin_dir(), objects, DDSTOOL)
    add_objects(sorted(f for f in os.listdir(objects) if f.startswith('a380_cabin_mood_') and f.endswith('.obj')), False)
    add_objects(['a380_cabin_lights.obj'], True)
    done('cabin_mood')

if not state.get('engine_separation'):
    exterior = [n for n in sorted(loaded) if n.startswith('a380_exterior')]
    print('engines hidden on separation')
    run(os.path.join(HERE, 'engine_separation.py'), objects, *exterior)
    done('engine_separation')

if not state.get('texel_sizing'):
    print('texture sizing')
    out = subprocess.run([sys.executable, '-I', os.path.join(HERE, 'texel_density.py'), aircraft], capture_output=True, text=True, check=True).stdout
    pick = []
    for line in out.splitlines():
        p = line.split()
        if len(p) > 8 and p[8] in ('x1', 'x2') and not LETTERING.search(p[2]):
            pick.append(f'{p[2]} {p[8][1:]}')
    print('  textures resized:', len(pick))
    with tempfile.TemporaryDirectory() as tmp:
        listing = os.path.join(tmp, 'halve_list.txt')
        open(listing, 'w').write('\n'.join(pick) + '\n')
        originals = os.path.abspath(aircraft) + ' optimize originals'
        if pick:
            run(os.path.join(HERE, 'halve_textures.py'), objects, originals, DDSTOOL, listing)
    done('texel_sizing')
