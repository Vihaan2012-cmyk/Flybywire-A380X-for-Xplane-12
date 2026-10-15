"""3D paint kit for A380 for X-Plane 12: the exterior, the cabin, the flight deck
placards, the IFE look and the cabin mood lighting, in one Blender file.

blender -b --python build_kit.py -- <aircraft folder> <kit folder>
"""
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys

import bpy
import numpy as np

DDSTOOL = os.path.join(os.environ.get('CARGO_TARGET_DIR', 'D:/A380/fbw-build/target'), 'release', 'ddstool.exe')
COCKPIT = ['a380_cockpit_011.obj', 'a380_cockpit_082.obj']
IFE_IMAGES = {'ife_logo': (512, 256, (0.0, 0.0, 0.0, 0.0)), 'ife_background': (512, 256, (0.04, 0.09, 0.15, 1.0))}

aircraft, kit = sys.argv[sys.argv.index('--') + 1:][:2]
objects_dir = os.path.join(aircraft, 'objects')
stored = os.path.join(objects_dir, 'cabin_on')
white = os.path.join(aircraft, 'liveries', 'White (test)', 'objects')
paintable = {f for f in os.listdir(white) if f.lower().endswith('.dds')}
tex_dir = os.path.join(kit, 'textures')
paint_dir = os.path.join(tex_dir, 'paint')
ife_dir = os.path.join(tex_dir, 'ife')
os.makedirs(paint_dir, exist_ok=True)
os.makedirs(ife_dir, exist_ok=True)

bpy.ops.wm.read_factory_settings(use_empty=True)
acf = next(os.path.join(aircraft, f) for f in os.listdir(aircraft) if f.endswith('.acf') and 'Clean' not in f)
exterior = sorted(set(re.findall(r'^P _obja/\d+/_v10_att_file_stl (a380_exterior\S*\.obj)', open(acf, encoding='utf-8', errors='replace').read(), re.M)))
cabin = sorted(os.listdir(os.path.join(objects_dir, 'cabin_off')))
cabin += sorted(f for f in os.listdir(objects_dir) if (f.startswith('a380_cabin_mood_') and f.endswith('_0.obj')) or (f.startswith('a380_cabin_ife_') and f.endswith('.obj')))

images = {}
materials = {}
originals = {}


def decode_to_image(src, stem):
    raw = os.path.join(paint_dir, stem + '.rgba')
    info = subprocess.run([DDSTOOL, 'decode', src, raw], capture_output=True, text=True, check=True).stdout.split()
    w, h = int(info[0]), int(info[1])
    rgba8 = np.fromfile(raw, np.uint8).reshape(h, w, 4)[::-1]
    os.remove(raw)
    img = bpy.data.images.new(stem, w, h, alpha=True)
    img.pixels.foreach_set((rgba8.astype(np.float32) / 255.0).ravel())
    img.file_format = 'PNG'
    img.filepath_raw = os.path.join(paint_dir, stem + '.png')
    img.save()
    return img, rgba8


def image_for(texture, interior):
    if texture in images:
        return images[texture]
    stem = os.path.splitext(texture)[0]
    if not interior and texture in paintable:
        img, _ = decode_to_image(os.path.join(white, texture), stem)
    elif interior and not texture.startswith('solid_') and os.path.exists(os.path.join(objects_dir, texture)):
        img, rgba8 = decode_to_image(os.path.join(objects_dir, texture), stem)
        originals[stem] = hashlib.md5(np.ascontiguousarray(rgba8[::-1]).tobytes()).hexdigest()
    else:
        src = os.path.join(objects_dir, texture)
        if not os.path.exists(src):
            alt = [src[:-4] + e for e in ('.dds', '.png') if os.path.exists(src[:-4] + e)]
            if not alt:
                images[texture] = None
                return None
            src = alt[0]
        dst = os.path.join(tex_dir, os.path.basename(src))
        if not os.path.exists(dst):
            shutil.copy2(src, dst)
        img = bpy.data.images.load(dst)
    images[texture] = img
    return img


def material_for(texture, interior):
    key = (texture, interior)
    if key in materials:
        return materials[key]
    mat = bpy.data.materials.new(os.path.splitext(texture)[0] if texture else 'untextured')
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes['Principled BSDF']
    img = image_for(texture, interior) if texture else None
    if img is not None:
        node = mat.node_tree.nodes.new('ShaderNodeTexImage')
        node.image = img
        node.interpolation = 'Linear'
        mat.node_tree.links.new(node.outputs['Color'], bsdf.inputs['Base Color'])
        mat.node_tree.nodes.active = node
    materials[key] = mat
    return mat


def collection(name):
    c = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(c)
    return c


paint_col = collection('Exterior - paintable')
other_col = collection('Exterior - reference')
cabin_col = collection('Cabin')
deck_col = collection('Flight deck placards')


def load(path, interior, pick):
    vt, idx, ranges, texture, lods = [], [], [], None, 0
    for line in open(path, encoding='latin-1'):
        p = line.split()
        if not p:
            continue
        k = p[0]
        if k == 'VT':
            vt.append([float(x) for x in p[1:9]])
        elif k in ('IDX', 'IDX10'):
            idx.extend(int(x) for x in p[1:])
        elif k == 'TEXTURE' and len(p) > 1:
            texture = p[1]
        elif k == 'ATTR_LOD':
            lods += 1
        elif k == 'TRIS' and lods <= 1:
            ranges.append((int(p[1]), int(p[2])))
    if not ranges:
        return
    name = os.path.basename(path)[:-4]
    vt = np.array(vt, dtype=np.float32)
    idx = np.array(idx, dtype=np.int64)
    tri = np.concatenate([idx[s:s + n] for s, n in ranges])
    used, local = np.unique(tri, return_inverse=True)
    pos = vt[used, 0:3]
    co = np.stack([pos[:, 0], -pos[:, 2], pos[:, 1]], axis=1)
    mesh = bpy.data.meshes.new(name)
    mesh.vertices.add(len(co))
    mesh.vertices.foreach_set('co', co.ravel())
    mesh.loops.add(len(local))
    mesh.loops.foreach_set('vertex_index', local.astype(np.int32))
    mesh.polygons.add(len(local) // 3)
    mesh.polygons.foreach_set('loop_start', np.arange(0, len(local), 3, dtype=np.int32))
    uv = mesh.uv_layers.new(name='UVMap')
    uv.data.foreach_set('uv', vt[used[local], 6:8].ravel())
    mesh.update(calc_edges=True)
    mesh.polygons.foreach_set('use_smooth', np.ones(len(mesh.polygons), dtype=bool))
    mesh.materials.append(material_for(texture, interior))
    obj = bpy.data.objects.new(name, mesh)
    pick(texture).objects.link(obj)
    print(f'{name}: {len(mesh.polygons)} triangles, {texture}')


for name in exterior:
    load(os.path.join(objects_dir, name), False, lambda t: paint_col if t in paintable else other_col)
for name in cabin:
    full = os.path.join(stored, name)
    load(full if os.path.exists(full) else os.path.join(objects_dir, name), True, lambda t: cabin_col)
for name in COCKPIT:
    load(os.path.join(objects_dir, name), True, lambda t: deck_col)

for name, (w, h, rgba) in IFE_IMAGES.items():
    img = bpy.data.images.new(name, w, h, alpha=True)
    img.pixels.foreach_set(np.tile(np.array(rgba, dtype=np.float32), w * h))
    img.file_format = 'PNG'
    img.filepath_raw = os.path.join(ife_dir, name + '.png')
    img.save()
    img.use_fake_user = True

here = os.path.dirname(os.path.abspath(__file__))
scripts = os.path.join(kit, 'scripts')
os.makedirs(scripts, exist_ok=True)
shutil.copy2(os.path.join(here, '..', 'paintkit', 'dds.py'), scripts)
shutil.copy2(os.path.join(here, 'export_livery.py'), scripts)
for f in ('HOW TO PAINT.txt', 'NOTICE.txt', 'cabin.json'):
    shutil.copy2(os.path.join(here, f), kit)
bpy.data.texts.load(os.path.join(scripts, 'export_livery.py'))
bpy.data.texts.load(os.path.join(kit, 'cabin.json'))
bpy.data.texts.new('kit_originals.json').write(json.dumps(originals, indent=1))
blend = os.path.join(kit, 'A380 3D Paint Kit.blend')
bpy.ops.wm.save_as_mainfile(filepath=blend)
bpy.ops.file.make_paths_relative()
bpy.ops.wm.save_mainfile(filepath=blend)
print('saved', len(exterior), 'exterior and', len(cabin) + len(COCKPIT), 'interior objects')
