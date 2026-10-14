"""3D paint kit for A380 for X-Plane 12: the exterior in Blender, ready for
Texture Paint on the livery sheets.

blender -b --python build_kit.py -- <aircraft folder> <kit folder>
"""
import os
import re
import shutil
import subprocess
import sys

import bpy
import numpy as np

DDSTOOL = os.path.join(os.environ.get('CARGO_TARGET_DIR', 'D:/A380/fbw-build/target'), 'release', 'ddstool.exe')

aircraft, kit = sys.argv[sys.argv.index('--') + 1:][:2]
objects_dir = os.path.join(aircraft, 'objects')
white = os.path.join(aircraft, 'liveries', 'White (test)', 'objects')
paintable = {f for f in os.listdir(white) if f.lower().endswith('.dds')}
tex_dir = os.path.join(kit, 'textures')
paint_dir = os.path.join(tex_dir, 'paint')
os.makedirs(paint_dir, exist_ok=True)

bpy.ops.wm.read_factory_settings(use_empty=True)
acf = next(os.path.join(aircraft, f) for f in os.listdir(aircraft) if f.endswith('.acf') and 'Clean' not in f)
names = sorted(set(re.findall(r'^P _obja/\d+/_v10_att_file_stl (a380_exterior\S*\.obj)', open(acf, encoding='utf-8', errors='replace').read(), re.M)))

images = {}
materials = {}


def image_for(texture):
    if texture in images:
        return images[texture]
    stem = os.path.splitext(texture)[0]
    if texture in paintable:
        raw = os.path.join(paint_dir, stem + '.rgba')
        info = subprocess.run([DDSTOOL, 'decode', os.path.join(white, texture), raw], capture_output=True, text=True, check=True).stdout.split()
        w, h = int(info[0]), int(info[1])
        rgba = np.fromfile(raw, np.uint8).reshape(h, w, 4)[::-1].astype(np.float32) / 255.0
        os.remove(raw)
        img = bpy.data.images.new(stem, w, h, alpha=True)
        img.pixels.foreach_set(rgba.ravel())
        img.file_format = 'PNG'
        img.filepath_raw = os.path.join(paint_dir, stem + '.png')
        img.save()
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


def material_for(texture):
    if texture in materials:
        return materials[texture]
    mat = bpy.data.materials.new(os.path.splitext(texture)[0] if texture else 'untextured')
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes['Principled BSDF']
    img = image_for(texture) if texture else None
    if img is not None:
        node = mat.node_tree.nodes.new('ShaderNodeTexImage')
        node.image = img
        node.interpolation = 'Linear'
        mat.node_tree.links.new(node.outputs['Color'], bsdf.inputs['Base Color'])
        mat.node_tree.nodes.active = node
    materials[texture] = mat
    return mat


paint_col = bpy.data.collections.new('Paintable')
other_col = bpy.data.collections.new('Reference')
bpy.context.scene.collection.children.link(paint_col)
bpy.context.scene.collection.children.link(other_col)

for name in names:
    path = os.path.join(objects_dir, name)
    vt, idx, ranges, texture, lods = [], [], [], None, 0
    for line in open(path, encoding='utf-8', errors='replace'):
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
        continue
    vt = np.array(vt, dtype=np.float32)
    idx = np.array(idx, dtype=np.int64)
    tri = np.concatenate([idx[s:s + n] for s, n in ranges])
    used, local = np.unique(tri, return_inverse=True)
    pos = vt[used, 0:3]
    co = np.stack([pos[:, 0], -pos[:, 2], pos[:, 1]], axis=1)
    mesh = bpy.data.meshes.new(name[:-4])
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
    mesh.materials.append(material_for(texture))
    obj = bpy.data.objects.new(name[:-4], mesh)
    (paint_col if texture in paintable else other_col).objects.link(obj)
    print(f'{name}: {len(mesh.polygons)} triangles, {texture}')

here = os.path.dirname(os.path.abspath(__file__))
scripts = os.path.join(kit, 'scripts')
os.makedirs(scripts, exist_ok=True)
shutil.copy2(os.path.join(here, '..', 'paintkit', 'dds.py'), scripts)
shutil.copy2(os.path.join(here, 'export_livery.py'), scripts)
shutil.copy2(os.path.join(here, 'HOW TO PAINT.txt'), kit)
bpy.data.texts.load(os.path.join(scripts, 'export_livery.py'))
blend = os.path.join(kit, 'A380 3D Paint Kit.blend')
bpy.ops.wm.save_as_mainfile(filepath=blend)
bpy.ops.file.make_paths_relative()
bpy.ops.wm.save_mainfile(filepath=blend)
print('saved', len(names), 'objects')
