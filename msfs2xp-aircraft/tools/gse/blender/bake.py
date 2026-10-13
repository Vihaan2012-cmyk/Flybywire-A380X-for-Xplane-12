import os

import bmesh
import bpy
import numpy as np


def is_decal(obj):
    return any(m is not None and 'gse_decal' in m for m in obj.data.materials)


def export_objects(coll):
    return [o for o in coll.all_objects if o.type == 'MESH' and not is_decal(o)]


def decal_objects(coll):
    return [o for o in coll.all_objects if o.type == 'MESH' and is_decal(o)]


def _select(objs, active):
    bpy.ops.object.select_all(action='DESELECT')
    for o in objs:
        o.hide_set(False)
        o.hide_render = False
        o.select_set(True)
    bpy.context.view_layer.objects.active = active


def build_target(coll, name):
    parts = export_objects(coll)
    copies = []
    for i, o in enumerate(parts):
        me = o.data.copy()
        me.transform(o.matrix_world)
        attr = me.attributes.new('gse_part', 'INT', 'FACE')
        for k in range(len(me.polygons)):
            attr.data[k].value = i
        c = bpy.data.objects.new(f'_bake_{i}', me)
        bpy.context.scene.collection.objects.link(c)
        copies.append(c)
    _select(copies, copies[0])
    bpy.ops.object.join()
    target = bpy.context.view_layer.objects.active
    target.name = f'{name}_target'
    target.data.materials.clear()
    for uv in list(target.data.uv_layers):
        target.data.uv_layers.remove(uv)
    target.data.uv_layers.new(name='UVMap')
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.uv.smart_project(angle_limit=1.15, island_margin=0.003, area_weight=0.0, scale_to_bounds=False)
    bpy.ops.uv.pack_islands(margin=0.003, rotate=True, shape_method='CONCAVE')
    bpy.ops.object.mode_set(mode='OBJECT')
    return target, parts


def _image(name, size, color, non_color):
    img = bpy.data.images.new(name, size, size, alpha=False)
    img.generated_color = color
    img.colorspace_settings.name = 'Non-Color' if non_color else 'sRGB'
    return img


def _target_material(target, img):
    m = bpy.data.materials.get('_bake_target') or bpy.data.materials.new('_bake_target')
    m.use_nodes = True
    nt = m.node_tree
    node = nt.nodes.get('_bake_image') or nt.nodes.new('ShaderNodeTexImage')
    node.name = '_bake_image'
    node.image = img
    nt.nodes.active = node
    target.data.materials.clear()
    target.data.materials.append(m)


def _emission_from(mat, value=None, image_alpha=False, image_color=False):
    nt = mat.node_tree
    p = nt.nodes.get('Principled BSDF')
    for link in list(p.inputs['Emission Color'].links):
        nt.links.remove(link)
    if value is not None:
        p.inputs['Emission Color'].default_value = (value, value, value, 1.0)
    else:
        tex = next(n for n in nt.nodes if n.type == 'TEX_IMAGE')
        nt.links.new(tex.outputs['Alpha' if image_alpha else 'Color'], p.inputs['Emission Color'])
    p.inputs['Emission Strength'].default_value = 1.0


def _clear_emission(mat):
    nt = mat.node_tree
    p = nt.nodes.get('Principled BSDF')
    for link in list(p.inputs['Emission Color'].links):
        nt.links.remove(link)
    p.inputs['Emission Strength'].default_value = 0.0


def _bake(kind, sources, target, img, selected_to_active=True, pass_filter=None, samples=1):
    _target_material(target, img)
    bpy.context.scene.cycles.samples = samples
    _select(sources + [target], target)
    scene = bpy.context.scene
    scene.render.bake.use_selected_to_active = selected_to_active
    scene.render.bake.cage_extrusion = 0.006
    scene.render.bake.max_ray_distance = 0.014
    scene.render.bake.margin = 12
    kwargs = dict(type=kind, use_selected_to_active=selected_to_active, cage_extrusion=0.006, max_ray_distance=0.014, margin=12)
    if pass_filter:
        kwargs['pass_filter'] = pass_filter
    bpy.ops.object.bake(**kwargs)


def _pixels(img):
    a = np.empty(len(img.pixels), dtype=np.float32)
    img.pixels.foreach_get(a)
    return a.reshape(img.size[1], img.size[0], 4)


def _save(arr, path, non_color):
    h, w = arr.shape[:2]
    img = bpy.data.images.new(os.path.basename(path), w, h, alpha=False)
    img.colorspace_settings.name = 'Non-Color' if non_color else 'sRGB'
    rgba = np.ones((h, w, 4), dtype=np.float32)
    rgba[..., :3] = np.clip(arr[..., :3], 0, 1)
    img.pixels.foreach_set(rgba.ravel())
    img.filepath_raw = path
    img.file_format = 'PNG'
    img.save()


def bake(coll, out_dir, name, size=2048, samples=32, ao_strength=0.75):
    os.makedirs(out_dir, exist_ok=True)
    scene = bpy.context.scene
    scene.render.engine = 'CYCLES'
    if scene.world is None:
        scene.world = bpy.data.worlds.new('World')
    scene.world.use_nodes = True
    scene.cycles.use_denoising = False
    if os.environ.get('GSE_CPU'):
        scene.cycles.device = 'CPU'
        scene.render.threads_mode = 'FIXED'
        scene.render.threads = int(os.environ.get('GSE_THREADS', '2'))
    else:
        try:
            prefs = bpy.context.preferences.addons['cycles'].preferences
            prefs.compute_device_type = 'OPTIX'
            prefs.get_devices()
            for d in prefs.devices:
                d.use = d.type == 'OPTIX'
            scene.cycles.device = 'GPU'
        except Exception:
            pass
    target, parts = build_target(coll, name)
    decals = decal_objects(coll)
    mats = {m for o in parts for m in o.data.materials if m is not None}

    base = _image(f'{name}_base', size, (0.5, 0.5, 0.5, 1.0), False)
    _bake('DIFFUSE', parts, target, base, pass_filter={'COLOR'})
    albedo = _pixels(base)[..., :3]

    if decals:
        dmats = {m for o in decals for m in o.data.materials if m is not None}
        dcol = _image(f'{name}_dcol', size, (0, 0, 0, 1), False)
        dalpha = _image(f'{name}_dalpha', size, (0, 0, 0, 1), True)
        for m in dmats:
            _emission_from(m, image_color=True)
        _bake('EMIT', decals, target, dcol)
        for m in dmats:
            _emission_from(m, image_alpha=True)
        _bake('EMIT', decals, target, dalpha)
        for m in dmats:
            _clear_emission(m)
        a = _pixels(dalpha)[..., :1]
        albedo = albedo * (1 - a) + _pixels(dcol)[..., :3] * a

    metal_img = _image(f'{name}_metal', size, (0, 0, 0, 1), True)
    for m in mats:
        _emission_from(m, value=float(m.get('gse_metallic', 0.0)))
    _bake('EMIT', parts, target, metal_img)
    rough_img = _image(f'{name}_rough', size, (0.5, 0.5, 0.5, 1), True)
    for m in mats:
        _emission_from(m, value=float(m.get('gse_roughness', 0.5)))
    _bake('EMIT', parts, target, rough_img)
    for m in mats:
        _clear_emission(m)

    ao_img = _image(f'{name}_ao', size, (1, 1, 1, 1), True)
    for o in parts + decals:
        o.hide_render = True
    scene.world.light_settings.distance = 0.35
    _bake('AO', [], target, ao_img, selected_to_active=False, samples=min(samples, 48))
    for o in parts + decals:
        o.hide_render = False
    ao = _pixels(ao_img)[..., :1]
    albedo = albedo * (1 - ao_strength + ao_strength * ao)

    metal = _pixels(metal_img)[..., 0]
    gloss = 1.0 - _pixels(rough_img)[..., 0]
    mat = np.zeros(albedo.shape, dtype=np.float32)
    mat[..., 0] = metal
    mat[..., 1] = gloss
    normal = np.zeros(albedo.shape, dtype=np.float32)
    normal[..., 0] = 0.5
    normal[..., 1] = 0.5
    normal[..., 2] = 1.0

    paths = {
        'albedo': os.path.join(out_dir, f'{name}_ALB.png'),
        'normal': os.path.join(out_dir, f'{name}_NML.png'),
        'material': os.path.join(out_dir, f'{name}_MAT.png'),
    }
    _save(albedo, paths['albedo'], False)
    _save(normal, paths['normal'], True)
    _save(mat, paths['material'], True)
    return target, parts, paths, albedo
