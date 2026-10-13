import math
import os

import bmesh
import bpy
from mathutils import Matrix, Vector

MM = 0.001


class Frame:
    def __init__(self, x_origin_mm):
        self.x0 = x_origin_mm

    def v(self, x, y, z):
        return Vector((y * MM, (x - self.x0) * MM, z * MM))


def reset_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    for c in list(bpy.data.collections):
        bpy.data.collections.remove(c)
    scene = bpy.context.scene
    scene.unit_settings.system = 'METRIC'
    scene.unit_settings.scale_length = 1.0
    return scene


def collection(name):
    c = bpy.data.collections.get(name) or bpy.data.collections.new(name)
    if c.name not in bpy.context.scene.collection.children:
        bpy.context.scene.collection.children.link(c)
    return c


_materials = {}


def mat(name, color, metallic=0.0, roughness=0.5, emission=None, emission_strength=0.0, alpha=None):
    if name in _materials:
        return _materials[name]
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    p = m.node_tree.nodes.get('Principled BSDF')
    p.inputs['Base Color'].default_value = (*srgb_to_linear(color), 1.0)
    p.inputs['Metallic'].default_value = metallic
    p.inputs['Roughness'].default_value = roughness
    if emission is not None:
        p.inputs['Emission Color'].default_value = (*srgb_to_linear(emission), 1.0)
        p.inputs['Emission Strength'].default_value = emission_strength
    if alpha is not None:
        p.inputs['Alpha'].default_value = alpha
    m['gse_color'] = list(color)
    m['gse_metallic'] = metallic
    m['gse_roughness'] = roughness
    _materials[name] = m
    return m


def image_mat(name, image_path, metallic=0.0, roughness=0.5, alpha_blend=True):
    if name in _materials:
        return _materials[name]
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    p = nt.nodes.get('Principled BSDF')
    tex = nt.nodes.new('ShaderNodeTexImage')
    tex.image = bpy.data.images.load(image_path, check_existing=True)
    tex.interpolation = 'Cubic'
    nt.links.new(tex.outputs['Color'], p.inputs['Base Color'])
    if alpha_blend:
        nt.links.new(tex.outputs['Alpha'], p.inputs['Alpha'])
        try:
            m.surface_render_method = 'DITHERED'
        except (AttributeError, TypeError):
            pass
    p.inputs['Metallic'].default_value = metallic
    p.inputs['Roughness'].default_value = roughness
    m['gse_decal'] = image_path
    _materials[name] = m
    return m


def srgb_to_linear(c):
    out = []
    for v in c:
        v = v / 255.0 if v > 1.0 else v
        out.append(v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4)
    return out


def _object(name, bm, material, coll, smooth=False):
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    obj = bpy.data.objects.new(name, me)
    (coll or bpy.context.scene.collection).objects.link(obj)
    if material is not None:
        if isinstance(material, (list, tuple)):
            for m in material:
                me.materials.append(m)
        else:
            me.materials.append(material)
    for poly in me.polygons:
        poly.use_smooth = smooth
    return obj


def _bevel_all(bm, width, segments=2, edges=None):
    if width <= 0:
        return
    edges = edges if edges is not None else [e for e in bm.edges]
    bmesh.ops.bevel(bm, geom=edges, offset=width, offset_type='OFFSET', segments=segments, profile=0.5, affect='EDGES', clamp_overlap=True)


def box(f, name, x0, x1, y0, y1, z0, z1, material, coll=None, bevel=0.0, segments=2):
    bm = bmesh.new()
    a, b = f.v(x0, y0, z0), f.v(x1, y1, z1)
    lo = Vector((min(a.x, b.x), min(a.y, b.y), min(a.z, b.z)))
    hi = Vector((max(a.x, b.x), max(a.y, b.y), max(a.z, b.z)))
    bmesh.ops.create_cube(bm, size=1.0)
    for vert in bm.verts:
        vert.co = Vector((lo.x if vert.co.x < 0 else hi.x, lo.y if vert.co.y < 0 else hi.y, lo.z if vert.co.z < 0 else hi.z))
    _bevel_all(bm, bevel * MM, segments)
    return _object(name, bm, material, coll)


def prism_xz(f, name, profile_xz, y0, y1, material, coll=None, bevel=0.0, segments=2, smooth=False):
    bm = bmesh.new()
    front = [bm.verts.new(f.v(x, y0, z)) for x, z in profile_xz]
    back = [bm.verts.new(f.v(x, y1, z)) for x, z in profile_xz]
    n = len(profile_xz)
    bm.faces.new(front)
    bm.faces.new(list(reversed(back)))
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new([front[i], front[j], back[j], back[i]])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    _bevel_all(bm, bevel * MM, segments)
    return _object(name, bm, material, coll, smooth)


def prism_yz(f, name, profile_yz, x0, x1, material, coll=None, bevel=0.0, segments=2, smooth=False):
    bm = bmesh.new()
    a = [bm.verts.new(f.v(x0, y, z)) for y, z in profile_yz]
    b = [bm.verts.new(f.v(x1, y, z)) for y, z in profile_yz]
    n = len(profile_yz)
    bm.faces.new(a)
    bm.faces.new(list(reversed(b)))
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new([a[i], a[j], b[j], b[i]])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    _bevel_all(bm, bevel * MM, segments)
    return _object(name, bm, material, coll, smooth)


def cylinder(f, name, center, axis, radius, length, material, coll=None, segments=32, bevel=0.0, cap=True, smooth=True):
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=cap, segments=segments, radius1=radius * MM, radius2=radius * MM, depth=length * MM)
    _bevel_all(bm, bevel * MM, 2, [e for e in bm.edges if len(e.link_faces) == 2 and abs(e.link_faces[0].normal.dot(e.link_faces[1].normal)) < 0.5])
    _orient(bm, f.v(*center), axis)
    obj = _object(name, bm, material, coll, smooth)
    return obj


def lathe(f, name, center, axis, profile_rz, material, coll=None, segments=48, smooth=True):
    bm = bmesh.new()
    rings = []
    for i in range(segments):
        a = 2 * math.pi * i / segments
        rings.append([bm.verts.new(Vector((r * MM * math.cos(a), r * MM * math.sin(a), z * MM))) for r, z in profile_rz])
    for i in range(segments):
        j = (i + 1) % segments
        for k in range(len(profile_rz) - 1):
            bm.faces.new([rings[i][k], rings[j][k], rings[j][k + 1], rings[i][k + 1]])
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-6)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    _orient(bm, f.v(*center), axis)
    return _object(name, bm, material, coll, smooth)


def _orient(bm, origin, axis):
    axis_v = {'x': Vector((0, 1, 0)), 'y': Vector((1, 0, 0)), 'z': Vector((0, 0, 1))}[axis] if isinstance(axis, str) else Vector(axis).normalized()
    rot = Vector((0, 0, 1)).rotation_difference(axis_v).to_matrix().to_4x4()
    bmesh.ops.transform(bm, matrix=Matrix.Translation(origin) @ rot, verts=bm.verts)


def tube(f, name, points_xyz, radius, material, coll=None, segments=16, smooth=True, cap=True):
    pts = [f.v(*p) for p in points_xyz]
    bm = bmesh.new()
    rings = []
    for i, p in enumerate(pts):
        if i == 0:
            t = (pts[1] - pts[0]).normalized()
        elif i == len(pts) - 1:
            t = (pts[-1] - pts[-2]).normalized()
        else:
            t = ((pts[i + 1] - pts[i]).normalized() + (pts[i] - pts[i - 1]).normalized()).normalized()
        ref = Vector((0, 0, 1)) if abs(t.z) < 0.9 else Vector((1, 0, 0))
        n1 = t.cross(ref).normalized()
        n2 = t.cross(n1).normalized()
        ring = []
        for k in range(segments):
            a = 2 * math.pi * k / segments
            ring.append(bm.verts.new(p + (n1 * math.cos(a) + n2 * math.sin(a)) * radius * MM))
        rings.append(ring)
    for i in range(len(rings) - 1):
        for k in range(segments):
            j = (k + 1) % segments
            bm.faces.new([rings[i][k], rings[i][j], rings[i + 1][j], rings[i + 1][k]])
    if cap:
        bm.faces.new(list(reversed(rings[0])))
        bm.faces.new(rings[-1])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return _object(name, bm, material, coll, smooth)


def smooth_path(points, samples_per_segment=8):
    pts = [Vector(p) for p in points]
    if len(pts) < 3:
        return [tuple(p) for p in pts]
    out = []
    ext = [pts[0] + (pts[0] - pts[1])] + pts + [pts[-1] + (pts[-1] - pts[-2])]
    for i in range(1, len(ext) - 2):
        p0, p1, p2, p3 = ext[i - 1], ext[i], ext[i + 1], ext[i + 2]
        for s in range(samples_per_segment):
            t = s / samples_per_segment
            t2, t3 = t * t, t * t * t
            out.append(tuple(0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2 + (-p0 + 3 * p1 - 3 * p2 + p3) * t3)))
    out.append(tuple(pts[-1]))
    return out


def decal(f, name, corners_xyz, material, coll=None):
    bm = bmesh.new()
    vs = [bm.verts.new(f.v(*c)) for c in corners_xyz]
    face = bm.faces.new(vs)
    uv = bm.loops.layers.uv.new('UVMap')
    for loop, (u, v) in zip(face.loops, [(0, 0), (1, 0), (1, 1), (0, 1)]):
        loop[uv].uv = (u, v)
    return _object(name, bm, material, coll)


def join(objs, name):
    bpy.ops.object.select_all(action='DESELECT')
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    bpy.ops.object.join()
    objs[0].name = name
    return objs[0]


def setup_render(scene, width, height, samples=48):
    scene.render.engine = 'CYCLES'
    scene.cycles.samples = samples
    scene.cycles.use_denoising = True
    if os.environ.get('GSE_CPU'):
        scene.cycles.device = 'CPU'
        scene.render.threads_mode = 'FIXED'
        scene.render.threads = int(os.environ.get('GSE_THREADS', '2'))
    try:
        prefs = bpy.context.preferences.addons['cycles'].preferences
        for backend in (() if os.environ.get('GSE_CPU') else ('OPTIX', 'CUDA')):
            try:
                prefs.compute_device_type = backend
                prefs.get_devices()
                if any(d.type == backend for d in prefs.devices):
                    for d in prefs.devices:
                        d.use = d.type == backend
                    scene.cycles.device = 'GPU'
                    break
            except TypeError:
                continue
    except Exception:
        pass
    scene.render.resolution_x = width
    scene.render.resolution_y = height
    scene.render.film_transparent = False
    scene.view_settings.view_transform = 'AgX'
    world = bpy.data.worlds.new('World')
    scene.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes.get('Background')
    bg.inputs['Color'].default_value = (0.75, 0.8, 0.88, 1.0)
    bg.inputs['Strength'].default_value = 0.9
    sun_data = bpy.data.lights.new('Sun', 'SUN')
    sun_data.energy = 4.0
    sun_data.angle = math.radians(3)
    sun = bpy.data.objects.new('Sun', sun_data)
    sun.rotation_euler = (math.radians(50), math.radians(10), math.radians(-35))
    scene.collection.objects.link(sun)
    ground = bpy.data.meshes.new('Ground')
    bm = bmesh.new()
    bmesh.ops.create_grid(bm, x_segments=1, y_segments=1, size=30)
    bm.to_mesh(ground)
    bm.free()
    g = bpy.data.objects.new('Ground', ground)
    g.data.materials.append(mat('ground', (120, 120, 118), roughness=0.9))
    scene.collection.objects.link(g)


def camera(scene, name, location, target, lens=35.0, ortho_scale=None):
    data = bpy.data.cameras.new(name)
    if ortho_scale is not None:
        data.type = 'ORTHO'
        data.ortho_scale = ortho_scale
    else:
        data.lens = lens
    cam = bpy.data.objects.new(name, data)
    cam.location = Vector(location)
    d = Vector(target) - Vector(location)
    cam.rotation_euler = d.to_track_quat('-Z', 'Y').to_euler()
    scene.collection.objects.link(cam)
    return cam


def render(scene, cam, path):
    scene.camera = cam
    scene.render.filepath = path
    bpy.ops.render.render(write_still=True)


def bevel_where(obj, predicate, width, segments=4):
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    edges = [e for e in bm.edges if predicate((e.verts[0].co + e.verts[1].co) / 2 / MM, (e.verts[1].co - e.verts[0].co).normalized())]
    if edges:
        bmesh.ops.bevel(bm, geom=edges, offset=width * MM, offset_type='OFFSET', segments=segments, profile=0.5, affect='EDGES', clamp_overlap=True)
    bm.to_mesh(obj.data)
    bm.free()
    return len(edges)


def set_anim(obj, steps):
    import json
    obj['gse_anim'] = json.dumps(steps)
    return obj
