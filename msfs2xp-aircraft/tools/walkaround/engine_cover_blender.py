"""Engine inlet covers, built in Blender (headless), one per engine, in the
aircraft's own X-Plane object axes (x right, y up, z aft).

    blender -b --python engine_cover_blender.py -- <inlets.json> <texture.png> <out.obj> [<preview.png>]

inlets.json comes from measure_inlets.py. Each cover: a domed cap across the
lip highlight, a roll over the lip and a skirt following the cowl's measured
profile with fabric clearance, a drawstring hem and fabric wrinkles.
build_engine_covers.py turns the export into the OBJ8 object.
"""
import json
import math
import sys

import bpy
import mathutils

SEG = 88
DOME = 0.035
CLEAR = 0.02
CORNER = 0.08
SKIRT = 0.85


def envelope(profile):
    out, m = [], 0.0
    for dz, r in profile:
        if dz > SKIRT:
            break
        m = max(m, r)
        out.append((dz, m))
    return out


def frame(info):
    c = mathutils.Vector(info["centre"])
    aft = -mathutils.Vector(info["normal"]).normalized()
    x = mathutils.Vector((1, 0, 0))
    x = (x - aft * x.dot(aft)).normalized()
    y = aft.cross(x).normalized()
    return c, x, y, aft


def folds(t, s):
    a = 0.003 + 0.022 * s * s
    w = 0.5 + 0.3 * math.sin(14 * t + 0.7 * math.sin(3 * t)) + 0.2 * math.sin(23 * t + 1.3)
    return a * w


def cover(name, info):
    c, ex, ey, ez = frame(info)
    prof = envelope(info["profile"])
    r_face = max(r for dz, r in prof if dz <= 0.12) + CLEAR
    z_face = -0.03
    rings = []
    for k in range(13):
        rho = k / 12
        rings.append(("face", (r_face - CORNER) * rho, z_face - DOME * (1 - rho * rho), 0.0))
    for k in range(1, 6):
        a = k / 5 * math.pi / 2
        rings.append(("edge", r_face - CORNER + CORNER * math.sin(a), z_face + CORNER * (1 - math.cos(a)), 0.0))
    z0 = z_face + CORNER
    for dz, r in prof:
        if dz <= z0 + 0.02:
            continue
        s = (dz - z0) / (SKIRT - z0)
        rings.append(("skirt", max(r + CLEAR, r_face if dz < 0.2 else 0), dz, s))
    last_r = rings[-1][1]
    rings.append(("skirt", last_r - 0.006, SKIRT + 0.015, 1.0))
    arc = [0.0]
    for (_, r0, z0_, _), (_, r1, z1, _) in zip(rings, rings[1:]):
        arc.append(arc[-1] + math.hypot(r1 - r0, z1 - z0_))
    edge_start = arc[12]
    verts, uvs, faces, rows = [], [], [], []
    for i, (kind, r, z, s) in enumerate(rings):
        row = []
        n = 1 if r == 0 else SEG + 1
        for j in range(n):
            t = 2 * math.pi * j / SEG
            rr = r + (folds(t, s) if kind == "skirt" else 0.0)
            p = c + ex * (rr * math.cos(t)) + ey * (rr * math.sin(t)) + ez * z
            if kind == "face":
                f = r / r_face
                uv = (0.25 - 0.246 * f * math.cos(t), 0.75 + 0.246 * f * math.sin(t))
            else:
                uv = (0.5 + 0.5 * j / SEG, 0.995 - 0.99 * (arc[i] - edge_start) / (arc[-1] - edge_start))
            row.append(len(verts))
            verts.append(p)
            uvs.append(uv)
        rows.append((kind, row))
    tags = []
    for (ka, a), (kb, b) in zip(rows, rows[1:]):
        for j in range(SEG):
            if len(a) == 1:
                faces.append([a[0], b[j + 1], b[j]])
                tags.append(None)
            else:
                faces.append([a[j], a[j + 1], b[j + 1], b[j]])
                tags.append("seam" if ka == "face" and kb != "face" else None)
    me = bpy.data.meshes.new(name)
    me.from_pydata([tuple(v) for v in verts], [], faces)
    me.update()
    uv = me.uv_layers.new(name="UV")
    face_rim = rows[12][1]
    rim = {vi: j for j, vi in enumerate(face_rim)}
    for poly, tag in zip(me.polygons, tags):
        for li in poly.loop_indices:
            vi = me.loops[li].vertex_index
            if tag == "seam" and vi in rim:
                uv.data[li].uv = (0.5 + 0.5 * rim[vi] / SEG, 0.995)
            else:
                uv.data[li].uv = uvs[vi]
    me.shade_smooth()
    ob = bpy.data.objects.new(name, me)
    bpy.context.collection.objects.link(ob)
    tex = bpy.data.textures.get("wrinkles") or bpy.data.textures.new("wrinkles", type="CLOUDS")
    tex.noise_scale = 0.45
    tex.noise_depth = 2
    mod = ob.modifiers.new("Wrinkles", "DISPLACE")
    mod.texture = tex
    mod.texture_coords = "GLOBAL"
    mod.strength = 0.012
    mod.direction = "NORMAL"
    return ob, None


def streamer(name, top):
    verts, uvs, faces = [], [], []
    n = 14
    for i in range(n + 1):
        s = i / n
        sway = 0.04 * math.sin(s * 2.5 * math.pi) * s
        for side in (-1, 1):
            verts.append((top.x + side * STREAMER_W / 2, top.y - STREAMER_L * s, top.z - 0.03 * s + sway))
            uvs.append((0.02 + 0.96 * s, 0.25 - side * 0.22))
    for i in range(n):
        a = 2 * i
        faces.append([a, a + 1, a + 3, a + 2])
    me = bpy.data.meshes.new(name)
    me.from_pydata(verts, [], faces)
    me.update()
    uv = me.uv_layers.new(name="UV")
    for poly in me.polygons:
        for li in poly.loop_indices:
            uv.data[li].uv = uvs[me.loops[li].vertex_index]
    ob = bpy.data.objects.new(name, me)
    bpy.context.collection.objects.link(ob)
    return ob


def material(tex_path):
    mat = bpy.data.materials.new("EngineCoverMat")
    mat.use_nodes = True
    nt = mat.node_tree
    bsdf = nt.nodes.get("Principled BSDF")
    img = nt.nodes.new("ShaderNodeTexImage")
    img.image = bpy.data.images.load(tex_path)
    nt.links.new(img.outputs["Color"], bsdf.inputs["Base Color"])
    bsdf.inputs["Roughness"].default_value = 0.85
    return mat


def preview(out_png, target):
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.display.shading.light = "STUDIO"
    scene.display.shading.color_type = "TEXTURE"
    scene.render.resolution_x, scene.render.resolution_y = 1200, 900
    scene.world = bpy.data.worlds.new("w")
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    scene.collection.objects.link(cam)
    cam.location = target + mathutils.Vector((4.2, 0.8, -6.8))
    back = (cam.location - target).normalized()
    right = mathutils.Vector((0, 1, 0)).cross(back).normalized()
    up = back.cross(right)
    cam.matrix_world = mathutils.Matrix((right, up, back)).transposed().to_4x4()
    cam.location = target + mathutils.Vector((4.2, 0.8, -6.8))
    cam.data.lens = 35
    scene.camera = cam
    scene.render.filepath = out_png
    bpy.ops.render.render(write_still=True)


def main():
    argv = sys.argv[sys.argv.index("--") + 1:]
    inlets = json.load(open(argv[0]))
    tex_path, out_obj = argv[1], argv[2]
    bpy.ops.wm.read_factory_settings(use_empty=True)
    mat = material(tex_path)
    objs = []
    for name, info in sorted(inlets.items()):
        ob, _ = cover(name, info)
        objs.append(ob)
    for ob in objs:
        ob.data.materials.append(mat)
        ob.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    bpy.ops.wm.obj_export(filepath=out_obj, export_selected_objects=True, export_uv=True, export_normals=True, export_materials=False,
                          export_triangulated_mesh=True, apply_modifiers=True, forward_axis="Y", up_axis="Z")
    if len(argv) > 3:
        preview(argv[3], mathutils.Vector(inlets["eng_inlet_cover_1"]["centre"]))


main()
