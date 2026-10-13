import importlib
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import bpy

import gselib as g

VIEWS = {
    'gpu_tld409': {
        'drawing_side': dict(res=(1300, 1140), ortho=1300 * 2.566 / 1000, loc=(10.0, (1432.0 - 1338.0) / 1000, 0.765), target=(0.0, (1432.0 - 1338.0) / 1000, 0.765)),
        'drawing_end': dict(res=(1080, 1140), ortho=1080 * 2.552 / 1000, loc=(0.0447, -10.0, 0.760), target=(0.0447, 0.0, 0.760)),
        'three_quarter_tow': dict(res=(700, 514), lens=35, loc=(-1.25, 3.35, 1.58), target=(0.12, 0.3, 0.8)),
        'three_quarter_ctrl': dict(res=(700, 549), lens=35, loc=(-1.94, -3.05, 1.45), target=(0.05, -0.25, 0.85)),
        'side_right': dict(res=(700, 548), lens=35, loc=(3.55, 0.25, 1.0), target=(0.0, 0.25, 0.8)),
        'connection': dict(res=(1200, 900), lens=24, loc=(-2.5, -2.0, 2.2), target=(2.2, -1.0, 1.2)),
    },
}


def bake_and_export(name, model, out):
    import bake as bk
    import export_obj8 as ex
    coll = bpy.data.collections[name]
    target, parts, paths, _ = bk.bake(coll, out, name, size=getattr(model, 'TEXTURE_SIZE', 2048))
    variants = {}
    for op, albedo in getattr(model, 'liveries', lambda out: {})(out).items():
        variants[op] = albedo
    written, nv, nt = ex.export(target, parts, out, name, paths, variants)
    print(f'GSE export {name}: {nv} vertices, {nt} triangles, files {written}')


def main():
    argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
    name, out = argv[0], argv[1]
    model_views = getattr(importlib.import_module(name), 'VIEWS', None) or VIEWS[name]
    views = [v for v in (argv[2].split(',') if len(argv) > 2 else list(model_views)) if v != 'bake']
    do_bake = len(argv) > 2 and 'bake' in argv[2].split(',')
    os.makedirs(out, exist_ok=True)
    model = importlib.import_module(name)
    model.build(with_connection=not any(v.startswith('drawing') for v in views))
    if do_bake:
        bake_and_export(name, model, out)
        return
    scene = bpy.context.scene
    for v in views:
        spec = model_views[v]
        g.setup_render(scene, *spec['res'], samples=spec.get('samples', 48)) if not scene.world else None
        scene.render.resolution_x, scene.render.resolution_y = spec['res']
        cam = g.camera(scene, f'cam_{v}', spec['loc'], spec['target'], lens=spec.get('lens', 35.0), ortho_scale=spec.get('ortho'))
        g.render(scene, cam, os.path.join(out, f'{name}_{v}.png'))
    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(out, f'{name}.blend'))


main()
