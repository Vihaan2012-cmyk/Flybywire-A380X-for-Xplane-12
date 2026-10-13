import math
import os

import gselib as g

NAME = 'udcat_highloader'
OUT_LIVERIES = 'E:/fbw-gse/out/udcat_highloader/liveries'

REAR_AXLES = (1700.0, 2900.0)
FRONT_AXLE = 7600.0
TRACK = 2000.0
TYRE_D = 1050.0
TYRE_W = 315.0
BOX_X0 = 700.0
BOX_X1 = 5700.0
BOX_HY = 1225.0
BOX_Z0 = 1500.0
BOX_Z1 = 3900.0
LIFT_TRAVEL = 6.5
BRIDGE_X1 = 6600.0
BRIDGE_TRAVEL = 0.9
CAB_X0 = 6900.0
CAB_X1 = 8750.0

LIFT_ANIM = [{'kind': 'trans', 'offset0': (0.0, 0.0, 0.0), 'offset1': (0.0, 0.0, LIFT_TRAVEL), 'v0': 0.0, 'v1': 1.0, 'dataref': 'fbw/gse/udcat/lift'}]
BRIDGE_ANIM = [{'kind': 'trans', 'offset0': (0.0, 0.0, 0.0), 'offset1': (0.0, BRIDGE_TRAVEL, 0.0), 'v0': 0.0, 'v1': 1.0, 'dataref': 'fbw/gse/udcat/bridge'}]

OPERATORS = ('sats', 'lsg', 'gategroup', 'dnata', 'newrest', 'doco')


def materials():
    return {
        'white': g.mat('gse_white', (236, 237, 233), roughness=0.32),
        'body': g.mat('gse_catering_body', (226, 228, 226), roughness=0.35),
        'galv': g.mat('gse_galvanized', (168, 171, 172), metallic=0.85, roughness=0.42),
        'black': g.mat('gse_black', (22, 22, 23), roughness=0.55),
        'rubber': g.mat('gse_rubber', (28, 28, 28), roughness=0.88),
        'rim': g.mat('gse_rim', (150, 150, 148), metallic=0.8, roughness=0.4),
        'chassis': g.mat('gse_chassis_grey', (70, 72, 76), metallic=0.4, roughness=0.5),
        'glass': g.mat('gse_cab_glass', (30, 40, 50), roughness=0.06),
        'amber': g.mat('gse_amber_lens', (255, 140, 10), roughness=0.12),
        'red': g.mat('gse_red', (185, 18, 20), roughness=0.3),
        'yellow': g.mat('gse_yellow', (245, 200, 0), roughness=0.35),
        'cab': g.mat('gse_cab_white', (240, 240, 238), roughness=0.3),
        'rail': g.mat('gse_rail_yellow', (240, 190, 0), roughness=0.4),
        'floor': g.mat('gse_floor_checker', (95, 98, 100), metallic=0.6, roughness=0.5),
    }


def chassis(f, m, c):
    parts = []
    for k in (-1, 1):
        parts.append(g.box(f, f'frame_rail_{k}', 0, 8300, k * 520 - 90, k * 520 + 90, 780, 1000, m['chassis'], c, bevel=8))
    for x in (400, 1100, 2300, 3500, 4700, 5900):
        parts.append(g.box(f, f'cross_member_{x}', x - 60, x + 60, -520, 520, 800, 960, m['chassis'], c, bevel=6))
    parts.append(g.box(f, 'subframe', BOX_X0 - 100, BOX_X1 + 100, -1000, 1000, 1000, 1180, m['galv'], c, bevel=10))
    parts.append(g.box(f, 'rear_bumper', -60, 20, -1150, 1150, 800, 1050, m['chassis'], c, bevel=8))
    for k in (-1, 1):
        parts.append(g.box(f, f'rear_light_{k}', -70, -55, k * 900 - 60, k * 900 + 60, 900, 1020, m['red'], c, bevel=3))
        parts.append(g.box(f, f'mudguard_{k}', 1100, 3500, k * 1180 - 20, k * 1180 + 20, 1240, 1300, m['chassis'], c, bevel=6))
        parts.append(g.box(f, f'fuel_tank_{k}', 3700, 5100, k * 1030 - 130, k * 1030 + 130, 600, 1000, m['galv'], c, bevel=20))
        parts.append(g.box(f, f'side_marker_{k}', 3000, 3150, k * 1262, k * 1268, 1050, 1110, m['amber'], c))
    return parts


def wheels(f, m, c):
    parts = []
    r = TYRE_D / 2
    rr = 280.0
    tyre = [(rr, -TYRE_W / 2 + 25), (rr + 12, -TYRE_W / 2 + 6), (r - 60, -TYRE_W / 2), (r - 15, -TYRE_W / 2 + 15), (r, -TYRE_W / 2 + 60),
            (r, TYRE_W / 2 - 60), (r - 15, TYRE_W / 2 - 15), (r - 60, TYRE_W / 2), (rr + 12, TYRE_W / 2 - 6), (rr, TYRE_W / 2 - 25)]
    rim = [(0.0, 70.0), (100.0, 70.0), (120.0, 55.0), (220.0, 45.0), (rr - 2, 40.0), (rr + 8, 100.0), (rr + 8, 120.0)]
    for i, x in enumerate(REAR_AXLES + (FRONT_AXLE,)):
        label = 'front' if x == FRONT_AXLE else f'rear{i}'
        for k in (-1, 1):
            y = k * TRACK / 2
            tag = f'{label}_{"L" if k < 0 else "R"}'
            parts.append(g.lathe(f, f'tyre_{tag}', (x, y, r), (k, 0, 0), tyre, m['rubber'], c, segments=40))
            parts.append(g.lathe(f, f'rim_{tag}', (x, y, r), (k, 0, 0), [(rad, -z) for rad, z in rim], m['rim'], c, segments=32))
            parts.append(g.cylinder(f, f'hub_{tag}', (x, y + k * 80, r), 'y', 110, 40, m['galv'], c, segments=20, bevel=6))
            for b in range(8):
                a = 2 * math.pi * b / 8
                parts.append(g.cylinder(f, f'lug_{tag}_{b}', (x + 150 * math.cos(a), y + k * 82, r + 150 * math.sin(a)), 'y', 14, 24, m['galv'], c, segments=8))
        parts.append(g.cylinder(f, f'axle_{label}', (x, 0, r), 'y', 70, TRACK - 200, m['chassis'], c, segments=16))
    return parts


def cab(f, m, c):
    parts = []
    parts.append(g.box(f, 'cab_body', CAB_X0, CAB_X1, -1200, 1200, 1000, 3000, m['cab'], c, bevel=60, segments=3))
    parts.append(g.box(f, 'cab_bumper', CAB_X1 - 20, CAB_X1 + 140, -1200, 1200, 800, 1050, m['chassis'], c, bevel=20))
    parts.append(g.box(f, 'cab_grille', CAB_X1 + 1, CAB_X1 + 12, -700, 700, 1300, 1900, m['black'], c, bevel=3))
    parts.append(g.box(f, 'windscreen', CAB_X1 - 6, CAB_X1 + 4, -1050, 1050, 2050, 2800, m['glass'], c, bevel=10))
    for k in (-1, 1):
        parts.append(g.box(f, f'door_window_{k}', CAB_X1 - 900, CAB_X1 - 150, k * 1200 - 4 * k, k * 1200 + 6 * k, 2050, 2780, m['glass'], c, bevel=8))
        parts.append(g.box(f, f'door_handle_{k}', CAB_X1 - 1200, CAB_X1 - 1000, k * 1200, k * 1200 + 20 * k, 1850, 1900, m['black'], c, bevel=3))
        parts.append(g.box(f, f'step_{k}', CAB_X0 + 800, CAB_X0 + 1500, k * 1200, k * 1200 + 140 * k, 760, 800, m['galv'], c, bevel=3))
        parts.append(g.cylinder(f, f'headlight_{k}', (CAB_X1 + 6, k * 900, 1500), 'x', 90, 20, m['yellow'], c, segments=24, bevel=4))
        parts.append(g.box(f, f'mirror_arm_{k}', CAB_X1 - 700, CAB_X1 - 650, k * 1200, k * 1450, 2500, 2540, m['black'], c, bevel=3))
        parts.append(g.box(f, f'mirror_{k}', CAB_X1 - 740, CAB_X1 - 620, k * 1450, k * 1480, 2300, 2800, m['black'], c, bevel=8))
    parts.append(g.box(f, 'roof_light_bar', CAB_X1 - 500, CAB_X1 - 300, -500, 500, 3000, 3060, m['amber'], c, bevel=8))
    parts.append(g.cylinder(f, 'beacon_base', (CAB_X0 + 600, 0, 3040), 'z', 100, 40, m['black'], c, segments=20, bevel=3))
    parts.append(g.lathe(f, 'beacon_lens', (CAB_X0 + 600, 0, 3060), 'z', [(0.0, 90.0), (50.0, 85.0), (90.0, 40.0), (90.0, 0.0)], m['amber'], c, segments=24))
    return parts


def stabilisers(f, m, c):
    parts = []
    for x in (BOX_X0 + 300, BOX_X1 - 300):
        for k in (-1, 1):
            tag = f'{int(x)}_{k}'
            parts.append(g.box(f, f'stab_arm_{tag}', x - 90, x + 90, k * 1000, k * 1450, 880, 1000, m['chassis'], c, bevel=6))
            parts.append(g.cylinder(f, f'stab_leg_{tag}', (x, k * 1400, 480), 'z', 70, 800, m['galv'], c, segments=16))
            parts.append(g.cylinder(f, f'stab_pad_{tag}', (x, k * 1400, 40), 'z', 190, 80, m['black'], c, segments=20, bevel=6))
    return parts


SCISSOR_DEG1 = 64.0


def scissor_step(f, pivot_xz, sign):
    v = f.v(0.0, 0.0, 0.0)
    p = f.v(pivot_xz[0], 0.0, pivot_xz[1])
    return {'kind': 'rotate', 'pivot': (0.0, p.y, p.z), 'axis': (1.0, 0.0, 0.0), 'deg0': 0.0, 'deg1': sign * SCISSOR_DEG1, 'v0': 0.0, 'v1': 1.0, 'dataref': 'fbw/gse/udcat/lift'}


def scissor_frame(f, m, c):
    parts = []
    for k in (-1, 1):
        for x0, x1 in ((BOX_X0 + 400, BOX_X0 + 2300), (BOX_X0 + 2700, BOX_X1 - 400)):
            arm_a = g.tube(f, f'scissor_a_{k}_{int(x0)}', [(x0, k * 800, 1230), (x1, k * 800, 1430)], 45, m['yellow'], c, segments=10)
            arm_b = g.tube(f, f'scissor_b_{k}_{int(x0)}', [(x0, k * 880, 1430), (x1, k * 880, 1230)], 45, m['yellow'], c, segments=10)
            g.set_anim(arm_a, [scissor_step(f, (x0, 1230.0), 1.0)])
            g.set_anim(arm_b, [scissor_step(f, (x1, 1230.0), -1.0)])
            parts += [arm_a, arm_b]
            roller = g.cylinder(f, f'scissor_roller_{k}_{int(x0)}', (x1, k * 840, 1430), 'y', 45, 150, m['galv'], c, segments=12)
            g.set_anim(roller, [{'kind': 'trans', 'offset0': (0.0, 0.0, 0.0), 'offset1': (0.0, -(x1 - x0) * (1 - math.cos(math.radians(SCISSOR_DEG1))) / 1000.0, 0.0), 'v0': 0.0, 'v1': 1.0, 'dataref': 'fbw/gse/udcat/lift'}])
            parts.append(roller)
            parts.append(g.cylinder(f, f'scissor_pivot_{k}_{int(x0)}', ((x0 + x1) / 2, k * 840, 1330), 'y', 55, 150, m['galv'], c, segments=12))
        parts.append(g.cylinder(f, f'lift_ram_{k}', (BOX_X0 + 2500, k * 700, 1320), 'x', 60, 1200, m['galv'], c, segments=14))
    return parts


def lift_box(f, m, c):
    parts = []
    parts.append(g.box(f, 'box_floor', BOX_X0, BOX_X1, -BOX_HY, BOX_HY, BOX_Z0 - 120, BOX_Z0, m['galv'], c, bevel=6))
    parts.append(g.box(f, 'box_body', BOX_X0, BOX_X1, -BOX_HY, BOX_HY, BOX_Z0, BOX_Z1, m['body'], c, bevel=35, segments=3))
    parts.append(g.box(f, 'box_roof_cap', BOX_X0 + 40, BOX_X1 - 40, -BOX_HY + 60, BOX_HY - 60, BOX_Z1, BOX_Z1 + 40, m['white'], c, bevel=15))
    parts.append(g.box(f, 'box_skirt', BOX_X0 - 20, BOX_X1 + 20, -BOX_HY - 15, BOX_HY + 15, BOX_Z0 - 40, BOX_Z0 + 40, m['chassis'], c, bevel=8))
    for k in (-1, 1):
        for i in range(7):
            x = BOX_X0 + 500 + i * 650
            parts.append(g.box(f, f'panel_rib_{k}_{i}', x - 15, x + 15, k * (BOX_HY + 1), k * (BOX_HY + 8), BOX_Z0 + 40, BOX_Z1 - 40, m['white'], c, bevel=3))
        parts.append(g.box(f, f'box_side_stripe_{k}', BOX_X0 + 20, BOX_X1 - 20, k * (BOX_HY + 1), k * (BOX_HY + 5), BOX_Z0 + 180, BOX_Z0 + 280, m['red'], c))
        parts.append(g.box(f, f'box_marker_{k}', BOX_X0 + 20, BOX_X0 + 120, k * (BOX_HY + 1), k * (BOX_HY + 8), BOX_Z1 - 200, BOX_Z1 - 100, m['amber'], c, bevel=2))
    parts.append(g.box(f, 'rear_door', BOX_X0 - 12, BOX_X0, -1000, 1000, BOX_Z0 + 120, BOX_Z1 - 120, m['white'], c, bevel=8))
    for k in (-1, 1):
        parts.append(g.box(f, f'rear_door_latch_{k}', BOX_X0 - 30, BOX_X0 - 10, k * 940 - 30, k * 940 + 30, BOX_Z0 + 900, BOX_Z0 + 1500, m['black'], c, bevel=4))
        parts.append(g.box(f, f'rear_light_box_{k}', BOX_X0 - 20, BOX_X0, k * 1100 - 40, k * 1100 + 40, BOX_Z0 + 150, BOX_Z0 + 350, m['red'], c, bevel=3))
        parts.append(g.box(f, f'front_guard_post_{k}', BOX_X1 - 30, BOX_X1 + 10, k * 1000 - 20, k * 1000 + 20, BOX_Z0, BOX_Z0 + 1050, m['rail'], c, bevel=3))
    for z in (BOX_Z0 + 400, BOX_Z0 + 700, BOX_Z0 + 1000):
        parts.append(g.box(f, f'rear_step_{int(z)}', BOX_X0 - 160, BOX_X0 - 20, -400, 400, z, z + 30, m['galv'], c, bevel=3))
    parts.append(g.box(f, 'control_cabinet', BOX_X1 - 600, BOX_X1 - 100, BOX_HY, BOX_HY + 120, BOX_Z0 + 100, BOX_Z0 + 800, m['black'], c, bevel=8))
    parts.append(g.box(f, 'estop_plate', BOX_X0 + 200, BOX_X0 + 400, BOX_HY + 5, BOX_HY + 20, BOX_Z0 + 700, BOX_Z0 + 900, m['yellow'], c, bevel=3))
    parts.append(g.cylinder(f, 'estop_button', (BOX_X0 + 300, BOX_HY + 30, BOX_Z0 + 800), 'y', 38, 30, m['red'], c, segments=20, bevel=5))
    parts.append(g.cylinder(f, 'lift_beacon_base', (BOX_X0 + 200, 0, BOX_Z1 + 60), 'z', 70, 40, m['black'], c, segments=18, bevel=3))
    parts.append(g.lathe(f, 'lift_beacon_lens', (BOX_X0 + 200, 0, BOX_Z1 + 80), 'z', [(0.0, 80.0), (45.0, 75.0), (70.0, 30.0), (70.0, 0.0)], m['amber'], c, segments=20))
    parts.append(g.box(f, 'box_front_wall', BOX_X1, BOX_X1 + 40, -BOX_HY, BOX_HY, BOX_Z0, BOX_Z1, m['body'], c, bevel=20))
    parts.append(g.box(f, 'box_front_opening', BOX_X1 + 36, BOX_X1 + 44, -950, 950, BOX_Z0 + 120, BOX_Z0 + 1750, m['black'], c, bevel=4))
    for o in parts:
        g.set_anim(o, LIFT_ANIM)
    return parts


def bridge(f, m, c):
    parts = []
    z = BOX_Z0
    parts.append(g.box(f, 'bridge_deck', BOX_X1 + 40, BRIDGE_X1, -1050, 1050, z - 80, z + 20, m['floor'], c, bevel=5))
    parts.append(g.box(f, 'bridge_bumper', BRIDGE_X1, BRIDGE_X1 + 90, -1050, 1050, z - 120, z + 250, m['rubber'], c, bevel=15))
    parts.append(g.box(f, 'bridge_curtain_frame', BRIDGE_X1 - 20, BRIDGE_X1 + 30, -1050, 1050, z + 250, z + 280, m['galv'], c, bevel=4))
    for k in (-1, 1):
        parts.append(g.box(f, f'bridge_side_{k}', BOX_X1 + 40, BRIDGE_X1, k * 1050 - 15, k * 1050 + 15, z + 20, z + 1050, m['body'], c, bevel=8))
        parts.append(g.tube(f, f'bridge_handrail_top_{k}', [(BOX_X1 + 40, k * 1055, z + 1100), (BRIDGE_X1 + 40, k * 1055, z + 1100)], 22, m['rail'], c, segments=10))
        parts.append(g.tube(f, f'bridge_handrail_mid_{k}', [(BOX_X1 + 40, k * 1055, z + 800), (BRIDGE_X1 + 40, k * 1055, z + 800)], 18, m['rail'], c, segments=10))
        for x in (BOX_X1 + 100, BOX_X1 + 500, BRIDGE_X1):
            parts.append(g.cylinder(f, f'bridge_post_{k}_{int(x)}', (x, k * 1055, z + 550), 'z', 20, 1100, m['rail'], c, segments=10))
        parts.append(g.box(f, f'bridge_fender_{k}', BRIDGE_X1 + 20, BRIDGE_X1 + 90, k * 1000 - 40, k * 1000 + 40, z + 250, z + 1000, m['rubber'], c, bevel=6))
    parts.append(g.tube(f, 'bridge_end_rail', [(BRIDGE_X1 + 60, -1055, z + 1100), (BRIDGE_X1 + 60, 1055, z + 1100)], 18, m['rail'], c, segments=10))
    for o in parts:
        g.set_anim(o, LIFT_ANIM + BRIDGE_ANIM)
    return parts


def decal_path(key):
    return os.path.join(OUT_LIVERIES, f'{key}_side.png')


def liveries(out):
    return {key: decal_path(key) for key in OPERATORS if os.path.exists(decal_path(key))}


def decals(f, c):
    path = decal_path('sats')
    if not os.path.exists(path):
        return
    mat = g.image_mat('decal_livery_default', path, roughness=0.4)
    x0, x1 = BOX_X0 + 500, BOX_X1 - 500
    z0, z1 = BOX_Z0 + 600, BOX_Z1 - 300
    y = BOX_HY + 14
    g.set_anim(g.decal(f, 'd_livery_R', [(x0, y, z0), (x1, y, z0), (x1, y, z1), (x0, y, z1)], mat, c), LIFT_ANIM)
    g.set_anim(g.decal(f, 'd_livery_L', [(x1, -y, z0), (x0, -y, z0), (x0, -y, z1), (x1, -y, z1)], mat, c), LIFT_ANIM)


def pose_raised(coll):
    import json
    from mathutils import Matrix, Vector
    for o in coll.objects:
        steps = json.loads(o.get('gse_anim', '[]'))
        mat = Matrix.Identity(4)
        for st in steps:
            if st['kind'] == 'trans':
                mat = Matrix.Translation(Vector(st['offset1'])) @ mat
            else:
                piv = Vector(st['pivot'])
                rot = Matrix.Rotation(math.radians(st['deg1']), 4, Vector(st['axis']))
                mat = Matrix.Translation(piv) @ rot @ Matrix.Translation(-piv) @ mat
        o.matrix_world = mat @ o.matrix_world


def build(with_connection=True):
    g.reset_scene()
    f = g.Frame(4500.0)
    c = g.collection(NAME)
    m = materials()
    chassis(f, m, c)
    wheels(f, m, c)
    cab(f, m, c)
    stabilisers(f, m, c)
    scissor_frame(f, m, c)
    lift_box(f, m, c)
    bridge(f, m, c)
    decals(f, c)
    if os.environ.get('GSE_POSE') == 'raised':
        pose_raised(c)
    return c


VIEWS = {
    'three_quarter_front': dict(res=(800, 600), lens=35, samples=16, loc=(11.0, 12.0, 4.5), target=(0.0, 0.5, 1.8)),
    'three_quarter_rear': dict(res=(800, 600), lens=35, samples=16, loc=(-11.0, -9.0, 4.5), target=(0.0, 0.0, 1.8)),
    'raised': dict(res=(800, 600), lens=28, samples=16, loc=(14.0, 10.0, 6.0), target=(0.5, 0.0, 4.5)),
    'side_right': dict(res=(800, 400), lens=50, samples=16, loc=(16.0, 0.0, 2.2), target=(0.0, 0.0, 2.2)),
}
