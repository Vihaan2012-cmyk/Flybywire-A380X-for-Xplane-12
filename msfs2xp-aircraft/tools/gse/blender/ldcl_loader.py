import math
import os
import subprocess
import sys

import bpy

import gselib as g

NAME = 'ldcl_loader'
LIVERY_DIR = os.path.join('E:/fbw-gse/out', NAME, 'liveries')
PAINT_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ldcl_livery_paint.py')
WIDTH = 2500.0
HY = WIDTH / 2
FRAME_Z0 = 420.0
FRAME_Z1 = 760.0
DECK_Z_LOW = 1020.0
DECK_THICK = 90.0
BRIDGE_X = (5000.0, 7300.0)
REAR_X = (1250.0, 4850.0)
DECK_HALF = 1150.0
BRIDGE_TRAVEL = 1750.0
REAR_TRAVEL = 1650.0
AXLES = (1000.0, 5600.0)
TRACK = 2000.0
TYRE_D = 760.0
TYRE_W = 270.0
ROLLER_PITCH = 95.0
OPERATORS = ('dnata', 'swissport', 'menzies')


def anim(dataref, travel):
    return [{'kind': 'trans', 'offset0': (0.0, 0.0, 0.0), 'offset1': (0.0, 0.0, travel / 1000.0), 'v0': 0.0, 'v1': 1.0, 'dataref': dataref}]


def materials():
    return {
        'yellow': g.mat('gse_ldcl_yellow', (240, 190, 10), roughness=0.38),
        'white': g.mat('gse_white', (236, 237, 233), roughness=0.32),
        'galv': g.mat('gse_galvanized', (168, 171, 172), metallic=0.85, roughness=0.42),
        'steel': g.mat('gse_roller_steel', (140, 143, 146), metallic=0.9, roughness=0.3),
        'black': g.mat('gse_black', (22, 22, 23), roughness=0.55),
        'deck': g.mat('gse_deck_grey', (70, 72, 74), metallic=0.3, roughness=0.6),
        'glass': g.mat('gse_cab_glass', (30, 45, 55), roughness=0.06),
        'rubber': g.mat('gse_rubber', (28, 28, 28), roughness=0.88),
        'rim': g.mat('gse_rim', (34, 34, 35), metallic=0.4, roughness=0.5),
        'hub': g.mat('gse_hub', (150, 150, 148), metallic=0.9, roughness=0.35),
        'amber': g.mat('gse_amber_lens', (255, 140, 10), roughness=0.12),
        'red': g.mat('gse_red', (185, 18, 20), roughness=0.3),
        'rail_red': g.mat('gse_rail_red', (200, 30, 25), roughness=0.4),
        'orange': g.mat('gse_hyd_orange', (230, 110, 10), roughness=0.4),
        'hose': g.mat('gse_hose', (18, 18, 18), roughness=0.7),
    }


def chassis(f, m, c):
    p = []
    p.append(g.box(f, 'frame_main', 300, 7300, -HY + 150, HY - 150, FRAME_Z0, FRAME_Z1, m['yellow'], c, bevel=8))
    for k in (-1, 1):
        p.append(g.box(f, f'frame_side_{k}', 200, 7400, k * (HY - 150), k * (HY - 90), 380, 900, m['yellow'], c, bevel=10))
        p.append(g.box(f, f'bumper_side_{k}', 250, 7350, k * (HY - 90), k * (HY - 60), 520, 640, m['black'], c, bevel=6))
    p.append(g.box(f, 'bumper_rear', 100, 200, -HY + 100, HY - 100, 400, 700, m['black'], c, bevel=8))
    p.append(g.box(f, 'bumper_front', 7400, 7560, -HY + 100, HY - 100, 400, 700, m['black'], c, bevel=8))
    for i, x in enumerate((5850.0, 6900.0)):
        for k in (-1, 1):
            p.append(g.box(f, f'outrigger_beam_{i}_{k}', x - 80, x + 80, k * (HY - 100), k * (HY + 110), 480, 600, m['galv'], c, bevel=5))
            p.append(g.cylinder(f, f'outrigger_leg_{i}_{k}', (x, k * (HY + 70), 290), 'z', 55, 540, m['steel'], c, segments=20))
            p.append(g.cylinder(f, f'outrigger_pad_{i}_{k}', (x, k * (HY + 70), 25), 'z', 130, 50, m['black'], c, segments=24, bevel=4))
    for x in (2400.0, 4300.0):
        p.append(g.box(f, f'fuel_tank_{int(x)}', x - 450, x + 450, -HY + 160, -HY + 460, 440, 740, m['yellow'], c, bevel=12))
    p.append(g.box(f, 'hyd_tank', 2000, 2700, HY - 460, HY - 160, 440, 740, m['orange'], c, bevel=12))
    for k in (-1, 1):
        for i, x in enumerate((400.0, 7000.0)):
            p.append(g.box(f, f'reflector_{k}_{i}', x - 80, x + 80, k * (HY - 58), k * (HY - 55), 640, 700, m['red'], c))
    p.append(g.cylinder(f, 'beacon_base', (7000, 0, 1380), 'z', 70, 60, m['black'], c, segments=20, bevel=4))
    p.append(g.lathe(f, 'beacon_lens', (7000, 0, 1410), 'z', [(0.0, 120.0), (40.0, 110.0), (60.0, 80.0), (62.0, 0.0)], m['amber'], c, segments=28))
    return p


def wheels(f, m, c):
    p = []
    r = TYRE_D / 2
    rr = 230.0
    tyre_profile = [(rr, -TYRE_W / 2 + 25), (rr + 12, -TYRE_W / 2 + 6), (r - 60, -TYRE_W / 2), (r - 18, -TYRE_W / 2 + 14), (r, -TYRE_W / 2 + 55),
                    (r, TYRE_W / 2 - 55), (r - 18, TYRE_W / 2 - 14), (r - 60, TYRE_W / 2), (rr + 12, TYRE_W / 2 - 6), (rr, TYRE_W / 2 - 25)]
    rim_profile = [(0.0, 70.0), (80.0, 70.0), (95.0, 52.0), (170.0, 44.0), (rr - 2, 40.0), (rr + 8, 95.0), (rr + 8, 110.0)]
    for x in AXLES:
        for k in (-1, 1):
            y = k * TRACK / 2
            name = f'{"rear" if x == AXLES[0] else "front"}_{"L" if k < 0 else "R"}'
            p.append(g.lathe(f, f'tyre_{name}', (x, y, r), (k, 0, 0), tyre_profile, m['rubber'], c, segments=56))
            p.append(g.lathe(f, f'rim_{name}', (x, y, r), (k, 0, 0), [(rad, -z) for rad, z in rim_profile], m['rim'], c, segments=48))
            p.append(g.cylinder(f, f'hubcap_{name}', (x, y + k * 70, r), 'y', 60, 30, m['hub'], c, segments=28, bevel=5))
            for b in range(8):
                a = 2 * math.pi * b / 8
                p.append(g.cylinder(f, f'lug_{name}_{b}', (x + 90 * math.cos(a), y + k * 66, r + 90 * math.sin(a)), 'y', 12, 22, m['hub'], c, segments=12))
            p.append(g.box(f, f'mudguard_{name}', x - 330, x + 330, y - 130, y + 130, r + 330, r + 350, m['yellow'], c, bevel=6))
        p.append(g.cylinder(f, f'axle_{int(x)}', (x, 0, r), 'y', 60, TRACK - 200, m['galv'], c, segments=20))
    return p


def scissors(f, m, c, name, xa, xb, z_low, steps):
    p = []
    ym = DECK_HALF - 250
    for k in (-1, 1):
        for i in range(2):
            xs, xe = (xa + 150, xb - 150) if i == 0 else (xb - 150, xa + 150)
            p.append(g.tube(f, f'{name}_scissor_{k}_{i}', [(xs, k * ym, FRAME_Z1 + 40), (xe, k * ym, z_low - 40)], 55, m['yellow'], c, segments=12))
        p.append(g.cylinder(f, f'{name}_pivot_{k}', ((xa + xb) / 2, k * ym, (FRAME_Z1 + z_low) / 2), 'y', 45, 160, m['galv'], c, segments=20))
        p.append(g.cylinder(f, f'{name}_ram_{k}', ((xa + xb) / 2, k * (ym - 160), FRAME_Z1 + 130), 'x', 38, (xb - xa) * 0.4, m['steel'], c, segments=16))
    for o in p:
        g.set_anim(o, steps)
    return p


def platform(f, m, c, name, x0, x1, dataref, travel):
    p = []
    steps = anim(dataref, travel)
    z0 = DECK_Z_LOW
    p.append(g.box(f, f'{name}_deck', x0, x1, -DECK_HALF, DECK_HALF, z0, z0 + DECK_THICK, m['deck'], c, bevel=6))
    p.append(g.box(f, f'{name}_frame_L', x0, x1, -DECK_HALF - 40, -DECK_HALF + 50, z0 - 40, z0 + 130, m['yellow'], c, bevel=8))
    p.append(g.box(f, f'{name}_frame_R', x0, x1, DECK_HALF - 50, DECK_HALF + 40, z0 - 40, z0 + 130, m['yellow'], c, bevel=8))
    p.append(g.box(f, f'{name}_frame_end0', x0 - 40, x0 + 40, -DECK_HALF - 40, DECK_HALF + 40, z0 - 40, z0 + 130, m['yellow'], c, bevel=8))
    p.append(g.box(f, f'{name}_frame_end1', x1 - 40, x1 + 40, -DECK_HALF - 40, DECK_HALF + 40, z0 - 40, z0 + 130, m['yellow'], c, bevel=8))
    rz = z0 + DECK_THICK + 35
    count = int((x1 - x0 - 200) / ROLLER_PITCH)
    for lane in (-1, 1):
        for i in range(count):
            x = x0 + 100 + i * ROLLER_PITCH
            p.append(g.cylinder(f, f'{name}_roller_{lane}_{i}', (x, lane * 540, rz), 'y', 32, 960, m['steel'], c, segments=28))
    p.append(g.box(f, f'{name}_centre_guide', x0 + 60, x1 - 60, -45, 45, z0 + DECK_THICK, z0 + DECK_THICK + 90, m['galv'], c, bevel=4))
    for k in (-1, 1):
        p.append(g.box(f, f'{name}_guide_rail_{k}', x0 + 60, x1 - 60, k * (DECK_HALF - 80) - 25, k * (DECK_HALF - 80) + 25, z0 + DECK_THICK, z0 + DECK_THICK + 110, m['black'], c, bevel=5))
        for j in range(4):
            px = x0 + 80 + j * (x1 - x0 - 160) / 3
            p.append(g.cylinder(f, f'{name}_post_{k}_{j}', (px, k * (DECK_HALF + 5), z0 + 480), 'z', 22, 700, m['rail_red'], c, segments=12))
        p.append(g.tube(f, f'{name}_handrail_{k}', [(x0 + 80, k * (DECK_HALF + 5), z0 + 800), (x1 - 80, k * (DECK_HALF + 5), z0 + 800)], 22, m['rail_red'], c, segments=12))
        p.append(g.tube(f, f'{name}_midrail_{k}', [(x0 + 80, k * (DECK_HALF + 5), z0 + 500), (x1 - 80, k * (DECK_HALF + 5), z0 + 500)], 18, m['rail_red'], c, segments=10))
    p.append(g.box(f, f'{name}_toe_skirt', x0, x1, -DECK_HALF - 45, DECK_HALF + 45, z0 - 130, z0 - 40, m['black'], c, bevel=4))
    for o in p:
        g.set_anim(o, steps)
    return p + scissors(f, m, c, name, x0, x1, z0, steps)


def bridge_extras(f, m, c):
    p = []
    steps = anim('fbw/gse/ldcl/bridge', BRIDGE_TRAVEL)
    z0 = DECK_Z_LOW + DECK_THICK
    p.append(g.box(f, 'bridge_lip', 7300, 7480, -DECK_HALF + 100, DECK_HALF - 100, z0 - 10, z0 + 60, m['galv'], c, bevel=6))
    p.append(g.box(f, 'bridge_bumper', 7480, 7540, -DECK_HALF + 100, DECK_HALF - 100, z0 - 40, z0 + 200, m['black'], c, bevel=10))
    for k in (-1, 1):
        p.append(g.cylinder(f, f'bridge_light_{k}', (7310, k * 900, z0 + 160), 'x', 45, 40, m['amber'], c, segments=20, bevel=4))
    for o in p:
        g.set_anim(o, steps)
    return p


def cab(f, m, c):
    p = []
    p.append(g.box(f, 'cab_body', 5000, 5950, HY - 1000, HY - 40, 760, 1450, m['yellow'], c, bevel=25))
    p.append(g.box(f, 'cab_roof', 4980, 5970, HY - 1020, HY - 20, 1450, 1520, m['white'], c, bevel=22))
    p.append(g.box(f, 'cab_windscreen', 5948, 5956, HY - 940, HY - 100, 980, 1400, m['glass'], c, bevel=2))
    p.append(g.box(f, 'cab_rear_window', 4994, 5002, HY - 940, HY - 100, 980, 1400, m['glass'], c, bevel=2))
    p.append(g.box(f, 'cab_side_window', 5100, 5860, HY - 44, HY - 38, 980, 1400, m['glass'], c, bevel=2))
    p.append(g.box(f, 'cab_inner_window', 5100, 5860, HY - 1002, HY - 996, 980, 1400, m['glass'], c, bevel=2))
    for i in range(3):
        p.append(g.box(f, f'cab_step_{i}', 5100, 5800, HY - 40, HY + 160, 420 + i * 120, 450 + i * 120, m['galv'], c, bevel=3))
    p.append(g.tube(f, 'cab_handrail', [(5050, HY + 150, 520), (5050, HY + 150, 1250), (5800, HY + 150, 1250), (5800, HY + 150, 520)], 18, m['rail_red'], c, segments=12))
    p.append(g.box(f, 'engine_cover', 300, 1100, -HY + 150, HY - 150, FRAME_Z1, 1250, m['yellow'], c, bevel=25))
    p.append(g.box(f, 'engine_grille', 280, 300, -HY + 400, HY - 400, 900, 1150, m['black'], c, bevel=3))
    for i in range(7):
        p.append(g.box(f, f'grille_slat_{i}', 270, 285, -HY + 420, HY - 420, 920 + i * 30, 935 + i * 30, m['steel'], c))
    return p


def hoses(f, m, c):
    p = []
    for k, (xa, xb) in enumerate(((REAR_X[0] + 300, REAR_X[1] - 300), (BRIDGE_X[0] + 300, BRIDGE_X[1] - 300))):
        pts = [(xa, -HY + 200, 740), (xa + 100, -HY + 300, 650), ((xa + xb) / 2, -HY + 320, 620), (xb, -HY + 300, 700), (xb, -DECK_HALF + 200, 900)]
        p.append(g.tube(f, f'hose_{k}', g.smooth_path(pts, 8), 20, m['hose'], c, segments=10))
    return p


def livery_path(operator):
    return os.path.join(LIVERY_DIR, f'{operator}.png')


def liveries(out):
    os.makedirs(LIVERY_DIR, exist_ok=True)
    if not all(os.path.exists(livery_path(op)) for op in OPERATORS):
        subprocess.run(['python', '-I', PAINT_SCRIPT, LIVERY_DIR], check=False)
    return {op: livery_path(op) for op in OPERATORS if os.path.exists(livery_path(op))}


def decals(f, c, operator='dnata'):
    path = livery_path(operator)
    if not os.path.exists(path):
        return
    mat = g.image_mat(f'decal_{operator}', path, roughness=0.4)
    for k in (-1, 1):
        y = k * (HY - 56)
        x0, x1 = 2300.0, 4300.0
        if k > 0:
            corners = [(x0, y, 520), (x1, y, 520), (x1, y, 720), (x0, y, 720)]
        else:
            corners = [(x1, y, 520), (x0, y, 520), (x0, y, 720), (x1, y, 720)]
        g.decal(f, f'd_operator_{k}', corners, mat, c)


def build(with_connection=True):
    g.reset_scene()
    f = g.Frame(0.0)
    c = g.collection(NAME)
    m = materials()
    chassis(f, m, c)
    wheels(f, m, c)
    cab(f, m, c)
    platform(f, m, c, 'rear', REAR_X[0], REAR_X[1], 'fbw/gse/ldcl/rear', REAR_TRAVEL)
    platform(f, m, c, 'bridge', BRIDGE_X[0], BRIDGE_X[1], 'fbw/gse/ldcl/bridge', BRIDGE_TRAVEL)
    bridge_extras(f, m, c)
    hoses(f, m, c)
    liveries(None)
    decals(f, c)
    return c


VIEWS = {
    'three_quarter_front': dict(res=(800, 560), samples=16, lens=35, loc=(5.6, -9.5, 4.2), target=(0.3, 3.7, 1.4)),
    'three_quarter_rear': dict(res=(800, 560), samples=16, lens=35, loc=(-6.2, 8.5, 3.8), target=(0.0, 3.4, 1.3)),
    'side_left': dict(res=(800, 400), samples=16, lens=35, loc=(-13.0, 3.8, 1.6), target=(0.0, 3.8, 1.5)),
    'front': dict(res=(640, 480), samples=16, lens=35, loc=(0.0, 14.0, 2.0), target=(0.0, 0.0, 1.5)),
}
