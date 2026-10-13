import math
import os

import gselib as g

NAME = 'bulk_cart'
X0 = 2000.0
LENGTH = 4000.0
WIDTH = 1500.0
HW = WIDTH / 2
DECK_Z0 = 380.0
DECK_Z1 = 430.0
ROOF_Z = 1900.0
FRAME_X0 = 60.0
FRAME_X1 = 3440.0
REAR_AXLE = 700.0
FRONT_AXLE = 2900.0
TRACK = 1360.0
TYRE_D = 400.0
TYRE_W = 130.0
LIVERY_DIR = 'E:/fbw-gse/out/bulk_cart/liveries'
OPERATORS = ('dnata', 'swissport', 'menzies', 'sats', 'wfs')
CURTAIN_ANIM = [{'kind': 'trans', 'offset0': (0.0, 0.0, 0.0), 'offset1': (0.0, 0.0, 0.9), 'v0': 0.0, 'v1': 1.0, 'dataref': 'fbw/gse/bulk_cart/curtain'}]

VIEWS = {
    'three_quarter_open': dict(res=(800, 560), lens=35, loc=(3.6, -3.2, 2.3), target=(0.0, 0.1, 0.9), samples=16),
    'three_quarter_closed': dict(res=(800, 560), lens=35, loc=(-3.6, 3.4, 2.3), target=(0.0, 0.1, 0.9), samples=16),
    'side_open': dict(res=(800, 400), lens=40, loc=(7.0, 0.0, 1.0), target=(0.0, 0.0, 0.95), samples=16),
    'top': dict(res=(800, 420), lens=18, loc=(0.0, 0.0, 9.0), target=(0.0, 0.0, 0.0), samples=16),
}


def materials():
    return {
        'steel': g.mat('gse_cart_steel', (150, 153, 155), metallic=0.8, roughness=0.45),
        'dark_steel': g.mat('gse_cart_dark_steel', (52, 54, 57), metallic=0.6, roughness=0.5),
        'rubber': g.mat('gse_rubber', (28, 28, 28), roughness=0.88),
        'rim': g.mat('gse_rim', (34, 34, 35), metallic=0.4, roughness=0.5),
        'hub': g.mat('gse_hub', (150, 150, 148), metallic=0.9, roughness=0.35),
        'floor': g.mat('gse_cart_floor', (70, 72, 70), roughness=0.7),
        'canvas': g.mat('gse_cart_canvas', (58, 66, 80), roughness=0.9),
        'canvas_dark': g.mat('gse_cart_canvas_dark', (36, 41, 52), roughness=0.9),
        'roof': g.mat('gse_cart_roof', (205, 207, 205), roughness=0.5),
        'red': g.mat('gse_red', (185, 18, 20), roughness=0.3),
        'yellow': g.mat('gse_yellow', (245, 200, 0), roughness=0.35),
        'bag_blue': g.mat('gse_bag_blue', (30, 60, 130), roughness=0.6),
        'bag_black': g.mat('gse_bag_black', (24, 24, 26), roughness=0.6),
        'bag_red': g.mat('gse_bag_red', (140, 30, 30), roughness=0.6),
        'bag_tan': g.mat('gse_bag_tan', (150, 120, 80), roughness=0.7),
        'bag_grey': g.mat('gse_bag_grey', (110, 112, 115), roughness=0.55),
    }


def chassis(f, m, c):
    parts = []
    for k in (-1, 1):
        parts.append(g.box(f, f'rail_{k}', FRAME_X0, FRAME_X1, k * (HW - 90) - 40, k * (HW - 90) + 40, 300, DECK_Z0, m['dark_steel'], c, bevel=5))
        parts.append(g.box(f, f'curb_{k}', FRAME_X0, FRAME_X1, min(k * HW - 30, k * HW + 30), max(k * HW - 30, k * HW + 30), DECK_Z0, DECK_Z1 + 40, m['steel'], c, bevel=6))
    for x in (200, 900, 1600, 2300, 3000, 3350):
        parts.append(g.box(f, f'cross_{x}', x - 35, x + 35, -HW + 40, HW - 40, 320, DECK_Z0, m['dark_steel'], c, bevel=4))
    parts.append(g.box(f, 'deck', FRAME_X0, FRAME_X1, -HW + 30, HW - 30, DECK_Z0, DECK_Z1, m['floor'], c, bevel=4))
    for k in (-1, 1):
        parts.append(g.box(f, f'bumper_rear_{k}', FRAME_X0 - 40, FRAME_X0, k * 600 - 120, k * 600 + 120, 330, 450, m['black'] if 'black' in m else m['dark_steel'], c, bevel=6))
    parts.append(g.box(f, 'rear_hitch_plate', -90, FRAME_X0, -180, 180, 320, 420, m['dark_steel'], c, bevel=6))
    parts.append(g.cylinder(f, 'rear_hitch_pin', (-40, 0, 420), 'z', 22, 120, m['steel'], c, segments=20))
    parts.append(g.tube(f, 'rear_hitch_eye', [(-90, -90, 370), (-140, -90, 370), (-140, 90, 370), (-90, 90, 370)], 18, m['steel'], c, segments=12))
    return parts


def wheels(f, m, c):
    parts = []
    r = TYRE_D / 2
    rr = 110.0
    tyre = [(rr, -TYRE_W / 2 + 12), (r - 25, -TYRE_W / 2), (r - 6, -TYRE_W / 2 + 10), (r, -TYRE_W / 2 + 28),
            (r, TYRE_W / 2 - 28), (r - 6, TYRE_W / 2 - 10), (r - 25, TYRE_W / 2), (rr, TYRE_W / 2 - 12)]
    rim = [(0.0, 40.0), (50.0, 40.0), (60.0, 28.0), (rr - 2, 24.0), (rr + 4, 50.0)]
    for x in (REAR_AXLE, FRONT_AXLE):
        for k in (-1, 1):
            y = k * TRACK / 2
            nm = f'{"rear" if x == REAR_AXLE else "front"}_{"L" if k < 0 else "R"}'
            parts.append(g.lathe(f, f'tyre_{nm}', (x, y, r), (k, 0, 0), tyre, m['rubber'], c, segments=56))
            for b in range(36):
                a = 2 * math.pi * b / 36
                parts.append(g.box(f, f'tread_{nm}_{b}', x + (r - 2) * math.cos(a) - 7, x + (r - 2) * math.cos(a) + 7, y - TYRE_W / 2 + 22, y + TYRE_W / 2 - 22, r + (r - 2) * math.sin(a) - 9, r + (r - 2) * math.sin(a) + 9, m['rubber'], c, bevel=2))
            parts.append(g.lathe(f, f'rim_{nm}', (x, y, r), (k, 0, 0), [(rad, -z) for rad, z in rim], m['rim'], c, segments=32))
            parts.append(g.cylinder(f, f'hubcap_{nm}', (x, y + k * 40, r), 'y', 34, 20, m['hub'], c, segments=20, bevel=3))
            parts.append(g.box(f, f'fender_{nm}', x - 230, x + 230, min(k * (TRACK / 2 - 70), k * (TRACK / 2 + 70)), max(k * (TRACK / 2 - 70), k * (TRACK / 2 + 70)), r + 90, r + 100, m['dark_steel'], c, bevel=2))
        parts.append(g.cylinder(f, f'axle_{int(x)}', (x, 0, r), 'y', 32, TRACK - 120, m['steel'], c, segments=16))
        parts.append(g.box(f, f'axle_beam_{int(x)}', x - 60, x + 60, -TRACK / 2 + 80, TRACK / 2 - 80, r + 40, 330, m['dark_steel'], c, bevel=4))
    parts.append(g.cylinder(f, 'turntable', (FRONT_AXLE, 0, 345), 'z', 220, 30, m['steel'], c, segments=40, bevel=3))
    return parts


def drawbar(f, m, c):
    parts = []
    parts.append(g.tube(f, 'drawbar_left', [(FRONT_AXLE, -160, 340), (3500, -140, 360), (3900, -30, 330)], 24, m['dark_steel'], c, segments=14))
    parts.append(g.tube(f, 'drawbar_right', [(FRONT_AXLE, 160, 340), (3500, 140, 360), (3900, 30, 330)], 24, m['dark_steel'], c, segments=14))
    parts.append(g.box(f, 'drawbar_cross', 3380, 3440, -170, 170, 320, 380, m['dark_steel'], c, bevel=5))
    parts.append(g.tube(f, 'drawbar_stem', [(3880, 0, 330), (4000, 0, 330)], 26, m['dark_steel'], c, segments=14))
    parts.append(g.lathe(f, 'tow_eye', (4010, 0, 330), (0, 0, 1), [(36, -14), (60, -14), (60, 14), (36, 14), (36, -14)], m['steel'], c, segments=28))
    parts.append(g.tube(f, 'drawbar_handle', [(3600, -120, 370), (3600, -120, 700), (3600, 120, 700), (3600, 120, 370)], 14, m['steel'], c, segments=12))
    parts.append(g.box(f, 'tow_reflector_l', 3450, 3470, -450, -330, 340, 400, m['red'], c))
    parts.append(g.box(f, 'tow_reflector_r', 3450, 3470, 330, 450, 340, 400, m['red'], c))
    return parts


def roof_profile(n=14):
    half = HW - 20
    pts = []
    for i in range(n + 1):
        t = -1 + 2.0 * i / n
        pts.append((t * half, ROOF_Z + 80.0 * (1 - t * t)))
    return pts


def frame(f, m, c):
    parts = []
    xs = (FRAME_X0 + 30, 1000.0, 2000.0, FRAME_X1 - 30)
    for k in (-1, 1):
        for x in xs:
            parts.append(g.box(f, f'post_{k}_{int(x)}', x - 25, x + 25, k * (HW - 45) - 25, k * (HW - 45) + 25, DECK_Z1, ROOF_Z, m['steel'], c, bevel=4))
        parts.append(g.box(f, f'roof_rail_{k}', FRAME_X0, FRAME_X1, k * (HW - 45) - 35, k * (HW - 45) + 35, ROOF_Z - 40, ROOF_Z + 10, m['steel'], c, bevel=5))
        parts.append(g.box(f, f'lower_rail_{k}', FRAME_X0, FRAME_X1, k * (HW - 45) - 20, k * (HW - 45) + 20, 560, 600, m['steel'], c, bevel=3))
    for x in xs:
        parts.append(g.box(f, f'roof_bow_{int(x)}', x - 30, x + 30, -HW + 10, HW - 10, ROOF_Z - 40, ROOF_Z + 5, m['steel'], c, bevel=4))
    prof = roof_profile()
    shell = prof + [(y, z - 30) for y, z in reversed(prof)]
    parts.append(g.prism_yz(f, 'roof_sheet', shell, FRAME_X0 - 20, FRAME_X1 + 40, m['roof'], c, bevel=3))
    parts.append(g.box(f, 'roof_front_lip', FRAME_X1 + 40, FRAME_X1 + 55, -HW + 20, HW - 20, ROOF_Z - 30, ROOF_Z + 20, m['roof'], c, bevel=3))
    for x in (400, 1500, 2600):
        parts.append(g.box(f, f'roof_rib_{x}', x - 25, x + 25, -HW + 25, HW - 25, ROOF_Z + 55, ROOF_Z + 85, m['steel'], c, bevel=4))
    for k in (-1, 1):
        parts.append(g.box(f, f'roof_marker_{k}', FRAME_X1 + 55, FRAME_X1 + 62, k * 550 - 80, k * 550 + 80, ROOF_Z - 10, ROOF_Z + 14, m['yellow'], c))
    return parts


def curtains(f, m, c):
    parts = []
    y = -(HW - 25)
    parts.append(g.box(f, 'curtain_closed_side', FRAME_X0 + 10, FRAME_X1 - 10, y - 6, y + 6, 600, ROOF_Z - 45, m['canvas'], c, bevel=2))
    for i in range(1, 18):
        x = FRAME_X0 + 10 + i * (FRAME_X1 - FRAME_X0 - 20) / 18
        parts.append(g.box(f, f'curtain_fold_{i}', x - 7, x + 7, y - 11, y + 8, 610, ROOF_Z - 55, m['canvas_dark'], c, bevel=2))
    parts.append(g.box(f, 'curtain_skirt_side', FRAME_X0 + 10, FRAME_X1 - 10, y - 4, y + 4, DECK_Z1, 600, m['canvas_dark'], c, bevel=2))
    for i in range(9):
        x = FRAME_X0 + 120 + i * 400
        parts.append(g.box(f, f'curtain_strap_{i}', x - 15, x + 15, y - 14, y - 6, 700, 760, m['dark_steel'], c, bevel=2))
    ky = HW - 40
    parts.append(g.cylinder(f, 'curtain_roll', (X0 + 20, ky, ROOF_Z - 110), 'x', 70, FRAME_X1 - FRAME_X0 - 60, m['canvas'], c, segments=24))
    for i in range(6):
        x = FRAME_X0 + 200 + i * 560
        parts.append(g.box(f, f'roll_tie_{i}', x - 18, x + 18, ky - 78, ky + 78, ROOF_Z - 120, ROOF_Z - 100, m['dark_steel'], c, bevel=2))
    parts.append(g.box(f, 'curtain_front', FRAME_X1 - 25, FRAME_X1 - 15, -HW + 55, HW - 55, 600, ROOF_Z - 45, m['canvas'], c, bevel=2))
    parts.append(g.box(f, 'curtain_front_skirt', FRAME_X1 - 22, FRAME_X1 - 16, -HW + 55, HW - 55, DECK_Z1, 600, m['canvas_dark'], c, bevel=2))
    parts.append(g.box(f, 'curtain_rear_rolled', FRAME_X0 + 20, FRAME_X0 + 90, -HW + 80, HW - 80, ROOF_Z - 160, ROOF_Z - 60, m['canvas'], c, bevel=20))
    for o in parts:
        if o.name.startswith('curtain_roll') or o.name.startswith('roll_tie'):
            g.set_anim(o, CURTAIN_ANIM)
    return parts


def luggage(f, m, c):
    parts = []
    c = g.collection('bulk_cart_luggage')
    spec = [
        (200, -520, 700, 400, 270, 'bag_blue'), (200, -90, 750, 420, 260, 'bag_black'), (200, 330, 650, 400, 300, 'bag_red'),
        (1000, -520, 720, 420, 280, 'bag_tan'), (1000, -80, 600, 380, 250, 'bag_grey'), (1000, 340, 700, 420, 300, 'bag_black'),
        (1800, -500, 800, 440, 260, 'bag_black'), (1800, -40, 700, 400, 290, 'bag_blue'), (1800, 350, 620, 360, 230, 'bag_red'),
        (300, -300, 600, 400, 250, 'bag_grey'), (1100, 0, 620, 400, 250, 'bag_blue'), (1700, 200, 500, 360, 240, 'bag_tan'),
    ]
    for i, (x, y, lx, ly, lz, col) in enumerate(spec):
        z0 = DECK_Z1
        if i >= 9:
            z0 = DECK_Z1 + {9: 270, 10: 250, 11: 230}[i]
            lz = 220
        parts.append(g.box(f, f'bag_{i}', FRAME_X0 + x, FRAME_X0 + x + lx, y - ly / 2, y + ly / 2, z0, z0 + lz, m[col], c, bevel=22, segments=3))
        parts.append(g.box(f, f'bag_handle_{i}', FRAME_X0 + x + lx / 2 - 40, FRAME_X0 + x + lx / 2 + 40, y - 6, y + 6, z0 + lz, z0 + lz + 22, m['dark_steel'], c, bevel=3))
        parts.append(g.box(f, f'bag_strap_{i}', FRAME_X0 + x + lx / 2 - 8, FRAME_X0 + x + lx / 2 + 8, y - ly / 2 - 2, y + ly / 2 + 2, z0 + 4, z0 + lz - 4, m['dark_steel'], c))
    return parts


def spring_path(x0, x1, y, z, radius, turns, n=14):
    pts = []
    for i in range(turns * n + 1):
        a = 2 * math.pi * i / n
        pts.append((x0 + (x1 - x0) * i / (turns * n), y + radius * math.cos(a), z + radius * math.sin(a)))
    return pts


def detail(f, m, c):
    parts = []
    parts.append(g.tube(f, 'drawbar_spring', spring_path(3560.0, 3830.0, 0.0, 330.0, 42.0, 9), 7, m['steel'], c, segments=8))
    parts.append(g.cylinder(f, 'drawbar_spring_sleeve', (3535.0, 0.0, 330.0), 'x', 34, 60, m['dark_steel'], c, segments=16))
    for k in (-1, 1):
        parts.append(g.box(f, f'jaw_{k}', 3870.0, 3960.0, k * 70 - 18 if k < 0 else k * 70 - 18, k * 70 + 18, 290.0, 370.0, m['steel'], c, bevel=5) if False else g.box(f, f'jaw_{k}', 3870.0, 3960.0, min(k * 70 - 18, k * 70 + 18), max(k * 70 - 18, k * 70 + 18), 285.0, 375.0, m['steel'], c, bevel=5))
        parts.append(g.box(f, f'rear_jaw_{k}', -150.0, -50.0, min(k * 90 - 14, k * 90 + 14), max(k * 90 - 14, k * 90 + 14), 330.0, 410.0, m['steel'], c, bevel=4))
        parts.append(g.box(f, f'rear_reflector_{k}', -10.0, FRAME_X0 - 1, min(k * 480, k * 600), max(k * 480, k * 600), 360.0, 420.0, m['red'], c))
        parts.append(g.box(f, f'side_reflector_{k}', 200, 320, k * HW - 6 if k < 0 else k * HW + 2, k * HW + 2 if k < 0 else k * HW + 6, 440, 520, m['red'], c))
        parts.append(g.box(f, f'side_reflector_front_{k}', 3200, 3320, k * HW - 6 if k < 0 else k * HW + 2, k * HW + 2 if k < 0 else k * HW + 6, 440, 520, m['yellow'], c))
    parts.append(g.cylinder(f, 'jaw_pin', (3915.0, 0.0, 330.0), 'z', 14, 130, m['steel'], c, segments=16))
    for i in range(18):
        x = FRAME_X0 + 30 + i * 190
        parts.append(g.box(f, f'deck_cleat_{i}', x, x + 90, -HW + 60, -HW + 80, DECK_Z1, DECK_Z1 + 10, m['dark_steel'], c))
    return parts


def livery_image(name):
    return os.path.join(LIVERY_DIR, f'{name}.png')


def decals(f, c, operator='dnata'):
    path = livery_image(operator)
    if not os.path.exists(path):
        return
    mat = g.image_mat(f'decal_{operator}', path, roughness=0.8)
    y = -(HW - 25) - 15
    g.decal(f, 'livery_side', [(FRAME_X1 - 300, y, 800), (FRAME_X0 + 300, y, 800), (FRAME_X0 + 300, y, 1650), (FRAME_X1 - 300, y, 1650)], mat, c)


def build(with_connection=True):
    g.reset_scene()
    f = g.Frame(X0)
    c = g.collection(NAME)
    m = materials()
    m['black'] = m['dark_steel']
    chassis(f, m, c)
    wheels(f, m, c)
    drawbar(f, m, c)
    frame(f, m, c)
    curtains(f, m, c)
    detail(f, m, c)
    luggage(f, m, c)
    decals(f, c)
    return c


def liveries(out):
    return {op: livery_image(op) for op in OPERATORS if os.path.exists(livery_image(op))}
