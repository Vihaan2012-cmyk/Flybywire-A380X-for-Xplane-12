import math
import os

import gselib as g

NAME = 'fuel_hydrant'
X0 = 3700.0
W = 2500.0
HY = W / 2
FRAME_Z0 = 820.0
FRAME_Z1 = 960.0
REAR_AXLE_1 = 1500.0
REAR_AXLE_2 = 2850.0
FRONT_AXLE = 6050.0
TRACK = 2050.0
TYRE_R = 530.0
TYRE_W = 295.0
DECK_Z = 2300.0
DECK_X0 = 120.0
DECK_X1 = 1500.0
CAB_X0 = 5150.0
CAB_X1 = 7350.0
BODY_X0 = 1650.0
BODY_X1 = 5000.0
PLATFORM_LIFT = 1.8
HOSE_LIFT = 3.1
PLATFORM_ANIM = [{'kind': 'trans', 'offset0': (0.0, 0.0, 0.0), 'offset1': (0.0, 0.0, PLATFORM_LIFT), 'v0': 0.0, 'v1': 1.0, 'dataref': 'fbw/gse/fuel_hydrant/platform'}]
HOSE_ANIM = [{'kind': 'trans', 'offset0': (0.0, 0.0, 0.0), 'offset1': (0.0, 0.0, HOSE_LIFT), 'v0': 0.0, 'v1': 1.0, 'dataref': 'fbw/gse/fuel_hydrant/hose'}]
LIVERY_OUT = 'E:/fbw-gse/out/fuel_hydrant'
DEFAULT_OPERATOR = 'Shell Aviation'
LIVERIES = {
    'Shell Aviation': ((251, 206, 0), (221, 29, 33)),
    'Air BP': ((0, 144, 60), (255, 242, 0)),
    'ExxonMobil Aviation': ((220, 30, 40), (20, 40, 110)),
    'Skytanking': ((0, 70, 140), (235, 235, 235)),
}

VIEWS = {
    'three_quarter_rear': dict(res=(800, 560), lens=32, loc=(9.0, 7.5, 3.8), target=(0.0, 0.0, 1.9), samples=16),
    'three_quarter_front': dict(res=(800, 560), lens=32, loc=(-11.0, -7.0, 3.4), target=(0.0, 0.0, 1.7), samples=16),
    'side_right': dict(res=(800, 400), lens=40, loc=(0.0, 13.0, 1.9), target=(0.0, 0.0, 1.9), samples=16),
    'rear_end': dict(res=(600, 600), lens=35, loc=(11.0, 0.0, 3.3), target=(0.0, 0.0, 2.1), samples=16),
}


def materials():
    return {
        'body': g.mat('hyd_body', (236, 236, 232), roughness=0.38),
        'stripe': g.mat('hyd_stripe', (200, 30, 32), roughness=0.38),
        'steel': g.mat('hyd_steel', (175, 178, 180), metallic=0.85, roughness=0.38),
        'alu': g.mat('hyd_aluminium', (205, 207, 208), metallic=0.8, roughness=0.45),
        'frame': g.mat('hyd_frame', (40, 41, 43), roughness=0.55),
        'black': g.mat('hyd_black', (22, 22, 23), roughness=0.55),
        'glass': g.mat('hyd_glass', (30, 42, 52), roughness=0.06),
        'rubber': g.mat('hyd_rubber', (26, 26, 26), roughness=0.9),
        'rim': g.mat('hyd_rim', (190, 190, 188), metallic=0.5, roughness=0.45),
        'hose': g.mat('hyd_hose', (22, 24, 28), roughness=0.75),
        'hose_red': g.mat('hyd_hose_red', (170, 20, 24), roughness=0.6),
        'yellow': g.mat('hyd_yellow', (245, 200, 0), roughness=0.4),
        'amber': g.mat('hyd_amber', (255, 140, 10), roughness=0.12),
        'red': g.mat('hyd_red', (185, 18, 20), roughness=0.3),
        'lamp': g.mat('hyd_lamp', (250, 250, 235), roughness=0.1),
        'brass': g.mat('hyd_brass', (190, 150, 60), metallic=0.9, roughness=0.35),
        'deck': g.mat('hyd_deck', (92, 94, 96), metallic=0.6, roughness=0.6),
    }


def livery_file(operator):
    return os.path.join(LIVERY_OUT, 'liveries', operator.lower().replace(' ', '_') + '.png')


def liveries(out=LIVERY_OUT):
    from PIL import Image, ImageDraw, ImageFont
    folder = os.path.join(out, 'liveries')
    os.makedirs(folder, exist_ok=True)
    result = {}
    try:
        font = ImageFont.truetype('arialbd.ttf', 150)
        small = ImageFont.truetype('arialbd.ttf', 56)
    except OSError:
        font = small = ImageFont.load_default()
    for operator, (main, accent) in LIVERIES.items():
        img = Image.new('RGBA', (2048, 512), main + (255,))
        d = ImageDraw.Draw(img)
        d.rectangle((0, 400, 2048, 470), fill=accent + (255,))
        d.rectangle((0, 478, 2048, 498), fill=accent + (255,))
        text_col = (255, 255, 255) if sum(main) < 400 else (20, 20, 20)
        d.text((90, 90), operator.upper(), font=font, fill=text_col + (255,))
        d.text((96, 300), 'AVIATION FUEL  JET A-1  HYDRANT DISPENSER', font=small, fill=text_col + (255,))
        path = os.path.join(folder, operator.lower().replace(' ', '_') + '.png')
        img.save(path)
        result[operator] = path
    return result


def wheel_set(f, m, c, x, y, k, tag):
    parts = []
    r = TYRE_R
    profile = [(150.0, -TYRE_W / 2 + 20), (r - 30, -TYRE_W / 2), (r - 8, -TYRE_W / 2 + 20), (r, -TYRE_W / 2 + 60), (r, TYRE_W / 2 - 60), (r - 8, TYRE_W / 2 - 20), (r - 30, TYRE_W / 2), (150.0, TYRE_W / 2 - 20)]
    parts.append(g.lathe(f, f'tyre_{tag}', (x, y, r), (k, 0, 0), profile, m['rubber'], c, segments=44))
    parts.append(g.lathe(f, f'rim_{tag}', (x, y, r), (k, 0, 0), [(0.0, 130.0), (120.0, 130.0), (170.0, 100.0), (225.0, 120.0), (232.0, 150.0), (232.0, 165.0)], m['rim'], c, segments=36))
    for b in range(10):
        a = 2 * math.pi * b / 10
        parts.append(g.cylinder(f, f'nut_{tag}_{b}', (x + 95 * math.cos(a), y + k * 135, r + 95 * math.sin(a)), 'y', 11, 22, m['steel'], c, segments=8))
    return parts


def chassis(f, m, c):
    parts = []
    for k in (-1, 1):
        parts.append(g.box(f, f'rail_{k}', 0, 7500, k * 420 - 55, k * 420 + 55, FRAME_Z0, FRAME_Z1 + 100, m['frame'], c, bevel=6))
    for i, x in enumerate((300, 1500, 2850, 4300, 5400, 6600)):
        parts.append(g.box(f, f'crossmember_{i}', x - 40, x + 40, -HY + 400, HY - 400, FRAME_Z0 + 40, FRAME_Z1 + 40, m['frame'], c, bevel=4))
    for k in (-1, 1):
        side = 'L' if k < 0 else 'R'
        for x, nm in ((REAR_AXLE_1, 'r1'), (REAR_AXLE_2, 'r2')):
            for i, off in enumerate((0.0, 330.0)):
                parts += wheel_set(f, m, c, x, k * (TRACK / 2 + off - 165.0), k, f'{nm}_{side}{i}')
        parts += wheel_set(f, m, c, FRONT_AXLE, k * TRACK / 2, k, f'f_{side}')
    for x in (REAR_AXLE_1, REAR_AXLE_2, FRONT_AXLE):
        parts.append(g.cylinder(f, f'axle_{int(x)}', (x, 0, TYRE_R), 'y', 70, TRACK - 200, m['frame'], c, segments=20))
    for k in (-1, 1):
        for x, span in (((REAR_AXLE_1 + REAR_AXLE_2) / 2, 1650.0), (FRONT_AXLE, 760.0)):
            half = span / 2
            top = TYRE_R + 700
            pts = [(x - half - 120, TYRE_R + 200), (x - half, TYRE_R + 560), (x - half * 0.5, top), (x + half * 0.5, top), (x + half, TYRE_R + 560), (x + half + 120, TYRE_R + 200),
                   (x + half + 100, TYRE_R + 200), (x + half - 20, TYRE_R + 540), (x + half * 0.5, top - 50), (x - half * 0.5, top - 50), (x - half + 20, TYRE_R + 540), (x - half - 100, TYRE_R + 200)]
            yc = k * (TRACK / 2 + (165 if span > 1000 else 0))
            parts.append(g.prism_xz(f, f'fender_{k}_{int(x)}', pts, yc - 260, yc + 260, m['frame'], c, bevel=3))
    parts.append(g.box(f, 'rear_bumper', -60, 40, -HY + 100, HY - 100, 700, 960, m['frame'], c, bevel=6))
    parts.append(g.box(f, 'rear_bumper_stripe', -64, -60, -HY + 100, HY - 100, 780, 880, m['yellow'], c))
    parts.append(g.box(f, 'front_bumper', 7380, 7560, -HY + 60, HY - 60, 480, 780, m['frame'], c, bevel=12))
    parts.append(g.box(f, 'front_bumper_stripe', 7556, 7562, -HY + 120, HY - 120, 600, 700, m['yellow'], c))
    for k in (-1, 1):
        parts.append(g.box(f, f'rear_light_{k}', -66, -58, k * 900 - 90, k * 900 + 90, 900, 1000, m['red'], c, bevel=3))
        parts.append(g.box(f, f'side_marker_{k}', 3500, 3620, k * (HY - 80) + k * 4, k * (HY - 80) + k * 10, 900, 960, m['amber'], c))
    return parts


def cab(f, m, c):
    parts = []
    profile = [(CAB_X0, 960.0), (CAB_X0, 3050.0), (CAB_X0 + 150, 3150.0), (CAB_X1 - 480, 3150.0), (CAB_X1 - 330, 3060.0), (CAB_X1 - 60, 2000.0), (CAB_X1, 1800.0), (CAB_X1, 1000.0), (CAB_X1 - 40, 960.0)]
    parts.append(g.prism_xz(f, 'cab_shell', profile, -HY + 100, HY - 100, m['body'], c, bevel=22))
    parts.append(g.prism_xz(f, 'windscreen', [(CAB_X1 - 330 + 6, 3050.0), (CAB_X1 - 60 + 6, 2080.0), (CAB_X1 - 60 + 22, 2080.0), (CAB_X1 - 330 + 22, 3050.0)], -HY + 260, HY - 260, m['glass'], c))
    for k in (-1, 1):
        y = k * (HY - 100)
        parts.append(g.box(f, f'door_{k}', 5500, 6700, min(y - k * 2, y + k * 12), max(y - k * 2, y + k * 12), 1050, 2900, m['body'], c, bevel=10))
        parts.append(g.box(f, f'door_window_{k}', 5600, 6500, min(y + k * 12, y + k * 16), max(y + k * 12, y + k * 16), 2150, 2800, m['glass'], c, bevel=3))
        parts.append(g.box(f, f'door_handle_{k}', 6560, 6680, min(y + k * 12, y + k * 40), max(y + k * 12, y + k * 40), 2050, 2090, m['black'], c, bevel=4))
        parts.append(g.box(f, f'step_{k}', 5700, 6500, min(k * (HY - 100), k * (HY + 80)), max(k * (HY - 100), k * (HY + 80)), 500, 540, m['deck'], c, bevel=3))
        for xs in (5700, 6440):
            parts.append(g.box(f, f'step_bracket_{k}_{xs}', xs, xs + 60, min(k * (HY - 100), k * (HY + 80)), max(k * (HY - 100), k * (HY + 80)), 540, 880, m['frame'], c))
        parts.append(g.box(f, f'headlamp_{k}', CAB_X1 - 6, CAB_X1 + 20, k * 900 - 150, k * 900 + 150, 1350, 1480, m['lamp'], c, bevel=8))
        parts.append(g.cylinder(f, f'indicator_{k}', (CAB_X1 + 4, k * 1120, 1415), 'x', 45, 20, m['amber'], c, segments=16))
        parts.append(g.tube(f, f'mirror_arm_{k}', [(6350.0, k * (HY - 100), 2500.0), (6350.0, k * (HY + 90), 2500.0), (6350.0, k * (HY + 190), 2480.0)], 14, m['frame'], c, segments=10))
        parts.append(g.box(f, f'mirror_{k}', 6320, 6390, min(k * (HY + 190), k * (HY + 210)), max(k * (HY + 190), k * (HY + 210)), 2300, 2750, m['black'], c, bevel=6))
    parts.append(g.box(f, 'grille', CAB_X1 - 2, CAB_X1 + 12, -520, 520, 1000, 1280, m['black'], c, bevel=4))
    for i in range(6):
        parts.append(g.box(f, f'grille_bar_{i}', CAB_X1 + 12, CAB_X1 + 20, -480, 480, 1030 + i * 38, 1050 + i * 38, m['alu'], c))
    parts.append(g.box(f, 'visor', CAB_X1 - 520, CAB_X1 - 180, -HY + 200, HY - 200, 3100, 3150, m['black'], c, bevel=4))
    parts.append(g.box(f, 'roof_light_bar', CAB_X1 - 560, CAB_X1 - 440, -420, 420, 3150, 3215, m['black'], c, bevel=4))
    for i, y in enumerate((-300, 0, 300)):
        parts.append(g.lathe(f, f'roof_beacon_{i}', (CAB_X1 - 500, y, 3215), 'z', [(0.0, 90.0), (28.0, 85.0), (40.0, 60.0), (42.0, 0.0)], m['amber'], c, segments=20))
    parts.append(g.cylinder(f, 'exhaust', (CAB_X0 - 40, HY - 60, 2200), 'z', 70, 2200, m['steel'], c, segments=16))
    return parts


def vessel(f, m, c, name, x0, x1, y, z, radius):
    parts = []
    cap = 160.0
    parts.append(g.cylinder(f, f'{name}_shell', ((x0 + x1) / 2, y, z), 'x', radius, x1 - x0 - 2 * cap, m['steel'], c, segments=40))
    for sign, xe in ((-1, x0 + cap), (1, x1 - cap)):
        dome = [(radius, 0.0), (radius * 0.92, 55.0), (radius * 0.72, 110.0), (radius * 0.4, 145.0), (0.0, cap)]
        parts.append(g.lathe(f, f'{name}_dome_{sign}', (xe, y, z), (sign, 0, 0), dome, m['steel'], c, segments=40))
    for xb in (x0 + 450, x1 - 450):
        parts.append(g.cylinder(f, f'{name}_band_{int(xb)}', (xb, y, z), 'x', radius + 10, 40, m['alu'], c, segments=40))
    parts.append(g.cylinder(f, f'{name}_drain', ((x0 + x1) / 2, y, z - radius - 40), 'z', 25, 60, m['brass'], c, segments=12))
    parts.append(g.cylinder(f, f'{name}_gauge', ((x0 + x1) / 2 + 300, y, z + radius + 50), 'z', 55, 40, m['black'], c, segments=20))
    return parts


def body(f, m, c):
    parts = []
    parts.append(g.box(f, 'body_floor', BODY_X0, BODY_X1, -HY + 120, HY - 120, 960, 1010, m['frame'], c, bevel=3))
    parts.append(g.box(f, 'body_left_wall', BODY_X0, BODY_X1, -HY + 80, -HY + 160, 1010, 2500, m['body'], c, bevel=10))
    parts.append(g.box(f, 'body_right_wall', BODY_X0, BODY_X1, HY - 160, HY - 80, 1010, 2500, m['body'], c, bevel=10))
    parts.append(g.box(f, 'body_roof', BODY_X0, BODY_X1, -HY + 80, HY - 80, 2500, 2560, m['body'], c, bevel=12))
    parts.append(g.box(f, 'body_front_wall', BODY_X1 - 60, BODY_X1, -HY + 80, HY - 80, 1010, 2500, m['body'], c, bevel=8))
    parts.append(g.box(f, 'body_rear_wall', BODY_X0, BODY_X0 + 40, -HY + 80, HY - 80, 1010, 2500, m['body'], c, bevel=8))
    parts.append(g.box(f, 'body_inner_dark', BODY_X0 + 40, BODY_X1 - 60, -HY + 160, HY - 160, 1010, 2490, m['frame'], c))
    parts += vessel(f, m, c, 'filter_a', BODY_X0 + 250, BODY_X0 + 1850, -430, 1780, 340)
    parts += vessel(f, m, c, 'filter_b', BODY_X0 + 250, BODY_X0 + 1850, 430, 1780, 340)
    parts += vessel(f, m, c, 'meter_vessel', BODY_X0 + 1950, BODY_X0 + 3150, 0, 1500, 250)
    pipes = [
        [(BODY_X0 + 2100, -200, 1500), (BODY_X0 + 2100, -430, 1600), (BODY_X0 + 2000, -430, 1780)],
        [(BODY_X0 + 2100, 200, 1500), (BODY_X0 + 2100, 430, 1600), (BODY_X0 + 2000, 430, 1780)],
        [(BODY_X0 + 400, -430, 2120), (BODY_X0 + 400, 0, 2300), (BODY_X0 + 400, 430, 2120)],
        [(BODY_X0 + 3100, 0, 1500), (BODY_X0 + 3250, 0, 1500), (BODY_X0 + 3300, 600, 1250), (BODY_X0 + 3300, HY - 200, 1150)],
    ]
    for i, p in enumerate(pipes):
        parts.append(g.tube(f, f'pipe_{i}', g.smooth_path(p, 6), 38, m['alu'], c, segments=16))
    for k in (-1, 1):
        for xr, nm in ((BODY_X0 + 500, 'a'), (BODY_X0 + 1500, 'b')):
            yc = k * (HY - 360)
            parts.append(g.cylinder(f, f'reel_{nm}_{k}', (xr + 40, yc, 1750), 'y', 380, 360, m['body'], c, segments=36, bevel=6))
            for side in (-1, 1):
                parts.append(g.cylinder(f, f'reel_flange_{nm}_{k}_{side}', (xr + 40, yc + side * 190, 1750), 'y', 420, 24, m['stripe'], c, segments=36, bevel=3))
            turns = [(xr + 40 + 330 * math.cos(a / 6), yc + (a % 3 - 1) * 90, 1750 + 330 * math.sin(a / 6)) for a in range(0, 38)]
            parts.append(g.tube(f, f'reel_hose_{nm}_{k}', g.smooth_path(turns, 2), 60, m['hose'], c, segments=12))
            parts.append(g.cylinder(f, f'reel_hub_{nm}_{k}', (xr + 40, yc + k * 200, 1750), 'y', 70, 30, m['steel'], c, segments=18))
        for i, (x0, x1) in enumerate(((BODY_X0 + 40, BODY_X0 + 1100), (BODY_X0 + 1130, BODY_X0 + 2200), (BODY_X0 + 2230, BODY_X0 + 3300))):
            ya, yb = sorted((k * (HY - 80) - 2, k * (HY - 80) + k * 6))
            parts.append(g.box(f, f'locker_door_{k}_{i}', x0, x1, ya, yb, 1060, 1500, m['body'], c, bevel=4))
            ya, yb = sorted((k * (HY - 80) + k * 6, k * (HY - 80) + k * 30))
            parts.append(g.box(f, f'locker_handle_{k}_{i}', x1 - 140, x1 - 40, ya, yb, 1260, 1290, m['black'], c, bevel=3))
    return parts


def hydrant_hose(f, m, c):
    parts = []
    loop = [(BODY_X1 + 240 + 160 * math.cos(a / 5), -HY - 210, 840 + 150 * math.sin(a / 5)) for a in range(0, 32)]
    parts.append(g.tube(f, 'hydrant_hose_loop', g.smooth_path(loop, 2), 62, m['hose_red'], c, segments=12))
    path = [(BODY_X1 + 60, -HY + 100, 1050), (BODY_X1 + 120, -HY - 120, 1000), (BODY_X1 + 120, -HY - 180, 760), (BODY_X1 + 300, -HY - 210, 700)]
    parts.append(g.tube(f, 'hydrant_hose_lead', g.smooth_path(path, 6), 62, m['hose_red'], c, segments=14))
    parts.append(g.cylinder(f, 'hydrant_coupler', (BODY_X1 + 100, -HY - 40, 1040), 'y', 80, 140, m['alu'], c, segments=20, bevel=4))
    parts.append(g.cylinder(f, 'hydrant_pit_adapter', (BODY_X1 + 100, -HY - 150, 1040), 'y', 55, 90, m['brass'], c, segments=16, bevel=3))
    return parts


def platform(f, m, c):
    parts = []
    mid = (DECK_X0 + DECK_X1) / 2
    parts.append(g.box(f, 'platform_deck', DECK_X0, DECK_X1, -HY + 120, HY - 120, DECK_Z - 40, DECK_Z, m['deck'], c, bevel=3))
    for i in range(12):
        parts.append(g.box(f, f'platform_tread_{i}', DECK_X0 + 40 + i * 110, DECK_X0 + 70 + i * 110, -HY + 160, HY - 160, DECK_Z, DECK_Z + 6, m['alu'], c))
    rail_z = (DECK_Z + 550, DECK_Z + 1100)
    for k in (-1, 1):
        y = k * (HY - 130)
        for xp in (DECK_X0 + 30, DECK_X1 - 30, mid):
            parts.append(g.cylinder(f, f'platform_post_{k}_{int(xp)}', (xp, y, DECK_Z + 550), 'z', 22, 1100, m['yellow'], c, segments=12))
        for z in rail_z:
            parts.append(g.cylinder(f, f'platform_rail_{k}_{int(z)}', (mid, y, z), 'x', 20, DECK_X1 - DECK_X0, m['yellow'], c, segments=12))
        parts.append(g.box(f, f'platform_toe_{k}', DECK_X0, DECK_X1, y - 5, y + 5, DECK_Z, DECK_Z + 120, m['yellow'], c))
    for nm, z in (('a', rail_z[1]), ('b', rail_z[0])):
        parts.append(g.cylinder(f, f'platform_rear_rail_{nm}', (DECK_X0 + 30, 0, z), 'y', 20, W - 260, m['yellow'], c, segments=12))
    parts.append(g.box(f, 'platform_front_panel', DECK_X1 - 30, DECK_X1, -HY + 150, HY - 150, DECK_Z, DECK_Z + 1100, m['body'], c, bevel=4))
    parts.append(g.box(f, 'platform_mast_inner', DECK_X1 - 220, DECK_X1 - 40, -150, 150, DECK_Z - 1300, DECK_Z - 40, m['alu'], c, bevel=4))
    parts.append(g.box(f, 'platform_support_beam', DECK_X0 + 100, DECK_X1 - 40, -80, 80, DECK_Z - 160, DECK_Z - 40, m['frame'], c, bevel=3))
    parts.append(g.box(f, 'platform_control_box', DECK_X1 - 140, DECK_X1 - 40, 400, 600, DECK_Z + 900, DECK_Z + 1100, m['black'], c, bevel=5))
    for i, col in enumerate(('red', 'yellow')):
        parts.append(g.cylinder(f, f'platform_ctrl_btn_{col}', (DECK_X1 - 145, 450 + i * 100, DECK_Z + 1000), 'x', 24, 14, m[col], c, segments=16))
    for o in parts:
        g.set_anim(o, PLATFORM_ANIM)
    return parts


def lift_fixed(f, m, c):
    parts = []
    parts.append(g.box(f, 'mast_outer', DECK_X1 - 260, DECK_X1, -190, 190, 1010, DECK_Z - 700, m['alu'], c, bevel=5))
    parts.append(g.box(f, 'mast_base', DECK_X1 - 340, DECK_X1 + 80, -300, 300, 960, 1060, m['frame'], c, bevel=5))
    parts.append(g.box(f, 'lift_cylinder_cover', DECK_X1 - 120, DECK_X1 - 40, -90, 90, 1060, DECK_Z - 900, m['steel'], c, bevel=3))
    for i in range(5):
        parts.append(g.cylinder(f, f'access_step_{i}', (260, 0, 620 + i * 130), 'y', 17, 700, m['alu'], c, segments=10))
    for k in (-1, 1):
        parts.append(g.box(f, f'access_step_stringer_{k}', 240, 280, k * 350 - 15, k * 350 + 15, 520, 1260, m['alu'], c))
    parts.append(g.box(f, 'under_ride_guard', 40, 120, -HY + 150, HY - 150, 520, 600, m['frame'], c, bevel=3))
    return parts


def hose_to_wing(f, m, c):
    parts = []
    base = (DECK_X1 - 120, 0.0, DECK_Z + 140)
    top = (DECK_X1 + 400, 850.0, DECK_Z + 140)
    path = [base, (base[0] + 120, 250.0, DECK_Z + 90), (base[0] + 380, 540.0, DECK_Z + 60), (top[0], top[1], DECK_Z + 160), (top[0] - 90, top[1] + 220, DECK_Z + 170)]
    parts.append(g.tube(f, 'wing_hose_a', g.smooth_path(path, 8), 40, m['hose'], c, segments=14))
    cy = top[1] + 220
    parts.append(g.lathe(f, 'wing_coupler_body', (top[0] - 90, cy, DECK_Z + 170), 'y', [(0.0, 0.0), (36.0, 0.0), (52.0, 40.0), (52.0, 200.0), (66.0, 220.0), (66.0, 260.0), (50.0, 280.0), (0.0, 280.0)], m['alu'], c, segments=24))
    parts.append(g.cylinder(f, 'wing_coupler_collar', (top[0] - 90, cy + 300, DECK_Z + 170), 'y', 74, 36, m['yellow'], c, segments=24, bevel=4))
    parts.append(g.box(f, 'wing_coupler_handle', top[0] - 180, top[0] - 20, cy + 110, cy + 130, DECK_Z + 220, DECK_Z + 320, m['black'], c, bevel=3))
    parts.append(g.cylinder(f, 'wing_coupler_dust_cap', (top[0] - 90, cy + 325, DECK_Z + 170), 'y', 56, 18, m['red'], c, segments=20, bevel=3))
    for o in parts:
        g.set_anim(o, HOSE_ANIM)
    return parts


def decals(f, c, operator=DEFAULT_OPERATOR):
    mat = g.image_mat('hyd_livery_' + operator.lower().replace(' ', '_'), livery_file(operator), roughness=0.4)
    x0, x1, z0, z1 = BODY_X0 + 60, BODY_X0 + 3260, 1560, 2420
    parts = []
    for k in (-1, 1):
        y = k * (HY - 80) + k * 8
        corners = [(x0, y, z0), (x1, y, z0), (x1, y, z1), (x0, y, z1)] if k > 0 else [(x1, y, z0), (x0, y, z0), (x0, y, z1), (x1, y, z1)]
        parts.append(g.decal(f, f'livery_side_{k}', corners, mat, c))
    return parts


def stripes(f, m, c):
    parts = []
    for k in (-1, 1):
        ya, yb = sorted((k * (HY - 80) + k * 2, k * (HY - 80) + k * 4))
        parts.append(g.box(f, f'chevron_band_{k}', BODY_X0, BODY_X1, ya, yb, 1010, 1100, m['stripe'], c))
    for i in range(8):
        parts.append(g.box(f, f'rear_chevron_{i}', -66, -62, -HY + 110 + i * 285, -HY + 110 + i * 285 + 140, 960, 1100, m['stripe'] if i % 2 else m['yellow'], c))
    return parts


def build(with_connection=True):
    g.reset_scene()
    f = g.Frame(X0)
    c = g.collection(NAME)
    m = materials()
    if not os.path.exists(livery_file(DEFAULT_OPERATOR)):
        liveries()
    chassis(f, m, c)
    cab(f, m, c)
    body(f, m, c)
    hydrant_hose(f, m, c)
    lift_fixed(f, m, c)
    platform(f, m, c)
    hose_to_wing(f, m, c)
    stripes(f, m, c)
    decals(f, c)
    return c
