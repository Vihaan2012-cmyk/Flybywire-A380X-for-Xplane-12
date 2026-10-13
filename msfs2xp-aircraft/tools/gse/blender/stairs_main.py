import math
import os

import bpy

import gselib as g

NAME = 'stairs_main'
OUT_DIR = 'E:/fbw-gse/out/stairs_main'
LIVERY_DIR = OUT_DIR + '/liveries'
X0 = 3000.0
CHASSIS_L = 7000.0
CHASSIS_W = 2500.0
HY = CHASSIS_W / 2
RAIL_Z0 = 520.0
RAIL_Z1 = 760.0
TYRE_D = 1000.0
TYRE_W = 295.0
AXLES = (1300.0, 2600.0, 5700.0)
CAB_X0 = 5200.0
CAB_X1 = 7000.0
CAB_Z1 = 2650.0
SILL_Z = 5200.0
STAIR_FOOT_X = -2500.0
STAIR_TOP_X = 5000.0
STAIR_FOOT_Z = 330.0
STAIR_WIDTH = 1200.0
STAIR_COUNT = 27
PLATFORM_X0 = 5000.0
PLATFORM_X1 = 6500.0
PLATFORM_HY = 1100.0
PLATFORM_T = 90.0
LIFT_ANIM = [{'kind': 'trans', 'offset0': (0.0, 0.0, 0.0), 'offset1': (0.0, 0.0, 0.4), 'v0': 0.0, 'v1': 1.0, 'dataref': 'fbw/gse/stairs_main/lift'}]
PLATFORM_ANIM = [{'kind': 'trans', 'offset0': (0.0, 0.0, 0.0), 'offset1': (0.0, 0.35, 0.0), 'v0': 0.0, 'v1': 1.0, 'dataref': 'fbw/gse/stairs_main/platform'}]
OPERATORS = {
    'dnata': ((0, 70, 140), (240, 240, 240), 'dnata'),
    'swissport': ((200, 20, 40), (255, 255, 255), 'SWISSPORT'),
    'menzies': ((20, 70, 40), (255, 205, 0), 'MENZIES'),
    'sats': ((230, 120, 0), (20, 30, 60), 'SATS'),
}
DEFAULT_OPERATOR = 'dnata'

VIEWS = {
    'three_quarter_front': dict(res=(800, 560), lens=35, samples=16, loc=(-9.5, 4.5, 4.8), target=(0.0, 0.8, 2.0)),
    'three_quarter_rear': dict(res=(800, 560), lens=35, samples=16, loc=(9.0, -6.5, 4.8), target=(0.0, 0.0, 2.2)),
    'side_right': dict(res=(800, 400), lens=35, samples=16, loc=(14.0, 0.5, 2.8), target=(0.0, 0.5, 2.6)),
    'platform_close': dict(res=(800, 560), lens=35, samples=16, loc=(-5.5, 8.0, 6.6), target=(0.0, 3.2, 5.0)),
}


def materials():
    return {
        'white': g.mat('gse_white', (236, 237, 233), roughness=0.32),
        'body': g.mat('gse_body_grey', (205, 208, 210), roughness=0.4),
        'galv': g.mat('gse_galvanized', (168, 171, 172), metallic=0.85, roughness=0.42),
        'black': g.mat('gse_black', (22, 22, 23), roughness=0.55),
        'rubber': g.mat('gse_rubber', (28, 28, 28), roughness=0.88),
        'rim': g.mat('gse_rim', (150, 152, 152), metallic=0.8, roughness=0.4),
        'amber': g.mat('gse_amber_lens', (255, 140, 10), roughness=0.12),
        'red': g.mat('gse_red', (185, 18, 20), roughness=0.3),
        'yellow': g.mat('gse_yellow', (245, 200, 0), roughness=0.35),
        'glass': g.mat('gse_glass', (70, 90, 100), roughness=0.05, alpha=0.35),
        'tread': g.mat('gse_tread_plate', (96, 98, 100), metallic=0.7, roughness=0.5),
        'rail': g.mat('gse_handrail', (235, 235, 235), roughness=0.35),
        'bumper': g.mat('gse_bumper', (30, 30, 32), roughness=0.9),
        'canopy': g.mat('gse_canopy', (225, 227, 228), roughness=0.35),
    }


def side_profile_strip(x0, z0, x1, z1, depth):
    dx, dz = x1 - x0, z1 - z0
    n = math.hypot(dx, dz)
    nx, nz = -dz / n * depth, dx / n * depth
    return [(x0, z0), (x1, z1), (x1 + nx, z1 + nz), (x0 + nx, z0 + nz)]


def chassis(f, m, c):
    parts = []
    for k in (-1, 1):
        parts.append(g.box(f, f'frame_rail_{k}', 0, CHASSIS_L, k * 520 - 90, k * 520 + 90, RAIL_Z0, RAIL_Z1, m['galv'], c, bevel=12))
    for i, x in enumerate((300, 1900, 3300, 4300, 5000, 6600)):
        parts.append(g.box(f, f'cross_member_{i}', x - 60, x + 60, -520, 520, RAIL_Z0, RAIL_Z1 - 40, m['galv'], c, bevel=8))
    parts.append(g.box(f, 'body_deck', 0, CAB_X0 - 100, -HY, HY, 760, 860, m['body'], c, bevel=10))
    for k in (-1, 1):
        parts.append(g.box(f, f'body_skirt_{k}', 150, CAB_X0 - 150, k * (HY - 20) - 10, k * (HY - 20) + 10, 600, 860, m['body'], c, bevel=6))
        parts.append(g.box(f, f'stabiliser_leg_{k}', 3000, 3200, k * 1450 - 60, k * 1450 + 60, 100, 860, m['yellow'], c, bevel=8))
        parts.append(g.box(f, f'stabiliser_foot_{k}', 2900, 3300, k * 1450 - 200, k * 1450 + 200, 40, 100, m['galv'], c, bevel=8))
        parts.append(g.box(f, f'stabiliser_arm_{k}', 3000, 3200, min(k * 1450, k * 520), max(k * 1450, k * 520), 700, 820, m['yellow'], c, bevel=8))
    parts.append(g.box(f, 'rear_bumper', -140, 0, -HY + 100, HY - 100, 520, 760, m['black'], c, bevel=8))
    for k in (-1, 1):
        parts.append(g.box(f, f'tail_lamp_{k}', -146, -138, k * 1000 - 70, k * 1000 + 70, 640, 740, m['red'], c, bevel=2))
        parts.append(g.box(f, f'rear_reflector_{k}', -146, -140, k * 700 - 60, k * 700 + 60, 540, 600, m['red'], c))
    return parts


def wheel(f, m, c, name, x, y, k):
    r = TYRE_D / 2
    tyre_profile = [(300.0, -TYRE_W / 2 + 20), (r - 60, -TYRE_W / 2), (r - 14, -TYRE_W / 2 + 14), (r, -TYRE_W / 2 + 60), (r, TYRE_W / 2 - 60),
                    (r - 14, TYRE_W / 2 - 14), (r - 60, TYRE_W / 2), (300.0, TYRE_W / 2 - 20)]
    parts = [g.lathe(f, f'tyre_{name}', (x, y, r), (k, 0, 0), tyre_profile, m['rubber'], c, segments=56)]
    parts.append(g.lathe(f, f'rim_{name}', (x, y, r), (k, 0, 0), [(0.0, 50.0), (110.0, 50.0), (150.0, 30.0), (290.0, 30.0), (300.0, 90.0), (300.0, 120.0)], m['rim'], c, segments=48))
    for b in range(8):
        a = 2 * math.pi * b / 8
        parts.append(g.cylinder(f, f'lug_{name}_{b}', (x + 90 * math.cos(a), y + k * 70, r + 90 * math.sin(a)), 'y', 14, 24, m['galv'], c, segments=10))
    return parts


def wheels(f, m, c):
    parts = []
    for ai, x in enumerate(AXLES):
        front = ai == 2
        for k in (-1, 1):
            offsets = (0.0,) if front else (-1.0, 1.0)
            for p in offsets:
                y = k * 1000.0 + p * (TYRE_W / 2 + 10) * 0.0 + (k * 0.0)
                y = k * (1000.0 + (0.0 if front else 0.0))
                name = f'a{ai}_{"L" if k < 0 else "R"}{int(p)}'
                if p != 0.0:
                    y = k * (1000.0 + (TYRE_W + 20) * (0.5 if p * k > 0 else -0.5))
                parts += wheel(f, m, c, name, x, y, k)
        parts.append(g.cylinder(f, f'axle_{ai}', (x, 0, TYRE_D / 2), 'y', 80, 2000, m['galv'], c, segments=20))
        for k in (-1, 1):
            parts.append(g.prism_xz(f, f'mudguard_{ai}_{k}', [(x - 560, 1060), (x - 420, 1190), (x, 1280), (x + 420, 1190), (x + 560, 1060), (x + 500, 1060), (x + 380, 1140), (x, 1220), (x - 380, 1140), (x - 500, 1060)],
                                    k * 1000 - 200, k * 1000 + 200, m['black'], c, bevel=3))
    return parts


def cab(f, m, c):
    parts = []
    profile = [(CAB_X0, 800), (CAB_X0, 2600), (CAB_X0 + 60, CAB_Z1), (6150, CAB_Z1), (6500, 2450), (6900, 2000), (CAB_X1, 1800), (CAB_X1, 900), (CAB_X1 - 60, 800)]
    body = g.prism_xz(f, 'cab_shell', profile, -1000, 1000, m['white'], c)
    g.bevel_where(body, lambda pt, d: abs(d.x) < 0.2 or abs(d.z) < 0.2 or abs(d.y) > 0.5, 45, 4)
    for poly in body.data.polygons:
        poly.use_smooth = True
    parts.append(body)
    parts.append(g.prism_xz(f, 'windscreen', [(6155, 2440), (6490, 2360), (6820, 2020), (6760, 1980), (6470, 2300), (6160, 2380)], -880, 880, m['glass'], c, bevel=2))
    for k in (-1, 1):
        y0, y1 = sorted((k * 1001, k * 1001 + k * 4))
        parts.append(g.box(f, f'door_window_{k}', 5500, 6180, y0, y1, 1850, 2400, m['glass'], c, bevel=3))
        parts.append(g.box(f, f'door_gap_{k}', 5250, 5254, y0, y1 + 3, 900, 2500, m['black'], c))
        hy0, hy1 = sorted((k * 1001, k * 1001 + k * 40))
        parts.append(g.box(f, f'door_handle_{k}', 5600, 5760, hy0, hy1, 1650, 1690, m['black'], c, bevel=3))
        ay0, ay1 = sorted((k * 1020, k * 1250))
        parts.append(g.box(f, f'mirror_arm_{k}', 6100, 6140, ay0, ay1, 2150, 2190, m['black'], c, bevel=3))
        my0, my1 = sorted((k * 1250, k * 1290))
        parts.append(g.box(f, f'mirror_{k}', 6060, 6180, my0, my1, 1950, 2400, m['black'], c, bevel=8))
        parts.append(g.box(f, f'headlamp_{k}', 6930, 7010, k * 800 - 130, k * 800 + 130, 1250, 1400, m['white'], c, bevel=6))
        parts.append(g.box(f, f'indicator_{k}', 6925, 7005, k * 800 - 130, k * 800 + 130, 1130, 1210, m['amber'], c, bevel=4))
        s0, s1 = sorted((k * 1001, k * 1001 + k * 190))
        parts.append(g.box(f, f'step_{k}', 5500, 5950, s0, s1, 560, 620, m['tread'], c, bevel=3))
        s0, s1 = sorted((k * 1001, k * 1001 + k * 160))
        parts.append(g.box(f, f'step_upper_{k}', 5500, 5950, s0, s1, 1060, 1110, m['tread'], c, bevel=3))
    parts.append(g.box(f, 'front_grille', 7000, 7012, -600, 600, 1000, 1700, m['black'], c, bevel=4))
    for i in range(7):
        parts.append(g.box(f, f'grille_slat_{i}', 7010, 7022, -560, 560, 1060 + i * 90, 1100 + i * 90, m['galv'], c, bevel=2))
    parts.append(g.box(f, 'front_bumper', 6980, 7090, -1000, 1000, 700, 960, m['black'], c, bevel=12))
    parts.append(g.box(f, 'number_plate', 7092, 7096, -260, 260, 760, 900, m['yellow'], c))
    parts.append(g.box(f, 'beacon_base', 5900, 6050, -80, 80, CAB_Z1, CAB_Z1 + 30, m['black'], c, bevel=3))
    parts.append(g.lathe(f, 'beacon_lens', (5975, 0, CAB_Z1 + 30), 'z', [(0.0, 130.0), (60.0, 120.0), (85.0, 60.0), (85.0, 0.0)], m['amber'], c, segments=24))
    return parts


def stair_steps():
    run = (STAIR_TOP_X - STAIR_FOOT_X) / STAIR_COUNT
    rise = (SILL_Z - STAIR_FOOT_Z) / STAIR_COUNT
    return run, rise


def stair_flight(f, m, c):
    parts = []
    run, rise = stair_steps()
    for k in (-1, 1):
        y = k * (STAIR_WIDTH / 2 + 40)
        strip = side_profile_strip(STAIR_FOOT_X, STAIR_FOOT_Z - 40, STAIR_TOP_X, SILL_Z - 40, -420)
        parts.append(g.prism_xz(f, f'stringer_{k}', strip, y - 40, y + 40, m['white'], c, bevel=6))
        flange = side_profile_strip(STAIR_FOOT_X, STAIR_FOOT_Z - 40, STAIR_TOP_X, SILL_Z - 40, 40)
        parts.append(g.prism_xz(f, f'stringer_cap_{k}', flange, y - 70, y + 70, m['galv'], c, bevel=4))
    for i in range(STAIR_COUNT):
        x = STAIR_FOOT_X + i * run
        z = STAIR_FOOT_Z + (i + 1) * rise
        parts.append(g.box(f, f'tread_{i}', x, x + run + 15, -STAIR_WIDTH / 2, STAIR_WIDTH / 2, z - 25, z, m['tread'], c, bevel=3, segments=1))
        parts.append(g.box(f, f'nosing_{i}', x + run - 25, x + run + 15, -STAIR_WIDTH / 2, STAIR_WIDTH / 2, z - 30, z + 3, m['yellow'], c, bevel=2, segments=1))
        parts.append(g.box(f, f'riser_{i}', x + run - 4, x + run + 2, -STAIR_WIDTH / 2, STAIR_WIDTH / 2, z - rise, z - 25, m['white'], c, segments=1))
    for k in (-1, 1):
        y = k * (STAIR_WIDTH / 2 + 80)

        def p(t, h):
            return (STAIR_FOOT_X + (STAIR_TOP_X - STAIR_FOOT_X) * t, y, STAIR_FOOT_Z + (SILL_Z - STAIR_FOOT_Z) * t + h)

        parts.append(g.tube(f, f'handrail_{k}', [p(0.0, 900), p(1.0, 900)], 22, m['rail'], c, segments=14))
        parts.append(g.tube(f, f'midrail_{k}', [p(0.0, 480), p(1.0, 480)], 15, m['rail'], c, segments=10))
        for i in range(0, STAIR_COUNT + 1, 3):
            parts.append(g.cylinder(f, f'rail_post_{k}_{i}', p(i / STAIR_COUNT, 450), 'z', 18, 900, m['rail'], c, segments=10))
        parts.append(g.box(f, f'foot_kickplate_{k}', STAIR_FOOT_X - 100, STAIR_FOOT_X + 300, y - 20, y + 20, 100, 330, m['black'], c, bevel=3))
    parts.append(g.box(f, 'foot_plate', STAIR_FOOT_X - 400, STAIR_FOOT_X + 40, -STAIR_WIDTH / 2 - 40, STAIR_WIDTH / 2 + 40, 160, 330, m['tread'], c, bevel=5))
    for k in (-1, 1):
        parts.append(g.cylinder(f, f'foot_wheel_{k}', (STAIR_FOOT_X - 140, k * 520, 120), 'y', 120, 90, m['rubber'], c, segments=24, bevel=4))
    return parts


def support_structure(f, m, c):
    parts = []
    for k in (-1, 1):
        y = k * 560
        parts.append(g.tube(f, f'lift_leg_{k}', [(STAIR_TOP_X - 400, y, 860), (STAIR_TOP_X - 400, y, SILL_Z - 360)], 70, m['galv'], c, segments=14))
        parts.append(g.tube(f, f'diag_brace_{k}', [(STAIR_TOP_X - 2200, y, 860), (STAIR_TOP_X - 450, y, SILL_Z - 900)], 45, m['galv'], c, segments=12))
        parts.append(g.tube(f, f'diag_brace_low_{k}', [(STAIR_TOP_X - 3800, y, 860), (STAIR_TOP_X - 1700, y, 2650)], 40, m['galv'], c, segments=12))
        parts.append(g.box(f, f'leg_foot_{k}', STAIR_TOP_X - 600, STAIR_TOP_X - 200, y - 150, y + 150, 860, 900, m['galv'], c, bevel=4))
    parts.append(g.box(f, 'under_stair_box', STAIR_TOP_X - 3200, STAIR_TOP_X - 1200, -650, 650, 900, 1700, m['body'], c, bevel=10))
    parts.append(g.box(f, 'control_door', STAIR_TOP_X - 3210, STAIR_TOP_X - 3190, -420, 420, 1000, 1600, m['white'], c, bevel=4))
    for i in range(3):
        parts.append(g.cylinder(f, f'ctrl_button_{i}', (STAIR_TOP_X - 3215, -200 + i * 200, 1450), 'x', 25, 14, m['red' if i == 0 else 'yellow'], c, segments=14, bevel=2))
    return parts


def platform(f, m, c):
    parts = []
    z0 = SILL_Z - PLATFORM_T
    parts.append(g.box(f, 'platform_deck', PLATFORM_X0, PLATFORM_X1, -PLATFORM_HY, PLATFORM_HY, z0, SILL_Z, m['tread'], c, bevel=8))
    parts.append(g.box(f, 'platform_frame', PLATFORM_X0, PLATFORM_X1, -PLATFORM_HY, PLATFORM_HY, z0 - 120, z0, m['white'], c, bevel=8))
    for k in (-1, 1):
        parts.append(g.box(f, f'platform_kick_{k}', PLATFORM_X0, PLATFORM_X1, k * PLATFORM_HY - 15, k * PLATFORM_HY + 15, SILL_Z, SILL_Z + 120, m['white'], c, bevel=3))
        parts.append(g.tube(f, f'platform_rail_{k}', [(PLATFORM_X0 + 40, k * (PLATFORM_HY - 30), SILL_Z + 1050), (PLATFORM_X1 - 40, k * (PLATFORM_HY - 30), SILL_Z + 1050)], 22, m['rail'], c, segments=14))
        parts.append(g.tube(f, f'platform_midrail_{k}', [(PLATFORM_X0 + 40, k * (PLATFORM_HY - 30), SILL_Z + 560), (PLATFORM_X1 - 40, k * (PLATFORM_HY - 30), SILL_Z + 560)], 14, m['rail'], c, segments=10))
        for x in (PLATFORM_X0 + 60, 5700.0, PLATFORM_X1 - 60):
            parts.append(g.cylinder(f, f'platform_post_{k}_{int(x)}', (x, k * (PLATFORM_HY - 30), SILL_Z + 560), 'z', 20, 1120, m['rail'], c, segments=10))
    return parts


def canopy(f, m, c):
    parts = []
    roof_z = SILL_Z + 2250
    profile = [(STAIR_TOP_X - 2400, roof_z - 120), (PLATFORM_X1, roof_z), (PLATFORM_X1, roof_z + 80), (STAIR_TOP_X - 2400, roof_z - 40)]
    parts.append(g.prism_xz(f, 'canopy_roof', profile, -PLATFORM_HY - 80, PLATFORM_HY + 80, m['canopy'], c, bevel=12))
    for k in (-1, 1):
        y = k * (PLATFORM_HY - 30)
        parts.append(g.tube(f, f'canopy_post_front_{k}', [(PLATFORM_X1 - 60, y, SILL_Z + 1050), (PLATFORM_X1 - 60, y, roof_z)], 35, m['white'], c, segments=12))
        parts.append(g.tube(f, f'canopy_post_back_{k}', [(PLATFORM_X0 + 60, y, SILL_Z + 1050), (PLATFORM_X0 + 60, y, roof_z - 100)], 35, m['white'], c, segments=12))
        for i, back in enumerate((STAIR_TOP_X - 1400, STAIR_TOP_X - 2400)):
            t = (back - STAIR_FOOT_X) / (STAIR_TOP_X - STAIR_FOOT_X)
            zb = STAIR_FOOT_Z + (SILL_Z - STAIR_FOOT_Z) * t + 900
            ztop = roof_z - 120 + 120 * (back - (STAIR_TOP_X - 2400)) / 2400
            parts.append(g.tube(f, f'canopy_stair_post_{k}_{i}', [(back, k * (STAIR_WIDTH / 2 + 80), zb), (back, k * (STAIR_WIDTH / 2 + 80), ztop)], 30, m['white'], c, segments=12))
        wy0, wy1 = sorted((y - 4, y + 4))
        parts.append(g.box(f, f'canopy_window_{k}', PLATFORM_X0 + 80, PLATFORM_X1 - 80, wy0, wy1, SILL_Z + 1080, roof_z - 200, m['glass'], c, bevel=2))
        parts.append(g.box(f, f'canopy_light_{k}', PLATFORM_X1 - 150, PLATFORM_X1 - 50, k * 800 - 100, k * 800 + 100, roof_z - 20, roof_z - 4, m['white'], c, bevel=2))
    parts.append(g.box(f, 'canopy_front_header', PLATFORM_X1 - 80, PLATFORM_X1, -PLATFORM_HY, PLATFORM_HY, roof_z - 280, roof_z, m['white'], c, bevel=6))
    parts.append(g.box(f, 'canopy_beacon_base', 5500, 5640, -70, 70, roof_z + 80, roof_z + 110, m['black'], c, bevel=3))
    parts.append(g.lathe(f, 'canopy_beacon', (5570, 0, roof_z + 110), 'z', [(0.0, 120.0), (55.0, 110.0), (80.0, 55.0), (80.0, 0.0)], m['amber'], c, segments=24))
    return parts


def bumper(f, m, c):
    parts = []
    parts.append(g.box(f, 'bumper_slide', PLATFORM_X1 - 400, PLATFORM_X1 + 100, -PLATFORM_HY + 80, PLATFORM_HY - 80, SILL_Z - 130, SILL_Z - 10, m['galv'], c, bevel=6))
    parts.append(g.box(f, 'bumper_pad', PLATFORM_X1 + 100, PLATFORM_X1 + 220, -PLATFORM_HY + 60, PLATFORM_HY - 60, SILL_Z - 60, SILL_Z + 1100, m['bumper'], c, bevel=25))
    parts.append(g.box(f, 'bumper_bottom_bar', PLATFORM_X1 + 100, PLATFORM_X1 + 160, -PLATFORM_HY + 60, PLATFORM_HY - 60, SILL_Z - 160, SILL_Z - 60, m['bumper'], c, bevel=12))
    return parts


def draw_livery(name, path):
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        import inspect
        import subprocess
        source = inspect.getsource(draw_livery_pil)
        script = chr(10).join(['import os', 'OPERATORS = ' + repr(OPERATORS), source, 'draw_livery_pil(' + repr(name) + ', ' + repr(path) + ')'])
        subprocess.run(['python', '-c', script], check=True)
        return
    draw_livery_pil(name, path)


def draw_livery_pil(name, path):
    from PIL import Image, ImageDraw, ImageFont
    main, accent, label = OPERATORS[name]
    img = Image.new('RGBA', (1024, 256), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    d.rounded_rectangle((0, 0, 1023, 255), 28, fill=main + (255,))
    d.rectangle((0, 196, 1023, 232), fill=accent + (255,))
    try:
        font = ImageFont.truetype('arialbd.ttf', 130)
    except OSError:
        font = ImageFont.load_default()
    d.text((512, 100), label, fill=accent + (255,), font=font, anchor='mm')
    os.makedirs(os.path.dirname(path), exist_ok=True)
    img.save(path)


def livery_path(name=DEFAULT_OPERATOR):
    path = f'{LIVERY_DIR}/{name}.png'
    if not os.path.exists(path):
        draw_livery(name, path)
    return path


def flat_decal(f, c, name, mat, k, x0, x1, y, z0, z1):
    if k > 0:
        corners = [(x0, y, z0), (x1, y, z0), (x1, y, z1), (x0, y, z1)]
    else:
        corners = [(x1, y, z0), (x0, y, z0), (x0, y, z1), (x1, y, z1)]
    return g.decal(f, name, corners, mat, c)


def decals(f, c):
    mat = g.image_mat('decal_operator', livery_path(), roughness=0.4)
    for k in (-1, 1):
        flat_decal(f, c, f'd_operator_side_{k}', mat, k, 300.0, 1800.0, k * (HY + 3), 980.0, 1226.0) if False else None
        flat_decal(f, c, f'd_operator_cab_{k}', mat, k, 5300.0, 6100.0, k * 1003.0, 1100.0, 1330.0)
        flat_decal(f, c, f'd_operator_platform_{k}', mat, k, PLATFORM_X0 + 100, PLATFORM_X1 - 100, k * (PLATFORM_HY + 6), SILL_Z - 120, SILL_Z + 110)
        flat_decal(f, c, f'd_operator_body_{k}', mat, k, 600.0, 2000.0, k * (HY + 3), 600.0, 840.0)


def build(with_connection=True):
    g.reset_scene()
    f = g.Frame(X0)
    c = g.collection(NAME)
    m = materials()
    chassis(f, m, c)
    wheels(f, m, c)
    cab(f, m, c)
    stair_flight(f, m, c)
    support_structure(f, m, c)
    for o in platform(f, m, c) + canopy(f, m, c):
        g.set_anim(o, LIFT_ANIM)
    for o in bumper(f, m, c):
        g.set_anim(o, PLATFORM_ANIM)
    decals(f, c)
    return c


def liveries(out):
    result = {}
    for name in OPERATORS:
        path = os.path.join(out, 'liveries', f'{name}.png')
        draw_livery(name, path)
        result[name] = path
    return result
