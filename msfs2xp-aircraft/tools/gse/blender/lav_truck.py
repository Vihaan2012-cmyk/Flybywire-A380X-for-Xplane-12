import math
import os

import gselib as g

NAME = 'lav_truck'
LIVERY_DIR = 'E:/fbw-gse/out/lav_truck/liveries'
X0 = 2800.0
WIDTH = 2150.0
HY = WIDTH / 2
REAR_AXLE = 1150.0
FRONT_AXLE = 4250.0
TRACK = 1800.0
TYRE_D = 780.0
TYRE_W = 235.0
FRAME_Z0 = 560.0
FRAME_Z1 = 740.0
BODY_X1 = 3500.0
BODY_Z0 = 800.0
CAB_X0 = 3620.0
CAB_X1 = 5200.0
CAB_TOP = 2350.0
PLATFORM_REST_Z = 1250.0
PLATFORM_TRAVEL = 1.15
HOSE_TRAVEL = 2.2
PANEL_HEIGHT = 3400.0
PLATFORM_ANIM = [{'kind': 'trans', 'offset0': (0.0, 0.0, 0.0), 'offset1': (0.0, 0.0, PLATFORM_TRAVEL), 'v0': 0.0, 'v1': 1.0, 'dataref': 'fbw/gse/lav/platform'}]
HOSE_ANIM = [{'kind': 'trans', 'offset0': (0.0, 0.0, 0.0), 'offset1': (0.0, 0.0, HOSE_TRAVEL), 'v0': 0.0, 'v1': 1.0, 'dataref': 'fbw/gse/lav/hose'}]

OPERATORS = {
    'dnata': ('dnata', (214, 40, 40), (255, 255, 255)),
    'swissport': ('SWISSPORT', (20, 70, 150), (255, 255, 255)),
    'menzies': ('MENZIES', (20, 110, 60), (255, 255, 255)),
    'sats': ('SATS', (235, 235, 235), (200, 30, 40)),
}


def materials():
    return {
        'white': g.mat('gse_white', (236, 237, 233), roughness=0.32),
        'cab': g.mat('gse_cab_white', (230, 231, 228), roughness=0.3),
        'galv': g.mat('gse_galvanized', (168, 171, 172), metallic=0.85, roughness=0.42),
        'steel': g.mat('gse_steel', (120, 124, 128), metallic=0.8, roughness=0.5),
        'black': g.mat('gse_black', (22, 22, 23), roughness=0.55),
        'glass': g.mat('gse_glass', (30, 42, 52), roughness=0.06),
        'rubber': g.mat('gse_rubber', (28, 28, 28), roughness=0.88),
        'rim': g.mat('gse_rim', (150, 152, 150), metallic=0.6, roughness=0.45),
        'amber': g.mat('gse_amber_lens', (255, 140, 10), roughness=0.12),
        'red': g.mat('gse_red', (185, 18, 20), roughness=0.3),
        'yellow': g.mat('gse_yellow', (245, 200, 0), roughness=0.35),
        'hose': g.mat('gse_hose', (40, 42, 44), roughness=0.75),
        'tank_blue': g.mat('gse_flush_blue', (30, 90, 170), roughness=0.4),
        'floor': g.mat('gse_checker_plate', (110, 114, 118), metallic=0.7, roughness=0.55),
        'lamp': g.mat('gse_headlamp', (240, 240, 225), roughness=0.1),
    }


def chassis(f, m, c):
    parts = []
    for k in (-1, 1):
        parts.append(g.box(f, f'frame_rail_{k}', 0, 5500, k * 380 - 50, k * 380 + 50, FRAME_Z0, FRAME_Z1, m['black'], c, bevel=6))
    parts.append(g.box(f, 'rear_bumper', -90, 0, -HY + 120, HY - 120, 560, 760, m['steel'], c, bevel=10))
    parts.append(g.box(f, 'rear_underrun', -60, 0, -HY + 200, HY - 200, 430, 560, m['yellow'], c, bevel=6))
    parts.append(g.box(f, 'front_bumper', 5200, 5420, -HY + 60, HY - 60, 520, 800, m['black'], c, bevel=18))
    parts.append(g.cylinder(f, 'rear_axle_beam', (REAR_AXLE, 0, TYRE_D / 2), 'y', 70, TRACK, m['steel'], c, segments=20))
    parts.append(g.cylinder(f, 'front_axle_beam', (FRONT_AXLE, 0, TYRE_D / 2), 'y', 55, TRACK, m['steel'], c, segments=20))
    parts.append(g.cylinder(f, 'driveshaft', (2400, 0, 600), 'x', 40, 2400, m['steel'], c, segments=16))
    parts.append(g.box(f, 'fuel_tank_L', 2000, 2900, -HY + 60, -HY + 360, 420, 740, m['galv'], c, bevel=14))
    parts.append(g.box(f, 'battery_box_R', 2300, 2900, HY - 400, HY - 70, 450, 720, m['black'], c, bevel=10))
    for xo in (REAR_AXLE, FRONT_AXLE):
        for k in (-1, 1):
            parts.append(g.box(f, f'mudguard_{int(xo)}_{k}', xo - 480, xo + 480, k * (HY - 60) - 5, k * (HY - 60) + 5, 850, 870, m['black'], c, bevel=2))
    for xl in (700, 1800, 2900):
        parts.append(g.box(f, f'cross_member_{xl}', xl - 40, xl + 40, -420, 420, FRAME_Z0, FRAME_Z0 + 120, m['black'], c, bevel=4))
    return parts


def wheels(f, m, c):
    parts = []
    r = TYRE_D / 2
    rr = 230.0
    w = TYRE_W
    tyre_profile = [(rr, -w / 2 + 25), (rr + 12, -w / 2 + 5), (r - 60, -w / 2), (r - 15, -w / 2 + 14), (r, -w / 2 + 55),
                    (r, w / 2 - 55), (r - 15, w / 2 - 14), (r - 60, w / 2), (rr + 12, w / 2 - 5), (rr, w / 2 - 25)]
    rim_profile = [(0.0, 60.0), (90.0, 60.0), (100.0, 40.0), (170.0, 34.0), (rr - 2, 30.0), (rr + 6, 80.0), (rr + 6, 95.0)]
    for x in (REAR_AXLE, FRONT_AXLE):
        for k in (-1, 1):
            for dual in ((-1, 1) if x == REAR_AXLE else (0,)):
                y = k * TRACK / 2 + k * dual * 135
                name = f'{"rear" if x == REAR_AXLE else "front"}_{"L" if k < 0 else "R"}{dual}'
                parts.append(g.lathe(f, f'tyre_{name}', (x, y, r), (k, 0, 0), tyre_profile, m['rubber'], c, segments=44))
                parts.append(g.lathe(f, f'rim_{name}', (x, y, r), (k, 0, 0), [(rad, -z) for rad, z in rim_profile], m['rim'], c, segments=36))
                for b in range(8):
                    a = 2 * math.pi * b / 8
                    parts.append(g.cylinder(f, f'lug_{name}_{b}', (x + 90 * math.cos(a), y + k * 70, r + 90 * math.sin(a)), 'y', 10, 16, m['steel'], c, segments=8))
    return parts


def cab(f, m, c):
    parts = []
    profile = [(CAB_X0, 760), (CAB_X0, CAB_TOP), (CAB_X0 + 1050, CAB_TOP), (CAB_X1 - 150, CAB_TOP - 120), (CAB_X1, CAB_TOP - 450),
               (CAB_X1 + 20, 1500), (CAB_X1 + 10, 1000), (CAB_X1 - 20, 760)]
    shell = g.prism_xz(f, 'cab_shell', profile, -HY + 50, HY - 50, m['cab'], c, bevel=22)
    for poly in shell.data.polygons:
        poly.use_smooth = True
    parts.append(shell)
    parts.append(g.prism_xz(f, 'windscreen', [(CAB_X1 - 141, CAB_TOP - 127), (CAB_X1 + 8, CAB_TOP - 450), (CAB_X1 + 28, 1500), (CAB_X1 - 150, 1500)], -HY + 150, HY - 150, m['glass'], c))
    for k in (-1, 1):
        y = k * (HY - 50)
        parts.append(g.box(f, f'side_window_{k}', CAB_X0 + 300, CAB_X1 - 350, y - k * 2, y + k * 4, 1500, 2150, m['glass'], c, bevel=6))
        parts.append(g.box(f, f'door_seam_{k}', CAB_X0 + 1000, CAB_X0 + 1008, y, y + k * 4, 800, 1950, m['black'], c))
        parts.append(g.box(f, f'door_handle_{k}', CAB_X0 + 1150, CAB_X0 + 1300, y, y + k * 30, 1380, 1410, m['black'], c, bevel=4))
        parts.append(g.tube(f, f'mirror_arm_{k}', [(CAB_X1 - 330, y, 1700), (CAB_X1 - 330, y + k * 110, 1750), (CAB_X1 - 330, y + k * 170, 1850)], 12, m['black'], c, segments=10))
        parts.append(g.box(f, f'mirror_{k}', CAB_X1 - 380, CAB_X1 - 340, y + k * 150, y + k * 200, 1780, 2130, m['black'], c, bevel=8))
        parts.append(g.cylinder(f, f'headlamp_{k}', (CAB_X1 + 12, k * 780, 1020), 'x', 85, 26, m['lamp'], c, segments=20, bevel=4))
        parts.append(g.cylinder(f, f'indicator_{k}', (CAB_X1 + 4, k * 780, 840), 'x', 40, 20, m['amber'], c, segments=14))
        parts.append(g.box(f, f'step_{k}', CAB_X0 + 600, CAB_X0 + 1200, y, y + k * 220, 450, 480, m['floor'], c, bevel=3))
    parts.append(g.box(f, 'grille', CAB_X1 + 12, CAB_X1 + 22, -520, 520, 880, 1280, m['black'], c, bevel=6))
    parts.append(g.box(f, 'number_plate', CAB_X1 + 12, CAB_X1 + 18, -260, 260, 560, 710, m['white'], c))
    parts.append(g.box(f, 'beacon_base', CAB_X0 + 500, CAB_X0 + 700, -90, 90, CAB_TOP, CAB_TOP + 25, m['black'], c, bevel=3))
    parts.append(g.lathe(f, 'beacon_lens', (CAB_X0 + 600, 0, CAB_TOP + 25), 'z', [(0.0, 120.0), (60.0, 105.0), (95.0, 70.0), (100.0, 0.0)], m['amber'], c, segments=24))
    return parts


def body(f, m, c):
    parts = []
    parts.append(g.box(f, 'subframe', 80, BODY_X1 + 60, -HY + 70, HY - 70, BODY_Z0 - 60, BODY_Z0, m['black'], c, bevel=6))
    parts.append(g.cylinder(f, 'waste_tank', (1900, 0, 1500), 'x', 600, 3000, m['white'], c, segments=48, bevel=14))
    parts.append(g.box(f, 'flush_tank_front', 3100, BODY_X1, -HY + 80, HY - 80, BODY_Z0, 1900, m['white'], c, bevel=40))
    parts.append(g.box(f, 'tank_cradle_low', 300, 3100, -HY + 80, HY - 80, BODY_Z0, 1150, m['white'], c, bevel=30))
    for k in (-1, 1):
        y = k * (HY - 80)
        parts.append(g.box(f, f'flush_water_band_{k}', 3105, BODY_X1 - 10, y, y + k * 3, 1400, 1700, m['tank_blue'], c))
        parts.append(g.box(f, f'locker_door_{k}', 700, 1700, y, y + k * 6, 860, 1120, m['white'], c, bevel=4))
        parts.append(g.box(f, f'locker_handle_{k}', 1500, 1620, y + k * 6, y + k * 26, 980, 1000, m['black'], c, bevel=3))
        parts.append(g.box(f, f'level_gauge_{k}', 2300, 2340, y, y + k * 14, 1000, 1700, m['glass'], c, bevel=3))
        parts.append(g.box(f, f'ladder_rail_{k}', 150, 160, y, y + k * 40, 900, 1900, m['galv'], c))
    parts.append(g.box(f, 'tank_top_walkway', 400, 3100, -300, 300, 2095, 2110, m['floor'], c, bevel=3))
    parts.append(g.cylinder(f, 'manhole', (1500, 0, 2110), 'z', 160, 60, m['galv'], c, segments=24, bevel=4))
    parts.append(g.cylinder(f, 'vent_stack', (2400, 450, 2110), 'z', 30, 180, m['galv'], c, segments=12))
    parts.append(g.box(f, 'rear_panel', 40, 120, -HY + 120, HY - 120, 800, 1950, m['white'], c, bevel=18))
    parts.append(g.box(f, 'control_box', 20, 60, 330, 750, 1250, 1650, m['black'], c, bevel=6))
    for i in range(4):
        parts.append(g.cylinder(f, f'control_lamp_{i}', (14, 440 + i * 85, 1580), 'x', 20, 10, m['red' if i == 0 else 'yellow'], c, segments=12))
    for i in range(2):
        parts.append(g.cylinder(f, f'control_valve_{i}', (14, 440 + i * 200, 1380), 'x', 45, 30, m['red'], c, segments=16, bevel=4))
    parts.append(g.box(f, 'body_face', BODY_X1, BODY_X1 + 40, -HY + 90, HY - 90, BODY_Z0, 2000, m['white'], c, bevel=15))
    for k in (-1, 1):
        parts.append(g.box(f, f'rear_light_{k}', -20, 40, k * (HY - 160) - 50, k * (HY - 160) + 50, 1000, 1220, m['red'], c, bevel=5))
        parts.append(g.box(f, f'rear_amber_{k}', -20, 40, k * (HY - 160) - 50, k * (HY - 160) + 50, 1260, 1360, m['amber'], c, bevel=5))
        parts.append(g.tube(f, f'handrail_{k}', [(400, k * 320, 2110), (400, k * 320, 2440), (3000, k * 320, 2440), (3000, k * 320, 2110)], 17, m['yellow'], c, segments=10))
    parts.append(g.tube(f, 'handrail_cross', [(400, -320, 2440), (400, 320, 2440)], 17, m['yellow'], c, segments=10))
    return parts


def platform(f, m, c):
    parts = []
    px0, px1, py = -1300.0, -150.0, 600.0
    z = PLATFORM_REST_Z
    for k in (-1, 1):
        parts.append(g.box(f, f'platform_base_{k}', -1420, -80, k * py - 40, k * py + 40, 460, 560, m['steel'], c, bevel=5))
    parts.append(g.box(f, 'platform_pedestal', -1000, -300, -260, 260, 560, z - 70, m['steel'], c, bevel=8))
    anim = []
    anim.append(g.box(f, 'platform_floor', px0, px1, -py, py, z - 70, z, m['floor'], c, bevel=4))
    anim.append(g.box(f, 'platform_toe_front', px0, px0 + 12, -py, py, z, z + 100, m['yellow'], c, bevel=2))
    anim.append(g.box(f, 'platform_toe_left', px0, px1, -py, -py + 12, z, z + 100, m['yellow'], c, bevel=2))
    anim.append(g.box(f, 'platform_toe_right', px0, px1, py - 12, py, z, z + 100, m['yellow'], c, bevel=2))
    rails = [(px0 + 15, -py + 15), (px0 + 15, py - 15), (px1 - 15, py - 15), (px1 - 15, -py + 15)]
    for i, (x, y) in enumerate(rails):
        anim.append(g.cylinder(f, f'platform_post_{i}', (x, y, z + 550), 'z', 16, 1100, m['yellow'], c, segments=10))
    for zr in (z + 540, z + 1050):
        anim.append(g.tube(f, f'platform_rail_{int(zr)}', [(x, y, zr) for x, y in rails] + [(rails[0][0], rails[0][1], zr)], 16, m['yellow'], c, segments=10))
    anim.append(g.box(f, 'platform_control_pod', px1 - 140, px1 - 30, -140, 140, z + 400, z + 760, m['black'], c, bevel=5))
    for k in (-1, 1):
        anim.append(g.tube(f, f'scissor_leg_{k}_a', [(-1250, k * (py - 80), 570), (-250, k * (py - 80), z - 80)], 40, m['steel'], c, segments=10))
        anim.append(g.tube(f, f'scissor_leg_{k}_b', [(-250, k * (py - 80), 570), (-1250, k * (py - 80), z - 80)], 40, m['steel'], c, segments=10))
    anim.append(g.box(f, 'platform_dock_pad', px0 - 60, px0 - 10, -300, 300, z + 20, z + 520, m['rubber'], c, bevel=10))
    for o in anim:
        g.set_anim(o, PLATFORM_ANIM)
    return parts + anim


def hose(f, m, c):
    parts = []
    parts.append(g.box(f, 'hose_tray', 60, 1500, HY - 420, HY - 120, 880, 900, m['galv'], c, bevel=3))
    parts.append(g.cylinder(f, 'hose_reel_drum', (700, HY - 270, 1030), 'y', 130, 240, m['steel'], c, segments=24))
    anim = []
    pts = [(120, HY - 270, 1030), (-100, HY - 270, 1000), (-400, HY - 270, 1040), (-650, HY - 240, 1100), (-900, HY - 200, 1200)]
    anim.append(g.tube(f, 'hose_run', g.smooth_path(pts, 10), 40, m['hose'], c, segments=14))
    for i, xr in enumerate((-150, -330, -520, -710)):
        anim.append(g.cylinder(f, f'hose_clamp_{i}', (xr, HY - 270, 1020 + 40 * i), 'x', 46, 14, m['steel'], c, segments=12))
    anim.append(g.cylinder(f, 'hose_coupling', (-950, HY - 190, 1215), 'x', 62, 120, m['steel'], c, segments=20, bevel=4))
    anim.append(g.cylinder(f, 'hose_cap', (-1030, HY - 190, 1220), 'x', 70, 30, m['yellow'], c, segments=20, bevel=3))
    for o in anim:
        g.set_anim(o, HOSE_ANIM)
    return parts + anim


def extras(f, m, c):
    parts = []
    for k in (-1, 1):
        y = k * (HY - 50)
        for i, xr in enumerate((CAB_X0 + 120, CAB_X0 + 1250)):
            parts.append(g.box(f, f'cab_pillar_{k}_{i}', xr, xr + 40, y, y + k * 6, 1480, 2160, m['black'], c, bevel=3))
        parts.append(g.box(f, f'cab_window_frame_{k}', CAB_X0 + 290, CAB_X1 - 340, y - k * 2, y + k * 2, 2140, 2165, m['black'], c, bevel=2))
        for zb in (1250, 1850):
            parts.append(g.box(f, f'tank_band_{k}_{zb}', 3110, 3490, y + k * 3, y + k * 7, zb, zb + 30, m['steel'], c))
        parts.append(g.box(f, f'wheel_arch_trim_{k}', FRONT_AXLE - 460, FRONT_AXLE + 460, k * (HY - 55), k * (HY - 55) + k * 14, 850, 880, m['black'], c, bevel=3))
        for i in range(3):
            parts.append(g.cylinder(f, f'side_marker_{k}_{i}', (300 + i * 1500, y + k * 8, 880), 'y', 22, 14, m['amber'], c, segments=10))
        parts.append(g.box(f, f'cab_roof_visor_{k}', CAB_X1 - 380, CAB_X1 - 60, y - k * 2, y + k * 4, 2150, 2170, m['black'], c, bevel=3))
        parts.append(g.box(f, f'cab_second_step_{k}', CAB_X0 + 620, CAB_X0 + 1180, y, y + k * 180, 620, 645, m['floor'], c, bevel=3))
    for yo in (-130, 130):
        parts.append(g.cylinder(f, f'reel_flange_{yo}', (700, HY - 270 + yo, 1030), 'y', 175, 14, m['steel'], c, segments=28, bevel=2))
    parts.append(g.cylinder(f, 'reel_crank', (700, HY - 120, 1030), 'y', 20, 80, m['black'], c, segments=10))
    for i, xv in enumerate((900, 1300, 1700, 2100)):
        parts.append(g.cylinder(f, f'drain_valve_{i}', (xv, -HY + 70, 1000), 'y', 38, 40, m['red'], c, segments=14, bevel=3))
        parts.append(g.cylinder(f, f'drain_pipe_{i}', (xv, -HY + 100, 930), 'z', 28, 140, m['galv'], c, segments=10))
    parts.append(g.tube(f, 'water_fill_pipe', [(3200, HY - 80, 1000), (3200, HY - 150, 1000), (3200, HY - 150, 1150)], 24, m['galv'], c, segments=10))
    parts.append(g.cylinder(f, 'water_fill_cap', (3200, HY - 150, 1180), 'z', 45, 40, m['tank_blue'], c, segments=14, bevel=3))
    for i in range(5):
        parts.append(g.box(f, f'ladder_rung_{i}', 130, 175, -HY + 76, -HY + 330, 950 + i * 220, 975 + i * 220, m['galv'], c, bevel=2))
    for k in (-1, 1):
        parts.append(g.cylinder(f, f'tank_bolt_ring_{k}', (3100, k * 500, 1500), 'x', 120, 14, m['steel'], c, segments=20))
    return parts


def livery_decal(f, c, key):
    path = os.path.join(LIVERY_DIR, f'{key}.png')
    if not os.path.exists(path):
        return
    mat = g.image_mat(f'decal_{key}', path, roughness=0.4)
    off = HY - 80 + 8
    for k in (-1, 1):
        y = k * off
        x0, x1, z0, z1 = 1750.0, 3000.0, 1250.0, 1560.0
        corners = [(x0, y, z0), (x1, y, z0), (x1, y, z1), (x0, y, z1)] if k > 0 else [(x1, y, z0), (x0, y, z0), (x0, y, z1), (x1, y, z1)]
        g.decal(f, f'livery_{key}_{k}', corners, mat, c)


def build(with_connection=True):
    g.reset_scene()
    f = g.Frame(X0)
    c = g.collection(NAME)
    m = materials()
    chassis(f, m, c)
    wheels(f, m, c)
    cab(f, m, c)
    body(f, m, c)
    platform(f, m, c)
    hose(f, m, c)
    extras(f, m, c)
    livery_decal(f, c, 'dnata')
    return c


def liveries(out):
    result = {}
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return result
    os.makedirs(os.path.join(out, 'liveries'), exist_ok=True)
    for key, (text, bg, fg) in OPERATORS.items():
        img = Image.new('RGBA', (1024, 256), bg + (255,))
        d = ImageDraw.Draw(img)
        d.rectangle((0, 214, 1024, 256), fill=fg + (255,))
        d.text((60, 70), text, fill=fg + (255,))
        path = os.path.join(out, 'liveries', f'{key}.png')
        img.save(path)
        result[key] = path
    return result


VIEWS = {
    'side_right': dict(res=(800, 460), lens=35, loc=(10.0, 0.0, 1.5), target=(0.0, 0.0, 1.3), samples=16),
    'three_quarter_rear': dict(res=(800, 560), lens=35, loc=(-5.5, -8.0, 3.4), target=(0.0, -0.8, 1.3), samples=16),
    'three_quarter_front': dict(res=(800, 560), lens=35, loc=(-6.0, 8.0, 3.0), target=(0.0, 0.8, 1.3), samples=16),
}
