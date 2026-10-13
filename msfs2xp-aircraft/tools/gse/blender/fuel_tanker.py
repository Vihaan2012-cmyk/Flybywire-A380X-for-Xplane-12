import math
import os

import bpy

import gselib as g

NAME = 'fuel_tanker'
X0 = 4700.0
LIVERY_DIR = 'E:/fbw-gse/out/fuel_tanker/liveries'
DEFAULT_OPERATOR = 'skyfuel'
OPERATORS = {
    'skyfuel': ('SKYFUEL AVIATION', (0, 98, 60), (255, 205, 0)),
    'aeroblue': ('AEROBLUE FUELS', (0, 70, 150), (240, 240, 240)),
    'redline': ('REDLINE AVIATION', (190, 25, 30), (250, 250, 250)),
    'globalplane': ('GLOBALPLANE FUEL', (30, 30, 34), (255, 130, 0)),
}

HALF_W = 1245.0
TANK_X0 = 1300.0
TANK_X1 = 7000.0
TANK_HALF_W = 1200.0
TANK_Z0 = 1100.0
TANK_Z1 = 2900.0
CAB_X0 = 7350.0
FRONT_AXLE = 8350.0
DRIVE_AXLE = 3550.0
TAG_AXLE = 2200.0
TYRE_R = 500.0
TYRE_W = 295.0
DECK_X0, DECK_X1 = 100.0, 1000.0
DECK_Z = 1540.0
DECK_ANIM = [{'kind': 'trans', 'offset0': (0.0, 0.0, 0.0), 'offset1': (0.0, 0.0, 1.5), 'v0': 0.0, 'v1': 1.0, 'dataref': 'fbw/gse/fuel_tanker/deck'}]
HOSE_ANIM = [{'kind': 'trans', 'offset0': (0.0, 0.0, 0.0), 'offset1': (0.0, 0.0, 1.5), 'v0': 0.0, 'v1': 1.0, 'dataref': 'fbw/gse/fuel_tanker/deck'},
             {'kind': 'trans', 'offset0': (0.0, 0.0, 0.0), 'offset1': (0.0, -0.9, -0.55), 'v0': 0.0, 'v1': 1.0, 'dataref': 'fbw/gse/fuel_tanker/hose'}]


def materials():
    return {
        'white': g.mat('gse_white', (236, 237, 233), roughness=0.32),
        'tank': g.mat('gse_tank_white', (226, 228, 228), metallic=0.25, roughness=0.3),
        'alu': g.mat('gse_alu', (176, 179, 182), metallic=0.85, roughness=0.38),
        'steel': g.mat('gse_chassis_steel', (60, 62, 66), metallic=0.6, roughness=0.55),
        'black': g.mat('gse_black', (22, 22, 23), roughness=0.55),
        'glass': g.mat('gse_glass', (24, 32, 38), roughness=0.06),
        'rubber': g.mat('gse_rubber', (28, 28, 28), roughness=0.88),
        'rim': g.mat('gse_rim', (160, 162, 165), metallic=0.8, roughness=0.4),
        'amber': g.mat('gse_amber_lens', (255, 140, 10), roughness=0.12),
        'red': g.mat('gse_red', (185, 18, 20), roughness=0.3),
        'yellow': g.mat('gse_yellow', (245, 200, 0), roughness=0.35),
        'hose': g.mat('gse_fuel_hose', (18, 18, 20), roughness=0.75),
        'grate': g.mat('gse_grate', (92, 95, 98), metallic=0.7, roughness=0.5),
        'lamp': g.mat('gse_headlamp', (235, 238, 240), roughness=0.1),
        'band': g.mat('gse_tank_band', (70, 74, 78), metallic=0.5, roughness=0.5),
        'cab': g.mat('gse_cab_white', (232, 233, 230), roughness=0.28),
        'accent': g.mat('gse_cab_accent', OPERATORS[DEFAULT_OPERATOR][1], roughness=0.3),
    }


def draw_liveries(out=LIVERY_DIR):
    from PIL import Image, ImageDraw, ImageFont
    os.makedirs(out, exist_ok=True)
    font_big = ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf', 190)
    font_small = ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf', 56)
    paths = {}
    for key, (title, main, accent) in OPERATORS.items():
        banner = Image.new('RGBA', (2400, 600), (0, 0, 0, 0))
        d = ImageDraw.Draw(banner)
        d.rectangle((0, 380, 2400, 600), fill=main + (255,))
        d.polygon([(0, 330), (1500, 330), (1400, 380), (0, 380)], fill=accent + (255,))
        d.polygon([(2400, 330), (2000, 330), (1900, 380), (2400, 380)], fill=accent + (255,))
        d.ellipse((80, 40, 330, 290), fill=main + (255,))
        d.polygon([(130, 240), (200, 80), (270, 240), (200, 200)], fill=accent + (255,))
        d.text((400, 60), title, font=font_big, fill=main + (255,))
        d.text((410, 280), 'JET A-1  /  AVIATION FUEL', font=font_small, fill=(40, 40, 40, 255))
        d.text((90, 440), 'FLAMMABLE   NO NAKED LIGHTS   NO SMOKING', font=font_small, fill=accent + (255,))
        path = os.path.join(out, f'{key}_tank.png')
        banner.save(path)
        door = Image.new('RGBA', (700, 420), (0, 0, 0, 0))
        d = ImageDraw.Draw(door)
        d.rounded_rectangle((0, 0, 699, 419), 40, fill=main + (255,))
        d.polygon([(40, 340), (130, 100), (220, 340), (130, 280)], fill=accent + (255,))
        d.text((250, 140), title.split()[0], font=font_small, fill=(255, 255, 255, 255))
        d.text((250, 215), 'AVIATION', font=font_small, fill=accent + (255,))
        door.save(os.path.join(out, f'{key}_door.png'))
        paths[key] = path
    warn = Image.new('RGBA', (400, 400), (0, 0, 0, 0))
    d = ImageDraw.Draw(warn)
    d.polygon([(200, 10), (390, 380), (10, 380)], fill=(250, 200, 0, 255), outline=(0, 0, 0, 255))
    d.rectangle((188, 120, 212, 270), fill=(0, 0, 0, 255))
    d.ellipse((186, 296, 214, 324), fill=(0, 0, 0, 255))
    warn.save(os.path.join(out, 'sign_warning.png'))
    plate = Image.new('RGBA', (900, 300), (0, 0, 0, 0))
    d = ImageDraw.Draw(plate)
    d.rounded_rectangle((0, 0, 899, 299), 20, fill=(255, 140, 0, 255), outline=(0, 0, 0, 255), width=8)
    d.text((40, 20), '1202   JET A-1', font=font_big, fill=(0, 0, 0, 255))
    plate.save(os.path.join(out, 'hazard_plate.png'))
    return paths


def liveries(out):
    return draw_liveries(out)


def decal_mat(name):
    path = os.path.join(LIVERY_DIR, f'{name}.png')
    if not os.path.exists(path):
        return None
    return g.image_mat(f'decal_{name}', path, roughness=0.4)


def superellipse(a, b, zc, n=3.6, count=64):
    pts = []
    for i in range(count):
        t = 2 * math.pi * i / count
        c, s = math.cos(t), math.sin(t)
        pts.append((a * math.copysign(abs(c) ** (2 / n), c), zc + b * math.copysign(abs(s) ** (2 / n), s)))
    return pts


def tank(f, m, c):
    parts = []
    zc = (TANK_Z0 + TANK_Z1) / 2
    b = (TANK_Z1 - TANK_Z0) / 2
    parts.append(g.prism_yz(f, 'tank_shell', superellipse(TANK_HALF_W, b, zc), TANK_X0 + 40, TANK_X1 - 40, m['tank'], c, smooth=True))
    for name, x0, x1 in (('tank_end_rear', TANK_X0, TANK_X0 + 40), ('tank_end_front', TANK_X1 - 40, TANK_X1)):
        parts.append(g.prism_yz(f, name, superellipse(TANK_HALF_W - 30, b - 30, zc), x0, x1, m['tank'], c, smooth=True))
    for i, x in enumerate((2600.0, 3900.0, 5300.0)):
        parts.append(g.prism_yz(f, f'tank_band_{i}', superellipse(TANK_HALF_W + 14, b + 14, zc), x - 30, x + 30, m['band'], c, smooth=True))
    for i, x in enumerate((1380.0, 6920.0)):
        parts.append(g.prism_yz(f, f'tank_head_band_{i}', superellipse(TANK_HALF_W + 10, b + 10, zc), x - 25, x + 25, m['band'], c, smooth=True))
    for i, x in enumerate((1950.0, 3250.0, 4600.0, 5950.0)):
        parts.append(g.cylinder(f, f'manhole_{i}', (x, 380, TANK_Z1 + 35), 'z', 230, 90, m['alu'], c, segments=32, bevel=8))
        parts.append(g.cylinder(f, f'manhole_lid_{i}', (x, 380, TANK_Z1 + 90), 'z', 190, 24, m['black'], c, segments=32, bevel=5))
        for k in range(8):
            a = 2 * math.pi * k / 8
            parts.append(g.cylinder(f, f'manhole_bolt_{i}_{k}', (x + 210 * math.cos(a), 380 + 210 * math.sin(a), TANK_Z1 + 84), 'z', 12, 16, m['alu'], c, segments=8))
        parts.append(g.cylinder(f, f'vent_{i}', (x + 330, 380, TANK_Z1 + 40), 'z', 36, 120, m['alu'], c, segments=14, bevel=3))
    parts.append(g.box(f, 'tank_walkway', 1450, 6850, -720, -330, TANK_Z1 + 8, TANK_Z1 + 28, m['grate'], c, bevel=3))
    for k in range(34):
        x = 1470 + k * 160
        parts.append(g.box(f, f'walkway_rib_{k}', x, x + 22, -722, -328, TANK_Z1 + 28, TANK_Z1 + 38, m['alu'], c))
    for yk in (-760, -300):
        for x in [1500 + i * 900 for i in range(7)]:
            parts.append(g.cylinder(f, f'rail_post_{yk}_{x}', (x, yk, TANK_Z1 + 320), 'z', 17, 600, m['yellow'], c, segments=10))
        parts.append(g.tube(f, f'rail_top_{yk}', [(1500, yk, TANK_Z1 + 600), (6900, yk, TANK_Z1 + 600)], 18, m['yellow'], c, segments=10))
        parts.append(g.tube(f, f'rail_mid_{yk}', [(1500, yk, TANK_Z1 + 300), (6900, yk, TANK_Z1 + 300)], 13, m['yellow'], c, segments=8))
    parts.append(g.cylinder(f, 'tank_beacon_base', (6700, 700, TANK_Z1 + 20), 'z', 70, 40, m['black'], c, segments=20, bevel=3))
    parts.append(g.lathe(f, 'tank_beacon_lens', (6700, 700, TANK_Z1 + 40), 'z', [(0.0, 105.0), (36.0, 98.0), (60.0, 80.0), (62.0, 0.0)], m['amber'], c, segments=24))
    parts.append(g.box(f, 'tank_headboard', TANK_X1, CAB_X0 - 10, -1100, 1100, 1000, TANK_Z1 - 100, m['tank'], c, bevel=20))
    for k in (-1, 1):
        for x in (TANK_X0 + 400, TANK_X1 - 400, 4200):
            parts.append(g.box(f, f'cradle_{k}_{x}', x - 90, x + 90, *sorted((k * 430, k * 1000)), 930, TANK_Z0 + 120, m['steel'], c, bevel=8))
        for x in (TANK_X0 + 200, 4100, TANK_X1 - 200):
            parts.append(g.box(f, f'side_marker_{k}_{int(x)}', x - 50, x + 50, *sorted((k * (TANK_HALF_W + 18), k * (TANK_HALF_W + 24))), 1180, 1230, m['amber'], c))
    return parts


def chassis(f, m, c):
    parts = []
    for k in (-1, 1):
        parts.append(g.box(f, f'frame_rail_{k}', 250, 7400, *sorted((k * 380, k * 440)), 780, 960, m['steel'], c, bevel=5))
    for i, x in enumerate((900, 2900, 4800, 6700)):
        parts.append(g.box(f, f'cross_member_{i}', x - 40, x + 40, -440, 440, 800, 920, m['steel'], c, bevel=4))
    r = TYRE_R
    tyre_profile = [(300.0, -TYRE_W / 2 + 24), (320.0, -TYRE_W / 2 + 6), (r - 70, -TYRE_W / 2), (r - 20, -TYRE_W / 2 + 14), (r, -TYRE_W / 2 + 50),
                    (r, TYRE_W / 2 - 50), (r - 20, TYRE_W / 2 - 14), (r - 70, TYRE_W / 2), (320.0, TYRE_W / 2 - 6), (300.0, TYRE_W / 2 - 24)]
    rim_profile = [(0.0, 60.0), (90.0, 60.0), (100.0, 46.0), (190.0, 38.0), (292.0, 34.0), (306.0, 90.0), (306.0, 105.0)]
    wheel_positions = []
    for k in (-1, 1):
        wheel_positions += [(FRONT_AXLE, k * 1045, 'front', k), (DRIVE_AXLE, k * 805, 'drive_in', k), (DRIVE_AXLE, k * 1145, 'drive_out', k), (TAG_AXLE, k * 1100, 'tag', k)]
    for x, y, tag, k in wheel_positions:
        name = f'{tag}_{"L" if k < 0 else "R"}'
        parts.append(g.lathe(f, f'tyre_{name}', (x, y, r), (k, 0, 0), tyre_profile, m['rubber'], c, segments=44))
        parts.append(g.lathe(f, f'rim_{name}', (x, y, r), (k, 0, 0), [(rad, -z) for rad, z in rim_profile], m['rim'], c, segments=40))
        if tag in ('front', 'drive_out', 'tag'):
            parts.append(g.cylinder(f, f'hub_{name}', (x, y + k * 50, r), 'y', 70, 40, m['alu'], c, segments=24, bevel=5))
            for b in range(10):
                a = 2 * math.pi * b / 10
                parts.append(g.cylinder(f, f'nut_{name}_{b}', (x + 120 * math.cos(a), y + k * 66, r + 120 * math.sin(a)), 'y', 14, 22, m['black'], c, segments=8))
    parts.append(g.cylinder(f, 'axle_front', (FRONT_AXLE, 0, r), 'y', 55, 2 * 1045, m['steel'], c, segments=16))
    parts.append(g.cylinder(f, 'axle_drive', (DRIVE_AXLE, 0, r), 'y', 70, 2 * 805, m['steel'], c, segments=16))
    parts.append(g.lathe(f, 'diff_housing', (DRIVE_AXLE, 0, r), (0, 1, 0), [(0.0, -300.0), (190.0, -230.0), (240.0, -100.0), (240.0, 100.0), (190.0, 230.0), (0.0, 300.0)], m['steel'], c, segments=24))
    parts.append(g.cylinder(f, 'axle_tag', (TAG_AXLE, 0, r), 'y', 50, 2 * 1100, m['steel'], c, segments=16))
    parts.append(g.tube(f, 'prop_shaft', [(4600, 0, 880), (DRIVE_AXLE, 0, 560)], 38, m['steel'], c, segments=12))
    for k in (-1, 1):
        for x in (DRIVE_AXLE, TAG_AXLE):
            inner = [(x + 560 * math.cos(math.radians(a)), 500 + 560 * math.sin(math.radians(a))) for a in range(10, 171, 10)]
            outer = [(x + 585 * math.cos(math.radians(a)), 500 + 585 * math.sin(math.radians(a))) for a in range(170, 9, -10)]
            parts.append(g.prism_xz(f, f'mudguard_{k}_{int(x)}', inner + outer, *sorted((k * 640, k * 1330)), m['black'], c, bevel=2))
    parts.append(g.cylinder(f, 'diesel_tank', (5300, -1050, 790), 'x', 260, 1100, m['alu'], c, segments=28, bevel=6))
    parts.append(g.cylinder(f, 'adblue_tank', (6200, -1050, 760), 'x', 200, 640, m['alu'], c, segments=24, bevel=5))
    parts.append(g.box(f, 'battery_box', 6800, 7200, -1100, -740, 700, 1000, m['black'], c, bevel=8))
    parts.append(g.box(f, 'exhaust_silencer', 5900, 6500, 520, 780, 700, 1010, m['steel'], c, bevel=10))
    return parts


def cab(f, m, c):
    parts = []
    arch = [(FRONT_AXLE + 580 * math.cos(math.radians(a)), 500 + 580 * math.sin(math.radians(a))) for a in range(165, 14, -10)]
    profile = [(CAB_X0, 650.0), (FRONT_AXLE - 560, 650.0)] + arch + [(FRONT_AXLE + 560, 650.0), (9250.0, 650.0), (9400.0, 800.0), (9400.0, 1100.0), (9330.0, 2560.0), (9000.0, 2790.0), (7500.0, 2790.0), (CAB_X0, 2690.0)]
    parts.append(g.prism_xz(f, 'cab_shell', profile, -HALF_W, HALF_W, m['cab'], c, bevel=18))
    parts.append(g.prism_xz(f, 'windscreen', [(9392, 1230), (9341, 2490), (9325, 2490), (9376, 1230)], -1130, 1130, m['glass'], c, bevel=3))
    parts.append(g.prism_xz(f, 'cab_accent_stripe', [(9402, 1000), (9400, 1100), (9392, 1230), (9378, 1180), (9380, 1000)], -HALF_W + 12, HALF_W - 12, m['accent'], c))
    for k in (-1, 1):
        y = k * HALF_W
        parts.append(g.box(f, f'door_glass_{k}', 8680, 9270, *sorted((y - k * 2, y + k * 10)), 900, 2500, m['glass'], c, bevel=8))
        parts.append(g.box(f, f'rear_window_{k}', 7650, 8450, *sorted((y - k * 2, y + k * 10)), 1700, 2500, m['glass'], c, bevel=8))
        for x in (8620, 9300):
            parts.append(g.box(f, f'door_seam_{k}_{x}', x - 4, x + 4, *sorted((y, y + k * 4)), 700, 2600, m['black'], c))
        parts.append(g.box(f, f'door_seam_low_{k}', 8620, 9300, *sorted((y, y + k * 4)), 700, 706, m['black'], c))
        parts.append(g.box(f, f'door_handle_{k}', 8660, 8800, *sorted((y + k * 2, y + k * 28)), 1500, 1540, m['black'], c, bevel=6))
        parts.append(g.box(f, f'step_{k}', 8750, 9250, *sorted((y + k * 2, y + k * 140)), 520, 560, m['alu'], c, bevel=5))
        parts.append(g.box(f, f'step_lower_{k}', 8750, 9250, *sorted((y + k * 2, y + k * 120)), 340, 380, m['alu'], c, bevel=5))
        parts.append(g.box(f, f'headlamp_{k}', 9345, 9420, *sorted((k * 760, k * 1190)), 880, 1020, m['lamp'], c, bevel=18))
        parts.append(g.box(f, f'indicator_{k}', 9345, 9415, *sorted((k * 720, k * 780)), 760, 850, m['amber'], c, bevel=8))
        parts.append(g.box(f, f'mirror_arm_{k}', 9060, 9100, *sorted((y, y + k * 190)), 2320, 2350, m['black'], c, bevel=6))
        parts.append(g.box(f, f'mirror_head_{k}', 9020, 9140, *sorted((y + k * 180, y + k * 230)), 2180, 2640, m['black'], c, bevel=12))
        parts.append(g.box(f, f'mirror_low_{k}', 9040, 9120, *sorted((y + k * 140, y + k * 200)), 1780, 1900, m['black'], c, bevel=8))
    parts.append(g.box(f, 'grille', 9380, 9440, -520, 520, 720, 1010, m['black'], c, bevel=10))
    for i in range(6):
        parts.append(g.box(f, f'grille_bar_{i}', 9430, 9444, -480, 480, 760 + i * 40, 780 + i * 40, m['steel'], c))
    parts.append(g.box(f, 'front_bumper', 9250, 9450, -HALF_W, HALF_W, 450, 700, m['black'], c, bevel=25))
    parts.append(g.box(f, 'bumper_plate', 9448, 9454, -260, 260, 520, 640, m['white'], c))
    parts.append(g.box(f, 'sun_visor', 9080, 9400, -1100, 1100, 2480, 2570, m['black'], c, bevel=12))
    for i in range(2):
        parts.append(g.tube(f, f'wiper_{i}', [(9395, -500 + i * 560, 1330), (9390, -80 + i * 560, 1900)], 9, m['black'], c, segments=8))
    parts.append(g.box(f, 'roof_vent', 7900, 8500, -450, 450, 2790, 2850, m['white'], c, bevel=15))
    parts.append(g.cylinder(f, 'cab_beacon_base', (8200, 0, 2870), 'z', 70, 40, m['black'], c, segments=20, bevel=3))
    parts.append(g.lathe(f, 'cab_beacon_lens', (8200, 0, 2890), 'z', [(0.0, 105.0), (36.0, 98.0), (60.0, 80.0), (62.0, 0.0)], m['amber'], c, segments=24))
    parts.append(g.box(f, 'roof_marker_bar', 9100, 9180, -420, 420, 2790, 2840, m['amber'], c, bevel=5))
    parts.append(g.cylinder(f, 'exhaust_stack', (7480, 1100, 1700), 'z', 55, 1400, m['steel'], c, segments=14))
    return parts


def pump_cabinet(f, m, c):
    parts = []
    parts.append(g.box(f, 'pump_cabinet_frame', 4400, 6800, 1010, 1236, 640, 1090, m['alu'], c, bevel=6))
    for i in range(14):
        x = 4430 + i * 168
        parts.append(g.box(f, f'pump_shutter_{i}', x, x + 154, 1236, 1250, 660, 1070, m['white'], c, bevel=3))
    parts.append(g.box(f, 'pump_handle', 5500, 5700, 1250, 1275, 700, 730, m['black'], c, bevel=4))
    for k in range(4):
        parts.append(g.cylinder(f, f'bottom_load_port_{k}', (4560 + k * 420, 1225, 1180), 'y', 42, 90, m['alu'], c, segments=20, bevel=4))
        parts.append(g.cylinder(f, f'bottom_load_cap_{k}', (4560 + k * 420, 1280, 1180), 'y', 36, 30, m['red'], c, segments=20, bevel=4))
    parts.append(g.box(f, 'meter_panel', 6000, 6700, 1000, 1240, 1110, 1240, m['black'], c, bevel=6))
    parts.append(g.box(f, 'meter_display', 6100, 6350, 1236, 1250, 1130, 1220, m['glass'], c, bevel=2))
    parts.append(g.cylinder(f, 'earth_reel', (4150, 1225, 1050), 'y', 130, 130, m['yellow'], c, segments=24, bevel=4))
    parts.append(g.cylinder(f, 'fire_extinguisher', (6750, -1255, 1100), 'z', 55, 320, m['red'], c, segments=16, bevel=5))
    return parts


def rear_cabinet(f, m, c):
    parts = []
    parts.append(g.box(f, 'cabinet_floor', 0, TANK_X0, -HALF_W, HALF_W, 540, 600, m['alu'], c, bevel=5))
    parts.append(g.box(f, 'cabinet_roof', 0, TANK_X0, -HALF_W, HALF_W, 1500, 1540, m['alu'], c, bevel=5))
    parts.append(g.box(f, 'cabinet_front_wall', TANK_X0 - 30, TANK_X0, -HALF_W, HALF_W, 540, 1540, m['alu'], c, bevel=3))
    for k in (-1, 1):
        parts.append(g.box(f, f'cabinet_side_{k}', 0, TANK_X0, *sorted((k * (HALF_W - 40), k * HALF_W)), 540, 1540, m['white'], c, bevel=5))
        parts.append(g.box(f, f'rear_light_{k}', -30, 0, *sorted((k * 1000, k * 1180)), 640, 1000, m['red'], c, bevel=8))
        parts.append(g.box(f, f'rear_reflector_{k}', -20, 0, *sorted((k * 880, k * 980)), 470, 530, m['red'], c))
    parts.append(g.box(f, 'underrun_bar', -80, -20, -HALF_W, HALF_W, 380, 480, m['yellow'], c, bevel=12))
    for i in range(7):
        parts.append(g.box(f, f'shutter_slat_{i}', -10, 14, -HALF_W + 40, HALF_W - 40, 1300 + i * 28, 1324 + i * 28, m['white'], c, bevel=2))
    for k in (-1, 1):
        yc = k * 620
        profile = [(0.0, -330.0), (430.0, -330.0), (430.0, -300.0), (405.0, -300.0)]
        for j in range(10):
            z = -270 + j * 60
            profile += [(405.0, z), (418.0, z + 30)]
        profile += [(405.0, 300.0), (430.0, 300.0), (430.0, 330.0), (0.0, 330.0)]
        parts.append(g.lathe(f, f'reel_{k}', (700, yc, 1060), (0, 1, 0), profile, m['hose'], c, segments=40))
        parts.append(g.lathe(f, f'reel_core_{k}', (700, yc, 1060), (0, 1, 0), [(0.0, -345.0), (90.0, -345.0), (90.0, 345.0), (0.0, 345.0)], m['alu'], c, segments=20))
        parts.append(g.box(f, f'reel_stand_{k}', 640, 760, *sorted((yc - 360, yc + 360)), 560, 640, m['steel'], c, bevel=4))
        parts.append(g.cylinder(f, f'reel_crank_{k}', (700, yc + k * 345, 1060), 'y', 40, 60, m['yellow'], c, segments=16, bevel=3))
    return parts


def deck(f, m, c):
    parts = []
    parts.append(g.box(f, 'deck_floor', DECK_X0, DECK_X1, -950, 950, DECK_Z, DECK_Z + 50, m['grate'], c, bevel=6))
    for i in range(20):
        x = DECK_X0 + 20 + i * 44
        parts.append(g.box(f, f'deck_rib_{i}', x, x + 14, -940, 940, DECK_Z + 50, DECK_Z + 58, m['alu'], c))
    parts.append(g.box(f, 'deck_toe_board', DECK_X0, DECK_X1, -950, 950, DECK_Z + 50, DECK_Z + 150, m['yellow'], c, bevel=4))
    for k in (-1, 1):
        for x in (DECK_X0 + 40, DECK_X1 - 40):
            parts.append(g.cylinder(f, f'deck_post_{k}_{int(x)}', (x, k * 910, DECK_Z + 600), 'z', 22, 1100, m['yellow'], c, segments=12))
        parts.append(g.tube(f, f'deck_rail_side_{k}', [(DECK_X0 + 40, k * 910, DECK_Z + 1100), (DECK_X1 - 40, k * 910, DECK_Z + 1100)], 20, m['yellow'], c, segments=10))
        parts.append(g.tube(f, f'deck_rail_mid_{k}', [(DECK_X0 + 40, k * 910, DECK_Z + 600), (DECK_X1 - 40, k * 910, DECK_Z + 600)], 14, m['yellow'], c, segments=8))
    parts.append(g.tube(f, 'deck_rail_rear', [(DECK_X0 + 40, -910, DECK_Z + 1100), (DECK_X0 + 40, 910, DECK_Z + 1100)], 20, m['yellow'], c, segments=10))
    parts.append(g.tube(f, 'deck_rail_rear_mid', [(DECK_X0 + 40, -910, DECK_Z + 600), (DECK_X0 + 40, 910, DECK_Z + 600)], 14, m['yellow'], c, segments=8))
    parts.append(g.box(f, 'deck_column_left', DECK_X1 - 140, DECK_X1 - 40, -900, -780, 600, DECK_Z, m['alu'], c, bevel=4))
    parts.append(g.box(f, 'deck_column_right', DECK_X1 - 140, DECK_X1 - 40, 780, 900, 600, DECK_Z, m['alu'], c, bevel=4))
    parts.append(g.box(f, 'deck_control_box', DECK_X1 - 160, DECK_X1 - 40, -300, 300, DECK_Z + 50, DECK_Z + 520, m['white'], c, bevel=6))
    for o in parts:
        g.set_anim(o, DECK_ANIM)
    return parts


def hose_run(f, m, c):
    parts = []
    path = [(DECK_X0 + 450, 0, DECK_Z + 130), (DECK_X0 + 200, 150, DECK_Z + 100), (DECK_X0 + 60, 450, DECK_Z + 80), (DECK_X0 + 300, 700, DECK_Z + 150),
            (DECK_X0 + 500, 500, DECK_Z + 260), (DECK_X0 + 300, 250, DECK_Z + 260), (DECK_X0 + 400, 0, DECK_Z + 320)]
    parts.append(g.tube(f, 'hose_run', g.smooth_path(path, 8), 40, m['hose'], c, segments=14))
    parts.append(g.lathe(f, 'hose_coupling_body', (DECK_X0 + 400, 0, DECK_Z + 320), (0, 0, 1), [(0.0, 0.0), (45.0, 0.0), (50.0, 70.0), (62.0, 90.0), (62.0, 190.0), (48.0, 220.0), (48.0, 270.0), (0.0, 270.0)], m['alu'], c, segments=28))
    parts.append(g.cylinder(f, 'hose_coupling_collar', (DECK_X0 + 400, 0, DECK_Z + 470), 'z', 78, 50, m['red'], c, segments=28, bevel=4))
    for o in parts:
        g.set_anim(o, HOSE_ANIM)
    return parts


def decals(f, c):
    off = TANK_HALF_W + 3.0
    mat_tank = decal_mat(f'{DEFAULT_OPERATOR}_tank')
    mat_door = decal_mat(f'{DEFAULT_OPERATOR}_door')
    mat_warn = decal_mat('sign_warning')
    mat_plate = decal_mat('hazard_plate')
    if mat_tank:
        g.decal(f, 'd_tank_right', [(1800, off, 1750), (6500, off, 1750), (6500, off, 2310), (1800, off, 2310)], mat_tank, c)
        g.decal(f, 'd_tank_left', [(6500, -off, 1750), (1800, -off, 1750), (1800, -off, 2310), (6500, -off, 2310)], mat_tank, c)
    if mat_door:
        g.decal(f, 'd_door_right', [(8700, HALF_W + 3, 1000), (9250, HALF_W + 3, 1000), (9250, HALF_W + 3, 1360), (8700, HALF_W + 3, 1360)], mat_door, c)
        g.decal(f, 'd_door_left', [(9250, -HALF_W - 3, 1000), (8700, -HALF_W - 3, 1000), (8700, -HALF_W - 3, 1360), (9250, -HALF_W - 3, 1360)], mat_door, c)
    if mat_plate:
        g.decal(f, 'd_plate_rear', [(-32, -420, 1000), (-32, 420, 1000), (-32, 420, 1130), (-32, -420, 1130)], mat_plate, c)
    if mat_warn:
        for k, (x0, x1) in enumerate(((6300, 6650), (1500, 1850))):
            g.decal(f, f'd_warn_{k}', [(x0, off, 2400), (x1, off, 2400), (x1, off, 2750), (x0, off, 2750)], mat_warn, c)


def build(with_connection=True):
    g.reset_scene()
    f = g.Frame(X0)
    c = g.collection(NAME)
    m = materials()
    chassis(f, m, c)
    tank(f, m, c)
    cab(f, m, c)
    pump_cabinet(f, m, c)
    rear_cabinet(f, m, c)
    deck(f, m, c)
    hose_run(f, m, c)
    decals(f, c)
    return c


VIEWS = {
    'side_right': dict(res=(800, 330), lens=35, samples=16, loc=(14.0, 0.0, 1.6), target=(0.0, 0.0, 1.6)),
    'three_quarter_front': dict(res=(800, 560), lens=35, samples=16, loc=(-8.0, 10.5, 3.6), target=(0.0, 1.8, 1.5)),
    'three_quarter_rear': dict(res=(800, 560), lens=35, samples=16, loc=(8.0, -11.0, 4.2), target=(0.0, -2.5, 1.6)),
    'rear': dict(res=(800, 560), lens=30, samples=16, loc=(0.0, -12.0, 2.6), target=(0.0, -4.7, 1.6)),
}
