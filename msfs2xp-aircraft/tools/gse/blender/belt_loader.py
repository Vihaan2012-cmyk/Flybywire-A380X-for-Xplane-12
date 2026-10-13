import math
import os

import bpy

import gselib as g

NAME = 'belt_loader'
X0 = 0.0
OUT_DIR = os.environ.get('GSE_BELT_OUT', 'E:/fbw-gse/out/belt_loader')
LIVERY_DIR = os.path.join(OUT_DIR, 'liveries')
RAISED_PREVIEW = float(os.environ.get('GSE_BELT_RAISE', '0'))

CHASSIS_L = 4600.0
CHASSIS_HW = 800.0
TRACK = 1560.0
REAR_AXLE = 900.0
FRONT_AXLE = 3700.0
TYRE_D = 640.0
TYRE_W = 225.0
FRAME_Z0 = 330.0
FRAME_Z1 = 700.0

HINGE_X = 700.0
HINGE_Z = 820.0
BOOM_LEN = 7400.0
BOOM_Y = -250.0
BOOM_HALF = 350.0
BELT_HALF = 300.0
BOOM_RAISE_DEG = 14.0
BOOM_TOP_Z = 150.0

BOOM_ANIM = [{'kind': 'rotate', 'pivot': (BOOM_Y / 1000.0, HINGE_X / 1000.0, HINGE_Z / 1000.0), 'axis': (1.0, 0.0, 0.0), 'deg0': 0.0, 'deg1': BOOM_RAISE_DEG,
              'v0': 0.0, 'v1': 1.0, 'dataref': 'fbw/gse/belt/boom'}]

OPERATORS = {
    'dnata': dict(text='dnata', body=(240, 240, 238), band=(200, 30, 45), ink=(200, 30, 45)),
    'swissport': dict(text='SWISSPORT', body=(240, 240, 238), band=(0, 70, 150), ink=(0, 70, 150)),
    'menzies': dict(text='MENZIES', body=(240, 240, 238), band=(240, 130, 20), ink=(40, 40, 40)),
}


def materials():
    return {
        'white': g.mat('gse_white', (236, 237, 233), roughness=0.32),
        'galv': g.mat('gse_galvanized', (168, 171, 172), metallic=0.85, roughness=0.42),
        'black': g.mat('gse_black', (22, 22, 23), roughness=0.55),
        'glass': g.mat('gse_screen', (18, 28, 36), roughness=0.08),
        'rubber': g.mat('gse_rubber', (28, 28, 28), roughness=0.88),
        'belt': g.mat('gse_belt', (40, 41, 43), roughness=0.8),
        'rim': g.mat('gse_rim', (34, 34, 35), metallic=0.4, roughness=0.5),
        'hub': g.mat('gse_hub', (150, 150, 148), metallic=0.9, roughness=0.35),
        'amber': g.mat('gse_amber_lens', (255, 140, 10), roughness=0.12),
        'red': g.mat('gse_red', (185, 18, 20), roughness=0.3),
        'yellow': g.mat('gse_yellow', (245, 200, 0), roughness=0.35),
        'steel': g.mat('gse_steel_grey', (92, 96, 100), metallic=0.6, roughness=0.5),
        'beam': g.mat('gse_beam_white', (226, 227, 224), roughness=0.4),
        'cylinder': g.mat('gse_cylinder_chrome', (190, 192, 194), metallic=0.95, roughness=0.25),
    }


def draw_liveries(out=None):
    out = out or LIVERY_DIR
    os.makedirs(out, exist_ok=True)
    from PIL import Image, ImageDraw, ImageFont
    paths = {}
    try:
        font = ImageFont.truetype('arialbd.ttf', 130)
    except OSError:
        font = ImageFont.load_default()
    for key, spec in OPERATORS.items():
        img = Image.new('RGBA', (2048, 256), spec['body'] + (255,))
        d = ImageDraw.Draw(img)
        d.rectangle([0, 170, 2048, 256], fill=spec['band'] + (255,))
        d.rectangle([0, 0, 2048, 14], fill=spec['band'] + (255,))
        d.text((90, 20), spec['text'], fill=spec['ink'] + (255,), font=font)
        path = os.path.join(out, f'{key}_side.png')
        img.save(path)
        paths[key] = path
    return paths


def liveries(out):
    try:
        return draw_liveries(os.path.join(out, 'liveries'))
    except ImportError:
        return {}


def rot(x, z, angle_deg):
    a = math.radians(angle_deg)
    dx, dz = x - HINGE_X, z - HINGE_Z
    return HINGE_X + dx * math.cos(a) - dz * math.sin(a), HINGE_Z + dx * math.sin(a) + dz * math.cos(a)


def boom_pt(u, w, y, angle):
    x, z = rot(HINGE_X + u, HINGE_Z + w, angle)
    return (x, y, z)


def boom_profile(pts_uw, angle):
    return [rot(HINGE_X + u, HINGE_Z + w, angle) for u, w in pts_uw]


def span(a, b):
    return min(a, b), max(a, b)


def chassis(f, m, c):
    parts = []
    parts.append(g.box(f, 'frame_main', 0, CHASSIS_L, -CHASSIS_HW + 100, CHASSIS_HW - 100, FRAME_Z0, FRAME_Z1, m['steel'], c, bevel=6))
    for k, side in ((-1, 'L'), (1, 'R')):
        parts.append(g.box(f, f'sill_{side}', 40, CHASSIS_L - 40, *span(k * (CHASSIS_HW - 110), k * CHASSIS_HW), 380, 760, m['white'], c, bevel=10))
        parts.append(g.box(f, f'rubrail_{side}', 0, CHASSIS_L, *span(k * (CHASSIS_HW - 10), k * (CHASSIS_HW + 30)), 420, 500, m['black'], c, bevel=8))
        parts.append(g.box(f, f'reflector_rear_{side}', -4, 4, *span(k * 500, k * 700), 500, 560, m['red'], c))
    parts.append(g.box(f, 'rear_bumper', -60, 0, -CHASSIS_HW, CHASSIS_HW, 380, 520, m['black'], c, bevel=12))
    parts.append(g.box(f, 'front_bumper', CHASSIS_L, CHASSIS_L + 60, -CHASSIS_HW, CHASSIS_HW, 380, 540, m['black'], c, bevel=12))
    parts.append(g.box(f, 'engine_cover', 90, 1500, -CHASSIS_HW + 60, CHASSIS_HW - 60, 700, 1020, m['white'], c, bevel=24))
    parts.append(g.box(f, 'engine_grille', 80, 94, -520, 520, 760, 960, m['black'], c, bevel=3))
    for i in range(7):
        parts.append(g.box(f, f'grille_slat_{i}', 76, 84, -500, 500, 780 + i * 25, 794 + i * 25, m['steel'], c))
    parts.append(g.box(f, 'tank_box', 1500, 2300, -CHASSIS_HW + 60, -150, 700, 960, m['white'], c, bevel=16))
    parts.append(g.box(f, 'boom_cradle_rear', HINGE_X - 120, HINGE_X + 120, BOOM_Y - 240, BOOM_Y + 240, 700, HINGE_Z - 40, m['steel'], c, bevel=8))
    for k in (-1, 1):
        parts.append(g.box(f, f'hinge_lug_{k}', HINGE_X - 70, HINGE_X + 70, *span(BOOM_Y + k * 270, BOOM_Y + k * 310), HINGE_Z - 80, HINGE_Z + 80, m['steel'], c, bevel=6))
    parts.append(g.cylinder(f, 'hinge_pin', (HINGE_X, BOOM_Y, HINGE_Z), 'y', 28, 700, m['galv'], c, segments=20))
    for xp in (3200.0, 3500.0):
        parts.append(g.box(f, f'rest_pad_{int(xp)}', xp - 120, xp + 120, BOOM_Y - 240, BOOM_Y + 240, 700, 760, m['black'], c, bevel=6))
    parts.append(g.box(f, 'step_plate', 3000, 3300, 520, CHASSIS_HW + 140, 300, 330, m['galv'], c, bevel=3))
    for i in range(6):
        parts.append(g.box(f, f'step_tread_{i}', 3010 + i * 48, 3040 + i * 48, 520, CHASSIS_HW + 140, 330, 340, m['black'], c))
    parts.append(g.box(f, 'tow_pintle', -150, 0, -90, 90, 420, 500, m['galv'], c, bevel=8))
    return parts


def cab(f, m, c):
    parts = []
    x0, x1, y0, y1 = 2750.0, 4350.0, 40.0, CHASSIS_HW + 40
    parts.append(g.box(f, 'cab_floor', x0, x1, y0, y1, 700, 760, m['steel'], c, bevel=4))
    for i, (px, py) in enumerate([(x0, y0), (x0, y1), (x1, y0), (x1, y1)]):
        top = 2120.0 if px == x0 else 2000.0
        parts.append(g.box(f, f'cab_pillar_{i}', px - 25, px + 25, py - 25, py + 25, 760, top, m['white'], c, bevel=4))
    roof_profile = [(x0 - 60, 2120.0), (x1 + 80, 2020.0), (x1 + 80, 1980.0), (x0 - 60, 2080.0)]
    parts.append(g.prism_xz(f, 'cab_roof', roof_profile, y0 - 40, y1 + 40, m['white'], c, bevel=10))
    parts.append(g.box(f, 'cab_back_lower', x0, x0 + 30, y0, y1, 760, 1200, m['white'], c, bevel=4))
    parts.append(g.box(f, 'cab_back_glass', x0 + 4, x0 + 12, y0 + 30, y1 - 30, 1220, 2060, m['glass'], c))
    parts.append(g.box(f, 'cab_door_lower', x0 + 120, x1 - 360, y1 - 30, y1, 760, 1200, m['white'], c, bevel=4))
    parts.append(g.box(f, 'cab_door_glass', x0 + 120, x1 - 360, y1 - 28, y1 - 22, 1220, 2040, m['glass'], c))
    parts.append(g.box(f, 'cab_windscreen', x1 - 14, x1 - 4, y0 + 20, y1 - 20, 1240, 1980, m['glass'], c))
    parts.append(g.box(f, 'cab_dash', x1 - 450, x1 - 20, y0 + 20, y1 - 20, 760, 1230, m['black'], c, bevel=10))
    parts.append(g.box(f, 'cab_front_skirt', x1 - 20, x1, y0, y1, 760, 1240, m['white'], c, bevel=4))
    parts.append(g.box(f, 'cab_seat_base', x0 + 250, x0 + 650, 300, 700, 760, 900, m['black'], c, bevel=8))
    parts.append(g.box(f, 'cab_seat_back', x0 + 250, x0 + 330, 300, 700, 900, 1500, m['black'], c, bevel=10))
    parts.append(g.cylinder(f, 'steering_wheel', (x1 - 520, 520, 1180), 'x', 190, 30, m['black'], c, segments=24, bevel=4))
    parts.append(g.cylinder(f, 'cab_beacon_base', (x0 + 400, 400, 2125), 'z', 55, 40, m['black'], c, segments=20, bevel=3))
    parts.append(g.lathe(f, 'cab_beacon_lens', (x0 + 400, 400, 2145), 'z', [(0.0, 90.0), (28.0, 86.0), (46.0, 70.0), (48.0, 0.0)], m['amber'], c, segments=24))
    for k in (-1, 1):
        parts.append(g.box(f, f'headlamp_{k}', x1 + 2, x1 + 40, *span(k * 640, k * 640 + 160), 560, 640, m['yellow'], c, bevel=6))
    parts.append(g.box(f, 'mirror_arm', x0 + 300, x0 + 340, y1, y1 + 220, 1700, 1730, m['black'], c, bevel=3))
    parts.append(g.box(f, 'mirror', x0 + 280, x0 + 360, y1 + 200, y1 + 240, 1620, 1860, m['black'], c, bevel=6))
    return parts


def wheels(f, m, c):
    parts = []
    r = TYRE_D / 2
    rr = 190.0
    half_w = TYRE_W / 2
    tyre_profile = [(rr, -half_w + 18), (rr + 8, -half_w + 4), (r - 45, -half_w), (r - 14, -half_w + 12), (r, -half_w + 45), (r, half_w - 45),
                    (r - 14, half_w - 12), (r - 45, half_w), (rr + 8, half_w - 4), (rr, half_w - 18)]
    rim_profile = [(0.0, 52.0), (70.0, 52.0), (85.0, 40.0), (140.0, 32.0), (rr - 2, 28.0), (rr + 6, 70.0), (rr + 6, 82.0)]
    for x, label in ((REAR_AXLE, 'rear'), (FRONT_AXLE, 'front')):
        for k in (-1, 1):
            y = k * TRACK / 2
            name = f'{label}_{"L" if k < 0 else "R"}'
            parts.append(g.lathe(f, f'tyre_{name}', (x, y, r), (k, 0, 0), tyre_profile, m['rubber'], c, segments=40))
            parts.append(g.lathe(f, f'rim_{name}', (x, y, r), (k, 0, 0), [(rad, -z) for rad, z in rim_profile], m['rim'], c, segments=32))
            parts.append(g.cylinder(f, f'hubcap_{name}', (x, y + k * 55, r), 'y', 48, 24, m['hub'], c, segments=20, bevel=4))
            for b in range(5):
                a = 2 * math.pi * b / 5
                parts.append(g.cylinder(f, f'lug_{name}_{b}', (x + 70 * math.cos(a), y + k * 50, r + 70 * math.sin(a)), 'y', 10, 20, m['hub'], c, segments=8))
            arch = [(x + 400 * math.cos(math.radians(a)), 330 + 400 * math.sin(math.radians(a))) for a in range(0, 181, 15)]
            inner = [(x + 360 * math.cos(math.radians(a)), 330 + 360 * math.sin(math.radians(a))) for a in range(180, -1, -15)]
            parts.append(g.prism_xz(f, f'fender_{name}', arch + inner, *span(y - k * 135, y + k * 135), m['white'], c, bevel=3))
        parts.append(g.cylinder(f, f'axle_{label}', (x, 0, r), 'y', 55, TRACK - TYRE_W, m['steel'], c, segments=16))
    parts.append(g.box(f, 'diff_housing', REAR_AXLE - 150, REAR_AXLE + 150, -180, 180, r - 130, r + 130, m['steel'], c, bevel=20))
    return parts


def boom_parts(f, m, c, angle):
    parts = []
    half = BOOM_HALF
    for k, side in ((-1, 'L'), (1, 'R')):
        yy = BOOM_Y + k * (half - 20)
        section = boom_profile([(0.0, -90.0), (BOOM_LEN, -20.0), (BOOM_LEN, BOOM_TOP_Z - 20), (0.0, BOOM_TOP_Z - 20)], angle)
        parts.append(g.prism_xz(f, f'boom_beam_{side}', section, yy - 20, yy + 20, m['beam'], c, bevel=5))
        top_lip = boom_profile([(0.0, BOOM_TOP_Z - 20), (BOOM_LEN, BOOM_TOP_Z - 20), (BOOM_LEN, BOOM_TOP_Z + 20), (0.0, BOOM_TOP_Z + 20)], angle)
        parts.append(g.prism_xz(f, f'boom_lip_{side}', top_lip, *span(BOOM_Y + k * (half - 50), BOOM_Y + k * (half + 10)), m['galv'], c, bevel=3))
    deck = boom_profile([(0.0, BOOM_TOP_Z - 40), (BOOM_LEN, BOOM_TOP_Z - 40), (BOOM_LEN, BOOM_TOP_Z - 10), (0.0, BOOM_TOP_Z - 10)], angle)
    parts.append(g.prism_xz(f, 'boom_deck', deck, BOOM_Y - BELT_HALF - 10, BOOM_Y + BELT_HALF + 10, m['steel'], c, bevel=3))
    belt_top = boom_profile([(60.0, BOOM_TOP_Z - 10), (BOOM_LEN - 60, BOOM_TOP_Z - 10), (BOOM_LEN - 60, BOOM_TOP_Z + 14), (60.0, BOOM_TOP_Z + 14)], angle)
    parts.append(g.prism_xz(f, 'belt_surface', belt_top, BOOM_Y - BELT_HALF, BOOM_Y + BELT_HALF, m['belt'], c, bevel=2))
    for i in range(1, 25):
        u = 150.0 + i * 285.0
        if u > BOOM_LEN - 150:
            break
        cleat = boom_profile([(u - 14, BOOM_TOP_Z + 12), (u + 14, BOOM_TOP_Z + 12), (u + 14, BOOM_TOP_Z + 30), (u - 14, BOOM_TOP_Z + 30)], angle)
        parts.append(g.prism_xz(f, f'belt_cleat_{i}', cleat, BOOM_Y - BELT_HALF + 20, BOOM_Y + BELT_HALF - 20, m['rubber'], c))
    for label, u in (('rear', 30.0), ('tip', BOOM_LEN - 30.0)):
        x, z = rot(HINGE_X + u, HINGE_Z + BOOM_TOP_Z - 10, angle)
        parts.append(g.cylinder(f, f'belt_roller_{label}', (x, BOOM_Y, z), 'y', 85, 2 * BELT_HALF + 40, m['belt'], c, segments=20))
        parts.append(g.cylinder(f, f'belt_roller_axle_{label}', (x, BOOM_Y, z), 'y', 18, 2 * half + 20, m['galv'], c, segments=12))
    for i in range(1, 14):
        x, z = rot(HINGE_X + i * 520.0, HINGE_Z + BOOM_TOP_Z - 70, angle)
        parts.append(g.cylinder(f, f'return_idler_{i}', (x, BOOM_Y, z), 'y', 28, 2 * BELT_HALF, m['galv'], c, segments=10))
    for k, side in ((-1, 'L'), (1, 'R')):
        yo = BOOM_Y + k * (half + 15)
        for u in (1000.0, 2800.0, 4600.0, 6400.0):
            x0, z0 = rot(HINGE_X + u, HINGE_Z + BOOM_TOP_Z + 10, angle)
            x1, z1 = rot(HINGE_X + u, HINGE_Z + BOOM_TOP_Z + 560, angle)
            parts.append(g.tube(f, f'rail_post_{side}_{int(u)}', [(x0, yo, z0), (x1, yo, z1)], 18, m['galv'], c, segments=10))
        top = [boom_pt(1000.0, BOOM_TOP_Z + 560, yo, angle), boom_pt(6400.0, BOOM_TOP_Z + 560, yo, angle), boom_pt(BOOM_LEN - 40.0, BOOM_TOP_Z + 380, yo, angle)]
        parts.append(g.tube(f, f'handrail_{side}', top, 18, m['galv'], c, segments=12))
        mid = [boom_pt(1000.0, BOOM_TOP_Z + 300, yo, angle), boom_pt(6400.0, BOOM_TOP_Z + 300, yo, angle)]
        parts.append(g.tube(f, f'midrail_{side}', mid, 14, m['galv'], c, segments=10))
        guard = boom_profile([(0.0, BOOM_TOP_Z + 20), (BOOM_LEN, BOOM_TOP_Z + 20), (BOOM_LEN, BOOM_TOP_Z + 130), (0.0, BOOM_TOP_Z + 130)], angle)
        parts.append(g.prism_xz(f, f'kick_plate_{side}', guard, yo - 5, yo + 5, m['white'], c, bevel=2))
    tip_x, tip_z = rot(HINGE_X + BOOM_LEN, HINGE_Z + BOOM_TOP_Z, angle)
    bumper = boom_profile([(BOOM_LEN - 20, BOOM_TOP_Z - 120), (BOOM_LEN + 130, BOOM_TOP_Z - 100), (BOOM_LEN + 150, BOOM_TOP_Z + 90), (BOOM_LEN - 20, BOOM_TOP_Z + 60)], angle)
    parts.append(g.prism_xz(f, 'tip_bumper', bumper, BOOM_Y - 330, BOOM_Y + 330, m['rubber'], c, bevel=18))
    flap = boom_profile([(BOOM_LEN + 10, BOOM_TOP_Z + 40), (BOOM_LEN + 190, BOOM_TOP_Z - 20), (BOOM_LEN + 200, BOOM_TOP_Z - 50), (BOOM_LEN + 10, BOOM_TOP_Z + 10)], angle)
    parts.append(g.prism_xz(f, 'tip_flap', flap, BOOM_Y - BELT_HALF, BOOM_Y + BELT_HALF, m['belt'], c))
    for k in (-1, 1):
        parts.append(g.box(f, f'tip_lamp_{k}', tip_x - 30, tip_x + 20, *span(BOOM_Y + k * 360, BOOM_Y + k * 410), tip_z - 140, tip_z - 60, m['amber'], c, bevel=3))
        parts.append(g.box(f, f'tip_reflector_{k}', tip_x - 5, tip_x + 5, BOOM_Y + k * 120 - 40, BOOM_Y + k * 120 + 40, tip_z - 100, tip_z - 60, m['red'], c))
    for i in range(5):
        x0, z0 = rot(HINGE_X + 600.0 + i * 1500.0, HINGE_Z - 120, angle)
        parts.append(g.box(f, f'boom_cross_{i}', x0 - 40, x0 + 40, BOOM_Y - half + 20, BOOM_Y + half - 20, z0 - 40, z0 + 10, m['steel'], c, bevel=3))
    return parts


def lift_cylinder(f, m, c):
    base = (1900.0, BOOM_Y, 760.0)
    attach = boom_pt(2500.0, -90.0, BOOM_Y, 0.0)
    mid = (base[0] + (attach[0] - base[0]) * 0.55, BOOM_Y, base[2] + (attach[2] - base[2]) * 0.55)
    return [g.tube(f, 'lift_cylinder_body', [base, mid], 40, m['steel'], c, segments=14),
            g.tube(f, 'lift_cylinder_rod', [mid, attach], 24, m['cylinder'], c, segments=12)]


def decal_material(key):
    path = os.path.join(LIVERY_DIR, f'{key}_side.png')
    if not os.path.exists(path):
        draw_liveries()
    return g.image_mat(f'decal_{key}', path, roughness=0.4)


def decals(f, c, angle):
    key = 'dnata'
    mat_d = decal_material(key)
    for k in (-1, 1):
        yo = BOOM_Y + k * (BOOM_HALF - 20 + 21)
        w0, w1 = -80.0, BOOM_TOP_Z - 24.0
        a = boom_pt(1200.0, w0, yo, angle)
        b = boom_pt(5400.0, w0, yo, angle)
        c2 = boom_pt(5400.0, w1, yo, angle)
        d = boom_pt(1200.0, w1, yo, angle)
        obj = g.decal(f, f'livery_boom_{k}', [a, b, c2, d] if k > 0 else [b, a, d, c2], mat_d, c)
        obj['gse_livery'] = key
        yb = k * (CHASSIS_HW + 1.5)
        side = [(1700.0, yb, 400.0), (3300.0, yb, 400.0), (3300.0, yb, 700.0), (1700.0, yb, 700.0)]
        g.decal(f, f'livery_chassis_{k}', side if k > 0 else [side[1], side[0], side[3], side[2]], mat_d, c)


def build(with_connection=True):
    g.reset_scene()
    f = g.Frame(X0)
    c = g.collection(NAME)
    m = materials()
    chassis(f, m, c)
    cab(f, m, c)
    wheels(f, m, c)
    lift_cylinder(f, m, c)
    angle = RAISED_PREVIEW
    boom_parts(f, m, c, angle)
    decals(f, c, angle)
    for o in c.objects:
        if o.name.startswith(('boom_', 'belt_', 'return_idler', 'rail_post', 'handrail', 'midrail', 'kick_plate', 'tip_', 'livery_boom')):
            g.set_anim(o, BOOM_ANIM)
    return c


VIEWS = {
    'three_quarter_front': dict(res=(800, 560), lens=35, samples=16, loc=(-7.0, 11.0, 3.6), target=(0.4, 4.0, 1.2)),
    'side_right': dict(res=(800, 400), lens=35, samples=16, loc=(14.0, 4.0, 1.4), target=(0.0, 4.0, 1.2)),
    'cab_rear': dict(res=(800, 560), lens=30, samples=16, loc=(4.8, -4.5, 2.6), target=(0.3, 2.5, 1.0)),
    'top_front': dict(res=(800, 400), lens=35, samples=16, loc=(0.0, 14.5, 3.0), target=(0.0, 2.8, 1.3)),
}
