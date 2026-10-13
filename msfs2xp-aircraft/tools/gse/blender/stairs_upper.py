import os

import gselib as g

NAME = 'stairs_upper'
X0 = 6000.0
OUT_DIR = 'E:/fbw-gse/out/stairs_upper'
LIVERY_DIR = os.path.join(OUT_DIR, 'liveries')

SILL = 7860.0
FLOOR_Z = SILL - 40.0
DECK_THICK = 60.0
STAIR_HALF = 600.0
STRINGER_Y = 640.0
STRINGER_W = 40.0
RISER = 175.0
FOOT_X = 0.0
LOW_Z0 = 420.0
LANDING_Z = 4000.0
LANDING_X0 = 4300.0
LANDING_X1 = 5900.0
UPPER_X1 = LANDING_X1 + 4650.0
PLATFORM_X0 = UPPER_X1
PLATFORM_X1 = PLATFORM_X0 + 1500.0
PLATFORM_HALF = 900.0
CHASSIS_Y = 1050.0
CHASSIS_X0 = 3400.0
CHASSIS_X1 = 11200.0
CHASSIS_Z0 = 430.0
CHASSIS_Z1 = 740.0
WHEEL_R = 460.0
WHEEL_XS = (4000.0, 5200.0, 9900.0, 11000.0)
CAB_X0 = 9600.0
CAB_X1 = 11300.0
RAIL_H = 1000.0

LIFT_ANIM = [{'kind': 'trans', 'offset0': (0.0, 0.0, -3.9), 'offset1': (0.0, 0.0, 0.0), 'v0': 0.0, 'v1': 1.0, 'dataref': 'fbw/gse/stairs_upper/lift'}]
PLATFORM_ANIM = [{'kind': 'trans', 'offset0': (0.0, 0.0, 0.0), 'offset1': (0.0, 0.45, 0.0), 'v0': 0.0, 'v1': 1.0, 'dataref': 'fbw/gse/stairs_upper/platform'}]

OPERATORS = {
    'dnata': ('dnata', (0, 62, 126), (255, 255, 255)),
    'swissport': ('Swissport', (214, 0, 28), (255, 255, 255)),
    'menzies': ('MENZIES', (0, 90, 60), (255, 205, 0)),
}

VIEWS = {
    'three_quarter': dict(res=(800, 600), lens=28, samples=16, loc=(-9.5, -7.0, 8.5), target=(0.2, 1.5, 3.5)),
    'side': dict(res=(800, 420), lens=35, samples=16, loc=(14.0, 0.0, 4.2), target=(0.0, 0.5, 4.0)),
    'platform_close': dict(res=(800, 600), lens=30, samples=16, loc=(-4.0, 7.0, 9.0), target=(0.0, 5.5, 7.4)),
    'front': dict(res=(600, 700), lens=35, samples=16, loc=(0.0, -9.0, 4.0), target=(0.0, 1.0, 3.9)),
}


def materials():
    return {
        'white': g.mat('gse_white', (236, 237, 233), roughness=0.32),
        'galv': g.mat('gse_galvanized', (168, 171, 172), metallic=0.85, roughness=0.42),
        'black': g.mat('gse_black', (22, 22, 23), roughness=0.55),
        'rubber': g.mat('gse_rubber', (28, 28, 28), roughness=0.88),
        'rim': g.mat('gse_rim', (34, 34, 35), metallic=0.4, roughness=0.5),
        'hub': g.mat('gse_hub', (150, 150, 148), metallic=0.9, roughness=0.35),
        'amber': g.mat('gse_amber_lens', (255, 140, 10), roughness=0.12),
        'red': g.mat('gse_red', (185, 18, 20), roughness=0.3),
        'yellow': g.mat('gse_yellow', (245, 200, 0), roughness=0.35),
        'body': g.mat('gse_stairs_body', (232, 234, 236), roughness=0.35),
        'tread': g.mat('gse_tread', (70, 72, 74), metallic=0.5, roughness=0.6),
        'glass': g.mat('gse_cab_glass', (40, 55, 65), roughness=0.08),
        'canopy': g.mat('gse_canopy', (214, 218, 222), roughness=0.4),
        'rubber_edge': g.mat('gse_bumper_rubber', (18, 18, 18), roughness=0.9),
    }


def slope_z(x, xa, za, xb, zb):
    return za + (zb - za) * (x - xa) / (xb - xa)


def tag_anim(parts, anim):
    if anim:
        for p in parts:
            g.set_anim(p, anim)
    return parts


def flight(f, m, c, tag, xa, za, xb, zb, anim):
    parts = []
    n = int(round((zb - za) / RISER))
    run = (xb - xa) / n
    for k in (-1, 1):
        side = 'L' if k < 0 else 'R'
        prof = [(xa, za - 260.0), (xb, zb - 260.0), (xb, zb + 40), (xa, za + 40)]
        parts.append(g.prism_xz(f, f'{tag}_stringer_{side}', prof, *sorted((k * STRINGER_Y - STRINGER_W / 2, k * STRINGER_Y + STRINGER_W / 2)), m['body'], c, bevel=4))
        post_x = [xa + 80, (xa + xb) / 2, xb - 80]
        zs = [slope_z(x, xa, za, xb, zb) for x in post_x]
        for px, pz in zip(post_x, zs):
            parts.append(g.cylinder(f, f'{tag}_post_{side}_{int(px)}', (px, k * STRINGER_Y, pz + 40 + RAIL_H / 2), 'z', 20, RAIL_H, m['galv'], c, segments=12))
        for dz, nm in ((RAIL_H, 'top'), (RAIL_H * 0.5, 'mid')):
            pts = [(post_x[0], k * STRINGER_Y, zs[0] + 40 + dz), (post_x[2], k * STRINGER_Y, zs[2] + 40 + dz)]
            parts.append(g.tube(f, f'{tag}_rail_{nm}_{side}', pts, 22 if nm == 'top' else 14, m['galv'], c, segments=12))
    for k in (-1, 1):
        side = 'L' if k < 0 else 'R'
        for i in range(1, n, 2):
            bx = xa + (i + 0.5) * run
            bz = za + (i + 1) * RISER
            parts.append(g.cylinder(f, f'{tag}_baluster_{side}_{i}', (bx, k * STRINGER_Y, bz + 40 + RAIL_H * 0.25), 'z', 8, RAIL_H * 0.5, m['galv'], c, segments=8))
    for i in range(n):
        x0 = xa + i * run
        z = za + (i + 1) * RISER
        for j in range(3):
            gx = x0 + 40 + j * (run - 80) / 3
            parts.append(g.box(f, f'{tag}_groove_{i}_{j}', gx, gx + 12, -STAIR_HALF + 30, STAIR_HALF - 30, z, z + 4, m['black'], c, segments=1))
        parts.append(g.box(f, f'{tag}_tread_{i}', x0 + 5, x0 + run, -STAIR_HALF, STAIR_HALF, z - 40, z, m['tread'], c, bevel=2, segments=1))
        parts.append(g.box(f, f'{tag}_nosing_{i}', x0 + run - 40, x0 + run, -STAIR_HALF, STAIR_HALF, z - 40, z + 2, m['yellow'], c, segments=1))
        parts.append(g.box(f, f'{tag}_riser_{i}', x0 + 5, x0 + 15, -STAIR_HALF, STAIR_HALF, z - RISER, z - 40, m['black'], c, segments=1))
    return tag_anim(parts, anim)


def landing(f, m, c, tag, x0, x1, z, half_y, anim):
    parts = []
    parts.append(g.box(f, f'{tag}_deck', x0, x1, -half_y, half_y, z - DECK_THICK, z, m['tread'], c, bevel=3))
    parts.append(g.box(f, f'{tag}_frame', x0, x1, -half_y, half_y, z - 200, z - DECK_THICK, m['body'], c, bevel=4))
    for k in (-1, 1):
        side = 'L' if k < 0 else 'R'
        xs = [x0 + 40, (x0 + x1) / 2, x1 - 40]
        for px in xs:
            parts.append(g.cylinder(f, f'{tag}_post_{side}_{int(px)}', (px, k * (half_y - 20), z + RAIL_H / 2), 'z', 20, RAIL_H, m['galv'], c, segments=12))
        for dz, nm in ((RAIL_H, 'top'), (RAIL_H * 0.5, 'mid')):
            parts.append(g.tube(f, f'{tag}_rail_{nm}_{side}', [(xs[0], k * (half_y - 20), z + dz), (xs[2], k * (half_y - 20), z + dz)], 22 if nm == 'top' else 14, m['galv'], c, segments=12))
    return tag_anim(parts, anim)


def chassis(f, m, c):
    parts = []
    parts.append(g.box(f, 'chassis_frame', CHASSIS_X0, CHASSIS_X1, -CHASSIS_Y, CHASSIS_Y, CHASSIS_Z0, CHASSIS_Z1, m['galv'], c, bevel=8))
    parts.append(g.box(f, 'chassis_skirt', CHASSIS_X0 + 1000, CHASSIS_X1 - 1500, -CHASSIS_Y - 30, CHASSIS_Y + 30, CHASSIS_Z1, CHASSIS_Z1 + 500, m['body'], c, bevel=10))
    rim_profile = [(0.0, 60.0), (120.0, 60.0), (150.0, 30.0), (250.0, 30.0), (290.0, 90.0), (290.0, 110.0)]
    tyre_w = 300.0
    tyre = [(290.0, -tyre_w / 2 + 20), (WHEEL_R - 60, -tyre_w / 2), (WHEEL_R - 15, -tyre_w / 2 + 25), (WHEEL_R, -tyre_w / 2 + 60), (WHEEL_R, tyre_w / 2 - 60),
            (WHEEL_R - 15, tyre_w / 2 - 25), (WHEEL_R - 60, tyre_w / 2), (290.0, tyre_w / 2 - 20)]
    for x in WHEEL_XS:
        for k in (-1, 1):
            y = k * (CHASSIS_Y + 110)
            tag = f'{int(x)}_{"L" if k < 0 else "R"}'
            parts.append(g.lathe(f, f'tyre_{tag}', (x, y, WHEEL_R), (k, 0, 0), tyre, m['rubber'], c, segments=32))
            parts.append(g.lathe(f, f'rim_{tag}', (x, y, WHEEL_R), (k, 0, 0), [(r, -z) for r, z in rim_profile], m['rim'], c, segments=28))
            parts.append(g.cylinder(f, f'hubcap_{tag}', (x, y + k * 60, WHEEL_R), 'y', 70, 30, m['hub'], c, segments=20, bevel=4))
            yg = k * (CHASSIS_Y + 110)
            parts.append(g.box(f, f'mudguard_{tag}', x - 520, x + 520, yg - 150, yg + 150, WHEEL_R + 360, WHEEL_R + 400, m['black'], c, bevel=2, segments=1))
        parts.append(g.cylinder(f, f'axle_{int(x)}', (x, 0, WHEEL_R), 'y', 45, 2 * CHASSIS_Y, m['galv'], c, segments=16))
    for xo in (CHASSIS_X0 + 100, CHASSIS_X1 - 100):
        for k in (-1, 1):
            parts.append(g.cylinder(f, f'stabiliser_pad_{int(xo)}_{k}', (xo, k * (CHASSIS_Y - 150), 200), 'z', 140, 40, m['rubber_edge'], c, segments=20, bevel=3))
            parts.append(g.box(f, f'stabiliser_leg_{int(xo)}_{k}', xo - 40, xo + 40, k * (CHASSIS_Y - 190) - 40, k * (CHASSIS_Y - 190) + 40, 220, CHASSIS_Z1, m['yellow'], c, bevel=3))
    return parts


def cab(f, m, c):
    parts = []
    parts.append(g.box(f, 'cab_body', CAB_X0, CAB_X1, -CHASSIS_Y + 50, CHASSIS_Y - 50, CHASSIS_Z1, 2500.0, m['white'], c, bevel=40))
    parts.append(g.box(f, 'cab_windscreen', CAB_X1 - 4, CAB_X1 + 6, -CHASSIS_Y + 150, CHASSIS_Y - 150, 1350.0, 2350.0, m['glass'], c, bevel=3))
    for k in (-1, 1):
        parts.append(g.box(f, f'cab_side_window_{k}', CAB_X0 + 250, CAB_X1 - 250, k * (CHASSIS_Y - 54) - 5, k * (CHASSIS_Y - 54) + 5, 1350.0, 2250.0, m['glass'], c, bevel=3))
        parts.append(g.box(f, f'cab_headlight_{k}', CAB_X1 - 2, CAB_X1 + 8, k * 800 - 110, k * 800 + 110, 900.0, 1020.0, m['yellow'], c, bevel=3))
        parts.append(g.box(f, f'cab_mirror_{k}', CAB_X1 - 250, CAB_X1 - 180, k * (CHASSIS_Y + 170) - 50, k * (CHASSIS_Y + 170) + 50, 1700.0, 2000.0, m['black'], c, bevel=4))
    parts.append(g.box(f, 'front_bumper', CAB_X1, CAB_X1 + 90, -CHASSIS_Y, CHASSIS_Y, 520.0, 780.0, m['rubber_edge'], c, bevel=15))
    parts.append(g.cylinder(f, 'cab_beacon', (CAB_X0 + 400, 0, 2560.0), 'z', 60, 100, m['amber'], c, segments=20, bevel=3))
    return parts


def canopy(f, m, c):
    parts = []
    z0 = FLOOR_Z
    z1 = z0 + 2350.0
    x0, x1 = PLATFORM_X0 + 100, PLATFORM_X1 - 30
    for k in (-1, 1):
        for xp in (x0, x1):
            parts.append(g.box(f, f'canopy_post_{k}_{int(xp)}', xp - 40, xp + 40, k * (PLATFORM_HALF - 60) - 40, k * (PLATFORM_HALF - 60) + 40, z0, z1, m['body'], c, bevel=5))
    parts.append(g.box(f, 'canopy_roof', x0 - 150, x1 + 100, -PLATFORM_HALF - 100, PLATFORM_HALF + 100, z1, z1 + 80, m['canopy'], c, bevel=10))
    parts.append(g.box(f, 'canopy_back_panel', x0 - 20, x0 + 20, -PLATFORM_HALF + 60, PLATFORM_HALF - 60, z0 + 1100, z1, m['canopy'], c, bevel=4))
    parts.append(g.box(f, 'canopy_roof_light', x1 + 20, x1 + 80, -300, 300, z1 - 30, z1 + 20, m['yellow'], c, bevel=3))
    return tag_anim(parts, LIFT_ANIM)


def platform(f, m, c):
    top = FLOOR_Z
    fixed = [
        g.box(f, 'platform_fixed_deck', PLATFORM_X0, PLATFORM_X1 - 300, -PLATFORM_HALF, PLATFORM_HALF, top - DECK_THICK, top, m['tread'], c, bevel=3),
        g.box(f, 'platform_fixed_frame', PLATFORM_X0, PLATFORM_X1 - 300, -PLATFORM_HALF, PLATFORM_HALF, top - 250, top - DECK_THICK, m['body'], c, bevel=4),
    ]
    for k in (-1, 1):
        side = 'L' if k < 0 else 'R'
        y = k * (PLATFORM_HALF - 20)
        for xp in (PLATFORM_X0 + 40, PLATFORM_X0 + 700):
            fixed.append(g.cylinder(f, f'platform_post_{side}_{int(xp)}', (xp, y, top + RAIL_H / 2), 'z', 20, RAIL_H, m['galv'], c, segments=12))
        fixed.append(g.tube(f, f'platform_rail_top_{side}', [(PLATFORM_X0 + 40, y, top + RAIL_H), (PLATFORM_X0 + 700, y, top + RAIL_H)], 22, m['galv'], c, segments=12))
        fixed.append(g.tube(f, f'platform_rail_mid_{side}', [(PLATFORM_X0 + 40, y, top + RAIL_H / 2), (PLATFORM_X0 + 700, y, top + RAIL_H / 2)], 14, m['galv'], c, segments=12))
    slide = [
        g.box(f, 'platform_slide_deck', PLATFORM_X1 - 300, PLATFORM_X1, -PLATFORM_HALF + 80, PLATFORM_HALF - 80, top - DECK_THICK, top, m['tread'], c, bevel=3),
        g.box(f, 'platform_slide_frame', PLATFORM_X1 - 300, PLATFORM_X1, -PLATFORM_HALF + 80, PLATFORM_HALF - 80, top - 120, top - DECK_THICK, m['body'], c, bevel=3),
        g.box(f, 'platform_slide_bumper', PLATFORM_X1, PLATFORM_X1 + 120, -PLATFORM_HALF + 40, PLATFORM_HALF - 40, top - 180, top + 120, m['rubber_edge'], c, bevel=25),
        g.box(f, 'platform_slide_edge_stripe', PLATFORM_X1 - 90, PLATFORM_X1 - 40, -PLATFORM_HALF + 80, PLATFORM_HALF - 80, top, top + 3, m['yellow'], c),
    ]
    tag_anim(fixed, LIFT_ANIM)
    tag_anim(slide, LIFT_ANIM + PLATFORM_ANIM)
    return fixed + slide


def lift_column(f, m, c):
    parts = []
    for k in (-1, 1):
        side = 'L' if k < 0 else 'R'
        parts.append(g.box(f, f'lift_column_{side}', LANDING_X1 - 300, LANDING_X1 + 100, k * (CHASSIS_Y - 150) - 60, k * (CHASSIS_Y - 150) + 60, CHASSIS_Z1, LANDING_Z - 200, m['galv'], c, bevel=6))
    for k in (-1, 1):
        side = 'L' if k < 0 else 'R'
        parts.append(g.box(f, f'upper_support_{side}', UPPER_X1 - 400, UPPER_X1 - 340, k * (STRINGER_Y + 20) - 30, k * (STRINGER_Y + 20) + 30, 3000.0, FLOOR_Z - 300, m['galv'], c, bevel=3))
        g.set_anim(parts[-1], LIFT_ANIM)
    return parts


def mast(f, m, c):
    parts = []
    mx = PLATFORM_X0 - 200
    parts.append(g.box(f, 'mast_outer', mx - 220, mx + 220, -300, 300, CHASSIS_Z1, 4800.0, m['white'], c, bevel=10))
    inner = [g.box(f, 'mast_inner', mx - 190, mx + 190, -270, 270, 4000.0, FLOOR_Z - 250, m['galv'], c, bevel=8)]
    inner.append(g.box(f, 'mast_head', mx - 330, mx + 330, -PLATFORM_HALF + 100, PLATFORM_HALF - 100, FLOOR_Z - 450, FLOOR_Z - 250, m['body'], c, bevel=8))
    tag_anim(inner, LIFT_ANIM)
    return parts + inner


def lower_foot(f, m, c):
    parts = [
        g.box(f, 'foot_plate', FOOT_X - 300, FOOT_X + 40, -STAIR_HALF - 40, STAIR_HALF + 40, 0.0, LOW_Z0 - 120, m['tread'], c, bevel=8),
        g.box(f, 'foot_bumper', FOOT_X - 330, FOOT_X - 300, -STAIR_HALF - 40, STAIR_HALF + 40, 0.0, 200.0, m['rubber_edge'], c, bevel=6),
    ]
    for k in (-1, 1):
        parts.append(g.box(f, f'foot_support_{k}', FOOT_X + 40, FOOT_X + 100, k * STRINGER_Y - 40, k * STRINGER_Y + 40, 0.0, LOW_Z0, m['galv'], c, bevel=4))
    return parts


def livery_png(path, name, bg, fg):
    from PIL import Image, ImageDraw, ImageFont
    w, h = 1024, 256
    img = Image.new('RGBA', (w, h), (*bg, 255))
    d = ImageDraw.Draw(img)
    d.rectangle((0, h - 40, w, h), fill=(*fg, 255))
    font = ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf', 150)
    box = d.textbbox((0, 0), name, font=font)
    d.text(((w - (box[2] - box[0])) / 2 - box[0], (h - 40 - (box[3] - box[1])) / 2 - box[1]), name, font=font, fill=(*fg, 255))
    img.save(path)


def liveries(out=None):
    os.makedirs(LIVERY_DIR, exist_ok=True)
    paths = {}
    for op, (label, bg, fg) in OPERATORS.items():
        path = os.path.join(LIVERY_DIR, f'{op}.png')
        if not os.path.exists(path):
            livery_png(path, label, bg, fg)
        paths[op] = path
    return paths


def quad(k, x0, x1, y, za0, zb0, za1, zb1):
    if k > 0:
        return [(x0, y, za0), (x1, y, zb0), (x1, y, zb1), (x0, y, za1)]
    return [(x1, y, zb0), (x0, y, za0), (x0, y, za1), (x1, y, zb1)]


def decals(f, c):
    try:
        path = liveries()['dnata']
    except Exception:
        return
    mat = g.image_mat('decal_operator', path, roughness=0.4)
    x0, x1 = 700.0, 3700.0
    za, zb = slope_z(x0, FOOT_X + 40, LOW_Z0, LANDING_X0, LANDING_Z), slope_z(x1, FOOT_X + 40, LOW_Z0, LANDING_X0, LANDING_Z)
    for k in (-1, 1):
        y = k * (STRINGER_Y + STRINGER_W / 2 + 2)
        g.decal(f, f'decal_operator_{k}', quad(k, x0, x1, y, za - 230, zb - 230, za - 50, zb - 50), mat, c)
    for k in (-1, 1):
        y = k * (CHASSIS_Y + 32)
        z0, z1 = CHASSIS_Z1 + 100, CHASSIS_Z1 + 400
        g.decal(f, f'decal_chassis_{k}', quad(k, CHASSIS_X0 + 1200, CHASSIS_X1 - 1800, y, z0, z0, z1, z1), mat, c)


def build(with_connection=True):
    g.reset_scene()
    f = g.Frame(X0)
    c = g.collection(NAME)
    m = materials()
    chassis(f, m, c)
    cab(f, m, c)
    lift_column(f, m, c)
    mast(f, m, c)
    lower_foot(f, m, c)
    flight(f, m, c, 'low', FOOT_X + 40, LOW_Z0, LANDING_X0, LANDING_Z, None)
    landing(f, m, c, 'mid', LANDING_X0, LANDING_X1, LANDING_Z, STAIR_HALF + 40, None)
    flight(f, m, c, 'up', LANDING_X1, LANDING_Z, UPPER_X1, FLOOR_Z, LIFT_ANIM)
    platform(f, m, c)
    canopy(f, m, c)
    decals(f, c)
    return c
