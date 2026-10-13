import json
import math
import os

import gselib as g

NAME = 'cat_highloader'
LIVERY_DIR = 'E:/fbw-gse/out/cat_highloader/liveries'
DEFAULT_OPERATOR = 'sats'

LENGTH = 8200.0
WIDTH = 2500.0
HY = WIDTH / 2
CHASSIS_TOP = 1100.0
BOX_X0 = 250.0
BOX_X1 = 5350.0
BOX_W = 2400.0
BOX_H = 2300.0
BOX_FLOOR = 1650.0
SILL_HEIGHT = 5100.0
LIFT_TRAVEL = SILL_HEIGHT - BOX_FLOOR
BRIDGE_TRAVEL = 1300.0
AXLES = (1300.0, 2600.0, 6500.0)
TYRE_D = 1020.0
TYRE_W = 315.0
TRACK = 2040.0
COLUMN_XS = (1100.0, 4500.0)
COLUMN_YS = (-900.0, 900.0)

LIFT_DATAREF = 'fbw/gse/cat/lift'
BRIDGE_DATAREF = 'fbw/gse/cat/bridge'


def lift_step(fraction):
    return {'kind': 'trans', 'offset0': (0.0, 0.0, 0.0), 'offset1': (0.0, 0.0, LIFT_TRAVEL * fraction / 1000.0), 'v0': 0.0, 'v1': 1.0, 'dataref': LIFT_DATAREF}


BRIDGE_STEP = {'kind': 'trans', 'offset0': (0.0, 0.0, 0.0), 'offset1': (0.0, BRIDGE_TRAVEL / 1000.0, 0.0), 'v0': 0.0, 'v1': 1.0, 'dataref': BRIDGE_DATAREF}

OPERATORS = {
    'sats': dict(label='SATS', sub='SINGAPORE AIR TERMINAL SERVICES', bg=(255, 255, 255), band=(0, 70, 150), text=(0, 70, 150), accent=(237, 28, 36)),
    'lsg': dict(label='LSG SKY CHEFS', sub='CATERING SERVICES', bg=(255, 255, 255), band=(200, 16, 46), text=(200, 16, 46), accent=(30, 30, 30)),
    'gategroup': dict(label='gategroup', sub='AIRLINE CATERING', bg=(245, 245, 245), band=(0, 120, 90), text=(0, 120, 90), accent=(255, 180, 0)),
    'dnata': dict(label='dnata', sub='CATERING', bg=(255, 255, 255), band=(0, 94, 184), text=(0, 94, 184), accent=(230, 120, 0)),
    'newrest': dict(label='NEWREST', sub='ONBOARD CATERING', bg=(255, 255, 255), band=(0, 160, 160), text=(0, 130, 130), accent=(240, 90, 40)),
    'doco': dict(label='DO & CO', sub='AIRLINE CATERING', bg=(250, 248, 240), band=(20, 20, 20), text=(20, 20, 20), accent=(190, 150, 60)),
}


def draw_liveries(out_dir=LIVERY_DIR):
    from PIL import Image, ImageDraw, ImageFont
    os.makedirs(out_dir, exist_ok=True)
    paths = {}
    try:
        big = ImageFont.truetype('arialbd.ttf', 220)
        small = ImageFont.truetype('arialbd.ttf', 70)
    except OSError:
        big = small = ImageFont.load_default()
    for key, s in OPERATORS.items():
        w, h = 2048, 960
        img = Image.new('RGBA', (w, h), s['bg'] + (255,))
        d = ImageDraw.Draw(img)
        d.rectangle([0, h - 230, w, h], fill=s['band'])
        d.rectangle([0, h - 250, w, h - 230], fill=s['accent'])
        d.polygon([(w - 700, 0), (w - 380, 0), (w - 980, h - 250), (w - 1300, h - 250)], fill=s['accent'])
        d.text((130, 250), s['label'], font=big, fill=s['text'])
        d.text((140, 520), s['sub'], font=small, fill=s['text'])
        path = os.path.join(out_dir, f'{key}.png')
        img.save(path)
        paths[key] = path
    return paths


def liveries(out):
    return draw_liveries(os.path.join(out, 'liveries'))


def materials():
    return {
        'white': g.mat('gse_white', (236, 237, 233), roughness=0.35),
        'box': g.mat('gse_box_white', (240, 240, 238), roughness=0.3),
        'cab': g.mat('gse_cab', (230, 230, 228), roughness=0.35),
        'chassis': g.mat('gse_chassis', (40, 42, 45), metallic=0.5, roughness=0.5),
        'steel': g.mat('gse_steel', (150, 153, 156), metallic=0.85, roughness=0.4),
        'yellow': g.mat('gse_yellow', (245, 200, 0), roughness=0.4),
        'black': g.mat('gse_black', (22, 22, 23), roughness=0.55),
        'rubber': g.mat('gse_rubber', (28, 28, 28), roughness=0.88),
        'rim': g.mat('gse_rim', (170, 172, 170), metallic=0.7, roughness=0.4),
        'glass': g.mat('gse_glass', (30, 45, 55), roughness=0.05),
        'amber': g.mat('gse_amber_lens', (255, 140, 10), roughness=0.12),
        'red': g.mat('gse_red', (185, 18, 20), roughness=0.3),
        'floor': g.mat('gse_deck_floor', (90, 92, 95), metallic=0.4, roughness=0.7),
        'seal': g.mat('gse_seal', (15, 15, 15), roughness=0.9),
    }


def chassis(f, m, c):
    for k in (-1, 1):
        g.box(f, f'frame_rail_{k}', 0, LENGTH - 300, k * 480 - 90, k * 480 + 90, 700, CHASSIS_TOP, m['chassis'], c, bevel=12)
    for i, x in enumerate((300, 2000, 3700, 5300, 6900)):
        g.box(f, f'cross_member_{i}', x - 60, x + 60, -480, 480, 780, 1000, m['chassis'], c, bevel=8)
    g.box(f, 'rear_bumper', -60, 40, -1100, 1100, 620, 900, m['yellow'], c, bevel=10)
    g.box(f, 'subframe', BOX_X0 - 40, BOX_X1 + 40, -1150, 1150, CHASSIS_TOP, CHASSIS_TOP + 60, m['steel'], c, bevel=10)
    for k in (-1, 1):
        g.box(f, f'stabiliser_housing_{k}', 3000, 3300, k * 1250, k * 1560, 1050, 1250, m['chassis'], c, bevel=8)
        g.cylinder(f, f'stabiliser_leg_{k}', (3150, k * 1480, 700), 'z', 70, 560, m['steel'], c, segments=16, bevel=4)
        g.cylinder(f, f'stabiliser_foot_{k}', (3150, k * 1480, 120), 'z', 170, 40, m['black'], c, segments=20, bevel=6)
        g.box(f, f'side_marker_{k}', 3600, 3800, k * (HY - 80), k * (HY - 60), 800, 860, m['amber'], c)
        g.box(f, f'fuel_tank_{k}', 3850, 5000, k * 1000, k * 1200, 650, 1050, m['steel'], c, bevel=20)
    for x in AXLES:
        g.cylinder(f, f'axle_{int(x)}', (x, 0, TYRE_D / 2), 'y', 70, TRACK, m['chassis'], c, segments=16)
    for x in (1950, 6500):
        for k in (-1, 1):
            g.box(f, f'mudguard_{int(x)}_{k}', x - 650 if x < 3000 else x - 520, x + 650 if x < 3000 else x + 520, k * (TRACK / 2 + 190), k * (TRACK / 2 - 190), 880, 900, m['chassis'], c, bevel=3)


def wheels(f, m, c):
    r = TYRE_D / 2
    profile = [(190, -TYRE_W / 2 + 25), (210, -TYRE_W / 2), (r - 40, -TYRE_W / 2), (r - 8, -TYRE_W / 2 + 25), (r, -TYRE_W / 2 + 70),
               (r, TYRE_W / 2 - 70), (r - 8, TYRE_W / 2 - 25), (r - 40, TYRE_W / 2), (210, TYRE_W / 2), (190, TYRE_W / 2 - 25)]
    for x in AXLES:
        for k in (-1, 1):
            y = k * TRACK / 2
            tag = f'{int(x)}_{"L" if k < 0 else "R"}'
            g.lathe(f, f'tyre_{tag}', (x, y, r), (k, 0, 0), profile, m['rubber'], c, segments=40)
            g.lathe(f, f'rim_{tag}', (x, y, r), (k, 0, 0), [(0.0, 40.0), (120.0, 40.0), (200.0, 30.0), (240.0, 20.0), (250.0, 80.0)], m['rim'], c, segments=32)
            for b in range(8):
                a = 2 * math.pi * b / 8
                g.cylinder(f, f'lug_{tag}_{b}', (x + 90 * math.cos(a), y + k * 48, r + 90 * math.sin(a)), 'y', 14, 22, m['steel'], c, segments=8)


def cab(f, m, c):
    x0, x1 = 5700.0, LENGTH
    g.prism_xz(f, 'cab_shell', [(x0, 1000), (x0, 3050), (7350, 3050), (7600, 2900), (x1, 2600), (x1, 1000)], -HY + 60, HY - 60, m['cab'], c, bevel=30)
    g.prism_xz(f, 'windscreen', [(7620, 1850), (7640, 2850), (x1 - 40, 2560), (x1 - 40, 1850)], -HY + 190, HY - 190, m['glass'], c, bevel=5)
    for k in (-1, 1):
        y = k * (HY - 55)
        g.box(f, f'door_window_{k}', 6350, 7250, y - k * 5, y + k * 10, 1850, 2700, m['glass'], c, bevel=8)
        g.box(f, f'door_line_{k}', 6300, 7300, y - k * 3, y + k * 5, 1050, 1060, m['black'], c)
        g.box(f, f'door_step_{k}', 6300, 7300, k * (HY - 20), k * (HY + 180), 980, 1010, m['steel'], c, bevel=4)
        g.box(f, f'mirror_arm_{k}', 7500, 7560, k * (HY - 40), k * (HY + 280), 2300, 2330, m['black'], c)
        g.box(f, f'mirror_{k}', 7480, 7580, k * (HY + 240), k * (HY + 320), 2000, 2500, m['black'], c, bevel=12)
        g.cylinder(f, f'headlight_{k}', (LENGTH - 5, k * 900, 1350), 'x', 90, 30, m['white'], c, segments=20)
        g.box(f, f'indicator_{k}', LENGTH - 10, LENGTH, k * 1050, k * 1150, 1250, 1450, m['amber'], c)
    g.box(f, 'front_grille', LENGTH - 8, LENGTH + 10, -700, 700, 1150, 1700, m['black'], c, bevel=8)
    g.box(f, 'front_bumper', LENGTH - 60, LENGTH + 80, -1150, 1150, 600, 950, m['yellow'], c, bevel=15)
    g.box(f, 'cab_roof_light_bar', 7000, 7150, -600, 600, 3050, 3110, m['black'], c, bevel=6)
    g.cylinder(f, 'beacon_base', (6200, 0, 3070), 'z', 80, 30, m['black'], c, segments=20)
    g.cylinder(f, 'beacon_lens', (6200, 0, 3130), 'z', 70, 90, m['amber'], c, segments=20, bevel=8)


ARM_LENGTH = 4000.0
ARM_REST_DEG = 5.0
ARM_RAISED_DEG = math.degrees(math.asin((SILL_HEIGHT - 1200.0) / ARM_LENGTH))
ARM_PIVOT_Z = 1200.0
ARM_REAR_X = 500.0
ARM_FRONT_X = ARM_REAR_X + ARM_LENGTH * math.cos(math.radians(ARM_REST_DEG))


def arm_step(f, pivot, sign):
    px, py, pz = f.v(*pivot)
    return {'kind': 'rotate', 'pivot': (px, py, pz), 'axis': (1.0, 0.0, 0.0), 'deg0': 0.0, 'deg1': sign * (ARM_RAISED_DEG - ARM_REST_DEG), 'v0': 0.0, 'v1': 1.0, 'dataref': LIFT_DATAREF}


def scissor_arm(f, m, c, name, pivot, sign, y0, y1):
    th = math.radians(ARM_REST_DEG)
    px, pz = pivot
    dx, dz = sign * -1 * 0 + math.cos(th) * (1 if sign > 0 else -1), math.sin(th)
    nx, nz = -dz, dx
    w = 90.0
    pts = [(px - nx * w, pz - nz * w), (px + nx * w, pz + nz * w), (px + dx * ARM_LENGTH + nx * w, pz + dz * ARM_LENGTH + nz * w), (px + dx * ARM_LENGTH - nx * w, pz + dz * ARM_LENGTH - nz * w)]
    arm = g.prism_xz(f, name, pts, y0, y1, m['steel'], c, bevel=6)
    g.set_anim(arm, [arm_step(f, (px, (y0 + y1) / 2, pz), sign)])
    return arm


def columns(f, m, c):
    for k in (-1, 1):
        for tag, (y0, y1) in (('in', (640, 760)), ('out', (840, 960))):
            ya, yb = sorted((k * y0, k * y1))
            if tag == 'in':
                scissor_arm(f, m, c, f'scissor_arm_rise_{k}', (ARM_REAR_X, ARM_PIVOT_Z), 1, ya, yb)
            else:
                scissor_arm(f, m, c, f'scissor_arm_fall_{k}', (ARM_FRONT_X, ARM_PIVOT_Z), -1, ya, yb)
        for x in (ARM_REAR_X, ARM_FRONT_X):
            g.cylinder(f, f'scissor_base_pin_{k}_{int(x)}', (x, k * 800, ARM_PIVOT_Z), 'y', 45, 420, m['black'], c, segments=14)
            g.box(f, f'scissor_base_bracket_{k}_{int(x)}', x - 130, x + 130, k * 560, k * 1040, CHASSIS_TOP + 40, ARM_PIVOT_Z - 60, m['chassis'], c, bevel=6)
    for x in (ARM_REAR_X, ARM_FRONT_X):
        g.cylinder(f, f'scissor_base_tube_{int(x)}', (x, 0, ARM_PIVOT_Z), 'y', 40, 2000, m['steel'], c, segments=14)
    for i, x in enumerate((1800.0, 3000.0)):
        for k in (-1, 1):
            g.cylinder(f, f'ram_static_{i}_{k}', (x, k * 540, 1450), 'y', 50, 100, m['black'], c, segments=14)


def box_body(f, m, c):
    parts = []
    zt = BOX_FLOOR + BOX_H
    parts.append(g.box(f, 'box_shell', BOX_X0, BOX_X1, -BOX_W / 2, BOX_W / 2, BOX_FLOOR, zt, m['box'], c, bevel=35, segments=3))
    parts.append(g.box(f, 'box_floor_frame', BOX_X0 - 40, BOX_X1 + 40, -BOX_W / 2 - 40, BOX_W / 2 + 40, BOX_FLOOR - 160, BOX_FLOOR, m['steel'], c, bevel=8))
    parts.append(g.box(f, 'box_roof_cap', BOX_X0 + 40, BOX_X1 - 40, -BOX_W / 2 + 60, BOX_W / 2 - 60, zt, zt + 40, m['white'], c, bevel=15))
    for k in (-1, 1):
        y = k * BOX_W / 2
        parts.append(g.box(f, f'bumper_strip_{k}', BOX_X0 + 20, BOX_X1 - 20, y - k * 5, y + k * 20, BOX_FLOOR + 80, BOX_FLOOR + 160, m['black'], c, bevel=4))
        parts.append(g.box(f, f'side_marker_box_{k}', BOX_X1 - 120, BOX_X1 - 20, y, y + k * 10, BOX_FLOOR + 400, BOX_FLOOR + 460, m['amber'], c))
        parts.append(g.box(f, f'hinge_rail_{k}', BOX_X0 + 20, BOX_X1 - 20, y - k * 5, y + k * 12, zt - 160, zt - 130, m['steel'], c))
        parts.append(g.box(f, f'rear_light_{k}', BOX_X0 - 12, BOX_X0, k * 1090, k * 1170, BOX_FLOOR + 100, BOX_FLOOR + 400, m['red'], c))
    parts.append(g.box(f, 'rear_door_roller', BOX_X0 - 12, BOX_X0, -1000, 1000, BOX_FLOOR + 150, zt - 150, m['white'], c, bevel=4))
    for i in range(10):
        z = BOX_FLOOR + 220 + i * 190
        parts.append(g.box(f, f'rear_door_rib_{i}', BOX_X0 - 16, BOX_X0 - 10, -980, 980, z, z + 20, m['steel'], c))
    parts.append(g.box(f, 'rear_door_handle', BOX_X0 - 40, BOX_X0 - 12, -150, 150, BOX_FLOOR + 800, BOX_FLOOR + 860, m['black'], c, bevel=6))
    parts.append(g.box(f, 'door_frame_front', BOX_X1, BOX_X1 + 60, -1050, 1050, BOX_FLOOR + 60, BOX_FLOOR + 2150, m['seal'], c, bevel=10))
    parts.append(g.box(f, 'door_opening_front', BOX_X1 + 20, BOX_X1 + 62, -950, 950, BOX_FLOOR + 140, BOX_FLOOR + 2060, m['black'], c, bevel=6))
    for p in parts:
        g.set_anim(p, [lift_step(1.0)])
    return parts


def bridge(f, m, c):
    z0 = BOX_FLOOR
    parts = []
    x0, x1 = BOX_X1 + 80, BOX_X1 + 80 + 1250
    parts.append(g.box(f, 'bridge_deck', x0, x1, -940, 940, z0 - 60, z0 + 20, m['floor'], c, bevel=5))
    parts.append(g.box(f, 'bridge_apron', x1 - 20, x1 + 120, -940, 940, z0 - 50, z0 + 20, m['yellow'], c, bevel=5))
    parts.append(g.box(f, 'bridge_seal', x1 + 100, x1 + 220, -960, 960, z0 - 100, z0 + 700, m['seal'], c, bevel=20))
    for k in (-1, 1):
        y = k * 940
        parts.append(g.box(f, f'bridge_side_{k}', x0, x1, y - k * 20, y + k * 15, z0 - 60, z0 + 120, m['steel'], c, bevel=4))
        parts.append(g.tube(f, f'bridge_rail_{k}', [(x0, y, z0 + 1000), (x1, y, z0 + 1000)], 22, m['yellow'], c, segments=12))
        parts.append(g.tube(f, f'bridge_rail_low_{k}', [(x0, y, z0 + 550), (x1, y, z0 + 550)], 18, m['yellow'], c, segments=12))
        for i in range(5):
            x = x0 + 40 + i * (x1 - x0 - 80) / 4
            parts.append(g.cylinder(f, f'bridge_post_{k}_{i}', (x, y, z0 + 500), 'z', 20, 1000, m['yellow'], c, segments=10))
    for p in parts:
        g.set_anim(p, [lift_step(1.0), BRIDGE_STEP])
    return parts


def decals(f, c, operator):
    path = os.path.join(LIVERY_DIR, f'{operator}.png')
    if not os.path.exists(path):
        draw_liveries()
    mat = g.image_mat(f'decal_{operator}', path, roughness=0.4)
    zb, zt = BOX_FLOOR + 150, BOX_FLOOR + BOX_H - 100
    off = BOX_W / 2 + 3
    x0, x1 = BOX_X0 + 100, BOX_X1 - 100
    for k in (-1, 1):
        if k > 0:
            corners = [(x0, k * off, zb), (x1, k * off, zb), (x1, k * off, zt), (x0, k * off, zt)]
        else:
            corners = [(x1, k * off, zb), (x0, k * off, zb), (x0, k * off, zt), (x1, k * off, zt)]
        d = g.decal(f, f'livery_side_{k}', corners, mat, c)
        g.set_anim(d, [lift_step(1.0)])


def pose_raised(coll):
    from mathutils import Matrix, Vector
    for o in coll.objects:
        if 'gse_anim' not in o:
            continue
        for st in json.loads(o['gse_anim']):
            if st['kind'] == 'trans':
                o.matrix_world = Matrix.Translation(Vector(st['offset1'])) @ o.matrix_world
            else:
                pv = Vector(st['pivot'])
                rot = Matrix.Rotation(math.radians(st['deg1']), 4, Vector(st['axis']))
                o.matrix_world = Matrix.Translation(pv) @ rot @ Matrix.Translation(-pv) @ o.matrix_world


def build(with_connection=True):
    g.reset_scene()
    f = g.Frame(LENGTH / 2)
    c = g.collection(NAME)
    m = materials()
    chassis(f, m, c)
    wheels(f, m, c)
    cab(f, m, c)
    columns(f, m, c)
    box_body(f, m, c)
    bridge(f, m, c)
    decals(f, c, DEFAULT_OPERATOR)
    if os.environ.get('CAT_POSE') == 'raised':
        pose_raised(c)
    return c


VIEWS = {
    'raised': dict(res=(800, 600), lens=30, loc=(-9.0, 11.0, 5.0), target=(0.0, 0.5, 3.5), samples=16),
    'side_right': dict(res=(800, 400), lens=35, loc=(14.0, 0.0, 3.0), target=(0.0, 0.0, 2.6), samples=16),
    'three_quarter_lifted': dict(res=(800, 600), lens=30, loc=(-9.0, 10.0, 6.5), target=(0.0, 0.5, 2.8), samples=16),
    'three_quarter_rear': dict(res=(800, 600), lens=30, loc=(8.0, -9.5, 4.5), target=(0.0, 0.0, 2.4), samples=16),
}
