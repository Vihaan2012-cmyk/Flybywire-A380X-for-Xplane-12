import math
import os

import bpy

import gselib as g

NAME = 'clean_van'
L = 6967.0
W = 2020.0
HY = W / 2
H = 2650.0
WHEELBASE = 4325.0
FRONT_AXLE = 5990.0
REAR_AXLE = FRONT_AXLE - WHEELBASE
TRACK = 1710.0
TYRE_D = 700.0
TYRE_W = 225.0
SILL = 420.0
DOOR_X0, DOOR_X1 = 2750.0, 4150.0
DOOR_Z0, DOOR_Z1 = 520.0, 2330.0
CAB_DOOR_X0, CAB_DOOR_X1 = 4950.0, 5650.0
LIVERY_DIR = os.path.join('E:/fbw-gse/out', NAME, 'liveries')
DOOR_ANIM = [{'kind': 'trans', 'offset0': (0.0, 0.0, 0.0), 'offset1': (0.0, -1.2, 0.0), 'v0': 0.0, 'v1': 1.0, 'dataref': 'fbw/gse/clean_van/door'}]

OPERATORS = {
    'dnata': ((0, 60, 140), 'dnata'),
    'swissport': ((200, 20, 40), 'SWISSPORT'),
    'menzies': ((20, 70, 150), 'MENZIES'),
}


def materials():
    return {
        'body': g.mat('gse_van_body', (238, 239, 236), roughness=0.3),
        'black': g.mat('gse_van_black', (22, 22, 23), roughness=0.55),
        'trim': g.mat('gse_van_trim', (50, 52, 54), roughness=0.5),
        'glass': g.mat('gse_van_glass', (24, 32, 38), roughness=0.05),
        'rubber': g.mat('gse_van_rubber', (26, 26, 26), roughness=0.88),
        'rim': g.mat('gse_van_rim', (150, 152, 150), metallic=0.8, roughness=0.4),
        'amber': g.mat('gse_van_amber', (255, 140, 10), roughness=0.12),
        'red': g.mat('gse_van_red', (185, 18, 20), roughness=0.3),
        'white_lamp': g.mat('gse_van_lamp', (250, 250, 245), roughness=0.1),
        'yellow': g.mat('gse_van_yellow', (245, 200, 0), roughness=0.35),
        'steel': g.mat('gse_van_steel', (160, 162, 164), metallic=0.85, roughness=0.4),
    }


def side_profile():
    return [(0, SILL), (0, H - 60), (60, H), (4950, H), (5150, H - 40), (5750, 1720), (5900, 1600), (6750, 1420),
            (6930, 1300), (L, 1150), (L, 520), (L - 90, 430), (L - 200, SILL)]


def body(f, m, c):
    parts = []
    parts.append(g.prism_xz(f, 'body_shell', side_profile(), -HY, HY, m['body'], c, bevel=22, segments=2))
    parts.append(g.box(f, 'rear_bumper', -40, 120, -HY + 40, HY - 40, 330, 560, m['black'], c, bevel=18))
    parts.append(g.box(f, 'front_bumper', L - 130, L + 25, -HY + 30, HY - 30, 330, 640, m['black'], c, bevel=20))
    parts.append(g.box(f, 'front_grille', L - 3, L + 3, -420, 420, 800, 1020, m['black'], c, bevel=8))
    parts.append(g.box(f, 'under_tray', 120, L - 200, -HY + 100, HY - 100, 330, SILL + 20, m['black'], c))
    for k, name in ((-1, 'L'), (1, 'R')):
        parts.append(g.box(f, f'sill_step_{name}', 2600, 4300, k * (HY - 5), k * (HY + 70), 330, 380, m['steel'], c, bevel=6))
        parts.append(g.box(f, f'lower_trim_{name}', 40, L - 220, k * (HY - 2), k * (HY + 8), SILL + 20, 640, m['trim'], c, bevel=4))
        parts.append(g.box(f, f'mirror_arm_{name}', 5230, 5340, k * (HY - 10), k * (HY + 160), 1700, 1740, m['black'], c, bevel=5))
        parts.append(g.box(f, f'mirror_{name}', 5200, 5380, k * (HY + 150), k * (HY + 230), 1580, 1960, m['black'], c, bevel=24))
        parts.append(g.box(f, f'tail_lamp_{name}', -6, 10, k * (HY - 260), k * (HY - 60), 700, 1250, m['red'], c, bevel=6))
        parts.append(g.box(f, f'head_lamp_{name}', L - 6, L + 8, k * 560, k * 960, 1000, 1160, m['white_lamp'], c, bevel=10))
        parts.append(g.box(f, f'indicator_{name}', L - 4, L + 6, k * 960, k * (HY - 30), 1000, 1130, m['amber'], c, bevel=6))
        parts.append(g.box(f, f'cab_door_window_{name}', CAB_DOOR_X0 + 120, CAB_DOOR_X1 - 40, k * (HY + 1), k * (HY + 6), 1550, 2150, m['glass'], c, bevel=10))
        parts.append(g.box(f, f'cab_door_gap_front_{name}', CAB_DOOR_X1, CAB_DOOR_X1 + 8, k * (HY + 1), k * (HY + 5), 480, 1900, m['black'], c))
        parts.append(g.box(f, f'cab_door_gap_rear_{name}', CAB_DOOR_X0 - 8, CAB_DOOR_X0, k * (HY + 1), k * (HY + 5), 480, 2330, m['black'], c))
        parts.append(g.box(f, f'cab_door_handle_{name}', CAB_DOOR_X0 + 40, CAB_DOOR_X0 + 190, k * (HY + 1), k * (HY + 18), 1330, 1370, m['black'], c, bevel=6))
    parts.append(g.box(f, 'rear_door_gap', -2, 4, -6, 6, 480, H - 120, m['black'], c))
    parts.append(g.box(f, 'rear_window_L', -2, 6, -880, -60, 1500, 2250, m['glass'], c, bevel=12))
    parts.append(g.box(f, 'rear_window_R', -2, 6, 60, 880, 1500, 2250, m['glass'], c, bevel=12))
    parts.append(g.prism_xz(f, 'windscreen', [(5170, 2540), (5700, 1790), (5720, 1795), (5190, 2548)], -HY + 150, HY - 150, m['glass'], c, bevel=3))
    parts.append(g.box(f, 'roof_beacon_base', 5000, 5260, -80, 80, H, H + 30, m['black'], c, bevel=6))
    parts.append(g.cylinder(f, 'roof_beacon', (5130, 0, H + 95), 'z', 85, 130, m['amber'], c, segments=24, bevel=4))
    for i, x in enumerate((900, 2000, 3100, 4200)):
        parts.append(g.box(f, f'roof_rib_{i}', x - 40, x + 40, -HY + 100, HY - 100, H, H + 8, m['trim'], c, bevel=2))
    for k, name in ((-1, 'L'), (1, 'R')):
        parts.append(g.box(f, f'wheel_arch_{name}_rear', REAR_AXLE - 480, REAR_AXLE + 480, k * (HY - 6), k * (HY + 18), 380, 640, m['black'], c, bevel=10))
        parts.append(g.box(f, f'wheel_arch_{name}_front', FRONT_AXLE - 480, FRONT_AXLE + 480, k * (HY - 6), k * (HY + 18), 380, 640, m['black'], c, bevel=10))
    return parts


def sliding_door(f, m, c):
    y_out = HY + 6
    panel = g.box(f, 'sliding_door', DOOR_X0, DOOR_X1, y_out, y_out + 14, DOOR_Z0, DOOR_Z1, m['body'], c, bevel=6)
    g.set_anim(panel, DOOR_ANIM)
    parts = [panel]
    for name, x0, x1, z0, z1, mat in (
        ('sliding_door_window', DOOR_X0 + 180, DOOR_X1 - 180, 1500, 2150, 'glass'),
        ('sliding_door_handle', DOOR_X1 - 220, DOOR_X1 - 70, 1300, 1340, 'black'),
        ('sliding_door_lower_trim', DOOR_X0 + 4, DOOR_X1 - 4, DOOR_Z0, 700, 'trim'),
    ):
        part = g.box(f, name, x0, x1, y_out + 12, y_out + 20, z0, z1, m[mat], c, bevel=4)
        g.set_anim(part, DOOR_ANIM)
        parts.append(part)
    for i, (x0, x1) in enumerate(((DOOR_X0 - 60, DOOR_X0 + 30), (DOOR_X1 - 30, DOOR_X1 + 60))):
        parts.append(g.box(f, f'door_track_post_{i}', x0, x1, HY + 1, HY + 10, DOOR_Z0 - 30, DOOR_Z1 + 20, m['black'], c))
    parts.append(g.box(f, 'door_rail_lower', DOOR_X0 - 1200, DOOR_X1 + 60, HY + 1, HY + 24, 470, 520, m['black'], c, bevel=4))
    parts.append(g.box(f, 'door_rail_upper', DOOR_X0 - 60, DOOR_X1 + 60, HY + 1, HY + 22, DOOR_Z1 + 10, DOOR_Z1 + 60, m['black'], c, bevel=4))
    return parts


def wheels(f, m, c):
    parts = []
    r = TYRE_D / 2
    half = TYRE_W / 2
    tyre_profile = [(r - 150, -half + 14), (r - 120, -half), (r - 25, -half + 4), (r, -half + 40), (r, half - 40), (r - 25, half - 4), (r - 120, half), (r - 150, half - 14)]
    rim_profile = [(0.0, 60.0), (70.0, 60.0), (90.0, 40.0), (r - 150, 30.0), (r - 148, 80.0)]
    for x, label in ((REAR_AXLE, 'rear'), (FRONT_AXLE, 'front')):
        for k, side in ((-1, 'L'), (1, 'R')):
            y = k * TRACK / 2
            name = f'{label}_{side}'
            parts.append(g.lathe(f, f'tyre_{name}', (x, y, r), (k, 0, 0), tyre_profile, m['rubber'], c, segments=40))
            parts.append(g.lathe(f, f'rim_{name}', (x, y, r), (k, 0, 0), [(rad, -z) for rad, z in rim_profile], m['rim'], c, segments=32))
            parts.append(g.cylinder(f, f'hubcap_{name}', (x, y + k * 55, r), 'y', 55, 20, m['black'], c, segments=20, bevel=3))
            for b in range(5):
                a = 2 * math.pi * b / 5
                parts.append(g.cylinder(f, f'lug_{name}_{b}', (x + 90 * math.cos(a), y + k * 62, r + 90 * math.sin(a)), 'y', 12, 14, m['steel'], c, segments=8))
    parts.append(g.cylinder(f, 'axle_rear', (REAR_AXLE, 0, r), 'y', 45, TRACK - 200, m['black'], c, segments=16))
    parts.append(g.cylinder(f, 'axle_front', (FRONT_AXLE, 0, r), 'y', 40, TRACK - 200, m['black'], c, segments=16))
    return parts


def fittings(f, m, c):
    parts = []
    parts.append(g.box(f, 'number_plate_rear', -8, 0, -260, 260, 560, 710, m['yellow'], c, bevel=3))
    parts.append(g.box(f, 'number_plate_front', L, L + 8, -260, 260, 560, 710, m['yellow'], c, bevel=3))
    parts.append(g.box(f, 'rear_step', -90, 0, -700, 700, 300, 340, m['steel'], c, bevel=5))
    parts.append(g.box(f, 'wiper_L', 5700, 5740, -820, -80, 1730, 1745, m['black'], c))
    parts.append(g.box(f, 'wiper_R', 5700, 5740, 80, 820, 1730, 1745, m['black'], c))
    parts.append(g.box(f, 'tow_eye_front', L, L + 60, -30, 30, 380, 420, m['steel'], c, bevel=3))
    parts.append(g.box(f, 'fuel_flap', 3700, 3860, -HY - 2, -HY + 6, 900, 1060, m['trim'], c, bevel=3))
    parts.append(g.box(f, 'seat_dummy', 5150, 5650, -800, 800, 900, 1250, m['trim'], c, bevel=20))
    parts.append(g.box(f, 'cargo_bulkhead', 4850, 4880, -HY + 60, HY - 60, 520, H - 160, m['trim'], c))
    return parts


def default_livery_path():
    return os.path.join(LIVERY_DIR, 'dnata_side.png')


def decals(f, c):
    path = default_livery_path()
    if not os.path.exists(path):
        return
    mat = g.image_mat('gse_van_decal', path, roughness=0.35)
    z = HY + 2.0
    g.decal(f, 'livery_left', [(4750, -z, 700), (400, -z, 700), (400, -z, 2300), (4750, -z, 2300)], mat, c)
    g.decal(f, 'livery_right', [(450, z, 1250), (2650, z, 1250), (2650, z, 2300), (450, z, 2300)], mat, c)


def build(with_connection=True):
    g.reset_scene()
    f = g.Frame(0.0)
    c = g.collection(NAME)
    m = materials()
    body(f, m, c)
    sliding_door(f, m, c)
    wheels(f, m, c)
    fittings(f, m, c)
    decals(f, c)
    return c


def liveries(out):
    from PIL import Image, ImageDraw, ImageFont
    folder = os.path.join(out, 'liveries')
    os.makedirs(folder, exist_ok=True)
    result = {}
    for operator, (colour, text) in OPERATORS.items():
        img = Image.new('RGBA', (1024, 512), (0, 0, 0, 0))
        d = ImageDraw.Draw(img)
        d.rectangle([0, 380, 1023, 450], fill=colour + (255,))
        try:
            big = ImageFont.truetype('arialbd.ttf', 150)
            small = ImageFont.truetype('arialbd.ttf', 48)
        except OSError:
            big = small = ImageFont.load_default()
        d.text((40, 120), text, fill=colour + (255,), font=big)
        d.text((44, 310), 'AIRCRAFT CABIN CLEANING', fill=colour + (255,), font=small)
        path = os.path.join(folder, f'{operator}_side.png')
        img.save(path)
        result[operator] = path
    return result


VIEWS = {
    'three_quarter_door': dict(res=(800, 560), lens=35, samples=16, loc=(8.5, 9.5, 3.4), target=(0.0, 3.2, 1.2)),
    'three_quarter_front': dict(res=(800, 560), lens=35, samples=16, loc=(-6.5, -2.5, 3.2), target=(0.0, 3.2, 1.2)),
    'side_right': dict(res=(800, 400), ortho=9.0, samples=16, loc=(14.0, 3.48, 1.3), target=(0.0, 3.48, 1.3)),
}
