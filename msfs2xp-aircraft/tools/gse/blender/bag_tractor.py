import math
import os

import bpy

import gselib as g

NAME = 'bag_tractor'
LIVERY_DIR = 'E:/fbw-gse/out/bag_tractor/liveries'
HALF_WIDTH = 625.0
REAR_AXLE = 520.0
FRONT_AXLE = 2220.0
TRACK = 1050.0
TYRE_D = 640.0
TYRE_W = 210.0
FLOOR_Z = 330.0
BONNET_TOP = 1100.0
ROOF_Z = 1950.0
CAB_X0 = 450.0
CAB_X1 = 1500.0
HITCH_Z = 430.0
PIN_RADIUS = 35.0


def span(a, b):
    return min(a, b), max(a, b)


def materials():
    return {
        'body': g.mat('gse_bt_body', (225, 225, 222), roughness=0.4),
        'accent': g.mat('gse_bt_accent', (200, 30, 35), roughness=0.4),
        'steel': g.mat('gse_bt_steel', (90, 92, 95), metallic=0.8, roughness=0.45),
        'black': g.mat('gse_bt_black', (22, 22, 23), roughness=0.6),
        'rubber': g.mat('gse_bt_rubber', (28, 28, 28), roughness=0.88),
        'rim': g.mat('gse_bt_rim', (170, 172, 170), metallic=0.6, roughness=0.5),
        'seat': g.mat('gse_bt_seat', (35, 38, 42), roughness=0.8),
        'glass': g.mat('gse_bt_glass', (150, 190, 205), roughness=0.05, alpha=0.3),
        'amber': g.mat('gse_bt_amber', (255, 140, 10), roughness=0.12),
        'yellow': g.mat('gse_bt_yellow', (245, 200, 0), roughness=0.35),
        'headlight': g.mat('gse_bt_headlight', (250, 250, 235), roughness=0.1),
        'red': g.mat('gse_bt_red', (185, 18, 20), roughness=0.3),
    }


def chassis(f, m, c):
    parts = []
    parts.append(g.box(f, 'frame_rail_L', 60, 2700, -380, -300, 230, 330, m['steel'], c, bevel=5))
    parts.append(g.box(f, 'frame_rail_R', 60, 2700, 300, 380, 230, 330, m['steel'], c, bevel=5))
    parts.append(g.box(f, 'floor_plate', 0, 1500, -HALF_WIDTH, HALF_WIDTH, FLOOR_Z, FLOOR_Z + 30, m['body'], c, bevel=6))
    parts.append(g.box(f, 'rear_deck', 0, 450, -HALF_WIDTH, HALF_WIDTH, FLOOR_Z, 560, m['body'], c, bevel=12))
    parts.append(g.box(f, 'rear_bumper', -40, 40, -HALF_WIDTH, HALF_WIDTH, 300, 480, m['black'], c, bevel=10))
    parts.append(g.box(f, 'hitch_plate', 40, 330, -140, 140, HITCH_Z - 60, HITCH_Z - 20, m['steel'], c, bevel=5))
    parts.append(g.box(f, 'hitch_jaw_top', 40, 330, -140, 140, HITCH_Z + 20, HITCH_Z + 60, m['steel'], c, bevel=5))
    parts.append(g.box(f, 'hitch_jaw_back', 40, 100, -140, 140, HITCH_Z - 60, HITCH_Z + 60, m['steel'], c, bevel=5))
    parts.append(g.cylinder(f, 'hitch_pin', (240, 0, HITCH_Z), 'z', PIN_RADIUS, 200, m['yellow'], c, segments=24, bevel=3))
    parts.append(g.cylinder(f, 'hitch_pin_head', (240, 0, HITCH_Z + 105), 'z', 48, 14, m['steel'], c, segments=24, bevel=3))
    parts.append(g.tube(f, 'hitch_pin_pull_ring', [(240, 0, HITCH_Z + 110), (240, 70, HITCH_Z + 170), (240, 0, HITCH_Z + 230), (240, -70, HITCH_Z + 170), (240, 0, HITCH_Z + 110)], 6, m['steel'], c, segments=10))
    parts.append(g.box(f, 'front_bumper', 2720, 2900, -HALF_WIDTH, HALF_WIDTH, 300, 520, m['black'], c, bevel=14))
    parts.append(g.box(f, 'front_bumper_stripe', 2900, 2904, -HALF_WIDTH + 40, HALF_WIDTH - 40, 380, 430, m['yellow'], c))
    parts.append(g.box(f, 'front_hitch_pad', 2900, 2960, -90, 90, 380, 460, m['steel'], c, bevel=6))
    for k in (-1, 1):
        n = 'L' if k < 0 else 'R'
        ya, yb = span(k * HALF_WIDTH, k * (HALF_WIDTH + 130))
        parts.append(g.box(f, f'step_{n}', 700, 1100, ya, yb, 230, 250, m['steel'], c, bevel=3))
        ya, yb = span(k * (HALF_WIDTH - 12), k * HALF_WIDTH)
        parts.append(g.box(f, f'sideskirt_{n}', 0, 1500, ya, yb, 250, FLOOR_Z, m['body'], c, bevel=3))
    return parts


def wheels(f, m, c):
    parts = []
    r = TYRE_D / 2
    rr = 190.0
    profile = [(rr, -TYRE_W / 2 + 14), (r - 40, -TYRE_W / 2), (r - 10, -TYRE_W / 2 + 12), (r, -TYRE_W / 2 + 40),
               (r, TYRE_W / 2 - 40), (r - 10, TYRE_W / 2 - 12), (r - 40, TYRE_W / 2), (rr, TYRE_W / 2 - 14)]
    rim_profile = [(0.0, 50.0), (70.0, 50.0), (90.0, 36.0), (rr - 4, 32.0), (rr + 4, 78.0)]
    for x, label in ((REAR_AXLE, 'rear'), (FRONT_AXLE, 'front')):
        for k in (-1, 1):
            y = k * TRACK / 2
            n = f'{label}_{"L" if k < 0 else "R"}'
            parts.append(g.lathe(f, f'tyre_{n}', (x, y, r), (k, 0, 0), profile, m['rubber'], c, segments=48))
            parts.append(g.lathe(f, f'rim_{n}', (x, y, r), (k, 0, 0), [(rad, -z) for rad, z in rim_profile], m['rim'], c, segments=36))
            parts.append(g.cylinder(f, f'hubcap_{n}', (x, y + k * 55, r), 'y', 50, 24, m['steel'], c, segments=20, bevel=4))
            for b in range(6):
                a = 2 * math.pi * b / 6
                parts.append(g.cylinder(f, f'lug_{n}_{b}', (x + 80 * math.cos(a), y + k * 52, r + 80 * math.sin(a)), 'y', 9, 16, m['steel'], c, segments=8))
        parts.append(g.cylinder(f, f'axle_{label}', (x, 0, r), 'y', 45, TRACK - 120, m['steel'], c, segments=20))
    parts.append(g.box(f, 'front_axle_beam', FRONT_AXLE - 90, FRONT_AXLE + 90, -TRACK / 2 + 80, TRACK / 2 - 80, r - 55, r + 55, m['steel'], c, bevel=8))
    arch_out = [(REAR_AXLE + 360 * math.cos(math.radians(a)), r + 360 * math.sin(math.radians(a))) for a in range(20, 161, 10)]
    arch_in = [(REAR_AXLE + 335 * math.cos(math.radians(a)), r + 335 * math.sin(math.radians(a))) for a in range(160, 19, -10)]
    for k in (-1, 1):
        n = 'L' if k < 0 else 'R'
        ya, yb = span(k * (TRACK / 2 - TYRE_W / 2 - 10), k * (TRACK / 2 + TYRE_W / 2 + 40))
        parts.append(g.prism_xz(f, f'rear_fender_{n}', arch_out + arch_in, ya, yb, m['body'], c, bevel=3))
        ya, yb = span(k * 480, k * 560)
        parts.append(g.box(f, f'rear_light_{n}', -6, 0, ya, yb, 420, 500, m['red'], c, bevel=2))
    return parts


def engine_bay(f, m, c):
    parts = []
    hood = [(1500.0, FLOOR_Z), (1500.0, BONNET_TOP), (2450.0, BONNET_TOP), (2600.0, 1060.0), (2720.0, 900.0), (2730.0, 420.0), (2720.0, FLOOR_Z)]
    hood_obj = g.prism_xz(f, 'bonnet', hood, -HALF_WIDTH + 20, HALF_WIDTH - 20, m['body'], c, bevel=6)
    for poly in hood_obj.data.polygons:
        poly.use_smooth = True
    parts.append(hood_obj)
    parts.append(g.box(f, 'grille', 2724, 2734, -330, 330, 520, 900, m['black'], c, bevel=4))
    for i in range(9):
        parts.append(g.box(f, f'grille_slat_{i}', 2733, 2740, -300, 300, 540 + i * 40, 560 + i * 40, m['steel'], c, bevel=1))
    for k in (-1, 1):
        n = 'L' if k < 0 else 'R'
        ya, yb = span(k * 400, k * 520)
        parts.append(g.box(f, f'headlight_{n}', 2640, 2720, ya, yb, 880, 980, m['headlight'], c, bevel=8))
        parts.append(g.box(f, f'indicator_{n}', 2680, 2740, ya, yb, 780, 830, m['amber'], c, bevel=4))
        ya, yb = span(k * (HALF_WIDTH - 21), k * (HALF_WIDTH - 17))
        parts.append(g.box(f, f'bonnet_stripe_{n}', 1800, 2500, ya, yb, 700, 780, m['accent'], c))
        parts.append(g.box(f, f'bonnet_vent_{n}', 1600, 1700, ya, yb, 850, 1000, m['black'], c, bevel=1))
    parts.append(g.box(f, 'exhaust_stack', 1560, 1620, 480, 540, BONNET_TOP, 1750, m['black'], c, bevel=6))
    parts.append(g.box(f, 'exhaust_cap', 1550, 1630, 470, 550, 1750, 1790, m['steel'], c, bevel=6))
    return parts


def cab(f, m, c):
    parts = []
    for x in (CAB_X0 + 20, CAB_X1 - 20):
        for k in (-1, 1):
            n = 'L' if k < 0 else 'R'
            yc = k * (HALF_WIDTH - 60)
            parts.append(g.box(f, f'cab_post_{n}_{int(x)}', x - 25, x + 25, yc - 25, yc + 25, FLOOR_Z, ROOF_Z, m['black'], c, bevel=4))
    parts.append(g.box(f, 'cab_roof', CAB_X0 - 60, CAB_X1 + 120, -HALF_WIDTH + 20, HALF_WIDTH - 20, ROOF_Z, ROOF_Z + 50, m['body'], c, bevel=14))
    parts.append(g.box(f, 'beacon_base', 900, 1000, -50, 50, ROOF_Z + 50, ROOF_Z + 80, m['black'], c, bevel=3))
    parts.append(g.cylinder(f, 'beacon_lens', (950, 0, ROOF_Z + 120), 'z', 45, 80, m['amber'], c, segments=24, bevel=5))
    parts.append(g.box(f, 'windscreen', CAB_X1 - 14, CAB_X1 - 8, -HALF_WIDTH + 80, HALF_WIDTH - 80, BONNET_TOP + 40, ROOF_Z - 5, m['glass'], c))
    parts.append(g.box(f, 'rear_cab_panel', CAB_X0 + 20, CAB_X0 + 30, -HALF_WIDTH + 80, HALF_WIDTH - 80, 560, 1250, m['glass'], c))
    for k in (-1, 1):
        n = 'L' if k < 0 else 'R'
        yc = k * (HALF_WIDTH - 40)
        parts.append(g.box(f, f'door_bar_{n}', 650, 1300, yc - 8, yc + 8, 800, 830, m['steel'], c, bevel=2))
        parts.append(g.box(f, f'door_bar_low_{n}', 650, 1300, yc - 8, yc + 8, 520, 550, m['steel'], c, bevel=2))
        ya, yb = span(k * (HALF_WIDTH + 20), k * (HALF_WIDTH + 130))
        parts.append(g.box(f, f'mirror_arm_{n}', 1480, 1500, ya, yb, 1450, 1470, m['black'], c, bevel=2))
        ya, yb = span(k * (HALF_WIDTH + 130), k * (HALF_WIDTH + 140))
        parts.append(g.box(f, f'mirror_{n}', 1470, 1500, ya, yb, 1380, 1560, m['black'], c, bevel=2))
    parts.append(g.box(f, 'seat_base', 650, 1050, -230, 230, 440, 760, m['seat'], c, bevel=20))
    parts.append(g.box(f, 'seat_back', 560, 660, -230, 230, 760, 1240, m['seat'], c, bevel=25))
    parts.append(g.box(f, 'seat_headrest', 560, 640, -120, 120, 1240, 1380, m['seat'], c, bevel=25))
    parts.append(g.box(f, 'dash', CAB_X1 - 330, CAB_X1 - 20, -HALF_WIDTH + 60, HALF_WIDTH - 60, 900, 1060, m['black'], c, bevel=18))
    parts.append(g.box(f, 'dash_column', 1230, 1330, -35, 35, 520, 940, m['black'], c, bevel=8))
    parts.append(g.lathe(f, 'steering_wheel', (1260, 0, 1000), (0.8, 0, 0.6), [(165, -10), (185, -10), (185, 10), (165, 10), (165, -10)], m['black'], c, segments=32))
    parts.append(g.cylinder(f, 'steering_hub', (1260, 0, 1000), 'x', 35, 40, m['black'], c, segments=16, bevel=4))
    for i in range(3):
        parts.append(g.box(f, f'pedal_{i}', 1280, 1330, -170 + i * 110, -110 + i * 110, 380, 470, m['black'], c, bevel=3))
    parts.append(g.tube(f, 'gear_lever', [(1100, 330, 450), (1100, 330, 700), (1110, 330, 780)], 14, m['black'], c, segments=10))
    parts.append(g.tube(f, 'hand_brake', [(1020, -330, 470), (1000, -330, 650), (1010, -330, 740)], 14, m['black'], c, segments=10))
    return parts


def decals(f, c):
    out = []
    path = os.path.join(LIVERY_DIR, 'generic_decal.png')
    if not os.path.exists(path):
        return out
    mat = g.image_mat('decal_bag_tractor_generic', path, roughness=0.4)
    x0, x1 = 1800.0, 2450.0
    for k in (-1, 1):
        y = k * (HALF_WIDTH + 1.5)
        corners = [(x0, y, 780), (x1, y, 780), (x1, y, 1080), (x0, y, 1080)] if k > 0 else [(x1, y, 780), (x0, y, 780), (x0, y, 1080), (x1, y, 1080)]
        out.append(g.decal(f, f'd_operator_{k}', corners, mat, c))
    return out


def build(with_connection=True):
    g.reset_scene()
    f = g.Frame(1450.0)
    c = g.collection(NAME)
    m = materials()
    chassis(f, m, c)
    wheels(f, m, c)
    engine_bay(f, m, c)
    cab(f, m, c)
    decals(f, c)
    return c


SCHEMES = {
    'dnata': ((20, 60, 140), (255, 255, 255), 'dnata'),
    'swissport': ((200, 20, 40), (255, 255, 255), 'SWISSPORT'),
    'menzies': ((250, 200, 0), (20, 20, 80), 'MENZIES'),
    'sats': ((0, 110, 80), (255, 255, 255), 'SATS'),
}


def liveries(out):
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        return {}
    os.makedirs(out, exist_ok=True)
    result = {}
    for op, (bg, fg, text) in SCHEMES.items():
        img = Image.new('RGBA', (1024, 512), bg + (255,))
        d = ImageDraw.Draw(img)
        try:
            font = ImageFont.truetype('arialbd.ttf', 170)
        except OSError:
            font = ImageFont.load_default()
        d.rectangle((0, 400, 1024, 450), fill=fg)
        d.text((512, 200), text, fill=fg, font=font, anchor='mm')
        p = os.path.join(out, f'{op}.png')
        img.save(p)
        result[op] = p
    return result


VIEWS = {
    'three_quarter_front': dict(res=(800, 600), lens=35, loc=(3.4, 3.6, 2.3), target=(0.0, 0.2, 0.9), samples=16),
    'three_quarter_rear': dict(res=(800, 600), lens=35, loc=(-3.2, -3.6, 2.2), target=(0.0, -0.2, 0.9), samples=16),
    'side_right': dict(res=(800, 500), lens=35, loc=(5.5, 0.0, 1.0), target=(0.0, 0.0, 0.9), samples=16),
    'rear': dict(res=(640, 600), lens=35, loc=(0.0, -4.5, 1.3), target=(0.0, 0.0, 0.9), samples=16),
}
