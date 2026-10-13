import math

import gselib as g

NAME = 'tow_tpx500'
X0 = 3000.0
HW = 1000.0
TRACK = 2700.0
WHEELBASE = 3600.0
REAR_AXLE = 1300.0
FRONT_AXLE = REAR_AXLE + WHEELBASE
TYRE_D = 1250.0
TYRE_W = 480.0
CHASSIS_Z0 = 330.0
CHASSIS_Z1 = 900.0
CRADLE_LIFT = 0.4
CRADLE_ANIM = [{'kind': 'trans', 'offset0': (0.0, 0.0, 0.0), 'offset1': (0.0, 0.0, CRADLE_LIFT), 'v0': 0.0, 'v1': 1.0, 'dataref': 'fbw/gse/tow/cradle'}]
SCHEMES = {'orange': (232, 98, 12), 'yellow': (245, 196, 0)}


def materials(scheme='orange'):
    return {
        'body': g.mat('gse_body_' + scheme, SCHEMES[scheme], roughness=0.38),
        'white': g.mat('gse_white', (232, 233, 230), roughness=0.35),
        'steel': g.mat('gse_steel', (90, 94, 98), metallic=0.8, roughness=0.45),
        'black': g.mat('gse_black', (22, 22, 23), roughness=0.55),
        'glass': g.mat('gse_cab_glass', (30, 44, 52), roughness=0.06),
        'rubber': g.mat('gse_rubber', (28, 28, 28), roughness=0.88),
        'rim': g.mat('gse_rim', (60, 62, 64), metallic=0.5, roughness=0.5),
        'amber': g.mat('gse_amber_lens', (255, 140, 10), roughness=0.12),
        'red': g.mat('gse_red', (185, 18, 20), roughness=0.3),
        'pad': g.mat('gse_cradle_pad', (40, 40, 42), roughness=0.8),
        'grille': g.mat('gse_grille', (14, 14, 14), metallic=0.3, roughness=0.6),
    }


def span(k, a, b):
    return (min(k * a, k * b), max(k * a, k * b))


def wheels(f, m, c):
    parts = []
    r = TYRE_D / 2
    inner = 330.0
    profile = [(inner, -TYRE_W / 2 + 30), (inner + 20, -TYRE_W / 2), (r - 70, -TYRE_W / 2 + 4), (r - 20, -TYRE_W / 2 + 40), (r, -TYRE_W / 2 + 100),
               (r, TYRE_W / 2 - 100), (r - 20, TYRE_W / 2 - 40), (r - 70, TYRE_W / 2 - 4), (inner + 20, TYRE_W / 2), (inner, TYRE_W / 2 - 30)]
    rim = [(0.0, 90.0), (120.0, 90.0), (140.0, 60.0), (inner - 4, 50.0), (inner + 6, 150.0), (inner + 6, 200.0)]
    for x, tag in ((REAR_AXLE, 'rear'), (FRONT_AXLE, 'front')):
        for k, side in ((-1, 'L'), (1, 'R')):
            y = k * TRACK / 2
            name = f'{tag}_{side}'
            parts.append(g.lathe(f, f'tyre_{name}', (x, y, r), (k, 0, 0), profile, m['rubber'], c, segments=56))
            parts.append(g.lathe(f, f'rim_{name}', (x, y, r), (k, 0, 0), [(rad, -z) for rad, z in rim], m['rim'], c, segments=40))
            parts.append(g.cylinder(f, f'hub_{name}', (x, y + k * 150, r), 'y', 110, 60, m['steel'], c, segments=24, bevel=8))
            for b in range(10):
                a = 2 * math.pi * b / 10
                parts.append(g.cylinder(f, f'lug_{name}_{b}', (x + 190 * math.cos(a), y + k * 140, r + 190 * math.sin(a)), 'y', 16, 30, m['steel'], c, segments=10))
            y0, y1 = span(k, TRACK / 2 + TYRE_W / 2 + 30, TRACK / 2 - TYRE_W / 2 - 30)
            parts.append(g.box(f, f'mudguard_{name}', x - 480, x + 480, y0, y1, TYRE_D + 20, TYRE_D + 70, m['body'], c, bevel=14))
            lip = k * (TRACK / 2 + TYRE_W / 2 + 30)
            parts.append(g.box(f, f'mudguard_lip_{name}', x - 480, x + 480, lip - 10, lip + 10, TYRE_D - 260, TYRE_D + 70, m['body'], c, bevel=6))
    for x, tag in ((REAR_AXLE, 'rear'), (FRONT_AXLE, 'front')):
        parts.append(g.cylinder(f, f'axle_{tag}', (x, 0, r), 'y', 110, TRACK - TYRE_W, m['steel'], c, segments=20))
        parts.append(g.box(f, f'axle_housing_{tag}', x - 260, x + 260, -TRACK / 2 + 320, TRACK / 2 - 320, r - 190, r + 190, m['steel'], c, bevel=10))
    return parts


def chassis(f, m, c):
    parts = []
    hull = [(0.0, 560.0), (0.0, CHASSIS_Z1), (4300.0, CHASSIS_Z1), (4700.0, 850.0), (5500.0, 760.0), (5500.0, CHASSIS_Z0), (700.0, CHASSIS_Z0), (260.0, 420.0)]
    parts.append(g.prism_xz(f, 'chassis_hull', hull, -HW, HW, m['body'], c, bevel=18))
    parts.append(g.box(f, 'rear_bumper', -60, 40, -HW - 60, HW + 60, 420, 700, m['black'], c, bevel=14))
    parts.append(g.box(f, 'engine_cover', 100, 2100, -HW + 80, HW - 80, CHASSIS_Z1, 1350, m['body'], c, bevel=30))
    parts.append(g.box(f, 'radiator_grille', -4, 40, -HW + 160, HW - 160, 980, 1280, m['grille'], c, bevel=4))
    for i in range(8):
        parts.append(g.box(f, f'grille_slat_{i}', -14, 6, -HW + 180, HW - 180, 1000 + i * 33, 1018 + i * 33, m['black'], c))
    parts.append(g.box(f, 'exhaust_stack_base', 1700, 1800, 650, 760, 1350, 1420, m['steel'], c, bevel=4))
    parts.append(g.cylinder(f, 'exhaust_stack', (1750, 705, 1700), 'z', 55, 700, m['steel'], c, segments=20, bevel=4))
    for k in (-1, 1):
        y0, y1 = span(k, HW + 280, HW)
        parts.append(g.box(f, f'step_{k}', 2200, 3000, y0, y1, 520, 560, m['steel'], c, bevel=4))
        y0, y1 = span(k, HW + 200, HW)
        parts.append(g.box(f, f'step_upper_{k}', 2300, 2900, y0, y1, 760, 800, m['steel'], c, bevel=4))
        parts.append(g.box(f, f'side_reflector_{k}', 600, 760, k * (HW + 1) - 2, k * (HW + 1) + 2, 600, 680, m['red'], c))
        parts.append(g.box(f, f'tow_lug_{k}', -80, 60, k * 720 - 40, k * 720 + 40, 360, 440, m['steel'], c, bevel=4))
    return parts


def cab(f, m, c):
    parts = []
    x0, x1 = 2250.0, 3700.0
    y0, y1 = -HW - 200, HW - 100
    z0, z1 = CHASSIS_Z1, 2300.0
    parts.append(g.box(f, 'cab_shell', x0, x1, y0, y1, z0, z1, m['body'], c, bevel=40))
    parts.append(g.box(f, 'cab_roof_pad', x0 + 40, x1 - 40, y0 + 40, y1 - 40, z1, z1 + 60, m['white'], c, bevel=20))
    parts.append(g.box(f, 'cab_windscreen', x1 - 2, x1 + 6, y0 + 70, y1 - 70, 1300, 2200, m['glass'], c, bevel=3))
    parts.append(g.box(f, 'cab_rear_glass', x0 - 6, x0 + 2, y0 + 120, y1 - 120, 1400, 2150, m['glass'], c, bevel=3))
    for yy, tag, k in ((y0, 'L', -1), (y1, 'R', 1)):
        parts.append(g.box(f, f'cab_side_glass_{tag}', x0 + 120, x1 - 120, yy - 4, yy + 4, 1350, 2200, m['glass'], c, bevel=3))
        a, b = span(k, 4, 28)
        parts.append(g.box(f, f'cab_door_handle_{tag}', 2400, 2520, yy + a, yy + b, 1500, 1540, m['black'], c, bevel=4))
        a, b = span(k, 4, 280)
        parts.append(g.box(f, f'mirror_arm_{tag}', x1 - 60, x1 - 20, yy + a, yy + b, 1900, 1940, m['black'], c, bevel=4))
        a, b = span(k, 280, 310)
        parts.append(g.box(f, f'mirror_{tag}', x1 - 90, x1 - 10, yy + a, yy + b, 1750, 2050, m['black'], c, bevel=8))
    parts.append(g.cylinder(f, 'beacon_base', (x0 + 300, 0, z1 + 70), 'z', 60, 40, m['black'], c, segments=24, bevel=4))
    parts.append(g.lathe(f, 'beacon_lens', (x0 + 300, 0, z1 + 110), 'z', [(0.0, 120.0), (45.0, 110.0), (80.0, 80.0), (80.0, 0.0)], m['amber'], c, segments=32))
    for k in (-1, 1):
        parts.append(g.box(f, f'work_light_{k}', x1 - 60, x1 - 10, k * 500 - 80, k * 500 + 80, z1 + 10, z1 + 80, m['white'], c, bevel=6))
    return parts


def cradle_fixed(f, m, c):
    parts = []
    for k in (-1, 1):
        y0, y1 = span(k, 700, 900)
        parts.append(g.box(f, f'cradle_arm_{k}', 4300, 6300, y0, y1, 560, 820, m['body'], c, bevel=20))
        parts.append(g.box(f, f'cradle_arm_cap_{k}', 6300, 6360, y0, y1, 560, 820, m['black'], c, bevel=8))
        a, b = span(k, 690, 760)
        parts.append(g.box(f, f'cradle_ramp_{k}', 5700, 6300, a, b, 820, 880, m['steel'], c, bevel=6))
        parts.append(g.box(f, f'front_lamp_{k}', 6340, 6410, k * 600 - 80, k * 600 + 80, 500, 560, m['amber'], c, bevel=4))
    parts.append(g.box(f, 'cradle_back_beam', 4300, 4500, -900, 900, 560, 1000, m['body'], c, bevel=24))
    parts.append(g.box(f, 'cradle_floor', 4500, 6300, -700, 700, 620, 660, m['steel'], c, bevel=4))
    parts.append(g.box(f, 'front_bumper', 6340, 6400, -HW - 100, HW + 100, 380, 640, m['black'], c, bevel=14))
    return parts


def cradle_moving(f, m, c):
    parts = []
    parts.append(g.box(f, 'cradle_roller_deck', 4700, 5300, -700, 700, 660, 700, m['steel'], c, bevel=4))
    for k in (-1, 1):
        a, b = span(k, 640, 760)
        parts.append(g.box(f, f'cradle_clamp_arm_{k}', 4600, 5400, a, b, 700, 1150, m['body'], c, bevel=20))
        parts.append(g.cylinder(f, f'cradle_clamp_roller_front_{k}', (5300, k * 640, 960), 'y', 120, 100, m['pad'], c, segments=28, bevel=6))
        parts.append(g.cylinder(f, f'cradle_clamp_roller_rear_{k}', (4700, k * 640, 960), 'y', 120, 100, m['pad'], c, segments=28, bevel=6))
        a, b = span(k, 570, 640)
        parts.append(g.box(f, f'cradle_clamp_pad_{k}', 4720, 5280, a, b, 800, 1100, m['pad'], c, bevel=10))
    parts.append(g.cylinder(f, 'cradle_lift_roller_front', (5250, 0, 760), 'y', 60, 1300, m['pad'], c, segments=24))
    parts.append(g.cylinder(f, 'cradle_lift_roller_rear', (4750, 0, 760), 'y', 60, 1300, m['pad'], c, segments=24))
    for o in parts:
        g.set_anim(o, CRADLE_ANIM)
    return parts


def hydraulics(f, m, c):
    parts = []
    for k in (-1, 1):
        parts.append(g.cylinder(f, f'lift_ram_{k}', (4900, k * 500, 560), 'x', 40, 600, m['steel'], c, segments=16))
    parts.append(g.box(f, 'fuel_tank', 700, 1700, HW - 40, HW + 260, 450, 830, m['steel'], c, bevel=18))
    parts.append(g.box(f, 'battery_box', 3900, 4300, -HW - 120, -HW + 5, 480, 800, m['black'], c, bevel=10))
    return parts


def build(with_connection=True, scheme='orange'):
    g.reset_scene()
    f = g.Frame(X0)
    c = g.collection(NAME)
    m = materials(scheme)
    chassis(f, m, c)
    wheels(f, m, c)
    cab(f, m, c)
    cradle_fixed(f, m, c)
    cradle_moving(f, m, c)
    hydraulics(f, m, c)
    detail(f, m, c)
    return c


def detail(f, m, c):
    r = TYRE_D / 2
    for x in (REAR_AXLE, FRONT_AXLE):
        for k in (-1, 1):
            y = k * TRACK / 2
            for b in range(72):
                a = 2 * math.pi * b / 72
                cx, cz = x + (r + 6) * math.cos(a), r + (r + 6) * math.sin(a)
                for dy in (-TYRE_W / 3, 0.0, TYRE_W / 3):
                    g.box(f, f'tread_{x:.0f}_{k}_{b}_{dy:.0f}', cx - 16, cx + 16, y + dy - 50, y + dy + 50, cz - 16, cz + 16, m['rubber'], c)
    x0, x1 = 2250.0, 3700.0
    y0, y1 = -HW - 200, HW - 100
    for x in (x0, x1 - 50):
        for yy in (y0, y1 - 50):
            g.box(f, f'cab_pillar_{x:.0f}_{yy:.0f}', x, x + 50, yy, yy + 50, 1250, 2300, m['black'], c, bevel=6)
    g.box(f, 'cab_door_frame_L', 2500, 3400, y0 - 8, y0 + 2, 1300, 1330, m['black'], c)
    g.box(f, 'cab_wiper', x1 + 6, x1 + 14, -300, 300, 1360, 1380, m['black'], c, bevel=2)
    for k in (-1, 1):
        yy = k * (HW + 280)
        g.tube(f, f'handrail_{k}', [(2300, yy, 800), (2300, yy, 1250), (2900, yy, 1250), (2900, yy, 800)], 18, m['steel'], c, segments=10)
        g.box(f, f'rear_light_{k}', -70, -40, k * 800 - 80, k * 800 + 80, 760, 820, m['red'], c, bevel=4)
        g.box(f, f'cab_head_light_{k}', x1 + 6, x1 + 40, k * 650 - 90, k * 650 + 90, 2150, 2230, m['white'], c, bevel=5)
        g.box(f, f'cradle_stripe_{k}', 4300, 6300, k * 800 - 20, k * 800 + 20, 822, 828, m['white'], c)
    for i in range(6):
        g.cylinder(f, f'cradle_roller_{i}', (4750 + i * 100, 0, 700), 'y', 22, 1300, m['steel'], c, segments=12)


def liveries(out):
    from PIL import Image, ImageDraw
    import os
    os.makedirs(os.path.join(out, 'liveries'), exist_ok=True)
    result = {}
    for operator, bg, fg, text in (('generic_orange', (232, 98, 12, 255), (255, 255, 255, 255), 'GROUND SERVICES'), ('generic_yellow', (245, 196, 0, 255), (20, 20, 20, 255), 'AIRPORT TUG')):
        im = Image.new('RGBA', (1024, 256), bg)
        d = ImageDraw.Draw(im)
        d.rectangle((0, 190, 1024, 230), fill=fg)
        d.text((40, 70), text, fill=fg)
        d.polygon([(900, 40), (980, 40), (940, 140)], fill=fg)
        path = os.path.join(out, 'liveries', f'{operator}.png')
        im.save(path)
        result[operator] = path
    return result


VIEWS = {
    'three_quarter_front': dict(res=(800, 560), lens=35, loc=(-7.5, 5.0, 3.6), target=(0.0, 0.3, 0.9), samples=16),
    'three_quarter_rear': dict(res=(800, 560), lens=35, loc=(6.5, -6.2, 3.8), target=(0.0, 0.0, 0.9), samples=16),
    'side_right': dict(res=(800, 400), lens=35, loc=(9.5, 0.0, 1.3), target=(0.0, 0.0, 1.0), samples=16),
    'front': dict(res=(700, 500), lens=35, loc=(0.0, 9.0, 1.8), target=(0.0, 0.0, 1.0), samples=16),
}
