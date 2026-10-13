import os
import subprocess

import gselib as g

NAME = 'deice_truck'
X0 = 4700.0
LIVERY_DIR = 'E:/fbw-gse/out/deice_truck/liveries'

TRACK = 2040.0
HALF_WIDTH = 1250.0
TYRE_D = 1050.0
TYRE_W = 315.0
FRONT_AXLE = 7350.0
REAR_AXLES = (1700.0, 3000.0)
RAIL_Z0 = 820.0
RAIL_Z1 = 960.0
CAB_X0 = 6700.0
CAB_X1 = 8650.0
CAB_Z1 = 3050.0
TANK_X0 = 350.0
TANK_X1 = 5600.0
TANK_RADIUS = 1000.0
TANK_Z = 1980.0
HINGE = (1250.0, 0.0, 3350.0)
SECTION_LENGTH = (4300.0, 4300.0, 4300.0)
EXTEND_MID = 3600.0
EXTEND_TOP = 7200.0
BOOM_LIMIT_DEG = 72.0

LIVERIES = {
    'swissport': ((0, 82, 155), (255, 255, 255), 'SWISSPORT'),
    'dnata': ((200, 16, 46), (255, 255, 255), 'dnata'),
    'airport': ((245, 190, 0), (20, 20, 22), 'AIRPORT DE-ICING'),
    'aviapartner': ((0, 120, 80), (255, 255, 255), 'AVIAPARTNER'),
}


def materials():
    return {
        'white': g.mat('gse_white', (236, 237, 233), roughness=0.32),
        'orange': g.mat('gse_deice_orange', (232, 112, 20), roughness=0.38),
        'galv': g.mat('gse_galvanized', (168, 171, 172), metallic=0.85, roughness=0.42),
        'black': g.mat('gse_black', (22, 22, 23), roughness=0.55),
        'chassis': g.mat('gse_chassis', (48, 50, 52), metallic=0.5, roughness=0.5),
        'glass': g.mat('gse_glass', (24, 34, 40), roughness=0.06),
        'rubber': g.mat('gse_rubber', (28, 28, 28), roughness=0.88),
        'rim': g.mat('gse_rim', (150, 152, 150), metallic=0.8, roughness=0.4),
        'amber': g.mat('gse_amber_lens', (255, 140, 10), roughness=0.12),
        'red': g.mat('gse_red', (185, 18, 20), roughness=0.3),
        'yellow': g.mat('gse_yellow', (245, 200, 0), roughness=0.35),
        'hose': g.mat('gse_hose', (20, 20, 20), roughness=0.7),
        'cab': g.mat('gse_cab_paint', (236, 237, 233), roughness=0.3),
    }


def pivot_m(f, point):
    v = f.v(*point)
    return (v.x, v.y, v.z)


def raise_step(f):
    return {'kind': 'rotate', 'pivot': pivot_m(f, HINGE), 'axis': (1.0, 0.0, 0.0), 'deg0': 0.0, 'deg1': BOOM_LIMIT_DEG, 'v0': 0.0, 'v1': 1.0, 'dataref': 'fbw/gse/deice/boom'}


def extend_step(distance_mm):
    d = distance_mm / 1000.0
    return {'kind': 'trans', 'offset0': (0.0, 0.0, 0.0), 'offset1': (0.0, d, 0.0), 'v0': 0.0, 'v1': 1.0, 'dataref': 'fbw/gse/deice/extend'}


def chassis(f, m, c):
    parts = []
    for k in (-1, 1):
        parts.append(g.box(f, f'frame_rail_{k}', 0, 9000, k * 430 - 70, k * 430 + 70, RAIL_Z0, RAIL_Z1, m['chassis'], c, bevel=8))
        parts.append(g.box(f, f'mudguard_rear_{k}', 1000, 3700, k * (HALF_WIDTH - 40) - 30, k * (HALF_WIDTH - 40) + 30, 1100, 1300, m['black'], c, bevel=6))
        parts.append(g.box(f, f'side_skirt_{k}', 3900, 6500, k * (HALF_WIDTH - 30) - 20, k * (HALF_WIDTH - 30) + 20, 700, 1000, m['galv'], c, bevel=5))
        parts.append(g.box(f, f'fuel_tank_{k}', 4000, 5200, k * 1000 - 120, k * 1000 + 120, 600, 1080, m['galv'], c, bevel=10))
        parts.append(g.box(f, f'rear_light_{k}', -90, -70, k * 900 - 90, k * 900 + 90, 800, 900, m['red'], c, bevel=4))
    parts.append(g.box(f, 'cross_beam_front', 8500, 8700, -1000, 1000, RAIL_Z0, RAIL_Z1 + 80, m['chassis'], c, bevel=6))
    parts.append(g.box(f, 'front_bumper', 8650, 8850, -1220, 1220, 520, 880, m['black'], c, bevel=22))
    parts.append(g.box(f, 'rear_bumper', -80, 90, -1100, 1100, 650, 960, m['black'], c, bevel=18))
    parts.append(g.box(f, 'subframe_deck', 150, 5700, -HALF_WIDTH + 80, HALF_WIDTH - 80, 960, 1060, m['chassis'], c, bevel=8))
    return parts


def wheels(f, m, c):
    parts = []
    r = TYRE_D / 2
    rr = 245.0
    tyre_profile = [(rr, -TYRE_W / 2 + 25), (rr + 12, -TYRE_W / 2 + 8), (r - 60, -TYRE_W / 2), (r - 18, -TYRE_W / 2 + 14), (r, -TYRE_W / 2 + 55),
                    (r, TYRE_W / 2 - 55), (r - 18, TYRE_W / 2 - 14), (r - 60, TYRE_W / 2), (rr + 12, TYRE_W / 2 - 8), (rr, TYRE_W / 2 - 25)]
    rim_profile = [(0.0, 60.0), (90.0, 60.0), (110.0, 40.0), (190.0, 35.0), (rr - 2, 30.0), (rr + 8, 110.0), (rr + 8, 130.0)]
    for x in (FRONT_AXLE,) + REAR_AXLES:
        single = x == FRONT_AXLE
        for k in (-1, 1):
            for j, o in enumerate((0,) if single else (-1, 1)):
                y = k * TRACK / 2 if single else k * (TRACK / 2 + 120) + o * (TYRE_W / 2 + 6)
                name = f'{int(x)}_{k}_{j}'
                parts.append(g.lathe(f, f'tyre_{name}', (x, y, r), (k, 0, 0), tyre_profile, m['rubber'], c, segments=36))
                parts.append(g.lathe(f, f'rim_{name}', (x, y, r), (k, 0, 0), [(rad, -z) for rad, z in rim_profile], m['rim'], c, segments=28))
                parts.append(g.cylinder(f, f'hub_{name}', (x, y + k * 70, r), 'y', 70, 40, m['galv'], c, segments=16))
        parts.append(g.cylinder(f, f'axle_{int(x)}', (x, 0, r), 'y', 70, TRACK, m['chassis'], c, segments=16))
        parts.append(g.box(f, f'diff_{int(x)}', x - 130, x + 130, -160, 160, r - 120, r + 120, m['chassis'], c, bevel=18))
    return parts


def cab(f, m, c):
    parts = []
    hull = [(CAB_X0, 960.0), (CAB_X0, CAB_Z1), (CAB_X1 - 700, CAB_Z1), (CAB_X1 - 250, CAB_Z1 - 280), (CAB_X1, CAB_Z1 - 1050), (CAB_X1, 960.0)]
    shell = g.prism_xz(f, 'cab_shell', hull, -1200, 1200, m['cab'], c)
    g.bevel_where(shell, lambda pt, d: True, 40, 3)
    for p in shell.data.polygons:
        p.use_smooth = True
    parts.append(shell)
    parts.append(g.prism_xz(f, 'windscreen', [(CAB_X1 - 703, CAB_Z1 - 25), (CAB_X1 - 255, CAB_Z1 - 300), (CAB_X1 - 20, CAB_Z1 - 1000), (CAB_X1 - 60, CAB_Z1 - 1010), (CAB_X1 - 300, CAB_Z1 - 400), (CAB_X1 - 703, CAB_Z1 - 130)], -1070, 1070, m['glass'], c, bevel=6))
    for k in (-1, 1):
        parts.append(g.box(f, f'door_window_{k}', CAB_X1 - 1250, CAB_X1 - 350, k * 1198, k * 1204, 1900, 2750, m['glass'], c, bevel=8))
        parts.append(g.box(f, f'door_handle_{k}', CAB_X0 + 1450, CAB_X0 + 1650, k * 1200, k * 1235, 1700, 1740, m['black'], c, bevel=5))
        parts.append(g.box(f, f'door_step_{k}', CAB_X1 - 1300, CAB_X1 - 500, k * (HALF_WIDTH - 60) - 100, k * (HALF_WIDTH - 60) + 100, 500, 540, m['galv'], c, bevel=4))
        parts.append(g.box(f, f'mirror_arm_{k}', CAB_X1 - 800, CAB_X1 - 740, k * 1200, k * 1400, 2300, 2330, m['black'], c, bevel=3))
        parts.append(g.box(f, f'mirror_{k}', CAB_X1 - 840, CAB_X1 - 760, k * 1400 - 40, k * 1400 + 40, 2000, 2650, m['black'], c, bevel=12))
        parts.append(g.box(f, f'headlight_{k}', CAB_X1 - 40, CAB_X1 + 10, k * 820 - 150, k * 820 + 150, 1000, 1160, m['white'], c, bevel=18))
        parts.append(g.box(f, f'indicator_{k}', CAB_X1 - 40, CAB_X1 + 8, k * 1060 - 60, k * 1060 + 60, 1000, 1160, m['amber'], c, bevel=10))
    parts.append(g.box(f, 'grille', CAB_X1 - 20, CAB_X1 + 12, -560, 560, 1250, 1850, m['black'], c, bevel=12))
    parts.append(g.box(f, 'roof_beacon_base', CAB_X0 + 800, CAB_X0 + 1000, -100, 100, CAB_Z1, CAB_Z1 + 60, m['black'], c, bevel=4))
    parts.append(g.cylinder(f, 'roof_beacon', (CAB_X0 + 900, 0, CAB_Z1 + 130), 'z', 100, 140, m['amber'], c, segments=20, bevel=6))
    return parts


def tank_block(f, m, c):
    parts = []
    length = TANK_X1 - TANK_X0
    cx = (TANK_X0 + TANK_X1) / 2
    parts.append(g.cylinder(f, 'fluid_tank', (cx, 0, TANK_Z), 'x', TANK_RADIUS, length, m['white'], c, segments=48, bevel=30))
    for k in (-1, 1):
        parts.append(g.box(f, f'tank_band_{k}', TANK_X0 + 400, TANK_X1 - 300, k * (TANK_RADIUS - 2) - 4, k * (TANK_RADIUS - 2) + 4, 2650, 2700, m['orange'], c))
        parts.append(g.box(f, f'locker_{k}', 3200, 5400, k * (HALF_WIDTH - 40) - 40, k * (HALF_WIDTH - 40) + 40, 1080, 1850, m['galv'], c, bevel=12))
        parts.append(g.box(f, f'locker_door_{k}', 3250, 5350, k * (HALF_WIDTH + 2) - 6, k * (HALF_WIDTH + 2) + 6, 1130, 1800, m['white'], c, bevel=8))
        parts.append(g.box(f, f'locker_handle_{k}', 4200, 4400, k * (HALF_WIDTH + 25) - 20, k * (HALF_WIDTH + 25) + 20, 1450, 1480, m['black'], c, bevel=4))
        for ex in (1800.0, 2500.0):
            parts.append(g.box(f, f'ladder_rung_{k}_{int(ex)}', ex - 15, ex + 15, k * (HALF_WIDTH - 20) - 40, k * (HALF_WIDTH - 20) + 40, 1100, 1700, m['galv'], c, bevel=2))
        parts.append(g.box(f, f'tank_reflector_{k}', 5200, 5420, k * (HALF_WIDTH + 2) - 3, k * (HALF_WIDTH + 2) + 3, 1000, 1070, m['red'], c))
        parts.append(g.box(f, f'hinge_cheek_{k}', 950, 1550, k * 380 - 60, k * 380 + 60, 3180, 3560, m['chassis'], c, bevel=12))
        parts.append(g.box(f, f'stabiliser_leg_{k}', 4300, 4500, k * 1200 - 70, k * 1200 + 70, 400, 900, m['galv'], c, bevel=8))
        parts.append(g.box(f, f'stabiliser_foot_{k}', 4200, 4600, k * 1200 - 160, k * 1200 + 160, 330, 400, m['black'], c, bevel=8))
        parts.append(g.box(f, f'boom_rest_{k}', 6300, 6500, k * 380 - 60, k * 380 + 60, CAB_Z1, CAB_Z1 + 420, m['chassis'], c, bevel=8))
    parts.append(g.box(f, 'tank_end_cap', TANK_X0 - 60, TANK_X0, -1050, 1050, 1000, 2950, m['white'], c, bevel=14))
    parts.append(g.box(f, 'fill_hatch', 3000, 3400, -250, 250, TANK_Z + TANK_RADIUS - 20, TANK_Z + TANK_RADIUS + 40, m['galv'], c, bevel=6))
    parts.append(g.box(f, 'pump_cabinet', 5700, 6450, -1000, 1000, 1060, 2300, m['white'], c, bevel=18))
    parts.append(g.box(f, 'pump_cabinet_grille', 5700, 5760, -700, 700, 1500, 2000, m['black'], c, bevel=6))
    parts.append(g.box(f, 'turntable_base', 700, 1850, -700, 700, 2900, 3050, m['galv'], c, bevel=20))
    parts.append(g.cylinder(f, 'turntable_ring', (1250, 0, 3100), 'z', 620, 180, m['chassis'], c, segments=36, bevel=10))
    return parts


def boom(f, m, c):
    parts = []
    raise_s = raise_step(f)
    widths = (400.0, 330.0, 270.0)
    heights = (520.0, 440.0, 370.0)
    mats = ('orange', 'white', 'orange')
    names = ('lower', 'mid', 'top')
    section_steps = ([raise_s], [raise_s, extend_step(EXTEND_MID)], [raise_s, extend_step(EXTEND_TOP)])
    x = HINGE[0]
    zc = HINGE[2]
    for i in range(3):
        length = SECTION_LENGTH[i]
        obj = g.box(f, f'boom_{names[i]}', x, x + length, -widths[i], widths[i], zc - heights[i] / 2, zc + heights[i] / 2, m[mats[i]], c, bevel=18)
        g.set_anim(obj, section_steps[i])
        parts.append(obj)
        for k in (-1, 1):
            pad = g.box(f, f'boom_{names[i]}_wearpad_{k}', x + 300, x + length - 300, k * (widths[i] + 6) - 8, k * (widths[i] + 6) + 8, zc - 60, zc + 60, m['black'], c, bevel=3)
            g.set_anim(pad, section_steps[i])
            parts.append(pad)
        x += (700.0, 400.0, 0.0)[i]
    parts.append(g.tube(f, 'ram_cylinder', [(900.0, -250.0, 3030.0), (1900.0, -250.0, 3200.0)], 85, m['galv'], c, segments=16))
    rod = g.tube(f, 'ram_rod', [(1900.0, -250.0, 3200.0), (2700.0, -250.0, 3280.0)], 45, m['galv'], c, segments=12)
    g.set_anim(rod, [raise_s])
    parts.append(rod)
    hose_points = [(1250.0, 0.0, 3150.0), (3000.0, 120.0, 3150.0), (6000.0, 150.0, 3130.0), (8600.0, 100.0, 3120.0)]
    hose = g.tube(f, 'boom_fluid_hose', g.smooth_path(hose_points, 6), 28, m['hose'], c, segments=10)
    g.set_anim(hose, [raise_s])
    parts.append(hose)
    return parts


def basket(f, m, c):
    parts = []
    tip_x = HINGE[0] + 1100.0 + SECTION_LENGTH[2] - 900.0
    cx = tip_x + 700.0
    z0 = HINGE[2] - 450.0
    spin = {'kind': 'rotate', 'pivot': pivot_m(f, (cx, 0.0, z0)), 'axis': (0.0, 0.0, 1.0), 'deg0': -90.0, 'deg1': 90.0, 'v0': 0.0, 'v1': 1.0, 'dataref': 'fbw/gse/deice/basket'}
    mount_steps = [raise_step(f), extend_step(EXTEND_TOP)]
    steps = mount_steps + [spin]
    mount = g.box(f, 'basket_mount_arm', tip_x - 300, cx + 100, -150, 150, HINGE[2] - 520, HINGE[2] - 150, m['chassis'], c, bevel=12)
    post = g.cylinder(f, 'basket_pivot_post', (cx, 0, z0 + 50), 'z', 110, 220, m['galv'], c, segments=20, bevel=4)
    g.set_anim(mount, mount_steps)
    g.set_anim(post, mount_steps)
    parts += [mount, post]
    pieces = [
        g.box(f, 'basket_shell', cx - 750, cx + 750, -720, 720, z0, z0 + 1900, m['orange'], c, bevel=36),
        g.box(f, 'basket_front_window', cx + 745, cx + 760, -560, 560, z0 + 900, z0 + 1750, m['glass'], c, bevel=10),
        g.box(f, 'basket_floor_grid', cx - 800, cx + 800, -780, 780, z0 - 80, z0, m['galv'], c, bevel=6),
        g.box(f, 'basket_roof_light_bar', cx + 300, cx + 700, -400, 400, z0 + 1900, z0 + 1960, m['white'], c, bevel=6),
        g.box(f, 'nozzle_turret', cx + 300, cx + 700, -150, 150, z0 + 1960, z0 + 2080, m['chassis'], c, bevel=10),
        g.tube(f, 'nozzle_barrel', [(cx + 500, 0.0, z0 + 2080), (cx + 850, 0.0, z0 + 2150), (cx + 1250, 0.0, z0 + 2200)], 55, m['galv'], c, segments=16),
        g.cylinder(f, 'nozzle_tip', (cx + 1330, 0, z0 + 2210), (1.0, 0.0, 0.08), 70, 140, m['black'], c, segments=16, bevel=6),
        g.cylinder(f, 'basket_beacon', (cx - 500, 0, z0 + 2000), 'z', 70, 110, m['amber'], c, segments=16, bevel=5),
    ]
    for k in (-1, 1):
        pieces.append(g.box(f, f'basket_side_window_{k}', cx - 400, cx + 450, k * 718, k * 726, z0 + 950, z0 + 1700, m['glass'], c, bevel=8))
        pieces.append(g.box(f, f'basket_side_rail_{k}', cx - 760, cx + 760, k * 735 - 10, k * 735 + 10, z0 + 560, z0 + 600, m['yellow'], c, bevel=3))
    for o in pieces:
        g.set_anim(o, steps)
    parts += pieces
    return parts


def draw_livery(out, name, spec):
    base, text_color, label = spec
    script = (
        'import sys\n'
        'from PIL import Image, ImageDraw, ImageFont\n'
        'path, label, br, bg, bb, tr, tg, tb = sys.argv[1:9]\n'
        'img = Image.new("RGBA", (1024, 256), (0, 0, 0, 0))\n'
        'd = ImageDraw.Draw(img)\n'
        'd.rounded_rectangle((8, 8, 1016, 248), 40, fill=(int(br), int(bg), int(bb), 255))\n'
        'font = ImageFont.truetype("C:/Windows/Fonts/arialbd.ttf", 128 if len(label) < 12 else 84)\n'
        'box = d.textbbox((0, 0), label, font=font)\n'
        'd.text(((1024 - (box[2] - box[0])) / 2 - box[0], (256 - (box[3] - box[1])) / 2 - box[1]), label, font=font, fill=(int(tr), int(tg), int(tb), 255))\n'
        'img.save(path)\n'
    )
    path = os.path.join(out, f'{name}_logo.png')
    subprocess.run(['python', '-I', '-c', script, path, label, *map(str, base), *map(str, text_color)], check=True)
    return path


def liveries(out=LIVERY_DIR):
    os.makedirs(out, exist_ok=True)
    return {name: draw_livery(out, name, spec) for name, spec in LIVERIES.items()}


def side_decals(f, c, logo_path):
    mat = g.image_mat('decal_operator', logo_path, roughness=0.4)
    placements = (
        ('d_operator', 1100.0, 4500.0, TANK_Z - 350, TANK_Z + 550, TANK_RADIUS + 3.0),
        ('d_cab_operator', CAB_X0 + 450, CAB_X0 + 1350, 1050.0, 1450.0, 1202.0),
    )
    for label, x0, x1, z0, z1, off in placements:
        for k in (-1, 1):
            y = k * off
            ends = (x0, x1) if k > 0 else (x1, x0)
            g.decal(f, f'{label}_{k}', [(ends[0], y, z0), (ends[1], y, z0), (ends[1], y, z1), (ends[0], y, z1)], mat, c)


def build(with_connection=True):
    g.reset_scene()
    f = g.Frame(X0)
    c = g.collection(NAME)
    m = materials()
    chassis(f, m, c)
    wheels(f, m, c)
    cab(f, m, c)
    tank_block(f, m, c)
    boom(f, m, c)
    basket(f, m, c)
    side_decals(f, c, liveries()['airport'])
    return c


def view(res, loc, target, lens=35):
    return dict(res=res, lens=lens, loc=loc, target=target, samples=16)


VIEWS = {
    'three_quarter_front': view((800, 520), (-9.0, 10.0, 4.5), (0.0, 0.0, 1.8), 30),
    'three_quarter_rear': view((800, 520), (9.0, -11.0, 4.5), (0.0, 0.2, 1.8), 30),
    'side_right': view((800, 400), (20.0, 0.0, 2.0), (0.0, 0.0, 1.9), 50),
}
