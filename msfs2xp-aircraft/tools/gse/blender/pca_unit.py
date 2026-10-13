import math

import bpy

import gselib as g

NAME = 'pca_unit'
X0 = 2000.0
BODY_X0 = 0.0
BODY_X1 = 3900.0
HALF_W = 1000.0
BODY_Z0 = 520.0
BODY_Z1 = 2150.0
AXLE_XS = (1500.0, 2650.0)
TRACK = 1900.0
TYRE_D = 700.0
TYRE_W = 235.0
HOSE_RADIUS = 100.0
HOSE_OUTLET_Y = 470.0
HOSE_OUTLET_Z = 1000.0
CONNECTORS = ((6300.0, -260.0, 2080.0), (6810.0, 260.0, 2080.0))
HOSE_SHOW = [{'kind': 'hide', 'v0': -0.01, 'v1': 0.5, 'dataref': 'fbw/gse/pca/hose'}]
HOSE_HIDE = [{'kind': 'hide', 'v0': 0.5, 'v1': 1.01, 'dataref': 'fbw/gse/pca/hose'}]


def materials():
    return {
        'body': g.mat('gse_pca_grey', (196, 200, 203), roughness=0.4),
        'galv': g.mat('gse_galvanized', (168, 171, 172), metallic=0.85, roughness=0.42),
        'black': g.mat('gse_black', (22, 22, 23), roughness=0.55),
        'rubber': g.mat('gse_rubber', (28, 28, 28), roughness=0.88),
        'rim': g.mat('gse_rim', (34, 34, 35), metallic=0.4, roughness=0.5),
        'hub': g.mat('gse_hub', (150, 150, 148), metallic=0.9, roughness=0.35),
        'amber': g.mat('gse_amber_lens', (255, 140, 10), roughness=0.12),
        'red': g.mat('gse_red', (185, 18, 20), roughness=0.3),
        'yellow': g.mat('gse_yellow', (245, 200, 0), roughness=0.35),
        'grille': g.mat('gse_grille', (14, 14, 14), metallic=0.3, roughness=0.6),
        'hose': g.mat('gse_pca_hose', (232, 232, 228), roughness=0.55),
        'hose_ring': g.mat('gse_pca_hose_ring', (60, 62, 66), metallic=0.5, roughness=0.5),
        'panel': g.mat('gse_panel_black', (30, 31, 33), roughness=0.4),
        'green': g.mat('gse_green', (0, 140, 70), roughness=0.4),
    }


def fender_profile(x, r):
    outer = [(x + (r + 70) * math.cos(math.radians(10 + 160 * i / 16)), r + (r + 70) * math.sin(math.radians(10 + 160 * i / 16))) for i in range(17)]
    inner = [(x + (r + 55) * math.cos(math.radians(170 - 160 * i / 16)), r + (r + 55) * math.sin(math.radians(170 - 160 * i / 16))) for i in range(17)]
    return outer + inner


def chassis(f, m, c):
    r = TYRE_D / 2
    rr = 190.0
    tyre_profile = [(rr, -TYRE_W / 2 + 18), (rr + 8, -TYRE_W / 2 + 4), (r - 45, -TYRE_W / 2), (r - 14, -TYRE_W / 2 + 12), (r, -TYRE_W / 2 + 45),
                    (r, TYRE_W / 2 - 45), (r - 14, TYRE_W / 2 - 12), (r - 45, TYRE_W / 2), (rr + 8, TYRE_W / 2 - 4), (rr, TYRE_W / 2 - 18)]
    rim_profile = [(0.0, 60.0), (70.0, 60.0), (85.0, 44.0), (140.0, 38.0), (rr - 2, 34.0), (rr + 6, 80.0), (rr + 6, 92.0)]
    parts = []
    parts.append(g.box(f, 'frame_deck', BODY_X0 - 40, BODY_X1 + 40, -HALF_W + 60, HALF_W - 60, 430, BODY_Z0, m['galv'], c, bevel=6))
    for k in (-1, 1):
        parts.append(g.box(f, f'frame_rail_{k}', BODY_X0 - 40, BODY_X1 + 300, k * (HALF_W - 200) - 50, k * (HALF_W - 200) + 50, 380, 470, m['galv'], c, bevel=5))
    for ix, x in enumerate(AXLE_XS):
        parts.append(g.cylinder(f, f'axle_{ix}', (x, 0, r), 'y', 45, TRACK, m['galv'], c, segments=20))
        for k in (-1, 1):
            y = k * TRACK / 2
            name = f'{ix}_{"L" if k < 0 else "R"}'
            parts.append(g.lathe(f, f'tyre_{name}', (x, y, r), (k, 0, 0), tyre_profile, m['rubber'], c, segments=40))
            parts.append(g.lathe(f, f'rim_{name}', (x, y, r), (k, 0, 0), [(rad, -z) for rad, z in rim_profile], m['rim'], c, segments=32))
            parts.append(g.cylinder(f, f'hubcap_{name}', (x, y + k * 55, r), 'y', 48, 26, m['hub'], c, segments=20, bevel=4))
            yc = k * TRACK / 2
            for t in range(24):
                a = 2 * math.pi * t / 24
                parts.append(g.box(f, f'tread_{name}_{t}', x + (r - 4) * math.cos(a) - 12, x + (r - 4) * math.cos(a) + 12, y - 80, y + 80, r + (r - 4) * math.sin(a) - 12, r + (r - 4) * math.sin(a) + 12, m['rubber'], c))
            parts.append(g.prism_xz(f, f'fender_{name}', fender_profile(x, r), yc - 125, yc + 125, m['body'], c))
    return parts


def towbar(f, m, c):
    parts = []
    tip = (BODY_X1 + 1300.0, 0.0, 820.0)
    parts.append(g.tube(f, 'tow_leg_L', [(BODY_X1 + 100, -380, 520), tip], 38, m['galv'], c, segments=12))
    parts.append(g.tube(f, 'tow_leg_R', [(BODY_X1 + 100, 380, 520), tip], 38, m['galv'], c, segments=12))
    parts.append(g.box(f, 'tow_cross', BODY_X1 + 150, BODY_X1 + 220, -400, 400, 500, 600, m['galv'], c, bevel=5))
    parts.append(g.lathe(f, 'tow_eye', (tip[0] + 30, 0, tip[2]), (1, 0, 0), [(42, -18), (66, -18), (66, 18), (42, 18), (42, -18)], m['galv'], c, segments=28))
    parts.append(g.cylinder(f, 'tow_jockey_post', (BODY_X1 + 900, 0, 560), 'z', 40, 520, m['galv'], c, segments=16))
    parts.append(g.cylinder(f, 'tow_jockey_wheel', (BODY_X1 + 900, 0, 280), 'y', 140, 70, m['rubber'], c, segments=24, bevel=4))
    parts.append(g.box(f, 'tow_brake_housing', BODY_X1 + 220, BODY_X1 + 520, -150, 150, 560, 700, m['black'], c, bevel=6))
    return parts


def housing(f, m, c):
    parts = []
    hood = [(BODY_X0, BODY_Z0), (BODY_X0, BODY_Z1), (BODY_X1 - 700, BODY_Z1), (BODY_X1 - 200, BODY_Z1 - 60), (BODY_X1, BODY_Z1 - 300), (BODY_X1, BODY_Z0)]
    parts.append(g.prism_xz(f, 'housing_shell', hood, -HALF_W + 20, HALF_W - 20, m['body'], c, bevel=18, segments=3))
    for k in (-1, 1):
        y = k * (HALF_W - 18)
        for i in range(4):
            x0 = 160 + i * 880
            parts.append(g.box(f, f'service_door_{k}_{i}', x0, x0 + 840, y - k * 2, y + k * 5, BODY_Z0 + 90, BODY_Z1 - 100, m['body'], c, bevel=5))
            parts.append(g.box(f, f'door_handle_{k}_{i}', x0 + 700, x0 + 780, y + k * 5, y + k * 28, 1150, 1230, m['black'], c, bevel=5))
            for zh in (BODY_Z0 + 220, BODY_Z1 - 220):
                parts.append(g.box(f, f'door_hinge_{k}_{i}_{int(zh)}', x0 + 8, x0 + 40, y + k * 5, y + k * 14, zh - 45, zh + 45, m['black'], c, bevel=3))
        for j in range(14):
            parts.append(g.box(f, f'louvre_{k}_{j}', 3050, 3640, y + k * 4, y + k * 14, 820 + j * 80, 840 + j * 80, m['grille'], c, bevel=2))
        parts.append(g.box(f, f'side_reflector_{k}', 120, 260, y + k * 4, y + k * 8, 600, 660, m['red'], c))
        parts.append(g.box(f, f'side_reflector_f_{k}', 3600, 3740, y + k * 4, y + k * 8, 600, 660, m['yellow'], c))
        for j in range(5):
            xs = 160 + j * 880 - 20
            parts.append(g.box(f, f'seam_{k}_{j}', xs, xs + 8, y + k * 4, y + k * 7, BODY_Z0 + 20, BODY_Z1 - 20, m['black'], c))
        parts.append(g.box(f, f'step_{k}', 1900, 2500, y + k * 4, y + k * 140, 400, 440, m['galv'], c, bevel=4))
    for j in range(10):
        parts.append(g.box(f, f'top_louvre_{j}', 2600, 3300, -500 + j * 105, -500 + j * 105 + 60, BODY_Z1 - 2, BODY_Z1 + 12, m['grille'], c, bevel=2))
    parts.append(g.box(f, 'top_fan_guard_frame', 300, 1500, -520, 520, BODY_Z1 - 2, BODY_Z1 + 25, m['black'], c, bevel=5))
    parts.append(g.box(f, 'top_fan_guard_grille', 350, 1450, -470, 470, BODY_Z1 + 20, BODY_Z1 + 28, m['grille'], c))
    for i in range(7):
        parts.append(g.box(f, f'fan_guard_bar_{i}', 350, 1450, -450 + i * 150, -450 + i * 150 + 20, BODY_Z1 + 25, BODY_Z1 + 40, m['black'], c))
    parts.append(g.cylinder(f, 'exhaust_stack', (3350.0, 0.0, BODY_Z1 + 150), 'z', 90, 300, m['black'], c, segments=20, bevel=4))
    parts.append(g.cylinder(f, 'exhaust_cap', (3350.0, 0.0, BODY_Z1 + 320), 'z', 120, 25, m['galv'], c, segments=20, bevel=3))
    parts.append(g.cylinder(f, 'beacon_base', (3000.0, 0.0, BODY_Z1 + 15), 'z', 60, 30, m['black'], c, segments=20, bevel=3))
    parts.append(g.lathe(f, 'beacon_lens', (3000.0, 0.0, BODY_Z1 + 30), 'z', [(0.0, 90.0), (28.0, 86.0), (46.0, 74.0), (50.0, 0.0)], m['amber'], c, segments=28))
    end = BODY_X0
    parts.append(g.box(f, 'ctrl_panel', end - 8, end + 2, -320, 320, 1000, 1700, m['panel'], c, bevel=6))
    parts.append(g.box(f, 'ctrl_screen', end - 12, end - 7, -120, 120, 1450, 1620, m['black'], c, bevel=3))
    for i, col in enumerate(('green', 'yellow', 'red', 'green')):
        parts.append(g.cylinder(f, f'ctrl_lamp_{i}', (end - 12, -240 + i * 160, 1300), 'x', 24, 8, m[col], c, segments=16, bevel=2))
    parts.append(g.cylinder(f, 'estop_base', (end - 10, 220, 1100), 'x', 50, 18, m['yellow'], c, segments=20, bevel=3))
    parts.append(g.cylinder(f, 'estop_button', (end - 26, 220, 1100), 'x', 34, 20, m['red'], c, segments=20, bevel=8))
    parts.append(g.box(f, 'rear_cable_gland_plate', end - 6, end + 2, -320, 320, 560, 900, m['galv'], c, bevel=3))
    return parts


def outlets(f, m, c):
    parts = []
    for k in (-1, 1):
        y = k * HOSE_OUTLET_Y
        parts.append(g.cylinder(f, f'outlet_collar_{k}', (BODY_X1 + 12, y, HOSE_OUTLET_Z), 'x', 150, 90, m['hose_ring'], c, segments=28, bevel=5))
        parts.append(g.cylinder(f, f'outlet_flange_{k}', (BODY_X1 + 2, y, HOSE_OUTLET_Z), 'x', 175, 14, m['galv'], c, segments=28, bevel=3))
        for b in range(8):
            a = 2 * math.pi * b / 8
            parts.append(g.cylinder(f, f'outlet_bolt_{k}_{b}', (BODY_X1 - 2, y + 160 * math.cos(a), HOSE_OUTLET_Z + 160 * math.sin(a)), 'x', 9, 14, m['hub'], c, segments=8))
    return parts


def deployed(f, m, c):
    parts = []
    for k, (cx, cy, cz) in zip((-1, 1), CONNECTORS):
        y0 = k * HOSE_OUTLET_Y
        path = [(BODY_X1 + 60, y0, HOSE_OUTLET_Z), (BODY_X1 + 500, y0 + k * 40, 700), (BODY_X1 + 1300, k * 330, 520),
                (BODY_X1 + 2000, k * 380, 800), (cx - 500, cy * 1.6, 1100), (cx - 250, cy * 1.2, 1500), (cx - 40, cy, cz - 350), (cx, cy, cz - 150)]
        smooth = g.smooth_path(path, 10)
        hose = g.tube(f, f'hose_deployed_{k}', smooth, HOSE_RADIUS, m['hose'], c, segments=16)
        g.set_anim(hose, HOSE_SHOW)
        parts.append(hose)
        for i, (px, py, pz) in enumerate(g.smooth_path(path, 2)[::3]):
            ring = g.cylinder(f, f'hose_deployed_clamp_{k}_{i}', (px, py, pz), 'z', HOSE_RADIUS + 10, 22, m['hose_ring'], c, segments=16)
            g.set_anim(ring, HOSE_SHOW)
            parts.append(ring)
        nozzle = g.lathe(f, f'hose_deployed_nozzle_{k}', (cx, cy, cz - 220), 'z',
                         [(0.0, 0.0), (HOSE_RADIUS + 12, 0.0), (HOSE_RADIUS + 12, 120.0), (105.0, 150.0), (105.0, 200.0), (125.0, 200.0), (125.0, 224.0), (0.0, 224.0)], m['hose_ring'], c, segments=24)
        g.set_anim(nozzle, HOSE_SHOW)
        parts.append(nozzle)
    return parts


def stowed(f, m, c):
    parts = []
    for k in (-1, 1):
        y0 = k * HOSE_OUTLET_Y
        path = [(BODY_X1 + 60, y0, HOSE_OUTLET_Z), (BODY_X1 + 400, y0, 760), (BODY_X1 + 560, y0 + k * 200, 560), (BODY_X1 + 520, k * 700, 520),
                (BODY_X1 + 200, k * 760, 520), (BODY_X1 - 40, k * 560, 520)]
        hose = g.tube(f, f'hose_stowed_{k}', g.smooth_path(path, 8), HOSE_RADIUS, m['hose'], c, segments=16)
        g.set_anim(hose, HOSE_HIDE)
        parts.append(hose)
        nozzle = g.lathe(f, f'hose_stowed_nozzle_{k}', (BODY_X1 - 40, k * 560, 520), 'x',
                         [(0.0, 0.0), (HOSE_RADIUS + 12, 0.0), (HOSE_RADIUS + 12, -100.0), (125.0, -100.0), (125.0, -124.0), (0.0, -124.0)], m['hose_ring'], c, segments=24)
        g.set_anim(nozzle, HOSE_HIDE)
        parts.append(nozzle)
    return parts


def hose_cradle(f, m, c):
    parts = []
    for k in (-1, 1):
        parts.append(g.box(f, f'cradle_{k}', BODY_X1 + 150, BODY_X1 + 650, k * 700 - 150, k * 700 + 150, 400, 440, m['galv'], c, bevel=4))
        for dx in (180, 520):
            parts.append(g.box(f, f'cradle_support_{k}_{dx}', BODY_X1 + dx - 20, BODY_X1 + dx + 20, k * 700 - 150, k * 700 + 150, 440, 600, m['galv'], c, bevel=3))
        parts.append(g.box(f, f'cradle_post_{k}', BODY_X1 + 300, BODY_X1 + 340, k * 700 - 6, k * 700 + 6, 440, 520, m['galv'], c))
    return parts


def build(with_connection=True):
    g.reset_scene()
    f = g.Frame(X0)
    c = g.collection(NAME)
    m = materials()
    chassis(f, m, c)
    towbar(f, m, c)
    housing(f, m, c)
    outlets(f, m, c)
    hose_cradle(f, m, c)
    stowed(f, m, c)
    if with_connection:
        deployed(f, m, c)
    return c


def liveries(out):
    return {}


VIEWS = {
    'side_right': dict(res=(800, 480), lens=35, loc=(9.0, 2.0, 1.4), target=(0.0, 2.0, 1.2), samples=16),
    'three_quarter_rear': dict(res=(800, 560), lens=35, loc=(-5.0, -6.0, 3.2), target=(0.0, 0.5, 1.2), samples=16),
    'connection': dict(res=(800, 560), lens=28, loc=(6.5, 3.0, 3.0), target=(0.0, 2.8, 1.6), samples=16),
}
