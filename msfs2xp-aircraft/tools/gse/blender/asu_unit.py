import math

import bpy

import gselib as g

NAME = 'asu_unit'
X0 = 1980.0
L = 3962.0
W = 1524.0
HY = W / 2
ENC_X0 = 120.0
ENC_X1 = 3250.0
ENC_Z0 = 650.0
ENC_Z1 = 1930.0
REAR_AXLE = 900.0
FRONT_AXLE = 2800.0
TRACK = 1400.0
TYRE_D = 700.0
TYRE_W = 230.0
PORT = (3250.0, 520.0, 1000.0)
CONNECTOR = (5400.0, 200.0, 1780.0)
HOSE_RADIUS = 50.0
HOSE_ANIM = [{'kind': 'trans', 'offset0': (0.0, 0.0, -0.6), 'offset1': (0.0, 0.0, 0.0), 'v0': 0.0, 'v1': 1.0, 'dataref': 'fbw/gse/asu/hose'}]


def materials():
    return {
        'white': g.mat('gse_white', (232, 233, 228), roughness=0.35),
        'grey': g.mat('gse_grey', (120, 124, 128), roughness=0.5),
        'galv': g.mat('gse_galvanized', (168, 171, 172), metallic=0.85, roughness=0.42),
        'black': g.mat('gse_black', (22, 22, 23), roughness=0.55),
        'rubber': g.mat('gse_rubber', (28, 28, 28), roughness=0.88),
        'rim': g.mat('gse_rim', (34, 34, 35), metallic=0.4, roughness=0.5),
        'hub': g.mat('gse_hub', (150, 150, 148), metallic=0.9, roughness=0.35),
        'amber': g.mat('gse_amber_lens', (255, 140, 10), roughness=0.12),
        'red': g.mat('gse_red', (185, 18, 20), roughness=0.3),
        'yellow': g.mat('gse_yellow', (245, 200, 0), roughness=0.35),
        'grille': g.mat('gse_grille', (14, 14, 14), metallic=0.3, roughness=0.6),
        'hose': g.mat('gse_hose', (150, 100, 20), roughness=0.7),
        'glass': g.mat('gse_screen', (18, 22, 26), roughness=0.08),
    }


def chassis(f, m, c):
    parts = []
    parts.append(g.box(f, 'frame_beam', 0, 3300, -HY + 120, HY - 120, 360, ENC_Z0, m['galv'], c, bevel=6))
    parts.append(g.box(f, 'frame_deck', 0, 3300, -HY, HY, ENC_Z0 - 40, ENC_Z0, m['grey'], c, bevel=4))
    parts.append(g.box(f, 'bumper_rear', -40, 0, -HY, HY, 420, 620, m['black'], c, bevel=6))
    for k, side in ((-1, 'L'), (1, 'R')):
        for xr in (60.0, 3200.0):
            parts.append(g.box(f, f'reflector_{side}_{int(xr)}', xr - 45, xr + 45, k * (HY + 1), k * (HY + 3), ENC_Z0 - 150, ENC_Z0 - 110, m['red'], c))
        parts.append(g.box(f, f'jack_{side}', 3000, 3060, k * (HY - 80), k * (HY - 20), 380, ENC_Z0 - 40, m['galv'], c, bevel=3))
        parts.append(g.cylinder(f, f'jack_foot_{side}', (3030, k * (HY - 50), 380), 'z', 55, 14, m['galv'], c, segments=16))
    return parts


def wheels(f, m, c):
    parts = []
    r = TYRE_D / 2
    rr = 220.0
    tyre = [(rr, -TYRE_W / 2 + 25), (rr + 12, -TYRE_W / 2 + 6), (r - 60, -TYRE_W / 2), (r - 20, -TYRE_W / 2 + 14), (r, -TYRE_W / 2 + 55),
            (r, TYRE_W / 2 - 55), (r - 20, TYRE_W / 2 - 14), (r - 60, TYRE_W / 2), (rr + 12, TYRE_W / 2 - 6), (rr, TYRE_W / 2 - 25)]
    rim = [(0.0, 70.0), (80.0, 70.0), (95.0, 52.0), (170.0, 44.0), (rr - 2, 40.0), (rr + 8, 90.0), (rr + 8, 105.0)]
    for x, label in ((REAR_AXLE, 'rear'), (FRONT_AXLE, 'front')):
        for k in (-1, 1):
            y = k * TRACK / 2
            n = f'{label}_{"L" if k < 0 else "R"}'
            parts.append(g.lathe(f, f'tyre_{n}', (x, y, r), (k, 0, 0), tyre, m['rubber'], c, segments=48))
            parts.append(g.lathe(f, f'rim_{n}', (x, y, r), (k, 0, 0), [(a, -b) for a, b in rim], m['rim'], c, segments=40))
            parts.append(g.cylinder(f, f'hubcap_{n}', (x, y + k * 70, r), 'y', 60, 26, m['hub'], c, segments=24, bevel=4))
            for b in range(6):
                a = 2 * math.pi * b / 6
                parts.append(g.cylinder(f, f'lug_{n}_{b}', (x + 85 * math.cos(a), y + k * 62, r + 85 * math.sin(a)), 'y', 10, 20, m['hub'], c, segments=10))
            y_lo, y_hi = sorted((k * (TRACK / 2 + TYRE_W / 2 + 25), k * (TRACK / 2 - TYRE_W / 2 - 25)))
            parts.append(g.box(f, f'fender_{n}', x - 400, x + 400, y_lo, y_hi, r + 230, r + 245, m['white'], c, bevel=5))
        parts.append(g.cylinder(f, f'axle_{label}', (x, 0, r), 'y', 45, TRACK, m['galv'], c, segments=20))
    parts.append(g.cylinder(f, 'fifth_wheel', (FRONT_AXLE, 0, 400), 'z', 300, 30, m['galv'], c, segments=48, bevel=4))
    return parts


def towbar(f, m, c):
    parts = []
    parts.append(g.prism_xz(f, 'tow_bar', [(3300, 420), (3300, 500), (3850, 330), (3962, 560), (3920, 560), (3830, 390), (3300, 380)], -55, 55, m['galv'], c, bevel=3))
    parts.append(g.lathe(f, 'tow_eye', (3950, 0, 560), (1, 0, 0), [(45, -16), (68, -16), (68, 16), (45, 16)], m['galv'], c, segments=32))
    parts.append(g.box(f, 'tow_brake_pad', 3380, 3520, -170, 170, 300, 380, m['black'], c, bevel=4))
    parts.append(g.tube(f, 'tow_handle', [(3700, -55, 400), (3720, -140, 480), (3740, -140, 600), (3760, -55, 640)], 12, m['galv'], c, segments=12))
    return parts


def enclosure(f, m, c):
    parts = []
    parts.append(g.box(f, 'enc_lower', ENC_X0, ENC_X1, -HY + 3, HY - 3, ENC_Z0, 1500, m['white'], c, bevel=10))
    hood = [(ENC_X0, 1500), (ENC_X0, ENC_Z1 - 80), (400, ENC_Z1), (2300, ENC_Z1), (2900, 1880), (3250, 1700), (3250, 1500)]
    parts.append(g.prism_xz(f, 'enc_hood', hood, -HY + 3, HY - 3, m['white'], c, bevel=8))
    parts.append(g.box(f, 'rear_grille_frame', ENC_X0 - 12, ENC_X0, -600, 600, 760, 1460, m['black'], c, bevel=4))
    for i in range(14):
        parts.append(g.box(f, f'rear_louvre_{i}', ENC_X0 - 22, ENC_X0 - 10, -560, 560, 790 + i * 48, 820 + i * 48, m['grille'], c, bevel=1.5))
    parts.append(g.box(f, 'top_grille_frame', 400, 1500, -600, 600, ENC_Z1 - 2, ENC_Z1 + 14, m['black'], c, bevel=3))
    for i in range(10):
        parts.append(g.box(f, f'top_louvre_{i}', 430 + i * 105, 480 + i * 105, -570, 570, ENC_Z1 + 10, ENC_Z1 + 22, m['grille'], c, bevel=1))
    parts.append(g.cylinder(f, 'exhaust_stack', (1900, -420, ENC_Z1 + 150), 'z', 55, 300, m['black'], c, segments=24, bevel=3))
    parts.append(g.cylinder(f, 'exhaust_cap', (1900, -420, ENC_Z1 + 320), 'z', 70, 20, m['galv'], c, segments=24, bevel=3))
    for k in (-1, 1):
        y = k * HY
        for x0, x1 in ((ENC_X0 + 8, 1100), (1110, 2200), (2210, ENC_X1 - 8)):
            parts.append(g.box(f, f'panel_{k}_{int(x0)}', x0, x1, y - k * 2, y + k * 2, ENC_Z0 + 40, 1480, m['white'], c, bevel=4))
            parts.append(g.box(f, f'handle_{k}_{int(x0)}', x0 + 30, x0 + 130, y + k * 2, y + k * 24, 1000, 1030, m['black'], c, bevel=4))
        for xh in (1105.0, 2205.0):
            for zh in (800.0, 1380.0):
                parts.append(g.box(f, f'hinge_{k}_{int(xh)}_{int(zh)}', xh - 20, xh + 20, y + k * 2, y + k * 12, zh - 40, zh + 40, m['black'], c, bevel=3))
        parts.append(g.box(f, f'hood_stripe_{k}', 300, 2300, y + k * 1, y + k * 3, 1540, 1590, m['yellow'], c))
        parts.append(g.box(f, f'work_light_{k}', 3200, 3240, k * 640, k * 760, 1620, 1700, m['black'], c, bevel=3))
        parts.append(g.box(f, f'work_lens_{k}', 3240, 3246, k * 650, k * 750, 1630, 1690, m['amber'], c))
    parts.append(g.box(f, 'front_panel', ENC_X1, ENC_X1 + 12, -HY + 3, HY - 3, ENC_Z0, 1700, m['white'], c, bevel=6))
    parts.append(g.box(f, 'ctrl_panel', ENC_X1 + 12, ENC_X1 + 40, -330, 130, 1050, 1500, m['black'], c, bevel=6))
    parts.append(g.box(f, 'ctrl_screen', ENC_X1 + 40, ENC_X1 + 46, -260, -60, 1280, 1430, m['glass'], c))
    for i, col in enumerate(('red', 'yellow', 'amber', 'red')):
        parts.append(g.cylinder(f, f'ctrl_btn_{i}', (ENC_X1 + 44, -250 + i * 85, 1150), 'x', 28, 12, m[col], c, segments=20, bevel=3))
    parts.append(g.cylinder(f, 'estop_base', (ENC_X1 + 44, 20, 1150), 'x', 50, 14, m['yellow'], c, segments=24, bevel=3))
    parts.append(g.cylinder(f, 'estop_button', (ENC_X1 + 56, 20, 1150), 'x', 36, 20, m['red'], c, segments=24, bevel=8))
    parts.append(g.cylinder(f, 'fuel_cap', (ENC_X1 + 10, -550, 800), 'x', 45, 20, m['grey'], c, segments=28, bevel=4))
    parts.append(g.cylinder(f, 'beacon_base', (1000, 0, ENC_Z1 + 18), 'z', 70, 36, m['black'], c, segments=24, bevel=4))
    parts.append(g.lathe(f, 'beacon_lens', (1000, 0, ENC_Z1 + 36), 'z', [(0.0, 100.0), (34.0, 96.0), (54.0, 82.0), (56.0, 0.0)], m['amber'], c, segments=32))
    return parts


def hose_stowage(f, m, c):
    parts = []
    px, py, pz = PORT
    parts.append(g.cylinder(f, 'hose_port_flange', (px + 20, py, pz), 'x', 95, 40, m['galv'], c, segments=32, bevel=3))
    parts.append(g.cylinder(f, 'hose_port_cuff', (px + 70, py, pz), 'x', 62, 80, m['black'], c, segments=24, bevel=3))
    parts.append(g.box(f, 'hose_cradle_arm', px, px + 420, py + 160, py + 220, 780, 900, m['galv'], c, bevel=4))
    parts.append(g.tube(f, 'hose_cradle_hoop', [(px + 420, py + 200, 900), (px + 420, py + 200, 780), (px + 420, py - 120, 760), (px + 420, py - 220, 840)], 14, m['galv'], c, segments=12))
    return parts


def hose(f, m, c):
    parts = []
    px, py, pz = PORT
    cx, cy, cz = CONNECTOR
    path = [(px + 70, py, pz), (px + 300, py + 30, pz - 160), (px + 600, py + 160, 430), (px + 1000, py + 480, 140), (px + 1500, py + 520, 100),
            (cx - 800, cy - 380, 120), (cx - 400, cy - 120, 500), (cx - 130, cy - 20, 1200), (cx, cy, cz - 520)]
    pts = g.smooth_path(path, 10)
    parts.append(g.tube(f, 'hose_body', pts, HOSE_RADIUS, m['hose'], c, segments=16))
    step = max(1, len(pts) // 22)
    for i in range(2, len(pts) - 2, step):
        x, y, z = pts[i]
        parts.append(g.cylinder(f, f'hose_rib_{i}', (x, y, z), 'z', HOSE_RADIUS + 6, 12, m['black'], c, segments=14))
    parts.append(g.cylinder(f, 'hose_nozzle_collar', (cx, cy, cz - 430), 'z', 62, 120, m['galv'], c, segments=24, bevel=3))
    parts.append(g.lathe(f, 'hose_nozzle_body', (cx, cy, cz - 340), 'z', [(0.0, 0.0), (55.0, 0.0), (60.0, 50.0), (48.0, 230.0), (40.0, 300.0), (0.0, 300.0)], m['galv'], c, segments=28))
    parts.append(g.cylinder(f, 'hose_nozzle_ring', (cx, cy, cz - 150), 'z', 70, 24, m['yellow'], c, segments=28, bevel=3))
    parts.append(g.tube(f, 'hose_nozzle_handle', [(cx - 50, cy - 60, cz - 380), (cx - 50, cy - 150, cz - 320), (cx - 50, cy - 150, cz - 220)], 12, m['black'], c, segments=12))
    return parts


def detail(f, m, c):
    parts = []
    for k in (-1, 1):
        y = k * HY
        for z0 in (ENC_Z0 + 30,):
            parts.append(g.box(f, f'bumper_rail_{k}', 0, 3300, y + k * 2, y + k * 70, z0 - 70, z0 + 90, m['black'], c, bevel=8))
        for i in range(34):
            x = 80 + i * 95
            parts.append(g.cylinder(f, f'rail_rivet_{k}_{i}', (x, y + k * 72, ENC_Z0 + 20), 'y', 13, 8, m['hub'], c, segments=16, bevel=1.5))
        for i, x in enumerate((ENC_X0 + 10, 1105.0, 2205.0, ENC_X1 - 10)):
            parts.append(g.box(f, f'post_{k}_{i}', x - 45, x + 45, y - k * 4, y + k * 8, ENC_Z0, 1500, m['galv'], c, bevel=4))
            for zz in range(700, 1500, 90):
                parts.append(g.cylinder(f, f'post_bolt_{k}_{i}_{zz}', (x, y + k * 9, zz), 'y', 9, 6, m['hub'], c, segments=12))
        parts.append(g.box(f, f'ctrl_window_{k}', 1160, 1520, y + k * 1, y + k * 8, 900, 1450, m['black'], c, bevel=5))
        parts.append(g.box(f, f'ctrl_window_glass_{k}', 1200, 1480, y + k * 8, y + k * 12, 1250, 1410, m['glass'], c))
        for i in range(5):
            parts.append(g.cylinder(f, f'ctrl_window_knob_{k}_{i}', (1230 + i * 60, y + k * 10, 1130), 'y', 20, 12, m['red' if i == 0 else 'yellow'], c, segments=16, bevel=2))
        for xp in (700.0, 2000.0):
            parts.append(g.box(f, f'fork_pocket_{k}_{int(xp)}', xp - 120, xp + 120, k * (HY - 380), k * (HY - 60), 300, 380, m['galv'], c, bevel=3))
            parts.append(g.box(f, f'fork_pocket_void_{k}_{int(xp)}', xp - 110, xp + 110, k * (HY - 372), k * (HY - 68), 308, 372, m['black'], c))
        for i in range(8):
            parts.append(g.box(f, f'front_louvre_{k}_{i}', ENC_X1 + 12, ENC_X1 + 20, k * 120, k * 700, 800 + i * 55, 835 + i * 55, m['grille'], c, bevel=1.5))
        parts.append(g.cylinder(f, f'tail_lamp_{k}', (-44, k * (HY - 120), 520), 'x', 38, 10, m['red'], c, segments=20, bevel=2))
        parts.append(g.cylinder(f, f'stop_lamp_{k}', (-44, k * (HY - 220), 520), 'x', 30, 10, m['amber'], c, segments=20, bevel=2))
        parts.append(g.cylinder(f, f'front_exhaust_{k}', (ENC_X1 - 150, k * (HY - 120), ENC_Z1 - 100), 'z', 40, 260, m['black'], c, segments=20, bevel=2))
    for i in range(30):
        for k in (-1, 1):
            for j in range(3):
                parts.append(g.cylinder(f, f'deck_rivet_{k}_{i}_{j}', (60 + i * 105, k * (HY - 40 - j * 90), ENC_Z0 - 38), 'z', 12, 6, m['hub'], c, segments=24))
    return parts


def build(with_connection=True):
    g.reset_scene()
    f = g.Frame(X0)
    c = g.collection(NAME)
    m = materials()
    chassis(f, m, c)
    wheels(f, m, c)
    towbar(f, m, c)
    enclosure(f, m, c)
    hose_stowage(f, m, c)
    detail(f, m, c)
    for o in hose(f, m, c):
        if o.name.startswith(('hose_nozzle', 'hose_rib', 'hose_body')):
            g.set_anim(o, HOSE_ANIM)
    return c


def liveries(out):
    return {}


VIEWS = {
    'three_quarter_front': dict(res=(800, 560), lens=35, loc=(5.2, -4.8, 2.6), target=(0.8, 0.0, 0.9), samples=16),
    'three_quarter_rear': dict(res=(800, 560), lens=35, loc=(-5.2, 4.5, 2.6), target=(0.3, 0.0, 1.0), samples=16),
    'side_right': dict(res=(800, 450), lens=35, loc=(8.0, 0.9, 1.2), target=(0.0, 0.9, 1.0), samples=16),
}
