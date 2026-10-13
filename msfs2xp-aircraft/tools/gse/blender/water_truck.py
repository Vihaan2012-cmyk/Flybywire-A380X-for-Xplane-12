import gselib as g

NAME = 'water_truck'
X0 = 3300.0
WHEELBASE = 3600.0
REAR_AXLE = 1000.0
FRONT_AXLE = REAR_AXLE + WHEELBASE
TRACK = 1800.0
HY = 1100.0
TYRE_D = 860.0
TYRE_W = 250.0
FRAME_Z0 = 560.0
FRAME_Z1 = 700.0
TANK_X0 = 150.0
TANK_X1 = 3350.0
TANK_R = 640.0
TANK_ZC = 1480.0
CAB_X0 = 3650.0
CAB_X1 = 5000.0
PLATFORM_X0 = -1050.0
PLATFORM_X1 = 150.0
PLATFORM_Z_LOW = 1100.0
PLATFORM_TRAVEL = 2.3

PLATFORM_ANIM = [{'kind': 'trans', 'offset0': (0.0, 0.0, 0.0), 'offset1': (0.0, 0.0, PLATFORM_TRAVEL), 'v0': 0.0, 'v1': 1.0, 'dataref': 'fbw/gse/water/platform'}]
HOSE_ANIM = [{'kind': 'trans', 'offset0': (0.0, 0.0, 0.0), 'offset1': (-0.6, 0.0, 0.0), 'v0': 0.0, 'v1': 1.0, 'dataref': 'fbw/gse/water/hose'}]


def materials():
    return {
        'white': g.mat('gse_white', (236, 237, 233), roughness=0.32),
        'steel': g.mat('gse_stainless', (190, 194, 197), metallic=0.9, roughness=0.3),
        'galv': g.mat('gse_galvanized', (168, 171, 172), metallic=0.85, roughness=0.42),
        'black': g.mat('gse_black', (22, 22, 23), roughness=0.55),
        'glass': g.mat('gse_glass', (30, 40, 48), roughness=0.08),
        'rubber': g.mat('gse_rubber', (28, 28, 28), roughness=0.88),
        'rim': g.mat('gse_rim', (150, 150, 148), metallic=0.8, roughness=0.4),
        'amber': g.mat('gse_amber_lens', (255, 140, 10), roughness=0.12),
        'red': g.mat('gse_red', (185, 18, 20), roughness=0.3),
        'yellow': g.mat('gse_yellow', (245, 200, 0), roughness=0.35),
        'blue': g.mat('gse_water_blue', (20, 90, 170), roughness=0.35),
        'hose': g.mat('gse_hose', (40, 70, 140), roughness=0.6),
        'grille': g.mat('gse_grille', (14, 14, 14), metallic=0.3, roughness=0.6),
        'deck': g.mat('gse_deck', (70, 72, 74), metallic=0.5, roughness=0.7),
    }


def chassis(f, m, c):
    for k in (-1, 1):
        g.box(f, f'frame_rail_{k}', -300, CAB_X1 - 300, k * 380 - 50, k * 380 + 50, FRAME_Z0, FRAME_Z1, m['galv'], c, bevel=6)
        g.box(f, f'rear_light_{k}', -440, -420, k * 880, k * 1020, 640, 720, m['red'], c)
        g.box(f, f'mudguard_rear_{k}', REAR_AXLE - 430, REAR_AXLE + 430, k * (TRACK / 2 + 50), k * (TRACK / 2 + 140), 900, 940, m['white'], c, bevel=8)
        g.box(f, f'stabiliser_{k}', 380, 460, k * 1000, k * 1060, 150, FRAME_Z0, m['yellow'], c, bevel=3)
        g.box(f, f'stabiliser_pad_{k}', 330, 510, k * 975, k * 1085, 100, 150, m['black'], c, bevel=5)
    g.box(f, 'rear_bumper', -420, -300, -HY, HY, 520, 760, m['galv'], c, bevel=8)
    g.box(f, 'bumper_front', CAB_X1 - 60, CAB_X1 + 120, -HY, HY, 480, 700, m['black'], c, bevel=18)
    g.box(f, 'rear_axle_beam', REAR_AXLE - 70, REAR_AXLE + 70, -TRACK / 2 + 150, TRACK / 2 - 150, TYRE_D / 2 - 60, TYRE_D / 2 + 60, m['black'], c, bevel=10)
    g.box(f, 'front_axle_beam', FRONT_AXLE - 60, FRONT_AXLE + 60, -TRACK / 2 + 150, TRACK / 2 - 150, TYRE_D / 2 - 50, TYRE_D / 2 + 50, m['black'], c, bevel=10)
    g.box(f, 'subframe', TANK_X0 - 50, TANK_X1 + 50, -HY + 100, HY - 100, FRAME_Z1, FRAME_Z1 + 110, m['galv'], c, bevel=5)
    g.box(f, 'fuel_tank_diesel', 1500, 2300, HY - 160, HY + 60, 450, 680, m['galv'], c, bevel=20)


def wheels(f, m, c):
    r = TYRE_D / 2
    rr = 230.0
    tyre = [(rr, -TYRE_W / 2 + 20), (r - 50, -TYRE_W / 2), (r - 12, -TYRE_W / 2 + 14), (r, -TYRE_W / 2 + 50), (r, TYRE_W / 2 - 50), (r - 12, TYRE_W / 2 - 14), (r - 50, TYRE_W / 2), (rr, TYRE_W / 2 - 20)]
    rim = [(0.0, 60.0), (90.0, 60.0), (150.0, 40.0), (rr - 4, 30.0), (rr + 6, 90.0)]
    for x, tag, dual in ((REAR_AXLE, 'rear', True), (FRONT_AXLE, 'front', False)):
        for k in (-1, 1):
            y = k * TRACK / 2
            side = 'L' if k < 0 else 'R'
            ys = (y, y + k * (TYRE_W + 20)) if dual else (y,)
            for i, yy in enumerate(ys):
                n = f'{tag}_{side}_{i}'
                g.lathe(f, f'tyre_{n}', (x, yy, r), (k, 0, 0), tyre, m['rubber'], c, segments=40)
                g.lathe(f, f'rim_{n}', (x, yy, r), (k, 0, 0), [(rad, -z) for rad, z in rim], m['rim'], c, segments=32)
            g.cylinder(f, f'hub_{tag}_{side}', (x, y + k * 70, r), 'y', 70, 40, m['rim'], c, segments=20, bevel=4)


def tank(f, m, c):
    length = TANK_X1 - TANK_X0
    g.cylinder(f, 'water_tank', ((TANK_X0 + TANK_X1) / 2, 0, TANK_ZC), 'x', TANK_R, length - 120, m['steel'], c, segments=48)
    for xe, d in ((TANK_X0 + 60, -1), (TANK_X1 - 60, 1)):
        g.lathe(f, f'tank_dome_{d}', (xe, 0, TANK_ZC), (d, 0, 0), [(0.0, 120.0), (200.0, 112.0), (420.0, 92.0), (560.0, 60.0), (TANK_R - 4, 0.0)], m['steel'], c, segments=48)
    for xb in (TANK_X0 + 40, (TANK_X0 + TANK_X1) / 2, TANK_X1 - 40):
        g.cylinder(f, f'tank_band_{int(xb)}', (xb, 0, TANK_ZC), 'x', TANK_R + 12, 40, m['galv'], c, segments=48)
    for xs in (TANK_X0 + 400, TANK_X1 - 400):
        g.box(f, f'tank_saddle_{int(xs)}', xs - 120, xs + 120, -HY + 180, HY - 180, FRAME_Z1 + 100, TANK_ZC - 380, m['galv'], c, bevel=5)
    g.cylinder(f, 'manway', (1800, 0, TANK_ZC + TANK_R + 20), 'z', 220, 70, m['galv'], c, segments=32, bevel=8)
    g.cylinder(f, 'manway_lid', (1800, 0, TANK_ZC + TANK_R + 70), 'z', 190, 30, m['steel'], c, segments=32, bevel=6)
    g.box(f, 'tank_walkway', 600, 3000, -260, 260, TANK_ZC + TANK_R + 10, TANK_ZC + TANK_R + 25, m['deck'], c)
    g.box(f, 'pump_cabinet', TANK_X0 + 100, 1100, -HY, HY, FRAME_Z1 + 120, 1000, m['white'], c, bevel=10)
    for k in (-1, 1):
        g.box(f, f'level_gauge_{k}', 2600, 2680, k * (TANK_R - 20), k * (TANK_R + 6), 1100, 1700, m['blue'], c, bevel=4)
        g.box(f, f'tank_light_{k}', 3330, 3350, k * 400, k * 520, 1000, 1060, m['amber'], c)


def cab(f, m, c):
    profile = [(CAB_X0, FRAME_Z1), (CAB_X1, FRAME_Z1), (CAB_X1, 1500.0), (CAB_X1 - 170, 2020.0), (CAB_X1 - 520, 2420.0), (CAB_X0 + 150, 2480.0), (CAB_X0, 2480.0)]
    g.prism_xz(f, 'cab_body', profile, -HY + 40, HY - 40, m['white'], c, bevel=22)
    g.prism_xz(f, 'windscreen', [(CAB_X1 - 165, 2030.0), (CAB_X1 - 510, 2395.0), (CAB_X1 - 523, 2395.0), (CAB_X1 - 190, 2030.0)], -HY + 140, HY - 140, m['glass'], c)
    for k in (-1, 1):
        y = k * (HY - 40)
        g.box(f, f'door_window_{k}', CAB_X0 + 450, CAB_X1 - 560, y - k * 4, y + k * 8, 1750, 2300, m['glass'], c, bevel=8)
        g.box(f, f'door_handle_{k}', CAB_X0 + 380, CAB_X0 + 520, y, y + k * 30, 1500, 1540, m['black'], c, bevel=4)
        g.box(f, f'door_gap_{k}', CAB_X0 + 400, CAB_X0 + 408, y, y + k * 3, 800, 2300, m['black'], c)
        g.box(f, f'headlamp_{k}', CAB_X1 - 40, CAB_X1 + 5, k * 700, k * 950, 1000, 1160, m['white'], c, bevel=12)
        g.box(f, f'mirror_arm_{k}', CAB_X1 - 600, CAB_X1 - 560, k * (HY + 20), k * (HY + 280), 1950, 1990, m['black'], c, bevel=3)
        g.box(f, f'mirror_{k}', CAB_X1 - 640, CAB_X1 - 540, k * (HY + 270), k * (HY + 330), 1900, 2300, m['black'], c, bevel=10)
        g.box(f, f'step_{k}', CAB_X0 + 250, CAB_X0 + 800, k * (HY - 10), k * (HY + 170), 470, 510, m['deck'], c, bevel=3)
    g.box(f, 'grille_front', CAB_X1 - 42, CAB_X1 + 4, -450, 450, 1000, 1380, m['grille'], c, bevel=6)
    g.box(f, 'roof_beacon_base', CAB_X0 + 600, CAB_X0 + 760, -80, 80, 2480, 2520, m['black'], c, bevel=4)
    g.lathe(f, 'roof_beacon', (CAB_X0 + 680, 0, 2520), 'z', [(0.0, 130.0), (45.0, 120.0), (75.0, 60.0), (80.0, 0.0)], m['amber'], c, segments=24)


def platform_lift(f, m, c):
    x0, x1 = PLATFORM_X0, PLATFORM_X1
    xm = (x0 + x1) / 2
    for k in (-1, 1):
        g.box(f, f'lift_base_rail_{k}', x0 + 100, x1 + 50, k * 800 - 40, k * 800 + 40, 740, 820, m['yellow'], c, bevel=4)
    g.box(f, 'lift_base_cross', x0 + 100, x1 + 50, -800, 800, 740, 820, m['galv'], c, bevel=4)
    moving = []
    for k in (-1, 1):
        for i, zs in enumerate((900, 1500)):
            moving.append(g.tube(f, f'scissor_a_{k}_{i}', [(xm - 420, k * 700, zs), (xm + 420, k * 700, zs + 600)], 32, m['galv'], c, segments=12))
            moving.append(g.tube(f, f'scissor_b_{k}_{i}', [(xm + 420, k * 700, zs), (xm - 420, k * 700, zs + 600)], 32, m['galv'], c, segments=12))
    zl = PLATFORM_Z_LOW
    moving.append(g.box(f, 'platform_deck', x0, x1, -HY, HY, zl, zl + 50, m['deck'], c, bevel=4))
    moving.append(g.box(f, 'platform_deck_frame', x0 - 20, x1 + 20, -HY - 20, HY + 20, zl - 40, zl, m['yellow'], c, bevel=4))
    for k in (-1, 1):
        moving.append(g.tube(f, f'rail_side_top_{k}', [(x0 + 20, k * (HY - 40), zl + 1050), (x1 - 20, k * (HY - 40), zl + 1050)], 22, m['yellow'], c, segments=12))
        moving.append(g.tube(f, f'rail_side_mid_{k}', [(x0 + 20, k * (HY - 40), zl + 550), (x1 - 20, k * (HY - 40), zl + 550)], 18, m['yellow'], c, segments=10))
        moving.append(g.box(f, f'kick_{k}', x0, x1, k * (HY - 55), k * (HY - 35), zl + 50, zl + 200, m['yellow'], c, bevel=2))
        for xp in (x0 + 25, x1 - 25):
            moving.append(g.cylinder(f, f'rail_post_{k}_{int(xp)}', (xp, k * (HY - 40), zl + 575), 'z', 20, 1050, m['yellow'], c, segments=10))
    moving.append(g.tube(f, 'rail_back_top', [(x0 + 20, -HY + 40, zl + 1050), (x0 + 20, HY - 40, zl + 1050)], 22, m['yellow'], c, segments=12))
    moving.append(g.tube(f, 'rail_back_mid', [(x0 + 20, -HY + 40, zl + 550), (x0 + 20, HY - 40, zl + 550)], 18, m['yellow'], c, segments=10))
    moving.append(g.box(f, 'control_post', x0 + 60, x0 + 200, 700, 900, zl + 50, zl + 1000, m['black'], c, bevel=8))
    moving.append(g.box(f, 'control_estop', x0 + 40, x0 + 62, 760, 840, zl + 800, zl + 880, m['red'], c, bevel=4))
    for o in moving:
        g.set_anim(o, PLATFORM_ANIM)


def hose(f, m, c):
    g.cylinder(f, 'hose_reel_drum', (800, -HY + 60, 900), 'y', 180, 280, m['galv'], c, segments=28, bevel=4)
    zl = PLATFORM_Z_LOW
    pts = [(800, -HY + 60, 1000), (200, -HY + 100, 1500), (-300, -700, zl + 160), (-700, -300, zl + 200), (-1250, -200, zl + 420), (-1700, -100, zl + 1100)]
    run = g.tube(f, 'hose_run', g.smooth_path(pts, 8), 38, m['hose'], c, segments=14)
    coupling = g.cylinder(f, 'hose_coupling', (-1730, -90, zl + 1150), 'x', 55, 140, m['rim'], c, segments=18, bevel=4)
    cap = g.cylinder(f, 'hose_cap', (-1810, -90, zl + 1150), 'x', 60, 30, m['black'], c, segments=18)
    for o in (run, coupling, cap):
        g.set_anim(o, HOSE_ANIM)


def build(with_connection=True):
    g.reset_scene()
    f = g.Frame(X0)
    c = g.collection(NAME)
    m = materials()
    chassis(f, m, c)
    wheels(f, m, c)
    tank(f, m, c)
    cab(f, m, c)
    platform_lift(f, m, c)
    hose(f, m, c)
    return c


def liveries(out):
    return {}


VIEWS = {
    'three_quarter_front': dict(res=(800, 600), samples=16, lens=35, loc=(7.5, 5.0, 3.5), target=(0.0, -1.7, 1.4)),
    'three_quarter_rear': dict(res=(800, 600), samples=16, lens=35, loc=(-7.0, -11.0, 4.0), target=(0.0, -2.5, 1.6)),
    'side_right': dict(res=(800, 400), samples=16, lens=35, loc=(14.0, -1.7, 1.6), target=(0.0, -1.7, 1.5)),
}
