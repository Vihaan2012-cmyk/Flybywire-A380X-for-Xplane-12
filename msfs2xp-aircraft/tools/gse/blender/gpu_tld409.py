import math

import bpy

import os

import gselib as g

DECALS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'decals')

NAME = 'gpu_tld409'
X0 = 1338.0
L = 2572.0
W_BOX = 1114.0
HY = W_BOX / 2
TRAY_Y = 862.0
REAR_AXLE = 383.0
FRONT_AXLE = 383.0 + 1911.0
TRACK = 1482.0
TYRE_D = 480.0
TYRE_W = 217.0
ENC_Z0 = 752.0
ENC_Z1 = 1478.0
ENC_X0 = 72.0
BOX_X1 = 1945.0

RECEPTACLE = (250.0, 3550.0, 2590.0)
RECEPTACLE_SPACING = 160.0
GLANDS = (300.0, 368.0)
PLUG_ANIM = [{'kind': 'trans', 'offset0': (0.0, 0.0, -0.15), 'offset1': (0.0, 0.0, 0.15), 'v0': -0.15, 'v1': 0.15, 'dataref': 'fbw/gse/gpu/plug_dz'}]
POCKETS = (X0 - 372.5, X0 + 372.5)


def materials():
    return {
        'white': g.mat('gse_white', (236, 237, 233), roughness=0.32),
        'galv': g.mat('gse_galvanized', (168, 171, 172), metallic=0.85, roughness=0.42),
        'black': g.mat('gse_black', (22, 22, 23), roughness=0.55),
        'panel': g.mat('gse_panel_black', (30, 31, 33), roughness=0.4),
        'glass': g.mat('gse_screen', (18, 22, 26), roughness=0.08),
        'rubber': g.mat('gse_rubber', (28, 28, 28), roughness=0.88),
        'rim': g.mat('gse_rim', (34, 34, 35), metallic=0.4, roughness=0.5),
        'hub': g.mat('gse_hub', (150, 150, 148), metallic=0.9, roughness=0.35),
        'amber': g.mat('gse_amber_lens', (255, 140, 10), roughness=0.12),
        'red': g.mat('gse_red', (185, 18, 20), roughness=0.3),
        'yellow': g.mat('gse_yellow', (245, 200, 0), roughness=0.35),
        'blue': g.mat('gse_tld_blue', (16, 40, 140), roughness=0.35),
        'green': g.mat('gse_green', (0, 140, 70), roughness=0.4),
        'cable': g.mat('gse_cable', (20, 20, 20), roughness=0.7),
        'plug': g.mat('gse_plug', (26, 26, 26), roughness=0.6),
        'plug_ring': g.mat('gse_plug_ring', (240, 190, 0), roughness=0.4),
        'grille': g.mat('gse_grille', (14, 14, 14), metallic=0.3, roughness=0.6),
        'fuel_cap': g.mat('gse_fuel_cap', (90, 92, 94), metallic=0.7, roughness=0.4),
        'fuel_green': g.mat('gse_fuel_green', (20, 95, 45), roughness=0.45),
    }


def arch_profile(x_center, radius, z_low, n=12):
    pts = []
    for i in range(n + 1):
        a = math.pi - math.pi * i / n
        pts.append((x_center + radius * math.cos(a), max(z_low, 240.0 + radius * math.sin(a))))
    return pts


def wall_profile():
    pts = [(0.0, 398.0)]
    a0 = math.degrees(math.asin((398.0 - 240.0) / 300.0))
    for i in range(13):
        a = math.radians(180.0 - a0 - (180.0 - 2 * a0 - (90.0 - a0) * 0 - a0) * i / 12) if False else math.radians((180.0 - a0) * (1 - i / 12))
        pts.append((REAR_AXLE + 300.0 * math.cos(a), 240.0 + 300.0 * math.sin(a)))
    pts.append((REAR_AXLE + 300.0, 230.0))
    for xc in POCKETS:
        pts += [(xc - 121.5, 230.0), (xc - 121.5, 318.0), (xc + 121.5, 318.0), (xc + 121.5, 230.0)]
    pts.append((FRONT_AXLE - 300.0, 230.0))
    a1 = math.degrees(math.acos((2380.0 - FRONT_AXLE) / 300.0))
    for i in range(13):
        a = math.radians(180.0 - (180.0 - a1) * i / 12)
        pts.append((FRONT_AXLE + 300.0 * math.cos(a), 240.0 + 300.0 * math.sin(a)))
    pts += [(2380.0, 640.0), (0.0, 640.0)]
    return pts


def chassis(f, m, c):
    parts = []
    parts.append(g.box(f, 'chassis_center', 0, 2380, -HY, HY, 203, ENC_Z0, m['galv'], c, bevel=4))
    for k, name in ((-1, 'L'), (1, 'R')):
        outer = wall_profile()
        y_out = k * TRAY_Y
        y_in = k * (TRAY_Y - 6)
        parts.append(g.prism_xz(f, f'tray_wall_{name}', outer, min(y_out, y_in), max(y_out, y_in), m['galv'], c, bevel=1.5))
        floor_z = 440.0
        parts.append(g.box(f, f'tray_floor_{name}_rear', 0, REAR_AXLE - 300, k * HY, k * TRAY_Y, floor_z - 6, floor_z, m['galv'], c))
        parts.append(g.box(f, f'tray_floor_{name}_mid', REAR_AXLE + 300, FRONT_AXLE - 300, k * HY, k * TRAY_Y, floor_z - 6, floor_z, m['galv'], c))
        parts.append(g.box(f, f'tray_end_{name}', 0, 6, k * HY, k * TRAY_Y, 398, 640, m['galv'], c, bevel=1))
        parts.append(g.tube(f, f'tray_rail_{name}', [(-20, k * (TRAY_Y + 4), 646), (2300, k * (TRAY_Y + 4), 646), (2470, k * (HY + 150), 646)], 21, m['galv'], c, segments=20))
        parts.append(g.cylinder(f, f'rail_sleeve_{name}', (X0, k * (TRAY_Y + 4), 646), 'x', 27, 90, m['galv'], c, segments=20, bevel=2))
        for xp in (REAR_AXLE + 520, FRONT_AXLE - 520):
            parts.append(g.box(f, f'rail_post_{name}_{int(xp)}', xp - 20, xp + 20, k * (TRAY_Y - 14), k * (TRAY_Y + 10), 600, 646, m['galv'], c, bevel=2))
        for xr in (40.0, 2330.0):
            parts.append(g.box(f, f'reflector_{name}_{int(xr)}', xr - 45, xr + 45, k * (TRAY_Y + 1), k * (TRAY_Y + 3), 540, 580, m['red'], c))
    parts.append(g.box(f, 'rear_plate', -6, 0, -TRAY_Y, TRAY_Y, 398, 640, m['galv'], c, bevel=1.5))
    for xc in POCKETS:
        parts.append(g.box(f, f'fork_pocket_{int(xc)}', xc - 121.5, xc + 121.5, -TRAY_Y + 7, TRAY_Y - 7, 203, 318, m['galv'], c, bevel=3))
        parts.append(g.box(f, f'fork_pocket_void_{int(xc)}', xc - 113, xc + 113, -TRAY_Y + 1, TRAY_Y - 1, 211, 310, m['black'], c))
    for yk in (-650, 650):
        parts.append(g.tube(f, f'tie_ring_{yk}', [(-6, yk - 40, 405), (-40, yk - 40, 380), (-40, yk + 40, 380), (-6, yk + 40, 405)], 9, m['galv'], c, segments=10))
    parts.append(g.box(f, 'front_bulkhead', 2380, 2410, -HY, HY, 203, 520, m['galv'], c, bevel=3))
    return parts


def wheels(f, m, c):
    parts = []
    r = TYRE_D / 2
    rr = 154.0
    tyre_profile = [(rr, -TYRE_W / 2 + 18), (rr + 8, -TYRE_W / 2 + 4), (r - 40, -TYRE_W / 2), (r - 12, -TYRE_W / 2 + 10), (r, -TYRE_W / 2 + 40),
                    (r, TYRE_W / 2 - 40), (r - 12, TYRE_W / 2 - 10), (r - 40, TYRE_W / 2), (rr + 8, TYRE_W / 2 - 4), (rr, TYRE_W / 2 - 18)]
    rim_profile = [(0.0, 52.0), (60.0, 52.0), (70.0, 40.0), (120.0, 34.0), (rr - 2, 30.0), (rr + 6, 70.0), (rr + 6, 82.0)]
    for x in (REAR_AXLE, FRONT_AXLE):
        for k in (-1, 1):
            y = k * TRACK / 2
            name = f'{"rear" if x == REAR_AXLE else "front"}_{"L" if k < 0 else "R"}'
            parts.append(g.lathe(f, f'tyre_{name}', (x, y, r), (k, 0, 0), tyre_profile, m['rubber'], c, segments=48))
            parts.append(g.lathe(f, f'rim_{name}', (x, y, r), (k, 0, 0), [(rad, -z) for rad, z in rim_profile], m['rim'], c, segments=40))
            parts.append(g.cylinder(f, f'hubcap_{name}', (x, y + k * 50, r), 'y', 42, 24, m['hub'], c, segments=24, bevel=4))
            for b in range(5):
                a = 2 * math.pi * b / 5
                parts.append(g.cylinder(f, f'lug_{name}_{b}', (x + 58 * math.cos(a), y + k * 48, r + 58 * math.sin(a)), 'y', 9, 18, m['hub'], c, segments=10))
            parts.append(g.cylinder(f, f'axle_stub_{name}', (x, k * (HY + (TRACK / 2 - HY) / 2), r), 'y', 32, TRACK / 2 - HY - 20, m['galv'], c, segments=16))
    parts.append(g.cylinder(f, 'torsion_axle_rear', (REAR_AXLE, 0, r), 'y', 50, 2 * HY - 40, m['galv'], c, segments=20))
    parts.append(g.cylinder(f, 'fifth_wheel', (FRONT_AXLE, 0, 385), 'z', 280, 30, m['galv'], c, segments=48, bevel=4))
    parts.append(g.box(f, 'front_axle_beam', FRONT_AXLE - 60, FRONT_AXLE + 60, -TRACK / 2 + 120, TRACK / 2 - 120, r - 50, r + 50, m['galv'], c, bevel=6))
    return parts


def towbar(f, m, c):
    parts = []
    pivot = (2700.0, 0.0, 300.0)
    top = (2812.0, 0.0, 1450.0)
    dx, dz = top[0] - pivot[0], top[2] - pivot[2]
    n = math.hypot(dx, dz)
    nx, nz = dz / n * 16.0, -dx / n * 16.0
    strip = [(pivot[0] - nx, pivot[2] - nz), (pivot[0] + nx, pivot[2] + nz), (top[0] + nx, top[2] + nz), (top[0] - nx, top[2] - nz)]
    parts.append(g.prism_xz(f, 'tow_bar', strip, -52, 52, m['galv'], c, bevel=3))
    parts.append(g.box(f, 'tow_hitch', 2540, 2700, -130, 130, 220, 330, m['galv'], c, bevel=6))
    parts.append(g.cylinder(f, 'tow_pivot_pin', (2700, 0, 300), 'y', 30, 300, m['galv'], c, segments=16))
    parts.append(g.lathe(f, 'tow_eye', (2817, 0, 1500), (1, 0, 0), [(46, -15), (66, -15), (66, 15), (46, 15), (46, -15)], m['galv'], c, segments=32))
    parts.append(g.tube(f, 'tow_hand_loop', [(2780, -52, 880), (2782, -120, 905), (2790, -125, 1080), (2792, -52, 1110)], 11, m['galv'], c, segments=12))
    parts.append(g.tube(f, 'tow_hook', [(2740, 0, 300), (2770, 0, 160), (2760, 0, 95), (2720, 0, 95)], 18, m['galv'], c, segments=12))
    parts.append(g.box(f, 'tow_brake_pad', 2560, 2660, -180, 180, 150, 215, m['black'], c, bevel=4))
    return parts, pivot, top, (dx / n, dz / n), (nx / 16.0, nz / 16.0)


def enclosure(f, m, c):
    parts = []
    parts.append(g.box(f, 'enc_box', ENC_X0, BOX_X1, -HY + 3, HY - 3, ENC_Z0, ENC_Z1, m['white'], c, bevel=8))
    hood = [(BOX_X1, ENC_Z0), (BOX_X1, ENC_Z1), (2073.0, ENC_Z1), (2300.0, 1462.0), (2470.0, 1432.0), (2534.0, 1405.0),
            (2552.0, 1340.0), (2552.0, 1200.0), (2543.0, 1000.0), (2522.0, 800.0), (2490.0, 620.0), (2433.0, 457.0), (2380.0, 457.0), (2380.0, ENC_Z0)]
    hood_obj = g.prism_xz(f, 'enc_hood', hood, -HY + 3, HY - 3, m['white'], c)
    front = 2380.0 - X0
    g.bevel_where(hood_obj, lambda pt, d: abs(d.x) > 0.9 and pt.y > 2460.0 - X0 and pt.z > 1300.0, 110, 8)
    g.bevel_where(hood_obj, lambda pt, d: abs(abs(pt.x) - (HY - 3)) < 0.5 and abs(d.x) < 0.1 and pt.y > 2420.0 - X0 and pt.z > 470.0, 150, 8)
    g.bevel_where(hood_obj, lambda pt, d: abs(abs(pt.x) - (HY - 3)) < 0.5 and abs(d.x) < 0.1 and pt.z > 1400.0, 45, 5)
    for poly in hood_obj.data.polygons:
        poly.use_smooth = True
    parts.append(hood_obj)
    parts.append(g.prism_xz(f, 'hood_grille_frame', [(2085.0, 1460.0), (2085.0, 1481.0), (2300.0, 1465.0), (2470.0, 1436.0), (2515.0, 1424.0), (2515.0, 1400.0), (2300.0, 1442.0)], -515, 515, m['black'], c, bevel=3))
    parts.append(g.prism_xz(f, 'hood_grille', [(2105.0, 1462.0), (2105.0, 1483.0), (2300.0, 1468.0), (2470.0, 1439.0), (2495.0, 1432.0), (2495.0, 1412.0), (2300.0, 1446.0)], -495, 495, m['grille'], c, bevel=2))
    for k in (-1, 1):
        parts.append(g.box(f, f'hood_front_reflector_{k}', hood_front_x(1290) - 2, hood_front_x(1290) + 3, k * 300, k * 410, 1258, 1298, m['red'], c))
    for k in (-1, 1):
        y = k * HY
        for x0, x1 in ((ENC_X0 + 4, 459.0 - 3), (459.0 + 3, 1221.0 - 3), (1221.0 + 3, BOX_X1 - 4)):
            parts.append(g.box(f, f'panel_{k}_{int(x0)}', x0, x1, y - k * 2, y + k * 2, ENC_Z0 + 8, ENC_Z1 - 10, m['white'], c, bevel=3))
        for xh in (459.0, 1221.0, BOX_X1):
            for zh in (ENC_Z0 + 90, ENC_Z1 - 90):
                parts.append(g.box(f, f'hinge_{k}_{int(xh)}_{int(zh)}', xh - 22, xh + 22, y + k * 2, y + k * 14, zh - 32, zh + 32, m['black'], c, bevel=3))
        for xl in (1221.0 - 70, 1221.0 + 70):
            parts.append(g.box(f, f'latch_{k}_{int(xl)}', xl - 12, xl + 12, y + k * 2, y + k * 30, 1020, 1110, m['black'], c, bevel=4))
        parts.append(g.box(f, f'enc_reflector_rear_{k}', 90, 180, y + k * 2, y + k * 4, 1427, 1470, m['red'], c))
        parts.append(g.box(f, f'hood_reflector_{k}', 2413, 2516, k * (HY - 1), k * (HY + 1), 1310, 1361, m['red'], c))
        parts.append(g.tube(f, f'grab_handle_{k}', [(282, y, 1012), (282, y + k * 34, 996), (282, y + k * 42, 950), (388, y + k * 42, 950), (388, y + k * 34, 996), (388, y, 1012)], 11, m['black'], c, segments=12))
        parts.append(g.tube(f, f'cable_hook_{k}', [(1832, y, 803), (1832, y + k * 42, 792), (1832, y + k * 62, 756), (1832, y + k * 52, 716), (1832, y + k * 26, 708)], 9, m['galv'], c, segments=10))
        for xs, zs in ((2054.0, 1386.0), (2041.0, 700.0)):
            parts.append(g.box(f, f'hood_latch_strap_{k}_{int(zs)}', xs - 55, xs + 55, y + k * 2, y + k * 16, zs - 9, zs + 9, m['black'], c, bevel=3))
            parts.append(g.box(f, f'hood_latch_t_{k}_{int(zs)}', xs + 40, xs + 58, y + k * 2, y + k * 30, zs - 34, zs + 34, m['black'], c, bevel=4))
    end = ENC_X0
    parts.append(g.box(f, 'ctrl_panel', end - 6, end + 2, -305, 305, 1010, 1431, m['panel'], c, bevel=6))
    parts.append(g.box(f, 'ctrl_screen_bezel', end - 9, end - 5, 33, 168, 1261, 1370, m['glass'], c, bevel=2))
    for i in range(8):
        parts.append(g.cylinder(f, f'ctrl_lamp_{i}', (end - 8, -250 + i * 62, 1200), 'x', 11, 6, m['green' if i % 3 else 'yellow'], c, segments=14, bevel=2))
    for i, col in enumerate(('blue', 'red', 'yellow', 'green')):
        parts.append(g.cylinder(f, f'ctrl_btn_{col}', (end - 10, -120 + i * 80, 1090), 'x', 22, 10, m[col], c, segments=20, bevel=3))
    parts.append(g.cylinder(f, 'ctrl_key', (end - 10, 220, 1250), 'x', 30, 14, m['hub'], c, segments=20, bevel=3))
    for name, (ey, ez, r0) in (('estop', (440.0, 870.0, 1.0)), ('estop_panel', (205.0, 1215.0, 0.75))):
        parts.append(g.cylinder(f, f'{name}_base', (end - 8, ey, ez), 'x', 55 * r0, 20, m['yellow'], c, segments=24, bevel=3))
        parts.append(g.cylinder(f, f'{name}_button', (end - 26, ey, ez), 'x', 38 * r0, 22, m['red'], c, segments=24, bevel=8))
    parts.append(g.cylinder(f, 'fuel_recess', (end + 1, -445, 880), 'x', 52, 12, m['black'], c, segments=32))
    parts.append(g.cylinder(f, 'fuel_cap', (end - 6, -445, 880), 'x', 36, 22, m['fuel_green'], c, segments=28, bevel=5))
    parts.append(g.box(f, 'fuel_cap_tether', end - 6, end - 2, -452, -438, 760, 830, m['black'], c))
    parts.append(g.cylinder(f, 'beacon_base', (2021, 0, ENC_Z1 + 20), 'z', 62, 40, m['black'], c, segments=24, bevel=4))
    parts.append(g.lathe(f, 'beacon_lens', (2021, 0, ENC_Z1 + 40), 'z', [(0.0, 91.0), (30.0, 88.0), (48.0, 76.0), (50.0, 0.0)], m['amber'], c, segments=32))
    return parts


def fittings(f, m, c):
    parts = []
    for k in (-1, 1):
        y = k * HY
        wedge = [(y, 1100.0), (y, 1262.0), (y + k * 85, 1262.0), (y + k * 160, 1195.0), (y + k * 160, 1100.0)]
        parts.append(g.prism_yz(f, f'plug_holder_{k}', wedge, 262, 420, m['white'], c, bevel=10))
        parts.append(g.box(f, f'plug_holder_mouth_{k}', 285, 397, y + k * 25, y + k * 145, 1097, 1101, m['black'], c, bevel=4))
        for i, gx in enumerate(GLANDS):
            parts.append(g.cylinder(f, f'cable_gland_{k}_{i}', (gx, y + k * 14, 695), 'y', 25, 28, m['black'], c, segments=20, bevel=3))
    for k in (-1, 1):
        slant = [(k * TRAY_Y, 460.0), (k * TRAY_Y, 640.0), (k * (TRAY_Y - 6), 640.0), (k * (TRAY_Y - 6), 460.0)]
        parts.append(g.prism_yz(f, f'front_tray_side_{k}', slant, 2370, 2380, m['galv'], c))
        p0, p1 = (2380.0, k * TRAY_Y), (2520.0, k * (HY + 70))
        parts.append(g.prism_xz(f, f'front_tray_end_{k}', [(2380.0, 460.0), (2380.0, 640.0), (2520.0, 640.0), (2520.0, 460.0)], *sorted((k * (HY + 70), k * (HY + 76))), m['galv'], c, bevel=2))
        parts.append(g.box(f, f'front_reflector_{k}', 2505, 2522, k * (HY + 90), k * (HY + 190), 545, 585, m['red'], c))
        ring = [(2440, k * (HY + 115), 590), (2490, k * (HY + 115), 590), (2515, k * (HY + 115), 548), (2490, k * (HY + 115), 505), (2440, k * (HY + 115), 505)]
        parts.append(g.tube(f, f'tie_dring_{k}', g.smooth_path(ring, 6), 9, m['galv'], c, segments=10))
    return parts


def fender_profile(xc, r0, r1, n=16):
    outer = [(xc + r1 * math.cos(math.radians(a)), 240.0 + r1 * math.sin(math.radians(a))) for a in [15 + 150 * i / n for i in range(n + 1)]]
    inner = [(xc + r0 * math.cos(math.radians(a)), 240.0 + r0 * math.sin(math.radians(a))) for a in [165 - 150 * i / n for i in range(n + 1)]]
    return outer + inner


def coil(x0, x1, y_in, y_out, z, loops, phase=0.0, n=28):
    pts = []
    cx, cy = (x0 + x1) / 2, (y_in + y_out) / 2
    rx, ry = (x1 - x0) / 2, abs(y_out - y_in) / 2
    for i in range(loops * n + 1):
        a = phase + 2 * math.pi * i / n
        shrink = 1.0 - 0.06 * i / n
        pts.append((cx + rx * shrink * math.cos(a), cy + ry * shrink * math.sin(a), z + 46.0 * (i / n) * 0.5))
    return pts


def plug(f, m, c, name, top, length=250.0, axis='z'):
    x, y, z = top
    body = g.lathe(f, f'{name}_body', (x, y, z - length), axis,
                   [(0.0, 0.0), (24.0, 0.0), (26.0, 50.0), (46.0, 105.0), (55.0, 150.0), (55.0, length - 20.0), (42.0, length), (0.0, length)], m['plug'], c, segments=28)
    ring = g.cylinder(f, f'{name}_ring', (x, y, z - length * 0.42), 'z', 58, 24, m['plug_ring'], c, segments=28, bevel=3)
    grip = g.box(f, f'{name}_grip', x - 14, x + 14, y - 62, y + 62, z - length * 0.85, z - length * 0.6, m['plug'], c, bevel=6)
    return [body, ring, grip]


def stowed(f, m, c, k):
    parts = []
    y = k * HY
    hx, hy = 341.0, y + k * 92
    parts += plug(f, m, c, f'stowed_plug_{k}', (hx, hy, 1180.0), 260.0)
    s_curve = [(hx, hy, 920.0), (hx, hy, 860.0), (hx + 40, hy - k * 10, 790.0), (hx + 130, hy - k * 30, 760.0), (hx + 210, y + k * 70, 720.0),
               (hx + 260, y + k * 150, 660.0), (hx + 300, k * (TRAY_Y - 120), 520.0), (hx + 340, k * (TRAY_Y - 140), 466.0)]
    parts.append(g.tube(f, f'stowed_cable_{k}', g.smooth_path(s_curve, 10), 22, m['cable'], c, segments=14))
    parts.append(g.tube(f, f'stowed_coil_{k}', g.smooth_path(coil(700.0, 1700.0, k * (HY + 45), k * (TRAY_Y - 40), 462.0, 3, 0.3), 2), 22, m['cable'], c, segments=12))
    for i, gx in enumerate(GLANDS):
        lead = [(gx, y + k * 28, 695.0), (gx + 40, y + k * 90, 650.0), (gx + 140, y + k * 140, 520.0), (gx + 300, y + k * 150, 470.0), (gx + 420, y + k * 160 + i * 20, 466.0)]
        parts.append(g.tube(f, f'gland_lead_{k}_{i}', g.smooth_path(lead, 8), 22, m['cable'], c, segments=12))
    return parts


def connection(f, m, c):
    parts = []
    y = HY
    for i, (gx, dy) in enumerate(zip(GLANDS, (-RECEPTACLE_SPACING / 2, RECEPTACLE_SPACING / 2))):
        rx, ry, rz = RECEPTACLE
        path = [(gx, y + 28, 695), (gx + 15, y + 130, 670), (gx + 30, TRAY_Y - 40, 640), (gx + 45, TRAY_Y + 30, 670), (gx + 60, TRAY_Y + 140, 520),
                (gx + 50, TRAY_Y + 260, 90), (gx + 20, 1500, 23), (rx + 40, 2400, 23), (rx + dy, ry - 420, 40), (rx + dy, ry - 120, 420)]
        hang = [(rx + dy, ry - 60, 900), (rx + dy, ry - 20, 1700), (rx + dy, ry, rz - 420)]
        parts.append(g.tube(f, f'cable_{i}', g.smooth_path(path, 10), 22, m['cable'], c, segments=14))
        parts.append(g.tube(f, f'anim_plug_{i}_cable', g.smooth_path([(rx + dy, ry - 120, 270), (rx + dy, ry - 120, 420)] + hang, 10), 22, m['cable'], c, segments=14))
        parts.append(g.lathe(f, f'anim_plug_{i}_body', (rx + dy, ry, rz - 400), 'z',
                             [(0.0, 0.0), (24.0, 0.0), (26.0, 60.0), (48.0, 120.0), (55.0, 240.0), (55.0, 380.0), (40.0, 400.0), (0.0, 400.0)], m['plug'], c, segments=28))
        parts.append(g.cylinder(f, f'anim_plug_{i}_ring', (rx + dy, ry, rz - 150), 'z', 58, 26, m['plug_ring'], c, segments=28, bevel=3))
    for o in parts:
        if o.name.startswith('anim_plug_'):
            g.set_anim(o, PLUG_ANIM)
    return parts


HOOD_FRONT = [(2490.0, 620.0), (2522.0, 800.0), (2543.0, 1000.0), (2552.0, 1200.0), (2552.0, 1340.0)]


def hood_front_x(z):
    pts = HOOD_FRONT
    for (x0, z0), (x1, z1) in zip(pts, pts[1:]):
        if z0 <= z <= z1:
            return x0 + (x1 - x0) * (z - z0) / (z1 - z0)
    return pts[-1][0] if z > pts[-1][1] else pts[0][0]


def dmat(name):
    return g.image_mat(f'decal_{name}', os.path.join(DECALS, f'{name}.png'), roughness=0.4)


def side_decal(f, c, name, img, x0, x1, z0, z1, k, off):
    y = k * off
    corners = [(x0, y, z0), (x1, y, z0), (x1, y, z1), (x0, y, z1)] if k > 0 else [(x1, y, z0), (x0, y, z0), (x0, y, z1), (x1, y, z1)]
    return g.decal(f, name, corners, dmat(img), c)


def rear_decal(f, c, name, img, y0, y1, z0, z1, x):
    return g.decal(f, name, [(x, y0, z0), (x, y1, z0), (x, y1, z1), (x, y0, z1)], dmat(img), c)


def front_decal(f, c, name, img, y0, y1, z0, z1, x_of_z):
    return g.decal(f, name, [(x_of_z(z0), y1, z0), (x_of_z(z0), y0, z0), (x_of_z(z1), y0, z1), (x_of_z(z1), y1, z1)], dmat(img), c)


def decals(f, c):
    off = HY + 2.6
    for k in (-1, 1):
        side_decal(f, c, f'd_tld_{k}', 'tld_logo', 1406, 1778, 1226, 1406, k, off)
        side_decal(f, c, f'd_tld_green_{k}', 'tld_green', 1386, 1791, 1103, 1206, k, off)
        side_decal(f, c, f'd_signs_{k}', 'sign_stack', 1106, 1226, 1021, 1334, k, off)
        side_decal(f, c, f'd_tyre_{k}', 'tyre_pressure', 103, 411, 765, 816, k, off)
    end = ENC_X0 - 1.2
    rear_decal(f, c, 'd_end_tld', 'tld_logo', -120, 230, 780, 960, end)
    rear_decal(f, c, 'd_end_explosion', 'sign_warning', -505, -385, 1300, 1410, end)
    rear_decal(f, c, 'd_end_fuel_spec', 'fuel_label', -515, -375, 1170, 1280, end)
    rear_decal(f, c, 'd_end_diesel', 'diesel', -515, -375, 1110, 1158, end)
    rear_decal(f, c, 'd_end_mand', 'sign_mandatory', 395, 505, 1305, 1415, end)
    rear_decal(f, c, 'd_end_warn2', 'sign_warning', 250, 335, 872, 957, end)
    rear_decal(f, c, 'd_end_elec', 'sign_electric', 250, 335, 782, 867, end)
    rear_decal(f, c, 'd_panel_tld', 'tld_white', 0, 110, 1170, 1215, ENC_X0 - 6.5)
    rear_decal(f, c, 'd_screen', 'screen', 43, 158, 1271, 1360, ENC_X0 - 9.5)
    for k in (-1, 1):
        y0, y1 = sorted((k * (HY + 60), k * (TRAY_Y - 60)))
        rear_decal(f, c, f'd_tray_tld_{k}', 'tray_stamp', y0, y1, 480, 600, -7)
    hood_x = lambda z: hood_front_x(z) + 2.0
    front_decal(f, c, 'd_hood_tld', 'embossed_tld', -60, 420, 1200, 1335, hood_x)
    front_decal(f, c, 'd_hood_tld_green', 'tld_green', 40, 430, 1065, 1150, hood_x)
    front_decal(f, c, 'd_hood_mand', 'sign_mandatory', 130, 230, 940, 1040, hood_x)
    front_decal(f, c, 'd_hood_speed', 'max_speed', -330, 50, 830, 980, hood_x)
    pivot, top = (2700.0, 1450.0), (2812.0, 1450.0)
    bx0, bz0, bx1, bz1 = 2700.0, 300.0, 2812.0, 1450.0
    t0 = (790.0 - bz0) / (bz1 - bz0)
    t1 = (1430.0 - bz0) / (bz1 - bz0)
    ux, uz = bx1 - bx0, bz1 - bz0
    un = math.hypot(ux, uz)
    nx, nz = uz / un, -ux / un
    lo = (bx0 + ux * t0, bz0 + uz * t0)
    hi = (bx0 + ux * t1, bz0 + uz * t1)
    off = 17.0
    g.decal(f, 'd_tow_chevrons_front', [(lo[0] + nx * off, 52, lo[1] + nz * off), (lo[0] + nx * off, -52, lo[1] + nz * off), (hi[0] + nx * off, -52, hi[1] + nz * off), (hi[0] + nx * off, 52, hi[1] + nz * off)], dmat('chevrons'), c)
    g.decal(f, 'd_tow_chevrons_back', [(lo[0] - nx * off, -52, lo[1] - nz * off), (lo[0] - nx * off, 52, lo[1] - nz * off), (hi[0] - nx * off, 52, hi[1] - nz * off), (hi[0] - nx * off, -52, hi[1] - nz * off)], dmat('chevrons'), c)


def build(with_connection=True):
    g.reset_scene()
    f = g.Frame(X0)
    c = g.collection(NAME)
    m = materials()
    chassis(f, m, c)
    wheels(f, m, c)
    towbar(f, m, c)
    enclosure(f, m, c)
    fittings(f, m, c)
    decals(f, c)
    stowed(f, m, c, -1)
    if with_connection:
        connection(f, m, c)
        g.tube(f, 'spare_coil_right', g.smooth_path(coil(900.0, 1750.0, HY + 45, TRAY_Y - 40, 462.0, 2, 1.1), 2), 22, m['cable'], c, segments=12)
    else:
        stowed(f, m, c, 1)
    return c
