import os
import shutil
import subprocess

import gselib as g

NAME = 'uld_set'
LIVERY_DIR = os.environ.get('ULD_LIVERY_DIR', 'E:/fbw-gse/out/uld_set/liveries')
VARIANTS = {'a': ('AKE 12345 EK', (200, 30, 40)), 'b': ('AKE 48213 QR', (110, 20, 70)), 'c': ('AKE 90377 LH', (20, 40, 110))}

DOLLY_LEN = 1700.0
DOLLY_HW = 830.0
DECK_Z0 = 330.0
DECK_Z1 = 400.0

AKE_X0 = 150.0
AKE_Y = 2500.0
AKE_DEPTH = 1534.0
AKE_BASE_W = 1562.0
AKE_TOP_W = 2007.0
AKE_H = 1626.0

PMC_X0 = -400.0
PMC_Y = -3300.0
PMC_L = 3175.0
PMC_W = 2438.0
PMC_DOLLY_L = 3400.0
PMC_DOLLY_W = 2600.0
PMC_DECK_Z0 = 360.0
PMC_DECK_Z1 = 430.0

LIVERY_SCRIPT = '''
import sys
from PIL import Image, ImageDraw, ImageFont
out = sys.argv[1]
variants = eval(sys.argv[2])
def font(size):
    for n in ('arialbd.ttf', 'arial.ttf', 'DejaVuSans-Bold.ttf'):
        try:
            return ImageFont.truetype(n, size)
        except OSError:
            pass
    return ImageFont.load_default()
for key, (text, color) in variants.items():
    parts = text.split()
    im = Image.new('RGBA', (1024, 512), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    d.rounded_rectangle((8, 8, 1015, 503), 28, fill=(244, 244, 240, 255), outline=color + (255,), width=14)
    d.rectangle((8, 8, 1015, 120), fill=color + (255,))
    d.text((40, 14), parts[2], font=font(96), fill=(255, 255, 255, 255))
    d.text((1000, 14), 'ULD', font=font(80), fill=(255, 255, 255, 255), anchor='ra')
    d.text((512, 300), parts[0] + ' ' + parts[1], font=font(170), fill=(20, 20, 24, 255), anchor='mm')
    d.text((512, 440), 'TARE 85 KG  MAX GROSS 1588 KG', font=font(52), fill=(20, 20, 24, 255), anchor='mm')
    im.save(out + '/ake_' + key + '.png')
    im = Image.new('RGBA', (1024, 256), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    d.rectangle((0, 0, 1023, 255), fill=color + (255,))
    d.text((512, 128), parts[2] + ' CARGO', font=font(150), fill=(255, 255, 255, 255), anchor='mm')
    im.save(out + '/band_' + key + '.png')
    im = Image.new('RGBA', (1024, 512), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    d.rounded_rectangle((8, 8, 1015, 503), 28, fill=(240, 240, 236, 255), outline=color + (255,), width=14)
    d.text((512, 190), 'PMC ' + parts[1], font=font(200), fill=(20, 20, 24, 255), anchor='mm')
    d.text((512, 370), parts[2], font=font(150), fill=color + (255,), anchor='mm')
    im.save(out + '/pmc_' + key + '.png')
'''


def draw_liveries():
    python = shutil.which('python') or shutil.which('python3')
    os.makedirs(LIVERY_DIR, exist_ok=True)
    if python and not os.path.exists(os.path.join(LIVERY_DIR, 'ake_c.png')):
        subprocess.run([python, '-I', '-c', LIVERY_SCRIPT, LIVERY_DIR, repr(VARIANTS)], check=False)


def liveries(out):
    return {}


def materials():
    return {
        'alu': g.mat('uld_aluminium', (178, 181, 184), metallic=0.8, roughness=0.38),
        'alu_dark': g.mat('uld_aluminium_dark', (120, 123, 127), metallic=0.8, roughness=0.45),
        'skin': g.mat('uld_skin', (190, 193, 196), metallic=0.7, roughness=0.5),
        'fabric': g.mat('uld_fabric_door', (205, 200, 185), roughness=0.85),
        'strap': g.mat('uld_strap', (210, 120, 20), roughness=0.8),
        'steel': g.mat('uld_steel', (150, 152, 154), metallic=0.85, roughness=0.45),
        'rubber': g.mat('uld_rubber', (28, 28, 28), roughness=0.88),
        'blue': g.mat('uld_dolly_blue', (24, 60, 150), roughness=0.4),
        'yellow': g.mat('uld_yellow', (240, 195, 0), roughness=0.4),
        'roller': g.mat('uld_roller', (95, 98, 100), metallic=0.8, roughness=0.4),
        'carton': g.mat('uld_carton', (170, 135, 90), roughness=0.8),
        'carton2': g.mat('uld_carton_pale', (196, 170, 125), roughness=0.8),
        'wrap': g.mat('uld_stretch_wrap', (210, 220, 225), roughness=0.25),
        'net': g.mat('uld_net', (30, 40, 70), roughness=0.9),
        'red': g.mat('uld_red', (185, 18, 20), roughness=0.3),
    }


def wheel(f, m, c, name, x, y, r, w):
    profile = [(r * 0.45, -w / 2), (r * 0.9, -w / 2), (r, -w / 2 + 12), (r, w / 2 - 12), (r * 0.9, w / 2), (r * 0.45, w / 2)]
    tyre = g.lathe(f, f'{name}_tyre', (x, y, r), 'y', profile, m['rubber'], c, segments=24)
    hub = g.cylinder(f, f'{name}_hub', (x, y, r), 'y', r * 0.45, w + 16, m['steel'], c, segments=16, bevel=3)
    return [tyre, hub]


def dolly(f, m, c, x0, yc, length, half_w, z0, z1, pre, tracks, wheel_r, bar_len):
    parts = []
    xe = x0 + length
    parts.append(g.box(f, f'{pre}_deck_plate', x0, xe, yc - half_w, yc + half_w, z0, z1, m['blue'], c, bevel=6))
    for k in (-1, 1):
        edge = yc + k * half_w
        parts.append(g.box(f, f'{pre}_side_rail_{k}', x0, xe, edge - 35, edge + 35, z0 - 70, z1 + 25, m['blue'], c, bevel=5))
    parts.append(g.box(f, f'{pre}_rear_beam', x0 - 25, x0 + 60, yc - half_w, yc + half_w, z0 - 70, z1 + 25, m['blue'], c, bevel=5))
    parts.append(g.box(f, f'{pre}_front_beam', xe - 60, xe + 25, yc - half_w, yc + half_w, z0 - 70, z1 + 25, m['blue'], c, bevel=5))
    for i, (ya, yb, xa, xb) in enumerate(tracks):
        parts.append(g.box(f, f'{pre}_track_{i}', xa, xb, yc + ya, yc + yb, z1, z1 + 28, m['steel'], c, bevel=2))
        n = max(2, int((xb - xa) / 130))
        for j in range(n):
            xr = xa + 40 + j * (xb - xa - 80) / (n - 1)
            parts.append(g.cylinder(f, f'{pre}_roller_{i}_{j}', (xr, yc + (ya + yb) / 2, z1 + 36), 'y', 24, yb - ya - 10, m['roller'], c, segments=12))
    for k in (-1, 1):
        ys = yc + k * (half_w - 120)
        parts.append(g.box(f, f'{pre}_corner_stop_rear_{k}', x0 + 10, x0 + 120, ys - 55, ys + 55, z1, z1 + 110, m['yellow'], c, bevel=4))
        parts.append(g.box(f, f'{pre}_corner_stop_front_{k}', xe - 120, xe - 10, ys - 55, ys + 55, z1, z1 + 110, m['yellow'], c, bevel=4))
        parts.append(g.box(f, f'{pre}_reflector_{k}', x0 + 40, x0 + 140, yc + k * (half_w + 36), yc + k * (half_w + 38), z0 - 30, z0 + 20, m['red'], c))
    for xw in (x0 + 260, xe - 260):
        for k in (-1, 1):
            parts += wheel(f, m, c, f'{pre}_wheel_{int(xw)}_{k}', xw, yc + k * (half_w + 60), wheel_r, 110)
        parts.append(g.cylinder(f, f'{pre}_axle_{int(xw)}', (xw, yc, wheel_r), 'y', 28, 2 * half_w + 80, m['steel'], c, segments=12))
    bar = g.prism_xz(f, f'{pre}_drawbar', [(xe + 25, z0 - 20), (xe + 25, z0 + 30), (xe + bar_len, z0 + 110), (xe + bar_len, z0 + 60)], yc - 24, yc + 24, m['steel'], c, bevel=3)
    parts.append(bar)
    parts.append(g.lathe(f, f'{pre}_tow_eye', (xe + bar_len, yc, z0 + 85), 'z', [(34, -20), (62, -20), (62, 20), (34, 20), (34, -20)], m['steel'], c, segments=20))
    parts.append(g.box(f, f'{pre}_hitch_block', xe + 25, xe + 140, yc - 80, yc + 80, z0 - 40, z0 + 90, m['blue'], c, bevel=5))
    parts.append(g.cylinder(f, f'{pre}_pivot_pin', (xe + 90, yc, z0 + 40), 'z', 22, 160, m['steel'], c, segments=12))
    return parts


def ake_profile():
    hb, ht = AKE_BASE_W / 2, AKE_TOP_W / 2
    return [(-hb, 0.0), (hb, 0.0), (hb, 160.0), (ht, 640.0), (ht, AKE_H - 190.0), (ht - 120.0, AKE_H), (-ht + 120.0, AKE_H), (-ht, AKE_H - 190.0), (-ht, 640.0), (-hb, 160.0)]


def side_quad(k, y, xa, xb, za, zb):
    if k > 0:
        return [(xa, y, za), (xb, y, za), (xb, y, zb), (xa, y, zb)]
    return [(xb, y, za), (xa, y, za), (xa, y, zb), (xb, y, zb)]


def ake(f, m, c, dec):
    parts = []
    x0, yc = AKE_X0, AKE_Y
    x1 = x0 + AKE_DEPTH
    hb, ht = AKE_BASE_W / 2, AKE_TOP_W / 2
    outline = [(yc + y, z) for y, z in ake_profile()]
    parts.append(g.prism_yz(f, 'ake_skin', outline, x0 + 20, x1 - 20, m['skin'], c, bevel=6))
    parts.append(g.box(f, 'ake_base_frame', x0, x1, yc - hb - 18, yc + hb + 18, 0, 70, m['alu'], c, bevel=6))
    parts.append(g.box(f, 'ake_base_deck', x0 + 20, x1 - 20, yc - hb, yc + hb, 70, 90, m['alu_dark'], c, bevel=2))
    parts.append(g.prism_yz(f, 'ake_frame_rear', outline, x0 - 20, x0 + 8, m['alu'], c, bevel=5))
    parts.append(g.prism_yz(f, 'ake_frame_front', outline, x1 - 8, x1 + 20, m['alu'], c, bevel=5))
    for k in (-1, 1):
        for i in range(5):
            xr = x0 + 130 + i * (AKE_DEPTH - 260) / 4
            parts.append(g.box(f, f'ake_rib_{k}_{i}', xr - 22, xr + 22, yc + k * (ht + 2) - 3, yc + k * (ht + 2) + 3, 660, AKE_H - 200, m['alu'], c, bevel=1.5))
        for i in range(3):
            xr = x0 + 180 + i * (AKE_DEPTH - 360) / 2
            parts.append(g.box(f, f'ake_gusset_{k}_{i}', xr - 18, xr + 18, yc + k * (hb + 4) - 3, yc + k * (hb + 4) + 3, 170, 330, m['alu_dark'], c))
        parts.append(g.box(f, f'ake_top_rail_{k}', x0 + 10, x1 - 10, yc + k * (ht - 120) - 22, yc + k * (ht - 120) + 22, AKE_H - 12, AKE_H + 12, m['alu'], c, bevel=3))
        parts.append(g.box(f, f'ake_door_strap_{k}', x1 + 38, x1 + 50, yc + k * 600 - 20, yc + k * 600 + 20, 190, AKE_H - 100, m['strap'], c, bevel=2))
        for zz in (400, 800, 1200):
            parts.append(g.box(f, f'ake_door_buckle_{k}_{zz}', x1 + 48, x1 + 60, yc + k * 600 - 28, yc + k * 600 + 28, zz - 25, zz + 25, m['steel'], c, bevel=3))
        for zz in (300, 700):
            yh = yc + k * 800
            parts.append(g.box(f, f'ake_hinge_tab_{k}_{zz}', x1 + 18, x1 + 34, yh - 22, yh + 22, zz - 40, zz + 40, m['steel'], c, bevel=3))
        parts.append(g.decal(f, f'ake_plate_{k}', side_quad(k, yc + k * (ht + 4), x0 + 150, x0 + 1150, 800, 1230), dec['ake'], c))
    d0, d1 = yc - 760, yc + 760
    parts.append(g.box(f, 'ake_door_frame', x1 + 18, x1 + 28, d0 - 40, d1 + 40, 150, AKE_H - 70, m['alu'], c, bevel=3))
    parts.append(g.box(f, 'ake_door_fabric', x1 + 22, x1 + 40, d0, d1, 190, AKE_H - 100, m['fabric'], c, bevel=12, segments=3))
    for i in range(6):
        zr = 260 + i * 180
        parts.append(g.box(f, f'ake_door_pleat_{i}', x1 + 38, x1 + 46, d0 + 30, d1 - 30, zr, zr + 14, m['fabric'], c, bevel=3))
    parts.append(g.box(f, 'ake_door_bottom_bar', x1 + 22, x1 + 40, d0 - 20, d1 + 20, 150, 200, m['alu'], c, bevel=4))
    xr0 = x0 - 21
    parts.append(g.decal(f, 'ake_band_rear', [(xr0, yc - 760, 1390), (xr0, yc + 760, 1390), (xr0, yc + 760, 1560), (xr0, yc - 760, 1560)], dec['band'], c))
    parts.append(g.decal(f, 'ake_plate_rear', [(xr0, yc - 640, 700), (xr0, yc + 640, 700), (xr0, yc + 640, 1300), (xr0, yc - 640, 1300)], dec['ake'], c))
    return parts


def netted_load(f, m, c, dec):
    parts = []
    x0, yc = PMC_X0 + (PMC_DOLLY_L - PMC_L) / 2, PMC_Y
    x1 = x0 + PMC_L
    pz = PMC_DECK_Z1 + 40
    half = PMC_W / 2
    parts.append(g.box(f, 'pmc_pallet_plate', x0, x1, yc - half, yc + half, pz, pz + 25, m['alu'], c, bevel=3))
    parts.append(g.box(f, 'pmc_pallet_rim_front', x1 - 40, x1, yc - half, yc + half, pz - 15, pz + 40, m['alu_dark'], c, bevel=3))
    parts.append(g.box(f, 'pmc_pallet_rim_rear', x0, x0 + 40, yc - half, yc + half, pz - 15, pz + 40, m['alu_dark'], c, bevel=3))
    for k in (-1, 1):
        ye = yc + k * (half - 20)
        parts.append(g.box(f, f'pmc_edge_rail_{k}', x0, x1, ye - 20, ye + 20, pz - 15, pz + 40, m['alu_dark'], c, bevel=3))
    for i in range(14):
        xr = x0 + 100 + i * (PMC_L - 200) / 13
        parts.append(g.box(f, f'pmc_rail_slot_a_{i}', xr - 14, xr + 14, yc - half + 4, yc - half + 36, pz + 25, pz + 40, m['alu'], c, bevel=2))
        parts.append(g.box(f, f'pmc_rail_slot_b_{i}', xr - 14, xr + 14, yc + half - 36, yc + half - 4, pz + 25, pz + 40, m['alu'], c, bevel=2))
    base_z = pz + 25
    lx0, lx1 = x0 + 60, x1 - 60
    hw = half - 70
    contour = [(yc - hw, base_z), (yc + hw, base_z), (yc + hw, base_z + 1250), (yc + hw - 380, base_z + 1560), (yc - hw + 380, base_z + 1560), (yc - hw, base_z + 1250)]
    parts.append(g.prism_yz(f, 'pmc_load_body', contour, lx0, lx1, m['carton'], c, bevel=40, segments=3))
    for i in range(4):
        zl = base_z + 8 + i * 300
        xs = lx0
        for j, sx in enumerate((1100, 900, 1000, 880)):
            xe = min(xs + sx, lx1)
            if xs >= lx1 - 50:
                break
            parts.append(g.box(f, f'pmc_carton_{i}_{j}', xs + 4, xe - 4, yc - hw - 14, yc + hw + 14, zl, zl + 286, m['carton' if (i + j) % 2 else 'carton2'], c, bevel=10))
            xs = xe
    wz = base_z + 1020
    parts.append(g.box(f, 'pmc_wrap_band', lx0 + 200, lx1 - 200, yc - hw - 22, yc + hw + 22, wz, wz + 230, m['wrap'], c, bevel=6))
    nx = 9
    for i in range(nx + 1):
        xr = lx0 + i * (lx1 - lx0) / nx
        pts = [(xr, yc - hw - 28, base_z + 20), (xr, yc - hw - 30, base_z + 1250), (xr, yc - hw + 380, base_z + 1600), (xr, yc + hw - 380, base_z + 1600), (xr, yc + hw + 30, base_z + 1250), (xr, yc + hw + 28, base_z + 20)]
        parts.append(g.tube(f, f'pmc_net_vertical_{i}', g.smooth_path(pts, 2), 6, m['net'], c, segments=6))
        parts.append(g.box(f, f'pmc_net_top_cross_{i}', xr - 6, xr + 6, yc - hw + 360, yc + hw - 360, base_z + 1584, base_z + 1604, m['net'], c))
    for i, zr in enumerate((base_z + 250, base_z + 560, base_z + 870, base_z + 1180)):
        for k in (-1, 1):
            ys = yc + k * (hw + 30)
            parts.append(g.box(f, f'pmc_net_horizontal_{k}_{i}', lx0 - 2, lx1 + 2, ys - 5, ys + 5, zr - 6, zr + 6, m['net'], c))
        parts.append(g.box(f, f'pmc_net_end_h_{i}', lx1 + 8, lx1 + 20, yc - hw, yc + hw, zr - 6, zr + 6, m['net'], c))
    for i in range(7):
        xr = x0 + 180 + i * (PMC_L - 360) / 6
        for k in (-1, 1):
            ys = yc + k * (half + 6)
            parts.append(g.box(f, f'pmc_net_hook_{k}_{i}', xr - 25, xr + 25, ys - 6, ys + 6, pz - 10, pz + 30, m['steel'], c, bevel=2))
    parts.append(g.box(f, 'pmc_net_front_panel', lx1 + 4, lx1 + 14, yc - hw, yc + hw, base_z + 20, base_z + 1250, m['net'], c))
    xf = x1 + 1
    parts.append(g.decal(f, 'pmc_plate_front', [(xf, yc - 500, pz - 10), (xf, yc + 500, pz - 10), (xf, yc + 500, pz + 35), (xf, yc - 500, pz + 35)], dec['band'], c))
    return parts


def decal_materials(variant):
    def mk(prefix):
        return g.image_mat(f'decal_{prefix}_{variant}', os.path.join(LIVERY_DIR, f'{prefix}_{variant}.png'), roughness=0.5)
    return {'ake': mk('ake'), 'band': mk('band'), 'pmc': mk('pmc')}


def build(with_connection=True):
    g.reset_scene()
    draw_liveries()
    f = g.Frame(0.0)
    c = g.collection(NAME)
    m = materials()
    dec = decal_materials('a')
    ld3_tracks = [(-740, -640, 120, 1600), (-60, 60, 120, 1600), (640, 740, 120, 1600)]
    dolly(f, m, c, 0.0, 0.0, DOLLY_LEN, DOLLY_HW, DECK_Z0, DECK_Z1, 'ld3dolly', ld3_tracks, 125, 780.0)
    ake(f, m, c, dec)
    pmc_tracks = [(-1000, -880, 150, PMC_DOLLY_L - 150), (-60, 60, 150, PMC_DOLLY_L - 150), (880, 1000, 150, PMC_DOLLY_L - 150)]
    dolly(f, m, c, PMC_X0, PMC_Y, PMC_DOLLY_L, PMC_DOLLY_W / 2, PMC_DECK_Z0, PMC_DECK_Z1, 'pmcdolly', pmc_tracks, 150, 900.0)
    netted_load(f, m, c, dec)
    for o in c.objects:
        o['gse_part'] = o.name.split('_')[0]
    return c


VIEWS = {
    'overview': dict(res=(800, 560), samples=16, lens=26, loc=(-1.0, -9.0, 6.5), target=(-0.4, 1.2, 0.4)),
    'ake_front': dict(res=(700, 600), samples=16, lens=35, loc=(4.6, 4.2, 2.2), target=(2.5, 0.9, 0.8)),
    'pmc_three_quarter': dict(res=(800, 560), samples=16, lens=30, loc=(-8.2, -3.4, 3.8), target=(-3.3, 1.2, 0.8)),
    'dolly_close': dict(res=(700, 500), samples=16, lens=35, loc=(2.8, 3.6, 1.5), target=(0.0, 0.85, 0.4)),
}
