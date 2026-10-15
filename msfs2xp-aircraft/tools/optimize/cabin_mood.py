import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from objtools import Obj, decode, png, sample, wrap

MOOD = {
    'A380X_MAIN_DECK_PEC_SIDEWALLS_EMIS.dds',
    'A380X_MAIN_DECK_YC_SIDEWALLS_EMIS.dds',
    'A380X_AFT_STAIRS_EMIS.dds',
    'A380X_SUITE_CEILING_EMIS.dds',
    'A380_SUITES_STAIRS_EMIS_K9F9F9F.dds',
}
STARS = 'A380X_SUITE_CEILING_EMIS.dds'
PALETTE = [
    (1.0, 0.95, 0.88),
    (1.0, 0.72, 0.42),
    (1.0, 0.5, 0.15),
    (1.0, 0.42, 0.62),
    (0.62, 0.36, 1.0),
    (0.32, 0.5, 1.0),
    (0.25, 0.85, 0.8),
    (0.12, 0.16, 0.45),
]
NIGHT = 7
SIZE = 1024
CABIN_SHOWN = ['ANIM_begin', 'ANIM_show 0.5 1.5 fbw/options/cabin_visible']
DECKS = [(3.2, -31.0, 19.0), (6.0, -28.0, 18.0)]
STEP_M = 6.0
CHANNELS = [((1.0, 0.0, 0.0), 'fbw/cabin/spill_r'), ((0.0, 1.0, 0.0), 'fbw/cabin/spill_g'), ((0.0, 0.0, 1.0), 'fbw/cabin/spill_b')]


def neutral_mask(rgb):
    lum = rgb.max(axis=2)
    sat = (lum - rgb.min(axis=2)) / np.maximum(lum, 1e-3)
    m = (lum > 0.05) & (sat < 0.35)
    grown = m.copy()
    for dy in (-2, -1, 0, 1, 2):
        for dx in (-2, -1, 0, 1, 2):
            grown |= np.roll(np.roll(m, dy, axis=0), dx, axis=1)
    return m, grown


def shrink(a, size):
    h, w = a.shape[:2]
    f = max(1, h // size)
    if f == 1:
        return a
    return a[: h // f * f, : w // f * f].reshape(h // f, f, w // f, f, -1).mean(axis=(1, 3))


def variants(rgb, mask, stars):
    lum = rgb.max(axis=2, keepdims=True)
    small_rgb = shrink(rgb, SIZE)
    small_lum = shrink(lum, SIZE)
    small_mask = shrink(mask[..., None].astype(np.float32), SIZE) > 0.25
    rng = np.random.default_rng(380)
    for k, colour in enumerate(PALETTE):
        out = np.where(small_mask, small_lum * np.array(colour), small_rgb)
        if stars and k == NIGHT:
            ys, xs = np.nonzero(small_mask[..., 0])
            pick = rng.random(len(ys)) < 0.004
            for y, x in zip(ys[pick], xs[pick]):
                b = rng.uniform(0.6, 1.0)
                out[max(0, y - 1):y + 1, max(0, x - 1):x + 1] = (b, b, b * 0.95)
        yield k, np.clip(out * 255 + 0.5, 0, 255)


def make_lights(objects_dir):
    out = ['I', '800', 'OBJ', '', 'POINT_COUNTS 0 0 0 0', '', 'ANIM_begin', 'ANIM_show 0.5 1.5 fbw/options/cabin_visible', 'ANIM_hide 0.5 1.5 fbw/anim/eye_in_flight_deck', 'ANIM_hide 0.5 1.5 sim/graphics/view/view_is_external']
    n = 0
    for y, z0, z1 in DECKS:
        z = z0
        while z <= z1:
            for (r, g, b), dref in CHANNELS:
                out.append(f'LIGHT_SPILL_CUSTOM 0.000 {y:.3f} {z:.3f} {r:.2f} {g:.2f} {b:.2f} 0.80 4.50 0.000 -1.000 0.000 0.20 {dref}')
                n += 1
            z += STEP_M
    out += ['ANIM_end', '']
    open(os.path.join(objects_dir, 'a380_cabin_lights.obj'), 'w', encoding='latin-1', newline='\n').write('\n'.join(out))
    return n


full_dir, objects_dir, ddstool = sys.argv[1:4]
for name in sorted(os.listdir(full_dir)):
    if not name.endswith('.obj'):
        continue
    path = os.path.join(full_dir, name)
    with open(path, encoding='latin-1') as f:
        head = f.read(4000)
    if not any(t in head for t in MOOD):
        continue
    obj = Obj(path)
    lit = obj.texture('TEXTURE_LIT')
    if lit not in MOOD:
        continue
    rgb = decode(ddstool, os.path.join(objects_dir, lit))[..., :3].astype(np.float32) / 255
    exact, grown = neutral_mask(rgb)
    uv = obj.vt[:, 6:8]
    keep_lit = []
    for t in obj.triangles():
        c = uv[t]
        probes = [c[:, 0], c[:, 1], c[:, 2], c.mean(axis=1), (c[:, 0] + c[:, 1]) / 2, (c[:, 1] + c[:, 2]) / 2, (c[:, 0] + c[:, 2]) / 2]
        keep_lit.append(np.any([sample(grown, p) for p in probes], axis=0))
    moved = int(sum(m.sum() for m in keep_lit))
    if not moved:
        print(name, 'no glowing faces left (already split)')
        continue
    stem = os.path.splitext(lit)[0]
    for k, img in variants(rgb, exact, lit == STARS):
        tex = f'{stem}_MOOD_{k}.png'
        png(os.path.join(objects_dir, tex), img)
        variant = os.path.join(objects_dir, f'a380_cabin_mood_{os.path.splitext(name)[0]}_{k}.obj')
        obj.write(variant, keep_lit)
        lines = open(variant, encoding='latin-1').read().split('\n')
        fixed = []
        for l in lines:
            p = l.split()
            if p[:1] == ['TEXTURE_LIT']:
                l = f'TEXTURE_LIT {tex}'
            elif p[:1] == ['ATTR_light_level'] and len(p) >= 4:
                p[3] = 'fbw/cabin/glow_level'
                l = ' '.join(p)
            fixed.append(l)
        open(variant, 'w', encoding='latin-1', newline='\n').write('\n'.join(fixed))
        wrap(variant, ['ANIM_begin', f'ANIM_show {k - 0.5} {k + 0.5} fbw/cabin/variant'])
        wrap(variant, CABIN_SHOWN)
    left = obj.write(path, [~m for m in keep_lit])
    print(f'{name}: {moved} glowing triangles into 8 colour variants, {left} left in the base object')
print('cabin lights:', make_lights(objects_dir), 'spill lights')
