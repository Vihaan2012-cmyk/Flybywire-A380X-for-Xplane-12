import multiprocessing as mp
import os
import re
import struct
import subprocess
import sys

import numpy as np
from PIL import Image

from bake import SKIP_TEXTURE_PREFIX, TEXPIPE, lin, obj_path, rebuild_base, srgb
from clean import clean_rgb
from unshare import header


def width(path):
    return struct.unpack('<II', open(path, 'rb').read(20)[12:20])[1]


def shrink(img, res):
    f = img.shape[1] // res
    if f <= 1:
        return img
    rows = img.shape[0] // f
    rgb = srgb(lin(img[..., :3]).reshape(rows, f, res, f, 3).mean(axis=(1, 3)))
    a = img[..., 3:4].reshape(rows, f, res, f, 1).mean(axis=(1, 3))
    return np.concatenate([rgb, a], axis=2)


def albedo(args):
    objdir, work, stage, tex = args
    try:
        base, best = rebuild_base(objdir, work, tex)
    except subprocess.CalledProcessError:
        return tex, 'no MSFS source found, left worn'
    if base is None:
        return tex, 'no recipe matched, left worn'
    rgb, spots = clean_rgb(base[..., :3], base.shape[1] / 4096)
    out = shrink(np.concatenate([rgb, base[..., 3:4]], axis=2), width(os.path.join(objdir, tex)))
    png = os.path.join(work, os.path.splitext(tex)[0] + '_clean.png')
    Image.fromarray((np.clip(out, 0, 1) * 255 + 0.5).astype(np.uint8), 'RGBA').save(png)
    subprocess.run([TEXPIPE, 'dds', png, os.path.join(stage, tex)], check=True, capture_output=True)
    return tex, f'{spots.mean() * 100:.1f}% cleaned at {out.shape[1]}'


def material(args):
    objdir, stage, f = args
    im = np.asarray(Image.open(obj_path(objdir, f)))
    x = im.astype(np.float32) / 255.0
    if x.ndim == 2:
        x = x[..., None]
    rgb, spots = clean_rgb(x, x.shape[1] / 4096)
    out = (np.clip(rgb, 0, 1) * 255 + 0.5).astype(np.uint8)
    Image.fromarray(out[..., 0] if im.ndim == 2 else out).save(os.path.join(stage, f))
    return f, f'{spots.mean() * 100:.1f}% cleaned'


def main(objdir, acf_path, work, stage, skip_baked):
    os.makedirs(stage, exist_ok=True)
    os.makedirs(work, exist_ok=True)
    baked = set(open(skip_baked).read().split()) if skip_baked != '-' else set()
    acf = open(acf_path, encoding='utf-8', errors='replace').read()
    names = [os.path.splitext(f)[0] for f in re.findall(r'_v10_att_file_stl (\S+)', acf)]
    texs, mats = set(), set()
    for n in names:
        if not n.startswith('a380_cockpit_') or not n[13:].isdigit():
            continue
        num = int(n[13:])
        if num >= 97 or num in (4, 8, 9, 10) or 20 <= num <= 25:
            continue
        h = header(obj_path(objdir, n + '.obj'))
        t = h.get('TEXTURE', '')
        if not t or t.startswith(SKIP_TEXTURE_PREFIX) or '_DECAL' in t.upper() or not t.lower().endswith('.dds'):
            continue
        if t not in baked:
            texs.add(t)
        if h.get('material_gloss', '').lower().endswith('.png'):
            mats.add(h['material_gloss'])
    print(f'{len(texs)} albedos to clean (+{len(baked)} cleaned inside the bake), {len(mats)} material maps', flush=True)
    with mp.Pool(12) as pool:
        for t, r in pool.imap_unordered(albedo, [(objdir, work, stage, t) for t in sorted(texs)]):
            print(f'  {t}: {r}', flush=True)
        for f, r in pool.imap_unordered(material, [(objdir, stage, f) for f in sorted(mats)]):
            print(f'  {f}: {r}', flush=True)


if __name__ == '__main__':
    main(*sys.argv[1:6])
