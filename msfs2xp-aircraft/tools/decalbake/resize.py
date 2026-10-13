import os, struct, subprocess, sys
import numpy as np
from PIL import Image
from bake import TEXPIPE, lin, srgb

objdir, work, stage = sys.argv[1:4]
ALBEDO_4K = {'OVHD01_4K_ALBEDO', 'OVHD02_4K_ALBEDO', 'PEDESTAL01_4K_ALBEDO', 'PEDESTAL02_4K_ALBEDO', 'MISC01_4K_ALBEDO', 'FCU_4K_ALBEDO_KFFFFFE', 'MIP_KNOBS_02_4K_ALBEDO'}
LIT_4K = {'OVHD01_4K_ALBEDO_TXTLIT', 'PEDESTAL01_4K_ALBEDO_TXTLIT', 'PEDESTAL02_4K_ALBEDO_TXTLIT', 'MIP_KNOBS_02_4K_ALBEDO_TXTLIT'}


def dds_size(path):
    h, w = struct.unpack('<II', open(path, 'rb').read(20)[12:20])
    return w


def shrink(img, res):
    f = img.shape[1] // res
    if f <= 1:
        return img
    rows = img.shape[0] // f
    x = img.astype(np.float32) / 255.0
    rgb = lin(x[..., :3]).reshape(rows, f, res, f, 3).mean(axis=(1, 3))
    out = [srgb(rgb)]
    if img.shape[2] == 4:
        out.append(x[..., 3:4].reshape(rows, f, res, f, 1).mean(axis=(1, 3)))
    return (np.concatenate(out, axis=2) * 255 + 0.5).astype(np.uint8)


for f in sorted(os.listdir(stage)):
    if not f.endswith('.dds'):
        continue
    stem = f[:-4]
    key = stem.replace('A380_COCKPIT_', '')
    if stem.endswith('_TXTLIT') or stem.endswith('_TXT'):
        png = os.path.join(work, stem + '.png')
        old = os.path.join(objdir, stem[:-4] + '.dds') if stem.endswith('_TXT') else None
        res = 4096 if key in LIT_4K else max(2048, dds_size(old) if old and os.path.exists(old) else 0)
    else:
        png = os.path.join(work, stem + '_baked.png')
        res = 4096 if key in ALBEDO_4K else max(2048, dds_size(os.path.join(objdir, f)))
    res = min(res, 4096)
    img = np.asarray(Image.open(png))
    if img.shape[1] == res:
        print(f'{f}: {res}')
        continue
    small = shrink(img, res)
    tmp = os.path.join(work, stem + f'_{res}.png')
    Image.fromarray(small).save(tmp)
    subprocess.run([TEXPIPE, 'dds', tmp, os.path.join(stage, f)], check=True, capture_output=True)
    print(f'{f}: {img.shape[1]} -> {res}')
