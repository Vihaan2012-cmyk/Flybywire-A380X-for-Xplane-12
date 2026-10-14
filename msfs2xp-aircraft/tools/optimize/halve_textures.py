import os
import shutil
import subprocess
import sys

import numpy as np
from PIL import Image

obj_dir, backup, tool, listing = sys.argv[1:5]
work = os.path.join(os.path.dirname(os.path.abspath(listing)), 'work')
os.makedirs(backup, exist_ok=True)
os.makedirs(work, exist_ok=True)


def shrink(rgba, times):
    img = rgba.astype(np.float32)
    for _ in range(times):
        h, w = img.shape[:2]
        img = img.reshape(h // 2, 2, w // 2, 2, img.shape[2]).mean(axis=(1, 3))
    return np.clip(np.round(img), 0, 255).astype(np.uint8)


for line in open(listing):
    name, times = line.split()
    times = int(times)
    src = os.path.join(obj_dir, name)
    shutil.copy2(src, os.path.join(backup, name))
    if name.lower().endswith('.png'):
        im = Image.open(src)
        mode = im.mode
        arr = np.asarray(im.convert('RGBA') if mode not in ('RGB', 'RGBA', 'L', 'LA') else im)
        if arr.ndim == 2:
            arr = arr[..., None]
        out = shrink(arr, times)
        out = out[..., 0] if out.shape[2] == 1 else out
        Image.fromarray(out, mode if mode in ('RGB', 'RGBA', 'L', 'LA') else 'RGBA').save(src, optimize=True)
        print(f'{name}: {im.size} -> {Image.open(src).size}')
        continue
    raw = os.path.join(work, name + '.rgba')
    info = subprocess.run([tool, 'decode', src, raw], capture_output=True, text=True, check=True).stdout.split()
    w, h = int(info[0]), int(info[1])
    arr = np.fromfile(raw, np.uint8).reshape(h, w, 4)
    out = shrink(arr, times)
    out.tofile(raw)
    nh, nw = out.shape[:2]
    subprocess.run([tool, 'encode', raw, str(nw), str(nh), src], capture_output=True, text=True, check=True)
    os.remove(raw)
    print(f'{name}: {w}x{h} {info[2]} -> {nw}x{nh}')
