import hashlib
import json
import os
import shutil
import sys

import bpy
import numpy as np

LIVERY_NAME = 'My Livery'

kit = bpy.path.abspath('//')
sys.path.insert(0, os.path.join(kit, 'scripts'))
from dds import write_dxt5_dds

livery = os.path.join(kit, 'liveries', LIVERY_NAME)
out = os.path.join(livery, 'objects')
os.makedirs(out, exist_ok=True)
originals_text = bpy.data.texts.get('kit_originals.json')
originals = json.loads(originals_text.as_string()) if originals_text else {}
skipped = 0
for img in bpy.data.images:
    path = os.path.normpath(bpy.path.abspath(img.filepath))
    parts = path.split(os.sep)
    if 'paint' not in parts and 'ife' not in parts:
        continue
    if img.is_dirty:
        img.save()
    stem = os.path.splitext(os.path.basename(path))[0]
    if 'ife' in parts:
        shutil.copy2(path, os.path.join(livery, os.path.basename(path)))
        print('wrote', os.path.basename(path))
        continue
    w, h = img.size
    px = np.empty(w * h * 4, np.float32)
    img.pixels.foreach_get(px)
    rgba = np.clip(px.reshape(h, w, 4)[::-1] * 255 + 0.5, 0, 255).astype(np.uint8)
    if stem in originals and hashlib.md5(np.ascontiguousarray(rgba).tobytes()).hexdigest() == originals[stem]:
        skipped += 1
        continue
    write_dxt5_dds(rgba, os.path.join(out, stem + '.dds'))
    print('wrote', stem + '.dds')

text = bpy.data.texts.get('cabin.json')
cabin = text.as_string() if text else open(os.path.join(kit, 'cabin.json'), encoding='utf-8').read()
json.loads(cabin)
open(os.path.join(livery, 'cabin.json'), 'w', encoding='utf-8').write(cabin)
print(skipped, 'unchanged cabin and flight deck sheets left out')
print('livery written to', livery, '- copy that folder into the aircraft\'s liveries folder')
