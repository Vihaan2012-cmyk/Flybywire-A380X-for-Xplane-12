import os
import sys

import bpy
import numpy as np

LIVERY_NAME = 'My Livery'

kit = bpy.path.abspath('//')
sys.path.insert(0, os.path.join(kit, 'scripts'))
from dds import write_dxt5_dds

out = os.path.join(kit, 'liveries', LIVERY_NAME, 'objects')
os.makedirs(out, exist_ok=True)
for img in bpy.data.images:
    path = os.path.normpath(bpy.path.abspath(img.filepath))
    if os.sep + 'paint' + os.sep not in path:
        continue
    if img.is_dirty:
        img.save()
    w, h = img.size
    px = np.empty(w * h * 4, np.float32)
    img.pixels.foreach_get(px)
    rgba = np.clip(px.reshape(h, w, 4)[::-1] * 255 + 0.5, 0, 255).astype(np.uint8)
    name = os.path.splitext(os.path.basename(path))[0] + '.dds'
    write_dxt5_dds(rgba, os.path.join(out, name))
    print('wrote', name)
print('livery written to', os.path.dirname(out), '- copy that folder into the aircraft\'s liveries folder')
