import sys

import numpy as np
from PIL import Image


def overlay(render_path, drawing_path, out_path, threshold=150):
    r = Image.open(render_path).convert('RGB')
    d = Image.open(drawing_path).convert('L').resize(r.size, Image.LANCZOS)
    rgb = np.asarray(r).astype(np.float32) * 0.75 + 40
    lines = np.asarray(d) < threshold
    rgb[lines] = (230, 20, 20)
    Image.fromarray(np.clip(rgb, 0, 255).astype(np.uint8)).save(out_path)


def side_by_side(paths, out_path, height=600):
    ims = [Image.open(p).convert('RGB') for p in paths]
    ims = [im.resize((int(im.width * height / im.height), height), Image.LANCZOS) for im in ims]
    sheet = Image.new('RGB', (sum(im.width for im in ims) + 8 * (len(ims) - 1), height), 'white')
    x = 0
    for im in ims:
        sheet.paste(im, (x, 0))
        x += im.width + 8
    sheet.save(out_path)


if __name__ == '__main__':
    if sys.argv[1] == 'overlay':
        overlay(sys.argv[2], sys.argv[3], sys.argv[4])
    else:
        side_by_side(sys.argv[3:], sys.argv[2])
