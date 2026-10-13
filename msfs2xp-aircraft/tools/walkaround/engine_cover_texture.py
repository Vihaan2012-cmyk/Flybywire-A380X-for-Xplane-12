"""Texture and normal map for the engine inlet covers (engine_cover_blender.py).

    python engine_cover_texture.py <out.png>

Writes <out.png> and <out>_NML.png. Red cover fabric; the front reads, like a
real cover, FRONT TOP with an arrow, A380 / TRENT 900 and REMOVE BEFORE FLIGHT.
Layout (OBJ8 UV, v up): front face u 0..0.5, v 0.5..1 (a disc in that
square); skirt u 0.5..1, v 0..1 (u around the rim, v along the skirt).
"""
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

S = 1024
RED = np.array([196, 24, 28], dtype=np.float32)
WHITE = (236, 233, 226, 255)


def font(px, bold=True):
    names = ("arialbd.ttf", "Arial Bold.ttf", "DejaVuSans-Bold.ttf") if bold else ("arial.ttf", "DejaVuSans.ttf")
    for name in names:
        try:
            return ImageFont.truetype(name, px)
        except OSError:
            continue
    return ImageFont.load_default()


def fabric_height(rng):
    y, x = np.mgrid[0:S, 0:S].astype(np.float32)
    weave = 0.5 + 0.25 * np.sin(x * np.pi / 2.0) * np.sign(np.sin(y * np.pi / 4.0)) + 0.25 * np.sin(y * np.pi / 2.0) * np.sign(np.sin(x * np.pi / 4.0))
    lumps = np.zeros((S, S), np.float32)
    for scale, amp in ((64, 0.6), (16, 0.3), (4, 0.1)):
        g = rng.random((S // scale + 2, S // scale + 2)).astype(np.float32)
        lumps += amp * np.array(Image.fromarray(g).resize((S, S), Image.BICUBIC))
    return weave, lumps


def colour(rng, lumps):
    shade = 0.93 + 0.035 * (lumps - lumps.mean()) / (lumps.std() + 1e-6)
    grain = rng.normal(0, 0.025, (S, S)).astype(np.float32)
    rgb = RED[None, None, :] * (shade + grain)[:, :, None]
    return Image.fromarray(np.clip(rgb, 0, 255).astype(np.uint8)).convert("RGBA")


def artwork(img):
    half = S // 2
    c = half / 2
    layer = Image.new("RGBA", (half, half), (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    top = c - 0.78 * c
    d.polygon([(c, top), (c - 9, top + 14), (c - 3, top + 14), (c - 3, top + 30), (c + 3, top + 30), (c + 3, top + 14), (c + 9, top + 14)], fill=WHITE)
    small = font(11)
    d.text((c, top + 40), "FRONT", font=small, fill=WHITE, anchor="mm")
    d.text((c, top + 53), "TOP", font=small, fill=WHITE, anchor="mm")
    d.text((c, c - 26), "A380", font=font(17), fill=WHITE, anchor="mm")
    d.text((c, c + 4), "TRENT 900", font=font(46), fill=WHITE, anchor="mm")
    d.text((c, c + 58), "REMOVE BEFORE FLIGHT", font=font(22), fill=WHITE, anchor="mm")
    layer = layer.filter(ImageFilter.GaussianBlur(0.6))
    img.alpha_composite(layer, (0, 0))


def normal_map(weave, lumps):
    h = 0.6 * weave + 1.6 * (lumps - lumps.mean())
    gy, gx = np.gradient(h)
    k = 1.2
    n = np.dstack([-gx * k, gy * k, np.ones_like(h)])
    n /= np.linalg.norm(n, axis=2, keepdims=True)
    return Image.fromarray(((n * 0.5 + 0.5) * 255).astype(np.uint8), "RGB")


def main(out):
    rng = np.random.default_rng(380)
    weave, lumps = fabric_height(rng)
    img = colour(rng, lumps)
    artwork(img)
    img.save(out)
    normal_map(weave, lumps).save(out[:-4] + "_NML.png")


if __name__ == "__main__":
    main(sys.argv[1])
