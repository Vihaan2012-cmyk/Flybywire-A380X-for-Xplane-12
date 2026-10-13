import sys

import numpy as np
from PIL import Image
from scipy import ndimage as ndi

PROBE = 11
DEPTH = 0.018
MAX_SPOT = 28
FLAT = 0.006


def small_parts(mask, max_size):
    lab, n = ndi.label(mask)
    if n == 0:
        return mask
    keep = np.zeros(n + 1, bool)
    for i, sl in enumerate(ndi.find_objects(lab), start=1):
        if sl is not None and max(sl[0].stop - sl[0].start, sl[1].stop - sl[1].start) <= max_size:
            keep[i] = True
    return keep[lab]


def clean_rgb(img, scale=1.0):
    img = img.astype(np.float32)
    probe = max(3, int(round(PROBE * scale)) | 1)
    lum = img[..., :3].mean(axis=2)
    closed = ndi.grey_closing(lum, size=(probe, probe))
    opened = ndi.grey_opening(lum, size=(probe, probe))
    def flat(x):
        g = ndi.gaussian_filter(x, 1.5 * scale)
        return ndi.maximum_filter(np.hypot(ndi.sobel(g, 0), ndi.sobel(g, 1)) / 8.0, size=probe) < FLAT
    spots = small_parts(((closed - lum) > DEPTH) & flat(closed), MAX_SPOT * scale) | small_parts(((lum - opened) > DEPTH) & flat(opened), MAX_SPOT * scale)
    spots = ndi.binary_dilation(spots, iterations=max(1, int(round(2 * scale))))
    known = ~spots
    smooth = np.empty_like(img)
    w = ndi.gaussian_filter(known.astype(np.float32), 3 * scale)
    for c in range(img.shape[2]):
        smooth[..., c] = ndi.gaussian_filter(img[..., c] * known, 3 * scale) / np.maximum(w, 1e-4)
    far = w < 0.05
    if far.any():
        idx = ndi.distance_transform_edt(~known, return_distances=False, return_indices=True)
        nearest = img[idx[0], idx[1]]
        smooth[far] = nearest[far]
    soft = ndi.gaussian_filter(spots.astype(np.float32), 1.0 * scale)[..., None]
    return img * (1 - soft) + smooth * soft, spots


if __name__ == '__main__':
    src, out = sys.argv[1], sys.argv[2]
    im = np.asarray(Image.open(src).convert('RGBA')).astype(np.float32) / 255.0
    scale = im.shape[1] / 4096
    rgb, spots = clean_rgb(im[..., :3], scale)
    print(f'{spots.mean() * 100:.1f}% of texels cleaned')
    res = np.concatenate([rgb, im[..., 3:4]], axis=2)
    Image.fromarray((np.clip(res, 0, 1) * 255 + 0.5).astype(np.uint8), 'RGBA').save(out)
