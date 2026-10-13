import struct, sys
import numpy as np
from PIL import Image


def decode_bc1(data, w, h):
    bw, bh = max(1, (w + 3) // 4), max(1, (h + 3) // 4)
    blocks = np.frombuffer(data, dtype='<u2', count=bw * bh * 4).reshape(bh, bw, 4)
    c0, c1 = blocks[..., 0].astype(np.int32), blocks[..., 1].astype(np.int32)
    idx = blocks[..., 2].astype(np.uint32) | (blocks[..., 3].astype(np.uint32) << 16)

    def rgb(c):
        return np.stack([((c >> 11) & 31) * 255 // 31, ((c >> 5) & 63) * 255 // 63, (c & 31) * 255 // 31], -1)

    p0, p1 = rgb(c0), rgb(c1)
    four = (c0 > c1)[..., None]
    p2 = np.where(four, (2 * p0 + p1) // 3, (p0 + p1) // 2)
    p3 = np.where(four, (p0 + 2 * p1) // 3, 0)
    pal = np.stack([p0, p1, p2, p3], 2)
    out = np.zeros((bh * 4, bw * 4, 3), np.uint8)
    for py in range(4):
        for px in range(4):
            sel = (idx >> (2 * (py * 4 + px))) & 3
            out[py::4, px::4] = np.take_along_axis(pal, sel[:, :, None, None].repeat(3, 3), 2)[:, :, 0]
    return out[:h, :w]


def mips(path):
    b = open(path, 'rb').read()
    h, w = struct.unpack('<II', b[12:20])
    count = max(1, struct.unpack('<I', b[28:32])[0])
    fourcc = b[84:88]
    assert fourcc == b'DXT1', fourcc
    off = 128
    for level in range(count):
        mw, mh = max(1, w >> level), max(1, h >> level)
        size = max(1, (mw + 3) // 4) * max(1, (mh + 3) // 4) * 8
        yield level, mw, mh, decode_bc1(b[off:off + size], mw, mh)
        off += size


if __name__ == '__main__':
    path, out_prefix = sys.argv[1], sys.argv[2]
    levels = [int(x) for x in sys.argv[3].split(',')]
    for level, mw, mh, img in mips(path):
        if level in levels:
            Image.fromarray(img).save(f'{out_prefix}_mip{level}_{mw}x{mh}.png')
            print(level, mw, mh)


def decode_bc3(data, w, h):
    bw, bh = max(1, (w + 3) // 4), max(1, (h + 3) // 4)
    raw = np.frombuffer(data, dtype=np.uint8, count=bw * bh * 16).reshape(bh, bw, 16)
    a0 = raw[..., 0].astype(np.int32); a1 = raw[..., 1].astype(np.int32)
    abits = np.zeros((bh, bw), np.uint64)
    for k in range(6):
        abits |= raw[..., 2 + k].astype(np.uint64) << np.uint64(8 * k)
    pal = np.zeros((bh, bw, 8), np.int32)
    pal[..., 0], pal[..., 1] = a0, a1
    big = a0 > a1
    for i in range(1, 7):
        pal[..., 1 + i] = np.where(big, ((7 - i) * a0 + i * a1) // 7, 0)
    for i in range(1, 5):
        pal[..., 1 + i] = np.where(big, pal[..., 1 + i], ((5 - i) * a0 + i * a1) // 5)
    pal[..., 6] = np.where(big, pal[..., 6], 0)
    pal[..., 7] = np.where(big, pal[..., 7], 255)
    alpha = np.zeros((bh * 4, bw * 4), np.uint8)
    for p in range(16):
        sel = ((abits >> np.uint64(3 * p)) & np.uint64(7)).astype(np.int64)
        alpha[p // 4::4, p % 4::4] = np.take_along_axis(pal, sel[..., None], 2)[..., 0]
    color = raw[..., 8:16].copy().reshape(-1)
    rgb = decode_bc1_four(color.tobytes(), w, h)
    return np.dstack([rgb, alpha[:h, :w]])


def decode_bc1_four(data, w, h):
    bw, bh = max(1, (w + 3) // 4), max(1, (h + 3) // 4)
    blocks = np.frombuffer(data, dtype='<u2', count=bw * bh * 4).reshape(bh, bw, 4)
    c0, c1 = blocks[..., 0].astype(np.int32), blocks[..., 1].astype(np.int32)
    idx = blocks[..., 2].astype(np.uint32) | (blocks[..., 3].astype(np.uint32) << 16)
    def rgb(c):
        return np.stack([((c >> 11) & 31) * 255 // 31, ((c >> 5) & 63) * 255 // 63, (c & 31) * 255 // 31], -1)
    p0, p1 = rgb(c0), rgb(c1)
    pal = np.stack([p0, p1, (2 * p0 + p1) // 3, (p0 + 2 * p1) // 3], 2)
    out = np.zeros((bh * 4, bw * 4, 3), np.uint8)
    for py in range(4):
        for px in range(4):
            sel = (idx >> (2 * (py * 4 + px))) & 3
            out[py::4, px::4] = np.take_along_axis(pal, sel[:, :, None, None].repeat(3, 3), 2)[:, :, 0]
    return out[:h, :w]


def mips_bc3(path):
    b = open(path, 'rb').read()
    h, w = struct.unpack('<II', b[12:20])
    count = max(1, struct.unpack('<I', b[28:32])[0])
    off = 128
    for level in range(count):
        mw, mh = max(1, w >> level), max(1, h >> level)
        size = max(1, (mw + 3) // 4) * max(1, (mh + 3) // 4) * 16
        yield level, mw, mh, decode_bc3(b[off:off + size], mw, mh)
        off += size
