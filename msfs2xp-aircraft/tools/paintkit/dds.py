"""
A small, dependency-free (numpy only) DXT5/BC3 DDS encoder.

Written for the paint kit because the example livery needs to ship textures
in the same format the aircraft's own textures use (BC3, full mip chain,
legacy DX9 header) rather than an uncompressed placeholder, and no existing
Python package in this environment writes compressed DDS.

Not a general-purpose encoder: bounding-box endpoints (not true principal-
axis / cluster-fit), no perceptual weighting. Good enough for a flat-shaded
demo livery; a real production texture would want a proper tool (NVTT,
texconv, Compressonator).

Usage:
    from dds import write_dxt5_dds
    write_dxt5_dds(rgba_uint8_hxwx4, "out.dds")           # full mip chain
    write_dxt5_dds(rgba_uint8_hxwx4, "out.dds", mips=1)   # single level
"""
from __future__ import annotations

import struct
from pathlib import Path

import numpy as np


def _rgb888_to_565(r, g, b):
    r5 = (r.astype(np.uint32) * 31 + 127) // 255
    g6 = (g.astype(np.uint32) * 63 + 127) // 255
    b5 = (b.astype(np.uint32) * 31 + 127) // 255
    return (r5 << 11) | (g6 << 5) | b5


def _565_to_rgb888(v):
    v = v.astype(np.uint32)
    r5 = (v >> 11) & 0x1F
    g6 = (v >> 5) & 0x3F
    b5 = v & 0x1F
    r = (r5 * 255 + 15) // 31
    g = (g6 * 255 + 31) // 63
    b = (b5 * 255 + 15) // 31
    return r.astype(np.float32), g.astype(np.float32), b.astype(np.float32)


def _pad_to_multiple_of_4(rgba: np.ndarray) -> np.ndarray:
    h, w = rgba.shape[:2]
    ph = (-h) % 4
    pw = (-w) % 4
    if ph == 0 and pw == 0:
        return rgba
    return np.pad(rgba, ((0, ph), (0, pw), (0, 0)), mode="edge")


def _to_blocks(rgba: np.ndarray) -> np.ndarray:
    """(H,W,4) -> (Nblocks, 16, 4), H and W already multiples of 4."""
    h, w = rgba.shape[:2]
    bh, bw = h // 4, w // 4
    # (bh,4,bw,4,4) -> (bh,bw,4,4,4) -> (bh*bw,16,4)
    blocks = rgba.reshape(bh, 4, bw, 4, 4).transpose(0, 2, 1, 3, 4)
    return blocks.reshape(bh * bw, 16, 4), bh, bw


def _pack_indices(idx: np.ndarray, bits: int) -> np.ndarray:
    """idx: (Nblocks, 16) small ints -> (Nblocks,) uint64 packed LSB-first,
    16 entries of `bits` bits each (48 bits for 3-bit alpha, 32 for 2-bit
    colour -- both fit in uint64)."""
    acc = np.zeros(idx.shape[0], dtype=np.uint64)
    for i in range(16):
        acc |= idx[:, i].astype(np.uint64) << np.uint64(bits * i)
    return acc


def _encode_alpha_block(block_a: np.ndarray) -> bytes:
    """block_a: (Nblocks, 16) uint8 -> alpha section bytes, (Nblocks, 8)."""
    a0 = block_a.max(axis=1).astype(np.int32)
    a1 = block_a.min(axis=1).astype(np.int32)
    eq = a0 == a1
    # Force a strict a0 > a1 so the 8-level interpolated mode is always
    # selected (the spec's alternate mode, a0<=a1, injects hard 0/255
    # values we do not want here). A flat-0 block becomes a0=1,a1=0 (index
    # 1 matches every texel exactly); any other flat block drops a1 by 1.
    a1 = np.where(eq, np.where(a0 > 0, a0 - 1, 0), a1)
    a0 = np.where(eq & (a0 == 0), 1, a0)

    # 8-value palette per BC3 spec (a0 > a1 case).
    k = np.arange(8, dtype=np.float32)
    # index 0 -> a0, 1 -> a1, 2..7 -> interpolated
    weight_a0 = np.array([7, 0, 6, 5, 4, 3, 2, 1], dtype=np.float32)
    weight_a1 = 7 - weight_a0
    palette = (a0[:, None].astype(np.float32) * weight_a0[None, :] + a1[:, None].astype(np.float32) * weight_a1[None, :]) / 7.0

    diff = np.abs(block_a[:, :, None].astype(np.float32) - palette[:, None, :])
    idx = diff.argmin(axis=2).astype(np.uint8)  # (Nblocks, 16), values 0..7

    packed = _pack_indices(idx, 3)  # 48 bits used of uint64
    nb = block_a.shape[0]
    out = np.empty((nb, 8), dtype=np.uint8)
    out[:, 0] = a0.astype(np.uint8)
    out[:, 1] = a1.astype(np.uint8)
    packed_bytes = packed.astype(">u8").view(np.uint8).reshape(nb, 8)[:, ::-1]  # little-endian bytes of the 64-bit value
    out[:, 2:8] = packed_bytes[:, 0:6]
    return out.tobytes()


def _encode_color_block(block_rgb: np.ndarray) -> bytes:
    """block_rgb: (Nblocks, 16, 3) uint8 -> colour section bytes, (Nblocks, 8)."""
    cmax = block_rgb.max(axis=1).astype(np.float32)  # (Nblocks,3)
    cmin = block_rgb.min(axis=1).astype(np.float32)

    c0_raw = _rgb888_to_565(cmax[:, 0], cmax[:, 1], cmax[:, 2])
    c1_raw = _rgb888_to_565(cmin[:, 0], cmin[:, 1], cmin[:, 2])

    # Need c0_raw > c1_raw (as uint16) for the 4-distinct-colour mode. If a
    # flat block quantises to equal 565 values, or max/min happened to
    # invert once quantised, force a strict order and keep endpoints
    # consistent with which corner is "0" vs "1".
    # Whichever of the two quantised 565 values is larger becomes c0 (raw
    # compare, as a real decoder does); this can invert which corner was
    # originally "max"/"min" once quantised, but that is fine -- col0/col1
    # below are re-derived from whichever raw value ends up in each slot.
    c0_raw2 = np.maximum(c0_raw, c1_raw).astype(np.uint32)
    c1_raw2 = np.minimum(c0_raw, c1_raw).astype(np.uint32)
    still_eq = c0_raw2 <= c1_raw2
    c0_raw2 = np.where(still_eq, np.minimum(c1_raw2.astype(np.int32) + 1, 65535), c0_raw2).astype(np.uint32)

    r0, g0, b0 = _565_to_rgb888(c0_raw2)
    r1, g1, b1 = _565_to_rgb888(c1_raw2)
    col0 = np.stack([r0, g0, b0], axis=1)
    col1 = np.stack([r1, g1, b1], axis=1)
    col2 = (2 * col0 + col1) / 3.0
    col3 = (col0 + 2 * col1) / 3.0
    palette = np.stack([col0, col1, col2, col3], axis=1)  # (Nblocks,4,3)

    diff = block_rgb[:, :, None, :].astype(np.float32) - palette[:, None, :, :]
    dist = (diff * diff).sum(axis=3)  # (Nblocks,16,4)
    idx = dist.argmin(axis=2).astype(np.uint8)

    packed = _pack_indices(idx, 2)  # 32 bits used
    nb = block_rgb.shape[0]
    out = np.empty((nb, 8), dtype=np.uint8)
    out[:, 0:2] = c0_raw2.astype("<u2").view(np.uint8).reshape(nb, 2)
    out[:, 2:4] = c1_raw2.astype("<u2").view(np.uint8).reshape(nb, 2)
    packed_bytes = packed.astype(">u8").view(np.uint8).reshape(nb, 8)[:, ::-1]
    out[:, 4:8] = packed_bytes[:, 0:4]
    return out.tobytes()


def encode_dxt5(rgba: np.ndarray) -> bytes:
    """rgba: (H,W,4) uint8, H and W multiples of 4. Returns raw BC3 block
    data (alpha block (8B) then colour block (8B), per 4x4 tile, in
    left-to-right/top-to-bottom tile order -- exactly DDS's own layout)."""
    h, w = rgba.shape[:2]
    assert h % 4 == 0 and w % 4 == 0, "pad to a multiple of 4 first"
    blocks, bh, bw = _to_blocks(rgba)
    alpha_bytes = _encode_alpha_block(blocks[:, :, 3])
    color_bytes = _encode_color_block(blocks[:, :, 0:3])
    nb = blocks.shape[0]
    alpha_arr = np.frombuffer(alpha_bytes, dtype=np.uint8).reshape(nb, 8)
    color_arr = np.frombuffer(color_bytes, dtype=np.uint8).reshape(nb, 8)
    combined = np.concatenate([alpha_arr, color_arr], axis=1)  # (nb,16)
    return combined.tobytes()


def _box_filter_half(rgba: np.ndarray) -> np.ndarray:
    h, w = rgba.shape[:2]
    h2, w2 = max(1, h // 2), max(1, w // 2)
    src = rgba
    if h % 2 or w % 2:
        src = np.pad(src, ((0, h % 2), (0, w % 2), (0, 0)), mode="edge")
    f = src.astype(np.float32)
    out = (f[0::2, 0::2] + f[1::2, 0::2] + f[0::2, 1::2] + f[1::2, 1::2]) / 4.0
    return np.clip(out, 0, 255).astype(np.uint8)[:h2, :w2]


def build_mip_chain(rgba: np.ndarray, max_levels: int | None = None) -> list[np.ndarray]:
    levels = [rgba]
    cur = rgba
    while cur.shape[0] > 1 or cur.shape[1] > 1:
        cur = _box_filter_half(cur)
        levels.append(cur)
        if max_levels and len(levels) >= max_levels:
            break
    return levels


def _dds_header(w: int, h: int, mip_count: int, linear_size: int) -> bytes:
    def u32(v):
        return int(v).to_bytes(4, "little")

    DDSD_CAPS = 0x1
    DDSD_HEIGHT = 0x2
    DDSD_WIDTH = 0x4
    DDSD_PIXELFORMAT = 0x1000
    DDSD_MIPMAPCOUNT = 0x20000
    DDSD_LINEARSIZE = 0x80000
    flags = DDSD_CAPS | DDSD_HEIGHT | DDSD_WIDTH | DDSD_PIXELFORMAT | DDSD_MIPMAPCOUNT | DDSD_LINEARSIZE

    DDPF_FOURCC = 0x4
    pf_flags = DDPF_FOURCC

    DDSCAPS_COMPLEX = 0x8
    DDSCAPS_TEXTURE = 0x1000
    DDSCAPS_MIPMAP = 0x400000
    caps = DDSCAPS_COMPLEX | DDSCAPS_TEXTURE | DDSCAPS_MIPMAP

    header = b"DDS "
    header += u32(124)  # header size
    header += u32(flags)
    header += u32(h)
    header += u32(w)
    header += u32(linear_size)
    header += u32(0)  # depth
    header += u32(mip_count)
    header += u32(0) * 11  # reserved1
    # pixel format (32 bytes)
    header += u32(32)
    header += u32(pf_flags)
    header += b"DXT5"
    header += u32(0) * 5  # RGBBitCount + 4 masks, unused for FOURCC
    header += u32(caps)
    header += u32(0) * 3  # caps2/3/4
    header += u32(0)  # reserved2
    assert len(header) == 128, len(header)
    return header


def write_dxt5_dds(rgba: np.ndarray, path, mips: int | None = None) -> dict:
    """Write `rgba` (H,W,4 uint8) as a legacy-header DXT5 DDS with a full
    (or `mips`-capped) box-filtered mip chain. Returns a small report dict
    (mip_count, level sizes) for logging/verification."""
    path = Path(path)
    levels = build_mip_chain(rgba, max_levels=mips)
    encoded = []
    for lvl in levels:
        padded = _pad_to_multiple_of_4(lvl)
        encoded.append(encode_dxt5(padded))
    h, w = rgba.shape[:2]
    ph, pw = _pad_to_multiple_of_4(rgba).shape[:2]
    linear_size = len(encoded[0])
    header = _dds_header(w, h, len(levels), linear_size)
    with open(path, "wb") as f:
        f.write(header)
        for e in encoded:
            f.write(e)
    return {"mip_count": len(levels), "level_bytes": [len(e) for e in encoded], "path": str(path)}


def psnr(a: np.ndarray, b: np.ndarray) -> float:
    a = a.astype(np.float64)
    b = b.astype(np.float64)
    mse = np.mean((a - b) ** 2)
    if mse == 0:
        return float("inf")
    return 10.0 * np.log10((255.0 ** 2) / mse)


if __name__ == "__main__":
    # Self-test: encode a synthetic RGBA image, decode with Pillow, report PSNR.
    from PIL import Image

    rng = np.random.default_rng(0)
    h, w = 64, 64
    grad_x = np.linspace(0, 255, w, dtype=np.float32)
    grad_y = np.linspace(0, 255, h, dtype=np.float32)
    r = np.tile(grad_x, (h, 1))
    g = np.tile(grad_y[:, None], (1, w))
    b = np.full((h, w), 128.0)
    a = np.where((np.indices((h, w))[0] // 8 + np.indices((h, w))[1] // 8) % 2 == 0, 255, 40).astype(np.float32)
    test = np.stack([r, g, b, a], axis=2).astype(np.uint8)

    write_dxt5_dds(test, "dds_selftest.dds")
    decoded = np.array(Image.open("dds_selftest.dds").convert("RGBA"))
    print("PSNR:", psnr(test, decoded[:h, :w]))
