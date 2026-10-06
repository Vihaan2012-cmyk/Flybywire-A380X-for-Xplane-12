//! Texture conversion: MSFS KTX2 and DDS in, textures X-Plane 12 loads out.
//!
//! The KTX2 files in MSFS 2024 packages are plain block-compressed data in a
//! KTX2 wrapper (supercompression 0), so no Basis Universal transcoder is
//! involved. Two output paths:
//!
//! - BC1/BC2/BC3 (DXT1/3/5) are passed through as DDS, keeping GPU compression
//!   and the mip chain, top row first as MSFS has them: X-Plane reads DDS
//!   like any other image. (The block-flipping helpers below remain for
//!   turning converted files over on request.)
//! - Every other format (BC4/5/6H/7, RGBA) is decoded and written as PNG, whose
//!   row order is unambiguous. X-Plane's BC7 DDS support is not confirmed, so it
//!   is not relied on.
//!
//! Either way the image ends up the way X-Plane expects, so object UVs use one
//! rule for every texture (T = 1 - V).

pub(crate) mod raw;

use raw::{as_usize, slice_at, u32_at, u64_at};

#[derive(thiserror::Error, Debug)]
pub enum TextureError {
    #[error("{container} is truncated: needed {need} bytes at offset {offset}, file has {len}")]
    Truncated {
        container: &'static str,
        offset: usize,
        need: usize,
        len: usize,
    },
    #[error("{container} is malformed: {detail}")]
    Malformed { container: &'static str, detail: String },
    #[error("unsupported texture: {0}")]
    Unsupported(String),
    #[error("PNG encoding failed: {0}")]
    Png(String),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceFormat {
    Ktx2,
    Dds,
    Png,
    Unknown,
}

const KTX2_ID: [u8; 12] = [0xAB, 0x4B, 0x54, 0x58, 0x20, 0x32, 0x30, 0xBB, 0x0D, 0x0A, 0x1A, 0x0A];

pub fn detect(data: &[u8]) -> SourceFormat {
    if data.starts_with(&KTX2_ID) {
        SourceFormat::Ktx2
    } else if data.starts_with(b"DDS ") {
        SourceFormat::Dds
    } else if data.starts_with(&[0x89, b'P', b'N', b'G']) {
        SourceFormat::Png
    } else {
        SourceFormat::Unknown
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PixelFormat {
    Bc1,
    Bc2,
    Bc3,
    Bc4,
    /// BC4 holding signed values (-1..1).
    Bc4s,
    Bc5,
    /// BC5 holding signed values, as MSFS normal maps do.
    Bc5s,
    Bc6h,
    Bc7,
    Rgba8,
}

impl PixelFormat {
    /// Bytes per 4x4 block, or `None` for uncompressed data.
    pub fn block_bytes(self) -> Option<usize> {
        match self {
            PixelFormat::Bc1 | PixelFormat::Bc4 | PixelFormat::Bc4s => Some(8),
            PixelFormat::Rgba8 => None,
            _ => Some(16),
        }
    }
}

/// Raw texture levels, largest first, in the source's row order (top first).
#[derive(Debug, Clone)]
pub struct TextureImage {
    pub width: u32,
    pub height: u32,
    pub format: PixelFormat,
    pub mips: Vec<Vec<u8>>,
}

fn level_dims(w: u32, h: u32, level: usize) -> (u32, u32) {
    ((w >> level).max(1), (h >> level).max(1))
}

fn level_size(fmt: PixelFormat, w: u32, h: u32) -> usize {
    match fmt.block_bytes() {
        Some(bb) => w.div_ceil(4) as usize * h.div_ceil(4) as usize * bb,
        None => w as usize * h as usize * 4,
    }
}

fn vk_format(vk: u32) -> Option<PixelFormat> {
    Some(match vk {
        131..=134 => PixelFormat::Bc1,
        135 | 136 => PixelFormat::Bc2,
        137 | 138 => PixelFormat::Bc3,
        139 => PixelFormat::Bc4,
        140 => PixelFormat::Bc4s,
        141 => PixelFormat::Bc5,
        142 => PixelFormat::Bc5s,
        143 | 144 => PixelFormat::Bc6h,
        145 | 146 => PixelFormat::Bc7,
        37 | 43 => PixelFormat::Rgba8,
        _ => return None,
    })
}

/// Parse a KTX2 container.
pub fn load_ktx2(data: &[u8]) -> Result<TextureImage, TextureError> {
    const C: &str = "KTX2";
    if !data.starts_with(&KTX2_ID) {
        return Err(TextureError::Malformed {
            container: C,
            detail: "bad identifier".into(),
        });
    }
    let vk = u32_at(data, 12, C)?;
    let width = u32_at(data, 20, C)?;
    let height = u32_at(data, 24, C)?.max(1);
    let depth = u32_at(data, 28, C)?;
    let layers = u32_at(data, 32, C)?;
    let faces = u32_at(data, 36, C)?;
    let levels = u32_at(data, 40, C)?.max(1) as usize;
    let scheme = u32_at(data, 44, C)?;
    if scheme != 0 {
        let name = match scheme {
            1 => "BasisLZ",
            2 => "Zstandard",
            3 => "ZLIB",
            _ => "unknown",
        };
        return Err(TextureError::Unsupported(format!(
            "KTX2 supercompression {scheme} ({name})"
        )));
    }
    if depth > 1 || layers > 1 || faces != 1 {
        return Err(TextureError::Unsupported("KTX2 arrays, cubes and volumes".into()));
    }
    if width == 0 || levels > 16 {
        return Err(TextureError::Malformed {
            container: C,
            detail: format!("{width}x{height} with {levels} levels"),
        });
    }
    let format = vk_format(vk).ok_or_else(|| TextureError::Unsupported(format!("KTX2 vkFormat {vk}")))?;
    let mut mips = Vec::with_capacity(levels);
    for i in 0..levels {
        let entry = 80 + i * 24;
        let offset = as_usize(u64_at(data, entry, C)?, C, "level offset")?;
        let length = as_usize(u64_at(data, entry + 8, C)?, C, "level length")?;
        let (w, h) = level_dims(width, height, i);
        let expected = level_size(format, w, h);
        if length < expected {
            return Err(TextureError::Malformed {
                container: C,
                detail: format!("level {i} holds {length} bytes, needs {expected}"),
            });
        }
        mips.push(slice_at(data, offset, expected, C)?.to_vec());
    }
    Ok(TextureImage {
        width,
        height,
        format,
        mips,
    })
}

/// Parse a DDS file (classic header, optionally with the DX10 extension).
pub fn load_dds(data: &[u8]) -> Result<TextureImage, TextureError> {
    const C: &str = "DDS";
    if !data.starts_with(b"DDS ") || u32_at(data, 4, C)? != 124 {
        return Err(TextureError::Malformed {
            container: C,
            detail: "bad header".into(),
        });
    }
    let height = u32_at(data, 12, C)?.max(1);
    let width = u32_at(data, 16, C)?.max(1);
    let mip_count = (u32_at(data, 28, C)?.max(1) as usize).min(16);
    let pf_flags = u32_at(data, 80, C)?;
    let four_cc = slice_at(data, 84, 4, C)?;
    let mut start = 128;
    let (format, bgra) = if pf_flags & 0x4 != 0 {
        match four_cc {
            b"DXT1" => (PixelFormat::Bc1, false),
            b"DXT2" | b"DXT3" => (PixelFormat::Bc2, false),
            b"DXT4" | b"DXT5" => (PixelFormat::Bc3, false),
            b"ATI1" | b"BC4U" => (PixelFormat::Bc4, false),
            b"BC4S" => (PixelFormat::Bc4s, false),
            b"ATI2" | b"BC5U" => (PixelFormat::Bc5, false),
            b"BC5S" => (PixelFormat::Bc5s, false),
            b"DX10" => {
                start = 148;
                let dxgi = u32_at(data, 128, C)?;
                match dxgi {
                    70..=72 => (PixelFormat::Bc1, false),
                    73..=75 => (PixelFormat::Bc2, false),
                    76..=78 => (PixelFormat::Bc3, false),
                    79 | 80 => (PixelFormat::Bc4, false),
                    81 => (PixelFormat::Bc4s, false),
                    82 | 83 => (PixelFormat::Bc5, false),
                    84 => (PixelFormat::Bc5s, false),
                    94..=96 => (PixelFormat::Bc6h, false),
                    97..=99 => (PixelFormat::Bc7, false),
                    27..=29 => (PixelFormat::Rgba8, false),
                    87 | 91 => (PixelFormat::Rgba8, true),
                    other => return Err(TextureError::Unsupported(format!("DDS DXGI format {other}"))),
                }
            }
            other => {
                return Err(TextureError::Unsupported(format!(
                    "DDS FourCC {:?}",
                    String::from_utf8_lossy(other)
                )))
            }
        }
    } else if u32_at(data, 88, C)? == 32 {
        // Uncompressed 32-bit: the red mask says whether it is RGBA or BGRA.
        (PixelFormat::Rgba8, u32_at(data, 92, C)? == 0x00FF_0000)
    } else {
        return Err(TextureError::Unsupported("DDS pixel format".into()));
    };

    let mut mips = Vec::with_capacity(mip_count);
    let mut pos = start;
    for i in 0..mip_count {
        let (w, h) = level_dims(width, height, i);
        let n = level_size(format, w, h);
        let Ok(level) = slice_at(data, pos, n, C) else {
            if i == 0 {
                return Err(TextureError::Truncated {
                    container: C,
                    offset: pos,
                    need: n,
                    len: data.len(),
                });
            }
            break; // a short mip chain is still a usable texture
        };
        let mut level = level.to_vec();
        if bgra {
            for px in level.chunks_exact_mut(4) {
                px.swap(0, 2);
            }
        }
        mips.push(level);
        pos += n;
    }
    Ok(TextureImage {
        width,
        height,
        format,
        mips,
    })
}

/// Parse either container.
pub fn load(data: &[u8]) -> Result<TextureImage, TextureError> {
    match detect(data) {
        SourceFormat::Ktx2 => load_ktx2(data),
        SourceFormat::Dds => load_dds(data),
        SourceFormat::Png => Err(TextureError::Unsupported("PNG needs no conversion".into())),
        SourceFormat::Unknown => Err(TextureError::Unsupported("not a KTX2, DDS or PNG file".into())),
    }
}

/// Decode one level to RGBA8, top row first.
pub fn decode_rgba8(img: &TextureImage, level: usize) -> Result<Vec<u8>, TextureError> {
    let data = img
        .mips
        .get(level)
        .ok_or_else(|| TextureError::Unsupported(format!("no mip level {level}")))?;
    let (w, h) = level_dims(img.width, img.height, level);
    let (w, h) = (w as usize, h as usize);
    let Some(bb) = img.format.block_bytes() else {
        return Ok(data.clone());
    };
    // One- and two-channel formats are decoded here, signed ones included.
    let channels = match img.format {
        PixelFormat::Bc4 => Some((1, false)),
        PixelFormat::Bc4s => Some((1, true)),
        PixelFormat::Bc5 => Some((2, false)),
        PixelFormat::Bc5s => Some((2, true)),
        _ => None,
    };
    if let Some((n, signed)) = channels {
        let (bw, bh) = (w.div_ceil(4), h.div_ceil(4));
        let mut out = vec![0u8; w * h * 4];
        for by in 0..bh {
            for bx in 0..bw {
                let at = (by * bw + bx) * bb;
                let Some(src) = data.get(at..at + bb) else {
                    return Err(TextureError::Truncated {
                        container: "texture level",
                        offset: at,
                        need: bb,
                        len: data.len(),
                    });
                };
                let r = bc4_block(&src[0..8], signed);
                let g = if n == 2 { bc4_block(&src[8..16], signed) } else { r };
                for py in 0..4 {
                    for px in 0..4 {
                        let (x, y) = (bx * 4 + px, by * 4 + py);
                        if x < w && y < h {
                            let (i, o) = (py * 4 + px, (y * w + x) * 4);
                            let b = if n == 1 { r[i] } else { 0 };
                            out[o..o + 4].copy_from_slice(&[r[i], g[i], b, 255]);
                        }
                    }
                }
            }
        }
        return Ok(out);
    }
    let decode: fn(&[u8], &mut [u32]) = match img.format {
        PixelFormat::Bc1 => texture2ddecoder::decode_bc1_block,
        PixelFormat::Bc2 => texture2ddecoder::decode_bc2_block,
        PixelFormat::Bc3 => texture2ddecoder::decode_bc3_block,
        PixelFormat::Bc4 | PixelFormat::Bc4s | PixelFormat::Bc5 | PixelFormat::Bc5s => unreachable!("decoded above"),
        PixelFormat::Bc6h => texture2ddecoder::decode_bc6_block_unsigned,
        PixelFormat::Bc7 => texture2ddecoder::decode_bc7_block,
        PixelFormat::Rgba8 => unreachable!("handled above"),
    };
    let (bw, bh) = (w.div_ceil(4), h.div_ceil(4));
    let mut out = vec![0u8; w * h * 4];
    let mut block = [0u32; 16];
    for by in 0..bh {
        for bx in 0..bw {
            let at = (by * bw + bx) * bb;
            let Some(src) = data.get(at..at + bb) else {
                return Err(TextureError::Truncated {
                    container: "texture level",
                    offset: at,
                    need: bb,
                    len: data.len(),
                });
            };
            decode(src, &mut block);
            for py in 0..4 {
                for px in 0..4 {
                    let (x, y) = (bx * 4 + px, by * 4 + py);
                    if x < w && y < h {
                        // The decoder packs pixels as little-endian B, G, R, A.
                        let [b, g, r, a] = block[py * 4 + px].to_le_bytes();
                        let o = (y * w + x) * 4;
                        out[o..o + 4].copy_from_slice(&[r, g, b, a]);
                    }
                }
            }
        }
    }
    Ok(out)
}

/// Decode one BC4 block (one channel) to 16 values. Signed data is mapped so
/// that -1 is 0 and +1 is 255.
fn bc4_block(b: &[u8], signed: bool) -> [u8; 16] {
    let raw = |x: u8| if signed { x as i8 as i16 } else { x as i16 };
    let unit = |x: u8| {
        if signed {
            (x as i8 as f32 / 127.0).max(-1.0)
        } else {
            x as f32 / 255.0
        }
    };
    let (e0, e1) = (unit(b[0]), unit(b[1]));
    let mut pal = [e0, e1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    if raw(b[0]) > raw(b[1]) {
        for i in 1..7 {
            pal[i + 1] = ((7 - i) as f32 * e0 + i as f32 * e1) / 7.0;
        }
    } else {
        for i in 1..5 {
            pal[i + 1] = ((5 - i) as f32 * e0 + i as f32 * e1) / 5.0;
        }
        pal[6] = if signed { -1.0 } else { 0.0 };
        pal[7] = 1.0;
    }
    let mut bits = 0u64;
    for (i, &x) in b[2..8].iter().enumerate() {
        bits |= (x as u64) << (8 * i);
    }
    let mut out = [0u8; 16];
    for (p, o) in out.iter_mut().enumerate() {
        let v = pal[((bits >> (3 * p)) & 7) as usize];
        let v = if signed { v * 0.5 + 0.5 } else { v };
        *o = (v.clamp(0.0, 1.0) * 255.0).round() as u8;
    }
    out
}

/// Flip the 4x4 pixel rows of a 64-bit BC1-style colour block.
fn flip_colour_block(b: &mut [u8]) {
    b[4..8].reverse(); // one index byte per pixel row
}

/// Flip a BC3 alpha block: 3-bit indices, 12 bits per pixel row.
fn flip_bc3_alpha(b: &mut [u8]) {
    let mut bits = 0u64;
    for (i, &byte) in b[2..8].iter().enumerate() {
        bits |= (byte as u64) << (8 * i);
    }
    let rows: Vec<u64> = (0..4).map(|r| (bits >> (12 * r)) & 0xFFF).collect();
    let mut flipped = 0u64;
    for (r, row) in rows.iter().rev().enumerate() {
        flipped |= row << (12 * r);
    }
    for (i, byte) in b[2..8].iter_mut().enumerate() {
        *byte = (flipped >> (8 * i)) as u8;
    }
}

/// Vertically flip one level of BC1/BC2/BC3 data without decoding it.
///
/// Only possible when the height is a whole number of blocks; otherwise the
/// padding rows of the last block would move into view.
pub fn flip_bc_level(fmt: PixelFormat, data: &[u8], w: u32, h: u32) -> Option<Vec<u8>> {
    if h % 4 != 0 || !matches!(fmt, PixelFormat::Bc1 | PixelFormat::Bc2 | PixelFormat::Bc3) {
        return None;
    }
    let bb = fmt.block_bytes()?;
    let (bw, bh) = (w.div_ceil(4) as usize, (h / 4) as usize);
    let row_bytes = bw * bb;
    if data.len() < row_bytes * bh {
        return None;
    }
    let mut out = Vec::with_capacity(row_bytes * bh);
    for row in (0..bh).rev() {
        let mut blocks = data[row * row_bytes..(row + 1) * row_bytes].to_vec();
        for b in blocks.chunks_exact_mut(bb) {
            match fmt {
                PixelFormat::Bc1 => flip_colour_block(b),
                PixelFormat::Bc2 => {
                    // Explicit 4-bit alpha, two bytes per pixel row, then colour.
                    let alpha: Vec<[u8; 2]> = b[0..8].chunks_exact(2).map(|c| [c[0], c[1]]).rev().collect();
                    for (i, pair) in alpha.iter().enumerate() {
                        b[i * 2..i * 2 + 2].copy_from_slice(pair);
                    }
                    flip_colour_block(&mut b[8..16]);
                }
                PixelFormat::Bc3 => {
                    flip_bc3_alpha(&mut b[0..8]);
                    flip_colour_block(&mut b[8..16]);
                }
                _ => return None,
            }
        }
        out.extend_from_slice(&blocks);
    }
    Some(out)
}

/// Mirror a BC1-style colour block left to right: two bits per pixel, one
/// index byte per row.
fn mirror_colour_block(b: &mut [u8]) {
    for byte in &mut b[4..8] {
        let v = *byte;
        let mut o = 0u8;
        for x in 0..4 {
            o |= ((v >> (2 * x)) & 3) << (2 * (3 - x));
        }
        *byte = o;
    }
}

/// Mirror a BC2 explicit alpha block: four bits per pixel, a u16 per row.
fn mirror_bc2_alpha(b: &mut [u8]) {
    for r in 0..4 {
        let v = u16::from_le_bytes([b[2 * r], b[2 * r + 1]]);
        let mut o = 0u16;
        for x in 0..4 {
            o |= ((v >> (4 * x)) & 0xF) << (4 * (3 - x));
        }
        b[2 * r..2 * r + 2].copy_from_slice(&o.to_le_bytes());
    }
}

/// Mirror a BC3 alpha block: three-bit indices, 12 bits per pixel row.
fn mirror_bc3_alpha(b: &mut [u8]) {
    let mut bits = 0u64;
    for (i, &byte) in b[2..8].iter().enumerate() {
        bits |= (byte as u64) << (8 * i);
    }
    let mut out = 0u64;
    for r in 0..4 {
        let row = (bits >> (12 * r)) & 0xFFF;
        let mut m = 0u64;
        for x in 0..4 {
            m |= ((row >> (3 * x)) & 7) << (3 * (3 - x));
        }
        out |= m << (12 * r);
    }
    for (i, byte) in b[2..8].iter_mut().enumerate() {
        *byte = (out >> (8 * i)) as u8;
    }
}

/// Mirror one level of BC1/BC2/BC3 data left to right without decoding it.
/// Only possible when the width is a whole number of blocks.
pub fn mirror_bc_level(fmt: PixelFormat, data: &[u8], w: u32, h: u32) -> Option<Vec<u8>> {
    if w % 4 != 0 || !matches!(fmt, PixelFormat::Bc1 | PixelFormat::Bc2 | PixelFormat::Bc3) {
        return None;
    }
    let bb = fmt.block_bytes()?;
    let (bw, bh) = ((w / 4) as usize, h.div_ceil(4) as usize);
    let row_bytes = bw * bb;
    if data.len() < row_bytes * bh {
        return None;
    }
    let mut out = Vec::with_capacity(row_bytes * bh);
    for row in 0..bh {
        let blocks = &data[row * row_bytes..(row + 1) * row_bytes];
        for block in blocks.chunks_exact(bb).rev() {
            let mut b = block.to_vec();
            match fmt {
                PixelFormat::Bc1 => mirror_colour_block(&mut b),
                PixelFormat::Bc2 => {
                    mirror_bc2_alpha(&mut b[0..8]);
                    mirror_colour_block(&mut b[8..16]);
                }
                _ => {
                    mirror_bc3_alpha(&mut b[0..8]);
                    mirror_colour_block(&mut b[8..16]);
                }
            }
            out.extend_from_slice(&b);
        }
    }
    Some(out)
}

/// Flip a converted texture file (the DDS or PNG this converter writes)
/// top to bottom and/or left to right. DDS data is rearranged block by
/// block, losslessly; mip levels too small to rearrange are dropped.
pub fn transform_texture_file(data: &[u8], flip_v: bool, flip_h: bool) -> Result<Vec<u8>, TextureError> {
    if !flip_v && !flip_h {
        return Ok(data.to_vec());
    }
    match detect(data) {
        SourceFormat::Png => {
            let mut img = image::load_from_memory_with_format(data, image::ImageFormat::Png)
                .map_err(|e| TextureError::Png(e.to_string()))?
                .to_rgba8();
            if flip_v {
                img = image::imageops::flip_vertical(&img);
            }
            if flip_h {
                img = image::imageops::flip_horizontal(&img);
            }
            let (w, h) = img.dimensions();
            let mut out = Vec::new();
            let encoder = image::codecs::png::PngEncoder::new(&mut out);
            image::ImageEncoder::write_image(encoder, img.as_raw(), w, h, image::ExtendedColorType::Rgba8)
                .map_err(|e| TextureError::Png(e.to_string()))?;
            Ok(out)
        }
        SourceFormat::Dds => {
            let img = load_dds(data)?;
            let mut levels = Vec::new();
            for (i, level) in img.mips.iter().enumerate() {
                let (w, h) = level_dims(img.width, img.height, i);
                let mut l = Some(level.clone());
                if flip_v {
                    l = l.and_then(|d| flip_bc_level(img.format, &d, w, h));
                }
                if flip_h {
                    l = l.and_then(|d| mirror_bc_level(img.format, &d, w, h));
                }
                match l {
                    Some(d) => levels.push(d),
                    None if i == 0 => {
                        return Err(TextureError::Unsupported(format!(
                            "{}x{} {:?} cannot be flipped block by block",
                            img.width, img.height, img.format
                        )))
                    }
                    None => break,
                }
            }
            // Same header, with the mip count of what is left.
            let mut out = data[..128].to_vec();
            out[28..32].copy_from_slice(&(levels.len() as u32).to_le_bytes());
            for l in levels {
                out.extend_from_slice(&l);
            }
            Ok(out)
        }
        _ => Err(TextureError::Unsupported("only converted DDS and PNG textures can be flipped".into())),
    }
}

/// Write BC1/BC2/BC3 data as a DDS for X-Plane: top row first, as the source
/// has it. (X-Plane reads DDS like any other image; turning the rows over,
/// as this once did, put every texture upside down: drawing a converted
/// aircraft's fuselage from its files both ways showed only top-first gives
/// the livery, and X-Plane showed the other.)
pub fn to_xplane_dds(img: &TextureImage) -> Result<Vec<u8>, TextureError> {
    let four_cc: &[u8; 4] = match img.format {
        PixelFormat::Bc1 => b"DXT1",
        PixelFormat::Bc2 => b"DXT3",
        PixelFormat::Bc3 => b"DXT5",
        other => {
            return Err(TextureError::Unsupported(format!(
                "{other:?} cannot be written as legacy DDS"
            )))
        }
    };
    let bb = img.format.block_bytes().unwrap_or(16);
    let mut levels = Vec::new();
    for (i, data) in img.mips.iter().enumerate() {
        let (w, h) = level_dims(img.width, img.height, i);
        let need = (w.div_ceil(4) * h.div_ceil(4)) as usize * bb;
        if data.len() < need {
            break;
        }
        levels.push(data[..need].to_vec());
    }
    if levels.is_empty() {
        return Err(TextureError::Unsupported(format!(
            "{}x{} is not a whole number of blocks",
            img.width, img.height
        )));
    }
    let mut out = Vec::with_capacity(128 + levels.iter().map(Vec::len).sum::<usize>());
    let put = |out: &mut Vec<u8>, v: u32| out.extend_from_slice(&v.to_le_bytes());
    out.extend_from_slice(b"DDS ");
    put(&mut out, 124);
    // CAPS | HEIGHT | WIDTH | PIXELFORMAT | MIPMAPCOUNT | LINEARSIZE
    put(&mut out, 0x1 | 0x2 | 0x4 | 0x1000 | 0x20000 | 0x80000);
    put(&mut out, img.height);
    put(&mut out, img.width);
    put(&mut out, levels[0].len() as u32);
    put(&mut out, 0); // depth
    put(&mut out, levels.len() as u32);
    out.extend_from_slice(&[0u8; 44]); // reserved
    put(&mut out, 32); // pixel format size
    put(&mut out, 0x4); // FOURCC
    out.extend_from_slice(four_cc);
    out.extend_from_slice(&[0u8; 20]); // bit count and masks
    let caps = 0x1000 | if levels.len() > 1 { 0x8 | 0x40_0000 } else { 0 };
    put(&mut out, caps);
    out.extend_from_slice(&[0u8; 16]); // caps2-4, reserved
    debug_assert_eq!(out.len(), 128);
    for l in levels {
        out.extend_from_slice(&l);
    }
    Ok(out)
}

/// Encode level 0 as a PNG (top row first, as PNG always is).
pub fn to_png(img: &TextureImage) -> Result<Vec<u8>, TextureError> {
    let rgba = decode_rgba8(img, 0)?;
    let mut out = Vec::new();
    let encoder = image::codecs::png::PngEncoder::new(&mut out);
    image::ImageEncoder::write_image(encoder, &rgba, img.width, img.height, image::ExtendedColorType::Rgba8)
        .map_err(|e| TextureError::Png(e.to_string()))?;
    Ok(out)
}

/// A converted texture and the file extension to give it.
#[derive(Debug, Clone)]
pub struct Converted {
    pub bytes: Vec<u8>,
    pub extension: &'static str,
    /// Roughly what the texture occupies in video memory, mip chain included.
    pub vram_bytes: usize,
}

/// Convert any MSFS texture into something X-Plane 12 loads.
pub fn convert_for_xplane(data: &[u8]) -> Result<Converted, TextureError> {
    convert_for_xplane_capped(data, u32::MAX)
}

/// Whether a texture goes out as DDS: BC1-3 whose top level flips cleanly.
fn writes_dds(img: &TextureImage) -> bool {
    matches!(img.format, PixelFormat::Bc1 | PixelFormat::Bc2 | PixelFormat::Bc3)
        && img
            .mips
            .first()
            .is_some_and(|m| flip_bc_level(img.format, m, img.width, img.height).is_some())
}

/// The extension [`convert_for_xplane_capped`] gives this texture at any cap,
/// so objects can name a texture before it is written.
pub fn output_extension(data: &[u8]) -> Result<&'static str, TextureError> {
    if detect(data) == SourceFormat::Png {
        return Ok("png");
    }
    Ok(if writes_dds(&load(data)?) { "dds" } else { "png" })
}

/// The first level whose longer side fits `max_side`, or the smallest level
/// allowed. DDS output needs every level a whole number of blocks tall.
fn first_level_within(img: &TextureImage, max_side: u32, whole_blocks: bool) -> usize {
    let mut best = 0;
    for i in 0..img.mips.len() {
        let (w, h) = level_dims(img.width, img.height, i);
        if i > 0 && whole_blocks && h % 4 != 0 {
            break;
        }
        best = i;
        if w.max(h) <= max_side {
            break;
        }
    }
    best
}

/// The image from level `first` down.
fn from_level(img: &TextureImage, first: usize) -> TextureImage {
    let (width, height) = level_dims(img.width, img.height, first);
    TextureImage {
        width,
        height,
        format: img.format,
        mips: img.mips[first..].to_vec(),
    }
}

/// Encode RGBA8 as PNG, shrunk to fit `max_side` when it is larger.
fn png_within(rgba: Vec<u8>, w: u32, h: u32, max_side: u32) -> Result<Converted, TextureError> {
    let mut img = image::RgbaImage::from_raw(w, h, rgba)
        .ok_or_else(|| TextureError::Unsupported("pixel data does not match its size".into()))?;
    if w.max(h) > max_side {
        let k = max_side as f64 / w.max(h) as f64;
        let nw = ((w as f64 * k).round() as u32).max(1);
        let nh = ((h as f64 * k).round() as u32).max(1);
        img = image::imageops::resize(&img, nw, nh, image::imageops::FilterType::Triangle);
    }
    let (w, h) = img.dimensions();
    let mut out = Vec::new();
    let encoder = image::codecs::png::PngEncoder::new(&mut out);
    image::ImageEncoder::write_image(encoder, img.as_raw(), w, h, image::ExtendedColorType::Rgba8)
        .map_err(|e| TextureError::Png(e.to_string()))?;
    Ok(Converted {
        bytes: out,
        extension: "png",
        // X-Plane compresses PNGs as it loads them, to about a byte a pixel.
        vram_bytes: w as usize * h as usize * 4 / 3,
    })
}

/// Shrink RGBA8 pixels to fit `max_side`, or to exactly `size` when given.
fn shrink(rgba: Vec<u8>, w: u32, h: u32, max_side: u32, size: Option<(u32, u32)>) -> (u32, u32, Vec<u8>) {
    let target = size.unwrap_or_else(|| {
        if w.max(h) <= max_side {
            (w, h)
        } else {
            let k = max_side as f64 / w.max(h) as f64;
            (((w as f64 * k).round() as u32).max(1), ((h as f64 * k).round() as u32).max(1))
        }
    });
    if target == (w, h) {
        return (w, h, rgba);
    }
    match image::RgbaImage::from_raw(w, h, rgba) {
        Some(img) => {
            let out = image::imageops::resize(&img, target.0, target.1, image::imageops::FilterType::Triangle);
            (target.0, target.1, out.into_raw())
        }
        None => (target.0, target.1, vec![0; (target.0 * target.1 * 4) as usize]),
    }
}

/// Decode any texture to RGBA8, top row first, at most `max_side` on its
/// longer side (from a smaller mip level where there is one).
pub fn decode_within(data: &[u8], max_side: u32) -> Result<(u32, u32, Vec<u8>), TextureError> {
    if detect(data) == SourceFormat::Png {
        let img = image::load_from_memory_with_format(data, image::ImageFormat::Png)
            .map_err(|e| TextureError::Png(e.to_string()))?
            .to_rgba8();
        let (w, h) = img.dimensions();
        return Ok(shrink(img.into_raw(), w, h, max_side, None));
    }
    let img = load(data)?;
    let small = from_level(&img, first_level_within(&img, max_side, false));
    let rgba = decode_rgba8(&small, 0)?;
    Ok(shrink(rgba, small.width, small.height, max_side, None))
}

/// X-Plane gloss (0..255) for an MSFS roughness (0..255). X-Plane's gloss is
/// not simply one minus MSFS's roughness: taken so, FBW's painted cockpit
/// panels (MSFS roughness about 0.5) came out two to three times as glossy as
/// Laminar's, sparkled on every scratch in the normal map and mirrored the sun
/// as white glare. Gloss = (1 - roughness)^2.2 lands MSFS values where
/// Laminar's own maps sit: the A330's cockpit near 43 of 255, its fuselage
/// paint near 207.
pub fn gloss(roughness: u8) -> u8 {
    ((1.0 - roughness as f32 / 255.0).powf(2.2) * 255.0).round() as u8
}

/// X-Plane metalness (0..255) for an MSFS COMP metalness paired with the
/// surface's own (already converted) gloss. MSFS's renderer softens a metal
/// reading with image-based lighting, so a half-metal, moderately rough
/// surface reads as a subtly sheened cap, not bare metal: the FBW A380's
/// glareshield CHRONO push button (COMP blue/metalness 217 of 255, green/
/// roughness 109 of 255, gloss 75 of 255) is a black plastic cap with a
/// slight sheen in MSFS. X-Plane's material_gloss shading has no such soft
/// reflection over a cockpit's small parts, only the sun's specular
/// highlight, and that highlight is not tinted by the (here near-black)
/// albedo the way a true metal's would be: the raw metalness rendered a hard
/// white-yellow ring around the button's bevel over an otherwise black face.
/// Scaling metalness by gloss leaves genuinely smooth, reflective metal (low
/// roughness, high gloss, so the two multiply to nearly the input metalness)
/// close to full strength, while damping metalness on rougher surfaces where
/// MSFS's blend was doing double duty as a stylised sheen rather than true
/// bare metal.
pub fn metal(metalness: u8, gloss: u8) -> u8 {
    (metalness as f32 * gloss as f32 / 255.0).round() as u8
}

/// How much detail a level of an RGB8 image would lose by being halved:
/// the RMS difference, over 0-255, between each texel and the mean of the
/// 2x2 block it belongs to. That block mean is exactly what the half-size
/// image would hold, so this is the error of reconstructing the original
/// from it.
///
/// (2026-09-27: this replaces an in-progress `worst_tile_error` rewrite --
/// tile-wise, measured against the true original every step rather than
/// the previous level -- that was still uncommitted here and never
/// reached the sizes it was meant to reproduce: against the installed,
/// user-approved cockpit it made every NML/MAT map this function's own
/// caller can shrink 2-8x too big, e.g. `A380_COCKPIT_AFT_MISC01_4K_NORMAL`
/// 256 -> 2048, because comparing to the true original lets error that
/// used to look fine one halving at a time accumulate into "too much"
/// several steps earlier than before. Its actual purpose -- finding the
/// maps whose real, load-bearing detail (FCU's embossed digits, a metal
/// bezel's edge) a whole-canvas average could hide behind a large flat
/// surround -- is done, not lost: that is exactly how
/// `msfs2xp-aircraft`'s `LETTERED_NORMAL_MAPS`/`LETTERED_MATERIAL_MAPS`
/// were found (their own doc comments cite the same worst-tile/RG-std
/// sweep), and those lists already force the maps it would have caught
/// to a 1024 floor regardless of what this function decides. Wiring it
/// into every other map's live shrink decision was the redundant, too-
/// conservative part.)
fn detail_lost_by_halving(px: &[u8], w: u32, h: u32) -> f32 {
    let (nw, nh) = (w / 2, h / 2);
    if nw == 0 || nh == 0 {
        return f32::MAX;
    }
    let (mut se, mut n) = (0f64, 0u64);
    for y in 0..nh {
        for x in 0..nw {
            for k in 0..3 {
                let at = |dx: u32, dy: u32| px[(((2 * y + dy) * w + 2 * x + dx) * 3 + k) as usize] as f64;
                let mean = (at(0, 0) + at(1, 0) + at(0, 1) + at(1, 1)) / 4.0;
                for (dx, dy) in [(0u32, 0u32), (1, 0), (0, 1), (1, 1)] {
                    let d = at(dx, dy) - mean;
                    se += d * d;
                }
                n += 4;
            }
        }
    }
    (se / n.max(1) as f64).sqrt() as f32
}

/// Halve an RGB8 image for as long as halving costs less than `limit` RMS,
/// never below `floor` on a side. Both maps this is used on are linear data
/// -- a normal's x and y, a surface's metalness and gloss -- so the filter
/// is a plain box average, with no gamma anywhere.
///
/// PNG is uncompressed on the GPU, four bytes a texel plus mips, so a side
/// halved is four times the video memory back. Most of these maps do not
/// use the resolution they are given: measured over FlyByWire's A380, the
/// median normal map loses 2.3 RMS by being halved and the median metal
/// map 3.5, while the handful that carry real detail -- the cockpit decal
/// normals with the panel lettering embossed in them, the knob and
/// pushbutton gloss -- cost 9 to 14 and keep their size. Halving only what
/// measures flat took the aircraft's referenced textures down by 624 MB
/// without touching anything the eye is on.
///
/// `floor` is not always [`FLAT_FLOOR`]: a caller that already knows a
/// particular map's detail cannot be told apart from an ordinary flat
/// map's own noise by any statistic run over it (see
/// `msfs2xp-aircraft`'s `LETTERED_NORMAL_MAPS`) can raise it to guarantee a
/// minimum size outright, without asking this function to make that call.
fn shrink_while_flat(mut px: Vec<u8>, mut w: u32, mut h: u32, limit: f32, floor: u32) -> (Vec<u8>, u32, u32) {
    while w > floor && h > floor && detail_lost_by_halving(&px, w, h) <= limit {
        let (nw, nh) = (w / 2, h / 2);
        let mut out = vec![0u8; (nw * nh * 3) as usize];
        for y in 0..nh {
            for x in 0..nw {
                for k in 0..3 {
                    let at = |dx: u32, dy: u32| px[(((2 * y + dy) * w + 2 * x + dx) * 3 + k) as usize] as u32;
                    out[((y * nw + x) * 3 + k) as usize] = ((at(0, 0) + at(1, 0) + at(0, 1) + at(1, 1)) / 4) as u8;
                }
            }
        }
        px = out;
        w = nw;
        h = nh;
    }
    (px, w, h)
}

/// The most RMS, over 0-255, a normal map may lose by being halved. Tighter
/// than a material map's: a normal drives the shading directly, so an error
/// in one shows as a change of light across a surface rather than a change
/// of finish.
const NORMAL_FLAT_LIMIT: f32 = 5.0;
/// The same for a metal/gloss map, which says how a whole surface behaves
/// rather than what shape it is.
const MATERIAL_FLAT_LIMIT: f32 = 8.0;
/// Never shrink either below this: past it a map stops being able to tell
/// two adjacent controls apart at all. Some callers ask for a higher floor
/// than this default (see [`shrink_while_flat`]'s doc).
pub const FLAT_FLOOR: u32 = 256;

/// An uncompressed RGB PNG, as X-Plane wants its normal and material maps.
fn rgb_png(px: &[u8], w: u32, h: u32) -> Result<Converted, TextureError> {
    let mut bytes = Vec::new();
    let encoder = image::codecs::png::PngEncoder::new(&mut bytes);
    image::ImageEncoder::write_image(encoder, px, w, h, image::ExtendedColorType::Rgb8)
        .map_err(|e| TextureError::Png(e.to_string()))?;
    Ok(Converted {
        bytes,
        extension: "png",
        // Uncompressed, four bytes a pixel on the GPU, plus mips.
        vram_bytes: w as usize * h as usize * 4 * 4 / 3,
    })
}

/// X-Plane 12's normal map (`TEXTURE_MAP normal`), laid out as Laminar's:
/// red and green the normal, blue empty. Green is inverted, because MSFS
/// packs DirectX-style normals (its glTF says so: `ASOBO_normal_map_convention`
/// is `DirectX`, green pointing down the image) and X-Plane reads green as up.
///
/// `used`: the UV-space triangles (`[[u, v]; 3]`, 0..1) at least one mesh
/// actually draws with into this normal map; empty when unknown. The same
/// remedy `material_png` applies to a COMP atlas's unused padding applies
/// here: an atlas normal map (FlyByWire's FCU and glareshield among them)
/// can leave most of its canvas unused, and without dilating the real
/// content out into it first, that padding's own arbitrary values would
/// dilute how much detail the used part is measured to lose by shrinking.
///
/// `floor`: never shrink below this side length (usually [`FLAT_FLOOR`],
/// see its doc and [`shrink_while_flat`]'s).
pub fn normal_png(normal: &[u8], max_side: u32, used: &[[[f32; 2]; 3]], floor: u32) -> Result<Converted, TextureError> {
    let (w, h, n) = decode_within(normal, max_side)?;
    let mut px: Vec<u8> = n.chunks_exact(4).flat_map(|p| [p[0], 255 - p[1], 0]).collect();
    dilate_outside_uv(&mut px, w, h, used);
    let (px, w, h) = shrink_while_flat(px, w, h, NORMAL_FLAT_LIMIT, floor);
    rgb_png(&px, w, h)
}

/// X-Plane 12's metal/gloss map (`TEXTURE_MAP material_gloss`), laid out as
/// Laminar's: red the metalness, green the gloss, blue empty. Made from the
/// MSFS "COMP" texture (occlusion, roughness, metalness in red, green, blue)
/// at its own size, which is often half the normal map's. Without one: not
/// metal, the gloss of roughness 0.5. Metalness is scaled by the surface's
/// own gloss (see [`metal`]), so a metal reading on a rough surface does not
/// glare as bright, un-tinted "chrome" in X-Plane's cockpit lighting.
///
/// `used`: the UV-space triangles (`[[u, v]; 3]`, 0..1) at least one mesh
/// actually draws with into this COMP texture; empty when unknown. MSFS
/// packs several small parts' COMP data into one shared atlas (FlyByWire's
/// A380 pushbutton and overhead-knob caps among them) with the rest of the
/// canvas left at whatever default the exporter fills unused space with -
/// roughness 0 there, gloss 255 once converted: mirror-smooth. No mesh ever
/// samples that padding directly, but it is not "wrong data" either - it is
/// the source's own roughness 0, read literally - so the defect is not in
/// the value, it is that X-Plane's own mip chain (built at load time from
/// the single image this function writes) still averages it into the real
/// content at any mip small enough for one texel to span both. A cockpit
/// pushbutton or knob cap is exactly that: small and usually distant enough
/// that a coarse mip is what gets sampled, so the real, correctly dark/matte
/// button face blew out to a flat mirror-bright white rectangle the size of
/// its own coarse mip texel. This dilates the real content in `used`
/// outward to cover the unused area first - the same remedy
/// [`dilate_transparent`] applies to a decal's transparent background, and
/// just as honest: it makes the padding say what its nearest real neighbour
/// says, not something no surface ever meant.
///
/// `floor`: never shrink below this side length (usually [`FLAT_FLOOR`]; see
/// its doc and [`shrink_while_flat`]'s). A handful of COMP maps carry a
/// gloss channel whose real, load-bearing contrast -- a switch's metal
/// bezel against its matte face, a keycap's edge -- no statistic run over
/// the map can reliably tell apart from an ordinary flat map's own noise,
/// the same problem T01 found for a normal map's relief (see
/// `msfs2xp-aircraft`'s `LETTERED_MATERIAL_MAPS`); a caller that already
/// knows which ones can raise `floor` to keep a guaranteed minimum size,
/// bypassing the flatness heuristic outright.
pub fn material_png(comp: Option<&[u8]>, max_side: u32, used: &[[[f32; 2]; 3]], floor: u32) -> Result<Converted, TextureError> {
    let Some(comp) = comp else {
        return rgb_png(&[0, gloss(128), 0].repeat(16), 4, 4);
    };
    let (w, h, c) = decode_within(comp, max_side)?;
    let mut px: Vec<u8> = c
        .chunks_exact(4)
        .flat_map(|p| {
            let g = gloss(p[1]);
            [metal(p[2], g), g, 0]
        })
        .collect();
    dilate_outside_uv(&mut px, w, h, used);
    // After the dilation, not before: the flatness measure has to see the
    // padding already carrying its neighbours' values, or a canvas that is
    // mostly unused padding reads as flat when its real content is not.
    let (px, w, h) = shrink_while_flat(px, w, h, MATERIAL_FLAT_LIMIT, floor);
    rgb_png(&px, w, h)
}

/// Flood an RGB8 image's real content (the union of `used`, UV-space
/// triangles) outward into the rest of the canvas, so nothing beyond it
/// keeps whatever value was there before. Each texel outside `used` takes
/// its nearest 4-connected neighbour's colour, propagated outward a
/// breadth-first ring at a time from every texel inside `used` - unlike
/// [`dilate_transparent`]'s repeated full-image passes (bounded there by a
/// small `passes`, since a decal's transparent margin is only ever a
/// handful of texels), the unused area here is routinely most of the
/// canvas, so a single linear-time pass is what keeps this affordable.
///
/// Each triangle is rasterized to its own true shape, not its bounding box:
/// MSFS's own UV unwrap puts a handful of triangles right on a seam (one
/// edge of a wrapped cylindrical bezel, say), their three corners
/// legitimately far apart in UV space, and a bounding box over even a
/// handful of such triangles was enough to swallow most of a canvas as
/// "used" - tried and seen directly on FlyByWire's own pushbutton COMP
/// atlas, whose real content is under a tenth of the canvas by area but
/// whose triangles' bounding boxes covered close to seventy percent of it.
/// A seam triangle's actual rasterized footprint (a thin sliver along the
/// real seam) does not have that problem; only its box did.
///
/// A no-op when `used` is empty (nothing is known to dilate from) or lands
/// entirely off the canvas.
fn dilate_outside_uv(px: &mut [u8], w: u32, h: u32, used: &[[[f32; 2]; 3]]) {
    if w == 0 || h == 0 || used.is_empty() || px.len() < (w as usize) * (h as usize) * 3 {
        return;
    }
    let (w, h) = (w as usize, h as usize);
    let mut known = vec![false; w * h];
    for tri in used {
        rasterize_triangle(&mut known, w, h, tri);
    }
    let mut queue: std::collections::VecDeque<usize> = (0..w * h).filter(|&i| known[i]).collect();
    if queue.is_empty() {
        return;
    }
    let mut visited = known;
    while let Some(i) = queue.pop_front() {
        let (x, y) = (i % w, i / w);
        for (dx, dy) in [(-1i32, 0i32), (1, 0), (0, -1), (0, 1)] {
            let (nx, ny) = (x as i32 + dx, y as i32 + dy);
            if nx < 0 || ny < 0 || nx as usize >= w || ny as usize >= h {
                continue;
            }
            let ni = ny as usize * w + nx as usize;
            if !visited[ni] {
                visited[ni] = true;
                let (src, dst) = (i * 3, ni * 3);
                px.copy_within(src..src + 3, dst);
                queue.push_back(ni);
            }
        }
    }
}

/// Mark a triangle's own rasterized pixels `true` in a `w`x`h` boolean
/// canvas: its UV corners (0..1, wrapped modulo 1 first - MSFS materials
/// can tile a UV past 1 via `ASOBO_material_UV_options`, and a coordinate
/// past 1 is not "off the texture", it is back around to where the tiled
/// sampler would actually read) scaled to pixels, then every pixel centre
/// inside that triangle (an edge-function test, backwards-winding safe:
/// only the sign pattern has to agree, not which sign). A degenerate
/// (zero-area, or too thin to cover any pixel centre) triangle still marks
/// its own bounding box, on the same reasoning `uv_triangles` gives for
/// preferring a mesh's real shape over a box: a sliver is real content too,
/// and a handful of degenerate triangles cannot swallow the canvas the way
/// a handful of ordinary ones' boxes did.
fn rasterize_triangle(known: &mut [bool], w: usize, h: usize, tri: &[[f32; 2]; 3]) {
    let px: Vec<[f32; 2]> = tri.iter().map(|&[u, v]| [u.rem_euclid(1.0) * w as f32, v.rem_euclid(1.0) * h as f32]).collect();
    let (x0, x1) = bounds(px.iter().map(|p| p[0]), w);
    let (y0, y1) = bounds(px.iter().map(|p| p[1]), h);
    let edge = |a: [f32; 2], b: [f32; 2], p: [f32; 2]| (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]);
    let area = edge(px[0], px[1], px[2]);
    let mut hit = false;
    if area.abs() > f32::EPSILON {
        for y in y0..y1 {
            for x in x0..x1 {
                let p = [x as f32 + 0.5, y as f32 + 0.5];
                let (e0, e1, e2) = (edge(px[0], px[1], p), edge(px[1], px[2], p), edge(px[2], px[0], p));
                if (e0 >= 0.0 && e1 >= 0.0 && e2 >= 0.0) || (e0 <= 0.0 && e1 <= 0.0 && e2 <= 0.0) {
                    known[y * w + x] = true;
                    hit = true;
                }
            }
        }
    }
    if !hit {
        for y in y0..y1 {
            known[y * w + x0..y * w + x1].fill(true);
        }
    }
}

/// The inclusive-exclusive pixel range `[lo, hi)` a set of coordinates
/// spans, clamped into `0..len` and always at least one pixel wide (even a
/// single point, or a range that lands entirely past the edge, still
/// covers the nearest pixel).
fn bounds(coords: impl Iterator<Item = f32>, len: usize) -> (usize, usize) {
    let (mut lo, mut hi) = (f32::MAX, f32::MIN);
    for c in coords {
        lo = lo.min(c);
        hi = hi.max(c);
    }
    let lo = (lo.floor() as isize).clamp(0, len as isize - 1) as usize;
    let hi = ((hi.ceil() as isize).clamp(1, len as isize) as usize).max(lo + 1);
    (lo, hi)
}

/// An X-Plane normal map in its older NORMAL_METALNESS layout, from an MSFS
/// normal map and, when there is one, its occlusion/roughness/metalness
/// texture: red and green the normal (green inverted, see [`normal_png`]),
/// blue the metalness and alpha the gloss (see [`gloss`]). Metalness is
/// scaled by gloss, as in [`material_png`].
/// X-Plane wants normal maps uncompressed, as PNG.
pub fn normal_metal_png(normal: &[u8], comp: Option<&[u8]>, max_side: u32) -> Result<Converted, TextureError> {
    let (w, h, n) = decode_within(normal, max_side)?;
    let comp = match comp {
        Some(c) => {
            let (cw, ch, c) = decode_within(c, max_side)?;
            Some(shrink(c, cw, ch, max_side, Some((w, h))).2)
        }
        None => None,
    };
    let mut out = vec![0u8; (w * h * 4) as usize];
    for (i, px) in out.chunks_exact_mut(4).enumerate() {
        let (r, g) = (n[i * 4], n[i * 4 + 1]);
        let (m, smooth) = comp.as_ref().map_or((0, 128), |c| {
            let smooth = gloss(c[i * 4 + 1]);
            (metal(c[i * 4 + 2], smooth), smooth)
        });
        px.copy_from_slice(&[r, 255 - g, m, smooth]);
    }
    let mut bytes = Vec::new();
    let encoder = image::codecs::png::PngEncoder::new(&mut bytes);
    image::ImageEncoder::write_image(encoder, &out, w, h, image::ExtendedColorType::Rgba8)
        .map_err(|e| TextureError::Png(e.to_string()))?;
    Ok(Converted {
        bytes,
        extension: "png",
        // Uncompressed, four bytes a pixel, plus mips.
        vram_bytes: w as usize * h as usize * 4 * 4 / 3,
    })
}

/// Colour-dilate (edge-pad) an RGBA8 image's transparent texels: iteratively
/// fill each texel whose alpha is below `alpha_threshold` with the average
/// RGB of its already-opaque 4-connected neighbours, leaving alpha
/// untouched. Run before mips are generated (here, or by whatever GPU or
/// encoder builds them downstream).
///
/// Panel lettering, placard and legend decals are cut from a mostly empty
/// texture: the RGB under their fully transparent texels is black (or
/// whatever the source happened to hold there), never painted. X-Plane's
/// cockpit draws these decals alpha-tested (see `DECAL_ALPHA_TEST` in
/// `msfs2xp-aircraft`), not blended, so a texel that fails the test should
/// never show at all; but mipmapping and bilinear filtering both average
/// neighbouring texels' RGB together regardless of alpha, so that black
/// bleeds into the opaque edge texels next to it, most visibly once a mip
/// has shrunk the letters to a few texels wide and every one of them sits
/// next to transparent black. The result is a dark fringe around cockpit
/// lettering, worst at a distance where smaller mips are sampled. Filling
/// the transparent RGB with nearby opaque colour first means there is
/// nothing dark left to bleed in.
///
/// `passes` bounds how many texels of transparent margin get filled; a
/// decal's margin is normally a handful of texels; a few dozen passes
/// comfortably outruns it while keeping the cost bounded on a texture whose
/// transparent area is mostly empty background far from any lettering (that
/// background's own colour never matters, since it stays below the alpha
/// test wherever it is sampled from).
pub fn dilate_transparent(rgba: &mut [u8], w: u32, h: u32, alpha_threshold: u8, passes: u32) {
    if w == 0 || h == 0 || rgba.len() < (w as usize) * (h as usize) * 4 {
        return;
    }
    let (w, h) = (w as usize, h as usize);
    let mut known: Vec<bool> = rgba.chunks_exact(4).map(|p| p[3] >= alpha_threshold).collect();
    for _ in 0..passes {
        if known.iter().all(|&k| k) {
            break;
        }
        let prev_known = known.clone();
        let prev_rgba = rgba.to_vec();
        let mut changed = false;
        for y in 0..h {
            for x in 0..w {
                let i = y * w + x;
                if prev_known[i] {
                    continue;
                }
                let mut sum = [0u32; 3];
                let mut n = 0u32;
                for (dx, dy) in [(-1i32, 0i32), (1, 0), (0, -1), (0, 1)] {
                    let (nx, ny) = (x as i32 + dx, y as i32 + dy);
                    if nx < 0 || ny < 0 || nx as usize >= w || ny as usize >= h {
                        continue;
                    }
                    let ni = ny as usize * w + nx as usize;
                    if prev_known[ni] {
                        for (k, s) in sum.iter_mut().enumerate() {
                            *s += prev_rgba[ni * 4 + k] as u32;
                        }
                        n += 1;
                    }
                }
                if n > 0 {
                    for (k, s) in sum.iter().enumerate() {
                        rgba[i * 4 + k] = (*s / n) as u8;
                    }
                    known[i] = true;
                    changed = true;
                }
            }
        }
        if !changed {
            break;
        }
    }
}

/// Rescale a mip level's alpha channel so the fraction of its texels passing
/// `cutoff` (its coverage) matches `target_coverage`, by binary-searching a
/// multiplicative scale on every alpha value. Colour is untouched.
///
/// A box-filtered mip halves a decal's opaque lettering into texels mostly
/// surrounded by transparent neighbours: a single opaque texel with three
/// transparent 4-connected neighbours comes out at a quarter of its alpha
/// (`half` in `msfs2xp-aircraft` averages 2x2 blocks). Against a 0.90
/// alpha-test cutoff that thin stroke fails and the letter drops a texel at
/// that mip, thinner and fainter at each level down until it vanishes well
/// before the texture itself would need to. Rescaling each mip's alpha to
/// keep the same fraction of texels above the cutoff as the level above kept
/// the lettering's outline intact at every size X-Plane draws it.
pub fn preserve_alpha_coverage(alpha: &mut [u8], cutoff: u8, target_coverage: f32) {
    preserve_alpha_coverage_under(alpha, cutoff, target_coverage, None);
}

/// What fraction of the full-resolution art under each texel of a mip level
/// actually passed the alpha test.
///
/// [`preserve_alpha_coverage`] restores a mip's lost coverage by scaling the
/// whole level's alpha until the fraction of texels above the cutoff matches
/// the full-resolution figure. That is the standard alpha-coverage mip
/// algorithm, and it assumes what a single texture usually satisfies: that
/// the art is spread over the sheet, so a texel measured above the cutoff
/// stands for a place that really was covered. A shared cockpit decal atlas
/// breaks the assumption. It is one 4096-square sheet holding hundreds of
/// small lettering patches with most of its area empty, so by the time box
/// filtering has thinned the strokes the alpha histogram is packed just
/// under the cutoff and a scale of barely 2.6 sweeps a fifth of the entire
/// sheet across it -- including the empty space between glyphs, whose RGB
/// [`dilate_transparent`] has already flooded with the nearest opaque
/// colour. That is the flat white rectangle over the placards.
///
/// Measured on the A380's own atlas, before this: plain box filtering leaves
/// 1.37% of the 64-square level above the 230 cutoff and the shipped file
/// stores 21.26%; at the 16-square level plain filtering leaves nothing at
/// all and the file stores 25.78%.
///
/// A per-texel *ceiling* does not catch this -- the highest alpha under a
/// 64-square footprint of a lettering atlas is 255 almost everywhere, so it
/// never binds. The quantity that does is the footprint's covered fraction,
/// carried down by the same box filter the image itself uses. A texel over
/// nothing may then never be promoted however far the search scales, while a
/// texel over a genuine block of lettering still may. Where the fraction
/// sits below the threshold the lettering fades instead, which is what an
/// alpha-tested atlas can honestly show at that size: `GLOBAL_no_blend`
/// leaves no way to draw a stroke a fraction of a texel wide, so the choice
/// is between fading it and painting its whole texel.
#[derive(Clone, Debug)]
pub struct CoverageFootprint {
    /// 255 = the whole footprint passed the alpha test, 0 = none of it.
    frac: Vec<u8>,
    w: usize,
    h: usize,
}

/// The covered fraction a texel needs before coverage restoration may push
/// it over the alpha test.
///
/// Set to the alpha cutoff itself, which makes the correction close to a
/// no-op, and deliberately so. A half threshold still promoted the
/// part-covered texels that ring every glyph: box filtering leaves those
/// between roughly half and nine tenths covered, so they sit below the
/// cutoff but above the gate. Promoting them thickens every stroke by a
/// texel, and at the mip a cockpit button legend is really sampled at --
/// around 250 atlas texels across some 30 screen pixels, so mip 2 or 3 --
/// that merges a cluster of glyphs into one filled plate. Those are the
/// white boxes on the placards.
///
/// Measured on the real atlas at the 512-square level: plain box filtering
/// leaves 11.09% of it above the cutoff, and restoration stored 20.59%.
/// Nearly all of that difference is glyph-edge texels.
///
/// The cost is the case the correction was written for: a stroke thinner
/// than a texel now fades with distance instead of being held visible. That
/// is the honest thing a hard alpha test can do with it, and it is plainly
/// better than drawing a filled rectangle where the lettering was.
pub const MIN_COVERED_FRACTION: u8 = 230;

impl CoverageFootprint {
    /// At full resolution a texel is covered or it is not.
    pub fn new(alpha: &[u8], w: usize, h: usize, cutoff: u8) -> Self {
        debug_assert_eq!(alpha.len(), w * h);
        Self { frac: alpha.iter().map(|&a| if a >= cutoff { 255 } else { 0 }).collect(), w, h }
    }

    /// Halve by the same 2x2 box average, with the same edge clamping, that
    /// `msfs2xp-aircraft`'s `half` filters the image with, so the two chains
    /// stay texel-for-texel aligned.
    pub fn half(&self) -> Self {
        let (w, h) = ((self.w / 2).max(1), (self.h / 2).max(1));
        let mut frac = vec![0u8; w * h];
        for y in 0..h {
            for x in 0..w {
                let mut acc = 0u32;
                for (dx, dy) in [(0, 0), (1, 0), (0, 1), (1, 1)] {
                    let sx = (2 * x + dx).min(self.w - 1);
                    let sy = (2 * y + dy).min(self.h - 1);
                    acc += self.frac[sy * self.w + sx] as u32;
                }
                frac[y * w + x] = ((acc + 2) / 4) as u8;
            }
        }
        Self { frac, w, h }
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.frac
    }
}

/// [`preserve_alpha_coverage`] with an optional per-texel
/// [`CoverageFootprint`] deciding which texels restoration may promote over
/// the cutoff at all. `None` is the ungated original.
pub fn preserve_alpha_coverage_under(alpha: &mut [u8], cutoff: u8, target_coverage: f32, footprint: Option<&[u8]>) {
    if alpha.is_empty() || !(0.0..=1.0).contains(&target_coverage) {
        return;
    }
    let footprint = footprint.filter(|c| c.len() == alpha.len());
    // A texel whose footprint was mostly empty keeps whatever the box filter
    // left it; scaling may thicken what is there, never promote it over the
    // alpha test.
    let scaled = |i: usize, a: u8, scale: f32| -> u8 {
        let v = (a as f32 * scale).round().clamp(0.0, 255.0) as u8;
        match footprint {
            Some(f) if f[i] < MIN_COVERED_FRACTION => a,
            _ => v,
        }
    };
    let coverage_at = |scale: f32| -> f32 {
        alpha.iter().enumerate().filter(|&(i, &a)| scaled(i, a, scale) >= cutoff).count() as f32 / alpha.len() as f32
    };
    // Already there (or coverage is naturally higher, e.g. a mostly-opaque
    // background): scaling up would only blow highlights out further.
    let natural = coverage_at(1.0);
    if natural >= target_coverage {
        return;
    }
    // How much coverage this level is allowed to manufacture.
    //
    // The target is the full-resolution figure, and chasing it down the
    // whole chain is what puts a white box around cockpit lettering. A fifth
    // of the A380's shared decal atlas really is covered at full size, but by
    // the 64-square level box filtering leaves only 1.37% of it above the
    // cutoff, because the strokes are far thinner than a texel there. Scaling
    // until 21% passes again does not put the strokes back -- there is no way
    // to draw a fifth of a texel through a hard alpha test -- it promotes the
    // gaps *between* the strokes instead, and `dilate_transparent` has
    // already flooded those gaps with the colour of the lettering beside
    // them. A filled rectangle of ink colour around the text is exactly what
    // that looks like. At the 16-square level plain filtering leaves nothing
    // at all and the shipped file stores 25.78%: every covered texel there is
    // invented.
    //
    // So bound the target by what the level itself still holds. The
    // multiplicative term is the correction's real intent -- thicken a stroke
    // that survived, up to doubling the area it covers -- and the additive
    // floor keeps the case it was written for, a lone stroke box filtering
    // pushed just under the cutoff that would otherwise vanish a mip early.
    // Past that, coverage decays the way the filter says it should, so
    // lettering fades with distance instead of blocking up.
    const MAX_COVERAGE_GAIN: f32 = 2.0;
    const MIN_COVERAGE_FLOOR: f32 = 0.01;
    // The floor is never less than a single texel: on a 16-square level one
    // texel is already 0.4% of it, and a level that small must still be able
    // to keep the one stroke it has left.
    let floor = MIN_COVERAGE_FLOOR.max(1.0 / alpha.len() as f32);
    // Where a footprint says the art underneath really did cover most of a
    // texel, drawing it opaque is not manufacturing anything, so the cap
    // must not hold those back: a texture that was solid at full size stays
    // solid all the way down. The gate already forbids promoting anything
    // else, so this fraction is also the most coverage the level can reach.
    let genuinely_covered = footprint.map_or(0.0, |f| {
        f.iter().filter(|&&v| v >= MIN_COVERED_FRACTION).count() as f32 / f.len() as f32
    });
    let ceiling = target_coverage.min((natural * MAX_COVERAGE_GAIN + floor).max(genuinely_covered));
    if natural >= ceiling {
        return;
    }
    // Search for the largest scale whose *resulting* coverage still sits
    // under that ceiling, not the smallest that reaches the target. Aiming
    // at a target overshoots it wholesale: by the coarse levels the alpha
    // histogram is packed just under the cutoff, so the coverage a scale
    // produces jumps from a fraction of a percent to most of the sheet in
    // one step, and a search for "first scale at or above the target" lands
    // on the far side of that jump every time.
    let (mut lo, mut hi) = (1.0f32, 64.0f32);
    let reachable = coverage_at(hi) <= ceiling;
    if !reachable {
        // Even full scaling stays under the ceiling, so there is nothing to
        // search for: take it.
        for _ in 0..20 {
            let mid = (lo + hi) / 2.0;
            if coverage_at(mid) <= ceiling {
                lo = mid;
            } else {
                hi = mid;
            }
        }
    }
    let scale = if reachable { hi } else { lo };
    for (i, a) in alpha.iter_mut().enumerate() {
        *a = scaled(i, *a, scale);
    }
}

/// sRGB 0..255 to linear 0..1.
fn srgb_to_linear(c: u8) -> f32 {
    let c = c as f32 / 255.0;
    if c <= 0.040_45 { c / 12.92 } else { ((c + 0.055) / 1.055).powf(2.4) }
}

/// Linear 0..1 to sRGB 0..255.
fn linear_to_srgb(l: f32) -> u8 {
    let c = if l <= 0.003_130_8 { l * 12.92 } else { 1.055 * l.powf(1.0 / 2.4) - 0.055 };
    (c.clamp(0.0, 1.0) * 255.0).round() as u8
}

/// Multiply MSFS ambient occlusion (a COMP texture's red channel) into RGBA8
/// albedo pixels, in linear light so a mid-grey occlusion texel darkens the
/// light reaching the surface by half, not its sRGB code value. `comp` is
/// resampled to the albedo's own size (the two are often different
/// resolutions) by proportion, which lines it up through the UV set they
/// share: MSFS keeps one UV channel per material, the same one the albedo
/// and its COMP texture are both painted against.
///
/// A straight multiply, not tempered: MSFS's own occlusion value is only
/// meant to attenuate the ambient/indirect term, but a baked albedo has no
/// separate ambient channel to attenuate once it reaches X-Plane, so this
/// multiplies it into everything the surface reflects, direct light
/// included. That over-darkens a surface lit dead-on by a strong direct
/// light MSFS would have kept bright - an accepted approximation for
/// converting between two engines with different lighting models, the same
/// kind already made wherever this pipeline bakes one engine's term into a
/// texture the other reads as flat diffuse. Tempering the multiply (a
/// gamma curve, a floor) would under-darken every occluded contact shadow
/// this exists to restore, in exchange for a guess at how strong the direct
/// light will be at render time, which this function has no way to know.
pub fn bake_occlusion(rgba: &mut [u8], w: u32, h: u32, comp: &[u8], cw: u32, ch: u32) {
    if w == 0 || h == 0 || cw == 0 || ch == 0 {
        return;
    }
    let to_lin = srgb_to_linear;
    let to_srgb = linear_to_srgb;
    for y in 0..h {
        let cy = (u64::from(y) * u64::from(ch) / u64::from(h)).min(u64::from(ch) - 1) as u32;
        for x in 0..w {
            let cx = (u64::from(x) * u64::from(cw) / u64::from(w)).min(u64::from(cw) - 1) as u32;
            let ao = comp[((cy * cw + cx) * 4) as usize] as f32 / 255.0;
            let i = ((y * w + x) * 4) as usize;
            for k in 0..3 {
                rgba[i + k] = to_srgb(to_lin(rgba[i + k]) * ao);
            }
        }
    }
}

/// The next mip level down: each 2x2 block averaged in linear light (the
/// same box filter `half` in `msfs2xp-aircraft`'s own DXT encoder uses for
/// its plain and decal texture paths, mirrored here so a baked albedo's own
/// mip chain is built the same way). Averaging sRGB code values directly
/// greys thin painted detail out a level early; a perceptual (Lanczos)
/// filter rings around it instead, which shimmers under FXAA/TAA.
fn half_in_linear_light(rgba: &[u8], w: u32, h: u32) -> (u32, u32, Vec<u8>) {
    let (nw, nh) = ((w / 2).max(1), (h / 2).max(1));
    let mut out = vec![0u8; (nw * nh * 4) as usize];
    for y in 0..nh {
        for x in 0..nw {
            let mut lin = [0f32; 3];
            let mut a = 0u32;
            for (dx, dy) in [(0u32, 0u32), (1, 0), (0, 1), (1, 1)] {
                let sx = (2 * x + dx).min(w - 1);
                let sy = (2 * y + dy).min(h - 1);
                let i = ((sy * w + sx) * 4) as usize;
                for (k, l) in lin.iter_mut().enumerate() {
                    *l += srgb_to_linear(rgba[i + k]);
                }
                a += rgba[i + 3] as u32;
            }
            let o = ((y * nw + x) * 4) as usize;
            for k in 0..3 {
                out[o + k] = linear_to_srgb(lin[k] / 4.0);
            }
            out[o + 3] = (a / 4) as u8;
        }
    }
    (nw, nh, out)
}

/// How much unsharp [`sharpen_mip_rgba`] should apply to a mip built by
/// [`half_in_linear_light`], keyed to how many times that box filter has
/// run so far (1 the first time, counting up): 0 at level 1, where the box
/// output is already close to (sometimes past) a direct Lanczos downsample
/// of the source and a flat correction overshoots it; ramping to 0.18 by
/// level 3, where repeated box filtering compounds into a real softness gap
/// against Lanczos. This is the same schedule `msfs2xp-aircraft`'s `main.rs`
/// uses for its own, separate copy of this same box filter (`half`,
/// `fixes/T03.md`) -- kept identical on purpose: [`half_in_linear_light`]'s
/// own doc says it mirrors `half()` "so a baked albedo's own mip chain is
/// built the same way", and a baked albedo compressed here should not
/// suddenly get a different sharpness profile than one that took the other
/// crate's `encode_dds` path. Capped at 0.18 for anything deeper, since
/// that is as far as either chain was measured and validated
/// (`E:/fbw-debug/fixes/W66.md`).
fn mip_sharpen_amount(level: u32) -> f32 {
    (0.09 * (level as f32 - 1.0)).clamp(0.0, 0.18)
}

/// A small unsharp mask in linear light, undoing part of the blur that
/// compounds through repeated [`half_in_linear_light`] calls: each mip
/// level is filtered from the previous level, not from the original, so by
/// level 3 the equivalent blur is much wider than a single box pass. The
/// box filter itself stays untouched ([`half_in_linear_light`]'s own doc:
/// a perceptual filter rang around painted lettering and shimmered under
/// FXAA/TAA) -- this only nudges the already-boxed result back toward a
/// Lanczos-sharp target, and only where `amount` > 0 (see
/// [`mip_sharpen_amount`]: zero at level 1, so the level closest to the eye
/// is untouched). Alpha is left exactly as `half_in_linear_light` produced
/// it; only RGB is sharpened.
fn sharpen_mip_rgba(rgba: &[u8], w: u32, h: u32, amount: f32) -> Vec<u8> {
    if amount <= 0.0 {
        return rgba.to_vec();
    }
    let (w, h) = (w as usize, h as usize);
    let mut centre = vec![0f32; w * h * 3];
    for (i, p) in rgba.chunks_exact(4).enumerate() {
        for k in 0..3 {
            centre[i * 3 + k] = srgb_to_linear(p[k]);
        }
    }
    // Two passes of a separable 1-2-1 blur (edges replicated, like
    // `half_in_linear_light`'s own sampling), wide enough to reach past the
    // compounding box blur without spreading past a glyph stroke onto the
    // placard around it.
    let blur_pass = |buf: &[f32]| -> Vec<f32> {
        let mut tmp = vec![0f32; w * h * 3];
        for y in 0..h {
            for x in 0..w {
                let xm = x.saturating_sub(1);
                let xp = (x + 1).min(w - 1);
                for k in 0..3 {
                    tmp[(y * w + x) * 3 + k] =
                        (buf[(y * w + xm) * 3 + k] + 2.0 * buf[(y * w + x) * 3 + k] + buf[(y * w + xp) * 3 + k]) / 4.0;
                }
            }
        }
        let mut out = vec![0f32; w * h * 3];
        for y in 0..h {
            let ym = y.saturating_sub(1);
            let yp = (y + 1).min(h - 1);
            for x in 0..w {
                for k in 0..3 {
                    out[(y * w + x) * 3 + k] =
                        (tmp[(ym * w + x) * 3 + k] + 2.0 * tmp[(y * w + x) * 3 + k] + tmp[(yp * w + x) * 3 + k]) / 4.0;
                }
            }
        }
        out
    };
    let blurred = blur_pass(&blur_pass(&centre));
    let mut out = rgba.to_vec();
    for (i, px) in out.chunks_exact_mut(4).enumerate() {
        for k in 0..3 {
            px[k] = linear_to_srgb(centre[i * 3 + k] + amount * (centre[i * 3 + k] - blurred[i * 3 + k]));
        }
        // px[3] (alpha) is left exactly as `half_in_linear_light` wrote it.
    }
    out
}

/// The lowest alpha a texel may have and still count as opaque when a
/// cockpit texture's BC1-or-BC3 format is chosen ([`effectively_opaque`]).
/// FlyByWire's A380 ships its cockpit textures as BC7, whose encoder leaves
/// alpha noise of 251-254 on surfaces meant to be solid; demanding exactly
/// 255 put 51 of them in BC3, twice BC1's video memory (360 MB on the A380,
/// 2026-09-28) for an alpha channel nothing reads. A real cutout or decal
/// edge sits far below this.
pub const OPAQUE_ALPHA_MIN: u8 = 250;

/// Whether RGBA8 pixels are opaque enough to drop the alpha channel (BC1):
/// every texel at [`OPAQUE_ALPHA_MIN`] or above, bar at most one in 100,000
/// -- the A380's CEILING01 is solid but carries 7 stray texels at 191 in 16.7
/// million, far too few for any alpha test or blend to be drawing with.
pub fn effectively_opaque(rgba: &[u8]) -> bool {
    let total = rgba.len() / 4;
    let below = rgba.chunks_exact(4).filter(|p| p[3] < OPAQUE_ALPHA_MIN).count();
    below <= total / 100_000
}

/// Compress RGBA8 pixels (top row first) into a DXT DDS with a full mip
/// chain, each level built by [`half_in_linear_light`] and, from the second
/// level down, nudged back toward Lanczos sharpness by [`sharpen_mip_rgba`]
/// (see [`mip_sharpen_amount`]): BC1 (DXT1) when [`effectively_opaque`],
/// BC3 (DXT5) otherwise, matching the format choice `encode_dds` in
/// `msfs2xp-aircraft` makes for its interior texture paths.
/// `ClusterFit`, the same quality tier that pipeline uses for its own
/// interior (non-"best") textures: this is only ever called for cockpit
/// materials.
///
/// This is what lets [`convert_for_xplane_capped_with_ao`] write a baked
/// albedo at the aircraft's usual on-disk and video-memory size instead of
/// the uncompressed PNG baking otherwise requires (see its own doc for why
/// that PNG cost was the reason baking stayed off).
pub fn compress_rgba_to_dds(rgba: Vec<u8>, width: u32, height: u32) -> Result<Converted, TextureError> {
    if width == 0 || height == 0 {
        return Err(TextureError::Unsupported("cannot compress an empty image".into()));
    }
    let opaque = effectively_opaque(&rgba);
    let (format, tformat) = if opaque {
        (PixelFormat::Bc1, texpresso::Format::Bc1)
    } else {
        (PixelFormat::Bc3, texpresso::Format::Bc3)
    };
    let params = texpresso::Params::default();
    let mut mips = Vec::new();
    let (mut w, mut h, mut px) = (width, height, rgba);
    let mut level = 0u32;
    loop {
        let mut out = vec![0u8; tformat.compressed_size(w as usize, h as usize)];
        tformat.compress(&px, w as usize, h as usize, params, &mut out);
        mips.push(out);
        if w <= 4 || h <= 4 {
            break;
        }
        let (nw, nh, next) = half_in_linear_light(&px, w, h);
        level += 1;
        (w, h, px) = (nw, nh, sharpen_mip_rgba(&next, nw, nh, mip_sharpen_amount(level)));
    }
    let img = TextureImage {
        width,
        height,
        format,
        mips,
    };
    let bytes = to_xplane_dds(&img)?;
    let vram_bytes = bytes.len().saturating_sub(128);
    Ok(Converted {
        bytes,
        extension: "dds",
        vram_bytes,
    })
}

/// [`convert_for_xplane_capped`], with MSFS's ambient occlusion (a COMP
/// texture's red channel, see [`material_png`]) baked into the albedo first.
/// MSFS applies that occlusion as part of its ambient lighting term; X-Plane
/// 12's interior lighting carries no such term, so the contact shadow where a
/// button cap meets its housing (or any raised part meets the surface under
/// it) went missing and those parts read as pasted on rather than seated in
/// it. Without a `comp`, this is exactly [`convert_for_xplane_capped`].
///
/// A `comp` is baked in, then recompressed to DXT with a full mip chain (see
/// [`compress_rgba_to_dds`]) rather than left as the uncompressed PNG this
/// once had no alternative to - the same aircraft-sized budget every other
/// interior texture keeps.
///
/// CAUTION, found decoding real FlyByWire A380 panels rather than assuming
/// the formula's own reasoning generalised: [`bake_occlusion`]'s straight
/// multiply is correct for the narrow case it was written for - a small
/// part sitting almost entirely in its own housing's shadow (occlusion a
/// few percent) - but MSFS's COMP occlusion channel is not reserved for
/// that case. Decoding `A380_COCKPIT_MISC02`'s own COMP texture, restricted
/// to the UV triangles its 25 meshes actually sample (not the shared
/// atlas's unused padding), its mean occlusion is 0.305 and baking it
/// straight darkens that panel's mean luminance by 57%; even remapping low
/// occlusion so only texels below a strict 0.03 cutoff darken at all (well
/// under the housing-shadow case's own value) still leaves a 27.6% mean
/// drop, because a large share of this busy, many-small-parts panel's real,
/// on-screen content - not just the shadowed fixtures - genuinely reads as
/// heavily occluded in MSFS's own data. There is no single per-texel curve
/// that fixes the fixtures without darkening the rest of the panel by far
/// more than "a couple of percent": that would need scoping the bake to the
/// specific meshes it is meant for (a UV mask, the way
/// [`dilate_outside_uv`] already scopes the material/gloss map's fix to
/// what a mesh actually draws), not a blanket per-material formula. Calling
/// this on a whole cockpit material's albedo as things stand risks trading
/// one narrow defect (a handful of white fixture meshes) for a much larger
/// one (the whole panel reading grimy); `msfs2xp-aircraft`'s own
/// `BAKE_OCCLUSION` is left off for exactly this reason, and this function
/// is exercised by this module's own tests but not yet wired into that
/// pipeline's default path.
/// Multiply a material's glTF `baseColorFactor` into RGBA8 albedo pixels.
///
/// glTF defines the factor as a multiplier *on* the base colour texture, not
/// as a fallback for materials that have none, and it is specified in linear
/// light -- so a factor of 0.8 halves nothing, it scales the light the
/// surface reflects to four fifths. `tint` is that factor sRGB-encoded, so
/// it can key a texture variant; it is taken back to linear here.
///
/// MSFS leans on this heavily. On the A380, 34 of the 35 cockpit materials
/// that carry both a texture and a factor use it for something visible:
/// `DECAL_BLACK_NOEMIS` is [0, 0, 0, 1], which is how a black placard is
/// drawn from the same white lettering stencil that the white placards use,
/// and about twenty interior surfaces sit at [0.8, 0.8, 0.8]. Dropping the
/// factor renders every one of those black decals as the raw white stencil.
///
/// X-Plane's OBJ8 has no per-draw albedo tint to carry this at render time
/// (the same reason a material's opacity is baked into a faded copy of its
/// texture), so it has to go into the texels.
pub fn tint_rgba(rgba: &mut [u8], tint: [u8; 3]) {
    if tint == [255, 255, 255] {
        return;
    }
    let f = [srgb_to_linear(tint[0]), srgb_to_linear(tint[1]), srgb_to_linear(tint[2])];
    for px in rgba.chunks_exact_mut(4) {
        for k in 0..3 {
            px[k] = linear_to_srgb(srgb_to_linear(px[k]) * f[k]);
        }
    }
}

pub fn convert_for_xplane_capped_with_ao(
    data: &[u8],
    comp: Option<&[u8]>,
    max_side: u32,
    tint: Option<[u8; 3]>,
) -> Result<Converted, TextureError> {
    let Some(comp) = comp else {
        return match tint {
            Some(t) => {
                let (w, h, mut rgba) = decode_within(data, max_side)?;
                tint_rgba(&mut rgba, t);
                compress_rgba_to_dds(rgba, w, h)
            }
            None => convert_for_xplane_capped(data, max_side),
        };
    };
    let (w, h, mut rgba) = decode_within(data, max_side)?;
    let (cw, ch, comp_px) = decode_within(comp, max_side)?;
    bake_occlusion(&mut rgba, w, h, &comp_px, cw, ch);
    if let Some(t) = tint {
        tint_rgba(&mut rgba, t);
    }
    compress_rgba_to_dds(rgba, w, h)
}

/// Convert any MSFS texture, keeping its longer side at most `max_side`.
/// Block-compressed textures drop their largest mip levels, which costs
/// nothing beyond the lower resolution; everything else is resized.
pub fn convert_for_xplane_capped(data: &[u8], max_side: u32) -> Result<Converted, TextureError> {
    if detect(data) == SourceFormat::Png {
        // Size from the IHDR chunk; a PNG that fits, or whose size cannot be
        // read, passes through untouched.
        let dims = (data.len() >= 24).then(|| {
            (
                u32::from_be_bytes([data[16], data[17], data[18], data[19]]),
                u32::from_be_bytes([data[20], data[21], data[22], data[23]]),
            )
        });
        let (w, h) = match dims {
            Some((w, h)) if w.max(h) > max_side => (w, h),
            _ => {
                return Ok(Converted {
                    bytes: data.to_vec(),
                    extension: "png",
                    vram_bytes: dims.map_or(0, |(w, h)| w as usize * h as usize * 4 / 3),
                })
            }
        };
        let img = image::load_from_memory_with_format(data, image::ImageFormat::Png)
            .map_err(|e| TextureError::Png(e.to_string()))?
            .to_rgba8();
        return png_within(img.into_raw(), w, h, max_side);
    }
    let img = load(data)?;
    if writes_dds(&img) {
        let small = from_level(&img, first_level_within(&img, max_side, true));
        let bytes = to_xplane_dds(&small)?;
        let vram_bytes = bytes.len().saturating_sub(128);
        return Ok(Converted {
            bytes,
            extension: "dds",
            vram_bytes,
        });
    }
    let small = from_level(&img, first_level_within(&img, max_side, false));
    let rgba = decode_rgba8(&small, 0)?;
    png_within(rgba, small.width, small.height, max_side)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Deterministic pseudo-random bytes.
    fn noise(n: usize, seed: u32) -> Vec<u8> {
        let mut x = seed;
        (0..n)
            .map(|_| {
                x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                (x >> 24) as u8
            })
            .collect()
    }

    fn ktx2(vk: u32, w: u32, h: u32, levels: &[Vec<u8>], scheme: u32) -> Vec<u8> {
        let mut out = KTX2_ID.to_vec();
        for v in [vk, 1, w, h, 0, 0, 1, levels.len() as u32, scheme] {
            out.extend_from_slice(&v.to_le_bytes());
        }
        out.resize(80, 0);
        let mut offset = 80 + levels.len() * 24;
        for l in levels {
            out.extend_from_slice(&(offset as u64).to_le_bytes());
            out.extend_from_slice(&(l.len() as u64).to_le_bytes());
            out.extend_from_slice(&(l.len() as u64).to_le_bytes());
            offset += l.len();
        }
        for l in levels {
            out.extend_from_slice(l);
        }
        out
    }

    #[test]
    fn signed_bc5_decodes_to_normal_map_values() {
        // Red: endpoints +127 and -127, every pixel index 0, so +1 (255).
        // Green: both endpoints 0, so 0 (the middle, 128).
        let block = [0x7F, 0x81, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        let img = TextureImage {
            width: 4,
            height: 4,
            format: PixelFormat::Bc5s,
            mips: vec![block.to_vec()],
        };
        let px = decode_rgba8(&img, 0).unwrap();
        assert_eq!(&px[0..4], &[255, 128, 0, 255]);
        assert!(px.chunks(4).all(|p| p == [255, 128, 0, 255]));
        let unsigned = TextureImage {
            format: PixelFormat::Bc5,
            ..img
        };
        assert_eq!(&decode_rgba8(&unsigned, 0).unwrap()[0..2], &[127, 0]);
    }

    #[test]
    fn laminar_style_normal_and_material_maps() {
        let normal = ktx2(37, 4, 4, &[[255u8, 128, 0, 255].repeat(16)], 0);
        let n = image::load_from_memory(&normal_png(&normal, 4096, &[], FLAT_FLOOR).unwrap().bytes).unwrap().to_rgb8();
        assert_eq!(n.get_pixel(0, 0).0, [255, 127, 0], "red kept, green inverted, blue empty");
        let comp = ktx2(37, 4, 4, &[[10u8, 200, 100, 255].repeat(16)], 0);
        let m = image::load_from_memory(&material_png(Some(&comp), 4096, &[], FLAT_FLOOR).unwrap().bytes).unwrap().to_rgb8();
        // Roughness 200 of 255 gives gloss 9 (see below); metal 100 of 255
        // scaled by that gloss (100 * 9 / 255, rounded) is 4: a metal
        // reading on a rough surface does not glare as bare chrome.
        assert_eq!(m.get_pixel(0, 0).0, [4, 9, 0], "metal in red scaled by gloss, gloss in green");
        let plain = image::load_from_memory(&material_png(None, 4096, &[], FLAT_FLOOR).unwrap().bytes).unwrap().to_rgb8();
        assert_eq!(plain.get_pixel(0, 0).0, [0, 55, 0], "no COMP: not metal, gloss of roughness 0.5");
    }

    /// A sparse COMP atlas: real content (roughness 200, gloss 9) in the top
    /// row, the rest left at MSFS's own default fill (roughness 0, gloss
    /// 255 - mirror smooth) the way an unused corner of a shared atlas is,
    /// per FlyByWire's own pushbutton and overhead-knob COMP textures.
    /// Without `used`, X-Plane's own mip chain averages that padding into
    /// the real content at any mip coarse enough to span both, blowing a
    /// small, distant part out to a flat white rectangle.
    #[test]
    fn material_png_dilates_padding_outside_the_used_uv_rectangle() {
        let mut rows = vec![[10u8, 200, 100, 255].repeat(4)]; // row 0: real content
        rows.extend(std::iter::repeat_n([10u8, 0, 0, 255].repeat(4), 3)); // rows 1..4: unused padding
        let comp = ktx2(37, 4, 4, &[rows.concat()], 0);

        let undilated = image::load_from_memory(&material_png(Some(&comp), 4096, &[], FLAT_FLOOR).unwrap().bytes).unwrap().to_rgb8();
        assert_eq!(undilated.get_pixel(0, 3).0, [0, 255, 0], "without `used`, padding keeps its own gloss (255): the bug");

        // Only row 0 (v in [0, 0.25)) is ever sampled: a quad as two triangles.
        let used = [[[0.0, 0.0], [1.0, 0.0], [1.0, 0.2]], [[0.0, 0.0], [1.0, 0.2], [0.0, 0.2]]];
        let dilated = image::load_from_memory(&material_png(Some(&comp), 4096, &used, FLAT_FLOOR).unwrap().bytes).unwrap().to_rgb8();
        assert_eq!(dilated.get_pixel(0, 0).0, [4, 9, 0], "the used row itself is untouched");
        assert_eq!(dilated.get_pixel(0, 3).0, [4, 9, 0], "the far padding row now reads the real content, not 255 gloss");
        assert_eq!(dilated.get_pixel(2, 2).0, [4, 9, 0], "so does the middle of the unused block");
    }

    /// Triangles built from raw MSFS vertex UVs are not always tidy 0..1
    /// coordinates: `ASOBO_material_UV_options` can tile a UV past 1, and a
    /// degenerate (zero-area or off-canvas) triangle should not panic (this
    /// landed live: converting FlyByWire's A380 hit `min > max` building the
    /// pixel range for a triangle corner that sat exactly on the last row).
    /// The reason `dilate_outside_uv` rasterizes a triangle's own shape
    /// rather than its bounding box: on FlyByWire's A380, a handful of
    /// triangles sit on a UV seam with corners legitimately far apart, and
    /// their bounding boxes alone covered most of a shared COMP atlas as
    /// "used" - leaving the fix a no-op the first two times this was tried
    /// (a box per `TextureKey` group, then a box per mesh). A seam
    /// triangle's actual rasterized footprint does not have that problem.
    #[test]
    fn a_thin_triangle_rasterizes_to_far_less_than_its_bounding_box() {
        let (w, h) = (64usize, 64usize);
        let mut known = vec![false; w * h];
        // A needle along the main diagonal: corners as far apart as a
        // wrap-around seam triangle's, but almost no actual area.
        rasterize_triangle(&mut known, w, h, &[[0.0, 0.0], [1.0, 1.0], [0.02, 0.0]]);
        let marked = known.iter().filter(|&&k| k).count();
        // Its bounding box is the whole canvas; the sliver itself is
        // nowhere near that.
        assert!(marked < (w * h) / 4, "a thin triangle marked {marked} of {} texels - too close to its bounding box", w * h);
    }

    #[test]
    fn material_png_does_not_panic_on_out_of_range_or_degenerate_triangles() {
        let comp = ktx2(37, 4, 4, &[[10u8, 200, 100, 255].repeat(16)], 0);
        for used in [
            vec![[[0.0, 1.0], [0.5, 1.0], [0.0, 1.0]]], // a corner exactly on the last row, zero area
            vec![[[2.5, -1.0], [3.0, -1.0], [3.0, -0.5]]], // tiled UVs, entirely outside 0..1
            vec![[[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]], // a single point
        ] {
            material_png(Some(&comp), 4096, &used, FLAT_FLOOR).unwrap();
        }
    }

    #[test]
    fn normal_maps_take_the_x_plane_channel_layout() {
        // One 4x4 level each: normal (R 255, G 128), and a COMP texture
        // (occlusion 10, roughness 200, metalness 100).
        let normal = ktx2(37, 4, 4, &[[255u8, 128, 0, 255].repeat(16)], 0);
        let comp = ktx2(37, 4, 4, &[[10u8, 200, 100, 255].repeat(16)], 0);
        let c = normal_metal_png(&normal, Some(&comp), 1024).unwrap();
        let img = image::load_from_memory_with_format(&c.bytes, image::ImageFormat::Png).unwrap().to_rgba8();
        // Roughness 200 of 255: gloss (1 - 0.784)^2.2 = 0.035, i.e. 9.
        // Metal 100 of 255 scaled by that gloss (100 * 9 / 255, rounded) is 4.
        assert_eq!(img.get_pixel(0, 0).0, [255, 127, 4, 9], "green inverted, metal in blue scaled by gloss, gloss in alpha");
        let bare = normal_metal_png(&normal, None, 1024).unwrap();
        let img = image::load_from_memory_with_format(&bare.bytes, image::ImageFormat::Png).unwrap().to_rgba8();
        assert_eq!(img.get_pixel(0, 0).0, [255, 127, 0, 128]);
    }

    #[test]
    fn occlusion_bakes_into_albedo_in_linear_light() {
        // Albedo (200, 150, 50) with COMP red 128 (about half occlusion):
        // halving the light reaching a surface is not halving its sRGB code.
        let mut rgba = vec![200u8, 150, 50, 255];
        let comp = [128u8, 0, 0, 255];
        bake_occlusion(&mut rgba, 1, 1, &comp, 1, 1);
        let lin = |c: u8| {
            let c = c as f32 / 255.0;
            if c <= 0.040_45 { c / 12.92 } else { ((c + 0.055) / 1.055).powf(2.4) }
        };
        let srgb = |l: f32| {
            let c = if l <= 0.003_130_8 { l * 12.92 } else { 1.055 * l.powf(1.0 / 2.4) - 0.055 };
            (c.clamp(0.0, 1.0) * 255.0).round() as u8
        };
        let expect = |c: u8| srgb(lin(c) * (128.0 / 255.0));
        assert_eq!(rgba, vec![expect(200), expect(150), expect(50), 255]);
        assert!(rgba[0] > 50, "half occlusion in linear light should not crush the surface to near-black: {rgba:?}");
    }

    #[test]
    fn ao_bake_resamples_comp_to_the_albedos_own_size() {
        // A 2x1 albedo against a 1x1 COMP (one occlusion value covering the
        // whole texture, a common size mismatch): both texels take it.
        let mut rgba = vec![255u8, 255, 255, 255, 255, 255, 255, 255];
        let comp = [64u8, 0, 0, 255];
        bake_occlusion(&mut rgba, 2, 1, &comp, 1, 1);
        assert_eq!(&rgba[0..4], &rgba[4..8], "both albedo texels see the same COMP texel");
        assert!(rgba[0] < 255, "occlusion darkened a white surface");
    }

    fn small_png(w: u32, h: u32, px: &[u8]) -> Vec<u8> {
        let mut out = Vec::new();
        let encoder = image::codecs::png::PngEncoder::new(&mut out);
        image::ImageEncoder::write_image(encoder, px, w, h, image::ExtendedColorType::Rgba8).unwrap();
        out
    }

    #[test]
    fn convert_with_ao_falls_back_to_the_plain_conversion_without_a_comp_texture() {
        let png = small_png(2, 2, &[200, 150, 50, 255].repeat(4));
        let plain = convert_for_xplane_capped(&png, 4096).unwrap();
        let no_comp = convert_for_xplane_capped_with_ao(&png, None, 4096, None).unwrap();
        assert_eq!(plain.bytes, no_comp.bytes, "without a comp texture this is convert_for_xplane_capped");
    }

    #[test]
    fn convert_with_ao_darkens_the_albedo_it_writes() {
        let png = small_png(4, 4, &[220u8, 220, 220, 255].repeat(16));
        let comp = small_png(4, 4, &[64u8, 0, 0, 255].repeat(16));
        let out = convert_for_xplane_capped_with_ao(&png, Some(&comp), 4096, None).unwrap();
        assert_eq!(out.extension, "dds", "baked albedo is compressed DXT, not an uncompressed PNG");
        let img = load_dds(&out.bytes).unwrap();
        let px = decode_rgba8(&img, 0).unwrap();
        assert!(px[0] < 220, "low occlusion darkened the albedo: {px:?}");
    }

    #[test]
    fn bc7_alpha_noise_on_a_solid_texture_still_compresses_as_bc1() {
        // FlyByWire's BC7 cockpit textures decode with alpha 251-254 on
        // surfaces meant to be solid; that is not an alpha channel.
        let noisy = vec![180u8, 90, 40, 252].repeat(64);
        let out = compress_rgba_to_dds(noisy, 8, 8).unwrap();
        assert_eq!(load_dds(&out.bytes).unwrap().format, PixelFormat::Bc1);
        // A real cutout keeps its alpha.
        let mut cut = vec![180u8, 90, 40, 255].repeat(64);
        cut[3] = 0;
        let out = compress_rgba_to_dds(cut, 8, 8).unwrap();
        assert_eq!(load_dds(&out.bytes).unwrap().format, PixelFormat::Bc3, "one transparent texel in 64 is a cutout, not noise");
    }

    #[test]
    fn compress_rgba_to_dds_round_trips_within_bc1_tolerance() {
        // A uniform 8x8 block: BC1's single-colour fit should reproduce it
        // almost exactly (still quantized to 5/6/5 bits, so allow a few
        // steps either way), the baseline this darkening-preservation test
        // below builds on.
        let px = vec![180u8, 90, 40, 255].repeat(64);
        let out = compress_rgba_to_dds(px, 8, 8).unwrap();
        assert_eq!(out.extension, "dds");
        let img = load_dds(&out.bytes).unwrap();
        assert_eq!(img.format, PixelFormat::Bc1, "opaque input compresses without an alpha channel");
        let back = decode_rgba8(&img, 0).unwrap();
        for (before, after) in [180u8, 90, 40].iter().zip(&back[0..3]) {
            assert!((*before as i32 - *after as i32).abs() <= 8, "before {before} after {after}");
        }
    }

    #[test]
    fn mip_sharpen_amount_is_zero_at_level_one_and_caps_at_level_three() {
        assert_eq!(mip_sharpen_amount(1), 0.0);
        assert!((mip_sharpen_amount(2) - 0.09).abs() < 1e-6);
        assert!((mip_sharpen_amount(3) - 0.18).abs() < 1e-6);
        // Deeper levels stay at the same cap, not keep climbing.
        assert_eq!(mip_sharpen_amount(4), 0.18);
        assert_eq!(mip_sharpen_amount(7), 0.18);
    }

    #[test]
    fn sharpen_mip_rgba_at_zero_amount_is_a_no_op() {
        let rgba: Vec<u8> = (0..16u32).flat_map(|i| [((i * 17) % 255) as u8, 128, 0, 200]).collect();
        let out = sharpen_mip_rgba(&rgba, 4, 4, 0.0);
        assert_eq!(out, rgba);
    }

    #[test]
    fn sharpen_mip_rgba_preserves_alpha_and_sharpens_an_edge() {
        // A flat dark half next to a flat bright half: unsharp should push
        // the dark side darker and the bright side brighter right at the
        // step (more local contrast there), without ever touching alpha.
        let (w, h) = (6u32, 6u32);
        let mut rgba = vec![0u8; (w * h * 4) as usize];
        for y in 0..h {
            for x in 0..w {
                let v = if x < 3 { 40u8 } else { 220u8 };
                let i = ((y * w + x) * 4) as usize;
                rgba[i..i + 4].copy_from_slice(&[v, v, v, 137]);
            }
        }
        let out = sharpen_mip_rgba(&rgba, w, h, 0.18);
        for chunk in out.chunks_exact(4) {
            assert_eq!(chunk[3], 137, "alpha must be untouched");
        }
        let at = |x: u32, y: u32| out[((y * w + x) * 4) as usize];
        // The edge texel (x=2, right next to the step) sees more of the
        // bright side in its own blurred average than the interior texel
        // (x=0) does, so unsharp pushes it further from that average, i.e.
        // darker, not lighter -- x=2 must come out no lighter than x=0.
        assert!(at(2, 3) <= at(0, 3), "the dark side should not get lighter right at the edge");
        assert!(at(3, 3) >= at(5, 3), "the light side should not get darker right at the edge");
    }

    /// The task this whole path exists for: baking occlusion in, then
    /// recompressing to DXT, still leaves the surface meaningfully darker
    /// than the unbaked original - BC1's 5/6/5 quantization does not erase
    /// the effect it was applied to preserve.
    #[test]
    fn dxt_recompression_preserves_occlusion_darkening() {
        let rgba = vec![255u8, 247, 216, 255].repeat(64); // 8x8, the floodlight fixture's own colour
        let comp = vec![8u8, 0, 0, 255].repeat(64); // occlusion 8/255, as decoded from the real fixture
        let mut baked = rgba.clone();
        bake_occlusion(&mut baked, 8, 8, &comp, 8, 8);
        let compressed = compress_rgba_to_dds(baked.clone(), 8, 8).unwrap();
        let img = load_dds(&compressed.bytes).unwrap();
        let back = decode_rgba8(&img, 0).unwrap();
        for k in 0..3 {
            // The recompressed value stays close (BC1 tolerance) to the
            // baked-but-uncompressed value...
            assert!((baked[k] as i32 - back[k] as i32).abs() <= 10, "channel {k}: baked {} back {}", baked[k], back[k]);
            // ...and both are far darker than the original, unbaked colour:
            // compression did not wash the darkening back out.
            assert!(rgba[k] as i32 - back[k] as i32 > 80, "channel {k}: original {} still-baked {}", rgba[k], back[k]);
        }
    }

    fn mirrored(px: &[u8], w: usize, h: usize) -> Vec<u8> {
        (0..h)
            .flat_map(|y| (0..w).rev().flat_map(move |x| (0..4).map(move |c| (y, x, c))))
            .map(|(y, x, c)| px[(y * w + x) * 4 + c])
            .collect()
    }

    #[test]
    fn block_mirroring_matches_a_pixel_mirror() {
        for (fmt, seed) in [(PixelFormat::Bc1, 7), (PixelFormat::Bc2, 8), (PixelFormat::Bc3, 9)] {
            let (w, h) = (16u32, 8u32);
            let img = TextureImage {
                width: w,
                height: h,
                format: fmt,
                mips: vec![noise(level_size(fmt, w, h), seed)],
            };
            let before = decode_rgba8(&img, 0).unwrap();
            let flipped = TextureImage {
                mips: vec![mirror_bc_level(fmt, &img.mips[0], w, h).unwrap()],
                ..img
            };
            let after = decode_rgba8(&flipped, 0).unwrap();
            assert_eq!(after, mirrored(&before, w as usize, h as usize), "{fmt:?}");
        }
    }

    #[test]
    fn converted_files_flip_both_ways() {
        let levels: Vec<Vec<u8>> = [(16, 4), (8, 5), (4, 6)]
            .iter()
            .map(|&(side, seed)| noise(level_size(PixelFormat::Bc3, side, side), seed))
            .collect();
        let dds = convert_for_xplane(&ktx2(137, 16, 16, &levels, 0)).unwrap().bytes;
        let px = |bytes: &[u8]| decode_rgba8(&load_dds(bytes).unwrap(), 0).unwrap();
        let before = px(&dds);
        let turned = transform_texture_file(&dds, true, true).unwrap();
        let back = transform_texture_file(&turned, true, true).unwrap();
        assert_eq!(px(&back), before, "turning twice gives the original back");
        let h = transform_texture_file(&dds, false, true).unwrap();
        assert_eq!(px(&h), mirrored(&before, 16, 16));
        assert_eq!(u32::from_le_bytes(h[28..32].try_into().unwrap()), 3, "all three levels kept");

        let img = image::RgbaImage::from_raw(2, 1, vec![1, 2, 3, 255, 9, 9, 9, 255]).unwrap();
        let mut png = Vec::new();
        img.write_to(&mut std::io::Cursor::new(&mut png), image::ImageFormat::Png).unwrap();
        let out = transform_texture_file(&png, false, true).unwrap();
        let back = image::load_from_memory(&out).unwrap().to_rgba8();
        assert_eq!(back.get_pixel(0, 0).0, [9, 9, 9, 255]);
    }

    #[test]
    fn capped_textures_drop_their_largest_levels() {
        let levels: Vec<Vec<u8>> = [(16, 1), (8, 2), (4, 3)]
            .iter()
            .map(|&(side, seed)| noise(level_size(PixelFormat::Bc1, side, side), seed))
            .collect();
        let data = ktx2(131, 16, 16, &levels, 0);
        let full = convert_for_xplane(&data).unwrap();
        let small = convert_for_xplane_capped(&data, 8).unwrap();
        assert_eq!((full.extension, small.extension), ("dds", "dds"));
        assert_eq!(output_extension(&data).unwrap(), "dds");
        let dims = |b: &[u8]| {
            let at = |o: usize| u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]);
            (at(16), at(12), at(28))
        };
        assert_eq!(dims(&full.bytes), (16, 16, 3));
        assert_eq!(dims(&small.bytes), (8, 8, 2), "the 16 px level is dropped");
        assert!(small.vram_bytes < full.vram_bytes);
    }

    #[test]
    fn capped_uncompressed_textures_are_resized() {
        let data = ktx2(37, 8, 4, &[noise(8 * 4 * 4, 5)], 0);
        let c = convert_for_xplane_capped(&data, 4).unwrap();
        assert_eq!(c.extension, "png");
        let img = image::load_from_memory_with_format(&c.bytes, image::ImageFormat::Png).unwrap();
        assert_eq!((img.width(), img.height()), (4, 2));
    }

    fn vflip(rgba: &[u8], w: usize, h: usize) -> Vec<u8> {
        (0..h)
            .rev()
            .flat_map(|y| rgba[y * w * 4..(y + 1) * w * 4].to_vec())
            .collect()
    }

    #[test]
    fn detects_containers() {
        assert_eq!(detect(&KTX2_ID), SourceFormat::Ktx2);
        assert_eq!(detect(b"DDS xxxx"), SourceFormat::Dds);
        assert_eq!(detect(&[0x89, b'P', b'N', b'G', 1]), SourceFormat::Png);
        assert_eq!(detect(b"nope"), SourceFormat::Unknown);
    }

    #[test]
    fn loads_a_bc7_ktx2_like_msfs_ships() {
        let data = ktx2(145, 8, 8, &[noise(64, 1), noise(16, 2)], 0);
        let img = load(&data).unwrap();
        assert_eq!((img.width, img.height, img.format), (8, 8, PixelFormat::Bc7));
        assert_eq!(img.mips.len(), 2);
        assert_eq!(img.mips[1], noise(16, 2));
    }

    #[test]
    fn rejects_supercompressed_and_unknown_formats() {
        let e = load(&ktx2(145, 4, 4, &[noise(16, 1)], 1)).unwrap_err();
        assert!(e.to_string().contains("BasisLZ"), "{e}");
        let e = load(&ktx2(999, 4, 4, &[noise(16, 1)], 0)).unwrap_err();
        assert!(e.to_string().contains("999"), "{e}");
    }

    #[test]
    fn truncated_files_error_instead_of_panicking() {
        let mut data = ktx2(133, 8, 8, &[noise(32, 1)], 0);
        data.truncate(data.len() - 5);
        assert!(load(&data).is_err());
        assert!(load(&data[..30]).is_err());
        assert!(load_dds(b"DDS \x7c\x00\x00\x00short").is_err());
    }

    #[test]
    fn block_flips_match_a_decoded_flip() {
        // For every format the DDS path uses, flipping the blocks must give the
        // same pixels as decoding and flipping the image.
        for (fmt, bb) in [(PixelFormat::Bc1, 8), (PixelFormat::Bc2, 16), (PixelFormat::Bc3, 16)] {
            let (w, h) = (8u32, 12u32);
            let data = noise(2 * 3 * bb, 7 + bb as u32);
            let img = TextureImage {
                width: w,
                height: h,
                format: fmt,
                mips: vec![data.clone()],
            };
            let flipped = TextureImage {
                mips: vec![flip_bc_level(fmt, &data, w, h).unwrap()],
                ..img.clone()
            };
            let expected = vflip(&decode_rgba8(&img, 0).unwrap(), w as usize, h as usize);
            assert_eq!(decode_rgba8(&flipped, 0).unwrap(), expected, "{fmt:?}");
        }
    }

    #[test]
    fn heights_that_are_not_whole_blocks_cannot_be_flipped() {
        assert!(flip_bc_level(PixelFormat::Bc1, &noise(8, 1), 4, 2).is_none());
        assert!(flip_bc_level(PixelFormat::Bc7, &noise(16, 1), 4, 4).is_none());
    }

    #[test]
    fn bc1_becomes_a_dds_that_reads_back() {
        let data = ktx2(133, 8, 8, &[noise(32, 3), noise(8, 4), noise(8, 5), noise(8, 6)], 0);
        let out = convert_for_xplane(&data).unwrap();
        assert_eq!(out.extension, "dds");
        let back = load_dds(&out.bytes).unwrap();
        assert_eq!((back.width, back.height, back.format), (8, 8, PixelFormat::Bc1));
        // Every level is kept, rows as in the source: X-Plane reads DDS top
        // row first.
        assert_eq!(back.mips.len(), 4);
        assert_eq!(back.mips[0], noise(32, 3));
    }

    #[test]
    fn bc7_becomes_a_png_of_the_right_size() {
        let data = ktx2(145, 8, 4, &[noise(32, 9)], 0);
        let out = convert_for_xplane(&data).unwrap();
        assert_eq!(out.extension, "png");
        let decoded = image::load_from_memory(&out.bytes).unwrap();
        assert_eq!((decoded.width(), decoded.height()), (8, 4));
    }

    #[test]
    fn png_passes_through() {
        let png = [0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A];
        assert_eq!(convert_for_xplane(&png).unwrap().bytes, png);
    }

    /// Convert every texture in `MSFS2XP_TEXTURES` and report the split.
    #[test]
    #[ignore]
    fn real_textures_convert() {
        let Ok(dir) = std::env::var("MSFS2XP_TEXTURES") else {
            return;
        };
        let (mut dds, mut png, mut failed) = (0, 0, 0);
        for entry in std::fs::read_dir(&dir).unwrap().filter_map(Result::ok) {
            let p = entry.path();
            if !p.to_string_lossy().to_ascii_lowercase().ends_with(".ktx2") {
                continue;
            }
            match convert_for_xplane(&std::fs::read(&p).unwrap()) {
                Ok(c) if c.extension == "dds" => dds += 1,
                Ok(_) => png += 1,
                Err(e) => {
                    failed += 1;
                    eprintln!("{}: {e}", p.display());
                }
            }
        }
        eprintln!("dds {dds}, png {png}, failed {failed}");
        assert_eq!(failed, 0);
    }

    #[test]
    fn dilation_fills_transparent_rgb_from_nearby_opaque_texels() {
        // A 5x1 row: one opaque red texel at x=0, everything else fully
        // transparent black (a decal's lettering edge against its empty
        // background).
        let mut rgba = vec![0u8; 5 * 4];
        rgba[0..4].copy_from_slice(&[200, 20, 20, 255]);
        dilate_transparent(&mut rgba, 5, 1, 128, 10);
        // Every texel took on the opaque texel's colour by the time the
        // fill reached it, one texel further per pass.
        for i in 0..5 {
            assert_eq!(&rgba[i * 4..i * 4 + 3], &[200, 20, 20], "texel {i} RGB");
        }
        // Alpha is untouched: only the originally opaque texel still passes
        // an alpha test.
        assert_eq!(rgba[3], 255);
        assert!(rgba[4..].chunks_exact(4).all(|p| p[3] == 0), "alpha untouched: {rgba:?}");
    }

    #[test]
    fn dilation_is_bounded_by_its_pass_count() {
        // Same row, but only one pass: the fill reaches one texel out.
        let mut rgba = vec![0u8; 5 * 4];
        rgba[0..4].copy_from_slice(&[200, 20, 20, 255]);
        dilate_transparent(&mut rgba, 5, 1, 128, 1);
        assert_eq!(&rgba[4..7], &[200, 20, 20], "texel 1 reached");
        assert_eq!(&rgba[8..11], &[0, 0, 0], "texel 2 not reached in one pass");
    }

    #[test]
    fn dilation_averages_more_than_one_opaque_neighbour() {
        // A 3x1 row: red at x=0, blue at x=2, transparent between them.
        let mut rgba = vec![0u8; 3 * 4];
        rgba[0..4].copy_from_slice(&[255, 0, 0, 255]);
        rgba[8..12].copy_from_slice(&[0, 0, 255, 255]);
        dilate_transparent(&mut rgba, 3, 1, 128, 5);
        assert_eq!(&rgba[4..7], &[127, 0, 127], "averaged from both neighbours: {:?}", &rgba[4..7]);
    }

    #[test]
    fn dilation_leaves_a_fully_opaque_image_alone() {
        let mut rgba: Vec<u8> = (0..16u8).flat_map(|i| [i, i, i, 255]).collect();
        let before = rgba.clone();
        dilate_transparent(&mut rgba, 4, 4, 128, 32);
        assert_eq!(rgba, before);
    }

    #[test]
    fn alpha_coverage_is_restored_after_a_mip_halves_thin_lettering() {
        // A single texel that was fully opaque one mip up, now a quarter
        // alpha (64 of 255) after a straight 2x2 box average pulled in
        // three fully transparent neighbours -- `half` in
        // `msfs2xp-aircraft` on a one-texel-wide decal stroke -- everything
        // else fully transparent. 64 fails a 0.90 (230 of 255) alpha-test
        // cutoff, so without correction the letter simply disappears here.
        let mut mip_alpha = vec![0u8; 100];
        mip_alpha[0] = 64;
        let full_res_coverage = 0.01; // one texel of 100, as the level above had.
        assert!(mip_alpha[0] < 230, "the crushed texel fails the cutoff before correction");
        preserve_alpha_coverage(&mut mip_alpha, 230, full_res_coverage);
        assert!(mip_alpha[0] >= 230, "scaled back up above the cutoff: {}", mip_alpha[0]);
        let coverage = mip_alpha.iter().filter(|&&a| a >= 230).count() as f32 / mip_alpha.len() as f32;
        assert!((coverage - full_res_coverage).abs() < 0.02, "coverage {coverage}, wanted {full_res_coverage}");
        // Texels that were already fully transparent stay that way: scaling
        // a zero alpha by anything is still zero.
        assert_eq!(mip_alpha[1], 0);
    }

    #[test]
    fn alpha_coverage_never_exceeds_255() {
        // Half the level is still above the cutoff, so doubling reaches the
        // whole of it, and the scaling saturates rather than wrapping.
        let mut mip_alpha = vec![255u8, 255, 120, 120];
        preserve_alpha_coverage(&mut mip_alpha, 200, 1.0);
        assert!(mip_alpha.iter().all(|&a| a >= 200), "every texel scaled up past the cutoff: {mip_alpha:?}");
    }

    /// Coverage may not be manufactured wholesale. A level box filtering has
    /// left with nothing above the cutoff gets the floor, not the
    /// full-resolution figure -- the difference between lettering that fades
    /// with distance and a filled box around it.
    #[test]
    fn coverage_restoration_is_capped_by_what_the_level_still_holds() {
        let mut alpha = vec![60u8; 400];
        alpha[0] = 200;
        preserve_alpha_coverage(&mut alpha, 230, 0.20);
        let covered = alpha.iter().filter(|&&a| a >= 230).count();
        assert!(covered > 0, "the strongest survivor is still rescued: {covered}");
        assert!(covered <= 400 / 50, "and no more than the 1% floor: {covered} of 400");
    }

    #[test]
    fn alpha_coverage_leaves_an_already_covered_mip_alone() {
        let mut mip_alpha = vec![255u8; 10];
        preserve_alpha_coverage(&mut mip_alpha, 200, 0.5);
        assert_eq!(mip_alpha, vec![255u8; 10], "already fully covered: no need to scale, let alone up");
    }

    /// The white-rectangle defect.
    ///
    /// A sparse decal atlas a few mips down: one lettering stroke that box
    /// filtering has thinned to alpha 64 over a quarter-covered footprint,
    /// and background smeared from a flat 0 up to a faint 8 with nothing
    /// under it at all. Aiming at the whole-sheet coverage figure, the
    /// search climbs until that background clears the cutoff and every one
    /// of those texels turns opaque; `dilate_transparent` has already filled
    /// their RGB with the nearest opaque colour, so on screen it is a solid
    /// near-white block over the placard.
    #[test]
    fn coverage_restoration_does_not_turn_faint_background_into_white_blocks() {
        let mut alpha = vec![8u8; 16];
        alpha[0] = 64;
        let mut footprint = vec![0u8; 16];
        footprint[0] = 245; // the stroke's own texel was essentially all ink
        // A whole-sheet figure far above anything this level still holds.
        preserve_alpha_coverage_under(&mut alpha, 230, 0.2, Some(&footprint));

        assert!(alpha[0] >= 230, "the real stroke is still rescued: {alpha:?}");
        assert!(
            alpha[1..].iter().all(|&a| a == 8),
            "and background with nothing under it is left exactly as the filter left it: {alpha:?}"
        );

        // The same stroke over a texel the art only partly covered is let
        // go instead. Claiming a texel claims its whole area for the ink,
        // and doing that to the part-covered texels that ring every glyph
        // thickens each stroke by a texel -- which at the mip a cockpit
        // button legend is actually sampled at merges the whole cluster of
        // glyphs into one filled plate. That is the white box on the
        // placards, measured: plain box filtering leaves 11.09% of the
        // 512-square level above the cutoff and restoration stored 20.59%.
        let mut alpha = vec![8u8; 16];
        alpha[0] = 64;
        footprint[0] = 180;
        preserve_alpha_coverage_under(&mut alpha, 230, 0.2, Some(&footprint));
        assert!(alpha.iter().all(|&a| a < 230), "a mostly-empty texel fades rather than blocking up: {alpha:?}");
    }

    /// The footprint halves by the same box average the image uses, and
    /// gates promotion on what was underneath rather than on how far the
    /// search wants to scale.
    #[test]
    fn covered_fraction_halves_by_area_and_gates_promotion() {
        // 4x2, so the halved level is 2x1 and each output texel clamps its
        // second row back onto the first.
        let full = vec![
            255, 255, 0, 0, //
            255, 0, 0, 255,
        ];
        let f = CoverageFootprint::new(&full, 4, 2, 230);
        assert_eq!(f.half().as_slice(), &[191, 64], "3 of 4 covered, then 1 of 4");

        let mut alpha = vec![200u8; 4];
        preserve_alpha_coverage_under(&mut alpha, 230, 1.0, Some(&[0, 0, 0, 0]));
        assert_eq!(alpha, vec![200u8; 4], "nothing underneath, so nothing over the cutoff");

        let mut alpha = vec![200u8; 4];
        preserve_alpha_coverage_under(&mut alpha, 230, 1.0, Some(&[255, 255, 255, 255]));
        assert!(alpha.iter().all(|&a| a >= 230), "fully covered underneath, so restoration applies: {alpha:?}");
    }
    /// The tint is a multiplier on the texture, applied in linear light, and
    /// it leaves alpha alone -- a decal's alpha test still sees the coverage
    /// the art had.
    #[test]
    fn a_base_colour_factor_multiplies_the_texture_in_linear_light() {
        // White: nothing to do, and nothing touched.
        let mut px = vec![200u8, 150, 100, 64];
        tint_rgba(&mut px, [255, 255, 255]);
        assert_eq!(px, vec![200, 150, 100, 64]);

        // Black, as FlyByWire's `DECAL_BLACK_NOEMIS` is: the white lettering
        // stencil becomes black lettering, alpha untouched.
        let mut px = vec![228u8, 238, 219, 255, 0, 0, 0, 0];
        tint_rgba(&mut px, [0, 0, 0]);
        assert_eq!(px, vec![0, 0, 0, 255, 0, 0, 0, 0], "black tint, alpha kept");

        // 0.8 in linear light is 231 in sRGB; a white texel under it must
        // come back to 0.8 linear, not to 80% of its sRGB code value.
        let eight_tenths = linear_to_srgb(0.8);
        let mut px = vec![255u8, 255, 255, 255];
        tint_rgba(&mut px, [eight_tenths; 3]);
        let got = srgb_to_linear(px[0]);
        assert!((got - 0.8).abs() < 0.01, "0.8 linear, got {got}");
        assert!(px[0] > 204, "and not the 204 a naive sRGB multiply would give: {}", px[0]);
    }

    #[test]
    fn a_flat_map_shrinks_and_a_detailed_one_does_not() {
        // A gentle ramp: every 2x2 block is nearly its own mean, so halving
        // costs almost nothing and the map should keep halving to the floor.
        let (w, h) = (1024u32, 1024u32);
        let ramp: Vec<u8> = (0..w * h).flat_map(|i| {
            let v = ((i % w) / 4) as u8;
            [v, v, 0]
        }).collect();
        let (_, rw, rh) = shrink_while_flat(ramp, w, h, NORMAL_FLAT_LIMIT, FLAT_FLOOR);
        assert_eq!((rw, rh), (FLAT_FLOOR, FLAT_FLOOR), "a flat ramp should shrink to the floor");

        // A one-texel checkerboard: every 2x2 block averages to grey and
        // each texel is 127 away from it, far past any limit.
        let check: Vec<u8> = (0..w * h).flat_map(|i| {
            let v = if (i % w + i / w) % 2 == 0 { 0u8 } else { 255 };
            [v, v, 0]
        }).collect();
        let (_, cw, ch) = shrink_while_flat(check, w, h, NORMAL_FLAT_LIMIT, FLAT_FLOOR);
        assert_eq!((cw, ch), (w, h), "per-texel detail must not be thrown away");
    }

    #[test]
    fn the_floor_is_never_crossed() {
        let px = vec![3u8; (FLAT_FLOOR as usize) * (FLAT_FLOOR as usize) * 3];
        let (_, w, h) = shrink_while_flat(px, FLAT_FLOOR, FLAT_FLOOR, MATERIAL_FLAT_LIMIT, FLAT_FLOOR);
        assert_eq!((w, h), (FLAT_FLOOR, FLAT_FLOOR));
    }

    /// `normal_png` gets the same padding-dilution fix `material_png` has
    /// had: an atlas whose real content is a small used rectangle, the rest
    /// left at MSFS's own default fill, must be judged by the used part
    /// alone once `used` is given.
    #[test]
    fn normal_png_dilates_padding_outside_the_used_uv_rectangle() {
        let mut rows = vec![[255u8, 128, 0, 255].repeat(4)]; // row 0: real content (flat normal, straight up)
        rows.extend(std::iter::repeat_n([0u8, 255, 0, 255].repeat(4), 3)); // rows 1..4: unused padding, a different value
        let normal = ktx2(37, 4, 4, &[rows.concat()], 0);

        let used = [[[0.0, 0.0], [1.0, 0.0], [1.0, 0.2]], [[0.0, 0.0], [1.0, 0.2], [0.0, 0.2]]];
        let dilated = image::load_from_memory(&normal_png(&normal, 4096, &used, FLAT_FLOOR).unwrap().bytes).unwrap().to_rgb8();
        assert_eq!(dilated.get_pixel(0, 0).0, [255, 127, 0], "the used row itself is untouched");
        assert_eq!(dilated.get_pixel(0, 3).0, [255, 127, 0], "far padding now reads the real content, not its own default fill");
    }

    /// A caller that already knows a map's real detail cannot be told apart
    /// from an ordinary flat map's own noise (see `msfs2xp-aircraft`'s
    /// `LETTERED_NORMAL_MAPS`) can raise `normal_png`'s `floor` to keep a
    /// guaranteed minimum size, bypassing the flatness heuristic outright.
    #[test]
    fn normal_png_floor_overrides_the_flatness_heuristic() {
        let (w, h) = (1024u32, 1024u32);
        // Perfectly flat: on its own this shrinks all the way to FLAT_FLOOR.
        let flat: Vec<u8> = [128u8, 128, 0, 255].repeat((w * h) as usize);
        let src = ktx2(37, w, h, &[flat], 0);
        let free = image::load_from_memory(&normal_png(&src, w, &[], FLAT_FLOOR).unwrap().bytes).unwrap();
        assert_eq!(free.width(), FLAT_FLOOR, "no override: shrinks to the default floor");

        let held = image::load_from_memory(&normal_png(&src, w, &[], 512).unwrap().bytes).unwrap();
        assert_eq!(held.width(), 512, "a raised floor is never crossed, even for a flat map");
    }

    /// Same override, for `material_png`: a caller that already knows a
    /// COMP map's real gloss/metal contrast cannot be told apart from an
    /// ordinary flat map's own noise by any statistic run over it (see
    /// `msfs2xp-aircraft`'s `LETTERED_MATERIAL_MAPS`, measured directly
    /// against FlyByWire's own GLARESHIELD/KEYBOARD/MIP03/MIP_KNOBS_02 COMP
    /// textures: E:/fbw-debug/fixes/W105.md) can raise `material_png`'s
    /// `floor` to keep a guaranteed minimum size.
    #[test]
    fn material_png_floor_overrides_the_flatness_heuristic() {
        let (w, h) = (1024u32, 1024u32);
        // Perfectly flat COMP (roughness/metalness constant everywhere): on
        // its own this shrinks all the way to FLAT_FLOOR.
        let flat: Vec<u8> = [10u8, 200, 100, 255].repeat((w * h) as usize);
        let comp = ktx2(37, w, h, &[flat], 0);
        let free = image::load_from_memory(&material_png(Some(&comp), w, &[], FLAT_FLOOR).unwrap().bytes).unwrap();
        assert_eq!(free.width(), FLAT_FLOOR, "no override: shrinks to the default floor");

        let held = image::load_from_memory(&material_png(Some(&comp), w, &[], 512).unwrap().bytes).unwrap();
        assert_eq!(held.width(), 512, "a raised floor is never crossed, even for a flat COMP map");
    }
}
