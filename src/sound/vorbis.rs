//! Wwise media (`.wem`, a RIFF WAVE) decoded to interleaved 16-bit PCM.
//!
//! - Format tag 0x0001 / 0xFFFE with 16 bits per sample: plain PCM, copied.
//! - Format tag 0xFFFF: Wwise Vorbis. Wwise strips a Vorbis stream down: no
//!   Ogg pages, no identification or comment header, a setup header whose
//!   codebooks are indices into a shared library and whose floors, residues,
//!   mappings and modes drop their fixed fields, and audio packets that drop
//!   the packet type and window flags. This rebuilds the standard Vorbis
//!   identification and setup headers and each audio packet exactly as
//!   ww2ogg does (github.com/hcs64/ww2ogg, BSD-3, src/wwriff.cpp
//!   `generate_ogg_header` and `generate_ogg`, src/codebook.cpp `rebuild`)
//!   and hands them to lewton's packet API; no Ogg container is built.
//!
//! The layout used is the one vgmstream calls WWVORBIS_V62
//! (github.com/vgmstream/vgmstream, src/meta/wwise.c, the `0x30` extra-size
//! case, and src/coding/vorbis_custom_utils_wwise.c `setup_version_config`):
//! a 0x42-byte `fmt ` whose extra data from 0x18 is the `vorb` block (sample
//! count at 0x00, seek-table size at 0x10, first audio packet at 0x14,
//! blocksize exponents at 0x28 and 0x29), 2-byte little-endian packet sizes,
//! the aoTuV 6.03 codebook library, and modified audio packets unless both
//! blocksizes are equal. The older standalone `vorb` chunk of 0x2A bytes has
//! the same offsets and is read the same way.

use lewton::audio::{read_audio_packet_generic, PreviousWindowRight};
use lewton::header::{read_header_ident, read_header_setup};
use lewton::samples::InterleavedSamples;

/// ww2ogg's external codebook library for Wwise 2012 and later, the aoTuV
/// 6.03 codebooks: https://github.com/hcs64/ww2ogg (packed_codebooks_aoTuV_603.bin),
/// BSD-3-Clause, Copyright (c) 2002 Xiph.org Foundation, (c) 2009-2016 Adam
/// Gashlin. Layout (codebook.cpp `codebook_library`): codebook bytes, then a
/// table of u32 LE offsets, the last u32 of the file giving the table's
/// offset.
static CODEBOOKS: &[u8] = include_bytes!("packed_codebooks_aoTuV_603.bin");

/// Decoded media.
#[derive(Clone, Debug)]
pub struct Pcm {
    pub sample_rate: u32,
    pub channels: u16,
    /// Interleaved, `channels` per frame.
    pub samples: Vec<i16>,
    /// The `smpl` chunk's loop as frames, start inclusive and end exclusive
    /// (vgmstream wwise.c: loop end + 1 "like standard RIFF"). Diagnostic
    /// only: `XPLMPlayPCMOnBus` loops whatever buffer it is given whole, with
    /// no sub-range, so a looped sound here repeats whole, not just this
    /// range (`sound::Sound::play_pcm`).
    pub loop_frames: Option<(u32, u32)>,
}

impl Pcm {
    pub fn frames(&self) -> usize {
        self.samples.len() / self.channels.max(1) as usize
    }
}

/// What a WEM's headers declare.
#[derive(Clone, Debug)]
pub struct WemInfo {
    pub format_tag: u16,
    pub channels: u16,
    pub sample_rate: u32,
    pub avg_bytes_per_second: u32,
    pub bits_per_sample: u16,
    /// Frames: the `vorb` sample count for Vorbis, data size over block
    /// align for PCM.
    pub frames: u32,
    pub loop_frames: Option<(u32, u32)>,
    data: (usize, usize),
    vorb: Option<usize>,
}

impl WemInfo {
    /// The Vorbis blocksize exponents (short, long), when Vorbis.
    pub fn blocksizes(&self, wem: &[u8]) -> Option<(u8, u8)> {
        let v = self.vorb?;
        Some((*wem.get(v + 0x28)?, *wem.get(v + 0x29)?))
    }
}

fn u16_at(b: &[u8], o: usize) -> Result<u16, String> {
    b.get(o..o + 2)
        .map(|s| u16::from_le_bytes([s[0], s[1]]))
        .ok_or_else(|| format!("wem: truncated at {o:#x}"))
}

fn u32_at(b: &[u8], o: usize) -> Result<u32, String> {
    b.get(o..o + 4)
        .map(|s| u32::from_le_bytes([s[0], s[1], s[2], s[3]]))
        .ok_or_else(|| format!("wem: truncated at {o:#x}"))
}

/// Reads the RIFF chunks (wwriff.cpp constructor; vgmstream wwise.c
/// `parse_wwise`).
pub fn wem_info(wem: &[u8]) -> Result<WemInfo, String> {
    if wem.get(0..4) != Some(b"RIFF") || wem.get(8..12) != Some(b"WAVE") {
        return Err("wem: not a little-endian RIFF WAVE".into());
    }
    let riff_end = (u32_at(wem, 4)? as usize + 8).min(wem.len());
    let (mut fmt, mut data, mut smpl, mut vorb) = (None, None, None, None);
    let mut off = 12;
    while off + 8 <= riff_end {
        let size = u32_at(wem, off + 4)? as usize;
        let body = (off + 8, size);
        match &wem[off..off + 4] {
            b"fmt " => fmt = Some(body),
            b"data" => data = Some(body),
            b"smpl" => smpl = Some(body),
            b"vorb" => vorb = Some(body),
            _ => {}
        }
        off = off + 8 + size;
    }
    let fmt: (usize, usize) = fmt.ok_or("wem: no fmt chunk")?;
    let data: (usize, usize) = data.ok_or("wem: no data chunk")?;
    if data.0 + data.1 > wem.len() {
        return Err("wem: data chunk truncated".into());
    }
    let format_tag = u16_at(wem, fmt.0)?;
    let channels = u16_at(wem, fmt.0 + 2)?;
    let sample_rate = u32_at(wem, fmt.0 + 4)?;
    let avg_bytes_per_second = u32_at(wem, fmt.0 + 8)?;
    let block_align = u16_at(wem, fmt.0 + 12)?;
    let bits_per_sample = u16_at(wem, fmt.0 + 14)?;
    if channels == 0 || sample_rate == 0 {
        return Err("wem: zero channels or sample rate".into());
    }

    let mut vorb_off = None;
    let frames = match format_tag {
        0xFFFF => {
            let v = match (vorb, fmt.1) {
                (Some((o, 0x2A)), _) => o,
                (Some((_, s)), _) => return Err(format!("wem: unsupported vorb chunk size {s:#x}")),
                (None, 0x42) => fmt.0 + 0x18,
                (None, s) => return Err(format!("wem: unsupported Vorbis fmt size {s:#x}")),
            };
            vorb_off = Some(v);
            u32_at(wem, v)?
        }
        0x0001 | 0xFFFE => {
            if bits_per_sample != 16 || block_align != channels * 2 {
                return Err(format!("wem: PCM with {bits_per_sample} bits, block align {block_align}"));
            }
            (data.1 / block_align as usize) as u32
        }
        t => return Err(format!("wem: unsupported format tag {t:#06x}")),
    };

    // vgmstream wwise.c: one loop (count at 0x1C) of type 0 (0x24 + 0x04),
    // start at 0x24 + 0x08, end at 0x24 + 0x0C, end + 1.
    let loop_frames = match smpl {
        Some((o, s)) if s >= 0x34 && u32_at(wem, o + 0x1C)? == 1 && u32_at(wem, o + 0x28)? == 0 => {
            Some((u32_at(wem, o + 0x2C)?, u32_at(wem, o + 0x30)?.saturating_add(1)))
        }
        _ => None,
    };

    Ok(WemInfo {
        format_tag,
        channels,
        sample_rate,
        avg_bytes_per_second,
        bits_per_sample,
        frames,
        loop_frames,
        data,
        vorb: vorb_off,
    })
}

/// Decodes a WEM to interleaved i16, trimmed to the declared frame count.
pub fn decode_wem(wem: &[u8]) -> Result<Pcm, String> {
    let (info, mut pcm) = decode_untrimmed(wem)?;
    let declared = info.frames as usize * info.channels as usize;
    if pcm.samples.len() > declared {
        pcm.samples.truncate(declared);
    }
    Ok(pcm)
}

/// Decodes everything the stream holds; Vorbis may run past the declared
/// frame count up to the end of the last block.
pub fn decode_untrimmed(wem: &[u8]) -> Result<(WemInfo, Pcm), String> {
    let info = wem_info(wem)?;
    let (d0, dn) = info.data;
    let samples = match info.format_tag {
        0xFFFF => decode_vorbis(wem, &info)?,
        _ => wem[d0..d0 + dn - dn % 2].chunks_exact(2).map(|s| i16::from_le_bytes([s[0], s[1]])).collect(),
    };
    let pcm = Pcm {
        sample_rate: info.sample_rate,
        channels: info.channels,
        samples,
        loop_frames: info.loop_frames,
    };
    Ok((info, pcm))
}

// ---------------------------------------------------------------------------
// Bit I/O, least significant bit first as Vorbis packs (ww2ogg Bit_stream.h).

struct BitReader<'a> {
    b: &'a [u8],
    bit: usize,
}

impl<'a> BitReader<'a> {
    fn new(b: &'a [u8]) -> Self {
        Self { b, bit: 0 }
    }

    fn read(&mut self, n: u32) -> Result<u32, String> {
        let mut v = 0u32;
        for i in 0..n {
            let byte = *self.b.get(self.bit >> 3).ok_or("vorbis: setup packet out of bits")?;
            if (byte >> (self.bit & 7)) & 1 != 0 {
                v |= 1 << i;
            }
            self.bit += 1;
        }
        Ok(v)
    }
}

struct BitWriter {
    out: Vec<u8>,
    acc: u64,
    n: u32,
}

impl BitWriter {
    fn new() -> Self {
        Self { out: Vec::new(), acc: 0, n: 0 }
    }

    fn write(&mut self, v: u32, bits: u32) {
        let v = if bits >= 32 { v as u64 } else { (v as u64) & ((1u64 << bits) - 1) };
        self.acc |= v << self.n;
        self.n += bits;
        while self.n >= 8 {
            self.out.push(self.acc as u8);
            self.acc >>= 8;
            self.n -= 8;
        }
    }

    fn bytes(&mut self, b: &[u8]) {
        if self.n == 0 {
            self.out.extend_from_slice(b);
        } else {
            self.out.reserve(b.len());
            for &x in b {
                self.write(x as u32, 8);
            }
        }
    }

    fn finish(mut self) -> Vec<u8> {
        if self.n > 0 {
            self.out.push(self.acc as u8);
        }
        self.out
    }
}

fn ilog(mut v: u32) -> u32 {
    let mut r = 0;
    while v != 0 {
        r += 1;
        v >>= 1;
    }
    r
}

/// Tremor's `_book_maptype1_quantvals` (ww2ogg codebook.h).
fn maptype1_quantvals(entries: u32, dimensions: u32) -> Result<u32, String> {
    if dimensions == 0 || entries == 0 {
        return Err("vorbis: codebook with zero dimensions or entries".into());
    }
    let bits = ilog(entries);
    let mut vals = entries >> ((bits - 1) * (dimensions - 1) / dimensions);
    for _ in 0..0x10000 {
        let (mut acc, mut acc1) = (1u64, 1u64);
        for _ in 0..dimensions {
            acc = acc.saturating_mul(vals as u64);
            acc1 = acc1.saturating_mul(vals as u64 + 1);
        }
        if acc <= entries as u64 && acc1 > entries as u64 {
            return Ok(vals);
        } else if acc > entries as u64 {
            vals -= 1;
        } else {
            vals += 1;
        }
    }
    Err("vorbis: quantvals did not converge".into())
}

fn library_codebook(id: u32) -> Result<&'static [u8], String> {
    let n = CODEBOOKS.len();
    let table = u32_at(CODEBOOKS, n - 4)? as usize;
    let count = (n - table) / 4;
    let id = id as usize;
    if id + 1 >= count {
        return Err(format!("vorbis: codebook {id} not in the library"));
    }
    let start = u32_at(CODEBOOKS, table + 4 * id)? as usize;
    let end = u32_at(CODEBOOKS, table + 4 * (id + 1))? as usize;
    CODEBOOKS.get(start..end).ok_or_else(|| format!("vorbis: codebook {id} out of range"))
}

/// codebook.cpp `codebook_library::rebuild`: a packed codebook to a
/// standard one.
fn rebuild_codebook(cb: &[u8], os: &mut BitWriter) -> Result<(), String> {
    let mut bis = BitReader::new(cb);
    let dimensions = bis.read(4)?;
    let entries = bis.read(14)?;
    os.write(0x564342, 24);
    os.write(dimensions, 16);
    os.write(entries, 24);

    let ordered = bis.read(1)?;
    os.write(ordered, 1);
    if ordered != 0 {
        let initial_length = bis.read(5)?;
        os.write(initial_length, 5);
        let mut current = 0;
        while current < entries {
            let bits = ilog(entries - current);
            let number = bis.read(bits)?;
            os.write(number, bits);
            current += number;
        }
        if current > entries {
            return Err("vorbis: codebook ordered entries out of range".into());
        }
    } else {
        let length_bits = bis.read(3)?;
        let sparse = bis.read(1)?;
        if length_bits == 0 || length_bits > 5 {
            return Err("vorbis: nonsense codeword length".into());
        }
        os.write(sparse, 1);
        for _ in 0..entries {
            let present = if sparse != 0 {
                let p = bis.read(1)?;
                os.write(p, 1);
                p != 0
            } else {
                true
            };
            if present {
                let len = bis.read(length_bits)?;
                os.write(len, 5);
            }
        }
    }

    let lookup_type = bis.read(1)?;
    os.write(lookup_type, 4);
    if lookup_type == 1 {
        let min = bis.read(32)?;
        let max = bis.read(32)?;
        let value_length = bis.read(4)?;
        let sequence_flag = bis.read(1)?;
        os.write(min, 32);
        os.write(max, 32);
        os.write(value_length, 4);
        os.write(sequence_flag, 1);
        for _ in 0..maptype1_quantvals(entries, dimensions)? {
            let v = bis.read(value_length + 1)?;
            os.write(v, value_length + 1);
        }
    }

    // "if all bits are used in the last byte there will be one extra 0 byte"
    if bis.bit / 8 + 1 != cb.len() {
        return Err(format!("vorbis: codebook size {} but {} bits read", cb.len(), bis.bit));
    }
    Ok(())
}

fn vorbis_header(os: &mut BitWriter, packet_type: u32) {
    os.write(packet_type, 8);
    os.bytes(b"vorbis");
}

/// wwriff.cpp `generate_ogg_header`, identification packet.
fn ident_packet(info: &WemInfo, blocksizes: (u8, u8)) -> Vec<u8> {
    let mut os = BitWriter::new();
    vorbis_header(&mut os, 1);
    os.write(0, 32);
    os.write(info.channels as u32, 8);
    os.write(info.sample_rate, 32);
    os.write(0, 32);
    os.write(info.avg_bytes_per_second.wrapping_mul(8), 32);
    os.write(0, 32);
    os.write(blocksizes.0 as u32, 4);
    os.write(blocksizes.1 as u32, 4);
    os.write(1, 1);
    os.finish()
}

/// wwriff.cpp `generate_ogg_header`, setup packet with external codebooks
/// and a stripped setup. Returns the packet and each mode's block flag.
fn setup_packet(setup: &[u8], channels: u32) -> Result<(Vec<u8>, Vec<bool>), String> {
    let mut ss = BitReader::new(setup);
    let mut os = BitWriter::new();
    vorbis_header(&mut os, 5);

    let codebook_count_less1 = ss.read(8)?;
    let codebook_count = codebook_count_less1 + 1;
    os.write(codebook_count_less1, 8);
    for _ in 0..codebook_count {
        let id = ss.read(10)?;
        rebuild_codebook(library_codebook(id)?, &mut os)?;
    }

    // Time domain transforms (placeholder).
    os.write(0, 6);
    os.write(0, 16);

    let floor_count_less1 = ss.read(6)?;
    let floor_count = floor_count_less1 + 1;
    os.write(floor_count_less1, 6);
    for _ in 0..floor_count {
        os.write(1, 16); // always floor type 1
        let partitions = ss.read(5)?;
        os.write(partitions, 5);
        let mut class_list = Vec::with_capacity(partitions as usize);
        let mut maximum_class = 0;
        for _ in 0..partitions {
            let c = ss.read(4)?;
            os.write(c, 4);
            class_list.push(c);
            maximum_class = maximum_class.max(c);
        }
        let mut class_dimensions = Vec::with_capacity(maximum_class as usize + 1);
        for _ in 0..=maximum_class {
            let dims_less1 = ss.read(3)?;
            os.write(dims_less1, 3);
            class_dimensions.push(dims_less1 + 1);
            let subclasses = ss.read(2)?;
            os.write(subclasses, 2);
            if subclasses != 0 {
                let masterbook = ss.read(8)?;
                os.write(masterbook, 8);
                if masterbook >= codebook_count {
                    return Err("vorbis: invalid floor1 masterbook".into());
                }
            }
            for _ in 0..(1u32 << subclasses) {
                let book_plus1 = ss.read(8)?;
                os.write(book_plus1, 8);
                if book_plus1 > codebook_count {
                    return Err("vorbis: invalid floor1 subclass book".into());
                }
            }
        }
        let multiplier_less1 = ss.read(2)?;
        os.write(multiplier_less1, 2);
        let rangebits = ss.read(4)?;
        os.write(rangebits, 4);
        for &c in &class_list {
            for _ in 0..class_dimensions[c as usize] {
                let x = ss.read(rangebits)?;
                os.write(x, rangebits);
            }
        }
    }

    let residue_count_less1 = ss.read(6)?;
    let residue_count = residue_count_less1 + 1;
    os.write(residue_count_less1, 6);
    for _ in 0..residue_count {
        let residue_type = ss.read(2)?;
        os.write(residue_type, 16);
        if residue_type > 2 {
            return Err("vorbis: invalid residue type".into());
        }
        let begin = ss.read(24)?;
        let end = ss.read(24)?;
        let partition_size_less1 = ss.read(24)?;
        let classifications_less1 = ss.read(6)?;
        let classbook = ss.read(8)?;
        os.write(begin, 24);
        os.write(end, 24);
        os.write(partition_size_less1, 24);
        os.write(classifications_less1, 6);
        os.write(classbook, 8);
        if classbook >= codebook_count {
            return Err("vorbis: invalid residue classbook".into());
        }
        let mut cascade = Vec::with_capacity(classifications_less1 as usize + 1);
        for _ in 0..=classifications_less1 {
            let low = ss.read(3)?;
            os.write(low, 3);
            let flag = ss.read(1)?;
            os.write(flag, 1);
            let high = if flag != 0 {
                let h = ss.read(5)?;
                os.write(h, 5);
                h
            } else {
                0
            };
            cascade.push(high * 8 + low);
        }
        for c in cascade {
            for k in 0..8 {
                if c & (1 << k) != 0 {
                    let book = ss.read(8)?;
                    os.write(book, 8);
                    if book >= codebook_count {
                        return Err("vorbis: invalid residue book".into());
                    }
                }
            }
        }
    }

    let mapping_count_less1 = ss.read(6)?;
    let mapping_count = mapping_count_less1 + 1;
    os.write(mapping_count_less1, 6);
    for _ in 0..mapping_count {
        os.write(0, 16); // always mapping type 0
        let submaps_flag = ss.read(1)?;
        os.write(submaps_flag, 1);
        let mut submaps = 1;
        if submaps_flag != 0 {
            let less1 = ss.read(4)?;
            os.write(less1, 4);
            submaps = less1 + 1;
        }
        let square_polar = ss.read(1)?;
        os.write(square_polar, 1);
        if square_polar != 0 {
            let steps_less1 = ss.read(8)?;
            os.write(steps_less1, 8);
            let bits = ilog(channels - 1);
            for _ in 0..=steps_less1 {
                let magnitude = ss.read(bits)?;
                let angle = ss.read(bits)?;
                os.write(magnitude, bits);
                os.write(angle, bits);
                if angle == magnitude || magnitude >= channels || angle >= channels {
                    return Err("vorbis: invalid coupling".into());
                }
            }
        }
        let reserved = ss.read(2)?;
        os.write(reserved, 2);
        if reserved != 0 {
            return Err("vorbis: mapping reserved field nonzero".into());
        }
        if submaps > 1 {
            for _ in 0..channels {
                let mux = ss.read(4)?;
                os.write(mux, 4);
                if mux >= submaps {
                    return Err("vorbis: mapping_mux >= submaps".into());
                }
            }
        }
        for _ in 0..submaps {
            let time_config = ss.read(8)?;
            os.write(time_config, 8);
            let floor = ss.read(8)?;
            os.write(floor, 8);
            if floor >= floor_count {
                return Err("vorbis: invalid floor mapping".into());
            }
            let residue = ss.read(8)?;
            os.write(residue, 8);
            if residue >= residue_count {
                return Err("vorbis: invalid residue mapping".into());
            }
        }
    }

    let mode_count_less1 = ss.read(6)?;
    os.write(mode_count_less1, 6);
    let mut blockflags = Vec::with_capacity(mode_count_less1 as usize + 1);
    for _ in 0..=mode_count_less1 {
        let block_flag = ss.read(1)?;
        os.write(block_flag, 1);
        blockflags.push(block_flag != 0);
        os.write(0, 16); // window type
        os.write(0, 16); // transform type
        let mapping = ss.read(8)?;
        os.write(mapping, 8);
        if mapping >= mapping_count {
            return Err("vorbis: invalid mode mapping".into());
        }
    }
    os.write(1, 1); // framing

    if (ss.bit + 7) / 8 != setup.len() {
        return Err(format!("vorbis: setup packet is {} bytes but {} bits were read", setup.len(), ss.bit));
    }
    Ok((os.finish(), blockflags))
}

/// A 2-byte size-prefixed packet at `off`: (payload start, payload end).
fn packet_at(wem: &[u8], off: usize, end: usize) -> Result<(usize, usize), String> {
    if off + 2 > end {
        return Err(format!("vorbis: packet header truncated at {off:#x}"));
    }
    let size = u16_at(wem, off)? as usize;
    if off + 2 + size > end {
        return Err(format!("vorbis: packet at {off:#x} runs past the data chunk"));
    }
    Ok((off + 2, off + 2 + size))
}

fn decode_vorbis(wem: &[u8], info: &WemInfo) -> Result<Vec<i16>, String> {
    let vorb = info.vorb.ok_or("vorbis: no vorb data")?;
    let blocksizes = info.blocksizes(wem).ok_or("vorbis: vorb truncated")?;
    let setup_off = u32_at(wem, vorb + 0x10)? as usize;
    let audio_off = u32_at(wem, vorb + 0x14)? as usize;
    if info.channels > 255 {
        return Err("vorbis: too many channels".into());
    }
    let (d0, dn) = info.data;
    let data_end = d0 + dn;

    let (s0, s1) = packet_at(wem, d0 + setup_off, data_end)?;
    if s1 != d0 + audio_off {
        return Err("vorbis: first audio packet does not follow the setup packet".into());
    }
    let (setup_bytes, blockflags) = setup_packet(&wem[s0..s1], info.channels as u32)?;
    let ident = read_header_ident(&ident_packet(info, blocksizes)).map_err(|e| format!("vorbis: ident header: {e:?}"))?;
    let setup = read_header_setup(&setup_bytes, info.channels as u8, (blocksizes.0, blocksizes.1))
        .map_err(|e| format!("vorbis: setup header: {e:?}"))?;

    let mode_bits = ilog(blockflags.len() as u32 - 1);
    let mode_mask = (1u32 << mode_bits) - 1;
    // vorbis_custom_utils_wwise.c: packets are modified unless both
    // blocksizes are equal.
    let modified = blocksizes.0 != blocksizes.1;

    let mut pwr = PreviousWindowRight::new();
    let mut out: Vec<i16> = Vec::with_capacity(info.frames as usize * info.channels as usize + 4096);
    let mut prev_blockflag = false;
    let mut off = d0 + audio_off;
    while off < data_end {
        let (p0, p1) = packet_at(wem, off, data_end)?;
        off = p1;
        if p1 == p0 {
            continue;
        }
        let payload = &wem[p0..p1];
        let decoded = if modified {
            // wwriff.cpp `generate_ogg`: rebuild the packet type bit and,
            // for long windows, the previous and next window flags.
            let mut os = BitWriter::new();
            os.out.reserve(payload.len() + 1);
            os.write(0, 1);
            let first = payload[0] as u32;
            let mode = first & mode_mask;
            os.write(mode, mode_bits);
            let blockflag = *blockflags.get(mode as usize).ok_or("vorbis: packet mode out of range")?;
            if blockflag {
                let mut next_blockflag = false;
                if let Ok((n0, n1)) = packet_at(wem, p1, data_end) {
                    if n1 > n0 {
                        next_blockflag = blockflags.get((wem[n0] as u32 & mode_mask) as usize).copied().unwrap_or(false);
                    }
                }
                os.write(prev_blockflag as u32, 1);
                os.write(next_blockflag as u32, 1);
            }
            prev_blockflag = blockflag;
            os.write(first >> mode_bits, 8 - mode_bits);
            os.bytes(&payload[1..]);
            let packet = os.finish();
            read_audio_packet_generic::<InterleavedSamples<i16>>(&ident, &setup, &packet, &mut pwr)
        } else {
            read_audio_packet_generic::<InterleavedSamples<i16>>(&ident, &setup, payload, &mut pwr)
        };
        let decoded = decoded.map_err(|e| format!("vorbis: audio packet at {:#x}: {e:?}", p0 - 2))?;
        out.extend_from_slice(&decoded.samples);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sound::wwise::{test_package, PACKAGE_SOUND_DIR};
    use std::time::Instant;

    #[test]
    fn codebook_library_layout() {
        let n = CODEBOOKS.len();
        let table = u32_at(CODEBOOKS, n - 4).unwrap() as usize;
        // 599 offsets, the last one the end of the final codebook.
        let count = (n - table) / 4;
        assert_eq!(count, 599);
        assert!(library_codebook(count as u32 - 1).is_err());
        for id in 0..count as u32 - 1 {
            let mut os = BitWriter::new();
            rebuild_codebook(library_codebook(id).unwrap(), &mut os).unwrap_or_else(|e| panic!("codebook {id}: {e}"));
        }
    }

    #[test]
    fn bit_writer_matches_reader() {
        let mut w = BitWriter::new();
        let vals = [(1u32, 1u32), (0x2A, 6), (0x12345, 17), (0xFFFF_FFFF, 32), (3, 2), (0, 5)];
        for &(v, n) in &vals {
            w.write(v, n);
        }
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        for &(v, n) in &vals {
            assert_eq!(r.read(n).unwrap(), v);
        }
    }

    /// Every media item in the three packages, decoded and checked against
    /// its declared length and for sane levels.
    #[test]
    fn decode_every_media_item() {
        let Some(pkg) = test_package() else {
            eprintln!("skipped: MSFS package not found at {PACKAGE_SOUND_DIR}");
            return;
        };
        let mut decoded = 0;
        let mut by_tag = std::collections::BTreeMap::new();
        let mut total_frames = 0u64;
        let mut total_seconds = 0.0f64;
        let mut worst_mismatch = 0i64;
        let mut quiet = Vec::new();
        let mut differences = std::collections::BTreeMap::new();
        let (mut max_rms, mut max_clip, mut loops) = (0.0f64, 0.0f64, 0);
        let t0 = Instant::now();
        for bank in &pkg.banks {
            for id in bank.media_ids() {
                let wem = bank.media(id).unwrap();
                let (info, pcm) = decode_untrimmed(wem).unwrap_or_else(|e| panic!("media {id} (bank {}): {e}", bank.id));
                let frames = pcm.frames() as i64;
                let declared = info.frames as i64;
                let tolerance = match info.blocksizes(wem) {
                    Some((_, long)) => 1i64 << long,
                    None => 0,
                };
                let mismatch = frames - declared;
                if mismatch.abs() > worst_mismatch.abs() {
                    worst_mismatch = mismatch;
                }
                *differences.entry(mismatch.signum()).or_insert(0) += 1;
                assert!(
                    mismatch.abs() <= tolerance,
                    "media {id}: decoded {frames} frames, declared {declared} (tolerance {tolerance})"
                );
                let n = (declared as usize * info.channels as usize).min(pcm.samples.len());
                let trimmed = &pcm.samples[..n];
                assert!(!trimmed.is_empty(), "media {id}: no samples");
                let rms = (trimmed.iter().map(|&s| (s as f64) * (s as f64)).sum::<f64>() / n as f64).sqrt();
                let clipped = trimmed.iter().filter(|&&s| s == i16::MAX || s == i16::MIN).count();
                let peak = trimmed.iter().map(|&s| (s as i32).abs()).max().unwrap_or(0);
                assert!(peak > 0, "media {id}: all zero");
                if info.format_tag != 0xFFFF {
                    assert_eq!(info.bits_per_sample, 16, "media {id}: unsupported PCM bit depth");
                }
                // Garbage decodes run at full-scale noise or pinned to the
                // rails. The hottest real media, 550405798 (rms 22004, 10 %
                // of samples at the rails), is mastered that way: vgmstream
                // decodes it to within 2 LSB of this.
                assert!(rms < 26000.0, "media {id}: rms {rms:.0} looks like garbage");
                max_rms = max_rms.max(rms);
                loops += info.loop_frames.is_some() as usize;
                let clip_share = clipped as f64 / n as f64;
                assert!(clip_share < 0.2, "media {id}: {clipped} of {n} samples clipped");
                max_clip = max_clip.max(clip_share);
                if rms < 30.0 {
                    quiet.push((id, rms as u32, peak));
                }
                *by_tag.entry(info.format_tag).or_insert(0) += 1;
                total_frames += declared as u64;
                total_seconds += declared as f64 / info.sample_rate as f64;
                decoded += 1;
            }
        }
        let elapsed = t0.elapsed();
        eprintln!(
            "decoded {decoded} media items ({by_tag:x?} by format tag), {total_frames} frames, {total_seconds:.1} s of audio in {:.2} s; decoded minus declared frames: worst {worst_mismatch}, by sign {differences:?}; {loops} with smpl loops; highest rms {max_rms:.0}; most clipped {:.2} %; below rms 30 (id, rms, peak): {quiet:?}",
            elapsed.as_secs_f64(),
            max_clip * 100.0
        );
        assert!(decoded > 0);
    }
}
