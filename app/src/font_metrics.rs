fn be16(b: &[u8], at: usize) -> Option<u16> {
    Some(u16::from_be_bytes(b.get(at..at + 2)?.try_into().ok()?))
}

fn be32(b: &[u8], at: usize) -> Option<u32> {
    Some(u32::from_be_bytes(b.get(at..at + 4)?.try_into().ok()?))
}

fn checksum(b: &[u8]) -> u32 {
    b.chunks(4).fold(0u32, |sum, c| {
        let mut w = [0u8; 4];
        w[..c.len()].copy_from_slice(c);
        sum.wrapping_add(u32::from_be_bytes(w))
    })
}

struct Table {
    record: usize,
    offset: usize,
    length: usize,
}

fn tables(font: &[u8]) -> Option<Vec<(String, Table)>> {
    let count = be16(font, 4)? as usize;
    (0..count)
        .map(|i| {
            let record = 12 + 16 * i;
            let tag = String::from_utf8_lossy(font.get(record..record + 4)?).into_owned();
            let offset = be32(font, record + 8)? as usize;
            let length = be32(font, record + 12)? as usize;
            (offset + length <= font.len()).then_some((tag, Table { record, offset, length }))
        })
        .collect()
}

pub fn coherent_line_metrics(font: &[u8]) -> Option<Vec<u8>> {
    let tables = tables(font)?;
    let find = |tag: &str| tables.iter().find(|(t, _)| t == tag).map(|(_, t)| t);
    let (hhea, os2, head) = (find("hhea")?, find("OS/2")?, find("head")?);
    if os2.length < 78 || hhea.length < 10 || head.length < 12 {
        return None;
    }
    let fs_selection = be16(font, os2.offset + 62)?;
    if fs_selection & 0x80 == 0 {
        return None;
    }
    let s16 = |at: usize| be16(font, at).map(|v| v as i16 as i32);
    let (asc, desc, gap) = (s16(hhea.offset + 4)?, s16(hhea.offset + 6)?, s16(hhea.offset + 8)?);
    let (t_asc, t_desc, t_gap) = (s16(os2.offset + 68)?, s16(os2.offset + 70)?, s16(os2.offset + 72)?);
    if t_asc - t_desc + t_gap <= asc - desc + gap {
        return None;
    }
    let mut out = font.to_vec();
    for (at, v) in [(68, asc), (70, desc), (72, gap)] {
        out[os2.offset + at..os2.offset + at + 2].copy_from_slice(&(v as i16).to_be_bytes());
    }
    let os2_sum = checksum(&out[os2.offset..os2.offset + os2.length]);
    out[os2.record + 4..os2.record + 8].copy_from_slice(&os2_sum.to_be_bytes());
    out[head.offset + 8..head.offset + 12].copy_from_slice(&[0; 4]);
    let adjustment = 0xB1B0_AFBAu32.wrapping_sub(checksum(&out));
    out[head.offset + 8..head.offset + 12].copy_from_slice(&adjustment.to_be_bytes());
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn font(fs_selection: u16, typo: [i16; 3], hhea: [i16; 3]) -> Vec<u8> {
        let mut head = vec![0u8; 54];
        head[0..4].copy_from_slice(&0x0001_0000u32.to_be_bytes());
        let mut hh = vec![0u8; 36];
        for (i, v) in hhea.iter().enumerate() {
            hh[4 + 2 * i..6 + 2 * i].copy_from_slice(&v.to_be_bytes());
        }
        let mut os2 = vec![0u8; 96];
        os2[62..64].copy_from_slice(&fs_selection.to_be_bytes());
        for (i, v) in typo.iter().enumerate() {
            os2[68 + 2 * i..70 + 2 * i].copy_from_slice(&v.to_be_bytes());
        }
        let tables = [("OS/2", os2), ("head", head), ("hhea", hh)];
        let mut out = vec![0u8; 12 + 16 * tables.len()];
        out[0..4].copy_from_slice(&0x0001_0000u32.to_be_bytes());
        out[4..6].copy_from_slice(&(tables.len() as u16).to_be_bytes());
        for (i, (tag, data)) in tables.iter().enumerate() {
            let offset = out.len();
            let r = 12 + 16 * i;
            out[r..r + 4].copy_from_slice(tag.as_bytes());
            out[r + 4..r + 8].copy_from_slice(&checksum(data).to_be_bytes());
            out[r + 8..r + 12].copy_from_slice(&(offset as u32).to_be_bytes());
            out[r + 12..r + 16].copy_from_slice(&(data.len() as u32).to_be_bytes());
            out.extend_from_slice(data);
        }
        out
    }

    #[test]
    fn the_eis_font_gets_its_hhea_lines() {
        let f = font(0x80, [3276, -820, 369], [3287, -3, 369]);
        let out = coherent_line_metrics(&f).expect("patched");
        let t = tables(&out).unwrap();
        let os2 = &t.iter().find(|(n, _)| n == "OS/2").unwrap().1;
        let s16 = |at: usize| be16(&out, os2.offset + at).unwrap() as i16;
        assert_eq!((s16(68), s16(70), s16(72)), (3287, -3, 369));
        assert_eq!(be32(&out, os2.record + 4).unwrap(), checksum(&out[os2.offset..os2.offset + os2.length]));
        assert_eq!(checksum(&out), 0xB1B0_AFBA, "the whole font sums to the magic number");
        assert_eq!(out.len(), f.len());
    }

    #[test]
    fn fonts_that_do_not_need_it_are_left_alone() {
        assert!(coherent_line_metrics(&font(0x00, [3276, -820, 369], [3287, -3, 369])).is_none(), "no USE_TYPO_METRICS");
        assert!(coherent_line_metrics(&font(0x80, [1800, -400, 0], [1800, -400, 200])).is_none(), "typo lines already shorter");
        assert!(coherent_line_metrics(b"not a font").is_none());
    }
}
