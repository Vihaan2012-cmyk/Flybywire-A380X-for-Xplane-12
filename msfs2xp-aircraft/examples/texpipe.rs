use msfs2xp::texture::{bake_occlusion, compress_rgba_to_dds, decode_within, tint_rgba};

fn hex(s: &str) -> Option<[u8; 3]> {
    if s == "-" || s.len() != 6 {
        return None;
    }
    let v = u32::from_str_radix(s, 16).ok()?;
    Some([(v >> 16) as u8, (v >> 8) as u8, v as u8])
}

fn main() -> anyhow::Result<()> {
    let a: Vec<String> = std::env::args().collect();
    match a.get(1).map(String::as_str) {
        Some("albedo") => {
            let (src, comp, tint, max, out) = (&a[2], &a[3], &a[4], a[5].parse::<u32>()?, &a[6]);
            let (w, h, mut rgba) = decode_within(&std::fs::read(src)?, max).map_err(|e| anyhow::anyhow!("{e}"))?;
            if comp != "-" {
                let (cw, ch, c) = decode_within(&std::fs::read(comp)?, max).map_err(|e| anyhow::anyhow!("{e}"))?;
                bake_occlusion(&mut rgba, w, h, &c, cw, ch);
            }
            if let Some(t) = hex(tint) {
                tint_rgba(&mut rgba, t);
            }
            image::RgbaImage::from_raw(w, h, rgba).ok_or_else(|| anyhow::anyhow!("size"))?.save(out)?;
            println!("{w}x{h}");
        }
        Some("decode") => {
            let (src, max, out) = (&a[2], a[3].parse::<u32>()?, &a[4]);
            let (w, h, rgba) = decode_within(&std::fs::read(src)?, max).map_err(|e| anyhow::anyhow!("{e}"))?;
            image::RgbaImage::from_raw(w, h, rgba).ok_or_else(|| anyhow::anyhow!("size"))?.save(out)?;
            println!("{w}x{h}");
        }
        Some("dds") => {
            let img = image::open(&a[2])?.to_rgba8();
            let (w, h) = img.dimensions();
            let dds = compress_rgba_to_dds(img.into_raw(), w, h).map_err(|e| anyhow::anyhow!("{e}"))?;
            std::fs::write(&a[3], &dds.bytes)?;
            println!("{w}x{h} {} bytes", dds.bytes.len());
        }
        _ => anyhow::bail!("texpipe albedo <src> <comp|-> <tint|-> <max> <out.png> | decode <src> <max> <out.png> | dds <in.png> <out.dds>"),
    }
    Ok(())
}
