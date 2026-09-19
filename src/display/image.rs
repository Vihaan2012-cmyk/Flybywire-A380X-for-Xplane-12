//! Images the instruments draw: PNG files from the instruments' folder, or
//! PNG data URIs. SVG images are not rasterised here; the DOM side already
//! renders SVG, so it sends an SVG image's content as drawing commands.

use std::path::{Path, PathBuf};

/// A decoded image, RGBA with straight alpha, with its mipmap chain.
pub struct Picture {
    pub width: u32,
    pub height: u32,
    /// Level 0 first; each level half the size of the one before.
    pub levels: Vec<(u32, u32, Vec<u8>)>,
}

/// Where an image URL points: a data URI, or a file under the root.
pub fn resolve(root: &Path, url: &str) -> Result<Source, String> {
    if let Some(data) = url.strip_prefix("data:") {
        let (header, body) = data.split_once(',').ok_or("a data URI without data")?;
        if !header.starts_with("image/png") {
            return Err(format!("{header} data URIs are not drawn natively; send SVG as drawing commands"));
        }
        if !header.ends_with(";base64") {
            return Err("a PNG data URI that is not base64".into());
        }
        return Ok(Source::Data(base64(body)?));
    }
    // Instruments write root-relative URLs (`/Images/...`) as well as
    // relative ones; both are taken from the root.
    let relative = url.split(['?', '#']).next().unwrap_or("").trim_start_matches('/');
    let relative = relative.replace("%20", " ");
    if relative.split(['/', '\\']).any(|part| part == "..") {
        return Err("an image path may not leave the instruments' folder".into());
    }
    let path = root.join(&relative);
    let ext = path.extension().and_then(|e| e.to_str()).unwrap_or("").to_ascii_lowercase();
    if ext != "png" {
        return Err(format!("{ext} images are not drawn natively; send SVG as drawing commands"));
    }
    Ok(Source::File(path))
}

pub enum Source {
    File(PathBuf),
    Data(Vec<u8>),
}

pub fn load(source: &Source) -> Result<Picture, String> {
    let bytes = match source {
        Source::File(path) => std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?,
        Source::Data(bytes) => bytes.clone(),
    };
    decode_png(&bytes)
}

fn decode_png(bytes: &[u8]) -> Result<Picture, String> {
    let mut decoder = png::Decoder::new(bytes);
    decoder.set_transformations(png::Transformations::normalize_to_color8());
    let mut reader = decoder.read_info().map_err(|e| e.to_string())?;
    let mut buffer = vec![0; reader.output_buffer_size()];
    let info = reader.next_frame(&mut buffer).map_err(|e| e.to_string())?;
    let (w, h) = (info.width, info.height);
    let pixels = (w * h) as usize;
    let rgba = match info.color_type {
        png::ColorType::Rgba => buffer[..pixels * 4].to_vec(),
        png::ColorType::Rgb => buffer[..pixels * 3].chunks(3).flat_map(|p| [p[0], p[1], p[2], 255]).collect(),
        png::ColorType::GrayscaleAlpha => buffer[..pixels * 2].chunks(2).flat_map(|p| [p[0], p[0], p[0], p[1]]).collect(),
        png::ColorType::Grayscale => buffer[..pixels].iter().flat_map(|&g| [g, g, g, 255]).collect(),
        png::ColorType::Indexed => return Err("an indexed PNG the decoder did not expand".into()),
    };
    Ok(Picture { width: w, height: h, levels: mipmaps(w, h, rgba) })
}

/// The mipmap chain, each level a box filter of the one before, averaged in
/// premultiplied alpha so transparent pixels do not darken edges.
fn mipmaps(w: u32, h: u32, rgba: Vec<u8>) -> Vec<(u32, u32, Vec<u8>)> {
    let mut levels = vec![(w, h, rgba)];
    loop {
        let (pw, ph, prev) = levels.last().expect("level 0");
        if *pw == 1 && *ph == 1 {
            break;
        }
        let (pw, ph) = (*pw, *ph);
        let (nw, nh) = ((pw / 2).max(1), (ph / 2).max(1));
        let mut next = vec![0u8; (nw * nh * 4) as usize];
        for y in 0..nh {
            for x in 0..nw {
                let mut sum = [0u32; 4];
                let mut n = 0;
                for (sx, sy) in [(x * 2, y * 2), (x * 2 + 1, y * 2), (x * 2, y * 2 + 1), (x * 2 + 1, y * 2 + 1)] {
                    if sx >= pw || sy >= ph {
                        continue;
                    }
                    let i = ((sy * pw + sx) * 4) as usize;
                    let a = prev[i + 3] as u32;
                    for c in 0..3 {
                        sum[c] += prev[i + c] as u32 * a;
                    }
                    sum[3] += a;
                    n += 1;
                }
                let o = ((y * nw + x) * 4) as usize;
                if sum[3] > 0 {
                    for c in 0..3 {
                        next[o + c] = ((sum[c] + sum[3] / 2) / sum[3]) as u8;
                    }
                }
                next[o + 3] = ((sum[3] + n / 2) / n) as u8;
            }
        }
        levels.push((nw, nh, next));
    }
    levels
}

fn base64(text: &str) -> Result<Vec<u8>, String> {
    let mut out = Vec::with_capacity(text.len() * 3 / 4);
    let mut acc = 0u32;
    let mut bits = 0;
    for c in text.bytes() {
        let v = match c {
            b'A'..=b'Z' => c - b'A',
            b'a'..=b'z' => c - b'a' + 26,
            b'0'..=b'9' => c - b'0' + 52,
            b'+' | b'-' => 62,
            b'/' | b'_' => 63,
            b'=' | b' ' | b'\n' | b'\r' | b'\t' => continue,
            _ => return Err(format!("{:?} is not base64", c as char)),
        };
        acc = (acc << 6) | v as u32;
        bits += 6;
        if bits >= 8 {
            bits -= 8;
            out.push((acc >> bits) as u8);
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn urls_resolve_under_the_root_and_nowhere_else() {
        let root = Path::new("C:/root");
        let Ok(Source::File(p)) = resolve(root, "/Images/fbw-a380x/TRIM_INDICATOR.png") else { panic!() };
        assert_eq!(p, root.join("Images/fbw-a380x/TRIM_INDICATOR.png"));
        assert!(resolve(root, "../secret.png").is_err());
        assert!(resolve(root, "/Images/a.svg").is_err());
        assert!(resolve(root, "data:image/svg+xml;base64,AAAA").is_err());
    }

    #[test]
    fn a_png_data_uri_decodes_with_mipmaps() {
        // A 2x2 RGBA PNG: red, green / blue, transparent.
        let mut png_bytes = Vec::new();
        {
            let mut e = png::Encoder::new(&mut png_bytes, 2, 2);
            e.set_color(png::ColorType::Rgba);
            e.set_depth(png::BitDepth::Eight);
            let mut w = e.write_header().unwrap();
            w.write_image_data(&[255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255, 0, 0, 0, 0]).unwrap();
        }
        let encoded: String = {
            const T: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
            png_bytes
                .chunks(3)
                .flat_map(|c| {
                    let n = (c[0] as u32) << 16 | (*c.get(1).unwrap_or(&0) as u32) << 8 | *c.get(2).unwrap_or(&0) as u32;
                    let chars = [T[(n >> 18) as usize & 63], T[(n >> 12) as usize & 63], T[(n >> 6) as usize & 63], T[n as usize & 63]];
                    let keep = c.len() + 1;
                    chars.into_iter().enumerate().map(move |(i, ch)| if i < keep { ch as char } else { '=' })
                })
                .collect()
        };
        let source = resolve(Path::new("."), &format!("data:image/png;base64,{encoded}")).unwrap();
        let picture = load(&source).unwrap();
        assert_eq!((picture.width, picture.height, picture.levels.len()), (2, 2, 2));
        // The transparent pixel does not pull the average towards black.
        assert_eq!(picture.levels[1].2, vec![85, 85, 85, 191]);
    }
}
