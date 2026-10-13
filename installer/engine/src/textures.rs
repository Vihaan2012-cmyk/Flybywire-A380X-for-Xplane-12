use std::collections::BTreeSet;
use std::path::{Path, PathBuf};


pub const COCKPIT_REDUCED: u32 = 2048;

#[derive(Debug, Default, PartialEq)]
pub struct Summary {
    pub resized: usize,
    pub kept: usize,
    pub bytes_before: u64,
    pub bytes_after: u64,
}

fn texture_names(obj: &Path) -> Vec<String> {
    let Ok(text) = std::fs::read(obj) else { return Vec::new() };
    let text = String::from_utf8_lossy(&text[..text.len().min(64 * 1024)]).into_owned();
    let mut names = Vec::new();
    for line in text.lines() {
        let mut words = line.split_whitespace();
        match words.next() {
            Some("TEXTURE" | "TEXTURE_LIT" | "TEXTURE_NORMAL" | "TEXTURE_MATERIAL") => {
                if let Some(name) = words.next() {
                    names.push(name.to_string());
                }
            }
            Some("POINT_COUNTS") => break,
            _ => {}
        }
    }
    names
}

pub fn cockpit_only(objects: &Path) -> Vec<PathBuf> {
    let Ok(entries) = std::fs::read_dir(objects) else { return Vec::new() };
    let (mut cockpit, mut other) = (BTreeSet::new(), BTreeSet::new());
    for entry in entries.filter_map(Result::ok) {
        let name = entry.file_name().to_string_lossy().to_ascii_lowercase();
        if !name.ends_with(".obj") {
            continue;
        }
        let target = if name.starts_with("a380_cockpit_") { &mut cockpit } else { &mut other };
        target.extend(texture_names(&entry.path()));
    }
    cockpit.difference(&other).map(|n| objects.join(n)).filter(|p| p.is_file()).collect()
}

fn size_of(data: &[u8]) -> Option<(u32, u32)> {
    if data.len() >= 24 && data.starts_with(b"\x89PNG") {
        return Some((u32::from_be_bytes(data[16..20].try_into().ok()?), u32::from_be_bytes(data[20..24].try_into().ok()?)));
    }
    if data.len() >= 20 && data.starts_with(b"DDS ") {
        return Some((u32::from_le_bytes(data[16..20].try_into().ok()?), u32::from_le_bytes(data[12..16].try_into().ok()?)));
    }
    None
}

fn resize_png(data: &[u8], cap: u32) -> Result<Vec<u8>, String> {
    let img = image::load_from_memory_with_format(data, image::ImageFormat::Png).map_err(|e| e.to_string())?;
    let (w, h) = (img.width(), img.height());
    let scale = cap as f64 / w.max(h) as f64;
    let (nw, nh) = (((w as f64 * scale).round() as u32).max(1), ((h as f64 * scale).round() as u32).max(1));
    let small = img.resize_exact(nw, nh, image::imageops::FilterType::Triangle);
    let mut out = std::io::Cursor::new(Vec::new());
    small.write_to(&mut out, image::ImageFormat::Png).map_err(|e| e.to_string())?;
    Ok(out.into_inner())
}

struct Dds {
    width: u32,
    height: u32,
    format: texpresso::Format,
    fourcc: [u8; 4],
}

fn dds_header(data: &[u8]) -> Option<Dds> {
    if data.len() < 128 || !data.starts_with(b"DDS ") {
        return None;
    }
    let u32_at = |o: usize| u32::from_le_bytes(data[o..o + 4].try_into().unwrap_or_default());
    let fourcc: [u8; 4] = data[84..88].try_into().ok()?;
    let format = match &fourcc {
        b"DXT1" => texpresso::Format::Bc1,
        b"DXT3" => texpresso::Format::Bc2,
        b"DXT5" => texpresso::Format::Bc3,
        _ => return None,
    };
    Some(Dds { width: u32_at(16), height: u32_at(12), format, fourcc })
}

fn write_dds(fourcc: [u8; 4], format: texpresso::Format, rgba: image::RgbaImage) -> Vec<u8> {
    let (width, height) = rgba.dimensions();
    let mut levels = Vec::new();
    let mut level = rgba;
    loop {
        let (w, h) = level.dimensions();
        let mut out = vec![0u8; format.compressed_size(w as usize, h as usize)];
        format.compress(level.as_raw(), w as usize, h as usize, texpresso::Params::default(), &mut out);
        levels.push(out);
        if w == 1 && h == 1 {
            break;
        }
        level = image::imageops::resize(&level, (w / 2).max(1), (h / 2).max(1), image::imageops::FilterType::Triangle);
    }
    let mut out = Vec::with_capacity(128 + levels.iter().map(Vec::len).sum::<usize>());
    let mut put = |v: u32| out.extend_from_slice(&v.to_le_bytes());
    let header: [u32; 32] = {
        let mut h = [0u32; 32];
        h[0] = u32::from_le_bytes(*b"DDS ");
        h[1] = 124;
        h[2] = 0x1 | 0x2 | 0x4 | 0x1000 | 0x20000 | 0x80000;
        h[3] = height;
        h[4] = width;
        h[5] = levels[0].len() as u32;
        h[7] = levels.len() as u32;
        h[19] = 32;
        h[20] = 0x4;
        h[21] = u32::from_le_bytes(fourcc);
        h[27] = 0x1000 | 0x400000 | 0x8;
        h
    };
    for v in header {
        put(v);
    }
    for l in levels {
        out.extend_from_slice(&l);
    }
    out
}

fn resize(data: &[u8], cap: u32) -> Result<Vec<u8>, String> {
    if data.starts_with(&[0x89, 0x50, 0x4e, 0x47]) {
        return resize_png(data, cap);
    }
    let dds = dds_header(data).ok_or("not a DXT1/DXT3/DXT5 DDS")?;
    let top = dds.format.compressed_size(dds.width as usize, dds.height as usize);
    let blocks = data.get(128..128 + top).ok_or("the DDS is shorter than its own top level")?;
    let mut rgba = vec![0u8; dds.width as usize * dds.height as usize * 4];
    dds.format.decompress(blocks, dds.width as usize, dds.height as usize, &mut rgba);
    let full = image::RgbaImage::from_raw(dds.width, dds.height, rgba).ok_or("the decoded texture has the wrong size")?;
    let scale = cap as f64 / dds.width.max(dds.height) as f64;
    let (nw, nh) = (((dds.width as f64 * scale).round() as u32).max(4), ((dds.height as f64 * scale).round() as u32).max(4));
    let small = image::imageops::resize(&full, nw, nh, image::imageops::FilterType::Triangle);
    Ok(write_dds(dds.fourcc, dds.format, small))
}

pub fn power_of_two_at_most(cap: u32) -> u32 {
    if cap == 0 {
        return 1;
    }
    1 << (31 - cap.leading_zeros())
}

fn textures_in(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(entries) = std::fs::read_dir(dir) else { return };
    for entry in entries.filter_map(Result::ok) {
        let path = entry.path();
        if path.is_dir() {
            textures_in(&path, out);
        } else if path.extension().is_some_and(|x| x.eq_ignore_ascii_case("dds") || x.eq_ignore_ascii_case("png")) {
            out.push(path);
        }
    }
}

pub fn all_textures(aircraft: &Path) -> Vec<PathBuf> {
    let mut files = Vec::new();
    textures_in(&aircraft.join("objects"), &mut files);
    textures_in(&aircraft.join("liveries"), &mut files);
    files.sort();
    files
}

pub fn cap_all(aircraft: &Path, cap: u32, progress: impl FnMut(usize, usize)) -> Result<Summary, String> {
    cap_files(&all_textures(aircraft), power_of_two_at_most(cap), progress)
}

pub fn cap_cockpit(aircraft: &Path, cap: u32, progress: impl FnMut(usize, usize)) -> Result<Summary, String> {
    let files = cockpit_only(&aircraft.join("objects"));
    if files.is_empty() {
        return Err(format!("no cockpit textures found in {}", aircraft.join("objects").display()));
    }
    cap_files(&files, power_of_two_at_most(cap), progress)
}

fn cap_files(files: &[PathBuf], cap: u32, mut progress: impl FnMut(usize, usize)) -> Result<Summary, String> {
    let mut summary = Summary::default();
    for (i, path) in files.iter().enumerate() {
        progress(i, files.len());
        let data = std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
        match size_of(&data) {
            Some((w, h)) if w.max(h) > cap => {
                let out = resize(&data, cap).map_err(|e| format!("{}: {e}", path.display()))?;
                let partial = path.with_extension("resizing");
                std::fs::write(&partial, &out).map_err(|e| format!("{}: {e}", partial.display()))?;
                std::fs::rename(&partial, path).map_err(|e| format!("{}: {e}", path.display()))?;
                summary.resized += 1;
                summary.bytes_before += data.len() as u64;
                summary.bytes_after += out.len() as u64;
            }
            _ => summary.kept += 1,
        }
    }
    progress(files.len(), files.len());
    Ok(summary)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("fbw-installer-textures-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(dir.join("objects")).unwrap();
        dir
    }

    fn png(w: u32, h: u32) -> Vec<u8> {
        let img = image::RgbImage::from_fn(w, h, |x, y| image::Rgb([(x % 256) as u8, (y % 256) as u8, 128]));
        let mut out = std::io::Cursor::new(Vec::new());
        image::DynamicImage::ImageRgb8(img).write_to(&mut out, image::ImageFormat::Png).unwrap();
        out.into_inner()
    }

    #[test]
    fn only_textures_the_cockpit_alone_uses_are_shrunk_and_to_the_cap() {
        let dir = scratch("cap");
        let objects = dir.join("objects");
        std::fs::write(objects.join("a380_cockpit_000.obj"), "I\n800\nOBJ\nTEXTURE panel.png\nTEXTURE_NORMAL shared.png\nPOINT_COUNTS 0 0 0 0\n").unwrap();
        std::fs::write(objects.join("a380_exterior_000.obj"), "I\n800\nOBJ\nTEXTURE shared.png\nPOINT_COUNTS 0 0 0 0\n").unwrap();
        std::fs::write(objects.join("panel.png"), png(2048, 1024)).unwrap();
        std::fs::write(objects.join("shared.png"), png(2048, 2048)).unwrap();
        let s = cap_cockpit(&dir, 1024, |_, _| {}).unwrap();
        assert_eq!(s.resized, 1);
        assert_eq!(size_of(&std::fs::read(objects.join("panel.png")).unwrap()), Some((1024, 512)));
        assert_eq!(size_of(&std::fs::read(objects.join("shared.png")).unwrap()), Some((2048, 2048)), "the exterior uses it too");
        let again = cap_cockpit(&dir, 1024, |_, _| {}).unwrap();
        assert_eq!(again.resized, 0, "already at the cap");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn every_aircraft_and_livery_texture_is_capped() {
        let dir = scratch("all");
        std::fs::create_dir_all(dir.join("liveries/one/objects")).unwrap();
        std::fs::write(dir.join("objects/a.png"), png(4096, 2048)).unwrap();
        std::fs::write(dir.join("objects/b.png"), png(1024, 1024)).unwrap();
        std::fs::write(dir.join("liveries/one/objects/c.png"), png(4096, 4096)).unwrap();
        std::fs::write(dir.join("objects/notes.txt"), "x").unwrap();
        let s = cap_all(&dir, 2048, |_, _| {}).unwrap();
        assert_eq!((s.resized, s.kept), (2, 1));
        assert_eq!(size_of(&std::fs::read(dir.join("liveries/one/objects/c.png")).unwrap()), Some((2048, 2048)));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn x_plane_only_takes_power_of_two_dds_so_a_cap_rounds_down_to_one() {
        assert_eq!(power_of_two_at_most(1440), 1024);
        assert_eq!(power_of_two_at_most(2048), 2048);
        assert_eq!(power_of_two_at_most(4095), 2048);
    }

    #[test]
    fn a_dds_comes_back_as_a_dds_of_the_same_format_at_the_cap() {
        let img = image::RgbaImage::from_fn(2048, 1024, |x, y| image::Rgba([(x % 251) as u8, (y % 241) as u8, 90, if x % 2 == 0 { 255 } else { 40 }]));
        let dds = write_dds(*b"DXT5", texpresso::Format::Bc3, img);
        assert_eq!(size_of(&dds), Some((2048, 1024)));
        let out = resize(&dds, 1024).unwrap();
        let h = dds_header(&out).unwrap();
        assert_eq!((h.width, h.height, &h.fourcc), (1024, 512, b"DXT5"));
        let mips = u32::from_le_bytes(out[28..32].try_into().unwrap());
        assert_eq!(mips, 11, "1024x512 down to 1x1");
        let expected: usize = 128 + (0..mips).map(|i| texpresso::Format::Bc3.compressed_size((1024 >> i).max(1) as usize, (512 >> i).max(1) as usize)).sum::<usize>();
        assert_eq!(out.len(), expected);
    }
}
