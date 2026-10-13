//! Generates a simple .ico at build time and embeds it as the executable's
//! icon resource (docs/briefs/xphfbw-app.md, agent L: "give the app an
//! .ico ... a build.rs with the winres crate or embed-resource").
//!
//! No binary asset is checked in: the icon (a white "X" roundel on navy,
//! for XPHFBW) is drawn procedurally as a 32x32 32bpp image and packed into
//! a minimal single-frame ICO container, then handed to `winresource` (an
//! actively maintained winres fork) to compile with the MSVC resource
//! compiler and link into the exe as resource id 1 — the id `LoadIconW`
//! uses in `tray.rs` for the tray icon, and the id Windows Explorer shows
//! for the exe itself.

use std::io::Write;
use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() != Ok("windows") {
        return;
    }

    let out_dir = PathBuf::from(std::env::var("OUT_DIR").expect("OUT_DIR"));
    let ico_path = out_dir.join("xphfbw.ico");
    std::fs::write(&ico_path, build_icon()).expect("write generated xphfbw.ico");

    let mut res = winresource::WindowsResource::new();
    res.set_icon(ico_path.to_str().expect("ico path is not valid UTF-8"));
    res.set("ProductName", "XPHFBW");
    res.set("FileDescription", "FlyByWire A380X companion app for X-Plane 12");
    if let Err(e) = res.compile() {
        // Missing Windows SDK / rc.exe shouldn't fail the whole build (the
        // app still runs, just without an exe/taskbar icon); the tray icon
        // itself falls back to none too (tray.rs: LoadIconW returns null).
        println!("cargo:warning=XPHFBW icon resource not embedded: {e}");
    }
}

/// A minimal single-image (32x32, 32bpp BGRA) ICO file: a white "X" inside
/// a navy roundel, built by hand so the app doesn't need a checked-in
/// binary asset. See the ICO/BMP layout this follows:
/// <https://learn.microsoft.com/en-us/previous-versions/ms997538(v=msdn.10)>
fn build_icon() -> Vec<u8> {
    const SIZE: usize = 32;
    const NAVY: [u8; 3] = [0x66, 0x2b, 0x0b]; // B,G,R
    let mut pixels = vec![0u8; SIZE * SIZE * 4]; // BGRA, top-down for now

    for y in 0..SIZE {
        for x in 0..SIZE {
            let (xf, yf) = (x as f32, y as f32);
            let center = (SIZE as f32 - 1.0) / 2.0;
            let (dx, dy) = (xf - center, yf - center);
            let in_roundel = (dx * dx + dy * dy).sqrt() <= SIZE as f32 / 2.0 - 0.5;
            let on_stroke = {
                let d1 = (x as i32 - y as i32).abs();
                let d2 = (x as i32 - (SIZE as i32 - 1 - y as i32)).abs();
                d1 <= 2 || d2 <= 2
            };
            let i = (y * SIZE + x) * 4;
            let bgra = if !in_roundel {
                [0, 0, 0, 0] // transparent outside the circle
            } else if on_stroke {
                [255, 255, 255, 255] // white X
            } else {
                [NAVY[0], NAVY[1], NAVY[2], 255]
            };
            pixels[i..i + 4].copy_from_slice(&bgra);
        }
    }

    let and_mask_row = ((SIZE + 31) / 32) * 4; // 1bpp rows, padded to 4 bytes
    let and_mask_len = and_mask_row * SIZE;
    let bmp_header_len = 40u32;
    let image_len = bmp_header_len as usize + pixels.len() + and_mask_len;

    let mut ico = Vec::with_capacity(6 + 16 + image_len);
    // ICONDIR
    ico.extend_from_slice(&0u16.to_le_bytes()); // reserved
    ico.extend_from_slice(&1u16.to_le_bytes()); // type: icon
    ico.extend_from_slice(&1u16.to_le_bytes()); // one image
    // ICONDIRENTRY
    ico.push(SIZE as u8); // width
    ico.push(SIZE as u8); // height
    ico.push(0); // color count (>=8bpp: 0)
    ico.push(0); // reserved
    ico.extend_from_slice(&1u16.to_le_bytes()); // color planes
    ico.extend_from_slice(&32u16.to_le_bytes()); // bits per pixel
    ico.extend_from_slice(&(image_len as u32).to_le_bytes());
    ico.extend_from_slice(&22u32.to_le_bytes()); // offset: 6 (ICONDIR) + 16 (this entry)
    // BITMAPINFOHEADER (the ICO packs a DIB without its BITMAPFILEHEADER)
    ico.extend_from_slice(&bmp_header_len.to_le_bytes());
    ico.extend_from_slice(&(SIZE as i32).to_le_bytes());
    ico.extend_from_slice(&((SIZE * 2) as i32).to_le_bytes()); // XOR + AND mask
    ico.extend_from_slice(&1u16.to_le_bytes()); // planes
    ico.extend_from_slice(&32u16.to_le_bytes()); // bpp
    ico.extend_from_slice(&0u32.to_le_bytes()); // BI_RGB
    ico.extend_from_slice(&(pixels.len() as u32).to_le_bytes());
    ico.extend_from_slice(&[0u8; 16]); // x/y ppm, colors used/important

    // XOR bitmap: bottom-up row order, as BMP/DIB requires.
    for row in (0..SIZE).rev() {
        ico.extend_from_slice(&pixels[row * SIZE * 4..(row + 1) * SIZE * 4]);
    }
    // AND mask: all zero (opaque); the alpha channel above carries
    // transparency, which every Windows version since XP honors for 32bpp
    // icons.
    ico.write_all(&vec![0u8; and_mask_len]).unwrap();

    ico
}
