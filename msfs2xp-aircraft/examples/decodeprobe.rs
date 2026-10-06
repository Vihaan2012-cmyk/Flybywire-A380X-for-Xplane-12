//! Throwaway probe: decode any texture file (PNG, DDS, whichever DXT/BC3
//! variant, KTX2) through the same `decode_within` the real pipeline uses,
//! print its alpha histogram and average RGB, and save it as a PNG for
//! visual inspection. Used to compare a source texture against what the
//! converter actually wrote for it.
use std::path::Path;

use image::{ImageBuffer, Rgba};
use msfs2xp::texture::decode_within;

fn main() {
    let mut args = std::env::args().skip(1);
    let path = args.next().expect("path");
    let out = args.next().expect("out png");
    let data = std::fs::read(&path).unwrap();
    let (w, h, px) = decode_within(&data, 4096).unwrap();
    println!("{path}: {w}x{h}");

    let (mut amin, mut amax) = (255u8, 0u8);
    let (mut lo, mut mid, mut hi) = (0u32, 0u32, 0u32);
    let (mut rsum, mut gsum, mut bsum) = (0u64, 0u64, 0u64);
    for p in px.chunks_exact(4) {
        amin = amin.min(p[3]);
        amax = amax.max(p[3]);
        match p[3] {
            0..=127 => lo += 1,
            230..=255 => hi += 1,
            _ => mid += 1,
        }
        rsum += p[0] as u64;
        gsum += p[1] as u64;
        bsum += p[2] as u64;
    }
    let n = (w * h) as f32;
    println!(
        "alpha min={amin} max={amax} <128:{:.1}% mid:{:.1}% >=230:{:.1}%",
        lo as f32 / n * 100.0,
        mid as f32 / n * 100.0,
        hi as f32 / n * 100.0
    );
    println!("avg rgb = {:.1},{:.1},{:.1}", rsum as f64 / n as f64, gsum as f64 / n as f64, bsum as f64 / n as f64);

    let img = ImageBuffer::<Rgba<u8>, Vec<u8>>::from_raw(w, h, px).unwrap();
    img.save(Path::new(&out)).unwrap();
}
