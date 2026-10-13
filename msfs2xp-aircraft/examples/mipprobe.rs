//! Throwaway probe: decode a specific stored mip level of a converted DDS
//! (the level X-Plane itself would sample from a normal cockpit viewing
//! distance), and report its alpha/whiteness stats plus a small pixel
//! window around a given UV, to see whether a coarse mip collapses a
//! decal's real lettering into a solid block.
use msfs2xp::texture::{decode_rgba8, load};

fn main() {
    let mut args = std::env::args().skip(1);
    let path = args.next().expect("dds path");
    let level: usize = args.next().expect("mip level").parse().unwrap();
    let (u, v): (f32, f32) = {
        let u = args.next().expect("u").parse().unwrap();
        let v = args.next().expect("v").parse().unwrap();
        (u, v)
    };
    let data = std::fs::read(&path).unwrap();
    let img = load(&data).unwrap();
    println!("source {}x{}, {} mip levels, format {:?}", img.width, img.height, img.mips.len(), img.format);
    let level = level.min(img.mips.len() - 1);
    let px = decode_rgba8(&img, level).unwrap();
    let (w, h) = ((img.width >> level).max(1), (img.height >> level).max(1));
    println!("level {level}: {w}x{h}");

    let (mut lo, mut mid, mut hi) = (0u32, 0u32, 0u32);
    let (mut rsum, mut gsum, mut bsum) = (0u64, 0u64, 0u64);
    for p in px.chunks_exact(4) {
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
        "alpha <128:{:.1}% mid:{:.1}% >=230:{:.1}%  avg rgb={:.1},{:.1},{:.1}",
        lo as f32 / n * 100.0,
        mid as f32 / n * 100.0,
        hi as f32 / n * 100.0,
        rsum as f64 / n as f64,
        gsum as f64 / n as f64,
        bsum as f64 / n as f64
    );

    // A small window (9x9) around the given UV at this level.
    let cx = (u.rem_euclid(1.0) * w as f32) as i32;
    let cy = (v.rem_euclid(1.0) * h as f32) as i32;
    println!("window around uv=({u},{v}) -> px=({cx},{cy}):");
    for dy in -4..=4 {
        let mut row = String::new();
        for dx in -4..=4 {
            let (x, y) = (cx + dx, cy + dy);
            if x < 0 || y < 0 || x as u32 >= w || y as u32 >= h {
                row.push_str("   . ");
                continue;
            }
            let i = ((y as u32 * w + x as u32) * 4) as usize;
            let p = &px[i..i + 4];
            row.push_str(&format!("{:3}a{:3} ", (p[0] as u16 + p[1] as u16 + p[2] as u16) / 3, p[3]));
        }
        println!("{row}");
    }
}
