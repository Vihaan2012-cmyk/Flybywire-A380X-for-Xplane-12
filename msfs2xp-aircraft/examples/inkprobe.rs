//! Throwaway probe: track the colour of the texels that actually pass the
//! alpha test, mip by mip. If lettering keeps its own ink colour the mean
//! stays put; if `dilate_transparent`'s flood is being box-filtered into it,
//! the mean drifts towards the flood as the levels coarsen.
use msfs2xp::texture::{decode_rgba8, load};

fn main() {
    let path = std::env::args().nth(1).expect("dds path");
    let cutoff: u8 = std::env::args().nth(2).map_or(230, |s| s.parse().unwrap());
    let data = std::fs::read(&path).unwrap();
    let img = load(&data).unwrap();
    println!("{}x{}, {} mips, cutoff {cutoff}", img.width, img.height, img.mips.len());
    println!("{:>4} {:>9} {:>22} {:>22} {:>10}", "mip", "size", "covered texels rgb", "transparent texels rgb", "dark<80");
    for level in 0..img.mips.len().min(10) {
        let px = match decode_rgba8(&img, level) {
            Ok(v) => v,
            Err(_) => break,
        };
        let (w, h) = ((img.width >> level).max(1), (img.height >> level).max(1));
        let (mut cs, mut cn, mut ts, mut tn, mut dark) = ([0u64; 3], 0u64, [0u64; 3], 0u64, 0u64);
        for p in px.chunks_exact(4) {
            let lum = (p[0] as u32 + p[1] as u32 + p[2] as u32) / 3;
            if p[3] >= cutoff {
                for k in 0..3 {
                    cs[k] += p[k] as u64;
                }
                cn += 1;
                if lum < 80 {
                    dark += 1;
                }
            } else if p[3] < 16 {
                for k in 0..3 {
                    ts[k] += p[k] as u64;
                }
                tn += 1;
            }
        }
        let m = |s: [u64; 3], n: u64| {
            if n == 0 { "-".into() } else { format!("{:.0},{:.0},{:.0}", s[0] / n, s[1] / n, s[2] / n) }
        };
        println!(
            "{level:>4} {:>4}x{:<4} {:>22} {:>22} {:>9.1}%",
            w, h, m(cs, cn), m(ts, tn),
            if cn == 0 { 0.0 } else { dark as f32 / cn as f32 * 100.0 }
        );
    }
}
