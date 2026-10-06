//! Throwaway probe: how much detail each DDS would lose by dropping its top
//! mip level, so the ones that do not use their resolution can be found.
//!
//! MSFS leaves the A380's panel faces bare and prints every legend on one
//! shared decal atlas, so most cockpit albedos are paint, bevels and bare
//! metal -- surfaces with nothing fine on them, carried at 4096 anyway. The
//! measure is the RMS difference, over 0-255, between each texel and the
//! mean of the 2x2 block it belongs to, which is exactly what the next mip
//! down already holds.
use msfs2xp::texture::decode_within;

fn detail_lost_by_halving(px: &[u8], w: u32, h: u32) -> f64 {
    let (nw, nh) = (w / 2, h / 2);
    if nw == 0 || nh == 0 {
        return f64::MAX;
    }
    let (mut se, mut n) = (0f64, 0u64);
    for y in 0..nh {
        for x in 0..nw {
            for k in 0..3 {
                let at = |dx: u32, dy: u32| px[(((2 * y + dy) * w + 2 * x + dx) * 4 + k) as usize] as f64;
                let mean = (at(0, 0) + at(1, 0) + at(0, 1) + at(1, 1)) / 4.0;
                for (dx, dy) in [(0u32, 0u32), (1, 0), (0, 1), (1, 1)] {
                    let d = at(dx, dy) - mean;
                    se += d * d;
                }
                n += 4;
            }
        }
    }
    (se / n.max(1) as f64).sqrt()
}

fn main() {
    let mut rows = Vec::new();
    for path in std::env::args().skip(1) {
        let Ok(data) = std::fs::read(&path) else { continue };
        let Ok((w, h, px)) = decode_within(&data, 8192) else {
            println!("  (could not decode) {path}");
            continue;
        };
        if w < 4096 {
            continue;
        }
        rows.push((detail_lost_by_halving(&px, w, h), w, h, path));
    }
    rows.sort_by(|a, b| b.0.total_cmp(&a.0));
    println!("{:>8}  {:>11}  {}", "RMS", "size", "texture");
    for (rms, w, h, path) in &rows {
        println!("{rms:>8.2}  {:>11}  {}", format!("{w}x{h}"), path.rsplit(['/', '\\']).next().unwrap_or(path));
    }
    if !rows.is_empty() {
        let med = rows[rows.len() / 2].0;
        println!("\n  {} at 4096, median {med:.2} RMS lost by halving", rows.len());
    }
}
