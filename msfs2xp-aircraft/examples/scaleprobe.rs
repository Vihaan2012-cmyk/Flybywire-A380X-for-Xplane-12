//! Throwaway probe: take a converted decal DDS's own full-resolution level,
//! box-filter it down exactly as `encode_dds` does, and report at each mip
//! what plain filtering leaves versus what the file actually stores -- i.e.
//! how much coverage `preserve_alpha_coverage` had to invent, and the scale
//! it needed to do it.
use msfs2xp::texture::{decode_rgba8, load};

fn half(px: &[u8], w: usize, h: usize) -> (Vec<u8>, usize, usize) {
    let (nw, nh) = ((w / 2).max(1), (h / 2).max(1));
    let mut out = vec![0u8; nw * nh * 4];
    for y in 0..nh {
        for x in 0..nw {
            let mut acc = [0u32; 4];
            for (dx, dy) in [(0, 0), (1, 0), (0, 1), (1, 1)] {
                let sx = (2 * x + dx).min(w - 1);
                let sy = (2 * y + dy).min(h - 1);
                for k in 0..4 {
                    acc[k] += px[(sy * w + sx) * 4 + k] as u32;
                }
            }
            for k in 0..4 {
                out[(y * nw + x) * 4 + k] = ((acc[k] + 2) / 4) as u8;
            }
        }
    }
    (out, nw, nh)
}

fn coverage(alpha: impl Iterator<Item = u8>, n: usize, cutoff: u8) -> f32 {
    alpha.filter(|&a| a >= cutoff).count() as f32 / n as f32
}

fn main() {
    let path = std::env::args().nth(1).expect("dds path");
    let cutoff: u8 = std::env::args().nth(2).map_or(230, |s| s.parse().unwrap());
    let data = std::fs::read(&path).unwrap();
    let img = load(&data).unwrap();
    let mut px = decode_rgba8(&img, 0).unwrap();
    let (mut w, mut h) = (img.width as usize, img.height as usize);
    let base = coverage(px.chunks_exact(4).map(|p| p[3]), w * h, cutoff);
    println!("{}x{}, {} mips, base coverage at cutoff {cutoff}: {:.2}%", w, h, img.mips.len(), base * 100.0);
    println!("{:>4}  {:>9}  {:>10}  {:>10}  {:>8}  {:>26}", "mip", "size", "plain box", "stored", "scale", "stored, alpha of plain<cutoff");
    for level in 1..img.mips.len().min(9) {
        let (p, nw, nh) = half(&px, w, h);
        px = p;
        w = nw;
        h = nh;
        let plain = coverage(px.chunks_exact(4).map(|p| p[3]), w * h, cutoff);
        let stored_px = match decode_rgba8(&img, level) {
            Ok(v) => v,
            Err(_) => break,
        };
        let stored = coverage(stored_px.chunks_exact(4).map(|p| p[3]), w * h, cutoff);
        // Median ratio stored/plain over texels the filter left with some alpha.
        let mut ratios: Vec<f32> = px
            .chunks_exact(4)
            .zip(stored_px.chunks_exact(4))
            .filter(|(a, _)| a[3] > 4)
            .map(|(a, b)| b[3] as f32 / a[3] as f32)
            .collect();
        ratios.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let med = ratios.get(ratios.len() / 2).copied().unwrap_or(0.0);
        // Of the texels the plain box filter left BELOW the cutoff, how many
        // does the stored file have at or above it? Those are the ones
        // coverage restoration promoted.
        let promoted = px
            .chunks_exact(4)
            .zip(stored_px.chunks_exact(4))
            .filter(|(a, b)| a[3] < cutoff && b[3] >= cutoff)
            .count();
        println!(
            "{level:>4}  {:>4}x{:<4}  {:>9.2}%  {:>9.2}%  {:>8.2}  {:>12} ({:>5.2}% of level)",
            w, h, plain * 100.0, stored * 100.0, med, promoted, promoted as f32 / (w * h) as f32 * 100.0
        );
    }
}
