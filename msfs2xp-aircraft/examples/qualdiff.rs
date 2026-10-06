//! Throwaway probe: decode two versions of the same texture and report how
//! far the second has drifted from the first, per channel and for alpha,
//! over the texels that carry the lettering. Built to put a number on what
//! re-encoding a BC7 source down to BC3 costs a decal atlas.
use msfs2xp::texture::decode_within;

fn main() {
    let mut args = std::env::args().skip(1);
    let a_path = args.next().expect("reference texture");
    let b_path = args.next().expect("comparison texture");

    let (aw, ah, a) = decode_within(&std::fs::read(&a_path).unwrap(), 4096).unwrap();
    let (bw, bh, b) = decode_within(&std::fs::read(&b_path).unwrap(), 4096).unwrap();
    if (aw, ah) != (bw, bh) {
        println!("different sizes: {aw}x{ah} vs {bw}x{bh}");
        return;
    }

    let (mut ae, mut an) = (0f64, 0u64); // alpha error, all texels
    let (mut ee, mut en) = (0f64, 0u64); // alpha error on edge texels only
    let (mut ce, mut cn) = (0f64, 0u64); // rgb error where the ink is
    let mut worst = 0u32;
    for (pa, pb) in a.chunks_exact(4).zip(b.chunks_exact(4)) {
        let da = (pa[3] as f64 - pb[3] as f64).abs();
        ae += da * da;
        an += 1;
        worst = worst.max(da as u32);
        // An "edge" texel: partially transparent in the reference, which is
        // exactly the anti-aliasing a hard re-encode destroys.
        if pa[3] > 8 && pa[3] < 248 {
            ee += da * da;
            en += 1;
        }
        if pa[3] >= 128 {
            for k in 0..3 {
                let d = pa[k] as f64 - pb[k] as f64;
                ce += d * d;
            }
            cn += 3;
        }
    }
    let rms = |sum: f64, n: u64| if n == 0 { 0.0 } else { (sum / n as f64).sqrt() };
    println!("{aw}x{ah}");
    println!("  alpha RMS error, all texels : {:.2}", rms(ae, an));
    println!("  alpha RMS error, edge texels: {:.2}   ({} texels, {:.1}% of the sheet)", rms(ee, en), en, 100.0 * en as f64 / an as f64);
    println!("  rgb   RMS error, ink texels : {:.2}", rms(ce, cn));
    println!("  worst single-texel alpha error: {worst}");
}
