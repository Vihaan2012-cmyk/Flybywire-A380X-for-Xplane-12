//! Throwaway probe: rewrite a DDS with its RGB forced to a flat colour and
//! its alpha left exactly as it was, then re-encode. Drop it in place of a
//! texture and the sim answers one question outright: if the geometry turns
//! a solid block of that colour, its alpha is being ignored and the whole
//! quad is drawing; if the colour appears only as lettering, alpha works and
//! the defect is elsewhere.
use msfs2xp::texture::{compress_rgba_to_dds, decode_within};

fn main() {
    let mut args = std::env::args().skip(1);
    let src = args.next().expect("source dds");
    let dst = args.next().expect("output dds");
    let r: u8 = args.next().map_or(255, |s| s.parse().unwrap());
    let g: u8 = args.next().map_or(0, |s| s.parse().unwrap());
    let b: u8 = args.next().map_or(0, |s| s.parse().unwrap());

    let data = std::fs::read(&src).unwrap();
    let (w, h, mut px) = decode_within(&data, 4096).unwrap();
    let (mut opaque, mut total) = (0u64, 0u64);
    for p in px.chunks_exact_mut(4) {
        total += 1;
        if p[3] >= 230 {
            opaque += 1;
        }
        p[0] = r;
        p[1] = g;
        p[2] = b;
    }
    let out = compress_rgba_to_dds(px, w, h).unwrap();
    std::fs::write(&dst, &out.bytes).unwrap();
    println!(
        "{w}x{h} -> {dst}: rgb forced to {r},{g},{b}, alpha untouched ({:.1}% of texels at/above 230)",
        100.0 * opaque as f64 / total as f64
    );
}
