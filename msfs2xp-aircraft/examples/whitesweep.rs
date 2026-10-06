//! Throwaway probe: decode one texture file and print it (silently doing
//! nothing if not) when a meaningful fraction of its texels average above
//! 200 per channel - a quick way to sweep a whole package's albedo files for
//! "which of these has a large near-white area" without opening each one.
//! Built while chasing a reported flat-white glareshield fixture; run over a
//! whole texture directory with a shell loop, e.g.
//! `for f in *_ALBEDO*.DDS; do ./whitesweep "$f"; done`.
use msfs2xp::texture::decode_within;

fn main() {
    let mut args = std::env::args().skip(1);
    let path = args.next().expect("path");
    let data = std::fs::read(&path).unwrap();
    let (w, h, px) = match decode_within(&data, 2048) {
        Ok(v) => v,
        Err(e) => {
            println!("{path}: decode failed: {e}");
            return;
        }
    };
    let total = (w * h) as f64;
    let mut near_white = 0u64;
    for p in px.chunks_exact(4) {
        if p[0] as u32 + p[1] as u32 + p[2] as u32 > 200 * 3 {
            near_white += 1;
        }
    }
    let frac = near_white as f64 / total * 100.0;
    if frac > 0.5 {
        println!("{path}: {w}x{h} near-white(>200 avg) = {:.1}%", frac);
    }
}
