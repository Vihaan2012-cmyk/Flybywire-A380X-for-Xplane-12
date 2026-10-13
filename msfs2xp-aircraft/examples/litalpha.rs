//! Throwaway probe/fix: give a night (LIT) texture the alpha channel of the
//! albedo it is drawn with, keeping its own RGB.
//!
//! X-Plane applies an alpha-tested object's cutoff against the night
//! texture's alpha when the object has one. MSFS's emissive atlases are
//! fully opaque -- their alpha carries no meaning -- so an alpha-tested
//! decal object with one never discards anything and every lettering quad
//! draws as a filled rectangle. Copying the albedo's alpha across makes the
//! test see the lettering's own coverage.
use msfs2xp::texture::{compress_rgba_to_dds, decode_within};

fn main() {
    let mut args = std::env::args().skip(1);
    let lit_path = args.next().expect("lit dds");
    let albedo_path = args.next().expect("albedo dds (alpha source)");
    let out_path = args.next().expect("output dds");

    let (lw, lh, mut lit) = decode_within(&std::fs::read(&lit_path).unwrap(), 4096).unwrap();
    let (aw, ah, alb) = decode_within(&std::fs::read(&albedo_path).unwrap(), 4096).unwrap();

    let (mut before, mut after, mut n) = (0u64, 0u64, 0u64);
    for y in 0..lh as usize {
        for x in 0..lw as usize {
            let i = (y * lw as usize + x) * 4;
            // Sample the albedo by proportion, so differing sizes still line
            // up through the UV set the two share.
            let ax = x * aw as usize / lw as usize;
            let ay = y * ah as usize / lh as usize;
            let a = alb[(ay * aw as usize + ax) * 4 + 3];
            n += 1;
            if lit[i + 3] >= 230 {
                before += 1;
            }
            if a >= 230 {
                after += 1;
            }
            lit[i + 3] = a;
        }
    }
    let out = compress_rgba_to_dds(lit, lw, lh).unwrap();
    std::fs::write(&out_path, &out.bytes).unwrap();
    println!(
        "{lw}x{lh} -> {out_path}\n  alpha at/above 230: was {:.1}%, now {:.1}% (taken from the albedo)",
        100.0 * before as f64 / n as f64,
        100.0 * after as f64 / n as f64
    );
}
