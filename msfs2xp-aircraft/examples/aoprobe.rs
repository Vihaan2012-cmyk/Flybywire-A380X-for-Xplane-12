//! Throwaway probe: bake a real COMP texture's occlusion into a real albedo
//! texture (the same [`bake_occlusion`] the shared texture module applies)
//! and report the mean and max per-channel change, in percent, plus the
//! fraction of texels whose COMP occlusion is "genuinely occluded" (below a
//! few thresholds) versus effectively unoccluded (at or near 1.0). Used to
//! measure how broadly the bake actually darkens a real cockpit panel,
//! rather than guessing from the formula alone.
use msfs2xp::texture::{bake_occlusion, decode_within};

fn main() {
    let mut args = std::env::args().skip(1);
    let albedo_path = args.next().expect("albedo path");
    let comp_path = args.next().expect("comp path");
    let cutoff: Option<f32> = args.next().map(|s| s.parse().unwrap());

    let albedo_data = std::fs::read(&albedo_path).unwrap();
    let comp_data = std::fs::read(&comp_path).unwrap();
    let (w, h, before) = decode_within(&albedo_data, 4096).unwrap();
    let (cw, ch, comp) = decode_within(&comp_data, 4096).unwrap();
    println!("{albedo_path}: {w}x{h}, comp {comp_path}: {cw}x{ch}");

    // Occlusion coverage from the COMP texture itself, resampled the same
    // way bake_occlusion does (nearest by proportion).
    let mut occ_hist = [0u32; 5]; // <0.25, <0.5, <0.75, <0.97, >=0.97
    for y in 0..h {
        let cy = (u64::from(y) * u64::from(ch) / u64::from(h)).min(u64::from(ch) - 1) as u32;
        for x in 0..w {
            let cx = (u64::from(x) * u64::from(cw) / u64::from(w)).min(u64::from(cw) - 1) as u32;
            let ao = comp[((cy * cw + cx) * 4) as usize] as f32 / 255.0;
            let bucket = if ao < 0.25 {
                0
            } else if ao < 0.5 {
                1
            } else if ao < 0.75 {
                2
            } else if ao < 0.97 {
                3
            } else {
                4
            };
            occ_hist[bucket] += 1;
        }
    }
    let n = (w * h) as f32;
    println!(
        "occlusion buckets: <0.25:{:.1}% <0.5:{:.1}% <0.75:{:.1}% <0.97:{:.1}% >=0.97:{:.1}%",
        occ_hist[0] as f32 / n * 100.0,
        occ_hist[1] as f32 / n * 100.0,
        occ_hist[2] as f32 / n * 100.0,
        occ_hist[3] as f32 / n * 100.0,
        occ_hist[4] as f32 / n * 100.0
    );

    let mut after = before.clone();
    match cutoff {
        None => bake_occlusion(&mut after, w, h, &comp, cw, ch),
        Some(cutoff) => {
            // Simulate a tempered bake: only occlusion below `cutoff` darkens,
            // ramped linearly to "no change" at the cutoff, so broad partial
            // ambient occlusion (MSFS's own panel-line/crevice shading) is
            // left alone and only near-total shadow still darkens.
            let mut tempered_comp = comp.clone();
            for p in tempered_comp.chunks_exact_mut(4) {
                let ao = p[0] as f32 / 255.0;
                let eff = if ao >= cutoff { 1.0 } else { ao / cutoff };
                p[0] = (eff * 255.0).round() as u8;
            }
            bake_occlusion(&mut after, w, h, &tempered_comp, cw, ch);
        }
    }

    let mut sum_before = [0f64; 3];
    let mut sum_after = [0f64; 3];
    let mut max_drop_pct = 0f32;
    let mut sum_drop_pct = 0f64;
    for (b, a) in before.chunks_exact(4).zip(after.chunks_exact(4)) {
        for k in 0..3 {
            sum_before[k] += b[k] as f64;
            sum_after[k] += a[k] as f64;
        }
        let lum_b = 0.2126 * b[0] as f32 + 0.7152 * b[1] as f32 + 0.0722 * b[2] as f32;
        let lum_a = 0.2126 * a[0] as f32 + 0.7152 * a[1] as f32 + 0.0722 * a[2] as f32;
        if lum_b > 0.5 {
            let drop_pct = (lum_b - lum_a) / lum_b * 100.0;
            sum_drop_pct += drop_pct as f64;
            if drop_pct > max_drop_pct {
                max_drop_pct = drop_pct;
            }
        }
    }
    let avg_before: Vec<f64> = sum_before.iter().map(|s| s / n as f64).collect();
    let avg_after: Vec<f64> = sum_after.iter().map(|s| s / n as f64).collect();
    let mean_lum_before = 0.2126 * avg_before[0] + 0.7152 * avg_before[1] + 0.0722 * avg_before[2];
    let mean_lum_after = 0.2126 * avg_after[0] + 0.7152 * avg_after[1] + 0.0722 * avg_after[2];
    println!(
        "avg rgb before = {:.1},{:.1},{:.1}  after = {:.1},{:.1},{:.1}",
        avg_before[0], avg_before[1], avg_before[2], avg_after[0], avg_after[1], avg_after[2]
    );
    println!(
        "mean luminance before={mean_lum_before:.2} after={mean_lum_after:.2}  overall mean change={:.2}%  per-texel mean drop={:.2}%  max single-texel drop={:.2}%",
        (mean_lum_before - mean_lum_after) / mean_lum_before * 100.0,
        sum_drop_pct / n as f64,
        max_drop_pct
    );
}
