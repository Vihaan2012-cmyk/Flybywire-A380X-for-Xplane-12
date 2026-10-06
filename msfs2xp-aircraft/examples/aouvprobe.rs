//! Throwaway probe: like `aoprobe`, but restricted to the texels a
//! material's own meshes actually sample (their UV triangles' rasterized
//! footprint), not the whole shared atlas. A COMP/albedo atlas is routinely
//! mostly unused padding (see `dilate_outside_uv`'s own doc in the texture
//! module - FlyByWire's pushbutton atlas is under a tenth real content), and
//! that padding's own default fill is not something any surface actually
//! shows, so a whole-image mean mixes it in and can look far worse than what
//! is ever on screen. This restricts to the real footprint the way
//! `dilate_outside_uv`/`uv_triangles` already do for the material/gloss map.
use std::path::Path;

use msfs2xp::texture::{bake_occlusion, decode_within};

#[path = "../src/model/mod.rs"]
mod model;
use model::glb::load_glb;

fn gltf_as_glb(gltf: &Path) -> anyhow::Result<Vec<u8>> {
    let mut json = std::fs::read(gltf)?;
    let j: serde_json::Value = serde_json::from_slice(&json)?;
    let uri = j["buffers"][0]["uri"].as_str().unwrap();
    let mut bin = std::fs::read(gltf.with_file_name(uri))?;
    while json.len() % 4 != 0 {
        json.push(b' ');
    }
    while bin.len() % 4 != 0 {
        bin.push(0);
    }
    let total = 12 + 8 + json.len() + 8 + bin.len();
    let mut out = Vec::with_capacity(total);
    out.extend_from_slice(b"glTF");
    out.extend_from_slice(&2u32.to_le_bytes());
    out.extend_from_slice(&(total as u32).to_le_bytes());
    out.extend_from_slice(&(json.len() as u32).to_le_bytes());
    out.extend_from_slice(b"JSON");
    out.extend_from_slice(&json);
    out.extend_from_slice(&(bin.len() as u32).to_le_bytes());
    out.extend_from_slice(b"BIN\0");
    out.extend_from_slice(&bin);
    Ok(out)
}

/// Mark a triangle's rasterized pixels in a `w`x`h` mask (same edge-function
/// test `rasterize_triangle` in the texture module uses).
fn rasterize(mask: &mut [bool], w: usize, h: usize, tri: &[[f32; 2]; 3]) {
    let px: Vec<[f32; 2]> = tri.iter().map(|&[u, v]| [u.rem_euclid(1.0) * w as f32, v.rem_euclid(1.0) * h as f32]).collect();
    let edge = |a: [f32; 2], b: [f32; 2], p: [f32; 2]| (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]);
    let area = edge(px[0], px[1], px[2]);
    if area.abs() <= f32::EPSILON {
        return;
    }
    let (x0, x1) = {
        let (lo, hi) = px.iter().fold((f32::MAX, f32::MIN), |(l, h), p| (l.min(p[0]), h.max(p[0])));
        ((lo.floor().max(0.0) as usize).min(w.saturating_sub(1)), (hi.ceil().max(1.0) as usize).min(w))
    };
    let (y0, y1) = {
        let (lo, hi) = px.iter().fold((f32::MAX, f32::MIN), |(l, h), p| (l.min(p[1]), h.max(p[1])));
        ((lo.floor().max(0.0) as usize).min(h.saturating_sub(1)), (hi.ceil().max(1.0) as usize).min(h))
    };
    for y in y0..y1 {
        for x in x0..x1 {
            let p = [x as f32 + 0.5, y as f32 + 0.5];
            let (e0, e1, e2) = (edge(px[0], px[1], p), edge(px[1], px[2], p), edge(px[2], px[0], p));
            if (e0 >= 0.0 && e1 >= 0.0 && e2 >= 0.0) || (e0 <= 0.0 && e1 <= 0.0 && e2 <= 0.0) {
                mask[y * w + x] = true;
            }
        }
    }
}

fn main() {
    let mut args = std::env::args().skip(1);
    let gltf_path = Path::new(&args.next().expect("gltf path")).to_path_buf();
    let tex_dir = Path::new(&args.next().expect("texture dir")).to_path_buf();
    let material_name = args.next().expect("material name");
    let cutoff: Option<f32> = args.next().map(|s| s.parse().unwrap());

    let glb = gltf_as_glb(&gltf_path).unwrap();
    let model = load_glb(&glb).unwrap();
    let mat_idx = model.materials.iter().position(|m| m.name == material_name).expect("material not found");
    let mat = &model.materials[mat_idx];
    let albedo_file = mat.base_color.clone().expect("material has no base_color");
    let comp_file = mat.metal_rough.clone().expect("material has no metal_rough/COMP texture");
    println!("material {mat_idx} {material_name}: albedo={albedo_file} comp={comp_file}");

    let mesh_ids: Vec<usize> = model.meshes.iter().enumerate().filter(|(_, m)| m.material == mat_idx).map(|(i, _)| i).collect();
    println!("{} meshes use this material", mesh_ids.len());

    let albedo_data = std::fs::read(tex_dir.join(&albedo_file)).unwrap();
    let comp_data = std::fs::read(tex_dir.join(&comp_file)).unwrap();
    let (w, h, before) = decode_within(&albedo_data, 4096).unwrap();
    let (cw, ch, comp) = decode_within(&comp_data, 4096).unwrap();
    println!("albedo {w}x{h}, comp {cw}x{ch}");

    // Rasterize this material's own UV footprint at the albedo's resolution.
    let (wu, hu) = (w as usize, h as usize);
    let mut used = vec![false; wu * hu];
    let mut tri_count = 0;
    for &mi in &mesh_ids {
        let mesh = &model.meshes[mi];
        for tri in mesh.indices.chunks_exact(3) {
            let Some(a) = mesh.vertices.get(tri[0] as usize) else { continue };
            let Some(b) = mesh.vertices.get(tri[1] as usize) else { continue };
            let Some(c) = mesh.vertices.get(tri[2] as usize) else { continue };
            rasterize(&mut used, wu, hu, &[a.uv, b.uv, c.uv]);
            tri_count += 1;
        }
    }
    let used_n = used.iter().filter(|&&u| u).count();
    println!(
        "{tri_count} triangles rasterized; {used_n} of {} texels ({:.2}%) are actually sampled",
        wu * hu,
        used_n as f32 / (wu * hu) as f32 * 100.0
    );

    let mut after = before.clone();
    let comp_for_bake = match cutoff {
        None => comp.clone(),
        Some(cutoff) => {
            let mut t = comp.clone();
            for p in t.chunks_exact_mut(4) {
                let ao = p[0] as f32 / 255.0;
                let eff = if ao >= cutoff { 1.0 } else { ao / cutoff };
                p[0] = (eff * 255.0).round() as u8;
            }
            t
        }
    };
    bake_occlusion(&mut after, w, h, &comp_for_bake, cw, ch);

    let (mut sb, mut sa) = (0f64, 0f64);
    let (mut occ_sum, mut occ_n) = (0f64, 0u32);
    let mut max_drop_pct = 0f32;
    let mut sum_drop_pct = 0f64;
    for i in 0..wu * hu {
        if !used[i] {
            continue;
        }
        let b = &before[i * 4..i * 4 + 4];
        let a = &after[i * 4..i * 4 + 4];
        let lum_b = 0.2126 * b[0] as f32 + 0.7152 * b[1] as f32 + 0.0722 * b[2] as f32;
        let lum_a = 0.2126 * a[0] as f32 + 0.7152 * a[1] as f32 + 0.0722 * a[2] as f32;
        sb += lum_b as f64;
        sa += lum_a as f64;
        let (cx, cy) = ((i % wu) as u32 * cw / w, (i / wu) as u32 * ch / h);
        occ_sum += comp[((cy * cw + cx) * 4) as usize] as f64 / 255.0;
        occ_n += 1;
        if lum_b > 0.5 {
            let drop_pct = (lum_b - lum_a) / lum_b * 100.0;
            sum_drop_pct += drop_pct as f64;
            if drop_pct > max_drop_pct {
                max_drop_pct = drop_pct;
            }
        }
    }
    let n = used_n.max(1) as f64;
    println!(
        "over used texels only: mean luminance before={:.2} after={:.2}  overall mean change={:.2}%  per-texel mean drop={:.2}%  max single-texel drop={:.2}%  mean occlusion={:.3}",
        sb / n,
        sa / n,
        (sb - sa) / sb * 100.0,
        sum_drop_pct / n,
        max_drop_pct,
        occ_sum / occ_n.max(1) as f64
    );
}
