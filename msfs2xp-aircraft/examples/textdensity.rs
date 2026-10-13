//! Throwaway probe: how many texels of its atlas a named part gets per
//! millimetre of its own surface.
//!
//! Sharpness of lettering is not the atlas's size and not the part's size;
//! it is the ratio. A 2048 sheet shared by fifty knobs can give a legend
//! fewer texels per millimetre than a 4096 sheet shared by five hundred
//! placards, and only this number says which. Printed alongside the part's
//! world size so the two can be compared directly against a part that looks
//! right.
use std::path::Path;

use msfs2xp::texture::decode_within;

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

fn main() {
    let mut args = std::env::args().skip(1);
    let gltf_path = Path::new(&args.next().expect("gltf path")).to_path_buf();
    let tex_dir = Path::new(&args.next().expect("texture dir")).to_path_buf();
    let needles: Vec<String> = args.map(|s| s.to_ascii_uppercase()).collect();

    let glb = gltf_as_glb(&gltf_path).unwrap();
    let m = load_glb(&glb).unwrap();

    println!(
        "{:<34} {:>7} {:>9} {:>11} {:>9}  {}",
        "node", "tris", "size mm", "texels/mm", "atlas", "texture"
    );
    let mut rows = Vec::new();
    for (ni, node) in m.nodes.iter().enumerate() {
        let up = node.name.to_ascii_uppercase();
        if !needles.iter().any(|n| up.contains(n.as_str())) {
            continue;
        }
        let Some(mesh) = m.meshes.iter().find(|x| x.node == Some(ni)) else { continue };
        if mesh.indices.is_empty() {
            continue;
        }

        // The part's own extent, and the extent of the UVs it samples.
        let (mut p0, mut p1) = ([f32::MAX; 3], [f32::MIN; 3]);
        let (mut u0, mut v0, mut u1, mut v1) = (f32::MAX, f32::MAX, f32::MIN, f32::MIN);
        for &i in &mesh.indices {
            let v = &mesh.vertices[i as usize];
            for k in 0..3 {
                p0[k] = p0[k].min(v.pos[k]);
                p1[k] = p1[k].max(v.pos[k]);
            }
            u0 = u0.min(v.uv[0]);
            v0 = v0.min(v.uv[1]);
            u1 = u1.max(v.uv[0]);
            v1 = v1.max(v.uv[1]);
        }
        // Longest side of the part, in mm, against the longest side of its
        // UV footprint, in texels.
        let mut sides = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
        sides.sort_by(f32::total_cmp);
        let mm = sides[2] * 1000.0;

        let tex = m
            .materials
            .get(mesh.material)
            .and_then(|mat| mat.base_color.clone())
            .unwrap_or_default();
        let (mut aw, mut ah) = (0u32, 0u32);
        if !tex.is_empty() {
            for cand in [tex.clone(), format!("{tex}.DDS"), format!("{tex}.PNG.DDS")] {
                if let Ok(d) = std::fs::read(tex_dir.join(&cand)) {
                    if let Ok((w, h, _)) = decode_within(&d, 8192) {
                        (aw, ah) = (w, h);
                        break;
                    }
                }
            }
        }
        let texels = ((u1 - u0) * aw as f32).max((v1 - v0) * ah as f32);
        let per_mm = if mm > 0.0 { texels / mm } else { 0.0 };
        rows.push((per_mm, node.name.clone(), mesh.indices.len() / 3, mm, aw, ah, tex));
    }
    rows.sort_by(|a, b| a.0.total_cmp(&b.0));
    for (per_mm, name, tris, mm, aw, ah, tex) in rows {
        println!(
            "{name:<34} {tris:>7} {mm:>9.1} {per_mm:>11.2} {:>9}  {tex}",
            format!("{aw}x{ah}")
        );
    }
}
