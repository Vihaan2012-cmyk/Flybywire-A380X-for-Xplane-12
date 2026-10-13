//! Throwaway probe: dump the CHRONO push button's UV bbox, material and a
//! crop of its albedo/COMP/normal/emissive textures for visual inspection.
use std::path::Path;

use image::{ImageBuffer, Rgba};
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

fn crop_and_save(name: &str, data: &[u8], u0: f32, v0: f32, u1: f32, v1: f32, pad: f32, out_dir: &Path) {
    let (w, h, px) = match decode_within(data, 4096) {
        Ok(v) => v,
        Err(e) => {
            eprintln!("{name}: decode failed: {e}");
            return;
        }
    };
    // Alpha histogram over the whole decoded image, not just the crop: is
    // there a meaningful transparent/opaque split at all, or is alpha flat?
    let mut lo = 0u32; // < 128
    let mut hi = 0u32; // >= 230 (DECAL_ALPHA_CUTOFF)
    let mut mid = 0u32;
    let (mut amin, mut amax) = (255u8, 0u8);
    for a in px.chunks_exact(4).map(|p| p[3]) {
        amin = amin.min(a);
        amax = amax.max(a);
        if a < 128 {
            lo += 1;
        } else if a >= 230 {
            hi += 1;
        } else {
            mid += 1;
        }
    }
    let total = (w * h) as f32;
    println!(
        "{name}: alpha min={amin} max={amax}, <128: {:.1}%, 128..230: {:.1}%, >=230: {:.1}%",
        lo as f32 / total * 100.0,
        mid as f32 / total * 100.0,
        hi as f32 / total * 100.0
    );
    let (u0, u1) = ((u0 - pad).clamp(0.0, 1.0), (u1 + pad).clamp(0.0, 1.0));
    let (v0, v1) = ((v0 - pad).clamp(0.0, 1.0), (v1 + pad).clamp(0.0, 1.0));
    let x0 = (u0 * w as f32) as u32;
    let x1 = ((u1 * w as f32) as u32).max(x0 + 1).min(w);
    let y0 = (v0 * h as f32) as u32;
    let y1 = ((v1 * h as f32) as u32).max(y0 + 1).min(h);
    let cw = x1 - x0;
    let ch = y1 - y0;
    println!("{name}: full {w}x{h}, crop [{x0},{y0}]-[{x1},{y1}] = {cw}x{ch}");
    let mut out = ImageBuffer::<Rgba<u8>, Vec<u8>>::new(cw, ch);
    for y in 0..ch {
        for x in 0..cw {
            let si = (((y0 + y) * w + (x0 + x)) * 4) as usize;
            out.put_pixel(x, y, Rgba([px[si], px[si + 1], px[si + 2], px[si + 3]]));
        }
    }
    out.save(out_dir.join(format!("{name}.png"))).unwrap();
}

fn main() {
    let mut args = std::env::args().skip(1);
    let gltf_path_s = args.next().expect("gltf path");
    let gltf_path = Path::new(&gltf_path_s);
    let tex_dir = args.next().expect("texture dir");
    let out_dir = args.next().expect("out dir");
    let out_dir = Path::new(&out_dir);
    std::fs::create_dir_all(out_dir).unwrap();

    let glb = gltf_as_glb(gltf_path).unwrap();
    let model = load_glb(&glb).unwrap();

    let node_name = args.next().unwrap_or_else(|| "PUSH_GLARESHIELD_CS_CHRONO".to_string());
    let node_idx = model
        .nodes
        .iter()
        .position(|n| n.name == node_name)
        .unwrap_or_else(|| panic!("node {node_name} not found"));
    println!("node {node_idx}: {:?}", model.nodes[node_idx]);

    let mesh = model
        .meshes
        .iter()
        .find(|m| m.node == Some(node_idx))
        .expect("mesh not found for node");
    println!("mesh {:?} material {}", mesh.name, mesh.material);

    let mat = &model.materials[mesh.material];
    println!("material: {mat:#?}");

    let (mut u0, mut v0, mut u1, mut v1) = (f32::MAX, f32::MAX, f32::MIN, f32::MIN);
    for v in &mesh.vertices {
        u0 = u0.min(v.uv[0]);
        v0 = v0.min(v.uv[1]);
        u1 = u1.max(v.uv[0]);
        v1 = v1.max(v.uv[1]);
    }
    println!("uv bbox: [{u0},{v0}]-[{u1},{v1}]");

    let tex_dir = Path::new(&tex_dir);
    for (label, file) in [
        ("albedo", mat.base_color.as_deref()),
        ("comp", mat.metal_rough.as_deref()),
        ("normal", mat.normal.as_deref()),
        ("emissive", mat.emissive.as_deref()),
    ] {
        if let Some(file) = file {
            let path = tex_dir.join(file);
            match std::fs::read(&path) {
                Ok(data) => crop_and_save(label, &data, u0, v0, u1, v1, 0.01, out_dir),
                Err(e) => eprintln!("{label} ({}): read failed: {e}", path.display()),
            }
        } else {
            println!("{label}: none");
        }
    }
}
