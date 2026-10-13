//! Throwaway probe: save the region of a texture that a named node's UVs
//! actually cover, as a PNG, magnified so the texels are visible.
//!
//! Every other answer about "why does this lettering look soft" is a guess
//! until someone looks at the texels the part is sampling.
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
    let node_name = args.next().expect("node name");
    let tex_path = args.next().expect("texture file");
    let out_path = args.next().expect("output png");
    let zoom: u32 = args.next().map_or(4, |s| s.parse().unwrap());

    let glb = gltf_as_glb(&gltf_path).unwrap();
    let m = load_glb(&glb).unwrap();
    let ni = m.nodes.iter().position(|n| n.name == node_name).expect("node not found");
    let mesh = m.meshes.iter().find(|x| x.node == Some(ni)).expect("mesh not found");

    let (mut u0, mut v0, mut u1, mut v1) = (f32::MAX, f32::MAX, f32::MIN, f32::MIN);
    for &i in &mesh.indices {
        let uv = mesh.vertices[i as usize].uv;
        u0 = u0.min(uv[0]);
        v0 = v0.min(uv[1]);
        u1 = u1.max(uv[0]);
        v1 = v1.max(uv[1]);
    }

    let data = std::fs::read(&tex_path).unwrap();
    let (w, h, px) = decode_within(&data, 8192).unwrap();
    let pad = 4i64;
    let x0 = ((u0 * w as f32) as i64 - pad).max(0) as u32;
    let x1 = (((u1 * w as f32) as i64 + pad) as u32).min(w);
    let y0 = ((v0 * h as f32) as i64 - pad).max(0) as u32;
    let y1 = (((v1 * h as f32) as i64 + pad) as u32).min(h);
    let (cw, ch) = (x1.saturating_sub(x0).max(1), y1.saturating_sub(y0).max(1));
    println!("{node_name}: uv [{u0:.4}..{u1:.4}]x[{v0:.4}..{v1:.4}] -> texels {cw}x{ch} of {w}x{h}, zoom {zoom}");

    let (ow, oh) = (cw * zoom, ch * zoom);
    let mut out = vec![0u8; (ow * oh * 4) as usize];
    for y in 0..oh {
        for x in 0..ow {
            let sx = (x0 + x / zoom).min(w - 1);
            let sy = (y0 + y / zoom).min(h - 1);
            let s = ((sy * w + sx) * 4) as usize;
            let d = ((y * ow + x) * 4) as usize;
            out[d..d + 4].copy_from_slice(&px[s..s + 4]);
        }
    }
    image::save_buffer(&out_path, &out, ow, oh, image::ColorType::Rgba8).unwrap();
    println!("wrote {out_path} ({ow}x{oh})");
}
