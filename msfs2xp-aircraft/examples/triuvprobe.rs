//! Throwaway probe: for a named node's mesh, print the material it uses and,
//! for every triangle, the decoded albedo pixel at that triangle's own UV
//! centroid (wrapped modulo 1, matching a tiled sampler). Built to settle
//! whether a mesh whose *bounding box* spans a shared atlas (so a crop of
//! that box shows many unrelated parts at once, see chronoprobe) actually
//! draws with the bright or the dark region within it - the per-triangle
//! answer a bounding-box crop cannot give.
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
    while json.len() % 4 != 0 { json.push(b' '); }
    while bin.len() % 4 != 0 { bin.push(0); }
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
    let node_name = args.next().expect("node name");

    let glb = gltf_as_glb(&gltf_path).unwrap();
    let model = load_glb(&glb).unwrap();
    let node_idx = model.nodes.iter().position(|n| n.name == node_name).expect("node not found");
    let mesh = model.meshes.iter().find(|m| m.node == Some(node_idx)).expect("mesh not found");
    println!("mesh {:?} material {} triangles {}", mesh.name, mesh.material, mesh.indices.len() / 3);
    let mat = &model.materials[mesh.material];
    let Some(albedo_file) = mat.base_color.as_deref() else {
        println!("no base_color texture");
        return;
    };
    let data = std::fs::read(tex_dir.join(albedo_file)).unwrap();
    let (w, h, px) = decode_within(&data, 4096).unwrap();
    println!("albedo {w}x{h}");
    for (i, tri) in mesh.indices.chunks_exact(3).enumerate() {
        let uvs: Vec<[f32; 2]> = tri.iter().map(|&idx| mesh.vertices[idx as usize].uv).collect();
        let cu = (uvs[0][0] + uvs[1][0] + uvs[2][0]) / 3.0;
        let cv = (uvs[0][1] + uvs[1][1] + uvs[2][1]) / 3.0;
        let x = ((cu.rem_euclid(1.0)) * w as f32) as u32;
        let y = ((cv.rem_euclid(1.0)) * h as f32) as u32;
        let x = x.min(w - 1);
        let y = y.min(h - 1);
        let idx = ((y * w + x) * 4) as usize;
        let p = &px[idx..idx + 4];
        println!("tri {i}: centroid uv=({cu:.4},{cv:.4}) px=({x},{y}) rgba={:?}", p);
    }
}
