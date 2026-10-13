//! Throwaway probe: every node whose geometry sits within a radius of a
//! named node, with its triangle count, size and albedo.
//!
//! MSFS leaves the A380's panel faces bare and prints every legend as its
//! own decal quad laid over them, so the part that carries a word is never
//! the part the word appears to be on. This finds it by position.
use std::path::Path;

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

fn centre_of(mesh: &model::glb::Mesh) -> [f32; 3] {
    let (mut lo, mut hi) = ([f32::MAX; 3], [f32::MIN; 3]);
    for &i in &mesh.indices {
        let p = mesh.vertices[i as usize].pos;
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0]
}

fn main() {
    let mut args = std::env::args().skip(1);
    let gltf_path = Path::new(&args.next().expect("gltf path")).to_path_buf();
    let node_name = args.next().expect("node name");
    let radius_mm: f32 = args.next().map_or(40.0, |s| s.parse().unwrap());

    let glb = gltf_as_glb(&gltf_path).unwrap();
    let m = load_glb(&glb).unwrap();
    let ni = m.nodes.iter().position(|n| n.name == node_name).expect("node not found");
    let anchor = centre_of(m.meshes.iter().find(|x| x.node == Some(ni)).expect("mesh"));

    let mut rows = Vec::new();
    for mesh in &m.meshes {
        let Some(k) = mesh.node else { continue };
        if mesh.indices.is_empty() {
            continue;
        }
        let c = centre_of(mesh);
        let d = ((c[0] - anchor[0]).powi(2) + (c[1] - anchor[1]).powi(2) + (c[2] - anchor[2]).powi(2)).sqrt() * 1000.0;
        if d > radius_mm {
            continue;
        }
        let tex = m
            .materials
            .get(mesh.material)
            .and_then(|mat| mat.base_color.clone())
            .unwrap_or_default();
        rows.push((d, m.nodes[k].name.clone(), mesh.indices.len() / 3, tex));
    }
    rows.sort_by(|a, b| a.0.total_cmp(&b.0));
    println!("within {radius_mm:.0} mm of {node_name}:");
    println!("{:>7}  {:<36} {:>6}  {}", "mm", "node", "tris", "albedo");
    for (d, name, tris, tex) in rows {
        println!("{d:>7.1}  {name:<36} {tris:>6}  {tex}");
    }
}
