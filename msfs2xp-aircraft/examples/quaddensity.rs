//! Throwaway probe: texel density triangle by triangle, not node by node.
//!
//! A node's UV bounding box says nothing when one node holds two hundred
//! separate legends scattered across a shared atlas -- its box spans the
//! sheet and the ratio comes out meaningless. Per triangle, the honest
//! number is the square root of (texels of UV area / square mm of surface),
//! which is texels per mm along a side.
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

fn main() {
    let mut args = std::env::args().skip(1);
    let gltf_path = Path::new(&args.next().expect("gltf path")).to_path_buf();
    let atlas: f32 = args.next().map_or(4096.0, |s| s.parse().unwrap());
    let needles: Vec<String> = args.map(|s| s.to_ascii_uppercase()).collect();

    let glb = gltf_as_glb(&gltf_path).unwrap();
    let m = load_glb(&glb).unwrap();

    for (ni, node) in m.nodes.iter().enumerate() {
        let up = node.name.to_ascii_uppercase();
        if !needles.iter().any(|n| up.contains(n.as_str())) {
            continue;
        }
        let Some(mesh) = m.meshes.iter().find(|x| x.node == Some(ni)) else { continue };
        let mut d: Vec<f32> = Vec::new();
        for t in mesh.indices.chunks_exact(3) {
            let v: Vec<_> = t.iter().map(|&i| &mesh.vertices[i as usize]).collect();
            // World area in mm^2, via the cross product of two edges.
            let e1 = [
                (v[1].pos[0] - v[0].pos[0]) * 1000.0,
                (v[1].pos[1] - v[0].pos[1]) * 1000.0,
                (v[1].pos[2] - v[0].pos[2]) * 1000.0,
            ];
            let e2 = [
                (v[2].pos[0] - v[0].pos[0]) * 1000.0,
                (v[2].pos[1] - v[0].pos[1]) * 1000.0,
                (v[2].pos[2] - v[0].pos[2]) * 1000.0,
            ];
            let cx = e1[1] * e2[2] - e1[2] * e2[1];
            let cy = e1[2] * e2[0] - e1[0] * e2[2];
            let cz = e1[0] * e2[1] - e1[1] * e2[0];
            let area_mm2 = (cx * cx + cy * cy + cz * cz).sqrt() / 2.0;
            // UV area in texels^2.
            let du1 = (v[1].uv[0] - v[0].uv[0]) * atlas;
            let dv1 = (v[1].uv[1] - v[0].uv[1]) * atlas;
            let du2 = (v[2].uv[0] - v[0].uv[0]) * atlas;
            let dv2 = (v[2].uv[1] - v[0].uv[1]) * atlas;
            let area_tex = (du1 * dv2 - dv1 * du2).abs() / 2.0;
            if area_mm2 > 1e-6 && area_tex > 0.0 {
                d.push((area_tex / area_mm2).sqrt());
            }
        }
        if d.is_empty() {
            continue;
        }
        d.sort_by(f32::total_cmp);
        println!(
            "{:<34} {:>5} tris   texels/mm  min {:>6.1}  median {:>6.1}  max {:>6.1}",
            node.name,
            d.len(),
            d[0],
            d[d.len() / 2],
            d[d.len() - 1]
        );
    }
}
