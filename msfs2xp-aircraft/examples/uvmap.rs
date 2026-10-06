//! Throwaway probe: for a named node, take its mesh's UV bounding box and
//! print an ASCII map of the albedo's alpha over that whole footprint, plus
//! the RGB of the texels that pass a given alpha cutoff. A per-triangle
//! centroid sample (triuvprobe) only reads one point per triangle and can
//! miss what a ring-shaped mesh actually covers; this shows the region.
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
    let tex_path = args.next().expect("texture file to sample");
    let cutoff: u8 = args.next().map_or(230, |s| s.parse().unwrap());

    let glb = gltf_as_glb(&gltf_path).unwrap();
    let m = load_glb(&glb).unwrap();
    let ni = m.nodes.iter().position(|n| n.name == node_name).expect("node not found");
    let mesh = m.meshes.iter().find(|x| x.node == Some(ni)).expect("mesh not found");

    let (mut u0, mut v0, mut u1, mut v1) = (f32::MAX, f32::MAX, f32::MIN, f32::MIN);
    for &i in &mesh.indices {
        let uv = m.meshes[0].vertices.get(0).map(|_| mesh.vertices[i as usize].uv).unwrap();
        u0 = u0.min(uv[0]);
        v0 = v0.min(uv[1]);
        u1 = u1.max(uv[0]);
        v1 = v1.max(uv[1]);
    }
    println!("node {node_name}  tris {}  uv [{u0:.4}..{u1:.4}] x [{v0:.4}..{v1:.4}]", mesh.indices.len() / 3);

    let data = std::fs::read(&tex_path).unwrap();
    let (w, h, px) = decode_within(&data, 4096).unwrap();
    let x0 = (u0 * w as f32).floor().max(0.0) as usize;
    let x1 = (u1 * w as f32).ceil().min(w as f32) as usize;
    let y0 = (v0 * h as f32).floor().max(0.0) as usize;
    let y1 = (v1 * h as f32).ceil().min(h as f32) as usize;
    println!("texels x {x0}..{x1}  y {y0}..{y1}  ({}x{})", x1 - x0, y1 - y0);

    let (mut pass, mut n, mut rs, mut gs, mut bs) = (0u64, 0u64, 0u64, 0u64, 0u64);
    for y in y0..y1 {
        for x in x0..x1 {
            let p = &px[(y * w as usize + x) * 4..][..4];
            n += 1;
            if p[3] >= cutoff {
                pass += 1;
                rs += p[0] as u64;
                gs += p[1] as u64;
                bs += p[2] as u64;
            }
        }
    }
    println!(
        "texels at/above alpha {cutoff}: {pass} of {n} ({:.1}%)   their mean rgb {}",
        100.0 * pass as f64 / n.max(1) as f64,
        if pass == 0 { "-".into() } else { format!("{},{},{}", rs / pass, gs / pass, bs / pass) }
    );

    // Coarse ASCII map: '#' opaque, '+' partial, '.' transparent.
    let rows = 28usize.min(y1 - y0);
    let cols = 96usize.min(x1 - x0);
    for r in 0..rows {
        let mut line = String::new();
        for c in 0..cols {
            let x = x0 + c * (x1 - x0) / cols;
            let y = y0 + r * (y1 - y0) / rows;
            let a = px[(y * w as usize + x) * 4 + 3];
            line.push(if a >= cutoff { '#' } else if a > 8 { '+' } else { '.' });
        }
        println!("  {line}");
    }
}
