//! Throwaway probe: for every node drawing a given material, report what
//! fraction of its UV footprint in the albedo is at or above the alpha
//! cutoff, and the mean RGB of those texels. A node whose footprint is
//! almost entirely opaque draws as a filled rectangle whatever the alpha
//! test or blend mode does -- which is what a render-state change cannot
//! fix. Sorted most-opaque first.
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
    let tex_path = args.next().expect("texture file");
    let want_mat: usize = args.next().expect("material index").parse().unwrap();
    let cutoff: u8 = args.next().map_or(230, |s| s.parse().unwrap());

    let glb = gltf_as_glb(&gltf_path).unwrap();
    let m = load_glb(&glb).unwrap();
    let data = std::fs::read(&tex_path).unwrap();
    let (w, h, px) = decode_within(&data, 4096).unwrap();

    let mut rows: Vec<(f64, String, usize, u64, u64, u64, u64)> = Vec::new();
    for mesh in &m.meshes {
        if mesh.material != want_mat || mesh.indices.is_empty() {
            continue;
        }
        let name = mesh.node.and_then(|n| m.nodes.get(n)).map(|n| n.name.clone()).unwrap_or_default();
        let (mut u0, mut v0, mut u1, mut v1) = (f32::MAX, f32::MAX, f32::MIN, f32::MIN);
        for &i in &mesh.indices {
            let uv = mesh.vertices[i as usize].uv;
            u0 = u0.min(uv[0]);
            v0 = v0.min(uv[1]);
            u1 = u1.max(uv[0]);
            v1 = v1.max(uv[1]);
        }
        let x0 = (u0.rem_euclid(1.0) * w as f32) as usize;
        let y0 = (v0.rem_euclid(1.0) * h as f32) as usize;
        let sx = (((u1 - u0) * w as f32) as usize).max(1).min(w as usize);
        let sy = (((v1 - v0) * h as f32) as usize).max(1).min(h as usize);
        let (mut pass, mut n, mut rs, mut gs, mut bs) = (0u64, 0u64, 0u64, 0u64, 0u64);
        for dy in 0..sy {
            for dx in 0..sx {
                let x = (x0 + dx) % w as usize;
                let y = (y0 + dy) % h as usize;
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
        let frac = pass as f64 / n.max(1) as f64;
        rows.push((frac, name, mesh.indices.len() / 3, n, rs / pass.max(1), gs / pass.max(1), bs / pass.max(1)));
    }
    rows.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
    println!("{} nodes on material {want_mat}", rows.len());
    let heavy = rows.iter().filter(|r| r.0 > 0.8).count();
    let light = rows.iter().filter(|r| r.0 < 0.5).count();
    println!("footprint at/above alpha {cutoff}:  >80% opaque: {heavy}   <50%: {light}");
    println!("\nmost-opaque 15:");
    for (f, n, t, px_n, r, g, b) in rows.iter().take(15) {
        println!("  {:>6.1}%  tris {:>4}  texels {:>7}  rgb {:>3},{:>3},{:>3}  {}", f * 100.0, t, px_n, r, g, b, n);
    }
    println!("\nleast-opaque 5:");
    for (f, n, t, px_n, r, g, b) in rows.iter().rev().take(5) {
        println!("  {:>6.1}%  tris {:>4}  texels {:>7}  rgb {:>3},{:>3},{:>3}  {}", f * 100.0, t, px_n, r, g, b, n);
    }
}
