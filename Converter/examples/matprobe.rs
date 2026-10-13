//! Throwaway probe: untextured materials of named models in a modellib.
use msfs2xp::bgl::modellib::ModelLibrary;
use serde_json::Value;

fn main() {
    let mut args = std::env::args().skip(1);
    let lib = ModelLibrary::open(std::path::Path::new(&args.next().unwrap())).unwrap();
    let wanted: Vec<String> = args.collect();
    for guid in lib.guids() {
        let Ok(info) = lib.info(guid) else { continue };
        if !wanted.iter().any(|w| info.name.contains(w)) {
            continue;
        }
        let Ok(glb) = lib.load_lod(guid, 0) else { continue };
        let jl = u32::from_le_bytes([glb[12], glb[13], glb[14], glb[15]]) as usize;
        let raw = &glb[20..20 + jl];
        let end = raw.iter().rposition(|&c| c != 0 && c != b' ').map_or(0, |i| i + 1);
        let j: Value = serde_json::from_slice(&raw[..end]).unwrap();
        println!("== {} ({} materials, {} nodes)", info.name, j["materials"].as_array().map_or(0, |a| a.len()), j["nodes"].as_array().map_or(0, |a| a.len()));
        let mut use_count = std::collections::HashMap::new();
        for m in j["meshes"].as_array().unwrap_or(&vec![]) {
            for p in m["primitives"].as_array().unwrap_or(&vec![]) {
                *use_count.entry(p["material"].as_u64().unwrap_or(0)).or_insert(0) += 1;
            }
        }
        for (i, m) in j["materials"].as_array().unwrap_or(&vec![]).iter().enumerate() {
            let pbr = &m["pbrMetallicRoughness"];
            if !pbr["baseColorTexture"].is_null() {
                continue;
            }
            println!(
                "  mat {i:3} {:38} prims {:3} alpha {:?} factor {} ext {}",
                m["name"].as_str().unwrap_or("").chars().take(38).collect::<String>(),
                use_count.get(&(i as u64)).unwrap_or(&0),
                m["alphaMode"].as_str().unwrap_or("OPAQUE"),
                pbr["baseColorFactor"],
                m["extensions"].as_object().map_or(String::new(), |o| o.keys().cloned().collect::<Vec<_>>().join(","))
            );
        }
    }
}
