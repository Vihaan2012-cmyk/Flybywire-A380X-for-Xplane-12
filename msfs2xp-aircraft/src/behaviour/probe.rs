//! Developer probes over a real behaviour XML (ignored tests):
//! `PROBE_XML=<model xml> PROBE_FILTER=<text> cargo test -- --ignored --nocapture probe_`.

use super::{bind, expand, xml};

fn lib() -> Option<xml::Library> {
    let p = std::env::var("PROBE_XML").ok()?;
    Some(xml::Library::load(std::path::Path::new(&p)).expect("load"))
}

#[test]
#[ignore]
fn probe_controls() {
    let Some(lib) = lib() else { return };
    let filter = std::env::var("PROBE_FILTER").unwrap_or_default();
    let e = expand::expand(&lib);
    let res = bind::resolve_all(&e.leaves);
    let (controls, _) = bind::controls(&e.leaves);
    for u in &res.unresolved {
        let line = format!("{} [{}] chain={:?} {}", u.anim, u.template, u.chain, u.reason);
        if !line.contains(&filter) {
            continue;
        }
        println!("== {line}");
        if let Some(c) = controls.iter().find(|c| c.anim == u.anim) {
            println!("   {:?}", c.action);
        }
    }
}

#[test]
#[ignore]
fn probe_leaves() {
    let Some(lib) = lib() else { return };
    let filter = std::env::var("PROBE_FILTER").unwrap_or_default();
    let e = expand::expand(&lib);
    for l in &e.leaves {
        let s = format!("{} node={:?} chain={:?} {:?}", l.template, l.node, l.chain, l.params);
        if s.contains(&filter) {
            println!("{s}\n");
        }
    }
}

#[test]
#[ignore]
fn probe_asobo() {
    let Ok(dir) = std::env::var("PROBE_ASOBO") else { return };
    let base = xml::Library::load_templates(std::path::Path::new(&dir));
    for m in &base.missing {
        println!("{m}");
    }
    println!("{} templates", base.templates.len());
}

#[test]
#[ignore]
fn probe_lights() {
    let (Some(lib), Ok(dir)) = (lib(), std::env::var("PROBE_ASOBO")) else { return };
    let filter = std::env::var("PROBE_FILTER").unwrap_or_default();
    let base = xml::Library::load_templates(std::path::Path::new(&dir));
    let e = expand::expand_with(&lib, Some(&base));
    for l in &e.lights {
        let s = format!("{:?} {:?} [{}] {}", l.kind, l.node, l.template, l.code.split_whitespace().collect::<Vec<_>>().join(" "));
        if s.contains(&filter) {
            println!("{s}");
        }
    }
}

/// Node names of a raw MSFS `.gltf` (JSON with an external `.bin`), read
/// directly from the JSON so multi-hundred-MB buffers are never touched.
fn gltf_node_names(path: &std::path::Path) -> Vec<String> {
    let Ok(text) = std::fs::read_to_string(path) else { return Vec::new() };
    let Ok(j) = serde_json::from_str::<serde_json::Value>(&text) else { return Vec::new() };
    j["nodes"]
        .as_array()
        .into_iter()
        .flatten()
        .map(|n| n["name"].as_str().unwrap_or_default().to_string())
        .collect()
}

/// Every emissive/visibility code's target node against a real shipped
/// model, grouped by why it is missing: `PROBE_XML` (interior model XML),
/// `PROBE_ASOBO` (MSFS's ModelBehaviorDefs), `PROBE_GLTF` (the cockpit LOD0
/// `.gltf` the converter loads), `PROBE_GLTF_LOD1` (the LOD1 `.gltf` the
/// converter skips) all as env vars:
/// `PROBE_XML=... PROBE_ASOBO=... PROBE_GLTF=... PROBE_GLTF_LOD1=... cargo
/// test -- --ignored --nocapture probe_missing_lights`.
#[test]
#[ignore]
fn probe_missing_lights() {
    let (Some(lib), Ok(dir), Ok(gltf)) = (lib(), std::env::var("PROBE_ASOBO"), std::env::var("PROBE_GLTF")) else {
        return;
    };
    let base = xml::Library::load_templates(std::path::Path::new(&dir));
    let e = expand::expand_with(&lib, Some(&base));

    let raw = gltf_node_names(std::path::Path::new(&gltf));
    let exact: std::collections::HashSet<&str> = raw.iter().map(String::as_str).collect();
    let lower: std::collections::HashSet<String> = raw.iter().map(|n| n.trim().to_ascii_lowercase()).collect();
    // A name with a Blender-style duplicate suffix (".001", "_2", ...) or
    // surrounding whitespace stripped, lower-cased, for the "renamed" bucket.
    let normalize = |n: &str| -> String {
        let n = n.trim();
        let stem = n.rsplit_once('.').filter(|(_, suf)| suf.len() == 3 && suf.chars().all(|c| c.is_ascii_digit())).map_or(n, |(s, _)| s);
        stem.trim_end_matches(|c: char| c == '_' || c.is_ascii_digit()).to_ascii_lowercase()
    };
    let normalized: std::collections::HashMap<String, Vec<&str>> = raw.iter().fold(std::collections::HashMap::new(), |mut m, n| {
        m.entry(normalize(n)).or_insert_with(Vec::new).push(n.as_str());
        m
    });
    let lod1: std::collections::HashSet<String> = std::env::var("PROBE_GLTF_LOD1")
        .ok()
        .map(|p| gltf_node_names(std::path::Path::new(&p)).iter().map(|n| n.trim().to_ascii_lowercase()).collect())
        .unwrap_or_default();

    let mut present = 0usize;
    let mut renamed: Vec<String> = Vec::new();
    let mut lod_only: Vec<String> = Vec::new();
    let mut absent: Vec<String> = Vec::new();
    let mut seen = std::collections::HashSet::new();
    for l in &e.lights {
        let Some(node) = l.node.as_deref().map(str::trim).filter(|n| !n.is_empty()) else { continue };
        if !seen.insert((node.to_string(), l.kind)) {
            continue;
        }
        let key = node.to_ascii_lowercase();
        if exact.contains(node) || lower.contains(&key) {
            present += 1;
            continue;
        }
        if let Some(cands) = normalized.get(&normalize(node)) {
            renamed.push(format!("{node} -> {cands:?}"));
            continue;
        }
        if lod1.contains(&key) {
            lod_only.push(node.to_string());
            continue;
        }
        absent.push(node.to_string());
    }
    println!("=== present: {present}");
    println!("=== renamed/duplicate-suffix ({}):", renamed.len());
    for n in &renamed {
        println!("  {n}");
    }
    println!("=== only in the LOD1 model ({}):", lod_only.len());
    for n in &lod_only {
        println!("  {n}");
    }
    println!("=== absent from every model file ({}):", absent.len());
    for n in &absent {
        println!("  {n}");
    }
}
