//! Module resolution, as Node and bundlers resolve ES module specifiers.
//!
//! Relative and absolute specifiers resolve against the importing file.
//! Bare specifiers go through an import map first (longest prefix wins),
//! then `node_modules` directories up from the importer. A path without an
//! extension tries the TypeScript and JavaScript extensions, then a
//! directory's `package.json` (`module`, `exports["."]`, `main`) and its
//! index file. Module names are normalised absolute paths with forward
//! slashes, so one file is one module however it is imported.

use std::path::{Component, Path, PathBuf};

const EXTENSIONS: [&str; 8] = ["ts", "tsx", "mts", "js", "mjs", "jsx", "cjs", "json"];
const INDEX: [&str; 5] = ["index.ts", "index.tsx", "index.mts", "index.js", "index.mjs"];

/// An import map: bare specifier prefix to a directory or file.
#[derive(Clone, Debug, Default)]
pub struct ImportMap {
    entries: Vec<(String, PathBuf)>,
}

impl ImportMap {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn insert(&mut self, prefix: impl Into<String>, target: impl Into<PathBuf>) {
        self.entries.push((prefix.into(), target.into()));
        // Longest prefix first.
        self.entries.sort_by(|a, b| b.0.len().cmp(&a.0.len()));
    }

    /// Read `{"imports": {"name": "relative/or/absolute/path"}}`, paths
    /// relative to `base`.
    pub fn from_json(text: &str, base: &Path) -> Result<Self, String> {
        let mut map = Self::new();
        let body = text.trim();
        let imports = find_object(body, "imports").ok_or("import map has no \"imports\" object")?;
        for (key, value) in string_pairs(imports) {
            let target = Path::new(&value);
            let target = if target.is_absolute() { target.to_path_buf() } else { base.join(target) };
            map.insert(key, target);
        }
        Ok(map)
    }

    fn lookup(&self, specifier: &str) -> Option<PathBuf> {
        for (prefix, target) in &self.entries {
            if specifier == prefix {
                return Some(target.clone());
            }
            let with_slash = if prefix.ends_with('/') { prefix.clone() } else { format!("{prefix}/") };
            if let Some(rest) = specifier.strip_prefix(&with_slash) {
                return Some(target.join(rest));
            }
        }
        None
    }
}

/// Resolve `specifier` imported from the module named `base` (a path, or a
/// non-path name for code evaluated directly, which resolves from `cwd`).
pub fn resolve(base: &str, specifier: &str, map: &ImportMap, cwd: &Path) -> Result<String, String> {
    let base_path = Path::new(base);
    let from_dir = if base_path.is_absolute() {
        base_path.parent().map(Path::to_path_buf).unwrap_or_else(|| cwd.to_path_buf())
    } else {
        cwd.to_path_buf()
    };

    let candidate = if specifier.starts_with("./") || specifier.starts_with("../") || specifier == "." || specifier == ".." {
        from_dir.join(specifier)
    } else if Path::new(specifier).is_absolute() || specifier.starts_with('/') {
        PathBuf::from(specifier)
    } else if let Some(mapped) = map.lookup(specifier) {
        mapped
    } else {
        return node_modules(&from_dir, specifier)
            .map(|p| name_of(&p))
            .ok_or_else(|| format!("cannot find module '{specifier}' from '{base}'"));
    };

    file_or_directory(&normalise(&candidate))
        .map(|p| name_of(&p))
        .ok_or_else(|| format!("cannot find module '{specifier}' from '{base}'"))
}

fn node_modules(from: &Path, specifier: &str) -> Option<PathBuf> {
    let mut dir = Some(from.to_path_buf());
    while let Some(d) = dir {
        let candidate = d.join("node_modules").join(specifier);
        if let Some(found) = file_or_directory(&normalise(&candidate)) {
            return Some(found);
        }
        dir = d.parent().map(Path::to_path_buf);
    }
    None
}

fn file_or_directory(path: &Path) -> Option<PathBuf> {
    if path.is_file() {
        return Some(path.to_path_buf());
    }
    for ext in EXTENSIONS {
        let with = PathBuf::from(format!("{}.{ext}", path.display()));
        if with.is_file() {
            return Some(with);
        }
    }
    // A `.js` import of a `.ts` source, as TypeScript projects write them.
    if let Some(stem) = path.to_str().and_then(|s| s.strip_suffix(".js")) {
        for ext in ["ts", "tsx", "mts"] {
            let with = PathBuf::from(format!("{stem}.{ext}"));
            if with.is_file() {
                return Some(with);
            }
        }
    }
    if path.is_dir() {
        if let Ok(text) = std::fs::read_to_string(path.join("package.json")) {
            for key in ["module", "main"] {
                if let Some(entry) = find_string(&text, key) {
                    if let Some(found) = file_or_directory(&normalise(&path.join(entry))) {
                        return Some(found);
                    }
                }
            }
            if let Some(exports) = find_object(&text, "exports") {
                if let Some(dot) = find_object(exports, ".") {
                    for key in ["import", "default"] {
                        if let Some(entry) = find_string(dot, key) {
                            if let Some(found) = file_or_directory(&normalise(&path.join(entry))) {
                                return Some(found);
                            }
                        }
                    }
                } else if let Some(entry) = find_string(&text, ".") {
                    if let Some(found) = file_or_directory(&normalise(&path.join(entry))) {
                        return Some(found);
                    }
                }
            }
        }
        for index in INDEX {
            let candidate = path.join(index);
            if candidate.is_file() {
                return Some(candidate);
            }
        }
    }
    None
}

/// Remove `.` and `..` without touching the filesystem.
pub fn normalise(path: &Path) -> PathBuf {
    let mut out = PathBuf::new();
    for c in path.components() {
        match c {
            Component::CurDir => {}
            Component::ParentDir => {
                out.pop();
            }
            other => out.push(other.as_os_str()),
        }
    }
    out
}

pub fn name_of(path: &Path) -> String {
    normalise(path).to_string_lossy().replace('\\', "/")
}

// A small JSON reader for package.json and import maps: just enough to find
// string values and nested objects by key, without a JSON dependency.

fn find_value<'a>(text: &'a str, key: &str) -> Option<&'a str> {
    let needle = format!("\"{key}\"");
    let mut search = 0;
    while let Some(at) = text[search..].find(&needle) {
        let after = search + at + needle.len();
        let rest = text[after..].trim_start();
        if let Some(value) = rest.strip_prefix(':') {
            return Some(value.trim_start());
        }
        search = after;
    }
    None
}

fn find_string(text: &str, key: &str) -> Option<String> {
    let value = find_value(text, key)?;
    let body = value.strip_prefix('"')?;
    let end = body.find('"')?;
    Some(body[..end].to_string())
}

fn find_object<'a>(text: &'a str, key: &str) -> Option<&'a str> {
    let value = find_value(text, key)?;
    if !value.starts_with('{') {
        return None;
    }
    let mut depth = 0usize;
    let mut in_string = false;
    let mut escaped = false;
    for (i, ch) in value.char_indices() {
        if in_string {
            match (escaped, ch) {
                (true, _) => escaped = false,
                (false, '\\') => escaped = true,
                (false, '"') => in_string = false,
                _ => {}
            }
            continue;
        }
        match ch {
            '"' => in_string = true,
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 {
                    return Some(&value[..=i]);
                }
            }
            _ => {}
        }
    }
    None
}

fn string_pairs(object: &str) -> Vec<(String, String)> {
    let mut out = Vec::new();
    let inner = object.trim().trim_start_matches('{').trim_end_matches('}');
    let mut rest = inner;
    loop {
        let Some(start) = rest.find('"') else { break };
        let after_key = &rest[start + 1..];
        let Some(key_end) = after_key.find('"') else { break };
        let key = after_key[..key_end].to_string();
        let after = after_key[key_end + 1..].trim_start();
        let Some(after_colon) = after.strip_prefix(':') else { break };
        let after_colon = after_colon.trim_start();
        let Some(value_body) = after_colon.strip_prefix('"') else { break };
        let Some(value_end) = value_body.find('"') else { break };
        out.push((key, value_body[..value_end].to_string()));
        rest = &value_body[value_end + 1..];
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("fbw-js-resolve-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn write(path: &Path, text: &str) {
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(path, text).unwrap();
    }

    #[test]
    fn relative_imports_try_typescript_extensions_and_index_files() {
        let dir = temp("relative");
        write(&dir.join("src/a.ts"), "");
        write(&dir.join("src/lib/index.ts"), "");
        write(&dir.join("src/b.ts"), "");
        let base = name_of(&dir.join("src/main.ts"));
        let cwd = dir.clone();
        assert!(resolve(&base, "./a", &ImportMap::new(), &cwd).unwrap().ends_with("src/a.ts"));
        assert!(resolve(&base, "./lib", &ImportMap::new(), &cwd).unwrap().ends_with("src/lib/index.ts"));
        assert!(resolve(&base, "./b.js", &ImportMap::new(), &cwd).unwrap().ends_with("src/b.ts"));
        assert!(resolve(&base, "./missing", &ImportMap::new(), &cwd).is_err());
    }

    #[test]
    fn bare_imports_use_the_map_then_node_modules() {
        let dir = temp("bare");
        write(&dir.join("sdk/index.ts"), "");
        write(&dir.join("sdk/utils/math.ts"), "");
        write(
            &dir.join("node_modules/pkg/package.json"),
            r#"{ "name": "pkg", "exports": { ".": { "import": "./dist/esm.mjs" } } }"#,
        );
        write(&dir.join("node_modules/pkg/dist/esm.mjs"), "");
        let mut map = ImportMap::new();
        map.insert("@msfs/sdk", dir.join("sdk"));
        let base = name_of(&dir.join("app/main.ts"));
        assert!(resolve(&base, "@msfs/sdk", &map, &dir).unwrap().ends_with("sdk/index.ts"));
        assert!(resolve(&base, "@msfs/sdk/utils/math", &map, &dir).unwrap().ends_with("sdk/utils/math.ts"));
        assert!(resolve(&base, "pkg", &map, &dir).unwrap().ends_with("node_modules/pkg/dist/esm.mjs"));
    }

    #[test]
    fn import_maps_read_from_json_relative_to_their_folder() {
        let dir = temp("map");
        write(&dir.join("vendor/sdk.ts"), "");
        let map = ImportMap::from_json(r#"{ "imports": { "sdk": "vendor/sdk.ts" } }"#, &dir).unwrap();
        assert!(resolve("eval", "sdk", &map, &dir).unwrap().ends_with("vendor/sdk.ts"));
    }

    #[test]
    fn paths_normalise_parent_components() {
        assert_eq!(name_of(Path::new("C:/a/b/../c/./d.ts")), "C:/a/c/d.ts");
    }
}
