//! MSFS model behaviour files as owned element trees, with their includes
//! followed and their templates collected by name.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use anyhow::{anyhow, Context};

/// One XML element: its name, attributes, child elements and own text.
#[derive(Clone, Debug, Default)]
pub struct El {
    pub name: String,
    pub attrs: Vec<(String, String)>,
    pub kids: Vec<El>,
    /// The element's direct text, joined (a parameter's value).
    pub text: String,
}

impl El {
    pub fn attr(&self, name: &str) -> Option<&str> {
        self.attrs.iter().find(|(k, _)| k.eq_ignore_ascii_case(name)).map(|(_, v)| v.as_str())
    }

    fn from_node(n: roxmltree::Node) -> El {
        let mut el = El {
            name: n.tag_name().name().replace(NAME_HASH, "#"),
            attrs: n.attributes().map(|a| (a.name().to_string(), a.value().to_string())).collect(),
            ..Default::default()
        };
        for c in n.children() {
            if c.is_element() {
                el.kids.push(El::from_node(c));
            } else if c.is_text() {
                el.text.push_str(c.text().unwrap_or(""));
            }
        }
        el
    }
}

/// Stands for `#` in element names while parsing: MSFS builds parameter
/// names from parameters (`<STR_STATE_#POS_ADF#>`, `<#PARAM_NAME#>`), which
/// XML does not allow.
const NAME_HASH: char = '\u{2C00}';

/// `#` inside element names replaced by [`NAME_HASH`].
fn escape_tag_names(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    let mut in_name = false;
    let mut prev = '\0';
    for c in text.chars() {
        if in_name {
            if c.is_whitespace() || c == '>' || (c == '/' && prev != '<') {
                in_name = false;
            }
        } else if prev == '<' && c != '!' && c != '?' {
            in_name = true;
        }
        out.push(if in_name && c == '#' { NAME_HASH } else { c });
        prev = c;
    }
    out
}

/// Closing tags renamed to the element they close: MSFS's own files close
/// some elements with a different name (`<BINDING_SET_#X#_PARAM_0>` ..
/// `</BINDING_SET_0_PARAM_#X#_PARAM_0>`, Inputs/Templates.xml), which MSFS
/// reads as closing the open element.
fn match_close_tags(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    let mut open: Vec<String> = Vec::new();
    let mut rest = text;
    while let Some(i) = rest.find('<') {
        out.push_str(&rest[..i]);
        let tag = &rest[i..];
        let Some(end) = tag.find('>') else {
            out.push_str(tag);
            return out;
        };
        let inner = &tag[1..end];
        if let Some(name) = inner.strip_prefix('/') {
            match open.pop() {
                Some(o) if o != name.trim() => out.push_str(&format!("</{o}>")),
                _ => out.push_str(&tag[..=end]),
            }
        } else {
            if !inner.starts_with('?') && !inner.starts_with('!') && !inner.ends_with('/') {
                let name: String = inner.chars().take_while(|c| !c.is_whitespace()).collect();
                open.push(name);
            }
            out.push_str(&tag[..=end]);
        }
        rest = &tag[end + 1..];
    }
    out.push_str(rest);
    out
}

/// Parse XML text into an element tree.
pub fn parse(text: &str) -> anyhow::Result<El> {
    // MSFS reads files roxmltree calls malformed: a DOCTYPE-less BOM and "--"
    // inside comments. Comments carry nothing a template needs.
    let text = text.trim_start_matches('\u{feff}');
    let cleaned = match_close_tags(&escape_tag_names(&strip_comments(text)));
    let opts = roxmltree::ParsingOptions { allow_dtd: true, ..Default::default() };
    let doc = roxmltree::Document::parse_with_options(&cleaned, opts).map_err(|e| anyhow!("{e}"))?;
    Ok(El::from_node(doc.root_element()))
}

fn strip_comments(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    let mut rest = text;
    while let Some(i) = rest.find("<!--") {
        out.push_str(&rest[..i]);
        match rest[i + 4..].find("-->") {
            Some(j) => rest = &rest[i + 4 + j + 3..],
            None => {
                rest = "";
            }
        }
    }
    out.push_str(rest);
    out
}

/// Every template and top-level component of a model's behaviours.
#[derive(Default)]
pub struct Library {
    pub templates: HashMap<String, El>,
    /// Top-level components, in file order.
    pub components: Vec<El>,
    pub files: Vec<PathBuf>,
    /// Includes that point outside the package (Asobo's own definitions).
    pub missing: Vec<String>,
    /// Templates defined more than once (the first definition is kept).
    pub duplicates: Vec<String>,
    /// `<Macro Name="X">value</Macro>`: `@X` in code stands for the value.
    pub macros: HashMap<String, String>,
}

/// `@NAME` replaced by its macro's value.
pub fn expand_macros(text: &str, macros: &HashMap<String, String>) -> String {
    if macros.is_empty() || !text.contains('@') {
        return text.to_string();
    }
    let mut out = String::with_capacity(text.len());
    let mut rest = text;
    while let Some(i) = rest.find('@') {
        out.push_str(&rest[..i]);
        let after = &rest[i + 1..];
        let end = after.find(|c: char| !(c.is_ascii_alphanumeric() || c == '_')).unwrap_or(after.len());
        match macros.get(&after[..end]) {
            Some(v) if end > 0 => out.push_str(v),
            _ => out.push_str(&rest[i..i + 1 + end]),
        }
        rest = &after[end..];
    }
    out.push_str(rest);
    out
}

impl Library {
    /// Load a model XML (`<ModelInfo>` with `<Behaviors>`, or a
    /// `<ModelBehaviors>` file) and everything it includes. Includes by
    /// `ModelBehaviorFile` or `Path` are looked up in the nearest
    /// `ModelBehaviorDefs` folder above the file.
    pub fn load(root: &Path) -> anyhow::Result<Library> {
        let mut lib = Library::default();
        let defs = root.ancestors().map(|a| a.join("ModelBehaviorDefs")).find(|d| d.is_dir());
        lib.load_file(root, defs.as_deref())?;
        Ok(lib)
    }

    /// Every template of every XML under a folder: MSFS's own definitions
    /// (`fs-base-aircraft-common/ModelBehaviorDefs`). Files that do not
    /// parse are counted in `missing`.
    pub fn load_templates(dir: &Path) -> Library {
        let mut lib = Library::default();
        let mut files: Vec<PathBuf> = walkdir::WalkDir::new(dir)
            .into_iter()
            .filter_map(Result::ok)
            .map(|e| e.into_path())
            .filter(|p| p.extension().is_some_and(|x| x.eq_ignore_ascii_case("xml")))
            .collect();
        files.sort();
        for f in files {
            let Ok(bytes) = std::fs::read(&f) else { continue };
            match parse(&String::from_utf8_lossy(&bytes)) {
                Ok(el) => {
                    lib.take_templates(&el);
                    lib.files.push(f);
                }
                Err(e) => lib.missing.push(format!("{}: {e}", f.display())),
            }
        }
        lib
    }

    /// Templates from text, for tests.
    #[cfg(test)]
    pub fn templates_from_text(text: &str) -> Library {
        let mut lib = Library::default();
        lib.take_templates(&parse(text).unwrap());
        lib
    }

    fn take_templates(&mut self, el: &El) {
        for k in &el.kids {
            if k.name == "Template" {
                if let Some(name) = k.attr("Name") {
                    let name = name.trim().to_string();
                    if self.templates.contains_key(&name) {
                        self.duplicates.push(name);
                    } else {
                        self.templates.insert(name, k.clone());
                    }
                }
            } else if matches!(k.name.as_str(), "ModelBehaviors" | "Behaviors") {
                self.take_templates(k);
            }
        }
    }

    /// Load from text, for tests: no includes are followed.
    #[cfg(test)]
    pub fn from_text(text: &str) -> anyhow::Result<Library> {
        let mut lib = Library::default();
        let el = parse(text)?;
        lib.take(&el, Path::new("."), None)?;
        Ok(lib)
    }

    fn load_file(&mut self, path: &Path, defs: Option<&Path>) -> anyhow::Result<()> {
        let canon = path.canonicalize().unwrap_or_else(|_| path.to_path_buf());
        if self.files.contains(&canon) {
            return Ok(());
        }
        self.files.push(canon);
        let bytes = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
        let text = String::from_utf8_lossy(&bytes);
        let el = parse(&text).with_context(|| format!("parsing {}", path.display()))?;
        let dir = path.parent().unwrap_or(Path::new("."));
        self.take(&el, dir, defs)
    }

    fn take(&mut self, el: &El, dir: &Path, defs: Option<&Path>) -> anyhow::Result<()> {
        for k in &el.kids {
            match k.name.as_str() {
                "Behaviors" | "ModelBehaviors" => self.take(k, dir, defs)?,
                "Include" | "IncludeBase" => {
                    let rel = |s: &str| s.replace('\\', "/");
                    let target = if let Some(f) = k.attr("RelativeFile") {
                        Some(dir.join(rel(f)))
                    } else if let Some(f) = k.attr("ModelBehaviorFile").or_else(|| k.attr("Path")) {
                        defs.map(|d| d.join(rel(f))).filter(|p| p.is_file()).or_else(|| {
                            self.missing.push(f.to_string());
                            None
                        })
                    } else {
                        None
                    };
                    if let Some(t) = target {
                        if t.is_file() {
                            self.load_file(&t, defs)?;
                        } else {
                            self.missing.push(t.display().to_string());
                        }
                    }
                }
                "Template" => {
                    if let Some(name) = k.attr("Name") {
                        let name = name.trim().to_string();
                        if self.templates.contains_key(&name) {
                            self.duplicates.push(name);
                        } else {
                            self.templates.insert(name, k.clone());
                        }
                    }
                }
                "Component" => self.components.push(k.clone()),
                "Macro" => {
                    if let Some(name) = k.attr("Name") {
                        self.macros.entry(name.trim().to_string()).or_insert_with(|| k.text.trim().to_string());
                    }
                }
                _ => {}
            }
        }
        Ok(())
    }
}
