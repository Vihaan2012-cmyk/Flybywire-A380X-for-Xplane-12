//! TypeScript and JSX to JavaScript, with Oxc.
//!
//! Types, interfaces, enums, namespaces, parameter properties and type-only
//! imports are removed or lowered; JSX becomes calls to a configurable
//! factory. MSFS's avionics framework builds components with
//! `FSComponent.buildComponent` and `FSComponent.Fragment`, which is the
//! default here. Everything else is left at ES2022, which QuickJS runs as is.

use std::path::Path;

use oxc::allocator::Allocator;
use oxc::codegen::Codegen;
use oxc::parser::Parser;
use oxc::semantic::SemanticBuilder;
use oxc::span::SourceType;
use oxc::transformer::{HelperLoaderMode, JsxRuntime, TransformOptions, Transformer};

/// The JSX factory calls JSX compiles to.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JsxConfig {
    pub pragma: String,
    pub pragma_frag: String,
}

impl Default for JsxConfig {
    fn default() -> Self {
        Self {
            pragma: "FSComponent.buildComponent".into(),
            pragma_frag: "FSComponent.Fragment".into(),
        }
    }
}

/// Whether a file needs transpiling before QuickJS can run it.
pub fn needs_transpile(path: &Path) -> bool {
    matches!(
        path.extension().and_then(|e| e.to_str()).map(str::to_ascii_lowercase).as_deref(),
        Some("ts" | "tsx" | "mts" | "cts" | "jsx")
    )
}

/// Transpile one file's source. `path` decides the dialect (.ts, .tsx,
/// .mts, .jsx); a path with another extension is read as TypeScript.
pub fn transpile(path: &Path, source: &str, jsx: &JsxConfig) -> Result<String, String> {
    let allocator = Allocator::default();
    let source_type = SourceType::from_path(path).unwrap_or_else(|_| SourceType::ts());
    let parsed = Parser::new(&allocator, source, source_type).parse();
    if !parsed.diagnostics.is_empty() {
        return Err(render(path, source, parsed.diagnostics));
    }
    let mut program = parsed.program;

    let semantic = SemanticBuilder::new().with_excess_capacity(2.0).with_enum_eval(true).build(&program);
    if !semantic.diagnostics.is_empty() {
        return Err(render(path, source, semantic.diagnostics));
    }
    let scoping = semantic.semantic.into_scoping();

    let mut options = TransformOptions::from_target("es2022").map_err(|e| e.to_string())?;
    options.jsx.runtime = JsxRuntime::Classic;
    options.jsx.pragma = Some(jsx.pragma.clone());
    options.jsx.pragma_frag = Some(jsx.pragma_frag.clone());
    options.typescript.jsx_pragma = jsx.pragma.clone().into();
    options.typescript.jsx_pragma_frag = jsx.pragma_frag.clone().into();
    // Helpers, when a transform needs one, come from a global rather than an
    // import this loader could not resolve.
    options.helper_loader.mode = HelperLoaderMode::External;

    let transformed = Transformer::new(&allocator, path, &options).build_with_scoping(scoping, &mut program);
    if !transformed.diagnostics.is_empty() {
        return Err(render(path, source, transformed.diagnostics));
    }
    Ok(Codegen::new().build(&program).code)
}

fn render<D>(path: &Path, source: &str, diagnostics: D) -> String
where
    D: IntoIterator<Item = oxc::diagnostics::OxcDiagnostic>,
{
    let mut out = format!("{}:\n", path.display());
    for d in diagnostics {
        out.push_str(&format!("{}\n", d.render_with_source_code(source.to_string())));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn types_are_removed_and_enums_lowered() {
        let js = transpile(
            Path::new("a.ts"),
            "interface P { x: number }\nenum Mode { Off, On = 4 }\nexport const f = (p: P): number => p.x + Mode.On;",
            &JsxConfig::default(),
        )
        .unwrap();
        assert!(!js.contains("interface"));
        assert!(!js.contains(": number"));
        assert!(js.contains("Mode"));
        assert!(js.contains("export const f"));
    }

    #[test]
    fn jsx_uses_the_avionics_framework_factory() {
        let js = transpile(Path::new("a.tsx"), "const e = <div class='x'><>hi</></div>;", &JsxConfig::default()).unwrap();
        assert!(js.contains("FSComponent.buildComponent"), "{js}");
        assert!(js.contains("FSComponent.Fragment"), "{js}");
    }

    #[test]
    fn syntax_errors_name_the_file() {
        let err = transpile(Path::new("broken.ts"), "let x: = 3;", &JsxConfig::default()).unwrap_err();
        assert!(err.contains("broken.ts"), "{err}");
    }

    #[test]
    fn which_files_need_transpiling() {
        assert!(needs_transpile(Path::new("a.tsx")));
        assert!(needs_transpile(Path::new("a.MTS")));
        assert!(!needs_transpile(Path::new("a.mjs")));
    }
}
