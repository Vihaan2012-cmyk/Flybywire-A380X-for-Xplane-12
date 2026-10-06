//! Custom CEF schemes for XPHFBW's instrument views
//! (docs/briefs/xphfbw-js-bridge.md, agent G):
//!
//! - `coui://html_ui/...` — the aircraft package's `html_ui` folder, exactly
//!   as MSFS serves it, so FlyByWire's own `coui://...` and `/VFS/...`
//!   fetches work unmodified (app/js/msfs/environment.js). A request whose
//!   path starts with `/VFS/` is served from the aircraft package root
//!   instead of `html_ui` (MSFS's own VFS mapping).
//! - `xphfbw://runtime/...` — `app/js` next to `XPHFBW.exe` (installed as
//!   `XPHFBW/js`, tools/install.sh), agent F's ported MSFS runtime.
//!
//! Both are registered once CEF's context is initialized (window.rs). Every
//! HTML response gets `RUNTIME_SCRIPT_TAG` injected as the first element of
//! `<head>` (or, for a `<head>`-less gauge fragment, right after `<html>`,
//! or at the very start of the file — the HTML5 parser's "before head"
//! insertion mode creates an implicit `<head>` and puts a leading `<script>`
//! into it in all three cases, so the script always ends up first in
//! `<head>` however the file is shaped). `msfs-runtime.js`
//! (app/js/msfs-runtime.js) expects exactly this and installs
//! `window.__xphfbw` before it, per agent E's renderer.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use cef::wrapper::stream_resource_handler::StreamResourceHandler;
use cef::*;
use fbw_a380_systems::source_patch::SourcePatch;

/// Injected as the first element of `<head>` of every `coui://` HTML
/// response (agent F's runtime; see the module doc for why this placement
/// is always correct, even for a `<head>`-less gauge fragment).
pub const RUNTIME_SCRIPT_TAG: &str = "<script src=\"xphfbw://runtime/msfs-runtime.js\"></script>";

const OPTIONS: &[SchemeOptions] = &[SchemeOptions::STANDARD, SchemeOptions::CORS_ENABLED, SchemeOptions::FETCH_ENABLED];

/// `App::on_register_custom_schemes` (window.rs): runs in every CEF process
/// (browser and every renderer/subprocess), which is required for a
/// standard scheme's URLs to parse (`location.host`/`location.pathname`)
/// the same way everywhere.
pub fn register_custom_schemes(registrar: &mut SchemeRegistrar) {
    let options = OPTIONS.iter().fold(0i32, |acc, o| acc | o.get_raw());
    registrar.add_custom_scheme(Some(&CefString::from("coui")), options);
    registrar.add_custom_scheme(Some(&CefString::from("xphfbw")), options);
}

#[derive(Clone)]
struct Roots {
    /// `<aircraft>/html_ui`.
    html_ui: PathBuf,
    /// `<aircraft>`, for `/VFS/...` requests.
    aircraft: PathBuf,
    /// `app/js`, installed as `XPHFBW/js` next to the exe.
    runtime: PathBuf,
    /// The plugin's own `SourcePatch`es that need no live simulation state
    /// (`fbw_a380_systems::static_source_patches`), applied to `coui://`
    /// responses below the same way the plugin's fallback QuickJS
    /// `Cockpit` applies them (`js/msfs/mod.rs`'s `read_file`) -- so the
    /// fixes they carry (ECAM/FWS FIXMEs, the ECL's momentary buttons, the
    /// deep-systems ECAM bridge, ...) reach the bundles XPHFBW actually
    /// draws, not only the fallback engine
    /// (`docs/deep/debug_screen_clicks.md`). Computed once in `install`,
    /// not per request.
    patches: Arc<Vec<SourcePatch>>,
}

wrap_scheme_handler_factory! {
    struct Factory {
        roots: Roots,
    }

    impl SchemeHandlerFactory {
        fn create(
            &self,
            _browser: Option<&mut Browser>,
            _frame: Option<&mut Frame>,
            scheme_name: Option<&CefString>,
            request: Option<&mut Request>,
        ) -> Option<ResourceHandler> {
            let scheme = scheme_name.map(|s| s.to_string()).unwrap_or_default();
            let request = request?;
            let url_cef = request.url();
            let url = CefString::from(&url_cef).to_string();
            let path = url_path(&url);
            let file = match scheme.as_str() {
                "coui" => resolve_coui(&self.roots, &path),
                "xphfbw" => resolve_under(&self.roots.runtime, &path),
                _ => None,
            };
            // Only `coui://` serves FlyByWire's own bundles; `xphfbw://`
            // serves this app's own runtime shim, which nothing patches.
            let patches: &[SourcePatch] = if scheme == "coui" { self.roots.patches.as_slice() } else { &[] };
            Some(serve(file, patches, &path))
        }
    }
}

/// Registers both schemes' handlers, once CEF's context is initialized
/// (window.rs `on_context_initialized`). `aircraft_root` is the `--aircraft=`
/// folder (panel/, html_ui/); `runtime_dir` is `app/js` next to the exe.
pub fn install(aircraft_root: PathBuf, runtime_dir: PathBuf) {
    let patches = Arc::new(fbw_a380_systems::static_source_patches());
    let roots = Roots { html_ui: aircraft_root.join("html_ui"), aircraft: aircraft_root, runtime: runtime_dir, patches };
    let mut factory = Factory::new(roots);
    register_scheme_handler_factory(Some(&CefString::from("coui")), None, Some(&mut factory));
    register_scheme_handler_factory(Some(&CefString::from("xphfbw")), None, Some(&mut factory));
}

// ---------------------------------------------------------------------------
// URL and path resolution.
// ---------------------------------------------------------------------------

/// The percent-decoded path (with its leading `/`), query and fragment
/// dropped, from a `scheme://host/path` URL.
fn url_path(url: &str) -> String {
    let rest = url.split_once("://").map(|(_, r)| r).unwrap_or(url);
    let rest = rest.split(['?', '#']).next().unwrap_or("");
    let path = match rest.split_once('/') {
        Some((_host, p)) => format!("/{p}"),
        None => String::new(),
    };
    percent_decode(&path)
}

fn percent_decode(s: &str) -> String {
    let bytes = s.as_bytes();
    let mut out = Vec::with_capacity(bytes.len());
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'%' && i + 3 <= bytes.len() {
            if let Ok(v) = u8::from_str_radix(std::str::from_utf8(&bytes[i + 1..i + 3]).unwrap_or(""), 16) {
                out.push(v);
                i += 3;
                continue;
            }
        }
        out.push(bytes[i]);
        i += 1;
    }
    String::from_utf8_lossy(&out).into_owned()
}

/// `path` under `base`, rejecting any `..` segment (no escaping `base`).
/// `None` for an empty `path` too (nothing to serve at the root itself).
fn safe_join(base: &Path, path: &str) -> Option<PathBuf> {
    let mut out = base.to_path_buf();
    let mut any = false;
    for seg in path.split('/') {
        match seg {
            "" | "." => continue,
            ".." => return None,
            s => {
                out.push(s);
                any = true;
            }
        }
    }
    any.then_some(out)
}

fn resolve_coui(roots: &Roots, path: &str) -> Option<PathBuf> {
    match path.strip_prefix("/VFS/") {
        Some(rest) => safe_join(&roots.aircraft, rest),
        None => safe_join(&roots.html_ui, path.trim_start_matches('/')),
    }
}

fn resolve_under(base: &Path, path: &str) -> Option<PathBuf> {
    safe_join(base, path.trim_start_matches('/'))
}

// ---------------------------------------------------------------------------
// Serving a file (or a 404).
// ---------------------------------------------------------------------------

// W142 note (integration adaptation): this report's own EDIT13/EDIT15 were
// written against a pre-W02 `serve(file)`/`not_found()` (no path/url
// argument at all). W02 landed first in this batch and added a `url`/
// `clean` string to both, for its own 404 log line. Rather than carry two
// separate strings (one for W02's log, one for W142's patch match) through
// every call site, this merges them onto the one VFS-relative `clean` path
// the call site already resolves (`url_path`) -- it serves W02's log
// exactly as well as the full URL would (it is the same request, just
// without the scheme/host), and it is the very string `SourcePatch::path`
// needs to match against.
fn serve(file: Option<PathBuf>, patches: &[SourcePatch], clean: &str) -> ResourceHandler {
    let Some(path) = file.filter(|p| p.is_file()) else {
        return not_found(clean);
    };
    let Ok(bytes) = std::fs::read(&path) else {
        return not_found(clean);
    };
    let mime = mime_of(&path);
    let mut bytes = if mime == "text/html" || mime == "text/javascript" {
        match String::from_utf8(bytes) {
            Ok(text) => {
                let text = fbw_a380_systems::source_patch::apply(patches, clean, text, |ok, msg| {
                    crate::logging::log(&format!("coui: {}{msg}", if ok { "" } else { "ERROR: " }));
                });
                if mime == "text/html" { inject_runtime_script(&text).into_bytes() } else { text.into_bytes() }
            }
            Err(e) => e.into_bytes(),
        }
    } else {
        bytes
    };
    let Some(stream) = stream_reader_create_for_data(bytes.as_mut_ptr(), bytes.len()) else {
        return not_found(clean);
    };
    StreamResourceHandler::new_with_stream(mime.to_string(), stream)
}

/// A missing `coui://`/`xphfbw://` file used to 404 with nothing in the log
/// to say which path was missing: today's lost `html_ui/Pages`+`JS` folder
/// (S06) 404'd every gauge and left every screen simply black. Capped: one
/// missing folder 404s every request a page makes (its HTML, then every
/// `<script>`/`<link>`/`<img>` it references), not just the page's own load
/// (that's `ViewLoad::on_load_end`, `app/src/views.rs`).
const NOT_FOUND_LOG_CAP: u32 = 50;

/// Whether the `n`th (1-based) 404 since this scheme handler was installed
/// should still be logged, split out so the cap is testable without CEF.
fn should_log_not_found(n: u32) -> bool {
    n <= NOT_FOUND_LOG_CAP
}

fn not_found(url: &str) -> ResourceHandler {
    static COUNT: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
    let n = COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed) + 1;
    if should_log_not_found(n) {
        crate::logging::log(&format!("scheme: 404 {url}"));
    }
    StreamResourceHandler::new(404, "Not Found".to_string(), "text/plain".to_string(), None, None)
}

fn mime_of(path: &Path) -> &'static str {
    let ext = path.extension().and_then(|e| e.to_str()).unwrap_or("").to_ascii_lowercase();
    match ext.as_str() {
        "html" | "htm" => "text/html",
        "js" | "mjs" => "text/javascript",
        "css" => "text/css",
        "json" => "application/json",
        "png" => "image/png",
        "jpg" | "jpeg" => "image/jpeg",
        "gif" => "image/gif",
        "svg" => "image/svg+xml",
        "webp" => "image/webp",
        "woff" => "font/woff",
        "woff2" => "font/woff2",
        "ttf" => "font/ttf",
        "otf" => "font/otf",
        "wasm" => "application/wasm",
        "xml" => "application/xml",
        _ => "application/octet-stream",
    }
}

// ---------------------------------------------------------------------------
// HTML injection.
// ---------------------------------------------------------------------------

/// `RUNTIME_SCRIPT_TAG` as the first element of `<head>` (see the module
/// doc for why the fallbacks below are equivalent for a browser's parser).
pub fn inject_runtime_script(html: &str) -> String {
    if let Some(at) = tag_end(html, "head").or_else(|| tag_end(html, "html")) {
        let mut out = String::with_capacity(html.len() + RUNTIME_SCRIPT_TAG.len());
        out.push_str(&html[..at]);
        out.push_str(RUNTIME_SCRIPT_TAG);
        out.push_str(&html[at..]);
        return out;
    }
    format!("{RUNTIME_SCRIPT_TAG}{html}")
}

/// The byte offset right after the first `<name ...>` tag's closing `>`
/// (case-insensitive, not matching a longer tag name like `<header>` for
/// `name = "head"`).
fn tag_end(html: &str, name: &str) -> Option<usize> {
    let lower = html.to_ascii_lowercase();
    let open = format!("<{name}");
    let mut from = 0;
    while let Some(rel) = lower[from..].find(open.as_str()) {
        let start = from + rel;
        let after = start + open.len();
        match lower.as_bytes().get(after) {
            Some(b'>' | b' ' | b'\t' | b'\n' | b'\r' | b'/') => return lower[after..].find('>').map(|gt| after + gt + 1),
            _ => from = after,
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn injects_into_an_existing_head() {
        let html = "<html><head><title>x</title></head><body></body></html>";
        let got = inject_runtime_script(html);
        assert_eq!(got, format!("<html><head>{RUNTIME_SCRIPT_TAG}<title>x</title></head><body></body></html>"));
    }

    #[test]
    fn injects_into_a_head_with_attributes() {
        let html = "<html>\n<head lang=\"en\">\n<title>x</title></head></html>";
        let got = inject_runtime_script(html);
        assert!(got.starts_with(&format!("<html>\n<head lang=\"en\">{RUNTIME_SCRIPT_TAG}\n")), "{got}");
    }

    #[test]
    fn does_not_confuse_head_with_header() {
        let html = "<html><header>not head</header><head>real</head></html>";
        let got = inject_runtime_script(html);
        let head_pos = got.find("<head>").unwrap();
        let script_pos = got.find(RUNTIME_SCRIPT_TAG).unwrap();
        assert!(script_pos > head_pos, "the script must land in <head>, not <header>: {got}");
    }

    #[test]
    fn falls_back_to_right_after_html_when_there_is_no_head() {
        let html = "<html><body>no head here</body></html>";
        let got = inject_runtime_script(html);
        assert_eq!(got, format!("<html>{RUNTIME_SCRIPT_TAG}<body>no head here</body></html>"));
    }

    #[test]
    fn falls_back_to_the_very_start_for_a_headless_html_less_gauge_fragment() {
        // A380X/PFD/pfd.html has none of <html>/<head>/<body>: just
        // <script type="text/html">/<link>/import-script fragments.
        let html = "<script type=\"text/html\" id=\"A380X_PFD\">...</script>\n<link rel=\"stylesheet\" href=\"x.css\" />\n";
        let got = inject_runtime_script(html);
        assert!(got.starts_with(RUNTIME_SCRIPT_TAG), "{got}");
        assert_eq!(got, format!("{RUNTIME_SCRIPT_TAG}{html}"));
    }

    #[test]
    fn percent_decode_handles_spaces_and_literal_percents() {
        assert_eq!(percent_decode("/A%20B/C%2Fx"), "/A B/C/x");
        assert_eq!(percent_decode("/no-escapes"), "/no-escapes");
        assert_eq!(percent_decode("/trailing%"), "/trailing%");
    }

    #[test]
    fn url_path_strips_scheme_host_query_and_fragment() {
        assert_eq!(url_path("coui://html_ui/Pages/x.html?a=1#b"), "/Pages/x.html");
        assert_eq!(url_path("xphfbw://runtime/msfs-runtime.js"), "/msfs-runtime.js");
        assert_eq!(url_path("coui://html_ui"), "");
    }

    #[test]
    fn resolve_coui_maps_vfs_to_the_aircraft_root_and_everything_else_under_html_ui() {
        let roots = Roots { html_ui: PathBuf::from("/ac/html_ui"), aircraft: PathBuf::from("/ac"), runtime: PathBuf::from("/exe/js"), patches: Arc::new(Vec::new()) };
        assert_eq!(resolve_coui(&roots, "/Pages/VCockpit/x.html"), Some(PathBuf::from("/ac/html_ui/Pages/VCockpit/x.html")));
        assert_eq!(resolve_coui(&roots, "/VFS/currentflight.json"), Some(PathBuf::from("/ac/currentflight.json")));
    }

    #[test]
    fn safe_join_rejects_escaping_the_root() {
        let base = Path::new("/ac/html_ui");
        assert_eq!(safe_join(base, "../../secret.txt"), None);
        assert_eq!(safe_join(base, "a/../../b"), None);
        assert_eq!(safe_join(base, ""), None);
        assert_eq!(safe_join(base, "Pages/a.html"), Some(PathBuf::from("/ac/html_ui/Pages/a.html")));
    }

    #[test]
    fn not_found_logging_is_capped() {
        assert!(should_log_not_found(1));
        assert!(should_log_not_found(NOT_FOUND_LOG_CAP));
        assert!(!should_log_not_found(NOT_FOUND_LOG_CAP + 1));
    }
}
