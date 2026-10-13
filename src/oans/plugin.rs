//! Wires [`super::AmdbProvider`] to the requests FlyByWire's scripts make,
//! and the [`SourcePatch`]es that get those requests made through `fetch`
//! in the first place (see `mod.rs`'s module docs for why the patch is
//! needed).
//!
//! [`oans_request`] is called from `js_bridge.rs`'s `direct_call`
//! (`simbridge_fetch`), which runs on the instruments' own worker thread
//! (`js_worker.rs` `WorkerHost::call`'s doc comment: "SimBridge's address
//! and map data answer here (both thread-safe)"). It is written to match:
//! no `thread_local!`, no shared state beyond the bridge's health and
//! [`super::hf`]'s downloads, both behind a `Mutex` (every call re-reads
//! [`data_root`] and re-opens the airport file it needs; the data folder
//! is small and this is a user-driven search/select action, not a per-frame
//! one), and no XPLM call -- [`data_root`] is a path relative to X-Plane's
//! own working directory, the same convention `efb::settings_path` and
//! `js_bridge::DATASTORE` already use.

use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::time::{Duration, Instant};

use super::{hf, AmdbProvider};

/// Navigraph's AMDB API host, exactly as `amdb.ts` builds its URLs.
const HOST: &str = "https://amdb.api.navigraph.com";

/// Where the local AMDB data folder is: `Output/fbw-a380x/amdb/`, or
/// whatever `Output/preferences/fbw_a380x_oans.ini`'s `data_dir=` line
/// says (docs/oans.md). Both are relative to X-Plane's own root, X-Plane's
/// plugin process working directory.
fn data_root() -> PathBuf {
    const DEFAULT: &str = "Output/fbw-a380x/amdb";
    const CONFIG: &str = "Output/preferences/fbw_a380x_oans.ini";
    let text = std::fs::read_to_string(CONFIG).unwrap_or_default();
    configured_data_dir(&text).unwrap_or_else(|| PathBuf::from(DEFAULT))
}

const DEFAULT_BRIDGE: &str = "http://127.0.0.1:8770";

fn configured_bridge(ini_text: &str) -> Option<String> {
    match crate::efb::parse_ini(ini_text).get("bridge").map(|b| b.trim().trim_end_matches('/').to_string()) {
        Some(b) if b.eq_ignore_ascii_case("off") || b.is_empty() => None,
        Some(b) => Some(b),
        None => Some(DEFAULT_BRIDGE.to_string()),
    }
}

pub fn bridge_base() -> Option<String> {
    configured_bridge(&std::fs::read_to_string("Output/preferences/fbw_a380x_oans.ini").unwrap_or_default())
}

const CHARTS_PATH: &str = "/efb/flybywire-aircraft-a380-842/v2/charts";
const BUILD_MS: u32 = 90_000;
const HEALTH_MS: u32 = 5_000;
const UP_FOR: Duration = Duration::from_secs(60);
const DOWN_FOR: Duration = Duration::from_secs(20);

#[derive(Clone, Copy, Debug, PartialEq)]
enum Health {
    Checking,
    Up,
    Down,
}

static HEALTH: Mutex<Option<(String, bool, Instant)>> = Mutex::new(None);

fn remember(base: &str, up: bool) {
    crate::diag::event("amdb", &format!("AMDB Bridge at {base} is {}", if up { "up" } else { "down" }));
    *HEALTH.lock().unwrap_or_else(std::sync::PoisonError::into_inner) = Some((base.to_string(), up, Instant::now()));
}

fn known_health(base: &str) -> Option<bool> {
    let guard = HEALTH.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
    let (b, up, at) = guard.as_ref()?;
    (b == base && at.elapsed() < if *up { UP_FOR } else { DOWN_FOR }).then_some(*up)
}

fn is_bridge(status: u16, body: &str) -> bool {
    status == 200 && serde_json::from_str::<serde_json::Value>(body).is_ok_and(|v| v["service"] == "amdb-bridge")
}

fn bridge_health(base: &str) -> Health {
    if let Some(up) = known_health(base) {
        return if up { Health::Up } else { Health::Down };
    }
    match crate::net::poll_with(&format!("amdb-bridge-health {base}"), "GET", &format!("{base}/v1"), "", HEALTH_MS) {
        None => Health::Checking,
        Some((status, body)) => {
            let up = is_bridge(status, &body);
            remember(base, up);
            if up {
                Health::Up
            } else {
                Health::Down
            }
        }
    }
}

fn bridge_health_blocking(base: &str) -> bool {
    if let Some(up) = known_health(base) {
        return up;
    }
    let up = crate::net::request_with("GET", &format!("{base}/v1"), "", HEALTH_MS).is_some_and(|(s, b)| is_bridge(s, &b));
    remember(base, up);
    up
}

pub fn charts_index(icao: &str) -> (u16, String) {
    let icao: String = icao.chars().filter(|c| c.is_ascii_alphanumeric()).take(4).collect::<String>().to_ascii_uppercase();
    if icao.len() != 4 {
        return (400, serde_json::json!({ "error": "an ICAO code has four letters" }).to_string());
    }
    let Some(base) = bridge_base() else {
        return (503, serde_json::json!({ "error": "the AMDB Bridge is turned off in fbw_a380x_oans.ini" }).to_string());
    };
    let not_running = || {
        let error = format!("charts need the AMDB Bridge, which is not running at {base}: start it with amdb-bridge serve --xplane");
        (503, serde_json::json!({ "error": error }).to_string())
    };
    if !bridge_health_blocking(&base) {
        return not_running();
    }
    match crate::net::request_with("GET", &format!("{base}{CHARTS_PATH}/{icao}"), "", BUILD_MS) {
        Some((status, body)) if status != 0 => (status, body),
        _ => {
            remember(&base, false);
            not_running()
        }
    }
}

/// `None`: not an AMDB request. `Some(None)`: still waiting. `Some(Some(_))`: the answer.
pub fn amdb_fetch(method: &str, url: &str, key: &str) -> Option<Option<(u16, String)>> {
    let answer = amdb_fetch_inner(method, url, key);
    if let Some(Some((status, body))) = &answer {
        crate::diag::event("amdb", &format!("{} -> {status} ({} bytes)", url.strip_prefix(HOST).unwrap_or(url), body.len()));
    }
    answer
}

fn amdb_fetch_inner(method: &str, url: &str, key: &str) -> Option<Option<(u16, String)>> {
    let rest = url.strip_prefix(HOST)?;
    if !method.is_empty() && !method.eq_ignore_ascii_case("GET") {
        return Some(Some((404, String::new())));
    }
    if let Some(base) = bridge_base() {
        match bridge_health(&base) {
            Health::Checking => return Some(None),
            Health::Up => match crate::net::poll_with(&format!("amdb-bridge {key}"), "GET", &format!("{base}{rest}"), "", BUILD_MS) {
                None => return Some(None),
                Some((status, body)) if (200..300).contains(&status) => return Some(Some((status, body))),
                Some((0, _)) => remember(&base, false),
                Some(_) => {}
            },
            Health::Down => {}
        }
    }
    let root = data_root();
    let (path, query) = rest.split_once('?').unwrap_or((rest, ""));
    if path == "/v1/search" {
        let q = parse_query(query).get("q").cloned().unwrap_or_default();
        let Some(index) = hf::search_index(&root, &q) else { return Some(None) };
        let mut results = AmdbProvider::new(root).search(&q);
        let local: std::collections::HashSet<String> = results.iter().filter_map(|r| r["idarpt"].as_str().map(str::to_string)).collect();
        results.extend(index.into_iter().filter(|e| hf::matches(e, &q) && e["idarpt"].as_str().is_some_and(|i| !local.contains(i))));
        return Some(Some((200, serde_json::Value::Array(results).to_string())));
    }
    if let Some(icao) = path.strip_prefix("/v1/").filter(|s| !s.is_empty() && !s.contains('/')) {
        if hf::airport(&root, icao) == hf::Fetch::Pending {
            return Some(None);
        }
    }
    Some(answer(method, url, "", &root))
}

/// The ini text's `data_dir=` line, if it has one and it is not empty.
/// Kept apart from [`data_root`] so it can be tested without touching the
/// process's current directory (shared, so a test changing it would be
/// unsafe alongside the rest of the suite).
fn configured_data_dir(ini_text: &str) -> Option<PathBuf> {
    crate::efb::parse_ini(ini_text).get("data_dir").filter(|d| !d.is_empty()).map(PathBuf::from)
}

/// Answers a `fetch(method, url, body)` call from the scripts if `url` is
/// Navigraph's AMDB API: `(status, body)`, in the same shape
/// `js_bridge.rs`'s `simbridge_request` answers SimBridge's. `None` if the
/// URL is not this API, so the caller falls through to its other answers.
pub fn oans_request(method: &str, url: &str, body: &str) -> Option<(u16, String)> {
    answer(method, url, body, &data_root())
}

/// [`oans_request`]'s logic with the data folder passed in, so tests can
/// point it at a scratch folder instead of `data_root`'s fixed, relative
/// path.
fn answer(method: &str, url: &str, _body: &str, root: &Path) -> Option<(u16, String)> {
    let path = url.strip_prefix(HOST)?;
    if !method.is_empty() && !method.eq_ignore_ascii_case("GET") {
        return Some((404, String::new()));
    }
    let (path, query) = path.split_once('?').unwrap_or((path, ""));
    let params = parse_query(query);
    let provider = AmdbProvider::new(root.to_path_buf());

    if path == "/v1/search" {
        let q = params.get("q").cloned().unwrap_or_default();
        let results = provider.search(&q);
        return Some((200, serde_json::Value::Array(results).to_string()));
    }

    let icao = path.strip_prefix("/v1/").filter(|s| !s.is_empty())?;
    let projection = params.get("projection").cloned().unwrap_or_else(|| "NAVIGRAPH:ARP_AZEQ".to_string());
    let include = csv(params.get("include"));
    let exclude = csv(params.get("exclude"));
    match provider.airport_data(icao, &projection, &include, &exclude) {
        Some(data) => Some((200, data.to_string())),
        None => Some((404, String::new())),
    }
}

fn csv(value: Option<&String>) -> Vec<String> {
    value.map(|v| v.split(',').filter(|s| !s.is_empty()).map(str::to_string).collect()).unwrap_or_default()
}

fn parse_query(query: &str) -> std::collections::HashMap<String, String> {
    query
        .split('&')
        .filter(|p| !p.is_empty())
        .map(|pair| {
            let (k, v) = pair.split_once('=').unwrap_or((pair, ""));
            (percent_decode(k), percent_decode(v))
        })
        .collect()
}

/// `application/x-www-form-urlencoded`-style decoding (`+` is a space,
/// `%XX` a byte). `amdb.ts` never actually calls `encodeURIComponent` on
/// these values, so this is mostly defensive; it is still correct if a
/// future FlyByWire build starts doing so.
fn percent_decode(s: &str) -> String {
    let bytes = s.as_bytes();
    let mut out = Vec::with_capacity(bytes.len());
    let mut i = 0;
    while i < bytes.len() {
        match bytes[i] {
            b'+' => {
                out.push(b' ');
                i += 1;
            }
            b'%' if i + 3 <= bytes.len() => match u8::from_str_radix(std::str::from_utf8(&bytes[i + 1..i + 3]).unwrap_or(""), 16) {
                Ok(byte) => {
                    out.push(byte);
                    i += 3;
                }
                Err(_) => {
                    out.push(bytes[i]);
                    i += 1;
                }
            },
            b => {
                out.push(b);
                i += 1;
            }
        }
    }
    String::from_utf8_lossy(&out).into_owned()
}

/// The two `SourcePatch`es `js_bridge.rs`'s `native_ports` adds: `amdb.ts`'s
/// `searchAmdbAirports` and `getAmdbData`, as esbuild's development build
/// bundles them into `nd.js` (verified against FlyByWire's actual MSFS
/// build, `.../flybywire-aircraft-a380-842/html_ui/Pages/VCockpit/
/// Instruments/A380X/ND/nd.js`, which the converter carries over unchanged
/// -- `mfd.js` does not bundle these functions, only `nd.js` does). Each
/// `find` block is long enough to be unique between the two otherwise
/// identical `navigraphRequest.get(...)` lines; `replace` keeps everything
/// but that one line, so a future rebuild that reformats the surrounding
/// code (but keeps this exact statement) still matches.
pub fn source_patches() -> Vec<crate::js::msfs::SourcePatch> {
    crate::source_patch::amdb_fetch_patches()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU64, Ordering};

    const EXAMPLE_AIRPORT: &str = r#"{
        "aerodromereferencepoint": {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "properties": { "feattype": 26, "id": 1, "name": "EXAMPLE AIRPORT", "iata": "XMP" },
                    "geometry": { "type": "Point", "coordinates": [20.0, 10.0, 100.0] }
                }
            ]
        },
        "runwayelement": {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "properties": { "feattype": 0, "id": 2, "idrwy": "09/27" },
                    "geometry": { "type": "LineString", "coordinates": [[19.995, 10.0], [20.005, 10.0]] }
                }
            ]
        }
    }"#;

    /// A fresh scratch data folder holding the given `<name>.json` files
    /// (the tests run in parallel; matches `AmdbProvider`'s own tests'
    /// convention).
    fn data_dir(files: &[(&str, &str)]) -> PathBuf {
        static COUNTER: AtomicU64 = AtomicU64::new(0);
        let n = COUNTER.fetch_add(1, Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!("fbw-oans-plugin-test-{}-{n}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        for (name, contents) in files {
            std::fs::write(dir.join(name), contents).unwrap();
        }
        dir
    }

    #[test]
    fn only_an_amdb_bridge_counts_as_the_bridge() {
        assert!(is_bridge(200, r#"{"loaded":1,"service":"amdb-bridge","version":"1.2.10"}"#));
        assert!(!is_bridge(200, r#"{"service":"something-else"}"#));
        assert!(!is_bridge(404, r#"{"service":"amdb-bridge"}"#));
        assert!(!is_bridge(200, "<html>"));
    }

    #[test]
    fn a_failed_bridge_is_skipped_for_a_while_and_a_different_one_is_asked_afresh() {
        remember("http://test-bridge-a", false);
        assert_eq!(known_health("http://test-bridge-a"), Some(false));
        assert_eq!(known_health("http://test-bridge-b"), None);
        remember("http://test-bridge-a", true);
        assert_eq!(known_health("http://test-bridge-a"), Some(true));
    }

    #[test]
    fn the_bridge_takes_amdb_requests_unless_turned_off() {
        assert_eq!(configured_bridge(""), Some(DEFAULT_BRIDGE.to_string()));
        assert_eq!(configured_bridge("bridge=off\n"), None);
        assert_eq!(configured_bridge("bridge=http://10.0.0.5:8770/\n"), Some("http://10.0.0.5:8770".to_string()));
    }

    #[test]
    fn search_request_answers_200_with_the_local_airports() {
        let root = data_dir(&[("EXMP.json", EXAMPLE_AIRPORT)]);
        let (status, body) = answer("GET", "https://amdb.api.navigraph.com/v1/search?q=", "", &root).expect("answered");
        assert_eq!(status, 200);
        let results: Vec<serde_json::Value> = serde_json::from_str(&body).unwrap();
        assert_eq!(results.len(), 1);
        assert_eq!(results[0]["idarpt"], "EXMP");
        assert_eq!(results[0]["iata"], "XMP");
    }

    #[test]
    fn search_request_filters_by_the_query_string() {
        let root = data_dir(&[("EXMP.json", EXAMPLE_AIRPORT)]);
        let (status, body) = answer("GET", "https://amdb.api.navigraph.com/v1/search?q=ZZZZ", "", &root).expect("answered");
        assert_eq!(status, 200);
        let results: Vec<serde_json::Value> = serde_json::from_str(&body).unwrap();
        assert!(results.is_empty());
    }

    #[test]
    fn airport_data_request_answers_the_requested_layers_reprojected() {
        let root = data_dir(&[("EXMP.json", EXAMPLE_AIRPORT)]);
        let (status, body) = answer(
            "GET",
            "https://amdb.api.navigraph.com/v1/EXMP?projection=NAVIGRAPH:ARP_AZEQ&format=geojson&exclude=&include=",
            "",
            &root,
        )
        .expect("answered");
        assert_eq!(status, 200);
        let data: serde_json::Value = serde_json::from_str(&body).unwrap();
        let arp = &data["aerodromereferencepoint"]["features"][0]["geometry"]["coordinates"];
        assert!(arp[0].as_f64().unwrap().abs() < 1e-6);
        assert!(arp[1].as_f64().unwrap().abs() < 1e-6);
    }

    #[test]
    fn airport_data_request_in_epsg4326_matches_the_file() {
        let root = data_dir(&[("EXMP.json", EXAMPLE_AIRPORT)]);
        let (status, body) =
            answer("GET", "https://amdb.api.navigraph.com/v1/EXMP?projection=EPSG:4326&format=geojson&exclude=&include=", "", &root)
                .expect("answered");
        assert_eq!(status, 200);
        let data: serde_json::Value = serde_json::from_str(&body).unwrap();
        let file: serde_json::Value = serde_json::from_str(EXAMPLE_AIRPORT).unwrap();
        assert_eq!(data, file);
    }

    #[test]
    fn airport_data_request_honours_include_and_exclude() {
        let root = data_dir(&[("EXMP.json", EXAMPLE_AIRPORT)]);
        let (_, body) = answer(
            "GET",
            "https://amdb.api.navigraph.com/v1/EXMP?projection=EPSG:4326&format=geojson&exclude=&include=runwayelement",
            "",
            &root,
        )
        .expect("answered");
        let data: serde_json::Value = serde_json::from_str(&body).unwrap();
        let obj = data.as_object().unwrap();
        assert_eq!(obj.len(), 1);
        assert!(obj.contains_key("runwayelement"));
    }

    #[test]
    fn unknown_airport_answers_404() {
        let root = data_dir(&[("EXMP.json", EXAMPLE_AIRPORT)]);
        let (status, body) = answer("GET", "https://amdb.api.navigraph.com/v1/ZZZZ?projection=EPSG:4326", "", &root).expect("answered");
        assert_eq!(status, 404);
        assert!(body.is_empty());
    }

    #[test]
    fn other_hosts_are_not_answered_here() {
        let root = data_dir(&[]);
        assert!(answer("GET", "http://localhost:8380/api/v1/terrain/aircraftStatusData", "", &root).is_none());
        assert!(answer("GET", "https://amdb.api.navigraph.com/v1/", "", &root).is_none());
    }

    #[test]
    fn oans_request_only_answers_the_amdb_host() {
        // The public entry point, unlike `answer`, always reads the real
        // `data_root()`; only exercise the host filter, which returns
        // before touching disk.
        assert!(oans_request("GET", "http://localhost:8380/health", "").is_none());
    }

    #[test]
    fn configured_data_dir_reads_the_inis_override() {
        assert_eq!(configured_data_dir("data_dir=D:/somewhere/amdb\n"), Some(PathBuf::from("D:/somewhere/amdb")));
        assert_eq!(configured_data_dir(""), None);
        assert_eq!(configured_data_dir("data_dir=\n"), None);
        assert_eq!(configured_data_dir("; a comment\ndata_dir = D:/spaced/amdb \n"), Some(PathBuf::from("D:/spaced/amdb")));
    }

    #[test]
    fn percent_decode_handles_plus_and_escapes() {
        assert_eq!(percent_decode("a+b%20c"), "a b c");
        assert_eq!(percent_decode("KJFK"), "KJFK");
    }

    #[test]
    fn source_patches_only_change_the_transport_line() {
        for patch in source_patches() {
            assert_eq!(patch.path, "/Pages/VCockpit/Instruments/A380X/ND/nd.js");
            assert_ne!(patch.find, patch.replace);
            assert!(patch.find.contains("await navigraphRequest.get"));
            assert!(patch.replace.contains("await (await fetch("));
            // Every line but the response line is unchanged.
            let find_lines: Vec<&str> = patch.find.lines().collect();
            let replace_lines: Vec<&str> = patch.replace.lines().collect();
            assert_eq!(find_lines.len(), replace_lines.len());
            let differing = find_lines.iter().zip(&replace_lines).filter(|(a, b)| a != b).count();
            assert_eq!(differing, 1);
        }
    }
}
