//! The Free Airport Mapping DB on Hugging Face, for when the AMDB Bridge is
//! not running: one airport file per ICAO code, downloaded on first use and
//! kept in the local AMDB folder in the plain lat/lon form [`super::AmdbProvider`]
//! reads. The data is OpenStreetMap's, under the ODbL.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::time::{Duration, Instant};

use serde_json::{json, Map, Value};

const DATASET: &str = "https://huggingface.co/datasets/Royale2072/free-airport-mapping-db/resolve/main";
const SPHERE_RADIUS_M: f64 = 6_371_008.8;
const RECEIVE_MS: u32 = 90_000;
const RETRY_AFTER: Duration = Duration::from_secs(600);
const INDEX_MAX_AGE: Duration = Duration::from_secs(7 * 24 * 3600);
pub const ATTRIBUTION: &str = "Airport map data \u{a9} OpenStreetMap contributors, available under the Open Database License (ODbL), \
from the Free Airport Mapping DB (huggingface.co/datasets/Royale2072/free-airport-mapping-db).";

#[derive(Clone, Debug, PartialEq)]
pub enum Fetch {
    Ready,
    Pending,
    Missing,
}

enum State {
    InFlight,
    Failed(Instant),
}

static DOWNLOADS: Mutex<Option<HashMap<PathBuf, State>>> = Mutex::new(None);

fn icao_code(icao: &str) -> Option<String> {
    let code: String = icao.trim().to_ascii_uppercase();
    (code.len() == 4 && code.chars().all(|c| c.is_ascii_alphanumeric())).then_some(code)
}

pub fn airport_url(icao: &str) -> Option<String> {
    let code = icao_code(icao)?;
    Some(format!("{DATASET}/v1/{}/{code}.json", &code[..2]))
}

fn index_path(root: &Path, query: &str) -> (PathBuf, String) {
    match query.chars().next().filter(char::is_ascii_alphanumeric).map(|c| c.to_ascii_uppercase()) {
        None => (root.join("index").join("search.json"), format!("{DATASET}/v1/search.json")),
        Some(c) => (root.join("index").join(format!("search-{c}.json")), format!("{DATASET}/v1/search/{c}.json")),
    }
}

fn fresh(path: &Path, max_age: Option<Duration>) -> bool {
    let Ok(meta) = std::fs::metadata(path) else { return false };
    match max_age {
        None => true,
        Some(age) => meta.modified().ok().and_then(|m| m.elapsed().ok()).is_some_and(|e| e < age),
    }
}

fn download(target: PathBuf, url: String, convert: fn(&str) -> Option<String>, max_age: Option<Duration>) -> Fetch {
    if fresh(&target, max_age) {
        return Fetch::Ready;
    }
    let mut guard = DOWNLOADS.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
    let map = guard.get_or_insert_with(HashMap::new);
    match map.get(&target) {
        Some(State::InFlight) => return Fetch::Pending,
        Some(State::Failed(at)) if at.elapsed() < RETRY_AFTER => {
            return if target.exists() { Fetch::Ready } else { Fetch::Missing };
        }
        _ => {}
    }
    map.insert(target.clone(), State::InFlight);
    let key = target.clone();
    let spawned = std::thread::Builder::new().name("fbw-amdb-hf".into()).spawn(move || {
        let ok = crate::net::request_with("GET", &url, "", RECEIVE_MS)
            .filter(|(status, _)| *status == 200)
            .and_then(|(_, body)| convert(&body))
            .is_some_and(|text| write(&key, &text));
        let mut guard = DOWNLOADS.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
        if let Some(map) = guard.as_mut() {
            if ok {
                map.remove(&key);
            } else {
                map.insert(key, State::Failed(Instant::now()));
            }
        }
    });
    if spawned.is_err() {
        map.insert(target, State::Failed(Instant::now()));
        return Fetch::Missing;
    }
    Fetch::Pending
}

fn write(path: &Path, text: &str) -> bool {
    let Some(dir) = path.parent() else { return false };
    if std::fs::create_dir_all(dir).is_err() {
        return false;
    }
    let root = if dir.file_name().is_some_and(|n| n == "index") { dir.parent().unwrap_or(dir) } else { dir };
    let _ = std::fs::write(root.join("ATTRIBUTION.txt"), format!("{ATTRIBUTION}\n"));
    let partial = path.with_extension("part");
    std::fs::write(&partial, text).is_ok() && std::fs::rename(&partial, path).is_ok()
}

pub fn airport(root: &Path, icao: &str) -> Fetch {
    let (Some(code), Some(url)) = (icao_code(icao), airport_url(icao)) else { return Fetch::Missing };
    download(root.join(format!("{code}.json")), url, |body| to_local(&serde_json::from_str(body).ok()?).map(|v| v.to_string()), None)
}

pub fn search_index(root: &Path, query: &str) -> Option<Vec<Value>> {
    let (path, url) = index_path(root, query);
    match download(path.clone(), url, |body| serde_json::from_str::<Vec<Value>>(body).ok().map(|_| body.to_string()), Some(INDEX_MAX_AGE)) {
        Fetch::Pending => None,
        Fetch::Ready | Fetch::Missing => Some(
            std::fs::read_to_string(&path).ok().and_then(|t| serde_json::from_str::<Vec<Value>>(&t).ok()).unwrap_or_default(),
        ),
    }
}

pub fn matches(entry: &Value, query: &str) -> bool {
    let q = query.trim().to_ascii_uppercase();
    q.is_empty()
        || ["idarpt", "iata", "name"]
            .iter()
            .filter_map(|k| entry.get(*k).and_then(Value::as_str))
            .any(|v| v.to_ascii_uppercase().contains(&q))
}

pub fn to_local(hf: &Value) -> Option<Value> {
    let arp = hf.get("arp")?.as_array()?;
    let origin = (arp.first()?.as_f64()?, arp.get(1)?.as_f64()?);
    let mut out = Map::new();
    for (name, collection) in hf.get("layers")?.as_object()? {
        let name = if name == "touchdownliftoffarea" { "touchdownliftofarea" } else { name.as_str() };
        let mut collection = collection.clone();
        if let Some(features) = collection.get_mut("features").and_then(Value::as_array_mut) {
            for feature in features {
                for pointer in ["/geometry/coordinates", "/properties/midpoint/coordinates", "/properties/centroid/coordinates"] {
                    if let Some(coordinates) = feature.pointer_mut(pointer) {
                        unproject_coordinates(coordinates, origin);
                    }
                }
            }
        }
        out.insert(name.to_string(), collection);
    }
    if let Some(feature) = out.get_mut("aerodromereferencepoint").and_then(|c| c.pointer_mut("/features/0")) {
        let elev = feature.pointer("/properties/elev").and_then(Value::as_f64);
        if let (Some(elev), Some(coordinates)) = (elev, feature.pointer_mut("/geometry/coordinates").and_then(Value::as_array_mut)) {
            coordinates.truncate(2);
            coordinates.push(json!(elev));
        }
    }
    Some(Value::Object(out))
}

fn from_arp_azeq(origin: (f64, f64), east: f64, north: f64) -> (f64, f64) {
    let (lat0, lon0) = (origin.0.to_radians(), origin.1.to_radians());
    let rho = east.hypot(north);
    if rho == 0.0 {
        return origin;
    }
    let c = rho / SPHERE_RADIUS_M;
    let lat = (c.cos() * lat0.sin() + north * c.sin() * lat0.cos() / rho).asin();
    let lon = lon0 + (east * c.sin()).atan2(rho * lat0.cos() * c.cos() - north * lat0.sin() * c.sin());
    (lat.to_degrees(), lon.to_degrees())
}

fn unproject_coordinates(value: &mut Value, origin: (f64, f64)) {
    let Value::Array(items) = value else { return };
    if (2..=3).contains(&items.len()) && items.iter().all(Value::is_number) {
        let (east, north) = (items[0].as_f64().unwrap_or(0.0), items[1].as_f64().unwrap_or(0.0));
        let (lat, lon) = from_arp_azeq(origin, east, north);
        let round = |v: f64| (v * 1e8).round() / 1e8;
        items[0] = json!(round(lon));
        items[1] = json!(round(lat));
        return;
    }
    for item in items.iter_mut() {
        unproject_coordinates(item, origin);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const HF_AIRPORT: &str = r#"{
        "icao": "EXMP",
        "arp": [10.0, 20.0],
        "layers": {
            "aerodromereferencepoint": { "type": "FeatureCollection", "features": [
                { "type": "Feature", "properties": { "feattype": 26, "id": 1, "idarpt": "EXMP", "name": "EXAMPLE", "iata": "XMP", "elev": 100.0 },
                  "geometry": { "type": "Point", "coordinates": [0.0, 0.0] } } ] },
            "runwayelement": { "type": "FeatureCollection", "features": [
                { "type": "Feature", "properties": { "feattype": 0, "id": 2, "idrwy": "09.27", "centroid": { "type": "Point", "coordinates": [0.0, 0.0] } },
                  "geometry": { "type": "LineString", "coordinates": [[-547.6, 0.0], [547.6, 0.0]] } } ] },
            "touchdownliftoffarea": { "type": "FeatureCollection", "features": [] }
        }
    }"#;

    #[test]
    fn a_dataset_airport_becomes_a_local_lat_lon_file_that_projects_back_to_the_same_metres() {
        let local = to_local(&serde_json::from_str(HF_AIRPORT).unwrap()).unwrap();
        assert!(local.get("touchdownliftofarea").is_some(), "FlyByWire's spelling");
        assert!(local.get("touchdownliftoffarea").is_none());
        let arp = &local["aerodromereferencepoint"]["features"][0]["geometry"]["coordinates"];
        assert_eq!((arp[0].as_f64().unwrap(), arp[1].as_f64().unwrap(), arp[2].as_f64().unwrap()), (20.0, 10.0, 100.0));
        let end = &local["runwayelement"]["features"][0]["geometry"]["coordinates"][1];
        let (lon, lat) = (end[0].as_f64().unwrap(), end[1].as_f64().unwrap());
        assert!((lat - 10.0).abs() < 1e-6 && lon > 20.0, "547.6 m east of the ARP: {lat}, {lon}");
        let (east, north) = super::super::to_arp_azeq(10.0, 20.0, lat, lon);
        assert!((east - 547.6).abs() < 0.05 && north.abs() < 0.05, "{east} {north}");
        let centroid = &local["runwayelement"]["features"][0]["properties"]["centroid"]["coordinates"];
        assert_eq!((centroid[0].as_f64().unwrap(), centroid[1].as_f64().unwrap()), (20.0, 10.0));
    }

    #[test]
    fn the_inverse_projection_round_trips_far_from_the_reference_point() {
        for (east, north) in [(3000.0, -4000.0), (-12000.0, 7000.0), (0.0, 25000.0)] {
            let (lat, lon) = from_arp_azeq((51.4775, -0.4614), east, north);
            let (e, n) = super::super::to_arp_azeq(51.4775, -0.4614, lat, lon);
            assert!((e - east).abs() < 0.1 && (n - north).abs() < 0.1, "{east},{north} -> {e},{n}");
        }
    }

    #[test]
    fn files_are_named_as_the_dataset_lays_them_out() {
        assert_eq!(airport_url("egll").unwrap(), format!("{DATASET}/v1/EG/EGLL.json"));
        assert!(airport_url("EGL").is_none());
        assert!(airport_url("EG/L").is_none());
        let root = Path::new("amdb");
        assert_eq!(index_path(root, ""), (root.join("index").join("search.json"), format!("{DATASET}/v1/search.json")));
        assert_eq!(index_path(root, "lhr"), (root.join("index").join("search-L.json"), format!("{DATASET}/v1/search/L.json")));
    }

    #[test]
    fn a_search_entry_matches_its_code_iata_or_name() {
        let entry = json!({ "idarpt": "EGLL", "iata": "LHR", "name": "London Heathrow" });
        for q in ["", "egll", "LHR", "heath"] {
            assert!(matches(&entry, q), "{q}");
        }
        assert!(!matches(&entry, "KJFK"));
        assert!(matches(&json!({ "idarpt": "XXXX", "iata": null, "name": "X" }), "xx"));
    }
}
