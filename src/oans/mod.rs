//! Local AMDB (ED-99/ARINC 816 Aerodrome Mapping Database) airport map data
//! for the A380X's OANS: `fbw-common/src/systems/instruments/src/OANC` and
//! `fbw-a380x/src/systems/instruments/src/ND/OansControlPanel.tsx`.
//!
//! **In MSFS**, FlyByWire's `NavigraphAmdbClient`
//! (`fbw-common/src/systems/instruments/src/OANC/api/NavigraphAmdbClient.ts:14-27`)
//! calls two functions in `fbw-common/src/systems/shared/src/navigraph/amdb.ts:1-39`
//! that hit Navigraph's AMDB REST API directly (an authenticated axios
//! instance, `navigraphRequest`, from the `navigraph/auth` npm package; no
//! SimBridge or Coherent call is involved):
//!
//! - `GET https://amdb.api.navigraph.com/v1/search?q=<queryString>` -- an
//!   `AmdbAirportSearchResult[]` (`idarpt`, `iata`, `name`, `coordinates`
//!   `{lat, lon}`, `elev` metres). Called with `q=''` at OANS start-up
//!   both to fill the airport list and, from whether the call succeeds at
//!   all, to decide whether AMDB data is available
//!   (`OansControlPanel.tsx:278-289`, `:305`) -- there is no separate
//!   Navigraph sign-in check in the OANC/ND path.
//! - `GET https://amdb.api.navigraph.com/v1/<ICAO>?projection=<proj>&format=geojson&exclude=<csv>&include=<csv>`
//!   -- an `AmdbResponse`, i.e. `Partial<Record<FeatureTypeString,
//!   AmdbFeatureCollection>>`: one GeoJSON `FeatureCollection` per ED-99
//!   layer (`FeatureTypeString`, `fbw-common/src/systems/shared/src/amdb.ts:63-98`;
//!   thirty-four layers -- runway/taxiway elements, guidance lines,
//!   holding positions, parking stand areas and locations, aprons,
//!   vertical structures, the aerodrome reference point, and more), each
//!   feature's `properties` carrying `feattype`, `id` and (depending on
//!   the layer) `idlin`/`idstd`/`idrwy`/`idthr`/`ident`/`name`/`iata`
//!   (`amdb.ts:152-188`). `projection` is `EPSG:4326` (WGS84 lat/lon) or
//!   `NAVIGRAPH:ARP_AZEQ` (`amdb.ts:7-10`), an azimuthal-equidistant
//!   projection centred on the airport reference point (ARP): OANC's own
//!   `loadAirportMap` (`Oanc.tsx:635-679`) fetches its working layers in
//!   `ArpAzeq` (the client's default projection) and, separately, just the
//!   `aerodromereferencepoint` layer in `Epsg4326` to learn the ARP's real
//!   lat/lon (`Oanc.tsx:706-723`).
//!
//! **Here**, [`AmdbProvider`] answers both calls from a local data folder
//! instead of the network (`src/oans/plugin.rs` wires it to the requests;
//! docs/oans.md has the folder layout the user must supply data in). Each
//! airport's file on disk is exactly an `AmdbResponse` in `EPSG:4326`
//! (i.e. the same JSON a `format=geojson&projection=EPSG:4326` call
//! returns) -- the plugin computes the `ARP_AZEQ` version itself with
//! [`to_arp_azeq`], so the user only ever supplies plain lat/lon geometry.
//! FlyByWire never checks Navigraph's sign-in status before either call
//! (see above), so answering them is itself the "signed in with AMDB
//! access" state OANC looks for -- no auth patch is needed.
//!
//! A transport patch *is* needed, though: `navigraphRequest`
//! (`node_modules/@navigraph/auth`, bundled into `nd.js`) is a plain axios
//! 0.24 instance, and axios's `getDefaultAdapter` only ever picks its `xhr`
//! adapter, gated on `typeof XMLHttpRequest !== "undefined"` (bundled
//! copy, `nd.js` around its `function getDefaultAdapter()`); nothing this
//! runtime provides defines `XMLHttpRequest` (`grep -r XMLHttpRequest
//! src/js` finds nothing), so `adapter` stays `undefined` and calling it
//! throws before any network call is even attempted -- `fetch` is never
//! reached. The runtime's `fetch` *does* work (`js/msfs/environment.js`
//! `globalThis.fetch`, routed to `host.call('fetch', ...)` ->
//! `js_bridge.rs`'s `direct_call` -> `simbridge_fetch`), so
//! [`plugin::source_patches`] swaps the last line of `amdb.ts`'s two
//! functions, as bundled in `nd.js` (the only instrument that bundles
//! them; `mfd.js` does not), from `navigraphRequest.get(...)` to
//! `fetch(...)`, keeping FlyByWire's own URL-building code unchanged. This
//! is the `SourcePatch` docs/team.md asks be justified: it is not
//! optional, since without it OANC's two AMDB calls throw synchronously
//! and `navigraphAvailable` never becomes `true`.

pub mod plugin;

use std::path::PathBuf;

use serde_json::{json, Map, Value};

/// Mean Earth radius, metres: the spherical approximation the great-circle
/// bearing/distance in [`to_arp_azeq`] is built from.
const EARTH_RADIUS_M: f64 = 6_371_000.0;

/// Reads a folder of per-airport AMDB files and answers `searchForAirports`
/// and `getAirportData` exactly as `NavigraphAmdbClient` expects.
pub struct AmdbProvider {
    root: PathBuf,
}

impl AmdbProvider {
    pub fn new(root: PathBuf) -> Self {
        Self { root }
    }

    fn airport_path(&self, icao: &str) -> PathBuf {
        self.root.join(format!("{}.json", icao.to_ascii_uppercase()))
    }

    /// The raw `AmdbResponse` object (`{layer: FeatureCollection, ...}`) for
    /// one airport, in the file's native `EPSG:4326`.
    fn load(&self, icao: &str) -> Option<Map<String, Value>> {
        let text = std::fs::read_to_string(self.airport_path(icao)).ok()?;
        serde_json::from_str::<Value>(&text).ok()?.as_object().cloned()
    }

    /// Every `<ICAO>.json` in the folder, upper-cased.
    fn airport_codes(&self) -> Vec<String> {
        let Ok(entries) = std::fs::read_dir(&self.root) else {
            return Vec::new();
        };
        let mut codes: Vec<String> = entries
            .filter_map(|e| e.ok())
            .filter_map(|e| {
                let path = e.path();
                (path.extension().and_then(|e| e.to_str()) == Some("json"))
                    .then(|| path.file_stem().map(|s| s.to_string_lossy().to_ascii_uppercase()))
                    .flatten()
            })
            .collect();
        codes.sort();
        codes
    }

    /// The `aerodromereferencepoint` layer's first feature: `(lat, lon,
    /// elevation metres -- its `Point`'s optional third coordinate, 0 if
    /// absent --, name, iata)`. `AmdbAirportSearchResult` has no AMDB layer
    /// of its own in the real API; this plugin derives its fields from the
    /// same feature `getAirportData` returns for that layer, so the two
    /// stay consistent (amdb.ts:89 `AerodromeReferencePoint`, :198-213
    /// `AmdbAirportSearchResult`).
    fn reference_point(data: &Map<String, Value>) -> Option<(f64, f64, f64, String, Option<String>)> {
        let feature = data.get("aerodromereferencepoint")?.get("features")?.as_array()?.first()?;
        let coords = feature.get("geometry")?.get("coordinates")?.as_array()?;
        let lon = coords.first()?.as_f64()?;
        let lat = coords.get(1)?.as_f64()?;
        let elev = coords.get(2).and_then(Value::as_f64).unwrap_or(0.0);
        let props = feature.get("properties");
        let name = props.and_then(|p| p.get("name")).and_then(Value::as_str).unwrap_or("").to_string();
        let iata = props.and_then(|p| p.get("iata")).and_then(Value::as_str).filter(|s| !s.is_empty()).map(str::to_string);
        Some((lat, lon, elev, name, iata))
    }

    /// `searchForAirports(query)`: every local airport whose ICAO, IATA or
    /// name contains `query` case-insensitively; an empty `query` matches
    /// all of them (FlyByWire's own start-up call, see the module docs).
    /// Returns `AmdbAirportSearchResult[]` JSON values, sorted by ICAO.
    pub fn search(&self, query: &str) -> Vec<Value> {
        let q = query.trim().to_ascii_uppercase();
        self.airport_codes()
            .into_iter()
            .filter_map(|icao| {
                let data = self.load(&icao)?;
                let (lat, lon, elev, name, iata) = Self::reference_point(&data)?;
                let matches = q.is_empty()
                    || icao.contains(&q)
                    || name.to_ascii_uppercase().contains(&q)
                    || iata.as_deref().map(|i| i.to_ascii_uppercase().contains(&q)).unwrap_or(false);
                matches.then(|| {
                    json!({
                        "idarpt": icao,
                        "iata": iata,
                        "name": name,
                        "coordinates": { "lat": lat, "lon": lon },
                        "elev": elev,
                    })
                })
            })
            .collect()
    }

    /// `getAirportData(icao, includeFeatureTypes, excludeFeatureTypes,
    /// projection)`: the requested layers, reprojected if asked for
    /// `NAVIGRAPH:ARP_AZEQ`. `include` empty means every layer the file
    /// has; `exclude` then removes from that, matching the real endpoint's
    /// query parameters (`amdb.ts:20-33`, both default to `''` when the
    /// client passes `undefined`).
    pub fn airport_data(&self, icao: &str, projection: &str, include: &[String], exclude: &[String]) -> Option<Value> {
        let data = self.load(icao)?;
        let arp = Self::reference_point(&data).map(|(lat, lon, ..)| (lat, lon));
        let mut out = Map::new();
        for (layer, collection) in &data {
            if !include.is_empty() && !include.iter().any(|l| l == layer) {
                continue;
            }
            if exclude.iter().any(|l| l == layer) {
                continue;
            }
            let mut collection = collection.clone();
            if projection == "NAVIGRAPH:ARP_AZEQ" {
                if let Some(arp) = arp {
                    reproject_feature_collection(&mut collection, arp);
                }
            }
            out.insert(layer.clone(), collection);
        }
        Some(Value::Object(out))
    }
}

/// The ARP-centred azimuthal equidistant projection `NAVIGRAPH:ARP_AZEQ`
/// geometry is in: for a point at great-circle bearing `b` (true, from
/// north) and distance `d` from the ARP, `(east, north) = (d*sin(b),
/// d*cos(b))` metres. This is the same construction FlyByWire's own
/// `OansMapProjection.globalToAirportCoordinates` uses to place the
/// aircraft's own position in the AMDB layers' local frame
/// (`fbw-common/src/systems/oans/OansMapProjection.ts:9-18`: bearing and
/// distance from the ARP, `out[0] = distance*sin(bearing)` east,
/// `out[1] = distance*cos(bearing)` north, in metres), so features placed
/// here and the aircraft placed there agree.
fn to_arp_azeq(arp_lat: f64, arp_lon: f64, lat: f64, lon: f64) -> (f64, f64) {
    let (lat1, lon1, lat2, lon2) = (arp_lat.to_radians(), arp_lon.to_radians(), lat.to_radians(), lon.to_radians());
    let dlon = lon2 - lon1;
    let a = ((lat2 - lat1) / 2.0).sin().powi(2) + lat1.cos() * lat2.cos() * (dlon / 2.0).sin().powi(2);
    let distance = EARTH_RADIUS_M * 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
    let bearing = (dlon.sin() * lat2.cos()).atan2(lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * dlon.cos());
    (distance * bearing.sin(), distance * bearing.cos())
}

/// Reprojects every feature's geometry (and a `midpoint` property, which
/// `amdb.ts:187` also types as a GeoJSON `Point`) in place.
fn reproject_feature_collection(collection: &mut Value, arp: (f64, f64)) {
    let Some(features) = collection.get_mut("features").and_then(Value::as_array_mut) else {
        return;
    };
    for feature in features {
        if let Some(geometry) = feature.get_mut("geometry") {
            reproject_geometry(geometry, arp);
        }
        if let Some(midpoint) = feature.pointer_mut("/properties/midpoint") {
            reproject_geometry(midpoint, arp);
        }
    }
}

fn reproject_geometry(geometry: &mut Value, arp: (f64, f64)) {
    if let Some(coordinates) = geometry.get_mut("coordinates") {
        reproject_coordinates(coordinates, arp);
    }
}

/// Walks a GeoJSON `coordinates` tree (`Point`, `LineString`, `Polygon`,
/// `MultiPoint`/`MultiLineString`/`MultiPolygon` -- any nesting depth) and
/// reprojects each `[lon, lat]`/`[lon, lat, elevation]` leaf in place,
/// leaving a third (elevation) coordinate untouched.
fn reproject_coordinates(value: &mut Value, arp: (f64, f64)) {
    let Value::Array(items) = value else {
        return;
    };
    let is_position = (2..=3).contains(&items.len()) && items.iter().all(Value::is_number);
    if is_position {
        let lon = items[0].as_f64().unwrap_or(0.0);
        let lat = items[1].as_f64().unwrap_or(0.0);
        let (east, north) = to_arp_azeq(arp.0, arp.1, lat, lon);
        items[0] = json!(east);
        items[1] = json!(north);
        return;
    }
    for item in items.iter_mut() {
        reproject_coordinates(item, arp);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU64, Ordering};

    /// A minimal but complete example airport: one runway (with both
    /// thresholds), one taxiway, one stand and the ARP -- the same shape as
    /// docs/oans.md's worked example, at made-up coordinates.
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
        },
        "runwaythreshold": {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "properties": { "feattype": 2, "id": 3, "idrwy": "09", "idthr": "09" },
                    "geometry": { "type": "Point", "coordinates": [19.995, 10.0] }
                },
                {
                    "type": "Feature",
                    "properties": { "feattype": 2, "id": 4, "idrwy": "27", "idthr": "27" },
                    "geometry": { "type": "Point", "coordinates": [20.005, 10.0] }
                }
            ]
        },
        "taxiwayelement": {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "properties": { "feattype": 14, "id": 5, "idlin": "A" },
                    "geometry": { "type": "LineString", "coordinates": [[20.0, 10.0], [20.0015, 10.0008]] }
                }
            ]
        },
        "parkingstandarea": {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "properties": { "feattype": 24, "id": 6, "idstd": "101" },
                    "geometry": {
                        "type": "Polygon",
                        "coordinates": [[[20.0014, 10.0007], [20.0016, 10.0007], [20.0016, 10.0009], [20.0014, 10.0009], [20.0014, 10.0007]]]
                    }
                }
            ]
        },
        "parkingstandlocation": {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "properties": { "feattype": 23, "id": 7, "idstd": "101" },
                    "geometry": { "type": "Point", "coordinates": [20.0015, 10.0008] }
                }
            ]
        }
    }"#;

    /// A fresh scratch folder per call (the tests run in parallel), holding
    /// the given `<name>.json` files; matches the `std::env::temp_dir()`
    /// convention other modules' tests use (e.g. `navdata::apt::tests`).
    fn provider_with(files: &[(&str, &str)]) -> (PathBuf, AmdbProvider) {
        static COUNTER: AtomicU64 = AtomicU64::new(0);
        let n = COUNTER.fetch_add(1, Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!("fbw-oans-test-{}-{n}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        for (name, contents) in files {
            std::fs::write(dir.join(name), contents).unwrap();
        }
        let provider = AmdbProvider::new(dir.clone());
        (dir, provider)
    }

    #[test]
    fn parses_the_example_airport_and_finds_its_reference_point() {
        let (_dir, provider) = provider_with(&[("EXMP.json", EXAMPLE_AIRPORT)]);
        let data = provider.load("EXMP").expect("EXMP.json parses");
        let (lat, lon, elev, name, iata) = AmdbProvider::reference_point(&data).expect("has an ARP");
        assert!((lat - 10.0).abs() < 1e-9);
        assert!((lon - 20.0).abs() < 1e-9);
        assert!((elev - 100.0).abs() < 1e-9);
        assert_eq!(name, "EXAMPLE AIRPORT");
        assert_eq!(iata.as_deref(), Some("XMP"));
    }

    #[test]
    fn search_matches_icao_iata_and_name_case_insensitively_and_empty_matches_all() {
        let (_dir, provider) = provider_with(&[("EXMP.json", EXAMPLE_AIRPORT)]);
        for q in ["", "exmp", "XMP", "example"] {
            let results = provider.search(q);
            assert_eq!(results.len(), 1, "query {q:?}");
            assert_eq!(results[0]["idarpt"], "EXMP");
            assert_eq!(results[0]["iata"], "XMP");
            assert_eq!(results[0]["coordinates"]["lat"].as_f64().map(|v| (v - 10.0).abs() < 1e-9), Some(true));
            assert_eq!(results[0]["elev"], 100.0);
        }
        assert!(provider.search("ZZZZ").is_empty());
    }

    #[test]
    fn airport_data_in_epsg4326_is_the_file_unchanged() {
        let (_dir, provider) = provider_with(&[("EXMP.json", EXAMPLE_AIRPORT)]);
        let data = provider.airport_data("EXMP", "EPSG:4326", &[], &[]).expect("EXMP found");
        let file: Value = serde_json::from_str(EXAMPLE_AIRPORT).unwrap();
        assert_eq!(data, file);
    }

    #[test]
    fn airport_data_include_and_exclude_filter_layers() {
        let (_dir, provider) = provider_with(&[("EXMP.json", EXAMPLE_AIRPORT)]);
        let only_runways =
            provider.airport_data("EXMP", "EPSG:4326", &["runwayelement".to_string()], &[]).expect("found");
        let obj = only_runways.as_object().unwrap();
        assert_eq!(obj.len(), 1);
        assert!(obj.contains_key("runwayelement"));

        let without_stands = provider
            .airport_data("EXMP", "EPSG:4326", &[], &["parkingstandarea".to_string(), "parkingstandlocation".to_string()])
            .expect("found");
        assert!(!without_stands.as_object().unwrap().contains_key("parkingstandarea"));
        assert!(without_stands.as_object().unwrap().contains_key("runwayelement"));
    }

    #[test]
    fn unknown_airport_returns_none() {
        let (_dir, provider) = provider_with(&[("EXMP.json", EXAMPLE_AIRPORT)]);
        assert!(provider.airport_data("ZZZZ", "EPSG:4326", &[], &[]).is_none());
        assert!(provider.search("nonexistent").is_empty());
    }

    #[test]
    fn arp_azeq_places_the_reference_point_at_the_origin() {
        let (east, north) = to_arp_azeq(50.0, 8.0, 50.0, 8.0);
        assert!(east.abs() < 1e-9 && north.abs() < 1e-9);
    }

    #[test]
    fn arp_azeq_matches_flybywires_bearing_distance_construction() {
        // A point 1000 m due east of the ARP: FlyByWire's own
        // OansMapProjection places a point at bearing b, distance d as
        // (d*sin(b), d*cos(b)); due east is bearing 90 deg, so east = d,
        // north = 0.
        let arp_lat: f64 = 50.0;
        let arp_lon = 8.0;
        // Roughly 1000 m east at this latitude.
        let dlon = 1000.0 / (EARTH_RADIUS_M * arp_lat.to_radians().cos());
        let (east, north) = to_arp_azeq(arp_lat, arp_lon, arp_lat, arp_lon + dlon.to_degrees());
        assert!((east - 1000.0).abs() < 1.0, "east = {east}");
        assert!(north.abs() < 1.0, "north = {north}");
    }

    #[test]
    fn reprojecting_the_example_airport_moves_the_arp_point_to_the_origin() {
        let (_dir, provider) = provider_with(&[("EXMP.json", EXAMPLE_AIRPORT)]);
        let data = provider.airport_data("EXMP", "NAVIGRAPH:ARP_AZEQ", &[], &[]).expect("found");
        let arp_feature = &data["aerodromereferencepoint"]["features"][0];
        let coords = arp_feature["geometry"]["coordinates"].as_array().unwrap();
        assert!(coords[0].as_f64().unwrap().abs() < 1e-6);
        assert!(coords[1].as_f64().unwrap().abs() < 1e-6);
        // A non-ARP feature (a LineString) should have moved away from (0, 0).
        let rwy = &data["runwayelement"]["features"][0];
        let rwy_coords = &rwy["geometry"]["coordinates"][0];
        let x = rwy_coords[0].as_f64().unwrap();
        let y = rwy_coords[1].as_f64().unwrap();
        assert!(x.abs() > 1.0 || y.abs() > 1.0);
    }
}
