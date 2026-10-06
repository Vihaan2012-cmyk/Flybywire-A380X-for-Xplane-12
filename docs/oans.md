# OANS airport map

The OANS (Onboard Airport Navigation System: `fbw-common/src/systems/instruments/src/OANC`,
drawn on the ND's "AIRPORT" mode and the MFD's OANS page,
`fbw-a380x/src/systems/instruments/src/ND/OansControlPanel.tsx`) shows a taxi chart built from
Navigraph's Aerodrome Mapping Database (AMDB, ED-99/ARINC 816). Code: `src/oans/`.

## How it works in MSFS

`NavigraphAmdbClient` (`OANC/api/NavigraphAmdbClient.ts:14-27`) is the only source OANC ever asks:
two functions in `fbw-common/src/systems/shared/src/navigraph/amdb.ts` that call Navigraph's AMDB
REST API directly with an authenticated axios instance (`navigraphRequest`, from the
`navigraph/auth` package). No SimBridge or Coherent call is involved.

- `searchAmdbAirports(queryString)` (`amdb.ts:8-17`): `GET .../v1/search?q=<queryString>`, an
  `AmdbAirportSearchResult[]`. `OansControlPanel.tsx`'s `loadOansDb` (:278-289) calls this with
  `q=''` at start-up (and again on every `NAVIGRAPH_ACCESS_TOKEN` change, :305) both to fill the
  airport list and, from whether the call throws, to set `navigraphAvailable`. There is no separate
  Navigraph sign-in check anywhere in the OANC/ND path -- this call succeeding *is* the check.
- `getAmdbData(icao, include, exclude, projection)` (`amdb.ts:19-39`): `GET
  .../v1/<icao>?projection=<proj>&format=geojson&exclude=<csv>&include=<csv>`, an `AmdbResponse`
  (`Partial<Record<FeatureTypeString, AmdbFeatureCollection>>`: one GeoJSON `FeatureCollection` per
  layer). `Oanc.tsx`'s `loadAirportMap` (:635-679) calls this twice: once for its working layers in
  `NAVIGRAPH:ARP_AZEQ` (the client's default projection, an azimuthal-equidistant projection
  centred on the airport reference point, ARP), and once for just the `aerodromereferencepoint`
  layer in `EPSG:4326` to learn the ARP's real latitude/longitude (:706-723).

## What the plugin does

`src/oans::AmdbProvider` answers both calls from a local data folder instead of the network.
`src/oans/plugin.rs` wires it in two places:

- `js_bridge.rs`'s `simbridge_fetch` calls `oans::plugin::oans_request(method, url, body)` before
  falling through to SimBridge's own endpoints; it answers only `https://amdb.api.navigraph.com/...`
  requests and returns `None` for anything else.
- `js_bridge.rs`'s `native_ports` extends its `SourcePatch` list with
  `oans::plugin::source_patches()`. These patch `nd.js` (verified against the converter's actual
  bundle, `.../ND/nd.js:47270` and `:47282` -- the only instrument that bundles `amdb.ts`) so
  `searchAmdbAirports`/`getAmdbData`'s last line goes through the runtime's `fetch` instead of
  `navigraphRequest.get`. That patch is necessary, not optional: `navigraphRequest` is axios 0.24,
  whose `getDefaultAdapter` only ever picks its `xhr` adapter (gated on `typeof XMLHttpRequest !==
  "undefined"`), and nothing in this runtime defines `XMLHttpRequest`, so the call throws
  synchronously before any network attempt -- `fetch` (which this runtime does implement,
  `js/msfs/environment.js` -> `js_bridge.rs` `direct_call` -> `simbridge_fetch`) is never reached
  without the patch. FlyByWire's own URL-building code is unchanged; only the transport line moves.
  No auth patch was needed or added: answering the two calls at all is what `navigraphAvailable`
  needs.

Since everything is answered locally, no Navigraph account, internet connection or SimBridge is
needed for OANS to work in X-Plane.

## Folder layout

```
<X-Plane 12>/Output/fbw-a380x/amdb/<ICAO>.json
```

One file per airport, named by its upper- or lower-case ICAO code (matched case-insensitively).
The default folder is `Output/fbw-a380x/amdb/`, relative to X-Plane's own root. To use a different
folder, add a `data_dir=` line to `Output/preferences/fbw_a380x_oans.ini` (created if missing;
`;`/`#` start a comment line, same syntax as `fbw_a380x_settings.ini`):

```ini
; Output/preferences/fbw_a380x_oans.ini
data_dir=D:/somewhere/amdb
```

A relative `data_dir` is relative to X-Plane's own root, same as the default. There is no per-
airport index file: every `*.json` in the folder is offered by `searchForAirports('')`, so an
airport appears in OANS's own airport list as soon as its file exists.

## File format

Each `<ICAO>.json` is exactly one `AmdbResponse` (`amdb.ts`'s type: an object whose keys are AMDB
layer names and whose values are GeoJSON `FeatureCollection`s), in plain `EPSG:4326` (WGS84
longitude/latitude) -- i.e. the same JSON Navigraph's real API returns for
`?projection=EPSG:4326&format=geojson`. The plugin computes the `NAVIGRAPH:ARP_AZEQ` projection
itself (great-circle bearing and distance from the ARP, matching
`OansMapProjection.globalToAirportCoordinates`'s own `(distance*sin(bearing), distance*cos(bearing))`
construction, `fbw-common/src/systems/oans/OansMapProjection.ts:9-18`), so you only ever supply
plain lat/lon geometry.

- Every feature's `geometry.coordinates` are `[longitude, latitude]` or `[longitude, latitude,
  elevation metres]` (GeoJSON `Point`/`LineString`/`Polygon`/`Multi*`, any nesting -- a third
  elevation value, where present, passes through reprojection untouched).
  A `properties.midpoint` (used on some layers, e.g. taxiway guidance lines) is itself a GeoJSON
  `Point` and is reprojected the same way.
- `properties.feattype` is the ED-99 feature type code (below); `properties.id` a number, unique
  within the file (Navigraph's own id space -- any number works, X-Plane never compares it to
  anything else). Other properties are used only where FlyByWire's OANC code reads them (`idlin`,
  `idstd`, `idrwy`, `idthr`, `ident`, `name`, `iata` -- `amdb.ts` `AmdbProperties`).
- You do not need every layer, or every feature FlyByWire's `LAYER_SPECIFICATIONS` might draw --
  only `include`/`exclude`-filtered layers are drawn per request (below), and a missing layer is
  simply absent from the response, exactly as Navigraph's API would answer a request that layer had
  no features for.
- **Required for OANS to load the airport at all:** an `aerodromereferencepoint` layer with one
  `Point` feature. Its coordinates are the ARP OANC centres the map on and the plugin projects every
  other layer's `ARP_AZEQ` version around; if that feature is missing OANC logs "aerodrome reference
  point not found" and never loads the map (`Oanc.tsx:709`). Its `properties.name` and
  `properties.iata` (optional) are what `searchForAirports` returns for the airport.

### Layers (`FeatureTypeString`, `amdb.ts:63-98`)

| feattype | layer key | typical geometry | id property |
|---:|---|---|---|
| 0 | `runwayelement` | LineString/Polygon | `idrwy` |
| 1 | `runwayintersection` | Polygon | `idrwy` |
| 2 | `runwaythreshold` | Point | `idrwy`, `idthr` |
| 3 | `runwaymarking` | LineString/Polygon | `idrwy` |
| 4 | `paintedcenterline` | LineString | `idrwy` or `idlin` |
| 5 | `landandholdshortoperationlocation` | Point/LineString | `idrwy` |
| 6 | `arrestinggearlocation` | LineString | `idrwy` |
| 7 | `runwayshoulder` | Polygon | `idrwy` |
| 8 | `stopway` | Polygon | `idrwy` |
| 9 | `runwaydisplacedarea` | Polygon | `idrwy` |
| 11 | `finalapproachandtakeoffarea` | Polygon | `ident` |
| 12 | `touchdownliftofarea` | Polygon | `ident` |
| 13 | `helipadthreshold` | Point | `ident` |
| 14 | `taxiwayelement` | LineString/Polygon | `idlin` |
| 15 | `taxiwayshoulder` | Polygon | `idlin` |
| 16 | `taxiwayguidanceline` | LineString | `idlin` |
| 17 | `taxiwayintersectionmarking` | LineString/Polygon | `idlin` |
| 18 | `taxiwayholdingposition` | LineString | `idlin`, `ident` |
| 19 | `runwayexitline` | LineString | `idlin` |
| 20 | `frequencyarea` | Polygon | -- |
| 21 | `apronelement` | Polygon | `ident` or `name` |
| 22 | `standguidanceline` | LineString | `idstd` |
| 23 | `parkingstandlocation` | Point | `idstd` |
| 24 | `parkingstandarea` | Polygon | `idstd` |
| 25 | `deicingarea` | Polygon | `name` |
| 26 | `aerodromereferencepoint` | Point | `name`, `iata` |
| 27 | `verticalpolygonalstructure` | Polygon | `name` |
| 28 | `verticalpointstructure` | Point | `name` |
| 29 | `verticallinestructure` | LineString | `name` |
| 30 | `constructionarea` | Polygon | -- |
| 33 | `blastpad` | Polygon | -- |
| 34 | `serviceroad` | LineString | -- |
| 35 | `water` | Polygon | -- |
| 37 | `hotspot` | Polygon | `ident` |

(FeatureType 10, 31, 32, 36, 38-40 are reserved in ED-99 but FlyByWire's own type does not provide
them either -- `amdb.ts:11-58`'s comments -- so they have no layer key and are left out here too.)

`getAmdbData`'s `include`/`exclude` query parameters (comma-separated layer keys) filter which of
the file's layers come back: `include` empty means every layer the file has; `exclude` then removes
from that. A request for an airport whose file does not exist answers 404, same as Navigraph's own
API for an unknown ICAO.

### Minimal valid example airport

One runway (with both thresholds), one taxiway and one stand -- the least a file needs to show a
runway, a taxiway line and a stand on the map, plus the required ARP. `src/oans/mod.rs`'s and
`src/oans/plugin.rs`'s tests use exactly this data (`EXAMPLE_AIRPORT`); the coordinates are made up,
about 1 km apart, and do not correspond to a real airport.

```json
{
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
}
```

Save this as `Output/fbw-a380x/amdb/EXMP.json` and it appears in OANS's airport list as "EXAMPLE
AIRPORT" (ICAO EXMP, IATA XMP).

## Tests

`cargo +stable-x86_64-pc-windows-gnu test --release --features js` (`CARGO_TARGET_DIR=D:\A380\fbw-build\target-oans`):
313 passed, 0 failed, 4 ignored (unrelated: `mapdata`/`js_worker` tests that need X-Plane or a
running worker). `src/oans/mod.rs`'s tests cover parsing the example airport and finding its
reference point, `searchForAirports` matching ICAO/IATA/name case-insensitively (and an empty query
matching everything), `getAirportData` in `EPSG:4326` round-tripping the file unchanged,
`include`/`exclude` filtering, an unknown airport answering nothing, and the `ARP_AZEQ` projection
both placing the ARP at the origin and matching `OansMapProjection`'s bearing/distance construction
on a synthetic point 1000 m due east. `src/oans/plugin.rs`'s tests cover the same requests in their
actual `fetch(method, url, body) -> (status, body)` shape (200/404, the `?q=`/`projection=`/
`include=`/`exclude=` query string), the ini override, percent-decoding, and that each
`SourcePatch`'s `find`/`replace` differ in exactly the transport line. The headless cockpit boot
(`-- --ignored boots_fbw_cockpit_views --nocapture`) still boots all 17 views with 0 script errors,
confirming the two `nd.js` patches match the converter's actual bundle
(`.../ND/nd.js:47270,47282`, checked directly) without breaking anything else nd.js does.

## Left to wire

Nothing outside `src/oans/` and this file -- `js_bridge.rs`'s `direct_call`/`native_ports` hooks are
in place (see "What the plugin does" above).

## Only X-Plane can verify

Whether OANC actually renders the example airport's runway/taxiway/stand on the ND's airport mode
and the MFD's OANS page, and whether `loadOansDb`'s 0-6 s poll after `onAfterRender` picks up the
local list without a Navigraph sign-in prompt appearing first.
