# Map data

What FlyByWire's A380X displays draw from map data, where MSFS gets it, and
what the plugin does in X-Plane. Code: `src/mapdata/`. Paths below are
relative to D:\fbw-aircraft unless they start with `simbridge`
(D:\A380\fbw-build\simbridge-src, FlyByWire's SimBridge at f593232) or `src/`
(this plugin).

| Display data | Status |
|---|---|
| ND and VD terrain | Implemented: SimBridge's renderer and the terronnd gauge, on X-Plane's DSF scenery |
| TCAS traffic (`GET_AIR_TRAFFIC`) | Implemented from X-Plane's TCAS datarefs; needs the Coherent hook wired |
| Weather radar | Not implemented: FlyByWire's A380X draws no radar image, and X-Plane has no radar field |
| Airport map (OANS) | Not implemented: needs Navigraph AMDB features apt.dat does not hold |

## 1. ND and vertical display terrain

### How it works in MSFS

1. **EGPWC (Rust systems, runs in the plugin already).**
   `fbw-common/src/wasm/systems/systems/src/enhanced_gpwc/mod.rs` writes
   `EGPWC_PRESENT_LAT/LONG/ALTITUDE/HEADING/VERTICAL_SPEED` (ARINC 429 words
   from ADIRU 1), `EGPWC_DEST_LAT/LONG`, `EGPWC_GEAR_IS_DOWN` and
   `EGPWC_TERRONND_RENDERING_MODE`; `navigation_display.rs` writes
   `EGPWC_ND_L/R_RANGE` and `EGPWC_ND_L/R_TERRAIN_ACTIVE`. The A380X builds it
   with rendering mode 3 (`fbw-a380x/src/wasm/systems/a380_systems/src/
   lib.rs:149-162`): scanline transitions, vertical display required.
2. **terronnd gauge** (`fbw-common/src/wasm/terronnd`). panel.cfg puts it
   under each ND: `[VCockpit07]` `htmlgauge00=WasmInstrument/...terronnd.wasm
   ...,L` then `htmlgauge01=A380X/ND/nd.html` (panel.cfg:70-71; `[VCockpit08]`
   :78-79 with `R`). It reads those variables (`navigationdisplay/
   configuration.h`, `collection.cpp:23-93`), sends a 46-byte status block
   (`types/simbridge.h` `AircraftStatusData`) over SimConnect client data
   `FBW_SIMBRIDGE_EGPWC_AIRCRAFT_STATUS` at most every 100 ms when something
   changed (`collection.cpp:117-176`), receives per side a thresholds block
   and then a PNG frame (`display.h:112-196`), writes the thresholds to
   `EGPWC_ND_L/R_TERRAIN_MIN/MAX_ELEVATION(_MODE)`, and draws: opaque black,
   then the frame with alpha = `LIGHT POTENTIOMETER:94/95`
   (`displaybase.cpp:34-69`).
3. **SimBridge terrain worker** (`simbridge apps/server/src/terrain/`, a Node
   worker thread with gpu.js kernels). It keeps the tiles within 800 nm of
   the aircraft stitched into one map (`mapdata/worldmap.ts`,
   `processing/maphandler.ts`), and for each side runs map cycles
   (`processing/terrainworker.ts:703-825`): a local elevation map around the
   aircraft, a histogram, the peaks or normal mode colouring with the
   density patterns (`processing/gpu/rendering/navigationdisplay.ts`), the
   vertical display profile (`processing/verticaldisplayrenderer.ts`), and
   transition frames every 40 ms composited into a 768 x 1024 screen image,
   PNG-encoded and streamed back.
4. **EfisTawsBridge** (`fbw-a380x/src/systems/systems-host/Misc/
   EfisTawsBridge.ts`) additionally POSTs the EFIS state (ND overlay, VD
   range, manual azimuth) to `/api/v1/terrain/aircraftStatusData` and the FMS
   path to `/verticalDisplayPath` (:587-593, via `fbw-common/src/systems/
   shared/src/simbridge/components/TawsData.ts`) when SimBridge's health check
   says it is connected (`ClientState.ts:115`). Once such a POST arrives the
   worker ignores terronnd's status for two minutes
   (`terrainworker.ts:321-333, 466-470, 856-869`).
5. **The ND** reads the thresholds through `EgpwcBusPublisher.ts:40-80` and
   draws over the terronnd image.

### What the plugin does

The same chain, with only the MSFS interfaces replaced:

| MSFS | Plugin |
|---|---|
| terronnd.wasm, NanoVG | `src/mapdata/terrain/terronnd.rs`, a line-for-line port of `Collection` and `Display`; compositing done in Rust |
| SimConnect client data between terronnd and SimBridge | Messages on the terrain worker's thread; the 46-byte status block is packed and parsed as the two sides do |
| SimBridge worker, gpu.js kernels | `worker.rs`, `worldmap.rs`, `navigation_display.rs`, `vertical_display.rs`, `geo.rs`: ported to CPU Rust with the same constants and logic |
| terrain.map (SRTM tiles) | X-Plane's DSF scenery converted to terrain.map's tile format (`tiles.rs`, `dsf.rs`, `scenery.rs`) |
| PNG frames | RGBA frames (PNG is lossless; SimBridge encodes and terronnd decodes) |
| HTTP POSTs from EfisTawsBridge | `mapdata::plugin::simbridge_request` (not yet reachable, see below) |
| Gauge drawn under nd.html | `mapdata::plugin::native_image("TERRONND_L"/"TERRONND_R")` and the proposed opcode 60 in docs/display-stream.md |

Threads: the worker thread runs the gauge logic, the map cycles and the
timers; one to three tile threads (a quarter of the cores) read DSFs. X-Plane's
thread (`MapData::update`, lib.rs slot `tick-after-systems: mapdata`) reads
17 variables and 2 datarefs, sends them, applies the threshold writes, and
reads the TCAS arrays.

**Terrain source.** For each one-degree tile the DSF X-Plane itself uses for
the base mesh (first non-overlay pack in `scenery_packs.ini`, then Global
Scenery, `scenery.rs`). Elevation is X-Plane 12's `elevation` raster layer
(1201 x 1201 post-centric metres), resampled to terrain.map's 278 to 281 px
tiles by area mean and quantised as terrain.map is (50 m steps from the
tile's lowest land); water is the mesh's `terrain_Water` patches; a tile
without a DSF is water, as a tile absent from terrain.map is. Converted
tiles are cached in `X-Plane 12/Output/caches/fbw-a380x-terrain`. The
previous engineer's comparison with FlyByWire's terrain.map on the same tiles
(`tiles.rs` `resampling_matches_flybywire_terrain_map`, run again here):

| Tile | Mean abs error (Mean, Floor) | Water agreement |
|---|---|---|
| +40-122 | 13.8 m | 98.4 % |
| +27+086 | 32.8 m | 99.7 % |
| -34+018 | 10.2 m | 99.1 % |

Both are SRTM-derived; the ND bins elevations by 100 ft, so this is within
a bin or two.

**Where the port differs, and why** (each also noted in the code):

- *Background tile loading.* SimBridge decompresses tiles synchronously; a
  DSF takes 0.2 to 0.4 s, so tiles load nearest first on background threads.
  A tile not yet loaded is `UnknownElevation` (magenta dots), which is what
  SimBridge's map holds for an unloaded tile. Arrived tiles are stitched in
  without waiting for the gauge's next status (`MapHandler::poll`), because
  a parked aircraft sends none.
- *Partial restitching.* When the lookup grid is unchanged only the arrived
  tiles' cells are rewritten; the result is identical.
- *Numbers.* The stitched map is `i16` (all values fit) rather than
  `Float32Array`. The kernels ran in 32-bit GPU floats; here geometry is
  64-bit, as gpu.js's CPU mode would be, and the histogram percentile sum
  stays 32-bit.
- *Reset and pause.* SimBridge resets on SimConnect's simulator-stopped event
  and sends no frames while paused. The plugin's flight loop does not run
  while X-Plane is paused (lib.rs `flight_loop`), so neither is signalled:
  the worker is told "unpaused" once at start and never reset.
- *One centre pixel.* `createLocalElevationMap` takes `acos(0/0)` at the
  aircraft's own pixel; that bearing is taken as 0 (any bearing gives the
  aircraft's position).

**Observed in SimBridge, ported as is:** `bearingWgs84`
(`processing/gpu/helper.ts`) adds pi to `atan2`, giving the reciprocal of
the usual initial bearing, and the elevation profile projects along it
(`elevationprofile.ts:54-60`). `wgs84toPixelCoordinate` compares a longitude
with the ground-truth latitude in its wrap test. The rebuild condition
`tilesLoaded || cachedTiles !== relevantTileCount` does not rebuild when the
grid shifts by a whole tile with no new tiles. The reference harness should
confirm the vertical display direction against SimBridge itself.

### Tests and timings

Run with `D:\A380\fbw-build\mapdata-harness` (`cargo test --release`), which
compiles `src/mapdata` without the plugin glue. 16 tests pass, 2 ignored
(they need X-Plane or terrain.map). Machine: this development PC.

- `worker::tests::the_captains_nd_gets_terrain_frames_and_thresholds`:
  synthetic tiles, full chain gauge -> worker -> gauge. ND map cycle
  (local map, histogram, colouring) 30 ms per side; slowest worker step with
  a full restitch 146 ms. All on the worker thread.
- `worker::xplane_tests::alps_from_xplane_scenery` (ignored; needs X-Plane):
  near Innsbruck at 9000 ft, heading 260, 40 nm, both NDs, from X-Plane's own
  scenery. First run: 662 DSF tiles converted in 79 s on 3 threads (about
  0.35 s each), nearest first, so the ND's own area is there within seconds.
  Map cycle 63 ms per side, slowest worker step 132 ms. Thresholds written:
  min 7000 ft peaks, max 11100 ft caution. The image
  (the test writes `terrain-nd-left.ppm`; a PNG copy is
  `D:\A380\fbw-build\terrain-nd-left.png`) shows the Tyrol ridges south of track
  and the lowlands north of it, and the vertical display profile below.
  From the cache: all 662 tiles in 0.9 s; map cycle 85 to 122 ms per side;
  slowest worker step 553 ms (the whole map stitched at once).
- X-Plane's thread: per frame, 19 hash lookups, 2 double datarefs, one
  channel send, and the TCAS reads (4 array reads and up to 189 double
  reads). Not measured inside X-Plane.

### Left to wire (other owners)

- **Renderer:** draw opcode 60 `NATIVE_IMAGE` (docs/display-stream.md,
  "Proposed") from `crate::mapdata::plugin::native_image`, first on
  SCREEN_DU_NDL/NDR.
- **MSFS runtime:** emit `60 TERRONND_L 0 0 768 1024` (and `_R`) before
  nd.html's ops for those screens. For the SimBridge client path, route
  `fetch` to `crate::mapdata::plugin::simbridge_request(method, path, body)`
  for `/api/v1/terrain/*`; the bridge only posts when SimBridge's `/health`
  answers and `CONFIG_SIMBRIDGE_ENABLED` is `AUTO ON`, which is not mine.
  Without it the worker uses terronnd's status, SimBridge's own fallback
  (manual azimuth along the heading on the VD, VD range -500 to 24000 ft).

### Only X-Plane can verify

The thresholds text on the ND, the image under nd.html, brightness, and
first-flight tile loading inside a running sim.

## 2. Traffic (TCAS)

FlyByWire calls `Coherent.call('GET_AIR_TRAFFIC')`
(`fbw-a380x/src/systems/systems-host/Misc/tcas/components/
LegacyTcasComputer.ts:480`, `fbw-common/src/systems/datalink/router/src/vhf/
VDL.ts:71`) and reads `JS_NPCPlane` (`tcas/lib/TcasConstants.ts:105-113`):
`name`, `uId`, `lat`, `lon`, `alt` (metres; they multiply by 3.281),
`isOnGround`, `heading`. It derives vertical speed, ground speed and closure
itself, and drops entries with all of lat, lon, alt and heading zero.

`src/mapdata/traffic.rs` answers it from X-Plane's TCAS targets
(DataRefs.txt:5157-5391): `sim/cockpit2/tcas/targets/modeS_id` as `uId`,
`flight_id` (8 bytes per target) as `name`,
`position/double/planeN_lat/_lon/_ele` (doubles, so no float rounding),
`position/psi`, `position/weight_on_wheels`. Index 0 is the user's aircraft
and is left out. Without `sim/operation/override/override_TCAS` targets are
indices below `tcas_indicators/tcas_num_acf`; with it, any slot with a
non-zero Mode S id. Targets are read on X-Plane's thread each frame.

Wiring: the runtime's `host.call` (coherent.js `callHost`) should ask
`crate::mapdata::plugin::coherent_call(name, args_json)` first; it returns
`None` for calls it does not answer. MSFS only answers `GET_AIR_TRAFFIC`
once a `JS_LISTENER_MAPS` listener has bound a bing map
(`fbw-common/src/systems/shared/src/TrafficListener.ts`); the plugin answers
regardless.

Only X-Plane can verify: whether `tcas_num_acf` counts the user's aircraft
(if it does not, the last AI plane is missed), and whether X-Plane's AI and
multiplayer planes carry Mode S ids.

## 3. Weather radar: not implemented

FlyByWire's A380X has no weather radar picture to feed. Its systems host
sets both weather radars failed (`EfisTawsBridge.ts:456-457`
`wxr1Failed = Subject.create(true)`, written to `L:A32NX_WXR_1/2_FAILED`),
and the ND and VD only use the WXR/TAWS selection and failure flags
(`ND/VerticalDisplay/VerticalDisplay.tsx:320-349`); no instrument binds a
weather map layer.

X-Plane's side, for when FlyByWire adds one: `XPLMGetWeatherAtLocation`
(XPLMWeather.h, XPLM400) returns `precip_rate_alt` for a single point and
is "not intended to be used per-frame"; `sim/weather/region/*`
(DataRefs.txt:3767-3804) are regional values (`rain_percent`, three cloud
layers) and `storm_points`/`storm_dim` are undocumented ("TODO"). None of
these is a precipitation reflectivity field around the aircraft, so a radar
picture would have to be invented from them. Gap reported, nothing built.

## 4. Airport map (OANS): not implemented

**What FlyByWire needs.** The OANS (`fbw-common/src/systems/instruments/src/
OANC/Oanc.tsx`, `fbw-a380x/src/systems/instruments/src/ND/
OansControlPanel.tsx:110`) loads Navigraph's Aerodrome Mapping Database over
the internet with the user's Navigraph account
(`fbw-common/src/systems/shared/src/navigraph/amdb.ts:19-39`,
`https://amdb.api.navigraph.com/v1/<icao>?projection=arpazeq&format=geojson`).
It asks for ED-99/ARINC 816 feature classes (`shared/src/amdb.ts:12-61`):
runway elements, thresholds and markings with `idrwy`/`idthr`, taxiway
elements and guidance lines with taxiway idents (`idlin`), holding
positions, parking stand areas and locations with `idstd`, aprons,
vertical structures, the aerodrome reference point, and more, as GeoJSON
polygons, lines and points in the ARP azimuthal-equidistant projection. It
labels taxiways, finds stands and runway exits by those idents.

**What apt.dat has.** X-Plane 12's `Global Scenery/Global Airports/Earth nav
data/apt.dat` for EDDM, for example: pavement polygons (110-116, with only a
surface type and a free-text name), painted line features (120 with 111-116
nodes carrying line type codes), signs (20, e.g. `{@Y}262`), startup
locations (1300, `gate heavy|jets 106`), runways (100), and the ATC taxi
routing network (1201 nodes, 1202 edges with a taxiway name, e.g. `1202 2 1
twoway taxiway_B K1`; 1204 active zones).

**Why that is not faithful.** The AMDB ties geometry to identity: a taxiway
element polygon or guidance line *is* taxiway K1, a stand area *is* stand
106. In apt.dat the painted geometry has no taxiway identity; names exist
only on the routing graph, which is an ATC network drawn by the scenery
author, not the painted centreline, and stand polygons do not exist (1300 is
a point). Producing AMDB features would mean matching polygons and lines to
graph edges and stands by proximity, which is guessing, and taxiway
shoulders, hotspots, deicing areas, vertical structures and holding position
idents have no apt.dat source at all. Per the team rules, left out.
