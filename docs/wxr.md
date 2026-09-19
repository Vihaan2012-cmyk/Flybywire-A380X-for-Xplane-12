# Weather radar

A WXR picture on the A380X's ND from X-Plane 12.4.4's real weather. Code:
`src/wxr/`. Paths below are relative to D:\fbw-aircraft unless they start
with `src/` (this plugin) or name an X-Plane file.

## 1. FlyByWire's side: real, but unimplemented

`docs/map-data.md` section 3 already found this while researching terrain:
FlyByWire's A380X has no weather radar picture to draw. Checked again for
this module (`rg -i "wxr|weather.?radar"` and `rg -i
"precipitation|radarReturn|weatherradar"` over fbw-a380x and fbw-common):

- **`EfisTawsBridge.ts:456-457`** (bundled as `SystemsHost.js`, since that is
  what actually runs -- `src/js/msfs/mod.rs` excludes nothing here):
  `wxr1Failed = Subject.create<boolean>(true)`, `wxr2Failed` the same.
  Neither is ever `.set()` again anywhere else in the class (contrast
  `terr1Failed`/`terr2Failed`/`gpws1Failed`/`gpws2Failed`, all recomputed
  every `onUpdate` from real bus power and the reset panel,
  `EfisTawsBridge.ts:562-580`). Both are written straight to
  `L:A32NX_WXR_1_FAILED`/`L:A32NX_WXR_2_FAILED` (`:487-488`). The radar is
  hardcoded failed, forever, on both sides.
- **`ND/VerticalDisplay/VerticalDisplay.tsx:320-423`**: the vertical
  display's "WXR INOP" flag, from `a32nx_aesu_wxr_failed_1/2`
  (`AesuBusPublisher.ts:59`, itself just `L:A32NX_WXR_#_FAILED` above) and
  `a32nx_aesu_wxr_taws_sys_selected` (`L:A32NX_WXR_TAWS_SYS_SELECTED`, the
  same system-select switch `terrFailed` uses, `EfisTawsBridge.ts:560,583`
  -- one shared flag for both NDs, not one per side). No WXR return is ever
  drawn on the VD either; this is only the inop flag.
- **`MFD/pages/SURV/MfdSurvControls.tsx:86-435`**: the SURV page's WXR
  controls (elevation/tilt, gain, mode, PRED W/S, turbulence, "ON VD").
  Every one of `wxrElevnTiltSelectedIndex`, `wxrAuto`, `wxrGainAuto`,
  `wxrModeWx`, `wxrPredWsAuto`, `wxrTurbAuto`, `wxrOnVd` is a bare
  `Subject.create`, never bound to a SimVar, never read anywhere else in the
  codebase (checked with `rg` for each name). The page exists; nothing
  behind it does. There is, in particular, **no `L:A32NX_WXR_*_TILT` or
  `_GAIN`** anywhere in fbw-a380x.
- **Real, working selectors** this module does use: `L:A380X_EFIS_L/R_
  ACTIVE_OVERLAY` (0 none, 1 WXR, 2 TERR -- `FcuBusPublisher.ts:16-17,32-33`,
  fbw-common), `L:A32NX_EFIS_L/R_ND_MODE` (`EfisNdMode`, `NavigationDisplay.
  ts:33-39`: ROSE_ILS/VOR/NAV, ARC, PLAN), `L:A32NX_EFIS_L/R_ND_RANGE`
  (index into `a380EfisRangeSettings`, `NavigationDisplay.ts:19`: -1 (OANS
  zoom), 10, 20, 40, 80, 160, 320, 640 nm), and `L:A32NX_WXR_TAWS_SYS_
  SELECTED` with `L:A32NX_WXR_1/2_FAILED`. These are the EFIS CP's real
  panel modes; terrain uses the same overlay/mode/range for `terrOnNd`
  (`EfisTawsBridge.ts:263`), and WXR is mutually exclusive with it through
  the same selector, not a second one this module invented.

**The one `SourcePatch` this module makes** (`wxr::source_patches`,
registered from `js_bridge.rs`'s `native_ports`): with a real radar behind
it, `wxr1Failed`/`wxr2Failed` hardcoded `true` would hide a working one, so
it is worth changing. The replacement does not invent a new failure
condition -- there is no `A380Failure::Wxr1/2` to check, since FlyByWire
registers none -- it reuses the exact AESU bus power and reset-panel signals
`terr1Failed`/`terr2Failed` already compute in the same file, minus the
extreme-latitude term (that is a TAWS terrain-database limit; a radar works
fine at high latitude). The patch runs against `SystemsHost.js`, the
plugin's actual loaded script (an esbuild development build, matching
`docs/js-build.md`; verified against the shipped
`.../SystemsHost/SystemsHost.js`, which the converter carries over
unchanged) -- never against the tracked TypeScript in D:\fbw-aircraft, which
this plugin never modifies.

## 2. X-Plane's side: a point sample, not a reflectivity field

`XPLMWeather.h` (SDK 4.3.0, D:\fbw-build\xpsdk\SDK\CHeaders\XPLM), `XPLM400`:

- **`XPLMGetWeatherAtLocation(lat, lon, alt_m, &XPLMWeatherInfo_t)`**: the
  weather at one point. Its header is explicit -- "This call is not
  intended to be used per-frame. It should be called only during the
  pre-flight loop callback" -- and it "does not work world-wide, only within
  the surrounding region", returning `0` and its "best data available"
  outside that. There is no batch or area form to sample a beam in one call.
- **`precip_rate_alt`**: "Precipitation rate at the given altitude." No
  documented units or range beyond being a rate; `src/xp.rs`'s own comment
  (from the earlier interrupted attempt, left as found) ties it to
  `sim/weather/region/rain_percent`'s 0..1 ratio, which is the closest named
  X-Plane field with the same shape. There is no dBZ or reflectivity value
  anywhere in the SDK.
- **`turbulence_alt`**: a 0..1 ratio at the sampled altitude, real and used
  here (`wxr/levels.rs`) to promote a strong-precipitation cell to magenta,
  in place of a turbulence *mode* FlyByWire never gives a toggle for.
- **`cloud_layers[3]`**: type/coverage/base/top per layer, fetched
  (`xp.rs::WeatherSample`) but not used by this module -- there was no
  budget left to turn it into a "convective vs. stratiform" distinction a
  real WXR's gain/mode logic makes, and inventing one from cloud type alone
  looked more like guessing than modelling. Left for later; said so rather
  than faked.
- **`sim/cockpit2/EFIS/EFIS_weather_*`** (DataRefs.txt:4144-4207): X-Plane's
  *own* built-in weather radar simulation, for its own 2D map/EFIS gauges --
  tilt, gain, sector width, antenna limit, sweep, mode, multiscan, ground
  clutter suppression, predictive windshear, all there, all real. But
  nothing in this block exposes the resulting *return image or grid* as a
  dataref for a plugin to read -- X-Plane draws it internally for its own
  gauge. These are documented here because they establish real-world
  parameter ranges (tilt ±15°, antenna sector semi-width, an example of 45°)
  that this module's own invented constants (`HALF_SECTOR_DEG = 60`) are
  chosen to be plausible next to, not because anything reads them.

So: a real per-point sample, explicitly not meant to be called every frame,
with no reflectivity field to threshold against a real one. Everything in
section 3 below follows from that gap.

## 3. The radar model

`src/wxr/geometry.rs`, `levels.rs`, `sampler.rs`, `image.rs`, `mod.rs`.

**Sampling.** Each tick, for each ND currently showing WXR (`side_config`:
overlay selected, mode not PLAN, a real range, radar not failed), up to
`SAMPLES_PER_TICK` (6) polar cells are sampled: azimuth `±60°` about the
true heading in `AZIMUTH_BINS` (61) steps, range out to the ND's selected
range in `RANGE_BINS` (24) steps, walked round-robin so a full sweep
(`AZIMUTH_BINS * RANGE_BINS` = 1464 cells) refreshes over several seconds
(≈4s at 60 fps per active side) rather than one `XPLMGetWeatherAtLocation`
call per pixel per frame. This runs from the plugin's own tick, main thread
only, after the scripts (`lib.rs` "[slot tick-after-systems: wxr]") --
never a worker thread, since the weather API is main-thread-only (the
brief's "Threads" note, and the header above).

**Beam geometry** (`geometry::beam_point`): from the aircraft's position,
project `range_nm` along `heading + azimuth` (`crate::mapdata::terrain::geo
::project_wgs84`, already ported from SimBridge for the terrain worker --
reused rather than duplicated). Tilt is always 0° (see below); a level beam
still loses altitude with range because the earth curves away under a
straight line (`range_m^2 / (2 * earth_radius)`, the standard horizon-dip
approximation).

**No tilt or gain to read** (section 1): this module always runs a level
beam and a fixed classification (no gain curve). That is a real
simplification, stated as one, not a disguised default.

**Geo-referenced returns, not a body-fixed grid.** A classified cell is
stored as `(lat, lon, level)` in a bucketed point store (`mod.rs`'s
`Point`/`bucket`, ~0.25 nm buckets), not at its sampled grid index. A
body-relative grid would mean a return sampled a few seconds ago, before the
aircraft turned, silently points at the wrong geography by the time the
image is drawn -- exactly the smearing a real, geo-stabilised WXR display
does not have. Storing the geography instead and re-projecting it onto the
ND every rebuild (`geometry::screen_offset`, `bearing_wgs84`+`
distance_wgs84`) gets that property for free, and lets both NDs, each on
its own selected range, draw from the one shared antenna's samples. Points
beyond `MAX_STORE_RADIUS_NM` (400 nm) are dropped each tick so the store
does not grow without bound over a long flight.

**Return classification** (`levels::classify`): four levels, green/amber/
red/magenta, the real Airbus WXR convention. The *thresholds* on `precip_
rate_alt`'s 0..1 ratio (0.05 / 0.30 / 0.60 / 0.85) are this module's own
division of that ratio, not a FlyByWire or X-Plane figure -- none exists to
divide instead (section 2). `turbulence_alt >= 0.5` promotes a red cell to
magenta early, standing in for the turbulence mode X-Plane gives no toggle
for.

**Ground clutter: left out.** The brief allows it only if real; X-Plane's
weather API has no ground-return or clutter field (`EFIS_weather_gcs` is a
*suppression* toggle for X-Plane's own built-in radar sim, not a clutter
value this module could read). Nothing here fakes it.

**Image** (`image.rs`): each stored return within range and the antenna's
`±60°` sector of the *current* heading (so a return that has turned behind
the aircraft stops showing, same as a real antenna that cannot look aft) is
painted as a small filled square on a straight RGBA canvas, worse levels
painted last so a severe cell is never hidden by a milder neighbour it
overlaps. The canvas matches terronnd's (768x1024, `terronnd.rs`
`GAUGE_WIDTH`/`GAUGE_HEIGHT`) so the two ids drop into the same `60
NATIVE_IMAGE 0 0 768 1024` rectangle. Ownship's pixel and the visible
radius (`ARC_CENTER_PX`/`ARC_MAX_RADIUS_PX`, `ROSE_CENTER_PX`/`ROSE_MAX_
RADIUS_PX`, `mod.rs`) are this module's own approximation of the real ND
layout, deliberately *not* imported from `mapdata/terrain/navigation_
display.rs`'s private `ARC_MODE_CENTER_OFFSET_Y_A380X`/`ROSE_MODE_CENTER_
OFFSET_Y_A380X` and pixel-height constants (that file is not this module's
to change, and duplicating its values with a citation seemed safer than
widening its `pub` surface for one caller). **Only X-Plane can verify**
whether these line up pixel-for-pixel with FlyByWire's own compass rose;
the fan should be in the right place and the right rough size, not
necessarily flush with the range rings.

**Rebuilding is throttled**, not per-tick: a side's image is rebuilt only
once new samples have arrived *and* at least `REBUILD_INTERVAL_S` (0.25 s)
has passed since the last rebuild. A full 768x1024 raster is a few hundred
thousand simple pixel writes -- cheap even every frame in a release build --
but nothing needs it faster than a real WXR's own sweep-to-sweep update, so
it is throttled anyway rather than argued about. `generation`
(`NativeImage`) only bumps on an actual rebuild, so the renderer only
re-uploads the texture then (`docs/display-stream.md`).

## 4. Placement

Real WXR shares the ND's map area with TERR, one selector choosing between
them (section 1) -- so this module follows terrain's own composition
exactly rather than inventing a second mechanism:

- `src/js/msfs/mod.rs` `Cockpit::new`: FlyByWire ships no WXR gauge in
  panel.cfg (section 1), so there is nothing for `native_image_id` to find
  there. Instead, wherever a `TERRONND_<side>` native gauge is recognised
  (from the real `terronnd.wasm` gauge line), a `WXR_<side>` one is added
  in the same rect, on the same side of the HTML gauge (`over`/`under`) --
  one additive block, `natives.extend(wxr_gauges)`.
- `src/display/mod.rs` `draw_screen`: `crate::mapdata::plugin::native_
  image(id)` is tried first (terrain's own ids); `crate::wxr::native_image`
  is the fallback for whatever it does not answer. One line changed
  (`.or_else(...)`), matching the brief's "minimal fallback".
- **Mutual exclusivity is enforced by `wxr::native_image` itself, not by
  construction**: both `TERRONND_<side>` and `WXR_<side>` ops are emitted
  into the stream every frame regardless of which is selected (mirroring
  how terrain's own gauge behaves in MSFS -- it does not know about the
  overlay selector either). `wxr::native_image` returns `None` whenever
  this ND is not currently showing WXR (deselected, PLAN mode, no range, or
  failed) -- "nothing to draw", the same convention terrain's own image
  uses before the terrain worker has produced a first frame
  (`docs/display-stream.md`). Since both images are opaque where they do
  draw, if both mapdata's and this module's `native_image` ever returned
  `Some` for the same screen at once, whichever drew second would hide the
  other; that should never happen (the two are gated by the same EFIS CP
  selector, one value, mutually exclusive by definition), but it is worth
  naming as the one thing that would visibly break if `side_config`'s
  gating were ever wrong.

## 5. Performance

- **XPLM calls**: up to 6 `XPLMGetWeatherAtLocation` calls per tick per
  active side (0 when neither ND is showing WXR), main thread, after the
  scripts. The header's "not per-frame" caution is about *this specific
  API*, not this plugin's tick in general; a handful of calls, not one per
  polar cell per frame, is the budgeting the brief asked for.
- **Aircraft state**: `sim/flightmodel/position/latitude/longitude/
  elevation/true_psi` looked up fresh every tick (`xplm.find`, four hash
  lookups) rather than cached in a `DataRef` field -- there is no `Plugin`
  struct field for this module at all (see below), and four lookups a tick
  is not worth the plumbing a cached one would need.
- **Rasterising**: throttled to at most once every 0.25 s per side, ~0.75 MB
  cleared and a few hundred point-squares painted -- microseconds in a
  release build, and only when there is something new to show.
- **Point store**: pruned every tick to within 400 nm of the aircraft;
  bounded by the sampling budget over that horizon, not by a fixed grid.

## 6. Hooks

- `src/lib.rs`: `mod wxr;` (`[slot modules: wxr]`), and one tick-after-
  systems call, `wxr::tick(&self.vars, xplm, self.time)`, placed after the
  scripts and the display-brightness block so this tick's selector L:vars
  (just written by the scripts) are current (`[slot tick-after-systems:
  wxr]`). No `Plugin` struct field: unlike `mapdata::MapData`, this module
  caches no `DataRef` (previous point), so all its state lives behind
  `wxr::state()`'s own `Mutex`, read by `wxr::tick` and `wxr::native_image`
  alike -- the same split `mapdata/plugin.rs` uses between its `MapData`
  field (owns X-Plane-only `DataRef`s) and its `TERRAIN` static (`Send`-safe
  state the display thread also reaches).
- `src/display/mod.rs` `draw_screen`: one line, `crate::wxr::native_image`
  as the fallback after `crate::mapdata::plugin::native_image`.
- `src/js_bridge.rs` `native_ports`: `patches.extend(crate::wxr::
  source_patches())`, alongside oans's own patches.
- `src/js/msfs/mod.rs` `Cockpit::new`: the `WXR_<side>` native-gauge
  synthesis next to `TERRONND_<side>`.
- `docs/display-stream.md`: `WXR_L`/`WXR_R` added to the `NATIVE_IMAGE`
  section's id list.

## 7. Tests

`cargo +stable-x86_64-pc-windows-gnu test --release --features js`:

- **Geometry** (`wxr::geometry::tests`): a level beam heading north still
  points north and loses altitude to curvature; azimuth turns with heading;
  a positive tilt climbs faster than curvature drops; `screen_offset` places
  a point dead ahead above ownship and a point to the right on the right;
  points beyond range or outside the antenna's sector are excluded.
- **Colour levels** (`wxr::levels::tests`): the four thresholds step the
  right level at the right ratio, on both edges; strong turbulence promotes
  red to magenta but weak turbulence and turbulence below red do not; `None`
  is the only transparent level.
- **Polar-to-image** (`wxr::image::tests`): an empty return set is fully
  transparent; a return dead ahead paints at ownship's pixel plus the
  expected radius; a worse level painted at (nearly) the same spot as a
  milder one wins; a return behind the aircraft is never painted.
- **Synthetic sampler** (`wxr::sampler::tests`, `SyntheticStorm`): peaks at
  its centre and fades to nothing at its edge, standing in for X-Plane's
  weather API so the sweep and classifier are testable without X-Plane
  running.
- **The sweep's own bookkeeping** (`wxr::mod::tests`): azimuth cells span
  the sector and centre on the nose; range cells reach exactly the selected
  range; an OANS-zoom range index (`-1` nm) is not a radar range; a cell
  that re-classifies to `None` is removed from the point store rather than
  left stale; nearby points bucket to the same store key.

Not covered by an automated test, because it needs a running X-Plane: the
`SourcePatch` actually matching and applying against the shipped
`SystemsHost.js` (the `find` text was copied from that exact file, not
retyped from the TypeScript, to keep it byte-for-byte), whether real weather
ever produces a `precip_rate_alt` worth seeing over the routes this
aircraft flies, and the ND pixel alignment noted in section 3.

## 8. Report summary

**Real**: the four-level WXR convention; the EFIS CP's overlay/mode/range
selectors and the shared WXR-failed flag (once patched); `XPLMGetWeather
AtLocation`'s `precip_rate_alt`/`turbulence_alt`; the terrain worker's own
geo/geometry helpers, reused rather than copied; the composition mechanism
(same slot as terronnd, same `NATIVE_IMAGE` contract).

**Unavailable / invented, stated as such**: no reflectivity (dBZ) field
exists on either side, so the four levels' thresholds are this module's
own division of a 0..1 ratio; no tilt or gain input exists, so the beam is
always level and the gain fixed; no ground-clutter field exists, so clutter
is not modelled; the antenna's `±60°` sector is a plausible, not sourced,
choice; the exact ND pixel geometry (ownship position, visible radius) is
this module's own approximation, duplicated rather than imported from
`navigation_display.rs`, and unverified against the real compass rose.

**Performance**: budgeted to a handful of main-thread `XPLMGetWeatherAtLocation`
calls per tick per active ND, geo-referenced storage pruned to 400 nm, and
image rebuilds throttled to 4 Hz -- reasoned about above, not yet measured
in a running X-Plane.

**Only X-Plane can verify**: whether real weather ever gives a `precip_
rate_alt` worth classifying above `PRECIP_THRESHOLD` along routes actually
flown; whether the fan's pixel placement reads correctly under FlyByWire's
real compass rose artwork; whether `XPLMGetWeatherAtLocation`'s "not
per-frame" caution has a real cost at this budget that only shows up in a
running sim; and whether the `SourcePatch`'s `find` text still matches after
any future FlyByWire or converter update to `SystemsHost.js`.
