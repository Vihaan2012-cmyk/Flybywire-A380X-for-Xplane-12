//! Weather radar on the A380X's ND, from X-Plane 12's real weather.
//!
//! FlyByWire never built a WXR picture: `EfisTawsBridge.ts:456-457` sets
//! `wxr1Failed`/`wxr2Failed = Subject.create(true)` and never changes them,
//! so both radars are permanently marked failed, and no FBW instrument
//! binds a weather layer to draw even if they were not (`docs/map-data.md`
//! section 3, confirmed again for this module: `rg -i wxr` and
//! `weatherRadar|precipitation|radarReturn` over fbw-a380x and fbw-common
//! turn up only the MFD SURV page's disconnected placeholder controls and
//! the failure/selection plumbing below). This module supplies a real one:
//!
//! - `geometry.rs`: where a polar cell (azimuth, range, tilt) points on the
//!   earth, and where a geo-referenced return sits on the ND.
//! - `levels.rs`: X-Plane's precipitation and turbulence ratios classified
//!   into the four Airbus WXR levels.
//! - `sampler.rs`: `XPLMGetWeatherAtLocation` behind a trait, so the sweep
//!   is testable without X-Plane.
//! - `image.rs`: the classified, geo-referenced returns rasterised to RGBA.
//!
//! This file ties them together: a budgeted sweep run from the plugin's own
//! tick (never a worker thread -- `XPLMGetWeatherAtLocation` is main-thread
//! only, XPLMWeather.h), a point store so a return survives being sampled
//! once and stays correctly placed as the aircraft moves and turns, and the
//! `NATIVE_IMAGE` (docs/display-stream.md) the ND's screen stream draws.
//!
//! See `docs/wxr.md` for what is real, what this module had to invent, and
//! why.

mod geometry;
mod image;
mod levels;
mod sampler;

use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock};

use levels::Level;
use sampler::{Sample, WeatherSampler, XplmSampler};

use crate::mapdata::terrain::geo::distance_wgs84;
use crate::mapdata::terrain::terronnd::NativeImage;
use crate::mapdata::terrain::types::Side;
use crate::xp::Xplm;
use crate::Vars;

/// The antenna's azimuth cells, `±HALF_SECTOR_DEG` about the nose -- real
/// airborne weather radar never scans aft, whatever the ND's own mode; 60°
/// each side is a middle-of-the-road certified sector (X-Plane's own
/// `sim/cockpit2/EFIS/EFIS_weather_sector_width`, DataRefs.txt:4185,
/// documents the field with a 45° example; this plugin's own choice, not a
/// FlyByWire figure, since FlyByWire has none -- `docs/wxr.md`).
const HALF_SECTOR_DEG: f64 = 60.;
const AZIMUTH_BINS: usize = 61;
const RANGE_BINS: usize = 24;
const TOTAL_CELLS: usize = AZIMUTH_BINS * RANGE_BINS;

/// `XPLMGetWeatherAtLocation`'s header: "not intended to be used per-frame".
/// This many calls a tick, per active side, spread the full sweep
/// (`TOTAL_CELLS`) over several seconds instead.
const SAMPLES_PER_TICK: usize = 6;

/// FlyByWire gives the WXR beam no tilt or gain input of its own to read (no
/// `A32NX_WXR_*_TILT`/`_GAIN` LVar exists anywhere in fbw-a380x -- checked
/// by `rg`; the MFD SURV page's `wxrElevnTiltSelectedIndex`/`wxrGainAuto`
/// are local UI state nothing else reads). The pedestal's real tilt/gain/
/// multiscan/GCS/mode knobs are converter datarefs instead
/// (`fbw/cockpit/SWITCH_RADAR_MULTISCAN`/`_GCS` already exist; tilt, gain
/// and mode are TBD names from a concurrent binding pass -- `read_controls`
/// below `xplm.find`s each by best-guess name every tick, the same lazy-
/// lookup pattern `read_aircraft` already uses for X-Plane's own datarefs,
/// and simply keeps this level-beam/full-gain/no-suppression default for
/// any input that is not there yet). This constant is that default and the
/// fallback whenever a knob is absent or MULTISCAN is in AUTO.
const TILT_DEG: f64 = 0.;

/// Beyond this, a stored return is dropped rather than carried forever
/// (`docs/wxr.md`): well past the ND's longest WXR-relevant range (320 nm,
/// `a380EfisRangeSettings`, `NavigationDisplay.ts:19`) with margin for the
/// beam's own range.
const MAX_STORE_RADIUS_NM: f64 = 400.;

/// How often a side's image is rebuilt from the point store, once new
/// samples have arrived: a full 768x1024 raster costs microseconds, but
/// nothing needs it faster than a real WXR's own sweep-to-sweep update.
const REBUILD_INTERVAL_S: f64 = 0.25;

/// `a380EfisRangeSettings` (`NavigationDisplay.ts:19`): index -> nm. -1 is
/// the OANS zoom, not a radar range.
const ND_RANGES_NM: [f64; 8] = [-1., 10., 20., 40., 80., 160., 320., 640.];

/// `EfisNdMode` (`NavigationDisplay.ts:33`): PLAN is north-up and excluded,
/// same as terrain (`EfisTawsBridge.ts` `terrOnNd`).
const ND_MODE_PLAN: f64 = 4.;
const ND_MODE_ARC: f64 = 3.;

/// The gauge's canvas, matching terronnd's (`terronnd.rs` `GAUGE_WIDTH`/
/// `GAUGE_HEIGHT`, panel.cfg `0,0,768,1024`): the same physical ND map area,
/// under the same opcode 60 rectangle.
const CANVAS_WIDTH: u32 = 768;
const CANVAS_HEIGHT: u32 = 1024;
/// Ownship's pixel and the range ring's radius, approximated from the ND's
/// real layout (`mapdata/terrain/navigation_display.rs`'s own, private,
/// `ARC_MODE_CENTER_OFFSET_Y_A380X`/`ROSE_MODE_CENTER_OFFSET_Y_A380X` and
/// pixel heights -- duplicated here rather than made `pub` there, since
/// that file is not this module's to change; `docs/wxr.md` flags the exact
/// pixel alignment with FlyByWire's compass rose as unverified).
const ARC_CENTER_PX: (f64, f64) = (384., 640.);
const ARC_MAX_RADIUS_PX: f64 = 480.;
const ROSE_CENTER_PX: (f64, f64) = (384., 512.);
const ROSE_MAX_RADIUS_PX: f64 = 420.;

/// A classified, geo-referenced return, kept independent of any one side's
/// range or mode (one antenna, two repeater displays).
struct Point {
    lat: f64,
    lon: f64,
    level: Level,
}

/// Buckets a point store's key to roughly a quarter of a nautical mile, fine
/// enough for the smallest ND range (10 nm over `RANGE_BINS` cells is ~0.4
/// nm apart) without a fresh bucket for every float rounding difference.
const BUCKET_SCALE: f64 = 240.;

fn bucket(lat: f64, lon: f64) -> (i32, i32) {
    ((lat * BUCKET_SCALE).round() as i32, (lon * BUCKET_SCALE).round() as i32)
}

#[derive(Default)]
struct SideState {
    cursor: usize,
    /// Whether the ND is currently showing this image (the selector
    /// L:vars this tick): `native_image` returns `None` when this is
    /// false, so a deselected or failed radar draws nothing rather than a
    /// stale picture (docs/display-stream.md's own convention: no image
    /// yet is `None`, not black).
    active: bool,
    dirty: bool,
    last_rebuild_s: f64,
    image: Option<Arc<NativeImage>>,
    generation: u64,
}

#[derive(Default)]
struct State {
    points: HashMap<(i32, i32), Point>,
    sides: [SideState; 2],
}

fn state() -> &'static Mutex<State> {
    static STATE: OnceLock<Mutex<State>> = OnceLock::new();
    STATE.get_or_init(Default::default)
}

/// The aircraft's position and true heading, X-Plane's own datarefs (not
/// FlyByWire's systems, so a plain `xplm.find` each tick rather than
/// caching a `DataRef` -- four lookups a tick is not worth a struct field
/// and the `Send`/`Sync` a static one would need; `mapdata/plugin.rs`'s
/// `MapData` caches its own because it is a `Plugin` field, not a static).
struct Aircraft {
    lat: f64,
    lon: f64,
    alt_m: f64,
    heading_true_deg: f64,
}

fn read_aircraft(xplm: &Xplm) -> Option<Aircraft> {
    let lat = xplm.find("sim/flightmodel/position/latitude")?;
    let lon = xplm.find("sim/flightmodel/position/longitude")?;
    let elevation = xplm.find("sim/flightmodel/position/elevation")?;
    let heading = xplm.find("sim/flightmodel/position/true_psi")?;
    Some(Aircraft {
        lat: xplm.get_d(lat),
        lon: xplm.get_d(lon),
        alt_m: xplm.get_d(elevation),
        heading_true_deg: xplm.get_f(heading) as f64,
    })
}

/// The pedestal's real weather-radar knob cluster (`docs/physics/
/// surveillance.md`'s "Still needed from the converter" and the module doc
/// above): `SWITCH_RADAR_MULTISCAN`/`_GCS` are already converter datarefs;
/// tilt, gain and mode are not yet, so `read_controls` looks each up by a
/// best-guess `fbw/cockpit/KNOB_RADAR_*` name and simply leaves this tick's
/// default (level beam, full gain, no clutter suppression, radar on) alone
/// for any one of them that is not published yet -- never a fabricated
/// value for a missing input.
struct Controls {
    /// `SWITCH_RADAR_MULTISCAN` > 0.5: automatic tilt computed by the
    /// AESU. Below that, or the dataref not found yet, is manual -- the
    /// pilot's own tilt knob applies (`knob_tilt_deg`, itself falling back
    /// to [`TILT_DEG`] if that knob is not there either). Polarity (which
    /// state is AUTO) is this module's own best guess pending the real
    /// binding's documented sense -- either way, both branches degrade to
    /// today's level beam whenever the tilt knob itself is absent, so a
    /// wrong guess here cannot fabricate a tilt value.
    multiscan_auto: bool,
    /// `SWITCH_RADAR_GCS` > 0.5: ground-clutter suppression on.
    gcs_on: bool,
    /// The tilt knob's raw reading in degrees, if that dataref exists.
    knob_tilt_deg: Option<f64>,
    /// The gain knob's raw reading, clamped to 0..1, if that dataref
    /// exists; `None` (full gain, [`apply_gain`]'s own default) otherwise.
    gain: Option<f32>,
    /// The mode selector's raw reading, if that dataref exists; used only
    /// by [`mode_is_off`] to detect OFF/STBY (assumed to be the lowest
    /// value the selector reports).
    mode_raw: Option<f32>,
}

fn read_controls(xplm: &Xplm) -> Controls {
    let multiscan_auto = xplm.find("fbw/cockpit/SWITCH_RADAR_MULTISCAN").map_or(true, |d| xplm.get_f(d) > 0.5);
    let gcs_on = xplm.find("fbw/cockpit/SWITCH_RADAR_GCS").is_some_and(|d| xplm.get_f(d) > 0.5);
    // The converter now binds the captain's ELEV and GAIN knobs
    // (fbw/cockpit/KNOB_RADAR_TILT, KNOB_RADAR_GAIN), but only as a raw
    // 0..1 knob travel: which travel is the CAL detent and how many degrees
    // the ELEV knob spans are not sourced yet. Reading the raw travel as
    // degrees or as a gain fraction would invent both (a knob resting at 0
    // would blank the radar), so neither is read until those figures are
    // taken from the A380 WXR documentation.
    let _ = xplm;
    let knob_tilt_deg: Option<f64> = None;
    let gain: Option<f32> = None;
    let mode_raw = xplm.find("fbw/cockpit/KNOB_RADAR_MODE").map(|d| xplm.get_f(d));
    Controls { multiscan_auto, gcs_on, knob_tilt_deg, gain, mode_raw }
}

/// The beam's tilt this tick: the manual knob only when MULTISCAN is not
/// AUTO and the knob is actually there, [`TILT_DEG`]'s level-beam default
/// otherwise (AUTO with no real auto-tilt model in this module yet, or the
/// knob simply not bound).
fn effective_tilt_deg(multiscan_auto: bool, knob_tilt_deg: Option<f64>) -> f64 {
    if multiscan_auto {
        TILT_DEG
    } else {
        knob_tilt_deg.unwrap_or(TILT_DEG)
    }
}

/// The gain knob scales the classifier's input ratio directly: a return
/// that would classify at some level at full gain classifies as a weaker
/// (or no) return at less. `None`/no dataref keeps `precip_rate_alt`
/// unchanged, exactly today's behaviour.
fn apply_gain(precip_rate_alt: f32, gain: Option<f32>) -> f32 {
    (precip_rate_alt * gain.unwrap_or(1.).clamp(0., 1.)).clamp(0., 1.)
}

/// Ground clutter suppression: a real AESU's GCS reduces low-level returns
/// near the antenna, which X-Plane's weather sampling has no direct
/// equivalent for (no terrain-relative clutter model here) -- this module's
/// principled proxy is dropping the weakest classified band, [`Level::
/// Green`], the same one a working GCS control visibly thins out on a real
/// ND. Stronger cells (Amber/Red/Magenta) are never touched: GCS does not
/// hide real weather, only clutter-strength returns.
fn gcs_suppress(level: Level, gcs_on: bool) -> Level {
    if gcs_on && level == Level::Green {
        Level::None
    } else {
        level
    }
}

/// Whether the mode selector reads as OFF/STBY: this module's best guess is
/// that state is the selector's lowest value, `<= 0.5`. `None` (dataref not
/// bound yet) never counts as off -- the radar stays on, today's behaviour.
fn mode_is_off(mode_raw: Option<f32>) -> bool {
    mode_raw.is_some_and(|m| m <= 0.5)
}

/// One of FlyByWire's LVars, read the same way the JS engine's `L:` writes
/// land (`js_bridge.rs` `VarsHost::id`: stripped of `L:`, added under
/// [`crate::NAMED`]). `None` until something has written it at least once.
fn read_named(vars: &Vars, name: &str) -> Option<f64> {
    let id = vars.ids.get(name)?;
    Some(vars.slots[id.identifier_type()][id.identifier_index()].value)
}

/// What one side's ND needs to show WXR this tick, or `None` if it should
/// not (PLAN mode, TERR or nothing selected, no range, or the radar is
/// failed).
struct SideConfig {
    range_nm: f64,
    arc_mode: bool,
}

fn side_config(vars: &Vars, side: char, wxr_failed: bool) -> Option<SideConfig> {
    if wxr_failed {
        return None;
    }
    let overlay = read_named(vars, &format!("A380X_EFIS_{side}_ACTIVE_OVERLAY"))?;
    if overlay != 1. {
        return None; // 0 none, 2 TERR (FcuBusPublisher.ts:16-17).
    }
    let mode = read_named(vars, &format!("A32NX_EFIS_{side}_ND_MODE"))?;
    if mode == ND_MODE_PLAN {
        return None;
    }
    let range_index = read_named(vars, &format!("A32NX_EFIS_{side}_ND_RANGE"))? as usize;
    let range_nm = *ND_RANGES_NM.get(range_index)?;
    if range_nm <= 0. {
        return None;
    }
    Some(SideConfig { range_nm, arc_mode: mode == ND_MODE_ARC })
}

/// The single, shared WXR failure flag (`EfisTawsBridge.ts` `terrFailed`'s
/// own pattern, `L:A32NX_WXR_TAWS_SYS_SELECTED` choosing between
/// `wxr1Failed`/`wxr2Failed` for both NDs alike, not one flag per side).
/// After `source_patches`' fix, these follow real AESU bus power instead of
/// the stock, permanently-failed `true`.
fn wxr_failed(vars: &Vars) -> bool {
    let selected = read_named(vars, "A32NX_WXR_TAWS_SYS_SELECTED").unwrap_or(0.);
    let flag = if selected == 1. {
        "A32NX_WXR_1_FAILED"
    } else if selected == 2. {
        "A32NX_WXR_2_FAILED"
    } else {
        return true;
    };
    read_named(vars, flag).unwrap_or(1.) != 0.
}

fn cell_azimuth(i: usize) -> f64 {
    -HALF_SECTOR_DEG + (i as f64) * (2. * HALF_SECTOR_DEG / (AZIMUTH_BINS - 1) as f64)
}

fn cell_range_nm(j: usize, max_range_nm: f64) -> f64 {
    ((j + 1) as f64 / RANGE_BINS as f64) * max_range_nm
}

fn upsert(points: &mut HashMap<(i32, i32), Point>, lat: f64, lon: f64, level: Level) {
    let key = bucket(lat, lon);
    if level.is_none() {
        points.remove(&key);
    } else {
        points.insert(key, Point { lat, lon, level });
    }
}

/// Each tick: budgeted sampling for every side currently showing WXR, then
/// (throttled) rasterising the sides whose picture changed. Main thread
/// only, called from the plugin's own tick after the scripts (`lib.rs`
/// "[slot tick-after-systems: wxr]"), so this tick's selector L:vars, just
/// written by the scripts, are the ones read here.
pub fn tick(vars: &Vars, xplm: &Xplm, time_s: f64) {
    let Some(aircraft) = read_aircraft(xplm) else { return };
    let controls = read_controls(xplm);
    // A mode selector read as OFF/STBY blanks the radar the same way a
    // failed or unpowered lane does -- one antenna, so this is not
    // per-side like `wxr_failed`'s own lane split.
    let failed = wxr_failed(vars) || mode_is_off(controls.mode_raw);
    let tilt_deg = effective_tilt_deg(controls.multiscan_auto, controls.knob_tilt_deg);
    let mut state = state().lock().unwrap_or_else(|e| e.into_inner());
    let mut sampler = XplmSampler;
    let configs = [side_config(vars, 'L', failed), side_config(vars, 'R', failed)];
    for (i, config) in configs.iter().enumerate() {
        let now_active = config.is_some();
        if now_active && !state.sides[i].active {
            // Just switched on: rebuild promptly even with no new samples,
            // so the picture is not blank for a whole rebuild interval.
            state.sides[i].dirty = true;
        }
        state.sides[i].active = now_active;
        let Some(config) = config else { continue };
        for _ in 0..SAMPLES_PER_TICK {
            let idx = state.sides[i].cursor;
            state.sides[i].cursor = (idx + 1) % TOTAL_CELLS;
            let (az_i, rg_i) = (idx / RANGE_BINS, idx % RANGE_BINS);
            let azimuth = cell_azimuth(az_i);
            let range_nm = cell_range_nm(rg_i, config.range_nm);
            let (lat, lon, alt_m) = geometry::beam_point(aircraft.lat, aircraft.lon, aircraft.alt_m, aircraft.heading_true_deg, azimuth, range_nm, tilt_deg);
            if let Some(Sample { precip_rate_alt, turbulence_alt }) = sampler.sample(lat, lon, alt_m) {
                let level = gcs_suppress(levels::classify(apply_gain(precip_rate_alt, controls.gain), turbulence_alt), controls.gcs_on);
                upsert(&mut state.points, lat, lon, level);
                state.sides[i].dirty = true;
            }
        }
    }
    state.points.retain(|_, p| distance_wgs84(aircraft.lat, aircraft.lon, p.lat, p.lon) <= MAX_STORE_RADIUS_NM);
    for (i, config) in configs.into_iter().enumerate() {
        let Some(config) = config else { continue };
        let side = &state.sides[i];
        let due = time_s - side.last_rebuild_s >= REBUILD_INTERVAL_S;
        if side.dirty && due {
            rebuild(&mut state, i, &aircraft, &config, time_s);
        }
    }
}

fn rebuild(state: &mut State, side: usize, aircraft: &Aircraft, config: &SideConfig, time_s: f64) {
    let (center_px, max_radius_px) = if config.arc_mode { (ARC_CENTER_PX, ARC_MAX_RADIUS_PX) } else { (ROSE_CENTER_PX, ROSE_MAX_RADIUS_PX) };
    let view = image::View {
        own_lat: aircraft.lat,
        own_lon: aircraft.lon,
        heading_true_deg: aircraft.heading_true_deg,
        range_nm: config.range_nm,
        half_sector_deg: HALF_SECTOR_DEG,
        width: CANVAS_WIDTH,
        height: CANVAS_HEIGHT,
        center_px,
        max_radius_px,
        cell_half_px: (max_radius_px / RANGE_BINS as f64).max(3.),
    };
    let returns: Vec<image::Return> = state.points.values().map(|p| image::Return { lat: p.lat, lon: p.lon, level: p.level }).collect();
    let rgba = image::rasterize(&returns, &view);
    let s = &mut state.sides[side];
    s.generation += 1;
    s.image = Some(Arc::new(NativeImage { width: CANVAS_WIDTH, height: CANVAS_HEIGHT, generation: s.generation, rgba: rgba.into() }));
    s.dirty = false;
    s.last_rebuild_s = time_s;
}

/// `WXR_L`/`WXR_R`'s image for a screen's `60 NATIVE_IMAGE` op
/// (docs/display-stream.md): `None` whenever this ND is not currently
/// showing WXR (deselected, PLAN mode, no range, or failed), so nothing is
/// drawn and whatever else the screen composed (terrain, or nd.html's own
/// background) shows instead.
pub fn native_image(id: &str) -> Option<Arc<NativeImage>> {
    let side = match id {
        "WXR_L" => 0,
        "WXR_R" => 1,
        _ => return None,
    };
    let state = state().lock().unwrap_or_else(|e| e.into_inner());
    let s = &state.sides[side];
    if !s.active {
        return None;
    }
    s.image.clone()
}

/// This side's WXR picture for a consumer that composites screen pixels
/// directly, the same tuple shape and rule as
/// `crate::mapdata::plugin::terrain_layer` (see that function's doc for the
/// full compositing contract -- this is its mutually-exclusive twin in the
/// same screen slot). `(width, height, generation, rgba)`, straight RGBA,
/// row 0 at the top, already the ND's exact 768x1024 canvas
/// (`CANVAS_WIDTH`/`CANVAS_HEIGHT`, matching `terrain_layer`'s size so the
/// compositor needs no side-specific scaling). Unlike the terrain layer this
/// one has no opaque background of its own -- mostly-transparent returns
/// over nothing -- so it must still be drawn as the bottom layer, under the
/// Chromium ND pixels, exactly where `native_image("WXR_L"/"WXR_R")` sits
/// today in the `60 NATIVE_IMAGE` stream (docs/display-stream.md). `None`
/// whenever this ND is not currently showing WXR (deselected, PLAN mode, no
/// range, or the radar failed) or the side is unrecognised: draw nothing,
/// same as the terrain layer being `None`, and let terrain's own layer (or
/// neither) fill that slot instead.
pub fn layer(side: Side) -> Option<(u32, u32, u64, Arc<[u8]>)> {
    let image = native_image(&format!("WXR_{}", side.letter()))?;
    Some((image.width, image.height, image.generation, image.rgba.clone()))
}

/// The one `SourcePatch` this module needs: FlyByWire's `wxr1Failed`/
/// `wxr2Failed` are hardcoded `Subject.create(true)` and never updated
/// (`EfisTawsBridge.ts:456-457,486-488`) -- with a real radar behind them,
/// that hides a working one, so the fix earns the patch (`docs/wxr.md`
/// explains the choice). The replacement reuses the exact bus-power and
/// reset-panel signals this same file already computes for `terr1Failed`/
/// `terr2Failed` (`onUpdate`, `EfisTawsBridge.ts:562-573`) rather than
/// inventing a new failure condition: no `A380Failure::Wxr*` exists to
/// gate on, so only power and the reset panel do, same as terrain's normal
/// (non-extreme-latitude) case.
#[cfg(feature = "js")]
pub fn source_patches() -> Vec<crate::js::msfs::SourcePatch> {
    const PATH: &str = "/Pages/VCockpit/Instruments/A380X/SystemsHost/SystemsHost.js";
    vec![crate::js::msfs::SourcePatch {
        path: PATH.to_string(),
        find: "      this.terr1Failed.set(\n        this.failuresConsumer.isActive(A380Failure.Terr1) || this.aesu1ResetPulled.get() || !this.acEssPowered.get() || extremeLatitude\n      );\n      this.terr2Failed.set(\n        this.failuresConsumer.isActive(A380Failure.Terr2) || this.aesu2ResetPulled.get() || !this.ac4Powered.get() || extremeLatitude\n      );"
            .to_string(),
        replace: "      this.terr1Failed.set(\n        this.failuresConsumer.isActive(A380Failure.Terr1) || this.aesu1ResetPulled.get() || !this.acEssPowered.get() || extremeLatitude\n      );\n      this.terr2Failed.set(\n        this.failuresConsumer.isActive(A380Failure.Terr2) || this.aesu2ResetPulled.get() || !this.ac4Powered.get() || extremeLatitude\n      );\n      this.wxr1Failed.set(this.aesu1ResetPulled.get() || !this.acEssPowered.get());\n      this.wxr2Failed.set(this.aesu2ResetPulled.get() || !this.ac4Powered.get());"
            .to_string(),
        reason: "wxr1Failed/wxr2Failed are hardcoded Subject.create(true) and never set again \
                 (EfisTawsBridge.ts:456-457), so a working radar (src/wxr) would always show as \
                 failed; this ties them to the same AESU bus power and reset-panel signals \
                 terr1Failed/terr2Failed already use, since no WXR-specific failure exists to gate on"
            .to_string(),
    }]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn azimuth_cells_span_the_sector_and_centre_on_the_nose() {
        assert_eq!(cell_azimuth(0), -HALF_SECTOR_DEG);
        assert_eq!(cell_azimuth(AZIMUTH_BINS - 1), HALF_SECTOR_DEG);
        assert_eq!(cell_azimuth((AZIMUTH_BINS - 1) / 2), 0.);
    }

    #[test]
    fn range_cells_reach_exactly_the_selected_range() {
        assert!((cell_range_nm(RANGE_BINS - 1, 40.) - 40.).abs() < 1e-9);
        assert!(cell_range_nm(0, 40.) > 0.);
        assert!(cell_range_nm(0, 40.) < cell_range_nm(1, 40.));
    }

    #[test]
    fn manual_tilt_uses_the_knob_only_when_not_auto() {
        assert_eq!(effective_tilt_deg(false, Some(7.5)), 7.5);
        assert_eq!(effective_tilt_deg(true, Some(7.5)), TILT_DEG);
        assert_eq!(effective_tilt_deg(false, None), TILT_DEG);
        assert_eq!(effective_tilt_deg(true, None), TILT_DEG);
    }

    #[test]
    fn missing_gain_knob_leaves_precip_unscaled() {
        assert_eq!(apply_gain(0.42, None), 0.42);
    }

    #[test]
    fn gain_scales_and_clamps_the_precip_ratio() {
        assert!((apply_gain(0.8, Some(0.5)) - 0.4).abs() < 1e-6);
        assert_eq!(apply_gain(0.8, Some(2.0)), 0.8); // gain itself clamped to 1
        assert_eq!(apply_gain(0.0, Some(1.0)), 0.0);
    }

    #[test]
    fn gcs_thins_only_the_weakest_band() {
        assert_eq!(gcs_suppress(Level::Green, true), Level::None);
        assert_eq!(gcs_suppress(Level::Green, false), Level::Green);
        for strong in [Level::Amber, Level::Red, Level::Magenta] {
            assert_eq!(gcs_suppress(strong, true), strong, "GCS must not hide real weather");
        }
        assert_eq!(gcs_suppress(Level::None, true), Level::None);
    }

    #[test]
    fn mode_off_only_when_the_selector_is_bound_and_reads_low() {
        assert!(!mode_is_off(None), "missing dataref must never fake OFF");
        assert!(mode_is_off(Some(0.0)));
        assert!(!mode_is_off(Some(1.0)));
    }

    #[test]
    fn range_index_zero_is_the_oans_zoom_not_a_radar_range() {
        // a380EfisRangeSettings[0] (NavigationDisplay.ts:19); side_config
        // rejects it via its `range_nm <= 0.` check.
        assert_eq!(ND_RANGES_NM[0], -1.);
    }

    #[test]
    fn upsert_removes_a_cell_that_classifies_back_to_none() {
        let mut points = HashMap::new();
        upsert(&mut points, 10., 20., Level::Red);
        assert_eq!(points.len(), 1);
        upsert(&mut points, 10., 20., Level::None);
        assert!(points.is_empty());
    }

    #[test]
    fn bucketing_merges_points_a_few_hundredths_of_a_degree_apart() {
        assert_eq!(bucket(10.001, 20.001), bucket(10.002, 20.002));
        assert_ne!(bucket(10.0, 20.0), bucket(10.1, 20.1));
    }

    /// `layer`'s size and generation contract (same tuple shape and rule as
    /// `mapdata::plugin::terrain_layer`, its twin for the XPHFBW
    /// compositor): `None` until this side is active and rebuilt, then the
    /// ND's exact 768x1024 canvas, and `generation` strictly higher after a
    /// rebuild that actually changed the picture.
    #[test]
    fn layer_is_none_until_active_and_rebuilt_then_reports_full_size_and_a_rising_generation() {
        let side = 1; // Side::Right: keep clear of any other test touching the shared state.
        assert!(layer(Side::Right).is_none());

        let aircraft = Aircraft { lat: 47.0, lon: 11.0, alt_m: 3000., heading_true_deg: 90. };
        let config = SideConfig { range_nm: 40., arc_mode: true };
        {
            let mut state = state().lock().unwrap_or_else(|e| e.into_inner());
            state.sides[side].active = true;
            upsert(&mut state.points, 47.01, 11.01, Level::Red);
            rebuild(&mut state, side, &aircraft, &config, 1.0);
        }
        let (w, h, gen1, rgba) = layer(Side::Right).expect("rebuilt while active");
        assert_eq!((w, h), (CANVAS_WIDTH, CANVAS_HEIGHT));
        assert_eq!(rgba.len(), (w * h * 4) as usize);
        assert!(gen1 > 0);

        {
            let mut state = state().lock().unwrap_or_else(|e| e.into_inner());
            upsert(&mut state.points, 47.02, 11.02, Level::Red);
            rebuild(&mut state, side, &aircraft, &config, 2.0);
        }
        let (_, _, gen2, _) = layer(Side::Right).expect("rebuilt again");
        assert!(gen2 > gen1);

        // Deselecting hides the picture again, same as `native_image`'s own
        // "not currently showing WXR" rule (this function's doc comment).
        state().lock().unwrap_or_else(|e| e.into_inner()).sides[side].active = false;
        assert!(layer(Side::Right).is_none());
    }
}
