//! The map data in the plugin: X-Plane's state in, FlyByWire's variables and
//! the terrain images out. Everything heavy runs on the terrain worker's
//! threads; a frame here reads about twenty variables and two hundred TCAS
//! values and hands them over.

use std::sync::mpsc::Sender;
use std::sync::{Arc, Mutex};

use super::terrain::terronnd::{GaugeInputs, NativeImage, LVAR_ND, LVAR_STATUS};
use super::terrain::tiles::TileSource;
use super::terrain::types::{AircraftStatus, Side, VerticalPathData};
use super::terrain::worker::{self, Message, Shared};
use super::traffic::{self, Target};
use crate::xp::{DataRef, Xplm};
use crate::{Vars, NAMED};

/// The terrain worker, reachable from the scripts' calls.
static TERRAIN: Mutex<Option<(Sender<Message>, Arc<Mutex<Shared>>)>> = Mutex::new(None);

/// The TCAS arrays hold 64 targets; index 0 is the user's aircraft.
const TCAS_TARGETS: usize = 64;

/// `sim/cockpit2/tcas/targets/position/double/planeN_ele` and own elevation
/// are metres MSL; `surveillance::TcasRange::window_ft` is feet, per FCOM.
const METRES_TO_FEET: f64 = 3.280_839_895_013_123;

struct TcasRefs {
    num_acf: Option<DataRef>,
    override_tcas: Option<DataRef>,
    mode_s_id: Option<DataRef>,
    flight_id: Option<DataRef>,
    psi: Option<DataRef>,
    weight_on_wheels: Option<DataRef>,
    /// `position/double/planeN_lat`, `_lon`, `_ele` for targets 1 to 63.
    positions: Vec<[Option<DataRef>; 3]>,
}

pub struct MapData {
    tcas: TcasRefs,
    latitude: Option<DataRef>,
    longitude: Option<DataRef>,
    /// Own altitude, metres MSL: the reference the TCAS range window
    /// (ABV/NORM/BLW, `surveillance::TcasRange`) is relative to.
    elevation: Option<DataRef>,
    started: bool,
    /// Lines already written to Log.txt: terronnd reports every reset packet.
    logged: std::collections::HashSet<String>,
}

impl MapData {
    pub fn new(xplm: &'static Xplm) -> Self {
        let tcas = TcasRefs {
            num_acf: xplm.find("sim/cockpit2/tcas/indicators/tcas_num_acf"),
            override_tcas: xplm.find("sim/operation/override/override_TCAS"),
            mode_s_id: xplm.find("sim/cockpit2/tcas/targets/modeS_id"),
            flight_id: xplm.find("sim/cockpit2/tcas/targets/flight_id"),
            psi: xplm.find("sim/cockpit2/tcas/targets/position/psi"),
            weight_on_wheels: xplm.find("sim/cockpit2/tcas/targets/position/weight_on_wheels"),
            positions: (1..TCAS_TARGETS)
                .map(|n| ["lat", "lon", "ele"].map(|c| xplm.find(&format!("sim/cockpit2/tcas/targets/position/double/plane{n}_{c}"))))
                .collect(),
        };
        Self {
            tcas,
            latitude: xplm.find("sim/flightmodel/position/latitude"),
            longitude: xplm.find("sim/flightmodel/position/longitude"),
            elevation: xplm.find("sim/flightmodel/position/elevation"),
            started: false,
            logged: std::collections::HashSet::new(),
        }
    }

    /// Start the terrain worker: its tiles come from this installation's
    /// scenery, converted once into X-Plane's `Output/caches`.
    fn start() {
        let Some(root) = crate::xp::system_path() else {
            crate::log("mapdata: X-Plane's folder is unknown; no terrain map");
            return;
        };
        let scenery = super::scenery::Scenery::of_installation(&root);
        let cache = root.join("Output").join("caches").join("fbw-a380x-terrain");
        let provider = Arc::new(TileSource::new(scenery, Some(cache)));
        let threads = std::thread::available_parallelism().map_or(1, |n| (n.get() / 4).clamp(1, 3));
        let (tx, shared) = worker::spawn(provider, threads);
        let _ = tx.send(Message::Paused(false));
        *TERRAIN.lock().unwrap_or_else(|e| e.into_inner()) = Some((tx, shared));
    }

    /// Each frame, after the systems.
    pub fn update(&mut self, vars: &mut Vars, xplm: &Xplm) {
        if !self.started {
            self.started = true;
            Self::start();
        }
        let guard = TERRAIN.lock().unwrap_or_else(|e| e.into_inner());
        if let Some((tx, shared)) = guard.as_ref() {
            let read = |vars: &Vars, name: &str| vars.ids.get(name).map_or(0., |id| vars.slots[id.identifier_type()][id.identifier_index()].value);
            let mut inputs = GaugeInputs::default();
            for (v, name) in inputs.status.iter_mut().zip(LVAR_STATUS) {
                *v = read(vars, name);
            }
            for (v, name) in inputs.nd.iter_mut().zip(LVAR_ND) {
                *v = read(vars, name);
            }
            inputs.simulator = [
                self.latitude.map_or(0., |d| xplm.get_d(d)),
                self.longitude.map_or(0., |d| xplm.get_d(d)),
                read(vars, "LIGHT POTENTIOMETER:94"),
                read(vars, "LIGHT POTENTIOMETER:95"),
            ];
            let _ = tx.send(Message::Gauge(inputs));
            let (writes, log) = {
                let mut s = shared.lock().unwrap_or_else(|e| e.into_inner());
                (std::mem::take(&mut s.writes), s.log.drain(..).collect::<Vec<_>>())
            };
            for (name, value) in writes {
                let id = vars.add(name.to_string(), NAMED);
                crate::SimulatorReaderWriter::write(vars, &id, value);
            }
            for line in log {
                if self.logged.len() < 256 && self.logged.insert(line.clone()) {
                    crate::log(&format!("mapdata: {line}"));
                }
            }
        }
        drop(guard);
        // The TCAS traffic display's altitude window (ABV/NORM/BLW):
        // `surveillance::Surveillance` owns the SURV panel's range buttons
        // and publishes the selection as `A32NX_TCAS_RANGE`
        // (`surveillance.rs` "[shared state]") rather than this module
        // reaching into that struct directly. A never-written slot reads
        // back 0., which `TcasRange::from_code` treats as `Normal` -- the
        // same default the button logic itself starts from.
        let range_code = vars.ids.get("A32NX_TCAS_RANGE").map_or(0., |id| vars.slots[id.identifier_type()][id.identifier_index()].value);
        let window_ft = crate::surveillance::TcasRange::from_code(range_code).window_ft();
        let own_alt_ft = self.elevation.map_or(0., |d| xplm.get_d(d)) * METRES_TO_FEET;
        traffic::set(self.read_tcas(xplm, own_alt_ft, window_ft));
    }

    /// `elevation_m` -> `own_alt_ft + window_ft` inclusion test, factored out
    /// so it is testable without an `Xplm`/`DataRef`.
    fn within_tcas_window(target_elevation_m: f64, own_alt_ft: f64, window_ft: (f64, f64)) -> bool {
        let relative_ft = target_elevation_m * METRES_TO_FEET - own_alt_ft;
        relative_ft >= window_ft.0 && relative_ft <= window_ft.1
    }

    fn read_tcas(&self, xplm: &Xplm, own_alt_ft: f64, window_ft: (f64, f64)) -> Vec<Target> {
        let t = &self.tcas;
        let (Some(ids_ref), Some(psi_ref)) = (t.mode_s_id, t.psi) else { return Vec::new() };
        let overridden = t.override_tcas.is_some_and(|d| xplm.get_i(d) != 0);
        let count = t.num_acf.map_or(0, |d| xplm.get_i(d).max(0) as usize).min(TCAS_TARGETS);
        let mut ids = [0i32; TCAS_TARGETS];
        xplm.get_vi(ids_ref, &mut ids);
        let mut psi = [0f32; TCAS_TARGETS];
        xplm.get_vf(psi_ref, &mut psi);
        let mut on_ground = [0i32; TCAS_TARGETS];
        if let Some(d) = t.weight_on_wheels {
            xplm.get_vi(d, &mut on_ground);
        }
        let mut flight_ids = [0u8; TCAS_TARGETS * 8];
        if let Some(d) = t.flight_id {
            xplm.get_vb(d, &mut flight_ids);
        }
        let mut out = Vec::new();
        for i in 1..TCAS_TARGETS {
            // Without an override X-Plane fills its own planes in order;
            // with one, a plugin writes whichever slots it uses.
            if ids[i] == 0 || (!overridden && i >= count) {
                continue;
            }
            let [Some(lat), Some(lon), Some(ele)] = t.positions[i - 1] else { continue };
            let elevation_m = xplm.get_d(ele);
            // TCAS display range (ABV/NORM/BLW): a target outside the
            // selected altitude window never reaches the ND, same as the
            // real AESU's own range-limited traffic display -- the range
            // buttons otherwise have nothing to do (`docs/physics/
            // surveillance.md`).
            if !Self::within_tcas_window(elevation_m, own_alt_ft, window_ft) {
                continue;
            }
            let name = &flight_ids[i * 8..(i + 1) * 8];
            let end = name.iter().position(|&b| b == 0).unwrap_or(8);
            out.push(Target {
                mode_s_id: ids[i] as u32,
                flight_id: String::from_utf8_lossy(&name[..end]).trim().to_string(),
                latitude: xplm.get_d(lat),
                longitude: xplm.get_d(lon),
                elevation_m,
                heading: psi[i] as f64,
                on_ground: on_ground[i] != 0,
            });
        }
        out
    }
}

/// The terronnd gauge's image for a screen's gauge, `TERRONND_L` or
/// `TERRONND_R` (docs/display-stream.md, "Proposed"). Cheap: a shared copy.
pub fn native_image(id: &str) -> Option<Arc<NativeImage>> {
    let side = match id {
        "TERRONND_L" => 0,
        "TERRONND_R" => 1,
        _ => return None,
    };
    let guard = TERRAIN.lock().unwrap_or_else(|e| e.into_inner());
    let (_, shared) = guard.as_ref()?;
    let image = shared.lock().unwrap_or_else(|e| e.into_inner()).images[side].clone();
    image
}

/// The terrain layer for one ND, for a consumer that composites screen
/// pixels directly instead of drawing a `60 NATIVE_IMAGE` op (docs/
/// display-stream.md): the XPHFBW screen compositor (agent D,
/// `src/display/**`), which owns a `ScreenBlock` per screen and paints
/// nd.html's real Chromium pixels into it, rather than emitting an opcode
/// stream. A plain tuple, not `NativeImage`, so that code needs no
/// dependency on this module's internal struct: `(width, height,
/// generation, rgba)`. Same pixels as `native_image("TERRONND_L"/
/// "TERRONND_R")`: the terronnd gauge's whole 768x1024 canvas, already
/// composited against its own opaque background and dimmed by the ND's
/// `LIGHT POTENTIOMETER` (`terrain::terronnd::Display::render`), so nothing
/// further needs doing to it before it is drawn. `None` before the terrain
/// worker has drawn a first frame, or the side is unrecognised.
///
/// Compositing contract for the XPHFBW ND screens (`SCREEN_DU_NDL`/
/// `SCREEN_DU_NDR`, `src/display/screens.rs`): draw this image as the
/// bottom of the screen's whole 0,0,768,1024 region, straight
/// (non-premultiplied) RGBA with row 0 at the top and no scaling needed
/// (the image is already the screen's exact size), then alpha-composite the
/// Chromium ND page's own pixels (from that screen's `ScreenBlock`) on top
/// with the ordinary "over" operator (`dst = src*srcA + dst*(1-srcA)` per
/// channel, `dst.a = srcA + dst.a*(1-srcA)`). The ND page's root element
/// paints a transparent background (`.nd-svg { background: transparent }`,
/// D:\fbw-aircraft\fbw-common\src\systems\instruments\src\ND\style.scss:9),
/// so this layer's pixels show through wherever the page does not draw --
/// that is why it goes first, never the other way around, matching MSFS's
/// own panel.cfg gauge order for this same screen (terronnd.wasm's
/// `htmlgauge00` then nd.html's `htmlgauge01`, confirmed at
/// D:\fbw-aircraft\fbw-a380x\src\base\flybywire-aircraft-a380-842\
/// SimObjects\AirPlanes\FlyByWire_A380X\attachments\flybywire\
/// Part_Interior_Cockpit\panel\panel.cfg:70-71 (NDL) and :78-79 (NDR)).
/// Re-upload the layer only when `generation` differs from the value the
/// compositor saw last (this crate never mutates a published image in
/// place, so a stable `generation` means stable pixels; `rgba` is an
/// `Arc<[u8]>`, cheap to hold across frames for that comparison).
pub fn terrain_layer(side: Side) -> Option<(u32, u32, u64, Arc<[u8]>)> {
    let image = native_image(&format!("TERRONND_{}", side.letter()))?;
    Some((image.width, image.height, image.generation, image.rgba.clone()))
}

/// A request to SimBridge's terrain API, as FlyByWire's `TawsData`
/// (fbw-common `simbridge/components/TawsData.ts`) sends it: the status
/// code and body, or `None` if the path is not one this answers.
pub fn simbridge_request(method: &str, path: &str, body: &str) -> Option<(u16, String)> {
    let path = path.split('?').next().unwrap_or(path);
    let message = match (method.to_ascii_uppercase().as_str(), path) {
        ("POST", "/api/v1/terrain/aircraftStatusData") => AircraftStatus::from_json(body).map(Message::AircraftStatusData),
        ("POST", "/api/v1/terrain/verticalDisplayPath") => VerticalPathData::from_json(body).map(Message::VerticalDisplayPath),
        _ => return None,
    };
    Some(match message {
        Ok(message) => {
            let guard = TERRAIN.lock().unwrap_or_else(|e| e.into_inner());
            match guard.as_ref() {
                Some((tx, _)) if tx.send(message).is_ok() => (201, String::new()),
                _ => (503, "the terrain worker is not running".into()),
            }
        }
        // Nest's ValidationPipe answers an invalid body with 400.
        Err(e) => (400, e),
    })
}

/// A Coherent call this answers (`GET_AIR_TRAFFIC`).
pub fn coherent_call(name: &str, args_json: &str) -> Option<Result<String, String>> {
    traffic::call(name, args_json)
}

#[cfg(test)]
mod tests {
    use super::MapData;

    const FT_PER_M: f64 = super::METRES_TO_FEET;

    #[test]
    fn normal_window_admits_a_target_within_2700_ft_either_way() {
        let own_alt_ft = 10_000.;
        let window = (-2700., 2700.); // TcasRange::Normal
        let just_above_m = (own_alt_ft + 2699.) / FT_PER_M;
        let just_below_m = (own_alt_ft - 2699.) / FT_PER_M;
        assert!(MapData::within_tcas_window(just_above_m, own_alt_ft, window));
        assert!(MapData::within_tcas_window(just_below_m, own_alt_ft, window));
    }

    #[test]
    fn normal_window_excludes_a_target_beyond_2700_ft() {
        let own_alt_ft = 10_000.;
        let window = (-2700., 2700.);
        let too_high_m = (own_alt_ft + 3000.) / FT_PER_M;
        let too_low_m = (own_alt_ft - 3000.) / FT_PER_M;
        assert!(!MapData::within_tcas_window(too_high_m, own_alt_ft, window));
        assert!(!MapData::within_tcas_window(too_low_m, own_alt_ft, window));
    }

    #[test]
    fn above_range_admits_a_target_9900_ft_high_that_normal_would_exclude() {
        let own_alt_ft = 20_000.;
        let above_window = (-2700., 9900.); // TcasRange::Above
        let normal_window = (-2700., 2700.);
        let far_above_m = (own_alt_ft + 9000.) / FT_PER_M;
        assert!(MapData::within_tcas_window(far_above_m, own_alt_ft, above_window));
        assert!(!MapData::within_tcas_window(far_above_m, own_alt_ft, normal_window));
    }

    #[test]
    fn below_range_admits_a_target_9900_ft_low_that_normal_would_exclude() {
        let own_alt_ft = 20_000.;
        let below_window = (-9900., 2700.); // TcasRange::Below
        let normal_window = (-2700., 2700.);
        let far_below_m = (own_alt_ft - 9000.) / FT_PER_M;
        assert!(MapData::within_tcas_window(far_below_m, own_alt_ft, below_window));
        assert!(!MapData::within_tcas_window(far_below_m, own_alt_ft, normal_window));
    }
}
