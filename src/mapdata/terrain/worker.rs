//! SimBridge's terrain worker (simbridge `apps/server/src/terrain/
//! processing/terrainworker.ts`) and the terronnd gauge, on one background
//! thread.
//!
//! SimBridge's worker is a Node worker thread driven by `setInterval` and
//! `setTimeout`; here the same timers run on this thread's own clock. What
//! it receives over SimConnect (the gauge's status packet, pause and
//! simulator state) and over HTTP (`/api/v1/terrain/aircraftStatusData` and
//! `/verticalDisplayPath` from the EfisTawsBridge) arrives as messages.
//! Frames go to the gauge as RGBA instead of PNG.

use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use super::geo::*;
use super::navigation_display::*;
use super::terronnd::{Frame, Gauge, GaugeInputs, GaugeOutput, NativeImage};
use super::types::*;
use super::vertical_display::*;
use super::worldmap::{MapHandler, TileProvider};

const DISPLAY_SCREEN_PIXEL_HEIGHT_WITHOUT_VERTICAL_DISPLAY: usize = 768;
const DISPLAY_SCREEN_PIXEL_HEIGHT_WITH_VERTICAL_DISPLAY: usize = 1024;
/// ms, equals two minutes
const SIMBRIDGE_CLIENT_DATA_TIMEOUT: f64 = 2. * 60. * 1000.;

pub enum Message {
    /// What terronnd reads this frame; the gauge draws.
    Gauge(GaugeInputs),
    /// SimConnect's `Pause_EX1`.
    Paused(bool),
    /// SimConnect's `Sim` event with the simulator stopped.
    Reset,
    /// A POST to `/api/v1/terrain/aircraftStatusData`.
    AircraftStatusData(AircraftStatus),
    /// A POST to `/api/v1/terrain/verticalDisplayPath`.
    VerticalDisplayPath(VerticalPathData),
}

/// What the worker leaves for the plugin's thread.
#[derive(Default)]
pub struct Shared {
    /// Variable writes, oldest first.
    pub writes: Vec<(&'static str, f64)>,
    pub images: [Option<Arc<NativeImage>>; 2],
    pub log: VecDeque<String>,
    /// How long the last map cycles took, ms (ND left, ND right).
    pub cycle_ms: [f64; 2],
}

struct SideRendering {
    timeout: Option<f64>,
    interval: Option<f64>,
    navigation_display: NavigationDisplayRenderer,
    rendered_last_frame_navigation_display: bool,
    vertical_display: VerticalDisplayRenderer,
    rendered_last_frame_vertical_display: bool,
    frames_in_cycle: usize,
    vertical_display_rendered_on_side: bool,
    navigation_display_rendered_on_side: bool,
}

pub struct TerrainWorker {
    clock: Instant,
    initialized: bool,
    sim_paused: bool,
    rendering_mode: u32,
    manual_azim_enabled: bool,
    manual_azim_degrees: f64,
    manual_azim_end_point: Option<(f64, f64)>,
    current_track_changes_significantly_at_distance: [f64; 2],
    simbridge_client_used: bool,
    simbridge_client_timeout: Option<f64>,
    map_handler: MapHandler,
    display_width: usize,
    display_height: usize,
    vertical_display_required: bool,
    sides: [SideRendering; 2],
    gauge: Gauge,
    shared: Arc<Mutex<Shared>>,
}

fn js_sign(v: f64) -> f64 {
    if v > 0. {
        1.
    } else if v < 0. {
        -1.
    } else {
        v
    }
}

impl TerrainWorker {
    pub fn new(provider: Arc<dyn TileProvider>, loader_threads: usize, shared: Arc<Mutex<Shared>>) -> Self {
        let clock = Instant::now();
        let startup_time = 0.;
        let side = |startup: f64| SideRendering {
            timeout: None,
            interval: None,
            navigation_display: NavigationDisplayRenderer::new(startup),
            rendered_last_frame_navigation_display: false,
            vertical_display: VerticalDisplayRenderer::new(startup),
            rendered_last_frame_vertical_display: false,
            frames_in_cycle: 0,
            vertical_display_rendered_on_side: false,
            navigation_display_rendered_on_side: false,
        };
        let mut out = GaugeOutput::default();
        let gauge = Gauge::new(&mut out);
        let mut worker = Self {
            clock,
            initialized: false,
            sim_paused: true,
            rendering_mode: ARC_MODE,
            manual_azim_enabled: true,
            manual_azim_degrees: 0.,
            manual_azim_end_point: None,
            current_track_changes_significantly_at_distance: [-1., -1.],
            simbridge_client_used: false,
            simbridge_client_timeout: None,
            map_handler: MapHandler::new(provider, loader_threads),
            display_width: 0,
            display_height: 0,
            vertical_display_required: false,
            // offset the rendering to have a more realistic bahaviour
            sides: [side(startup_time), side(startup_time - 1500.)],
            gauge,
            shared,
        };
        worker.publish(out);
        worker.initialize();
        worker
    }

    fn now(&self) -> f64 {
        self.clock.elapsed().as_secs_f64() * 1000.
    }

    /// The constructor's `mapHandler.initialize().then(...)`.
    fn initialize(&mut self) {
        self.map_handler.initialize();
        let config = |nd_range: f64| {
            let mut c = EfisData::new(nd_range, true, false, false, 0., -500., 24000.);
            c.map_width = NAVIGATION_DISPLAY_MAX_PIXEL_WIDTH;
            c.map_height = NAVIGATION_DISPLAY_MAX_PIXEL_HEIGHT;
            c
        };
        let startup_status = AircraftStatus {
            adiru_data_valid: true,
            taws_inop: false,
            latitude: 47.26081085205078,
            longitude: 11.349658966064453,
            altitude: 1904.,
            heading: 260.,
            vertical_speed: 0.,
            gear_is_down: true,
            runway_data_valid: false,
            runway_latitude: 0.,
            runway_longitude: 0.,
            efis_data_capt: config(20.),
            efis_data_fo: config(10.),
            navigation_display_rendering_mode: ARC_MODE,
            manual_azim_enabled: false,
            manual_azim_degrees: 0.,
            ground_truth_latitude: 47.26081085205078,
            ground_truth_longitude: 11.349658966064453,
        };
        let now = self.now();
        for side in Side::BOTH {
            let s = &mut self.sides[side.index()];
            s.navigation_display.aircraft_status_update(&startup_status, side);
            s.vertical_display.aircraft_status_update(&startup_status, side);
            // initialize(): a first map cycle, with no map yet.
            s.navigation_display.start_new_map_cycle(&self.map_handler, now);
            s.vertical_display.path_data_update(&VerticalPathData { path_width: 1., track_changes_significantly_at_distance: -1., waypoints: vec![(47.26081085205078, 11.349658966064453)] });
            s.vertical_display.start_new_map_cycle(&self.map_handler, now);
        }
        self.map_handler.reset();
        for s in &mut self.sides {
            s.navigation_display.reset();
            s.vertical_display.reset(true);
        }
        self.initialized = true;
    }

    fn on_reset(&mut self) {
        if !self.initialized {
            return;
        }
        self.map_handler.reset();
        for s in &mut self.sides {
            s.navigation_display.reset();
            s.vertical_display.reset(true);
        }
    }

    fn enable_simbridge_client_data(&mut self) {
        if !self.simbridge_client_used {
            self.log("SimBridge client data received, ignoring SimConnect aircraftStatusUpdate from now on.".into());
        }
        self.simbridge_client_used = true;
    }

    fn disable_simbridge_client_data(&mut self) {
        if self.simbridge_client_used {
            self.log("SimBridge client data stopped (due to timeout), resuming SimConnect aircraftStatusUpdate.".into());
        }
        self.simbridge_client_used = false;
    }

    fn log(&self, message: String) {
        let mut shared = self.shared.lock().unwrap_or_else(|e| e.into_inner());
        if shared.log.len() == 64 {
            shared.log.pop_front();
        }
        shared.log.push_back(message);
    }

    fn update_rendering(&mut self, side: Side, status: &AircraftStatus) {
        let configuration = status.efis(side);
        let i = side.index();
        let last_config = self.sides[i].navigation_display.display_configuration();
        let config_changed = last_config.is_some_and(|l| {
            l.efis_mode != configuration.efis_mode
                || l.nd_range != configuration.nd_range
                || l.arc_mode != configuration.arc_mode
                || l.terr_on_nd != configuration.terr_on_nd
                || l.terr_on_vd != configuration.terr_on_vd
        });
        let stop_rendering = last_config.is_some_and(|l| (l.terr_on_nd && !configuration.terr_on_nd) || (l.terr_on_vd && !configuration.terr_on_vd));
        // this.manualAzimEnabled was set from the status just before.
        let start_rendering = config_changed || self.manual_azim_enabled != status.manual_azim_enabled || last_config.is_none();

        if stop_rendering || start_rendering {
            self.reset_rendering_cycle(side, false);
        }
        self.sides[i].navigation_display.aircraft_status_update(status, side);
        self.sides[i].vertical_display.aircraft_status_update(status, side);

        if self.manual_azim_enabled {
            let end = project_wgs84(status.latitude, status.longitude, status.heading, 160. * NAUTICAL_MILES_TO_METRES);
            self.manual_azim_end_point = Some(end);
            self.sides[i].vertical_display.path_data_update(&VerticalPathData { path_width: 1., track_changes_significantly_at_distance: -1., waypoints: vec![end] });
        }
        if start_rendering {
            self.start_navigation_display_rendering_cycle(side);
        }
    }

    fn update_path_data(&mut self, side: Side, path: &VerticalPathData) {
        let i = side.index();
        let current = self.current_track_changes_significantly_at_distance[i];
        let force_redraw = self.sides[i].navigation_display.display_configuration().is_none()
            || self.sides[i].vertical_display.num_path_elements() != path.waypoints.len()
            || (path.track_changes_significantly_at_distance - current).abs() > 0.1
            || js_sign(path.track_changes_significantly_at_distance) != js_sign(current);
        if force_redraw {
            self.reset_rendering_cycle(side, true);
        }
        if self.manual_azim_enabled || path.waypoints.is_empty() {
            let waypoints = self.manual_azim_end_point.map(|p| vec![p]).unwrap_or_default();
            self.sides[i].vertical_display.path_data_update(&VerticalPathData { path_width: 1., track_changes_significantly_at_distance: -1., waypoints });
        } else {
            self.sides[i].vertical_display.path_data_update(path);
        }
        self.current_track_changes_significantly_at_distance[i] = path.track_changes_significantly_at_distance;
        if force_redraw {
            self.start_navigation_display_rendering_cycle(side);
        }
    }

    fn on_aircraft_status_update(&mut self, data: &AircraftStatus) {
        if !self.initialized {
            return;
        }
        self.vertical_display_required = data.navigation_display_rendering_mode & VERTICAL_DISPLAY_REQUIRED == VERTICAL_DISPLAY_REQUIRED;
        self.rendering_mode = data.navigation_display_rendering_mode & (ARC_MODE | SCANLINE_MODE);
        self.manual_azim_enabled = data.manual_azim_enabled;
        self.manual_azim_degrees = data.manual_azim_degrees;
        self.manual_azim_end_point = data
            .manual_azim_enabled
            .then(|| project_wgs84(data.latitude, data.longitude, self.manual_azim_degrees, 160. * NAUTICAL_MILES_TO_METRES));
        self.display_height = if self.vertical_display_required {
            DISPLAY_SCREEN_PIXEL_HEIGHT_WITH_VERTICAL_DISPLAY
        } else {
            DISPLAY_SCREEN_PIXEL_HEIGHT_WITHOUT_VERTICAL_DISPLAY
        };
        self.display_width = NAVIGATION_DISPLAY_MAX_PIXEL_WIDTH;
        self.map_handler.aircraft_status_update(data);
        self.update_rendering(Side::Left, data);
        self.update_rendering(Side::Right, data);
    }

    fn on_vertical_path_data_update(&mut self, data: &VerticalPathData) {
        if !self.initialized {
            return;
        }
        self.update_path_data(Side::Left, data);
        self.update_path_data(Side::Right, data);
    }

    /// `createScreenResolutionFrame`.
    fn create_screen_resolution_frame(&self, side: Side, navigation_display: Option<&[u8]>, vertical_display: Option<&[u8]>) -> Frame {
        let (width, height) = (self.display_width, self.display_height);
        let mut result = CLEAR.repeat(width * height);
        let s = &self.sides[side.index()];
        if let (Some(source), Some(config)) = (navigation_display, s.navigation_display.display_configuration()) {
            for y in 0..config.map_height {
                let row = NAVIGATION_DISPLAY_MAP_START_OFFSET_Y + y;
                if row >= height {
                    break;
                }
                let destination = (row * width + config.map_offset_x as usize) * 4;
                let source_row = y * config.map_width * 4;
                result[destination..destination + config.map_width * 4].copy_from_slice(&source[source_row..source_row + config.map_width * 4]);
            }
        }
        // add the vertical display map
        if let Some(source) = vertical_display {
            let config = s.vertical_display.display_configuration();
            for y in 0..config.map_height {
                let row = VERTICAL_DISPLAY_MAP_START_OFFSET_Y + y;
                if row >= height {
                    break;
                }
                let destination = (row * width + VERTICAL_DISPLAY_MAP_START_OFFSET_X) * 4;
                let source_row = y * config.map_width * 4;
                result[destination..destination + config.map_width * 4].copy_from_slice(&source[source_row..source_row + config.map_width * 4]);
            }
        }
        Frame { width, height, rgba: result }
    }

    /// `resetRenderingCycle`.
    fn reset_rendering_cycle(&mut self, side: Side, only_redraw: bool) {
        let s = &mut self.sides[side.index()];
        s.interval = None;
        s.timeout = None;
        if !only_redraw {
            s.navigation_display.reset();
            s.vertical_display.reset(false);
        }
        // reset also the aircraft data
        let data = s.navigation_display.display_data();
        self.send_to_gauge(side, &data, None);
    }

    /// `startNavigationDisplayRenderingCycle`.
    fn start_navigation_display_rendering_cycle(&mut self, side: Side) {
        let now = self.now();
        let t = Instant::now();
        let i = side.index();
        let Some(config) = self.sides[i].navigation_display.display_configuration() else { return };
        let vertical_display_rendered_on_side = self.vertical_display_required && config.terr_on_vd && (config.efis_mode == 2. || config.efis_mode == 3.);
        let navigation_display_rendered_on_side = config.terr_on_nd;
        let s = &mut self.sides[i];
        s.timeout = None;
        s.interval = None;
        s.vertical_display_rendered_on_side = vertical_display_rendered_on_side;
        s.navigation_display_rendered_on_side = navigation_display_rendered_on_side;
        s.rendered_last_frame_navigation_display = false;
        s.rendered_last_frame_vertical_display = false;
        s.navigation_display.start_new_map_cycle(&self.map_handler, now);
        if vertical_display_rendered_on_side {
            s.vertical_display.start_new_map_cycle(&self.map_handler, now);
        }
        s.frames_in_cycle = 0;
        s.interval = Some(now + RENDERING_MAP_TRANSITION_DELTA_TIME as f64);
        let took = t.elapsed().as_secs_f64() * 1000.;
        self.shared.lock().unwrap_or_else(|e| e.into_inner()).cycle_ms[i] = took;
    }

    /// The `setInterval` callback of a rendering cycle.
    fn rendering_interval(&mut self, side: Side) {
        let i = side.index();
        let now = self.now();
        let s = &mut self.sides[i];
        if !s.rendered_last_frame_navigation_display {
            s.rendered_last_frame_navigation_display = s.navigation_display.render();
        }
        if s.vertical_display_rendered_on_side {
            if !s.rendered_last_frame_vertical_display {
                s.rendered_last_frame_vertical_display = s.vertical_display.render();
            }
        } else {
            s.rendered_last_frame_vertical_display = true;
        }
        let s = &self.sides[i];
        let nd_map = s.navigation_display_rendered_on_side.then(|| s.navigation_display.current_frame()).flatten();
        let vd_map = s.vertical_display_rendered_on_side.then(|| s.vertical_display.current_frame()).flatten();
        let frame = self.create_screen_resolution_frame(side, nd_map, vd_map);

        if !self.sim_paused {
            let mut display_data = self.sides[i].navigation_display.display_data();
            display_data.frame_byte_count = frame.rgba.len() as u32;
            display_data.first_frame = self.sides[i].frames_in_cycle == 0;
            self.send_to_gauge(side, &display_data, Some(Arc::new(frame)));
            self.sides[i].frames_in_cycle += 1;
        }

        let s = &mut self.sides[i];
        s.interval = Some(now + RENDERING_MAP_TRANSITION_DELTA_TIME as f64);
        if s.rendered_last_frame_navigation_display && s.rendered_last_frame_vertical_display {
            s.interval = None;
            s.timeout = None;
            let active = s.navigation_display.display_configuration().is_some_and(|c| c.terr_on_nd || c.terr_on_vd);
            if active {
                let timeout = if self.rendering_mode == ARC_MODE { RENDERING_MAP_UPDATE_TIMEOUT_ARC_MODE } else { RENDERING_MAP_UPDATE_TIMEOUT_SCANLINE_MODE };
                s.timeout = Some(now + timeout as f64);
            }
        }
    }

    /// SimBridge's `sendNavigationDisplayTerrainMapMetadata` and
    /// `sendNavigationDisplayTerrainMapFrame`, received by the gauge.
    fn send_to_gauge(&mut self, side: Side, data: &NavigationDisplayData, frame: Option<Arc<Frame>>) {
        let mut out = GaugeOutput::default();
        self.gauge.receive(side, &ThresholdData::from(data), frame, &mut out);
        self.publish(out);
    }

    fn publish(&mut self, out: GaugeOutput) {
        if let Some(packet) = &out.status_packet {
            self.on_status_packet(packet);
        }
        let mut shared = self.shared.lock().unwrap_or_else(|e| e.into_inner());
        shared.writes.extend(out.writes);
        for (side, image) in out.images {
            shared.images[side.index()] = Some(Arc::new(image));
        }
        for line in out.log {
            if shared.log.len() == 64 {
                shared.log.pop_front();
            }
            shared.log.push_back(line);
        }
    }

    /// `simConnectReceivedClientData` for `FBW_SIMBRIDGE_EGPWC_AIRCRAFT_STATUS`.
    fn on_status_packet(&mut self, b: &[u8]) {
        let f32_at = |o: usize| f32::from_le_bytes(b[o..o + 4].try_into().unwrap()) as f64;
        let u16_at = |o: usize| u16::from_le_bytes(b[o..o + 2].try_into().unwrap()) as f64;
        let i16_at = |o: usize| i16::from_le_bytes(b[o..o + 2].try_into().unwrap()) as f64;
        if self.initialized {
            self.map_handler.position_update(f32_at(38), f32_at(42));
        }
        if self.simbridge_client_used {
            return;
        }
        let (lat, lon) = (f32_at(1), f32_at(5));
        let (terr_capt, terr_fo) = (b[30] != 0, b[35] != 0);
        let heading = i16_at(13);
        let status = AircraftStatus {
            adiru_data_valid: b[0] != 0,
            taws_inop: false,
            latitude: lat,
            longitude: lon,
            altitude: i32::from_le_bytes(b[9..13].try_into().unwrap()) as f64,
            heading,
            vertical_speed: i16_at(15),
            gear_is_down: b[17] != 0,
            runway_data_valid: b[18] != 0,
            runway_latitude: f32_at(19),
            runway_longitude: f32_at(23),
            efis_data_capt: EfisData::new(u16_at(27), b[29] != 0, terr_capt, terr_capt, b[31] as f64, -500., 24000.),
            efis_data_fo: EfisData::new(u16_at(32), b[34] != 0, terr_fo, terr_fo, b[36] as f64, -500., 24500.),
            navigation_display_rendering_mode: b[37] as u32,
            manual_azim_enabled: true,
            manual_azim_degrees: heading,
            ground_truth_latitude: lat,
            ground_truth_longitude: lon,
        };
        self.on_aircraft_status_update(&status);
    }

    fn handle(&mut self, message: Message) {
        match message {
            Message::Gauge(inputs) => {
                let mut out = GaugeOutput::default();
                self.gauge.draw(Instant::now(), &inputs, &mut out);
                self.publish(out);
            }
            Message::Paused(paused) => self.sim_paused = paused,
            Message::Reset => self.on_reset(),
            Message::AircraftStatusData(status) => {
                self.enable_simbridge_client_data();
                self.on_aircraft_status_update(&status);
                // Re-start timeout for disabling the SimBridge client data after two minutes of inactivity
                self.simbridge_client_timeout = Some(self.now() + SIMBRIDGE_CLIENT_DATA_TIMEOUT);
            }
            Message::VerticalDisplayPath(path) => self.on_vertical_path_data_update(&path),
        }
    }

    fn next_deadline(&self) -> Option<f64> {
        self.sides
            .iter()
            .flat_map(|s| [s.interval, s.timeout])
            .chain([self.simbridge_client_timeout])
            .flatten()
            .min_by(f64::total_cmp)
    }

    fn run_timers(&mut self) {
        let now = self.now();
        if self.simbridge_client_timeout.is_some_and(|t| t <= now) {
            self.simbridge_client_timeout = None;
            self.disable_simbridge_client_data();
        }
        for side in Side::BOTH {
            let i = side.index();
            if self.sides[i].interval.is_some_and(|t| t <= now) {
                self.rendering_interval(side);
            }
            if self.sides[i].timeout.is_some_and(|t| t <= now) {
                self.sides[i].timeout = None;
                self.start_navigation_display_rendering_cycle(side);
            }
        }
    }

    /// The thread's loop, until every sender is gone.
    pub fn run(mut self, messages: Receiver<Message>) {
        loop {
            let wait = self.next_deadline().map_or(Duration::from_millis(250), |d| Duration::from_secs_f64(((d - self.now()) / 1000.).max(0.)));
            match messages.recv_timeout(wait) {
                Ok(message) => self.handle(message),
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => return,
            }
            self.idle();
        }
    }

    /// Timers, and tiles that have arrived.
    fn idle(&mut self) {
        self.run_timers();
        self.map_handler.poll();
        let errors = std::mem::take(&mut self.map_handler.errors);
        for e in errors {
            self.log(e);
        }
    }
}

/// Start the worker thread.
pub fn spawn(provider: Arc<dyn TileProvider>, loader_threads: usize) -> (Sender<Message>, Arc<Mutex<Shared>>) {
    let (tx, rx) = std::sync::mpsc::channel();
    let shared = Arc::new(Mutex::new(Shared::default()));
    let for_thread = shared.clone();
    let _ = std::thread::Builder::new().name("fbw-terrain".into()).spawn(move || {
        TerrainWorker::new(provider, loader_threads, for_thread).run(rx);
    });
    (tx, shared)
}

#[cfg(test)]
mod tests {
    use super::super::worldmap::tests::Slope;
    use super::*;

    fn word(value: f32) -> f64 {
        ((0b11u64 << 32) | value.to_bits() as u64) as f64
    }

    /// The A380X at 47 N 11 E, 3000 ft above the terrain's highest point,
    /// TERR on the captain's ND in ROSE NAV, 40 nm.
    fn inputs(terrain: f64) -> GaugeInputs {
        GaugeInputs {
            status: [0., 0., word(47.26), word(11.35), 3., word(8000.), word(90.), word(0.), 1.],
            nd: [40., 2., terrain, 40., 2., 0., 1., 1.],
            simulator: [47.26, 11.35, 1., 1.],
        }
    }

    #[test]
    fn the_captains_nd_gets_terrain_frames_and_thresholds() {
        let shared = Arc::new(Mutex::new(Shared::default()));
        let mut worker = TerrainWorker::new(Arc::new(Slope), 4, shared.clone());
        worker.handle(Message::Paused(false));
        let start = Instant::now();
        let mut stitched = Duration::ZERO;
        // Wait for the tiles, as the gauge keeps sending its status.
        while worker.map_handler.loading() || !worker.map_handler.has_aircraft_status() {
            let t = Instant::now();
            worker.handle(Message::Gauge(inputs(0.)));
            worker.idle();
            stitched = stitched.max(t.elapsed());
            std::thread::sleep(Duration::from_millis(110));
            assert!(start.elapsed().as_secs() < 120);
        }
        // TERR ON: a new cycle; run it to its end.
        worker.handle(Message::Gauge(inputs(1.)));
        std::thread::sleep(Duration::from_millis(250));
        worker.handle(Message::Gauge(inputs(1.)));
        let cycle_start = Instant::now();
        while cycle_start.elapsed().as_millis() < 1500 {
            worker.handle(Message::Gauge(inputs(1.)));
            worker.idle();
            std::thread::sleep(Duration::from_millis(10));
        }
        let shared = shared.lock().unwrap();
        println!("stitching and status handling at most {stitched:?}; ND map cycle {:?} ms", shared.cycle_ms);
        let image = shared.images[0].as_ref().expect("the captain's gauge image");
        assert_eq!((image.width, image.height), (768, 1024));
        // The terrain is 1000 to 1300 ft below 8000 ft: peaks mode, drawn green or clear, over black.
        let green = image.rgba.chunks(4).filter(|p| p[1] > 0 && p[0] == 0).count();
        assert!(green > 1000, "{green} green pixels");
        let max = shared.writes.iter().rev().find(|(n, _)| *n == "A32NX_EGPWC_ND_L_TERRAIN_MAX_ELEVATION").expect("thresholds");
        assert!(max.1 >= 1000., "{max:?}");
    }
}

#[cfg(test)]
mod xplane_tests {
    use crate::mapdata::scenery::Scenery;
    use super::super::tiles::TileSource;
    use super::*;

    fn word(value: f32) -> f64 {
        ((0b11u64 << 32) | value.to_bits() as u64) as f64
    }

    /// The captain's ND over the Alps from X-Plane's own scenery: timings,
    /// and the gauge image written to D:/fbw-build/terrain-nd-left.ppm to
    /// look at. Needs X-Plane; the first run converts every tile within
    /// 800 nm into D:/fbw-build/terrain-cache.
    #[test]
    #[ignore]
    fn alps_from_xplane_scenery() {
        let root = std::path::Path::new("D:/Steam Games/steamapps/common/X-Plane 12");
        let t = Instant::now();
        let provider = Arc::new(TileSource::new(Scenery::of_installation(root), Some("D:/fbw-build/terrain-cache".into())));
        let shared = Arc::new(Mutex::new(Shared::default()));
        let mut worker = TerrainWorker::new(provider, 3, shared.clone());
        worker.handle(Message::Paused(false));
        // Near Innsbruck, 9000 ft, heading 260, ARC 40 nm, TERR ON ND.
        let inputs = GaugeInputs {
            status: [word(47.26), word(11.34), word(47.35), word(11.9), 3., word(9000.), word(260.), word(-500.), 0.],
            nd: [40., 3., 1., 40., 3., 1., 1., 1.],
            simulator: [47.35, 11.9, 1., 1.],
        };
        let mut slowest = Duration::ZERO;
        while worker.map_handler.loading() || !worker.map_handler.has_aircraft_status() {
            let step = Instant::now();
            worker.handle(Message::Gauge(inputs));
            worker.idle();
            slowest = slowest.max(step.elapsed());
            std::thread::sleep(Duration::from_millis(50));
            assert!(t.elapsed().as_secs() < 1200, "tiles did not load");
        }
        println!("tiles loaded in {:?}; slowest worker step {slowest:?}", t.elapsed());
        // A new cycle with the full map, run to its end.
        worker.sides[0].timeout = Some(worker.now());
        let cycle = Instant::now();
        while cycle.elapsed().as_millis() < 1500 {
            worker.handle(Message::Gauge(inputs));
            worker.idle();
            std::thread::sleep(Duration::from_millis(5));
        }
        let shared = shared.lock().unwrap();
        println!("map cycle {:?} ms; errors {:?}", shared.cycle_ms, shared.log);
        let image = shared.images[0].as_ref().expect("an image");
        let mut ppm = format!("P6 {} {} 255\n", image.width, image.height).into_bytes();
        for p in image.rgba.chunks(4) {
            ppm.extend(&p[0..3]);
        }
        std::fs::write("D:/fbw-build/terrain-nd-left.ppm", ppm).unwrap();
        let thresholds: Vec<_> = shared.writes.iter().rev().take(4).collect();
        println!("last threshold writes {thresholds:?}");
    }
}
