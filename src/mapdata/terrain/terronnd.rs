//! FlyByWire's terronnd gauge (fbw-common `src/wasm/terronnd`), which
//! panel.cfg runs under each ND's nd.html (`[VCockpit07]`/`[VCockpit08]`:
//! `htmlgauge00=WasmInstrument/...terronnd.wasm...,L`, then
//! `htmlgauge01=A380X/ND/nd.html`). It sends the EGPWC's status to the
//! terrain renderer, writes the thresholds the ND prints, and draws the
//! renderer's frame as an image with the ND drawn over it.
//!
//! Only its MSFS interfaces change: the named variables come from the
//! plugin each frame instead of `get_named_variable_value`, the client data
//! areas are messages to and from the renderer on the same thread instead
//! of SimConnect, frames arrive as RGBA instead of PNG (SimBridge encodes
//! and terronnd decodes losslessly), and NanoVG's drawing becomes the
//! compositing below, done here off X-Plane's thread.

use std::sync::Arc;
use std::time::Instant;

use super::types::{Side, ThresholdData};

/// `collection.cpp` builds the A380X with `VD_ALWAYS_ACTIVE 1`.
const VD_ALWAYS_ACTIVE: bool = true;
/// `displaybase.cpp`: `INSTRUMENT_BG_COLOR nvgRGBA(0, 0, 0, 255)` for the A380X.
const INSTRUMENT_BG_COLOR: [u8; 4] = [0, 0, 0, 255];
/// `display.h`: `MaxFrameByteCount`.
const MAX_FRAME_BYTE_COUNT: u32 = 4 * 1024 * 1024;
/// The gauge's size in panel.cfg (`0,0,768,1024`).
pub const GAUGE_WIDTH: usize = 768;
pub const GAUGE_HEIGHT: usize = 1024;

// configuration.h
const ROSE_LS: u8 = 0;
const ROSE_VOR: u8 = 1;
const ROSE_NAV: u8 = 2;
const ARC: u8 = 3;

/// The variables terronnd reads, by the names it registers (`A32NX_` +
/// configuration.h) and the simulator variables of its `SimulatorData`.
pub const LVAR_STATUS: [&str; 9] = [
    "A32NX_EGPWC_DEST_LAT",
    "A32NX_EGPWC_DEST_LONG",
    "A32NX_EGPWC_PRESENT_LAT",
    "A32NX_EGPWC_PRESENT_LONG",
    "A32NX_EGPWC_TERRONND_RENDERING_MODE",
    "A32NX_EGPWC_PRESENT_ALTITUDE",
    "A32NX_EGPWC_PRESENT_HEADING",
    "A32NX_EGPWC_PRESENT_VERTICAL_SPEED",
    "A32NX_EGPWC_GEAR_IS_DOWN",
];
pub const LVAR_ND: [&str; 8] = [
    "A32NX_EGPWC_ND_L_RANGE",
    "A32NX_EFIS_L_ND_MODE",
    "A32NX_EGPWC_ND_L_TERRAIN_ACTIVE",
    "A32NX_EGPWC_ND_R_RANGE",
    "A32NX_EFIS_R_ND_MODE",
    "A32NX_EGPWC_ND_R_TERRAIN_ACTIVE",
    "A32NX_ELEC_AC_ESS_BUS_IS_POWERED",
    "A32NX_ELEC_AC_2_BUS_IS_POWERED",
];
/// The thresholds each display writes, left then right: min, min mode, max, max mode.
pub const LVAR_THRESHOLDS: [[&str; 4]; 2] = [
    [
        "A32NX_EGPWC_ND_L_TERRAIN_MIN_ELEVATION",
        "A32NX_EGPWC_ND_L_TERRAIN_MIN_ELEVATION_MODE",
        "A32NX_EGPWC_ND_L_TERRAIN_MAX_ELEVATION",
        "A32NX_EGPWC_ND_L_TERRAIN_MAX_ELEVATION_MODE",
    ],
    [
        "A32NX_EGPWC_ND_R_TERRAIN_MIN_ELEVATION",
        "A32NX_EGPWC_ND_R_TERRAIN_MIN_ELEVATION_MODE",
        "A32NX_EGPWC_ND_R_TERRAIN_MAX_ELEVATION",
        "A32NX_EGPWC_ND_R_TERRAIN_MAX_ELEVATION_MODE",
    ],
];

/// What terronnd reads in one frame.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GaugeInputs {
    pub status: [f64; 9],
    pub nd: [f64; 8],
    /// `PLANE LATITUDE`, `PLANE LONGITUDE` (degrees) and the two `LIGHT
    /// POTENTIOMETER`s (percent over 100).
    pub simulator: [f64; 4],
}

/// `types::Arinc429Word<T>::fromSimVar(simVar, factor)`: the value's float
/// and whether its SSM is normal operation.
fn arinc429(sim_var: f64) -> (f32, bool) {
    let q = sim_var as u64;
    (f32::from_bits((q & 0xffff_ffff) as u32), (q >> 32) as u32 == 0b11)
}

/// `helper::Math::almostEqual` for doubles.
fn almost_equal(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-4
}

/// `LVarObjectBase`: read every `cycle` ms, changed when any value moved.
struct LVarObject<const N: usize> {
    values: [f64; N],
    last_update: Option<Instant>,
    cycle_ms: u128,
}

impl<const N: usize> LVarObject<N> {
    fn new(cycle_ms: u128) -> Self {
        Self { values: [0.; N], last_update: None, cycle_ms }
    }

    fn read(&mut self, now: Instant, inputs: &[f64; N]) -> bool {
        if self.last_update.is_some_and(|t| now.duration_since(t).as_millis() < self.cycle_ms) {
            return false;
        }
        self.last_update = Some(now);
        let mut changed = false;
        for (v, &input) in self.values.iter_mut().zip(inputs) {
            if !almost_equal(input, *v) {
                *v = input;
                changed = true;
            }
        }
        changed
    }
}

/// `DisplayBase::NdConfiguration`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct NdConfiguration {
    /// nm
    range: f32,
    mode: u8,
    terr_on_nd: bool,
    terr_on_vd: bool,
    potentiometer: f32,
    powered: bool,
}

/// A frame as the renderer sends it.
pub struct Frame {
    pub width: usize,
    pub height: usize,
    pub rgba: Vec<u8>,
}

/// The gauge's image, ready to draw under the ND.
///
/// `rgba` is `Arc<[u8]>` rather than `Vec<u8>` so a consumer that only wants
/// a shared, read-only handle (e.g. the XPHFBW screen compositor's
/// `mapdata::terrain_layer`, docs on that function) can clone
/// it for free instead of copying the buffer; the pixels are never mutated
/// again once a `Display::render` finishes building them.
pub struct NativeImage {
    pub width: u32,
    pub height: u32,
    /// Counts up with every change.
    pub generation: u64,
    /// Straight RGBA, row 0 at the top.
    pub rgba: Arc<[u8]>,
}

/// `navigationdisplay::Display` for one side.
struct Display {
    side: Side,
    configuration: NdConfiguration,
    frame_buffer_size: u32,
    image: Option<Arc<Frame>>,
    ignore_next_frame: bool,
    /// The threshold variables' values (`_ndThresholdData`).
    thresholds: [f64; 4],
    last_drawn: Option<(NdConfiguration, usize)>,
    generation: u64,
}

impl Display {
    fn new(side: Side, writes: &mut Vec<(&'static str, f64)>) -> Self {
        let mut d = Self {
            side,
            configuration: NdConfiguration::default(),
            frame_buffer_size: 0,
            image: None,
            ignore_next_frame: false,
            thresholds: [0.; 4],
            last_drawn: None,
            generation: 0,
        };
        // write initial values to avoid invalid drawings
        d.reset_navigation_display_data(writes);
        d
    }

    fn reset_navigation_display_data(&mut self, writes: &mut Vec<(&'static str, f64)>) {
        self.thresholds = [-1., 0., -1., 0.];
        self.write_values(writes);
    }

    fn write_values(&self, writes: &mut Vec<(&'static str, f64)>) {
        for (name, value) in LVAR_THRESHOLDS[self.side.index()].iter().zip(self.thresholds) {
            writes.push((name, value));
        }
    }

    /// The thresholds client data callback.
    fn on_thresholds(&mut self, data: &ThresholdData, writes: &mut Vec<(&'static str, f64)>) -> Option<String> {
        let frame_byte_count = data.frame_byte_count;
        if frame_byte_count == 0 || frame_byte_count > MAX_FRAME_BYTE_COUNT {
            // corrupted or incompatible packet: allocating this size could kill the module
            self.frame_buffer_size = 0;
            return Some(format!("TERR ON ND: Ignoring thresholds packet with implausible frame size: {frame_byte_count}"));
        }
        self.frame_buffer_size = frame_byte_count;
        self.ignore_next_frame = self.ignore_next_frame
            && (data.first_frame == 0 || self.configuration.mode != data.display_mode || self.configuration.range != data.display_range as f32);
        if !self.ignore_next_frame {
            self.thresholds = [
                data.lower_threshold as f64,
                data.lower_threshold_mode as f64,
                data.upper_threshold as f64,
                data.upper_threshold_mode as f64,
            ];
            self.write_values(writes);
        }
        None
    }

    /// The frame data client data callback.
    fn on_frame(&mut self, frame: Arc<Frame>, writes: &mut Vec<(&'static str, f64)>) -> Option<String> {
        if !self.ignore_next_frame && (self.configuration.terr_on_nd || self.configuration.terr_on_vd) {
            if let Some(image) = &self.image {
                if frame.width != image.width || frame.height != image.height {
                    // This should never happen, but bail just in case
                    return Some(format!(
                        "TERR ON ND: The image size does not match the expected size. Expected: {}x{}, actual: {}x{}",
                        image.width, image.height, frame.width, frame.height
                    ));
                }
            }
            self.image = Some(frame);
        } else {
            self.reset_navigation_display_data(writes);
        }
        None
    }

    /// `Display::update`.
    fn update(&mut self, config: NdConfiguration, writes: &mut Vec<(&'static str, f64)>) {
        let reset_map_data = self.configuration.mode != config.mode
            || config.range != self.configuration.range
            || self.configuration.terr_on_nd != config.terr_on_nd
            || self.configuration.terr_on_vd != config.terr_on_vd;
        let valid_efis_mode = matches!(config.mode, ARC | ROSE_LS | ROSE_NAV | ROSE_VOR);
        self.configuration = config;
        self.configuration.terr_on_nd &= valid_efis_mode;
        if !(self.configuration.terr_on_nd || self.configuration.terr_on_vd) || !valid_efis_mode || reset_map_data {
            self.reset_navigation_display_data(writes);
            self.image = None;
            self.ignore_next_frame = true;
        }
    }

    /// `DisplayBase::render`: the gauge's picture, redrawn when it would change.
    fn render(&mut self) -> Option<NativeImage> {
        let image_key = self.image.as_ref().map_or(0, |i| Arc::as_ptr(i) as usize);
        if self.last_drawn == Some((self.configuration, image_key)) {
            return None;
        }
        self.last_drawn = Some((self.configuration, image_key));
        let (w, h) = (GAUGE_WIDTH, GAUGE_HEIGHT);
        let mut rgba = if self.configuration.powered { INSTRUMENT_BG_COLOR.repeat(w * h) } else { [0, 0, 0, 255].repeat(w * h) };
        if self.configuration.powered && self.configuration.potentiometer.abs() > 1e-4 {
            if let Some(image) = &self.image {
                // nvgImagePattern(0, 0, winWidth, winHeight, 0, image, potentiometer)
                // over the background: NanoVG premultiplies the image and
                // scales it by the paint's alpha.
                let alpha = self.configuration.potentiometer;
                let (sx, sy) = (image.width as f32 / w as f32, image.height as f32 / h as f32);
                for y in 0..h {
                    let iy = ((y as f32 + 0.5) * sy) as usize;
                    for x in 0..w {
                        let ix = ((x as f32 + 0.5) * sx) as usize;
                        let s = &image.rgba[(iy * image.width + ix) * 4..(iy * image.width + ix) * 4 + 4];
                        let a = s[3] as f32 / 255. * alpha;
                        let d = &mut rgba[(y * w + x) * 4..(y * w + x) * 4 + 4];
                        for c in 0..3 {
                            d[c] = (s[c] as f32 * a + d[c] as f32 * (1. - a)).round().clamp(0., 255.) as u8;
                        }
                        d[3] = ((a + d[3] as f32 / 255. * (1. - a)) * 255.).round().clamp(0., 255.) as u8;
                    }
                }
            }
        }
        self.generation += 1;
        Some(NativeImage { width: w as u32, height: h as u32, generation: self.generation, rgba: rgba.into() })
    }
}

/// What the gauge wants done after a frame.
#[derive(Default)]
pub struct GaugeOutput {
    /// The aircraft status packet for the renderer (46 bytes, `types::AircraftStatusData`).
    pub status_packet: Option<Vec<u8>>,
    pub writes: Vec<(&'static str, f64)>,
    pub images: Vec<(Side, NativeImage)>,
    pub log: Vec<String>,
}

/// `navigationdisplay::Collection` with its two displays.
pub struct Gauge {
    aircraft_status: LVarObject<9>,
    nd_configuration: LVarObject<8>,
    simulator: Option<[f64; 4]>,
    ground_truth: (f32, f32),
    configuration: [NdConfiguration; 2],
    reconfigure: [bool; 2],
    send_aircraft_status: bool,
    last_status_transmission: Option<Instant>,
    displays: [Display; 2],
}

impl Gauge {
    pub fn new(out: &mut GaugeOutput) -> Self {
        Self {
            aircraft_status: LVarObject::new(100),
            nd_configuration: LVarObject::new(200),
            simulator: None,
            ground_truth: (0., 0.),
            configuration: [NdConfiguration::default(); 2],
            reconfigure: [false; 2],
            send_aircraft_status: false,
            last_status_transmission: None,
            displays: [Display::new(Side::Left, &mut out.writes), Display::new(Side::Right, &mut out.writes)],
        }
    }

    /// A thresholds packet, and the frame after it when it has one.
    pub fn receive(&mut self, side: Side, thresholds: &ThresholdData, frame: Option<Arc<Frame>>, out: &mut GaugeOutput) {
        let display = &mut self.displays[side.index()];
        out.log.extend(display.on_thresholds(thresholds, &mut out.writes));
        if let Some(frame) = frame {
            if display.frame_buffer_size > 0 {
                out.log.extend(display.on_frame(frame, &mut out.writes));
            }
        }
    }

    /// `PANEL_SERVICE_PRE_DRAW` of both gauges: `readData`, `updateDisplay`, `renderDisplay`.
    pub fn draw(&mut self, now: Instant, inputs: &GaugeInputs, out: &mut GaugeOutput) {
        for side in Side::BOTH {
            self.read_data(now, inputs);
            self.update_display(now, side, out);
            if let Some(image) = self.displays[side.index()].render() {
                out.images.push((side, image));
            }
        }
    }

    fn read_data(&mut self, now: Instant, inputs: &GaugeInputs) {
        // The simulator object arrives every visual frame and fires when it changed.
        if self.simulator != Some(inputs.simulator) {
            self.simulator = Some(inputs.simulator);
            let s = inputs.simulator;
            self.configuration[0].potentiometer = s[2] as f32;
            self.configuration[1].potentiometer = s[3] as f32;
            self.reconfigure = [true, true];
            let (latitude, longitude) = (s[0] as f32, s[1] as f32);
            if latitude != self.ground_truth.0 || longitude != self.ground_truth.1 {
                self.ground_truth = (latitude, longitude);
                self.send_aircraft_status = true;
            }
        }
        if self.aircraft_status.read(now, &inputs.status) {
            self.send_aircraft_status = true;
        }
        if self.nd_configuration.read(now, &inputs.nd) {
            let v = self.nd_configuration.values;
            for (i, c) in self.configuration.iter_mut().enumerate() {
                let o = i * 3;
                c.range = v[o] as f32;
                c.mode = v[o + 1] as u8;
                c.terr_on_nd = v[o + 2] as u8 != 0;
                c.terr_on_vd = VD_ALWAYS_ACTIVE;
                c.powered = v[6 + i] as u8 != 0;
            }
            self.reconfigure = [true, true];
            self.send_aircraft_status = true;
        }
    }

    fn update_display(&mut self, now: Instant, side: Side, out: &mut GaugeOutput) {
        let dt = self.last_status_transmission.map_or(u128::MAX, |t| now.duration_since(t).as_millis());
        if self.send_aircraft_status && dt >= 100 {
            out.status_packet = Some(self.status_packet());
            self.last_status_transmission = Some(now);
            self.send_aircraft_status = false;
        }
        let i = side.index();
        if self.reconfigure[i] {
            self.displays[i].update(self.configuration[i], &mut out.writes);
            self.reconfigure[i] = false;
        }
    }

    /// `types::AircraftStatusData`, packed.
    fn status_packet(&self) -> Vec<u8> {
        let v = self.aircraft_status.values;
        let (dest_lat, dest_lat_no) = arinc429(v[0]);
        let (dest_lon, dest_lon_no) = arinc429(v[1]);
        let (lat, lat_no) = arinc429(v[2]);
        let (lon, lon_no) = arinc429(v[3]);
        let rendering_mode = v[4] as u8;
        let (altitude, altitude_no) = arinc429(v[5]);
        let (heading, heading_no) = arinc429(v[6]);
        let (vertical_speed, vertical_speed_no) = arinc429(v[7]);
        let gear_is_down = v[8] as u8 != 0;

        let mut p = Vec::with_capacity(46);
        p.push((lat_no && lon_no && altitude_no && heading_no && vertical_speed_no) as u8);
        p.extend(lat.to_le_bytes());
        p.extend(lon.to_le_bytes());
        p.extend((altitude as i32).to_le_bytes());
        p.extend((heading as i16).to_le_bytes());
        p.extend((vertical_speed as i16).to_le_bytes());
        p.push(gear_is_down as u8);
        p.push((dest_lat_no && dest_lon_no) as u8);
        p.extend(dest_lat.to_le_bytes());
        p.extend(dest_lon.to_le_bytes());
        for (i, c) in self.configuration.iter().enumerate() {
            let terrain_map_mode = matches!(c.mode, ROSE_LS | ROSE_VOR | ROSE_NAV | ARC);
            p.extend((c.range as u16).to_le_bytes());
            p.push((c.mode == ARC) as u8);
            // The captain's side sends TERR ON ND, the first officer's TERR on ND or VD (collection.cpp).
            let active = if i == 0 { c.terr_on_nd } else { c.terr_on_nd || c.terr_on_vd };
            p.push((active && terrain_map_mode) as u8);
            p.push(c.mode);
        }
        p.push(rendering_mode);
        p.extend(self.ground_truth.0.to_le_bytes());
        p.extend(self.ground_truth.1.to_le_bytes());
        p
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn word(value: f32) -> f64 {
        (((0b11u64) << 32) | value.to_bits() as u64) as f64
    }

    #[test]
    fn the_status_packet_follows_the_struct() {
        let mut out = GaugeOutput::default();
        let mut gauge = Gauge::new(&mut out);
        assert_eq!(out.writes.len(), 8);
        let inputs = GaugeInputs {
            status: [word(47.26), word(11.35), word(47.), word(11.), 3., word(9000.), word(263.7), word(-1200.), 0.],
            nd: [40., 3., 1., 20., 2., 0., 1., 1.],
            simulator: [47.01, 11.01, 1., 0.5],
        };
        let mut out = GaugeOutput::default();
        gauge.draw(Instant::now(), &inputs, &mut out);
        let p = out.status_packet.expect("a packet");
        assert_eq!(p.len(), 46);
        assert_eq!(p[0], 1);
        assert_eq!(i32::from_le_bytes(p[9..13].try_into().unwrap()), 9000);
        assert_eq!(i16::from_le_bytes(p[13..15].try_into().unwrap()), 263);
        assert_eq!(u16::from_le_bytes(p[27..29].try_into().unwrap()), 40);
        assert_eq!((p[29], p[30], p[31]), (1, 1, 3));
        // The first officer's flag includes the always-active VD.
        assert_eq!((p[34], p[35], p[36]), (0, 1, 2));
        assert_eq!(p[37], 3);
        assert_eq!(f32::from_le_bytes(p[38..42].try_into().unwrap()), 47.01);
        // Powered, no frame yet: black.
        assert_eq!(out.images.len(), 2);
        assert!(out.images[0].1.rgba.chunks(4).all(|c| c == [0, 0, 0, 255]));
    }

    #[test]
    fn frames_after_a_reset_wait_for_the_first_frame_of_the_new_configuration() {
        let mut out = GaugeOutput::default();
        let mut gauge = Gauge::new(&mut out);
        let inputs = GaugeInputs { status: [0.; 9], nd: [40., 3., 1., 20., 3., 1., 1., 1.], simulator: [0., 0., 1., 1.] };
        gauge.draw(Instant::now(), &inputs, &mut out);
        // The configuration change set ignore_next_frame; a frame of an old cycle is dropped.
        let old = ThresholdData { lower_threshold: 100, lower_threshold_mode: 0, upper_threshold: 900, upper_threshold_mode: 0, first_frame: 0, display_range: 40, display_mode: 3, frame_byte_count: 4 };
        let frame = || Arc::new(Frame { width: GAUGE_WIDTH, height: GAUGE_HEIGHT, rgba: [255, 0, 0, 255].repeat(GAUGE_WIDTH * GAUGE_HEIGHT) });
        let mut out = GaugeOutput::default();
        gauge.receive(Side::Left, &old, Some(frame()), &mut out);
        assert!(gauge.displays[0].image.is_none());
        // The first frame of the matching cycle is taken and its thresholds written.
        let first = ThresholdData { first_frame: 1, ..old };
        gauge.receive(Side::Left, &first, Some(frame()), &mut out);
        assert!(gauge.displays[0].image.is_some());
        assert!(out.writes.contains(&("A32NX_EGPWC_ND_L_TERRAIN_MAX_ELEVATION", 900.)));
        let mut out = GaugeOutput::default();
        gauge.draw(Instant::now(), &inputs, &mut out);
        let left = out.images.iter().find(|(s, _)| *s == Side::Left).expect("redrawn");
        assert_eq!(&left.1.rgba[0..4], &[255, 0, 0, 255]);
    }

    /// `NativeImage`'s size and `generation` are exactly what
    /// `mapdata::plugin::terrain_layer` hands the XPHFBW compositor
    /// (untouched -- that function only destructures this struct), so this
    /// exercises the contract documented there: the image is always the
    /// screen's full 768x1024, and `generation` moves only when the
    /// rendered picture actually changes, never on an unchanged redraw.
    #[test]
    fn generation_only_advances_when_the_rendered_picture_changes_and_stays_full_screen_size() {
        let mut out = GaugeOutput::default();
        let mut gauge = Gauge::new(&mut out);
        let inputs = GaugeInputs { status: [0.; 9], nd: [40., 3., 1., 20., 3., 1., 1., 1.], simulator: [0., 0., 1., 1.] };

        let mut out = GaugeOutput::default();
        gauge.draw(Instant::now(), &inputs, &mut out);
        let (_, first) = out.images.into_iter().find(|(s, _)| *s == Side::Left).expect("a first image");
        assert_eq!((first.width, first.height), (GAUGE_WIDTH as u32, GAUGE_HEIGHT as u32));
        assert_eq!(first.rgba.len(), GAUGE_WIDTH * GAUGE_HEIGHT * 4);
        assert_eq!(first.generation, 1);

        // Nothing changed: Display::render's `last_drawn` short-circuit
        // means no image at all comes out, so generation cannot have moved.
        let mut unchanged = GaugeOutput::default();
        gauge.draw(Instant::now(), &inputs, &mut unchanged);
        assert!(unchanged.images.iter().all(|(s, _)| *s != Side::Left));

        // A new frame arrives for the same configuration: the next render()
        // must bump generation past what the compositor already has.
        let frame = Arc::new(Frame { width: GAUGE_WIDTH, height: GAUGE_HEIGHT, rgba: [0, 255, 0, 255].repeat(GAUGE_WIDTH * GAUGE_HEIGHT) });
        let thresholds = ThresholdData {
            lower_threshold: 100,
            lower_threshold_mode: 0,
            upper_threshold: 900,
            upper_threshold_mode: 0,
            first_frame: 1,
            display_range: 40,
            display_mode: 3,
            frame_byte_count: 4,
        };
        let mut received = GaugeOutput::default();
        gauge.receive(Side::Left, &thresholds, Some(frame), &mut received);
        let mut out = GaugeOutput::default();
        gauge.draw(Instant::now(), &inputs, &mut out);
        let (_, second) = out.images.into_iter().find(|(s, _)| *s == Side::Left).expect("a changed image");
        assert!(second.generation > first.generation);
        assert_eq!((second.width, second.height), (GAUGE_WIDTH as u32, GAUGE_HEIGHT as u32));
    }
}
