//! SimBridge's ND terrain renderer (simbridge `apps/server/src/terrain/
//! processing/navigationdisplayrenderer.ts` with its kernels in
//! `processing/gpu/statistics.ts` and `processing/gpu/rendering/
//! navigationdisplay.ts`).
//!
//! The kernels run once per output channel on the GPU; their statistics do
//! not depend on the pixel, so they are worked out once per frame here, and
//! the 8 x 8 patch maxima once per patch. The images are the same.

use super::geo::*;
use super::types::*;
use super::worldmap::MapHandler;

pub const NAVIGATION_DISPLAY_MAP_START_OFFSET_Y: usize = 128;
pub const NAVIGATION_DISPLAY_MAX_PIXEL_WIDTH: usize = 768;
const ARC_MODE_PIXEL_HEIGHT_A32NX: usize = 492;
const ROSE_MODE_PIXEL_HEIGHT_A32NX: usize = 250;
const ARC_MODE_PIXEL_HEIGHT_A380X: usize = 592;
const ROSE_MODE_PIXEL_HEIGHT_A380X: usize = 592;
pub const NAVIGATION_DISPLAY_MAX_PIXEL_HEIGHT: usize = 592;
const CENTER_OFFSET_Y_A32NX: f64 = 0.;
const ARC_MODE_CENTER_OFFSET_Y_A380X: f64 = 100.;
const ROSE_MODE_CENTER_OFFSET_Y_A380X: f64 = 342.;

// rendering timing (generic/constants.ts)
pub const RENDERING_MAP_TRANSITION_DELTA_TIME: u64 = 40;
pub const RENDERING_MAP_TRANSITION_DURATION_ARC_MODE: f64 = 1500.;
pub const RENDERING_MAP_UPDATE_TIMEOUT_ARC_MODE: u64 = 1000;
pub const RENDERING_MAP_TRANSITION_DURATION_SCANLINE_MODE: f64 = 600.;
pub const RENDERING_MAP_UPDATE_TIMEOUT_SCANLINE_MODE: u64 = 500;
const FRAME_VALIDITY_TIME_ARC_MODE: f64 = RENDERING_MAP_TRANSITION_DURATION_ARC_MODE + RENDERING_MAP_UPDATE_TIMEOUT_ARC_MODE as f64;
pub const FRAME_VALIDITY_TIME_SCANLINE_MODE: f64 =
    RENDERING_MAP_TRANSITION_DURATION_SCANLINE_MODE + RENDERING_MAP_UPDATE_TIMEOUT_SCANLINE_MODE as f64;

// histogram parameters
const HISTOGRAM_BIN_RANGE: f64 = 100.;
const HISTOGRAM_MINIMUM_ELEVATION: f64 = -500.;
const HISTOGRAM_MAXIMUM_ELEVATION: f64 = 29040.;
/// `Math.ceil((29040 - -500 + 1) / 100)`, 296: the quotient is not whole.
const HISTOGRAM_BIN_COUNT: usize = ((HISTOGRAM_MAXIMUM_ELEVATION - HISTOGRAM_MINIMUM_ELEVATION + 1.) / HISTOGRAM_BIN_RANGE) as usize + 1;

// rendering parameters
const RENDERING_ARC_MODE_PIXEL_WIDTH: usize = 756;
const RENDERING_ROSE_MODE_PIXEL_WIDTH: usize = 678;
const CUT_OFF_ALTITUDE_MINIMUM: f64 = 200.;
const CUT_OFF_ALTITUDE_MAXIMUM: f64 = 400.;
const LOWER_PERCENTILE: f32 = 0.85;
const UPPER_PERCENTILE: f32 = 0.95;
const FLAT_EARTH_THRESHOLD: f64 = 100.;
const MAX_AIRPORT_DISTANCE: f64 = 4.;
const NORMAL_MODE_LOW_DENSITY_GREEN_OFFSET: f64 = 2000.;
const NORMAL_MODE_HIGH_DENSITY_GREEN_OFFSET: f64 = 1000.;
const NORMAL_MODE_HIGH_DENSITY_YELLOW_OFFSET: f64 = 1000.;
const NORMAL_MODE_HIGH_DENSITY_RED_OFFSET: f64 = 2000.;
const GEAR_DOWN_OFFSET: f64 = 250.;
const NON_GEAR_DOWN_OFFSET: f64 = 500.;
/// `Math.round((90 / 1500) * 40)`
const MAP_TRANSITION_ANGULAR_STEP: f64 = 2.;

/// RGBA (4, 4, 5, 0): a pixel the terrain leaves clear.
pub const CLEAR: [u8; 4] = [4, 4, 5, 0];

pub const ARC_MODE_PATTERN: &[u8] = include_bytes!("patterns/arcmode.bin");
pub const SCANLINE_MODE_PATTERN: &[u8] = include_bytes!("patterns/scanlinemode.bin");

#[derive(Default)]
struct RenderingData {
    start_transition_border: f64,
    current_transition_border: f64,
    threshold_data: NavigationDisplayData,
    final_frame: Option<Vec<u8>>,
    last_frame: Option<Vec<u8>>,
    current_frame: Option<Vec<u8>>,
    frame_validity_duration: f64,
}

pub struct NavigationDisplayRenderer {
    configuration: Option<EfisData>,
    pattern: &'static [u8],
    pattern_ready: bool,
    aircraft_status: Option<AircraftStatus>,
    angle_map: Option<(usize, usize, Vec<f32>)>,
    rendering: RenderingData,
    startup_time: f64,
}

/// The statistics the render kernel computes from the histogram.
struct Statistics {
    reference_altitude: f64,
    min_elevation: f64,
    max_elevation: f64,
    lower_percentile_elevation: f64,
    upper_percentile_elevation: f64,
    flat_earth: f64,
    half_elevation: f64,
}

fn draw_density_pixel(pattern_value: u8, pattern_index: u8, color: [u8; 4]) -> [u8; 4] {
    if pattern_value % pattern_index == 0 {
        color
    } else {
        CLEAR
    }
}

/// `calculateNormalModeGreenThresholds`
fn normal_mode_green_thresholds(reference_altitude: f64, minimum_elevation: f64, flat_earth: f64, lower_percentile: f64, half_elevation: f64) -> (f64, f64) {
    let mut low_density_green = if reference_altitude - NORMAL_MODE_LOW_DENSITY_GREEN_OFFSET <= minimum_elevation {
        minimum_elevation + 200.
    } else {
        reference_altitude - NORMAL_MODE_LOW_DENSITY_GREEN_OFFSET
    };
    let high_density_green = if reference_altitude - NORMAL_MODE_HIGH_DENSITY_GREEN_OFFSET <= minimum_elevation {
        minimum_elevation + 200.
    } else {
        reference_altitude - NORMAL_MODE_HIGH_DENSITY_GREEN_OFFSET
    };
    if flat_earth >= 0. {
        if half_elevation <= lower_percentile && low_density_green > half_elevation {
            low_density_green = half_elevation;
        } else if half_elevation > lower_percentile && low_density_green > lower_percentile {
            low_density_green = lower_percentile;
        }
    }
    (low_density_green, high_density_green)
}

/// `calculateNormalModeWarningThresholds`
fn normal_mode_warning_thresholds(reference_altitude: f64, minimum_elevation: f64, gear_down_altitude_offset: f64) -> (f64, f64, f64) {
    let mut low_density_yellow = reference_altitude - gear_down_altitude_offset;
    let high_density_yellow = reference_altitude + NORMAL_MODE_HIGH_DENSITY_YELLOW_OFFSET;
    let high_density_red = reference_altitude + NORMAL_MODE_HIGH_DENSITY_RED_OFFSET;
    if low_density_yellow <= minimum_elevation {
        low_density_yellow = minimum_elevation + 200.;
    }
    (low_density_yellow, high_density_yellow, high_density_red)
}

/// `calculatePeaksModeThresholds`
fn peaks_mode_thresholds(lower_percentile: f64, upper_percentile: f64, half_elevation: f64, minimum_elevation: f64, maximum_elevation: f64) -> (f64, f64, f64) {
    let lower_density = lower_percentile.min(half_elevation);
    let mut higher_density = upper_percentile.min((maximum_elevation - minimum_elevation) * 0.65 + minimum_elevation);
    let mut solid_density = (maximum_elevation - minimum_elevation) * 0.95 + minimum_elevation;
    if lower_density >= higher_density
        || lower_density >= solid_density
        || higher_density >= solid_density
        || lower_percentile >= upper_percentile
        || lower_percentile >= solid_density
        || upper_percentile >= solid_density
    {
        higher_density = maximum_elevation + 100.;
        solid_density = maximum_elevation + 100.;
    }
    (lower_density, higher_density, solid_density)
}

fn is_elevation(e: f64) -> bool {
    e != INVALID_ELEVATION as f64 && e != UNKNOWN_ELEVATION as f64 && e != WATER_ELEVATION as f64
}

impl NavigationDisplayRenderer {
    pub fn new(startup_time: f64) -> Self {
        Self {
            configuration: None,
            pattern: ARC_MODE_PATTERN,
            pattern_ready: false,
            aircraft_status: None,
            angle_map: None,
            rendering: RenderingData { frame_validity_duration: 1000., ..Default::default() },
            startup_time,
        }
    }

    fn configure_navigation_display(&mut self, config: &EfisData) {
        let last = self.configuration.take();
        let config_changed = last.as_ref().is_some_and(|l| {
            l.efis_mode != config.efis_mode
                || l.nd_range != config.nd_range
                || l.arc_mode != config.arc_mode
                || l.terr_on_nd != config.terr_on_nd
                || l.terr_on_vd != config.terr_on_vd
        });
        let stop_rendering = last.as_ref().is_some_and(|l| (l.terr_on_nd && !config.terr_on_nd) || (l.terr_on_vd && !config.terr_on_vd));
        let start_rendering = config_changed || last.is_none();
        let mut configuration = config.clone();
        if let Some(l) = &last {
            configuration.map_width = l.map_width;
            configuration.map_height = l.map_height;
            configuration.map_offset_x = l.map_offset_x;
            configuration.center_offset_y = l.center_offset_y;
        }
        self.configuration = Some(configuration);
        if stop_rendering || start_rendering {
            self.rendering.threshold_data = NavigationDisplayData::default();
        }
    }

    /// `aircraftStatusUpdate`.
    pub fn aircraft_status_update(&mut self, status: &AircraftStatus, side: Side) {
        let mode_changed = self.aircraft_status.as_ref().is_none_or(|s| s.navigation_display_rendering_mode != status.navigation_display_rendering_mode);
        if mode_changed || !self.pattern_ready {
            if status.navigation_display_rendering_mode & SCANLINE_MODE == SCANLINE_MODE {
                self.pattern = SCANLINE_MODE_PATTERN;
                self.rendering.frame_validity_duration = FRAME_VALIDITY_TIME_SCANLINE_MODE;
            } else {
                self.pattern = ARC_MODE_PATTERN;
                self.rendering.frame_validity_duration = FRAME_VALIDITY_TIME_ARC_MODE;
            }
            self.pattern_ready = true;
        }
        self.aircraft_status = Some(status.clone());
        self.configure_navigation_display(status.efis(side));
    }

    fn scanline(&self) -> bool {
        self.aircraft_status.as_ref().is_some_and(|s| s.navigation_display_rendering_mode & SCANLINE_MODE == SCANLINE_MODE)
    }

    /// The pattern texture, 768 wide; texels past its end read 0.
    fn pattern_value(&self, x: usize, y: usize) -> u8 {
        self.pattern.get(y * NAVIGATION_DISPLAY_MAX_PIXEL_WIDTH + x).copied().unwrap_or(0)
    }

    /// `createElevationHistogram` with both histogram kernels.
    fn elevation_histogram(elevations: &[i16]) -> [u32; HISTOGRAM_BIN_COUNT] {
        let mut histogram = [0u32; HISTOGRAM_BIN_COUNT];
        for &e in elevations {
            let e = e as f64;
            if e != UNKNOWN_ELEVATION as f64 && e != INVALID_ELEVATION as f64 && e != WATER_ELEVATION as f64 {
                let bin = ((e - HISTOGRAM_MINIMUM_ELEVATION) / HISTOGRAM_BIN_RANGE).ceil().min(HISTOGRAM_BIN_COUNT as f64).max(0.);
                if let Some(count) = histogram.get_mut(bin as usize) {
                    *count += 1;
                }
            }
        }
        histogram
    }

    /// `calculateAbsoluteCutOffAltitude`.
    fn absolute_cut_off_altitude(&self, map: &MapHandler) -> f64 {
        let Some(status) = self.aircraft_status.as_ref().filter(|s| s.runway_data_valid) else {
            return HISTOGRAM_MINIMUM_ELEVATION;
        };
        let destination_elevation = map.extract_elevation(status.runway_latitude, status.runway_longitude);
        if destination_elevation == INVALID_ELEVATION {
            return HISTOGRAM_MINIMUM_ELEVATION;
        }
        let destination_elevation = destination_elevation as f64;
        let mut cut_off_altitude = CUT_OFF_ALTITUDE_MAXIMUM;
        let distance = distance_wgs84(status.latitude, status.longitude, status.runway_latitude, status.runway_longitude);
        if distance <= MAX_AIRPORT_DISTANCE {
            let distance_feet = distance * FEET_PER_NAUTICAL_MILE;
            // calculate the glide until touchdown
            let opposite = status.altitude - destination_elevation;
            let mut glide_radian = 0.;
            if opposite > 0. && distance > 0. {
                glide_radian = (opposite / distance_feet).atan();
            }
            // check if the glide is greater or equal 3°
            if glide_radian < 0.0523599 {
                if distance <= 1. || glide_radian == 0. {
                    cut_off_altitude = CUT_OFF_ALTITUDE_MINIMUM;
                } else {
                    let slope = (CUT_OFF_ALTITUDE_MINIMUM - CUT_OFF_ALTITUDE_MAXIMUM) / THREE_NAUTICAL_MILES_IN_FEET;
                    cut_off_altitude = js_round(slope * (distance_feet - FEET_PER_NAUTICAL_MILE) + CUT_OFF_ALTITUDE_MAXIMUM);
                    cut_off_altitude = cut_off_altitude.max(CUT_OFF_ALTITUDE_MINIMUM).min(CUT_OFF_ALTITUDE_MAXIMUM);
                }
            }
        }
        cut_off_altitude
    }

    /// The histogram statistics at the top of `renderNavigationDisplay`.
    fn statistics(histogram: &[u32; HISTOGRAM_BIN_COUNT], altitude: f64, vertical_speed: f64, cut_off_altitude: f64) -> Statistics {
        let cut_off_altitude_bin = ((cut_off_altitude - HISTOGRAM_MINIMUM_ELEVATION) / HISTOGRAM_BIN_RANGE).floor().max(0.) as usize;
        // predict 30 seconds -> half of the vertical speed (feet per minute)
        let reference_altitude = altitude + if vertical_speed <= -1000. { vertical_speed * 0.5 } else { 0. };
        let bins = cut_off_altitude_bin.min(HISTOGRAM_BIN_COUNT)..HISTOGRAM_BIN_COUNT;
        // The GPU sums in 32-bit floats.
        let total_frequency: f32 = histogram[bins.clone()].iter().map(|&c| c as f32).sum();
        let (mut min_bin, mut max_bin, mut lower_bin, mut upper_bin) = (-1i64, -1i64, -1i64, -1i64);
        let mut current_percentile = 0f32;
        for bin in bins {
            if total_frequency > 0. {
                current_percentile += histogram[bin] as f32 / total_frequency;
                if lower_bin == -1 && current_percentile >= LOWER_PERCENTILE {
                    lower_bin = bin as i64;
                }
                if upper_bin == -1 && current_percentile >= UPPER_PERCENTILE {
                    upper_bin = bin as i64;
                }
            }
            if histogram[bin] > 0 {
                if min_bin < 0 {
                    min_bin = bin as i64;
                }
                max_bin = bin as i64;
            }
        }
        if lower_bin > HISTOGRAM_BIN_COUNT as i64 {
            lower_bin = HISTOGRAM_BIN_COUNT as i64 - 1;
        }
        if upper_bin < 0 {
            upper_bin = HISTOGRAM_BIN_COUNT as i64 - 1;
        }
        let lower_percentile_elevation = lower_bin as f64 * HISTOGRAM_BIN_RANGE + HISTOGRAM_MINIMUM_ELEVATION;
        let upper_percentile_elevation = upper_bin as f64 * HISTOGRAM_BIN_RANGE + HISTOGRAM_MINIMUM_ELEVATION;
        let min_elevation = if min_bin >= 0 { min_bin as f64 * HISTOGRAM_BIN_RANGE + HISTOGRAM_MINIMUM_ELEVATION } else { -1. };
        let max_elevation = if max_bin >= 0 { (max_bin + 1) as f64 * HISTOGRAM_BIN_RANGE + HISTOGRAM_MINIMUM_ELEVATION } else { 0. };
        Statistics {
            reference_altitude,
            min_elevation,
            max_elevation,
            lower_percentile_elevation,
            upper_percentile_elevation,
            flat_earth: FLAT_EARTH_THRESHOLD - (max_elevation - min_elevation),
            half_elevation: max_elevation * 0.5,
        }
    }

    /// `createNavigationDisplayMap` with the `renderNavigationDisplay`
    /// kernel: the frame (RGBA, `map_width` x `map_height`) and its metadata
    /// row, as floats.
    fn render_map(&self, config: &EfisData, elevations: &[i16], histogram: &[u32; HISTOGRAM_BIN_COUNT], cut_off_altitude: f64) -> (Vec<u8>, [f64; 8]) {
        let status = self.aircraft_status.as_ref().expect("rendered with a status");
        let gear_down_altitude_offset = if status.gear_is_down { GEAR_DOWN_OFFSET } else { NON_GEAR_DOWN_OFFSET };
        let s = Self::statistics(histogram, status.altitude, status.vertical_speed, cut_off_altitude);
        let (width, height) = (config.map_width, config.map_height);
        let normal = s.max_elevation >= s.reference_altitude - gear_down_altitude_offset;

        // find highest elevation in 8x8 patch to simulate the lower resolution of the real system
        let (patches_x, patches_y) = (width.div_ceil(8), height.div_ceil(8));
        let mut patch_max = vec![-1000f64; patches_x * patches_y];
        for y in 0..height {
            for x in 0..width {
                let e = elevations[y * width + x];
                let p = &mut patch_max[(y / 8) * patches_x + x / 8];
                if e as f64 > *p && e != INVALID_ELEVATION {
                    *p = e as f64;
                }
            }
        }

        let warning = normal_mode_warning_thresholds(s.reference_altitude, s.min_elevation, gear_down_altitude_offset);
        let green = normal_mode_green_thresholds(s.reference_altitude, s.min_elevation, s.flat_earth, s.lower_percentile_elevation, s.half_elevation);
        let peaks = peaks_mode_thresholds(s.lower_percentile_elevation, s.upper_percentile_elevation, s.half_elevation, s.min_elevation, s.max_elevation);

        let mut frame = vec![0u8; width * height * 4];
        for y in 0..height {
            for x in 0..width {
                let pattern_value = self.pattern_value(x, y);
                let color = if pattern_value == 0 {
                    CLEAR
                } else {
                    let elevation = patch_max[(y / 8) * patches_x + x / 8];
                    if normal {
                        Self::normal_mode_pixel(elevation, pattern_value, warning, green, cut_off_altitude)
                    } else {
                        Self::peaks_mode_pixel(elevation, pattern_value, peaks)
                    }
                };
                frame[(y * width + x) * 4..(y * width + x + 1) * 4].copy_from_slice(&color);
            }
        }
        let metadata = if normal {
            [0., s.min_elevation, s.max_elevation, warning.2, warning.1, warning.0, green.1, green.0]
        } else {
            [1., s.min_elevation, s.max_elevation, peaks.2, peaks.1, peaks.0, 0., 0.]
        };
        (frame, metadata)
    }

    /// `renderNormalMode` below the metadata row.
    fn normal_mode_pixel(elevation: f64, pattern_value: u8, warning: (f64, f64, f64), green: (f64, f64), cut_off: f64) -> [u8; 4] {
        if is_elevation(elevation) && elevation >= cut_off {
            if elevation >= warning.2 {
                return draw_density_pixel(pattern_value, 5, [255, 0, 0, 255]);
            }
            if elevation >= warning.1 {
                return draw_density_pixel(pattern_value, 5, [255, 255, 50, 255]);
            }
            if elevation >= green.1 && elevation < warning.0 {
                return draw_density_pixel(pattern_value, 5, [0, 255, 0, 255]);
            }
            if elevation >= warning.0 && elevation < warning.1 {
                return draw_density_pixel(pattern_value, 3, [255, 255, 50, 255]);
            }
            if elevation >= green.0 && elevation < green.1 {
                return draw_density_pixel(pattern_value, 3, [0, 255, 0, 255]);
            }
        } else if elevation == WATER_ELEVATION as f64 {
            return draw_density_pixel(pattern_value, 7, [0, 255, 255, 255]);
        } else if elevation == UNKNOWN_ELEVATION as f64 {
            return draw_density_pixel(pattern_value, 5, [255, 148, 255, 255]);
        }
        [0, 0, 0, 255]
    }

    /// `renderPeaksMode` below the metadata row.
    fn peaks_mode_pixel(elevation: f64, pattern_value: u8, thresholds: (f64, f64, f64)) -> [u8; 4] {
        if is_elevation(elevation) {
            if thresholds.2 <= elevation {
                // solid threshold
                return [0, 255, 0, 255];
            }
            if thresholds.1 <= elevation {
                return draw_density_pixel(pattern_value, 5, [0, 255, 0, 255]);
            }
            if thresholds.0 <= elevation {
                return draw_density_pixel(pattern_value, 3, [0, 255, 0, 255]);
            }
        } else if elevation == WATER_ELEVATION as f64 {
            return draw_density_pixel(pattern_value, 7, [0, 255, 255, 255]);
        } else if elevation == UNKNOWN_ELEVATION as f64 {
            return draw_density_pixel(pattern_value, 5, [255, 148, 255, 255]);
        }
        [0, 0, 0, 255]
    }

    /// `analyzeMetadata`.
    fn analyze_metadata(metadata: &[f64; 8], cut_off_altitude: f64) -> NavigationDisplayData {
        let mut r = NavigationDisplayData {
            minimum_elevation: f64::INFINITY,
            minimum_elevation_mode: PEAKS_MODE,
            maximum_elevation: f64::INFINITY,
            maximum_elevation_mode: PEAKS_MODE,
            first_frame: false,
            display_range: 10.,
            display_mode: 0.,
            frame_byte_count: 0,
        };
        if metadata[0] == 0. {
            // normal mode
            let (max_elevation, high_density_red, low_density_yellow, high_density_green, low_density_green) =
                (metadata[2], metadata[3], metadata[5], metadata[6], metadata[7]);
            r.minimum_elevation = if cut_off_altitude > low_density_green { cut_off_altitude } else { low_density_green };
            r.minimum_elevation_mode = if low_density_yellow <= high_density_green { WARNING } else { PEAKS_MODE };
            r.maximum_elevation = max_elevation;
            r.maximum_elevation_mode = if max_elevation >= high_density_red { CAUTION } else { WARNING };
        } else {
            // peaks mode
            let (min_elevation, max_elevation, low_density_green) = (metadata[1], metadata[2], metadata[5]);
            if max_elevation < 0. {
                r.minimum_elevation = -1.;
                r.maximum_elevation = 0.;
            } else {
                r.minimum_elevation = if low_density_green > min_elevation { low_density_green } else { min_elevation };
                r.maximum_elevation = max_elevation;
            }
        }
        r
    }

    fn angle_map(&mut self, width: usize, height: usize) -> &[f32] {
        if !matches!(&self.angle_map, Some((w, h, _)) if *w == width && *h == height) {
            let mut angles = Vec::with_capacity(width * height);
            for y in 0..height {
                for x in 0..width {
                    let dx = x as f64 - width as f64 / 2.;
                    let dy = (height - y) as f64;
                    let distance = (dx * dx + dy * dy).sqrt();
                    angles.push(if distance == 0. { 0. } else { ((dy / distance).acos() * (180. / std::f64::consts::PI)) as f32 });
                }
            }
            self.angle_map = Some((width, height, angles));
        }
        &self.angle_map.as_ref().expect("just made").2
    }

    fn arc_mode_transition_frame(&mut self, start_angle: f64, end_angle: f64) -> Option<Vec<u8>> {
        let config = self.configuration.as_ref()?;
        let (width, height) = (config.map_width, config.map_height);
        let new_frame = self.rendering.final_frame.take()?;
        let old_frame = self.rendering.last_frame.take();
        let mut result = CLEAR.repeat(width * height);
        let angles = self.angle_map(width, height);
        for (i, &angle) in angles.iter().enumerate() {
            let angle = angle as f64;
            let source = if start_angle <= angle && angle <= end_angle { Some(&new_frame) } else { old_frame.as_ref() };
            if let Some(source) = source {
                result[i * 4..i * 4 + 4].copy_from_slice(&source[i * 4..i * 4 + 4]);
            }
        }
        self.rendering.final_frame = Some(new_frame);
        self.rendering.last_frame = old_frame;
        Some(result)
    }

    fn arc_mode_transition(&mut self) -> bool {
        let Some(config) = self.configuration.as_ref() else { return true };
        if self.rendering.final_frame.is_none() {
            return true;
        }
        self.rendering.threshold_data.display_range = config.nd_range;
        self.rendering.threshold_data.display_mode = config.efis_mode;
        self.rendering.current_transition_border += MAP_TRANSITION_ANGULAR_STEP;
        let (start, current) = (self.rendering.start_transition_border, self.rendering.current_transition_border);
        if current < 90. {
            self.rendering.current_frame = self.arc_mode_transition_frame(start, current);
            return false;
        }
        // perform the last frame
        if current - MAP_TRANSITION_ANGULAR_STEP < 90. {
            self.rendering.current_frame = self.arc_mode_transition_frame(start, 90.);
        }
        // do not overwrite the last frame of the initialization
        self.rendering.last_frame = self.rendering.current_frame.clone();
        true
    }

    fn scanline_mode_transition_frame(&self) -> Option<Vec<u8>> {
        let config = self.configuration.as_ref()?;
        let new_frame = self.rendering.final_frame.as_ref()?;
        let old_frame = self.rendering.last_frame.as_ref();
        let (width, height) = (config.map_width, config.map_height);
        let mut result = CLEAR.repeat(width * height);
        let (start, current) = (self.rendering.start_transition_border, self.rendering.current_transition_border);
        for y in 0..height {
            let yf = y as f64;
            let source = if yf <= start && yf >= current { Some(new_frame) } else { old_frame };
            if let Some(source) = source {
                let row = y * width * 4..(y + 1) * width * 4;
                result[row.clone()].copy_from_slice(&source[row]);
            }
        }
        Some(result)
    }

    fn scanline_mode_transition(&mut self) -> bool {
        let Some(config) = self.configuration.as_ref() else { return true };
        if self.rendering.final_frame.is_none() {
            return true;
        }
        let vertical_step = js_round((config.map_height as f64 / RENDERING_MAP_TRANSITION_DURATION_SCANLINE_MODE) * RENDERING_MAP_TRANSITION_DELTA_TIME as f64);
        self.rendering.threshold_data.display_range = config.nd_range;
        self.rendering.threshold_data.display_mode = config.efis_mode;
        self.rendering.current_transition_border -= vertical_step;
        if self.rendering.current_transition_border > 0. {
            self.rendering.current_frame = self.scanline_mode_transition_frame();
            return false;
        }
        // perform the last frame
        if self.rendering.current_transition_border + vertical_step >= 0. {
            self.rendering.current_frame = self.scanline_mode_transition_frame();
        }
        // do not overwrite the last frame of the initialization
        self.rendering.last_frame = self.rendering.current_frame.clone();
        true
    }

    /// `reset`.
    pub fn reset(&mut self) {
        self.rendering = RenderingData { frame_validity_duration: 1000., ..Default::default() };
    }

    /// `startNewMapCycle`.
    pub fn start_new_map_cycle(&mut self, map: &MapHandler, current_time: f64) {
        let Some(status) = self.aircraft_status.clone() else { return };
        let Some(config) = self.configuration.as_mut() else { return };
        config.map_width = if config.arc_mode { RENDERING_ARC_MODE_PIXEL_WIDTH } else { RENDERING_ROSE_MODE_PIXEL_WIDTH };
        if status.navigation_display_rendering_mode & VERTICAL_DISPLAY_REQUIRED == VERTICAL_DISPLAY_REQUIRED {
            // Only A380X requires vertical display
            config.map_height = if config.arc_mode { ARC_MODE_PIXEL_HEIGHT_A380X } else { ROSE_MODE_PIXEL_HEIGHT_A380X };
            config.center_offset_y = if config.arc_mode { ARC_MODE_CENTER_OFFSET_Y_A380X } else { ROSE_MODE_CENTER_OFFSET_Y_A380X };
        } else {
            config.map_height = if config.arc_mode { ARC_MODE_PIXEL_HEIGHT_A32NX } else { ROSE_MODE_PIXEL_HEIGHT_A32NX };
            config.center_offset_y = CENTER_OFFSET_Y_A32NX;
        }
        config.map_offset_x = ((NAVIGATION_DISPLAY_MAX_PIXEL_WIDTH - config.map_width) as f64 * 0.5).ceil();
        if config.nd_range == 0. {
            self.reset();
            return;
        }
        let config = config.clone();
        // A frame rendered while the map handler has nothing: SimBridge's
        // kernels return null and the cycle keeps its previous state.
        let Some(elevations) = map.create_local_elevation_map(&config) else { return };
        let histogram = Self::elevation_histogram(&elevations);
        let cut_off_altitude = self.absolute_cut_off_altitude(map);
        let (frame, metadata) = self.render_map(&config, &elevations, &histogram, cut_off_altitude);

        self.rendering.final_frame = Some(frame);
        self.rendering.threshold_data = Self::analyze_metadata(&metadata, cut_off_altitude);
        if !config.terr_on_nd {
            // metadata is used in the TERRONND WASM module to detect frame changes, so we still have to send it even
            // though ND TERR would be disabled on the A380X. Send negative values for the thresholds in order to hide them.
            self.rendering.threshold_data.minimum_elevation = -1.;
            self.rendering.threshold_data.maximum_elevation = -1.;
        }
        self.rendering.threshold_data.display_range = config.nd_range;
        self.rendering.threshold_data.display_mode = config.efis_mode;

        let scanline = self.scanline();
        if self.rendering.last_frame.is_none() {
            let time_since_start = current_time - self.startup_time;
            let frame_update_count = time_since_start / self.rendering.frame_validity_duration;
            let ratio_since_last_frame = frame_update_count - frame_update_count.floor();
            self.rendering.start_transition_border = if scanline {
                config.map_height as f64 - (config.map_height as f64 * ratio_since_last_frame).floor()
            } else {
                (90. * ratio_since_last_frame).floor()
            };
        } else if scanline {
            self.rendering.start_transition_border = config.map_height as f64;
        } else {
            self.rendering.start_transition_border = 0.;
        }
        self.rendering.current_transition_border = self.rendering.start_transition_border;
    }

    /// `render`: one transition step; true when the cycle's last frame is done.
    pub fn render(&mut self) -> bool {
        if self.scanline() { self.scanline_mode_transition() } else { self.arc_mode_transition() }
    }

    pub fn display_configuration(&self) -> Option<&EfisData> {
        self.configuration.as_ref()
    }

    pub fn display_data(&self) -> NavigationDisplayData {
        self.rendering.threshold_data
    }

    pub fn current_frame(&self) -> Option<&[u8]> {
        self.rendering.current_frame.as_deref()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_histogram_statistics_follow_the_kernel() {
        let mut histogram = [0u32; HISTOGRAM_BIN_COUNT];
        // 100 pixels at 1000 ft (bin 15), 10 at 5000 ft (bin 55).
        histogram[15] = 100;
        histogram[55] = 10;
        let s = NavigationDisplayRenderer::statistics(&histogram, 10000., 0., -500.);
        assert_eq!(s.min_elevation, 1000.);
        assert_eq!(s.max_elevation, 5100.);
        assert_eq!(s.lower_percentile_elevation, 1000.);
        assert_eq!(s.upper_percentile_elevation, 5000.);
        assert_eq!(s.half_elevation, 2550.);
        // Peaks mode below the aircraft: min is the higher of min and the low density threshold.
        let peaks = peaks_mode_thresholds(1000., 5000., 2550., 1000., 5100.);
        let data = NavigationDisplayRenderer::analyze_metadata(&[1., 1000., 5100., peaks.2, peaks.1, peaks.0, 0., 0.], -500.);
        assert_eq!((data.minimum_elevation, data.maximum_elevation), (1000., 5100.));
        assert_eq!(draw_density_pixel(35, 7, [1, 2, 3, 4]), [1, 2, 3, 4]);
        assert_eq!(draw_density_pixel(35, 3, [1, 2, 3, 4]), CLEAR);
    }
}
