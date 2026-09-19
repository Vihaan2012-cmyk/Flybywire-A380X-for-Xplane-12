//! SimBridge's vertical display terrain renderer (simbridge `apps/server/
//! src/terrain/processing/verticaldisplayrenderer.ts` and its kernel in
//! `processing/gpu/rendering/verticaldisplay.ts`).

use super::geo::*;
use super::navigation_display::{FRAME_VALIDITY_TIME_SCANLINE_MODE, RENDERING_MAP_TRANSITION_DELTA_TIME, RENDERING_MAP_TRANSITION_DURATION_SCANLINE_MODE};
use super::types::*;
use super::worldmap::{ElevationProfile, MapHandler};

pub const PROFILE_WIDTH: usize = 540;
pub const PROFILE_HEIGHT: usize = 200;
pub const VERTICAL_DISPLAY_MAP_START_OFFSET_Y: usize = 800;
pub const VERTICAL_DISPLAY_MAP_START_OFFSET_X: usize = 150;
/// RGBA (4, 4, 4, 0), the transition frame's fill.
const CLEAR: [u8; 4] = [4, 4, 4, 0];

/// `VerticalDisplay`.
#[derive(Clone, Debug, PartialEq)]
pub struct VerticalDisplay {
    pub range: f64,
    pub minimum_altitude: f64,
    pub maximum_altitude: f64,
    pub map_width: usize,
    pub map_height: usize,
}

impl Default for VerticalDisplay {
    fn default() -> Self {
        Self { range: 0., minimum_altitude: -500., maximum_altitude: 24000., map_width: 0, map_height: 0 }
    }
}

fn empty_profile() -> ElevationProfile {
    ElevationProfile {
        path_width: 1.,
        waypoints_latitudes: Vec::new(),
        waypoints_longitudes: Vec::new(),
        range: 0.,
        track_changes_significantly_at_distance: -1.,
        fms_path_used: false,
    }
}

#[derive(Default)]
struct RenderingData {
    start_transition_border: f64,
    current_transition_border: f64,
    final_frame: Option<Vec<u8>>,
    last_frame: Option<Vec<u8>>,
    current_frame: Option<Vec<u8>>,
}

pub struct VerticalDisplayRenderer {
    elevation_config: ElevationProfile,
    display_config: VerticalDisplay,
    rendering: RenderingData,
    startup_time: f64,
}

impl VerticalDisplayRenderer {
    pub fn new(startup_time: f64) -> Self {
        Self { elevation_config: empty_profile(), display_config: VerticalDisplay::default(), rendering: RenderingData::default(), startup_time }
    }

    /// `aircraftStatusUpdate`.
    pub fn aircraft_status_update(&mut self, status: &AircraftStatus, side: Side) {
        self.elevation_config.fms_path_used = !status.manual_azim_enabled && !self.elevation_config.waypoints_latitudes.is_empty();
        let efis = status.efis(side);
        let vd_range = if efis.arc_mode { 10f64.max(efis.nd_range.min(160.)) } else { 5f64.max((efis.nd_range / 2.).min(160.)) };
        self.elevation_config.range = vd_range;
        self.display_config.range = efis.nd_range;
        self.display_config.minimum_altitude = efis.vd_range_lower;
        self.display_config.maximum_altitude = efis.vd_range_upper;
    }

    /// `pathDataUpdate`.
    pub fn path_data_update(&mut self, data: &VerticalPathData) {
        self.elevation_config.path_width = data.path_width;
        self.elevation_config.waypoints_latitudes = data.waypoints.iter().map(|w| w.0).collect();
        self.elevation_config.waypoints_longitudes = data.waypoints.iter().map(|w| w.1).collect();
        self.elevation_config.track_changes_significantly_at_distance = data.track_changes_significantly_at_distance;
    }

    pub fn num_path_elements(&self) -> usize {
        self.elevation_config.waypoints_latitudes.len()
    }

    /// `reset`.
    pub fn reset(&mut self, reset_path: bool) {
        self.rendering = RenderingData::default();
        if reset_path {
            self.elevation_config = empty_profile();
        }
        self.display_config = VerticalDisplay::default();
    }

    /// The `renderVerticalDisplay` kernel for one pixel.
    fn pixel(elevation: i32, x: usize, y: usize, minimum_altitude: f64, maximum_altitude: f64, grey_from_x: f64) -> [u8; 4] {
        if elevation == INVALID_ELEVATION as i32 || elevation == UNKNOWN_ELEVATION as i32 {
            return [255, 148, 255, 255];
        }
        let step_y = (maximum_altitude - minimum_altitude) / PROFILE_HEIGHT as f64;
        let altitude = (PROFILE_HEIGHT as f64 - y as f64) * step_y + minimum_altitude;
        // altitude is above the elevation -> draw the background
        if altitude > elevation as f64 {
            return if grey_from_x >= 0. && x as f64 >= grey_from_x { [78, 78, 97, 255] } else { [0, 0, 0, 0] };
        }
        // elevation is water -> check if we draw the water until 0
        if elevation == WATER_ELEVATION as i32 {
            return if altitude <= 0. { [0, 255, 255, 255] } else { [0, 0, 0, 0] };
        }
        // draw the obstacle
        [110, 51, 14, 255]
    }

    /// `startNewMapCycle`.
    pub fn start_new_map_cycle(&mut self, map: &MapHandler, current_time: f64) {
        if self.elevation_config.range == 0. {
            self.reset(true);
            return;
        }
        self.display_config.map_width = PROFILE_WIDTH;
        self.display_config.map_height = PROFILE_HEIGHT;
        let Some(profile) = map.create_elevation_profile(&self.elevation_config, PROFILE_WIDTH) else { return };
        let grey_area_starts_at_x = if self.elevation_config.track_changes_significantly_at_distance >= 0. && self.elevation_config.fms_path_used {
            vertical_display_distance_to_pixel_x(self.elevation_config.track_changes_significantly_at_distance, self.elevation_config.range)
        } else {
            -1.
        };
        let mut frame = Vec::with_capacity(PROFILE_WIDTH * PROFILE_HEIGHT * 4);
        for y in 0..PROFILE_HEIGHT {
            for (x, &elevation) in profile.iter().enumerate() {
                frame.extend(Self::pixel(elevation, x, y, self.display_config.minimum_altitude, self.display_config.maximum_altitude, grey_area_starts_at_x));
            }
        }
        self.rendering.final_frame = Some(frame);
        if self.rendering.last_frame.is_none() {
            let time_since_start = current_time - self.startup_time;
            let frame_update_count = time_since_start / FRAME_VALIDITY_TIME_SCANLINE_MODE;
            let ratio_since_last_frame = frame_update_count - frame_update_count.floor();
            self.rendering.start_transition_border = (PROFILE_WIDTH as f64 * ratio_since_last_frame).floor();
        } else {
            self.rendering.start_transition_border = 0.;
        }
        self.rendering.current_transition_border = self.rendering.start_transition_border;
    }

    fn transition_frame(&self) -> Option<Vec<u8>> {
        let new_frame = self.rendering.final_frame.as_ref()?;
        let old_frame = self.rendering.last_frame.as_ref();
        let mut result = CLEAR.repeat(PROFILE_WIDTH * PROFILE_HEIGHT);
        let (start, current) = (self.rendering.start_transition_border, self.rendering.current_transition_border);
        for y in 0..PROFILE_HEIGHT {
            for x in 0..PROFILE_WIDTH {
                let xf = x as f64;
                let source = if xf >= start && xf <= current { Some(new_frame) } else { old_frame };
                if let Some(source) = source {
                    let i = (y * PROFILE_WIDTH + x) * 4;
                    result[i..i + 4].copy_from_slice(&source[i..i + 4]);
                }
            }
        }
        Some(result)
    }

    /// `render`.
    pub fn render(&mut self) -> bool {
        if self.rendering.final_frame.is_none() {
            return true;
        }
        let horizontal_step = js_round((PROFILE_WIDTH as f64 / RENDERING_MAP_TRANSITION_DURATION_SCANLINE_MODE) * RENDERING_MAP_TRANSITION_DELTA_TIME as f64);
        self.rendering.current_transition_border += horizontal_step;
        if self.rendering.current_transition_border < PROFILE_WIDTH as f64 {
            self.rendering.current_frame = self.transition_frame();
            return false;
        }
        // perform the last frame
        if self.rendering.current_transition_border + horizontal_step > PROFILE_WIDTH as f64 {
            self.rendering.current_frame = self.transition_frame();
        }
        // do not overwrite the last frame of the initialization
        self.rendering.last_frame = self.rendering.current_frame.clone();
        true
    }

    pub fn display_configuration(&self) -> &VerticalDisplay {
        &self.display_config
    }

    pub fn current_frame(&self) -> Option<&[u8]> {
        self.rendering.current_frame.as_deref()
    }
}
