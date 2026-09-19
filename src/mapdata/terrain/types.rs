//! SimBridge's terrain types (simbridge `apps/server/src/terrain/types/
//! msfstypes.ts`, `dto/`), and reading the JSON FlyByWire's EfisTawsBridge
//! posts to `/api/v1/terrain/*` (fbw-common `simbridge/components/
//! TawsData.ts`).

use serde_json::Value;

/// `TerrainRenderingMode`: bit 0 scanline (else arc), bit 1 vertical display.
pub const ARC_MODE: u32 = 0;
pub const SCANLINE_MODE: u32 = 1;
pub const VERTICAL_DISPLAY_REQUIRED: u32 = 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Side {
    Left,
    Right,
}

impl Side {
    pub const BOTH: [Side; 2] = [Side::Left, Side::Right];

    pub fn index(self) -> usize {
        self as usize
    }

    pub fn letter(self) -> &'static str {
        match self {
            Side::Left => "L",
            Side::Right => "R",
        }
    }
}

/// `EfisData`. Numbers stay JavaScript numbers.
#[derive(Clone, Debug, PartialEq)]
pub struct EfisData {
    pub nd_range: f64,
    pub arc_mode: bool,
    pub terr_on_nd: bool,
    pub terr_on_vd: bool,
    pub efis_mode: f64,
    pub vd_range_lower: f64,
    pub vd_range_upper: f64,
    pub map_offset_x: f64,
    pub map_width: usize,
    pub map_height: usize,
    pub center_offset_y: f64,
}

impl EfisData {
    /// The fields the renderer fills in itself left at their defaults.
    pub fn new(nd_range: f64, arc_mode: bool, terr_on_nd: bool, terr_on_vd: bool, efis_mode: f64, vd_range_lower: f64, vd_range_upper: f64) -> Self {
        Self {
            nd_range,
            arc_mode,
            terr_on_nd,
            terr_on_vd,
            efis_mode,
            vd_range_lower,
            vd_range_upper,
            map_offset_x: 0.,
            map_width: 0,
            map_height: 0,
            center_offset_y: 0.,
        }
    }
}

/// `AircraftStatus`.
#[derive(Clone, Debug, PartialEq)]
pub struct AircraftStatus {
    pub adiru_data_valid: bool,
    pub taws_inop: bool,
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub heading: f64,
    pub vertical_speed: f64,
    pub gear_is_down: bool,
    pub runway_data_valid: bool,
    pub runway_latitude: f64,
    pub runway_longitude: f64,
    pub efis_data_capt: EfisData,
    pub efis_data_fo: EfisData,
    pub navigation_display_rendering_mode: u32,
    pub manual_azim_enabled: bool,
    pub manual_azim_degrees: f64,
    pub ground_truth_latitude: f64,
    pub ground_truth_longitude: f64,
}

impl AircraftStatus {
    pub fn efis(&self, side: Side) -> &EfisData {
        match side {
            Side::Left => &self.efis_data_capt,
            Side::Right => &self.efis_data_fo,
        }
    }

    /// `TawsAircraftStatusDataDto` as the EfisTawsBridge posts it. Nest's
    /// validation (`class-validator` decorators in `tawsaircraftstatusdata.
    /// dto.ts`) rejects a body with a missing or mistyped field.
    pub fn from_json(body: &str) -> Result<Self, String> {
        let v: Value = serde_json::from_str(body).map_err(|e| format!("aircraftStatusData: {e}"))?;
        let efis = |name: &str| -> Result<EfisData, String> {
            let e = v.get(name).ok_or_else(|| format!("aircraftStatusData: {name} is missing"))?;
            Ok(EfisData::new(
                number(e, "ndRange")?,
                boolean(e, "arcMode")?,
                boolean(e, "terrOnNd")?,
                boolean(e, "terrOnVd")?,
                number(e, "efisMode")?,
                number(e, "vdRangeLower")?,
                number(e, "vdRangeUpper")?,
            ))
        };
        let azim = number(&v, "manualAzimDegrees")?;
        if !(0. ..=360.).contains(&azim) {
            return Err("aircraftStatusData: manualAzimDegrees is outside 0..360".into());
        }
        Ok(Self {
            adiru_data_valid: boolean(&v, "adiruDataValid")?,
            taws_inop: boolean(&v, "tawsInop")?,
            latitude: number(&v, "latitude")?,
            longitude: number(&v, "longitude")?,
            altitude: number(&v, "altitude")?,
            heading: number(&v, "heading")?,
            vertical_speed: number(&v, "verticalSpeed")?,
            gear_is_down: boolean(&v, "gearIsDown")?,
            runway_data_valid: boolean(&v, "runwayDataValid")?,
            runway_latitude: number(&v, "runwayLatitude")?,
            runway_longitude: number(&v, "runwayLongitude")?,
            efis_data_capt: efis("efisDataCapt")?,
            efis_data_fo: efis("efisDataFO")?,
            navigation_display_rendering_mode: number(&v, "navigationDisplayRenderingMode")? as u32,
            manual_azim_enabled: boolean(&v, "manualAzimEnabled")?,
            manual_azim_degrees: azim,
            ground_truth_latitude: number(&v, "groundTruthLatitude")?,
            ground_truth_longitude: number(&v, "groundTruthLongitude")?,
        })
    }
}

/// `VerticalPathData`.
#[derive(Clone, Debug, PartialEq)]
pub struct VerticalPathData {
    pub path_width: f64,
    pub track_changes_significantly_at_distance: f64,
    /// (latitude, longitude)
    pub waypoints: Vec<(f64, f64)>,
}

impl VerticalPathData {
    /// `ElevationSamplePathDto` (`elevationsamplepath.dto.ts`, `waypoint.dto.ts`).
    pub fn from_json(body: &str) -> Result<Self, String> {
        let v: Value = serde_json::from_str(body).map_err(|e| format!("verticalDisplayPath: {e}"))?;
        let list = v.get("waypoints").and_then(Value::as_array).ok_or("verticalDisplayPath: waypoints is not an array")?;
        let mut waypoints = Vec::with_capacity(list.len());
        for w in list {
            let lat = number(w, "latitude")?;
            let lon = number(w, "longitude")?;
            if !(-90. ..=90.).contains(&lat) || !(-180. ..=180.).contains(&lon) {
                return Err("verticalDisplayPath: a waypoint is outside WGS84's range".into());
            }
            waypoints.push((lat, lon));
        }
        Ok(Self {
            path_width: number(&v, "pathWidth")?,
            track_changes_significantly_at_distance: number(&v, "trackChangesSignificantlyAtDistance")?,
            waypoints,
        })
    }
}

fn number(v: &Value, name: &str) -> Result<f64, String> {
    v.get(name).and_then(Value::as_f64).filter(|n| n.is_finite()).ok_or_else(|| format!("{name} is not a number"))
}

fn boolean(v: &Value, name: &str) -> Result<bool, String> {
    v.get(name).and_then(Value::as_bool).ok_or_else(|| format!("{name} is not a boolean"))
}

/// `TerrainLevelMode`.
pub const PEAKS_MODE: u8 = 0;
pub const WARNING: u8 = 1;
pub const CAUTION: u8 = 2;

/// `NavigationDisplayData`: the thresholds packet sent before each frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NavigationDisplayData {
    pub minimum_elevation: f64,
    pub minimum_elevation_mode: u8,
    pub maximum_elevation: f64,
    pub maximum_elevation_mode: u8,
    pub first_frame: bool,
    pub display_range: f64,
    pub display_mode: f64,
    pub frame_byte_count: u32,
}

impl Default for NavigationDisplayData {
    /// The reset packet (`NavigationDisplayRenderer.reset`).
    fn default() -> Self {
        Self {
            minimum_elevation: -1.,
            minimum_elevation_mode: PEAKS_MODE,
            maximum_elevation: -1.,
            maximum_elevation_mode: PEAKS_MODE,
            first_frame: true,
            display_range: 0.,
            display_mode: 0.,
            frame_byte_count: 0,
        }
    }
}

/// `types::ThresholdData` as terronnd receives it: SimBridge packs the
/// packet with Node's `writeInt16LE`/`writeUInt8`/`writeUInt16LE`
/// (`communication/simconnect.ts`), which truncate to the field.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ThresholdData {
    pub lower_threshold: i16,
    pub lower_threshold_mode: u8,
    pub upper_threshold: i16,
    pub upper_threshold_mode: u8,
    pub first_frame: u8,
    pub display_range: u16,
    pub display_mode: u8,
    pub frame_byte_count: u32,
}

impl From<&NavigationDisplayData> for ThresholdData {
    fn from(d: &NavigationDisplayData) -> Self {
        Self {
            lower_threshold: d.minimum_elevation as i32 as i16,
            lower_threshold_mode: d.minimum_elevation_mode,
            upper_threshold: d.maximum_elevation as i32 as i16,
            upper_threshold_mode: d.maximum_elevation_mode,
            first_frame: d.first_frame as u8,
            display_range: crate::mapdata::terrain::geo::js_round(d.display_range) as i32 as u16,
            display_mode: d.display_mode as i32 as u8,
            frame_byte_count: d.frame_byte_count,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_bridges_json_is_read() {
        let efis = r#"{"ndRange":40,"arcMode":true,"terrOnNd":true,"terrOnVd":true,"efisMode":3,"vdRangeLower":-500,"vdRangeUpper":24000}"#;
        let body = format!(
            r#"{{"adiruDataValid":true,"tawsInop":false,"latitude":47.2,"longitude":11.3,"altitude":9000,"heading":263,"verticalSpeed":-1200,"gearIsDown":false,"runwayDataValid":true,"runwayLatitude":47.26,"runwayLongitude":11.34,"efisDataCapt":{efis},"efisDataFO":{efis},"navigationDisplayRenderingMode":3,"manualAzimEnabled":false,"manualAzimDegrees":263,"groundTruthLatitude":47.2,"groundTruthLongitude":11.3}}"#
        );
        let s = AircraftStatus::from_json(&body).unwrap();
        assert_eq!(s.navigation_display_rendering_mode, 3);
        assert_eq!(s.efis(Side::Right).nd_range, 40.);
        assert!(AircraftStatus::from_json(&body.replace("\"tawsInop\":false,", "")).is_err());
        let path = VerticalPathData::from_json(r#"{"pathWidth":1,"trackChangesSignificantlyAtDistance":-1,"waypoints":[{"latitude":47,"longitude":11}]}"#).unwrap();
        assert_eq!(path.waypoints, vec![(47., 11.)]);
    }
}
