//! SimBridge's terrain geometry helpers (simbridge `apps/server/src/terrain/
//! processing/generic/helper.ts` and `processing/gpu/helper.ts`), line for
//! line. SimBridge runs most of them as gpu.js kernels in 32-bit floats;
//! here they run on the CPU in 64-bit floats, as gpu.js's own CPU mode does.

pub const FEET_PER_NAUTICAL_MILE: f64 = 6076.12;
pub const THREE_NAUTICAL_MILES_IN_FEET: f64 = 18228.3;
pub const NAUTICAL_MILES_TO_METRES: f64 = 1852.;

// map grid creation (generic/constants.ts)
pub const INVALID_ELEVATION: i16 = 32767;
pub const UNKNOWN_ELEVATION: i16 = 32766;
pub const WATER_ELEVATION: i16 = -1;
pub const DEFAULT_TILE_SIZE: usize = 300;

pub fn deg2rad(degree: f64) -> f64 {
    degree * (std::f64::consts::PI / 180.)
}

pub fn rad2deg(radian: f64) -> f64 {
    radian * (180. / std::f64::consts::PI)
}

/// JavaScript's `Math.round`: halves go towards positive infinity.
pub fn js_round(v: f64) -> f64 {
    (v + 0.5).floor()
}

/// Great circle distance in nautical miles.
pub fn distance_wgs84(latitude0: f64, longitude0: f64, latitude1: f64, longitude1: f64) -> f64 {
    let delta_latitude = deg2rad(latitude1 - latitude0);
    let delta_longitude = deg2rad(longitude1 - longitude0);
    let latitude0radian = deg2rad(latitude0);
    let latitude1radian = deg2rad(latitude1);
    let a = 0.5 - delta_latitude.cos() * 0.5
        + latitude0radian.cos() * latitude1radian.cos() * (1. - delta_longitude.cos()) * 0.5;
    let distance_metres = 12742020. * a.sqrt().asin();
    distance_metres * 0.000539957
}

/// Degrees of latitude and longitude per world map pixel.
pub fn degrees_per_pixel(
    southwest_latitude: f64,
    southwest_longitude: f64,
    northeast_latitude: f64,
    northeast_longitude: f64,
    current_latitude: f64,
    map_width: f64,
    map_height: f64,
) -> (f64, f64) {
    let mut lat_step = if southwest_latitude >= current_latitude {
        // we are at the south pole
        southwest_latitude + northeast_latitude + 180.
    } else if northeast_latitude <= current_latitude {
        // we are at the north pole
        180. - southwest_latitude - northeast_latitude
    } else {
        northeast_latitude - southwest_latitude
    };
    lat_step /= map_height;
    let mut long_step = if northeast_longitude < southwest_longitude {
        180. - southwest_longitude + (northeast_longitude + 180.).abs()
    } else {
        northeast_longitude - southwest_longitude
    };
    long_step /= map_width;
    (lat_step, long_step)
}

pub fn normalize_heading(angle: f64) -> f64 {
    angle - (angle / 360.).floor() * 360.
}

/// The point `distance` metres from a position along a bearing.
pub fn project_wgs84(latitude: f64, longitude: f64, bearing: f64, distance: f64) -> (f64, f64) {
    let lat_rad = deg2rad(latitude);
    let long_rad = deg2rad(longitude);
    let bearing_rad = deg2rad(bearing);
    let ratio = distance / 6371010.;
    let lat_dest = (lat_rad.sin() * ratio.cos() + lat_rad.cos() * ratio.sin() * bearing_rad.cos()).asin();
    let long_dest = long_rad
        + (bearing_rad.sin() * ratio.sin() * lat_rad.cos()).atan2(ratio.cos() - lat_rad.sin() * lat_dest.sin());
    // ensure that the latitude is between [-90.0, 90.0]
    let mut lat_dest = rad2deg(lat_dest);
    if lat_dest < -90. {
        lat_dest = -180. - lat_dest;
    }
    if lat_dest > 90. {
        lat_dest = 180. - lat_dest;
    }
    // ensure that the longitude is between [-180.0, 180.0]
    let mut long_dest = rad2deg(long_dest);
    if long_dest < -180. {
        long_dest += 360.;
    }
    if long_dest > 180. {
        long_dest -= 360.;
    }
    (lat_dest, long_dest)
}

pub fn bearing_wgs84(latitude0: f64, longitude0: f64, latitude1: f64, longitude1: f64) -> f64 {
    let start_lat = deg2rad(latitude0);
    let start_long = deg2rad(longitude0);
    let end_lat = deg2rad(latitude1);
    let end_long = deg2rad(longitude1);
    let y = (end_long - start_long).sin() * end_lat.cos();
    let x = start_lat.cos() * end_lat.sin() - start_lat.sin() * end_lat.cos() * (end_long - start_long).cos();
    let bearing = y.atan2(x) + std::f64::consts::PI;
    (rad2deg(bearing) + 360.) % 360.
}

/// Where the world map has the world map layout this pixel mapping needs.
#[derive(Clone, Copy, Debug, Default)]
pub struct WorldMapFrame {
    pub ground_truth_latitude: f64,
    pub ground_truth_longitude: f64,
    pub southwest_lat: f64,
    pub southwest_long: f64,
    pub northeast_lat: f64,
    pub northeast_long: f64,
    pub width: f64,
    pub height: f64,
    pub grid_x: f64,
    pub grid_y: f64,
}

/// A position's world map pixel, as `wgs84toPixelCoordinate` computes it
/// (including its comparison of a longitude with the ground truth latitude).
pub fn wgs84_to_pixel_coordinate(latitude: f64, projected_latitude: f64, projected_longitude: f64, map: &WorldMapFrame) -> (f64, f64) {
    let mut lat_step = if map.southwest_lat >= latitude {
        // we are at the south pole
        map.southwest_lat + map.northeast_lat + 180.
    } else if map.northeast_lat <= latitude {
        // we are at the north pole
        180. - map.southwest_lat - map.northeast_lat
    } else {
        map.northeast_lat - map.southwest_lat
    };
    lat_step /= map.height;
    let mut long_step = if map.northeast_long < map.southwest_long {
        180. - map.southwest_long + (map.northeast_long + 180.).abs()
    } else {
        map.northeast_long - map.southwest_long
    };
    long_step /= map.width;
    let lat_pixel_delta = (map.ground_truth_latitude - projected_latitude) / lat_step;
    let mut long_pixel_delta = if (projected_longitude - map.ground_truth_longitude).abs() >= 180. {
        if projected_longitude > map.ground_truth_latitude {
            180. - projected_longitude + map.ground_truth_longitude.abs() - 180.
        } else {
            180. - map.ground_truth_longitude + projected_longitude.abs() - 180.
        }
    } else {
        projected_longitude - map.ground_truth_longitude
    };
    long_pixel_delta /= long_step;
    (js_round(map.grid_x + long_pixel_delta), js_round(map.grid_y + lat_pixel_delta))
}

pub fn vertical_display_distance_to_pixel_x(distance: f64, range: f64) -> f64 {
    (distance / range) * 540.
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn projection_and_distance_agree() {
        let (lat, lon) = project_wgs84(47.26, 11.35, 90., 100. * NAUTICAL_MILES_TO_METRES);
        assert!((distance_wgs84(47.26, 11.35, lat, lon) - 100.).abs() < 0.1);
        let bearing = bearing_wgs84(47.26, 11.35, lat, lon);
        // SimBridge's bearing adds half a turn to atan2 (helper.ts).
        assert!((bearing - 270.).abs() < 1., "{bearing}");
        assert_eq!(normalize_heading(-10.), 350.);
        assert_eq!(js_round(-2.5), -2.);
    }
}
