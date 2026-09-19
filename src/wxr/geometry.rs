//! Pure geometry for the radar beam: where a polar cell (azimuth relative to
//! heading, range, tilt) points on the earth, and where a geo-referenced
//! return maps back onto the ND under the aircraft's current position and
//! heading. Built on FlyByWire's own projection math
//! (`crate::mapdata::terrain::geo`, SimBridge's `helper.ts` ported once
//! already for the terrain worker) instead of a second copy of it.

use crate::mapdata::terrain::geo::{bearing_wgs84, distance_wgs84, normalize_heading, project_wgs84, NAUTICAL_MILES_TO_METRES};

/// The earth radius `project_wgs84` assumes, for the line-of-sight drop a
/// level beam has at range (small-angle approximation of horizon dip).
const EARTH_RADIUS_M: f64 = 6_371_010.;

/// Where a beam at `azimuth_deg` (relative to `heading_true_deg`, positive
/// right of the nose) and `range_nm` points from `(lat, lon, alt_m)`,
/// tilted `tilt_deg` up from the local horizon (XPLMWeather.h's convention
/// for `EFIS_weather_tilt`, DataRefs.txt:4188, positive up). A level beam
/// (`tilt_deg == 0`) still loses altitude with range because the earth
/// curves away under a straight line: `range_m^2 / (2 * earth_radius)`.
pub fn beam_point(lat: f64, lon: f64, alt_m: f64, heading_true_deg: f64, azimuth_deg: f64, range_nm: f64, tilt_deg: f64) -> (f64, f64, f64) {
    let bearing = normalize_heading(heading_true_deg + azimuth_deg);
    let range_m = range_nm * NAUTICAL_MILES_TO_METRES;
    let (dest_lat, dest_lon) = project_wgs84(lat, lon, bearing, range_m);
    let curvature_drop_m = (range_m * range_m) / (2. * EARTH_RADIUS_M);
    let beam_alt_m = alt_m + range_m * tilt_deg.to_radians().tan() - curvature_drop_m;
    (dest_lat, dest_lon, beam_alt_m)
}

/// A geo-referenced return's position on the ND relative to the aircraft:
/// `x` right, `y` up, in pixels. `None` if it is beyond `range_nm`, outside
/// the antenna's `±half_sector_deg` (real weather radar never looks aft,
/// whichever way the ND itself is drawn -- a return that was ahead a
/// minute ago and now is not stops showing, same as the real thing), or
/// `range_nm` is not a usable range.
#[allow(clippy::too_many_arguments)]
pub fn screen_offset(
    lat: f64,
    lon: f64,
    own_lat: f64,
    own_lon: f64,
    heading_true_deg: f64,
    range_nm: f64,
    px_per_nm: f64,
    half_sector_deg: f64,
) -> Option<(f64, f64)> {
    if range_nm <= 0. || px_per_nm <= 0. {
        return None;
    }
    let d_nm = distance_wgs84(own_lat, own_lon, lat, lon);
    if d_nm > range_nm {
        return None;
    }
    // `bearing_wgs84` returns the reciprocal bearing (its own test in
    // geo.rs: "SimBridge's bearing adds half a turn to atan2"); add the
    // turn back to get the true bearing from us to the point.
    let bearing = normalize_heading(bearing_wgs84(own_lat, own_lon, lat, lon) + 180.);
    // -180..180, positive right of the nose (up on a heading-up ND).
    let relative = normalize_heading(bearing - heading_true_deg + 180.) - 180.;
    if !in_sector(relative, half_sector_deg) {
        return None;
    }
    let r_px = d_nm * px_per_nm;
    let theta = relative.to_radians();
    Some((r_px * theta.sin(), -r_px * theta.cos()))
}

/// Whether `azimuth_deg` (relative to the nose, as `beam_point` takes it)
/// falls inside the antenna's scan sector (`±half_width_deg`).
pub fn in_sector(azimuth_deg: f64, half_width_deg: f64) -> bool {
    azimuth_deg.abs() <= half_width_deg
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_forward_level_beam_stays_on_the_nose() {
        // Due north at the equator, heading north: forward (azimuth 0) at
        // 60 nm should land due north, further along the same meridian.
        let (lat, lon, alt) = beam_point(0., 0., 10_000., 0., 0., 60., 0.);
        assert!(lat > 0.);
        assert!((lon).abs() < 1e-6);
        // The curvature drop at 60 nm (~111 km) is a few hundred metres.
        assert!(alt < 10_000.);
        assert!(alt > 9_000.);
    }

    #[test]
    fn azimuth_turns_with_heading() {
        // Heading east, beam straight ahead (azimuth 0) should point east.
        let (lat, lon, _) = beam_point(0., 0., 0., 90., 0., 60., 0.);
        assert!(lon > 0.);
        assert!(lat.abs() < 1e-6);
    }

    #[test]
    fn positive_tilt_climbs_faster_than_curvature_drops() {
        let (_, _, level) = beam_point(0., 0., 10_000., 0., 0., 100., 0.);
        let (_, _, tilted_up) = beam_point(0., 0., 10_000., 0., 0., 100., 5.);
        assert!(tilted_up > level);
    }

    #[test]
    fn screen_offset_places_the_nose_straight_up() {
        // A point 10 nm due north of an aircraft heading north is dead
        // ahead: x ~ 0, y negative (up).
        let (x, y) = screen_offset(0.2, 0., 0., 0., 0., 40., 10., 60.).unwrap();
        assert!(x.abs() < 0.5, "x = {x}");
        assert!(y < 0.);
    }

    #[test]
    fn screen_offset_places_a_beam_to_the_right() {
        // A point 30° right of the nose (inside a ±60° sector, unlike due
        // east at 90°, which a real antenna could not see either) should
        // come back to the right: x positive.
        let (lat, lon, _) = beam_point(0., 0., 0., 0., 30., 10., 0.);
        let (x, y) = screen_offset(lat, lon, 0., 0., 0., 40., 10., 60.).unwrap();
        assert!(x > 0., "x = {x}");
        assert!(y < 0., "y = {y}");
    }

    #[test]
    fn beyond_range_is_excluded() {
        assert!(screen_offset(2., 0., 0., 0., 0., 40., 10., 60.).is_none());
    }

    #[test]
    fn behind_the_antenna_sector_is_excluded() {
        // Heading north, a point due south (behind) is outside a ±60° sector.
        assert!(screen_offset(-0.2, 0., 0., 0., 0., 40., 10., 60.).is_none());
    }

    #[test]
    fn sector_membership() {
        assert!(in_sector(0., 60.));
        assert!(in_sector(60., 60.));
        assert!(in_sector(-60., 60.));
        assert!(!in_sector(60.1, 60.));
        assert!(!in_sector(-179., 60.));
    }
}
