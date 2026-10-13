//! `EnvironmentTruth` -> `deep::fire_ice::icing::IcingEnvironment`, one per
//! surface the ice model tracks (wing leading edge x2, nacelle inlet x4,
//! windshield). `deep::fire_ice::icing::IcingSurface::step` already owns
//! the physics (Messinger freezing fraction, collection efficiency,
//! aerodynamic penalty); this is only the unit/quantity conversion from
//! real X-Plane weather truth into that struct's fields, since
//! `fire_ice`'s own module doc says its `IcingEnvironment` is "passed in
//! directly rather than hard-coded so callers can drive it from whatever
//! weather/cloud model the plugin uses" -- this is that caller.

use super::weather_truth::{dominant_cloud, droplet_diameter_m_from_conditions, lwc_kg_m3_from_conditions, EnvironmentTruth};
use crate::deep::fire_ice::icing::IcingEnvironment;

/// One `IcingEnvironment` shared by every airframe surface this tick (the
/// LWC/MVD/SAT/TAS/pressure are the same free-stream atmosphere regardless
/// of which surface is asked; `IcingSurface::step` applies each surface's
/// own geometry/heat-transfer coefficient on top of this).
pub fn icing_environment(truth: &EnvironmentTruth) -> IcingEnvironment {
    let cloud = truth.weather.as_ref().and_then(dominant_cloud);
    IcingEnvironment {
        lwc_kg_m3: lwc_kg_m3_from_conditions(truth.sat_c, cloud),
        droplet_diameter_m: droplet_diameter_m_from_conditions(cloud),
        static_air_c: truth.sat_c,
        tas_m_s: truth.tas_ms,
        ambient_pressure_pa: truth.ambient_pressure_pa,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::fire_ice::icing::{IcingSurface, WING_LEADING_EDGE};
    use crate::deep::integration::weather_truth::CloudKind;
    use crate::deep::weather::{WeatherCloudLayer, WeatherSample};

    fn truth_in_icing_stratus() -> EnvironmentTruth {
        let mut clouds = [WeatherCloudLayer::default(); 3];
        clouds[0] = WeatherCloudLayer { cloud_type: 1.0, coverage: 1.0, alt_base_m: 500.0, alt_top_m: 3000.0 };
        EnvironmentTruth {
            sat_c: -10.0,
            leading_edge_c: -8.0,
            ambient_pressure_pa: 80_000.0,
            tas_ms: 100.0,
            precipitation_on_aircraft_ratio: 0.0,
            weather: Some(WeatherSample { precip_rate_alt: 0.2, precip_rate: 0.2, turbulence_alt: 0.1, clouds, detailed: true }),
        }
    }

    #[test]
    fn a_stratus_cloud_at_minus_ten_produces_a_nonzero_icing_environment() {
        let env = icing_environment(&truth_in_icing_stratus());
        assert!(env.lwc_kg_m3 > 0.0);
        assert!(env.droplet_diameter_m > 0.0);
        assert_eq!(env.static_air_c, -10.0);
        assert_eq!(env.tas_m_s, 100.0);
    }

    #[test]
    fn clear_sky_gives_zero_lwc_so_no_ice_accretes() {
        let mut truth = truth_in_icing_stratus();
        truth.weather = None;
        let env = icing_environment(&truth);
        assert_eq!(env.lwc_kg_m3, 0.0);
        let mut surface = IcingSurface::new(WING_LEADING_EDGE);
        let out = surface.step(&env, 0.0, 60.0);
        assert_eq!(out.ice_mass_kg, 0.0);
    }

    #[test]
    fn feeding_a_wing_leading_edge_surface_accretes_ice_under_the_derived_environment() {
        let env = icing_environment(&truth_in_icing_stratus());
        let mut surface = IcingSurface::new(WING_LEADING_EDGE);
        let mut out = Default::default();
        for _ in 0..600 {
            out = surface.step(&env, 0.0, 1.0);
        }
        assert!(out.ice_mass_kg > 0.0, "should accrete over 10 minutes in a derived icing environment");
    }

    #[test]
    fn dominant_cloud_kind_is_reachable_from_the_public_reexport() {
        // Sanity that this adapter and `weather_truth` agree on the enum
        // it consumes (a refactor of one without the other would fail to
        // compile here, not silently drift).
        let _: Option<CloudKind> = None;
    }
}
