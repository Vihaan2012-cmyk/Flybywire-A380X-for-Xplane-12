//! Real X-Plane weather/atmosphere at the aircraft, gathered once per tick
//! into one [`EnvironmentTruth`], plus the pure physics that turns
//! "temperature and cloud type" into the liquid-water-content/droplet-size
//! inputs the icing models need (no X-Plane dataref publishes LWC/MVD
//! directly -- confirmed by grepping this crate's own uses of
//! `sim/weather/*` and the `XPLMWeatherInfo_t` fields `crate::xp` already
//! exposes; only temperature, pressure, precipitation ratio, cloud type/
//! coverage/base/top and turbulence are real X-Plane outputs).
//!
//! ## Sources for each field (see also `crate::xp`'s own `[wxr]`-tagged
//! section, which already wires `XPLMGetWeatherAtLocation`)
//! - `sat_c`: `sim/weather/aircraft/temperature_ambient_deg_c`, already
//!   aliased as the plugin's `"AMBIENT TEMPERATURE"` `Var`
//!   (`src/lib.rs:629`) and read here the same way `fadec.rs`/`fuel.rs`/
//!   `engine_commands.rs` already do -- true static air temperature at the
//!   aircraft, X-Plane's own real weather, not FlyByWire's ADR (which can
//!   itself be faulted -- feeding *that* back into the fault models would
//!   be circular).
//! - `tas_ms`: `"AIRSPEED TRUE"`, the same `Var` `fuel.rs`/
//!   `engine_commands.rs` already read, kt converted to m/s.
//! - `leading_edge_c`: `sim/weather/aircraft/temperature_leadingedge_deg_c`,
//!   already read raw (not aliased as a `Var`) by
//!   `engine_commands.rs:281`/`physics/tyre.rs:253` -- X-Plane's own
//!   kinetically-heated skin temperature, a real recovery-temperature
//!   output, read directly here rather than recomputed, since X-Plane
//!   already has the aircraft's real skin-exposure geometry this crate
//!   does not.
//! - `ambient_pressure_pa`: `sim/weather/aircraft/barometer_current_pas`,
//!   already read raw by `physics/adirs.rs:1518`.
//! - `precipitation_on_aircraft_ratio`: `sim/weather/aircraft/
//!   precipitation_on_aircraft_ratio`, already read raw by
//!   `physics/adirs.rs:1524` (its own doc there: 0..1).
//! - `latitude`/`longitude`/`elevation_m`: `"PLANE LATITUDE"` (aliased
//!   `Var`, `src/lib.rs:664`) plus `sim/flightmodel/position/longitude`/
//!   `sim/flightmodel/position/elevation`, the same raw datarefs
//!   `extra_backend_fbw.rs:976`/`lib.rs` already use for longitude
//!   (`elevation` is X-Plane's fundamental MSL-metres position dataref,
//!   used here only to feed `XPLMGetWeatherAtLocation`'s altitude
//!   argument, not published further).
//! - `weather`: [`deep::weather::WeatherSource::weather_at_location`]
//!   (X-Plane's implementation forwards to `XPLMGetWeatherAtLocation`,
//!   already wired for `src/wxr`), sampled at the aircraft's own position —
//!   gives precipitation rate, turbulence ratio and up to three cloud
//!   layers' type/coverage/base/top, all real X-Plane outputs on that host.
//!   `None` when the host has no weather API at all
//!   (`WeatherSource::has_weather_api`) or no regional data at this point
//!   (X-Plane's own documented "not world-wide" limitation) — every
//!   consumer below must treat that as "unknown", not "clear", per the
//!   no-fake-values rule (`fallback_dry` is explicit about this).
//!
//! `weather` is read through the host-neutral [`deep::weather::WeatherSource`]
//! trait (`crate::xp` supplies the one live implementation, over
//! `XPLMGetWeatherAtLocation`); everything else here is still read through
//! `Option<DataRef>`/`Option<&Xplm>` directly, exactly the way
//! `physics/xp_effects.rs::XpEffects` already does, so it degrades to a
//! documented default rather than panicking when a dataref is missing (an
//! older SDK target, or the offline test harness).

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::deep::weather::{WeatherSample, WeatherSource};
use crate::xp::{DataRef, Xplm};

pub use super::weather_model::*;

// ---------------------------------------------------------------------------
// The plugin-side reader.
// ---------------------------------------------------------------------------

struct Ids {
    sat: VariableIdentifier,
    tas: VariableIdentifier,
    latitude: VariableIdentifier,
}

struct Refs {
    leading_edge_c: Option<DataRef>,
    ambient_pressure_pa: Option<DataRef>,
    precip_on_aircraft: Option<DataRef>,
    longitude: Option<DataRef>,
    elevation_m: Option<DataRef>,
}

/// Reads [`EnvironmentTruth`] each tick. Construct once (`new`), call
/// [`WeatherTruthReader::read`] every tick after X-Plane's own flight model
/// has run.
pub struct WeatherTruthReader {
    ids: Ids,
    refs: Refs,
}

impl WeatherTruthReader {
    pub fn new<V: VariableRegistry>(vars: &mut V, xplm: Option<&Xplm>) -> Self {
        Self {
            ids: Ids {
                sat: vars.get("AMBIENT TEMPERATURE".to_owned()),
                tas: vars.get("AIRSPEED TRUE".to_owned()),
                latitude: vars.get_unprefixed("PLANE LATITUDE".to_owned()),
            },
            refs: Refs {
                leading_edge_c: xplm.and_then(|x| x.find("sim/weather/aircraft/temperature_leadingedge_deg_c")),
                ambient_pressure_pa: xplm.and_then(|x| x.find("sim/weather/aircraft/barometer_current_pas")),
                precip_on_aircraft: xplm.and_then(|x| x.find("sim/weather/aircraft/precipitation_on_aircraft_ratio")),
                longitude: xplm.and_then(|x| x.find("sim/flightmodel/position/longitude")),
                elevation_m: xplm.and_then(|x| x.find("sim/flightmodel/position/elevation")),
            },
        }
    }

    /// `weather_source` is the same "are we live" presence this reader
    /// already keys off for everything else (`xplm`) -- pass
    /// `xplm.map(|x| x as &dyn WeatherSource)` to keep the identical
    /// gating X-Plane always had (`Some` exactly when `xplm` is `Some`);
    /// a different host passes its own [`WeatherSource`] implementation.
    pub fn read<V: SimulatorReaderWriter>(&self, vars: &mut V, xplm: Option<&Xplm>, weather_source: Option<&dyn WeatherSource>) -> EnvironmentTruth {
        let f = |d: Option<DataRef>| d.map_or(0.0, |d| xplm.map_or(0.0, |x| x.get_f(d) as f64));
        let sat_c = vars.read(&self.ids.sat);
        let tas_ms = vars.read(&self.ids.tas) * KT_TO_MS;
        let latitude = vars.read(&self.ids.latitude);
        let longitude = f(self.refs.longitude);
        let elevation_m = f(self.refs.elevation_m);
        let weather = weather_source.and_then(|w| w.weather_at_location(latitude, longitude, elevation_m));
        EnvironmentTruth {
            sat_c,
            leading_edge_c: f(self.refs.leading_edge_c),
            ambient_pressure_pa: f(self.refs.ambient_pressure_pa),
            tas_ms,
            precipitation_on_aircraft_ratio: f(self.refs.precip_on_aircraft),
            weather,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::weather::WeatherCloudLayer;

    fn sample(cloud_type: f32, coverage: f32, turbulence: f32, precip: f32) -> WeatherSample {
        let mut clouds = [WeatherCloudLayer::default(); 3];
        clouds[0] = WeatherCloudLayer { cloud_type, coverage, alt_base_m: 0.0, alt_top_m: 3000.0 };
        WeatherSample { precip_rate_alt: precip, precip_rate: precip, turbulence_alt: turbulence, clouds, detailed: true }
    }

    #[test]
    fn mach_is_zero_at_rest_and_finite_at_extreme_cold() {
        assert_eq!(mach_from_tas_sat(0.0, 15.0), 0.0);
        let m = mach_from_tas_sat(250.0, -80.0);
        assert!(m.is_finite() && m > 0.0);
    }

    #[test]
    fn mach_matches_the_standard_atmosphere_at_sea_level() {
        // ISA sea level: a = 340.3 m/s; 100 m/s TAS should read close to
        // 100/340.3.
        let m = mach_from_tas_sat(100.0, 15.0);
        assert!((m - 100.0 / 340.3).abs() < 0.01, "{m}");
    }

    #[test]
    fn cloud_kind_thresholds_match_xplanes_documented_enum() {
        assert_eq!(cloud_kind(0.0), CloudKind::Cirrus);
        assert_eq!(cloud_kind(1.0), CloudKind::Stratus);
        assert_eq!(cloud_kind(2.0), CloudKind::Cumulus);
        assert_eq!(cloud_kind(3.0), CloudKind::Cumulonimbus);
    }

    #[test]
    fn dominant_cloud_picks_the_highest_coverage_layer() {
        let mut w = sample(1.0, 0.2, 0.0, 0.0);
        w.clouds[1] = WeatherCloudLayer { cloud_type: 3.0, coverage: 0.9, alt_base_m: 0.0, alt_top_m: 0.0 };
        let (kind, coverage) = dominant_cloud(&w).unwrap();
        assert_eq!(kind, CloudKind::Cumulonimbus);
        assert_eq!(coverage, 0.9);
    }

    #[test]
    fn clear_sky_gives_no_dominant_cloud() {
        let w = sample(0.0, 0.0, 0.0, 0.0);
        assert!(dominant_cloud(&w).is_none());
    }

    #[test]
    fn lwc_is_zero_outside_the_icing_temperature_band_and_for_cirrus() {
        assert_eq!(lwc_kg_m3_from_conditions(15.0, Some((CloudKind::Stratus, 1.0))), 0.0, "above freezing");
        assert_eq!(lwc_kg_m3_from_conditions(-45.0, Some((CloudKind::Stratus, 1.0))), 0.0, "too cold for supercooled liquid");
        assert_eq!(lwc_kg_m3_from_conditions(-10.0, Some((CloudKind::Cirrus, 1.0))), 0.0, "ice-crystal cloud has no liquid water");
        assert_eq!(lwc_kg_m3_from_conditions(-10.0, None), 0.0, "no cloud at all");
    }

    #[test]
    fn lwc_peaks_near_minus_ten_and_scales_with_coverage_and_convection() {
        let peak = lwc_kg_m3_from_conditions(-10.0, Some((CloudKind::Stratus, 1.0)));
        let off_peak = lwc_kg_m3_from_conditions(-25.0, Some((CloudKind::Stratus, 1.0)));
        assert!(peak > off_peak && peak > 0.0);
        let half_cover = lwc_kg_m3_from_conditions(-10.0, Some((CloudKind::Stratus, 0.5)));
        assert!((half_cover - peak / 2.0).abs() < 1e-9);
        let cb = lwc_kg_m3_from_conditions(-10.0, Some((CloudKind::Cumulonimbus, 1.0)));
        assert!(cb > peak, "cumulonimbus should carry more LWC than stratus at the same temperature");
        // Order-of-magnitude sanity against `deep::fire_ice::icing`'s own
        // cited continuous-maximum figure (0.0002-0.0008 kg/m^3).
        assert!(peak >= 2e-4 && peak <= 8e-4);
    }

    #[test]
    fn droplet_diameter_stays_within_the_cited_continuous_maximum_band() {
        let d = droplet_diameter_m_from_conditions(Some((CloudKind::Stratus, 1.0)));
        assert!((15e-6..=40e-6).contains(&d));
        let d_cu = droplet_diameter_m_from_conditions(Some((CloudKind::Cumulus, 1.0)));
        assert!(d_cu > d, "convective cloud should carry larger droplets");
    }

    #[test]
    fn convective_intensity_needs_a_cumulonimbus_cell_not_just_turbulence() {
        let stratus_turbulent = sample(1.0, 1.0, 0.9, 0.9);
        assert_eq!(convective_intensity(&stratus_turbulent, dominant_cloud(&stratus_turbulent)), 0.0);
        let cb = sample(3.0, 0.8, 0.7, 0.5);
        let intensity = convective_intensity(&cb, dominant_cloud(&cb));
        assert!(intensity > 0.0 && intensity <= 1.0);
    }

    #[test]
    fn convective_intensity_is_zero_for_a_weak_calm_cumulonimbus_report() {
        let calm_cb = sample(3.0, 1.0, 0.0, 0.0);
        assert_eq!(convective_intensity(&calm_cb, dominant_cloud(&calm_cb)), 0.0, "no vertical vigour signature yet");
    }

    #[test]
    fn hail_intensity_tracks_convective_intensity() {
        let cb = sample(3.0, 0.6, 0.4, 0.9);
        assert_eq!(hail_intensity(&cb, dominant_cloud(&cb)), convective_intensity(&cb, dominant_cloud(&cb)));
    }
}
