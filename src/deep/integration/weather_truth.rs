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
//! - `weather`: `crate::xp::weather_at_location` (`XPLMGetWeatherAtLocation`,
//!   already wired for `src/wxr`), sampled at the aircraft's own position —
//!   gives precipitation rate, turbulence ratio and up to three cloud
//!   layers' type/coverage/base/top, all real X-Plane outputs. `None` when
//!   X-Plane has no weather API (`crate::xp::has_weather_api`) or no
//!   regional data at this point (its own documented "not world-wide"
//!   limitation) — every consumer below must treat that as "unknown", not
//!   "clear", per the no-fake-values rule (`fallback_dry` is explicit about
//!   this).
//!
//! Everything here is read through `Option<DataRef>`/`Option<&Xplm>`
//! exactly the way `physics/xp_effects.rs::XpEffects` already does, so it
//! degrades to a documented default rather than panicking when a dataref
//! is missing (an older SDK target, or the offline test harness).

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::xp::{DataRef, Xplm, WeatherSample};

/// kt -> m/s (NIST international nautical mile / 3600 s).
pub const KT_TO_MS: f64 = 0.514_444;
const GAMMA_AIR: f64 = 1.4;
/// Specific gas constant for dry air, J/(kg*K) (ISA/ICAO standard atmosphere).
const R_AIR_J_KGK: f64 = 287.052_87;

/// Real X-Plane weather/atmosphere at the aircraft, this tick. See module
/// doc for each field's exact source.
#[derive(Clone, Copy, Debug, Default)]
pub struct EnvironmentTruth {
    pub sat_c: f64,
    pub leading_edge_c: f64,
    pub ambient_pressure_pa: f64,
    pub tas_ms: f64,
    pub precipitation_on_aircraft_ratio: f64,
    /// `XPLMGetWeatherAtLocation` at the aircraft's own position/altitude;
    /// `None` if unavailable (see module doc) -- never defaulted to zero
    /// here, so a caller cannot mistake "unknown" for "clear skies".
    pub weather: Option<WeatherSample>,
}

impl EnvironmentTruth {
    /// True Mach from true airspeed and static air temperature (standard
    /// compressible-flow relation `a = sqrt(gamma*R*T)`, `M = V/a` -- the
    /// same formula `deep::sensors::tat_probe`'s module doc cites
    /// independently for the same reason: computed from the crate's own
    /// truth inputs rather than read from FlyByWire's own (potentially
    /// faulted) ADR Mach output, which would be circular for models that
    /// exist to fault that same ADR).
    pub fn mach(&self) -> f64 {
        mach_from_tas_sat(self.tas_ms, self.sat_c)
    }
}

/// Standard compressible-flow Mach number from true airspeed and static
/// air temperature; `sat_c` is floored at 1 K so a nonsensical/absent
/// reading can never divide by zero or take a negative square root.
pub fn mach_from_tas_sat(tas_ms: f64, sat_c: f64) -> f64 {
    let t_k = (sat_c + 273.15).max(1.0);
    let speed_of_sound_ms = (GAMMA_AIR * R_AIR_J_KGK * t_k).sqrt();
    (tas_ms.max(0.0) / speed_of_sound_ms).max(0.0)
}

/// One X-Plane weather cloud layer's type, `crate::xp::WeatherCloudLayer`'s
/// own documented enum (`XPLMWeatherInfoClouds_t`): 0 cirrus, 1 stratus,
/// 2 cumulus, 3 cumulonimbus.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CloudKind {
    Cirrus,
    Stratus,
    Cumulus,
    Cumulonimbus,
}

pub fn cloud_kind(cloud_type: f32) -> CloudKind {
    if cloud_type >= 2.5 {
        CloudKind::Cumulonimbus
    } else if cloud_type >= 1.5 {
        CloudKind::Cumulus
    } else if cloud_type >= 0.5 {
        CloudKind::Stratus
    } else {
        CloudKind::Cirrus
    }
}

/// The densest (highest-coverage-weighted) cloud layer of the up to three
/// `XPLMGetWeatherAtLocation` reports, or `None` if every layer is clear.
/// GENERIC combination rule (X-Plane gives three independent layers, not a
/// single "the" cloud): the layer with the greatest coverage wins, ties
/// broken toward the more convective type since that is the
/// aviation-relevant worst case for both icing severity and hazard events.
pub fn dominant_cloud(weather: &WeatherSample) -> Option<(CloudKind, f32)> {
    weather
        .clouds
        .iter()
        .filter(|c| c.coverage > 0.0)
        .max_by(|a, b| a.coverage.partial_cmp(&b.coverage).unwrap_or(std::cmp::Ordering::Equal).then((a.cloud_type as i32).cmp(&(b.cloud_type as i32))))
        .map(|c| (cloud_kind(c.cloud_type), c.coverage))
}

/// Liquid water content, kg/m^3, from static air temperature and the
/// dominant cloud's type/coverage.
///
/// 14 CFR Part 25 Appendix C's "Continuous Maximum" (stratiform) and
/// "Intermittent Maximum" (cumuliform) icing envelopes give LWC as a
/// function of temperature and cloud horizontal extent from a published
/// lookup chart, not a closed form; peaking in a band roughly -8..-12 C
/// and vanishing outside about -1..-30 C, with intermittent-maximum
/// (cumuliform) LWC running higher than continuous-maximum (stratiform) at
/// the same temperature. This is a GENERIC smooth (Gaussian) stand-in for
/// that chart's shape, peaking at -10 C with an 8 C half-width, scaled to
/// the peak order of magnitude `deep::fire_ice::icing`'s own module doc
/// already cites for continuous-maximum conditions (0.0002-0.0008 kg/m^3),
/// doubled for cumulus and roughly tripled for cumulonimbus to reproduce
/// intermittent-maximum's higher published peak -- not a digitisation of
/// the actual chart. Cirrus (X-Plane type 0) is an ice-crystal cloud, not
/// supercooled liquid, so it gives zero (rime/glaze icing needs liquid
/// water; ice-crystal icing is a distinct engine-core phenomenon this
/// function does not model -- see `PROGRESS.md`).
pub fn lwc_kg_m3_from_conditions(sat_c: f64, cloud: Option<(CloudKind, f32)>) -> f64 {
    let Some((kind, coverage)) = cloud else { return 0.0 };
    if kind == CloudKind::Cirrus || !(-30.0..=0.0).contains(&sat_c) {
        return 0.0;
    }
    let peak_kg_m3 = match kind {
        CloudKind::Stratus => 6.0e-4,
        CloudKind::Cumulus => 1.0e-3,
        CloudKind::Cumulonimbus => 1.6e-3,
        CloudKind::Cirrus => 0.0,
    };
    let bell = (-((sat_c + 10.0).powi(2)) / (2.0 * 8.0_f64.powi(2))).exp();
    peak_kg_m3 * bell * (coverage as f64).clamp(0.0, 1.0)
}

/// Median volumetric droplet diameter, m. GENERIC: convective (cumuliform)
/// cloud carries larger droplets than stratiform, both kept within the
/// 15-40 micron continuous-maximum band `deep::fire_ice::icing`'s module
/// doc cites (no per-cloud-type public figure exists to interpolate more
/// precisely than "toward the upper end for convective cloud").
pub fn droplet_diameter_m_from_conditions(cloud: Option<(CloudKind, f32)>) -> f64 {
    match cloud.map(|(k, _)| k) {
        Some(CloudKind::Cumulus) | Some(CloudKind::Cumulonimbus) => 28e-6,
        Some(CloudKind::Stratus) => 18e-6,
        _ => 0.0,
    }
}

/// "How deep into active convective weather" (0..1), the weather-model
/// input `deep::environment::lightning::LightningModel::step` documents
/// wanting. GENERIC combination: 1.0 only for a fully-covering
/// cumulonimbus layer with strong turbulence and precipitation at the
/// sampled altitude (X-Plane's own signature of an active convective
/// cell -- vigorous vertical motion is exactly what both drives charge
/// separation in a real thunderstorm and what `turbulence_alt` measures),
/// scaled down for a weaker/partial cell; zero for anything else.
pub fn convective_intensity(weather: &WeatherSample, cloud: Option<(CloudKind, f32)>) -> f64 {
    let Some((CloudKind::Cumulonimbus, coverage)) = cloud else { return 0.0 };
    let cell = (coverage as f64).clamp(0.0, 1.0);
    let vigour = (weather.turbulence_alt as f64).clamp(0.0, 1.0).max((weather.precip_rate_alt as f64).clamp(0.0, 1.0));
    cell * vigour
}

/// Hail intensity (0..1), the weather-model input
/// `deep::environment::hail::HailModel::step` documents wanting. GENERIC:
/// hail forms in the same vigorous convective updraft as lightning, so
/// this reuses [`convective_intensity`] directly rather than an
/// independently tuned curve (both are stand-ins for "how strong is this
/// cumulonimbus cell", not two physically distinct quantities X-Plane
/// separately reports).
pub fn hail_intensity(weather: &WeatherSample, cloud: Option<(CloudKind, f32)>) -> f64 {
    convective_intensity(weather, cloud)
}

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

    pub fn read<V: SimulatorReaderWriter>(&self, vars: &mut V, xplm: Option<&Xplm>) -> EnvironmentTruth {
        let f = |d: Option<DataRef>| d.map_or(0.0, |d| xplm.map_or(0.0, |x| x.get_f(d) as f64));
        let sat_c = vars.read(&self.ids.sat);
        let tas_ms = vars.read(&self.ids.tas) * KT_TO_MS;
        let latitude = vars.read(&self.ids.latitude);
        let longitude = f(self.refs.longitude);
        let elevation_m = f(self.refs.elevation_m);
        let weather = xplm.and_then(|_| crate::xp::weather_at_location(latitude, longitude, elevation_m));
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
    use crate::xp::WeatherCloudLayer;

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
