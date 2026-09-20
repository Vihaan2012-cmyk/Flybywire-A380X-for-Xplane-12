//! Real X-Plane weather -> the weather-model inputs
//! `deep::environment`'s event models document wanting:
//! `lightning::LightningModel::step`'s `convective_intensity`,
//! `hail::HailModel::step`'s `hail_intensity`,
//! `wind_shear::TurbulenceModel::step`'s `TurbulenceIntensity` (this one
//! from a **real** X-Plane output, `turbulence_alt`, not a derived
//! heuristic), `wind_shear::f_factor`'s `WindShearInputs`,
//! `ice_crystal_icing::IceCrystalInputs`'s `ice_water_content_g_m3`, and a
//! best-effort `runway_contamination::Contaminant` classification.
//!
//! `deep::environment::bird_strike::BirdStrikeModel` needs a `FlightState`
//! that is mostly *flight* state, not weather (TAS, AGL, phase, gear, N1) —
//! out of this file's scope except for the two fields that genuinely come
//! from the environment (`month`, `night`), which reuse existing plugin
//! logic rather than re-deriving it (see [`night_from_sun`]).
//! `deep::environment::volcanic_ash` needs an ash concentration X-Plane's
//! weather system has no concept of at all (confirmed: no `sim/weather/*`
//! field or `XPLMWeatherInfo_t` member represents airborne ash) -- there is
//! nothing to adapt from real weather for it; see `PROGRESS.md`.

use super::weather_truth::{convective_intensity as convective_intensity_from_cloud, dominant_cloud, hail_intensity as hail_intensity_from_cloud, EnvironmentTruth};
use crate::deep::environment::runway_contamination::Contaminant;
use crate::deep::environment::wind_shear::{TurbulenceIntensity, WindShearInputs};

/// `LightningModel::step`'s `convective_intensity` input, straight from
/// [`super::weather_truth::convective_intensity`] (kept re-exported here so
/// every environment-events adapter is reachable from one module).
pub fn lightning_convective_intensity(truth: &EnvironmentTruth) -> f64 {
    let Some(weather) = truth.weather.as_ref() else { return 0.0 };
    convective_intensity_from_cloud(weather, dominant_cloud(weather))
}

/// `HailModel::step`'s `hail_intensity` input.
pub fn hail_intensity(truth: &EnvironmentTruth) -> f64 {
    let Some(weather) = truth.weather.as_ref() else { return 0.0 };
    hail_intensity_from_cloud(weather, dominant_cloud(weather))
}

/// `TurbulenceModel::step`'s `TurbulenceIntensity`, from X-Plane's own real
/// `turbulence_alt` (`XPLMWeatherInfo_t`, 0..1) -- unlike convective
/// intensity/hail, this needs no cloud-type inference at all, since X-Plane
/// already reports a turbulence ratio directly. GENERIC banding (X-Plane
/// gives one continuous ratio, `wind_shear.rs`'s `TurbulenceIntensity` only
/// three discrete bands): below 1/3 no turbulence at all, 1/3..2/3 Light,
/// 2/3..0.9 Moderate, 0.9..1 Severe -- independently chosen (not a reuse of
/// `src/wxr/levels.rs`'s own, differently-purposed `MAGENTA_TURBULENCE`
/// threshold, which that module's own privacy keeps out of reach here
/// anyway), with the top band narrowed to 0.9..1 so "severe" stays reserved
/// for X-Plane's own most extreme reports rather than a third of the range.
pub fn turbulence_intensity(truth: &EnvironmentTruth) -> Option<TurbulenceIntensity> {
    let ratio = truth.weather.as_ref()?.turbulence_alt.clamp(0.0, 1.0);
    Some(if ratio < 1.0 / 3.0 {
        return None; // below this, "no turbulence" is the honest answer
    } else if ratio < 2.0 / 3.0 {
        TurbulenceIntensity::Light
    } else if ratio < 0.9 {
        TurbulenceIntensity::Moderate
    } else {
        TurbulenceIntensity::Severe
    })
}

/// Ice water content of the core airflow, g/m^3, for
/// `ice_crystal_icing::IceCrystalInputs`. GENERIC: ice-crystal icing forms
/// deep inside a convective core well above the freezing level, where all
/// water is already glaciated (module doc of `ice_crystal_icing.rs`) --
/// modelled here as scaling with the same cumulonimbus coverage/vigour
/// [`super::weather_truth::convective_intensity`] already extracts, gated
/// on static air temperature being cold enough (`<= -20 C`, representative
/// of the mid/upper troposphere a deep convective core's glaciated region
/// occupies) that the crystals are fully frozen rather than a mixed-phase
/// or supercooled-liquid regime. Peak value (~3 g/m^3) is the order of
/// magnitude Mason/Strapp/Chow (cited in `ice_crystal_icing.rs`'s module
/// doc) report for deep convective cores; a GENERIC scaling, not measured.
pub fn ice_water_content_g_m3(truth: &EnvironmentTruth) -> f64 {
    if truth.sat_c > -20.0 {
        return 0.0;
    }
    let Some(weather) = truth.weather.as_ref() else { return 0.0 };
    let cloud = dominant_cloud(weather);
    3.0 * convective_intensity_from_cloud(weather, cloud)
}

/// Best-effort runway surface classification from real weather alone.
/// **Low confidence, documented as such**: X-Plane's SDK has no runway
/// contamination/depth dataref at all (confirmed: no `sim/weather/*` or
/// `sim/flightmodel/*` field represents standing water/snow/ice depth on a
/// specific runway) -- this only distinguishes "precipitating and at or
/// below freezing" from "precipitating and above freezing" from "not
/// precipitating", and invents a depth from precipitation ratio purely so
/// `runway_contamination::friction`'s 3 mm good/medium threshold has
/// something non-arbitrary to compare against; a real source (a scenery/
/// METAR/NOTAM-driven contamination report) would replace this outright
/// rather than refine it.
/// Coldest static air temperature at which falling snow still behaves as
/// *wet* snow. ICAO Doc 9981 / EASA's runway condition assessment matrix
/// define the two by cohesion, not by a number: dry snow can be blown or
/// brushed and will not hold together when squeezed, wet snow sticks and
/// packs into a snowball. That cohesion comes from liquid water in the
/// snowpack, which only survives within a few degrees of melting, so wet
/// snow is a near-freezing phenomenon -- around -5 C the free water is gone
/// and the fall is dry. The previous -15 C here classified ordinary dry
/// continental snowfall as wet, which is both meteorologically wrong and
/// the more optimistic of the two for braking action.
const WET_SNOW_MIN_C: f64 = -5.0;

pub fn contaminant_from_weather(sat_c: f64, precipitation_on_aircraft_ratio: f64) -> Contaminant {
    let precip = precipitation_on_aircraft_ratio.clamp(0.0, 1.0);
    if precip < 0.05 {
        return Contaminant::Dry;
    }
    let depth_mm = precip * 6.0; // GENERIC: spans the RCAM's 3 mm band across the observed ratio range
    if sat_c > 3.0 {
        Contaminant::Water { depth_mm }
    } else if sat_c > 0.0 {
        Contaminant::Slush { depth_mm }
    } else if sat_c > WET_SNOW_MIN_C {
        Contaminant::WetSnow { depth_mm }
    } else {
        Contaminant::DrySnow { depth_mm }
    }
}

/// Whether the sun is down, reusing `extra_backend_fbw::time_of_day_from_sun`
/// (already the plugin's own day/night classification, civil-twilight
/// based) rather than re-deriving a threshold independently -- `bird_strike`
/// module doc: strike rates and species behaviour differ by day/night.
pub fn night_from_sun(sun_pitch_deg: f64, sun_heading_deg: f64) -> bool {
    crate::extra_backend_fbw::time_of_day_from_sun(sun_pitch_deg, sun_heading_deg) == 3
}

/// Calendar month (1..=12) from X-Plane's own `sim/time/local_date_days`
/// (days since 1 Jan, 0-based) -- `bird_strike` module doc: strike rates
/// have a strong migratory-season/month dependence. Ordinary (non-leap)
/// cumulative day-of-year boundaries; a leap-year February 29th reads as
/// March 1st, a one-day-per-four-years inaccuracy no bird-strike-rate model
/// resolves anyway.
pub fn month_from_day_of_year(day_of_year_0based: u16) -> u8 {
    const CUMULATIVE: [u16; 12] = [0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334];
    let d = day_of_year_0based % 365;
    CUMULATIVE.iter().rposition(|&c| d >= c).map_or(1, |i| i as u8 + 1)
}

/// Bowles F-factor input assembly (`wind_shear::WindShearInputs`) from
/// real wind at the aircraft (`"AMBIENT WIND DIRECTION"`/`"AMBIENT WIND
/// VELOCITY"`, already-aliased `Var`s, `src/lib.rs`'s own comment: "the
/// a380_systems_wasm registry's own polar form of the wind vector") and the
/// aircraft's true heading, by finite-differencing the headwind component
/// tick to tick. Standard meteorological convention: wind direction is
/// where it blows *from*, so the headwind component is `speed *
/// cos(wind_dir - heading)` (zero difference = wind on the nose = pure
/// headwind). Vertical (downdraft) component is not derived here (no
/// aliased `Var` gives it; a raw `sim/weather/aircraft/wind_now_y_msc`
/// reading would, left to the caller per this crate's `Option<DataRef>`
/// convention) -- `downdraft_ms` is `0.0` until a caller supplies one,
/// which under-detects a shear signature that is downdraft-only with no
/// accompanying horizontal change; documented, not silently assumed away.
#[derive(Clone, Copy, Debug, Default)]
pub struct WindShearSampler {
    prev_headwind_ms: Option<f64>,
}

impl WindShearSampler {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn step(&mut self, wind_dir_true_deg: f64, wind_speed_ms: f64, heading_true_deg: f64, downdraft_ms: f64, tas_ms: f64, dt_s: f64) -> WindShearInputs {
        let delta = (wind_dir_true_deg - heading_true_deg).to_radians();
        let headwind_ms = wind_speed_ms.max(0.0) * delta.cos();
        let rate = self.prev_headwind_ms.map_or(0.0, |p| (headwind_ms - p) / dt_s.max(1e-3));
        self.prev_headwind_ms = Some(headwind_ms);
        WindShearInputs { headwind_rate_ms2: rate, downdraft_ms, tas_ms }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::environment::runway_contamination::friction;
    use crate::deep::environment::wind_shear::f_factor;
    use crate::xp::{WeatherCloudLayer, WeatherSample};

    // `LightningModel::step`/`HailModel::step`/`TurbulenceModel::step`/
    // `BirdStrikeModel::step` all take `&mut environment::rng::Rng`, but
    // `src/deep/environment/mod.rs` declares that module as plain `mod
    // rng;` (private) rather than `pub(crate) mod rng;` -- so `Rng` cannot
    // be named or constructed from outside `deep::environment`, and these
    // otherwise-`pub` step functions cannot actually be called from this
    // (or any other) adapter as currently declared. Documented as an exact
    // one-line patch in `docs/deep/integration.md`; the tests below stop at
    // this module's own pure outputs (the correct scope for an adapter
    // anyway) rather than re-testing the other area's models.

    fn cb_truth(turbulence: f32, precip: f32) -> EnvironmentTruth {
        let mut clouds = [WeatherCloudLayer::default(); 3];
        clouds[0] = WeatherCloudLayer { cloud_type: 3.0, coverage: 0.9, alt_base_m: 500.0, alt_top_m: 12000.0 };
        EnvironmentTruth {
            sat_c: -25.0,
            weather: Some(WeatherSample { precip_rate_alt: precip, precip_rate: precip, turbulence_alt: turbulence, clouds, detailed: true }),
            ..Default::default()
        }
    }

    #[test]
    fn a_calm_clear_sky_drives_no_lightning_or_hail_intensity() {
        let truth = EnvironmentTruth::default();
        assert_eq!(lightning_convective_intensity(&truth), 0.0);
        assert_eq!(hail_intensity(&truth), 0.0);
    }

    #[test]
    fn a_strong_convective_cell_drives_high_lightning_and_hail_intensity() {
        let truth = cb_truth(0.8, 0.9);
        let intensity = lightning_convective_intensity(&truth);
        assert!(intensity > 0.5 && intensity <= 1.0);
        // Both intensities are the same underlying convective signature
        // (module doc), so they must agree exactly.
        assert_eq!(hail_intensity(&truth), intensity);
    }

    #[test]
    fn a_stratus_layer_never_drives_convective_intensity_regardless_of_turbulence() {
        let mut clouds = [WeatherCloudLayer::default(); 3];
        clouds[0] = WeatherCloudLayer { cloud_type: 1.0, coverage: 1.0, alt_base_m: 500.0, alt_top_m: 3000.0 };
        let truth = EnvironmentTruth {
            weather: Some(WeatherSample { precip_rate_alt: 0.9, precip_rate: 0.9, turbulence_alt: 0.9, clouds, detailed: true }),
            ..Default::default()
        };
        assert_eq!(lightning_convective_intensity(&truth), 0.0);
        assert_eq!(hail_intensity(&truth), 0.0);
    }

    #[test]
    fn turbulence_intensity_bands_the_real_turbulence_alt_ratio() {
        assert_eq!(turbulence_intensity(&cb_truth(0.1, 0.0)), None);
        assert_eq!(turbulence_intensity(&cb_truth(0.5, 0.0)), Some(TurbulenceIntensity::Light));
        assert_eq!(turbulence_intensity(&cb_truth(0.8, 0.0)), Some(TurbulenceIntensity::Moderate));
        assert_eq!(turbulence_intensity(&cb_truth(0.95, 0.0)), Some(TurbulenceIntensity::Severe));
    }

    // A test exercising `TurbulenceModel::step` end-to-end would need to
    // construct `environment::rng::Rng`, which is not reachable from here
    // (see the block comment above `mod tests`); `turbulence_intensity`'s
    // own banding is covered by the test below instead.

    #[test]
    fn ice_water_content_needs_both_a_deep_convective_core_and_glaciated_temperature() {
        assert!(ice_water_content_g_m3(&cb_truth(0.8, 0.8)) > 0.0, "cold deep convective core should carry ice water content");
        let warm_cb = EnvironmentTruth { sat_c: -5.0, ..cb_truth(0.8, 0.8) };
        assert_eq!(ice_water_content_g_m3(&warm_cb), 0.0, "too warm for a glaciated core");
        let cold_clear = EnvironmentTruth { sat_c: -40.0, weather: None, ..Default::default() };
        assert_eq!(ice_water_content_g_m3(&cold_clear), 0.0, "cold but no convective core at all");
        assert!(ice_water_content_g_m3(&cb_truth(0.9, 0.9)) > 0.0);
    }

    #[test]
    fn contaminant_classification_follows_temperature_once_precipitating() {
        assert_eq!(contaminant_from_weather(15.0, 0.0), Contaminant::Dry);
        assert!(matches!(contaminant_from_weather(10.0, 0.5), Contaminant::Water { .. }));
        assert!(matches!(contaminant_from_weather(-10.0, 0.5), Contaminant::DrySnow { .. }));
        // Feeds straight into the real friction model without further glue.
        let c = contaminant_from_weather(-10.0, 0.6);
        let out = friction(c, 200.0, 80.0);
        assert!(out.mu_effective > 0.0 && out.mu_effective < 1.0);
    }

    #[test]
    fn night_from_sun_matches_the_existing_civil_twilight_classification() {
        assert!(!night_from_sun(10.0, 90.0), "sun above horizon is day");
        assert!(night_from_sun(-10.0, 90.0), "well below horizon is night");
    }

    #[test]
    fn month_from_day_of_year_matches_ordinary_calendar_boundaries() {
        assert_eq!(month_from_day_of_year(0), 1); // Jan 1
        assert_eq!(month_from_day_of_year(31), 2); // Feb 1
        assert_eq!(month_from_day_of_year(364), 12); // Dec 31
    }

    #[test]
    fn a_headwind_gain_then_loss_produces_the_bowles_f_factor_hazard_signature() {
        let mut sampler = WindShearSampler::new();
        // Wind directly on the nose (0 deg relative), then swinging to a
        // tailwind over one second: a sharp headwind *loss*, the hazardous
        // half of a microburst encounter.
        let before = sampler.step(360.0, 20.0, 0.0, 0.0, 70.0, 1.0);
        let after = sampler.step(180.0, 20.0, 0.0, 5.0, 70.0, 1.0);
        let f_before = f_factor(&before);
        let f_after = f_factor(&after);
        assert!(f_after > f_before, "a swing from headwind to tailwind should read as more hazardous");
    }
}
