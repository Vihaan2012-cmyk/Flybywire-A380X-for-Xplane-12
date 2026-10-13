//! The weather capability `deep` needs from whatever host it runs in:
//! whether real weather data is available at all, and a sample at a
//! point. [`WeatherSample`]/[`WeatherCloudLayer`] are `deep`'s own data
//! types -- moved out of `crate::xp`, which now imports them back rather
//! than the other way around -- so nothing under `deep` needs to know
//! X-Plane's `XPLMWeatherInfo_t` shape to use them. `crate::xp` provides
//! the one live implementation of [`WeatherSource`] (`impl WeatherSource
//! for Xplm`, forwarding to `XPLMGetWeatherAtLocation` unchanged); an
//! MSFS implementation would supply another.

/// Cloud layer count each [`WeatherSample`] carries -- X-Plane's own
/// `XPLMWeatherInfo_t.cloud_layers` gives three.
pub const CLOUD_LAYERS: usize = 3;

/// One weather cloud layer: type (0 cirrus, 1 stratus, 2 cumulus, 3
/// cumulonimbus), coverage (0..1) and its base/top, metres MSL.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WeatherCloudLayer {
    pub cloud_type: f32,
    pub coverage: f32,
    pub alt_base_m: f32,
    pub alt_top_m: f32,
}

/// A weather sample at a point: precipitation, turbulence and up to three
/// cloud layers -- the fields `deep::environment`/`deep::fire_ice`'s
/// icing, lightning, hail and turbulence models actually key off.
#[derive(Clone, Copy, Debug, Default)]
pub struct WeatherSample {
    /// Precipitation rate at the sampled altitude, 0..1 (unitless ratio).
    pub precip_rate_alt: f32,
    /// Precipitation rate at 0 altitude, 0..1.
    pub precip_rate: f32,
    /// Turbulence ratio at the sampled altitude, 0..1.
    pub turbulence_alt: f32,
    pub clouds: [WeatherCloudLayer; CLOUD_LAYERS],
    /// Whether the host found a detailed (e.g. METAR-backed) report here,
    /// as opposed to the best interpolation it could give.
    pub detailed: bool,
}

/// What `deep` needs from a host's weather system: whether it has one at
/// all, and a sample at a point. See the module doc for which
/// implementation is live and which is offline/test-only.
pub trait WeatherSource {
    /// Whether this host answered with real weather data this session
    /// (X-Plane: the weather API exists, `XPLM400`+; a pre-12 SDK target
    /// does not have it). `false` does not mean "clear" -- every consumer
    /// must fall back to a documented dry/calm default, never a
    /// fabricated reading.
    fn has_weather_api(&self) -> bool;

    /// Weather at `(lat, lon)`, `alt_m` metres MSL, or `None` if
    /// unavailable (no weather API, or no regional data at this point --
    /// a real, documented "not world-wide" limitation, not an error).
    fn weather_at_location(&self, lat: f64, lon: f64, alt_m: f64) -> Option<WeatherSample>;
}

/// Offline/test implementation: answers with whatever was configured at
/// construction and never touches a real host. Lets `deep`'s own tests
/// (and any headless harness) exercise weather-dependent code without
/// X-Plane or MSFS. Never wired up in the live plugin path -- that is
/// `crate::xp`'s `impl WeatherSource for Xplm`.
#[derive(Clone, Copy, Debug, Default)]
pub struct OfflineWeatherSource {
    pub available: bool,
    pub sample: Option<WeatherSample>,
}

impl OfflineWeatherSource {
    /// No weather API at all (the pre-12-SDK case): every consumer must
    /// fall back to its documented dry/calm default.
    pub fn unavailable() -> Self {
        Self { available: false, sample: None }
    }

    /// A weather API that is available but reports clear, calm air.
    pub fn clear() -> Self {
        Self { available: true, sample: Some(WeatherSample::default()) }
    }

    /// A weather API available and reporting exactly `sample`.
    pub fn with_sample(sample: WeatherSample) -> Self {
        Self { available: true, sample: Some(sample) }
    }
}

impl WeatherSource for OfflineWeatherSource {
    fn has_weather_api(&self) -> bool {
        self.available
    }

    fn weather_at_location(&self, _lat: f64, _lon: f64, _alt_m: f64) -> Option<WeatherSample> {
        self.sample
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unavailable_never_answers() {
        let s = OfflineWeatherSource::unavailable();
        assert!(!s.has_weather_api());
        assert!(s.weather_at_location(0.0, 0.0, 0.0).is_none());
    }

    #[test]
    fn with_sample_is_available_and_ignores_the_queried_point() {
        let mut clouds = [WeatherCloudLayer::default(); CLOUD_LAYERS];
        clouds[0] = WeatherCloudLayer { cloud_type: 3.0, coverage: 1.0, alt_base_m: 500.0, alt_top_m: 12_000.0 };
        let sample = WeatherSample { precip_rate_alt: 0.5, precip_rate: 0.5, turbulence_alt: 0.4, clouds, detailed: true };
        let s = OfflineWeatherSource::with_sample(sample);
        assert!(s.has_weather_api());
        assert_eq!(s.weather_at_location(51.0, 0.0, 1000.0).unwrap().clouds[0].cloud_type, 3.0);
        // Same answer regardless of where it's asked -- it's a fixed fixture.
        assert_eq!(s.weather_at_location(-10.0, 100.0, 30_000.0).unwrap().clouds[0].cloud_type, 3.0);
    }
}
