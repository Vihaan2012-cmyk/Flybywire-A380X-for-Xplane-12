pub const CLOUD_LAYERS: usize = 3;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WeatherCloudLayer {
    pub cloud_type: f32,
    pub coverage: f32,
    pub alt_base_m: f32,
    pub alt_top_m: f32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct WeatherSample {
    pub precip_rate_alt: f32,
    pub precip_rate: f32,
    pub turbulence_alt: f32,
    pub clouds: [WeatherCloudLayer; CLOUD_LAYERS],
    pub detailed: bool,
}

pub trait WeatherSource {
    fn has_weather_api(&self) -> bool;

    fn weather_at_location(&self, lat: f64, lon: f64, alt_m: f64) -> Option<WeatherSample>;
}

#[derive(Clone, Copy, Debug, Default)]
pub struct OfflineWeatherSource {
    pub available: bool,
    pub sample: Option<WeatherSample>,
}

impl OfflineWeatherSource {
    pub fn unavailable() -> Self {
        Self { available: false, sample: None }
    }

    pub fn clear() -> Self {
        Self { available: true, sample: Some(WeatherSample::default()) }
    }

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
        assert_eq!(s.weather_at_location(-10.0, 100.0, 30_000.0).unwrap().clouds[0].cloud_type, 3.0);
    }
}
