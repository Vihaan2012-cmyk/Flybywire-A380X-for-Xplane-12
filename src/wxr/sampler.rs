//! Where a sampled cell's weather comes from: X-Plane's
//! `XPLMGetWeatherAtLocation` (`crate::xp::weather_at_location`, main thread
//! only, src/xp.rs "[wxr]"), behind a trait so the polar sweep and the image
//! it builds can be tested against a synthetic storm instead of a running
//! X-Plane.

/// What one sampled point gives the classifier: the two `XPLMWeatherInfo_t`
/// fields with a real bearing on a return (xp.rs `WeatherSample`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Sample {
    pub precip_rate_alt: f32,
    pub turbulence_alt: f32,
}

/// A source of weather at a point. `crate::xp::weather_at_location` is the
/// real one; tests use a synthetic field instead.
pub trait WeatherSampler {
    /// `None` if X-Plane has nothing there (no weather API, or the point is
    /// outside its regional data -- `XPLMGetWeatherAtLocation`'s "does not
    /// work world-wide" in XPLMWeather.h).
    fn sample(&mut self, lat: f64, lon: f64, alt_m: f64) -> Option<Sample>;
}

/// X-Plane's own weather, main thread only (the header: "not intended to be
/// used per-frame ... called only during the pre-flight loop callback"; this
/// plugin instead budgets a handful of calls per tick -- see `wxr/mod.rs`).
pub struct XplmSampler;

impl WeatherSampler for XplmSampler {
    fn sample(&mut self, lat: f64, lon: f64, alt_m: f64) -> Option<Sample> {
        let w = crate::xp::weather_at_location(lat, lon, alt_m)?;
        Some(Sample { precip_rate_alt: w.precip_rate_alt, turbulence_alt: w.turbulence_alt })
    }
}

/// A synthetic storm for tests: a single circular precipitation cell
/// centred at `(lat, lon)`, `radius_nm` across, at full precipitation in
/// the middle and fading linearly to the edge; optionally with turbulence
/// baked in. Distances are the flat-earth approximation (fine over the few
/// tens of nautical miles the tests use).
#[cfg(test)]
pub struct SyntheticStorm {
    pub lat: f64,
    pub lon: f64,
    pub radius_nm: f64,
    pub peak_precip: f32,
    pub peak_turbulence: f32,
}

#[cfg(test)]
impl WeatherSampler for SyntheticStorm {
    fn sample(&mut self, lat: f64, lon: f64, _alt_m: f64) -> Option<Sample> {
        let d_nm = crate::mapdata::terrain::geo::distance_wgs84(self.lat, self.lon, lat, lon);
        let t = (1. - d_nm / self.radius_nm).clamp(0., 1.);
        Some(Sample { precip_rate_alt: self.peak_precip * t as f32, turbulence_alt: self.peak_turbulence * t as f32 })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn synthetic_storm_peaks_at_its_centre_and_fades_out() {
        let mut storm = SyntheticStorm { lat: 10., lon: 10., radius_nm: 20., peak_precip: 0.9, peak_turbulence: 0.6 };
        let centre = storm.sample(10., 10., 0.).unwrap();
        assert!((centre.precip_rate_alt - 0.9).abs() < 0.01);
        let edge = storm.sample(10.4, 10., 0.).unwrap(); // ~24 nm away
        assert_eq!(edge.precip_rate_alt, 0.);
    }
}
