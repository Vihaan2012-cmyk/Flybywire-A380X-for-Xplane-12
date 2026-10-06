use super::weather_truth::{convective_intensity as convective_intensity_from_cloud, dominant_cloud, hail_intensity as hail_intensity_from_cloud, EnvironmentTruth};
use crate::deep::environment::runway_contamination::Contaminant;
use crate::deep::environment::wind_shear::{TurbulenceIntensity, WindShearInputs};

pub fn lightning_convective_intensity(truth: &EnvironmentTruth) -> f64 {
    let Some(weather) = truth.weather.as_ref() else { return 0.0 };
    convective_intensity_from_cloud(weather, dominant_cloud(weather))
}

pub fn hail_intensity(truth: &EnvironmentTruth) -> f64 {
    let Some(weather) = truth.weather.as_ref() else { return 0.0 };
    hail_intensity_from_cloud(weather, dominant_cloud(weather))
}

pub fn turbulence_intensity(truth: &EnvironmentTruth) -> Option<TurbulenceIntensity> {
    let ratio = truth.weather.as_ref()?.turbulence_alt.clamp(0.0, 1.0);
    Some(if ratio < 1.0 / 3.0 {
        return None;
    } else if ratio < 2.0 / 3.0 {
        TurbulenceIntensity::Light
    } else if ratio < 0.9 {
        TurbulenceIntensity::Moderate
    } else {
        TurbulenceIntensity::Severe
    })
}

pub fn ice_water_content_g_m3(truth: &EnvironmentTruth) -> f64 {
    if truth.sat_c > -20.0 {
        return 0.0;
    }
    let Some(weather) = truth.weather.as_ref() else { return 0.0 };
    let cloud = dominant_cloud(weather);
    3.0 * convective_intensity_from_cloud(weather, cloud)
}

const WET_SNOW_MIN_C: f64 = -5.0;

pub fn contaminant_from_weather(sat_c: f64, precipitation_on_aircraft_ratio: f64) -> Contaminant {
    let precip = precipitation_on_aircraft_ratio.clamp(0.0, 1.0);
    if precip < 0.05 {
        return Contaminant::Dry;
    }
    let depth_mm = precip * 6.0;
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

pub fn night_from_sun(sun_pitch_deg: f64, sun_heading_deg: f64) -> bool {
    crate::extra_backend_fbw::time_of_day_from_sun(sun_pitch_deg, sun_heading_deg) == 3
}

pub fn month_from_day_of_year(day_of_year_0based: u16) -> u8 {
    const CUMULATIVE: [u16; 12] = [0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334];
    let d = day_of_year_0based % 365;
    CUMULATIVE.iter().rposition(|&c| d >= c).map_or(1, |i| i as u8 + 1)
}

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
    use crate::deep::weather::{WeatherCloudLayer, WeatherSample};

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
        assert_eq!(month_from_day_of_year(0), 1);
        assert_eq!(month_from_day_of_year(31), 2);
        assert_eq!(month_from_day_of_year(364), 12);
    }

    #[test]
    fn a_headwind_gain_then_loss_produces_the_bowles_f_factor_hazard_signature() {
        let mut sampler = WindShearSampler::new();
        let before = sampler.step(360.0, 20.0, 0.0, 0.0, 70.0, 1.0);
        let after = sampler.step(180.0, 20.0, 0.0, 5.0, 70.0, 1.0);
        let f_before = f_factor(&before);
        let f_after = f_factor(&after);
        assert!(f_after > f_before, "a swing from headwind to tailwind should read as more hazardous");
    }
}
