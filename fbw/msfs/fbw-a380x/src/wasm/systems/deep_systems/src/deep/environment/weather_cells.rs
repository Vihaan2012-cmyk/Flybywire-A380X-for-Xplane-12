use super::hail;
use super::wind_shear::TurbulenceIntensity;

#[derive(Clone, Copy, Debug)]
pub struct WeatherCell {
    pub x_m: f64,
    pub z_m: f64,
    pub radius_m: f64,
    pub base_m: f64,
    pub top_m: f64,
    pub intensity: f64,
}

const MIN_RADIUS_M: f64 = 1500.0;
const MAX_RADIUS_EXTRA_M: f64 = 4500.0;
const MIN_TOP_M: f64 = 4000.0;
const MAX_TOP_EXTRA_M: f64 = 11000.0;
const BASE_FREEZING_LEVEL_FRACTION: f64 = 0.3;
const MIN_BASE_M: f64 = 500.0;

impl WeatherCell {
    pub fn from_xp_point_weather(x_m: f64, z_m: f64, precip_rate_frac: f64, thunderstorm_frac: f64, cloud_top_m: f64, freezing_level_m: f64) -> Option<WeatherCell> {
        let intensity = (0.3 * precip_rate_frac.clamp(0.0, 1.0) + 0.7 * thunderstorm_frac.clamp(0.0, 1.0)).clamp(0.0, 1.0);
        if intensity <= 0.0 {
            return None;
        }
        let radius_m = MIN_RADIUS_M + MAX_RADIUS_EXTRA_M * intensity;
        let top_m = if cloud_top_m > 0.0 { cloud_top_m } else { MIN_TOP_M + MAX_TOP_EXTRA_M * intensity };
        let base_m = (freezing_level_m * BASE_FREEZING_LEVEL_FRACTION).max(MIN_BASE_M).min(top_m * 0.5);
        Some(WeatherCell { x_m, z_m, radius_m, base_m, top_m, intensity })
    }

    pub fn updraft_peak_ms(&self) -> f64 {
        5.0 + 30.0 * self.intensity.clamp(0.0, 1.0)
    }

    pub fn downdraft_peak_ms(&self) -> f64 {
        0.4 * self.updraft_peak_ms()
    }

    fn horizontal_weight(&self, x_m: f64, z_m: f64) -> f64 {
        let d = ((x_m - self.x_m).powi(2) + (z_m - self.z_m).powi(2)).sqrt();
        (1.0 - d / self.radius_m.max(1.0)).clamp(0.0, 1.0)
    }

    fn vertical_weight(&self, altitude_m: f64) -> f64 {
        if altitude_m < self.base_m || altitude_m > self.top_m {
            return 0.0;
        }
        let span = (self.top_m - self.base_m).max(1.0);
        let edge = (0.2 * span).max(1.0);
        let dist_from_edge = (altitude_m - self.base_m).min(self.top_m - altitude_m);
        (dist_from_edge / edge).clamp(0.0, 1.0)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct CellInfluence {
    pub convective_intensity: f64,
    pub hail_intensity: f64,
    pub ice_water_content_g_m3: f64,
    pub turbulence_intensity: Option<TurbulenceIntensity>,
    pub in_cloud: bool,
}

const PEAK_IWC_G_M3: f64 = 6.0;

#[derive(Clone, Debug, Default)]
pub struct WeatherCellField {
    pub cells: Vec<WeatherCell>,
}

impl WeatherCellField {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add(&mut self, cell: WeatherCell) {
        self.cells.push(cell);
    }

    pub fn clear(&mut self) {
        self.cells.clear();
    }

    pub fn set_from_xp_point_weather(&mut self, aircraft_x_m: f64, aircraft_z_m: f64, precip_rate_frac: f64, thunderstorm_frac: f64, cloud_top_m: f64, freezing_level_m: f64) {
        self.clear();
        if let Some(c) = WeatherCell::from_xp_point_weather(aircraft_x_m, aircraft_z_m, precip_rate_frac, thunderstorm_frac, cloud_top_m, freezing_level_m) {
            self.add(c);
        }
    }

    pub fn sample_at(&self, x_m: f64, z_m: f64, altitude_m: f64) -> CellInfluence {
        let mut in_cloud = false;
        let mut best: Option<&WeatherCell> = None;
        let mut best_effective = 0.0;
        for c in &self.cells {
            let w = c.horizontal_weight(x_m, z_m) * c.vertical_weight(altitude_m);
            if w <= 0.0 {
                continue;
            }
            in_cloud = true;
            let effective = c.intensity * w;
            if best.is_none() || effective > best_effective {
                best_effective = effective;
                best = Some(c);
            }
        }
        let Some(cell) = best else {
            return CellInfluence { convective_intensity: 0.0, hail_intensity: 0.0, ice_water_content_g_m3: 0.0, turbulence_intensity: None, in_cloud: false };
        };
        let convective_intensity = best_effective;

        let max_hail_mm = hail::max_sustainable_diameter_mm(cell.updraft_peak_ms());
        let hail_intensity = (convective_intensity * (max_hail_mm / 25.0)).clamp(0.0, 1.0);

        let frac_of_depth = ((altitude_m - cell.base_m) / (cell.top_m - cell.base_m).max(1.0)).clamp(0.0, 1.0);
        let upper_third_weight = ((frac_of_depth - 0.5) / 0.5).clamp(0.0, 1.0);
        let ice_water_content_g_m3 = convective_intensity * PEAK_IWC_G_M3 * upper_third_weight;

        let turbulence_intensity = if convective_intensity > 0.6 {
            Some(TurbulenceIntensity::Severe)
        } else if convective_intensity > 0.25 {
            Some(TurbulenceIntensity::Moderate)
        } else if convective_intensity > 0.0 {
            Some(TurbulenceIntensity::Light)
        } else {
            None
        };

        CellInfluence { convective_intensity, hail_intensity, ice_water_content_g_m3, turbulence_intensity, in_cloud }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_weather_gives_no_cell() {
        assert!(WeatherCell::from_xp_point_weather(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).is_none());
    }

    #[test]
    fn thunderstorm_fraction_dominates_over_plain_precipitation() {
        let rain = WeatherCell::from_xp_point_weather(0.0, 0.0, 1.0, 0.0, 0.0, 0.0).unwrap();
        let storm = WeatherCell::from_xp_point_weather(0.0, 0.0, 0.0, 1.0, 0.0, 0.0).unwrap();
        assert!(storm.intensity > rain.intensity);
    }

    #[test]
    fn far_from_any_cell_and_outside_its_vertical_extent_is_calm() {
        let mut field = WeatherCellField::new();
        field.add(WeatherCell { x_m: 0.0, z_m: 0.0, radius_m: 3000.0, base_m: 1000.0, top_m: 8000.0, intensity: 0.8 });
        let far = field.sample_at(50_000.0, 0.0, 4000.0);
        assert!(!far.in_cloud);
        assert_eq!(far.convective_intensity, 0.0);
        let too_low = field.sample_at(0.0, 0.0, 500.0);
        assert!(!too_low.in_cloud);
    }

    #[test]
    fn dead_centre_of_a_severe_cell_is_hazardous_on_every_axis() {
        let mut field = WeatherCellField::new();
        field.add(WeatherCell { x_m: 0.0, z_m: 0.0, radius_m: 5000.0, base_m: 1000.0, top_m: 12000.0, intensity: 0.9 });
        let mid_altitude = 1000.0 + (12000.0 - 1000.0) * 0.5;
        let out = field.sample_at(0.0, 0.0, mid_altitude);
        assert!(out.in_cloud);
        assert!(out.convective_intensity > 0.8);
        assert!(out.hail_intensity > 0.0);
        assert_eq!(out.turbulence_intensity, Some(TurbulenceIntensity::Severe));
    }

    #[test]
    fn ice_water_content_peaks_high_in_the_cell_not_near_the_base() {
        let mut field = WeatherCellField::new();
        field.add(WeatherCell { x_m: 0.0, z_m: 0.0, radius_m: 5000.0, base_m: 1000.0, top_m: 12000.0, intensity: 0.9 });
        let near_base = field.sample_at(0.0, 0.0, 1200.0);
        let near_top = field.sample_at(0.0, 0.0, 11000.0);
        assert_eq!(near_base.ice_water_content_g_m3, 0.0);
        assert!(near_top.ice_water_content_g_m3 > 0.0);
    }

    #[test]
    fn the_strongest_overlapping_cell_wins_rather_than_summing() {
        let mut field = WeatherCellField::new();
        field.add(WeatherCell { x_m: 0.0, z_m: 0.0, radius_m: 10_000.0, base_m: 0.0, top_m: 10_000.0, intensity: 0.3 });
        field.add(WeatherCell { x_m: 100.0, z_m: 0.0, radius_m: 10_000.0, base_m: 0.0, top_m: 10_000.0, intensity: 0.9 });
        let out = field.sample_at(0.0, 0.0, 5000.0);
        assert!(out.convective_intensity <= 0.9 + 1e-9);
        assert!(out.convective_intensity > 0.3);
    }

    #[test]
    fn set_from_xp_point_weather_replaces_rather_than_accumulates() {
        let mut field = WeatherCellField::new();
        field.set_from_xp_point_weather(0.0, 0.0, 0.5, 1.0, 9000.0, 4000.0);
        assert_eq!(field.cells.len(), 1);
        field.set_from_xp_point_weather(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        assert_eq!(field.cells.len(), 0);
    }

    #[test]
    fn no_nan_at_degenerate_inputs() {
        let cell = WeatherCell::from_xp_point_weather(0.0, 0.0, 1.0, 1.0, 0.0, 0.0).unwrap();
        assert!(!cell.radius_m.is_nan() && !cell.top_m.is_nan() && !cell.base_m.is_nan());
        let mut field = WeatherCellField::new();
        field.add(cell);
        let out = field.sample_at(0.0, 0.0, 0.0);
        assert!(!out.convective_intensity.is_nan() && !out.hail_intensity.is_nan() && !out.ice_water_content_g_m3.is_nan());
    }
}
