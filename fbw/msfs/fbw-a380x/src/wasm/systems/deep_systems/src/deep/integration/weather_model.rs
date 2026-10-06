use crate::deep::weather::WeatherSample;

pub const KT_TO_MS: f64 = 0.514_444;
const GAMMA_AIR: f64 = 1.4;
const R_AIR_J_KGK: f64 = 287.052_87;

#[derive(Clone, Copy, Debug, Default)]
pub struct EnvironmentTruth {
    pub sat_c: f64,
    pub leading_edge_c: f64,
    pub ambient_pressure_pa: f64,
    pub tas_ms: f64,
    pub precipitation_on_aircraft_ratio: f64,
    pub weather: Option<WeatherSample>,
}

impl EnvironmentTruth {
    pub fn mach(&self) -> f64 {
        mach_from_tas_sat(self.tas_ms, self.sat_c)
    }
}

pub fn mach_from_tas_sat(tas_ms: f64, sat_c: f64) -> f64 {
    let t_k = (sat_c + 273.15).max(1.0);
    let speed_of_sound_ms = (GAMMA_AIR * R_AIR_J_KGK * t_k).sqrt();
    (tas_ms.max(0.0) / speed_of_sound_ms).max(0.0)
}

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

pub fn dominant_cloud(weather: &WeatherSample) -> Option<(CloudKind, f32)> {
    weather
        .clouds
        .iter()
        .filter(|c| c.coverage > 0.0)
        .max_by(|a, b| a.coverage.partial_cmp(&b.coverage).unwrap_or(std::cmp::Ordering::Equal).then((a.cloud_type as i32).cmp(&(b.cloud_type as i32))))
        .map(|c| (cloud_kind(c.cloud_type), c.coverage))
}

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

pub fn droplet_diameter_m_from_conditions(cloud: Option<(CloudKind, f32)>) -> f64 {
    match cloud.map(|(k, _)| k) {
        Some(CloudKind::Cumulus) | Some(CloudKind::Cumulonimbus) => 28e-6,
        Some(CloudKind::Stratus) => 18e-6,
        _ => 0.0,
    }
}

pub fn convective_intensity(weather: &WeatherSample, cloud: Option<(CloudKind, f32)>) -> f64 {
    let Some((CloudKind::Cumulonimbus, coverage)) = cloud else { return 0.0 };
    let cell = (coverage as f64).clamp(0.0, 1.0);
    let vigour = (weather.turbulence_alt as f64).clamp(0.0, 1.0).max((weather.precip_rate_alt as f64).clamp(0.0, 1.0));
    cell * vigour
}

pub fn hail_intensity(weather: &WeatherSample, cloud: Option<(CloudKind, f32)>) -> f64 {
    convective_intensity(weather, cloud)
}
