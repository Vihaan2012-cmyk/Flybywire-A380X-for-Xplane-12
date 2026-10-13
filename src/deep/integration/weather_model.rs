//! The host-neutral half of `weather_truth`: the environment every area
//! reads, and the pure physics that turns temperature and cloud type into
//! the icing models' liquid water content and droplet size. The X-Plane
//! reader that fills it stays in `weather_truth`; MSFS fills it from its
//! own simulator variables.

use crate::deep::weather::WeatherSample;

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

/// One weather cloud layer's type, `deep::weather::WeatherCloudLayer`'s
/// own documented enum (X-Plane's `XPLMWeatherInfoClouds_t`): 0 cirrus,
/// 1 stratus, 2 cumulus, 3 cumulonimbus.
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
