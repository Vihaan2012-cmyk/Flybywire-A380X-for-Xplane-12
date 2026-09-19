//! Convective cells: position, horizontal extent, base/top height,
//! updraft/downdraft structure and an intensity proxy, from which this
//! directory's other models' weather inputs (`convective_intensity` for
//! `lightning.rs`/`wind_shear.rs`, `hail_intensity` for `hail.rs`,
//! `ice_water_content_g_m3` for `ice_crystal_icing.rs`, and a turbulence
//! intensity for `wind_shear::TurbulenceModel`) are all derived from one
//! consistent picture of "where the storms are" instead of being set
//! independently by whatever calls each model.
//!
//! ## Feeding this from X-Plane (for `src\deep\integration\`)
//! X-Plane 12's weather API (`XPLMGetWeatherAtLocation`/the
//! `XPLMWeatherInfo_t` it fills) reports *local point* weather -- a
//! precipitation rate and a thunderstorm/convection fraction (both 0..1)
//! and cloud-layer coverage/base/top at the queried location -- not a
//! catalogue of discrete storm cells with their own geometry the way a
//! real radar mosaic would. [`WeatherCell::from_xp_point_weather`] is the
//! concrete recipe for that: call it every so often with the aircraft's
//! own position (X-Plane's `sim/flightmodel/position/local_x` and
//! `local_z`, its native local/OpenGL coordinate system, already in
//! metres, needs no separate geodesy conversion) and the point-weather
//! fields there, and it synthesises a single convective cell centred on
//! the aircraft when convection is present. A more capable integration
//! (sampling several nearby points, or a future real radar-mosaic feed)
//! can instead build a richer [`WeatherCellField`] directly with
//! [`WeatherCellField::add`].
//!
//! ## Sources
//! - Cell horizontal scale (a few to ~15 km diameter) and storm-top
//!   heights (weak convection a few km, severe storms reaching the
//!   tropopause, roughly 12-15 km) are standard, widely published
//!   meteorology (e.g. AMS Glossary of Meteorology entries for
//!   "thunderstorm"/"cumulonimbus").
//! - Radar-reflectivity intensity categories: the U.S. NWS's historical
//!   VIP (Video Integrator Processor) scale, widely reproduced in aviation
//!   weather-radar training material: VIP1 (18-30 dBZ) weak, VIP2 (30-38)
//!   moderate, VIP3 (38-44) strong, VIP4 (44-50) very strong, VIP5
//!   (50-57) intense, VIP6 (>57) extreme; used qualitatively to motivate
//!   mapping a single 0..1 `intensity` proxy onto storm severity, since
//!   X-Plane does not expose reflectivity directly.
//! - Parcel-theory updraft scale `w_max ~ sqrt(2 CAPE)` (textbook
//!   convective meteorology) is cited for context in `updraft_peak_ms`'s
//!   doc, but X-Plane exposes no CAPE, so the actual number used is a
//!   `GENERIC` direct mapping from the 0..1 intensity proxy.
//! - Hail growth requiring an updraft strong enough to suspend the
//!   growing stone is standard convective-storm meteorology (the
//!   qualitative basis of `hail::max_sustainable_diameter_mm`, reused
//!   here).
//! - Ice water content peaking in a deep convective cell's upper
//!   third/anvil region matches the HAIC/HIWC field-campaign finding
//!   already cited in `ice_crystal_icing.rs`.
//! - All specific numeric mappings (cell radius/top vs. intensity,
//!   updraft/downdraft peaks, the horizontal/vertical falloff shape, the
//!   ice-water-content peak value) are `GENERIC`, derived as documented at
//!   each constant.

use super::hail;
use super::wind_shear::TurbulenceIntensity;

/// A single convective cell.
#[derive(Clone, Copy, Debug)]
pub struct WeatherCell {
    /// Local horizontal position, m (matching X-Plane's own `local_x`/
    /// `local_z`, see module doc -- any consistent flat-earth frame works
    /// as long as the aircraft position passed to `sample_at` uses the
    /// same one).
    pub x_m: f64,
    pub z_m: f64,
    /// Horizontal radius, m.
    pub radius_m: f64,
    /// Base and top height, m, in whatever vertical reference the caller
    /// keeps consistent between a cell and the aircraft altitude passed to
    /// `sample_at` (MSL is the natural choice: real storms are structured
    /// relative to sea level/pressure levels, not local terrain).
    pub base_m: f64,
    pub top_m: f64,
    /// 0 (no convection) .. 1 (extreme, VIP6-class) intensity proxy (see
    /// module doc).
    pub intensity: f64,
}

/// GENERIC: cell radius grows from a small shower to a severe multicell
/// core across the intensity range, m.
const MIN_RADIUS_M: f64 = 1500.0;
const MAX_RADIUS_EXTRA_M: f64 = 4500.0;
/// GENERIC: storm top from a shallow shower to a severe storm reaching
/// the tropopause, m.
const MIN_TOP_M: f64 = 4000.0;
const MAX_TOP_EXTRA_M: f64 = 11000.0;
/// GENERIC: cell base as a fraction of the freezing level (storms build a
/// cloud base well below the freezing level; this is a coarse proxy, not
/// a lifted-condensation-level calculation), floored and capped so a
/// degenerate/missing freezing-level input stays sane.
const BASE_FREEZING_LEVEL_FRACTION: f64 = 0.3;
const MIN_BASE_M: f64 = 500.0;

impl WeatherCell {
    /// The recipe from this module's doc: build one convective cell
    /// centred at `(x_m, z_m)` from X-Plane's own per-point weather
    /// fields. `precip_rate_frac`/`thunderstorm_frac` are 0..1 as
    /// `XPLMWeatherInfo_t` reports them; `cloud_top_m`/`freezing_level_m`
    /// are whatever X-Plane gives (pass 0.0 if unknown, and this falls
    /// back to the GENERIC intensity-scaled defaults). Returns `None`
    /// when there is nothing convective to model (keeps a
    /// `WeatherCellField` from accumulating calm-air cells every frame).
    pub fn from_xp_point_weather(x_m: f64, z_m: f64, precip_rate_frac: f64, thunderstorm_frac: f64, cloud_top_m: f64, freezing_level_m: f64) -> Option<WeatherCell> {
        // GENERIC blend: thunderstorm fraction dominates (it is X-Plane's
        // own direct signal for convection), precipitation rate alone
        // (e.g. steady stratiform rain) still contributes a little since
        // even non-convective precipitation shares this module's
        // hail/icing/turbulence machinery at a low intensity.
        let intensity = (0.3 * precip_rate_frac.clamp(0.0, 1.0) + 0.7 * thunderstorm_frac.clamp(0.0, 1.0)).clamp(0.0, 1.0);
        if intensity <= 0.0 {
            return None;
        }
        let radius_m = MIN_RADIUS_M + MAX_RADIUS_EXTRA_M * intensity;
        let top_m = if cloud_top_m > 0.0 { cloud_top_m } else { MIN_TOP_M + MAX_TOP_EXTRA_M * intensity };
        let base_m = (freezing_level_m * BASE_FREEZING_LEVEL_FRACTION).max(MIN_BASE_M).min(top_m * 0.5);
        Some(WeatherCell { x_m, z_m, radius_m, base_m, top_m, intensity })
    }

    /// GENERIC peak updraft, m/s: loosely consistent with parcel theory's
    /// `w_max ~ sqrt(2 CAPE)` (a 30 m/s updraft corresponds to roughly
    /// 450 J/kg CAPE) but driven directly by the 0..1 intensity proxy
    /// since X-Plane exposes no CAPE (module doc). 5 m/s at `intensity=0`
    /// (ordinary cumulus), 35 m/s at `intensity=1` (severe supercell
    /// range).
    pub fn updraft_peak_ms(&self) -> f64 {
        5.0 + 30.0 * self.intensity.clamp(0.0, 1.0)
    }

    /// GENERIC: microburst/gust-front downdrafts are typically weaker
    /// than the core updraft that built the cell.
    pub fn downdraft_peak_ms(&self) -> f64 {
        0.4 * self.updraft_peak_ms()
    }

    fn horizontal_weight(&self, x_m: f64, z_m: f64) -> f64 {
        let d = ((x_m - self.x_m).powi(2) + (z_m - self.z_m).powi(2)).sqrt();
        (1.0 - d / self.radius_m.max(1.0)).clamp(0.0, 1.0)
    }

    /// GENERIC smoothed vertical presence: zero outside `[base_m, top_m]`,
    /// ramping to full weight over the outer 20% of the cell's depth at
    /// each end so a moving aircraft sees a gradual entry/exit rather than
    /// a hard step.
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

/// The combined derived influence of the nearest/strongest cell at one
/// point, ready to feed straight into this directory's other `step`
/// calls.
#[derive(Clone, Copy, Debug)]
pub struct CellInfluence {
    /// Feeds `lightning::LightningModel::step`'s and
    /// `wind_shear::WindShearModel::step`'s own `convective_intensity`
    /// parameter directly.
    pub convective_intensity: f64,
    /// Feeds `hail::HailModel::step`'s `hail_intensity`.
    pub hail_intensity: f64,
    /// Feeds `ice_crystal_icing::IceCrystalInputs::ice_water_content_g_m3`.
    pub ice_water_content_g_m3: f64,
    /// Feeds `wind_shear::TurbulenceModel::step`'s intensity; `None` means
    /// no convective contribution here (this module does not model
    /// ordinary clear-air turbulence).
    pub turbulence_intensity: Option<TurbulenceIntensity>,
    /// Whether the point is inside any cell's horizontal+vertical extent
    /// at all (a convenience for callers gating rain/visibility effects).
    pub in_cloud: bool,
}

/// GENERIC: ice water content peak, g/m^3, in the cell's upper-third/
/// anvil-outflow region at `intensity = 1.0` -- the same order of
/// magnitude the HAIC/HIWC campaigns found in deep convective cores (see
/// `ice_crystal_icing.rs`'s module doc).
const PEAK_IWC_G_M3: f64 = 6.0;

/// A set of active cells (typically nearby storms; the integration layer
/// decides how many to track).
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

    /// Replaces the field with a single cell built straight from
    /// X-Plane's per-point weather at the aircraft's own position (see
    /// this module's doc); a no-op leaving the field empty when there is
    /// nothing convective there.
    pub fn set_from_xp_point_weather(&mut self, aircraft_x_m: f64, aircraft_z_m: f64, precip_rate_frac: f64, thunderstorm_frac: f64, cloud_top_m: f64, freezing_level_m: f64) {
        self.clear();
        if let Some(c) = WeatherCell::from_xp_point_weather(aircraft_x_m, aircraft_z_m, precip_rate_frac, thunderstorm_frac, cloud_top_m, freezing_level_m) {
            self.add(c);
        }
    }

    /// The combined influence at `(x_m, z_m, altitude_m)`: among every
    /// cell that reaches this point at all, the one with the highest
    /// *effective* (weight-scaled) intensity dominates -- two overlapping
    /// storms do not make "twice the thunderstorm", and a strong storm a
    /// little further off outweighs a weak one dead centre.
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

        // Hail needs an updraft strong enough to suspend a growing stone;
        // compare what this cell's updraft can sustain against the NWS
        // "large hail" scale (25 mm) to gate the generic intensity proxy.
        let max_hail_mm = hail::max_sustainable_diameter_mm(cell.updraft_peak_ms());
        let hail_intensity = (convective_intensity * (max_hail_mm / 25.0)).clamp(0.0, 1.0);

        // Ice water content peaks in the upper third of the cell (the
        // anvil/outflow region), zero below the mid-point.
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
