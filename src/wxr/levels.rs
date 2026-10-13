//! The four levels a real Airbus weather radar paints (green, amber, red for
//! rising return strength, magenta for the most hazardous cells -- on the
//! real aircraft, usually convective cores strong enough to also trigger the
//! turbulence mode). That four-level convention is real; the thresholds
//! below are not, because X-Plane's weather has nothing to threshold against
//! a real one with. `XPLMWeatherInfo_t.precip_rate_alt` (XPLMWeather.h,
//! src/xp.rs "[wxr]") is a 0..1 ratio with no documented mapping to
//! reflectivity (dBZ), so this module just divides that ratio into four
//! bands. `docs/wxr.md` says so; nothing here claims to be FlyByWire's own
//! figure.

/// One radar return level, most severe last.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Default)]
pub enum Level {
    #[default]
    None,
    Green,
    Amber,
    Red,
    Magenta,
}

/// Below this ratio, X-Plane's weather has nothing worth a return.
pub const PRECIP_THRESHOLD: f32 = 0.05;
pub const AMBER_THRESHOLD: f32 = 0.30;
pub const RED_THRESHOLD: f32 = 0.60;
pub const MAGENTA_THRESHOLD: f32 = 0.85;
/// From [`RED_THRESHOLD`], turbulence at least this strong (`turbulence_alt`,
/// also 0..1, XPLMWeather.h) paints magenta early: a real WXR's "turbulence
/// mode" folded in, since X-Plane gives no separate mode to switch to.
pub const MAGENTA_TURBULENCE: f32 = 0.5;

/// A cell's level from what X-Plane's weather gives at the sampled point:
/// the precipitation ratio there and the turbulence ratio there.
pub fn classify(precip_rate_alt: f32, turbulence_alt: f32) -> Level {
    if precip_rate_alt < PRECIP_THRESHOLD {
        Level::None
    } else if precip_rate_alt >= MAGENTA_THRESHOLD || (precip_rate_alt >= RED_THRESHOLD && turbulence_alt >= MAGENTA_TURBULENCE) {
        Level::Magenta
    } else if precip_rate_alt >= RED_THRESHOLD {
        Level::Red
    } else if precip_rate_alt >= AMBER_THRESHOLD {
        Level::Amber
    } else {
        Level::Green
    }
}

impl Level {
    /// Straight RGBA (not premultiplied), X-Plane's `nd.js` amber
    /// (`#ffff00`, `VerticalDisplay.tsx:673`) and the industry-standard WXR
    /// palette otherwise (pure green/red/magenta returns).
    pub fn rgba(self) -> [u8; 4] {
        match self {
            Level::None => [0, 0, 0, 0],
            Level::Green => [0, 255, 0, 255],
            Level::Amber => [255, 255, 0, 255],
            Level::Red => [255, 0, 0, 255],
            Level::Magenta => [255, 0, 255, 255],
        }
    }

    pub fn is_none(self) -> bool {
        self == Level::None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn below_threshold_is_no_return() {
        assert_eq!(classify(0.0, 0.0), Level::None);
        assert_eq!(classify(PRECIP_THRESHOLD - 0.01, 1.0), Level::None);
    }

    #[test]
    fn bands_step_up_with_precipitation() {
        assert_eq!(classify(PRECIP_THRESHOLD, 0.0), Level::Green);
        assert_eq!(classify(AMBER_THRESHOLD - 0.01, 0.0), Level::Green);
        assert_eq!(classify(AMBER_THRESHOLD, 0.0), Level::Amber);
        assert_eq!(classify(RED_THRESHOLD - 0.01, 0.0), Level::Amber);
        assert_eq!(classify(RED_THRESHOLD, 0.0), Level::Red);
        assert_eq!(classify(MAGENTA_THRESHOLD - 0.01, 0.0), Level::Red);
        assert_eq!(classify(MAGENTA_THRESHOLD, 0.0), Level::Magenta);
    }

    #[test]
    fn strong_turbulence_promotes_a_red_cell_to_magenta() {
        assert_eq!(classify(RED_THRESHOLD, MAGENTA_TURBULENCE - 0.01), Level::Red);
        assert_eq!(classify(RED_THRESHOLD, MAGENTA_TURBULENCE), Level::Magenta);
        // Below red, turbulence alone never promotes a cell.
        assert_eq!(classify(AMBER_THRESHOLD, 1.0), Level::Amber);
    }

    #[test]
    fn levels_order_by_severity() {
        assert!(Level::None < Level::Green);
        assert!(Level::Green < Level::Amber);
        assert!(Level::Amber < Level::Red);
        assert!(Level::Red < Level::Magenta);
    }

    #[test]
    fn none_is_transparent() {
        assert_eq!(Level::None.rgba()[3], 0);
        for l in [Level::Green, Level::Amber, Level::Red, Level::Magenta] {
            assert_eq!(l.rgba()[3], 255);
        }
    }
}
