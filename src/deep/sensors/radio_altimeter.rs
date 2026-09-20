//! Radio altimeter: a low-power FM-CW radar that measures true height above
//! whatever is directly below the antennas (not barometric altitude). Models
//! antenna/transceiver faults, the well-known small fixed-offset false
//! reading some installations show at/near the ground, multipath-induced
//! noise, and the system's own range limit.
//!
//! ## The "-6 ft" style false reading
//! A radio altimeter measures the two-way time of flight to the nearest
//! reflecting surface along its beam. Antenna near-field effects, ground
//! clutter and installation geometry commonly leave a small residual
//! calibration offset even when the system is working exactly as designed;
//! this is why real installations quote (and cockpits sometimes display) a
//! reading of a few feet negative while parked on a hard, flat ramp, rather
//! than exactly zero -- a well-known, publicly documented characteristic of
//! radio altimeter installations (widely discussed in type-specific FCOM/
//! flight-test notes and pilot references on radio altimeter behaviour on
//! the ground). Modelled here as a fixed bias fault rather than a
//! computed effect, since the exact antenna near-field physics that
//! produces it is airframe/installation-specific and not published for the
//! A380.
//!
//! ## Multipath
//! Over water, snow, or near large flat reflective surfaces, secondary
//! reflected paths can arrive close enough in time/amplitude to the direct
//! path that the tracking loop's height estimate jitters -- a well
//! documented radar altimeter phenomenon (see e.g. FAA/EUROCAE radio
//! altimeter MOPS discussions of multipath susceptibility over water).
//! Modelled as additive noise whose severity depends on the terrain type and
//! is worse at low altitude (where the reflected geometry is closest to the
//! direct path in both time and angle) -- GENERIC functional form and
//! magnitude, since no public multipath error budget exists for this
//! specific system.
//!
//! ## Range limit
//! Radio altimeters are only valid over a bounded height range; above it
//! there is no usable return within their power budget/timing window, so
//! the system reports no computed data rather than an arbitrary large
//! number. 2500 ft is used here as a representative transport-category
//! radio altimeter upper range limit (a commonly cited figure for this
//! class of equipment in public radio-altimeter literature) -- GENERIC (not
//! verified against the A380's specific ALA-52B-family installation, which
//! `src/physics/adirs.rs`'s own radio altimeter probe notes is not public).

use super::rng::Rng;

/// Representative upper range limit, ft. GENERIC (see module docs).
pub const MAX_RANGE_FT: f64 = 2500.0;
/// Below this height, multipath is at its worst (closest reflected-path
/// geometry); above it, decays to a small residual. GENERIC.
const MULTIPATH_REFERENCE_HEIGHT_FT: f64 = 50.0;
/// Base 1-sigma noise at the reference height and severity 1.0, ft. GENERIC.
const MULTIPATH_BASE_SIGMA_FT: f64 = 1.5;
/// Multiplier applied over water/snow (specular reflectors) vs. varied
/// terrain (diffuse scattering breaks up the reflected path). GENERIC,
/// consistent with multipath being a specifically over-water/over-snow
/// concern in the public literature cited above.
const SPECULAR_TERRAIN_MULTIPLIER: f64 = 3.0;

/// A radio altimeter is three separately maintainable, separately
/// failable physical parts: the transceiver (the electronics box: RF
/// generation, receiver, tracking-loop signal processing) and its two
/// antennas (a radio altimeter transmits and receives on separate
/// antennas, standard practice to isolate the strong transmitted signal
/// from the much weaker return -- publicly documented FM-CW radar
/// altimeter installation practice). Any one of the three being fully
/// failed loses the height reading entirely (the signal path needs all
/// three), while a *degraded* (not fully failed) antenna -- corrosion,
/// paint buildup, minor impact damage reducing gain rather than an open
/// circuit -- instead just raises the noise floor, the same symptom
/// [`RadioAltimeterFaults::tracking_loop_degradation`] produces from a
/// different physical cause (a degraded receiver tracking-loop filter),
/// kept as a separate field so the catalogue can name the antenna
/// specifically as the failed part.
#[derive(Clone, Copy, Debug, Default)]
pub struct RadioAltimeterFaults {
    /// Transceiver electronics failure: `1.0` fully failed, no valid
    /// return at all.
    pub transceiver_fault: f64,
    /// Transmit antenna failure (corrosion, connector, physical damage):
    /// `1.0` fully failed, no signal transmitted.
    pub tx_antenna_fault: f64,
    /// Receive antenna failure, same causes/effect as the transmit antenna
    /// but the receive side.
    pub rx_antenna_fault: f64,
    /// Receive antenna *degradation* (reduced gain, not an open circuit):
    /// raises the effective noise floor the same way
    /// `tracking_loop_degradation` does, physically distinct cause.
    pub rx_antenna_degradation: f64,
    /// A fixed installation/near-field calibration offset, ft (see module
    /// docs) -- signed; a healthy but imperfectly compensated unit might
    /// carry a permanent small negative value.
    pub false_offset_ft: f64,
    /// Additional multipath susceptibility beyond the terrain-driven
    /// baseline from a degraded tracking-loop filter in the transceiver
    /// (see `rx_antenna_degradation` for the antenna-side equivalent):
    /// `0.0` = only the terrain-driven baseline applies.
    pub tracking_loop_degradation: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct RadioAltimeterOutput {
    pub agl_ft: f64,
    pub valid: bool,
}

pub struct RadioAltimeter {
    rng: Rng,
}

impl RadioAltimeter {
    pub fn new(seed: u64) -> Self {
        Self { rng: Rng::new(seed) }
    }

    /// `true_agl_ft`: the aircraft's true height above whatever is directly
    /// below it (terrain/water/structures). `over_water_or_snow`: whether
    /// the reflecting surface is the kind of specular reflector that makes
    /// multipath worse.
    pub fn step(&mut self, true_agl_ft: f64, over_water_or_snow: bool, faults: &RadioAltimeterFaults) -> RadioAltimeterOutput {
        let any_part_fully_failed = faults.transceiver_fault.clamp(0.0, 1.0) >= 0.98
            || faults.tx_antenna_fault.clamp(0.0, 1.0) >= 0.98
            || faults.rx_antenna_fault.clamp(0.0, 1.0) >= 0.98;
        if any_part_fully_failed {
            return RadioAltimeterOutput { agl_ft: 0.0, valid: false };
        }
        if !(0.0..=MAX_RANGE_FT).contains(&true_agl_ft) {
            // Below the ground (a placement glitch) or above range: no
            // computed data either way -- a real unit does not report a
            // clamped number outside its designed envelope.
            return RadioAltimeterOutput { agl_ft: 0.0, valid: false };
        }

        let terrain_multiplier = if over_water_or_snow { SPECULAR_TERRAIN_MULTIPLIER } else { 1.0 };
        let height_factor = (MULTIPATH_REFERENCE_HEIGHT_FT / true_agl_ft.max(1.0)).min(5.0);
        let severity = 1.0 + faults.tracking_loop_degradation.max(0.0) + faults.rx_antenna_degradation.max(0.0);
        let sigma_ft = MULTIPATH_BASE_SIGMA_FT * terrain_multiplier * height_factor * severity;
        let noise_ft = self.rng.gaussian() * sigma_ft;

        let agl_ft = (true_agl_ft + faults.false_offset_ft + noise_ft).max(-50.0);
        RadioAltimeterOutput { agl_ft, valid: true }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn healthy_unit_reads_close_to_true_agl_away_from_multipath_terrain() {
        let mut ra = RadioAltimeter::new(1);
        let out = ra.step(1000.0, false, &RadioAltimeterFaults::default());
        assert!(out.valid);
        assert!((out.agl_ft - 1000.0).abs() < 5.0, "{}", out.agl_ft);
    }

    #[test]
    fn above_range_is_invalid_not_clamped() {
        let mut ra = RadioAltimeter::new(1);
        let out = ra.step(MAX_RANGE_FT + 500.0, false, &RadioAltimeterFaults::default());
        assert!(!out.valid);
    }

    #[test]
    fn transceiver_fault_reports_invalid_even_at_a_normal_height() {
        let mut ra = RadioAltimeter::new(1);
        let faults = RadioAltimeterFaults { transceiver_fault: 1.0, ..Default::default() };
        let out = ra.step(500.0, false, &faults);
        assert!(!out.valid);
    }

    #[test]
    fn either_antenna_failing_reports_invalid_even_with_a_healthy_transceiver() {
        let mut tx_failed = RadioAltimeter::new(1);
        let tx_faults = RadioAltimeterFaults { tx_antenna_fault: 1.0, ..Default::default() };
        assert!(!tx_failed.step(500.0, false, &tx_faults).valid);

        let mut rx_failed = RadioAltimeter::new(1);
        let rx_faults = RadioAltimeterFaults { rx_antenna_fault: 1.0, ..Default::default() };
        assert!(!rx_failed.step(500.0, false, &rx_faults).valid);
    }

    #[test]
    fn degraded_receive_antenna_adds_noise_like_tracking_loop_degradation_but_stays_valid() {
        let mut healthy = RadioAltimeter::new(4);
        let mut degraded_antenna = RadioAltimeter::new(4);
        let faults = RadioAltimeterFaults { rx_antenna_degradation: 2.0, ..Default::default() };
        let n = 2000;
        let (mut healthy_sq, mut degraded_sq) = (0.0, 0.0);
        for _ in 0..n {
            let h = healthy.step(500.0, false, &RadioAltimeterFaults::default());
            let d = degraded_antenna.step(500.0, false, &faults);
            assert!(d.valid, "a merely degraded (not failed) antenna must still return valid data");
            healthy_sq += (h.agl_ft - 500.0).powi(2);
            degraded_sq += (d.agl_ft - 500.0).powi(2);
        }
        assert!(degraded_sq > healthy_sq, "healthy {healthy_sq} degraded {degraded_sq}");
    }

    #[test]
    fn false_offset_reproduces_a_small_negative_ground_reading() {
        let mut ra = RadioAltimeter::new(1);
        let faults = RadioAltimeterFaults { false_offset_ft: -6.0, ..Default::default() };
        // Average over several ticks to see past the multipath noise.
        let mut sum = 0.0;
        let n = 200;
        for _ in 0..n {
            sum += ra.step(0.0, false, &faults).agl_ft;
        }
        assert!((sum / n as f64 - (-6.0)).abs() < 2.0, "{}", sum / n as f64);
    }

    #[test]
    fn multipath_noise_is_worse_over_water_than_over_varied_terrain() {
        let mut over_land = RadioAltimeter::new(2);
        let mut over_water = RadioAltimeter::new(2);
        let n = 2000;
        let (mut land_sq, mut water_sq) = (0.0, 0.0);
        for _ in 0..n {
            let l = over_land.step(30.0, false, &RadioAltimeterFaults::default());
            let w = over_water.step(30.0, true, &RadioAltimeterFaults::default());
            land_sq += (l.agl_ft - 30.0).powi(2);
            water_sq += (w.agl_ft - 30.0).powi(2);
        }
        assert!(water_sq > land_sq, "land {land_sq} water {water_sq}");
    }

    #[test]
    fn multipath_noise_is_worse_close_to_the_ground_than_high_up() {
        let mut low = RadioAltimeter::new(3);
        let mut high = RadioAltimeter::new(3);
        let n = 2000;
        let (mut low_sq, mut high_sq) = (0.0, 0.0);
        for _ in 0..n {
            let l = low.step(20.0, false, &RadioAltimeterFaults::default());
            let h = high.step(2000.0, false, &RadioAltimeterFaults::default());
            low_sq += (l.agl_ft - 20.0).powi(2);
            high_sq += (h.agl_ft - 2000.0).powi(2);
        }
        assert!(low_sq > high_sq, "low {low_sq} high {high_sq}");
    }

    #[test]
    fn negative_agl_glitch_is_invalid() {
        let mut ra = RadioAltimeter::new(1);
        let out = ra.step(-5.0, false, &RadioAltimeterFaults::default());
        assert!(!out.valid);
    }

    #[test]
    fn no_nan_at_zero_height() {
        let mut ra = RadioAltimeter::new(1);
        let out = ra.step(0.0, false, &RadioAltimeterFaults::default());
        assert!(out.agl_ft.is_finite());
    }
}
