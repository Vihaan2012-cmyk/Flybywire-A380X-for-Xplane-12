//! GPS receiver: a geometry-free position-error model (no per-satellite
//! ephemeris/pseudorange simulation -- that is out of scope for a plugin
//! sensor model), plus loss of fix, jamming and a gradually-applied spoofing
//! offset.
//!
//! ## Geometry-free error model
//! A GPS fix's horizontal error is, to first order,
//! `sigma_position = HDOP * UERE` (User Equivalent Range Error): the
//! satellite geometry (Dilution of Precision) multiplies the per-satellite
//! ranging error into a position error. This is standard, publicly
//! documented GPS error-budget theory (Kaplan & Hegarty, eds.,
//! *Understanding GPS/GNSS: Principles and Applications*, ch. 7 "Precise
//! Positioning"; Misra & Enge, *Global Positioning System: Signals,
//! Measurements, and Performance*). `UERE` for modern civil GPS (post-2000,
//! Selective Availability off) is commonly quoted around 3-6 m (1-sigma);
//! 4 m is used here -- GENERIC mid-value, no satellite-specific almanac is
//! modelled. A full satellite-geometry computation (actual DOP from visible
//! satellites' azimuth/elevation) is out of scope; instead `hdop` is
//! obtained from a monotonic, clearly-marked GENERIC proxy in the number of
//! satellites visible (fewer satellites -> worse, geometry-independent,
//! geometry usually also degrades with fewer satellites in view) rather
//! than asserted as a real DOP computation.
//!
//! ## Minimum satellites for a fix
//! A GPS position fix needs at least 4 satellites (3 for x/y/z position
//! plus 1 to resolve the receiver clock bias) -- a basic, universally
//! published GPS requirement, not GENERIC.
//!
//! ## Jamming
//! RF jamming raises the noise floor at the receiver, reducing carrier-to-
//! noise density (C/N0) until weak satellites drop out of track first, and
//! eventually the whole receiver loses lock. Modelled as jamming strength
//! progressively reducing the *effective* number of usable satellites,
//! consistent with the widely documented weakest-signal-drops-first
//! behaviour of jamming, then losing the fix entirely above a threshold --
//! GENERIC mapping (no published jammer-power-to-satellite-dropout curve
//! for this receiver class is available), but the qualitative mechanism is
//! standard GNSS-interference behaviour.
//!
//! ## Spoofing
//! A naive spoofing attack that jumps the reported position instantly is
//! easily caught by a receiver autonomous integrity monitor (RAIM) comparing
//! the jump against the inertial/previous solution; real documented spoofing
//! incidents and academic spoofing literature describe a gradual "walk-off"
//! from the true position toward the false one instead, precisely to stay
//! under a plausible-rate-of-change detector (Humphreys et al., "Assessing
//! the Spoofing Threat", GPS World, 2009, describes exactly this
//! technique). Modelled here as the reported position ramping toward a
//! commanded spoof offset at a bounded rate rather than jumping.

use super::rng::Rng;

/// User Equivalent Range Error, 1-sigma, metres.
///
/// **Sourced**: the US Government's GPS Standard Positioning Service
/// Performance Standard commits to broadcasting the signal in space with a
/// global average user range error of "<=7.8 m ... with 95% probability",
/// stated in the same document as the signal-in-space range accuracy
/// commitment of **4 metres RMS** (GPS.gov, "GPS Accuracy", quoting the SPS
/// Performance Standard; the 4 m rms / 7.8 m 95% pair replaced an earlier
/// 6 m rms commitment). 4 m RMS about a zero mean is 4 m 1-sigma, which is
/// this constant.
///
/// Honest caveat, unchanged by the citation: 4 m RMS is the **signal in
/// space** URE alone. A receiver's total UERE also carries ionospheric and
/// tropospheric delay residuals, multipath and receiver noise, so using the
/// SIS figure as the whole UERE is the optimistic direction -- a real
/// airborne single-frequency UERE runs larger. It is used anyway because it
/// is the only part of the budget with a published, committed number; the
/// user-equipment contribution is exactly what would otherwise have to be
/// invented. Actual on-orbit performance is far better than the commitment
/// (GPS.gov reports global average URE under 1 m), so the committed figure
/// is a conservative stand-in for the whole budget rather than a bare
/// underestimate of it.
const UERE_M: f64 = 4.0;
/// Minimum satellites for any 3D+clock fix. Not GENERIC -- a basic GPS
/// requirement.
const MIN_SATELLITES_FOR_FIX: u32 = 4;
/// HDOP proxy reference: at this many satellites visible, HDOP is
/// approximately its typical open-sky value.
///
/// **Now derived rather than picked.** 8 is the mean number of satellites a
/// 5-degree-mask receiver sees from the GPS *baseline* constellation -- the
/// 24 slots the SPS Performance Standard is written against, of which the
/// spherical-cap visibility fraction above a 5-degree mask is 0.338568, so
/// `24 * 0.338568 = 8.13`. The full geometry, with all its inputs and their
/// sources, is written out at `sensors::live`'s
/// `NOMINAL_SATELLITES_VISIBLE`, which applies the identical calculation to
/// the 31-satellite constellation actually flown and gets 10. The two
/// constants are therefore the same derivation at the standard's
/// constellation and at the real one, which is why a receiver at the
/// nominal 10 satellites sits a little *better* than this reference HDOP:
/// that is the real, and correct, benefit of the over-populated
/// constellation.
///
/// Still an approximation in what it does with that number: mapping
/// satellite count to HDOP is a monotonic proxy, not a geometry
/// computation (no almanac, no azimuth/elevation), so which satellites are
/// lost does not matter here, only how many. See the module docs.
const REFERENCE_SATELLITES: f64 = 8.0;
const REFERENCE_HDOP: f64 = 1.2;
/// Maximum rate a spoofing "walk-off" can move the reported position without
/// triggering a plausibility/RAIM-style rejection elsewhere downstream,
/// m/s. GENERIC (chosen well below a physically implausible aircraft
/// acceleration, consistent with the walk-off literature's premise).
const MAX_SPOOF_WALKOFF_MS: f64 = 1.0;

/// A GPS installation is (at least) two separately failable physical parts:
/// the receiver electronics and its antenna. Either fully failing loses the
/// fix entirely (the signal path needs both); a *degraded* (not failed)
/// antenna -- corroded connector, cracked radome letting water in, a
/// partially delaminated patch element -- instead just attenuates the
/// received signal, which presents to the receiver exactly like additional
/// RF noise: modelled by feeding `antenna_degradation` into the same
/// effective-satellite-count reduction [`GpsFaults::jamming`] uses, a
/// physically distinct cause (passive antenna gain loss vs. active RF
/// interference) producing the same weakest-satellite-drops-first symptom.
#[derive(Clone, Copy, Debug, Default)]
pub struct GpsFaults {
    /// Receiver electronics/processing hardware failure: `1.0` no fix at
    /// all.
    pub receiver_fault: f64,
    /// Antenna failure (open circuit, severed cable, sheared-off radome):
    /// `1.0` no fix at all -- same effect as `receiver_fault`, distinct
    /// physical part.
    pub antenna_fault: f64,
    /// Antenna gain loss short of outright failure: acts like additional
    /// jamming on the effective satellite count (see module docs), `0.0`
    /// none .. `1.0` as severe as full jamming.
    pub antenna_degradation: f64,
    /// RF jamming strength, `0.0` none .. `1.0` receiver fully jammed
    /// (no usable satellites).
    pub jamming: f64,
    /// The commanded spoof target, as a north/east offset from the true
    /// position, metres. The receiver ramps toward this at
    /// [`MAX_SPOOF_WALKOFF_MS`] rather than jumping to it; `[0.0, 0.0]`
    /// (the default) means "not being spoofed".
    pub spoof_target_offset_m: [f64; 2],
}

#[derive(Clone, Copy, Debug, Default)]
pub struct GpsOutput {
    /// 1-sigma horizontal position error this tick's noise draw represents,
    /// m (diagnostic/Study use).
    pub position_error_1sigma_m: f64,
    /// The spoofing offset actually being applied right now (after
    /// ramping), north/east, m.
    pub applied_spoof_offset_m: [f64; 2],
    /// North/east noise+spoof offset to add to the true position, m.
    pub position_offset_m: [f64; 2],
    pub valid: bool,
    pub effective_satellites: f64,
}

pub struct GpsReceiver {
    rng: Rng,
    applied_spoof_offset_m: [f64; 2],
}

impl GpsReceiver {
    pub fn new(seed: u64) -> Self {
        Self { rng: Rng::new(seed), applied_spoof_offset_m: [0.0, 0.0] }
    }

    pub fn step(&mut self, satellites_visible: u32, faults: &GpsFaults, dt_s: f64) -> GpsOutput {
        let dt = dt_s.max(0.0);
        if faults.receiver_fault.clamp(0.0, 1.0) >= 0.98 || faults.antenna_fault.clamp(0.0, 1.0) >= 0.98 {
            return GpsOutput { valid: false, ..Default::default() };
        }

        // Jamming and antenna gain loss both eat into the effective
        // satellite count (weakest signals drop first) before either eats
        // into noise on the remainder -- same symptom, two distinct
        // physical causes (see module docs).
        let jam = (faults.jamming.clamp(0.0, 1.0) + faults.antenna_degradation.max(0.0)).min(1.0);
        let effective_satellites = (satellites_visible as f64 * (1.0 - jam)).max(0.0);

        if effective_satellites < MIN_SATELLITES_FOR_FIX as f64 {
            return GpsOutput { effective_satellites, valid: false, ..Default::default() };
        }

        let hdop = REFERENCE_HDOP * (REFERENCE_SATELLITES / effective_satellites.max(1.0));
        let sigma_m = hdop * UERE_M;
        let noise_n = self.rng.gaussian() * sigma_m / std::f64::consts::SQRT_2;
        let noise_e = self.rng.gaussian() * sigma_m / std::f64::consts::SQRT_2;

        // Spoof walk-off: move the applied offset toward the commanded
        // target at a bounded rate per axis.
        for i in 0..2 {
            let target = faults.spoof_target_offset_m[i];
            let delta = target - self.applied_spoof_offset_m[i];
            let max_step = MAX_SPOOF_WALKOFF_MS * dt;
            self.applied_spoof_offset_m[i] += delta.clamp(-max_step, max_step);
        }

        GpsOutput {
            position_error_1sigma_m: sigma_m,
            applied_spoof_offset_m: self.applied_spoof_offset_m,
            position_offset_m: [noise_n + self.applied_spoof_offset_m[0], noise_e + self.applied_spoof_offset_m[1]],
            valid: true,
            effective_satellites,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn more_satellites_gives_lower_position_error() {
        let mut few = GpsReceiver::new(1);
        let mut many = GpsReceiver::new(1);
        let few_out = few.step(5, &GpsFaults::default(), 1.0);
        let many_out = many.step(12, &GpsFaults::default(), 1.0);
        assert!(few_out.valid && many_out.valid);
        assert!(few_out.position_error_1sigma_m > many_out.position_error_1sigma_m);
    }

    #[test]
    fn fewer_than_four_satellites_has_no_fix() {
        let mut gps = GpsReceiver::new(1);
        let out = gps.step(3, &GpsFaults::default(), 1.0);
        assert!(!out.valid);
    }

    #[test]
    fn heavy_jamming_reduces_effective_satellites_below_the_fix_minimum() {
        let mut gps = GpsReceiver::new(1);
        let faults = GpsFaults { jamming: 0.9, ..Default::default() };
        let out = gps.step(8, &faults, 1.0);
        assert!(out.effective_satellites < MIN_SATELLITES_FOR_FIX as f64);
        assert!(!out.valid);
    }

    #[test]
    fn light_jamming_still_gives_a_fix_with_more_noise() {
        let mut clean = GpsReceiver::new(2);
        let mut jammed = GpsReceiver::new(2);
        let clean_out = clean.step(10, &GpsFaults::default(), 1.0);
        let jammed_out = jammed.step(10, &GpsFaults { jamming: 0.4, ..Default::default() }, 1.0);
        assert!(jammed_out.valid);
        assert!(jammed_out.position_error_1sigma_m > clean_out.position_error_1sigma_m);
    }

    #[test]
    fn receiver_fault_gives_no_fix_regardless_of_satellites() {
        let mut gps = GpsReceiver::new(1);
        let faults = GpsFaults { receiver_fault: 1.0, ..Default::default() };
        let out = gps.step(12, &faults, 1.0);
        assert!(!out.valid);
    }

    #[test]
    fn antenna_fault_gives_no_fix_regardless_of_satellites() {
        let mut gps = GpsReceiver::new(1);
        let faults = GpsFaults { antenna_fault: 1.0, ..Default::default() };
        let out = gps.step(12, &faults, 1.0);
        assert!(!out.valid);
    }

    #[test]
    fn degraded_antenna_acts_like_jamming_on_the_effective_satellite_count() {
        let mut healthy = GpsReceiver::new(1);
        let mut degraded = GpsReceiver::new(1);
        let faults = GpsFaults { antenna_degradation: 0.5, ..Default::default() };
        let healthy_out = healthy.step(10, &GpsFaults::default(), 1.0);
        let degraded_out = degraded.step(10, &faults, 1.0);
        assert!(degraded_out.valid);
        assert!(degraded_out.effective_satellites < healthy_out.effective_satellites);
        assert!(degraded_out.position_error_1sigma_m > healthy_out.position_error_1sigma_m);
    }

    #[test]
    fn spoofing_ramps_gradually_rather_than_jumping() {
        let mut gps = GpsReceiver::new(1);
        let faults = GpsFaults { spoof_target_offset_m: [500.0, 0.0], ..Default::default() };
        let out = gps.step(10, &faults, 1.0);
        // One second at the bounded walk-off rate cannot reach 500 m.
        assert!(out.applied_spoof_offset_m[0] < 5.0, "{}", out.applied_spoof_offset_m[0]);
        assert!(out.applied_spoof_offset_m[0] > 0.0);
    }

    #[test]
    fn spoofing_eventually_reaches_its_target_given_enough_time() {
        let mut gps = GpsReceiver::new(1);
        let faults = GpsFaults { spoof_target_offset_m: [50.0, -20.0], ..Default::default() };
        let mut out = GpsOutput::default();
        for _ in 0..1000 {
            out = gps.step(10, &faults, 1.0);
        }
        assert!((out.applied_spoof_offset_m[0] - 50.0).abs() < 1.0);
        assert!((out.applied_spoof_offset_m[1] - (-20.0)).abs() < 1.0);
    }

    #[test]
    fn no_nan_at_zero_dt_or_zero_satellites() {
        let mut gps = GpsReceiver::new(1);
        let out = gps.step(0, &GpsFaults::default(), 0.0);
        assert!(!out.valid);
        assert!(out.effective_satellites.is_finite());
    }
}
