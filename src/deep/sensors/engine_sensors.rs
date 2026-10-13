//! Engine-mounted sensors feeding the EEC (FADEC): shaft speed pickups
//! (N1/N2/N3), the TGT thermocouple harness, vibration pickups, and the
//! fuel flow transmitter. Each models one physically distinct, publicly
//! documented sensor technology and its real failure mechanisms, not a
//! generic stand-in.
//!
//! ## Speed pickups (N1/N2/N3)
//! A variable-reluctance (VR) speed probe sits near a toothed phonic wheel
//! on the shaft; each tooth passing the probe perturbs a magnetic circuit
//! and induces a voltage pulse (Faraday's law: induced EMF is proportional
//! to the *rate of change* of flux, so for a fixed tooth geometry the
//! induced pulse amplitude is proportional to shaft speed) at a frequency
//! proportional to shaft speed -- this is basic, textbook electromagnetic
//! sensor operation, not proprietary, and is the standard principle behind
//! every VR speed sensor used across gas-turbine and automotive industries.
//! Two real, physically distinct failure modes follow directly from this:
//! - **increased air gap** (loose mounting, vibration) attenuates the
//!   induced amplitude (flux linkage falls off sharply with gap for a
//!   reluctance sensor); since amplitude is *also* proportional to speed,
//!   a widened gap raises the *minimum speed* at which the EEC's amplitude
//!   threshold can still detect a pulse at all -- a real, commonly
//!   documented VR-sensor characteristic (loss of signal at low
//!   speed/cranking with a marginal air gap), not a scripted symptom;
//! - **open circuit / broken wire** removes the signal entirely at any
//!   speed.
//! Each EEC channel (A/B) has its own independent pickup and wiring run
//! (the standard dual-lane FADEC redundancy architecture), so this module
//! models one pickup per channel.
//!
//! ## TGT thermocouple harness
//! Turbine gas temperature is measured by several thermocouple junctions
//! spaced around the turbine annulus and electrically averaged, both to get
//! a representative reading of a temperature field that varies
//! circumferentially and to tolerate individual junction failures --
//! standard, publicly described gas-turbine instrumentation practice. This
//! module averages `junction_count` junctions, each independently able to
//! go open-circuit (removed from the average, biasing it toward whichever
//! junctions remain -- a hot streak or cold streak the healthy average was
//! smoothing out) or drift (a wiring/junction degradation adding a signed
//! offset before averaging).
//!
//! ## Vibration pickups
//! An accelerometer bolted to the engine case reports vibration amplitude
//! (conventionally inches/second or mils, generic units here). Modelled
//! fault modes: a fixed bias, a stuck reading, and *intermittent dropout*
//! (a loose connector momentarily losing signal, common for
//! vibration-monitoring wiring subject to continuous engine-case
//! vibration) -- distinct from a clean bias or a permanent stuck failure.
//!
//! ## Fuel flow transmitter
//! A turbine flowmeter: fuel spins a small rotor at a speed proportional to
//! volumetric flow rate, and a pickup counts rotor blade passes, giving a
//! pulse frequency `f = K*Q` (`K`, the meter's own "K-factor", in pulses per
//! unit volume) -- standard, publicly documented turbine-flowmeter
//! principle (used across the flow-measurement industry, not proprietary).
//! Two distinct real failure modes: **bearing wear/drag** slows the rotor
//! below the speed the true flow would spin it at, lowering the effective
//! K-factor (the meter under-reads); **debris partial blockage** reduces
//! the flow actually reaching the rotor (the engine still gets fuel through
//! a bypass/orifice path, but the *meter* sees less than the engine
//! actually burns) -- a different physical cause with the same
//! under-reading direction but a distinguishable magnitude relationship
//! (blockage acts on flow before the meter, wear acts on the meter's own
//! calibration), plus a **stuck rotor** (frozen, zero/fixed output).

use super::rng::Rng;

// ---------------------------------------------------------------------
// Speed pickup.
// ---------------------------------------------------------------------

/// Reference (small) air gap induced-voltage amplitude, volts, at 100 %
/// shaft speed. GENERIC: VR speed sensor output amplitudes for aerospace
/// applications are commonly in the low-single-digit-volts range at rated
/// speed; not sourced to this specific probe.
const REFERENCE_AMPLITUDE_V: f64 = 3.0;
/// The EEC's own minimum detectable pulse amplitude (below this, the
/// channel cannot reliably discriminate pulses from noise). GENERIC.
const DETECTION_FLOOR_V: f64 = 0.15;
/// How fast amplitude falls off with air-gap increase: reluctance-sensor
/// flux linkage falls off roughly with the square of the gap for a simple
/// magnetic circuit (inverse-square-law reasoning for the gap's magnetic
/// reluctance); modelled here as `amplitude ~ 1/(1+k*gap_increase)^2`,
/// `k` GENERIC (chosen so a fully "maximum modelled" gap increase drops
/// amplitude by roughly an order of magnitude at rated speed).
const GAP_SENSITIVITY: f64 = 8.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct SpeedPickupFaults {
    /// Mounting/rigging gap increase, `0.0` nominal .. `1.0` maximum
    /// modelled increase (not a physical mm value -- no published nominal
    /// air gap for this probe class is available to anchor an absolute
    /// unit, so this is a fraction of the maximum modelled degradation).
    pub air_gap_increase: f64,
    /// Broken wire / connector: `1.0` fully open (no signal at any speed).
    pub open_circuit: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SpeedPickupOutput {
    /// Speed reading as a fraction of rated (100 %) speed; equals the true
    /// value when `valid`, otherwise meaningless (the EEC lane treats a
    /// `!valid` channel as no-signal, not as a wrong number).
    pub speed_frac: f64,
    pub valid: bool,
}

/// One shaft speed pickup on one EEC channel. Stateless (no internal
/// dynamics beyond the instantaneous amplitude-vs-detection-floor check),
/// so this is a pure function rather than a struct with `step`.
pub fn speed_pickup_reading(true_speed_frac: f64, faults: &SpeedPickupFaults) -> SpeedPickupOutput {
    if faults.open_circuit.clamp(0.0, 1.0) >= 0.98 {
        return SpeedPickupOutput { speed_frac: 0.0, valid: false };
    }
    let gap = faults.air_gap_increase.clamp(0.0, 1.0);
    let amplitude_v = REFERENCE_AMPLITUDE_V * true_speed_frac.max(0.0) / (1.0 + GAP_SENSITIVITY * gap).powi(2);
    let valid = amplitude_v >= DETECTION_FLOOR_V;
    SpeedPickupOutput { speed_frac: if valid { true_speed_frac } else { 0.0 }, valid }
}

// ---------------------------------------------------------------------
// TGT thermocouple harness.
// ---------------------------------------------------------------------

/// One thermocouple junction's fault state.
#[derive(Clone, Copy, Debug, Default)]
pub struct TgtJunctionFaults {
    /// Open circuit: `1.0` fully open (this junction drops out of the
    /// average entirely).
    pub open_circuit: f64,
    /// Signed temperature offset this junction's own wiring/junction
    /// degradation adds before averaging, K.
    pub drift_k: f64,
}

/// One physical thermocouple junction: its fixed circumferential mounting
/// position around the turbine annulus (`angle_deg`, 0..360 -- an
/// installation fact, not a fault) plus its own fault state.
#[derive(Clone, Copy, Debug, Default)]
pub struct TgtJunction {
    pub angle_deg: f64,
    pub faults: TgtJunctionFaults,
}

/// A localized combustor exit temperature excess ("hot streak"), the
/// documented consequence of an imbalance across the fuel nozzles feeding
/// the combustor -- for example a partially coked nozzle group flowing less
/// fuel than its neighbours, or one flowing a poorly atomized spray,
/// leaving a hotter- or cooler-than-average sector of gas hitting the
/// turbine (a standard gas-turbine combustion concept; see e.g. Lefebvre &
/// Ballal, *Gas Turbine Combustion*, on circumferential temperature
/// traverse quality and hot streaks). This module takes the hot streak as
/// a **plain input** -- it does not model *why* one exists (fuel nozzle
/// coking is a fuel-system fault another agent's model owns; see
/// `PROGRESS.md`) -- only how a harness of thermocouples at fixed
/// positions around the annulus would sense one.
#[derive(Clone, Copy, Debug, Default)]
pub struct HotStreak {
    /// Circumferential centre of the streak, degrees (0..360).
    pub center_deg: f64,
    /// Angular width (Gaussian standard deviation), degrees. A generic
    /// bell-shaped angular profile is the standard simplified
    /// representation of a localized thermal streak (no public
    /// per-nozzle-group traverse profile exists for this class of
    /// engine) -- GENERIC shape, real position/magnitude driven by the
    /// caller.
    pub width_deg: f64,
    /// Signed peak temperature excess at the streak's centre, K (positive
    /// for a hot streak, negative for a locally cool sector from a nozzle
    /// flowing too little/too lean).
    pub peak_delta_k: f64,
}

/// Shortest angular separation between two compass-style angles, degrees
/// (handles wraparound at 0/360).
fn angular_distance_deg(a_deg: f64, b_deg: f64) -> f64 {
    let d = (a_deg - b_deg).rem_euclid(360.0);
    d.min(360.0 - d)
}

fn hot_streak_delta_k(hot_streak: Option<HotStreak>, angle_deg: f64) -> f64 {
    match hot_streak {
        None => 0.0,
        Some(hs) => {
            let d = angular_distance_deg(angle_deg, hs.center_deg);
            let width = hs.width_deg.max(1e-6);
            hs.peak_delta_k * (-(d * d) / (2.0 * width * width)).exp()
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct TgtHarnessOutput {
    pub average_c: f64,
    /// How many of the harness's junctions actually contributed.
    pub healthy_junctions: usize,
    /// All junctions open: no valid TGT at all.
    pub valid: bool,
}

/// Averages `bulk_tgt_c` (the annulus-average true TGT absent any
/// localized streak) across a harness of thermocouple junctions at their
/// real circumferential positions, each with its own fault state, with an
/// optional [`HotStreak`] locally raising (or lowering) the temperature
/// each junction actually sees depending on how close it sits to the
/// streak's centre.
pub fn tgt_harness_average_c(bulk_tgt_c: f64, hot_streak: Option<HotStreak>, junctions: &[TgtJunction]) -> TgtHarnessOutput {
    let mut sum = 0.0;
    let mut count = 0usize;
    for j in junctions {
        if j.faults.open_circuit.clamp(0.0, 1.0) < 0.98 {
            let local_c = bulk_tgt_c + hot_streak_delta_k(hot_streak, j.angle_deg);
            sum += local_c + j.faults.drift_k;
            count += 1;
        }
    }
    if count == 0 {
        TgtHarnessOutput { average_c: 0.0, healthy_junctions: 0, valid: false }
    } else {
        TgtHarnessOutput { average_c: sum / count as f64, healthy_junctions: count, valid: true }
    }
}

// ---------------------------------------------------------------------
// Vibration pickup.
// ---------------------------------------------------------------------

#[derive(Clone, Copy, Debug, Default)]
pub struct VibrationPickupFaults {
    /// Signed bias added to the true reading, in the same generic
    /// amplitude units as the input.
    pub bias: f64,
    /// Frozen at the last output: `1.0` fully stuck.
    pub stuck: f64,
    /// Probability per second of a momentary signal dropout (loose
    /// connector) -- an intermittent fault, distinct from a clean bias or a
    /// permanent stuck failure.
    pub intermittent_dropout_rate_per_s: f64,
}

pub struct VibrationPickup {
    rng: Rng,
    last_output: f64,
}

impl VibrationPickup {
    pub fn new(seed: u64) -> Self {
        Self { rng: Rng::new(seed), last_output: 0.0 }
    }

    /// Returns `(reading, valid)`; `valid == false` on a dropout tick (the
    /// EEC/indicating system holds its own last value, which is this
    /// struct's caller's responsibility, matching how a real intermittent
    /// dropout is handled downstream rather than reported as a wrong
    /// number).
    pub fn step(&mut self, true_amplitude: f64, faults: &VibrationPickupFaults, dt_s: f64) -> (f64, bool) {
        let dt = dt_s.max(0.0);
        let dropout_p = (faults.intermittent_dropout_rate_per_s.max(0.0) * dt).min(1.0);
        if dropout_p > 0.0 && self.rng.next_f64() < dropout_p {
            return (self.last_output, false);
        }
        let stuck = faults.stuck.clamp(0.0, 1.0);
        let healthy = true_amplitude + faults.bias;
        self.last_output = healthy * (1.0 - stuck) + self.last_output * stuck;
        (self.last_output, true)
    }
}

// ---------------------------------------------------------------------
// Fuel flow transmitter.
// ---------------------------------------------------------------------

#[derive(Clone, Copy, Debug, Default)]
pub struct FuelFlowTransmitterFaults {
    /// Bearing wear/drag lowering the meter's own effective K-factor:
    /// `0.0` nominal .. `1.0` maximum modelled wear (the rotor under-spins
    /// for a given true flow by this fraction).
    pub bearing_wear: f64,
    /// Debris partially blocking flow *through the meter* (upstream of the
    /// rotor), reducing the flow the meter actually sees below the true
    /// engine flow: `0.0` none .. `1.0` fully blocked.
    pub debris_blockage: f64,
    /// Rotor seized: `1.0` fully stuck (reads zero regardless of flow).
    pub stuck_rotor: f64,
}

/// Returns the *indicated* fuel flow (same units as `true_flow_kg_s`) the
/// transmitter reports for a true flow of `true_flow_kg_s`.
pub fn fuel_flow_transmitter_reading(true_flow_kg_s: f64, faults: &FuelFlowTransmitterFaults) -> f64 {
    if faults.stuck_rotor.clamp(0.0, 1.0) >= 0.98 {
        return 0.0;
    }
    let flow_through_meter = true_flow_kg_s.max(0.0) * (1.0 - faults.debris_blockage.clamp(0.0, 1.0));
    let effective_k_factor = 1.0 - faults.bearing_wear.clamp(0.0, 1.0);
    flow_through_meter * effective_k_factor
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- Speed pickup.

    #[test]
    fn healthy_pickup_reads_true_speed_at_any_reasonable_speed() {
        let out = speed_pickup_reading(0.8, &SpeedPickupFaults::default());
        assert!(out.valid);
        assert_eq!(out.speed_frac, 0.8);
    }

    #[test]
    fn open_circuit_is_invalid_at_full_speed() {
        let faults = SpeedPickupFaults { open_circuit: 1.0, ..Default::default() };
        let out = speed_pickup_reading(1.0, &faults);
        assert!(!out.valid);
    }

    #[test]
    fn a_healthy_pickup_still_detects_low_idle_speed() {
        let out = speed_pickup_reading(0.2, &SpeedPickupFaults::default());
        assert!(out.valid);
    }

    #[test]
    fn a_widened_air_gap_loses_signal_at_low_speed_but_not_at_high_speed() {
        let faults = SpeedPickupFaults { air_gap_increase: 0.3, ..Default::default() };
        let low = speed_pickup_reading(0.05, &faults);
        let high = speed_pickup_reading(1.0, &faults);
        assert!(!low.valid, "expected loss of signal at low speed with a widened gap");
        assert!(high.valid, "expected the signal to still be detectable at full speed");
    }

    // ---- TGT harness.

    /// 8 junctions evenly spaced around the annulus, all healthy.
    fn healthy_ring(n: usize) -> Vec<TgtJunction> {
        (0..n).map(|i| TgtJunction { angle_deg: i as f64 * 360.0 / n as f64, faults: TgtJunctionFaults::default() }).collect()
    }

    #[test]
    fn healthy_harness_averages_to_the_true_temperature_with_no_hot_streak() {
        let junctions = healthy_ring(8);
        let out = tgt_harness_average_c(650.0, None, &junctions);
        assert!(out.valid);
        assert_eq!(out.healthy_junctions, 8);
        assert!((out.average_c - 650.0).abs() < 1e-9);
    }

    #[test]
    fn open_junctions_drop_out_of_the_average() {
        let mut junctions = healthy_ring(8);
        junctions[0].faults.open_circuit = 1.0;
        junctions[1].faults.open_circuit = 1.0;
        let out = tgt_harness_average_c(650.0, None, &junctions);
        assert_eq!(out.healthy_junctions, 6);
    }

    #[test]
    fn a_drifting_junction_biases_the_average() {
        let mut junctions = healthy_ring(8);
        junctions[0].faults.drift_k = 80.0;
        let out = tgt_harness_average_c(650.0, None, &junctions);
        // One of 8 junctions reading 80 K high shifts the mean by 10 K.
        assert!((out.average_c - 660.0).abs() < 1e-6, "{}", out.average_c);
    }

    #[test]
    fn all_junctions_open_is_invalid() {
        let junctions: Vec<TgtJunction> = healthy_ring(8)
            .into_iter()
            .map(|j| TgtJunction { faults: TgtJunctionFaults { open_circuit: 1.0, ..Default::default() }, ..j })
            .collect();
        let out = tgt_harness_average_c(650.0, None, &junctions);
        assert!(!out.valid);
    }

    #[test]
    fn a_hot_streak_raises_only_the_nearby_junctions_pulling_the_average_up_less_than_the_peak() {
        let junctions = healthy_ring(8); // at 0, 45, 90, ..., 315 degrees
        let streak = HotStreak { center_deg: 0.0, width_deg: 20.0, peak_delta_k: 80.0 };
        let out = tgt_harness_average_c(650.0, Some(streak), &junctions);
        // The junction at 0 degrees sees (nearly) the full peak; its
        // neighbours at +-45 degrees are far outside a 20-degree-wide
        // streak and see essentially none of it -- so the *average* rises,
        // but by far less than the peak itself.
        assert!(out.average_c > 650.0, "{}", out.average_c);
        assert!(out.average_c < 650.0 + 80.0, "{}", out.average_c);
        assert!(out.average_c < 670.0, "average rose too much for a localized streak: {}", out.average_c);
    }

    #[test]
    fn a_junction_exactly_at_the_streak_centre_reads_close_to_the_full_peak() {
        let junctions = vec![TgtJunction { angle_deg: 90.0, faults: TgtJunctionFaults::default() }];
        let streak = HotStreak { center_deg: 90.0, width_deg: 20.0, peak_delta_k: 80.0 };
        let out = tgt_harness_average_c(650.0, Some(streak), &junctions);
        assert!((out.average_c - 730.0).abs() < 1e-6, "{}", out.average_c);
    }

    #[test]
    fn hot_streak_wraps_around_zero_degrees() {
        // A junction at 355 degrees is only 10 degrees from a streak
        // centred at 5 degrees (wrapping through 0), not 350 degrees the
        // long way around.
        let junctions = vec![TgtJunction { angle_deg: 355.0, faults: TgtJunctionFaults::default() }];
        let narrow_streak = HotStreak { center_deg: 5.0, width_deg: 5.0, peak_delta_k: 80.0 };
        let out = tgt_harness_average_c(650.0, Some(narrow_streak), &junctions);
        // 10 degrees is 2 widths away on a 5-degree-wide streak: still a
        // meaningful fraction of the peak (not the near-zero it would be
        // if the wraparound were computed wrong as 350 degrees away).
        assert!(out.average_c > 655.0, "{}", out.average_c);
    }

    // A plain, out-of-scope-cause input: whatever produces a hot streak
    // (e.g. a coked fuel nozzle group -- another agent's model) only needs
    // to hand this module a `HotStreak { center_deg, width_deg,
    // peak_delta_k }`; this module does not know or care why one exists.
    #[test]
    fn hot_streak_is_a_plain_input_independent_of_its_cause() {
        let junctions = healthy_ring(8);
        let from_any_cause = HotStreak { center_deg: 180.0, width_deg: 15.0, peak_delta_k: 120.0 };
        let out = tgt_harness_average_c(700.0, Some(from_any_cause), &junctions);
        assert!(out.valid);
        assert!(out.average_c > 700.0);
    }

    // ---- Vibration pickup.

    #[test]
    fn healthy_vibration_pickup_tracks_true_amplitude() {
        let mut v = VibrationPickup::new(1);
        let (reading, valid) = v.step(0.3, &VibrationPickupFaults::default(), 0.1);
        assert!(valid);
        assert!((reading - 0.3).abs() < 1e-9);
    }

    #[test]
    fn stuck_vibration_pickup_freezes() {
        let mut v = VibrationPickup::new(1);
        let (first, _) = v.step(0.3, &VibrationPickupFaults::default(), 0.1);
        let faults = VibrationPickupFaults { stuck: 1.0, ..Default::default() };
        let (second, _) = v.step(0.9, &faults, 0.1);
        assert_eq!(first, second);
    }

    #[test]
    fn intermittent_dropout_eventually_occurs_and_holds_the_last_value() {
        let mut v = VibrationPickup::new(1);
        let faults = VibrationPickupFaults { intermittent_dropout_rate_per_s: 5.0, ..Default::default() };
        let mut saw_dropout = false;
        let mut last_valid_reading = 0.0;
        for _ in 0..200 {
            let (reading, valid) = v.step(0.5, &faults, 0.1);
            if valid {
                last_valid_reading = reading;
            } else {
                saw_dropout = true;
                assert_eq!(reading, last_valid_reading);
            }
        }
        assert!(saw_dropout, "expected at least one dropout over 20 s at a 5/s dropout rate");
    }

    // ---- Fuel flow transmitter.

    #[test]
    fn healthy_transmitter_reads_true_flow() {
        let reading = fuel_flow_transmitter_reading(1.2, &FuelFlowTransmitterFaults::default());
        assert!((reading - 1.2).abs() < 1e-9);
    }

    #[test]
    fn bearing_wear_under_reads() {
        let faults = FuelFlowTransmitterFaults { bearing_wear: 0.2, ..Default::default() };
        let reading = fuel_flow_transmitter_reading(1.2, &faults);
        assert!((reading - 0.96).abs() < 1e-9, "{reading}");
    }

    #[test]
    fn debris_blockage_under_reads_distinctly_from_wear() {
        let faults = FuelFlowTransmitterFaults { debris_blockage: 0.5, ..Default::default() };
        let reading = fuel_flow_transmitter_reading(1.2, &faults);
        assert!((reading - 0.6).abs() < 1e-9, "{reading}");
    }

    #[test]
    fn stuck_rotor_reads_zero_regardless_of_flow() {
        let faults = FuelFlowTransmitterFaults { stuck_rotor: 1.0, ..Default::default() };
        assert_eq!(fuel_flow_transmitter_reading(5.0, &faults), 0.0);
    }

    #[test]
    fn no_nan_at_zero_inputs() {
        assert!(speed_pickup_reading(0.0, &SpeedPickupFaults::default()).speed_frac.is_finite());
        assert!(tgt_harness_average_c(0.0, None, &[]).average_c.is_finite());
        let mut v = VibrationPickup::new(1);
        assert!(v.step(0.0, &VibrationPickupFaults::default(), 0.0).0.is_finite());
        assert!(fuel_flow_transmitter_reading(0.0, &FuelFlowTransmitterFaults::default()).is_finite());
    }
}
