use super::rng::Rng;

const REFERENCE_AMPLITUDE_V: f64 = 3.0;
const DETECTION_FLOOR_V: f64 = 0.15;
const GAP_SENSITIVITY: f64 = 8.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct SpeedPickupFaults {
    pub air_gap_increase: f64,
    pub open_circuit: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SpeedPickupOutput {
    pub speed_frac: f64,
    pub valid: bool,
}

pub fn speed_pickup_reading(true_speed_frac: f64, faults: &SpeedPickupFaults) -> SpeedPickupOutput {
    if faults.open_circuit.clamp(0.0, 1.0) >= 0.98 {
        return SpeedPickupOutput { speed_frac: 0.0, valid: false };
    }
    let gap = faults.air_gap_increase.clamp(0.0, 1.0);
    let amplitude_v = REFERENCE_AMPLITUDE_V * true_speed_frac.max(0.0) / (1.0 + GAP_SENSITIVITY * gap).powi(2);
    let valid = amplitude_v >= DETECTION_FLOOR_V;
    SpeedPickupOutput { speed_frac: if valid { true_speed_frac } else { 0.0 }, valid }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct TgtJunctionFaults {
    pub open_circuit: f64,
    pub drift_k: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct TgtJunction {
    pub angle_deg: f64,
    pub faults: TgtJunctionFaults,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct HotStreak {
    pub center_deg: f64,
    pub width_deg: f64,
    pub peak_delta_k: f64,
}

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
    pub healthy_junctions: usize,
    pub valid: bool,
}

pub fn tgt_harness_average_c(bulk_tgt_c: f64, hot_streak: Option<HotStreak>, junctions: &[TgtJunction]) -> TgtHarnessOutput {
    let one;
    let streaks: &[HotStreak] = match hot_streak {
        Some(h) => {
            one = [h];
            &one
        }
        None => &[],
    };
    tgt_harness_average_streaks_c(bulk_tgt_c, streaks, junctions)
}

pub fn tgt_harness_average_streaks_c(bulk_tgt_c: f64, streaks: &[HotStreak], junctions: &[TgtJunction]) -> TgtHarnessOutput {
    let mut sum = 0.0;
    let mut count = 0usize;
    for j in junctions {
        if j.faults.open_circuit.clamp(0.0, 1.0) < 0.98 {
            let local_c = bulk_tgt_c + streaks.iter().map(|h| hot_streak_delta_k(Some(*h), j.angle_deg)).sum::<f64>();
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

#[derive(Clone, Copy, Debug, Default)]
pub struct VibrationPickupFaults {
    pub bias: f64,
    pub stuck: f64,
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

#[derive(Clone, Copy, Debug, Default)]
pub struct FuelFlowTransmitterFaults {
    pub bearing_wear: f64,
    pub debris_blockage: f64,
    pub stuck_rotor: f64,
}

pub fn fuel_flow_transmitter_reading(true_flow_kg_s: f64, faults: &FuelFlowTransmitterFaults) -> f64 {
    if faults.stuck_rotor.clamp(0.0, 1.0) >= 0.98 {
        return 0.0;
    }
    let flow_through_meter = true_flow_kg_s.max(0.0) * (1.0 - faults.debris_blockage.clamp(0.0, 1.0));
    let effective_k_factor = 1.0 - faults.bearing_wear.clamp(0.0, 1.0);
    flow_through_meter * effective_k_factor
}

const FUEL_FLOW_RUNNING_THRESHOLD_KG_S: f64 = 0.05;

pub fn fuel_flow_transmitter_fault(true_flow_kg_s: f64, faults: &FuelFlowTransmitterFaults) -> bool {
    true_flow_kg_s > FUEL_FLOW_RUNNING_THRESHOLD_KG_S
        && fuel_flow_transmitter_reading(true_flow_kg_s, faults) < FUEL_FLOW_RUNNING_THRESHOLD_KG_S * 0.5
}

#[cfg(test)]
mod tests {
    use super::*;

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
        let junctions = healthy_ring(8);
        let streak = HotStreak { center_deg: 0.0, width_deg: 20.0, peak_delta_k: 80.0 };
        let out = tgt_harness_average_c(650.0, Some(streak), &junctions);
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
        let junctions = vec![TgtJunction { angle_deg: 355.0, faults: TgtJunctionFaults::default() }];
        let narrow_streak = HotStreak { center_deg: 5.0, width_deg: 5.0, peak_delta_k: 80.0 };
        let out = tgt_harness_average_c(650.0, Some(narrow_streak), &junctions);
        assert!(out.average_c > 655.0, "{}", out.average_c);
    }

    #[test]
    fn hot_streak_is_a_plain_input_independent_of_its_cause() {
        let junctions = healthy_ring(8);
        let from_any_cause = HotStreak { center_deg: 180.0, width_deg: 15.0, peak_delta_k: 120.0 };
        let out = tgt_harness_average_c(700.0, Some(from_any_cause), &junctions);
        assert!(out.valid);
        assert!(out.average_c > 700.0);
    }

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
    fn a_seized_rotor_with_the_engine_running_is_a_fault_not_a_shutdown() {
        let faults = FuelFlowTransmitterFaults { stuck_rotor: 1.0, ..Default::default() };
        assert!(fuel_flow_transmitter_fault(2.0, &faults));
    }

    #[test]
    fn a_zero_reading_with_the_engine_actually_shut_down_is_not_a_fault() {
        assert!(!fuel_flow_transmitter_fault(0.0, &FuelFlowTransmitterFaults::default()));
    }

    #[test]
    fn a_healthy_transmitter_with_the_engine_running_is_not_a_fault() {
        assert!(!fuel_flow_transmitter_fault(2.0, &FuelFlowTransmitterFaults::default()));
    }

    #[test]
    fn a_seized_rotor_at_idle_flow_is_still_a_fault() {
        let faults = FuelFlowTransmitterFaults { stuck_rotor: 1.0, ..Default::default() };
        assert!(fuel_flow_transmitter_fault(0.22, &faults), "idle-range true flow must still trip the fault");
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
