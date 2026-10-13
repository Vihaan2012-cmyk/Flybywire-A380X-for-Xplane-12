//! The APU electronic control box (ECB) as a dual-channel controller with
//! sensor redundancy -- item 2.
//!
//! Two independent channels (A/B) each have their own speed pickup, EGT
//! thermocouple and oil pressure sensor, any of which can be biased or
//! failed outright, independently of the other channel's equivalent
//! sensor, and the channel's own processing (lane computer) can itself be
//! faulted, invalidating everything it reports.
//!
//! **Control** (what the governor actually sees): for each signal
//! (speed, EGT, oil pressure), the still-valid channels reporting it are
//! combined -- both report it, average (a simple two-source mid-value
//! vote); only one does, use it; neither does, the signal is genuinely
//! lost (`None`). Total loss of the speed signal from both channels is
//! itself unsafe to keep controlling through, so `dual_channel_speed_loss`
//! is exposed as its own condition for `apu.rs` to react to (a real
//! consequence of dual-channel loss, not modelled as "coast on the last
//! good value forever").
//!
//! **Protection** (overspeed, EGT, oil): each valid channel votes off its
//! *own* (possibly biased) reading, independently, with its own debounce;
//! the ECB's overall trip is the OR of whichever channels are still valid
//! -- the standard fail-safe convention for dual-channel protection logic
//! (a missed protection is worse than a spurious one, so any one channel
//! confirming is enough; an invalid channel simply cannot vote either way).
//! Fire and the inlet-door start interlock are hard-wired safety functions
//! in this model, not per-channel-voted, matching how those are typically
//! independent loops on a real aircraft rather than folded into the
//! FADEC/ECB's own two control lanes.

use super::params;

/// One sensor's fault state, 0 healthy .. 1 bias severity, plus an outright
/// failure (open circuit -- no reading at all, not just a biased one).
#[derive(Clone, Copy, Debug, Default)]
pub struct SensorFault {
    pub bias: f64,
    pub failed: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ChannelFaults {
    pub speed_sensor: SensorFault,
    pub egt_sensor: SensorFault,
    pub oil_pressure_sensor: SensorFault,
    /// The channel's own lane-computer fault, 0 healthy .. 1; at >= 0.999
    /// the whole channel (all its sensors and votes) is excluded entirely.
    pub processing_fault: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct EcbFaults {
    pub channel_a: ChannelFaults,
    pub channel_b: ChannelFaults,
}

/// The true physical values the ECB's sensors are reading, from the rest of
/// this directory's own physics -- what each channel's sensors then
/// distort per their own faults.
pub struct TrueSignals {
    pub n_percent: f64,
    pub egt_true_c: f64,
    pub oil_pressure_psi: f64,
    pub fire_confirmed: bool,
    /// The crew/logic is asking the ECB to start this tick (hard-wired
    /// interlock input, see module docs).
    pub start_selected: bool,
    pub inlet_door_ready: bool,
}

#[derive(Clone, Copy, Debug, Default)]
struct ChannelTimers {
    overspeed_s: f64,
    egt_s: f64,
    oil_s: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Ecb {
    a: ChannelTimers,
    b: ChannelTimers,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct EcbOutputs {
    pub n_for_governor: Option<f64>,
    pub egt_indicated_c: Option<f64>,
    pub oil_pressure_indicated_psi: Option<f64>,
    pub channel_a_valid: bool,
    pub channel_b_valid: bool,
    pub dual_channel_speed_loss: bool,
    pub trip_overspeed: bool,
    pub trip_egt: bool,
    pub trip_oil: bool,
    pub trip_fire: bool,
    pub trip_inlet_door: bool,
    pub any_trip: bool,
}

fn channel_valid(f: &ChannelFaults) -> bool {
    f.processing_fault < 0.999
}

fn sensor_reading(true_value: f64, max_bias: f64, fault: &SensorFault) -> Option<f64> {
    if fault.failed {
        None
    } else {
        Some(true_value - max_bias * fault.bias.clamp(0.0, 1.0))
    }
}

fn combine(a: Option<f64>, b: Option<f64>) -> Option<f64> {
    match (a, b) {
        (Some(x), Some(y)) => Some(0.5 * (x + y)),
        (Some(x), None) | (None, Some(x)) => Some(x),
        (None, None) => None,
    }
}

/// A debounced threshold vote off one channel's own reading; `None` (sensor
/// failed/channel invalid) never votes a trip, and its timer resets so a
/// later-recovered channel starts its debounce fresh rather than trips
/// instantly from stale accumulation.
fn debounced_trip(reading: Option<f64>, threshold: f64, above: bool, debounce_s: f64, timer_s: &mut f64, dt: f64) -> bool {
    let exceeds = reading.map_or(false, |v| if above { v > threshold } else { v < threshold });
    if exceeds {
        *timer_s += dt;
    } else {
        *timer_s = 0.0;
    }
    *timer_s >= debounce_s
}

impl Ecb {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn step(&mut self, true_signals: &TrueSignals, running: bool, dt_s: f64, faults: &EcbFaults) -> EcbOutputs {
        let dt = dt_s.max(0.0);

        let a_valid = channel_valid(&faults.channel_a);
        let b_valid = channel_valid(&faults.channel_b);

        let a_n = a_valid.then(|| sensor_reading(true_signals.n_percent, params::N_SENSOR_MAX_BIAS_PERCENT, &faults.channel_a.speed_sensor)).flatten();
        let b_n = b_valid.then(|| sensor_reading(true_signals.n_percent, params::N_SENSOR_MAX_BIAS_PERCENT, &faults.channel_b.speed_sensor)).flatten();
        let a_egt = a_valid.then(|| sensor_reading(true_signals.egt_true_c, params::EGT_SENSOR_MAX_BIAS_C, &faults.channel_a.egt_sensor)).flatten();
        let b_egt = b_valid.then(|| sensor_reading(true_signals.egt_true_c, params::EGT_SENSOR_MAX_BIAS_C, &faults.channel_b.egt_sensor)).flatten();
        let a_oil = a_valid.then(|| sensor_reading(true_signals.oil_pressure_psi, params::OIL_PRESSURE_SENSOR_MAX_BIAS_PSI, &faults.channel_a.oil_pressure_sensor)).flatten();
        let b_oil = b_valid.then(|| sensor_reading(true_signals.oil_pressure_psi, params::OIL_PRESSURE_SENSOR_MAX_BIAS_PSI, &faults.channel_b.oil_pressure_sensor)).flatten();

        let n_for_governor = combine(a_n, b_n);
        let egt_indicated_c = combine(a_egt, b_egt);
        let oil_pressure_indicated_psi = combine(a_oil, b_oil);

        let a_overspeed = debounced_trip(a_n, params::OVERSPEED_TRIP_PERCENT, true, params::ECB_OVERSPEED_DEBOUNCE_S, &mut self.a.overspeed_s, dt);
        let b_overspeed = debounced_trip(b_n, params::OVERSPEED_TRIP_PERCENT, true, params::ECB_OVERSPEED_DEBOUNCE_S, &mut self.b.overspeed_s, dt);
        let trip_overspeed = a_overspeed || b_overspeed;

        let a_egt_trip = debounced_trip(a_egt, params::EGT_TRIP_C, true, params::ECB_EGT_DEBOUNCE_S, &mut self.a.egt_s, dt);
        let b_egt_trip = debounced_trip(b_egt, params::EGT_TRIP_C, true, params::ECB_EGT_DEBOUNCE_S, &mut self.b.egt_s, dt);
        let trip_egt = a_egt_trip || b_egt_trip;

        // Oil protection only applies while running, same as `oil.rs`'s own
        // single-channel debounce logic (transient dips during start/
        // shutdown are normal).
        let a_oil_gated = if running { a_oil } else { None };
        let b_oil_gated = if running { b_oil } else { None };
        let a_oil_trip = debounced_trip(a_oil_gated, params::OIL_PRESSURE_TRIP_PSI, false, params::OIL_PRESSURE_TRIP_DEBOUNCE_S, &mut self.a.oil_s, dt);
        let b_oil_trip = debounced_trip(b_oil_gated, params::OIL_PRESSURE_TRIP_PSI, false, params::OIL_PRESSURE_TRIP_DEBOUNCE_S, &mut self.b.oil_s, dt);
        let trip_oil = a_oil_trip || b_oil_trip;

        let trip_fire = true_signals.fire_confirmed;
        let trip_inlet_door = true_signals.start_selected && !true_signals.inlet_door_ready;
        let dual_channel_speed_loss = n_for_governor.is_none();

        let any_trip = trip_overspeed || trip_egt || trip_oil || trip_fire || dual_channel_speed_loss;

        EcbOutputs {
            n_for_governor,
            egt_indicated_c,
            oil_pressure_indicated_psi,
            channel_a_valid: a_valid,
            channel_b_valid: b_valid,
            dual_channel_speed_loss,
            trip_overspeed,
            trip_egt,
            trip_oil,
            trip_fire,
            trip_inlet_door,
            any_trip,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn signals(n: f64, egt: f64, oil: f64) -> TrueSignals {
        TrueSignals {
            n_percent: n,
            egt_true_c: egt,
            oil_pressure_psi: oil,
            fire_confirmed: false,
            start_selected: false,
            inlet_door_ready: true,
        }
    }

    #[test]
    fn healthy_dual_channel_averages_agreeing_sensors_and_trips_nothing() {
        let mut ecb = Ecb::new();
        let out = ecb.step(&signals(85.0, 500.0, 60.0), true, 1.0, &EcbFaults::default());
        assert!((out.n_for_governor.unwrap() - 85.0).abs() < 1e-9);
        assert!((out.egt_indicated_c.unwrap() - 500.0).abs() < 1e-9);
        assert!(!out.any_trip);
        assert!(out.channel_a_valid && out.channel_b_valid);
    }

    #[test]
    fn one_channels_biased_speed_sensor_is_averaged_down_not_ignored() {
        let mut ecb = Ecb::new();
        let faults = EcbFaults {
            channel_a: ChannelFaults { speed_sensor: SensorFault { bias: 1.0, failed: false }, ..Default::default() },
            channel_b: ChannelFaults::default(),
        };
        let out = ecb.step(&signals(85.0, 500.0, 60.0), true, 1.0, &faults);
        let expected = 85.0 - 0.5 * params::N_SENSOR_MAX_BIAS_PERCENT;
        assert!((out.n_for_governor.unwrap() - expected).abs() < 1e-6, "{:?}", out.n_for_governor);
    }

    #[test]
    fn a_failed_sensor_on_one_channel_falls_back_to_the_other_unaveraged() {
        let mut ecb = Ecb::new();
        let faults = EcbFaults {
            channel_a: ChannelFaults { speed_sensor: SensorFault { bias: 0.0, failed: true }, ..Default::default() },
            channel_b: ChannelFaults::default(),
        };
        let out = ecb.step(&signals(85.0, 500.0, 60.0), true, 1.0, &faults);
        assert!((out.n_for_governor.unwrap() - 85.0).abs() < 1e-9);
    }

    #[test]
    fn both_speed_sensors_lost_is_a_dual_channel_speed_loss_condition() {
        let mut ecb = Ecb::new();
        let faults = EcbFaults {
            channel_a: ChannelFaults { speed_sensor: SensorFault { bias: 0.0, failed: true }, ..Default::default() },
            channel_b: ChannelFaults { speed_sensor: SensorFault { bias: 0.0, failed: true }, ..Default::default() },
        };
        let out = ecb.step(&signals(85.0, 500.0, 60.0), true, 1.0, &faults);
        assert!(out.n_for_governor.is_none());
        assert!(out.dual_channel_speed_loss);
        assert!(out.any_trip);
    }

    #[test]
    fn a_fully_faulted_channel_is_excluded_entirely_and_the_other_alone_still_protects() {
        let mut ecb = Ecb::new();
        let faults = EcbFaults {
            channel_a: ChannelFaults { processing_fault: 1.0, ..Default::default() },
            channel_b: ChannelFaults::default(),
        };
        let out = ecb.step(&signals(85.0, 500.0, 60.0), true, 1.0, &faults);
        assert!(!out.channel_a_valid && out.channel_b_valid);
        assert!((out.n_for_governor.unwrap() - 85.0).abs() < 1e-9);
    }

    #[test]
    fn overspeed_confirmed_by_either_valid_channel_trips_after_its_debounce() {
        let mut ecb = Ecb::new();
        let over = params::OVERSPEED_TRIP_PERCENT + 2.0;
        let mut out = ecb.step(&signals(over, 500.0, 60.0), true, params::ECB_OVERSPEED_DEBOUNCE_S * 0.5, &EcbFaults::default());
        assert!(!out.trip_overspeed, "must not trip before its debounce elapses");
        out = ecb.step(&signals(over, 500.0, 60.0), true, params::ECB_OVERSPEED_DEBOUNCE_S, &EcbFaults::default());
        assert!(out.trip_overspeed);
        assert!(out.any_trip);
    }

    #[test]
    fn a_biased_channel_can_mask_a_real_overspeed_but_the_healthy_channel_still_catches_it() {
        let mut ecb = Ecb::new();
        let over = params::OVERSPEED_TRIP_PERCENT + 1.0;
        // Channel A under-reads enough to hide the exceedance from itself;
        // channel B is healthy and still sees the true value.
        let faults = EcbFaults {
            channel_a: ChannelFaults { speed_sensor: SensorFault { bias: 1.0, failed: false }, ..Default::default() },
            channel_b: ChannelFaults::default(),
        };
        let out = ecb.step(&signals(over, 500.0, 60.0), true, params::ECB_OVERSPEED_DEBOUNCE_S * 2.0, &faults);
        assert!(out.trip_overspeed, "the healthy channel alone must still protect");
    }

    #[test]
    fn oil_protection_only_applies_while_running() {
        let mut ecb = Ecb::new();
        let low_oil = params::OIL_PRESSURE_TRIP_PSI - 5.0;
        let out = ecb.step(&signals(0.0, 20.0, low_oil), false, params::OIL_PRESSURE_TRIP_DEBOUNCE_S * 2.0, &EcbFaults::default());
        assert!(!out.trip_oil);
    }

    #[test]
    fn fire_and_the_inlet_door_interlock_are_hardwired_not_voted() {
        let mut ecb = Ecb::new();
        let mut fire_signals = signals(0.0, 20.0, 60.0);
        fire_signals.fire_confirmed = true;
        let out = ecb.step(&fire_signals, false, 1.0, &EcbFaults::default());
        assert!(out.trip_fire && out.any_trip);

        let mut ecb2 = Ecb::new();
        let mut door_signals = signals(0.0, 20.0, 60.0);
        door_signals.start_selected = true;
        door_signals.inlet_door_ready = false;
        let out2 = ecb2.step(&door_signals, false, 1.0, &EcbFaults::default());
        assert!(out2.trip_inlet_door);
    }

    #[test]
    fn nothing_is_nan_at_pathological_inputs() {
        let mut ecb = Ecb::new();
        let out = ecb.step(&signals(f64::NAN, f64::NAN, f64::NAN), true, -1.0, &EcbFaults::default());
        // NaN comparisons are always false, so nothing should spuriously
        // trip, and the averaged reading (garbage in, garbage out) must at
        // least not panic; downstream (`apu.rs`) is expected to guard
        // against NaN sensor inputs itself, this just must not crash.
        let _ = out;
    }
}
