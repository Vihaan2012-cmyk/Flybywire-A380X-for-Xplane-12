//! EEC (electronic engine control, the Trent's FADEC): dual, fully
//! independent channels (A/B), each with its own complete set of N1/N2/N3/
//! TGT/P30 sensors -- not a single sensor set duplicated in software, a
//! second physically separate measurement chain for every parameter the
//! control law needs, exactly like the fuel flow transmitter's own two
//! pick-off coils (`fuel::flow_transmitter`) this module treats as a sixth
//! voted parameter rather than re-modelling. Normal operation runs on one
//! channel (A, by convention) with the other in hot standby; a channel
//! declared faulted hands control to the healthy one, and only losing
//! *both* leaves no valid channel at all -- the same redundant-restraint
//! shape `thrust_reverser.rs`'s three locks use, here across two channels
//! instead of three.
//!
//! "Sensor drift/disagree voting" is exactly what it says: each channel's
//! sensors independently lag and can carry a bias/frozen fault (reusing the
//! same first-order-lag-plus-bias-plus-freeze shape as every other sensor
//! in this directory, `fuel::flow_transmitter::PickoffFaults`); this module
//! then compares the two channels' readings for each parameter and flags a
//! disagreement once it exceeds a generic per-parameter threshold, which is
//! the actual physical basis for a real EEC's channel-disagree monitor.
//!
//! No Trent-900 EEC sensor figures are public. Every threshold/bias/time
//! constant below is **GENERIC**, chosen to be a plausible order of
//! magnitude for a digital engine sensor chain, not measured.

/// The five parameters each channel senses independently (fuel flow is
/// handled separately by `fuel::flow_transmitter` and folded into this
/// module's voting only as precomputed channel readings, see `step`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Param {
    N1,
    N2,
    N3,
    Tgt,
    P30,
}
pub const PARAMS: [Param; 5] = [Param::N1, Param::N2, Param::N3, Param::Tgt, Param::P30];

fn index(p: Param) -> usize {
    match p {
        Param::N1 => 0,
        Param::N2 => 1,
        Param::N3 => 2,
        Param::Tgt => 3,
        Param::P30 => 4,
    }
}

/// Full-scale bias magnitude at `bias == 1.0`, and the disagree threshold,
/// in each parameter's own natural unit (percent for N1/N2/N3, K for TGT,
/// Pa for P30). **GENERIC**.
const FULL_SCALE_BIAS: [f64; 5] = [5.0, 5.0, 5.0, 50.0, 3.0e5];
const DISAGREE_THRESHOLD: [f64; 5] = [2.0, 2.0, 2.0, 15.0, 1.5e5];
/// Sensor electronics lag, s (**GENERIC**, a fast digital sensor chain).
const SENSOR_TIME_CONSTANT_S: f64 = 0.1;
/// Fuel-flow channel disagree threshold, kg/s (mirrors
/// `fuel::flow_transmitter`'s own bias scale; kept here too so this
/// module's voting logic is self-contained for all six parameters).
pub const FUEL_FLOW_DISAGREE_THRESHOLD_KG_S: f64 = 1.0;

/// One sensor channel's fault state, 0 (healthy) .. 1 (fully failed).
#[derive(Clone, Copy, Debug, Default)]
pub struct SensorFaults {
    pub bias: f64,
    pub frozen: f64,
}

/// Faults the whole EEC can carry.
#[derive(Clone, Copy, Debug)]
pub struct EecFaults {
    /// Whole-channel electronics failure, 0 healthy .. 1 dead: a dead
    /// channel's sensors are ignored entirely by the voting/selection logic
    /// below, not merely biased.
    pub channel_a_fault: f64,
    pub channel_b_fault: f64,
    pub sensor_a: [SensorFaults; 5],
    pub sensor_b: [SensorFaults; 5],
}

impl Default for EecFaults {
    fn default() -> Self {
        Self { channel_a_fault: 0.0, channel_b_fault: 0.0, sensor_a: [SensorFaults::default(); 5], sensor_b: [SensorFaults::default(); 5] }
    }
}

#[derive(Clone, Copy, Debug)]
struct Channel {
    held: [f64; 5],
}

impl Channel {
    fn new() -> Self {
        Self { held: [0.0; 5] }
    }

    fn step(&mut self, true_values: [f64; 5], faults: &[SensorFaults; 5], dt_s: f64) -> [f64; 5] {
        let dt = dt_s.max(0.0);
        let k = 1.0 - (-dt / SENSOR_TIME_CONSTANT_S).exp();
        for i in 0..5 {
            let biased_target = true_values[i] + faults[i].bias.clamp(-1.0, 1.0) * FULL_SCALE_BIAS[i];
            let lagged = self.held[i] + (biased_target - self.held[i]) * k;
            let frozen = faults[i].frozen.clamp(0.0, 1.0);
            self.held[i] = self.held[i] * frozen + lagged * (1.0 - frozen);
        }
        self.held
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ActiveChannel {
    A,
    B,
    /// Both channels declared faulted: no valid EEC control channel
    /// remains (a real aircraft would revert to a degraded/alternate mode
    /// this directory does not model further).
    None,
}

#[derive(Clone, Copy, Debug)]
pub struct EecState {
    /// The five sensed parameters as the active channel (or, with no valid
    /// channel, the average of both -- a degraded last resort, not a
    /// control-quality signal) reports them.
    pub selected: [f64; 5],
    pub disagree: [bool; 5],
    pub active: ActiveChannel,
    pub fuel_flow_disagree: bool,
    /// Each channel's own serviceability, as the EEC's built-in test
    /// equipment declares it -- the same discrete verdict [`Eec::select`]
    /// already derives to decide which channel flies the engine, reported
    /// rather than thrown away.
    ///
    /// Without this, losing the *standby* channel is invisible: selection
    /// keeps the healthy channel in control and nothing downstream can
    /// tell a dual-channel EEC from one running single-channel, which is
    /// precisely the condition an EEC CHANNEL FAULT annunciates.
    pub channel_a_serviceable: bool,
    pub channel_b_serviceable: bool,
}

impl EecState {
    pub fn value(&self, p: Param) -> f64 {
        self.selected[index(p)]
    }
    pub fn disagrees(&self, p: Param) -> bool {
        self.disagree[index(p)]
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Eec {
    channel_a: Channel,
    channel_b: Channel,
}

impl Eec {
    pub fn new() -> Self {
        Self { channel_a: Channel::new(), channel_b: Channel::new() }
    }

    /// Whether a channel's electronics are serviceable at all: below half
    /// a fully-dead channel it still controls, above it the EEC declares
    /// it lost and ignores it entirely rather than merely distrusting it.
    fn channel_serviceable(fault: f64) -> bool {
        fault.clamp(0.0, 1.0) < 0.5
    }

    fn select(a_fault: f64, b_fault: f64) -> ActiveChannel {
        let a_ok = Self::channel_serviceable(a_fault);
        let b_ok = Self::channel_serviceable(b_fault);
        if a_ok {
            ActiveChannel::A
        } else if b_ok {
            ActiveChannel::B
        } else {
            ActiveChannel::None
        }
    }

    /// One step. `true_values` are the five real physical values (N1%,
    /// N2%, N3%, TGT K, P30 Pa) both channels' sensors are trying to
    /// measure; `fuel_flow_a_kg_s`/`fuel_flow_b_kg_s` are the already-
    /// computed dual pick-off readings from `fuel::flow_transmitter`.
    pub fn step(&mut self, true_values: [f64; 5], fuel_flow_a_kg_s: f64, fuel_flow_b_kg_s: f64, faults: &EecFaults, dt_s: f64) -> EecState {
        let reads_a = self.channel_a.step(true_values, &faults.sensor_a, dt_s);
        let reads_b = self.channel_b.step(true_values, &faults.sensor_b, dt_s);
        let active = Self::select(faults.channel_a_fault, faults.channel_b_fault);

        let selected = match active {
            ActiveChannel::A => reads_a,
            ActiveChannel::B => reads_b,
            ActiveChannel::None => {
                let mut mid = [0.0; 5];
                for i in 0..5 {
                    mid[i] = 0.5 * (reads_a[i] + reads_b[i]);
                }
                mid
            }
        };
        let mut disagree = [false; 5];
        for i in 0..5 {
            disagree[i] = (reads_a[i] - reads_b[i]).abs() > DISAGREE_THRESHOLD[i];
        }

        EecState {
            selected,
            disagree,
            active,
            fuel_flow_disagree: (fuel_flow_a_kg_s - fuel_flow_b_kg_s).abs() > FUEL_FLOW_DISAGREE_THRESHOLD_KG_S,
            channel_a_serviceable: Self::channel_serviceable(faults.channel_a_fault),
            channel_b_serviceable: Self::channel_serviceable(faults.channel_b_fault),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn truth() -> [f64; 5] {
        [19.0, 55.0, 62.0, 650.0, 1.2e6]
    }

    fn settle(eec: &mut Eec, faults: &EecFaults, seconds: f64) -> EecState {
        let dt = 0.02;
        let mut out = eec.step(truth(), 3.4, 3.4, faults, 0.0);
        for _ in 0..(seconds / dt) as usize {
            out = eec.step(truth(), 3.4, 3.4, faults, dt);
        }
        out
    }

    #[test]
    fn a_healthy_eec_settles_on_the_true_values_from_channel_a_with_no_disagree() {
        let mut eec = Eec::new();
        let s = settle(&mut eec, &EecFaults::default(), 3.0);
        assert_eq!(s.active, ActiveChannel::A);
        for (i, t) in truth().iter().enumerate() {
            assert!((s.selected[i] - t).abs() < 0.5, "param {i}: {} vs {}", s.selected[i], t);
        }
        assert!(s.disagree.iter().all(|d| !d));
        assert!(!s.fuel_flow_disagree);
    }

    #[test]
    fn a_dead_channel_a_hands_control_to_channel_b() {
        let mut eec = Eec::new();
        let faults = EecFaults { channel_a_fault: 1.0, ..EecFaults::default() };
        let s = settle(&mut eec, &faults, 3.0);
        assert_eq!(s.active, ActiveChannel::B);
        assert!((s.selected[0] - truth()[0]).abs() < 0.5, "B is healthy and should still report the truth");
    }

    #[test]
    fn both_channels_dead_gives_no_valid_channel() {
        let mut eec = Eec::new();
        let faults = EecFaults { channel_a_fault: 1.0, channel_b_fault: 1.0, ..EecFaults::default() };
        let s = settle(&mut eec, &faults, 3.0);
        assert_eq!(s.active, ActiveChannel::None);
    }

    #[test]
    fn a_biased_channel_a_sensor_disagrees_with_a_healthy_channel_b() {
        let mut eec = Eec::new();
        let mut faults = EecFaults::default();
        faults.sensor_a[super::index(Param::N1)] = SensorFaults { bias: 1.0, ..Default::default() };
        let s = settle(&mut eec, &faults, 3.0);
        assert!(s.disagrees(Param::N1));
        assert!(!s.disagrees(Param::N2), "only the biased parameter should disagree");
    }

    #[test]
    fn control_still_uses_the_biased_channel_a_reading_while_it_is_not_declared_dead() {
        // A biased sensor alone does not fail the whole channel in this
        // model (that is a separate, coarser `channel_a_fault`); the
        // disagree flag exists for annunciation/maintenance, control
        // authority stays with the nominally-active channel.
        let mut eec = Eec::new();
        let mut faults = EecFaults::default();
        faults.sensor_a[super::index(Param::N1)] = SensorFaults { bias: 1.0, ..Default::default() };
        let s = settle(&mut eec, &faults, 3.0);
        assert!((s.value(Param::N1) - (truth()[0] + FULL_SCALE_BIAS[0])).abs() < 0.5);
    }

    #[test]
    fn a_frozen_sensor_stops_updating() {
        let mut eec = Eec::new();
        eec.step(truth(), 3.4, 3.4, &EecFaults::default(), 1.0);
        let mut faults = EecFaults::default();
        faults.sensor_a[super::index(Param::Tgt)] = SensorFaults { frozen: 1.0, ..Default::default() };
        let before = eec.step(truth(), 3.4, 3.4, &faults, 0.02).selected[3];
        let hotter = [truth()[0], truth()[1], truth()[2], truth()[3] + 100.0, truth()[4]];
        let after = eec.step(hotter, 3.4, 3.4, &faults, 5.0).selected[3];
        assert!((after - before).abs() < 1e-6);
    }

    #[test]
    fn losing_the_standby_channel_is_reported_even_though_the_other_keeps_control() {
        // A dual-channel EEC running on one channel is an annunciated
        // condition, and which channel died does not change that. With
        // only `active` to go on, a dead channel B was invisible.
        let mut eec = Eec::new();
        let healthy = settle(&mut eec, &EecFaults::default(), 1.0);
        assert!(healthy.channel_a_serviceable && healthy.channel_b_serviceable);

        let mut eec = Eec::new();
        let b_dead = settle(&mut eec, &EecFaults { channel_b_fault: 1.0, ..EecFaults::default() }, 1.0);
        assert_eq!(b_dead.active, ActiveChannel::A, "A still flies the engine");
        assert!(b_dead.channel_a_serviceable);
        assert!(!b_dead.channel_b_serviceable, "but the EEC knows it has lost its standby channel");

        let mut eec = Eec::new();
        let a_dead = settle(&mut eec, &EecFaults { channel_a_fault: 1.0, ..EecFaults::default() }, 1.0);
        assert!(!a_dead.channel_a_serviceable);
        assert!(a_dead.channel_b_serviceable);
    }

    #[test]
    fn a_fuel_flow_disagree_is_flagged_independently_of_the_other_five_parameters() {
        let mut eec = Eec::new();
        let dt = 0.02;
        let mut out = EecState { selected: [0.0; 5], disagree: [false; 5], active: ActiveChannel::A, fuel_flow_disagree: false, channel_a_serviceable: true, channel_b_serviceable: true };
        for _ in 0..150 {
            out = eec.step(truth(), 3.4, 1.0, &EecFaults::default(), dt);
        }
        assert!(out.fuel_flow_disagree);
        assert!(out.disagree.iter().all(|d| !d));
    }

    #[test]
    fn zero_dt_gives_no_nan() {
        let mut eec = Eec::new();
        let s = eec.step(truth(), 3.4, 3.4, &EecFaults::default(), 0.0);
        assert!(!s.selected[0].is_nan());
    }
}
