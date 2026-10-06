#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Param {
    N1,
    N2,
    N3,
    Tgt,
    P30,
}
pub const PARAMS: [Param; 5] = [Param::N1, Param::N2, Param::N3, Param::Tgt, Param::P30];

const TRIM_KNEE_UNTRIMMED_C: f64 = 600.0;
const TRIM_MCT_POINT_C: (f64, f64) = (963.0, 850.0);
const TRIM_SLOPE_ABOVE_MCT: f64 = 2.5;

pub fn trimmed_tgt_c(untrimmed_c: f64) -> f64 {
    let (mct_untrimmed, mct_trimmed) = TRIM_MCT_POINT_C;
    if untrimmed_c <= TRIM_KNEE_UNTRIMMED_C {
        untrimmed_c
    } else if untrimmed_c <= mct_untrimmed {
        TRIM_KNEE_UNTRIMMED_C + (untrimmed_c - TRIM_KNEE_UNTRIMMED_C) * (mct_trimmed - TRIM_KNEE_UNTRIMMED_C) / (mct_untrimmed - TRIM_KNEE_UNTRIMMED_C)
    } else {
        mct_trimmed + (untrimmed_c - mct_untrimmed) * TRIM_SLOPE_ABOVE_MCT
    }
}

fn index(p: Param) -> usize {
    match p {
        Param::N1 => 0,
        Param::N2 => 1,
        Param::N3 => 2,
        Param::Tgt => 3,
        Param::P30 => 4,
    }
}

const FULL_SCALE_BIAS: [f64; 5] = [5.0, 5.0, 5.0, 50.0, 3.0e5];
const DISAGREE_THRESHOLD: [f64; 5] = [2.0, 2.0, 2.0, 15.0, 1.5e5];
const SENSOR_TIME_CONSTANT_S: f64 = 0.1;
pub const FUEL_FLOW_DISAGREE_THRESHOLD_KG_S: f64 = 1.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct SensorFaults {
    pub bias: f64,
    pub frozen: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct EecFaults {
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
    frozen_truth: [f64; 5],
}

impl Channel {
    fn new() -> Self {
        Self { held: [0.0; 5], frozen_truth: [0.0; 5] }
    }

    fn step(&mut self, true_values: [f64; 5], faults: &[SensorFaults; 5], dt_s: f64) -> [f64; 5] {
        let dt = dt_s.max(0.0);
        let k = 1.0 - (-dt / SENSOR_TIME_CONSTANT_S).exp();
        for i in 0..5 {
            let frozen = faults[i].frozen.clamp(0.0, 1.0);
            if frozen < 1.0 {
                self.frozen_truth[i] = true_values[i];
            }
            let tracked_truth = true_values[i] * (1.0 - frozen) + self.frozen_truth[i] * frozen;
            let biased_target = tracked_truth + faults[i].bias.clamp(-1.0, 1.0) * FULL_SCALE_BIAS[i];
            self.held[i] += (biased_target - self.held[i]) * k;
        }
        self.held
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ActiveChannel {
    A,
    B,
    None,
}

const SENSOR_PAIR_DEAD_THRESHOLD: f64 = 0.98;

#[derive(Clone, Copy, Debug)]
pub struct EecState {
    pub selected: [f64; 5],
    pub disagree: [bool; 5],
    pub no_data: [bool; 5],
    pub active: ActiveChannel,
    pub fuel_flow_disagree: bool,
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
    pub fn no_valid_data(&self, p: Param) -> bool {
        self.no_data[index(p)]
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
        let mut no_data = [false; 5];
        for i in 0..5 {
            disagree[i] = (reads_a[i] - reads_b[i]).abs() > DISAGREE_THRESHOLD[i];
            no_data[i] = faults.sensor_a[i].frozen.clamp(0.0, 1.0) >= SENSOR_PAIR_DEAD_THRESHOLD
                && faults.sensor_b[i].frozen.clamp(0.0, 1.0) >= SENSOR_PAIR_DEAD_THRESHOLD;
        }

        EecState {
            selected,
            disagree,
            no_data,
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

    #[test]
    fn the_displayed_tgt_meets_every_tcds_trimmed_untrimmed_pair_and_leaves_idle_alone() {
        for (untrimmed, trimmed) in [(963.0, 850.0), (983.0, 900.0), (991.0, 920.0)] {
            assert!((trimmed_tgt_c(untrimmed) - trimmed).abs() < 1e-9, "{untrimmed} -> {}", trimmed_tgt_c(untrimmed));
        }
        assert_eq!(trimmed_tgt_c(348.6), 348.6);
        let mut prev = f64::MIN;
        for t in (0..1100).map(f64::from) {
            let d = trimmed_tgt_c(t);
            assert!(d >= prev, "the trim must never reverse the reading at {t} C");
            prev = d;
        }
    }

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
        let mut eec = Eec::new();
        let mut faults = EecFaults::default();
        faults.sensor_a[super::index(Param::N1)] = SensorFaults { bias: 1.0, ..Default::default() };
        let s = settle(&mut eec, &faults, 3.0);
        assert!((s.value(Param::N1) - (truth()[0] + FULL_SCALE_BIAS[0])).abs() < 0.5);
    }

    #[test]
    fn a_frozen_sensor_stops_updating() {
        let mut eec = Eec::new();
        settle(&mut eec, &EecFaults::default(), 3.0);
        let mut faults = EecFaults::default();
        faults.sensor_a[super::index(Param::Tgt)] = SensorFaults { frozen: 1.0, ..Default::default() };
        let before = settle(&mut eec, &faults, 3.0).selected[3];
        let hotter = [truth()[0], truth()[1], truth()[2], truth()[3] + 100.0, truth()[4]];
        let dt = 0.02;
        let mut after = before;
        for _ in 0..(5.0 / dt) as usize {
            after = eec.step(hotter, 3.4, 3.4, &faults, dt).selected[3];
        }
        assert!((after - before).abs() < 1e-6);
    }

    #[test]
    fn one_dead_tgt_sensor_channel_is_not_no_data() {
        let mut eec = Eec::new();
        let mut faults = EecFaults::default();
        faults.sensor_a[super::index(Param::Tgt)] = SensorFaults { frozen: 1.0, ..Default::default() };
        let s = settle(&mut eec, &faults, 3.0);
        assert!(!s.no_valid_data(Param::Tgt), "the other channel still has a valid reading");
    }

    #[test]
    fn both_dead_tgt_sensor_channels_gives_no_valid_data() {
        let mut eec = Eec::new();
        let mut faults = EecFaults::default();
        faults.sensor_a[super::index(Param::Tgt)] = SensorFaults { frozen: 1.0, ..Default::default() };
        faults.sensor_b[super::index(Param::Tgt)] = SensorFaults { frozen: 1.0, ..Default::default() };
        let s = settle(&mut eec, &faults, 3.0);
        assert!(s.no_valid_data(Param::Tgt));
        assert!(!s.no_valid_data(Param::N1), "only the failed parameter should report no data");
    }

    #[test]
    fn losing_the_standby_channel_is_reported_even_though_the_other_keeps_control() {
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
        let mut out = EecState { selected: [0.0; 5], disagree: [false; 5], no_data: [false; 5], active: ActiveChannel::A, fuel_flow_disagree: false, channel_a_serviceable: true, channel_b_serviceable: true };
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
