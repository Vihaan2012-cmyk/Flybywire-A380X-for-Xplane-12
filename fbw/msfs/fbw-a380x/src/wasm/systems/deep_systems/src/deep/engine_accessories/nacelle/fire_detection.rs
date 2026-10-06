pub const TRIP_K: f64 = 273.15 + 450.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Zone {
    Core,
    FanAccessory,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct LoopFaults {
    pub fails_to_detect: f64,
    pub false_trip: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct FireZoneReading {
    pub loop_a_tripped: bool,
    pub loop_b_tripped: bool,
    pub confirmed: bool,
    pub loop_disagree: bool,
}

fn loop_reading(true_temp_k: f64, faults: &LoopFaults) -> bool {
    if faults.false_trip.clamp(0.0, 1.0) >= 0.5 {
        true
    } else if faults.fails_to_detect.clamp(0.0, 1.0) >= 0.5 {
        false
    } else {
        true_temp_k >= TRIP_K
    }
}

pub fn read(zone_true_temp_k: f64, loop_a: &LoopFaults, loop_b: &LoopFaults) -> FireZoneReading {
    let a = loop_reading(zone_true_temp_k, loop_a);
    let b = loop_reading(zone_true_temp_k, loop_b);
    FireZoneReading { loop_a_tripped: a, loop_b_tripped: b, confirmed: a && b, loop_disagree: a != b }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_cool_zone_with_healthy_loops_reads_no_fire_no_nan() {
        let r = read(300.0, &LoopFaults::default(), &LoopFaults::default());
        assert!(!r.confirmed && !r.loop_disagree);
        assert!(!r.loop_a_tripped && !r.loop_b_tripped);
    }

    #[test]
    fn a_hot_zone_above_trip_with_healthy_loops_confirms() {
        let r = read(TRIP_K + 50.0, &LoopFaults::default(), &LoopFaults::default());
        assert!(r.confirmed);
        assert!(!r.loop_disagree);
    }

    #[test]
    fn one_loop_failing_to_detect_alone_does_not_confirm_a_real_fire() {
        let faulty = LoopFaults { fails_to_detect: 1.0, ..Default::default() };
        let r = read(TRIP_K + 50.0, &faulty, &LoopFaults::default());
        assert!(!r.confirmed, "the other loop alone is not enough to confirm -- both must agree");
        assert!(r.loop_disagree, "but the disagreement itself is flagged");
    }

    #[test]
    fn one_loop_false_tripping_alone_does_not_confirm_a_fire_in_a_cool_zone() {
        let faulty = LoopFaults { false_trip: 1.0, ..Default::default() };
        let r = read(300.0, &faulty, &LoopFaults::default());
        assert!(!r.confirmed, "a lone false trip cannot confirm a fire");
        assert!(r.loop_disagree);
    }

    #[test]
    fn both_loops_false_tripping_together_confirms_even_in_a_cool_zone() {
        let faulty = LoopFaults { false_trip: 1.0, ..Default::default() };
        let r = read(300.0, &faulty, &faulty);
        assert!(r.confirmed);
        assert!(!r.loop_disagree);
    }
}
