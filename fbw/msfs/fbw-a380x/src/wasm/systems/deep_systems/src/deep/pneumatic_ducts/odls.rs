#[derive(Clone, Copy, Debug)]
struct SensingElement {
    element_k: f64,
}
impl SensingElement {
    const TIME_CONSTANT_S: f64 = 2.0;

    fn new(start_k: f64) -> Self {
        Self { element_k: start_k.max(1.0) }
    }
    fn step(&mut self, zone_k: f64, dt_s: f64) -> f64 {
        let dt = dt_s.max(0.0);
        let k = 1.0 / Self::TIME_CONSTANT_S;
        self.element_k = zone_k + (self.element_k - zone_k) * (-k * dt).exp();
        self.element_k
    }
}

enum LoopReading {
    Valid(f64),
    Fault,
}
fn interpret(sensed_k: f64, threshold_abs_k: f64, open: f64, short: f64) -> LoopReading {
    if open.clamp(0.0, 1.0) >= 0.5 {
        LoopReading::Fault
    } else if short.clamp(0.0, 1.0) >= 0.5 {
        LoopReading::Valid(threshold_abs_k + OverheatDetectionLoop::SHORT_PEG_MARGIN_K)
    } else {
        LoopReading::Valid(sensed_k)
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct OdlsFaults {
    pub loop_a_open: f64,
    pub loop_a_short: f64,
    pub loop_b_open: f64,
    pub loop_b_short: f64,
    pub false_detection: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct OdlsOutputs {
    pub trip: bool,
    pub loop_fault: bool,
    pub loop_a_fault: bool,
    pub loop_b_fault: bool,
    pub confirming_s: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct OverheatDetectionLoop {
    loop_a: SensingElement,
    loop_b: SensingElement,
    confirming_s: f64,
    threshold_abs_k: f64,
}
impl OverheatDetectionLoop {
    pub const THRESHOLD_WING_FUSELAGE_K: f64 = 124.0 + 273.15;
    pub const THRESHOLD_PYLON_STRUT_K: f64 = 200.0 + 273.15;
    pub const CONFIRM_TIME_S: f64 = 3.0;
    const SHORT_PEG_MARGIN_K: f64 = 50.0;
    const FALSE_DETECTION_FULL_K: f64 = 300.0;

    pub fn new(start_k: f64, threshold_abs_k: f64) -> Self {
        Self { loop_a: SensingElement::new(start_k), loop_b: SensingElement::new(start_k), confirming_s: 0.0, threshold_abs_k }
    }

    pub fn step(&mut self, zone_air_k: f64, dt_s: f64, faults: &OdlsFaults) -> OdlsOutputs {
        let a_sensed = self.loop_a.step(zone_air_k, dt_s);
        let b_sensed = self.loop_b.step(zone_air_k, dt_s);
        let a = interpret(a_sensed, self.threshold_abs_k, faults.loop_a_open, faults.loop_a_short);
        let b = interpret(b_sensed, self.threshold_abs_k, faults.loop_b_open, faults.loop_b_short);

        let loop_a_fault = matches!(a, LoopReading::Fault);
        let loop_b_fault = matches!(b, LoopReading::Fault);

        let mut valid_readings: Vec<f64> = Vec::with_capacity(2);
        for r in [a, b] {
            if let LoopReading::Valid(v) = r {
                valid_readings.push(v);
            }
        }
        let loop_fault = valid_readings.is_empty();
        debug_assert_eq!(loop_fault, loop_a_fault && loop_b_fault);

        let hottest = valid_readings.iter().cloned().fold(f64::MIN, f64::max);
        let false_bump = faults.false_detection.clamp(0.0, 1.0) * Self::FALSE_DETECTION_FULL_K;
        let effective_reading = if loop_fault { self.threshold_abs_k - 1.0 + false_bump } else { hottest + false_bump };

        let over_threshold = effective_reading > self.threshold_abs_k;
        self.confirming_s = if over_threshold { self.confirming_s + dt_s.max(0.0) } else { 0.0 };

        OdlsOutputs {
            trip: self.confirming_s >= Self::CONFIRM_TIME_S,
            loop_fault,
            loop_a_fault,
            loop_b_fault,
            confirming_s: self.confirming_s,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const T: f64 = OverheatDetectionLoop::THRESHOLD_WING_FUSELAGE_K;

    fn run_for(zone_k: f64, threshold_abs_k: f64, faults: &OdlsFaults, seconds: f64) -> OdlsOutputs {
        let mut o = OverheatDetectionLoop::new(288.0, threshold_abs_k);
        let dt = 0.5;
        let mut out = OdlsOutputs::default();
        let mut t = 0.0;
        while t < seconds {
            out = o.step(zone_k, dt, faults);
            t += dt;
        }
        out
    }

    #[test]
    fn a_healthy_zone_never_trips() {
        let out = run_for(300.0, T, &OdlsFaults::default(), 600.0);
        assert!(!out.trip);
        assert!(!out.loop_fault);
        assert!(!out.loop_a_fault);
        assert!(!out.loop_b_fault);
    }

    #[test]
    fn a_sustained_overheat_trips_after_the_confirm_delay_not_instantly() {
        let mut o = OverheatDetectionLoop::new(288.0, T);
        let faults = OdlsFaults::default();
        let hot = T + 20.0;
        let first = o.step(hot, 0.1, &faults);
        assert!(!first.trip);
        let mut out = first;
        for _ in 0..200 {
            out = o.step(hot, 0.1, &faults);
        }
        assert!(out.trip, "a sustained real overheat must eventually trip");
    }

    #[test]
    fn a_brief_transient_does_not_trip() {
        let mut o = OverheatDetectionLoop::new(288.0, T);
        let faults = OdlsFaults::default();
        let hot = T + 50.0;
        let out = o.step(hot, OverheatDetectionLoop::CONFIRM_TIME_S * 0.3, &faults);
        assert!(!out.trip, "a transient shorter than the confirm delay must not trip");
    }

    #[test]
    fn both_loops_open_reports_a_fault_with_no_trip_from_a_healthy_zone() {
        let faults = OdlsFaults { loop_a_open: 1.0, loop_b_open: 1.0, ..Default::default() };
        let out = run_for(300.0, T, &faults, 60.0);
        assert!(out.loop_fault);
        assert!(out.loop_a_fault);
        assert!(out.loop_b_fault);
        assert!(!out.trip);
    }

    #[test]
    fn a_single_open_loop_reports_on_its_own_without_masking_or_tripping() {
        let faults = OdlsFaults { loop_a_open: 1.0, ..Default::default() };
        let out = run_for(300.0, T, &faults, 60.0);
        assert!(out.loop_a_fault, "loop A's own open failure must be visible");
        assert!(!out.loop_b_fault, "loop B is healthy");
        assert!(!out.loop_fault, "one surviving loop is not a system fault");
        assert!(!out.trip, "a healthy zone with one open loop must not trip");
    }

    #[test]
    fn one_loop_shorted_alone_still_trips_even_with_the_other_loop_open() {
        let faults = OdlsFaults { loop_a_open: 1.0, loop_b_short: 1.0, ..Default::default() };
        let out = run_for(288.0, T, &faults, 60.0);
        assert!(out.trip, "a shorted loop must be able to trip on its own");
        assert!(!out.loop_fault, "one loop still provides a (if false) reading");
    }

    #[test]
    fn false_detection_alone_trips_a_cold_zone() {
        let faults = OdlsFaults { false_detection: 1.0, ..Default::default() };
        let out = run_for(218.0, OverheatDetectionLoop::THRESHOLD_PYLON_STRUT_K, &faults, 60.0);
        assert!(out.trip);
    }

    #[test]
    fn the_same_real_temperature_trips_one_compartment_class_but_not_the_other() {
        let hot_for_wing_not_pylon = OverheatDetectionLoop::THRESHOLD_WING_FUSELAGE_K + 10.0;
        let wing = run_for(hot_for_wing_not_pylon, OverheatDetectionLoop::THRESHOLD_WING_FUSELAGE_K, &OdlsFaults::default(), 60.0);
        let pylon = run_for(hot_for_wing_not_pylon, OverheatDetectionLoop::THRESHOLD_PYLON_STRUT_K, &OdlsFaults::default(), 60.0);
        assert!(wing.trip, "past the wing/fuselage threshold must trip a wing/fuselage-class loop");
        assert!(!pylon.trip, "the same temperature is normal for a pylon/strut-class bay and must not trip it");
    }

    #[test]
    fn no_nan_at_dt_zero() {
        let mut o = OverheatDetectionLoop::new(288.0, T);
        let out = o.step(500.0, 0.0, &OdlsFaults::default());
        assert!(!out.trip);
        assert_eq!(out.confirming_s, 0.0);
    }
}
