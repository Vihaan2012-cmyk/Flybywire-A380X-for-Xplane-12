pub const REFERENCE_AMBIENT_C: f64 = 25.0;

fn thermal_ambient_derate(ambient_c: f64) -> f64 {
    let c = ambient_c.clamp(-55.0, 71.0);
    let above_ref = ((c - REFERENCE_AMBIENT_C) / (71.0 - REFERENCE_AMBIENT_C)).max(0.0);
    1.0 - 0.15 * above_ref
}

const MAGNETIC_TRIP_MULTIPLE: f64 = 10.0;

const MAX_CALIBRATION_DRIFT: f64 = 0.4;

const ARC_FAULT_DI_DT_A_PER_S: f64 = 500.0;

const ARC_FAULT_CONFIRM_S: f64 = 0.1;

const THERMAL_TAU_S: f64 = 20.0;

const SSPC_TAU_S: f64 = 8.0;

const LOCKOUT_TRIP_COUNT: usize = 3;
const LOCKOUT_WINDOW_S: f64 = 300.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BreakerKind {
    Thermal,
    Sspc,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SspcStatus {
    Closed,
    OpenCommanded,
    Tripped(TripCause),
    LockedOut,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RemoteControlError {
    NotRemoteCapable,
    LockedOut,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TripCause {
    None,
    Thermal,
    Magnetic,
    ArcFault,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BreakerFaults {
    pub trip_calibration_drift: f64,
    pub contact_resistance: f64,
}

pub struct Breaker {
    kind: BreakerKind,
    rated_a: f64,
    heat: f64,
    prev_current_a: f64,
    arc_signature_s: f64,
    pub closed: bool,
    pub trip_cause: TripCause,
    elapsed_s: f64,
    trip_times_s: Vec<f64>,
    locked_out: bool,
}

impl Breaker {
    pub fn new(kind: BreakerKind, rated_a: f64) -> Self {
        Self { kind, rated_a, heat: 0.0, prev_current_a: 0.0, arc_signature_s: 0.0, closed: true, trip_cause: TripCause::None, elapsed_s: 0.0, trip_times_s: Vec::new(), locked_out: false }
    }

    pub fn reset(&mut self) -> Result<(), RemoteControlError> {
        if self.locked_out {
            return Err(RemoteControlError::LockedOut);
        }
        self.closed = true;
        self.heat = 0.0;
        self.arc_signature_s = 0.0;
        self.trip_cause = TripCause::None;
        Ok(())
    }

    pub fn remote_reset(&mut self) -> Result<(), RemoteControlError> {
        if self.kind != BreakerKind::Sspc {
            return Err(RemoteControlError::NotRemoteCapable);
        }
        self.reset()
    }

    pub fn remote_open(&mut self) -> Result<(), RemoteControlError> {
        if self.kind != BreakerKind::Sspc {
            return Err(RemoteControlError::NotRemoteCapable);
        }
        self.closed = false;
        Ok(())
    }

    pub fn pull(&mut self) {
        self.closed = false;
    }

    pub fn maintenance_clear_lockout(&mut self) {
        self.locked_out = false;
        self.trip_times_s.clear();
    }

    pub fn is_locked_out(&self) -> bool {
        self.locked_out
    }

    pub fn status(&self) -> SspcStatus {
        if self.locked_out {
            SspcStatus::LockedOut
        } else if self.closed {
            SspcStatus::Closed
        } else if self.trip_cause == TripCause::None {
            SspcStatus::OpenCommanded
        } else {
            SspcStatus::Tripped(self.trip_cause)
        }
    }

    pub fn heat_fraction(&self) -> f64 {
        self.heat
    }

    fn record_trip(&mut self) {
        if self.kind != BreakerKind::Sspc {
            return;
        }
        self.trip_times_s.push(self.elapsed_s);
        let window_start = self.elapsed_s - LOCKOUT_WINDOW_S;
        self.trip_times_s.retain(|&t| t >= window_start);
        if self.trip_times_s.len() >= LOCKOUT_TRIP_COUNT {
            self.locked_out = true;
        }
    }

    fn trip(&mut self, cause: TripCause) -> bool {
        self.closed = false;
        self.trip_cause = cause;
        self.record_trip();
        true
    }

    pub fn minimum_trip_current_a(&self, ambient_c: f64) -> f64 {
        let ambient_factor = if self.kind == BreakerKind::Thermal { thermal_ambient_derate(ambient_c) } else { 1.0 };
        self.rated_a * (1.0 - MAX_CALIBRATION_DRIFT) * ambient_factor
    }

    pub fn step(&mut self, current_a: f64, ambient_c: f64, faults: BreakerFaults, dt_s: f64) -> bool {
        let current_a = current_a.max(0.0);
        let dt = dt_s.max(0.0);
        self.elapsed_s += dt;
        if self.locked_out {
            self.closed = false;
        }
        if !self.closed {
            self.prev_current_a = current_a;
            return false;
        }

        let drift = faults.trip_calibration_drift.clamp(0.0, 1.0);
        let weld = faults.contact_resistance.clamp(0.0, 1.0);

        let ambient_factor = if self.kind == BreakerKind::Thermal { thermal_ambient_derate(ambient_c) } else { 1.0 };
        let effective_rated_a = (self.rated_a * (1.0 - MAX_CALIBRATION_DRIFT * drift) * ambient_factor).max(1e-6);
        let ratio = current_a / effective_rated_a;

        let weld_factor = 1.0 / (1.0 - weld.min(0.999));
        let magnetic_multiple = MAGNETIC_TRIP_MULTIPLE * weld_factor;

        let di_dt = (current_a - self.prev_current_a).abs() / dt.max(1e-6);
        self.prev_current_a = current_a;

        if ratio >= magnetic_multiple {
            return self.trip(TripCause::Magnetic);
        }

        let arc_signature = self.kind == BreakerKind::Sspc && di_dt > ARC_FAULT_DI_DT_A_PER_S && current_a > effective_rated_a * 0.5;
        self.arc_signature_s = if arc_signature { self.arc_signature_s + dt } else { 0.0 };
        if self.arc_signature_s >= ARC_FAULT_CONFIRM_S {
            return self.trip(TripCause::ArcFault);
        }

        let tau_s = if self.kind == BreakerKind::Thermal { THERMAL_TAU_S } else { SSPC_TAU_S };
        let target = ratio * ratio;
        let decay = (-dt / tau_s.max(1e-6)).exp();
        self.heat = (target + (self.heat - target) * decay).max(0.0);

        if self.heat >= weld_factor {
            self.trip(TripCause::Thermal)
        } else {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_healthy_breaker_at_rest_never_trips_and_never_produces_nan() {
        let mut b = Breaker::new(BreakerKind::Thermal, 10.0);
        for _ in 0..1000 {
            let tripped = b.step(0.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.0);
            assert!(!tripped);
            assert!(!b.heat_fraction().is_nan());
        }
        assert!(b.closed);
    }

    #[test]
    fn a_sustained_overload_trips_the_thermal_element_but_a_brief_inrush_does_not() {
        let mut b = Breaker::new(BreakerKind::Thermal, 10.0);
        for _ in 0..50 {
            assert!(!b.step(15.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01));
        }
        assert!(b.closed);
        let mut tripped = false;
        for _ in 0..20_000 {
            if b.step(15.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01) {
                tripped = true;
                break;
            }
        }
        assert!(tripped, "a sustained 1.5x overload never tripped the thermal element");
        assert_eq!(b.trip_cause, TripCause::Thermal);
        assert!(!b.closed);
    }

    #[test]
    fn the_trip_curve_is_monotonic_time_to_trip_falls_as_overload_current_rises() {
        fn time_to_trip(multiple: f64) -> f64 {
            let mut b = Breaker::new(BreakerKind::Thermal, 10.0);
            let mut t = 0.0;
            for _ in 0..200_000 {
                t += 0.01;
                if b.step(10.0 * multiple, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01) {
                    return t;
                }
            }
            f64::INFINITY
        }
        let t_1_2 = time_to_trip(1.2);
        let t_1_5 = time_to_trip(1.5);
        let t_2_0 = time_to_trip(2.0);
        let t_5_0 = time_to_trip(5.0);
        assert!(t_1_2 > t_1_5, "{t_1_2} should be slower to trip than {t_1_5}");
        assert!(t_1_5 > t_2_0, "{t_1_5} should be slower to trip than {t_2_0}");
        assert!(t_2_0 > t_5_0, "{t_2_0} should be slower to trip than {t_5_0}");
    }

    #[test]
    fn magnetic_trip_fires_instantly_on_a_hard_short_regardless_of_thermal_history() {
        let mut b = Breaker::new(BreakerKind::Thermal, 10.0);
        let tripped = b.step(200.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01);
        assert!(tripped);
        assert_eq!(b.trip_cause, TripCause::Magnetic);
    }

    #[test]
    fn calibration_drift_causes_a_nuisance_trip_at_a_current_a_healthy_breaker_would_carry_all_day() {
        let healthy_faults = BreakerFaults::default();
        let drifted_faults = BreakerFaults { trip_calibration_drift: 1.0, contact_resistance: 0.0 };
        let mut healthy = Breaker::new(BreakerKind::Thermal, 10.0);
        let mut drifted = Breaker::new(BreakerKind::Thermal, 10.0);
        for _ in 0..50_000 {
            assert!(!healthy.step(9.0, REFERENCE_AMBIENT_C, healthy_faults, 0.01));
        }
        let mut drifted_tripped = false;
        for _ in 0..50_000 {
            if drifted.step(9.0, REFERENCE_AMBIENT_C, drifted_faults, 0.01) {
                drifted_tripped = true;
                break;
            }
        }
        assert!(drifted_tripped, "full calibration drift should nuisance-trip well below true rated current");
    }

    #[test]
    fn welded_contacts_fail_to_trip_even_on_a_severe_sustained_overload() {
        let welded_faults = BreakerFaults { trip_calibration_drift: 0.0, contact_resistance: 1.0 };
        let mut b = Breaker::new(BreakerKind::Thermal, 10.0);
        for _ in 0..500_000 {
            b.step(30.0, REFERENCE_AMBIENT_C, welded_faults, 0.01);
        }
        assert!(b.closed, "fully welded contacts must not open even under a severe sustained overload");
    }

    #[test]
    fn an_sspc_trips_faster_than_a_thermal_breaker_at_the_same_overload() {
        fn time_to_trip(kind: BreakerKind) -> f64 {
            let mut b = Breaker::new(kind, 10.0);
            let mut t = 0.0;
            for _ in 0..200_000 {
                t += 0.01;
                if b.step(20.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01) {
                    return t;
                }
            }
            f64::INFINITY
        }
        assert!(time_to_trip(BreakerKind::Sspc) < time_to_trip(BreakerKind::Thermal));
    }

    #[test]
    fn an_sspc_trips_on_a_fast_current_step_an_arc_fault_signature_even_below_its_thermal_curve() {
        let mut b = Breaker::new(BreakerKind::Sspc, 10.0);
        assert!(!b.step(2.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01));
        let mut tripped = false;
        for i in 0..1_000 {
            let i_a = if i % 2 == 0 { 9.0 } else { 6.0 };
            if b.step(i_a, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.001) {
                tripped = true;
                break;
            }
        }
        assert!(tripped);
        assert_eq!(b.trip_cause, TripCause::ArcFault);
    }

    #[test]
    fn an_ordinary_load_switching_on_is_not_an_arc_fault() {
        let mut b = Breaker::new(BreakerKind::Sspc, 10.0);
        assert!(!b.step(0.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 1.0 / 30.0));
        assert!(!b.step(20.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 1.0 / 30.0));
        let mut i_a = 20.0;
        for _ in 0..120 {
            i_a = 8.0 + (i_a - 8.0) * 0.5;
            assert!(!b.step(i_a, REFERENCE_AMBIENT_C, BreakerFaults::default(), 1.0 / 30.0));
        }
        assert_eq!(b.trip_cause, TripCause::None);
        assert!(b.closed);
    }

    #[test]
    fn a_thermal_breaker_never_detects_an_arc_fault_it_has_no_such_capability() {
        let mut b = Breaker::new(BreakerKind::Thermal, 10.0);
        assert!(!b.step(2.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01));
        for i in 0..1_000 {
            let i_a = if i % 2 == 0 { 9.0 } else { 6.0 };
            b.step(i_a, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.001);
            assert_ne!(b.trip_cause, TripCause::ArcFault);
        }
    }

    #[test]
    fn ambient_heat_makes_a_thermal_breaker_trip_sooner_than_at_reference_ambient() {
        fn time_to_trip(ambient_c: f64) -> f64 {
            let mut b = Breaker::new(BreakerKind::Thermal, 10.0);
            let mut t = 0.0;
            for _ in 0..500_000 {
                t += 0.01;
                if b.step(13.0, ambient_c, BreakerFaults::default(), 0.01) {
                    return t;
                }
            }
            f64::INFINITY
        }
        assert!(time_to_trip(71.0) < time_to_trip(REFERENCE_AMBIENT_C));
    }

    #[test]
    fn reset_clears_heat_and_trip_cause_and_recloses() {
        let mut b = Breaker::new(BreakerKind::Thermal, 10.0);
        b.step(200.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01);
        assert!(!b.closed);
        b.reset().unwrap();
        assert!(b.closed);
        assert_eq!(b.trip_cause, TripCause::None);
        assert_eq!(b.heat_fraction(), 0.0);
    }

    #[test]
    fn a_thermal_breaker_has_no_remote_control_interface_at_all() {
        let mut b = Breaker::new(BreakerKind::Thermal, 10.0);
        b.step(200.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01);
        assert_eq!(b.remote_reset(), Err(RemoteControlError::NotRemoteCapable));
        assert_eq!(b.remote_open(), Err(RemoteControlError::NotRemoteCapable));
        assert!(b.reset().is_ok());
    }

    #[test]
    fn an_sspc_can_be_remotely_reset_and_remotely_opened_from_the_cds() {
        let mut b = Breaker::new(BreakerKind::Sspc, 10.0);
        b.step(200.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01);
        assert!(!b.closed);
        assert_eq!(b.status(), SspcStatus::Tripped(TripCause::Magnetic));
        b.remote_reset().unwrap();
        assert!(b.closed);
        assert_eq!(b.status(), SspcStatus::Closed);
        b.remote_open().unwrap();
        assert!(!b.closed);
        assert_eq!(b.status(), SspcStatus::OpenCommanded, "commanded open, not tripped");
    }

    #[test]
    fn repeated_trips_within_the_lockout_window_latch_an_sspc_out_and_a_flight_deck_reset_cannot_clear_it() {
        let mut b = Breaker::new(BreakerKind::Sspc, 10.0);
        for i in 0..LOCKOUT_TRIP_COUNT {
            assert!(!b.is_locked_out(), "should not be locked out before trip {i}");
            let tripped = b.step(200.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01);
            assert!(tripped, "trip {i} should have tripped");
            if i + 1 < LOCKOUT_TRIP_COUNT {
                b.remote_reset().unwrap();
            }
        }
        assert!(b.is_locked_out(), "3 trips within the lockout window should latch it out");
        assert_eq!(b.status(), SspcStatus::LockedOut);
        assert_eq!(b.remote_reset(), Err(RemoteControlError::LockedOut));
        assert_eq!(b.reset(), Err(RemoteControlError::LockedOut));
        b.step(0.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01);
        assert!(!b.closed);
    }

    #[test]
    fn maintenance_clear_lockout_lets_a_locked_out_sspc_be_reset_again() {
        let mut b = Breaker::new(BreakerKind::Sspc, 10.0);
        for i in 0..LOCKOUT_TRIP_COUNT {
            b.step(200.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01);
            if i + 1 < LOCKOUT_TRIP_COUNT {
                b.remote_reset().unwrap();
            }
        }
        assert!(b.is_locked_out());
        b.maintenance_clear_lockout();
        assert!(!b.is_locked_out());
        b.remote_reset().unwrap();
        assert!(b.closed);
    }

    #[test]
    fn trips_spaced_outside_the_lockout_window_do_not_accumulate_toward_lockout() {
        let mut b = Breaker::new(BreakerKind::Sspc, 10.0);
        for i in 0..LOCKOUT_TRIP_COUNT {
            b.step(200.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), 0.01);
            assert!(!b.is_locked_out(), "trip {i} alone should never lock it out");
            b.remote_reset().unwrap();
            b.step(0.0, REFERENCE_AMBIENT_C, BreakerFaults::default(), LOCKOUT_WINDOW_S + 1.0);
        }
        assert!(!b.is_locked_out(), "trips spaced outside the window must not latch a lockout");
    }
}
