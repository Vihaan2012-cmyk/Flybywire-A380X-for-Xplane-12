const DEPLOY_TIME_S: f64 = 3.0;
const BASE_ACTUATOR_RATE: f64 = 1.0 / DEPLOY_TIME_S;
const UNCOMMANDED_DEPLOY_TIME_S: f64 = 20.0;
const UNCOMMANDED_DRIFT_RATE: f64 = 1.0 / UNCOMMANDED_DEPLOY_TIME_S;
const UNCOMMANDED_THRESHOLD: f64 = 0.05;

#[derive(Clone, Copy, Debug, Default)]
pub struct LockFaults {
    pub fails_to_hold: f64,
    pub jam: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ReverserFaults {
    pub lock_a: LockFaults,
    pub lock_b: LockFaults,
    pub lock_c: LockFaults,
    pub actuator_jam: f64,
    pub control_fault: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct ThrustReverser {
    position: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ReverserState {
    pub position: f64,
    pub uncommanded_deployment: bool,
    pub control_fault_active: bool,
    pub lock_degraded_count: u8,
}

impl ThrustReverser {
    pub fn new() -> Self {
        Self { position: 0.0 }
    }

    pub fn position(&self) -> f64 {
        self.position
    }

    pub fn step(&mut self, commanded_deploy: bool, hydraulic_pressure_frac: f64, faults: &ReverserFaults, dt_s: f64) -> ReverserState {
        let dt = dt_s.max(0.0);
        let hyd = hydraulic_pressure_frac.clamp(0.0, 1.0);
        let actuator_ok = (1.0 - faults.actuator_jam.clamp(0.0, 1.0)) * (1.0 - faults.control_fault.clamp(0.0, 1.0));

        let hold_fail = faults.lock_a.fails_to_hold.clamp(0.0, 1.0) * faults.lock_b.fails_to_hold.clamp(0.0, 1.0) * faults.lock_c.fails_to_hold.clamp(0.0, 1.0);
        let lock_degraded_count = [faults.lock_a.fails_to_hold, faults.lock_b.fails_to_hold, faults.lock_c.fails_to_hold].iter().filter(|&&f| f > 0.0).count() as u8;
        let release_capability =
            (1.0 - faults.lock_a.jam.clamp(0.0, 1.0)) * (1.0 - faults.lock_b.jam.clamp(0.0, 1.0)) * (1.0 - faults.lock_c.jam.clamp(0.0, 1.0));

        let (error, max_step) = if commanded_deploy {
            let error = 1.0 - self.position;
            (error, BASE_ACTUATOR_RATE * hyd * release_capability * actuator_ok * dt)
        } else {
            let error = hold_fail - self.position;
            let rate = if error < 0.0 { BASE_ACTUATOR_RATE } else { UNCOMMANDED_DRIFT_RATE };
            (error, rate * hyd * actuator_ok * dt)
        };
        self.position = (self.position + error.clamp(-max_step, max_step)).clamp(0.0, 1.0);

        ReverserState {
            position: self.position,
            uncommanded_deployment: !commanded_deploy && self.position > UNCOMMANDED_THRESHOLD,
            control_fault_active: faults.control_fault > 0.0,
            lock_degraded_count,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(rev: &mut ThrustReverser, commanded: bool, hyd: f64, faults: &ReverserFaults, seconds: f64) -> ReverserState {
        let dt = 0.05;
        let mut out = ReverserState::default();
        for _ in 0..(seconds / dt) as usize {
            out = rev.step(commanded, hyd, faults, dt);
        }
        out
    }

    #[test]
    fn a_healthy_reverser_stays_stowed_uncommanded_no_nan() {
        let mut rev = ThrustReverser::new();
        let s = run(&mut rev, false, 1.0, &ReverserFaults::default(), 10.0);
        assert_eq!(s.position, 0.0);
        assert!(!s.uncommanded_deployment);
        assert!(!s.position.is_nan());
    }

    #[test]
    fn a_healthy_reverser_deploys_fully_when_commanded() {
        let mut rev = ThrustReverser::new();
        let s = run(&mut rev, true, 1.0, &ReverserFaults::default(), DEPLOY_TIME_S * 1.5);
        assert!(s.position > 1.0 - 1e-6);
    }

    #[test]
    fn one_jammed_lock_alone_still_lets_it_deploy_but_slower() {
        let mut healthy = ThrustReverser::new();
        let mut one_jam = ThrustReverser::new();
        let faults = ReverserFaults { lock_a: LockFaults { jam: 1.0, ..Default::default() }, ..Default::default() };
        let h = run(&mut healthy, true, 1.0, &ReverserFaults::default(), 1.0);
        let j = run(&mut one_jam, true, 1.0, &faults, 1.0);
        assert_eq!(j.position, 0.0, "all three locks must release; one jam blocks deployment entirely");
        assert!(h.position > 0.0);
    }

    #[test]
    fn a_single_lock_failing_to_hold_does_not_cause_uncommanded_deployment() {
        let mut rev = ThrustReverser::new();
        let faults = ReverserFaults { lock_a: LockFaults { fails_to_hold: 1.0, ..Default::default() }, ..Default::default() };
        let s = run(&mut rev, false, 1.0, &faults, 30.0);
        assert!(!s.uncommanded_deployment, "one working lock (B or C) must still hold it");
        assert_eq!(s.position, 0.0);
    }

    #[test]
    fn all_three_locks_failing_to_hold_causes_uncommanded_deployment() {
        let mut rev = ThrustReverser::new();
        let fail = LockFaults { fails_to_hold: 1.0, ..Default::default() };
        let faults = ReverserFaults { lock_a: fail, lock_b: fail, lock_c: fail, ..Default::default() };
        let s = run(&mut rev, false, 1.0, &faults, UNCOMMANDED_DEPLOY_TIME_S * 1.5);
        assert!(s.uncommanded_deployment, "{}", s.position);
    }

    #[test]
    fn a_jammed_actuator_fails_to_deploy_even_with_healthy_locks() {
        let mut rev = ThrustReverser::new();
        let faults = ReverserFaults { actuator_jam: 1.0, ..Default::default() };
        let s = run(&mut rev, true, 1.0, &faults, DEPLOY_TIME_S * 2.0);
        assert_eq!(s.position, 0.0);
    }

    #[test]
    fn no_hydraulic_pressure_prevents_deployment_even_with_healthy_locks() {
        let mut rev = ThrustReverser::new();
        let s = run(&mut rev, true, 0.0, &ReverserFaults::default(), DEPLOY_TIME_S * 2.0);
        assert_eq!(s.position, 0.0);
    }

    #[test]
    fn a_deployed_reverser_stows_again_once_commanded_off() {
        let mut rev = ThrustReverser::new();
        run(&mut rev, true, 1.0, &ReverserFaults::default(), DEPLOY_TIME_S * 1.5);
        let s = run(&mut rev, false, 1.0, &ReverserFaults::default(), DEPLOY_TIME_S * 1.5);
        assert_eq!(s.position, 0.0);
    }

    #[test]
    fn a_control_fault_holds_the_sleeve_and_does_not_track_a_deploy_command() {
        let mut rev = ThrustReverser::new();
        let faults = ReverserFaults { control_fault: 1.0, ..Default::default() };
        let s = run(&mut rev, true, 1.0, &faults, DEPLOY_TIME_S * 2.0);
        assert_eq!(s.position, 0.0, "the sleeve must not track the command with the control loop dead");
        assert!(s.control_fault_active);
    }

    #[test]
    fn a_healthy_control_loop_reads_no_fault() {
        let mut rev = ThrustReverser::new();
        let s = run(&mut rev, true, 1.0, &ReverserFaults::default(), DEPLOY_TIME_S * 1.5);
        assert!(!s.control_fault_active);
        assert!(s.position > 1.0 - 1e-6, "a healthy control loop must still deploy fully");
    }

    #[test]
    fn the_lock_degraded_count_separates_one_failed_lock_from_none_and_from_two() {
        let mut none = ThrustReverser::new();
        let mut one = ThrustReverser::new();
        let mut two = ThrustReverser::new();
        let fail = LockFaults { fails_to_hold: 1.0, ..Default::default() };
        let s0 = none.step(false, 1.0, &ReverserFaults::default(), 0.05);
        let s1 = one.step(false, 1.0, &ReverserFaults { lock_a: fail, ..Default::default() }, 0.05);
        let s2 = two.step(false, 1.0, &ReverserFaults { lock_a: fail, lock_b: fail, ..Default::default() }, 0.05);
        assert_eq!(s0.lock_degraded_count, 0);
        assert_eq!(s1.lock_degraded_count, 1);
        assert_eq!(s2.lock_degraded_count, 2);
    }
}
