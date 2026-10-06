//! Thrust reverser: hydraulically (EBHA-backed) actuated translating
//! sleeve, on the A380's two inboard engines only (2 and 3 -- the outboard
//! engines carry no reverser at all, the real A380 design, also reflected
//! in FlyByWire's own model, `fbw-a380x/.../a380_systems/reverser`: "engine
//! 2/3 EBHA" reversers). Guarded by three independent lines of defence
//! against uncommanded deployment, the real reason large transport
//! reversers are built this way: a primary lock restraining the actuator
//! itself, a secondary lock (a hydraulic isolation valve gating supply to
//! the actuator), and a tertiary lock (a mechanical latch on the
//! translating sleeve). Any *one* of the three holding is enough to keep
//! the sleeve stowed (redundant restraint, an OR across the three), but
//! *all three* must release for a legitimate deployment to proceed (a
//! series interlock, an AND across the three) -- which is exactly why an
//! uncommanded deployment needs all three to fail simultaneously in this
//! model, not a single scripted "reverser fault", and why a single jammed
//! lock alone gives a safe failure (fails to deploy), never a dangerous one.
//!
//! No Trent-900/A380 reverser actuation rate or lock design load is public.
//! `BASE_ACTUATOR_RATE` and `UNCOMMANDED_DRIFT_RATE` are **GENERIC**: a
//! healthy reverser deploying in a few seconds (typical of large transport
//! reversers) and an unlocked-but-not-driven sleeve creeping out far more
//! slowly under residual hydraulic/aerodynamic load if (and only if) every
//! lock has actually failed.

/// Full deploy/stow actuation time when healthy and fully powered, s
/// (**GENERIC**, typical of a large transport thrust reverser).
const DEPLOY_TIME_S: f64 = 3.0;
const BASE_ACTUATOR_RATE: f64 = 1.0 / DEPLOY_TIME_S;
/// Sleeve creep rate if every lock has failed to hold and the sleeve is
/// left to residual hydraulic/aerodynamic load with no actuation command,
/// full stroke, s (**GENERIC**, deliberately much slower than a commanded
/// deployment: this path exists only for a triple-simultaneous-failure
/// case, not ordinary operation).
const UNCOMMANDED_DEPLOY_TIME_S: f64 = 20.0;
const UNCOMMANDED_DRIFT_RATE: f64 = 1.0 / UNCOMMANDED_DEPLOY_TIME_S;
/// Sleeve position above which it reads as an uncommanded deployment.
const UNCOMMANDED_THRESHOLD: f64 = 0.05;

/// One lock's fault state, 0 (healthy) .. 1 (fully failed).
#[derive(Clone, Copy, Debug, Default)]
pub struct LockFaults {
    /// Structurally cannot restrain the sleeve even when it should (a
    /// broken spring/worn latch): the safety-critical direction, the one
    /// this module's three-lock redundancy exists to protect against.
    pub fails_to_hold: f64,
    /// Mechanically jammed engaged: cannot release when legitimately
    /// commanded to. The safe-direction failure (fails to deploy), never
    /// itself a cause of uncommanded deployment.
    pub jam: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ReverserFaults {
    pub lock_a: LockFaults,
    pub lock_b: LockFaults,
    pub lock_c: LockFaults,
    /// The actuator ram itself seized, 0 free .. 1 solid: blocks *both*
    /// deploy and stow motion regardless of the locks, the direct
    /// "fails to deploy/stow" fault distinct from the lock logic above.
    pub actuator_jam: f64,
    /// The reverser's own control loop (the EEC-side logic that drives the
    /// actuator from a deploy/stow command), 0 healthy .. 1 dead: distinct
    /// from `actuator_jam` (a physical seizure) -- this is a software/
    /// electrical control fault that leaves the actuator itself free but
    /// unable to be commanded, so the sleeve holds whatever position it
    /// was already at and does not track a new command (`E-ENG-DESIGN.md`
    /// Pattern 24).
    pub control_fault: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct ThrustReverser {
    position: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ReverserState {
    /// 0 fully stowed .. 1 fully deployed.
    pub position: f64,
    pub uncommanded_deployment: bool,
    /// The control loop itself reads faulted this frame
    /// (`ReverserFaults::control_fault > 0`), independent of whether a
    /// deploy is actually being commanded right now -- a real control-loop
    /// self-test is continuous, not only active while a command is given.
    pub control_fault_active: bool,
    /// How many of the three independent locks (A, B, C) are failing to
    /// hold this frame, 0..3 -- the same `fails_to_hold` inputs the
    /// holding-capability calculation below already combines, counted
    /// individually rather than only as their product. `E-ENG-DESIGN.md`
    /// Pattern 28's "one of three" degraded-redundancy case is exactly
    /// `lock_degraded_count == 1`; all three together is Pattern 13's own
    /// `uncommanded_deployment` territory, not this count's job to flag
    /// separately.
    pub lock_degraded_count: u8,
}

impl ThrustReverser {
    pub fn new() -> Self {
        Self { position: 0.0 }
    }

    pub fn position(&self) -> f64 {
        self.position
    }

    /// One step. `commanded_deploy` is the already-validated deploy command
    /// (on ground/engine-running interlocks are the caller's concern, not
    /// modelled here); `hydraulic_pressure_frac` the actuation system's
    /// available pressure fraction (0..1, from whichever hydraulic system
    /// area supplies this reverser -- a documented cross-area input, not
    /// modelled in this directory).
    pub fn step(&mut self, commanded_deploy: bool, hydraulic_pressure_frac: f64, faults: &ReverserFaults, dt_s: f64) -> ReverserState {
        let dt = dt_s.max(0.0);
        let hyd = hydraulic_pressure_frac.clamp(0.0, 1.0);
        // `control_fault` freezes the sleeve exactly as `actuator_jam`
        // does, multiplicatively: at 1.0 the actuator authority is zero and
        // the sleeve holds whatever position it already had, not tracking
        // a new command -- the same functional effect as a seized ram, from
        // a different (control-loop, not physical) cause.
        let actuator_ok = (1.0 - faults.actuator_jam.clamp(0.0, 1.0)) * (1.0 - faults.control_fault.clamp(0.0, 1.0));

        // Holding capability: any one lock holding is enough (OR), so the
        // combined chance of failing to hold needs *all three* to fail.
        let hold_fail = faults.lock_a.fails_to_hold.clamp(0.0, 1.0) * faults.lock_b.fails_to_hold.clamp(0.0, 1.0) * faults.lock_c.fails_to_hold.clamp(0.0, 1.0);
        let lock_degraded_count = [faults.lock_a.fails_to_hold, faults.lock_b.fails_to_hold, faults.lock_c.fails_to_hold].iter().filter(|&&f| f > 0.0).count() as u8;
        // Release capability for a legitimate deployment: every lock must
        // actually retract (AND), so any one jam blocks it.
        let release_capability =
            (1.0 - faults.lock_a.jam.clamp(0.0, 1.0)) * (1.0 - faults.lock_b.jam.clamp(0.0, 1.0)) * (1.0 - faults.lock_c.jam.clamp(0.0, 1.0));

        let (error, max_step) = if commanded_deploy {
            let error = 1.0 - self.position;
            (error, BASE_ACTUATOR_RATE * hyd * release_capability * actuator_ok * dt)
        } else {
            // Not commanded: the equilibrium the sleeve settles to is zero
            // only if the locks can actually hold against it (`hold_fail`
            // near zero); if every lock has failed, there is nothing left
            // to hold it stowed and it settles toward fully deployed
            // instead. Retracting *toward* that equilibrium (the ordinary
            // case, including actively stowing a just-deployed sleeve) uses
            // the actuator's normal authority; creeping *away* from zero
            // because the locks have failed is the slow, residual-load-only
            // path -- the actuator is not being driven open, the locks are
            // simply no longer holding it shut.
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
