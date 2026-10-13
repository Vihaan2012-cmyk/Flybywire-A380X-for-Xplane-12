//! HP fuel shut-off valve (HP SOV): the final positive fuel cutoff between
//! the FMU (`fmu.rs`) and the burner manifold (`manifold.rs`), commanded by
//! the master fuel control (engine master switch / fire handle). Modelled
//! as a motor-driven ball or poppet valve with a finite travel time (it is
//! not instantaneous on a real aircraft -- there is a certificated maximum
//! time for fuel flow to stop after selecting an engine master off), gating
//! the metered flow through by its open fraction.
//!
//! No Trent-900 SOV figures are public; the travel time is **GENERIC**,
//! typical of a large turbofan's HP SOV (on the order of a second).

/// Full travel time, open to closed or closed to open, seconds (**GENERIC**).
const TRAVEL_TIME_S: f64 = 1.0;

/// Faults the HP SOV can carry, 0 (healthy) .. 1 (fully failed).
#[derive(Clone, Copy, Debug, Default)]
pub struct ShutoffValveFaults {
    /// Mechanically stuck: scales the actuator's slew rate toward zero, so
    /// the valve responds ever more sluggishly to a command and, fully
    /// stuck, does not move at all -- whatever position it is in when the
    /// fault reaches 1.0 is where it stays (a stuck-open valve cannot be
    /// used to shut an engine down on fire; a stuck-closed one flames the
    /// engine out and cannot be restarted until freed).
    pub stuck: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct ShutoffValve {
    position: f64,
}

impl ShutoffValve {
    /// `start_open`: true starts the valve fully open (engine already
    /// running/starting), false fully closed (engine shut down).
    pub fn new(start_open: bool) -> Self {
        Self { position: if start_open { 1.0 } else { 0.0 } }
    }

    pub fn position(&self) -> f64 {
        self.position
    }

    /// One step. `commanded_open`: true = master/fire handle commands open.
    /// Returns the actual open fraction (0 closed .. 1 open) fuel downstream
    /// must be multiplied by.
    pub fn step(&mut self, commanded_open: bool, faults: &ShutoffValveFaults, dt_s: f64) -> f64 {
        let dt = dt_s.max(0.0);
        let stuck = faults.stuck.clamp(0.0, 1.0);
        let rate = (1.0 / TRAVEL_TIME_S) * (1.0 - stuck);
        let target = if commanded_open { 1.0 } else { 0.0 };
        let max_step = rate * dt;
        let error = target - self.position;
        self.position = (self.position + error.clamp(-max_step, max_step)).clamp(0.0, 1.0);
        self.position
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_healthy_valve_fully_closes_within_its_travel_time() {
        let mut v = ShutoffValve::new(true);
        for _ in 0..((TRAVEL_TIME_S * 1.5) / 0.05) as usize {
            v.step(false, &ShutoffValveFaults::default(), 0.05);
        }
        assert!(v.position() < 1e-6);
    }

    #[test]
    fn a_healthy_valve_fully_opens_within_its_travel_time() {
        let mut v = ShutoffValve::new(false);
        for _ in 0..((TRAVEL_TIME_S * 1.5) / 0.05) as usize {
            v.step(true, &ShutoffValveFaults::default(), 0.05);
        }
        assert!(v.position() > 1.0 - 1e-6);
    }

    #[test]
    fn a_fully_stuck_valve_never_moves() {
        let mut v = ShutoffValve::new(true);
        for _ in 0..200 {
            v.step(false, &ShutoffValveFaults { stuck: 1.0 }, 0.05);
        }
        assert_eq!(v.position(), 1.0);
    }

    #[test]
    fn a_partly_stuck_valve_moves_slower_than_healthy() {
        let mut healthy = ShutoffValve::new(true);
        let mut slow = ShutoffValve::new(true);
        healthy.step(false, &ShutoffValveFaults::default(), 0.3);
        slow.step(false, &ShutoffValveFaults { stuck: 0.7 }, 0.3);
        assert!(slow.position() > healthy.position());
    }

    #[test]
    fn zero_dt_never_produces_nan() {
        let mut v = ShutoffValve::new(true);
        let p = v.step(false, &ShutoffValveFaults::default(), 0.0);
        assert!(!p.is_nan());
    }
}
