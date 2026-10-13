//! Starter air valve (SAV): the pneumatically-actuated butterfly valve that
//! admits bleed air (from a cross-bleed engine, the APU, or a ground cart)
//! to the air turbine starter (`turbine.rs`). Commanded open for the start
//! sequence and closed once the EEC calls for handoff at the self-
//! sustaining speed; like the fuel HP SOV (`../fuel/shutoff_valve.rs`) it
//! is not instantaneous, and the same single "how stuck is it" fault
//! fraction that made that valve's model self-contained does the same job
//! here: a small fraction gives a sluggish valve, a large one a valve that
//! simply does not move, and whether that reads as "stuck open" or "stuck
//! closed" on the real aircraft falls out of whichever position it was
//! commanded away from when it seized, not a separate fault mode.
//!
//! No Trent-900/A380 SAV figures are public; the travel time is
//! **GENERIC**, slower than the fuel HP SOV's because a pneumatic actuator
//! working against a large-diameter bleed duct's flow forces is typically
//! slower than a small hydraulic/electric fuel valve.

/// Full travel time, s (**GENERIC**).
const TRAVEL_TIME_S: f64 = 3.0;

/// Faults the SAV can carry, 0 (healthy) .. 1 (fully failed).
#[derive(Clone, Copy, Debug, Default)]
pub struct AirValveFaults {
    /// Mechanically stuck (moisture-seized actuator, fouled linkage):
    /// scales travel rate toward zero. A small value reads as "valve slow
    /// to respond"; 1.0 reads as "stuck" in whichever position it was in
    /// when the fault reached that level.
    pub stuck: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct AirValve {
    position: f64,
}

impl AirValve {
    pub fn new(start_open: bool) -> Self {
        Self { position: if start_open { 1.0 } else { 0.0 } }
    }

    pub fn position(&self) -> f64 {
        self.position
    }

    /// One step. Returns the actual open fraction (0 closed .. 1 open).
    pub fn step(&mut self, commanded_open: bool, faults: &AirValveFaults, dt_s: f64) -> f64 {
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
    fn a_healthy_valve_opens_fully_within_its_travel_time() {
        let mut v = AirValve::new(false);
        for _ in 0..((TRAVEL_TIME_S * 1.5) / 0.05) as usize {
            v.step(true, &AirValveFaults::default(), 0.05);
        }
        assert!(v.position() > 1.0 - 1e-6);
    }

    #[test]
    fn a_fully_stuck_valve_never_moves() {
        let mut v = AirValve::new(false);
        for _ in 0..500 {
            v.step(true, &AirValveFaults { stuck: 1.0 }, 0.05);
        }
        assert_eq!(v.position(), 0.0, "stuck closed: the valve that never opened for the start");
    }

    #[test]
    fn a_stuck_open_valve_stays_open_when_commanded_shut() {
        let mut v = AirValve::new(true);
        for _ in 0..500 {
            v.step(false, &AirValveFaults { stuck: 1.0 }, 0.05);
        }
        assert_eq!(v.position(), 1.0, "the same fault reads as stuck open depending on the commanded direction");
    }

    #[test]
    fn a_partly_stuck_valve_is_slower_than_healthy_not_frozen() {
        let mut healthy = AirValve::new(false);
        let mut slow = AirValve::new(false);
        healthy.step(true, &AirValveFaults::default(), 1.0);
        slow.step(true, &AirValveFaults { stuck: 0.8 }, 1.0);
        assert!(slow.position() > 0.0 && slow.position() < healthy.position());
    }

    #[test]
    fn zero_dt_gives_no_nan() {
        let mut v = AirValve::new(false);
        let p = v.step(true, &AirValveFaults::default(), 0.0);
        assert!(!p.is_nan());
    }
}
