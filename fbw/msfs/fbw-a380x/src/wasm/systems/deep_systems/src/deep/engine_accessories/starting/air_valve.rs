const TRAVEL_TIME_S: f64 = 3.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct AirValveFaults {
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
