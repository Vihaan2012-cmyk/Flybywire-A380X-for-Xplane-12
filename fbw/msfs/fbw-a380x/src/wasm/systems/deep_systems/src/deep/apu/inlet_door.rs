use super::actuator::Actuator;
use super::params;

#[derive(Clone, Copy, Debug, Default)]
pub struct InletDoorFaults {
    pub jam: f64,
}

#[derive(Clone, Debug)]
pub struct InletDoor {
    actuator: Actuator,
}

impl InletDoor {
    pub fn new() -> Self {
        Self { actuator: Actuator::new(0.0, params::INLET_DOOR_RATE_PER_S) }
    }

    pub fn open_frac(&self) -> f64 {
        self.actuator.position_frac()
    }

    pub fn is_fully_open(&self) -> bool {
        self.open_frac() > 0.99
    }

    pub fn step(&mut self, commanded_open: bool, faults: &InletDoorFaults, dt_s: f64) -> f64 {
        let target = if commanded_open { 1.0 } else { 0.0 };
        self.actuator.step(target, faults.jam, dt_s)
    }

    pub fn pressure_loss_frac(open_frac: f64) -> f64 {
        let closed = (1.0 - open_frac.clamp(0.0, 1.0)).max(0.0);
        (0.5 * closed.powi(2)).min(0.5)
    }
}

impl Default for InletDoor {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn commanding_open_from_closed_takes_time_not_an_instant_jump() {
        let mut door = InletDoor::new();
        let p = door.step(true, &InletDoorFaults::default(), 1.0);
        assert!(p > 0.0 && p < 1.0, "{p}");
    }

    #[test]
    fn given_enough_time_it_reaches_fully_open() {
        let mut door = InletDoor::new();
        for _ in 0..200 {
            door.step(true, &InletDoorFaults::default(), 1.0);
        }
        assert!(door.is_fully_open());
        assert!((InletDoor::pressure_loss_frac(door.open_frac())).abs() < 1e-9);
    }

    #[test]
    fn a_fully_jammed_door_never_opens_and_keeps_its_pressure_loss() {
        let mut door = InletDoor::new();
        for _ in 0..200 {
            door.step(true, &InletDoorFaults { jam: 1.0 }, 1.0);
        }
        assert!(!door.is_fully_open());
        assert!(InletDoor::pressure_loss_frac(door.open_frac()) > 0.0);
    }

    #[test]
    fn a_more_closed_door_imposes_more_pressure_loss() {
        assert!(InletDoor::pressure_loss_frac(0.2) > InletDoor::pressure_loss_frac(0.8));
        assert_eq!(InletDoor::pressure_loss_frac(1.0), 0.0);
    }

    #[test]
    fn pressure_loss_is_always_finite_and_bounded() {
        for f in [-1.0, 0.0, 0.5, 1.0, 2.0] {
            let l = InletDoor::pressure_loss_frac(f);
            assert!(l.is_finite() && (0.0..=0.5).contains(&l), "{f} -> {l}");
        }
    }
}
