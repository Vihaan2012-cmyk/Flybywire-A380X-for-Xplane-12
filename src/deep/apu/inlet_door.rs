//! APU inlet door actuator -- item 4.
//!
//! A motor-driven flap, modelled as a rate-limited actuator (`actuator.rs`)
//! with its own jam fault. An inlet door that fails to reach fully open
//! imposes a genuine inlet total-pressure loss on the compressor (flow
//! separation/blockage around a part-open door), which is what actually
//! limits available shaft power and raises EGT for the same demand
//! (`power_section::Inputs::inlet_pressure_loss_frac`) -- a real physical
//! consequence of the door's own position, not a scripted "door failure"
//! symptom bolted on separately.

use super::actuator::Actuator;
use super::params;

#[derive(Clone, Copy, Debug, Default)]
pub struct InletDoorFaults {
    /// Door actuator seizure, 0 healthy .. 1 fully jammed.
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

    /// Fractional total-pressure loss the compressor's inlet sees for a
    /// given door opening: 0 at fully open, rising as the door closes.
    /// GENERIC shape: most of a flap-type door's flow area is still usable
    /// until it is significantly closed, so the loss grows with the square
    /// of how far shut it is (a gentle initial loss, worsening sharply near
    /// fully closed), capped well short of total blockage since some
    /// leakage flow area remains even nominally "closed".
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
