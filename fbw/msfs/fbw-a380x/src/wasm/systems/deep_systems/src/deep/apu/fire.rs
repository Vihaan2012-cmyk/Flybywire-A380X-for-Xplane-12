#[derive(Clone, Copy, Debug)]
pub struct FireBottle {
    pressure_frac: f64,
    discharging: bool,
}

const DISCHARGE_TIME_CONSTANT_S: f64 = 0.4;

impl FireBottle {
    pub fn new() -> Self {
        Self { pressure_frac: 1.0, discharging: false }
    }

    pub fn pressure_frac(&self) -> f64 {
        self.pressure_frac
    }

    pub fn is_armed(&self) -> bool {
        self.pressure_frac > 0.05
    }

    pub fn step(&mut self, discharge_command: bool, dt_s: f64) {
        if discharge_command && self.is_armed() {
            self.discharging = true;
        }
        if self.discharging {
            self.pressure_frac =
                (self.pressure_frac * (-dt_s.max(0.0) / DISCHARGE_TIME_CONSTANT_S).exp()).max(0.0);
            if self.pressure_frac < 0.01 {
                self.pressure_frac = 0.0;
                self.discharging = false;
            }
        }
    }
}

impl Default for FireBottle {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct FireFaults {
    pub loop_failure: f64,
    pub squib_failure: f64,
}

pub struct Inputs {
    pub fire_loop_detected: bool,
    pub fire_button_pushed: bool,
    pub dt_s: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Outputs {
    pub fire_confirmed: bool,
    pub fuel_shutoff_commanded: bool,
    pub bleed_valve_close_commanded: bool,
    pub bottle_pressure_frac: f64,
    pub bottle_discharged: bool,
}

pub struct FireInterface {
    bottle: FireBottle,
}

impl FireInterface {
    pub fn new() -> Self {
        Self { bottle: FireBottle::new() }
    }

    pub fn step(&mut self, inputs: &Inputs, faults: &FireFaults) -> Outputs {
        let fire_confirmed = inputs.fire_loop_detected && faults.loop_failure < 0.999;
        let discharge_command =
            fire_confirmed && inputs.fire_button_pushed && faults.squib_failure < 0.999;
        self.bottle.step(discharge_command, inputs.dt_s);
        Outputs {
            fire_confirmed,
            fuel_shutoff_commanded: fire_confirmed,
            bleed_valve_close_commanded: fire_confirmed,
            bottle_pressure_frac: self.bottle.pressure_frac(),
            bottle_discharged: self.bottle.pressure_frac() <= 0.0,
        }
    }
}

impl Default for FireInterface {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_fire_no_button_nothing_happens() {
        let mut fi = FireInterface::new();
        let out = fi.step(
            &Inputs { fire_loop_detected: false, fire_button_pushed: false, dt_s: 1.0 },
            &FireFaults::default(),
        );
        assert!(!out.fire_confirmed);
        assert!(!out.fuel_shutoff_commanded);
        assert_eq!(out.bottle_pressure_frac, 1.0);
    }

    #[test]
    fn a_confirmed_fire_alone_commands_shutoff_but_does_not_discharge_without_the_button() {
        let mut fi = FireInterface::new();
        let out = fi.step(
            &Inputs { fire_loop_detected: true, fire_button_pushed: false, dt_s: 1.0 },
            &FireFaults::default(),
        );
        assert!(out.fire_confirmed);
        assert!(out.fuel_shutoff_commanded && out.bleed_valve_close_commanded);
        assert_eq!(out.bottle_pressure_frac, 1.0);
    }

    #[test]
    fn pushing_the_button_on_a_confirmed_fire_discharges_the_bottle_over_time() {
        let mut fi = FireInterface::new();
        let mut out = Outputs::default();
        for _ in 0..20 {
            out = fi.step(
                &Inputs { fire_loop_detected: true, fire_button_pushed: true, dt_s: 0.5 },
                &FireFaults::default(),
            );
        }
        assert!(out.bottle_discharged);
        assert_eq!(out.bottle_pressure_frac, 0.0);
    }

    #[test]
    fn a_spent_bottle_cannot_discharge_again() {
        let mut fi = FireInterface::new();
        for _ in 0..20 {
            fi.step(
                &Inputs { fire_loop_detected: true, fire_button_pushed: true, dt_s: 0.5 },
                &FireFaults::default(),
            );
        }
        let out = fi.step(
            &Inputs { fire_loop_detected: true, fire_button_pushed: true, dt_s: 1.0 },
            &FireFaults::default(),
        );
        assert_eq!(out.bottle_pressure_frac, 0.0);
        assert!(out.bottle_discharged);
    }

    #[test]
    fn a_failed_loop_never_confirms_the_fire_even_if_a_real_one_exists() {
        let mut fi = FireInterface::new();
        let out = fi.step(
            &Inputs { fire_loop_detected: true, fire_button_pushed: true, dt_s: 1.0 },
            &FireFaults { loop_failure: 1.0, squib_failure: 0.0 },
        );
        assert!(!out.fire_confirmed);
        assert!(!out.fuel_shutoff_commanded);
        assert_eq!(out.bottle_pressure_frac, 1.0);
    }

    #[test]
    fn a_failed_squib_confirms_the_fire_but_never_discharges() {
        let mut fi = FireInterface::new();
        let mut out = Outputs::default();
        for _ in 0..20 {
            out = fi.step(
                &Inputs { fire_loop_detected: true, fire_button_pushed: true, dt_s: 0.5 },
                &FireFaults { loop_failure: 0.0, squib_failure: 1.0 },
            );
        }
        assert!(out.fire_confirmed);
        assert!(out.fuel_shutoff_commanded);
        assert_eq!(out.bottle_pressure_frac, 1.0);
        assert!(!out.bottle_discharged);
    }
}
