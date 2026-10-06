use super::actuator::ActuatorMode;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Computer {
    Prim(usize),
    Sec(usize),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PowerSource {
    Green,
    Yellow,
    Eha,
}

#[derive(Clone, Copy, Debug)]
pub struct ComputerHealth {
    pub prim: [bool; 3],
    pub sec: [bool; 3],
}

impl ComputerHealth {
    pub fn all_healthy() -> Self {
        Self { prim: [true; 3], sec: [true; 3] }
    }

    fn healthy(&self, computer: Computer) -> bool {
        match computer {
            Computer::Prim(n) => self.prim[n],
            Computer::Sec(n) => self.sec[n],
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct PowerAvailability {
    pub green: f64,
    pub yellow: f64,
    pub eha: f64,
}

impl PowerAvailability {
    pub fn all_available() -> Self {
        Self { green: 1.0, yellow: 1.0, eha: 1.0 }
    }

    fn available(&self, source: PowerSource) -> bool {
        let frac = match source {
            PowerSource::Green => self.green,
            PowerSource::Yellow => self.yellow,
            PowerSource::Eha => self.eha,
        };
        frac > 0.5
    }
}

const POWER_DEBOUNCE_TICKS: u8 = 3;

#[derive(Clone, Copy, Debug, Default)]
pub struct PowerDebounce {
    green_ticks: u8,
    yellow_ticks: u8,
    eha_ticks: u8,
}

impl PowerDebounce {
    fn step_one(ticks: &mut u8, available_now: bool) -> bool {
        if available_now {
            *ticks = ticks.saturating_add(1);
        } else {
            *ticks = 0;
        }
        *ticks >= POWER_DEBOUNCE_TICKS
    }

    pub fn step(&mut self, power: &PowerAvailability) -> PowerAvailability {
        PowerAvailability {
            green: if Self::step_one(&mut self.green_ticks, power.green > 0.5) { power.green } else { 0.0 },
            yellow: if Self::step_one(&mut self.yellow_ticks, power.yellow > 0.5) { power.yellow } else { 0.0 },
            eha: if Self::step_one(&mut self.eha_ticks, power.eha > 0.5) { power.eha } else { 0.0 },
        }
    }
}

#[derive(Clone, Debug)]
pub struct ActuatorAllocation {
    pub options: Vec<(Computer, PowerSource)>,
}

impl ActuatorAllocation {
    fn resolve(&self, health: &ComputerHealth, power: &PowerAvailability) -> Option<Computer> {
        self.options.iter().find(|&&(computer, source)| health.healthy(computer) && power.available(source)).map(|&(c, _)| c)
    }
}

pub fn mode_for_surface<const N: usize>(
    allocations: &[ActuatorAllocation; N],
    health: &ComputerHealth,
    power: &PowerAvailability,
) -> ([ActuatorMode; N], [Option<Computer>; N]) {
    let mut modes = [ActuatorMode::Damping; N];
    let mut driving = [None; N];
    for i in 0..N {
        if let Some(computer) = allocations[i].resolve(health, power) {
            modes[i] = ActuatorMode::Active;
            driving[i] = Some(computer);
            break;
        }
    }
    (modes, driving)
}

fn alloc(options: &[(Computer, PowerSource)]) -> ActuatorAllocation {
    ActuatorAllocation { options: options.to_vec() }
}

pub fn aileron_outboard() -> [ActuatorAllocation; 2] {
    [alloc(&[(Computer::Prim(1), PowerSource::Green)]), alloc(&[(Computer::Prim(2), PowerSource::Yellow)])]
}

pub fn aileron_midboard() -> [ActuatorAllocation; 2] {
    [
        alloc(&[(Computer::Prim(2), PowerSource::Yellow), (Computer::Sec(2), PowerSource::Yellow)]),
        alloc(&[(Computer::Prim(0), PowerSource::Eha), (Computer::Sec(0), PowerSource::Eha)]),
    ]
}

pub fn aileron_inboard() -> [ActuatorAllocation; 2] {
    [
        alloc(&[(Computer::Prim(0), PowerSource::Green), (Computer::Sec(0), PowerSource::Green)]),
        alloc(&[(Computer::Prim(1), PowerSource::Eha), (Computer::Sec(1), PowerSource::Eha)]),
    ]
}

pub fn elevator_inboard() -> [ActuatorAllocation; 2] {
    [
        alloc(&[(Computer::Prim(2), PowerSource::Green), (Computer::Sec(2), PowerSource::Green)]),
        alloc(&[(Computer::Prim(0), PowerSource::Eha), (Computer::Sec(0), PowerSource::Eha)]),
    ]
}

pub fn elevator_outboard() -> [ActuatorAllocation; 2] {
    [
        alloc(&[(Computer::Prim(0), PowerSource::Green), (Computer::Sec(0), PowerSource::Green)]),
        alloc(&[(Computer::Prim(1), PowerSource::Eha), (Computer::Sec(1), PowerSource::Eha)]),
    ]
}

pub fn rudder_upper() -> [ActuatorAllocation; 2] {
    [
        alloc(&[(Computer::Prim(0), PowerSource::Yellow), (Computer::Sec(0), PowerSource::Yellow)]),
        alloc(&[(Computer::Prim(1), PowerSource::Green), (Computer::Sec(1), PowerSource::Green)]),
    ]
}

pub fn rudder_lower() -> [ActuatorAllocation; 2] {
    [
        alloc(&[(Computer::Prim(0), PowerSource::Green), (Computer::Sec(0), PowerSource::Green)]),
        alloc(&[(Computer::Prim(2), PowerSource::Yellow), (Computer::Sec(2), PowerSource::Yellow)]),
    ]
}

pub fn ths_motors() -> [ActuatorAllocation; 2] {
    [
        alloc(&[(Computer::Prim(2), PowerSource::Green), (Computer::Sec(2), PowerSource::Green)]),
        alloc(&[(Computer::Prim(0), PowerSource::Yellow), (Computer::Sec(0), PowerSource::Yellow)]),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn everything_healthy_the_first_priority_actuator_wins_and_the_rest_damp() {
        let (modes, driving) = mode_for_surface(&aileron_inboard(), &ComputerHealth::all_healthy(), &PowerAvailability::all_available());
        assert_eq!(modes, [ActuatorMode::Active, ActuatorMode::Damping]);
        assert_eq!(driving[0], Some(Computer::Prim(0)));
    }

    #[test]
    fn losing_the_primary_computer_falls_back_to_its_sec() {
        let mut health = ComputerHealth::all_healthy();
        health.prim[0] = false;
        let (modes, driving) = mode_for_surface(&aileron_inboard(), &health, &PowerAvailability::all_available());
        assert_eq!(modes[0], ActuatorMode::Active);
        assert_eq!(driving[0], Some(Computer::Sec(0)));
    }

    #[test]
    fn losing_a_hydraulic_circuit_hands_control_to_the_other_actuator() {
        let power = PowerAvailability { green: 0.0, ..PowerAvailability::all_available() };
        let (modes, driving) = mode_for_surface(&aileron_inboard(), &ComputerHealth::all_healthy(), &power);
        assert_eq!(modes, [ActuatorMode::Damping, ActuatorMode::Active]);
        assert_eq!(driving[1], Some(Computer::Prim(1)));
    }

    #[test]
    fn the_outboard_panel_has_no_sec_backup_unlike_every_other_aileron_actuator() {
        let mut health = ComputerHealth::all_healthy();
        health.prim[1] = false;
        health.sec[1] = false;
        let (modes, _) = mode_for_surface(&aileron_outboard(), &health, &PowerAvailability::all_available());
        assert_eq!(modes[0], ActuatorMode::Damping);
        assert_eq!(modes[1], ActuatorMode::Active);
    }

    #[test]
    fn losing_every_option_leaves_the_actuator_in_damping_not_a_panic() {
        let mut health = ComputerHealth::all_healthy();
        health.prim = [false; 3];
        health.sec = [false; 3];
        let (modes, driving) = mode_for_surface(&elevator_outboard(), &health, &PowerAvailability::all_available());
        assert_eq!(modes, [ActuatorMode::Damping, ActuatorMode::Damping]);
        assert_eq!(driving, [None, None]);
    }

    #[test]
    fn ths_and_rudder_tables_are_well_formed() {
        for allocs in [ths_motors(), rudder_upper(), rudder_lower()] {
            let (modes, _) = mode_for_surface(&allocs, &ComputerHealth::all_healthy(), &PowerAvailability::all_available());
            assert_eq!(modes.iter().filter(|&&m| m == ActuatorMode::Active).count(), 1);
        }
    }

    #[test]
    fn a_single_glitchy_tick_of_pressure_never_reaches_debounced_availability() {
        let mut debounce = PowerDebounce::default();
        let one_tick_spike = PowerAvailability { green: 1.0, yellow: 0.0, eha: 0.0 };
        let out = debounce.step(&one_tick_spike);
        assert!(!out.available(PowerSource::Green), "one glitchy tick must not grant Active authority");
        let out = debounce.step(&PowerAvailability::default());
        assert!(!out.available(PowerSource::Green));
    }

    #[test]
    fn losing_power_mid_ramp_resets_the_debounce_count() {
        let mut debounce = PowerDebounce::default();
        let up = PowerAvailability { green: 1.0, ..PowerAvailability::default() };
        debounce.step(&up);
        debounce.step(&up);
        debounce.step(&PowerAvailability::default());
        let out = debounce.step(&up);
        assert!(!out.available(PowerSource::Green));
    }

    #[test]
    fn sustained_power_becomes_available_after_the_debounce_window() {
        let mut debounce = PowerDebounce::default();
        let up = PowerAvailability { green: 1.0, ..PowerAvailability::default() };
        let mut out = debounce.step(&up);
        for _ in 1..POWER_DEBOUNCE_TICKS {
            out = debounce.step(&up);
        }
        assert!(out.available(PowerSource::Green), "three consecutive healthy ticks must grant availability");
        assert_eq!(out.green, 1.0, "the granted fraction still carries the real pressure, for rate-limit scaling");
    }
}
