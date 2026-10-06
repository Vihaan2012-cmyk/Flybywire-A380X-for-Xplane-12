//! Which flight control computer (PRIM 1/2/3, SEC 1/2/3) commands which
//! actuator, and which of a surface's several actuators becomes `Active`
//! (the rest `Damping`) as computers or hydraulic/electric supplies fail.
//!
//! `a380_systems/src/hydraulic/mod.rs` itself carries no such table: it only
//! consumes `*_SOLENOID_ENERGIZED` booleans that something else sets
//! ("energized" -> `PositionControl`, not -> `ActiveDamping`/
//! `ClosedCircuitDamping`). This port's own `prim.rs::update_servo_solenoid_status`
//! (lines 1660-1740) is where those booleans actually come from, one `or(x,
//! y)` per actuator meaning "either computer x or computer y commands this
//! actuator active"; the tables below are a direct transcription of it,
//! with computers indexed the same way `prim.rs` does (`prim_discrete`/
//! `sec_discrete`, `FAILURE_PRIM`/`FAILURE_SEC`, prim.rs:54-59): index 0/1/2
//! = PRIM1/2/3 or SEC1/2/3.
//!
//! Real per-actuator wiring (Green/Yellow/EHA) is cited from
//! `a380_systems/src/hydraulic/mod.rs:2613-2825`.
//!
//! What is *not* visible anywhere in this codebase is the arbitration used
//! *inside* FBW's compiled PrimComputer/SecComputer when more than one of an
//! actuator's listed computers is healthy at once, or the logic that
//! decides which of a panel's two actuators actually drives when both
//! could. The one clue in Rust is `a380_systems/src/hydraulic/mod.rs`'s own
//! `filter_dual_control` (a FIXME whose comment reads "we don't allow dual
//! control for now"): it forces exactly one of a panel's two actuators
//! active, by a fixed priority between them. `mode_for_surface` below
//! reconstructs that shape as a GENERIC priority-list arbitration -- first
//! healthy, powered option in the list wins, everyone else is `Damping` --
//! which is not FBW's real internal logic (inaccessible, compiled C++) but
//! matches the one real behaviour this port can observe: never drive two
//! actuators on the same surface active at once.

use super::actuator::ActuatorMode;

/// One of the six flight control computers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Computer {
    Prim(usize),
    Sec(usize),
}

/// Which hydraulic/electric supply an actuator needs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PowerSource {
    Green,
    Yellow,
    Eha,
}

/// This tick's computer health (true = healthy), indexed like `prim.rs`'s
/// own arrays.
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

/// This tick's hydraulic/electric power availability, as fractions (e.g.
/// from a hydraulic circuit pressure model or an `ElectricMotorPump`); a
/// source counts as usable above 50% of nominal. `Default` is all-zero (no
/// power at all), the "dropout"/"nothing available" case the tests below
/// build off of with `..PowerAvailability::default()`.
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

/// Consecutive ticks a power source must read available (`> 50%`, the same
/// threshold [`PowerAvailability::available`] itself uses) before
/// [`PowerDebounce`] reports it available for *mode selection*. Three ticks
/// is comfortably past `deep::plugin::MAX_DT_S` (0.2 s) worth of a single
/// glitchy tick (a numerically unstable hydraulic solve during pump
/// spin-up, or any other one-frame glitch in the upstream `Truth` reading)
/// while staying well under a second at any real frame rate -- imperceptible
/// next to how long a real battery/APU/engine start actually takes.
const POWER_DEBOUNCE_TICKS: u8 = 3;

/// Debounces one power source's `available()` decision against a
/// single-tick pressure/voltage blip, so a marginal or briefly-wrong
/// reading can never hand an actuator `Active` authority for even one
/// tick. This matters because once an actuator is driven `Active` and then
/// drops back to `Damping`, the surface freezes wherever it got to
/// (`Damping` applies exactly zero torque at zero rate by design -- a
/// genuinely unpowered surface must free-float, not spring back), so a
/// spurious `Active` tick that never gets a chance to servo all the way to
/// the real command leaves a permanent, wrong position behind with no
/// restoring force at zero airspeed.
///
/// Only gates *mode selection* (`mode_for_surface`). The continuous
/// fraction (`PowerAvailability.green`/`yellow`/`eha`) still reaches
/// `FlightControlsLive::pressures` undebounced -- an already-`Active`
/// actuator's rate limit must track real supply pressure instantly, this
/// only guards the one-shot decision to grant `Active` in the first place.
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

    /// This tick's debounced view of `power`, for `mode_for_surface` only.
    pub fn step(&mut self, power: &PowerAvailability) -> PowerAvailability {
        PowerAvailability {
            green: if Self::step_one(&mut self.green_ticks, power.green > 0.5) { power.green } else { 0.0 },
            yellow: if Self::step_one(&mut self.yellow_ticks, power.yellow > 0.5) { power.yellow } else { 0.0 },
            eha: if Self::step_one(&mut self.eha_ticks, power.eha > 0.5) { power.eha } else { 0.0 },
        }
    }
}

/// One actuator's commanding computer(s), in priority order, each paired
/// with the power source it needs. A single-entry list (no backup computer)
/// is real for some actuators -- see `aileron_outboard` below.
#[derive(Clone, Debug)]
pub struct ActuatorAllocation {
    pub options: Vec<(Computer, PowerSource)>,
}

impl ActuatorAllocation {
    fn resolve(&self, health: &ComputerHealth, power: &PowerAvailability) -> Option<Computer> {
        self.options.iter().find(|&&(computer, source)| health.healthy(computer) && power.available(source)).map(|&(c, _)| c)
    }
}

/// Given a surface's actuators in priority order, the mode each should run
/// in this tick and which computer (if any) is driving the winner: the
/// first whose allocation resolves becomes `Active`, every other actuator
/// on the same surface is `Damping` (see module doc comment on
/// `filter_dual_control`).
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

// ---------------------------------------------------------------------------
// Aileron panels: [actuator 0, actuator 1], priority order per panel.
// prim.rs:1666-1688.
// ---------------------------------------------------------------------------

/// Outboard panel: Green actuator PRIM2 only, Yellow actuator PRIM3 only --
/// prim.rs:1666-1673 gives these two solenoids no SEC `or(...)` term at all,
/// unlike every other aileron actuator, so this panel alone has no SEC
/// backup for either of its actuators.
pub fn aileron_outboard() -> [ActuatorAllocation; 2] {
    [alloc(&[(Computer::Prim(1), PowerSource::Green)]), alloc(&[(Computer::Prim(2), PowerSource::Yellow)])]
}

/// Midboard panel: Yellow actuator PRIM3|SEC3, EHA actuator PRIM1|SEC1
/// (prim.rs:1674-1680).
pub fn aileron_midboard() -> [ActuatorAllocation; 2] {
    [
        alloc(&[(Computer::Prim(2), PowerSource::Yellow), (Computer::Sec(2), PowerSource::Yellow)]),
        alloc(&[(Computer::Prim(0), PowerSource::Eha), (Computer::Sec(0), PowerSource::Eha)]),
    ]
}

/// Inboard panel: Green actuator PRIM1|SEC1, EHA actuator PRIM2|SEC2
/// (prim.rs:1681-1688).
pub fn aileron_inboard() -> [ActuatorAllocation; 2] {
    [
        alloc(&[(Computer::Prim(0), PowerSource::Green), (Computer::Sec(0), PowerSource::Green)]),
        alloc(&[(Computer::Prim(1), PowerSource::Eha), (Computer::Sec(1), PowerSource::Eha)]),
    ]
}

// ---------------------------------------------------------------------------
// Elevator panels. prim.rs:1708-1722.
// ---------------------------------------------------------------------------

/// Inboard panel: Green actuator PRIM3|SEC3, EHA actuator PRIM1|SEC1.
pub fn elevator_inboard() -> [ActuatorAllocation; 2] {
    [
        alloc(&[(Computer::Prim(2), PowerSource::Green), (Computer::Sec(2), PowerSource::Green)]),
        alloc(&[(Computer::Prim(0), PowerSource::Eha), (Computer::Sec(0), PowerSource::Eha)]),
    ]
}

/// Outboard panel: Green actuator PRIM1|SEC1, EHA actuator PRIM2|SEC2.
pub fn elevator_outboard() -> [ActuatorAllocation; 2] {
    [
        alloc(&[(Computer::Prim(0), PowerSource::Green), (Computer::Sec(0), PowerSource::Green)]),
        alloc(&[(Computer::Prim(1), PowerSource::Eha), (Computer::Sec(1), PowerSource::Eha)]),
    ]
}

// ---------------------------------------------------------------------------
// Rudder panels (each actuator is EBHA-capable: hydraulic normally,
// electric backup). prim.rs:1730-1740.
// ---------------------------------------------------------------------------

/// Upper rudder: Yellow actuator PRIM1|SEC1, Green actuator PRIM2|SEC2.
pub fn rudder_upper() -> [ActuatorAllocation; 2] {
    [
        alloc(&[(Computer::Prim(0), PowerSource::Yellow), (Computer::Sec(0), PowerSource::Yellow)]),
        alloc(&[(Computer::Prim(1), PowerSource::Green), (Computer::Sec(1), PowerSource::Green)]),
    ]
}

/// Lower rudder: Green actuator PRIM1|SEC1, Yellow actuator PRIM3|SEC3.
pub fn rudder_lower() -> [ActuatorAllocation; 2] {
    [
        alloc(&[(Computer::Prim(0), PowerSource::Green), (Computer::Sec(0), PowerSource::Green)]),
        alloc(&[(Computer::Prim(2), PowerSource::Yellow), (Computer::Sec(2), PowerSource::Yellow)]),
    ]
}

// ---------------------------------------------------------------------------
// THS motors. prim.rs:1725-1727.
// ---------------------------------------------------------------------------

/// Green motor PRIM3|SEC3, Yellow motor PRIM1|SEC1.
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
        health.prim[0] = false; // PRIM1 dead
        let (modes, driving) = mode_for_surface(&aileron_inboard(), &health, &PowerAvailability::all_available());
        assert_eq!(modes[0], ActuatorMode::Active);
        assert_eq!(driving[0], Some(Computer::Sec(0)));
    }

    #[test]
    fn losing_a_hydraulic_circuit_hands_control_to_the_other_actuator() {
        let power = PowerAvailability { green: 0.0, ..PowerAvailability::all_available() };
        let (modes, driving) = mode_for_surface(&aileron_inboard(), &ComputerHealth::all_healthy(), &power);
        // Green actuator (index 0) can't be driven with no green pressure,
        // even with both PRIM1 and SEC1 healthy, so the EHA actuator wins.
        assert_eq!(modes, [ActuatorMode::Damping, ActuatorMode::Active]);
        assert_eq!(driving[1], Some(Computer::Prim(1)));
    }

    #[test]
    fn the_outboard_panel_has_no_sec_backup_unlike_every_other_aileron_actuator() {
        let mut health = ComputerHealth::all_healthy();
        health.prim[1] = false; // PRIM2 dead
        health.sec[1] = false; // SEC2 (irrelevant here, but also dead)
        let (modes, _) = mode_for_surface(&aileron_outboard(), &health, &PowerAvailability::all_available());
        // Its Green actuator (PRIM2-only, no SEC) is stranded even though
        // SEC2 would have covered the *inboard* panel's EHA actuator.
        assert_eq!(modes[0], ActuatorMode::Damping);
        // Its Yellow actuator (PRIM3) is unaffected and still takes over.
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
        // The spike ends; a real, sustained loss of pressure follows.
        let out = debounce.step(&PowerAvailability::default());
        assert!(!out.available(PowerSource::Green));
    }

    #[test]
    fn losing_power_mid_ramp_resets_the_debounce_count() {
        let mut debounce = PowerDebounce::default();
        let up = PowerAvailability { green: 1.0, ..PowerAvailability::default() };
        debounce.step(&up); // 1
        debounce.step(&up); // 2
        debounce.step(&PowerAvailability::default()); // dropout resets to 0
        let out = debounce.step(&up); // 1 again, not 3
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
