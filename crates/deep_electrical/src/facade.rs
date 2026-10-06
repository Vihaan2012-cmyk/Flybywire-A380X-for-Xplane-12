//! [`DeepElectrical`]: the three areas stepped as the plugin steps them,
//! plus the crew's commands and the names a host publishes under.

use crate::deep::breakers::live::BreakersLive;
use crate::deep::breakers::trip::{Breaker, RemoteControlError};
use crate::deep::electrical::live::ElectricalLive;
use crate::deep::frame::{DerivedFailure, Faults};
use crate::deep::live::{Area, ElectricalInputs};
use crate::deep::wiring::live::WiringLive;

pub use crate::gates::{Gate, GATES};

/// A crew command to one protection unit, from the EFB's circuit-breaker
/// page (the real aircraft's OIT page) or the overhead RESET panel.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BreakerCommand {
    /// A solid-state unit is commanded open remotely; a thermal breaker is
    /// pulled by hand.
    Open,
    /// A solid-state unit is reset remotely; a thermal breaker is pushed
    /// back in. A unit locked out after repeated trips refuses either.
    Close,
}

/// The electrical network, its protection units and the wiring, stepped
/// together in the plugin's order: breakers, electrical, wiring.
///
/// The three areas hand each other per-breaker currents, open commands and
/// harness damage through a thread-local board (`electrical::live::board`)
/// one frame late, exactly as in X-Plane. That board belongs to whichever
/// `DeepElectrical` was built last on the thread, as it belongs to the last
/// aircraft in the plugin: build one per aircraft.
pub struct DeepElectrical {
    breakers: BreakersLive,
    electrical: ElectricalLive,
    wiring: WiringLive,
    derived: Vec<DerivedFailure>,
}

impl Default for DeepElectrical {
    fn default() -> Self {
        Self::new()
    }
}

impl DeepElectrical {
    /// Cold: every unit closed, every machine stopped.
    pub fn new() -> Self {
        // The plugin's `all_areas()` order: the breakers first, then the
        // electrical area, whose constructor clears the shared board, then
        // the wiring.
        let breakers = BreakersLive::new();
        let electrical = ElectricalLive::new();
        let wiring = WiringLive::new();
        Self { breakers, electrical, wiring, derived: Vec::new() }
    }

    /// Every name [`tick`](Self::tick) publishes, read off the current
    /// state without stepping anything (the areas' `publish` is a pure read
    /// of their own state; the plugin primes its variable table the same
    /// way).
    pub fn published_names(&self) -> Vec<String> {
        let mut names = Vec::new();
        for area in self.areas() {
            area.publish(&mut |name, _| names.push(name.to_owned()));
        }
        names
    }

    /// One frame: step the three areas, collect their verdicts on
    /// FlyByWire's own components, then publish.
    pub fn tick(&mut self, inputs: &ElectricalInputs, faults: &Faults, out: &mut dyn FnMut(&str, f64)) {
        self.breakers.tick(inputs, faults);
        self.electrical.tick(inputs, faults);
        self.wiring.tick(inputs, faults);
        let mut derived = std::mem::take(&mut self.derived);
        derived.clear();
        for area in self.areas() {
            area.derived_failures(&mut |d| {
                if d.magnitude > 0.0 {
                    derived.push(d);
                }
            });
        }
        self.derived = derived;
        for area in self.areas() {
            area.publish(out);
        }
    }

    /// Every FlyByWire failure the areas concluded was real on the last
    /// tick (magnitude above zero only).
    pub fn derived_failures(&self) -> &[DerivedFailure] {
        &self.derived
    }

    /// How many protection units there are.
    pub fn unit_count(&self) -> usize {
        self.breakers.units().count()
    }

    /// Every unit's id, in the order the `*_at` methods index them.
    pub fn unit_ids(&self) -> Vec<&'static str> {
        self.breakers.units().map(|(id, _, _)| id).collect()
    }

    pub fn unit_index(&self, id: &str) -> Option<usize> {
        self.breakers.units().position(|(u, _, _)| u == id)
    }

    /// Apply a crew command to the unit at `index`. Returns whether the unit
    /// accepted it: a unit locked out after repeated trips refuses a close.
    pub fn command_at(&mut self, index: usize, command: BreakerCommand) -> bool {
        let Some(unit) = self.breakers.breaker_at_mut(index) else { return false };
        apply(unit, command)
    }

    pub fn command(&mut self, id: &str, command: BreakerCommand) -> bool {
        match self.unit_index(id) {
            Some(i) => self.command_at(i, command),
            None => false,
        }
    }

    /// Whether the unit at `index` is open, for any reason.
    pub fn is_open_at(&self, index: usize) -> bool {
        self.breakers.units().nth(index).map(|(_, b, _)| !b.closed).unwrap_or(false)
    }

    pub fn is_open(&self, id: &str) -> bool {
        self.unit_index(id).map(|i| self.is_open_at(i)).unwrap_or(false)
    }

    /// The current through the unit at `index` on the last tick, A.
    pub fn current_a_at(&self, index: usize) -> f64 {
        self.breakers.units().nth(index).map(|(_, _, a)| a).unwrap_or(0.0)
    }

    pub fn current_a(&self, id: &str) -> f64 {
        self.unit_index(id).map(|i| self.current_a_at(i)).unwrap_or(0.0)
    }

    /// Every unit: id, trip unit, current on the last tick (A), in index
    /// order. One pass, for a host that reads all of them every frame.
    pub fn units(&self) -> impl Iterator<Item = (&'static str, &Breaker, f64)> + '_ {
        self.breakers.units()
    }

    fn areas(&self) -> [&dyn Area; 3] {
        [&self.breakers, &self.electrical, &self.wiring]
    }
}

fn apply(unit: &mut Breaker, command: BreakerCommand) -> bool {
    match command {
        BreakerCommand::Open => {
            if unit.remote_open() == Err(RemoteControlError::NotRemoteCapable) {
                unit.pull();
            }
            true
        }
        BreakerCommand::Close => match unit.remote_reset() {
            Ok(()) => true,
            Err(RemoteControlError::NotRemoteCapable) => unit.reset().is_ok(),
            Err(RemoteControlError::LockedOut) => false,
        },
    }
}

/// The simulator variable key for a unit id or a published name: every
/// character outside `[A-Za-z0-9]` becomes `_`, then upper case.
/// `fms-1-normal-bkr` becomes `FMS_1_NORMAL_BKR`. The EFB's `lvarKey`
/// (`EFB/Study/catalogue.ts`) is the same rule.
pub fn lvar_key(name: &str) -> String {
    name.chars().map(|c| if c.is_ascii_alphanumeric() { c.to_ascii_uppercase() } else { '_' }).collect()
}
