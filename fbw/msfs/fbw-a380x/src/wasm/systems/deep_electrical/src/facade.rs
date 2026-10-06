use crate::deep::breakers::live::BreakersLive;
use crate::deep::breakers::trip::{Breaker, RemoteControlError};
use crate::deep::electrical::live::ElectricalLive;
use crate::deep::frame::{DerivedFailure, Faults};
use crate::deep::live::{Area, ElectricalInputs};
use crate::deep::wiring::live::WiringLive;

pub use crate::gates::{Gate, GATES};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BreakerCommand {
    Open,
    Close,
}

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
    pub fn new() -> Self {
        let breakers = BreakersLive::new();
        let electrical = ElectricalLive::new();
        let wiring = WiringLive::new();
        Self { breakers, electrical, wiring, derived: Vec::new() }
    }

    pub fn published_names(&self) -> Vec<String> {
        let mut names = Vec::new();
        for area in self.areas() {
            area.publish(&mut |name, _| names.push(name.to_owned()));
        }
        names
    }

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

    pub fn derived_failures(&self) -> &[DerivedFailure] {
        &self.derived
    }

    pub fn unit_count(&self) -> usize {
        self.breakers.units().count()
    }

    pub fn unit_ids(&self) -> Vec<&'static str> {
        self.breakers.units().map(|(id, _, _)| id).collect()
    }

    pub fn unit_index(&self, id: &str) -> Option<usize> {
        self.breakers.units().position(|(u, _, _)| u == id)
    }

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

    pub fn is_open_at(&self, index: usize) -> bool {
        self.breakers.units().nth(index).map(|(_, b, _)| !b.closed).unwrap_or(false)
    }

    pub fn is_open(&self, id: &str) -> bool {
        self.unit_index(id).map(|i| self.is_open_at(i)).unwrap_or(false)
    }

    pub fn current_a_at(&self, index: usize) -> f64 {
        self.breakers.units().nth(index).map(|(_, _, a)| a).unwrap_or(0.0)
    }

    pub fn current_a(&self, id: &str) -> f64 {
        self.unit_index(id).map(|i| self.current_a_at(i)).unwrap_or(0.0)
    }

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

pub fn lvar_key(name: &str) -> String {
    name.chars().map(|c| if c.is_ascii_alphanumeric() { c.to_ascii_uppercase() } else { '_' }).collect()
}
