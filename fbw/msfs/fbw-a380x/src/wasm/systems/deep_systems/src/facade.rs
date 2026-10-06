use std::collections::BTreeSet;

use crate::deep::breakers::trip::{Breaker, RemoteControlError};
use crate::deep::frame::{DerivedFailure, Faults};
use crate::deep::live::{all_areas, Deep, Truth};

pub use crate::gates::{Gate, GATES};

pub fn all_gates() -> impl Iterator<Item = &'static Gate> {
    GATES
        .iter()
        .chain(crate::gates_cpp::GATES_CPP)
        .chain(crate::gates_js::GATES_JS)
        .chain(crate::gates_msfs::GATES_MSFS)
        .chain(crate::gates_wave3::GATES_WAVE3)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BreakerCommand {
    Open,
    Close,
}

pub struct DeepSystems {
    deep: Deep,
}

impl Default for DeepSystems {
    fn default() -> Self {
        Self::new()
    }
}

impl DeepSystems {
    pub fn new() -> Self {
        Self { deep: all_areas() }
    }

    pub fn published_names(&self) -> Vec<String> {
        self.deep.published_names()
    }

    pub fn tick(&mut self, truth: Truth, faults: &Faults, out: &mut dyn FnMut(&str, f64)) {
        self.deep.tick(truth, faults, out);
    }

    pub fn tick_timed(
        &mut self,
        truth: Truth,
        faults: &Faults,
        out: &mut dyn FnMut(&str, f64),
    ) -> Vec<(&'static str, std::time::Duration, std::time::Duration)> {
        self.deep.tick_timed(truth, faults, out)
    }

    pub fn derived_failures(&self) -> &[DerivedFailure] {
        self.deep.derived_failures()
    }

    pub fn units(&self) -> impl Iterator<Item = (&'static str, &Breaker, f64)> + '_ {
        self.deep.breakers().into_iter().flat_map(|b| b.units())
    }

    pub fn unit_ids(&self) -> Vec<&'static str> {
        self.units().map(|(id, _, _)| id).collect()
    }

    pub fn is_open_at(&self, index: usize) -> bool {
        self.units().nth(index).map(|(_, b, _)| !b.closed).unwrap_or(false)
    }

    pub fn command_at(&mut self, index: usize, command: BreakerCommand) -> bool {
        let Some(unit) = self.deep.breakers_mut().and_then(|b| b.breaker_at_mut(index)) else { return false };
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
}

pub fn failure_ids() -> BTreeSet<u64> {
    crate::deep::registry()
        .failures
        .iter()
        .map(|f| f.id)
        .filter(|&id| !crate::msfs_excluded::is_msfs_excluded(id))
        .collect()
}

pub fn lvar_key(name: &str) -> String {
    name.chars().map(|c| if c.is_ascii_alphanumeric() { c.to_ascii_uppercase() } else { '_' }).collect()
}
