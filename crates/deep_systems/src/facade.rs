//! What the MSFS systems module drives: every deep area, stepped exactly as
//! the X-Plane plugin steps them (`deep::live::all_areas`), with the
//! breakers area reachable for the EFB's and RESET panel's commands, and
//! the failure catalogue the module accepts armed failures from.

use std::collections::BTreeSet;

use crate::deep::breakers::trip::{Breaker, RemoteControlError};
use crate::deep::frame::{DerivedFailure, Faults};
use crate::deep::live::{all_areas, Deep, Truth};

pub use crate::gates::{Gate, GATES};

/// Every gate: FlyByWire's Rust consumers (`GATES`), then its C++, its
/// JavaScript and MSFS's own systems.
pub fn all_gates() -> impl Iterator<Item = &'static Gate> {
    GATES
        .iter()
        .chain(crate::gates_cpp::GATES_CPP)
        .chain(crate::gates_js::GATES_JS)
        .chain(crate::gates_msfs::GATES_MSFS)
}

/// What the crew asks of a protection unit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BreakerCommand {
    /// A solid-state unit is commanded open remotely; a thermal breaker is
    /// pulled by hand.
    Open,
    /// A solid-state unit is reset remotely; a thermal breaker is pushed
    /// back in. A unit locked out after repeated trips refuses either.
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

    /// Every name the areas publish, in the order they publish them.
    pub fn published_names(&self) -> Vec<String> {
        self.deep.published_names()
    }

    /// One frame: every area steps, then publishes through `out`.
    pub fn tick(&mut self, truth: Truth, faults: &Faults, out: &mut dyn FnMut(&str, f64)) {
        self.deep.tick(truth, faults, out);
    }

    /// Every FlyByWire failure the areas concluded was real on the last
    /// tick (magnitude above zero only).
    pub fn derived_failures(&self) -> &[DerivedFailure] {
        self.deep.derived_failures()
    }

    /// Every protection unit: its id, its breaker, and its current on the
    /// last tick, A.
    pub fn units(&self) -> impl Iterator<Item = (&'static str, &Breaker, f64)> + '_ {
        self.deep.breakers().into_iter().flat_map(|b| b.units())
    }

    pub fn unit_ids(&self) -> Vec<&'static str> {
        self.units().map(|(id, _, _)| id).collect()
    }

    pub fn is_open_at(&self, index: usize) -> bool {
        self.units().nth(index).map(|(_, b, _)| !b.closed).unwrap_or(false)
    }

    /// Opens or resets the unit at `index`; `false` when it refused (a
    /// locked-out unit) or there is no such unit.
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

/// Every failure id the areas register: what the module accepts from the
/// EFB's arming command.
pub fn failure_ids() -> BTreeSet<u64> {
    crate::deep::registry().failures.iter().map(|f| f.id).collect()
}

/// The simulator variable key for a unit id or a published name: every
/// character outside `[A-Za-z0-9]` becomes `_`, then upper case.
/// `fms-1-normal-bkr` becomes `FMS_1_NORMAL_BKR`. The EFB's `lvarKey`
/// (`EFB/Study/catalogue.ts`) is the same rule.
pub fn lvar_key(name: &str) -> String {
    name.chars().map(|c| if c.is_ascii_alphanumeric() { c.to_ascii_uppercase() } else { '_' }).collect()
}
