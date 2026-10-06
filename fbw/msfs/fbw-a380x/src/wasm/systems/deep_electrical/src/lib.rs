#![allow(dead_code, unused_imports, unused_variables, clippy::all)]

pub mod deep;
mod facade;
mod gates;
pub mod golden;

pub use deep::frame::{DerivedFailure, Faults, PublishedFrame};
pub use deep::live::{ElectricalControls, ElectricalInputs, Environment};
pub use facade::*;
