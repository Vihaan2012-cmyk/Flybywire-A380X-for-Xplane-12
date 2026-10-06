//! The A380's circuit protection, electrical network and wiring, as one
//! library both simulators run.
//!
//! The physics is the X-Plane plugin's `deep::{breakers, electrical, wiring}`
//! areas, compiled from the plugin's own source files (see `deep/mod.rs`),
//! so X-Plane and MSFS run the same code. [`DeepElectrical`] steps the three
//! the way the plugin's frame loop does; the MSFS host is FlyByWire's
//! `a380_systems::electrical::CircuitProtection`.
#![allow(dead_code, unused_imports, unused_variables, clippy::all)]

pub mod deep;
mod facade;
mod gates;
pub mod golden;

pub use deep::frame::{DerivedFailure, Faults, PublishedFrame};
pub use deep::live::{ElectricalControls, ElectricalInputs, Environment};
pub use facade::*;
