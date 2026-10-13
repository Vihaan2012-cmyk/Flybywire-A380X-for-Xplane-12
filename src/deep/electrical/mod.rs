//! The A380 per-load electrical network: replaces FlyByWire's 14 lumped
//! per-bus consumers (`a380_systems::power_consumption::A380PowerConsumption`)
//! with an individually modelled load, breaker, contactor, diode, bus and
//! source network (`docs/deep/BRIEF.md`'s electrical workstream).
//!
//! - [`network`]: the data model and per-tick solver (buses, loads,
//!   breakers with a real I^2t/magnetic trip curve, contactors, diodes).
//! - [`loads`]: the load catalogue -- every consumer named in
//!   `D:\xp-systems\src\breakers.rs` plus the major loads it does not
//!   enumerate (galleys, IFE, fuel pumps, lighting feeders, window/probe
//!   heat, avionics computers).
//! - [`sources`]: VFG/APU-generator/TRU/battery/static-inverter/RAT/ground-
//!   power source models and the wiring that feeds them onto the network.
//! - [`shedding`]: galley/commercial load shedding, power budget, and the
//!   bus-transfer transient.
//! - [`registry`]: registers this area's failures, components and ECAM
//!   alerts with `crate::deep::api::Registry` (`Area::Electrical`).

pub mod live;
pub mod loads;
pub mod network;
pub mod registry;
pub mod shedding;
pub mod sources;
