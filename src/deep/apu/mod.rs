//! The APU deep physics model -- PW980A-class, single-spool gas path with a
//! separate load/bleed compressor, full start sequence, oil/fuel/inlet-door
//! systems and a fire interface. See `docs/deep/BRIEF.md` for the shared
//! brief and this directory's `PROGRESS.md`/`FAILURES.md` for what is built
//! and every failure it supports.
//!
//! `registry.rs::register` wires all of this into the unified failure/
//! component/ECAM catalogue (`crate::deep::api`); nothing else in the crate
//! references this module tree yet -- the lead adds `pub mod apu;` to
//! `src/deep/mod.rs` once every area's directory is in place.

pub mod actuator;
pub mod apu;
pub mod combustor;
pub mod compressor_map;
pub mod ecb;
pub mod faults;
pub mod fire;
pub mod fuel_control;
pub mod gas;
pub mod generators;
pub mod governor;
pub mod inlet_door;
pub mod interfaces;
pub mod life;
pub mod load_compressor;
pub mod oil;
pub mod params;
pub mod power_section;
pub mod registry;
pub mod start_envelope;
pub mod starter;
pub mod turbine_flow;

pub use apu::{Apu, Inputs, Outputs};
pub use faults::ApuFaults;
