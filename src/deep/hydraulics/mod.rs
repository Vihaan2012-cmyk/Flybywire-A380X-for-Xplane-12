//! Deep hydraulics: a generic line/volume hydraulic network solver
//! (`network.rs`) plus the A380 green/yellow circuit built from it
//! (`topology.rs`, `pump.rs`, `reservoir.rs`, `accumulator.rs`, `thermal.rs`,
//! `fluid.rs`). See `D:\A380\fbw-xp-systems\docs\deep\BRIEF.md` for the shared
//! rules this directory follows; see this directory's own `PROGRESS.md` and
//! `FAILURES.md` for what has been built and every failure it supports.
//!
//! Self-contained per the brief: nothing here depends on crate internals
//! outside this directory, and nothing outside this directory references it
//! yet (the lead wires it into `deep::mod.rs` once every area is done).

pub mod accumulator;
pub mod fluid;
pub mod live;
pub mod network;
pub mod pump;
pub mod registry;
pub mod reservoir;
pub mod thermal;
pub mod topology;
