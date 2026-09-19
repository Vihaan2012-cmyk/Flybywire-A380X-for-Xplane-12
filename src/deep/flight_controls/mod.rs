//! Deep flight-control-surface physics: per-actuator servo/EHA/EBHA models
//! (`actuator.rs`), aerodynamic hinge moments (`hinge_moment.rs`),
//! control-surface rigid-body integration with actuator/surface faults
//! (`surface.rs`), the flap/slat/droop-nose high-lift transmission
//! (`high_lift.rs`), the trimmable horizontal stabiliser and rudder trim
//! (`ths.rs`), PRIM/SEC-to-actuator allocation (`allocation.rs`), LVDT/RVDT
//! position transducers (`sensors.rs`), spoiler blowdown/load-alleviation/
//! ground-spoiler logic (`spoiler.rs`), and the aggregate output a flight
//! model consumes (`output.rs`). See `docs/deep/BRIEF.md`, and this
//! directory's own `PROGRESS.md` and `FAILURES.md`.
//!
//! Self-contained: nothing outside this directory references it yet, and it
//! does not depend on the rest of the crate (std only, f64 throughout)
//! except `registry.rs`, which registers into `crate::deep::api` as the
//! shared brief requires.

#![allow(dead_code)]

pub mod actuator;
pub mod hinge_moment;
pub mod surface;
pub mod high_lift;
pub mod ths;
pub mod allocation;
pub mod sensors;
pub mod spoiler;
pub mod output;
pub mod registry;
