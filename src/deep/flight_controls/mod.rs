//! Deep flight-control-surface physics: per-actuator servo/EHA/EBHA models
//! (`actuator.rs`), aerodynamic hinge moments (`hinge_moment.rs`),
//! control-surface rigid-body integration with actuator/surface faults
//! (`surface.rs`), the flap/slat/droop-nose high-lift transmission
//! (`high_lift.rs`), the trimmable horizontal stabiliser and rudder trim
//! (`ths.rs`), PRIM/SEC-to-actuator allocation (`allocation.rs`), LVDT/RVDT
//! position transducers (`sensors.rs`), spoiler blowdown/load-alleviation/
//! ground-spoiler logic (`spoiler.rs`), the backup control module and its
//! two hydraulically-driven power supplies (`backup.rs`), and the aggregate
//! output a flight model consumes (`output.rs`). See `docs/deep/BRIEF.md`, and this
//! directory's own `PROGRESS.md` and `FAILURES.md`.
//!
//! Almost self-contained: it does not depend on the rest of the crate (std
//! only, f64 throughout) except `registry.rs`, which registers into
//! `crate::deep::api` as the shared brief requires. `deep::live`'s `Area`
//! trait now references this directory's own `live::SurfaceAngles` (the
//! `flight_control_surface_angles` method, added for `deep::integration::
//! flight_control_surfaces::SurfaceOverrideWriter` -- see
//! `docs/deep/integration.md` and `E:/fbw-debug/fixes/W124.md`), which is
//! the one place outside this directory that names a type from inside it;
//! nothing else does.

#![allow(dead_code)]

pub mod actuator;
pub mod hinge_moment;
pub mod live;
pub mod surface;
pub mod high_lift;
pub mod ths;
pub mod allocation;
pub mod backup;
pub mod sensors;
pub mod spoiler;
pub mod output;
pub mod registry;
