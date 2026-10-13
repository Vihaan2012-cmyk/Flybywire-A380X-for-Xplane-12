//! Deep A380 fuel system models: tank geometry/attitude, quantity gauging
//! (FQMS), automatic CG control/transfer sequencing faults, extended fuel
//! temperature/contamination, jettison and leaks. See `docs/deep/BRIEF.md`
//! for the shared brief and this directory's `PROGRESS.md`/`FAILURES.md` for
//! what is built and every failure it supports.
//!
//! Area: `crate::deep::api::Area::Fuel` (19, added to `api.rs` by the lead;
//! this directory's own code and `PROGRESS.md` were written against that
//! name from the start per this task's instructions). ATA 28 throughout.
//!
//! This module tree is self-contained (`docs/deep/BRIEF.md` rule 2): it does
//! not call into `crate::fuel`, `crate::fuel_network`, `crate::fuel_transfer`
//! or `crate::physics::fluids`, even though those already model a real MSFS
//! fuel network, tank temperatures and unporting for the plugin today (read
//! first, cited throughout the modules below). Each module here instead goes
//! one layer deeper than that existing, plugin-wide model at a specific,
//! named gap (per-tank shape instead of one shared aspect ratio, per-probe
//! FQMS failures instead of an ideal totaliser, named/registrable transfer
//! valve and pump faults instead of anonymous numeric indices, FCOC heat and
//! water/wax contamination the existing thermal model does not consume,
//! per-nozzle jettison dynamics instead of one lumped orifice, and a leak
//! detector the existing network has no equivalent of at all) -- nothing
//! here duplicates logic that module already gets right.
//!
//! `registry.rs::register` wires all of this into the unified failure/
//! component/ECAM catalogue (`crate::deep::api`); nothing else in the crate
//! references this module tree yet -- the lead adds `pub mod fuel;` to
//! `src/deep/mod.rs` once every area's directory is in place.

pub mod cg_transfer;
pub mod gauging;
pub mod geometry;
pub mod jettison;
pub mod leak;
pub mod registry;
pub mod thermal;

pub mod live;
