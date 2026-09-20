//! Environmental hazard coupling: severe-weather and foreign-object events
//! that hit other systems (engine, airframe, electrical, flight controls,
//! landing gear) from the outside, as opposed to internal component wear.
//! Each submodule is a self-contained probability + physics model
//! producing a documented output struct another system's owner can read
//! (or, once wired, publish as a Var); nothing here is wired into the rest
//! of the crate yet (`docs/deep/BRIEF.md`).
//!
//! Backlog, in order: bird strike, lightning, hail, volcanic ash, ice
//! crystal icing, runway contamination, wind shear/turbulence. See
//! `PROGRESS.md` for status, `FAILURES.md` for the fault catalogue in
//! prose, and `registry.rs` for the same faults registered in code
//! against `crate::deep::api::Registry` (`Area::Environment`).

pub mod bird_strike;
pub mod lightning;
pub mod hail;
pub mod volcanic_ash;
pub mod ice_crystal_icing;
pub mod runway_contamination;
pub mod wind_shear;
pub mod weather_cells;
pub mod dispatch;

pub mod registry;

pub mod live;

pub(crate) mod rng;

// Backlog complete; weather_cells.rs and dispatch.rs are the lead's
// follow-up extensions.
