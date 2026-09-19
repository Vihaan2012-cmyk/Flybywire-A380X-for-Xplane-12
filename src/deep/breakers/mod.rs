//! ATA circuit-breaker catalogue: expands the plugin's existing 265-entry
//! `src/breakers.rs` toward the real A380's much larger set, one breaker
//! per modelled electrical load ([`catalog`]'s entries protecting every
//! consumer `deep::electrical::loads.rs` defines, matched by id) plus the
//! A380's other breaker-protected equipment by ATA chapter that has no load
//! model anywhere in this codebase yet (batteries, recorders, oxygen, the
//! APU controller, engine FADEC/ignition, fire-extinguisher squibs, ...),
//! honestly marked as protecting nothing modelled rather than a fabricated
//! gate (`docs/deep/BRIEF.md` hard rule 3).
//!
//! [`trip`] is the physics: a thermal I2t bimetal model with ambient
//! compensation, a magnetic instantaneous trip, and a separate SSPC
//! electronic trip curve with arc-fault detection -- the real A380
//! Electrical Load Management System's split between conventional thermal
//! breakers and solid-state power controllers (remote reset via the CDS),
//! not a single shared curve. Every breaker carries the same two continuous
//! health faults (`trip_calibration_drift` -> nuisance trip,
//! `contact_resistance` -> fails to trip), registered as components/
//! failures in [`registry`].
//!
//! Self-contained per `docs/deep/BRIEF.md` hard rule 2: independently
//! re-derives its own `Bus`/rating/trip-curve types rather than importing
//! `deep::electrical`'s or `crate::breakers`'s -- only the load *ids* are
//! matched (by reading `deep::electrical::loads.rs` in full, not
//! importing), and this directory's own catalogue stands alone. Nothing
//! outside `src/deep/breakers` references this code yet. The one
//! coordinator-requested exception is [`integration_test`], a `cfg(test)`
//! cross-check against `deep::electrical`'s real load ids that only
//! compiles once the lead's build wires both areas in.

pub mod catalog;
pub mod registry;
pub mod trip;

#[cfg(test)]
mod integration_test;
