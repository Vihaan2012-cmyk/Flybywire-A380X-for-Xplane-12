//! The ECAM bridge: turns every area's registered `deep::api::EcamAlert`s
//! into FlyByWire's own Flight Warning System's native data shapes, and the
//! `SourcePatch`es (see `js_bridge.rs::native_ports`, `ecam_patches.rs`)
//! that splice them into FlyByWire's own compiled JS at load, so alerts
//! appear, sound, sort and resolve inside FlyByWire's real A380 ECAM
//! exactly like its own -- see `docs/deep/ecam_bridge.md` for the full
//! design, FlyByWire's data model, and the worked example.
//!
//! Two halves, sharing the same `Cond` encoding, the same shim and the same
//! `FwsCore.update()` anchor:
//!
//! * **Our alerts in their system** ([`codegen`], [`ids`]): a
//!   `deep::api::EcamAlert` no FlyByWire procedure exists for gets a
//!   ten-digit id, its title and items merged into
//!   `EcamAbnormalSensedProcedures`, and an `EwdAbnormalItem` built for it.
//! * **Their procedures, given the trigger they never had** ([`fbw`],
//!   [`fbw_codegen`]): FlyByWire defines 1004 abnormal-sensed procedures
//!   and triggers 273 of them; the other 732 are text -- and, because the
//!   ECL's ABN PROC page renders the same table, electronic checklists the
//!   crew can never be shown. These entries carry **only** a trigger and
//!   per-item predicates for FlyByWire's *own* nine-digit id. Nothing of
//!   their text is copied; a second copy would show the crew the same
//!   warning twice. See `docs/deep/fbw_unwired.md`.

pub mod cond_json;
#[cfg(test)]
mod dump_published;
pub mod codegen;
pub mod fbw;
pub mod fbw_codegen;
pub mod ids;
#[cfg(feature = "js")]
pub mod patches;

#[cfg(test)]
mod fbw_tests;

#[cfg(test)]
#[cfg(feature = "js")]
mod tests;
