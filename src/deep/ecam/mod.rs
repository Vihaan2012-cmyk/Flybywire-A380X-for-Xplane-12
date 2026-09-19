//! The ECAM bridge: turns every area's registered `deep::api::EcamAlert`s
//! into FlyByWire's own Flight Warning System's native data shapes, and the
//! `SourcePatch`es (see `js_bridge.rs::native_ports`, `ecam_patches.rs`)
//! that splice them into FlyByWire's own compiled JS at load, so alerts
//! appear, sound, sort and resolve inside FlyByWire's real A380 ECAM
//! exactly like its own -- see `docs/deep/ecam_bridge.md` for the full
//! design, FlyByWire's data model, and the worked example.
//!
//! Not wired into the crate tree yet: `src/deep/mod.rs` (owned by the lead,
//! like every other area's directory here) needs one line, `pub mod ecam;`,
//! documented in `docs/deep/ecam_bridge.md`.

pub mod cond_json;
pub mod codegen;
pub mod ids;
#[cfg(feature = "js")]
pub mod patches;

#[cfg(test)]
#[cfg(feature = "js")]
mod tests;
