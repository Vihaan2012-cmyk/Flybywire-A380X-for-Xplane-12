//! ATA 22 -- Auto Flight: the FCU (AFS control panel), its two MFD backup
//! pages, the TCAS/AP mode arbitration fault, and the approach-capability
//! downgrade verdict. New with the ECAM-completeness pass
//! (`E:/fbw-debug/ecam/E-AIR-DESIGN.md`'s ATA 22 section) -- no `deep::`
//! area modelled the autoflight control panel before this, and (unlike
//! ATA 21) `a380_systems` does not own this domain either: the real
//! flight-warning/autoflight logic lives in the systems-host TypeScript
//! (`FwsCore`/`FwsAbnormalSensed`), out of scope to change (`no FlyByWire
//! TypeScript changes`). Self-contained per `docs/deep/BRIEF.md` hard rule
//! 2: reads only `Truth` fields other areas already publish or bridge
//! (`prim_healthy`, `radio_height_ft`), never another area's private state.

pub mod live;
pub mod registry;
