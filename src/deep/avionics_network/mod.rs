//! The A380's dual-redundant AFDX (ARINC 664 Part 7) avionics data
//! communication network: end systems (CPIOM/IOM), switches, virtual
//! links and the redundant A/B networks (`topology`, `graph`), message
//! delivery with redundancy management, latency, jitter and integrity
//! checking (`message`), the faults that act on every element of that
//! graph (`faults`), the legacy ARINC 429 point-to-point links the network
//! gateways to (`arinc429`), the avionics bay ventilation that keeps its
//! modules from overheating (`ventilation`), and the resulting per-function
//! data availability a consuming system can read (`consequences`).
//!
//! See `D:\fbw-xp-systems\docs\deep\BRIEF.md` for the project-wide rules
//! this module follows (SI units, `Faults` structs default-healthy,
//! std-only, no crate-internal dependencies).

pub mod arinc429;
pub mod consequences;
pub mod faults;
pub mod graph;
pub mod message;
pub mod registry;
pub mod topology;
pub mod ventilation;
