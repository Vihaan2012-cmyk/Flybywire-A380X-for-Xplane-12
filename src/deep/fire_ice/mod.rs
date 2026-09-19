//! Fire, smoke, extinguishing, ice accretion and anti-ice: `docs/deep/
//! BRIEF.md`'s fire_ice backlog. Every submodule except `registry` is
//! self-contained (BRIEF rule 2: no crate dependency, plain `f64`/`std`)
//! and is exercised entirely by its own `#[cfg(test)]` unit tests; nothing
//! else in the crate references this directory yet. `registry` is the one
//! exception the lead's registration API requires (`crate::deep::api`).
//!
//! - `util`: shared constants/formulas (orifice flow, saturation vapor
//!   pressure, the Messinger surface energy balance, droplet collection
//!   efficiency) reused by the modules below.
//! - `fire_loops`: dual-loop (thermistor + pneumatic) continuous fire/
//!   overheat detection per zone, AND/OR logic, loop-fault vs false-fire
//!   behaviour. Backlog item 1.
//! - `combustion`: per-zone fuel/air-limited combustion and inter-zone heat
//!   spread. Backlog item 2.
//! - `extinguishing`: Halon bottles, squibs, distribution and zone
//!   concentration decay, cross-feed, cargo optical smoke detection and
//!   two-stage suppression, lavatory smoke/fusible-link extinguisher.
//!   Backlog item 3.
//! - `icing`: Messinger water-catch/freezing-fraction ice accretion on
//!   wing, nacelle, probe and windshield surfaces, with aerodynamic
//!   penalty. Backlog item 4.
//! - `anti_ice`: wing/nacelle bleed-air heat balance, probe and window
//!   electrical heat, windshield rain removal, and their faults. Backlog
//!   item 5.
//! - `registry`: registers every failure, component and ECAM alert above
//!   through `crate::deep::api::Registry` (`Area::FireIce`).

pub mod util;

pub mod fire_loops;
pub mod combustion;
pub mod extinguishing;
pub mod icing;
pub mod anti_ice;

pub mod registry;
