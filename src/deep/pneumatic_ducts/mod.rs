//! Pneumatic ducts: the coupling layer connecting all 4 engines' and the
//! APU's bleed sources through precoolers and a shared cross-bleed manifold
//! to packs, wing anti-ice, engine start and hydraulic reservoir
//! pressurisation -- real compressible flow between real gas volumes, real
//! leaks that remove real mass and dump real heat into the airframe zones
//! a duct run passes through, and a real dual-loop overheat detection
//! system that isolates a faulted duct from the rest of the network.
//!
//! `docs/deep/BRIEF.md`'s backlog, in order:
//! 1. [`duct`] (volumes, compressible flow, insulation loss) + [`network`]
//!    (the actual engine/APU -> manifold -> consumers topology).
//! 2. [`leak`] (per-section leak/rupture, hot jet impingement into the zone).
//! 3. [`odls`] (dual-loop overheat detection, confirm delay, and the
//!    isolation-valve latch it drives, wired in `network.rs`).
//! 4. [`precooler`] (NTU-effectiveness heat exchanger, FAV regulation,
//!    overpressure/overtemperature protection).
//! 5. Faults: distributed across the modules above as each item's own
//!    `...Faults` struct (`duct::DuctSectionFaults`, `odls::OdlsFaults`,
//!    `precooler::PrecoolerFaults`, `network::DuctNetworkFaults` for the
//!    per-instance start check valve), catalogued in full in
//!    [`registry`]/`FAILURES.md`.
//!
//! **Scope decisions** (see `network.rs`'s own module docs for the detail):
//! potable-water pneumatic pressurisation is `deep::cabin::water.rs`'s own
//! self-contained system, not duplicated here; the APU's gas-generator core
//! is `deep::apu::power_section.rs`'s; engine/APU/gear-bay/cargo/avionics
//! *fire* detection is `deep::fire_ice::fire_loops.rs`'s -- ODLS here is
//! the distinct ATA 36 bleed-duct overheat system.
//!
//! Per `docs/deep/BRIEF.md` hard rule 2, this directory is self-contained
//! (plain `f64`, no `uom`, no `systems`/`a380_systems` crate dependency, no
//! import of any other deep-systems directory's internals) and nothing
//! outside it references this module yet.

pub mod duct;
pub mod leak;
pub mod live;
pub mod network;
pub mod odls;
pub mod precooler;
pub mod registry;
