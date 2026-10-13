//! Airframe thermal network (deep-systems coupling workstream): a
//! zone-based thermal model spanning the whole A380 airframe -- avionics
//! bays, gear wells, wing leading/trailing-edge compartments, engine
//! pylons/nacelle cowls, the APU compartment, cargo holds, the crown
//! area, cabin decks, the tail cone and the belly fairing pack bays --
//! each a lumped air+structure node connected to its neighbours by
//! conduction and ventilation, and to the outside world by ram/recovery
//! convection, solar load and altitude.
//!
//! This is a **coupling** module (`docs/deep/BRIEF.md`'s "COUPLING
//! agent"): it does not simulate any *source* physics of its own (no
//! engine, no fire, no bleed system, no ECS pack) -- it is the medium
//! those systems' heat and smoke propagate through, and the thing damage
//! (`damage.rs`) reads to degrade whatever else lives in a hot zone.
//! `physics::bays.rs` already does a smaller, five-node version of
//! exactly this idea, coupled directly into `breakers.rs`'s live simvars;
//! this module is deliberately **not** that -- it is a free-standing,
//! self-contained generic network (any number of zones/links, no `Vars`/
//! X-Plane/crate-internal dependency, per this push's hard rule 2) sized
//! for the whole airframe rather than the five bays `bays.rs` already
//! owns. Wiring the two together (or migrating `bays.rs`'s bays onto this
//! network) is integration work for later, not this module's job.
//!
//! Submodules:
//! - [`network`]: the generic `ThermalNetwork` engine -- zones, links,
//!   the heat-source and outside-air/sun interfaces, and the per-tick
//!   solve (with sub-stepping for stiff configurations).
//! - [`smoke`]: the smoke mass-fraction advection helpers `network`'s
//!   ventilation links use, and their own focused unit tests.
//! - [`damage`]: the temperature-limit/damage-rate interface (Montsinger's
//!   rule) that lets any component "living" in a zone be degraded by heat.
//! - [`topology_a380`]: the concrete, GENERIC A380 zone topology --
//!   dimensions, adjacency and typical equipment heat loads, built from
//!   public Airbus dimensions since no AMM zone drawing is public.
//! - [`registry`]: registers this area's failures, components and ECAM
//!   alerts into `crate::deep::api::Registry` (`Area::ThermalZones`).
//!
//! See `FAILURES.md` for every failure this model supports and
//! `PROGRESS.md` for the build log.

pub mod network;
pub mod smoke;
pub mod damage;
pub mod topology_a380;
pub mod registry;
pub mod live;
