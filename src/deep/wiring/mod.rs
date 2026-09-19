//! Wiring: the physical wire-bundle layer between every circuit's bus/panel
//! and its consumer -- routes, gauge/resistance/insulation, faults (chafe,
//! bundle overheat, connector corrosion, water ingress, rodent/maintenance
//! damage, open wire) and the arc-fault current/heat/breaker-blind-spot
//! model, plus the zone/bundle queries. See `docs/deep/BRIEF.md`'s "wiring"
//! task and this crate's `src/breakers.rs`/`src/circuits.rs` (read for
//! context, not depended on -- every module here is self-contained).

pub mod arc;
pub mod bundle;
pub mod faults;
pub mod gauge;
pub mod query;
pub mod registry;
pub mod routing;
pub mod zones;
