//! Nacelle systems: the anti-ice valve protecting the inlet cowl lip,
//! ventilation (heat and flammable-vapour clearance), and the fire/overheat
//! detection sensing element. `fire_detection` is deliberately sense-only:
//! see its module docs for the documented hand-off boundary to the
//! dedicated fire-system agent.

pub mod anti_ice;
pub mod fire_detection;
pub mod ventilation;
