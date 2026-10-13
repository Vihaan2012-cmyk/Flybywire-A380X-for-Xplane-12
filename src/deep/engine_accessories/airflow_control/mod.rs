//! Compressor airflow control: the IP compressor's variable stator vanes
//! and the IP/HP handling (surge) bleed valves -- the two mechanisms a
//! three-spool axial-flow engine uses to keep every compressor stage
//! working at a sensible incidence and mass flow across its whole speed
//! range, rather than being sized for one point and stalling everywhere
//! else. Neither module touches the compressor map itself
//! (`physics::engine::compressor`, the lead's, never edited here); each
//! exposes a documented stall-margin/mass-flow output for that model to
//! consume.

pub mod bleed_valve;
pub mod vsv;
