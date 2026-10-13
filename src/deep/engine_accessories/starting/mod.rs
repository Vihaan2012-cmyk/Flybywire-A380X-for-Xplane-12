//! Engine starting: the starter air valve, the air turbine starter and its
//! sprag clutch, and the starter's own duty-cycle heating -- everything
//! between the bleed-air source and the point `physics::engine::starter`'s
//! own torque model (the lead's gas-path rebuild, not touched here) takes
//! over as the causal driver of HP spool speed during a start.

pub mod air_valve;
pub mod duty_cycle;
pub mod turbine;
