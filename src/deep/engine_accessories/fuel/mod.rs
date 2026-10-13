//! Engine fuel system: LP boost pump -> filter -> HP gear pump -> fuel
//! metering unit -> HP shut-off valve -> flow transmitter -> burner
//! manifold and nozzles, in the order fuel actually travels. Each stage is
//! a self-contained module; wiring them together frame-to-frame (LP outlet
//! feeds the filter, filter outlet feeds the HP pump inlet, HP pump
//! delivery feeds the FMU, and so on) is left to whatever future harness
//! wires this directory into the engine physics, per this directory's
//! isolation rule.

pub mod common;
pub mod filter;
pub mod flow_transmitter;
pub mod fmu;
pub mod hp_pump;
pub mod lp_pump;
pub mod manifold;
pub mod shutoff_valve;
pub mod strainer;
