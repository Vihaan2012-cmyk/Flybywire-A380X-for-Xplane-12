//! `deep::integration` without its X-Plane glue (weather reads, surface
//! writes and X-Plane consequences), which the MSFS host replaces.
#[path = "../../../../../src/deep/integration/fire_ice_adapter.rs"]
pub mod fire_ice_adapter;
#[path = "../../../../../src/deep/integration/sensors_adapter.rs"]
pub mod sensors_adapter;
#[path = "../../../../../src/deep/integration/thermal_zones_adapter.rs"]
pub mod thermal_zones_adapter;
#[path = "../../../../../src/deep/integration/environment_events_adapter.rs"]
pub mod environment_events_adapter;
#[path = "../../../../../src/deep/integration/registry.rs"]
pub mod registry;
#[path = "../../../../../src/deep/integration/weather_model.rs"]
pub mod weather_model;
/// The areas import the environment from `weather_truth`; its host-neutral
/// half is `weather_model`, and the X-Plane reader is left out.
pub mod weather_truth {
    pub use super::weather_model::*;
}
#[path = "../../../../../src/deep/integration/failure_audit.rs"]
pub mod failure_audit;
