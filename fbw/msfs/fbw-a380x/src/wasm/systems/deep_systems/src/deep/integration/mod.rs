pub mod fire_ice_adapter;
pub mod sensors_adapter;
pub mod thermal_zones_adapter;
pub mod environment_events_adapter;
pub mod registry;
pub mod weather_model;
pub mod weather_truth {
    pub use super::weather_model::*;
}
pub mod failure_audit;
pub mod provocation;
pub mod consequences;
