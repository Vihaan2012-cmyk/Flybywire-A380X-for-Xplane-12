//! The deep-systems push: new physical models, each area in its own
//! directory, all registering through `api`.

pub mod api;
pub mod apu;
pub mod avionics_network;
pub mod breakers;
pub mod cabin;
pub mod ecam;
pub mod electrical;
pub mod engine_accessories;
pub mod environment;
pub mod fire_ice;
pub mod flight_controls;
pub mod fuel;
pub mod gear_structure;
pub mod hydraulics;
pub mod integration;
pub mod pneumatic_ducts;
pub mod sensors;
pub mod thermal_zones;
pub mod wiring;
