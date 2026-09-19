//! The map data FlyByWire's displays draw that comes neither from the DOM
//! nor from the nav database: the terrain on the ND and vertical display,
//! and traffic. Weather radar and the airport moving map's data are not
//! here; docs/map-data.md says why.

pub mod dsf;
pub mod plugin;
pub mod scenery;
pub mod terrain;
pub mod traffic;

pub use plugin::MapData;
