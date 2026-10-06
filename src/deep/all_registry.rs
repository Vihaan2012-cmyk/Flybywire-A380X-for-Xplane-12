//! Every area's registration in one place, shared by the X-Plane plugin
//! and the MSFS systems module (`crates/deep_systems`).

/// Every area's failures, components and ECAM alerts in one registry.
pub fn registry() -> super::api::Registry {
    let mut r = super::api::Registry::default();
    super::apu::registry::register(&mut r);
    super::autoflight::registry::register(&mut r);
    super::avionics_network::registry::register(&mut r);
    super::breakers::registry::register(&mut r);
    super::cabin::registry::register(&mut r);
    super::communications::registry::register(&mut r);
    super::electrical::registry::register(&mut r);
    super::engine_accessories::registry::register(&mut r);
    super::environment::registry::register(&mut r);
    super::fire_ice::registry::register(&mut r);
    super::flight_controls::registry::register(&mut r);
    super::fuel::registry::register(&mut r);
    super::gear_structure::registry::register(&mut r);
    super::hydraulics::registry::register(&mut r);
    super::integration::registry::register(&mut r);
    super::oxygen::registry::register(&mut r);
    super::pneumatic_ducts::registry::register(&mut r);
    super::sensors::registry::register(&mut r);
    super::thermal_zones::registry::register(&mut r);
    super::wiring::registry::register(&mut r);
    // One flight-deck warning, one catalogue entry: an area that models a
    // further cause for an alert another area owns, or another side of a
    // component another area owns, registers a contribution instead of a
    // second copy, and this folds them in. Nothing may read the registry
    // before it runs.
    r.resolve();
    r
}
