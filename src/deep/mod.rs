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
pub mod live;
pub mod oxygen;
/// The plugin's own side of `live`: filling `Truth`, snapshotting `Faults`
/// and turning a published name into a `Vars` write. Kept out of `live`
/// itself so the contract stays free of `crate::Vars`/X-Plane.
pub mod plugin;
pub mod pneumatic_ducts;
pub mod sensors;
pub mod thermal_zones;
/// The weather capability `deep` needs from a host (X-Plane, MSFS),
/// behind a trait -- see the module doc for why this lives here rather
/// than borrowing `crate::xp`'s types.
pub mod weather;
pub mod wiring;

/// Every area's failures, components and ECAM alerts in one registry.
pub fn registry() -> api::Registry {
    let mut r = api::Registry::default();
    apu::registry::register(&mut r);
    avionics_network::registry::register(&mut r);
    breakers::registry::register(&mut r);
    cabin::registry::register(&mut r);
    electrical::registry::register(&mut r);
    engine_accessories::registry::register(&mut r);
    environment::registry::register(&mut r);
    fire_ice::registry::register(&mut r);
    flight_controls::registry::register(&mut r);
    fuel::registry::register(&mut r);
    gear_structure::registry::register(&mut r);
    hydraulics::registry::register(&mut r);
    integration::registry::register(&mut r);
    oxygen::registry::register(&mut r);
    pneumatic_ducts::registry::register(&mut r);
    sensors::registry::register(&mut r);
    thermal_zones::registry::register(&mut r);
    wiring::registry::register(&mut r);
    // One flight-deck warning, one catalogue entry: an area that models a
    // further cause for an alert another area owns, or another side of a
    // component another area owns, registers a contribution instead of a
    // second copy, and this folds them in. Nothing may read the registry
    // before it runs.
    r.resolve();
    r
}

#[cfg(test)]
mod tests {

    #[test]
    fn every_area_registers_together_without_collisions_or_dangling_references() {
        let r = super::registry();
        let errors = r.validate();
        let mut by_area: std::collections::BTreeMap<String, usize> = Default::default();
        for f in &r.failures {
            *by_area.entry(format!("{:?}", f.area)).or_default() += 1;
        }
        println!("DEEP failures {} components {} ECAM alerts {}", r.failures.len(), r.components.len(), r.alerts.len());
        println!("DEEP by area {by_area:?}");
        for e in errors.iter().take(40) {
            println!("DEEP error: {e}");
        }
        assert!(errors.is_empty(), "{} registry errors", errors.len());
    }
}
