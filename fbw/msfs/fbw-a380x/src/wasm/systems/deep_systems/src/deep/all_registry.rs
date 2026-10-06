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
    super::ecam::generated_alerts::register(&mut r);
    super::ecam::wave5_w_ata21_1_alerts::register(&mut r);
    super::ecam::wave5_t02_hydraulic_tests_alerts::register(&mut r);
    super::ecam::wave5_f02_fuel_indication_alerts::register(&mut r);
    super::ecam::wave5_w_ata34_1_alerts::register(&mut r);
    super::ecam::wave5_e02_fan_damage_alerts::register(&mut r);
    r.resolve();
    r
}
