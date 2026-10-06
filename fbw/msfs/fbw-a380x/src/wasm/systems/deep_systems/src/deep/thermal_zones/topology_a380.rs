use super::damage::ThermalDamageRegistry;
use super::network::{ThermalNetwork, Zone, ZoneId, ZoneRef};

pub struct A380Zones {
    pub main_avionics: ZoneId,
    pub upper_avionics: ZoneId,
    pub aft_avionics: ZoneId,
    pub nose_gear_well: ZoneId,
    pub wing_gear_well: ZoneId,
    pub body_gear_well: ZoneId,
    pub wing_le_left: ZoneId,
    pub wing_le_right: ZoneId,
    pub wing_te_left: ZoneId,
    pub wing_te_right: ZoneId,
    pub pylon: [ZoneId; 4],
    pub nacelle_cowl: [ZoneId; 4],
    pub apu_compartment: ZoneId,
    pub cargo_fwd: ZoneId,
    pub cargo_aft: ZoneId,
    pub cargo_bulk: ZoneId,
    pub crown_area: ZoneId,
    pub cabin_main_deck: ZoneId,
    pub cabin_upper_deck: ZoneId,
    pub tail_cone: ZoneId,
    pub belly_fairing_packs: ZoneId,
    pub fwd_lower_crew_rest: ZoneId,
}

pub struct A380VentLinks {
    pub main_avionics_fan: usize,
    pub upper_avionics_fan: usize,
    pub cargo_fwd_fan: usize,
    pub cargo_aft_fan: usize,
    pub cargo_bulk_fan: usize,
    pub nose_gear_door: usize,
    pub wing_gear_door: usize,
    pub body_gear_door: usize,
    pub nacelle_vent: [usize; 4],
    pub pylon_vent: [usize; 4],
    pub apu_compartment_vent: usize,
    pub belly_pack_bay_vent: usize,
    pub tail_cone_vent: usize,
    pub aft_avionics_fan: usize,
    pub fwd_lower_crew_rest_fan: usize,
}

pub struct A380Thermal {
    pub network: ThermalNetwork,
    pub zones: A380Zones,
    pub vents: A380VentLinks,
    pub damage: ThermalDamageRegistry,
}

const INITIAL_TEMP_C: f64 = 22.0;

pub fn build() -> A380Thermal {
    let mut net = ThermalNetwork::new();

    let main_avionics = net.add_zone(Zone::new("MainAvionics", 12.0, 5.0e5, 150.0, 4.0, 0.0, 3000.0, INITIAL_TEMP_C));
    let upper_avionics = net.add_zone(Zone::new("UpperAvionics", 6.0, 3.0e5, 100.0, 3.0, 0.0, 2000.0, INITIAL_TEMP_C));

    let nose_gear_well = net.add_zone(Zone::new("NoseGearWell", 8.0, 2.0e5, 60.0, 6.0, 0.0, 0.0, INITIAL_TEMP_C));
    let wing_gear_well = net.add_zone(Zone::new("WingGearWell", 15.0, 4.0e5, 80.0, 10.0, 0.0, 0.0, INITIAL_TEMP_C));
    let body_gear_well = net.add_zone(Zone::new("BodyGearWell", 15.0, 4.0e5, 80.0, 8.0, 0.0, 0.0, INITIAL_TEMP_C));

    let wing_le_left = net.add_zone(Zone::new("WingLeLeft", 25.0, 6.0e5, 40.0, 40.0, 0.6, 0.0, INITIAL_TEMP_C));
    let wing_le_right = net.add_zone(Zone::new("WingLeRight", 25.0, 6.0e5, 40.0, 40.0, 0.6, 0.0, INITIAL_TEMP_C));
    let wing_te_left = net.add_zone(Zone::new("WingTeLeft", 22.0, 5.5e5, 35.0, 35.0, 0.4, 0.0, INITIAL_TEMP_C));
    let wing_te_right = net.add_zone(Zone::new("WingTeRight", 22.0, 5.5e5, 35.0, 35.0, 0.4, 0.0, INITIAL_TEMP_C));

    let pylon = [
        net.add_zone(Zone::new("PylonEngine1", 5.0, 2.0e5, 50.0, 10.0, 0.3, 200.0, INITIAL_TEMP_C)),
        net.add_zone(Zone::new("PylonEngine2", 5.0, 2.0e5, 50.0, 10.0, 0.3, 200.0, INITIAL_TEMP_C)),
        net.add_zone(Zone::new("PylonEngine3", 5.0, 2.0e5, 50.0, 10.0, 0.3, 200.0, INITIAL_TEMP_C)),
        net.add_zone(Zone::new("PylonEngine4", 5.0, 2.0e5, 50.0, 10.0, 0.3, 200.0, INITIAL_TEMP_C)),
    ];
    let nacelle_cowl = [
        net.add_zone(Zone::new("NacelleCowl1", 8.0, 1.5e5, 60.0, 15.0, 0.2, 0.0, INITIAL_TEMP_C)),
        net.add_zone(Zone::new("NacelleCowl2", 8.0, 1.5e5, 60.0, 15.0, 0.2, 0.0, INITIAL_TEMP_C)),
        net.add_zone(Zone::new("NacelleCowl3", 8.0, 1.5e5, 60.0, 15.0, 0.2, 0.0, INITIAL_TEMP_C)),
        net.add_zone(Zone::new("NacelleCowl4", 8.0, 1.5e5, 60.0, 15.0, 0.2, 0.0, INITIAL_TEMP_C)),
    ];

    let apu_compartment = net.add_zone(Zone::new("ApuCompartment", 6.0, 2.0e5, 70.0, 8.0, 0.1, 0.0, INITIAL_TEMP_C));

    let aft_avionics = net.add_zone(Zone::new("AftAvionics", 6.0, 3.0e5, 100.0, 3.0, 0.0, 2000.0, INITIAL_TEMP_C));

    let cargo_fwd = net.add_zone(Zone::new("CargoFwd", 110.0, 5.0e5, 120.0, 30.0, 0.0, 50.0, INITIAL_TEMP_C));
    let cargo_aft = net.add_zone(Zone::new("CargoAft", 60.0, 3.0e5, 80.0, 20.0, 0.0, 50.0, INITIAL_TEMP_C));
    let cargo_bulk = net.add_zone(Zone::new("CargoBulk", 14.0, 1.0e5, 30.0, 8.0, 0.0, 20.0, INITIAL_TEMP_C));

    let fwd_lower_crew_rest = net.add_zone(Zone::new("FwdLowerCrewRest", 20.0, 1.5e5, 40.0, 0.0, 0.0, 100.0, INITIAL_TEMP_C));

    let crown_area = net.add_zone(Zone::new("CrownArea", 40.0, 3.0e5, 30.0, 60.0, 0.8, 0.0, INITIAL_TEMP_C));

    let cabin_main_deck = net.add_zone(Zone::new("CabinMainDeck", 700.0, 1.0e6, 500.0, 0.0, 0.0, 0.0, INITIAL_TEMP_C));
    let cabin_upper_deck = net.add_zone(Zone::new("CabinUpperDeck", 350.0, 5.0e5, 300.0, 0.0, 0.0, 0.0, INITIAL_TEMP_C));

    let tail_cone = net.add_zone(Zone::new("TailCone", 25.0, 2.0e5, 40.0, 20.0, 0.3, 0.0, INITIAL_TEMP_C));

    let belly_fairing_packs = net.add_zone(Zone::new("BellyFairingPacks", 30.0, 2.0e5, 60.0, 25.0, 0.0, 1500.0, INITIAL_TEMP_C));

    net.add_conduction_link(main_avionics, upper_avionics, 40.0);
    net.add_conduction_link(nose_gear_well, main_avionics, 30.0);
    net.add_conduction_link(nose_gear_well, cargo_fwd, 20.0);
    net.add_conduction_link(main_avionics, cargo_fwd, 20.0);
    net.add_conduction_link(wing_gear_well, wing_le_left, 20.0);
    net.add_conduction_link(wing_gear_well, wing_le_right, 20.0);
    net.add_conduction_link(pylon[0], wing_te_left, 20.0);
    net.add_conduction_link(pylon[1], wing_te_left, 20.0);
    net.add_conduction_link(pylon[2], wing_te_right, 20.0);
    net.add_conduction_link(pylon[3], wing_te_right, 20.0);
    net.add_conduction_link(nacelle_cowl[0], pylon[0], 30.0);
    net.add_conduction_link(nacelle_cowl[1], pylon[1], 30.0);
    net.add_conduction_link(nacelle_cowl[2], pylon[2], 30.0);
    net.add_conduction_link(nacelle_cowl[3], pylon[3], 30.0);
    net.add_conduction_link(body_gear_well, cargo_aft, 25.0);
    net.add_conduction_link(body_gear_well, tail_cone, 15.0);
    net.add_conduction_link(cargo_fwd, belly_fairing_packs, 25.0);
    net.add_conduction_link(cargo_aft, belly_fairing_packs, 20.0);
    net.add_conduction_link(crown_area, cabin_upper_deck, 150.0);
    net.add_conduction_link(cabin_upper_deck, cabin_main_deck, 300.0);
    net.add_conduction_link(cabin_main_deck, cargo_fwd, 80.0);
    net.add_conduction_link(cabin_main_deck, cargo_aft, 80.0);
    net.add_conduction_link(cabin_main_deck, cargo_bulk, 40.0);
    net.add_conduction_link(tail_cone, apu_compartment, 50.0);
    net.add_conduction_link(tail_cone, cargo_aft, 20.0);
    net.add_conduction_link(aft_avionics, tail_cone, 20.0);
    net.add_conduction_link(aft_avionics, apu_compartment, 20.0);
    net.add_conduction_link(fwd_lower_crew_rest, cargo_fwd, 20.0);

    let main_avionics_fan = net.add_ventilation_link(main_avionics, ZoneRef::Zone(cabin_main_deck), 0.35);
    let upper_avionics_fan = net.add_ventilation_link(upper_avionics, ZoneRef::Zone(cabin_main_deck), 0.25);
    let cargo_fwd_fan = net.add_ventilation_link(cargo_fwd, ZoneRef::Zone(cabin_main_deck), 0.30);
    let cargo_aft_fan = net.add_ventilation_link(cargo_aft, ZoneRef::Zone(cabin_main_deck), 0.30);
    let cargo_bulk_fan = net.add_ventilation_link(cargo_bulk, ZoneRef::Zone(cabin_main_deck), 0.10);

    let nose_gear_door = net.add_ventilation_link(nose_gear_well, ZoneRef::OutsideAir, 1.0);
    let wing_gear_door = net.add_ventilation_link(wing_gear_well, ZoneRef::OutsideAir, 1.2);
    let body_gear_door = net.add_ventilation_link(body_gear_well, ZoneRef::OutsideAir, 1.2);
    net.set_ventilation_health(nose_gear_door, 0.0);
    net.set_ventilation_health(wing_gear_door, 0.0);
    net.set_ventilation_health(body_gear_door, 0.0);

    let nacelle_vent = [
        net.add_ventilation_link(nacelle_cowl[0], ZoneRef::OutsideAir, 3.0),
        net.add_ventilation_link(nacelle_cowl[1], ZoneRef::OutsideAir, 3.0),
        net.add_ventilation_link(nacelle_cowl[2], ZoneRef::OutsideAir, 3.0),
        net.add_ventilation_link(nacelle_cowl[3], ZoneRef::OutsideAir, 3.0),
    ];
    let pylon_vent = [
        net.add_ventilation_link(pylon[0], ZoneRef::OutsideAir, 0.5),
        net.add_ventilation_link(pylon[1], ZoneRef::OutsideAir, 0.5),
        net.add_ventilation_link(pylon[2], ZoneRef::OutsideAir, 0.5),
        net.add_ventilation_link(pylon[3], ZoneRef::OutsideAir, 0.5),
    ];
    let apu_compartment_vent = net.add_ventilation_link(apu_compartment, ZoneRef::OutsideAir, 1.5);
    let belly_pack_bay_vent = net.add_ventilation_link(belly_fairing_packs, ZoneRef::OutsideAir, 4.0);
    let tail_cone_vent = net.add_ventilation_link(tail_cone, ZoneRef::OutsideAir, 0.3);
    let aft_avionics_fan = net.add_ventilation_link(aft_avionics, ZoneRef::OutsideAir, 0.35);
    let fwd_lower_crew_rest_fan = net.add_ventilation_link(fwd_lower_crew_rest, ZoneRef::Zone(cabin_main_deck), 0.10);

    let mut damage = ThermalDamageRegistry::new();
    damage.register("MainAvionicsWiringBundle", main_avionics, 70.0, 1.0 / 1800.0);
    damage.register("NoseGearWellHydraulicHose", nose_gear_well, 135.0, 1.0 / 3600.0);
    damage.register("WingLeLeftAntiIceDuctInsulation", wing_le_left, 150.0, 1.0 / 1800.0);
    damage.register("NacelleCowl1Wiring", nacelle_cowl[0], 125.0, 1.0 / 1800.0);
    damage.register("BellyFairingPackDuctSeal", belly_fairing_packs, 200.0, 1.0 / 1800.0);

    A380Thermal {
        network: net,
        zones: A380Zones {
            main_avionics,
            upper_avionics,
            aft_avionics,
            nose_gear_well,
            wing_gear_well,
            body_gear_well,
            wing_le_left,
            wing_le_right,
            wing_te_left,
            wing_te_right,
            pylon,
            nacelle_cowl,
            apu_compartment,
            cargo_fwd,
            cargo_aft,
            cargo_bulk,
            crown_area,
            cabin_main_deck,
            cabin_upper_deck,
            tail_cone,
            belly_fairing_packs,
            fwd_lower_crew_rest,
        },
        vents: A380VentLinks {
            main_avionics_fan,
            upper_avionics_fan,
            cargo_fwd_fan,
            cargo_aft_fan,
            cargo_bulk_fan,
            nose_gear_door,
            wing_gear_door,
            body_gear_door,
            nacelle_vent,
            pylon_vent,
            apu_compartment_vent,
            belly_pack_bay_vent,
            tail_cone_vent,
            aft_avionics_fan,
            fwd_lower_crew_rest_fan,
        },
        damage,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::thermal_zones::network::OutsideAir;

    fn ground_air() -> OutsideAir {
        OutsideAir { static_temp_c: 15.0, mach: 0.0, true_airspeed_m_s: 0.0 }
    }

    #[test]
    fn builds_every_zone_at_a_sane_initial_temperature() {
        let a380 = build();
        for z in &a380.network.zones {
            assert!((z.air_temp_c - INITIAL_TEMP_C).abs() < 1e-9, "{} did not start at the standard initial temperature", z.name);
            assert!(z.air_mass_kg > 0.0, "{} has non-positive air mass", z.name);
            assert!(z.structure_thermal_mass_j_per_k > 0.0, "{} has non-positive structure mass", z.name);
        }
    }

    #[test]
    fn zone_count_matches_the_named_topology() {
        let a380 = build();
        assert_eq!(a380.network.zones.len(), 28);
    }

    #[test]
    fn no_nan_after_running_the_whole_network_at_rest() {
        let mut a380 = build();
        let outside = ground_air();
        for _ in 0..2000 {
            a380.network.step(1.0, &outside, 0.0);
            a380.damage.update(&a380.network, 1.0);
        }
        for z in &a380.network.zones {
            assert!(z.air_temp_c.is_finite(), "{} air temp went non-finite", z.name);
            assert!(z.structure_temp_c.is_finite(), "{} structure temp went non-finite", z.name);
        }
    }

    #[test]
    fn cargo_fire_heats_a_neighbouring_zone_through_the_real_conduction_chain() {
        let outside = ground_air();

        let mut with_fire = build();
        let mut without_fire = build();

        const FIRE_HEAT_W: f64 = 50_000.0;
        const SMOKE_KG_S: f64 = 0.002;
        for _ in 0..3000 {
            with_fire.network.inject_heat_w(with_fire.zones.cargo_fwd, FIRE_HEAT_W);
            with_fire.network.inject_smoke_kg_s(with_fire.zones.cargo_fwd, SMOKE_KG_S);
            with_fire.network.step(1.0, &outside, 0.0);
            without_fire.network.step(1.0, &outside, 0.0);
        }

        let cargo_hot = with_fire.network.air_temp_c(with_fire.zones.cargo_fwd);
        let cargo_cold = without_fire.network.air_temp_c(without_fire.zones.cargo_fwd);
        assert!(cargo_hot > cargo_cold + 50.0, "the fire must dominate CargoFwd's own temperature: hot={cargo_hot} cold={cargo_cold}");

        let avionics_hot = with_fire.network.structure_temp_c(with_fire.zones.main_avionics);
        let avionics_cold = without_fire.network.structure_temp_c(without_fire.zones.main_avionics);
        assert!(avionics_hot > avionics_cold + 1.0, "heat should reach the neighbouring MainAvionics zone through the real conduction link: hot={avionics_hot} cold={avionics_cold}");

        assert!(with_fire.network.smoke_concentration(with_fire.zones.cargo_fwd) > 0.0);
        assert_eq!(without_fire.network.smoke_concentration(without_fire.zones.cargo_fwd), 0.0);
    }

    #[test]
    fn blocking_a_cargo_extract_fan_makes_the_same_fire_hotter() {
        let outside = ground_air();
        let mut healthy = build();
        let mut blocked = build();
        blocked.network.set_ventilation_health(blocked.vents.cargo_fwd_fan, 0.0);

        const FIRE_HEAT_W: f64 = 20_000.0;
        for _ in 0..3000 {
            healthy.network.inject_heat_w(healthy.zones.cargo_fwd, FIRE_HEAT_W);
            blocked.network.inject_heat_w(blocked.zones.cargo_fwd, FIRE_HEAT_W);
            healthy.network.step(1.0, &outside, 0.0);
            blocked.network.step(1.0, &outside, 0.0);
        }
        assert!(
            blocked.network.air_temp_c(blocked.zones.cargo_fwd) > healthy.network.air_temp_c(healthy.zones.cargo_fwd),
            "a blocked cargo extract fan must leave the compartment hotter for the same fire"
        );
    }

    #[test]
    fn a_registered_component_accrues_damage_only_once_its_zone_gets_hot() {
        let mut a380 = build();
        let outside = ground_air();
        let hydraulic_hose = a380.damage.components.iter().position(|c| c.name == "NoseGearWellHydraulicHose").expect("component registered in build()");

        for _ in 0..500 {
            a380.network.step(1.0, &outside, 0.0);
            a380.damage.update(&a380.network, 1.0);
        }
        assert_eq!(a380.damage.damage_fraction(hydraulic_hose), 0.0);

        for _ in 0..3000 {
            a380.network.inject_heat_w(a380.zones.nose_gear_well, 8_000.0);
            a380.network.step(1.0, &outside, 0.0);
            a380.damage.update(&a380.network, 1.0);
        }
        assert!(a380.damage.damage_fraction(hydraulic_hose) > 0.0, "the hose should accrue damage once its bay runs hot");
    }

    #[test]
    fn gear_door_closed_keeps_the_bay_warmer_in_flight_than_doors_open() {
        let outside = OutsideAir { static_temp_c: -50.0, mach: 0.82, true_airspeed_m_s: 230.0 };
        let mut doors_open = build();
        doors_open.network.set_ventilation_health(doors_open.vents.wing_gear_door, 1.0);
        let mut doors_closed = build();

        for _ in 0..300 {
            doors_open.network.step(1.0, &outside, 0.0);
            doors_closed.network.step(1.0, &outside, 0.0);
        }
        assert!(
            doors_closed.network.air_temp_c(doors_closed.zones.wing_gear_well) > doors_open.network.air_temp_c(doors_open.zones.wing_gear_well) + 5.0,
            "closed doors {} vs open doors {}",
            doors_closed.network.air_temp_c(doors_closed.zones.wing_gear_well),
            doors_open.network.air_temp_c(doors_open.zones.wing_gear_well)
        );
    }
}
