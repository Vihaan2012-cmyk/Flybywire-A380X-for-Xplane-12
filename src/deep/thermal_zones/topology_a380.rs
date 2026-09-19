//! The A380 zone topology (task step 4): a GENERIC, public-data airframe
//! breakdown into the zones the brief calls for -- avionics bays, gear
//! wells, wing leading/trailing-edge compartments per wing, engine pylons
//! and nacelle cowls, the APU compartment, cargo holds, the crown area,
//! cabin decks, the tail cone and the belly fairing pack bays -- built
//! onto [`super::network::ThermalNetwork`].
//!
//! **No AMM zone drawing is public** for the A380, so every zone's
//! volume/area/mass figure here is a **GENERIC**, order-of-magnitude
//! estimate, derived from the aircraft's public overall dimensions
//! (Airbus's own published A380 "Aircraft Characteristics -- Airport and
//! Maintenance Planning" data, widely repeated: overall length 72.72 m,
//! wingspan 79.75 m, fuselage external width/height about 7.14 m x 8.41 m
//! (twin-deck cross-section), lower-deck cargo volume about 184 m^3 across
//! its forward/aft/bulk compartments) and on the same category of
//! large-transport equipment/heat-load figures `physics::bays.rs` already
//! uses for its own avionics/cargo bays (repeated here independently per
//! this module's self-contained-module rule, not imported). Every number
//! below is commented with its basis; none is a precision claim.
//!
//! Landing gear arrangement (nose + two wing-mounted + two body-mounted
//! main gear bogies, 20 wheels total) and the four wing-pylon-mounted
//! engines (1/2 left wing, 3/4 right wing, Airbus's own public numbering)
//! are both from Airbus's public A380 factsheet/type description, not
//! invented.

use super::damage::ThermalDamageRegistry;
use super::network::{ThermalNetwork, Zone, ZoneId, ZoneRef};

/// Every zone's `ZoneId`, named, so a caller never has to remember a raw
/// index.
pub struct A380Zones {
    pub main_avionics: ZoneId,
    pub upper_avionics: ZoneId,
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
}

/// Ventilation link indices a fault/system model needs to drive
/// (`ThermalNetwork::set_ventilation_health`): fan failures, gear-door
/// position, cowl/duct blockage.
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
}

pub struct A380Thermal {
    pub network: ThermalNetwork,
    pub zones: A380Zones,
    pub vents: A380VentLinks,
    pub damage: ThermalDamageRegistry,
}

/// Standard sea-level/typical-cabin ambient used as every zone's initial
/// temperature -- a cold-and-dark aircraft on stand, not a specific flight
/// condition (matches `physics::bays.rs::Bays::new`'s own 25 C initial
/// convention, reproduced independently here).
const INITIAL_TEMP_C: f64 = 22.0;

/// Builds the full A380 thermal network with GENERIC public-data
/// dimensions, typical equipment heat loads, the conduction/ventilation
/// adjacency between zones, and a handful of representative
/// [`super::damage::ThermalComponent`] registrations demonstrating the
/// damage interface at real locations (a wiring bundle, a hydraulic hose,
/// an anti-ice duct seal, wiring near an engine, a pack-duct seal).
pub fn build() -> A380Thermal {
    let mut net = ThermalNetwork::new();

    // -- Avionics bays. Same order-of-magnitude baseline heat figures as
    // `physics::bays.rs`'s own MAIN_AVIONICS/UPPER_AVIONICS (typical for a
    // large transport's main/upper equipment centre), volumes GENERIC
    // (main equipment centre under the cockpit floor is a walk-in space,
    // a few cubic metres; the smaller upper bay behind the cockpit less).
    let main_avionics = net.add_zone(Zone::new("MainAvionics", 12.0, 5.0e5, 150.0, 4.0, 0.0, 3000.0, INITIAL_TEMP_C));
    let upper_avionics = net.add_zone(Zone::new("UpperAvionics", 6.0, 3.0e5, 100.0, 3.0, 0.0, 2000.0, INITIAL_TEMP_C));

    // -- Gear wells. No baseline heat (brake/hydraulic heat is injected
    // externally via `inject_heat_w`); volumes GENERIC from the physical
    // envelope needed to stow each bogie (nose: 2 wheels; wing MLG: 2 legs
    // x 4 wheels; body MLG: 2 legs x 6 wheels, per Airbus's public A380
    // landing-gear description), each with a sizeable exterior skin area
    // (gear doors + bay walls exposed to the slipstream once open).
    let nose_gear_well = net.add_zone(Zone::new("NoseGearWell", 8.0, 2.0e5, 60.0, 6.0, 0.0, 0.0, INITIAL_TEMP_C));
    let wing_gear_well = net.add_zone(Zone::new("WingGearWell", 15.0, 4.0e5, 80.0, 10.0, 0.0, 0.0, INITIAL_TEMP_C));
    let body_gear_well = net.add_zone(Zone::new("BodyGearWell", 15.0, 4.0e5, 80.0, 8.0, 0.0, 0.0, INITIAL_TEMP_C));

    // -- Wing leading/trailing-edge compartments, per wing (left/right).
    // Long, shallow compartments running much of the half-span (A380
    // half-span ~36 m outboard of the fuselage): volumes/areas GENERIC,
    // sized as a thin box along the span rather than a cabin-like room.
    // No baseline heat (anti-ice bleed heat is injected externally, a
    // transient system input, not a standing load); LE gets more direct
    // sun (top/forward-facing skin) than TE.
    let wing_le_left = net.add_zone(Zone::new("WingLeLeft", 25.0, 6.0e5, 40.0, 40.0, 0.6, 0.0, INITIAL_TEMP_C));
    let wing_le_right = net.add_zone(Zone::new("WingLeRight", 25.0, 6.0e5, 40.0, 40.0, 0.6, 0.0, INITIAL_TEMP_C));
    let wing_te_left = net.add_zone(Zone::new("WingTeLeft", 22.0, 5.5e5, 35.0, 35.0, 0.4, 0.0, INITIAL_TEMP_C));
    let wing_te_right = net.add_zone(Zone::new("WingTeRight", 22.0, 5.5e5, 35.0, 35.0, 0.4, 0.0, INITIAL_TEMP_C));

    // -- Pylons (1/2 left wing, 3/4 right wing, Airbus's public engine
    // numbering) and nacelle cowls, one per engine. Pylon baseline heat is
    // harness/equipment dissipation (GENERIC, small); nacelle baseline is
    // 0 -- engine-proximity heat is injected externally by the engine
    // model, this zone only carries the airframe-side cowl thermal mass
    // and its large ram-air-exposed exterior.
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

    // -- APU compartment (tail cone region): no baseline heat, the APU's
    // own running heat is injected externally by the APU model.
    let apu_compartment = net.add_zone(Zone::new("ApuCompartment", 6.0, 2.0e5, 70.0, 8.0, 0.1, 0.0, INITIAL_TEMP_C));

    // -- Cargo holds (forward/aft/bulk). Volumes GENERIC, split from
    // Airbus's own public ~184 m^3 total lower-deck cargo volume figure
    // roughly by the fwd/aft/bulk proportions typical of a widebody
    // lower-deck layout (forward the largest, bulk the smallest).
    // Baseline heat (lighting/equipment) matches `physics::bays.rs`'s own
    // FWD_CARGO/AFT_CARGO figure (50 W).
    let cargo_fwd = net.add_zone(Zone::new("CargoFwd", 110.0, 5.0e5, 120.0, 30.0, 0.0, 50.0, INITIAL_TEMP_C));
    let cargo_aft = net.add_zone(Zone::new("CargoAft", 60.0, 3.0e5, 80.0, 20.0, 0.0, 50.0, INITIAL_TEMP_C));
    let cargo_bulk = net.add_zone(Zone::new("CargoBulk", 14.0, 1.0e5, 30.0, 8.0, 0.0, 20.0, INITIAL_TEMP_C));

    // -- Crown area: the long void between the upper-deck ceiling and the
    // upper fuselage skin, running most of the fuselage length. GENERIC
    // volume; large upper-fuselage exterior skin area with strong direct
    // sun exposure (top of the aircraft).
    let crown_area = net.add_zone(Zone::new("CrownArea", 40.0, 3.0e5, 30.0, 60.0, 0.8, 0.0, INITIAL_TEMP_C));

    // -- Cabin decks (main deck / upper deck). Deliberately crude boundary
    // nodes as far as this network's *neighbours* are concerned: real
    // cabin-temperature regulation (the ECS pack control loop) is a
    // separate workstream already modelled elsewhere in the crate
    // (`physics::bays.rs`'s own module doc cites `docs/physics/air.md`'s
    // cabin-zone model); this module only needs the cabin as a
    // ventilation/conduction *reference* other zones exchange with, not a
    // second implementation of cabin climate control. No exterior skin
    // (insulated from the raw exterior by design -- only reachable via
    // Crown/cargo/tail-cone conduction) and no baseline heat load, so a
    // standalone run of this module does not diverge; a caller that wants
    // the cabin's own heat balance modelled drives it externally via
    // `inject_heat_w` the same as any other system.
    let cabin_main_deck = net.add_zone(Zone::new("CabinMainDeck", 700.0, 1.0e6, 500.0, 0.0, 0.0, 0.0, INITIAL_TEMP_C));
    let cabin_upper_deck = net.add_zone(Zone::new("CabinUpperDeck", 350.0, 5.0e5, 300.0, 0.0, 0.0, 0.0, INITIAL_TEMP_C));

    // -- Tail cone: unpressurised aft fuselage cone aft of the rear
    // pressure bulkhead, carrying control cable runs and APU bleed
    // ducting. GENERIC volume/area.
    let tail_cone = net.add_zone(Zone::new("TailCone", 25.0, 2.0e5, 40.0, 20.0, 0.3, 0.0, INITIAL_TEMP_C));

    // -- Belly fairing / pack bays: the under-fuselage aerodynamic fairing
    // housing the ECS packs and their ducting. Baseline heat is a small
    // GENERIC standing parasitic loss (pack casing/ducting, not the
    // packs' own multi-hundred-kW bleed-air thermodynamic cycle, which is
    // the air/ECS workstream's model, not this one's).
    let belly_fairing_packs = net.add_zone(Zone::new("BellyFairingPacks", 30.0, 2.0e5, 60.0, 25.0, 0.0, 1500.0, INITIAL_TEMP_C));

    // -----------------------------------------------------------------
    // Conduction links: physically adjacent airframe structure.
    // -----------------------------------------------------------------
    net.add_conduction_link(main_avionics, upper_avionics, 40.0); // vertically stacked
    net.add_conduction_link(nose_gear_well, main_avionics, 30.0); // MEC sits directly above/behind the nose bay
    net.add_conduction_link(nose_gear_well, cargo_fwd, 20.0);
    net.add_conduction_link(main_avionics, cargo_fwd, 20.0); // both forward lower lobe
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
    net.add_conduction_link(crown_area, cabin_upper_deck, 150.0); // shared ceiling/skin panel
    net.add_conduction_link(cabin_upper_deck, cabin_main_deck, 300.0); // shared inter-deck floor/ceiling
    net.add_conduction_link(cabin_main_deck, cargo_fwd, 80.0); // main deck floor beams
    net.add_conduction_link(cabin_main_deck, cargo_aft, 80.0);
    net.add_conduction_link(cabin_main_deck, cargo_bulk, 40.0);
    net.add_conduction_link(tail_cone, apu_compartment, 50.0);
    net.add_conduction_link(tail_cone, cargo_aft, 20.0);

    // -----------------------------------------------------------------
    // Ventilation links: extract/supply flow-through, generalising
    // `physics::bays.rs`'s single hardcoded "cabin supply" fan term into
    // real zone-to-zone (avionics/cargo bays draw from the modelled
    // cabin) and zone-to-outside (gear bays, nacelles, pylons, the APU
    // compartment, the pack bay, the tail cone) paths. Flow figures
    // GENERIC, same order of magnitude as `physics::bays.rs`'s own
    // fan-flow constants for the bays this generalises (0.25-0.35 kg/s).
    // -----------------------------------------------------------------
    let main_avionics_fan = net.add_ventilation_link(main_avionics, ZoneRef::Zone(cabin_main_deck), 0.35);
    let upper_avionics_fan = net.add_ventilation_link(upper_avionics, ZoneRef::Zone(cabin_main_deck), 0.25);
    let cargo_fwd_fan = net.add_ventilation_link(cargo_fwd, ZoneRef::Zone(cabin_main_deck), 0.30);
    let cargo_aft_fan = net.add_ventilation_link(cargo_aft, ZoneRef::Zone(cabin_main_deck), 0.30);
    let cargo_bulk_fan = net.add_ventilation_link(cargo_bulk, ZoneRef::Zone(cabin_main_deck), 0.10);

    // Gear bay <-> outside: nameplate flow represents the bay fully open
    // to the airstream (doors open, gear extended); `health` is the gear
    // door's own open fraction, driven by the landing-gear system, not a
    // failure by itself (though a stuck door is one -- see FAILURES.md).
    let nose_gear_door = net.add_ventilation_link(nose_gear_well, ZoneRef::OutsideAir, 1.0);
    let wing_gear_door = net.add_ventilation_link(wing_gear_well, ZoneRef::OutsideAir, 1.2);
    let body_gear_door = net.add_ventilation_link(body_gear_well, ZoneRef::OutsideAir, 1.2);
    // Gear doors are closed for most of the flight (only open briefly
    // during retraction/extension), so 0.0 (closed) -- not
    // `add_ventilation_link`'s own default of 1.0 -- is this link's
    // physically correct resting state; a landing-gear system sets it to
    // the door's real open fraction while cycling.
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

    // -----------------------------------------------------------------
    // Damage interface (task step 3): a handful of representative
    // components. Temperature limits are GENERIC/typical figures for
    // their class of part (no A380-specific public spec exists for any of
    // them); damage rates are GENERIC modelling choices for plausible
    // failure pacing (module doc).
    // -----------------------------------------------------------------
    let mut damage = ThermalDamageRegistry::new();
    // Avionics wiring/equipment: typical continuous equipment operating
    // temperature limit for an air-cooled avionics bay, order of
    // magnitude of RTCA DO-160 environmental category "B1" max operating
    // temperatures (commonly ~55-70 C for standard equipment classes).
    damage.register("MainAvionicsWiringBundle", main_avionics, 70.0, 1.0 / 1800.0); // ~30 min to fail at +10 C over
    // Hydraulic hose in a gear bay: typical continuous-service temperature
    // limit for aircraft synthetic hydraulic fluid/hose (commonly cited
    // ~135 C for phosphate-ester fluid systems).
    damage.register("NoseGearWellHydraulicHose", nose_gear_well, 135.0, 1.0 / 3600.0);
    // Wing anti-ice duct insulation blanket: typical duct-lagging
    // continuous rating for a hot bleed-air anti-ice duct run.
    damage.register("WingLeLeftAntiIceDuctInsulation", wing_le_left, 150.0, 1.0 / 1800.0);
    // Nacelle wiring (fire loop / reverser actuation harness): aircraft
    // wiring's typical continuous rating class (e.g. SAE AS22759-class
    // insulation, commonly 125-200 C depending on grade; a conservative
    // mid-range figure is used).
    damage.register("NacelleCowl1Wiring", nacelle_cowl[0], 125.0, 1.0 / 1800.0);
    // Pack-bay duct seal, near hot bleed-conditioned air discharge.
    damage.register("BellyFairingPackDuctSeal", belly_fairing_packs, 200.0, 1.0 / 1800.0);

    A380Thermal {
        network: net,
        zones: A380Zones {
            main_avionics,
            upper_avionics,
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
        // 2 avionics + 3 gear wells + 4 wing LE/TE + 4 pylons + 4 nacelles
        // + 1 APU + 3 cargo + 1 crown + 2 cabin decks + 1 tail cone + 1
        // belly = 26.
        assert_eq!(a380.network.zones.len(), 26);
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

    // -- Headline coupling test: a cargo fire (heat + smoke) propagates
    // through the real conduction chain to a neighbouring zone, and its
    // smoke is readable in the fire's own compartment -- proving the
    // topology's links, not just the generic engine, actually couple. --

    #[test]
    fn cargo_fire_heats_a_neighbouring_zone_through_the_real_conduction_chain() {
        let outside = ground_air();

        let mut with_fire = build();
        let mut without_fire = build();

        const FIRE_HEAT_W: f64 = 50_000.0; // a cargo-fire-scale heat release, GENERIC order of magnitude
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

        // Cross-zone propagation via CargoFwd <-> MainAvionics conduction.
        let avionics_hot = with_fire.network.structure_temp_c(with_fire.zones.main_avionics);
        let avionics_cold = without_fire.network.structure_temp_c(without_fire.zones.main_avionics);
        assert!(avionics_hot > avionics_cold + 1.0, "heat should reach the neighbouring MainAvionics zone through the real conduction link: hot={avionics_hot} cold={avionics_cold}");

        // Smoke is readable directly in the fire's own compartment (what
        // a cargo smoke detector there would sense).
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

        // Cold: no damage over a while.
        for _ in 0..500 {
            a380.network.step(1.0, &outside, 0.0);
            a380.damage.update(&a380.network, 1.0);
        }
        assert_eq!(a380.damage.damage_fraction(hydraulic_hose), 0.0);

        // A brake-heat-soak-scale injection into the nose gear well (gear
        // doors closed, per `build()`'s resting state -- no ventilation
        // to wash it out): hose limit is 135 C, drive the bay comfortably
        // over it.
        for _ in 0..3000 {
            a380.network.inject_heat_w(a380.zones.nose_gear_well, 8_000.0);
            a380.network.step(1.0, &outside, 0.0);
            a380.damage.update(&a380.network, 1.0);
        }
        assert!(a380.damage.damage_fraction(hydraulic_hose) > 0.0, "the hose should accrue damage once its bay runs hot");
    }

    #[test]
    fn gear_door_closed_keeps_the_bay_warmer_in_flight_than_doors_open() {
        // In flight (nonzero TAS), an open gear door ventilates the bay's
        // *air* node toward the cold recovery temperature almost
        // immediately (the air node's own thermal mass is small); the
        // much heavier structure node only cools slowly via skin
        // conduction, identically whether the door is open or closed. So
        // this compares the two shortly after the door state diverges
        // (well inside the structure's own multi-hundred-second time
        // constant) rather than at full steady state, where both would
        // eventually converge to the same recovery temperature anyway.
        let outside = OutsideAir { static_temp_c: -50.0, mach: 0.82, true_airspeed_m_s: 230.0 };
        let mut doors_open = build();
        doors_open.network.set_ventilation_health(doors_open.vents.wing_gear_door, 1.0);
        let mut doors_closed = build(); // `build()`'s own resting state is already closed

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
