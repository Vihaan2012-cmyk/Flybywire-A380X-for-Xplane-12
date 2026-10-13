//! The physics models' quantities, added to the pages that show their
//! systems (docs/physics/*.md). Every box leaves out the fields nothing
//! publishes yet, so a page only gains what the simulation really holds.

use super::canvas::{group, lamp, num, Group};
use super::pages::{hidden_group as hidden, tint};
use super::PageKind;

/// Extra boxes for a field-list page.
pub fn extra(kind: PageKind) -> Vec<Group> {
    match kind {
        PageKind::Apu => apu(),
        PageKind::Bleed => bleed(),
        PageKind::AirConditioning => packs(),
        PageKind::Pressurisation => oxygen(),
        PageKind::GearBrakes => wear(),
        PageKind::Electrical => electrical(),
        PageKind::Hydraulics => hydraulics(),
        PageKind::Fuel => fuel(),
        PageKind::Engine(n) => engine(n),
        PageKind::AirData => adirs(),
        _ => Vec::new(),
    }
}

/// Schematic pages whose physics has its own view (the PHYSICS button).
pub fn has_physics(kind: PageKind) -> bool {
    matches!(kind, PageKind::Electrical | PageKind::Hydraulics | PageKind::Fuel | PageKind::Engine(_))
}

/// Solved voltages and currents, generator shaft power, battery state, and
/// the circuit protection (physics/electrical.rs, FBW electrical patch).
fn electrical() -> Vec<Group> {
    let buses = ["AC_1", "AC_2", "AC_3", "AC_4", "AC_ESS", "AC_ESS_SHED", "DC_1", "DC_2", "DC_ESS", "DC_HOT_1", "DC_HOT_2", "DC_HOT_3", "DC_HOT_4", "247PP", "108PH"];
    let mut groups = vec![hidden(group(
        "Bus voltage",
        tint(0),
        buses.iter().map(|b| num(b, format!("A32NX_ELEC_{b}_BUS_POTENTIAL"), "V", 1)).collect(),
    ))];
    for n in 1..=4 {
        let g = |what: &str| format!("A32NX_ELEC_ENG_GEN_{n}_{what}");
        groups.push(hidden(group(
            &format!("Engine generator {n}"),
            tint(n),
            vec![
                num("Terminal voltage", g("POTENTIAL"), "V", 1),
                num("Current", g("CURRENT"), "A", 0),
                num("Frequency", g("FREQUENCY"), "Hz", 0),
                num("Load", g("LOAD"), "%", 0),
                num("Shaft power demand", g("SHAFT_POWER_DEMAND"), "W", 0),
                num("Engine gearbox load", format!("ENGINE_GEARBOX_ELEC_LOAD_W:{n}"), "W", 0),
            ],
        )));
    }
    for n in 1..=4 {
        let b = |what: &str| format!("A32NX_ELEC_BAT_{n}_{what}");
        groups.push(hidden(group(
            &format!("Battery {n}"),
            tint(n + 3),
            vec![
                num("Voltage", b("POTENTIAL"), "V", 2),
                num("Current", b("CURRENT"), "A", 1),
                num("Temperature", b("TEMPERATURE"), "C", 1),
                num("Internal resistance", b("INTERNAL_RESISTANCE"), "ohm", 4),
            ],
        )));
    }
    groups
}

/// Pump flow, shaft power and current (physics/hydraulics.rs, FBW hydraulic
/// patch), and what the pumps take from each engine.
fn hydraulics() -> Vec<Group> {
    let edps = ["GREEN_1A", "GREEN_1B", "GREEN_2A", "GREEN_2B", "YELLOW_3A", "YELLOW_3B", "YELLOW_4A", "YELLOW_4B"];
    let epumps = ["GA", "GB", "YA", "YB"];
    vec![
        hidden(group(
            "Engine-driven pumps",
            tint(1),
            edps.iter()
                .flat_map(|id| {
                    [
                        num(&format!("{id} flow"), format!("A32NX_HYD_{id}_EDPUMP_FLOW"), "gpm", 1),
                        num(&format!("{id} shaft power"), format!("A32NX_HYD_{id}_EDPUMP_SHAFT_POWER_W"), "W", 0),
                    ]
                })
                .collect(),
        )),
        hidden(group(
            "Electric pumps",
            tint(2),
            epumps
                .iter()
                .flat_map(|id| {
                    [
                        num(&format!("{id} flow"), format!("A32NX_HYD_{id}_EPUMP_FLOW"), "gpm", 1),
                        num(&format!("{id} current"), format!("A32NX_HYD_{id}_EPUMP_CURRENT"), "A", 1),
                        num(&format!("{id} power"), format!("A32NX_HYD_{id}_EPUMP_POWER_W"), "W", 0),
                    ]
                })
                .collect(),
        )),
        hidden(group(
            "Load on the engines",
            tint(3),
            (1..=4).map(|n| num(&format!("Engine {n}"), format!("ENGINE_GEARBOX_HYD_LOAD_W:{n}"), "W", 0)).collect(),
        )),
    ]
}

/// Tank temperatures from their heat balance, pump curves, feed pressure,
/// jettison and freezing (fuel.rs, physics/fluids.rs).
fn fuel() -> Vec<Group> {
    vec![
        hidden(group(
            "Tank temperatures",
            tint(0),
            (1..=11).map(|i| num(&format!("Tank {i}"), format!("FUEL_TEMP_{i}"), "C", 1)).collect(),
        )),
        hidden(group(
            "Freezing and jettison",
            tint(1),
            vec![
                num("Freeze point", "FUEL_TEMP_FREEZE_POINT", "C", 0),
                lamp("FOB LO TEMP", "FUEL_FOB_LO_TEMP"),
                lamp("Jettison", "FUEL JETTISON SWITCH"),
            ],
        )),
        hidden(group(
            "Engine feed",
            tint(2),
            (1..=4)
                .flat_map(|n| {
                    [
                        num(&format!("Engine {n} feed pressure"), format!("FUELSYSTEM ENGINE PRESSURE:{n}"), "psi", 1),
                        num(&format!("Engine {n} fuel demand"), format!("ENGINE_FUEL_DEMAND_KG_S:{n}"), "kg/s", 3),
                    ]
                })
                .collect(),
        )),
        hidden(group(
            "Pumps",
            tint(3),
            (1..=21)
                .flat_map(|n| {
                    [
                        num(&format!("Pump {n} pressure"), format!("FUEL_PUMP_PRESSURE_PSI:{n}"), "psi", 1),
                        num(&format!("Pump {n} current"), format!("FUEL_PUMP_CURRENT_A:{n}"), "A", 1),
                    ]
                })
                .collect(),
        )),
    ]
}

/// One engine's gas-turbine model: what it burns, what it drives, and its
/// accumulated life (physics/engine, physics/damage.rs).
fn engine(n: usize) -> Vec<Group> {
    vec![
        hidden(group(
            "Spools and fuel",
            tint(n),
            vec![
                num("N1", format!("ENGINE_N1:{n}"), "%", 1),
                num("N2", format!("ENGINE_N2:{n}"), "%", 1),
                num("N3", format!("A32NX_ENGINE_N3:{n}"), "%", 1),
                num("EGT", format!("A32NX_ENGINE_EGT:{n}"), "C", 0),
                num("Fuel demand", format!("ENGINE_FUEL_DEMAND_KG_S:{n}"), "kg/s", 3),
            ],
        )),
        hidden(group(
            "What the core drives",
            tint(n + 1),
            vec![
                num("Bleed extraction", format!("ENGINE_BLEED_EXTRACTION_KG_S:{n}"), "kg/s", 3),
                num("Generator load", format!("ENGINE_GEARBOX_ELEC_LOAD_W:{n}"), "W", 0),
                num("Hydraulic pump load", format!("ENGINE_GEARBOX_HYD_LOAD_W:{n}"), "W", 0),
            ],
        )),
        hidden(group(
            "Life",
            tint(n + 2),
            vec![
                num("Creep life used", format!("ENGINE_CREEP_LIFE_FRACTION:{n}"), "", 3),
                num("Compressor efficiency loss", format!("ENGINE_COMPRESSOR_EFFICIENCY_LOSS:{n}"), "", 3),
            ],
        )),
    ]
}

/// The PW980 core (fbw-common apu/pw980_physics.rs): oil, bleed supply,
/// the loads the spool turns, protective trips.
fn apu() -> Vec<Group> {
    vec![
        hidden(group(
            "Core physics",
            tint(1),
            vec![
                num("Fuel demand", "ENGINE_FUEL_DEMAND_KG_S:0", "kg/s", 3),
                num("Generator shaft load", "ENGINE_GEARBOX_ELEC_LOAD_W:0", "W", 0),
                num("Gen 1 shaft power", "ELEC_APU_GEN_1_SHAFT_POWER_DEMAND", "W", 0),
                num("Gen 2 shaft power", "ELEC_APU_GEN_2_SHAFT_POWER_DEMAND", "W", 0),
                num("Oil pressure", "APU_OIL_PRESSURE_PSI", "psi", 1),
                num("Oil temperature", "APU_OIL_TEMPERATURE_C", "C", 0),
                lamp("Protective trip", "APU_PROTECTIVE_TRIP"),
            ],
        )),
        hidden(group(
            "Load compressor",
            tint(2),
            vec![
                num("Bleed supply", "APU_BLEED_SUPPLY_KG_S", "kg/s", 3),
                num("Bleed pressure", "APU_BLEED_SUPPLY_PSI", "psi", 1),
            ],
        )),
    ]
}

/// What each engine's bleed takes from its compressor (the engine model
/// burns fuel for it).
fn bleed() -> Vec<Group> {
    vec![hidden(group(
        "Extraction from the engines",
        tint(4),
        (1..=4)
            .flat_map(|n| {
                [
                    num(&format!("Engine {n} bleed flow"), format!("ENGINE_BLEED_EXTRACTION_KG_S:{n}"), "kg/s", 3),
                    num(&format!("Engine {n} generator load"), format!("ENGINE_GEARBOX_ELEC_LOAD_W:{n}"), "W", 0),
                    num(&format!("Engine {n} hydraulic load"), format!("ENGINE_GEARBOX_HYD_LOAD_W:{n}"), "W", 0),
                ]
            })
            .collect(),
    ))]
}

/// Each pack's air cycle machine (a380_systems air_cycle_machine.rs).
fn packs() -> Vec<Group> {
    (1..=2)
        .map(|n| {
            let p = |what: &str| format!("A32NX_COND_PACK_{n}_{what}");
            hidden(group(
                &format!("Pack {n} air cycle machine"),
                tint(n + 1),
                vec![
                    num("ACM pressure ratio", p("ACM_PRESSURE_RATIO"), "", 2),
                    num("Turbine outlet", p("TURBINE_OUTLET_TEMPERATURE"), "C", 1),
                    num("Pack outlet", p("OUTLET_TEMPERATURE"), "C", 1),
                    num("Ram air flow", p("RAM_AIR_FLOW"), "kg/s", 2),
                    num("Ram air outlet", p("RAM_AIR_OUTLET_TEMPERATURE"), "C", 1),
                    num("Ram air door", p("RAM_AIR_DOOR_POSITION"), "%", 0),
                    num("Hot air bypass", p("HOT_AIR_BYPASS_POSITION"), "%", 0),
                    num("Water extracted", p("WATER_EXTRACTED"), "kg/s", 4),
                ],
            ))
        })
        .collect()
}

/// Crew and passenger oxygen (oxygen.rs).
fn oxygen() -> Vec<Group> {
    vec![hidden(group(
        "Oxygen",
        tint(5),
        vec![
            num("Crew bottle pressure", "OXYGEN_CREW_PRESSURE_PSI", "psi", 0),
            num("Crew quantity", "OXYGEN_CREW_QUANTITY_PERCENT", "%", 0),
            num("Crew flow", "OXYGEN_CREW_FLOW_KG_S", "kg/s", 5),
            num("Diluter demand", "OXYGEN_CREW_DILUTION_FRACTION", "", 2),
            lamp("Crew low pressure", "OXYGEN_CREW_LOW_PRESSURE"),
            num("Passenger generators", "OXYGEN_PAX_QUANTITY_PERCENT", "%", 0),
            num("Passenger flow", "OXYGEN_PAX_FLOW_LPM", "L/min", 1),
            lamp("Passenger masks deployed", "OXYGEN_PAX_MASKS_DEPLOYED"),
        ],
    ))]
}

/// Wear and life the damage model keeps (physics/damage.rs).
fn wear() -> Vec<Group> {
    vec![hidden(group(
        "Engine life",
        tint(6),
        (1..=4)
            .flat_map(|n| {
                [
                    num(&format!("Engine {n} creep life used"), format!("ENGINE_CREEP_LIFE_FRACTION:{n}"), "", 3),
                    num(&format!("Engine {n} compressor loss"), format!("ENGINE_COMPRESSOR_EFFICIENCY_LOSS:{n}"), "", 3),
                ]
            })
            .collect(),
    ))]
}

/// Each ADIRU's inertial and air data physics (physics/adirs.rs): alignment,
/// drift and sensor errors, the pressures its probes sense, probe heat.
fn adirs() -> Vec<Group> {
    let mut groups: Vec<Group> = (1..=3)
        .map(|n| {
            let s = |what: &str| format!("ADIRS_STUDY_{n}_{what}");
            hidden(group(
                &format!("ADIRU {n} physics"),
                tint(n + 2),
                vec![
                    num("Alignment state", s("ALIGN_STATE"), "", 0),
                    num("Position error", s("POSITION_ERROR_NM"), "nm", 2),
                    num("Drift", s("DRIFT_NM_HR"), "nm/h", 2),
                    num("Gyro bias", s("GYRO_BIAS_DEG_HR"), "deg/h", 4),
                    num("Accelerometer bias", s("ACCEL_BIAS_UG"), "ug", 0),
                    num("Static pressure", s("STATIC_PRESSURE_PA"), "Pa", 0),
                    num("Total pressure", s("TOTAL_PRESSURE_PA"), "Pa", 0),
                    num("Probe heat", s("PROBE_HEAT_W"), "W", 0),
                    lamp("Pitot blocked", s("PITOT_BLOCKED")),
                    lamp("Static blocked", s("STATIC_BLOCKED")),
                ],
            ))
        })
        .collect();
    groups.push(hidden(group(
        "Radio altimeter terrain probe",
        tint(6),
        vec![
            lamp("Probe valid", "RA_TERRAIN_PROBE_VALID"),
            num("Height above terrain", "RA_TERRAIN_PROBE_ALT_ABOVE_GROUND", "ft", 0),
        ],
    )));
    groups
}
