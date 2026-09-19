//! Registers every `deep::cabin` component, failure and ECAM alert through
//! `deep::api`'s `Registry` (see `docs/deep/BRIEF.md`, "Registering
//! failures, components and ECAM alerts"). Extended as each backlog item in
//! this directory is finished; see `PROGRESS.md` for the order and for the
//! new published Vars each alert's trigger/procedure assumes (none of them
//! exist in the plugin yet — this directory's models are not wired to `Vars`
//! at all, per the deep-systems brief's self-containment rule — so every
//! `var(...)` name below is a documented contract for whoever wires this
//! area in, not a variable read anywhere today).

use crate::deep::api::*;

const ATA_WATER: u16 = 38;
const ATA_WASTE: u16 = 38;
const ATA_IFE: u16 = 44;
const ATA_GALLEY: u16 = 25;
const ATA_DOORS: u16 = 52;

pub fn register(r: &mut Registry) {
    register_water(r);
    register_waste(r);
    register_ife(r);
    register_galley(r);
    register_doors_slides(r);
}

// ---------------------------------------------------------------------------
// ATA 38: potable water (water.rs).
// ---------------------------------------------------------------------------

fn register_water(r: &mut Registry) {
    let tank = "38_wtr.potable_tank";
    let bleed_valve = "38_wtr.bleed_valve";
    let compressor = "38_wtr.compressor";
    let qty_sensor = "38_wtr.qty_sensor";
    let heaters = ["38_wtr.heater_fwd", "38_wtr.heater_mid", "38_wtr.heater_aft"];
    let masts = ["38_wtr.mast_fwd", "38_wtr.mast_aft"];

    r.component(ComponentDef {
        id: tank.into(),
        area: Area::Cabin,
        ata: ATA_WATER,
        name: "Potable water tank/lines".into(),
        params: vec![ParamDef { name: "leak".into(), meaning: "0 sealed .. 1 fully open leak orifice, vented to cabin pressure".into(), healthy: 0.0 }],
        failures: vec![],
    });
    r.component(ComponentDef {
        id: bleed_valve.into(),
        area: Area::Cabin,
        ata: ATA_WATER,
        name: "Potable water bleed pressurisation valve".into(),
        params: vec![ParamDef { name: "fault".into(), meaning: "0 healthy .. 1 delivers no pressurisation air".into(), healthy: 0.0 }],
        failures: vec![],
    });
    r.component(ComponentDef {
        id: compressor.into(),
        area: Area::Cabin,
        ata: ATA_WATER,
        name: "Potable water backup pressurisation compressor".into(),
        params: vec![ParamDef { name: "fault".into(), meaning: "0 healthy .. 1 delivers no pressurisation air".into(), healthy: 0.0 }],
        failures: vec![],
    });
    r.component(ComponentDef {
        id: qty_sensor.into(),
        area: Area::Cabin,
        ata: ATA_WATER,
        name: "Potable water quantity sensor".into(),
        params: vec![ParamDef { name: "stuck".into(), meaning: "0 healthy .. 1 (>0.5) freezes the last reading".into(), healthy: 0.0 }],
        failures: vec![],
    });
    for (i, id) in heaters.iter().enumerate() {
        r.component(ComponentDef {
            id: (*id).into(),
            area: Area::Cabin,
            ata: ATA_WATER,
            name: format!("{} zone point-of-use water heater", super::Zone::ALL[i].name()),
            params: vec![ParamDef { name: "fault".into(), meaning: "0 healthy .. 1 no heating element output".into(), healthy: 0.0 }],
            failures: vec![],
        });
    }
    for (i, id) in masts.iter().enumerate() {
        let name = if i == 0 { "Forward" } else { "Aft" };
        r.component(ComponentDef {
            id: (*id).into(),
            area: Area::Cabin,
            ata: ATA_WATER,
            name: format!("{name} drain mast anti-ice heater"),
            params: vec![ParamDef { name: "fault".into(), meaning: "0 healthy .. 1 no heater output, mast can ice at cold OAT".into(), healthy: 0.0 }],
            failures: vec![],
        });
    }

    let f_leak = r.failure(FailureDef {
        id: failure_id(Area::Cabin, ATA_WATER, 1),
        area: Area::Cabin,
        ata: ATA_WATER,
        name: "Potable water tank/line leak".into(),
        component: tank.into(),
        model_field: "deep::cabin::water::WaterFaults.leak".into(),
        magnitude: "0 sealed .. 1 fully open leak, orifice flow scaled by tank gauge pressure".into(),
        effect: "continuous water loss, faster quantity depletion, eventual dry tank".into(),
    });
    let f_bleed = r.failure(FailureDef {
        id: failure_id(Area::Cabin, ATA_WATER, 2),
        area: Area::Cabin,
        ata: ATA_WATER,
        name: "Potable water bleed pressurisation valve fault".into(),
        component: bleed_valve.into(),
        model_field: "deep::cabin::water::WaterFaults.bleed_valve_fault".into(),
        magnitude: "0 healthy .. 1 no bleed air delivered to the tank ullage".into(),
        effect: "pressurisation falls back to the slower backup compressor, or is lost entirely without it".into(),
    });
    let f_compressor = r.failure(FailureDef {
        id: failure_id(Area::Cabin, ATA_WATER, 3),
        area: Area::Cabin,
        ata: ATA_WATER,
        name: "Potable water backup compressor fault".into(),
        component: compressor.into(),
        model_field: "deep::cabin::water::WaterFaults.compressor_fault".into(),
        magnitude: "0 healthy .. 1 no backup pressurisation air delivered".into(),
        effect: "no pressurisation available if bleed is also unavailable: distribution flow fails".into(),
    });
    let f_qty = r.failure(FailureDef {
        id: failure_id(Area::Cabin, ATA_WATER, 4),
        area: Area::Cabin,
        ata: ATA_WATER,
        name: "Potable water quantity sensor stuck".into(),
        component: qty_sensor.into(),
        model_field: "deep::cabin::water::WaterFaults.quantity_sensor_fault".into(),
        magnitude: "0 healthy .. 1 (>0.5 latches) freezes the displayed quantity at its last reading".into(),
        effect: "displayed quantity stops tracking the real, still-draining tank".into(),
    });
    let mut f_heaters = Vec::new();
    for (i, id) in heaters.iter().enumerate() {
        f_heaters.push(r.failure(FailureDef {
            id: failure_id(Area::Cabin, ATA_WATER, 5 + i as u16),
            area: Area::Cabin,
            ata: ATA_WATER,
            name: format!("{} zone water heater fault", super::Zone::ALL[i].name()),
            component: (*id).into(),
            model_field: format!("deep::cabin::water::WaterFaults.heater_fault[{i}]"),
            magnitude: "0 healthy .. 1 no heating element output".into(),
            effect: "hot water unavailable at that zone's basin/galley tap".into(),
        }));
    }
    let mut f_masts = Vec::new();
    for (i, id) in masts.iter().enumerate() {
        f_masts.push(r.failure(FailureDef {
            id: failure_id(Area::Cabin, ATA_WATER, 8 + i as u16),
            area: Area::Cabin,
            ata: ATA_WATER,
            name: format!("{} drain mast heater fault", if i == 0 { "Forward" } else { "Aft" }),
            component: (*id).into(),
            model_field: format!("deep::cabin::water::WaterFaults.mast_heater_fault[{i}]"),
            magnitude: "0 healthy .. 1 no heater output".into(),
            effect: "mast can drop below freezing at cold OAT and ice/block, with a real drain-mast-icing hazard".into(),
        }));
    }

    r.alert(
        EcamAlert::new("CABIN_WATER_QTY_LO", ATA_WATER, "POTABLE WATER QTY LO", Level::Advisory, var("CABIN_WATER_QTY_PERCENT").lt(10.0))
            .confirm(5.0)
            .step(line("ADVISE CABIN CREW", "").colour("white"))
            .status_line("POTABLE WATER LO QTY")
            .raised_by(&[f_leak]),
    );
    r.alert(
        EcamAlert::new("CABIN_WATER_PRESS_FAULT", ATA_WATER, "POTABLE WATER PRESS FAULT", Level::Caution, var("CABIN_WATER_PRESS_PSI").lt(20.0))
            .confirm(10.0)
            .step(line("WATER COMPRESSOR", "ON").done(var("CABIN_WATER_COMPRESSOR_CMD").on()))
            .status_line("POTABLE WATER SYS DEGRADED")
            .raised_by(&[f_bleed, f_compressor]),
    );
    r.alert(
        EcamAlert::new("CABIN_DRAIN_MAST_ICE", ATA_WATER, "DRAIN MAST ICING", Level::Advisory, any(vec![var("CABIN_MAST_BLOCKED:1").on(), var("CABIN_MAST_BLOCKED:2").on()]))
            .confirm(30.0)
            .step(line("MONITOR OAT", "").colour("white"))
            .status_line("DRAIN MAST HEATER — CHECK MAINT")
            .raised_by(&f_masts),
    );
    let _ = f_qty; // registered above as a component/failure; not tied to its own ECAM alert.
    let _ = f_heaters;
}

// ---------------------------------------------------------------------------
// ATA 38: waste (waste.rs).
// ---------------------------------------------------------------------------

fn register_waste(r: &mut Registry) {
    let generator = "38_wst.vacuum_generator";
    let tanks = ["38_wst.tank_fwd", "38_wst.tank_mid", "38_wst.tank_aft"];
    let sensors = ["38_wst.level_sensor_fwd", "38_wst.level_sensor_mid", "38_wst.level_sensor_aft"];
    let valves = ["38_wst.flush_valve_fwd", "38_wst.flush_valve_mid", "38_wst.flush_valve_aft"];

    r.component(ComponentDef {
        id: generator.into(),
        area: Area::Cabin,
        ata: ATA_WASTE,
        name: "Vacuum toilet generator (blower)".into(),
        params: vec![ParamDef { name: "fault".into(), meaning: "0 healthy .. 1 no assisted suction below the natural-vacuum altitude".into(), healthy: 0.0 }],
        failures: vec![],
    });
    for (i, id) in tanks.iter().enumerate() {
        r.component(ComponentDef {
            id: (*id).into(),
            area: Area::Cabin,
            ata: ATA_WASTE,
            name: format!("{} zone waste tank", super::Zone::ALL[i].name()),
            // The tank's fill level is an operating state (`WasteSystem`'s
            // own `level_l`), not a health/degradation parameter; its
            // sensor and flush valve, registered separately below, carry
            // this tank's actual fault parameters.
            params: vec![],
            failures: vec![],
        });
    }
    for (i, id) in sensors.iter().enumerate() {
        r.component(ComponentDef {
            id: (*id).into(),
            area: Area::Cabin,
            ata: ATA_WASTE,
            name: format!("{} zone waste tank level sensor", super::Zone::ALL[i].name()),
            params: vec![ParamDef { name: "stuck".into(), meaning: "0 healthy .. 1 (>0.5) freezes the last reading".into(), healthy: 0.0 }],
            failures: vec![],
        });
    }
    for (i, id) in valves.iter().enumerate() {
        r.component(ComponentDef {
            id: (*id).into(),
            area: Area::Cabin,
            ata: ATA_WASTE,
            name: format!("{} zone toilet flush valve", super::Zone::ALL[i].name()),
            params: vec![
                ParamDef { name: "stuck_open".into(), meaning: "0 healthy .. 1 (>0.5) continuous leak/suction loss".into(), healthy: 0.0 },
                ParamDef { name: "stuck_closed".into(), meaning: "0 healthy .. 1 flush command proportionally ineffective".into(), healthy: 0.0 },
            ],
            failures: vec![],
        });
    }

    let f_generator = r.failure(FailureDef {
        id: failure_id(Area::Cabin, ATA_WASTE, 20),
        area: Area::Cabin,
        ata: ATA_WASTE,
        name: "Vacuum toilet generator fault".into(),
        component: generator.into(),
        model_field: "deep::cabin::waste::WasteFaults.generator_fault".into(),
        magnitude: "0 healthy .. 1 delivers no assisted suction".into(),
        effect: "on the ground/low altitude (below the natural cabin/ambient differential threshold), flushes are weak or ineffective".into(),
    });
    let mut f_full = Vec::new();
    for i in 0..super::Zone::COUNT {
        f_full.push(r.failure(FailureDef {
            id: failure_id(Area::Cabin, ATA_WASTE, 21 + i as u16),
            area: Area::Cabin,
            ata: ATA_WASTE,
            name: format!("{} zone waste tank level sensor stuck", super::Zone::ALL[i].name()),
            component: sensors[i].into(),
            model_field: format!("deep::cabin::waste::WasteFaults.tank_level_sensor_fault[{i}]"),
            magnitude: "0 healthy .. 1 (>0.5 latches) freezes the displayed level".into(),
            effect: "crew cannot see the tank filling toward full for that zone".into(),
        }));
    }
    let mut f_stuck_open = Vec::new();
    let mut f_stuck_closed = Vec::new();
    for i in 0..super::Zone::COUNT {
        f_stuck_open.push(r.failure(FailureDef {
            id: failure_id(Area::Cabin, ATA_WASTE, 24 + i as u16),
            area: Area::Cabin,
            ata: ATA_WASTE,
            name: format!("{} zone toilet flush valve stuck open", super::Zone::ALL[i].name()),
            component: valves[i].into(),
            model_field: format!("deep::cabin::waste::WasteFaults.valve_stuck_open[{i}]"),
            magnitude: "0 healthy .. 1 (>0.5 latches) continuous small leak into the tank with no flush command".into(),
            effect: "the zone's tank fills without use and reaches full early".into(),
        }));
        f_stuck_closed.push(r.failure(FailureDef {
            id: failure_id(Area::Cabin, ATA_WASTE, 27 + i as u16),
            area: Area::Cabin,
            ata: ATA_WASTE,
            name: format!("{} zone toilet flush valve stuck closed", super::Zone::ALL[i].name()),
            component: valves[i].into(),
            model_field: format!("deep::cabin::waste::WasteFaults.valve_stuck_closed[{i}]"),
            magnitude: "0 healthy .. 1 flush command proportionally loses effect".into(),
            effect: "the bowl does not clear on flush".into(),
        }));
    }

    let _ = f_full;
    let _ = f_stuck_open;
    let _ = f_stuck_closed;
    let _ = f_generator;
}

// ---------------------------------------------------------------------------
// ATA 44/25: IFE and cabin power (ife.rs).
// ---------------------------------------------------------------------------

fn register_ife(r: &mut Registry) {
    let zones = ["44_ife.seat_wiring_fwd", "44_ife.seat_wiring_mid", "44_ife.seat_wiring_aft"];
    let servers = ["44_ife.server_1", "44_ife.server_2"];

    for (i, id) in zones.iter().enumerate() {
        r.component(ComponentDef {
            id: (*id).into(),
            area: Area::Cabin,
            ata: ATA_IFE,
            name: format!("{} seat power/IFE wiring", super::Zone::ALL[i].name()),
            params: vec![ParamDef { name: "fault".into(), meaning: "0 healthy .. 1 fraction of the zone's seat wiring shorted, self-heating toward smoke".into(), healthy: 0.0 }],
            failures: vec![],
        });
    }
    for (i, id) in servers.iter().enumerate() {
        r.component(ComponentDef {
            id: (*id).into(),
            area: Area::Cabin,
            ata: ATA_IFE,
            name: format!("IFE head-end server {}", i + 1),
            params: vec![ParamDef { name: "fault".into(), meaning: "0 healthy .. 1 (>=0.95) server failed outright".into(), healthy: 0.0 }],
            failures: vec![],
        });
    }

    let mut f_seat = Vec::new();
    for i in 0..super::Zone::COUNT {
        f_seat.push(r.failure(FailureDef {
            id: failure_id(Area::Cabin, ATA_IFE, 1 + i as u16),
            area: Area::Cabin,
            ata: ATA_IFE,
            name: format!("{} zone seat power/IFE wiring short", super::Zone::ALL[i].name()),
            component: zones[i].into(),
            model_field: format!("deep::cabin::ife::IfeFaults.seat_fault[{i}]"),
            magnitude: "0 healthy .. 1 fraction of the zone's wiring shorted, driving I^2R self-heating".into(),
            effect: "zone wiring heats over minutes, passing through an overheating advisory to a smoke warning, then the zone's own protection trips it dead".into(),
        }));
    }
    let mut f_server = Vec::new();
    for i in 0..2usize {
        f_server.push(r.failure(FailureDef {
            id: failure_id(Area::Cabin, ATA_IFE, 4 + i as u16),
            area: Area::Cabin,
            ata: ATA_IFE,
            name: format!("IFE head-end server {} failure", i + 1),
            component: servers[i].into(),
            model_field: format!("deep::cabin::ife::IfeFaults.server_fault[{i}]"),
            magnitude: "0 healthy .. 1 (>=0.95 fails outright) server output lost".into(),
            effect: "that server's content is lost; the redundant server keeps the cabin served until both fail".into(),
        }));
    }

    r.alert(
        EcamAlert::new(
            "CABIN_IFE_SMOKE",
            ATA_IFE,
            "CAB IFE SMOKE",
            Level::Warning,
            any(vec![var("CABIN_IFE_ZONE_SMOKE:1").on(), var("CABIN_IFE_ZONE_SMOKE:2").on(), var("CABIN_IFE_ZONE_SMOKE:3").on()]),
        )
        .step(line("SEAT POWER", "OFF").done(var("CABIN_SEAT_POWER_CMD").off()))
        .step(line("CABIN CREW", "ADVISE").colour("white"))
        .status_line("IFE SEAT POWER — CHECK MAINT")
        .inop_sys("IFE")
        .raised_by(&f_seat),
    );
    r.alert(
        EcamAlert::new(
            "CABIN_IFE_SYS_FAULT",
            ATA_IFE,
            "IFE SYS FAULT",
            Level::Advisory,
            all(vec![var("CABIN_IFE_SERVER_FAIL:1").on(), var("CABIN_IFE_SERVER_FAIL:2").on()]),
        )
        .confirm(5.0)
        .status_line("IFE SYS INOP")
        .inop_sys("IFE")
        .raised_by(&f_server),
    );
}

// ---------------------------------------------------------------------------
// ATA 25: galleys (galley.rs).
// ---------------------------------------------------------------------------

fn register_galley(r: &mut Registry) {
    let buses = ["25_gal.bus_fwd", "25_gal.bus_mid", "25_gal.bus_aft"];
    let ovens = ["25_gal.oven_fwd", "25_gal.oven_mid", "25_gal.oven_aft"];
    let chillers = ["25_gal.chiller_fwd", "25_gal.chiller_mid", "25_gal.chiller_aft"];
    let boilers = ["25_gal.boiler_fwd", "25_gal.boiler_mid", "25_gal.boiler_aft"];

    for (i, id) in buses.iter().enumerate() {
        r.component(ComponentDef {
            id: (*id).into(),
            area: Area::Cabin,
            ata: ATA_GALLEY,
            name: format!("{} galley bus feed", super::Zone::ALL[i].name()),
            params: vec![ParamDef { name: "fault".into(), meaning: "0 healthy .. 1 (>=1.0) galley dead regardless of the aircraft commercial bus".into(), healthy: 0.0 }],
            failures: vec![],
        });
    }
    for (i, id) in ovens.iter().enumerate() {
        r.component(ComponentDef {
            id: (*id).into(),
            area: Area::Cabin,
            ata: ATA_GALLEY,
            name: format!("{} galley oven", super::Zone::ALL[i].name()),
            params: vec![ParamDef { name: "thermostat_stuck".into(), meaning: "0 healthy cycling thermostat .. >0 stuck closed, drives the cavity toward smoke".into(), healthy: 0.0 }],
            failures: vec![],
        });
    }
    for (i, id) in chillers.iter().enumerate() {
        r.component(ComponentDef {
            id: (*id).into(),
            area: Area::Cabin,
            ata: ATA_GALLEY,
            name: format!("{} galley chiller compressor", super::Zone::ALL[i].name()),
            params: vec![ParamDef { name: "fault".into(), meaning: "0 healthy .. 1 no cooling capacity, compartment drifts to cabin ambient".into(), healthy: 0.0 }],
            failures: vec![],
        });
    }
    for (i, id) in boilers.iter().enumerate() {
        r.component(ComponentDef {
            id: (*id).into(),
            area: Area::Cabin,
            ata: ATA_GALLEY,
            name: format!("{} galley water boiler", super::Zone::ALL[i].name()),
            params: vec![ParamDef { name: "fault".into(), meaning: "0 healthy .. 1 no heating element output".into(), healthy: 0.0 }],
            failures: vec![],
        });
    }

    let mut f_bus = Vec::new();
    let mut f_oven = Vec::new();
    let mut f_chiller = Vec::new();
    let mut f_boiler = Vec::new();
    for i in 0..super::Zone::COUNT {
        f_bus.push(r.failure(FailureDef {
            id: failure_id(Area::Cabin, ATA_GALLEY, 1 + i as u16),
            area: Area::Cabin,
            ata: ATA_GALLEY,
            name: format!("{} galley bus feed fault", super::Zone::ALL[i].name()),
            component: buses[i].into(),
            model_field: format!("deep::cabin::galley::GalleyFaults.bus_fault[{i}]"),
            magnitude: "0 healthy .. 1 (>=1.0) galley loses all three appliances".into(),
            effect: "oven, chiller and boiler all lose power in that galley only".into(),
        }));
        f_oven.push(r.failure(FailureDef {
            id: failure_id(Area::Cabin, ATA_GALLEY, 4 + i as u16),
            area: Area::Cabin,
            ata: ATA_GALLEY,
            name: format!("{} galley oven thermostat stuck", super::Zone::ALL[i].name()),
            component: ovens[i].into(),
            model_field: format!("deep::cabin::galley::GalleyFaults.oven_overheat[{i}]"),
            magnitude: "0 healthy cycling .. >0 stuck closed (element never cycles off)".into(),
            effect: "cavity temperature runs away past its setpoint and, left long enough, reaches the smoke threshold".into(),
        }));
        f_chiller.push(r.failure(FailureDef {
            id: failure_id(Area::Cabin, ATA_GALLEY, 7 + i as u16),
            area: Area::Cabin,
            ata: ATA_GALLEY,
            name: format!("{} galley chiller compressor failure", super::Zone::ALL[i].name()),
            component: chillers[i].into(),
            model_field: format!("deep::cabin::galley::GalleyFaults.chiller_fault[{i}]"),
            magnitude: "0 healthy .. 1 no cooling capacity".into(),
            effect: "the compartment warms back toward cabin ambient over tens of minutes".into(),
        }));
        f_boiler.push(r.failure(FailureDef {
            id: failure_id(Area::Cabin, ATA_GALLEY, 10 + i as u16),
            area: Area::Cabin,
            ata: ATA_GALLEY,
            name: format!("{} galley water boiler fault", super::Zone::ALL[i].name()),
            component: boilers[i].into(),
            model_field: format!("deep::cabin::galley::GalleyFaults.boiler_fault[{i}]"),
            magnitude: "0 healthy .. 1 no heating element output".into(),
            effect: "no hot water for beverages from that galley".into(),
        }));
    }

    r.alert(
        EcamAlert::new(
            "CABIN_GALLEY_OVEN_SMOKE",
            ATA_GALLEY,
            "GALLEY OVEN SMOKE",
            Level::Warning,
            any(vec![var("CABIN_GALLEY_OVEN_SMOKE:1").on(), var("CABIN_GALLEY_OVEN_SMOKE:2").on(), var("CABIN_GALLEY_OVEN_SMOKE:3").on()]),
        )
        .step(line("GALLEY OVEN", "OFF").done(var("CABIN_GALLEY_OVEN_CMD").off()))
        .step(line("CABIN CREW", "ADVISE").colour("white"))
        .status_line("GALLEY OVEN — CHECK MAINT")
        .raised_by(&f_oven),
    );
    r.alert(
        EcamAlert::new(
            "CABIN_GALLEY_FAULT",
            ATA_GALLEY,
            "GALLEY FAULT",
            Level::Advisory,
            any(vec![var("CABIN_GALLEY_BUS_FAULT:1").on(), var("CABIN_GALLEY_BUS_FAULT:2").on(), var("CABIN_GALLEY_BUS_FAULT:3").on()]),
        )
        .confirm(5.0)
        .status_line("GALLEY — CHECK MAINT")
        .raised_by(&f_bus),
    );
    let _ = f_chiller;
    let _ = f_boiler;
}

// ---------------------------------------------------------------------------
// ATA 52: doors and slides (doors_slides.rs).
// ---------------------------------------------------------------------------

fn register_doors_slides(r: &mut Registry) {
    // `doors_slides.rs`'s model is generic — one `DoorSlide` per physical
    // door/slide pair, instantiated once per `doors.rs`'s twenty named
    // interactive points (`doors::NAMES`). These components/failures are
    // registered once as the class; a real integration instantiates one
    // `DoorSlide` (and, if it wants per-door catalogue rows, one component
    // id per `doors::NAMES` entry) for each door that actually has a seal/
    // slide/latch/actuator in `doors.rs`'s own point table.
    let seal = "52_dr.seal";
    let bottle = "52_dr.slide_bottle";
    let latch_sensor = "52_dr.latch_sensor";
    let actuator = "52_dr.cargo_actuator";

    r.component(ComponentDef {
        id: seal.into(),
        area: Area::Cabin,
        ata: ATA_DOORS,
        name: "Door seal (per passenger/emergency door, x up to 16 main+upper deck doors)".into(),
        params: vec![ParamDef { name: "leak".into(), meaning: "0 sealed .. 1 fully open leak orifice into the pressurisation model".into(), healthy: 0.0 }],
        failures: vec![],
    });
    r.component(ComponentDef {
        id: bottle.into(),
        area: Area::Cabin,
        ata: ATA_DOORS,
        name: "Evacuation slide gas bottle (per armed door)".into(),
        params: vec![ParamDef { name: "leak".into(), meaning: "0 healthy .. 1 fully depletes the bottle over 24 h".into(), healthy: 0.0 }],
        failures: vec![],
    });
    r.component(ComponentDef {
        id: latch_sensor.into(),
        area: Area::Cabin,
        ata: ATA_DOORS,
        name: "Door not-latched proximity sensor (per door)".into(),
        params: vec![ParamDef { name: "stuck".into(), meaning: "0 healthy .. 1 (>=0.5) freezes the last reading".into(), healthy: 0.0 }],
        failures: vec![],
    });
    r.component(ComponentDef {
        id: actuator.into(),
        area: Area::Cabin,
        ata: ATA_DOORS,
        name: "Cargo door hydraulic actuator (forward/aft cargo doors)".into(),
        params: vec![
            ParamDef { name: "jam".into(), meaning: "0 free .. 1 fully seized, caps travel at the jammed fraction".into(), healthy: 0.0 },
            ParamDef { name: "hydraulic_loss".into(), meaning: "0 full system pressure .. 1 no driving pressure at all".into(), healthy: 0.0 },
        ],
        failures: vec![],
    });

    let f_seal = r.failure(FailureDef {
        id: failure_id(Area::Cabin, ATA_DOORS, 1),
        area: Area::Cabin,
        ata: ATA_DOORS,
        name: "Door seal leak".into(),
        component: seal.into(),
        model_field: "deep::cabin::doors_slides::DoorSlideFaults.seal_leak".into(),
        magnitude: "0 sealed .. 1 fully open (2 cm^2) leak orifice, flow via mdot=Cd*A*sqrt(2*rho*dP)".into(),
        effect: "continuous cabin air loss at that door, an extra load on the pressurisation outflow valves".into(),
    });
    let f_bottle = r.failure(FailureDef {
        id: failure_id(Area::Cabin, ATA_DOORS, 2),
        area: Area::Cabin,
        ata: ATA_DOORS,
        name: "Evacuation slide bottle leak".into(),
        component: bottle.into(),
        model_field: "deep::cabin::doors_slides::DoorSlideFaults.bottle_leak".into(),
        magnitude: "0 healthy .. 1 empties the bottle over 24 h".into(),
        effect: "the slide may not inflate to a usable pressure if armed/fired after the leak has run long enough".into(),
    });
    let f_latch = r.failure(FailureDef {
        id: failure_id(Area::Cabin, ATA_DOORS, 3),
        area: Area::Cabin,
        ata: ATA_DOORS,
        name: "Door not-latched sensor stuck".into(),
        component: latch_sensor.into(),
        model_field: "deep::cabin::doors_slides::DoorSlideFaults.latch_sensor_fault".into(),
        magnitude: "0 healthy .. 1 (>=0.5 latches) freezes the displayed latched/not-latched state".into(),
        effect: "the indication can disagree with the door's real position".into(),
    });
    let f_jam = r.failure(FailureDef {
        id: failure_id(Area::Cabin, ATA_DOORS, 4),
        area: Area::Cabin,
        ata: ATA_DOORS,
        name: "Cargo door actuator jam".into(),
        component: actuator.into(),
        model_field: "deep::cabin::doors_slides::DoorSlideFaults.actuator_jam".into(),
        magnitude: "0 free .. 1 fully seized; caps travel at (1 - jam) * 100 percent".into(),
        effect: "the cargo door cannot reach a commanded target beyond the jammed travel limit".into(),
    });
    let f_hyd = r.failure(FailureDef {
        id: failure_id(Area::Cabin, ATA_DOORS, 5),
        area: Area::Cabin,
        ata: ATA_DOORS,
        name: "Cargo door actuator hydraulic circuit loss".into(),
        component: actuator.into(),
        model_field: "deep::cabin::doors_slides::DoorSlideFaults.hydraulic_loss".into(),
        magnitude: "0 full pressure .. 1 no driving pressure".into(),
        effect: "the actuator cannot move at all".into(),
    });

    r.alert(
        EcamAlert::new("CABIN_DOOR_NOT_LATCHED", ATA_DOORS, "DOOR NOT LATCHED", Level::Warning, var("CABIN_DOOR_LATCHED:1").off())
            .step(line("DOOR", "CHECK").colour("white"))
            .status_line("DOOR NOT LATCHED")
            .raised_by(&[f_latch]),
    );
    r.alert(
        EcamAlert::new("CABIN_SLIDE_LO_PRESSURE", ATA_DOORS, "SLIDE LO PRESSURE", Level::Advisory, var("CABIN_SLIDE_PRESSURE_LOW:1").on())
            .confirm(5.0)
            .status_line("SLIDE 1 — CHECK MAINT")
            .raised_by(&[f_bottle]),
    );
    r.alert(
        EcamAlert::new("CABIN_CARGO_DOOR_FAULT", ATA_DOORS, "CARGO DOOR FAULT", Level::Caution, var("CABIN_CARGO_DOOR_JAMMED:1").on())
            .confirm(2.0)
            .step(line("CARGO DOOR", "STOP").done(var("CABIN_CARGO_DOOR_CMD:1").off()))
            .status_line("CARGO DOOR — CHECK MAINT")
            .inop_sys("CARGO DOOR")
            .raised_by(&[f_jam, f_hyd]),
    );
    let _ = f_seal;
}
