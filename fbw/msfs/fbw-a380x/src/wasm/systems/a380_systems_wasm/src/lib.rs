mod ailerons;
mod autobrakes;
mod body_wheel_steering;
mod brakes;
mod cargo_doors;
mod elevators;
mod fire;
mod flaps;
mod fuel;
mod gear;
mod nose_wheel_steering;
mod payload;
mod reversers;
mod rudder;
mod spoilers;
mod trimmable_horizontal_stabilizer;

use a380_systems::A380;
use ailerons::ailerons;
use autobrakes::autobrakes;
use body_wheel_steering::body_wheel_steering;
use brakes::brakes;
use cargo_doors::cargo_doors;
use elevators::elevators;
use fire::fire;
use flaps::flaps;
use fuel::fuel;
use gear::gear;
use nose_wheel_steering::nose_wheel_steering;
use payload::payload;
use reversers::reversers;
use rudder::rudder;
use spoilers::spoilers;
use std::error::Error;
use systems::shared::{report_diagnostic, ElectricalBusType};

use systems_wasm::{MsfsSimulationBuilder, Variable};
use trimmable_horizontal_stabilizer::trimmable_horizontal_stabilizer;

#[msfs::gauge(name=systems)]
async fn systems(mut gauge: msfs::Gauge) -> Result<(), Box<dyn Error>> {
    // The default panic output is lost when the WASM instance aborts, so print the
    // panic message and location to the MSFS console before the trap happens.
    std::panic::set_hook(Box::new(|panic_info| {
        println!("A380X_SYSTEMS PANIC: {panic_info}");
        report_diagnostic(&format!("A380X_SYSTEMS PANIC: {panic_info}"));
    }));

    let mut sim_connect = gauge.open_simconnect("systems")?;

    let key_prefix = "A32NX_";
    let (mut simulation, mut handler) = MsfsSimulationBuilder::new(
        key_prefix,
        Variable::named(&format!("{}START_STATE", key_prefix)),
        sim_connect.as_mut().get_mut(),
    )
    .with_electrical_buses([
        (ElectricalBusType::AlternatingCurrent(1), 2),
        (ElectricalBusType::AlternatingCurrent(2), 3),
        (ElectricalBusType::AlternatingCurrent(3), 4),
        (ElectricalBusType::AlternatingCurrent(4), 5),
        (ElectricalBusType::AlternatingCurrentEssential, 6),
        (ElectricalBusType::AlternatingCurrentEssentialShed, 7),
        (ElectricalBusType::AlternatingCurrentGndFltService, 16),
        (ElectricalBusType::DirectCurrent(1), 8),
        (ElectricalBusType::DirectCurrent(2), 9),
        (ElectricalBusType::DirectCurrentEssential, 10),
        (ElectricalBusType::DirectCurrentNamed("309PP"), 11),
        (ElectricalBusType::DirectCurrentHot(1), 12),
        (ElectricalBusType::DirectCurrentHot(2), 13),
        (ElectricalBusType::DirectCurrentHot(3), 14),
        (ElectricalBusType::DirectCurrentHot(4), 15),
        (ElectricalBusType::DirectCurrentGndFltService, 17),
    ])?
    .with_auxiliary_power_unit(Variable::named("OVHD_APU_START_PB_IS_AVAILABLE"), 8, 21)?
    .with_engine_anti_ice(4)?
    .with_wing_anti_ice()?
    .with_fuel_pumps(1..=21)?
    .with_failures(a380_systems::fbw_failures())
    .provides_aircraft_variable("ACCELERATION BODY X", "feet per second squared", 0)?
    .provides_aircraft_variable("ACCELERATION BODY Y", "feet per second squared", 0)?
    .provides_aircraft_variable("ACCELERATION BODY Z", "feet per second squared", 0)?
    .provides_aircraft_variable("AIRSPEED INDICATED", "Knots", 0)?
    .provides_aircraft_variable("AIRSPEED MACH", "Mach", 0)?
    .provides_aircraft_variable("AIRSPEED TRUE", "Knots", 0)?
    .provides_aircraft_variable("AMBIENT DENSITY", "Slugs per cubic feet", 0)?
    .provides_aircraft_variable("AMBIENT IN CLOUD", "Bool", 0)?
    .provides_aircraft_variable("AMBIENT PRECIP RATE", "millimeters of water", 0)?
    .provides_aircraft_variable("AMBIENT PRESSURE", "inHg", 0)?
    .provides_aircraft_variable("AMBIENT TEMPERATURE", "celsius", 0)?
    .provides_aircraft_variable("AMBIENT WIND DIRECTION", "Degrees", 0)?
    .provides_aircraft_variable("AMBIENT WIND VELOCITY", "Knots", 0)?
    .provides_aircraft_variable("AMBIENT WIND X", "meter per second", 0)?
    .provides_aircraft_variable("AMBIENT WIND Y", "meter per second", 0)?
    .provides_aircraft_variable("AMBIENT WIND Z", "meter per second", 0)?
    .provides_aircraft_variable("ANTISKID BRAKES ACTIVE", "Bool", 0)?
    .provides_aircraft_variable("CENTER WHEEL ROTATION ANGLE", "Degrees", 0)?
    .provides_aircraft_variable("CONTACT POINT COMPRESSION", "Percent", 0)?
    .provides_aircraft_variable("CONTACT POINT COMPRESSION", "Percent", 1)?
    .provides_aircraft_variable("CONTACT POINT COMPRESSION", "Percent", 2)?
    .provides_aircraft_variable("CONTACT POINT COMPRESSION", "Percent", 3)?
    .provides_aircraft_variable("CONTACT POINT COMPRESSION", "Percent", 4)?
    .provides_aircraft_variable("ENG ON FIRE", "Bool", 1)?
    .provides_aircraft_variable("ENG ON FIRE", "Bool", 2)?
    .provides_aircraft_variable("ENG ON FIRE", "Bool", 3)?
    .provides_aircraft_variable("ENG ON FIRE", "Bool", 4)?
    .provides_aircraft_variable("FUELSYSTEM TANK QUANTITY", "gallons", 1)?
    .provides_aircraft_variable("FUELSYSTEM TANK QUANTITY", "gallons", 2)?
    .provides_aircraft_variable("FUELSYSTEM TANK QUANTITY", "gallons", 3)?
    .provides_aircraft_variable("FUELSYSTEM TANK QUANTITY", "gallons", 4)?
    .provides_aircraft_variable("FUELSYSTEM TANK QUANTITY", "gallons", 5)?
    .provides_aircraft_variable("FUELSYSTEM TANK QUANTITY", "gallons", 6)?
    .provides_aircraft_variable("FUELSYSTEM TANK QUANTITY", "gallons", 7)?
    .provides_aircraft_variable("FUELSYSTEM TANK QUANTITY", "gallons", 8)?
    .provides_aircraft_variable("FUELSYSTEM TANK QUANTITY", "gallons", 9)?
    .provides_aircraft_variable("FUELSYSTEM TANK QUANTITY", "gallons", 10)?
    .provides_aircraft_variable("FUELSYSTEM TANK QUANTITY", "gallons", 11)?
    .provides_aircraft_variable("FUELSYSTEM LINE FUEL FLOW", "gallons per hour", 141)?
    .provides_aircraft_variable("FUELSYSTEM VALVE OPEN", "Bool", 46)?
    .provides_aircraft_variable("FUELSYSTEM VALVE OPEN", "Bool", 47)?
    .provides_aircraft_variable("FUELSYSTEM VALVE OPEN", "Bool", 48)?
    .provides_aircraft_variable("FUELSYSTEM VALVE OPEN", "Bool", 49)?
    .provides_aircraft_variable("FUELSYSTEM VALVE OPEN", "Bool", 57)?
    .provides_aircraft_variable("FUELSYSTEM VALVE OPEN", "Bool", 58)?
    .provides_aircraft_variable("GEAR ANIMATION POSITION", "Percent", 0)?
    .provides_aircraft_variable("GEAR ANIMATION POSITION", "Percent", 1)?
    .provides_aircraft_variable("GEAR ANIMATION POSITION", "Percent", 2)?
    .provides_aircraft_variable("GEAR ANIMATION POSITION", "Percent", 3)?
    .provides_aircraft_variable("GEAR ANIMATION POSITION", "Percent", 4)?
    .provides_aircraft_variable("GEAR CENTER POSITION", "Percent", 0)?
    .provides_aircraft_variable("GEAR LEFT POSITION", "Percent", 0)?
    .provides_aircraft_variable("GEAR RIGHT POSITION", "Percent", 0)?
    .provides_aircraft_variable("GENERAL ENG STARTER ACTIVE", "Bool", 1)?
    .provides_aircraft_variable("GENERAL ENG STARTER ACTIVE", "Bool", 2)?
    .provides_aircraft_variable("GENERAL ENG STARTER", "Bool", 1)?
    .provides_aircraft_variable("GENERAL ENG STARTER", "Bool", 2)?
    .provides_aircraft_variable("GENERAL ENG STARTER", "Bool", 3)?
    .provides_aircraft_variable("GENERAL ENG STARTER", "Bool", 4)?
    .provides_aircraft_variable("GENERAL ENG OIL TEMPERATURE", "celsius", 1)?
    .provides_aircraft_variable("GENERAL ENG OIL TEMPERATURE", "celsius", 2)?
    .provides_aircraft_variable("GENERAL ENG OIL TEMPERATURE", "celsius", 3)?
    .provides_aircraft_variable("GENERAL ENG OIL TEMPERATURE", "celsius", 4)?
    .provides_aircraft_variable("GENERAL ENG OIL PRESSURE", "psi", 1)?
    .provides_aircraft_variable("GENERAL ENG OIL PRESSURE", "psi", 2)?
    .provides_aircraft_variable("GENERAL ENG OIL PRESSURE", "psi", 3)?
    .provides_aircraft_variable("GENERAL ENG OIL PRESSURE", "psi", 4)?
    .provides_aircraft_variable("GPS GROUND SPEED", "Knots", 0)?
    .provides_aircraft_variable("GPS GROUND MAGNETIC TRACK", "Degrees", 0)?
    .provides_aircraft_variable("GPS GROUND TRUE TRACK", "Degrees", 0)?
    .provides_aircraft_variable("INCIDENCE ALPHA", "Degrees", 0)?
    .provides_aircraft_variable("INDICATED ALTITUDE", "Feet", 0)?
    .provides_aircraft_variable("INTERACTIVE POINT OPEN:0", "Percent", 0)?
    .provides_aircraft_variable("INTERACTIVE POINT OPEN", "Percent", 2)?
    .provides_aircraft_variable("INTERACTIVE POINT OPEN", "Percent", 3)?
    .provides_aircraft_variable("INTERACTIVE POINT OPEN", "Percent", 6)?
    .provides_aircraft_variable("INTERACTIVE POINT OPEN", "Percent", 8)?
    .provides_aircraft_variable("INTERACTIVE POINT OPEN", "Percent", 10)?
    .provides_aircraft_variable("INTERACTIVE POINT OPEN", "Percent", 11)?
    .provides_aircraft_variable("INTERACTIVE POINT OPEN", "Percent", 12)?
    .provides_aircraft_variable("INTERACTIVE POINT OPEN", "Percent", 13)?
    .provides_aircraft_variable("INTERACTIVE POINT OPEN", "Percent", 14)?
    .provides_aircraft_variable("INTERACTIVE POINT OPEN", "Percent", 15)?
    .provides_aircraft_variable("INTERACTIVE POINT OPEN", "Percent", 16)?
    .provides_aircraft_variable("INTERACTIVE POINT OPEN", "Percent", 17)?
    .provides_aircraft_variable("KOHLSMAN SETTING MB", "Millibars", 1)?
    .provides_aircraft_variable("LIGHT BEACON", "Bool", 0)?
    .provides_aircraft_variable("LIGHT BEACON ON", "Bool", 0)?
    .provides_aircraft_variable("PLANE ALT ABOVE GROUND", "Feet", 0)?
    .provides_aircraft_variable("PLANE ALTITUDE", "Feet", 0)?
    .provides_aircraft_variable("PLANE PITCH DEGREES", "Degrees", 0)?
    .provides_aircraft_variable("PLANE BANK DEGREES", "Degrees", 0)?
    .provides_aircraft_variable("PLANE HEADING DEGREES MAGNETIC", "Degrees", 0)?
    .provides_aircraft_variable("PLANE HEADING DEGREES TRUE", "Degrees", 0)?
    .provides_aircraft_variable("PLANE LATITUDE", "degree latitude", 0)?
    .provides_aircraft_variable("PLANE LONGITUDE", "degree longitude", 0)?
    .provides_aircraft_variable("PRESSURE ALTITUDE", "Feet", 0)?
    .provides_aircraft_variable("PUSHBACK STATE", "Enum", 0)?
    .provides_aircraft_variable("PUSHBACK ANGLE", "Radians", 0)?
    .provides_aircraft_variable("SEA LEVEL PRESSURE", "Millibars", 0)?
    .provides_aircraft_variable("SIM ON GROUND", "Bool", 0)?
    .provides_aircraft_variable("SURFACE TYPE", "Enum", 0)?
    .provides_aircraft_variable("TOTAL AIR TEMPERATURE", "celsius", 0)?
    .provides_aircraft_variable("TOTAL WEIGHT", "Pounds", 0)?
    .provides_aircraft_variable("TOTAL WEIGHT YAW MOI", "Slugs feet squared", 0)?
    .provides_aircraft_variable("TOTAL WEIGHT PITCH MOI", "Slugs feet squared", 0)?
    .provides_aircraft_variable("TRAILING EDGE FLAPS LEFT PERCENT", "Percent", 0)?
    .provides_aircraft_variable("TRAILING EDGE FLAPS RIGHT PERCENT", "Percent", 0)?
    .provides_aircraft_variable("TURB ENG CORRECTED N1", "Percent", 1)?
    .provides_aircraft_variable("TURB ENG CORRECTED N1", "Percent", 2)?
    .provides_aircraft_variable("TURB ENG CORRECTED N1", "Percent", 3)?
    .provides_aircraft_variable("TURB ENG CORRECTED N1", "Percent", 4)?
    .provides_aircraft_variable("TURB ENG CORRECTED N2", "Percent", 1)?
    .provides_aircraft_variable("TURB ENG CORRECTED N2", "Percent", 2)?
    .provides_aircraft_variable("TURB ENG CORRECTED N2", "Percent", 3)?
    .provides_aircraft_variable("TURB ENG CORRECTED N2", "Percent", 4)?
    .provides_aircraft_variable("TURB ENG IGNITION SWITCH EX1", "Enum", 1)?
    .provides_aircraft_variable("TURB ENG IGNITION SWITCH EX1", "Enum", 2)?
    .provides_aircraft_variable("TURB ENG IGNITION SWITCH EX1", "Enum", 3)?
    .provides_aircraft_variable("TURB ENG IGNITION SWITCH EX1", "Enum", 4)?
    .provides_aircraft_variable("TURB ENG JET THRUST", "Pounds", 1)?
    .provides_aircraft_variable("TURB ENG JET THRUST", "Pounds", 2)?
    .provides_aircraft_variable("TURB ENG JET THRUST", "Pounds", 3)?
    .provides_aircraft_variable("TURB ENG JET THRUST", "Pounds", 4)?
    .provides_aircraft_variable("UNLIMITED FUEL", "Bool", 0)?
    .provides_aircraft_variable("ZULU TIME", "seconds", 0)?
    .provides_aircraft_variable("ZULU DAY OF YEAR", "Number", 0)?
    .provides_aircraft_variable("VELOCITY BODY X", "feet per second", 0)?
    .provides_aircraft_variable("VELOCITY BODY Y", "feet per second", 0)?
    .provides_aircraft_variable("VELOCITY BODY Z", "feet per second", 0)?
    .provides_aircraft_variable("VELOCITY WORLD Y", "feet per minute", 0)?
    .provides_aircraft_variable("WHEEL RPM", "RPM", 1)?
    .provides_aircraft_variable("WHEEL RPM", "RPM", 2)?
    .provides_aircraft_variable("ROTATION VELOCITY BODY X", "degree per second", 0)?
    .provides_aircraft_variable("ROTATION VELOCITY BODY Y", "degree per second", 0)?
    .provides_aircraft_variable("ROTATION VELOCITY BODY Z", "degree per second", 0)?
    .provides_aircraft_variable(
        "ROTATION ACCELERATION BODY X",
        "radian per second squared",
        0,
    )?
    .provides_aircraft_variable(
        "ROTATION ACCELERATION BODY Y",
        "radian per second squared",
        0,
    )?
    .provides_aircraft_variable(
        "ROTATION ACCELERATION BODY Z",
        "radian per second squared",
        0,
    )?
    .provides_aircraft_variable("PAYLOAD STATION WEIGHT", "Pounds", 1)?
    .provides_aircraft_variable("PAYLOAD STATION WEIGHT", "Pounds", 2)?
    .provides_aircraft_variable("PAYLOAD STATION WEIGHT", "Pounds", 3)?
    .provides_aircraft_variable("PAYLOAD STATION WEIGHT", "Pounds", 4)?
    .provides_aircraft_variable("PAYLOAD STATION WEIGHT", "Pounds", 5)?
    .provides_aircraft_variable("PAYLOAD STATION WEIGHT", "Pounds", 6)?
    .provides_aircraft_variable("PAYLOAD STATION WEIGHT", "Pounds", 7)?
    .provides_aircraft_variable("PAYLOAD STATION WEIGHT", "Pounds", 8)?
    .provides_aircraft_variable("PAYLOAD STATION WEIGHT", "Pounds", 9)?
    .provides_aircraft_variable("PAYLOAD STATION WEIGHT", "Pounds", 10)?
    .provides_aircraft_variable("PAYLOAD STATION WEIGHT", "Pounds", 11)?
    .provides_aircraft_variable("PAYLOAD STATION WEIGHT", "Pounds", 12)?
    .provides_aircraft_variable("PAYLOAD STATION WEIGHT", "Pounds", 13)?
    .provides_aircraft_variable("PAYLOAD STATION WEIGHT", "Pounds", 14)?
    .provides_aircraft_variable("PAYLOAD STATION WEIGHT", "Pounds", 15)?
    .provides_aircraft_variable("PAYLOAD STATION WEIGHT", "Pounds", 16)?
    .provides_aircraft_variable("PAYLOAD STATION WEIGHT", "Pounds", 17)?
    .provides_aircraft_variable("PAYLOAD STATION WEIGHT", "Pounds", 18)?
    .provides_named_variable("FSDT_GSX_BOARDING_STATE")?
    .provides_named_variable("FSDT_GSX_DEBOARDING_STATE")?
    .provides_named_variable("FSDT_GSX_NUMPASSENGERS_BOARDING_TOTAL")?
    .provides_named_variable("FSDT_GSX_NUMPASSENGERS_DEBOARDING_TOTAL")?
    .provides_named_variable("FSDT_GSX_BOARDING_CARGO_PERCENT")?
    .provides_named_variable("FSDT_GSX_DEBOARDING_CARGO_PERCENT")?
    .provides_named_variable("FSDT_GSX_BYPASS_PIN")?
    .with_aspect(|builder| {
        builder.copy(
            Variable::named("FSDT_GSX_BYPASS_PIN"),
            Variable::aspect("EXTERNAL_BYPASS_PIN_INSERTED"),
        );
        Ok(())
    })?
    .with_aspect(|builder| {
        for i in 1..=2 {
            builder.copy(
                Variable::aircraft("APU GENERATOR SWITCH", "Bool", i),
                Variable::aspect(&format!("OVHD_ELEC_APU_GEN_{i}_PB_IS_ON")),
            );
        }

        for i in 1..=4 {
            builder.copy(
                Variable::named(&format!("EXT_PWR_AVAIL:{i}")),
                Variable::aspect(&format!("OVHD_ELEC_EXT_PWR_{i}_PB_IS_AVAILABLE")),
            );

            builder.copy(
                Variable::aircraft("GENERAL ENG MASTER ALTERNATOR", "Bool", i),
                Variable::aspect(&format!("OVHD_ELEC_ENG_GEN_{i}_PB_IS_ON")),
            );
        }

        Ok(())
    })?
    .with_aspect(reversers)?
    .with_aspect(brakes)?
    .with_aspect(cargo_doors)?
    .with_aspect(autobrakes)?
    .with_aspect(nose_wheel_steering)?
    .with_aspect(body_wheel_steering)?
    .with_aspect(fire)?
    .with_aspect(flaps)?
    .with_aspect(spoilers)?
    .with_aspect(ailerons)?
    .with_aspect(elevators)?
    .with_aspect(rudder)?
    .with_aspect(gear)?
    .with_aspect(payload)?
    .with_aspect(fuel)?
    .with_aspect(trimmable_horizontal_stabilizer)?
    .build(A380::new)?;

    while let Some(event) = gauge.next_event().await {
        handler.handle(event, &mut simulation, sim_connect.as_mut().get_mut())?;
    }

    Ok(())
}
