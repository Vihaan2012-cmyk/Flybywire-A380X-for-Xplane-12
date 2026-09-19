//! This area's registration into the shared failure/component/ECAM API
//! (`docs/deep/BRIEF.md`'s "Registering failures, components and ECAM
//! alerts"): every physical fault this flight model accepts, in code, so
//! the lead's `Registry::validate()` and the Components/Failures pages
//! find them without any of this described only in prose.
//!
//! Every id uses `Area::FlightModel` (this agent's area code); the `ata`
//! passed to `failure_id`/`ComponentDef` is the real ATA chapter the part
//! belongs to (27 flight controls, 32 landing gear, 30 ice and rain
//! protection, 57 wings), so the Components/Failures pages group this
//! agent's entries next to any other area's work on the same chapter.
//!
//! This flight model raises no ECAM alerts of its own (it is a physics
//! layer underneath the systems that would raise them -- e.g. a real
//! "F/CTL AIL JAM" alert belongs to whichever area owns the flight-control
//! computers reading these fault flags), so only `component`/`failure` are
//! registered here, per the brief's "ECAM.md only if you add alerts" note
//! (translated to this code-registration pass: no `r.alert(...)` calls).

use fbw_a380_systems::deep::api::*;

/// One flight-control surface's three fault modes -- see `actuator.rs`'s
/// module doc for the physical meaning of each field, and
/// `aerodynamics.rs`'s `AeroFaults` for exactly which struct field each
/// drives. `max_rate_deg_s` is the surface's own travel limit used to turn
/// the runaway failures' 0..1 magnitude into a signed rad/s rate.
struct Surface {
    id: &'static str,
    name: &'static str,
    field: &'static str,
}
const SURFACES: [Surface; 7] = [
    Surface { id: "27_fctl.elevator_left", name: "Left elevator", field: "elevator_left" },
    Surface { id: "27_fctl.elevator_right", name: "Right elevator", field: "elevator_right" },
    Surface { id: "27_fctl.aileron_left", name: "Left aileron", field: "aileron_left" },
    Surface { id: "27_fctl.aileron_right", name: "Right aileron", field: "aileron_right" },
    Surface { id: "27_fctl.rudder", name: "Rudder", field: "rudder" },
    Surface { id: "27_fctl.spoiler_left", name: "Left spoiler panel", field: "spoiler_left" },
    Surface { id: "27_fctl.spoiler_right", name: "Right spoiler panel", field: "spoiler_right" },
];

#[derive(Clone, Copy)]
enum Leg {
    Nose,
    WingLeft,
    WingRight,
    BodyLeft,
    BodyRight,
}
struct GearLegDef {
    leg: Leg,
    id: &'static str,
    name: &'static str,
    index: usize,
    braked: bool,
    steerable: bool,
}
const GEAR_LEGS: [GearLegDef; 5] = [
    GearLegDef { leg: Leg::Nose, id: "32_gear.nose", name: "Nose gear leg", index: 0, braked: false, steerable: true },
    GearLegDef { leg: Leg::WingLeft, id: "32_gear.wing_left", name: "Left wing gear leg", index: 1, braked: true, steerable: false },
    GearLegDef { leg: Leg::WingRight, id: "32_gear.wing_right", name: "Right wing gear leg", index: 2, braked: true, steerable: false },
    GearLegDef { leg: Leg::BodyLeft, id: "32_gear.body_left", name: "Left body gear leg", index: 3, braked: true, steerable: false },
    GearLegDef { leg: Leg::BodyRight, id: "32_gear.body_right", name: "Right body gear leg", index: 4, braked: true, steerable: false },
];

pub fn register(r: &mut Registry) {
    register_flight_controls(r);
    register_landing_gear(r);
    register_aero_environment(r);
}

fn register_flight_controls(r: &mut Registry) {
    const ATA: u16 = 27;
    let mut n: u16 = 1;
    for s in &SURFACES {
        r.component(ComponentDef {
            id: s.id.to_owned(),
            area: Area::FlightModel,
            ata: ATA,
            name: s.name.to_owned(),
            params: vec![
                ParamDef { name: "jam_fraction".into(), meaning: "0 = free .. 1 = fully seized (frozen wherever it is)".into(), healthy: 0.0 },
                ParamDef { name: "float_fraction".into(), meaning: "0 = full actuator authority .. 1 = fully floating (unpowered)".into(), healthy: 0.0 },
                ParamDef { name: "runaway_active".into(), meaning: "0 = none .. 1 = driving hardover at full rate (either direction)".into(), healthy: 0.0 },
            ],
            failures: Vec::new(),
        });

        let jam = r.failure(FailureDef {
            id: failure_id(Area::FlightModel, ATA, n),
            area: Area::FlightModel,
            ata: ATA,
            name: format!("{} jam", s.name),
            component: s.id.into(),
            model_field: format!("emulator/src/flight_model/aerodynamics.rs::AeroFaults.{}.jam_fraction", s.field),
            magnitude: "0 = free .. 1 = fully seized; the achievable actuator rate scales as (1-magnitude), so it freezes at whatever position it had reached, not a chosen one".into(),
            effect: "reduced-to-zero authority on this surface; asymmetric (one side only) also couples into roll/yaw".into(),
        });
        n += 1;
        let float = r.failure(FailureDef {
            id: failure_id(Area::FlightModel, ATA, n),
            area: Area::FlightModel,
            ata: ATA,
            name: format!("{} hydraulic/electrical power loss (float)", s.name),
            component: s.id.into(),
            model_field: format!("emulator/src/flight_model/aerodynamics.rs::AeroFaults.{}.float_fraction", s.field),
            magnitude: "0 = full authority .. 1 = fully floating (commanded target blended fully to zero deflection)".into(),
            effect: "lost control authority on this surface; the surface aerodynamically streamlines".into(),
        });
        n += 1;
        let runaway_up = r.failure(FailureDef {
            id: failure_id(Area::FlightModel, ATA, n),
            area: Area::FlightModel,
            ata: ATA,
            name: format!("{} runaway toward its positive travel limit", s.name),
            component: s.id.into(),
            model_field: format!("emulator/src/flight_model/aerodynamics.rs::AeroFaults.{}.runaway_rate_rad_s", s.field),
            magnitude: "0 = healthy .. 1 = drives at the surface's full rate limit toward +limit regardless of command".into(),
            effect: "uncommanded control input toward one travel limit (a failed control valve/amplifier driving hardover)".into(),
        });
        n += 1;
        let runaway_down = r.failure(FailureDef {
            id: failure_id(Area::FlightModel, ATA, n),
            area: Area::FlightModel,
            ata: ATA,
            name: format!("{} runaway toward its negative travel limit", s.name),
            component: s.id.into(),
            model_field: format!("emulator/src/flight_model/aerodynamics.rs::AeroFaults.{}.runaway_rate_rad_s", s.field),
            magnitude: "0 = healthy .. 1 = drives at the surface's full rate limit toward -limit regardless of command".into(),
            effect: "uncommanded control input toward the opposite travel limit".into(),
        });
        n += 1;
        let _ = (jam, float, runaway_up, runaway_down);
    }
}

fn register_landing_gear(r: &mut Registry) {
    const ATA: u16 = 32;
    let mut n: u16 = 1;
    for g in &GEAR_LEGS {
        let mut params = vec![
            ParamDef { name: "collapse_fraction".into(), meaning: "0 = healthy strut .. 1 = fully collapsed (no vertical support)".into(), healthy: 0.0 },
            ParamDef { name: "tyre_friction_loss_fraction".into(), meaning: "0 = full grip .. 1 = no grip (a blown/deflated tyre)".into(), healthy: 0.0 },
        ];
        if g.braked {
            params.push(ParamDef { name: "brake_fade_fraction".into(), meaning: "0 = full brake authority .. 1 = fully faded".into(), healthy: 0.0 });
        }
        if g.steerable {
            params.push(ParamDef { name: "steering_fault".into(), meaning: "0 = healthy .. 1 = steering jammed/floating/running away".into(), healthy: 0.0 });
        }
        r.component(ComponentDef { id: g.id.to_owned(), area: Area::FlightModel, ata: ATA, name: g.name.to_owned(), params, failures: Vec::new() });

        r.failure(FailureDef {
            id: failure_id(Area::FlightModel, ATA, n),
            area: Area::FlightModel,
            ata: ATA,
            name: format!("{} structural collapse", g.name),
            component: g.id.into(),
            model_field: format!("emulator/src/flight_model/landing_gear.rs::GearFaults.collapse_fraction[{}]", g.index),
            magnitude: "0 = healthy .. 1 = fully collapsed; vertical reaction scales as (1-magnitude)^2".into(),
            effect: "this leg stops carrying weight; the airframe settles onto the remaining legs (and possibly the ground/fuselage)".into(),
        });
        n += 1;
        r.failure(FailureDef {
            id: failure_id(Area::FlightModel, ATA, n),
            area: Area::FlightModel,
            ata: ATA,
            name: format!("{} tyre burst/deflation", g.name),
            component: g.id.into(),
            model_field: format!("emulator/src/flight_model/landing_gear.rs::GearFaults.tyre_friction_loss_fraction[{}]", g.index),
            magnitude: "0 = full grip .. 1 = no grip".into(),
            effect: "reduced braking/cornering force on this leg; asymmetric bursts pull the aircraft toward the good side under braking".into(),
        });
        n += 1;
        if g.braked {
            r.failure(FailureDef {
                id: failure_id(Area::FlightModel, ATA, n),
                area: Area::FlightModel,
                ata: ATA,
                name: format!("{} brake fade", g.name),
                component: g.id.into(),
                model_field: format!("emulator/src/flight_model/landing_gear.rs::GearFaults.brake_fade_fraction[{}]", g.index),
                magnitude: "0 = full brake authority .. 1 = fully faded (overheated brakes)".into(),
                effect: "longer stopping distance; asymmetric fade yaws the aircraft under braking".into(),
            });
            n += 1;
        }
        if g.steerable {
            r.failure(FailureDef {
                id: failure_id(Area::FlightModel, ATA, n),
                area: Area::FlightModel,
                ata: ATA,
                name: format!("{} steering jam", g.name),
                component: g.id.into(),
                model_field: "emulator/src/flight_model/landing_gear.rs::GearFaults.nose_steering.jam_fraction".into(),
                magnitude: "0 = free .. 1 = fully seized at its current angle".into(),
                effect: "loss of nosewheel steering authority for ground manoeuvring".into(),
            });
            n += 1;
            r.failure(FailureDef {
                id: failure_id(Area::FlightModel, ATA, n),
                area: Area::FlightModel,
                ata: ATA,
                name: format!("{} steering runaway", g.name),
                component: g.id.into(),
                model_field: "emulator/src/flight_model/landing_gear.rs::GearFaults.nose_steering.runaway_rate_rad_s".into(),
                magnitude: "0 = healthy .. 1 = drives at full rate to a steering hardstop regardless of command".into(),
                effect: "uncommanded nosewheel steering input during taxi/takeoff/landing roll".into(),
            });
            n += 1;
        }
        let _ = g.leg; // identifies which leg this is for readers; not otherwise used here.
    }
}

fn register_aero_environment(r: &mut Registry) {
    r.component(ComponentDef {
        id: "30_ice.wing".into(),
        area: Area::FlightModel,
        ata: 30,
        name: "Wing leading-edge ice accretion".into(),
        params: vec![ParamDef { name: "ice_fraction".into(), meaning: "0 = clean .. 1 = severe accretion".into(), healthy: 0.0 }],
        failures: Vec::new(),
    });
    r.failure(FailureDef {
        id: failure_id(Area::FlightModel, 30, 1),
        area: Area::FlightModel,
        ata: 30,
        name: "Wing ice accretion".into(),
        component: "30_ice.wing".into(),
        model_field: "emulator/src/flight_model/aerodynamics.rs::AeroFaults.wing_ice_fraction".into(),
        magnitude: "0 = clean .. 1 = severe: CLmax reduced up to 50%, lift-curve slope reduced up to 40%, parasite drag increased".into(),
        effect: "higher stall speed, reduced climb/manoeuvre margin, earlier and harder stall".into(),
    });

    r.component(ComponentDef {
        id: "30_ice.htail".into(),
        area: Area::FlightModel,
        ata: 30,
        name: "Horizontal tailplane ice accretion".into(),
        params: vec![ParamDef { name: "ice_fraction".into(), meaning: "0 = clean .. 1 = severe accretion".into(), healthy: 0.0 }],
        failures: Vec::new(),
    });
    r.failure(FailureDef {
        id: failure_id(Area::FlightModel, 30, 2),
        area: Area::FlightModel,
        ata: 30,
        name: "Tailplane ice accretion".into(),
        component: "30_ice.htail".into(),
        model_field: "emulator/src/flight_model/aerodynamics.rs::AeroFaults.tail_ice_fraction".into(),
        magnitude: "0 = clean .. 1 = severe: tail CLmax and lift-curve slope both reduced".into(),
        effect: "the classic tailplane-icing hazard: reduced pitch authority and an earlier, more sudden tail stall (can pitch the nose down uncommanded), especially with flaps extended".into(),
    });

    r.component(ComponentDef {
        id: "57_wing.airframe".into(),
        area: Area::FlightModel,
        ata: 57,
        name: "Airframe aerodynamic damage (skin/rivets/fairings)".into(),
        params: vec![ParamDef { name: "damage_fraction".into(), meaning: "0 = undamaged .. 1 = severe (hail/bird-strike denting, missing fairings, skin damage)".into(), healthy: 0.0 }],
        failures: Vec::new(),
    });
    r.failure(FailureDef {
        id: failure_id(Area::FlightModel, 57, 1),
        area: Area::FlightModel,
        ata: 57,
        name: "Airframe aerodynamic damage".into(),
        component: "57_wing.airframe".into(),
        model_field: "emulator/src/flight_model/aerodynamics.rs::AeroFaults.airframe_damage_fraction".into(),
        magnitude: "0 = undamaged .. 1 = severe: CLmax reduced up to 30%, lift-curve slope reduced up to 20%, parasite drag increased up to 80%".into(),
        effect: "higher stall speed, reduced range/climb performance, generally degraded handling".into(),
    });
}
