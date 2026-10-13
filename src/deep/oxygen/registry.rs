//! Registers this area's components, failures and ECAM alerts with the
//! shared deep-systems catalogue (`crate::deep::api`), under
//! [`Area::Oxygen`] and ATA 35 ("Oxygen" -- the standard ATA-100 chapter,
//! not a GENERIC assignment).
//!
//! ## Ids live here
//!
//! [`ids`] carries one named constant per failure, and both this file and
//! `live.rs` use them. That is deliberate: a live system that resolved
//! failures by position, or by re-deriving the numbering, could drift away
//! from the catalogue without anything noticing. `live.rs`'s own test
//! walks the registry and asserts that every registered id is one the
//! live system actually reads, which is only meaningful because there is
//! exactly one place the numbers are written down.
//!
//! ## ECAM titles
//!
//! `OXYGEN CKPT SYS LO PR` and `OXYGEN PAX SYS ON` are the wordings
//! published for the Airbus ATA 35 alerts. The third
//! (`OXYGEN CREW SUPPLY LO PR`) is **GENERIC** representative Airbus-style
//! wording for a low-pressure-distribution failure -- the same labelling
//! `wiring::registry` applies to alert text it could not source, and not a
//! claim that this string appears in an A380 FCOM.
//!
//! ## What is deliberately not an alert
//!
//! A generator burning inside a stowed PSU is the most dangerous single
//! thing in this chapter, and it has no alert here, because the real
//! aeroplane has no sensor on a generator case. What it has is a cabin
//! that gets hot and smoke detectors that eventually see it -- which is
//! why this area publishes its heat in watts per deck rather than
//! inventing an annunciation for it.

use crate::deep::api::*;

/// Every failure id this area owns, by name. `n` is sequential within
/// ATA 35 in the order they are registered.
pub mod ids {
    use crate::deep::api::{failure_id, Area};

    pub const CREW_CYLINDER_LEAK: u64 = failure_id(Area::Oxygen, 35, 1);
    pub const CREW_CYLINDER_DISC_RUPTURE: u64 = failure_id(Area::Oxygen, 35, 2);
    pub const CREW_SUPPLY_VALVE_SEIZED: u64 = failure_id(Area::Oxygen, 35, 3);
    pub const CREW_REDUCER_SETPOINT_LOW: u64 = failure_id(Area::Oxygen, 35, 4);
    pub const CREW_REDUCER_SETPOINT_HIGH: u64 = failure_id(Area::Oxygen, 35, 5);
    pub const CREW_REDUCER_SEAT_LEAK: u64 = failure_id(Area::Oxygen, 35, 6);
    pub const CREW_DISTRIBUTION_LEAK: u64 = failure_id(Area::Oxygen, 35, 7);
    /// One per flight-deck station, 1..=4.
    pub const CREW_MASK_DILUTER_STUCK: [u64; 4] =
        [failure_id(Area::Oxygen, 35, 8), failure_id(Area::Oxygen, 35, 9), failure_id(Area::Oxygen, 35, 10), failure_id(Area::Oxygen, 35, 11)];
    pub const PAX_DUD_INITIATORS: u64 = failure_id(Area::Oxygen, 35, 12);
    pub const PAX_CANDLE_QUENCH: u64 = failure_id(Area::Oxygen, 35, 13);
    pub const PAX_INADVERTENT_IGNITION: u64 = failure_id(Area::Oxygen, 35, 14);
    pub const PAX_LATCH_FAILED: u64 = failure_id(Area::Oxygen, 35, 15);
    pub const PAX_AUTO_DEPLOY_CONTROLLER: u64 = failure_id(Area::Oxygen, 35, 16);
    pub const THERAPEUTIC_CYLINDER_LEAK: u64 = failure_id(Area::Oxygen, 35, 17);
    pub const THERAPEUTIC_CYLINDER_DISC_RUPTURE: u64 = failure_id(Area::Oxygen, 35, 18);
    pub const THERAPEUTIC_REDUCER_SETPOINT_LOW: u64 = failure_id(Area::Oxygen, 35, 19);
    pub const THERAPEUTIC_REDUCER_SEAT_LEAK: u64 = failure_id(Area::Oxygen, 35, 20);
    pub const THERAPEUTIC_OUTLET_STUCK_OPEN: u64 = failure_id(Area::Oxygen, 35, 21);

    /// Every id above, for the cross-checks in `registry.rs` and
    /// `live.rs`.
    pub fn all() -> Vec<u64> {
        let mut v = vec![
            CREW_CYLINDER_LEAK,
            CREW_CYLINDER_DISC_RUPTURE,
            CREW_SUPPLY_VALVE_SEIZED,
            CREW_REDUCER_SETPOINT_LOW,
            CREW_REDUCER_SETPOINT_HIGH,
            CREW_REDUCER_SEAT_LEAK,
            CREW_DISTRIBUTION_LEAK,
        ];
        v.extend_from_slice(&CREW_MASK_DILUTER_STUCK);
        v.extend_from_slice(&[
            PAX_DUD_INITIATORS,
            PAX_CANDLE_QUENCH,
            PAX_INADVERTENT_IGNITION,
            PAX_LATCH_FAILED,
            PAX_AUTO_DEPLOY_CONTROLLER,
            THERAPEUTIC_CYLINDER_LEAK,
            THERAPEUTIC_CYLINDER_DISC_RUPTURE,
            THERAPEUTIC_REDUCER_SETPOINT_LOW,
            THERAPEUTIC_REDUCER_SEAT_LEAK,
            THERAPEUTIC_OUTLET_STUCK_OPEN,
        ]);
        v
    }
}

fn param(name: &str, meaning: &str) -> ParamDef {
    ParamDef { name: name.to_string(), meaning: meaning.to_string(), healthy: 0.0 }
}

struct Fail {
    id: u64,
    name: &'static str,
    component: &'static str,
    model_field: &'static str,
    magnitude: &'static str,
    effect: &'static str,
}

fn register_failures(r: &mut Registry, failures: &[Fail]) {
    for f in failures {
        r.failure(FailureDef {
            id: f.id,
            area: Area::Oxygen,
            ata: 35,
            name: f.name.to_string(),
            component: f.component.to_string(),
            model_field: f.model_field.to_string(),
            magnitude: f.magnitude.to_string(),
            effect: f.effect.to_string(),
        });
    }
}

fn component(r: &mut Registry, id: &str, name: &str, params: Vec<ParamDef>, failures: Vec<u64>) {
    r.component(ComponentDef { id: id.to_string(), area: Area::Oxygen, ata: 35, name: name.to_string(), params, failures });
}

pub fn register(r: &mut Registry) {
    register_crew(r);
    register_pax(r);
    register_therapeutic(r);
    register_alerts(r);
}

// ---------------------------------------------------------------------
// 35-10: flight crew oxygen.
// ---------------------------------------------------------------------

fn register_crew(r: &mut Registry) {
    component(
        r,
        "35_oxy.crew_cylinder",
        "Crew oxygen cylinder group (2 x 3260 L free air, 1850 psig)",
        vec![
            param("leak_orifice", "equivalent leak orifice area, 0 = sound .. 1 = 1 mm^2 hole (cylinder::LEAK_FULL_SCALE_M2)"),
            param("disc_degradation", "how far the overpressure discharge disc's rupture pressure has fallen below its 2775 psig rating, 0 = rated .. 1 = ruptures at ambient"),
        ],
        vec![ids::CREW_CYLINDER_LEAK, ids::CREW_CYLINDER_DISC_RUPTURE],
    );
    component(
        r,
        "35_oxy.crew_supply_valve",
        "Crew oxygen supply shutoff valve (motor operated, DC 1)",
        vec![param("travel_lost", "fraction of the valve's travel it can no longer reach, 0 = free .. 1 = seized on its seat")],
        vec![ids::CREW_SUPPLY_VALVE_SEIZED],
    );
    component(
        r,
        "35_oxy.crew_reducer",
        "Crew oxygen pressure reducer (85 psi low-pressure distribution)",
        vec![
            param("setpoint_error", "signed shift of the outlet setpoint as a fraction of it, 0 = on setting"),
            param("seat_leak", "leak past the reducing poppet, 0 = tight .. 1 = regulator::SEAT_LEAK_FULL_SCALE_M2"),
        ],
        vec![ids::CREW_REDUCER_SETPOINT_LOW, ids::CREW_REDUCER_SETPOINT_HIGH, ids::CREW_REDUCER_SEAT_LEAK],
    );
    component(
        r,
        "35_oxy.crew_distribution",
        "Crew oxygen low-pressure distribution and mask hoses",
        vec![param("leak_orifice", "equivalent leak orifice area in the low-pressure side, 0 = sound .. 1 = crew::DISTRIBUTION_LEAK_FULL_SCALE_M2")],
        vec![ids::CREW_DISTRIBUTION_LEAK],
    );
    for (i, id) in ids::CREW_MASK_DILUTER_STUCK.iter().enumerate() {
        let station = i + 1;
        component(
            r,
            &format!("35_oxy.crew_mask_regulator_{station}"),
            &format!("Crew oxygen mask diluter-demand regulator, station {station}"),
            vec![param("diluter_jam", "how far the diluter is jammed toward its cabin-air inlet, 0 = schedules normally .. 1 = delivers cabin air at any altitude")],
            vec![*id],
        );
    }

    register_failures(
        r,
        &[
            Fail {
                id: ids::CREW_CYLINDER_LEAK,
                name: "Crew oxygen cylinder leak",
                component: "35_oxy.crew_cylinder",
                model_field: "cylinder::CylinderFaults.leak",
                magnitude: "0 .. 1 of a 1 mm^2 equivalent orifice (not a 0..1 loss fraction)",
                effect: "oxygen escapes through a choked orifice, so indicated bottle pressure falls steadily and the temperature-corrected reading falls with it; at full magnitude the group is empty in about twenty minutes and the low-pressure caution comes up on the way",
            },
            Fail {
                id: ids::CREW_CYLINDER_DISC_RUPTURE,
                name: "Crew oxygen overpressure disc degraded",
                component: "35_oxy.crew_cylinder",
                model_field: "cylinder::CylinderFaults.disc_weakened",
                magnitude: "0 .. 1 reduction of the 2775 psig rupture pressure",
                effect: "above about 0.34 the disc ruptures at or below the cylinder's own charge pressure and dumps the whole group overboard through its 3 mm bore in under a minute; below that it still ruptures early if the bay heats the bottle, which is the case it exists for",
            },
            Fail {
                id: ids::CREW_SUPPLY_VALVE_SEIZED,
                name: "Crew oxygen supply valve seized",
                component: "35_oxy.crew_supply_valve",
                model_field: "crew::CrewOxygenFaults.valve_jam",
                magnitude: "0 .. 1 of the valve's travel lost; the supply only starves in the last few percent, because a partly open shutoff valve still passes far more than this system draws",
                effect: "at full magnitude the low-pressure distribution falls to zero and the mask regulators stop delivering altogether, while the bottle stays full -- the quantity gauge cannot see this one",
            },
            Fail {
                id: ids::CREW_REDUCER_SETPOINT_LOW,
                name: "Crew oxygen reducer setting low",
                component: "35_oxy.crew_reducer",
                model_field: "regulator::RegulatorFaults.setpoint_shift (negative)",
                magnitude: "0 .. 1 of the setpoint lost, so 1 delivers nothing",
                effect: "distribution pressure falls below what a demand regulator needs to open; past about half setpoint the masks deliver nothing at all rather than delivering something thin",
            },
            Fail {
                id: ids::CREW_REDUCER_SETPOINT_HIGH,
                name: "Crew oxygen reducer setting high",
                component: "35_oxy.crew_reducer",
                model_field: "regulator::RegulatorFaults.setpoint_shift (positive)",
                magnitude: "0 .. 1 added to the setpoint, so 1 doubles it",
                effect: "distribution pressure rises until the low-pressure relief lifts at 1.5 times setpoint and holds it there, venting the excess overboard",
            },
            Fail {
                id: ids::CREW_REDUCER_SEAT_LEAK,
                name: "Crew oxygen reducer seat leak",
                component: "35_oxy.crew_reducer",
                model_field: "regulator::RegulatorFaults.seat_leak",
                magnitude: "0 .. 1 of a 0.01 mm^2 equivalent orifice past the poppet",
                effect: "cylinder pressure bleeds continuously into the low-pressure side and out through its relief with nobody breathing any of it; a full group empties overnight, so the signature is a bottle that was full yesterday and is not today",
            },
            Fail {
                id: ids::CREW_DISTRIBUTION_LEAK,
                name: "Crew oxygen distribution or mask hose leak",
                component: "35_oxy.crew_distribution",
                model_field: "crew::CrewOxygenFaults.distribution_leak",
                magnitude: "0 .. 1 of a 0.5 mm^2 equivalent orifice at 85 psi",
                effect: "the low-pressure side leaks to the flight deck whether or not a mask is donned, draining the cylinder over about five hours at full magnitude",
            },
        ],
    );
    for (i, id) in ids::CREW_MASK_DILUTER_STUCK.iter().enumerate() {
        let station = i + 1;
        r.failure(FailureDef {
            id: *id,
            area: Area::Oxygen,
            ata: 35,
            name: format!("Crew mask {station} diluter jammed to ambient"),
            component: format!("35_oxy.crew_mask_regulator_{station}"),
            model_field: format!("crew::CrewOxygenFaults.dilution_stuck_ambient[{i}]"),
            magnitude: "0 .. 1, how far the diluter is jammed toward its cabin-air inlet".into(),
            effect: format!(
                "station {station}'s delivered oxygen fraction falls from the alveolar-solved schedule toward 0.2095 -- the mask breathes normally and contains cabin air, so it is silent at altitude, and the bottle stops being drawn down, which is why the quantity gauge cannot catch it either"
            ),
        });
    }
}

// ---------------------------------------------------------------------
// 35-20: passenger oxygen (chemical generators).
// ---------------------------------------------------------------------

fn register_pax(r: &mut Registry) {
    component(
        r,
        "35_oxy.pax_generators",
        "Passenger chemical oxygen generators (cabin installation)",
        vec![
            param("dud_initiators", "fraction of units whose percussion initiator will not light the candle"),
            param("candle_quench", "how far short of the end of its burn the reaction front quenches, 0 = full 15 minutes .. 1 = never propagates"),
            param("spontaneous_ignition", "fraction of units liable to light with nothing pulling on them"),
        ],
        vec![ids::PAX_DUD_INITIATORS, ids::PAX_CANDLE_QUENCH, ids::PAX_INADVERTENT_IGNITION],
    );
    component(
        r,
        "35_oxy.pax_psu_latches",
        "Passenger service unit oxygen door latches",
        vec![param("latch_seized", "fraction of PSU doors that will not release when commanded")],
        vec![ids::PAX_LATCH_FAILED],
    );
    component(
        r,
        "35_oxy.pax_deploy_controller",
        "Passenger oxygen automatic deployment controller (DC ESS)",
        vec![param("controller_failed", "how much of the automatic altitude-triggered deployment is lost, 0 = healthy .. 1 = only the manual command works")],
        vec![ids::PAX_AUTO_DEPLOY_CONTROLLER],
    );

    register_failures(
        r,
        &[
            Fail {
                id: ids::PAX_DUD_INITIATORS,
                name: "Passenger oxygen generator initiators dud",
                component: "35_oxy.pax_generators",
                model_field: "pax::PassengerOxygenFaults.dud_initiators",
                magnitude: "0 .. 1, the fraction of units that will not light",
                effect: "the masks still present -- that is the latch's job -- and that fraction of them delivers nothing when pulled; cabin oxygen flow and the generators' heat both fall in proportion, and the unfired generators are still full",
            },
            Fail {
                id: ids::PAX_CANDLE_QUENCH,
                name: "Passenger oxygen candle quenches early",
                component: "35_oxy.pax_generators",
                model_field: "pax::PassengerOxygenFaults.candle_quench",
                magnitude: "0 .. 1, the fraction of the candle the front fails to reach",
                effect: "the burn stops early, so the cabin's oxygen stops before the rated 15 minutes are up and unburnt candle is left behind; at 0.5 the supply runs out after about seven and a half minutes",
            },
            Fail {
                id: ids::PAX_INADVERTENT_IGNITION,
                name: "Passenger oxygen generator inadvertent ignition",
                component: "35_oxy.pax_generators",
                model_field: "pax::PassengerOxygenFaults.inadvertent_ignition",
                magnitude: "0 .. 1, the fraction of units that light unbidden",
                effect: "those units burn inside a closed PSU with no masks presented, reaching a case temperature near 300 C and putting their full chemical heat into the cabin zone until the candle is gone; the supply they represent is spent and cannot be recovered in flight",
            },
            Fail {
                id: ids::PAX_LATCH_FAILED,
                name: "Passenger oxygen PSU latches seized",
                component: "35_oxy.pax_psu_latches",
                model_field: "pax::PassengerOxygenFaults.latch_failed",
                magnitude: "0 .. 1, the fraction of PSU doors that stay shut",
                effect: "that fraction of the cabin gets no mask at all when the system deploys, so no generator in it is ever lit; the deployed fraction, the oxygen flow and the generator heat all fall together",
            },
            Fail {
                id: ids::PAX_AUTO_DEPLOY_CONTROLLER,
                name: "Passenger oxygen automatic deployment controller failed",
                component: "35_oxy.pax_deploy_controller",
                model_field: "pax::PassengerOxygenFaults.auto_deploy_controller",
                magnitude: "0 .. 1 of the automatic deployment authority lost",
                effect: "at full magnitude the cabin altitude trigger never presents the masks, however high the cabin goes; the flight deck's manual command is a separate path and still works, which is exactly why it is there",
            },
        ],
    );
}

// ---------------------------------------------------------------------
// 35-30: first-aid (therapeutic) oxygen.
// ---------------------------------------------------------------------

fn register_therapeutic(r: &mut Registry) {
    component(
        r,
        "35_oxy.therapeutic_cylinder",
        "First-aid oxygen cylinder (2772 L free air, 1800 psig)",
        vec![
            param("leak_orifice", "equivalent leak orifice area, 0 = sound .. 1 = 1 mm^2 hole"),
            param("disc_degradation", "how far the overpressure discharge disc's rupture pressure has fallen below its 2700 psig rating"),
        ],
        vec![ids::THERAPEUTIC_CYLINDER_LEAK, ids::THERAPEUTIC_CYLINDER_DISC_RUPTURE],
    );
    component(
        r,
        "35_oxy.therapeutic_regulator",
        "First-aid oxygen continuous-flow regulator (50 psi, 2/4 L/min outlets)",
        vec![
            param("setpoint_error", "signed shift of the delivery pressure as a fraction of setpoint"),
            param("seat_leak", "leak past the reducing poppet"),
        ],
        vec![ids::THERAPEUTIC_REDUCER_SETPOINT_LOW, ids::THERAPEUTIC_REDUCER_SEAT_LEAK],
    );
    component(
        r,
        "35_oxy.therapeutic_outlets",
        "First-aid oxygen cabin outlets",
        vec![param("stuck_open", "fraction of outlets flowing with nothing plugged into them")],
        vec![ids::THERAPEUTIC_OUTLET_STUCK_OPEN],
    );

    register_failures(
        r,
        &[
            Fail {
                id: ids::THERAPEUTIC_CYLINDER_LEAK,
                name: "First-aid oxygen cylinder leak",
                component: "35_oxy.therapeutic_cylinder",
                model_field: "cylinder::CylinderFaults.leak (therapeutic cylinder)",
                magnitude: "0 .. 1 of a 1 mm^2 equivalent orifice",
                effect: "indicated first-aid bottle pressure falls steadily; once it drops below about half the 50 psi delivery setting the outlets stop flowing entirely, and the cabin has no therapeutic oxygen at all",
            },
            Fail {
                id: ids::THERAPEUTIC_CYLINDER_DISC_RUPTURE,
                name: "First-aid oxygen overpressure disc degraded",
                component: "35_oxy.therapeutic_cylinder",
                model_field: "cylinder::CylinderFaults.disc_weakened (therapeutic cylinder)",
                magnitude: "0 .. 1 reduction of the 2700 psig rupture pressure",
                effect: "past about a third the disc ruptures at the cylinder's own charge pressure and dumps it overboard in well under a minute",
            },
            Fail {
                id: ids::THERAPEUTIC_REDUCER_SETPOINT_LOW,
                name: "First-aid oxygen regulator setting low",
                component: "35_oxy.therapeutic_regulator",
                model_field: "regulator::RegulatorFaults.setpoint_shift (negative, therapeutic)",
                magnitude: "0 .. 1 of the 50 psi setting lost",
                effect: "delivery pressure falls; below half setting the continuous-flow outlets stop delivering, with the bottle still full",
            },
            Fail {
                id: ids::THERAPEUTIC_REDUCER_SEAT_LEAK,
                name: "First-aid oxygen regulator seat leak",
                component: "35_oxy.therapeutic_regulator",
                model_field: "regulator::RegulatorFaults.seat_leak (therapeutic)",
                magnitude: "0 .. 1 of a 0.01 mm^2 equivalent orifice past the poppet",
                effect: "the cylinder bleeds down over hours into the low-pressure side with nobody using it, so the bottle is found short at the next check",
            },
            Fail {
                id: ids::THERAPEUTIC_OUTLET_STUCK_OPEN,
                name: "First-aid oxygen outlet stuck open",
                component: "35_oxy.therapeutic_outlets",
                model_field: "therapeutic::TherapeuticFaults.outlets_stuck_open",
                magnitude: "0 .. 1, the fraction of the installed outlets flowing unattended",
                effect: "a continuous-flow outlet does not care whether anyone is breathing through it, so each stuck outlet drains the cylinder at its full 4 L/min and the supply is gone long before it is wanted",
            },
        ],
    );
}

// ---------------------------------------------------------------------
// ECAM.
// ---------------------------------------------------------------------

fn register_alerts(r: &mut Registry) {
    r.alert(
        EcamAlert::new("OXY_CKPT_SYS_LO_PR", 35, "OXYGEN CKPT SYS LO PR", Level::Caution, var("DEEP_OXY_CREW_LOW_PRESSURE").on())
            .confirm(5.0)
            .inhibit(&[Phase::FirstEngineTakeoffPower, Phase::Above80Kt, Phase::LiftOff, Phase::Below800Ft, Phase::Touchdown])
            .step(line("CKPT OXY", "CHECK AVAIL").colour("white"))
            .step(line("MAX FL", "100/MEA").colour("white"))
            .status_line("CREW OXY - LO PR")
            .inop_sys("CKPT OXY")
            .raised_by(&[ids::CREW_CYLINDER_LEAK, ids::CREW_CYLINDER_DISC_RUPTURE, ids::CREW_REDUCER_SEAT_LEAK, ids::CREW_DISTRIBUTION_LEAK]),
    );

    // The distribution can be dead with the cylinder full, and that is a
    // different message with a different cause list: the crew's masks are
    // gone even though the quantity gauge is reading normal.
    r.alert(
        EcamAlert::new(
            "OXY_CREW_SUPPLY_LO_PR",
            35,
            "OXYGEN CREW SUPPLY LO PR",
            Level::Caution,
            all(vec![var("DEEP_OXY_CREW_SUPPLY_AVAILABLE").off(), var("DEEP_OXY_CREW_BOTTLE_GAUGE_PSI").gt(100.0)]),
        )
        .confirm(5.0)
        .inhibit(&[Phase::FirstEngineTakeoffPower, Phase::Above80Kt, Phase::LiftOff, Phase::Below800Ft, Phase::Touchdown])
        .step(line("CREW OXY SUPPLY", "CHECK ON"))
        .step(line("MAX FL", "100/MEA").colour("white"))
        .status_line("CREW OXY SUPPLY - LO PR")
        .inop_sys("CKPT OXY")
        .raised_by(&[ids::CREW_SUPPLY_VALVE_SEIZED, ids::CREW_REDUCER_SETPOINT_LOW]),
    );

    r.alert(
        EcamAlert::new("OXY_PAX_SYS_ON", 35, "OXYGEN PAX SYS ON", Level::Caution, var("DEEP_OXY_PAX_MASKS_DEPLOYED").on())
            .confirm(1.0)
            .step(line("CREW OXY MASKS", "USE").only_if(var("DEEP_OXY_PAX_CABIN_ALTITUDE_FT").gt(10_000.0)))
            .step(line("DESCENT", "INITIATE").only_if(var("DEEP_OXY_PAX_CABIN_ALTITUDE_FT").gt(14_000.0)))
            .step(line("PAX OXY DURATION", "15 MIN").colour("white"))
            .status_line("PAX OXY - ON")
            .raised_by(&[ids::PAX_INADVERTENT_IGNITION]),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registers_without_validation_errors() {
        let mut r = Registry::default();
        register(&mut r);
        let errors = r.validate_area();
        assert!(errors.is_empty(), "{errors:?}");
    }

    #[test]
    fn every_id_is_registered_exactly_once_and_carries_its_chapter() {
        let mut r = Registry::default();
        register(&mut r);
        let declared: std::collections::BTreeSet<u64> = ids::all().into_iter().collect();
        assert_eq!(declared.len(), ids::all().len(), "ids::all has a duplicate");
        let registered: std::collections::BTreeSet<u64> = r.failures.iter().map(|f| f.id).collect();
        assert_eq!(declared, registered, "ids::all and the registered failures disagree");
        for f in &r.failures {
            assert_eq!(f.ata, 35);
            assert_eq!(f.id / 1_000_000, Area::Oxygen as u64);
            assert!(!f.effect.is_empty() && !f.model_field.is_empty());
        }
    }

    #[test]
    fn every_failure_names_a_component_this_area_registers_and_every_component_is_reachable() {
        let mut r = Registry::default();
        register(&mut r);
        for f in &r.failures {
            assert!(r.components.iter().any(|c| c.id == f.component), "{} names unknown component {}", f.id, f.component);
        }
        for c in &r.components {
            assert!(!c.failures.is_empty(), "{} has no failures at all", c.id);
            assert!(!c.params.is_empty(), "{} has no health parameters", c.id);
            for id in &c.failures {
                assert!(r.failures.iter().any(|f| f.id == *id), "{} lists unknown failure {id}", c.id);
            }
        }
    }

    #[test]
    fn the_component_and_failure_counts_are_what_this_area_claims() {
        let mut r = Registry::default();
        register(&mut r);
        assert_eq!(r.failures.len(), 21);
        assert_eq!(r.components.len(), 14);
        assert_eq!(r.alerts.len(), 3);
    }

    #[test]
    fn every_alert_is_raised_by_a_failure_this_area_owns() {
        let mut r = Registry::default();
        register(&mut r);
        let owned: std::collections::BTreeSet<u64> = ids::all().into_iter().collect();
        for a in &r.alerts {
            assert_eq!(a.ata, 35);
            assert!(!a.failures.is_empty(), "{} is raised by nothing", a.key);
            for id in &a.failures {
                assert!(owned.contains(id), "{} names {id}, which this area does not own", a.key);
            }
        }
    }
}
