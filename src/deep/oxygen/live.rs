//! The live oxygen system: the three supplies, running, published, and
//! driven by the failures `registry.rs` catalogues.
//!
//! ## What it reads
//!
//! | input | source | why |
//! |---|---|---|
//! | cabin pressure | `Truth::cabin_pressure_pa` | the diluter schedule and the mask deployment trigger are both written against cabin altitude, which this is the physical form of |
//! | cabin temperature | `Truth::cabin_temp_k` | what a chemical generator's case radiates into |
//! | crew cylinder bay temperature | `THERMAL_ZONE_MAINAVIONICS_TEMPERATURE_C` (`deep::thermal_zones`), falling back to the cabin | the bottles live in the forward avionics/cargo area, and their temperature is the second thing the gauge reads |
//! | crew supply valve power | `ELEC_DC_1_BUS_IS_POWERED` (`deep::electrical`), falling back to `Truth::dc_bus_volts[0]` | the valve is a DC 1 motor -- `deep::breakers`' own catalogue already carries CREW OXYGEN SHUTOFF VALVE on DC 1 |
//! | passenger deployment power | `ELEC_DC_ESS_BUS_IS_POWERED`, falling back to the DC buses | likewise PAX OXYGEN GENERATOR CONTROL, on DC ESS |
//!
//! ## What it cannot read yet, and what that costs
//!
//! Three real cockpit and cabin states have no `Truth` field and no
//! published source anywhere in this directory:
//!
//! * **whether a crew mask is donned**, and what its regulator is
//!   selected to (N / 100% / EMER);
//! * **the flight deck's MASK MAN ON command** for the passenger system;
//! * **how many first-aid oxygen outlets the cabin crew have in use**.
//!
//! They are held at their real resting values -- masks stowed, nothing
//! commanded, no outlet in use -- rather than being invented, which is the
//! same choice `fire_ice`'s live system documents for solar flux. The cost
//! is honest and worth stating: with nobody wearing a mask the crew
//! cylinder is only drawn down by its modelled leaks, so the *consumption*
//! half of this area is exercised by its own unit tests and by any failure
//! that opens a hole, but not yet by normal operation. `PROGRESS.md`
//! carries the `Truth` requests. Nothing else here is gated on them: the
//! passenger system's automatic deployment, every cylinder's pressure and
//! temperature, the generators' burn and their heat all run from real
//! inputs today.
//!
//! ## Authority
//!
//! Level 1 of `docs/deep/authority.md`, entirely: `a380_systems` has no
//! oxygen system of any kind to disagree with (this crate's own
//! `src/oxygen.rs` records the same absence), so this area publishes and
//! derives nothing.
//!
//! ## `OXYGEN_BOTTLE_PRESSURE_PA:n`
//!
//! `deep::sensors` has had four bottle-pressure transducer failures
//! registered against two instances since it was written, and its
//! `live_discrete::BLOCKED` table names `OXYGEN_BOTTLE_PRESSURE_PA:n` as
//! the variable nobody published. This area publishes it, **absolute**
//! (SI, like every other pressure in this directory) for two bottles:
//!
//! * `:1` the crew cylinder group;
//! * `:2` the first-aid cylinder -- *not* a passenger-supply bottle.
//!   `deep::sensors` guessed at a gaseous passenger supply and labelled
//!   the guess GENERIC; see [`super::therapeutic`] for why the real answer
//!   is this cylinder instead.
//!
//! The gauge readings each transducer's own indication would show are
//! published beside them as `DEEP_OXY_*_GAUGE_PSI`.

use crate::deep::live::{Area, Faults, Truth};

use super::crew::{CrewOxygenFaults, CrewOxygenInputs, CrewOxygenOutputs, CrewOxygenSystem, CREW_MASK_COUNT};
use super::cylinder::CylinderFaults;
use super::gas;
use super::pax::{PassengerOxygenFaults, PassengerOxygenInputs, PassengerOxygenOutputs, PassengerOxygenSystem};
use super::regulator::{MaskMode, RegulatorFaults};
use super::registry::ids;
use super::therapeutic::{TherapeuticFaults, TherapeuticInputs, TherapeuticOutputs, TherapeuticOxygenSystem};

/// Bus voltage at which a DC bus counts as powered, V. The same threshold
/// `deep::thermal_zones` uses for its own fan supply, and only a fallback:
/// when `deep::electrical` has published a bus state, that is used
/// instead.
const MIN_BUS_VOLTS: f64 = 20.0;

/// The crew cylinders live in the forward avionics/cargo area, which
/// `deep::thermal_zones` models as `MainAvionics`.
const BOTTLE_BAY_TEMP_VAR: &str = "THERMAL_ZONE_MAINAVIONICS_TEMPERATURE_C";

pub struct OxygenLive {
    crew: CrewOxygenSystem,
    pax: PassengerOxygenSystem,
    therapeutic: TherapeuticOxygenSystem,
    crew_out: CrewOxygenOutputs,
    pax_out: PassengerOxygenOutputs,
    therapeutic_out: TherapeuticOutputs,
    /// Per-station variable names, built once (they are formatted, and
    /// `publish` runs every frame).
    mask_fraction_var: [String; CREW_MASK_COUNT],
    mask_flow_var: [String; CREW_MASK_COUNT],
    bank_var: [BankVars; 2],
}

struct BankVars {
    heat_w: String,
    presented: String,
    lit: String,
    case_temp_c: String,
    unit_flow: String,
    unit_count: String,
}

impl BankVars {
    fn new(tag: &str) -> Self {
        Self {
            heat_w: format!("DEEP_OXY_PAX_{tag}_HEAT_W"),
            presented: format!("DEEP_OXY_PAX_{tag}_PRESENTED_FRACTION"),
            lit: format!("DEEP_OXY_PAX_{tag}_LIT_FRACTION"),
            case_temp_c: format!("DEEP_OXY_PAX_{tag}_GENERATOR_CASE_TEMP_C"),
            unit_flow: format!("DEEP_OXY_PAX_{tag}_UNIT_FLOW_L_MIN"),
            unit_count: format!("DEEP_OXY_PAX_{tag}_GENERATOR_COUNT"),
        }
    }
}

/// This area's live system, constructed cold: full bottles, unfired
/// generators, the crew supply valve open.
pub fn live_system() -> Box<dyn Area> {
    Box::new(OxygenLive::new())
}

impl Default for OxygenLive {
    fn default() -> Self {
        Self::new()
    }
}

fn b(x: bool) -> f64 {
    if x {
        1.0
    } else {
        0.0
    }
}

impl OxygenLive {
    pub fn new() -> Self {
        Self {
            crew: CrewOxygenSystem::new(),
            pax: PassengerOxygenSystem::new(),
            therapeutic: TherapeuticOxygenSystem::default(),
            crew_out: CrewOxygenOutputs::default(),
            pax_out: PassengerOxygenOutputs::default(),
            therapeutic_out: TherapeuticOutputs::default(),
            mask_fraction_var: std::array::from_fn(|i| format!("DEEP_OXY_CREW_DELIVERED_O2_FRACTION:{}", i + 1)),
            mask_flow_var: std::array::from_fn(|i| format!("DEEP_OXY_CREW_MASK_FLOW_KG_S:{}", i + 1)),
            bank_var: [BankVars::new("MAIN_DECK"), BankVars::new("UPPER_DECK")],
        }
    }

    /// Whether a bus is up: `deep::electrical`'s own published verdict if
    /// it has one, its raw voltage from `Truth` otherwise.
    fn bus_powered(truth: &Truth, published_name: &str, fallback_volts: f64) -> bool {
        match truth.published.get(published_name) {
            Some(v) => v != 0.0,
            None => fallback_volts >= MIN_BUS_VOLTS,
        }
    }

    fn crew_faults(faults: &Faults) -> CrewOxygenFaults {
        let low = faults.get(ids::CREW_REDUCER_SETPOINT_LOW);
        let high = faults.get(ids::CREW_REDUCER_SETPOINT_HIGH);
        CrewOxygenFaults {
            cylinder: CylinderFaults { leak: faults.get(ids::CREW_CYLINDER_LEAK), disc_weakened: faults.get(ids::CREW_CYLINDER_DISC_RUPTURE) },
            regulator: RegulatorFaults { setpoint_shift: high - low, seat_leak: faults.get(ids::CREW_REDUCER_SEAT_LEAK) },
            valve_jam: faults.get(ids::CREW_SUPPLY_VALVE_SEIZED),
            distribution_leak: faults.get(ids::CREW_DISTRIBUTION_LEAK),
            dilution_stuck_ambient: std::array::from_fn(|i| faults.get(ids::CREW_MASK_DILUTER_STUCK[i])),
        }
    }

    fn pax_faults(faults: &Faults) -> PassengerOxygenFaults {
        PassengerOxygenFaults {
            latch_failed: faults.get(ids::PAX_LATCH_FAILED),
            dud_initiators: faults.get(ids::PAX_DUD_INITIATORS),
            candle_quench: faults.get(ids::PAX_CANDLE_QUENCH),
            inadvertent_ignition: faults.get(ids::PAX_INADVERTENT_IGNITION),
            auto_deploy_controller: faults.get(ids::PAX_AUTO_DEPLOY_CONTROLLER),
        }
    }

    fn therapeutic_faults(faults: &Faults) -> TherapeuticFaults {
        TherapeuticFaults {
            cylinder: CylinderFaults {
                leak: faults.get(ids::THERAPEUTIC_CYLINDER_LEAK),
                disc_weakened: faults.get(ids::THERAPEUTIC_CYLINDER_DISC_RUPTURE),
            },
            regulator: RegulatorFaults {
                setpoint_shift: -faults.get(ids::THERAPEUTIC_REDUCER_SETPOINT_LOW),
                seat_leak: faults.get(ids::THERAPEUTIC_REDUCER_SEAT_LEAK),
            },
            outlets_stuck_open: faults.get(ids::THERAPEUTIC_OUTLET_STUCK_OPEN),
        }
    }

    pub fn crew_outputs(&self) -> &CrewOxygenOutputs {
        &self.crew_out
    }

    pub fn pax_outputs(&self) -> &PassengerOxygenOutputs {
        &self.pax_out
    }

    pub fn therapeutic_outputs(&self) -> &TherapeuticOutputs {
        &self.therapeutic_out
    }

    /// Ground servicing: recharge both cylinders and fit new generators.
    /// The turnaround, in one call.
    pub fn service(&mut self) {
        self.crew.service();
        self.pax.service();
        self.therapeutic.service();
    }
}

impl Area for OxygenLive {
    fn name(&self) -> &'static str {
        "oxygen"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        let dt = truth.dt_s.max(0.0);
        let cabin_pressure_pa = truth.cabin_pressure_pa.max(1.0);
        let cabin_temp_k = if truth.cabin_temp_k > 0.0 { truth.cabin_temp_k } else { 288.15 };
        // The bay the bottles are clamped in, if the thermal area is
        // modelling it; the cabin if nothing is. Never a constant.
        let bay_temp_k = match truth.published.get(BOTTLE_BAY_TEMP_VAR) {
            Some(c) => c + 273.15,
            None => cabin_temp_k,
        };

        let dc1 = Self::bus_powered(truth, "ELEC_DC_1_BUS_IS_POWERED", truth.dc_bus_volts[0]);
        let dc_ess = Self::bus_powered(truth, "ELEC_DC_ESS_BUS_IS_POWERED", truth.dc_bus_volts.iter().copied().fold(0.0, f64::max));

        self.crew_out = self.crew.step(
            CrewOxygenInputs {
                cabin_pressure_pa,
                bay_temp_k,
                // No `Truth` field carries mask donning or the mask
                // regulator selectors; see this module's header.
                masks_donned: [false; CREW_MASK_COUNT],
                mask_mode: [MaskMode::Normal; CREW_MASK_COUNT],
                supply_valve_commanded_open: true,
                valve_actuator_powered: dc1,
            },
            Self::crew_faults(faults),
            dt,
        );

        self.pax_out = self.pax.step(
            PassengerOxygenInputs {
                cabin_pressure_pa,
                cabin_temp_k,
                // Likewise MASK MAN ON.
                manual_deploy_commanded: false,
                control_circuit_powered: dc_ess,
                // Passengers do pull the masks down when they appear;
                // that is behaviour, not a cockpit control, and a cabin
                // in which nobody pulled would be the invented case.
                masks_pulled: true,
            },
            Self::pax_faults(faults),
            dt,
        );

        self.therapeutic_out = self.therapeutic.step(
            TherapeuticInputs {
                cabin_pressure_pa,
                bay_temp_k: cabin_temp_k,
                // Likewise how many outlets the cabin crew are using.
                outlets_in_use: 0.0,
                high_flow: true,
            },
            Self::therapeutic_faults(faults),
            dt,
        );
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        let c = &self.crew_out;
        // The name `deep::sensors` has been waiting on, absolute, indexed
        // as its own registry numbers the transducers.
        out("OXYGEN_BOTTLE_PRESSURE_PA:1", c.cylinder.absolute_pressure_pa);
        out("DEEP_OXY_CREW_BOTTLE_GAUGE_PA", c.cylinder.gauge_pressure_pa);
        out("DEEP_OXY_CREW_BOTTLE_GAUGE_PSI", c.cylinder.gauge_pressure_pa / gas::PSI_TO_PA);
        out("DEEP_OXY_CREW_BOTTLE_CORRECTED_PSI", c.cylinder.corrected_gauge_pressure_pa / gas::PSI_TO_PA);
        out("DEEP_OXY_CREW_BOTTLE_TEMP_C", c.cylinder.gas_temp_k - 273.15);
        out("DEEP_OXY_CREW_BOTTLE_WALL_TEMP_C", c.cylinder.wall_temp_k - 273.15);
        out("DEEP_OXY_CREW_QUANTITY_FRACTION", c.cylinder.quantity_fraction);
        out("DEEP_OXY_CREW_MASS_KG", c.cylinder.mass_kg);
        out("DEEP_OXY_CREW_LOW_PRESSURE", b(c.low_pressure));
        out("DEEP_OXY_CREW_DISC_RUPTURED", b(c.cylinder.disc_ruptured));
        out("DEEP_OXY_CREW_CYLINDER_LEAK_KG_S", c.cylinder.leak_kg_s);
        out("DEEP_OXY_CREW_DISTRIBUTION_PSI", c.distribution_gauge_pa / gas::PSI_TO_PA);
        out("DEEP_OXY_CREW_DISTRIBUTION_LEAK_KG_S", c.distribution_leak_kg_s);
        out("DEEP_OXY_CREW_SUPPLY_AVAILABLE", b(c.supply_available));
        out("DEEP_OXY_CREW_VALVE_POSITION", c.valve_position);
        out("DEEP_OXY_CREW_RELIEF_LIFTED", b(c.regulator.relief_lifted));
        out("DEEP_OXY_CREW_REDUCER_DROPPED_OUT", b(c.regulator.dropped_out));
        out("DEEP_OXY_CREW_TOTAL_FLOW_KG_S", c.total_mask_flow_kg_s);
        out("DEEP_OXY_CREW_ENDURANCE_S", c.endurance_s.min(1e9));
        for i in 0..CREW_MASK_COUNT {
            out(&self.mask_fraction_var[i], c.delivered_o2_fraction[i]);
            out(&self.mask_flow_var[i], c.mask_flow_kg_s[i]);
        }

        let p = &self.pax_out;
        out("DEEP_OXY_PAX_CABIN_ALTITUDE_FT", p.cabin_altitude_ft);
        out("DEEP_OXY_PAX_MASKS_DEPLOYED", b(p.masks_deployed));
        out("DEEP_OXY_PAX_GENERATORS_RUNNING", b(p.generators_running));
        out("DEEP_OXY_PAX_SUPPLY_REMAINING_FRACTION", p.supply_remaining_fraction);
        out("DEEP_OXY_PAX_TOTAL_FLOW_KG_S", p.total_o2_kg_s);
        out("DEEP_OXY_PAX_TOTAL_HEAT_W", p.total_heat_w);
        out("DEEP_OXY_PAX_REMAINING_DURATION_S", p.remaining_duration_s);
        for i in 0..2 {
            let v = &self.bank_var[i];
            let bank = &p.banks[i];
            out(&v.heat_w, bank.heat_w);
            out(&v.presented, bank.presented_fraction);
            out(&v.lit, bank.lit_fraction);
            out(&v.case_temp_c, bank.case_temp_k - 273.15);
            out(&v.unit_flow, bank.unit_o2_l_per_min);
            out(&v.unit_count, bank.unit_count);
        }

        let t = &self.therapeutic_out;
        out("OXYGEN_BOTTLE_PRESSURE_PA:2", t.cylinder.absolute_pressure_pa);
        out("DEEP_OXY_THERAPEUTIC_GAUGE_PSI", t.cylinder.gauge_pressure_pa / gas::PSI_TO_PA);
        out("DEEP_OXY_THERAPEUTIC_CORRECTED_PSI", t.cylinder.corrected_gauge_pressure_pa / gas::PSI_TO_PA);
        out("DEEP_OXY_THERAPEUTIC_TEMP_C", t.cylinder.gas_temp_k - 273.15);
        out("DEEP_OXY_THERAPEUTIC_QUANTITY_FRACTION", t.cylinder.quantity_fraction);
        out("DEEP_OXY_THERAPEUTIC_DISC_RUPTURED", b(t.cylinder.disc_ruptured));
        out("DEEP_OXY_THERAPEUTIC_OUTLET_PSI", t.outlet_gauge_pa / gas::PSI_TO_PA);
        out("DEEP_OXY_THERAPEUTIC_OUTLETS_FLOWING", t.outlets_flowing);
        out("DEEP_OXY_THERAPEUTIC_FLOW_L_MIN", t.total_flow_l_per_min);
        out("DEEP_OXY_THERAPEUTIC_SUPPLY_AVAILABLE", b(t.supply_available));
        out("DEEP_OXY_THERAPEUTIC_ENDURANCE_S", t.endurance_s.min(1e9));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::api::Registry;
    use crate::deep::live::PublishedFrame;
    use std::collections::BTreeMap;
    use std::time::Instant;

    fn published(area: &OxygenLive) -> BTreeMap<String, f64> {
        let mut m = BTreeMap::new();
        area.publish(&mut |n, v| {
            m.insert(n.to_string(), v);
        });
        m
    }

    /// A cabin at a given altitude, everything else at rest.
    fn truth_at_cabin_ft(ft: f64) -> Truth {
        let pa = 101_325.0 * (1.0 - ft * 0.3048 / 44_330.77).powf(1.0 / 0.190_263_1);
        let mut t = Truth { dt_s: 1.0, cabin_pressure_pa: pa, cabin_temp_k: 297.15, ..Truth::default() };
        t.dc_bus_volts = [28.0; 2];
        t
    }

    fn run(area: &mut OxygenLive, truth: &Truth, faults: &Faults, seconds: usize) -> BTreeMap<String, f64> {
        for _ in 0..seconds {
            area.tick(truth, faults);
        }
        published(area)
    }

    #[test]
    fn a_cold_aircraft_publishes_full_bottles_and_no_nonsense() {
        let mut area = OxygenLive::new();
        area.tick(&Truth::default(), &Faults::default());
        let p = published(&area);
        for (name, value) in &p {
            assert!(value.is_finite(), "{name} published {value}");
        }
        assert!((p["DEEP_OXY_CREW_BOTTLE_GAUGE_PSI"] - 1850.0).abs() < 5.0, "{}", p["DEEP_OXY_CREW_BOTTLE_GAUGE_PSI"]);
        assert!((p["DEEP_OXY_THERAPEUTIC_GAUGE_PSI"] - 1800.0).abs() < 5.0, "{}", p["DEEP_OXY_THERAPEUTIC_GAUGE_PSI"]);
        assert_eq!(p["DEEP_OXY_CREW_LOW_PRESSURE"], 0.0);
        assert_eq!(p["DEEP_OXY_PAX_MASKS_DEPLOYED"], 0.0);
        assert_eq!(p["DEEP_OXY_PAX_SUPPLY_REMAINING_FRACTION"], 1.0);
        assert_eq!(p["DEEP_OXY_CREW_SUPPLY_AVAILABLE"], 1.0);
        assert!(p["OXYGEN_BOTTLE_PRESSURE_PA:1"] > 1e7);
        assert!(p["OXYGEN_BOTTLE_PRESSURE_PA:2"] > 1e7);
    }

    #[test]
    fn the_variable_the_sensors_area_has_been_waiting_for_is_published_under_that_exact_name() {
        // `deep::sensors::live_discrete::BLOCKED` names
        // OXYGEN_BOTTLE_PRESSURE_PA:n against component prefix
        // "35_oxy.pressure_", and that area registers two instances. Both
        // indices have to exist here or half its transducers still have
        // nothing to sense.
        let p = published(&OxygenLive::new());
        assert!(p.contains_key("OXYGEN_BOTTLE_PRESSURE_PA:1"));
        assert!(p.contains_key("OXYGEN_BOTTLE_PRESSURE_PA:2"));
        let mut sensors = Registry::default();
        crate::deep::sensors::registry::register(&mut sensors);
        let registered = sensors.components.iter().filter(|c| c.id.starts_with("35_oxy.pressure_")).count();
        assert_eq!(registered, 2, "sensors registers {registered} bottle transducers; this area publishes 2 and must publish one per instance");
    }

    #[test]
    fn the_passenger_masks_drop_when_the_cabin_climbs_and_the_generators_light() {
        let mut area = OxygenLive::new();
        let low = run(&mut area, &truth_at_cabin_ft(8000.0), &Faults::default(), 60);
        assert_eq!(low["DEEP_OXY_PAX_MASKS_DEPLOYED"], 0.0);
        assert_eq!(low["DEEP_OXY_PAX_TOTAL_HEAT_W"], 0.0);
        assert!((low["DEEP_OXY_PAX_CABIN_ALTITUDE_FT"] - 8000.0).abs() < 50.0, "{}", low["DEEP_OXY_PAX_CABIN_ALTITUDE_FT"]);

        let high = run(&mut area, &truth_at_cabin_ft(20_000.0), &Faults::default(), 300);
        assert_eq!(high["DEEP_OXY_PAX_MASKS_DEPLOYED"], 1.0);
        assert_eq!(high["DEEP_OXY_PAX_GENERATORS_RUNNING"], 1.0);
        assert!(high["DEEP_OXY_PAX_TOTAL_HEAT_W"] > 10_000.0, "{}", high["DEEP_OXY_PAX_TOTAL_HEAT_W"]);
        assert!(high["DEEP_OXY_PAX_MAIN_DECK_GENERATOR_CASE_TEMP_C"] > 150.0, "{}", high["DEEP_OXY_PAX_MAIN_DECK_GENERATOR_CASE_TEMP_C"]);
        assert!(high["DEEP_OXY_PAX_SUPPLY_REMAINING_FRACTION"] < 1.0);
    }

    #[test]
    fn the_crew_bottle_reading_follows_the_bay_the_thermal_area_models() {
        // The one cross-area coupling this file has, and the thing that
        // makes the gauge interesting: the same bottle, the same oxygen,
        // two bay temperatures, two readings.
        let mut hot = OxygenLive::new();
        let mut cold = OxygenLive::new();
        let mut hot_truth = truth_at_cabin_ft(0.0);
        let mut cold_truth = truth_at_cabin_ft(0.0);
        hot_truth.published = PublishedFrame::from(BTreeMap::from([(BOTTLE_BAY_TEMP_VAR.to_string(), 45.0)]));
        cold_truth.published = PublishedFrame::from(BTreeMap::from([(BOTTLE_BAY_TEMP_VAR.to_string(), -30.0)]));
        let h = run(&mut hot, &hot_truth, &Faults::default(), 6 * 3600);
        let c = run(&mut cold, &cold_truth, &Faults::default(), 6 * 3600);
        assert!((h["DEEP_OXY_CREW_MASS_KG"] - c["DEEP_OXY_CREW_MASS_KG"]).abs() < 1e-12, "neither bottle lost a gram");
        assert!(h["DEEP_OXY_CREW_BOTTLE_GAUGE_PSI"] > c["DEEP_OXY_CREW_BOTTLE_GAUGE_PSI"] + 200.0, "hot {} cold {}", h["DEEP_OXY_CREW_BOTTLE_GAUGE_PSI"], c["DEEP_OXY_CREW_BOTTLE_GAUGE_PSI"]);
        // And the corrected reading -- the one the caution uses -- does
        // not move, so a cold soak does not look like an empty bottle.
        assert!((h["DEEP_OXY_CREW_BOTTLE_CORRECTED_PSI"] - c["DEEP_OXY_CREW_BOTTLE_CORRECTED_PSI"]).abs() < 1.0);
        assert_eq!(c["DEEP_OXY_CREW_LOW_PRESSURE"], 0.0);
    }

    #[test]
    fn an_unpowered_deployment_controller_leaves_the_masks_up() {
        let mut area = OxygenLive::new();
        let mut truth = truth_at_cabin_ft(20_000.0);
        truth.dc_bus_volts = [0.0; 2];
        truth.published = PublishedFrame::from(BTreeMap::from([("ELEC_DC_ESS_BUS_IS_POWERED".to_string(), 0.0)]));
        let p = run(&mut area, &truth, &Faults::default(), 60);
        assert_eq!(p["DEEP_OXY_PAX_MASKS_DEPLOYED"], 0.0, "the automatic trigger is electrical");
    }

    // -----------------------------------------------------------------
    // One armed failure per registered fault, each moving the variable
    // its catalogue entry says it moves.
    // -----------------------------------------------------------------

    fn armed(id: u64, magnitude: f64, ft: f64, seconds: usize) -> BTreeMap<String, f64> {
        let mut area = OxygenLive::new();
        run(&mut area, &truth_at_cabin_ft(ft), &Faults::from_pairs([(id, magnitude)]), seconds)
    }

    fn healthy(ft: f64, seconds: usize) -> BTreeMap<String, f64> {
        let mut area = OxygenLive::new();
        run(&mut area, &truth_at_cabin_ft(ft), &Faults::default(), seconds)
    }

    #[test]
    fn a_crew_cylinder_leak_drops_the_indicated_pressure_and_raises_the_caution() {
        let p = armed(ids::CREW_CYLINDER_LEAK, 1.0, 0.0, 900);
        assert!(p["DEEP_OXY_CREW_CYLINDER_LEAK_KG_S"] > 0.0);
        assert!(p["DEEP_OXY_CREW_BOTTLE_GAUGE_PSI"] < 500.0, "{}", p["DEEP_OXY_CREW_BOTTLE_GAUGE_PSI"]);
        assert_eq!(p["DEEP_OXY_CREW_LOW_PRESSURE"], 1.0);
    }

    #[test]
    fn a_degraded_burst_disc_dumps_the_crew_bottle_overboard() {
        let p = armed(ids::CREW_CYLINDER_DISC_RUPTURE, 1.0, 0.0, 300);
        assert_eq!(p["DEEP_OXY_CREW_DISC_RUPTURED"], 1.0);
        assert!(p["DEEP_OXY_CREW_QUANTITY_FRACTION"] < 0.05, "{}", p["DEEP_OXY_CREW_QUANTITY_FRACTION"]);
        assert_eq!(p["DEEP_OXY_CREW_LOW_PRESSURE"], 1.0);
        assert_eq!(healthy(0.0, 300)["DEEP_OXY_CREW_DISC_RUPTURED"], 0.0);
    }

    #[test]
    fn a_seized_supply_valve_kills_the_distribution_and_leaves_the_bottle_full() {
        let p = armed(ids::CREW_SUPPLY_VALVE_SEIZED, 1.0, 0.0, 60);
        assert_eq!(p["DEEP_OXY_CREW_VALVE_POSITION"], 0.0);
        assert_eq!(p["DEEP_OXY_CREW_DISTRIBUTION_PSI"], 0.0);
        assert_eq!(p["DEEP_OXY_CREW_SUPPLY_AVAILABLE"], 0.0);
        assert!((p["DEEP_OXY_CREW_QUANTITY_FRACTION"] - 1.0).abs() < 1e-9, "the gauge cannot see this one");
    }

    #[test]
    fn a_reducer_set_low_starves_the_masks_and_set_high_lifts_the_relief() {
        let low = armed(ids::CREW_REDUCER_SETPOINT_LOW, 1.0, 0.0, 10);
        assert_eq!(low["DEEP_OXY_CREW_DISTRIBUTION_PSI"], 0.0);
        assert_eq!(low["DEEP_OXY_CREW_SUPPLY_AVAILABLE"], 0.0);
        let high = armed(ids::CREW_REDUCER_SETPOINT_HIGH, 1.0, 0.0, 10);
        assert_eq!(high["DEEP_OXY_CREW_RELIEF_LIFTED"], 1.0);
        assert!(high["DEEP_OXY_CREW_DISTRIBUTION_PSI"] > healthy(0.0, 10)["DEEP_OXY_CREW_DISTRIBUTION_PSI"]);
    }

    #[test]
    fn a_reducer_seat_leak_and_a_distribution_leak_both_drain_the_bottle_with_no_mask_on() {
        let seat = armed(ids::CREW_REDUCER_SEAT_LEAK, 1.0, 0.0, 3600);
        assert!(seat["DEEP_OXY_CREW_DISTRIBUTION_LEAK_KG_S"] > 0.0);
        assert!(seat["DEEP_OXY_CREW_QUANTITY_FRACTION"] < 0.99);
        let hose = armed(ids::CREW_DISTRIBUTION_LEAK, 1.0, 0.0, 3600);
        assert!(hose["DEEP_OXY_CREW_DISTRIBUTION_LEAK_KG_S"] > 0.0);
        assert!(hose["DEEP_OXY_CREW_QUANTITY_FRACTION"] < 0.95, "{}", hose["DEEP_OXY_CREW_QUANTITY_FRACTION"]);
        assert_eq!(healthy(0.0, 3600)["DEEP_OXY_CREW_DISTRIBUTION_LEAK_KG_S"], 0.0);
    }

    #[test]
    fn each_masks_diluter_fault_moves_only_that_stations_delivered_fraction() {
        for station in 0..CREW_MASK_COUNT {
            let p = armed(ids::CREW_MASK_DILUTER_STUCK[station], 1.0, 35_000.0, 10);
            for other in 0..CREW_MASK_COUNT {
                let name = format!("DEEP_OXY_CREW_DELIVERED_O2_FRACTION:{}", other + 1);
                if other == station {
                    assert!((p[&name] - gas::AIR_O2_MOLE_FRACTION).abs() < 1e-9, "station {} read {}", other + 1, p[&name]);
                } else {
                    assert!(p[&name] > 0.9, "station {} should still be scheduling oxygen, read {}", other + 1, p[&name]);
                }
            }
        }
    }

    #[test]
    fn dud_initiators_present_the_masks_and_deliver_nothing() {
        let p = armed(ids::PAX_DUD_INITIATORS, 1.0, 20_000.0, 60);
        assert_eq!(p["DEEP_OXY_PAX_MASKS_DEPLOYED"], 1.0);
        assert_eq!(p["DEEP_OXY_PAX_GENERATORS_RUNNING"], 0.0);
        assert_eq!(p["DEEP_OXY_PAX_TOTAL_FLOW_KG_S"], 0.0);
        assert_eq!(p["DEEP_OXY_PAX_TOTAL_HEAT_W"], 0.0);
    }

    #[test]
    fn quenching_candles_stop_the_cabin_supply_early() {
        let p = armed(ids::PAX_CANDLE_QUENCH, 0.6, 20_000.0, 400);
        assert_eq!(p["DEEP_OXY_PAX_GENERATORS_RUNNING"], 0.0, "0.4 of the candle is gone by 360 s");
        assert_eq!(p["DEEP_OXY_PAX_TOTAL_FLOW_KG_S"], 0.0);
        assert_eq!(healthy(20_000.0, 400)["DEEP_OXY_PAX_GENERATORS_RUNNING"], 1.0);
    }

    #[test]
    fn an_inadvertent_ignition_heats_the_cabin_with_the_masks_still_stowed() {
        let p = armed(ids::PAX_INADVERTENT_IGNITION, 0.05, 0.0, 600);
        assert_eq!(p["DEEP_OXY_PAX_MASKS_DEPLOYED"], 0.0);
        assert_eq!(p["DEEP_OXY_PAX_GENERATORS_RUNNING"], 1.0);
        assert!(p["DEEP_OXY_PAX_TOTAL_HEAT_W"] > 500.0, "{}", p["DEEP_OXY_PAX_TOTAL_HEAT_W"]);
        assert!(p["DEEP_OXY_PAX_MAIN_DECK_GENERATOR_CASE_TEMP_C"] > 140.0, "{}", p["DEEP_OXY_PAX_MAIN_DECK_GENERATOR_CASE_TEMP_C"]);
        assert_eq!(healthy(0.0, 600)["DEEP_OXY_PAX_TOTAL_HEAT_W"], 0.0);
    }

    #[test]
    fn seized_latches_leave_part_of_the_cabin_with_no_mask() {
        let p = armed(ids::PAX_LATCH_FAILED, 0.5, 20_000.0, 60);
        assert!((p["DEEP_OXY_PAX_MAIN_DECK_PRESENTED_FRACTION"] - 0.5).abs() < 1e-9);
        assert!(p["DEEP_OXY_PAX_TOTAL_HEAT_W"] < 0.75 * healthy(20_000.0, 60)["DEEP_OXY_PAX_TOTAL_HEAT_W"]);
    }

    #[test]
    fn a_failed_deployment_controller_never_presents_the_masks() {
        let p = armed(ids::PAX_AUTO_DEPLOY_CONTROLLER, 1.0, 25_000.0, 120);
        assert_eq!(p["DEEP_OXY_PAX_MASKS_DEPLOYED"], 0.0);
        assert_eq!(p["DEEP_OXY_PAX_TOTAL_FLOW_KG_S"], 0.0);
        assert_eq!(healthy(25_000.0, 120)["DEEP_OXY_PAX_MASKS_DEPLOYED"], 1.0);
    }

    #[test]
    fn every_therapeutic_failure_moves_the_first_aid_supply() {
        let base = healthy(0.0, 3600);
        let leak = armed(ids::THERAPEUTIC_CYLINDER_LEAK, 1.0, 0.0, 1200);
        assert!(leak["DEEP_OXY_THERAPEUTIC_QUANTITY_FRACTION"] < 0.05, "{}", leak["DEEP_OXY_THERAPEUTIC_QUANTITY_FRACTION"]);
        assert_eq!(leak["DEEP_OXY_THERAPEUTIC_SUPPLY_AVAILABLE"], 0.0);

        let disc = armed(ids::THERAPEUTIC_CYLINDER_DISC_RUPTURE, 1.0, 0.0, 300);
        assert_eq!(disc["DEEP_OXY_THERAPEUTIC_DISC_RUPTURED"], 1.0);
        assert!(disc["DEEP_OXY_THERAPEUTIC_QUANTITY_FRACTION"] < 0.05);

        let low = armed(ids::THERAPEUTIC_REDUCER_SETPOINT_LOW, 1.0, 0.0, 10);
        assert_eq!(low["DEEP_OXY_THERAPEUTIC_OUTLET_PSI"], 0.0);
        assert_eq!(low["DEEP_OXY_THERAPEUTIC_SUPPLY_AVAILABLE"], 0.0);

        let seat = armed(ids::THERAPEUTIC_REDUCER_SEAT_LEAK, 1.0, 0.0, 3600);
        assert!(seat["DEEP_OXY_THERAPEUTIC_QUANTITY_FRACTION"] < base["DEEP_OXY_THERAPEUTIC_QUANTITY_FRACTION"] - 0.01);

        let stuck = armed(ids::THERAPEUTIC_OUTLET_STUCK_OPEN, 1.0, 0.0, 600);
        assert!(stuck["DEEP_OXY_THERAPEUTIC_OUTLETS_FLOWING"] > 0.0);
        assert!(stuck["DEEP_OXY_THERAPEUTIC_FLOW_L_MIN"] > 40.0, "{}", stuck["DEEP_OXY_THERAPEUTIC_FLOW_L_MIN"]);
        assert!(stuck["DEEP_OXY_THERAPEUTIC_QUANTITY_FRACTION"] < base["DEEP_OXY_THERAPEUTIC_QUANTITY_FRACTION"] - 0.05);
    }

    #[test]
    fn every_registered_failure_is_one_this_live_system_actually_reads() {
        // The other half of the guarantee: the catalogue and the live
        // system share `registry::ids`, and this walks the registry to
        // check nothing was registered that no fault struct consumes.
        let mut r = Registry::default();
        super::super::registry::register(&mut r);
        let declared: std::collections::BTreeSet<u64> = ids::all().into_iter().collect();
        for f in &r.failures {
            assert!(declared.contains(&f.id), "failure {} ({}) is registered but is in no ids:: constant", f.id, f.name);
        }
        // And each of them reaches a fault field: arming it alone must
        // produce a fault struct that differs from the healthy one.
        for id in ids::all() {
            let armed = Faults::from_pairs([(id, 1.0)]);
            let crew_differs = OxygenLive::crew_faults(&armed) != OxygenLive::crew_faults(&Faults::default());
            let pax_differs = OxygenLive::pax_faults(&armed) != OxygenLive::pax_faults(&Faults::default());
            let ther_differs = OxygenLive::therapeutic_faults(&armed) != OxygenLive::therapeutic_faults(&Faults::default());
            assert!(crew_differs || pax_differs || ther_differs, "failure {id} reaches no modelled field");
        }
    }

    #[test]
    fn a_healthy_aircraft_derives_no_flybywire_failures() {
        // Level 1: `a380_systems` has no oxygen system to argue with.
        let mut area = OxygenLive::new();
        area.tick(&Truth::default(), &Faults::default());
        let mut n = 0;
        area.derived_failures(&mut |_| n += 1);
        assert_eq!(n, 0);
    }

    #[test]
    fn the_frame_cost_is_reported_and_small() {
        let mut area = OxygenLive::new();
        let truth = truth_at_cabin_ft(8000.0);
        let faults = Faults::default();
        for _ in 0..200 {
            area.tick(&truth, &faults);
        }
        const N: u32 = 20_000;
        let t0 = Instant::now();
        for _ in 0..N {
            area.tick(&truth, &faults);
            area.publish(&mut |_, _| {});
        }
        let per_frame_us = t0.elapsed().as_secs_f64() * 1e6 / N as f64;
        println!("OXYGEN frame cost: {per_frame_us:.3} us per tick+publish");
        // A whole area at 60 Hz has 16 600 us to share; this one models
        // three supplies and must not be a measurable part of that.
        assert!(per_frame_us < 60.0, "oxygen is costing {per_frame_us} us a frame");
    }

    #[test]
    fn a_thousand_frames_at_a_zero_dt_change_nothing_and_produce_no_nan() {
        let mut area = OxygenLive::new();
        let truth = Truth { dt_s: 0.0, ..truth_at_cabin_ft(20_000.0) };
        area.tick(&truth, &Faults::default());
        let first = published(&area);
        for _ in 0..1000 {
            area.tick(&truth, &Faults::default());
        }
        let last = published(&area);
        for (name, value) in &first {
            assert!(value.is_finite());
            assert_eq!(last[name], *value, "{name} drifted with dt = 0");
        }
    }
}
