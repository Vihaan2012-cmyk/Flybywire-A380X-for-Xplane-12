//! The live thermal-zone system: one owned [`topology_a380::A380Thermal`]
//! stepped every frame from [`Truth`], with every failure this area's
//! [`super::registry`] registers driving the exact model field that
//! registry entry names, and every variable its ECAM triggers read
//! published back out.
//!
//! Until this file existed the airframe thermal network was a type with
//! tests and no instance: `topology_a380::build()` was never called
//! anywhere in the running plugin, so no zone had a temperature, no
//! `THERMAL_ZONE_*` variable existed, and every ECAM alert
//! `registry.rs` hangs off those variables was unreachable. This is the
//! instance.
//!
//! ## What drives the network from `Truth`
//! - **Outside air** ([`OutsideAir`]): static air temperature, Mach and
//!   true airspeed straight from `truth.environment` (X-Plane's own real
//!   weather, see `integration::weather_truth`). Ram/recovery heating and
//!   the forced-convection coefficient both come out of those three, so
//!   altitude and speed reach every zone's exterior through the one input
//!   the network already has for them.
//! - **AC power**: the avionics- and cargo-bay extract fans are electric.
//!   An induction fan motor either runs at essentially its synchronous
//!   speed or it does not run at all, so the fans are modelled as
//!   delivering full nameplate flow while at least one main AC bus is at
//!   or above [`MIN_FAN_BUS_VOLTS`], and none below it -- which makes a
//!   total AC loss heat the avionics bay for the same physical reason a
//!   fan failure does, with no separate code path. The per-fan bus
//!   assignment is not public for the A380, so "any main AC bus" is used
//!   rather than inventing one (see the report/`PROGRESS.md`).
//!
//! Ram-air paths (nacelle, pylon, APU compartment, belly fairing, tail
//! cone) need no power and are driven only by their own failures.
//!
//! ## What is not driven yet, and why
//! - **Solar flux**: `ThermalNetwork::step` takes an incident solar flux
//!   and every zone already carries its own `sun_exposure_fraction`, but
//!   [`Truth`] carries no solar irradiance (nor sun elevation to derive
//!   one from), so this passes 0. Publishing a guessed 800 W/m^2 would be
//!   a fabricated input, not a modelled one.
//! - **Gear bay door position**: `registry`'s ATA 32 door-jam failures
//!   freeze a door's ventilation link away from its commanded position.
//!   The jam itself is implemented (the link's health latches at the value
//!   it held when the fault engaged), but `Truth` carries no commanded
//!   gear/door position, so the commanded value is `topology_a380`'s own
//!   resting state (doors closed) and a jam has nothing to diverge from.
//! - **Zone heat from other areas** (engine heat into a nacelle, a
//!   pneumatic duct leak's enthalpy, brake heat into a gear bay): those
//!   are other areas' models. `ThermalNetwork::inject_heat_w` is the
//!   interface they will use; nothing is injected on their behalf here.

use super::network::OutsideAir;
use super::topology_a380::{self, A380Thermal};
use crate::deep::api::{failure_id, Area as RegArea};
use crate::deep::live::{Faults, Truth};

/// Minimum main-AC-bus voltage at which a ventilation fan motor is
/// treated as running. Aircraft 115 V 400 Hz AC: RTCA DO-160/MIL-STD-704F
/// put normal steady-state operation at 108-118 V rms and the abnormal
/// low-voltage limit at 100 V, so 100 V is the documented floor below
/// which equipment is not required to operate.
pub const MIN_FAN_BUS_VOLTS: f64 = 100.0;

/// Incident solar flux handed to `ThermalNetwork::step`. Zero because
/// [`Truth`] has no solar irradiance field (module doc) -- not a modelling
/// claim that the sun is never up.
const SOLAR_FLUX_W_M2: f64 = 0.0;

// Reference full-severity magnitudes. Each one is the exact figure the
// matching `FailureDef::model_field`/`magnitude` text in `registry.rs`
// cites, repeated here so the two cannot drift silently (the registry
// builds its documentation strings from its own copies).
const CARGO_FIRE_MAX_HEAT_W: f64 = 200_000.0;
const CARGO_FIRE_MAX_SMOKE_KG_S: f64 = 0.01;
const NACELLE_FIRE_MAX_HEAT_W: f64 = 500_000.0;
const NACELLE_FIRE_MAX_SMOKE_KG_S: f64 = 0.005;
const APU_FIRE_MAX_HEAT_W: f64 = 300_000.0;
const APU_FIRE_MAX_SMOKE_KG_S: f64 = 0.008;
const WING_DUCT_LEAK_MAX_HEAT_W: f64 = 30_000.0;
const NACELLE_DUCT_LEAK_MAX_HEAT_W: f64 = 20_000.0;
const PYLON_BLEED_LEAK_MAX_HEAT_W: f64 = 40_000.0;
const APU_DUCT_LEAK_MAX_HEAT_W: f64 = 25_000.0;

fn f(ata: u16, n: u16) -> u64 {
    failure_id(RegArea::ThermalZones, ata, n)
}

/// The three variable names one zone publishes, built once at
/// construction (the `Area` trait publishes by `&str`, and a zone's name
/// never changes).
struct ZoneVars {
    temperature_c: String,
    structure_temperature_c: String,
    smoke_concentration: String,
}

pub struct ThermalZonesLive {
    a380: A380Thermal,
    zone_vars: Vec<ZoneVars>,
    damage_vars: Vec<String>,
    /// Per gear bay (nose, wing, body): the ventilation-link health the
    /// door was at when its jam failure first engaged, `None` while the
    /// door is free. A jam freezes the door where it is; it does not move
    /// it (`registry.rs`, ATA 32).
    gear_door_jammed_at: [Option<f64>; 3],
}

impl Default for ThermalZonesLive {
    fn default() -> Self {
        Self::new()
    }
}

impl ThermalZonesLive {
    pub fn new() -> Self {
        let a380 = topology_a380::build();
        let zone_vars = a380
            .network
            .zones
            .iter()
            .map(|z| {
                let up = z.name.to_uppercase();
                ZoneVars {
                    temperature_c: format!("THERMAL_ZONE_{up}_TEMPERATURE_C"),
                    structure_temperature_c: format!("THERMAL_ZONE_{up}_STRUCTURE_TEMPERATURE_C"),
                    smoke_concentration: format!("THERMAL_ZONE_{up}_SMOKE_CONCENTRATION"),
                }
            })
            .collect();
        let damage_vars = a380.damage.components.iter().map(|c| format!("THERMAL_COMPONENT_{}_DAMAGE", c.name.to_uppercase())).collect();
        Self { a380, zone_vars, damage_vars, gear_door_jammed_at: [None; 3] }
    }

    /// Whether the electric ventilation fans have a bus to run on
    /// (module doc).
    fn fan_power_fraction(truth: &Truth) -> f64 {
        if truth.ac_bus_volts.iter().any(|&v| v >= MIN_FAN_BUS_VOLTS) {
            1.0
        } else {
            0.0
        }
    }

    fn outside_air(truth: &Truth) -> OutsideAir {
        OutsideAir {
            static_temp_c: truth.environment.sat_c,
            mach: truth.environment.mach(),
            true_airspeed_m_s: truth.environment.tas_ms,
        }
    }

    /// ATA 21: the five electric extract fans plus the two ram-air paths
    /// (`registry::ventilation_zones`, in its own order).
    fn apply_ventilation_failures(&mut self, faults: &Faults, fan_power: f64) {
        let v = &self.a380.vents;
        let electric = [
            (v.main_avionics_fan, f(21, 1)),
            (v.upper_avionics_fan, f(21, 2)),
            (v.cargo_fwd_fan, f(21, 3)),
            (v.cargo_aft_fan, f(21, 4)),
            (v.cargo_bulk_fan, f(21, 5)),
        ];
        for (link, id) in electric {
            let health = fan_power * (1.0 - faults.get(id));
            self.a380.network.set_ventilation_health(link, health);
        }
        // Ram-air scoop/drain paths: no electrical supply of their own.
        self.a380.network.set_ventilation_health(v.belly_pack_bay_vent, 1.0 - faults.get(f(21, 6)));
        self.a380.network.set_ventilation_health(v.apu_compartment_vent, 1.0 - faults.get(f(21, 7)));
    }

    /// ATA 26: cargo, nacelle and APU compartment fires -- heat and smoke
    /// into the zone the fire is in.
    fn apply_fire_failures(&mut self, faults: &Faults) {
        let z = &self.a380.zones;
        let cargo = [(z.cargo_fwd, f(26, 1)), (z.cargo_aft, f(26, 2)), (z.cargo_bulk, f(26, 3))];
        for (zone, id) in cargo {
            let severity = faults.get(id);
            if severity > 0.0 {
                self.a380.network.inject_heat_w(zone, severity * CARGO_FIRE_MAX_HEAT_W);
                self.a380.network.inject_smoke_kg_s(zone, severity * CARGO_FIRE_MAX_SMOKE_KG_S);
            }
        }
        for engine in 0..4usize {
            let severity = faults.get(f(26, 4 + engine as u16));
            if severity > 0.0 {
                let zone = z.nacelle_cowl[engine];
                self.a380.network.inject_heat_w(zone, severity * NACELLE_FIRE_MAX_HEAT_W);
                self.a380.network.inject_smoke_kg_s(zone, severity * NACELLE_FIRE_MAX_SMOKE_KG_S);
            }
        }
        let apu = faults.get(f(26, 8));
        if apu > 0.0 {
            self.a380.network.inject_heat_w(z.apu_compartment, apu * APU_FIRE_MAX_HEAT_W);
            self.a380.network.inject_smoke_kg_s(z.apu_compartment, apu * APU_FIRE_MAX_SMOKE_KG_S);
        }
    }

    /// ATA 30: wing/nacelle anti-ice duct leaks (heat) and nacelle vent
    /// scoop ice blockage (ventilation health).
    fn apply_ice_and_duct_failures(&mut self, faults: &Faults) {
        let z = &self.a380.zones;
        for (zone, id) in [(z.wing_le_left, f(30, 1)), (z.wing_le_right, f(30, 2))] {
            let leak = faults.get(id);
            if leak > 0.0 {
                self.a380.network.inject_heat_w(zone, leak * WING_DUCT_LEAK_MAX_HEAT_W);
            }
        }
        for engine in 0..4usize {
            let leak = faults.get(f(30, 3 + engine as u16));
            if leak > 0.0 {
                self.a380.network.inject_heat_w(z.nacelle_cowl[engine], leak * NACELLE_DUCT_LEAK_MAX_HEAT_W);
            }
        }
        for engine in 0..4usize {
            let blockage = faults.get(f(30, 7 + engine as u16));
            let link = self.a380.vents.nacelle_vent[engine];
            self.a380.network.set_ventilation_health(link, 1.0 - blockage);
        }
    }

    /// ATA 32: a jammed bay door stops following its commanded position
    /// and stays where it was. `Truth` carries no commanded door position
    /// yet (module doc), so "where it was" is `topology_a380`'s resting
    /// closed state; the latch itself is real and needs no change once
    /// that input exists.
    fn apply_gear_door_failures(&mut self, faults: &Faults) {
        let doors = [
            (0usize, self.a380.vents.nose_gear_door, f(32, 1)),
            (1, self.a380.vents.wing_gear_door, f(32, 2)),
            (2, self.a380.vents.body_gear_door, f(32, 3)),
        ];
        for (i, link, id) in doors {
            let jam = faults.get(id);
            if jam > 0.0 {
                let stuck_at = *self.gear_door_jammed_at[i].get_or_insert_with(|| self.a380.network.ventilation_links[link].health);
                // A partial jam still partly follows the commanded
                // position; a full jam holds `stuck_at` outright.
                let commanded = self.a380.network.ventilation_links[link].health;
                let health = commanded + (stuck_at - commanded) * jam;
                self.a380.network.set_ventilation_health(link, health);
            } else {
                self.gear_door_jammed_at[i] = None;
            }
        }
    }

    /// ATA 36/49: bleed duct runs through the pylons and the tail cone.
    fn apply_bleed_duct_failures(&mut self, faults: &Faults) {
        for engine in 0..4usize {
            let leak = faults.get(f(36, 1 + engine as u16));
            if leak > 0.0 {
                self.a380.network.inject_heat_w(self.a380.zones.pylon[engine], leak * PYLON_BLEED_LEAK_MAX_HEAT_W);
            }
        }
        let apu_duct = faults.get(f(49, 1));
        if apu_duct > 0.0 {
            self.a380.network.inject_heat_w(self.a380.zones.tail_cone, apu_duct * APU_DUCT_LEAK_MAX_HEAT_W);
        }
    }

    /// ATA 53: the crown insulation blanket's condition is the zone's own
    /// `insulation_effectiveness` (1 = intact .. 0 = missing).
    fn apply_insulation_failures(&mut self, faults: &Faults) {
        let zone = self.a380.zones.crown_area;
        self.a380.network.zones[zone].insulation_effectiveness = 1.0 - faults.get(f(53, 1));
    }

    /// Read access for tests and for anything that wants the network
    /// itself rather than its published variables.
    pub fn network(&self) -> &super::network::ThermalNetwork {
        &self.a380.network
    }
}

impl crate::deep::live::Area for ThermalZonesLive {
    fn name(&self) -> &'static str {
        "thermal_zones"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        let fan_power = Self::fan_power_fraction(truth);
        self.apply_ventilation_failures(faults, fan_power);
        self.apply_ice_and_duct_failures(faults);
        self.apply_gear_door_failures(faults);
        self.apply_insulation_failures(faults);
        // Heat/smoke sources are accumulated per tick and consumed by the
        // step below, so they are injected last, immediately before it.
        self.apply_fire_failures(faults);
        self.apply_bleed_duct_failures(faults);

        let outside = Self::outside_air(truth);
        self.a380.network.step(truth.dt_s, &outside, SOLAR_FLUX_W_M2);
        self.a380.damage.update(&self.a380.network, truth.dt_s);
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        for (i, vars) in self.zone_vars.iter().enumerate() {
            out(&vars.temperature_c, self.a380.network.air_temp_c(i));
            out(&vars.structure_temperature_c, self.a380.network.structure_temp_c(i));
            out(&vars.smoke_concentration, self.a380.network.smoke_concentration(i));
        }
        for (i, name) in self.damage_vars.iter().enumerate() {
            out(name, self.a380.damage.damage_fraction(i));
        }
    }
}

pub fn live_system() -> Box<dyn crate::deep::live::Area> {
    Box::new(ThermalZonesLive::new())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    fn powered_ground_truth() -> Truth {
        Truth { dt_s: 1.0, ac_bus_volts: [115.0; 4], ..Truth::default() }
    }

    fn published(area: &dyn crate::deep::live::Area) -> BTreeMap<String, f64> {
        let mut map = BTreeMap::new();
        area.publish(&mut |name, value| {
            map.insert(name.to_string(), value);
        });
        map
    }

    fn run(area: &mut dyn crate::deep::live::Area, truth: &Truth, faults: &Faults, ticks: usize) {
        for _ in 0..ticks {
            area.tick(truth, faults);
        }
    }

    #[test]
    fn every_variable_the_registry_triggers_on_is_actually_published() {
        // The whole point of this file: an ECAM trigger reading a variable
        // nobody publishes can never fire. Every name below is read by a
        // trigger or contribution in `registry.rs`.
        let area = live_system();
        let map = published(area.as_ref());
        let required = [
            "THERMAL_ZONE_CARGOFWD_SMOKE_CONCENTRATION",
            "THERMAL_ZONE_CARGOAFT_SMOKE_CONCENTRATION",
            "THERMAL_ZONE_CARGOBULK_SMOKE_CONCENTRATION",
            "THERMAL_ZONE_NACELLECOWL1_TEMPERATURE_C",
            "THERMAL_ZONE_NACELLECOWL2_TEMPERATURE_C",
            "THERMAL_ZONE_NACELLECOWL3_TEMPERATURE_C",
            "THERMAL_ZONE_NACELLECOWL4_TEMPERATURE_C",
            "THERMAL_ZONE_APUCOMPARTMENT_TEMPERATURE_C",
            "THERMAL_ZONE_WINGLELEFT_TEMPERATURE_C",
            "THERMAL_ZONE_WINGLERIGHT_TEMPERATURE_C",
            "THERMAL_ZONE_BELLYFAIRINGPACKS_TEMPERATURE_C",
            "THERMAL_ZONE_MAINAVIONICS_TEMPERATURE_C",
            "THERMAL_COMPONENT_MAINAVIONICSWIRINGBUNDLE_DAMAGE",
        ];
        for name in required {
            assert!(map.contains_key(name), "{name} is read by an ECAM trigger but never published");
        }
        assert_eq!(map.len(), 26 * 3 + 5, "26 zones x 3 variables plus the 5 registered thermal components");
    }

    #[test]
    fn a_cargo_fire_raises_the_published_smoke_concentration_past_what_the_detectors_see() {
        // Failure 11_026_001 (CargoFwd cargo compartment fire), effect:
        // "CargoFwd air temperature and smoke concentration rise". The
        // contribution to CARGO_SMOKE_FWD triggers above 2e-4.
        let truth = powered_ground_truth();
        let mut area = live_system();
        let armed = Faults::from_pairs([(f(26, 1), 1.0)]);
        run(area.as_mut(), &truth, &armed, 300);
        let hot = published(area.as_ref());

        let mut healthy = live_system();
        run(healthy.as_mut(), &truth, &Faults::default(), 300);
        let cold = published(healthy.as_ref());

        let smoke = hot["THERMAL_ZONE_CARGOFWD_SMOKE_CONCENTRATION"];
        assert!(smoke > 0.0002, "a full-severity cargo fire must put the bay past the detectors' 2e-4 threshold, got {smoke}");
        assert_eq!(cold["THERMAL_ZONE_CARGOFWD_SMOKE_CONCENTRATION"], 0.0);
        assert!(
            hot["THERMAL_ZONE_CARGOFWD_TEMPERATURE_C"] > cold["THERMAL_ZONE_CARGOFWD_TEMPERATURE_C"] + 50.0,
            "the fire must dominate the bay's own temperature: {} vs {}",
            hot["THERMAL_ZONE_CARGOFWD_TEMPERATURE_C"],
            cold["THERMAL_ZONE_CARGOFWD_TEMPERATURE_C"]
        );
    }

    #[test]
    fn arming_the_main_avionics_fan_failure_heats_the_bay_and_damages_its_wiring() {
        // Failure 11_021_001, effect: "MainAvionics loses cooling/purge
        // airflow; its steady-state air temperature rises ... and any
        // registered thermal component there accrues damage faster."
        // AVIONICS_VENT_FAULT triggers above 70 C or on any wiring damage.
        let truth = powered_ground_truth();
        let mut failed = live_system();
        let mut healthy = live_system();
        let armed = Faults::from_pairs([(f(21, 1), 1.0)]);
        run(failed.as_mut(), &truth, &armed, 20_000);
        run(healthy.as_mut(), &truth, &Faults::default(), 20_000);

        let failed_vars = published(failed.as_ref());
        let healthy_vars = published(healthy.as_ref());
        let hot = failed_vars["THERMAL_ZONE_MAINAVIONICS_TEMPERATURE_C"];
        let cool = healthy_vars["THERMAL_ZONE_MAINAVIONICS_TEMPERATURE_C"];
        assert!(hot > cool + 5.0, "a failed extract fan must leave the bay hotter: {hot} vs {cool}");
        assert!(hot > 70.0, "it must reach the AVIONICS VENT FAULT trigger temperature, got {hot}");
        assert!(
            failed_vars["THERMAL_COMPONENT_MAINAVIONICSWIRINGBUNDLE_DAMAGE"] > 0.0,
            "the wiring bundle registered in that bay must start accruing damage once it runs over its 70 C limit"
        );
        assert_eq!(healthy_vars["THERMAL_COMPONENT_MAINAVIONICSWIRINGBUNDLE_DAMAGE"], 0.0);
    }

    #[test]
    fn losing_every_ac_bus_stops_the_extract_fans_exactly_as_a_fan_failure_does() {
        let unpowered = Truth { dt_s: 1.0, ..Truth::default() }; // ac_bus_volts all 0
        let powered = powered_ground_truth();
        let mut dark = live_system();
        let mut live = live_system();
        run(dark.as_mut(), &unpowered, &Faults::default(), 20_000);
        run(live.as_mut(), &powered, &Faults::default(), 20_000);
        let dark_temp = published(dark.as_ref())["THERMAL_ZONE_MAINAVIONICS_TEMPERATURE_C"];
        let live_temp = published(live.as_ref())["THERMAL_ZONE_MAINAVIONICS_TEMPERATURE_C"];
        assert!(dark_temp > live_temp + 5.0, "unpowered fans must leave the bay hotter: {dark_temp} vs {live_temp}");
    }

    #[test]
    fn a_crown_insulation_failure_lets_the_crown_track_a_cold_outside_faster() {
        // Failure 11_053_001, effect: "CrownArea's structure tracks the
        // outside recovery temperature much more closely (colder at
        // altitude ...)".
        let truth = Truth {
            dt_s: 1.0,
            environment: crate::deep::integration::weather_truth::EnvironmentTruth { sat_c: -50.0, tas_ms: 230.0, ambient_pressure_pa: 25_000.0, ..Truth::default().environment },
            altitude_ft: 35_000.0,
            on_ground: false,
            ac_bus_volts: [115.0; 4],
            ..Truth::default()
        };
        let mut damaged = live_system();
        let mut intact = live_system();
        let armed = Faults::from_pairs([(f(53, 1), 1.0)]);
        run(damaged.as_mut(), &truth, &armed, 600);
        run(intact.as_mut(), &truth, &Faults::default(), 600);
        let damaged_c = published(damaged.as_ref())["THERMAL_ZONE_CROWNAREA_STRUCTURE_TEMPERATURE_C"];
        let intact_c = published(intact.as_ref())["THERMAL_ZONE_CROWNAREA_STRUCTURE_TEMPERATURE_C"];
        assert!(damaged_c < intact_c - 2.0, "a damaged blanket must chill faster: {damaged_c} vs {intact_c}");
    }

    #[test]
    fn a_nacelle_fire_drives_that_cowl_past_its_overheat_trigger_and_leaves_the_others_alone() {
        // Failure 11_026_005 (engine 2 nacelle fire) raises ENG 2 NAC OVHT
        // (trigger: THERMAL_ZONE_NACELLECOWL2_TEMPERATURE_C > 150 C).
        let truth = powered_ground_truth();
        let mut area = live_system();
        let armed = Faults::from_pairs([(f(26, 5), 1.0)]);
        run(area.as_mut(), &truth, &armed, 200);
        let map = published(area.as_ref());
        assert!(map["THERMAL_ZONE_NACELLECOWL2_TEMPERATURE_C"] > 150.0, "got {}", map["THERMAL_ZONE_NACELLECOWL2_TEMPERATURE_C"]);
        assert!(map["THERMAL_ZONE_NACELLECOWL1_TEMPERATURE_C"] < 150.0, "engine 1's cowl has no fire");
    }

    #[test]
    fn a_blocked_nacelle_vent_scoop_makes_the_same_duct_leak_hotter() {
        // Failure 11_030_007 (engine 1 vent scoop ice blockage), effect:
        // "NacelleCowl1 loses its large ram-air ventilation term, so any
        // heat present (engine proximity, a duct leak) accumulates faster".
        let truth = powered_ground_truth();
        let leak_only = Faults::from_pairs([(f(30, 3), 1.0)]);
        let leak_and_blockage = Faults::from_pairs([(f(30, 3), 1.0), (f(30, 7), 1.0)]);
        let mut vented = live_system();
        let mut blocked = live_system();
        run(vented.as_mut(), &truth, &leak_only, 600);
        run(blocked.as_mut(), &truth, &leak_and_blockage, 600);
        let vented_c = published(vented.as_ref())["THERMAL_ZONE_NACELLECOWL1_TEMPERATURE_C"];
        let blocked_c = published(blocked.as_ref())["THERMAL_ZONE_NACELLECOWL1_TEMPERATURE_C"];
        assert!(blocked_c > vented_c + 10.0, "blocked {blocked_c} vs vented {vented_c}");
    }

    #[test]
    fn an_unarmed_cold_aircraft_publishes_finite_values_and_no_smoke() {
        let mut area = live_system();
        let truth = Truth::default();
        run(area.as_mut(), &truth, &Faults::default(), 100);
        for (name, value) in published(area.as_ref()) {
            assert!(value.is_finite(), "{name} went non-finite");
            if name.ends_with("_SMOKE_CONCENTRATION") {
                assert_eq!(value, 0.0, "{name} must be clean with nothing burning");
            }
        }
    }

    #[test]
    fn a_zero_length_frame_changes_nothing() {
        let mut area = live_system();
        let truth = Truth { dt_s: 0.0, ..powered_ground_truth() };
        area.tick(&truth, &Faults::default());
        let before = published(area.as_ref());
        area.tick(&truth, &Faults::default());
        assert_eq!(before, published(area.as_ref()));
    }
}
