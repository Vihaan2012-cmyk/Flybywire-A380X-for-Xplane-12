use super::network::OutsideAir;
use super::topology_a380::{self, A380Thermal};
use crate::deep::api::{failure_id, Area as RegArea};
use crate::deep::live::{Faults, Truth};

pub const MIN_FAN_BUS_VOLTS: f64 = 100.0;

const SOLAR_CONSTANT_W_M2: f64 = 1361.0;

const CLEAR_SKY_TRANSMITTANCE: f64 = 0.75;

fn solar_flux_w_m2(truth: &Truth) -> f64 {
    let elevation_rad = truth.sun_elevation_deg.to_radians();
    if elevation_rad <= 0.0 {
        0.0
    } else {
        SOLAR_CONSTANT_W_M2 * CLEAR_SKY_TRANSMITTANCE * elevation_rad.sin()
    }
}

const CARGO_FIRE_MAX_HEAT_W: f64 = 200_000.0;
const CARGO_FIRE_MAX_SMOKE_KG_S: f64 = 0.01;
const NACELLE_FIRE_MAX_HEAT_W: f64 = 500_000.0;
const NACELLE_FIRE_MAX_SMOKE_KG_S: f64 = 0.005;
const APU_FIRE_MAX_HEAT_W: f64 = 300_000.0;
const APU_FIRE_MAX_SMOKE_KG_S: f64 = 0.008;
const LAVATORY_FIRE_MAX_HEAT_W: f64 = 20_000.0;
const LAVATORY_FIRE_MAX_SMOKE_KG_S: f64 = 0.002;
const AVNCS_FIRE_MAX_HEAT_W: f64 = 20_000.0;
const AVNCS_FIRE_MAX_SMOKE_KG_S: f64 = 0.002;
const LDCR_FIRE_MAX_HEAT_W: f64 = 20_000.0;
const LDCR_FIRE_MAX_SMOKE_KG_S: f64 = 0.002;
const APU_BLEED_DUCT_BORE_M: f64 = 0.08;

const ANTI_ICE_DUCT_BORE_M: f64 = 0.05;

const GAMMA_AIR_LEAK: f64 = 1.4;

const R_AIR_J_PER_KG_K: f64 = 287.057_005;

const CRITICAL_PRESSURE_RATIO: f64 = 0.528_281_787_717_685_7;

const PYLON_BLEED_DUCT_BORE_M: f64 = 0.1016;

const PYLON_BLEED_LEAK_AREA_FRACTION_OF_BORE: f64 = 0.02;

const BLEED_LEAK_DISCHARGE_COEFFICIENT: f64 = 0.65;

fn orifice_mass_flow_kg_s(discharge_coefficient: f64, area_m2: f64, upstream_pa: f64, upstream_k: f64, downstream_pa: f64) -> f64 {
    if area_m2 <= 0.0 || upstream_pa <= 0.0 || upstream_k <= 0.0 || downstream_pa >= upstream_pa {
        return 0.0;
    }
    let pressure_ratio = (downstream_pa.max(0.0) / upstream_pa).clamp(0.0, 1.0);
    let flow_function = if pressure_ratio <= CRITICAL_PRESSURE_RATIO {
        (GAMMA_AIR_LEAK * (2.0 / (GAMMA_AIR_LEAK + 1.0)).powf((GAMMA_AIR_LEAK + 1.0) / (GAMMA_AIR_LEAK - 1.0))).sqrt()
    } else {
        (2.0 * GAMMA_AIR_LEAK / (GAMMA_AIR_LEAK - 1.0) * (pressure_ratio.powf(2.0 / GAMMA_AIR_LEAK) - pressure_ratio.powf((GAMMA_AIR_LEAK + 1.0) / GAMMA_AIR_LEAK)))
            .max(0.0)
            .sqrt()
    };
    discharge_coefficient.max(0.0) * area_m2 * upstream_pa / (R_AIR_J_PER_KG_K * upstream_k).sqrt() * flow_function
}

fn pylon_bleed_leak_heat_w(severity: f64, duct_pa: f64, duct_k: f64, bay_air_k: f64, bay_pa: f64) -> f64 {
    duct_leak_heat_w(severity, PYLON_BLEED_DUCT_BORE_M, PYLON_BLEED_LEAK_AREA_FRACTION_OF_BORE, duct_pa, duct_k, bay_air_k, bay_pa)
}

fn duct_leak_heat_w(severity: f64, bore_m: f64, leak_area_fraction_of_bore: f64, duct_pa: f64, duct_k: f64, bay_air_k: f64, bay_pa: f64) -> f64 {
    let severity = severity.clamp(0.0, 1.0);
    if severity <= 0.0 {
        return 0.0;
    }
    let bore_area_m2 = std::f64::consts::PI / 4.0 * bore_m * bore_m;
    let crack_area_m2 = severity * leak_area_fraction_of_bore * bore_area_m2;
    let mdot = orifice_mass_flow_kg_s(BLEED_LEAK_DISCHARGE_COEFFICIENT, crack_area_m2, duct_pa, duct_k, bay_pa);
    mdot * super::network::CP_AIR_J_PER_KG_K * (duct_k - bay_air_k).max(0.0)
}

fn anti_ice_duct_leak_heat_w(severity: f64, duct_pa: f64, duct_k: f64, bay_air_k: f64, bay_pa: f64) -> f64 {
    duct_leak_heat_w(severity, ANTI_ICE_DUCT_BORE_M, PYLON_BLEED_LEAK_AREA_FRACTION_OF_BORE, duct_pa, duct_k, bay_air_k, bay_pa)
}

fn apu_bleed_leak_heat_w(severity: f64, duct_pa: f64, duct_k: f64, bay_air_k: f64, bay_pa: f64) -> f64 {
    duct_leak_heat_w(severity, APU_BLEED_DUCT_BORE_M, PYLON_BLEED_LEAK_AREA_FRACTION_OF_BORE, duct_pa, duct_k, bay_air_k, bay_pa)
}

fn f(ata: u16, n: u16) -> u64 {
    failure_id(RegArea::ThermalZones, ata, n)
}

pub const CONTENT_FIRE_HEAT_VARS: [&str; 3] = ["THERMAL_ZONE_CARGOFWD_FIRE_HEAT_W", "THERMAL_ZONE_CARGOAFT_FIRE_HEAT_W", "THERMAL_ZONE_MAINAVIONICS_FIRE_HEAT_W"];

struct ZoneVars {
    temperature_c: String,
    structure_temperature_c: String,
    smoke_concentration: String,
}

pub struct ThermalZonesLive {
    a380: A380Thermal,
    zone_vars: Vec<ZoneVars>,
    damage_vars: Vec<String>,
    gear_door_jammed_at: [Option<f64>; 3],
    fwd_cargo_trv_fault: f64,
    ths_bay_vent_fault: f64,
    bulk_cargo_duct_temp_c: f64,
    trim_air_duct_temp_c: f64,
    upper_avionics_fan_fault: f64,
    cargo_aft_fan_fault: f64,
    content_fire_heat_w: [f64; 3],
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
        Self {
            a380,
            zone_vars,
            damage_vars,
            gear_door_jammed_at: [None; 3],
            fwd_cargo_trv_fault: 0.0,
            ths_bay_vent_fault: 0.0,
            bulk_cargo_duct_temp_c: 20.0,
            trim_air_duct_temp_c: 20.0,
            content_fire_heat_w: [0.0; 3],
            upper_avionics_fan_fault: 0.0,
            cargo_aft_fan_fault: 0.0,
        }
    }

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
        self.a380.network.set_ventilation_health(v.belly_pack_bay_vent, 1.0 - faults.get(f(21, 6)));
        self.a380.network.set_ventilation_health(v.apu_compartment_vent, 1.0 - faults.get(f(21, 7)));
    }

    fn apply_fire_failures(&mut self, faults: &Faults) {
        let z = &self.a380.zones;
        self.content_fire_heat_w = [faults.get(f(26, 1)) * CARGO_FIRE_MAX_HEAT_W, faults.get(f(26, 2)) * CARGO_FIRE_MAX_HEAT_W, faults.get(f(26, 12)) * AVNCS_FIRE_MAX_HEAT_W];
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
        for (zone, id) in [(z.cabin_main_deck, f(26, 9)), (z.cabin_upper_deck, f(26, 10))] {
            let severity = faults.get(id);
            if severity > 0.0 {
                self.a380.network.inject_heat_w(zone, severity * LAVATORY_FIRE_MAX_HEAT_W);
                self.a380.network.inject_smoke_kg_s(zone, severity * LAVATORY_FIRE_MAX_SMOKE_KG_S);
            }
        }

        for (zone, id) in [(z.aft_avionics, f(26, 11)), (z.main_avionics, f(26, 12)), (z.upper_avionics, f(26, 13))] {
            let severity = faults.get(id);
            if severity > 0.0 {
                self.a380.network.inject_heat_w(zone, severity * AVNCS_FIRE_MAX_HEAT_W);
                self.a380.network.inject_smoke_kg_s(zone, severity * AVNCS_FIRE_MAX_SMOKE_KG_S);
            }
        }
        let ldcr = faults.get(f(26, 14));
        if ldcr > 0.0 {
            self.a380.network.inject_heat_w(z.fwd_lower_crew_rest, ldcr * LDCR_FIRE_MAX_HEAT_W);
            self.a380.network.inject_smoke_kg_s(z.fwd_lower_crew_rest, ldcr * LDCR_FIRE_MAX_SMOKE_KG_S);
        }
    }

    fn apply_ice_and_duct_failures(&mut self, faults: &Faults, truth: &Truth) {
        let z = &self.a380.zones;
        let bay_pa = truth.environment.ambient_pressure_pa;
        let wings: [(super::network::ZoneId, u64, [usize; 2]); 2] = [(z.wing_le_left, f(30, 1), [0, 1]), (z.wing_le_right, f(30, 2), [2, 3])];
        for (zone, id, engines) in wings {
            let leak = faults.get(id);
            if leak > 0.0 {
                let (a, b) = (engines[0], engines[1]);
                let engine = if truth.engine_bleed_pressure_pa[a] >= truth.engine_bleed_pressure_pa[b] { a } else { b };
                let bay_air_k = self.a380.network.air_temp_c(zone) + 273.15;
                let heat_w = anti_ice_duct_leak_heat_w(leak, truth.engine_bleed_pressure_pa[engine], truth.engine_bleed_temp_k[engine], bay_air_k, bay_pa);
                self.a380.network.inject_heat_w(zone, heat_w);
            }
        }
        for engine in 0..4usize {
            let leak = faults.get(f(30, 3 + engine as u16));
            if leak > 0.0 {
                let zone = z.nacelle_cowl[engine];
                let bay_air_k = self.a380.network.air_temp_c(zone) + 273.15;
                let heat_w = anti_ice_duct_leak_heat_w(leak, truth.engine_bleed_pressure_pa[engine], truth.engine_bleed_temp_k[engine], bay_air_k, bay_pa);
                self.a380.network.inject_heat_w(zone, heat_w);
            }
        }
        for engine in 0..4usize {
            let blockage = faults.get(f(30, 7 + engine as u16));
            let link = self.a380.vents.nacelle_vent[engine];
            self.a380.network.set_ventilation_health(link, 1.0 - blockage);
        }
    }

    fn apply_gear_door_failures(&mut self, faults: &Faults, commanded_open: [f64; 3]) {
        let doors = [
            (0usize, self.a380.vents.nose_gear_door, f(32, 1)),
            (1, self.a380.vents.wing_gear_door, f(32, 2)),
            (2, self.a380.vents.body_gear_door, f(32, 3)),
        ];
        for (i, link, id) in doors {
            let commanded = commanded_open[i].clamp(0.0, 1.0);
            let jam = faults.get(id);
            if jam > 0.0 {
                let stuck_at = *self.gear_door_jammed_at[i].get_or_insert_with(|| self.a380.network.ventilation_links[link].health);
                let health = commanded + (stuck_at - commanded) * jam;
                self.a380.network.set_ventilation_health(link, health);
            } else {
                self.gear_door_jammed_at[i] = None;
                self.a380.network.set_ventilation_health(link, commanded);
            }
        }
    }

    fn apply_bleed_duct_failures(&mut self, faults: &Faults, truth: &Truth) {
        for engine in 0..4usize {
            let leak = faults.get(f(36, 1 + engine as u16));
            if leak > 0.0 {
                let zone = self.a380.zones.pylon[engine];
                let bay_air_k = self.a380.network.air_temp_c(zone) + 273.15;
                let heat_w = pylon_bleed_leak_heat_w(
                    leak,
                    truth.engine_bleed_pressure_pa[engine],
                    truth.engine_bleed_temp_k[engine],
                    bay_air_k,
                    truth.environment.ambient_pressure_pa,
                );
                self.a380.network.inject_heat_w(zone, heat_w);
            }
        }
        let apu_duct = faults.get(f(49, 1));
        if apu_duct > 0.0 {
            let zone = self.a380.zones.tail_cone;
            let bay_air_k = self.a380.network.air_temp_c(zone) + 273.15;
            let apu_duct_k = truth.published.get_or("DEEP_PNEU_APU_DUCT_TEMPERATURE_C", truth.environment.sat_c) + 273.15;
            let heat_w = apu_bleed_leak_heat_w(apu_duct, truth.apu_bleed_pressure_pa, apu_duct_k, bay_air_k, truth.environment.ambient_pressure_pa);
            self.a380.network.inject_heat_w(zone, heat_w);
        }
    }

    fn apply_insulation_failures(&mut self, faults: &Faults) {
        let zone = self.a380.zones.crown_area;
        self.a380.network.zones[zone].insulation_effectiveness = 1.0 - faults.get(f(53, 1));
    }

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
        self.upper_avionics_fan_fault = faults.get(f(21, 2));
        self.cargo_aft_fan_fault = faults.get(f(21, 4));
        self.apply_ice_and_duct_failures(faults, truth);
        self.apply_gear_door_failures(faults, truth.controls.gear_door_commanded_open);
        self.apply_insulation_failures(faults);
        self.apply_fire_failures(faults);
        self.apply_bleed_duct_failures(faults, truth);

        let outside = Self::outside_air(truth);
        self.a380.network.step(truth.dt_s, &outside, solar_flux_w_m2(truth));
        self.a380.damage.update(&self.a380.network, truth.dt_s);

        self.fwd_cargo_trv_fault = faults.get(f(21, 8));
        self.ths_bay_vent_fault = faults.get(f(21, 9));

        const DUCT_OVERHEAT_EXCESS_C: f64 = 120.0;
        let bulk_cargo_duct_overheat = faults.get(f(21, 10));
        self.bulk_cargo_duct_temp_c = self.a380.network.air_temp_c(self.a380.zones.cargo_bulk) + bulk_cargo_duct_overheat * DUCT_OVERHEAT_EXCESS_C;
        let trim_air_duct_overheat = faults.get(f(21, 11));
        self.trim_air_duct_temp_c = self.a380.network.air_temp_c(self.a380.zones.cabin_main_deck) + trim_air_duct_overheat * DUCT_OVERHEAT_EXCESS_C;
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
        out("DEEP_THERM_FWD_CARGO_TRV_FAULT", self.fwd_cargo_trv_fault);
        out("DEEP_THERM_THS_BAY_VENT_FAULT", self.ths_bay_vent_fault);
        out("DEEP_THERM_BULK_CARGO_DUCT_TEMPERATURE_C", self.bulk_cargo_duct_temp_c);
        out("DEEP_THERM_TRIM_AIR_DUCT_TEMPERATURE_C", self.trim_air_duct_temp_c);
        for (name, heat_w) in CONTENT_FIRE_HEAT_VARS.iter().zip(self.content_fire_heat_w) {
            out(name, heat_w);
        }
        out("DEEP_THERM_UPPER_AVIONICS_FAN_FAULT", self.upper_avionics_fan_fault);
        out("DEEP_THERM_CARGO_AFT_FAN_FAULT", self.cargo_aft_fan_fault);
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
        assert_eq!(
            map.len(),
            28 * 3 + 5 + 4 + 3 + 2,
            "28 zones x 3 variables plus the 5 registered thermal components plus the 4 ECAM-completeness additions plus the 3 content-fire heat-release vars plus the 2 fan-fault vars"
        );
    }

    #[test]
    fn a_cargo_fire_raises_the_published_smoke_concentration_past_what_the_detectors_see() {
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
        let unpowered = Truth { dt_s: 1.0, ..Truth::default() };
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
    fn a_commanded_open_gear_door_ventilates_the_bay_toward_outside_air_faster_than_closed() {
        let cold_air = Truth {
            dt_s: 1.0,
            environment: crate::deep::integration::weather_truth::EnvironmentTruth { sat_c: -50.0, tas_ms: 230.0, ambient_pressure_pa: 25_000.0, ..Truth::default().environment },
            altitude_ft: 35_000.0,
            on_ground: false,
            ac_bus_volts: [115.0; 4],
            ..Truth::default()
        };
        let mut open_truth = cold_air.clone();
        open_truth.controls.gear_door_commanded_open = [0.0, 1.0, 0.0];
        let closed_truth = cold_air;

        let mut open = live_system();
        let mut closed = live_system();
        run(open.as_mut(), &open_truth, &Faults::default(), 300);
        run(closed.as_mut(), &closed_truth, &Faults::default(), 300);
        let open_c = published(open.as_ref())["THERMAL_ZONE_WINGGEARWELL_TEMPERATURE_C"];
        let closed_c = published(closed.as_ref())["THERMAL_ZONE_WINGGEARWELL_TEMPERATURE_C"];
        assert!(closed_c > open_c + 5.0, "a door truly commanded open must ventilate the bay toward the cold outside air faster than one held closed: open {open_c} vs closed {closed_c}");
    }

    #[test]
    fn a_high_sun_elevation_heats_a_sun_exposed_zones_structure_more_than_no_sun_at_all() {
        let mut high_sun = powered_ground_truth();
        high_sun.sun_elevation_deg = 60.0;
        let mut no_sun = powered_ground_truth();
        no_sun.sun_elevation_deg = -10.0;
        let mut high = live_system();
        let mut low = live_system();
        run(high.as_mut(), &high_sun, &Faults::default(), 3000);
        run(low.as_mut(), &no_sun, &Faults::default(), 3000);
        let hot = published(high.as_ref())["THERMAL_ZONE_CROWNAREA_STRUCTURE_TEMPERATURE_C"];
        let cold = published(low.as_ref())["THERMAL_ZONE_CROWNAREA_STRUCTURE_TEMPERATURE_C"];
        assert!(hot > cold + 1.0, "a high sun elevation must warm a sun-exposed zone's structure more than no sun at all: {hot} vs {cold}");
    }

    #[test]
    fn a_below_horizon_sun_never_produces_a_negative_or_nonzero_flux() {
        assert_eq!(solar_flux_w_m2(&Truth { sun_elevation_deg: -5.0, ..Truth::default() }), 0.0);
        assert_eq!(solar_flux_w_m2(&Truth { sun_elevation_deg: 0.0, ..Truth::default() }), 0.0);
        assert!(solar_flux_w_m2(&Truth { sun_elevation_deg: 90.0, ..Truth::default() }) > 0.0);
    }

    #[test]
    fn a_nacelle_fire_drives_that_cowl_past_its_overheat_trigger_and_leaves_the_others_alone() {
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
        let truth = takeoff_truth();
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

    const TAKEOFF_IP8_PA: f64 = 970_000.0;
    const TAKEOFF_IP8_K: f64 = 590.0;

    fn takeoff_truth() -> Truth {
        Truth {
            dt_s: 1.0,
            ac_bus_volts: [115.0; 4],
            engine_running: [true; 4],
            engine_n1_frac: [1.0; 4],
            engine_bleed_pressure_pa: [TAKEOFF_IP8_PA; 4],
            engine_bleed_temp_k: [TAKEOFF_IP8_K; 4],
            ..Truth::default()
        }
    }

    #[test]
    fn a_pylon_leak_carries_the_cracks_own_choked_flow_and_nothing_more() {
        let bay_k = 288.15;
        let ambient = 101_325.0;

        let takeoff = pylon_bleed_leak_heat_w(1.0, TAKEOFF_IP8_PA, TAKEOFF_IP8_K, bay_k, ambient);
        assert!((takeoff - 51_600.0).abs() < 300.0, "take-off IP8 port: expected ~51.6 kW from the derivation, got {takeoff}");

        let precooled = pylon_bleed_leak_heat_w(1.0, 404_694.0, 473.15, bay_k, ambient);
        assert!((precooled - 14_730.0).abs() < 200.0, "200 C / 44 psig: expected ~14.7 kW from the derivation, got {precooled}");
        assert!(precooled < 40_000.0 / 2.5, "the constant this replaced claimed these very conditions as its basis and was 2.7x larger than they give");

        let half = pylon_bleed_leak_heat_w(0.5, TAKEOFF_IP8_PA, TAKEOFF_IP8_K, bay_k, ambient);
        assert!((half - takeoff / 2.0).abs() < 1.0, "half the crack area must pass half the flow: {half} vs {takeoff}");
    }

    #[test]
    fn a_leak_from_an_unpressurised_duct_delivers_nothing() {
        let cold = Truth::default();
        let heat = pylon_bleed_leak_heat_w(1.0, cold.engine_bleed_pressure_pa[0], cold.engine_bleed_temp_k[0], 288.15, cold.environment.ambient_pressure_pa);
        assert_eq!(heat, 0.0, "a duct at ambient pressure has nothing to leak");

        let mut area = live_system();
        let armed = Faults::from_pairs([(f(36, 1), 1.0)]);
        run(area.as_mut(), &cold, &armed, 2000);
        let leaking = published(area.as_ref());
        let mut healthy_area = live_system();
        run(healthy_area.as_mut(), &cold, &Faults::default(), 2000);
        let healthy = published(healthy_area.as_ref());
        assert!(
            (leaking["THERMAL_ZONE_PYLONENGINE1_TEMPERATURE_C"] - healthy["THERMAL_ZONE_PYLONENGINE1_TEMPERATURE_C"]).abs() < 0.01,
            "a full-severity leak on a shut-down engine must not warm its pylon at all"
        );
    }

    #[test]
    fn a_wing_anti_ice_duct_leak_on_a_cold_aircraft_delivers_no_heat_and_running_engines_deliver_bounded_heat() {
        let cold = Truth::default();
        let mut cold_leaking = live_system();
        run(cold_leaking.as_mut(), &cold, &Faults::from_pairs([(f(30, 1), 1.0)]), 2000);
        let mut cold_healthy = live_system();
        run(cold_healthy.as_mut(), &cold, &Faults::default(), 2000);
        let leaking_c = published(cold_leaking.as_ref())["THERMAL_ZONE_WINGLELEFT_TEMPERATURE_C"];
        let healthy_c = published(cold_healthy.as_ref())["THERMAL_ZONE_WINGLELEFT_TEMPERATURE_C"];
        assert!((leaking_c - healthy_c).abs() < 0.01, "a full-severity wing duct leak on a shut-down aircraft must not warm the bay at all: leaking {leaking_c} vs healthy {healthy_c}");

        let hot = takeoff_truth();
        let mut hot_leaking = live_system();
        run(hot_leaking.as_mut(), &hot, &Faults::from_pairs([(f(30, 1), 1.0)]), 2000);
        let hot_leaking_c = published(hot_leaking.as_ref())["THERMAL_ZONE_WINGLELEFT_TEMPERATURE_C"];
        assert!(hot_leaking_c > healthy_c + 10.0, "a running engine's leak must actually heat the wing bay: {hot_leaking_c} vs healthy {healthy_c}");
        assert!(hot_leaking_c < TAKEOFF_IP8_K - 273.15, "and never past the duct feeding it: {hot_leaking_c} C");
    }

    #[test]
    fn a_pylon_bay_never_gets_hotter_than_the_air_leaking_into_it() {
        let cool_duct_k = 350.0;
        let truth = Truth { engine_bleed_temp_k: [cool_duct_k; 4], ..takeoff_truth() };
        let mut area = live_system();
        let armed = Faults::from_pairs([(f(36, 1), 1.0)]);
        run(area.as_mut(), &truth, &armed, 6000);
        let bay_c = published(area.as_ref())["THERMAL_ZONE_PYLONENGINE1_TEMPERATURE_C"];
        assert!(bay_c < cool_duct_k - 273.15, "the bay reached {bay_c} C from a duct at {} C", cool_duct_k - 273.15);
        assert!(bay_c > 20.0, "and it must still be a real, substantial leak, not a rounding error: {bay_c} C");
    }

    #[test]
    fn a_pylon_leak_heats_only_its_own_bay_and_by_how_much_the_vent_flow_allows() {
        let mut deep = crate::deep::live::Deep::new()
            .with_area(crate::deep::pneumatic_ducts::live::live_system())
            .with_area(live_system());
        let armed = Faults::from_pairs([(f(36, 1), 1.0)]);
        let truth = takeoff_truth();
        let mut published = BTreeMap::new();
        for _ in 0..3000 {
            deep.tick(truth.clone(), &armed, &mut |name, value| {
                published.insert(name.to_string(), value);
            });
        }
        let leaking = published["THERMAL_ZONE_PYLONENGINE1_TEMPERATURE_C"];
        let untouched = published["THERMAL_ZONE_PYLONENGINE2_TEMPERATURE_C"];
        assert!(leaking - untouched > 60.0, "a full-severity pylon leak at take-off port conditions must heat its own bay substantially: {leaking} C vs {untouched} C");
        assert!(leaking < TAKEOFF_IP8_K - 273.15, "and never past the duct feeding it: {leaking} C");
        assert!(
            leaking - untouched < 100.0,
            "the ram-vented bay cannot reach the 100 K margin pneumatic_ducts::odls confirms on -- if this ever exceeds 100 K, the leak's mass flow has been inflated past what the crack passes: {} K",
            leaking - untouched
        );
    }

}

