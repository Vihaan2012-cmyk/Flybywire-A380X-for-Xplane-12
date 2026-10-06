use super::anti_ice::{BleedAntiIceFaults, BleedAntiIceSurface, ProbeHeater, ProbeHeaterFaults, RainRemoval, RainRemovalFaults, WindowHeat, WindowHeatFaults, NACELLE_ANTI_ICE, WINDOW_TARGET_C, WING_ANTI_ICE};
use super::combustion::{Fluid, ZoneCombustion, ZoneSupply, HYDRAULIC_FLUID, JET_FUEL};
use super::extinguishing::{Bottle, BottleFaults, CargoSuppressionFaults, CargoSuppressionSystem, LavatoryFaults, LavatoryProtection, OpticalSmokeDetector, SmokeDetectorFaults, ZoneConcentration};
use super::fire_loops::{LoopFaults, LoopLogic, ZoneDetector};
use super::icing::{IcingEnvironment, IcingOutputs, IcingSurface, NACELLE_INLET, WINDSHIELD, WING_LEADING_EDGE};
use super::util::{air_dynamic_viscosity_pa_s, clamp01, collection_efficiency_beta0, droplet_inertia_parameter, recovery_temperature_c};
use crate::deep::api::{failure_id, Area as RegArea};
use crate::deep::integration::weather_truth::{dominant_cloud, droplet_diameter_m_from_conditions, lwc_kg_m3_from_conditions};
use crate::deep::gear_structure::live::N_BRAKED_WHEELS;
use crate::deep::live::{Area, DerivedFailure, Faults, Truth};
use crate::deep::thermal_zones::live::CONTENT_FIRE_HEAT_VARS;

const ATA_FIRE: u16 = 26;
const ATA_ICE: u16 = 30;

const ZONE_KEYS: [&str; 9] = ["ENG1", "ENG2", "ENG3", "ENG4", "APU", "MLG", "CARGO_FWD", "CARGO_AFT", "AVIONICS"];

const ZONE_MAX_LEAK_KG_S: [f64; 9] = [0.05, 0.05, 0.05, 0.05, 0.03, 0.01, 0.02, 0.02, 0.005];

const ZONE_VENTILATION_KG_S: [f64; 9] = [3.0, 3.0, 3.0, 3.0, 1.5, 1.2, 0.30, 0.30, 0.35];

const ZONE_VOLUME_M3: [f64; 9] = [8.0, 8.0, 8.0, 8.0, 6.0, 15.0, 110.0, 60.0, 12.0];

const ZONE_THERMAL_MASS_J_K: [f64; 9] = [1.5e5, 1.5e5, 1.5e5, 1.5e5, 2.0e5, 4.0e5, 5.0e5, 3.0e5, 5.0e5];

const AIR_DENSITY_KG_M3: f64 = 1.225;

const HEAVY_RAIN_LWC_KG_M3: f64 = 2.0e-3;

const RECOVERY_FACTOR: f64 = 0.9;

const RAIN_REMOVAL_JET_VELOCITY_M_S: f64 = 200.0;

const ZONE_FLUID: [Fluid; 9] = [JET_FUEL, JET_FUEL, JET_FUEL, JET_FUEL, JET_FUEL, HYDRAULIC_FLUID, HYDRAULIC_FLUID, HYDRAULIC_FLUID, HYDRAULIC_FLUID];

const ZONE_LINKS: [(usize, usize, f64); 2] = [(6, 8, 20.0), (5, 7, 25.0)];

const HOLD_SMOKE_VARS: [&str; 2] = ["THERMAL_ZONE_CARGOFWD_SMOKE_CONCENTRATION", "THERMAL_ZONE_CARGOAFT_SMOKE_CONCENTRATION"];

const PROBE_KEYS: [&str; 8] = ["PITOT1", "PITOT2", "PITOT3", "AOA1", "AOA2", "AOA3", "TAT1", "TAT2"];
const WINDOW_KEYS: [&str; 2] = ["L", "R"];

fn fire(n: u16) -> u64 {
    failure_id(RegArea::FireIce, ATA_FIRE, n)
}
fn ice(n: u16) -> u64 {
    failure_id(RegArea::FireIce, ATA_ICE, n)
}
fn on(b: bool) -> f64 {
    if b {
        1.0
    } else {
        0.0
    }
}

#[derive(Clone, Copy, Debug)]
struct Conditions {
    static_air_c: f64,
    recovery_c: f64,
    ambient_pressure_pa: f64,
    tas_m_s: f64,
    lwc_kg_m3: f64,
    droplet_diameter_m: f64,
}

impl Conditions {
    fn from(truth: &Truth) -> Self {
        let cloud = truth.environment.weather.as_ref().and_then(dominant_cloud);
        Self {
            static_air_c: truth.environment.sat_c,
            recovery_c: recovery_temperature_c(truth.environment.sat_c, truth.environment.tas_ms, RECOVERY_FACTOR),
            ambient_pressure_pa: truth.environment.ambient_pressure_pa.max(1.0),
            tas_m_s: truth.environment.tas_ms.max(0.0),
            lwc_kg_m3: lwc_kg_m3_from_conditions(truth.environment.sat_c, cloud),
            droplet_diameter_m: droplet_diameter_m_from_conditions(cloud),
        }
    }

    fn beta0(&self, characteristic_length_m: f64) -> f64 {
        let mu = air_dynamic_viscosity_pa_s(self.static_air_c);
        let k = droplet_inertia_parameter(self.droplet_diameter_m, self.tas_m_s, characteristic_length_m, mu);
        collection_efficiency_beta0(k)
    }

    fn icing_environment(&self) -> IcingEnvironment {
        IcingEnvironment {
            lwc_kg_m3: self.lwc_kg_m3,
            droplet_diameter_m: self.droplet_diameter_m,
            static_air_c: self.static_air_c,
            tas_m_s: self.tas_m_s,
            ambient_pressure_pa: self.ambient_pressure_pa,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct ZoneReport {
    temp_c: f64,
    burning: bool,
    burn_rate_kg_s: f64,
    fire: bool,
    loop_a_fault: bool,
    loop_b_fault: bool,
    loop_a_signal: bool,
    loop_b_signal: bool,
    agent_fraction: f64,
}

pub struct FireIceLive {
    detectors: Vec<ZoneDetector>,
    combustion: Vec<ZoneCombustion>,
    concentration: Vec<ZoneConcentration>,
    zones: [ZoneReport; 9],

    engine_bottles: Vec<[Bottle; 2]>,
    engine_bottle_low: [[bool; 2]; 4],
    engine_squib_discharged: [[bool; 2]; 4],
    engine_bottle_squib_blocked: [[bool; 2]; 4],
    engine_bottle_empty: [[bool; 2]; 4],
    engine_bottle_charge_loss: [[f64; 2]; 4],
    apu_bottle: Bottle,
    apu_squib_discharged: bool,
    apu_bottle_squib_blocked: bool,
    apu_bottle_empty: bool,
    apu_bottle_charge_loss: f64,

    cargo_suppression: Vec<CargoSuppressionSystem>,
    cargo_smoke: Vec<OpticalSmokeDetector>,
    cargo_smoke_alarm: [bool; 2],
    cargo_smoke_total_kg_m3: [f64; 2],
    cargo_distribution_fault: [bool; 2],
    cargo_knockdown_squib_fault: [bool; 2],
    cargo_extended_squib_fault: [bool; 2],
    ldcr_bottle_squib_fault: [bool; 2],
    lavatory: LavatoryProtection,
    lav_bin_temp_c: f64,
    lav_bin_burn_kg_s: f64,
    lav_bin_fire_out: bool,

    wing_anti_ice: Vec<BleedAntiIceSurface>,
    wing_anti_ice_out: [super::anti_ice::BleedAntiIceOutputs; 2],
    nacelle_anti_ice: Vec<BleedAntiIceSurface>,
    nacelle_anti_ice_out: [super::anti_ice::BleedAntiIceOutputs; 4],
    probes: Vec<ProbeHeater>,
    probe_out: [super::anti_ice::ProbeHeaterOutputs; 8],
    windows: Vec<WindowHeat>,
    window_out: [super::anti_ice::WindowHeatOutputs; 2],
    rain: Vec<RainRemoval>,
    rain_film_kg_m2: [f64; 2],

    wing_ice: Vec<IcingSurface>,
    wing_ice_out: [IcingOutputs; 2],
    nacelle_ice: Vec<IcingSurface>,
    nacelle_ice_out: [IcingOutputs; 4],

    names: VarNames,
}

struct VarNames {
    fire_detected: [String; 9],
    zone_temp_c: [String; 9],
    zone_burning: [String; 9],
    zone_agent: [String; 9],
    loop_a_fault: [String; 9],
    loop_b_fault: [String; 9],
    loop_a_fire: [String; 9],
    loop_b_fire: [String; 9],
    loop_disagree: [String; 9],
    bottle_low: [[String; 2]; 4],
    squib: [[String; 2]; 4],
    probe_fault: [String; 8],
    probe_temp_c: [String; 8],
    window_fault: [String; 2],
    window_temp_c: [String; 2],
    window_hot_spot_c: [String; 2],
    window_film: [String; 2],
    wing_valve_open: [String; 2],
    wing_overheat: [String; 2],
    wing_surface_c: [String; 2],
    wing_ice_thickness: [String; 2],
    wing_cl_loss: [String; 2],
    nacelle_valve_open: [String; 4],
    nacelle_overheat: [String; 4],
    nacelle_surface_c: [String; 4],
    nacelle_ice_thickness: [String; 4],
    cargo_smoke_detected: [String; 2],
    cargo_smoke_density: [String; 2],
    cargo_bottle_low_pressure: [String; 2],
    read_brake_stack_temp_c: [String; N_BRAKED_WHEELS],
    read_brake_fire: [String; N_BRAKED_WHEELS],
    read_engine_nacelle_leak_kg_s: [String; 4],
    read_engine_hot_section_c: [String; 4],
}

impl VarNames {
    fn new() -> Self {
        let zone_name = |z: usize| {
            match z {
                0..=3 => format!("ENG:{}", z + 1),
                _ => ZONE_KEYS[z].to_string(),
            }
        };
        Self {
            fire_detected: std::array::from_fn(|z| format!("DEEP_FIRE_DETECTED_{}", zone_name(z))),
            zone_temp_c: std::array::from_fn(|z| format!("FIRE_ZONE_{}_TEMPERATURE_C", ZONE_KEYS[z])),
            zone_burning: std::array::from_fn(|z| format!("FIRE_ZONE_{}_BURNING", ZONE_KEYS[z])),
            zone_agent: std::array::from_fn(|z| format!("FIRE_ZONE_{}_AGENT_FRACTION", ZONE_KEYS[z])),
            loop_a_fault: std::array::from_fn(|z| format!("FIRE_LOOP_A_{}_FAULT", ZONE_KEYS[z])),
            loop_b_fault: std::array::from_fn(|z| format!("FIRE_LOOP_B_{}_FAULT", ZONE_KEYS[z])),
            loop_a_fire: std::array::from_fn(|z| format!("FIRE_LOOP_A_{}_FIRE", ZONE_KEYS[z])),
            loop_b_fire: std::array::from_fn(|z| format!("FIRE_LOOP_B_{}_FIRE", ZONE_KEYS[z])),
            loop_disagree: std::array::from_fn(|z| format!("FIRE_LOOP_{}_DISAGREE", ZONE_KEYS[z])),
            bottle_low: std::array::from_fn(|e| std::array::from_fn(|b| format!("FIRE_BOTTLE_ENG{}_{}_LOW_PRESSURE", e + 1, b + 1))),
            squib: std::array::from_fn(|e| std::array::from_fn(|b| format!("DEEP_FIRE_SQUIB_{}_ENG_{}_IS_DISCHARGED", b + 1, e + 1))),
            probe_fault: std::array::from_fn(|p| format!("PROBE_HEAT_{}_FAULT", PROBE_KEYS[p])),
            probe_temp_c: std::array::from_fn(|p| format!("PROBE_HEAT_{}_TEMPERATURE_C", PROBE_KEYS[p])),
            window_fault: std::array::from_fn(|w| format!("WINDOW_HEAT_{}_FAULT", WINDOW_KEYS[w])),
            window_temp_c: std::array::from_fn(|w| format!("WINDOW_HEAT_{}_TEMPERATURE_C", WINDOW_KEYS[w])),
            window_hot_spot_c: std::array::from_fn(|w| format!("WINDOW_HEAT_{}_HOT_SPOT_C", WINDOW_KEYS[w])),
            window_film: std::array::from_fn(|w| format!("WINDSHIELD_{}_WATER_FILM_KG_M2", WINDOW_KEYS[w])),
            wing_valve_open: std::array::from_fn(|s| format!("ANTI_ICE_WING_{}_VALVE_OPEN", WINDOW_KEYS[s])),
            wing_overheat: std::array::from_fn(|s| format!("ANTI_ICE_WING_{}_OVERHEAT", WINDOW_KEYS[s])),
            wing_surface_c: std::array::from_fn(|s| format!("ANTI_ICE_WING_{}_SURFACE_C", WINDOW_KEYS[s])),
            wing_ice_thickness: std::array::from_fn(|s| format!("ICE_WING_{}_THICKNESS_M", WINDOW_KEYS[s])),
            wing_cl_loss: std::array::from_fn(|s| format!("ICE_WING_{}_CL_MAX_LOSS", WINDOW_KEYS[s])),
            nacelle_valve_open: std::array::from_fn(|e| format!("ANTI_ICE_NACELLE{}_VALVE_OPEN", e + 1)),
            nacelle_overheat: std::array::from_fn(|e| format!("ANTI_ICE_NACELLE{}_OVERHEAT", e + 1)),
            nacelle_surface_c: std::array::from_fn(|e| format!("ANTI_ICE_NACELLE{}_SURFACE_C", e + 1)),
            nacelle_ice_thickness: std::array::from_fn(|e| format!("ICE_NACELLE{}_THICKNESS_M", e + 1)),
            cargo_smoke_detected: std::array::from_fn(|b| format!("CARGO_{}_SMOKE_DETECTED", ["FWD", "AFT"][b])),
            cargo_smoke_density: std::array::from_fn(|b| format!("CARGO_{}_SMOKE_DENSITY_KG_M3", ["FWD", "AFT"][b])),
            cargo_bottle_low_pressure: std::array::from_fn(|b| format!("FIRE_BOTTLE_CARGO_{}_LOW_PRESSURE", ["FWD", "AFT"][b])),
            read_brake_stack_temp_c: std::array::from_fn(|w| format!("BRAKE_STACK_TEMP_C:{}", w + 1)),
            read_brake_fire: std::array::from_fn(|w| format!("BRAKE_FIRE:{}", w + 1)),
            read_engine_nacelle_leak_kg_s: std::array::from_fn(|e| format!("A32NX_ENG_{}_NACELLE_FUEL_LEAK_KG_S", e + 1)),
            read_engine_hot_section_c: std::array::from_fn(|e| format!("A32NX_ENG_{}_HOT_SECTION_SOAK_C", e + 1)),
        }
    }
}

impl Default for FireIceLive {
    fn default() -> Self {
        Self::new()
    }
}

impl FireIceLive {
    const NACELLE_BOTTLE_CHARGE_KG: f64 = 5.0;
    const BOTTLE_VOLUME_M3: f64 = 0.005;
    const CARGO_BOTTLE_CHARGE_KG: f64 = 30.0;
    const CARGO_BOTTLE_VOLUME_M3: f64 = 0.03;
    const SMOKE_DETECTOR_PATH_M: f64 = 1.0;
    const LAVATORY_VOLUME_M3: f64 = 2.0;
    const LAV_BIN_FIRE_BURN_KG_S: f64 = 0.002;
    const LAV_BIN_FIRE_RISE_C: f64 = 200.0;
    const LAV_BIN_TIME_CONSTANT_S: f64 = 60.0;

    pub fn new() -> Self {
        let start_c = 15.0;
        Self {
            detectors: (0..9).map(|_| ZoneDetector::new(LoopLogic::And)).collect(),
            combustion: (0..9)
                .map(|z| ZoneCombustion::new(ZONE_FLUID[z], start_c, ZONE_THERMAL_MASS_J_K[z], ZONE_VENTILATION_KG_S[z] * super::util::CP_AIR))
                .collect(),
            concentration: (0..9).map(|z| ZoneConcentration::new(ZONE_VOLUME_M3[z])).collect(),
            zones: [ZoneReport::default(); 9],

            engine_bottles: (0..4).map(|_| [Bottle::new(Self::NACELLE_BOTTLE_CHARGE_KG, Self::BOTTLE_VOLUME_M3), Bottle::new(Self::NACELLE_BOTTLE_CHARGE_KG, Self::BOTTLE_VOLUME_M3)]).collect(),
            engine_bottle_low: [[false; 2]; 4],
            engine_squib_discharged: [[false; 2]; 4],
            engine_bottle_squib_blocked: [[false; 2]; 4],
            engine_bottle_empty: [[false; 2]; 4],
            engine_bottle_charge_loss: [[0.0; 2]; 4],
            apu_bottle: Bottle::new(Self::NACELLE_BOTTLE_CHARGE_KG, Self::BOTTLE_VOLUME_M3),
            apu_squib_discharged: false,
            apu_bottle_squib_blocked: false,
            apu_bottle_empty: false,
            apu_bottle_charge_loss: 0.0,

            cargo_suppression: (0..2).map(|_| CargoSuppressionSystem::new(Self::CARGO_BOTTLE_CHARGE_KG, Self::CARGO_BOTTLE_VOLUME_M3)).collect(),
            cargo_smoke: (0..2).map(|b| OpticalSmokeDetector::new(Self::SMOKE_DETECTOR_PATH_M, ZONE_VOLUME_M3[6 + b])).collect(),
            cargo_smoke_alarm: [false; 2],
            cargo_smoke_total_kg_m3: [0.0; 2],
            cargo_distribution_fault: [false; 2],
            cargo_knockdown_squib_fault: [false; 2],
            cargo_extended_squib_fault: [false; 2],
            ldcr_bottle_squib_fault: [false; 2],
            lavatory: LavatoryProtection::new(Self::LAVATORY_VOLUME_M3),
            lav_bin_temp_c: 20.0,
            lav_bin_burn_kg_s: 0.0,
            lav_bin_fire_out: false,

            wing_anti_ice: (0..2).map(|_| BleedAntiIceSurface::new(WING_ANTI_ICE, start_c)).collect(),
            wing_anti_ice_out: [Default::default(); 2],
            nacelle_anti_ice: (0..4).map(|_| BleedAntiIceSurface::new(NACELLE_ANTI_ICE, start_c)).collect(),
            nacelle_anti_ice_out: [Default::default(); 4],
            probes: (0..8).map(|_| ProbeHeater::new(start_c)).collect(),
            probe_out: [Default::default(); 8],
            windows: (0..2).map(|_| WindowHeat::new(start_c)).collect(),
            window_out: [Default::default(); 2],
            rain: (0..2).map(|_| RainRemoval::new()).collect(),
            rain_film_kg_m2: [0.0; 2],

            wing_ice: (0..2).map(|_| IcingSurface::new(WING_LEADING_EDGE)).collect(),
            wing_ice_out: [Default::default(); 2],
            nacelle_ice: (0..4).map(|_| IcingSurface::new(NACELLE_INLET)).collect(),
            nacelle_ice_out: [Default::default(); 4],

            names: VarNames::new(),
        }
    }

    fn loop_faults(faults: &Faults, zone: usize) -> (LoopFaults, LoopFaults) {
        let base = zone as u16 * 4;
        (
            LoopFaults { open_circuit: faults.get(fire(base + 1)), short_circuit: faults.get(fire(base + 2)) },
            LoopFaults { open_circuit: faults.get(fire(base + 3)), short_circuit: faults.get(fire(base + 4)) },
        )
    }

    fn ignition_source(&self, truth: &Truth, zone: usize) -> bool {
        match zone {
            0..=3 => {
                truth.engine_running[zone]
                    || truth.published.get_or(&self.names.read_engine_hot_section_c[zone], f64::NEG_INFINITY) >= ZONE_FLUID[zone].autoignition_c
            }
            4 => truth.apu_running,
            5 => {
                let autoignition_c = ZONE_FLUID[5].autoignition_c;
                (0..N_BRAKED_WHEELS).any(|w| {
                    truth.published.get_or(&self.names.read_brake_fire[w], 0.0) >= 0.5
                        || truth.published.get_or(&self.names.read_brake_stack_temp_c[w], f64::NEG_INFINITY) >= autoignition_c
                })
            }
            6..=8 => truth.published.get_or(CONTENT_FIRE_HEAT_VARS[zone - 6], 0.0) > 0.0,
            _ => false,
        }
    }

    fn step_fire(&mut self, truth: &Truth, faults: &Faults, cond: &Conditions) {
        let dt = truth.dt_s;
        let ambient_c = cond.recovery_c;

        let before_c: [f64; 9] = std::array::from_fn(|z| self.combustion[z].temp_c());
        let mut extra_w = [0.0_f64; 9];
        for (a, b, ua) in ZONE_LINKS {
            let q = super::combustion::conductive_link_w(ua, before_c[a], before_c[b]);
            extra_w[a] -= q;
            extra_w[b] += q;
        }

        let agent_in = self.step_bottles(truth, faults, cond);
        for z in 0..9 {
            let vent_m3_s = ZONE_VENTILATION_KG_S[z] / AIR_DENSITY_KG_M3;
            self.concentration[z].step(agent_in[z], cond.static_air_c, cond.ambient_pressure_pa, vent_m3_s, dt);
        }

        for z in 0..9 {
            let debris_leak_kg_s = if z < 4 { truth.published.get_or(&self.names.read_engine_nacelle_leak_kg_s[z], 0.0).max(0.0) } else { 0.0 };
            let supply = ZoneSupply {
                fuel_available_kg_s: faults.get(fire(100 + z as u16)) * ZONE_MAX_LEAK_KG_S[z] + debris_leak_kg_s,
                air_available_kg_s: ZONE_VENTILATION_KG_S[z],
                ignition_source: self.ignition_source(truth, z),
                suppression_fraction: self.concentration[z].suppression_fraction(),
            };
            let state = self.combustion[z].step(&supply, ambient_c, extra_w[z], dt);
            let (fa, fb) = Self::loop_faults(faults, z);
            let status = self.detectors[z].evaluate(state.temp_c, state.temp_c, fa, fb);
            self.zones[z] = ZoneReport {
                temp_c: state.temp_c,
                burning: state.burning,
                burn_rate_kg_s: state.burn_rate_kg_s,
                fire: status.fire,
                loop_a_fault: status.loop_a_fault,
                loop_b_fault: status.loop_b_fault,
                loop_a_signal: status.loop_a_signal,
                loop_b_signal: status.loop_b_signal,
                agent_fraction: self.concentration[z].suppression_fraction(),
            };
        }

        for b in 0..2 {
            let zone = 6 + b;
            let faults_det = SmokeDetectorFaults { lens_obscured: faults.get(fire(227 + b as u16)) };
            let vent_m3_s = ZONE_VENTILATION_KG_S[zone] / AIR_DENSITY_KG_M3;
            let hold_smoke_kg_m3 = truth.published.get_or(HOLD_SMOKE_VARS[b], 0.0) * AIR_DENSITY_KG_M3;
            self.cargo_smoke_alarm[b] = self.cargo_smoke[b].step_with_ambient(self.zones[zone].burn_rate_kg_s, vent_m3_s, hold_smoke_kg_m3, &faults_det, dt);
            self.cargo_smoke_total_kg_m3[b] = self.cargo_smoke[b].smoke_density_kg_m3() + hold_smoke_kg_m3;
        }

        let lav_smoke = SmokeDetectorFaults::default();
        let lav_link = LavatoryFaults { link_degraded: faults.get(fire(229)) };
        let bin_fire = faults.get(fire(232)).clamp(0.0, 1.0);
        if bin_fire <= 0.0 {
            self.lav_bin_fire_out = false;
        }
        let burning = bin_fire > 0.0 && !self.lav_bin_fire_out;
        let cabin_c = truth.cabin_temp_k - 273.15;
        self.lav_bin_burn_kg_s = if burning { Self::LAV_BIN_FIRE_BURN_KG_S * bin_fire } else { 0.0 };
        let target_c = cabin_c + if burning { Self::LAV_BIN_FIRE_RISE_C * bin_fire } else { 0.0 };
        self.lav_bin_temp_c += (target_c - self.lav_bin_temp_c) * (dt / Self::LAV_BIN_TIME_CONSTANT_S).min(1.0);
        self.lavatory.step(self.lav_bin_temp_c, self.lav_bin_burn_kg_s, 0.01, &lav_smoke, &lav_link, dt);
        if burning && self.lavatory.is_discharged() {
            self.lav_bin_fire_out = true;
        }
    }

    fn step_bottles(&mut self, truth: &Truth, faults: &Faults, cond: &Conditions) -> [f64; 9] {
        let dt = truth.dt_s;
        let ambient_c = cond.static_air_c;
        let zone_pa = cond.ambient_pressure_pa;
        let mut agent = [0.0_f64; 9];

        self.ldcr_bottle_squib_fault = [fire(230), fire(231)].map(|id| faults.get(id) > 0.0);

        for e in 0..4usize {
            for b in 0..2usize {
                let leak_id = fire(201 + (e as u16) * 4 + (b as u16) * 2);
                let bottle_faults = BottleFaults { leak: faults.get(leak_id), squib_failure: faults.get(leak_id + 1) };
                let fire_command = truth.controls.fire_pb_released[e] && truth.controls.fire_agent_pb_pressed[e][b];
                let delivered = self.engine_bottles[e][b].step(ambient_c, fire_command, zone_pa, &bottle_faults, dt);
                agent[e] += delivered;
                self.engine_bottle_low[e][b] = self.engine_bottles[e][b].is_low_pressure();
                self.engine_squib_discharged[e][b] = self.engine_bottles[e][b].is_discharged();
                self.engine_bottle_squib_blocked[e][b] = clamp01(bottle_faults.squib_failure) >= 1.0;
                self.engine_bottle_empty[e][b] = self.engine_bottles[e][b].agent_mass_kg() <= 0.0;
                self.engine_bottle_charge_loss[e][b] = self.engine_bottles[e][b].charge_loss_fraction();
            }
        }

        let apu_faults = BottleFaults { leak: faults.get(fire(217)), squib_failure: faults.get(fire(218)) };
        let apu_command = (self.zones[4].fire && truth.on_ground) || (truth.controls.fire_pb_apu_released && truth.controls.fire_agent_pb_apu_pressed);
        agent[4] += self.apu_bottle.step(ambient_c, apu_command, zone_pa, &apu_faults, dt);
        self.apu_squib_discharged = self.apu_bottle.is_discharged();
        self.apu_bottle_squib_blocked = clamp01(apu_faults.squib_failure) >= 1.0;
        self.apu_bottle_empty = self.apu_bottle.agent_mass_kg() <= 0.0;
        self.apu_bottle_charge_loss = self.apu_bottle.charge_loss_fraction();

        for b in 0..2usize {
            let zone = 6 + b;
            let base = fire(219 + (b as u16) * 4);
            let cargo_faults = CargoSuppressionFaults {
                leak: faults.get(base),
                knockdown_squib_fault: faults.get(base + 1),
                extended_squib_fault: faults.get(base + 2),
                distribution_fault: faults.get(base + 3),
            };
            self.cargo_knockdown_squib_fault[b] = cargo_faults.knockdown_squib_fault > 0.0;
            self.cargo_extended_squib_fault[b] = cargo_faults.extended_squib_fault > 0.0;
            self.cargo_distribution_fault[b] = cargo_faults.distribution_fault > 0.0;
            let delivered = {
                let (system, concentration) = (&mut self.cargo_suppression[b], &self.concentration[zone]);
                system.step(ambient_c, truth.controls.cargo_agent_pb_pressed[b], zone_pa, concentration, &cargo_faults, dt)
            };
            agent[zone] += delivered;
        }

        agent
    }

    fn step_ice(&mut self, truth: &Truth, faults: &Faults, cond: &Conditions) {
        let dt = truth.dt_s;

        let wing_command = on(truth.controls.wing_anti_ice_selected);
        let wing_beta = cond.beta0(WING_LEADING_EDGE.characteristic_length_m);
        for s in 0..2usize {
            let base = 1 + (s as u16) * 3;
            let f = BleedAntiIceFaults {
                valve_stuck_closed: faults.get(ice(base)),
                valve_stuck_open: faults.get(ice(base + 1)),
                duct_leak: faults.get(ice(base + 2)),
            };
            let out = self.wing_anti_ice[s].step(wing_command, cond.static_air_c, cond.recovery_c, cond.lwc_kg_m3, wing_beta, cond.tas_m_s, cond.ambient_pressure_pa, &f, dt);
            self.wing_anti_ice_out[s] = out;

            let natural_ff = self.wing_ice_out[s].freezing_fraction;
            let removal = (self.wing_ice_out[s].impingement_kg_m2_s * (natural_ff - out.freezing_fraction)).max(0.0);
            self.wing_ice_out[s] = self.wing_ice[s].step(&cond.icing_environment(), removal, dt);
        }

        let nacelle_beta = cond.beta0(NACELLE_INLET.characteristic_length_m);
        for e in 0..4usize {
            let base = 7 + (e as u16) * 3;
            let f = BleedAntiIceFaults {
                valve_stuck_closed: faults.get(ice(base)),
                valve_stuck_open: faults.get(ice(base + 1)),
                duct_leak: faults.get(ice(base + 2)),
            };
            let nacelle_command = on(truth.controls.nacelle_anti_ice_selected[e]);
            let out = self.nacelle_anti_ice[e].step(nacelle_command, cond.static_air_c, cond.recovery_c, cond.lwc_kg_m3, nacelle_beta, cond.tas_m_s, cond.ambient_pressure_pa, &f, dt);
            self.nacelle_anti_ice_out[e] = out;
            let natural_ff = self.nacelle_ice_out[e].freezing_fraction;
            let removal = (self.nacelle_ice_out[e].impingement_kg_m2_s * (natural_ff - out.freezing_fraction)).max(0.0);
            self.nacelle_ice_out[e] = self.nacelle_ice[e].step(&cond.icing_environment(), removal, dt);
        }

        let probe_beta = cond.beta0(super::icing::PROBE.characteristic_length_m);
        for p in 0..8usize {
            let base = 19 + (p as u16) * 3;
            let f = ProbeHeaterFaults {
                heater_open_circuit: faults.get(ice(base)),
                controller_fault: faults.get(ice(base + 1)),
                sensor_fault: faults.get(ice(base + 2)),
            };
            self.probe_out[p] = self.probes[p].step(cond.static_air_c, cond.recovery_c, cond.lwc_kg_m3, probe_beta, cond.tas_m_s, cond.ambient_pressure_pa, &f, dt);
        }

        let window_beta = cond.beta0(WINDSHIELD.characteristic_length_m);
        let rain_catch_kg_m2_s = HEAVY_RAIN_LWC_KG_M3 * truth.environment.precipitation_on_aircraft_ratio.clamp(0.0, 1.0) * cond.tas_m_s * window_beta;
        for w in 0..2usize {
            let base = 43 + (w as u16) * 3;
            let f = WindowHeatFaults {
                film_defect: faults.get(ice(base)),
                controller_fault: faults.get(ice(base + 1)),
                sensor_fault: faults.get(ice(base + 2)),
            };
            self.window_out[w] = self.windows[w].step(cond.static_air_c, cond.recovery_c, cond.lwc_kg_m3, window_beta, cond.tas_m_s, cond.ambient_pressure_pa, &f, dt);

            let rain_faults = RainRemovalFaults { system_fault: faults.get(ice(49 + w as u16)) };
            let jet_velocity_m_s = if truth.controls.rain_removal_selected[w] { RAIN_REMOVAL_JET_VELOCITY_M_S } else { 0.0 };
            self.rain_film_kg_m2[w] = self.rain[w].step(rain_catch_kg_m2_s, jet_velocity_m_s, 0.0, &rain_faults, dt);
        }
    }

    fn probe_fault(out: &super::anti_ice::ProbeHeaterOutputs) -> bool {
        out.surface_c < 0.0 && out.power_w <= 0.0
    }

    fn window_fault(out: &super::anti_ice::WindowHeatOutputs) -> bool {
        out.overheat_tripped || out.delaminated || out.cracked || (out.surface_c < WINDOW_TARGET_C - 20.0 && out.power_w <= 0.0)
    }
}

impl crate::deep::live::Area for FireIceLive {
    fn name(&self) -> &'static str {
        "fire_ice"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        let cond = Conditions::from(truth);
        self.step_fire(truth, faults, &cond);
        self.step_ice(truth, faults, &cond);
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        let n = &self.names;
        for z in 0..9 {
            out(&n.fire_detected[z], on(self.zones[z].fire));
            out(&n.zone_temp_c[z], self.zones[z].temp_c);
            out(&n.zone_burning[z], on(self.zones[z].burning));
            out(&n.zone_agent[z], self.zones[z].agent_fraction);
            out(&n.loop_a_fault[z], on(self.zones[z].loop_a_fault));
            out(&n.loop_b_fault[z], on(self.zones[z].loop_b_fault));
            out(&n.loop_a_fire[z], on(self.zones[z].loop_a_signal));
            out(&n.loop_b_fire[z], on(self.zones[z].loop_b_signal));
            out(&n.loop_disagree[z], on(self.zones[z].loop_a_signal != self.zones[z].loop_b_signal));
        }
        for e in 0..4 {
            for b in 0..2 {
                out(&n.bottle_low[e][b], on(self.engine_bottle_low[e][b]));
                out(&n.squib[e][b], on(self.engine_squib_discharged[e][b]));
                out(&format!("DEEP_FIRE_BOTTLE_{}_ENG_{}_SQUIB_BLOCKED", b + 1, e + 1), on(self.engine_bottle_squib_blocked[e][b]));
                out(&format!("DEEP_FIRE_BOTTLE_{}_ENG_{}_EMPTY", b + 1, e + 1), on(self.engine_bottle_empty[e][b]));
                out(&format!("DEEP_FIRE_BOTTLE_{}_ENG_{}_CHARGE_LOSS_FRACTION", b + 1, e + 1), self.engine_bottle_charge_loss[e][b]);
            }
        }
        out("DEEP_FIRE_SQUIB_1_APU_1_IS_DISCHARGED", on(self.apu_squib_discharged));
        out("DEEP_FIRE_BOTTLE_1_APU_1_CHARGE_LOSS_FRACTION", self.apu_bottle_charge_loss);
        out("FIRE_BOTTLE_APU_LOW_PRESSURE", on(self.apu_bottle.is_low_pressure()));
        out("DEEP_FIRE_BOTTLE_1_APU_1_SQUIB_BLOCKED", on(self.apu_bottle_squib_blocked));
        out("DEEP_FIRE_BOTTLE_1_APU_1_EMPTY", on(self.apu_bottle_empty));
        out("LAVATORY_EXTINGUISHER_DISCHARGED", on(self.lavatory.is_discharged()));
        out("LAVATORY_1_BIN_FIRE", on(self.lav_bin_burn_kg_s > 0.0));
        out("LAVATORY_1_BIN_TEMP_C", self.lav_bin_temp_c);
        out("LAVATORY_1_SMOKE_DENSITY_KG_M3", self.lavatory.smoke_detector.smoke_density_kg_m3());

        for b in 0..2 {
            out(&n.cargo_smoke_detected[b], on(self.cargo_smoke_alarm[b]));
            out(&n.cargo_smoke_density[b], self.cargo_smoke_total_kg_m3[b]);
            out(&format!("CARGO_{}_AGENT_METERING", ["FWD", "AFT"][b]), on(self.cargo_suppression[b].is_metering()));
            out(&n.cargo_bottle_low_pressure[b], on(self.cargo_suppression[b].bottle.is_low_pressure()));
            out(&format!("FIRE_CARGO_{}_DISTRIBUTION_FAULT", ["FWD", "AFT"][b]), on(self.cargo_distribution_fault[b]));
            out(&format!("FIRE_CARGO_{}_KNOCKDOWN_SQUIB_FAULT", ["FWD", "AFT"][b]), on(self.cargo_knockdown_squib_fault[b]));
            out(&format!("FIRE_CARGO_{}_EXTENDED_SQUIB_FAULT", ["FWD", "AFT"][b]), on(self.cargo_extended_squib_fault[b]));
        }
        for (i, bottle) in ["1", "2"].iter().enumerate() {
            out(&format!("FIRE_LDCR_BTL_{bottle}_SQUIB_FAULT"), on(self.ldcr_bottle_squib_fault[i]));
        }

        for s in 0..2 {
            let o = &self.wing_anti_ice_out[s];
            out(&n.wing_valve_open[s], on(o.bleed_delivered_kg_s > 0.0));
            out(&n.wing_overheat[s], on(o.overheat));
            out(&n.wing_surface_c[s], o.surface_c);
            out(&n.wing_ice_thickness[s], self.wing_ice_out[s].ice_thickness_m);
            out(&n.wing_cl_loss[s], self.wing_ice_out[s].cl_max_loss_fraction);
        }
        for e in 0..4 {
            let o = &self.nacelle_anti_ice_out[e];
            out(&n.nacelle_valve_open[e], on(o.bleed_delivered_kg_s > 0.0));
            out(&n.nacelle_overheat[e], on(o.overheat));
            out(&n.nacelle_surface_c[e], o.surface_c);
            out(&n.nacelle_ice_thickness[e], self.nacelle_ice_out[e].ice_thickness_m);
        }
        for p in 0..8 {
            out(&n.probe_fault[p], on(Self::probe_fault(&self.probe_out[p])));
            out(&n.probe_temp_c[p], self.probe_out[p].surface_c);
        }
        for w in 0..2 {
            out(&n.window_fault[w], on(Self::window_fault(&self.window_out[w])));
            out(&n.window_temp_c[w], self.window_out[w].surface_c);
            out(&n.window_hot_spot_c[w], self.window_out[w].hot_spot_c);
            out(&n.window_film[w], self.rain_film_kg_m2[w]);
        }

        self.derived_failures(&mut |d| {
            out(&format!("DEEP_DERIVED_FBW_FAILURE_{}", d.fbw_id), d.magnitude);
        });
    }

    fn derived_failures(&self, out: &mut dyn FnMut(DerivedFailure)) {
        const SET_ON_FIRE_ID: [u64; 6] = [26_001, 26_002, 26_003, 26_004, 26_005, 26_006];
        const LOOP_A_ID: [u64; 6] = [26_007, 26_009, 26_011, 26_013, 26_015, 26_017];
        const LOOP_B_ID: [u64; 6] = [26_008, 26_010, 26_012, 26_014, 26_016, 26_018];
        const COMPONENT: [&str; 6] = [
            "26_fire.zone.engine1",
            "26_fire.zone.engine2",
            "26_fire.zone.engine3",
            "26_fire.zone.engine4",
            "26_fire.zone.apu",
            "26_fire.zone.mlg",
        ];
        let level = |b: bool| if b { 1.0 } else { 0.0 };
        for z in 0..6 {
            out(DerivedFailure {
                fbw_id: SET_ON_FIRE_ID[z],
                magnitude: level(self.zones[z].fire),
                deep_component: COMPONENT[z],
                reason: if self.zones[z].fire { "zone fire-detection verdict is genuinely on fire" } else { REASON_HEALTHY },
            });
            out(DerivedFailure {
                fbw_id: LOOP_A_ID[z],
                magnitude: level(self.zones[z].loop_a_fault),
                deep_component: COMPONENT[z],
                reason: if self.zones[z].loop_a_fault { "loop A open/short/unpowered" } else { REASON_HEALTHY },
            });
            out(DerivedFailure {
                fbw_id: LOOP_B_ID[z],
                magnitude: level(self.zones[z].loop_b_fault),
                deep_component: COMPONENT[z],
                reason: if self.zones[z].loop_b_fault { "loop B open/short/unpowered" } else { REASON_HEALTHY },
            });
        }
    }
}

const REASON_HEALTHY: &str = "healthy";

pub fn live_system() -> Box<dyn crate::deep::live::Area> {
    Box::new(FireIceLive::new())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    fn ground_running_truth() -> Truth {
        Truth {
            dt_s: 1.0,
            engine_running: [true; 4],
            engine_n1_frac: [0.25; 4],
            apu_running: true,
            on_ground: true,
            ac_bus_volts: [115.0; 4],
            ..Truth::default()
        }
    }

    fn icing_truth() -> Truth {
        Truth {
            dt_s: 1.0,
            environment: crate::deep::integration::weather_truth::EnvironmentTruth {
                sat_c: -10.0,
                leading_edge_c: -5.0,
                ambient_pressure_pa: 80_000.0,
                tas_ms: 100.0,
                precipitation_on_aircraft_ratio: 0.0,
                weather: None,
            },
            altitude_ft: 6000.0,
            on_ground: false,
            ..Truth::default()
        }
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
        let mut required: Vec<String> = vec!["DEEP_FIRE_DETECTED_APU".into(), "DEEP_FIRE_DETECTED_MLG".into(), "DEEP_FIRE_SQUIB_1_APU_1_IS_DISCHARGED".into(), "WINDOW_HEAT_L_FAULT".into(), "WINDOW_HEAT_R_FAULT".into(), "ANTI_ICE_WING_L_OVERHEAT".into(), "ANTI_ICE_WING_R_OVERHEAT".into(), "ANTI_ICE_WING_L_VALVE_OPEN".into(), "ANTI_ICE_WING_R_VALVE_OPEN".into(), "CARGO_FWD_SMOKE_DETECTED".into(), "CARGO_AFT_SMOKE_DETECTED".into()];
        for e in 1..=4 {
            required.push(format!("DEEP_FIRE_DETECTED_ENG:{e}"));
            required.push(format!("ANTI_ICE_NACELLE{e}_OVERHEAT"));
            required.push(format!("ANTI_ICE_NACELLE{e}_VALVE_OPEN"));
            required.push(format!("FIRE_BOTTLE_ENG{e}_1_LOW_PRESSURE"));
            required.push(format!("FIRE_BOTTLE_ENG{e}_2_LOW_PRESSURE"));
            required.push(format!("DEEP_FIRE_SQUIB_1_ENG_{e}_IS_DISCHARGED"));
            required.push(format!("DEEP_FIRE_SQUIB_2_ENG_{e}_IS_DISCHARGED"));
        }
        for zone in ZONE_KEYS {
            required.push(format!("FIRE_LOOP_A_{zone}_FAULT"));
            required.push(format!("FIRE_LOOP_B_{zone}_FAULT"));
            required.push(format!("FIRE_LOOP_{zone}_DISAGREE"));
        }
        for probe in ["PITOT1", "PITOT2", "PITOT3"] {
            required.push(format!("PROBE_HEAT_{probe}_FAULT"));
        }
        for name in required {
            assert!(map.contains_key(&name), "{name} is read by an ECAM trigger but never published");
        }
    }

    #[test]
    fn a_fuel_leak_in_a_running_engines_nacelle_ignites_and_is_declared_as_a_fire() {
        let truth = ground_running_truth();
        let mut area = live_system();
        let armed = Faults::from_pairs([(fire(100), 1.0)]);
        run(area.as_mut(), &truth, &armed, 300);
        let map = published(area.as_ref());
        assert_eq!(map["FIRE_ZONE_ENG1_BURNING"], 1.0, "a leak with air and a hot turbine case must light");
        assert!(map["FIRE_ZONE_ENG1_TEMPERATURE_C"] > 200.0, "the zone must heat past the loops' trip temperature, got {}", map["FIRE_ZONE_ENG1_TEMPERATURE_C"]);
        assert_eq!(map["DEEP_FIRE_DETECTED_ENG:1"], 1.0, "and both loops must declare it");
        assert_eq!(map["DEEP_FIRE_DETECTED_ENG:2"], 0.0, "engine 2 has no leak");
    }

    #[test]
    fn the_same_leak_without_a_running_engine_never_ignites() {
        let mut truth = ground_running_truth();
        truth.engine_running = [false; 4];
        truth.apu_running = false;
        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::from_pairs([(fire(100), 1.0)]), 300);
        let map = published(area.as_ref());
        assert_eq!(map["FIRE_ZONE_ENG1_BURNING"], 0.0, "with no ignition source the fuel just pools");
        assert_eq!(map["DEEP_FIRE_DETECTED_ENG:1"], 0.0);
    }

    #[test]
    fn an_apu_fire_on_the_ground_fires_its_own_bottle_without_any_crew_action() {
        let truth = ground_running_truth();
        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::from_pairs([(fire(104), 1.0)]), 600);
        let map = published(area.as_ref());
        assert_eq!(map["DEEP_FIRE_DETECTED_APU"], 1.0);
        assert_eq!(map["DEEP_FIRE_SQUIB_1_APU_1_IS_DISCHARGED"], 1.0, "the APU's ground discharge is automatic");
        assert!(map["FIRE_ZONE_APU_AGENT_FRACTION"] > 0.0, "agent must actually reach the bay");
    }

    #[test]
    fn a_shorted_loop_alone_is_rejected_but_a_shorted_loop_beside_a_failed_one_declares_a_false_fire() {
        let mut truth = ground_running_truth();
        truth.engine_running = [false; 4];
        truth.apu_running = false;
        let mut only_short = live_system();
        run(only_short.as_mut(), &truth, &Faults::from_pairs([(fire(2), 1.0)]), 10);
        assert_eq!(published(only_short.as_ref())["DEEP_FIRE_DETECTED_ENG:1"], 0.0, "AND logic must reject a single disagreeing loop");

        let mut short_and_open = live_system();
        run(short_and_open.as_mut(), &truth, &Faults::from_pairs([(fire(2), 1.0), (fire(3), 1.0)]), 10);
        let map = published(short_and_open.as_ref());
        assert_eq!(map["FIRE_LOOP_B_ENG1_FAULT"], 1.0, "loop B open must read as a loop fault");
        assert_eq!(map["FIRE_LOOP_A_ENG1_FAULT"], 0.0, "a short is not distinguishable from heat, so it is not a fault");
        assert_eq!(map["DEEP_FIRE_DETECTED_ENG:1"], 1.0, "with B faulted the unit trusts A alone, and A says fire");
    }

    #[test]
    fn a_lone_shorted_engine_fire_loop_annunciates_disagreement_while_the_zone_stays_quiet() {
        let mut truth = ground_running_truth();
        truth.engine_running = [false; 4];
        truth.apu_running = false;
        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::from_pairs([(fire(2), 1.0)]), 10);
        let map = published(area.as_ref());
        assert_eq!(map["DEEP_FIRE_DETECTED_ENG:1"], 0.0, "AND logic must still withhold the zone-level warning");
        assert_eq!(map["FIRE_LOOP_A_ENG1_FAULT"], 0.0, "a short is not a loop fault");
        assert_eq!(map["FIRE_LOOP_A_ENG1_FIRE"], 1.0, "the shorted loop's own raw reading must say fire");
        assert_eq!(map["FIRE_LOOP_B_ENG1_FIRE"], 0.0, "the healthy loop's own raw reading must say no fire");
        assert_eq!(map["FIRE_LOOP_ENG1_DISAGREE"], 1.0, "the disagreement itself must be a visible discrete");
        assert_eq!(map["FIRE_LOOP_ENG2_DISAGREE"], 0.0, "engine 2 is untouched and healthy");
    }

    #[test]
    fn a_leaking_cargo_bottle_shows_low_pressure_when_not_pressed() {
        let mut truth = ground_running_truth();
        truth.engine_running = [false; 4];
        truth.apu_running = false;
        truth.dt_s = 60.0;
        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::from_pairs([(fire(219), 1.0)]), 5000);
        let map = published(area.as_ref());
        assert_eq!(map["FIRE_BOTTLE_CARGO_FWD_LOW_PRESSURE"], 1.0, "a full-severity leak must empty the cargo FWD bottle over hours");
        assert_eq!(map["FIRE_BOTTLE_CARGO_AFT_LOW_PRESSURE"], 0.0, "the AFT bottle is healthy");
    }

    #[test]
    fn cargo_distribution_and_squib_stage_faults_publish_independently_per_hold() {
        let mut healthy = live_system();
        run(healthy.as_mut(), &ground_running_truth(), &Faults::default(), 5);
        let map = published(healthy.as_ref());
        for b in ["FWD", "AFT"] {
            assert_eq!(map[&format!("FIRE_CARGO_{b}_DISTRIBUTION_FAULT")], 0.0, "healthy must be silent");
            assert_eq!(map[&format!("FIRE_CARGO_{b}_KNOCKDOWN_SQUIB_FAULT")], 0.0);
            assert_eq!(map[&format!("FIRE_CARGO_{b}_EXTENDED_SQUIB_FAULT")], 0.0);
        }

        let mut faulted = live_system();
        run(faulted.as_mut(), &ground_running_truth(), &Faults::from_pairs([(fire(222), 1.0)]), 5);
        let map = published(faulted.as_ref());
        assert_eq!(map["FIRE_CARGO_FWD_DISTRIBUTION_FAULT"], 1.0);
        assert_eq!(map["FIRE_CARGO_FWD_KNOCKDOWN_SQUIB_FAULT"], 0.0, "the distribution fault must not raise the squib discretes");
        assert_eq!(map["FIRE_CARGO_AFT_DISTRIBUTION_FAULT"], 0.0, "the AFT hold is unaffected");

        let mut faulted2 = live_system();
        run(faulted2.as_mut(), &ground_running_truth(), &Faults::from_pairs([(fire(225), 1.0)]), 5);
        let map2 = published(faulted2.as_ref());
        assert_eq!(map2["FIRE_CARGO_AFT_EXTENDED_SQUIB_FAULT"], 1.0);
        assert_eq!(map2["FIRE_CARGO_AFT_KNOCKDOWN_SQUIB_FAULT"], 0.0);
        assert_eq!(map2["FIRE_CARGO_FWD_EXTENDED_SQUIB_FAULT"], 0.0, "the FWD hold is unaffected");
    }

    #[test]
    fn pressing_the_cargo_smoke_discharge_pushbutton_actually_fires_a_healthy_bottle() {
        let mut truth = ground_running_truth();
        truth.engine_running = [false; 4];
        truth.apu_running = false;
        truth.controls.cargo_agent_pb_pressed = [true, false];
        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::default(), 5);
        let map = published(area.as_ref());
        assert!(map["FIRE_ZONE_CARGO_FWD_AGENT_FRACTION"] > 0.0, "agent must actually reach the pressed FWD cargo zone");
        assert_eq!(map["FIRE_ZONE_CARGO_AFT_AGENT_FRACTION"], 0.0, "the AFT bottle was not pressed and must not discharge");
    }

    #[test]
    fn pressing_neither_cargo_smoke_pushbutton_fires_nothing() {
        let truth = ground_running_truth();
        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::default(), 5);
        let map = published(area.as_ref());
        assert_eq!(map["FIRE_ZONE_CARGO_FWD_AGENT_FRACTION"], 0.0);
        assert_eq!(map["FIRE_ZONE_CARGO_AFT_AGENT_FRACTION"], 0.0);
    }

    #[test]
    fn a_wing_anti_ice_valve_stuck_open_overheats_the_leading_edge_in_clear_air() {
        let truth = Truth {
            dt_s: 1.0,
            environment: crate::deep::integration::weather_truth::EnvironmentTruth { sat_c: 15.0, leading_edge_c: 15.0, ambient_pressure_pa: 101_325.0, tas_ms: 100.0, precipitation_on_aircraft_ratio: 0.0, weather: None },
            on_ground: false,
            ..Truth::default()
        };
        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::from_pairs([(ice(2), 1.0)]), 400);
        let map = published(area.as_ref());
        assert_eq!(map["ANTI_ICE_WING_L_VALVE_OPEN"], 1.0, "a stuck-open valve flows regardless of command");
        assert_eq!(map["ANTI_ICE_WING_L_OVERHEAT"], 1.0, "surface reached {} C", map["ANTI_ICE_WING_L_SURFACE_C"]);
        assert_eq!(map["ANTI_ICE_WING_R_OVERHEAT"], 0.0, "the right wing's valve is healthy");
    }

    #[test]
    fn a_probe_heater_controller_fault_leaves_the_probe_icing_and_annunciates() {
        let truth = icing_truth();
        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::from_pairs([(ice(20), 1.0)]), 120);
        let map = published(area.as_ref());
        assert_eq!(map["PROBE_HEAT_PITOT1_FAULT"], 1.0);
        assert!(map["PROBE_HEAT_PITOT1_TEMPERATURE_C"] < 0.0, "an unheated probe in icing air must sit below freezing, got {}", map["PROBE_HEAT_PITOT1_TEMPERATURE_C"]);
        assert_eq!(map["PROBE_HEAT_PITOT2_FAULT"], 0.0, "the other probes are healthy and stay warm");
        assert!(map["PROBE_HEAT_PITOT2_TEMPERATURE_C"] > 0.0);
    }

    #[test]
    fn a_windshield_film_defect_burns_a_hot_spot_and_annunciates() {
        let truth = icing_truth();
        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::from_pairs([(ice(43), 0.9)]), 120);
        let map = published(area.as_ref());
        assert!(map["WINDOW_HEAT_L_HOT_SPOT_C"] > map["WINDOW_HEAT_L_TEMPERATURE_C"], "the defect must concentrate power into a hot spot");
        assert_eq!(map["WINDOW_HEAT_L_FAULT"], 1.0);
        assert_eq!(map["WINDOW_HEAT_R_FAULT"], 0.0);
    }

    #[test]
    fn a_leaking_fire_bottle_falls_below_its_pressure_switch_with_no_fire_anywhere() {
        let mut truth = ground_running_truth();
        truth.engine_running = [false; 4];
        truth.apu_running = false;
        truth.dt_s = 10.0;
        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::from_pairs([(fire(201), 1.0)]), 4000);
        let map = published(area.as_ref());
        assert_eq!(map["FIRE_BOTTLE_ENG1_1_LOW_PRESSURE"], 1.0, "a full-severity leak must empty the bottle over hours");
        assert_eq!(map["FIRE_BOTTLE_ENG1_2_LOW_PRESSURE"], 0.0, "the second bottle is healthy");
        assert_eq!(map["DEEP_FIRE_DETECTED_ENG:1"], 0.0, "a leaking bottle is not a fire");
    }

    #[test]
    fn pressing_the_fire_and_agent_pushbuttons_actually_fires_a_healthy_bottle() {
        let mut truth = ground_running_truth();
        truth.engine_running = [false; 4];
        truth.apu_running = false;
        truth.controls.fire_pb_released[0] = true;
        truth.controls.fire_agent_pb_pressed[0][0] = true;
        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::default(), 5);
        let map = published(area.as_ref());
        assert_eq!(map["DEEP_FIRE_SQUIB_1_ENG_1_IS_DISCHARGED"], 1.0, "the fire and agent pushbuttons together must fire bottle 1's squib");
        assert_eq!(map["DEEP_FIRE_SQUIB_2_ENG_1_IS_DISCHARGED"], 0.0, "only the pressed bottle's squib fires");
        assert!(map["FIRE_ZONE_ENG1_AGENT_FRACTION"] > 0.0, "agent must actually reach the zone");
    }

    #[test]
    fn pulling_only_the_fire_handle_without_pressing_an_agent_bottle_fires_nothing() {
        let mut truth = ground_running_truth();
        truth.engine_running = [false; 4];
        truth.apu_running = false;
        truth.controls.fire_pb_released[0] = true;
        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::default(), 5);
        let map = published(area.as_ref());
        assert_eq!(map["DEEP_FIRE_SQUIB_1_ENG_1_IS_DISCHARGED"], 0.0);
        assert_eq!(map["DEEP_FIRE_SQUIB_2_ENG_1_IS_DISCHARGED"], 0.0);
    }

    #[test]
    fn an_apu_bottle_also_fires_from_the_real_pushbutton_pair_in_flight_with_no_automatic_path() {
        let mut truth = ground_running_truth();
        truth.on_ground = false;
        truth.apu_running = true;
        truth.controls.fire_pb_apu_released = true;
        truth.controls.fire_agent_pb_apu_pressed = true;
        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::default(), 5);
        assert_eq!(published(area.as_ref())["DEEP_FIRE_SQUIB_1_APU_1_IS_DISCHARGED"], 1.0);
    }

    #[test]
    fn a_wing_anti_ice_valve_stuck_closed_with_the_crew_selected_on_still_ices_the_leading_edge() {
        let truth = icing_truth();
        let mut stuck = live_system();
        let mut healthy = live_system();
        let selected_on = Truth { controls: crate::deep::live::Controls { wing_anti_ice_selected: true, ..truth.controls }, ..truth.clone() };
        run(stuck.as_mut(), &selected_on, &Faults::from_pairs([(ice(1), 1.0)]), 300);
        run(healthy.as_mut(), &selected_on, &Faults::default(), 300);
        let stuck_map = published(stuck.as_ref());
        let healthy_map = published(healthy.as_ref());
        assert_eq!(stuck_map["ANTI_ICE_WING_L_VALVE_OPEN"], 0.0, "a valve stuck closed must deliver no bleed even though the crew selected anti-ice on");
        assert!(healthy_map["ANTI_ICE_WING_L_VALVE_OPEN"] > 0.0, "the healthy wing must actually flow once selected on");
        assert!(stuck_map["ANTI_ICE_WING_L_SURFACE_C"] < healthy_map["ANTI_ICE_WING_L_SURFACE_C"] - 5.0, "the stuck-closed wing must run colder than the heated one");
    }

    #[test]
    fn without_any_selection_a_healthy_wing_anti_ice_valve_stays_shut_in_icing_air() {
        let truth = icing_truth();
        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::default(), 300);
        assert_eq!(published(area.as_ref())["ANTI_ICE_WING_L_VALVE_OPEN"], 0.0, "with anti-ice not selected, a healthy valve must stay shut");
    }

    #[test]
    fn ice_accretes_on_an_unprotected_wing_in_a_real_cloud_and_costs_lift() {
        let mut truth = icing_truth();
        truth.environment.weather = Some(crate::deep::weather::WeatherSample {
            clouds: [
                crate::deep::weather::WeatherCloudLayer { cloud_type: 1.0, coverage: 1.0, alt_base_m: 1000.0, alt_top_m: 4000.0 },
                crate::deep::weather::WeatherCloudLayer::default(),
                crate::deep::weather::WeatherCloudLayer::default(),
            ],
            ..Default::default()
        });
        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::default(), 1800);
        let map = published(area.as_ref());
        assert!(map["ICE_WING_L_THICKNESS_M"] > 0.0, "a stratus cloud at -10 C must accrete ice on an unheated leading edge");
        assert!(map["ICE_WING_L_CL_MAX_LOSS"] > 0.0);
    }

    #[test]
    fn a_cold_dark_aircraft_publishes_finite_values_and_a_zero_dt_frame_changes_nothing() {
        let mut area = live_system();
        let truth = Truth::default();
        run(area.as_mut(), &truth, &Faults::default(), 100);
        for (name, value) in published(area.as_ref()) {
            assert!(value.is_finite(), "{name} went non-finite");
            if name.starts_with("FIRE_DETECTED_") {
                assert_eq!(value, 0.0, "{name} must be quiet on a cold aircraft");
            }
        }
        let still = Truth { dt_s: 0.0, ..Truth::default() };
        area.tick(&still, &Faults::default());
        let before = published(area.as_ref());
        area.tick(&still, &Faults::default());
        assert_eq!(before, published(area.as_ref()));
    }
}
