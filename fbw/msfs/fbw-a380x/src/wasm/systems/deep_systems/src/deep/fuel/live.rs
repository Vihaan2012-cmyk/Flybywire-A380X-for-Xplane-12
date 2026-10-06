use crate::deep::api::{failure_id, Area, Registry};
use crate::deep::live::{Faults, Truth};
use crate::weight_balance;

use super::cg_transfer::{self, TransferFaultDetector, TransferFaults};
use super::gauging::{self, ProbeFault};
use super::geometry::{self, Tank, TankShape, ALL_TANKS};
use super::jettison::{self, JettisonValve, NOMINAL_JETTISON_PUMP_RISE_PA, NOMINAL_NOZZLE_CDA_M2};
use super::leak::{self, LeakDetector};
use super::thermal::{self, FuelType};

const ATA: u16 = 28;
const N_TANKS: usize = 11;
const N_ENGINES: usize = 4;
const ENGINE_NACELLE_LEAK_VARS: [&str; N_ENGINES] =
    ["A32NX_ENG_1_NACELLE_FUEL_LEAK_KG_S", "A32NX_ENG_2_NACELLE_FUEL_LEAK_KG_S", "A32NX_ENG_3_NACELLE_FUEL_LEAK_KG_S", "A32NX_ENG_4_NACELLE_FUEL_LEAK_KG_S"];

use crate::physics::fluids::JET_A_DENSITY_KG_M3_AT_15C as REFERENCE_DENSITY_15C_KG_M3;
const FUEL_CP_J_KGK: f64 = 2010.0;
const TANK_SKIN_U_W_M2K: f64 = 40.0;

const MAX_TANK_WALL_LEAK_AREA_M2: f64 = 1.0e-4;
const MAX_GALLERY_LEAK_AREA_M2: f64 = 1.0e-5;

const LEAK_WINDOW_S: f64 = 35.0;
const LEAK_THRESHOLD_KG: f64 = 10.0;
const LEAK_CONFIRM_WINDOWS: u32 = 3;

const FULL_LOAD_FRACTION: f64 = 0.95;

const LINE_FLOW_GAIN: f64 = crate::fuel_network::DEFAULT_LINE_FLOW_GAIN;
const LB_TO_KG: f64 = 0.45359237;

const FEED_LINE_FUEL_FLOW_AT_1PSI: f64 = 0.00175;

const TRIM_PUMP_PRESSURE_PSI: f64 = 53.28;
const WING_TRANSFER_PUMP_PRESSURE_PSI: f64 = 36.8;

fn nominal_transfer_rate_kg_s(pressure_psi: f64) -> f64 {
    (FEED_LINE_FUEL_FLOW_AT_1PSI * LINE_FLOW_GAIN * pressure_psi.max(0.0) * LB_TO_KG).max(0.0)
}
const TRANSFER_TOLERANCE: f64 = 0.10;

const JETTISON_VALVE_TRAVEL_S: f64 = 5.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct FuelCommands {
    pub apu_fuel_flow_kg_s: f64,
    pub pitch_deg: f64,
    pub bank_deg: f64,
    pub lateral_accel_g: f64,
    pub longitudinal_accel_g: f64,
}

struct Ids {
    baffle: [u64; N_TANKS],
    probe: [u64; N_TANKS],
    compensator: [u64; N_TANKS],
    densitometer: [u64; N_TANKS],
    trim_pump: [u64; 2],
    trim_inlet: [u64; 2],
    trim_iso: [u64; 2],
    outer_xfer: [u64; 2],
    inner_xfer: [u64; 2],
    mid_xfer: [u64; 2],
    crossfeed: [u64; 4],
    fcoc: [u64; N_ENGINES],
    filter_water: [u64; N_ENGINES],
    filter_heater: [u64; N_ENGINES],
    jettison_valve: [u64; 2],
    jettison_nozzle: [u64; 2],
    tank_leak: [u64; N_TANKS],
    gallery_leak: [u64; 2],
    apu_feed_pump: u64,
    apu_feed_valve: u64,
    eng_lp_valve: [u64; N_ENGINES],
    feed_main: [u64; N_ENGINES],
    feed_stby: [u64; N_ENGINES],
    wing_outer: [u64; 2],
    wing_mid_fwd: [u64; 2],
    wing_mid_aft: [u64; 2],
    wing_inner_fwd: [u64; 2],
    wing_inner_aft: [u64; 2],
    leak_detector_fault: u64,
    fqdc: [u64; 2],
    fqms: [u64; 2],
    seq_norm: u64,
    seq_altn: u64,
    wb_backup: u64,
}

fn fid(reg: &Registry, component: &str, field_fragment: &str) -> u64 {
    let mut found = reg.failures.iter().filter(|f| f.component == component && f.model_field.contains(field_fragment));
    let first = found.next().unwrap_or_else(|| panic!("no failure on {component} whose model_field contains {field_fragment:?}"));
    assert!(found.next().is_none(), "more than one failure on {component} matches {field_fragment:?}");
    first.id
}

fn fids(reg: &Registry, component: &str) -> Vec<u64> {
    reg.failures.iter().filter(|f| f.component == component).map(|f| f.id).collect()
}

const TANK_SUFFIX: [&str; N_TANKS] =
    ["left_outer", "feed_1", "left_mid", "left_inner", "feed_2", "feed_3", "right_inner", "right_mid", "feed_4", "right_outer", "trim"];

impl Ids {
    fn resolve() -> Self {
        let mut reg = Registry::default();
        super::registry::register(&mut reg);

        let mut baffle = [0u64; N_TANKS];
        let mut probe = [0u64; N_TANKS];
        let mut compensator = [0u64; N_TANKS];
        let mut densitometer = [0u64; N_TANKS];
        let mut tank_leak = [0u64; N_TANKS];
        for (i, suffix) in TANK_SUFFIX.iter().enumerate() {
            baffle[i] = fid(&reg, &format!("28_fuel.tank_geometry.{suffix}"), "slosh_damping_ratio");
            let probes = fids(&reg, &format!("28_fuel.fqms_probes.{suffix}"));
            assert_eq!(probes.len(), 2, "each probe array registers one probe and one compensator failure");
            probe[i] = probes[0];
            compensator[i] = probes[1];
            densitometer[i] = fid(&reg, &format!("28_fuel.densitometer.{suffix}"), "densitometer_failed");
            tank_leak[i] = fid(&reg, &format!("28_fuel.tank_wall.{suffix}"), "tank_wall_leak_kg_s");
        }

        let valve = |id: &str| fid(&reg, id, "valve_stuck_fraction");
        let mut crossfeed = [0u64; 4];
        for (i, slot) in crossfeed.iter_mut().enumerate() {
            *slot = valve(&format!("28_fuel.valve.crossfeed_{}", i + 1));
        }

        let mut fcoc = [0u64; N_ENGINES];
        let mut filter_water = [0u64; N_ENGINES];
        let mut filter_heater = [0u64; N_ENGINES];
        for n in 0..N_ENGINES {
            fcoc[n] = fid(&reg, &format!("28_fuel.fcoc.{}", n + 1), "fcoc_temperature_rise_k");
            let filters = fids(&reg, &format!("28_fuel.filter.{}", n + 1));
            assert_eq!(filters.len(), 2, "each feed filter registers a water and a heater failure");
            filter_water[n] = filters[0];
            filter_heater[n] = filters[1];
        }

        Self {
            baffle,
            probe,
            compensator,
            densitometer,
            trim_pump: [fid(&reg, "28_fuel.pump.trim_left", "pump_degradation_fraction"), fid(&reg, "28_fuel.pump.trim_right", "pump_degradation_fraction")],
            trim_inlet: [valve("28_fuel.valve.trim_inlet_1"), valve("28_fuel.valve.trim_inlet_2")],
            trim_iso: [valve("28_fuel.valve.trim_iso_fwd"), valve("28_fuel.valve.trim_iso_aft")],
            outer_xfer: [valve("28_fuel.valve.outer_xfer_left"), valve("28_fuel.valve.outer_xfer_right")],
            inner_xfer: [valve("28_fuel.valve.inner_xfer_left"), valve("28_fuel.valve.inner_xfer_right")],
            mid_xfer: [valve("28_fuel.valve.mid_xfer_left"), valve("28_fuel.valve.mid_xfer_right")],
            crossfeed,
            fcoc,
            filter_water,
            filter_heater,
            jettison_valve: [fid(&reg, "28_fuel.valve.jettison_left", "JettisonValve"), fid(&reg, "28_fuel.valve.jettison_right", "JettisonValve")],
            jettison_nozzle: [fid(&reg, "28_fuel.nozzle.jettison_left", "effective_cda_m2"), fid(&reg, "28_fuel.nozzle.jettison_right", "effective_cda_m2")],
            tank_leak,
            gallery_leak: [fid(&reg, "28_fuel.gallery.forward", "gallery_leak_fraction"), fid(&reg, "28_fuel.gallery.aft", "gallery_leak_fraction")],
            apu_feed_pump: fid(&reg, "28_fuel.pump.apu_feed", "apu_feed_pump_degradation"),
            apu_feed_valve: fid(&reg, "28_fuel.valve.apu_feed", "JettisonValve"),
            eng_lp_valve: std::array::from_fn(|i| fid(&reg, &format!("28_fuel.valve.eng_lp.{}", i + 1), "JettisonValve")),
            feed_main: std::array::from_fn(|i| fid(&reg, &format!("28_fuel.pump.feed_main.{}", i + 1), "feed_pump_degradation")),
            feed_stby: std::array::from_fn(|i| fid(&reg, &format!("28_fuel.pump.feed_stby.{}", i + 1), "feed_pump_degradation")),
            wing_outer: [fid(&reg, "28_fuel.pump.outer.left", "pump_degradation_fraction"), fid(&reg, "28_fuel.pump.outer.right", "pump_degradation_fraction")],
            wing_mid_fwd: [fid(&reg, "28_fuel.pump.mid_fwd.left", "pump_degradation_fraction"), fid(&reg, "28_fuel.pump.mid_fwd.right", "pump_degradation_fraction")],
            wing_mid_aft: [fid(&reg, "28_fuel.pump.mid_aft.left", "pump_degradation_fraction"), fid(&reg, "28_fuel.pump.mid_aft.right", "pump_degradation_fraction")],
            wing_inner_fwd: [fid(&reg, "28_fuel.pump.inner_fwd.left", "pump_degradation_fraction"), fid(&reg, "28_fuel.pump.inner_fwd.right", "pump_degradation_fraction")],
            wing_inner_aft: [fid(&reg, "28_fuel.pump.inner_aft.left", "pump_degradation_fraction"), fid(&reg, "28_fuel.pump.inner_aft.right", "pump_degradation_fraction")],
            leak_detector_fault: fid(&reg, "28_fuel.leak_detector", "LeakDetector"),
            fqdc: [fid(&reg, "28_fuel.computer.fqdc.1", "fqms_low_confidence"), fid(&reg, "28_fuel.computer.fqdc.2", "fqms_low_confidence")],
            fqms: [fid(&reg, "28_fuel.computer.fqms.1", "fqms_low_confidence"), fid(&reg, "28_fuel.computer.fqms.2", "fqms_low_confidence")],
            seq_norm: fid(&reg, "28_fuel.computer.transfer_sequencer", "transfer_sequencer_norm_fault"),
            seq_altn: fid(&reg, "28_fuel.computer.transfer_sequencer", "transfer_sequencer_altn_fault"),
            wb_backup: fid(&reg, "28_fuel.computer.wb_backup", "wb_backup_fault"),
        }
    }
}

const COMPONENT_FAULT_FRACTION: f64 = 0.5;

#[derive(Clone, Debug)]
struct TankState {
    shape: TankShape,
    mass_kg: f64,
    temp_c: f64,
    probes: Vec<ProbeFault>,
    indicated_fraction: f64,
    confidence: f64,
    indicated_mass_kg: f64,
    leak_kg_s: f64,
}

impl TankState {
    fn new(tank: Tank, ambient_c: f64) -> Self {
        let shape = TankShape::of(tank);
        let probe_count = gauging::probe_count_for_capacity_gal(shape.capacity_gal).max(1) as usize;
        Self {
            shape,
            mass_kg: 0.0,
            temp_c: ambient_c,
            probes: vec![ProbeFault::default(); probe_count],
            indicated_fraction: 0.0,
            confidence: 1.0,
            indicated_mass_kg: 0.0,
            leak_kg_s: 0.0,
        }
    }

    fn density_kg_m3(&self) -> f64 {
        crate::physics::fluids::jet_a_density_kg_m3(self.temp_c).max(1.0)
    }

    fn fill_fraction(&self) -> f64 {
        let capacity_kg = self.shape.capacity_m3() * self.density_kg_m3();
        if capacity_kg <= 0.0 {
            return 0.0;
        }
        (self.mass_kg / capacity_kg).clamp(0.0, 1.0)
    }

    fn liquid_depth_m(&self) -> f64 {
        self.fill_fraction() * self.shape.box_height_m()
    }

    fn wetted_area_m2(&self) -> f64 {
        let h = self.shape.box_height_m();
        let l = self.shape.box_length_m();
        let depth = self.liquid_depth_m();
        (h * l) + 2.0 * (depth * l)
    }
}

pub struct FuelLive {
    ids: Ids,
    tanks: Vec<TankState>,
    filter_blockage: [f64; N_ENGINES],
    filter_ice: [f64; N_ENGINES],
    jettison_valves: [JettisonValve; 2],
    jettison_flow_kg_s: [f64; 2],
    jettison_fault: [bool; 2],
    trim_detector: TransferFaultDetector,
    cg_detector: TransferFaultDetector,
    crossfeed_detector: TransferFaultDetector,
    trim_fault: bool,
    cg_fault: bool,
    crossfeed_fault: bool,
    crossfeed_open: bool,
    trim_pump_degradation: [f64; 2],
    leak_detector: LeakDetector,
    leak_detected: bool,
    total_leak_kg_s: f64,
    baffle_damage_detected: bool,
    fqms_low_confidence: bool,
    fob_lo_temp: bool,
    filter_ice_detected: bool,
    filter_water_fraction: [f64; N_ENGINES],
    filter_heater_failed: [bool; N_ENGINES],
    indicated_fob_kg: f64,
    tank_positions_ft: Vec<[f64; 3]>,
    fuel_cg_ft: f64,
    pub commands: FuelCommands,
    pub fuel_type: FuelType,
    synced_from_real: bool,

    apu_feed_valve: JettisonValve,
    apu_feed_valve_fault: bool,
    apu_feed_valve_not_closed: bool,
    apu_feed_pump_fault: bool,
    eng_lp_valve: [JettisonValve; N_ENGINES],
    eng_lp_valve_fault: [bool; N_ENGINES],
    feed_main_fault: [bool; N_ENGINES],
    feed_stby_fault: [bool; N_ENGINES],
    wing_outer_fault: [bool; 2],
    wing_mid_fwd_fault: [bool; 2],
    wing_mid_aft_fault: [bool; 2],
    wing_inner_fwd_fault: [bool; 2],
    wing_inner_aft_fault: [bool; 2],
    leak_detector_self_fault: bool,
    outer_transfer_fault: bool,
    jettison_valve_not_closed: [bool; 2],
    wing_imbalance_kg: f64,
    wing_imbalance_known: bool,
    wing_imbalance_ever_exceeded: bool,
    fqdc_fault: [bool; 2],
    fqms_fault: [bool; 2],
    transfer_sequencer_norm_fault: bool,
    transfer_sequencer_altn_fault: bool,
    wb_backup_fault: bool,
    crossfeed_valve_fault: [bool; 4],
    eng_leak_detectors: [LeakDetector; N_ENGINES],
    eng_leak_detected: [bool; N_ENGINES],
    eng_contamination_detected: [bool; N_ENGINES],
}

impl Default for FuelLive {
    fn default() -> Self {
        Self::new()
    }
}

impl FuelLive {
    pub fn new() -> Self {
        let ambient_c = Truth::default().environment.sat_c;
        let mut live = Self {
            ids: Ids::resolve(),
            tanks: ALL_TANKS.iter().map(|&t| TankState::new(t, ambient_c)).collect(),
            filter_blockage: [0.0; N_ENGINES],
            filter_ice: [0.0; N_ENGINES],
            jettison_valves: [JettisonValve::new(); 2],
            jettison_flow_kg_s: [0.0; 2],
            jettison_fault: [false; 2],
            trim_detector: TransferFaultDetector::default(),
            cg_detector: TransferFaultDetector::default(),
            crossfeed_detector: TransferFaultDetector::default(),
            trim_fault: false,
            cg_fault: false,
            crossfeed_fault: false,
            crossfeed_open: false,
            trim_pump_degradation: [0.0; 2],
            leak_detector: LeakDetector::new(),
            leak_detected: false,
            total_leak_kg_s: 0.0,
            baffle_damage_detected: false,
            fqms_low_confidence: false,
            fob_lo_temp: false,
            filter_ice_detected: false,
            filter_water_fraction: [0.0; N_ENGINES],
            filter_heater_failed: [false; N_ENGINES],
            indicated_fob_kg: 0.0,
            tank_positions_ft: weight_balance::parse(weight_balance::FLIGHT_MODEL_CFG).tanks.into_iter().take(N_TANKS).collect(),
            fuel_cg_ft: 0.0,
            commands: FuelCommands::default(),
            fuel_type: FuelType::JetA1,
            synced_from_real: false,
            apu_feed_valve: JettisonValve::new(),
            apu_feed_valve_fault: false,
            apu_feed_valve_not_closed: false,
            apu_feed_pump_fault: false,
            eng_lp_valve: [JettisonValve::new(); N_ENGINES],
            eng_lp_valve_fault: [false; N_ENGINES],
            feed_main_fault: [false; N_ENGINES],
            feed_stby_fault: [false; N_ENGINES],
            wing_outer_fault: [false; 2],
            wing_mid_fwd_fault: [false; 2],
            wing_mid_aft_fault: [false; 2],
            wing_inner_fwd_fault: [false; 2],
            wing_inner_aft_fault: [false; 2],
            leak_detector_self_fault: false,
            outer_transfer_fault: false,
            jettison_valve_not_closed: [false; 2],
            wing_imbalance_kg: 0.0,
            wing_imbalance_known: false,
            wing_imbalance_ever_exceeded: false,
            fqdc_fault: [false; 2],
            fqms_fault: [false; 2],
            transfer_sequencer_norm_fault: false,
            transfer_sequencer_altn_fault: false,
            wb_backup_fault: false,
            crossfeed_valve_fault: [false; 4],
            eng_leak_detectors: [LeakDetector::new(), LeakDetector::new(), LeakDetector::new(), LeakDetector::new()],
            eng_leak_detected: [false; N_ENGINES],
            eng_contamination_detected: [false; N_ENGINES],
        };
        live.seed_default_fuel_load(ambient_c);
        live
    }

    fn seed_default_fuel_load(&mut self, temp_c: f64) {
        for &tank in ALL_TANKS.iter() {
            let capacity_gal = TankShape::of(tank).capacity_gal;
            let kg = capacity_gal * FULL_LOAD_FRACTION * geometry::GAL_TO_M3 * REFERENCE_DENSITY_15C_KG_M3;
            self.load_tank(tank, kg, temp_c);
        }
    }

    pub fn load_tank(&mut self, tank: Tank, kg: f64, temp_c: f64) {
        let i = ALL_TANKS.iter().position(|&t| t == tank).expect("every tank is in ALL_TANKS");
        self.tanks[i].temp_c = temp_c;
        let capacity_kg = self.tanks[i].shape.capacity_m3() * self.tanks[i].density_kg_m3();
        self.tanks[i].mass_kg = kg.max(0.0).min(capacity_kg.max(0.0));
    }

    fn sync_from_real(&mut self, gallons: [f64; N_TANKS]) {
        for (i, &tank) in ALL_TANKS.iter().enumerate() {
            let density = self.tanks[i].density_kg_m3();
            let kg = gallons[i].max(0.0) * geometry::GAL_TO_M3 * density;
            self.load_tank(tank, kg, self.tanks[i].temp_c);
        }
        self.synced_from_real = true;
    }

    const EXTERNAL_CHANGE_KG: f64 = 300.0;

    fn adopt_external_changes(&mut self, gallons: [f64; N_TANKS]) {
        for (i, &tank) in ALL_TANKS.iter().enumerate() {
            let density = self.tanks[i].density_kg_m3();
            let real_kg = gallons[i].max(0.0) * geometry::GAL_TO_M3 * density;
            if (real_kg - self.tanks[i].mass_kg).abs() > Self::EXTERNAL_CHANGE_KG {
                self.load_tank(tank, real_kg, self.tanks[i].temp_c);
            }
        }
    }

    pub fn tank_mass_kg(&self, tank: Tank) -> f64 {
        let i = ALL_TANKS.iter().position(|&t| t == tank).expect("every tank is in ALL_TANKS");
        self.tanks[i].mass_kg
    }

    pub fn tank_temp_c(&self, tank: Tank) -> f64 {
        let i = ALL_TANKS.iter().position(|&t| t == tank).expect("every tank is in ALL_TANKS");
        self.tanks[i].temp_c
    }

    pub fn true_fob_kg(&self) -> f64 {
        self.tanks.iter().map(|t| t.mass_kg).sum()
    }

    pub fn indicated_fob_kg(&self) -> f64 {
        self.indicated_fob_kg
    }

    fn feed_tank_index(engine: usize) -> usize {
        const FEED: [Tank; N_ENGINES] = [Tank::Feed1, Tank::Feed2, Tank::Feed3, Tank::Feed4];
        ALL_TANKS.iter().position(|&t| t == FEED[engine]).expect("feed tanks are in ALL_TANKS")
    }

    fn wing_masses_kg(&self) -> (f64, f64) {
        let mut left = 0.0;
        let mut right = 0.0;
        for (i, tank) in ALL_TANKS.iter().enumerate() {
            match tank {
                Tank::LeftOuter | Tank::Feed1 | Tank::LeftMid | Tank::LeftInner | Tank::Feed2 => left += self.tanks[i].mass_kg,
                Tank::Feed3 | Tank::RightInner | Tank::RightMid | Tank::Feed4 | Tank::RightOuter => right += self.tanks[i].mass_kg,
                Tank::Trim => {}
            }
        }
        (left, right)
    }
}

const WING_IMBALANCE_LIMIT_KG: f64 = 3000.0;

const OUTER_RETENTION_UNTIL_FRACTION: f64 = 0.25;

impl crate::deep::live::Area for FuelLive {
    fn name(&self) -> &'static str {
        "fuel"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        if let Some(gallons) = truth.fuel_tank_quantity_gal {
            if !self.synced_from_real {
                if gallons.iter().any(|&g| g > 0.0) {
                    self.sync_from_real(gallons);
                }
            } else {
                self.adopt_external_changes(gallons);
            }
        }
        let dt = truth.dt_s.max(0.0);
        let ambient_pa = truth.environment.ambient_pressure_pa.max(1.0);
        let skin_c = truth.environment.leading_edge_c;

        self.total_leak_kg_s = 0.0;
        self.baffle_damage_detected = false;
        self.fqms_low_confidence = false;
        self.fob_lo_temp = false;

        let gallery_leak_fraction = self.ids.gallery_leak.iter().map(|&id| faults.get(id)).fold(0.0f64, f64::max);
        let gallery_leak_area = leak::leak_area_m2(gallery_leak_fraction, MAX_GALLERY_LEAK_AREA_M2);

        for i in 0..N_TANKS {
            let baffle_damage = faults.get(self.ids.baffle[i]);
            let nominal_damping = TankShape::of(ALL_TANKS[i]).slosh_damping_ratio;
            self.tanks[i].shape.slosh_damping_ratio = nominal_damping * (1.0 - baffle_damage);
            if baffle_damage > 0.0 {
                self.baffle_damage_detected = true;
            }

            let probe_fault = faults.get(self.ids.probe[i]);
            let compensator = faults.get(self.ids.compensator[i]);
            let count = self.tanks[i].probes.len();
            for (j, p) in self.tanks[i].probes.iter_mut().enumerate() {
                let own = if j == 0 { probe_fault } else { 0.0 };
                let common = compensator * gauging::PROBE_DEAD_THRESHOLD * 0.999;
                p.failure_fraction = own.max(common).clamp(0.0, 1.0);
            }
            debug_assert!(count > 0);

            let fcoc_w = self.fcoc_heat_into_tank_w(i, truth, faults);
            let ua_w_k = TANK_SKIN_U_W_M2K * self.tanks[i].wetted_area_m2();
            let mass = self.tanks[i].mass_kg;
            if mass > 0.0 && ua_w_k > 0.0 {
                let equilibrium_c = skin_c + fcoc_w / ua_w_k;
                let net_w = fcoc_w + ua_w_k * (skin_c - self.tanks[i].temp_c);
                let step_k = thermal::fcoc_temperature_rise_k(net_w.abs(), dt, mass, FUEL_CP_J_KGK).copysign(net_w);
                let stepped = self.tanks[i].temp_c + step_k;
                self.tanks[i].temp_c = if net_w >= 0.0 { stepped.min(equilibrium_c) } else { stepped.max(equilibrium_c) };
            } else if mass <= 0.0 {
                self.tanks[i].temp_c = skin_c;
            }

            if thermal::wax_fraction(self.tanks[i].temp_c, self.fuel_type) > 0.0 {
                self.fob_lo_temp = true;
            }

            let density = self.tanks[i].density_kg_m3();
            let area = leak::leak_area_m2(faults.get(self.ids.tank_leak[i]), MAX_TANK_WALL_LEAK_AREA_M2);
            let wall_leak = leak::tank_wall_leak_kg_s(area, self.tanks[i].liquid_depth_m(), density);
            self.tanks[i].leak_kg_s = wall_leak;
            self.tanks[i].mass_kg = (self.tanks[i].mass_kg - wall_leak * dt).max(0.0);
            self.total_leak_kg_s += wall_leak;
        }

        if gallery_leak_area > 0.0 {
            let density = self.tanks[0].density_kg_m3();
            let line_pa = ambient_pa + NOMINAL_JETTISON_PUMP_RISE_PA;
            let gallery = leak::gallery_leak_kg_s(gallery_leak_area, line_pa, ambient_pa, density);
            self.total_leak_kg_s += gallery;
            if let Some(idx) = (0..N_TANKS).max_by(|&a, &b| self.tanks[a].mass_kg.total_cmp(&self.tanks[b].mass_kg)) {
                self.tanks[idx].mass_kg = (self.tanks[idx].mass_kg - gallery * dt).max(0.0);
            }
        }

        for eng in 0..N_ENGINES {
            let idx = Self::feed_tank_index(eng);
            let burn = truth.engine_fuel_flow_kg_s[eng].max(0.0) * dt;
            let nacelle_leak_kg_s = truth.published.get_or(ENGINE_NACELLE_LEAK_VARS[eng], 0.0).max(0.0);
            self.total_leak_kg_s += nacelle_leak_kg_s;
            self.tanks[idx].mass_kg = (self.tanks[idx].mass_kg - burn - nacelle_leak_kg_s * dt).max(0.0);
        }
        if self.commands.apu_fuel_flow_kg_s > 0.0 {
            let idx = Self::feed_tank_index(1);
            self.tanks[idx].mass_kg = (self.tanks[idx].mass_kg - self.commands.apu_fuel_flow_kg_s * dt).max(0.0);
        }

        self.filter_ice_detected = false;
        for eng in 0..N_ENGINES {
            let idx = Self::feed_tank_index(eng);
            let temp_c = self.tanks[idx].temp_c;
            let water = faults.get(self.ids.filter_water[eng]);
            let heater_failed = faults.get(self.ids.filter_heater[eng]) >= 0.5;
            let ice = thermal::filter_ice_blockage_fraction(temp_c, water, !heater_failed);
            let wax = thermal::wax_fraction(temp_c, self.fuel_type);
            self.filter_ice[eng] = ice;
            self.filter_blockage[eng] = thermal::filter_blockage_fraction(wax, ice);
            self.filter_water_fraction[eng] = water;
            self.filter_heater_failed[eng] = heater_failed;
            if ice > 0.0 {
                self.filter_ice_detected = true;
            }
        }

        self.indicated_fob_kg = 0.0;
        for i in 0..N_TANKS {
            let shape = self.tanks[i].shape;
            let fill = self.tanks[i].fill_fraction();
            let tilt = geometry::tilt_fraction(&shape, self.commands.pitch_deg, self.commands.bank_deg, self.commands.lateral_accel_g, self.commands.longitudinal_accel_g);
            let (indicated, confidence) = gauging::fqms_indicated_fraction(fill, tilt, &self.tanks[i].probes);
            self.tanks[i].indicated_fraction = indicated;
            self.tanks[i].confidence = confidence;
            if confidence < 1.0 {
                self.fqms_low_confidence = true;
            }
            let densitometer_failed = faults.get(self.ids.densitometer[i]) >= 0.5;
            if densitometer_failed {
                self.fqms_low_confidence = true;
            }
            let mass = gauging::indicated_mass_kg(indicated, shape.capacity_gal, self.tanks[i].density_kg_m3(), densitometer_failed, REFERENCE_DENSITY_15C_KG_M3);
            self.tanks[i].indicated_mass_kg = mass;
            self.indicated_fob_kg += mass;
        }

        self.tick_transfers(truth, faults, dt, gallery_leak_fraction);

        let masses = self.tanks.iter().zip(self.tank_positions_ft.iter()).map(|(t, &position)| weight_balance::Mass { pounds: t.mass_kg / weight_balance::LB_TO_KG, position });
        self.fuel_cg_ft = weight_balance::centre_of_gravity(masses).1[0];

        self.tick_jettison(truth, faults, dt);

        let metered_flow =
            truth.engine_fuel_flow_kg_s.iter().map(|f| f.max(0.0)).sum::<f64>() + self.commands.apu_fuel_flow_kg_s.max(0.0) + self.jettison_flow_kg_s[0].max(0.0) + self.jettison_flow_kg_s[1].max(0.0);
        let raw_leak_detected = self.leak_detector.update(self.indicated_fob_kg, metered_flow, dt, LEAK_WINDOW_S, LEAK_THRESHOLD_KG, LEAK_CONFIRM_WINDOWS);
        self.leak_detector_self_fault = faults.get(self.ids.leak_detector_fault) > 0.0;
        self.leak_detected = raw_leak_detected && !self.leak_detector_self_fault;

        for eng in 0..N_ENGINES {
            let idx = Self::feed_tank_index(eng);
            let indicated = self.tanks[idx].indicated_mass_kg;
            let flow = truth.engine_fuel_flow_kg_s[eng].max(0.0);
            self.eng_leak_detected[eng] = self.eng_leak_detectors[eng].update(indicated, flow, dt, LEAK_WINDOW_S, LEAK_THRESHOLD_KG, LEAK_CONFIRM_WINDOWS) && !self.leak_detector_self_fault;
        }

        for eng in 0..N_ENGINES {
            self.eng_contamination_detected[eng] = self.filter_water_fraction[eng] > 0.0;
        }

        self.tick_named_units(truth, faults, dt);
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        let b = |x: bool| if x { 1.0 } else { 0.0 };

        out("FUEL_LEAK_DETECTED", b(self.leak_detected));
        out("FUEL_CROSSFEED_OPEN", b(self.crossfeed_open));
        out("FUEL_CROSSFEED_FAULT", b(self.crossfeed_fault));
        out("FUEL_TRIM_TRANSFER_FAULT", b(self.trim_fault));
        out("FUEL_CG_TRANSFER_DEGRADED", b(self.cg_fault));
        out("FUEL_FOB_LO_TEMP", b(self.fob_lo_temp));
        out("FUEL_FILTER_ICE_DETECTED", b(self.filter_ice_detected));
        out("FUEL_JETTISON_L_VALVE_FAULT", b(self.jettison_fault[0]));
        out("FUEL_JETTISON_R_VALVE_FAULT", b(self.jettison_fault[1]));
        out("FUEL_FQMS_LOW_CONFIDENCE", b(self.fqms_low_confidence));
        out("FUEL_TANK_BAFFLE_DAMAGE_DETECTED", b(self.baffle_damage_detected));

        out("FUEL_TOTAL_FOB_KG", self.indicated_fob_kg);
        out("FUEL_TOTAL_TRUE_FOB_KG", self.true_fob_kg());
        out("FUEL_TOTAL_LEAK_KG_S", self.total_leak_kg_s);
        out("FUEL_CG_LONGITUDINAL_FT", self.fuel_cg_ft);
        for (i, tank) in self.tanks.iter().enumerate() {
            let n = i + 1;
            out(&format!("FUEL_TANK_QTY_KG:{n}"), tank.indicated_mass_kg);
            out(&format!("FUEL_TANK_TRUE_QTY_KG:{n}"), if self.synced_from_real { tank.mass_kg } else { -1.0 });
            out(&format!("FUEL_TANK_TEMP_C:{n}"), tank.temp_c);
            out(&format!("FUEL_TANK_FQMS_CONFIDENCE:{n}"), tank.confidence);
            out(&format!("FUEL_TANK_LEAK_KG_S:{n}"), tank.leak_kg_s);
        }
        for eng in 0..N_ENGINES {
            let n = eng + 1;
            out(&format!("FUEL_FILTER_BLOCKAGE:{n}"), self.filter_blockage[eng]);
            out(&format!("FUEL_FILTER_ICE:{n}"), self.filter_ice[eng]);
            out(&format!("FUEL_FILTER_WATER_FRACTION:{n}"), self.filter_water_fraction[eng]);
            out(&format!("FUEL_FILTER_HEATER_FAULT:{n}"), b(self.filter_heater_failed[eng]));
        }
        for side in 0..2 {
            let n = side + 1;
            out(&format!("FUEL_JETTISON_VALVE_POSITION:{n}"), self.jettison_valves[side].position);
            out(&format!("FUEL_JETTISON_FLOW_KG_S:{n}"), self.jettison_flow_kg_s[side]);
        }
        for pump in 0..2 {
            out(&format!("FUEL_TRIM_PUMP_DEGRADATION:{}", pump + 1), self.trim_pump_degradation[pump]);
        }

        out("FUEL_APU_FEED_PUMP_FAULT", b(self.apu_feed_pump_fault));
        out("FUEL_APU_FEED_VALVE_FAULT", b(self.apu_feed_valve_fault));
        out("FUEL_APU_FEED_VALVE_NOT_CLOSED", b(self.apu_feed_valve_not_closed));
        out("FUEL_APU_FEED_VALVE_POSITION", self.apu_feed_valve.position);
        for eng in 0..N_ENGINES {
            let n = eng + 1;
            out(&format!("FUEL_ENG_LP_VALVE_FAULT:{n}"), b(self.eng_lp_valve_fault[eng]));
            out(&format!("FUEL_FEED_PUMP_FAULT:main_{n}"), b(self.feed_main_fault[eng]));
            out(&format!("FUEL_FEED_PUMP_FAULT:stby_{n}"), b(self.feed_stby_fault[eng]));
            out(&format!("FUEL_ENG_LEAK_DETECTED:{n}"), b(self.eng_leak_detected[eng]));
            out(&format!("FUEL_ENG_CONTAMINATION_DETECTED:{n}"), b(self.eng_contamination_detected[eng]));
        }
        for (side, name) in [(0usize, "left"), (1, "right")] {
            out(&format!("FUEL_WING_PUMP_FAULT:outer_{name}"), b(self.wing_outer_fault[side]));
            out(&format!("FUEL_WING_PUMP_FAULT:mid_fwd_{name}"), b(self.wing_mid_fwd_fault[side]));
            out(&format!("FUEL_WING_PUMP_FAULT:mid_aft_{name}"), b(self.wing_mid_aft_fault[side]));
            out(&format!("FUEL_WING_PUMP_FAULT:inner_fwd_{name}"), b(self.wing_inner_fwd_fault[side]));
            out(&format!("FUEL_WING_PUMP_FAULT:inner_aft_{name}"), b(self.wing_inner_aft_fault[side]));
        }
        out("FUEL_LEAK_DETECTOR_FAULT", b(self.leak_detector_self_fault));
        out("FUEL_OUTER_TRANSFER_FAULT", b(self.outer_transfer_fault));
        out("FUEL_JETTISON_VALVE_NOT_CLOSED:1", b(self.jettison_valve_not_closed[0]));
        out("FUEL_JETTISON_VALVE_NOT_CLOSED:2", b(self.jettison_valve_not_closed[1]));
        out("FUEL_WING_IMBALANCE_KG", self.wing_imbalance_kg);
        out("FUEL_WING_IMBALANCE_KNOWN", b(self.wing_imbalance_known));
        out("FUEL_WING_IMBALANCE_EVER_EXCEEDED", b(self.wing_imbalance_ever_exceeded));
        out("FUEL_FQDC_FAULT:1", b(self.fqdc_fault[0]));
        out("FUEL_FQDC_FAULT:2", b(self.fqdc_fault[1]));
        out("FUEL_FQMS_FAULT:1", b(self.fqms_fault[0]));
        out("FUEL_FQMS_FAULT:2", b(self.fqms_fault[1]));
        out("FUEL_TRANSFER_SEQUENCER_FAULT:norm", b(self.transfer_sequencer_norm_fault));
        out("FUEL_TRANSFER_SEQUENCER_FAULT:altn", b(self.transfer_sequencer_altn_fault));
        out("FUEL_WB_BACKUP_FAULT", b(self.wb_backup_fault));
        for i in 0..4 {
            out(&format!("FUEL_CROSSFEED_VALVE_FAULT:{}", i + 1), b(self.crossfeed_valve_fault[i]));
        }
    }
}

impl FuelLive {
    fn fcoc_heat_into_tank_w(&self, tank_index: usize, truth: &Truth, faults: &Faults) -> f64 {
        let mut total = 0.0;
        for eng in 0..N_ENGINES {
            if Self::feed_tank_index(eng) != tank_index || !truth.engine_running[eng] {
                continue;
            }
            let fouling = faults.get(self.ids.fcoc[eng]);
            total += FCOC_HEAT_AT_TAKEOFF_W * truth.engine_n1_frac[eng].clamp(0.0, 1.0) * (1.0 - fouling);
        }
        total
    }

    fn move_fuel(&mut self, from: usize, to: usize, want_kg: f64) -> f64 {
        let want = want_kg.max(0.0);
        if want <= 0.0 || from == to {
            return 0.0;
        }
        let available = self.tanks[from].mass_kg.max(0.0);
        let to_capacity_kg = self.tanks[to].shape.capacity_m3() * self.tanks[to].density_kg_m3();
        let room = (to_capacity_kg - self.tanks[to].mass_kg).max(0.0);
        let moved = want.min(available).min(room);
        self.tanks[from].mass_kg -= moved;
        self.tanks[to].mass_kg += moved;
        moved
    }

    fn cg_source_index(&self, inner: Tank, mid: Tank, outer: Tank, outer_retained: bool) -> Option<usize> {
        let idx = |t: Tank| ALL_TANKS.iter().position(|&x| x == t).expect("every tank is in ALL_TANKS");
        let (i, m, o) = (idx(inner), idx(mid), idx(outer));
        if self.tanks[i].mass_kg > 0.0 {
            Some(i)
        } else if self.tanks[m].mass_kg > 0.0 {
            Some(m)
        } else if !outer_retained && self.tanks[o].mass_kg > 0.0 {
            Some(o)
        } else {
            None
        }
    }

    fn tick_transfers(&mut self, truth: &Truth, faults: &Faults, dt: f64, gallery_leak_fraction: f64) {
        let worst = |ids: &[u64]| ids.iter().map(|&id| faults.get(id)).fold(0.0f64, f64::max);

        for (i, &id) in self.ids.trim_pump.iter().enumerate() {
            self.trim_pump_degradation[i] = faults.get(id);
        }

        let trim_nominal = nominal_transfer_rate_kg_s(TRIM_PUMP_PRESSURE_PSI);
        let trim_pump_loss = self.ids.trim_pump.iter().map(|&id| faults.get(id)).fold(f64::INFINITY, f64::min);
        let trim = TransferFaults {
            valve_stuck_fraction: worst(&self.ids.trim_inlet).min(1.0).max(worst(&self.ids.trim_iso)),
            pump_degradation_fraction: if trim_pump_loss.is_finite() { trim_pump_loss } else { 0.0 },
            gallery_leak_fraction,
        };
        let achieved = cg_transfer::achieved_transfer_rate_kg_s(trim_nominal, &trim);
        self.trim_fault = self.trim_detector.update(trim_nominal, achieved, TRANSFER_TOLERANCE, 1.0, dt);

        if let Some(trim_idx) = ALL_TANKS.iter().position(|&t| t == Tank::Trim) {
            let feed_indices = [Tank::Feed1, Tank::Feed2, Tank::Feed3, Tank::Feed4].map(|t| ALL_TANKS.iter().position(|&x| x == t).expect("feed tanks are in ALL_TANKS"));
            let share = achieved * dt / feed_indices.len() as f64;
            for feed_idx in feed_indices {
                self.move_fuel(trim_idx, feed_idx, share);
            }
        }

        let inner_fill = self.fill_of(Tank::LeftInner).max(self.fill_of(Tank::RightInner));
        let mid_fill = self.fill_of(Tank::LeftMid).max(self.fill_of(Tank::RightMid));
        let outer_retained = cg_transfer::outer_tank_retention_active(inner_fill, mid_fill, OUTER_RETENTION_UNTIL_FRACTION);
        let cg_required = self.true_fob_kg() > 0.0 || outer_retained;
        let cg_nominal = nominal_transfer_rate_kg_s(WING_TRANSFER_PUMP_PRESSURE_PSI);
        let cg = TransferFaults {
            valve_stuck_fraction: worst(&self.ids.outer_xfer).max(worst(&self.ids.inner_xfer)).max(worst(&self.ids.mid_xfer)),
            pump_degradation_fraction: 0.0,
            gallery_leak_fraction,
        };
        let cg_required_rate = if cg_required { cg_nominal } else { 0.0 };
        let cg_achieved = cg_transfer::achieved_transfer_rate_kg_s(cg_required_rate, &cg);
        self.cg_fault = self.cg_detector.update(cg_required_rate, cg_achieved, TRANSFER_TOLERANCE, 5.0, dt);

        let sides = [(Tank::LeftInner, Tank::LeftMid, Tank::LeftOuter, [Tank::Feed1, Tank::Feed2]), (Tank::RightInner, Tank::RightMid, Tank::RightOuter, [Tank::Feed3, Tank::Feed4])];
        for (side, (inner, mid, outer, feeds)) in sides.into_iter().enumerate() {
            if let Some(src) = self.cg_source_index(inner, mid, outer, outer_retained) {
                let pump_derate = 1.0 - self.wing_pump_derate(side, ALL_TANKS[src]);
                let share = cg_achieved * pump_derate * dt / feeds.len() as f64;
                for feed in feeds {
                    let feed_idx = ALL_TANKS.iter().position(|&x| x == feed).expect("feed tanks are in ALL_TANKS");
                    self.move_fuel(src, feed_idx, share);
                }
            }
        }

        let (left, right) = self.wing_masses_kg();
        self.crossfeed_open = truth.controls.crossfeed_valve_selected.iter().any(|&s| s);
        let xfeed_required = self.crossfeed_open || cg_transfer::wing_balance_transfer_needed(left, right, WING_IMBALANCE_LIMIT_KG);
        let xfeed_nominal = nominal_transfer_rate_kg_s(WING_TRANSFER_PUMP_PRESSURE_PSI);
        let xfeed = TransferFaults { valve_stuck_fraction: worst(&self.ids.crossfeed), pump_degradation_fraction: 0.0, gallery_leak_fraction };
        let xfeed_required_rate = if xfeed_required { xfeed_nominal } else { 0.0 };
        let xfeed_achieved = cg_transfer::achieved_transfer_rate_kg_s(xfeed_required_rate, &xfeed);
        self.crossfeed_fault = self.crossfeed_detector.update(xfeed_required_rate, xfeed_achieved, TRANSFER_TOLERANCE, 2.0, dt);

        if self.crossfeed_open {
            let xfeed_move_rate = xfeed_achieved;
            let (left_now, right_now) = self.wing_masses_kg();
            if let Some(heavy) = cg_transfer::heavy_side(left_now, right_now, WING_IMBALANCE_LIMIT_KG) {
                let (heavy_feeds, light_feeds): (&[Tank], &[Tank]) =
                    if heavy == cg_transfer::HeavySide::Left { (&[Tank::Feed1, Tank::Feed2], &[Tank::Feed3, Tank::Feed4]) } else { (&[Tank::Feed3, Tank::Feed4], &[Tank::Feed1, Tank::Feed2]) };
                let idx_of = |t: Tank| ALL_TANKS.iter().position(|&x| x == t).expect("feed tanks are in ALL_TANKS");
                let source = heavy_feeds.iter().map(|&t| idx_of(t)).max_by(|&a, &b| self.tanks[a].mass_kg.total_cmp(&self.tanks[b].mass_kg)).expect("heavy_feeds is non-empty");
                let dest = light_feeds.iter().map(|&t| idx_of(t)).min_by(|&a, &b| self.tanks[a].mass_kg.total_cmp(&self.tanks[b].mass_kg)).expect("light_feeds is non-empty");
                self.move_fuel(source, dest, xfeed_move_rate * dt);
            }
        }
    }

    fn fill_of(&self, tank: Tank) -> f64 {
        let i = ALL_TANKS.iter().position(|&t| t == tank).expect("every tank is in ALL_TANKS");
        self.tanks[i].fill_fraction()
    }

    fn tick_jettison(&mut self, truth: &Truth, faults: &Faults, dt: f64) {
        let ambient_pa = truth.environment.ambient_pressure_pa.max(0.0);

        const LEFT_GROUP: [Tank; 4] = [Tank::LeftInner, Tank::LeftMid, Tank::LeftOuter, Tank::Feed1];
        const RIGHT_GROUP: [Tank; 4] = [Tank::RightInner, Tank::RightMid, Tank::RightOuter, Tank::Feed4];

        for side in 0..2 {
            let commanded = truth.controls.jettison_armed && truth.controls.jettison_valve_selected[side];
            let stuck = faults.get(self.ids.jettison_valve[side]);
            self.jettison_valves[side].step(commanded, JETTISON_VALVE_TRAVEL_S, stuck, dt);
            let position = self.jettison_valves[side].position;

            let blockage = faults.get(self.ids.jettison_nozzle[side]);
            let cda = jettison::effective_cda_m2(NOMINAL_NOZZLE_CDA_M2, blockage, position);
            let clear_cda = jettison::effective_cda_m2(NOMINAL_NOZZLE_CDA_M2, 0.0, position);

            let group = if side == 0 { LEFT_GROUP } else { RIGHT_GROUP };
            let source = group
                .iter()
                .map(|&t| ALL_TANKS.iter().position(|&x| x == t).expect("every tank is in ALL_TANKS"))
                .max_by(|&a, &b| self.tanks[a].liquid_depth_m().total_cmp(&self.tanks[b].liquid_depth_m()));
            let Some(idx) = source else { continue };

            let density = self.tanks[idx].density_kg_m3();
            let depth = self.tanks[idx].liquid_depth_m();
            let pump_pa = if depth > 0.0 && commanded { NOMINAL_JETTISON_PUMP_RISE_PA } else { 0.0 };
            let flow = jettison::jettison_mass_flow_kg_s(cda, depth, pump_pa, ambient_pa, ambient_pa, density);
            let clear_flow = jettison::jettison_mass_flow_kg_s(clear_cda, depth, pump_pa, ambient_pa, ambient_pa, density);
            self.jettison_flow_kg_s[side] = flow;
            self.tanks[idx].mass_kg = (self.tanks[idx].mass_kg - flow * dt).max(0.0);

            let target = if commanded { 1.0 } else { 0.0 };
            let valve_disagree = stuck > 0.0 && (position - target).abs() > VALVE_DISAGREE_TOLERANCE;
            let rate_short = commanded && clear_flow > 0.0 && flow < clear_flow * (1.0 - TRANSFER_TOLERANCE);
            self.jettison_fault[side] = valve_disagree || rate_short;

            self.jettison_valve_not_closed[side] = stuck > 0.0 && position > VALVE_DISAGREE_TOLERANCE && !commanded;
        }
    }

    fn tick_named_units(&mut self, truth: &Truth, faults: &Faults, dt: f64) {
        let fault = |frac: f64| frac >= COMPONENT_FAULT_FRACTION;

        self.apu_feed_pump_fault = fault(faults.get(self.ids.apu_feed_pump));
        let apu_commanded = truth.controls.apu_master_sw_on || self.commands.apu_fuel_flow_kg_s > 0.0;
        let apu_stuck = faults.get(self.ids.apu_feed_valve);
        self.apu_feed_valve.step(apu_commanded, JETTISON_VALVE_TRAVEL_S, apu_stuck, dt);
        let apu_target = if apu_commanded { 1.0 } else { 0.0 };
        self.apu_feed_valve_fault = apu_stuck > 0.0 && (self.apu_feed_valve.position - apu_target).abs() > VALVE_DISAGREE_TOLERANCE;
        self.apu_feed_valve_not_closed = apu_stuck > 0.0 && self.apu_feed_valve.position > VALVE_DISAGREE_TOLERANCE && !apu_commanded;

        for eng in 0..N_ENGINES {
            let commanded = truth.controls.engine_master_on[eng];
            let stuck = faults.get(self.ids.eng_lp_valve[eng]);
            self.eng_lp_valve[eng].step(commanded, JETTISON_VALVE_TRAVEL_S, stuck, dt);
            let target = if commanded { 1.0 } else { 0.0 };
            self.eng_lp_valve_fault[eng] = stuck > 0.0 && (self.eng_lp_valve[eng].position - target).abs() > VALVE_DISAGREE_TOLERANCE;
        }

        for eng in 0..N_ENGINES {
            self.feed_main_fault[eng] = fault(faults.get(self.ids.feed_main[eng]));
            self.feed_stby_fault[eng] = fault(faults.get(self.ids.feed_stby[eng]));
        }

        for side in 0..2 {
            self.wing_outer_fault[side] = fault(faults.get(self.ids.wing_outer[side]));
            self.wing_mid_fwd_fault[side] = fault(faults.get(self.ids.wing_mid_fwd[side]));
            self.wing_mid_aft_fault[side] = fault(faults.get(self.ids.wing_mid_aft[side]));
            self.wing_inner_fwd_fault[side] = fault(faults.get(self.ids.wing_inner_fwd[side]));
            self.wing_inner_aft_fault[side] = fault(faults.get(self.ids.wing_inner_aft[side]));
        }

        self.outer_transfer_fault = fault(faults.get(self.ids.outer_xfer[0])) || fault(faults.get(self.ids.outer_xfer[1]));

        match truth.fuel_tank_quantity_gal {
            Some(gal) => {
                let left_kg: f64 = gal[0..5].iter().sum::<f64>() * geometry::GAL_TO_M3 * REFERENCE_DENSITY_15C_KG_M3;
                let right_kg: f64 = gal[5..10].iter().sum::<f64>() * geometry::GAL_TO_M3 * REFERENCE_DENSITY_15C_KG_M3;
                self.wing_imbalance_kg = (left_kg - right_kg).abs();
                self.wing_imbalance_known = true;
                if self.wing_imbalance_kg >= WING_IMBALANCE_LIMIT_KG {
                    self.wing_imbalance_ever_exceeded = true;
                }
            }
            None => {
                self.wing_imbalance_kg = 0.0;
                self.wing_imbalance_known = false;
            }
        }

        for ch in 0..2 {
            self.fqdc_fault[ch] = faults.get(self.ids.fqdc[ch]) > 0.0;
            self.fqms_fault[ch] = faults.get(self.ids.fqms[ch]) > 0.0;
            if self.fqdc_fault[ch] || self.fqms_fault[ch] {
                self.fqms_low_confidence = true;
            }
        }
        self.transfer_sequencer_norm_fault = faults.get(self.ids.seq_norm) > 0.0;
        self.transfer_sequencer_altn_fault = faults.get(self.ids.seq_altn) > 0.0;

        for i in 0..4 {
            self.crossfeed_valve_fault[i] = fault(faults.get(self.ids.crossfeed[i]));
        }
        self.wb_backup_fault = faults.get(self.ids.wb_backup) > 0.0;
    }

    fn wing_pump_derate(&self, side: usize, src_tank: Tank) -> f64 {
        let d = match src_tank {
            Tank::LeftOuter | Tank::RightOuter => self.wing_outer_health(side),
            Tank::LeftMid | Tank::RightMid => self.wing_mid_health(side),
            Tank::LeftInner | Tank::RightInner => self.wing_inner_health(side),
            _ => 0.0,
        };
        d.clamp(0.0, 1.0)
    }
    fn wing_outer_health(&self, side: usize) -> f64 {
        if self.wing_outer_fault[side] {
            1.0
        } else {
            0.0
        }
    }
    fn wing_mid_health(&self, side: usize) -> f64 {
        if self.wing_mid_fwd_fault[side] && self.wing_mid_aft_fault[side] {
            1.0
        } else {
            0.0
        }
    }
    fn wing_inner_health(&self, side: usize) -> f64 {
        if self.wing_inner_fwd_fault[side] && self.wing_inner_aft_fault[side] {
            1.0
        } else {
            0.0
        }
    }
}

const VALVE_DISAGREE_TOLERANCE: f64 = 0.05;

const FCOC_HEAT_AT_TAKEOFF_W: f64 = 60_000.0;

pub fn live_system() -> Box<dyn crate::deep::live::Area> {
    Box::new(FuelLive::new())
}

#[cfg(test)]
pub(crate) mod test_support {
    use crate::deep::api::Cond;

    pub(crate) fn collect_vars(cond: &Cond, out: &mut Vec<String>) {
        match cond {
            Cond::Always => {}
            Cond::Var { name, .. } => out.push(name.clone()),
            Cond::VarVar { a, b, .. } => {
                out.push(a.clone());
                out.push(b.clone());
            }
            Cond::And(v) | Cond::Or(v) => v.iter().for_each(|c| collect_vars(c, out)),
            Cond::Not(c) => collect_vars(c, out),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::live::{Area as _, Controls};
    use std::collections::BTreeMap;

    fn published(area: &dyn crate::deep::live::Area) -> BTreeMap<String, f64> {
        let mut map = BTreeMap::new();
        area.publish(&mut |name, value| {
            map.insert(name.to_string(), value);
        });
        map
    }

    fn run(area: &mut dyn crate::deep::live::Area, truth: &Truth, faults: &Faults, seconds: f64) -> BTreeMap<String, f64> {
        let steps = (seconds / truth.dt_s).ceil() as usize;
        for _ in 0..steps.max(1) {
            area.tick(truth, faults);
        }
        published(area)
    }

    fn full_tanks(live: &mut FuelLive, temp_c: f64) {
        for &tank in ALL_TANKS.iter() {
            let shape = TankShape::of(tank);
            let kg = shape.capacity_m3() * REFERENCE_DENSITY_15C_KG_M3 * 0.9;
            live.load_tank(tank, kg, temp_c);
        }
    }

    #[test]
    fn a_healthy_cold_aircraft_publishes_every_trigger_variable_and_raises_nothing() {
        let mut live = FuelLive::new();
        let out = run(&mut live, &Truth::default(), &Faults::default(), 1.0);
        for name in [
            "FUEL_LEAK_DETECTED",
            "FUEL_CROSSFEED_OPEN",
            "FUEL_CROSSFEED_FAULT",
            "FUEL_TRIM_TRANSFER_FAULT",
            "FUEL_CG_TRANSFER_DEGRADED",
            "FUEL_FOB_LO_TEMP",
            "FUEL_FILTER_ICE_DETECTED",
            "FUEL_JETTISON_L_VALVE_FAULT",
            "FUEL_JETTISON_R_VALVE_FAULT",
            "FUEL_FQMS_LOW_CONFIDENCE",
            "FUEL_TANK_BAFFLE_DAMAGE_DETECTED",
        ] {
            assert_eq!(out.get(name), Some(&0.0), "{name} should be published and healthy on a cold aircraft");
        }
    }

    #[test]
    fn every_variable_this_areas_alerts_trigger_on_is_published_by_this_live_system() {
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        let mut names = Vec::new();
        for alert in &reg.alerts {
            super::test_support::collect_vars(&alert.trigger, &mut names);
        }
        let mut live = FuelLive::new();
        live.tick(&Truth::default(), &Faults::default());
        let out = published(&live);
        for name in names {
            assert!(out.contains_key(&name), "alert trigger reads {name}, which nothing publishes");
        }
    }

    fn crossfeed_selected_truth() -> Truth {
        Truth { controls: Controls { crossfeed_valve_selected: [true; 4], ..Controls::default() }, ..Truth::default() }
    }

    fn jettison_selected_truth() -> Truth {
        Truth { controls: Controls { jettison_armed: true, jettison_valve_selected: [true; 2], ..Controls::default() }, ..Truth::default() }
    }

    #[test]
    fn a_stuck_crossfeed_valve_raises_the_wing_crossfeed_fault_its_registry_entry_promises() {
        let mut live = FuelLive::new();
        full_tanks(&mut live, 10.0);
        let id = live.ids.crossfeed[0];

        let healthy = run(&mut live, &crossfeed_selected_truth(), &Faults::default(), 5.0);
        assert_eq!(healthy.get("FUEL_CROSSFEED_FAULT"), Some(&0.0));

        let mut live = FuelLive::new();
        full_tanks(&mut live, 10.0);
        let faulted = run(&mut live, &crossfeed_selected_truth(), &Faults::from_pairs([(id, 1.0)]), 5.0);
        assert_eq!(faulted.get("FUEL_CROSSFEED_FAULT"), Some(&1.0), "a seized cross-feed valve must raise FUEL WING XFEED FAULT");
        assert_eq!(faulted.get("FUEL_CROSSFEED_OPEN"), Some(&1.0));
    }

    #[test]
    fn a_blocked_jettison_nozzle_cuts_the_rate_and_raises_the_jettison_fault() {
        let truth = jettison_selected_truth();
        let mut clear = FuelLive::new();
        full_tanks(&mut clear, 10.0);
        let nozzle = clear.ids.jettison_nozzle[0];
        let clear_out = run(&mut clear, &truth, &Faults::default(), 20.0);

        let mut blocked = FuelLive::new();
        full_tanks(&mut blocked, 10.0);
        let blocked_out = run(&mut blocked, &truth, &Faults::from_pairs([(nozzle, 0.8)]), 20.0);

        let clear_flow = clear_out["FUEL_JETTISON_FLOW_KG_S:1"];
        let blocked_flow = blocked_out["FUEL_JETTISON_FLOW_KG_S:1"];
        assert!(clear_flow > 0.0, "a commanded jettison with full tanks must actually flow");
        assert!(blocked_flow < clear_flow * 0.5, "an 80% blocked nozzle must roughly halve the rate at least: {blocked_flow} vs {clear_flow}");
        assert_eq!(blocked_out.get("FUEL_JETTISON_L_VALVE_FAULT"), Some(&1.0));
        assert_eq!(clear_out.get("FUEL_JETTISON_L_VALVE_FAULT"), Some(&0.0));
    }

    #[test]
    fn jettison_is_driven_by_head_and_pump_rise_not_by_altitude() {
        let mut sea_level = FuelLive::new();
        full_tanks(&mut sea_level, 10.0);
        let low = run(&mut sea_level, &jettison_selected_truth(), &Faults::default(), 20.0);

        let mut cruise = FuelLive::new();
        full_tanks(&mut cruise, 10.0);
        let mut truth = jettison_selected_truth();
        truth.environment.ambient_pressure_pa = 22_600.0;
        truth.altitude_ft = 35_000.0;
        truth.on_ground = false;
        let high = run(&mut cruise, &truth, &Faults::default(), 20.0);

        let a = low["FUEL_JETTISON_FLOW_KG_S:1"];
        let b = high["FUEL_JETTISON_FLOW_KG_S:1"];
        assert!(a > 0.0 && b > 0.0);
        assert!((a - b).abs() / a < 1e-9, "a vented tank's jettison rate cannot depend on altitude: {a} vs {b}");
    }

    #[test]
    fn a_failed_probe_costs_the_fqms_its_confidence_and_raises_the_qty_advisory() {
        let mut live = FuelLive::new();
        full_tanks(&mut live, 10.0);
        let id = live.ids.probe[3];
        let out = run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 1.0)]), 1.0);
        assert_eq!(out.get("FUEL_FQMS_LOW_CONFIDENCE"), Some(&1.0));
        assert!(out["FUEL_TANK_FQMS_CONFIDENCE:4"] < 1.0, "a dead probe must be excluded from its array");
    }

    #[test]
    fn a_holed_tank_loses_fuel_overboard_at_a_rate_set_by_its_own_head() {
        let mut live = FuelLive::new();
        full_tanks(&mut live, 10.0);
        let id = live.ids.tank_leak[3];
        let before = live.tank_mass_kg(Tank::LeftInner);
        let out = run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 1.0)]), 10.0);
        let after = live.tank_mass_kg(Tank::LeftInner);
        assert!(after < before, "a holed tank must actually lose fuel");
        assert!(out["FUEL_TANK_LEAK_KG_S:4"] > 0.0);
        assert!(out["FUEL_TOTAL_LEAK_KG_S"] > 0.0);
    }

    #[test]
    fn every_one_of_the_eleven_seeded_tanks_can_leak_not_just_the_four_feed_tanks() {
        for (tank, idx) in [(Tank::LeftOuter, 0usize), (Tank::Trim, 10usize)] {
            let mut live = FuelLive::new();
            let before = live.tank_mass_kg(tank);
            assert!(before > 0.0, "{tank:?} must be seeded with real fuel, not left dry: {before} kg");
            let id = live.ids.tank_leak[idx];
            run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 1.0)]), 10.0);
            let after = live.tank_mass_kg(tank);
            assert!(after < before, "{tank:?} must actually lose fuel once holed: {before} -> {after}");
        }
    }

    #[test]
    fn a_single_failed_trim_pump_is_masked_by_its_own_redundancy_but_still_shows_on_its_own_gauge() {
        let mut live = FuelLive::new();
        let id = live.ids.trim_pump[0];
        let out = run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 1.0)]), 5.0);
        assert_eq!(out.get("FUEL_TRIM_TRANSFER_FAULT"), Some(&0.0), "the healthy twin pump genuinely covers a single failure");
        assert_eq!(out.get("FUEL_TRIM_PUMP_DEGRADATION:1"), Some(&1.0), "but the failed pump's own health must still be a real, published reading");
        assert_eq!(out.get("FUEL_TRIM_PUMP_DEGRADATION:2"), Some(&0.0));
    }

    #[test]
    fn filter_water_and_heater_failures_each_move_their_own_direct_reading_even_alone() {
        let mut cold = Truth::default();
        cold.environment.sat_c = -30.0;
        cold.environment.leading_edge_c = -30.0;

        let mut live = FuelLive::new();
        let water_id = live.ids.filter_water[0];
        let out = run(&mut live, &cold, &Faults::from_pairs([(water_id, 1.0)]), 1.0);
        assert_eq!(out.get("FUEL_FILTER_WATER_FRACTION:1"), Some(&1.0), "the water-in-fuel reading must move even with a healthy heater");
        assert_eq!(out.get("FUEL_FILTER_ICE_DETECTED"), Some(&0.0), "a healthy heater genuinely suppresses ice regardless of water present");

        let mut live = FuelLive::new();
        let heater_id = live.ids.filter_heater[0];
        let out = run(&mut live, &cold, &Faults::from_pairs([(heater_id, 1.0)]), 1.0);
        assert_eq!(out.get("FUEL_FILTER_HEATER_FAULT:1"), Some(&1.0), "the heater-fault caution must move even with no water contamination armed");
        assert_eq!(out.get("FUEL_FILTER_ICE_DETECTED"), Some(&0.0), "there is genuinely nothing to freeze with no water present");
    }

    #[test]
    fn a_full_severity_feed_tank_leak_raises_fuel_leak_detected_within_120_seconds() {
        let mut live = FuelLive::new();
        let id = live.ids.tank_leak[1];
        let mut truth = Truth::default();
        truth.dt_s = 1.0;
        let out = run(&mut live, &truth, &Faults::from_pairs([(id, 1.0)]), 110.0);
        assert_eq!(out.get("FUEL_LEAK_DETECTED"), Some(&1.0), "a full-severity feed-tank leak must confirm within 120 s of unmetered loss");
    }

    #[test]
    fn a_commanded_jettison_alone_never_raises_fuel_leak_detected() {
        let mut live = FuelLive::new();
        let mut truth = jettison_selected_truth();
        truth.dt_s = 1.0;
        let out = run(&mut live, &truth, &Faults::default(), 110.0);
        assert_eq!(out.get("FUEL_LEAK_DETECTED"), Some(&0.0), "a commanded jettison is accounted for, not a leak");
    }

    fn drain_transfer_sources(live: &mut FuelLive) {
        for tank in [Tank::Trim, Tank::LeftOuter, Tank::LeftMid, Tank::LeftInner, Tank::RightInner, Tank::RightMid, Tank::RightOuter] {
            live.load_tank(tank, 0.0, 15.0);
        }
    }

    #[test]
    fn a_feed_tank_drains_at_the_engines_real_burn_from_a_seeded_load() {
        let mut live = FuelLive::new();
        let before = live.tank_mass_kg(Tank::Feed1);
        let feed2_before = live.tank_mass_kg(Tank::Feed2);
        assert!(before > 0.0, "a live aircraft must not start with dry tanks: {before} kg");
        drain_transfer_sources(&mut live);

        let mut truth = Truth::default();
        truth.dt_s = 1.0;
        truth.engine_running = [true; 4];
        truth.engine_fuel_flow_kg_s = [1.0, 0.0, 0.0, 0.0];
        for _ in 0..100 {
            live.tick(&truth, &Faults::default());
        }
        let after = live.tank_mass_kg(Tank::Feed1);
        assert!((before - after - 100.0).abs() < 1e-6, "feed 1 must lose exactly the commanded 1 kg/s burn: {before} -> {after}");
        assert_eq!(live.tank_mass_kg(Tank::Feed2), feed2_before, "the other feed tanks, fed by engines with zero commanded flow, must be untouched");

        let mut idle = FuelLive::new();
        drain_transfer_sources(&mut idle);
        let idle_before = idle.tank_mass_kg(Tank::Feed2);
        let mut idle_truth = Truth::default();
        idle_truth.dt_s = 1.0;
        idle_truth.engine_running = [true; 4];
        idle_truth.engine_n1_frac = [0.9; 4];
        idle_truth.engine_fuel_flow_kg_s = [0.0; 4];
        for _ in 0..50 {
            idle.tick(&idle_truth, &Faults::default());
        }
        assert_eq!(idle.tank_mass_kg(Tank::Feed2), idle_before, "N1 alone must not burn fuel; only the real Truth fuel flow does");
    }

    #[test]
    fn filter_icing_needs_free_water_cold_fuel_and_a_failed_heater() {
        let mut truth = Truth::default();
        truth.environment.sat_c = -30.0;
        truth.environment.leading_edge_c = -30.0;

        let mut live = FuelLive::new();
        full_tanks(&mut live, -10.0);
        let water = live.ids.filter_water[0];
        let heater = live.ids.filter_heater[0];

        let with_heater = run(&mut live, &truth, &Faults::from_pairs([(water, 0.5)]), 1.0);
        assert_eq!(with_heater.get("FUEL_FILTER_ICE_DETECTED"), Some(&0.0));

        let mut live = FuelLive::new();
        full_tanks(&mut live, -10.0);
        let iced = run(&mut live, &truth, &Faults::from_pairs([(water, 0.5), (heater, 1.0)]), 1.0);
        assert_eq!(iced.get("FUEL_FILTER_ICE_DETECTED"), Some(&1.0));
        assert!(iced["FUEL_FILTER_ICE:1"] > 0.0);
    }

    #[test]
    fn baffle_damage_reduces_slosh_damping_and_is_annunciated() {
        let mut live = FuelLive::new();
        full_tanks(&mut live, 10.0);
        let id = live.ids.baffle[3];
        let nominal = TankShape::of(Tank::LeftInner).slosh_damping_ratio;
        let out = run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 0.5)]), 1.0);
        assert_eq!(out.get("FUEL_TANK_BAFFLE_DAMAGE_DETECTED"), Some(&1.0));
        assert!((live.tanks[3].shape.slosh_damping_ratio - nominal * 0.5).abs() < 1e-12);
        let healthy_settle = geometry::sloshing_settle_time_s(&TankShape::of(Tank::LeftInner), 0.5);
        let damaged_settle = geometry::sloshing_settle_time_s(&live.tanks[3].shape, 0.5);
        assert!(damaged_settle > healthy_settle);
    }

    #[test]
    fn fuel_cold_soaks_toward_the_wing_skin_and_never_overshoots_it() {
        let mut truth = Truth::default();
        truth.dt_s = 1.0;
        truth.environment.sat_c = -55.0;
        truth.environment.leading_edge_c = -35.0;
        truth.on_ground = false;
        truth.altitude_ft = 37_000.0;

        let mut live = FuelLive::new();
        full_tanks(&mut live, 15.0);
        for _ in 0..20_000 {
            live.tick(&truth, &Faults::default());
        }
        let t = live.tank_temp_c(Tank::LeftInner);
        assert!(t < 0.0, "a long cruise in cold air must cold-soak the fuel: {t} C");
        assert!(t >= -35.0 - 1e-6, "fuel cannot get colder than the wall it is cooling against: {t} C");
        assert!(t.is_finite());
    }

    #[test]
    fn a_fouled_fcoc_leaves_the_feed_tank_colder_than_a_healthy_one() {
        let mut truth = Truth::default();
        truth.dt_s = 1.0;
        truth.environment.sat_c = -50.0;
        truth.environment.leading_edge_c = -30.0;
        truth.on_ground = false;
        truth.engine_running = [true; 4];
        truth.engine_n1_frac = [0.85; 4];

        let mut healthy = FuelLive::new();
        full_tanks(&mut healthy, 0.0);
        let id = healthy.ids.fcoc[0];
        for _ in 0..4000 {
            healthy.tick(&truth, &Faults::default());
        }

        let mut fouled = FuelLive::new();
        full_tanks(&mut fouled, 0.0);
        let faults = Faults::from_pairs([(id, 1.0)]);
        for _ in 0..4000 {
            fouled.tick(&truth, &faults);
        }

        assert!(
            fouled.tank_temp_c(Tank::Feed1) < healthy.tank_temp_c(Tank::Feed1),
            "a fouled FCOC must leave feed tank 1 colder: {} vs {}",
            fouled.tank_temp_c(Tank::Feed1),
            healthy.tank_temp_c(Tank::Feed1)
        );
    }

    #[test]
    fn cold_soaked_fuel_raises_the_low_temperature_caution_at_its_own_cloud_point() {
        let mut live = FuelLive::new();
        full_tanks(&mut live, -38.0);
        let mut truth = Truth::default();
        truth.environment.leading_edge_c = -38.0;
        let out = run(&mut live, &truth, &Faults::default(), 1.0);
        assert_eq!(out.get("FUEL_FOB_LO_TEMP"), Some(&1.0));

        let mut warm = FuelLive::new();
        full_tanks(&mut warm, 0.0);
        let mut truth = Truth::default();
        truth.environment.leading_edge_c = 0.0;
        let out = run(&mut warm, &truth, &Faults::default(), 1.0);
        assert_eq!(out.get("FUEL_FOB_LO_TEMP"), Some(&0.0));
    }

    #[test]
    fn nothing_divides_by_zero_on_an_empty_aircraft_at_zero_dt() {
        let mut live = FuelLive::new();
        let truth = Truth { dt_s: 0.0, ..Truth::default() };
        live.tick(&truth, &Faults::default());
        for (name, value) in published(&live) {
            assert!(value.is_finite(), "{name} is not finite");
        }
    }

    #[test]
    fn every_registered_failure_is_either_consumed_or_listed_as_not() {
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        let ids = Ids::resolve();
        let mut consumed: Vec<u64> = Vec::new();
        consumed.extend(ids.baffle);
        consumed.extend(ids.probe);
        consumed.extend(ids.compensator);
        consumed.extend(ids.densitometer);
        consumed.extend(ids.trim_pump);
        consumed.extend(ids.trim_inlet);
        consumed.extend(ids.trim_iso);
        consumed.extend(ids.outer_xfer);
        consumed.extend(ids.inner_xfer);
        consumed.extend(ids.mid_xfer);
        consumed.extend(ids.crossfeed);
        consumed.extend(ids.fcoc);
        consumed.extend(ids.filter_water);
        consumed.extend(ids.filter_heater);
        consumed.extend(ids.jettison_valve);
        consumed.extend(ids.jettison_nozzle);
        consumed.extend(ids.tank_leak);
        consumed.extend(ids.gallery_leak);
        consumed.push(ids.apu_feed_pump);
        consumed.push(ids.apu_feed_valve);
        consumed.extend(ids.eng_lp_valve);
        consumed.extend(ids.feed_main);
        consumed.extend(ids.feed_stby);
        consumed.extend(ids.wing_outer);
        consumed.extend(ids.wing_mid_fwd);
        consumed.extend(ids.wing_mid_aft);
        consumed.extend(ids.wing_inner_fwd);
        consumed.extend(ids.wing_inner_aft);
        consumed.push(ids.leak_detector_fault);
        consumed.extend(ids.fqdc);
        consumed.extend(ids.fqms);
        consumed.push(ids.seq_norm);
        consumed.push(ids.seq_altn);
        consumed.push(ids.wb_backup);
        consumed.sort_unstable();
        consumed.dedup();

        let registered: Vec<u64> = reg.failures.iter().map(|f| f.id).collect();
        assert_eq!(consumed.len(), registered.len(), "every fuel failure should be consumed by the live system");
        for id in registered {
            assert!(consumed.contains(&id), "failure {id} is registered but never read by the live system");
        }
    }

    #[test]
    fn the_ids_this_system_resolves_are_the_ones_the_registry_hands_out() {
        let a = Ids::resolve();
        let b = Ids::resolve();
        assert_eq!(a.crossfeed, b.crossfeed);
        assert_eq!(a.tank_leak, b.tank_leak);
        for id in a.tank_leak {
            assert_eq!(id / 1_000_000, Area::Fuel as u64);
            assert_eq!(id / 1_000 % 1_000, ATA as u64);
        }
        assert_eq!(a.baffle[0], failure_id(Area::Fuel, ATA, 1), "the first tank's baffle failure is the area's first id");
    }

    #[test]
    fn a_transfer_tick_moves_real_mass_and_conserves_total_fuel() {
        let mut live = FuelLive::new();
        for feed in [Tank::Feed1, Tank::Feed2, Tank::Feed3, Tank::Feed4] {
            live.load_tank(feed, 100.0, 10.0);
        }
        let before = live.true_fob_kg();
        let mut truth = Truth::default();
        truth.dt_s = 1.0;
        for _ in 0..30 {
            live.tick(&truth, &Faults::default());
        }
        let after = live.true_fob_kg();
        let moved_into_feed1 = live.tank_mass_kg(Tank::Feed1) - 100.0;
        assert!(moved_into_feed1 > 1.0, "trim/CG transfer must move real mass into a drained feed tank, not zero: {moved_into_feed1} kg");
        assert!((before - after).abs() < 1e-6, "moving fuel between tanks must conserve total system mass exactly: {before} -> {after}");
    }

    #[test]
    fn a_degraded_trim_pump_moves_less_mass_than_a_healthy_one_and_a_failed_pair_moves_none() {
        let feed_total_kg = |pump_magnitude: f64| -> f64 {
            let mut live = FuelLive::new();
            live.load_tank(Tank::Trim, 20_000.0, 10.0);
            for feed in [Tank::Feed1, Tank::Feed2, Tank::Feed3, Tank::Feed4] {
                live.load_tank(feed, 0.0, 10.0);
            }
            for tank in [Tank::LeftOuter, Tank::LeftMid, Tank::LeftInner, Tank::RightInner, Tank::RightMid, Tank::RightOuter] {
                live.load_tank(tank, 0.0, 10.0);
            }
            let (id0, id1) = (live.ids.trim_pump[0], live.ids.trim_pump[1]);
            let faults = Faults::from_pairs([(id0, pump_magnitude), (id1, pump_magnitude)]);
            let mut truth = Truth::default();
            truth.dt_s = 1.0;
            for _ in 0..20 {
                live.tick(&truth, &faults);
            }
            [Tank::Feed1, Tank::Feed2, Tank::Feed3, Tank::Feed4].iter().map(|&t| live.tank_mass_kg(t)).sum()
        };

        let healthy = feed_total_kg(0.0);
        let degraded = feed_total_kg(0.6);
        let failed = feed_total_kg(1.0);

        assert!(healthy > 1.0, "a healthy trim path must move real, measurable mass: {healthy} kg");
        assert!(degraded < healthy - 1.0, "a degraded trim pump must move measurably less than a healthy one: {degraded} vs {healthy}");
        assert!(degraded > 1.0, "a partially degraded pump pair still moves some fuel: {degraded} kg");
        assert_eq!(failed, 0.0, "both trim pumps fully failed must move exactly zero mass");
    }

    #[test]
    fn crossfeed_selected_rebalances_the_wings_and_shut_does_not() {
        let light_side_total_kg = |crossfeed_selected: bool| -> f64 {
            let mut live = FuelLive::new();
            for tank in [Tank::Trim, Tank::LeftOuter, Tank::LeftMid, Tank::LeftInner, Tank::RightInner, Tank::RightMid, Tank::RightOuter] {
                live.load_tank(tank, 0.0, 10.0);
            }
            live.load_tank(Tank::Feed1, 30_000.0, 10.0);
            live.load_tank(Tank::Feed2, 30_000.0, 10.0);
            live.load_tank(Tank::Feed3, 1_000.0, 10.0);
            live.load_tank(Tank::Feed4, 1_000.0, 10.0);
            let mut truth = if crossfeed_selected { crossfeed_selected_truth() } else { Truth::default() };
            truth.dt_s = 1.0;
            for _ in 0..60 {
                live.tick(&truth, &Faults::default());
            }
            live.tank_mass_kg(Tank::Feed3) + live.tank_mass_kg(Tank::Feed4)
        };

        let open = light_side_total_kg(true);
        let shut = light_side_total_kg(false);

        assert!(open > 2_000.0 + 10.0, "cross-feed selected must move real mass into the light side: {open} kg (started at 2000)");
        assert_eq!(shut, 2_000.0, "cross-feed shut must not move any fuel into the light side, however unbalanced the wings are");
    }

    #[test]
    fn jettison_armed_and_selected_reduces_total_fuel_at_a_real_rate_and_not_otherwise() {
        let mut armed = FuelLive::new();
        full_tanks(&mut armed, 10.0);
        let before_armed = armed.true_fob_kg();
        let out = run(&mut armed, &jettison_selected_truth(), &Faults::default(), 10.0);
        let after_armed = armed.true_fob_kg();
        let flow_kg_s = out["FUEL_JETTISON_FLOW_KG_S:1"] + out["FUEL_JETTISON_FLOW_KG_S:2"];
        assert!(flow_kg_s > 0.0, "a commanded jettison must show a real, nonzero published flow");
        assert!(before_armed - after_armed > 1.0, "jettison armed and selected must reduce total fuel at a real rate: {before_armed} -> {after_armed}");

        let mut idle = FuelLive::new();
        full_tanks(&mut idle, 10.0);
        let before_idle = idle.true_fob_kg();
        run(&mut idle, &Truth::default(), &Faults::default(), 10.0);
        let after_idle = idle.true_fob_kg();
        assert!((after_idle - before_idle).abs() < 1e-6, "with nothing commanded, total fuel must not change: {before_idle} -> {after_idle}");
    }

    #[test]
    fn trim_transfer_moves_the_published_fuel_cg_forward() {
        let mut live = FuelLive::new();
        live.load_tank(Tank::Trim, 6_000.0, 10.0);
        for feed in [Tank::Feed1, Tank::Feed2, Tank::Feed3, Tank::Feed4] {
            live.load_tank(feed, 0.0, 10.0);
        }
        for tank in [Tank::LeftOuter, Tank::LeftMid, Tank::LeftInner, Tank::RightInner, Tank::RightMid, Tank::RightOuter] {
            live.load_tank(tank, 0.0, 10.0);
        }
        let mut truth = Truth::default();
        truth.dt_s = 1.0;
        live.tick(&truth, &Faults::default());
        let before = live.fuel_cg_ft;
        for _ in 0..59 {
            live.tick(&truth, &Faults::default());
        }
        let after = live.fuel_cg_ft;
        assert!(live.tank_mass_kg(Tank::Trim) < 6_000.0, "the trim tank must actually have drained some mass forward");
        assert!(after > before + 0.01, "trim transfer must move the published fuel CG forward as it drains: {before} -> {after} ft");
    }

    #[test]
    fn a_tank_cannot_exceed_capacity_or_go_negative() {
        let mut live = FuelLive::new();
        let capacity_kg = TankShape::of(Tank::Feed1).capacity_m3() * crate::physics::fluids::jet_a_density_kg_m3(10.0);

        live.load_tank(Tank::Feed1, capacity_kg * 10.0, 10.0);
        let loaded = live.tank_mass_kg(Tank::Feed1);
        assert!(loaded <= capacity_kg + 1e-6, "load_tank must not let a tank hold more than its own capacity: {loaded} > {capacity_kg}");
        assert!(loaded.is_finite());

        live.load_tank(Tank::Feed1, -500.0, 10.0);
        assert_eq!(live.tank_mass_kg(Tank::Feed1), 0.0, "load_tank must floor a negative request at zero");

        let trim_capacity_kg = TankShape::of(Tank::Trim).capacity_m3() * crate::physics::fluids::jet_a_density_kg_m3(10.0);
        live.load_tank(Tank::Trim, trim_capacity_kg, 10.0);
        live.load_tank(Tank::Feed1, capacity_kg - 1.0, 10.0);
        let mut truth = Truth::default();
        truth.dt_s = 1.0;
        for _ in 0..50 {
            live.tick(&truth, &Faults::default());
        }
        let live_capacity_kg = TankShape::of(Tank::Feed1).capacity_m3() * crate::physics::fluids::jet_a_density_kg_m3(live.tank_temp_c(Tank::Feed1));
        assert!(
            live.tank_mass_kg(Tank::Feed1) <= live_capacity_kg * 1.001,
            "a transfer must not push a tank past its own capacity: {} > {live_capacity_kg}",
            live.tank_mass_kg(Tank::Feed1)
        );
        for &t in ALL_TANKS.iter() {
            assert!(live.tank_mass_kg(t) >= 0.0, "{t:?} must never go negative: {}", live.tank_mass_kg(t));
        }
    }

    #[test]
    fn a_real_reading_replaces_the_construction_seed_exactly_once() {
        let mut live = FuelLive::new();
        let seeded_feed1 = live.tank_mass_kg(Tank::Feed1);

        let mut real = [0.0; N_TANKS];
        for (i, &tank) in ALL_TANKS.iter().enumerate() {
            real[i] = TankShape::of(tank).capacity_gal * 0.1;
        }
        let mut truth = Truth::default();
        truth.dt_s = 1.0;
        truth.fuel_tank_quantity_gal = Some(real);
        live.tick(&truth, &Faults::default());
        let synced_feed1 = live.tank_mass_kg(Tank::Feed1);
        assert!(
            synced_feed1 < seeded_feed1 * 0.5,
            "a real reading must replace the seed, not add to it: seed {seeded_feed1} kg, after sync {synced_feed1} kg"
        );

        for (i, &tank) in ALL_TANKS.iter().enumerate() {
            real[i] = TankShape::of(tank).capacity_gal * 0.95;
        }
        truth.fuel_tank_quantity_gal = Some(real);
        live.tick(&truth, &Faults::default());
        let after_large_external_change = live.tank_mass_kg(Tank::Feed1);
        assert!(
            after_large_external_change - synced_feed1 > FuelLive::EXTERNAL_CHANGE_KG,
            "a real-tank jump past the ledger's own authority band (EFB refuelling, MSFS's native transfers) must be adopted, not ignored as a stale duplicate: {synced_feed1} kg -> {after_large_external_change} kg"
        );
    }

    #[test]
    fn apu_feed_pump_fault_fires_on_pump_degradation_and_not_on_a_cold_aircraft() {
        let mut live = FuelLive::new();
        let id = live.ids.apu_feed_pump;
        let healthy = run(&mut live, &Truth::default(), &Faults::default(), 1.0);
        assert_eq!(healthy.get("FUEL_APU_FEED_PUMP_FAULT"), Some(&0.0));

        let mut live = FuelLive::new();
        let out = run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 1.0)]), 1.0);
        assert_eq!(out.get("FUEL_APU_FEED_PUMP_FAULT"), Some(&1.0), "a degraded APU feed pump must raise its own fault reading");
    }

    #[test]
    fn apu_feed_valve_stuck_open_fires_and_not_when_commanded_open() {
        let mut live = FuelLive::new();
        let id = live.ids.apu_feed_valve;
        live.commands.apu_fuel_flow_kg_s = 5.0;
        for _ in 0..200 {
            live.tick(&Truth::default(), &Faults::default());
        }
        for _ in 0..5 {
            live.tick(&Truth::default(), &Faults::from_pairs([(id, 1.0)]));
        }
        let commanded_open = published(&live);
        assert_eq!(commanded_open.get("FUEL_APU_FEED_VALVE_FAULT"), Some(&0.0), "a valve stuck at the position it was already commanded to is not a disagreement");

        let mut live = FuelLive::new();
        live.commands.apu_fuel_flow_kg_s = 5.0;
        let id = live.ids.apu_feed_valve;
        for _ in 0..75 {
            live.tick(&Truth::default(), &Faults::default());
        }
        let armed = Faults::from_pairs([(id, 1.0)]);
        live.tick(&Truth::default(), &armed);
        live.commands.apu_fuel_flow_kg_s = 0.0;
        for _ in 0..50 {
            live.tick(&Truth::default(), &armed);
        }
        let out = published(&live);
        assert_eq!(out.get("FUEL_APU_FEED_VALVE_FAULT"), Some(&1.0), "a valve stuck open after the APU stops drawing fuel must raise a fault");

        let mut cold = FuelLive::new();
        let cold_out = run(&mut cold, &Truth::default(), &Faults::from_pairs([(id, 1.0)]), 5.0);
        assert_eq!(cold_out.get("FUEL_APU_FEED_VALVE_FAULT"), Some(&0.0), "the APU never commanded open, so the valve never moved and never disagrees");
    }

    #[test]
    fn engine_lp_valve_fault_fires_per_engine_on_a_command_disagreement() {
        let mut live = FuelLive::new();
        let truth = Truth { controls: crate::deep::live::Controls { engine_master_on: [true; 4], ..Default::default() }, ..Truth::default() };
        let id = live.ids.eng_lp_valve[1];
        for _ in 0..75 {
            live.tick(&truth, &Faults::default());
        }
        let armed = Faults::from_pairs([(id, 1.0)]);
        let off = Truth { controls: crate::deep::live::Controls { engine_master_on: [true, false, true, true], ..Default::default() }, ..Truth::default() };
        for _ in 0..50 {
            live.tick(&off, &armed);
        }
        let out = published(&live);
        assert_eq!(out.get("FUEL_ENG_LP_VALVE_FAULT:2"), Some(&1.0), "engine 2's own LP valve must disagree once its master is off but the valve stays open");
        assert_eq!(out.get("FUEL_ENG_LP_VALVE_FAULT:1"), Some(&0.0));
        assert_eq!(out.get("FUEL_ENG_LP_VALVE_FAULT:3"), Some(&0.0));

        let cold = run(&mut FuelLive::new(), &Truth::default(), &Faults::from_pairs([(id, 1.0)]), 5.0);
        assert_eq!(cold.get("FUEL_ENG_LP_VALVE_FAULT:2"), Some(&0.0), "master never commanded on, so the valve never moved and never disagrees");
    }

    #[test]
    fn feed_pump_faults_are_direct_readings_per_pump_and_silent_when_healthy() {
        let mut live = FuelLive::new();
        let main_id = live.ids.feed_main[2];
        let healthy = run(&mut live, &Truth::default(), &Faults::default(), 1.0);
        assert_eq!(healthy.get("FUEL_FEED_PUMP_FAULT:main_3"), Some(&0.0));

        let mut live = FuelLive::new();
        let out = run(&mut live, &Truth::default(), &Faults::from_pairs([(main_id, 1.0)]), 1.0);
        assert_eq!(out.get("FUEL_FEED_PUMP_FAULT:main_3"), Some(&1.0));
        assert_eq!(out.get("FUEL_FEED_PUMP_FAULT:stby_3"), Some(&0.0), "the standby pump is untouched");
    }

    #[test]
    fn a_faulted_wing_pump_is_read_directly_and_throttles_the_real_transfer_it_serves() {
        let mut clear = FuelLive::new();
        full_tanks(&mut clear, 10.0);
        let clear_out = run(&mut clear, &Truth::default(), &Faults::default(), 5.0);

        let mut faulted = FuelLive::new();
        full_tanks(&mut faulted, 10.0);
        let id = faulted.ids.wing_outer[0];
        let faulted_out = run(&mut faulted, &Truth::default(), &Faults::from_pairs([(id, 1.0)]), 5.0);
        assert_eq!(faulted_out.get("FUEL_WING_PUMP_FAULT:outer_left"), Some(&1.0));
        assert_eq!(clear_out.get("FUEL_WING_PUMP_FAULT:outer_left"), Some(&0.0));

        let mut healthy_mid = FuelLive::new();
        healthy_mid.load_tank(Tank::LeftInner, 0.0, 10.0);
        healthy_mid.load_tank(Tank::LeftMid, TankShape::of(Tank::LeftMid).capacity_m3() * REFERENCE_DENSITY_15C_KG_M3 * 0.5, 10.0);
        healthy_mid.load_tank(Tank::LeftOuter, TankShape::of(Tank::LeftOuter).capacity_m3() * REFERENCE_DENSITY_15C_KG_M3 * 0.5, 10.0);
        for t in [Tank::Feed1, Tank::Feed2, Tank::Feed3, Tank::Feed4, Tank::RightInner, Tank::RightMid, Tank::RightOuter, Tank::Trim] {
            healthy_mid.load_tank(t, 0.0, 10.0);
        }
        let before = healthy_mid.tank_mass_kg(Tank::LeftMid);
        let healthy_run = run(&mut healthy_mid, &Truth::default(), &Faults::default(), 30.0);
        let after_healthy = healthy_mid.tank_mass_kg(Tank::LeftMid);
        let _ = healthy_run;

        let mut faulted_mid = FuelLive::new();
        faulted_mid.load_tank(Tank::LeftInner, 0.0, 10.0);
        faulted_mid.load_tank(Tank::LeftMid, TankShape::of(Tank::LeftMid).capacity_m3() * REFERENCE_DENSITY_15C_KG_M3 * 0.5, 10.0);
        faulted_mid.load_tank(Tank::LeftOuter, TankShape::of(Tank::LeftOuter).capacity_m3() * REFERENCE_DENSITY_15C_KG_M3 * 0.5, 10.0);
        for t in [Tank::Feed1, Tank::Feed2, Tank::Feed3, Tank::Feed4, Tank::RightInner, Tank::RightMid, Tank::RightOuter, Tank::Trim] {
            faulted_mid.load_tank(t, 0.0, 10.0);
        }
        let fwd_id = faulted_mid.ids.wing_mid_fwd[0];
        let aft_id = faulted_mid.ids.wing_mid_aft[0];
        let _ = run(&mut faulted_mid, &Truth::default(), &Faults::from_pairs([(fwd_id, 1.0), (aft_id, 1.0)]), 30.0);
        let after_faulted = faulted_mid.tank_mass_kg(Tank::LeftMid);

        assert!(before > after_healthy, "the mid tank must actually drain when it is the active CG-transfer source");
        assert!(after_faulted > after_healthy, "both mid pumps faulted must slow the real transfer out of the mid tank: {after_faulted} kg drained vs {after_healthy} kg healthy");
    }

    #[test]
    fn leak_detector_self_fault_suppresses_the_aggregate_leak_alert_but_fires_its_own() {
        let mut live = FuelLive::new();
        full_tanks(&mut live, 10.0);
        let detector_fault_id = live.ids.leak_detector_fault;
        let leak_id = live.ids.tank_leak[1];
        let out = run(&mut live, &Truth::default(), &Faults::from_pairs([(detector_fault_id, 1.0), (leak_id, 1.0)]), 200.0);
        assert_eq!(out.get("FUEL_LEAK_DETECTOR_FAULT"), Some(&1.0));
        assert_eq!(out.get("FUEL_LEAK_DETECTED"), Some(&0.0), "the detector's own fault must suppress the real leak it can no longer see");

        let mut healthy_detector = FuelLive::new();
        full_tanks(&mut healthy_detector, 10.0);
        let healthy_out = run(&mut healthy_detector, &Truth::default(), &Faults::from_pairs([(leak_id, 1.0)]), 200.0);
        assert_eq!(healthy_out.get("FUEL_LEAK_DETECTED"), Some(&1.0), "the same leak must still be caught with the detector itself healthy");
    }

    #[test]
    fn per_engine_leak_detector_resolves_a_leak_to_the_one_feed_tank_it_is_on() {
        let mut live = FuelLive::new();
        for &tank in ALL_TANKS.iter() {
            live.load_tank(tank, 0.0, 10.0);
        }
        let feed3_capacity_kg = TankShape::of(Tank::Feed3).capacity_m3() * REFERENCE_DENSITY_15C_KG_M3;
        live.load_tank(Tank::Feed3, feed3_capacity_kg * 0.9, 10.0);
        let truth = Truth { engine_running: [true; 4], engine_fuel_flow_kg_s: [1.0; 4], ..Truth::default() };
        let leak_id = live.ids.tank_leak[5];
        let out = run(&mut live, &truth, &Faults::from_pairs([(leak_id, 1.0)]), 200.0);
        assert_eq!(out.get("FUEL_ENG_LEAK_DETECTED:3"), Some(&1.0), "a feed-3 tank-wall leak must resolve to engine 3");
        assert_eq!(out.get("FUEL_ENG_LEAK_DETECTED:1"), Some(&0.0));
        assert_eq!(out.get("FUEL_ENG_LEAK_DETECTED:2"), Some(&0.0));
        assert_eq!(out.get("FUEL_ENG_LEAK_DETECTED:4"), Some(&0.0));
    }

    #[test]
    fn per_engine_contamination_is_detected_from_the_existing_filter_water_fraction() {
        let mut live = FuelLive::new();
        let id = live.ids.filter_water[2];
        let healthy = run(&mut live, &Truth::default(), &Faults::default(), 1.0);
        for n in 1..=4 {
            assert_eq!(healthy.get(&format!("FUEL_ENG_CONTAMINATION_DETECTED:{n}")), Some(&0.0));
        }
        let mut live = FuelLive::new();
        let out = run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 0.3)]), 1.0);
        assert_eq!(out.get("FUEL_ENG_CONTAMINATION_DETECTED:3"), Some(&1.0));
        assert_eq!(out.get("FUEL_ENG_CONTAMINATION_DETECTED:1"), Some(&0.0));
    }

    #[test]
    fn wing_balance_uses_the_real_bridge_and_stays_silent_when_it_is_unknown() {
        let mut live = FuelLive::new();
        let unknown = run(&mut live, &Truth::default(), &Faults::default(), 1.0);
        assert_eq!(unknown.get("FUEL_WING_IMBALANCE_KNOWN"), Some(&0.0), "Truth::fuel_tank_quantity_gal is None by default");

        let mut balanced_gal = [1000.0; N_TANKS];
        balanced_gal[10] = 0.0;
        let mut live = FuelLive::new();
        let mut truth = Truth::default();
        truth.dt_s = 1.0;
        truth.fuel_tank_quantity_gal = Some(balanced_gal);
        let out = run(&mut live, &truth, &Faults::default(), 1.0);
        assert_eq!(out.get("FUEL_WING_IMBALANCE_KNOWN"), Some(&1.0));
        assert!(out["FUEL_WING_IMBALANCE_KG"] < 1.0, "five equal tanks per side must balance: {}", out["FUEL_WING_IMBALANCE_KG"]);

        let mut imbalanced_gal = [1000.0; N_TANKS];
        imbalanced_gal[10] = 0.0;
        imbalanced_gal[0] = 0.0;
        imbalanced_gal[2] = 0.0;
        let mut live = FuelLive::new();
        let mut truth = Truth::default();
        truth.dt_s = 1.0;
        truth.fuel_tank_quantity_gal = Some(imbalanced_gal);
        let out = run(&mut live, &truth, &Faults::default(), 1.0);
        assert!(out["FUEL_WING_IMBALANCE_KG"] > 3000.0, "draining one side's own outer tank must show a real imbalance: {}", out["FUEL_WING_IMBALANCE_KG"]);
    }

    #[test]
    fn fqdc_and_fqms_channel_faults_are_discrete_and_cost_the_gauging_chain_its_confidence() {
        let mut live = FuelLive::new();
        let id = live.ids.fqdc[0];
        let healthy = run(&mut live, &Truth::default(), &Faults::default(), 1.0);
        assert_eq!(healthy.get("FUEL_FQDC_FAULT:1"), Some(&0.0));
        assert_eq!(healthy.get("FUEL_FQMS_LOW_CONFIDENCE"), Some(&0.0));

        let mut live = FuelLive::new();
        let out = run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 1.0)]), 1.0);
        assert_eq!(out.get("FUEL_FQDC_FAULT:1"), Some(&1.0));
        assert_eq!(out.get("FUEL_FQMS_LOW_CONFIDENCE"), Some(&1.0), "an FQDC channel fault must cost the gauging chain its confidence, the same real consequence a probe fault already produces");
    }

    #[test]
    fn transfer_sequencer_and_wb_backup_faults_are_discrete_and_silent_when_healthy() {
        let mut live = FuelLive::new();
        let norm_id = live.ids.seq_norm;
        let altn_id = live.ids.seq_altn;
        let wb_id = live.ids.wb_backup;
        let healthy = run(&mut live, &Truth::default(), &Faults::default(), 1.0);
        assert_eq!(healthy.get("FUEL_TRANSFER_SEQUENCER_FAULT:norm"), Some(&0.0));
        assert_eq!(healthy.get("FUEL_TRANSFER_SEQUENCER_FAULT:altn"), Some(&0.0));
        assert_eq!(healthy.get("FUEL_WB_BACKUP_FAULT"), Some(&0.0));

        let mut live = FuelLive::new();
        let out = run(&mut live, &Truth::default(), &Faults::from_pairs([(norm_id, 1.0), (wb_id, 1.0)]), 1.0);
        assert_eq!(out.get("FUEL_TRANSFER_SEQUENCER_FAULT:norm"), Some(&1.0));
        assert_eq!(out.get("FUEL_TRANSFER_SEQUENCER_FAULT:altn"), Some(&0.0));
        assert_eq!(out.get("FUEL_WB_BACKUP_FAULT"), Some(&1.0));
        let _ = altn_id;
    }

    #[test]
    fn each_crossfeed_valve_fault_is_wired_individually_and_the_others_stay_quiet() {
        let mut live = FuelLive::new();
        full_tanks(&mut live, 10.0);
        let id = live.ids.crossfeed[1];
        let healthy = run(&mut live, &crossfeed_selected_truth(), &Faults::default(), 1.0);
        assert_eq!(healthy.get("FUEL_CROSSFEED_VALVE_FAULT:2"), Some(&0.0));

        let mut live = FuelLive::new();
        full_tanks(&mut live, 10.0);
        let out = run(&mut live, &crossfeed_selected_truth(), &Faults::from_pairs([(id, 1.0)]), 1.0);
        assert_eq!(out.get("FUEL_CROSSFEED_VALVE_FAULT:2"), Some(&1.0));
        assert_eq!(out.get("FUEL_CROSSFEED_VALVE_FAULT:1"), Some(&0.0));
        assert_eq!(out.get("FUEL_CROSSFEED_VALVE_FAULT:3"), Some(&0.0));
        assert_eq!(out.get("FUEL_CROSSFEED_VALVE_FAULT:4"), Some(&0.0));
    }

    #[test]
    fn outer_transfer_fault_publishes_from_the_already_registered_outer_valves() {
        let mut live = FuelLive::new();
        let id = live.ids.outer_xfer[1];
        let healthy = run(&mut live, &Truth::default(), &Faults::default(), 1.0);
        assert_eq!(healthy.get("FUEL_OUTER_TRANSFER_FAULT"), Some(&0.0));
        let mut live = FuelLive::new();
        let out = run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 1.0)]), 1.0);
        assert_eq!(out.get("FUEL_OUTER_TRANSFER_FAULT"), Some(&1.0));
    }

    #[test]
    fn a_jettison_valve_stuck_open_after_deselection_is_flagged_not_closed() {
        let truth = jettison_selected_truth();
        let mut live = FuelLive::new();
        full_tanks(&mut live, 10.0);
        let id = live.ids.jettison_valve[0];
        for _ in 0..30 {
            live.tick(&truth, &Faults::default());
        }
        let armed = Faults::from_pairs([(id, 1.0)]);
        live.tick(&truth, &armed);
        let deselected = Truth { controls: crate::deep::live::Controls { jettison_armed: false, jettison_valve_selected: [false; 2], ..Default::default() }, ..Truth::default() };
        for _ in 0..30 {
            live.tick(&deselected, &armed);
        }
        let out = published(&live);
        assert_eq!(out.get("FUEL_JETTISON_VALVE_NOT_CLOSED:1"), Some(&1.0));

        let mut healthy = FuelLive::new();
        full_tanks(&mut healthy, 10.0);
        for _ in 0..30 {
            healthy.tick(&truth, &Faults::default());
        }
        for _ in 0..30 {
            healthy.tick(&deselected, &Faults::default());
        }
        let healthy_out = published(&healthy);
        assert_eq!(healthy_out.get("FUEL_JETTISON_VALVE_NOT_CLOSED:1"), Some(&0.0), "a healthy valve closes on deselection and must not be flagged");
    }
}
