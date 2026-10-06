use std::f64::consts::PI;

use crate::deep::api::Registry;
use crate::deep::live::{Faults, Truth};

use super::airflow_control::bleed_valve::{self, BleedValve, BleedValveFaults, BleedValveSpec, BleedValveState, HP_HANDLING_BLEED, IP_HANDLING_BLEED};
use super::airflow_control::vsv::{Vsv, VsvFaults, VsvState};
use super::eec::{ActiveChannel, Eec, EecFaults, EecState, SensorFaults, PARAMS};
use super::fuel::filter::{self, FilterFaults, FilterState};
use super::fuel::strainer::{self, StrainerFaults, StrainerState};
use super::fuel::flow_transmitter::{FlowReading, FlowTransmitter, PickoffFaults};
use super::fuel::fmu::{FmuFaults, FmuState, FuelMeteringUnit};
use super::fuel::hp_pump::{self, HpPumpFaults, HpPumpState};
use super::fuel::lp_pump::{self, LpPumpFaults, LpPumpState};
use super::fuel::manifold::{self, ManifoldFaults, ManifoldState, NUM_GROUPS};
use super::fuel::shutoff_valve::{ShutoffValve, ShutoffValveFaults};
use super::ignition::{self, IgnitionFaults, IgnitionState};
use super::nacelle::anti_ice::{AntiIceState, AntiIceValve, AntiIceValveFaults};
use super::nacelle::fire_detection::{self, FireZoneReading, LoopFaults};
use super::nacelle::ventilation::{self, VentilationFaults, VentilationState};
use super::rotor_dynamics::bearing::BearingFaults;
use super::rotor_dynamics::bearings::{BearingState, EngineBearingFaults, EngineBearings};
use super::rotor_dynamics::imbalance::{ImbalanceFaults, SpoolVibration, VibrationState, FAN_SPEC, HP_SPEC, IP_SPEC};
use super::starting::air_valve::{AirValve, AirValveFaults};
use super::starting::duty_cycle::DutyCycleHeat;
use super::starting::turbine::{AirTurbineStarter, AtsFaults, AtsState, FREE_SPEED_FRAC, N3_DESIGN_RPM};
use super::thrust_reverser::{LockFaults, ReverserFaults, ReverserState, ThrustReverser};

use crate::physics::engine as gas_path_model;
use crate::physics::engine::free_engine::{self, FreeEngine};

const N_ENGINES: usize = 4;
const N_EEC_PARAMS: usize = 5;
const N_SPOOLS: usize = 3;
const N_BEARINGS: usize = 5;
const N_FIRE_ZONES: usize = 2;
const N_IGN_CHAINS: usize = 2;
const N_HANDLING_BLEEDS: usize = 2;
const N_REV_LOCKS: usize = 3;

const N1_DESIGN_RPM: f64 = 2900.0;
const N2_DESIGN_RPM: f64 = 8300.0;

const ISA_SL_TEMP_K: f64 = 288.15;

const R_AIR: f64 = 287.05;

const FEED_BOOST_RISE_PA: f64 = 50_000.0;

const METERING_TOLERANCE: f64 = 0.10;
const THRUST_ABNORMAL_TOLERANCE: f64 = 0.33;
const FF_DISAGREE_KG_S: f64 = 1.0;
const NOZZLE_IMBALANCE_SEVERITY: f64 = 0.05;
const SOV_DISAGREE_TOLERANCE: f64 = 0.05;
const HP_PUMP_LOW_FLOW_TOLERANCE: f64 = 0.10;
const METERING_DISAGREE_CONFIRM_S: f64 = 2.0;

const IGNITION_MIN_BUS_V: f64 = 0.5 * ignition::V_BUS_V;

const START_DESIGN_DP_PA: f64 = 207_000.0;
const START_VALVE_DISAGREE_TOLERANCE: f64 = 0.05;
const START_VALVE_SETTLE_S: f64 = 4.5;
const STARTER_OVERRUN_MARGIN: f64 = 1.01;

const MAX_RIGGING_BIAS_DEG: f64 = 20.0;

const CORE_DESIGN_FLOW_KG_S: f64 = 124.0;
const HANDLING_BLEED_DISAGREE_TOLERANCE: f64 = 0.15;

const ANTI_ICE_DISAGREE_TOLERANCE: f64 = 0.05;

const REVERSER_DISAGREE_TOLERANCE: f64 = 0.05;
const HYDRAULIC_NOMINAL_PA: f64 = 34_474_000.0;

const EEC_BACKUP_PROBE_DISAGREE_C: f64 = 5.0;
const EEC_BACKUP_PROBE_MAX_BIAS_C: f64 = 10.0;

const THR_LEVER_DISAGREE_TOLERANCE_DEG: f64 = 0.05;
const THR_LEVER_CHANNEL_B_MAX_BIAS_DEG: f64 = 2.0;

pub const UNCONSUMED_ATA: [u16; 0] = [];

#[derive(Clone, Copy, Debug)]
pub struct EngineAccessoryCommands {
    pub engine_tgt_k: Option<[f64; N_ENGINES]>,
    pub wf_command_kg_s: Option<[f64; N_ENGINES]>,
    pub fuel_inlet_k: Option<f64>,
    pub reverser_deploy_commanded: [bool; 2],
}

impl Default for EngineAccessoryCommands {
    fn default() -> Self {
        Self { engine_tgt_k: None, wf_command_kg_s: None, fuel_inlet_k: None, reverser_deploy_commanded: [false; 2] }
    }
}

const FEED_TANK_NUMBER: [u32; N_ENGINES] = [2, 5, 6, 9];

const REVERSER_ENGINES: [usize; 2] = [2, 3];

const FAILURE_OIL_PUMP: [u64; N_ENGINES] = [79_000, 79_001, 79_002, 79_003];

const SPOOL_KEYS: [&str; N_SPOOLS] = ["N1", "N2", "N3"];
const SPOOL_COMPONENT_KEYS: [&str; N_SPOOLS] = ["fan", "ip", "hp"];
const BEARING_KEYS: [&str; N_BEARINGS] = ["fan_front", "ip_front", "hp_turbine", "ip_turbine", "lp_turbine_rear"];
const BEARING_VAR_SPOOL: [&str; N_BEARINGS] = ["N1", "N2", "N3", "N2", "N1"];
const BLEED_KEYS: [&str; N_HANDLING_BLEEDS] = ["IP", "HP"];
const FIRE_ZONE_KEYS: [&str; N_FIRE_ZONES] = ["CORE", "FAN"];
const IGN_CHAIN_KEYS: [&str; N_IGN_CHAINS] = ["a", "b"];
const REV_LOCK_KEYS: [&str; N_REV_LOCKS] = ["lock_a", "lock_b", "lock_c"];

fn omega_rad_s(rpm: f64) -> f64 {
    rpm * PI / 30.0
}

fn fid(reg: &Registry, component: &str, fragment: &str) -> u64 {
    let mut found = reg.failures.iter().filter(|f| f.component == component && f.model_field.contains(fragment));
    let first = found.next().unwrap_or_else(|| panic!("no failure on {component} whose model_field contains {fragment:?}"));
    assert!(found.next().is_none(), "more than one failure on {component} matches {fragment:?}");
    first.id
}

struct FuelIds {
    lp_wear: u64,
    lp_inlet_restriction: u64,
    strainer_clog: u64,
    filter_clog: u64,
    filter_monitor_fault: u64,
    hp_wear: u64,
    hp_starvation: u64,
    fmu_sticking: u64,
    fmu_spill_open: u64,
    fmu_spill_closed: u64,
    sov_stuck: u64,
    ft_a_bias: u64,
    ft_a_frozen: u64,
    ft_b_bias: u64,
    ft_b_frozen: u64,
    nozzle: [u64; NUM_GROUPS],
    eec_channel_a: u64,
    eec_channel_b: u64,
    eec_sensor_a: [u64; N_EEC_PARAMS],
    eec_sensor_b: [u64; N_EEC_PARAMS],
    eec_backup_probe_bias: u64,
    tla_channel_b_bias: u64,
}

impl FuelIds {
    fn resolve(reg: &Registry, eng: usize) -> Self {
        let n = eng + 1;
        let lp = format!("73_fuel.lp_pump_{n}");
        let strainer_id = format!("73_fuel.strainer_{n}");
        let filter_id = format!("73_fuel.filter_{n}");
        let hp = format!("73_fuel.hp_pump_{n}");
        let fmu = format!("73_fuel.fmu_{n}");
        let sov = format!("73_fuel.hp_sov_{n}");
        let ft = format!("73_fuel.flow_transmitter_{n}");
        let man = format!("73_fuel.manifold_{n}");
        let eec = format!("73_eec.channels_{n}");
        let tla = format!("76_ctl.tla_channel_b_{n}");

        let mut nozzle = [0u64; NUM_GROUPS];
        for (g, slot) in nozzle.iter_mut().enumerate() {
            *slot = fid(reg, &man, &format!("group_blockage[{g}]"));
        }
        let mut eec_sensor_a = [0u64; N_EEC_PARAMS];
        let mut eec_sensor_b = [0u64; N_EEC_PARAMS];
        for (i, p) in ["N1", "N2", "N3", "TGT", "P30"].iter().enumerate() {
            eec_sensor_a[i] = fid(reg, &eec, &format!("sensor_a[{p}]"));
            eec_sensor_b[i] = fid(reg, &eec, &format!("sensor_b[{p}]"));
        }

        Self {
            lp_wear: fid(reg, &lp, "LpPumpFaults.wear"),
            lp_inlet_restriction: fid(reg, &lp, "inlet_restriction"),
            strainer_clog: fid(reg, &strainer_id, "StrainerFaults.clog"),
            filter_clog: fid(reg, &filter_id, "FilterFaults.clog"),
            filter_monitor_fault: fid(reg, &filter_id, "monitor_fault"),
            hp_wear: fid(reg, &hp, "HpPumpFaults.wear"),
            hp_starvation: fid(reg, &hp, "inlet_starvation"),
            fmu_sticking: fid(reg, &fmu, "valve_sticking"),
            fmu_spill_open: fid(reg, &fmu, "spill_stuck_open"),
            fmu_spill_closed: fid(reg, &fmu, "spill_stuck_closed"),
            sov_stuck: fid(reg, &sov, "ShutoffValveFaults.stuck"),
            ft_a_bias: fid(reg, &ft, "bias_frac_of_design (channel A)"),
            ft_a_frozen: fid(reg, &ft, "frozen (channel A)"),
            ft_b_bias: fid(reg, &ft, "bias_frac_of_design (channel B)"),
            ft_b_frozen: fid(reg, &ft, "frozen (channel B)"),
            nozzle,
            eec_channel_a: fid(reg, &eec, "channel_a_fault"),
            eec_channel_b: fid(reg, &eec, "channel_b_fault"),
            eec_sensor_a,
            eec_sensor_b,
            eec_backup_probe_bias: fid(reg, &eec, "backup_oil_temp_probe_bias"),
            tla_channel_b_bias: fid(reg, &tla, "TlaChannelBFaults.bias"),
        }
    }

    fn all(&self) -> Vec<u64> {
        let mut v = vec![
            self.lp_wear,
            self.lp_inlet_restriction,
            self.strainer_clog,
            self.filter_clog,
            self.filter_monitor_fault,
            self.hp_wear,
            self.hp_starvation,
            self.fmu_sticking,
            self.fmu_spill_open,
            self.fmu_spill_closed,
            self.sov_stuck,
            self.ft_a_bias,
            self.ft_a_frozen,
            self.ft_b_bias,
            self.ft_b_frozen,
            self.eec_channel_a,
            self.eec_channel_b,
        ];
        v.extend(self.nozzle);
        v.extend(self.eec_sensor_a);
        v.extend(self.eec_sensor_b);
        v.push(self.eec_backup_probe_bias);
        v.push(self.tla_channel_b_bias);
        v
    }
}

struct IgnitionIds {
    exciter: [u64; N_IGN_CHAINS],
    igniter: [u64; N_IGN_CHAINS],
}

impl IgnitionIds {
    fn resolve(reg: &Registry, eng: usize) -> Self {
        let n = eng + 1;
        Self {
            exciter: std::array::from_fn(|c| fid(reg, &format!("74_ignition.exciter_{}_{n}", IGN_CHAIN_KEYS[c]), "IgnitionFaults.exciter_")),
            igniter: std::array::from_fn(|c| fid(reg, &format!("74_ignition.igniter_{}_{n}", IGN_CHAIN_KEYS[c]), "IgnitionFaults.igniter_")),
        }
    }

    fn all(&self) -> Vec<u64> {
        let mut v = self.exciter.to_vec();
        v.extend(self.igniter);
        v
    }
}

struct StartIds {
    sav_stuck: u64,
    clutch_engage: u64,
    clutch_disengage: u64,
}

impl StartIds {
    fn resolve(reg: &Registry, eng: usize) -> Self {
        let n = eng + 1;
        let ats = format!("80_start.ats_{n}");
        Self {
            sav_stuck: fid(reg, &format!("80_start.air_valve_{n}"), "AirValveFaults.stuck"),
            clutch_engage: fid(reg, &ats, "clutch_fails_to_engage"),
            clutch_disengage: fid(reg, &ats, "clutch_fails_to_disengage"),
        }
    }

    fn all(&self) -> Vec<u64> {
        vec![self.sav_stuck, self.clutch_engage, self.clutch_disengage]
    }
}

struct AirflowIds {
    vsv_jam: u64,
    vsv_rigging: u64,
    bleed_jam: [u64; N_HANDLING_BLEEDS],
}

impl AirflowIds {
    fn resolve(reg: &Registry, eng: usize) -> Self {
        let n = eng + 1;
        let vsv = format!("72_air.vsv_{n}");
        Self {
            vsv_jam: fid(reg, &vsv, "VsvFaults.jam"),
            vsv_rigging: fid(reg, &vsv, "VsvFaults.rigging_bias_deg"),
            bleed_jam: std::array::from_fn(|i| fid(reg, &format!("75_air.{}_handling_bleed_{n}", BLEED_KEYS[i].to_lowercase()), "BleedValveFaults.jam")),
        }
    }

    fn all(&self) -> Vec<u64> {
        let mut v = vec![self.vsv_jam, self.vsv_rigging];
        v.extend(self.bleed_jam);
        v
    }
}

const FAILURE_COMPRESSOR_STALL: [u64; 4] = [72_004, 72_005, 72_006, 72_007];
const FAILURE_HP_COMPRESSOR_DESTRUCTION: [u64; 4] = [72_012, 72_013, 72_014, 72_015];
const HP_SPOOL_INDEX: usize = 2;
const STALL_LOSS_FRACTION_AT_FULL_SEVERITY: f64 = 0.7;
const STALL_VIBRATION_PROXY_AT_FULL_SEVERITY: f64 = 0.3;
const CASE_CONTAINED_RELEASE_FRAC: f64 = 0.5;
const DEBRIS_FUEL_BREACH_AREA_M2: f64 = 2.0e-5;
const DEBRIS_OIL_BREACH_LEAK_FRAC: f64 = 1.0;
const ORIFICE_CD: f64 = 0.6;

struct RotorIds {
    imbalance: [[u64; 3]; N_SPOOLS],
    bearing: [[u64; 4]; N_BEARINGS],
}

impl RotorIds {
    fn resolve(reg: &Registry, eng: usize) -> Self {
        let n = eng + 1;
        const CAUSES: [&str; 3] = ["blade_loss_frac", "ice_frac", "bird_strike_frac"];
        const DEFECTS: [&str; 4] = ["outer_race_spall", "inner_race_spall", "rolling_element_spall", "cage_wear"];
        Self {
            imbalance: std::array::from_fn(|s| {
                let rotor = format!("77_vib.{}_rotor_{n}", SPOOL_COMPONENT_KEYS[s]);
                std::array::from_fn(|c| fid(reg, &rotor, CAUSES[c]))
            }),
            bearing: std::array::from_fn(|b| {
                let bearing = format!("77_vib.bearing_{}_{n}", BEARING_KEYS[b]);
                std::array::from_fn(|d| fid(reg, &bearing, DEFECTS[d]))
            }),
        }
    }

    fn all(&self) -> Vec<u64> {
        let mut v = Vec::new();
        for s in self.imbalance {
            v.extend(s);
        }
        for b in self.bearing {
            v.extend(b);
        }
        v
    }
}

struct ReverserIds {
    lock_hold: [u64; N_REV_LOCKS],
    lock_jam: [u64; N_REV_LOCKS],
    actuator_jam: u64,
    control_fault: u64,
}

impl ReverserIds {
    fn resolve(reg: &Registry, eng_number: usize) -> Self {
        let id = format!("78_rev.reverser_{eng_number}");
        Self {
            lock_hold: std::array::from_fn(|l| fid(reg, &id, &format!("fails_to_hold ({})", REV_LOCK_KEYS[l]))),
            lock_jam: std::array::from_fn(|l| fid(reg, &id, &format!("jam ({})", REV_LOCK_KEYS[l]))),
            actuator_jam: fid(reg, &id, "ReverserFaults.actuator_jam"),
            control_fault: fid(reg, &id, "ReverserFaults.control_fault"),
        }
    }

    fn all(&self) -> Vec<u64> {
        let mut v = self.lock_hold.to_vec();
        v.extend(self.lock_jam);
        v.push(self.actuator_jam);
        v.push(self.control_fault);
        v
    }
}

struct NacelleIds {
    anti_ice_stuck: u64,
    scoop: u64,
    eductor: u64,
    fire: [[u64; 4]; N_FIRE_ZONES],
}

impl NacelleIds {
    fn resolve(reg: &Registry, eng: usize) -> Self {
        let n = eng + 1;
        let vent = format!("71_pwr.nacelle_ventilation_{n}");
        const LOOP_FRAGMENTS: [&str; 4] = ["fails_to_detect (loop A", "false_trip (loop A", "fails_to_detect (loop B", "false_trip (loop B"];
        Self {
            anti_ice_stuck: fid(reg, &format!("30_ice.nacelle_valve_{n}"), "AntiIceValveFaults.stuck"),
            scoop: fid(reg, &vent, "scoop_blockage"),
            eductor: fid(reg, &vent, "eductor_blockage"),
            fire: std::array::from_fn(|z| {
                let zone = format!("26_fire.zone_{}_{n}", FIRE_ZONE_KEYS[z].to_lowercase());
                std::array::from_fn(|l| fid(reg, &zone, LOOP_FRAGMENTS[l]))
            }),
        }
    }

    fn all(&self) -> Vec<u64> {
        let mut v = vec![self.anti_ice_stuck, self.scoop, self.eductor];
        for z in self.fire {
            v.extend(z);
        }
        v
    }
}

struct OilIds {
    leak: u64,
    pump_fault: u64,
}

impl OilIds {
    fn resolve(reg: &Registry, eng: usize) -> Self {
        let n = eng + 1;
        Self {
            leak: fid(reg, &format!("79_oil.leak_{n}"), "OilFaults.leak"),
            pump_fault: fid(reg, &format!("79_oil.pump_{n}"), "pump_fraction"),
        }
    }

    fn all(&self) -> Vec<u64> {
        vec![self.leak, self.pump_fault]
    }
}

struct SeizureIds {
    frac: u64,
}

impl SeizureIds {
    fn resolve(reg: &Registry, eng: usize) -> Self {
        let n = eng + 1;
        Self { frac: fid(reg, &format!("72_eng.bearing_seizure_{n}"), "bearing_seizure_frac") }
    }

    fn all(&self) -> Vec<u64> {
        vec![self.frac]
    }
}

struct TurbineDamageIds {
    damage: u64,
    release: u64,
}

impl TurbineDamageIds {
    fn resolve(reg: &Registry, eng: usize) -> Self {
        let n = eng + 1;
        let id = format!("72_turb.blade_damage_{n}");
        Self { damage: fid(reg, &id, "damage_frac"), release: fid(reg, &id, "release_frac") }
    }

    fn all(&self) -> Vec<u64> {
        vec![self.damage, self.release]
    }
}

struct ChainNames {
    filter_impending_bypass: String,
    filter_bypassed: String,
    filter_dp_pa: String,
    filter_monitor_fault: String,
    strainer_clogged: String,
    oil_filter_bypassed: String,
    oil_system_contamination: String,
    hp_pump_low_flow: String,
    hp_pump_flow: String,
    lp_pump_outlet_pa: String,
    lp_pump_cavitating: String,
    fmu_fault: String,
    thrust_abnormal: String,
    fmu_metered: String,
    fmu_dp_pa: String,
    wf_command: String,
    sov_disagree: String,
    sov_position: String,
    ff_channel_disagree: String,
    ff_disagree: String,
    ff_indicated: String,
    ff_true: String,
    nozzle_imbalance: String,
    nozzle_hot_streak: String,
    nozzle_group_flow_dev: [String; NUM_GROUPS],
    combustor_rise_fraction: String,
    manifold_gauge_pa: String,
    eec_channel_fault: String,
    eec_no_valid_channel: String,
    eec_sensor_disagree: String,
    eec_selected: [String; N_EEC_PARAMS],
    eec_disagree: [String; N_EEC_PARAMS],
    eec_no_data: [String; N_EEC_PARAMS],
    eec_backup_probe_fault: String,
    tla_deg: String,
    tla_channel_b_deg: String,
    tla_channel_b_disagree: String,
    ign_spark_rate: [String; N_IGN_CHAINS],
    ign_powered: String,
    no_ignition_available: String,
    start_valve_position: String,
    start_valve_disagree: String,
    starter_torque: String,
    starter_rotor_rpm: String,
    starter_disengage_fault: String,
    starter_disintegrated: String,
    starter_overheat: String,
    starter_housing_rise_k: String,
    vsv_angle: String,
    vsv_schedule_error: String,
    vsv_stall_margin: String,
    bleed_position: [String; N_HANDLING_BLEEDS],
    bleed_disagree: [String; N_HANDLING_BLEEDS],
    bleed_flow: [String; N_HANDLING_BLEEDS],
    bleed_stall_margin: [String; N_HANDLING_BLEEDS],
    vib_index: [String; N_SPOOLS],
    bearing_amplitude: [String; N_BEARINGS],
    bearing_chip: [String; N_BEARINGS],
    bearing_debris: [String; N_BEARINGS],
    anti_ice_position: String,
    anti_ice_disagree: String,
    anti_ice_lip_temp_k: String,
    anti_ice_bleed_flow: String,
    nacelle_vent_flow: String,
    nacelle_vapour_risk: String,
    fire_loop_disagree: [String; N_FIRE_ZONES],
    fire_confirmed: [String; N_FIRE_ZONES],
    read_feed_tank_temp_c: String,
    read_start_duct_pa: String,
    read_nacelle_zone_temp_c: String,
    read_nacelle_fire_zone_temp_c: String,
    read_env_bird_fan_frac: String,
    read_env_bird_core_frac: String,
    gp_egt_delta_c: String,
    gp_oil_temp_delta_c: String,
    gp_oil_press_delta_psi: String,
    gp_oil_quantity_delta_frac: String,
    gp_stall_margin_loss_pct: String,
    gp_n2_capability_loss_pct: String,
    gp_n3_capability_loss_pct: String,
    gp_thrust_loss_pct: String,
    gp_surge: String,
    gp_oil_temp_c: String,
    gp_oil_press_psi: String,
    gp_oil_quantity_frac: String,
    gp_oil_supply_c: String,
    gp_oil_filter_bypassed: String,
    gp_oil_relief_open: String,
    gp_fuel_heat_w: String,
    gp_fuel_out_c: String,
    gp_hot_section_soak_c: String,
    gp_egt_shadow_c: String,
    case_breach: String,
    nacelle_fuel_leak: String,
    phys_n1: String,
    phys_n2: String,
    phys_n3: String,
    phys_egt: String,
    phys_ff: String,
    phys_lit: String,
    phys_thrust: String,
    phys_valid: String,
    gp_ip_port_pressure_pa: String,
    gp_ip_port_temp_k: String,
    bearing_seizure_frac: String,
}

impl ChainNames {
    fn new(eng: usize) -> Self {
        let n = eng + 1;
        let v = |suffix: &str| format!("A32NX_ENG_{n}_{suffix}");
        Self {
            filter_impending_bypass: v("FUEL_FILTER_IMPENDING_BYPASS"),
            filter_bypassed: v("FUEL_FILTER_BYPASSED"),
            filter_dp_pa: v("FUEL_FILTER_DP_PA"),
            filter_monitor_fault: v("FUEL_FILTER_MONITOR_FAULT"),
            strainer_clogged: v("FUEL_STRAINER_CLOGGED"),
            oil_filter_bypassed: v("OIL_FILTER_BYPASSED"),
            oil_system_contamination: v("OIL_SYSTEM_CONTAMINATION"),
            hp_pump_low_flow: v("HP_PUMP_LOW_FLOW"),
            hp_pump_flow: v("HP_PUMP_FLOW_KG_S"),
            lp_pump_outlet_pa: v("LP_PUMP_OUTLET_PA"),
            lp_pump_cavitating: v("LP_PUMP_CAVITATING"),
            fmu_fault: v("FMU_FAULT"),
            thrust_abnormal: v("THRUST_ABNORMAL"),
            fmu_metered: v("FMU_METERED_KG_S"),
            fmu_dp_pa: v("FMU_DP_PA"),
            wf_command: v("WF_COMMAND_KG_S"),
            sov_disagree: v("HP_SOV_DISAGREE"),
            sov_position: v("HP_SOV_POSITION"),
            ff_channel_disagree: v("FF_CHANNEL_DISAGREE"),
            ff_disagree: v("FF_DISAGREE"),
            ff_indicated: v("FF_INDICATED_KG_S"),
            ff_true: v("FF_TRUE_KG_S"),
            nozzle_imbalance: v("NOZZLE_IMBALANCE"),
            nozzle_hot_streak: v("NOZZLE_HOT_STREAK"),
            nozzle_group_flow_dev: std::array::from_fn(|k| v(&format!("NOZZLE_GROUP_{}_FLOW_DEV", k + 1))),
            combustor_rise_fraction: v("COMBUSTOR_RISE_FRACTION"),
            manifold_gauge_pa: v("MANIFOLD_GAUGE_PA"),
            eec_channel_fault: v("EEC_CHANNEL_FAULT"),
            eec_no_valid_channel: v("EEC_NO_VALID_CHANNEL"),
            eec_sensor_disagree: v("EEC_SENSOR_DISAGREE"),
            eec_selected: std::array::from_fn(|i| v(&format!("EEC_{}_SELECTED", eec_param_key(i)))),
            eec_disagree: std::array::from_fn(|i| v(&format!("EEC_{}_DISAGREE", eec_param_key(i)))),
            eec_no_data: std::array::from_fn(|i| v(&format!("EEC_{}_NO_DATA", eec_param_key(i)))),
            eec_backup_probe_fault: v("EEC_MAINTENANCE_FAULT"),
            tla_deg: v("TLA_DEG"),
            tla_channel_b_deg: v("TLA_CHANNEL_B_DEG"),
            tla_channel_b_disagree: v("THR_LEVER_DISAGREE"),
            ign_spark_rate: std::array::from_fn(|c| v(&format!("IGN_{}_SPARK_RATE_HZ", IGN_CHAIN_KEYS[c].to_uppercase()))),
            ign_powered: v("IGN_POWERED"),
            no_ignition_available: v("NO_IGNITION_AVAILABLE"),
            start_valve_position: v("START_VALVE_POSITION"),
            start_valve_disagree: v("START_VALVE_DISAGREE"),
            starter_torque: v("STARTER_TORQUE_NM"),
            starter_rotor_rpm: v("STARTER_ROTOR_RPM"),
            starter_disengage_fault: v("STARTER_DISENGAGE_FAULT"),
            starter_disintegrated: v("STARTER_DISINTEGRATED"),
            starter_overheat: v("STARTER_OVERHEAT"),
            starter_housing_rise_k: v("STARTER_HOUSING_RISE_K"),
            vsv_angle: v("VSV_ANGLE_DEG"),
            vsv_schedule_error: v("VSV_SCHEDULE_ERROR_DEG"),
            vsv_stall_margin: v("VSV_STALL_MARGIN_DELTA_PCT"),
            bleed_position: std::array::from_fn(|i| v(&format!("{}_HANDLING_BLEED_POSITION", BLEED_KEYS[i]))),
            bleed_disagree: std::array::from_fn(|i| v(&format!("{}_HANDLING_BLEED_DISAGREE", BLEED_KEYS[i]))),
            bleed_flow: std::array::from_fn(|i| v(&format!("{}_HANDLING_BLEED_KG_S", BLEED_KEYS[i]))),
            bleed_stall_margin: std::array::from_fn(|i| v(&format!("{}_HANDLING_BLEED_STALL_MARGIN_PCT", BLEED_KEYS[i]))),
            vib_index: std::array::from_fn(|s| v(&format!("{}_VIB_INDEX", SPOOL_KEYS[s]))),
            bearing_amplitude: std::array::from_fn(|b| v(&format!("{}_{}_DEFECT_AMPLITUDE_MM_S", BEARING_VAR_SPOOL[b], BEARING_KEYS[b].to_uppercase()))),
            bearing_chip: std::array::from_fn(|b| v(&format!("{}_CHIP_DETECTED", BEARING_KEYS[b].to_uppercase()))),
            bearing_debris: std::array::from_fn(|b| v(&format!("{}_DEBRIS_G", BEARING_KEYS[b].to_uppercase()))),
            anti_ice_position: v("ANTI_ICE_POSITION"),
            anti_ice_disagree: v("ANTI_ICE_DISAGREE"),
            anti_ice_lip_temp_k: v("ANTI_ICE_LIP_TEMP_K"),
            anti_ice_bleed_flow: v("ANTI_ICE_BLEED_KG_S"),
            nacelle_vent_flow: v("NACELLE_VENT_FLOW_KG_S"),
            nacelle_vapour_risk: v("NACELLE_VAPOUR_RISK"),
            fire_loop_disagree: std::array::from_fn(|z| v(&format!("{}_FIRE_LOOP_DISAGREE", FIRE_ZONE_KEYS[z]))),
            fire_confirmed: std::array::from_fn(|z| v(&format!("{}_FIRE_CONFIRMED", FIRE_ZONE_KEYS[z]))),
            read_feed_tank_temp_c: format!("FUEL_TANK_TEMP_C:{}", FEED_TANK_NUMBER[eng]),
            read_start_duct_pa: format!("DEEP_PNEU_ENG_{n}_START_DUCT_PRESSURE_PA"),
            read_nacelle_zone_temp_c: format!("THERMAL_ZONE_NACELLECOWL{n}_TEMPERATURE_C"),
            read_nacelle_fire_zone_temp_c: format!("FIRE_ZONE_ENG{n}_TEMPERATURE_C"),
            read_env_bird_fan_frac: format!("ENV_BIRD_FAN_DAMAGE:{n}"),
            read_env_bird_core_frac: format!("ENV_BIRD_CORE_FOD:{n}"),
            gp_egt_delta_c: v("EGT_DELTA_C"),
            gp_oil_temp_delta_c: v("OIL_TEMP_DELTA_C"),
            gp_oil_press_delta_psi: v("OIL_PRESS_DELTA_PSI"),
            gp_oil_quantity_delta_frac: v("OIL_QUANTITY_DELTA_FRAC"),
            gp_stall_margin_loss_pct: v("GASPATH_STALL_MARGIN_LOSS_PCT"),
            gp_n2_capability_loss_pct: v("N2_CAPABILITY_LOSS_PCT"),
            gp_n3_capability_loss_pct: v("N3_CAPABILITY_LOSS_PCT"),
            gp_thrust_loss_pct: v("GASPATH_THRUST_LOSS_PCT"),
            gp_surge: v("GASPATH_SURGE"),
            gp_oil_temp_c: v("GASPATH_OIL_TEMP_C"),
            gp_oil_press_psi: v("GASPATH_OIL_PRESS_PSI"),
            gp_oil_quantity_frac: v("GASPATH_OIL_QUANTITY_FRAC"),
            gp_oil_supply_c: v("GASPATH_OIL_SUPPLY_C"),
            gp_oil_filter_bypassed: v("GASPATH_OIL_FILTER_BYPASSED"),
            gp_oil_relief_open: v("GASPATH_OIL_RELIEF_OPEN"),
            gp_fuel_heat_w: v("GASPATH_FUEL_HEAT_W"),
            gp_fuel_out_c: v("GASPATH_FUEL_OUT_C"),
            gp_hot_section_soak_c: v("HOT_SECTION_SOAK_C"),
            gp_egt_shadow_c: v("EGT_SHADOW_C"),
            case_breach: v("CASE_BREACH_FRAC"),
            nacelle_fuel_leak: v("NACELLE_FUEL_LEAK_KG_S"),
            phys_n1: v("PHYS_N1"),
            phys_n2: v("PHYS_N2"),
            phys_n3: v("PHYS_N3"),
            phys_egt: v("PHYS_EGT_C"),
            phys_ff: v("PHYS_FF_KG_S"),
            phys_lit: v("PHYS_LIT"),
            phys_thrust: v("PHYS_THRUST_N"),
            phys_valid: v("PHYS_VALID"),
            gp_ip_port_pressure_pa: v("IP_PORT_PRESSURE_PA"),
            gp_ip_port_temp_k: v("IP_PORT_TEMP_K"),
            bearing_seizure_frac: v("BEARING_SEIZURE_FRAC"),
        }
    }
}

fn eec_param_key(i: usize) -> &'static str {
    match PARAMS[i] {
        super::eec::Param::N1 => "N1",
        super::eec::Param::N2 => "N2",
        super::eec::Param::N3 => "N3",
        super::eec::Param::Tgt => "TGT",
        super::eec::Param::P30 => "P30",
    }
}

struct ReverserNames {
    position: String,
    uncommanded: String,
    position_disagree: String,
    energized: String,
    ctl_fault: String,
    lock_degraded_count: String,
    mel_inop: String,
}

impl ReverserNames {
    fn new(eng_number: usize) -> Self {
        Self {
            position: format!("A32NX_ENG_{eng_number}_REV_POSITION"),
            uncommanded: format!("A32NX_ENG_{eng_number}_REV_UNCOMMANDED"),
            position_disagree: format!("A32NX_ENG_{eng_number}_REV_POSITION_DISAGREE"),
            energized: format!("A32NX_ENG_{eng_number}_REV_ENERGIZED"),
            mel_inop: format!("A32NX_ENG_{eng_number}_REV_MEL_INOP"),
            ctl_fault: format!("A32NX_ENG_{eng_number}_REV_CTL_FAULT"),
            lock_degraded_count: format!("A32NX_ENG_{eng_number}_REV_LOCK_DEGRADED_COUNT"),
        }
    }
}

struct ReverserUnit {
    slot: usize,
    ids: ReverserIds,
    names: ReverserNames,
    model: ThrustReverser,
    state: ReverserState,
    commanded: bool,
}

impl ReverserUnit {
    fn new(reg: &Registry, eng_number: usize) -> Self {
        Self {
            slot: REVERSER_ENGINES.iter().position(|e| *e == eng_number).expect("only engines 2 and 3 carry a reverser"),
            ids: ReverserIds::resolve(reg, eng_number),
            names: ReverserNames::new(eng_number),
            model: ThrustReverser::new(),
            state: ReverserState::default(),
            commanded: false,
        }
    }

    fn step(&mut self, commanded: bool, hydraulic_frac: f64, faults: &Faults, dt: f64) {
        let lock = |hold: u64, jam: u64| LockFaults { fails_to_hold: faults.get(hold), jam: faults.get(jam) };
        let rev_faults = ReverserFaults {
            lock_a: lock(self.ids.lock_hold[0], self.ids.lock_jam[0]),
            lock_b: lock(self.ids.lock_hold[1], self.ids.lock_jam[1]),
            lock_c: lock(self.ids.lock_hold[2], self.ids.lock_jam[2]),
            actuator_jam: faults.get(self.ids.actuator_jam),
            control_fault: faults.get(self.ids.control_fault),
        };
        self.commanded = commanded;
        self.state = self.model.step(commanded, hydraulic_frac, &rev_faults, dt);
    }
}

struct EngineChain {
    names: ChainNames,

    fuel_ids: FuelIds,
    fmu: FuelMeteringUnit,
    sov: ShutoffValve,
    transmitter: FlowTransmitter,
    eec: Eec,
    lp: LpPumpState,
    strainer: StrainerState,
    filter: FilterState,
    hp: HpPumpState,
    fmu_state: FmuState,
    sov_position: f64,
    flow: FlowReading,
    manifold: ManifoldState,
    eec_state: EecState,
    delivered_kg_s: f64,
    wf_command_kg_s: f64,
    fmu_fault_latched: bool,
    thrust_abnormal_latched: bool,
    hp_pump_low_flow_latched: bool,
    fmu_fault_confirm_s: f64,
    thrust_abnormal_confirm_s: f64,
    hp_pump_low_flow_confirm_s: f64,
    sov_commanded_open: bool,
    eec_backup_probe_fault: bool,
    tla_deg: f64,
    ground_protection: super::ground_protection::GroundProtection,
    tla_channel_b_deg: f64,
    tla_channel_b_disagree: bool,

    ignition_ids: IgnitionIds,
    ignition_state: IgnitionState,
    ignition_powered: bool,

    start_ids: StartIds,
    start_valve: AirValve,
    starter: AirTurbineStarter,
    starter_heat: DutyCycleHeat,
    start_valve_position: f64,
    start_valve_commanded_open: bool,
    start_valve_steady_s: f64,
    ats_state: AtsState,

    airflow_ids: AirflowIds,
    vsv: Vsv,
    vsv_state: VsvState,
    handling_bleeds: [BleedValve; N_HANDLING_BLEEDS],
    handling_bleed_state: [BleedValveState; N_HANDLING_BLEEDS],
    handling_bleed_target: [f64; N_HANDLING_BLEEDS],

    rotor_ids: RotorIds,
    spool_vibration: [SpoolVibration; N_SPOOLS],
    vibration_state: [VibrationState; N_SPOOLS],
    bearings: EngineBearings,
    bearing_state: [BearingState; N_BEARINGS],

    nacelle_ids: NacelleIds,
    anti_ice: AntiIceValve,
    anti_ice_state: AntiIceState,
    anti_ice_commanded_open: bool,
    ventilation_state: VentilationState,
    fire_reading: [FireZoneReading; N_FIRE_ZONES],
    engine_running: bool,
    oil_filter_bypassed: bool,

    reverser: Option<ReverserUnit>,

    oil_ids: OilIds,

    turbine_damage_ids: TurbineDamageIds,
    seizure_ids: SeizureIds,
    bearing_seizure_frac: f64,
    debris_breach_frac: f64,
    nacelle_fuel_leak_kg_s: f64,

    gas_path_shadow: gas_path_model::ShadowEngine,
    gas_path_out: gas_path_model::ShadowOutputs,
    free: FreeEngine,
    free_out: free_engine::Outputs,
    free_ready: bool,
    alive_s: f64,
    fuel_inlet_k: f64,
}

impl EngineChain {
    fn new(reg: &Registry, eng: usize) -> Self {
        let n = eng + 1;
        Self {
            names: ChainNames::new(eng),

            fuel_ids: FuelIds::resolve(reg, eng),
            fmu: FuelMeteringUnit::new(),
            sov: ShutoffValve::new(false),
            transmitter: FlowTransmitter::new(),
            eec: Eec::new(),
            lp: LpPumpState::default(),
            strainer: StrainerState::default(),
            filter: FilterState::default(),
            hp: HpPumpState::default(),
            fmu_state: FmuState::default(),
            sov_position: 0.0,
            flow: FlowReading::default(),
            manifold: manifold::step(0.0, 101_325.0, &ManifoldFaults::default()),
            eec_state: EecState {
                selected: [0.0; N_EEC_PARAMS],
                disagree: [false; N_EEC_PARAMS],
                no_data: [false; N_EEC_PARAMS],
                active: ActiveChannel::A,
                fuel_flow_disagree: false,
                channel_a_serviceable: true,
                channel_b_serviceable: true,
            },
            delivered_kg_s: 0.0,
            wf_command_kg_s: 0.0,
            fmu_fault_latched: false,
            thrust_abnormal_latched: false,
            hp_pump_low_flow_latched: false,
            fmu_fault_confirm_s: 0.0,
            thrust_abnormal_confirm_s: 0.0,
            hp_pump_low_flow_confirm_s: 0.0,
            sov_commanded_open: false,
            eec_backup_probe_fault: false,
            tla_deg: 0.0,
            ground_protection: super::ground_protection::GroundProtection::default(),
            tla_channel_b_deg: 0.0,
            tla_channel_b_disagree: false,

            ignition_ids: IgnitionIds::resolve(reg, eng),
            ignition_state: IgnitionState::default(),
            ignition_powered: false,

            start_ids: StartIds::resolve(reg, eng),
            start_valve: AirValve::new(false),
            starter: AirTurbineStarter::new(),
            starter_heat: DutyCycleHeat::new(),
            start_valve_position: 0.0,
            start_valve_commanded_open: false,
            start_valve_steady_s: START_VALVE_SETTLE_S,
            ats_state: AtsState::default(),

            airflow_ids: AirflowIds::resolve(reg, eng),
            vsv: Vsv::new(0.0),
            vsv_state: VsvState::default(),
            handling_bleeds: [BleedValve::new(); N_HANDLING_BLEEDS],
            handling_bleed_state: [BleedValveState::default(); N_HANDLING_BLEEDS],
            handling_bleed_target: [1.0; N_HANDLING_BLEEDS],

            rotor_ids: RotorIds::resolve(reg, eng),
            spool_vibration: [SpoolVibration::new(FAN_SPEC), SpoolVibration::new(IP_SPEC), SpoolVibration::new(HP_SPEC)],
            vibration_state: [VibrationState::default(); N_SPOOLS],
            bearings: EngineBearings::new(),
            bearing_state: [BearingState::default(); N_BEARINGS],

            nacelle_ids: NacelleIds::resolve(reg, eng),
            anti_ice: AntiIceValve::new(ISA_SL_TEMP_K),
            anti_ice_state: AntiIceState::default(),
            anti_ice_commanded_open: false,
            ventilation_state: VentilationState::default(),
            fire_reading: [FireZoneReading::default(); N_FIRE_ZONES],
            engine_running: false,
            oil_filter_bypassed: false,

            reverser: REVERSER_ENGINES.contains(&n).then(|| ReverserUnit::new(reg, n)),

            oil_ids: OilIds::resolve(reg, eng),

            turbine_damage_ids: TurbineDamageIds::resolve(reg, eng),
            seizure_ids: SeizureIds::resolve(reg, eng),
            bearing_seizure_frac: 0.0,
            debris_breach_frac: 0.0,
            nacelle_fuel_leak_kg_s: 0.0,

            gas_path_shadow: gas_path_model::ShadowEngine::new(),
            gas_path_out: gas_path_model::ShadowOutputs::default(),
            free: FreeEngine::new(),
            free_out: free_engine::Outputs::default(),
            free_ready: false,
            alive_s: 0.0,
            fuel_inlet_k: ISA_SL_TEMP_K,
        }
    }

    fn consumed_ids(&self) -> Vec<u64> {
        let mut v = self.fuel_ids.all();
        v.extend(self.ignition_ids.all());
        v.extend(self.start_ids.all());
        v.extend(self.airflow_ids.all());
        v.extend(self.rotor_ids.all());
        v.extend(self.nacelle_ids.all());
        if let Some(rev) = &self.reverser {
            v.extend(rev.ids.all());
        }
        v.extend(self.oil_ids.all());
        v.extend(self.turbine_damage_ids.all());
        v.extend(self.seizure_ids.all());
        v
    }

    fn step(&mut self, eng: usize, truth: &Truth, faults: &Faults, commands: &EngineAccessoryCommands, dt: f64) {
        let ambient_pa = truth.environment.ambient_pressure_pa.max(0.0);
        let ambient_k = (truth.environment.sat_c + 273.15).max(1.0);
        let theta_sqrt = (ambient_k / ISA_SL_TEMP_K).sqrt().max(1e-6);

        let n1 = truth.engine_n1_frac[eng].max(0.0);
        let n2 = truth.engine_n2_frac[eng].max(0.0);
        let n3 = truth.engine_n3_frac[eng].max(0.0);

        let env_bird_fan_frac = truth.published.get_or(&self.names.read_env_bird_fan_frac, 0.0).clamp(0.0, 1.0);
        let env_bird_core_frac = truth.published.get_or(&self.names.read_env_bird_core_frac, 0.0).clamp(0.0, 1.0);

        self.step_debris(eng, faults, n3);
        self.step_fuel(eng, truth, faults, commands, dt, ambient_pa, n2, n3);
        self.step_ignition(eng, truth, faults);
        self.step_starting(eng, truth, faults, dt, ambient_pa, n3);
        self.step_airflow(eng, truth, faults, dt, ambient_pa, n2 / theta_sqrt, n3 / theta_sqrt);
        self.step_rotors(eng, faults, dt, n1, n2, n3, env_bird_fan_frac, env_bird_core_frac);
        self.step_nacelle(eng, truth, faults, dt, ambient_pa, ambient_k);
        self.step_reverser(truth, faults, commands, dt);
        self.step_gas_path_shadow(eng, truth, faults, dt, ambient_pa, ambient_k, env_bird_fan_frac, env_bird_core_frac);
        self.step_latches(dt);
    }

    fn step_debris(&mut self, eng: usize, faults: &Faults, n3: f64) {
        let destruction_sev = faults.get(FAILURE_HP_COMPRESSOR_DESTRUCTION[eng]).clamp(0.0, 1.0);
        if destruction_sev <= 0.0 {
            self.debris_breach_frac = 0.0;
            return;
        }
        let n3_frac = if self.free_ready { self.free_out.n3_pct / 100.0 } else { n3 };
        let release = destruction_sev * n3_frac.max(0.0).powi(2);
        let uncontained = ((release - CASE_CONTAINED_RELEASE_FRAC) / (1.0 - CASE_CONTAINED_RELEASE_FRAC)).clamp(0.0, 1.0);
        self.debris_breach_frac = self.debris_breach_frac.max(uncontained);
    }

    fn step_gas_path_shadow(
        &mut self,
        eng: usize,
        truth: &Truth,
        faults: &Faults,
        dt: f64,
        ambient_pa: f64,
        ambient_k: f64,
        env_bird_fan_frac: f64,
        env_bird_core_frac: f64,
    ) {
        let sound_speed = (gas_path_model::gas::GAMMA_AIR * gas_path_model::gas::R_AIR * ambient_k).max(1.0).sqrt();
        let mach = (truth.environment.tas_ms.max(0.0) / sound_speed).max(0.0);

        const MARGIN_PCT_AT_FULL_LOSS: f64 = 15.0;
        let margin_lost_pct = (-self.vsv_state.stall_margin_delta_pct).max(0.0)
            + self.handling_bleed_state.iter().map(|b| (-b.stall_margin_delta_pct).max(0.0)).sum::<f64>()
            + env_bird_fan_frac * MARGIN_PCT_AT_FULL_LOSS;
        let margin_loss_frac = (margin_lost_pct / MARGIN_PCT_AT_FULL_LOSS).clamp(0.0, 1.0);
        let stall_sev = faults.get(FAILURE_COMPRESSOR_STALL[eng]).clamp(0.0, 1.0);
        let destruction_sev = faults.get(FAILURE_HP_COMPRESSOR_DESTRUCTION[eng]).clamp(0.0, 1.0);
        let compressor_loss_frac = margin_loss_frac.max(stall_sev * STALL_LOSS_FRACTION_AT_FULL_SEVERITY).max(destruction_sev);

        let seizure = faults.get(self.seizure_ids.frac).clamp(0.0, 1.0);
        self.bearing_seizure_frac = seizure;
        let compressor_loss_frac = compressor_loss_frac.max(seizure);

        const CORE_FOD_TURBINE_EFFICIENCY_LOSS_AT_FULL_SEVERITY: f64 = 0.4;
        let fod_turbine_loss = (env_bird_core_frac * CORE_FOD_TURBINE_EFFICIENCY_LOSS_AT_FULL_SEVERITY).max(seizure);

        let oil_pump_fault = faults.get(FAILURE_OIL_PUMP[eng]).max(faults.get(self.oil_ids.pump_fault)).clamp(0.0, 1.0);
        let oil_pressure_fraction = 1.0 - oil_pump_fault;

        let chips_active = self.bearing_state.iter().filter(|b| b.chip_detected).count();
        let oil_faults =
            gas_path_model::oil::OilFaults {
                filter_clog: (chips_active as f64 / N_BEARINGS as f64 * 0.5).clamp(0.0, 1.0).max(if self.oil_filter_bypassed { 1.0 } else { 0.0 }),
                leak: faults.get(self.oil_ids.leak).max(self.debris_breach_frac * DEBRIS_OIL_BREACH_LEAK_FRAC),
            };

        let bleed_extraction_kg_s = self.handling_bleed_state.iter().map(|b| b.bled_kg_s).sum::<f64>() + self.anti_ice_state.bled_kg_s;

        const TURBINE_DAMAGE_LOSS_CEILING: f64 = 0.15;
        const TURBINE_RELEASE_LOSS_CEILING: f64 = 0.35;
        let turbine_efficiency_loss_fraction = (faults.get(self.turbine_damage_ids.damage) * TURBINE_DAMAGE_LOSS_CEILING
            + faults.get(self.turbine_damage_ids.release) * TURBINE_RELEASE_LOSS_CEILING)
            .clamp(0.0, 1.0)
            .max(fod_turbine_loss);

        self.gas_path_out = self.gas_path_shadow.step(&gas_path_model::ShadowInputs {
            ambient_pressure_pa: ambient_pa,
            ambient_temp_k: ambient_k,
            mach,
            n1_pct: truth.engine_n1_frac[eng].max(0.0) * 100.0,
            n2_pct: truth.engine_n2_healthy_frac[eng].max(0.0) * 100.0,
            n3_pct: truth.engine_n3_healthy_frac[eng].max(0.0) * 100.0,
            wf_kg_s: self.wf_command_kg_s,
            bleed_extraction_kg_s,
            bleed_from_ip_port: false,
            compressor_efficiency_loss_fraction: compressor_loss_frac,
            compressor_flow_capacity_loss_fraction: compressor_loss_frac,
            turbine_efficiency_loss_fraction,
            oil_pressure_fraction,
            oil_faults,
            fuel_temp_k: self.fuel_inlet_k,
            dt_s: dt,
        });

        const SEIZURE_DRAG_DESIGN_TORQUES: f64 = 2.0;
        const SCHEDULE_MARGIN_EFFICIENCY_SHARE: f64 = 0.25;
        let free_compressor_loss = (margin_loss_frac * SCHEDULE_MARGIN_EFFICIENCY_SHARE)
            .max(stall_sev * STALL_LOSS_FRACTION_AT_FULL_SEVERITY)
            .max(destruction_sev)
            .max(seizure);
        let n1_target_pct = self.ground_protection.n1_target(
            truth.engine_n1_commanded_pct[eng].max(0.0),
            truth.on_ground,
            truth.groundspeed_m_s,
            truth.engine_tla_deg[eng],
        );
        let free_in = free_engine::Inputs {
            n1_target_pct,
            fuel_available: self.sov_position > 0.05,
            ignition: self.ignition_state.chain_a_firing || self.ignition_state.chain_b_firing,
            ip_bleed_kg_s: truth.engine_customer_bleed_kg_s[eng],
            hp_bleed_kg_s: self.anti_ice_state.bled_kg_s,
            hpc_efficiency: 1.0 - free_compressor_loss,
            hpc_flow_capacity: 1.0 - free_compressor_loss,
            turbine_efficiency: 1.0 - turbine_efficiency_loss_fraction,
            hp_drag_torque_n_m: seizure * SEIZURE_DRAG_DESIGN_TORQUES * self.free.hp_design_torque_n_m(),
            fuel_metered_kg_s: Some(self.delivered_kg_s),
            starter_torque_n_m: Some(self.ats_state.torque_n_m),
            ..free_engine::Inputs::at(ambient_pa, ambient_k, mach, truth.environment.tas_ms.max(0.0), dt)
        };
        const VALID_AFTER_S: f64 = 3.0;
        const PRESET_IDLE_N3_PCT: f64 = 63.0;
        const SIM_RUNNING_CN2_PCT: f64 = 50.0;
        const SIM_STOPPED_CN2_PCT: f64 = 5.0;
        if truth.flight_ready {
            self.alive_s += dt;
        }
        let mut free_in = free_in;
        let cold = !self.free.is_lit() && self.free_out.n3_pct < 5.0;
        let sqrt_theta2 = (gas_path_model::inlet::station2(ambient_pa, ambient_k, mach).tt_k / ISA_SL_TEMP_K).max(1e-6).sqrt();
        let sim_cn2 = truth.sim_engine_corrected_n2_pct[eng];
        let fbw_running = truth.engine_running[eng] && truth.engine_n3_frac[eng] > 0.5;
        let sim_running = sim_cn2 > SIM_RUNNING_CN2_PCT;
        let spawned_running = !self.free_ready && (fbw_running || sim_running);
        let preset_start = truth.aircraft_preset_quick_mode && truth.controls.engine_master_on[eng];
        if cold && (spawned_running || preset_start) {
            let settle_in = free_engine::Inputs { fuel_available: true, fuel_metered_kg_s: None, ..free_in };
            let (n1, n2, n3) = if spawned_running && fbw_running {
                (truth.engine_n1_frac[eng] * 100.0, truth.engine_n2_frac[eng] * 100.0, truth.engine_n3_frac[eng] * 100.0)
            } else if spawned_running {
                let n3 = sim_cn2 * sqrt_theta2;
                (truth.sim_engine_corrected_n1_pct[eng] * sqrt_theta2, n3 * 0.75, n3)
            } else {
                let idle_n1 = truth.engine_n1_commanded_pct[eng].max(1.0);
                (idle_n1, idle_n1 + 30.0, PRESET_IDLE_N3_PCT)
            };
            self.free.settle_running(&settle_in, n1, n2, n3);
            self.free_ready = true;
            if faults.get(self.airflow_ids.vsv_jam) <= 0.0 {
                self.vsv = Vsv::new(truth.engine_n2_frac[eng].max(0.0) / (ambient_k / ISA_SL_TEMP_K).max(1e-6).sqrt());
            }
            let demand = self.free.fuel_demand_kg_s();
            let fmu_impaired = faults.get(self.fuel_ids.fmu_sticking) > 0.0
                || faults.get(self.fuel_ids.fmu_spill_open) > 0.0
                || faults.get(self.fuel_ids.fmu_spill_closed) > 0.0;
            if faults.get(self.fuel_ids.sov_stuck) <= 0.0 {
                self.sov = ShutoffValve::new(true);
                self.sov_position = 1.0;
            }
            let fuel_metered_for_free = if fmu_impaired {
                self.delivered_kg_s
            } else {
                self.fmu.prime(demand);
                self.fmu_state.metered_kg_s = demand;
                self.delivered_kg_s = demand;
                demand
            };
            free_in = free_engine::Inputs { fuel_available: true, fuel_metered_kg_s: Some(fuel_metered_for_free), ..free_in };
        }
        if !self.free_ready && self.alive_s >= VALID_AFTER_S && !fbw_running && !truth.engine_running[eng] && sim_cn2 < SIM_STOPPED_CN2_PCT {
            self.free_ready = true;
        }
        self.free_out = self.free.step(&free_in);
    }

    #[allow(clippy::too_many_arguments)]
    fn step_fuel(&mut self, eng: usize, truth: &Truth, faults: &Faults, commands: &EngineAccessoryCommands, dt: f64, ambient_pa: f64, n2: f64, n3: f64) {
        let published_fuel_k = truth.published.get(&self.names.read_feed_tank_temp_c).map(|c| c + 273.15);
        let fuel_k = commands.fuel_inlet_k.or(published_fuel_k).unwrap_or(truth.environment.sat_c + 273.15).max(1.0);
        self.fuel_inlet_k = fuel_k;
        let inlet_pa = ambient_pa + FEED_BOOST_RISE_PA;

        let wf_command = commands
            .wf_command_kg_s
            .map_or(if self.free_ready { self.free_out.wf_demand_kg_s } else { truth.engine_fuel_flow_kg_s[eng] }, |c| c[eng])
            .max(0.0);
        self.wf_command_kg_s = wf_command;

        let starvation_from_lp = if self.hp.delivered_m3_s > 1e-9 { (1.0 - self.lp.flow_m3_s / self.hp.delivered_m3_s).clamp(0.0, 1.0) } else { 0.0 };
        let hp_faults = HpPumpFaults {
            wear: faults.get(self.fuel_ids.hp_wear),
            inlet_starvation: faults.get(self.fuel_ids.hp_starvation).max(starvation_from_lp),
        };
        self.hp = hp_pump::step(n3, self.fmu_state.differential_pa, &hp_faults);

        let strainer_faults = StrainerFaults { clog: faults.get(self.fuel_ids.strainer_clog) };
        self.strainer = strainer::step(inlet_pa, self.hp.delivered_m3_s, fuel_k, &strainer_faults);
        let lp_faults = LpPumpFaults { wear: faults.get(self.fuel_ids.lp_wear), inlet_restriction: faults.get(self.fuel_ids.lp_inlet_restriction) };
        self.lp = lp_pump::step(n3, inlet_pa, fuel_k, self.hp.delivered_m3_s, &lp_faults);
        let lp_valve_open = truth.controls.engine_master_on[eng] && !truth.controls.fire_pb_released[eng];
        let line_gauge_pa = (self.lp.outlet_pa - ambient_pa).max(0.0);
        let fuel_density = super::fuel::common::FUEL_DENSITY_KG_M3;
        self.nacelle_fuel_leak_kg_s = if lp_valve_open {
            ORIFICE_CD * self.debris_breach_frac * DEBRIS_FUEL_BREACH_AREA_M2 * (2.0 * fuel_density * line_gauge_pa).sqrt()
        } else {
            0.0
        };
        let filter_faults = FilterFaults { clog: faults.get(self.fuel_ids.filter_clog), monitor_fault: faults.get(self.fuel_ids.filter_monitor_fault) };
        self.filter = filter::step(self.lp.outlet_pa, self.hp.delivered_m3_s, fuel_k, &filter_faults);

        let backup_bias_c = faults.get(self.fuel_ids.eec_backup_probe_bias) * EEC_BACKUP_PROBE_MAX_BIAS_C;
        self.eec_backup_probe_fault = backup_bias_c.abs() > EEC_BACKUP_PROBE_DISAGREE_C;

        let tla_bias_deg = faults.get(self.fuel_ids.tla_channel_b_bias) * THR_LEVER_CHANNEL_B_MAX_BIAS_DEG;
        self.tla_deg = truth.engine_tla_deg[eng];
        self.tla_channel_b_deg = truth.engine_tla_deg[eng] + tla_bias_deg;
        self.tla_channel_b_disagree = (self.tla_channel_b_deg - truth.engine_tla_deg[eng]).abs() > THR_LEVER_DISAGREE_TOLERANCE_DEG;

        let fmu_faults = FmuFaults {
            valve_sticking: faults.get(self.fuel_ids.fmu_sticking),
            spill_stuck_open: faults.get(self.fuel_ids.fmu_spill_open),
            spill_stuck_closed: faults.get(self.fuel_ids.fmu_spill_closed),
        };
        let hp_supply_pa = self.filter.outlet_pa + self.fmu_state.differential_pa;
        self.fmu_state = self.fmu.step(wf_command, hp_supply_pa, self.hp.delivered_m3_s, &fmu_faults, dt);

        let sov_faults = ShutoffValveFaults { stuck: faults.get(self.fuel_ids.sov_stuck) };
        self.sov_commanded_open = truth.controls.engine_master_on[eng] && !truth.controls.fire_pb_released[eng];
        self.sov_position = self.sov.step(self.sov_commanded_open, &sov_faults, dt);
        self.delivered_kg_s = self.fmu_state.metered_kg_s * self.sov_position;

        let a = PickoffFaults { bias_frac_of_design: faults.get(self.fuel_ids.ft_a_bias), frozen: faults.get(self.fuel_ids.ft_a_frozen) };
        let b = PickoffFaults { bias_frac_of_design: faults.get(self.fuel_ids.ft_b_bias), frozen: faults.get(self.fuel_ids.ft_b_frozen) };
        self.flow = self.transmitter.step(self.delivered_kg_s, &a, &b, dt);

        let p30_pa = truth.engine_hp_port_pressure_pa[eng];
        let manifold_faults = ManifoldFaults { group_blockage: std::array::from_fn(|g| faults.get(self.fuel_ids.nozzle[g])) };
        self.manifold = manifold::step(self.delivered_kg_s, p30_pa, &manifold_faults);

        let sensor = |id: u64| {
            let m = faults.get(id);
            SensorFaults { bias: m, frozen: m }
        };
        let eec_faults = EecFaults {
            channel_a_fault: faults.get(self.fuel_ids.eec_channel_a),
            channel_b_fault: faults.get(self.fuel_ids.eec_channel_b),
            sensor_a: std::array::from_fn(|i| sensor(self.fuel_ids.eec_sensor_a[i])),
            sensor_b: std::array::from_fn(|i| sensor(self.fuel_ids.eec_sensor_b[i])),
        };
        let tgt_k = commands.engine_tgt_k.map_or(self.free_out.egt_c + 273.15, |t| t[eng]);
        let true_values = [truth.engine_n1_frac[eng] * 100.0, n2 * 100.0, n3 * 100.0, tgt_k, p30_pa];
        self.eec_state = self.eec.step(true_values, self.flow.channel_a_kg_s, self.flow.channel_b_kg_s, &eec_faults, dt);
    }

    fn step_ignition(&mut self, eng: usize, truth: &Truth, faults: &Faults) {
        let bus_live = truth.ac_bus_volts[eng] >= IGNITION_MIN_BUS_V;
        let selected = truth.controls.starter_engaged[eng] || (truth.controls.engine_master_on[eng] && !truth.engine_running[eng]);
        self.ignition_powered = bus_live && selected;
        let ign_faults = IgnitionFaults {
            exciter_a_failure: faults.get(self.ignition_ids.exciter[0]),
            exciter_b_failure: faults.get(self.ignition_ids.exciter[1]),
            igniter_a_erosion: faults.get(self.ignition_ids.igniter[0]),
            igniter_b_erosion: faults.get(self.ignition_ids.igniter[1]),
        };
        self.ignition_state = ignition::step(self.ignition_powered, &ign_faults);
    }

    fn step_starting(&mut self, eng: usize, truth: &Truth, faults: &Faults, dt: f64, ambient_pa: f64, n3: f64) {
        self.start_valve_commanded_open = truth.controls.starter_engaged[eng];
        let valve_faults = AirValveFaults { stuck: faults.get(self.start_ids.sav_stuck) };
        self.start_valve_position = self.start_valve.step(self.start_valve_commanded_open, &valve_faults, dt);

        let duct_pa = truth.published.get_or(&self.names.read_start_duct_pa, ambient_pa);
        let supply_ratio = ((duct_pa - ambient_pa) / START_DESIGN_DP_PA).clamp(0.0, 1.5);
        let supply_fraction = supply_ratio * self.start_valve_position;

        let ats_faults = AtsFaults {
            clutch_fails_to_engage: faults.get(self.start_ids.clutch_engage),
            clutch_fails_to_disengage: faults.get(self.start_ids.clutch_disengage),
        };
        self.ats_state = self.starter.step(supply_fraction, n3 * N3_DESIGN_RPM, &ats_faults);

        let rotor_omega = omega_rad_s(self.ats_state.rotor_rpm);
        let cranking_power_w = (self.ats_state.torque_n_m * rotor_omega).max(0.0);
        self.starter_heat.step(cranking_power_w, dt);
    }

    fn start_valve_disagrees(&self) -> bool {
        let target = if self.start_valve_commanded_open { 1.0 } else { 0.0 };
        self.start_valve_steady_s >= START_VALVE_SETTLE_S && (self.start_valve_position - target).abs() > START_VALVE_DISAGREE_TOLERANCE
    }

    fn starter_disengage_fault(&self) -> bool {
        self.ats_state.rotor_rpm > N3_DESIGN_RPM * FREE_SPEED_FRAC * STARTER_OVERRUN_MARGIN
    }

    #[allow(clippy::too_many_arguments)]
    fn step_airflow(&mut self, eng: usize, truth: &Truth, faults: &Faults, dt: f64, ambient_pa: f64, n2_corrected: f64, n3_corrected: f64) {
        let vsv_faults = VsvFaults { jam: faults.get(self.airflow_ids.vsv_jam), rigging_bias_deg: -MAX_RIGGING_BIAS_DEG * faults.get(self.airflow_ids.vsv_rigging) };
        self.vsv_state = self.vsv.step(n2_corrected, &vsv_faults, dt);

        let sources = [
            (truth.engine_bleed_pressure_pa[eng], truth.engine_bleed_temp_k[eng], n2_corrected, &IP_HANDLING_BLEED),
            (truth.engine_hp_port_pressure_pa[eng], truth.engine_hp_port_temp_k[eng], n3_corrected, &HP_HANDLING_BLEED),
        ];
        for i in 0..N_HANDLING_BLEEDS {
            let (upstream_pa, upstream_k, corrected, spec): (f64, f64, f64, &BleedValveSpec) = sources[i];
            self.handling_bleed_target[i] = bleed_valve::schedule_open_fraction(spec, corrected);
            let bleed_faults = BleedValveFaults { jam: faults.get(self.airflow_ids.bleed_jam[i]) };
            self.handling_bleed_state[i] = self.handling_bleeds[i].step(spec, corrected, upstream_pa, upstream_k, ambient_pa, CORE_DESIGN_FLOW_KG_S, &bleed_faults, dt);
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn step_rotors(&mut self, eng: usize, faults: &Faults, dt: f64, n1: f64, n2: f64, n3: f64, env_bird_fan_frac: f64, env_bird_core_frac: f64) {
        let design = [omega_rad_s(N1_DESIGN_RPM), omega_rad_s(N2_DESIGN_RPM), omega_rad_s(N3_DESIGN_RPM)];
        let omega = [design[0] * n1, design[1] * n2, design[2] * n3];

        let env_bird_by_spool = [env_bird_fan_frac, env_bird_core_frac, env_bird_core_frac];
        let stall_sev = faults.get(FAILURE_COMPRESSOR_STALL[eng]).clamp(0.0, 1.0);
        let destruction_sev = faults.get(FAILURE_HP_COMPRESSOR_DESTRUCTION[eng]).clamp(0.0, 1.0);
        let extra_hp_blade_loss_frac = (stall_sev * STALL_VIBRATION_PROXY_AT_FULL_SEVERITY).max(destruction_sev);

        for s in 0..N_SPOOLS {
            let ids = self.rotor_ids.imbalance[s];
            let turbine_release_mass = if s == 2 { faults.get(self.turbine_damage_ids.release) } else { 0.0 };
            let mut imbalance = ImbalanceFaults {
                blade_loss_frac: (faults.get(ids[0]) + turbine_release_mass).clamp(0.0, 1.0),
                ice_frac: faults.get(ids[1]),
                bird_strike_frac: faults.get(ids[2]).max(env_bird_by_spool[s]),
            };
            if s == HP_SPOOL_INDEX {
                imbalance.blade_loss_frac = imbalance.blade_loss_frac.max(extra_hp_blade_loss_frac);
            }
            self.vibration_state[s] = self.spool_vibration[s].step(omega[s], &imbalance, dt);
        }

        let mut bearing_faults = EngineBearingFaults::default();
        for b in 0..N_BEARINGS {
            let ids = self.rotor_ids.bearing[b];
            bearing_faults.faults[b] =
                BearingFaults { outer_race_spall: faults.get(ids[0]), inner_race_spall: faults.get(ids[1]), rolling_element_spall: faults.get(ids[2]), cage_wear: faults.get(ids[3]) };
        }
        self.bearing_state = self.bearings.step(omega[0], omega[1], omega[2], design[0], design[1], design[2], &bearing_faults, dt);
    }

    fn step_nacelle(&mut self, eng: usize, truth: &Truth, faults: &Faults, dt: f64, ambient_pa: f64, ambient_k: f64) {
        self.anti_ice_commanded_open = truth.controls.nacelle_anti_ice_selected[eng];
        let ai_faults = AntiIceValveFaults { stuck: faults.get(self.nacelle_ids.anti_ice_stuck) };
        self.anti_ice_state = self.anti_ice.step(
            self.anti_ice_commanded_open,
            truth.engine_bleed_pressure_pa[eng],
            truth.engine_bleed_temp_k[eng],
            ambient_k,
            &ai_faults,
            dt,
        );

        let rho = ambient_pa / (R_AIR * ambient_k);
        let tas = truth.environment.tas_ms.max(0.0);
        let dynamic_pressure_pa = 0.5 * rho * tas * tas;
        let vent_faults = VentilationFaults { scoop_blockage: faults.get(self.nacelle_ids.scoop), eductor_blockage: faults.get(self.nacelle_ids.eductor) };
        self.engine_running = truth.engine_running[eng];
        self.oil_filter_bypassed = truth.engine_oil_filter_bypassed[eng];
        self.ventilation_state = ventilation::step(dynamic_pressure_pa, self.engine_running, &vent_faults);

        let cowl_c = truth.published.get_or(&self.names.read_nacelle_zone_temp_c, truth.environment.sat_c);
        let fire_c = truth.published.get_or(&self.names.read_nacelle_fire_zone_temp_c, cowl_c);
        let zone_k = cowl_c.max(fire_c) + 273.15;
        for z in 0..N_FIRE_ZONES {
            let ids = self.nacelle_ids.fire[z];
            let loop_a = LoopFaults { fails_to_detect: faults.get(ids[0]), false_trip: faults.get(ids[1]) };
            let loop_b = LoopFaults { fails_to_detect: faults.get(ids[2]), false_trip: faults.get(ids[3]) };
            self.fire_reading[z] = fire_detection::read(zone_k, &loop_a, &loop_b);
        }
    }

    fn step_reverser(&mut self, truth: &Truth, faults: &Faults, commands: &EngineAccessoryCommands, dt: f64) {
        let Some(rev) = &mut self.reverser else { return };
        let hydraulic_frac = (truth.hydraulic_pressure_pa[0].max(truth.hydraulic_pressure_pa[1]) / HYDRAULIC_NOMINAL_PA).clamp(0.0, 1.0);
        rev.step(commands.reverser_deploy_commanded[rev.slot], hydraulic_frac, faults, dt);
    }

    fn step_latches(&mut self, dt: f64) {
        if !self.sov_commanded_open {
            self.fmu_fault_confirm_s = 0.0;
            self.thrust_abnormal_confirm_s = 0.0;
            self.hp_pump_low_flow_confirm_s = 0.0;
            self.fmu_fault_latched = false;
            self.thrust_abnormal_latched = false;
            self.hp_pump_low_flow_latched = false;
            return;
        }
        let error = self.metering_error_fraction();

        self.fmu_fault_confirm_s = if error > METERING_TOLERANCE { self.fmu_fault_confirm_s + dt } else { 0.0 };
        if self.fmu_fault_confirm_s >= METERING_DISAGREE_CONFIRM_S {
            self.fmu_fault_latched = true;
        }

        self.thrust_abnormal_confirm_s = if error > THRUST_ABNORMAL_TOLERANCE { self.thrust_abnormal_confirm_s + dt } else { 0.0 };
        if self.thrust_abnormal_confirm_s >= METERING_DISAGREE_CONFIRM_S {
            self.thrust_abnormal_latched = true;
        }

        self.hp_pump_low_flow_confirm_s = if self.hp_pump_low_flow() { self.hp_pump_low_flow_confirm_s + dt } else { 0.0 };
        if self.hp_pump_low_flow_confirm_s >= METERING_DISAGREE_CONFIRM_S {
            self.hp_pump_low_flow_latched = true;
        }
    }

    fn required_pump_flow_m3_s(wf_command_kg_s: f64) -> f64 {
        wf_command_kg_s.max(0.0) / super::fuel::common::FUEL_DENSITY_KG_M3
    }

    fn hp_pump_low_flow(&self) -> bool {
        let required = Self::required_pump_flow_m3_s(self.wf_command_kg_s);
        required > 0.0 && self.hp.delivered_m3_s < required * (1.0 - HP_PUMP_LOW_FLOW_TOLERANCE)
    }

    fn metering_error_fraction(&self) -> f64 {
        if self.wf_command_kg_s <= 0.0 {
            return 0.0;
        }
        (self.fmu_state.metered_kg_s - self.wf_command_kg_s).abs() / self.wf_command_kg_s
    }

    fn bearing_amplitude_mm_s(&self, b: usize) -> f64 {
        let s = &self.bearing_state[b].signature;
        s.outer_race.1.max(s.inner_race.1).max(s.ball_spin.1).max(s.cage.1)
    }
}

pub struct EngineAccessoriesLive {
    engines: Vec<EngineChain>,
    pub commands: EngineAccessoryCommands,
    to_flex_temp_set: bool,
}

impl Default for EngineAccessoriesLive {
    fn default() -> Self {
        Self::new()
    }
}

impl EngineAccessoriesLive {
    pub fn new() -> Self {
        let mut reg = Registry::default();
        super::registry::register(&mut reg);
        Self { engines: (0..N_ENGINES).map(|e| EngineChain::new(&reg, e)).collect(), commands: EngineAccessoryCommands::default(), to_flex_temp_set: false }
    }

    pub fn delivered_fuel_kg_s(&self, eng: usize) -> f64 {
        self.engines[eng].delivered_kg_s
    }

    pub fn bleed_loss_kg_s(&self, eng: usize) -> f64 {
        let c = &self.engines[eng];
        c.handling_bleed_state.iter().map(|b| b.bled_kg_s).sum::<f64>() + c.anti_ice_state.bled_kg_s
    }
}

impl crate::deep::live::Area for EngineAccessoriesLive {
    fn name(&self) -> &'static str {
        "engine_accessories"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        let dt = truth.dt_s.max(0.0);
        let mut commands = self.commands;
        commands.reverser_deploy_commanded = truth.controls.reverser_deploy_commanded;
        self.to_flex_temp_set = truth.to_flex_temp_set;
        for (eng, chain) in self.engines.iter_mut().enumerate() {
            let was_commanded = chain.start_valve_commanded_open;
            let now_commanded = truth.controls.starter_engaged[eng];
            chain.start_valve_steady_s = if was_commanded == now_commanded { chain.start_valve_steady_s + dt } else { 0.0 };
            chain.step(eng, truth, faults, &commands, dt);
        }
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        let b = |x: bool| if x { 1.0 } else { 0.0 };

        out("A32NX_TO_FLEX_TEMP_SET", b(self.to_flex_temp_set));

        for chain in self.engines.iter() {
            let n = &chain.names;

            out(&n.strainer_clogged, b(chain.strainer.bypassed));
            out(&n.filter_impending_bypass, b(chain.filter.impending_bypass));
            out(&n.filter_bypassed, b(chain.filter.bypassed));
            out(&n.filter_dp_pa, chain.filter.differential_pa);
            out(&n.filter_monitor_fault, b(chain.filter.monitor_fault));
            out(&n.oil_filter_bypassed, b(chain.gas_path_out.oil_filter_bypassed));
            let chips_active = (0..N_BEARINGS).filter(|&i| chain.bearing_state[i].chip_detected).count();
            out(&n.oil_system_contamination, b(chips_active >= 2));

            out(&n.hp_pump_low_flow, b(chain.hp_pump_low_flow_latched));
            out(&n.hp_pump_flow, chain.hp.delivered_kg_s);
            out(&n.lp_pump_outlet_pa, chain.lp.outlet_pa);
            out(&n.lp_pump_cavitating, b(chain.lp.cavitating));

            out(&n.fmu_fault, b(chain.fmu_fault_latched));
            out(&n.thrust_abnormal, b(chain.thrust_abnormal_latched));
            out(&n.fmu_metered, chain.fmu_state.metered_kg_s);
            out(&n.fmu_dp_pa, chain.fmu_state.differential_pa);
            out(&n.wf_command, chain.wf_command_kg_s);

            let sov_target = if chain.sov_commanded_open { 1.0 } else { 0.0 };
            out(&n.sov_disagree, b((chain.sov_position - sov_target).abs() > SOV_DISAGREE_TOLERANCE));
            out(&n.sov_position, chain.sov_position);

            out(&n.ff_channel_disagree, b(chain.eec_state.fuel_flow_disagree));
            let indicated = 0.5 * (chain.flow.channel_a_kg_s + chain.flow.channel_b_kg_s);
            out(&n.ff_disagree, b((indicated - chain.delivered_kg_s).abs() > FF_DISAGREE_KG_S));
            out(&n.ff_indicated, indicated);
            out(&n.ff_true, chain.flow.true_flow_kg_s);

            out(&n.nozzle_imbalance, b(chain.manifold.hot_streak_severity > NOZZLE_IMBALANCE_SEVERITY));
            out(&n.nozzle_hot_streak, chain.manifold.hot_streak_severity);
            let mean = chain.manifold.total_flow_kg_s / NUM_GROUPS as f64;
            for (name, q) in n.nozzle_group_flow_dev.iter().zip(chain.manifold.group_flow_kg_s) {
                out(name, if mean > 1e-9 { (q - mean) / mean } else { 0.0 });
            }
            out(&n.combustor_rise_fraction, chain.gas_path_out.combustor_rise_fraction);
            out(&n.manifold_gauge_pa, chain.manifold.manifold_gauge_pa);

            out(&n.eec_channel_fault, b(!(chain.eec_state.channel_a_serviceable && chain.eec_state.channel_b_serviceable)));
            out(&n.eec_no_valid_channel, b(chain.eec_state.active == ActiveChannel::None));
            out(&n.eec_sensor_disagree, b(chain.eec_state.disagree.iter().any(|&d| d)));
            for i in 0..N_EEC_PARAMS {
                let selected = chain.eec_state.selected[i];
                let shown = if PARAMS[i] == super::eec::Param::Tgt && selected > 0.0 {
                    super::eec::trimmed_tgt_c(selected - 273.15) + 273.15
                } else {
                    selected
                };
                out(&n.eec_selected[i], shown);
                out(&n.eec_disagree[i], b(chain.eec_state.disagree[i]));
                out(&n.eec_no_data[i], b(chain.eec_state.no_data[i]));
            }
            out(&n.eec_backup_probe_fault, b(chain.eec_backup_probe_fault));
            out(&n.tla_deg, chain.tla_deg);
            out(&n.tla_channel_b_deg, chain.tla_channel_b_deg);
            out(&n.tla_channel_b_disagree, b(chain.tla_channel_b_disagree));

            out(&n.ign_spark_rate[0], chain.ignition_state.chain_a_hz);
            out(&n.ign_spark_rate[1], chain.ignition_state.chain_b_hz);
            out(&n.ign_powered, b(chain.ignition_powered));
            out(&n.no_ignition_available, b(chain.ignition_state.no_ignition_available));

            out(&n.start_valve_position, chain.start_valve_position);
            out(&n.start_valve_disagree, b(chain.start_valve_disagrees()));
            out(&n.starter_torque, chain.ats_state.torque_n_m);
            out(&n.starter_rotor_rpm, chain.ats_state.rotor_rpm);
            out(&n.starter_disengage_fault, b(chain.starter_disengage_fault()));
            out(&n.starter_disintegrated, b(chain.ats_state.disintegrated));
            out(&n.starter_overheat, b(chain.starter_heat.overheated()));
            out(&n.starter_housing_rise_k, chain.starter_heat.rise_k());

            out(&n.vsv_angle, chain.vsv_state.angle_deg);
            out(&n.vsv_schedule_error, chain.vsv_state.schedule_error_deg.abs());
            out(&n.vsv_stall_margin, chain.vsv_state.stall_margin_delta_pct);

            for i in 0..N_HANDLING_BLEEDS {
                let s = &chain.handling_bleed_state[i];
                out(&n.bleed_position[i], s.position);
                out(&n.bleed_disagree[i], b((s.position - chain.handling_bleed_target[i]).abs() > HANDLING_BLEED_DISAGREE_TOLERANCE));
                out(&n.bleed_flow[i], s.bled_kg_s);
                out(&n.bleed_stall_margin[i], s.stall_margin_delta_pct);
            }

            for s in 0..N_SPOOLS {
                out(&n.vib_index[s], chain.vibration_state[s].index);
            }
            for i in 0..N_BEARINGS {
                out(&n.bearing_amplitude[i], chain.bearing_amplitude_mm_s(i));
                out(&n.bearing_chip[i], b(chain.bearing_state[i].chip_detected));
                out(&n.bearing_debris[i], chain.bearing_state[i].debris_g);
            }

            let ai_target = if chain.anti_ice_commanded_open { 1.0 } else { 0.0 };
            out(&n.anti_ice_position, chain.anti_ice_state.position);
            out(&n.anti_ice_disagree, b((chain.anti_ice_state.position - ai_target).abs() > ANTI_ICE_DISAGREE_TOLERANCE));
            out(&n.anti_ice_lip_temp_k, chain.anti_ice_state.lip_k);
            out(&n.anti_ice_bleed_flow, chain.anti_ice_state.bled_kg_s);

            out(&n.nacelle_vent_flow, chain.ventilation_state.flow_kg_s);
            out(&n.nacelle_vapour_risk, b(chain.ventilation_state.vapour_accumulation_risk && chain.engine_running));

            for z in 0..N_FIRE_ZONES {
                out(&n.fire_loop_disagree[z], b(chain.fire_reading[z].loop_disagree));
                out(&n.fire_confirmed[z], b(chain.fire_reading[z].confirmed));
            }

            let gp = &chain.gas_path_out;
            out(&n.gp_egt_delta_c, gp.egt_delta_c);
            out(&n.gp_oil_temp_delta_c, gp.oil_temp_delta_c);
            out(&n.gp_oil_press_delta_psi, gp.oil_press_delta_psi);
            out(&n.gp_oil_quantity_delta_frac, gp.oil_quantity_delta_fraction);
            out(&n.gp_stall_margin_loss_pct, gp.stall_margin_loss_pct);
            out(&n.gp_n2_capability_loss_pct, gp.n2_capability_loss_pct);
            out(&n.gp_n3_capability_loss_pct, gp.n3_capability_loss_pct);
            out(&n.gp_thrust_loss_pct, gp.thrust_loss_pct);
            out(&n.gp_surge, b(gp.surge));
            out(&n.gp_oil_temp_c, gp.oil_temp_c);
            out(&n.gp_oil_press_psi, gp.oil_press_psi);
            out(&n.gp_oil_quantity_frac, gp.oil_quantity_fraction);
            out(&n.gp_oil_supply_c, gp.oil_supply_c);
            out(&n.gp_oil_filter_bypassed, b(gp.oil_filter_bypassed));
            out(&n.gp_oil_relief_open, b(gp.oil_relief_open));
            out(&n.gp_fuel_heat_w, gp.fuel_heat_w);
            out(&n.gp_fuel_out_c, gp.fuel_out_c);
            out(&n.gp_hot_section_soak_c, gp.hot_section_soak_c);
            out(&n.gp_egt_shadow_c, gp.egt_shadow_c);
            out(&n.case_breach, chain.debris_breach_frac);
            out(&n.nacelle_fuel_leak, chain.nacelle_fuel_leak_kg_s);
            let fe = &chain.free_out;
            out(&n.phys_n1, fe.n1_pct);
            out(&n.phys_n2, fe.n2_pct);
            out(&n.phys_n3, fe.n3_pct);
            out(&n.phys_egt, fe.egt_c);
            out(&n.phys_ff, fe.wf_kg_s);
            out(&n.phys_lit, b(fe.lit));
            out(&n.phys_thrust, fe.net_thrust_n);
            out(&n.phys_valid, b(chain.free_ready));
            out(&n.gp_ip_port_pressure_pa, gp.ip_port_pressure_pa);
            out(&n.gp_ip_port_temp_k, gp.ip_port_temp_k);
            out(&n.bearing_seizure_frac, chain.bearing_seizure_frac);

            if let Some(rev) = &chain.reverser {
                let target = if rev.commanded { 1.0 } else { 0.0 };
                out(&rev.names.position, rev.state.position);
                out(&rev.names.uncommanded, b(rev.state.uncommanded_deployment));
                out(&rev.names.position_disagree, b((rev.state.position - target).abs() > REVERSER_DISAGREE_TOLERANCE));
                out(&rev.names.energized, b(rev.commanded));
                out(&rev.names.ctl_fault, b(rev.state.control_fault_active));
                out(&rev.names.lock_degraded_count, f64::from(rev.state.lock_degraded_count));
                let mel_failure_id = 78_000 + (REVERSER_ENGINES[rev.slot] as u64 - 1);
                out(&rev.names.mel_inop, b(crate::mel::deferred_state(mel_failure_id).is_some()));
            }
        }
    }
}

pub fn live_system() -> Box<dyn crate::deep::live::Area> {
    Box::new(EngineAccessoriesLive::new())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::fuel::live::test_support::collect_vars;
    use crate::deep::live::{Area as _, Controls, PublishedFrame};
    use std::collections::BTreeMap;

    const DESIGN_WF_KG_S: f64 = 2.48;

    fn published(area: &dyn crate::deep::live::Area) -> BTreeMap<String, f64> {
        let mut map = BTreeMap::new();
        area.publish(&mut |name, value| {
            map.insert(name.to_string(), value);
        });
        map
    }

    fn running() -> (Truth, EngineAccessoryCommands) {
        let truth = Truth {
            dt_s: 0.02,
            engine_running: [true; 4],
            engine_n1_frac: [0.85; 4],
            engine_n2_frac: [0.88; 4],
            engine_n3_frac: [0.9; 4],
            engine_n1_commanded_pct: [85.0; 4],
            engine_fuel_flow_kg_s: [DESIGN_WF_KG_S; 4],
            engine_bleed_pressure_pa: [300_000.0; 4],
            engine_bleed_temp_k: [520.0; 4],
            engine_hp_port_pressure_pa: [2.5e6; 4],
            engine_hp_port_temp_k: [700.0; 4],
            ac_bus_volts: [115.0; 4],
            hydraulic_pressure_pa: [HYDRAULIC_NOMINAL_PA; 2],
            controls: Controls { engine_master_on: [true; 4], ..Controls::default() },
            ..Truth::default()
        };
        let commands = EngineAccessoryCommands { engine_tgt_k: Some([900.0; 4]), fuel_inlet_k: Some(300.0), ..EngineAccessoryCommands::default() };
        (truth, commands)
    }

    fn run(live: &mut EngineAccessoriesLive, truth: &Truth, faults: &Faults, seconds: f64) -> BTreeMap<String, f64> {
        let steps = (seconds / truth.dt_s).ceil() as usize;
        for _ in 0..steps.max(1) {
            live.tick(truth, faults);
        }
        published(live)
    }

    fn settled(faults: &Faults) -> BTreeMap<String, f64> {
        let (truth, commands) = running();
        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        run(&mut live, &truth, faults, 30.0)
    }

    fn registry() -> Registry {
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        reg
    }

    #[test]
    fn engines_msfs_only_reports_running_once_the_flight_is_ready_are_picked_up_running() {
        let (mut truth, commands) = running();
        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        truth.flight_ready = false;
        truth.engine_running = [false; 4];
        truth.engine_n3_frac = [0.0; 4];
        truth.sim_engine_corrected_n2_pct = [0.0; 4];
        let loading = run(&mut live, &truth, &Faults::default(), 20.0);
        assert_eq!(loading.get("A32NX_ENG_1_PHYS_VALID"), Some(&0.0), "nothing is decided about the engines while the flight is still loading");

        truth.flight_ready = true;
        truth.engine_running = [true; 4];
        truth.engine_n3_frac = [0.65; 4];
        truth.sim_engine_corrected_n2_pct = [65.0; 4];
        let out = run(&mut live, &truth, &Faults::default(), 5.0);
        for n in 1..=4 {
            assert_eq!(out.get(&format!("A32NX_ENG_{n}_PHYS_LIT")), Some(&1.0), "engine {n} was running when the flight became ready");
        }
    }

    #[test]
    fn engines_still_stopped_three_seconds_after_the_flight_is_ready_are_cold() {
        let (mut truth, commands) = running();
        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        truth.engine_running = [false; 4];
        truth.engine_n3_frac = [0.0; 4];
        truth.sim_engine_corrected_n2_pct = [0.0; 4];
        let out = run(&mut live, &truth, &Faults::default(), 4.0);
        assert_eq!(out.get("A32NX_ENG_1_PHYS_VALID"), Some(&1.0));
        assert_eq!(out.get("A32NX_ENG_1_PHYS_LIT"), Some(&0.0));
    }

    #[test]
    fn the_governor_command_is_the_engines_own_fuel_flow_not_a_constant_zero() {
        let out = settled(&Faults::default());
        for n in 1..=4 {
            assert!(out[&format!("A32NX_ENG_{n}_WF_COMMAND_KG_S")] > 1.0, "engine {n} should be asked for the engine model's own flow, not {}", out[&format!("A32NX_ENG_{n}_WF_COMMAND_KG_S")]);
        }

        let (mut truth, commands) = running();
        truth.on_ground = false;
        truth.engine_n1_commanded_pct = [40.0, 60.0, 80.0, 95.0];
        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        let out = run(&mut live, &truth, &Faults::default(), 30.0);
        let mut prev_wf = 0.0;
        for n in 1..=4 {
            let wf = out[&format!("A32NX_ENG_{n}_WF_COMMAND_KG_S")];
            assert!(wf > prev_wf, "engine {n} commanding {wf} should rise with a higher N1 target than the previous engine's {prev_wf}");
            assert!((out[&format!("A32NX_ENG_{n}_FMU_METERED_KG_S")] - wf).abs() < 0.05, "engine {n} metered {} against its own {wf} command", out[&format!("A32NX_ENG_{n}_FMU_METERED_KG_S")]);
            prev_wf = wf;
        }
    }

    #[test]
    fn four_healthy_engines_meter_what_the_governor_asks_and_raise_nothing() {
        let out = settled(&Faults::default());
        for n in 1..=4 {
            let wf = out[&format!("A32NX_ENG_{n}_WF_COMMAND_KG_S")];
            assert!(wf > 1.0, "engine {n} should have a real commanded flow, not {wf}");
            assert!(
                (out[&format!("A32NX_ENG_{n}_FMU_METERED_KG_S")] - wf).abs() < 0.05,
                "engine {n} metered {} against its own {wf} command",
                out[&format!("A32NX_ENG_{n}_FMU_METERED_KG_S")]
            );
            for v in ["FMU_FAULT", "THRUST_ABNORMAL", "HP_PUMP_LOW_FLOW", "FUEL_FILTER_BYPASSED", "FUEL_FILTER_IMPENDING_BYPASS", "HP_SOV_DISAGREE", "FF_CHANNEL_DISAGREE", "FF_DISAGREE", "NOZZLE_IMBALANCE", "EEC_CHANNEL_FAULT", "EEC_NO_VALID_CHANNEL", "EEC_SENSOR_DISAGREE"] {
                assert_eq!(out.get(&format!("A32NX_ENG_{n}_{v}")), Some(&0.0), "engine {n} {v} should be healthy");
            }
        }
    }

    #[test]
    fn throttle_slams_on_healthy_engines_never_latch_a_metering_or_pump_fault() {
        let (mut truth, commands) = running();
        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        run(&mut live, &truth, &Faults::default(), 30.0);
        for target in [20.0, 100.0, 20.0, 100.0, 60.0, 20.0] {
            truth.engine_n1_commanded_pct = [target; 4];
            let out = run(&mut live, &truth, &Faults::default(), 8.0);
            for n in 1..=4 {
                for v in ["FMU_FAULT", "THRUST_ABNORMAL", "HP_PUMP_LOW_FLOW"] {
                    assert_eq!(out.get(&format!("A32NX_ENG_{n}_{v}")), Some(&0.0), "engine {n} {v} latched on a healthy throttle move to {target}% N1");
                }
            }
        }
    }

    #[test]
    fn a_latched_metering_fault_clears_once_the_engine_master_is_switched_off() {
        let (mut truth, commands) = running();
        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        let stuck = live.engines[0].fuel_ids.fmu_sticking;
        run(&mut live, &truth, &Faults::default(), 30.0);
        let faulted = run(&mut live, &truth, &Faults::from_pairs([(stuck, 1.0)]), 1.0);
        truth.engine_n1_commanded_pct = [20.0; 4];
        let faulted = { let _ = faulted; run(&mut live, &truth, &Faults::from_pairs([(stuck, 1.0)]), 10.0) };
        assert_eq!(faulted.get("A32NX_ENG_1_FMU_FAULT"), Some(&1.0), "a stuck metering valve must latch FMU_FAULT once the command moves away from it");
        truth.controls.engine_master_on[0] = false;
        let off = run(&mut live, &truth, &Faults::default(), 2.0);
        assert_eq!(off.get("A32NX_ENG_1_FMU_FAULT"), Some(&0.0), "switching the engine master off resets the FADEC's metering monitor");
    }

    #[test]
    fn every_variable_this_areas_alerts_trigger_on_is_published() {
        let reg = registry();
        let mut names = Vec::new();
        for alert in &reg.alerts {
            collect_vars(&alert.trigger, &mut names);
            for line in &alert.procedure {
                collect_vars(&line.applies_if, &mut names);
            }
        }
        assert!(!names.is_empty(), "this area's alerts and procedure conditions read no variables at all");
        let out = settled(&Faults::default());
        let mut checked = 0usize;
        for name in names {
            if !name.starts_with("A32NX_ENG_") {
                continue;
            }
            assert!(out.contains_key(&name), "an alert in this area reads {name}, which this live system does not publish");
            checked += 1;
        }
        assert!(checked > 0, "no trigger variable of this area was checked");
    }

    #[test]
    fn a_clogging_filter_warns_before_it_bypasses_and_then_bypasses() {
        let live = EngineAccessoriesLive::new();
        let id = live.engines[0].fuel_ids.filter_clog;

        let clean = settled(&Faults::default());
        assert_eq!(clean.get("A32NX_ENG_1_FUEL_FILTER_IMPENDING_BYPASS"), Some(&0.0));

        let mut worst_dp = 0.0;
        let mut impending_at = None;
        let mut bypass_at = None;
        for step in 0..=20 {
            let clog = step as f64 / 20.0;
            let out = settled(&Faults::from_pairs([(id, clog)]));
            let dp = out["A32NX_ENG_1_FUEL_FILTER_DP_PA"];
            assert!(dp >= worst_dp - 1e-9, "differential must rise monotonically with clog");
            worst_dp = dp;
            if impending_at.is_none() && out["A32NX_ENG_1_FUEL_FILTER_IMPENDING_BYPASS"] > 0.0 {
                impending_at = Some(clog);
            }
            if bypass_at.is_none() && out["A32NX_ENG_1_FUEL_FILTER_BYPASSED"] > 0.0 {
                bypass_at = Some(clog);
            }
        }
        let impending = impending_at.expect("a filter clogging toward fully blocked must warn");
        let bypass = bypass_at.expect("and must eventually bypass");
        assert!(impending < bypass, "the warning must come before the bypass: {impending} vs {bypass}");
    }

    #[test]
    fn an_fmu_spill_valve_stuck_open_starves_the_burners_and_reports_abnormal_thrust() {
        let live = EngineAccessoriesLive::new();
        let id = live.engines[1].fuel_ids.fmu_spill_open;
        let out = settled(&Faults::from_pairs([(id, 1.0)]));
        assert!(out["A32NX_ENG_2_FMU_METERED_KG_S"] < 0.1 * DESIGN_WF_KG_S, "a collapsed differential must starve the burners");
        assert_eq!(out.get("A32NX_ENG_2_FMU_FAULT"), Some(&1.0));
        assert_eq!(out.get("A32NX_ENG_2_THRUST_ABNORMAL"), Some(&1.0));
        assert_eq!(out.get("A32NX_ENG_1_FMU_FAULT"), Some(&0.0), "the other engines are untouched");
    }

    #[test]
    fn an_fmu_metering_valve_stuck_shut_cannot_follow_a_command_at_all() {
        let live = EngineAccessoriesLive::new();
        let id = live.engines[0].fuel_ids.fmu_sticking;
        let out = settled(&Faults::from_pairs([(id, 1.0)]));
        assert!(out["A32NX_ENG_1_FMU_METERED_KG_S"] < 1e-6, "a fully seized valve cannot open");
        assert_eq!(out.get("A32NX_ENG_1_FMU_FAULT"), Some(&1.0));
    }

    #[test]
    fn a_worn_hp_pump_slips_and_eventually_cannot_supply_the_commanded_flow() {
        let live = EngineAccessoriesLive::new();
        let id = live.engines[2].fuel_ids.hp_wear;
        let healthy = settled(&Faults::default());
        let worn = settled(&Faults::from_pairs([(id, 0.9)]));
        assert!(
            worn["A32NX_ENG_3_HP_PUMP_FLOW_KG_S"] < healthy["A32NX_ENG_3_HP_PUMP_FLOW_KG_S"],
            "wear must cost delivered flow: {} vs {}",
            worn["A32NX_ENG_3_HP_PUMP_FLOW_KG_S"],
            healthy["A32NX_ENG_3_HP_PUMP_FLOW_KG_S"]
        );
        let starved = settled(&Faults::from_pairs([(live.engines[2].fuel_ids.hp_starvation, 1.0)]));
        assert_eq!(starved.get("A32NX_ENG_3_HP_PUMP_LOW_FLOW"), Some(&1.0), "a totally starved pump must post its low-flow warning");
        assert_eq!(starved.get("A32NX_ENG_4_HP_PUMP_LOW_FLOW"), Some(&0.0));
    }

    #[test]
    fn a_stuck_hp_shutoff_valve_cannot_follow_the_master_switch() {
        let (mut truth, commands) = running();
        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        let id = live.engines[0].fuel_ids.sov_stuck;

        let open = run(&mut live, &truth, &Faults::default(), 10.0);
        assert!(open["A32NX_ENG_1_HP_SOV_POSITION"] > 0.95);
        assert_eq!(open.get("A32NX_ENG_1_HP_SOV_DISAGREE"), Some(&0.0));
        truth.controls.fire_pb_released[0] = true;
        let shut = run(&mut live, &truth, &Faults::default(), 10.0);
        assert!(shut["A32NX_ENG_1_HP_SOV_POSITION"] < 0.05);

        let (mut truth, commands) = running();
        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        run(&mut live, &truth, &Faults::default(), 10.0);
        truth.controls.fire_pb_released[0] = true;
        let seized = run(&mut live, &truth, &Faults::from_pairs([(id, 1.0)]), 10.0);
        assert!(seized["A32NX_ENG_1_HP_SOV_POSITION"] > 0.95, "a seized valve does not move");
        assert_eq!(seized.get("A32NX_ENG_1_HP_SOV_DISAGREE"), Some(&1.0));
    }

    #[test]
    fn a_biased_flow_transmitter_channel_makes_the_two_channels_disagree() {
        let live = EngineAccessoriesLive::new();
        let id = live.engines[3].fuel_ids.ft_a_bias;
        let out = settled(&Faults::from_pairs([(id, 1.0)]));
        assert_eq!(out.get("A32NX_ENG_4_FF_CHANNEL_DISAGREE"), Some(&1.0));
        assert_eq!(out.get("A32NX_ENG_3_FF_CHANNEL_DISAGREE"), Some(&0.0));
        assert!(out["A32NX_ENG_4_FF_INDICATED_KG_S"] > out["A32NX_ENG_3_FF_INDICATED_KG_S"], "a positive bias must read high");
    }

    #[test]
    fn a_frozen_flow_transmitter_channel_stops_tracking_a_real_flow() {
        let live = EngineAccessoriesLive::new();
        let id = live.engines[0].fuel_ids.ft_a_frozen;
        let healthy = settled(&Faults::default());
        let frozen = settled(&Faults::from_pairs([(id, 1.0)]));
        assert!(healthy["A32NX_ENG_1_FF_INDICATED_KG_S"] > 0.5 * DESIGN_WF_KG_S);
        assert!(frozen["A32NX_ENG_1_FF_INDICATED_KG_S"] < healthy["A32NX_ENG_1_FF_INDICATED_KG_S"], "a frozen channel drags the mean indication down");
        assert_eq!(frozen.get("A32NX_ENG_1_FF_CHANNEL_DISAGREE"), Some(&1.0));
    }

    #[test]
    fn a_coked_nozzle_group_skews_the_manifold_and_raises_a_hot_streak() {
        let live = EngineAccessoriesLive::new();
        let id = live.engines[0].fuel_ids.nozzle[3];
        let clean = settled(&Faults::default());
        let coked = settled(&Faults::from_pairs([(id, 0.8)]));
        assert_eq!(clean.get("A32NX_ENG_1_NOZZLE_IMBALANCE"), Some(&0.0));
        assert_eq!(coked.get("A32NX_ENG_1_NOZZLE_IMBALANCE"), Some(&1.0));
        assert!(coked["A32NX_ENG_1_NOZZLE_HOT_STREAK"] > clean["A32NX_ENG_1_NOZZLE_HOT_STREAK"]);
        assert!(coked["A32NX_ENG_1_MANIFOLD_GAUGE_PA"] > clean["A32NX_ENG_1_MANIFOLD_GAUGE_PA"], "the surviving groups must pass the difference at a higher pressure");
    }

    #[test]
    fn one_dead_eec_channel_hands_over_and_two_leave_no_valid_channel() {
        let live = EngineAccessoriesLive::new();
        let (a, b) = (live.engines[1].fuel_ids.eec_channel_a, live.engines[1].fuel_ids.eec_channel_b);

        let one = settled(&Faults::from_pairs([(a, 1.0)]));
        assert_eq!(one.get("A32NX_ENG_2_EEC_CHANNEL_FAULT"), Some(&1.0));
        assert_eq!(one.get("A32NX_ENG_2_EEC_NO_VALID_CHANNEL"), Some(&0.0), "channel B is still flying the engine");

        let standby = settled(&Faults::from_pairs([(b, 1.0)]));
        assert_eq!(standby.get("A32NX_ENG_2_EEC_CHANNEL_FAULT"), Some(&1.0), "ENG 2 EEC CHANNEL FAULT must be reachable from channel B too");
        assert_eq!(standby.get("A32NX_ENG_2_EEC_NO_VALID_CHANNEL"), Some(&0.0));
        assert_eq!(standby.get("A32NX_ENG_1_EEC_CHANNEL_FAULT"), Some(&0.0));

        let both = settled(&Faults::from_pairs([(a, 1.0), (b, 1.0)]));
        assert_eq!(both.get("A32NX_ENG_2_EEC_NO_VALID_CHANNEL"), Some(&1.0));
        assert_eq!(both.get("A32NX_ENG_1_EEC_NO_VALID_CHANNEL"), Some(&0.0));
    }

    #[test]
    fn a_drifting_eec_sensor_is_caught_by_the_channel_disagree_monitor() {
        let live = EngineAccessoriesLive::new();
        let tgt_a = live.engines[0].fuel_ids.eec_sensor_a[3];
        let out = settled(&Faults::from_pairs([(tgt_a, 1.0)]));
        assert_eq!(out.get("A32NX_ENG_1_EEC_SENSOR_DISAGREE"), Some(&1.0));
        assert_eq!(out.get("A32NX_ENG_1_EEC_TGT_DISAGREE"), Some(&1.0));
        assert_eq!(out.get("A32NX_ENG_1_EEC_N1_DISAGREE"), Some(&0.0), "only the failed parameter disagrees");
        assert_eq!(out.get("A32NX_ENG_2_EEC_SENSOR_DISAGREE"), Some(&0.0));
    }

    fn starting_truth() -> Truth {
        Truth {
            dt_s: 0.1,
            on_ground: true,
            engine_running: [false; 4],
            engine_n1_frac: [0.08; 4],
            engine_n2_frac: [0.25; 4],
            engine_n3_frac: [0.22; 4],
            engine_fuel_flow_kg_s: [0.05; 4],
            engine_bleed_pressure_pa: [130_000.0; 4],
            engine_bleed_temp_k: [400.0; 4],
            engine_hp_port_pressure_pa: [200_000.0; 4],
            engine_hp_port_temp_k: [450.0; 4],
            ac_bus_volts: [115.0; 4],
            apu_running: true,
            apu_bleed_pressure_pa: 310_000.0,
            controls: Controls { engine_master_on: [true; 4], starter_engaged: [true; 4], apu_bleed_pb_on: true, ..Controls::default() },
            published: (1..=4)
                .map(|n| (format!("DEEP_PNEU_ENG_{n}_START_DUCT_PRESSURE_PA"), 310_000.0))
                .collect::<PublishedFrame>(),
            ..Truth::default()
        }
    }

    fn start_run(faults: &Faults, seconds: f64) -> BTreeMap<String, f64> {
        let truth = starting_truth();
        let mut live = EngineAccessoriesLive::new();
        run(&mut live, &truth, faults, seconds)
    }

    #[test]
    fn both_ignition_chains_spark_during_a_start_and_neither_does_when_nothing_selects_them() {
        let out = start_run(&Faults::default(), 5.0);
        for n in 1..=4 {
            assert_eq!(out.get(&format!("A32NX_ENG_{n}_IGN_POWERED")), Some(&1.0));
            assert!(out[&format!("A32NX_ENG_{n}_IGN_A_SPARK_RATE_HZ")] > 0.0);
            assert!(out[&format!("A32NX_ENG_{n}_IGN_B_SPARK_RATE_HZ")] > 0.0);
            assert_eq!(out.get(&format!("A32NX_ENG_{n}_NO_IGNITION_AVAILABLE")), Some(&0.0));
        }

        let cruise = settled(&Faults::default());
        assert_eq!(cruise.get("A32NX_ENG_1_IGN_POWERED"), Some(&0.0));
        assert_eq!(cruise.get("A32NX_ENG_1_IGN_A_SPARK_RATE_HZ"), Some(&0.0));
        assert_eq!(cruise.get("A32NX_ENG_1_NO_IGNITION_AVAILABLE"), Some(&0.0));
    }

    #[test]
    fn one_dead_exciter_halves_the_ignition_and_both_dead_raise_the_ignition_fault() {
        let reg = registry();
        let ids = IgnitionIds::resolve(&reg, 1);

        let one = start_run(&Faults::from_pairs([(ids.exciter[0], 1.0)]), 5.0);
        assert_eq!(one.get("A32NX_ENG_2_IGN_A_SPARK_RATE_HZ"), Some(&0.0));
        assert!(one["A32NX_ENG_2_IGN_B_SPARK_RATE_HZ"] > 0.0, "the other chain is untouched");
        assert_eq!(one.get("A32NX_ENG_2_NO_IGNITION_AVAILABLE"), Some(&0.0), "one chain alone still lights the engine");

        let degraded = start_run(&Faults::from_pairs([(ids.exciter[0], 0.6)]), 5.0);
        let healthy = start_run(&Faults::default(), 5.0);
        assert!(degraded["A32NX_ENG_2_IGN_A_SPARK_RATE_HZ"] > 0.0);
        assert!(degraded["A32NX_ENG_2_IGN_A_SPARK_RATE_HZ"] < healthy["A32NX_ENG_2_IGN_A_SPARK_RATE_HZ"]);

        let both = start_run(&Faults::from_pairs([(ids.exciter[0], 1.0), (ids.exciter[1], 1.0)]), 5.0);
        assert_eq!(both.get("A32NX_ENG_2_NO_IGNITION_AVAILABLE"), Some(&1.0));
        assert_eq!(both.get("A32NX_ENG_1_NO_IGNITION_AVAILABLE"), Some(&0.0));
    }

    #[test]
    fn a_fully_eroded_igniter_plug_stops_firing_and_two_of_them_lose_ignition() {
        let reg = registry();
        let ids = IgnitionIds::resolve(&reg, 0);
        let one = start_run(&Faults::from_pairs([(ids.igniter[0], 1.0)]), 5.0);
        assert_eq!(one.get("A32NX_ENG_1_IGN_A_SPARK_RATE_HZ"), Some(&0.0));
        assert_eq!(one.get("A32NX_ENG_1_NO_IGNITION_AVAILABLE"), Some(&0.0));
        let both = start_run(&Faults::from_pairs([(ids.igniter[0], 1.0), (ids.igniter[1], 1.0)]), 5.0);
        assert_eq!(both.get("A32NX_ENG_1_NO_IGNITION_AVAILABLE"), Some(&1.0));
    }

    #[test]
    fn a_healthy_start_opens_the_valve_drives_the_starter_and_reports_no_fault() {
        let out = start_run(&Faults::default(), 10.0);
        for n in 1..=4 {
            assert!(out[&format!("A32NX_ENG_{n}_START_VALVE_POSITION")] > 0.95, "the valve should be open for a start");
            assert_eq!(out.get(&format!("A32NX_ENG_{n}_START_VALVE_DISAGREE")), Some(&0.0));
            assert!(out[&format!("A32NX_ENG_{n}_STARTER_TORQUE_NM")] > 0.0, "the turbine should be driving the spool");
            assert_eq!(out.get(&format!("A32NX_ENG_{n}_STARTER_DISENGAGE_FAULT")), Some(&0.0));
            assert_eq!(out.get(&format!("A32NX_ENG_{n}_STARTER_DISINTEGRATED")), Some(&0.0));
            assert_eq!(out.get(&format!("A32NX_ENG_{n}_STARTER_OVERHEAT")), Some(&0.0));
            assert!(out[&format!("A32NX_ENG_{n}_STARTER_HOUSING_RISE_K")] > 0.0, "cranking heats the gearbox");
        }
    }

    #[test]
    fn a_stuck_starter_air_valve_never_opens_and_the_position_monitor_says_so() {
        let reg = registry();
        let ids = StartIds::resolve(&reg, 2);
        let out = start_run(&Faults::from_pairs([(ids.sav_stuck, 1.0)]), 10.0);
        assert_eq!(out.get("A32NX_ENG_3_START_VALVE_POSITION"), Some(&0.0));
        assert_eq!(out.get("A32NX_ENG_3_START_VALVE_DISAGREE"), Some(&1.0), "ENG 3 START VALVE FAULT must be reachable");
        assert_eq!(out.get("A32NX_ENG_3_STARTER_TORQUE_NM"), Some(&0.0), "no air, no torque");
        assert_eq!(out.get("A32NX_ENG_4_START_VALVE_DISAGREE"), Some(&0.0));
    }

    #[test]
    fn the_valve_position_monitor_gives_a_healthy_valve_its_own_travel_time() {
        let truth = starting_truth();
        let mut live = EngineAccessoriesLive::new();
        let early = run(&mut live, &truth, &Faults::default(), 1.0);
        assert!(early["A32NX_ENG_1_START_VALVE_POSITION"] < 0.95, "the valve is still on its way");
        assert_eq!(early.get("A32NX_ENG_1_START_VALVE_DISAGREE"), Some(&0.0), "a valve still travelling is not a faulty valve");
    }

    #[test]
    fn a_clutch_that_will_not_engage_transmits_less_torque_to_the_spool() {
        let reg = registry();
        let ids = StartIds::resolve(&reg, 0);
        let healthy = start_run(&Faults::default(), 10.0);
        let hung = start_run(&Faults::from_pairs([(ids.clutch_engage, 0.9)]), 10.0);
        assert!(hung["A32NX_ENG_1_STARTER_TORQUE_NM"] < 0.2 * healthy["A32NX_ENG_1_STARTER_TORQUE_NM"], "{} vs {}", hung["A32NX_ENG_1_STARTER_TORQUE_NM"], healthy["A32NX_ENG_1_STARTER_TORQUE_NM"]);
    }

    #[test]
    fn a_clutch_that_will_not_disengage_drags_the_starter_at_spool_speed_and_then_bursts_it() {
        let reg = registry();
        let ids = StartIds::resolve(&reg, 1);

        let (truth, commands) = running();
        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        let out = run(&mut live, &truth, &Faults::from_pairs([(ids.clutch_disengage, 1.0)]), 2.0);
        assert_eq!(out.get("A32NX_ENG_2_STARTER_DISENGAGE_FAULT"), Some(&1.0), "ENG 2 STARTER FAULT must be reachable");
        assert!(out["A32NX_ENG_2_STARTER_TORQUE_NM"] < 0.0, "a coupled sprag drags the spool, it does not drive it");
        assert_eq!(out.get("A32NX_ENG_1_STARTER_DISENGAGE_FAULT"), Some(&0.0));
        assert_eq!(out.get("A32NX_ENG_2_STARTER_DISINTEGRATED"), Some(&0.0), "0.90 N3 is below the rotor's burst margin");

        let mut takeoff = truth.clone();
        takeoff.engine_n3_frac = [1.0; 4];
        let mut live = EngineAccessoriesLive::new();
        let out = run(&mut live, &takeoff, &Faults::from_pairs([(ids.clutch_disengage, 1.0)]), 2.0);
        assert_eq!(out.get("A32NX_ENG_2_STARTER_DISINTEGRATED"), Some(&1.0));
        assert_eq!(out.get("A32NX_ENG_3_STARTER_DISINTEGRATED"), Some(&0.0), "only the engine whose clutch failed");
    }

    #[test]
    fn continuous_cranking_eventually_overheats_the_starter_and_a_normal_start_does_not() {
        let short = start_run(&Faults::default(), 60.0);
        assert_eq!(short.get("A32NX_ENG_1_STARTER_OVERHEAT"), Some(&0.0), "one normal-length crank must not overheat it");
        let long = start_run(&Faults::default(), 900.0);
        assert_eq!(long.get("A32NX_ENG_1_STARTER_OVERHEAT"), Some(&1.0), "ENG 1 STARTER OVERHEAT must be reachable");
        assert!(long["A32NX_ENG_1_STARTER_HOUSING_RISE_K"] > short["A32NX_ENG_1_STARTER_HOUSING_RISE_K"]);
    }

    #[test]
    fn healthy_vanes_and_bleeds_settle_on_their_schedules() {
        let out = settled(&Faults::default());
        for n in 1..=4 {
            assert!(out[&format!("A32NX_ENG_{n}_VSV_SCHEDULE_ERROR_DEG")] < 0.5);
            assert_eq!(out.get(&format!("A32NX_ENG_{n}_IP_HANDLING_BLEED_DISAGREE")), Some(&0.0));
            assert_eq!(out.get(&format!("A32NX_ENG_{n}_HP_HANDLING_BLEED_DISAGREE")), Some(&0.0));
            assert!(out[&format!("A32NX_ENG_{n}_IP_HANDLING_BLEED_KG_S")] < 1e-6);
        }
    }

    #[test]
    fn a_jammed_vsv_actuator_drifts_off_schedule_and_costs_stall_margin() {
        let reg = registry();
        let ids = AirflowIds::resolve(&reg, 0);
        let out = settled(&Faults::from_pairs([(ids.vsv_jam, 1.0)]));
        assert!(out["A32NX_ENG_1_VSV_SCHEDULE_ERROR_DEG"] > 10.0, "ENG 1 VSV FAULT must be reachable: {}", out["A32NX_ENG_1_VSV_SCHEDULE_ERROR_DEG"]);
        assert!(out["A32NX_ENG_1_VSV_STALL_MARGIN_DELTA_PCT"] < -1.0, "margin must be lost, not gained");
        assert!(out["A32NX_ENG_2_VSV_SCHEDULE_ERROR_DEG"] < 0.5, "the other engines stay on schedule");
    }

    #[test]
    fn a_vsv_rigging_error_settles_off_schedule_by_the_bias_and_also_costs_margin() {
        let reg = registry();
        let ids = AirflowIds::resolve(&reg, 3);
        let half = settled(&Faults::from_pairs([(ids.vsv_rigging, 0.5)]));
        let full = settled(&Faults::from_pairs([(ids.vsv_rigging, 1.0)]));
        assert!(half["A32NX_ENG_4_VSV_SCHEDULE_ERROR_DEG"] > 5.0);
        assert!(full["A32NX_ENG_4_VSV_SCHEDULE_ERROR_DEG"] > half["A32NX_ENG_4_VSV_SCHEDULE_ERROR_DEG"], "a bigger rigging error is a bigger schedule error");
        assert!(full["A32NX_ENG_4_VSV_SCHEDULE_ERROR_DEG"] > 10.0, "ENG 4 VSV FAULT must be reachable from a rigging error too");
        assert!(full["A32NX_ENG_4_VSV_STALL_MARGIN_DELTA_PCT"] < half["A32NX_ENG_4_VSV_STALL_MARGIN_DELTA_PCT"]);
    }

    #[test]
    fn a_handling_bleed_jammed_open_at_cruise_keeps_dumping_core_air() {
        let reg = registry();
        for (i, key) in BLEED_KEYS.iter().enumerate() {
            let ids = AirflowIds::resolve(&reg, 0);
            let out = settled(&Faults::from_pairs([(ids.bleed_jam[i], 1.0)]));
            assert_eq!(out.get(&format!("A32NX_ENG_1_{key}_HANDLING_BLEED_DISAGREE")), Some(&1.0), "ENG 1 {key} BLEED VALVE FAULT must be reachable");
            assert!(out[&format!("A32NX_ENG_1_{key}_HANDLING_BLEED_KG_S")] > 0.1, "a valve jammed open at cruise is still bleeding: {}", out[&format!("A32NX_ENG_1_{key}_HANDLING_BLEED_KG_S")]);
            assert_eq!(out.get(&format!("A32NX_ENG_2_{key}_HANDLING_BLEED_DISAGREE")), Some(&0.0));
        }
    }

    #[test]
    fn a_balanced_healthy_engine_reads_no_vibration_and_no_bearing_tone() {
        let out = settled(&Faults::default());
        for n in 1..=4 {
            for s in SPOOL_KEYS {
                assert_eq!(out.get(&format!("A32NX_ENG_{n}_{s}_VIB_INDEX")), Some(&0.0));
            }
            for (b, key) in BEARING_KEYS.iter().enumerate() {
                let upper = key.to_uppercase();
                assert_eq!(out.get(&format!("A32NX_ENG_{n}_{}_{upper}_DEFECT_AMPLITUDE_MM_S", BEARING_VAR_SPOOL[b])), Some(&0.0));
                assert_eq!(out.get(&format!("A32NX_ENG_{n}_{upper}_CHIP_DETECTED")), Some(&0.0));
            }
        }
    }

    #[test]
    fn a_blade_loss_raises_that_spools_vibration_index_past_its_alert_threshold() {
        let reg = registry();
        let ids = RotorIds::resolve(&reg, 0);
        for (s, key) in SPOOL_KEYS.iter().enumerate() {
            let out = settled(&Faults::from_pairs([(ids.imbalance[s][0], 1.0)]));
            assert!(out[&format!("A32NX_ENG_1_{key}_VIB_INDEX")] > 4.0, "ENG 1 {key} VIB HI must be reachable: {}", out[&format!("A32NX_ENG_1_{key}_VIB_INDEX")]);
            for other in SPOOL_KEYS.iter().filter(|o| *o != key) {
                assert_eq!(out.get(&format!("A32NX_ENG_1_{other}_VIB_INDEX")), Some(&0.0), "only the damaged spool vibrates");
            }
            assert_eq!(out.get(&format!("A32NX_ENG_2_{key}_VIB_INDEX")), Some(&0.0));
        }
    }

    #[test]
    fn ice_and_bird_strike_add_eccentric_mass_the_same_way_a_blade_loss_does() {
        let reg = registry();
        let ids = RotorIds::resolve(&reg, 1);
        let blade = settled(&Faults::from_pairs([(ids.imbalance[0][0], 0.5)]));
        let ice = settled(&Faults::from_pairs([(ids.imbalance[0][1], 0.5)]));
        let bird = settled(&Faults::from_pairs([(ids.imbalance[0][2], 0.5)]));
        let v = |m: &BTreeMap<String, f64>| m["A32NX_ENG_2_N1_VIB_INDEX"];
        assert!(v(&blade) > 0.0);
        assert!((v(&ice) - v(&blade)).abs() < 1e-9, "the same eccentric mass gives the same vibration whatever put it there");
        assert!((v(&bird) - v(&blade)).abs() < 1e-9);
    }

    #[test]
    fn each_bearing_defect_rings_on_its_own_bearing_and_trips_its_own_alert() {
        let reg = registry();
        let ids = RotorIds::resolve(&reg, 2);
        for (bi, key) in BEARING_KEYS.iter().enumerate() {
            for d in 0..4 {
                let out = settled(&Faults::from_pairs([(ids.bearing[bi][d], 1.0)]));
                let upper = key.to_uppercase();
                let name = format!("A32NX_ENG_3_{}_{upper}_DEFECT_AMPLITUDE_MM_S", BEARING_VAR_SPOOL[bi]);
                assert!(out[&name] > 2.0, "ENG 3 {key} defect {d} must reach its alert: {}", out[&name]);
                for (oj, other) in BEARING_KEYS.iter().enumerate().filter(|(oj, _)| *oj != bi) {
                    let other_name = format!("A32NX_ENG_3_{}_{}_DEFECT_AMPLITUDE_MM_S", BEARING_VAR_SPOOL[oj], other.to_uppercase());
                    assert_eq!(out.get(&other_name), Some(&0.0), "{other} should be silent");
                }
            }
        }
    }

    #[test]
    fn a_sustained_bearing_spall_sheds_metal_until_the_chip_detector_trips() {
        let reg = registry();
        let ids = RotorIds::resolve(&reg, 0);
        let (mut truth, commands) = running();
        truth.dt_s = 60.0;
        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        let faults = Faults::from_pairs([(ids.bearing[0][1], 1.0)]);

        let early = run(&mut live, &truth, &faults, 120.0);
        assert_eq!(early.get("A32NX_ENG_1_FAN_FRONT_CHIP_DETECTED"), Some(&0.0), "two minutes is not enough to bridge the gap");
        assert!(early["A32NX_ENG_1_FAN_FRONT_DEBRIS_G"] > 0.0, "but it is already shedding metal");

        let late = run(&mut live, &truth, &faults, 3600.0);
        assert_eq!(late.get("A32NX_ENG_1_FAN_FRONT_CHIP_DETECTED"), Some(&1.0), "ENG 1 FAN FRONT BRG CHIP DET must be reachable");
        assert_eq!(late.get("A32NX_ENG_1_IP_FRONT_CHIP_DETECTED"), Some(&0.0), "only the failed bearing's detector");
        assert!(late["A32NX_ENG_1_FAN_FRONT_DEBRIS_G"] > early["A32NX_ENG_1_FAN_FRONT_DEBRIS_G"]);
    }

    fn icing_truth() -> Truth {
        let (mut truth, _) = running();
        truth.dt_s = 0.1;
        truth.environment.sat_c = -8.0;
        truth.environment.ambient_pressure_pa = 64_400.0;
        truth.environment.tas_ms = 170.0;
        truth.controls.nacelle_anti_ice_selected = [true; 4];
        truth
    }

    #[test]
    fn a_selected_anti_ice_valve_opens_warms_the_cowl_lip_and_agrees_with_its_command() {
        let truth = icing_truth();
        let mut live = EngineAccessoriesLive::new();
        live.commands = EngineAccessoryCommands { fuel_inlet_k: Some(280.0), ..Default::default() };
        let out = run(&mut live, &truth, &Faults::default(), 60.0);
        for n in 1..=4 {
            assert!(out[&format!("A32NX_ENG_{n}_ANTI_ICE_POSITION")] > 0.95);
            assert_eq!(out.get(&format!("A32NX_ENG_{n}_ANTI_ICE_DISAGREE")), Some(&0.0));
            assert!(out[&format!("A32NX_ENG_{n}_ANTI_ICE_BLEED_KG_S")] > 0.0);
            assert!(out[&format!("A32NX_ENG_{n}_ANTI_ICE_LIP_TEMP_K")] > 273.15, "the lip must be held above freezing: {}", out[&format!("A32NX_ENG_{n}_ANTI_ICE_LIP_TEMP_K")]);
        }
    }

    #[test]
    fn a_stuck_anti_ice_valve_leaves_the_cowl_lip_cold_and_disagrees() {
        let reg = registry();
        let ids = NacelleIds::resolve(&reg, 1);
        let truth = icing_truth();
        let mut live = EngineAccessoriesLive::new();
        live.commands = EngineAccessoryCommands { fuel_inlet_k: Some(280.0), ..Default::default() };
        let out = run(&mut live, &truth, &Faults::from_pairs([(ids.anti_ice_stuck, 1.0)]), 60.0);
        assert_eq!(out.get("A32NX_ENG_2_ANTI_ICE_POSITION"), Some(&0.0));
        assert_eq!(out.get("A32NX_ENG_2_ANTI_ICE_DISAGREE"), Some(&1.0), "ENG 2 ANTI ICE FAULT must be reachable");
        assert!(out["A32NX_ENG_2_ANTI_ICE_LIP_TEMP_K"] < 273.15, "no bleed, no ice protection");
        assert!(out["A32NX_ENG_1_ANTI_ICE_LIP_TEMP_K"] > 273.15, "the other engines are protected");
    }

    #[test]
    fn a_blocked_nacelle_eductor_on_the_ground_reintroduces_the_vapour_risk() {
        let reg = registry();
        let ids = NacelleIds::resolve(&reg, 0);
        let (mut truth, commands) = running();
        truth.on_ground = true;
        truth.environment.tas_ms = 0.0;
        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        let healthy = run(&mut live, &truth, &Faults::default(), 1.0);
        assert_eq!(healthy.get("A32NX_ENG_1_NACELLE_VAPOUR_RISK"), Some(&0.0), "the eductor alone clears the minimum");

        let mut live = EngineAccessoriesLive::new();
        let blocked = run(&mut live, &truth, &Faults::from_pairs([(ids.eductor, 1.0)]), 1.0);
        assert_eq!(blocked.get("A32NX_ENG_1_NACELLE_VAPOUR_RISK"), Some(&1.0), "ENG 1 NACELLE VENT LO must be reachable");
        assert_eq!(blocked.get("A32NX_ENG_2_NACELLE_VAPOUR_RISK"), Some(&0.0));
    }

    #[test]
    fn a_cold_aircraft_on_stand_raises_no_nacelle_vent_caution() {
        let mut live = EngineAccessoriesLive::new();
        let out = run(&mut live, &Truth { dt_s: 0.5, ..Truth::default() }, &Faults::default(), 30.0);
        for n in 1..=4 {
            assert_eq!(out.get(&format!("A32NX_ENG_{n}_NACELLE_VENT_FLOW_KG_S")), Some(&0.0));
            assert_eq!(out.get(&format!("A32NX_ENG_{n}_NACELLE_VAPOUR_RISK")), Some(&0.0));
        }
    }

    #[test]
    fn a_blocked_nacelle_scoop_costs_ram_ventilation_in_flight() {
        let reg = registry();
        let ids = NacelleIds::resolve(&reg, 2);
        let truth = icing_truth();
        let mut live = EngineAccessoriesLive::new();
        let healthy = run(&mut live, &truth, &Faults::default(), 1.0);
        let mut live = EngineAccessoriesLive::new();
        let blocked = run(&mut live, &truth, &Faults::from_pairs([(ids.scoop, 1.0)]), 1.0);
        assert!(healthy["A32NX_ENG_3_NACELLE_VENT_FLOW_KG_S"] > blocked["A32NX_ENG_3_NACELLE_VENT_FLOW_KG_S"], "a blocked scoop must cost ram flow");
        assert!(blocked["A32NX_ENG_3_NACELLE_VENT_FLOW_KG_S"] > 0.0, "the eductor still runs");
    }

    #[test]
    fn a_single_false_tripping_fire_loop_disagrees_without_declaring_a_fire() {
        let reg = registry();
        let ids = NacelleIds::resolve(&reg, 3);
        for (z, zone) in FIRE_ZONE_KEYS.iter().enumerate() {
            for l in [1usize, 3] {
                let out = settled(&Faults::from_pairs([(ids.fire[z][l], 1.0)]));
                assert_eq!(out.get(&format!("A32NX_ENG_4_{zone}_FIRE_LOOP_DISAGREE")), Some(&1.0), "ENG 4 {zone} FIRE DET FAULT must be reachable");
                assert_eq!(out.get(&format!("A32NX_ENG_4_{zone}_FIRE_CONFIRMED")), Some(&0.0), "one loop cannot declare a fire");
                assert_eq!(out.get(&format!("A32NX_ENG_3_{zone}_FIRE_LOOP_DISAGREE")), Some(&0.0));
            }
        }
    }

    #[test]
    fn a_hot_nacelle_zone_confirms_a_fire_and_a_broken_loop_stops_it_confirming() {
        let reg = registry();
        let ids = NacelleIds::resolve(&reg, 0);
        let (mut truth, commands) = running();
        let hot_c = fire_detection::TRIP_K - 273.15 + 50.0;
        truth.published = (1..=4).map(|n| (format!("THERMAL_ZONE_NACELLECOWL{n}_TEMPERATURE_C"), hot_c)).collect::<PublishedFrame>();

        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        let out = run(&mut live, &truth, &Faults::default(), 1.0);
        assert_eq!(out.get("A32NX_ENG_1_CORE_FIRE_CONFIRMED"), Some(&1.0), "both healthy loops above the trip temperature confirm");
        assert_eq!(out.get("A32NX_ENG_1_CORE_FIRE_LOOP_DISAGREE"), Some(&0.0));

        let mut live = EngineAccessoriesLive::new();
        let broken = run(&mut live, &truth, &Faults::from_pairs([(ids.fire[0][0], 1.0)]), 1.0);
        assert_eq!(broken.get("A32NX_ENG_1_CORE_FIRE_CONFIRMED"), Some(&0.0), "a broken loop A leaves the fire unconfirmed");
        assert_eq!(broken.get("A32NX_ENG_1_CORE_FIRE_LOOP_DISAGREE"), Some(&1.0), "and the disagreement is what the crew is told");
    }

    #[test]
    fn only_the_inboard_engines_have_a_reverser_at_all() {
        let out = settled(&Faults::default());
        for n in [2, 3] {
            assert!(out.contains_key(&format!("A32NX_ENG_{n}_REV_POSITION")), "engine {n} carries a reverser");
        }
        for n in [1, 4] {
            assert!(!out.contains_key(&format!("A32NX_ENG_{n}_REV_POSITION")), "the A380 has no reverser on engine {n}");
        }
    }

    #[test]
    fn every_lock_failing_to_hold_at_once_deploys_the_sleeve_uncommanded() {
        let reg = registry();
        let ids = ReverserIds::resolve(&reg, 2);
        let (truth, commands) = running();

        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        let one = run(&mut live, &truth, &Faults::from_pairs([(ids.lock_hold[0], 1.0)]), 60.0);
        assert_eq!(one.get("A32NX_ENG_2_REV_UNCOMMANDED"), Some(&0.0), "one failed lock: the other two still hold");
        assert_eq!(one.get("A32NX_ENG_2_REV_POSITION"), Some(&0.0));

        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        let all = run(&mut live, &truth, &Faults::from_pairs([(ids.lock_hold[0], 1.0), (ids.lock_hold[1], 1.0), (ids.lock_hold[2], 1.0)]), 60.0);
        assert_eq!(all.get("A32NX_ENG_2_REV_UNCOMMANDED"), Some(&1.0), "ENG 2 REVERSER UNLOCKED must be reachable");
        assert_eq!(all.get("A32NX_ENG_2_REV_POSITION_DISAGREE"), Some(&1.0));
        assert_eq!(all.get("A32NX_ENG_3_REV_UNCOMMANDED"), Some(&0.0));
    }

    #[test]
    fn a_jammed_lock_blocks_a_commanded_deployment_and_the_position_disagrees() {
        let reg = registry();
        let ids = ReverserIds::resolve(&reg, 3);
        let (truth, commands) = running();
        let deploying_truth = Truth { controls: Controls { reverser_deploy_commanded: [true, true], ..truth.controls }, ..truth };

        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        let healthy = run(&mut live, &deploying_truth, &Faults::default(), 10.0);
        assert!(healthy["A32NX_ENG_3_REV_POSITION"] > 0.95, "a healthy reverser deploys when commanded");
        assert_eq!(healthy.get("A32NX_ENG_3_REV_POSITION_DISAGREE"), Some(&0.0));

        for lock in 0..N_REV_LOCKS {
            let mut live = EngineAccessoriesLive::new();
            live.commands = commands;
            let jammed = run(&mut live, &deploying_truth, &Faults::from_pairs([(ids.lock_jam[lock], 1.0)]), 10.0);
            assert_eq!(jammed.get("A32NX_ENG_3_REV_POSITION"), Some(&0.0), "lock {lock} jammed must block deployment");
            assert_eq!(jammed.get("A32NX_ENG_3_REV_POSITION_DISAGREE"), Some(&1.0), "ENG 3 REVERSER FAULT must be reachable");
        }

        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        let seized = run(&mut live, &deploying_truth, &Faults::from_pairs([(ids.actuator_jam, 1.0)]), 10.0);
        assert_eq!(seized.get("A32NX_ENG_3_REV_POSITION"), Some(&0.0));
        assert_eq!(seized.get("A32NX_ENG_3_REV_POSITION_DISAGREE"), Some(&1.0));
    }

    #[test]
    fn the_reverser_deploy_command_comes_from_truth_controls_not_self_commands() {
        let (truth, commands) = running();
        let not_deploying_commands = EngineAccessoryCommands { reverser_deploy_commanded: [false, false], ..commands };
        let deploying_truth = Truth { controls: Controls { reverser_deploy_commanded: [true, true], ..truth.controls }, ..truth };

        let mut live = EngineAccessoriesLive::new();
        live.commands = not_deploying_commands;
        let out = run(&mut live, &deploying_truth, &Faults::default(), 10.0);
        assert!(out["A32NX_ENG_2_REV_POSITION"] > 0.95, "Truth::controls.reverser_deploy_commanded must drive deployment even when self.commands disagrees");
        assert!(out["A32NX_ENG_3_REV_POSITION"] > 0.95);
    }

    #[test]
    fn nothing_divides_by_zero_on_four_cold_engines_at_zero_dt() {
        let mut live = EngineAccessoriesLive::new();
        live.tick(&Truth { dt_s: 0.0, ..Truth::default() }, &Faults::default());
        for (name, value) in published(&live) {
            assert!(value.is_finite(), "{name} is not finite");
        }
    }

    #[test]
    fn every_registered_failure_in_this_area_is_consumed_by_the_live_system() {
        let reg = registry();
        let live = EngineAccessoriesLive::new();

        let mut consumed: Vec<u64> = live.engines.iter().flat_map(|c| c.consumed_ids()).collect();
        consumed.sort_unstable();
        let before = consumed.len();
        consumed.dedup();
        assert_eq!(before, consumed.len(), "the live system resolved the same failure id twice");

        let registered: std::collections::BTreeSet<u64> = reg.failures.iter().map(|f| f.id).collect();
        for id in &consumed {
            assert!(registered.contains(id), "the live system reads {id}, which this area does not register");
        }
        for f in &reg.failures {
            assert!(consumed.contains(&f.id), "{} ({}, ata {}, {}) is registered but never read by the live system", f.id, f.name, f.ata, f.model_field);
        }
        assert_eq!(consumed.len(), registered.len());

        let mut ata: Vec<u16> = reg.failures.iter().map(|f| f.ata).collect();
        ata.sort_unstable();
        ata.dedup();
        assert_eq!(
            ata,
            [26, 30, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80],
            "this area's chapters -- 76 (Engine Controls) added for the thrust-lever channel B position transducer, E-ENG-DESIGN.md Pattern 31; 79 (Oil) added by register_oil_system"
        );
        assert!(UNCONSUMED_ATA.is_empty(), "every chapter is stepped now");
    }

    #[test]
    fn resolving_the_ids_twice_gives_the_same_answer() {
        let reg = registry();
        let reg2 = registry();
        for eng in 0..N_ENGINES {
            assert_eq!(FuelIds::resolve(&reg, eng).filter_clog, FuelIds::resolve(&reg2, eng).filter_clog);
            assert_eq!(RotorIds::resolve(&reg, eng).bearing[2][1], RotorIds::resolve(&reg2, eng).bearing[2][1]);
        }
    }

    #[test]
    fn four_engines_of_accessories_fit_comfortably_inside_a_frame() {
        let (truth, commands) = running();
        let mut live = EngineAccessoriesLive::new();
        live.commands = commands;
        let faults = Faults::default();

        for _ in 0..100 {
            live.tick(&truth, &faults);
        }

        let frames = 2_000;
        let t0 = std::time::Instant::now();
        let mut sink = 0.0;
        for _ in 0..frames {
            live.tick(&truth, &faults);
            live.publish(&mut |_, v| sink += v);
        }
        let per_frame_us = t0.elapsed().as_secs_f64() * 1e6 / frames as f64;
        println!("engine_accessories: {per_frame_us:.1} us per frame (tick + publish, 4 engines), sink {sink}");
        assert!(per_frame_us < 1_000.0, "{per_frame_us:.1} us per frame is more than 6% of a 60 Hz frame");
    }
}
