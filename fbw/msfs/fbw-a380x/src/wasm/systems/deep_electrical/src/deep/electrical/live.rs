use std::cell::RefCell;
use std::collections::HashMap;
use std::sync::OnceLock;

use crate::deep::api::Registry;
use crate::deep::live::{Area, DerivedFailure, Faults, Truth};

use super::loads::{self, Catalog};
use super::network::{BusId, Contactor, ContactorKind, FeedSource, Network, NetworkReport, ALL_BUS_IDS};
use super::shedding::{power_budget, SheddingRelays, ShedInputs};
use super::sources::{
    ApuGeneratorFaults, BatteryFaults, GroundPowerFaults, RatFaults, StaticInverterFaults, TruFaults, VfgFaults, Wiring, WiringInputs,
};

const GEN_RATED_TRUE_POWER_W: f64 = 150_000.0;
const APU_GEN_RATED_TRUE_POWER_W: f64 = 120_000.0;
const GPU_RATED_TRUE_POWER_W: f64 = 90_000.0 * 0.8;
const GPU_RECEPTACLES: usize = 4;

const AC_UNDERVOLTAGE_TRIP_V: f64 = 108.0;
const AC_OVERVOLTAGE_TRIP_V: f64 = 118.0;

fn dc_undervoltage_trip_v() -> f64 {
    BusId::Dc1.nominal_voltage() * 0.85
}

const OHM_PER_M_AWG_4_0: f64 = 0.049_01 / 304.8;
const OHM_PER_M_AWG_4: f64 = 0.248_5 / 304.8;

const AC_TIE_OHM: f64 = OHM_PER_M_AWG_4_0 * 15.0;
const ESS_FEEDER_OHM: f64 = OHM_PER_M_AWG_4 * 20.0;
const DC_FEEDER_OHM: f64 = OHM_PER_M_AWG_4 * 15.0;
const GND_SVC_FEEDER_OHM: f64 = OHM_PER_M_AWG_4 * 25.0;

fn source_rating_a(bus: BusId) -> f64 {
    match bus {
        BusId::Ac1 | BusId::Ac2 | BusId::Ac3 | BusId::Ac4 => 150_000.0 / 0.8 / 115.0,
        BusId::Dc1 | BusId::Dc2 | BusId::DcEss | BusId::DcApu => 200.0,
        BusId::DcBat | BusId::DcHot1 => 23.0 * 4.0,
        BusId::AcEmer => 70_000.0 / 115.0,
        BusId::AcGndFltSvc => 90_000.0 / 115.0,
        _ => 0.0,
    }
}

const TRANSIT_ONLY_ACTUATORS: [&str; 6] =
    ["gear-actuator-nose", "gear-actuator-left", "gear-actuator-right", "gear-door-actuator-nose", "gear-door-actuator-left", "gear-door-actuator-right"];

const TRANSIT_ONLY_NO_TRUTH_INPUT: [&str; 17] = [
    "ignition-1a",
    "ignition-1b",
    "ignition-2a",
    "ignition-2b",
    "ignition-3a",
    "ignition-3b",
    "ignition-4a",
    "ignition-4b",
    "eng-fire-bottle-1-squib-1",
    "eng-fire-bottle-1-squib-2",
    "eng-fire-bottle-2-squib-1",
    "eng-fire-bottle-2-squib-2",
    "apu-fire-bottle-squib-1",
    "apu-fire-bottle-squib-2",
    "apu-start-contactor",
    "cargo-door-fwd-actuator-ctl",
    "cargo-door-aft-actuator-ctl",
];

const EQUIPMENT_BAY_FALLBACK_C: f64 = 20.0;

const CARGO_DOOR_TRAVEL_MARGIN_PERCENT: f64 = 2.0;

const GEAR_TRANSIT_MARGIN_FRACTION: f64 = 0.02;

const SHED_RELEASE_MARGIN_FRACTION: f64 = 0.10;

pub mod board {
    use super::*;

    #[derive(Default, Clone, Debug)]
    pub struct Board {
        pub breaker_current_a: Vec<f64>,
        pub breaker_open_cmd: Vec<f64>,
        pub bus_voltage: [f64; 17],
        pub load_short: Vec<f64>,
        pub load_open: Vec<f64>,
        pub load_high_resistance: Vec<f64>,
    }

    thread_local! {
        static BOARD: RefCell<Board> = RefCell::new(Board::default());
    }

    pub fn with_board<R>(f: impl FnOnce(&Board) -> R) -> R {
        BOARD.with(|b| f(&b.borrow()))
    }

    pub fn with_board_mut<R>(f: impl FnOnce(&mut Board) -> R) -> R {
        BOARD.with(|b| f(&mut b.borrow_mut()))
    }

    pub fn clear() {
        with_board_mut(|b| *b = Board::default());
    }

    pub struct Topology {
        pub breaker_index: HashMap<&'static str, usize>,
        pub load_index: HashMap<&'static str, usize>,
        pub breaker_count: usize,
        pub load_count: usize,
    }

    static TOPOLOGY: OnceLock<Topology> = OnceLock::new();

    pub fn topology() -> &'static Topology {
        TOPOLOGY.get_or_init(|| {
            let mut net = Network::new();
            loads::build(&mut net);
            Wiring::build(&mut net, 15.0);
            Topology {
                breaker_index: net.breakers.iter().enumerate().map(|(i, b)| (b.id, i)).collect(),
                load_index: net.loads.iter().enumerate().map(|(i, l)| (l.spec.id, i)).collect(),
                breaker_count: net.breakers.len(),
                load_count: net.loads.len(),
            }
        })
    }
}

use board::topology;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LoadField {
    OpenCircuit,
    ShortToGround,
    HighResistance,
    Intermittent,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Target {
    Load(usize, LoadField),
    BreakerFailsToTrip(usize),
    BreakerNuisance(usize),
    ContactorFailsToClose(usize),
    ContactorWelded(usize),
    Diode(usize),
    Bus(usize),
    VfgWinding(usize),
    VfgRegulator(usize),
    ApuGenWinding(usize),
    ApuGenRegulator(usize),
    Tru(usize),
    BatteryCapacity(usize),
    BatteryResistance(usize),
    StaticInverter,
    Rat,
    Gpu,
    Misc(usize),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(dead_code)]
pub(super) enum MiscFault {
    Enmu1 = 0,
    Enmu2 = 1,
    Elmu = 2,
    TrMonitoring1 = 3,
    TrMonitoring2 = 4,
    TrMonitoringEss = 5,
    FctlActuatorPwr = 6,
    Psc1 = 7,
    Psc2 = 8,
    ApuBat = 9,
    DriveDiscFault1 = 10,
    DriveDiscFault2 = 11,
    DriveDiscFault3 = 12,
    DriveDiscFault4 = 13,
    DriveOilLeak1 = 14,
    DriveOilLeak2 = 15,
    DriveOilLeak3 = 16,
    DriveOilLeak4 = 17,
    BusTieOff = 18,
    CabinOvhtL = 19,
    CabinOvhtR = 20,
    CabinOvhtDetL = 21,
    CabinOvhtDetR = 22,
    Ssc1CommDegraded = 23,
    Ssc2CommDegraded = 24,
    Ssc1SupplyFault = 25,
    Ssc2SupplyFault = 26,
    Ssc1RedundLost = 27,
    Ssc2RedundLost = 28,
    ExtPwr1Fault = 29,
    ExtPwr2Fault = 30,
    ExtPwr3Fault = 31,
    ExtPwr4Fault = 32,
}

pub(super) const MISC_FAULT_COUNT: usize = 33;

pub(super) const MISC_FAULTS: [(&str, &str, &str); MISC_FAULT_COUNT] = [
    ("enmu-1", "ENMU 1", "Electrical Network Management Unit 1 (owns the GEN 1+2 tie/shed decision): the computer's own health, independent of the buses/contactors it drives (240800052)"),
    ("enmu-2", "ENMU 2", "Electrical Network Management Unit 2 (GEN 3+4): the computer's own health (240800053)"),
    ("elmu", "ELMU", "Electrical Load Management Unit (owns the GALLEY/COMMERCIAL shed decision this area's shedding.rs executes): the computer's own health (240800069)"),
    ("tr-mon-1", "TR 1 MONITORING", "TR 1's own voltage/current monitoring circuit, independent of TR 1's own electrical verdict (240800084)"),
    ("tr-mon-2", "TR 2 MONITORING", "TR 2's own monitoring circuit (240800085)"),
    ("tr-mon-ess", "TR ESS MONITORING", "TR ESS's own monitoring circuit (240800086)"),
    ("fctl-actuator-pwr", "F/CTL ACTUATOR PWR SUPPLY", "the power-conditioning unit feeding the flight-control EHA/EBHA backup actuators, independent of the AC/DC bus it draws from (240800060)"),
    ("psc-1", "PRIMARY SUPPLY CENTER 1", "PSC1's own health as a distribution-centre unit, independent of the individual buses/contactors it groups (240800070)"),
    ("psc-2", "PRIMARY SUPPLY CENTER 2", "PSC2's own health (240800071)"),
    ("apu-bat", "APU BAT", "the APU's own dedicated start/standby battery, a third instance of this area's Battery-class hardware (240800013)"),
    ("drive-disc-1", "DRIVE 1 DISC FAULT", "generator 1's mechanical drive-disconnect coupling's own fault-detection circuit, independent of the disconnect actually happening (240800032)"),
    ("drive-disc-2", "DRIVE 2 DISC FAULT", "generator 2's drive-disconnect detection circuit (240800033)"),
    ("drive-disc-3", "DRIVE 3 DISC FAULT", "generator 3's drive-disconnect detection circuit (240800034)"),
    ("drive-disc-4", "DRIVE 4 DISC FAULT", "generator 4's drive-disconnect detection circuit (240800035)"),
    ("drive-oil-1", "DRIVE 1 OIL LEAK", "generator 1's drive lubrication reservoir leak, 0..1 (feeds ELEC_DRIVE_1_OIL_LEVEL_FRAC/_OIL_TEMP_C/_OIL_LOW_PRESSURE_TRIPPED, 240800040/044/048)"),
    ("drive-oil-2", "DRIVE 2 OIL LEAK", "generator 2's drive lubrication reservoir leak (240800041/045/049)"),
    ("drive-oil-3", "DRIVE 3 OIL LEAK", "generator 3's drive lubrication reservoir leak (240800042/046/050)"),
    ("drive-oil-4", "DRIVE 4 OIL LEAK", "generator 4's drive lubrication reservoir leak (240800043/047/051)"),
    ("bus-tie-off", "BUS TIE OFF", "the BUS TIE pb-sw is abnormally set to OFF, FCOM p.4879 (240800019)"),
    ("cabin-ovht-l", "CABIN L SUPPLY CENTER OVHT", "the overheat detector has detected an overheat in the left cabin supply center, FCOM p.4882 (240800022)"),
    ("cabin-ovht-r", "CABIN R SUPPLY CENTER OVHT", "the overheat detector has detected an overheat in the right cabin supply center, FCOM p.4882 (240800023)"),
    ("cabin-ovht-det-l", "CABIN L SUPPLY CENTER OVHT DET", "the left cabin supply center's own overheat detector health, independent of whether it has tripped (240800024)"),
    ("cabin-ovht-det-r", "CABIN R SUPPLY CENTER OVHT DET", "the right cabin supply center's own overheat detector health (240800025)"),
    ("ssc-1-comm-degraded", "SECONDARY SUPPLY CENTER 1 COMM DEGRADED", "communication is degraded between SSC1 and CPIOM E / other aircraft systems, FCOM p.4955 (240800074)"),
    ("ssc-2-comm-degraded", "SECONDARY SUPPLY CENTER 2 COMM DEGRADED", "communication is degraded between SSC2 and CPIOM E / other aircraft systems, FCOM p.4955 (240800075)"),
    ("ssc-1-supply-fault", "SECONDARY SUPPLY CENTER 1 SUPPLY FAULT", "some systems are no longer supplied by SSC1, FCOM p.4956 (240800076)"),
    ("ssc-2-supply-fault", "SECONDARY SUPPLY CENTER 2 SUPPLY FAULT", "some systems are no longer supplied by SSC2, FCOM p.4956 (240800077)"),
    ("ssc-1-redund-lost", "SECONDARY SUPPLY CENTER 1 REDUND LOST", "the redundancy of some system electrical supply from SSC1 is lost, no operational impact, FCOM p.4958 (240800078)"),
    ("ssc-2-redund-lost", "SECONDARY SUPPLY CENTER 2 REDUND LOST", "the redundancy of some system electrical supply from SSC2 is lost (240800079)"),
    ("ext-pwr-1-fault", "EXT PWR 1 FAULT", "the external power unit 1, or its associated GGPCU, is failed, FCOM p.4943 (240800056)"),
    ("ext-pwr-2-fault", "EXT PWR 2 FAULT", "the external power unit 2, or its associated GGPCU, is failed (240800057)"),
    ("ext-pwr-3-fault", "EXT PWR 3 FAULT", "the external power unit 3, or its associated GGPCU, is failed (240800058)"),
    ("ext-pwr-4-fault", "EXT PWR 4 FAULT", "the external power unit 4, or its associated GGPCU, is failed (240800059)"),
];

fn route_failures(net: &Network) -> (Vec<(u64, Target)>, Vec<String>) {
    let mut reg = Registry::default();
    super::registry::register(&mut reg);
    let mut out = Vec::with_capacity(reg.failures.len());
    let mut unresolved = Vec::new();

    let source_index = |prefix: &str, comp: &str| -> Option<usize> {
        let rest = comp.strip_prefix(prefix)?;
        rest.parse::<usize>().ok().map(|n| n - 1)
    };

    for f in &reg.failures {
        let Some((type_path, field)) = f.model_field.split_once(".faults.") else {
            unresolved.push(f.model_field.clone());
            continue;
        };
        let comp = f.component.as_str();
        let target = match (type_path, field) {
            ("deep::electrical::network::Load", field) => {
                let id = comp.split_once('.').map(|(_, rest)| rest).unwrap_or(comp);
                let Some(&idx) = topology().load_index.get(id) else {
                    unresolved.push(f.component.clone());
                    continue;
                };
                let lf = match field {
                    "open_circuit" => LoadField::OpenCircuit,
                    "short_to_ground" => LoadField::ShortToGround,
                    "high_resistance" => LoadField::HighResistance,
                    "intermittent" => LoadField::Intermittent,
                    _ => {
                        unresolved.push(f.model_field.clone());
                        continue;
                    }
                };
                Target::Load(idx, lf)
            }
            ("deep::electrical::network::Breaker", field) => {
                let id = comp.strip_prefix("24_elec.bkr.").unwrap_or(comp);
                let Some(&idx) = topology().breaker_index.get(id) else {
                    unresolved.push(f.component.clone());
                    continue;
                };
                match field {
                    "fails_to_trip" => Target::BreakerFailsToTrip(idx),
                    "nuisance_trip" => Target::BreakerNuisance(idx),
                    _ => {
                        unresolved.push(f.model_field.clone());
                        continue;
                    }
                }
            }
            ("deep::electrical::network::Contactor", field) => {
                let id = comp.strip_prefix("24_elec.contactor.").unwrap_or(comp);
                let Some(idx) = net.contactor_index(id) else {
                    unresolved.push(f.component.clone());
                    continue;
                };
                match field {
                    "fails_to_close" => Target::ContactorFailsToClose(idx),
                    "welded_closed" => Target::ContactorWelded(idx),
                    _ => {
                        unresolved.push(f.model_field.clone());
                        continue;
                    }
                }
            }
            ("deep::electrical::network::Diode", _) => {
                let id = comp.strip_prefix("24_elec.diode.").unwrap_or(comp);
                let Some(idx) = net.diode_index(id) else {
                    unresolved.push(f.component.clone());
                    continue;
                };
                Target::Diode(idx)
            }
            ("deep::electrical::network::Bus", _) => {
                let label = comp.strip_prefix("24_elec.bus.").unwrap_or(comp);
                let Some(bus) = ALL_BUS_IDS.iter().find(|b| b.label() == label) else {
                    unresolved.push(f.component.clone());
                    continue;
                };
                Target::Bus(bus.index())
            }
            ("deep::electrical::sources::Vfg", field) => {
                let Some(n) = source_index("24_elec.vfg-", comp) else {
                    unresolved.push(f.component.clone());
                    continue;
                };
                if field == "winding_degradation" {
                    Target::VfgWinding(n)
                } else {
                    Target::VfgRegulator(n)
                }
            }
            ("deep::electrical::sources::ApuGenerator", field) => {
                let Some(n) = source_index("24_elec.apu-gen-", comp) else {
                    unresolved.push(f.component.clone());
                    continue;
                };
                if field == "winding_degradation" {
                    Target::ApuGenWinding(n)
                } else {
                    Target::ApuGenRegulator(n)
                }
            }
            ("deep::electrical::sources::Tru", _) => {
                let idx = match comp {
                    "24_elec.tr-1" => 0,
                    "24_elec.tr-2" => 1,
                    "24_elec.tr-ess" => 2,
                    "24_elec.tr-apu" => 3,
                    _ => {
                        unresolved.push(f.component.clone());
                        continue;
                    }
                };
                Target::Tru(idx)
            }
            ("deep::electrical::sources::Battery", field) => {
                let Some(n) = source_index("24_elec.bat-", comp) else {
                    unresolved.push(f.component.clone());
                    continue;
                };
                if field == "capacity_fade" {
                    Target::BatteryCapacity(n)
                } else {
                    Target::BatteryResistance(n)
                }
            }
            ("deep::electrical::sources::StaticInverter", _) => Target::StaticInverter,
            ("deep::electrical::sources::Rat", _) => Target::Rat,
            ("deep::electrical::sources::GroundPower", _) => Target::Gpu,
            ("deep::electrical::misc::Health", "fault") => {
                let suffix = comp.strip_prefix("24_elec.misc.").unwrap_or(comp);
                let Some(idx) = MISC_FAULTS.iter().position(|&(s, _, _)| s == suffix) else {
                    unresolved.push(f.component.clone());
                    continue;
                };
                Target::Misc(idx)
            }
            _ => {
                unresolved.push(f.model_field.clone());
                continue;
            }
        };
        out.push((f.id, target));
    }
    (out, unresolved)
}

fn welded_channel_pairs() -> Vec<(usize, u64)> {
    let mut reg = Registry::default();
    crate::deep::breakers::registry::register(&mut reg);
    let mut out = Vec::new();
    for f in &reg.failures {
        if !f.model_field.contains("contact_resistance") {
            continue;
        }
        let Some(id) = f.component.strip_prefix("17_breakers.") else { continue };
        if let Some(&idx) = topology().breaker_index.get(id) {
            out.push((idx, f.id));
        }
    }
    out
}

fn bus_tag(bus: BusId) -> &'static str {
    match bus {
        BusId::Ac1 => "AC_1",
        BusId::Ac2 => "AC_2",
        BusId::Ac3 => "AC_3",
        BusId::Ac4 => "AC_4",
        BusId::AcEss => "AC_ESS",
        BusId::AcEssShed => "AC_ESS_SHED",
        BusId::AcEmer => "AC_EMER",
        BusId::AcGndFltSvc => "AC_GND_FLT_SVC",
        BusId::Dc1 => "DC_1",
        BusId::Dc2 => "DC_2",
        BusId::DcEss => "DC_ESS",
        BusId::DcEssShed => "DC_ESS_SHED",
        BusId::DcBat => "DC_BAT",
        BusId::DcHot1 => "DC_HOT_1",
        BusId::DcHot2 => "DC_HOT_2",
        BusId::DcApu => "DC_APU",
        BusId::DcGndFltSvc => "DC_GND_FLT_SVC",
    }
}

const FBW_GENERATOR: [u64; 4] = [24_020, 24_021, 24_022, 24_023];
const FBW_APU_GENERATOR: [u64; 2] = [24_030, 24_031];
const FBW_TR: [u64; 4] = [24_000, 24_001, 24_002, 24_003];
const FBW_STATIC_INVERTER: u64 = 24_004;

const VFG_COMPONENT: [&str; 4] = ["24_elec.vfg-1", "24_elec.vfg-2", "24_elec.vfg-3", "24_elec.vfg-4"];
const APU_GEN_COMPONENT: [&str; 2] = ["24_elec.apu-gen-1", "24_elec.apu-gen-2"];
const TR_COMPONENT: [&str; 4] = ["24_elec.tr-1", "24_elec.tr-2", "24_elec.tr-ess", "24_elec.tr-apu"];
const STATIC_INVERTER_COMPONENT: &str = "24_elec.static-inv";
const BUS_COMPONENT: [&str; 17] = [
    "24_elec.bus.AC1",
    "24_elec.bus.AC2",
    "24_elec.bus.AC3",
    "24_elec.bus.AC4",
    "24_elec.bus.AC_ESS",
    "24_elec.bus.AC_ESS_SHED",
    "24_elec.bus.AC_EMER",
    "24_elec.bus.AC_GND_FLT_SVC",
    "24_elec.bus.DC1",
    "24_elec.bus.DC2",
    "24_elec.bus.DC_ESS",
    "24_elec.bus.DC_ESS_SHED",
    "24_elec.bus.DC_BAT",
    "24_elec.bus.DC_HOT1",
    "24_elec.bus.DC_HOT2",
    "24_elec.bus.DC_APU",
    "24_elec.bus.DC_GND_FLT_SVC",
];

fn fbw_bus_failure(bus: BusId) -> Option<u64> {
    Some(match bus {
        BusId::Ac1 => 24_100,
        BusId::Ac2 => 24_101,
        BusId::Ac3 => 24_102,
        BusId::Ac4 => 24_103,
        BusId::AcEss => 24_105,
        BusId::AcEmer => 24_104,
        BusId::AcGndFltSvc => 24_107,
        BusId::Dc1 => 24_108,
        BusId::Dc2 => 24_109,
        BusId::DcEss => 24_110,
        BusId::DcHot1 => 24_113,
        BusId::DcHot2 => 24_114,
        BusId::DcApu => 24_112,
        BusId::DcGndFltSvc => 24_117,
        BusId::AcEssShed | BusId::DcEssShed | BusId::DcBat => return None,
    })
}

const DEGRADED_BEYOND_HALF: f64 = 0.5;

const REASON_OVERLOAD_TRIPPED: &str = "its own I^2t overload element ran to the trip and took it off line";
const REASON_REGULATOR_OUT_OF_BAND: &str = "driven and excited, but regulating outside MIL-STD-704F's 108-118 V band";
const REASON_WINDING_DEGRADED: &str = "stator/winding degraded past half its range: it can no longer hold rated voltage under load";
const REASON_TRU_DEGRADED: &str = "rectifier winding degraded past half its range toward its own degraded internal resistance";
const REASON_INVERTER_DEGRADED: &str = "conversion efficiency degraded past half its range toward its own floor";
const REASON_FEEDER_OPEN: &str = "the feeder breaker protecting this bus has tripped open";
const REASON_HEALTHY: &str = "healthy";

fn coupling_table() -> Vec<(u64, &'static str)> {
    let mut v: Vec<(u64, &'static str)> = Vec::new();
    for i in 0..4 {
        v.push((FBW_GENERATOR[i], VFG_COMPONENT[i]));
    }
    for i in 0..2 {
        v.push((FBW_APU_GENERATOR[i], APU_GEN_COMPONENT[i]));
    }
    for i in 0..4 {
        v.push((FBW_TR[i], TR_COMPONENT[i]));
    }
    v.push((FBW_STATIC_INVERTER, STATIC_INVERTER_COMPONENT));
    for (bi, &bus) in ALL_BUS_IDS.iter().enumerate() {
        if let Some(id) = fbw_bus_failure(bus) {
            v.push((id, BUS_COMPONENT[bi]));
        }
    }
    v
}

struct Names {
    bus_potential: Vec<String>,
    bus_powered: Vec<String>,
    bus_frequency: Vec<String>,
    gen_fault: Vec<String>,
    apu_gen_fault: Vec<String>,
    gen_load_w: Vec<String>,
    apu_gen_load_w: Vec<String>,
    tr_fault: Vec<String>,
    bat_fault: Vec<String>,
    bat_charge: Vec<String>,
    bcl_fault: Vec<String>,
    breaker_current: Vec<String>,
    breaker_closed: Vec<String>,
    load_powered: Vec<String>,
    fbw_raw_ac_potential: Vec<String>,
    fbw_raw_dc_potential: Vec<String>,
    fbw_raw_ac_powered: Vec<String>,
    fbw_raw_dc_powered: Vec<String>,
    derived: Vec<String>,
}

pub struct ElectricalLive {
    net: Network,
    catalog: Catalog,
    wiring: Wiring,
    shedding: SheddingRelays,

    routed: Vec<(u64, Target)>,
    welded_pairs: Vec<(usize, u64)>,
    faults_were_armed: bool,

    names: Names,

    contactor: ContactorIndices,
    src_gen: [usize; 4],
    src_apu_gen: [usize; 2],
    src_tr: [usize; 4],
    src_bat: [usize; 2],
    src_gpu: usize,
    src_rat: usize,
    bcl_load: [usize; 2],

    ignition_load: [[usize; 2]; 4],
    eng_fire_squib: [[usize; 2]; 2],
    apu_fire_squib: [usize; 2],
    apu_start_contactor: usize,
    cargo_door_ctl: [usize; 2],
    gear_actuator: [usize; 3],
    gear_door_actuator: [usize; 3],

    emergency_shed_load: Vec<usize>,

    externally_opened: Vec<bool>,

    feeder_breaker: [usize; 17],

    report: NetworkReport,
    gen_fault: [bool; 4],
    apu_gen_fault: [bool; 2],
    tr_fault: [bool; 4],
    gen_verdict: [Option<&'static str>; 4],
    apu_gen_verdict: [Option<&'static str>; 2],
    tr_verdict: [Option<&'static str>; 4],
    static_inv_verdict: Option<&'static str>,
    bat_fault: [bool; 2],
    bat_charge: [f64; 2],
    bcl_fault: [bool; 2],
    galley_shed: bool,
    commercial_shed: bool,
    galley_shed_commanded: bool,
    emergency_config: bool,
    total_demand_w: f64,
    capacity_w: f64,
    rat_deployed: bool,
    rat_solenoid_on: bool,

    measured_gen_load_w: [f64; 4],
    measured_apu_gen_load_w: [f64; 2],
    measured_tr_load_w: [f64; 4],
    measured_battery_current_a: [f64; 2],
    measured_rat_load_w: f64,

    fbw_raw_ac_potential: [f64; 4],
    fbw_raw_dc_potential: [f64; 2],
    fbw_raw_ac_powered: [bool; 4],
    fbw_raw_dc_powered: [bool; 2],

    misc_fault: [f64; MISC_FAULT_COUNT],
    drive_oil: [crate::deep::apu::oil::OilSystem; 4],
    drive_oil_state: [crate::deep::apu::oil::OilState; 4],
    rat_fault: bool,
    gen_pb_on: [bool; 4],
}

struct ContactorIndices {
    gen_line: [usize; 4],
    apu_gen_line: [usize; 2],
    tr_line: [usize; 4],
    static_inv_line: usize,
    bat_direct: [usize; 2],
    gpu_line: [usize; 4],
    rat_line: usize,
    ac_tie: [usize; 3],
    ac_ess_feed_1: usize,
    ac_ess_feed_4: usize,
    ac_ess_shed: usize,
    ac_emer_to_ess: usize,
    dc_tie_1_2: usize,
    dc_ess_feed_1: usize,
    dc_ess_shed: usize,
    dc_bat_tie: usize,
    ac_gnd_svc_feed: usize,
    dc_gnd_svc_feed: usize,
}

pub fn live_system() -> Box<dyn Area> {
    Box::new(ElectricalLive::new())
}

impl Default for ElectricalLive {
    fn default() -> Self {
        Self::new()
    }
}

impl ElectricalLive {
    pub fn new() -> Self {
        board::clear();

        let mut net = Network::new();
        let catalog = loads::build(&mut net);
        let wiring = Wiring::build(&mut net, 15.0);

        let mut tie = |id: &'static str, kind: ContactorKind, from: BusId, to: BusId, ohm: f64| -> usize {
            net.add_contactor(Contactor::new(id, kind, FeedSource::Bus(from), to, ohm))
        };
        let ac_tie = [
            tie("ac-tie-1-2", ContactorKind::BusTie, BusId::Ac1, BusId::Ac2, AC_TIE_OHM),
            tie("ac-tie-2-3", ContactorKind::BusTie, BusId::Ac2, BusId::Ac3, AC_TIE_OHM),
            tie("ac-tie-3-4", ContactorKind::BusTie, BusId::Ac3, BusId::Ac4, AC_TIE_OHM),
        ];
        let ac_ess_feed_1 = tie("ac-ess-feed-1", ContactorKind::Feeder, BusId::Ac1, BusId::AcEss, ESS_FEEDER_OHM);
        let ac_ess_feed_4 = tie("ac-ess-feed-4", ContactorKind::Feeder, BusId::Ac4, BusId::AcEss, ESS_FEEDER_OHM);
        let ac_ess_shed = tie("ac-ess-shed", ContactorKind::Feeder, BusId::AcEss, BusId::AcEssShed, ESS_FEEDER_OHM);
        let ac_emer_to_ess = tie("ac-emer-to-ess", ContactorKind::Feeder, BusId::AcEmer, BusId::AcEss, ESS_FEEDER_OHM);
        let dc_tie_1_2 = tie("dc-tie-1-2", ContactorKind::BusTie, BusId::Dc1, BusId::Dc2, DC_FEEDER_OHM);
        let dc_ess_feed_1 = tie("dc-ess-feed-1", ContactorKind::Feeder, BusId::Dc1, BusId::DcEss, DC_FEEDER_OHM);
        let dc_ess_shed = tie("dc-ess-shed", ContactorKind::Feeder, BusId::DcEss, BusId::DcEssShed, DC_FEEDER_OHM);
        let dc_bat_tie = tie("dc-bat-tie", ContactorKind::Feeder, BusId::DcEss, BusId::DcBat, DC_FEEDER_OHM);
        let ac_gnd_svc_feed = tie("ac-gnd-svc-feed", ContactorKind::Feeder, BusId::Ac1, BusId::AcGndFltSvc, GND_SVC_FEEDER_OHM);
        let dc_gnd_svc_feed = tie("dc-gnd-svc-feed", ContactorKind::Feeder, BusId::Dc2, BusId::DcGndFltSvc, GND_SVC_FEEDER_OHM);

        for id in TRANSIT_ONLY_ACTUATORS.iter().copied().chain(TRANSIT_ONLY_NO_TRUTH_INPUT.iter().copied()) {
            if let Some(i) = net.load_index(id) {
                net.loads[i].commanded_on = false;
            }
        }

        let mut feeder_breaker = [0usize; 17];
        for &bus in ALL_BUS_IDS.iter() {
            let connected_a: f64 = net
                .loads
                .iter()
                .filter(|l| l.spec.bus == bus)
                .map(|l| l.spec.rated_power_w / (bus.nominal_voltage() * l.spec.power_factor.max(0.1)))
                .sum();
            let rated = (connected_a * 1.25).max(source_rating_a(bus));
            let id: &'static str = Box::leak(format!("feeder-{}", bus.label()).into_boxed_str());
            feeder_breaker[bus.index()] = net.add_feeder_breaker(super::network::Breaker::new(id, rated.max(5.0), bus), bus);
        }

        let find_c = |net: &Network, id: &str| net.contactor_index(id).unwrap_or_else(|| panic!("sources::Wiring::build no longer builds contactor {id}"));
        let find_s = |net: &Network, id: &str| net.sources.iter().position(|s| s.id == id).unwrap_or_else(|| panic!("sources::Wiring::build no longer builds source {id}"));

        let contactor = ContactorIndices {
            gen_line: [find_c(&net, "gen-1-line"), find_c(&net, "gen-2-line"), find_c(&net, "gen-3-line"), find_c(&net, "gen-4-line")],
            apu_gen_line: [find_c(&net, "apu-gen-1-line"), find_c(&net, "apu-gen-2-line")],
            tr_line: [find_c(&net, "tr-1-line"), find_c(&net, "tr-2-line"), find_c(&net, "tr-ess-line"), find_c(&net, "tr-apu-line")],
            static_inv_line: find_c(&net, "static-inv-line"),
            bat_direct: [find_c(&net, "bat-1-direct"), find_c(&net, "bat-2-direct")],
            gpu_line: [1, 2, 3, 4].map(|n| find_c(&net, &format!("gpu-{n}-line"))),
            rat_line: find_c(&net, "rat-line"),
            ac_tie,
            ac_ess_feed_1,
            ac_ess_feed_4,
            ac_ess_shed,
            ac_emer_to_ess,
            dc_tie_1_2,
            dc_ess_feed_1,
            dc_ess_shed,
            dc_bat_tie,
            ac_gnd_svc_feed,
            dc_gnd_svc_feed,
        };
        let src_gen = [find_s(&net, "gen-1"), find_s(&net, "gen-2"), find_s(&net, "gen-3"), find_s(&net, "gen-4")];
        let src_apu_gen = [find_s(&net, "apu-gen-1"), find_s(&net, "apu-gen-2")];
        let src_tr = [find_s(&net, "tr-1"), find_s(&net, "tr-2"), find_s(&net, "tr-ess"), find_s(&net, "tr-apu")];
        let src_bat = [find_s(&net, "bat-1"), find_s(&net, "bat-2")];
        let src_gpu = find_s(&net, "gpu");
        let src_rat = find_s(&net, "rat");

        let find_l = |net: &Network, id: &str| net.load_index(id).unwrap_or_else(|| panic!("loads::build no longer builds load {id}"));
        let bcl_load = [find_l(&net, "bat-charge-limiter-1"), find_l(&net, "bat-charge-limiter-2")];
        let ignition_load: [[usize; 2]; 4] = std::array::from_fn(|e| {
            let n = e + 1;
            [find_l(&net, &format!("ignition-{n}a")), find_l(&net, &format!("ignition-{n}b"))]
        });
        let eng_fire_squib: [[usize; 2]; 2] = std::array::from_fn(|bottle| {
            let b = bottle + 1;
            [find_l(&net, &format!("eng-fire-bottle-{b}-squib-1")), find_l(&net, &format!("eng-fire-bottle-{b}-squib-2"))]
        });
        let apu_fire_squib = [find_l(&net, "apu-fire-bottle-squib-1"), find_l(&net, "apu-fire-bottle-squib-2")];
        let apu_start_contactor = find_l(&net, "apu-start-contactor");
        let cargo_door_ctl = [find_l(&net, "cargo-door-fwd-actuator-ctl"), find_l(&net, "cargo-door-aft-actuator-ctl")];
        let gear_actuator = [find_l(&net, "gear-actuator-nose"), find_l(&net, "gear-actuator-left"), find_l(&net, "gear-actuator-right")];
        let gear_door_actuator = [find_l(&net, "gear-door-actuator-nose"), find_l(&net, "gear-door-actuator-left"), find_l(&net, "gear-door-actuator-right")];

        let emergency_shed_load: Vec<usize> = ["hyd-epump-ga", "hyd-epump-gb", "hyd-epump-ya", "hyd-epump-yb"]
            .iter()
            .map(|id| find_l(&net, id))
            .chain((0..25).map(|i| find_l(&net, &format!("fuel-pump-{i}"))))
            .collect();

        let (routed, unresolved) = route_failures(&net);
        debug_assert!(unresolved.is_empty(), "unrouted registered failures: {unresolved:?}");

        let names = Names {
            bus_potential: ALL_BUS_IDS.iter().map(|&b| format!("ELEC_{}_BUS_POTENTIAL", bus_tag(b))).collect(),
            bus_powered: ALL_BUS_IDS.iter().map(|&b| format!("ELEC_{}_BUS_IS_POWERED", bus_tag(b))).collect(),
            bus_frequency: ALL_BUS_IDS.iter().map(|&b| format!("ELEC_{}_BUS_FREQUENCY", bus_tag(b))).collect(),
            fbw_raw_ac_potential: (1..=4).map(|n| format!("DEEP_ELEC_AC_{n}_FBW_RAW_POTENTIAL")).collect(),
            fbw_raw_dc_potential: (1..=2).map(|n| format!("DEEP_ELEC_DC_{n}_FBW_RAW_POTENTIAL")).collect(),
            fbw_raw_ac_powered: (1..=4).map(|n| format!("DEEP_ELEC_AC_{n}_FBW_RAW_IS_POWERED")).collect(),
            fbw_raw_dc_powered: (1..=2).map(|n| format!("DEEP_ELEC_DC_{n}_FBW_RAW_IS_POWERED")).collect(),
            gen_fault: (1..=4).map(|n| format!("ELEC_GEN_{n}_FAULT")).collect(),
            apu_gen_fault: (1..=2).map(|n| format!("ELEC_APU_GEN_{n}_FAULT")).collect(),
            gen_load_w: (1..=4).map(|n| format!("ELEC_ENG_GEN_{n}_LOAD_W")).collect(),
            apu_gen_load_w: (1..=2).map(|n| format!("ELEC_APU_GEN_{n}_LOAD_W")).collect(),
            tr_fault: ["1", "2", "ESS", "APU"].iter().map(|s| format!("ELEC_TR_{s}_FAULT")).collect(),
            bat_fault: (1..=2).map(|n| format!("ELEC_BAT_{n}_FAULT")).collect(),
            bat_charge: (1..=2).map(|n| format!("ELEC_BAT_{n}_CHARGE_FRACTION")).collect(),
            bcl_fault: (1..=2).map(|n| format!("ELEC_BCL_{n}_FAULT")).collect(),
            breaker_current: net.breakers.iter().map(|b| format!("ELEC_BKR_{}_CURRENT_A", b.id)).collect(),
            breaker_closed: net.breakers.iter().map(|b| format!("ELEC_BKR_{}_CLOSED", b.id)).collect(),
            load_powered: net.loads.iter().map(|l| format!("ELEC_LOAD_{}_POWERED", l.spec.id)).collect(),
            derived: coupling_table().into_iter().map(|(id, _)| format!("DEEP_DERIVED_FBW_FAILURE_{id}")).collect(),
        };

        let n_breakers = net.breakers.len();
        Self {
            net,
            catalog,
            wiring,
            shedding: SheddingRelays::new(),
            routed,
            welded_pairs: welded_channel_pairs(),
            faults_were_armed: false,
            names,
            contactor,
            src_gen,
            src_apu_gen,
            src_tr,
            src_bat,
            src_gpu,
            src_rat,
            bcl_load,
            ignition_load,
            eng_fire_squib,
            apu_fire_squib,
            apu_start_contactor,
            cargo_door_ctl,
            gear_actuator,
            gear_door_actuator,
            emergency_shed_load,
            externally_opened: vec![false; n_breakers],
            feeder_breaker,
            report: NetworkReport::default(),
            gen_fault: [false; 4],
            apu_gen_fault: [false; 2],
            tr_fault: [false; 4],
            gen_verdict: [None; 4],
            apu_gen_verdict: [None; 2],
            tr_verdict: [None; 4],
            static_inv_verdict: None,
            bat_fault: [false; 2],
            bat_charge: [1.0; 2],
            bcl_fault: [false; 2],
            galley_shed: false,
            commercial_shed: false,
            galley_shed_commanded: false,
            emergency_config: false,
            total_demand_w: 0.0,
            capacity_w: 0.0,
            rat_deployed: false,
            rat_solenoid_on: false,
            measured_gen_load_w: [0.0; 4],
            measured_apu_gen_load_w: [0.0; 2],
            measured_tr_load_w: [0.0; 4],
            measured_battery_current_a: [0.0; 2],
            measured_rat_load_w: 0.0,
            fbw_raw_ac_potential: [0.0; 4],
            fbw_raw_dc_potential: [0.0; 2],
            misc_fault: [0.0; MISC_FAULT_COUNT],
            drive_oil: std::array::from_fn(|_| crate::deep::apu::oil::OilSystem::new(288.15)),
            drive_oil_state: [crate::deep::apu::oil::OilState::default(); 4],
            rat_fault: false,
            gen_pb_on: [false; 4],
            fbw_raw_ac_powered: [false; 4],
            fbw_raw_dc_powered: [false; 2],
        }
    }

    pub fn network(&self) -> &Network {
        &self.net
    }

    fn clear_model_faults(&mut self) {
        for l in &mut self.net.loads {
            l.faults = Default::default();
        }
        for b in &mut self.net.breakers {
            b.faults = Default::default();
        }
        for c in &mut self.net.contactors {
            c.faults = Default::default();
        }
        for d in &mut self.net.diodes {
            d.faults = Default::default();
        }
        for b in &mut self.net.buses {
            b.faults = Default::default();
        }
        self.misc_fault = [0.0; MISC_FAULT_COUNT];
    }

    fn apply_faults(&mut self, faults: &Faults) -> SourceFaults {
        let mut sf = SourceFaults::default();
        for i in 0..self.routed.len() {
            let (id, target) = self.routed[i];
            let m = faults.get(id);
            if m <= 0.0 {
                continue;
            }
            match target {
                Target::Load(idx, field) => {
                    let f = &mut self.net.loads[idx].faults;
                    let slot = match field {
                        LoadField::OpenCircuit => &mut f.open_circuit,
                        LoadField::ShortToGround => &mut f.short_to_ground,
                        LoadField::HighResistance => &mut f.high_resistance,
                        LoadField::Intermittent => &mut f.intermittent,
                    };
                    *slot = slot.max(m);
                }
                Target::BreakerFailsToTrip(idx) => {
                    let s = &mut self.net.breakers[idx].faults.fails_to_trip;
                    *s = s.max(m);
                }
                Target::BreakerNuisance(idx) => {
                    let s = &mut self.net.breakers[idx].faults.nuisance_trip;
                    *s = s.max(m);
                }
                Target::ContactorFailsToClose(idx) => {
                    let s = &mut self.net.contactors[idx].faults.fails_to_close;
                    *s = s.max(m);
                }
                Target::ContactorWelded(idx) => {
                    let s = &mut self.net.contactors[idx].faults.welded_closed;
                    *s = s.max(m);
                }
                Target::Diode(idx) => {
                    let s = &mut self.net.diodes[idx].faults.open_circuit;
                    *s = s.max(m);
                }
                Target::Bus(idx) => {
                    let s = &mut self.net.buses[idx].faults.short_to_ground;
                    *s = s.max(m);
                }
                Target::VfgWinding(n) => sf.vfg[n].winding_degradation = sf.vfg[n].winding_degradation.max(m),
                Target::VfgRegulator(n) => sf.vfg[n].regulator_drift = sf.vfg[n].regulator_drift.max(m),
                Target::ApuGenWinding(n) => sf.apu_gen[n].winding_degradation = sf.apu_gen[n].winding_degradation.max(m),
                Target::ApuGenRegulator(n) => sf.apu_gen[n].regulator_drift = sf.apu_gen[n].regulator_drift.max(m),
                Target::Tru(n) => sf.tru[n].winding_degradation = sf.tru[n].winding_degradation.max(m),
                Target::BatteryCapacity(n) => sf.battery[n].capacity_fade = sf.battery[n].capacity_fade.max(m),
                Target::BatteryResistance(n) => sf.battery[n].resistance_growth = sf.battery[n].resistance_growth.max(m),
                Target::StaticInverter => sf.static_inverter.efficiency_loss = sf.static_inverter.efficiency_loss.max(m),
                Target::Rat => sf.rat.jammed = sf.rat.jammed.max(m),
                Target::Gpu => sf.ground_power.weak_cart = sf.ground_power.weak_cart.max(m),
                Target::Misc(idx) => self.misc_fault[idx] = self.misc_fault[idx].max(m),
            }
        }
        for i in 0..self.welded_pairs.len() {
            let (idx, id) = self.welded_pairs[i];
            let m = faults.get(id);
            if m > 0.0 {
                let s = &mut self.net.breakers[idx].faults.fails_to_trip;
                *s = s.max(m);
            }
        }
        sf
    }

    fn apply_wiring_damage(&mut self) {
        board::with_board(|b| {
            let n = self.net.loads.len();
            for i in 0..n.min(b.load_short.len()) {
                let f = &mut self.net.loads[i].faults;
                f.short_to_ground = f.short_to_ground.max(b.load_short[i]);
            }
            for i in 0..n.min(b.load_open.len()) {
                let f = &mut self.net.loads[i].faults;
                f.open_circuit = f.open_circuit.max(b.load_open[i]);
            }
            for i in 0..n.min(b.load_high_resistance.len()) {
                let f = &mut self.net.loads[i].faults;
                f.high_resistance = f.high_resistance.max(b.load_high_resistance[i]);
            }
        });
    }

    fn apply_breaker_commands(&mut self) {
        board::with_board(|b| {
            let n = self.net.breakers.len().min(b.breaker_open_cmd.len());
            for i in 0..n {
                let commanded_open = b.breaker_open_cmd[i] > 0.5;
                if commanded_open {
                    if !self.externally_opened[i] {
                        self.net.breakers[i].pull();
                        self.externally_opened[i] = true;
                    }
                } else if self.externally_opened[i] {
                    self.net.breakers[i].reset();
                    self.externally_opened[i] = false;
                }
            }
        });
    }

    fn command_contactors(&mut self, truth: &Truth, gpu_plugged_in: bool) {
        let producing = |net: &Network, src: usize| net.sources[src].open_circuit_v > 0.0;

        let mut gen_on_line = [false; 4];
        for i in 0..4 {
            let tripped = self.wiring.vfg[i].overload_heat() >= 1.0;
            gen_on_line[i] = truth.controls.eng_gen_pb_on[i] && truth.engine_running[i] && producing(&self.net, self.src_gen[i]) && !tripped;
            self.gen_pb_on[i] = truth.controls.eng_gen_pb_on[i];
            let idx = self.contactor.gen_line[i];
            self.net.contactors[idx].commanded_closed = gen_on_line[i];
        }

        let mut apu_on_line = false;
        for i in 0..2 {
            let on = truth.controls.apu_gen_pb_on[i] && truth.apu_running && producing(&self.net, self.src_apu_gen[i]) && self.wiring.apu_gen[i].overload_heat() < 1.0;
            apu_on_line |= on;
            self.net.contactors[self.contactor.apu_gen_line[i]].commanded_closed = on;
        }

        let anchored = gen_on_line.iter().any(|&g| g) || apu_on_line || gpu_plugged_in;
        let need_tie = anchored && (gen_on_line.iter().any(|&g| !g) || apu_on_line || gpu_plugged_in);
        for &idx in &self.contactor.ac_tie {
            self.net.contactors[idx].commanded_closed = need_tie;
        }

        let main_ac_anchored = anchored;
        let main_ac_at_voltage = [BusId::Ac1, BusId::Ac2, BusId::Ac3, BusId::Ac4].map(|b| self.net.bus(b).voltage >= AC_UNDERVOLTAGE_TRIP_V);
        let ac1_live = main_ac_anchored && main_ac_at_voltage[0];
        let ac4_live = main_ac_anchored && main_ac_at_voltage[3];
        self.net.contactors[self.contactor.ac_ess_feed_1].commanded_closed = ac1_live;
        self.net.contactors[self.contactor.ac_ess_feed_4].commanded_closed = !ac1_live && ac4_live;

        let emergency = !main_ac_anchored || !main_ac_at_voltage.iter().any(|&live| live);
        self.emergency_config = emergency;

        let was_rat_deployed = self.wiring.rat.deployed();
        if emergency && !truth.on_ground {
            self.wiring.rat.deploy();
        }
        self.rat_deployed = self.wiring.rat.deployed();
        self.rat_solenoid_on = emergency && !was_rat_deployed;
        if let Some(i) = self.net.load_index("rat-deploy-solenoid") {
            self.net.loads[i].commanded_on = self.rat_solenoid_on;
        }

        let inv_live = producing(&self.net, self.contactor_source(self.contactor.static_inv_line));
        self.net.contactors[self.contactor.static_inv_line].commanded_closed = emergency && inv_live;
        let rat_live = self.rat_deployed && producing(&self.net, self.src_rat);
        self.net.contactors[self.contactor.rat_line].commanded_closed = rat_live;

        self.net.contactors[self.contactor.ac_emer_to_ess].commanded_closed = emergency && rat_live;

        self.net.contactors[self.contactor.ac_ess_shed].commanded_closed = !emergency;
        self.net.contactors[self.contactor.dc_ess_shed].commanded_closed = !emergency;

        let mut tr_live = [false; 4];
        for i in 0..4 {
            tr_live[i] = producing(&self.net, self.src_tr[i]);
            self.net.contactors[self.contactor.tr_line[i]].commanded_closed = tr_live[i];
        }
        self.net.contactors[self.contactor.dc_ess_feed_1].commanded_closed = tr_live[0];
        self.net.contactors[self.contactor.dc_tie_1_2].commanded_closed = tr_live[0] != tr_live[1];
        self.net.contactors[self.contactor.dc_bat_tie].commanded_closed = tr_live[0] || tr_live[2] || (emergency && !truth.on_ground);
        for i in 0..2 {
            self.net.contactors[self.contactor.bat_direct[i]].commanded_closed = truth.controls.bat_pb_auto[i];
        }

        let gpu_live = gpu_plugged_in && producing(&self.net, self.src_gpu);
        for &idx in &self.contactor.gpu_line {
            self.net.contactors[idx].commanded_closed = gpu_live;
        }

        self.net.contactors[self.contactor.ac_gnd_svc_feed].commanded_closed = truth.on_ground && ac1_live;
        self.net.contactors[self.contactor.dc_gnd_svc_feed].commanded_closed = truth.on_ground && tr_live[1];

        for bi in 0..17 {
            if !self.net.breakers[self.feeder_breaker[bi]].closed {
                let bus = ALL_BUS_IDS[bi];
                for c in &mut self.net.contactors {
                    if c.to == bus || c.from == FeedSource::Bus(bus) {
                        c.commanded_closed = false;
                    }
                }
            }
        }
    }

    fn command_transit_loads(&mut self, truth: &Truth) {
        let c = &truth.controls;

        for eng in 0..4 {
            let on = c.starter_engaged[eng];
            self.net.loads[self.ignition_load[eng][0]].commanded_on = on;
            self.net.loads[self.ignition_load[eng][1]].commanded_on = on;
        }

        for bottle in 0..2 {
            let fired = (0..4).any(|eng| c.fire_pb_released[eng] && c.fire_agent_pb_pressed[eng][bottle]);
            self.net.loads[self.eng_fire_squib[bottle][0]].commanded_on = fired;
            self.net.loads[self.eng_fire_squib[bottle][1]].commanded_on = fired;
        }

        let apu_fired = c.fire_pb_apu_released && c.fire_agent_pb_apu_pressed;
        self.net.loads[self.apu_fire_squib[0]].commanded_on = apu_fired;
        self.net.loads[self.apu_fire_squib[1]].commanded_on = apu_fired;

        self.net.loads[self.apu_start_contactor].commanded_on = c.apu_start_pb_on;

        let cargo_pos = truth.published.get_or("CABIN_CARGO_DOOR_PERCENT:1", 0.0);
        let cargo_cmd = truth.published.get_or("CABIN_CARGO_DOOR_CMD:1", 0.0);
        let cargo_moving = (cargo_cmd - cargo_pos).abs() > CARGO_DOOR_TRAVEL_MARGIN_PERCENT;
        self.net.loads[self.cargo_door_ctl[0]].commanded_on = cargo_moving;
        self.net.loads[self.cargo_door_ctl[1]].commanded_on = cargo_moving;

        for i in 0..3 {
            let pos = c.gear_door_commanded_open[i];
            let in_transit = pos > GEAR_TRANSIT_MARGIN_FRACTION && pos < 1.0 - GEAR_TRANSIT_MARGIN_FRACTION;
            self.net.loads[self.gear_door_actuator[i]].commanded_on = in_transit;
            self.net.loads[self.gear_actuator[i]].commanded_on = in_transit;
        }
    }

    fn command_emergency_shed(&mut self, generation_on_line_w: f64) {
        let shed = generation_on_line_w <= 0.0;
        for &i in &self.emergency_shed_load {
            self.net.loads[i].commanded_on = !shed;
        }
    }

    fn contactor_source(&self, contactor: usize) -> usize {
        match self.net.contactors[contactor].from {
            FeedSource::Source(i) => i,
            FeedSource::Bus(_) => unreachable!("contactor {contactor} is bus-fed, not source-fed"),
        }
    }

    fn capacity_w(&self, truth: &Truth, gpu_plugged_in: bool) -> f64 {
        let mut cap = 0.0;
        for i in 0..4 {
            if self.net.contactors[self.contactor.gen_line[i]].commanded_closed && truth.engine_running[i] {
                cap += GEN_RATED_TRUE_POWER_W;
            }
        }
        for i in 0..2 {
            if self.net.contactors[self.contactor.apu_gen_line[i]].commanded_closed && truth.apu_running {
                cap += APU_GEN_RATED_TRUE_POWER_W;
            }
        }
        if gpu_plugged_in {
            cap += GPU_RATED_TRUE_POWER_W * GPU_RECEPTACLES as f64;
        }
        cap
    }

    fn source_branch_current_a(&self, src: usize, contactor: usize) -> f64 {
        let c = &self.net.contactors[contactor];
        if !c.closed {
            return 0.0;
        }
        let s = &self.net.sources[src];
        let r = (c.resistance_ohm + s.resistance_ohm).max(1.0e-6);
        (s.open_circuit_v - self.net.bus(c.to).voltage) / r
    }

    fn source_delivered_w(&self, src: usize, contactor: usize) -> f64 {
        let i = self.source_branch_current_a(src, contactor);
        if i <= 0.0 {
            return 0.0;
        }
        i * self.net.bus(self.net.contactors[contactor].to).voltage
    }

    fn each_coupling(&self, out: &mut dyn FnMut(DerivedFailure)) {
        let verdict = |v: Option<&'static str>| (if v.is_some() { 1.0 } else { 0.0 }, v.unwrap_or(REASON_HEALTHY));
        for i in 0..4 {
            let (magnitude, reason) = verdict(self.gen_verdict[i]);
            out(DerivedFailure { fbw_id: FBW_GENERATOR[i], magnitude, deep_component: VFG_COMPONENT[i], reason });
        }
        for i in 0..2 {
            let (magnitude, reason) = verdict(self.apu_gen_verdict[i]);
            out(DerivedFailure { fbw_id: FBW_APU_GENERATOR[i], magnitude, deep_component: APU_GEN_COMPONENT[i], reason });
        }
        for i in 0..4 {
            let (magnitude, reason) = verdict(self.tr_verdict[i]);
            out(DerivedFailure { fbw_id: FBW_TR[i], magnitude, deep_component: TR_COMPONENT[i], reason });
        }
        let (magnitude, reason) = verdict(self.static_inv_verdict);
        out(DerivedFailure { fbw_id: FBW_STATIC_INVERTER, magnitude, deep_component: STATIC_INVERTER_COMPONENT, reason });
        for (bi, &bus) in ALL_BUS_IDS.iter().enumerate() {
            let Some(id) = fbw_bus_failure(bus) else { continue };
            let isolated = !self.net.breakers[self.feeder_breaker[bi]].closed;
            out(DerivedFailure {
                fbw_id: id,
                magnitude: if isolated { 1.0 } else { 0.0 },
                deep_component: BUS_COMPONENT[bi],
                reason: if isolated { REASON_FEEDER_OPEN } else { REASON_HEALTHY },
            });
        }
    }
}

#[derive(Default)]
struct SourceFaults {
    vfg: [VfgFaults; 4],
    apu_gen: [ApuGeneratorFaults; 2],
    tru: [TruFaults; 4],
    battery: [BatteryFaults; 2],
    static_inverter: StaticInverterFaults,
    rat: RatFaults,
    ground_power: GroundPowerFaults,
}

impl Area for ElectricalLive {
    fn name(&self) -> &'static str {
        "electrical"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        self.fbw_raw_ac_potential = truth.ac_bus_volts;
        self.fbw_raw_dc_potential = truth.dc_bus_volts;
        self.fbw_raw_ac_powered = truth.ac_bus_powered;
        self.fbw_raw_dc_powered = truth.dc_bus_powered;

        let dt = truth.dt_s.max(0.0);

        let armed = faults.any();
        let sf = if armed || self.faults_were_armed {
            self.clear_model_faults();
            self.apply_faults(faults)
        } else {
            SourceFaults::default()
        };
        self.faults_were_armed = armed;

        self.apply_wiring_damage();
        self.apply_breaker_commands();

        let gpu_plugged_in = truth.gpu_plugged_in;
        let inputs = WiringInputs {
            engine_speed_fraction: truth.engine_n2_frac,
            measured_gen_load_w: self.measured_gen_load_w,
            vfg_faults: sf.vfg,
            apu_speed_fraction: if truth.apu_running { 1.0 } else { 0.0 },
            measured_apu_gen_load_w: self.measured_apu_gen_load_w,
            apu_gen_faults: sf.apu_gen,
            tru_faults: sf.tru,
            measured_tr_load_w: self.measured_tr_load_w,
            battery_faults: sf.battery,
            measured_battery_current_a: self.measured_battery_current_a,
            static_inverter_faults: sf.static_inverter,
            gpu_plugged_in,
            ground_power_faults: sf.ground_power,
            airspeed_kt: truth.environment.tas_ms / 0.514_444,
            rat_faults: sf.rat,
            ambient_c: truth.published.get_or("THERMAL_ZONE_MAINAVIONICS_TEMPERATURE_C", EQUIPMENT_BAY_FALLBACK_C),
        };
        self.wiring.pre_step(&mut self.net, &inputs, dt);

        self.command_contactors(truth, gpu_plugged_in);
        self.command_transit_loads(truth);

        let capacity_w = self.capacity_w(truth, gpu_plugged_in);
        self.command_emergency_shed(capacity_w);
        let budget = power_budget(&self.net, capacity_w);
        self.galley_shed_commanded = if self.galley_shed_commanded {
            budget.margin_w < capacity_w * SHED_RELEASE_MARGIN_FRACTION
        } else {
            budget.overloaded
        };
        let shed_inputs = ShedInputs { galley_shed_commanded: self.galley_shed_commanded, emergency_config_commanded: self.emergency_config };
        let shed = self.shedding.step(&mut self.net, &self.catalog, &shed_inputs);
        self.galley_shed = shed.galley_shed;
        self.commercial_shed = shed.commercial_shed;
        self.capacity_w = capacity_w;
        self.total_demand_w = budget.total_demand_w;

        self.report = self.net.step(dt);
        for i in 0..4 {
            self.measured_gen_load_w[i] = self.source_delivered_w(self.src_gen[i], self.contactor.gen_line[i]);
        }
        for i in 0..2 {
            self.measured_apu_gen_load_w[i] = self.source_delivered_w(self.src_apu_gen[i], self.contactor.apu_gen_line[i]);
            self.measured_battery_current_a[i] = self.source_branch_current_a(self.src_bat[i], self.contactor.bat_direct[i]);
        }
        self.measured_rat_load_w = self.source_delivered_w(self.src_rat, self.contactor.rat_line);
        for i in 0..4 {
            self.measured_tr_load_w[i] = self.source_delivered_w(self.src_tr[i], self.contactor.tr_line[i]);
        }
        self.wiring.post_step(&self.net, &inputs, dt);

        let dc_trip = dc_undervoltage_trip_v();
        for i in 0..4 {
            let on_line = self.net.contactors[self.contactor.gen_line[i]].closed;
            let bus_v = self.net.bus(self.net.contactors[self.contactor.gen_line[i]].to).voltage;
            let position_disagree = on_line != self.net.contactors[self.contactor.gen_line[i]].commanded_closed;
            self.gen_fault[i] = self.wiring.vfg[i].overload_heat() >= 1.0 || (on_line && (bus_v < AC_UNDERVOLTAGE_TRIP_V || bus_v > AC_OVERVOLTAGE_TRIP_V)) || position_disagree;
        }
        for i in 0..2 {
            let on_line = self.net.contactors[self.contactor.apu_gen_line[i]].closed;
            let bus_v = self.net.bus(self.net.contactors[self.contactor.apu_gen_line[i]].to).voltage;
            self.apu_gen_fault[i] = self.wiring.apu_gen[i].overload_heat() >= 1.0 || (on_line && (bus_v < AC_UNDERVOLTAGE_TRIP_V || bus_v > AC_OVERVOLTAGE_TRIP_V));
        }
        let tr_ac_in = [BusId::Ac1, BusId::Ac2, BusId::AcEss, BusId::AcEss];
        let tr_dc_out = [BusId::Dc1, BusId::Dc2, BusId::DcEss, BusId::DcApu];
        for i in 0..4 {
            let ac_powered = self.net.bus(tr_ac_in[i]).voltage > 90.0;
            self.tr_fault[i] = ac_powered && self.net.bus(tr_dc_out[i]).voltage < dc_trip;
        }
        for i in 0..2 {
            let (v, _r) = self.wiring.battery[i].terminal(sf.battery[i]);
            let direct = &self.net.contactors[self.contactor.bat_direct[i]];
            let position_disagree = direct.closed != direct.commanded_closed;
            self.bat_fault[i] = (direct.closed && v < dc_trip) || position_disagree;
            self.bat_charge[i] = self.wiring.battery[i].charge_fraction(sf.battery[i]);
            let bcl = &self.net.loads[self.bcl_load[i]].faults;
            self.bcl_fault[i] = bcl.open_circuit > 0.0 || bcl.high_resistance > 0.0 || bcl.intermittent > 0.0;
        }
        for i in 0..4 {
            let driven = truth.engine_running[i] && self.net.sources[self.src_gen[i]].open_circuit_v > 0.0;
            let terminal = self.net.sources[self.src_gen[i]].open_circuit_v;
            self.gen_verdict[i] = if self.wiring.vfg[i].overload_heat() >= 1.0 {
                Some(REASON_OVERLOAD_TRIPPED)
            } else if driven && (terminal < AC_UNDERVOLTAGE_TRIP_V || terminal > AC_OVERVOLTAGE_TRIP_V) {
                Some(REASON_REGULATOR_OUT_OF_BAND)
            } else if sf.vfg[i].winding_degradation >= DEGRADED_BEYOND_HALF {
                Some(REASON_WINDING_DEGRADED)
            } else {
                None
            };
        }
        for i in 0..2 {
            let driven = truth.apu_running && self.net.sources[self.src_apu_gen[i]].open_circuit_v > 0.0;
            let terminal = self.net.sources[self.src_apu_gen[i]].open_circuit_v;
            self.apu_gen_verdict[i] = if self.wiring.apu_gen[i].overload_heat() >= 1.0 {
                Some(REASON_OVERLOAD_TRIPPED)
            } else if driven && (terminal < AC_UNDERVOLTAGE_TRIP_V || terminal > AC_OVERVOLTAGE_TRIP_V) {
                Some(REASON_REGULATOR_OUT_OF_BAND)
            } else if sf.apu_gen[i].winding_degradation >= DEGRADED_BEYOND_HALF {
                Some(REASON_WINDING_DEGRADED)
            } else {
                None
            };
        }
        for i in 0..4 {
            self.tr_verdict[i] = (sf.tru[i].winding_degradation >= DEGRADED_BEYOND_HALF).then_some(REASON_TRU_DEGRADED);
        }
        self.static_inv_verdict = (sf.static_inverter.efficiency_loss >= DEGRADED_BEYOND_HALF).then_some(REASON_INVERTER_DEGRADED);

        self.rat_fault = sf.rat.jammed >= DEGRADED_BEYOND_HALF;

        for i in 0..4 {
            let ambient_k = (truth.engine_oil_temp_c[i] + 273.15).max(200.0);
            let n_frac = truth.engine_n2_frac[i].max(0.0);
            let starvation = 1.0 - self.drive_oil_state[i].level_frac;
            let friction_heat_w = (400.0 + starvation * 15_000.0) * n_frac;
            let n_percent = n_frac * 100.0;
            let oil_faults = crate::deep::apu::oil::OilFaults { leak: self.misc_fault[MiscFault::DriveOilLeak1 as usize + i] };
            self.drive_oil_state[i] = self.drive_oil[i].step(n_percent, truth.engine_running[i], ambient_k, friction_heat_w, &oil_faults, dt);
        }
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        for (i, bus) in self.net.buses.iter().enumerate() {
            out(&self.names.bus_potential[i], bus.voltage);
            out(&self.names.bus_powered[i], if self.report.bus_powered[i] { 1.0 } else { 0.0 });
            out(&self.names.bus_frequency[i], bus.frequency_hz);
        }
        for i in 0..4 {
            out(&self.names.fbw_raw_ac_potential[i], self.fbw_raw_ac_potential[i]);
            out(&self.names.fbw_raw_ac_powered[i], if self.fbw_raw_ac_powered[i] { 1.0 } else { 0.0 });
        }
        for i in 0..2 {
            out(&self.names.fbw_raw_dc_potential[i], self.fbw_raw_dc_potential[i]);
            out(&self.names.fbw_raw_dc_powered[i], if self.fbw_raw_dc_powered[i] { 1.0 } else { 0.0 });
        }
        for i in 0..4 {
            out(&self.names.gen_fault[i], if self.gen_fault[i] { 1.0 } else { 0.0 });
            out(&self.names.gen_load_w[i], self.measured_gen_load_w[i]);
            out(&self.names.tr_fault[i], if self.tr_fault[i] { 1.0 } else { 0.0 });
        }
        for i in 0..2 {
            out(&self.names.apu_gen_fault[i], if self.apu_gen_fault[i] { 1.0 } else { 0.0 });
            out(&self.names.apu_gen_load_w[i], self.measured_apu_gen_load_w[i]);
            out(&self.names.bat_fault[i], if self.bat_fault[i] { 1.0 } else { 0.0 });
            out(&self.names.bat_charge[i], self.bat_charge[i]);
            out(&self.names.bcl_fault[i], if self.bcl_fault[i] { 1.0 } else { 0.0 });
        }
        out("ELEC_APU_GEN_FAULT", if self.apu_gen_fault[0] || self.apu_gen_fault[1] { 1.0 } else { 0.0 });
        out("ELEC_GALLEY_SHED_ACTIVE", if self.galley_shed { 1.0 } else { 0.0 });
        out("ELEC_COMMERCIAL_SHED_ACTIVE", if self.commercial_shed { 1.0 } else { 0.0 });
        out("ELEC_RAT_DEPLOYED", if self.rat_deployed { 1.0 } else { 0.0 });
        let emer_gen_load_pct = self.measured_rat_load_w / super::sources::Rat::MAX_POWER_W * 100.0;
        out("ELEC_EMER_GEN_LOAD", emer_gen_load_pct);
        out("ELEC_EMER_GEN_LOAD_NORMAL", if emer_gen_load_pct < 108.0 { 1.0 } else { 0.0 });
        out("ELEC_EMER_CONFIG_ACTIVE", if self.emergency_config { 1.0 } else { 0.0 });
        out("ELEC_TOTAL_DEMAND_W", self.total_demand_w);
        out("ELEC_AVAILABLE_CAPACITY_W", self.capacity_w);
        out("ELEC_TOTAL_DELIVERED_W", self.report.total_power_w);

        out("ELEC_AC_ESS_FED_BY_ALTN", if self.net.contactors[self.contactor.ac_ess_feed_4].commanded_closed { 1.0 } else { 0.0 });
        out("ELEC_STATIC_INV_FAULT", if self.static_inv_verdict.is_some() { 1.0 } else { 0.0 });
        out("ELEC_RAT_FAULT", if self.rat_fault { 1.0 } else { 0.0 });
        for i in 0..4 {
            out(&format!("ELEC_EXT_PWR_{}_ON_LINE", i + 1), if self.net.contactors[self.contactor.gpu_line[i]].closed { 1.0 } else { 0.0 });
        }
        out("ELEC_ENMU_1_FAULT", if self.misc_fault[MiscFault::Enmu1 as usize] > 0.0 { 1.0 } else { 0.0 });
        out("ELEC_ENMU_2_FAULT", if self.misc_fault[MiscFault::Enmu2 as usize] > 0.0 { 1.0 } else { 0.0 });
        out("ELEC_ELMU_FAULT", if self.misc_fault[MiscFault::Elmu as usize] > 0.0 { 1.0 } else { 0.0 });
        out("ELEC_TR_1_MONITORING_FAULT", if self.misc_fault[MiscFault::TrMonitoring1 as usize] > 0.0 { 1.0 } else { 0.0 });
        out("ELEC_TR_2_MONITORING_FAULT", if self.misc_fault[MiscFault::TrMonitoring2 as usize] > 0.0 { 1.0 } else { 0.0 });
        out("ELEC_TR_ESS_MONITORING_FAULT", if self.misc_fault[MiscFault::TrMonitoringEss as usize] > 0.0 { 1.0 } else { 0.0 });
        out("ELEC_FCTL_ACTUATOR_PWR_FAULT", if self.misc_fault[MiscFault::FctlActuatorPwr as usize] > 0.0 { 1.0 } else { 0.0 });
        out("ELEC_PSC_1_FAULT", if self.misc_fault[MiscFault::Psc1 as usize] > 0.0 { 1.0 } else { 0.0 });
        out("ELEC_PSC_2_FAULT", if self.misc_fault[MiscFault::Psc2 as usize] > 0.0 { 1.0 } else { 0.0 });
        out("ELEC_APU_BAT_FAULT", if self.misc_fault[MiscFault::ApuBat as usize] > 0.0 { 1.0 } else { 0.0 });
        for i in 0..4 {
            let disc_idx = MiscFault::DriveDiscFault1 as usize + i;
            out(&format!("ELEC_DRIVE_{}_DISC_FAULT", i + 1), if self.misc_fault[disc_idx] > 0.0 { 1.0 } else { 0.0 });
            out(&format!("ELEC_DRIVE_{}_OIL_LEVEL_FRAC", i + 1), self.drive_oil_state[i].level_frac);
            out(&format!("ELEC_DRIVE_{}_OIL_TEMP_C", i + 1), self.drive_oil_state[i].temp_c);
            out(&format!("ELEC_DRIVE_{}_OIL_PRESSURE_PSI", i + 1), self.drive_oil_state[i].pressure_psi);
            out(&format!("ELEC_DRIVE_{}_OIL_LOW_PRESSURE_TRIPPED", i + 1), if self.drive_oil_state[i].low_pressure_tripped { 1.0 } else { 0.0 });
            out(&format!("ELEC_EXT_PWR_{}_FAULT", i + 1), if self.misc_fault[MiscFault::ExtPwr1Fault as usize + i] > 0.0 { 1.0 } else { 0.0 });
        }
        out("ELEC_BUS_TIE_OFF", if self.misc_fault[MiscFault::BusTieOff as usize] > 0.0 { 1.0 } else { 0.0 });
        for i in 0..4 {
            out(&format!("ELEC_GEN_{}_PB_ON", i + 1), if self.gen_pb_on[i] { 1.0 } else { 0.0 });
        }
        out("ELEC_CABIN_L_SUPPLY_CENTER_OVHT", if self.misc_fault[MiscFault::CabinOvhtL as usize] > 0.0 { 1.0 } else { 0.0 });
        out("ELEC_CABIN_R_SUPPLY_CENTER_OVHT", if self.misc_fault[MiscFault::CabinOvhtR as usize] > 0.0 { 1.0 } else { 0.0 });
        out("ELEC_CABIN_L_SUPPLY_CENTER_OVHT_DET_FAULT", if self.misc_fault[MiscFault::CabinOvhtDetL as usize] > 0.0 { 1.0 } else { 0.0 });
        out("ELEC_CABIN_R_SUPPLY_CENTER_OVHT_DET_FAULT", if self.misc_fault[MiscFault::CabinOvhtDetR as usize] > 0.0 { 1.0 } else { 0.0 });
        out("ELEC_SSC_1_DEGRADED", if self.misc_fault[MiscFault::Ssc1CommDegraded as usize] > 0.0 { 1.0 } else { 0.0 });
        out("ELEC_SSC_2_DEGRADED", if self.misc_fault[MiscFault::Ssc2CommDegraded as usize] > 0.0 { 1.0 } else { 0.0 });
        out("ELEC_SSC_1_FAULT", if self.misc_fault[MiscFault::Ssc1SupplyFault as usize] > 0.0 { 1.0 } else { 0.0 });
        out("ELEC_SSC_2_FAULT", if self.misc_fault[MiscFault::Ssc2SupplyFault as usize] > 0.0 { 1.0 } else { 0.0 });
        out("ELEC_SSC_1_REDUND_LOST", if self.misc_fault[MiscFault::Ssc1RedundLost as usize] > 0.0 { 1.0 } else { 0.0 });
        out("ELEC_SSC_2_REDUND_LOST", if self.misc_fault[MiscFault::Ssc2RedundLost as usize] > 0.0 { 1.0 } else { 0.0 });

        for (i, b) in self.net.breakers.iter().enumerate() {
            out(&self.names.breaker_current[i], b.current_a);
            out(&self.names.breaker_closed[i], if b.closed { 1.0 } else { 0.0 });
        }
        for (i, l) in self.net.loads.iter().enumerate() {
            out(&self.names.load_powered[i], if l.powered { 1.0 } else { 0.0 });
        }

        let mut k = 0usize;
        self.each_coupling(&mut |d| {
            if let Some(name) = self.names.derived.get(k) {
                out(name, d.magnitude);
            }
            k += 1;
        });

        board::with_board_mut(|board| {
            let n = self.net.breakers.len();
            if board.breaker_current_a.len() != n {
                board.breaker_current_a = vec![0.0; n];
            }
            for (i, b) in self.net.breakers.iter().enumerate() {
                board.breaker_current_a[i] = b.current_a;
            }
            for (i, bus) in self.net.buses.iter().enumerate() {
                board.bus_voltage[i] = bus.voltage;
            }
        });
    }

    fn derived_failures(&self, out: &mut dyn FnMut(DerivedFailure)) {
        self.each_coupling(out);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::live::Deep;
    use std::collections::BTreeMap;

    fn flying_truth() -> Truth {
        Truth {
            dt_s: 1.0 / 30.0,
            on_ground: false,
            altitude_ft: 35_000.0,
            engine_n1_frac: [0.9; 4],
            engine_n2_frac: [0.9; 4],
            engine_running: [true; 4],
            ..Truth::default()
        }
    }

    fn run(live: &mut ElectricalLive, truth: &Truth, faults: &Faults, frames: usize) -> BTreeMap<String, f64> {
        let mut published = BTreeMap::new();
        for _ in 0..frames {
            live.tick(truth, faults);
            published.clear();
            live.publish(&mut |n, v| {
                published.insert(n.to_string(), v);
            });
        }
        published
    }

    #[test]
    fn every_registered_failure_resolves_to_a_real_model_field() {
        let live = ElectricalLive::new();
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        let (routed, unresolved) = route_failures(&live.net);
        assert!(unresolved.is_empty(), "{} registered failures do not resolve: {:?}", unresolved.len(), &unresolved[..unresolved.len().min(10)]);
        assert_eq!(routed.len(), reg.failures.len(), "every registered failure must be routed exactly once");
        let mut ids: Vec<u64> = routed.iter().map(|(id, _)| *id).collect();
        let before = ids.len();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), before, "a failure id was routed twice");
        assert!(routed.len() > 1_000, "expected the real catalogue, got {}", routed.len());
    }

    #[test]
    fn source_protection_breakers_now_carry_their_buss_real_current() {
        board::clear();
        let mut live = ElectricalLive::new();
        let published = run(&mut live, &flying_truth(), &Faults::default(), 20);
        for id in ["gen-1-bkr", "gen-2-bkr", "tr-1-bkr", "tr-ess-bkr"] {
            let v = published[&format!("ELEC_BKR_{id}_CURRENT_A")];
            assert!(v > 0.0, "{id} should now carry its bus's real demand current, got {v} A");
        }
        board::clear();
    }

    #[test]
    fn four_running_engines_energise_every_main_ac_bus_and_the_dc_buses_behind_them() {
        board::clear();
        let mut live = ElectricalLive::new();
        let published = run(&mut live, &flying_truth(), &Faults::default(), 20);
        for tag in ["AC_1", "AC_2", "AC_3", "AC_4", "AC_ESS", "DC_1", "DC_2", "DC_ESS"] {
            let v = published[&format!("ELEC_{tag}_BUS_POTENTIAL")];
            assert!(published[&format!("ELEC_{tag}_BUS_IS_POWERED")] == 1.0, "{tag} should be powered, it sits at {v} V");
        }
        assert!(published["ELEC_AC_1_BUS_POTENTIAL"] > 108.0);
        assert!(published["ELEC_AC_1_BUS_FREQUENCY"] > 360.0, "a VFG's bus runs at its own variable frequency");
        assert_eq!(published["ELEC_GEN_1_FAULT"], 0.0);
    }

    #[test]
    fn a_cold_dark_aircraft_has_no_ac_at_all_but_its_hot_buses_stay_alive() {
        board::clear();
        let mut live = ElectricalLive::new();
        let published = run(&mut live, &Truth::default(), &Faults::default(), 10);
        assert_eq!(published["ELEC_AC_1_BUS_IS_POWERED"], 0.0);
        assert_eq!(published["ELEC_AC_2_BUS_IS_POWERED"], 0.0);
        assert!(published["ELEC_DC_HOT_1_BUS_POTENTIAL"] > 20.0, "a battery-direct hot bus is live on a cold aircraft");
    }

    #[test]
    fn fbw_raw_bus_values_are_captured_before_this_areas_own_override() {
        board::clear();
        let mut live = ElectricalLive::new();
        let truth = Truth {
            ac_bus_volts: [115.0, 114.5, 0.0, 113.9],
            dc_bus_volts: [28.2, 0.0],
            ac_bus_powered: [true, true, false, true],
            dc_bus_powered: [true, false],
            ..flying_truth()
        };
        let published = run(&mut live, &truth, &Faults::default(), 1);
        assert_eq!(published["DEEP_ELEC_AC_1_FBW_RAW_POTENTIAL"], 115.0);
        assert_eq!(published["DEEP_ELEC_AC_2_FBW_RAW_POTENTIAL"], 114.5);
        assert_eq!(published["DEEP_ELEC_AC_3_FBW_RAW_POTENTIAL"], 0.0);
        assert_eq!(published["DEEP_ELEC_AC_4_FBW_RAW_POTENTIAL"], 113.9);
        assert_eq!(published["DEEP_ELEC_DC_1_FBW_RAW_POTENTIAL"], 28.2);
        assert_eq!(published["DEEP_ELEC_DC_2_FBW_RAW_POTENTIAL"], 0.0);
        assert_eq!(published["DEEP_ELEC_AC_1_FBW_RAW_IS_POWERED"], 1.0);
        assert_eq!(published["DEEP_ELEC_AC_3_FBW_RAW_IS_POWERED"], 0.0);
        assert_eq!(published["DEEP_ELEC_DC_1_FBW_RAW_IS_POWERED"], 1.0);
        assert_eq!(published["DEEP_ELEC_DC_2_FBW_RAW_IS_POWERED"], 0.0);
        assert!(published.contains_key("ELEC_AC_1_BUS_POTENTIAL"), "the override this area performs must still happen -- the raw names are additional, not a replacement");
    }

    #[test]
    fn arming_gen_1_winding_degradation_sags_its_own_bus_the_way_its_effect_says() {
        board::clear();
        let truth = flying_truth();

        let mut healthy = ElectricalLive::new();
        let before = run(&mut healthy, &truth, &Faults::default(), 30);

        let id = {
            let mut reg = Registry::default();
            super::super::registry::register(&mut reg);
            reg.failures
                .iter()
                .find(|f| f.component == "24_elec.vfg-1" && f.model_field.ends_with("winding_degradation"))
                .expect("GEN 1 winding degradation must be registered")
                .id
        };

        board::clear();
        let mut degraded = ElectricalLive::new();
        let after = run(&mut degraded, &truth, &Faults::from_pairs([(id, 1.0)]), 30);

        assert!(
            after["ELEC_AC_1_BUS_POTENTIAL"] < before["ELEC_AC_1_BUS_POTENTIAL"] - 0.05,
            "a fully degraded stator must sag its own bus: {} V healthy vs {} V degraded",
            before["ELEC_AC_1_BUS_POTENTIAL"],
            after["ELEC_AC_1_BUS_POTENTIAL"]
        );
    }

    #[test]
    fn arming_an_ac_1_bus_short_collapses_that_bus_and_kills_the_loads_on_it() {
        board::clear();
        let truth = flying_truth();
        let id = {
            let mut reg = Registry::default();
            super::super::registry::register(&mut reg);
            reg.failures.iter().find(|f| f.component == "24_elec.bus.AC1").expect("the AC1 bus short must be registered").id
        };
        let mut live = ElectricalLive::new();
        let healthy = run(&mut live, &truth, &Faults::default(), 20);
        assert_eq!(healthy["ELEC_LOAD_cab-fan-1_POWERED"], 1.0, "CAB FAN 1 sits on AC1 and should be running");

        board::clear();
        let mut shorted = ElectricalLive::new();
        let slow = Truth { dt_s: 0.05, ..truth };
        let after = run(&mut shorted, &slow, &Faults::from_pairs([(id, 1.0)]), 400);
        assert!(
            after["ELEC_AC_1_BUS_POTENTIAL"] < healthy["ELEC_AC_1_BUS_POTENTIAL"] * 0.5,
            "a bus short must end with the bus collapsed, not dented: {} V",
            after["ELEC_AC_1_BUS_POTENTIAL"]
        );
        assert_eq!(after["ELEC_BKR_feeder-AC1_CLOSED"], 0.0, "the feeder breaker protecting AC1 is what clears it");
        assert_eq!(after["ELEC_LOAD_cab-fan-1_POWERED"], 0.0, "a collapsed bus cannot run its own loads");
    }

    #[test]
    fn a_pulled_breaker_takes_its_own_load_dead_and_releasing_it_brings_it_back() {
        board::clear();
        let truth = flying_truth();
        let mut live = ElectricalLive::new();
        run(&mut live, &truth, &Faults::default(), 10);
        let idx = topology().breaker_index["cab-fan-1"];

        board::with_board_mut(|b| {
            b.breaker_open_cmd = vec![0.0; topology().breaker_count];
            b.breaker_open_cmd[idx] = 1.0;
        });
        let opened = run(&mut live, &truth, &Faults::default(), 5);
        assert_eq!(opened["ELEC_BKR_cab-fan-1_CLOSED"], 0.0);
        assert_eq!(opened["ELEC_LOAD_cab-fan-1_POWERED"], 0.0, "a load behind an open breaker draws nothing");
        assert_eq!(opened["ELEC_BKR_cab-fan-1_CURRENT_A"], 0.0);
        assert_eq!(opened["ELEC_LOAD_cab-fan-2_POWERED"], 1.0, "its neighbour on another bus is untouched");

        board::with_board_mut(|b| b.breaker_open_cmd[idx] = 0.0);
        let closed = run(&mut live, &truth, &Faults::default(), 5);
        assert_eq!(closed["ELEC_BKR_cab-fan-1_CLOSED"], 1.0);
        assert_eq!(closed["ELEC_LOAD_cab-fan-1_POWERED"], 1.0);
        board::clear();
    }

    #[test]
    fn a_representative_sample_of_the_new_gap_closing_loads_goes_dead_when_its_own_breaker_is_pulled() {
        board::clear();
        let truth = flying_truth();
        for id in ["satcom", "dfdr", "fadec-1a", "crew-o2-shutoff", "fuel-valve-0-pos-ind"] {
            let mut live = ElectricalLive::new();
            let healthy = run(&mut live, &truth, &Faults::default(), 20);
            let power_var = format!("ELEC_LOAD_{id}_POWERED");
            let current_var = format!("ELEC_BKR_{id}_CURRENT_A");
            assert_eq!(healthy[&power_var], 1.0, "{id} should be powered and drawing on a healthy aircraft");
            assert!(healthy[&current_var] > 0.0, "{id}'s own breaker should carry real current now it protects a modelled load, got {}", healthy[&current_var]);

            let idx = topology().breaker_index[id];
            board::with_board_mut(|b| {
                b.breaker_open_cmd = vec![0.0; topology().breaker_count];
                b.breaker_open_cmd[idx] = 1.0;
            });
            let opened = run(&mut live, &truth, &Faults::default(), 5);
            assert_eq!(opened[&power_var], 0.0, "{id} must go dead once its own breaker is pulled");
            assert_eq!(opened[&current_var], 0.0);
            board::clear();
        }
    }

    #[test]
    fn transit_only_loads_stay_dead_on_a_steady_state_aircraft_with_nothing_commanding_them() {
        board::clear();
        let mut live = ElectricalLive::new();
        let published = run(&mut live, &flying_truth(), &Faults::default(), 20);
        for id in ["ignition-1a", "ignition-4b", "eng-fire-bottle-1-squib-1", "apu-fire-bottle-squib-2", "apu-start-contactor", "cargo-door-fwd-actuator-ctl", "cargo-door-aft-actuator-ctl"] {
            assert_eq!(published[&format!("ELEC_LOAD_{id}_POWERED")], 0.0, "{id} is not presently commanded and must stay dead, not silently on");
        }
        board::clear();
    }

    #[test]
    fn an_ignition_exciter_draws_only_while_its_engine_is_being_started() {
        board::clear();
        let mut live = ElectricalLive::new();

        let mut cranking = flying_truth();
        cranking.engine_running = [false; 4];
        cranking.engine_n1_frac = [0.0; 4];
        cranking.apu_running = true;
        cranking.controls.starter_engaged[0] = true;
        let during_start = run(&mut live, &cranking, &Faults::default(), 5);
        assert_eq!(during_start["ELEC_LOAD_ignition-1a_POWERED"], 1.0, "exciter A must draw while engine 1 is being cranked");
        assert_eq!(during_start["ELEC_LOAD_ignition-1b_POWERED"], 1.0, "exciter B must draw while engine 1 is being cranked");
        assert_eq!(during_start["ELEC_LOAD_ignition-2a_POWERED"], 0.0, "engine 2's own exciter must be untouched by engine 1's start");

        let mut running = flying_truth();
        running.apu_running = true;
        let after_start = run(&mut live, &running, &Faults::default(), 5);
        assert_eq!(after_start["ELEC_LOAD_ignition-1a_POWERED"], 0.0, "a running engine's exciter must go dead once the starter disengages");
        board::clear();
    }

    #[test]
    fn ground_power_actually_energises_the_ground_service_buses() {
        board::clear();
        let cold_dark = Truth { on_ground: true, ..Truth::default() };
        let mut unplugged = ElectricalLive::new();
        let dark = run(&mut unplugged, &cold_dark, &Faults::default(), 20);
        assert_eq!(dark["ELEC_AC_GND_FLT_SVC_BUS_IS_POWERED"], 0.0, "a cold, dark, unplugged aircraft must have no ground-service power");

        board::clear();
        let plugged_in = Truth { on_ground: true, gpu_plugged_in: true, ..Truth::default() };
        let mut plugged = ElectricalLive::new();
        let lit = run(&mut plugged, &plugged_in, &Faults::default(), 20);
        assert_eq!(lit["ELEC_AC_GND_FLT_SVC_BUS_IS_POWERED"], 1.0, "a plugged-in GPU must actually energise the AC ground-service bus");
        assert!(lit["ELEC_AC_GND_FLT_SVC_BUS_POTENTIAL"] > 100.0, "{}", lit["ELEC_AC_GND_FLT_SVC_BUS_POTENTIAL"]);
        board::clear();
    }

    #[test]
    fn the_rat_deploy_solenoid_only_draws_while_an_emergency_is_commanding_the_rat_out() {
        board::clear();
        let mut live = ElectricalLive::new();
        let normal = run(&mut live, &flying_truth(), &Faults::default(), 20);
        assert_eq!(normal["ELEC_LOAD_rat-deploy-solenoid_POWERED"], 0.0, "no emergency: the solenoid must not be drawing");

        let dead_truth = Truth {
            dt_s: 1.0 / 30.0,
            on_ground: false,
            engine_n1_frac: [0.0; 4],
            engine_running: [false; 4],
            environment: crate::deep::integration::weather_truth::EnvironmentTruth { tas_ms: 150.0, ..Truth::default().environment },
            ..Truth::default()
        };

        let mut saw_solenoid_on = false;
        let mut published = BTreeMap::new();
        for _ in 0..60 {
            live.tick(&dead_truth, &Faults::default());
            published.clear();
            live.publish(&mut |n, v| {
                published.insert(n.to_string(), v);
            });
            if published["ELEC_LOAD_rat-deploy-solenoid_POWERED"] == 1.0 {
                saw_solenoid_on = true;
                assert_eq!(published["ELEC_EMER_CONFIG_ACTIVE"], 1.0, "the solenoid must only draw while an emergency is actually commanding the RAT out");
                break;
            }
        }
        assert!(saw_solenoid_on, "the RAT deploy solenoid never drew any current across a full emergency-configuration transition");

        let after = run(&mut live, &dead_truth, &Faults::default(), 10);
        assert_eq!(after["ELEC_LOAD_rat-deploy-solenoid_POWERED"], 0.0, "once the RAT is deployed the solenoid has nothing left to do and must go dead again");
        board::clear();
    }

    #[test]
    fn wiring_damage_published_onto_the_board_opens_the_load_it_names() {
        board::clear();
        let truth = flying_truth();
        let mut live = ElectricalLive::new();
        run(&mut live, &truth, &Faults::default(), 10);
        let idx = topology().load_index["cab-fan-1"];
        board::with_board_mut(|b| {
            b.load_open = vec![0.0; topology().load_count];
            b.load_open[idx] = 1.0;
        });
        let after = run(&mut live, &truth, &Faults::default(), 5);
        assert_eq!(after["ELEC_LOAD_cab-fan-1_POWERED"], 0.0, "a burnt-through conductor carries nothing");
        board::clear();
    }

    #[test]
    fn losing_every_generator_in_flight_deploys_the_rat_and_raises_emergency_config() {
        board::clear();
        let mut live = ElectricalLive::new();
        let truth = Truth { dt_s: 1.0 / 30.0, on_ground: false, engine_n1_frac: [0.0; 4], engine_running: [false; 4], environment: crate::deep::integration::weather_truth::EnvironmentTruth { tas_ms: 150.0, ..Truth::default().environment }, ..Truth::default() };
        let published = run(&mut live, &truth, &Faults::default(), 30);
        assert_eq!(published["ELEC_EMER_CONFIG_ACTIVE"], 1.0);
        assert_eq!(published["ELEC_RAT_DEPLOYED"], 1.0);
        assert_eq!(published["ELEC_COMMERCIAL_SHED_ACTIVE"], 1.0, "emergency configuration sheds the commercial load");
        assert_eq!(published["ELEC_AC_ESS_SHED_BUS_IS_POWERED"], 0.0, "the shed bus is what gets shed");
    }

    #[test]
    fn it_plugs_into_deep_and_publishes_every_variable_its_ecam_triggers_read() {
        board::clear();
        let mut deep = Deep::new().with_area(live_system());
        let mut published = BTreeMap::new();
        deep.tick(flying_truth(), &Faults::default(), &mut |n, v| {
            published.insert(n.to_string(), v);
        });
        for name in [
            "ELEC_GEN_1_FAULT",
            "ELEC_APU_GEN_FAULT",
            "ELEC_TR_1_FAULT",
            "ELEC_BAT_1_FAULT",
            "ELEC_AC_1_BUS_POTENTIAL",
            "ELEC_AC_1_BUS_IS_POWERED",
            "ELEC_AC_2_BUS_IS_POWERED",
            "ELEC_AC_3_BUS_IS_POWERED",
            "ELEC_AC_4_BUS_IS_POWERED",
            "ELEC_GALLEY_SHED_ACTIVE",
            "ELEC_COMMERCIAL_SHED_ACTIVE",
        ] {
            assert!(published.contains_key(name), "{name} is read by an ECAM trigger but nobody publishes it");
        }
        assert_eq!(deep.area_names(), vec!["electrical"]);
        board::clear();
    }

    #[test]
    #[ignore = "diagnostic"]
    fn diagnose() {
        board::clear();
        let mut live = ElectricalLive::new();
        let p = run(&mut live, &flying_truth(), &Faults::default(), 30);
        println!("AC1 {} Hz {} demand {} cap {}", p["ELEC_AC_1_BUS_POTENTIAL"], p["ELEC_AC_1_BUS_FREQUENCY"], p["ELEC_TOTAL_DEMAND_W"], p["ELEC_AVAILABLE_CAPACITY_W"]);
        println!("galley shed {} commercial {}", p["ELEC_GALLEY_SHED_ACTIVE"], p["ELEC_COMMERCIAL_SHED_ACTIVE"]);
        let idx = topology().breaker_index["cab-fan-1"];
        let b = &live.net.breakers[idx];
        println!("cab-fan-1 bkr closed {} rated {} I {} heat {} cause {:?}", b.closed, b.rated_a, b.current_a, b.heat_fraction(), b.trip_cause);
        let li = topology().load_index["cab-fan-1"];
        println!("cab-fan-1 load powered {} I {} feed {:?}", live.net.loads[li].powered, live.net.loads[li].current_a, live.net.loads[li].active_feed);
        let mut open: Vec<&str> = live.net.breakers.iter().filter(|b| !b.closed).map(|b| b.id).collect();
        open.sort_unstable();
        println!("open breakers ({}): {:?}", open.len(), &open[..open.len().min(30)]);
        let dead: Vec<(&str, &str)> = live.net.loads.iter().filter(|l| !l.powered).map(|l| (l.spec.id, l.spec.bus.label())).collect();
        println!("unpowered loads {} of {}: {:?}", dead.len(), live.net.loads.len(), dead);
        for b in super::ALL_BUS_IDS {
            let p: f64 = live.net.loads.iter().filter(|l| l.spec.bus == b).map(|l| l.spec.rated_power_w).sum();
            let i: f64 = live.net.loads.iter().filter(|l| l.spec.bus == b).map(|l| l.current_a).sum();
            println!("{:>16} = {:8.2} V  rated {:9.0} W  I {:8.1} A", b.label(), live.net.bus(b).voltage, p, i);
        }
        for c in &live.net.contactors {
            println!("contactor {:>18} cmd {} closed {} -> {}", c.id, c.commanded_closed, c.closed, c.to.label());
        }
        let mut top: Vec<(&str, f64, f64)> = live.net.loads.iter().filter(|l| l.spec.bus == BusId::DcEss).map(|l| (l.spec.id, l.current_a, l.spec.rated_power_w)).collect();
        top.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
        println!("DC_ESS top loads {:?}", &top[..top.len().min(8)]);
    }

    #[test]
    #[ignore = "diagnostic"]
    fn frame_cost() {
        board::clear();
        let mut live = ElectricalLive::new();
        let truth = flying_truth();
        let faults = Faults::default();
        for _ in 0..30 {
            live.tick(&truth, &faults);
            let mut out = |_: &str, _: f64| {};
            live.publish(&mut out);
        }
        let n = 2000;
        let start = std::time::Instant::now();
        for _ in 0..n {
            live.tick(&truth, &faults);
            let mut out = |_: &str, _: f64| {};
            live.publish(&mut out);
        }
        let elapsed = start.elapsed();
        let per_tick_us = elapsed.as_secs_f64() * 1_000_000.0 / n as f64;
        println!("electrical: {per_tick_us:.1} us/tick over {n} ticks ({:.3} ms total)", elapsed.as_secs_f64() * 1000.0);
        println!("budget at 60 Hz: 16667 us/frame for the whole plugin; at 30 Hz: 33333 us/frame");
        board::clear();
    }

    #[test]
    fn nothing_produces_a_nan_at_rest_or_at_zero_dt() {
        board::clear();
        let mut live = ElectricalLive::new();
        let truth = Truth { dt_s: 0.0, ..Truth::default() };
        let published = run(&mut live, &truth, &Faults::default(), 3);
        for (name, v) in &published {
            assert!(v.is_finite(), "{name} is {v}");
        }
    }

    fn derived(live: &ElectricalLive) -> BTreeMap<u64, f64> {
        let mut out = BTreeMap::new();
        live.derived_failures(&mut |d| {
            out.insert(d.fbw_id, d.magnitude);
        });
        out
    }

    #[test]
    fn the_coupling_table_matches_what_the_area_actually_emits() {
        board::clear();
        let live = ElectricalLive::new();
        let table = coupling_table();
        let mut emitted: Vec<(u64, &'static str)> = Vec::new();
        live.each_coupling(&mut |d| emitted.push((d.fbw_id, d.deep_component)));
        assert_eq!(emitted, table, "coupling_table() and each_coupling() must walk the same list in the same order");
        assert_eq!(live.names.derived.len(), table.len());

        let catalogue: std::collections::BTreeSet<u64> = crate::failures::a380_failures().into_iter().map(|(id, _)| id).collect();
        for (id, component) in &table {
            assert!(catalogue.contains(id), "{component} derives {id}, which FlyByWire does not register");
        }
        let mut ids: Vec<u64> = table.iter().map(|(id, _)| *id).collect();
        let before = ids.len();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), before, "two deep components must not claim the same FlyByWire failure");
        assert_eq!(table.len(), 25);
    }

    #[test]
    fn a_healthy_aircraft_tells_flybywire_nothing_at_all() {
        board::clear();
        let mut live = ElectricalLive::new();
        run(&mut live, &flying_truth(), &Faults::default(), 60);
        assert!(derived(&live).values().all(|&m| m == 0.0), "cruise: {:?}", derived(&live));

        board::clear();
        let mut cold = ElectricalLive::new();
        run(&mut cold, &Truth::default(), &Faults::default(), 60);
        assert!(
            derived(&cold).values().all(|&m| m == 0.0),
            "a cold dark aircraft has dead buses, but a dead bus is not a failed one: {:?}",
            derived(&cold)
        );
    }

    #[test]
    fn a_cleared_bus_short_tells_flybywire_that_bus_is_failed() {
        board::clear();
        let truth = flying_truth();
        let id = {
            let mut reg = Registry::default();
            super::super::registry::register(&mut reg);
            reg.failures.iter().find(|f| f.component == "24_elec.bus.AC1").expect("the AC1 bus short must be registered").id
        };
        let mut live = ElectricalLive::new();
        let slow = Truth { dt_s: 0.05, ..truth };
        let published = run(&mut live, &slow, &Faults::from_pairs([(id, 1.0)]), 400);

        assert_eq!(published["ELEC_BKR_feeder-AC1_CLOSED"], 0.0, "setup: the feeder must have tripped");
        assert_eq!(derived(&live).get(&24_100), Some(&1.0), "AC1 behind an open feeder must reach FlyByWire as a failed bus");
        assert_eq!(derived(&live).get(&24_101), Some(&0.0), "and no other bus may be blamed for it");
        assert_eq!(published["DEEP_DERIVED_FBW_FAILURE_24100"], 1.0, "and it must be visible, not silent");
    }

    #[test]
    fn a_bus_short_is_never_blamed_on_the_generator_feeding_that_bus() {
        board::clear();
        let truth = flying_truth();
        let id = {
            let mut reg = Registry::default();
            super::super::registry::register(&mut reg);
            reg.failures.iter().find(|f| f.component == "24_elec.bus.AC2").expect("the AC2 bus short must be registered").id
        };
        let mut live = ElectricalLive::new();
        let slow = Truth { dt_s: 0.05, ..truth };
        run(&mut live, &slow, &Faults::from_pairs([(id, 1.0)]), 400);
        let d = derived(&live);
        assert_eq!(d.get(&24_101), Some(&1.0), "AC2 is isolated behind its own feeder, and that is the verdict");
        for gen in FBW_GENERATOR {
            assert_eq!(d.get(&gen), Some(&0.0), "no generator may be blamed for a busbar fault ({gen})");
        }
    }

    #[test]
    fn a_stator_degraded_past_half_reaches_flybywire_as_a_failed_generator() {
        board::clear();
        let truth = flying_truth();
        let id = {
            let mut reg = Registry::default();
            super::super::registry::register(&mut reg);
            reg.failures
                .iter()
                .find(|f| f.component == "24_elec.vfg-1" && f.model_field.ends_with("winding_degradation"))
                .expect("GEN 1 winding degradation must be registered")
                .id
        };

        let mut mild = ElectricalLive::new();
        run(&mut mild, &truth, &Faults::from_pairs([(id, 0.3)]), 30);
        assert_eq!(derived(&mild).get(&24_020), Some(&0.0), "a third-degraded stator still holds its bus; FlyByWire is told nothing");

        board::clear();
        let mut gone = ElectricalLive::new();
        run(&mut gone, &truth, &Faults::from_pairs([(id, 1.0)]), 30);
        assert_eq!(derived(&gone).get(&24_020), Some(&1.0), "a fully degraded stator is a failed generator");
        assert_eq!(derived(&gone).get(&24_021), Some(&0.0), "and only that one");
    }

    #[test]
    fn a_degraded_transformer_rectifier_reaches_flybywires_own_tr() {
        board::clear();
        let truth = flying_truth();
        let id = {
            let mut reg = Registry::default();
            super::super::registry::register(&mut reg);
            reg.failures.iter().find(|f| f.component == "24_elec.tr-ess").expect("the TR ESS failure must be registered").id
        };
        let mut live = ElectricalLive::new();
        run(&mut live, &truth, &Faults::from_pairs([(id, 1.0)]), 30);
        let d = derived(&live);
        assert_eq!(d.get(&24_002), Some(&1.0), "TR ESS is FlyByWire's TransformerRectifier(3)");
        assert_eq!(d.get(&24_000), Some(&0.0));
        assert_eq!(d.get(&24_001), Some(&0.0));
        assert_eq!(d.get(&24_003), Some(&0.0));
    }

    #[test]
    fn a_derived_bus_failure_changes_flybywires_own_solve() {
        use crate::aspects::test_vars::TestVars;
        use std::time::Duration;
        use systems::simulation::{Simulation, StartState};

        board::clear();
        let id = {
            let mut reg = Registry::default();
            super::super::registry::register(&mut reg);
            reg.failures.iter().find(|f| f.component == "24_elec.bus.DC_HOT1").expect("the DC HOT 1 bus short must be registered").id
        };
        let mut live = ElectricalLive::new();
        let slow = Truth { dt_s: 0.05, ..Truth::default() };
        run(&mut live, &slow, &Faults::from_pairs([(id, 1.0)]), 600);
        let verdict = derived(&live);
        assert_eq!(verdict.get(&24_113), Some(&1.0), "setup: the deep model must have concluded DC HOT 1 is isolated");

        let mut vars = TestVars::default();
        let mut sim = Simulation::new(StartState::Apron, a380_systems::A380::new, &mut vars);
        for bat in ["BAT_1", "BAT_2", "BAT_ESS", "BAT_APU"] {
            vars.set(&format!("A32NX_OVHD_ELEC_{bat}_PB_IS_AUTO"), 1.0);
        }
        let tick = |sim: &mut Simulation<a380_systems::A380>, vars: &mut TestVars, n: usize| {
            for i in 0..n {
                sim.tick(Duration::from_millis(50), 100.0 + i as f64 * 0.05, vars);
            }
        };
        tick(&mut sim, &mut vars, 20);
        assert_eq!(vars.value("A32NX_ELEC_DC_HOT_1_BUS_IS_POWERED"), 1.0, "setup: FlyByWire's own hot bus is alive on its battery");

        let types: Vec<systems::failures::FailureType> =
            crate::failures::a380_failures().into_iter().filter(|(fid, _)| verdict.get(fid).copied().unwrap_or(0.0) > 0.0).map(|(_, t)| t).collect();
        assert!(!types.is_empty());
        sim.update_active_failures(types.into_iter().collect());
        tick(&mut sim, &mut vars, 20);
        assert_eq!(
            vars.value("A32NX_ELEC_DC_HOT_1_BUS_IS_POWERED"),
            0.0,
            "the deep model's verdict must change FlyByWire's own solve, not just a display"
        );
    }

    #[test]
    fn a_second_aircraft_in_the_same_process_does_not_inherit_the_first_ones_board() {
        let truth = flying_truth();
        let faults = Faults::default();

        let mut first = ElectricalLive::new();
        let a = run(&mut first, &truth, &faults, 30);
        assert!(board::with_board(|b| b.breaker_current_a.iter().any(|&i| i > 0.0)), "the first aircraft must leave real current on the board, or this test proves nothing");

        let mut second = ElectricalLive::new();
        let b = run(&mut second, &truth, &faults, 30);

        let differing: Vec<&String> = a.iter().filter(|(k, v)| b.get(*k).map_or(true, |w| w != *v)).map(|(k, _)| k).collect();
        assert!(differing.is_empty(), "{} published variables differ between two identical healthy aircraft, e.g. {:?}", differing.len(), differing.iter().take(6).collect::<Vec<_>>());
    }

    #[test]
    fn each_generator_publishes_its_own_delivered_power() {
        let truth = flying_truth();
        let published = run(&mut ElectricalLive::new(), &truth, &Faults::default(), 30);
        for n in 1..=4 {
            let w = published[&format!("ELEC_ENG_GEN_{n}_LOAD_W")];
            assert!(w > 0.0 && w.is_finite(), "engine generator {n} is on line and carrying the aircraft, but publishes {w} W");
        }
        for n in 1..=2 {
            let name = format!("ELEC_APU_GEN_{n}_LOAD_W");
            let w = published[&name];
            assert!(w.is_finite() && w >= 0.0, "{name} published {w}");
        }

        let apu = Truth { apu_running: true, on_ground: true, engine_running: [false; 4], engine_n2_frac: [0.0; 4], ..flying_truth() };
        let published = run(&mut ElectricalLive::new(), &apu, &Faults::default(), 30);
        let total: f64 = (1..=2).map(|n| published[&format!("ELEC_APU_GEN_{n}_LOAD_W")]).sum();
        assert!(total > 0.0, "both APU generators on line carrying the whole aircraft publish {total} W between them");
    }

    #[test]
    fn a_battery_only_aircraft_settles_instead_of_limit_cycling_its_essential_buses() {
        const ESSENTIAL_CHAIN: [BusId; 5] = [BusId::AcEmer, BusId::AcEss, BusId::AcEssShed, BusId::DcEss, BusId::DcEssShed];
        const FRAMES: usize = 900;
        const SETTLE: usize = 90;

        board::clear();
        let mut live = ElectricalLive::new();
        let truth = Truth::default();
        let faults = Faults::default();

        let (mut dc_ess_changes, mut ac_ess_changes, mut load_changes) = (0usize, 0usize, 0usize);
        let mut previous: Option<(bool, bool, Vec<bool>)> = None;
        for frame in 0..FRAMES {
            live.tick(&truth, &faults);
            let dc_ess = live.report.bus_powered[BusId::DcEss.index()];
            let ac_ess = live.report.bus_powered[BusId::AcEss.index()];
            let loads: Vec<bool> = live.net.loads.iter().filter(|l| ESSENTIAL_CHAIN.contains(&l.spec.bus)).map(|l| l.powered).collect();
            if let Some((was_dc, was_ac, was_loads)) = &previous {
                if frame >= SETTLE {
                    dc_ess_changes += (*was_dc != dc_ess) as usize;
                    ac_ess_changes += (*was_ac != ac_ess) as usize;
                    load_changes += was_loads.iter().zip(&loads).filter(|(a, b)| a != b).count();
                }
            }
            previous = Some((dc_ess, ac_ess, loads));
        }

        assert_eq!(
            load_changes, 0,
            "the essential buses' own loads changed state {load_changes} times over {} settled frames: something on the essential chain is limit-cycling",
            FRAMES - SETTLE
        );
        assert!(
            dc_ess_changes <= 2,
            "DC ESS changed powered state {dc_ess_changes} times in {} settled frames; a battery-only aircraft has one configuration, not a cycle",
            FRAMES - SETTLE
        );
        assert!(ac_ess_changes <= 2, "AC ESS changed powered state {ac_ess_changes} times in {} settled frames", FRAMES - SETTLE);
        board::clear();
    }

    #[test]
    fn ground_power_connected_to_an_already_dark_aircraft_energises_the_main_ac_network() {
        board::clear();
        let mut live = ElectricalLive::new();
        let cold = Truth { on_ground: true, ..Truth::default() };
        let dark = run(&mut live, &cold, &Faults::default(), 60);
        assert_eq!(dark["ELEC_AC_1_BUS_IS_POWERED"], 0.0, "setup: the aircraft must really be dark before the cart arrives, or this test proves nothing");
        assert_eq!(dark["ELEC_AC_1_BUS_POTENTIAL"], 0.0, "setup: and dark means 0 V, not a tie ring holding its own seed up");

        let plugged_in = Truth { gpu_plugged_in: true, ..cold };
        let published = run(&mut live, &plugged_in, &Faults::default(), 60);
        for n in 1..=4 {
            assert_eq!(
                published[&format!("ELEC_AC_{n}_BUS_IS_POWERED")],
                1.0,
                "ground power must carry the main AC network: AC{n} sits at {} V",
                published[&format!("ELEC_AC_{n}_BUS_POTENTIAL")]
            );
            assert!(
                published[&format!("ELEC_AC_{n}_BUS_POTENTIAL")] >= AC_UNDERVOLTAGE_TRIP_V,
                "AC{n} must be inside MIL-STD-704F's own utilisation band on ground power, not merely above zero: {} V",
                published[&format!("ELEC_AC_{n}_BUS_POTENTIAL")]
            );
        }
        assert_eq!(published["ELEC_AC_GND_FLT_SVC_BUS_IS_POWERED"], 1.0, "and the service bus the cart used to be the only source of");
        assert_eq!(published["ELEC_DC_1_BUS_IS_POWERED"], 1.0, "the TRs behind the main AC buses come up with them");
        board::clear();
    }
}
