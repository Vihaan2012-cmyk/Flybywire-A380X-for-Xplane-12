use super::duct::DuctSectionFaults;
use super::network::{ApuBleedInput, DuctNetwork, DuctNetworkFaults, EngineBleedInput, NetworkInputs, NetworkOutputs, ODLS_ZONE_COUNT, ZONE_COUNT, ZONE_NAMES};
use super::odls::OdlsFaults;
use super::precooler::PrecoolerFaults;
use crate::deep::api::{failure_id, Area as RegArea};
use crate::deep::live::{DerivedFailure, Faults, Truth};

const GAMMA_AIR: f64 = 1.4;
const SEA_LEVEL_DENSITY_KG_M3: f64 = 1.225;
const R_AIR_J_KG_K: f64 = 287.057_005;

const TRENT_900_BYPASS_MDOT_SLS_KG_S: f64 = 1204.0 * 8.7 / 9.7;

const ZONE_VENTILATION_KG_S: f64 = 0.5;
const CP_AIR_J_KGK: f64 = 1005.0;
const ZONE_EXCESS_TIME_CONSTANT_S: f64 = 10.0;

const APU_COOLING_AIR_KG_S: f64 = 30.0;

const APU_LOAD_COMPRESSOR_POLYTROPIC_EFFICIENCY: f64 = 0.8;

fn f(ata: u16, n: u16) -> u64 {
    failure_id(RegArea::PneumaticDucts, ata, n)
}

const FBW_HP_VALVE: [u64; 4] = [36_008, 36_009, 36_010, 36_011];
const FBW_PR_VALVE: [u64; 4] = [36_012, 36_013, 36_014, 36_015];

const EXTRA_PRECOOLER_FAULT_IDS: [u64; 4] = [36_004, 36_005, 36_006, 36_007];

const EXTRA_DUCT_LEAK_IDS: [u64; 4] = [36_000, 36_001, 36_002, 36_003];

const UPSTREAM_VALVE_COMPONENT: &str = "36_pneu.engine_upstream_valve_stage";

const HP_VALVE_REASON: [&str; 4] = [
    "engine 1 HP bleed valve seized at its last position",
    "engine 2 HP bleed valve seized at its last position",
    "engine 3 HP bleed valve seized at its last position",
    "engine 4 HP bleed valve seized at its last position",
];
const PR_VALVE_REASON: [&str; 4] = [
    "engine 1 bleed PR/shutoff valve seized at its last position",
    "engine 2 bleed PR/shutoff valve seized at its last position",
    "engine 3 bleed PR/shutoff valve seized at its last position",
    "engine 4 bleed PR/shutoff valve seized at its last position",
];

fn coupling_table() -> Vec<(u64, &'static str)> {
    let mut v: Vec<(u64, &'static str)> = Vec::new();
    for i in 0..4 {
        v.push((FBW_HP_VALVE[i], UPSTREAM_VALVE_COMPONENT));
    }
    for i in 0..4 {
        v.push((FBW_PR_VALVE[i], UPSTREAM_VALVE_COMPONENT));
    }
    v
}

struct VarNames {
    odls_trip: [String; ODLS_ZONE_COUNT],
    odls_fault: [String; ODLS_ZONE_COUNT],
    odls_loop_a_fault: [String; ODLS_ZONE_COUNT],
    odls_loop_b_fault: [String; ODLS_ZONE_COUNT],
    zone_heat_w: [String; ZONE_COUNT],
    zone_jet_flux: [String; ZONE_COUNT],
    engine_precooler_ovht: [String; 4],
    engine_precooler_outlet_c: [String; 4],
    engine_isolation_open: [String; 4],
    engine_duct_pressure: [String; 4],
    engine_duct_temp_c: [String; 4],
    transfer_pipe_pressure: [String; 4],
    hp_valve_open: [String; 4],
    pr_valve_open: [String; 4],
    start_duct_pressure: [String; 4],
    pack_supply_pressure: [String; 2],
    pack_supply_temp_c: [String; 2],
    wai_duct_pressure: [String; 2],
    wai_duct_temp_c: [String; 2],
    wai_valve_open: [String; 2],
    hyd_reservoir_pressure: [String; 2],
    cross_bleed_open: [String; 3],
    engine_odls_isolated: [String; 4],
    engine_precooler_fouling: [String; 4],
    engine_duct_leak_kg_s: [String; 4],
    apu_duct_leak_kg_s: String,
    pack_leak_kg_s: [String; 2],
    wai_leak_kg_s: [String; 2],
    engine_start_duct_leak_kg_s: [String; 4],
    hyd_reservoir_leak_kg_s: [String; 2],
    pack_regul_fault: [String; 2],
    mixer_press_regul_fault: String,
    ram_air_door_fault: [String; 2],
    press_man_ctl_fault: String,
    cabin_air_extract_vlv_fault: String,
    pack_regul_redundancy_fault: String,
    outflw_vlv_ctl_fault_all: String,
    pack_acm_outlet_temp_c: [String; 2],
    pack_fcv_fault: [[String; 2]; 2],
}

fn per_engine(fmt: impl Fn(usize) -> String) -> [String; 4] {
    std::array::from_fn(|i| fmt(i + 1))
}

impl VarNames {
    fn new() -> Self {
        let side = ["L", "R"];
        let hyd = ["GREEN", "YELLOW"];
        let xbleed = ["L", "C", "R"];
        Self {
            odls_trip: std::array::from_fn(|z| format!("DEEP_PNEU_ODLS_{}_TRIP", ZONE_NAMES[z])),
            odls_fault: std::array::from_fn(|z| format!("DEEP_PNEU_ODLS_{}_FAULT", ZONE_NAMES[z])),
            odls_loop_a_fault: std::array::from_fn(|z| format!("DEEP_PNEU_ODLS_{}_LOOP_A_FAULT", ZONE_NAMES[z])),
            odls_loop_b_fault: std::array::from_fn(|z| format!("DEEP_PNEU_ODLS_{}_LOOP_B_FAULT", ZONE_NAMES[z])),
            zone_heat_w: std::array::from_fn(|z| format!("DEEP_PNEU_ZONE_{}_HEAT_W", ZONE_NAMES[z])),
            zone_jet_flux: std::array::from_fn(|z| format!("DEEP_PNEU_ZONE_{}_JET_FLUX_W_M2", ZONE_NAMES[z])),
            engine_precooler_ovht: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_PRECOOLER_OVHT")),
            engine_precooler_outlet_c: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_PRECOOLER_OUTLET_C")),
            engine_isolation_open: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_ISOLATION_OPEN")),
            engine_duct_pressure: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_DUCT_PRESSURE_PA")),
            engine_duct_temp_c: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_DUCT_TEMPERATURE_C")),
            transfer_pipe_pressure: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_TRANSFER_PRESSURE_PA")),
            hp_valve_open: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_HP_VALVE_OPEN")),
            pr_valve_open: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_PR_VALVE_OPEN")),
            start_duct_pressure: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_START_DUCT_PRESSURE_PA")),
            pack_supply_pressure: std::array::from_fn(|i| format!("DEEP_PNEU_PACK_{}_SUPPLY_PRESSURE_PA", i + 1)),
            pack_supply_temp_c: std::array::from_fn(|i| format!("DEEP_PNEU_PACK_{}_SUPPLY_TEMPERATURE_C", i + 1)),
            wai_duct_pressure: std::array::from_fn(|i| format!("DEEP_PNEU_WAI_{}_DUCT_PRESSURE_PA", side[i])),
            wai_duct_temp_c: std::array::from_fn(|i| format!("DEEP_PNEU_WAI_{}_DUCT_TEMPERATURE_C", side[i])),
            wai_valve_open: std::array::from_fn(|i| format!("DEEP_PNEU_WAI_{}_VALVE_OPEN", side[i])),
            hyd_reservoir_pressure: std::array::from_fn(|i| format!("DEEP_PNEU_HYD_{}_RESERVOIR_PRESSURE_PA", hyd[i])),
            cross_bleed_open: std::array::from_fn(|i| format!("DEEP_PNEU_XBLEED_{}_OPEN", xbleed[i])),
            engine_odls_isolated: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_ODLS_ISOLATED")),
            engine_precooler_fouling: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_PRECOOLER_FOULING")),
            engine_duct_leak_kg_s: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_DUCT_LEAK_KG_S")),
            apu_duct_leak_kg_s: "DEEP_PNEU_APU_DUCT_LEAK_KG_S".to_owned(),
            pack_leak_kg_s: std::array::from_fn(|i| format!("DEEP_PNEU_PACK_{}_LEAK_KG_S", i + 1)),
            wai_leak_kg_s: std::array::from_fn(|i| format!("DEEP_PNEU_WAI_{}_LEAK_KG_S", side[i])),
            engine_start_duct_leak_kg_s: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_START_DUCT_LEAK_KG_S")),
            hyd_reservoir_leak_kg_s: std::array::from_fn(|i| format!("DEEP_PNEU_HYD_{}_RESERVOIR_LEAK_KG_S", hyd[i])),
            pack_regul_fault: std::array::from_fn(|i| format!("DEEP_PNEU_PACK_{}_REGUL_FAULT", i + 1)),
            mixer_press_regul_fault: "DEEP_PNEU_MIXER_PRESS_REGUL_FAULT".to_owned(),
            ram_air_door_fault: std::array::from_fn(|i| format!("DEEP_PNEU_RAM_AIR_{}_FAULT", i + 1)),
            press_man_ctl_fault: "DEEP_PNEU_PRESS_MAN_CTL_FAULT".to_owned(),
            cabin_air_extract_vlv_fault: "DEEP_PNEU_CABIN_AIR_EXTRACT_VLV_FAULT".to_owned(),
            pack_regul_redundancy_fault: "DEEP_PNEU_PACK_REGUL_REDUNDANCY_FAULT".to_owned(),
            outflw_vlv_ctl_fault_all: "DEEP_PNEU_OUTFLW_VLV_CTL_FAULT_ALL".to_owned(),
            pack_acm_outlet_temp_c: std::array::from_fn(|i| format!("DEEP_PNEU_PACK_{}_ACM_OUTLET_TEMPERATURE_C", i + 1)),
            pack_fcv_fault: std::array::from_fn(|p| std::array::from_fn(|v| format!("DEEP_PNEU_PACK_{}_FCV_{}_FAULT", p + 1, v + 1))),
        }
    }
}

pub struct PneumaticDuctsLive {
    network: DuctNetwork,
    faults: DuctNetworkFaults,
    out: NetworkOutputs,
    names: VarNames,
    derived_names: Vec<String>,
    own_zone_excess_state: [f64; ZONE_COUNT],
    pack_regul_fault: [f64; 2],
    mixer_press_regul_fault: f64,
    ram_air_door_fault: [f64; 2],
    press_man_ctl_fault: f64,
    cabin_air_extract_vlv_fault: f64,
    pack_regul_redundancy_fault: bool,
    outflw_vlv_ctl_fault_all: bool,
    pack_acm_outlet_temp_c: [f64; 2],
    pack_fcv_fault: [[f64; 2]; 2],
    pack_flow_insufficient_fwd_crg: bool,
}

impl Default for PneumaticDuctsLive {
    fn default() -> Self {
        Self::new()
    }
}

fn on(b: bool) -> f64 {
    if b {
        1.0
    } else {
        0.0
    }
}

impl PneumaticDuctsLive {
    pub fn new() -> Self {
        Self {
            network: DuctNetwork::new(),
            faults: DuctNetworkFaults::default(),
            out: NetworkOutputs::default(),
            names: VarNames::new(),
            derived_names: coupling_table().into_iter().map(|(id, _)| format!("DEEP_DERIVED_FBW_FAILURE_{id}")).collect(),
            own_zone_excess_state: [0.0; ZONE_COUNT],
            pack_regul_fault: [0.0; 2],
            mixer_press_regul_fault: 0.0,
            ram_air_door_fault: [0.0; 2],
            press_man_ctl_fault: 0.0,
            cabin_air_extract_vlv_fault: 0.0,
            pack_regul_redundancy_fault: false,
            outflw_vlv_ctl_fault_all: false,
            pack_acm_outlet_temp_c: [15.0; 2],
            pack_fcv_fault: [[0.0; 2]; 2],
            pack_flow_insufficient_fwd_crg: false,
        }
    }

    pub fn outputs(&self) -> &NetworkOutputs {
        &self.out
    }

    fn bypass_mdot_kg_s(truth: &Truth, engine: usize) -> f64 {
        if !truth.engine_running[engine] {
            return 0.0;
        }
        let density = truth.environment.ambient_pressure_pa.max(1.0) / (R_AIR_J_KG_K * (truth.environment.sat_c + 273.15).max(1.0));
        TRENT_900_BYPASS_MDOT_SLS_KG_S * truth.engine_n1_frac[engine].clamp(0.0, 1.2) * (density / SEA_LEVEL_DENSITY_KG_M3)
    }

    fn ram_total_temp_k(truth: &Truth) -> f64 {
        let static_k = (truth.environment.sat_c + 273.15).max(1.0);
        let mach = truth.environment.mach();
        static_k * (1.0 + (GAMMA_AIR - 1.0) / 2.0 * mach * mach)
    }

    fn engine_inputs(truth: &Truth) -> [EngineBleedInput; 4] {
        let fan_air_k = Self::ram_total_temp_k(truth);
        std::array::from_fn(|i| EngineBleedInput {
            ip_port_pressure_pa: truth.engine_ip_port_pressure_pa[i].max(0.0),
            ip_port_temp_k: truth.engine_ip_port_temp_k[i].max(1.0),
            hp_port_pressure_pa: truth.engine_hp_port_pressure_pa[i].max(0.0),
            hp_port_temp_k: truth.engine_hp_port_temp_k[i].max(1.0),
            fan_air_available_kg_s: Self::bypass_mdot_kg_s(truth, i),
            fan_air_k,
        })
    }

    fn apu_input(truth: &Truth) -> ApuBleedInput {
        let ambient_pa = truth.environment.ambient_pressure_pa.max(1.0);
        let ambient_k = (truth.environment.sat_c + 273.15).max(1.0);
        let pressure_pa = if truth.apu_running { truth.apu_bleed_pressure_pa.max(0.0) } else { 0.0 };
        let ratio = (pressure_pa / ambient_pa).max(1.0);
        let exponent = (GAMMA_AIR - 1.0) / (GAMMA_AIR * APU_LOAD_COMPRESSOR_POLYTROPIC_EFFICIENCY);
        ApuBleedInput {
            pressure_pa,
            temp_k: ambient_k * ratio.powf(exponent),
            fan_air_available_kg_s: if truth.apu_running { APU_COOLING_AIR_KG_S } else { 0.0 },
            fan_air_k: ambient_k,
        }
    }

    fn apu_bleed_available(truth: &Truth) -> bool {
        truth.controls.apu_bleed_pb_on && truth.apu_running && truth.apu_bleed_pressure_pa > truth.environment.ambient_pressure_pa * 1.05
    }

    fn cross_bleed_command(truth: &Truth) -> f64 {
        if truth.controls.cross_bleed_selector <= 0.5 {
            0.0
        } else if truth.controls.cross_bleed_selector >= 1.5 {
            1.0
        } else {
            on(Self::apu_bleed_available(truth))
        }
    }

    fn inputs(&self, truth: &Truth) -> NetworkInputs {
        let apu_available = Self::apu_bleed_available(truth);
        let cross = Self::cross_bleed_command(truth);
        let recovery_k = Self::recovery_temp_k(truth);
        NetworkInputs {
            dt_s: truth.dt_s,
            ambient_pa: truth.environment.ambient_pressure_pa.max(1.0),
            ambient_k: (truth.environment.sat_c + 273.15).max(1.0),
            engines: Self::engine_inputs(truth),
            apu: Self::apu_input(truth),
            apu_bleed_selected: apu_available,
            apu_bleed_valve_command: on(apu_available),
            cross_bleed_valve_command: [cross; 3],
            pack_valve_open: [[on(truth.controls.pack_pb_on[0]); 2], [on(truth.controls.pack_pb_on[1]); 2]],
            wai_selected: [truth.controls.wing_anti_ice_selected; 2],
            starter_engaged: truth.controls.starter_engaged,
            engine_bleed_pb_auto: truth.controls.engine_bleed_pb_auto,
            zone_air_k: self.zone_air_k(truth, recovery_k),
        }
    }

    fn recovery_temp_k(truth: &Truth) -> f64 {
        const RECOVERY_FACTOR: f64 = 0.9;
        let static_k = (truth.environment.sat_c + 273.15).max(1.0);
        let mach = truth.environment.mach();
        static_k * (1.0 + RECOVERY_FACTOR * (GAMMA_AIR - 1.0) / 2.0 * mach * mach)
    }

    fn zone_air_k(&self, truth: &Truth, recovery_k: f64) -> [f64; ZONE_COUNT] {
        let recovery_c = recovery_k - 273.15;
        std::array::from_fn(|z| {
            let name = format!("THERMAL_ZONE_{}_TEMPERATURE_C", ZONE_NAMES[z].to_ascii_uppercase());
            truth.published.get_or(&name, recovery_c) + 273.15 + self.own_zone_excess_k(z)
        })
    }

    fn own_zone_excess_k(&self, zone: usize) -> f64 {
        self.own_zone_excess_state[zone]
    }

    fn relax_own_zone_excess(&mut self, dt_s: f64) {
        let dt = dt_s.max(0.0);
        let a = (-dt / ZONE_EXCESS_TIME_CONSTANT_S).exp();
        for z in 0..ZONE_COUNT {
            let target = (self.out.zone_heat_w[z] / (ZONE_VENTILATION_KG_S * CP_AIR_J_KGK)).max(0.0);
            self.own_zone_excess_state[z] = target + (self.own_zone_excess_state[z] - target) * a;
        }
    }

    fn apply_faults(&mut self, faults: &Faults) {
        let duct = |leak_id: u64, rupture_id: u64, insulation_id: u64| DuctSectionFaults {
            leak: faults.get(leak_id),
            rupture: faults.get(rupture_id),
            insulation_damage: faults.get(insulation_id),
        };
        let precooler = |fouling: u64, fav: u64, sensor: u64, check: u64| PrecoolerFaults {
            fouling: faults.get(fouling),
            fan_air_valve_stuck: faults.get(fav),
            temp_sensor_fault: faults.get(sensor),
            check_valve_failure: faults.get(check),
        };

        let engine_duct = duct(f(36, 1), f(36, 2), f(36, 3));
        let engine_precooler = precooler(f(36, 4), f(36, 5), f(36, 6), f(36, 7));
        let upstream_ip = faults.get(f(36, 17));
        let upstream_hp = faults.get(f(36, 15));
        let upstream_pr = faults.get(f(36, 16));
        let start_duct = duct(f(36, 21), f(36, 22), f(36, 23));
        let start_check_valve = faults.get(f(36, 24));
        for i in 0..4 {
            self.faults.engine_duct[i] = DuctSectionFaults { leak: engine_duct.leak.max(faults.get(EXTRA_DUCT_LEAK_IDS[i])), ..engine_duct };
            self.faults.engine_precooler[i] = PrecoolerFaults {
                fouling: engine_precooler.fouling.max(faults.get(EXTRA_PRECOOLER_FAULT_IDS[i])),
                ..engine_precooler
            };
            self.faults.upstream[i].hp_valve_stuck = upstream_hp.max(faults.get(FBW_HP_VALVE[i]));
            self.faults.upstream[i].pr_valve_stuck = upstream_pr.max(faults.get(FBW_PR_VALVE[i]));
            self.faults.upstream[i].ip_check_valve_stuck_closed = upstream_ip;
            self.faults.start[i] = start_duct;
            self.faults.start_check_valve_failure[i] = start_check_valve;
        }

        self.faults.apu_duct = duct(f(36, 8), f(36, 9), f(36, 10));
        self.faults.apu_precooler = precooler(f(36, 11), f(36, 12), f(36, 13), f(36, 14));

        let pack_duct = duct(f(36, 18), f(36, 19), f(36, 20));
        let hyd_duct = duct(f(36, 25), f(36, 26), f(36, 27));
        let wai_duct = duct(f(30, 1), f(30, 2), f(30, 3));
        for i in 0..2 {
            self.faults.packs[i] = pack_duct;
            self.faults.hyd_reservoir[i] = hyd_duct;
            self.faults.wai[i] = wai_duct;
        }

        let odls = OdlsFaults {
            loop_a_open: faults.get(f(36, 28)),
            loop_a_short: faults.get(f(36, 29)),
            loop_b_open: faults.get(f(36, 30)),
            loop_b_short: faults.get(f(36, 31)),
            false_detection: faults.get(f(36, 32)),
        };
        for z in 0..ODLS_ZONE_COUNT {
            self.faults.odls[z] = odls;
        }
    }

    fn each_coupling(&self, out: &mut dyn FnMut(DerivedFailure)) {
        for i in 0..4 {
            out(DerivedFailure {
                fbw_id: FBW_HP_VALVE[i],
                magnitude: self.faults.upstream[i].hp_valve_stuck.clamp(0.0, 1.0),
                deep_component: UPSTREAM_VALVE_COMPONENT,
                reason: HP_VALVE_REASON[i],
            });
        }
        for i in 0..4 {
            out(DerivedFailure {
                fbw_id: FBW_PR_VALVE[i],
                magnitude: self.faults.upstream[i].pr_valve_stuck.clamp(0.0, 1.0),
                deep_component: UPSTREAM_VALVE_COMPONENT,
                reason: PR_VALVE_REASON[i],
            });
        }
    }
}

impl crate::deep::live::Area for PneumaticDuctsLive {
    fn name(&self) -> &'static str {
        "pneumatic_ducts"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        self.apply_faults(faults);
        let inputs = self.inputs(truth);
        self.out = self.network.step(&inputs, &self.faults);
        self.relax_own_zone_excess(truth.dt_s);

        self.pack_regul_fault = [faults.get(f(21, 1)), faults.get(f(21, 2))];
        self.mixer_press_regul_fault = faults.get(f(21, 3));
        self.ram_air_door_fault = [faults.get(f(21, 4)), faults.get(f(21, 5))];
        self.press_man_ctl_fault = faults.get(f(21, 6));
        self.cabin_air_extract_vlv_fault = faults.get(f(21, 7));
        let pack_has_a_fault = |i: usize| self.pack_regul_fault[i] > 0.0 || self.ram_air_door_fault[i] > 0.0 || truth.fdac_channel_failure[i][0] || truth.fdac_channel_failure[i][1];
        self.pack_regul_redundancy_fault = pack_has_a_fault(0) && pack_has_a_fault(1);
        self.outflw_vlv_ctl_fault_all = truth.ocsm_channel_failure.iter().all(|ch| ch[0] && ch[1]);

        let ambient_c = truth.environment.sat_c;
        for i in 0..2 {
            let inlet_c = self.out.pack_supply_temp_k[i] - 273.15;
            let acm_overheat = faults.get(f(21, 8 + i as u16));
            let cooling_effectiveness = (1.0 - acm_overheat).clamp(0.0, 1.0);
            self.pack_acm_outlet_temp_c[i] = inlet_c - cooling_effectiveness * (inlet_c - ambient_c);
        }

        for p in 0..2 {
            for v in 0..2 {
                self.pack_fcv_fault[p][v] = faults.get(f(21, 10 + (p * 2 + v) as u16));
            }
        }

        self.pack_flow_insufficient_fwd_crg = truth.pack_flow_insufficient_fwd_crg;
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        let n = &self.names;
        let o = &self.out;
        for z in 0..ODLS_ZONE_COUNT {
            out(&n.odls_trip[z], on(o.odls_trip[z]));
            out(&n.odls_fault[z], on(o.odls_loop_fault[z]));
            out(&n.odls_loop_a_fault[z], on(o.odls_loop_a_fault[z]));
            out(&n.odls_loop_b_fault[z], on(o.odls_loop_b_fault[z]));
        }
        for z in 0..ZONE_COUNT {
            out(&n.zone_heat_w[z], o.zone_heat_w[z]);
            out(&n.zone_jet_flux[z], o.jet_impact_flux_w_m2[z]);
        }
        for i in 0..4 {
            out(&n.engine_precooler_ovht[i], on(o.engine_precooler_overtemp[i]));
            out(&n.engine_precooler_outlet_c[i], o.engine_precooler_outlet_k[i] - 273.15);
            out(&n.engine_isolation_open[i], on(!o.engine_isolated[i]));
            out(&n.engine_duct_pressure[i], o.engine_duct_pressure_pa[i]);
            out(&n.engine_duct_temp_c[i], o.engine_duct_temp_k[i] - 273.15);
            out(&n.transfer_pipe_pressure[i], o.transfer_pipe_pressure_pa[i]);
            out(&n.hp_valve_open[i], o.hp_valve_open[i]);
            out(&n.pr_valve_open[i], o.pr_valve_open[i]);
            out(&n.start_duct_pressure[i], o.start_duct_pressure_pa[i]);
            out(&n.engine_odls_isolated[i], on(o.engine_isolated[i]));
            out(&n.engine_precooler_fouling[i], self.faults.engine_precooler[i].fouling.clamp(0.0, 1.0));
            out(&n.engine_duct_leak_kg_s[i], o.engine_duct_leak_kg_s[i]);
            out(&n.engine_start_duct_leak_kg_s[i], o.start_leak_kg_s[i]);
        }
        out("DEEP_PNEU_APU_PRECOOLER_OVHT", on(o.apu_precooler_overtemp));
        out("DEEP_PNEU_APU_PRECOOLER_OUTLET_C", o.apu_precooler_outlet_k - 273.15);
        out("DEEP_PNEU_APU_ISOLATION_OPEN", on(!o.apu_isolated));
        out("DEEP_PNEU_APU_DUCT_TEMPERATURE_C", o.apu_duct_temp_k - 273.15);
        out("DEEP_PNEU_APU_BLEED_VALVE_OPEN", o.apu_bleed_valve_open);
        out("PNEU_APU_BLEED_DEMAND_KG_S", o.apu_bleed_demand_kg_s);
        out(&n.apu_duct_leak_kg_s, o.apu_duct_leak_kg_s);
        for i in 0..2 {
            out(&n.pack_supply_pressure[i], o.pack_supply_pressure_pa[i]);
            out(&n.pack_supply_temp_c[i], o.pack_supply_temp_k[i] - 273.15);
            out(&n.wai_duct_pressure[i], o.wai_duct_pressure_pa[i]);
            out(&n.wai_duct_temp_c[i], o.wai_duct_temp_k[i] - 273.15);
            out(&n.wai_valve_open[i], o.wai_valve_open[i]);
            out(&n.hyd_reservoir_pressure[i], o.hyd_reservoir_pressure_pa[i]);
            out(&n.pack_leak_kg_s[i], o.pack_leak_kg_s[i]);
            out(&n.wai_leak_kg_s[i], o.wai_leak_kg_s[i]);
            out(&n.hyd_reservoir_leak_kg_s[i], o.hyd_reservoir_leak_kg_s[i]);
        }
        for i in 0..3 {
            out(&n.cross_bleed_open[i], o.cross_bleed_valve_open[i]);
        }

        for i in 0..2 {
            out(&n.pack_regul_fault[i], self.pack_regul_fault[i]);
            out(&n.ram_air_door_fault[i], self.ram_air_door_fault[i]);
        }
        out(&n.mixer_press_regul_fault, self.mixer_press_regul_fault);
        out(&n.press_man_ctl_fault, self.press_man_ctl_fault);
        out(&n.cabin_air_extract_vlv_fault, self.cabin_air_extract_vlv_fault);
        out(&n.pack_regul_redundancy_fault, on(self.pack_regul_redundancy_fault));
        out(&n.outflw_vlv_ctl_fault_all, on(self.outflw_vlv_ctl_fault_all));
        for i in 0..2 {
            out(&n.pack_acm_outlet_temp_c[i], self.pack_acm_outlet_temp_c[i]);
            for v in 0..2 {
                out(&n.pack_fcv_fault[i][v], self.pack_fcv_fault[i][v]);
            }
        }
        out("DEEP_PNEU_PACK_FLOW_INSUFFICIENT_FWD_CRG", on(self.pack_flow_insufficient_fwd_crg));

        let mut k = 0usize;
        self.each_coupling(&mut |d| {
            if let Some(name) = self.derived_names.get(k) {
                out(name, d.magnitude);
            }
            k += 1;
        });
    }

    fn derived_failures(&self, out: &mut dyn FnMut(DerivedFailure)) {
        self.each_coupling(out);
    }
}

pub fn live_system() -> Box<dyn crate::deep::live::Area> {
    Box::new(PneumaticDuctsLive::new())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    fn cruise_truth() -> Truth {
        Truth {
            dt_s: 1.0,
            environment: crate::deep::integration::weather_truth::EnvironmentTruth {
                sat_c: -40.0,
                leading_edge_c: -20.0,
                ambient_pressure_pa: 24_000.0,
                tas_ms: 240.0,
                precipitation_on_aircraft_ratio: 0.0,
                weather: None,
            },
            altitude_ft: 35_000.0,
            on_ground: false,
            engine_n1_frac: [0.85; 4],
            engine_running: [true; 4],
            engine_ip_port_pressure_pa: [260_000.0; 4],
            engine_ip_port_temp_k: [400.0; 4],
            ac_bus_volts: [115.0; 4],
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
        let mut required: Vec<String> = Vec::new();
        for zone in ZONE_NAMES.iter().take(ODLS_ZONE_COUNT) {
            required.push(format!("DEEP_PNEU_ODLS_{zone}_TRIP"));
            required.push(format!("DEEP_PNEU_ODLS_{zone}_FAULT"));
        }
        for n in 1..=4 {
            required.push(format!("DEEP_PNEU_ENG_{n}_PRECOOLER_OVHT"));
        }
        required.push("DEEP_PNEU_APU_PRECOOLER_OVHT".into());
        required.push("DEEP_PNEU_APU_ISOLATION_OPEN".into());
        for n in 1..=4 {
            required.push(format!("DEEP_PNEU_ENG_{n}_ISOLATION_OPEN"));
        }
        for name in required {
            assert!(map.contains_key(&name), "{name} is read by an ECAM trigger but never published");
        }
    }

    #[test]
    fn a_healthy_network_pressurises_its_ducts_and_trips_nothing() {
        let mut area = live_system();
        run(area.as_mut(), &cruise_truth(), &Faults::default(), 300);
        let map = published(area.as_ref());
        for n in 1..=4 {
            assert!(map[&format!("DEEP_PNEU_ENG_{n}_DUCT_PRESSURE_PA")] > 101_325.0, "engine {n} duct must pressurise, got {}", map[&format!("DEEP_PNEU_ENG_{n}_DUCT_PRESSURE_PA")]);
            assert_eq!(map[&format!("DEEP_PNEU_ENG_{n}_ISOLATION_OPEN")], 1.0);
        }
        assert!(map["DEEP_PNEU_PACK_1_SUPPLY_PRESSURE_PA"] > 101_325.0);
        for zone in ZONE_NAMES.iter().take(ODLS_ZONE_COUNT) {
            assert_eq!(map[&format!("DEEP_PNEU_ODLS_{zone}_TRIP")], 0.0, "{zone} must not trip with nothing wrong");
        }
    }

    #[test]
    fn arming_the_engine_bleed_duct_rupture_sags_the_duct_and_heats_the_pylon() {
        let truth = cruise_truth();
        let mut ruptured = live_system();
        let mut healthy = live_system();
        run(ruptured.as_mut(), &truth, &Faults::from_pairs([(f(36, 2), 1.0)]), 3);
        let early = published(ruptured.as_ref());
        assert!(early["DEEP_PNEU_ZONE_PylonEngine1_HEAT_W"] > 1000.0, "the escaping gas must dump real heat into its own pylon, got {}", early["DEEP_PNEU_ZONE_PylonEngine1_HEAT_W"]);
        assert!(early["DEEP_PNEU_ZONE_PylonEngine1_JET_FLUX_W_M2"] > 0.0, "a full rupture must report an impinging-jet flux");

        run(ruptured.as_mut(), &truth, &Faults::from_pairs([(f(36, 2), 1.0)]), 97);
        run(healthy.as_mut(), &truth, &Faults::default(), 100);
        let bad = published(ruptured.as_ref());
        let good = published(healthy.as_ref());
        assert!(
            bad["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"] < good["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"],
            "a ruptured duct must sag: {} vs {}",
            bad["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"],
            good["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"]
        );
        assert_eq!(good["DEEP_PNEU_ZONE_PylonEngine1_JET_FLUX_W_M2"], 0.0);
    }

    #[test]
    fn arming_the_odls_false_detection_trips_and_latches_that_zones_isolation() {
        let truth = cruise_truth();
        let mut area = live_system();
        let armed = Faults::from_pairs([(f(36, 32), 1.0)]);
        run(area.as_mut(), &truth, &armed, 30);
        let map = published(area.as_ref());
        assert_eq!(map["DEEP_PNEU_ODLS_PylonEngine1_TRIP"], 1.0, "a full-severity false detection must trip a cold zone");
        assert_eq!(map["DEEP_PNEU_ENG_1_ISOLATION_OPEN"], 0.0, "the trip must isolate that engine's bleed");
        assert!(map["DEEP_PNEU_ENG_1_PR_VALVE_OPEN"] < 0.01, "and drive its PR valve shut, got {}", map["DEEP_PNEU_ENG_1_PR_VALVE_OPEN"]);

        run(area.as_mut(), &truth, &Faults::default(), 30);
        let after = published(area.as_ref());
        assert_eq!(after["DEEP_PNEU_ENG_1_ISOLATION_OPEN"], 0.0, "a real ODLS trip needs a reset, not self-clearing");
    }

    #[test]
    fn arming_a_loop_open_circuit_reports_a_detection_fault_without_tripping() {
        let truth = cruise_truth();
        let mut area = live_system();
        let armed = Faults::from_pairs([(f(36, 28), 1.0), (f(36, 30), 1.0)]);
        run(area.as_mut(), &truth, &armed, 30);
        let map = published(area.as_ref());
        assert_eq!(map["DEEP_PNEU_ODLS_PylonEngine1_FAULT"], 1.0);
        assert_eq!(map["DEEP_PNEU_ODLS_PylonEngine1_TRIP"], 0.0, "a loop fault is not a leak");
    }

    #[test]
    fn arming_the_extra_catalogues_precooler_fault_also_fouls_the_core() {
        let mut truth = cruise_truth();
        truth.engine_ip_port_temp_k = [560.0; 4];
        truth.engine_ip_port_pressure_pa = [300_000.0; 4];
        truth.controls.pack_pb_on = [false, false];

        let mut faulted = live_system();
        let mut clean = live_system();
        run(faulted.as_mut(), &truth, &Faults::from_pairs([(36_004, 1.0)]), 60);
        run(clean.as_mut(), &truth, &Faults::default(), 60);
        let hot = published(faulted.as_ref())["DEEP_PNEU_ENG_1_DUCT_TEMPERATURE_C"];
        let cool = published(clean.as_ref())["DEEP_PNEU_ENG_1_DUCT_TEMPERATURE_C"];
        assert!(hot > cool + 3.0, "the extra catalogue's precooler fault must foul the core too: {hot} C vs {cool} C");

        let mut eng2 = live_system();
        run(eng2.as_mut(), &truth, &Faults::from_pairs([(36_005, 1.0)]), 60);
        let eng1_untouched = published(eng2.as_ref())["DEEP_PNEU_ENG_1_DUCT_TEMPERATURE_C"];
        assert!((eng1_untouched - cool).abs() < 3.0, "engine 2's fault id must not foul engine 1's core: {eng1_untouched} C vs clean {cool} C");
    }

    #[test]
    fn arming_precooler_fouling_leaves_the_delivered_bleed_hotter() {
        let mut truth = cruise_truth();
        truth.engine_ip_port_temp_k = [560.0; 4];
        truth.engine_ip_port_pressure_pa = [300_000.0; 4];

        let mut fouled = live_system();
        let mut clean = live_system();
        run(fouled.as_mut(), &truth, &Faults::from_pairs([(f(36, 4), 1.0)]), 60);
        run(clean.as_mut(), &truth, &Faults::default(), 60);
        let hot = published(fouled.as_ref())["DEEP_PNEU_ENG_1_DUCT_TEMPERATURE_C"];
        let cool = published(clean.as_ref())["DEEP_PNEU_ENG_1_DUCT_TEMPERATURE_C"];
        assert!(hot > cool + 3.0, "a fouled core must leave the duct hotter: {hot} C vs {cool} C");
    }

    #[test]
    fn the_apu_feeds_engine_one_through_the_cross_bleed_when_the_engines_are_dead() {
        let mut truth = cruise_truth();
        truth.engine_running = [false; 4];
        truth.engine_n1_frac = [0.0; 4];
        truth.engine_ip_port_pressure_pa = [101_325.0; 4];
        truth.engine_ip_port_temp_k = [288.15; 4];
        truth.on_ground = true;
        truth.environment.sat_c = 15.0;
        truth.environment.ambient_pressure_pa = 101_325.0;
        truth.environment.tas_ms = 0.0;
        truth.apu_running = true;
        truth.apu_bleed_pressure_pa = 320_000.0;
        truth.controls.apu_bleed_pb_on = true;

        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::default(), 300);
        let map = published(area.as_ref());
        assert_eq!(map["DEEP_PNEU_APU_BLEED_VALVE_OPEN"], 1.0);
        assert!(map["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"] > 120_000.0, "the APU must pressurise engine 1's duct, got {}", map["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"]);
        assert!(map["DEEP_PNEU_APU_DUCT_TEMPERATURE_C"] > 15.0, "load-compressor discharge must be hotter than the air it drew in");
    }

    #[test]
    fn the_hp6_branch_now_uses_the_real_hp_port_instead_of_a_fixed_zero() {
        let mut truth = cruise_truth();
        truth.controls.cross_bleed_selector = 0.0;
        truth.controls.pack_pb_on = [false, false];
        truth.engine_ip_port_pressure_pa[0] = 150_000.0;
        truth.engine_ip_port_temp_k[0] = 400.0;
        truth.engine_hp_port_pressure_pa[0] = 500_000.0;
        truth.engine_hp_port_temp_k[0] = 600.0;

        let mut area = live_system();
        let mut peak_hp_open = 0.0_f64;
        for _ in 0..300 {
            area.tick(&truth, &Faults::default());
            peak_hp_open = peak_hp_open.max(published(area.as_ref())["DEEP_PNEU_ENG_1_HP_VALVE_OPEN"]);
        }
        assert!(peak_hp_open > 0.1, "the HP valve must open off the real HP6 port once IP8 alone cannot hold regulation, peak {peak_hp_open}");
    }

    #[test]
    fn the_upstream_stage_follows_the_real_unswitched_ip8_port_not_the_pre_switched_customer_bleed_pair() {
        let mut truth = cruise_truth();
        truth.engine_bleed_pressure_pa = [900_000.0; 4];
        truth.engine_bleed_temp_k = [650.0; 4];
        truth.engine_ip_port_pressure_pa = [260_000.0; 4];
        truth.engine_ip_port_temp_k = [400.0; 4];
        truth.engine_hp_port_pressure_pa = [0.0; 4];
        truth.engine_hp_port_temp_k = [288.15; 4];

        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::default(), 60);
        let map = published(area.as_ref());
        assert!(
            map["DEEP_PNEU_ENG_1_PRECOOLER_OUTLET_C"] < 300.0,
            "must not read the pre-switched customer-bleed pair's HP6-hot temperature when the real IP8 tap is cool and the real HP6 port is unavailable, got {} C",
            map["DEEP_PNEU_ENG_1_PRECOOLER_OUTLET_C"]
        );
        assert_eq!(map["DEEP_PNEU_ENG_1_PRECOOLER_OVHT"], 0.0);
    }

    #[test]
    fn cross_bleed_selector_shut_overrides_the_apu_sole_source_heuristic() {
        let mut truth = cruise_truth();
        truth.engine_running = [false; 4];
        truth.engine_n1_frac = [0.0; 4];
        truth.engine_ip_port_pressure_pa = [101_325.0; 4];
        truth.engine_ip_port_temp_k = [288.15; 4];
        truth.on_ground = true;
        truth.environment.sat_c = 15.0;
        truth.environment.ambient_pressure_pa = 101_325.0;
        truth.environment.tas_ms = 0.0;
        truth.apu_running = true;
        truth.apu_bleed_pressure_pa = 320_000.0;
        truth.controls.apu_bleed_pb_on = true;
        truth.controls.cross_bleed_selector = 0.0;
        truth.controls.pack_pb_on = [false, false];

        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::default(), 300);
        let map = published(area.as_ref());
        assert_eq!(map["DEEP_PNEU_XBLEED_L_OPEN"], 0.0, "SHUT must override even the sole-source AUTO heuristic");
        assert!(map["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"] > 120_000.0, "the APU's own valve into engine 1 is unrelated to the cross-bleed selector, got {}", map["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"]);
        assert!(map["DEEP_PNEU_ENG_2_DUCT_PRESSURE_PA"] < 110_000.0, "with the cross-bleed selector SHUT, engine 2 must not be fed through the left valve, got {}", map["DEEP_PNEU_ENG_2_DUCT_PRESSURE_PA"]);
    }

    #[test]
    fn cross_bleed_selector_open_forces_every_valve_open_with_no_sole_source_condition() {
        let mut truth = cruise_truth();
        truth.controls.cross_bleed_selector = 2.0;
        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::default(), 30);
        let map = published(area.as_ref());
        assert_eq!(map["DEEP_PNEU_XBLEED_L_OPEN"], 1.0);
        assert_eq!(map["DEEP_PNEU_XBLEED_C_OPEN"], 1.0);
        assert_eq!(map["DEEP_PNEU_XBLEED_R_OPEN"], 1.0);
    }

    #[test]
    fn switching_a_pack_pushbutton_off_stops_feeding_that_pack() {
        let truth = cruise_truth();
        let mut off_truth = cruise_truth();
        off_truth.controls.pack_pb_on[0] = false;

        let mut on_area = live_system();
        let mut off_area = live_system();
        run(on_area.as_mut(), &truth, &Faults::default(), 300);
        run(off_area.as_mut(), &off_truth, &Faults::default(), 300);
        let on_pressure = published(on_area.as_ref())["DEEP_PNEU_PACK_1_SUPPLY_PRESSURE_PA"];
        let off_pressure = published(off_area.as_ref())["DEEP_PNEU_PACK_1_SUPPLY_PRESSURE_PA"];
        assert!(on_pressure > off_pressure + 5000.0, "switching pack 1's pushbutton off must stop feeding it: on {on_pressure} vs off {off_pressure}");
    }

    #[test]
    fn a_thermal_areas_own_wing_duct_leak_heats_the_bay_enough_to_trip_this_areas_odls() {
        let mut deep = crate::deep::live::Deep::new().with_area(live_system()).with_area(crate::deep::thermal_zones::live::live_system());
        let leak_id = f_thermal(30, 1);
        let armed = Faults::from_pairs([(leak_id, 1.0)]);
        let truth = Truth {
            dt_s: 1.0,
            engine_running: [true; 4],
            engine_n1_frac: [1.0; 4],
            engine_ip_port_pressure_pa: [970_000.0; 4],
            engine_ip_port_temp_k: [590.0; 4],
            engine_bleed_pressure_pa: [970_000.0; 4],
            engine_bleed_temp_k: [590.0; 4],
            ..Truth::default()
        };
        let mut published = BTreeMap::new();
        for _ in 0..600 {
            deep.tick(truth.clone(), &armed, &mut |name, value| {
                published.insert(name.to_string(), value);
            });
        }
        let bay_k = published["THERMAL_ZONE_WINGLELEFT_TEMPERATURE_C"] + 273.15;
        let threshold_k = crate::deep::pneumatic_ducts::odls::OverheatDetectionLoop::THRESHOLD_WING_FUSELAGE_K;
        assert!(bay_k > threshold_k, "setup: the thermal area's own leak failure must heat the bay past the ODLS threshold ({threshold_k:.1} K), got {bay_k:.1} K");
        assert_eq!(published["DEEP_PNEU_ODLS_WingLeLeft_TRIP"], 1.0, "the real bay heat must now reach this area's own ODLS and trip it");

        assert_eq!(published["DEEP_PNEU_ODLS_WingLeRight_TRIP"], 0.0);
    }

    fn f_thermal(ata: u16, n: u16) -> u64 {
        crate::deep::api::failure_id(RegArea::ThermalZones, ata, n)
    }

    #[test]
    fn this_areas_own_engine_duct_rupture_now_reaches_its_own_odls() {
        let mut truth = cruise_truth();
        truth.on_ground = true;
        truth.environment.ambient_pressure_pa = 101_325.0;
        truth.environment.sat_c = 15.0;
        truth.environment.tas_ms = 0.0;
        truth.engine_n1_frac = [0.05; 4];
        truth.engine_ip_port_pressure_pa = [900_000.0; 4];
        truth.engine_ip_port_temp_k = [560.0; 4];
        truth.engine_hp_port_pressure_pa = [1_200_000.0; 4];
        truth.engine_hp_port_temp_k = [700.0; 4];
        let rupture_id = f(36, 2);

        let mut healthy = live_system();
        run(healthy.as_mut(), &truth, &Faults::default(), 10);
        let healthy_out = published(healthy.as_ref());
        assert_eq!(healthy_out["DEEP_PNEU_ODLS_PylonEngine1_TRIP"], 0.0, "a healthy duct must not trip its own bay");

        let mut ruptured = live_system();
        let rupture_faults = Faults::from_pairs([(rupture_id, 1.0)]);
        run(ruptured.as_mut(), &truth, &rupture_faults, 1);
        let heat_pulse = published(ruptured.as_ref())["DEEP_PNEU_ZONE_PylonEngine1_HEAT_W"];
        assert!(heat_pulse > 0.0, "a rupture must actually deliver heat to its own zone, got {heat_pulse}");

        run(ruptured.as_mut(), &truth, &rupture_faults, 8);
        let ruptured_out = published(ruptured.as_ref());
        assert_eq!(ruptured_out["DEEP_PNEU_ODLS_PylonEngine1_TRIP"], 1.0, "that heat must now reach this area's own ODLS and trip it");
        assert_eq!(ruptured_out["DEEP_PNEU_ODLS_WingLeLeft_TRIP"], 0.0, "a zone this fault cannot reach must stay clear");
    }

    #[test]
    fn apu_bleed_demand_is_published_and_real_once_the_apu_is_bled() {
        let mut truth = Truth { dt_s: 0.2, on_ground: true, apu_running: true, apu_bleed_pressure_pa: 310_000.0, ..Truth::default() };
        truth.controls.apu_bleed_pb_on = true;

        let mut off = live_system();
        run(off.as_mut(), &Truth { dt_s: 0.2, ..Truth::default() }, &Faults::default(), 5);
        assert_eq!(published(off.as_ref())["PNEU_APU_BLEED_DEMAND_KG_S"], 0.0, "no bleed selected, no demand");

        let mut on = live_system();
        run(on.as_mut(), &truth, &Faults::default(), 5);
        assert!(published(on.as_ref())["PNEU_APU_BLEED_DEMAND_KG_S"] > 0.0, "a real APU bleed source must show a real, nonzero demand");
    }

    fn derived(area: &PneumaticDuctsLive) -> BTreeMap<u64, f64> {
        let mut out = BTreeMap::new();
        crate::deep::live::Area::derived_failures(area, &mut |d| {
            out.insert(d.fbw_id, d.magnitude);
        });
        out
    }

    #[test]
    fn the_coupling_table_matches_what_the_area_actually_emits() {
        let area = PneumaticDuctsLive::new();
        let table = coupling_table();
        let mut emitted: Vec<(u64, &'static str)> = Vec::new();
        area.each_coupling(&mut |d| emitted.push((d.fbw_id, d.deep_component)));
        assert_eq!(emitted, table);
        assert_eq!(area.derived_names.len(), table.len());
        assert_eq!(table.len(), 8, "four HP valves and four PR valves -- all FlyByWire models of this area's components");
    }

    #[test]
    fn a_healthy_network_tells_flybywire_nothing_at_all() {
        let mut area = PneumaticDuctsLive::new();
        crate::deep::live::Area::tick(&mut area, &cruise_truth(), &Faults::default());
        assert!(derived(&area).values().all(|&m| m == 0.0), "{:?}", derived(&area));
    }

    #[test]
    fn a_seized_bleed_valve_reaches_flybywires_own_valve_at_the_same_severity_on_its_own_engine_only() {
        let mut area = PneumaticDuctsLive::new();
        crate::deep::live::Area::tick(&mut area, &cruise_truth(), &Faults::from_pairs([(FBW_PR_VALVE[0], 0.4)]));
        let d = derived(&area);
        assert!((d[&FBW_PR_VALVE[0]] - 0.4).abs() < 1e-9, "engine 1 PR valve should cross at 0.4, got {}", d[&FBW_PR_VALVE[0]]);
        for id in &FBW_PR_VALVE[1..] {
            assert_eq!(d[id], 0.0, "engine 1's own seized PR valve must not seize the other engines' PR valves");
        }
        for id in FBW_HP_VALVE {
            assert_eq!(d[&id], 0.0, "the HP valves are a different valve and must be untouched");
        }
        assert!((published(&area)["DEEP_DERIVED_FBW_FAILURE_36012"] - 0.4).abs() < 1e-9, "and it must be visible");

        let mut hp = PneumaticDuctsLive::new();
        crate::deep::live::Area::tick(&mut hp, &cruise_truth(), &Faults::from_pairs([(FBW_HP_VALVE[0], 1.0)]));
        let d = derived(&hp);
        assert_eq!(d[&FBW_HP_VALVE[0]], 1.0);
        for id in &FBW_HP_VALVE[1..] {
            assert_eq!(d[id], 0.0, "engine 1's own seized HP valve must not seize the other engines' HP valves");
        }
        for id in FBW_PR_VALVE {
            assert_eq!(d[&id], 0.0);
        }
    }

    #[test]
    fn a_cold_dark_aircraft_publishes_finite_values_and_a_zero_dt_frame_changes_nothing() {
        let mut area = live_system();
        let truth = Truth::default();
        run(area.as_mut(), &truth, &Faults::default(), 100);
        for (name, value) in published(area.as_ref()) {
            assert!(value.is_finite(), "{name} went non-finite");
        }
        let still = Truth { dt_s: 0.0, ..Truth::default() };
        area.tick(&still, &Faults::default());
        let before = published(area.as_ref());
        area.tick(&still, &Faults::default());
        assert_eq!(before, published(area.as_ref()));
    }

    #[test]
    fn pack_flow_insufficient_fwd_crg_is_a_plain_truth_passthrough() {
        let mut area = live_system();
        area.tick(&Truth::default(), &Faults::default());
        assert_eq!(published(area.as_ref())["DEEP_PNEU_PACK_FLOW_INSUFFICIENT_FWD_CRG"], 0.0);

        area.tick(&Truth { pack_flow_insufficient_fwd_crg: true, ..Truth::default() }, &Faults::default());
        assert_eq!(published(area.as_ref())["DEEP_PNEU_PACK_FLOW_INSUFFICIENT_FWD_CRG"], 1.0);
    }
}

