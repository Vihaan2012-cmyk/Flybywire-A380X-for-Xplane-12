use crate::deep::api::Registry;
use crate::deep::live::{Faults, Truth};

use super::crew_calls::{CabinEvent, CabinSnapshot, CallPriority, CrewCallInputs, CrewCallSystem};
use super::doors_slides::{DoorSlide, DoorSlideFaults, DoorSlideInputs, DoorSlideOutputs};
use super::galley::{GalleyFaults, GalleyInputs, GalleyOutputs, GalleySystem};
use super::ife::{self, IfeFaults, IfeInputs, IfeOutputs, IfeSystem, N_SERVERS};
use super::waste::{WasteFaults, WasteInputs, WasteOutputs, WasteSystem};
use super::water::{WaterFaults, WaterInputs, WaterOutputs, WaterSystem, N_DRAIN_MASTS, N_HEATERS, PSI_TO_PA};
use super::Zone;

const ATA_WATER: u16 = 38;
const ATA_WASTE: u16 = 38;
const ATA_IFE: u16 = 44;
const ATA_GALLEY: u16 = 25;
const ATA_DOORS: u16 = 52;

const BUS_LIVE_VOLTS: f64 = 100.0;

const BLEED_USABLE_GAUGE_PA: f64 = 40.0 * PSI_TO_PA;

const SHOWERS_FITTED: bool = true;

const WATER_SYSTEM_FAULT_FLOW_FRACTION: f64 = 0.5;
const WATER_SYSTEM_FAULT_CONFIRM_S: f64 = 5.0;

#[derive(Clone, Copy, Debug)]
pub struct CabinCommands {
    pub shower_requests: usize,
    pub flush_commanded: [bool; Zone::COUNT],
    pub oven_commanded: [bool; Zone::COUNT],
    pub chiller_commanded: [bool; Zone::COUNT],
    pub boiler_commanded: [bool; Zone::COUNT],
    pub water_heater_commanded: [bool; N_HEATERS],
    pub water_compressor_commanded: bool,
    pub seat_power_on: bool,
    pub door_open_percent: f64,
    pub slide_armed_commanded: bool,
    pub attendant_call_pressed: [bool; Zone::COUNT],
    pub purser_call_pressed: bool,
    pub emergency_call_pressed: bool,
    pub cockpit_call_pressed: bool,
}

impl Default for CabinCommands {
    fn default() -> Self {
        Self {
            shower_requests: 0,
            flush_commanded: [false; Zone::COUNT],
            oven_commanded: [true; Zone::COUNT],
            chiller_commanded: [true; Zone::COUNT],
            boiler_commanded: [true; Zone::COUNT],
            water_heater_commanded: [true; N_HEATERS],
            water_compressor_commanded: false,
            seat_power_on: true,
            door_open_percent: 0.0,
            slide_armed_commanded: false,
            attendant_call_pressed: [false; Zone::COUNT],
            purser_call_pressed: false,
            emergency_call_pressed: false,
            cockpit_call_pressed: false,
        }
    }
}

struct Ids {
    water_leak: u64,
    water_bleed_valve: u64,
    water_compressor: u64,
    water_qty_sensor: u64,
    water_heater: [u64; N_HEATERS],
    mast_heater: [u64; N_DRAIN_MASTS],
    waste_generator: u64,
    waste_level_sensor: [u64; Zone::COUNT],
    waste_valve_stuck_open: [u64; Zone::COUNT],
    waste_valve_stuck_closed: [u64; Zone::COUNT],
    ife_seat: [u64; Zone::COUNT],
    ife_smoke_fault: [u64; Zone::COUNT],
    ife_server: [u64; N_SERVERS],
    galley_bus: [u64; Zone::COUNT],
    galley_oven: [u64; Zone::COUNT],
    galley_chiller: [u64; Zone::COUNT],
    galley_boiler: [u64; Zone::COUNT],
    door_seal: u64,
    slide_bottle: u64,
    latch_sensor: u64,
    cargo_jam: u64,
    cargo_hydraulic: u64,
    upper_door_latch: [u64; 6],
    purser_temp_sel_fault: u64,
    ife_bay_isol_fault: u64,
    ife_bay_vent_fault: u64,
    lav_galley_extract_fault: u64,
    secondary_cabin_fan: [u64; 4],
}

fn fid(reg: &Registry, component: &str, fragment: &str) -> u64 {
    let mut found = reg.failures.iter().filter(|f| f.component == component && f.model_field.contains(fragment));
    let first = found.next().unwrap_or_else(|| panic!("no failure on {component} whose model_field contains {fragment:?}"));
    assert!(found.next().is_none(), "more than one failure on {component} matches {fragment:?}");
    first.id
}

impl Ids {
    fn resolve() -> Self {
        let mut reg = Registry::default();
        super::registry::register(&mut reg);
        let zone_suffix = ["fwd", "mid", "aft"];

        let mut water_heater = [0u64; N_HEATERS];
        let mut waste_level_sensor = [0u64; Zone::COUNT];
        let mut waste_valve_stuck_open = [0u64; Zone::COUNT];
        let mut waste_valve_stuck_closed = [0u64; Zone::COUNT];
        let mut ife_seat = [0u64; Zone::COUNT];
        let mut ife_smoke_fault = [0u64; Zone::COUNT];
        let mut galley_bus = [0u64; Zone::COUNT];
        let mut galley_oven = [0u64; Zone::COUNT];
        let mut galley_chiller = [0u64; Zone::COUNT];
        let mut galley_boiler = [0u64; Zone::COUNT];
        for (i, z) in zone_suffix.iter().enumerate() {
            water_heater[i] = fid(&reg, &format!("38_wtr.heater_{z}"), &format!("heater_fault[{i}]"));
            waste_level_sensor[i] = fid(&reg, &format!("38_wst.level_sensor_{z}"), &format!("tank_level_sensor_fault[{i}]"));
            waste_valve_stuck_open[i] = fid(&reg, &format!("38_wst.flush_valve_{z}"), &format!("valve_stuck_open[{i}]"));
            waste_valve_stuck_closed[i] = fid(&reg, &format!("38_wst.flush_valve_{z}"), &format!("valve_stuck_closed[{i}]"));
            ife_seat[i] = fid(&reg, &format!("44_ife.seat_wiring_{z}"), &format!("seat_fault[{i}]"));
            ife_smoke_fault[i] = fid(&reg, &format!("44_ife.seat_wiring_{z}"), &format!("smoke_detector_fault[{i}]"));
            galley_bus[i] = fid(&reg, &format!("25_gal.bus_{z}"), &format!("bus_fault[{i}]"));
            galley_oven[i] = fid(&reg, &format!("25_gal.oven_{z}"), &format!("oven_overheat[{i}]"));
            galley_chiller[i] = fid(&reg, &format!("25_gal.chiller_{z}"), &format!("chiller_fault[{i}]"));
            galley_boiler[i] = fid(&reg, &format!("25_gal.boiler_{z}"), &format!("boiler_fault[{i}]"));
        }

        let mut mast_heater = [0u64; N_DRAIN_MASTS];
        for (i, m) in ["fwd", "aft"].iter().enumerate() {
            mast_heater[i] = fid(&reg, &format!("38_wtr.mast_{m}"), &format!("mast_heater_fault[{i}]"));
        }

        let mut ife_server = [0u64; N_SERVERS];
        for i in 0..N_SERVERS {
            ife_server[i] = fid(&reg, &format!("44_ife.server_{}", i + 1), &format!("server_fault[{i}]"));
        }

        Self {
            water_leak: fid(&reg, "38_wtr.potable_tank", "WaterFaults.leak"),
            water_bleed_valve: fid(&reg, "38_wtr.bleed_valve", "bleed_valve_fault"),
            water_compressor: fid(&reg, "38_wtr.compressor", "compressor_fault"),
            water_qty_sensor: fid(&reg, "38_wtr.qty_sensor", "quantity_sensor_fault"),
            water_heater,
            mast_heater,
            waste_generator: fid(&reg, "38_wst.vacuum_generator", "generator_fault"),
            waste_level_sensor,
            waste_valve_stuck_open,
            waste_valve_stuck_closed,
            ife_seat,
            ife_smoke_fault,
            ife_server,
            galley_bus,
            galley_oven,
            galley_chiller,
            galley_boiler,
            door_seal: fid(&reg, "52_dr.seal", "seal_leak"),
            slide_bottle: fid(&reg, "52_dr.slide_bottle", "bottle_leak"),
            latch_sensor: fid(&reg, "52_dr.latch_sensor", "latch_sensor_fault"),
            cargo_jam: fid(&reg, "52_dr.cargo_actuator", "actuator_jam"),
            cargo_hydraulic: fid(&reg, "52_dr.cargo_actuator", "hydraulic_loss"),
            upper_door_latch: std::array::from_fn(|i| fid(&reg, &format!("52_dr.door_upper_{}_latch_sensor", UPPER_DOOR_POS[i]), "upper_door_latch_fault")),
            purser_temp_sel_fault: fid(&reg, "21_vent.purser_temp_sel_panel", "purser_temp_sel_fault"),
            ife_bay_isol_fault: fid(&reg, "21_vent.ife_bay_ventilation", "ife_bay_isol_fault"),
            ife_bay_vent_fault: fid(&reg, "21_vent.ife_bay_ventilation", "ife_bay_vent_fault"),
            lav_galley_extract_fault: fid(&reg, "21_vent.lav_galley_extract_fan", "lav_galley_extract_fault"),
            secondary_cabin_fan: std::array::from_fn(|i| fid(&reg, &format!("21_vent.secondary_cabin_fan_{}", i + 1), "secondary_cabin_fan_failed")),
        }
    }
}

const UPPER_DOOR_POS: [&str; 6] = ["1L", "1R", "2L", "2R", "3L", "3R"];
const UPPER_DOOR_TRUTH_INDEX: [usize; 6] = [5, 6, 7, 8, 9, 10];

pub struct CabinLive {
    ids: Ids,
    galley: GalleySystem,
    ife: IfeSystem,
    water: WaterSystem,
    waste: WasteSystem,
    door: DoorSlide,
    crew_calls: CrewCallSystem,

    galley_out: GalleyOutputs,
    ife_out: IfeOutputs,
    water_out: WaterOutputs,
    waste_out: WasteOutputs,
    door_out: DoorSlideOutputs,
    galley_bus_fault: [bool; Zone::COUNT],
    new_calls: Vec<CabinEvent>,
    cargo_door_commanded_percent: f64,

    water_system_fault_timer_s: f64,
    water_system_fault_confirmed: bool,

    upper_door_open_percent: [f64; 6],
    upper_door_latch_fault: [f64; 6],

    cargo_door_jam_fraction: f64,
    cargo_door_hydraulic_loss_fraction: f64,

    pub commands: CabinCommands,

    purser_temp_sel_fault: f64,
    ife_bay_isol_fault: f64,
    ife_bay_vent_fault: f64,
    lav_galley_extract_fault: f64,
    secondary_cabin_fan_failed: [f64; 4],
}

impl Default for CabinLive {
    fn default() -> Self {
        Self::new()
    }
}

impl CabinLive {
    pub fn new() -> Self {
        Self {
            ids: Ids::resolve(),
            galley: GalleySystem::new(),
            ife: IfeSystem::new(),
            water: WaterSystem::new(SHOWERS_FITTED),
            waste: WasteSystem::new(),
            door: DoorSlide::new(),
            crew_calls: CrewCallSystem::new(),
            galley_out: GalleyOutputs::default(),
            ife_out: IfeOutputs::default(),
            water_out: WaterOutputs::default(),
            waste_out: WasteOutputs::default(),
            door_out: DoorSlideOutputs::default(),
            galley_bus_fault: [false; Zone::COUNT],
            new_calls: Vec::new(),
            cargo_door_commanded_percent: 0.0,
            water_system_fault_timer_s: 0.0,
            water_system_fault_confirmed: false,
            upper_door_open_percent: [0.0; 6],
            upper_door_latch_fault: [0.0; 6],
            cargo_door_jam_fraction: 0.0,
            cargo_door_hydraulic_loss_fraction: 0.0,
            commands: CabinCommands::default(),
            purser_temp_sel_fault: 0.0,
            ife_bay_isol_fault: 0.0,
            ife_bay_vent_fault: 0.0,
            lav_galley_extract_fault: 0.0,
            secondary_cabin_fan_failed: [0.0; 4],
        }
    }

    pub fn service(&mut self) {
        self.water.service();
        self.waste.service();
        self.door.service();
    }

    pub fn active_call(&self) -> Option<&CabinEvent> {
        self.crew_calls.active_call()
    }

    pub fn acknowledge_all_calls(&mut self) {
        self.crew_calls.acknowledge_all();
    }

    fn galley_bus_fault(commercial_power: bool, faults: &GalleyFaults) -> [bool; Zone::COUNT] {
        std::array::from_fn(|i| commercial_power && faults.bus_fault[i] >= 1.0)
    }
}

fn commercial_power_available(truth: &Truth) -> bool {
    truth.ac_bus_volts.iter().any(|&v| v >= BUS_LIVE_VOLTS)
}

fn bleed_available(truth: &Truth) -> bool {
    let ambient = truth.environment.ambient_pressure_pa;
    let threshold = ambient + BLEED_USABLE_GAUGE_PA;
    truth.engine_bleed_pressure_pa.iter().any(|&p| p >= threshold) || (truth.apu_running && truth.apu_bleed_pressure_pa >= threshold)
}

impl crate::deep::live::Area for CabinLive {
    fn name(&self) -> &'static str {
        "cabin"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        let dt = truth.dt_s.max(0.0);
        let commercial_power = commercial_power_available(truth);
        let cabin_diff_pa = (truth.cabin_pressure_pa - truth.environment.ambient_pressure_pa).max(0.0);

        let galley_faults = GalleyFaults {
            bus_fault: std::array::from_fn(|i| faults.get(self.ids.galley_bus[i])),
            oven_overheat: std::array::from_fn(|i| faults.get(self.ids.galley_oven[i])),
            chiller_fault: std::array::from_fn(|i| faults.get(self.ids.galley_chiller[i])),
            boiler_fault: std::array::from_fn(|i| faults.get(self.ids.galley_boiler[i])),
        };
        let galley_inputs = GalleyInputs {
            commercial_power_available: commercial_power,
            oven_commanded: self.commands.oven_commanded,
            chiller_commanded: self.commands.chiller_commanded,
            boiler_commanded: self.commands.boiler_commanded,
        };
        self.galley_out = self.galley.step(&galley_inputs, &galley_faults, dt);
        self.galley_bus_fault = Self::galley_bus_fault(commercial_power, &galley_faults);

        let ife_faults = IfeFaults {
            seat_fault: std::array::from_fn(|i| faults.get(self.ids.ife_seat[i])),
            server_fault: std::array::from_fn(|i| faults.get(self.ids.ife_server[i])),
            smoke_detector_fault: std::array::from_fn(|i| faults.get(self.ids.ife_smoke_fault[i])),
        };
        let ife_inputs = IfeInputs { commercial_power_available: commercial_power, seat_power_on: self.commands.seat_power_on };
        let (ife_out, _ife_events) = self.ife.step(&ife_inputs, &ife_faults, dt);
        self.ife_out = ife_out;

        let waste_faults = WasteFaults {
            generator_fault: faults.get(self.ids.waste_generator),
            tank_level_sensor_fault: std::array::from_fn(|i| faults.get(self.ids.waste_level_sensor[i])),
            valve_stuck_open: std::array::from_fn(|i| faults.get(self.ids.waste_valve_stuck_open[i])),
            valve_stuck_closed: std::array::from_fn(|i| faults.get(self.ids.waste_valve_stuck_closed[i])),
        };
        let waste_inputs = WasteInputs { cabin_diff_pressure_pa: cabin_diff_pa, flush_commanded: self.commands.flush_commanded };
        self.waste_out = self.waste.step(&waste_inputs, &waste_faults, dt);

        let rinse_l_s = if dt > 0.0 { self.waste_out.rinse_used_l.iter().sum::<f64>() / dt } else { 0.0 };
        let water_faults = WaterFaults {
            leak: faults.get(self.ids.water_leak),
            bleed_valve_fault: faults.get(self.ids.water_bleed_valve),
            compressor_fault: faults.get(self.ids.water_compressor),
            heater_fault: std::array::from_fn(|i| faults.get(self.ids.water_heater[i])),
            mast_heater_fault: std::array::from_fn(|i| faults.get(self.ids.mast_heater[i])),
            quantity_sensor_fault: faults.get(self.ids.water_qty_sensor),
        };
        let water_inputs = WaterInputs {
            bleed_available: bleed_available(truth),
            compressor_commanded: self.commands.water_compressor_commanded,
            cabin_pressure_pa: truth.cabin_pressure_pa,
            cabin_temp_k: truth.cabin_temp_k,
            oat_c: truth.environment.sat_c,
            tas_mps: truth.environment.tas_ms,
            ambient_pressure_pa: truth.environment.ambient_pressure_pa,
            galley_demand_l_s: truth.controls.water_demand_l_s[0],
            lav_demand_l_s: truth.controls.water_demand_l_s[1] + rinse_l_s,
            shower_requests: self.commands.shower_requests,
            heater_commanded: self.commands.water_heater_commanded,
        };
        self.water_out = self.water.step(&water_inputs, &water_faults, dt);

        let water_system_fault_now = self.water_out.tank_empty || self.water_out.flow_fraction < WATER_SYSTEM_FAULT_FLOW_FRACTION;
        if water_system_fault_now {
            self.water_system_fault_timer_s += dt;
        } else {
            self.water_system_fault_timer_s = 0.0;
        }
        self.water_system_fault_confirmed = self.water_system_fault_timer_s >= WATER_SYSTEM_FAULT_CONFIRM_S;

        let door_faults = DoorSlideFaults {
            seal_leak: faults.get(self.ids.door_seal),
            bottle_leak: faults.get(self.ids.slide_bottle),
            latch_sensor_fault: faults.get(self.ids.latch_sensor),
            actuator_jam: faults.get(self.ids.cargo_jam),
            hydraulic_loss: faults.get(self.ids.cargo_hydraulic),
        };
        self.cargo_door_jam_fraction = door_faults.actuator_jam;
        self.cargo_door_hydraulic_loss_fraction = door_faults.hydraulic_loss;
        self.cargo_door_commanded_percent = (truth.controls.cargo_door_commanded_open[0] * 100.0).clamp(0.0, 100.0);

        for i in 0..6 {
            self.upper_door_open_percent[i] = (truth.door_open_fraction[UPPER_DOOR_TRUTH_INDEX[i]] * 100.0).clamp(0.0, 100.0);
            self.upper_door_latch_fault[i] = faults.get(self.ids.upper_door_latch[i]);
        }
        let door_inputs = DoorSlideInputs {
            door_open_percent: self.commands.door_open_percent,
            cabin_diff_pressure_pa: cabin_diff_pa,
            slide_armed_commanded: self.commands.slide_armed_commanded,
            hydraulic_pressure_pa: truth.hydraulic_pressure_pa[1],
            cargo_door_target_percent: self.cargo_door_commanded_percent,
        };
        self.door_out = self.door.step(&door_inputs, &door_faults, dt);

        let snapshot = CabinSnapshot {
            oven_smoke: self.galley_out.oven_smoke,
            ife_zone_smoke: std::array::from_fn(|i| self.ife_out.zone_temp_c[i] >= ife::SMOKE_TEMP_C),
            waste_tank_full: self.waste_out.tank_full,
            water_system_fault: self.water_system_fault_confirmed,
            door_not_latched_disagree: vec![self.door_out.door_not_latched_disagree],
            slide_low_pressure: vec![!self.door_out.slide_pressure_adequate],
        };
        let call_inputs = CrewCallInputs {
            attendant_call_pressed: self.commands.attendant_call_pressed,
            purser_call_pressed: self.commands.purser_call_pressed,
            emergency_call_pressed: self.commands.emergency_call_pressed,
            cockpit_call_pressed: self.commands.cockpit_call_pressed,
        };
        self.new_calls = self.crew_calls.step(&call_inputs, &snapshot, dt);

        self.purser_temp_sel_fault = faults.get(self.ids.purser_temp_sel_fault);
        self.ife_bay_isol_fault = faults.get(self.ids.ife_bay_isol_fault);
        self.ife_bay_vent_fault = faults.get(self.ids.ife_bay_vent_fault);
        self.lav_galley_extract_fault = faults.get(self.ids.lav_galley_extract_fault);
        self.secondary_cabin_fan_failed = std::array::from_fn(|i| faults.get(self.ids.secondary_cabin_fan[i]));
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        let b = |x: bool| if x { 1.0 } else { 0.0 };

        out("CABIN_WATER_QTY_PERCENT", self.water_out.quantity_percent);
        out("CABIN_WATER_PRESS_PSI", self.water_out.gauge_pressure_pa / PSI_TO_PA);
        out("CABIN_WATER_QTY_KG", self.water_out.water_mass_kg);
        out("CABIN_WATER_LEAK_L_S", self.water_out.leak_l_s);
        out("CABIN_WATER_FLOW_FRACTION", self.water_out.flow_fraction);
        for i in 0..N_DRAIN_MASTS {
            let n = i + 1;
            out(&format!("CABIN_MAST_BLOCKED:{n}"), b(self.water_out.mast_blocked[i]));
            out(&format!("CABIN_MAST_ICE_KG:{n}"), self.water_out.mast_ice_kg[i]);
        }
        for i in 0..N_HEATERS {
            let n = i + 1;
            out(&format!("CABIN_WATER_HEATER_TEMP_C:{n}"), self.water_out.heater_temp_c[i]);
        }
        out("CABIN_WATER_COMPRESSOR_CMD", b(self.commands.water_compressor_commanded));

        for i in 0..Zone::COUNT {
            let n = i + 1;
            out(&format!("CABIN_WASTE_LEVEL_PERCENT:{n}"), self.waste_out.level_percent[i]);
            out(&format!("CABIN_WASTE_TANK_FULL:{n}"), b(self.waste_out.tank_full[i]));
            out(&format!("CABIN_LAV_INOPERATIVE:{n}"), b(self.waste_out.lav_inoperative[i]));
        }
        out("CABIN_WASTE_GENERATOR_RUNNING", b(self.waste_out.generator_running));

        for i in 0..Zone::COUNT {
            let n = i + 1;
            out(&format!("CABIN_IFE_ZONE_SMOKE:{n}"), b(self.ife_out.zone_temp_c[i] >= ife::SMOKE_TEMP_C));
            out(&format!("CABIN_IFE_ZONE_OVERHEAT:{n}"), b(self.ife_out.zone_temp_c[i] >= ife::OVERHEAT_TEMP_C));
            out(&format!("CABIN_IFE_ZONE_TEMP_C:{n}"), self.ife_out.zone_temp_c[i]);
            out(&format!("CABIN_IFE_ZONE_TRIPPED:{n}"), b(self.ife_out.zone_tripped[i]));
            out(&format!("CABIN_IFE_ZONE_SMOKE_FAULT:{n}"), b(self.ife_out.zone_smoke_detector_fault[i]));
        }
        for i in 0..N_SERVERS {
            let n = i + 1;
            out(&format!("CABIN_IFE_SERVER_FAIL:{n}"), b(self.ife_out.server_failed[i]));
        }
        out("CABIN_IFE_CONTENT_AVAILABLE", b(self.ife_out.content_available));
        out("CABIN_SEAT_POWER_CMD", b(self.commands.seat_power_on));

        for i in 0..Zone::COUNT {
            let n = i + 1;
            out(&format!("CABIN_GALLEY_OVEN_SMOKE:{n}"), b(self.galley_out.oven_smoke[i]));
            out(&format!("CABIN_GALLEY_OVEN_TEMP_C:{n}"), self.galley_out.oven_temp_c[i]);
            out(&format!("CABIN_GALLEY_BUS_FAULT:{n}"), b(self.galley_bus_fault[i]));
            out(&format!("CABIN_GALLEY_CHILLER_TEMP_C:{n}"), self.galley_out.chiller_temp_c[i]);
            out(&format!("CABIN_GALLEY_CHILLER_WARM:{n}"), b(self.galley_out.chiller_failed_warm[i]));
            out(&format!("CABIN_GALLEY_BOILER_TEMP_C:{n}"), self.galley_out.boiler_temp_c[i]);
        }
        out("CABIN_GALLEY_OVEN_CMD", b(self.commands.oven_commanded.iter().any(|&c| c)));
        out("CABIN_GALLEY_TOTAL_POWER_W", self.galley_out.total_power_w);

        out("CABIN_DOOR_LATCHED:1", b(self.door_out.latched_indication));
        out("CABIN_DOOR_SEAL_LEAK_KG_S:1", self.door_out.seal_leak_kg_s);
        out("CABIN_SLIDE_PRESSURE_LOW:1", b(!self.door_out.slide_pressure_adequate));
        out("CABIN_SLIDE_BOTTLE_PA:1", self.door_out.slide_bottle_pressure_pa);
        out("CABIN_SLIDE_DEPLOYED:1", b(self.door_out.slide_deployed));
        out("CABIN_CARGO_DOOR_JAMMED:1", b(self.door_out.cargo_door_jammed));
        out("CABIN_CARGO_DOOR_PERCENT:1", self.door_out.cargo_door_percent);

        for (i, pos) in UPPER_DOOR_POS.iter().enumerate() {
            out(&format!("CABIN_DOOR_UPPER_{pos}_OPEN_PERCENT"), self.upper_door_open_percent[i]);
            out(&format!("CABIN_DOOR_UPPER_{pos}_LATCH_SENSOR_FAULT"), b(self.upper_door_latch_fault[i] >= 0.5));
        }
        out("CABIN_CARGO_DOOR_CMD:1", self.cargo_door_commanded_percent);

        out("CABIN_CARGO_DOOR_JAM_FRACTION:1", self.cargo_door_jam_fraction);
        out("CABIN_CARGO_DOOR_HYDRAULIC_LOSS_FRACTION:1", self.cargo_door_hydraulic_loss_fraction);

        let priority = self.crew_calls.active_call().map_or(0.0, |e| match e.priority() {
            CallPriority::Emergency => 3.0,
            CallPriority::Purser => 2.0,
            CallPriority::Normal => 1.0,
        });
        out("CABIN_CREW_CALL_PRIORITY", priority);
        out("CABIN_CREW_CALL_NEW_COUNT", self.new_calls.len() as f64);

        out("DEEP_CABIN_PURSER_TEMP_SEL_FAULT", self.purser_temp_sel_fault);
        out("DEEP_CABIN_IFE_BAY_ISOL_FAULT", self.ife_bay_isol_fault);
        out("DEEP_CABIN_IFE_BAY_VENT_FAULT", self.ife_bay_vent_fault);
        out("DEEP_CABIN_LAV_GALLEY_EXTRACT_FAULT", self.lav_galley_extract_fault);
        let failed_count = self.secondary_cabin_fan_failed.iter().filter(|&&m| m >= 0.5).count();
        out("DEEP_CABIN_SECONDARY_FANS_FAILED_COUNT", failed_count as f64);
    }
}

pub fn live_system() -> Box<dyn crate::deep::live::Area> {
    Box::new(CabinLive::new())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::fuel::live::test_support::collect_vars;
    use crate::deep::live::Area as _;
    use std::collections::BTreeMap;

    fn published(area: &dyn crate::deep::live::Area) -> BTreeMap<String, f64> {
        let mut map = BTreeMap::new();
        area.publish(&mut |name, value| {
            map.insert(name.to_string(), value);
        });
        map
    }

    fn powered_truth() -> Truth {
        Truth {
            ac_bus_volts: [115.0; 4],
            dc_bus_volts: [28.0; 2],
            engine_running: [true; 4],
            engine_n1_frac: [0.6; 4],
            engine_bleed_pressure_pa: [500_000.0; 4],
            hydraulic_pressure_pa: [5000.0 * 6894.757; 2],
            ..Truth::default()
        }
    }

    fn run(live: &mut CabinLive, truth: &Truth, faults: &Faults, seconds: f64) -> BTreeMap<String, f64> {
        let steps = (seconds / truth.dt_s).ceil() as usize;
        for _ in 0..steps.max(1) {
            live.tick(truth, faults);
        }
        published(live)
    }

    #[test]
    fn cabin_pressure_is_read_from_truth_not_a_fixed_default() {
        let id = {
            let live = CabinLive::new();
            live.ids.door_seal
        };
        let faults = Faults::from_pairs([(id, 1.0)]);

        let mut low = CabinLive::new();
        let mut truth = powered_truth();
        truth.cabin_pressure_pa = truth.environment.ambient_pressure_pa + 10_000.0;
        let low_dp = run(&mut low, &truth, &faults, 1.0);

        let mut high = CabinLive::new();
        truth.cabin_pressure_pa = truth.environment.ambient_pressure_pa + 50_000.0;
        let high_dp = run(&mut high, &truth, &faults, 1.0);

        assert!(low_dp["CABIN_DOOR_SEAL_LEAK_KG_S:1"] > 0.0);
        assert!(
            high_dp["CABIN_DOOR_SEAL_LEAK_KG_S:1"] > low_dp["CABIN_DOOR_SEAL_LEAK_KG_S:1"],
            "a higher Truth::cabin_pressure_pa must leak faster: {} vs {}",
            low_dp["CABIN_DOOR_SEAL_LEAK_KG_S:1"],
            high_dp["CABIN_DOOR_SEAL_LEAK_KG_S:1"]
        );
    }

    #[test]
    fn a_serviced_cabin_on_a_powered_aircraft_raises_nothing() {
        let mut live = CabinLive::new();
        let out = run(&mut live, &powered_truth(), &Faults::default(), 5.0);
        for n in 1..=3 {
            assert_eq!(out.get(&format!("CABIN_GALLEY_OVEN_SMOKE:{n}")), Some(&0.0));
            assert_eq!(out.get(&format!("CABIN_IFE_ZONE_SMOKE:{n}")), Some(&0.0));
            assert_eq!(out.get(&format!("CABIN_GALLEY_BUS_FAULT:{n}")), Some(&0.0));
        }
        assert_eq!(out.get("CABIN_DOOR_LATCHED:1"), Some(&1.0));
        assert_eq!(out.get("CABIN_SLIDE_PRESSURE_LOW:1"), Some(&0.0));
        assert_eq!(out.get("CABIN_CARGO_DOOR_JAMMED:1"), Some(&0.0));
        assert_eq!(out.get("CABIN_WATER_QTY_PERCENT"), Some(&100.0));
        assert!(out["CABIN_WATER_PRESS_PSI"] > 20.0, "a serviced tank on bleed should be pressurised");
    }

    #[test]
    fn every_variable_this_areas_alerts_trigger_on_is_published_by_this_live_system() {
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        let mut names = Vec::new();
        for alert in &reg.alerts {
            collect_vars(&alert.trigger, &mut names);
        }
        let mut live = CabinLive::new();
        live.tick(&powered_truth(), &Faults::default());
        let out = published(&live);
        for name in names {
            assert!(out.contains_key(&name), "alert trigger reads {name}, which nothing publishes");
        }
    }

    #[test]
    fn a_stuck_oven_thermostat_cooks_its_cavity_to_smoke_and_calls_the_crew() {
        let mut truth = powered_truth();
        truth.dt_s = 1.0;
        let mut live = CabinLive::new();
        let id = live.ids.galley_oven[1];
        let out = run(&mut live, &truth, &Faults::from_pairs([(id, 1.0)]), 3600.0);
        assert_eq!(out.get("CABIN_GALLEY_OVEN_SMOKE:2"), Some(&1.0));
        assert_eq!(out.get("CABIN_GALLEY_OVEN_SMOKE:1"), Some(&0.0), "the other galleys are untouched");
        assert!(out["CABIN_GALLEY_OVEN_TEMP_C:2"] > out["CABIN_GALLEY_OVEN_TEMP_C:1"]);
        assert!(out["CABIN_CREW_CALL_PRIORITY"] >= 2.0, "a galley fire is a crew call");
    }

    #[test]
    fn a_shorted_ife_zone_heats_to_smoke_then_trips_itself_dead() {
        let mut truth = powered_truth();
        truth.dt_s = 1.0;
        let mut live = CabinLive::new();
        let id = live.ids.ife_seat[2];

        let early = run(&mut live, &truth, &Faults::from_pairs([(id, 1.0)]), 300.0);
        assert_eq!(early.get("CABIN_IFE_ZONE_OVERHEAT:3"), Some(&1.0), "overheat is called first, around 274 s");
        assert_eq!(early.get("CABIN_IFE_ZONE_SMOKE:3"), Some(&0.0), "but not smoke yet");

        let late = run(&mut live, &truth, &Faults::from_pairs([(id, 1.0)]), 600.0);
        assert_eq!(late.get("CABIN_IFE_ZONE_SMOKE:3"), Some(&1.0));
        assert_eq!(late.get("CABIN_IFE_ZONE_TRIPPED:3"), Some(&1.0));
        assert_eq!(late.get("CABIN_IFE_ZONE_SMOKE:1"), Some(&0.0));
    }

    #[test]
    fn ife_content_survives_one_server_and_is_lost_only_when_both_fail() {
        let truth = powered_truth();
        let mut live = CabinLive::new();
        let (a, b) = (live.ids.ife_server[0], live.ids.ife_server[1]);

        let one = run(&mut live, &truth, &Faults::from_pairs([(a, 1.0)]), 1.0);
        assert_eq!(one.get("CABIN_IFE_SERVER_FAIL:1"), Some(&1.0));
        assert_eq!(one.get("CABIN_IFE_SERVER_FAIL:2"), Some(&0.0));
        assert_eq!(one.get("CABIN_IFE_CONTENT_AVAILABLE"), Some(&1.0));

        let both = run(&mut live, &truth, &Faults::from_pairs([(a, 1.0), (b, 1.0)]), 1.0);
        assert_eq!(both.get("CABIN_IFE_CONTENT_AVAILABLE"), Some(&0.0));
    }

    #[test]
    fn a_water_tank_leak_drains_the_tank_and_drops_the_indicated_quantity() {
        let mut truth = powered_truth();
        truth.dt_s = 1.0;
        let mut live = CabinLive::new();
        let id = live.ids.water_leak;
        let out = run(&mut live, &truth, &Faults::from_pairs([(id, 1.0)]), 3600.0);
        assert!(out["CABIN_WATER_LEAK_L_S"] > 0.0);
        assert!(out["CABIN_WATER_QTY_PERCENT"] < 100.0, "a leaking tank must actually lose water");

        let drained = run(&mut live, &truth, &Faults::from_pairs([(id, 1.0)]), 36_000.0);
        assert!(drained["CABIN_WATER_QTY_PERCENT"] < 10.0, "a fully drained tank must show a near-empty published quantity (no ECAM alert exists for this cabin-only system)");
    }

    #[test]
    fn normal_water_pressure_cycling_never_raises_a_crew_call() {
        let mut truth = powered_truth();
        truth.dt_s = 1.0;
        truth.controls.water_demand_l_s = [0.05, 0.05];
        let mut live = CabinLive::new();
        let mut total_new_calls = 0.0;
        for t in 0..1800 {
            if t == 900 {
                truth.controls.water_demand_l_s = [5.0, 5.0];
            } else if t == 901 {
                truth.controls.water_demand_l_s = [0.05, 0.05];
            }
            live.tick(&truth, &Faults::default());
            let out = published(&live);
            total_new_calls += out["CABIN_CREW_CALL_NEW_COUNT"];
        }
        assert_eq!(total_new_calls, 0.0, "a healthy water system must never raise a crew call from ordinary pressure cycling");
    }

    #[test]
    fn a_failed_water_pump_with_no_delivery_pressure_raises_a_crew_call() {
        let mut truth = powered_truth();
        truth.dt_s = 1.0;
        truth.controls.water_demand_l_s = [0.1, 0.1];
        let mut live = CabinLive::new();
        let faults = Faults::from_pairs([(live.ids.water_bleed_valve, 1.0), (live.ids.water_compressor, 1.0)]);

        let mut saw_starved_flow = false;
        let mut saw_water_system_fault_call = false;
        for _ in 0..1800 {
            live.tick(&truth, &faults);
            if live.water_out.flow_fraction < WATER_SYSTEM_FAULT_FLOW_FRACTION {
                saw_starved_flow = true;
            }
            if live.new_calls.contains(&CabinEvent::WaterSystemFault) {
                saw_water_system_fault_call = true;
            }
        }
        assert!(saw_starved_flow, "a water system with no air source and ongoing demand must starve its delivery pressure");
        assert!(saw_water_system_fault_call, "a sustained loss of water delivery pressure must raise a crew call");
    }

    #[test]
    fn a_failed_drain_mast_heater_lets_the_mast_ice_up_in_cold_air() {
        let mut truth = powered_truth();
        truth.dt_s = 1.0;
        truth.environment.sat_c = -30.0;
        truth.environment.tas_ms = 100.0;
        truth.on_ground = false;
        truth.controls.water_demand_l_s = [0.0, 0.02];

        let mut live = CabinLive::new();
        let id = live.ids.mast_heater[0];

        let healthy = run(&mut live, &truth, &Faults::default(), 3600.0);
        assert_eq!(healthy.get("CABIN_MAST_BLOCKED:1"), Some(&0.0), "a heated mast does not ice");

        let mut live = CabinLive::new();
        let failed = run(&mut live, &truth, &Faults::from_pairs([(id, 1.0)]), 3600.0);
        assert!(failed["CABIN_MAST_ICE_KG:1"] > healthy["CABIN_MAST_ICE_KG:1"], "an unheated mast must accrete ice");
        assert_eq!(failed.get("CABIN_MAST_BLOCKED:1"), Some(&1.0));
        assert_eq!(failed.get("CABIN_MAST_BLOCKED:2"), Some(&0.0), "the other mast is still heated");
    }

    #[test]
    fn a_stuck_water_quantity_sensor_only_shows_once_real_demand_drains_the_tank() {
        let mut truth = powered_truth();
        truth.controls.water_demand_l_s = [0.1, 0.05];
        let id = CabinLive::new().ids.water_qty_sensor;

        let healthy = run(&mut CabinLive::new(), &truth, &Faults::default(), 3600.0);
        let stuck = run(&mut CabinLive::new(), &truth, &Faults::from_pairs([(id, 1.0)]), 3600.0);

        assert!(healthy["CABIN_WATER_QTY_PERCENT"] < 99.0, "an hour of real draw must show up as the tank actually emptying: {}", healthy["CABIN_WATER_QTY_PERCENT"]);
        assert!((stuck["CABIN_WATER_QTY_PERCENT"] - 100.0).abs() < 1e-6, "a stuck sensor must freeze at its last (full) reading instead of tracking the real drain");
    }

    #[test]
    fn a_jammed_cargo_door_actuator_caps_its_travel_and_reports_the_fault() {
        let mut truth = powered_truth();
        truth.dt_s = 0.5;
        truth.controls.cargo_door_commanded_open = [1.0, 0.0, 0.0];
        let mut live = CabinLive::new();
        let id = live.ids.cargo_jam;

        let healthy = run(&mut live, &truth, &Faults::default(), 120.0);
        assert!(healthy["CABIN_CARGO_DOOR_PERCENT:1"] > 99.0, "a healthy actuator opens the door fully");
        assert_eq!(healthy.get("CABIN_CARGO_DOOR_JAMMED:1"), Some(&0.0));

        let mut live = CabinLive::new();
        let jammed = run(&mut live, &truth, &Faults::from_pairs([(id, 0.6)]), 120.0);
        assert!(jammed["CABIN_CARGO_DOOR_PERCENT:1"] < 45.0, "a 60% jam caps travel at 40%: {}", jammed["CABIN_CARGO_DOOR_PERCENT:1"]);
        assert_eq!(jammed.get("CABIN_CARGO_DOOR_JAMMED:1"), Some(&1.0));
    }

    #[test]
    fn a_leaking_slide_bottle_eventually_cannot_inflate_the_slide() {
        let mut truth = powered_truth();
        truth.dt_s = 10.0;
        let mut live = CabinLive::new();
        let id = live.ids.slide_bottle;
        let out = run(&mut live, &truth, &Faults::from_pairs([(id, 1.0)]), 100_000.0);
        assert_eq!(out.get("CABIN_SLIDE_PRESSURE_LOW:1"), Some(&1.0));
        assert!(out["CABIN_SLIDE_BOTTLE_PA:1"] < 3000.0 * 6894.757);
    }

    #[test]
    fn a_stuck_latch_sensor_freezes_the_indication_away_from_the_door() {
        let truth = powered_truth();
        let mut live = CabinLive::new();
        let id = live.ids.latch_sensor;
        let shut = run(&mut live, &truth, &Faults::default(), 1.0);
        assert_eq!(shut.get("CABIN_DOOR_LATCHED:1"), Some(&1.0));
        live.commands.door_open_percent = 50.0;
        let open = run(&mut live, &truth, &Faults::from_pairs([(id, 1.0)]), 1.0);
        assert_eq!(open.get("CABIN_DOOR_LATCHED:1"), Some(&1.0), "a frozen sensor still reads latched on an open door");
    }

    #[test]
    fn a_galley_bus_fault_only_reports_while_the_aircraft_bus_behind_it_is_live() {
        let mut live = CabinLive::new();
        let id = live.ids.galley_bus[0];
        let faults = Faults::from_pairs([(id, 1.0)]);

        let powered = run(&mut live, &powered_truth(), &faults, 1.0);
        assert_eq!(powered.get("CABIN_GALLEY_BUS_FAULT:1"), Some(&1.0));

        let mut live = CabinLive::new();
        let dark = run(&mut live, &Truth::default(), &faults, 1.0);
        assert_eq!(dark.get("CABIN_GALLEY_BUS_FAULT:1"), Some(&0.0));
    }

    #[test]
    fn a_full_waste_tank_takes_its_zones_lavatories_out_of_service() {
        let mut truth = powered_truth();
        truth.dt_s = 1.0;
        truth.on_ground = false;
        truth.cabin_pressure_pa = 80_000.0;
        let mut live = CabinLive::new();
        let id = live.ids.waste_valve_stuck_open[1];
        let out = run(&mut live, &truth, &Faults::from_pairs([(id, 1.0)]), 200_000.0);
        assert!(out["CABIN_WASTE_LEVEL_PERCENT:2"] > 0.0);
        assert_eq!(out.get("CABIN_WASTE_LEVEL_PERCENT:1"), Some(&0.0), "the other zones stay empty");
    }

    #[test]
    fn nothing_divides_by_zero_on_a_cold_cabin_at_zero_dt() {
        let mut live = CabinLive::new();
        live.tick(&Truth { dt_s: 0.0, ..Truth::default() }, &Faults::default());
        for (name, value) in published(&live) {
            assert!(value.is_finite(), "{name} is not finite");
        }
    }

    #[test]
    fn every_registered_failure_is_either_consumed_or_listed_as_not() {
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        let ids = Ids::resolve();
        let mut consumed = vec![
            ids.water_leak,
            ids.water_bleed_valve,
            ids.water_compressor,
            ids.water_qty_sensor,
            ids.waste_generator,
            ids.door_seal,
            ids.slide_bottle,
            ids.latch_sensor,
            ids.cargo_jam,
            ids.cargo_hydraulic,
            ids.purser_temp_sel_fault,
            ids.ife_bay_isol_fault,
            ids.ife_bay_vent_fault,
            ids.lav_galley_extract_fault,
        ];
        consumed.extend(ids.secondary_cabin_fan);
        consumed.extend(ids.water_heater);
        consumed.extend(ids.mast_heater);
        consumed.extend(ids.waste_level_sensor);
        consumed.extend(ids.waste_valve_stuck_open);
        consumed.extend(ids.waste_valve_stuck_closed);
        consumed.extend(ids.ife_seat);
        consumed.extend(ids.ife_smoke_fault);
        consumed.extend(ids.ife_server);
        consumed.extend(ids.galley_bus);
        consumed.extend(ids.galley_oven);
        consumed.extend(ids.galley_chiller);
        consumed.extend(ids.galley_boiler);
        consumed.extend(ids.upper_door_latch);
        consumed.sort_unstable();
        consumed.dedup();

        let registered: Vec<u64> = reg.failures.iter().map(|f| f.id).collect();
        assert_eq!(consumed.len(), registered.len(), "every cabin failure should be consumed by the live system");
        for id in registered {
            assert!(consumed.contains(&id), "failure {id} is registered but never read by the live system");
        }
        for ata in [ATA_WATER, ATA_WASTE, ATA_IFE, ATA_GALLEY, ATA_DOORS] {
            assert!(reg.failures.iter().any(|f| f.ata == ata), "nothing registered under ATA {ata}");
        }
    }
}
