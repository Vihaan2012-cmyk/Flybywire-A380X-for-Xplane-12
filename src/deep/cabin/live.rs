//! The live cabin: one owned instance of every cabin system this directory
//! models, stepped every frame from [`Truth`] and published under the
//! variable names `registry.rs` names in its ECAM triggers.
//!
//! What it owns is the A380's real cabin fit as this area models it:
//!
//! * three galleys (forward, mid, aft), each with its own commercial bus
//!   feed, oven, chiller and water boiler (`galley.rs`);
//! * the IFE: three seat-power/IFE zones on their own wiring bundles and
//!   two redundant head-end servers (`ife.rs`);
//! * the potable water system -- one 800 l tank, its bleed and compressor
//!   pressurisation paths, three point-of-use heaters, two heated drain
//!   masts and the shower installation (`water.rs`);
//! * three zones of vacuum waste with their level sensors, flush valves
//!   and the vacuum generator (`waste.rs`);
//! * a door's seal, slide bottle, latch sensor and cargo-door actuator
//!   (`doors_slides.rs` -- `registry.rs` registers one of each as the
//!   class);
//! * and the crew-call system (`crew_calls.rs`), which is the one piece
//!   that consumes all of the above: an oven smoking, an IFE zone smoking,
//!   a tank full or a slide flat is a call the flight crew receives.
//!
//! ## What drives it
//!
//! Commercial electrical power comes from `truth.ac_bus_volts` -- the
//! galleys and the IFE are shed together when the aircraft sheds its
//! commercial buses, which is what makes a galley bus fault distinguishable
//! from a galley that is simply unpowered. Water pressurisation follows
//! `truth.engine_bleed_pressure_pa`/`apu_bleed_pressure_pa` against
//! ambient, drain-mast icing follows `truth.environment`'s own OAT and TAS,
//! and the cargo door's actuator follows `truth.hydraulic_pressure_pa`.
//!
//! ## What is not in `Truth` yet
//!
//! `Truth::cabin_pressure_pa`/`cabin_temp_k` now carry the cabin's own
//! environment for real (`plugin.rs`'s own sourcing table: ambient plus
//! FlyByWire's ARINC 429 cabin delta-pressure word, and one representative
//! `A32NX_COND_MAIN_DECK_1_TEMP` zone), so this area reads them directly
//! instead of the interim `CabinCommands` fields that used to stand in for
//! them. `Truth::controls::water_demand_l_s` (galley, lavatory) and
//! `cargo_door_commanded_open` are real now too, and are read from there
//! (see [`CabinLive::tick`]) instead of `CabinCommands`'s own now-removed
//! stand-ins -- without a real service demand the potable-water system
//! never actually draws down or flows, and without a real target the cargo
//! door actuator never moves, so neither system's own failures had
//! anything to act on (`deep::integration::failure_audit`'s sweep found
//! exactly this). What is still missing is anything else the cabin crew or
//! passengers *do* -- toilets flushed, call buttons pressed, passenger
//! doors opened -- which stays collected in [`CabinCommands`], documented
//! one by one.

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

/// The volts at which a 115 V AC commercial bus counts as live. **GENERIC**
/// as a number, but not as an idea: galley and IFE contactors drop out well
/// before the bus reaches zero, and 100 V is comfortably below the 115 V
/// nominal and far above the residual on a dead bus.
const BUS_LIVE_VOLTS: f64 = 100.0;

/// Bleed pressure above ambient at which the potable water tank's
/// pressurisation valve has something to work with, Pa. **GENERIC**: the
/// tank is regulated to 40 psi gauge (`water::TARGET_GAUGE_PA`), so a bleed
/// supply below that gauge pressure cannot reach the target at all, and
/// that is exactly where "bleed available" stops being true.
const BLEED_USABLE_GAUGE_PA: f64 = 40.0 * PSI_TO_PA;

/// Whether the A380's showers are fitted: an airline configuration option
/// (`water::WaterSystem::new`'s own argument). The A380's first-class
/// shower spa is the installation this model exists for, so the live
/// aircraft carries it.
const SHOWERS_FITTED: bool = true;

// ---------------------------------------------------------------------------
// Inputs that `Truth` does not carry yet.
// ---------------------------------------------------------------------------

/// Everything the cabin needs that is not in [`Truth`]: the cabin's own
/// environment, and what the people in it are doing.
#[derive(Clone, Copy, Debug)]
pub struct CabinCommands {
    /// Showers requested this tick. `Truth::controls::water_demand_l_s`
    /// (galley, lavatory) now carries the water draw itself; there is no
    /// real shower-request count in this port, so it stays here.
    pub shower_requests: usize,
    /// A toilet flushed in each zone this tick (edge-triggered, as
    /// `waste::WasteInputs` documents).
    pub flush_commanded: [bool; Zone::COUNT],
    /// The galley equipment the cabin crew has switched on.
    pub oven_commanded: [bool; Zone::COUNT],
    pub chiller_commanded: [bool; Zone::COUNT],
    pub boiler_commanded: [bool; Zone::COUNT],
    /// The point-of-use water heaters.
    pub water_heater_commanded: [bool; N_HEATERS],
    /// The cockpit's potable-water backup compressor switch.
    pub water_compressor_commanded: bool,
    /// The cabin seat-power master switch, which the CAB IFE SMOKE
    /// procedure's own `SEAT POWER ... OFF` line reads back.
    pub seat_power_on: bool,
    /// The modelled passenger door: how far open it is (0..100 percent),
    /// and whether its slide is armed. `Truth::controls::
    /// cargo_door_commanded_open` now carries the cargo-door switch's own
    /// commanded travel (see [`CabinLive::tick`]).
    pub door_open_percent: f64,
    pub slide_armed_commanded: bool,
    /// The cabin's own call buttons.
    pub attendant_call_pressed: [bool; Zone::COUNT],
    pub purser_call_pressed: bool,
    pub emergency_call_pressed: bool,
    pub cockpit_call_pressed: bool,
}

impl Default for CabinCommands {
    /// A cabin on the ground, unpressurised, with the galleys and the
    /// water heaters on (the normal turnaround state), nobody drawing
    /// water and every door shut.
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

// ---------------------------------------------------------------------------
// Failure ids.
// ---------------------------------------------------------------------------

/// Every failure id `registry.rs` assigns, resolved once at construction
/// from the registry itself.
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
}

/// The one failure whose registered `model_field` contains `fragment`,
/// among those registered against `component`.
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
        }
    }
}

// ---------------------------------------------------------------------------
// The live system.
// ---------------------------------------------------------------------------

/// The live A380 cabin.
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
    /// The galley bus feeds that are actually dead, as the galley
    /// controller reports them (see [`CabinLive::galley_bus_fault`]).
    galley_bus_fault: [bool; Zone::COUNT],
    /// This frame's new crew calls, highest priority first.
    new_calls: Vec<CabinEvent>,
    /// This tick's cargo-door target, 0..100 percent, from
    /// `Truth::controls::cargo_door_commanded_open[0]` (the forward cargo
    /// door, this model's one representative cargo-door instance). Stored
    /// because `publish` only ever sees `&self`, never `Truth`.
    cargo_door_commanded_percent: f64,

    /// Inputs `Truth` does not carry; see [`CabinCommands`].
    pub commands: CabinCommands,
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
            commands: CabinCommands::default(),
        }
    }

    /// Turnaround: refill the water tank, empty the waste tanks, repack the
    /// slide.
    pub fn service(&mut self) {
        self.water.service();
        self.waste.service();
        self.door.service();
    }

    /// The highest-priority crew call outstanding, if any.
    pub fn active_call(&self) -> Option<&CabinEvent> {
        self.crew_calls.active_call()
    }

    pub fn acknowledge_all_calls(&mut self) {
        self.crew_calls.acknowledge_all();
    }

    /// A galley bus feed the galley controller reports as faulted: its own
    /// feed is dead (`bus_fault >= 1.0` is exactly the threshold
    /// `galley::step` uses to declare the galley dead) even though the
    /// aircraft's commercial bus behind it is live. A galley that is dark
    /// because the whole aircraft has shed its commercial buses is not a
    /// galley fault, which is why the aircraft bus is part of the test.
    fn galley_bus_fault(commercial_power: bool, faults: &GalleyFaults) -> [bool; Zone::COUNT] {
        std::array::from_fn(|i| commercial_power && faults.bus_fault[i] >= 1.0)
    }
}

/// Whether any 115 V AC commercial bus is live.
fn commercial_power_available(truth: &Truth) -> bool {
    truth.ac_bus_volts.iter().any(|&v| v >= BUS_LIVE_VOLTS)
}

/// Whether bleed air is available to pressurise the potable water tank:
/// any engine's pylon bleed, or the APU's, more than the tank's own
/// regulated gauge pressure above ambient.
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

        // ---- Galleys -----------------------------------------------------
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

        // ---- IFE ---------------------------------------------------------
        let ife_faults = IfeFaults {
            seat_fault: std::array::from_fn(|i| faults.get(self.ids.ife_seat[i])),
            server_fault: std::array::from_fn(|i| faults.get(self.ids.ife_server[i])),
        };
        let ife_inputs = IfeInputs { commercial_power_available: commercial_power, seat_power_on: self.commands.seat_power_on };
        let (ife_out, _ife_events) = self.ife.step(&ife_inputs, &ife_faults, dt);
        self.ife_out = ife_out;

        // ---- Waste -------------------------------------------------------
        let waste_faults = WasteFaults {
            generator_fault: faults.get(self.ids.waste_generator),
            tank_level_sensor_fault: std::array::from_fn(|i| faults.get(self.ids.waste_level_sensor[i])),
            valve_stuck_open: std::array::from_fn(|i| faults.get(self.ids.waste_valve_stuck_open[i])),
            valve_stuck_closed: std::array::from_fn(|i| faults.get(self.ids.waste_valve_stuck_closed[i])),
        };
        let waste_inputs = WasteInputs { cabin_diff_pressure_pa: cabin_diff_pa, flush_commanded: self.commands.flush_commanded };
        self.waste_out = self.waste.step(&waste_inputs, &waste_faults, dt);

        // ---- Potable water -----------------------------------------------
        // The lavatories' rinse water is a real draw on the potable system,
        // so the waste system's own rinse output is added to whatever the
        // cabin is asking for directly.
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
            // Real cabin-service draw (`Truth::controls::water_demand_l_s`,
            // `[galley, lavatory]`) plus the lavatories' own rinse draw:
            // without this, the potable system never actually flowed, so
            // none of its leak/heater/pressurisation failures had anything
            // to act on (`deep::integration::failure_audit`'s sweep).
            galley_demand_l_s: truth.controls.water_demand_l_s[0],
            lav_demand_l_s: truth.controls.water_demand_l_s[1] + rinse_l_s,
            shower_requests: self.commands.shower_requests,
            heater_commanded: self.commands.water_heater_commanded,
        };
        self.water_out = self.water.step(&water_inputs, &water_faults, dt);

        // ---- Door, slide, cargo door -------------------------------------
        let door_faults = DoorSlideFaults {
            seal_leak: faults.get(self.ids.door_seal),
            bottle_leak: faults.get(self.ids.slide_bottle),
            latch_sensor_fault: faults.get(self.ids.latch_sensor),
            actuator_jam: faults.get(self.ids.cargo_jam),
            hydraulic_loss: faults.get(self.ids.cargo_hydraulic),
        };
        // The forward cargo door's switch, `Truth::controls::
        // cargo_door_commanded_open[0]` (`[fwd, aft, bulk]`; this model
        // registers one cargo-door actuator as the representative class,
        // per `registry.rs`'s own note): without a real commanded target
        // the actuator never had anywhere to go, so a jam or a hydraulic
        // loss had no travel to cap (`deep::integration::failure_audit`'s
        // sweep found exactly this).
        self.cargo_door_commanded_percent = (truth.controls.cargo_door_commanded_open[0] * 100.0).clamp(0.0, 100.0);
        let door_inputs = DoorSlideInputs {
            door_open_percent: self.commands.door_open_percent,
            cabin_diff_pressure_pa: cabin_diff_pa,
            slide_armed_commanded: self.commands.slide_armed_commanded,
            // GENERIC circuit assignment: no public source names which of
            // the two A380 hydraulic systems drives the cargo doors, so
            // they are put on the yellow system, the same one
            // `gear_structure` assigns the nose and body legs.
            hydraulic_pressure_pa: truth.hydraulic_pressure_pa[1],
            cargo_door_target_percent: self.cargo_door_commanded_percent,
        };
        self.door_out = self.door.step(&door_inputs, &door_faults, dt);

        // ---- Crew calls ---------------------------------------------------
        let snapshot = CabinSnapshot {
            oven_smoke: self.galley_out.oven_smoke,
            ife_zone_smoke: std::array::from_fn(|i| self.ife_out.zone_temp_c[i] >= ife::SMOKE_TEMP_C),
            waste_tank_full: self.waste_out.tank_full,
            // The one thing the crew is called about that is a *system*
            // fault rather than a single component: the water system
            // cannot deliver, whether because the tank has run dry or
            // because it has lost its pressurisation.
            water_system_fault: self.water_out.tank_empty || self.water_out.flow_fraction < 1.0,
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
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        let b = |x: bool| if x { 1.0 } else { 0.0 };

        // ---- Potable water ------------------------------------------------
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

        // ---- Waste --------------------------------------------------------
        for i in 0..Zone::COUNT {
            let n = i + 1;
            out(&format!("CABIN_WASTE_LEVEL_PERCENT:{n}"), self.waste_out.level_percent[i]);
            out(&format!("CABIN_WASTE_TANK_FULL:{n}"), b(self.waste_out.tank_full[i]));
            out(&format!("CABIN_LAV_INOPERATIVE:{n}"), b(self.waste_out.lav_inoperative[i]));
        }
        out("CABIN_WASTE_GENERATOR_RUNNING", b(self.waste_out.generator_running));

        // ---- IFE ----------------------------------------------------------
        for i in 0..Zone::COUNT {
            let n = i + 1;
            out(&format!("CABIN_IFE_ZONE_SMOKE:{n}"), b(self.ife_out.zone_temp_c[i] >= ife::SMOKE_TEMP_C));
            out(&format!("CABIN_IFE_ZONE_OVERHEAT:{n}"), b(self.ife_out.zone_temp_c[i] >= ife::OVERHEAT_TEMP_C));
            out(&format!("CABIN_IFE_ZONE_TEMP_C:{n}"), self.ife_out.zone_temp_c[i]);
            out(&format!("CABIN_IFE_ZONE_TRIPPED:{n}"), b(self.ife_out.zone_tripped[i]));
        }
        for i in 0..N_SERVERS {
            let n = i + 1;
            out(&format!("CABIN_IFE_SERVER_FAIL:{n}"), b(self.ife_out.server_failed[i]));
        }
        out("CABIN_IFE_CONTENT_AVAILABLE", b(self.ife_out.content_available));
        out("CABIN_SEAT_POWER_CMD", b(self.commands.seat_power_on));

        // ---- Galleys ------------------------------------------------------
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

        // ---- Doors and slides ---------------------------------------------
        out("CABIN_DOOR_LATCHED:1", b(self.door_out.latched_indication));
        out("CABIN_DOOR_SEAL_LEAK_KG_S:1", self.door_out.seal_leak_kg_s);
        out("CABIN_SLIDE_PRESSURE_LOW:1", b(!self.door_out.slide_pressure_adequate));
        out("CABIN_SLIDE_BOTTLE_PA:1", self.door_out.slide_bottle_pressure_pa);
        out("CABIN_SLIDE_DEPLOYED:1", b(self.door_out.slide_deployed));
        out("CABIN_CARGO_DOOR_JAMMED:1", b(self.door_out.cargo_door_jammed));
        out("CABIN_CARGO_DOOR_PERCENT:1", self.door_out.cargo_door_percent);
        out("CABIN_CARGO_DOOR_CMD:1", self.cargo_door_commanded_percent);

        // ---- Crew calls ----------------------------------------------------
        let priority = self.crew_calls.active_call().map_or(0.0, |e| match e.priority() {
            CallPriority::Emergency => 3.0,
            CallPriority::Purser => 2.0,
            CallPriority::Normal => 1.0,
        });
        out("CABIN_CREW_CALL_PRIORITY", priority);
        out("CABIN_CREW_CALL_NEW_COUNT", self.new_calls.len() as f64);
    }
}

/// This area's live system.
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

    /// An aircraft with its commercial buses live and its bleed up: the
    /// state the cabin actually operates in.
    fn powered_truth() -> Truth {
        Truth {
            ac_bus_volts: [115.0; 4],
            dc_bus_volts: [28.0; 2],
            engine_running: [true; 4],
            engine_n1_frac: [0.6; 4],
            // Comfortably above the 40 psi gauge the potable tank
            // regulates to, so the bleed pressurisation path is genuinely
            // available (see `BLEED_USABLE_GAUGE_PA`).
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

    /// `truth.cabin_pressure_pa` now drives the cabin directly
    /// (`CabinCommands` no longer carries it): the door seal leak scales
    /// with `cabin_diff_pressure_pa = cabin_pressure_pa - ambient`
    /// (`doors_slides.rs`'s own `orifice_flow_kg_s`), so a higher
    /// commanded cabin pressure with a failed seal must leak faster --
    /// proof this reads `Truth` every tick rather than a fixed interim
    /// default.
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
        // registry.rs: the stuck thermostat "keeps the element on past
        // setpoint, driving the cavity toward the smoke threshold".
        let mut truth = powered_truth();
        truth.dt_s = 1.0;
        let mut live = CabinLive::new();
        let id = live.ids.galley_oven[1]; // mid galley
        let out = run(&mut live, &truth, &Faults::from_pairs([(id, 1.0)]), 3600.0);
        assert_eq!(out.get("CABIN_GALLEY_OVEN_SMOKE:2"), Some(&1.0));
        assert_eq!(out.get("CABIN_GALLEY_OVEN_SMOKE:1"), Some(&0.0), "the other galleys are untouched");
        assert!(out["CABIN_GALLEY_OVEN_TEMP_C:2"] > out["CABIN_GALLEY_OVEN_TEMP_C:1"]);
        assert!(out["CABIN_CREW_CALL_PRIORITY"] >= 2.0, "a galley fire is a crew call");
    }

    #[test]
    fn a_shorted_ife_zone_heats_to_smoke_then_trips_itself_dead() {
        // registry.rs: "zone wiring heats over minutes, passing through an
        // overheating advisory to a smoke warning, then the zone's own
        // protection trips it dead".
        let mut truth = powered_truth();
        truth.dt_s = 1.0;
        let mut live = CabinLive::new();
        let id = live.ids.ife_seat[2]; // aft zone

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
        // registry.rs: "the redundant server keeps the cabin served until
        // both fail" -- and the IFE SYS FAULT alert is an AND of the two.
        let truth = powered_truth();
        let mut live = CabinLive::new();
        let (a, b) = (live.ids.ife_server[0], live.ids.ife_server[1]);

        let one = run(&mut live, &truth, &Faults::from_pairs([(a, 1.0)]), 1.0);
        assert_eq!(one.get("CABIN_IFE_SERVER_FAIL:1"), Some(&1.0));
        assert_eq!(one.get("CABIN_IFE_SERVER_FAIL:2"), Some(&0.0));
        assert_eq!(one.get("CABIN_IFE_CONTENT_AVAILABLE"), Some(&1.0));

        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        let alert = reg.alerts.iter().find(|x| x.key == "CABIN_IFE_SYS_FAULT").expect("registered");
        assert!(!alert.trigger.eval(&|n: &str| one.get(n).copied().unwrap_or(0.0)), "one dead server is not an IFE system fault");

        let both = run(&mut live, &truth, &Faults::from_pairs([(a, 1.0), (b, 1.0)]), 1.0);
        assert_eq!(both.get("CABIN_IFE_CONTENT_AVAILABLE"), Some(&0.0));
        assert!(alert.trigger.eval(&|n: &str| both.get(n).copied().unwrap_or(0.0)));
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

        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        let alert = reg.alerts.iter().find(|a| a.key == "CABIN_WATER_QTY_LO").expect("registered");
        let drained = run(&mut live, &truth, &Faults::from_pairs([(id, 1.0)]), 36_000.0);
        assert!(alert.trigger.eval(&|n: &str| drained.get(n).copied().unwrap_or(0.0)), "a fully drained tank must raise POTABLE WATER QTY LO");
    }

    #[test]
    fn a_failed_drain_mast_heater_lets_the_mast_ice_up_in_cold_air() {
        // registry.rs: "no heater output, mast can ice at cold OAT".
        let mut truth = powered_truth();
        truth.dt_s = 1.0;
        // Cold enough to freeze an unheated mast, and slow enough that a
        // *healthy* 150 W heater still wins against the convective loss
        // (`water.rs`: the mast's own balance is `oat + P/(h*A)`), so the
        // difference the test sees is the heater failing, not the weather.
        truth.environment.sat_c = -30.0;
        truth.environment.tas_ms = 100.0;
        truth.on_ground = false;
        truth.controls.water_demand_l_s = [0.0, 0.02]; // water actually reaching the masts

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

    /// `deep::integration::failure_audit`'s sweep found the water quantity
    /// sensor fault dead: with no real service demand ever reaching
    /// `water::WaterInputs`, the tank never actually drained, so "frozen at
    /// the last reading" and "tracking the real level" read identically
    /// (both ~100%). `Truth::controls::water_demand_l_s` fixes that; this
    /// proves it end to end, driven purely through `Truth`, not through
    /// `CabinCommands` (which no longer has a demand field at all).
    #[test]
    fn a_stuck_water_quantity_sensor_only_shows_once_real_demand_drains_the_tank() {
        let mut truth = powered_truth();
        truth.controls.water_demand_l_s = [0.1, 0.05]; // a real galley + lavatory draw
        let id = CabinLive::new().ids.water_qty_sensor;

        let healthy = run(&mut CabinLive::new(), &truth, &Faults::default(), 3600.0);
        let stuck = run(&mut CabinLive::new(), &truth, &Faults::from_pairs([(id, 1.0)]), 3600.0);

        assert!(healthy["CABIN_WATER_QTY_PERCENT"] < 99.0, "an hour of real draw must show up as the tank actually emptying: {}", healthy["CABIN_WATER_QTY_PERCENT"]);
        assert!((stuck["CABIN_WATER_QTY_PERCENT"] - 100.0).abs() < 1e-6, "a stuck sensor must freeze at its last (full) reading instead of tracking the real drain");
    }

    #[test]
    fn a_jammed_cargo_door_actuator_caps_its_travel_and_reports_the_fault() {
        // registry.rs: "the cargo door cannot reach a commanded target
        // beyond the jammed travel limit".
        let mut truth = powered_truth();
        truth.dt_s = 0.5;
        truth.controls.cargo_door_commanded_open = [1.0, 0.0, 0.0]; // forward cargo door commanded fully open
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
        // Door shut, sensor healthy: the indication is "latched".
        let shut = run(&mut live, &truth, &Faults::default(), 1.0);
        assert_eq!(shut.get("CABIN_DOOR_LATCHED:1"), Some(&1.0));
        // Freeze the sensor, then open the door: the indication lies.
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

        // Same fault, whole aircraft unpowered: a dark galley is not a
        // galley fault.
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
        // A flush valve stuck open is the registered way a tank fills
        // without anybody flushing.
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
        ];
        consumed.extend(ids.water_heater);
        consumed.extend(ids.mast_heater);
        consumed.extend(ids.waste_level_sensor);
        consumed.extend(ids.waste_valve_stuck_open);
        consumed.extend(ids.waste_valve_stuck_closed);
        consumed.extend(ids.ife_seat);
        consumed.extend(ids.ife_server);
        consumed.extend(ids.galley_bus);
        consumed.extend(ids.galley_oven);
        consumed.extend(ids.galley_chiller);
        consumed.extend(ids.galley_boiler);
        consumed.sort_unstable();
        consumed.dedup();

        let registered: Vec<u64> = reg.failures.iter().map(|f| f.id).collect();
        assert_eq!(consumed.len(), registered.len(), "every cabin failure should be consumed by the live system");
        for id in registered {
            assert!(consumed.contains(&id), "failure {id} is registered but never read by the live system");
        }
        // The ATA chapters this area actually covers.
        for ata in [ATA_WATER, ATA_WASTE, ATA_IFE, ATA_GALLEY, ATA_DOORS] {
            assert!(reg.failures.iter().any(|f| f.ata == ata), "nothing registered under ATA {ata}");
        }
    }
}
