//! The live fuel system: one owned instance of everything this directory
//! models, stepped every frame from [`Truth`] and published under the
//! variable names `registry.rs` already names in its ECAM triggers.
//!
//! What it owns, in the order fuel actually moves through it:
//!
//! * the eleven real tanks (`geometry::ALL_TANKS`), each with its own box
//!   shape, its own fuel mass and its own bulk temperature;
//! * each tank's FQMS capacitance-probe array (sized by
//!   `gauging::probe_count_for_capacity_gal`) and its densitometer;
//! * the sixteen named transfer valves and the two trim pumps
//!   (`cg_transfer::TransferFaults`), with the three shortfall detectors
//!   the real FQMS runs over them (trim, auto-CG, wing cross-feed);
//! * the two jettison nozzle valves and their nozzles, flowing through
//!   [`jettison::jettison_mass_flow_kg_s`];
//! * the four engine feed filters and fuel-cooled oil coolers
//!   (`thermal.rs`), and the leak detector (`leak::LeakDetector`) that
//!   watches indicated quantity against metered burn.
//!
//! ## What drives it
//!
//! Tank bulk temperature is a real energy balance, not a table: each tank
//! exchanges heat with the wing skin (X-Plane's own kinetically-heated
//! `leading_edge_c`, via `truth.environment`) through a conductance set by
//! the tank's own box geometry, and receives whatever the engine's
//! fuel-cooled oil cooler rejects into it, attenuated by that FCOC's
//! fouling. `thermal::fcoc_temperature_rise_k` does the `Q*dt/(m*cp)`
//! arithmetic for both terms, so the tank cools toward the skin with a time
//! constant that falls out of its own fuel mass rather than being chosen.
//!
//! ## What drives the burn
//!
//! `Truth::engine_fuel_flow_kg_s` is `physics::engine`'s own real fuel flow
//! into each combustor -- not a fan-speed guess -- and is what drains each
//! feed tank now. Every tank starts loaded, too: [`FuelLive::new`] seeds a
//! realistic full long-haul dispatch load across all eleven tanks (see
//! [`FuelLive::seed_default_fuel_load`]) rather than
//! `crate::fuel::DEFAULT_GALLONS`'s ramp/cold-start defaults, which fill the
//! four feed tanks only -- real fuel for "what is in the feed tanks for
//! engine start", not for "what does a fuelled long-haul departure look
//! like", and a seed that left the other seven tanks permanently empty, so
//! their own wall-leak and trim-pump failures could never move a kilogram.
//!
//! ## What is not in `Truth` yet
//!
//! A handful of inputs this system genuinely needs still have no field in
//! [`Truth`]: APU fuel burn, and the aircraft's pitch/bank and sustained
//! accelerations. They are collected in [`FuelCommands`] as one explicit,
//! documented block rather than invented from what is available. The
//! defaults are an aircraft that is not burning APU fuel and is wings
//! level -- a real state, not a placeholder quantity. Jettison and
//! cross-feed selection are real cockpit controls now
//! (`Truth::controls::jettison_armed`/`jettison_valve_selected`/
//! `crossfeed_valve_selected`) and are read from there instead.

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

/// Jet A-1 density at 15 C, kg/m^3. This directory used to restate its own
/// figure here (804.0, "the standard reference figure used for the
/// volume-to-mass conversion in flight planning") independently of
/// `src/fuel.rs:164`'s `JET_A_LBS_PER_GAL = 6.699` (itself MSFS/FlyByWire's
/// own Jet A weight constant) and `src/fuel_network.rs:124`'s
/// `JET_A_LBS_PER_GAL = 6.7` (that module's own SDK-default fallback for an
/// unrecognised/generic `fuel_type`, overridden with the real 6.699 figure
/// at construction, `fuel.rs:570`) -- three numbers about 0.15% apart, ASTM
/// D1655/DEF STAN 91-091 allow 775..840 kg/m^3 so none of the three was
/// wrong, but none was cited precisely enough to prefer either. Rather than
/// pick a fourth, this now calls the one already-derived, precisely cited
/// constant the rest of the crate uses for the same real aircraft's fuel:
/// `physics::fluids::JET_A_DENSITY_KG_M3_AT_15C` = 802.5 kg/m^3, converted
/// directly from `fuel.rs`'s own MSFS-sourced 6.699 lb/US gal (that
/// module's own doc comment shows the conversion). `fuel_network.rs`'s 6.7
/// is left alone: it is the SDK's own generic default for an unspecified
/// MSFS `fuel_type`, not a second claim about this aircraft's real fuel,
/// and it is already overridden by the same 6.699 figure wherever this
/// crate actually flies the A380X.
use crate::physics::fluids::JET_A_DENSITY_KG_M3_AT_15C as REFERENCE_DENSITY_15C_KG_M3;
/// Specific heat of kerosene, J/(kg*K) (standard value for Jet A/A-1 near
/// ambient; the same 2010 figure `engine_accessories::fuel::common` cites
/// independently for the same fluid).
const FUEL_CP_J_KGK: f64 = 2010.0;
/// Overall heat-transfer coefficient between the fuel and the wing skin,
/// W/(m^2*K). **GENERIC**: derived as an order of magnitude, not measured --
/// an aluminium tank skin is a negligible thermal resistance, so the number
/// is set by the fuel-side natural/forced convection in a slowly stirred
/// tank, which is the 20..100 W/(m^2*K) class for a hydrocarbon liquid
/// against a metal wall. 40 puts a full inner tank's time constant
/// (`m*cp/(U*A)`) at several hours, which is the right order for the
/// cold-soak rate a long-haul fuel-temperature trend actually shows.
const TANK_SKIN_U_W_M2K: f64 = 40.0;

/// Maximum leak orifice area a tank-wall failure at magnitude 1.0 opens,
/// m^2. **GENERIC** (`leak::leak_area_m2`'s own doc asks the caller to pick
/// one per site): 1 cm^2 is a large stress-crack/impact puncture in a tank
/// skin, big enough to empty a wing tank within a sector and small enough
/// that it is not a structural failure in its own right.
const MAX_TANK_WALL_LEAK_AREA_M2: f64 = 1.0e-4;
/// Maximum leak orifice area a transfer-gallery failure at magnitude 1.0
/// opens, m^2. **GENERIC**, same reasoning, one order smaller: a gallery
/// leak is a line/coupling leak, not a hole in a structural skin.
const MAX_GALLERY_LEAK_AREA_M2: f64 = 1.0e-5;

/// Rolling window and discrepancy threshold for `leak::LeakDetector`.
/// **GENERIC** -- the module's own doc is explicit that no public numeric
/// FUEL LEAK window or threshold exists for this type, and that only the
/// shape of the algorithm is the documented real principle.
///
/// Fuel leak is the one failure whose entire value is early annunciation, so
/// these three constants were sized against the *smallest* leak the
/// catalogue actually arms rather than picked independently of it: a
/// full-severity tank-wall leak (`MAX_TANK_WALL_LEAK_AREA_M2` = 1 cm^2) out
/// of a feed tank at this module's own seeded 95%-full load
/// (`FULL_LOAD_FRACTION`, ~1.8 m of head) is
/// `1.0e-4 * 804 * sqrt(2*9.80665*1.8) = 0.48 kg/s`
/// (`leak::tank_wall_leak_kg_s`'s own orifice law). Over one 35 s window that
/// is 16.8 kg of unmetered loss -- comfortably clear of a 10 kg threshold
/// (this model runs the FQMS with no gauging noise of its own once no probe
/// fault is armed, so 10 kg is not fighting a noise floor, only guarding
/// against a single-tick rounding artefact) -- and three confirmed windows
/// is 105 s, inside the 120 s an early-annunciation failure has to clear to
/// be worth anything. The previous constants (60 s / 100 kg / 3 windows,
/// 180 s minimum and a required rate of 1.7 kg/s) demanded more than three
/// times the leak the catalogue's own maximum orifice can produce, over
/// three times the window: `deep::integration::failure_audit`'s sweep found
/// exactly that -- all four live feed-tank leaks measured as dead because no
/// profile ran long enough at a high enough rate to confirm even once.
const LEAK_WINDOW_S: f64 = 35.0;
const LEAK_THRESHOLD_KG: f64 = 10.0;
const LEAK_CONFIRM_WINDOWS: u32 = 3;

/// Fraction of each tank's own structural capacity [`FuelLive::
/// seed_default_fuel_load`] loads it to. **GENERIC**: no published "standard
/// full load" order exists for an A380 dispatch; 0.95 leaves the same few
/// percent of thermal-expansion ullage a real fuelling procedure keeps
/// (CS 25.969), applied uniformly across all eleven tanks rather than
/// guessing a route-specific loading schedule this model has no basis to
/// pick.
const FULL_LOAD_FRACTION: f64 = 0.95;

// ---------------------------------------------------------------------------
// Real transfer rates: the nominal (fully healthy) mass flow each transfer
// path can deliver, kg/s -- what both the shortfall detectors compare
// against *and* what `tick_transfers` actually moves between tanks now
// (previously a placeholder `1.0` that only ever cancelled out of the
// detectors' own ratio test; see git history). `achieved_transfer_rate_kg_s`
// derates this by the path's own valve/pump/gallery health, so a failed
// pump or a stuck valve genuinely delivers less real mass, not just a lower
// number in a fault-detection ratio.
//
// The figures are FlyByWire's own `flight_model.cfg` `[FUEL_SYSTEM]`
// numbers, read the same way `src/fuel_network.rs` already parses them, and
// combined with that module's own documented (if undocumented-by-Asobo)
// flow formula -- reused by reference, not by owning a second live
// simulation of the network (see `tick_transfers`'s own doc for why not
// instantiating a second `FuelNetwork` here is the honest call).
//
// SDK-documented baseline (`fuel_network.rs`'s own module doc, "Flow"): a
// line carries at most `FuelFlowAt1PSI` (lb/s per psi) times its pressure.
// `fuel_network.rs`'s own doc ("Undocumented MSFS behaviour" #1) and its
// `intersection_pump_degradation_and_stuck_crossfeed_valve_starve_engine`
// test both show the *actually used* mass flow is that baseline multiplied
// by `DEFAULT_LINE_FLOW_GAIN` (60): FBW's own PR #9045 tuned the A380X's
// transfer lines to beat cruise fuel burn, well past the raw SDK unit.
// `nominal_transfer_rate_kg_s` below is exactly that: `FuelFlowAt1PSI *
// gain * pressure_psi`, in lb/s, converted to kg/s -- no separate gal/h or
// density round-trip needed, since `FuelFlowAt1PSI` is already a mass-flow
// constant (lb/s per psi).
const LINE_FLOW_GAIN: f64 = crate::fuel_network::DEFAULT_LINE_FLOW_GAIN;
const LB_TO_KG: f64 = 0.45359237;

/// The single delivery line into a feed tank's own gallery inlet --
/// `flight_model.cfg` `Line.75`/`Line.80`
/// (`FeedTank1FwdXferValve1ToFeedTank1`/`FeedTank4FwdXferValve1ToFeedTank4`),
/// `FuelFlowAt1PSI:0.00175`. Feed2/Feed3 have two such lines each (double
/// the capacity); this single-line figure is used uniformly here as the
/// bottleneck every transfer path shares on its way into *a* feed tank --
/// conservative for Feed2/Feed3, exact for Feed1/Feed4.
const FEED_LINE_FUEL_FLOW_AT_1PSI: f64 = 0.00175;

/// Trim pump rated pressure, psi -- `flight_model.cfg` `Pump.19`/`Pump.20`
/// (`TrimTankPumpLeft`/`TrimTankPumpRight`), `Pressure:53.28`.
const TRIM_PUMP_PRESSURE_PSI: f64 = 53.28;
/// Inner/mid wing-tank transfer pump rated pressure, psi -- `Pump.10`,
/// `Pump.11`, `Pump.12`, `Pump.13`, `Pump.15`, `Pump.16`, `Pump.17`,
/// `Pump.18` (`Left`/`RightMidTankPump{Fwd,Aft}`,
/// `Left`/`RightInnerTankPump{Fwd,Aft}`), all `Pressure:36.8`. Used as the
/// CG-transfer and cross-feed paths' own representative driving pressure:
/// the outer tanks' own pumps are weaker (`Pump.9`/`Pump.14`, `14.72` psi),
/// but outer-tank transfer is the last, shortest stage of the sequence
/// `outer_tank_retention_active` gates, and cross-feed draws on the same
/// gallery network the inner/mid pumps feed.
const WING_TRANSFER_PUMP_PRESSURE_PSI: f64 = 36.8;

/// `FuelFlowAt1PSI * gain * pressure_psi`, in lb/s, to kg/s: see this
/// section's own doc.
fn nominal_transfer_rate_kg_s(pressure_psi: f64) -> f64 {
    (FEED_LINE_FUEL_FLOW_AT_1PSI * LINE_FLOW_GAIN * pressure_psi.max(0.0) * LB_TO_KG).max(0.0)
}
/// Shortfall tolerance and confirm time for the three detectors, matching
/// the `confirm(..)` each corresponding `EcamAlert` in `registry.rs`
/// already carries (1 s trim, 5 s auto-CG, 2 s cross-feed), so the detector
/// and the alert cannot drift apart. The 10% tolerance is **GENERIC**.
const TRANSFER_TOLERANCE: f64 = 0.10;

/// Time a jettison valve takes to travel between shut and fully open, s --
/// `fuel_network.rs`'s own `DEFAULT_VALVE_OPENING_TIME_S`, which
/// `jettison::JettisonValve`'s doc already names as the convention its
/// `opening_time_s` argument follows.
const JETTISON_VALVE_TRAVEL_S: f64 = 5.0;

// ---------------------------------------------------------------------------
// Inputs that `Truth` does not carry yet.
// ---------------------------------------------------------------------------

/// Everything this area needs that is not in [`Truth`].
///
/// Each field is a real quantity the plugin already has somewhere; none is
/// derivable from `Truth` as it stands today, and none is guessed here.
/// The plugin sets them before `tick`; until it does, the defaults describe
/// an aircraft that has not been fuelled, is not burning fuel, is wings
/// level and has neither jettison nor cross-feed selected.
#[derive(Clone, Copy, Debug, Default)]
pub struct FuelCommands {
    /// APU fuel burn, kg/s -- the leak detector's balance is over every
    /// metered consumer, not just the engines (`leak::LeakDetector::update`).
    pub apu_fuel_flow_kg_s: f64,
    /// Body attitude and sustained accelerations, for
    /// `geometry::tilt_fraction` (and through it the FQMS's tilt error and
    /// the unporting margin).
    pub pitch_deg: f64,
    pub bank_deg: f64,
    pub lateral_accel_g: f64,
    pub longitudinal_accel_g: f64,
}

// ---------------------------------------------------------------------------
// Failure ids.
// ---------------------------------------------------------------------------

/// Every failure id this area's `registry.rs` assigns, resolved once at
/// construction by asking the registry itself rather than by restating its
/// numbering here.
///
/// `registry::register` numbers its failures with a counter local to the
/// call, so registering into a throw-away `Registry` yields exactly the ids
/// the plugin's own `deep::registry()` hands out in `Faults`. Looking them
/// up this way means a failure renumbered in `registry.rs` cannot silently
/// stop being consumed -- the lookup fails loudly instead (see
/// `every_registered_failure_is_either_consumed_or_listed_as_not`).
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
}

/// The one failure on `component` whose registered `model_field` contains
/// `field_fragment`. Panics if there is not exactly one: a live system that
/// cannot find the failure it is meant to consume is a build error, not a
/// runtime condition to tolerate.
fn fid(reg: &Registry, component: &str, field_fragment: &str) -> u64 {
    let mut found = reg.failures.iter().filter(|f| f.component == component && f.model_field.contains(field_fragment));
    let first = found.next().unwrap_or_else(|| panic!("no failure on {component} whose model_field contains {field_fragment:?}"));
    assert!(found.next().is_none(), "more than one failure on {component} matches {field_fragment:?}");
    first.id
}

/// The failures registered against `component`, in registration order.
fn fids(reg: &Registry, component: &str) -> Vec<u64> {
    reg.failures.iter().filter(|f| f.component == component).map(|f| f.id).collect()
}

/// `registry.rs`'s own tank id suffixes, in `ALL_TANKS` order.
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
        }
    }
}

// ---------------------------------------------------------------------------
// Per-tank state.
// ---------------------------------------------------------------------------

/// One tank as the live system carries it: its shape, how much fuel is in
/// it, how warm that fuel is, and its own probe array.
#[derive(Clone, Debug)]
struct TankState {
    shape: TankShape,
    mass_kg: f64,
    temp_c: f64,
    probes: Vec<ProbeFault>,
    /// Last FQMS output for this tank: indicated fill fraction and the
    /// surviving fraction of the probe array.
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

    /// Density of this tank's fuel at its current bulk temperature: the one
    /// canonical `physics::fluids::jet_a_density_kg_m3` the rest of the
    /// crate uses for this same real aircraft's fuel (see
    /// `REFERENCE_DENSITY_15C_KG_M3`'s own doc), not a second, locally
    /// restated copy of its expansion formula.
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

    /// Depth of the liquid column above the tank floor, m -- the head that
    /// drives both a wall leak and the jettison nozzle.
    fn liquid_depth_m(&self) -> f64 {
        self.fill_fraction() * self.shape.box_height_m()
    }

    /// Wetted wall area of the box approximation, m^2: the floor plus the
    /// two long side walls up to the liquid surface. The top of an
    /// unfilled tank is ullage, not fuel, so it does not conduct.
    fn wetted_area_m2(&self) -> f64 {
        let h = self.shape.box_height_m();
        let l = self.shape.box_length_m();
        let depth = self.liquid_depth_m();
        (h * l) + 2.0 * (depth * l)
    }
}

// ---------------------------------------------------------------------------
// The live system.
// ---------------------------------------------------------------------------

/// The live A380 fuel system.
pub struct FuelLive {
    ids: Ids,
    tanks: Vec<TankState>,
    /// The engine feed filters' blockage, and the FCOC heat each engine is
    /// rejecting into its feed tank.
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
    /// Whether any of the four cross-feed valves is crew-selected open this
    /// tick (`Truth::controls::crossfeed_valve_selected`), published as
    /// `FUEL_CROSSFEED_OPEN` -- kept as its own field rather than read back
    /// from `Truth` in `publish` because `publish` only ever sees `&self`.
    crossfeed_open: bool,
    /// Each trim pump's own raw degradation, published individually
    /// (`FUEL_TRIM_PUMP_DEGRADATION:1`/`:2`) because the path's own fault
    /// detector cannot show a single pump's failure at all: the two pumps
    /// are modelled as fully mutually redundant (`tick_transfers`'s own
    /// `trim_pump_loss` takes the *minimum* of the two, i.e. the healthier
    /// one), which is a real design choice `registry.rs` documents
    /// explicitly ("both together stop trim transfer altogether") and not a
    /// bug to remove -- but it does mean a maintenance-facing indication of
    /// one pump's own health has to be a direct instrument reading, the same
    /// way a real aircraft still shows a LO PRESS caution on the specific
    /// failed pump even though its redundant twin keeps the transfer going.
    trim_pump_degradation: [f64; 2],
    leak_detector: LeakDetector,
    leak_detected: bool,
    total_leak_kg_s: f64,
    baffle_damage_detected: bool,
    fqms_low_confidence: bool,
    fob_lo_temp: bool,
    filter_ice_detected: bool,
    /// Each engine feed filter's own raw free-water fraction and heater
    /// health, published individually
    /// (`FUEL_FILTER_WATER_FRACTION:n`/`FUEL_FILTER_HEATER_FAULT:n`) for the
    /// same reason as `trim_pump_degradation`: `thermal::
    /// filter_ice_blockage_fraction` only shows *ice*, which needs cold fuel
    /// **and** free water **and** a failed heater all at once (a working
    /// heater's whole job is to prevent any icing at all, so it correctly
    /// suppresses the water fault's ice consequence to zero on its own, and
    /// there is correctly no water to freeze when only the heater has
    /// failed) -- a real aircraft still carries a direct water-in-fuel
    /// sensor and a direct heater-fault caution independent of whether ice
    /// has actually formed yet, which is what these two publish.
    filter_water_fraction: [f64; N_ENGINES],
    filter_heater_failed: [bool; N_ENGINES],
    indicated_fob_kg: f64,
    /// Each of the eleven real tanks' own `Position` from `flight_model.cfg`
    /// (`weight_balance::parse`'s own `Balance::tanks`, `Tank.1`..`Tank.11`
    /// order, matching `ALL_TANKS`), feet from the reference datum. Resolved
    /// once at construction -- this is FlyByWire's own real geometry, not a
    /// per-tick reparse -- and used only to feed `fuel_cg_ft`'s call into
    /// `weight_balance::centre_of_gravity`, the crate's one real arm/moment
    /// computation, rather than a second one duplicated in this directory.
    tank_positions_ft: Vec<[f64; 3]>,
    /// The eleven tanks' own longitudinal centre of gravity this tick, feet
    /// from `weight_balance`'s reference datum (its own sign convention:
    /// more positive is further forward). This is fuel-only -- it does not
    /// include the empty aircraft or payload, so it is not `weight_balance::
    /// WeightBalance`'s own published aircraft CG -- but it is a real
    /// consequence of trim/CG transfer's own mass movement: without it nothing
    /// in this directory ever showed the sole reason trim transfer exists
    /// (moving the CG), even once the mass itself started moving for real.
    fuel_cg_ft: f64,
    /// Inputs `Truth` does not carry; see [`FuelCommands`].
    pub commands: FuelCommands,
    /// Fuel grade uplifted this sector. `JetA1` is the international
    /// long-haul grade `thermal.rs`'s own doc names as what most A380
    /// operators actually load; the crew/dispatch can change it.
    pub fuel_type: FuelType,
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
        };
        live.seed_default_fuel_load(ambient_c);
        live
    }

    /// Seeds every one of the eleven tanks to [`FULL_LOAD_FRACTION`] of its
    /// own structural capacity (`geometry::TankShape::capacity_gal`): a
    /// realistic full long-haul dispatch load, not
    /// `crate::fuel::DEFAULT_GALLONS`'s ramp/cold-start default (FlyByWire's
    /// own FADEC cold-start gallons, `FuelConfiguration_A380X.h:31-43`),
    /// which fills the four feed tanks alone and leaves the other seven --
    /// both outer, both mid, both inner and the trim tank -- permanently
    /// dry.
    ///
    /// This is the honest fix for "nothing seeds this system, so a live
    /// aircraft starts with dry tanks": `deep::fuel` cannot read X-Plane's
    /// own live tank quantities itself (areas depend on `Truth`/`Faults`
    /// only, and neither carries a per-tank fuel mass -- see this module's
    /// own doc and `docs/deep/truth-requests.md`), and the plugin wiring
    /// that would call [`Self::load_tank`] with those live readings each
    /// time the aircraft loads or is refuelled belongs to `plugin.rs`,
    /// outside this pass's directory. But seeding only the feed tanks (a
    /// real number, for cold engine start) meant the other seven tanks'
    /// own wall-leak failures, and both trim-tank pumps, could never act on
    /// anything: a wall leak's driving head is that tank's own liquid depth
    /// and a pump's job is to move that tank's own contents
    /// (`deep::integration::failure_audit`'s sweep found exactly this: 7 of
    /// 11 tank-wall leaks measured dead). A uniform fraction of each real,
    /// cited capacity is a real dispatch state, not a fabricated fill level,
    /// and every tank still responds correctly to a real `load_tank` call
    /// once the plugin makes one (refuelling, or a future live-quantity
    /// sync, both already exercised by this file's own tests).
    fn seed_default_fuel_load(&mut self, temp_c: f64) {
        for &tank in ALL_TANKS.iter() {
            let capacity_gal = TankShape::of(tank).capacity_gal;
            let kg = capacity_gal * FULL_LOAD_FRACTION * geometry::GAL_TO_M3 * REFERENCE_DENSITY_15C_KG_M3;
            self.load_tank(tank, kg, temp_c);
        }
    }

    /// Put `kg` of fuel at `temp_c` into one tank: refuelling, or the
    /// plugin synchronising this system with the aircraft's real fuel load
    /// once `Truth` carries it. Floored at zero and capped at the tank's own
    /// structural capacity at `temp_c`'s density -- a load request above
    /// what the tank can physically hold is clamped, not passed straight
    /// through (a tank cannot hold more fuel than its own box volume times
    /// the fuel's own density, whatever the caller asks for).
    pub fn load_tank(&mut self, tank: Tank, kg: f64, temp_c: f64) {
        let i = ALL_TANKS.iter().position(|&t| t == tank).expect("every tank is in ALL_TANKS");
        self.tanks[i].temp_c = temp_c;
        let capacity_kg = self.tanks[i].shape.capacity_m3() * self.tanks[i].density_kg_m3();
        self.tanks[i].mass_kg = kg.max(0.0).min(capacity_kg.max(0.0));
    }

    pub fn tank_mass_kg(&self, tank: Tank) -> f64 {
        let i = ALL_TANKS.iter().position(|&t| t == tank).expect("every tank is in ALL_TANKS");
        self.tanks[i].mass_kg
    }

    pub fn tank_temp_c(&self, tank: Tank) -> f64 {
        let i = ALL_TANKS.iter().position(|&t| t == tank).expect("every tank is in ALL_TANKS");
        self.tanks[i].temp_c
    }

    /// Total fuel on board the tanks actually hold, kg (as opposed to
    /// [`Self::indicated_fob_kg`], which is what the FQMS believes).
    pub fn true_fob_kg(&self) -> f64 {
        self.tanks.iter().map(|t| t.mass_kg).sum()
    }

    pub fn indicated_fob_kg(&self) -> f64 {
        self.indicated_fob_kg
    }

    /// The four feed tanks, in engine order: engine 1 is fed by Feed 1 and
    /// so on (`flight_model.cfg`'s own `Tank.2`/`Tank.5`/`Tank.6`/`Tank.9`
    /// naming, which `geometry::Tank` reproduces).
    fn feed_tank_index(engine: usize) -> usize {
        const FEED: [Tank; N_ENGINES] = [Tank::Feed1, Tank::Feed2, Tank::Feed3, Tank::Feed4];
        ALL_TANKS.iter().position(|&t| t == FEED[engine]).expect("feed tanks are in ALL_TANKS")
    }

    /// Total mass in the left and right wing groups, for the wing-balance
    /// check. The trim tank is on the centreline and belongs to neither.
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

/// Lateral imbalance the A380's own wing-balance procedure acts on, kg.
/// **GENERIC**: no public A380 figure. Set at 3000 kg, the order of
/// magnitude large-transport imbalance limits sit at, and used only to
/// decide whether a cross-feed is *required* -- the cross-feed fault itself
/// is detected from the valve's own stuck fraction, not from this number.
const WING_IMBALANCE_LIMIT_KG: f64 = 3000.0;

/// Outer-tank retention: the load-alleviation sequence keeps the outer
/// tanks full until the inner and mid tanks have fallen to this fraction
/// (`cg_transfer::outer_tank_retention_active`'s own
/// `retain_until_fraction`). **GENERIC**.
const OUTER_RETENTION_UNTIL_FRACTION: f64 = 0.25;

impl crate::deep::live::Area for FuelLive {
    fn name(&self) -> &'static str {
        "fuel"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        let dt = truth.dt_s.max(0.0);
        let ambient_pa = truth.environment.ambient_pressure_pa.max(1.0);
        // The wing skin the fuel exchanges heat with is X-Plane's own
        // kinetically-heated leading-edge temperature, not free-stream SAT:
        // at cruise Mach the skin runs tens of degrees warmer than the air,
        // and that difference is exactly what keeps a long-haul tank above
        // its freeze point.
        let skin_c = truth.environment.leading_edge_c;

        // ---- Tanks: faults, thermal balance, leaks -----------------------
        self.total_leak_kg_s = 0.0;
        self.baffle_damage_detected = false;
        self.fqms_low_confidence = false;
        self.fob_lo_temp = false;

        let gallery_leak_fraction = self.ids.gallery_leak.iter().map(|&id| faults.get(id)).fold(0.0f64, f64::max);
        let gallery_leak_area = leak::leak_area_m2(gallery_leak_fraction, MAX_GALLERY_LEAK_AREA_M2);

        for i in 0..N_TANKS {
            // Baffle/rib damage reduces this tank's slosh damping in
            // proportion, exactly as `registry.rs` names it.
            let baffle_damage = faults.get(self.ids.baffle[i]);
            let nominal_damping = TankShape::of(ALL_TANKS[i]).slosh_damping_ratio;
            self.tanks[i].shape.slosh_damping_ratio = nominal_damping * (1.0 - baffle_damage);
            if baffle_damage > 0.0 {
                self.baffle_damage_detected = true;
            }

            // Probe array: one element carries the single-probe failure,
            // every element carries the compensator's common-mode bias.
            let probe_fault = faults.get(self.ids.probe[i]);
            let compensator = faults.get(self.ids.compensator[i]);
            let count = self.tanks[i].probes.len();
            for (j, p) in self.tanks[i].probes.iter_mut().enumerate() {
                let own = if j == 0 { probe_fault } else { 0.0 };
                // A compensator fault biases every surviving probe
                // together; `fqms_indicated_fraction` turns a probe's own
                // failure fraction into a bias, so the common-mode fault
                // enters as a floor under every probe's fraction. Held
                // below the BITE dead threshold so it stays the
                // undetectable common-mode error `registry.rs` describes
                // rather than killing the whole array.
                let common = compensator * gauging::PROBE_DEAD_THRESHOLD * 0.999;
                p.failure_fraction = own.max(common).clamp(0.0, 1.0);
            }
            debug_assert!(count > 0);

            // Heat balance: conduction to the wing skin, plus this tank's
            // share of the FCOC heat its engine rejects into the fuel.
            // `thermal::fcoc_temperature_rise_k` does the `Q*dt/(m*cp)`
            // arithmetic but floors its heat at zero (it was written for
            // the heating term alone), so the cooling direction is taken
            // by magnitude and signed back on.
            let fcoc_w = self.fcoc_heat_into_tank_w(i, truth, faults);
            let ua_w_k = TANK_SKIN_U_W_M2K * self.tanks[i].wetted_area_m2();
            let mass = self.tanks[i].mass_kg;
            if mass > 0.0 && ua_w_k > 0.0 {
                // Where this balance is heading: the skin temperature,
                // offset by whatever the FCOC is pushing in against the
                // wall's conductance.
                let equilibrium_c = skin_c + fcoc_w / ua_w_k;
                let net_w = fcoc_w + ua_w_k * (skin_c - self.tanks[i].temp_c);
                let step_k = thermal::fcoc_temperature_rise_k(net_w.abs(), dt, mass, FUEL_CP_J_KGK).copysign(net_w);
                let stepped = self.tanks[i].temp_c + step_k;
                // An explicit step with a long `dt` on a nearly empty tank
                // would otherwise overshoot and oscillate: the fuel can
                // approach its equilibrium but never cross it.
                self.tanks[i].temp_c = if net_w >= 0.0 { stepped.min(equilibrium_c) } else { stepped.max(equilibrium_c) };
            } else if mass <= 0.0 {
                self.tanks[i].temp_c = skin_c;
            }

            if thermal::wax_fraction(self.tanks[i].temp_c, self.fuel_type) > 0.0 {
                self.fob_lo_temp = true;
            }

            // Wall leak: driven by this tank's own remaining head.
            let density = self.tanks[i].density_kg_m3();
            let area = leak::leak_area_m2(faults.get(self.ids.tank_leak[i]), MAX_TANK_WALL_LEAK_AREA_M2);
            let wall_leak = leak::tank_wall_leak_kg_s(area, self.tanks[i].liquid_depth_m(), density);
            self.tanks[i].leak_kg_s = wall_leak;
            self.tanks[i].mass_kg = (self.tanks[i].mass_kg - wall_leak * dt).max(0.0);
            self.total_leak_kg_s += wall_leak;
        }

        // Gallery leak: the transfer galleries run at the boost pumps'
        // delivery pressure above ambient, so the leak is driven by that
        // rise rather than by a tank's head.
        if gallery_leak_area > 0.0 {
            let density = self.tanks[0].density_kg_m3();
            let line_pa = ambient_pa + NOMINAL_JETTISON_PUMP_RISE_PA;
            let gallery = leak::gallery_leak_kg_s(gallery_leak_area, line_pa, ambient_pa, density);
            self.total_leak_kg_s += gallery;
            // Taken out of whichever tank currently holds the most fuel:
            // the gallery is fed from the tank being transferred out of.
            if let Some(idx) = (0..N_TANKS).max_by(|&a, &b| self.tanks[a].mass_kg.total_cmp(&self.tanks[b].mass_kg)) {
                self.tanks[idx].mass_kg = (self.tanks[idx].mass_kg - gallery * dt).max(0.0);
            }
        }

        // ---- Burn --------------------------------------------------------
        // `physics::engine`'s own real fuel flow, not a fan-speed guess: see
        // this module's doc.
        for eng in 0..N_ENGINES {
            let idx = Self::feed_tank_index(eng);
            let burn = truth.engine_fuel_flow_kg_s[eng].max(0.0) * dt;
            self.tanks[idx].mass_kg = (self.tanks[idx].mass_kg - burn).max(0.0);
        }
        if self.commands.apu_fuel_flow_kg_s > 0.0 {
            let idx = Self::feed_tank_index(1);
            self.tanks[idx].mass_kg = (self.tanks[idx].mass_kg - self.commands.apu_fuel_flow_kg_s * dt).max(0.0);
        }

        // ---- Filters -----------------------------------------------------
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
            // Direct instrument readings, independent of whether ice has
            // actually formed: see `filter_water_fraction`'s own doc for why
            // `filter_ice`/`filter_blockage` alone cannot show either fault
            // in isolation.
            self.filter_water_fraction[eng] = water;
            self.filter_heater_failed[eng] = heater_failed;
            if ice > 0.0 {
                self.filter_ice_detected = true;
            }
        }

        // ---- FQMS --------------------------------------------------------
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

        // ---- Transfer paths ----------------------------------------------
        self.tick_transfers(truth, faults, dt, gallery_leak_fraction);

        // The fuel-only longitudinal CG, from `weight_balance`'s own real
        // arm/moment computation over this tick's own (just-moved) tank
        // masses -- the one place trim/CG transfer's mass movement actually
        // consumes for the purpose the real component exists for.
        let masses = self.tanks.iter().zip(self.tank_positions_ft.iter()).map(|(t, &position)| weight_balance::Mass { pounds: t.mass_kg / weight_balance::LB_TO_KG, position });
        self.fuel_cg_ft = weight_balance::centre_of_gravity(masses).1[0];

        // ---- Jettison ----------------------------------------------------
        self.tick_jettison(truth, faults, dt);

        // ---- Leak detection ----------------------------------------------
        // Metered *and accounted* consumption: the engines, the APU, and a
        // commanded jettison. `leak::LeakDetector` has no notion of
        // jettison on its own -- it only ever sees a total indicated
        // quantity and a "fuel used" figure -- so a jettison in progress
        // has to be folded into the same accounted term the engines are, or
        // the detector reads a real, commanded, non-leak loss as an
        // unmetered one and either raises a false FUEL LEAK during every
        // jettison or, worse, has its confirm window's accounting thrown off
        // by jettison flow while a real leak is also present.
        let metered_flow =
            truth.engine_fuel_flow_kg_s.iter().map(|f| f.max(0.0)).sum::<f64>() + self.commands.apu_fuel_flow_kg_s.max(0.0) + self.jettison_flow_kg_s[0].max(0.0) + self.jettison_flow_kg_s[1].max(0.0);
        self.leak_detected = self.leak_detector.update(self.indicated_fob_kg, metered_flow, dt, LEAK_WINDOW_S, LEAK_THRESHOLD_KG, LEAK_CONFIRM_WINDOWS);
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        let b = |x: bool| if x { 1.0 } else { 0.0 };

        // Every variable this area's `registry.rs` names in an ECAM
        // trigger.
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

        // The state behind those flags, for the EFB's Study pages.
        out("FUEL_TOTAL_FOB_KG", self.indicated_fob_kg);
        out("FUEL_TOTAL_TRUE_FOB_KG", self.true_fob_kg());
        out("FUEL_TOTAL_LEAK_KG_S", self.total_leak_kg_s);
        out("FUEL_CG_LONGITUDINAL_FT", self.fuel_cg_ft);
        for (i, tank) in self.tanks.iter().enumerate() {
            let n = i + 1;
            out(&format!("FUEL_TANK_QTY_KG:{n}"), tank.indicated_mass_kg);
            out(&format!("FUEL_TANK_TRUE_QTY_KG:{n}"), tank.mass_kg);
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
    }
}

impl FuelLive {
    /// Heat this engine's fuel-cooled oil cooler rejects into its feed
    /// tank, W. The FCOC only rejects heat while its engine is turning,
    /// and its fouling fault attenuates the heat that reaches the fuel --
    /// exactly as `registry.rs` names it ("`fcoc_heat_w` input attenuated
    /// by `1 - fouling_fraction`").
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

    /// Moves up to `want_kg` of real mass from tank `from` to tank `to`,
    /// capped by what `from` actually has and by the room left in `to`'s own
    /// structural capacity at its current bulk temperature's density (the
    /// same capacity `fill_fraction`/`load_tank` use) -- never drained
    /// below zero, never filled past capacity, and total system mass is
    /// conserved exactly: whatever leaves `from` is exactly what enters
    /// `to`, nothing created or destroyed in between. Returns the kg
    /// actually moved.
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

    /// The source tank a wing side's own CG-transfer sequence draws from
    /// this tick: inner first, then mid, then outer once retention has
    /// released it -- `None` once the whole side is dry. Mirrors
    /// `LegacyFuel.ts`'s own real ordering (inner/mid transfer first, outer
    /// last) that `cg_transfer.rs`'s module doc already credits to
    /// `fuel_transfer.rs`; this is the same priority applied to *this*
    /// ledger's own tanks.
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

    /// The transfer paths: each one's `TransferFaults` from the failures
    /// registered against its own valves and pumps, the shortfall detector
    /// `registry.rs` wires to its ECAM alert, and now the real mass each
    /// path actually moves at `achieved_transfer_rate_kg_s`'s own delivered
    /// rate (see the `nominal_transfer_rate_kg_s` section for where that
    /// rate comes from).
    ///
    /// This does not route through a second, live `fuel_network::
    /// FuelNetwork`: that module's own tanks, pumps and valves already
    /// drive the real aircraft's weight through `crate::fuel::FuelSystem`,
    /// and this ledger's eleven tanks are necessarily a separate, seeded
    /// shadow of them (`FuelLive::seed_default_fuel_load`'s own doc: `Truth`
    /// carries no per-tank quantity for this area to read back from the
    /// live network). Owning a second `FuelNetwork` instance here, with its
    /// own topology and its own cockpit-driven valve/pump state, would be
    /// exactly the second fuel model this pass was told not to add -- two
    /// ledgers that could disagree about where the mass is. Instead, this
    /// reuses the *numbers* `fuel_network.rs` already parses from FlyByWire's
    /// own cfg (pump pressure, line `FuelFlowAt1PSI`) and the same flow
    /// formula that module's own tests already exercise, so a failed pump or
    /// valve here and in the live network are rated against the same real
    /// figures, without a second stateful topology to keep in sync.
    fn tick_transfers(&mut self, truth: &Truth, faults: &Faults, dt: f64, gallery_leak_fraction: f64) {
        let worst = |ids: &[u64]| ids.iter().map(|&id| faults.get(id)).fold(0.0f64, f64::max);

        // Each pump's own raw health, for the direct per-pump indication
        // (`trim_pump_degradation`'s own doc): recorded before the
        // redundancy fold below so a single pump's failure is still visible
        // even though it cannot move the path-level fault flag on its own.
        for (i, &id) in self.ids.trim_pump.iter().enumerate() {
            self.trim_pump_degradation[i] = faults.get(id);
        }

        // Trim transfer: the two trim pumps in parallel (both must degrade
        // for the path to lose flow), in series with the inlet valves and
        // the line isolation valves.
        let trim_nominal = nominal_transfer_rate_kg_s(TRIM_PUMP_PRESSURE_PSI);
        let trim_pump_loss = self.ids.trim_pump.iter().map(|&id| faults.get(id)).fold(f64::INFINITY, f64::min);
        let trim = TransferFaults {
            valve_stuck_fraction: worst(&self.ids.trim_inlet).min(1.0).max(worst(&self.ids.trim_iso)),
            pump_degradation_fraction: if trim_pump_loss.is_finite() { trim_pump_loss } else { 0.0 },
            gallery_leak_fraction,
        };
        let achieved = cg_transfer::achieved_transfer_rate_kg_s(trim_nominal, &trim);
        self.trim_fault = self.trim_detector.update(trim_nominal, achieved, TRANSFER_TOLERANCE, 1.0, dt);

        // Real mass: the trim tank feeds forward into the four feed tanks
        // (`Pump.19`/`20`'s own `TankFuelRequired:Trim`, draining through
        // the inlet/iso valves and both galleries to reach every feed tank),
        // split evenly across whichever of the four still have room. A dry
        // trim tank or a fully failed path both floor `moved` at zero
        // through `move_fuel`'s own availability/capacity caps.
        if let Some(trim_idx) = ALL_TANKS.iter().position(|&t| t == Tank::Trim) {
            let feed_indices = [Tank::Feed1, Tank::Feed2, Tank::Feed3, Tank::Feed4].map(|t| ALL_TANKS.iter().position(|&x| x == t).expect("feed tanks are in ALL_TANKS"));
            let share = achieved * dt / feed_indices.len() as f64;
            for feed_idx in feed_indices {
                self.move_fuel(trim_idx, feed_idx, share);
            }
        }

        // Automatic CG control through the outer/inner/mid transfer valves.
        // The sequence is only *required* while the outer tanks are still
        // being retained or the inners are still feeding the feed tanks --
        // which, with any fuel on board at all, is the whole flight.
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

        // Real mass: each wing side independently, inner/mid/outer (by
        // `cg_source_index`'s own priority) into that side's own two feed
        // tanks, split evenly between them. `cg_achieved` is this path's own
        // per-side delivered rate (`WING_TRANSFER_PUMP_PRESSURE_PSI` is one
        // side's own pump), so both sides run it independently rather than
        // splitting one combined budget between them.
        let sides = [(Tank::LeftInner, Tank::LeftMid, Tank::LeftOuter, [Tank::Feed1, Tank::Feed2]), (Tank::RightInner, Tank::RightMid, Tank::RightOuter, [Tank::Feed3, Tank::Feed4])];
        for (inner, mid, outer, feeds) in sides {
            if let Some(src) = self.cg_source_index(inner, mid, outer, outer_retained) {
                let share = cg_achieved * dt / feeds.len() as f64;
                for feed in feeds {
                    let feed_idx = ALL_TANKS.iter().position(|&x| x == feed).expect("feed tanks are in ALL_TANKS");
                    self.move_fuel(src, feed_idx, share);
                }
            }
        }

        // Wing cross-feed: the fault detector treats a transfer as
        // *required* either because the crew has selected at least one of
        // the four cross-feed valves open
        // (`Truth::controls::crossfeed_valve_selected`) or because the
        // wings are genuinely out of balance -- the same shape as before.
        // Real mass only ever moves when the valve is actually selected
        // open, though: an imbalance alone raises the fault (the crew has
        // not corrected it) but cannot open a valve nobody has selected, so
        // it must not move fuel on its own -- a closed valve passes zero
        // flow regardless of how unbalanced the wings are.
        let (left, right) = self.wing_masses_kg();
        self.crossfeed_open = truth.controls.crossfeed_valve_selected.iter().any(|&s| s);
        let xfeed_required = self.crossfeed_open || cg_transfer::wing_balance_transfer_needed(left, right, WING_IMBALANCE_LIMIT_KG);
        let xfeed_nominal = nominal_transfer_rate_kg_s(WING_TRANSFER_PUMP_PRESSURE_PSI);
        let xfeed = TransferFaults { valve_stuck_fraction: worst(&self.ids.crossfeed), pump_degradation_fraction: 0.0, gallery_leak_fraction };
        let xfeed_required_rate = if xfeed_required { xfeed_nominal } else { 0.0 };
        let xfeed_achieved = cg_transfer::achieved_transfer_rate_kg_s(xfeed_required_rate, &xfeed);
        self.crossfeed_fault = self.crossfeed_detector.update(xfeed_required_rate, xfeed_achieved, TRANSFER_TOLERANCE, 2.0, dt);

        // Real mass: only while the valve is actually selected open, from
        // whichever side is heavy into whichever side is light, using each
        // side's own most-full/least-full feed tank as the physical
        // manifold (`CrossFeedValve1..4` all terminate on the feed-tank
        // galleries, `flight_model.cfg` `Line.132..137`) -- recomputed after
        // the trim/CG moves above, so cross-feed reacts to this tick's own
        // post-transfer imbalance, not a stale one from the top of the
        // function.
        if self.crossfeed_open {
            // `xfeed_required_rate` is `xfeed_nominal` whenever
            // `crossfeed_open` is true (it is one of `xfeed_required`'s own
            // two conditions), so `xfeed_achieved` above already is this
            // path's real delivered rate with the valve open -- reused
            // rather than recomputed.
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

    /// Jettison: each side's valve travels, its nozzle passes what the
    /// remaining head and the jettison pump can push through it, and the
    /// fuel leaves the wing it came from.
    ///
    /// This is the first caller of
    /// [`jettison::jettison_mass_flow_kg_s`]'s nozzle-exit-pressure
    /// argument. The A380's tanks are vented through the surge tanks' NACA
    /// vents, so the ullage sits at ambient static pressure and the nozzle
    /// discharges into air at very nearly the same static pressure: ullage
    /// = exit = `truth.environment.ambient_pressure_pa` is the correct
    /// default, and the two cancel to leave the gauge head plus the pump
    /// rise, exactly as that function's own doc sets out.
    fn tick_jettison(&mut self, truth: &Truth, faults: &Faults, dt: f64) {
        let ambient_pa = truth.environment.ambient_pressure_pa.max(0.0);

        // Jettison draws from the wing tanks, left nozzle from the left
        // wing and right from the right: the deepest tank on that side
        // sets the head at the nozzle and is the one that drains.
        const LEFT_GROUP: [Tank; 4] = [Tank::LeftInner, Tank::LeftMid, Tank::LeftOuter, Tank::Feed1];
        const RIGHT_GROUP: [Tank; 4] = [Tank::RightInner, Tank::RightMid, Tank::RightOuter, Tank::Feed4];

        for side in 0..2 {
            // Master jettison arm switch, plus this side's own nozzle-valve
            // pushbutton (`Truth::controls::jettison_armed`/
            // `jettison_valve_selected`) -- the real two-stage A380 panel
            // (an ARM guard plus an independent VALVE OPEN pushbutton per
            // side), not a single combined selector.
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
            // The jettison pump only pushes while there is fuel over its
            // inlet to push.
            let pump_pa = if depth > 0.0 && commanded { NOMINAL_JETTISON_PUMP_RISE_PA } else { 0.0 };
            let flow = jettison::jettison_mass_flow_kg_s(cda, depth, pump_pa, ambient_pa, ambient_pa, density);
            let clear_flow = jettison::jettison_mass_flow_kg_s(clear_cda, depth, pump_pa, ambient_pa, ambient_pa, density);
            self.jettison_flow_kg_s[side] = flow;
            self.tanks[idx].mass_kg = (self.tanks[idx].mass_kg - flow * dt).max(0.0);

            // The nozzle's own fault monitor: the valve has not reached the
            // position it was told to take, or the rate through an open
            // nozzle is short of what its position should be passing.
            let target = if commanded { 1.0 } else { 0.0 };
            let valve_disagree = stuck > 0.0 && (position - target).abs() > VALVE_DISAGREE_TOLERANCE;
            let rate_short = commanded && clear_flow > 0.0 && flow < clear_flow * (1.0 - TRANSFER_TOLERANCE);
            self.jettison_fault[side] = valve_disagree || rate_short;
        }
    }
}

/// How far a jettison valve may sit from its commanded position before the
/// system calls it a disagreement, as a fraction of full travel.
/// **GENERIC**: a position feedback tolerance, set well outside the travel
/// the valve covers in one frame at 30 Hz (1/5 s per frame of a 5 s travel
/// is 0.7%) so normal travel never trips it.
const VALVE_DISAGREE_TOLERANCE: f64 = 0.05;

/// Heat one engine's fuel-cooled oil cooler rejects into the fuel at
/// take-off power, W. **GENERIC**: no Trent 900 FCOC duty is published.
/// Derived from the scavenge oil heat a large turbofan's oil system carries
/// -- of order 100 kW per engine at take-off, of which the FCOC takes the
/// share the air-cooled cooler does not -- so 60 kW is the right order for
/// the fuel side. It scales with N1 because both the oil heat generated and
/// the fuel flow through the cooler do.
const FCOC_HEAT_AT_TAKEOFF_W: f64 = 60_000.0;

/// This area's live system.
pub fn live_system() -> Box<dyn crate::deep::live::Area> {
    Box::new(FuelLive::new())
}

/// Test-only scaffolding shared by the five live systems in this push.
///
/// Walking an `EcamAlert`'s trigger for the variables it reads is the same
/// job in every area, and every area's live system needs it to prove that
/// nothing it registers triggers on a variable nobody publishes. It lives
/// here rather than in `deep::api` only because this push may not edit
/// that file; it belongs there.
#[cfg(test)]
pub(crate) mod test_support {
    use crate::deep::api::Cond;

    /// Every variable name `cond` reads, in any position.
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

    /// The whole point of the live layer: every variable an alert triggers
    /// on has to be published by somebody. These are the ones this area
    /// owns; the rest are cockpit controls and other areas' outputs, named
    /// explicitly so the split is deliberate rather than an oversight.
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

    /// `Truth::controls` with the four cross-feed valves crew-selected open.
    fn crossfeed_selected_truth() -> Truth {
        Truth { controls: Controls { crossfeed_valve_selected: [true; 4], ..Controls::default() }, ..Truth::default() }
    }

    /// `Truth::controls` with jettison armed and both nozzle valves
    /// selected -- the real two-stage panel `tick_jettison` now reads.
    fn jettison_selected_truth() -> Truth {
        Truth { controls: Controls { jettison_armed: true, jettison_valve_selected: [true; 2], ..Controls::default() }, ..Truth::default() }
    }

    #[test]
    fn a_stuck_crossfeed_valve_raises_the_wing_crossfeed_fault_its_registry_entry_promises() {
        // registry.rs: "wing-balance cross-feed cannot move fuel between
        // wings through this valve".
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
        // registry.rs: "jettison rate through that nozzle falls in
        // proportion, lengthening the time needed to reach max landing
        // weight".
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
        // The tanks are vented: ullage = ambient = nozzle exit, so the two
        // cancel and the rate is the same at sea level and at cruise.
        let mut sea_level = FuelLive::new();
        full_tanks(&mut sea_level, 10.0);
        let low = run(&mut sea_level, &jettison_selected_truth(), &Faults::default(), 20.0);

        let mut cruise = FuelLive::new();
        full_tanks(&mut cruise, 10.0);
        let mut truth = jettison_selected_truth();
        truth.environment.ambient_pressure_pa = 22_600.0; // ~FL350
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
        let id = live.ids.probe[3]; // left inner: the tank with the most probes
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

    /// `deep::integration::failure_audit`'s sweep found 7 of the 11
    /// tank-wall leaks dead because `seed_default_fuel_load` only filled the
    /// four feed tanks. This proves the fix directly: a leak on an
    /// otherwise-untouched non-feed tank, from the aircraft's own default
    /// seeded state (no `full_tanks` test helper), must still lose real
    /// fuel. `LeftOuter` and `Trim` are two of the seven tanks that could
    /// never leak a drop before this fix.
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

    /// The two trim pumps are seeded now (`FULL_LOAD_FRACTION` fills the
    /// trim tank too), but `TransferFaultDetector`'s own redundancy model
    /// means a single pump's failure genuinely cannot move
    /// `FUEL_TRIM_TRANSFER_FAULT` on its own -- that is a real, registered
    /// design choice (`registry.rs`: "both together stop trim transfer
    /// altogether"), not a bug. What must still change is the direct
    /// per-pump indication.
    #[test]
    fn a_single_failed_trim_pump_is_masked_by_its_own_redundancy_but_still_shows_on_its_own_gauge() {
        let mut live = FuelLive::new();
        let id = live.ids.trim_pump[0];
        let out = run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 1.0)]), 5.0);
        assert_eq!(out.get("FUEL_TRIM_TRANSFER_FAULT"), Some(&0.0), "the healthy twin pump genuinely covers a single failure");
        assert_eq!(out.get("FUEL_TRIM_PUMP_DEGRADATION:1"), Some(&1.0), "but the failed pump's own health must still be a real, published reading");
        assert_eq!(out.get("FUEL_TRIM_PUMP_DEGRADATION:2"), Some(&0.0));
    }

    /// `thermal::filter_ice_blockage_fraction` needs cold fuel, free water
    /// *and* a failed heater all at once, so a single-fault audit sweep
    /// cannot see either the water-contamination or the heater failure on
    /// its own through the ice consequence alone (a working heater's whole
    /// job is to suppress ice regardless of how much water is present, and
    /// there is nothing to freeze with no water). Each failure still has to
    /// move something on its own: a real water-in-fuel sensor and a real
    /// heater-fault caution, independent of whether ice has actually formed.
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

    /// The whole point of `leak::LeakDetector`: fuel leak's entire value is
    /// early annunciation, so a full-severity feed-tank leak (the kind
    /// `deep::integration::failure_audit`'s sweep found could never confirm
    /// inside any profile it ran) must raise `FUEL_LEAK_DETECTED` well
    /// inside 120 s -- not the 180 s minimum the previous constants
    /// demanded.
    #[test]
    fn a_full_severity_feed_tank_leak_raises_fuel_leak_detected_within_120_seconds() {
        let mut live = FuelLive::new();
        let id = live.ids.tank_leak[1]; // Feed1
        let mut truth = Truth::default();
        truth.dt_s = 1.0; // one tick per simulated second, matching the window math
        let out = run(&mut live, &truth, &Faults::from_pairs([(id, 1.0)]), 110.0);
        assert_eq!(out.get("FUEL_LEAK_DETECTED"), Some(&1.0), "a full-severity feed-tank leak must confirm within 120 s of unmetered loss");
    }

    /// The companion fix this leak-detector change depends on: jettison flow
    /// is a real, commanded, *accounted* loss, and must not itself be read
    /// as an unmetered "leak" by the same detector a genuine leak uses.
    #[test]
    fn a_commanded_jettison_alone_never_raises_fuel_leak_detected() {
        let mut live = FuelLive::new();
        let mut truth = jettison_selected_truth();
        truth.dt_s = 1.0;
        let out = run(&mut live, &truth, &Faults::default(), 110.0);
        assert_eq!(out.get("FUEL_LEAK_DETECTED"), Some(&0.0), "a commanded jettison is accounted for, not a leak");
    }

    /// Every tank that now feeds a feed tank by real transfer (trim, and
    /// each wing's own inner/mid/outer sequence) rather than by burn --
    /// drained to zero so a test that wants to isolate burn's own effect on
    /// a feed tank is not also seeing this pass's other fix, real transfer
    /// mass movement, replenishing it in the background.
    fn drain_transfer_sources(live: &mut FuelLive) {
        for tank in [Tank::Trim, Tank::LeftOuter, Tank::LeftMid, Tank::LeftInner, Tank::RightInner, Tank::RightMid, Tank::RightOuter] {
            live.load_tank(tank, 0.0, 15.0);
        }
    }

    /// The whole point of this pass: a feed tank drains at the engine's
    /// own real fuel flow (`Truth::engine_fuel_flow_kg_s`), not at a
    /// derived, fan-speed-based guess, and the aircraft starts with fuel
    /// on board in the first place instead of dry tanks. Isolated from this
    /// same pass's other fix (trim/CG transfer now moves real mass into the
    /// feed tanks) by draining every non-feed tank first -- an empty source
    /// genuinely cannot replenish anything (`move_fuel`'s own availability
    /// cap), so what is left is burn alone, which is what this test means
    /// to measure.
    #[test]
    fn a_feed_tank_drains_at_the_engines_real_burn_from_a_seeded_load() {
        let mut live = FuelLive::new();
        let before = live.tank_mass_kg(Tank::Feed1);
        // Feed1 and Feed2 have different real capacities (7299.6 vs 7753.2
        // US gal, `geometry::shape`), so each needs its own "before" --
        // comparing Feed2 against Feed1's is only safe by the coincidence
        // that `crate::fuel::DEFAULT_GALLONS` used to load every feed tank
        // to the same 1233.9 gal, which the real per-tank seed
        // (`FULL_LOAD_FRACTION` of each tank's own capacity) no longer does.
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

        // A fan spinning with no engine fuel flow reported must not burn
        // anything -- this is not the fan-speed-derived guess it replaced.
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

        // Water alone, heater working: no ice.
        let with_heater = run(&mut live, &truth, &Faults::from_pairs([(water, 0.5)]), 1.0);
        assert_eq!(with_heater.get("FUEL_FILTER_ICE_DETECTED"), Some(&0.0));

        // Heater failed as well: the ice forms.
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
        // And the physical consequence the registry promises: it sloshes
        // for longer.
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
        // registry.rs: "less of the engine oil's heat is rejected into the
        // returning fuel: the feed tank runs colder than it otherwise
        // would".
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
        // Jet A-1 freezes at -47 C and clouds 10 K above that.
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

    /// Every failure this area registers is either driven by the live
    /// system or named here as deliberately not consumed, with the reason.
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
        // A second registration must hand out the same ids, or `Faults`
        // from the plugin's own registry would not match.
        let a = Ids::resolve();
        let b = Ids::resolve();
        assert_eq!(a.crossfeed, b.crossfeed);
        assert_eq!(a.tank_leak, b.tank_leak);
        // And they are genuinely this area's ids.
        for id in a.tank_leak {
            assert_eq!(id / 1_000_000, Area::Fuel as u64);
            assert_eq!(id / 1_000 % 1_000, ATA as u64);
        }
        assert_eq!(a.baffle[0], failure_id(Area::Fuel, ATA, 1), "the first tank's baffle failure is the area's first id");
    }

    // -----------------------------------------------------------------
    // Real transfer mass movement (this pass's own fix).
    // -----------------------------------------------------------------

    /// Trim/CG transfer moving real mass into a deliberately drained feed
    /// tank, and the aircraft's total fuel unchanged by it: moving fuel
    /// between tanks must conserve mass exactly, not just approximately.
    /// Reverting `tick_transfers`' mass movement (back to fault detection
    /// only) makes `moved_into_feed1` exactly `0.0`, which fails this
    /// test's own first assertion.
    #[test]
    fn a_transfer_tick_moves_real_mass_and_conserves_total_fuel() {
        let mut live = FuelLive::new(); // every tank seeded near-full
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

    /// A degraded trim pump moves measurably less mass than a healthy one,
    /// and a fully failed pair moves none -- the yardstick the task set for
    /// this whole pass. Both trim pumps are failed together
    /// (`TransferFaults::pump_degradation_fraction` takes the *healthier*
    /// of the two, `tick_transfers`' own `trim_pump_loss`, matching
    /// `registry.rs`'s documented "both together stop trim transfer"
    /// redundancy), so a single failed pump alone would be masked by its
    /// twin -- this exercises the path actually losing flow, not the
    /// redundancy. Reverting the mass-movement fix collapses `healthy`,
    /// `degraded` and `failed` to the same `0.0`, which fails every
    /// assertion here except the final one.
    #[test]
    fn a_degraded_trim_pump_moves_less_mass_than_a_healthy_one_and_a_failed_pair_moves_none() {
        let feed_total_kg = |pump_magnitude: f64| -> f64 {
            let mut live = FuelLive::new();
            live.load_tank(Tank::Trim, 20_000.0, 10.0);
            for feed in [Tank::Feed1, Tank::Feed2, Tank::Feed3, Tank::Feed4] {
                live.load_tank(feed, 0.0, 10.0);
            }
            // Drain every CG-transfer source so only the trim path can feed
            // the feed tanks this tick -- an unrelated fix (CG transfer)
            // must not be what this test is actually measuring.
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

    /// Cross-feed selected on the overhead rebalances the wings for real;
    /// selected off, an identical imbalance is left untouched -- the fault
    /// flag may still trip either way (an uncorrected imbalance is still a
    /// real fault), but a shut valve passes no flow. Reverting the
    /// movement gate back to firing on `xfeed_required` (imbalance alone)
    /// instead of `crossfeed_open` would move fuel in the "shut" case too,
    /// failing the second assertion; reverting mass movement entirely
    /// fails the first.
    #[test]
    fn crossfeed_selected_rebalances_the_wings_and_shut_does_not() {
        let light_side_total_kg = |crossfeed_selected: bool| -> f64 {
            let mut live = FuelLive::new();
            for tank in [Tank::Trim, Tank::LeftOuter, Tank::LeftMid, Tank::LeftInner, Tank::RightInner, Tank::RightMid, Tank::RightOuter] {
                live.load_tank(tank, 0.0, 10.0);
            }
            // Left wing heavy, right wing light, well past
            // `WING_IMBALANCE_LIMIT_KG`, with plenty of capacity headroom
            // on the light side to receive real mass.
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

    /// Jettison armed and both nozzle valves selected reduces total fuel at
    /// the real rate `FUEL_JETTISON_FLOW_KG_S` publishes; with nothing
    /// commanded (no burn, no leak, no jettison), total fuel must not move
    /// at all -- transfers alone only ever redistribute it. Reverting
    /// either `tick_jettison`'s own mass subtraction or the `Truth::
    /// controls` wiring this pass added would leave `after_armed` equal to
    /// `before_armed`.
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

    /// Trim transfer exists to move the CG -- the whole reason this pass
    /// also had to connect the mass movement to `weight_balance::
    /// centre_of_gravity` rather than let the moved mass go unconsumed.
    /// The trim tank sits at the aircraft's most aft real position
    /// (`flight_model.cfg` `Tank.11`, `-87.14`) and the feed tanks sit far
    /// forward of it (`-7.45`..`-25.0`), so draining the trim tank into them
    /// must move `FUEL_CG_LONGITUDINAL_FT` forward (`weight_balance`'s own
    /// sign convention: more positive is further forward, its module doc).
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
        // One tick first: `fuel_cg_ft` is only ever computed inside `tick`,
        // so reading it beforehand would just be its `0.0` construction
        // default, not a real "before" CG.
        live.tick(&truth, &Faults::default());
        let before = live.fuel_cg_ft;
        for _ in 0..59 {
            live.tick(&truth, &Faults::default());
        }
        let after = live.fuel_cg_ft;
        assert!(live.tank_mass_kg(Tank::Trim) < 6_000.0, "the trim tank must actually have drained some mass forward");
        assert!(after > before + 0.01, "trim transfer must move the published fuel CG forward as it drains: {before} -> {after} ft");
    }

    /// A tank cannot be loaded, or transferred into, past its own
    /// structural capacity, and never holds negative fuel -- the two flags
    /// this pass's own bullet 4 raised against `load_tank`. Reverting
    /// `load_tank`'s capacity clamp lets the first assertion see roughly
    /// ten times the tank's real capacity.
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

        // A transfer must not push a near-full destination past capacity
        // either: fill the trim tank (source) and Feed1 (destination, one
        // kilogram short of full) and run real transfers for a while.
        let trim_capacity_kg = TankShape::of(Tank::Trim).capacity_m3() * crate::physics::fluids::jet_a_density_kg_m3(10.0);
        live.load_tank(Tank::Trim, trim_capacity_kg, 10.0);
        live.load_tank(Tank::Feed1, capacity_kg - 1.0, 10.0);
        let mut truth = Truth::default();
        truth.dt_s = 1.0;
        for _ in 0..50 {
            live.tick(&truth, &Faults::default());
        }
        // A generous relative slack (0.1% of capacity, ~22 kg here) rather
        // than a tight absolute one: each tick's own transfer was capped
        // against *that tick's* density-based capacity (`move_fuel`'s own
        // doc), and fifty ticks of ordinary thermal drift between the fill
        // temperature and the ambient skin can move the *current* capacity
        // bar by a similar small amount by the time this reads it back --
        // the same tolerance `fill_fraction`'s own `.clamp(0.0, 1.0)`
        // already accepts. A reverted capacity cap would overshoot by
        // orders of magnitude more than this, not a fraction of a percent.
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
}
