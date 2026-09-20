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
//! ## What is not in `Truth` yet
//!
//! Five inputs this system genuinely needs have no field in [`Truth`]:
//! how much fuel is in each tank, how fast each engine is burning it, the
//! aircraft's pitch/bank and sustained accelerations, and whether the crew
//! has selected jettison or cross-feed. They are collected in
//! [`FuelCommands`] as one explicit, documented block rather than invented
//! from what is available: `Truth::engine_n1_frac` is a fan speed, not a
//! fuel flow, and deriving one from the other here would be a second,
//! disagreeing engine model. The defaults are an aircraft that has not been
//! fuelled and is not burning anything -- a real state, not a placeholder
//! quantity.

use crate::deep::api::{failure_id, Area, Registry};
use crate::deep::live::{Faults, Truth};

use super::cg_transfer::{self, TransferFaultDetector, TransferFaults};
use super::gauging::{self, ProbeFault};
use super::geometry::{self, Tank, TankShape, ALL_TANKS};
use super::jettison::{self, JettisonValve, NOMINAL_JETTISON_PUMP_RISE_PA, NOMINAL_NOZZLE_CDA_M2};
use super::leak::{self, LeakDetector};
use super::thermal::{self, FuelType};

const ATA: u16 = 28;
const N_TANKS: usize = 11;
const N_ENGINES: usize = 4;

/// Jet A-1 density at 15 C, kg/m^3. ASTM D1655 / DEF STAN 91-091 allow
/// 775..840; 804 is the standard reference figure used for the
/// volume-to-mass conversion in flight planning, and is what the FQMS's
/// densitometer is compared against when it fails
/// (`gauging::indicated_mass_kg`'s `default_density_kg_m3`).
const REFERENCE_DENSITY_15C_KG_M3: f64 = 804.0;
/// Volumetric thermal expansion coefficient of kerosene, 1/K -- the same
/// ~9e-4 /K figure `gauging.rs`'s own doc cites from
/// `physics::fluids::jet_a_density_kg_m3`, restated here rather than called
/// so this directory stays self-contained (`mod.rs`).
const FUEL_THERMAL_EXPANSION_PER_K: f64 = 9.0e-4;
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
/// shape of the algorithm is the documented real principle. 60 s windows
/// confirmed over 3 of them put the alert about three minutes behind a leak
/// big enough to matter, and 100 kg over a minute (1.7 kg/s) is far above
/// any gauging noise while being well under the smallest leak worth
/// annunciating.
const LEAK_WINDOW_S: f64 = 60.0;
const LEAK_THRESHOLD_KG: f64 = 100.0;
const LEAK_CONFIRM_WINDOWS: u32 = 3;

/// Reference transfer rate the three shortfall detectors compare against,
/// kg/s. Only the *ratio* of achieved to required reaches
/// `cg_transfer::TransferFaultDetector::update` (it tests
/// `achieved < required * (1 - tolerance)`), and
/// `cg_transfer::achieved_transfer_rate_kg_s` is linear in its nominal
/// rate, so this value cancels exactly out of every detection decision: it
/// is a scale, not a claim about how fast the real A380 transfers fuel. No
/// mass is moved between tanks on it -- see this module's own PROGRESS
/// note.
const TRANSFER_REFERENCE_RATE_KG_S: f64 = 1.0;
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
    /// Per engine, the fuel actually being burned, kg/s. `Truth` carries
    /// `engine_n1_frac`, which is a fan speed; turning that into a fuel
    /// flow is the engine model's job, and doing it here would be a second
    /// engine model that disagreed with it.
    pub engine_fuel_flow_kg_s: [f64; N_ENGINES],
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
    /// The jettison selector, and the cross-feed selector the FUEL LEAK
    /// procedure's own `CROSSFEED ... OFF` line reads back.
    pub jettison_selected: bool,
    pub crossfeed_selected: bool,
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

    /// Density of this tank's fuel at its current bulk temperature, from
    /// the reference density and kerosene's own volumetric expansion.
    fn density_kg_m3(&self) -> f64 {
        (REFERENCE_DENSITY_15C_KG_M3 / (1.0 + FUEL_THERMAL_EXPANSION_PER_K * (self.temp_c - 15.0))).max(1.0)
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
    leak_detector: LeakDetector,
    leak_detected: bool,
    total_leak_kg_s: f64,
    baffle_damage_detected: bool,
    fqms_low_confidence: bool,
    fob_lo_temp: bool,
    filter_ice_detected: bool,
    indicated_fob_kg: f64,
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
        Self {
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
            leak_detector: LeakDetector::new(),
            leak_detected: false,
            total_leak_kg_s: 0.0,
            baffle_damage_detected: false,
            fqms_low_confidence: false,
            fob_lo_temp: false,
            filter_ice_detected: false,
            indicated_fob_kg: 0.0,
            commands: FuelCommands::default(),
            fuel_type: FuelType::JetA1,
        }
    }

    /// Put `kg` of fuel at `temp_c` into one tank: refuelling, or the
    /// plugin synchronising this system with the aircraft's real fuel load
    /// once `Truth` carries it.
    pub fn load_tank(&mut self, tank: Tank, kg: f64, temp_c: f64) {
        let i = ALL_TANKS.iter().position(|&t| t == tank).expect("every tank is in ALL_TANKS");
        self.tanks[i].mass_kg = kg.max(0.0);
        self.tanks[i].temp_c = temp_c;
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
        for eng in 0..N_ENGINES {
            let idx = Self::feed_tank_index(eng);
            let burn = self.commands.engine_fuel_flow_kg_s[eng].max(0.0) * dt;
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
        self.tick_transfers(faults, dt, gallery_leak_fraction);

        // ---- Jettison ----------------------------------------------------
        self.tick_jettison(truth, faults, dt);

        // ---- Leak detection ----------------------------------------------
        let metered_flow = self.commands.engine_fuel_flow_kg_s.iter().map(|f| f.max(0.0)).sum::<f64>() + self.commands.apu_fuel_flow_kg_s.max(0.0);
        self.leak_detected = self.leak_detector.update(self.indicated_fob_kg, metered_flow, dt, LEAK_WINDOW_S, LEAK_THRESHOLD_KG, LEAK_CONFIRM_WINDOWS);
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        let b = |x: bool| if x { 1.0 } else { 0.0 };

        // Every variable this area's `registry.rs` names in an ECAM
        // trigger.
        out("FUEL_LEAK_DETECTED", b(self.leak_detected));
        out("FUEL_CROSSFEED_OPEN", b(self.commands.crossfeed_selected));
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
        }
        for side in 0..2 {
            let n = side + 1;
            out(&format!("FUEL_JETTISON_VALVE_POSITION:{n}"), self.jettison_valves[side].position);
            out(&format!("FUEL_JETTISON_FLOW_KG_S:{n}"), self.jettison_flow_kg_s[side]);
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

    /// The transfer paths: each one's `TransferFaults` from the failures
    /// registered against its own valves and pumps, then the shortfall
    /// detector `registry.rs` wires to its ECAM alert.
    fn tick_transfers(&mut self, faults: &Faults, dt: f64, gallery_leak_fraction: f64) {
        let worst = |ids: &[u64]| ids.iter().map(|&id| faults.get(id)).fold(0.0f64, f64::max);

        // Trim transfer: the two trim pumps in parallel (both must degrade
        // for the path to lose flow), in series with the inlet valves and
        // the line isolation valves.
        let trim_pump_loss = self.ids.trim_pump.iter().map(|&id| faults.get(id)).fold(f64::INFINITY, f64::min);
        let trim = TransferFaults {
            valve_stuck_fraction: worst(&self.ids.trim_inlet).min(1.0).max(worst(&self.ids.trim_iso)),
            pump_degradation_fraction: if trim_pump_loss.is_finite() { trim_pump_loss } else { 0.0 },
            gallery_leak_fraction,
        };
        let achieved = cg_transfer::achieved_transfer_rate_kg_s(TRANSFER_REFERENCE_RATE_KG_S, &trim);
        self.trim_fault = self.trim_detector.update(TRANSFER_REFERENCE_RATE_KG_S, achieved, TRANSFER_TOLERANCE, 1.0, dt);

        // Automatic CG control through the outer/inner/mid transfer valves.
        // The sequence is only *required* while the outer tanks are still
        // being retained or the inners are still feeding the feed tanks --
        // which, with any fuel on board at all, is the whole flight.
        let inner_fill = self.fill_of(Tank::LeftInner).max(self.fill_of(Tank::RightInner));
        let mid_fill = self.fill_of(Tank::LeftMid).max(self.fill_of(Tank::RightMid));
        let cg_required = self.true_fob_kg() > 0.0 || cg_transfer::outer_tank_retention_active(inner_fill, mid_fill, OUTER_RETENTION_UNTIL_FRACTION);
        let cg = TransferFaults {
            valve_stuck_fraction: worst(&self.ids.outer_xfer).max(worst(&self.ids.inner_xfer)).max(worst(&self.ids.mid_xfer)),
            pump_degradation_fraction: 0.0,
            gallery_leak_fraction,
        };
        let cg_required_rate = if cg_required { TRANSFER_REFERENCE_RATE_KG_S } else { 0.0 };
        let cg_achieved = cg_transfer::achieved_transfer_rate_kg_s(cg_required_rate, &cg);
        self.cg_fault = self.cg_detector.update(cg_required_rate, cg_achieved, TRANSFER_TOLERANCE, 5.0, dt);

        // Wing cross-feed: required either because the crew selected it or
        // because the wings are genuinely out of balance.
        let (left, right) = self.wing_masses_kg();
        let xfeed_required = self.commands.crossfeed_selected || cg_transfer::wing_balance_transfer_needed(left, right, WING_IMBALANCE_LIMIT_KG);
        let xfeed = TransferFaults { valve_stuck_fraction: worst(&self.ids.crossfeed), pump_degradation_fraction: 0.0, gallery_leak_fraction };
        let xfeed_required_rate = if xfeed_required { TRANSFER_REFERENCE_RATE_KG_S } else { 0.0 };
        let xfeed_achieved = cg_transfer::achieved_transfer_rate_kg_s(xfeed_required_rate, &xfeed);
        self.crossfeed_fault = self.crossfeed_detector.update(xfeed_required_rate, xfeed_achieved, TRANSFER_TOLERANCE, 2.0, dt);
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
        let commanded = self.commands.jettison_selected;

        // Jettison draws from the wing tanks, left nozzle from the left
        // wing and right from the right: the deepest tank on that side
        // sets the head at the nozzle and is the one that drains.
        const LEFT_GROUP: [Tank; 4] = [Tank::LeftInner, Tank::LeftMid, Tank::LeftOuter, Tank::Feed1];
        const RIGHT_GROUP: [Tank; 4] = [Tank::RightInner, Tank::RightMid, Tank::RightOuter, Tank::Feed4];

        for side in 0..2 {
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
    use crate::deep::live::Area as _;
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

    #[test]
    fn a_stuck_crossfeed_valve_raises_the_wing_crossfeed_fault_its_registry_entry_promises() {
        // registry.rs: "wing-balance cross-feed cannot move fuel between
        // wings through this valve".
        let mut live = FuelLive::new();
        full_tanks(&mut live, 10.0);
        live.commands.crossfeed_selected = true;
        let id = live.ids.crossfeed[0];

        let healthy = run(&mut live, &Truth::default(), &Faults::default(), 5.0);
        assert_eq!(healthy.get("FUEL_CROSSFEED_FAULT"), Some(&0.0));

        let mut live = FuelLive::new();
        full_tanks(&mut live, 10.0);
        live.commands.crossfeed_selected = true;
        let faulted = run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 1.0)]), 5.0);
        assert_eq!(faulted.get("FUEL_CROSSFEED_FAULT"), Some(&1.0), "a seized cross-feed valve must raise FUEL WING XFEED FAULT");
        assert_eq!(faulted.get("FUEL_CROSSFEED_OPEN"), Some(&1.0));
    }

    #[test]
    fn a_blocked_jettison_nozzle_cuts_the_rate_and_raises_the_jettison_fault() {
        // registry.rs: "jettison rate through that nozzle falls in
        // proportion, lengthening the time needed to reach max landing
        // weight".
        let truth = Truth::default();
        let mut clear = FuelLive::new();
        full_tanks(&mut clear, 10.0);
        clear.commands.jettison_selected = true;
        let nozzle = clear.ids.jettison_nozzle[0];
        let clear_out = run(&mut clear, &truth, &Faults::default(), 20.0);

        let mut blocked = FuelLive::new();
        full_tanks(&mut blocked, 10.0);
        blocked.commands.jettison_selected = true;
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
        sea_level.commands.jettison_selected = true;
        let low = run(&mut sea_level, &Truth::default(), &Faults::default(), 20.0);

        let mut cruise = FuelLive::new();
        full_tanks(&mut cruise, 10.0);
        cruise.commands.jettison_selected = true;
        let mut truth = Truth::default();
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
}
