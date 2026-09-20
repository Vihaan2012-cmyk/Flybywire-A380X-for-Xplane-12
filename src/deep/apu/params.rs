//! Cited and GENERIC parameters for the PW980A-class APU deep model.
//!
//! Two facts about this exact machine are public:
//! - It is a two-shaft gas turbine: a gas-generator core (compressor,
//!   combustor, turbine) drives, through its hot gas stream, a free/power
//!   section that turns the accessory gearbox -- the two generators and a
//!   *separate* load (customer bleed) compressor ("PW980A ... two-shaft gas
//!   turbine engine", "low spool-driven load compressor",
//!   aeroexpo.online/Pratt & Whitney PW980 product page, 2024; RTX/P&W PW980
//!   press materials).
//! - Its rated output class is >1800 hp (1.342 MW) (same sources).
//!
//! This task's brief asks specifically for a "single-spool gas path" (the
//! gas-generator core) "with a power section compressor and a separate load
//! compressor (bleed)". Folding the free/power section into the one modelled
//! spool, rather than giving it a second, much lighter rotor, is the same
//! reduced-order choice documented in FlyByWire's own
//! `apu/pw980_physics.rs` module docs (that rotor is an order of magnitude
//! lighter than the gas-generator core, so on the timescales that matter for
//! a real-time sim it can be treated as rigidly geared to the core rather
//! than solved as a second, numerically stiff torque balance) -- this file's
//! numbers are, however, derived independently of that file's, from the two
//! public facts above plus public generic-turbomachinery literature for
//! this class of small single-spool APU gas generator, not read from or
//! fitted to FlyByWire's own file.
//!
//! No public component-level map, temperature or geometry data exists for
//! this exact APU. Every `GENERIC` value below says what class of machine
//! (or what public comparator) it is drawn from and why; `power_section.rs`
//! then *calibrates* the one or two genuinely free scale parameters (fuel
//! flow at the design point, turbine flow capacity) against the design
//! shapes chosen here, rather than asserting an arbitrary absolute number.

// ---- Rated/public ----------------------------------------------------

/// Rated shaft power class, one source of the two public numbers this
/// machine has (see module docs). Used only as a plausibility check in this
/// file's tests, not as a value fed directly into the cycle -- the cycle's
/// own combustor/turbine energy balance is what produces shaft power here.
pub const RATED_SHAFT_POWER_W: f64 = 1_342_000.0;

// ---- GENERIC: gas-generator spool -------------------------------------

/// GENERIC: no published PW980A gas-generator rpm exists. 60,000 rpm is
/// representative of single centrifugal-stage gas-generator spools in this
/// power class (public comparators: Honeywell 131-9A ~60,000 rpm N1;
/// Honeywell/AiResearch GTCP331 ~60,000-65,000 rpm) -- chosen independently
/// of, but consistent with, the same public comparator class FlyByWire's own
/// file cites for the same reason.
pub const N_DESIGN_RPM: f64 = 60_000.0;

/// GENERIC: order-of-magnitude rotor inertia (compressor + turbine + shaft)
/// for a gas-generator spool of this power class -- comparable machines in
/// this class have a similarly small, light single-stage rotor assembly.
pub const ROTOR_INERTIA_KG_M2: f64 = 0.028;

/// Real APUs are governed to one constant frequency-critical speed
/// regardless of load (unlike a main engine's throttle-set speed): the two
/// 120 kVA generators need a constant shaft speed to hold 400 Hz. This is
/// the governed setpoint, ATA-generic "100%" convention.
pub const GOVERNED_N_PERCENT: f64 = 100.0;

/// ATA 49 generic overspeed protection trip point, a few percent above
/// governed speed (no PW980A-specific figure is public).
pub const OVERSPEED_TRIP_PERCENT: f64 = 105.0;

/// Gas-generator light-off speed: below this the spool is only being
/// motored by the starter, no combustion torque yet. Typical small gas
/// turbine light-off speeds are 8-15% N (public, generic to this class of
/// machine).
pub const LIGHT_OFF_N_PERCENT: f64 = 12.0;

/// Speed above which the starter's overrunning clutch has disengaged and
/// the gas generator is considered self-sustaining without starter
/// assistance (generic small-APU convention, comparable to FlyByWire's own
/// cited ~50-54% figure for the same class of machine, derived here
/// independently from the starter torque balance in `starter.rs` rather than
/// asserted).
pub const SELF_SUSTAINING_N_PERCENT: f64 = 55.0;

// ---- GENERIC: power-section (core) compressor -------------------------

/// GENERIC: single centrifugal-stage gas-generator pressure ratio, typical
/// of this APU power class (3.5-5:1 in gas turbine literature for
/// single-stage centrifugal APU cores).
pub const CORE_PRESSURE_RATIO_DESIGN: f64 = 4.2;
/// GENERIC: typical small centrifugal-compressor isentropic efficiency
/// (Cohen, Rogers & Saravanamuttoo, *Gas Turbine Theory*: typical 0.75-0.82
/// for this size class).
pub const CORE_COMPRESSOR_EFFICIENCY_DESIGN: f64 = 0.78;
/// GENERIC: design corrected core airflow. Public comparators of this power
/// class (Honeywell 131-9B ~3.2 kg/s, GTCP331 ~3.6-4.5 kg/s core flow) put a
/// PW980A-class core in the same 3-5 kg/s band.
pub const CORE_MDOT_DESIGN_KG_S: f64 = 4.0;
/// GENERIC efficiency-island curvature (how sharply efficiency falls off
/// away from design speed), same generic shape convention as
/// `physics/engine/compressor.rs`'s own `efficiency_falloff`.
pub const CORE_COMPRESSOR_EFFICIENCY_FALLOFF: f64 = 0.55;
/// GENERIC minimum stable corrected flow at design speed, as a fraction of
/// design corrected flow (a 12% surge margin at the design point is a
/// typical target for a single-stage centrifugal compressor design,
/// Saravanamuttoo et al.).
pub const CORE_SURGE_MARGIN_DESIGN_FRAC: f64 = 0.12;
/// GENERIC surge-line shape: how much flatter than the nominal N^1
/// corrected-flow operating line the surge line runs (0 = same shape,
/// constant fractional margin at every speed; 1 = surge line barely moves
/// with speed at all). Published generic centrifugal compressor
/// characteristics (e.g. Cohen/Rogers/Saravanamuttoo fig. 5.9) show the
/// surge line diverging from the working lines at part speed, narrowing the
/// available margin -- this is the generic shape parameter for that.
pub const CORE_SURGE_LINE_FLATNESS: f64 = 0.5;
/// GENERIC choke corrected flow as a multiple of the design corrected flow.
pub const CORE_CHOKE_FLOW_MULTIPLE: f64 = 1.3;

// ---- GENERIC: load (customer bleed) compressor -------------------------

/// GENERIC: bleed manifold pressure of ~40-45 psia against ~14.7 psia sea
/// level ambient implies a load-compressor pressure ratio around 2.7-3.1;
/// 3.0 is used (no PW980A-specific figure is public).
pub const LOAD_PRESSURE_RATIO_DESIGN: f64 = 3.0;
/// GENERIC: typical radial-inflow load-compressor isentropic efficiency for
/// customer bleed air, this size class.
pub const LOAD_COMPRESSOR_EFFICIENCY_DESIGN: f64 = 0.75;
/// GENERIC: design bleed mass-flow capacity. A defensible order-of-magnitude
/// figure for an APU rated to supply full packs plus engine start on a
/// widebody this size (no PW980A-specific figure is public).
pub const LOAD_MDOT_DESIGN_KG_S: f64 = 1.2;
pub const LOAD_COMPRESSOR_EFFICIENCY_FALLOFF: f64 = 0.6;
/// GENERIC: a load/bleed compressor with inlet guide vanes and a surge
/// control valve is normally kept with more surge margin in reserve than an
/// unprotected core compressor, since its downstream demand (aircraft bleed
/// off-take) can vary far more abruptly than a governed core's own airflow
/// -- 18% vs. the core's 12%, still GENERIC.
pub const LOAD_SURGE_MARGIN_DESIGN_FRAC: f64 = 0.18;
pub const LOAD_SURGE_LINE_FLATNESS: f64 = 0.5;
pub const LOAD_CHOKE_FLOW_MULTIPLE: f64 = 1.3;

/// GENERIC: inlet guide vane actuator full-stroke time, typical of a small
/// pneumatic/electric metering vane actuator (no PW980A-specific figure is
/// public).
pub const IGV_FULL_TRAVEL_RATE_PER_S: f64 = 0.5;
/// GENERIC: the surge/anti-surge control valve is a fast-acting protection
/// device by design (it exists to react before the compressor reaches its
/// surge line), so it is given a shorter full-stroke time than the IGVs.
pub const SCV_FULL_TRAVEL_RATE_PER_S: f64 = 1.0;
/// GENERIC: the surge control valve's own target margin above the surge
/// line -- a control-law design choice (kept clear of the line with
/// headroom to spare), not a measured figure.
pub const SCV_SURGE_SAFETY_MARGIN_FRAC: f64 = 0.25;

// ---- GENERIC: combustor and turbine ------------------------------------

/// Modern annular combustor efficiency (Gas Turbine Theory, typical >0.98 at
/// max power). Public, generic to kerosene/air annular combustors.
pub const COMBUSTOR_EFFICIENCY: f64 = 0.98;
/// Typical total-pressure loss fraction across an annular combustor
/// (Mattingly: 3-6% typical range).
pub const COMBUSTOR_PRESSURE_LOSS_FRAC: f64 = 0.04;
/// Jet A/A-1 lower heating value (ASTM D1655 / CRC Report No. 635,
/// *Handbook of Aviation Fuel Properties*): 42.8-43.5 MJ/kg typical; the
/// mid-range figure most gas turbine texts quote.
pub const LHV_JET_A_J_KG: f64 = 43.1e6;

/// GENERIC: design turbine-inlet (combustor exit) temperature. Public
/// comparators of this small-APU class typically quote turbine inlet
/// temperatures in the 800-950 degC band at rated power; 877 degC
/// (1150.15 K) is used as a representative mid-range design point.
pub const T4_DESIGN_K: f64 = 1150.15;
/// GENERIC: single-stage axial/radial-inflow turbine isentropic efficiency,
/// typical of this size class (Gas Turbine Theory).
pub const TURBINE_EFFICIENCY_DESIGN: f64 = 0.82;
pub const TURBINE_EFFICIENCY_FALLOFF: f64 = 0.45;
/// GENERIC: turbine design pressure ratio, taken as the core compressor's
/// design pressure ratio net of the combustor's pressure loss (the gas
/// generator's whole pressure rise is available to the turbine, since the
/// APU's own exhaust is close to ambient).
pub fn turbine_pressure_ratio_design() -> f64 {
    CORE_PRESSURE_RATIO_DESIGN * (1.0 - COMBUSTOR_PRESSURE_LOSS_FRAC)
}

/// Upper bound the fuel metering valve can physically pass, set a margin
/// above the design-point fuel flow `power_section::design_point()`
/// computes, so the governor and the acceleration/EGT-limit schedule have
/// headroom for a real start's richer transient fuelling without being able
/// to command literally unbounded fuel.
pub const MAX_FUEL_FLOW_MARGIN: f64 = 1.6;

// ---- EGT limits: mostly sourced to FlyByWire's own PW980A model --------
//
// These were written as a "generic ATA 49 family of figures". They are not
// generic: FlyByWire model this exact APU and carry A380-specific values,
// which this section now cites and matches. Note the distinction the real
// ECB makes and this file now makes too:
//   * a *control* limit -- what the fuel schedule holds EGT below, so the
//     APU never normally annunciates at all;
//   * a *warning/caution* temperature -- what the ECAM/SD APU page draws
//     its red line and amber band at;
//   * a *protective trip* -- what shuts the APU down.
// Conflating them is how a model ends up annunciating in normal operation.

/// Transient start EGT limit the fuel schedule holds to while starting,
/// deg C. **Sourced**: FlyByWire's PW980A physics uses exactly this as its
/// own start fuel-limit anchor -- `const START_EGT_LIMIT_K: f64 = 1173.15;
/// // 900 deg C` (fbw-common/src/wasm/systems/systems/src/apu/
/// pw980_physics.rs:547), commented there as following the ECB's own
/// `calculate_egt_warning_temperature`.
pub const EGT_START_LIMIT_C: f64 = 900.0;
/// Continuous running EGT *control* limit, deg C: what the governor's fuel
/// schedule holds EGT below once self-sustaining.
///
/// **GENERIC, and deliberately well below the 900 deg C running warning**
/// ([`EGT_RUNNING_WARNING_C`]): a control limit set at the warning would
/// let the APU sit on its own red line in normal operation. No published
/// PW980A continuous EGT control limit exists (FlyByWire model the warning,
/// not the control schedule), so the margin below the warning -- 150 deg C
/// here -- is the generic part. Searched: FlyByWire's `pw980.rs` /
/// `pw980_physics.rs` / `electronic_control_box.rs`, and public A380 ATA 49
/// material.
pub const EGT_RUNNING_LIMIT_C: f64 = 750.0;
/// Hard over-temperature protective trip, deg C: the ECB shuts the APU
/// down above this. **Sourced**: FlyByWire's PW980A physics trips at
/// exactly this temperature -- `if self.egt.get::<degree_celsius>() >
/// 950.0` (fbw-common/src/wasm/systems/systems/src/apu/pw980_physics.rs:664).
pub const EGT_TRIP_C: f64 = 950.0;

/// Running EGT **warning** (red) temperature for the PW980A, deg C.
/// **Sourced, and A380-specific**: FlyByWire's `Pw980Constants` sets
/// `const RUNNING_WARNING_EGT: f64 = 900.; // Deg C`
/// (fbw-common/src/wasm/systems/systems/src/apu/pw980.rs:32). Worth noting
/// how aircraft-specific this is: the same trait on the A320's APS3200 sets
/// it to 682 deg C (aps3200.rs:28), so it is not a family figure that could
/// have been guessed.
pub const EGT_RUNNING_WARNING_C: f64 = 900.0;
/// Starting EGT warning temperature below FL250, deg C. **Sourced**:
/// `const STARTING_WARNING_EGT_BELOW_25000_FEET: f64 = 900.;`
/// (fbw-common/.../apu/electronic_control_box.rs:276).
pub const EGT_START_WARNING_BELOW_FL250_C: f64 = 900.0;
/// Starting EGT warning temperature at or above FL250, deg C -- the ECB
/// raises the start warning with altitude because thinner air cools the
/// turbine less. **Sourced**: `const
/// STARTING_WARNING_EGT_AT_OR_ABOVE_25000_FEET: f64 = 982.;`
/// (fbw-common/.../apu/electronic_control_box.rs:277).
pub const EGT_START_WARNING_AT_OR_ABOVE_FL250_C: f64 = 982.0;
/// How far below the warning the **caution** (amber) temperature sits,
/// deg C. **Sourced**: `const WARNING_TO_CAUTION_DIFFERENCE: f64 = 33.;`
/// in `ElectronicControlBox::egt_caution_temperature`
/// (fbw-common/.../apu/electronic_control_box.rs:341-344).
pub const EGT_WARNING_TO_CAUTION_DIFFERENCE_C: f64 = 33.0;

/// Pressure altitude at which the ECB switches between the two start
/// warning temperatures, ft. FlyByWire express the same switch as an inlet
/// pressure threshold of 5.45 psi ("fl250_isa_pressure",
/// electronic_control_box.rs:279), i.e. ISA pressure at FL250.
pub const EGT_START_WARNING_ALTITUDE_SWITCH_FT: f64 = 25_000.0;

/// The EGT warning (red) temperature the ECAM/SD APU page should show,
/// given whether the APU is still starting and the current pressure
/// altitude. Follows `ElectronicControlBox::calculate_egt_warning_temperature`
/// (fbw-common/.../apu/electronic_control_box.rs:267-292): the altitude
/// split applies only while starting; shutdown, running and stopping all
/// use the running warning.
pub fn egt_warning_c(starting: bool, pressure_altitude_ft: f64) -> f64 {
    if starting && pressure_altitude_ft >= EGT_START_WARNING_ALTITUDE_SWITCH_FT {
        EGT_START_WARNING_AT_OR_ABOVE_FL250_C
    } else if starting {
        EGT_START_WARNING_BELOW_FL250_C
    } else {
        EGT_RUNNING_WARNING_C
    }
}

/// The EGT caution (amber) temperature that goes with [`egt_warning_c`].
pub fn egt_caution_c(starting: bool, pressure_altitude_ft: f64) -> f64 {
    egt_warning_c(starting, pressure_altitude_ft) - EGT_WARNING_TO_CAUTION_DIFFERENCE_C
}

// ---- GENERIC: oil system ------------------------------------------------

pub const OIL_REGULATED_PRESSURE_PSI: f64 = 60.0;
pub const OIL_REGULATION_N_PERCENT: f64 = 50.0;
pub const OIL_PRESSURE_TRIP_PSI: f64 = 15.0;
pub const OIL_PRESSURE_TRIP_DEBOUNCE_S: f64 = 5.0;
/// GENERIC: oil charge and cooling time constant, typical accessory-gearbox
/// scale for this size of gas turbine (no PW980A-specific figures are
/// public).
pub const OIL_HEAT_CAPACITY_J_K: f64 = 9_000.0;
pub const OIL_TIME_CONSTANT_S: f64 = 150.0;
pub const OIL_TANK_CAPACITY_L: f64 = 8.0;
pub const OIL_LOW_LEVEL_TRIP_L: f64 = 2.0;

// ---- GENERIC: starter/battery -------------------------------------------

/// Aircraft main battery nominal open-circuit voltage. Standard aviation
/// NiCd main battery convention (24 V nominal), generic to the class of
/// aircraft, not a published A380-specific figure.
pub const BATTERY_NOMINAL_OPEN_CIRCUIT_V: f64 = 24.0;
/// GENERIC: battery internal resistance, chosen so the starter's realistic
/// inrush current (calibrated in `starter.rs`) sags the bus a plausible few
/// volts, the well documented behaviour of a loaded aircraft battery.
pub const BATTERY_INTERNAL_RESISTANCE_OHM: f64 = 0.018;
/// GENERIC: series-wound DC starter motor's armature resistance, chosen
/// (with the battery resistance above and `STARTER_KE_KT` below) so a
/// stalled-rotor inrush current in the several-hundred-amp class (typical
/// for an aircraft starter motor this size) results, and so the starter
/// alone -- see the derivation below -- has enough torque margin over the
/// core's own compressor drag to reliably motor the core through
/// `LIGHT_OFF_N_PERCENT`.
pub const STARTER_ARMATURE_RESISTANCE_OHM: f64 = 0.022;
/// GENERIC: starter back-EMF/torque constant. A series motor's torque at a
/// given speed, `torque(omega) = Ke*(V - Ke*omega)/R_total`, is maximised
/// over `Ke` at `Ke = V / (2*omega)`, giving `torque_max(omega) =
/// V^2/(4*omega*R_total)`; this Ke is chosen close to that optimum evaluated
/// at `LIGHT_OFF_N_PERCENT`'s own angular speed, so the starter is putting
/// out close to the most torque a motor with this resistance budget *can*
/// produce right where it matters most -- clearing light-off -- rather than
/// an arbitrary value that (as an earlier calibration pass here found, by
/// working through `compressor_map.rs`'s own n^2.8 power-vs-speed scaling
/// applied at this spool's rated speed and rotor inertia) let the core's own
/// compressor drag exceed starter torque *before* reaching light-off,
/// stalling every start. With this Ke and the resistances above, hand
/// calculation from this file's own compressor/turbine equations
/// (`power_section.rs`, `compressor_map.rs`) gives starter torque at
/// `LIGHT_OFF_N_PERCENT` (~12%, ~754 rad/s) of ~4.8 N*m against a
/// compressor-drag torque there of ~2.6 N*m (a ~1.8x margin); starter torque
/// alone falls behind compressor drag again somewhere around 13-15% N
/// (unsurprising -- a starter motor is not sized to motor a gas generator
/// all the way to a governed speed on its own), but by then combustion has
/// already begun and the turbine's own torque (on the order of tens of
/// N*m even a few percent past light-off, since a lit core's shaft output
/// grows far faster with speed than its own compressor drag does) takes
/// over the acceleration completely -- see `starter.rs`'s and `apu.rs`'s
/// own full-start tests.
pub const STARTER_KE_KT: f64 = 0.016;

// ---- GENERIC: generators --------------------------------------------

/// Public: FlyByWire's own PW980A model cites "two 120 kVA generators"
/// (`apu/pw980_physics.rs` module docs, itself citing P&W/RTX PW980
/// materials) -- a public fact about this exact machine, not a value read
/// from FBW's file.
pub const GENERATOR_RATED_APPARENT_VA: f64 = 120_000.0;
/// GENERIC: typical aircraft generator design power factor.
pub const GENERATOR_RATED_POWER_FACTOR: f64 = 0.8;
/// GENERIC: typical brushless aircraft generator electro-mechanical
/// efficiency.
pub const GENERATOR_EFFICIENCY_DESIGN: f64 = 0.88;

// ---- GENERIC: fuel control and inlet door -----------------------------

/// GENERIC: fast solenoid-actuated metering valve full-stroke time.
pub const FUEL_METERING_VALVE_RATE_PER_S: f64 = 2.0;
/// GENERIC: inlet door actuator full-stroke time. FlyByWire's own file
/// documents a 12 s maximum travel time for the equivalent door on this
/// aircraft type as a *test-harness* bound, not sourced to a manufacturer
/// figure there either; this value is chosen independently as a plausible
/// order of magnitude for a small electrically-actuated APU inlet door.
pub const INLET_DOOR_RATE_PER_S: f64 = 0.1;

// ---- GENERIC: sensor faults and fixed accessory load -------------------

/// GENERIC: a fixed shaft power for accessories not otherwise individually
/// modelled (oil pump drive, gearbox windage) -- small relative to rated
/// shaft power, order-of-magnitude typical of gearbox parasitic losses.
pub const FIXED_ACCESSORY_POWER_W: f64 = 3_000.0;
/// GENERIC: how far a fully-failed speed sensor can under-read the true
/// spool speed -- large enough that the governor's resulting overfuelling
/// is a genuine, testable excursion toward the physical overspeed
/// protection, not a negligible offset.
pub const N_SENSOR_MAX_BIAS_PERCENT: f64 = 20.0;
/// GENERIC: how far a fully-failed EGT sensor can under-read the true
/// turbine-exit temperature.
pub const EGT_SENSOR_MAX_BIAS_C: f64 = 150.0;

// ---- GENERIC: start envelope (altitude/temperature/airspeed) ----------

/// GENERIC: minimum ambient density ratio (`gas::density_ratio`) at which
/// relight is certified. Real APU relight/start envelopes are commonly
/// bounded by a pressure-altitude ceiling in the 20,000-25,000 ft class
/// (public knowledge across several small-APU types' AFM/QRH relight
/// envelopes); 20,000 ft ISA has a density ratio of about 0.53, which is
/// used as the generic cutoff -- below it, combustion cannot be reliably
/// sustained at the airflows this size of combustor/turbine was sized for,
/// so `starter.rs` refuses light-off regardless of cranking speed.
pub const MIN_RELIGHT_DENSITY_RATIO: f64 = 0.53;
/// GENERIC: how strongly the APU's tailcone inlet scoop couples to
/// freestream Mach number for ram effects. A scoop inlet (this class of
/// APU) is not a forward-facing podded-engine inlet, so it recovers only a
/// fraction of the freestream dynamic pressure/ram heating a true axial
/// inlet would -- chosen as a fraction well below 1 to reflect that, not a
/// measured figure.
pub const SCOOP_RAM_COUPLING_FRAC: f64 = 0.3;
/// GENERIC: windmilling torque coefficient, N*m per Pa of (scoop-coupled)
/// dynamic pressure at zero spool speed, chosen so windmilling settles in
/// the low tens-of-percent N band at high-subsonic cruise dynamic pressure
/// against this core's own compressor drag (`compressor_map.rs`'s n^2.8
/// scaling) -- a real, modest effect (this is a scoop, not a podded engine
/// inlet), not the main driver of an in-flight start.
pub const WINDMILL_TORQUE_COEFF_NM_PER_PA: f64 = 0.02;

// ---- GENERIC: oil viscosity and cold-soak cranking drag ----------------

/// Kinematic viscosity Walther-relation coefficients (ASTM D341), fitted
/// through MIL-PRF-23699 turbine oil's published data points (e.g. Mobil
/// Jet Oil II: 27.6 cSt at 40 degC, 5.1 cSt at 100 degC) -- the same public
/// oil-property data `physics/engine/oil.rs` documents using, restated
/// independently here per this directory's self-containment rule.
pub const OIL_VISCOSITY_WALTHER_A: f64 = 9.3116;
pub const OIL_VISCOSITY_WALTHER_B: f64 = 3.6661;
/// The gas-generator spool's rated angular velocity, rad/s (derived from
/// `N_DESIGN_RPM`, not an independent number).
pub fn omega_rated_rad_s() -> f64 {
    N_DESIGN_RPM * std::f64::consts::TAU / 60.0
}

/// The viscous (bearing and gear shear) part of the accessory drag torque,
/// at rated speed and at the hot reference oil viscosity, N*m.
///
/// Derived, not asserted: it is the whole of `FIXED_ACCESSORY_POWER_W`
/// expressed as a torque at rated speed (3000 W / 6283.2 rad/s =
/// 0.477 N*m). Treating all of that un-itemised accessory power as viscous
/// shear at *hot* viscosity is the conservative reading -- some of it is
/// really the oil pump's own displacement work -- and it means the cold-oil
/// penalty below needs no free constant of its own: it is that same shear,
/// scaled by how much thicker than hot the oil currently is.
pub fn oil_viscous_drag_torque_hot_rated_nm() -> f64 {
    FIXED_ACCESSORY_POWER_W / omega_rated_rad_s()
}

/// Exponent on both the viscosity ratio and the speed ratio in the bearing/
/// gearbox viscous drag term (`oil.rs`'s `cold_drag_torque_nm`).
///
/// Palmgren's standard rolling-bearing no-load friction relation (A.
/// Palmgren, *Ball and Roller Bearing Engineering*, 1959; reproduced in
/// every bearing manufacturer's catalogue, e.g. SKF's `M_0 = f_0 (nu*n)^(2/3)
/// d_m^3 * 1e-7`) makes the viscous term grow as the *two-thirds* power of
/// the viscosity-speed product, not linearly in either. That matters a great
/// deal here: MIL-PRF-23699 oil is ~2400 times more viscous at -40 degC than
/// at this model's hot reference, so a linear-in-viscosity drag term would
/// claim thousands of newton-metres of cranking drag on a cold-soaked start
/// -- two orders of magnitude more than the starter can produce, i.e. no
/// cold start would ever be physically possible, and even a 15 degC ambient
/// start would stall the spool below light-off. The 2/3 power is the
/// published, measured shape.
pub const OIL_VISCOUS_DRAG_EXPONENT: f64 = 2.0 / 3.0;
/// Reference (fully warmed) oil temperature the cold-drag term is measured
/// relative to -- no extra drag once oil is at/above this.
pub const OIL_HOT_REFERENCE_K: f64 = 353.15;

// ---- GENERIC: hot-section life/wear tracking ---------------------------

/// GENERIC: compressor erosion accumulation rate, fractional efficiency
/// loss per operating hour -- an order-of-magnitude figure for particulate
/// erosion in an uncertificated generic small gas-generator compressor (no
/// PW980A-specific erosion-rate data is public); combined with the
/// per-start-cycle term below through `life.rs`.
pub const COMPRESSOR_WEAR_PER_HOUR: f64 = 0.00006;
/// GENERIC: additional compressor erosion per start cycle (thermal cycling
/// and the particulate ingestion transient of a ground/APU-bay start),
/// small relative to the per-hour term but nonzero.
pub const COMPRESSOR_WEAR_PER_START: f64 = 0.00015;
/// GENERIC: turbine baseline wear per operating hour (independent of
/// creep -- erosion/FOD-class wear), same order of magnitude reasoning as
/// the compressor term.
pub const TURBINE_WEAR_PER_HOUR: f64 = 0.00008;
/// GENERIC: hot-section creep rate at the design EGT, per hour spent at or
/// above it -- a simplified Arrhenius-style temperature dependence (creep
/// rate doubles for every `CREEP_DOUBLING_C` degrees above the reference)
/// stands in for a full Larson-Miller parameter calculation, which needs
/// alloy-specific data this task has no public source for; the *shape*
/// (creep accelerates sharply, not linearly, with temperature) is the
/// well-established, generic metallurgical fact this represents.
pub const CREEP_RATE_PER_HOUR_AT_DESIGN_EGT: f64 = 0.00004;
/// The "design EGT" the rate above is quoted at: this model's own design-
/// point turbine-exit temperature (hand-derived from `power_section.rs`'s
/// design-point calibration: T4 1150.15 K expanding to a turbine-exit T5 of
/// ~872.7 K, i.e. ~600 degC), not an externally sourced figure.
pub const CREEP_REFERENCE_EGT_C: f64 = 600.0;
pub const CREEP_DOUBLING_C: f64 = 25.0;
/// GENERIC: below this indicated EGT, creep accumulation is treated as
/// negligible over a maintenance-relevant timescale (well below the design
/// point, where a real nickel superalloy's creep rate is orders of
/// magnitude slower).
pub const CREEP_THRESHOLD_EGT_C: f64 = 500.0;

// ---- GENERIC: starter duty cycle ---------------------------------------

/// GENERIC: cumulative cranking-heat "thermal capacity" of the starter
/// motor, expressed directly in seconds of continuous full-current
/// cranking it can sustain from cold before needing a cool-down -- the same
/// lumped-thermal-model convention `breakers.rs`'s own thermal trips use
/// elsewhere in this crate (a single time-constant heat/cool budget rather
/// than a full motor thermal model), restated independently here. No
/// PW980A-specific duty-cycle figure is public; a small aircraft starter
/// motor sustaining on the order of a minute of continuous full-current
/// cranking before requiring a cool-down is a generic, defensible figure
/// for this class of machine.
pub const STARTER_DUTY_LIMIT_S: f64 = 60.0;
/// GENERIC: cool-down time constant -- how fast the accumulated cranking
/// heat dissipates once the starter is no longer engaged.
pub const STARTER_COOLDOWN_TIME_CONSTANT_S: f64 = 180.0;

// ---- GENERIC: ECB dual-channel sensors ---------------------------------

/// GENERIC: oil pressure sensor's maximum under-read bias when fully
/// failed, same convention as the speed/EGT sensor biases above.
pub const OIL_PRESSURE_SENSOR_MAX_BIAS_PSI: f64 = 20.0;
/// GENERIC: debounce for the ECB's own overspeed protection channel vote
/// (short -- overspeed is fast-acting protection).
pub const ECB_OVERSPEED_DEBOUNCE_S: f64 = 0.2;
/// GENERIC: debounce for the ECB's own EGT protection channel vote.
pub const ECB_EGT_DEBOUNCE_S: f64 = 1.0;

// ---- GENERIC: bleed/generator load priority ----------------------------

/// GENERIC: combined electrical load fraction (of both generators' rated
/// power) above which the IGV schedule begins shedding bleed demand to
/// protect shaft torque margin/surge margin -- a generic load-management
/// threshold, not a measured figure.
pub const IGV_LOAD_SHED_START_FRAC: f64 = 0.7;
/// Electrical load fraction at which the IGV schedule has shed bleed demand
/// down to its floor.
pub const IGV_LOAD_SHED_FULL_FRAC: f64 = 1.1;
/// The IGV demand scaling floor once electrical load has shed it fully --
/// never cut to zero (some minimum bleed/ventilation demand is still
/// allowed through), a design choice, not a measured figure.
pub const IGV_LOAD_SHED_FLOOR: f64 = 0.3;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn turbine_pressure_ratio_design_is_below_the_compressor_pressure_ratio() {
        // The combustor's own pressure loss means the turbine never sees
        // quite as much pressure ratio as the compressor generated.
        assert!(turbine_pressure_ratio_design() < CORE_PRESSURE_RATIO_DESIGN);
        assert!(turbine_pressure_ratio_design() > 1.0);
    }

    #[test]
    fn rated_power_is_in_a_physically_plausible_band_for_this_spool_speed() {
        // Sanity check only (not used by the cycle itself): rated shaft
        // power at the design N should demand a torque within an
        // order-of-magnitude of what a 0.03ish kg*m^2 rotor spooling in tens
        // of seconds could plausibly produce/absorb, i.e. not some
        // wildly-off number from a units mistake.
        let omega_rated = N_DESIGN_RPM * std::f64::consts::TAU / 60.0;
        let torque_at_rated_power = RATED_SHAFT_POWER_W / omega_rated;
        assert!(torque_at_rated_power > 10.0 && torque_at_rated_power < 5000.0);
    }
}

#[cfg(test)]
mod egt_limit_tests {
    use super::*;

    #[test]
    fn the_three_kinds_of_egt_limit_are_ordered_the_way_the_ecb_orders_them() {
        // Control limit < caution < warning < protective trip. If this
        // ordering is ever broken the APU annunciates (or shuts down) in
        // normal operation, which is the exact failure mode these constants
        // were split apart to prevent.
        //   750 (control) < 867 (caution = 900 - 33) < 900 (warning) < 950 (trip)
        assert!(EGT_RUNNING_LIMIT_C < egt_caution_c(false, 0.0));
        assert!(egt_caution_c(false, 0.0) < EGT_RUNNING_WARNING_C);
        assert!(EGT_RUNNING_WARNING_C < EGT_TRIP_C);
        // The start schedule's own control limit likewise stays at or below
        // the start warning it is derived from, and under the trip.
        assert!(EGT_START_LIMIT_C <= EGT_START_WARNING_BELOW_FL250_C);
        assert!(EGT_START_LIMIT_C < EGT_TRIP_C);
    }

    #[test]
    fn the_start_warning_rises_above_fl250_and_only_while_starting() {
        // ECB behaviour (electronic_control_box.rs:267-292): the altitude
        // split applies to the Starting state only.
        assert_eq!(egt_warning_c(true, 0.0), EGT_START_WARNING_BELOW_FL250_C);
        assert_eq!(egt_warning_c(true, 24_999.0), EGT_START_WARNING_BELOW_FL250_C);
        assert_eq!(egt_warning_c(true, 25_000.0), EGT_START_WARNING_AT_OR_ABOVE_FL250_C);
        assert_eq!(egt_warning_c(true, 39_000.0), EGT_START_WARNING_AT_OR_ABOVE_FL250_C);
        // Not starting: the running warning regardless of altitude.
        assert_eq!(egt_warning_c(false, 0.0), EGT_RUNNING_WARNING_C);
        assert_eq!(egt_warning_c(false, 39_000.0), EGT_RUNNING_WARNING_C);
    }

    #[test]
    fn caution_tracks_warning_by_the_ecbs_own_fixed_difference() {
        for &(starting, alt) in &[(true, 0.0), (true, 30_000.0), (false, 0.0), (false, 30_000.0)] {
            let gap = egt_warning_c(starting, alt) - egt_caution_c(starting, alt);
            assert!((gap - EGT_WARNING_TO_CAUTION_DIFFERENCE_C).abs() < 1e-12);
        }
        // The one arithmetic result worth pinning: 900 - 33 = 867.
        assert!((egt_caution_c(false, 0.0) - 867.0).abs() < 1e-12);
    }

    #[test]
    fn the_pw980a_running_warning_is_not_the_a320_figure() {
        // A guard against someone "generalising" this back to a family
        // value: FlyByWire's APS3200 (A320) sets RUNNING_WARNING_EGT to
        // 682 deg C against the PW980A's 900, so the two are not
        // interchangeable and neither could have been guessed from the
        // other.
        assert!(EGT_RUNNING_WARNING_C > 682.0 + 100.0);
    }
}
