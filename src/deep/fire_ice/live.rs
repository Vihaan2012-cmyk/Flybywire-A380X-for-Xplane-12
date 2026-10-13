//! The live fire and ice-protection system: the nine detection zones,
//! their combustion sources, the bottles that suppress them, the cargo
//! smoke detectors, and every anti-ice surface -- all owned in one place,
//! stepped every frame from [`Truth`], with each failure
//! [`super::registry`] registers driving the exact model field that
//! registry entry names.
//!
//! Before this file, `fire_loops`, `combustion`, `extinguishing`,
//! `icing` and `anti_ice` were types with unit tests and no instance:
//! `FIRE_DETECTED_ENG:1` did not exist, so `ENG 1 FIRE` could not fire,
//! and no failure in the catalogue reached any of them.
//!
//! ## The causal chain this assembles
//! A registered leak failure puts flammable fluid into a zone
//! ([`combustion::ZoneSupply::fuel_available_kg_s`]); the zone's own
//! ventilation air and an ignition source (a running engine's hot turbine
//! case, a running APU) decide whether it lights; its heat release raises
//! that zone's temperature; the two detection loops sense *that
//! temperature* and declare the fire; the heat crossing a real inter-zone
//! link can light a neighbour's own leak. Nothing anywhere sets a "fire"
//! flag directly.
//!
//! ## How `Truth` drives it
//! - `environment` (static air temperature, true airspeed, ambient
//!   pressure, X-Plane's own cloud sample) gives the icing environment:
//!   liquid water content and droplet size come from
//!   `integration::weather_truth`'s own CS-25 Appendix C-shaped model of
//!   the real cloud, not from a chosen number.
//! - `engine_running` and `apu_running` are the ignition sources in the
//!   engine and APU fire zones, and gate the engine fan-duct/APU
//!   ventilation those zones are swept by.
//! - `on_ground` + `apu_running` drive the one extinguishing path that
//!   needs no cockpit action: the APU's automatic on-ground agent
//!   discharge.
//! - `environment.precipitation_on_aircraft_ratio` and `tas_ms` give the
//!   windshield its real water catch rate (`catch = LWC_rain * TAS *
//!   beta`), which is what the rain-removal jet has to shear off.
//!
//! ## Inputs this area needs that `Truth` does not carry yet
//! `docs/deep/truth-requests.md`'s 2026-09-20 pass sourced most of the
//! cockpit controls this area used to assume; this pass wires them in:
//! - **Fire/agent pushbuttons, per engine and the APU** -- now read from
//!   `truth.controls.fire_pb_released`/`fire_agent_pb_pressed`(+APU),
//!   which brings every bottle squib live (previously only the APU's
//!   automatic ground discharge could ever fire a bottle).
//! - **Wing/nacelle anti-ice selection** -- now read from
//!   `truth.controls.wing_anti_ice_selected`/`nacelle_anti_ice_selected`,
//!   so the valve-stuck-*closed* and duct-leak failures finally have a
//!   commanded flow to subtract from; valve-stuck-*open* worked already,
//!   because that failure floors flow regardless of command.
//! - **Cabin/lavatory local temperature** -- now read from
//!   `truth.cabin_temp_k` (one representative cabin zone; `Truth` carries
//!   no lavatory-specific reading, see the truth-requests doc) for the
//!   fusible-link extinguisher, instead of the outside static air
//!   temperature.
//! - **Rain-removal selection.** `truth.controls.rain_removal_selected`
//!   exists and is read here, but stays permanently off: no real
//!   pushbutton exists in this port (`plugin.rs`'s own sourcing table), so
//!   `Controls::default()` never sets it. The water film still accumulates
//!   from real precipitation; the jet's own fault has no jet to shear it
//!   with until a real control surfaces.
//! - **Cargo BULK**: `fire_loops::ZONES` has no bulk hold and no bulk
//!   detector is registered, so `CARGO_BULK_SMOKE_DETECTED` is not
//!   published here. That alert still reaches its trigger through
//!   `thermal_zones`' contribution on the bulk hold's own smoke
//!   concentration. The cargo FWD/AFT agent-discharge pushbuttons now have
//!   a real command (`truth.controls.cargo_agent_pb_pressed`, W194 --
//!   FlyByWire's own tooltip for them says "(Inop.)", but a real cockpit
//!   control writes the name and this area's bottle model already
//!   existed), so cargo bottles fire and leak alike; the bottle's own
//!   low-pressure switch is published either way, since the leak that
//!   drives it needs no command.
//! - **A single shorted (or open) fire loop used to be invisible.** Zone
//!   detection is correctly AND (`fire_loops::ZoneDetector`), and a short is
//!   correctly not classed as a `loop_x_fault` (indistinguishable from real
//!   heat) -- but nothing published either loop's own raw reading, so a
//!   lone shorted loop moved nothing at all anywhere in this crate, even
//!   though the real dual-loop architecture exists precisely so a single
//!   loop disagreeing is itself annunciated. `FIRE_LOOP_A/B_<ZONE>_FIRE`
//!   (each loop's own raw signal) and `FIRE_LOOP_<ZONE>_DISAGREE` (the two
//!   loops disagreeing) are published below for exactly that, and
//!   `registry.rs`'s `<ZONE> FIRE DET FAULT` alert now triggers on the
//!   disagreement too, alongside the pre-existing open-circuit fault.

use super::anti_ice::{BleedAntiIceFaults, BleedAntiIceSurface, ProbeHeater, ProbeHeaterFaults, RainRemoval, RainRemovalFaults, WindowHeat, WindowHeatFaults, NACELLE_ANTI_ICE, WINDOW_TARGET_C, WING_ANTI_ICE};
use super::combustion::{Fluid, ZoneCombustion, ZoneSupply, HYDRAULIC_FLUID, JET_FUEL};
use super::extinguishing::{Bottle, BottleFaults, CargoSuppressionFaults, CargoSuppressionSystem, LavatoryFaults, LavatoryProtection, OpticalSmokeDetector, SmokeDetectorFaults, ZoneConcentration};
use super::fire_loops::{LoopFaults, LoopLogic, ZoneDetector};
use super::icing::{IcingEnvironment, IcingOutputs, IcingSurface, NACELLE_INLET, WINDSHIELD, WING_LEADING_EDGE};
use super::util::{air_dynamic_viscosity_pa_s, collection_efficiency_beta0, droplet_inertia_parameter, recovery_temperature_c};
use crate::deep::api::{failure_id, Area as RegArea};
use crate::deep::integration::weather_truth::{dominant_cloud, droplet_diameter_m_from_conditions, lwc_kg_m3_from_conditions};
use crate::deep::live::{Faults, Truth};

const ATA_FIRE: u16 = 26;
const ATA_ICE: u16 = 30;

/// Zone order, exactly `fire_loops::ZONES` and `registry::ZONES`.
const ZONE_KEYS: [&str; 9] = ["ENG1", "ENG2", "ENG3", "ENG4", "APU", "MLG", "CARGO_FWD", "CARGO_AFT", "AVIONICS"];

/// Per-zone maximum modelled leak rate at failure magnitude 1.0, kg/s --
/// the same figures `registry::ZONE_MAX_LEAK_KG_S` documents, repeated
/// here so the magnitude a failure means and the magnitude the model
/// applies cannot drift.
const ZONE_MAX_LEAK_KG_S: [f64; 9] = [0.05, 0.05, 0.05, 0.05, 0.03, 0.01, 0.02, 0.02, 0.005];

/// Ventilation air each zone is swept by, kg/s -- the oxidiser supply that
/// limits combustion and the flow that washes out smoke and suppression
/// agent. These are the same GENERIC per-compartment ventilation flows
/// `deep::thermal_zones::topology_a380` builds its own ventilation links
/// with for these same compartments (nacelle 3.0, APU compartment 1.5,
/// gear bay 1.2 with the doors open, cargo extract 0.30, avionics extract
/// 0.35), reproduced rather than imported per BRIEF rule 2.
const ZONE_VENTILATION_KG_S: [f64; 9] = [3.0, 3.0, 3.0, 3.0, 1.5, 1.2, 0.30, 0.30, 0.35];

/// Free air volume of each zone, m^3: again the same GENERIC figures
/// `topology_a380` uses (nacelle cowl 8, APU compartment 6, gear bay 15,
/// cargo fwd 110, cargo aft 60, main avionics 12). Sets how fast a
/// discharged bottle reaches its design concentration and how fast smoke
/// builds up.
const ZONE_VOLUME_M3: [f64; 9] = [8.0, 8.0, 8.0, 8.0, 6.0, 15.0, 110.0, 60.0, 12.0];

/// Lumped thermal mass of each zone's contents/structure, J/K: the same
/// GENERIC structure thermal masses `topology_a380` gives these
/// compartments.
const ZONE_THERMAL_MASS_J_K: [f64; 9] = [1.5e5, 1.5e5, 1.5e5, 1.5e5, 2.0e5, 4.0e5, 5.0e5, 3.0e5, 5.0e5];

/// Sea-level air density, kg/m^3 (ICAO Doc 7488), used only to turn the
/// ventilation mass flows above into the volume flows
/// `ZoneConcentration`/`OpticalSmokeDetector` wash out with.
const AIR_DENSITY_KG_M3: f64 = 1.225;

/// Liquid water content of heavy rain, kg/m^3. A rain rate around
/// 100 mm/h carries roughly 2 g of liquid water per cubic metre (standard
/// rainfall-rate/water-content relation used in radar meteorology);
/// `environment.precipitation_on_aircraft_ratio` (X-Plane's own 0..1
/// wetness at the aircraft) scales it. Used only for the windshield's
/// water catch rate.
const HEAVY_RAIN_LWC_KG_M3: f64 = 2.0e-3;

/// Turbulent-boundary-layer recovery factor, standard.
const RECOVERY_FACTOR: f64 = 0.9;

/// **GENERIC** rain-removal jet velocity once selected on, m/s: the same
/// order-of-magnitude figure `anti_ice.rs`'s own rain-removal tests already
/// exercise for "the jet is on" (a bleed-air-blower duct's jet velocity is
/// not a published A380 figure). `truth.controls.rain_removal_selected`
/// never actually reads true in this port (no real pushbutton exists, see
/// this module's own doc), so this constant is exercised only by this
/// file's own coupling test today.
const RAIN_REMOVAL_JET_VELOCITY_M_S: f64 = 200.0;

/// Which fluid leaks in which zone. Engine and APU zones sit among fuel
/// and oil manifolds (`JET_FUEL`, the most easily ignited of the three);
/// the gear bay, holds and avionics bay carry phosphate-ester hydraulic
/// lines, whose much higher autoignition temperature (468 C, the reason
/// transport hydraulics use it) is exactly why a leak there does not
/// light on its own.
const ZONE_FLUID: [Fluid; 9] = [JET_FUEL, JET_FUEL, JET_FUEL, JET_FUEL, JET_FUEL, HYDRAULIC_FLUID, HYDRAULIC_FLUID, HYDRAULIC_FLUID, HYDRAULIC_FLUID];

/// Conductive links between fire zones, `(hot zone, cold zone, W/K)`, and
/// the path by which one zone's fire can light the next one's own leak.
/// Only genuinely adjacent pairs, with the conductances
/// `topology_a380` already gives those same pairs of compartments
/// (MainAvionics<->CargoFwd 20 W/K, BodyGearWell<->CargoAft 25 W/K). The
/// four engines are metres of wing apart and are not linked.
const ZONE_LINKS: [(usize, usize, f64); 2] = [(6, 8, 20.0), (5, 7, 25.0)];

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

/// The atmospheric icing condition and the surface heat-transfer inputs
/// every heated/unheated surface shares this tick.
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

    /// Droplet collection efficiency at the stagnation point of a body of
    /// this leading-edge size, in this cloud.
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

/// What one zone reported this tick.
#[derive(Clone, Copy, Debug, Default)]
struct ZoneReport {
    temp_c: f64,
    burning: bool,
    burn_rate_kg_s: f64,
    fire: bool,
    loop_a_fault: bool,
    loop_b_fault: bool,
    /// Each loop's own raw "I see fire" opinion, published so a single
    /// shorted (or otherwise disagreeing) loop is visible even though the
    /// zone's own AND logic correctly withholds `fire` and a short is
    /// correctly not a `loop_x_fault` (see `fire_loops::ZoneFireStatus`'s
    /// own doc).
    loop_a_signal: bool,
    loop_b_signal: bool,
    agent_fraction: f64,
}

pub struct FireIceLive {
    detectors: Vec<ZoneDetector>,
    combustion: Vec<ZoneCombustion>,
    concentration: Vec<ZoneConcentration>,
    zones: [ZoneReport; 9],

    /// Two bottles per engine, one for the APU (`registry`'s own split).
    engine_bottles: Vec<[Bottle; 2]>,
    engine_bottle_low: [[bool; 2]; 4],
    engine_squib_discharged: [[bool; 2]; 4],
    apu_bottle: Bottle,
    apu_squib_discharged: bool,

    cargo_suppression: Vec<CargoSuppressionSystem>,
    cargo_smoke: Vec<OpticalSmokeDetector>,
    cargo_smoke_alarm: [bool; 2],
    /// ECAM completeness pass (E-FIRE §F): this frame's cargo suppression
    /// discretes, `[FWD, AFT]`, cached from `step_bottles` because
    /// `publish` takes `&self`. None of these are physics -- each is a
    /// direct pass-through of its own registered failure's armed state.
    cargo_distribution_fault: [bool; 2],
    cargo_knockdown_squib_fault: [bool; 2],
    cargo_extended_squib_fault: [bool; 2],
    /// ECAM completeness pass (E-FIRE §E): the FWD Lower Crew Rest (LDCR)
    /// module's own two-bottle squib-circuit discretes, `[bottle 1, bottle
    /// 2]` -- pure pass-throughs of `registry.rs`'s own `26_fire.ldcr_
    /// bottle_{1,2}` failures (`fire(230)`/`fire(231)`), cached from `tick`
    /// because `publish` takes `&self`.
    ldcr_bottle_squib_fault: [bool; 2],
    lavatory: LavatoryProtection,

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
}

impl VarNames {
    fn new() -> Self {
        let zone_name = |z: usize| {
            // The four engine zones are addressed the way this crate
            // already addresses per-engine variables (`DEEP_FIRE_DETECTED_
            // ENG:n`, the name `registry.rs`'s own trigger uses).
            match z {
                0..=3 => format!("ENG:{}", z + 1),
                _ => ZONE_KEYS[z].to_string(),
            }
        };
        Self {
            // W162: `DEEP_` prefixed. `ZONE_KEYS[4]`/`[5]` are "APU"/"MLG",
            // and FlyByWire's own `fire_and_smoke_protection.rs` publishes
            // `FIRE_DETECTED_APU`/`FIRE_DETECTED_MLG` for its own live
            // dual-loop-AND detection -- an exact, unintended collision
            // with this area's own zone-temperature-driven verdict (the
            // other 7 zones never collided, but are renamed too so this
            // stays one array built from one format! call, not a
            // per-index special case that could leave one unrenamed).
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
            // W162: `DEEP_` prefixed -- FlyByWire's own `fire_and_smoke_
            // protection.rs` publishes `FIRE_SQUIB_{bottle}_ENG_{engine}_
            // IS_DISCHARGED` for its own squib-timer model, an exact
            // collision with this area's bottle-pressure-drain model.
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
        }
    }
}

impl Default for FireIceLive {
    fn default() -> Self {
        Self::new()
    }
}

impl FireIceLive {
    /// Halon charge per engine/APU bottle, kg. **GENERIC**: no public
    /// A380 figure. Sized against NFPA 12A's own 5% v/v design
    /// concentration for the nacelle volume above (8 m^3 of air needs
    /// about 2.5 kg of Halon 1301 at sea-level density to reach 5% by
    /// volume), with margin for the ventilation washing through during
    /// discharge.
    const NACELLE_BOTTLE_CHARGE_KG: f64 = 5.0;
    /// Bottle internal volume, m^3. **GENERIC**: liquid Halon 1301 at
    /// about 1570 kg/m^3 needs roughly 3 litres for the charge above; a
    /// 5 litre bottle leaves the nitrogen head space a
    /// super-pressurized bottle needs.
    const BOTTLE_VOLUME_M3: f64 = 0.005;
    /// Cargo bottles are sized for the extended metered discharge
    /// CS-25.858 requires across a diversion, into a much larger hold.
    /// **GENERIC**, same reasoning as above applied to the 110/60 m^3
    /// holds.
    const CARGO_BOTTLE_CHARGE_KG: f64 = 30.0;
    const CARGO_BOTTLE_VOLUME_M3: f64 = 0.03;
    /// A cargo smoke detector's optical path length, m. **GENERIC**:
    /// a ceiling-mounted photoelectric chamber's path is centimetres, but
    /// the detector samples a duct drawing from the hold; 1 m is the
    /// figure `extinguishing`'s own tests use.
    const SMOKE_DETECTOR_PATH_M: f64 = 1.0;
    /// A lavatory's own free volume, m^3. **GENERIC**.
    const LAVATORY_VOLUME_M3: f64 = 2.0;

    /// (W194) `Truth::controls.cargo_agent_pb_pressed` now exists
    /// (`src/deep/live.rs`), sourced from the real
    /// `A32NX_CARGOSMOKE_{FWD,AFT}_DISCHARGED` overhead pushbuttons
    /// (`deep/plugin.rs`'s own sourcing table). FlyByWire's own tooltip
    /// for that button says "(Inop.)" and no FBW system reads the name --
    /// but a real cockpit control exists and this module's own cargo
    /// bottle physics (leak, low-pressure switch, agent metering) were
    /// already real, so the command is wired for real (`step_bottles`
    /// below) rather than held at a permanent `false` as it used to be
    /// here (that used to be a named `NO_CARGO_FIRE_COMMAND` constant, for
    /// exactly the reasons this comment used to give). The cargo squib
    /// failures (`cargo_fwd_bottle`/`cargo_aft_bottle`'s, since E-FIRE's
    /// ECAM completeness pass split into `knockdown_squib_fault`/
    /// `extended_squib_fault`) are therefore observable now too --
    /// `registry.rs`'s `FailureDef.effect` text describing them as inert
    /// has been updated to match (W194). The bottle's *leak* failure is
    /// unaffected either way -- it drains independently of `fire_command`.

    pub fn new() -> Self {
        let start_c = 15.0;
        Self {
            // AND logic: a real fire-detection unit requires both loops to
            // agree before declaring a fire, and falls back to whichever
            // loop is still valid once the other faults
            // (`fire_loops::ZoneDetector::evaluate`).
            detectors: (0..9).map(|_| ZoneDetector::new(LoopLogic::And)).collect(),
            combustion: (0..9)
                .map(|z| ZoneCombustion::new(ZONE_FLUID[z], start_c, ZONE_THERMAL_MASS_J_K[z], ZONE_VENTILATION_KG_S[z] * super::util::CP_AIR))
                .collect(),
            concentration: (0..9).map(|z| ZoneConcentration::new(ZONE_VOLUME_M3[z])).collect(),
            zones: [ZoneReport::default(); 9],

            engine_bottles: (0..4).map(|_| [Bottle::new(Self::NACELLE_BOTTLE_CHARGE_KG, Self::BOTTLE_VOLUME_M3), Bottle::new(Self::NACELLE_BOTTLE_CHARGE_KG, Self::BOTTLE_VOLUME_M3)]).collect(),
            engine_bottle_low: [[false; 2]; 4],
            engine_squib_discharged: [[false; 2]; 4],
            apu_bottle: Bottle::new(Self::NACELLE_BOTTLE_CHARGE_KG, Self::BOTTLE_VOLUME_M3),
            apu_squib_discharged: false,

            cargo_suppression: (0..2).map(|_| CargoSuppressionSystem::new(Self::CARGO_BOTTLE_CHARGE_KG, Self::CARGO_BOTTLE_VOLUME_M3)).collect(),
            cargo_smoke: (0..2).map(|b| OpticalSmokeDetector::new(Self::SMOKE_DETECTOR_PATH_M, ZONE_VOLUME_M3[6 + b])).collect(),
            cargo_smoke_alarm: [false; 2],
            cargo_distribution_fault: [false; 2],
            cargo_knockdown_squib_fault: [false; 2],
            cargo_extended_squib_fault: [false; 2],
            ldcr_bottle_squib_fault: [false; 2],
            lavatory: LavatoryProtection::new(Self::LAVATORY_VOLUME_M3),

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

    /// An external ignition source in this zone: a running engine's hot
    /// turbine case in its own nacelle, a running APU in its bay. Nothing
    /// else is a standing ignition source, which is exactly why a hold or
    /// gear-bay leak only lights from heat crossing a link from a
    /// neighbour already on fire.
    fn ignition_source(truth: &Truth, zone: usize) -> bool {
        match zone {
            0..=3 => truth.engine_running[zone],
            4 => truth.apu_running,
            _ => false,
        }
    }

    fn step_fire(&mut self, truth: &Truth, faults: &Faults, cond: &Conditions) {
        let dt = truth.dt_s;
        let ambient_c = cond.recovery_c;

        // Inter-zone heat is computed from the temperatures every zone
        // held at the start of this tick, so no zone can see half a frame
        // (`live.rs`'s own ordering rule, applied inside the area too).
        let before_c: [f64; 9] = std::array::from_fn(|z| self.combustion[z].temp_c());
        let mut extra_w = [0.0_f64; 9];
        for (a, b, ua) in ZONE_LINKS {
            let q = super::combustion::conductive_link_w(ua, before_c[a], before_c[b]);
            extra_w[a] -= q;
            extra_w[b] += q;
        }

        // Agent delivered into each zone this tick, before combustion
        // reads its suppression fraction.
        let agent_in = self.step_bottles(truth, faults, cond);
        for z in 0..9 {
            let vent_m3_s = ZONE_VENTILATION_KG_S[z] / AIR_DENSITY_KG_M3;
            self.concentration[z].step(agent_in[z], cond.static_air_c, cond.ambient_pressure_pa, vent_m3_s, dt);
        }

        for z in 0..9 {
            let supply = ZoneSupply {
                fuel_available_kg_s: faults.get(fire(100 + z as u16)) * ZONE_MAX_LEAK_KG_S[z],
                air_available_kg_s: ZONE_VENTILATION_KG_S[z],
                ignition_source: Self::ignition_source(truth, z),
                suppression_fraction: self.concentration[z].suppression_fraction(),
            };
            let state = self.combustion[z].step(&supply, ambient_c, extra_w[z], dt);
            let (fa, fb) = Self::loop_faults(faults, z);
            // A continuous detection loop's reading is dominated by its
            // hottest point; this model carries one lumped temperature per
            // zone, so the hot spot is that temperature (a conservative
            // reading -- a real flame's local temperature is higher than
            // its compartment's average, so anything this declares, the
            // real detector would declare sooner).
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

        // Cargo optical smoke detection, off the same burn rate.
        //
        // (E-FIRE §F) IDs shifted from 223/224 to 227/228: the two cargo
        // bottles below now register 4 failures each (leak, knockdown
        // squib, extended squib, distribution) instead of 2, so every id
        // after them in this function's own sequential counter moved by
        // +4. There is no numbering scheme other than this function's own
        // `n` counter in `registry.rs::register_extinguishing` to drift
        // against; this comment is the cross-check.
        for b in 0..2 {
            let zone = 6 + b;
            let faults_det = SmokeDetectorFaults { lens_obscured: faults.get(fire(227 + b as u16)) };
            let vent_m3_s = ZONE_VENTILATION_KG_S[zone] / AIR_DENSITY_KG_M3;
            self.cargo_smoke_alarm[b] = self.cargo_smoke[b].step(self.zones[zone].burn_rate_kg_s, vent_m3_s, &faults_det, dt);
        }

        // Lavatory fusible link. `Truth` carries no lavatory-specific
        // temperature, but it now carries one representative cabin zone's
        // (`truth.cabin_temp_k`), which is a far better local reading than
        // the outside static air this used to fall back to.
        let lav_smoke = SmokeDetectorFaults::default();
        let lav_link = LavatoryFaults { link_degraded: faults.get(fire(229)) };
        self.lavatory.step(truth.cabin_temp_k - 273.15, 0.0, 0.01, &lav_smoke, &lav_link, dt);
    }

    /// Steps every bottle and returns the agent mass flow each zone
    /// received, kg/s.
    fn step_bottles(&mut self, truth: &Truth, faults: &Faults, cond: &Conditions) -> [f64; 9] {
        let dt = truth.dt_s;
        let ambient_c = cond.static_air_c;
        let zone_pa = cond.ambient_pressure_pa;
        let mut agent = [0.0_f64; 9];

        // ECAM completeness pass (E-FIRE §E): the LDCR module's own two
        // bottles' squib-circuit discretes, pure pass-throughs (struct doc
        // on `ldcr_bottle_squib_fault`).
        self.ldcr_bottle_squib_fault = [fire(230), fire(231)].map(|id| faults.get(id) > 0.0);

        for e in 0..4usize {
            for b in 0..2usize {
                let leak_id = fire(201 + (e as u16) * 4 + (b as u16) * 2);
                let bottle_faults = BottleFaults { leak: faults.get(leak_id), squib_failure: faults.get(leak_id + 1) };
                // The squib fires once the crew has pulled that engine's
                // fire pushbutton (arms the squib circuit) *and* pressed
                // this bottle's own agent pushbutton (the real two-step
                // engine fire drill), both now real cockpit reads
                // (`truth.controls`) instead of an unreachable `false`.
                let fire_command = truth.controls.fire_pb_released[e] && truth.controls.fire_agent_pb_pressed[e][b];
                let delivered = self.engine_bottles[e][b].step(ambient_c, fire_command, zone_pa, &bottle_faults, dt);
                agent[e] += delivered;
                self.engine_bottle_low[e][b] = self.engine_bottles[e][b].is_low_pressure();
                self.engine_squib_discharged[e][b] = self.engine_bottles[e][b].is_discharged();
            }
        }

        // The APU bottle fires either automatically on the ground (the one
        // extinguishing path that never needed crew action) or on the same
        // two-pushbutton drill as the engines', now both real.
        let apu_faults = BottleFaults { leak: faults.get(fire(217)), squib_failure: faults.get(fire(218)) };
        let apu_command = (self.zones[4].fire && truth.on_ground) || (truth.controls.fire_pb_apu_released && truth.controls.fire_agent_pb_apu_pressed);
        agent[4] += self.apu_bottle.step(ambient_c, apu_command, zone_pa, &apu_faults, dt);
        self.apu_squib_discharged = self.apu_bottle.is_discharged();

        for b in 0..2usize {
            let zone = 6 + b;
            // (E-FIRE §F) Each hold now registers 4 sequential ids: leak,
            // knockdown squib, extended squib, distribution -- replacing
            // the old 2 (leak, squib_failure). Base 219 for FWD, 223 for
            // AFT (`registry.rs::register_extinguishing`'s own sequence).
            let base = fire(219 + (b as u16) * 4);
            let cargo_faults = CargoSuppressionFaults {
                leak: faults.get(base),
                knockdown_squib_fault: faults.get(base + 1),
                extended_squib_fault: faults.get(base + 2),
                distribution_fault: faults.get(base + 3),
            };
            // ECAM completeness pass (E-FIRE §F): cache this frame's
            // discretes for `publish` (module doc on the struct fields).
            self.cargo_knockdown_squib_fault[b] = cargo_faults.knockdown_squib_fault > 0.0;
            self.cargo_extended_squib_fault[b] = cargo_faults.extended_squib_fault > 0.0;
            self.cargo_distribution_fault[b] = cargo_faults.distribution_fault > 0.0;
            let delivered = {
                let (system, concentration) = (&mut self.cargo_suppression[b], &self.concentration[zone]);
                // (W194) `cargo_agent_pb_pressed[b]`: `b=0` is FWD, `b=1` is
                // AFT, matching this same loop's `zone = 6 + b` and the
                // `["FWD","AFT"][b]` used a few lines below for the
                // published `CARGO_*_AGENT_METERING` name. No separate
                // "fire handle" gate exists for cargo the way the engine/
                // APU bottles have (`fire_pb_released[e] &&
                // fire_agent_pb_pressed[e][b]`) -- FlyByWire's own overhead
                // panel has only the one discharge pushbutton per bay, so
                // the pushbutton alone is the command.
                system.step(ambient_c, truth.controls.cargo_agent_pb_pressed[b], zone_pa, concentration, &cargo_faults, dt)
            };
            agent[zone] += delivered;
        }

        agent
    }

    fn step_ice(&mut self, truth: &Truth, faults: &Faults, cond: &Conditions) {
        let dt = truth.dt_s;

        // -- Wing leading edges. The anti-ice valve command is now the real
        // wing anti-ice pushbutton (one selection, both wings --
        // `truth.controls.wing_anti_ice_selected`): a stuck-open valve
        // still floors flow regardless of command, but stuck-*closed* and
        // duct-leak now have a real commanded flow to act against.
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

            // What the heated surface stops freezing is removed from the
            // accretion the unheated Messinger balance would give, using
            // last tick's impingement (the same one-frame lag `live.rs`
            // documents between coupled models).
            let natural_ff = self.wing_ice_out[s].freezing_fraction;
            let removal = (self.wing_ice_out[s].impingement_kg_m2_s * (natural_ff - out.freezing_fraction)).max(0.0);
            self.wing_ice_out[s] = self.wing_ice[s].step(&cond.icing_environment(), removal, dt);
        }

        // -- Nacelle inlets, per engine's own real nacelle anti-ice
        // pushbutton (`truth.controls.nacelle_anti_ice_selected`).
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

        // -- Probe heaters: thermostatic and permanently energised, the
        // real automatic behaviour (there is no probe-heat selection to
        // read).
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

        // -- Windshields: heated film, plus the water film the real
        // precipitation lands on it and the rain-removal jet's shear.
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
            // `truth.controls.rain_removal_selected` is real and read here,
            // but stays permanently false in this port (no rain-removal
            // pushbutton exists to write it, module doc) -- the coupling is
            // wired and ready for the moment a real source appears, but
            // today the film still only accumulates from real rain, since
            // no jet is ever commanded.
            let jet_velocity_m_s = if truth.controls.rain_removal_selected[w] { RAIN_REMOVAL_JET_VELOCITY_M_S } else { 0.0 };
            self.rain_film_kg_m2[w] = self.rain[w].step(rain_catch_kg_m2_s, jet_velocity_m_s, 0.0, &rain_faults, dt);
        }
    }

    /// A probe heater has failed when the probe is genuinely below
    /// freezing and its heater is delivering nothing -- the condition all
    /// three registered probe failures (open element, dead controller,
    /// sensor stuck warm) produce, and that a healthy thermostatic heater
    /// never does.
    fn probe_fault(out: &super::anti_ice::ProbeHeaterOutputs) -> bool {
        out.surface_c < 0.0 && out.power_w <= 0.0
    }

    /// A window heat fault: the film has overheated or damaged itself, or
    /// the window is far below its target with no power going in.
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
            }
        }
        // W162: `DEEP_` prefixed -- collided with FlyByWire's own bottle
        // id "1_APU_1" (`fire_and_smoke_protection.rs`).
        out("DEEP_FIRE_SQUIB_1_APU_1_IS_DISCHARGED", on(self.apu_squib_discharged));
        out("FIRE_BOTTLE_APU_LOW_PRESSURE", on(self.apu_bottle.is_low_pressure()));
        out("LAVATORY_EXTINGUISHER_DISCHARGED", on(self.lavatory.is_discharged()));

        for b in 0..2 {
            out(&n.cargo_smoke_detected[b], on(self.cargo_smoke_alarm[b]));
            out(&n.cargo_smoke_density[b], self.cargo_smoke[b].smoke_density_kg_m3());
            out(&format!("CARGO_{}_AGENT_METERING", ["FWD", "AFT"][b]), on(self.cargo_suppression[b].is_metering()));
            // The cargo bottle's leak failure drains it independently of
            // `fire_command`, so its low-pressure switch is a real,
            // observable consequence of a leak alone; since W194 it is
            // also a real consequence of an actual discharge (the
            // pushbutton is a real command now, not a permanent `false`).
            out(&n.cargo_bottle_low_pressure[b], on(self.cargo_suppression[b].bottle.is_low_pressure()));
            // ECAM completeness pass (E-FIRE §F): `260800043`/`044` FWD/AFT
            // CARGO BOTTLES FAULT (distribution path), `260800052`/`053`
            // FWD+AFT CARGO BTL 1/2 FAULT (the OR of both holds' own
            // knockdown/extended squib fault is composed in the FWC bridge,
            // `deep::ecam::fbw::ata26`, off these two per-hold discretes).
            out(&format!("FIRE_CARGO_{}_DISTRIBUTION_FAULT", ["FWD", "AFT"][b]), on(self.cargo_distribution_fault[b]));
            out(&format!("FIRE_CARGO_{}_KNOCKDOWN_SQUIB_FAULT", ["FWD", "AFT"][b]), on(self.cargo_knockdown_squib_fault[b]));
            out(&format!("FIRE_CARGO_{}_EXTENDED_SQUIB_FAULT", ["FWD", "AFT"][b]), on(self.cargo_extended_squib_fault[b]));
        }
        // ECAM completeness pass (E-FIRE §E): `260800054`/`055` SMOKE FWD
        // LWR CAB REST BTL 1/2 FAULT.
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
    }
}

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
        // Failure 8_026_100 (ENG 1 fuel/oil/hydraulic leak feeding a
        // fire), effect: "with an ignition source ... this fuel/air-limited
        // leak sustains combustion and raises the zone's own temperature".
        // ENG 1 FIRE triggers on FIRE_DETECTED_ENG:1.
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
        // Failure 8_026_002 (ENG 1 loop A short), effect: "loop reports
        // fire_signal=true indistinguishably from a real fire; under OR
        // logic (or once the other loop is also faulted) the zone is
        // falsely declared on fire". With the unit's normal AND logic a
        // single shorted loop must be rejected.
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
        // The live-level version of `fire_loops`'s own unit test: with
        // nothing else armed and no ignition source, engine 1's loop A
        // shorted must now show up as a per-loop signal and a disagree
        // discrete even though the zone itself correctly never declares
        // ENG 1 FIRE (AND logic, one loop still cold).
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
        // The leak failure drains the bottle independently of
        // `fire_command` (unaffected by W194's real command), so its low-
        // pressure switch must still move on a leak alone, with the
        // pushbutton never pressed.
        // The cargo bottle's design charge (30 kg, `Self::CARGO_BOTTLE_
        // CHARGE_KG`) is 6x the engine bottles' (5 kg), but the leak orifice
        // is the same absolute `extinguishing::LEAK_AREA_MAX_M2` regardless
        // of bottle size, and pressure holds flat until the charge falls
        // below the 5% residual-liquid fraction -- so draining a 30 kg
        // bottle down to that same fraction takes proportionally longer
        // than the ~11 h the engine-bottle version of this test needs.
        let mut truth = ground_running_truth();
        truth.engine_running = [false; 4];
        truth.apu_running = false;
        truth.dt_s = 60.0;
        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::from_pairs([(fire(219), 1.0)]), 5000); // ~83 h
        let map = published(area.as_ref());
        assert_eq!(map["FIRE_BOTTLE_CARGO_FWD_LOW_PRESSURE"], 1.0, "a full-severity leak must empty the cargo FWD bottle over hours");
        assert_eq!(map["FIRE_BOTTLE_CARGO_AFT_LOW_PRESSURE"], 0.0, "the AFT bottle is healthy");
    }

    /// E-FIRE §F: the three new per-hold cargo suppression discretes
    /// (`260800043`/`044` distribution, `260800052`/`053` knockdown/
    /// extended squib) each publish independently, off the exact ids
    /// `registry.rs::register_extinguishing` assigns (leak, knockdown,
    /// extended, distribution, base 219 FWD / 223 AFT).
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

        // FWD distribution fault (id 222) raises only its own discrete.
        let mut faulted = live_system();
        run(faulted.as_mut(), &ground_running_truth(), &Faults::from_pairs([(fire(222), 1.0)]), 5);
        let map = published(faulted.as_ref());
        assert_eq!(map["FIRE_CARGO_FWD_DISTRIBUTION_FAULT"], 1.0);
        assert_eq!(map["FIRE_CARGO_FWD_KNOCKDOWN_SQUIB_FAULT"], 0.0, "the distribution fault must not raise the squib discretes");
        assert_eq!(map["FIRE_CARGO_AFT_DISTRIBUTION_FAULT"], 0.0, "the AFT hold is unaffected");

        // AFT extended-squib fault (id 225) raises only its own discrete.
        let mut faulted2 = live_system();
        run(faulted2.as_mut(), &ground_running_truth(), &Faults::from_pairs([(fire(225), 1.0)]), 5);
        let map2 = published(faulted2.as_ref());
        assert_eq!(map2["FIRE_CARGO_AFT_EXTENDED_SQUIB_FAULT"], 1.0);
        assert_eq!(map2["FIRE_CARGO_AFT_KNOCKDOWN_SQUIB_FAULT"], 0.0);
        assert_eq!(map2["FIRE_CARGO_FWD_EXTENDED_SQUIB_FAULT"], 0.0, "the FWD hold is unaffected");
    }

    /// W194: before this pass `Self::NO_CARGO_FIRE_COMMAND` held both cargo
    /// bottles' fire command permanently false (module doc's old "no cargo
    /// fire/agent pushbutton in `Truth`"), so pressing PUSH_OVHD_CARGOSMOKE_
    /// FWD/_AFT could never discharge a bottle. With `truth.controls.
    /// cargo_agent_pb_pressed` wired to those real overhead pushbuttons,
    /// pressing FWD's must actually deliver agent to the FWD cargo zone,
    /// with the AFT bottle -- not pressed -- untouched. Unlike the engine/
    /// APU bottles there is no separate "fire handle" gate for cargo
    /// (FlyByWire's own overhead panel has only the one agent-discharge
    /// pushbutton per bay, `A380_Cockpit_Behavior.xml`'s `PUSH_OVHD_
    /// CARGOSMOKE_{FWD,AFT}`), so the pushbutton alone commands it, matching
    /// `step_bottles`'s own unconditional `cargo_agent_pb_pressed[b]` (no
    /// `&&` with a handle-released field, unlike the engine bottles' own
    /// `fire_pb_released[e] && fire_agent_pb_pressed[e][b]`).
    ///
    /// `FIRE_ZONE_CARGO_*_AGENT_FRACTION`, not `CARGO_*_AGENT_METERING`, is
    /// the right signal here: `CargoSuppressionSystem::is_metering` only
    /// turns true *after* the knockdown stage switches to the metered one
    /// (`extinguishing.rs`'s own `step`, once `zone_concentration.
    /// suppression_fraction() >= 1.0`), so a fresh press stays in the
    /// (faster-flowing) knockdown stage and would wrongly read as "not
    /// metering, so nothing happened" if that name were used instead.
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
        // Failure 8_030_002 (L WING anti-ice valve stuck open), effect:
        // "continues delivering full bleed heat once icing conditions/
        // demand end, driving the skin/duct temperature into an overheat
        // trip". WING A-ICE OVHT triggers on ANTI_ICE_WING_L_OVERHEAT.
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
        // Failure 8_030_020 (PITOT1 heater controller fault), effect:
        // "heater never energises even in icing conditions; probe ices".
        // PROBE/WINDOW HEAT triggers on PROBE_HEAT_PITOT1_FAULT.
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
        // Failure 8_030_043 (L WINDSHIELD film defect), effect: "a severe
        // defect drives the local hot spot past the delamination and crack
        // damage thresholds". WINDSHIELD HEAT FAULT triggers on
        // WINDOW_HEAT_L_FAULT.
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
        // Failure 8_026_201 (ENG 1 fire bottle 1 leak), effect: "bottle
        // mass/pressure fall over time; if not caught before use, delivers
        // less agent (or none) when actually fired". ENG 1 FIRE AGENT LO PR
        // triggers on FIRE_BOTTLE_ENG1_1_LOW_PRESSURE.
        let mut truth = ground_running_truth();
        truth.engine_running = [false; 4];
        truth.apu_running = false;
        truth.dt_s = 10.0;
        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::from_pairs([(fire(201), 1.0)]), 4000); // ~11 h
        let map = published(area.as_ref());
        assert_eq!(map["FIRE_BOTTLE_ENG1_1_LOW_PRESSURE"], 1.0, "a full-severity leak must empty the bottle over hours");
        assert_eq!(map["FIRE_BOTTLE_ENG1_2_LOW_PRESSURE"], 0.0, "the second bottle is healthy");
        assert_eq!(map["DEEP_FIRE_DETECTED_ENG:1"], 0.0, "a leaking bottle is not a fire");
    }

    #[test]
    fn pressing_the_fire_and_agent_pushbuttons_actually_fires_a_healthy_bottle() {
        // Coupling test: before this pass, no cockpit path could ever set
        // `fire_command` true for an engine bottle (module doc's old "no
        // fire/agent pushbutton in `Truth`"), so a squib could never
        // discharge outside the APU's automatic ground path. With
        // `truth.controls.fire_pb_released`/`fire_agent_pb_pressed` wired
        // in, the real two-step drill (pull the fire handle, then press an
        // agent bottle) must discharge that bottle -- with no fire and no
        // leak fault armed at all, isolating this from every other model.
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
        truth.controls.fire_pb_released[0] = true; // handle pulled, no bottle pressed yet
        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::default(), 5);
        let map = published(area.as_ref());
        assert_eq!(map["DEEP_FIRE_SQUIB_1_ENG_1_IS_DISCHARGED"], 0.0);
        assert_eq!(map["DEEP_FIRE_SQUIB_2_ENG_1_IS_DISCHARGED"], 0.0);
    }

    #[test]
    fn an_apu_bottle_also_fires_from_the_real_pushbutton_pair_in_flight_with_no_automatic_path() {
        // The APU's automatic ground discharge only applies `on_ground`;
        // in flight the only path left is the same two-pushbutton drill,
        // now real.
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
        // Failure 8_030_001 (L WING anti-ice valve stuck closed), effect:
        // "commanded flow is reduced toward zero regardless of demand".
        // Before this pass the valve command was hardcoded to 0 (off), so
        // this failure was indistinguishable from the crew simply never
        // selecting anti-ice on; with `truth.controls.
        // wing_anti_ice_selected` wired in, arming this fault must diverge
        // from a healthy selected-on wing, which it could not before.
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
        // The other half of the same coupling: with the pushbutton wired
        // in, a *healthy* system must now genuinely respect "off" (it used
        // to be hardcoded off already, but for the wrong reason -- no
        // command existed at all, so this is the regression guard that the
        // new command path defaults to the same safe state).
        let truth = icing_truth();
        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::default(), 300);
        assert_eq!(published(area.as_ref())["ANTI_ICE_WING_L_VALVE_OPEN"], 0.0, "with anti-ice not selected, a healthy valve must stay shut");
    }

    #[test]
    fn ice_accretes_on_an_unprotected_wing_in_a_real_cloud_and_costs_lift() {
        // No failure: the environmental baseline the anti-ice systems
        // exist to prevent, driven from the real cloud sample.
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
