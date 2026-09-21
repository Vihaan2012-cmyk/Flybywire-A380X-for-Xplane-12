//! The live environment: the one area whose events are *random*, made
//! reproducible.
//!
//! What it owns is the whole weather-driven hazard set this directory
//! models, each with its own persistent damage state:
//!
//! * bird strikes (`bird_strike.rs`) against the four engine inlets, the
//!   six windshield panels, the radome, the twelve wing leading-edge
//!   segments, the nose gear and the six air-data probes;
//! * lightning (`lightning.rs`): radome and composite-extremity burn, the
//!   standby compass's post-strike deviation, and the conducted transients
//!   each bus and computer sees;
//! * hail (`hail.rs`): the same airframe targets again, plus each engine's
//!   own ingestion state;
//! * ice-crystal icing (`ice_crystal_icing.rs`), one accretion state per
//!   engine core;
//! * volcanic ash (`volcanic_ash.rs`), one exposure state per engine;
//! * wind shear and turbulence (`wind_shear.rs`), and the runway's own
//!   friction state (`runway_contamination.rs`).
//!
//! ## Determinism
//!
//! Bird strikes, hail shafts and lightning strikes are genuinely random
//! processes, and this is the only area in `deep/` whose models take a
//! `Rng`. One [`rng::Rng`] is created **once, at construction, from a seed
//! the caller chooses** -- [`DEFAULT_SEED`] unless [`EnvironmentLive::
//! with_seed`] is used -- and is then advanced only by `tick`, in a fixed
//! order (bird, lightning, hail, wind shear, turbulence). Nothing here
//! reads a clock, a frame counter, an address or any other ambient source
//! of entropy, so the same seed, the same weather and the same sequence of
//! `dt` values reproduce a flight's hazards exactly: a bird strike that
//! happened in a recording happens again at the same second on replay.
//! Give a flight its seed from anything reproducible (a flight number, a
//! scenario id, a saved-replay header) and it will replay.
//!
//! ## Armed failures
//!
//! Unlike every other area, this one's registered failures name the
//! *outcome* of a hazard rather than a component's own degradation
//! ("Bird strike fan blade damage" acts on
//! `StrikeOutcome.fan_damage_frac`). Arming one therefore means "this
//! element is damaged this badly": each armed magnitude is applied as a
//! floor under the damage state the stochastic models have already
//! accumulated for that element, so a failure the crew arms and a bird the
//! aircraft actually hit end up in the same number, and the worse of the
//! two wins.

use crate::deep::api::Registry;
use crate::deep::integration::environment_events_adapter as weather;
use crate::deep::live::{Faults, Truth};

use super::bird_strike::{self, BirdStrikeModel, FlightState, ImpactTarget as BirdTarget, Phase};
use super::hail::{self, HailFlightState, HailModel, ImpactTarget as HailTarget};
use super::ice_crystal_icing::{IceCrystalIcingState, IceCrystalInputs};
use super::lightning::{BusId, LightningModel};
use super::rng::Rng;
use super::runway_contamination::{self, RunwayFrictionOutput};
use super::volcanic_ash::{AshInputs, AshOutputs, VolcanicAshState};
use super::wind_shear::{self, TurbulenceGusts, TurbulenceModel, WindShearModel};

const N_ENGINES: usize = 4;
const N_WINDSHIELD: usize = 6;
const N_WING_LE: usize = 12;
const N_PROBE: usize = 6;
const N_NACELLE: usize = 4;

/// The seed a flight uses unless it is given one. A fixed compile-time
/// constant, never a clock reading: two runs of the same flight with the
/// same weather see the same birds. (The value is arbitrary and only has
/// to be non-zero, which `Rng::new` enforces anyway.)
pub const DEFAULT_SEED: u64 = 0x_A380_0BEE_0BEE_0001;

/// How long a conducted lightning transient keeps a bus or computer upset,
/// s. **GENERIC**: `lightning.rs` reports the transient as an instantaneous
/// event, but what the crew sees is the equipment's *recovery* -- a reset
/// and reboot. Ten seconds is the order of magnitude a digital avionics box
/// takes to restart, and it is what makes a 0.5 s-confirm ECAM alert
/// (`registry.rs`'s ENG FADEC TRANSIENT) able to catch a transient at all.
const BUS_UPSET_HOLD_S: f64 = 10.0;

/// A380 main landing gear tyre inflation pressure, psi. Published in the
/// Airbus A380 Aircraft Characteristics document's pavement-loading
/// section; it is what sets the NASA/Horne hydroplaning speed
/// (`runway_contamination::hydroplane_speed_kt`).
const MAIN_TYRE_PRESSURE_PSI: f64 = 218.0;

/// Altitude above which the aircraft is treated as being in the cruise
/// phase for bird-strike purposes. Bird-strike rate already falls off with
/// height inside `bird_strike::altitude_relative_risk`, so this only picks
/// which `Phase` label the model is given; 10 000 ft is the conventional
/// boundary between the terminal area and the en-route phase.
const EN_ROUTE_FT: f64 = 10_000.0;

// ---------------------------------------------------------------------------
// Inputs that `Truth` does not carry yet.
// ---------------------------------------------------------------------------

/// Everything this area needs that is not in [`Truth`].
#[derive(Clone, Copy, Debug)]
pub struct EnvironmentCommands {
    /// Height above the ground, m. `Truth::altitude_ft` is the aircraft's
    /// altitude, and bird strike risk is a function of height above the
    /// *terrain*, which is a different number everywhere but over the sea.
    pub altitude_agl_m: f64,
    /// Calendar month (1..=12) and whether the sun is down. Both have a
    /// strong, documented effect on strike rate and species mix
    /// (`bird_strike.rs`); `integration::environment_events_adapter` already
    /// derives both from X-Plane, but through datarefs `Truth` does not
    /// carry.
    pub month: u8,
    pub night: bool,
    /// Whether the gear is extended -- it is a bird-strike target only when
    /// it is out.
    pub gear_down: bool,
    /// Groundspeed, m/s: how fast the aircraft crosses a microburst.
    pub groundspeed_ms: f64,
    /// Vertical (downdraft) wind component, m/s, and the rate at which the
    /// headwind is changing, m/s^2 -- the two halves of the Bowles
    /// F-factor. `WindShearModel` synthesises them for an encounter it
    /// generates itself; a caller with a real wind field
    /// (`integration::environment_events_adapter::WindShearSampler`) can
    /// supply them instead and they are used in preference.
    pub measured_shear: Option<wind_shear::WindShearInputs>,
    /// Per engine, the core (not bypass) mass flow, kg/s: what actually
    /// carries ice crystals and ash into the gas path. Without it neither
    /// the ice-crystal nor the ash model can accrete anything, and there is
    /// nothing in `Truth` to derive it from -- `engine_n1_frac` is a fan
    /// speed, and turning it into a core flow needs the engine's own
    /// compressor map.
    pub engine_core_mass_flow_kg_s: [f64; N_ENGINES],
    /// Per engine, the temperature of the compressor-front surfaces ice can
    /// stick to, C, and the NGV gas temperature ash melts against, C, and
    /// the compressor-face relative velocity ash erodes at, m/s.
    pub engine_warm_surface_temp_c: [f64; N_ENGINES],
    pub engine_ngv_gas_temp_c: [f64; N_ENGINES],
    pub engine_compressor_velocity_ms: [f64; N_ENGINES],
    /// Airborne volcanic ash concentration, mg/m^3. X-Plane's weather
    /// system has no concept of ash at all (confirmed in
    /// `integration::environment_events_adapter`'s own module doc: no
    /// `sim/weather/*` field or `XPLMWeatherInfo_t` member represents it),
    /// so there is nothing to adapt and this can only come from a scenario
    /// or from the crew arming the ash failures directly.
    pub ash_concentration_mg_m3: f64,
    /// Whether the background random hazard generators are armed at all.
    /// Off by default: a flight only meets a bird or a hail shaft because
    /// something asked for that, and a scenario that arms failures
    /// explicitly should not also be fighting a random generator.
    pub random_hazards: bool,
}

impl Default for EnvironmentCommands {
    fn default() -> Self {
        Self {
            altitude_agl_m: 0.0,
            month: 1,
            night: false,
            gear_down: true,
            groundspeed_ms: 0.0,
            measured_shear: None,
            engine_core_mass_flow_kg_s: [0.0; N_ENGINES],
            engine_warm_surface_temp_c: [15.0; N_ENGINES],
            engine_ngv_gas_temp_c: [15.0; N_ENGINES],
            engine_compressor_velocity_ms: [0.0; N_ENGINES],
            ash_concentration_mg_m3: 0.0,
            random_hazards: false,
        }
    }
}

// ---------------------------------------------------------------------------
// Failure ids.
// ---------------------------------------------------------------------------

/// Every failure id `registry.rs` assigns, resolved once at construction.
struct Ids {
    bird_fan: u64,
    bird_core: u64,
    bird_windshield: u64,
    bird_radome: u64,
    bird_wing_le: u64,
    bird_nose_gear: u64,
    bird_probe: u64,
    ltg_radome: u64,
    ltg_structure: u64,
    ltg_compass: u64,
    ltg_bus: u64,
    hail_radome: u64,
    hail_windshield: u64,
    hail_wing_le: u64,
    hail_engine: u64,
    hail_probe: u64,
    hail_nacelle: u64,
    ash_glassing: u64,
    ash_erosion: u64,
    ash_windshield: u64,
    ash_pitot: u64,
    ice_accretion: u64,
    ice_rollback: u64,
    runway_friction: u64,
}

/// The one failure registered against `component`.
fn only_fid(reg: &Registry, component: &str) -> u64 {
    let mut found = reg.failures.iter().filter(|f| f.component == component);
    let first = found.next().unwrap_or_else(|| panic!("no failure registered on {component}"));
    let second = found.next();
    assert!(second.is_none(), "{component} has more than one failure; name the model_field to pick one");
    first.id
}

/// The one failure on `component` whose `model_field` contains `fragment`.
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
        Self {
            bird_fan: only_fid(&reg, "72_env.fan_bird_damage"),
            bird_core: only_fid(&reg, "72_env.core_fod"),
            bird_windshield: only_fid(&reg, "56_env.windshield"),
            bird_radome: only_fid(&reg, "53_env.radome"),
            bird_wing_le: only_fid(&reg, "57_env.wing_leading_edge"),
            bird_nose_gear: only_fid(&reg, "32_env.nose_gear"),
            bird_probe: only_fid(&reg, "34_env.air_data_probe"),
            ltg_radome: only_fid(&reg, "53_env.radome_lightning"),
            ltg_structure: only_fid(&reg, "53_env.composite_extremity"),
            ltg_compass: only_fid(&reg, "34_env.standby_compass"),
            ltg_bus: only_fid(&reg, "24_env.bus_transient"),
            hail_radome: only_fid(&reg, "53_env.radome_hail"),
            hail_windshield: only_fid(&reg, "56_env.windshield_hail"),
            hail_wing_le: only_fid(&reg, "57_env.wing_leading_edge_hail"),
            hail_engine: only_fid(&reg, "72_env.engine_hail_ingestion"),
            hail_probe: only_fid(&reg, "34_env.air_data_probe_hail"),
            hail_nacelle: only_fid(&reg, "71_env.nacelle_hail"),
            ash_glassing: only_fid(&reg, "72_env.ngv_glassing"),
            ash_erosion: only_fid(&reg, "72_env.compressor_erosion"),
            ash_windshield: only_fid(&reg, "56_env.windshield_ash"),
            ash_pitot: only_fid(&reg, "34_env.air_data_probe_ash"),
            ice_accretion: fid(&reg, "72_env.ice_crystal_accretion", "IceCrystalOutputs.flow_capacity_loss_frac"),
            ice_rollback: fid(&reg, "72_env.ice_crystal_accretion", "rollback_risk_frac"),
            runway_friction: only_fid(&reg, "32_env.runway_friction"),
        }
    }
}

// ---------------------------------------------------------------------------
// Persistent damage state.
// ---------------------------------------------------------------------------

/// What birds have done to the airframe so far. `bird_strike.rs` reports
/// one strike's outcome and keeps no state of its own, so the live system
/// keeps it: a dented radome stays dented, a blocked probe stays blocked.
#[derive(Clone, Debug, Default)]
struct BirdDamage {
    fan: [f64; N_ENGINES],
    core: [f64; N_ENGINES],
    windshield: [f64; N_WINDSHIELD],
    radome: f64,
    wing_le_drag: [f64; N_WING_LE],
    nose_gear: f64,
    probe_blocked: [f64; N_PROBE],
}

impl BirdDamage {
    fn absorb(&mut self, o: &bird_strike::StrikeOutcome) {
        let worst = |slot: &mut f64, v: f64| *slot = slot.max(v);
        match o.target {
            BirdTarget::EngineInlet(i) => {
                let i = i as usize % N_ENGINES;
                worst(&mut self.fan[i], o.fan_damage_frac);
                worst(&mut self.core[i], o.core_ingestion_frac);
            }
            BirdTarget::Windshield(i) => {
                let i = i as usize % N_WINDSHIELD;
                // `registry.rs` reads the pair as one 0..1 severity:
                // >0.6 cracked, >=1.0 penetrated.
                let severity = if o.windshield_penetrated {
                    1.0
                } else if o.windshield_crack {
                    0.6_f64.max(0.6)
                } else {
                    0.0
                };
                worst(&mut self.windshield[i], severity);
            }
            BirdTarget::Radome => worst(&mut self.radome, o.radome_damage_frac),
            BirdTarget::WingLeadingEdge(i) => worst(&mut self.wing_le_drag[i as usize % N_WING_LE], o.leading_edge_dent_drag_delta_cd),
            BirdTarget::NoseGear => worst(&mut self.nose_gear, o.nose_gear_damage_frac),
            BirdTarget::PitotAoaProbe(i) => {
                if o.probe_blocked {
                    worst(&mut self.probe_blocked[i as usize % N_PROBE], 1.0);
                }
            }
        }
    }
}

/// What lightning has left behind, and what it is still upsetting.
#[derive(Clone, Debug, Default)]
struct LightningDamage {
    radome: f64,
    structure: f64,
    compass_error_deg: f64,
    /// Seconds of upset still to run, one per `BUS_KINDS` entry.
    bus_upset_s: [f64; BUS_KINDS.len()],
    /// The largest transient each kind of bus has seen, V.
    bus_peak_volts: [f64; BUS_KINDS.len()],
}

/// The bus/computer kinds `lightning::BusId` distinguishes, by the name
/// `registry.rs` uses in its trigger (`ENV_LTG_BUS_UPSET:EngineFadec`).
/// Instanced buses (PRIM 1-3, FADEC 1-4, ...) are published per *kind*: the
/// flight deck's FADEC transient caution does not name which engine's
/// channel was upset, and neither does the alert.
const BUS_KINDS: [&str; 7] = ["Prim", "Sec", "Fmgc", "Adirs", "StandbyInstruments", "EngineFadec", "Ife"];

fn bus_kind_index(bus: BusId) -> usize {
    match bus {
        BusId::Prim(_) => 0,
        BusId::Sec(_) => 1,
        BusId::Fmgc(_) => 2,
        BusId::Adirs(_) => 3,
        BusId::StandbyInstruments => 4,
        BusId::EngineFadec(_) => 5,
        BusId::Ife => 6,
    }
}

/// What hail has done. `hail::HailDamageState` already accumulates its own
/// per-target severity, but only reports it through the `HailOutcome` of a
/// strike, so the live system keeps the last reported value per target.
#[derive(Clone, Debug, Default)]
struct HailDamage {
    radome: f64,
    windshield: [f64; N_WINDSHIELD],
    window_heat_fault: [bool; N_WINDSHIELD],
    wing_le: [f64; N_WING_LE],
    slat_jam_risk: [f64; N_WING_LE],
    engine_fan: [f64; N_ENGINES],
    engine_compressor_loss: [f64; N_ENGINES],
    engine_flameout_risk: [f64; N_ENGINES],
    nacelle: [f64; N_NACELLE],
    probe: [f64; N_PROBE],
}

impl HailDamage {
    fn absorb(&mut self, o: &hail::HailOutcome) {
        let worst = |slot: &mut f64, v: f64| *slot = slot.max(v);
        match o.target {
            HailTarget::Radome => worst(&mut self.radome, o.damage_frac),
            HailTarget::Windshield(i) => {
                let i = i as usize % N_WINDSHIELD;
                worst(&mut self.windshield[i], o.damage_frac);
                self.window_heat_fault[i] |= o.window_heat_fault;
            }
            HailTarget::WingLeadingEdge(i) => {
                let i = i as usize % N_WING_LE;
                worst(&mut self.wing_le[i], o.damage_frac);
                worst(&mut self.slat_jam_risk[i], o.slat_jam_risk_frac);
            }
            HailTarget::EngineInlet(i) => {
                let i = i as usize % N_ENGINES;
                worst(&mut self.engine_fan[i], o.fan_damage_frac);
                worst(&mut self.engine_compressor_loss[i], o.compressor_efficiency_loss_frac);
                // Flameout risk is instantaneous, not cumulative: it is
                // about the ice going down the core *now*.
                self.engine_flameout_risk[i] = o.flameout_risk_frac;
            }
            HailTarget::Nacelle(i) => worst(&mut self.nacelle[i as usize % N_NACELLE], o.damage_frac),
            HailTarget::Probe(i) => worst(&mut self.probe[i as usize % N_PROBE], o.damage_frac),
        }
    }
}

// ---------------------------------------------------------------------------
// The live system.
// ---------------------------------------------------------------------------

/// The live environmental hazard set.
pub struct EnvironmentLive {
    ids: Ids,
    /// The one source of randomness in `deep/`, seeded once (see the module
    /// doc) and advanced only by `tick`.
    rng: Rng,
    seed: u64,

    birds: BirdStrikeModel,
    bird_damage: BirdDamage,
    lightning: LightningModel,
    lightning_damage: LightningDamage,
    hail: HailModel,
    hail_damage: HailDamage,
    ice: [IceCrystalIcingState; N_ENGINES],
    ice_rollback_risk: [f64; N_ENGINES],
    ice_flow_loss: [f64; N_ENGINES],
    ice_shedding: [bool; N_ENGINES],
    ash: [VolcanicAshState; N_ENGINES],
    ash_out: [Option<AshOutputs>; N_ENGINES],
    shear: WindShearModel,
    turbulence: TurbulenceModel,
    f_factor: f64,
    gusts: TurbulenceGusts,
    runway: Option<RunwayFrictionOutput>,

    /// Inputs `Truth` does not carry; see [`EnvironmentCommands`].
    pub commands: EnvironmentCommands,
}

impl Default for EnvironmentLive {
    fn default() -> Self {
        Self::new()
    }
}

impl EnvironmentLive {
    /// A fresh, undamaged aircraft with the default flight seed.
    pub fn new() -> Self {
        Self::with_seed(DEFAULT_SEED)
    }

    /// A fresh aircraft whose whole hazard history is determined by `seed`.
    /// The seed is taken once, here, and never re-read: two
    /// `EnvironmentLive::with_seed(s)` stepped through the same weather and
    /// the same `dt` values produce byte-identical hazards.
    pub fn with_seed(seed: u64) -> Self {
        Self {
            ids: Ids::resolve(),
            rng: Rng::new(seed),
            seed,
            birds: BirdStrikeModel::new(),
            bird_damage: BirdDamage::default(),
            lightning: LightningModel::new(),
            lightning_damage: LightningDamage::default(),
            hail: HailModel::new(),
            hail_damage: HailDamage::default(),
            ice: [IceCrystalIcingState::new(); N_ENGINES],
            ice_rollback_risk: [0.0; N_ENGINES],
            ice_flow_loss: [0.0; N_ENGINES],
            ice_shedding: [false; N_ENGINES],
            ash: [VolcanicAshState::new(); N_ENGINES],
            ash_out: [None; N_ENGINES],
            shear: WindShearModel::new(),
            turbulence: TurbulenceModel::new(),
            f_factor: 0.0,
            gusts: TurbulenceGusts::default(),
            runway: None,
            commands: EnvironmentCommands::default(),
        }
    }

    /// The seed this flight's hazards were generated from, for a replay
    /// header or a bug report.
    pub fn seed(&self) -> u64 {
        self.seed
    }

    /// Arm a scripted bird strike (the scenario/EFB side of the model).
    pub fn arm_bird_strike(&mut self, strike: bird_strike::ScriptedStrike) {
        self.birds.arm(strike);
    }

    /// Arm a scripted hail encounter on one target.
    pub fn arm_hail(&mut self, target: HailTarget, diameter_mm: Option<f64>, count: u32) {
        self.hail.trigger(target, diameter_mm, count);
    }

    /// Arm a scripted lightning strike.
    pub fn arm_lightning(&mut self, entry: Option<super::lightning::AttachPoint>, peak_current_ka: Option<f64>) {
        self.lightning.trigger(entry, peak_current_ka);
    }

    /// Which flight phase the bird-strike rate model should use, from what
    /// `Truth` and the commands actually say about the aircraft.
    fn phase(&self, truth: &Truth) -> Phase {
        let moving = truth.environment.tas_ms > 5.0;
        if truth.on_ground {
            return if moving { Phase::TakeoffRun } else { Phase::Taxi };
        }
        if truth.altitude_ft >= EN_ROUTE_FT {
            return Phase::EnRoute;
        }
        // Below the terminal-area boundary: gear out means approach or
        // landing, gear up means the climb out.
        if self.commands.gear_down {
            Phase::Approach
        } else {
            Phase::Climb
        }
    }
}

impl crate::deep::live::Area for EnvironmentLive {
    fn name(&self) -> &'static str {
        "environment"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        let dt = truth.dt_s.max(0.0);
        let env = &truth.environment;
        let convective = weather::lightning_convective_intensity(env);
        let hail_intensity = weather::hail_intensity(env);

        // ---- Bird strikes -------------------------------------------------
        self.birds.random_mode = self.commands.random_hazards;
        let flight = FlightState {
            tas_ms: env.tas_ms,
            altitude_agl_m: self.commands.altitude_agl_m,
            phase: self.phase(truth),
            month: self.commands.month.clamp(1, 12),
            night: self.commands.night,
            gear_down: self.commands.gear_down,
            n1_frac: truth.engine_n1_frac,
        };
        for outcome in self.birds.step(&flight, dt, &mut self.rng) {
            self.bird_damage.absorb(&outcome);
        }

        // ---- Lightning ----------------------------------------------------
        self.lightning.random_mode = self.commands.random_hazards;
        for slot in &mut self.lightning_damage.bus_upset_s {
            *slot = (*slot - dt).max(0.0);
        }
        if let Some(event) = self.lightning.step(convective, dt, &mut self.rng) {
            self.lightning_damage.radome = self.lightning_damage.radome.max(event.radome_damage_frac);
            self.lightning_damage.structure = self.lightning_damage.structure.max(event.structure_damage_frac);
            // A compass swing is needed after a strike: the deviation the
            // strike leaves is persistent, and a second strike can only
            // make it worse.
            self.lightning_damage.compass_error_deg = self.lightning_damage.compass_error_deg.max(event.compass_error_deg);
            for transient in &event.transients {
                let k = bus_kind_index(transient.bus);
                self.lightning_damage.bus_peak_volts[k] = self.lightning_damage.bus_peak_volts[k].max(transient.peak_volts);
                if transient.upset_likely {
                    self.lightning_damage.bus_upset_s[k] = BUS_UPSET_HOLD_S;
                }
            }
        }

        // ---- Hail ---------------------------------------------------------
        self.hail.random_mode = self.commands.random_hazards;
        let hail_state = HailFlightState { tas_ms: env.tas_ms, n1_frac: truth.engine_n1_frac };
        let outcomes = self.hail.step(hail_intensity, &hail_state, dt, &mut self.rng);
        if outcomes.is_empty() {
            // Nothing hit an engine this step, so nothing is going down a
            // core right now: the instantaneous flameout risk decays to
            // zero rather than latching on the last stone.
            self.hail_damage.engine_flameout_risk = [0.0; N_ENGINES];
        }
        for outcome in &outcomes {
            self.hail_damage.absorb(outcome);
        }

        // ---- Ice crystal icing --------------------------------------------
        let iwc = weather::ice_water_content_g_m3(env);
        for eng in 0..N_ENGINES {
            let out = self.ice[eng].step(&IceCrystalInputs {
                ice_water_content_g_m3: iwc,
                core_mass_flow_kg_s: self.commands.engine_core_mass_flow_kg_s[eng],
                warm_surface_temp_c: self.commands.engine_warm_surface_temp_c[eng],
                dt_s: dt,
            });
            self.ice_flow_loss[eng] = out.flow_capacity_loss_frac;
            self.ice_rollback_risk[eng] = out.rollback_risk_frac;
            self.ice_shedding[eng] = out.shedding_event;
        }

        // ---- Volcanic ash --------------------------------------------------
        for eng in 0..N_ENGINES {
            let out = self.ash[eng].step(&AshInputs {
                concentration_mg_m3: self.commands.ash_concentration_mg_m3,
                core_mass_flow_kg_s: self.commands.engine_core_mass_flow_kg_s[eng],
                ngv_gas_temp_c: self.commands.engine_ngv_gas_temp_c[eng],
                compressor_velocity_ms: self.commands.engine_compressor_velocity_ms[eng],
                tas_ms: env.tas_ms,
                dt_s: dt,
            });
            self.ash_out[eng] = Some(out);
        }

        // ---- Wind shear and turbulence -------------------------------------
        self.shear.random_mode = self.commands.random_hazards;
        let generated = self.shear.step(convective, self.commands.groundspeed_ms, dt, &mut self.rng);
        // A real measured wind field always beats a synthesised encounter.
        let shear_inputs = self.commands.measured_shear.or(generated);
        self.f_factor = shear_inputs.map_or(0.0, |i| wind_shear::f_factor(&i));

        self.gusts = match weather::turbulence_intensity(env) {
            Some(intensity) => self.turbulence.step(self.commands.altitude_agl_m, env.tas_ms, intensity, dt, &mut self.rng),
            None => TurbulenceGusts::default(),
        };

        // ---- Runway friction ------------------------------------------------
        self.runway = if truth.on_ground {
            let contaminant = weather::contaminant_from_weather(env.sat_c, env.precipitation_on_aircraft_ratio);
            let groundspeed_kt = self.commands.groundspeed_ms / weather_kt();
            Some(runway_contamination::friction(contaminant, MAIN_TYRE_PRESSURE_PSI, groundspeed_kt))
        } else {
            None
        };

        // ---- Armed failures: a floor under every damage state ---------------
        self.apply_armed(faults);
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        let b = |x: bool| if x { 1.0 } else { 0.0 };

        // ---- Bird strike ----------------------------------------------------
        for i in 0..N_ENGINES {
            let n = i + 1;
            out(&format!("ENV_BIRD_FAN_DAMAGE:{n}"), self.bird_damage.fan[i]);
            out(&format!("ENV_BIRD_CORE_FOD:{n}"), self.bird_damage.core[i]);
        }
        for i in 0..N_WINDSHIELD {
            out(&format!("ENV_BIRD_WINDSHIELD_DAMAGE:{}", i + 1), self.bird_damage.windshield[i]);
        }
        out("ENV_BIRD_RADOME_DAMAGE", self.bird_damage.radome);
        out("ENV_BIRD_NOSE_GEAR_DAMAGE", self.bird_damage.nose_gear);
        out("ENV_BIRD_WING_LE_DRAG", self.bird_damage.wing_le_drag.iter().copied().fold(0.0, f64::max));
        for i in 0..N_PROBE {
            out(&format!("ENV_BIRD_PROBE_BLOCKED:{}", i + 1), self.bird_damage.probe_blocked[i]);
        }

        // ---- Lightning -------------------------------------------------------
        out("ENV_LTG_RADOME_DAMAGE", self.lightning_damage.radome);
        out("ENV_LTG_STRUCTURE_DAMAGE", self.lightning_damage.structure);
        out("ENV_LTG_COMPASS_ERROR_DEG", self.lightning_damage.compass_error_deg);
        for (k, kind) in BUS_KINDS.iter().enumerate() {
            out(&format!("ENV_LTG_BUS_UPSET:{kind}"), b(self.lightning_damage.bus_upset_s[k] > 0.0));
            out(&format!("ENV_LTG_BUS_PEAK_VOLTS:{kind}"), self.lightning_damage.bus_peak_volts[k]);
        }

        // ---- Hail -------------------------------------------------------------
        out("ENV_HAIL_RADOME_DAMAGE", self.hail_damage.radome);
        for i in 0..N_WINDSHIELD {
            let n = i + 1;
            out(&format!("ENV_HAIL_WINDSHIELD_DAMAGE:{n}"), self.hail_damage.windshield[i]);
            out(&format!("ENV_HAIL_WINDOW_HEAT_FAULT:{n}"), b(self.hail_damage.window_heat_fault[i]));
        }
        for i in 0..N_WING_LE {
            let n = i + 1;
            out(&format!("ENV_HAIL_WING_LE_DAMAGE:{n}"), self.hail_damage.wing_le[i]);
            out(&format!("ENV_HAIL_SLAT_JAM_RISK:{n}"), self.hail_damage.slat_jam_risk[i]);
        }
        for i in 0..N_ENGINES {
            let n = i + 1;
            out(&format!("ENV_HAIL_FAN_DAMAGE:{n}"), self.hail_damage.engine_fan[i]);
            out(&format!("ENV_HAIL_COMPRESSOR_EFF_LOSS:{n}"), self.hail_damage.engine_compressor_loss[i]);
            out(&format!("ENV_HAIL_FLAMEOUT_RISK:{n}"), self.hail_damage.engine_flameout_risk[i]);
            out(&format!("ENV_HAIL_NACELLE_DAMAGE:{n}"), self.hail_damage.nacelle[i]);
        }
        for i in 0..N_PROBE {
            out(&format!("ENV_HAIL_PROBE_DAMAGE:{}", i + 1), self.hail_damage.probe[i]);
        }

        // ---- Volcanic ash ------------------------------------------------------
        let mut worst_odour = 0.0_f64;
        for i in 0..N_ENGINES {
            let n = i + 1;
            let o = self.ash_out[i];
            out(&format!("ENV_ASH_FLOW_CAPACITY_LOSS:{n}"), o.map_or(0.0, |o| o.flow_capacity_loss_frac));
            out(&format!("ENV_ASH_COMPRESSOR_EFF_LOSS:{n}"), o.map_or(0.0, |o| o.compressor_efficiency_loss_frac));
            out(&format!("ENV_ASH_FLAMEOUT_RISK:{n}"), o.map_or(0.0, |o| o.flameout_risk_frac));
            out(&format!("ENV_ASH_RELIGHT_POSSIBLE:{n}"), b(o.map_or(true, |o| o.relight_possible)));
            if let Some(o) = o {
                worst_odour = worst_odour.max(o.cabin_odor_intensity);
            }
        }
        // The cabin smells the ash once, not four times.
        out("ENV_ASH_CABIN_ODOR", worst_odour);
        out("ENV_ASH_WINDSHIELD_VISIBILITY_LOSS", self.ash_out[0].map_or(0.0, |o| o.windshield_visibility_loss_frac));
        out("ENV_ASH_PITOT_BLOCKED", self.ash_out[0].map_or(0.0, |o| o.pitot_blockage_frac));

        // ---- Ice crystal icing --------------------------------------------------
        for i in 0..N_ENGINES {
            let n = i + 1;
            out(&format!("ENV_ICE_ROLLBACK_RISK:{n}"), self.ice_rollback_risk[i]);
            out(&format!("ENV_ICE_FLOW_CAPACITY_LOSS:{n}"), self.ice_flow_loss[i]);
            out(&format!("ENV_ICE_SHEDDING:{n}"), b(self.ice_shedding[i]));
        }

        // ---- Wind shear, turbulence, runway --------------------------------------
        out("ENV_F_FACTOR", self.f_factor);
        out("ENV_TURBULENCE_GUST_U_MS", self.gusts.u_ms);
        out("ENV_TURBULENCE_GUST_V_MS", self.gusts.v_ms);
        out("ENV_TURBULENCE_GUST_W_MS", self.gusts.w_ms);
        out("ENV_RWY_HYDROPLANING", b(self.runway.map_or(false, |r| r.hydroplaning)));
        out("ENV_RWY_MU_EFFECTIVE", self.runway.map_or(0.0, |r| r.mu_effective));
        out("ENV_RWY_CONDITION_CODE", self.runway.map_or(6.0, |r| r.rwy_cc as f64));
    }
}

/// kt -> m/s, from `integration::weather_truth`'s own NIST conversion, so
/// the groundspeed handed to `runway_contamination::friction` in knots is
/// the same nautical mile the rest of the crate uses.
fn weather_kt() -> f64 {
    crate::deep::integration::weather_truth::KT_TO_MS
}

impl EnvironmentLive {
    /// Apply each armed failure as a floor under the damage state it names.
    ///
    /// See the module doc: this area's failures name a hazard's *outcome*,
    /// so arming one means "this element is damaged this badly". The armed
    /// magnitude and whatever the stochastic models have accumulated are
    /// the same physical quantity, and the worse of the two is the state
    /// the aircraft is in.
    fn apply_armed(&mut self, faults: &Faults) {
        let floor = |slot: &mut f64, m: f64| *slot = slot.max(m);
        let floor_all = |slots: &mut [f64], m: f64| {
            if m > 0.0 {
                for s in slots.iter_mut() {
                    *s = s.max(m);
                }
            }
        };

        // Bird strike.
        floor_all(&mut self.bird_damage.fan, faults.get(self.ids.bird_fan));
        floor_all(&mut self.bird_damage.core, faults.get(self.ids.bird_core));
        floor_all(&mut self.bird_damage.windshield, faults.get(self.ids.bird_windshield));
        floor(&mut self.bird_damage.radome, faults.get(self.ids.bird_radome));
        floor_all(&mut self.bird_damage.wing_le_drag, faults.get(self.ids.bird_wing_le));
        floor(&mut self.bird_damage.nose_gear, faults.get(self.ids.bird_nose_gear));
        // The probe failure is registered as binary ("any strike on a probe
        // is assumed to disable it"), so it blocks past half.
        let probe = faults.get(self.ids.bird_probe);
        if probe >= 0.5 {
            floor_all(&mut self.bird_damage.probe_blocked, 1.0);
        }

        // Lightning.
        floor(&mut self.lightning_damage.radome, faults.get(self.ids.ltg_radome));
        floor(&mut self.lightning_damage.structure, faults.get(self.ids.ltg_structure));
        // `registry.rs`: the compass deviation is a GENERIC 0..10 degrees.
        floor(&mut self.lightning_damage.compass_error_deg, faults.get(self.ids.ltg_compass) * MAX_COMPASS_DEVIATION_DEG);
        let bus = faults.get(self.ids.ltg_bus);
        if bus > 0.0 {
            for k in 0..BUS_KINDS.len() {
                self.lightning_damage.bus_peak_volts[k] = self.lightning_damage.bus_peak_volts[k].max(bus * MAX_BUS_TRANSIENT_V);
                if bus * MAX_BUS_TRANSIENT_V >= UPSET_THRESHOLD_V {
                    self.lightning_damage.bus_upset_s[k] = BUS_UPSET_HOLD_S;
                }
            }
        }

        // Hail.
        floor(&mut self.hail_damage.radome, faults.get(self.ids.hail_radome));
        let hail_windshield = faults.get(self.ids.hail_windshield);
        floor_all(&mut self.hail_damage.windshield, hail_windshield);
        if hail_windshield > WINDOW_HEAT_FAULT_SEVERITY {
            // `registry.rs`: the conductive heating film faults above a
            // GENERIC 0.3 severity.
            for f in &mut self.hail_damage.window_heat_fault {
                *f = true;
            }
        }
        let hail_le = faults.get(self.ids.hail_wing_le);
        floor_all(&mut self.hail_damage.wing_le, hail_le);
        // `registry.rs`: slat jam risk "ramps above 0.5 damage".
        floor_all(&mut self.hail_damage.slat_jam_risk, ((hail_le - 0.5) / 0.5).clamp(0.0, 1.0));
        let hail_engine = faults.get(self.ids.hail_engine);
        floor_all(&mut self.hail_damage.engine_fan, hail_engine);
        floor_all(&mut self.hail_damage.engine_compressor_loss, hail_engine * MAX_HAIL_COMPRESSOR_LOSS);
        floor_all(&mut self.hail_damage.engine_flameout_risk, hail_engine);
        floor_all(&mut self.hail_damage.nacelle, faults.get(self.ids.hail_nacelle));
        floor_all(&mut self.hail_damage.probe, faults.get(self.ids.hail_probe));

        // Volcanic ash and ice crystal icing act on their models' outputs.
        let ash_glassing = faults.get(self.ids.ash_glassing);
        let ash_erosion = faults.get(self.ids.ash_erosion);
        let ash_windshield = faults.get(self.ids.ash_windshield);
        let ash_pitot = faults.get(self.ids.ash_pitot);
        for slot in &mut self.ash_out {
            let mut o = slot.unwrap_or_else(|| VolcanicAshState::new().step(&AshInputs { concentration_mg_m3: 0.0, core_mass_flow_kg_s: 0.0, ngv_gas_temp_c: 0.0, compressor_velocity_ms: 0.0, tas_ms: 0.0, dt_s: 0.0 }));
            o.flow_capacity_loss_frac = o.flow_capacity_loss_frac.max(ash_glassing);
            o.compressor_efficiency_loss_frac = o.compressor_efficiency_loss_frac.max(ash_erosion * MAX_ASH_EROSION_LOSS);
            o.windshield_visibility_loss_frac = o.windshield_visibility_loss_frac.max(ash_windshield);
            o.pitot_blockage_frac = o.pitot_blockage_frac.max(ash_pitot);
            // Flameout risk follows the blockage that now stands
            // (`volcanic_ash.rs`'s own weighting of the accumulated half).
            o.flameout_risk_frac = o.flameout_risk_frac.max(o.flow_capacity_loss_frac * 0.5);
            *slot = Some(o);
        }

        let ice_blockage = faults.get(self.ids.ice_accretion);
        let ice_rollback = faults.get(self.ids.ice_rollback);
        for eng in 0..N_ENGINES {
            self.ice_flow_loss[eng] = self.ice_flow_loss[eng].max(ice_blockage);
            // `registry.rs`: rollback risk is the blockage squared.
            self.ice_rollback_risk[eng] = self.ice_rollback_risk[eng].max(ice_rollback).max(self.ice_flow_loss[eng].powi(2));
        }

        // Runway friction: the armed magnitude is `1 - mu/0.40`, so it
        // names the friction the runway actually has.
        let friction_loss = faults.get(self.ids.runway_friction);
        if friction_loss > 0.0 {
            let mu = (DRY_MU_REFERENCE * (1.0 - friction_loss)).max(0.0);
            let existing = self.runway;
            let mut r = existing.unwrap_or_else(|| runway_contamination::friction(super::runway_contamination::Contaminant::Dry, MAIN_TYRE_PRESSURE_PSI, self.commands.groundspeed_ms / weather_kt()));
            if mu < r.mu_effective {
                r.mu_effective = mu;
                // Full loss is the hydroplaning case `registry.rs` names.
                r.hydroplaning = r.hydroplaning || friction_loss >= HYDROPLANING_FRICTION_LOSS;
            }
            self.runway = Some(r);
        }
    }
}

/// `registry.rs`: the lightning compass deviation is a GENERIC 0..10 deg.
const MAX_COMPASS_DEVIATION_DEG: f64 = 10.0;
/// The transient a fully-armed lightning bus failure puts on a bus, V.
/// From `lightning.rs`'s own scale: 200 kA (the ARP5412 reference peak) at
/// the worst exposure factor (0.35, the nacelle-mounted FADEC) and 1 V/kA
/// gives 70 V, comfortably over the upset threshold.
const MAX_BUS_TRANSIENT_V: f64 = 70.0;
/// `lightning.rs`'s own GENERIC DO-160-style upset threshold, V, restated
/// here because the module keeps it private.
const UPSET_THRESHOLD_V: f64 = 50.0;
/// `registry.rs`: the windshield's conductive heating film faults above a
/// GENERIC 0.3 severity.
const WINDOW_HEAT_FAULT_SEVERITY: f64 = 0.3;
/// `registry.rs`: hail compressor efficiency loss is capped at a GENERIC
/// 0.25, the same ceiling `hail.rs` applies.
const MAX_HAIL_COMPRESSOR_LOSS: f64 = 0.25;
/// `volcanic_ash.rs`'s own ceiling on erosion efficiency loss.
const MAX_ASH_EROSION_LOSS: f64 = 0.25;
/// `registry.rs`: the runway friction failure's magnitude is
/// `1 - mu/0.40`, so 0.40 is the dry reference.
const DRY_MU_REFERENCE: f64 = 0.40;
/// `registry.rs`: the magnitude reaches "~0.9 in full dynamic
/// hydroplaning", so that is where the armed failure hydroplanes.
const HYDROPLANING_FRICTION_LOSS: f64 = 0.9;

/// This area's live system, with the default flight seed.
pub fn live_system() -> Box<dyn crate::deep::live::Area> {
    Box::new(EnvironmentLive::new())
}

/// This area's live system with a chosen seed, so a flight's random
/// hazards can be reproduced.
pub fn live_system_with_seed(seed: u64) -> Box<dyn crate::deep::live::Area> {
    Box::new(EnvironmentLive::with_seed(seed))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::fuel::live::test_support::collect_vars;
    use crate::deep::integration::weather_truth::EnvironmentTruth;
    use crate::deep::live::Area as _;
    use crate::deep::weather::{WeatherCloudLayer, WeatherSample};
    use std::collections::BTreeMap;

    fn published(area: &dyn crate::deep::live::Area) -> BTreeMap<String, f64> {
        let mut map = BTreeMap::new();
        area.publish(&mut |name, value| {
            map.insert(name.to_string(), value);
        });
        map
    }

    /// A deep, cold cumulonimbus: the weather every random hazard here
    /// keys off.
    fn thunderstorm() -> Truth {
        let mut clouds = [WeatherCloudLayer::default(); 3];
        clouds[0] = WeatherCloudLayer { cloud_type: 3.0, coverage: 1.0, alt_base_m: 500.0, alt_top_m: 13_000.0 };
        Truth {
            dt_s: 0.1,
            environment: EnvironmentTruth {
                sat_c: -30.0,
                leading_edge_c: -20.0,
                ambient_pressure_pa: 40_000.0,
                tas_ms: 200.0,
                precipitation_on_aircraft_ratio: 0.9,
                weather: Some(WeatherSample { precip_rate_alt: 0.9, precip_rate: 0.9, turbulence_alt: 0.9, clouds, detailed: true }),
            },
            altitude_ft: 20_000.0,
            on_ground: false,
            engine_running: [true; 4],
            engine_n1_frac: [0.8; 4],
            ..Truth::default()
        }
    }

    fn run(live: &mut EnvironmentLive, truth: &Truth, faults: &Faults, seconds: f64) -> BTreeMap<String, f64> {
        let steps = (seconds / truth.dt_s).ceil() as usize;
        for _ in 0..steps.max(1) {
            live.tick(truth, faults);
        }
        published(live)
    }

    #[test]
    fn an_undamaged_aircraft_in_clear_air_publishes_every_trigger_variable_at_zero() {
        let mut live = EnvironmentLive::new();
        let out = run(&mut live, &Truth::default(), &Faults::default(), 1.0);
        for name in [
            "ENV_BIRD_FAN_DAMAGE:1",
            "ENV_BIRD_RADOME_DAMAGE",
            "ENV_BIRD_NOSE_GEAR_DAMAGE",
            "ENV_BIRD_WINDSHIELD_DAMAGE:1",
            "ENV_BIRD_PROBE_BLOCKED:1",
            "ENV_LTG_RADOME_DAMAGE",
            "ENV_LTG_COMPASS_ERROR_DEG",
            "ENV_LTG_BUS_UPSET:EngineFadec",
            "ENV_HAIL_RADOME_DAMAGE",
            "ENV_HAIL_WINDSHIELD_DAMAGE:1",
            "ENV_HAIL_WINDOW_HEAT_FAULT:1",
            "ENV_HAIL_SLAT_JAM_RISK:1",
            "ENV_HAIL_FLAMEOUT_RISK:1",
            "ENV_ASH_FLOW_CAPACITY_LOSS:1",
            "ENV_ASH_COMPRESSOR_EFF_LOSS:1",
            "ENV_ASH_FLAMEOUT_RISK:1",
            "ENV_ASH_CABIN_ODOR",
            "ENV_ICE_ROLLBACK_RISK:1",
            "ENV_F_FACTOR",
            "ENV_RWY_HYDROPLANING",
        ] {
            assert_eq!(out.get(name), Some(&0.0), "{name} should be published and clean on an undamaged aircraft");
        }
    }

    #[test]
    fn every_variable_this_areas_alerts_trigger_on_is_published_by_this_live_system() {
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        let mut names = Vec::new();
        for alert in &reg.alerts {
            collect_vars(&alert.trigger, &mut names);
        }
        let mut live = EnvironmentLive::new();
        live.tick(&Truth::default(), &Faults::default());
        let out = published(&live);
        for name in names {
            // The alerts also read cockpit controls and other areas'
            // outputs (throttle position, engine masters, airspeed, the
            // WXR power switch); those are not this area's to publish.
            if !name.starts_with("ENV_") {
                continue;
            }
            assert!(out.contains_key(&name), "alert trigger reads {name}, which nothing publishes");
        }
    }

    // ---- Determinism -------------------------------------------------------

    #[test]
    fn the_same_seed_and_the_same_weather_reproduce_a_flights_hazards_exactly() {
        let truth = thunderstorm();
        let mut a = EnvironmentLive::with_seed(12_345);
        a.commands.random_hazards = true;
        a.commands.altitude_agl_m = 3000.0;
        a.commands.groundspeed_ms = 200.0;
        a.commands.month = 9;
        let mut b = EnvironmentLive::with_seed(12_345);
        b.commands = a.commands;

        let first = run(&mut a, &truth, &Faults::default(), 600.0);
        let second = run(&mut b, &truth, &Faults::default(), 600.0);
        assert_eq!(first, second, "the same seed must replay a flight exactly");
        assert_eq!(a.seed(), b.seed());
    }

    #[test]
    fn a_different_seed_gives_a_different_flight() {
        let truth = thunderstorm();
        let mut a = EnvironmentLive::with_seed(1);
        a.commands.random_hazards = true;
        a.commands.altitude_agl_m = 3000.0;
        a.commands.groundspeed_ms = 200.0;
        let mut b = EnvironmentLive::with_seed(2);
        b.commands = a.commands;
        let first = run(&mut a, &truth, &Faults::default(), 600.0);
        let second = run(&mut b, &truth, &Faults::default(), 600.0);
        assert_ne!(first, second, "two different seeds in a thunderstorm should not produce identical hazards");
    }

    #[test]
    fn nothing_random_happens_at_all_unless_the_random_generators_are_armed() {
        let truth = thunderstorm();
        let mut live = EnvironmentLive::new(); // random_hazards defaults off
        let out = run(&mut live, &truth, &Faults::default(), 3600.0);
        assert_eq!(out.get("ENV_BIRD_RADOME_DAMAGE"), Some(&0.0));
        assert_eq!(out.get("ENV_HAIL_RADOME_DAMAGE"), Some(&0.0));
        assert_eq!(out.get("ENV_LTG_RADOME_DAMAGE"), Some(&0.0));
    }

    // ---- Armed failures ------------------------------------------------------

    #[test]
    fn arming_the_bird_strike_fan_failure_moves_the_variable_its_alert_triggers_on() {
        // registry.rs: "0 no damage .. 1 destructive blade fracture", and
        // ENG 1 FAN DAMAGE triggers above 0.3.
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        let alert = reg.alerts.iter().find(|a| a.key == "ENV_ENG_1_BIRD_FAN_DAMAGE").expect("registered");

        let mut live = EnvironmentLive::new();
        let id = live.ids.bird_fan;

        let mild = run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 0.2)]), 1.0);
        assert_eq!(mild.get("ENV_BIRD_FAN_DAMAGE:1"), Some(&0.2));
        assert!(!alert.trigger.eval(&|n: &str| mild.get(n).copied().unwrap_or(0.0)), "0.2 is below the 0.3 trigger");

        let mut live = EnvironmentLive::new();
        let bad = run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 0.8)]), 1.0);
        assert_eq!(bad.get("ENV_BIRD_FAN_DAMAGE:1"), Some(&0.8));
        assert!(alert.trigger.eval(&|n: &str| bad.get(n).copied().unwrap_or(0.0)), "0.8 must raise ENG 1 FAN DAMAGE");
    }

    #[test]
    fn a_scripted_bird_strike_damages_the_airframe_and_the_damage_persists() {
        let mut live = EnvironmentLive::with_seed(7);
        live.commands.altitude_agl_m = 50.0;
        live.commands.gear_down = true;
        live.arm_bird_strike(bird_strike::ScriptedStrike {
            bird: bird_strike::BirdClass::Large,
            bird_count: 4,
            target: Some(BirdTarget::EngineInlet(0)),
            condition: bird_strike::TriggerCondition::Now,
        });
        let truth = Truth { dt_s: 0.1, on_ground: false, engine_n1_frac: [0.95; 4], environment: EnvironmentTruth { tas_ms: 90.0, ..Truth::default().environment }, ..Truth::default() };
        let out = run(&mut live, &truth, &Faults::default(), 1.0);
        assert!(out["ENV_BIRD_FAN_DAMAGE:1"] > 0.0, "four large birds into a fan at 95% N1 must damage it");
        assert_eq!(out.get("ENV_BIRD_FAN_DAMAGE:2"), Some(&0.0), "the other engines are untouched");

        // The damage is persistent: another minute of clear air does not
        // heal a fractured blade.
        let later = run(&mut live, &Truth::default(), &Faults::default(), 60.0);
        assert_eq!(later["ENV_BIRD_FAN_DAMAGE:1"], out["ENV_BIRD_FAN_DAMAGE:1"]);
    }

    #[test]
    fn a_scripted_hail_encounter_damages_the_radome_and_attenuates_the_radar() {
        let mut live = EnvironmentLive::with_seed(11);
        let truth = thunderstorm();
        live.arm_hail(HailTarget::Radome, Some(40.0), 30);
        let out = run(&mut live, &truth, &Faults::default(), 1.0);
        assert!(out["ENV_HAIL_RADOME_DAMAGE"] > 0.0, "40 mm hail at 200 m/s must damage a radome");

        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        let alert = reg.alerts.iter().find(|a| a.key == "ENV_HAIL_RADOME_DAMAGE").expect("registered");
        let armed = run(&mut EnvironmentLive::new(), &Truth::default(), &Faults::from_pairs([(live.ids.hail_radome, 0.9)]), 1.0);
        assert!(alert.trigger.eval(&|n: &str| armed.get(n).copied().unwrap_or(0.0)));
    }

    #[test]
    fn a_lightning_strike_upsets_the_fadec_bus_and_the_upset_clears_by_itself() {
        let mut live = EnvironmentLive::with_seed(3);
        let truth = thunderstorm();
        live.arm_lightning(Some(super::super::lightning::AttachPoint::NoseRadome), Some(200.0));
        let struck = run(&mut live, &truth, &Faults::default(), 0.1);
        assert_eq!(struck.get("ENV_LTG_BUS_UPSET:EngineFadec"), Some(&1.0), "a 200 kA strike must upset the nacelle-mounted FADEC");
        assert!(struck["ENV_LTG_COMPASS_ERROR_DEG"] > 0.0);

        // The box reboots; the compass deviation does not go away.
        let later = run(&mut live, &truth, &Faults::default(), BUS_UPSET_HOLD_S + 5.0);
        assert_eq!(later.get("ENV_LTG_BUS_UPSET:EngineFadec"), Some(&0.0), "the upset must clear once the equipment has restarted");
        assert_eq!(later["ENV_LTG_COMPASS_ERROR_DEG"], struck["ENV_LTG_COMPASS_ERROR_DEG"], "a compass swing is still required");
    }

    #[test]
    fn arming_the_lightning_bus_transient_raises_the_fadec_transient_caution() {
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        let alert = reg.alerts.iter().find(|a| a.key == "ENV_LTG_FADEC_TRANSIENT").expect("registered");
        let mut live = EnvironmentLive::new();
        let id = live.ids.ltg_bus;
        let out = run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 1.0)]), 0.1);
        assert!(alert.trigger.eval(&|n: &str| out.get(n).copied().unwrap_or(0.0)));
    }

    #[test]
    fn ice_crystal_icing_accretes_in_a_glaciated_core_and_rolls_the_engine_back() {
        // The weather half is real: a cold, deep cumulonimbus carries the
        // ice water content (`environment_events_adapter`). The core mass
        // flow is the caller's, since nothing in `Truth` gives it.
        let truth = thunderstorm();
        let mut live = EnvironmentLive::new();
        live.commands.engine_core_mass_flow_kg_s = [40.0; 4];
        live.commands.engine_warm_surface_temp_c = [2.0; 4]; // in the adherence window
        let out = run(&mut live, &truth, &Faults::default(), 600.0);
        assert!(out["ENV_ICE_FLOW_CAPACITY_LOSS:1"] > 0.0, "a glaciated core must accrete");
        assert!(out["ENV_ICE_ROLLBACK_RISK:1"] > 0.0);

        // Same weather, no core flow: nothing goes through the engine, so
        // nothing sticks in it.
        let mut dry = EnvironmentLive::new();
        let none = run(&mut dry, &truth, &Faults::default(), 600.0);
        assert_eq!(none.get("ENV_ICE_FLOW_CAPACITY_LOSS:1"), Some(&0.0));
    }

    #[test]
    fn arming_the_ice_rollback_failure_raises_the_rollback_caution() {
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        let alert = reg.alerts.iter().find(|a| a.key == "ENV_ICE_ENG_1_ROLLBACK").expect("registered");
        let mut live = EnvironmentLive::new();
        let id = live.ids.ice_rollback;
        let out = run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 0.5)]), 1.0);
        assert_eq!(out.get("ENV_ICE_ROLLBACK_RISK:1"), Some(&0.5));
        assert!(alert.trigger.eval(&|n: &str| out.get(n).copied().unwrap_or(0.0)));
    }

    #[test]
    fn a_volcanic_ash_encounter_glasses_the_ngvs_and_the_cabin_smells_it() {
        let mut truth = thunderstorm();
        truth.dt_s = 1.0;
        let mut live = EnvironmentLive::new();
        live.commands.ash_concentration_mg_m3 = 4.0; // ICAO "high" contamination
        live.commands.engine_core_mass_flow_kg_s = [40.0; 4];
        live.commands.engine_ngv_gas_temp_c = [1400.0; 4]; // well above ash's melting range
        live.commands.engine_compressor_velocity_ms = [300.0; 4];
        let out = run(&mut live, &truth, &Faults::default(), 600.0);
        assert!(out["ENV_ASH_FLOW_CAPACITY_LOSS:1"] > 0.0, "molten ash must deposit on the NGVs");
        assert!(out["ENV_ASH_CABIN_ODOR"] > 0.1);
        assert!(out["ENV_ASH_FLAMEOUT_RISK:1"] > 0.0);

        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        let alert = reg.alerts.iter().find(|a| a.key == "ENV_ASH_ENCOUNTER").expect("registered");
        assert!(alert.trigger.eval(&|n: &str| out.get(n).copied().unwrap_or(0.0)));
    }

    #[test]
    fn a_wet_runway_at_speed_hydroplanes_and_a_dry_one_does_not() {
        let mut wet = Truth {
            on_ground: true,
            environment: EnvironmentTruth { sat_c: 15.0, precipitation_on_aircraft_ratio: 0.9, ..Truth::default().environment },
            ..Truth::default()
        };
        wet.environment.tas_ms = 70.0;
        let mut live = EnvironmentLive::new();
        // Above the NASA/Horne dynamic hydroplaning speed for a 218 psi
        // tyre (9*sqrt(p) ~ 133 kt).
        live.commands.groundspeed_ms = 150.0 * weather_kt();
        let out = run(&mut live, &wet, &Faults::default(), 1.0);
        assert_eq!(out.get("ENV_RWY_HYDROPLANING"), Some(&1.0));
        assert!(out["ENV_RWY_MU_EFFECTIVE"] < 0.4);

        let dry = Truth { on_ground: true, ..Truth::default() };
        let mut live = EnvironmentLive::new();
        live.commands.groundspeed_ms = 150.0 * weather_kt();
        let out = run(&mut live, &dry, &Faults::default(), 1.0);
        assert_eq!(out.get("ENV_RWY_HYDROPLANING"), Some(&0.0));
        assert!((out["ENV_RWY_MU_EFFECTIVE"] - 0.4).abs() < 0.05);
    }

    #[test]
    fn a_measured_headwind_loss_raises_the_windshear_warning() {
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        let alert = reg.alerts.iter().find(|a| a.key == "ENV_WINDSHEAR_WARNING").expect("registered");

        let mut live = EnvironmentLive::new();
        let truth = Truth { dt_s: 0.1, on_ground: false, environment: EnvironmentTruth { tas_ms: 75.0, ..Truth::default().environment }, ..Truth::default() };
        live.commands.measured_shear = Some(wind_shear::WindShearInputs { headwind_rate_ms2: -3.0, downdraft_ms: 8.0, tas_ms: 75.0 });
        let out = run(&mut live, &truth, &Faults::default(), 1.0);
        assert!(out["ENV_F_FACTOR"] > wind_shear::F_FACTOR_HAZARD_THRESHOLD, "f-factor {}", out["ENV_F_FACTOR"]);
        assert!(alert.trigger.eval(&|n: &str| out.get(n).copied().unwrap_or(0.0)));
    }

    #[test]
    fn nothing_divides_by_zero_at_rest_with_zero_dt() {
        let mut live = EnvironmentLive::new();
        live.tick(&Truth { dt_s: 0.0, ..Truth::default() }, &Faults::default());
        for (name, value) in published(&live) {
            assert!(value.is_finite(), "{name} is not finite");
        }
    }

    #[test]
    fn every_registered_failure_is_consumed_by_the_live_system() {
        let mut reg = Registry::default();
        super::super::registry::register(&mut reg);
        let ids = Ids::resolve();
        let mut consumed = vec![
            ids.bird_fan,
            ids.bird_core,
            ids.bird_windshield,
            ids.bird_radome,
            ids.bird_wing_le,
            ids.bird_nose_gear,
            ids.bird_probe,
            ids.ltg_radome,
            ids.ltg_structure,
            ids.ltg_compass,
            ids.ltg_bus,
            ids.hail_radome,
            ids.hail_windshield,
            ids.hail_wing_le,
            ids.hail_engine,
            ids.hail_probe,
            ids.hail_nacelle,
            ids.ash_glassing,
            ids.ash_erosion,
            ids.ash_windshield,
            ids.ash_pitot,
            ids.ice_accretion,
            ids.ice_rollback,
            ids.runway_friction,
        ];
        consumed.sort_unstable();
        consumed.dedup();
        let registered: Vec<u64> = reg.failures.iter().map(|f| f.id).collect();
        assert_eq!(consumed.len(), registered.len(), "every environment failure should be consumed");
        for id in registered {
            assert!(consumed.contains(&id), "failure {id} is registered but never read by the live system");
        }
    }
}
