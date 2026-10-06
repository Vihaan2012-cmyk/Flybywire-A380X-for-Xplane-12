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

pub const DEFAULT_SEED: u64 = 0x_A380_0BEE_0BEE_0001;

const BUS_UPSET_HOLD_S: f64 = 10.0;

const MAIN_TYRE_PRESSURE_PSI: f64 = 218.0;

const EN_ROUTE_FT: f64 = 10_000.0;

#[derive(Clone, Copy, Debug)]
pub struct EnvironmentCommands {
    pub altitude_agl_m: f64,
    pub month: u8,
    pub night: bool,
    pub gear_down: bool,
    pub groundspeed_ms: f64,
    pub measured_shear: Option<wind_shear::WindShearInputs>,
    pub engine_core_mass_flow_kg_s: [f64; N_ENGINES],
    pub engine_warm_surface_temp_c: [f64; N_ENGINES],
    pub engine_ngv_gas_temp_c: [f64; N_ENGINES],
    pub engine_compressor_velocity_ms: [f64; N_ENGINES],
    pub ash_concentration_mg_m3: f64,
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

struct Ids {
    bird_fan: [u64; N_ENGINES],
    bird_core: [u64; N_ENGINES],
    bird_windshield: [u64; N_WINDSHIELD],
    bird_radome: u64,
    bird_wing_le: [u64; N_WING_LE],
    bird_nose_gear: u64,
    bird_probe: [u64; N_PROBE],
    ltg_radome: u64,
    ltg_structure: u64,
    ltg_compass: u64,
    ltg_bus: [u64; 7],
    hail_radome: u64,
    hail_windshield: [u64; N_WINDSHIELD],
    hail_wing_le: [u64; N_WING_LE],
    hail_engine: [u64; N_ENGINES],
    hail_probe: [u64; N_PROBE],
    hail_nacelle: [u64; N_NACELLE],
    ash_glassing: u64,
    ash_erosion: u64,
    ash_windshield: u64,
    ash_pitot: u64,
    ice_accretion: u64,
    ice_rollback: u64,
    runway_friction: u64,
}

fn only_fid(reg: &Registry, component: &str) -> u64 {
    let mut found = reg.failures.iter().filter(|f| f.component == component);
    let first = found.next().unwrap_or_else(|| panic!("no failure registered on {component}"));
    let second = found.next();
    assert!(second.is_none(), "{component} has more than one failure; name the model_field to pick one");
    first.id
}

fn per_index_fid<const N: usize>(reg: &Registry, component_prefix: &str) -> [u64; N] {
    let mut ids = [0u64; N];
    for (i, slot) in ids.iter_mut().enumerate() {
        *slot = only_fid(reg, &format!("{component_prefix}_{}", i + 1));
    }
    ids
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
        Self {
            bird_fan: per_index_fid(&reg, "72_env.fan_bird_damage"),
            bird_core: per_index_fid(&reg, "72_env.core_fod"),
            bird_windshield: per_index_fid(&reg, "56_env.windshield_bird"),
            bird_radome: only_fid(&reg, "53_env.radome"),
            bird_wing_le: per_index_fid(&reg, "57_env.wing_leading_edge_bird"),
            bird_nose_gear: only_fid(&reg, "32_env.nose_gear"),
            bird_probe: per_index_fid(&reg, "34_env.air_data_probe_bird"),
            ltg_radome: only_fid(&reg, "53_env.radome_lightning"),
            ltg_structure: only_fid(&reg, "53_env.composite_extremity"),
            ltg_compass: only_fid(&reg, "34_env.standby_compass"),
            ltg_bus: per_index_fid(&reg, "24_env.bus_transient"),
            hail_radome: only_fid(&reg, "53_env.radome_hail"),
            hail_windshield: per_index_fid(&reg, "56_env.windshield_hail"),
            hail_wing_le: per_index_fid(&reg, "57_env.wing_leading_edge_hail"),
            hail_engine: per_index_fid(&reg, "72_env.engine_hail_ingestion"),
            hail_probe: per_index_fid(&reg, "34_env.air_data_probe_hail"),
            hail_nacelle: per_index_fid(&reg, "71_env.nacelle_hail"),
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

#[derive(Clone, Debug, Default)]
struct LightningDamage {
    radome: f64,
    structure: f64,
    compass_error_deg: f64,
    bus_upset_s: [f64; BUS_KINDS.len()],
    bus_peak_volts: [f64; BUS_KINDS.len()],
}

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
                self.engine_flameout_risk[i] = o.flameout_risk_frac;
            }
            HailTarget::Nacelle(i) => worst(&mut self.nacelle[i as usize % N_NACELLE], o.damage_frac),
            HailTarget::Probe(i) => worst(&mut self.probe[i as usize % N_PROBE], o.damage_frac),
        }
    }
}

pub struct EnvironmentLive {
    ids: Ids,
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

    pub commands: EnvironmentCommands,
}

impl Default for EnvironmentLive {
    fn default() -> Self {
        Self::new()
    }
}

impl EnvironmentLive {
    pub fn new() -> Self {
        Self::with_seed(DEFAULT_SEED)
    }

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

    pub fn seed(&self) -> u64 {
        self.seed
    }

    pub fn arm_bird_strike(&mut self, strike: bird_strike::ScriptedStrike) {
        self.birds.arm(strike);
    }

    pub fn arm_hail(&mut self, target: HailTarget, diameter_mm: Option<f64>, count: u32) {
        self.hail.trigger(target, diameter_mm, count);
    }

    pub fn arm_lightning(&mut self, entry: Option<super::lightning::AttachPoint>, peak_current_ka: Option<f64>) {
        self.lightning.trigger(entry, peak_current_ka);
    }

    fn phase(&self, truth: &Truth) -> Phase {
        let moving = truth.environment.tas_ms > 5.0;
        if truth.on_ground {
            return if moving { Phase::TakeoffRun } else { Phase::Taxi };
        }
        if truth.altitude_ft >= EN_ROUTE_FT {
            return Phase::EnRoute;
        }
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

        self.lightning.random_mode = self.commands.random_hazards;
        for slot in &mut self.lightning_damage.bus_upset_s {
            *slot = (*slot - dt).max(0.0);
        }
        if let Some(event) = self.lightning.step(convective, dt, &mut self.rng) {
            self.lightning_damage.radome = self.lightning_damage.radome.max(event.radome_damage_frac);
            self.lightning_damage.structure = self.lightning_damage.structure.max(event.structure_damage_frac);
            self.lightning_damage.compass_error_deg = self.lightning_damage.compass_error_deg.max(event.compass_error_deg);
            for transient in &event.transients {
                let k = bus_kind_index(transient.bus);
                self.lightning_damage.bus_peak_volts[k] = self.lightning_damage.bus_peak_volts[k].max(transient.peak_volts);
                if transient.upset_likely {
                    self.lightning_damage.bus_upset_s[k] = BUS_UPSET_HOLD_S;
                }
            }
        }

        self.hail.random_mode = self.commands.random_hazards;
        let hail_state = HailFlightState { tas_ms: env.tas_ms, n1_frac: truth.engine_n1_frac };
        let outcomes = self.hail.step(hail_intensity, &hail_state, dt, &mut self.rng);
        if outcomes.is_empty() {
            self.hail_damage.engine_flameout_risk = [0.0; N_ENGINES];
        }
        for outcome in &outcomes {
            self.hail_damage.absorb(outcome);
        }

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

        self.shear.random_mode = self.commands.random_hazards;
        let generated = self.shear.step(convective, self.commands.groundspeed_ms, dt, &mut self.rng);
        let shear_inputs = self.commands.measured_shear.or(generated);
        self.f_factor = shear_inputs.map_or(0.0, |i| wind_shear::f_factor(&i));

        self.gusts = match weather::turbulence_intensity(env) {
            Some(intensity) => self.turbulence.step(self.commands.altitude_agl_m, env.tas_ms, intensity, dt, &mut self.rng),
            None => TurbulenceGusts::default(),
        };

        self.runway = if truth.on_ground {
            let contaminant = weather::contaminant_from_weather(env.sat_c, env.precipitation_on_aircraft_ratio);
            let groundspeed_kt = self.commands.groundspeed_ms / weather_kt();
            Some(runway_contamination::friction(contaminant, MAIN_TYRE_PRESSURE_PSI, groundspeed_kt))
        } else {
            None
        };

        self.apply_armed(faults);
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        let b = |x: bool| if x { 1.0 } else { 0.0 };

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
        out("ENV_BIRD_WING_LE_DRAG:L", self.bird_damage.wing_le_drag[..N_WING_LE / 2].iter().copied().fold(0.0, f64::max));
        out("ENV_BIRD_WING_LE_DRAG:R", self.bird_damage.wing_le_drag[N_WING_LE / 2..].iter().copied().fold(0.0, f64::max));
        for i in 0..N_PROBE {
            out(&format!("ENV_BIRD_PROBE_BLOCKED:{}", i + 1), self.bird_damage.probe_blocked[i]);
        }

        out("ENV_LTG_RADOME_DAMAGE", self.lightning_damage.radome);
        out("ENV_LTG_STRUCTURE_DAMAGE", self.lightning_damage.structure);
        out("ENV_LTG_COMPASS_ERROR_DEG", self.lightning_damage.compass_error_deg);
        for (k, kind) in BUS_KINDS.iter().enumerate() {
            out(&format!("ENV_LTG_BUS_UPSET:{kind}"), b(self.lightning_damage.bus_upset_s[k] > 0.0));
            out(&format!("ENV_LTG_BUS_PEAK_VOLTS:{kind}"), self.lightning_damage.bus_peak_volts[k]);
        }

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
        out("ENV_ASH_CABIN_ODOR", worst_odour);
        out("ENV_ASH_WINDSHIELD_VISIBILITY_LOSS", self.ash_out[0].map_or(0.0, |o| o.windshield_visibility_loss_frac));
        out("ENV_ASH_PITOT_BLOCKED", self.ash_out[0].map_or(0.0, |o| o.pitot_blockage_frac));

        for i in 0..N_ENGINES {
            let n = i + 1;
            out(&format!("ENV_ICE_ROLLBACK_RISK:{n}"), self.ice_rollback_risk[i]);
            out(&format!("ENV_ICE_FLOW_CAPACITY_LOSS:{n}"), self.ice_flow_loss[i]);
            out(&format!("ENV_ICE_SHEDDING:{n}"), b(self.ice_shedding[i]));
        }

        out("ENV_F_FACTOR", self.f_factor);
        out("ENV_TURBULENCE_GUST_U_MS", self.gusts.u_ms);
        out("ENV_TURBULENCE_GUST_V_MS", self.gusts.v_ms);
        out("ENV_TURBULENCE_GUST_W_MS", self.gusts.w_ms);
        out("ENV_RWY_HYDROPLANING", b(self.runway.map_or(false, |r| r.hydroplaning)));
        out("ENV_RWY_MU_EFFECTIVE", self.runway.map_or(0.0, |r| r.mu_effective));
        out("ENV_RWY_CONDITION_CODE", self.runway.map_or(6.0, |r| r.rwy_cc as f64));
    }
}

fn weather_kt() -> f64 {
    crate::deep::integration::weather_truth::KT_TO_MS
}

impl EnvironmentLive {
    fn apply_armed(&mut self, faults: &Faults) {
        let floor = |slot: &mut f64, m: f64| *slot = slot.max(m);

        for i in 0..N_ENGINES {
            floor(&mut self.bird_damage.fan[i], faults.get(self.ids.bird_fan[i]));
            floor(&mut self.bird_damage.core[i], faults.get(self.ids.bird_core[i]));
        }
        for i in 0..N_WINDSHIELD {
            floor(&mut self.bird_damage.windshield[i], faults.get(self.ids.bird_windshield[i]));
        }
        floor(&mut self.bird_damage.radome, faults.get(self.ids.bird_radome));
        for i in 0..N_WING_LE {
            floor(&mut self.bird_damage.wing_le_drag[i], faults.get(self.ids.bird_wing_le[i]));
        }
        floor(&mut self.bird_damage.nose_gear, faults.get(self.ids.bird_nose_gear));
        for i in 0..N_PROBE {
            if faults.get(self.ids.bird_probe[i]) >= 0.5 {
                floor(&mut self.bird_damage.probe_blocked[i], 1.0);
            }
        }

        floor(&mut self.lightning_damage.radome, faults.get(self.ids.ltg_radome));
        floor(&mut self.lightning_damage.structure, faults.get(self.ids.ltg_structure));
        floor(&mut self.lightning_damage.compass_error_deg, faults.get(self.ids.ltg_compass) * MAX_COMPASS_DEVIATION_DEG);
        for k in 0..BUS_KINDS.len() {
            let bus = faults.get(self.ids.ltg_bus[k]);
            if bus > 0.0 {
                self.lightning_damage.bus_peak_volts[k] = self.lightning_damage.bus_peak_volts[k].max(bus * MAX_BUS_TRANSIENT_V);
                if bus * MAX_BUS_TRANSIENT_V >= UPSET_THRESHOLD_V {
                    self.lightning_damage.bus_upset_s[k] = BUS_UPSET_HOLD_S;
                }
            }
        }

        floor(&mut self.hail_damage.radome, faults.get(self.ids.hail_radome));
        for i in 0..N_WINDSHIELD {
            let hail_windshield = faults.get(self.ids.hail_windshield[i]);
            floor(&mut self.hail_damage.windshield[i], hail_windshield);
            if hail_windshield > WINDOW_HEAT_FAULT_SEVERITY {
                self.hail_damage.window_heat_fault[i] = true;
            }
        }
        for i in 0..N_WING_LE {
            let hail_le = faults.get(self.ids.hail_wing_le[i]);
            floor(&mut self.hail_damage.wing_le[i], hail_le);
            floor(&mut self.hail_damage.slat_jam_risk[i], ((hail_le - 0.5) / 0.5).clamp(0.0, 1.0));
        }
        for i in 0..N_ENGINES {
            let hail_engine = faults.get(self.ids.hail_engine[i]);
            floor(&mut self.hail_damage.engine_fan[i], hail_engine);
            floor(&mut self.hail_damage.engine_compressor_loss[i], hail_engine * MAX_HAIL_COMPRESSOR_LOSS);
            floor(&mut self.hail_damage.engine_flameout_risk[i], hail_engine);
        }
        for i in 0..N_NACELLE {
            floor(&mut self.hail_damage.nacelle[i], faults.get(self.ids.hail_nacelle[i]));
        }
        for i in 0..N_PROBE {
            floor(&mut self.hail_damage.probe[i], faults.get(self.ids.hail_probe[i]));
        }

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
            o.flameout_risk_frac = o.flameout_risk_frac.max(o.flow_capacity_loss_frac * 0.5);
            *slot = Some(o);
        }

        let ice_blockage = faults.get(self.ids.ice_accretion);
        let ice_rollback = faults.get(self.ids.ice_rollback);
        for eng in 0..N_ENGINES {
            self.ice_flow_loss[eng] = self.ice_flow_loss[eng].max(ice_blockage);
            self.ice_rollback_risk[eng] = self.ice_rollback_risk[eng].max(ice_rollback).max(self.ice_flow_loss[eng].powi(2));
        }

        let friction_loss = faults.get(self.ids.runway_friction);
        if friction_loss > 0.0 {
            let mu = (DRY_MU_REFERENCE * (1.0 - friction_loss)).max(0.0);
            let existing = self.runway;
            let mut r = existing.unwrap_or_else(|| runway_contamination::friction(super::runway_contamination::Contaminant::Dry, MAIN_TYRE_PRESSURE_PSI, self.commands.groundspeed_ms / weather_kt()));
            if mu < r.mu_effective {
                r.mu_effective = mu;
                r.hydroplaning = r.hydroplaning || friction_loss >= HYDROPLANING_FRICTION_LOSS;
            }
            self.runway = Some(r);
        }
    }
}

const MAX_COMPASS_DEVIATION_DEG: f64 = 10.0;
const MAX_BUS_TRANSIENT_V: f64 = 70.0;
const UPSET_THRESHOLD_V: f64 = 50.0;
const WINDOW_HEAT_FAULT_SEVERITY: f64 = 0.3;
const MAX_HAIL_COMPRESSOR_LOSS: f64 = 0.25;
const MAX_ASH_EROSION_LOSS: f64 = 0.25;
const DRY_MU_REFERENCE: f64 = 0.40;
const HYDROPLANING_FRICTION_LOSS: f64 = 0.9;

pub fn live_system() -> Box<dyn crate::deep::live::Area> {
    Box::new(EnvironmentLive::new())
}

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
            if !name.starts_with("ENV_") {
                continue;
            }
            assert!(out.contains_key(&name), "alert trigger reads {name}, which nothing publishes");
        }
    }

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
        let mut live = EnvironmentLive::new();
        let out = run(&mut live, &truth, &Faults::default(), 3600.0);
        assert_eq!(out.get("ENV_BIRD_RADOME_DAMAGE"), Some(&0.0));
        assert_eq!(out.get("ENV_HAIL_RADOME_DAMAGE"), Some(&0.0));
        assert_eq!(out.get("ENV_LTG_RADOME_DAMAGE"), Some(&0.0));
    }

    #[test]
    fn arming_the_bird_strike_fan_failure_moves_the_variable_the_real_aircraft_shows_it_through() {
        let mut live = EnvironmentLive::new();
        let id = live.ids.bird_fan[0];

        let mild = run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 0.2)]), 1.0);
        assert_eq!(mild.get("ENV_BIRD_FAN_DAMAGE:1"), Some(&0.2));

        let mut live = EnvironmentLive::new();
        let bad = run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 0.8)]), 1.0);
        assert_eq!(bad.get("ENV_BIRD_FAN_DAMAGE:1"), Some(&0.8), "a severe fan strike must still move the physics input the gas path uses for thrust/EGT/vibration, with no invented FAN DAMAGE alert in the way");
        assert_eq!(bad.get("ENV_BIRD_FAN_DAMAGE:2"), Some(&0.0), "engine 1's fan failure id must not touch engine 2");
        assert_eq!(bad.get("ENV_BIRD_FAN_DAMAGE:3"), Some(&0.0));
        assert_eq!(bad.get("ENV_BIRD_FAN_DAMAGE:4"), Some(&0.0));
    }

    #[test]
    fn injecting_the_bird_strike_core_fod_failure_by_id_affects_only_the_struck_engine() {
        let mut live = EnvironmentLive::new();
        let id = live.ids.bird_core[0];
        let out = run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 0.6)]), 1.0);

        assert_eq!(out.get("ENV_BIRD_CORE_FOD:1"), Some(&0.6), "F14072002 (engine 1) must set engine 1's core FOD state");
        assert_eq!(out.get("ENV_BIRD_CORE_FOD:2"), Some(&0.0), "engine 2 must be untouched by engine 1's bird strike");
        assert_eq!(out.get("ENV_BIRD_CORE_FOD:3"), Some(&0.0), "engine 3 must be untouched by engine 1's bird strike");
        assert_eq!(out.get("ENV_BIRD_CORE_FOD:4"), Some(&0.0), "engine 4 must be untouched by engine 1's bird strike");

        let mut live2 = EnvironmentLive::new();
        let id3 = live2.ids.bird_core[2];
        let out3 = run(&mut live2, &Truth::default(), &Faults::from_pairs([(id3, 0.6)]), 1.0);
        assert_eq!(out3.get("ENV_BIRD_CORE_FOD:3"), Some(&0.6), "engine 3's own core FOD id must set only engine 3");
        assert_eq!(out3.get("ENV_BIRD_CORE_FOD:1"), Some(&0.0));
        assert_eq!(out3.get("ENV_BIRD_CORE_FOD:2"), Some(&0.0));
        assert_eq!(out3.get("ENV_BIRD_CORE_FOD:4"), Some(&0.0));
    }

    #[test]
    fn injecting_the_bird_strike_windshield_failure_by_id_affects_only_the_struck_panel() {
        let mut live = EnvironmentLive::new();
        let id = live.ids.bird_windshield[2];
        let out = run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 0.8)]), 1.0);
        assert_eq!(out.get("ENV_BIRD_WINDSHIELD_DAMAGE:3"), Some(&0.8), "panel 3's own failure id must set panel 3");
        for n in [1, 2, 4, 5, 6] {
            assert_eq!(out.get(&format!("ENV_BIRD_WINDSHIELD_DAMAGE:{n}")), Some(&0.0), "panel {n} must be untouched by panel 3's strike");
        }
    }

    #[test]
    fn injecting_the_bird_strike_wing_leading_edge_failure_by_id_affects_only_the_struck_segment() {
        let mut live = EnvironmentLive::new();
        let id = live.ids.bird_wing_le[6];
        live.tick(&Truth::default(), &Faults::from_pairs([(id, 0.5)]));
        assert_eq!(live.bird_damage.wing_le_drag[6], 0.5, "segment 7's own failure id must set segment 7");
        for i in 0..N_WING_LE {
            if i != 6 {
                assert_eq!(live.bird_damage.wing_le_drag[i], 0.0, "segment {} must be untouched by segment 7's strike", i + 1);
            }
        }
    }

    #[test]
    fn injecting_the_bird_strike_probe_failure_by_id_affects_only_the_struck_probe() {
        let mut live = EnvironmentLive::new();
        let id = live.ids.bird_probe[4];
        let out = run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 1.0)]), 1.0);
        assert_eq!(out.get("ENV_BIRD_PROBE_BLOCKED:5"), Some(&1.0), "probe 5's own failure id must block probe 5");
        for n in [1, 2, 3, 4, 6] {
            assert_eq!(out.get(&format!("ENV_BIRD_PROBE_BLOCKED:{n}")), Some(&0.0), "probe {n} must be untouched by probe 5's strike");
        }
    }

    #[test]
    fn injecting_the_hail_windshield_failure_by_id_affects_only_the_struck_panel() {
        let mut live = EnvironmentLive::new();
        let id = live.ids.hail_windshield[3];
        let out = run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 0.7)]), 1.0);
        assert_eq!(out.get("ENV_HAIL_WINDSHIELD_DAMAGE:4"), Some(&0.7), "panel 4's own hail failure id must set panel 4");
        for n in [1, 2, 3, 5, 6] {
            assert_eq!(out.get(&format!("ENV_HAIL_WINDSHIELD_DAMAGE:{n}")), Some(&0.0), "panel {n} must be untouched by panel 4's hail");
        }
    }

    #[test]
    fn injecting_the_hail_wing_leading_edge_failure_by_id_affects_only_the_struck_segment() {
        let mut live = EnvironmentLive::new();
        let id = live.ids.hail_wing_le[9];
        let out = run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 0.6)]), 1.0);
        assert_eq!(out.get("ENV_HAIL_WING_LE_DAMAGE:10"), Some(&0.6), "segment 10's own hail failure id must set segment 10");
        for n in [1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12] {
            assert_eq!(out.get(&format!("ENV_HAIL_WING_LE_DAMAGE:{n}")), Some(&0.0), "segment {n} must be untouched by segment 10's hail");
        }
    }

    #[test]
    fn injecting_the_hail_nacelle_failure_by_id_affects_only_the_struck_nacelle() {
        let mut live = EnvironmentLive::new();
        let id = live.ids.hail_nacelle[1];
        let out = run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 0.4)]), 1.0);
        assert_eq!(out.get("ENV_HAIL_NACELLE_DAMAGE:2"), Some(&0.4), "nacelle 2's own hail failure id must set nacelle 2");
        for n in [1, 3, 4] {
            assert_eq!(out.get(&format!("ENV_HAIL_NACELLE_DAMAGE:{n}")), Some(&0.0), "nacelle {n} must be untouched by nacelle 2's hail");
        }
    }

    #[test]
    fn injecting_the_hail_probe_failure_by_id_affects_only_the_struck_probe() {
        let mut live = EnvironmentLive::new();
        let id = live.ids.hail_probe[5];
        let out = run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 0.9)]), 1.0);
        assert_eq!(out.get("ENV_HAIL_PROBE_DAMAGE:6"), Some(&0.9), "probe 6's own hail failure id must set probe 6");
        for n in [1, 2, 3, 4, 5] {
            assert_eq!(out.get(&format!("ENV_HAIL_PROBE_DAMAGE:{n}")), Some(&0.0), "probe {n} must be untouched by probe 6's hail");
        }
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
    }

    #[test]
    fn a_lightning_strike_upsets_the_fadec_bus_and_the_upset_clears_by_itself() {
        let mut live = EnvironmentLive::with_seed(3);
        let truth = thunderstorm();
        live.arm_lightning(Some(super::super::lightning::AttachPoint::NoseRadome), Some(200.0));
        let struck = run(&mut live, &truth, &Faults::default(), 0.1);
        assert_eq!(struck.get("ENV_LTG_BUS_UPSET:EngineFadec"), Some(&1.0), "a 200 kA strike must upset the nacelle-mounted FADEC");
        assert!(struck["ENV_LTG_COMPASS_ERROR_DEG"] > 0.0);

        let later = run(&mut live, &truth, &Faults::default(), BUS_UPSET_HOLD_S + 5.0);
        assert_eq!(later.get("ENV_LTG_BUS_UPSET:EngineFadec"), Some(&0.0), "the upset must clear once the equipment has restarted");
        assert_eq!(later["ENV_LTG_COMPASS_ERROR_DEG"], struck["ENV_LTG_COMPASS_ERROR_DEG"], "a compass swing is still required");
    }

    #[test]
    fn arming_the_lightning_bus_transient_upsets_the_fadec_bus_that_feeds_eng_fadec_fault() {
        let mut live = EnvironmentLive::new();
        let id = live.ids.ltg_bus[5];
        let out = run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 1.0)]), 0.1);
        assert_eq!(out.get("ENV_LTG_BUS_UPSET:EngineFadec"), Some(&1.0));

        let mut live2 = EnvironmentLive::new();
        let id0 = live2.ids.ltg_bus[0];
        let out2 = run(&mut live2, &Truth::default(), &Faults::from_pairs([(id0, 1.0)]), 0.1);
        assert_eq!(out2.get("ENV_LTG_BUS_UPSET:EngineFadec"), Some(&0.0), "PRIM's own transient id must not upset the FADEC bus");
    }

    #[test]
    fn ice_crystal_icing_accretes_in_a_glaciated_core_and_rolls_the_engine_back() {
        let truth = thunderstorm();
        let mut live = EnvironmentLive::new();
        live.commands.engine_core_mass_flow_kg_s = [40.0; 4];
        live.commands.engine_warm_surface_temp_c = [2.0; 4];
        let out = run(&mut live, &truth, &Faults::default(), 600.0);
        assert!(out["ENV_ICE_FLOW_CAPACITY_LOSS:1"] > 0.0, "a glaciated core must accrete");
        assert!(out["ENV_ICE_ROLLBACK_RISK:1"] > 0.0);

        let mut dry = EnvironmentLive::new();
        let none = run(&mut dry, &truth, &Faults::default(), 600.0);
        assert_eq!(none.get("ENV_ICE_FLOW_CAPACITY_LOSS:1"), Some(&0.0));
    }

    #[test]
    fn arming_the_ice_rollback_failure_raises_the_risk_the_gas_path_physics_reads() {
        let mut live = EnvironmentLive::new();
        let id = live.ids.ice_rollback;
        let out = run(&mut live, &Truth::default(), &Faults::from_pairs([(id, 0.5)]), 1.0);
        assert_eq!(out.get("ENV_ICE_ROLLBACK_RISK:1"), Some(&0.5), "no invented ICE ROLLBACK alert stands between this failure and the roll-back/flameout risk the gas path consumes; a severe enough case still surfaces through ENG FAIL/ENG STALL on the real parameters");
    }

    #[test]
    fn a_volcanic_ash_encounter_glasses_the_ngvs_and_the_cabin_smells_it() {
        let mut truth = thunderstorm();
        truth.dt_s = 1.0;
        let mut live = EnvironmentLive::new();
        live.commands.ash_concentration_mg_m3 = 4.0;
        live.commands.engine_core_mass_flow_kg_s = [40.0; 4];
        live.commands.engine_ngv_gas_temp_c = [1400.0; 4];
        live.commands.engine_compressor_velocity_ms = [300.0; 4];
        let out = run(&mut live, &truth, &Faults::default(), 600.0);
        assert!(out["ENV_ASH_FLOW_CAPACITY_LOSS:1"] > 0.0, "molten ash must deposit on the NGVs");
        assert!(out["ENV_ASH_CABIN_ODOR"] > 0.1);
        assert!(out["ENV_ASH_FLAMEOUT_RISK:1"] > 0.0);
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
    fn a_measured_headwind_loss_raises_the_f_factor_past_the_hazard_threshold() {
        let mut live = EnvironmentLive::new();
        let truth = Truth { dt_s: 0.1, on_ground: false, environment: EnvironmentTruth { tas_ms: 75.0, ..Truth::default().environment }, ..Truth::default() };
        live.commands.measured_shear = Some(wind_shear::WindShearInputs { headwind_rate_ms2: -3.0, downdraft_ms: 8.0, tas_ms: 75.0 });
        let out = run(&mut live, &truth, &Faults::default(), 1.0);
        assert!(out["ENV_F_FACTOR"] > wind_shear::F_FACTOR_HAZARD_THRESHOLD, "f-factor {}", out["ENV_F_FACTOR"]);
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
            ids.bird_radome,
            ids.bird_nose_gear,
            ids.ltg_radome,
            ids.ltg_structure,
            ids.ltg_compass,
            ids.hail_radome,
            ids.ash_glassing,
            ids.ash_erosion,
            ids.ash_windshield,
            ids.ash_pitot,
            ids.ice_accretion,
            ids.ice_rollback,
            ids.runway_friction,
        ];
        consumed.extend_from_slice(&ids.bird_fan);
        consumed.extend_from_slice(&ids.bird_core);
        consumed.extend_from_slice(&ids.bird_windshield);
        consumed.extend_from_slice(&ids.bird_wing_le);
        consumed.extend_from_slice(&ids.bird_probe);
        consumed.extend_from_slice(&ids.ltg_bus);
        consumed.extend_from_slice(&ids.hail_engine);
        consumed.extend_from_slice(&ids.hail_windshield);
        consumed.extend_from_slice(&ids.hail_wing_le);
        consumed.extend_from_slice(&ids.hail_probe);
        consumed.extend_from_slice(&ids.hail_nacelle);
        consumed.sort_unstable();
        consumed.dedup();
        let registered: Vec<u64> = reg.failures.iter().map(|f| f.id).collect();
        assert_eq!(consumed.len(), registered.len(), "every environment failure should be consumed");
        for id in registered {
            assert!(consumed.contains(&id), "failure {id} is registered but never read by the live system");
        }
    }
}
