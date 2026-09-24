//! The throttle each engine receives, from FlyByWire's own FADEC computer.
//!
//! In MSFS, FlyByWire's fly-by-wire module steps one `A380FadecComputer` per
//! engine (Simulink-generated C++, compiled here unchanged; see
//! `fbw_controllers.rs`) and writes its lever output to the sim's throttle
//! (`FlyByWireInterface.cpp` updateFadec, lines 2907-3046). The same happens
//! here, with X-Plane's throttle override taking the place of MSFS's
//! `GENERAL ENG THROTTLE LEVER POSITION`.
//!
//! Autothrust engagement and its N1 command come from the PRIM computers
//! (`prim.rs`), whose three output buses are handed in each tick as
//! FlyByWireInterface does (cpp:2985-2987). The FADECs' own buses go back to
//! the PRIMs and SECs on the next tick (cpp:3001, 1692-1695).
//!
//! FlyByWire's authority over thrust (XP-002, superseded by the
//! hyperrealism engine workstream): each FADEC's `N1_c_percent` output is
//! the real commanded corrected N1 from the compiled EEC control law, not an
//! estimate. `physics::engine::Engine` (a component-level thermodynamic
//! model of the package's own Trent 972B-84; see `docs/physics/engine.md`)
//! turns that command into a real fuel flow, three real spool speeds, EGT,
//! oil temperature/pressure and thrust — replacing both the
//! pressure-ratio-scaled `n1_and_mach_on_thrust_table` lookup this used
//! before and the reliance on X-Plane's own generic-turbine spool dynamics
//! that `fadec.rs`'s simpler engine model rode on. `fadec.rs`'s own
//! start/shutdown/EGT/fuel-flow/oil polynomials still run first each tick
//! (tick order in `lib.rs`) and still own the discrete engine state machine
//! (`EngineState`, from the master switch and ignition selector) and the
//! N1/N3 thrust-limit schedule (real EASA-cited data); this module's writes
//! to the same `ENGINE_N1:n`/`ENGINE_N3:n`/`ENGINE_EGT:n`/`ENGINE_FF:n`/oil
//! Vars, made afterwards in the same tick, are what is actually physical
//! and take precedence for that tick, one-tick fresher than the polynomial
//! estimate they overwrite.
//!
//! `thrust_trim` below still closes a bounded loop on
//! `sim/flightmodel/engine/POINT_thrust`, now against the physics model's
//! own computed thrust rather than the table. No X-Plane 12 mechanism was
//! found that hands a single engine's thrust to a plugin while leaving
//! X-Plane's own per-engine position/moment geometry alone:
//! `override_engines`/`override_engine_forces` replace the *whole
//! aircraft's* summed propulsive force and moment
//! (`fside/fnrml/faxil_prop`, `L/M/N_prop`), not one engine's, so taking
//! them would mean this plugin computing every engine's force and moment
//! geometry itself from `POINT_XYZ`, with no way to verify the result's
//! sign conventions without X-Plane running (see the workstream report).
//! Trimming the throttle X-Plane's own per-engine model receives keeps
//! X-Plane's already-correct per-engine position and asymmetric yaw, at
//! the cost of X-Plane's own generic turbine curve still mediating the
//! last step from throttle fraction to thrust; the trim's job is to make
//! that step converge on the physics model's number.

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::fadec::{self, ratios};
use crate::afs_events::SimInputThrottles;
use crate::fbw_controllers::{AthrIn, BaseEec, BasePrimOutBus, FadecModel};
use crate::physics::engine::{Engine, EngineInputs, EngineOutputs};
use crate::xp::{DataRef, Xplm};
use crate::Vars;

/// FlyByWireInterface caps the lever just below 100 before sending it.
const LEVER_MAX: f64 = 99.9999999999999;
/// The reverse N1 limit, as a share of TOGA (FlyByWireInterface.cpp:2923).
const REVERSE_SHARE_OF_TOGA: f64 = 0.813;

/// The thrust-trim closed loop's proportional and integral gains, in
/// throttle fraction per fraction of static thrust of error.
const TRIM_KP: f64 = 0.6;
const TRIM_KI: f64 = 0.15;
/// The trim's bound, throttle fraction either way: it refines X-Plane's own
/// engine response, it does not replace it.
pub(crate) const TRIM_LIMIT: f64 = 0.12;

/// One step of the throttle trim that closes the loop on
/// `sim/flightmodel/engine/POINT_thrust`: `error_fraction` is (the physics
/// model's commanded thrust − X-Plane's actual thrust) as a fraction of one
/// engine's static rating. Returns the new integrator state and the trim to
/// add to the lever's own throttle fraction, both clamped to `TRIM_LIMIT` (a
/// clamp-before-integrate anti-windup: the integral never grows past what
/// the output could use).
pub(crate) fn thrust_trim(error_fraction: f64, integral: f64, delta: f64) -> (f64, f64) {
    let integral = (integral + error_fraction * delta).clamp(-TRIM_LIMIT, TRIM_LIMIT);
    let trim = (TRIM_KP * error_fraction + TRIM_KI * integral).clamp(-TRIM_LIMIT, TRIM_LIMIT);
    (integral, trim)
}

struct EngineIds {
    tla: VariableIdentifier,
    tla_n1: VariableIdentifier,
    reverse: VariableIdentifier,
    n1_commanded: VariableIdentifier,
    // The cockpit's engine master switch and start selector, and
    // `fadec.rs`'s discrete state machine and start-delay timer built from
    // them (same Var names, so both modules see the same identifier).
    master: VariableIdentifier,
    igniter: VariableIdentifier,
    state: VariableIdentifier,
    timer: VariableIdentifier,
    // The simulator-facing engine Vars `fadec.rs` also writes (this
    // module's writes happen later in the tick and take precedence, see
    // module docs) plus, for N2 (the IP spool), a real value where
    // `fadec.rs`'s two-spool port had to fake one.
    n1: VariableIdentifier,
    n2: VariableIdentifier,
    n3: VariableIdentifier,
    egt: VariableIdentifier,
    ff: VariableIdentifier,
    oil_temp: VariableIdentifier,
    oil_press: VariableIdentifier,
    // The hyperrealism brief's shared contract (engine loads): bleed and
    // gearbox extraction are read from here (0 until another workstream
    // writes them); fuel demand is written here for the fuel workstream.
    bleed_extraction_kg_s: VariableIdentifier,
    gearbox_elec_load_w: VariableIdentifier,
    gearbox_hyd_load_w: VariableIdentifier,
    fuel_demand_kg_s: VariableIdentifier,
    // `physics/damage.rs`'s continuous oil-pressure hook (module docs
    // there): 1.0 normally, falling with a leak or pump fault.
    oil_pressure_fraction: VariableIdentifier,
    // The customer bleed ports as the engine computes them, for FlyByWire's
    // bleed system (its IP and HP compression chambers), and which one its
    // HP valve is drawing from.
    ip_port_pressure: VariableIdentifier,
    ip_port_temp: VariableIdentifier,
    hp_port_pressure: VariableIdentifier,
    hp_port_temp: VariableIdentifier,
    hp_valve_open: VariableIdentifier,
    bleed_pb_auto: VariableIdentifier,
    tet: VariableIdentifier,
    bleed_limit: VariableIdentifier,
    // 0 while the port in use is outside the temperatures the data sheet
    // lists it for (bleed_limits::customer_bleed_limit_kg_s).
    bleed_port_scheduled: VariableIdentifier,
    // The measured TGT; `egt` above carries the EEC's trimmed, displayed one.
    egt_untrimmed: VariableIdentifier,
    // This engine's feed tank temperature (`fuel.rs`), and what the oil
    // system and hot section publish.
    feed_fuel_temp: VariableIdentifier,
    hot_section_temp: VariableIdentifier,
    oil_supply_temp: VariableIdentifier,
    oil_chamber_temp: [VariableIdentifier; 3],
    fuel_out_temp: VariableIdentifier,
    fcoc_heat: VariableIdentifier,
    oil_filter_bypass: VariableIdentifier,
    oil_relief_open: VariableIdentifier,
    /// `physics::engine::oil`'s own tank level, 1.0 serviced full ..
    /// 0.0 dry: the real quantity, which `deep::live` carries as
    /// `Truth::engine_oil_quantity_fraction` for the tank's quantity
    /// probes to sense.
    oil_quantity_fraction: VariableIdentifier,
    acoc_open: VariableIdentifier,
}

struct Refs {
    override_throttles: Option<DataRef>,
    override_prop_mode: Option<DataRef>,
    override_fuel_flow: Option<DataRef>,
    throttle_use: Option<DataRef>,
    prop_mode: Option<DataRef>,
    running: Option<DataRef>,
    leading_edge_temp: Option<DataRef>,
    inlet_heat: Option<DataRef>,
    /// X-Plane's own engine thrust, the closed loop's feedback
    /// (`sim/flightmodel/engine/POINT_thrust`).
    thrust: Option<DataRef>,
    /// The physics model's outputs, mirrored onto X-Plane's own displayed
    /// engine datarefs so anything reading them natively (replay, other
    /// plugins, X-Plane's own default gauges) sees the same numbers the
    /// converted cockpit does. `n1`/`n3` have no override flag documented
    /// in DataRefs.txt (freely plugin-writable); fuel flow's does
    /// (`override_fuel_flow`).
    n1: Option<DataRef>,
    n2: Option<DataRef>,
    egt: Option<DataRef>,
    itt: Option<DataRef>,
    fuel_flow: Option<DataRef>,
    oil_temp: Option<DataRef>,
    oil_press: Option<DataRef>,
}

/// The four FADEC computers, the four physical engines they command, and
/// what both need.
pub struct EngineCommands {
    models: [FadecModel; 4],
    physics: [Engine; 4],
    /// Last tick's physics output, fed back into this tick's FADEC bus as
    /// the engine's sensed N1/N2 (one tick of lag, the same kind a real
    /// digital EEC's sampled feedback has).
    last_out: [EngineOutputs; 4],
    engines: [EngineIds; 4],
    refs: Refs,
    /// The thrust-trim loop's integrator state, per engine.
    thrust_integral: [f64; 4],
    /// Each engine's damageable components (`components.rs`).
    damage: [DamageHandles; 4],
    /// `FBW_ENG_STATS` only.
    stats_at: Option<std::time::Instant>,
    /// `AIRCRAFT_PRESET_QUICK_MODE`: set while a preset is being applied in
    /// expedited mode, which is the default (`aircraft_presets.rs`).
    preset_quick_mode: VariableIdentifier,

    airspeed: VariableIdentifier,
    true_airspeed: VariableIdentifier,
    mach: VariableIdentifier,
    ground_speed: VariableIdentifier,
    pressure_altitude: VariableIdentifier,
    vertical_speed: VariableIdentifier,
    ambient_temperature: VariableIdentifier,
    total_air_temperature: VariableIdentifier,
    ambient_pressure: VariableIdentifier,
    density: VariableIdentifier,
    lgciu: [(VariableIdentifier, VariableIdentifier); 2],
    limit_idle: VariableIdentifier,
    limit_clb: VariableIdentifier,
    limit_mct: VariableIdentifier,
    limit_flx: VariableIdentifier,
    limit_toga: VariableIdentifier,
    limit_rev: VariableIdentifier,
    limit_type: VariableIdentifier,
    limit: VariableIdentifier,
    athr_disconnect: VariableIdentifier,
    athr_disabled: VariableIdentifier,
    flap_handle: VariableIdentifier,
    pack_1: VariableIdentifier,
}

/// FlyByWire's own idle N3 at this altitude and Mach: the speed its FADEC
/// declares a start complete at, and so the speed a quick-mode engine has
/// to be at for the rest of the aeroplane to agree it is running.
fn fbw_idle_n3(inputs: &EngineInputs) -> f64 {
    let alt_ft = (1.0 - (inputs.ambient_pressure_pa / crate::physics::engine::params::P_REF_PA).powf(0.190_284)) * 145_366.45;
    crate::fadec::table1502::icn3(alt_ft, inputs.mach)
}

/// The three spool speeds a settled ground idle sits at, from FlyByWire's
/// own idle tables rather than from a figure of this model's own, so a
/// quick-started engine lands where a real start would have left it.
fn idle_speeds(inputs: &EngineInputs) -> (f64, f64, f64) {
    let alt_ft = (1.0 - (inputs.ambient_pressure_pa / crate::physics::engine::params::P_REF_PA).powf(0.190_284)) * 145_366.45;
    let n1 = crate::fadec::table1502::icn1(alt_ft, inputs.mach, inputs.ambient_temp_k - 273.15);
    let n3 = crate::fadec::table1502::icn3(alt_ft, inputs.mach);
    // The IP spool has no published idle table; it sits between the other
    // two, and the gas path pulls it to its own equilibrium within a second
    // either way.
    (n1, 0.5 * (n1 + n3), n3)
}

/// Whether `FBW_ENG_STATS` is set to something other than "0"/empty.
fn stats_on() -> bool {
    use std::sync::OnceLock;
    static ON: OnceLock<bool> = OnceLock::new();
    *ON.get_or_init(|| std::env::var("FBW_ENG_STATS").is_ok_and(|v| v.trim() != "0" && !v.trim().is_empty()))
}

impl EngineCommands {
    /// Put every engine's physics at its own settled ground idle
    /// (`physics::engine::idle_engine`): an engines-running spawn.
    pub fn spawn_at_idle(&mut self) {
        for e in self.physics.iter_mut() {
            *e = crate::physics::engine::idle_engine();
        }
    }

    pub fn new(vars: &mut Vars, xplm: &Xplm) -> Self {
        let engine = |vars: &mut Vars, n: usize| EngineIds {
            tla: vars.get(format!("AUTOTHRUST_TLA:{n}")),
            tla_n1: vars.get(format!("AUTOTHRUST_TLA_N1:{n}")),
            reverse: vars.get(format!("AUTOTHRUST_REVERSE:{n}")),
            n1_commanded: vars.get(format!("AUTOTHRUST_N1_COMMANDED:{n}")),
            master: vars.get(format!("GENERAL ENG STARTER:{n}")),
            igniter: vars.get(format!("TURB ENG IGNITION SWITCH EX1:{n}")),
            state: vars.get(format!("ENGINE_STATE:{n}")),
            timer: vars.get(format!("ENGINE_TIMER:{n}")),
            n1: vars.get(format!("ENGINE_N1:{n}")),
            n2: vars.get(format!("ENGINE_N2:{n}")),
            n3: vars.get(format!("ENGINE_N3:{n}")),
            egt: vars.get(format!("ENGINE_EGT:{n}")),
            ff: vars.get(format!("ENGINE_FF:{n}")),
            oil_temp: vars.get(format!("GENERAL ENG OIL TEMPERATURE:{n}")),
            oil_press: vars.get(format!("GENERAL ENG OIL PRESSURE:{n}")),
            bleed_extraction_kg_s: vars.get(format!("ENGINE_BLEED_EXTRACTION_KG_S:{n}")),
            gearbox_elec_load_w: vars.get(format!("ENGINE_GEARBOX_ELEC_LOAD_W:{n}")),
            gearbox_hyd_load_w: vars.get(format!("ENGINE_GEARBOX_HYD_LOAD_W:{n}")),
            fuel_demand_kg_s: vars.get(format!("ENGINE_FUEL_DEMAND_KG_S:{n}")),
            oil_pressure_fraction: vars.get(format!("ENGINE_OIL_PRESSURE_FRACTION:{n}")),
            ip_port_pressure: vars.get(format!("ENGINE_IP_PORT_PRESSURE_PA:{n}")),
            ip_port_temp: vars.get(format!("ENGINE_IP_PORT_TEMP_K:{n}")),
            hp_port_pressure: vars.get(format!("ENGINE_HP_PORT_PRESSURE_PA:{n}")),
            hp_port_temp: vars.get(format!("ENGINE_HP_PORT_TEMP_K:{n}")),
            hp_valve_open: vars.get(format!("PNEU_ENG_{n}_HP_VALVE_OPEN")),
            bleed_pb_auto: vars.get(format!("OVHD_PNEU_ENG_{n}_BLEED_PB_IS_AUTO")),
            tet: vars.get(format!("ENGINE_TET_K:{n}")),
            bleed_limit: vars.get(format!("ENGINE_BLEED_LIMIT_KG_S:{n}")),
            bleed_port_scheduled: vars.get(format!("ENGINE_BLEED_PORT_SCHEDULED:{n}")),
            egt_untrimmed: vars.get(format!("ENGINE_EGT_UNTRIMMED:{n}")),
            // Feed tanks 2, 5, 6, 9 feed engines 1-4 (`fuel.rs`'s ENGINE_FEED_TANKS).
            feed_fuel_temp: vars.get(format!("FUEL_TEMP_{}", [2, 5, 6, 9][n - 1])),
            hot_section_temp: vars.get(format!("ENGINE_HOT_SECTION_TEMP_C:{n}")),
            oil_supply_temp: vars.get(format!("ENGINE_OIL_SUPPLY_TEMP_C:{n}")),
            oil_chamber_temp: ["FRONT", "HPIP", "TAIL"].map(|c| vars.get(format!("ENGINE_OIL_CHAMBER_{c}_TEMP_C:{n}"))),
            fuel_out_temp: vars.get(format!("ENGINE_FUEL_OUTLET_TEMP_C:{n}")),
            fcoc_heat: vars.get(format!("ENGINE_FCOC_HEAT_W:{n}")),
            oil_filter_bypass: vars.get(format!("ENGINE_OIL_FILTER_BYPASS:{n}")),
            oil_relief_open: vars.get(format!("ENGINE_OIL_RELIEF_OPEN:{n}")),
            oil_quantity_fraction: vars.get(format!("ENGINE_OIL_QUANTITY_FRACTION:{n}")),
            acoc_open: vars.get(format!("ENGINE_ACOC_OPEN:{n}")),
        };
        let engines = [engine(vars, 1), engine(vars, 2), engine(vars, 3), engine(vars, 4)];
        Self {
            models: [FadecModel::new(), FadecModel::new(), FadecModel::new(), FadecModel::new()],
            physics: [Engine::new(), Engine::new(), Engine::new(), Engine::new()],
            last_out: [EngineOutputs::default(); 4],
            engines,
            refs: Refs {
                override_throttles: xplm.find("sim/operation/override/override_throttles"),
                override_prop_mode: xplm.find("sim/operation/override/override_prop_mode"),
                override_fuel_flow: xplm.find("sim/operation/override/override_fuel_flow"),
                throttle_use: xplm.find("sim/flightmodel/engine/ENGN_thro_use"),
                prop_mode: xplm.find("sim/flightmodel/engine/ENGN_propmode"),
                running: xplm.find("sim/flightmodel/engine/ENGN_running"),
                leading_edge_temp: xplm.find("sim/weather/aircraft/temperature_leadingedge_deg_c"),
                inlet_heat: xplm.find("sim/cockpit2/ice/ice_inlet_heat_on_per_engine"),
                thrust: xplm.find("sim/flightmodel/engine/POINT_thrust"),
                n1: xplm.find("sim/flightmodel/engine/ENGN_N1_"),
                n2: xplm.find("sim/flightmodel/engine/ENGN_N2_"),
                egt: xplm.find("sim/flightmodel2/engines/EGT_deg_cel"),
                itt: xplm.find("sim/flightmodel2/engines/ITT_deg_cel"),
                fuel_flow: xplm.find("sim/flightmodel/engine/ENGN_FF_"),
                oil_temp: xplm.find("sim/flightmodel/engine/ENGN_oil_temp_c"),
                oil_press: xplm.find("sim/flightmodel/engine/ENGN_oil_press_psi"),
            },
            thrust_integral: [0.; 4],
            damage: std::array::from_fn(|i| DamageHandles::register(i + 1)),
            stats_at: None,
            preset_quick_mode: vars.get("AIRCRAFT_PRESET_QUICK_MODE".to_owned()),
            airspeed: vars.get("AIRSPEED INDICATED".into()),
            true_airspeed: vars.get("AIRSPEED TRUE".into()),
            mach: vars.get("AIRSPEED MACH".into()),
            ground_speed: vars.get("GPS GROUND SPEED".into()),
            pressure_altitude: vars.get("PRESSURE ALTITUDE".into()),
            vertical_speed: vars.get("VELOCITY WORLD Y".into()),
            ambient_temperature: vars.get("AMBIENT TEMPERATURE".into()),
            total_air_temperature: vars.get("TOTAL AIR TEMPERATURE".into()),
            ambient_pressure: vars.get("AMBIENT PRESSURE".into()),
            density: vars.get("AMBIENT DENSITY".into()),
            lgciu: [
                (vars.get("LGCIU_1_LEFT_GEAR_COMPRESSED".into()), vars.get("LGCIU_1_RIGHT_GEAR_COMPRESSED".into())),
                (vars.get("LGCIU_2_LEFT_GEAR_COMPRESSED".into()), vars.get("LGCIU_2_RIGHT_GEAR_COMPRESSED".into())),
            ],
            limit_idle: vars.get("AUTOTHRUST_THRUST_LIMIT_IDLE".into()),
            limit_clb: vars.get("AUTOTHRUST_THRUST_LIMIT_CLB".into()),
            limit_mct: vars.get("AUTOTHRUST_THRUST_LIMIT_MCT".into()),
            limit_flx: vars.get("AUTOTHRUST_THRUST_LIMIT_FLX".into()),
            limit_toga: vars.get("AUTOTHRUST_THRUST_LIMIT_TOGA".into()),
            limit_rev: vars.get("AUTOTHRUST_THRUST_LIMIT_REV".into()),
            limit_type: vars.get("AUTOTHRUST_THRUST_LIMIT_TYPE".into()),
            limit: vars.get("AUTOTHRUST_THRUST_LIMIT".into()),
            athr_disconnect: vars.get("AUTOTHRUST_DISCONNECT".into()),
            athr_disabled: vars.get("AUTOTHRUST_DISABLED".into()),
            flap_handle: vars.get("FLAPS_HANDLE_INDEX".into()),
            pack_1: vars.get("OVHD_COND_PACK_1_PB_IS_ON".into()),
        }
    }

    /// Steps the four FADECs with this tick's PRIM buses and one-frame
    /// autothrust inputs, then the four physical engines with the FADECs'
    /// commanded N1, and returns the FADECs' EEC buses.
    pub fn update(
        &mut self,
        vars: &mut Vars,
        xplm: &Xplm,
        delta: f64,
        simulation_time: f64,
        prims: &[BasePrimOutBus; 3],
        throttles: &SimInputThrottles,
    ) -> [BaseEec; 4] {
        let mut eec = [BaseEec::default(); 4];
        let mut running = [0i32; 4];
        let mut thrust = [0f32; 4];
        if let Some(d) = self.refs.running {
            xplm.get_vi(d, &mut running);
        }
        if let Some(d) = self.refs.thrust {
            xplm.get_vf(d, &mut thrust);
        }
        let ambient_pressure_hpa = vars.read(&self.ambient_pressure) * fadec::INHG_TO_HPA;

        // Total air temperature: X-Plane's temperature at the leading edge.
        let tat = self.refs.leading_edge_temp.map(|d| xplm.get_f(d) as f64);
        if let Some(tat) = tat {
            let id = self.total_air_temperature;
            vars.write_from_xplane(&id, tat);
        }

        let mach = vars.read(&self.mach);
        let oat = vars.read(&self.ambient_temperature);
        let correction = ratios::theta2(mach, oat).sqrt();
        let toga = vars.read(&self.limit_toga);
        let rev = toga * REVERSE_SHARE_OF_TOGA;
        vars.write(&self.limit_rev, rev);

        let mut common = AthrIn::default();
        common.time.dt = delta.max(0.002);
        common.time.simulation_time = simulation_time;
        common.data.V_ias_kn = vars.read(&self.airspeed);
        common.data.V_tas_kn = vars.read(&self.true_airspeed);
        common.data.V_mach = mach;
        common.data.V_gnd_kn = vars.read(&self.ground_speed);
        common.data.H_ft = vars.read(&self.pressure_altitude);
        common.data.H_ind_ft = common.data.H_ft;
        common.data.H_dot_fpm = vars.read(&self.vertical_speed);
        common.data.flap_handle_index = vars.read(&self.flap_handle);
        common.data.TAT_degC = vars.read(&self.total_air_temperature);
        common.data.OAT_degC = oat;
        common.data.ambient_density_kg_per_m3 = vars.read(&self.density) / 0.001_940_32;
        // cpp:2972-2973
        common.input.ATHR_disconnect = (throttles.athr_disconnect || vars.read(&self.athr_disconnect) == 1.) as u8;
        common.input.thrust_limit_REV_percent = rev;
        common.input.thrust_limit_IDLE_percent = vars.read(&self.limit_idle);
        common.input.thrust_limit_CLB_percent = vars.read(&self.limit_clb);
        common.input.thrust_limit_MCT_percent = vars.read(&self.limit_mct);
        common.input.thrust_limit_FLEX_percent = vars.read(&self.limit_flx);
        common.input.thrust_limit_TOGA_percent = toga;
        // cpp:2981-2983: ENG ANTI ICE:1 for every engine, pack 1's pushbutton.
        let mut inlet_heat = [0i32; 1];
        if let Some(d) = self.refs.inlet_heat {
            xplm.get_vi(d, &mut inlet_heat);
        }
        common.input.is_anti_ice_active = (inlet_heat[0] == 1) as u8;
        common.input.is_air_conditioning_active = (vars.read(&self.pack_1) != 0.) as u8;
        common.input.ATHR_reset_disable = throttles.athr_reset_disable as u8;
        // cpp:2985-2987
        common.prim_1 = prims[0];
        common.prim_2 = prims[1];
        common.prim_3 = prims[2];

        let ambient_pressure_pa = ambient_pressure_hpa * 100.;
        let ambient_temp_k = oat + 273.15;
        let true_airspeed_m_s = vars.read(&self.true_airspeed) * 0.514_444;

        // A preset asking for an aeroplane that is ready to fly gets one
        // now rather than after four real engine starts. See
        // `Engine::snap_to_idle`: the APU and the ADIRS already treat quick
        // mode this way, and the engines were the only thing left that a
        // "ready for takeoff" preset still had to sit and wait for.
        let quick = vars.read(&self.preset_quick_mode) != 0.;
        let mut disabled = false;
        // `FBW_ENG_STATS=1`: the gas path's own station values, gathered
        // inside the per-engine loop and logged once below.
        //
        // The damage model reports take-off TGT around 1127 C against an
        // untrimmed over-temperature limit of 957 (EASA.E.012 Note 14), so
        // either the limit is being compared against the wrong quantity or
        // the engine is genuinely running 170 C hot -- and nothing short of
        // the stations between fuel flow and the turbine tells those apart.
        // Fuel flow says whether the FADEC is over-fuelling; TET against
        // TGT says whether the turbines are extracting the work they
        // should; core mass flow and the bleed extraction beside its own
        // scheduled limit say whether the core is being starved of the air
        // that would otherwise carry that heat away.
        let mut stats_line = String::new();
        for i in 0..4 {
            // LGCIU 1 for engines 1 and 2, LGCIU 2 for 3 and 4. FlyByWire
            // indexes a two-element array by engine, reading past it for
            // engines 3 and 4; this is what that code means.
            let (left, right) = self.lgciu[i / 2];
            let on_ground = vars.read(&left) != 0. && vars.read(&right) != 0.;

            // The FADEC's sensed feedback is last tick's physics output
            // (see module docs): a real corrected N1, not an estimate.
            let last = self.last_out[i];
            let actual = last.n1_pct;
            let commanded = last.n1_pct / correction;

            let mut inputs = common;
            inputs.data.on_ground = on_ground as u8;
            inputs.data.is_engine_operative = (running[i] != 0) as u8;
            inputs.data.engine_N1_percent = actual;
            inputs.data.engine_N2_percent = last.n3_pct; // A380's N3, see fadec.rs's own convention
            inputs.data.commanded_engine_N1_percent = commanded;
            inputs.input.TLA_deg = vars.read(&self.engines[i].tla);

            let out = self.models[i].step(&inputs);
            eec[i] = out.fadec_bus_output; // cpp:3001
            let o = out.output;
            let e = &self.engines[i];
            let (tla_n1, reverse, n1_commanded) = (e.tla_n1, e.reverse, e.n1_commanded);
            vars.write(&tla_n1, o.N1_TLA_percent);
            vars.write(&reverse, o.is_in_reverse as f64);
            vars.write(&n1_commanded, o.N1_c_percent);
            if i == 0 {
                vars.write(&self.limit_type, o.thrust_limit_type as f64);
                vars.write(&self.limit, o.thrust_limit_percent);
            }
            if i < 2 {
                disabled |= out.data_computed.ATHR_disabled != 0;
            }

            // ---- The physical engine: FlyByWire's FADEC decided the
            // target above; this is the engine responding to it (bleed and
            // gearbox extraction per the hyperrealism brief's shared
            // contract, read as 0 until another workstream writes them).
            let master = vars.read(&e.master) != 0.;
            let igniter = vars.read(&e.igniter).round() as i32;
            let state = fadec::EngineState::from(vars.read(&e.state));
            let timer = vars.read(&e.timer);
            let starter_engaged = master
                && igniter == 2
                && matches!(state, fadec::EngineState::Starting | fadec::EngineState::Restarting)
                && timer >= 1.7;

            let phys_inputs = EngineInputs {
                ambient_pressure_pa,
                ambient_temp_k,
                mach,
                true_airspeed_m_s,
                target_n1_corrected_pct: ground_n1_protection(o.N1_c_percent, on_ground, common.data.V_gnd_kn),
                fuel_valve_open: master,
                starter_engaged,
                // No shared variable yet carries cross-bleed/APU/ground-cart
                // starter supply pressure (see the workstream report); full
                // nameplate starter performance is assumed until one does.
                starter_supply_fraction: 1.0,
                bleed_extraction_kg_s: vars.read(&e.bleed_extraction_kg_s),
                // FlyByWire's HP valve open means its HP6 port is feeding
                // the bleed (the IP check valve shuts against the higher
                // pressure); otherwise it comes off IP8.
                bleed_from_ip_port: vars.read(&e.hp_valve_open) == 0.,
                gearbox_elec_load_w: vars.read(&e.gearbox_elec_load_w),
                gearbox_hyd_load_w: vars.read(&e.gearbox_hyd_load_w),
                // Continuous gas-path degradation: `failures::magnitude`
                // for this engine's own 72_004 "compressor stall", 72_008
                // "turbine blade damage" and 72_000 "bearing wear" ids
                // (`i` is 0-indexed here, matching `failures.rs`'s
                // `item.base + (n - 1)` per-engine id scheme). 0.0 (no
                // active failure) is the same healthy default the unit
                // tests use.
                compressor_efficiency_loss_fraction: crate::components::value(self.damage[i].hpc_efficiency_loss),
                compressor_flow_capacity_loss_fraction: crate::components::value(self.damage[i].hpc_flow_capacity_loss),
                turbine_efficiency_loss_fraction: crate::components::value(self.damage[i].hpt_efficiency_loss),
                bearing_friction_extra_fraction: crate::components::value(self.damage[i].bearing_friction),
                // `physics/damage.rs` always writes this hook every tick
                // (module docs there), so this already reflects a healthy
                // 1.0 unless a leak/pump fault is active.
                oil_pressure_fraction: vars.read(&e.oil_pressure_fraction),
                fuel_temp_k: vars.read(&e.feed_fuel_temp) + 273.15,
                // failures::extra 79_004+n ("engine oil leak"): a hole in
                // the pressurised feed gallery that drains the tank, which is
                // what that failure's name has always described.
                // `physics/damage.rs` reads the same id for its coarser
                // `oil_pressure_fraction` hook above; they are one leak, and
                // `oil.rs`'s own pump-inlet threshold is set well below that
                // hook's quantity band so the two do not stack into one cliff.
                oil_faults: crate::physics::engine::oil::OilFaults {
                    leak: crate::failures::magnitude(79_004 + i as u64),
                    ..Default::default()
                },
                dt_s: delta,
            };
            if quick && phys_inputs.fuel_valve_open && self.last_out[i].n3_pct < fbw_idle_n3(&phys_inputs) {
                let (n1, n2, n3) = idle_speeds(&phys_inputs);
                self.physics[i].snap_to_idle(n1, n2, n3);
            }
            let phys = self.physics[i].step(&phys_inputs);
            self.last_out[i] = phys;

            // Real spool speeds, EGT, fuel flow and oil state, overwriting
            // `fadec.rs`'s polynomial estimate of the same Vars from
            // earlier this tick (module docs).
            vars.write(&e.n1, phys.n1_pct);
            vars.write(&e.n2, phys.n2_pct);
            vars.write(&e.n3, phys.n3_pct);
            // The EEC's TGT trim (EASA.E.012 Note 16): the cockpit, and
            // everything reading `ENGINE_EGT:n`, gets the trimmed value; the
            // engine's own measured TGT goes to `ENGINE_EGT_UNTRIMMED:n`.
            // The trim never takes the display below ambient: a stopped
            // engine's TGT is the air's, trimmed or not.
            let takeoff_rating = matches!(o.thrust_limit_type, 3 | 4); // FLEX, TOGA
            let trim = crate::physics::damage::tgt_trim_c(takeoff_rating);
            let ambient_c = ambient_temp_k - 273.15;
            let egt_displayed = (phys.egt_c - trim).max(phys.egt_c.min(ambient_c));
            vars.write(&e.egt_untrimmed, phys.egt_c);
            vars.write(&e.egt, egt_displayed);
            vars.write(&e.ip_port_pressure, phys.ip_port_pressure_pa);
            vars.write(&e.ip_port_temp, phys.ip_port_temp_k);
            vars.write(&e.hp_port_pressure, phys.hp_port_pressure_pa);
            vars.write(&e.hp_port_temp, phys.hp_port_temp_k);
            vars.write(&e.tet, phys.tet_k);
            vars.write(&e.hot_section_temp, phys.hot_section_c);
            vars.write(&e.oil_supply_temp, phys.oil_supply_c);
            for (id, c) in e.oil_chamber_temp.iter().zip(phys.oil_chamber_c) {
                vars.write(id, c);
            }
            vars.write(&e.fuel_out_temp, phys.fuel_out_c);
            vars.write(&e.fcoc_heat, phys.fuel_heat_w);
            vars.write(&e.oil_filter_bypass, phys.oil_filter_bypassed as i32 as f64);
            vars.write(&e.oil_relief_open, phys.oil_relief_open as i32 as f64);
            vars.write(&e.oil_quantity_fraction, phys.oil_quantity_fraction);
            vars.write(&e.acoc_open, phys.acoc_open);
            {
                use crate::physics::engine::bleed_limits::{customer_bleed_limit_kg_s, Configuration, Port};
                // Two bleeds or fewer on is the data sheet's abnormal case.
                let bleeds_on = self.engines.iter().filter(|x| vars.read(&x.bleed_pb_auto) != 0.).count();
                let configuration = if bleeds_on <= 2 { Configuration::Abnormal } else { Configuration::Normal };
                let (port, flow) = if phys_inputs.bleed_from_ip_port { (Port::Ip8, phys.w24_kg_s) } else { (Port::Hp6, phys.w26_kg_s) };
                let (limit, scheduled) = customer_bleed_limit_kg_s(port, configuration, phys.tet_k, flow);
                vars.write(&e.bleed_limit, limit);
                vars.write(&e.bleed_port_scheduled, scheduled as i32 as f64);
                if stats_on() {
                    stats_line.push_str(&format!(
                        " eng{}: N1 {:.0} N2 {:.0} N3 {:.0}%, wf {:.3} kg/s, TET {:.0} K, TGT {:.0} C, core {:.1} kg/s, thrust {:.0} kN, bleed {:.3} of {:.3} kg/s ({:?});",
                        i + 1,
                        phys.n1_pct,
                        phys.n2_pct,
                        phys.n3_pct,
                        phys.fuel_flow_kg_s,
                        phys.tet_k,
                        phys.egt_c,
                        phys.core_mdot_kg_s,
                        phys.net_thrust_n / 1000.,
                        vars.read(&e.bleed_extraction_kg_s),
                        limit,
                        port,
                    ));
                }
            }
            vars.write(&e.ff, phys.fuel_flow_kg_s * 3600.); // ENGINE_FF:n is kg/h
            vars.write(&e.oil_temp, phys.oil_temp_c);
            vars.write(&e.oil_press, phys.oil_press_psi);
            vars.write(&e.fuel_demand_kg_s, phys.fuel_flow_kg_s);

            if let Some(d) = self.refs.n1 {
                xplm.set_vf_at(d, i, phys.n1_pct as f32);
            }
            if let Some(d) = self.refs.n2 {
                xplm.set_vf_at(d, i, phys.n3_pct as f32);
            }
            for d in [self.refs.egt, self.refs.itt] {
                if let Some(d) = d {
                    xplm.set_vf_at(d, i, phys.egt_c as f32);
                }
            }
            if let Some(d) = self.refs.fuel_flow {
                xplm.set_vf_at(d, i, phys.fuel_flow_kg_s as f32);
            }
            if let Some(d) = self.refs.oil_temp {
                xplm.set_vf_at(d, i, phys.oil_temp_c as f32);
            }
            if let Some(d) = self.refs.oil_press {
                xplm.set_vf_at(d, i, phys.oil_press_psi as f32);
            }

            // The lever to X-Plane's engine. Reverse (lever < 0) keeps
            // X-Plane's own propmode (for whatever reads it) but holds its
            // throttle at zero: FlyByWire's own reverser force (reversers.rs;
            // ReverserForce, already running inside `A380`) is the sole
            // source of the deceleration, applied to X-Plane's velocity in
            // extra_backend_fbw.rs. Leaving X-Plane's throttle nonzero here
            // too would double-count the reverse thrust.
            let lever = o.sim_throttle_lever_pos.min(LEVER_MAX);
            let in_reverse = lever < 0.;
            if let Some(d) = self.refs.prop_mode {
                xplm.set_vi_at(d, i, if in_reverse { 3 } else { 1 });
            }
            if let Some(d) = self.refs.throttle_use {
                let base = (lever.abs() / 100.) as f64;
                let ratio = if in_reverse || running[i] == 0 {
                    // No thrust target while stopped, starting or in
                    // reverse (reverse's deceleration is reversers.rs's, not
                    // X-Plane's throttle); hold the integrator at zero so it
                    // does not wind up before there is a thrust to trim.
                    self.thrust_integral[i] = 0.;
                    if in_reverse {
                        0.
                    } else {
                        base
                    }
                } else {
                    // The physics model's own commanded thrust (module
                    // docs), not a table lookup.
                    let target = phys.net_thrust_n * fadec::N_TO_LBF;
                    let actual = thrust[i] as f64 * fadec::N_TO_LBF;
                    let static_thrust_lbf = crate::physics::engine::params::STATIC_THRUST_N * fadec::N_TO_LBF;
                    let error = (target - actual) / static_thrust_lbf.max(1.);
                    let (integral, trim) = thrust_trim(error, self.thrust_integral[i], delta);
                    self.thrust_integral[i] = integral;
                    (base + trim).clamp(0., 1.)
                };
                xplm.set_vf_at(d, i, ratio as f32);
            }
        }
        if stats_on() && !stats_line.is_empty() {
            let now = std::time::Instant::now();
            if !self.stats_at.is_some_and(|t| now - t < std::time::Duration::from_secs(2)) {
                self.stats_at = Some(now);
                crate::log(&format!("eng:{stats_line}"));
            }
        }
        vars.write(&self.athr_disabled, disabled as i32 as f64);

        for d in [self.refs.override_throttles, self.refs.override_prop_mode, self.refs.override_fuel_flow].into_iter().flatten() {
            xplm.set_i(d, 1);
        }

        eec
    }

    /// Hand the throttles back to X-Plane.
    pub fn release(&mut self, xplm: &Xplm) {
        for d in [self.refs.override_throttles, self.refs.override_prop_mode, self.refs.override_fuel_flow].into_iter().flatten() {
            xplm.set_i(d, 0);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The throttle trim loop that closes on X-Plane's own per-engine
    /// thrust (module docs): a positive error (the physics model wants more
    /// thrust than X-Plane's generic turbine gave) must trim the lever up,
    /// not down, and a run of the same-signed error must not windup the
    /// integrator, or the trim, past `TRIM_LIMIT`.
    #[test]
    fn positive_error_trims_the_lever_up_and_never_past_the_limit() {
        let (mut integral, mut trim) = (0., 0.);
        for _ in 0..1_000 {
            (integral, trim) = thrust_trim(0.5, integral, 0.05);
            assert!(integral.abs() <= TRIM_LIMIT + 1e-12, "integral wound up past its clamp: {integral}");
            assert!(trim.abs() <= TRIM_LIMIT + 1e-12, "trim exceeded its clamp: {trim}");
        }
        assert!(trim > 0., "a positive thrust error trimmed the lever down: {trim}");
        assert!((trim - TRIM_LIMIT).abs() < 1e-9, "a sustained error should settle at the trim limit, got {trim}");
    }

    /// The mirror image: a negative error (X-Plane already giving more
    /// thrust than commanded) must trim down, never up.
    #[test]
    fn negative_error_trims_the_lever_down() {
        let (mut integral, mut trim) = (0., 0.);
        for _ in 0..1_000 {
            (integral, trim) = thrust_trim(-0.5, integral, 0.05);
        }
        assert!(trim < 0., "a negative thrust error trimmed the lever up: {trim}");
        assert!((trim + TRIM_LIMIT).abs() < 1e-9);
    }

    /// Zero error must neither wind the integrator up nor apply any trim,
    /// regardless of `delta` (a per-frame-delta bug would drift this even
    /// with no error).
    #[test]
    fn zero_error_leaves_the_trim_at_zero_no_matter_the_frame_time() {
        for delta in [0.002, 0.016_7, 0.05, 0.25] {
            let (integral, trim) = thrust_trim(0., 0.05, delta);
            // The integrator itself is untouched by a zero error (no decay
            // term), but the output trim commanded this frame is exactly
            // the proportional+integral combination, not re-scaled by delta
            // a second time.
            assert_eq!(integral, 0.05);
            assert!((trim - (TRIM_KI * 0.05)).abs() < 1e-12, "delta {delta}: trim {trim}");
        }
    }

    /// A step from a large positive to a large negative error must cross
    /// zero and settle on the new sign within a handful of frames, not stay
    /// pinned at the old limit (a clamp bug on the wrong side would do
    /// that).
    #[test]
    fn the_trim_follows_a_reversed_error_within_a_few_frames() {
        let (mut integral, mut trim) = (0., 0.);
        for _ in 0..200 {
            (integral, trim) = thrust_trim(0.5, integral, 0.05);
        }
        assert!(trim > 0.);
        let mut frames_to_cross = None;
        for i in 0..200 {
            (integral, trim) = thrust_trim(-0.5, integral, 0.05);
            if trim < 0. && frames_to_cross.is_none() {
                frames_to_cross = Some(i);
            }
        }
        assert!(frames_to_cross.is_some(), "trim never crossed zero after the error reversed");
        assert!(frames_to_cross.unwrap() < 100, "trim took {:?} frames to follow a reversed error", frames_to_cross);
        assert!((trim + TRIM_LIMIT).abs() < 1e-9);
    }
}

use crate::components::{self, Combine, FailureDef, Handle, ParamSpec, Perturbation};

const LOSS: fn(&'static str, &'static str) -> ParamSpec = |name, description| ParamSpec {
    name,
    unit: "fraction",
    healthy: 0.0,
    min: 0.0,
    max: 1.0,
    combine: Combine::CompoundLoss,
    description,
};

/// Every engine's damageable components and the failures acting on them,
/// registered without running the engines: the catalogue the XPHFBW app
/// shows when X-Plane is not running. Registering is idempotent, so the
/// running engines later find the same parameters.
pub(crate) fn register_component_catalogue() {
    for n in 1..=4 {
        DamageHandles::register(n);
    }
}

/// Whether failure `id` acts on the engine's own components
/// (`EXOTIC`, one id per engine) rather than being a component of its own.
pub(crate) fn is_engine_component_failure(id: u64) -> bool {
    EXOTIC.iter().any(|(base, _)| (*base..*base + 4).contains(&id))
}

/// One engine's damageable components, as the engine model reads them.
struct DamageHandles {
    hpc_efficiency_loss: Handle,
    hpc_flow_capacity_loss: Handle,
    hpt_efficiency_loss: Handle,
    bearing_friction: Handle,
}

impl DamageHandles {
    /// Register engine `n`'s components and the catalogued failures that
    /// act on them.
    fn register(n: usize) -> Self {
        let hpc = format!("engine.{n}.hp_compressor");
        let hpt = format!("engine.{n}.hp_turbine");
        let bearings = format!("engine.{n}.bearings");
        let c = components::register(
            &hpc,
            &[
                LOSS("efficiency_loss", "isentropic efficiency lost: eroded, bent or missing blades"),
                LOSS("flow_capacity_loss", "corrected flow capacity lost: blockage, missing blades"),
            ],
        );
        let t = components::register(&hpt, &[LOSS("efficiency_loss", "turbine efficiency lost: burnt, cracked or released blades")]);
        let b = components::register(
            &bearings,
            &[ParamSpec {
                name: "extra_friction",
                unit: "x HP turbine design torque",
                healthy: 0.0,
                min: 0.0,
                max: 5.0,
                combine: Combine::Sum,
                description: "extra shaft friction: wear, imbalance load, seizure",
            }],
        );
        for (base, d) in EXOTIC {
            let id = base + (n as u64 - 1);
            let mut perturbations = Vec::new();
            let mut push = |component: &str, param: &str, value: f64| {
                if value != 0.0 {
                    perturbations.push(Perturbation { component: component.to_owned(), param: param.to_owned(), value });
                }
            };
            push(&hpc, "efficiency_loss", d.compressor_efficiency_loss);
            push(&hpc, "flow_capacity_loss", d.compressor_flow_capacity_loss);
            push(&hpt, "efficiency_loss", d.turbine_efficiency_loss);
            push(&bearings, "extra_friction", d.bearing_friction);
            components::define_failure(FailureDef { id, perturbations });
        }
        Self { hpc_efficiency_loss: c[0], hpc_flow_capacity_loss: c[1], hpt_efficiency_loss: t[0], bearing_friction: b[0] }
    }
}

/// What one catalogued failure moves on an engine's components at full
/// magnitude (turned into `components` perturbations by
/// `DamageHandles::register`, which combine with any others by each
/// parameter's own rule). Nothing more: every consequence comes out of the
/// gas path and spool physics.
#[derive(Clone, Copy, Default)]
struct EngineDamage {
    compressor_efficiency_loss: f64,
    compressor_flow_capacity_loss: f64,
    turbine_efficiency_loss: f64,
    /// Extra shaft friction, in the engine model's own unit (a fraction of
    /// the HP turbine's design torque): additive, and may exceed 1.
    bearing_friction: f64,
}

/// Each engine failure (by its base id; engine n is base + n - 1) and the
/// physical quantities it moves at full magnitude.
const EXOTIC: [(u64, EngineDamage); 6] = [
    // Bearing *wear* (`failures.rs`: "a main shaft bearing wears, raising
    // vibration and running clearances"), not a seizure. This was 1.0 --
    // a parasitic drag equal to the HP turbine's entire design torque,
    // which is more than the whole turbine makes anywhere below its design
    // speed. Armed in flight it did not degrade the engine, it stopped it:
    // N3 fell from 68 % to 9 % in two seconds, the flame went out, and the
    // EEC's start-abort latched the fuel off for good. Four engines, every
    // take-off, from a failure whose own description is a wear item.
    //
    // The table contradicted itself, which is the cheapest way to see it:
    // 72_012 below is HP compressor *destruction* -- blades gone, the rotor
    // out of balance -- and it asks for 0.3. Wear cannot drag harder than
    // that. Main-shaft bearings absorb well under a percent of shaft power
    // when healthy, so a worn one is a few percent: enough to cost fuel,
    // spool time and oil temperature, which is what a wear failure is for.
    (72_000, EngineDamage { bearing_friction: 0.03, compressor_efficiency_loss: 0.0, compressor_flow_capacity_loss: 0.0, turbine_efficiency_loss: 0.0 }),
    (72_004, EngineDamage { compressor_efficiency_loss: 1.0, compressor_flow_capacity_loss: 1.0, turbine_efficiency_loss: 0.0, bearing_friction: 0.0 }),
    (72_008, EngineDamage { turbine_efficiency_loss: 1.0, compressor_efficiency_loss: 0.0, compressor_flow_capacity_loss: 0.0, bearing_friction: 0.0 }),
    // HP compressor destruction: compression and flow gone, imbalance on
    // the bearings.
    (72_012, EngineDamage { compressor_efficiency_loss: 1.0, compressor_flow_capacity_loss: 0.9, turbine_efficiency_loss: 0.0, bearing_friction: 0.3 }),
    // HP turbine blade release: most turbine work gone, imbalance on the
    // bearings.
    (72_016, EngineDamage { turbine_efficiency_loss: 0.8, bearing_friction: 0.5, compressor_efficiency_loss: 0.0, compressor_flow_capacity_loss: 0.0 }),
    // Main bearing seizure: more friction than the HP turbine's design torque.
    (72_020, EngineDamage { bearing_friction: 1.5, compressor_efficiency_loss: 0.0, compressor_flow_capacity_loss: 0.0, turbine_efficiency_loss: 0.0 }),
];

/// The EEC's ground protections, EASA.E.012 §IV.3: on the ground below
/// 60 kt it prevents stabilised operation at 64-72% N1 (the keep-out zone;
/// a target inside it is moved to the nearer edge, while passing through
/// it is never blocked), and below 32.5 kt it limits N1 to 78% (the
/// modified take-off thrust setting).
pub(crate) fn ground_n1_protection(target_n1: f64, on_ground: bool, ground_speed_kt: f64) -> f64 {
    const KEEP_OUT: (f64, f64) = (64.0, 72.0);
    const KEEP_OUT_BELOW_KT: f64 = 60.0;
    const LIMITED_N1: f64 = 78.0;
    const LIMITED_BELOW_KT: f64 = 32.5;
    if !on_ground {
        return target_n1;
    }
    let mut n1 = target_n1;
    if ground_speed_kt < KEEP_OUT_BELOW_KT && n1 > KEEP_OUT.0 && n1 < KEEP_OUT.1 {
        n1 = if n1 < (KEEP_OUT.0 + KEEP_OUT.1) / 2.0 { KEEP_OUT.0 } else { KEEP_OUT.1 };
    }
    if ground_speed_kt < LIMITED_BELOW_KT {
        n1 = n1.min(LIMITED_N1);
    }
    n1
}

#[cfg(test)]
mod ground_protection_tests {
    use super::ground_n1_protection as p;

    #[test]
    fn keep_out_zone_and_low_speed_limit_apply_only_on_the_ground_and_slow() {
        assert_eq!(p(66.0, true, 10.0), 64.0, "inside the zone, nearer the bottom");
        assert_eq!(p(70.0, true, 10.0), 72.0, "inside the zone, nearer the top");
        assert_eq!(p(70.0, true, 65.0), 70.0, "above 60 kt the zone is off");
        assert_eq!(p(95.0, true, 20.0), 78.0, "below 32.5 kt N1 is limited to 78%");
        assert_eq!(p(95.0, true, 40.0), 95.0, "above 32.5 kt full take-off N1");
        assert_eq!(p(66.0, false, 10.0), 66.0, "airborne: no ground protection");
    }
}
