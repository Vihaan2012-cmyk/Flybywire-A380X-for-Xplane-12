//! A deterministic, X-Plane-free test bench for the FlyByWire A380X X-Plane
//! port.
//!
//! [`Emulator`] owns the same pieces the real plugin's `Plugin` does --
//! FlyByWire's own `Simulation<A380>`, this port's `Vars` (FlyByWire's
//! `VariableRegistry`/`SimulatorReaderWriter`), the breaker/circuit
//! catalogues, the failure catalogue, and the three "hyperrealism" physics
//! modules that couple the engines to the electrical/hydraulic/bleed
//! systems -- and ticks them in the same relative order
//! `fbw_a380_systems_xp`'s own `Plugin::tick` does (see `tick_order` below
//! and that crate's `src/lib.rs`). No X-Plane process is opened; there is
//! no reimplementation of any of this behaviour, only real production code
//! run against an offline `Vars` built on `Xplm::dummy()` (a do-nothing
//! XPLM binding), exactly the seam `fbw_a380_systems_xp`'s own
//! `src/offline_harness.rs` documents and exercises.
//!
//! ## What this does *not* cover, and why
//!
//! - **What does run of the X-Plane-facing modules**: the plugin's engine and
//!   fuel chain -- throttles, FADEC, PRIMs, engine commands with the engine
//!   physics, FCDCs, bays and `fuel.rs` -- in `Plugin::tick`'s order
//!   (`offline_chain`), on an `Xplm::memory()` binding whose datarefs hold
//!   what those modules write (the engine physics' N1/N2 the FADEC reads
//!   back) and read 0 otherwise. So the APU starts on real fuel pressure and
//!   the engines start, burn fuel and load the generators, pumps and bleed.
//!   The fuel system's saved tank levels are not read or written.
//! - **Not run**: the radios/sensors/ADIRS glue, doors, efb, sound, mapdata,
//!   `extra_backend_fbw`, damage/tyres/X-Plane effects: they need X-Plane's
//!   own flight model, weather or renderer behind their datarefs, which no
//!   offline binding supplies.
//! - **The JS/QuickJS cockpit instruments (`src/js`, `src/display`,
//!   feature `js`)**: CEF/JS-rendered screens, entirely out of scope for a
//!   headless physics/systems test bench; not linked in here at all (this
//!   crate does not enable the plugin's `js` feature).
//! - **Passenger count / cargo / fuel loading**: FlyByWire's own
//!   `PAYLOAD STATION WEIGHT:n` (`weight_balance.rs`/`payload.rs` in
//!   `a380_systems`) is reachable and wired below (`Loading`), but this
//!   pass did not derive the FCOM per-station standard passenger/cargo
//!   split table -- see [`Loading::set_pax_and_cargo`]'s doc comment for
//!   the simplification used instead (a single evenly-distributed
//!   approximation), called out there rather than silently presented as
//!   the certified loadsheet split.
//! - **Ground services (ground air, refuel, pushback)**: the plugin's own
//!   modules for these (`efb.rs`, `doors.rs`, `sensors.rs` pushback state)
//!   are not run; a ground power cart is connected by writing
//!   `EXT_PWR_AVAIL:1` (`presets::ground_power`). FlyByWire's own external-power/APU-bleed
//!   logic inside `a380_systems` itself *is* reachable, through the same
//!   named/simulator variables `Vars` publishes (see `set_var`).
//!
//! Everything else the task asked for -- every breaker, every failure
//! (with continuous magnitude), every circuit, wear, invariants, MEL,
//! scripted and random failure triggers, FlyByWire's own full systems tick
//! (electrical, hydraulics, pneumatics, ADIRS state machine, flight
//! controls, APU, fire protection, payload) -- runs for real.

use std::time::Duration;

use a380_systems::A380;
use systems::simulation::{Simulation, SimulatorReaderWriter, StartState, VariableRegistry};

use fbw_a380_systems::test_support::{aspects, breakers, circuits, failures, invariants, physics, random_failures, scenarios, xp, Vars};

pub mod controls;
pub mod presets;
pub mod work;
pub mod flight_model;

pub use fbw_a380_systems::test_support;

/// A fresh, offline `Vars` bound to a leaked `Xplm::dummy()` (a do-nothing
/// XPLM binding, test-only): the same construction `offline_harness.rs`
/// uses. `Vars::new` wants `&'static Xplm`, the same signature the real
/// `Plugin::new` uses with a live `Xplm::load()`.
fn offline_vars() -> (&'static xp::Xplm, Vars) {
    // Datarefs that hold what the plugin's modules write to them (the engine
    // physics' N1/N2 the FADEC reads back); nothing X-Plane computes.
    xp::Xplm::reset_memory();
    let xplm: &'static xp::Xplm = Box::leak(Box::new(xp::Xplm::memory()));
    (xplm, Vars::new(xplm))
}

/// The full offline test bench: FlyByWire's `Simulation<A380>`, this port's
/// `Vars`, and every module reachable without a live X-Plane process (see
/// the module doc for what's excluded and why).
pub struct Emulator {
    vars: Vars,
    simulation: Simulation<A380>,
    aspects: aspects::Aspects,
    breakers: breakers::Breakers,
    circuits: circuits::Circuits,
    failures: failures::Failures,
    electrical_loads: physics::electrical::EngineLoads,
    circuit_protection: physics::electrical::CircuitProtection,
    bleed_loads: physics::air::EngineBleedLoads,
    hydraulics: physics::hydraulics::Hydraulics,
    random_failures: random_failures::RandomFailures,
    /// The plugin's engine and fuel chain (throttles, FADEC, PRIMs, engine
    /// commands and physics, FCDCs, bays, fuel), ticked in `Plugin::tick`'s
    /// order: the engines start, burn fuel and load the generators, pumps
    /// and bleed through the live code (`offline_chain`'s module doc).
    chain: test_support::offline_chain::EngineChain,
    /// FlyByWire's four gas-turbine engine models (this port's own
    /// replacement for MSFS's engine simulation, `physics::engine`), driven
    /// separately from `simulation.tick` -- see [`Emulator::step_engine`]'s
    /// doc comment for why they are not auto-coupled into
    /// [`Emulator::tick`].
    pub engines: [physics::engine::Engine; 4],
    time_s: f64,
}

impl Emulator {
    /// A cold-and-dark aircraft in the given FlyByWire start state, with
    /// every process-global module (failures, breakers, circuits, random/
    /// scripted-failure triggers, damage, motor wear) reset first
    /// ([`scenarios::reset_global_state`]) so this is deterministic
    /// regardless of what any earlier `Emulator` in the same test process
    /// left behind. A baseline `TOTAL WEIGHT` is written (a parked aircraft
    /// is never actually weightless; see `start_state.rs`'s cold-and-dark
    /// test for the same reasoning) so FlyByWire's own
    /// `total_weight_on_wheels / total_weight()` ground-weight ratio isn't
    /// a `0. / 0.` on the very first tick.
    pub fn new(state: StartState) -> Self {
        scenarios::reset_global_state();
        fbw_a380_systems::test_support::wear::reset_all();
        invariants::reset();

        let (xplm, mut vars) = offline_vars();
        // X-Plane always feeds an atmosphere (lib.rs `mapping()`); with no
        // dataref behind them these would read 0, i.e. a vacuum at 0 K, and
        // FlyByWire's pneumatic containers go 0/0 on the first tick. Start at
        // ISA sea level; the environment setters below override it.
        for (name, value) in [
            ("AMBIENT TEMPERATURE", 15.0),
            ("AMBIENT PRESSURE", 101_325.0 * 0.0002953),
            ("PRESSURE ALTITUDE", 0.0),
            ("AMBIENT DENSITY", 1.225 * 0.00194032),
            // The live plugin decides and writes A32NX_START_STATE before the
            // systems are built (start_state::detect); fuel and the FADEC
            // read it.
            ("START_STATE", f64::from(state)),
        ] {
            let id = vars.get(name.to_owned());
            vars.write(&id, value);
        }
        let simulation = Simulation::new(state, A380::new, &mut vars);
        // The cockpit's switch positions at spawn, from the same
        // cockpit_variables.txt the plugin loads (lib.rs
        // `register_cockpit_variables`): without them every switch reads 0,
        // so e.g. each engine-driven pump's DISC pushbutton reads "not AUTO"
        // and the pumps' clutches disconnect. `FBW_COCKPIT_VARIABLES`
        // overrides the path.
        let cockpit_variables = std::env::var("FBW_COCKPIT_VARIABLES").unwrap_or_else(|_| {
            "D:/Steam Games/steamapps/common/X-Plane 12/Aircraft/FlyByWire A380X/plugins/fbw_a380_systems/cockpit_variables.txt"
                .to_owned()
        });
        if let Ok(text) = std::fs::read_to_string(&cockpit_variables) {
            vars.register_cockpit_variables_text(&text);
        }
        let chain = test_support::offline_chain::EngineChain::new(&mut vars, xplm, state.into());
        // Every catalogued failure as a component (after the engines registered theirs).
        test_support::failures::register_component_catalogue();
        let aspects = aspects::a380(&mut vars);
        let circuits = circuits::Circuits::new(&mut vars);
        let breakers = breakers::Breakers::new(&mut vars);
        let failures = failures::Failures::new();
        let electrical_loads = physics::electrical::EngineLoads::new(&mut vars);
        let circuit_protection = physics::electrical::CircuitProtection::new(&mut vars, &circuits);
        let bleed_loads = physics::air::EngineBleedLoads::new(&mut vars);
        let hydraulics = physics::hydraulics::Hydraulics::new(&mut vars);
        let random_failures = random_failures::RandomFailures::new(0x1234_5678_9abc_def0);
        let engines = std::array::from_fn(|_| physics::engine::Engine::new());

        let id = vars.get("TOTAL WEIGHT".to_owned());
        vars.write(&id, 600_000.0);

        Self {
            vars,
            simulation,
            aspects,
            breakers,
            circuits,
            failures,
            electrical_loads,
            circuit_protection,
            bleed_loads,
            hydraulics,
            random_failures,
            chain,
            engines,
            time_s: 0.0,
        }
    }

    /// One tick, in `fbw_a380_systems_xp`'s own `Plugin::tick` order (see
    /// that crate's `src/lib.rs` `fn tick`), for the pieces reachable
    /// offline:
    ///
    /// 1. `invariants::advance_tick` (the clamp-log's own tick counter);
    /// 2. `circuits.apply_requests` / `breakers.apply_requests` /
    ///    `breakers.pre_systems` -- Study-panel-style pull/reset requests
    ///    (see [`Emulator::pull_breaker`] etc.) land before the systems see
    ///    them, and every catalogue breaker's closed state bridges onto
    ///    FlyByWire's own failure ids / absorbed circuits;
    /// 3. `failures.apply` -- hands the active failure set (with
    ///    magnitudes already live in `failures::magnitude`) to
    ///    `Simulation::update_active_failures` before the tick that must
    ///    see them;
    /// 4. `aspects.pre_tick` / `simulation.tick` / `aspects.post_tick` --
    ///    FlyByWire's own full systems tick, the same bridging layer the
    ///    real plugin runs;
    /// 5. `hydraulics.update` / `electrical_loads.update` /
    ///    `breakers.post_systems` / `bleed_loads.update` -- the
    ///    hyperrealism engine-coupling contract terms and the wider
    ///    breaker catalogue's real current/thermal trip, in the real
    ///    plugin's own post-tick order;
    /// 6. random failure triggers (`random_failures.update`), same as the
    ///    real plugin.
    ///
    /// Not run here: `physics::damage`/`physics::tyre`/`physics::xp_effects`
    /// (X-Plane-visible failure effects: fire, smoke, tyre burst datarefs --
    /// state machines that exist to drive X-Plane's renderer, not physics a
    /// test would assert on) and `mel`'s own per-tick `apply_requests` (its
    /// deferral list needs `persistence::AirframeState::airframe_hours`,
    /// which this bench does not track a running total of); both catalogues
    /// are still fully readable/settable through their own modules (see
    /// `test_support::physics`, `test_support::mel`).
    pub fn tick(&mut self, dt: f64) {
        invariants::advance_tick(dt);
        self.circuits.apply_requests(&mut self.vars);
        self.breakers.apply_requests(&mut self.vars);
        self.breakers.pre_systems(&mut self.vars, &mut self.circuits);

        self.failures.apply(&mut self.simulation, &mut self.vars, None);

        // Plugin::tick: the engines first, so the systems see this tick's.
        self.chain.before_systems(&mut self.vars, dt, self.time_s);

        self.aspects.pre_tick(&mut self.vars, dt);
        self.time_s += dt;
        self.simulation.tick(Duration::from_secs_f64(dt), self.time_s, &mut self.vars);
        self.aspects.post_tick(&mut self.vars);

        self.hydraulics.update(&mut self.vars);
        self.electrical_loads.update(&mut self.vars);
        self.circuit_protection.update(&mut self.vars, &mut self.circuits, dt);
        self.chain.bays(&mut self.vars, dt);
        self.breakers.post_systems(&mut self.vars, dt);
        self.bleed_loads.update(&mut self.vars);
        // Plugin::tick: damage and tyres, then fuel, after the systems.
        self.chain.damage_and_tyres(&mut self.vars, dt);
        self.chain.fuel(&mut self.vars, dt, &self.circuits);

        // Damageable components: progress and recombine, as Plugin::tick.
        test_support::components::tick(dt / 3600.0);
        self.random_failures.apply_requests();
        let already: std::collections::BTreeSet<u64> = failures::active_ids().into_iter().collect();
        for id in self.random_failures.update(dt / 3600.0, &already) {
            failures::set_active(id, true);
        }
    }

    /// Ticks `n` times at `dt` seconds each.
    pub fn run(&mut self, dt: f64, n: u32) {
        for _ in 0..n {
            self.tick(dt);
        }
    }

    pub fn elapsed_s(&self) -> f64 {
        self.time_s
    }

    // ---------------------------------------------------------------
    // Raw escape hatch.
    // ---------------------------------------------------------------

    /// Set any FlyByWire/plugin variable by name: a simulator variable
    /// (MSFS-style, contains a space -- e.g. `"AMBIENT TEMPERATURE"`,
    /// `"TOTAL WEIGHT"`) or an aircraft variable (FlyByWire's own,
    /// `A32NX_...` prefix optional -- `Vars::get` adds it). This is the
    /// same call FlyByWire's own systems make (`VariableRegistry::get` +
    /// `SimulatorReaderWriter::write`); nothing about it is emulator-only.
    ///
    /// For a simulator variable that has a real X-Plane dataref mapping in
    /// the plugin (`lib.rs`'s private `mapping()` table -- e.g.
    /// `"AMBIENT TEMPERATURE"`, `"AIRSPEED INDICATED"`), the live plugin
    /// feeds it from that dataref every tick (`Vars::read_inputs`); this
    /// bench never calls `read_inputs` (there is no dataref to read --
    /// `Xplm::dummy()`'s `find` always answers "no such dataref"), so a
    /// value written here simply stays, the same value every subsequent
    /// tick until written again. That is the offline equivalent of a fixed
    /// dataref reading, not a behavioural difference in the systems code.
    pub fn set_var(&mut self, name: &str, value: f64) {
        let id = self.vars.get(name.to_owned());
        self.vars.write(&id, value);
    }

    /// Read any FlyByWire/plugin variable by name (see [`Self::set_var`]).
    pub fn get_var(&mut self, name: &str) -> f64 {
        let id = self.vars.get(name.to_owned());
        self.vars.read(&id)
    }

    /// Set a cockpit control's own dataref by its exact name from
    /// `cockpit_bindings.txt` (e.g. `"fbw/cockpit/BIGARMREST_CPT_TILT_CLICK"`,
    /// `"fbw/cockpit/ie/A380X_PED_RMP_1_STBY_RAD_NAV_COVER"`), the same
    /// registration path the plugin's `register_cockpit_variables` uses
    /// (`Vars::register_named`) -- unlike [`Self::set_var`], this never
    /// injects FlyByWire's `A32NX_` prefix, matching a cockpit dataref name
    /// exactly as X-Plane sees it.
    pub fn set_dataref(&mut self, name: &str, value: f64) {
        let id = self.vars.register_named(name);
        self.vars.write(&id, value);
    }

    pub fn get_dataref(&mut self, name: &str) -> f64 {
        let id = self.vars.register_named(name);
        self.vars.read(&id)
    }

    // ---------------------------------------------------------------
    // Environment.
    // ---------------------------------------------------------------

    /// Outside air temperature, degrees Celsius (`"AMBIENT TEMPERATURE"`).
    pub fn set_oat_c(&mut self, c: f64) {
        self.set_var("AMBIENT TEMPERATURE", c);
        self.set_density_from_state();
    }
    /// Static (ambient) pressure, pascals. In the live plugin
    /// `"AMBIENT PRESSURE"`, `"PRESSURE ALTITUDE"` and (with OAT)
    /// `"AMBIENT DENSITY"` all follow X-Plane's one barometer dataref
    /// (lib.rs `mapping()`), so all three are set from it here too.
    pub fn set_qnh_pa(&mut self, pa: f64) {
        self.set_var("AMBIENT PRESSURE", pa * 0.0002953);
        // lib.rs `pressure_altitude_ft`.
        self.set_var("PRESSURE ALTITUDE", (1. - (pa / 101_325.).powf(0.190_284)) * 145_366.45);
        self.set_density_from_state();
    }
    /// Pressure altitude, feet: sets the static pressure that altitude
    /// stands for (the exact inverse of lib.rs `pressure_altitude_ft`), so
    /// a case at FL350 has FL350's air, not sea level's.
    pub fn set_pressure_altitude_ft(&mut self, ft: f64) {
        let pa = 101_325. * (1. - ft / 145_366.45).max(0.).powf(1. / 0.190_284);
        self.set_qnh_pa(pa);
    }
    /// Ideal-gas density from the current static pressure and OAT, in the
    /// slug/ft^3 `"AMBIENT DENSITY"` carries.
    fn set_density_from_state(&mut self) {
        let pa = self.get_var("AMBIENT PRESSURE") / 0.0002953;
        let k = self.get_var("AMBIENT TEMPERATURE") + 273.15;
        if k > 0. {
            self.set_var("AMBIENT DENSITY", pa / (287.05 * k) * 0.00194032);
        }
    }
    pub fn set_wind_ms(&mut self, x: f64, y: f64, z: f64) {
        self.set_var("AMBIENT WIND X", x);
        self.set_var("AMBIENT WIND Y", y);
        self.set_var("AMBIENT WIND Z", z);
    }
    /// FlyByWire reads this as a `0.0..=1.0` ratio (`"STRUCTURAL ICE PCT"`).
    pub fn set_structural_icing_fraction(&mut self, f: f64) {
        self.set_var("STRUCTURAL ICE PCT", f);
    }
    pub fn set_on_ground(&mut self, on_ground: bool) {
        self.set_var("SIM ON GROUND", on_ground as u8 as f64);
    }

    // ---------------------------------------------------------------
    // Flight state.
    // ---------------------------------------------------------------

    pub fn set_indicated_airspeed_kt(&mut self, kt: f64) {
        self.set_var("AIRSPEED INDICATED", kt);
    }
    pub fn set_true_airspeed_kt(&mut self, kt: f64) {
        self.set_var("AIRSPEED TRUE", kt / 1.943_844);
    }
    pub fn set_mach(&mut self, m: f64) {
        self.set_var("AIRSPEED MACH", m);
    }
    pub fn set_groundspeed_kt(&mut self, kt: f64) {
        self.set_var("GPS GROUND SPEED", kt / 1.943_844);
    }
    pub fn set_agl_ft(&mut self, ft: f64) {
        self.set_var("PLANE ALT ABOVE GROUND", ft);
        self.set_var("PLANE ALT ABOVE GROUND MINUS CG", ft);
    }
    pub fn set_pitch_deg(&mut self, deg: f64) {
        self.set_var("PLANE PITCH DEGREES", -deg);
    }
    pub fn set_bank_deg(&mut self, deg: f64) {
        self.set_var("PLANE BANK DEGREES", -deg);
    }
    pub fn set_heading_true_deg(&mut self, deg: f64) {
        self.set_var("PLANE HEADING DEGREES TRUE", deg);
    }
    pub fn set_gear_handle_down(&mut self, down: bool) {
        self.set_var("GEAR HANDLE POSITION", down as u8 as f64);
    }
    /// Gross weight, pounds (`"TOTAL WEIGHT"`). See [`Self::new`] for why a
    /// realistic value should always be set.
    pub fn set_total_weight_lb(&mut self, lb: f64) {
        self.set_var("TOTAL WEIGHT", lb);
    }

    // ---------------------------------------------------------------
    // Weights and loading (FlyByWire's own payload stations).
    // ---------------------------------------------------------------

    /// FlyByWire's raw per-station payload weight, pounds
    /// (`"PAYLOAD STATION WEIGHT:n"`, `n` in `1..=18`; the A380's cabin/
    /// cargo zones -- see `a380_systems::payload` for what each number is).
    pub fn set_payload_station_lb(&mut self, station: u32, lb: f64) {
        self.set_var(&format!("PAYLOAD STATION WEIGHT:{station}"), lb);
    }

    // ---------------------------------------------------------------
    // Breakers (docs/analysis/cockpit-study-cbs.md's catalogue).
    // ---------------------------------------------------------------

    /// Every catalogued breaker, generated from the plugin's own
    /// `breakers::catalog()` -- not hand-maintained here.
    pub fn list_breakers(&self) -> &'static [breakers::BreakerDef] {
        breakers::catalog()
    }

    /// Requests a pull (open); takes effect on the next [`Self::tick`]
    /// (`Breakers::apply_requests`, the same as the Study panel).
    pub fn pull_breaker(&self, id: &str) {
        breakers::request_pull(id.to_owned());
    }
    pub fn reset_breaker(&self, id: &str) {
        breakers::request_reset(id.to_owned());
    }
    pub fn reset_all_breakers(&self) {
        breakers::request_reset_all();
    }
    /// Live state (closed/current/trip cause) for every catalogued
    /// breaker, as of the last [`Self::tick`].
    pub fn breaker_states(&mut self) -> Vec<breakers::BreakerSnapshot> {
        self.breakers.snapshot(&mut self.vars)
    }

    // ---------------------------------------------------------------
    // Circuits (systems.cfg's own [ELECTRICAL] section).
    // ---------------------------------------------------------------

    pub fn set_circuit_breaker(&mut self, number: usize, closed: bool) {
        self.circuits.set_breaker(&mut self.vars, number, closed);
    }
    pub fn circuit_breaker_closed(&mut self, number: usize) -> bool {
        self.circuits.breaker_closed(&mut self.vars, number)
    }
    pub fn circuit_powered(&mut self, number: usize) -> bool {
        self.circuits.powered(&mut self.vars, number)
    }

    // ---------------------------------------------------------------
    // Failures (0..=1 continuous magnitude; docs/physics/failures.md).
    // ---------------------------------------------------------------

    /// Every failure id this build's `Simulation` registered, generated
    /// from `Failures::ids` (FlyByWire's own catalogue plus this port's
    /// computer/extra failures) -- not hand-maintained here.
    pub fn list_failures(&self) -> Vec<u64> {
        self.failures.ids().collect()
    }
    pub fn failure_name(&self, id: u64) -> String {
        failures::failure_name(id)
    }
    /// Sets `id`'s magnitude in `0.0..=1.0` (`<= 0.0` clears it); takes
    /// effect on the next [`Self::tick`] (`Failures::apply`).
    pub fn set_failure_magnitude(&self, id: u64, magnitude: f64) {
        failures::set_magnitude(id, magnitude);
    }
    pub fn failure_magnitude(&self, id: u64) -> f64 {
        failures::magnitude(id)
    }
    pub fn active_failures(&self) -> Vec<u64> {
        failures::active_ids()
    }

    // ---------------------------------------------------------------
    // Wear (per component; wear.rs).
    // ---------------------------------------------------------------

    pub fn wear(&self, component_id: &str) -> fbw_a380_systems::test_support::wear::Wear {
        fbw_a380_systems::test_support::wear::snapshot().get(component_id)
    }

    // ---------------------------------------------------------------
    // Invariants (NaN/impossible-value clamp log; invariants.rs).
    // ---------------------------------------------------------------

    pub fn invariant_report(&self) -> Vec<invariants::Violation> {
        invariants::report()
    }

    // ---------------------------------------------------------------
    // Engines (physics::engine -- this port's own gas-turbine model).
    // ---------------------------------------------------------------

    /// Steps engine `n` (`0..=3`) directly against `inputs`, the same way
    /// `offline_harness.rs`'s own proven test does, and returns its
    /// outputs. This is deliberately **not** run automatically inside
    /// [`Self::tick`]: the live plugin's real coupling from engine output
    /// back into FlyByWire's own N1/EGT/fuel-flow variables goes through
    /// `engine_commands.rs`, which is `Xplm`-bound (reads
    /// `GENERAL ENG THROTTLE LEVER POSITION`/writes `GENERAL ENG RPM`-style
    /// simulator variables via the live dataref path) and was not opened up
    /// in this pass -- see the module doc's "what this does not cover".
    /// The *electrical/hydraulic/bleed* half of the coupling (accessory
    /// load flowing from the systems tick back onto `EngineInputs`) *is*
    /// exercised for real, the same way, by
    /// `offline_harness.rs`'s own cross-system test: read
    /// `"ENGINE_GEARBOX_ELEC_LOAD_W:n"` / `"ENGINE_GEARBOX_HYD_LOAD_W:n"` /
    /// `"ENGINE_BLEED_EXTRACTION_KG_S:n"` with [`Self::get_var`] after
    /// [`Self::tick`], and feed them into `inputs` here.
    pub fn step_engine(&mut self, n: usize, inputs: &physics::engine::EngineInputs) -> physics::engine::EngineOutputs {
        self.engines[n].step(inputs)
    }

    /// Runs just `physics::electrical::EngineLoads::update` (the
    /// `"ELEC_ENG_GEN_n_SHAFT_POWER_DEMAND"` -> `"ENGINE_GEARBOX_ELEC_LOAD_W:n"`
    /// contract), without a full [`Self::tick`]. Useful for testing that one
    /// contract in isolation: inside a full tick, FlyByWire's own systems
    /// tick computes and overwrites the demand variable itself (it is the
    /// generator's own Kirchhoff-solved shaft power, not a free-standing
    /// input) before this runs, so a value set with [`Self::set_var`] right
    /// before [`Self::tick`] will not, in general, survive to be read back
    /// unchanged -- this method lets a test drive the contract directly, the
    /// same way `offline_harness.rs`'s own proven test does.
    pub fn update_electrical_loads(&mut self) {
        self.electrical_loads.update(&mut self.vars);
    }

    /// Direct access to `Vars`' underlying registry, for anything not
    /// covered by a typed method above.
    pub fn vars(&mut self) -> &mut Vars {
        &mut self.vars
    }

    /// Every variable registered so far, with its current value
    /// (`Vars::snapshot_all`, `test-support`).
    pub fn snapshot_all(&self) -> Vec<(String, f64)> {
        self.vars.snapshot_all()
    }

    /// A thrust lever, as X-Plane supplies it (`throttle_jet_rev_ratio`,
    /// 0 idle .. 1 full forward; the plugin's throttle mapping reads it).
    pub fn set_thrust_lever(&mut self, engine: usize, ratio: f64) {
        xp::Xplm::memory_set("sim/cockpit2/engine/actuators/throttle_jet_rev_ratio", engine - 1, ratio);
    }

    /// Engine `engine` (1-4)'s accumulated life from the damage model.
    pub fn engine_wear(&self, engine: usize) -> test_support::physics::damage::EngineWear {
        self.chain.engine_wear()[engine - 1]
    }

    /// An engines-running spawn: every engine's physics at its own settled
    /// ground idle. Set the masters on and ignition to NORM with it.
    pub fn spawn_engines_at_idle(&mut self) {
        self.chain.spawn_engines_at_idle();
    }

    /// Set any damageable component's physical parameter (`components.rs`),
    /// optionally worsening at `rate_per_hour` in its own unit.
    pub fn set_component_param(&mut self, component: &str, param: &str, value: f64, rate_per_hour: f64) -> Result<(), String> {
        test_support::components::set_direct(component, param, value, rate_per_hour)
    }

    /// Every registered damageable component parameter.
    pub fn list_component_params(&self) -> Vec<(String, String, f64)> {
        test_support::components::list().into_iter().map(|c| (c.component, c.spec.name.to_owned(), c.value)).collect()
    }

    /// Every variable's value in [`Self::snapshot_all`]'s order, without
    /// copying names (the per-tick checks).
    pub fn values(&self) -> Vec<f64> {
        self.vars.values()
    }

    /// How many variables exist (the length of [`Self::values`]).
    pub fn var_count(&self) -> usize {
        self.vars.len()
    }

    /// Every cockpit control's dataref, from `cockpit_bindings.txt`
    /// (see [`controls::list`]; empty if that file is not present in this
    /// environment -- not an error, see its doc comment).
    pub fn list_controls(&self) -> Vec<controls::Control> {
        controls::list(None)
    }

    /// Every breaker id, every failure id and every cockpit control
    /// dataref this `Emulator` exposes -- for the coverage test asserting
    /// each catalogue entry is reachable through the API (see
    /// `tests/coverage.rs`).
    pub fn list_breaker_ids(&self) -> Vec<&'static str> {
        self.list_breakers().iter().map(|b| b.id).collect()
    }
}
