//! The plugin's side of the live layer: what fills [`Truth`] every frame,
//! what takes the [`Faults`] snapshot, and what a published name becomes.
//!
//! `deep::live` defines the contract; this file is the only place that
//! knows both that contract and `crate::Vars`/X-Plane, which is what keeps
//! `docs/deep/BRIEF.md` hard rule 2 (no area depends on `Vars` or X-Plane)
//! true even once every area is running. `Plugin` owns one [`DeepLayer`]
//! and calls [`DeepLayer::tick`] once per frame.
//!
//! ## Where every `Truth` field comes from
//!
//! The standing rule is that no field may hold an invented number: each is
//! either a real reading, or left at the value `Truth::default()`
//! documents. Per field:
//!
//! | `Truth` field | Source |
//! |---|---|
//! | `dt_s` | the frame time `Plugin::tick` is given, clamped to [`MIN_DT_S`]..[`MAX_DT_S`] |
//! | `environment` | `deep::integration::weather_truth::WeatherTruthReader` (real X-Plane weather; see its own module doc for each of its fields) |
//! | `altitude_ft` | `sim/flightmodel/position/elevation` (MSL metres), X-Plane's own position |
//! | `on_ground` | `sim/flightmodel/failures/onground_any`, the same dataref `physics/adirs.rs` and `physics/damage.rs` already read |
//! | `engine_n1_frac[i]` | `ENGINE_N1:n` / 100, this crate's own `physics::engine` output (`engine_commands.rs:494`) |
//! | `engine_running[i]` | `ENGINE_STATE:n` == `EngineState::On`, FlyByWire's own start-state machine (`fadec.rs`) |
//! | `engine_bleed_pressure_pa[i]`, `engine_bleed_temp_k[i]` | `ENGINE_{IP,HP}_PORT_{PRESSURE_PA,TEMP_K}:n`, `physics::engine`'s own customer-bleed port outputs, picked by which port the engine is actually bled from (`PNEU_ENG_n_HP_VALVE_OPEN`, exactly as `engine_commands.rs:466` decides it) |
//! | `apu_running` | `A32NX_OVHD_APU_START_PB_IS_AVAILABLE`, FlyByWire's own APU ECB `is_available()` |
//! | `apu_bleed_pressure_pa` | `A32NX_APU_BLEED_AIR_PRESSURE`, FlyByWire's own ARINC 429 word (psi absolute) |
//! | `ac_bus_volts[i]` | `A32NX_ELEC_AC_{1..4}_BUS_POTENTIAL`, FlyByWire's own electrical system |
//! | `dc_bus_volts[i]` | `A32NX_ELEC_DC_{1,2}_BUS_POTENTIAL`, ditto |
//! | `hydraulic_pressure_pa[i]` | `A32NX_HYD_{GREEN,YELLOW}_SYSTEM_1_SECTION_PRESSURE` (psi), FlyByWire's own hydraulic system |
//!
//! Nothing here derives a value it cannot read. Where a dataref is missing
//! (an older SDK target, or the offline harness) the reading degrades to
//! the field's documented `Truth::default()` value rather than to zero --
//! `ambient_pressure_pa` in particular, since several areas divide by it
//! and zero is a vacuum, not a missing reading.
//!
//! ## Frame cost
//!
//! Everything that can be done once is done once:
//!
//! * every variable read is through a `VariableIdentifier` resolved in
//!   [`DeepLayer::new`], never by name;
//! * every variable *written* goes through the `Publisher` cache built in
//!   [`DeepLayer::new`] from [`Deep::published_names`], so the frame loop
//!   resolves a published name in one string comparison and never
//!   allocates;
//! * `XPLMGetWeatherAtLocation`, which X-Plane's own header says is not
//!   for per-frame use, is called at [`WEATHER_INTERVAL_S`] rather than
//!   every frame;
//! * the deep failure id set is built once, from `deep::registry()`, into
//!   a `BTreeSet`;
//! * the per-frame [`Faults`] snapshot takes `crate::failures`' lock
//!   **once** (`failures::active_magnitudes`) and keeps the armed ids that
//!   are deep ones, instead of asking `failures::armed_magnitude(id)` for
//!   each of the several thousand registered deep ids in turn. The result
//!   is identical -- `armed_magnitude` returns `0.0` for an id that is not
//!   in the active set, and `Faults::get` returns `0.0` for an id that is
//!   not in the snapshot -- but it is one lock per frame instead of
//!   thousands.

use std::collections::{BTreeSet, HashMap};

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::deep::integration::weather_truth::{EnvironmentTruth, WeatherTruthReader};
use crate::deep::live::{Deep, Faults, Truth};
use crate::xp::{DataRef, Xplm};
use crate::Vars;

/// psi -> Pa, the same constant `fuel.rs` and the deep areas already use.
const PSI_TO_PA: f64 = 6894.757;

/// The shortest frame the models are stepped with. X-Plane hands out a
/// zero `dt` on the frame a flight loads; several areas integrate against
/// `dt_s` and a few divide by it, so it is floored here at the same 1 ms
/// `lib.rs`'s own flight-loop clamp uses.
pub const MIN_DT_S: f64 = 0.001;
/// The longest. After a pause, a scenery load or a long frame X-Plane
/// reports the whole wall-clock gap; integrating a first-order lag across
/// several seconds in one step is where an exact-exponential model stays
/// stable but an explicit one does not, and no model here is validated
/// beyond a 5 Hz step. Same value as `lib.rs`'s own flight-loop clamp.
pub const MAX_DT_S: f64 = 0.2;

/// ARINC 429 sign/status "normal operation": the sender vouches for the
/// value. FlyByWire packs a 32 bit float's bits in the low half of the
/// `f64` and the status in the two bits above
/// (`fbw-common/.../shared/arinc429.rs:154`, `to_arinc429`).
const SSM_NORMAL_OPERATION: u32 = 3;

/// One ARINC 429 word as FlyByWire packs it: the value and its status.
fn unpack_arinc(packed: f64) -> (f64, u32) {
    let bits = packed as u64;
    (f32::from_bits(bits as u32) as f64, ((bits >> 32) & 0b11) as u32)
}

/// `fadec.rs`'s `EngineState::On`: the core is turning and lit.
const ENGINE_STATE_ON: f64 = 1.0;

/// The variables one engine contributes to [`Truth`].
struct EngineIds {
    n1_pct: VariableIdentifier,
    state: VariableIdentifier,
    ip_port_pressure_pa: VariableIdentifier,
    ip_port_temp_k: VariableIdentifier,
    hp_port_pressure_pa: VariableIdentifier,
    hp_port_temp_k: VariableIdentifier,
    /// Which customer port this engine is bled from this tick:
    /// `engine_commands.rs` sets `bleed_from_ip_port` to `hp_valve_open ==
    /// 0`, so the same test picks the same port's pressure/temperature
    /// here and the two can never disagree.
    hp_valve_open: VariableIdentifier,
}

impl EngineIds {
    fn new(vars: &mut Vars, n: usize) -> Self {
        Self {
            n1_pct: vars.get(format!("ENGINE_N1:{n}")),
            state: vars.get(format!("ENGINE_STATE:{n}")),
            ip_port_pressure_pa: vars.get(format!("ENGINE_IP_PORT_PRESSURE_PA:{n}")),
            ip_port_temp_k: vars.get(format!("ENGINE_IP_PORT_TEMP_K:{n}")),
            hp_port_pressure_pa: vars.get(format!("ENGINE_HP_PORT_PRESSURE_PA:{n}")),
            hp_port_temp_k: vars.get(format!("ENGINE_HP_PORT_TEMP_K:{n}")),
            hp_valve_open: vars.get(format!("PNEU_ENG_{n}_HP_VALVE_OPEN")),
        }
    }
}

/// Every variable [`Truth`] is filled from, resolved once.
struct Ids {
    engines: [EngineIds; 4],
    apu_available: VariableIdentifier,
    apu_bleed_air_pressure: VariableIdentifier,
    ac_bus_potential: [VariableIdentifier; 4],
    dc_bus_potential: [VariableIdentifier; 2],
    /// Green and yellow, in `Truth::hydraulic_pressure_pa`'s order.
    hydraulic_pressure_psi: [VariableIdentifier; 2],
}

/// The X-Plane datarefs [`Truth`] is filled from, found once.
struct Refs {
    /// `sim/flightmodel/position/elevation`, MSL metres: X-Plane's own
    /// fundamental position dataref (`weather_truth.rs` reads the same one
    /// for `XPLMGetWeatherAtLocation`'s altitude argument).
    elevation_m: Option<DataRef>,
    /// `sim/flightmodel/failures/onground_any`, the same dataref
    /// `physics/adirs.rs:1522` and `physics/damage.rs:432` read and
    /// `lib.rs`'s `"SIM ON GROUND"` mapping is built on.
    on_ground: Option<DataRef>,
}

/// A published name resolved to the variable it writes.
///
/// Primed in [`DeepLayer::new`] from [`Deep::published_names`], so the
/// frame loop never allocates and never calls `Vars::get` (which would
/// allocate the name and its `A32NX_` prefix again, every frame, for every
/// published value).
///
/// Two levels. `order` is the sequence of names the priming pass saw:
/// areas publish the same names in the same order every frame, so the
/// n-th call of a frame is the n-th entry, and confirming that is one
/// length-then-bytes string comparison -- measured at 2.4 ns per published
/// value against 12.6 ns for hashing the name, over the 2445 values ten
/// areas publish. `by_name` is the fallback for anything that does not
/// line up (an area that publishes conditionally, or a name the priming
/// pass never saw), and is what keeps the fast path safe to take: a
/// mismatch costs a hash lookup, never a wrong variable.
#[derive(Default)]
struct Publisher {
    order: Vec<(String, VariableIdentifier)>,
    by_name: HashMap<String, VariableIdentifier>,
}

impl Publisher {
    /// The variable `name` writes, and whether the positional cache is
    /// still in step (so the caller can advance it).
    fn resolve(&mut self, vars: &mut Vars, at: usize, name: &str) -> (VariableIdentifier, bool) {
        if let Some((cached, id)) = self.order.get(at) {
            if cached == name {
                return (*id, true);
            }
        }
        if let Some(id) = self.by_name.get(name) {
            return (*id, false);
        }
        // A name neither cache has seen: resolve it once, then never
        // again. Only reachable on the first frame an area publishes it.
        let id = vars.get(name.to_owned());
        self.by_name.insert(name.to_owned(), id);
        (id, false)
    }
}

/// The deep layer as the plugin owns it.
pub struct DeepLayer {
    deep: Deep,
    weather: WeatherTruthReader,
    ids: Ids,
    refs: Refs,
    /// Every failure id `deep::registry()` assigned, built once. Sorted,
    /// so intersecting the (usually tiny) armed set with it is a handful
    /// of binary searches.
    failure_ids: BTreeSet<u64>,
    publisher: Publisher,
    /// The last full weather read, and the time since it was taken.
    ///
    /// `WeatherTruthReader::read` calls `XPLMGetWeatherAtLocation`, which
    /// `XPLMWeather.h` itself says is "not intended to be used per-frame"
    /// (`wxr/sampler.rs` quotes the same line and budgets its own calls for
    /// the same reason). It is taken at [`WEATHER_INTERVAL_S`] and held in
    /// between, so a 30-60 Hz frame loop makes that call ten times a second
    /// rather than sixty. Everything it reads -- air temperature, ambient
    /// pressure, true airspeed, precipitation, cloud layers -- changes far
    /// more slowly than 0.1 s: the fastest of them, TAS in a take-off
    /// acceleration, moves under a tenth of a knot in that time.
    environment: EnvironmentTruth,
    since_weather_s: f64,
}

/// How often the weather/atmosphere read above is taken. See
/// `DeepLayer::environment`.
pub const WEATHER_INTERVAL_S: f64 = 0.1;

impl DeepLayer {
    pub fn new(vars: &mut Vars, xplm: Option<&Xplm>) -> Self {
        let deep = crate::deep::live::all_areas();
        let mut publisher = Publisher::default();
        for name in deep.published_names() {
            let id = vars.get(name.clone());
            publisher.by_name.insert(name.clone(), id);
            publisher.order.push((name, id));
        }
        let failure_ids = crate::deep::registry().failures.iter().map(|f| f.id).collect();
        let ids = Ids {
            engines: [EngineIds::new(vars, 1), EngineIds::new(vars, 2), EngineIds::new(vars, 3), EngineIds::new(vars, 4)],
            apu_available: vars.get("OVHD_APU_START_PB_IS_AVAILABLE".to_owned()),
            apu_bleed_air_pressure: vars.get("APU_BLEED_AIR_PRESSURE".to_owned()),
            ac_bus_potential: [1, 2, 3, 4].map(|n| vars.get(format!("ELEC_AC_{n}_BUS_POTENTIAL"))),
            dc_bus_potential: [1, 2].map(|n| vars.get(format!("ELEC_DC_{n}_BUS_POTENTIAL"))),
            hydraulic_pressure_psi: ["GREEN", "YELLOW"].map(|c| vars.get(format!("HYD_{c}_SYSTEM_1_SECTION_PRESSURE"))),
        };
        let refs = Refs {
            elevation_m: xplm.and_then(|x| x.find("sim/flightmodel/position/elevation")),
            on_ground: xplm.and_then(|x| x.find("sim/flightmodel/failures/onground_any")),
        };
        Self {
            deep,
            weather: WeatherTruthReader::new(vars, xplm),
            ids,
            refs,
            failure_ids,
            publisher,
            environment: Truth::default().environment,
            // Due immediately, so the first frame reads for real rather
            // than handing the areas the default atmosphere.
            since_weather_s: f64::MAX,
        }
    }

    /// The areas this layer was built with, for the startup log.
    pub fn area_names(&self) -> Vec<&'static str> {
        self.deep.area_names()
    }

    /// This frame's armed deep failures.
    ///
    /// One `crate::failures` lock, then the armed ids that belong to the
    /// deep catalogue. See this module's "Frame cost" note for why this is
    /// exactly `armed_magnitude(id)` over every deep id, without being
    /// thousands of locks.
    fn faults(&self) -> Faults {
        Faults::from_pairs(
            crate::failures::active_magnitudes().into_iter().filter(|(id, _)| self.failure_ids.contains(id)),
        )
    }

    /// Everything the areas read about the rest of the simulation, this
    /// frame. See this module's table for each field's source.
    fn truth(&mut self, vars: &mut Vars, xplm: Option<&Xplm>, delta: f64) -> Truth {
        let default = Truth::default();
        let dt_s = clamp_dt(delta);
        let f = |d: Option<DataRef>| d.and_then(|d| xplm.map(|x| x.get_f(d) as f64));
        self.since_weather_s = (self.since_weather_s + dt_s).min(f64::MAX);
        if self.since_weather_s >= WEATHER_INTERVAL_S {
            self.since_weather_s = 0.0;
            self.environment = self.weather.read(vars, xplm);
            // A missing barometer dataref reads 0.0, which is a vacuum,
            // not a measurement: several areas divide by ambient pressure,
            // so fall back to what `Truth::default()` documents instead.
            if !(self.environment.ambient_pressure_pa > 0.0) {
                self.environment.ambient_pressure_pa = default.environment.ambient_pressure_pa;
            }
        }
        let environment = self.environment;

        let mut engine_n1_frac = [0.0; 4];
        let mut engine_running = [false; 4];
        let mut engine_bleed_pressure_pa = default.engine_bleed_pressure_pa;
        let mut engine_bleed_temp_k = default.engine_bleed_temp_k;
        for (i, e) in self.ids.engines.iter().enumerate() {
            engine_n1_frac[i] = vars.read(&e.n1_pct) / 100.0;
            engine_running[i] = vars.read(&e.state) == ENGINE_STATE_ON;
            // `engine_commands.rs:466`: the IP port feeds the customer
            // bleed unless the HP valve is open.
            let from_ip = vars.read(&e.hp_valve_open) == 0.0;
            let (pressure, temp) = if from_ip {
                (vars.read(&e.ip_port_pressure_pa), vars.read(&e.ip_port_temp_k))
            } else {
                (vars.read(&e.hp_port_pressure_pa), vars.read(&e.hp_port_temp_k))
            };
            // Before `engine_commands` has written a frame these read 0:
            // an absolute pressure of zero and a temperature of absolute
            // zero are both impossible, so keep the documented cold-engine
            // default rather than hand an area a number no gas can have.
            if pressure > 0.0 {
                engine_bleed_pressure_pa[i] = pressure;
            }
            if temp > 0.0 {
                engine_bleed_temp_k[i] = temp;
            }
        }

        let (apu_bleed_psi, apu_bleed_ssm) = unpack_arinc(vars.read(&self.ids.apu_bleed_air_pressure));
        // The ECB only vouches for its word while it is powered; with the
        // APU cold there is no bleed and the port sits at ambient, which
        // is a reading this frame already has.
        let apu_bleed_pressure_pa = if apu_bleed_ssm == SSM_NORMAL_OPERATION && apu_bleed_psi > 0.0 {
            apu_bleed_psi * PSI_TO_PA
        } else {
            environment.ambient_pressure_pa
        };

        Truth {
            dt_s,
            altitude_ft: f(self.refs.elevation_m).map_or(default.altitude_ft, |m| m * crate::M_TO_FT),
            on_ground: self
                .refs
                .on_ground
                .and_then(|d| xplm.map(|x| x.get_i(d) != 0))
                .unwrap_or(default.on_ground),
            environment,
            engine_n1_frac,
            engine_running,
            engine_bleed_pressure_pa,
            engine_bleed_temp_k,
            apu_running: vars.read(&self.ids.apu_available) != 0.0,
            apu_bleed_pressure_pa,
            ac_bus_volts: std::array::from_fn(|i| vars.read(&self.ids.ac_bus_potential[i])),
            dc_bus_volts: std::array::from_fn(|i| vars.read(&self.ids.dc_bus_potential[i])),
            hydraulic_pressure_pa: std::array::from_fn(|i| vars.read(&self.ids.hydraulic_pressure_psi[i]) * PSI_TO_PA),
        }
    }

    /// Step every area and publish what they expose, once per frame.
    pub fn tick(&mut self, vars: &mut Vars, xplm: Option<&Xplm>, delta: f64) {
        let truth = self.truth(vars, xplm, delta);
        let faults = self.faults();
        let Self { deep, publisher, .. } = self;
        let mut at = 0usize;
        deep.tick(truth, &faults, &mut |name, value| {
            let (id, in_step) = publisher.resolve(vars, at, name);
            at += in_step as usize;
            vars.write(&id, value);
        });
    }
}

/// This frame's length as the models may be stepped with it. A `dt` that
/// is not a finite positive number at all (X-Plane's zero on the frame a
/// flight loads) becomes [`MIN_DT_S`], not zero: an area that divides by
/// `dt_s` must never see one it cannot divide by.
pub fn clamp_dt(delta: f64) -> f64 {
    if delta.is_finite() {
        delta.clamp(MIN_DT_S, MAX_DT_S)
    } else {
        MIN_DT_S
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_frame_time_is_never_zero_never_huge_and_never_nan() {
        // The two real cases: X-Plane's zero dt on the frame a flight
        // loads, and the whole wall-clock gap after a pause.
        assert_eq!(clamp_dt(0.0), MIN_DT_S);
        assert_eq!(clamp_dt(12.0), MAX_DT_S);
        assert_eq!(clamp_dt(-1.0), MIN_DT_S);
        assert_eq!(clamp_dt(f64::NAN), MIN_DT_S);
        assert_eq!(clamp_dt(f64::INFINITY), MIN_DT_S);
        assert_eq!(clamp_dt(1.0 / 60.0), 1.0 / 60.0);
    }

    #[test]
    fn an_arinc_word_unpacks_the_way_flybywire_packs_it() {
        // FlyByWire's `to_arinc429`: the f32's bits, the status two bits
        // above. A running APU's 50 psi with the ECB vouching for it.
        let packed = (((SSM_NORMAL_OPERATION as u64) << 32) | 50.0f32.to_bits() as u64) as f64;
        let (value, ssm) = unpack_arinc(packed);
        assert_eq!(ssm, SSM_NORMAL_OPERATION);
        assert!((value - 50.0).abs() < 1e-6, "{value}");
        // A word nobody wrote is not a pressure of zero psi with a good
        // status; it is failure-warning, which `truth` falls back from.
        assert_eq!(unpack_arinc(0.0), (0.0, 0));
    }

    /// A `Vars` off a do-nothing X-Plane binding, the same way
    /// `fadec.rs`'s own tests build one: `find` answers "no such dataref",
    /// so every `Refs` entry is `None` and every `Truth` field that has no
    /// variable written under it falls back to its documented default --
    /// which is exactly what these tests are checking.
    fn rig() -> (&'static Xplm, Vars) {
        let xplm: &'static Xplm = Box::leak(Box::new(Xplm::dummy()));
        (xplm, Vars::new(xplm))
    }

    #[test]
    fn a_truth_with_nothing_written_is_the_documented_default_not_zero() {
        let (xplm, mut vars) = rig();
        let mut layer = DeepLayer::new(&mut vars, Some(xplm));
        let t = layer.truth(&mut vars, Some(xplm), 0.0);
        let d = Truth::default();
        // A vacuum, absolute zero and a dead engine's bleed port at 0 Pa
        // are all numbers no area may be handed just because a dataref or
        // a variable has not been written yet.
        assert_eq!(t.environment.ambient_pressure_pa, d.environment.ambient_pressure_pa);
        assert_eq!(t.engine_bleed_pressure_pa, d.engine_bleed_pressure_pa);
        assert_eq!(t.engine_bleed_temp_k, d.engine_bleed_temp_k);
        assert_eq!(t.apu_bleed_pressure_pa, d.environment.ambient_pressure_pa);
        assert_eq!(t.altitude_ft, d.altitude_ft);
        assert_eq!(t.on_ground, d.on_ground);
        assert_eq!(t.dt_s, MIN_DT_S, "X-Plane's zero dt on the frame a flight loads");
        // These genuinely are zero on a cold aircraft, and FlyByWire
        // publishes them as zero, so zero is the reading, not a gap.
        assert_eq!(t.engine_n1_frac, [0.0; 4]);
        assert_eq!(t.engine_running, [false; 4]);
        assert!(!t.apu_running);
        assert_eq!(t.ac_bus_volts, [0.0; 4]);
        assert_eq!(t.hydraulic_pressure_pa, [0.0; 2]);
    }

    #[test]
    fn the_weather_read_is_taken_on_the_first_frame_and_then_at_its_own_interval() {
        // X-Plane's own header: XPLMGetWeatherAtLocation is "not intended
        // to be used per-frame". The first frame must still read for real
        // rather than hand the areas the default atmosphere.
        let (xplm, mut vars) = rig();
        let mut layer = DeepLayer::new(&mut vars, Some(xplm));
        let sat = vars.get("AMBIENT TEMPERATURE".to_owned());
        vars.write(&sat, -40.0);
        assert_eq!(layer.truth(&mut vars, Some(xplm), 1.0 / 60.0).environment.sat_c, -40.0);
        // Inside the interval the held reading stands, unchanged.
        vars.write(&sat, 20.0);
        let held = ((WEATHER_INTERVAL_S * 60.0) as usize).saturating_sub(1);
        for _ in 0..held {
            assert_eq!(layer.truth(&mut vars, Some(xplm), 1.0 / 60.0).environment.sat_c, -40.0, "a held reading must not change inside the interval");
        }
        // And past it the next read lands, within a frame or two of the
        // interval (exactly which frame is float accumulation, and not
        // something worth pinning a test to).
        let mut fresh = None;
        for _ in 0..3 {
            fresh = Some(layer.truth(&mut vars, Some(xplm), 1.0 / 60.0).environment.sat_c);
            if fresh == Some(20.0) {
                break;
            }
        }
        assert_eq!(fresh, Some(20.0), "the reading must be taken again once the interval has passed");
    }

    #[test]
    fn truth_reads_the_port_the_engine_is_actually_bled_from() {
        // `engine_commands.rs` bleeds the IP port unless the HP valve is
        // open; reading the other one would hand the pneumatic areas a
        // pressure the engine is not actually delivering.
        let (xplm, mut vars) = rig();
        let mut layer = DeepLayer::new(&mut vars, Some(xplm));
        for n in 1..=4 {
            let ip_p = vars.get(format!("ENGINE_IP_PORT_PRESSURE_PA:{n}"));
            let ip_t = vars.get(format!("ENGINE_IP_PORT_TEMP_K:{n}"));
            let hp_p = vars.get(format!("ENGINE_HP_PORT_PRESSURE_PA:{n}"));
            let hp_t = vars.get(format!("ENGINE_HP_PORT_TEMP_K:{n}"));
            vars.write(&ip_p, 300_000.0);
            vars.write(&ip_t, 500.0);
            vars.write(&hp_p, 900_000.0);
            vars.write(&hp_t, 700.0);
        }
        let hp_valve_2 = vars.get("PNEU_ENG_2_HP_VALVE_OPEN".to_owned());
        vars.write(&hp_valve_2, 1.0);
        let t = layer.truth(&mut vars, Some(xplm), 1.0 / 60.0);
        assert_eq!(t.engine_bleed_pressure_pa, [300_000.0, 900_000.0, 300_000.0, 300_000.0]);
        assert_eq!(t.engine_bleed_temp_k, [500.0, 700.0, 500.0, 500.0]);
    }

    #[test]
    fn flybywires_own_psi_and_arinc_readings_arrive_in_si() {
        let (xplm, mut vars) = rig();
        let mut layer = DeepLayer::new(&mut vars, Some(xplm));
        let green = vars.get("HYD_GREEN_SYSTEM_1_SECTION_PRESSURE".to_owned());
        let yellow = vars.get("HYD_YELLOW_SYSTEM_1_SECTION_PRESSURE".to_owned());
        vars.write(&green, 5000.0);
        vars.write(&yellow, 0.0);
        let apu_p = vars.get("APU_BLEED_AIR_PRESSURE".to_owned());
        vars.write(&apu_p, (((SSM_NORMAL_OPERATION as u64) << 32) | 50.0f32.to_bits() as u64) as f64);
        let apu_avail = vars.get("OVHD_APU_START_PB_IS_AVAILABLE".to_owned());
        vars.write(&apu_avail, 1.0);
        let t = layer.truth(&mut vars, Some(xplm), 1.0 / 60.0);
        // The A380's 5000 psi systems, as FlyByWire publishes them.
        assert!((t.hydraulic_pressure_pa[0] - 5000.0 * PSI_TO_PA).abs() < 1.0, "{:?}", t.hydraulic_pressure_pa);
        assert_eq!(t.hydraulic_pressure_pa[1], 0.0);
        assert!(t.apu_running);
        assert!((t.apu_bleed_pressure_pa - 50.0 * PSI_TO_PA).abs() < 1.0, "{}", t.apu_bleed_pressure_pa);
    }

    #[test]
    fn a_published_name_resolves_to_the_same_variable_in_order_or_out_of_it() {
        // The positional cache is only an optimisation: whichever path a
        // name takes, it must reach the same variable, or an area's
        // published value would land on the wrong one.
        let (_xplm, mut vars) = rig();
        let mut p = Publisher::default();
        for name in ["DEEP_TEST_A", "DEEP_TEST_B", "DEEP_TEST_C"] {
            let id = vars.get(name.to_owned());
            p.by_name.insert(name.to_owned(), id);
            p.order.push((name.to_owned(), id));
        }
        let expected: Vec<_> = p.order.iter().map(|(_, id)| *id).collect();
        // In order: every call takes the fast path and advances.
        let mut at = 0;
        for (i, name) in ["DEEP_TEST_A", "DEEP_TEST_B", "DEEP_TEST_C"].iter().enumerate() {
            let (id, in_step) = p.resolve(&mut vars, at, name);
            assert!(in_step);
            assert_eq!(id, expected[i]);
            at += 1;
        }
        // Out of order, and a name no priming pass saw: still correct.
        let (id, in_step) = p.resolve(&mut vars, 0, "DEEP_TEST_C");
        assert!(!in_step);
        assert_eq!(id, expected[2]);
        let (new_id, in_step) = p.resolve(&mut vars, 0, "DEEP_TEST_NEW");
        assert!(!in_step);
        assert_eq!(new_id, vars.get("DEEP_TEST_NEW".to_owned()), "a name seen late must resolve to the same variable");
        assert!(!expected.contains(&new_id));
    }

    #[test]
    fn the_deep_failure_catalogue_is_a_set_of_unique_ids() {
        // `faults()` looks an armed id up in this set, so a duplicate id
        // would silently give one failure two meanings.
        let failures = crate::deep::registry().failures;
        let unique: BTreeSet<u64> = failures.iter().map(|f| f.id).collect();
        assert_eq!(unique.len(), failures.len(), "{} of {} ids are duplicates", failures.len() - unique.len(), failures.len());
    }
}
