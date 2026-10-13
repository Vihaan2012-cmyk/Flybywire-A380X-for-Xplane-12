//! Plugin-side circuit simulation for physics workstream 2 (electrical),
//! `docs/briefs/hyperrealism.md`.
//!
//! **Audit finding.** FlyByWire's own electrical crate
//! (`fbw-common/.../electrical/`, `a380_systems/.../electrical/`) is, as the
//! brief warned, a topology/potential graph, not a solved circuit: buses are
//! grouped into equipotential sets by which closed [`Contactor`]s connect
//! them to a source (`Electricity`/`Potential`, `electrical/mod.rs`), which
//! *is* the real graph-connectivity half of Kirchhoff's laws for
//! near-zero-impedance busbars and contactors, and is kept unmodified here.
//! What it did *not* do, before this workstream's patch
//! (`patches/fbw-rust/electrical.patch`), was solve any actual current or
//! voltage drop: every generator/TRU/battery/external-power source wrote a
//! flat nameplate voltage (115 V AC / 28 V DC) regardless of load, batteries
//! used an admittedly "fake" 0.15 ohm/10 A placeholder
//! (`battery.rs`'s old `calculate_charging_current` comment), and nothing
//! fed electrical load back onto the engines as mechanical drag.
//!
//! **What the FBW-side patch adds** (real, sourced physics; parameter
//! sources and equations are tabulated in `docs/physics/electrical.md`):
//! - VFGs/APU generators: a Kirchhoff loop across an equivalent synchronous
//!   reactance (`V^2 - V_rated*V + S*Xs = 0`, the same quadratic-in-V
//!   technique `transformer_rectifier.rs` already used for its own output
//!   impedance), so terminal voltage sags with real current and self-limits
//!   at the reactance's own maximum power transfer under overload, plus a
//!   real electrical-to-mechanical efficiency for shaft power.
//! - Batteries: the cell's own previously-unused 0.011 ohm internal
//!   resistance plus a derived wiring resistance, a temperature-dependent
//!   resistance and a first-order I^2R/convective thermal model, and a
//!   max-power-transfer-limited discharge instead of an arbitrary current
//!   cap.
//! - External power: the same equivalent-source-impedance treatment, sized
//!   to a stiffer, better-regulated ground cart.
//! - Every [`ElectricalBus`] now also publishes its real, load-sagged
//!   terminal voltage (`ELEC_<bus>_BUS_POTENTIAL`), not only the previous
//!   powered/normal booleans.
//!
//! **What this module adds, plugin-side** (things with no FlyByWire
//! equivalent to extend at all):
//! 1. [`EngineLoads`]: closes the shared engine-load contract
//!    (`ENGINE_GEARBOX_ELEC_LOAD_W:n`/`:0`) by reading the shaft power the
//!    FBW-side patch's generators now expose as plain simulator variables
//!    and republishing the contract name, the same pattern
//!    `physics::hydraulics::Hydraulics` already uses for the hydraulic term.
//! 2. [`CircuitProtection`]: FBW's crate has no circuit breakers or SSPCs at
//!    all (the brief's stage-3 foundation). This estimates each
//!    `circuits.rs` circuit's real current from a cited, sensible A380/
//!    generic-transport consumer load assignment and the bus's own
//!    Kirchhoff-solved voltage, and applies a real I^2t thermal trip plus an
//!    instant magnetic trip (the two curves the A380's SSPCs and any
//!    remaining thermal-magnetic breakers both implement in different
//!    proportions), tripping `circuits.rs`'s own breaker state so the Study
//!    CB page shows it and `fuel.rs`/`lights.rs` (which already gate on
//!    `Circuits::powered`) actually lose that consumer.

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::circuits::Circuits;
use crate::Vars;

/// Closes the shared engine-load contract
/// (docs/briefs/hyperrealism.md: `ENGINE_GEARBOX_ELEC_LOAD_W:n`, "generator
/// shaft power ... electrical load divided by generator efficiency") by
/// summing FlyByWire's own generators' real shaft-power demand, which the
/// FBW-side patch now writes as a plain simulator variable per generator
/// (`ELEC_ENG_GEN_n_SHAFT_POWER_DEMAND`, `ELEC_APU_GEN_n_SHAFT_POWER_DEMAND`)
/// instead of a Rust API across the plugin/systems crate boundary -- the
/// same pattern `physics::hydraulics::Hydraulics` uses for the hydraulic
/// term.
pub struct EngineLoads {
    engines: [(VariableIdentifier, VariableIdentifier); 4],
    /// APU generators 1 and 2 both drive the one shared APU gearbox
    /// (`alternating_current.rs`'s `ACBusPowerSource::APUGenerator(1|2)`),
    /// so both are summed onto the shared contract's `n = 0` APU slot.
    apu_gens: [VariableIdentifier; 2],
    apu_load: VariableIdentifier,
}
impl EngineLoads {
    pub fn new(vars: &mut Vars) -> Self {
        let engines = std::array::from_fn(|i| {
            let n = i + 1;
            (
                vars.get(format!("ELEC_ENG_GEN_{n}_SHAFT_POWER_DEMAND")),
                vars.get(format!("ENGINE_GEARBOX_ELEC_LOAD_W:{n}")),
            )
        });
        let apu_gens = std::array::from_fn(|i| {
            let n = i + 1;
            vars.get(format!("ELEC_APU_GEN_{n}_SHAFT_POWER_DEMAND"))
        });
        Self {
            engines,
            apu_gens,
            apu_load: vars.get("ENGINE_GEARBOX_ELEC_LOAD_W:0".to_string()),
        }
    }

    /// Reads the tick's shaft-power demands FlyByWire's generators just
    /// wrote (during `Simulation::tick`'s `report_electricity_consumption`)
    /// and republishes the contract names. Call after the systems tick.
    pub fn update(&self, vars: &mut Vars) {
        for (shaft_power, load) in &self.engines {
            let power = vars.read(shaft_power);
            vars.write(load, power);
        }
        let apu_total: f64 = self.apu_gens.iter().map(|id| vars.read(id)).sum();
        vars.write(&self.apu_load, apu_total);
    }
}

/// One `circuits.rs` circuit's real load assignment: a nominal power draw
/// whenever the circuit is live (`Circuits::powered`), used with the bus's
/// own Kirchhoff-solved voltage to get a real current for the breaker model
/// below. FBW's own per-consumer wattage is not this granular (its own
/// `power_consumption.rs` models load in per-bus aggregate, "the watts in
/// this function are all provided by komp", not per physical circuit), so
/// these are generic large-transport-aircraft equipment ratings, publicly
/// typical rather than FBW- or type-certificate-sourced; each is cited and
/// marked as a derived/typical estimate in docs/physics/electrical.md.
/// Falls back to a generic small-avionics-box rating for any circuit type
/// not explicitly listed (most of the remaining `CIRCUIT_*` types are
/// exactly that: one LRU on a dedicated breaker).
fn rated_watts(type_name: &str) -> f64 {
    match type_name {
        "CIRCUIT_FUEL_PUMP" => 600.,
        "CIRCUIT_FUEL_VALVE" => 50.,
        "CIRCUIT_LIGHT_LANDING" => 600.,
        "CIRCUIT_LIGHT_TAXI" => 250.,
        "CIRCUIT_LIGHT_NAV" => 40.,
        "CIRCUIT_LIGHT_BEACON" => 100.,
        "CIRCUIT_LIGHT_STROBE" => 300.,
        "CIRCUIT_LIGHT_LOGO" => 150.,
        "CIRCUIT_LIGHT_WING" => 150.,
        "CIRCUIT_LIGHT_RECOGNITION" => 40.,
        "CIRCUIT_LIGHT_CABIN" => 200.,
        "CIRCUIT_LIGHT_PANEL" => 30.,
        "CIRCUIT_LIGHT_PEDESTAL" => 20.,
        "CIRCUIT_LIGHT_GLARESHIELD" => 20.,
        "CIRCUIT_GEAR_MOTOR" => 1500.,
        "CIRCUIT_GEAR_WARNING" => 20.,
        "CIRCUIT_PITOT_HEAT" => 600.,
        "CIRCUIT_STARTER" | "CIRCUIT_APU_STARTER" => 2000.,
        "CIRCUIT_STANDBY_VACUUM" => 100.,
        _ => 50.,
    }
}

/// The nominal bus voltage a `circuits.rs` MSFS bus number's real current
/// should be computed against: FlyByWire's AC buses are 115 V (three-phase
/// equivalent, matching `EngineGenerator::RATED_VOLTAGE_VOLT`), its DC buses
/// 28 V (`TransformerRectifier`/`Battery`). Falls back to 115 V (bus 1,
/// INFINIBAT, is DC in the real aircraft, but is always-powered and not fed
/// through any of this workstream's Kirchhoff sources, so its exact nominal
/// voltage does not affect any trip decision).
///
/// `pub` (not just `pub(crate)`): `breakers.rs`'s wider circuit-breaker
/// catalogue (docs/analysis/cockpit-study-cbs.md CB-002/STUDY-001) reuses
/// this exact function for its own MSFS-bus-backed breakers, rather than
/// duplicating the AC/DC voltage split.
pub fn nominal_bus_voltage(msfs_bus: u32) -> f64 {
    match msfs_bus {
        2..=7 | 16 => 115.,
        _ => 28.,
    }
}

/// Which curve tripped: [`Protection`] and `breakers.rs` both write this to
/// their own `CIRCUIT TRIP CAUSE:n` (1 = thermal, 2 = magnetic).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TripCause {
    Thermal,
    Magnetic,
}

/// Time-to-trip at 2x rated current, in seconds: `K / (r^2 - 1)` with
/// `r = 2` gives `K / 3`, so `K = 3 * 10 = 30` trips in ~10 s at 2x.
const THERMAL_TRIP_K: f64 = 30.;
/// Instant ("magnetic") trip threshold: a dead short is many times rated
/// current; 10x is a typical magnetic-element pickup multiple for aircraft
/// breakers/SSPCs (derived/typical, electrical.md).
const MAGNETIC_TRIP_MULTIPLE: f64 = 10.;
/// Cools at the same order of time constant a real thermal element
/// dissipates its stored heat between trips.
const COOLDOWN_SECONDS: f64 = 20.;

/// Reference ambient a thermal circuit breaker's I^2t curve above is rated
/// at (`THERMAL_TRIP_K`/`MAGNETIC_TRIP_MULTIPLE` both implicitly assume
/// this): 25 degC, the standard ambient reference aircraft/industrial
/// thermal-magnetic breaker datasheets publish their trip-time curve at
/// (e.g. Eaton/TE Connectivity aerospace CB derating charts always quote a
/// 25 degC baseline curve plus a separate derating-vs-ambient table) --
/// derived/typical, no A380-specific breaker datasheet is public, but the
/// *shape* (hotter ambient -> less thermal margin before trip, same curve)
/// is the real, cited physical behaviour every thermal breaker has: its
/// bimetal element trips at a fixed absolute temperature, not a fixed
/// temperature *rise*, so less ambient headroom means less current rise
/// needed to reach it.
pub const REFERENCE_AMBIENT_C: f64 = 25.;
/// Typical bimetal-element temperature rise (above ambient) at the moment
/// of trip for an aerospace thermal-magnetic breaker of this class
/// (derived/typical -- no public A380 figure). Used only to convert a bay
/// ambient temperature into a fraction of the trip budget already
/// "pre-loaded" by that ambient, so `trip_step_with_ambient` reduces to
/// exactly `trip_step` at `REFERENCE_AMBIENT_C`.
const ELEMENT_RISE_AT_TRIP_C: f64 = 75.;

/// The one I^2t thermal / instant magnetic trip curve in the plugin
/// (`docs/physics/electrical.md` section 6), shared by every breaker
/// model: [`Protection`] below (the 154 `circuits.rs` `systems.cfg`
/// circuits) and `breakers.rs`'s own wider catalogue. `heat` is the
/// caller's own normalised I^2t accumulator (trips at 1.0, persisted
/// between calls); `ratio` is this tick's `current / rated_current`.
/// Returns `Some` the tick it trips (the caller opens its own breaker and
/// records the cause); `heat` resets to 0 on a trip, matching a real
/// bimetal element's stored heat dissipating once the breaker opens and
/// current stops. Equivalent to `trip_step_with_ambient` at
/// [`REFERENCE_AMBIENT_C`] -- kept as the zero-ambient-effect entry point
/// so every existing caller (and its tests) is unaffected.
pub fn trip_step(heat: &mut f64, ratio: f64, delta: f64) -> Option<TripCause> {
    trip_step_with_ambient(heat, ratio, delta, REFERENCE_AMBIENT_C)
}

/// [`trip_step`], but the breaker's own bay ambient temperature shifts the
/// I^2t trip decision physically rather than just adding a fudge factor:
/// a bimetal thermal element trips once *its own* temperature reaches a
/// fixed absolute value, so a bay that starts hotter than
/// [`REFERENCE_AMBIENT_C`] has already used up some of that margin before
/// any load current is drawn at all (`ambient_bias`, a fraction of
/// [`ELEMENT_RISE_AT_TRIP_C`]), and -- because the element sheds heat to
/// the bay air by the same delta-T that drives it toward trip -- a hot bay
/// also cools a healthy element's residual heat *slower* once the overload
/// clears (`cooldown_scale`). At `ambient_c == REFERENCE_AMBIENT_C` both
/// terms are neutral and this is bit-for-bit `trip_step`.
pub fn trip_step_with_ambient(heat: &mut f64, ratio: f64, delta: f64, ambient_c: f64) -> Option<TripCause> {
    // Magnetic (instant, short-circuit) trip elements are current-driven
    // only -- a real magnetic pickup is a solenoid plunger, not a bimetal
    // strip, so it is not meaningfully temperature-derated the way the
    // thermal curve is; ambient never enters this branch.
    if ratio >= MAGNETIC_TRIP_MULTIPLE {
        *heat = 0.;
        return Some(TripCause::Magnetic);
    }

    // A bimetal element trips at a fixed *absolute* temperature, and its
    // steady-state temperature rise above ambient scales with (I/I_rated)^2
    // (ohmic self-heating). Folding the ambient into the same `ratio^2`
    // term the original curve already used keeps this bit-for-bit equal to
    // `trip_step` at `REFERENCE_AMBIENT_C` (`ambient_term` is 0 there), and
    // lets a *sub-rated* load (ratio < 1, which alone never heats the
    // element in this curve) still cross the trip threshold once the bay
    // itself is hot enough -- exactly "a normal load trips in a hot bay".
    let ambient_term = (ambient_c - REFERENCE_AMBIENT_C) / ELEMENT_RISE_AT_TRIP_C;
    let thermal_input = ratio * ratio + ambient_term;

    if thermal_input > 1. {
        *heat += delta * (thermal_input - 1.) / THERMAL_TRIP_K;
        if *heat >= 1. {
            *heat = 0.;
            return Some(TripCause::Thermal);
        }
    } else {
        // The same delta-T that drives the element toward trip is also what
        // it sheds to the bay air, so a hotter bay (smaller delta-T to the
        // trip point) cools the element's residual heat slower too; floored
        // at 10% of the reference cooldown rate so a pathologically hot bay
        // still cools eventually rather than latching `heat` forever.
        let cooldown_scale = (1. - ambient_term).max(0.1);
        *heat = (*heat - delta / (COOLDOWN_SECONDS / cooldown_scale)).max(0.);
    }
    None
}

/// One circuit's thermal-magnetic/SSPC protection state.
struct Protection {
    number: usize,
    /// This circuit's own real load, in watts: `CircuitDef::rated_w` (the
    /// systems.cfg file's own literal `Power:` field) when the line has
    /// one, else the generic `rated_watts(type_name)` typical/derived
    /// fallback. Real current is derived from this divided by the *live*
    /// bus voltage every tick (`update`), not baked into a fixed current at
    /// nominal voltage -- a sagging bus really draws more current to hold
    /// the same power, the same physical relationship FlyByWire's own
    /// generators/TRUs already model on the source side.
    rated_watts: f64,
    /// The rated current *at nominal bus voltage* -- the breaker's own
    /// rating, used only as the trip curve's `ratio` denominator (a
    /// breaker is rated in amps at nominal voltage, not in watts), not as
    /// the current actually applied to the thermal/magnetic curve.
    rated_current: f64,
    /// Normalised I^2t thermal accumulator: trips at 1.0. Real SSPC/
    /// thermal-magnetic breaker curves are inverse-time in (I/I_rated)^2;
    /// `THERMAL_TRIP_K` is chosen (docs/physics/electrical.md) so 2x rated
    /// current trips in about 10 s and severe overloads trip in well under
    /// a second, a typical shape for aerospace SSPC application notes (no
    /// FBW source, so marked derived).
    heat: f64,
    current_id: VariableIdentifier,
    trip_cause_id: VariableIdentifier,
    fault_id: VariableIdentifier,
}
impl Protection {
    fn new<V: VariableRegistry + SimulatorReaderWriter>(vars: &mut V, number: usize, type_name: &str, buses: &[u32], rated_w: Option<f64>) -> Self {
        let voltage = buses.first().map_or(115., |&b| nominal_bus_voltage(b));
        let watts = rated_w.unwrap_or_else(|| rated_watts(type_name));
        Self {
            number,
            rated_watts: watts,
            rated_current: watts / voltage,
            heat: 0.,
            current_id: vars.get(format!("CIRCUIT CURRENT:{number}")),
            trip_cause_id: vars.get(format!("CIRCUIT TRIP CAUSE:{number}")),
            fault_id: vars.get(format!("CIRCUIT FAULT CURRENT MULTIPLE:{number}")),
        }
    }

    /// Estimates this tick's current, updates the thermal state, and trips
    /// the breaker (via `circuits.set_breaker`) on either curve. Returns the
    /// current for the Study panel.
    ///
    /// `CIRCUIT FAULT CURRENT MULTIPLE:n` is a test/failure-injection input:
    /// 0 (the default) draws the consumer's own rated current when live: a
    /// short is a failure mode where the load's own impedance collapses, so
    /// a value above 1 stands in for a wiring short or an internal fault
    /// drawing that multiple of rated current, limited in reality only by
    /// upstream source/wiring impedance.
    fn update<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V, circuits: &mut Circuits, delta: f64, voltage: f64) -> f64 {
        if !circuits.breaker_closed(vars, self.number) {
            // Open (tripped or manually pulled): no current, and the
            // thermal state resets so a manual reset (or the next tick,
            // once the fault clears) starts fresh, the way a real breaker's
            // bimetal element cools while open.
            self.heat = 0.;
            vars.write(&self.current_id, 0.);
            return 0.;
        }

        let live = circuits.powered(vars, self.number);
        let fault_multiple = vars.read(&self.fault_id);
        // Real current: this circuit's own real power draw (its systems.cfg
        // `Power:` field while it is actually on, `rated_watts`) divided by
        // the *real*, Kirchhoff-solved bus voltage this tick (`voltage`,
        // already load-sagged by the FBW-side patch, `docs/physics/
        // electrical.md`), not the old fixed `rated_current` computed once
        // at nominal voltage -- a sagging bus makes a constant-power load
        // draw genuinely more current, the same relationship every real
        // motor/resistive load has. `fault_multiple` (test/failure
        // injection only) still scales on top, unchanged.
        let current = if !live {
            0.
        } else if voltage <= 0. {
            0.
        } else {
            (self.rated_watts / voltage) * fault_multiple.max(1.)
        };
        vars.write(&self.current_id, current);

        let ratio = if self.rated_current > 0. { current / self.rated_current } else { 0. };
        match trip_step(&mut self.heat, ratio, delta) {
            Some(TripCause::Magnetic) => self.trip(vars, circuits, 2.),
            Some(TripCause::Thermal) => self.trip(vars, circuits, 1.),
            None => {}
        }

        current
    }

    fn trip<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V, circuits: &mut Circuits, cause: f64) {
        circuits.set_breaker(vars, self.number, false);
        vars.write(&self.trip_cause_id, cause);
        self.heat = 0.;
    }
}

/// Real breaker/SSPC current and trip physics for every `circuits.rs`
/// circuit (the "circuit protection" stage-3 foundation the brief asks for
/// -- FBW's own crate has no breaker model at all).
pub struct CircuitProtection {
    protections: Vec<Protection>,
    /// Cached per-circuit bus (first bus only; almost every circuit has
    /// exactly one), looked up once at construction.
    bus_voltage_ids: Vec<Option<VariableIdentifier>>,
    bus_of: Vec<Option<u32>>,
}
impl CircuitProtection {
    pub fn new<V: VariableRegistry + SimulatorReaderWriter>(vars: &mut V, circuits: &Circuits) -> Self {
        let mut protections = Vec::new();
        let mut bus_voltage_ids = Vec::new();
        let mut bus_of = Vec::new();
        for c in circuits.list() {
            protections.push(Protection::new(vars, c.number, &c.type_name, &c.buses, c.rated_w));
            let bus = c.buses.first().copied();
            bus_voltage_ids.push(bus.and_then(|b| {
                crate::circuits::bus_power_variable(b)
                    .map(|name| name.replace("_IS_POWERED", "_POTENTIAL"))
                    .map(|name| vars.get(name))
            }));
            bus_of.push(bus);
        }
        Self { protections, bus_voltage_ids, bus_of }
    }

    /// Call after the systems tick (so `ELEC_*_BUS_POTENTIAL` and
    /// `Circuits`'s own bus-power booleans are this tick's) and before
    /// `fuel.rs`/`lights.rs`, so a trip this tick already cuts their
    /// consumer this same tick.
    pub fn update<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V, circuits: &mut Circuits, delta: f64) {
        for (i, protection) in self.protections.iter_mut().enumerate() {
            let voltage = match self.bus_voltage_ids[i] {
                Some(id) => vars.read(&id),
                // bus.1 (INFINIBAT) has no FlyByWire bus/potential of its
                // own; treat it as the aircraft's always-on hot battery bus
                // voltage for current-estimation purposes.
                None => self.bus_of[i].map_or(28., nominal_bus_voltage),
            };
            protection.update(vars, circuits, delta, voltage.abs());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aspects::test_vars::TestVars;

    fn circuits_with_recognition() -> (TestVars, Circuits) {
        let mut vars = TestVars::default();
        let circuits = Circuits::new(&mut vars);
        (vars, circuits)
    }

    #[test]
    fn rated_watts_covers_the_main_consumer_types() {
        assert_eq!(rated_watts("CIRCUIT_FUEL_PUMP"), 600.);
        assert_eq!(rated_watts("CIRCUIT_LIGHT_LANDING"), 600.);
        // Unknown/avionics-box types fall back to a sensible default rather
        // than zero (which would mean "never trips").
        assert_gt(rated_watts("CIRCUIT_XPNDR"), 0.);
    }

    fn assert_gt(a: f64, b: f64) {
        assert!(a > b, "{a} should be greater than {b}");
    }

    #[test]
    fn nominal_voltage_matches_ac_and_dc_buses() {
        assert_eq!(nominal_bus_voltage(2), 115.); // AC_BUS_1
        assert_eq!(nominal_bus_voltage(8), 28.); // DC_BUS_1
    }

    #[test]
    fn a_normal_load_never_trips() {
        let (mut vars, mut circuits) = circuits_with_recognition();
        let recognition = circuits.list().into_iter().find(|c| c.type_name == "CIRCUIT_LIGHT_RECOGNITION").unwrap();
        let mut protection = CircuitProtection::new(&mut vars, &circuits);

        for _ in 0..600 {
            protection.update(&mut vars, &mut circuits, 1.0);
        }

        assert!(circuits.breaker_closed(&mut vars, recognition.number));
    }

    /// Drives the recognition-light circuit at `multiple` times its rated
    /// current every tick until it trips (or `max_seconds` elapses),
    /// returning the elapsed simulated time and the trip cause (1 =
    /// thermal, 2 = magnetic) `CircuitProtection` recorded.
    fn time_to_trip(multiple: f64, max_seconds: f64) -> Option<(f64, f64)> {
        let (mut vars, mut circuits) = circuits_with_recognition();
        let recognition = circuits.list().into_iter().find(|c| c.type_name == "CIRCUIT_LIGHT_RECOGNITION").unwrap();
        let mut protection = CircuitProtection::new(&mut vars, &circuits);
        let fault_id = vars.get(format!("CIRCUIT FAULT CURRENT MULTIPLE:{}", recognition.number));
        vars.write(&fault_id, multiple);
        let cause_id = vars.get(format!("CIRCUIT TRIP CAUSE:{}", recognition.number));

        const DT: f64 = 1.0 / 60.0;
        let mut elapsed = 0.;
        while elapsed < max_seconds {
            protection.update(&mut vars, &mut circuits, DT);
            elapsed += DT;
            if !circuits.breaker_closed(&mut vars, recognition.number) {
                return Some((elapsed, vars.read(&cause_id)));
            }
        }
        None
    }

    #[test]
    fn a_short_circuit_fault_trips_instantly_on_the_magnetic_curve() {
        // A dead short (well above the 10x magnetic threshold) trips within
        // one tick, not merely "eventually".
        let (elapsed, cause) = time_to_trip(20., 1.0).expect("expected an instant trip");

        assert!(elapsed <= 1.0 / 60.0 + 1e-9);
        assert_eq!(cause, 2.); // magnetic
    }

    #[test]
    fn a_moderate_overload_trips_on_the_thermal_curve_near_its_predicted_time() {
        // At 2x rated current, THERMAL_TRIP_K's own construction predicts a
        // trip at K/(2^2-1) = 30/3 = 10 s: the real "breaker trip time
        // versus the curve" check the brief asks for.
        let (elapsed, cause) = time_to_trip(2., 30.).expect("expected a trip within 30 s");

        assert!((elapsed - 10.0).abs() < 0.2, "expected ~10s, got {elapsed}s");
        assert_eq!(cause, 1.); // thermal
    }

    #[test]
    fn a_heavier_overload_trips_faster_than_a_lighter_one() {
        // The inverse-time shape: doubling the overload multiple shortens
        // the thermal trip time, matching a real breaker/SSPC curve rather
        // than a fixed timer.
        let (at_1_5x, _) = time_to_trip(1.5, 60.).expect("1.5x should eventually trip");
        let (at_3x, _) = time_to_trip(3., 60.).expect("3x should eventually trip");

        assert!(at_3x < at_1_5x, "3x ({at_3x}s) should trip faster than 1.5x ({at_1_5x}s)");
    }

    #[test]
    fn a_reset_breaker_can_carry_load_again_once_the_fault_clears() {
        let (mut vars, mut circuits) = circuits_with_recognition();
        let recognition = circuits.list().into_iter().find(|c| c.type_name == "CIRCUIT_LIGHT_RECOGNITION").unwrap();
        let mut protection = CircuitProtection::new(&mut vars, &circuits);

        let fault_id = vars.get(format!("CIRCUIT FAULT CURRENT MULTIPLE:{}", recognition.number));
        vars.write(&fault_id, 20.);
        protection.update(&mut vars, &mut circuits, 1.0 / 60.0);
        assert!(!circuits.breaker_closed(&mut vars, recognition.number));

        vars.write(&fault_id, 0.);
        circuits.set_breaker(&mut vars, recognition.number, true);
        protection.update(&mut vars, &mut circuits, 1.0 / 60.0);

        assert!(circuits.breaker_closed(&mut vars, recognition.number));
    }
}
