//! The live hydraulic system: one instance of the A380's green and yellow
//! circuits, stepped every frame from [`Truth`] with every failure this
//! directory's `registry.rs` registers applied to the exact model field
//! that entry names.
//!
//! Everything physical is already in `topology.rs` (`A380Hydraulics` ->
//! two `Circuit`s, each with four engine-driven pumps, a reservoir, an
//! accumulator, priority/relief valves, a return filter and five consumer
//! branches; yellow additionally has the electric motor pump). This module
//! adds nothing to the physics. It does exactly three things:
//!
//! 1. **Owns** the instance, so something in the running plugin finally
//!    holds a `Circuit` rather than only the type.
//! 2. **Translates** `Truth` into `topology::CircuitInputs` -- engine HP
//!    spool speed through the accessory gearbox into pump shaft rpm, bus
//!    voltage into the electric pump's supply, bleed/electrical
//!    availability into the reservoir's bootstrap pressurisation, static
//!    air temperature into the bay temperature the thermal model sinks to.
//! 3. **Binds** every registered failure id to its model field, by reading
//!    the ids straight out of `registry::register`'s own output rather
//!    than re-deriving the numbering here -- so a failure added, removed
//!    or reordered in `registry.rs` can never silently stop being wired.
//!
//! ## Inputs that are not on `Truth` yet
//!
//! Four inputs the model genuinely needs have no field on [`Truth`]. None
//! of them is invented here; each has a setter the plugin calls, and until
//! it does they sit at a value that is *honest about not knowing* rather
//! than a plausible-looking constant:
//!
//! * **HP spool speed** ([`HydraulicsLive::set_engine_n3_frac`]). The EDP
//!   is driven off the HP (N3) spool through the accessory gearbox, and
//!   `Truth` carries only N1. Until the plugin supplies N3 (it already
//!   publishes `ENGINE_N3:n` from this crate's own engine model), the
//!   shaft speed is interpolated between this aircraft's own two cited
//!   operating points -- see [`n3_frac_from_n1_frac`].
//! * **Fire handle position** ([`HydraulicsLive::set_fire_handles`]).
//! * **Consumer flow demand** ([`HydraulicsLive::set_demands`]) -- the
//!   flow the flight controls, gear, brakes, steering, cargo doors and
//!   reversers are actually drawing. `deep::flight_controls` knows its own
//!   half of this, but the [`Area`] trait is tick-then-publish with no
//!   channel between areas, so it has to arrive through the plugin.
//! * **Heat-exchanger fuel flow** ([`HydraulicsLive::set_fuel`]). With no
//!   fuel flow the only cooling path left is the passive bay loss, so the
//!   circuit runs hotter than it should; it is deliberately left that way
//!   rather than guessing an engine feed flow.

use std::collections::BTreeMap;

use crate::deep::live::{Area as LiveArea, Faults, Truth};

use super::accumulator::AccumulatorFaults;
use super::network::{CheckValveFaults, PSI_PA};
use super::pump::PumpFaults;
use super::reservoir::ReservoirFaults;
use super::topology::{A380Hydraulics, CircuitFaults, CircuitInputs, CircuitOutputs, ConsumerDemands, EdpFaults, EdpInputs};

// ---------------------------------------------------------------------------
// Constants (all cited; none of them a stand-in for physics).
// ---------------------------------------------------------------------------

/// HP spool speed at 100%: `engines.cfg`'s own header comment "HP - Real N3
/// - Sim N2 - 12,200RPM", already restated as
/// `physics::engine::params::N3_DESIGN_RPM` in this crate; repeated here
/// rather than imported so this directory stays self-contained per
/// `docs/deep/BRIEF.md` hard rule 2.
const N3_DESIGN_RPM: f64 = 12_200.0;
/// Accessory-gearbox ratio from HP spool to EDP drive shaft, FlyByWire's
/// own `TrentEngine::PUMP_N3_GEAR_RATIO`
/// (`fbw-common/src/wasm/systems/systems/src/engine/trent_engine.rs:40`),
/// giving the EDP's 3775 rpm rated speed at 100% N3.
const PUMP_N3_GEAR_RATIO: f64 = 0.31;
/// `engines.cfg` `[TURBINEENGINEDATA] low_idle_n1`/`low_idle_n2`, restated
/// from `physics::engine::params::{IDLE_N1_PCT, IDLE_N3_PCT}`: ground idle
/// is 15% N1 and 60% N3 on this aircraft.
const IDLE_N1_PCT: f64 = 15.0;
const IDLE_N3_PCT: f64 = 60.0;

/// The reservoir's own regulated bootstrap air pressure,
/// `reservoir.rs`'s `NOMINAL_BOOST_PA` (50 psi gauge, FlyByWire's
/// `hydraulic/mod.rs:2311`). A pneumatic supply below this cannot hold the
/// regulator's setting, so the fraction it can deliver is its own gauge
/// pressure against this figure.
const RESERVOIR_REGULATED_BOOST_PA: f64 = 50.0 * PSI_PA;

/// A 115 V AC bus is energised. Half nominal is far below anything a
/// contactor would hold in and far above sensor noise on a dead bus.
const AC_BUS_LIVE_V: f64 = 100.0;
/// Likewise for a 28 V DC bus.
const DC_BUS_LIVE_V: f64 = 20.0;

/// The HP spool fraction implied by an LP spool fraction, for as long as
/// [`Truth`] carries only N1.
///
/// **This is a stand-in, not physics**: the real relationship between the
/// two spools is set by the engine's own working line, which this crate's
/// `physics::engine` does model but `Truth` does not expose. It is a
/// straight-line interpolation through this aircraft's own two cited
/// operating points -- ground idle (15% N1 / 60% N3) and take-off (100% /
/// 100%) -- extended to the origin below idle, chosen because those are
/// the only two points on the curve that *are* public. It exists so the
/// live circuit runs before `Truth` grows an `engine_n3_frac`, and
/// [`HydraulicsLive::set_engine_n3_frac`] bypasses it entirely the moment
/// the real value is available.
pub fn n3_frac_from_n1_frac(n1_frac: f64) -> f64 {
    let n1_pct = n1_frac.clamp(0.0, 1.2) * 100.0;
    let n3_pct = if n1_pct <= IDLE_N1_PCT {
        n1_pct / IDLE_N1_PCT * IDLE_N3_PCT
    } else {
        IDLE_N3_PCT + (n1_pct - IDLE_N1_PCT) * (100.0 - IDLE_N3_PCT) / (100.0 - IDLE_N1_PCT)
    };
    n3_pct / 100.0
}

// ---------------------------------------------------------------------------
// Failure-id binding.
// ---------------------------------------------------------------------------

/// Every failure id this area registers, indexed by the component it lives
/// on, in the order `registry.rs` registered them (which is the order
/// `ComponentDef::failures` holds).
fn registered_failures() -> BTreeMap<String, Vec<u64>> {
    let mut r = crate::deep::api::Registry::default();
    super::registry::register(&mut r);
    r.components.into_iter().map(|c| (c.id, c.failures)).collect()
}

/// Panics only if `registry.rs` and this file have drifted, which the unit
/// tests below catch at build time rather than in the simulator.
fn take(map: &BTreeMap<String, Vec<u64>>, component: &str, count: usize) -> Vec<u64> {
    let ids = map.get(component).unwrap_or_else(|| panic!("hydraulics registry has no component {component}"));
    assert_eq!(ids.len(), count, "component {component} registers {} failures, live.rs binds {count}", ids.len());
    ids.clone()
}

/// The five failures each engine-driven pump registers, in registration
/// order (`registry.rs`: displacement loss, seizure, check valve stuck
/// open, check valve stuck shut, fire shutoff valve stuck).
#[derive(Clone, Copy, Debug)]
struct EdpFailureIds {
    displacement_loss: u64,
    seizure: u64,
    check_stuck_open: u64,
    check_stuck_shut: u64,
    fire_sov_stuck: u64,
}

/// One circuit's complete failure-id set.
#[derive(Clone, Debug)]
struct CircuitFailureIds {
    /// Same order as `topology::CircuitFaults::edp`: engine A pump a, A
    /// pump b, engine B pump a, engine B pump b.
    edp: [EdpFailureIds; 4],
    reservoir_leak: u64,
    reservoir_pressurization_loss: u64,
    air_ingestion: u64,
    accumulator_precharge_loss: u64,
    priority_valve_stuck: u64,
    relief_valve_crack_low: u64,
    filter_clog: u64,
    /// `[gear, brakes, steering, cargo_doors, reversers]`, matching
    /// `CircuitFaults::line_leak_area_m2`.
    line_leaks: [u64; 5],
}

impl CircuitFailureIds {
    fn build(map: &BTreeMap<String, Vec<u64>>, color: &str, engines: [u16; 2]) -> Self {
        let mut edp = Vec::new();
        for engine in engines {
            for half in ['a', 'b'] {
                let ids = take(map, &format!("29_hyd.{color}_edp_{engine}{half}"), 5);
                edp.push(EdpFailureIds {
                    displacement_loss: ids[0],
                    seizure: ids[1],
                    check_stuck_open: ids[2],
                    check_stuck_shut: ids[3],
                    fire_sov_stuck: ids[4],
                });
            }
        }
        let reservoir = take(map, &format!("29_hyd.{color}_reservoir"), 3);
        let line_leaks = ["gear", "brakes", "steering", "cargo_doors", "reversers"]
            .map(|branch| take(map, &format!("29_hyd.{color}_line_{branch}"), 1)[0]);
        Self {
            edp: [edp[0], edp[1], edp[2], edp[3]],
            reservoir_leak: reservoir[0],
            reservoir_pressurization_loss: reservoir[1],
            air_ingestion: reservoir[2],
            accumulator_precharge_loss: take(map, &format!("29_hyd.{color}_accumulator"), 1)[0],
            priority_valve_stuck: take(map, &format!("29_hyd.{color}_priority_valve"), 1)[0],
            relief_valve_crack_low: take(map, &format!("29_hyd.{color}_relief_valve"), 1)[0],
            filter_clog: take(map, &format!("29_hyd.{color}_return_filter"), 1)[0],
            line_leaks,
        }
    }
}

/// The registered magnitude meaning of every leak failure in this area:
/// "leak orifice area, 0..20 mm^2".
const MAX_LEAK_AREA_M2: f64 = 20.0e-6;

// ---------------------------------------------------------------------------
// The live system.
// ---------------------------------------------------------------------------

/// Which physical engine drives each of a circuit's four pumps, as
/// `topology.rs` documents (green from engines 1 and 2, yellow from 3 and
/// 4; each engine drives two pumps, a and b). Zero-based here because
/// `Truth`'s per-engine arrays are.
const GREEN_PUMP_ENGINE_INDEX: [usize; 4] = [0, 0, 1, 1];
const YELLOW_PUMP_ENGINE_INDEX: [usize; 4] = [2, 2, 3, 3];

pub struct HydraulicsLive {
    hyd: A380Hydraulics,
    green_ids: CircuitFailureIds,
    yellow_ids: CircuitFailureIds,
    /// `[displacement loss, seizure]` for the yellow electric pump.
    electric_pump_ids: [u64; 2],
    green_out: CircuitOutputs,
    yellow_out: CircuitOutputs,

    // Inputs `Truth` does not carry yet (see module doc).
    engine_n3_frac: Option<[f64; 4]>,
    fire_handle_pulled: [bool; 4],
    green_demands: ConsumerDemands,
    yellow_demands: ConsumerDemands,
    fuel_kg_s: f64,
    fuel_temp_k: f64,
}

impl Default for HydraulicsLive {
    fn default() -> Self {
        Self::new()
    }
}

impl HydraulicsLive {
    pub fn new() -> Self {
        let map = registered_failures();
        let electric = take(&map, "29_hyd.yellow_electric_pump", 2);
        Self {
            hyd: A380Hydraulics::new(),
            green_ids: CircuitFailureIds::build(&map, "green", [1, 2]),
            yellow_ids: CircuitFailureIds::build(&map, "yellow", [3, 4]),
            electric_pump_ids: [electric[0], electric[1]],
            green_out: CircuitOutputs::default(),
            yellow_out: CircuitOutputs::default(),
            engine_n3_frac: None,
            fire_handle_pulled: [false; 4],
            green_demands: ConsumerDemands::default(),
            yellow_demands: ConsumerDemands::default(),
            fuel_kg_s: 0.0,
            fuel_temp_k: 288.15,
        }
    }

    /// The real HP spool fraction per engine, once the plugin has it. It
    /// already publishes `ENGINE_N3:n` (percent) from this crate's own
    /// engine model; pass it as a fraction.
    pub fn set_engine_n3_frac(&mut self, n3_frac: [f64; 4]) {
        self.engine_n3_frac = Some(n3_frac);
    }

    /// Per engine, whether its firewall FIRE handle has been pulled (which
    /// shuts that engine's two pumps' firewall shutoff valves).
    pub fn set_fire_handles(&mut self, pulled: [bool; 4]) {
        self.fire_handle_pulled = pulled;
    }

    /// This tick's consumer flow demand on each circuit, m^3/s.
    pub fn set_demands(&mut self, green: ConsumerDemands, yellow: ConsumerDemands) {
        self.green_demands = green;
        self.yellow_demands = yellow;
    }

    /// Engine feed fuel available to the hydraulic/fuel heat exchangers.
    pub fn set_fuel(&mut self, fuel_kg_s: f64, fuel_temp_k: f64) {
        self.fuel_kg_s = fuel_kg_s.max(0.0);
        self.fuel_temp_k = fuel_temp_k;
    }

    pub fn green(&self) -> &CircuitOutputs {
        &self.green_out
    }

    pub fn yellow(&self) -> &CircuitOutputs {
        &self.yellow_out
    }

    /// Green and yellow manifold pressures, Pa -- the same two numbers the
    /// plugin puts back on `Truth::hydraulic_pressure_pa` for the areas
    /// that consume hydraulic power (`deep::flight_controls` above all).
    pub fn manifold_pressures_pa(&self) -> [f64; 2] {
        [self.green_out.manifold_pressure_pa, self.yellow_out.manifold_pressure_pa]
    }

    fn n3_frac(&self, truth: &Truth) -> [f64; 4] {
        match self.engine_n3_frac {
            Some(n3) => n3,
            None => std::array::from_fn(|i| n3_frac_from_n1_frac(truth.engine_n1_frac[i])),
        }
    }

    fn edp_inputs(&self, truth: &Truth, engine_index: [usize; 4]) -> [EdpInputs; 4] {
        let n3 = self.n3_frac(truth);
        std::array::from_fn(|pump| {
            let e = engine_index[pump];
            // A pump only turns while its engine's core is actually
            // rotating: a windmilling-but-unlit core still drives its
            // pumps, which is why this keys off N3 rather than off
            // `engine_running`, and why a shut-down engine's residual N3
            // still delivers a little flow as it runs down.
            let shaft_rpm = (n3[e].max(0.0) * N3_DESIGN_RPM * PUMP_N3_GEAR_RATIO).max(0.0);
            EdpInputs { shaft_rpm, fire_handle_pulled: self.fire_handle_pulled[e] }
        })
    }

    /// The bootstrap air supply available to a reservoir's pressurising
    /// valve, 0..1. `reservoir.rs`'s own doc defines this as the
    /// cabin/bleed-air supply, "normally 1.0 with electrical/pneumatic
    /// power up", so it is the better of what the two sources can give:
    /// a live AC or DC bus (cabin air conditioning running) delivers the
    /// regulator's full setting, and otherwise a bleed source delivers
    /// whatever fraction of the 50 psi setting its own gauge pressure
    /// covers.
    fn pressurization_supply_fraction(truth: &Truth) -> f64 {
        let electrical = truth.ac_bus_volts.iter().any(|&v| v > AC_BUS_LIVE_V) || truth.dc_bus_volts.iter().any(|&v| v > DC_BUS_LIVE_V);
        if electrical {
            return 1.0;
        }
        let best_bleed_pa = truth
            .engine_bleed_pressure_pa
            .iter()
            .copied()
            .fold(truth.apu_bleed_pressure_pa, f64::max);
        let gauge_pa = best_bleed_pa - truth.environment.ambient_pressure_pa;
        (gauge_pa / RESERVOIR_REGULATED_BOOST_PA).clamp(0.0, 1.0)
    }

    fn circuit_faults(ids: &CircuitFailureIds, faults: &Faults, electric_pump: PumpFaults) -> CircuitFaults {
        CircuitFaults {
            edp: std::array::from_fn(|i| {
                let id = ids.edp[i];
                EdpFaults {
                    pump: PumpFaults {
                        // `wear` is a persisted component health parameter,
                        // not an armable failure (registry.rs's own
                        // health-parameter/failure split), so nothing in
                        // `Faults` drives it.
                        wear: 0.0,
                        displacement_loss: faults.get(id.displacement_loss),
                        seizure: faults.get(id.seizure),
                    },
                    check_valve: CheckValveFaults {
                        stuck_open: faults.get(id.check_stuck_open),
                        stuck_shut: faults.get(id.check_stuck_shut),
                    },
                    fire_sov_stuck: faults.get(id.fire_sov_stuck),
                }
            }),
            electric_pump,
            reservoir: ReservoirFaults {
                leak_area_m2: faults.get(ids.reservoir_leak) * MAX_LEAK_AREA_M2,
                pressurization_loss: faults.get(ids.reservoir_pressurization_loss),
            },
            accumulator: AccumulatorFaults { precharge_loss: faults.get(ids.accumulator_precharge_loss) },
            priority_valve_stuck: faults.get(ids.priority_valve_stuck),
            relief_valve_crack_low: faults.get(ids.relief_valve_crack_low),
            filter_clog: faults.get(ids.filter_clog),
            line_leak_area_m2: ids.line_leaks.map(|id| faults.get(id) * MAX_LEAK_AREA_M2),
            air_ingestion: faults.get(ids.air_ingestion),
        }
    }
}

/// Circuit-level published variables, per colour.
fn publish_circuit(out: &mut dyn FnMut(&str, f64), color: &str, c: &CircuitOutputs, pumps: [&str; 4], electric: bool) {
    // The four names `registry.rs` cites in its ECAM triggers.
    out(&format!("HYD_{color}_MANIFOLD_PRESSURE_PSI"), c.manifold_pressure_pa / PSI_PA);
    out(&format!("HYD_{color}_RESERVOIR_LEVEL_IS_LOW"), f64::from(u8::from(c.reservoir_low_level_warning)));
    out(&format!("HYD_{color}_RESERVOIR_AIR_PRESSURE_IS_LOW"), f64::from(u8::from(c.reservoir_low_pressure_warning)));
    out(&format!("HYD_{color}_RESERVOIR_OVHT"), f64::from(u8::from(c.fluid_overheat)));

    // Study-page detail: the state behind each of those four indications.
    out(&format!("HYD_{color}_ESSENTIAL_PRESSURE_PSI"), c.essential_pressure_pa / PSI_PA);
    out(&format!("HYD_{color}_ACCUMULATOR_PRESSURE_PSI"), c.accumulator_pressure_pa / PSI_PA);
    out(&format!("HYD_{color}_RESERVOIR_LEVEL_FRACTION"), c.reservoir_fill_fraction);
    out(&format!("HYD_{color}_FLUID_TEMP_C"), c.fluid_temp_c);
    const M3_S_TO_L_MIN: f64 = 60_000.0;
    for (i, name) in pumps.iter().enumerate() {
        out(&format!("HYD_{color}_EDP_{name}_FLOW_L_MIN"), c.edp[i].flow_m3_s * M3_S_TO_L_MIN);
        out(&format!("HYD_{color}_EDP_{name}_CASE_DRAIN_L_MIN"), c.edp[i].case_drain_m3_s * M3_S_TO_L_MIN);
        out(&format!("HYD_{color}_EDP_{name}_VOLUMETRIC_EFFICIENCY"), c.edp[i].volumetric_efficiency);
    }
    if electric {
        out(&format!("HYD_{color}_ELEC_PUMP_FLOW_L_MIN"), c.electric_pump_flow_m3_s * M3_S_TO_L_MIN);
    }
}

impl LiveArea for HydraulicsLive {
    fn name(&self) -> &'static str {
        "hydraulics"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        let dt = truth.dt_s.max(0.0);
        let ambient_k = truth.environment.sat_c + 273.15;
        let pressurization = Self::pressurization_supply_fraction(truth);

        let green_inputs = CircuitInputs {
            edp: self.edp_inputs(truth, GREEN_PUMP_ENGINE_INDEX),
            // `topology.rs`: no green electric pump exists in this model.
            electric_pump_powered: false,
            electric_pump_bus_voltage_v: 0.0,
            demands: self.green_demands,
            fuel_kg_s: self.fuel_kg_s,
            fuel_temp_k: self.fuel_temp_k,
            ambient_k,
            pressurization_supply_fraction: pressurization,
        };

        // The yellow electric motor pump stands in for FlyByWire's own
        // yellow pump pair, supplied from AC bus 3 (pump a) and AC bus 4
        // (pump b) with DC 1 control power
        // (`a380_systems/src/hydraulic/mod.rs:1769,1777-1779`): it runs
        // while either supply bus is live and its control bus is up.
        let supply_v = truth.ac_bus_volts[2].max(truth.ac_bus_volts[3]);
        let control_up = truth.dc_bus_volts[0] > DC_BUS_LIVE_V;
        let yellow_inputs = CircuitInputs {
            edp: self.edp_inputs(truth, YELLOW_PUMP_ENGINE_INDEX),
            electric_pump_powered: control_up && supply_v > AC_BUS_LIVE_V,
            electric_pump_bus_voltage_v: supply_v,
            demands: self.yellow_demands,
            fuel_kg_s: self.fuel_kg_s,
            fuel_temp_k: self.fuel_temp_k,
            ambient_k,
            pressurization_supply_fraction: pressurization,
        };

        let green_faults = Self::circuit_faults(&self.green_ids, faults, PumpFaults::default());
        let yellow_faults = Self::circuit_faults(
            &self.yellow_ids,
            faults,
            PumpFaults {
                wear: 0.0,
                displacement_loss: faults.get(self.electric_pump_ids[0]),
                seizure: faults.get(self.electric_pump_ids[1]),
            },
        );

        self.green_out = self.hyd.green.step(&green_inputs, &green_faults, dt);
        self.yellow_out = self.hyd.yellow.step(&yellow_inputs, &yellow_faults, dt);
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        publish_circuit(out, "GREEN", &self.green_out, ["1A", "1B", "2A", "2B"], false);
        publish_circuit(out, "YELLOW", &self.yellow_out, ["3A", "3B", "4A", "4B"], true);
    }
}

/// This area's live system, for `deep::live::Deep::with_area`.
pub fn live_system() -> Box<dyn LiveArea> {
    Box::new(HydraulicsLive::new())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::api::Registry;

    /// Four engines at take-off power, both AC and DC buses live.
    fn running_truth() -> Truth {
        Truth {
            dt_s: 0.02,
            engine_n1_frac: [1.0; 4],
            engine_running: [true; 4],
            ac_bus_volts: [115.0; 4],
            dc_bus_volts: [28.0; 2],
            ..Truth::default()
        }
    }

    fn run(area: &mut HydraulicsLive, truth: &Truth, faults: &Faults, seconds: f64) -> BTreeMap<String, f64> {
        let ticks = (seconds / truth.dt_s).round() as usize;
        for _ in 0..ticks {
            area.tick(truth, faults);
        }
        let mut published = BTreeMap::new();
        area.publish(&mut |name, value| {
            published.insert(name.to_string(), value);
        });
        published
    }

    #[test]
    fn every_registered_failure_id_is_bound_to_a_model_field() {
        // `registered_failures` is the single source of ids, so the only
        // way to drift is for a component to gain or lose a failure. That
        // is exactly what `take`'s count assertion catches, and building
        // the live system runs every one of them.
        let live = HydraulicsLive::new();
        let mut r = Registry::default();
        super::super::registry::register(&mut r);

        let mut bound: Vec<u64> = Vec::new();
        for ids in [&live.green_ids, &live.yellow_ids] {
            for e in ids.edp {
                bound.extend([e.displacement_loss, e.seizure, e.check_stuck_open, e.check_stuck_shut, e.fire_sov_stuck]);
            }
            bound.extend([
                ids.reservoir_leak,
                ids.reservoir_pressurization_loss,
                ids.air_ingestion,
                ids.accumulator_precharge_loss,
                ids.priority_valve_stuck,
                ids.relief_valve_crack_low,
                ids.filter_clog,
            ]);
            bound.extend(ids.line_leaks);
        }
        bound.extend(live.electric_pump_ids);
        bound.sort_unstable();

        let mut registered: Vec<u64> = r.failures.iter().map(|f| f.id).collect();
        registered.sort_unstable();
        assert_eq!(bound, registered, "every registered hydraulics failure must reach a model field");
    }

    #[test]
    fn a_healthy_running_aircraft_publishes_a_real_service_pressure_and_no_cautions() {
        let mut live = HydraulicsLive::new();
        let published = run(&mut live, &running_truth(), &Faults::default(), 60.0);
        for color in ["GREEN", "YELLOW"] {
            let psi = published[&format!("HYD_{color}_MANIFOLD_PRESSURE_PSI")];
            assert!(psi > 2900.0, "{color} should be above the SYS LO PR threshold: {psi:.0} psi");
            assert!(psi < 6000.0, "{color} relief valve should cap it: {psi:.0} psi");
            assert_eq!(published[&format!("HYD_{color}_RESERVOIR_LEVEL_IS_LOW")], 0.0);
            assert_eq!(published[&format!("HYD_{color}_RESERVOIR_AIR_PRESSURE_IS_LOW")], 0.0);
            assert_eq!(published[&format!("HYD_{color}_RESERVOIR_OVHT")], 0.0);
        }
    }

    #[test]
    fn seizing_every_green_engine_driven_pump_drops_the_published_pressure_below_the_ecam_threshold() {
        // The registered effect of `green EDP n seizure` is "zero flow and
        // zero case drain from this pump"; with all four seized the green
        // circuit has no source at all, so `HYD_GREEN_SYS_LO_PR`'s own
        // trigger variable must fall through 2900 psi. Yellow, untouched,
        // must not move -- the two circuits are independent.
        let live_ids = HydraulicsLive::new();
        let armed: Vec<(u64, f64)> = live_ids.green_ids.edp.iter().map(|e| (e.seizure, 1.0)).collect();
        let faults = Faults::from_pairs(armed);

        let mut healthy = HydraulicsLive::new();
        let mut seized = HydraulicsLive::new();
        let truth = running_truth();
        let healthy_out = run(&mut healthy, &truth, &Faults::default(), 60.0);
        let seized_out = run(&mut seized, &truth, &faults, 60.0);

        assert!(healthy_out["HYD_GREEN_MANIFOLD_PRESSURE_PSI"] > 2900.0);
        assert!(
            seized_out["HYD_GREEN_MANIFOLD_PRESSURE_PSI"] < 2900.0,
            "four seized green EDPs must trip HYD GREEN SYS LO PR: {:.0} psi",
            seized_out["HYD_GREEN_MANIFOLD_PRESSURE_PSI"]
        );
        assert!(seized_out["HYD_GREEN_EDP_1A_FLOW_L_MIN"].abs() < 1e-9);
        assert!(seized_out["HYD_YELLOW_MANIFOLD_PRESSURE_PSI"] > 2900.0, "yellow is a separate circuit and must be unaffected");
    }

    #[test]
    fn a_reservoir_leak_drains_the_published_level_and_raises_the_low_level_indication() {
        // Registered effect: "reservoir fluid quantity falls over time; low
        // level eventually unports the pump inlets".
        let ids = HydraulicsLive::new().green_ids.reservoir_leak;
        let faults = Faults::from_pairs([(ids, 1.0)]);
        let truth = running_truth();

        let mut healthy = HydraulicsLive::new();
        let mut leaking = HydraulicsLive::new();
        let healthy_out = run(&mut healthy, &truth, &Faults::default(), 120.0);
        let leaking_out = run(&mut leaking, &truth, &faults, 120.0);

        let healthy_level = healthy_out["HYD_GREEN_RESERVOIR_LEVEL_FRACTION"];
        let leaking_level = leaking_out["HYD_GREEN_RESERVOIR_LEVEL_FRACTION"];
        assert!(leaking_level < healthy_level, "a 20 mm^2 leak must cost fluid: {leaking_level} vs {healthy_level}");
        assert_eq!(leaking_out["HYD_GREEN_RESERVOIR_LEVEL_IS_LOW"], 1.0, "and eventually raise HYD G RSVR LEVEL LO");
    }

    #[test]
    fn losing_reservoir_pressurisation_raises_the_air_pressure_caution_and_cavitates_the_pumps() {
        // Registered effect: "pump inlet gauge pressure collapses toward
        // zero, cavitating every pump on this circuit even with a full
        // reservoir".
        let ids = HydraulicsLive::new().yellow_ids.reservoir_pressurization_loss;
        let faults = Faults::from_pairs([(ids, 1.0)]);
        let truth = running_truth();
        let out = run(&mut HydraulicsLive::new(), &truth, &faults, 60.0);
        assert_eq!(out["HYD_YELLOW_RESERVOIR_AIR_PRESSURE_IS_LOW"], 1.0);
        assert!(
            out["HYD_YELLOW_MANIFOLD_PRESSURE_PSI"] < 2900.0,
            "cavitating pumps cannot hold service pressure: {:.0} psi",
            out["HYD_YELLOW_MANIFOLD_PRESSURE_PSI"]
        );
    }

    #[test]
    fn the_yellow_electric_pump_needs_its_own_supply_and_control_buses() {
        // AC 3/4 supply, DC 1 control (a380_systems/src/hydraulic/mod.rs).
        let mut on_batteries_only = Truth { dt_s: 0.02, dc_bus_volts: [28.0, 28.0], ..Truth::default() };
        on_batteries_only.ac_bus_volts = [0.0; 4];
        let out = run(&mut HydraulicsLive::new(), &on_batteries_only, &Faults::default(), 30.0);
        assert!(out["HYD_YELLOW_ELEC_PUMP_FLOW_L_MIN"].abs() < 1e-9, "no AC supply, no pump");

        let ac4_only = Truth { dt_s: 0.02, ac_bus_volts: [0.0, 0.0, 0.0, 115.0], dc_bus_volts: [28.0, 28.0], ..Truth::default() };
        let out = run(&mut HydraulicsLive::new(), &ac4_only, &Faults::default(), 30.0);
        assert!(out["HYD_YELLOW_ELEC_PUMP_FLOW_L_MIN"] > 0.0, "AC 4 alone still runs it");
        assert!(out["HYD_YELLOW_MANIFOLD_PRESSURE_PSI"] > 100.0);
    }

    #[test]
    fn the_hp_spool_stand_in_reproduces_both_cited_operating_points() {
        assert!((n3_frac_from_n1_frac(0.15) - 0.60).abs() < 1e-12, "ground idle: 15% N1 is 60% N3");
        assert!((n3_frac_from_n1_frac(1.0) - 1.0).abs() < 1e-12, "take-off: 100% N1 is 100% N3");
        assert_eq!(n3_frac_from_n1_frac(0.0), 0.0);
        assert!(n3_frac_from_n1_frac(0.5) > n3_frac_from_n1_frac(0.3), "monotonic");
    }

    #[test]
    fn a_supplied_hp_spool_speed_overrides_the_stand_in_entirely() {
        let mut live = HydraulicsLive::new();
        // N1 says take-off, N3 says the engines are stopped: the real
        // value must win, so the pumps must not turn.
        live.set_engine_n3_frac([0.0; 4]);
        let out = run(&mut live, &running_truth(), &Faults::default(), 30.0);
        assert!(out["HYD_GREEN_EDP_1A_FLOW_L_MIN"].abs() < 1e-9);
        assert!(out["HYD_GREEN_MANIFOLD_PRESSURE_PSI"] < 100.0);
    }

    #[test]
    fn nothing_is_nan_on_the_very_first_frame_of_a_cold_aircraft() {
        let mut live = HydraulicsLive::new();
        live.tick(&Truth::default(), &Faults::default());
        let mut ok = true;
        live.publish(&mut |name, value| {
            if !value.is_finite() {
                println!("non-finite {name}");
                ok = false;
            }
        });
        assert!(ok);
    }

    #[test]
    fn the_area_plugs_into_deep_through_the_live_contract() {
        use crate::deep::live::Deep;
        let mut deep = Deep::new().with_area(super::live_system());
        assert_eq!(deep.area_names(), vec!["hydraulics"]);
        let mut published = BTreeMap::new();
        deep.tick(running_truth(), &Faults::default(), &mut |name, value| {
            published.insert(name.to_string(), value);
        });
        assert!(published.contains_key("HYD_GREEN_MANIFOLD_PRESSURE_PSI"));
        assert!(published.contains_key("HYD_YELLOW_RESERVOIR_OVHT"));
    }
}
