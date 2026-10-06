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
//! ## Wired from `Truth` (2026-09-20 pass)
//!
//! Three of the four inputs this model needed with no field on `Truth` are
//! real now. As with every other deep area, `deep::live::Deep::tick` drives
//! this one purely through the `Area` trait (`tick`/`publish`), so the old
//! setters below were never called by anything in production; they are
//! superseded, not merely supplemented, by reading `Truth` directly in
//! `tick`.
//!
//! * **HP spool speed** -- `truth.engine_n3_frac`. The EDP is driven off
//!   the HP (N3) spool through the accessory gearbox; `Truth` used to
//!   carry only N1, so the shaft speed was interpolated between this
//!   aircraft's own two cited operating points instead (still available,
//!   unused now, as [`n3_frac_from_n1_frac`] -- its own test keeps proving
//!   that stand-in reproduces its two citations, in case anything ever
//!   needs it again).
//! * **Fire handle position** -- `truth.controls.fire_pb_released`, read
//!   straight into [`EdpInputs::fire_handle_pulled`] every tick; the
//!   registered fire-shutoff-valve failures finally have a healthy
//!   counterpart (a handle that has *not* been pulled) to act against.
//! * **Heat-exchanger fuel flow** -- `truth.engine_fuel_flow_kg_s`, summed
//!   over the two engines that drive each circuit's pumps
//!   ([`GREEN_PUMP_ENGINE_INDEX`]/[`YELLOW_PUMP_ENGINE_INDEX`]): the fluid
//!   a circuit's own fuel/hydraulic heat exchanger can reach is the feed
//!   flow of the engines it shares a gearbox with, not a third, unrelated
//!   engine's. **Feed temperature is still not on `Truth`**
//!   (`engine_commands.rs` reads a per-engine `feed_fuel_temp` internally
//!   but it is never published) -- kept at the same interim 288.15 K
//!   (15 C) `Truth::environment`'s own cold-aircraft default already uses,
//!   documented here rather than invented as something more specific.
//!
//! **Consumer flow demand** is the one input still not read from
//! `Truth`, but it is no longer completely absent either:
//! `deep::flight_controls::live` now publishes its own half of it
//! (`FCTL_{GREEN,YELLOW}_DEMAND_M3_S`) through `Truth::published`, and
//! `tick` reads it back with `get_or(name, 0.0)` -- the mechanism
//! `docs/deep/truth-requests.md`'s "Contract gap" section already
//! describes for exactly this kind of cross-area read. Gear, brakes,
//! steering, cargo-door and reverser demand are still zero: those areas
//! do not yet publish a flow number of their own.

use std::collections::BTreeMap;

use crate::deep::live::{Area as LiveArea, DerivedFailure, Faults, Truth};

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

/// The HP spool fraction implied by an LP spool fraction, kept for
/// reference after `Truth::engine_n3_frac` went real
/// (`docs/deep/truth-requests.md`'s 2026-09-20 pass) and this area's own
/// `tick` stopped calling it.
///
/// **This was a stand-in, not physics**: the real relationship between the
/// two spools is set by the engine's own working line, which this crate's
/// `physics::engine` does model but `Truth` used to not expose. It is a
/// straight-line interpolation through this aircraft's own two cited
/// operating points -- ground idle (15% N1 / 60% N3) and take-off (100% /
/// 100%) -- extended to the origin below idle, chosen because those are
/// the only two points on the curve that *are* public. Left in place, with
/// its own test still proving it reproduces those two citations, in case a
/// future caller ever needs an N1-only estimate again.
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
    /// ECAM completeness pass (E-FIRE §G/§H): pure discretes, read
    /// directly by `HydraulicsLive::publish` -- none of these feed
    /// `topology::CircuitFaults` (module doc, `registry.rs`'s own comment
    /// at their registration site).
    fuel_hx_valve_stuck: u64,
    fuel_hx_air_leak: u64,
    fuel_hx_air_leak_circuit_fault: u64,
    ovht_chan_a_fault: u64,
    ovht_chan_b_fault: u64,
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
        // ECAM completeness pass (E-FIRE §G/§H): the fuel/hydraulic HX
        // valve (1 failure: stuck), its air-leak-detection switch (2:
        // leak then circuit_fault, `registry.rs`'s own registration
        // order), and the two overheat-detection channels (1 failure
        // each).
        let fuel_hx_valve_stuck = take(map, &format!("29_hyd.{color}_fuel_hx_valve"), 1)[0];
        let fuel_hx_air_leak_switch = take(map, &format!("29_hyd.{color}_fuel_hx_air_leak_switch"), 2);
        let ovht_chan_a_fault = take(map, &format!("29_hyd.{color}_ovht_chan_a"), 1)[0];
        let ovht_chan_b_fault = take(map, &format!("29_hyd.{color}_ovht_chan_b"), 1)[0];
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
            fuel_hx_valve_stuck,
            fuel_hx_air_leak: fuel_hx_air_leak_switch[0],
            fuel_hx_air_leak_circuit_fault: fuel_hx_air_leak_switch[1],
            ovht_chan_a_fault,
            ovht_chan_b_fault,
        }
    }
}

/// The registered magnitude meaning of every leak failure in this area:
/// "leak orifice area, 0..20 mm^2".
const MAX_LEAK_AREA_M2: f64 = 20.0e-6;

// ---------------------------------------------------------------------------
// Authority: this area's level-2 couplings into FlyByWire's own failures
// (`docs/deep/authority.md`).
//
// FlyByWire's A380 hydraulic system models the same eight engine-driven
// pumps, the same four electric pumps and the same two reservoirs this one
// does, and its failure catalogue has an id for each. Everything finer --
// the compensator curve, case drain, cavitation, the accumulator, the
// priority and relief valves, the return filter, the five consumer branch
// line leaks, fluid temperature -- is below its resolution and is published
// and nothing else (level 1).
//
// Ids are `crate::failures::a380_failures()`'s own.

/// `FailureType::EnginePumpOverheat(Edp1a..Edp4b)`, in that function's own
/// order: engine 1 pump a, 1b, 2a, 2b, 3a, 3b, 4a, 4b. Green drives
/// engines 1 and 2, yellow 3 and 4, which is exactly this area's own
/// `GREEN_PUMP_ENGINE_INDEX`/`YELLOW_PUMP_ENGINE_INDEX` split, so the first
/// four are green's four pumps and the last four yellow's.
const FBW_ENGINE_PUMP: [u64; 8] = [29_010, 29_011, 29_012, 29_013, 29_014, 29_015, 29_016, 29_017];
/// `FailureType::ElecPumpOverheat(GreenA, GreenB, YellowA, YellowB)`.
const FBW_ELECTRIC_PUMP: [u64; 4] = [29_006, 29_007, 29_008, 29_009];
/// `FailureType::ReservoirLeak(Green, Yellow)`: FlyByWire drains its own
/// reservoir, which drives its `HYD_<colour>_RESERVOIR_LEVEL_IS_LOW` and
/// everything downstream of it.
const FBW_RESERVOIR_LEAK: [u64; 2] = [29_000, 29_001];
/// `FailureType::ReservoirAirLeak(Green, Yellow)` -- FlyByWire keeps the
/// reservoir's *pressurisation* in its pneumatic system
/// (`fbw-common .../pneumatic/mod.rs:692`), which is where that failure
/// empties the bootstrap air.
const FBW_RESERVOIR_AIR_LEAK: [u64; 2] = [29_002, 29_003];

/// How much of its rated displacement a pump must have lost before the
/// verdict "this pump has failed" is worth sending to FlyByWire, whose own
/// pump failure is binary.
///
/// **GENERIC**, and unavoidably so: `authority.md`'s stated granularity
/// limit is that a continuously degraded component either rounds to a trip
/// or stays invisible at level 2. Half is the rounding point at which a
/// pump has lost more capability than it has left; below it the deep model
/// keeps the remaining fraction and FlyByWire is told nothing, which is the
/// honest half of the same limit.
const PUMP_LOST_CAPABILITY: f64 = 0.5;

/// How much of the reservoir's regulated bootstrap air supply has to be
/// available before a low reservoir air pressure is the *reservoir's* fault
/// rather than the absence of a supply. A cold, dark aircraft has no bleed
/// and no packs, so its reservoirs genuinely sit unpressurised; blaming the
/// reservoir for that would hand the crew a failure on every cold start.
/// **GENERIC**: half the regulator's setting, the same rounding logic as
/// [`PUMP_LOST_CAPABILITY`], chosen so that a partial supply still has to
/// be able to do most of the job before the reservoir is blamed.
const PRESSURISATION_SUPPLY_PRESENT: f64 = 0.5;

/// What fraction of its rated displacement a pump can still deliver, from
/// the two faults that take displacement away. `wear` is deliberately not
/// in here: it is a persisted health parameter that raises case drain
/// rather than a failure, and `circuit_faults` holds it at zero.
fn pump_capability(f: &PumpFaults) -> f64 {
    ((1.0 - f.seizure.clamp(0.0, 1.0)) * (1.0 - f.displacement_loss.clamp(0.0, 1.0))).clamp(0.0, 1.0)
}

/// The deep component behind each engine-driven pump coupling, in
/// [`FBW_ENGINE_PUMP`]'s order (`registry.rs`'s own component ids).
const EDP_COMPONENT: [&str; 8] = [
    "29_hyd.green_edp_1a",
    "29_hyd.green_edp_1b",
    "29_hyd.green_edp_2a",
    "29_hyd.green_edp_2b",
    "29_hyd.yellow_edp_3a",
    "29_hyd.yellow_edp_3b",
    "29_hyd.yellow_edp_4a",
    "29_hyd.yellow_edp_4b",
];
const ELECTRIC_PUMP_COMPONENT: [&str; 4] =
    ["29_hyd.green_electric_pump_a", "29_hyd.green_electric_pump_b", "29_hyd.yellow_electric_pump_a", "29_hyd.yellow_electric_pump_b"];
const RESERVOIR_COMPONENT: [&str; 2] = ["29_hyd.green_reservoir", "29_hyd.yellow_reservoir"];

/// Every level-2 coupling this area owns, as `(FlyByWire failure id, deep
/// component)`, in the order [`HydraulicsLive::each_coupling`] emits them.
fn coupling_table() -> Vec<(u64, &'static str)> {
    let mut v: Vec<(u64, &'static str)> = Vec::new();
    for i in 0..8 {
        v.push((FBW_ENGINE_PUMP[i], EDP_COMPONENT[i]));
    }
    for i in 0..4 {
        v.push((FBW_ELECTRIC_PUMP[i], ELECTRIC_PUMP_COMPONENT[i]));
    }
    for i in 0..2 {
        v.push((FBW_RESERVOIR_LEAK[i], RESERVOIR_COMPONENT[i]));
    }
    for i in 0..2 {
        v.push((FBW_RESERVOIR_AIR_LEAK[i], RESERVOIR_COMPONENT[i]));
    }
    v
}

// ---------------------------------------------------------------------------
// The live system.
// ---------------------------------------------------------------------------

/// Which physical engine drives each of a circuit's four pumps, as
/// `topology.rs` documents (green from engines 1 and 2, yellow from 3 and
/// 4; each engine drives two pumps, a and b). Zero-based here because
/// `Truth`'s per-engine arrays are.
const GREEN_PUMP_ENGINE_INDEX: [usize; 4] = [0, 0, 1, 1];
const YELLOW_PUMP_ENGINE_INDEX: [usize; 4] = [2, 2, 3, 3];

/// One tick of what both circuits were given and what they did, kept by
/// [`DrainRecorder`].
#[derive(Clone, Copy, Default)]
struct RecordedTick {
    time_s: f64,
    dt_s: f64,
    pressurization: f64,
    n3_frac: [f64; 4],
    fire_pb_released: [bool; 4],
    ac_bus_volts: [f64; 4],
    dc_bus_volts: [f64; 2],
    fctl_demand_m3_s: [f64; 2],
    out: [CircuitOutputs; 2],
}

/// A flight recorder for the live-only reservoir drain
/// (`E:/fbw-debug` 2026-09-26: live, both circuits pinned at the network
/// solver's pressure floor seconds after the EFB preset's "APU off / spoiler
/// arm / flaps 1" block, then both reservoirs emptied within ~30 s, which no
/// offline scenario reproduces). Keeps the last few seconds of per-tick
/// inputs and outputs and writes them to the X-Plane log the moment either
/// circuit pins below zero pressure or loses reservoir fluid quickly, then
/// one line a second for a while after, so the log shows exactly which
/// input moved. Diagnostic only: it never changes the simulation.
struct DrainRecorder {
    ring: std::collections::VecDeque<RecordedTick>,
    time_s: f64,
    dumps_left: u32,
    follow_until_s: f64,
    next_follow_s: f64,
}

impl DrainRecorder {
    /// ~5 s at X-Plane's usual 35-45 frames a second.
    const CAPACITY: usize = 200;
    /// Below this a node is pinned at the solver floor (-300,000 Pa), not at
    /// a real near-zero pressure.
    const PINNED_PA: f64 = -1.0 * PSI_PA;
    /// Fill lost within the ring's span that is not the accumulator and
    /// lines charging after a start (~1.2 L, 0.026 of the green reservoir).
    const FAST_FILL_LOSS: f64 = 0.05;
    const FOLLOW_S: f64 = 30.0;

    fn new() -> Self {
        Self { ring: std::collections::VecDeque::with_capacity(Self::CAPACITY), time_s: 0.0, dumps_left: 3, follow_until_s: 0.0, next_follow_s: 0.0 }
    }

    fn line(t: &RecordedTick, now_s: f64) -> String {
        let mut l = format!(
            "t{:+.2}s dt {:.3} press {:.2} n3 [{:.2} {:.2} {:.2} {:.2}] fire {:?} ac [{:.0} {:.0} {:.0} {:.0}] dc [{:.0} {:.0}] fctl L/s [{:.3} {:.3}]",
            t.time_s - now_s,
            t.dt_s,
            t.pressurization,
            t.n3_frac[0], t.n3_frac[1], t.n3_frac[2], t.n3_frac[3],
            t.fire_pb_released.map(u8::from),
            t.ac_bus_volts[0], t.ac_bus_volts[1], t.ac_bus_volts[2], t.ac_bus_volts[3],
            t.dc_bus_volts[0], t.dc_bus_volts[1],
            t.fctl_demand_m3_s[0] * 1e3, t.fctl_demand_m3_s[1] * 1e3,
        );
        for (name, c) in ["G", "Y"].iter().zip(t.out.iter()) {
            let nodes: Vec<String> = c.node_pressures_pa.iter().map(|p| format!("{:.0}", p / PSI_PA)).collect();
            l += &format!(
                " | {name} fill {:.4} in {:.3} out {:.3} leak {:.3} relief {:.3} acc {:.3} L/s inlet {:.1} psi edp {:.3} {:.3} {:.3} {:.3} elec {:.3} {:.3} L/s nodes psi [{}] unconserved mL: solver {:.2} relief {:.2} casedrain {:.2} check {:.2} return {:.2} resclamp {:.2} acc {:.2} lmv {:.2}",
                c.reservoir_fill_fraction,
                c.reservoir_inflow_m3_s * 1e3, c.reservoir_outflow_m3_s * 1e3, c.network_leaked_m3_s * 1e3,
                c.relief_flow_m3_s * 1e3, c.accumulator_flow_m3_s * 1e3,
                c.reservoir_inlet_pa / PSI_PA,
                c.edp[0].flow_m3_s * 1e3, c.edp[1].flow_m3_s * 1e3, c.edp[2].flow_m3_s * 1e3, c.edp[3].flow_m3_s * 1e3,
                c.electric_pump_flow_m3_s[0] * 1e3, c.electric_pump_flow_m3_s[1] * 1e3,
                nodes.join(" "),
                c.conservation.solver_m3 * 1e6, c.conservation.relief_lag_m3 * 1e6, c.conservation.case_drain_lag_m3 * 1e6, c.conservation.check_valve_m3 * 1e6,
                c.conservation.return_clamp_m3 * 1e6, c.conservation.reservoir_clamp_m3 * 1e6, c.conservation.accumulator_clamp_m3 * 1e6, c.conservation.lmv_tap_m3 * 1e6,
            );
        }
        l
    }

    fn record(&mut self, mut tick: RecordedTick) {
        self.time_s += tick.dt_s;
        tick.time_s = self.time_s;
        if self.ring.len() == Self::CAPACITY {
            self.ring.pop_front();
        }
        self.ring.push_back(tick);

        let now = self.time_s;
        if now < self.follow_until_s {
            if now >= self.next_follow_s {
                self.next_follow_s = now + 1.0;
                crate::log(&format!("hyd recorder: after {}", Self::line(&tick, now)));
            }
            return;
        }
        if self.dumps_left == 0 {
            return;
        }
        let pinned = tick.out.iter().any(|c| c.manifold_pressure_pa < Self::PINNED_PA || c.essential_pressure_pa < Self::PINNED_PA);
        let oldest = self.ring.front().copied().unwrap_or(tick);
        let lost: Vec<f64> = (0..2).map(|i| oldest.out[i].reservoir_fill_fraction - tick.out[i].reservoir_fill_fraction).collect();
        let draining = lost.iter().any(|&l| l > Self::FAST_FILL_LOSS);
        if !(pinned || draining) {
            return;
        }
        self.dumps_left -= 1;
        crate::log(&format!(
            "hyd recorder: {} (fill lost over the last {:.1} s: green {:.4}, yellow {:.4}); the last {} ticks follow, then one line a second for {:.0} s",
            if pinned { "a circuit pinned below zero pressure" } else { "a reservoir is losing fluid fast" },
            now - oldest.time_s, lost[0], lost[1], self.ring.len(), Self::FOLLOW_S,
        ));
        let n = self.ring.len();
        for (k, t) in self.ring.iter().enumerate() {
            if k % 5 == 0 || k + 15 >= n {
                crate::log(&format!("hyd recorder: {}", Self::line(t, now)));
            }
        }
        self.follow_until_s = now + Self::FOLLOW_S;
        self.next_follow_s = now + 1.0;
    }
}

/// ECAM completeness pass (E-FIRE §G/§H): one circuit's pure monitored-
/// discrete faults, cached from `tick` because `publish` takes `&self`
/// (the same pattern `edp_failed`/`reservoir_fluid_lost` etc. already use
/// just below). None of these feed `topology::CircuitFaults`; each is a
/// direct pass-through of its own registered failure's armed state.
#[derive(Clone, Copy, Debug, Default)]
struct HxDiscreteFaults {
    fuel_hx_valve_stuck: bool,
    fuel_hx_air_leak: bool,
    fuel_hx_air_leak_circuit_fault: bool,
    ovht_chan_a_fault: bool,
    ovht_chan_b_fault: bool,
}

pub struct HydraulicsLive {
    hyd: A380Hydraulics,
    green_ids: CircuitFailureIds,
    yellow_ids: CircuitFailureIds,
    /// `[displacement loss, seizure]` per electric pump: green A, green B,
    /// yellow A, yellow B.
    electric_pump_ids: [[u64; 2]; 4],
    green_out: CircuitOutputs,
    yellow_out: CircuitOutputs,
    /// ECAM completeness pass (E-FIRE §G/§H): this frame's discrete faults,
    /// per circuit.
    green_hx: HxDiscreteFaults,
    yellow_hx: HxDiscreteFaults,

    /// This frame's level-2 verdicts (`docs/deep/authority.md`), kept from
    /// `tick` because `derived_failures`/`publish` take `&self`. Eight
    /// engine-driven pumps in [`FBW_ENGINE_PUMP`]'s order, four electric
    /// pumps in [`FBW_ELECTRIC_PUMP`]'s, then the two reservoirs' fluid and
    /// air verdicts, green then yellow.
    edp_failed: [bool; 8],
    electric_pump_failed: [bool; 4],
    reservoir_fluid_lost: [bool; 2],
    reservoir_air_lost: [bool; 2],
    /// One published name per coupling, in [`coupling_table`]'s order.
    derived_names: Vec<String>,

    /// Engine feed fuel temperature into the heat exchangers, K. Still not
    /// on `Truth` (see module doc); kept at the same interim cold-aircraft
    /// value `Truth::environment`'s own default uses, and documented as
    /// such rather than invented as something more specific.
    fuel_temp_k: f64,

    recorder: DrainRecorder,
}

impl Default for HydraulicsLive {
    fn default() -> Self {
        Self::new()
    }
}

impl HydraulicsLive {
    pub fn new() -> Self {
        let map = registered_failures();
        let electric_pump_ids = [
            take(&map, "29_hyd.green_electric_pump_a", 2),
            take(&map, "29_hyd.green_electric_pump_b", 2),
            take(&map, "29_hyd.yellow_electric_pump_a", 2),
            take(&map, "29_hyd.yellow_electric_pump_b", 2),
        ]
        .map(|v| [v[0], v[1]]);
        Self {
            hyd: A380Hydraulics::new(),
            green_ids: CircuitFailureIds::build(&map, "green", [1, 2]),
            yellow_ids: CircuitFailureIds::build(&map, "yellow", [3, 4]),
            electric_pump_ids,
            green_out: CircuitOutputs::default(),
            yellow_out: CircuitOutputs::default(),
            green_hx: HxDiscreteFaults::default(),
            yellow_hx: HxDiscreteFaults::default(),
            edp_failed: [false; 8],
            electric_pump_failed: [false; 4],
            reservoir_fluid_lost: [false; 2],
            reservoir_air_lost: [false; 2],
            derived_names: coupling_table().into_iter().map(|(id, _)| format!("DEEP_DERIVED_FBW_FAILURE_{id}")).collect(),
            fuel_temp_k: 288.15,
            recorder: DrainRecorder::new(),
        }
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

    fn edp_inputs(truth: &Truth, engine_index: [usize; 4]) -> [EdpInputs; 4] {
        std::array::from_fn(|pump| {
            let e = engine_index[pump];
            // A pump only turns while its engine's core is actually
            // rotating: a windmilling-but-unlit core still drives its
            // pumps, which is why this keys off N3 rather than off
            // `engine_running`, and why a shut-down engine's residual N3
            // still delivers a little flow as it runs down. `Truth::
            // engine_n3_frac` is real now (`docs/deep/truth-requests.md`'s
            // 2026-09-20 pass); the module doc's `n3_frac_from_n1_frac`
            // stand-in is no longer read here at all.
            let shaft_rpm = (truth.engine_n3_frac[e].max(0.0) * N3_DESIGN_RPM * PUMP_N3_GEAR_RATIO).max(0.0);
            EdpInputs { shaft_rpm, fire_handle_pulled: truth.controls.fire_pb_released[e] }
        })
    }

    /// This tick's fuel flow available to one circuit's fuel/hydraulic heat
    /// exchanger, kg/s: the sum of the feed flow of the two engines that
    /// drive that circuit's own pumps (see module doc). Real per-engine
    /// readings, combined the same way `pressurization_supply_fraction`
    /// above already combines several real sources into one circuit-level
    /// number -- not a fabricated figure, an aggregation of ones that are.
    fn circuit_fuel_kg_s(truth: &Truth, engine_index: [usize; 4]) -> f64 {
        // `engine_index` names each of the circuit's four pumps' engine,
        // repeating each of the circuit's two engines twice (one entry per
        // pump, `GREEN_PUMP_ENGINE_INDEX`/`YELLOW_PUMP_ENGINE_INDEX`'s own
        // `[e, e, e2, e2]` shape) -- so summing entries 0 and 2 sums each
        // engine's feed exactly once.
        truth.engine_fuel_flow_kg_s[engine_index[0]] + truth.engine_fuel_flow_kg_s[engine_index[2]]
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

    /// `catalogue_filter_clog`: the extra catalogue's own "<colour> circuit
    /// filter clogging" id (29_100 green, 29_101 yellow), which had no
    /// consumer anywhere until it was folded into this circuit's own
    /// return-filter clog here (`deep/plugin.rs`'s `EXTRA_IDS`); combined by
    /// `max` with the area's own id, so arming both is not double the clog.
    fn circuit_faults(ids: &CircuitFailureIds, faults: &Faults, electric_pump: [PumpFaults; 2], catalogue_filter_clog: u64) -> CircuitFaults {
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
            filter_clog: faults.get(ids.filter_clog).max(faults.get(catalogue_filter_clog)),
            line_leak_area_m2: ids.line_leaks.map(|id| faults.get(id) * MAX_LEAK_AREA_M2),
            air_ingestion: faults.get(ids.air_ingestion),
        }
    }

    /// This area's whole level-2 coupling table, with this frame's verdict
    /// on each entry (`docs/deep/authority.md`). Emitted in
    /// [`coupling_table`]'s order, healthy entries at magnitude `0.0`, so
    /// `publish` and `derived_failures` walk one list.
    fn each_coupling(&self, out: &mut dyn FnMut(DerivedFailure)) {
        let m = |b: bool| if b { 1.0 } else { 0.0 };
        for i in 0..8 {
            out(DerivedFailure {
                fbw_id: FBW_ENGINE_PUMP[i],
                magnitude: m(self.edp_failed[i]),
                deep_component: EDP_COMPONENT[i],
                reason: "seized, or below half its rated displacement",
            });
        }
        for i in 0..4 {
            out(DerivedFailure {
                fbw_id: FBW_ELECTRIC_PUMP[i],
                magnitude: m(self.electric_pump_failed[i]),
                deep_component: ELECTRIC_PUMP_COMPONENT[i],
                reason: "seized, or below half its rated displacement",
            });
        }
        for i in 0..2 {
            out(DerivedFailure {
                fbw_id: FBW_RESERVOIR_LEAK[i],
                magnitude: m(self.reservoir_fluid_lost[i]),
                deep_component: RESERVOIR_COMPONENT[i],
                reason: "fluid quantity has fallen to the low-level switch",
            });
        }
        for i in 0..2 {
            out(DerivedFailure {
                fbw_id: FBW_RESERVOIR_AIR_LEAK[i],
                magnitude: m(self.reservoir_air_lost[i]),
                deep_component: RESERVOIR_COMPONENT[i],
                reason: "bootstrap air supply present, reservoir still below its pressure switch",
            });
        }
    }
}

/// Circuit-level published variables, per colour.
fn publish_circuit(out: &mut dyn FnMut(&str, f64), color: &str, c: &CircuitOutputs, pumps: [&str; 4]) {
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
    // Both circuits carry two electric pumps.
    for (i, letter) in ["A", "B"].iter().enumerate() {
        out(&format!("HYD_{color}_ELEC_PUMP_{letter}_FLOW_L_MIN"), c.electric_pump_flow_m3_s[i] * M3_S_TO_L_MIN);
    }
    // ECAM completeness pass (E-FIRE §I, `290800037`/`290800038` HYD {G,Y}
    // SYS TEMP HI): the manifold's own thermal state, genuinely distinct
    // from `HYD_{color}_RESERVOIR_OVHT` above -- see `ThermalSizing::
    // a380_manifold`'s module doc.
    out(&format!("HYD_{color}_MANIFOLD_TEMP_C"), c.manifold_temp_c);
    out(&format!("HYD_{color}_SYS_TEMP_HI"), f64::from(u8::from(c.manifold_overheat)));
}

/// ECAM completeness pass (E-FIRE §G/§H): the fuel/hydraulic HX valve, its
/// air-leak-detection switch and the two overheat-detection channels' own
/// monitored-circuit discretes, per colour.
fn publish_hx(out: &mut dyn FnMut(&str, f64), color: &str, d: &HxDiscreteFaults) {
    let b = |x: bool| f64::from(u8::from(x));
    out(&format!("HYD_{color}_FUEL_HX_VALVE_FAULT"), b(d.fuel_hx_valve_stuck));
    out(&format!("HYD_{color}_FUEL_HX_AIR_LEAK"), b(d.fuel_hx_air_leak));
    out(&format!("HYD_{color}_FUEL_HX_AIR_LEAK_DET_FAULT"), b(d.fuel_hx_air_leak_circuit_fault));
    out(&format!("HYD_{color}_SYS_CHAN_A_OVHT_DET_FAULT"), b(d.ovht_chan_a_fault));
    out(&format!("HYD_{color}_SYS_CHAN_B_OVHT_DET_FAULT"), b(d.ovht_chan_b_fault));
}

impl LiveArea for HydraulicsLive {
    fn name(&self) -> &'static str {
        "hydraulics"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        let dt = truth.dt_s.max(0.0);
        let ambient_k = truth.environment.sat_c + 273.15;
        let pressurization = Self::pressurization_supply_fraction(truth);

        // Each circuit's two electric pumps sit on separate AC buses, with
        // one shared control bus per circuit: green A/B on AC 1/2 with DC 2
        // control, yellow A/B on AC 3/4 with DC 1
        // (`a380_systems/src/hydraulic/mod.rs:1767-1780`). A pump runs when
        // its own supply bus is live and its circuit's control bus is up,
        // so losing one AC bus costs that circuit one pump, not both.
        let green_control = truth.dc_bus_volts[1] > DC_BUS_LIVE_V;
        let green_supply = [truth.ac_bus_volts[0], truth.ac_bus_volts[1]];
        let yellow_control = truth.dc_bus_volts[0] > DC_BUS_LIVE_V;
        let yellow_supply = [truth.ac_bus_volts[2], truth.ac_bus_volts[3]];

        // `deep::flight_controls::live` publishes its own half of consumer
        // flow demand (module doc); gear/brakes/steering/cargo-doors/
        // reversers stay at `ConsumerDemands::default()`'s zero until
        // their own areas publish a number too.
        let green_demands = ConsumerDemands { flight_controls_m3_s: truth.published.get_or("FCTL_GREEN_DEMAND_M3_S", 0.0), ..ConsumerDemands::default() };
        let yellow_demands = ConsumerDemands { flight_controls_m3_s: truth.published.get_or("FCTL_YELLOW_DEMAND_M3_S", 0.0), ..ConsumerDemands::default() };

        let green_inputs = CircuitInputs {
            edp: Self::edp_inputs(truth, GREEN_PUMP_ENGINE_INDEX),
            electric_pump_powered: green_supply.map(|v| green_control && v > AC_BUS_LIVE_V),
            electric_pump_bus_voltage_v: green_supply,
            demands: green_demands,
            fuel_kg_s: Self::circuit_fuel_kg_s(truth, GREEN_PUMP_ENGINE_INDEX),
            fuel_temp_k: self.fuel_temp_k,
            ambient_k,
            pressurization_supply_fraction: pressurization,
        };

        let yellow_inputs = CircuitInputs {
            edp: Self::edp_inputs(truth, YELLOW_PUMP_ENGINE_INDEX),
            electric_pump_powered: yellow_supply.map(|v| yellow_control && v > AC_BUS_LIVE_V),
            electric_pump_bus_voltage_v: yellow_supply,
            demands: yellow_demands,
            fuel_kg_s: Self::circuit_fuel_kg_s(truth, YELLOW_PUMP_ENGINE_INDEX),
            fuel_temp_k: self.fuel_temp_k,
            ambient_k,
            pressurization_supply_fraction: pressurization,
        };

        let pump_faults = |slot: usize| PumpFaults {
            wear: 0.0,
            displacement_loss: faults.get(self.electric_pump_ids[slot][0]),
            seizure: faults.get(self.electric_pump_ids[slot][1]),
        };
        let green_faults = Self::circuit_faults(&self.green_ids, faults, [pump_faults(0), pump_faults(1)], 29_100);
        let yellow_faults = Self::circuit_faults(&self.yellow_ids, faults, [pump_faults(2), pump_faults(3)], 29_101);

        self.green_out = self.hyd.green.step(&green_inputs, &green_faults, dt);
        self.yellow_out = self.hyd.yellow.step(&yellow_inputs, &yellow_faults, dt);
        self.recorder.record(RecordedTick {
            time_s: 0.0,
            dt_s: dt,
            pressurization,
            n3_frac: truth.engine_n3_frac,
            fire_pb_released: truth.controls.fire_pb_released,
            ac_bus_volts: truth.ac_bus_volts,
            dc_bus_volts: truth.dc_bus_volts,
            fctl_demand_m3_s: [green_inputs.demands.flight_controls_m3_s, yellow_inputs.demands.flight_controls_m3_s],
            out: [self.green_out, self.yellow_out],
        });

        // ECAM completeness pass (E-FIRE §G/§H): pure discretes, a direct
        // pass-through of each registered failure's own armed state (module
        // doc, `HxDiscreteFaults`) -- not physics, so read straight from
        // `faults` rather than through `CircuitFaults`/`Circuit::step`.
        self.green_hx = HxDiscreteFaults {
            fuel_hx_valve_stuck: faults.get(self.green_ids.fuel_hx_valve_stuck) > 0.0,
            fuel_hx_air_leak: faults.get(self.green_ids.fuel_hx_air_leak) > 0.0,
            fuel_hx_air_leak_circuit_fault: faults.get(self.green_ids.fuel_hx_air_leak_circuit_fault) > 0.0,
            ovht_chan_a_fault: faults.get(self.green_ids.ovht_chan_a_fault) > 0.0,
            ovht_chan_b_fault: faults.get(self.green_ids.ovht_chan_b_fault) > 0.0,
        };
        self.yellow_hx = HxDiscreteFaults {
            fuel_hx_valve_stuck: faults.get(self.yellow_ids.fuel_hx_valve_stuck) > 0.0,
            fuel_hx_air_leak: faults.get(self.yellow_ids.fuel_hx_air_leak) > 0.0,
            fuel_hx_air_leak_circuit_fault: faults.get(self.yellow_ids.fuel_hx_air_leak_circuit_fault) > 0.0,
            ovht_chan_a_fault: faults.get(self.yellow_ids.ovht_chan_a_fault) > 0.0,
            ovht_chan_b_fault: faults.get(self.yellow_ids.ovht_chan_b_fault) > 0.0,
        };

        // Level 2 (`docs/deep/authority.md`): the verdicts on the twelve
        // pumps and two reservoirs FlyByWire also models. A pump's verdict
        // is about the *machine*, not about its operating point -- a pump
        // shut down by a pulled fire handle, destroked by its own
        // compensator or starved of bus power is not a failed pump, and
        // none of those touch `pump_capability`.
        for i in 0..4 {
            self.edp_failed[i] = pump_capability(&green_faults.edp[i].pump) < PUMP_LOST_CAPABILITY;
            self.edp_failed[4 + i] = pump_capability(&yellow_faults.edp[i].pump) < PUMP_LOST_CAPABILITY;
        }
        for i in 0..2 {
            self.electric_pump_failed[i] = pump_capability(&green_faults.electric_pump[i]) < PUMP_LOST_CAPABILITY;
            self.electric_pump_failed[2 + i] = pump_capability(&yellow_faults.electric_pump[i]) < PUMP_LOST_CAPABILITY;
        }
        let supply_present = pressurization >= PRESSURISATION_SUPPLY_PRESENT;
        for (i, c) in [&self.green_out, &self.yellow_out].into_iter().enumerate() {
            self.reservoir_fluid_lost[i] = c.reservoir_low_level_warning;
            // Only the reservoir's own fault: with no bootstrap air supply
            // at all (a cold, dark aircraft) an unpressurised reservoir is
            // the absence of a supply, not a leak.
            self.reservoir_air_lost[i] = supply_present && c.reservoir_low_pressure_warning;
        }
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        publish_circuit(out, "GREEN", &self.green_out, ["1A", "1B", "2A", "2B"]);
        publish_circuit(out, "YELLOW", &self.yellow_out, ["3A", "3B", "4A", "4B"]);
        publish_hx(out, "GREEN", &self.green_hx);
        publish_hx(out, "YELLOW", &self.yellow_hx);
        // The level-2 couplings, so a derived failure is never silent
        // (`docs/deep/authority.md`).
        let mut k = 0usize;
        self.each_coupling(&mut |d| {
            if let Some(name) = self.derived_names.get(k) {
                out(name, d.magnitude);
            }
            k += 1;
        });
    }

    fn derived_failures(&self, out: &mut dyn FnMut(DerivedFailure)) {
        self.each_coupling(out);
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
            // Take-off power on the core too, not just the fan -- now that
            // `tick` reads `engine_n3_frac` directly instead of deriving it
            // from N1, a fixture claiming "four engines at take-off power"
            // has to set both, or every engine-driven pump in these fixtures
            // would see a stopped core regardless of N1.
            engine_n3_frac: [1.0; 4],
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
            // ECAM completeness pass (E-FIRE §G/§H).
            bound.extend([
                ids.fuel_hx_valve_stuck,
                ids.fuel_hx_air_leak,
                ids.fuel_hx_air_leak_circuit_fault,
                ids.ovht_chan_a_fault,
                ids.ovht_chan_b_fault,
            ]);
        }
        bound.extend(live.electric_pump_ids.iter().flatten().copied());
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
        // zero case drain from this pump". Seizing all four now leaves
        // green its two electric pumps, and those alone hold service
        // pressure -- that redundancy is the point of the arrangement. So
        // this checks both halves: the engine-driven pumps alone are not
        // enough to lose the circuit, and with the electric pumps also
        // unpowered `HYD_GREEN_SYS_LO_PR`'s own trigger variable falls
        // through 2900 psi. Yellow, untouched, must not move -- the two
        // circuits are independent.
        let live_ids = HydraulicsLive::new();
        let armed: Vec<(u64, f64)> = live_ids.green_ids.edp.iter().map(|e| (e.seizure, 1.0)).collect();
        let faults = Faults::from_pairs(armed);

        let mut healthy = HydraulicsLive::new();
        let mut seized = HydraulicsLive::new();
        let truth = running_truth();
        let no_green_ac = Truth { ac_bus_volts: [0.0, 0.0, truth.ac_bus_volts[2], truth.ac_bus_volts[3]], ..truth.clone() };
        let edps_seized_and_no_ac = run(&mut HydraulicsLive::new(), &no_green_ac, &faults, 60.0);
        let healthy_out = run(&mut healthy, &truth, &Faults::default(), 60.0);
        let seized_out = run(&mut seized, &truth, &faults, 60.0);

        assert!(healthy_out["HYD_GREEN_MANIFOLD_PRESSURE_PSI"] > 2900.0);
        assert!(
            seized_out["HYD_GREEN_MANIFOLD_PRESSURE_PSI"] > 2900.0,
            "four seized EDPs still leave green its two electric pumps: {:.0} psi",
            seized_out["HYD_GREEN_MANIFOLD_PRESSURE_PSI"]
        );
        assert!(
            edps_seized_and_no_ac["HYD_GREEN_MANIFOLD_PRESSURE_PSI"] < 2900.0,
            "with every green source gone HYD GREEN SYS LO PR must trip: {:.0} psi",
            edps_seized_and_no_ac["HYD_GREEN_MANIFOLD_PRESSURE_PSI"]
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

        // (build fix, INT-P4) 120s -> 400s: this batch's own hydraulics
        // accumulator fix (`accumulator.rs::exchange_flow_at`'s physical-
        // capacity bound) removed a real bug where an unbounded phantom
        // exchange kept the manifold pressure oscillating/chattering every
        // tick, which made the EDPs work continuously harder and draw
        // measurably more fluid from the reservoir than the corrected,
        // calmer model now does. The leak still measurably outpaces the
        // healthy circuit (the assertion just below this comment, which
        // already passed); it now just needs more simulated time to cross
        // the LOW threshold from a full reservoir under the corrected,
        // slower draw rate.
        let mut healthy = HydraulicsLive::new();
        let mut leaking = HydraulicsLive::new();
        let healthy_out = run(&mut healthy, &truth, &Faults::default(), 400.0);
        let leaking_out = run(&mut leaking, &truth, &faults, 400.0);

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
    fn each_electric_pump_needs_its_own_supply_bus_and_its_circuits_control_bus() {
        // Green A/B on AC 1/2 with DC 2 control, yellow A/B on AC 3/4 with
        // DC 1 (a380_systems/src/hydraulic/mod.rs:1767-1780).
        let mut on_batteries_only = Truth { dt_s: 0.02, dc_bus_volts: [28.0, 28.0], ..Truth::default() };
        on_batteries_only.ac_bus_volts = [0.0; 4];
        let out = run(&mut HydraulicsLive::new(), &on_batteries_only, &Faults::default(), 30.0);
        for name in ["HYD_GREEN_ELEC_PUMP_A_FLOW_L_MIN", "HYD_GREEN_ELEC_PUMP_B_FLOW_L_MIN", "HYD_YELLOW_ELEC_PUMP_A_FLOW_L_MIN", "HYD_YELLOW_ELEC_PUMP_B_FLOW_L_MIN"] {
            assert!(out[name].abs() < 1e-9, "no AC supply, no pump: {name}");
        }

        // One AC bus live each side: each circuit keeps exactly the pump on
        // the bus that survived, which is why the pair is split across two.
        let ac1_and_ac4 = Truth { dt_s: 0.02, ac_bus_volts: [115.0, 0.0, 0.0, 115.0], dc_bus_volts: [28.0, 28.0], ..Truth::default() };
        let out = run(&mut HydraulicsLive::new(), &ac1_and_ac4, &Faults::default(), 30.0);
        assert!(out["HYD_GREEN_ELEC_PUMP_A_FLOW_L_MIN"] > 0.0, "AC 1 runs green A");
        assert!(out["HYD_GREEN_ELEC_PUMP_B_FLOW_L_MIN"].abs() < 1e-9, "AC 2 is dead, so green B is");
        assert!(out["HYD_YELLOW_ELEC_PUMP_B_FLOW_L_MIN"] > 0.0, "AC 4 runs yellow B");
        assert!(out["HYD_YELLOW_ELEC_PUMP_A_FLOW_L_MIN"].abs() < 1e-9, "AC 3 is dead, so yellow A is");
        assert!(out["HYD_GREEN_MANIFOLD_PRESSURE_PSI"] > 100.0);
        assert!(out["HYD_YELLOW_MANIFOLD_PRESSURE_PSI"] > 100.0);

        // Control power is per circuit: losing DC 2 stops both green pumps
        // and leaves yellow's alone.
        let no_dc2 = Truth { dt_s: 0.02, ac_bus_volts: [115.0; 4], dc_bus_volts: [28.0, 0.0], ..Truth::default() };
        let out = run(&mut HydraulicsLive::new(), &no_dc2, &Faults::default(), 30.0);
        assert!(out["HYD_GREEN_ELEC_PUMP_A_FLOW_L_MIN"].abs() < 1e-9, "no DC 2 control power, no green pump");
        assert!(out["HYD_GREEN_ELEC_PUMP_B_FLOW_L_MIN"].abs() < 1e-9);
        assert!(out["HYD_YELLOW_ELEC_PUMP_A_FLOW_L_MIN"] > 0.0, "yellow's control bus is DC 1 and is untouched");
    }

    #[test]
    fn the_hp_spool_stand_in_reproduces_both_cited_operating_points() {
        assert!((n3_frac_from_n1_frac(0.15) - 0.60).abs() < 1e-12, "ground idle: 15% N1 is 60% N3");
        assert!((n3_frac_from_n1_frac(1.0) - 1.0).abs() < 1e-12, "take-off: 100% N1 is 100% N3");
        assert_eq!(n3_frac_from_n1_frac(0.0), 0.0);
        assert!(n3_frac_from_n1_frac(0.5) > n3_frac_from_n1_frac(0.3), "monotonic");
    }

    #[test]
    fn an_engine_driven_pump_follows_core_speed_not_fan_speed() {
        // The EDP is geared to the HP (N3) spool through the accessory
        // gearbox, not the fan (N1) -- `Truth::engine_n3_frac` is real now
        // (`docs/deep/truth-requests.md`'s 2026-09-20 pass) and `edp_inputs`
        // reads it directly, with no more N1-derived stand-in in the loop.
        // N1 says take-off, N3 says the core is stopped: the pump must not
        // turn. The AC buses are dead too, so the only thing that could
        // hold green pressure here is an engine-driven pump actually
        // turning.
        let n1_high_n3_stopped = Truth { engine_n3_frac: [0.0; 4], ac_bus_volts: [0.0; 4], ..running_truth() };
        let out = run(&mut HydraulicsLive::new(), &n1_high_n3_stopped, &Faults::default(), 30.0);
        assert!(out["HYD_GREEN_EDP_1A_FLOW_L_MIN"].abs() < 1e-9, "N1 alone must not turn an engine-driven pump: {}", out["HYD_GREEN_EDP_1A_FLOW_L_MIN"]);
        assert!(out["HYD_GREEN_MANIFOLD_PRESSURE_PSI"] < 100.0);

        // The other way round: fan stopped, core still turning (a
        // windmilling-but-unlit core, or simply N1 not yet spun up) -- the
        // pump must turn anyway, on N3 alone. A pressure-compensated pump
        // destrokes toward zero net flow once it has reached and is
        // holding its regulated pressure with nothing drawing on it
        // (`a_healthy_running_aircraft_...`'s own steady state shows the
        // same thing), so the proof here is that pressure, not
        // instantaneous flow.
        let n1_stopped_n3_high =
            Truth { dt_s: 0.02, engine_n1_frac: [0.0; 4], engine_n3_frac: [1.0; 4], ac_bus_volts: [0.0; 4], dc_bus_volts: [28.0; 2], ..Truth::default() };
        let out = run(&mut HydraulicsLive::new(), &n1_stopped_n3_high, &Faults::default(), 30.0);
        assert!(
            out["HYD_GREEN_MANIFOLD_PRESSURE_PSI"] > 2900.0,
            "N3 alone must still turn an engine-driven pump enough to hold service pressure: {:.0} psi",
            out["HYD_GREEN_MANIFOLD_PRESSURE_PSI"]
        );
    }

    #[test]
    fn pulling_a_fire_handle_shuts_that_engines_pump_unless_the_shutoff_valve_is_stuck() {
        // `truth.controls.fire_pb_released` gives the registered
        // fire-shutoff-valve failures a healthy counterpart to act
        // against (module doc): without a real "handle pulled" signal the
        // valve was never actually being commanded shut, so "stuck open"
        // had nothing to disagree with.
        //
        // Isolated so engine 1's own two green pumps are the *only*
        // possible green source, the same isolation style
        // `seizing_every_green_engine_driven_pump_...` above already uses:
        // AC power off (no electric pumps) and engine 2 stopped too, so
        // there is nothing else that could be holding the pressure this
        // test reads.
        let mut truth = running_truth();
        truth.ac_bus_volts = [0.0; 4];
        truth.engine_n1_frac[1] = 0.0;
        truth.engine_n3_frac[1] = 0.0;
        truth.engine_running[1] = false;
        truth.controls.fire_pb_released[0] = true; // engine 1's handle pulled

        let healthy_valve = run(&mut HydraulicsLive::new(), &truth, &Faults::default(), 30.0);
        assert!(
            healthy_valve["HYD_GREEN_MANIFOLD_PRESSURE_PSI"] < 2900.0,
            "a healthy fire shutoff valve must close, leaving the circuit with no green source at all: {:.0} psi",
            healthy_valve["HYD_GREEN_MANIFOLD_PRESSURE_PSI"]
        );

        let id = HydraulicsLive::new().green_ids.edp[0].fire_sov_stuck;
        let stuck_open = run(&mut HydraulicsLive::new(), &truth, &Faults::from_pairs([(id, 1.0)]), 30.0);
        assert!(
            stuck_open["HYD_GREEN_MANIFOLD_PRESSURE_PSI"] > 2900.0,
            "a stuck-open valve must keep engine 1's pump delivering even with the handle pulled: {:.0} psi",
            stuck_open["HYD_GREEN_MANIFOLD_PRESSURE_PSI"]
        );
    }

    #[test]
    fn engine_fuel_flow_now_cools_the_circuit_that_used_to_have_none() {
        // Module doc: with no fuel flow the only cooling path was the
        // passive bay loss, so the circuit ran hotter than it should.
        // `Truth::engine_fuel_flow_kg_s` is real now; a circuit with real
        // fuel flow through its heat exchanger must run cooler than the
        // same circuit with none, everything else equal.
        let mut hot = running_truth();
        hot.dt_s = 0.5;
        let mut cooled = hot.clone();
        cooled.engine_fuel_flow_kg_s = [1.0; 4];

        let hot_out = run(&mut HydraulicsLive::new(), &hot, &Faults::default(), 1800.0);
        let cooled_out = run(&mut HydraulicsLive::new(), &cooled, &Faults::default(), 1800.0);
        assert!(
            cooled_out["HYD_GREEN_FLUID_TEMP_C"] < hot_out["HYD_GREEN_FLUID_TEMP_C"],
            "real fuel flow through the heat exchanger must cool the circuit below the fuel-less baseline: {} vs {}",
            cooled_out["HYD_GREEN_FLUID_TEMP_C"],
            hot_out["HYD_GREEN_FLUID_TEMP_C"]
        );
    }

    #[test]
    fn flight_controls_demand_read_through_published_raises_flow_and_drops_pressure() {
        // `deep::flight_controls::live` publishes `FCTL_GREEN_DEMAND_M3_S`;
        // this proves `deep::hydraulics::live` actually reads it back
        // through `Truth::published` (the cross-area mechanism this
        // module's doc describes) rather than leaving every circuit
        // permanently unloaded.
        let truth = running_truth();
        let mut unloaded = crate::deep::live::PublishedFrame::default();
        let mut loaded = crate::deep::live::PublishedFrame::default();
        // Comfortably past this circuit's own four-EDP capacity (~2.8 in^3
        // x 3782 rpm each at full N3, ~11.6 L/s combined -- `pump.rs`'s own
        // compensator curve and this area's `PUMP_N3_GEAR_RATIO`/
        // `N3_DESIGN_RPM`), so the compensator cannot hide this demand
        // behind its usual constant-pressure regulation the way a modest,
        // within-capacity demand could.
        loaded.insert("FCTL_GREEN_DEMAND_M3_S".to_string(), 0.02);

        // A startup transient (both cases spike well above regulated
        // pressure in the first second as the reservoir/accumulator prime)
        // needs real time to settle before the steady-state effect of the
        // demand itself is visible, hence the long run.
        let mut idle = HydraulicsLive::new();
        let mut loaded_live = HydraulicsLive::new();
        let idle_out = run(&mut idle, &Truth { published: unloaded, ..truth.clone() }, &Faults::default(), 60.0);
        let loaded_out = run(&mut loaded_live, &Truth { published: loaded, ..truth }, &Faults::default(), 60.0);
        assert!(
            loaded_out["HYD_GREEN_MANIFOLD_PRESSURE_PSI"] < idle_out["HYD_GREEN_MANIFOLD_PRESSURE_PSI"],
            "a real consumer demand must load the circuit, not leave it as optimistic as an unloaded one: {} vs {}",
            loaded_out["HYD_GREEN_MANIFOLD_PRESSURE_PSI"],
            idle_out["HYD_GREEN_MANIFOLD_PRESSURE_PSI"]
        );
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

    // -----------------------------------------------------------------
    // Authority (docs/deep/authority.md): the level-2 couplings.

    fn derived(live: &HydraulicsLive) -> BTreeMap<u64, f64> {
        let mut out = BTreeMap::new();
        live.derived_failures(&mut |d| {
            // Two couplings share the reservoir component (fluid and air),
            // so key by the FlyByWire id, which is unique.
            out.insert(d.fbw_id, d.magnitude);
        });
        out
    }

    #[test]
    fn the_coupling_table_matches_what_the_area_actually_emits() {
        let live = HydraulicsLive::new();
        let table = coupling_table();
        let mut emitted: Vec<(u64, &'static str)> = Vec::new();
        live.each_coupling(&mut |d| emitted.push((d.fbw_id, d.deep_component)));
        assert_eq!(emitted, table, "coupling_table() and each_coupling() must walk the same list in the same order");
        assert_eq!(live.derived_names.len(), table.len());
        assert_eq!(table.len(), 16, "8 engine-driven pumps, 4 electric pumps, 2 reservoirs x (fluid, air)");

        let catalogue: std::collections::BTreeSet<u64> = crate::failures::a380_failures().into_iter().map(|(id, _)| id).collect();
        for (id, component) in &table {
            assert!(catalogue.contains(id), "{component} derives {id}, which FlyByWire does not register");
        }
        let mut ids: Vec<u64> = table.iter().map(|(id, _)| *id).collect();
        let before = ids.len();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), before, "two deep components must not claim the same FlyByWire failure");

        // Every registered component named here really exists in this
        // area's registry, so a rename cannot leave the crew looking at a
        // component that is not there.
        let map = registered_failures();
        for (_, component) in &table {
            assert!(map.contains_key(*component), "{component} is not a registered hydraulics component");
        }
    }

    #[test]
    fn a_healthy_aircraft_tells_flybywire_nothing_at_all() {
        let mut running = HydraulicsLive::new();
        run(&mut running, &running_truth(), &Faults::default(), 60.0);
        assert!(derived(&running).values().all(|&m| m == 0.0), "running: {:?}", derived(&running));

        // A cold, dark aircraft has unpressurised reservoirs because there
        // is no bleed and no pack running, not because they leak.
        let mut cold = HydraulicsLive::new();
        run(&mut cold, &Truth { dt_s: 0.02, ..Truth::default() }, &Faults::default(), 60.0);
        assert!(derived(&cold).values().all(|&m| m == 0.0), "cold and dark: {:?}", derived(&cold));
    }

    /// The extra catalogue's "green/yellow circuit filter clogging" (29_100,
    /// 29_101) had no consumer; each is now its own circuit's return filter
    /// clog here, and only that circuit's.
    #[test]
    fn the_catalogues_filter_clogs_reach_their_own_circuits_return_filter() {
        let live = HydraulicsLive::new();
        let pumps = || [PumpFaults::default(); 2];
        for (id, green_clog, yellow_clog) in [(29_100, 0.7, 0.0), (29_101, 0.0, 0.7)] {
            let faults = Faults::from_pairs([(id, 0.7)]);
            let green = HydraulicsLive::circuit_faults(&live.green_ids, &faults, pumps(), 29_100);
            let yellow = HydraulicsLive::circuit_faults(&live.yellow_ids, &faults, pumps(), 29_101);
            assert_eq!(green.filter_clog, green_clog, "{id} on green");
            assert_eq!(yellow.filter_clog, yellow_clog, "{id} on yellow");
        }
    }

    #[test]
    fn a_seized_engine_driven_pump_reaches_flybywire_as_that_exact_pump() {
        // Green EDP 1a is `AirbusEngineDrivenPumpId::Edp1a`, id 29_010.
        let id = HydraulicsLive::new().green_ids.edp[0].seizure;
        let mut live = HydraulicsLive::new();
        run(&mut live, &running_truth(), &Faults::from_pairs([(id, 1.0)]), 10.0);
        let d = derived(&live);
        assert_eq!(d.get(&29_010), Some(&1.0), "the seized pump must reach FlyByWire");
        for other in [29_011, 29_012, 29_013, 29_014, 29_015, 29_016, 29_017] {
            assert_eq!(d.get(&other), Some(&0.0), "no other pump may be blamed ({other})");
        }
    }

    #[test]
    fn a_partly_destroked_pump_stays_below_flybywires_resolution() {
        // `authority.md`'s granularity limit, tested rather than assumed:
        // a pump that has lost a third of its displacement is still doing
        // most of its job, and FlyByWire's binary pump failure has no way
        // to say "two thirds of a pump". The deep model keeps the two
        // thirds; FlyByWire is told nothing.
        let ids = HydraulicsLive::new().green_ids.edp[0];
        let mut mild = HydraulicsLive::new();
        run(&mut mild, &running_truth(), &Faults::from_pairs([(ids.displacement_loss, 0.33)]), 10.0);
        assert_eq!(derived(&mild).get(&29_010), Some(&0.0), "a third of a pump is not a failed pump");

        let mut severe = HydraulicsLive::new();
        run(&mut severe, &running_truth(), &Faults::from_pairs([(ids.displacement_loss, 0.9)]), 10.0);
        assert_eq!(derived(&severe).get(&29_010), Some(&1.0), "a pump down to a tenth of its displacement has failed");
    }

    #[test]
    fn a_pulled_fire_handle_is_not_a_failed_pump() {
        // The pump delivers nothing, and is in perfect health: the fire
        // shutoff valve FlyByWire models itself is what shut it. Blaming
        // the pump would leave it failed after the handle was pushed back
        // in.
        let mut truth = running_truth();
        truth.controls.fire_pb_released[0] = true;
        let mut live = HydraulicsLive::new();
        run(&mut live, &truth, &Faults::default(), 30.0);
        assert!(derived(&live).values().all(|&m| m == 0.0), "{:?}", derived(&live));
    }

    #[test]
    fn a_reservoir_drained_to_its_low_level_switch_reaches_flybywire_as_a_leak() {
        // The same failure `a_reservoir_leak_drains_the_published_level...`
        // above exercises, followed through to FlyByWire: its coarse model
        // has a green reservoir too, and `ReservoirLeak(Green)` is the
        // coarsest input that carries "this circuit is losing its fluid".
        let id = HydraulicsLive::new().green_ids.reservoir_leak;
        let mut live = HydraulicsLive::new();
        // (build fix, INT-P4) 120s -> 400s, same reason as the sibling test
        // above (`a_reservoir_leak_drains_the_published_level...`).
        let published = run(&mut live, &running_truth(), &Faults::from_pairs([(id, 1.0)]), 400.0);
        assert_eq!(published["HYD_GREEN_RESERVOIR_LEVEL_IS_LOW"], 1.0, "setup");
        let d = derived(&live);
        assert_eq!(d.get(&29_000), Some(&1.0), "green reservoir leak must reach FlyByWire");
        assert_eq!(d.get(&29_001), Some(&0.0), "yellow is a separate reservoir");
        assert_eq!(published["DEEP_DERIVED_FBW_FAILURE_29000"], 1.0, "and it must be visible");
    }

    #[test]
    fn losing_reservoir_pressurisation_with_a_supply_present_reaches_flybywire() {
        let id = HydraulicsLive::new().yellow_ids.reservoir_pressurization_loss;
        let mut live = HydraulicsLive::new();
        let published = run(&mut live, &running_truth(), &Faults::from_pairs([(id, 1.0)]), 60.0);
        assert_eq!(published["HYD_YELLOW_RESERVOIR_AIR_PRESSURE_IS_LOW"], 1.0, "setup");
        let d = derived(&live);
        assert_eq!(d.get(&29_003), Some(&1.0), "yellow reservoir air leak must reach FlyByWire");
        assert_eq!(d.get(&29_002), Some(&0.0));
    }

    /// End to end, with FlyByWire's own A380 systems on the other side.
    /// The global `crate::failures` registry is not touched (it is
    /// process-wide); the id is mapped to its `FailureType` exactly as
    /// `Failures::apply` does.
    #[test]
    fn a_derived_reservoir_leak_changes_flybywires_own_solve() {
        use crate::aspects::test_vars::TestVars;
        use std::time::Duration;
        use systems::simulation::{Simulation, StartState};

        let id = HydraulicsLive::new().green_ids.reservoir_leak;
        let mut live = HydraulicsLive::new();
        // (build fix, INT-P4) 120s -> 400s, same reason as the sibling
        // reservoir-leak tests above.
        run(&mut live, &running_truth(), &Faults::from_pairs([(id, 1.0)]), 400.0);
        let verdict = derived(&live);
        assert_eq!(verdict.get(&29_000), Some(&1.0), "setup: the deep reservoir must have run down to its switch");

        let mut vars = TestVars::default();
        let mut sim = Simulation::new(StartState::Cruise, a380_systems::A380::new, &mut vars);
        let tick = |sim: &mut Simulation<a380_systems::A380>, vars: &mut TestVars, n: usize| {
            for i in 0..n {
                sim.tick(Duration::from_millis(50), 100.0 + i as f64 * 0.05, vars);
            }
        };
        tick(&mut sim, &mut vars, 20);
        let before: f64 = vars.value("A32NX_HYD_GREEN_RESERVOIR_LEVEL");
        assert!(before > 0.0, "setup: FlyByWire's own green reservoir has fluid in it");

        let types: Vec<systems::failures::FailureType> =
            crate::failures::a380_failures().into_iter().filter(|(fid, _)| verdict.get(fid).copied().unwrap_or(0.0) > 0.0).map(|(_, t)| t).collect();
        assert!(!types.is_empty());
        sim.update_active_failures(types.into_iter().collect());
        tick(&mut sim, &mut vars, 200);
        let after: f64 = vars.value("A32NX_HYD_GREEN_RESERVOIR_LEVEL");
        assert!(after < before, "the deep model's verdict must drain FlyByWire's own reservoir too: {before} -> {after}");
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

    #[test]
    fn the_drain_recorder_fires_once_on_a_pinned_circuit_and_is_quiet_on_a_healthy_one() {
        let healthy = RecordedTick { dt_s: 0.027, out: [CircuitOutputs { manifold_pressure_pa: 5000.0 * PSI_PA, essential_pressure_pa: 5000.0 * PSI_PA, reservoir_fill_fraction: 0.97, ..CircuitOutputs::default() }; 2], ..RecordedTick::default() };
        let mut rec = DrainRecorder::new();
        for _ in 0..500 {
            rec.record(healthy);
        }
        assert_eq!(rec.dumps_left, 3, "a steady healthy circuit must never trigger it");

        let mut pinned = healthy;
        pinned.out[0].manifold_pressure_pa = -300_000.0;
        rec.record(pinned);
        assert_eq!(rec.dumps_left, 2, "the solver-floor pin is exactly what it is there to catch");
        for _ in 0..100 {
            rec.record(pinned);
        }
        assert_eq!(rec.dumps_left, 2, "and it follows up with one line a second, not a dump every tick");
    }

    /// The live drain this area once had (23/24 Sep: both reservoirs empty
    /// within ~30 s of an EFB preset, which raised FlyByWire's reservoir
    /// leaks) came from the network solver pinning at its bracket, fixed
    /// since. Random sequences of everything the plugin feeds this area --
    /// bus power, bleed, per-engine N3, fire pushbuttons, flight-control
    /// demand on either circuit, frame time -- must never take a healthy
    /// reservoir anywhere near its low-level switch.
    #[test]
    fn random_input_sequences_never_drain_a_healthy_reservoir() {
        let mut seed: u64 = 0x9E37_79B9_7F4A_7C15;
        let mut rnd = move |n: u64| {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            seed % n
        };
        for scenario in 0..6 {
            let mut area = HydraulicsLive::new();
            let faults = Faults::default();
            let mut t = Truth { engine_n3_frac: [0.66; 4], engine_running: [true; 4], ac_bus_volts: [115.0; 4], dc_bus_volts: [28.0; 2], ..running_truth() };
            run(&mut area, &t, &faults, 20.0);
            for _ in 0..12 {
                let ac = rnd(3);
                let dc = rnd(3);
                t.ac_bus_volts = std::array::from_fn(|k| if ac == 0 || (ac == 1 && k % 2 == 1) { 0.0 } else { 115.0 });
                t.dc_bus_volts = std::array::from_fn(|k| if dc == 0 || (dc == 1 && k == 1) { 0.0 } else { 28.0 });
                t.apu_bleed_pressure_pa = [0.0, 101_325.0 + 40.0 * PSI_PA][rnd(2) as usize];
                t.engine_n3_frac = std::array::from_fn(|_| [0.0, 0.2, 0.66, 1.0][rnd(4) as usize]);
                t.controls.fire_pb_released = std::array::from_fn(|_| rnd(8) == 0);
                for name in ["FCTL_GREEN_DEMAND_M3_S", "FCTL_YELLOW_DEMAND_M3_S"] {
                    t.published.insert(name, [0.0, 0.3e-3, 2e-3, 20e-3][rnd(4) as usize]);
                }
                t.dt_s = [0.027, 0.05, 0.2][rnd(3) as usize];
                let seconds = [0.5, 1.0, 3.0][rnd(3) as usize];
                for _ in 0..((seconds / t.dt_s) as usize).max(1) {
                    area.tick(&t, &faults);
                    for c in [area.green(), area.yellow()] {
                        assert!(
                            c.reservoir_fill_fraction > 0.8 && !c.reservoir_low_level_warning,
                            "scenario {scenario}: a healthy circuit lost fluid it has no way to lose: fill {:.4}",
                            c.reservoir_fill_fraction
                        );
                    }
                }
            }
        }
    }
}
