//! Per-load A380 electrical network core: buses, individual loads, breakers
//! with a real thermal (I^2t) / instant (magnetic) trip curve, contactors
//! (open/closed/welded/failed-open), bus ties and diodes -- solved every
//! tick as a resistive, constant-power/constant-conductance nodal network,
//! not a topology-only powered/unpowered graph.
//!
//! **Why this exists.** FlyByWire's own `a380_systems::electrical` module
//! (`D:\fbw-aircraft\...\a380_systems\src\electrical\mod.rs`) is a real
//! Kirchhoff *connectivity* graph (`Electricity`/`Potential`, contactors
//! grouping buses into equipotential sets) with real source physics (VFG/TRU/
//! battery internal impedance -- see `sources.rs`'s citations), but its own
//! load side (`power_consumption.rs`, `A380PowerConsumption`) is 14 lumped
//! `FlightPhasePowerConsumer`s, one flat wattage per *whole bus* keyed only
//! to flight phase ("the watts in this function are all provided by komp").
//! A breaker pull, a chafed wire, a seized motor or a bus tie transient has
//! nothing individual to act on there. This module is that missing layer:
//! every one of `D:\fbw-xp-systems\src\breakers.rs`'s ~130 named ATA-grouped
//! consumers (`loads.rs`'s catalogue cites each one back to its `breakers.rs`
//! entry) plus the major loads it does not enumerate (galleys, IFE, fuel
//! pumps, window/probe heat, lighting feeders, ...) becomes its own
//! [`Load`], each with its own rated power, power factor, minimum operating
//! voltage, inrush, protecting [`Breaker`] and fault state, so a fault (or a
//! pulled breaker) changes exactly what it should and nothing else.
//!
//! **The physics.** Every tick:
//! 1. Each closed [`Contactor`]/conducting [`Diode`] is resolved (its own
//!    fault state -- welded/fails-to-close/open-circuit -- decided first).
//! 2. Each bus's loads are aggregated into a real power demand `P` (every
//!    healthy/degraded/inrush-boosted load, which behaves as a *regulated,
//!    constant-power* consumer -- the same physical assumption
//!    `physics::electrical.rs`'s `Protection::update` already makes for its
//!    154 `systems.cfg` circuits, `current = rated_watts / bus_voltage`) and
//!    a fault conductance `G` (every `short_to_ground`, which is
//!    *unregulated*: current set only by the bus voltage and the wiring's
//!    own resistance, `I = V / R_wiring`, not by the load's control loop).
//! 3. Every bus's own Thevenin-equivalent supply (`V_th`, `R_th`) is formed
//!    by combining, in parallel (Millman's theorem, elementary circuit
//!    theory, not a model assumption), every closed contactor/diode path
//!    reaching it -- a direct [`Source`] (a VFG/TRU/battery/GPU `sources.rs`
//!    already reduced to an open-circuit voltage and internal resistance) or
//!    a tie to a neighbouring bus (using that bus's own voltage from the
//!    previous sweep). [`Network::step`] repeats this a fixed number of
//!    sweeps (Gauss-Seidel/Jacobi relaxation over the resistive network,
//!    textbook technique for a network with loops -- a straight one-pass
//!    calculation would only be exact for a tree topology, and a live bus
//!    tie can close a loop) so a multi-hop tie chain converges within one
//!    tick.
//! 4. Each bus's own voltage is then the same self-consistent
//!    constant-power/constant-conductance solve `transformer_rectifier.rs`
//!    and `engine_generator.rs` already use for *their* single source
//!    (`V^2 - V_rated*V + S*Xs = 0`, `electrical.md` section 1/4): here
//!    generalised to `(1 + R_th*G)*V^2 - V_th*V + R_th*P = 0`, which reduces
//!    to exactly that shape at `G = 0`. A negative discriminant means the
//!    bus's demand exceeds what its Thevenin source can ever deliver (the
//!    real "source current-limits and the bus sags hard" state); the branch
//!    below takes the source's own maximum-power-transfer point
//!    (`V_th / (2*(1+R_th*G))`, found by maximising delivered power over
//!    `V`) instead of returning a complex/nonsense voltage.
//! 5. Each load's real current is read back at the converged bus voltage,
//!    summed onto its protecting breaker, and each breaker steps its own
//!    I^2t thermal accumulator / instant magnetic trip (the same curve
//!    shape -- and, at `REFERENCE_AMBIENT`-equivalent, the same K/multiple/
//!    cooldown constants -- as `physics::electrical.rs`'s already-cited
//!    `trip_step`; reimplemented here rather than imported because this
//!    directory's code must not depend on crate internals).
//!
//! Every fault is a plain `0.0..=1.0` severity on its own model
//! (`LoadFaults`, `BreakerFaults`, `ContactorFaults`, `DiodeFaults`,
//! `BusFaults`), `Default` = healthy, exactly the brief's convention.
//! Probabilistic faults (a breaker that only *sometimes* fails to trip, a
//! contactor that only sometimes welds) use a deterministic seeded hash
//! (`fnv1a`/`splitmix64`-style, [`bernoulli`]) keyed on the component's own
//! `'static` id and the network's tick counter, not the system RNG -- so a
//! severity of exactly `0.0` or `1.0` (the values every test below drives)
//! is bit-for-bit deterministic (the `bernoulli` short-circuits before
//! touching the hash at all), while a partial severity still produces a
//! reproducible, seedless-external-state failure rate.

use std::collections::HashMap;

// ---------------------------------------------------------------------
// Tunable constants. Every one is cited to its physical justification or
// marked GENERIC with how it was chosen; none are invented magic numbers.

/// Gauss-Seidel/Jacobi relaxation sweeps per tick for the bus-tie network
/// (GENERIC: a small resistive network with at most a handful of live tie
/// loops converges to float precision in well under this many passes --
/// `network_converges_within_its_own_iteration_budget` below checks it).
const ITERATIONS: usize = 15;
/// Passes for the (secondary, informational) AC frequency propagation --
/// same reasoning as `ITERATIONS`, smaller because frequency has no
/// nonlinear feedback to converge, just hop-count.
const FREQUENCY_PASSES: usize = 4;
/// Floor on any path resistance, so a caller-supplied zero (a "perfect"
/// busbar or contactor) never divides by zero in the admittance sum.
const MIN_RESISTANCE_OHM: f64 = 1.0e-4;
/// Below this admittance a bus is treated as fed by nothing at all (open
/// circuit -> 0 V), rather than risking a near-zero-divide blow-up.
const MIN_ADMITTANCE: f64 = 1.0e-9;
const MIN_VOLTAGE_V: f64 = 1.0e-6;
/// A bus counts as "powered" above this fraction of its nominal voltage --
/// GENERIC, matching the same order of undervoltage margin MIL-STD-704F
/// allows an AC/DC bus to sag to before equipment is no longer guaranteed to
/// operate (the standard's own AC/DC steady-state limits are roughly 90-107%
/// nominal; 50% is deliberately looser, this flag is "is there any real
/// power here at all", per-load `min_operating_voltage` in `loads.rs` is
/// what actually gates a given consumer).
const POWERED_VOLTAGE_FRACTION: f64 = 0.5;
/// A load latched out by a genuine under-voltage condition must see this
/// fraction *above* its own `min_operating_voltage` before it is allowed to
/// resume (comparator/Schmitt-trigger hysteresis) -- GENERIC, a modest,
/// typical margin real under-voltage lockout circuits use precisely to stop
/// a load right at its own dropout point chattering on and off as its own
/// switch-on inrush sags the bus back below the threshold it just cleared.
const UNDERVOLTAGE_RESTART_MARGIN: f64 = 1.05;
/// A `high_resistance` (overheat) load fault's ceiling: at full severity the
/// load draws this much *extra* real power on top of its own rated demand
/// (GENERIC: a badly-degraded but not yet fully shorted winding/connector
/// dissipating up to half as much again as heat, not function).
const HIGH_RESISTANCE_MAX_EXTRA_FRACTION: f64 = 0.5;
/// An `intermittent` load fault's ceiling dropout rate at full severity, Hz
/// (GENERIC: "flickers a couple of times a second", the classic symptom of a
/// chafed/loose connector, not a hard failure).
const INTERMITTENT_MAX_RATE_HZ: f64 = 2.0;
/// A bus-fault (short to structure) is limited by the busbar's own short
/// feeder wiring resistance, not the zero-resistance ideal busbar the
/// topology graph otherwise assumes (GENERIC, same order of magnitude as
/// `Battery::WIRING_RESISTANCE_OHM` = 0.02 ohm,
/// `fbw-common/.../electrical/battery.rs:78`, cited in `sources.rs`).
const BUS_FAULT_RESISTANCE_OHM: f64 = 0.03;

/// Breaker I^2t thermal curve: time-to-trip at 2x rated current, seconds,
/// `K / (r^2 - 1)`; same construction and same value as
/// `physics::electrical.rs`'s `THERMAL_TRIP_K` (`K/(2^2-1) = 10 s`), just
/// reimplemented here (this directory's code may not depend on crate
/// internals) rather than sharing the constant.
const THERMAL_TRIP_K: f64 = 30.0;
/// Instant ("magnetic") trip threshold, same value/citation as
/// `physics::electrical.rs`'s `MAGNETIC_TRIP_MULTIPLE`.
const MAGNETIC_TRIP_MULTIPLE: f64 = 10.0;
/// Thermal element cooldown time constant once current drops below rated,
/// same value/citation as `physics::electrical.rs`'s `COOLDOWN_SECONDS`.
const COOLDOWN_SECONDS: f64 = 20.0;
/// A `nuisance_trip` breaker fault's time-to-trip at full severity with *no*
/// real overload at all (GENERIC: represents a marginal thermal element or a
/// resistive, self-heating loose connection inside the breaker itself).
const NUISANCE_TIME_CONSTANT_S: f64 = 5.0;

// Deterministic-hash salts: distinct constants XORed into a component's own
// id hash so two different fault channels on the same component (e.g. a
// contactor's `fails_to_close` and `welded_closed`) draw independent, not
// correlated, pseudo-random sequences.
const WELD_SALT: u64 = 0x1111_1111_1111_1111;
const FAIL_TO_CLOSE_SALT: u64 = 0x2222_2222_2222_2222;
const DIODE_OPEN_SALT: u64 = 0x3333_3333_3333_3333;
const FAILS_TO_TRIP_SALT: u64 = 0x4444_4444_4444_4444;
const INTERMITTENT_SALT: u64 = 0x5555_5555_5555_5555;

/// `splitmix64` (public-domain, Vigna): a fast, well-mixed 64-bit hash, used
/// only to turn a deterministic `(id, tick)` pair into a reproducible
/// pseudo-random draw for a probabilistic fault -- not a cryptographic or
/// even statistically rigorous RNG, just enough decorrelation that two
/// different components/ticks do not trip in lockstep.
fn splitmix64(mut x: u64) -> u64 {
    x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = x;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// FNV-1a: a simple, deterministic string hash (public domain), used only to
/// turn a component's own `'static` id into a stable seed. `pub(super)`:
/// `shedding.rs`'s own shed-relay faults reuse this exact deterministic-
/// fault convention rather than keeping a separate copy.
pub(super) fn fnv1a(s: &str) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in s.bytes() {
        h ^= b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01B3);
    }
    h
}

/// Uniform pseudo-random value in `[0, 1)` for one `(seed, tick)` draw.
fn uniform01(seed: u64, tick: u64) -> f64 {
    let h = splitmix64(seed ^ splitmix64(tick));
    (h >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
}

/// One deterministic Bernoulli trial: `false` always at `p <= 0.0`, `true`
/// always at `p >= 1.0` (the two endpoints every test below drives, so they
/// never touch the hash and can never flake), otherwise a reproducible draw.
/// `pub(super)`: see [`fnv1a`]'s own doc.
pub(super) fn bernoulli(seed: u64, tick: u64, p: f64) -> bool {
    if p <= 0.0 {
        false
    } else if p >= 1.0 {
        true
    } else {
        uniform01(seed, tick) < p
    }
}

// ---------------------------------------------------------------------
// Buses.

/// The A380's electrical buses this network models (`docs/deep/BRIEF.md`'s
/// own list), matching FlyByWire's `ElectricalBusType` variants
/// (`fbw-common/.../shared/mod.rs`) and `breakers.rs`/`circuits.rs`'s
/// `MSFS_BUSES` names one-for-one in spirit (this module does not import
/// either, per the "no crate internals" rule, but uses the same real names).
/// `AcEmer` is the static inverter's own output bus (FlyByWire's
/// `AlternatingCurrentStaticInverter`, real name kept short here); the two
/// ground-service buses are the GPU-fed AC/DC buses used on stand.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BusId {
    Ac1,
    Ac2,
    Ac3,
    Ac4,
    AcEss,
    AcEssShed,
    AcEmer,
    AcGndFltSvc,
    Dc1,
    Dc2,
    DcEss,
    DcEssShed,
    DcBat,
    DcHot1,
    DcHot2,
    DcApu,
    DcGndFltSvc,
}

/// [`BusId`] in a fixed order matching [`BusId::index`] -- the network's own
/// bus array is always built and indexed in this order.
pub const ALL_BUS_IDS: [BusId; 17] = [
    BusId::Ac1,
    BusId::Ac2,
    BusId::Ac3,
    BusId::Ac4,
    BusId::AcEss,
    BusId::AcEssShed,
    BusId::AcEmer,
    BusId::AcGndFltSvc,
    BusId::Dc1,
    BusId::Dc2,
    BusId::DcEss,
    BusId::DcEssShed,
    BusId::DcBat,
    BusId::DcHot1,
    BusId::DcHot2,
    BusId::DcApu,
    BusId::DcGndFltSvc,
];

impl BusId {
    /// Stable array index, matching [`ALL_BUS_IDS`]'s own order.
    pub const fn index(self) -> usize {
        match self {
            BusId::Ac1 => 0,
            BusId::Ac2 => 1,
            BusId::Ac3 => 2,
            BusId::Ac4 => 3,
            BusId::AcEss => 4,
            BusId::AcEssShed => 5,
            BusId::AcEmer => 6,
            BusId::AcGndFltSvc => 7,
            BusId::Dc1 => 8,
            BusId::Dc2 => 9,
            BusId::DcEss => 10,
            BusId::DcEssShed => 11,
            BusId::DcBat => 12,
            BusId::DcHot1 => 13,
            BusId::DcHot2 => 14,
            BusId::DcApu => 15,
            BusId::DcGndFltSvc => 16,
        }
    }

    pub const fn is_ac(self) -> bool {
        matches!(
            self,
            BusId::Ac1 | BusId::Ac2 | BusId::Ac3 | BusId::Ac4 | BusId::AcEss | BusId::AcEssShed | BusId::AcEmer | BusId::AcGndFltSvc
        )
    }

    /// Nominal bus voltage: 115 V AC (three-phase equivalent) / 28 V DC,
    /// same split `physics::electrical.rs::nominal_bus_voltage` uses and
    /// cites (`EngineGenerator::RATED_VOLTAGE_VOLT` / TRU-fed DC).
    pub const fn nominal_voltage(self) -> f64 {
        if self.is_ac() {
            115.0
        } else {
            28.0
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            BusId::Ac1 => "AC1",
            BusId::Ac2 => "AC2",
            BusId::Ac3 => "AC3",
            BusId::Ac4 => "AC4",
            BusId::AcEss => "AC_ESS",
            BusId::AcEssShed => "AC_ESS_SHED",
            BusId::AcEmer => "AC_EMER",
            BusId::AcGndFltSvc => "AC_GND_FLT_SVC",
            BusId::Dc1 => "DC1",
            BusId::Dc2 => "DC2",
            BusId::DcEss => "DC_ESS",
            BusId::DcEssShed => "DC_ESS_SHED",
            BusId::DcBat => "DC_BAT",
            BusId::DcHot1 => "DC_HOT1",
            BusId::DcHot2 => "DC_HOT2",
            BusId::DcApu => "DC_APU",
            BusId::DcGndFltSvc => "DC_GND_FLT_SVC",
        }
    }
}

const NUM_BUSES: usize = 17;

/// A bus fault: a short from the busbar itself to airframe structure/ground,
/// independent of any one load on it (`docs/deep/BRIEF.md` backlog item 3,
/// "Faults per bus: bus fault"). `0.0` healthy; `1.0` a dead short limited
/// only by [`BUS_FAULT_RESISTANCE_OHM`].
#[derive(Clone, Copy, Debug, Default)]
pub struct BusFaults {
    pub short_to_ground: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct Bus {
    pub id: BusId,
    /// This tick's solved terminal voltage, V.
    pub voltage: f64,
    /// This tick's AC frequency, Hz (0 for a DC bus or an unpowered AC bus;
    /// see [`Network::resolve_frequency`]'s own doc for how this is traced).
    /// A real, load-affecting quantity: [`Load::frequency_multiplier`]
    /// applies fan/pump affinity-law scaling to every load with a nonzero
    /// `LoadSpec::rated_frequency_hz`.
    pub frequency_hz: f64,
    pub faults: BusFaults,
}

impl Bus {
    fn new(id: BusId) -> Self {
        Self { id, voltage: id.nominal_voltage(), frequency_hz: 0.0, faults: BusFaults::default() }
    }
}

// ---------------------------------------------------------------------
// Sources: the network-facing interface every real source in `sources.rs`
// (VFG/TRU/battery/static inverter/RAT/APU gen/ground power) reduces itself
// to each tick -- an open-circuit (no-load) terminal voltage and a series
// (Thevenin) internal resistance, the same reduction FlyByWire's own
// `transformer_rectifier.rs`/`engine_generator.rs` sources use internally
// (`docs/physics/electrical.md` sections 1 and 4, cited in `sources.rs`).
// This module only ever *consumes* that pair; it has no opinion on how a
// VFG's reactance or a battery's internal resistance was derived.
#[derive(Clone, Copy, Debug)]
pub struct Source {
    pub id: &'static str,
    pub open_circuit_v: f64,
    pub resistance_ohm: f64,
    /// AC source frequency, Hz; 0 for a DC source (TRU/battery/GPU DC).
    pub frequency_hz: f64,
}

impl Source {
    pub fn new(id: &'static str) -> Self {
        Self { id, open_circuit_v: 0.0, resistance_ohm: MIN_RESISTANCE_OHM, frequency_hz: 0.0 }
    }
}

/// Which side of a [`Contactor`]/[`Diode`] is the "from" terminal: either a
/// [`Source`] by index, or another [`Bus`] (a tie or a downstream feed).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FeedSource {
    Source(usize),
    Bus(BusId),
}

// ---------------------------------------------------------------------
// Contactors: generator line contactors, bus-tie contactors and feeder
// contactors are electrically identical (a resistance that is either
// connected or not); `kind` is documentation/labelling only, matching the
// real A380's own naming (BTC/GLC/...).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ContactorKind {
    GeneratorLine,
    BusTie,
    Feeder,
    BatteryDirect,
}

/// `0.0` healthy. `fails_to_close`: commanded closed but the contacts never
/// make (a real stuck/burned contactor or a dead coil) -- probability the
/// close attempt fails, re-evaluated every tick it is commanded closed.
/// `welded_closed`: the contacts are fused together and the contactor closes
/// (and stays closed) regardless of command -- a real, if rare, high-current
/// contactor failure mode; takes priority over an open command.
#[derive(Clone, Copy, Debug, Default)]
pub struct ContactorFaults {
    pub fails_to_close: f64,
    pub welded_closed: f64,
}

pub struct Contactor {
    pub id: &'static str,
    pub kind: ContactorKind,
    pub from: FeedSource,
    pub to: BusId,
    pub resistance_ohm: f64,
    pub commanded_closed: bool,
    pub faults: ContactorFaults,
    /// This tick's resolved state, after faults -- what [`Network::step`]'s
    /// solver actually uses.
    pub closed: bool,
}

impl Contactor {
    pub fn new(id: &'static str, kind: ContactorKind, from: FeedSource, to: BusId, resistance_ohm: f64) -> Self {
        Self { id, kind, from, to, resistance_ohm, commanded_closed: false, faults: ContactorFaults::default(), closed: false }
    }

    fn resolve(&mut self, tick: u64) {
        let seed = fnv1a(self.id);
        if bernoulli(seed ^ WELD_SALT, tick, self.faults.welded_closed) {
            self.closed = true;
            return;
        }
        if !self.commanded_closed {
            self.closed = false;
            return;
        }
        self.closed = !bernoulli(seed ^ FAIL_TO_CLOSE_SALT, tick, self.faults.fails_to_close);
    }
}

// ---------------------------------------------------------------------
// Diodes: one-way bus paths (e.g. a battery-direct hot-bus feed that must
// never back-feed into the battery bus from the hot bus side), with a real
// forward voltage drop and series resistance.
#[derive(Clone, Copy, Debug, Default)]
pub struct DiodeFaults {
    /// The diode fails open (junction destroyed by an over-current/reverse
    /// surge -- a real, if uncommon, power-diode failure mode): the path it
    /// provided is simply gone.
    pub open_circuit: f64,
}

pub struct Diode {
    pub id: &'static str,
    pub from: FeedSource,
    pub to: BusId,
    /// Forward conduction drop, V (GENERIC: ~1 V is typical of a high-current
    /// silicon power rectifier at the currents a bus-isolation diode of this
    /// class carries -- an order of magnitude above a small-signal diode's
    /// 0.6-0.7 V because of the diode's own bulk/contact resistance at load
    /// current; no A380-specific figure is public).
    pub forward_drop_v: f64,
    pub resistance_ohm: f64,
    pub faults: DiodeFaults,
}

impl Diode {
    pub fn new(id: &'static str, from: FeedSource, to: BusId, forward_drop_v: f64, resistance_ohm: f64) -> Self {
        Self { id, from, to, forward_drop_v, resistance_ohm, faults: DiodeFaults::default() }
    }

    fn open_fault(&self, tick: u64) -> bool {
        bernoulli(fnv1a(self.id) ^ DIODE_OPEN_SALT, tick, self.faults.open_circuit)
    }
}

// ---------------------------------------------------------------------
// Breakers.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TripCause {
    Thermal,
    Magnetic,
    /// Tripped by heat accumulated from a `nuisance_trip` fault alone (see
    /// [`BreakerFaults::nuisance_trip`]), not a real overload.
    Nuisance,
}

/// `0.0` healthy. `fails_to_trip`: the thermal/magnetic element reaches its
/// threshold but the contacts never actually open (a jammed mechanism) --
/// probability the trip attempt fails, re-evaluated on every attempt.
/// `nuisance_trip`: the breaker heats and eventually trips with *no* real
/// overload at all (a marginal element or a resistive loose connection
/// inside the breaker itself), continuous extra heat proportional to
/// severity.
#[derive(Clone, Copy, Debug, Default)]
pub struct BreakerFaults {
    pub fails_to_trip: f64,
    pub nuisance_trip: f64,
}

pub struct Breaker {
    pub id: &'static str,
    pub rated_a: f64,
    pub bus: BusId,
    pub closed: bool,
    heat: f64,
    pub trip_cause: Option<TripCause>,
    pub faults: BreakerFaults,
    pub current_a: f64,
}

impl Breaker {
    pub fn new(id: &'static str, rated_a: f64, bus: BusId) -> Self {
        Self { id, rated_a, bus, closed: true, heat: 0.0, trip_cause: None, faults: BreakerFaults::default(), current_a: 0.0 }
    }

    /// Manually close a tripped (or pulled) breaker, cooling its thermal
    /// element the way a real bimetal strip cools with current removed.
    pub fn reset(&mut self) {
        self.closed = true;
        self.heat = 0.0;
        self.trip_cause = None;
    }

    pub fn pull(&mut self) {
        self.closed = false;
    }

    /// Normalised I^2t heat accumulator, `0.0..=1.0` (trips at 1.0) --
    /// exposed for tests/telemetry, not part of the public fault contract.
    pub fn heat_fraction(&self) -> f64 {
        self.heat
    }

    fn step(&mut self, current_a: f64, dt_s: f64, tick: u64) {
        if !self.closed {
            self.current_a = 0.0;
            self.heat = (self.heat - dt_s / COOLDOWN_SECONDS).max(0.0);
            return;
        }
        self.current_a = current_a;

        let nuisance = self.faults.nuisance_trip.clamp(0.0, 1.0);
        if nuisance > 0.0 {
            self.heat += dt_s * nuisance / NUISANCE_TIME_CONSTANT_S;
        }

        let ratio = if self.rated_a > 0.0 { current_a / self.rated_a } else { 0.0 };
        if ratio >= MAGNETIC_TRIP_MULTIPLE {
            self.attempt_trip(TripCause::Magnetic, tick);
            return;
        }
        if ratio > 1.0 {
            self.heat += dt_s * (ratio * ratio - 1.0) / THERMAL_TRIP_K;
        } else if nuisance <= 0.0 {
            self.heat = (self.heat - dt_s / COOLDOWN_SECONDS).max(0.0);
        }
        if self.heat >= 1.0 {
            let cause = if ratio > 1.0 { TripCause::Thermal } else { TripCause::Nuisance };
            self.attempt_trip(cause, tick);
        }
    }

    fn attempt_trip(&mut self, cause: TripCause, tick: u64) {
        if bernoulli(fnv1a(self.id) ^ FAILS_TO_TRIP_SALT, tick, self.faults.fails_to_trip) {
            // A jammed thermal-magnetic mechanism: stays right at the trip
            // threshold (so it trips the instant the fault clears enough for
            // `fails_to_trip` to no longer roll true, rather than silently
            // resetting) but the contacts never actually open.
            self.heat = 0.999;
            return;
        }
        self.closed = false;
        self.trip_cause = Some(cause);
        self.heat = 0.0;
    }
}

// ---------------------------------------------------------------------
// Loads.

/// `0.0` healthy for every field.
/// - `open_circuit`: the load's own internal path opens (a broken wire
///   inside the LRU, a burned-out element) -- draws proportionally less
///   current and delivers proportionally less function; `1.0` is fully open
///   (no current, no function).
/// - `short_to_ground`: a wiring/internal short whose current is limited
///   only by the feeder wiring's own resistance, not the load's regulation
///   -- this is what can overload a breaker/bus that a plain open-circuit
///   fault never would.
/// - `high_resistance`: a degrading connection/winding that draws *more*
///   current than rated for the same useful output, dissipated as heat
///   inside the load (an early-stage version of a short, or corrosion at a
///   connector).
/// - `intermittent`: a chafed/loose connection that drops the load out at a
///   rate proportional to severity, independent of the breaker/bus.
#[derive(Clone, Copy, Debug, Default)]
pub struct LoadFaults {
    pub open_circuit: f64,
    pub short_to_ground: f64,
    pub high_resistance: f64,
    pub intermittent: f64,
}

/// One consumer's static definition (`loads.rs`'s catalogue builds these).
#[derive(Clone, Debug)]
pub struct LoadSpec {
    pub id: &'static str,
    pub name: &'static str,
    pub ata: u16,
    pub bus: BusId,
    /// Real power at rated operation, W.
    pub rated_power_w: f64,
    /// `1.0` for a DC/resistive load; `< 1.0` for an AC induction-motor-class
    /// load (current = P / (V * pf), the same relation `breakers.rs`'s own
    /// `generator_rated_a`/`apu_generator_rated_a` use for their own rated
    /// current from a true-power figure).
    pub power_factor: f64,
    /// Below this bus voltage the load's own internal regulation/contactor
    /// drops it off the bus entirely (a real LRU under-voltage lockout), V.
    pub min_operating_voltage: f64,
    /// Steady current at switch-on as a multiple of rated (motors/lamps/
    /// capacitor-input supplies all draw a real inrush); `1.0` for a load
    /// with no meaningful inrush.
    pub inrush_multiple: f64,
    /// Time constant over which the inrush multiple decays back to 1x, s
    /// (an exact `exp(-t / (duration/3))`, so the inrush has decayed to
    /// ~5% of its own excess by `duration` seconds); `0.0` disables inrush.
    pub inrush_duration_s: f64,
    /// This load's own feeder wiring resistance, ohm -- the limit on its
    /// `short_to_ground` fault current (GENERIC per load class, see
    /// `loads.rs`'s constructors; same order of magnitude as
    /// `Battery::WIRING_RESISTANCE_OHM` = 0.02 ohm).
    pub wiring_resistance_ohm: f64,
    /// The AC line frequency this load's own real power is rated at, Hz;
    /// `0.0` for a load whose function does not depend on line frequency at
    /// all (every DC load; every AC load with its own internal rectifier/
    /// switching supply or its own motor-speed controller -- the large
    /// majority of this catalogue). Only a handful of real A380 loads are
    /// simple induction motors driving straight off the bus with no speed
    /// control of their own (cabin/avionics recirculation fans are the
    /// textbook case, and a real, cited engineering complexity of a
    /// variable-frequency system): for those, a nonzero value here makes
    /// [`Load::frequency_multiplier`] apply the fan-affinity-law scaling a
    /// real such motor would show as the VFG's own frequency varies with
    /// engine speed.
    pub rated_frequency_hz: f64,
    pub basis: &'static str,
}

/// One of a [`Load`]'s power inputs: its own bus, the breaker protecting
/// that specific feed, and a construction-order priority (`0` = first
/// choice/normal feed, higher = a backup/ESS feed) -- real dual/triple-fed
/// A380 LRUs (flight-control computers, ADIRUs, CPIOMs, the FWS, display
/// units, DMCs, ...) each have exactly this: two or three independent power
/// inputs (e.g. a normal bus and an essential bus, or an AC and a DC
/// supply), each on its own breaker/SSPC, OR-ed together inside the box's
/// own power supply so losing any one feed alone does not lose the unit.
#[derive(Clone, Copy, Debug)]
pub struct LoadFeed {
    pub bus: BusId,
    /// Index into [`Network::breakers`].
    pub breaker: usize,
    pub priority: u8,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct LoadOutputs {
    pub current_a: f64,
    pub power_w: f64,
    pub powered: bool,
    /// The portion of `current_a` coming from a `short_to_ground` fault
    /// (already included in `current_a`; broken out for telemetry/tests).
    pub fault_current_a: f64,
}

pub struct Load {
    pub spec: LoadSpec,
    /// This load's power inputs, in priority order (index 0 = first choice).
    /// A single-feed load (the large majority of the catalogue) has exactly
    /// one entry; a real dual/triple-fed LRU has two or three.
    pub feeds: Vec<LoadFeed>,
    /// The load's own on/off commanded state (a pushbutton, a running pump
    /// command, ...) independent of bus/breaker health; `true` by default
    /// (most catalogue entries are "on whenever powered").
    pub commanded_on: bool,
    pub faults: LoadFaults,
    time_energized_s: f64,
    was_on: bool,
    pub current_a: f64,
    pub powered: bool,
    /// Which of `feeds` is presently supplying this load, if any -- e.g. a
    /// dual-fed computer showing `Some(1)` after its normal (`0`) feed's
    /// breaker was pulled but its ESS feed (`1`) picked it up with no loss
    /// of function, the real behaviour internal power-supply OR-ing gives.
    pub active_feed: Option<usize>,
    /// `true` once this load has dropped out on a genuine under-voltage
    /// condition on every one of its feeds; raises the voltage it needs to
    /// resume by [`UNDERVOLTAGE_RESTART_MARGIN`] (comparator/Schmitt-trigger
    /// hysteresis, the standard way a real under-voltage lockout avoids
    /// chattering on and off right at its own threshold) until a feed
    /// clears that raised bar, at which point it resumes immediately and
    /// the latch clears.
    undervoltage_latched: bool,
}

impl Load {
    /// A single-feed load (the common case): one bus, one breaker.
    pub fn new(spec: LoadSpec, breaker: usize) -> Self {
        let bus = spec.bus;
        Self::new_multi_feed(spec, vec![LoadFeed { bus, breaker, priority: 0 }])
    }

    /// A load with two or more independent power feeds, OR-ed together
    /// (see [`LoadFeed`]'s own doc) -- every real dual/triple-fed A380 LRU
    /// this catalogue models uses this constructor.
    pub fn new_multi_feed(spec: LoadSpec, feeds: Vec<LoadFeed>) -> Self {
        Self { spec, feeds, commanded_on: true, faults: LoadFaults::default(), time_energized_s: 0.0, was_on: false, current_a: 0.0, powered: false, active_feed: None, undervoltage_latched: false }
    }

    fn health(&self) -> f64 {
        (1.0 - self.faults.open_circuit.clamp(0.0, 1.0)).max(0.0)
    }

    fn inrush_multiplier(&self) -> f64 {
        if self.spec.inrush_duration_s <= 0.0 || self.spec.inrush_multiple <= 1.0 {
            1.0
        } else {
            let tau = (self.spec.inrush_duration_s / 3.0).max(1.0e-6);
            1.0 + (self.spec.inrush_multiple - 1.0) * (-self.time_energized_s / tau).exp()
        }
    }

    /// Fan/pump affinity-law scaling for the small subset of loads with a
    /// nonzero `LoadSpec::rated_frequency_hz` (simple line-frequency
    /// induction motors with no speed control of their own): shaft speed
    /// tracks line frequency directly (a fixed-pole induction motor has no
    /// other option), and for a centrifugal fan/pump the mechanical power
    /// needed follows the cube of speed -- the standard affinity law
    /// `P2/P1 = (N2/N1)^3` (textbook fluid-machinery relation, not a
    /// per-motor characteristic curve). `1.0` (no effect) for every load
    /// with its own internal speed/frequency regulation (`rated_frequency_hz
    /// == 0.0`), and `0.0` at `freq_hz <= 0.0` (a real induction motor
    /// simply does not turn with no rotating field at all).
    fn frequency_multiplier(&self, freq_hz: f64) -> f64 {
        if self.spec.rated_frequency_hz <= 0.0 {
            1.0
        } else if freq_hz <= 0.0 {
            0.0
        } else {
            (freq_hz / self.spec.rated_frequency_hz).powi(3)
        }
    }

    fn intermittent_dropout(&self, dt_s: f64, tick: u64) -> bool {
        let severity = self.faults.intermittent.clamp(0.0, 1.0);
        if severity <= 0.0 {
            return false;
        }
        let rate = severity * INTERMITTENT_MAX_RATE_HZ;
        let p = 1.0 - (-rate * dt_s.max(0.0)).exp();
        bernoulli(fnv1a(self.spec.id) ^ INTERMITTENT_SALT, tick, p)
    }

    fn short_conductance(&self) -> f64 {
        let short = self.faults.short_to_ground.clamp(0.0, 1.0);
        if short <= 0.0 {
            0.0
        } else {
            short / self.spec.wiring_resistance_ohm.max(MIN_RESISTANCE_OHM)
        }
    }

    fn undervoltage_threshold(&self) -> f64 {
        if self.undervoltage_latched {
            self.spec.min_operating_voltage * UNDERVOLTAGE_RESTART_MARGIN
        } else {
            self.spec.min_operating_voltage
        }
    }

    /// Picks the first (highest-priority) feed whose own breaker is closed
    /// and whose bus voltage clears this load's own under-voltage threshold
    /// (raised while [`Load::undervoltage_latched`]) -- the real behaviour
    /// of a dual-fed LRU's internal power-supply OR-ing. `voltages`/
    /// `breakers` are the network's own per-bus voltage array and breaker
    /// list; read-only, so both [`Network::relax`]'s trial sweeps and
    /// [`Network::step`]'s final commit share the identical selection
    /// logic. Returns the winning feed's index into `self.feeds` and that
    /// bus's voltage.
    fn select_feed(&self, voltages: &[f64], breakers: &[Breaker]) -> Option<(usize, f64)> {
        let threshold = self.undervoltage_threshold();
        for (i, feed) in self.feeds.iter().enumerate() {
            if feed.breaker >= breakers.len() || !breakers[feed.breaker].closed {
                continue;
            }
            let v = voltages[feed.bus.index()];
            if v >= threshold {
                return Some((i, v));
            }
        }
        None
    }

    /// Real power `P` and fault conductance `G` this load presents to the
    /// network solver at a trial bus voltage/frequency -- read-only (does
    /// not mutate energised-time/dropout bookkeeping), so
    /// [`Network::step`]'s relaxation sweep can call it many times per tick
    /// against successive voltage estimates. [`Load::step`] below re-derives
    /// the identical `P`/`G` from the tick's *converged* voltage to commit
    /// real state.
    fn contribution(&self, v: f64, freq_hz: f64, dt_s: f64, tick: u64) -> (f64, f64) {
        if !self.commanded_on {
            return (0.0, 0.0);
        }
        if self.intermittent_dropout(dt_s, tick) {
            return (0.0, 0.0);
        }
        let health = self.health();
        if health <= 0.0 {
            // Fully open internally: no regulated demand, but a short fault
            // is a separate, independent conduction path (frayed wire on
            // the supply side of an otherwise-dead load), so it still shows
            // up here.
            return (0.0, self.short_conductance());
        }
        let base_p = self.spec.rated_power_w * health * self.inrush_multiplier() * self.frequency_multiplier(freq_hz);
        let hr = self.faults.high_resistance.clamp(0.0, 1.0);
        let p = base_p * (1.0 + hr * HIGH_RESISTANCE_MAX_EXTRA_FRACTION);
        (p, self.short_conductance())
    }

    /// Commits this tick's real current/power/energised-time state from the
    /// network's converged bus voltages/frequencies, after selecting
    /// whichever feed (if any) is presently live. Call once per tick, after
    /// [`Network::step`]'s relaxation sweep has settled `voltages`.
    pub fn step(&mut self, voltages: &[f64], frequencies: &[f64], breakers: &[Breaker], dt_s: f64, tick: u64) -> LoadOutputs {
        let Some((feed_idx, v)) = self.select_feed(voltages, breakers) else {
            // No feed can supply this load right now. Distinguish "every
            // feed's breaker is simply open" (not a voltage fault, no latch)
            // from "at least one feed's breaker is closed but its bus is
            // below this load's own minimum" (a genuine under-voltage
            // condition, worth latching so the eventual recovery needs the
            // hysteresis margin rather than chattering right at the limit).
            let best_energised_feed_voltage = self
                .feeds
                .iter()
                .filter(|f| f.breaker < breakers.len() && breakers[f.breaker].closed)
                .map(|f| voltages[f.bus.index()])
                .fold(0.0_f64, f64::max);
            if best_energised_feed_voltage > 0.0 {
                self.undervoltage_latched = true;
            }
            self.was_on = false;
            self.time_energized_s = 0.0;
            self.powered = false;
            self.current_a = 0.0;
            self.active_feed = None;
            return LoadOutputs::default();
        };
        self.active_feed = Some(feed_idx);
        self.undervoltage_latched = false;

        let freq_hz = frequencies[self.feeds[feed_idx].bus.index()];
        let (p, g) = self.contribution(v, freq_hz, dt_s, tick);
        let is_on = p > 0.0 || g > 0.0;
        if is_on && !self.was_on {
            self.time_energized_s = 0.0;
        }
        self.time_energized_s = if is_on { self.time_energized_s + dt_s } else { 0.0 };
        self.was_on = is_on;

        let fault_current_a = g * v;
        let base_current_a = if v > MIN_VOLTAGE_V && self.spec.power_factor > 1.0e-6 { p / (v * self.spec.power_factor) } else { 0.0 };
        let current_a = base_current_a + fault_current_a;
        self.current_a = current_a;
        self.powered = p > 0.0;

        LoadOutputs { current_a, power_w: p, powered: self.powered, fault_current_a }
    }
}

// ---------------------------------------------------------------------
// The network.

#[derive(Clone, Debug)]
pub struct NetworkReport {
    pub bus_voltage: [f64; NUM_BUSES],
    pub bus_powered: [bool; NUM_BUSES],
    /// Sum of every load's real delivered power, W (excludes fault
    /// current -- a short is wasted heat, not useful load).
    pub total_power_w: f64,
    /// Breakers that opened (by trip, not a caller's `pull`) this tick.
    pub tripped_breakers: Vec<&'static str>,
}

impl Default for NetworkReport {
    fn default() -> Self {
        Self { bus_voltage: [0.0; NUM_BUSES], bus_powered: [false; NUM_BUSES], total_power_w: 0.0, tripped_breakers: Vec::new() }
    }
}

pub struct Network {
    pub buses: Vec<Bus>,
    pub sources: Vec<Source>,
    pub contactors: Vec<Contactor>,
    pub diodes: Vec<Diode>,
    pub breakers: Vec<Breaker>,
    pub loads: Vec<Load>,
    /// Breakers registered via [`Network::add_feeder_breaker`]: `(breaker
    /// index, bus)`. Unlike an ordinary breaker (whose current is the sum
    /// of the specific loads that name it as their own `Load::breaker`), a
    /// feeder breaker's current is the *whole bus's* total draw -- every
    /// load on it regardless of their own breaker, plus that bus's own
    /// [`BusFaults::short_to_ground`] current -- matching a real A380 bus-
    /// tie/feeder breaker (`breakers.rs`'s own "AC1 BUS FEED"-class
    /// entries), which protects the feeder itself, not one consumer.
    feeder_breakers: Vec<(usize, BusId)>,
    tick: u64,
}

impl Network {
    pub fn new() -> Self {
        Self {
            buses: ALL_BUS_IDS.iter().map(|&id| Bus::new(id)).collect(),
            sources: Vec::new(),
            contactors: Vec::new(),
            diodes: Vec::new(),
            breakers: Vec::new(),
            loads: Vec::new(),
            feeder_breakers: Vec::new(),
            tick: 0,
        }
    }

    pub fn bus(&self, id: BusId) -> &Bus {
        &self.buses[id.index()]
    }

    pub fn set_bus_fault(&mut self, id: BusId, short_to_ground: f64) {
        self.buses[id.index()].faults.short_to_ground = short_to_ground.clamp(0.0, 1.0);
    }

    pub fn add_source(&mut self, source: Source) -> usize {
        self.sources.push(source);
        self.sources.len() - 1
    }

    pub fn add_contactor(&mut self, contactor: Contactor) -> usize {
        self.contactors.push(contactor);
        self.contactors.len() - 1
    }

    pub fn add_diode(&mut self, diode: Diode) -> usize {
        self.diodes.push(diode);
        self.diodes.len() - 1
    }

    pub fn add_breaker(&mut self, breaker: Breaker) -> usize {
        self.breakers.push(breaker);
        self.breakers.len() - 1
    }

    /// Registers a whole-bus feeder/tie breaker (see [`Network::feeder_breakers`]'s
    /// own doc).
    pub fn add_feeder_breaker(&mut self, breaker: Breaker, bus: BusId) -> usize {
        let idx = self.add_breaker(breaker);
        self.feeder_breakers.push((idx, bus));
        idx
    }

    pub fn add_load(&mut self, spec: LoadSpec, breaker: usize) -> usize {
        self.loads.push(Load::new(spec, breaker));
        self.loads.len() - 1
    }

    /// Registers a load with two or more independent power feeds (see
    /// [`LoadFeed`]'s own doc) -- every real dual/triple-fed A380 LRU.
    pub fn add_load_multi_feed(&mut self, spec: LoadSpec, feeds: Vec<LoadFeed>) -> usize {
        self.loads.push(Load::new_multi_feed(spec, feeds));
        self.loads.len() - 1
    }

    pub fn contactor_index(&self, id: &str) -> Option<usize> {
        self.contactors.iter().position(|c| c.id == id)
    }

    pub fn breaker_index(&self, id: &str) -> Option<usize> {
        self.breakers.iter().position(|b| b.id == id)
    }

    pub fn load_index(&self, id: &str) -> Option<usize> {
        self.loads.iter().position(|l| l.spec.id == id)
    }

    pub fn diode_index(&self, id: &str) -> Option<usize> {
        self.diodes.iter().position(|d| d.id == id)
    }

    pub fn command_contactor(&mut self, id: &str, closed: bool) {
        if let Some(c) = self.contactors.iter_mut().find(|c| c.id == id) {
            c.commanded_closed = closed;
        }
    }

    pub fn set_source(&mut self, index: usize, open_circuit_v: f64, resistance_ohm: f64, frequency_hz: f64) {
        if let Some(s) = self.sources.get_mut(index) {
            s.open_circuit_v = open_circuit_v.max(0.0);
            s.resistance_ohm = resistance_ohm.max(MIN_RESISTANCE_OHM);
            s.frequency_hz = frequency_hz;
        }
    }

    /// Advance the whole network one tick: resolve contactors/diodes, solve
    /// every bus's voltage (see this module's own doc comment for the
    /// physics), commit every load's current and every breaker's thermal/
    /// trip state. Safe at `dt_s = 0` (no heat/energised-time integration,
    /// but the resistive solve itself is instantaneous) and with nothing
    /// connected at all (every bus solves to 0 V, no NaN).
    pub fn step(&mut self, dt_s: f64) -> NetworkReport {
        let dt = dt_s.max(0.0);
        self.tick = self.tick.wrapping_add(1);
        let tick = self.tick;

        for c in &mut self.contactors {
            c.resolve(tick);
        }

        let mut voltage = [0.0f64; NUM_BUSES];
        for (i, b) in self.buses.iter().enumerate() {
            voltage[i] = if b.voltage > 0.0 { b.voltage } else { b.id.nominal_voltage() };
        }

        // Frequency depends only on which sources reach which buses through
        // closed contactors/diodes (pure topology), never on load current,
        // so it is resolved once up front and held fixed through every
        // relaxation sweep below -- unlike voltage, it has no circular
        // dependency on the load aggregation to converge.
        let frequency = self.resolve_frequency();

        for _ in 0..ITERATIONS {
            voltage = self.relax(&voltage, &frequency, dt, tick);
        }

        for i in 0..NUM_BUSES {
            self.buses[i].voltage = voltage[i];
        }
        self.commit_frequency(&voltage, &frequency);

        let mut breaker_current = vec![0.0f64; self.breakers.len()];
        for i in 0..self.loads.len() {
            let out = self.loads[i].step(&voltage, &frequency, &self.breakers, dt, tick);
            if let Some(feed_idx) = self.loads[i].active_feed {
                let bkr = self.loads[i].feeds[feed_idx].breaker;
                breaker_current[bkr] += out.current_a;
            }
        }
        for &(idx, bus) in &self.feeder_breakers {
            let bus_idx = bus.index();
            let mut total = 0.0;
            for load in &self.loads {
                let on_this_bus = matches!(load.active_feed, Some(fi) if load.feeds[fi].bus == bus);
                if on_this_bus {
                    total += load.current_a;
                }
            }
            let b = &self.buses[bus_idx];
            let short = b.faults.short_to_ground.clamp(0.0, 1.0);
            if short > 0.0 {
                total += (short / BUS_FAULT_RESISTANCE_OHM) * b.voltage;
            }
            breaker_current[idx] = total;
        }

        let mut tripped = Vec::new();
        for (i, b) in self.breakers.iter_mut().enumerate() {
            let was_closed = b.closed;
            b.step(breaker_current[i], dt, tick);
            if was_closed && !b.closed {
                tripped.push(b.id);
            }
        }

        let mut report = NetworkReport::default();
        let mut total_power = 0.0;
        for i in 0..NUM_BUSES {
            report.bus_voltage[i] = self.buses[i].voltage;
            report.bus_powered[i] = self.buses[i].voltage >= self.buses[i].id.nominal_voltage() * POWERED_VOLTAGE_FRACTION;
        }
        for load in &self.loads {
            if let Some(fi) = load.active_feed {
                let bus_idx = load.feeds[fi].bus.index();
                total_power += load.spec.power_factor * load.current_a * self.buses[bus_idx].voltage;
            }
        }
        report.total_power_w = total_power;
        report.tripped_breakers = tripped;
        report
    }

    /// One Jacobi relaxation sweep: aggregate every bus's load demand at the
    /// given trial `voltage`, form every bus's Thevenin-equivalent supply
    /// from closed contactors/diodes (using the same trial voltages for
    /// neighbouring buses), and solve each bus's own voltage against its
    /// aggregate demand. See this module's own doc comment for the full
    /// derivation.
    fn relax(&self, voltage: &[f64; NUM_BUSES], frequency: &[f64; NUM_BUSES], dt: f64, tick: u64) -> [f64; NUM_BUSES] {
        let mut agg_p = [0.0f64; NUM_BUSES];
        let mut agg_g = [0.0f64; NUM_BUSES];
        for load in &self.loads {
            let Some((feed_idx, _v)) = load.select_feed(voltage, &self.breakers) else {
                continue;
            };
            let idx = load.feeds[feed_idx].bus.index();
            let (p, g) = load.contribution(voltage[idx], frequency[idx], dt, tick);
            agg_p[idx] += p;
            agg_g[idx] += g;
        }
        for bus in &self.buses {
            let short = bus.faults.short_to_ground.clamp(0.0, 1.0);
            if short > 0.0 {
                agg_g[bus.id.index()] += short / BUS_FAULT_RESISTANCE_OHM;
            }
        }

        let mut vth = [0.0f64; NUM_BUSES];
        let mut yth = [0.0f64; NUM_BUSES];
        for c in &self.contactors {
            if !c.closed {
                continue;
            }
            match c.from {
                FeedSource::Source(i) => {
                    if let Some(src) = self.sources.get(i) {
                        // The path resistance is the *source's own* internal
                        // (Thevenin) resistance in series with the
                        // contactor's own contact/wiring resistance -- two
                        // distinct physical resistances on the same path,
                        // not alternatives. Dropping either one (e.g. only
                        // ever reading the contactor's) would silently make
                        // a source's own internal resistance invisible to
                        // every bus it feeds.
                        let r = (c.resistance_ohm + src.resistance_ohm).max(MIN_RESISTANCE_OHM);
                        let b = c.to.index();
                        vth[b] += src.open_circuit_v / r;
                        yth[b] += 1.0 / r;
                    }
                }
                FeedSource::Bus(a) => {
                    let r = c.resistance_ohm.max(MIN_RESISTANCE_OHM);
                    let ai = a.index();
                    let bi = c.to.index();
                    vth[bi] += voltage[ai] / r;
                    yth[bi] += 1.0 / r;
                    vth[ai] += voltage[bi] / r;
                    yth[ai] += 1.0 / r;
                }
            }
        }
        for d in &self.diodes {
            if d.open_fault(tick) {
                continue;
            }
            let (from_v, source_r) = match d.from {
                FeedSource::Source(i) => self.sources.get(i).map_or((0.0, 0.0), |s| (s.open_circuit_v, s.resistance_ohm)),
                FeedSource::Bus(b) => (voltage[b.index()], 0.0),
            };
            let biased = from_v - d.forward_drop_v;
            if biased > 0.0 {
                // Same series combination as the contactor case above: a
                // diode fed directly from a `Source` must include that
                // source's own internal resistance, not only the diode's.
                let r = (d.resistance_ohm + source_r).max(MIN_RESISTANCE_OHM);
                let bi = d.to.index();
                vth[bi] += biased / r;
                yth[bi] += 1.0 / r;
            }
        }

        let mut next = [0.0f64; NUM_BUSES];
        for i in 0..NUM_BUSES {
            if yth[i] <= MIN_ADMITTANCE {
                next[i] = 0.0;
                continue;
            }
            let rth = 1.0 / yth[i];
            let vth_i = vth[i] * rth;
            let a = 1.0 + rth * agg_g[i];
            let p = agg_p[i];
            let disc = vth_i * vth_i - 4.0 * a * rth * p;
            next[i] = if disc < 0.0 { (vth_i / (2.0 * a)).max(0.0) } else { ((vth_i + disc.sqrt()) / (2.0 * a)).max(0.0) };
        }
        next
    }

    /// A real, propagated quantity now (not merely informational): a bus fed
    /// directly (one hop) by a closed generator-line contactor from a
    /// [`Source`] takes that source's own frequency; a bus fed only via a
    /// tie inherits its neighbour's already-resolved frequency, propagated
    /// up to [`FREQUENCY_PASSES`] hops. Pure topology -- independent of any
    /// bus's voltage/load current, so it can (and must) be resolved once
    /// *before* the voltage relaxation sweep, whose own per-tick load
    /// aggregation now needs it for [`Load::frequency_multiplier`]
    /// (`Network::relax`'s own call). Deliberately not a real synchronising-
    /// before-paralleling model (that logic lives upstream, in whichever
    /// generator-control model owns the contactor's own close command).
    fn resolve_frequency(&self) -> [f64; NUM_BUSES] {
        let mut freq = [0.0f64; NUM_BUSES];
        for _ in 0..FREQUENCY_PASSES {
            for c in &self.contactors {
                if !c.closed {
                    continue;
                }
                match c.from {
                    FeedSource::Source(i) => {
                        if let Some(src) = self.sources.get(i) {
                            if src.frequency_hz > 0.0 {
                                freq[c.to.index()] = src.frequency_hz;
                            }
                        }
                    }
                    FeedSource::Bus(a) => {
                        let ai = a.index();
                        let bi = c.to.index();
                        if freq[ai] > 0.0 && freq[bi] <= 0.0 {
                            freq[bi] = freq[ai];
                        } else if freq[bi] > 0.0 && freq[ai] <= 0.0 {
                            freq[ai] = freq[bi];
                        }
                    }
                }
            }
        }
        freq
    }

    /// Applies the "is this bus actually powered" gate to the resolved
    /// frequency (a bus with no live source at all should read 0 Hz, not
    /// whatever stale topology guess `resolve_frequency` produced for it)
    /// and stores the result on each [`Bus`] for external reporting.
    fn commit_frequency(&mut self, voltage: &[f64; NUM_BUSES], freq: &[f64; NUM_BUSES]) {
        for i in 0..NUM_BUSES {
            let bus_id = self.buses[i].id;
            self.buses[i].frequency_hz = if bus_id.is_ac() && voltage[i] >= bus_id.nominal_voltage() * POWERED_VOLTAGE_FRACTION { freq[i] } else { 0.0 };
        }
    }
}

impl Default for Network {
    fn default() -> Self {
        Self::new()
    }
}

/// A convenience index some `loads.rs`/`sources.rs`/`shedding.rs` callers
/// want: every load currently on a given bus, by index into
/// [`Network::loads`].
pub fn loads_on_bus(network: &Network, bus: BusId) -> Vec<usize> {
    network.loads.iter().enumerate().filter(|(_, l)| l.spec.bus == bus).map(|(i, _)| i).collect()
}

/// A convenience id->index map builder for a large static catalogue
/// (`loads.rs` uses this once at startup rather than every caller doing its
/// own linear scan).
pub fn index_by_id<'a, T>(items: &'a [T], id_of: impl Fn(&'a T) -> &'static str) -> HashMap<&'static str, usize> {
    items.iter().enumerate().map(|(i, item)| (id_of(item), i)).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn network_with_one_source_and_load(open_circuit_v: f64, source_r: f64, load_w: f64) -> (Network, usize, usize, usize) {
        let mut net = Network::new();
        let src = net.add_source(Source { id: "src", open_circuit_v, resistance_ohm: source_r, frequency_hz: 0.0 });
        let gen_contactor = net.add_contactor(Contactor::new("gen-line", ContactorKind::GeneratorLine, FeedSource::Source(src), BusId::Dc1, 0.01));
        net.contactors[gen_contactor].commanded_closed = true;
        let breaker = net.add_breaker(Breaker::new("bkr", 100.0, BusId::Dc1));
        let spec = LoadSpec {
            id: "load",
            name: "Test Load",
            ata: 24,
            bus: BusId::Dc1,
            rated_power_w: load_w,
            power_factor: 1.0,
            min_operating_voltage: 0.0,
            inrush_multiple: 1.0,
            inrush_duration_s: 0.0,
            wiring_resistance_ohm: 0.05,
            rated_frequency_hz: 0.0,
            basis: "test",
        };
        let load = net.add_load(spec, breaker);
        (net, src, breaker, load)
    }

    #[test]
    fn a_single_source_and_load_matches_the_closed_form_quadratic() {
        let (mut net, _src, _bkr, load) = network_with_one_source_and_load(28.0, 0.02, 300.0);
        for _ in 0..5 {
            net.step(1.0 / 60.0);
        }
        let v = net.bus(BusId::Dc1).voltage;
        // Thevenin resistance here is the *source's own* 0.02 ohm in series
        // with `network_with_one_source_and_load`'s own generator-line
        // contactor (0.01 ohm, hard-coded in that helper) -- both are real,
        // distinct series resistances on the same path (`Network::relax`'s
        // own doc on the `FeedSource::Source` case): total 0.03 ohm.
        // V^2 - 28*V + 0.03*300 = 0.
        const TOTAL_R: f64 = 0.02 + 0.01;
        let expected = (28.0 + (28.0f64.powi(2) - 4.0 * TOTAL_R * 300.0).sqrt()) / 2.0;
        assert!((v - expected).abs() < 0.05, "solved {v}, expected {expected}");
        let expected_current = 300.0 / expected;
        assert!((net.loads[load].current_a - expected_current).abs() < 0.05);
    }

    #[test]
    fn nothing_connected_never_produces_nan_and_reads_zero() {
        let mut net = Network::new();
        let report = net.step(1.0 / 60.0);
        for v in report.bus_voltage {
            assert!(v.is_finite());
            assert_eq!(v, 0.0);
        }
        assert_eq!(report.total_power_w, 0.0);
        // Also safe at dt = 0.
        let report0 = net.step(0.0);
        assert!(report0.bus_voltage.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn an_open_breaker_cuts_load_current_to_zero() {
        let (mut net, _src, bkr, load) = network_with_one_source_and_load(28.0, 0.02, 300.0);
        net.breakers[bkr].pull();
        net.step(1.0 / 60.0);
        assert_eq!(net.loads[load].current_a, 0.0);
        assert!(!net.loads[load].powered);
    }

    #[test]
    fn a_dead_short_trips_the_magnetic_curve_within_one_tick() {
        let (mut net, _src, bkr, load) = network_with_one_source_and_load(28.0, 0.02, 300.0);
        net.loads[load].faults.short_to_ground = 1.0;
        net.loads[load].spec.wiring_resistance_ohm = 0.001; // a genuine near-zero-impedance short
        net.step(1.0 / 60.0);
        assert!(!net.breakers[bkr].closed, "expected an instant magnetic trip");
        assert_eq!(net.breakers[bkr].trip_cause, Some(TripCause::Magnetic));
    }

    #[test]
    fn a_moderate_overload_trips_thermally_near_its_predicted_time() {
        // Rated 100 A breaker; the total path resistance is 0.001
        // (source) + 0.01 (the helper's own generator-line contactor) =
        // 0.011 ohm. Solving V = 28 - 0.011*I for the load's own constant-
        // power relation at the target I = 200 A (2x rated) gives V = 25.8
        // and P = I*V = 5160 W -- engineered so the solved current lands
        // almost exactly on 2x, where K/(2^2-1) predicts a ~10 s trip.
        let (mut net, _src, bkr, _load) = network_with_one_source_and_load(28.0, 0.001, 5160.0);
        const DT: f64 = 1.0 / 60.0;
        let mut elapsed = 0.0;
        let mut tripped_at = None;
        while elapsed < 30.0 {
            net.step(DT);
            elapsed += DT;
            if !net.breakers[bkr].closed {
                tripped_at = Some(elapsed);
                break;
            }
        }
        let t = tripped_at.expect("expected a thermal trip within 30s");
        assert!((t - 10.0).abs() < 1.0, "expected ~10s, got {t}s");
        assert_eq!(net.breakers[bkr].trip_cause, Some(TripCause::Thermal));
    }

    #[test]
    fn a_breaker_that_always_fails_to_trip_stays_closed_under_sustained_overload() {
        let (mut net, _src, bkr, _load) = network_with_one_source_and_load(28.0, 0.001, 5600.0);
        net.breakers[bkr].faults.fails_to_trip = 1.0;
        for _ in 0..(60 * 30) {
            net.step(1.0 / 60.0);
        }
        assert!(net.breakers[bkr].closed, "a jammed breaker should never actually open");
        assert!(net.breakers[bkr].heat_fraction() > 0.9, "should sit right at the trip threshold");
    }

    #[test]
    fn a_welded_contactor_stays_closed_even_when_commanded_open() {
        let (mut net, _src, _bkr, load) = network_with_one_source_and_load(28.0, 0.02, 100.0);
        let gc = net.contactor_index("gen-line").unwrap();
        net.contactors[gc].faults.welded_closed = 1.0;
        net.contactors[gc].commanded_closed = false;
        net.step(1.0 / 60.0);
        assert!(net.contactors[gc].closed);
        assert!(net.loads[load].powered, "the bus should still be live through the welded contactor");
    }

    #[test]
    fn a_contactor_that_always_fails_to_close_never_energises_its_bus() {
        let (mut net, _src, _bkr, load) = network_with_one_source_and_load(28.0, 0.02, 100.0);
        let gc = net.contactor_index("gen-line").unwrap();
        net.contactors[gc].faults.fails_to_close = 1.0;
        net.step(1.0 / 60.0);
        assert!(!net.contactors[gc].closed);
        assert_eq!(net.bus(BusId::Dc1).voltage, 0.0);
        assert!(!net.loads[load].powered);
    }

    #[test]
    fn a_bus_tie_carries_power_to_a_bus_with_no_source_of_its_own() {
        let mut net = Network::new();
        let src = net.add_source(Source { id: "src", open_circuit_v: 28.0, resistance_ohm: 0.02, frequency_hz: 0.0 });
        let gc = net.add_contactor(Contactor::new("gen-line", ContactorKind::GeneratorLine, FeedSource::Source(src), BusId::Dc1, 0.01));
        net.contactors[gc].commanded_closed = true;
        let tie = net.add_contactor(Contactor::new("tie", ContactorKind::BusTie, FeedSource::Bus(BusId::Dc1), BusId::Dc2, 0.02));
        net.contactors[tie].commanded_closed = true;
        let bkr = net.add_breaker(Breaker::new("bkr2", 50.0, BusId::Dc2));
        let spec = LoadSpec {
            id: "load2",
            name: "Tied Load",
            ata: 24,
            bus: BusId::Dc2,
            rated_power_w: 100.0,
            power_factor: 1.0,
            min_operating_voltage: 0.0,
            inrush_multiple: 1.0,
            inrush_duration_s: 0.0,
            wiring_resistance_ohm: 0.05,
            rated_frequency_hz: 0.0,
            basis: "test",
        };
        let load = net.add_load(spec, bkr);
        for _ in 0..5 {
            net.step(1.0 / 60.0);
        }
        assert!(net.bus(BusId::Dc2).voltage > 20.0, "tie should carry real voltage across, got {}", net.bus(BusId::Dc2).voltage);
        assert!(net.loads[load].powered);
        // Opening the tie should de-energise the far bus (no other source).
        net.contactors[tie].commanded_closed = false;
        for _ in 0..5 {
            net.step(1.0 / 60.0);
        }
        assert_eq!(net.bus(BusId::Dc2).voltage, 0.0);
    }

    #[test]
    fn a_diode_does_not_backfeed_the_lower_side_into_the_higher_one() {
        let mut net = Network::new();
        let strong = net.add_source(Source { id: "strong", open_circuit_v: 28.0, resistance_ohm: 0.01, frequency_hz: 0.0 });
        let weak = net.add_source(Source { id: "weak", open_circuit_v: 10.0, resistance_ohm: 0.01, frequency_hz: 0.0 });
        let gc1 = net.add_contactor(Contactor::new("gc1", ContactorKind::GeneratorLine, FeedSource::Source(strong), BusId::DcBat, 0.01));
        net.contactors[gc1].commanded_closed = true;
        let gc2 = net.add_contactor(Contactor::new("gc2", ContactorKind::GeneratorLine, FeedSource::Source(weak), BusId::DcHot1, 0.01));
        net.contactors[gc2].commanded_closed = true;
        // Diode from the weak DcHot1 bus toward the strong DcBat bus should
        // never conduct (reverse-biased): DcBat must stay near 28 V, not
        // get dragged toward 10 V.
        net.add_diode(Diode::new("d", FeedSource::Bus(BusId::DcHot1), BusId::DcBat, 0.7, 0.01));
        for _ in 0..5 {
            net.step(1.0 / 60.0);
        }
        assert!(net.bus(BusId::DcBat).voltage > 25.0, "reverse-biased diode should not have loaded down DcBat: {}", net.bus(BusId::DcBat).voltage);
    }

    #[test]
    fn a_diode_forward_feeds_with_its_own_drop() {
        let mut net = Network::new();
        let src = net.add_source(Source { id: "src", open_circuit_v: 28.0, resistance_ohm: 0.01, frequency_hz: 0.0 });
        let gc = net.add_contactor(Contactor::new("gc", ContactorKind::GeneratorLine, FeedSource::Source(src), BusId::DcBat, 0.01));
        net.contactors[gc].commanded_closed = true;
        net.add_diode(Diode::new("d", FeedSource::Bus(BusId::DcBat), BusId::DcHot1, 1.0, 0.01));
        for _ in 0..5 {
            net.step(1.0 / 60.0);
        }
        let hot1 = net.bus(BusId::DcHot1).voltage;
        let bat = net.bus(BusId::DcBat).voltage;
        assert!(hot1 > 0.0 && hot1 < bat, "hot bus {hot1} should be fed but below the battery bus {bat} by roughly the diode drop");
    }

    #[test]
    fn a_bus_fault_collapses_its_own_voltage_and_can_trip_its_feeder() {
        // A real feeder/tie breaker (`add_feeder_breaker`) measures the
        // *whole bus's* current, including its own fault current -- unlike
        // an individual load's own small breaker, which never sees a fault
        // that is not routed through it at all.
        let mut net = Network::new();
        let src = net.add_source(Source { id: "src", open_circuit_v: 28.0, resistance_ohm: 0.02, frequency_hz: 0.0 });
        let gc = net.add_contactor(Contactor::new("gen-line", ContactorKind::GeneratorLine, FeedSource::Source(src), BusId::Dc1, 0.01));
        net.contactors[gc].commanded_closed = true;
        let feeder = net.add_feeder_breaker(Breaker::new("feeder", 50.0, BusId::Dc1), BusId::Dc1);
        net.set_bus_fault(BusId::Dc1, 1.0);
        for _ in 0..(60 * 15) {
            net.step(1.0 / 60.0);
        }
        assert!(net.bus(BusId::Dc1).voltage < 15.0, "a dead bus short should sag the bus hard: {}", net.bus(BusId::Dc1).voltage);
        assert!(!net.breakers[feeder].closed, "the feeder breaker should trip on the bus fault current");
    }

    #[test]
    fn a_feeder_breaker_sums_every_load_on_its_bus_not_just_one() {
        let mut net = Network::new();
        let src = net.add_source(Source { id: "src", open_circuit_v: 28.0, resistance_ohm: 0.01, frequency_hz: 0.0 });
        let gc = net.add_contactor(Contactor::new("gen-line", ContactorKind::GeneratorLine, FeedSource::Source(src), BusId::Dc1, 0.01));
        net.contactors[gc].commanded_closed = true;
        let feeder = net.add_feeder_breaker(Breaker::new("feeder", 100.0, BusId::Dc1), BusId::Dc1);
        for i in 0..3 {
            let id: &'static str = Box::leak(format!("l{i}").into_boxed_str());
            let bkr = net.add_breaker(Breaker::new(id, 50.0, BusId::Dc1));
            let spec = LoadSpec { id, name: id, ata: 24, bus: BusId::Dc1, rated_power_w: 100.0, power_factor: 1.0, min_operating_voltage: 0.0, inrush_multiple: 1.0, inrush_duration_s: 0.0, wiring_resistance_ohm: 0.05, rated_frequency_hz: 0.0, basis: "test" };
            net.add_load(spec, bkr);
        }
        for _ in 0..5 {
            net.step(1.0 / 60.0);
        }
        let sum_of_loads: f64 = net.loads.iter().map(|l| l.current_a).sum();
        assert!((net.breakers[feeder].current_a - sum_of_loads).abs() < 1e-6, "feeder current {} should equal the sum of all three loads {}", net.breakers[feeder].current_a, sum_of_loads);
    }

    #[test]
    fn undervoltage_drops_a_load_before_full_bus_death() {
        let (mut net, src, _bkr, load) = network_with_one_source_and_load(28.0, 0.02, 50.0);
        net.loads[load].spec.min_operating_voltage = 26.0;
        net.step(1.0 / 60.0);
        assert!(net.loads[load].powered);
        // Sag the source hard (a failing generator) without killing the bus
        // outright.
        net.set_source(src, 20.0, 0.02, 0.0);
        for _ in 0..5 {
            net.step(1.0 / 60.0);
        }
        assert!(net.bus(BusId::Dc1).voltage > 0.0, "bus should still have some voltage");
        assert!(!net.loads[load].powered, "load should have dropped out below its own min operating voltage");
    }

    #[test]
    fn inrush_current_decays_toward_the_steady_state_value() {
        let (mut net, _src, _bkr, load) = network_with_one_source_and_load(28.0, 0.01, 200.0);
        net.loads[load].spec.inrush_multiple = 4.0;
        net.loads[load].spec.inrush_duration_s = 0.3;
        net.step(1.0 / 60.0);
        let first_tick_current = net.loads[load].current_a;
        for _ in 0..120 {
            net.step(1.0 / 60.0);
        }
        let steady_current = net.loads[load].current_a;
        assert!(first_tick_current > steady_current * 1.5, "expected a real inrush spike: first {first_tick_current} A vs steady {steady_current} A");
    }

    #[test]
    fn an_intermittent_fault_at_full_severity_reliably_drops_the_load() {
        let (mut net, _src, _bkr, load) = network_with_one_source_and_load(28.0, 0.02, 50.0);
        net.loads[load].faults.intermittent = 1.0;
        // A huge dt drives the dropout probability to exactly 1.0 (exp
        // underflow), which `bernoulli`'s own `p >= 1.0` branch returns
        // deterministically -- no flakiness from the underlying hash.
        net.step(1000.0);
        assert!(!net.loads[load].powered);
        assert_eq!(net.loads[load].current_a, 0.0);
    }

    #[test]
    fn an_open_circuit_fault_at_full_severity_draws_no_current() {
        let (mut net, _src, _bkr, load) = network_with_one_source_and_load(28.0, 0.02, 200.0);
        net.loads[load].faults.open_circuit = 1.0;
        net.step(1.0 / 60.0);
        assert_eq!(net.loads[load].current_a, 0.0);
        assert!(!net.loads[load].powered);
    }

    #[test]
    fn a_high_resistance_fault_draws_more_current_for_the_same_load() {
        let (mut healthy_net, _s1, _b1, healthy_load) = network_with_one_source_and_load(28.0, 0.02, 300.0);
        healthy_net.step(1.0 / 60.0);
        let healthy_current = healthy_net.loads[healthy_load].current_a;

        let (mut degraded_net, _s2, _b2, degraded_load) = network_with_one_source_and_load(28.0, 0.02, 300.0);
        degraded_net.loads[degraded_load].faults.high_resistance = 1.0;
        degraded_net.step(1.0 / 60.0);
        let degraded_current = degraded_net.loads[degraded_load].current_a;

        assert!(degraded_current > healthy_current, "high-resistance fault should draw more current: {degraded_current} vs {healthy_current}");
    }

    #[test]
    fn a_short_to_ground_current_is_limited_by_wiring_resistance() {
        let (mut net, _src, _bkr, load) = network_with_one_source_and_load(28.0, 0.02, 10.0);
        net.loads[load].faults.short_to_ground = 1.0;
        net.loads[load].spec.wiring_resistance_ohm = 2.0;
        net.step(1.0 / 60.0);
        let v = net.bus(BusId::Dc1).voltage;
        let expected_fault_current = v / 2.0;
        assert!((net.loads[load].current_a - expected_fault_current).abs() < 0.5, "current {} should be close to V/R = {}", net.loads[load].current_a, expected_fault_current);
    }

    #[test]
    fn network_converges_within_its_own_iteration_budget() {
        // A three-bus tie chain: source -> Dc1 -> (tie) -> Dc2 -> (tie) ->
        // DcEss, each with its own load. If `ITERATIONS` were too small this
        // would settle to a visibly different (higher) voltage on `DcEss`
        // than running many more sweeps by hand would; check it matches a
        // manually-iterated 200-sweep reference within tight tolerance.
        let mut net = Network::new();
        let src = net.add_source(Source { id: "src", open_circuit_v: 28.0, resistance_ohm: 0.01, frequency_hz: 0.0 });
        let gc = net.add_contactor(Contactor::new("gc", ContactorKind::GeneratorLine, FeedSource::Source(src), BusId::Dc1, 0.01));
        net.contactors[gc].commanded_closed = true;
        let tie1 = net.add_contactor(Contactor::new("tie1", ContactorKind::BusTie, FeedSource::Bus(BusId::Dc1), BusId::Dc2, 0.02));
        net.contactors[tie1].commanded_closed = true;
        let tie2 = net.add_contactor(Contactor::new("tie2", ContactorKind::BusTie, FeedSource::Bus(BusId::Dc2), BusId::DcEss, 0.02));
        net.contactors[tie2].commanded_closed = true;
        for (id, bus) in [("b1", BusId::Dc1), ("b2", BusId::Dc2), ("b3", BusId::DcEss)] {
            let bkr = net.add_breaker(Breaker::new(id, 50.0, bus));
            let spec = LoadSpec { id, name: id, ata: 24, bus, rated_power_w: 80.0, power_factor: 1.0, min_operating_voltage: 0.0, inrush_multiple: 1.0, inrush_duration_s: 0.0, wiring_resistance_ohm: 0.05, rated_frequency_hz: 0.0, basis: "test" };
            net.add_load(spec, bkr);
        }
        for _ in 0..10 {
            net.step(1.0 / 60.0);
        }
        let v = net.bus(BusId::DcEss).voltage;
        assert!(v.is_finite() && v > 0.0);
        // A second, independent run with many more relax() sweeps folded in
        // manually should land within a small tolerance.
        let mut voltage = [28.0f64; NUM_BUSES];
        let frequency = [0.0f64; NUM_BUSES];
        for _ in 0..300 {
            voltage = net.relax(&voltage, &frequency, 1.0 / 60.0, 999);
        }
        assert!((voltage[BusId::DcEss.index()] - v).abs() < 0.1, "extra sweeps should not move the answer much: {} vs {}", voltage[BusId::DcEss.index()], v);
    }
}
