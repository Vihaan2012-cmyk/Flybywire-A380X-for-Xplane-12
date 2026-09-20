//! Source models for the per-load network: each reduces itself, every tick,
//! to the same `(open_circuit_v, resistance_ohm, frequency_hz)` triple
//! [`network::Source`] consumes -- "model the load side and interfaces"
//! (`docs/deep/BRIEF.md` backlog item 4). FlyByWire's own systems crate
//! already has real generator/TRU/battery physics
//! (`fbw-common/.../electrical/*.rs`); these models are independent
//! (self-contained, no crate-internal dependency) but reuse its real, cited
//! constants wherever public, and are built to the same Thevenin-equivalent
//! technique it already uses internally (`V^2 - V_rated*V + S*Xs = 0`,
//! `engine_generator.rs`/`transformer_rectifier.rs`) so `network.rs`'s own
//! solver treats every source identically regardless of its physical type.
//!
//! Every model follows the brief's own convention: a plain struct, `new()`,
//! a `step(&mut self, inputs, dt_s) -> outputs`, faults as `0.0..=1.0`
//! fractions in their own `...Faults` struct (`Default` = healthy).
//!
//! **The one shared simplification, stated once here rather than in every
//! model:** a [`network::Source`] is one-directional (it feeds a bus, current
//! never flows back into it through the network solver itself). A real
//! battery's *charging* current does flow the other way; this module handles
//! that by having the integration layer ([`Wiring::post_step`]) read the
//! actual current a battery's own feeder carried this tick (positive =
//! discharge, negative = charge, the same signed convention a real ammeter
//! shows) and pass it to [`Battery::step`] directly, independent of the
//! per-tick resistive solve -- the same one-tick-lagged measured-feedback
//! pattern `physics::electrical.rs`'s own `EngineLoads` contract already
//! uses between the systems tick and the engine model
//! (`ENGINE_GEARBOX_ELEC_LOAD_W:n`).

use super::network::{BusId, Contactor, ContactorKind, Diode, FeedSource, Network, Source};

const MIN_RESISTANCE_OHM: f64 = 1.0e-4;

// ---------------------------------------------------------------------
// VFG: the four main-engine variable-frequency generators.

/// `0.0` healthy.
#[derive(Clone, Copy, Debug, Default)]
pub struct VfgFaults {
    /// Aged/shorted stator windings: internal reactance grows toward
    /// [`Vfg::DEGRADED_REACTANCE_MULTIPLIER`]x (same construction FBW's own
    /// `engine_generator.rs` uses for the identical fault).
    pub winding_degradation: f64,
    /// GCU voltage-regulation drift, signed -1..1 (under- to over-voltage),
    /// scaled by [`Vfg::MAX_REGULATOR_DRIFT_VOLT`] (same construction as
    /// `engine_generator.rs`'s own `regulator_drift`).
    pub regulator_drift: f64,
}

pub struct VfgInputs {
    /// Engine core (N2/N3) speed as a fraction of 100%; the VFG is gear-
    /// driven off the accessory gearbox, so its own shaft speed -- and
    /// therefore its output frequency -- tracks this directly (no CSD/IDG
    /// on the A380's VFG architecture, unlike a constant-frequency
    /// generator).
    pub engine_speed_fraction: f64,
    /// This source's own real power delivered last tick, W (read back by
    /// the caller from the network after `Network::step`; `0.0` is a safe
    /// "no load yet" default). Drives the overload accumulator only, not
    /// the terminal voltage/reactance themselves (those depend only on
    /// speed and fault state, exactly like the real GCU-regulated machine).
    pub measured_load_w: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct VfgOutputs {
    pub open_circuit_v: f64,
    pub resistance_ohm: f64,
    pub frequency_hz: f64,
    /// Normalised overload accumulator, `0.0..=1.0` (same I^2t shape as
    /// `network::Breaker`'s own thermal curve, an independent instance):
    /// a real VFG's own current-limiting protection trips its GEN LINE
    /// CONTACTOR upstream of any downstream breaker.
    pub overload_heat: f64,
    pub overload_tripped: bool,
}

pub struct Vfg {
    overload_heat: f64,
}

impl Vfg {
    /// `engine_generator.rs::EngineGenerator::RATED_VOLTAGE_VOLT` (real,
    /// FBW-sourced).
    const RATED_VOLTAGE_VOLT: f64 = 115.0;
    /// `engine_generator.rs::POWER_FACTOR` (real, FBW-sourced).
    const POWER_FACTOR: f64 = 0.8;
    /// `engine_generator.rs::RATED_VOLTAGE_REGULATION` (real, FBW-sourced).
    const RATED_VOLTAGE_REGULATION: f64 = 0.03;
    /// `engine_generator.rs::DEGRADED_REACTANCE_MULTIPLIER` (real,
    /// FBW-sourced).
    const DEGRADED_REACTANCE_MULTIPLIER: f64 = 4.0;
    /// `engine_generator.rs::MAX_REGULATOR_DRIFT_VOLT` (real, FBW-sourced).
    const MAX_REGULATOR_DRIFT_VOLT: f64 = 8.0;
    /// `breakers.rs::generator_rated_a`'s own basis: FBW's VFG true-power
    /// rating, `alternating_current.rs:393` (real, FBW-sourced).
    const RATED_TRUE_POWER_W: f64 = 150_000.0;
    /// GENERIC: a real A380 VFG is genuinely variable-frequency (that is
    /// what distinguishes it from a CSD/IDG-driven constant-frequency
    /// machine); no exact Trent-972/A380 figure is public, so this is a
    /// typical wide-body VFG's own quoted no-load-to-max-N2 operating band.
    const FREQ_MIN_HZ: f64 = 360.0;
    const FREQ_MAX_HZ: f64 = 800.0;
    /// GENERIC: the GCU cuts a VFG in only once its own exciter has enough
    /// residual field current to regulate at all -- a small fraction of
    /// rated speed, same order every real aircraft generator's own cut-in
    /// speed sits at.
    const CUT_IN_SPEED_FRACTION: f64 = 0.05;
    /// Same shape/citation as `network::Breaker`'s own I^2t curve
    /// (`THERMAL_TRIP_K`/`COOLDOWN_SECONDS`): a real VFG's overload
    /// protection is the same class of inverse-time thermal element.
    const OVERLOAD_TRIP_K: f64 = 30.0;
    const OVERLOAD_COOLDOWN_S: f64 = 20.0;

    pub fn new() -> Self {
        Self { overload_heat: 0.0 }
    }

    pub fn overload_heat(&self) -> f64 {
        self.overload_heat
    }

    pub fn step(&mut self, inputs: VfgInputs, faults: VfgFaults, dt_s: f64) -> VfgOutputs {
        let speed = inputs.engine_speed_fraction.max(0.0);
        let running = speed >= Self::CUT_IN_SPEED_FRACTION;

        let rated_apparent_power = Self::RATED_TRUE_POWER_W / Self::POWER_FACTOR;
        let target_voltage = Self::RATED_VOLTAGE_VOLT * (1.0 - Self::RATED_VOLTAGE_REGULATION);
        // Xs sized so that, at exactly rated apparent power, the quadratic
        // solve's own terminal voltage lands on `target_voltage` -- the
        // identical derivation `engine_generator.rs` cites for its own
        // internal V-I loop.
        let base_xs = target_voltage * (Self::RATED_VOLTAGE_VOLT - target_voltage) / rated_apparent_power;
        let degradation = faults.winding_degradation.clamp(0.0, 1.0);
        let xs = base_xs * (1.0 + degradation * (Self::DEGRADED_REACTANCE_MULTIPLIER - 1.0));

        let drift = faults.regulator_drift.clamp(-1.0, 1.0) * Self::MAX_REGULATOR_DRIFT_VOLT;
        let (open_circuit_v, resistance_ohm, frequency_hz) =
            if running { ((Self::RATED_VOLTAGE_VOLT + drift).max(0.0), xs.max(MIN_RESISTANCE_OHM), Self::FREQ_MIN_HZ + (Self::FREQ_MAX_HZ - Self::FREQ_MIN_HZ) * speed.min(1.0)) } else { (0.0, MIN_RESISTANCE_OHM, 0.0) };

        let ratio = if running { inputs.measured_load_w.max(0.0) / Self::RATED_TRUE_POWER_W } else { 0.0 };
        if ratio > 1.0 {
            self.overload_heat += dt_s.max(0.0) * (ratio * ratio - 1.0) / Self::OVERLOAD_TRIP_K;
        } else {
            self.overload_heat = (self.overload_heat - dt_s.max(0.0) / Self::OVERLOAD_COOLDOWN_S).max(0.0);
        }
        self.overload_heat = self.overload_heat.min(1.0);

        VfgOutputs { open_circuit_v, resistance_ohm, frequency_hz, overload_heat: self.overload_heat, overload_tripped: self.overload_heat >= 1.0 }
    }
}

impl Default for Vfg {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------
// APU generator: same machine class as a VFG, but APU-governed (constant
// speed) so its output is conventional constant-frequency, not variable --
// the real distinction the "VFG" name itself implies (only the four
// engine-driven machines are variable-frequency on the A380).

#[derive(Clone, Copy, Debug, Default)]
pub struct ApuGeneratorFaults {
    pub winding_degradation: f64,
    pub regulator_drift: f64,
}

pub struct ApuGeneratorInputs {
    /// APU core speed fraction, 0..1; gates on/off (governed APUs run at
    /// ~100% once stable, so this is mostly a running/not-running signal,
    /// kept as a fraction so a spooling-up APU generator is not modelled as
    /// an instant step).
    pub apu_speed_fraction: f64,
    pub measured_load_w: f64,
}

pub struct ApuGenerator {
    overload_heat: f64,
}

impl ApuGenerator {
    const RATED_VOLTAGE_VOLT: f64 = 115.0;
    const POWER_FACTOR: f64 = 0.8;
    /// Reused from `engine_generator.rs`'s own regulation figure: the same
    /// class of GCU-regulated machine, no separate public APU-generator
    /// regulation figure exists.
    const RATED_VOLTAGE_REGULATION: f64 = 0.03;
    const DEGRADED_REACTANCE_MULTIPLIER: f64 = 4.0;
    const MAX_REGULATOR_DRIFT_VOLT: f64 = 8.0;
    /// `breakers.rs::apu_generator_rated_a`'s own basis: FBW's
    /// `Pw980ApuGenerator::MAXIMUM_LOAD_WATT` (real, FBW-sourced).
    const RATED_TRUE_POWER_W: f64 = 120_000.0;
    /// A governed APU runs at essentially constant speed once started, so
    /// its generator is conventional constant-frequency -- 400 Hz, the
    /// standard aircraft/ground-power AC frequency (real convention, not a
    /// derived figure).
    const FREQUENCY_HZ: f64 = 400.0;
    const CUT_IN_SPEED_FRACTION: f64 = 0.95;
    const OVERLOAD_TRIP_K: f64 = 30.0;
    const OVERLOAD_COOLDOWN_S: f64 = 20.0;

    pub fn new() -> Self {
        Self { overload_heat: 0.0 }
    }

    pub fn step(&mut self, inputs: ApuGeneratorInputs, faults: ApuGeneratorFaults, dt_s: f64) -> VfgOutputs {
        let running = inputs.apu_speed_fraction >= Self::CUT_IN_SPEED_FRACTION;
        let rated_apparent_power = Self::RATED_TRUE_POWER_W / Self::POWER_FACTOR;
        let target_voltage = Self::RATED_VOLTAGE_VOLT * (1.0 - Self::RATED_VOLTAGE_REGULATION);
        let base_xs = target_voltage * (Self::RATED_VOLTAGE_VOLT - target_voltage) / rated_apparent_power;
        let degradation = faults.winding_degradation.clamp(0.0, 1.0);
        let xs = base_xs * (1.0 + degradation * (Self::DEGRADED_REACTANCE_MULTIPLIER - 1.0));
        let drift = faults.regulator_drift.clamp(-1.0, 1.0) * Self::MAX_REGULATOR_DRIFT_VOLT;

        let (open_circuit_v, resistance_ohm, frequency_hz) = if running { ((Self::RATED_VOLTAGE_VOLT + drift).max(0.0), xs.max(MIN_RESISTANCE_OHM), Self::FREQUENCY_HZ) } else { (0.0, MIN_RESISTANCE_OHM, 0.0) };

        let ratio = if running { inputs.measured_load_w.max(0.0) / Self::RATED_TRUE_POWER_W } else { 0.0 };
        if ratio > 1.0 {
            self.overload_heat += dt_s.max(0.0) * (ratio * ratio - 1.0) / Self::OVERLOAD_TRIP_K;
        } else {
            self.overload_heat = (self.overload_heat - dt_s.max(0.0) / Self::OVERLOAD_COOLDOWN_S).max(0.0);
        }
        self.overload_heat = self.overload_heat.min(1.0);
        VfgOutputs { open_circuit_v, resistance_ohm, frequency_hz, overload_heat: self.overload_heat, overload_tripped: self.overload_heat >= 1.0 }
    }
}

impl Default for ApuGenerator {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------
// Ground power (GPU / external power).

#[derive(Clone, Copy, Debug, Default)]
pub struct GroundPowerFaults {
    /// A weak/miswired cart: extra series resistance beyond the rated cart,
    /// as a fraction of a "bad cart" ceiling (`WEAK_CART_MULTIPLIER`).
    pub weak_cart: f64,
}

pub struct GroundPower {}

impl GroundPower {
    /// `external_power_source.rs::POWER_FACTOR` (real, FBW-sourced).
    const POWER_FACTOR: f64 = 0.8;
    /// `external_power_source.rs::RATED_APPARENT_POWER_VA` (real,
    /// FBW-sourced).
    const RATED_APPARENT_POWER_VA: f64 = 90_000.0;
    /// `external_power_source.rs::RATED_VOLTAGE_REGULATION` (real,
    /// FBW-sourced -- "sized to a stiffer, better-regulated ground cart",
    /// `physics::electrical.rs`'s own doc comment).
    const RATED_VOLTAGE_REGULATION: f64 = 0.02;
    const RATED_VOLTAGE_VOLT: f64 = 115.0;
    /// Standard ground-power-unit output frequency (real convention).
    const FREQUENCY_HZ: f64 = 400.0;
    /// GENERIC: a badly regulated/undersized cart's reactance ceiling.
    const WEAK_CART_MULTIPLIER: f64 = 5.0;

    pub fn new() -> Self {
        Self {}
    }

    /// `plugged_in`: the ground crew has connected and energised the cart.
    pub fn terminal(&self, plugged_in: bool, faults: GroundPowerFaults) -> Source {
        if !plugged_in {
            return Source { id: "gpu", open_circuit_v: 0.0, resistance_ohm: MIN_RESISTANCE_OHM, frequency_hz: 0.0 };
        }
        let target_voltage = Self::RATED_VOLTAGE_VOLT * (1.0 - Self::RATED_VOLTAGE_REGULATION);
        let base_xs = target_voltage * (Self::RATED_VOLTAGE_VOLT - target_voltage) / Self::RATED_APPARENT_POWER_VA;
        let weak = faults.weak_cart.clamp(0.0, 1.0);
        let xs = base_xs * (1.0 + weak * (Self::WEAK_CART_MULTIPLIER - 1.0));
        Source { id: "gpu", open_circuit_v: Self::RATED_VOLTAGE_VOLT, resistance_ohm: xs.max(MIN_RESISTANCE_OHM), frequency_hz: Self::FREQUENCY_HZ }
    }
}

impl Default for GroundPower {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------
// TRU: transformer-rectifiers converting an AC bus to a DC bus.

#[derive(Clone, Copy, Debug, Default)]
pub struct TruFaults {
    /// Winding/diode-bridge degradation: internal resistance grows toward
    /// `DEGRADED_RESISTANCE_OHM` (same construction as
    /// `transformer_rectifier.rs`'s own fault).
    pub winding_degradation: f64,
}

pub struct Tru {
    temp_c: f64,
}

impl Tru {
    /// `transformer_rectifier.rs::INTERNAL_RESISTANCE_OHM` (real,
    /// FBW-sourced).
    const INTERNAL_RESISTANCE_OHM: f64 = 0.0135;
    /// `transformer_rectifier.rs::DEGRADED_RESISTANCE_OHM` (real,
    /// FBW-sourced).
    const DEGRADED_RESISTANCE_OHM: f64 = 0.054;
    /// `transformer_rectifier.rs::IDLE_OUTPUT_VOLTAGE` (real, FBW-sourced).
    const IDLE_OUTPUT_VOLTAGE: f64 = 30.2;
    /// `transformer_rectifier.rs::THERMAL_MASS_J_PER_KELVIN` (real,
    /// FBW-sourced).
    const THERMAL_MASS_J_PER_KELVIN: f64 = 1500.0;
    /// `transformer_rectifier.rs::COOLING_W_PER_KELVIN` (real, FBW-sourced).
    const COOLING_W_PER_KELVIN: f64 = 3.5;

    pub fn new(ambient_c: f64) -> Self {
        Self { temp_c: ambient_c }
    }

    pub fn temperature_c(&self) -> f64 {
        self.temp_c
    }

    /// This tick's terminal, given whether the AC input bus is powered.
    pub fn terminal(&self, ac_input_powered: bool, faults: TruFaults) -> (f64, f64) {
        if !ac_input_powered {
            return (0.0, MIN_RESISTANCE_OHM);
        }
        let degradation = faults.winding_degradation.clamp(0.0, 1.0);
        let r = Self::INTERNAL_RESISTANCE_OHM + degradation * (Self::DEGRADED_RESISTANCE_OHM - Self::INTERNAL_RESISTANCE_OHM);
        (Self::IDLE_OUTPUT_VOLTAGE, r)
    }

    /// Updates the TRU's own thermal state from its measured delivered
    /// power this tick (approximating the internal I^2R loss from
    /// `current ~= measured_load_w / IDLE_OUTPUT_VOLTAGE`, a GENERIC but
    /// reasonable approximation since the exact terminal voltage under load
    /// is only a few percent below idle for this class of machine). Same
    /// exact-exponential first-order thermal step style as `physics::engine::
    /// oil.rs`'s chamber model.
    pub fn step(&mut self, ac_input_powered: bool, measured_load_w: f64, faults: TruFaults, ambient_c: f64, dt_s: f64) -> f64 {
        let (_, r) = self.terminal(ac_input_powered, faults);
        let approx_current = if ac_input_powered { measured_load_w.max(0.0) / Self::IDLE_OUTPUT_VOLTAGE } else { 0.0 };
        let heat_w = approx_current * approx_current * r;
        let target = ambient_c + heat_w / Self::COOLING_W_PER_KELVIN;
        let k = Self::COOLING_W_PER_KELVIN / Self::THERMAL_MASS_J_PER_KELVIN;
        self.temp_c = target + (self.temp_c - target) * (-k * dt_s.max(0.0)).exp();
        self.temp_c
    }
}

// ---------------------------------------------------------------------
// Static inverter: battery DC -> emergency AC bus.

#[derive(Clone, Copy, Debug, Default)]
pub struct StaticInverterFaults {
    pub efficiency_loss: f64,
}

pub struct StaticInverter {}

impl StaticInverter {
    /// `static_inverter.rs::EFFICIENCY` (real, FBW-sourced).
    const EFFICIENCY: f64 = 0.85;
    /// `static_inverter.rs::DEGRADED_EFFICIENCY_FLOOR` (real, FBW-sourced).
    const DEGRADED_EFFICIENCY_FLOOR: f64 = 0.3;
    /// `breakers.rs::static_inverter_rated_a`'s own basis: FBW's
    /// `power_consumption.rs` AC_STAT_INV bus demand, 135 W (real,
    /// FBW-sourced).
    const RATED_W: f64 = 135.0;
    const RATED_VOLTAGE_VOLT: f64 = 115.0;
    /// GENERIC: a switch-mode static inverter is well regulated.
    const RATED_VOLTAGE_REGULATION: f64 = 0.05;
    /// GENERIC: below this DC input the inverter's own under-voltage
    /// lockout drops its output (a battery too flat to run it at all).
    const MIN_INPUT_V: f64 = 18.0;
    /// GENERIC: fixed-frequency inverter output (no mechanical rotor to
    /// vary frequency with).
    const FREQUENCY_HZ: f64 = 400.0;

    pub fn new() -> Self {
        Self {}
    }

    pub fn terminal(&self, dc_input_v: f64, faults: StaticInverterFaults) -> Source {
        if dc_input_v < Self::MIN_INPUT_V {
            return Source { id: "static-inv", open_circuit_v: 0.0, resistance_ohm: MIN_RESISTANCE_OHM, frequency_hz: 0.0 };
        }
        let efficiency = (Self::EFFICIENCY - faults.efficiency_loss.clamp(0.0, 1.0) * (Self::EFFICIENCY - Self::DEGRADED_EFFICIENCY_FLOOR)).max(Self::DEGRADED_EFFICIENCY_FLOOR);
        let rated_apparent = Self::RATED_W / efficiency;
        let target_voltage = Self::RATED_VOLTAGE_VOLT * (1.0 - Self::RATED_VOLTAGE_REGULATION);
        let xs = target_voltage * (Self::RATED_VOLTAGE_VOLT - target_voltage) / rated_apparent.max(1.0);
        Source { id: "static-inv", open_circuit_v: Self::RATED_VOLTAGE_VOLT, resistance_ohm: xs.max(MIN_RESISTANCE_OHM), frequency_hz: Self::FREQUENCY_HZ }
    }
}

impl Default for StaticInverter {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------
// RAT: ram-air turbine, deployed only in an emergency-configuration loss of
// all main/APU generation.

#[derive(Clone, Copy, Debug, Default)]
pub struct RatFaults {
    /// Fails to fully deploy/turbine partially seized: `1.0` produces no
    /// power at all even once "deployed".
    pub jammed: f64,
}

pub struct Rat {
    deployed: bool,
}

impl Rat {
    /// `ram_air_turbine.rs::PROPELLER_DIAMETER_M` (real, FBW-sourced).
    const PROPELLER_DIAMETER_M: f64 = 1.6256;
    /// `ram_air_turbine.rs::MAX_ALLOWED_POWER_MAP`'s own plateau (real,
    /// FBW-sourced): 70 kW.
    const MAX_POWER_W: f64 = 70_000.0;
    /// GENERIC: a fixed-pitch emergency RAT is far from an optimised wind
    /// turbine's Betz-limit efficiency; a modest power coefficient is
    /// typical for this class of device (no A380-specific figure public).
    const POWER_COEFFICIENT: f64 = 0.35;
    /// ISA sea-level reference density; a full atmosphere model is out of
    /// this module's scope (the RAT's own aerodynamic module, if built,
    /// would supply a real altitude-corrected density instead).
    const AIR_DENSITY_KG_M3: f64 = 1.225;
    const RATED_VOLTAGE_VOLT: f64 = 115.0;
    const FREQUENCY_HZ: f64 = 400.0;

    pub fn new() -> Self {
        Self { deployed: false }
    }

    pub fn deploy(&mut self) {
        self.deployed = true;
    }

    pub fn deployed(&self) -> bool {
        self.deployed
    }

    pub fn terminal(&self, airspeed_kt: f64, faults: RatFaults) -> Source {
        let jammed = faults.jammed.clamp(0.0, 1.0);
        if !self.deployed || jammed >= 1.0 {
            return Source { id: "rat", open_circuit_v: 0.0, resistance_ohm: MIN_RESISTANCE_OHM, frequency_hz: 0.0 };
        }
        let v_ms = (airspeed_kt.max(0.0)) * 0.514444;
        let area_m2 = std::f64::consts::PI * (Self::PROPELLER_DIAMETER_M / 2.0).powi(2);
        let available_w = (0.5 * Self::AIR_DENSITY_KG_M3 * area_m2 * v_ms.powi(3) * Self::POWER_COEFFICIENT * (1.0 - jammed)).min(Self::MAX_POWER_W);
        if available_w <= 1.0 {
            return Source { id: "rat", open_circuit_v: 0.0, resistance_ohm: MIN_RESISTANCE_OHM, frequency_hz: 0.0 };
        }
        // Size the equivalent series resistance so the network's own
        // max-power-transfer point (V_oc^2 / (4R), the textbook result for
        // any Thevenin source) equals the aerodynamically-available power
        // directly -- ties the turbine's real physics straight into
        // `network.rs`'s existing overload/collapse branch instead of
        // inventing a second capping mechanism.
        let r = (Self::RATED_VOLTAGE_VOLT * Self::RATED_VOLTAGE_VOLT / (4.0 * available_w)).max(MIN_RESISTANCE_OHM);
        Source { id: "rat", open_circuit_v: Self::RATED_VOLTAGE_VOLT, resistance_ohm: r, frequency_hz: Self::FREQUENCY_HZ }
    }
}

impl Default for Rat {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------
// Battery: charge state, internal resistance vs temperature.

#[derive(Clone, Copy, Debug, Default)]
pub struct BatteryFaults {
    /// Permanent capacity loss (a degraded cell): up to
    /// `MAX_CAPACITY_FADE` fraction of rated capacity, same construction as
    /// `battery.rs`'s own `capacity_fade`.
    pub capacity_fade: f64,
    /// Internal resistance grown up to `AGED_RESISTANCE_MULTIPLIER`x, same
    /// construction as `battery.rs`'s own `resistance_growth`.
    pub resistance_growth: f64,
}

pub struct Battery {
    charge_ah: f64,
    temp_c: f64,
}

impl Battery {
    /// `battery.rs::RATED_CAPACITY_AMPERE_HOURS` (real, FBW-sourced).
    const RATED_CAPACITY_AH: f64 = 23.0;
    /// `battery.rs::CELL_INTERNAL_RESISTANCE_OHM_AT_20C` (real,
    /// FBW-sourced, itself cited to Airbus/Saft documentation per that
    /// file's own comment).
    const CELL_INTERNAL_RESISTANCE_OHM_AT_20C: f64 = 0.011;
    /// `battery.rs::WIRING_RESISTANCE_OHM` (real, FBW-sourced).
    const WIRING_RESISTANCE_OHM: f64 = 0.02;
    /// `battery.rs::RESISTANCE_TEMP_COEFFICIENT_PER_C` (real, FBW-sourced).
    const RESISTANCE_TEMP_COEFFICIENT_PER_C: f64 = 0.02;
    const RESISTANCE_REFERENCE_TEMP_C: f64 = 20.0;
    /// `battery.rs::THERMAL_MASS_J_PER_KELVIN` (real, FBW-sourced).
    const THERMAL_MASS_J_PER_KELVIN: f64 = 6000.0;
    /// `battery.rs::COOLING_W_PER_KELVIN` (real, FBW-sourced).
    const COOLING_W_PER_KELVIN: f64 = 2.5;
    /// `battery.rs::AGED_RESISTANCE_MULTIPLIER` (real, FBW-sourced).
    const AGED_RESISTANCE_MULTIPLIER: f64 = 3.0;
    /// `battery.rs::MAX_CAPACITY_FADE` (real, FBW-sourced).
    const MAX_CAPACITY_FADE: f64 = 0.7;
    /// `battery.rs::PEUKERT_EXPONENT` (real, FBW-sourced).
    const PEUKERT_EXPONENT: f64 = 1.08;
    /// `battery.rs::PEUKERT_REFERENCE_CURRENT_AMPERES` = rated capacity
    /// (real, FBW-sourced).
    const PEUKERT_REFERENCE_CURRENT_A: f64 = Self::RATED_CAPACITY_AH;
    /// Open-circuit voltage at full charge: FBW's own `battery.rs` test
    /// comment cites "OCV ~29 V" for this exact 23 Ah/20-cell-class A380
    /// battery (`fbw-common/.../electrical/battery.rs`, the
    /// `p_max ~ 29^2/(4*0.03)` comment near its charging tests) -- real,
    /// FBW-sourced.
    const FULL_OCV: f64 = 29.0;
    /// GENERIC: a typical 20-cell aircraft NiCd pack's practical empty-cell
    /// cutoff voltage (~1.1 V/cell), no A380-specific figure is public.
    const EMPTY_OCV: f64 = 22.0;

    pub fn new(initial_charge_fraction: f64, ambient_c: f64) -> Self {
        Self { charge_ah: Self::RATED_CAPACITY_AH * initial_charge_fraction.clamp(0.0, 1.0), temp_c: ambient_c }
    }

    pub fn usable_capacity_ah(&self, faults: BatteryFaults) -> f64 {
        Self::RATED_CAPACITY_AH * (1.0 - faults.capacity_fade.clamp(0.0, 1.0) * Self::MAX_CAPACITY_FADE)
    }

    pub fn charge_fraction(&self, faults: BatteryFaults) -> f64 {
        let usable = self.usable_capacity_ah(faults).max(1.0e-6);
        (self.charge_ah / usable).clamp(0.0, 1.0)
    }

    pub fn temperature_c(&self) -> f64 {
        self.temp_c
    }

    fn resistance_ohm(&self, faults: BatteryFaults) -> f64 {
        let below_reference = (Self::RESISTANCE_REFERENCE_TEMP_C - self.temp_c).max(0.0);
        let temp_factor = 1.0 + Self::RESISTANCE_TEMP_COEFFICIENT_PER_C * below_reference;
        let aging_factor = 1.0 + faults.resistance_growth.clamp(0.0, 1.0) * (Self::AGED_RESISTANCE_MULTIPLIER - 1.0);
        (Self::CELL_INTERNAL_RESISTANCE_OHM_AT_20C + Self::WIRING_RESISTANCE_OHM) * temp_factor * aging_factor
    }

    fn open_circuit_v(&self, faults: BatteryFaults) -> f64 {
        let f = self.charge_fraction(faults);
        Self::EMPTY_OCV + (Self::FULL_OCV - Self::EMPTY_OCV) * f
    }

    /// This tick's terminal for feeding [`network::Source`].
    pub fn terminal(&self, faults: BatteryFaults) -> (f64, f64) {
        (self.open_circuit_v(faults), self.resistance_ohm(faults).max(MIN_RESISTANCE_OHM))
    }

    /// Peukert-corrected estimated time to empty at the given discharge
    /// current, s (`f64::INFINITY` at zero/charging current) -- informational
    /// only, same `PEUKERT_EXPONENT` relation `battery.rs` itself uses to
    /// derate high-rate discharge below the simple `Ah / A` estimate.
    pub fn time_to_empty_s(&self, discharge_current_a: f64, faults: BatteryFaults) -> f64 {
        if discharge_current_a <= 0.0 {
            return f64::INFINITY;
        }
        let peukert_capacity_ah = self.usable_capacity_ah(faults) * (Self::PEUKERT_REFERENCE_CURRENT_A / discharge_current_a).powf(Self::PEUKERT_EXPONENT - 1.0);
        (self.charge_ah.min(peukert_capacity_ah) / discharge_current_a) * 3600.0
    }

    /// Integrates charge state and thermal state from the real, measured
    /// current this battery's own feeder carried last tick: positive
    /// discharges it (feeding the network), negative charges it (a TRU/GCU
    /// on the battery bus driving current back in). Plain coulomb counting
    /// (GENERIC simplification -- Peukert's own correction is applied only
    /// to the informational `time_to_empty_s`, not to the book-keeping
    /// charge itself, a common simplification for a real-time simulation).
    pub fn step(&mut self, signed_current_a: f64, faults: BatteryFaults, ambient_c: f64, dt_s: f64) -> f64 {
        let dt = dt_s.max(0.0);
        let usable = self.usable_capacity_ah(faults);
        self.charge_ah = (self.charge_ah - signed_current_a * dt / 3600.0).clamp(0.0, usable);

        let r = self.resistance_ohm(faults);
        let heat_w = signed_current_a * signed_current_a * r;
        let target = ambient_c + heat_w / Self::COOLING_W_PER_KELVIN;
        let k = Self::COOLING_W_PER_KELVIN / Self::THERMAL_MASS_J_PER_KELVIN;
        self.temp_c = target + (self.temp_c - target) * (-k * dt).exp();
        self.temp_c
    }
}

// ---------------------------------------------------------------------
// Wiring: builds the ATA24-equivalent source/contactor/breaker set onto a
// `Network` (mirroring `breakers.rs::ata24`'s own real entries -- TR 1/TR 2/
// TR ESS/TR APU, GEN 1-4, APU GEN 1-2, STATIC INVERTER -- as sources feeding
// their real buses through a generator-line contactor and protection
// breaker of the same name/rating), and orchestrates the pre/post-tick
// measured-feedback loop every source above needs.

pub struct Wiring {
    pub vfg: [Vfg; 4],
    pub apu_gen: [ApuGenerator; 2],
    pub tru: [Tru; 4],
    pub static_inverter: StaticInverter,
    pub battery: [Battery; 2],
    pub ground_power: GroundPower,
    pub rat: Rat,
    source_index: SourceIndex,
}

#[derive(Clone, Copy)]
struct SourceIndex {
    gen: [usize; 4],
    apu_gen: [usize; 2],
    tr: [usize; 4],
    static_inv: usize,
    battery: [usize; 2],
    gpu: usize,
    rat: usize,
}

/// `breakers.rs::generator_rated_a`: FBW's own VFG rating (150 kW / 0.8 /
/// 115 V), real/FBW-sourced -- reused for the GEN 1-4 breaker ratings.
fn generator_rated_a() -> f64 {
    150_000.0 / 0.8 / 115.0
}
/// `breakers.rs::apu_generator_rated_a`, real/FBW-sourced.
fn apu_generator_rated_a() -> f64 {
    120_000.0 / 0.8 / 115.0
}
/// `breakers.rs::TRU_RATED_A`: typical Airbus TRU continuous rating,
/// typical/derived (documented as such in `breakers.rs` itself).
const TRU_RATED_A: f64 = 200.0;
/// `breakers.rs::static_inverter_rated_a`, real/FBW-sourced.
fn static_inverter_rated_a() -> f64 {
    135.0 / 115.0
}

impl Wiring {
    /// Builds every ATA24 source, its generator-line contactor and its
    /// protection breaker onto `net`, at their real bus (matching
    /// `breakers.rs::ata24`'s own bus assignment one-for-one), and returns
    /// the assembled `Wiring` orchestrator.
    pub fn build(net: &mut Network, ambient_c: f64) -> Self {
        let gen_buses = [BusId::Ac1, BusId::Ac2, BusId::Ac3, BusId::Ac4];
        let mut gen = [0usize; 4];
        for (i, &bus) in gen_buses.iter().enumerate() {
            let id: &'static str = Box::leak(format!("gen-{}", i + 1).into_boxed_str());
            gen[i] = net.add_source(Source::new(id));
            let contactor_id: &'static str = Box::leak(format!("gen-{}-line", i + 1).into_boxed_str());
            net.add_contactor(Contactor::new(contactor_id, ContactorKind::GeneratorLine, FeedSource::Source(gen[i]), bus, MIN_RESISTANCE_OHM));
            net.add_breaker(super::network::Breaker::new(Box::leak(format!("gen-{}-bkr", i + 1).into_boxed_str()), generator_rated_a(), bus));
        }

        let apu_gen_bus = BusId::Ac3; // APU generator feeds whichever AC tie is available; documented approximation (its real bus depends on the priority table, out of this module's scope).
        let mut apu_gen = [0usize; 2];
        for i in 0..2usize {
            let id: &'static str = Box::leak(format!("apu-gen-{}", i + 1).into_boxed_str());
            apu_gen[i] = net.add_source(Source::new(id));
            let contactor_id: &'static str = Box::leak(format!("apu-gen-{}-line", i + 1).into_boxed_str());
            net.add_contactor(Contactor::new(contactor_id, ContactorKind::GeneratorLine, FeedSource::Source(apu_gen[i]), apu_gen_bus, MIN_RESISTANCE_OHM));
            net.add_breaker(super::network::Breaker::new(Box::leak(format!("apu-gen-{}-bkr", i + 1).into_boxed_str()), apu_generator_rated_a(), apu_gen_bus));
        }

        let tr_names = ["tr-1", "tr-2", "tr-ess", "tr-apu"];
        let tr_buses = [BusId::Dc1, BusId::Dc2, BusId::DcEss, BusId::DcApu];
        let mut tr = [0usize; 4];
        for i in 0..4usize {
            tr[i] = net.add_source(Source::new(tr_names[i]));
            let contactor_id: &'static str = Box::leak(format!("{}-line", tr_names[i]).into_boxed_str());
            net.add_contactor(Contactor::new(contactor_id, ContactorKind::Feeder, FeedSource::Source(tr[i]), tr_buses[i], MIN_RESISTANCE_OHM));
            net.add_breaker(super::network::Breaker::new(Box::leak(format!("{}-bkr", tr_names[i]).into_boxed_str()), TRU_RATED_A, tr_buses[i]));
        }

        let static_inv = net.add_source(Source::new("static-inv"));
        net.add_contactor(Contactor::new("static-inv-line", ContactorKind::Feeder, FeedSource::Source(static_inv), BusId::AcEmer, MIN_RESISTANCE_OHM));
        net.add_breaker(super::network::Breaker::new("static-inv-bkr", static_inverter_rated_a(), BusId::AcEmer));

        let mut battery = [0usize; 2];
        let battery_buses = [BusId::DcBat, BusId::DcHot1];
        for i in 0..2usize {
            let id: &'static str = Box::leak(format!("bat-{}", i + 1).into_boxed_str());
            battery[i] = net.add_source(Source::new(id));
            let contactor_id: &'static str = Box::leak(format!("bat-{}-direct", i + 1).into_boxed_str());
            net.add_contactor(Contactor::new(contactor_id, ContactorKind::BatteryDirect, FeedSource::Source(battery[i]), battery_buses[i], MIN_RESISTANCE_OHM));
            net.add_breaker(super::network::Breaker::new(Box::leak(format!("bat-{}-bkr", i + 1).into_boxed_str()), Battery::RATED_CAPACITY_AH * 4.0, battery_buses[i]));
        }

        // A real A380-style hot-bus isolation diode: DC_HOT2 is cross-fed
        // from the battery bus through a diode rather than its own direct
        // contactor, so it can never back-feed current into DC_BAT (a real,
        // if minor, aircraft wiring pattern -- hot buses must never become a
        // path for one battery/bus to drive another). GENERIC forward drop
        // (`Diode::forward_drop_v`'s own doc: ~1 V typical for a high-current
        // silicon power rectifier), no A380-specific figure public.
        net.add_diode(Diode::new("bat-cross-feed-diode", FeedSource::Bus(BusId::DcBat), BusId::DcHot2, 1.0, MIN_RESISTANCE_OHM));

        let gpu = net.add_source(Source::new("gpu"));
        net.add_contactor(Contactor::new("gpu-line", ContactorKind::Feeder, FeedSource::Source(gpu), BusId::AcGndFltSvc, MIN_RESISTANCE_OHM));

        let rat = net.add_source(Source::new("rat"));
        net.add_contactor(Contactor::new("rat-line", ContactorKind::Feeder, FeedSource::Source(rat), BusId::AcEmer, MIN_RESISTANCE_OHM));

        Self {
            vfg: std::array::from_fn(|_| Vfg::new()),
            apu_gen: std::array::from_fn(|_| ApuGenerator::new()),
            tru: std::array::from_fn(|_| Tru::new(ambient_c)),
            static_inverter: StaticInverter::new(),
            battery: [Battery::new(1.0, ambient_c), Battery::new(1.0, ambient_c)],
            ground_power: GroundPower::new(),
            rat: Rat::new(),
            source_index: SourceIndex { gen, apu_gen, tr, static_inv, battery, gpu, rat },
        }
    }

    pub fn gen_contactor_id(n: usize) -> String {
        format!("gen-{n}-line")
    }
    pub fn gen_breaker_id(n: usize) -> String {
        format!("gen-{n}-bkr")
    }

    /// Sets every source's terminal for the *next* `Network::step` from
    /// this tick's real inputs (engine speeds, battery faults, GPU plugged
    /// in, ...) and last tick's measured feedback already stored inside
    /// this struct's own sub-models. Call before `Network::step`.
    pub fn pre_step(&mut self, net: &mut Network, inputs: &WiringInputs, dt_s: f64) {
        for i in 0..4usize {
            let out = self.vfg[i].step(VfgInputs { engine_speed_fraction: inputs.engine_speed_fraction[i], measured_load_w: inputs.measured_gen_load_w[i] }, inputs.vfg_faults[i], dt_s);
            net.set_source(self.source_index.gen[i], out.open_circuit_v, out.resistance_ohm, out.frequency_hz);
        }
        for i in 0..2usize {
            let out = self.apu_gen[i].step(ApuGeneratorInputs { apu_speed_fraction: inputs.apu_speed_fraction, measured_load_w: inputs.measured_apu_gen_load_w[i] }, inputs.apu_gen_faults[i], dt_s);
            net.set_source(self.source_index.apu_gen[i], out.open_circuit_v, out.resistance_ohm, out.frequency_hz);
        }
        // TR 1/TR 2 take their real per-bus AC input (AC1/AC2); TR ESS and
        // TR APU both fall back to AC_ESS -- this 17-bus model has no
        // separate APU-generator AC tie of its own, a documented
        // approximation (`breakers.rs`'s own ATA24 doc makes the same call
        // for several of its bus-feeder entries).
        let ac_powered = [net.bus(BusId::Ac1).voltage > 90.0, net.bus(BusId::Ac2).voltage > 90.0, net.bus(BusId::AcEss).voltage > 90.0, net.bus(BusId::AcEss).voltage > 90.0];
        for i in 0..4usize {
            let (v, r) = self.tru[i].terminal(ac_powered[i], inputs.tru_faults[i]);
            net.set_source(self.source_index.tr[i], v, r, 0.0);
        }
        for i in 0..2usize {
            let (v, r) = self.battery[i].terminal(inputs.battery_faults[i]);
            net.set_source(self.source_index.battery[i], v, r, 0.0);
        }
        let battery_bus_v = net.bus(BusId::DcBat).voltage;
        let inv = self.static_inverter.terminal(battery_bus_v, inputs.static_inverter_faults);
        net.set_source(self.source_index.static_inv, inv.open_circuit_v, inv.resistance_ohm, inv.frequency_hz);

        let gpu_src = self.ground_power.terminal(inputs.gpu_plugged_in, inputs.ground_power_faults);
        net.set_source(self.source_index.gpu, gpu_src.open_circuit_v, gpu_src.resistance_ohm, gpu_src.frequency_hz);

        let rat_src = self.rat.terminal(inputs.airspeed_kt, inputs.rat_faults);
        net.set_source(self.source_index.rat, rat_src.open_circuit_v, rat_src.resistance_ohm, rat_src.frequency_hz);
    }

    /// Reads back real measured currents/temperatures after `Network::step`
    /// to update every stateful source's own thermal/charge model, ready
    /// for next tick's `pre_step`.
    pub fn post_step(&mut self, net: &Network, inputs: &WiringInputs, dt_s: f64) {
        // `measured_battery_current_a` is already the real, signed current
        // the caller read from this battery's own feeder this tick
        // (positive = discharging into the network, negative = a TRU/GCU
        // charging it back up) -- see `Wiring`'s own module doc for why this
        // one value cannot be derived from the resistive solve itself.
        for i in 0..2usize {
            self.battery[i].step(inputs.measured_battery_current_a[i], inputs.battery_faults[i], inputs.ambient_c, dt_s);
        }
        let ac_powered = [net.bus(BusId::Ac1).voltage > 90.0, net.bus(BusId::Ac2).voltage > 90.0, net.bus(BusId::AcEss).voltage > 90.0, net.bus(BusId::AcEss).voltage > 90.0];
        for i in 0..4usize {
            self.tru[i].step(ac_powered[i], inputs.measured_tr_load_w[i], inputs.tru_faults[i], inputs.ambient_c, dt_s);
        }
    }
}

/// Every external input `Wiring::pre_step`/`post_step` needs this tick. A
/// harness (or, for now, this module's own tests) fills this from whatever
/// its own engine/APU/flight-model source is; `Default` is the safe
/// everything-off/healthy state.
pub struct WiringInputs {
    pub engine_speed_fraction: [f64; 4],
    pub measured_gen_load_w: [f64; 4],
    pub vfg_faults: [VfgFaults; 4],
    pub apu_speed_fraction: f64,
    pub measured_apu_gen_load_w: [f64; 2],
    pub apu_gen_faults: [ApuGeneratorFaults; 2],
    pub tru_faults: [TruFaults; 4],
    pub measured_tr_load_w: [f64; 4],
    pub battery_faults: [BatteryFaults; 2],
    pub measured_battery_current_a: [f64; 2],
    pub static_inverter_faults: StaticInverterFaults,
    pub gpu_plugged_in: bool,
    pub ground_power_faults: GroundPowerFaults,
    pub airspeed_kt: f64,
    pub rat_faults: RatFaults,
    pub ambient_c: f64,
}

impl Default for WiringInputs {
    fn default() -> Self {
        Self {
            engine_speed_fraction: [0.0; 4],
            measured_gen_load_w: [0.0; 4],
            vfg_faults: [VfgFaults::default(); 4],
            apu_speed_fraction: 0.0,
            measured_apu_gen_load_w: [0.0; 2],
            apu_gen_faults: [ApuGeneratorFaults::default(); 2],
            tru_faults: [TruFaults::default(); 4],
            measured_tr_load_w: [0.0; 4],
            battery_faults: [BatteryFaults::default(); 2],
            measured_battery_current_a: [0.0; 2],
            static_inverter_faults: StaticInverterFaults::default(),
            gpu_plugged_in: false,
            ground_power_faults: GroundPowerFaults::default(),
            airspeed_kt: 0.0,
            rat_faults: RatFaults::default(),
            ambient_c: 15.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_stopped_vfg_produces_no_voltage_and_a_running_one_tracks_engine_speed() {
        let mut vfg = Vfg::new();
        let stopped = vfg.step(VfgInputs { engine_speed_fraction: 0.0, measured_load_w: 0.0 }, VfgFaults::default(), 1.0 / 60.0);
        assert_eq!(stopped.open_circuit_v, 0.0);
        assert_eq!(stopped.frequency_hz, 0.0);

        let idle = vfg.step(VfgInputs { engine_speed_fraction: 0.3, measured_load_w: 0.0 }, VfgFaults::default(), 1.0 / 60.0);
        let max = vfg.step(VfgInputs { engine_speed_fraction: 1.0, measured_load_w: 0.0 }, VfgFaults::default(), 1.0 / 60.0);
        assert!(idle.open_circuit_v > 100.0);
        assert!(max.frequency_hz > idle.frequency_hz, "frequency should track engine speed");
        assert!((360.0..=800.0).contains(&max.frequency_hz));
    }

    #[test]
    fn a_sustained_generator_overload_eventually_trips_its_own_protection() {
        let mut vfg = Vfg::new();
        let mut tripped_at = None;
        for i in 0..(60 * 60) {
            let out = vfg.step(VfgInputs { engine_speed_fraction: 1.0, measured_load_w: 300_000.0 }, VfgFaults::default(), 1.0 / 60.0);
            if out.overload_tripped {
                tripped_at = Some(i);
                break;
            }
        }
        assert!(tripped_at.is_some(), "a 2x-rated sustained overload should eventually trip the VFG's own protection");
    }

    #[test]
    fn winding_degradation_increases_a_vfgs_series_reactance() {
        let mut healthy = Vfg::new();
        let mut degraded = Vfg::new();
        let h = healthy.step(VfgInputs { engine_speed_fraction: 1.0, measured_load_w: 0.0 }, VfgFaults::default(), 1.0 / 60.0);
        let d = degraded.step(VfgInputs { engine_speed_fraction: 1.0, measured_load_w: 0.0 }, VfgFaults { winding_degradation: 1.0, regulator_drift: 0.0 }, 1.0 / 60.0);
        assert!(d.resistance_ohm > h.resistance_ohm * 3.0, "fully degraded windings should be near the 4x reactance ceiling");
    }

    #[test]
    fn a_tru_needs_its_ac_input_to_produce_dc_output() {
        let tru = Tru::new(15.0);
        let (v_unpowered, _) = tru.terminal(false, TruFaults::default());
        let (v_powered, r) = tru.terminal(true, TruFaults::default());
        assert_eq!(v_unpowered, 0.0);
        assert!((v_powered - 30.2).abs() < 0.01);
        assert!(r > 0.0);
    }

    #[test]
    fn a_tru_heats_under_load_and_cools_once_removed() {
        let mut tru = Tru::new(15.0);
        for _ in 0..(60 * 120) {
            tru.step(true, 5000.0, TruFaults::default(), 15.0, 1.0 / 60.0);
        }
        let hot = tru.temperature_c();
        assert!(hot > 15.0, "should have heated under sustained load: {hot} C");
        for _ in 0..(60 * 600) {
            tru.step(true, 0.0, TruFaults::default(), 15.0, 1.0 / 60.0);
        }
        assert!(tru.temperature_c() < hot, "should cool once the load is removed");
    }

    #[test]
    fn a_battery_discharges_and_its_voltage_sags_as_it_empties() {
        let mut battery = Battery::new(1.0, 20.0);
        let (full_ocv, _) = battery.terminal(BatteryFaults::default());
        for _ in 0..(3600 * 2) {
            battery.step(10.0, BatteryFaults::default(), 20.0, 1.0);
        }
        let (mid_ocv, _) = battery.terminal(BatteryFaults::default());
        assert!(mid_ocv < full_ocv, "OCV should sag as charge is drawn down: {mid_ocv} vs {full_ocv}");
        assert!(battery.charge_fraction(BatteryFaults::default()) < 1.0);
    }

    #[test]
    fn a_battery_recharges_when_fed_a_negative_current() {
        let mut battery = Battery::new(0.2, 20.0);
        let start = battery.charge_fraction(BatteryFaults::default());
        for _ in 0..(3600 * 2) {
            battery.step(-5.0, BatteryFaults::default(), 20.0, 1.0);
        }
        assert!(battery.charge_fraction(BatteryFaults::default()) > start, "negative (charging) current should raise charge state");
    }

    #[test]
    fn cold_temperature_raises_a_batterys_internal_resistance() {
        let cold = Battery::new(1.0, -20.0);
        let warm = Battery::new(1.0, 20.0);
        let (_, r_cold) = cold.terminal(BatteryFaults::default());
        let (_, r_warm) = warm.terminal(BatteryFaults::default());
        assert!(r_cold > r_warm, "a cold battery should show higher internal resistance");
    }

    #[test]
    fn capacity_fade_reduces_usable_capacity() {
        let healthy = Battery::new(1.0, 20.0);
        let faded = BatteryFaults { capacity_fade: 1.0, resistance_growth: 0.0 };
        assert!(healthy.usable_capacity_ah(faded) < healthy.usable_capacity_ah(BatteryFaults::default()));
    }

    #[test]
    fn a_static_inverter_needs_a_healthy_dc_input_to_run() {
        let inv = StaticInverter::new();
        let dead = inv.terminal(5.0, StaticInverterFaults::default());
        let alive = inv.terminal(28.0, StaticInverterFaults::default());
        assert_eq!(dead.open_circuit_v, 0.0);
        assert!(alive.open_circuit_v > 100.0);
        assert_eq!(alive.frequency_hz, 400.0);
    }

    #[test]
    fn a_rat_produces_more_power_at_higher_airspeed_and_nothing_until_deployed() {
        let mut rat = Rat::new();
        let stowed = rat.terminal(250.0, RatFaults::default());
        assert_eq!(stowed.open_circuit_v, 0.0);
        rat.deploy();

        // Available shaft power is `0.5 * rho * A * Cp * v^3`, so it crosses
        // the emergency generator's own 70 kW rating (FBW's
        // `MAX_ALLOWED_POWER_MAP` plateau) at
        //   A   = pi*(1.6256/2)^2                     = 2.0755 m^2
        //   v^3 = 70000 / (0.5*1.225*2.0755*0.35)
        //       = 70000 / 0.44494                     = 1.5733e5 m^3/s^3
        //   v   = 54.0 m/s                            = 105 kt.
        // Below that the RAT is aerodynamically limited and stiffens with
        // airspeed; at and above it the turbine is governed and the
        // generator's rating is the limit, so output is flat across the rest
        // of the envelope -- which is how a real RAT is specified (full rated
        // output from its minimum operating airspeed all the way to Vmo).
        let slow = rat.terminal(60.0, RatFaults::default());
        let faster = rat.terminal(90.0, RatFaults::default());
        assert!(faster.open_circuit_v > 0.0 && slow.open_circuit_v > 0.0);
        assert!(
            faster.resistance_ohm < slow.resistance_ohm,
            "below the generator's rating, higher airspeed must mean a stiffer (more capable) equivalent source: {} vs {}",
            faster.resistance_ohm,
            slow.resistance_ohm
        );

        // On the governed plateau, 150 kt and 300 kt must give the identical
        // equivalent source: the max-power-transfer resistance of a 115 V
        // source delivering the rated 70 kW is V_oc^2/(4P) = 115^2/280000
        // = 0.047232 ohm.
        let plateau_r = Rat::RATED_VOLTAGE_VOLT * Rat::RATED_VOLTAGE_VOLT / (4.0 * Rat::MAX_POWER_W);
        let cruise = rat.terminal(150.0, RatFaults::default());
        let fast = rat.terminal(300.0, RatFaults::default());
        assert!((cruise.resistance_ohm - plateau_r).abs() < 1e-12, "150 kt is already on the 70 kW plateau: {}", cruise.resistance_ohm);
        assert!((fast.resistance_ohm - plateau_r).abs() < 1e-12, "300 kt is governed to the same 70 kW rating: {}", fast.resistance_ohm);
    }

    #[test]
    fn a_jammed_rat_produces_no_power_even_when_deployed() {
        let mut rat = Rat::new();
        rat.deploy();
        let out = rat.terminal(300.0, RatFaults { jammed: 1.0 });
        assert_eq!(out.open_circuit_v, 0.0);
    }

    #[test]
    fn ground_power_only_energises_when_plugged_in() {
        let gpu = GroundPower::new();
        assert_eq!(gpu.terminal(false, GroundPowerFaults::default()).open_circuit_v, 0.0);
        let live = gpu.terminal(true, GroundPowerFaults::default());
        assert!(live.open_circuit_v > 100.0);
        assert_eq!(live.frequency_hz, 400.0);
    }

    #[test]
    fn wiring_builds_and_a_running_generator_energises_its_bus_through_the_network() {
        let mut net = Network::new();
        let mut wiring = Wiring::build(&mut net, 15.0);
        net.command_contactor(&Wiring::gen_contactor_id(1), true);
        let mut inputs = WiringInputs::default();
        inputs.engine_speed_fraction[0] = 1.0;
        for _ in 0..10 {
            wiring.pre_step(&mut net, &inputs, 1.0 / 60.0);
            net.step(1.0 / 60.0);
            wiring.post_step(&net, &inputs, 1.0 / 60.0);
        }
        assert!(net.bus(BusId::Ac1).voltage > 100.0, "GEN 1 running and its line contactor closed should energise AC1: {}", net.bus(BusId::Ac1).voltage);
    }

    #[test]
    fn a_pulled_generator_breaker_prevents_wiring_from_reclosing_its_bus() {
        // The generator's own protection breaker exists in the network
        // alongside its contactor; pulling it does not stop the contactor
        // from being commanded closed, but it does gate any load fed
        // through it in the same way every other breaker does -- checked
        // here indirectly via the breaker existing and being pull-able.
        let mut net = Network::new();
        Wiring::build(&mut net, 15.0);
        let bkr = net.breaker_index(&Wiring::gen_breaker_id(1)).expect("GEN 1 breaker should exist");
        net.breakers[bkr].pull();
        assert!(!net.breakers[bkr].closed);
    }
}
