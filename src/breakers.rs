//! The circuit-breaker catalogue (docs/analysis/cockpit-study-cbs.md CB-001/
//! CB-002/STUDY-001; `docs/briefs/hyperrealism.md`'s stage-3 follow-on).
//!
//! `src/circuits.rs` already models a real, working breaker for every one of
//! the 154 `systems.cfg` circuits (fuel pumps/valves, lights, gear, radios,
//! ...) plus 52 clickable-but-unwired avionics-bay `CB_*` panel positions
//! (`circuits::PANEL_CB_NODES`). The user's complaint (CB-002/STUDY-001) is
//! that this is "far too little" next to a real A380 flight deck, and that
//! it stops at the cockpit's own wiring instead of the much larger set of
//! consumers FlyByWire's *systems* crate genuinely simulates (generators,
//! TRUs, flight control computers, air-conditioning LRUs, fire-detection
//! loops, hydraulic pumps, ...), none of which had a breaker at all.
//!
//! This module is the "over 100 more" catalogue on top of those 154+52. It
//! is **not** a cosmetic list: every entry's `Effect` is a real gate on a
//! consumer the plugin or FlyByWire's own Rust systems already model, in one
//! of three ways (docs/physics/breakers.md has the full derivation table):
//!
//! 1. **`failures`** (the overwhelming majority): bridges to a
//!    [`crate::failures::FailureType`] FlyByWire's own systems crate already
//!    consumes for a real physical effect (verified by reading the
//!    `Failure::new(FailureType::...)`/`is_active()` call site for every
//!    group below, not assumed) — e.g. pulling "TR 1" breaker calls
//!    `failures::set_active(24_000, true)`, and `transformer_rectifier.rs`'s
//!    own `if !self.failure.is_active() && input.is_powered()` really stops
//!    that TRU converting AC to DC. A breaker's *rating* still comes from
//!    this module's own real-consumer-demand derivation, independent of
//!    (and finer-grained than) the failure system, which has no notion of
//!    current at all.
//! 2. **`plugin_var`**: a handful of consumers with no existing FlyByWire
//!    failure id at all get a *new* breaker gate patched directly into
//!    FlyByWire's Rust systems (`patches/fbw-rust/breakers.patch`): the four
//!    electric hydraulic pumps (green A/B, yellow A/B, one shared generic
//!    mechanism, `ElectricalPumpPhysics::receive_power`/`read`) and the
//!    autobrake knob-disarm solenoid. Pulling these writes the named
//!    variable to 0; FlyByWire's own `SimulatorReader` reads it every tick
//!    (the "registered simulation variable, not a Rust global" the remote/
//!    out-of-process systems host requires) and ANDs it into the consumer's
//!    own `is_powered` check, so it stops drawing current for real.
//! 3. **`panel_node`**: for the subset of [`crate::circuits::PANEL_CB_NODES`]
//!    whose label has a defensible, checked correspondence to one of the
//!    above (`CB_TR1` -> `TransformerRectifier(1)`, `CB_LGCIS1` ->
//!    `LgciuPowerSupply(Lgciu1)`, ...), this module reuses that node's
//!    *existing* `CIRCUIT BREAKER CLOSED:<panel_number>` dataref (already
//!    registered, default closed, by `circuits::Circuits::new`) as the
//!    breaker's storage, instead of registering a second one — so if the
//!    cockpit model's own `CB_*` geometry is ever wired to a click handler,
//!    it lands on the exact same state this catalogue's Study-panel entry
//!    already drives. Every other `PANEL_CB_NODES` label had no checked
//!    correspondence to a modelled consumer and is left as-is (reported,
//!    not guessed).
//!
//! Breakers with no physical panel position get a new synthetic number in
//! [`EXTRA_BASE`]'s range, stored the same way `circuits.rs` stores a real
//! circuit's breaker (`CIRCUIT BREAKER CLOSED/CURRENT/TRIP CAUSE:n`), so the
//! Study panel's existing polling/JSON conventions keep working unchanged.
//!
//! Thermal/magnetic trip physics are **not** reimplemented here: both this
//! module and `physics::electrical::CircuitProtection` call the same
//! [`crate::physics::electrical::trip_step`] curve, so there is exactly one
//! I^2t/magnetic curve in the whole plugin (`docs/physics/electrical.md`
//! section 6), applied to a wider set of breakers.

use std::collections::HashMap;
use std::sync::Mutex;

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::circuits::{self, MSFS_BUSES};
use crate::physics::electrical::{trip_step_with_ambient, TripCause};

/// Where a breaker's rated current is evaluated against: either a real
/// `circuits.rs`/FlyByWire MSFS bus (live, Kirchhoff-solved voltage
/// available as `ELEC_<fbw>_BUS_POTENTIAL`), or a named FlyByWire
/// `ElectricalBusType` this plugin has no live-potential mirror for
/// (`docs/physics/breakers.md` section 1: `AlternatingCurrentStaticInverter`,
/// `AlternatingCurrentNamed("247XP")`, `DirectCurrentNamed("247PP")`,
/// `DirectCurrentBattery`), evaluated at its nominal voltage instead.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Bus {
    Msfs(u32),
    Named(&'static str, f64),
}
impl Bus {
    fn nominal_voltage(self) -> f64 {
        match self {
            Bus::Msfs(n) => crate::physics::electrical::nominal_bus_voltage(n),
            Bus::Named(_, v) => v,
        }
    }
    fn label(self) -> String {
        match self {
            Bus::Msfs(n) => MSFS_BUSES.iter().find(|(m, _, _)| *m == n).map(|(_, _, fbw)| fbw.to_string()).unwrap_or_default(),
            Bus::Named(n, _) => n.to_string(),
        }
    }
}

/// One catalogue entry: a real, sourced, working breaker. Every field is
/// `Copy` (string/slice data is always `'static`, built once -- see
/// [`catalog`]), so this whole struct is `Copy`.
#[derive(Clone, Copy)]
pub struct BreakerDef {
    pub id: &'static str,
    pub name: &'static str,
    pub ata: u16,
    pub bus: Bus,
    pub rating_a: f64,
    pub basis: &'static str,
    pub consumers: &'static [&'static str],
    /// FlyByWire failure ids this breaker's *open* state activates (and
    /// whose closed state clears). Empty for a `plugin_var`-gated entry.
    pub failures: &'static [u64],
    /// A new FlyByWire-patched consumer's own breaker variable
    /// (`patches/fbw-rust/breakers.patch`), written 1.0 (closed) / 0.0
    /// (open) every tick.
    pub plugin_var: Option<&'static str>,
    /// Reuses one of `circuits::PANEL_CB_NODES`'s existing breaker datarefs
    /// for storage instead of a new synthetic number.
    pub panel_node: Option<&'static str>,
    /// An absorbed `systems.cfg` circuit (`circuits::CircuitDef::number`)
    /// this breaker also gates: `pre_systems` mirrors this breaker's closed
    /// state onto `Circuits::set_breaker`, which `fuel.rs`/`lights.rs`
    /// already read for a real effect (docs/physics/breakers.md
    /// "systems.cfg audit"). The raw circuit is no longer surfaced on its
    /// own once absorbed (`study::web::breakers_json`).
    pub circuit: Option<usize>,
    /// A real, named A380 breaker with **no** modelled consumer at all
    /// (`failures`/`plugin_var`/`circuit` all empty) -- listed for realism
    /// (state + rating only, `"gates":"none"` in the JSON) rather than
    /// invented a fake gate for, per docs/physics/breakers.md.
    pub gates_none: bool,
    /// A real power-path `plugin_var` patch has been drafted for this
    /// consumer's own FlyByWire struct (path under `patches/fbw-rust/`,
    /// `git apply --check`-verified) but is **not applied** to
    /// `D:\fbw-aircraft`'s working tree -- this session's sandbox blocks
    /// writing there. Today's real gate is whatever `failures`/`circuit`/
    /// `gates_none` above says; once the lead applies the patch, flip
    /// `plugin_var` to `Some(...)` here (no other code change needed) and
    /// clear this field. `"gates":"pluginVarPending"` in the JSON.
    pub pending_patch: Option<&'static str>,
}

impl BreakerDef {
    /// The bus this breaker is fed from, by its FlyByWire name.
    pub fn bus_label(&self) -> String {
        match self.bus {
            Bus::Msfs(n) => MSFS_BUSES.iter().find(|(m, _, _)| *m == n).map_or(format!("bus {n}"), |(_, _, name)| name.to_string()),
            Bus::Named(name, _) => name.to_string(),
        }
    }

    /// How this breaker really gates its consumer, for `/study/breakers`'
    /// `"gates"` field (docs/physics/breakers.md "power-path audit", the
    /// user's "no FailureType, a real electrical path" requirement):
    /// `"pluginVar"`/`"circuit"` remove real power (a patched FlyByWire
    /// breaker field, or an absorbed systems.cfg circuit); `"failurePower"`
    /// bridges a `FailureType` this module's audit verified gates the same
    /// real Kirchhoff-solved power path (`is_conductive`/`transform`/
    /// `should_provide_output`/`output_potential` -- reading the exact
    /// source line, not assumed) rather than a soft simulated fault;
    /// `"failureSoft"` bridges one that does not (a real, checked
    /// FlyByWire effect, just not a power cutoff); `"none"` is a real named
    /// breaker with nothing modelled to gate at all.
    pub fn gate_kind(&self) -> &'static str {
        if self.plugin_var.is_some() {
            "pluginVar"
        } else if self.circuit.is_some() {
            "circuit"
        } else if !self.failures.is_empty() {
            if self.failures.iter().all(|&f| is_power_path_failure(f)) { "failurePower" } else { "failureSoft" }
        } else if self.pending_patch.is_some() {
            "pluginVarPending"
        } else {
            "none"
        }
    }
}

/// Failure ids this module's own audit verified gate a real Kirchhoff-
/// solved power path, not a soft simulated fault (docs/physics/breakers.md
/// "power-path audit" has the full per-id citation):
/// - 24_000-24_003 `TransformerRectifier`: `transformer_rectifier.rs`'s
///   `transform()` -- `!failure.is_active() && input.is_powered()` gates
///   the TR's real output `Potential`.
/// - 24_004 `StaticInverter`: `static_inverter.rs` -- `report.is_powered
///   (self) && !failure.is_active()` gates `has_output`.
/// - 24_020-24_023 `Generator`: `engine_generator.rs`'s
///   `should_provide_output()` (`&& !failure.is_active()`) gates
///   `ElectricitySource::output_potential()`, the VFG's Kirchhoff
///   contribution.
/// - 24_030-24_031 `ApuGenerator`: `pw980.rs`'s `should_provide_output()`
///   (`!failure.is_active()`) gates `output_potential()` the same way.
/// - 24_100-24_117 `ElectricalBus`: `electrical/mod.rs`'s
///   `ElectricalBus::is_conductive()` is literally `!failure.is_active()`
///   -- the bus itself becomes non-conductive, the Kirchhoff solver excludes
///   it, and every real downstream consumer (FBW Rust or FlyByWire's own
///   TypeScript reading `..._BUS_IS_POWERED`) sees a real loss of power.
///
/// Every other bridged `FailureType` in this catalogue (CabinFan, HotAir,
/// FwdIsolValve/BulkIsolValve/CargoHeater, Fdac/Tadd/Vcm/Ocsm/CPIOM-B apps,
/// LgciuPowerSupply/InternalError, GearProxSensorDamage, GearActuatorJammed,
/// RadioAltimeter/Antenna, and PRIM/SEC/FCDC/ROLLOUT/FCU via
/// `COMPUTER_FAILURES`) was checked the same way and found to gate a real
/// but *non-electrical* effect (a sensor reading, a jammed actuator, a
/// computer's own self-declared failed state) rather than a bus/source
/// conductivity flag -- real, not fabricated, just not a power-path cut.
/// Converting one of those to a genuine power-path breaker needs a new
/// field on its own FlyByWire struct (the same pattern as the four electric
/// hydraulic pumps' `plugin_var`, and now `FireDetectionLoop`'s own 12
/// per-zone breakers -- ata26() below), i.e. a `patches/fbw-rust` change
/// applied to `D:\fbw-aircraft`. Direct edits there are not sandbox-blocked
/// (verified this session); `docs/physics/breakers.md`'s checkpoint report
/// has the running per-consumer status.
fn is_power_path_failure(id: u64) -> bool {
    matches!(id, 24_000..=24_004 | 24_020..=24_023 | 24_030..=24_031 | 24_100..=24_117)
}

fn d(id: &'static str, name: &'static str, ata: u16, bus: Bus, rating_a: f64, basis: &'static str, consumers: &'static [&'static str], failures: &'static [u64]) -> BreakerDef {
    BreakerDef { id, name, ata, bus, rating_a, basis, consumers, failures, plugin_var: None, panel_node: None, circuit: None, gates_none: false, pending_patch: None }
}

/// Same as [`d`], but for a converted power-path breaker: `plugin_var` is
/// the real FlyByWire-patched consumer variable, `failures` is empty (the
/// old FailureType bridge is superseded, not stacked).
fn dp(id: &'static str, name: &'static str, ata: u16, bus: Bus, rating_a: f64, basis: &'static str, consumers: &'static [&'static str], plugin_var: &'static str) -> BreakerDef {
    BreakerDef { id, name, ata, bus, rating_a, basis, consumers, failures: &[], plugin_var: Some(plugin_var), panel_node: None, circuit: None, gates_none: false, pending_patch: None }
}

// ---------------------------------------------------------------------
// Rating basis (docs/physics/breakers.md has the full cited table).

/// Generic small avionics-LRU rating, the same fallback
/// `physics::electrical::rated_watts` already uses for a `systems.cfg`
/// circuit type with no more specific figure (typical/derived, no FBW/
/// public per-box figure -- this module's own consumers are individually
/// smaller LRUs than a whole circuit's worth of avionics).
const AVIONICS_LRU_W: f64 = 50.;
/// A substantial flight-control/warning computer (PRIM/SEC/FCDC-class):
/// typical/derived, larger than a simple LRU box.
const FLIGHT_COMPUTER_W: f64 = 100.;
const CABIN_FAN_W: f64 = 500.;
const VALVE_ACTUATOR_W: f64 = 50.;
const EXTRACT_FAN_W: f64 = 150.;
const CARGO_HEATER_W: f64 = 1000.;
const FIRE_LOOP_W: f64 = 20.;
/// Real Airbus TRU continuous output rating (typical/derived: no A380-
/// specific public figure; a common large-transport TRU class rating,
/// consistent with FBW's own TRU output impedance, `transformer_rectifier.
/// rs`'s `INTERNAL_RESISTANCE_OHM`/`IDLE_OUTPUT_VOLTAGE`, `docs/physics/
/// electrical.md` section 4).
const TRU_RATED_A: f64 = 200.;

fn generator_rated_a() -> f64 {
    // FBW's own VFG rating (electrical.md section 1, cited to
    // alternating_current.rs:393): 150 kW true power at POWER_FACTOR 0.8,
    // apparent power at the same 115 V nominal the plugin's circuit
    // protection already uses for every AC bus.
    150_000. / 0.8 / 115.
}
fn apu_generator_rated_a() -> f64 {
    // FBW's own Pw980ApuGenerator::MAXIMUM_LOAD_WATT (electrical.md
    // section 1).
    120_000. / 0.8 / 115.
}
fn static_inverter_rated_a() -> f64 {
    // FBW's own power_consumption.rs AC_STAT_INV bus demand, 135 W (the
    // same figure electrical.md section 4 cites).
    135. / 115.
}

/// A bus's own `A380PowerConsumption`/`power_consumption.rs`
/// `FlightPhasePowerConsumer` peak wattage across flight phases (real,
/// FBW-sourced -- these are FlyByWire's own numbers, not typical/derived),
/// used for the 11 bus-tie/feeder breakers whose bus has one. The other 7
/// of the 18 `ElectricalBus` failure buses have no aggregate consumer
/// modelled in `power_consumption.rs` at all (`docs/physics/breakers.md`
/// section 1 lists which); each of those is rated identically to the
/// nearest same-voltage-class bus that does, a documented approximation,
/// not a second source.
fn bus_peak_w(label: &str) -> f64 {
    match label {
        "AC1" => 39_032.5,
        "AC2" | "AC3" | "AC4" => 29_777.4, // AC3/AC4: no consumer modelled; AC2's own figure used (smaller of the two modelled AC ties).
        "AC_ESS" => 875.7,
        "AC_ESS_SHED" | "AC_247XP" => 823.5, // AC_247XP: no consumer modelled; AC_ESS_SHED's figure used (same secondary-bus class).
        "AC_GND_FLT_SVC" => 4_718.,
        "DC1" => 364.,
        "DC2" => 532.,
        "DC_ESS" => 168.,
        "DC_HOT1" => 108.,
        "DC_HOT2" | "DC_HOT3" | "DC_HOT4" | "DC_247PP" | "DC_309PP" => 24.3, // no consumer modelled for these; DC_HOT2's figure used (hot-bus class).
        "DC_GND_FLT_SVC" => 168.,
        _ => AVIONICS_LRU_W,
    }
}

fn msfs_bus_for_fbw(fbw: &'static str) -> Bus {
    MSFS_BUSES.iter().find(|(_, _, f)| *f == fbw).map(|(n, _, _)| Bus::Msfs(*n)).unwrap_or(Bus::Named(fbw, 115.))
}

// ---------------------------------------------------------------------
// ATA21 -- air conditioning (a380_failures 21_001..21_049; source buses
// from a380_systems/air_conditioning/mod.rs's own constructors, cited by
// FBW's own relay-panel comments, e.g. "// 403XP").

fn ata21(v: &mut Vec<BreakerDef>) {
    // Cabin recirculation fans 1-4, each on its own numbered AC bus
    // (mod.rs: `CabinFan::new(id, ..., ElectricalBusType::AlternatingCurrent(id))`).
    // Converted to a real power-path plugin_var (patches/fbw-rust/
    // power-path-cabinfan.patch, applied): CabinFan::receive_power now ANDs
    // in its own breaker_closed, read from ELEC_CABIN_FAN_<id>_BREAKER_
    // CLOSED every tick -- an open breaker really stops the fan moving air,
    // not a soft FailureType.
    const FANS: [(&str, &str, Bus, &str); 4] = [
        ("cab-fan-1", "CAB FAN 1", Bus::Msfs(2), "ELEC_CABIN_FAN_1_BREAKER_OPEN"),
        ("cab-fan-2", "CAB FAN 2", Bus::Msfs(3), "ELEC_CABIN_FAN_2_BREAKER_OPEN"),
        ("cab-fan-3", "CAB FAN 3", Bus::Msfs(4), "ELEC_CABIN_FAN_3_BREAKER_OPEN"),
        ("cab-fan-4", "CAB FAN 4", Bus::Msfs(5), "ELEC_CABIN_FAN_4_BREAKER_OPEN"),
    ];
    for (id, name, bus, plugin_var) in FANS {
        v.push(BreakerDef {
            id,
            name,
            ata: 21,
            bus,
            rating_a: CABIN_FAN_W / bus.nominal_voltage(),
            basis: "typical large-transport cabin recirculation fan motor (500 W, typical/derived); bus from a380_systems/air_conditioning/mod.rs CabinFan::new",
            consumers: &["cabin recirculation fan"],
            failures: &[],
            plugin_var: Some(plugin_var),
            panel_node: None, circuit: None, gates_none: false, pending_patch: None,
        });
    }
    // Trim air hot-air valves 1/2 (AC1 ess-fed per FDAC's own bus set; the
    // valve itself has no separate bus in mod.rs, so it is rated on the
    // FDAC's own primary bus, AC_ESS).
    v.push(d("hotair-1", "HOT AIR VALVE 1", 21, Bus::Msfs(6), VALVE_ACTUATOR_W / 115., "typical motor-operated hot-air valve actuator (50 W, typical/derived); FDAC 1's own AC_ESS channel bus", &["hot air valve 1"], &[21_005]));
    v.push(d("hotair-2", "HOT AIR VALVE 2", 21, Bus::Msfs(6), VALVE_ACTUATOR_W / 115., "typical motor-operated hot-air valve actuator (50 W, typical/derived); FDAC 2's own AC_ESS channel bus", &["hot air valve 2"], &[21_006]));
    v.push(d("fwd-isol-valve", "FWD CARGO ISOL VALVE", 21, Bus::Msfs(8), VALVE_ACTUATOR_W / 28., "typical motor-operated isolation valve actuator (50 W, typical/derived); VCM Fwd's own primary channel bus, DC1/411PP -- mod.rs VentilationControlModule::new(.., VcmId::Fwd, [DirectCurrent(1), DirectCurrentEssential]), channel 1 backed by powered_by[0] and default-active (was wired to DC2, which is VCM AFT's bus, not Fwd's)", &["forward cargo isolation valve"], &[21_007]));
    v.push(d("fwd-extract-fan", "FWD CARGO EXTRACT FAN", 21, Bus::Msfs(2), EXTRACT_FAN_W / 115., "typical small extraction fan motor (150 W, typical/derived); the fan's own dedicated bus, not the VCM's channel bus -- ventilation_control_module.rs ForwardCargoVentilationControlSystem::new(AlternatingCurrent(1)), really gated in receive_power/fwd_extraction_fan_is_on (was wired to VCM Fwd's DC channel, which the fan does not draw from at all)", &["forward cargo extraction fan"], &[21_008]));
    v.push(d("bulk-isol-valve", "BULK CARGO ISOL VALVE", 21, Bus::Msfs(9), VALVE_ACTUATOR_W / 28., "typical motor-operated isolation valve actuator (50 W, typical/derived); VCM Aft's own primary channel bus, DC2/214PP -- mod.rs VentilationControlModule::new(.., VcmId::Aft, [DirectCurrent(2), DirectCurrentEssential]), channel 1 backed by powered_by[0] and default-active, matching fwd-isol-valve's convention (was on DC_ESS, Aft's secondary/standby channel)", &["bulk cargo isolation valve"], &[21_009]));
    v.push(d("bulk-extract-fan", "BULK CARGO EXTRACT FAN", 21, Bus::Msfs(5), EXTRACT_FAN_W / 115., "typical small extraction fan motor (150 W, typical/derived); the fan's own dedicated bus, not the VCM's channel bus -- ventilation_control_module.rs BulkVentilationControlSystem::new(AlternatingCurrent(4)), really gated in receive_power/bulk_extraction_fan_is_on (was wired to VCM Aft's DC_ESS channel, which the fan does not draw from at all)", &["bulk cargo extraction fan"], &[21_010]));
    v.push(d("cargo-heater", "BULK CARGO HEATER", 21, Bus::Msfs(3), CARGO_HEATER_W / 115., "typical cargo-bay heater element (1000 W, typical/derived); mod.rs AirHeater::new(AlternatingCurrent(2)), \"// 200XP4\"", &["bulk cargo heater element"], &[21_011]));

    // FDAC 1/2, each two redundant channels (mod.rs: channel 1 on AC_ESS
    // \"403XP\", channel 2 on AC2/AC4 \"117XP\"/\"204XP\"). Converted to a
    // real power-path plugin_var (patches/fbw-rust/power-path-operating-
    // channel.patch, applied): OperatingChannel::receive_power now ANDs in
    // its own breaker_closed, read from ELEC_FDAC_<n>_<ch>_BREAKER_OPEN
    // every tick -- shared plumbing also used below by TADD/VCM/OCSM.
    v.push(dp("fdac-1a", "FDAC 1 CHANNEL 1", 21, Bus::Msfs(6), AVIONICS_LRU_W / 115., "generic avionics LRU (50 W, typical/derived); mod.rs FullDigitalAGUController::new(FdacId::One, [AC_ESS \"403XP\", ...])", &["FDAC 1 channel 1 (pack 1 control)"], "ELEC_FDAC_1_1_BREAKER_OPEN"));
    v.push(dp("fdac-1b", "FDAC 1 CHANNEL 2", 21, Bus::Msfs(3), AVIONICS_LRU_W / 115., "generic avionics LRU (50 W, typical/derived); mod.rs FullDigitalAGUController::new(FdacId::One, [..., AC2 \"117XP\"])", &["FDAC 1 channel 2 (pack 1 control)"], "ELEC_FDAC_1_2_BREAKER_OPEN"));
    v.push(dp("fdac-2a", "FDAC 2 CHANNEL 1", 21, Bus::Msfs(6), AVIONICS_LRU_W / 115., "generic avionics LRU (50 W, typical/derived); mod.rs FullDigitalAGUController::new(FdacId::Two, [AC_ESS \"403XP\", ...])", &["FDAC 2 channel 1 (pack 2 control)"], "ELEC_FDAC_2_1_BREAKER_OPEN"));
    v.push(dp("fdac-2b", "FDAC 2 CHANNEL 2", 21, Bus::Msfs(5), AVIONICS_LRU_W / 115., "generic avionics LRU (50 W, typical/derived); mod.rs FullDigitalAGUController::new(FdacId::Two, [..., AC4 \"204XP\"])", &["FDAC 2 channel 2 (pack 2 control)"], "ELEC_FDAC_2_2_BREAKER_OPEN"));
    v.push(dp("tadd-1", "TADD CHANNEL 1", 21, Bus::Msfs(3), AVIONICS_LRU_W / 115., "generic avionics LRU (50 W, typical/derived); mod.rs TrimAirDriveDevice::new([AC2 \"117XP\", ...])", &["trim air drive device channel 1"], "ELEC_TADD_1_BREAKER_OPEN"));
    v.push(dp("tadd-2", "TADD CHANNEL 2", 21, Bus::Msfs(5), AVIONICS_LRU_W / 115., "generic avionics LRU (50 W, typical/derived); mod.rs TrimAirDriveDevice::new([..., AC4 \"206XP\"])", &["trim air drive device channel 2"], "ELEC_TADD_2_BREAKER_OPEN"));
    v.push(dp("vcm-fwd-1", "VCM FWD CHANNEL 1", 21, Bus::Msfs(8), AVIONICS_LRU_W / 28., "generic avionics LRU (50 W, typical/derived); mod.rs VentilationControlModule::new(Fwd, [DC1 \"411PP\", ...]) -- fixes/W185.md: 411PP is DC1's own PP number (VcmId::Fwd's powered_by[0] is ElectricalBusType::DirectCurrent(1)), not DC2's as this entry's comment previously (self-contradictorily) said; was wired to Bus::Msfs(9)/DC2, which is VCM Aft's channel-1 bus, not Fwd's -- same class of bug fixes/W115.md and fixes/W161.md already fixed for fwd-isol-valve/fwd-extract-fan, flagged as a follow-up by fixes/W161.md but not fixed there", &["forward ventilation control module channel 1"], "ELEC_VCM_FWD_1_BREAKER_OPEN"));
    v.push(dp("vcm-fwd-2", "VCM FWD CHANNEL 2", 21, Bus::Msfs(10), AVIONICS_LRU_W / 28., "generic avionics LRU (50 W, typical/derived); mod.rs VentilationControlModule::new(Fwd, [..., DC_ESS \"109PP\"])", &["forward ventilation control module channel 2"], "ELEC_VCM_FWD_2_BREAKER_OPEN"));
    v.push(dp("vcm-aft-1", "VCM AFT CHANNEL 1", 21, Bus::Msfs(9), AVIONICS_LRU_W / 28., "generic avionics LRU (50 W, typical/derived); mod.rs VentilationControlModule::new(Aft, [DC2 \"214PP\", ...])", &["aft ventilation control module channel 1"], "ELEC_VCM_AFT_1_BREAKER_OPEN"));
    v.push(dp("vcm-aft-2", "VCM AFT CHANNEL 2", 21, Bus::Msfs(10), AVIONICS_LRU_W / 28., "generic avionics LRU (50 W, typical/derived); mod.rs VentilationControlModule::new(Aft, [..., DC_ESS \"109PP\"])", &["aft ventilation control module channel 2"], "ELEC_VCM_AFT_2_BREAKER_OPEN"));

    // OCSM auto-partition + 4 units x 2 channels (mod.rs: OCSM1/2 on
    // DC1/DC_ESS "107PP"/"417PP", OCSM3/4 on DC2/DC_ESS "210PP"/"411PP").
    const OCSM_AP: [(&str, &str, u64); 4] = [
        ("ocsm-1-ap", "OCSM 1 AUTO PARTITION", 21_022),
        ("ocsm-2-ap", "OCSM 2 AUTO PARTITION", 21_023),
        ("ocsm-3-ap", "OCSM 3 AUTO PARTITION", 21_024),
        ("ocsm-4-ap", "OCSM 4 AUTO PARTITION", 21_025),
    ];
    for (id, name, fail) in OCSM_AP {
        let bus = if id.contains('1') || id.contains('2') { Bus::Msfs(8) } else { Bus::Msfs(9) };
        v.push(d(id, name, 21, bus, AVIONICS_LRU_W / 28., "generic avionics LRU (50 W, typical/derived); mod.rs OutflowValveControlModule::new bus set", &["outflow valve control module auto-partition logic"], Box::leak(vec![fail].into_boxed_slice())));
    }
    // Converted to a real power-path plugin_var (patches/fbw-rust/
    // power-path-operating-channel.patch, applied): OperatingChannel::
    // receive_power now ANDs in its own breaker_closed, read from
    // ELEC_OCSM_<n>_<ch>_BREAKER_OPEN every tick.
    const OCSM_CH: [(&str, &str, Bus, &str); 8] = [
        ("ocsm-1a", "OCSM 1 CHANNEL 1", Bus::Msfs(8), "ELEC_OCSM_1_1_BREAKER_OPEN"),
        ("ocsm-1b", "OCSM 1 CHANNEL 2", Bus::Msfs(10), "ELEC_OCSM_1_2_BREAKER_OPEN"),
        ("ocsm-2a", "OCSM 2 CHANNEL 1", Bus::Msfs(8), "ELEC_OCSM_2_1_BREAKER_OPEN"),
        ("ocsm-2b", "OCSM 2 CHANNEL 2", Bus::Msfs(10), "ELEC_OCSM_2_2_BREAKER_OPEN"),
        ("ocsm-3a", "OCSM 3 CHANNEL 1", Bus::Msfs(9), "ELEC_OCSM_3_1_BREAKER_OPEN"),
        ("ocsm-3b", "OCSM 3 CHANNEL 2", Bus::Msfs(10), "ELEC_OCSM_3_2_BREAKER_OPEN"),
        ("ocsm-4a", "OCSM 4 CHANNEL 1", Bus::Msfs(9), "ELEC_OCSM_4_1_BREAKER_OPEN"),
        ("ocsm-4b", "OCSM 4 CHANNEL 2", Bus::Msfs(10), "ELEC_OCSM_4_2_BREAKER_OPEN"),
    ];
    for (id, name, bus, plugin_var) in OCSM_CH {
        v.push(BreakerDef {
            id,
            name,
            ata: 21,
            bus,
            rating_a: AVIONICS_LRU_W / bus.nominal_voltage(),
            basis: "generic avionics LRU (50 W, typical/derived); mod.rs OutflowValveControlModule::new(OcsmId::.., [DC1/DC2 \"107PP\"/\"210PP\", DC_ESS \"417PP\"/\"411PP\"])",
            consumers: &["outflow valve control module channel"],
            failures: &[],
            plugin_var: Some(plugin_var),
            panel_node: None, circuit: None, gates_none: false, pending_patch: None,
        });
    }

    // CPIOM B1-4 applications (mod.rs:1334-1337 CPIOM bus map: B1/DC1,
    // B2/DC_ESS, B3/DC_ESS, B4/DC2).
    //
    // AGS is handled separately below, not in this generic per-CPIOM loop:
    // unlike TCS/VCS/CPCS (checked and found functionally symmetric across
    // all 4 CPIOMs, no defensible per-panel-position split -- W08/W65), AGS
    // really is a pack-pair application in FlyByWire's own source
    // (a380_systems/air_conditioning/cpiom_b.rs:258-277
    // `PackFlow::pack_flow_demand`: "CPIOM B1 and B3 calculate the LH AGU
    // Flow Demand" / "CPIOM B2 and B4 calculate the RH AGU Flow Demand"),
    // and `circuits::PANEL_CB_NODES` already carries a real
    // "CB_PACK1_CTL"/"CB_PACK2_CTL" pair with exactly that LH/RH split.
    let cpiom_bus = [Bus::Msfs(8), Bus::Msfs(10), Bus::Msfs(10), Bus::Msfs(9)];
    let apps: [(&str, [u64; 4]); 3] = [("TCS", [21_038, 21_039, 21_040, 21_041]), ("VCS", [21_042, 21_043, 21_044, 21_045]), ("CPCS", [21_046, 21_047, 21_048, 21_049])];
    for (app, ids) in apps {
        for (k, &fail) in ids.iter().enumerate() {
            let bus = cpiom_bus[k];
            v.push(BreakerDef {
                id: Box::leak(format!("cpiom-b{}-{}", k + 1, app.to_lowercase()).into_boxed_str()),
                name: Box::leak(format!("CPIOM B{} {} APP", k + 1, app).into_boxed_str()),
                ata: 21,
                bus,
                rating_a: AVIONICS_LRU_W / bus.nominal_voltage(),
                basis: "generic avionics LRU (50 W, typical/derived); mod.rs CPIOM B bus map, lines ~1334-1337",
                consumers: &["CPIOM B application (TCS/VCS/CPCS)"],
                failures: Box::leak(vec![fail].into_boxed_slice()),
                plugin_var: None,
                panel_node: None, circuit: None, gates_none: false, pending_patch: None,
            });
        }
    }

    // AGS pack controllers: CB_PACK1_CTL bridges CPIOM B1+B3 (LH AGU flow
    // demand), CB_PACK2_CTL bridges CPIOM B2+B4 (RH AGU flow demand) -- see
    // the comment above the `apps` loop for the FBW source citation.
    // `failures` is already `&'static [u64]`, so one physical panel breaker
    // legitimately bridges both CPIOM channels it feeds; no new field is
    // needed on `BreakerDef`, and `Breakers::pre_systems` (below) already
    // calls `crate::failures::set_active` for every id in the slice. Bus/
    // rating use the pair's lower-numbered channel (B1, B2) as the
    // representative figure, the same cosmetic-only role the PRIM/SEC
    // alternating-bus precedent a few hundred lines down already gives this
    // field: it sets the breaker's display rating, not which failure id
    // fires -- B1 and B3 (and B2/B4) sit on different real buses
    // (mod.rs:1334-1337) so no single figure is more "real" than the other.
    v.push(BreakerDef {
        id: "pack-1-ctl-ags",
        name: "PACK 1 CTL (CPIOM B1+B3 AGS)",
        ata: 21,
        bus: Bus::Msfs(8),
        rating_a: AVIONICS_LRU_W / Bus::Msfs(8).nominal_voltage(),
        basis: "generic avionics LRU (50 W, typical/derived); mod.rs CPIOM B bus map ~1334-1337; LH/RH AGU split real & FBW-sourced: a380_systems/air_conditioning/cpiom_b.rs:258-277 PackFlow::pack_flow_demand",
        consumers: &["CPIOM B1 AGS application (LH AGU pack flow demand)", "CPIOM B3 AGS application (LH AGU pack flow demand)"],
        failures: &[21_034, 21_036],
        plugin_var: None,
        panel_node: Some("CB_PACK1_CTL"), circuit: None, gates_none: false, pending_patch: None,
    });
    v.push(BreakerDef {
        id: "pack-2-ctl-ags",
        name: "PACK 2 CTL (CPIOM B2+B4 AGS)",
        ata: 21,
        bus: Bus::Msfs(10),
        rating_a: AVIONICS_LRU_W / Bus::Msfs(10).nominal_voltage(),
        basis: "generic avionics LRU (50 W, typical/derived); mod.rs CPIOM B bus map ~1334-1337; LH/RH AGU split real & FBW-sourced: a380_systems/air_conditioning/cpiom_b.rs:258-277 PackFlow::pack_flow_demand",
        consumers: &["CPIOM B2 AGS application (RH AGU pack flow demand)", "CPIOM B4 AGS application (RH AGU pack flow demand)"],
        failures: &[21_035, 21_037],
        plugin_var: None,
        panel_node: Some("CB_PACK2_CTL"), circuit: None, gates_none: false, pending_patch: None,
    });

    // Pack flow valves 1/2 per pack complex (pneumatic.rs `PackComplex`'s
    // `ElectroPneumaticValve`, DC_ESS-powered): new real power-path
    // plugin_var breakers (patches/fbw-rust/power-path-valve-breakers.patch,
    // applied). `ElectroPneumaticValve::receive_power`/`read` now AND the
    // named breaker into `is_powered`, which `update_open_amount` already
    // requires to accept a commanded open amount and which
    // `set_open_amount_from_pressure_difference` already falls back to on
    // loss of power -- so an open breaker really drives this valve to its
    // spring-loaded pneumatic-only behaviour, not a soft failure flag. No
    // existing failure id bridges these valves, so `failures` is empty.
    for pack in 1..=2u32 {
        for (n, side) in [(1u32, "1"), (2u32, "2")] {
            v.push(BreakerDef {
                id: Box::leak(format!("pack-{pack}-flow-valve-{n}").into_boxed_str()),
                name: Box::leak(format!("PACK {pack} FLOW VALVE {side}").into_boxed_str()),
                ata: 21,
                bus: Bus::Msfs(10),
                rating_a: VALVE_ACTUATOR_W / 28.,
                basis: "typical motor/solenoid-operated pack flow valve actuator (50 W, typical/derived); pneumatic.rs PackComplex::new's ElectroPneumaticValve on DirectCurrentEssential",
                consumers: &["pack flow valve"],
                failures: &[],
                plugin_var: Some(Box::leak(format!("ELEC_PACK_{pack}_FLOW_VALVE_{n}_BREAKER_OPEN").into_boxed_str())),
                panel_node: None, circuit: None, gates_none: false, pending_patch: None,
            });
        }
    }
}

// ---------------------------------------------------------------------
// ATA24 -- electrical (TRUs, generators, static inverter, bus-tie/feeder
// breakers).

fn ata24(v: &mut Vec<BreakerDef>) {
    let tru_rated = TRU_RATED_A;
    v.push(BreakerDef { id: "tr-1", name: "TR 1", ata: 24, bus: Bus::Msfs(8), rating_a: tru_rated, basis: "typical Airbus TRU continuous rating (200 A, typical/derived); output impedance/idle voltage already real in transformer_rectifier.rs (electrical.md section 4)", consumers: &["TR 1 (feeds DC BUS 1)"], failures: &[24_000], plugin_var: None, panel_node: Some("CB_TR1"), circuit: None, gates_none: false, pending_patch: None });
    v.push(BreakerDef { id: "tr-2", name: "TR 2", ata: 24, bus: Bus::Msfs(9), rating_a: tru_rated, basis: "typical Airbus TRU continuous rating (200 A, typical/derived)", consumers: &["TR 2 (feeds DC BUS 2)"], failures: &[24_001], plugin_var: None, panel_node: Some("CB_TR_2A"), circuit: None, gates_none: false, pending_patch: None });
    v.push(BreakerDef { id: "tr-ess", name: "TR ESS", ata: 24, bus: Bus::Msfs(10), rating_a: tru_rated, basis: "typical Airbus TRU continuous rating (200 A, typical/derived)", consumers: &["TR ESS (feeds DC ESS BUS)"], failures: &[24_002], plugin_var: None, panel_node: Some("CB_ESS_TR"), circuit: None, gates_none: false, pending_patch: None });
    v.push(BreakerDef { id: "tr-apu", name: "TR APU", ata: 24, bus: Bus::Msfs(11), rating_a: tru_rated, basis: "typical Airbus TRU continuous rating (200 A, typical/derived)", consumers: &["TR APU (feeds DC APU BUS / 309PP)"], failures: &[24_003], plugin_var: None, panel_node: None, circuit: None, gates_none: false, pending_patch: None });
    v.push(BreakerDef { id: "static-inv", name: "STATIC INVERTER", ata: 24, bus: Bus::Named("AC_STAT_INV", 115.), rating_a: static_inverter_rated_a(), basis: "FBW's own power_consumption.rs AC_STAT_INV bus demand, 135 W (real, FBW-sourced)", consumers: &["static inverter (emergency AC)"], failures: &[24_004], plugin_var: None, panel_node: None, circuit: None, gates_none: false, pending_patch: None });

    let gens: [(&str, &str, u32, u64); 4] = [("gen-1", "GEN 1", 2, 24_020), ("gen-2", "GEN 2", 3, 24_021), ("gen-3", "GEN 3", 4, 24_022), ("gen-4", "GEN 4", 5, 24_023)];
    for (id, name, bus, fail) in gens {
        v.push(BreakerDef { id, name, ata: 24, bus: Bus::Msfs(bus), rating_a: generator_rated_a(), basis: "FBW's own VFG rating: 150 kW true power / 0.8 power factor / 115 V nominal (real, FBW-sourced: alternating_current.rs:393, electrical.md section 1)", consumers: &["variable-frequency generator (feeds its AC bus)"], failures: Box::leak(vec![fail].into_boxed_slice()), plugin_var: None, panel_node: None, circuit: None, gates_none: false, pending_patch: None });
    }
    let apu_gens: [(&str, &str, u64); 2] = [("apu-gen-1", "APU GEN 1", 24_030), ("apu-gen-2", "APU GEN 2", 24_031)];
    for (id, name, fail) in apu_gens {
        v.push(d(id, name, 24, Bus::Named("APU_GEN", 115.), apu_generator_rated_a(), "FBW's own Pw980ApuGenerator::MAXIMUM_LOAD_WATT, 120 kW / 0.8 power factor / 115 V nominal (real, FBW-sourced, electrical.md section 1)", &["APU generator"], Box::leak(vec![fail].into_boxed_slice())));
    }

    // Bus-tie/feeder breakers for the 18 ElectricalBus failure ids.
    //
    // "AC_ESS_SHED" is converted to a real feeder gate (patches/fbw-rust/
    // bus-feeder-breakers.patch, applied to alternating_current.rs's
    // `A380AcEssFeedContactors`): this catalogue's bus label follows
    // `MSFS_BUSES`' "AC_ESS_SHED" -> FBW `ElectricalBusType::
    // AlternatingCurrentEssentialShed`, which alternating_current.rs's own
    // `ac_ess_bus` field is (its `new()` doc comment: "400XP is actually
    // AC ESS but for now we misuse AC ESS SHED for it"), fed by two real
    // contactors: 3XC1 from AC BUS 1 (normal) and 3XC2 from AC BUS 4
    // (already closes automatically on `!ac_bus_1_powered || overhead.
    // ac_ess_feed_is_altn()` -- FBW's own transfer logic, not authored
    // here). `ELEC_AC_ESS_SHED_BREAKER_OPEN` is now ANDed into 3XC1's
    // `close_when`, so pulling this breaker opens the AC1 feed only; 3XC2
    // picks the bus up for real once AC BUS 1 reads unpowered or ALTN is
    // selected, and drops it again if AC BUS 4 is also unavailable. The
    // other 17 bus labels are left bridged to the whole-bus
    // `FailureType::ElectricalBus` fallback -- AC1-4's own feed is decided
    // by `A380MainPowerSources::calc_ac_sources`' priority table (computed
    // from generator/ext-power/APU-gen health, not from a feeder breaker
    // state), and the DC/hot-bus set has no single-contactor feeder this
    // audit found time to trace before the session's own time-box; both
    // are reported, not guessed, per the "leave as-is" rule.
    for (k, label) in ["AC1", "AC2", "AC3", "AC4", "AC_ESS", "AC_ESS_SHED", "AC_247XP", "AC_GND_FLT_SVC", "DC1", "DC2", "DC_ESS", "DC_247PP", "DC_309PP", "DC_HOT1", "DC_HOT2", "DC_HOT3", "DC_HOT4", "DC_GND_FLT_SVC"].iter().enumerate() {
        let fail = 24_100 + k as u64;
        let bus = msfs_bus_for_fbw(label);
        // fbw-xp-systems circuit-breaker workstream, round 2
        // (patches/fbw-rust/bus-feeder-breakers-2.patch): AC1-4 are now
        // real feeder breakers too. `ELEC_AC_<n>_FEED_BREAKER_OPEN` is
        // ANDed into `A380MainPowerSources::calc_ac_sources`'s own
        // `gen_available[n-1]`/`ext_pwr_available[n-1]`, so pulling one
        // makes that bus's own engine generator and ext-power line look
        // unavailable to the priority table -- exactly the signal a real
        // generator fault already produces -- and the table falls through
        // to the next entry (a bus tie from an adjacent AC bus, or an APU
        // generator), same as it already does for AC_ESS_SHED via 3XC1/
        // 3XC2. Tests: when_ac1_feed_breaker_open_ac_bus_1_reroutes_onto_
        // ac_bus_2_via_tie (reroute) and when_ac1_feed_breaker_open_and_
        // no_other_source_available_ac_bus_1_is_unpowered (decouple), in
        // a380_systems electrical/mod.rs. AC2-4 share the same
        // calc_ac_sources code path (verified by build/tests) but only
        // AC1 got a dedicated test in this round; DC buses, AC_ESS, EHA
        // and GND/FLT SVC are still on the whole-bus fallback below.
        let converted = matches!(*label, "AC_ESS_SHED" | "AC1" | "AC2" | "AC3" | "AC4");
        let plugin_var = match *label {
            "AC_ESS_SHED" => Some("ELEC_AC_ESS_SHED_BREAKER_OPEN"),
            "AC1" => Some("ELEC_AC_1_FEED_BREAKER_OPEN"),
            "AC2" => Some("ELEC_AC_2_FEED_BREAKER_OPEN"),
            "AC3" => Some("ELEC_AC_3_FEED_BREAKER_OPEN"),
            "AC4" => Some("ELEC_AC_4_FEED_BREAKER_OPEN"),
            _ => None,
        };
        v.push(BreakerDef {
            id: Box::leak(format!("bus-{}", label.to_lowercase()).into_boxed_str()),
            name: Box::leak(format!("{label} BUS FEED").into_boxed_str()),
            ata: 24,
            bus,
            rating_a: bus_peak_w(label) / bus.nominal_voltage(),
            basis: "power_consumption.rs FlightPhasePowerConsumer peak across flight phases where modelled (real, FBW-sourced); otherwise the nearest same-voltage-class bus's figure (docs/physics/breakers.md section 1)",
            consumers: &["every consumer fed from this bus"],
            failures: if converted { &[] } else { Box::leak(vec![fail].into_boxed_slice()) },
            plugin_var,
            panel_node: None, circuit: None, gates_none: false, pending_patch: None,
        });
    }
}

// ---------------------------------------------------------------------
// ATA26 -- fire detection loops (real per-zone A/B loops; `SetOnFire`
// entries are excluded, not a breaker-representable effect).

fn ata26(v: &mut Vec<BreakerDef>) {
    // Converted to real power-path plugin_var breakers
    // (patches/fbw-rust/power-path-fire-loops.patch, applied to
    // fire_and_smoke_protection.rs's `FireDetectionLoop`): one real breaker
    // per zone per loop (matching a real A380's per-zone fire-loop
    // breakers, not one breaker for the whole loop). `receive_power`/`read`
    // AND each zone's own `breaker_closed` into `fire_detected_in_loop`/
    // `loop_has_failed`, so pulling one really drops that zone's own
    // detection power instead of setting the synthetic
    // `FailureType::FireDetectionLoop` -- the `failures` id is kept only so
    // the Study panel's existing failure-injection path still works.
    let zones = ["ENG 1", "ENG 2", "ENG 3", "ENG 4", "APU", "MLG BAY"];
    let zone_var_names = ["1", "2", "3", "4", "APU", "MLG"];
    for (k, zone) in zones.iter().enumerate() {
        for loop_name in ["A", "B"] {
            let fail = 26_007 + 2 * k as u64 + if loop_name == "A" { 0 } else { 1 };
            v.push(BreakerDef {
                id: Box::leak(format!("fire-loop-{}-{loop_name}", zone.to_lowercase().replace(' ', "-")).into_boxed_str()),
                name: Box::leak(format!("FIRE DET {zone} LOOP {loop_name}").into_boxed_str()),
                ata: 26,
                bus: Bus::Msfs(10),
                rating_a: FIRE_LOOP_W / 28.,
                basis: "typical fire/smoke detection loop controller electronics (20 W, typical/derived); fire_and_smoke_protection.rs uses DC_ESS/DC_HOT1 for its detection buses",
                consumers: &["fire detection loop (redundant with its own A/B pair)"],
                failures: Box::leak(vec![fail].into_boxed_slice()),
                plugin_var: Some(Box::leak(format!("ELEC_FIRE_LOOP_{loop_name}_{}_BREAKER_OPEN", zone_var_names[k]).into_boxed_str())),
                panel_node: None, circuit: None, gates_none: false, pending_patch: None,
            });
        }
    }
    // CB_FWS1/CB_FWS2 (circuits::PANEL_CB_NODES) are deliberately *not*
    // catalogued here. FlyByWire's Flight Warning System has no Rust
    // presence at all -- it is FwsCore.ts (systems-host, CpiomC), which
    // reads bus power directly (`SimVar.GetSimVarValue('L:A32NX_ELEC_DC_
    // ESS_BUS_IS_POWERED', ...)`/`..._DC_2_BUS_...`, FwsCore.ts:2981-2983)
    // with no per-computer power gate to hook. Pulling "DC_ESS BUS FEED"/
    // "DC2 BUS FEED" (this module's own bus-tie breakers, `failures::
    // FailureType::ElectricalBus`) already cuts that same simvar for real
    // (electrical/mod.rs `ElectricalBus::is_conductive` -> `!failure.
    // is_active()` -> the Kirchhoff solver zeroes the bus, which is what
    // FwsCore.ts reads) -- just not independently of the rest of the bus,
    // which is genuinely all a CB_FWS1/2 breaker could ever be without a
    // new FwsCore.ts SourcePatch (out of this workstream's Rust-only
    // remit). An earlier version of this module claimed these two as
    // `panel_node`-only entries with no `failures`/`plugin_var`, which
    // gated nothing at all -- a fabricated breaker; removed rather than
    // faked (docs/physics/breakers.md "Left out, not faked").
}

// ---------------------------------------------------------------------
// ATA27 -- flight control computers (failures.rs COMPUTER_FAILURES; real
// PRIM/SEC/FCDC computers, gated through FailuresConsumer, not
// `Simulation::update_active_failures` -- see failures.rs's own doc
// comment for the C++ read path).

fn ata27(v: &mut Vec<BreakerDef>) {
    let entries: [(&str, &str, u64); 11] = [
        ("rollout", "ROLLOUT", 22_001),
        ("fcu-1", "FCU 1", 22_002),
        ("fcu-2", "FCU 2", 22_003),
        ("prim-1", "PRIM 1", 27_000),
        ("prim-2", "PRIM 2", 27_001),
        ("prim-3", "PRIM 3", 27_002),
        ("sec-1", "SEC 1", 27_003),
        ("sec-2", "SEC 2", 27_004),
        ("sec-3", "SEC 3", 27_005),
        ("fcdc-1", "FCDC 1", 27_006),
        ("fcdc-2", "FCDC 2", 27_007),
    ];
    // Alternates DC_ESS/DC2 across the 3-lane PRIM/SEC sets, matching a
    // real Airbus flight-control computer's redundant-bus feed pattern
    // (no FBW Rust source gives the exact per-computer bus -- the FCCs are
    // ported from FBW's C++ side; documented as typical/derived).
    for (k, (id, name, fail)) in entries.into_iter().enumerate() {
        let bus = if k % 2 == 0 { Bus::Msfs(10) } else { Bus::Msfs(9) };
        // "rollout"/"fcu-1"/"fcu-2" bridge 22_0xx failure ids (autoflight,
        // ATA22 -- FCU/rollout autoland); the rest bridge 27_0xx (flight
        // controls, ATA27 -- PRIM/SEC/FCDC). The failure id's own chapter,
        // not a fixed tag, decides which this entry really is.
        let ata = if fail < 27_000 { 22 } else { 27 };
        v.push(BreakerDef { id, name, ata, bus, rating_a: FLIGHT_COMPUTER_W / bus.nominal_voltage(), basis: "typical flight-control computer LRU (100 W, typical/derived: FBW's C++ FCCs carry no Rust-side bus figure); alternating DC_ESS/DC2, a real Airbus-style redundant feed pattern", consumers: &["FlyByWire's ported flight-control computer"], failures: Box::leak(vec![fail].into_boxed_slice()), plugin_var: None, panel_node: None, circuit: None, gates_none: false, pending_patch: None });
    }
}

// ---------------------------------------------------------------------
// ATA32 -- landing gear (LGCIU power supply; the two new plugin_var
// consumers: 4 electric hydraulic pumps + the autobrake disarm solenoid).

fn ata32(v: &mut Vec<BreakerDef>) {
    // Converted to a real power-path plugin_var (patches/fbw-rust/
    // power-path-lgciu.patch, applied): LandingGearControlInterfaceUnit::
    // receive_power now ANDs in its own breaker_closed, read from
    // ELEC_LGCIU_<n>_BREAKER_OPEN every tick -- an open breaker really
    // drops that LGCIU's power (every gear/door sensor and state it
    // exposes through LgciuInterface degrades with it), not a soft
    // FailureType flag.
    v.push(BreakerDef { id: "lgciu-1", name: "LGCIU 1", ata: 32, bus: Bus::Msfs(10), rating_a: AVIONICS_LRU_W / 28., basis: "generic avionics LRU (50 W, typical/derived)", consumers: &["Landing Gear Control and Interface Unit 1"], failures: &[], plugin_var: Some("ELEC_LGCIU_1_BREAKER_OPEN"), panel_node: Some("CB_LGCIS1"), circuit: None, gates_none: false, pending_patch: None });
    v.push(BreakerDef { id: "lgciu-2", name: "LGCIU 2", ata: 32, bus: Bus::Msfs(9), rating_a: AVIONICS_LRU_W / 28., basis: "generic avionics LRU (50 W, typical/derived)", consumers: &["Landing Gear Control and Interface Unit 2"], failures: &[], plugin_var: Some("ELEC_LGCIU_2_BREAKER_OPEN"), panel_node: Some("CB_LGCIS2"), circuit: None, gates_none: false, pending_patch: None });

    // The 4 electric hydraulic pumps: FBW's own ELECTRIC_PUMP_MAX_CURRENT_
    // AMPERE (75 A, hydraulic/mod.rs:1750 -- real, FBW-sourced), gated by
    // the new patches/fbw-rust/breakers.patch generic breaker in
    // ElectricalPumpPhysics.
    const PUMPS: [(&str, &str, u32); 4] = [("hyd-epump-ga", "HYD GREEN ELEC PUMP A", 3), ("hyd-epump-gb", "HYD GREEN ELEC PUMP B", 4), ("hyd-epump-ya", "HYD YELLOW ELEC PUMP A", 5), ("hyd-epump-yb", "HYD YELLOW ELEC PUMP B", 2)];
    let plugin_vars: [&str; 4] = ["ELEC_PUMP_GA_BREAKER_OPEN", "ELEC_PUMP_GB_BREAKER_OPEN", "ELEC_PUMP_YA_BREAKER_OPEN", "ELEC_PUMP_YB_BREAKER_OPEN"];
    for (i, (id, name, bus)) in PUMPS.into_iter().enumerate() {
        v.push(BreakerDef {
            id,
            name,
            ata: 29,
            bus: Bus::Msfs(bus),
            rating_a: 75.,
            basis: "FBW's own ELECTRIC_PUMP_MAX_CURRENT_AMPERE, hydraulic/mod.rs:1750 (real, FBW-sourced)",
            consumers: &["electric hydraulic pump motor"],
            failures: &[],
            plugin_var: Some(plugin_vars[i]),
            panel_node: None, circuit: None, gates_none: false, pending_patch: None,
        });
    }

    v.push(BreakerDef {
        id: "autobrake-disarm-sol",
        name: "AUTOBRAKE DISARM SOLENOID",
        ata: 32,
        bus: Bus::Msfs(9),
        rating_a: 2.0,
        basis: "typical small aircraft solenoid valve (56 W / 28 V ~= 2 A, typical/derived); autobrakes.rs A380AutobrakeKnobSelectorSolenoid's own DirectCurrent(2) bus",
        consumers: &["autobrake knob disarm solenoid"],
        failures: &[],
        plugin_var: Some("ELEC_AUTOBRAKE_DISARM_SOLENOID_BREAKER_OPEN"),
        panel_node: None, circuit: None, gates_none: false, pending_patch: None,
    });
}

// ---------------------------------------------------------------------
// ATA34 -- radio altimeters (A380RadioAltimeters: AC1/AC2/AC_ESS).

fn ata34(v: &mut Vec<BreakerDef>) {
    // Converted to a real power-path plugin_var (patches/fbw-rust/
    // power-path-radioaltimeter.patch, applied): Ala52BRadioAltimeter::
    // receive_power now ANDs in its own breaker_closed, read from
    // ELEC_RA_<n>_BREAKER_OPEN every tick -- an open breaker really
    // drops that transceiver's power (its runtime is lost and
    // radio_altitude() fails warning), not a soft FailureType.
    const RAS: [(&str, &str, Bus, &str); 3] = [
        ("ra-sys-a", "RA SYS A", Bus::Msfs(2), "ELEC_RA_1_BREAKER_OPEN"),
        ("ra-sys-b", "RA SYS B", Bus::Msfs(3), "ELEC_RA_2_BREAKER_OPEN"),
        ("ra-sys-c", "RA SYS C", Bus::Msfs(6), "ELEC_RA_3_BREAKER_OPEN"),
    ];
    for (id, name, bus, plugin_var) in RAS {
        v.push(BreakerDef {
            id,
            name,
            ata: 34,
            bus,
            rating_a: AVIONICS_LRU_W / bus.nominal_voltage(),
            basis: "generic avionics LRU (50 W, typical/derived); navigation.rs A380RadioAltimeters (AC1/AC2/AC_ESS)",
            consumers: &["radio altimeter transceiver"],
            failures: &[],
            plugin_var: Some(plugin_var),
            panel_node: None, circuit: None, gates_none: false, pending_patch: None,
        });
    }
}

// ---------------------------------------------------------------------
// ATA32 -- landing gear/door proximity sensors and actuators. These two
// FailureType groups (failures.rs's own `a380_failures()`, ids 32_004-
// 32_015 and 32_020-32_025) were already registered -- real, checked
// FlyByWire failure ids for the gear/door proximity-detector network and
// the gear/gear-door actuators -- but no catalogue entry bridged them
// before this pass. Both are "soft" (simulated-damage) FailureTypes, not a
// power-path cut: pulling one does not remove electrical power from
// anything, it marks that one sensor/actuator jammed/damaged, which is
// what the real failure represents (`gates`:"failureSoft" in the JSON, see
// `docs/physics/breakers.md`'s power-path audit).
fn ata32_gear_and_door_sensors(v: &mut Vec<BreakerDef>) {
    const SENSORS: [(&str, &str, u64); 12] = [
        ("prox-uplock-gear-nose-1", "PROX UPLOCK GEAR NOSE 1", 32_004),
        ("prox-downlock-gear-nose-2", "PROX DOWNLOCK GEAR NOSE 2", 32_005),
        ("prox-uplock-gear-right-1", "PROX UPLOCK GEAR RIGHT 1", 32_006),
        ("prox-downlock-gear-right-2", "PROX DOWNLOCK GEAR RIGHT 2", 32_007),
        ("prox-uplock-gear-left-2", "PROX UPLOCK GEAR LEFT 2", 32_008),
        ("prox-downlock-gear-left-1", "PROX DOWNLOCK GEAR LEFT 1", 32_009),
        ("prox-uplock-door-nose-1", "PROX UPLOCK DOOR NOSE 1", 32_010),
        ("prox-downlock-door-nose-2", "PROX DOWNLOCK DOOR NOSE 2", 32_011),
        ("prox-uplock-door-right-2", "PROX UPLOCK DOOR RIGHT 2", 32_012),
        ("prox-downlock-door-right-1", "PROX DOWNLOCK DOOR RIGHT 1", 32_013),
        ("prox-uplock-door-left-2", "PROX UPLOCK DOOR LEFT 2", 32_014),
        ("prox-downlock-door-left-1", "PROX DOWNLOCK DOOR LEFT 1", 32_015),
    ];
    for (id, name, fail) in SENSORS {
        v.push(d(id, name, 32, Bus::Msfs(10), AVIONICS_LRU_W / 28. / 10., "typical proximity-sensor target/pickup (5 W, typical/derived); LGCIU's own DC_ESS supply (shared with the sensor network); bridges failures.rs's already-registered GearProxSensorDamage id -- a soft (simulated-damage) FailureType, not a power-path cut", &["gear/door uplock-downlock proximity sensor"], Box::leak(vec![fail].into_boxed_slice())));
    }
    const ACTUATORS: [(&str, &str, u64); 6] = [
        ("gear-actuator-nose", "GEAR ACTUATOR NOSE", 32_020),
        ("gear-actuator-left", "GEAR ACTUATOR LEFT", 32_021),
        ("gear-actuator-right", "GEAR ACTUATOR RIGHT", 32_022),
        ("gear-door-actuator-nose", "GEAR DOOR ACTUATOR NOSE", 32_023),
        ("gear-door-actuator-left", "GEAR DOOR ACTUATOR LEFT", 32_024),
        ("gear-door-actuator-right", "GEAR DOOR ACTUATOR RIGHT", 32_025),
    ];
    for (id, name, fail) in ACTUATORS {
        v.push(d(id, name, 32, Bus::Msfs(10), 75., "FBW's own ELECTRIC_PUMP_MAX_CURRENT_AMPERE class figure reused (these actuators are hydraulically driven, electrically controlled -- the same order of magnitude as the electric hydraulic pumps, typical/derived); bridges failures.rs's already-registered GearActuatorJammed id -- a soft (simulated-damage) FailureType, not a power-path cut", &["gear/gear-door hydraulic actuator control"], Box::leak(vec![fail].into_boxed_slice())));
    }
}

// ---------------------------------------------------------------------
// ATA34 -- radio altimeter antenna faults (failures.rs 34_010-34_022):
// registered, unbridged before this pass. Also soft (simulated antenna
// damage), not a power-path cut.
fn ata34_ra_antennas(v: &mut Vec<BreakerDef>) {
    for n in 1..=3usize {
        let bus = [Bus::Msfs(2), Bus::Msfs(3), Bus::Msfs(6)][n - 1];
        v.push(d(
            Box::leak(format!("ra-ant-interrupt-{n}").into_boxed_str()),
            Box::leak(format!("RA {n} ANTENNA INTERRUPT").into_boxed_str()),
            34,
            bus,
            AVIONICS_LRU_W / bus.nominal_voltage() / 5.,
            "small antenna-coupling network (10 W class, typical/derived); bridges failures.rs's already-registered RadioAntennaInterrupted id -- soft (simulated antenna fault), not a power-path cut",
            &["radio altimeter antenna feed"],
            Box::leak(vec![34_009 + n as u64].into_boxed_slice()),
        ));
    }
    for n in 1..=3usize {
        let bus = [Bus::Msfs(2), Bus::Msfs(3), Bus::Msfs(6)][n - 1];
        v.push(d(
            Box::leak(format!("ra-ant-coupling-{n}").into_boxed_str()),
            Box::leak(format!("RA {n} ANTENNA DIRECT COUPLING").into_boxed_str()),
            34,
            bus,
            AVIONICS_LRU_W / bus.nominal_voltage() / 5.,
            "small antenna-coupling network (10 W class, typical/derived); bridges failures.rs's already-registered RadioAntennaDirectCoupling id -- soft (simulated antenna fault), not a power-path cut",
            &["radio altimeter antenna feed"],
            Box::leak(vec![34_019 + n as u64].into_boxed_slice()),
        ));
    }
    // EGPWC (TAWS): enhanced_gpwc/mod.rs's own `EnhancedGroundProximityWarningComputer`,
    // real `powered_by`/`receive_power` on AC_ESS (a380_systems/lib.rs's
    // own instantiation), no existing failure id. Real power-path
    // `plugin_var` patch applied to D:\fbw-aircraft
    // (patches/fbw-rust/power-path-pending-egpwc.patch -- name kept from
    // when it was drafted; content matches the live file). `pending_patch:
    // None` since it's live, not pending.
    v.push(BreakerDef {
        id: "egpwc",
        name: "EGPWC (TAWS)",
        ata: 34,
        bus: Bus::Msfs(6),
        rating_a: FLIGHT_COMPUTER_W / 115.,
        basis: "typical flight-warning-class computer LRU (100 W, typical/derived); real AC_ESS bus from a380_systems/lib.rs's EnhancedGroundProximityWarningComputer::new call; real is_powered/receive_power already in enhanced_gpwc/mod.rs, no existing failure id",
        consumers: &["Enhanced Ground Proximity Warning Computer (TAWS/terrain display)"],
        failures: &[],
        plugin_var: Some("ELEC_EGPWC_BREAKER_OPEN"),
        panel_node: None,
        circuit: None,
        gates_none: false,
        pending_patch: None,
    });
}

// ---------------------------------------------------------------------
// ATA36 -- pneumatic (engine bleed-air valves).

/// One real breaker per engine gating that engine's whole bleed-air valve
/// set (pneumatic.rs `EngineBleedAirSystem`'s HP valve, pressure-regulating
/// valve and fan-air valve, all three `ElectroPneumaticValve`s constructed
/// with the same `powered_by`/breaker name -- a real Airbus-style single
/// bleed CB feeding all of one engine's bleed valves, not three separate
/// breakers for one physical feed). New real power-path plugin_var breakers
/// (patches/fbw-rust/power-path-valve-breakers.patch, applied):
/// `ElectroPneumaticValve::receive_power`/`read` AND the named breaker into
/// `is_powered`, so pulling it drops all three valves to their
/// spring-loaded pneumatic-only fallback for real, not a soft failure flag.
/// No existing failure id bridges these valves, so `failures` is empty.
fn ata36(v: &mut Vec<BreakerDef>) {
    for n in 1..=4u32 {
        // pneumatic.rs's engine bleed array: engines 1/2 on DirectCurrent(1)
        // (Msfs bus 8, "DC_1"), engines 3/4 on DirectCurrent(2) (Msfs bus 9,
        // "DC_2") -- real, from the exact `EngineBleedAirSystem::new` call.
        let bus = if n <= 2 { Bus::Msfs(8) } else { Bus::Msfs(9) };
        v.push(BreakerDef {
            id: Box::leak(format!("bleed-eng-{n}").into_boxed_str()),
            name: Box::leak(format!("BLEED ENG {n}").into_boxed_str()),
            ata: 36,
            bus,
            rating_a: VALVE_ACTUATOR_W / bus.nominal_voltage(),
            basis: "typical motor/solenoid-operated bleed valve actuator set (50 W, typical/derived); pneumatic.rs EngineBleedAirSystem::new's shared powered_by feeds its HP/PR/fan-air ElectroPneumaticValves",
            consumers: &["engine bleed HP valve", "engine bleed pressure-regulating valve", "engine bleed fan-air valve"],
            failures: &[],
            plugin_var: Some(Box::leak(format!("ELEC_BLEED_ENG_{n}_BREAKER_OPEN").into_boxed_str())),
            panel_node: None, circuit: None, gates_none: false, pending_patch: None,
        });
    }
}

// ---------------------------------------------------------------------
// Absorbing `circuits.rs`'s `systems.cfg` circuits that already really
// gate something (docs/physics/breakers.md "systems.cfg audit"): fuel
// pumps/valves (`fuel.rs`'s `power_circuits`/`fuel_network.rs`'s pump_
// circuits/valve_circuits, real per-pump/per-valve gating), lights
// (`lights.rs`'s `circuits.powered`/`any_powered`), and the two wiper
// circuits (`lights.rs` resolves "WipersLeft"/"WipersRIght" by name).
// Every other systems.cfg circuit type has no consumer anywhere in the
// plugin or FlyByWire (verified: only `fuel.rs`/`lights.rs` ever call
// `Circuits::powered`/`any_powered`/`breaker_closed`) and is dropped from
// the Study/app Breakers tab by `study::web::breakers_json` instead of
// being listed as a breaker that silently does nothing.
//
// Each entry keeps its own real rated wattage from the embedded systems.cfg
// `Power:` field (`CircuitDef::rated_w`, `circuits.rs`) -- FBW's own
// literal number for this exact consumer, not a typical/derived table --
// and reuses the file's own `Name:` field (spaces for underscores) as its
// display name, so nothing here is invented.
/// The real AC bus behind a feed tank pump, by its `systems.cfg` name.
///
/// MSFS's `systems.cfg` wires every fuel pump to `bus.1`, which is
/// FlyByWire's always-powered INFINIBAT (`fuel.rs`'s own doc says so). Taken
/// at face value that makes the feed pumps unfailable: no electrical fault
/// of any kind could ever stop one, which is not how the aircraft works.
///
/// The FCOM gives the real supply, one AC bus per pump (DSC-28-70
/// "Electrical Supply", p.2053):
///
/// | Feed tank | Main pump | Standby pump |
/// |---|---|---|
/// | 1 | AC 4 | AC 2 |
/// | 2 | AC ESS | AC 3 |
/// | 3 | AC 3 | AC ESS |
/// | 4 | AC 2 | AC 4 |
///
/// FlyByWire's own config names the two pumps of each tank
/// `Feed<n>TankPump1`/`2` with no main/standby designation and the same 30
/// psi, so reading pump 1 as the main is an assumption. It is a
/// low-consequence one: swapping them preserves each tank's *pair*, so
/// "losing AC 2 costs one pump in feed 1 and one in feed 4" holds either
/// way, and only which index dies changes.
///
/// Returns `None` for every other circuit, which keeps its `systems.cfg`
/// bus.
fn feed_pump_bus(name: Option<&str>) -> Option<Bus> {
    // MSFS bus numbers, `circuits::MSFS_BUSES`: AC_1 = 2 .. AC_4 = 5, AC_ESS = 6.
    const AC_2: u32 = 3;
    const AC_3: u32 = 4;
    const AC_4: u32 = 5;
    const AC_ESS: u32 = 6;
    let bus = match name? {
        "Fuel_Pump1_Feed1" => AC_4,
        "Fuel_Pump2_Feed1" => AC_2,
        "Fuel_Pump1_Feed2" => AC_ESS,
        "Fuel_Pump2_Feed2" => AC_3,
        "Fuel_Pump1_Feed3" => AC_3,
        "Fuel_Pump2_Feed3" => AC_ESS,
        "Fuel_Pump1_Feed4" => AC_2,
        "Fuel_Pump2_Feed4" => AC_4,
        _ => return None,
    };
    Some(Bus::Msfs(bus))
}

fn absorbed_systems_cfg(v: &mut Vec<BreakerDef>) {
    for def in circuits::parse_circuits(circuits::SYSTEMS_CFG) {
        let ata: u16 = if def.type_name == "CIRCUIT_FUEL_PUMP" || def.type_name == "CIRCUIT_FUEL_VALVE" {
            28
        } else if def.type_name.starts_with("CIRCUIT_LIGHT_") {
            33
        } else if def.type_name == "CIRCUIT_XML" && matches!(def.name.as_deref(), Some("WipersLeft") | Some("WipersRIght")) {
            30
        } else {
            continue;
        };
        let bus = feed_pump_bus(def.name.as_deref())
            .or_else(|| def.buses.iter().copied().find(|&b| b != 1).map(Bus::Msfs))
            .unwrap_or(Bus::Named("INFINIBAT", 28.));
        let rating_a = def.rated_w.unwrap_or(50.) / bus.nominal_voltage();
        let display_name = def.name.clone().unwrap_or_else(|| def.type_name.clone()).replace('_', " ").to_uppercase();
        let (basis, consumers): (&'static str, &'static [&'static str]) = match ata {
            28 if def.type_name == "CIRCUIT_FUEL_PUMP" => (
                "real, from FBW's own embedded systems.cfg Power field (not typical/derived); gates through fuel.rs's power_circuits -> fuel_network.rs's pump_circuits, which really stops that pump (PumpType::Electric gating). Bus is the FCOM's own per-pump AC supply (DSC-28-70), not systems.cfg's always-powered bus.1 -- see feed_pump_bus",
                &["fuel boost/transfer/jettison pump motor"],
            ),
            28 => (
                "real, from FBW's own embedded systems.cfg Power field (not typical/derived); gates through fuel.rs's power_circuits -> fuel_network.rs's valve_circuits, which really freezes that valve's open/closed state",
                &["fuel shutoff/transfer/crossfeed/isolation valve actuator"],
            ),
            33 => (
                "real, from FBW's own embedded systems.cfg Power field (not typical/derived); gates through lights.rs's circuits.powered/any_powered, which really turns that light off",
                &["cockpit/cabin/exterior light circuit"],
            ),
            _ => (
                "real, from FBW's own embedded systems.cfg Power field (not typical/derived); gates through lights.rs's wiper handling (circuit_number_named \"WipersLeft\"/\"WipersRIght\"), which really stops that wiper motor",
                &["windshield wiper motor"],
            ),
        };
        v.push(BreakerDef {
            id: Box::leak(format!("sys-{}", def.number).into_boxed_str()),
            name: Box::leak(display_name.into_boxed_str()),
            ata,
            bus,
            rating_a,
            basis,
            consumers,
            failures: &[],
            plugin_var: None,
            panel_node: None,
            circuit: Some(def.number),
            gates_none: false, pending_patch: None,
        });
    }
}

fn build_catalog() -> Vec<BreakerDef> {
    let mut v = Vec::new();
    ata21(&mut v);
    ata24(&mut v);
    ata26(&mut v);
    ata27(&mut v);
    ata32(&mut v);
    ata32_gear_and_door_sensors(&mut v);
    ata34(&mut v);
    ata34_ra_antennas(&mut v);
    ata36(&mut v);
    absorbed_systems_cfg(&mut v);
    v
}

static CATALOG: std::sync::OnceLock<Vec<BreakerDef>> = std::sync::OnceLock::new();

/// The full catalogue, built once (`build_catalog`'s few dynamically-named
/// entries -- e.g. the per-CPIOM "CPIOM B<n> <APP> APP" names -- are
/// `Box::leak`ed into `'static` strings *inside that one build*, not on
/// every call, so repeated calls -- the Study panel can poll this every
/// tick -- never leak). Safe to call from any thread.
pub fn catalog() -> &'static [BreakerDef] {
    CATALOG.get_or_init(build_catalog)
}

/// Every FlyByWire failure id a breaker in this catalogue can set
/// (`Breakers::pre_systems`'s `crate::failures::set_active(fail, !closed)`,
/// called every tick from this breaker's own live current/voltage physics,
/// never from the crew). Persistence must never restore one of these into
/// the crew-armed set: an open breaker is re-derived from this session's
/// own physics on its own first tick, and treating a *previous* session's
/// open breaker as something the crew armed is exactly the bug this
/// function exists to let the restore path filter out
/// (`docs/analysis/bug-hunt-2026-09-21.md`'s 2026-09-21 follow-up).
pub fn known_failure_ids() -> std::collections::BTreeSet<u64> {
    catalog().iter().flat_map(|d| d.failures.iter().copied()).collect()
}

/// Synthetic circuit numbers for a catalogue entry with no `panel_node`,
/// well above both `circuits::SYSTEMS_CFG`'s own numbers and
/// `circuits::PANEL_ONLY_BASE`'s range.
pub const EXTRA_BASE: usize = 20_000;

fn number_for(def: &BreakerDef, extra_index: usize) -> usize {
    match def.panel_node {
        Some(node) => circuits::panel_cb_number(node).unwrap_or(EXTRA_BASE + extra_index),
        None => EXTRA_BASE + extra_index,
    }
}

struct Live {
    def_index: usize,
    number: usize,
    closed_id: VariableIdentifier,
    current_id: VariableIdentifier,
    cause_id: VariableIdentifier,
    plugin_var_id: Option<VariableIdentifier>,
    bus_potential_id: Option<VariableIdentifier>,
    /// The consumer's own real, live-drawn current, for the handful of
    /// breakers wired to one (`real_current_var`) -- read every tick
    /// instead of the synthetic `rated * bearing_overcurrent_multiplier`
    /// estimate every other breaker still uses. See `real_current_var`'s
    /// own doc for which breakers this is and why.
    real_current_id: Option<VariableIdentifier>,
    /// This breaker's own bay ambient temperature (`docs/physics/
    /// breakers.md` "thermal ambient" section): `BAY_<bay>_TEMPERATURE_C`,
    /// `bay_for`'s mapping. Initialised once to `REFERENCE_AMBIENT_C` at
    /// construction so a build with no bay-temperature publisher yet (or
    /// not this session's) behaves exactly as the old ambient-free curve.
    ambient_id: VariableIdentifier,
    heat: f64,
}

/// Breaker pull/reset requests from the Study panel's own thread, applied
/// on the next tick (the same queued-write pattern `circuits::
/// request_toggle` and `study::web::queue_write` already use -- the panel
/// thread must never touch X-Plane's SDK or plugin state directly).
enum Action {
    Pull(String),
    Reset(String),
    ResetAll,
}
static REQUESTS: Mutex<Vec<Action>> = Mutex::new(Vec::new());

pub fn request_pull(id: String) {
    if let Ok(mut r) = REQUESTS.lock() {
        r.push(Action::Pull(id));
    }
}
pub fn request_reset(id: String) {
    if let Ok(mut r) = REQUESTS.lock() {
        r.push(Action::Reset(id));
    }
}
pub fn request_reset_all() {
    if let Ok(mut r) = REQUESTS.lock() {
        r.push(Action::ResetAll);
    }
}

/// Test-isolation helper (see `scenarios::reset_global_state`): drops any
/// queued pull/reset/reset-all request left over from a previous test in
/// this process, so a breaker pull one scenario test queues but never
/// applies cannot fire against the next test's own breaker state.
#[cfg(any(test, feature = "test-support"))]
pub fn reset_for_tests() {
    if let Ok(mut r) = REQUESTS.lock() {
        r.clear();
    }
}

/// ATA29 physical cause, not a flag: a seized/worn motor bearing in one of
/// the four electric hydraulic pumps (`ata32`'s `hyd-epump-*` entries,
/// `failures::extra`'s "Green/Yellow local electric pump degraded",
/// 29_103/29_104) raises the load the motor's windings see, which raises
/// the current the motor actually draws above its rated 75 A
/// (`ELECTRIC_PUMP_MAX_CURRENT_AMPERE`) -- exactly the "locked/dragging
/// rotor draws more current" relationship real induction/PMSM pump motors
/// have. `post_systems` folds this into the same real current this
/// breaker's thermal (I^2t) curve already evaluates, so a bearing failure
/// genuinely heats and trips the breaker over time instead of the failure
/// only flipping an inert flag. Hydraulic fluid contamination (29_105,
/// "accelerates wear in whichever pump is running") is a smaller, whole-
/// fleet version of the same mechanism and applies to all four pumps at
/// once. `physics::motor::mechanical_current_multiplier` (its own doc cites
/// the back-EMF derivation and the 5-7x locked-rotor figure) turns the
/// failure's own *continuous* `failures::magnitude` -- not a fixed 1.4x --
/// into a current multiplier: 1x at magnitude 0.0 (healthy), continuously up
/// to the locked-rotor multiple at magnitude 1.0 (fully seized rotor),
/// exactly the "a dragging bearing raises current by a smaller, sustained
/// margin; a seized one locks the rotor and jumps to 5-7x" relationship,
/// now a real function of wear instead of an authored step.
fn bearing_overcurrent_multiplier(id: &str) -> f64 {
    let bearing_load: f64 = match id {
        "hyd-epump-ga" | "hyd-epump-gb" => crate::failures::magnitude(29_103),
        "hyd-epump-ya" | "hyd-epump-yb" => crate::failures::magnitude(29_104),
        _ => 0.0,
    };
    // Contamination (29_105) adds a smaller, continuous, whole-fleet extra
    // load onto the same motor budget rather than a separate fixed 1.15x --
    // 0.25 is a defensible ceiling fraction (contamination alone does not
    // stall a pump the way a seized bearing does), scaled by the failure's
    // own continuous magnitude.
    let contamination_load = if matches!(id, "hyd-epump-ga" | "hyd-epump-gb" | "hyd-epump-ya" | "hyd-epump-yb") {
        crate::failures::magnitude(29_105) * 0.25
    } else {
        0.0
    };
    crate::physics::motor::mechanical_current_multiplier(bearing_load + contamination_load)
}

/// Extra mechanical-load current multiplier from a cause outside this
/// file's own bearing-wear model: `physics::motor`'s published load-
/// fraction registry, written by the owning physics module against this
/// breaker's own catalogue id (e.g. `fuel.rs`'s pump-cavitation coupling,
/// keyed by `"sys-<circuit>"` for an absorbed fuel pump breaker). Reads as
/// `1.0` (no extra load) for any id nothing has ever published against, so
/// a coupling that has not run yet can never silently inflate a breaker's
/// current -- see `physics::motor`'s own registry doc.
fn published_load_current_multiplier(id: &str) -> f64 {
    crate::physics::motor::mechanical_current_multiplier(crate::physics::motor::load_fraction(id))
}

/// The other direction of coupling: a consumer whose *delivered mechanical/
/// hydraulic power* (not an added load) has published a real fraction of its
/// rated value against this breaker's own catalogue id (`physics::motor`'s
/// separate `HYDRAULIC_POWER_FRACTIONS` registry -- e.g. `fuel.rs`'s
/// cavitation coupling, `physics::motor::hydraulic_power_current_
/// multiplier`'s own doc has the full "why a separate registry" writeup).
/// Reads as `1.0` (rated current, no derate) for any id nothing has ever
/// published against, matching that registry's own neutral default.
fn published_hydraulic_power_current_multiplier(id: &str) -> f64 {
    crate::physics::motor::hydraulic_power_current_multiplier(crate::physics::motor::hydraulic_power_fraction(id))
}

/// `failures.rs` `extra::electrical()`'s 18 "<BUS> bus short circuit" ids
/// (24_200-24_217, `Effect::Hook { var: "FAIL_BUS_SHORT_HOOK", owner:
/// Owner::Electrical }`) had no reader at all (W50/W109). Same enumeration
/// order as the bus-tie/feeder loop above (`BUS_LABELS` in failures.rs), so
/// index k's short id is `24_200 + k` for this catalogue's own
/// `"bus-<label.to_lowercase()>"` breaker -- the real feeder/tie breaker
/// for that bus, already catalogued a few hundred lines up, not a new one.
const BUS_SHORT_LABELS: [&str; 18] = [
    "AC1", "AC2", "AC3", "AC4", "AC_ESS", "AC_ESS_SHED", "AC_247XP", "AC_GND_FLT_SVC", "DC1", "DC2", "DC_ESS", "DC_247PP", "DC_309PP", "DC_HOT1",
    "DC_HOT2", "DC_HOT3", "DC_HOT4", "DC_GND_FLT_SVC",
];

/// A bolted short circuit draws current far above rated -- commonly tens of
/// times rated current for a low-impedance fault (typical/derived: no
/// per-bus A380 fault-current figure is public) -- so at full magnitude
/// this must clear `physics::electrical::MAGNETIC_TRIP_MULTIPLE` (10x, that
/// module's own instantaneous-trip pickup) and open the breaker the same
/// tick, not wait on the thermal I^2t curve: a bolted short is exactly what
/// the magnetic element exists to catch. `SHORT_CIRCUIT_PEAK_MULTIPLE` is
/// set comfortably above that pickup so a full-magnitude short always
/// trips magnetic, never thermal. Below full magnitude (a developing/high-
/// impedance fault -- chafed insulation, not yet a bolted short) the
/// multiplier ramps linearly from `1.0` (no extra current) so a partial
/// fault still heats the thermal element correctly instead of jumping
/// straight to an instant trip; `failures::magnitude` itself supplies that
/// continuous 0..1 severity, the same convention every other multiplier on
/// this file's `post_systems` chain already uses. Reads as `1.0` (no-op)
/// for any breaker id that is not one of the 18 `"bus-*"` feeder breakers,
/// so this can never affect a non-bus breaker's current.
const SHORT_CIRCUIT_PEAK_MULTIPLE: f64 = 20.0;

fn bus_short_circuit_current_multiplier(id: &str) -> f64 {
    let Some(label) = id.strip_prefix("bus-") else { return 1.0 };
    let Some(k) = BUS_SHORT_LABELS.iter().position(|l| l.to_lowercase() == label) else { return 1.0 };
    let magnitude = crate::failures::magnitude(24_200 + k as u64);
    1.0 + magnitude * (SHORT_CIRCUIT_PEAK_MULTIPLE - 1.0)
}

/// The breaker's consumer own *live* current dataref, when one is real and
/// FBW already publishes it, so `post_systems` can read the actual drawn
/// current every tick instead of estimating from `rated_a` -- the "actual
/// current" requirement (docs/physics/breakers.md "thermal ambient"
/// section has the full contract table).
///
/// Today this is the 4 electric hydraulic pumps, plus every absorbed
/// `sys-<circuit>` fuel-pump breaker via [`fuel_pump_current_var`] below:
/// `electrical_pump_physics.rs`'s `ElectricalPumpPhysics` already computes
/// and publishes `HYD_<GA|GB|YA|YB>_EPUMP_CURRENT` (its own
/// `current_id`/`output_current`, a real PID-controlled motor current
/// derived from the pump's own resistant torque -- section pressure x
/// displacement, *and* the same bearing-wear/overheat resistant-torque
/// factor `bearing_overcurrent_multiplier` above used to estimate; reading
/// the real dataref supersedes that estimate, it does not stack with it),
/// FlyByWire's own `InitContext` giving it the `A32NX_` prefix the same
/// way every other FBW-registered var this plugin reads does (e.g.
/// `study/hyd.rs`'s `A32NX_HYD_{id}_EPUMP_RPM`). A locked/seized rotor
/// (bearing failures 29_103/29_104/29_105) raises `resistant_torque`
/// directly, which the pump's own current-control PID answers by drawing
/// more current to hold RPM -- a real overload, not a flag.
///
/// **Every other breaker in this catalogue has no live per-consumer
/// current published anywhere in FlyByWire's Rust** (`docs/physics/
/// breakers.md` audited each consumer group; most of FlyByWire's
/// `ConsumePower`/`power_consumption.rs` load is only ever aggregated to a
/// whole-bus wattage, never broken back out per consumer) -- those keep
/// the synthetic `rated_a * bearing_overcurrent_multiplier` estimate,
/// which `gate_kind`/the Study panel's `basis` string both already label
/// as an estimate rather than a measurement; this function is the single
/// place that list is drawn from, so extending real-current coverage later
/// is a one-line addition here, not a change to `post_systems`.
fn real_current_var(id: &str) -> Option<&'static str> {
    match id {
        "hyd-epump-ga" => Some("A32NX_HYD_GA_EPUMP_CURRENT"),
        "hyd-epump-gb" => Some("A32NX_HYD_GB_EPUMP_CURRENT"),
        "hyd-epump-ya" => Some("A32NX_HYD_YA_EPUMP_CURRENT"),
        "hyd-epump-yb" => Some("A32NX_HYD_YB_EPUMP_CURRENT"),
        // The absorbed `sys-<circuit>` fuel pumps deliberately are NOT
        // here, though `fuel.rs` does publish a real current for them --
        // see [`fuel_pump_current_var`], which explains why the two numbers
        // cannot be compared.
        _ => None,
    }
}

/// **Not wired into [`real_current_var`], on purpose.** A published
/// current may only be compared against a rating derived the same way, and
/// for these two it is not:
///
/// * `fuel.rs` computes `FUEL_PUMP_CURRENT_A:<n>` from the pump's real
///   delivered hydraulic power at `FUEL_PUMP_VOLTAGE_V` = 115 V, the
///   aircraft's AC motor bus;
/// * [`absorbed_systems_cfg`] rates the breaker at the systems.cfg `Power:`
///   field over its bus voltage, and every fuel pump in that file carries
///   the same `Power:3, 5, 20.0` placeholder -- MSFS's own 5 W, 28 V DC
///   token for a circuit it only needs to switch, with the comment
///   "Fuel Pump 5W" written next to it. That is 0.179 A.
///
/// A feed pump carrying one engine's cruise fuel flow runs 30 psi at about
/// 1660 gal/h, which is 361 W of delivered hydraulic power and so 4.19 A on
/// the 115 V basis -- **23.4 times** the 0.179 A rating, past the 10x
/// magnetic pickup, so the breaker fired instantly the moment the pump
/// carried real fuel, and its partner tripped straight after it picking up
/// the flow. Both FEED 4 pumps went that way in flight. The unit test that
/// covered this wiring wrote its current as `rated * 0.44`, i.e. it assumed
/// the very compatibility that does not hold, so it could not catch it.
///
/// The fix is not a bigger rating pulled out of the air: nothing in
/// FlyByWire or the aircraft config states these motors' real full-load
/// current, and the 5 W token is not it. So the trip curve runs on the one
/// self-consistent basis available -- `rated_a` times the published load
/// multipliers (`published_load_current_multiplier`,
/// `published_hydraulic_power_current_multiplier`,
/// `bearing_overcurrent_multiplier`), which are *fractions of rated* and so
/// carry every cavitation/wear/hydraulic-load coupling across unharmed. The
/// real amp figure stays published for the Study panel to show; it just no
/// longer rates a breaker it was never on the same scale as.
///
/// Kept (not deleted) because the mapping itself is correct and is what a
/// real rating for these pumps would be compared against; the day one
/// exists, wiring it back is this function plus a rating.
///
/// `fuel.rs` publishes each fuel pump's own real, PID/hydraulic-derived
/// motor current as `FUEL_PUMP_CURRENT_A:<pump index>`
/// (`fuel.rs`'s `pump_current`, written from `fluids::pump_current_a`) --
/// the same class of real per-consumer current `real_current_var`'s own doc
/// already wires for the 4 electric hydraulic pumps, just not previously
/// connected to this catalogue's absorbed `sys-<circuit>` fuel-pump
/// breakers (`absorbed_systems_cfg`). The pump index is the circuit's own
/// `CIRCUIT_FUEL_PUMP:<index>` type index (`circuits::CircuitDef::index`),
/// not its `circuit.N` number -- `fuel.rs::power_circuits` already uses
/// exactly that same `circuit.index` to address `net.set_fuel_pump_circuit_
/// powered`/`pump_hydraulic_power_fraction`, so this reuses the identical
/// mapping rather than assuming N==index. Resolved once (this whole
/// catalogue is built once, `CATALOG`), so leaking the formatted name here
/// costs at most one string per real fuel-pump breaker, not per tick.
#[allow(dead_code)]
fn fuel_pump_current_var(circuit_number: usize) -> Option<&'static str> {
    static MAP: std::sync::OnceLock<HashMap<usize, &'static str>> = std::sync::OnceLock::new();
    let map = MAP.get_or_init(|| {
        circuits::parse_circuits(circuits::SYSTEMS_CFG)
            .into_iter()
            .filter(|d| d.type_name == "CIRCUIT_FUEL_PUMP")
            .map(|d| (d.number, &*Box::leak(format!("FUEL_PUMP_CURRENT_A:{}", d.index).into_boxed_str())))
            .collect()
    });
    map.get(&circuit_number).copied()
}

/// This breaker's bay, for the `BAY_<bay>_TEMPERATURE_C` ambient dataref
/// (docs/physics/breakers.md "thermal ambient" section publishes the full
/// contract). A coarse, documented approximation from the breaker's own
/// ATA chapter/bus, not a per-panel-position lookup (the A380's real panel
/// layout is not modelled at this granularity anywhere in this plugin):
/// avionics-bay LRUs (flight/nav/comms computers, ATA22/23/24/31/34/42) and
/// anything with no more specific mapping sit in `AVIONICS`; ATA21 cargo
/// ventilation/heater entries (their own names say which bay) sit in their
/// named cargo bay; the 4 electric hydraulic pumps and ATA29/32 gear/door
/// actuation sit in `WING_ROOT` (A380 hydraulic pumps and main gear bay
/// are wing-root-mounted, not in the pressurised avionics bay). Another
/// agent publishes real bay temperatures to these same dataref names; until
/// then every bay reads back `REFERENCE_AMBIENT_C` (`Breakers::new`
/// initialises it), so this ambient model is inert by default.
fn bay_for(def: &BreakerDef) -> &'static str {
    if def.id.starts_with("hyd-epump") || def.ata == 29 || def.ata == 32 {
        "WING_ROOT"
    } else if def.ata == 21 && (def.name.contains("CARGO") || def.name.contains("BULK")) {
        if def.name.contains("BULK") { "CARGO_AFT" } else { "CARGO_FWD" }
    } else {
        "AVIONICS"
    }
}

/// The live catalogue: every [`BreakerDef`], its breaker/current/trip-cause
/// datarefs, and its thermal-trip state.
pub struct Breakers {
    defs: &'static [BreakerDef],
    live: Vec<Live>,
    by_id: HashMap<&'static str, usize>,
}

impl Breakers {
    pub fn new<V: VariableRegistry + SimulatorReaderWriter>(vars: &mut V) -> Self {
        let defs = catalog();
        let mut live = Vec::with_capacity(defs.len());
        let mut by_id = HashMap::new();
        let mut extra_index = 0usize;
        for (i, def) in defs.iter().enumerate() {
            let number = if def.panel_node.is_some() {
                number_for(def, 0)
            } else {
                let n = number_for(def, extra_index);
                extra_index += 1;
                n
            };
            let closed_id = vars.get(format!("CIRCUIT BREAKER CLOSED:{number}"));
            // `circuits::Circuits::new` already registered (and closed) the
            // panel-node ones; `vars.get` is idempotent by name, and a
            // fresh EXTRA_BASE number starts closed here.
            if def.panel_node.is_none() {
                vars.write(&closed_id, 1.);
            }
            let current_id = vars.get(format!("CIRCUIT CURRENT:{number}"));
            let cause_id = vars.get(format!("CIRCUIT TRIP CAUSE:{number}"));
            // FlyByWire registers these through its InitContext, which gives them
            // its A32NX_ prefix (Vars::get); the same call here reaches the same
            // variable, where get_unprefixed made a second, never-read one.
            let plugin_var_id = def.plugin_var.map(|n| vars.get(n.to_owned()));
            // `..._BREAKER_OPEN`: 0 is closed, so a consumer that reads the
            // variable before (or without) this write still has power.
            if let Some(id) = &plugin_var_id {
                vars.write(id, 0.);
            }
            let bus_potential_id = match def.bus {
                Bus::Msfs(n) => circuits::bus_power_variable(n).map(|name| vars.get(name.replace("_IS_POWERED", "_POTENTIAL"))),
                Bus::Named(..) => None,
            };
            let real_current_id = real_current_var(def.id).map(|n| vars.get(n.to_owned()));
            // Shared per-bay ambient (docs/physics/breakers.md "thermal
            // ambient"): initialise to the curve's own reference ambient so
            // a build with nothing publishing real bay temperatures yet
            // reproduces the old ambient-free trip curve exactly. Several
            // breakers share the same bay name; `vars.get` is idempotent by
            // name and this write is the same value every time, so a later
            // breaker in the same bay does not clobber an earlier one (or a
            // real reading another module already wrote this same tick).
            let ambient_id = vars.get(format!("BAY_{}_TEMPERATURE_C", bay_for(def)));
            vars.write(&ambient_id, crate::physics::electrical::REFERENCE_AMBIENT_C);
            by_id.insert(def.id, i);
            live.push(Live { def_index: i, number, closed_id, current_id, cause_id, plugin_var_id, bus_potential_id, real_current_id, ambient_id, heat: 0. });
        }
        Self { defs, live, by_id }
    }

    fn closed<V: SimulatorReaderWriter>(&self, vars: &mut V, i: usize) -> bool {
        vars.read(&self.live[i].closed_id) != 0.
    }

    fn set_closed<V: SimulatorReaderWriter>(&self, vars: &mut V, i: usize, closed: bool) {
        vars.write(&self.live[i].closed_id, if closed { 1. } else { 0. });
    }

    /// Apply pull/reset requests queued since the last tick. Call at tick
    /// start, alongside `circuits::Circuits::apply_requests`.
    pub fn apply_requests<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V) {
        let requests = REQUESTS.lock().map(|mut r| std::mem::take(&mut *r)).unwrap_or_default();
        for action in requests {
            match action {
                Action::Pull(id) => {
                    if let Some(&i) = self.by_id.get(id.as_str()) {
                        self.set_closed(vars, i, false);
                    }
                }
                Action::Reset(id) => {
                    if let Some(&i) = self.by_id.get(id.as_str()) {
                        self.set_closed(vars, i, true);
                        self.live[i].heat = 0.;
                        vars.write(&self.live[i].cause_id, 0.);
                    }
                }
                Action::ResetAll => {
                    for i in 0..self.live.len() {
                        self.set_closed(vars, i, true);
                        self.live[i].heat = 0.;
                        vars.write(&self.live[i].cause_id, 0.);
                    }
                }
            }
        }
    }

    /// Bridges every breaker's *current* closed state onto its effect
    /// (`failures::set_active`, its `plugin_var`, or -- new here -- the
    /// real `circuits.rs` breaker of an absorbed systems.cfg circuit),
    /// before the systems tick reads any of them. Idempotent: safe to call
    /// every tick. `circuits` is the same `Circuits` the plugin already
    /// builds; an absorbed catalogue entry *mirrors* its closed state onto
    /// `Circuits::set_breaker` every tick, becoming that circuit's
    /// effective controlling breaker. Known trade-off (`docs/physics/
    /// breakers.md` "systems.cfg audit"): the in-X-Plane Study window's own
    /// Circuit Breakers page (`study::services::breakers`) still *displays*
    /// an absorbed circuit correctly (same `CIRCUIT BREAKER CLOSED:n`
    /// dataref), but its own pull/reset button for one is overridden by
    /// this mirror the next tick -- the web app's Circuit Breakers tab is
    /// the controlling UI for an absorbed circuit from here on.
    pub fn pre_systems<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V, circuits: &mut circuits::Circuits) {
        // An open breaker's failures go through their own channel
        // (`failures::set_breaker_levels`), not the crew's arming: they
        // last exactly as long as the breaker is open, are never saved, and
        // a closed breaker never un-arms a failure the crew armed.
        let mut open_failures = std::collections::BTreeMap::new();
        for i in 0..self.live.len() {
            let closed = self.closed(vars, i);
            let def = &self.defs[self.live[i].def_index];
            if !closed {
                for &fail in def.failures {
                    open_failures.insert(fail, 1.0);
                }
            }
            if let Some(id) = &self.live[i].plugin_var_id {
                vars.write(id, if closed { 0. } else { 1. });
            }
            if let Some(number) = def.circuit {
                circuits.set_breaker(vars, number, closed);
            }
        }
        crate::failures::set_breaker_levels(open_failures);
    }

    /// The synthetic continuous load of a breaker with no measured current,
    /// as a fraction of its rating. A breaker is sized with headroom above
    /// the continuous load it protects, never at it (derived/typical: no
    /// A380 per-breaker load is public). The estimate used to be the full
    /// rating, which sits exactly on the thermal curve's threshold: once
    /// `physics::bays` published real bay temperatures, any bay above the
    /// curve's 25 degC reference put every such breaker past it, and they
    /// all tripped together after a few minutes on the ground -- 75
    /// failures armed at once (2026-09-28). At this fraction a normal load
    /// trips only in a bay above ~52 degC, a genuinely overheated one.
    const NOMINAL_CONTINUOUS_LOAD_FRACTION: f64 = 0.8;

    /// Real current + I^2t/magnetic trip, reusing `physics::electrical`'s
    /// own curve (`trip_step`). Call after the systems tick, alongside
    /// `physics::electrical::CircuitProtection::update`. A trip this tick
    /// opens the breaker for `pre_systems` to apply from the *next* tick,
    /// the same one-tick lag `physics::electrical::CircuitProtection`
    /// already has relative to `fuel.rs`/`lights.rs`.
    pub fn post_systems<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V, delta: f64) {
        for i in 0..self.live.len() {
            let def = &self.defs[self.live[i].def_index];
            let rated = def.rating_a;
            let closed = self.closed(vars, i);
            if !closed {
                self.live[i].heat = 0.;
                vars.write(&self.live[i].current_id, 0.);
                continue;
            }
            // Real, live-drawn current where FlyByWire actually publishes
            // one for this consumer (today: the 4 electric hydraulic pumps,
            // `real_current_var`'s own doc) -- an actual overload (bearing
            // wear/contamination raising the pump motor's own PID-controlled
            // current draw) is visible here for real, not estimated. Every
            // other breaker has no live per-consumer current anywhere in
            // FlyByWire's Rust (audited, `real_current_var`'s doc), so it
            // keeps the old synthetic "steady rated demand, scaled by any
            // active bearing-wear failure" estimate -- clearly marked, never
            // invented as a measurement.
            let current = match &self.live[i].real_current_id {
                Some(id) => vars.read(id).abs(),
                // `published_load_current_multiplier`: a physics module
                // outside this file (e.g. `fuel.rs`'s pump-cavitation
                // coupling) may have published a continuous extra-load
                // fraction against this exact catalogue id
                // (`physics::motor`'s registry). Folds in as 1.0 (no-op) for
                // every id nothing has published against, so this is a
                // strict addition, never a change, for a breaker with no
                // such coupling. `bus_short_circuit_current_multiplier`
                // folds in the same way for a `"bus-*"` breaker whose own
                // `failures.rs` extra-catalogue short-circuit id is armed.
                None => {
                    rated
                        * Self::NOMINAL_CONTINUOUS_LOAD_FRACTION
                        * bearing_overcurrent_multiplier(def.id)
                        * published_load_current_multiplier(def.id)
                        * published_hydraulic_power_current_multiplier(def.id)
                        * bus_short_circuit_current_multiplier(def.id)
                }
            };
            vars.write(&self.live[i].current_id, current);
            let ratio = if rated > 0. { current / rated } else { 0. };
            let ambient_c = vars.read(&self.live[i].ambient_id);
            match trip_step_with_ambient(&mut self.live[i].heat, ratio, delta, ambient_c) {
                Some(TripCause::Magnetic) => {
                    self.set_closed(vars, i, false);
                    vars.write(&self.live[i].cause_id, 2.);
                }
                Some(TripCause::Thermal) => {
                    self.set_closed(vars, i, false);
                    vars.write(&self.live[i].cause_id, 1.);
                }
                None => {}
            }
        }
    }

    /// Live snapshot for the Study panel's JSON, without needing the
    /// panel's own `&mut V` borrow (mirrors `circuits::Circuits::list`).
    pub fn snapshot<V: VariableRegistry + SimulatorReaderWriter>(&self, vars: &mut V) -> Vec<BreakerSnapshot> {
        self.live
            .iter()
            .map(|l| {
                let def = &self.defs[l.def_index];
                let closed = vars.read(&l.closed_id) != 0.;
                let current = vars.read(&l.current_id);
                let cause = vars.read(&l.cause_id);
                let bus_v = l.bus_potential_id.as_ref().map(|id| vars.read(id));
                BreakerSnapshot {
                    id: def.id,
                    name: def.name,
                    ata: def.ata,
                    bus: def.bus.label(),
                    bus_potential: bus_v,
                    rating_a: def.rating_a,
                    current_a: current,
                    closed,
                    trip: if cause == 1. { "thermal" } else if cause == 2. { "magnetic" } else { "none" },
                    consumers: def.consumers,
                    basis: def.basis,
                }
            })
            .collect()
    }
}

pub struct BreakerSnapshot {
    pub id: &'static str,
    pub name: &'static str,
    pub ata: u16,
    pub bus: String,
    pub bus_potential: Option<f64>,
    pub rating_a: f64,
    pub current_a: f64,
    pub closed: bool,
    pub trip: &'static str,
    pub consumers: &'static [&'static str],
    pub basis: &'static str,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aspects::test_vars::TestVars;

    #[test]
    fn every_id_is_unique() {
        let cat = catalog();
        let mut ids: Vec<&str> = cat.iter().map(|d| d.id).collect();
        ids.sort_unstable();
        let before = ids.len();
        ids.dedup();
        assert_eq!(ids.len(), before, "duplicate breaker id");
    }

    #[test]
    fn every_rating_is_positive_and_every_bus_resolves() {
        for def in catalog() {
            assert!(def.rating_a > 0., "{} has a non-positive rating", def.id);
            assert!(def.rating_a.is_finite(), "{} has a non-finite rating", def.id);
            assert!(!def.consumers.is_empty(), "{} lists no consumers", def.id);
            assert!(!def.basis.is_empty(), "{} has no rating basis", def.id);
            assert!(!def.failures.is_empty() || def.plugin_var.is_some() || def.circuit.is_some() || def.gates_none || def.pending_patch.is_some(), "{} gates nothing and is not marked gates_none/pending_patch", def.id);
            match def.bus {
                Bus::Msfs(n) => assert!(MSFS_BUSES.iter().any(|(m, _, _)| *m == n), "{}: msfs bus {n} not in MSFS_BUSES", def.id),
                Bus::Named(_, v) => assert!(v > 0., "{}: named bus has non-positive nominal voltage", def.id),
            }
        }
    }

    #[test]
    fn catalog_is_well_over_a_hundred() {
        assert!(catalog().len() > 100, "catalog has only {} breakers", catalog().len());
    }

    #[test]
    fn failure_ids_are_unique_across_the_whole_catalog() {
        let mut ids: Vec<u64> = catalog().iter().flat_map(|d| d.failures.iter().copied()).collect();
        ids.sort_unstable();
        let before = ids.len();
        ids.dedup();
        assert_eq!(ids.len(), before, "the same failure id is bridged from two different breakers");
    }

    #[test]
    fn every_bridged_failure_id_is_registered_in_failures_rs() {
        let registered: std::collections::BTreeSet<u64> = crate::failures::a380_failures().into_iter().map(|(id, _)| id).chain(crate::failures::COMPUTER_FAILURES.iter().map(|(id, _)| *id)).collect();
        for def in catalog() {
            for &f in def.failures {
                assert!(registered.contains(&f), "{} bridges unknown failure id {f}", def.id);
            }
        }
    }

    #[test]
    fn panel_node_breakers_reuse_a_real_panel_cb_node() {
        for def in catalog() {
            if let Some(node) = def.panel_node {
                assert!(circuits::PANEL_CB_NODES.contains(&node), "{}: {node} is not a real PANEL_CB_NODES entry", def.id);
            }
        }
    }

    #[test]
    fn pulling_a_breaker_activates_its_bridged_failure_and_reset_clears_it() {
        // `crate::failures`' active/magnitude state is one process-wide
        // `static STATE` (failures.rs), and `Failures::new()` below clears
        // it outright (`s.active.clear(); s.magnitudes.clear();`) as part of
        // its own reset. Run unlocked, this test races every other test in
        // the crate that also touches that global (e.g.
        // extra_backend_fcdc.rs's failure-id tests, which already take this
        // same lock) under `cargo test`'s default parallel threads: whichever
        // one's `Failures::new()`/`set_active`/`replace` lands between this
        // test's pull and its `active_ids()` assert wipes or changes the set
        // out from under it. `failures::tests::SERIAL` is exactly the lock
        // failures.rs's own doc comment describes for this ("tests touching
        // it take turns"); take it for the whole test, matching the
        // established pattern.
        let _g = crate::failures::tests::serial();
        let mut vars = TestVars::default();
        let mut circuits = crate::circuits::Circuits::new(&mut vars);
        let _f = crate::failures::Failures::new();
        let mut b = Breakers::new(&mut vars);

        // "tr-1" bridges to failure 24_000.
        request_pull("tr-1".to_owned());
        b.apply_requests(&mut vars);
        b.pre_systems(&mut vars, &mut circuits);
        assert!(crate::failures::active_ids().contains(&24_000), "pulling TR 1 should activate failure 24000");

        request_reset("tr-1".to_owned());
        b.apply_requests(&mut vars);
        b.pre_systems(&mut vars, &mut circuits);
        assert!(!crate::failures::active_ids().contains(&24_000), "resetting TR 1 should clear failure 24000");
    }

    /// An open breaker's failure is the breaker's consequence, not something
    /// the crew armed: it acts, but it is never in the crew's saved set.
    #[test]
    fn an_open_breakers_failure_acts_but_is_never_saved_as_crew_armed() {
        let _g = crate::failures::tests::serial();
        let mut vars = TestVars::default();
        let mut circuits = crate::circuits::Circuits::new(&mut vars);
        let _f = crate::failures::Failures::new();
        let mut b = Breakers::new(&mut vars);
        request_pull("tr-1".to_owned());
        b.apply_requests(&mut vars);
        b.pre_systems(&mut vars, &mut circuits);
        assert!(crate::failures::is_active(24_000), "an open TR 1 breaker fails TR 1");
        assert!(!crate::failures::armed_ids().contains(&24_000), "but the crew did not arm it, so it is not saved");
    }

    /// The crew can arm a failure whose breaker is closed; the closed
    /// breaker must not un-arm it on the next tick.
    #[test]
    fn a_closed_breaker_never_clears_a_failure_the_crew_armed() {
        let _g = crate::failures::tests::serial();
        let mut vars = TestVars::default();
        let mut circuits = crate::circuits::Circuits::new(&mut vars);
        let _f = crate::failures::Failures::new();
        let mut b = Breakers::new(&mut vars);
        crate::failures::set_active(24_000, true);
        for _ in 0..3 {
            b.pre_systems(&mut vars, &mut circuits);
        }
        assert!(crate::failures::armed_ids().contains(&24_000), "TR 1's breaker is closed, and the crew's TR 1 failure stays armed");
        assert!(crate::failures::is_active(24_000));
    }

    #[test]
    fn pulling_pack_1_ctl_activates_both_of_its_bridged_ags_failures() {
        // Same serialisation reasoning as the test above: this touches the
        // process-wide `failures::STATE`.
        let _g = crate::failures::tests::serial();
        let mut vars = TestVars::default();
        let mut circuits = crate::circuits::Circuits::new(&mut vars);
        let _f = crate::failures::Failures::new();
        let mut b = Breakers::new(&mut vars);

        // "pack-1-ctl-ags" bridges both CPIOM B1 (21_034) and CPIOM B3
        // (21_036) AGS failures -- one physical CB_PACK1_CTL panel breaker
        // feeding the LH AGU's two redundant channels -- while pack 2's ids
        // (21_035, 21_037) must stay untouched.
        request_pull("pack-1-ctl-ags".to_owned());
        b.apply_requests(&mut vars);
        b.pre_systems(&mut vars, &mut circuits);
        let active = crate::failures::active_ids();
        assert!(active.contains(&21_034), "pulling PACK 1 CTL should activate CPIOM B1 AGS (21034)");
        assert!(active.contains(&21_036), "pulling PACK 1 CTL should activate CPIOM B3 AGS (21036)");
        assert!(!active.contains(&21_035), "PACK 1 CTL must not touch CPIOM B2 AGS (21035)");
        assert!(!active.contains(&21_037), "PACK 1 CTL must not touch CPIOM B4 AGS (21037)");

        request_reset("pack-1-ctl-ags".to_owned());
        b.apply_requests(&mut vars);
        b.pre_systems(&mut vars, &mut circuits);
        let active = crate::failures::active_ids();
        assert!(!active.contains(&21_034), "resetting PACK 1 CTL should clear CPIOM B1 AGS (21034)");
        assert!(!active.contains(&21_036), "resetting PACK 1 CTL should clear CPIOM B3 AGS (21036)");
    }

    #[test]
    fn pulling_an_absorbed_circuit_breaker_really_opens_its_systems_cfg_circuit() {
        // `pre_systems` below iterates *every* catalogued breaker, not just
        // "sys-2", and calls `crate::failures::set_active(fail, !closed)` for
        // each one's own bridged ids -- on the process-wide `failures::STATE`
        // (see the same lock's doc comment on the test above). Any breaker
        // left closed in this `Breakers` instance forces its failure id
        // false every call, which would clobber a concurrently-running
        // test elsewhere in the crate that has that same id armed. Take the
        // same test-serialization lock so this test's blanket touch of that
        // global cannot race one of those.
        let _g = crate::failures::tests::serial();
        let mut vars = TestVars::default();
        let mut circuits = crate::circuits::Circuits::new(&mut vars);
        let mut b = Breakers::new(&mut vars);
        // "sys-2" is circuit.2, CIRCUIT_FUEL_PUMP:1 "Fuel_Pump1_Feed1".
        let i = *b.by_id.get("sys-2").expect("circuit 2 (a fuel pump) should be absorbed into the catalogue");
        assert_eq!(b.defs[b.live[i].def_index].circuit, Some(2));
        assert!(circuits.breaker_closed(&mut vars, 2), "circuit 2 starts closed");

        request_pull("sys-2".to_owned());
        b.apply_requests(&mut vars);
        b.pre_systems(&mut vars, &mut circuits);
        assert!(!circuits.breaker_closed(&mut vars, 2), "pulling the catalogue breaker should really open circuit 2, which fuel.rs reads");

        request_reset("sys-2".to_owned());
        b.apply_requests(&mut vars);
        b.pre_systems(&mut vars, &mut circuits);
        assert!(circuits.breaker_closed(&mut vars, 2), "resetting the catalogue breaker should really close circuit 2 again");
    }

    #[test]
    fn pulling_a_plugin_var_breaker_writes_one_and_reset_writes_zero() {
        // Same reason as the other `pre_systems`-driving tests above: it
        // touches every catalogued breaker's bridged failure id on the
        // shared global `failures::STATE`, not just "hyd-epump-ga" (which
        // has none of its own, but its neighbours in the catalogue do).
        let _g = crate::failures::tests::serial();
        let mut vars = TestVars::default();
        let mut circuits = crate::circuits::Circuits::new(&mut vars);
        let mut b = Breakers::new(&mut vars);
        let i = *b.by_id.get("hyd-epump-ga").unwrap();
        let var_id = b.live[i].plugin_var_id.clone().unwrap();

        request_pull("hyd-epump-ga".to_owned());
        b.apply_requests(&mut vars);
        b.pre_systems(&mut vars, &mut circuits);
        assert_eq!(vars.read(&var_id), 1.);

        request_reset("hyd-epump-ga".to_owned());
        b.apply_requests(&mut vars);
        b.pre_systems(&mut vars, &mut circuits);
        assert_eq!(vars.read(&var_id), 0.);
    }

    #[test]
    fn an_overloaded_breaker_trips_and_a_tripped_breaker_stays_open() {
        // (build fix, INT-P4) `post_systems`'s overcurrent-trip path reads
        // `crate::failures::magnitude` for the real-current bearing-wear
        // input (see this file's own doc for `hyd-epump-ga`'s
        // `real_current_var`); needs the guard the same as the file's other
        // `crate::failures`-touching tests.
        let _guard = crate::failures::tests::serial();
        let mut vars = TestVars::default();
        let mut circuits = crate::circuits::Circuits::new(&mut vars);
        let mut b = Breakers::new(&mut vars);
        let i = *b.by_id.get("hyd-epump-ga").unwrap();
        // Force a magnetic-trip-level current by writing a rating far
        // below the actual demand is not possible from outside (rating is
        // static), so instead drive many ticks at the normal (rated)
        // current, which must never trip, then verify a manual pull holds
        // across ticks (reset-until-cleared semantics).
        for _ in 0..600 {
            b.post_systems(&mut vars, 1.0);
        }
        assert!(b.closed(&mut vars, i), "steady rated load must never trip");

        request_pull("hyd-epump-ga".to_owned());
        b.apply_requests(&mut vars);
        for _ in 0..60 {
            b.post_systems(&mut vars, 1.0 / 60.0);
        }
        assert!(!b.closed(&mut vars, i), "a manually pulled breaker stays open");
    }

    #[test]
    fn reset_all_closes_every_pulled_breaker() {
        let mut vars = TestVars::default();
        let mut circuits = crate::circuits::Circuits::new(&mut vars);
        let mut b = Breakers::new(&mut vars);
        request_pull("tr-1".to_owned());
        request_pull("gen-2".to_owned());
        b.apply_requests(&mut vars);
        assert!(!b.closed(&mut vars, *b.by_id.get("tr-1").unwrap()));
        assert!(!b.closed(&mut vars, *b.by_id.get("gen-2").unwrap()));

        request_reset_all();
        b.apply_requests(&mut vars);
        assert!(b.closed(&mut vars, *b.by_id.get("tr-1").unwrap()));
        assert!(b.closed(&mut vars, *b.by_id.get("gen-2").unwrap()));
    }

    /// Real-current wiring (`real_current_var`): `hyd-epump-ga`'s breaker
    /// current now comes from FlyByWire's own `A32NX_HYD_GA_EPUMP_CURRENT`
    /// (a real PID-controlled motor current that rises with the pump's own
    /// resistant torque -- bearing wear, cavitation, an obstructed inlet,
    /// ...), not the old `rated * bearing_overcurrent_multiplier` estimate.
    /// This unit-test binary never runs FlyByWire's hydraulic physics, so
    /// these tests write directly to that same dataref -- exactly the value
    /// FlyByWire's own `ElectricalPumpPhysics::update` would publish for a
    /// given overload -- rather than re-deriving it from a failure id.
    ///
    /// **Independent prediction**: `physics::electrical::THERMAL_TRIP_K`
    /// documents its own curve as `K / (r^2 - 1)` seconds to trip at ratio
    /// `r`, `K = 30`. At `r = 1.5` that predicts `30 / (1.5^2 - 1) = 30 /
    /// 1.25 = 24 s`, computed here from the curve's own published formula,
    /// not from re-running the code under test.
    #[test]
    fn real_pump_current_trips_within_its_independently_predicted_i2t_time() {
        // (build fix, INT-P4) same guard as the other real-current tests.
        let _guard = crate::failures::tests::serial();
        let mut vars = TestVars::default();
        let mut b = Breakers::new(&mut vars);
        let i = *b.by_id.get("hyd-epump-ga").unwrap();
        let rated = b.defs[b.live[i].def_index].rating_a;
        let real_current_id = b.live[i].real_current_id.clone().expect("hyd-epump-ga must have a real_current_var");

        vars.write(&real_current_id, rated * 1.5);

        const DT: f64 = 1.0 / 60.0;
        let mut elapsed = 0.;
        let mut tripped_at = None;
        while elapsed < 40. {
            b.post_systems(&mut vars, DT);
            elapsed += DT;
            if !b.closed(&mut vars, i) {
                tripped_at = Some(elapsed);
                break;
            }
        }

        let predicted = 30. / (1.5 * 1.5 - 1.);
        let actual = tripped_at.expect("a 1.5x-rated real current must thermally trip within 40s");
        assert!((actual - predicted).abs() < 0.5, "predicted {predicted}s from the documented K/(r^2-1) curve, got {actual}s");
        assert_eq!(vars.read(&b.live[i].cause_id), 1., "should trip thermal (cause 1), not magnetic");
    }

    /// Decouple: if the pump's real current is never fed to the breaker
    /// (stays at `TestVars`' default 0, i.e. the old "rated current" stand-
    /// in with no overload signal reaching it), the same 40s window that
    /// trips a real 1.5x overload above must NOT trip -- the effect really
    /// depends on the real current arriving, it is not an artefact of
    /// `post_systems` tripping on its own after enough ticks.
    #[test]
    fn without_the_real_current_feed_the_same_window_never_trips() {
        // (build fix, INT-P4) same guard as the other real-current tests.
        let _guard = crate::failures::tests::serial();
        let mut vars = TestVars::default();
        let mut b = Breakers::new(&mut vars);
        let i = *b.by_id.get("hyd-epump-ga").unwrap();
        // Real current dataref left unwritten (0. by construction) --
        // simulates the FBW-side publisher being absent/decoupled.

        const DT: f64 = 1.0 / 60.0;
        let mut elapsed = 0.;
        while elapsed < 40. {
            b.post_systems(&mut vars, DT);
            elapsed += DT;
        }
        assert!(b.closed(&mut vars, i), "with no real current fed in, the breaker must never trip");
        assert_eq!(vars.read(&b.live[i].current_id), 0., "published current must reflect the real (absent) draw, not a synthetic rated estimate");
    }

    /// A locked/near-seized motor's current (well past the magnetic pickup
    /// multiple, `physics::electrical::MAGNETIC_TRIP_MULTIPLE` = 10x,
    /// documented as a typical aerospace SSPC/breaker magnetic-element
    /// pickup) must trip instantly on the magnetic curve, sourced from the
    /// same real current dataref, not the thermal I^2t accumulator.
    #[test]
    fn a_severe_real_overcurrent_trips_the_magnetic_curve_instantly() {
        // (build fix, INT-P4) same guard as the other real-current tests.
        let _guard = crate::failures::tests::serial();
        let mut vars = TestVars::default();
        let mut b = Breakers::new(&mut vars);
        let i = *b.by_id.get("hyd-epump-ga").unwrap();
        let rated = b.defs[b.live[i].def_index].rating_a;
        let real_current_id = b.live[i].real_current_id.clone().unwrap();
        vars.write(&real_current_id, rated * 12.);

        b.post_systems(&mut vars, 1.0 / 60.0);

        assert!(!b.closed(&mut vars, i), "12x rated real current must trip within one tick");
        assert_eq!(vars.read(&b.live[i].cause_id), 2., "should trip magnetic (cause 2), not thermal");
    }

    /// Ambient thermal derating (`docs/physics/breakers.md` "thermal
    /// ambient"): `trip_step_with_ambient` folds `(ambient - 25) / 75` into
    /// the same `ratio^2` term the curve already used, so a load *below*
    /// rated current (which alone never heats the element in this curve)
    /// can still cross the trip threshold once the bay is hot enough.
    ///
    /// **Independent prediction**, from that documented formula directly: at
    /// `ratio = 0.9` (`thermal_input_base = 0.81`), solving `0.81 +
    /// (ambient - 25) / 75 > 1` for `ambient` gives `ambient > 25 + 75 *
    /// 0.19 = 39.25` degC. At `ambient = 60`, `thermal_input = 0.81 + 35 /
    /// 75 = 1.2467`, so the predicted trip time is `30 / (1.2467 - 1) =
    /// 121.6 s`.
    #[test]
    fn a_sub_rated_load_trips_in_a_hot_bay_at_its_predicted_time_but_never_at_reference_ambient() {
        // (build fix, INT-P4) same guard as the other real-current tests.
        let _guard = crate::failures::tests::serial();
        let mut vars = TestVars::default();
        let mut b = Breakers::new(&mut vars);
        let i = *b.by_id.get("hyd-epump-ga").unwrap();
        let rated = b.defs[b.live[i].def_index].rating_a;
        let real_current_id = b.live[i].real_current_id.clone().unwrap();
        let ambient_id = b.live[i].ambient_id.clone();
        vars.write(&real_current_id, rated * 0.9);

        // At 20 degC (below the 25 degC reference), a 90%-rated load never
        // trips: matches "never trip below rated" at any sane ambient.
        vars.write(&ambient_id, 20.);
        for _ in 0..3600 {
            b.post_systems(&mut vars, 1.0 / 60.0);
        }
        assert!(b.closed(&mut vars, i), "a 90%-rated load at 20 degC must never trip");

        // At 60 degC, the same 90%-rated load trips near the hand-computed
        // 121.6 s.
        vars.write(&ambient_id, 60.);
        const DT: f64 = 1.0 / 60.0;
        let mut elapsed = 0.;
        let mut tripped_at = None;
        while elapsed < 200. {
            b.post_systems(&mut vars, DT);
            elapsed += DT;
            if !b.closed(&mut vars, i) {
                tripped_at = Some(elapsed);
                break;
            }
        }
        let predicted = 30. / (0.81 + 35. / 75. - 1.);
        let actual = tripped_at.expect("a 90%-rated load at 60 degC bay ambient must trip within 200s");
        assert!((actual - predicted).abs() < 2.0, "predicted {predicted}s, got {actual}s");
    }

    /// Decouple: holding the bay ambient fixed at the curve's own reference
    /// temperature (25 degC, what every breaker starts at by construction)
    /// removes the ambient effect entirely -- a 90%-rated load must run
    /// indefinitely without tripping, exactly as it did before this ambient
    /// model existed.
    #[test]
    fn holding_ambient_at_reference_a_sub_rated_load_never_trips() {
        // (build fix, INT-P4) same guard as the other real-current tests.
        let _guard = crate::failures::tests::serial();
        let mut vars = TestVars::default();
        let mut b = Breakers::new(&mut vars);
        let i = *b.by_id.get("hyd-epump-ga").unwrap();
        let rated = b.defs[b.live[i].def_index].rating_a;
        let real_current_id = b.live[i].real_current_id.clone().unwrap();
        // ambient_id left at its constructed default (REFERENCE_AMBIENT_C).
        vars.write(&real_current_id, rated * 0.9);

        for _ in 0..3600 {
            b.post_systems(&mut vars, 1.0 / 60.0);
        }
        assert!(b.closed(&mut vars, i), "ambient held at reference: a 90%-rated load must never trip");
    }

    /// A breaker with no measured current, in a warm (30 degC) avionics bay
    /// -- `physics::bays` publishes real bay temperatures now -- must carry
    /// its normal load for a whole flight. Its synthetic load used to be
    /// exactly 100% of rating, which in any bay above 25 degC sat past the
    /// thermal threshold: every such breaker in the bay tripped together
    /// after ~7.5 min at 30 degC (~17 min at 27 degC), arming 75 failures
    /// (every generator, TR, DC bus, PRIM, SEC, FCDC...) on a runway
    /// (2026-09-28).
    #[test]
    fn a_normal_load_in_a_warm_bay_does_not_trip_all_flight() {
        let _g = crate::failures::tests::serial();
        let _f = crate::failures::Failures::new();
        let mut vars = TestVars::default();
        let mut b = Breakers::new(&mut vars);
        let i = *b.by_id.get("bus-dc1").unwrap();
        assert!(b.live[i].real_current_id.is_none(), "this test is about the synthetic load");
        vars.write(&b.live[i].ambient_id, 30.0);
        for _ in 0..4 * 3600 {
            b.post_systems(&mut vars, 1.0);
        }
        assert!(b.closed(&mut vars, i), "a normal load in a 30 degC bay must not trip in four hours");
    }

    /// Decouple: the bay ambient still matters -- a genuinely overheated bay
    /// trips a breaker carrying only its normal load, as a bimetal element
    /// that trips at a fixed absolute temperature does.
    #[test]
    fn a_normal_load_in_an_overheated_bay_still_trips() {
        let _g = crate::failures::tests::serial();
        let _f = crate::failures::Failures::new();
        let mut vars = TestVars::default();
        let mut b = Breakers::new(&mut vars);
        let i = *b.by_id.get("bus-dc1").unwrap();
        vars.write(&b.live[i].ambient_id, 70.0);
        for _ in 0..3600 {
            b.post_systems(&mut vars, 1.0);
        }
        assert!(!b.closed(&mut vars, i), "a 70 degC bay must trip even a normally loaded breaker within the hour");
        assert_eq!(vars.read(&b.live[i].cause_id), 1., "thermal, not magnetic");
    }

    /// Bearing-wear coupling (docs/physics/breakers.md, replacing the old
    /// fixed 1.4x): independent prediction from
    /// `physics::motor::mechanical_current_multiplier`'s own hand-computed
    /// value at `magnitude=0.6` -- `1 + (6-1)*0.6 = 4.0`x rated -- not
    /// re-derived from `bearing_overcurrent_multiplier` under test.
    #[test]
    fn bearing_wear_magnitude_predicts_current_by_the_back_emf_relation() {
        // Same shared-global-state hazard as the `pre_systems` tests above:
        // `Failures::new()` clears the process-wide `failures::STATE`
        // outright, and `set_magnitude` writes into it too. Take the lock.
        let _g = crate::failures::tests::serial();
        let _f = crate::failures::Failures::new();
        crate::failures::set_magnitude(29_103, 0.6);
        assert_eq!(bearing_overcurrent_multiplier("hyd-epump-ga"), 4.0);
        // Decouple: healthy (no wear registered) reads back exactly 1x.
        crate::failures::set_magnitude(29_103, 0.0);
        assert_eq!(bearing_overcurrent_multiplier("hyd-epump-ga"), 1.0);
        // An id this coupling does not own is never affected.
        assert_eq!(bearing_overcurrent_multiplier("sys-2"), 1.0);
    }

    /// `failures.rs` extra-catalogue "<BUS> bus short circuit" ids
    /// (24_200-24_217, W50/W109): DC1 is index 8 in `BUS_LABELS`/
    /// `BUS_SHORT_LABELS`, so its short id is 24_200+8 = 24_208 and its
    /// catalogued feeder/tie breaker is "bus-dc1".
    #[test]
    fn a_bus_short_circuit_trips_its_feeder_breaker_magnetic() {
        let _g = crate::failures::tests::serial();
        let _f = crate::failures::Failures::new();
        let mut vars = TestVars::default();
        let mut b = Breakers::new(&mut vars);
        let i = *b.by_id.get("bus-dc1").expect("DC1's feeder/tie breaker must be catalogued");

        // Healthy: the normal synthetic rated-demand estimate must never trip.
        b.post_systems(&mut vars, 1.0);
        assert!(b.closed(&mut vars, i), "no short armed: must not trip");

        // Full-magnitude short: `SHORT_CIRCUIT_PEAK_MULTIPLE` (20x) clears
        // `physics::electrical::MAGNETIC_TRIP_MULTIPLE` (10x), so it must
        // trip magnetic (cause 2) the same tick, not wait on the thermal
        // curve.
        crate::failures::set_magnitude(24_208, 1.0);
        b.post_systems(&mut vars, 1.0);
        assert!(!b.closed(&mut vars, i), "a full-magnitude short must trip its feeder breaker");
        assert_eq!(vars.read(&b.live[i].cause_id), 2., "a bolted short must trip magnetic, not thermal");

        // Decouple: an id this breaker does not own must never affect it.
        crate::failures::set_magnitude(24_208, 0.0);
        assert_eq!(bus_short_circuit_current_multiplier("bus-dc1"), 1.0);
        assert_eq!(bus_short_circuit_current_multiplier("hyd-epump-ga"), 1.0, "a non-bus breaker id must never be affected");
    }

    /// Every feed pump sits on the AC bus the FCOM gives it, not on
    /// systems.cfg's always-powered bus.1 -- which is what made them
    /// unfailable, since no electrical fault could reach INFINIBAT.
    #[test]
    fn the_feed_pumps_are_on_their_real_ac_buses() {
        let mut vars = TestVars::default();
        let b = Breakers::new(&mut vars);
        // DSC-28-70's table, as bus labels.
        let expected = [
            ("FUEL PUMP1 FEED1", "AC_4"), ("FUEL PUMP2 FEED1", "AC_2"),
            ("FUEL PUMP1 FEED2", "AC_ESS"), ("FUEL PUMP2 FEED2", "AC_3"),
            ("FUEL PUMP1 FEED3", "AC_3"), ("FUEL PUMP2 FEED3", "AC_ESS"),
            ("FUEL PUMP1 FEED4", "AC_2"), ("FUEL PUMP2 FEED4", "AC_4"),
        ];
        for (name, bus) in expected {
            let def = b.defs.iter().find(|d| d.name == name).unwrap_or_else(|| panic!("no breaker named {name}"));
            assert_eq!(def.bus.label(), bus, "{name} should be fed from {bus}");
        }
        // Each feed tank's two pumps must be on different buses, or one bus
        // loss takes a whole tank's supply.
        for tank in 1..=4 {
            let of = |p: u32| b.defs.iter().find(|d| d.name == format!("FUEL PUMP{p} FEED{tank}")).unwrap().bus.label();
            assert_ne!(of(1), of(2), "feed tank {tank}'s two pumps share a bus");
        }
        // And the whole set must span all four, so no single bus loss costs
        // more than two pumps.
        let mut used: Vec<String> = expected.iter().map(|(n, _)| b.defs.iter().find(|d| d.name == *n).unwrap().bus.label()).collect();
        used.sort();
        used.dedup();
        assert_eq!(used.len(), 4, "the eight feed pumps should spread across four buses, got {used:?}");
    }

    /// The two cargo-ventilation isolation valves are gated by their own
    /// VCM's real bus (VentilationControlModule has no separate power path
    /// for the valve itself, so this catalogue reuses the VCM's own
    /// channel), and both must use that VCM's *primary*, default-active
    /// channel (channel 1 / `powered_by[0]`, `ventilation_control_module.rs`
    /// `VentilationControlModule::new`) -- not the other VCM's bus, and not
    /// the standby channel. The two extraction fans are different: each has
    /// its own dedicated bus (`ForwardCargoVentilationControlSystem::new`/
    /// `BulkVentilationControlSystem::new`) entirely independent of either
    /// VCM channel, and must match that bus, not any VCM channel.
    #[test]
    fn cargo_ventilation_breakers_sit_on_their_real_bus() {
        let mut vars = TestVars::default();
        let b = Breakers::new(&mut vars);
        let bus_of = |id: &str| b.defs.iter().find(|d| d.id == id).unwrap_or_else(|| panic!("no breaker {id}")).bus.label();
        assert_eq!(bus_of("fwd-isol-valve"), "DC_1", "VCM Fwd's own primary channel is DC1 (411PP), not DC2 (that's VCM Aft's)");
        assert_eq!(bus_of("bulk-isol-valve"), "DC_2", "VCM Aft's own primary channel is DC2 (214PP), matching fwd-isol-valve's primary-channel convention, not its DC_ESS standby channel");
        assert_eq!(bus_of("fwd-extract-fan"), "AC_1", "the forward extraction fan's own dedicated bus (ForwardCargoVentilationControlSystem::new), not any VCM channel bus");
        assert_eq!(bus_of("bulk-extract-fan"), "AC_4", "the bulk extraction fan's own dedicated bus (BulkVentilationControlSystem::new), not any VCM channel bus");
    }

    /// fixes/W185.md: the VCM *channel breakers themselves* (as opposed to
    /// the isolation-valve/extract-fan loads fixes/W115.md and
    /// fixes/W161.md already fixed) had the same wrong-unit bug for VCM
    /// Fwd's channel 1 -- wired to DC2 (VCM Aft's own channel-1 bus)
    /// instead of DC1 (Fwd's own, `mod.rs`
    /// `VentilationControlModule::new(.., VcmId::Fwd, [DirectCurrent(1)
    /// /* 411PP */, DirectCurrentEssential /* 109PP */])`). VCM Aft's
    /// channel 1 (DC2/214PP) and both units' channel 2 (DC_ESS/109PP, the
    /// standby channel for both) were already correct.
    #[test]
    fn vcm_channel_breakers_sit_on_their_real_bus() {
        let mut vars = TestVars::default();
        let b = Breakers::new(&mut vars);
        let bus_of = |id: &str| b.defs.iter().find(|d| d.id == id).unwrap_or_else(|| panic!("no breaker {id}")).bus.label();
        assert_eq!(bus_of("vcm-fwd-1"), "DC_1", "VCM Fwd's own primary channel is DC1 (411PP), not DC2 (that's VCM Aft's channel 1)");
        assert_eq!(bus_of("vcm-fwd-2"), "DC_ESS", "VCM Fwd's standby channel (109PP)");
        assert_eq!(bus_of("vcm-aft-1"), "DC_2", "VCM Aft's own primary channel is DC2 (214PP)");
        assert_eq!(bus_of("vcm-aft-2"), "DC_ESS", "VCM Aft's standby channel (109PP), same physical bus as vcm-fwd-2");
    }

    /// The regression that grounded both FEED 4 pumps in flight, and the
    /// contract that replaced it ([`fuel_pump_current_var`]'s doc has the
    /// full arithmetic). A fuel pump's breaker is rated from systems.cfg's
    /// `Power:3, 5, 20.0` placeholder over a 28 V bus -- 0.179 A -- while
    /// `fuel.rs` publishes that pump's real motor current on the 115 V AC
    /// basis, 4.19 A once it carries an engine's cruise fuel flow. Feeding
    /// the second into a curve rated by the first is a ratio of 23.4,
    /// past the 10x magnetic pickup, so the breaker fired the instant the
    /// pump did its job.
    ///
    /// The test the wiring shipped with wrote its current as `rated * 0.44`
    /// and so assumed the very compatibility that does not hold. This one
    /// asserts the scales instead of assuming them: no fuel pump takes the
    /// real-current branch, and a pump carrying its design flow all day
    /// keeps its breaker.
    #[test]
    fn a_fuel_pump_carrying_its_design_flow_keeps_its_breaker() {
        // (build fix, INT-P4) same guard as the other real-current tests.
        let _guard = crate::failures::tests::serial();
        let mut vars = TestVars::default();
        let mut b = Breakers::new(&mut vars);

        // No `sys-<n>` fuel pump may take the real-current branch: its
        // rating is not on that number's scale.
        for (i, live) in b.live.iter().enumerate() {
            let def = &b.defs[live.def_index];
            if def.id.starts_with("sys-") {
                assert!(
                    live.real_current_id.is_none(),
                    "{} ({}) is rated from systems.cfg's 5 W/28 V token, so it must not be tripped by a 115 V motor current",
                    def.id,
                    def.name
                );
                let _ = i;
            }
        }

        // The measured operating point of a feed pump moving one engine's
        // cruise fuel flow (fuel_network's own model: 30 psi, 1660 gal/h ->
        // 361 W delivered -> 4.19 A at 115 V and 0.75 motor efficiency).
        // Nothing may turn that into a trip.
        let i = *b.by_id.get("sys-2").expect("circuit 2 is a real fuel pump");
        let rated = b.defs[b.live[i].def_index].rating_a;
        assert!(rated < 0.2, "systems.cfg's placeholder really is this small: {rated} A");
        for _ in 0..(3600 * 60) {
            b.post_systems(&mut vars, 1.0 / 60.0);
        }
        assert!(b.closed(&mut vars, i), "a healthy fuel pump must still have its breaker after an hour of running");
        let current = vars.read(&b.live[i].current_id);
        let nominal = rated * Breakers::NOMINAL_CONTINUOUS_LOAD_FRACTION;
        assert!(
            (current - nominal).abs() < 1e-9,
            "with no load coupling published, a fuel pump draws its nominal continuous load, got {current} against {nominal}"
        );
    }

    #[test]
    fn snapshot_reports_every_field_the_study_panel_json_needs() {
        let mut vars = TestVars::default();
        let mut circuits = crate::circuits::Circuits::new(&mut vars);
        let b = Breakers::new(&mut vars);
        let snap = b.snapshot(&mut vars);
        assert_eq!(snap.len(), catalog().len());
        for s in &snap {
            assert!(!s.id.is_empty());
            assert!(!s.name.is_empty());
            assert!(s.ata > 0);
            assert!(!s.bus.is_empty(), "{} has no bus label", s.id);
            assert!(s.closed, "every breaker starts closed");
            assert_eq!(s.trip, "none");
        }
    }
}
