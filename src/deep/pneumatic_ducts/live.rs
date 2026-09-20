//! The live pneumatic duct system: one owned [`DuctNetwork`] stepped
//! every frame from [`Truth`], with every failure [`super::registry`]
//! registers driving the exact `DuctNetworkFaults` field that registry
//! entry names, and every variable its ECAM triggers read published back
//! out.
//!
//! Before this file the network was a type nothing instantiated: no duct
//! anywhere in the plugin held gas, `DEEP_PNEU_ODLS_*_TRIP` did not
//! exist, and the `AIR ENG n BLEED LEAK`/`AIR ENG n PRECOOLER OVHT`
//! alerts hanging off those names could never fire.
//!
//! ## How `Truth` drives it
//! - **Ambient**: `environment.ambient_pressure_pa` and `sat_c` are the
//!   pressure and temperature every leak discharges into, every relief
//!   valve references and every duct conducts to.
//! - **Engine bleed ports**: `engine_bleed_pressure_pa`/`engine_bleed_
//!   temp_k` are the IP8 tap's own upstream condition; `engine_hp_port_
//!   pressure_pa`/`_temp_k` are the HP6 tap's own, read *unconditionally*
//!   (module doc on that pair in `deep::live`), so the HP valve's own
//!   stuck-valve failure (15_036_015) and the precooler's real hot source
//!   are both reachable now -- previously this branch was fed a fixed
//!   [`HP_PORT_UNAVAILABLE_PA`] (zero), which correctly held the valve
//!   shut but meant nothing behind it could ever be exercised.
//! - **Precooler cooling air**: the precooler is an air-to-air exchanger
//!   against engine fan-duct air. `Truth` has no bypass mass flow, so it
//!   is derived from `engine_n1_frac` and the ambient density at
//!   [`TRENT_900_BYPASS_MDOT_SLS_KG_S`] (public Trent 900 sea-level-static
//!   figures) -- a real relation between a real `Truth` input and the
//!   quantity the model needs, not a stand-in constant. The engine's own
//!   `bypass_mdot_kg_s` in `Truth` would be strictly better (still not
//!   sourced, `docs/deep/truth-requests.md`).
//! - **APU**: `apu_running` plus `apu_bleed_pressure_pa`, gated by the real
//!   `controls.apu_bleed_pb_on` pushbutton. The APU's bleed *temperature*
//!   is not in `Truth`, so it is computed from the pressure ratio the load
//!   compressor is actually achieving against ambient
//!   ([`APU_LOAD_COMPRESSOR_POLYTROPIC_EFFICIENCY`]) -- thermodynamics on
//!   a real input rather than a chosen number.
//! - **Cockpit controls** (`truth.controls`, real this pass): pack
//!   pushbuttons (both feed valves of a pack open together, since only the
//!   pushbutton is a real crew control -- the flow-control valve itself is
//!   this area's own modelled component, `docs/deep/truth-requests.md`),
//!   wing anti-ice selection (one pushbutton, both sides), the cross-bleed
//!   selector (raw 0 SHUT/1 AUTO/2 OPEN), the engine bleed pushbuttons
//!   (this area's own `network::NetworkInputs::engine_bleed_pb_auto`, the
//!   real shutoff the "ENG n BLEED" pushbutton *is*) and starter
//!   engagement, replacing the interim `ControlAssumptions` struct this
//!   area used to run on entirely (packs always open, wing anti-ice always
//!   off, starters always off, cross-bleed opened only by the same
//!   heuristic this area still uses for the selector's own AUTO position,
//!   since FBW's real AUTO logic is flight-deck software out of this plant
//!   model's scope, `network.rs`'s own module doc).
//! - **Zone air temperatures.** ODLS watches the temperature of the bay
//!   each duct runs through, which is `deep::thermal_zones`' own output.
//!   `Truth::published` now carries it (this pass's own contract fix, see
//!   `docs/deep/truth-requests.md`'s "Contract gap" section), so this area
//!   reads `THERMAL_ZONE_<NAME>_TEMPERATURE_C` back through
//!   `truth.published.get_or(...)` with the recovery temperature as the
//!   fallback for a frame nothing has published yet -- closing the leak ->
//!   bay overheat -> isolation chain this area exists for, previously cut
//!   in the middle because a duct leak's own heat (published here as
//!   `DEEP_PNEU_ZONE_<zone>_HEAT_W`) could never come back round to the
//!   loop that should trip on it.

use super::duct::DuctSectionFaults;
use super::network::{ApuBleedInput, DuctNetwork, DuctNetworkFaults, EngineBleedInput, NetworkInputs, NetworkOutputs, ODLS_ZONE_COUNT, ZONE_COUNT, ZONE_NAMES};
use super::odls::OdlsFaults;
use super::precooler::PrecoolerFaults;
use crate::deep::api::{failure_id, Area as RegArea};
use crate::deep::live::{Faults, Truth};

/// Ratio of specific heats for air, standard.
const GAMMA_AIR: f64 = 1.4;
/// ICAO standard atmosphere sea-level density, kg/m^3 (ICAO Doc 7488).
const SEA_LEVEL_DENSITY_KG_M3: f64 = 1.225;
/// Specific gas constant for dry air, J/(kg K).
const R_AIR_J_KG_K: f64 = 287.057_005;

/// Trent 900 sea-level-static bypass mass flow, kg/s. Public engine data
/// for the Trent 970/972 family gives about 1204 kg/s total intake flow at
/// take-off against a bypass ratio of about 8.7, i.e. 1204 * 8.7/9.7 of it
/// through the fan duct. The precooler's cooling air is a small bleed off
/// exactly that stream.
const TRENT_900_BYPASS_MDOT_SLS_KG_S: f64 = 1204.0 * 8.7 / 9.7;

/// Cooling air the APU's own fan/load-compressor stream makes available to
/// its precooler, kg/s, while it runs. **GENERIC**: no public figure
/// exists. Derived by the requirement a precooler has to meet to cool at
/// all -- its fan-air valve bleeds off 2% of the available stream at full
/// opening (`precooler::FAN_AIR_BLEED_OFF_FRACTION_AT_FULL_OPEN`), so the
/// stream must be about two orders of magnitude above the APU's own bleed
/// flow (order 0.5 kg/s through its bleed valve) for the cold side to
/// match the hot side.
const APU_COOLING_AIR_KG_S: f64 = 30.0;

/// Polytropic efficiency of the APU load compressor, used to get its
/// discharge temperature from the pressure ratio it is actually achieving
/// (module doc). **GENERIC**: 0.8 is the ordinary range for a single-stage
/// centrifugal load compressor; no PW980-specific figure is public.
const APU_LOAD_COMPRESSOR_POLYTROPIC_EFFICIENCY: f64 = 0.8;

fn f(ata: u16, n: u16) -> u64 {
    failure_id(RegArea::PneumaticDucts, ata, n)
}

/// Every variable name this area publishes, built once (the `Area` trait
/// publishes by `&str` and these names never change).
struct VarNames {
    odls_trip: [String; ODLS_ZONE_COUNT],
    odls_fault: [String; ODLS_ZONE_COUNT],
    zone_heat_w: [String; ZONE_COUNT],
    zone_jet_flux: [String; ZONE_COUNT],
    engine_precooler_ovht: [String; 4],
    engine_precooler_outlet_c: [String; 4],
    engine_isolation_open: [String; 4],
    engine_duct_pressure: [String; 4],
    engine_duct_temp_c: [String; 4],
    transfer_pipe_pressure: [String; 4],
    hp_valve_open: [String; 4],
    pr_valve_open: [String; 4],
    start_duct_pressure: [String; 4],
    pack_supply_pressure: [String; 2],
    pack_supply_temp_c: [String; 2],
    wai_duct_pressure: [String; 2],
    wai_duct_temp_c: [String; 2],
    wai_valve_open: [String; 2],
    hyd_reservoir_pressure: [String; 2],
    cross_bleed_open: [String; 3],
}

fn per_engine(fmt: impl Fn(usize) -> String) -> [String; 4] {
    std::array::from_fn(|i| fmt(i + 1))
}

impl VarNames {
    fn new() -> Self {
        let side = ["L", "R"];
        let hyd = ["GREEN", "YELLOW"];
        let xbleed = ["L", "C", "R"];
        Self {
            odls_trip: std::array::from_fn(|z| format!("DEEP_PNEU_ODLS_{}_TRIP", ZONE_NAMES[z])),
            odls_fault: std::array::from_fn(|z| format!("DEEP_PNEU_ODLS_{}_FAULT", ZONE_NAMES[z])),
            zone_heat_w: std::array::from_fn(|z| format!("DEEP_PNEU_ZONE_{}_HEAT_W", ZONE_NAMES[z])),
            zone_jet_flux: std::array::from_fn(|z| format!("DEEP_PNEU_ZONE_{}_JET_FLUX_W_M2", ZONE_NAMES[z])),
            engine_precooler_ovht: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_PRECOOLER_OVHT")),
            engine_precooler_outlet_c: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_PRECOOLER_OUTLET_C")),
            engine_isolation_open: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_ISOLATION_OPEN")),
            engine_duct_pressure: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_DUCT_PRESSURE_PA")),
            engine_duct_temp_c: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_DUCT_TEMPERATURE_C")),
            transfer_pipe_pressure: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_TRANSFER_PRESSURE_PA")),
            hp_valve_open: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_HP_VALVE_OPEN")),
            pr_valve_open: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_PR_VALVE_OPEN")),
            start_duct_pressure: per_engine(|n| format!("DEEP_PNEU_ENG_{n}_START_DUCT_PRESSURE_PA")),
            pack_supply_pressure: std::array::from_fn(|i| format!("DEEP_PNEU_PACK_{}_SUPPLY_PRESSURE_PA", i + 1)),
            pack_supply_temp_c: std::array::from_fn(|i| format!("DEEP_PNEU_PACK_{}_SUPPLY_TEMPERATURE_C", i + 1)),
            wai_duct_pressure: std::array::from_fn(|i| format!("DEEP_PNEU_WAI_{}_DUCT_PRESSURE_PA", side[i])),
            wai_duct_temp_c: std::array::from_fn(|i| format!("DEEP_PNEU_WAI_{}_DUCT_TEMPERATURE_C", side[i])),
            wai_valve_open: std::array::from_fn(|i| format!("DEEP_PNEU_WAI_{}_VALVE_OPEN", side[i])),
            hyd_reservoir_pressure: std::array::from_fn(|i| format!("DEEP_PNEU_HYD_{}_RESERVOIR_PRESSURE_PA", hyd[i])),
            cross_bleed_open: std::array::from_fn(|i| format!("DEEP_PNEU_XBLEED_{}_OPEN", xbleed[i])),
        }
    }
}

pub struct PneumaticDuctsLive {
    network: DuctNetwork,
    faults: DuctNetworkFaults,
    out: NetworkOutputs,
    names: VarNames,
}

impl Default for PneumaticDuctsLive {
    fn default() -> Self {
        Self::new()
    }
}

fn on(b: bool) -> f64 {
    if b {
        1.0
    } else {
        0.0
    }
}

impl PneumaticDuctsLive {
    pub fn new() -> Self {
        Self { network: DuctNetwork::new(), faults: DuctNetworkFaults::default(), out: NetworkOutputs::default(), names: VarNames::new() }
    }

    /// The last tick's outputs, for anything that wants the model's state
    /// rather than its published variables.
    pub fn outputs(&self) -> &NetworkOutputs {
        &self.out
    }

    /// Fan-duct mass flow available to one engine's precooler, kg/s
    /// (module doc). Fan flow is `rho * A * V` through a fixed annulus, so
    /// at a given air density it scales with fan speed; the density ratio
    /// carries the altitude dependence.
    fn bypass_mdot_kg_s(truth: &Truth, engine: usize) -> f64 {
        if !truth.engine_running[engine] {
            return 0.0;
        }
        let density = truth.environment.ambient_pressure_pa.max(1.0) / (R_AIR_J_KG_K * (truth.environment.sat_c + 273.15).max(1.0));
        TRENT_900_BYPASS_MDOT_SLS_KG_S * truth.engine_n1_frac[engine].clamp(0.0, 1.2) * (density / SEA_LEVEL_DENSITY_KG_M3)
    }

    /// Total (ram) air temperature, K: what the fan duct actually swallows
    /// and therefore the coldest the precooler's cooling side can be.
    fn ram_total_temp_k(truth: &Truth) -> f64 {
        let static_k = (truth.environment.sat_c + 273.15).max(1.0);
        let mach = truth.environment.mach();
        static_k * (1.0 + (GAMMA_AIR - 1.0) / 2.0 * mach * mach)
    }

    fn engine_inputs(truth: &Truth) -> [EngineBleedInput; 4] {
        let fan_air_k = Self::ram_total_temp_k(truth);
        std::array::from_fn(|i| EngineBleedInput {
            ip_port_pressure_pa: truth.engine_bleed_pressure_pa[i].max(0.0),
            ip_port_temp_k: truth.engine_bleed_temp_k[i].max(1.0),
            // Real HP6 port condition, read unconditionally (`Truth`'s own
            // doc on this pair): below FlyByWire's HP-valve interlock
            // whenever the engine genuinely has no usable HP source, and a
            // real hot pressure once it does -- no longer a fixed zero.
            hp_port_pressure_pa: truth.engine_hp_port_pressure_pa[i].max(0.0),
            hp_port_temp_k: truth.engine_hp_port_temp_k[i].max(1.0),
            fan_air_available_kg_s: Self::bypass_mdot_kg_s(truth, i),
            fan_air_k,
        })
    }

    /// The APU's load-compressor discharge condition. Its temperature is
    /// the compression it is actually doing, from the pressure ratio
    /// `Truth` publishes (module doc): `T2 = T1 * PR^((g-1)/(g*eta))`.
    fn apu_input(truth: &Truth) -> ApuBleedInput {
        let ambient_pa = truth.environment.ambient_pressure_pa.max(1.0);
        let ambient_k = (truth.environment.sat_c + 273.15).max(1.0);
        let pressure_pa = if truth.apu_running { truth.apu_bleed_pressure_pa.max(0.0) } else { 0.0 };
        let ratio = (pressure_pa / ambient_pa).max(1.0);
        let exponent = (GAMMA_AIR - 1.0) / (GAMMA_AIR * APU_LOAD_COMPRESSOR_POLYTROPIC_EFFICIENCY);
        ApuBleedInput {
            pressure_pa,
            temp_k: ambient_k * ratio.powf(exponent),
            fan_air_available_kg_s: if truth.apu_running { APU_COOLING_AIR_KG_S } else { 0.0 },
            fan_air_k: ambient_k,
        }
    }

    /// Whether the APU is actually able to deliver bleed: the real APU
    /// bleed pushbutton on (`truth.controls.apu_bleed_pb_on`), the APU
    /// running, and its port genuinely above the air it would have to push
    /// into.
    fn apu_bleed_available(truth: &Truth) -> bool {
        truth.controls.apu_bleed_pb_on && truth.apu_running && truth.apu_bleed_pressure_pa > truth.environment.ambient_pressure_pa * 1.05
    }

    /// The three cross-bleed valves' commanded position from the real
    /// selector knob (`truth.controls.cross_bleed_selector`, raw 0 SHUT /
    /// 1 AUTO / 2 OPEN -- `Controls`' own doc). SHUT and OPEN are the
    /// selector's own literal positions; AUTO keeps this area's prior
    /// heuristic (open only when the APU is genuinely the sole bleed
    /// source available) because FlyByWire's own AUTO logic is flight-deck
    /// computer software, out of this self-contained plant model's scope
    /// (`network.rs`'s own module doc precedent for the upstream stage).
    fn cross_bleed_command(truth: &Truth) -> f64 {
        if truth.controls.cross_bleed_selector <= 0.5 {
            0.0 // SHUT
        } else if truth.controls.cross_bleed_selector >= 1.5 {
            1.0 // OPEN
        } else {
            on(Self::apu_bleed_available(truth)) // AUTO
        }
    }

    fn inputs(&self, truth: &Truth) -> NetworkInputs {
        let apu_available = Self::apu_bleed_available(truth);
        let cross = Self::cross_bleed_command(truth);
        let recovery_k = Self::recovery_temp_k(truth);
        NetworkInputs {
            dt_s: truth.dt_s,
            ambient_pa: truth.environment.ambient_pressure_pa.max(1.0),
            ambient_k: (truth.environment.sat_c + 273.15).max(1.0),
            engines: Self::engine_inputs(truth),
            apu: Self::apu_input(truth),
            apu_bleed_selected: apu_available,
            apu_bleed_valve_command: on(apu_available),
            cross_bleed_valve_command: [cross; 3],
            // Only the pushbutton is a real crew control (`docs/deep/
            // truth-requests.md`): both feed valves of a pack open
            // together once its own pushbutton is on.
            pack_valve_open: [[on(truth.controls.pack_pb_on[0]); 2], [on(truth.controls.pack_pb_on[1]); 2]],
            // One pushbutton, both sides (`Controls`' own doc).
            wai_selected: [truth.controls.wing_anti_ice_selected; 2],
            starter_engaged: truth.controls.starter_engaged,
            engine_bleed_pb_auto: truth.controls.engine_bleed_pb_auto,
            zone_air_k: Self::zone_air_k(truth, recovery_k),
        }
    }

    /// Adiabatic-wall recovery temperature, K: what a bay's structure and
    /// the air washing through it actually sit at with no internal heat
    /// source, turbulent recovery factor 0.9.
    fn recovery_temp_k(truth: &Truth) -> f64 {
        const RECOVERY_FACTOR: f64 = 0.9;
        let static_k = (truth.environment.sat_c + 273.15).max(1.0);
        let mach = truth.environment.mach();
        static_k * (1.0 + RECOVERY_FACTOR * (GAMMA_AIR - 1.0) / 2.0 * mach * mach)
    }

    /// Each zone's air temperature this tick, K, indexed by `ZONE_NAMES`:
    /// `deep::thermal_zones`' own published `THERMAL_ZONE_<NAME>_
    /// TEMPERATURE_C` (previous frame, `Truth::published`'s documented
    /// one-frame lag) when it has published one, the same recovery
    /// temperature as before otherwise -- an unheated, ram-ventilated
    /// bay's own physically sane resting state, and what every zone reads
    /// on the first frame or if `thermal_zones` were ever absent from
    /// `all_areas()`. This is the coupling that lets a real duct leak's own
    /// heat (via `thermal_zones`' identically-named ATA 36/49 leak
    /// failures heating the same zones, `network.rs`'s own module doc)
    /// come back around and trip this area's own ODLS, instead of every
    /// zone being permanently pinned at recovery temperature regardless of
    /// what is actually leaking into it.
    fn zone_air_k(truth: &Truth, recovery_k: f64) -> [f64; ZONE_COUNT] {
        let recovery_c = recovery_k - 273.15;
        std::array::from_fn(|z| {
            let name = format!("THERMAL_ZONE_{}_TEMPERATURE_C", ZONE_NAMES[z].to_ascii_uppercase());
            truth.published.get_or(&name, recovery_c) + 273.15
        })
    }

    /// Every failure `registry.rs` registers, onto the exact model field
    /// it names.
    ///
    /// The catalogue registers **one id per distinct fault mechanism on a
    /// component class** (`registry.rs`'s own module doc: "Engine bleed
    /// duct, pylon run (x4 engines)"), not one per instance, so arming a
    /// duct-leak failure leaks every instance of that duct. Per-instance
    /// arming needs per-instance ids; noted in the report.
    fn apply_faults(&mut self, faults: &Faults) {
        let duct = |leak_id: u64, rupture_id: u64, insulation_id: u64| DuctSectionFaults {
            leak: faults.get(leak_id),
            rupture: faults.get(rupture_id),
            insulation_damage: faults.get(insulation_id),
        };
        let precooler = |fouling: u64, fav: u64, sensor: u64, check: u64| PrecoolerFaults {
            fouling: faults.get(fouling),
            fan_air_valve_stuck: faults.get(fav),
            temp_sensor_fault: faults.get(sensor),
            check_valve_failure: faults.get(check),
        };

        let engine_duct = duct(f(36, 1), f(36, 2), f(36, 3));
        let engine_precooler = precooler(f(36, 4), f(36, 5), f(36, 6), f(36, 7));
        let upstream_hp = faults.get(f(36, 15));
        let upstream_pr = faults.get(f(36, 16));
        let upstream_ip = faults.get(f(36, 17));
        let start_duct = duct(f(36, 21), f(36, 22), f(36, 23));
        let start_check_valve = faults.get(f(36, 24));
        for i in 0..4 {
            self.faults.engine_duct[i] = engine_duct;
            self.faults.engine_precooler[i] = engine_precooler;
            self.faults.upstream[i].hp_valve_stuck = upstream_hp;
            self.faults.upstream[i].pr_valve_stuck = upstream_pr;
            self.faults.upstream[i].ip_check_valve_stuck_closed = upstream_ip;
            self.faults.start[i] = start_duct;
            self.faults.start_check_valve_failure[i] = start_check_valve;
        }

        self.faults.apu_duct = duct(f(36, 8), f(36, 9), f(36, 10));
        self.faults.apu_precooler = precooler(f(36, 11), f(36, 12), f(36, 13), f(36, 14));

        let pack_duct = duct(f(36, 18), f(36, 19), f(36, 20));
        let hyd_duct = duct(f(36, 25), f(36, 26), f(36, 27));
        let wai_duct = duct(f(30, 1), f(30, 2), f(30, 3));
        for i in 0..2 {
            self.faults.packs[i] = pack_duct;
            self.faults.hyd_reservoir[i] = hyd_duct;
            self.faults.wai[i] = wai_duct;
        }

        let odls = OdlsFaults {
            loop_a_open: faults.get(f(36, 28)),
            loop_a_short: faults.get(f(36, 29)),
            loop_b_open: faults.get(f(36, 30)),
            loop_b_short: faults.get(f(36, 31)),
            false_detection: faults.get(f(36, 32)),
        };
        for z in 0..ODLS_ZONE_COUNT {
            self.faults.odls[z] = odls;
        }
    }
}

impl crate::deep::live::Area for PneumaticDuctsLive {
    fn name(&self) -> &'static str {
        "pneumatic_ducts"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        self.apply_faults(faults);
        let inputs = self.inputs(truth);
        self.out = self.network.step(&inputs, &self.faults);
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        let n = &self.names;
        let o = &self.out;
        for z in 0..ODLS_ZONE_COUNT {
            out(&n.odls_trip[z], on(o.odls_trip[z]));
            out(&n.odls_fault[z], on(o.odls_loop_fault[z]));
        }
        for z in 0..ZONE_COUNT {
            out(&n.zone_heat_w[z], o.zone_heat_w[z]);
            out(&n.zone_jet_flux[z], o.jet_impact_flux_w_m2[z]);
        }
        for i in 0..4 {
            out(&n.engine_precooler_ovht[i], on(o.engine_precooler_overtemp[i]));
            out(&n.engine_precooler_outlet_c[i], o.engine_precooler_outlet_k[i] - 273.15);
            out(&n.engine_isolation_open[i], on(!o.engine_isolated[i]));
            out(&n.engine_duct_pressure[i], o.engine_duct_pressure_pa[i]);
            out(&n.engine_duct_temp_c[i], o.engine_duct_temp_k[i] - 273.15);
            out(&n.transfer_pipe_pressure[i], o.transfer_pipe_pressure_pa[i]);
            out(&n.hp_valve_open[i], o.hp_valve_open[i]);
            out(&n.pr_valve_open[i], o.pr_valve_open[i]);
            out(&n.start_duct_pressure[i], o.start_duct_pressure_pa[i]);
        }
        out("DEEP_PNEU_APU_PRECOOLER_OVHT", on(o.apu_precooler_overtemp));
        out("DEEP_PNEU_APU_PRECOOLER_OUTLET_C", o.apu_precooler_outlet_k - 273.15);
        out("DEEP_PNEU_APU_ISOLATION_OPEN", on(!o.apu_isolated));
        out("DEEP_PNEU_APU_DUCT_TEMPERATURE_C", o.apu_duct_temp_k - 273.15);
        out("DEEP_PNEU_APU_BLEED_VALVE_OPEN", o.apu_bleed_valve_open);
        for i in 0..2 {
            out(&n.pack_supply_pressure[i], o.pack_supply_pressure_pa[i]);
            out(&n.pack_supply_temp_c[i], o.pack_supply_temp_k[i] - 273.15);
            out(&n.wai_duct_pressure[i], o.wai_duct_pressure_pa[i]);
            out(&n.wai_duct_temp_c[i], o.wai_duct_temp_k[i] - 273.15);
            out(&n.wai_valve_open[i], o.wai_valve_open[i]);
            out(&n.hyd_reservoir_pressure[i], o.hyd_reservoir_pressure_pa[i]);
        }
        for i in 0..3 {
            out(&n.cross_bleed_open[i], o.cross_bleed_valve_open[i]);
        }
    }
}

pub fn live_system() -> Box<dyn crate::deep::live::Area> {
    Box::new(PneumaticDuctsLive::new())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    /// Four engines at cruise power with a real IP8 bleed condition at the
    /// pylon, at a cruise ambient.
    fn cruise_truth() -> Truth {
        Truth {
            dt_s: 1.0,
            environment: crate::deep::integration::weather_truth::EnvironmentTruth {
                sat_c: -40.0,
                leading_edge_c: -20.0,
                ambient_pressure_pa: 24_000.0,
                tas_ms: 240.0,
                precipitation_on_aircraft_ratio: 0.0,
                weather: None,
            },
            altitude_ft: 35_000.0,
            on_ground: false,
            engine_n1_frac: [0.85; 4],
            engine_running: [true; 4],
            engine_bleed_pressure_pa: [260_000.0; 4],
            engine_bleed_temp_k: [400.0; 4],
            ac_bus_volts: [115.0; 4],
            ..Truth::default()
        }
    }

    fn published(area: &dyn crate::deep::live::Area) -> BTreeMap<String, f64> {
        let mut map = BTreeMap::new();
        area.publish(&mut |name, value| {
            map.insert(name.to_string(), value);
        });
        map
    }

    fn run(area: &mut dyn crate::deep::live::Area, truth: &Truth, faults: &Faults, ticks: usize) {
        for _ in 0..ticks {
            area.tick(truth, faults);
        }
    }

    #[test]
    fn every_variable_the_registry_triggers_on_is_actually_published() {
        let area = live_system();
        let map = published(area.as_ref());
        let mut required: Vec<String> = Vec::new();
        for zone in ZONE_NAMES.iter().take(ODLS_ZONE_COUNT) {
            required.push(format!("DEEP_PNEU_ODLS_{zone}_TRIP"));
            required.push(format!("DEEP_PNEU_ODLS_{zone}_FAULT"));
        }
        for n in 1..=4 {
            required.push(format!("DEEP_PNEU_ENG_{n}_PRECOOLER_OVHT"));
        }
        // Named in `registry.rs`'s own "new Vars this model must publish"
        // list as well as its triggers.
        required.push("DEEP_PNEU_APU_PRECOOLER_OVHT".into());
        required.push("DEEP_PNEU_APU_ISOLATION_OPEN".into());
        for n in 1..=4 {
            required.push(format!("DEEP_PNEU_ENG_{n}_ISOLATION_OPEN"));
        }
        for name in required {
            assert!(map.contains_key(&name), "{name} is read by an ECAM trigger but never published");
        }
    }

    #[test]
    fn a_healthy_network_pressurises_its_ducts_and_trips_nothing() {
        let mut area = live_system();
        run(area.as_mut(), &cruise_truth(), &Faults::default(), 300);
        let map = published(area.as_ref());
        for n in 1..=4 {
            assert!(map[&format!("DEEP_PNEU_ENG_{n}_DUCT_PRESSURE_PA")] > 101_325.0, "engine {n} duct must pressurise, got {}", map[&format!("DEEP_PNEU_ENG_{n}_DUCT_PRESSURE_PA")]);
            assert_eq!(map[&format!("DEEP_PNEU_ENG_{n}_ISOLATION_OPEN")], 1.0);
        }
        assert!(map["DEEP_PNEU_PACK_1_SUPPLY_PRESSURE_PA"] > 101_325.0);
        for zone in ZONE_NAMES.iter().take(ODLS_ZONE_COUNT) {
            assert_eq!(map[&format!("DEEP_PNEU_ODLS_{zone}_TRIP")], 0.0, "{zone} must not trip with nothing wrong");
        }
    }

    #[test]
    fn arming_the_engine_bleed_duct_rupture_sags_the_duct_and_heats_the_pylon() {
        // Failure 15_036_002, effect: "Large mass flow escapes as a
        // near-sonic jet ... higher heat-transfer effectiveness into the
        // pylon zone" and (fault 1's shared mechanism) "manifold/
        // downstream pressure sags proportionally".
        let truth = cruise_truth();
        let mut ruptured = live_system();
        let mut healthy = live_system();
        run(ruptured.as_mut(), &truth, &Faults::from_pairs([(f(36, 2), 1.0)]), 100);
        run(healthy.as_mut(), &truth, &Faults::default(), 100);
        let bad = published(ruptured.as_ref());
        let good = published(healthy.as_ref());

        assert!(
            bad["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"] < good["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"],
            "a ruptured duct must sag: {} vs {}",
            bad["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"],
            good["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"]
        );
        assert!(bad["DEEP_PNEU_ZONE_PylonEngine1_HEAT_W"] > 1000.0, "the escaping gas must dump real heat into its own pylon, got {}", bad["DEEP_PNEU_ZONE_PylonEngine1_HEAT_W"]);
        assert!(bad["DEEP_PNEU_ZONE_PylonEngine1_JET_FLUX_W_M2"] > 0.0, "a full rupture must report an impinging-jet flux");
        assert_eq!(good["DEEP_PNEU_ZONE_PylonEngine1_JET_FLUX_W_M2"], 0.0);
    }

    #[test]
    fn arming_the_odls_false_detection_trips_and_latches_that_zones_isolation() {
        // Failure 15_036_032, effect: "Trips and latches that zone's
        // isolation valve shut with no real overheat present". The
        // published trip is what `AIR ENG n BLEED LEAK` triggers on.
        let truth = cruise_truth();
        let mut area = live_system();
        let armed = Faults::from_pairs([(f(36, 32), 1.0)]);
        run(area.as_mut(), &truth, &armed, 30);
        let map = published(area.as_ref());
        assert_eq!(map["DEEP_PNEU_ODLS_PylonEngine1_TRIP"], 1.0, "a full-severity false detection must trip a cold zone");
        assert_eq!(map["DEEP_PNEU_ENG_1_ISOLATION_OPEN"], 0.0, "the trip must isolate that engine's bleed");
        assert!(map["DEEP_PNEU_ENG_1_PR_VALVE_OPEN"] < 0.01, "and drive its PR valve shut, got {}", map["DEEP_PNEU_ENG_1_PR_VALVE_OPEN"]);

        // Latching: clearing the fault does not un-trip the isolation.
        run(area.as_mut(), &truth, &Faults::default(), 30);
        let after = published(area.as_ref());
        assert_eq!(after["DEEP_PNEU_ENG_1_ISOLATION_OPEN"], 0.0, "a real ODLS trip needs a reset, not self-clearing");
    }

    #[test]
    fn arming_a_loop_open_circuit_reports_a_detection_fault_without_tripping() {
        // Failures 15_036_028/030 raise AIR BLEED LEAK DET FAULT: both
        // loops open leaves no valid detection at all.
        let truth = cruise_truth();
        let mut area = live_system();
        let armed = Faults::from_pairs([(f(36, 28), 1.0), (f(36, 30), 1.0)]);
        run(area.as_mut(), &truth, &armed, 30);
        let map = published(area.as_ref());
        assert_eq!(map["DEEP_PNEU_ODLS_PylonEngine1_FAULT"], 1.0);
        assert_eq!(map["DEEP_PNEU_ODLS_PylonEngine1_TRIP"], 0.0, "a loop fault is not a leak");
    }

    #[test]
    fn arming_precooler_fouling_leaves_the_delivered_bleed_hotter() {
        // Failure 15_036_004, effect: "For the same cooling flow the
        // outlet runs hotter ... raising overtemperature-trip risk".
        // Read on the duct the precooler actually delivers into rather
        // than on `..._PRECOOLER_OUTLET_C`: with no consumer downstream
        // (see `ControlAssumptions`) the PR valve meters in bursts, and on
        // a tick with no flow through it a heat exchanger has nothing to
        // exchange, so its instantaneous outlet reads back as its own
        // source temperature. The duct's gas temperature is the integral
        // of what was actually delivered and is what the bay, the ODLS and
        // every consumer downstream really see.
        //
        // Needs a bleed hot enough for the precooler to have work to do at
        // all (its regulation target is 200 C), i.e. a high-power tap.
        let mut truth = cruise_truth();
        truth.engine_bleed_temp_k = [560.0; 4];
        truth.engine_bleed_pressure_pa = [300_000.0; 4];

        let mut fouled = live_system();
        let mut clean = live_system();
        run(fouled.as_mut(), &truth, &Faults::from_pairs([(f(36, 4), 1.0)]), 60);
        run(clean.as_mut(), &truth, &Faults::default(), 60);
        let hot = published(fouled.as_ref())["DEEP_PNEU_ENG_1_DUCT_TEMPERATURE_C"];
        let cool = published(clean.as_ref())["DEEP_PNEU_ENG_1_DUCT_TEMPERATURE_C"];
        assert!(hot > cool + 3.0, "a fouled core must leave the duct hotter: {hot} C vs {cool} C");
    }

    #[test]
    fn the_apu_feeds_engine_one_through_the_cross_bleed_when_the_engines_are_dead() {
        let mut truth = cruise_truth();
        truth.engine_running = [false; 4];
        truth.engine_n1_frac = [0.0; 4];
        truth.engine_bleed_pressure_pa = [101_325.0; 4];
        truth.engine_bleed_temp_k = [288.15; 4];
        truth.on_ground = true;
        truth.environment.sat_c = 15.0;
        truth.environment.ambient_pressure_pa = 101_325.0;
        truth.environment.tas_ms = 0.0;
        truth.apu_running = true;
        truth.apu_bleed_pressure_pa = 320_000.0;
        truth.controls.apu_bleed_pb_on = true;

        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::default(), 300);
        let map = published(area.as_ref());
        assert_eq!(map["DEEP_PNEU_APU_BLEED_VALVE_OPEN"], 1.0);
        assert!(map["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"] > 120_000.0, "the APU must pressurise engine 1's duct, got {}", map["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"]);
        assert!(map["DEEP_PNEU_APU_DUCT_TEMPERATURE_C"] > 15.0, "load-compressor discharge must be hotter than the air it drew in");
    }

    #[test]
    fn the_hp6_branch_now_uses_the_real_hp_port_instead_of_a_fixed_zero() {
        // Before this pass `Truth::engine_hp_port_pressure_pa`/`_temp_k`
        // were ignored entirely (this file's own old `HP_PORT_UNAVAILABLE_
        // PA` = 0), so the HP valve could never open regardless of what
        // the engine's own HP6 port was doing. With the real port wired
        // in, an engine isolated from every other source and starved of
        // IP8 must still pressurise through its own HP valve, exactly like
        // `network.rs`'s own `the_hp_valve_opens_when_ip8_alone_cannot_
        // hold_regulation` test proves the underlying model already can.
        let mut truth = cruise_truth();
        truth.controls.cross_bleed_selector = 0.0; // SHUT: no neighbour can help
        truth.controls.pack_pb_on = [false, false]; // no consumer to mask the source
        truth.engine_bleed_pressure_pa[0] = 150_000.0; // below the 206.8 kPa IP8/HP6 switch-over
        truth.engine_bleed_temp_k[0] = 400.0;
        truth.engine_hp_port_pressure_pa[0] = 500_000.0;
        truth.engine_hp_port_temp_k[0] = 600.0;

        let mut area = live_system();
        let mut peak_hp_open = 0.0_f64;
        for _ in 0..300 {
            area.tick(&truth, &Faults::default());
            peak_hp_open = peak_hp_open.max(published(area.as_ref())["DEEP_PNEU_ENG_1_HP_VALVE_OPEN"]);
        }
        assert!(peak_hp_open > 0.1, "the HP valve must open off the real HP6 port once IP8 alone cannot hold regulation, peak {peak_hp_open}");
    }

    #[test]
    fn cross_bleed_selector_shut_overrides_the_apu_sole_source_heuristic() {
        let mut truth = cruise_truth();
        truth.engine_running = [false; 4];
        truth.engine_n1_frac = [0.0; 4];
        truth.engine_bleed_pressure_pa = [101_325.0; 4];
        truth.engine_bleed_temp_k = [288.15; 4];
        truth.on_ground = true;
        truth.environment.sat_c = 15.0;
        truth.environment.ambient_pressure_pa = 101_325.0;
        truth.environment.tas_ms = 0.0;
        truth.apu_running = true;
        truth.apu_bleed_pressure_pa = 320_000.0;
        truth.controls.apu_bleed_pb_on = true;
        truth.controls.cross_bleed_selector = 0.0; // SHUT
        // Pack 1's own dual feed (engines 1 *and* 2) is itself a second
        // bridge between their ducts, entirely independent of the
        // cross-bleed valves (`network.rs`'s own `closing_all_cross_bleed_
        // valves_stops_a_non_running_engine_from_pressurising` test notes
        // exactly this) -- shut here so this test isolates what the
        // cross-bleed selector itself controls.
        truth.controls.pack_pb_on = [false, false];

        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::default(), 300);
        let map = published(area.as_ref());
        assert_eq!(map["DEEP_PNEU_XBLEED_L_OPEN"], 0.0, "SHUT must override even the sole-source AUTO heuristic");
        // The APU's own bleed valve feeds engine 1's duct directly and is
        // not one of the three valves this selector controls (`Controls`'
        // own doc: "a single knob controls all three cross-bleed valves",
        // i.e. L/C/R, not the separate APU valve), so engine 1 still
        // pressurises; SHUT is proven by engine 2 -- reachable only
        // through the now-shut left cross-bleed valve -- staying unfed.
        assert!(map["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"] > 120_000.0, "the APU's own valve into engine 1 is unrelated to the cross-bleed selector, got {}", map["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"]);
        assert!(map["DEEP_PNEU_ENG_2_DUCT_PRESSURE_PA"] < 110_000.0, "with the cross-bleed selector SHUT, engine 2 must not be fed through the left valve, got {}", map["DEEP_PNEU_ENG_2_DUCT_PRESSURE_PA"]);
    }

    #[test]
    fn cross_bleed_selector_open_forces_every_valve_open_with_no_sole_source_condition() {
        let mut truth = cruise_truth();
        truth.controls.cross_bleed_selector = 2.0; // OPEN
        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::default(), 30);
        let map = published(area.as_ref());
        assert_eq!(map["DEEP_PNEU_XBLEED_L_OPEN"], 1.0);
        assert_eq!(map["DEEP_PNEU_XBLEED_C_OPEN"], 1.0);
        assert_eq!(map["DEEP_PNEU_XBLEED_R_OPEN"], 1.0);
    }

    #[test]
    fn switching_a_pack_pushbutton_off_stops_feeding_that_pack() {
        let truth = cruise_truth(); // default pack_pb_on = [true, true]
        let mut off_truth = cruise_truth();
        off_truth.controls.pack_pb_on[0] = false;

        let mut on_area = live_system();
        let mut off_area = live_system();
        run(on_area.as_mut(), &truth, &Faults::default(), 300);
        run(off_area.as_mut(), &off_truth, &Faults::default(), 300);
        let on_pressure = published(on_area.as_ref())["DEEP_PNEU_PACK_1_SUPPLY_PRESSURE_PA"];
        let off_pressure = published(off_area.as_ref())["DEEP_PNEU_PACK_1_SUPPLY_PRESSURE_PA"];
        assert!(on_pressure > off_pressure + 5000.0, "switching pack 1's pushbutton off must stop feeding it: on {on_pressure} vs off {off_pressure}");
    }

    #[test]
    fn a_thermal_areas_own_wing_duct_leak_heats_the_bay_enough_to_trip_this_areas_odls() {
        // End-to-end coupling test: `thermal_zones` carries its own
        // (pre-existing, interim) ATA 30 wing-anti-ice-duct-leak failure
        // that heats WingLeLeft's real air node directly
        // (`thermal_zones::live::apply_ice_and_duct_failures`). Before this
        // pass this area could never see that heat -- every zone was given
        // recovery temperature regardless (module doc's old "Contract
        // gap") -- so the leak -> bay overheat -> isolation chain this
        // area exists for was cut in the middle. With `Self::zone_air_k`
        // now reading `truth.published` instead, the other area's real
        // heat must reach this area's own ODLS and trip it.
        //
        // WingLeLeft (unlike a pylon) has no forced-ventilation link at
        // all in `topology_a380::build` (anti-ice heat is a transient
        // system input into an otherwise unventilated compartment, that
        // module's own doc), so its air node's only loss path is the
        // 40 W/K air<->structure coupling -- a full-severity 30 kW leak
        // drives it far past the 100 K ODLS margin, unlike a ram-vented
        // pylon's own 40 kW leak, which this test found settles only
        // ~75 K above ambient under this area's own 0.5 kg/s pylon vent
        // and so never confirms a trip (a real, if modest, gap between
        // `thermal_zones`' interim leak magnitude and this area's fixed
        // detection margin -- noted in the report, not papered over here).
        let mut deep = crate::deep::live::Deep::new().with_area(live_system()).with_area(crate::deep::thermal_zones::live::live_system());
        let leak_id = f_thermal(30, 1); // thermal_zones' own WingLeLeft anti-ice duct leak id
        let armed = Faults::from_pairs([(leak_id, 1.0)]);
        let truth = Truth { dt_s: 1.0, ..Truth::default() };
        let mut published = BTreeMap::new();
        for _ in 0..600 {
            deep.tick(truth.clone(), &armed, &mut |name, value| {
                published.insert(name.to_string(), value);
            });
        }
        assert!(published["THERMAL_ZONE_WINGLELEFT_TEMPERATURE_C"] > 150.0, "setup: the thermal area's own leak failure must actually heat the bay, got {}", published["THERMAL_ZONE_WINGLELEFT_TEMPERATURE_C"]);
        assert_eq!(published["DEEP_PNEU_ODLS_WingLeLeft_TRIP"], 1.0, "the real bay heat must now reach this area's own ODLS and trip it");

        // The right wing never had anything leak into it.
        assert_eq!(published["DEEP_PNEU_ODLS_WingLeRight_TRIP"], 0.0);
    }

    /// `thermal_zones` registers its ATA 30/36 duct-leak failures under
    /// `Area::ThermalZones`, a different (interim, pre-existing) failure
    /// id from this area's own equivalent faults -- both model a leak into
    /// the same physical bay, from two different areas' own components,
    /// per `network.rs`'s own module doc.
    fn f_thermal(ata: u16, n: u16) -> u64 {
        crate::deep::api::failure_id(RegArea::ThermalZones, ata, n)
    }

    #[test]
    fn a_cold_dark_aircraft_publishes_finite_values_and_a_zero_dt_frame_changes_nothing() {
        let mut area = live_system();
        let truth = Truth::default();
        run(area.as_mut(), &truth, &Faults::default(), 100);
        for (name, value) in published(area.as_ref()) {
            assert!(value.is_finite(), "{name} went non-finite");
        }
        let still = Truth { dt_s: 0.0, ..Truth::default() };
        area.tick(&still, &Faults::default());
        let before = published(area.as_ref());
        area.tick(&still, &Faults::default());
        assert_eq!(before, published(area.as_ref()));
    }
}

