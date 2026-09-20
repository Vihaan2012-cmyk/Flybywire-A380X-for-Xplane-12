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
//!   temp_k` are the IP8 tap's own upstream condition, per their own doc
//!   in `live.rs` ("bleed air available at the pylon ... IP8/HP6 port
//!   outputs"). `Truth` carries **one** port per engine, not the two the
//!   real stage has, so the HP6 branch is driven with
//!   [`HP_PORT_UNAVAILABLE_PA`] -- zero, i.e. below FlyByWire's own
//!   `psi(15.)` HP-valve interlock, so the HP valve correctly stays shut
//!   instead of being fed a fabricated pressure. See the report: `Truth`
//!   needs `engine_hp_port_pressure_pa`/`_temp_k` for that branch (and
//!   with it failure 15_036_015, HP valve stuck) to do anything.
//! - **Precooler cooling air**: the precooler is an air-to-air exchanger
//!   against engine fan-duct air. `Truth` has no bypass mass flow, so it
//!   is derived from `engine_n1_frac` and the ambient density at
//!   [`TRENT_900_BYPASS_MDOT_SLS_KG_S`] (public Trent 900 sea-level-static
//!   figures) -- a real relation between a real `Truth` input and the
//!   quantity the model needs, not a stand-in constant. The engine's own
//!   `bypass_mdot_kg_s` in `Truth` would be strictly better.
//! - **APU**: `apu_running` plus `apu_bleed_pressure_pa`. The APU's bleed
//!   *temperature* is not in `Truth`, so it is computed from the pressure
//!   ratio the load compressor is actually achieving against ambient
//!   ([`APU_LOAD_COMPRESSOR_POLYTROPIC_EFFICIENCY`]) -- thermodynamics on
//!   a real input rather than a chosen number.
//!
//! ## Inputs this area needs that `Truth` does not carry yet
//! Every one of these is a cockpit control or another area's output, not
//! a physical quantity this module may invent; each is listed in the
//! report with what it gates.
//! - Bleed/cross-bleed/pack/wing-anti-ice/starter selections. Interim
//!   positions are in [`ControlAssumptions`], which documents each one and
//!   is the single place a wiring pass replaces with real inputs.
//! - **Zone air temperatures.** ODLS watches the temperature of the bay
//!   each duct runs through, which is `deep::thermal_zones`' output, and
//!   an area cannot read another area's published variables from inside
//!   `tick`. Until those reach `Truth`, every zone is given the recovery
//!   temperature -- the temperature an unheated, ram-ventilated bay
//!   actually tends to, so the loops see a true baseline and false-trip
//!   faults still work, but a real duct leak's own heat (published here as
//!   `DEEP_PNEU_ZONE_<zone>_HEAT_W`) cannot yet come back round to trip
//!   the loop it would in the aircraft.

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

/// The HP6 port pressure handed to the upstream stage while `Truth`
/// carries only one bleed port per engine (module doc). Zero is below
/// FlyByWire's own HP-valve minimum-source interlock, so the valve stays
/// shut -- the model's own correct response to "no usable HP source",
/// rather than a made-up pressure.
const HP_PORT_UNAVAILABLE_PA: f64 = 0.0;

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

/// Cockpit/controller positions this area needs and `Truth` does not
/// carry yet (module doc). Every field is an *input* to the pneumatic
/// plant, not a physical property of it, so each is documented with the
/// position it is held at and why -- and all of them are replaced in one
/// place once the corresponding `Truth` fields exist.
#[derive(Clone, Copy, Debug)]
pub struct ControlAssumptions {
    /// Both flow-control valves of both packs, open: the packs are the
    /// duct system's normal continuous consumer, and with every consumer
    /// shut the network is a dead-end in which no leak, precooler or
    /// isolation fault can express itself at all.
    pub pack_valve_open: [[f64; 2]; 2],
    /// Wing anti-ice selected, per side. Held off: anti-ice is a crew
    /// selection made for an icing encounter, and assuming it on would
    /// flow hot air through the leading edge of an aircraft in clear air.
    /// The three ATA 30 wing-anti-ice duct failures cannot express
    /// themselves until this is a real input.
    pub wai_selected: [bool; 2],
    /// Starter air valves, per engine. Held shut for the same reason.
    pub starter_engaged: [bool; 4],
}

impl Default for ControlAssumptions {
    fn default() -> Self {
        Self { pack_valve_open: [[1.0, 1.0], [1.0, 1.0]], wai_selected: [false, false], starter_engaged: [false; 4] }
    }
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
    controls: ControlAssumptions,
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
        Self { network: DuctNetwork::new(), faults: DuctNetworkFaults::default(), out: NetworkOutputs::default(), names: VarNames::new(), controls: ControlAssumptions::default() }
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
            hp_port_pressure_pa: HP_PORT_UNAVAILABLE_PA,
            // Never used while the HP valve's orifice area is zero; kept
            // at the same port's temperature so no branch ever sees a
            // physically impossible 0 K gas.
            hp_port_temp_k: truth.engine_bleed_temp_k[i].max(1.0),
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

    /// Whether the APU is actually able to deliver bleed: running, and its
    /// port genuinely above the air it would have to push into. The
    /// pushbutton itself is not in `Truth` (module doc).
    fn apu_bleed_available(truth: &Truth) -> bool {
        truth.apu_running && truth.apu_bleed_pressure_pa > truth.environment.ambient_pressure_pa * 1.05
    }

    fn inputs(&self, truth: &Truth) -> NetworkInputs {
        let apu_available = Self::apu_bleed_available(truth);
        // Interim cross-bleed control: the real selector is not in `Truth`
        // (module doc). FlyByWire's own AUTO logic opens the cross-bleed
        // valves so one source can feed the other ducts, which is exactly
        // the case where the APU is the only source, and leaves them shut
        // otherwise so a leak on one engine cannot be fed by its
        // neighbours.
        let cross = if apu_available { 1.0 } else { 0.0 };
        // Every zone at the recovery temperature (module doc).
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
            pack_valve_open: self.controls.pack_valve_open,
            wai_selected: self.controls.wai_selected,
            starter_engaged: self.controls.starter_engaged,
            zone_air_k: [recovery_k; ZONE_COUNT],
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

        let mut area = live_system();
        run(area.as_mut(), &truth, &Faults::default(), 300);
        let map = published(area.as_ref());
        assert_eq!(map["DEEP_PNEU_APU_BLEED_VALVE_OPEN"], 1.0);
        assert!(map["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"] > 120_000.0, "the APU must pressurise engine 1's duct, got {}", map["DEEP_PNEU_ENG_1_DUCT_PRESSURE_PA"]);
        assert!(map["DEEP_PNEU_APU_DUCT_TEMPERATURE_C"] > 15.0, "load-compressor discharge must be hotter than the air it drew in");
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

