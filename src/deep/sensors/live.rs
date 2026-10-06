//! The live sensor set: the A380's actual air-data, angle-of-attack,
//! temperature, ice-detection, radio-altimeter, GPS and engine-speed
//! sensing hardware, instantiated once and stepped every frame.
//!
//! Everything under `deep::sensors` was written as probe physics with no
//! owner: `PitotProbe`, `StaticPort`, `AoaVane`, `TatProbe`,
//! `IceDetector`, `RadioAltimeter`, `GpsReceiver` and the ADR computation
//! all existed as types nothing in the plugin ever constructed. This module
//! is the owner, and the complement it builds is the real one:
//!
//! * **4 pitot probes** -- one per air data reference (ADR 1/2/3) plus the
//!   standby system, matching `registry.rs`'s own `34_nav.pitot_1..4`.
//! * **8 static ports** -- left and right for each of those four systems
//!   (`34_nav.static_<system>_<side>`), each pair joined by its own
//!   **averaging line** (`34_nav.static_avg_line_1..4`), which is a
//!   separately failable part.
//! * **3 AoA vanes**, one per ADIRU.
//! * **2 TAT probes**, captain's and first officer's. There are two probes
//!   and three ADRs, so ADR 3 shares the captain's probe -- an
//!   installation fact, not a modelling shortcut, and the reason a single
//!   TAT probe fault can move two ADRs' SAT at once.
//! * **2 ice detectors.**
//! * **3 radio altimeters**, each with its own transmit and receive
//!   antenna as separate failable parts.
//! * **3 GPS receivers**, each with its own antenna.
//! * **4 x 2 engine N1 speed pickups** -- EEC channel A and channel B per
//!   engine, the real dual-channel arrangement.
//! * **The standby (ISIS) outside air temperature probe**, a plain Pt100
//!   element rather than a recovery-corrected TAT probe.
//!
//! Each instance's fault inputs are resolved to the exact `registry.rs`
//! failure ids at construction, by component id and model field, so a
//! renumbering in the catalogue cannot silently detach a live probe from
//! its failure (see [`FaultIndex`]).
//!
//! ## What `Truth` cannot supply yet
//!
//! Nothing here invents an input (`docs/deep/BRIEF.md` hard rule 3). These
//! are the gaps, each named with the `Truth` field that would close it:
//!
//! * **True angle of attack** (`Truth::angle_of_attack_deg`). The AoA vanes
//!   are stepped with a true angle of zero, so their *icing, jamming and
//!   heater* behaviour -- which is what `DEEP_AOA_n_JAMMED` and the NAV AOA
//!   DISAGREE alert read -- is fully live, but the angle they report is
//!   not yet meaningful. The static ports are stepped at their own
//!   reference AoA, i.e. with no position-error correction applied at all,
//!   since there is no real angle to compute one from.
//! * **Cabin pressure** (`Truth::cabin_pressure_pa`). A static port's
//!   registered "leak into the pressurised fuselage" biases the reading
//!   toward cabin pressure; with no cabin pressure available the cabin is
//!   taken to be at ambient, which is true on the ground with the aircraft
//!   unpressurised and understates the fault in flight.
//! * **Radio height** (`Truth::radio_height_ft`, or terrain elevation).
//!   `Truth::altitude_ft` is used as the height above the surface, which is
//!   exact only over terrain at sea level.
//! * **Satellite visibility** (`Truth::gps_satellites_visible`). See
//!   [`NOMINAL_SATELLITES_VISIBLE`].
//! * **Engine N2/N3, TGT, fuel flow and vibration.** `Truth` carries
//!   `engine_n1_frac` only, so this module owns the N1 speed pickups and
//!   nothing else on the engine. The N2/N3 pickups, the TGT thermocouple
//!   harnesses, the vibration pickups, the P30/T25 probes and the fuel
//!   flow transmitters that `registry.rs` registers are *not* instantiated
//!   here. `Truth` has since grown `engine_n2_frac`, `engine_n3_frac`,
//!   `engine_hp_port_pressure_pa` and `engine_fuel_flow_kg_s`, so the
//!   N2/N3 pickups, the P30 probes and the fuel flow transmitters are now
//!   live in [`super::live_discrete`]; the TGT harnesses, the vibration
//!   pickups and the T25 probes still are not, for want of a true TGT, a
//!   vibration amplitude and a station-2.5 temperature.
//! * The discrete instrumentation `registry.rs` also registers (gear and
//!   door proximity, brake temperature and wear, tyre pressure, hydraulic
//!   and engine oil pressure/temperature/quantity, duct and cabin
//!   pressure/temperature, oxygen bottle pressure, smoke detectors) now
//!   lives in [`super::live_discrete`], which senses what the other deep
//!   areas publish rather than what `Truth` carries. The ones still
//!   without a source -- tyre pressure, the passenger and aft cargo doors,
//!   engine oil, vibration, TGT, T25, oxygen bottle pressure, the trim air
//!   duct -- are listed in `live_discrete::BLOCKED` with the variable each
//!   needs, and are not instantiated.

use super::adr;
use super::aoa_vane::{AoaVane, AoaVaneFaults};
use super::gps::{GpsFaults, GpsReceiver};
use super::ice_detector::{IceDetector, IceDetectorFaults};
use super::pitot::{PitotFaults, PitotProbe};
use super::radio_altimeter::{RadioAltimeter, RadioAltimeterFaults};
use super::static_port::{
    average_pair, StaticAveragingLineFaults, StaticPort, StaticPortFaults,
};
use super::tat_probe::{TatProbe, TatProbeFaults};
use super::discrete::{self, TemperatureSensorFaults};
use super::live_discrete::DiscreteSensors;
use super::{engine_sensors, registry};
use crate::deep::api::Registry;
use crate::deep::integration::weather_truth;
use crate::deep::live::{Area, Faults, Truth};
use std::collections::BTreeMap;

/// The live system for this area.
pub fn live_system() -> Box<dyn Area> {
    Box::new(LiveSensors::new())
}

// ---------------------------------------------------------------------
// Constants this module (not the probe models) owns.
// ---------------------------------------------------------------------

/// A 115 V AC bus is alive above this. Probe heaters are AC loads, so a
/// bus below this simply does not heat its probes.
const AC_BUS_ALIVE_V: f64 = 90.0;

/// A heater-current monitor calls the element failed once it is drawing
/// less than this fraction of rated power while its bus is alive -- which
/// is exactly what `DEEP_PITOT_n_HEATER_FAILED` / `DEEP_TAT_n_HEATER_FAILED`
/// report. GENERIC threshold: half of rated is far below any healthy
/// element's draw and far above zero, so neither a healthy probe nor a
/// fully open element is ambiguous.
const HEATER_FAILED_FRACTION: f64 = 0.5;

/// Satellites an unobstructed airborne GPS antenna has in view. **Derived
/// from published constellation geometry, not picked.**
///
/// A satellite at orbital radius `r` is above a user's mask elevation `eps`
/// exactly when its Earth-central angle from the user is at most
///
///   `psi = arccos(R_e * cos(eps) / r) - eps`
///
/// (standard spherical Earth/satellite visibility geometry, e.g. Wertz,
/// *Space Mission Analysis and Design*). The spherical cap that subtends is
/// `(1 - cos psi)/2` of the whole sphere, and a constellation designed for
/// uniform global coverage puts that same fraction of its satellites there
/// on average.
///
/// Numbers, all public:
/// - `R_e = 6371 km` (mean Earth radius, IUGG).
/// - `r = 26_560 km`: GPS semi-major axis, from the 20_180 km nominal
///   orbital altitude in the GPS Space Segment / Navstar GPS Space Segment
///   Navigation User Interfaces (IS-GPS-200) and the GPS Standard
///   Positioning Service Performance Standard.
/// - `eps = 5 deg`: the mask angle GNSS avionics standards work to (RTCA
///   DO-229, WAAS MOPS, states its accuracy and availability against a
///   5-degree mask; ARINC 743A GNSS sensor units use the same).
///
///   `R_e cos(5 deg) / r = 6371 * 0.996195 / 26_560 = 0.238957`
///   `arccos(0.238957) = 76.177 deg`,  `psi = 71.177 deg`
///   `(1 - cos 71.177 deg)/2 = (1 - 0.322865)/2 = 0.338568`
///
/// Applied to the constellation the SPS Performance Standard commits to
/// (at least 24 operational satellites for 95% of the time, the baseline
/// 24-slot constellation), that is `24 * 0.338568 = 8.13` in view -- which
/// is also, independently, exactly where `gps::REFERENCE_SATELLITES = 8`
/// sits, so the two constants are consistent rather than separately
/// invented. Against the constellation actually flown (31 operational
/// satellites, the long-running figure the US Space Force publishes for
/// the on-orbit GPS constellation), it is `31 * 0.338568 = 10.50`.
///
/// This model takes the flown constellation, floored to a whole satellite:
/// **10**. That is the same value an earlier revision of this file carried
/// as a GENERIC "typically around ten"; what has changed is that it is now
/// derived from published orbital geometry and a published constellation
/// size, so it can be checked and so its sensitivity is explicit -- a
/// 24-satellite constellation would give 8, and a 5-degree change in mask
/// angle moves it by about one satellite.
///
/// Still an approximation in one respect, honestly: GPS satellites sit in
/// six inclined planes rather than uniformly on the sphere, so the true
/// count at a given place and time oscillates about this mean (roughly 8
/// to 13 at mid latitudes). `Truth` carries no receiver position or time of
/// day, so the mean is what this file can offer; it is what
/// `Truth::gps_satellites_visible` should replace, and nothing but the GPS
/// noise magnitude and the jamming threshold depends on it.
const NOMINAL_SATELLITES_VISIBLE: u32 = 10;

/// Indicating range of the standby outside air temperature display, deg C.
/// GENERIC: an OAT display covering every temperature an airliner ever
/// meets, from a cold-soaked tropopause to a hot-and-high ramp, with the
/// margin a real instrument's range carries -- what it is for here is that
/// an open or shorted Pt100 pegs to an end of *some* range rather than
/// running off to infinity.
const OAT_RANGE_C: (f64, f64) = (-99.0, 99.0);

/// NAV ADR DISAGREE monitor thresholds. GENERIC: the A380's own monitor
/// thresholds are not public, so these are the figures commonly quoted for
/// the Airbus ADR comparison monitors (16 kt of computed airspeed, 250 ft
/// of pressure altitude), which sit far above the cross-channel scatter
/// this model's healthy probes produce (their position errors differ by
/// fractions of a percent) and far below any real single-probe blockage.
const ADR_DISAGREE_CAS_MS: f64 = 16.0 * 0.514_444;
const ADR_DISAGREE_ALT_M: f64 = 250.0 * 0.304_8;

/// Full-scale values for the catalogue entries whose magnitude is not a
/// 0..1 fraction but a signed physical offset (`registry.rs` says so in
/// each one's `magnitude` text). `deep::live::Faults` carries a 0..1
/// magnitude for every failure, so each is scaled onto its own full scale
/// here, and the sign is the one that produces the registered effect.
mod full_scale {
    /// Radio altimeter fixed-offset fault, ft. Negative: the registered
    /// effect cites the historically documented negative ground reading,
    /// and a low height reading is the hazardous direction.
    pub const RA_FALSE_OFFSET_FT: f64 = -20.0;
    /// Ice detector electronics/probe bias, as a fractional resonant
    /// frequency shift. Sized just above the detector's own detection
    /// threshold (`ice_detector`'s `DETECTION_MASS_FRACTION * 0.5` =
    /// 0.025), so a fully armed bias can fabricate a detection with no ice
    /// present -- the registered false-positive effect.
    pub const ICE_DETECTOR_SHIFT: f64 = 0.05;
    /// GPS spoofing walk-off target, m per axis. GENERIC: a kilometre is
    /// far beyond any navigation tolerance while remaining a plausible
    /// gradual walk-off rather than an obvious jump.
    pub const GPS_SPOOF_OFFSET_M: f64 = 1_000.0;
}

// ---------------------------------------------------------------------
// Failure id resolution.
// ---------------------------------------------------------------------

/// Maps `(component id, model field)` to the failure id `registry.rs`
/// assigned it.
///
/// This area's catalogue is generated from instance-name tables with a
/// running per-ATA-chapter counter, so its ids are not constants that can
/// be named directly. Rather than duplicate that counting here -- which
/// would break silently the first time an instance is inserted in the
/// middle -- the index is built by running the real registration into a
/// throwaway `Registry` and reading back what it produced.
pub struct FaultIndex(BTreeMap<(String, String), u64>);

impl FaultIndex {
    pub fn build() -> Self {
        let mut r = Registry::default();
        registry::register(&mut r);
        let mut map = BTreeMap::new();
        for f in &r.failures {
            // `register_instance` writes `model_field` as
            // `"<model path>(faults.<field>)"`.
            if let Some(start) = f.model_field.find("(faults.") {
                let field = &f.model_field[start + "(faults.".len()..];
                if let Some(field) = field.strip_suffix(')') {
                    map.insert((f.component.clone(), field.to_string()), f.id);
                }
            }
        }
        Self(map)
    }

    /// The id for one component's one fault field, or 0 -- which
    /// `Faults::get` reads as healthy -- if the catalogue has no such
    /// entry. `every_live_sensor_resolved_a_real_failure_id` asserts that
    /// never happens for anything this module actually owns.
    pub fn id(&self, component: &str, field: &str) -> u64 {
        self.0
            .get(&(component.to_string(), field.to_string()))
            .copied()
            .unwrap_or(0)
    }

    pub fn ids<const N: usize>(&self, component: &str, fields: [&str; N]) -> [u64; N] {
        fields.map(|f| self.id(component, f))
    }
}

// ---------------------------------------------------------------------
// The live area.
// ---------------------------------------------------------------------

/// One air data system: its pitot probe, its left/right static ports and
/// the averaging line that joins them.
struct AirDataChannel {
    pitot: PitotProbe,
    static_left: StaticPort,
    static_right: StaticPort,
    pitot_faults: [u64; 4],
    static_left_faults: [u64; 2],
    static_right_faults: [u64; 2],
    averaging_line_fault: u64,
    /// Which AC bus this system's probe heaters are fed from.
    ac_bus: usize,
}

struct LiveVane {
    vane: AoaVane,
    faults: [u64; 4],
    ac_bus: usize,
}

struct LiveTat {
    probe: TatProbe,
    faults: [u64; 2],
    ac_bus: usize,
}

struct LiveIceDetector {
    detector: IceDetector,
    faults: [u64; 3],
}

struct LiveRadioAltimeter {
    unit: RadioAltimeter,
    transceiver_faults: [u64; 3],
    tx_antenna_fault: u64,
    rx_antenna_faults: [u64; 2],
}

struct LiveGps {
    receiver: GpsReceiver,
    receiver_faults: [u64; 3],
    antenna_faults: [u64; 2],
}

/// What `tick` computed and `publish` hands out. Held as state because
/// `Area::publish` takes `&self`.
#[derive(Clone, Copy, Debug, Default)]
struct Snapshot {
    pitot_heater_failed: [bool; 4],
    pitot_blocked: [bool; 4],
    pitot_ice_kg: [f64; 4],
    static_degraded: [bool; 4],
    adr: [adr::AdrOutputs; 3],
    standby: adr::AdrOutputs,
    adr_disagree: bool,
    adr_cas_vote: adr::VoteResult,
    adr_alt_vote: adr::VoteResult,
    aoa_jammed: [bool; 3],
    aoa_deg: [f64; 3],
    aoa_heater_failed: [bool; 3],
    tat_heater_failed: [bool; 2],
    tat_c: [f64; 2],
    ice_detected: [bool; 2],
    ice_kg: [f64; 2],
    ra_valid: [bool; 3],
    ra_in_range: [bool; 3],
    ra_agl_ft: [f64; 3],
    gps_valid: [bool; 3],
    /// `340800033`: jamming armed but the fix still holds (see `tick`'s own
    /// doc next to where this is set).
    gps_degraded: [bool; 3],
    gps_error_m: [f64; 3],
    gps_offset_m: [[f64; 2]; 3],
    standby_oat_c: f64,
    /// Per engine, per EEC channel (A, B).
    n1_pickup_valid: [[bool; 2]; 4],
    n1_pickup_frac: [[f64; 2]; 4],
    total_heater_power_w: f64,
    /// E-ELEC Phase 2 additions -- see `LiveSensors`'s own struct doc.
    oat_1_2_heater_failed: [bool; 2],
    sideslip_jammed: [bool; 3],
    sideslip_heater_failed: [bool; 3],
    tat3_heater_failed: bool,
    /// `340800016 NAV CAPT AND F/O ALT DISAGREE`: `|ADR1 - ADR2| pressure
    /// altitude`, feet, the same two channels this port's `AC ESS`/GEN
    /// numbering elsewhere treats as CAPT (1) and F.O (2).
    adr_capt_fo_alt_diff_ft: f64,
    /// `340800017`/`020`/`316800002`: how far the IRs selected for the
    /// captain's and the first officer's side disagree, degrees, from
    /// their own published outputs (`Truth::ir`); 0 while either word is
    /// invalid, since an invalid IR flags its parameter instead.
    ir_capt_fo_pitch_diff_deg: f64,
    ir_capt_fo_roll_diff_deg: f64,
    ir_capt_fo_hdg_diff_deg: f64,
    ir_capt_fo_fpa_diff_deg: f64,
    /// Each IR's armed gyro drift, deg/hr, for `physics::adirs` to add to
    /// that unit's gyros (see [`IR_GYRO_DRIFT_AT_FULL_FAULT_DEG_HR`]).
    ir_gyro_drift_deg_hr: [f64; 3],
    /// `313800001`/`002`/`005`/`006`: each KCCU's cursor control device
    /// and keyboard BITE, `[capt, fo][ccd, keyboard]`.
    kccu_part_failed: [[bool; 2]; 2],
    /// `311800012`: each monitored display unit's CDS display/monitor
    /// comparison, in [`CDS_MONITORED_DUS`] order.
    du_display_monitor_disagree: [bool; 5],
    /// `340800018 NAV CAPT AND F/O BARO REF DISAGREE`.
    baro_ref_disagree: bool,
}

/// The display units `deep::electrical` models as their own loads, in the
/// order `311800012` CDS DISPLAY DISAGREE monitors them. FCOM p.5393 lists
/// the SD and both MFDs too; they have no component of their own yet.
pub const CDS_MONITORED_DUS: [&str; 5] = ["capt-pfd-du", "capt-nd-du", "capt-ewd-du", "fo-pfd-du", "fo-nd-du"];

/// The variable [`LiveSensors`] publishes one [`CDS_MONITORED_DUS`] entry's
/// display/monitor comparison on.
pub fn cds_monitor_var(du: &str) -> String {
    format!("DEEP_CDS_{}_MONITOR_DISAGREE", du.to_ascii_uppercase().replace('-', "_"))
}

/// Full-magnitude `alignment_drift`: gyro bias added to all three of that
/// IR's gyros. GENERIC: no public ARINC 704 or A380 figure for a degraded
/// but undetected ring-laser gyro; 100 deg/hr is four orders of magnitude
/// past the 0.01 deg/hr navigation-grade bias `physics::adirs` draws, and
/// sized so a full fault carries one IR past the FCOM's 5 deg ATT DISAGREE
/// threshold within a few minutes (tilt grows as bias times time well
/// inside the 84 minute Schuler period), as a failing unit would.
pub const IR_GYRO_DRIFT_AT_FULL_FAULT_DEG_HR: f64 = 100.0;

/// How far two IR words disagree, degrees; 0 unless both are valid.
/// `wrap` for angles that wrap at 360 (roll, heading).
fn ir_disagreement_deg(a: Option<f64>, b: Option<f64>, wrap: bool) -> f64 {
    match (a, b) {
        (Some(a), Some(b)) if wrap => ((a - b + 540.0).rem_euclid(360.0) - 180.0).abs(),
        (Some(a), Some(b)) => (a - b).abs(),
        _ => 0.0,
    }
}

pub struct LiveSensors {
    channels: [AirDataChannel; 4],
    vanes: [LiveVane; 3],
    tat: [LiveTat; 2],
    ice: [LiveIceDetector; 2],
    ra: [LiveRadioAltimeter; 3],
    gps: [LiveGps; 3],
    /// Per engine, per EEC channel.
    n1_pickup_faults: [[[u64; 2]; 2]; 4],
    /// Standby OAT probe: open circuit, short circuit.
    oat_faults: [u64; 2],
    /// E-ELEC Phase 2: `340800050`/`051` OAT probes 1/2 (heater-failure
    /// boolean each, no simulated reading -- see `registry::
    /// register_oat_probe_1_2`).
    oat_1_2_faults: [u64; 2],
    /// `340800064`-`066` sideslip vanes 1-3: `[heater_failure,
    /// mechanically_stuck]` per unit (see `registry::register_sideslip_vane`).
    sideslip_faults: [[u64; 2]; 3],
    /// `340800070` TAT probe 3: `[heater_failure, recovery_degradation]`,
    /// only the first feeding this alert (see `registry::
    /// register_tat_probe_3`).
    tat3_faults: [u64; 2],
    /// `340800017`/`020`: one alignment/gyro-drift fault per IR unit (see
    /// `registry::register_inertial_reference`).
    ir_faults: [u64; 3],
    /// `[capt, fo][ccd_failed, keyboard_failed]` (see `registry::
    /// register_kccu_parts`).
    kccu_faults: [[u64; 2]; 2],
    /// One `display_monitor_disagree` per [`CDS_MONITORED_DUS`] entry (see
    /// `registry::register_cds_display_monitors`).
    du_faults: [u64; 5],
    /// Every sensor that senses something another deep area computes --
    /// gear and door proximity, brake temperature and wear, smoke,
    /// hydraulic reservoir and system transducers, duct temperature, the
    /// engine core speed pickups, P30 and the fuel flow transmitters. See
    /// `live_discrete`.
    discrete: DiscreteSensors,
    snapshot: Snapshot,
}

impl Default for LiveSensors {
    fn default() -> Self {
        Self::new()
    }
}

impl LiveSensors {
    pub fn new() -> Self {
        let index = FaultIndex::build();
        // Everything is seated at ISA sea level, the state
        // `Truth::default()` describes; every probe's own lag then carries
        // it to wherever the aircraft actually is within a second or two of
        // the first real frame.
        const ISA_SEA_LEVEL_PA: f64 = 101_325.0;

        // GENERIC bus allocation: `Truth` gives four AC buses and the
        // aircraft has four probe systems, so system n is fed from bus n.
        // The real A380's probe-heat bus assignment is not public; what
        // matters physically, and is preserved here, is that the systems do
        // not share a bus -- losing one bus must not take out more than one
        // system's probes.
        let channels = core::array::from_fn(|i| {
            let n = i + 1;
            AirDataChannel {
                pitot: PitotProbe::new(ISA_SEA_LEVEL_PA),
                static_left: StaticPort::new(ISA_SEA_LEVEL_PA),
                static_right: StaticPort::new(ISA_SEA_LEVEL_PA),
                pitot_faults: index.ids(
                    &format!("34_nav.pitot_{n}"),
                    [
                        "heater_failure",
                        "insect_or_tape_blockage",
                        "mechanical_damage",
                        "drain_blocked",
                    ],
                ),
                static_left_faults: index
                    .ids(&format!("34_nav.static_{n}_1"), ["blocked", "leak_to_cabin"]),
                static_right_faults: index
                    .ids(&format!("34_nav.static_{n}_2"), ["blocked", "leak_to_cabin"]),
                averaging_line_fault: index
                    .id(&format!("34_nav.static_avg_line_{n}"), "line_blocked"),
                ac_bus: i,
            }
        });

        let vanes = core::array::from_fn(|i| LiveVane {
            // The seed only drives each vane's own resolver-drift random
            // walk; distinct seeds are what stop three vanes drifting in
            // lockstep, which would defeat the voter they feed.
            vane: AoaVane::new(0x5A0A_0001 + i as u64, 0.0),
            faults: index.ids(
                &format!("34_nav.aoa_{}", i + 1),
                ["heater_failure", "mechanically_stuck", "resolver_wear", "damage"],
            ),
            ac_bus: i,
        });

        let tat = core::array::from_fn(|i| LiveTat {
            probe: TatProbe::new(15.0),
            faults: index.ids(
                &format!("34_nav.tat_{}", i + 1),
                ["heater_failure", "recovery_degradation"],
            ),
            ac_bus: i,
        });

        let ice = core::array::from_fn(|i| LiveIceDetector {
            detector: IceDetector::new(),
            faults: index.ids(
                &format!("30_ice.detector_{}", i + 1),
                ["heater_failure", "frequency_sensor_bias", "probe_damage_bias"],
            ),
        });

        let ra = core::array::from_fn(|i| {
            let n = i + 1;
            LiveRadioAltimeter {
                unit: RadioAltimeter::new(0x4A17_0001 + i as u64),
                transceiver_faults: index.ids(
                    &format!("34_nav.ra_transceiver_{n}"),
                    ["transceiver_fault", "false_offset_ft", "tracking_loop_degradation"],
                ),
                tx_antenna_fault: index
                    .id(&format!("34_nav.ra_tx_antenna_{n}"), "tx_antenna_fault"),
                rx_antenna_faults: index.ids(
                    &format!("34_nav.ra_rx_antenna_{n}"),
                    ["rx_antenna_fault", "rx_antenna_degradation"],
                ),
            }
        });

        let gps = core::array::from_fn(|i| {
            let n = i + 1;
            LiveGps {
                receiver: GpsReceiver::new(0x6950_0001u64 + i as u64),
                receiver_faults: index.ids(
                    &format!("34_nav.gps_receiver_{n}"),
                    ["receiver_fault", "jamming", "spoof_target_offset_m"],
                ),
                antenna_faults: index.ids(
                    &format!("34_nav.gps_antenna_{n}"),
                    ["antenna_fault", "antenna_degradation"],
                ),
            }
        });

        let n1_pickup_faults = core::array::from_fn(|e| {
            core::array::from_fn(|ch| {
                let channel = if ch == 0 { "a" } else { "b" };
                index.ids(
                    &format!("77_eng.speed_n1_{}_{channel}", e + 1),
                    ["air_gap_increase", "open_circuit"],
                )
            })
        });

        let oat_faults = index.ids("34_nav.oat_standby", ["open_circuit", "short_circuit"]);

        // E-ELEC Phase 2: see `registry::register_oat_probe_1_2`/
        // `register_sideslip_vane`/`register_tat_probe_3`'s own doc for why
        // these are boolean-only faults, not a full probe/vane engine.
        let oat_1_2_faults = core::array::from_fn(|i| index.id(&format!("34_nav.oat_{}", i + 1), "heater_failure"));
        let sideslip_faults = core::array::from_fn(|i| index.ids(&format!("34_nav.sideslip_{}", i + 1), ["heater_failure", "mechanically_stuck"]));
        let tat3_faults = index.ids("34_nav.tat_3", ["heater_failure", "recovery_degradation"]);
        let ir_faults = core::array::from_fn(|i| index.id(&format!("34_nav.ir_{}", i + 1), "alignment_drift"));
        let kccu_faults = ["capt", "fo"].map(|side| index.ids(&format!("31_elec.kccu-{side}"), ["ccd_failed", "keyboard_failed"]));
        let du_faults = CDS_MONITORED_DUS.map(|du| index.id(&format!("31_elec.{du}"), "display_monitor_disagree"));

        let discrete = DiscreteSensors::new(&index);

        Self {
            channels,
            vanes,
            tat,
            ice,
            ra,
            gps,
            n1_pickup_faults,
            oat_faults,
            oat_1_2_faults,
            sideslip_faults,
            tat3_faults,
            ir_faults,
            kccu_faults,
            du_faults,
            discrete,
            snapshot: Snapshot::default(),
        }
    }

    /// Whether the AC bus feeding a probe heater is alive.
    fn bus_alive(truth: &Truth, bus: usize) -> bool {
        truth.ac_bus_volts.get(bus).copied().unwrap_or(0.0) > AC_BUS_ALIVE_V
    }

    /// Free-stream total (pitot) pressure: the isentropic subsonic
    /// relation `pt = p * (1 + (gamma-1)/2 * M^2)^(gamma/(gamma-1))` on
    /// `Truth`'s own ambient pressure and Mach. The probe models want the
    /// *true* pressure at the tip; what they report after their own icing,
    /// blockage and lag is the sensed one.
    fn true_total_pressure_pa(static_pa: f64, mach: f64) -> f64 {
        static_pa.max(1.0) * (1.0 + 0.2 * mach.max(0.0).powi(2)).powf(3.5)
    }

    /// Cloud liquid water content at the aircraft, g/m^3 -- the icing
    /// driver every heated probe here shares. Real X-Plane weather, through
    /// `integration::weather_truth`'s own Part 25 Appendix C envelope; zero
    /// when X-Plane reports no weather, which that module is explicit must
    /// be read as "unknown", never as "clear".
    fn lwc_gm3(truth: &Truth) -> f64 {
        let Some(weather) = truth.environment.weather.as_ref() else { return 0.0 };
        let cloud = weather_truth::dominant_cloud(weather);
        weather_truth::lwc_kg_m3_from_conditions(truth.environment.sat_c, cloud) * 1_000.0
    }
}

impl Area for LiveSensors {
    fn name(&self) -> &'static str {
        "sensors"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        let dt = truth.dt_s;
        let env = &truth.environment;
        let static_pa = env.ambient_pressure_pa.max(1.0);
        let mach = env.mach();
        let total_pa = Self::true_total_pressure_pa(static_pa, mach);
        let lwc_gm3 = Self::lwc_gm3(truth);
        // See the module docs: no true AoA and no cabin pressure in `Truth`.
        // The vanes are stepped at zero (their icing/jamming behaviour is
        // what is live; the angle they report is not yet meaningful). The
        // static ports are stepped at their own *reference* AoA instead,
        // which is the angle at which their position error is zero by
        // definition -- with no real AoA to compute a correction from, the
        // honest thing is to apply none, rather than to apply the
        // correction for an angle the aircraft is not at (at a nominal zero
        // the error would be 0.6% of ambient, which on the ground at rest
        // is a fabricated 60 kt of computed airspeed).
        let true_aoa_deg = 0.0;
        let static_port_aoa_deg = super::static_port::REFERENCE_AOA_DEG;
        let cabin_pa = static_pa;

        let mut snap = Snapshot::default();
        let mut heater_w = 0.0;

        // ---- Pitot/static, per air data system -------------------------
        let mut sensed_total = [0.0; 4];
        let mut sensed_static = [0.0; 4];
        for (i, ch) in self.channels.iter_mut().enumerate() {
            let powered = Self::bus_alive(truth, ch.ac_bus);
            let pitot_faults = PitotFaults {
                heater_failure: faults.get(ch.pitot_faults[0]),
                insect_or_tape_blockage: faults.get(ch.pitot_faults[1]),
                mechanical_damage: faults.get(ch.pitot_faults[2]),
                drain_blocked: faults.get(ch.pitot_faults[3]),
            };
            let pitot = ch.pitot.step(
                total_pa,
                static_pa,
                env.tas_ms,
                env.sat_c,
                lwc_gm3,
                powered,
                &pitot_faults,
                dt,
            );

            let left = ch.static_left.step(
                static_pa,
                cabin_pa,
                static_port_aoa_deg,
                mach,
                &StaticPortFaults {
                    blocked: faults.get(ch.static_left_faults[0]),
                    leak_to_cabin: faults.get(ch.static_left_faults[1]),
                },
                dt,
            );
            let right = ch.static_right.step(
                static_pa,
                cabin_pa,
                static_port_aoa_deg,
                mach,
                &StaticPortFaults {
                    blocked: faults.get(ch.static_right_faults[0]),
                    leak_to_cabin: faults.get(ch.static_right_faults[1]),
                },
                dt,
            );
            let pair = average_pair(
                left,
                right,
                &StaticAveragingLineFaults {
                    line_blocked: faults.get(ch.averaging_line_fault),
                },
            );

            sensed_total[i] = pitot.sensed_total_pressure_pa;
            sensed_static[i] = pair.averaged_pa;
            snap.pitot_blocked[i] = pitot.blocked;
            snap.pitot_ice_kg[i] = pitot.ice_kg;
            snap.static_degraded[i] = pair.degraded;
            snap.pitot_heater_failed[i] = powered
                && pitot.heater_power_w < HEATER_FAILED_FRACTION * super::pitot::RATED_HEATER_W;
            heater_w += pitot.heater_power_w;
        }

        // ---- TAT probes ------------------------------------------------
        for (i, t) in self.tat.iter_mut().enumerate() {
            let powered = Self::bus_alive(truth, t.ac_bus);
            let out = t.probe.step(
                env.sat_c,
                mach,
                env.tas_ms,
                lwc_gm3,
                powered,
                &TatProbeFaults {
                    heater_failure: faults.get(t.faults[0]),
                    recovery_degradation: faults.get(t.faults[1]),
                },
                dt,
            );
            snap.tat_c[i] = out.sensed_tat_c;
            snap.tat_heater_failed[i] = powered
                && out.heater_power_w < HEATER_FAILED_FRACTION * super::tat_probe::RATED_HEATER_W;
            heater_w += out.heater_power_w;
        }

        // ---- The three ADRs and the voter ------------------------------
        // Two TAT probes feed three ADRs; ADR 3 shares the captain's probe
        // (see the module docs).
        let tat_for_adr = [snap.tat_c[0], snap.tat_c[1], snap.tat_c[0]];
        for i in 0..3 {
            snap.adr[i] = adr::compute(sensed_total[i], sensed_static[i], tat_for_adr[i]);
        }
        snap.standby = adr::compute(sensed_total[3], sensed_static[3], snap.tat_c[0]);
        snap.adr_cas_vote = adr::vote3(
            snap.adr[0].cas_ms,
            snap.adr[1].cas_ms,
            snap.adr[2].cas_ms,
            ADR_DISAGREE_CAS_MS,
        );
        snap.adr_alt_vote = adr::vote3(
            snap.adr[0].pressure_altitude_m,
            snap.adr[1].pressure_altitude_m,
            snap.adr[2].pressure_altitude_m,
            ADR_DISAGREE_ALT_M,
        );
        snap.adr_disagree = snap.adr_cas_vote.disagree || snap.adr_alt_vote.disagree;

        // ---- AoA vanes -------------------------------------------------
        for (i, v) in self.vanes.iter_mut().enumerate() {
            let powered = Self::bus_alive(truth, v.ac_bus);
            let stuck = faults.get(v.faults[1]);
            let out = v.vane.step(
                true_aoa_deg,
                env.tas_ms,
                env.sat_c,
                lwc_gm3,
                powered,
                &AoaVaneFaults {
                    heater_failure: faults.get(v.faults[0]),
                    mechanically_stuck: stuck,
                    resolver_wear: faults.get(v.faults[2]),
                    damage: faults.get(v.faults[3]),
                },
                dt,
            );
            snap.aoa_deg[i] = out.sensed_aoa_deg;
            // The vane is jammed whether the ice froze it or the bearing
            // seized it -- the two causes `registry.rs` names as raising
            // NAV AOA DISAGREE.
            snap.aoa_jammed[i] = out.jammed_by_ice || stuck >= 0.98;
            snap.aoa_heater_failed[i] = powered
                && out.heater_power_w < HEATER_FAILED_FRACTION * super::aoa_vane::RATED_HEATER_W;
            heater_w += out.heater_power_w;
        }

        // ---- Ice detectors ---------------------------------------------
        for (i, d) in self.ice.iter_mut().enumerate() {
            let out = d.detector.step(
                env.sat_c,
                env.tas_ms,
                lwc_gm3,
                &IceDetectorFaults {
                    heater_failure: faults.get(d.faults[0]),
                    frequency_sensor_bias: faults.get(d.faults[1])
                        * full_scale::ICE_DETECTOR_SHIFT,
                    probe_damage_bias: faults.get(d.faults[2]) * full_scale::ICE_DETECTOR_SHIFT,
                },
                dt,
            );
            snap.ice_detected[i] = out.ice_detected;
            snap.ice_kg[i] = out.ice_kg;
        }

        // ---- Radio altimeters ------------------------------------------
        // See the module docs: `Truth::altitude_ft` stands in for radio
        // height, and no `Truth` field says what is underneath.
        let agl_ft = if truth.on_ground { 0.0 } else { truth.altitude_ft.max(0.0) };
        let in_range = (0.0..=super::radio_altimeter::MAX_RANGE_FT).contains(&agl_ft);
        for (i, r) in self.ra.iter_mut().enumerate() {
            let out = r.unit.step(
                agl_ft,
                false,
                &RadioAltimeterFaults {
                    transceiver_fault: faults.get(r.transceiver_faults[0]),
                    tx_antenna_fault: faults.get(r.tx_antenna_fault),
                    rx_antenna_fault: faults.get(r.rx_antenna_faults[0]),
                    rx_antenna_degradation: faults.get(r.rx_antenna_faults[1]),
                    false_offset_ft: faults.get(r.transceiver_faults[1])
                        * full_scale::RA_FALSE_OFFSET_FT,
                    tracking_loop_degradation: faults.get(r.transceiver_faults[2]),
                },
            );
            snap.ra_valid[i] = out.valid;
            snap.ra_in_range[i] = in_range;
            snap.ra_agl_ft[i] = out.agl_ft;
        }

        // ---- GPS receivers ---------------------------------------------
        for (i, g) in self.gps.iter_mut().enumerate() {
            let spoof = faults.get(g.receiver_faults[2]) * full_scale::GPS_SPOOF_OFFSET_M;
            let out = g.receiver.step(
                NOMINAL_SATELLITES_VISIBLE,
                &GpsFaults {
                    receiver_fault: faults.get(g.receiver_faults[0]),
                    jamming: faults.get(g.receiver_faults[1]),
                    antenna_fault: faults.get(g.antenna_faults[0]),
                    antenna_degradation: faults.get(g.antenna_faults[1]),
                    spoof_target_offset_m: [spoof, spoof],
                },
                dt,
            );
            snap.gps_valid[i] = out.valid;
            snap.gps_error_m[i] = out.position_error_1sigma_m;
            snap.gps_offset_m[i] = out.position_offset_m;
            // `340800033 NAV GNSS SIGNAL DEGRADED`: the already-tested
            // intermediate state between healthy and failed --
            // `armed_gps_jamming_costs_that_receiver_its_fix`'s own "mild"
            // case (jamming armed but the fix still holds), not a new
            // threshold.
            snap.gps_degraded[i] = faults.get(g.receiver_faults[1]) > 0.0 && out.valid;
        }

        // ---- Engine N1 speed pickups, EEC channels A and B -------------
        for engine in 0..4 {
            let true_frac = truth.engine_n1_frac[engine].max(0.0);
            for channel in 0..2 {
                let ids = self.n1_pickup_faults[engine][channel];
                let out = engine_sensors::speed_pickup_reading(
                    true_frac,
                    &engine_sensors::SpeedPickupFaults {
                        air_gap_increase: faults.get(ids[0]),
                        open_circuit: faults.get(ids[1]),
                    },
                );
                snap.n1_pickup_valid[engine][channel] = out.valid;
                snap.n1_pickup_frac[engine][channel] = out.speed_frac;
            }
        }

        // ---- Standby (ISIS) outside air temperature probe --------------
        // An unheated, direct-reading Pt100: it reads static air
        // temperature, not the recovery temperature the TAT probes read.
        snap.standby_oat_c = discrete::temperature_sensor_reading_c(
            env.sat_c,
            OAT_RANGE_C,
            &TemperatureSensorFaults {
                open_circuit: faults.get(self.oat_faults[0]),
                short_circuit: faults.get(self.oat_faults[1]),
            },
        );

        // ---- E-ELEC Phase 2 additions -----------------------------------
        // `340800050`/`051` OAT probes 1/2, `340800064`-`066` sideslip
        // vanes, `340800070` TAT probe 3: boolean-only faults (see each
        // `registry.rs` doc for why no reading is simulated), `> 0.0` is
        // the whole verdict, the same convention this whole pass uses for
        // every computer/monitoring-class fault.
        for i in 0..2 {
            snap.oat_1_2_heater_failed[i] = faults.get(self.oat_1_2_faults[i]) > 0.0;
        }
        for i in 0..3 {
            snap.sideslip_heater_failed[i] = faults.get(self.sideslip_faults[i][0]) > 0.0;
            snap.sideslip_jammed[i] = faults.get(self.sideslip_faults[i][1]) > 0.0;
        }
        snap.tat3_heater_failed = faults.get(self.tat3_faults[0]) > 0.0;

        // `340800016 NAV CAPT AND F/O ALT DISAGREE`: FCOM p.5569 gives 500 ft
        // (STD) or 250 ft (QNH); this pass applies the tighter, more
        // conservative 250 ft threshold unconditionally (no baro-mode input
        // exists here to pick between them -- see `ata34.rs`'s own citation)
        // rather than inventing which of the two applies.
        snap.adr_capt_fo_alt_diff_ft = (snap.adr[0].pressure_altitude_m - snap.adr[1].pressure_altitude_m).abs() * crate::M_TO_FT;

        // `340800017`/`020 NAV CAPT AND F/O ATT/HDG DISAGREE` and
        // `316800002 NAV HUD FPV DISAGREE`: the two IRs the ATT HDG knob
        // selects for the captain's and the first officer's side, compared
        // on their own published outputs. They are `physics::adirs`'s three
        // strapdown solutions, each with its own gyro and accelerometer
        // errors, so they only part company when one really drifts -- which
        // an armed `alignment_drift` does by adding gyro bias to that unit
        // (`ir_gyro_drift_deg_hr` below), not by offsetting its answer.
        let (capt, fo) = crate::deep::live::capt_fo_ir(truth.att_hdg_switching_knob);
        let (a, b) = (&truth.ir[capt], &truth.ir[fo]);
        snap.ir_capt_fo_pitch_diff_deg = ir_disagreement_deg(a.pitch_deg, b.pitch_deg, false);
        snap.ir_capt_fo_roll_diff_deg = ir_disagreement_deg(a.roll_deg, b.roll_deg, true);
        snap.ir_capt_fo_hdg_diff_deg = ir_disagreement_deg(a.true_heading_deg, b.true_heading_deg, true);
        snap.ir_capt_fo_fpa_diff_deg = ir_disagreement_deg(a.flight_path_angle_deg, b.flight_path_angle_deg, false);
        snap.ir_gyro_drift_deg_hr = core::array::from_fn(|i| faults.get(self.ir_faults[i]) * IR_GYRO_DRIFT_AT_FULL_FAULT_DEG_HR);

        // `313800001`/`002`/`005`/`006` CDS CAPT (F/O) CURSOR CTL /
        // KEYBOARD FAULT: FCOM p.5377/5383, "the Cursor Control Device (the
        // Keyboard) is failed" -- the KCCU's own BITE on either part, which
        // FCOM DSC-31-30-10 says fail independently of each other.
        snap.kccu_part_failed = core::array::from_fn(|side| core::array::from_fn(|part| faults.get(self.kccu_faults[side][part]) > 0.0));
        // `311800012` CDS DISPLAY DISAGREE: FCOM p.5393, the CDS sends what
        // a display unit shows to a second unit to monitor, and the two
        // disagree.
        snap.du_display_monitor_disagree = core::array::from_fn(|i| faults.get(self.du_faults[i]) > 0.0);

        // `340800018 NAV CAPT AND F/O BARO REF DISAGREE` -- FCOM p.5572:
        // "The Captain's barometric reference is QNH(STD), and the First
        // Officer's barometric reference is STD(QNH)." FlyByWire's own raw
        // `A32NX_FCU_EFIS_{L,R}_DISPLAY_BARO_MODE` enum (`Truth::controls::
        // baro_mode`, E-ELEC Phase 2) already carries exactly this state;
        // this port does not need to decode which enum value is STD vs
        // QNH, only that the two sides differ.
        snap.baro_ref_disagree = truth.controls.baro_mode[0] != truth.controls.baro_mode[1];

        snap.total_heater_power_w = heater_w;
        self.snapshot = snap;

        // ---- Everything that senses another area's output ---------------
        self.discrete.tick(truth, faults);
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        let s = &self.snapshot;

        // --- the variables `registry.rs` triggers ECAM alerts on ---------
        out("DEEP_ADR_VOTE_DISAGREE", f64::from(s.adr_disagree));
        for i in 0..4 {
            out(
                &format!("DEEP_PITOT_{}_HEATER_FAILED", i + 1),
                f64::from(s.pitot_heater_failed[i]),
            );
        }
        for i in 0..3 {
            out(&format!("DEEP_AOA_{}_JAMMED", i + 1), f64::from(s.aoa_jammed[i]));
        }
        for i in 0..2 {
            out(
                &format!("DEEP_TAT_{}_HEATER_FAILED", i + 1),
                f64::from(s.tat_heater_failed[i]),
            );
        }
        for i in 0..3 {
            out(&format!("DEEP_RA_{}_VALID", i + 1), f64::from(s.ra_valid[i]));
            out(&format!("DEEP_GPS_{}_VALID", i + 1), f64::from(s.gps_valid[i]));
            out(&format!("DEEP_GPS_{}_DEGRADED", i + 1), f64::from(s.gps_degraded[i]));
        }

        // --- the rest of the sensor set, for the EFB Study pages ---------
        for i in 0..3 {
            let n = i + 1;
            out(&format!("DEEP_ADR_{n}_CAS_MS"), s.adr[i].cas_ms);
            out(&format!("DEEP_ADR_{n}_MACH"), s.adr[i].mach);
            out(&format!("DEEP_ADR_{n}_ALT_M"), s.adr[i].pressure_altitude_m);
            out(&format!("DEEP_ADR_{n}_TAS_MS"), s.adr[i].tas_ms);
            out(&format!("DEEP_ADR_{n}_SAT_C"), s.adr[i].sat_c);
            out(&format!("DEEP_ADR_{n}_OUTLIER"), f64::from(s.adr_cas_vote.outlier[i] || s.adr_alt_vote.outlier[i]));
            out(&format!("DEEP_AOA_{n}_DEG"), s.aoa_deg[i]);
            out(&format!("DEEP_AOA_{n}_HEATER_FAILED"), f64::from(s.aoa_heater_failed[i]));
            out(&format!("DEEP_RA_{n}_AGL_FT"), s.ra_agl_ft[i]);
            out(&format!("DEEP_RA_{n}_IN_RANGE"), f64::from(s.ra_in_range[i]));
            out(&format!("DEEP_GPS_{n}_ERROR_M"), s.gps_error_m[i]);
            out(&format!("DEEP_GPS_{n}_OFFSET_N_M"), s.gps_offset_m[i][0]);
            out(&format!("DEEP_GPS_{n}_OFFSET_E_M"), s.gps_offset_m[i][1]);
        }
        out("DEEP_ADR_VOTED_CAS_MS", s.adr_cas_vote.value);
        out("DEEP_ADR_VOTED_ALT_M", s.adr_alt_vote.value);
        out("DEEP_STANDBY_CAS_MS", s.standby.cas_ms);
        out("DEEP_STANDBY_ALT_M", s.standby.pressure_altitude_m);
        for i in 0..4 {
            let n = i + 1;
            out(&format!("DEEP_PITOT_{n}_BLOCKED"), f64::from(s.pitot_blocked[i]));
            out(&format!("DEEP_PITOT_{n}_ICE_KG"), s.pitot_ice_kg[i]);
            out(&format!("DEEP_STATIC_{n}_DEGRADED"), f64::from(s.static_degraded[i]));
        }
        for i in 0..2 {
            let n = i + 1;
            out(&format!("DEEP_TAT_{n}_C"), s.tat_c[i]);
            out(&format!("DEEP_ICE_DETECTOR_{n}"), f64::from(s.ice_detected[i]));
            out(&format!("DEEP_ICE_DETECTOR_{n}_ICE_KG"), s.ice_kg[i]);
        }
        for engine in 0..4 {
            for (channel, label) in ["A", "B"].iter().enumerate() {
                out(
                    &format!("DEEP_ENG_{}_N1_PICKUP_{label}_VALID", engine + 1),
                    f64::from(s.n1_pickup_valid[engine][channel]),
                );
                out(
                    &format!("DEEP_ENG_{}_N1_PICKUP_{label}_FRAC", engine + 1),
                    s.n1_pickup_frac[engine][channel],
                );
            }
        }
        out("DEEP_STANDBY_OAT_C", s.standby_oat_c);
        out("DEEP_PROBE_HEAT_TOTAL_W", s.total_heater_power_w);

        // ---- E-ELEC Phase 2 additions -----------------------------------
        for i in 0..2 {
            out(&format!("DEEP_OAT_{}_HEATER_FAILED", i + 1), f64::from(s.oat_1_2_heater_failed[i]));
        }
        for i in 0..3 {
            out(&format!("DEEP_SIDESLIP_{}_HEATER_FAILED", i + 1), f64::from(s.sideslip_heater_failed[i]));
            out(&format!("DEEP_SIDESLIP_{}_JAMMED", i + 1), f64::from(s.sideslip_jammed[i]));
        }
        out("DEEP_TAT_3_HEATER_FAILED", f64::from(s.tat3_heater_failed));
        out("DEEP_ADR_CAPT_FO_ALT_DIFF_FT", s.adr_capt_fo_alt_diff_ft);
        for i in 0..3 {
            out(&format!("DEEP_IR_{}_GYRO_DRIFT_DEG_HR", i + 1), s.ir_gyro_drift_deg_hr[i]);
        }
        out("DEEP_IR_CAPT_FO_PITCH_DIFF_DEG", s.ir_capt_fo_pitch_diff_deg);
        out("DEEP_IR_CAPT_FO_ROLL_DIFF_DEG", s.ir_capt_fo_roll_diff_deg);
        out("DEEP_IR_CAPT_FO_HDG_DIFF_DEG", s.ir_capt_fo_hdg_diff_deg);
        out("DEEP_IR_CAPT_FO_FPA_DIFF_DEG", s.ir_capt_fo_fpa_diff_deg);
        for (side, name) in ["CAPT", "FO"].iter().enumerate() {
            out(&format!("DEEP_KCCU_{name}_CCD_FAILED"), f64::from(s.kccu_part_failed[side][0]));
            out(&format!("DEEP_KCCU_{name}_KEYBOARD_FAILED"), f64::from(s.kccu_part_failed[side][1]));
        }
        for (i, du) in CDS_MONITORED_DUS.iter().enumerate() {
            out(&cds_monitor_var(du), f64::from(s.du_display_monitor_disagree[i]));
        }
        out("DEEP_BARO_REF_DISAGREE", f64::from(s.baro_ref_disagree));

        self.discrete.publish(out);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::integration::weather_truth::EnvironmentTruth;

    /// Cruise in cloud cold enough to ice, with every AC bus alive.
    fn icing_cruise() -> Truth {
        Truth {
            dt_s: 1.0 / 30.0,
            environment: EnvironmentTruth {
                sat_c: -10.0,
                leading_edge_c: -5.0,
                ambient_pressure_pa: 46_500.0,
                tas_ms: 200.0,
                precipitation_on_aircraft_ratio: 0.0,
                weather: None,
            },
            altitude_ft: 20_000.0,
            on_ground: false,
            ac_bus_volts: [115.0; 4],
            dc_bus_volts: [28.0; 2],
            engine_n1_frac: [0.85; 4],
            engine_running: [true; 4],
            ..Truth::default()
        }
    }

    fn vars(area: &LiveSensors) -> BTreeMap<String, f64> {
        let mut map = BTreeMap::new();
        area.publish(&mut |name, value| {
            map.insert(name.to_string(), value);
        });
        map
    }

    fn run(area: &mut LiveSensors, truth: &Truth, faults: &Faults, seconds: f64) -> BTreeMap<String, f64> {
        let ticks = (seconds / truth.dt_s).round().max(1.0) as u32;
        for _ in 0..ticks {
            area.tick(truth, faults);
        }
        vars(area)
    }

    #[test]
    fn every_live_sensor_resolved_a_real_failure_id() {
        let area = LiveSensors::new();
        let mut ids: Vec<u64> = Vec::new();
        for ch in &area.channels {
            ids.extend_from_slice(&ch.pitot_faults);
            ids.extend_from_slice(&ch.static_left_faults);
            ids.extend_from_slice(&ch.static_right_faults);
            ids.push(ch.averaging_line_fault);
        }
        for v in &area.vanes {
            ids.extend_from_slice(&v.faults);
        }
        for t in &area.tat {
            ids.extend_from_slice(&t.faults);
        }
        for d in &area.ice {
            ids.extend_from_slice(&d.faults);
        }
        for r in &area.ra {
            ids.extend_from_slice(&r.transceiver_faults);
            ids.push(r.tx_antenna_fault);
            ids.extend_from_slice(&r.rx_antenna_faults);
        }
        for g in &area.gps {
            ids.extend_from_slice(&g.receiver_faults);
            ids.extend_from_slice(&g.antenna_faults);
        }
        for engine in &area.n1_pickup_faults {
            for channel in engine {
                ids.extend_from_slice(channel);
            }
        }
        ids.extend_from_slice(&area.oat_faults);
        assert!(ids.iter().all(|&id| id != 0), "a live sensor has no catalogue failure behind it");
        let mut sorted = ids.clone();
        sorted.sort_unstable();
        sorted.dedup();
        assert_eq!(sorted.len(), ids.len(), "two live sensors resolved to the same failure id");
    }

    #[test]
    fn a_healthy_aircraft_agrees_with_itself_and_publishes_real_air_data() {
        let mut area = LiveSensors::new();
        let truth = icing_cruise();
        let published = run(&mut area, &truth, &Faults::default(), 30.0);

        assert_eq!(published["DEEP_ADR_VOTE_DISAGREE"], 0.0);
        // Mach 0.63 at -10 C: the ADRs must recover the free-stream Mach
        // from their own sensed pressures.
        let mach = truth.environment.mach();
        assert!((published["DEEP_ADR_1_MACH"] - mach).abs() < 0.02, "{} vs {mach}", published["DEEP_ADR_1_MACH"]);
        for i in 1..=3 {
            assert!(published[&format!("DEEP_ADR_{i}_CAS_MS")] > 100.0);
            assert_eq!(published[&format!("DEEP_GPS_{i}_VALID")], 1.0);
        }
        for (name, value) in &published {
            assert!(value.is_finite(), "{name} is not finite");
        }
    }

    /// `registry.rs`'s pitot heater failure: "ice accretes in icing
    /// conditions, eventually blocking the tube", and PROBE PITOT HEAT
    /// FAULT triggers on `DEEP_PITOT_n_HEATER_FAILED`.
    #[test]
    fn an_armed_pitot_heater_failure_raises_its_own_published_flag_and_ices_the_probe() {
        let mut area = LiveSensors::new();
        let index = FaultIndex::build();
        let id = index.id("34_nav.pitot_2", "heater_failure");
        assert_ne!(id, 0);

        let truth = Truth {
            environment: EnvironmentTruth {
                // Real supercooled cloud, so there is genuinely water to
                // freeze: the fault only matters because the icing is real.
                weather: None,
                ..icing_cruise().environment
            },
            ..icing_cruise()
        };
        let published = run(&mut area, &truth, &Faults::from_pairs([(id, 1.0)]), 10.0);

        assert_eq!(published["DEEP_PITOT_2_HEATER_FAILED"], 1.0, "the alert could never fire");
        assert_eq!(published["DEEP_PITOT_1_HEATER_FAILED"], 0.0, "it took out the wrong probe");
        assert_eq!(published["DEEP_PITOT_3_HEATER_FAILED"], 0.0);
        assert_eq!(published["DEEP_PITOT_4_HEATER_FAILED"], 0.0);
    }

    /// Losing an AC bus must fail that system's probe heaters and no
    /// others -- the reason the four systems are on four buses.
    #[test]
    fn losing_one_ac_bus_fails_only_that_systems_probe_heaters() {
        let mut area = LiveSensors::new();
        let mut truth = icing_cruise();
        truth.ac_bus_volts = [115.0, 0.0, 115.0, 115.0];
        let published = run(&mut area, &truth, &Faults::default(), 5.0);
        // An unpowered heater is not a *failed* heater: the current monitor
        // has no bus to measure against, so it does not annunciate.
        assert_eq!(published["DEEP_PITOT_2_HEATER_FAILED"], 0.0);
        assert!(published["DEEP_PITOT_2_ICE_KG"] >= published["DEEP_PITOT_1_ICE_KG"]);
    }

    /// `registry.rs`'s static port blockage: "sensed static pressure
    /// freezes; altitude/airspeed stop tracking reality", which is what
    /// NAV ADR DISAGREE exists to catch. Both ports of one system have to
    /// go, because the averaging line falls back to the healthy side --
    /// which is itself the registered behaviour of `average_pair`.
    #[test]
    fn blocking_one_systems_static_ports_makes_the_published_adr_voter_disagree() {
        let index = FaultIndex::build();
        let left = index.id("34_nav.static_3_1", "blocked");
        let right = index.id("34_nav.static_3_2", "blocked");

        let mut area = LiveSensors::new();
        let mut truth = icing_cruise();
        // Seat every port at the current altitude first, then climb: a
        // blocked port holds the pressure it was sealed at.
        run(&mut area, &truth, &Faults::default(), 20.0);
        let faults = Faults::from_pairs([(left, 1.0), (right, 1.0)]);
        area.tick(&truth, &faults);

        truth.environment.ambient_pressure_pa = 30_000.0;
        truth.altitude_ft = 30_000.0;
        let published = run(&mut area, &truth, &faults, 30.0);

        assert_eq!(published["DEEP_ADR_VOTE_DISAGREE"], 1.0, "the blocked system voted with the others");
        assert_eq!(published["DEEP_ADR_3_OUTLIER"], 1.0, "the wrong channel was flagged");
        assert!(
            published["DEEP_ADR_3_ALT_M"] < published["DEEP_ADR_1_ALT_M"] - 1_000.0,
            "the frozen port kept reporting the climb"
        );
    }

    /// `registry.rs`'s AoA vane seizure: "reported AoA frozen at last free
    /// angle", and NAV AOA DISAGREE triggers on `DEEP_AOA_n_JAMMED`.
    #[test]
    fn an_armed_aoa_vane_seizure_raises_its_own_published_jam_flag() {
        let index = FaultIndex::build();
        let id = index.id("34_nav.aoa_2", "mechanically_stuck");
        let mut area = LiveSensors::new();
        let published = run(&mut area, &icing_cruise(), &Faults::from_pairs([(id, 1.0)]), 5.0);
        assert_eq!(published["DEEP_AOA_2_JAMMED"], 1.0);
        assert_eq!(published["DEEP_AOA_1_JAMMED"], 0.0);
        assert_eq!(published["DEEP_AOA_3_JAMMED"], 0.0);
    }

    /// `registry.rs`'s radio altimeter transceiver fault: "no valid height
    /// output at all", which NAV RA 1 FAULT reads as
    /// `DEEP_RA_1_VALID == 0`.
    #[test]
    fn an_armed_ra_transceiver_fault_invalidates_only_that_unit() {
        let index = FaultIndex::build();
        let id = index.id("34_nav.ra_transceiver_1", "transceiver_fault");
        let mut area = LiveSensors::new();
        let truth = Truth { altitude_ft: 1_000.0, on_ground: false, ..icing_cruise() };

        let healthy = run(&mut area, &truth, &Faults::default(), 2.0);
        assert_eq!(healthy["DEEP_RA_1_VALID"], 1.0);
        assert_eq!(healthy["DEEP_RA_1_IN_RANGE"], 1.0);

        let mut failed_area = LiveSensors::new();
        let published = run(&mut failed_area, &truth, &Faults::from_pairs([(id, 1.0)]), 2.0);
        assert_eq!(published["DEEP_RA_1_VALID"], 0.0);
        assert_eq!(published["DEEP_RA_2_VALID"], 1.0);
        assert_eq!(published["DEEP_RA_3_VALID"], 1.0);
    }

    /// `registry.rs`'s GPS jamming: "effective satellites fall (weakest
    /// first) ... fix lost below 4", read by NAV GPS 1 FAULT as
    /// `DEEP_GPS_1_VALID == 0`.
    #[test]
    fn armed_gps_jamming_costs_that_receiver_its_fix() {
        let index = FaultIndex::build();
        let id = index.id("34_nav.gps_receiver_1", "jamming");
        let mut area = LiveSensors::new();
        let truth = icing_cruise();

        let mild = run(&mut area, &truth, &Faults::from_pairs([(id, 0.3)]), 2.0);
        assert_eq!(mild["DEEP_GPS_1_VALID"], 1.0, "mild jamming must not drop the fix outright");

        let mut area = LiveSensors::new();
        let heavy = run(&mut area, &truth, &Faults::from_pairs([(id, 0.9)]), 2.0);
        assert_eq!(heavy["DEEP_GPS_1_VALID"], 0.0);
        assert_eq!(heavy["DEEP_GPS_2_VALID"], 1.0);
        assert!(mild["DEEP_GPS_1_ERROR_M"] > 0.0);
    }

    /// `registry.rs`'s speed pickup open circuit: "no signal at any speed".
    #[test]
    fn an_armed_speed_pickup_open_circuit_invalidates_one_eec_channel_only() {
        let index = FaultIndex::build();
        let id = index.id("77_eng.speed_n1_3_b", "open_circuit");
        let mut area = LiveSensors::new();
        let published = run(&mut area, &icing_cruise(), &Faults::from_pairs([(id, 1.0)]), 1.0);
        assert_eq!(published["DEEP_ENG_3_N1_PICKUP_B_VALID"], 0.0);
        assert_eq!(published["DEEP_ENG_3_N1_PICKUP_A_VALID"], 1.0);
        assert_eq!(published["DEEP_ENG_4_N1_PICKUP_B_VALID"], 1.0);
        assert!((published["DEEP_ENG_3_N1_PICKUP_A_FRAC"] - 0.85).abs() < 1e-9);
    }

    /// `registry.rs`'s ice detector probe damage: "offsets the calibrated
    /// baseline, can false-trigger with no ice present".
    #[test]
    fn an_armed_ice_detector_probe_damage_false_triggers_in_clear_warm_air() {
        let index = FaultIndex::build();
        let id = index.id("30_ice.detector_1", "probe_damage_bias");
        let mut area = LiveSensors::new();
        // Warm, dry, no cloud: there is genuinely no ice anywhere.
        let truth = Truth {
            environment: EnvironmentTruth { sat_c: 20.0, ..Truth::default().environment },
            ac_bus_volts: [115.0; 4],
            ..Truth::default()
        };
        let healthy = run(&mut area, &truth, &Faults::default(), 5.0);
        assert_eq!(healthy["DEEP_ICE_DETECTOR_1"], 0.0);

        let mut damaged = LiveSensors::new();
        let published = run(&mut damaged, &truth, &Faults::from_pairs([(id, 1.0)]), 5.0);
        assert_eq!(published["DEEP_ICE_DETECTOR_1"], 1.0);
        assert_eq!(published["DEEP_ICE_DETECTOR_2"], 0.0);
        assert_eq!(published["DEEP_ICE_DETECTOR_1_ICE_KG"], 0.0, "it must be a false alarm, not real ice");
    }

    /// `registry.rs`'s standby OAT probe open circuit: "reading pegs to
    /// top of indicating range".
    #[test]
    fn an_armed_standby_oat_open_circuit_pegs_the_published_reading() {
        let index = FaultIndex::build();
        let id = index.id("34_nav.oat_standby", "open_circuit");
        let truth = icing_cruise();

        let mut healthy = LiveSensors::new();
        let published = run(&mut healthy, &truth, &Faults::default(), 1.0);
        assert!((published["DEEP_STANDBY_OAT_C"] - truth.environment.sat_c).abs() < 0.5);

        let mut open = LiveSensors::new();
        let published = run(&mut open, &truth, &Faults::from_pairs([(id, 1.0)]), 1.0);
        assert_eq!(published["DEEP_STANDBY_OAT_C"], OAT_RANGE_C.1);
    }

    #[test]
    fn every_variable_an_ecam_trigger_names_is_published() {
        let mut area = LiveSensors::new();
        area.tick(&icing_cruise(), &Faults::default());
        let published = vars(&area);
        let mut required = vec![
            "DEEP_ADR_VOTE_DISAGREE".to_string(),
            "DEEP_RA_1_VALID".to_string(),
            "DEEP_GPS_1_VALID".to_string(),
        ];
        required.extend((1..=4).map(|i| format!("DEEP_PITOT_{i}_HEATER_FAILED")));
        required.extend((1..=3).map(|i| format!("DEEP_AOA_{i}_JAMMED")));
        required.extend((1..=2).map(|i| format!("DEEP_TAT_{i}_HEATER_FAILED")));
        for name in required {
            assert!(published.contains_key(&name), "{name} is never published");
        }
        assert_eq!(area.name(), "sensors");
    }

    #[test]
    fn a_cold_dark_aircraft_on_the_ground_produces_nothing_but_finite_numbers() {
        let mut area = LiveSensors::new();
        let published = run(&mut area, &Truth::default(), &Faults::default(), 60.0);
        for (name, value) in &published {
            assert!(value.is_finite(), "{name} = {value}");
        }
        assert_eq!(published["DEEP_ADR_VOTE_DISAGREE"], 0.0);
        // On the ground at ISA the ADRs must read about zero airspeed and
        // about zero pressure altitude.
        assert!(published["DEEP_ADR_1_CAS_MS"] < 1.0);
        assert!(published["DEEP_ADR_1_ALT_M"].abs() < 5.0);
    }
}
