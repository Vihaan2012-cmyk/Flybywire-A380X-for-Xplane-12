//! FlyByWire's A380 failures, activated from X-Plane.
//!
//! In MSFS the EFB's failure orchestrator broadcasts the whole set of active
//! failure ids on the CommBus as `FBW_FAILURE_UPDATE` (a JSON array); the
//! systems glue maps each id it registered to its `FailureType`, ignores
//! unknown ids (systems_wasm failures.rs:15-55), and hands the set to
//! `Simulation::update_active_failures` just before the next systems tick
//! (systems_wasm lib.rs:250-256, 323-327). At start it asks for the current
//! set with `FBW_FAILURE_REQUEST` (lib.rs:218).
//!
//! The registrations are a380_systems_wasm lib.rs:86-428, copied id for id.
//!
//! X-Plane side:
//! - command `fbw/failure/<id>/toggle` flips one failure;
//! - int dataref `fbw/failure/<id>`, 0 or 1, readable and writable;
//! - int array dataref `fbw/failures/active`: the active ids in ascending
//!   order, zero padded to the number of registered failures. Writing it
//!   replaces the whole set, as FBW_FAILURE_UPDATE does (writes may cover a
//!   sub-range; the ids left in the array are the set).
//! - int dataref `fbw/failures/count`: how many are active.
//!
//! Every change is logged with FlyByWire's failure name (EFB definitions,
//! fbw-a380x/src/systems/failures/src/a380.ts:194-340).

#![allow(dead_code)]

use std::collections::BTreeSet;
use std::ffi::{c_char, c_int, c_void, CString};
use std::sync::Mutex;

use a380_systems::A380;
use systems::air_conditioning::{Channel, FdacId, OcsmId, VcmId};
use systems::failures::FailureType;
use systems::integrated_modular_avionics::core_processing_input_output_module::CpiomId;
use systems::shared::{
    AirbusElectricPumpId, AirbusEngineDrivenPumpId, ElectricalBusType, FireDetectionLoopID, FireDetectionZone,
    GearActuatorId, HydraulicColor, LgciuId, ProximityDetectorId,
};
use systems::simulation::Simulation;

use crate::xp::{CommandRef, DataRef, Xplm};

/// a380_systems_wasm lib.rs:86-428: (id, failure type), in registration
/// order.
/// The failure types in catalogue order: both sides of the systems bridge
/// (remote/) number failures by their place here.
pub fn catalogue_types() -> Vec<FailureType> {
    a380_failures().into_iter().map(|(_, t)| t).collect()
}

pub fn a380_failures() -> Vec<(u64, FailureType)> {
    use FailureType as F;
    let mut v = vec![
        (21_000, F::RapidDecompression),
        (21_001, F::CabinFan(1)),
        (21_002, F::CabinFan(2)),
        (21_003, F::CabinFan(3)),
        (21_004, F::CabinFan(4)),
        (21_005, F::HotAir(1)),
        (21_006, F::HotAir(2)),
        (21_007, F::FwdIsolValve),
        (21_008, F::FwdExtractFan),
        (21_009, F::BulkIsolValve),
        (21_010, F::BulkExtractFan),
        (21_011, F::CargoHeater),
        (21_012, F::Fdac(FdacId::One, Channel::ChannelOne)),
        (21_013, F::Fdac(FdacId::One, Channel::ChannelTwo)),
        (21_014, F::Fdac(FdacId::Two, Channel::ChannelOne)),
        (21_015, F::Fdac(FdacId::Two, Channel::ChannelTwo)),
        (21_016, F::Tadd(Channel::ChannelOne)),
        (21_017, F::Tadd(Channel::ChannelTwo)),
        (21_018, F::Vcm(VcmId::Fwd, Channel::ChannelOne)),
        (21_019, F::Vcm(VcmId::Fwd, Channel::ChannelTwo)),
        (21_020, F::Vcm(VcmId::Aft, Channel::ChannelOne)),
        (21_021, F::Vcm(VcmId::Aft, Channel::ChannelTwo)),
        (21_022, F::OcsmAutoPartition(OcsmId::One)),
        (21_023, F::OcsmAutoPartition(OcsmId::Two)),
        (21_024, F::OcsmAutoPartition(OcsmId::Three)),
        (21_025, F::OcsmAutoPartition(OcsmId::Four)),
        (21_026, F::Ocsm(OcsmId::One, Channel::ChannelOne)),
        (21_027, F::Ocsm(OcsmId::One, Channel::ChannelTwo)),
        (21_028, F::Ocsm(OcsmId::Two, Channel::ChannelOne)),
        (21_029, F::Ocsm(OcsmId::Two, Channel::ChannelTwo)),
        (21_030, F::Ocsm(OcsmId::Three, Channel::ChannelOne)),
        (21_031, F::Ocsm(OcsmId::Three, Channel::ChannelTwo)),
        (21_032, F::Ocsm(OcsmId::Four, Channel::ChannelOne)),
        (21_033, F::Ocsm(OcsmId::Four, Channel::ChannelTwo)),
    ];
    let cpioms = [CpiomId::B1, CpiomId::B2, CpiomId::B3, CpiomId::B4];
    for (k, c) in cpioms.iter().enumerate() {
        v.push((21_034 + k as u64, F::AgsApp(*c)));
    }
    for (k, c) in cpioms.iter().enumerate() {
        v.push((21_038 + k as u64, F::TcsApp(*c)));
    }
    for (k, c) in cpioms.iter().enumerate() {
        v.push((21_042 + k as u64, F::VcsApp(*c)));
    }
    for (k, c) in cpioms.iter().enumerate() {
        v.push((21_046 + k as u64, F::CpcsApp(*c)));
    }
    v.extend([
        (24_000, F::TransformerRectifier(1)),
        (24_001, F::TransformerRectifier(2)),
        (24_002, F::TransformerRectifier(3)),
        (24_003, F::TransformerRectifier(4)),
        (24_004, F::StaticInverter),
        (24_020, F::Generator(1)),
        (24_021, F::Generator(2)),
        (24_022, F::Generator(3)),
        (24_023, F::Generator(4)),
        (24_030, F::ApuGenerator(1)),
        (24_031, F::ApuGenerator(2)),
    ]);
    use ElectricalBusType as B;
    let buses = [
        B::AlternatingCurrent(1),
        B::AlternatingCurrent(2),
        B::AlternatingCurrent(3),
        B::AlternatingCurrent(4),
        B::AlternatingCurrentEssential,
        B::AlternatingCurrentEssentialShed,
        B::AlternatingCurrentNamed("247XP"),
        B::AlternatingCurrentGndFltService,
        B::DirectCurrent(1),
        B::DirectCurrent(2),
        B::DirectCurrentEssential,
        B::DirectCurrentNamed("247PP"),
        B::DirectCurrentNamed("309PP"),
        B::DirectCurrentHot(1),
        B::DirectCurrentHot(2),
        B::DirectCurrentHot(3),
        B::DirectCurrentHot(4),
        B::DirectCurrentGndFltService,
    ];
    for (k, b) in buses.into_iter().enumerate() {
        v.push((24_100 + k as u64, F::ElectricalBus(b)));
    }
    use FireDetectionZone as Z;
    v.extend([
        (26_001, F::SetOnFire(Z::Engine(1))),
        (26_002, F::SetOnFire(Z::Engine(2))),
        (26_003, F::SetOnFire(Z::Engine(3))),
        (26_004, F::SetOnFire(Z::Engine(4))),
        (26_005, F::SetOnFire(Z::Apu)),
        (26_006, F::SetOnFire(Z::Mlg)),
    ]);
    let zones = [Z::Engine(1), Z::Engine(2), Z::Engine(3), Z::Engine(4), Z::Apu, Z::Mlg];
    for (k, z) in zones.into_iter().enumerate() {
        v.push((26_007 + 2 * k as u64, F::FireDetectionLoop(FireDetectionLoopID::A, z)));
        v.push((26_008 + 2 * k as u64, F::FireDetectionLoop(FireDetectionLoopID::B, z)));
    }
    use HydraulicColor::{Green, Yellow};
    v.extend([
        (29_000, F::ReservoirLeak(Green)),
        (29_001, F::ReservoirLeak(Yellow)),
        (29_002, F::ReservoirAirLeak(Green)),
        (29_003, F::ReservoirAirLeak(Yellow)),
        (29_004, F::ReservoirReturnLeak(Green)),
        (29_005, F::ReservoirReturnLeak(Yellow)),
        (29_006, F::ElecPumpOverheat(AirbusElectricPumpId::GreenA)),
        (29_007, F::ElecPumpOverheat(AirbusElectricPumpId::GreenB)),
        (29_008, F::ElecPumpOverheat(AirbusElectricPumpId::YellowA)),
        (29_009, F::ElecPumpOverheat(AirbusElectricPumpId::YellowB)),
    ]);
    use AirbusEngineDrivenPumpId as E;
    for (k, p) in [E::Edp1a, E::Edp1b, E::Edp2a, E::Edp2b, E::Edp3a, E::Edp3b, E::Edp4a, E::Edp4b].into_iter().enumerate() {
        v.push((29_010 + k as u64, F::EnginePumpOverheat(p)));
    }
    v.extend([
        (32_000, F::LgciuPowerSupply(LgciuId::Lgciu1)),
        (32_001, F::LgciuPowerSupply(LgciuId::Lgciu2)),
        (32_002, F::LgciuInternalError(LgciuId::Lgciu1)),
        (32_003, F::LgciuInternalError(LgciuId::Lgciu2)),
    ]);
    use ProximityDetectorId as P;
    let sensors = [
        P::UplockGearNose1,
        P::DownlockGearNose2,
        P::UplockGearRight1,
        P::DownlockGearRight2,
        P::UplockGearLeft2,
        P::DownlockGearLeft1,
        P::UplockDoorNose1,
        P::DownlockDoorNose2,
        P::UplockDoorRight2,
        P::DownlockDoorRight1,
        P::UplockDoorLeft2,
        P::DownlockDoorLeft1,
    ];
    for (k, s) in sensors.into_iter().enumerate() {
        v.push((32_004 + k as u64, F::GearProxSensorDamage(s)));
    }
    use GearActuatorId as G;
    for (k, a) in [G::GearNose, G::GearLeft, G::GearRight, G::GearDoorNose, G::GearDoorLeft, G::GearDoorRight]
        .into_iter()
        .enumerate()
    {
        v.push((32_020 + k as u64, F::GearActuatorJammed(a)));
    }
    for n in 1..=3usize {
        v.push((34_000 + n as u64 - 1, F::RadioAltimeter(n)));
    }
    for n in 1..=3usize {
        v.push((34_010 + n as u64 - 1, F::RadioAntennaInterrupted(n)));
    }
    for n in 1..=3usize {
        v.push((34_020 + n as u64 - 1, F::RadioAntennaDirectCoupling(n)));
    }
    v
}

/// The ids `fbw_a380`'s FailuresConsumer registers for the C++ computers
/// (FailuresConsumer.cpp:27-39, FailureList.h), in the same order, with
/// FBW's EFB names (a380.ts:249-251, 311-318). Consumed at
/// `failuresConsumer.isActive` in FlyByWireInterface.cpp:2249 (Rollout,
/// through extra_backend_fcdc.rs), 2374 (Fcu1/Fcu2, through prim.rs),
/// 1711 (Prim1..3, through prim.rs), 2200 (Sec1..3, through prim.rs) and
/// 2299 (Fcdc1/Fcdc2, through extra_backend_fcdc.rs).
pub const COMPUTER_FAILURES: &[(u64, &str)] = &[
    (22_001, "ROLLOUT"),
    (22_002, "FCU 1"),
    (22_003, "FCU 2"),
    (27_000, "PRIM 1"),
    (27_001, "PRIM 2"),
    (27_002, "PRIM 3"),
    (27_003, "SEC 1"),
    (27_004, "SEC 2"),
    (27_005, "SEC 3"),
    (27_006, "FCDC 1"),
    (27_007, "FCDC 2"),
];

/// FlyByWire's EFB name for a failure id (a380.ts:194-340), for the log.
/// The gear proximity sensors (32_004..) and actuators (32_020..), in
/// `a380_failures`' own order.
fn gear_failure_name(id: u64) -> Option<String> {
    const SENSORS: [&str; 12] = [
        "Nose gear uplock proximity sensor 1",
        "Nose gear downlock proximity sensor 2",
        "Right gear uplock proximity sensor 1",
        "Right gear downlock proximity sensor 2",
        "Left gear uplock proximity sensor 2",
        "Left gear downlock proximity sensor 1",
        "Nose gear door uplock proximity sensor 1",
        "Nose gear door downlock proximity sensor 2",
        "Right gear door uplock proximity sensor 2",
        "Right gear door downlock proximity sensor 1",
        "Left gear door uplock proximity sensor 2",
        "Left gear door downlock proximity sensor 1",
    ];
    const ACTUATORS: [&str; 6] = [
        "Nose gear actuator jammed",
        "Left gear actuator jammed",
        "Right gear actuator jammed",
        "Nose gear door actuator jammed",
        "Left gear door actuator jammed",
        "Right gear door actuator jammed",
    ];
    match id {
        32_004..=32_015 => SENSORS.get((id - 32_004) as usize),
        32_020..=32_025 => ACTUATORS.get((id - 32_020) as usize),
        _ => None,
    }
    .map(|s| (*s).to_owned())
}

pub fn failure_name(id: u64) -> String {
    if let Some((_, name)) = COMPUTER_FAILURES.iter().find(|(i, _)| *i == id) {
        return (*name).to_owned();
    }
    let fixed = match id {
        21_000 => "Rapid Decompression",
        21_005 => "Hot Air Valve 1",
        21_006 => "Hot Air Valve 2",
        21_007 => "Foward Cargo Isolation Valve",
        21_008 => "Foward Cargo Extraction Fan",
        21_009 => "Bulk Cargo Isolation Valve",
        21_010 => "Bulk Cargo Extraction Fan",
        21_011 => "Bulk Cargo Heater",
        24_002 => "TR ESS",
        24_003 => "TR APU",
        24_004 => "Static Inverter",
        24_104 => "AC EMER",
        24_105 => "AC ESS",
        24_106 => "AC 247XP",
        24_107 => "AC GND FLT SRV",
        24_110 => "DC ESS",
        24_111 => "DC 247PP",
        24_112 => "DC 309PP",
        24_115 => "DC HOT ESS",
        24_116 => "DC HOT APU",
        24_117 => "DC GND FLT SRV",
        26_005 => "Fire - APU",
        26_006 => "Fire - Main Landing Gear Bay",
        26_015 => "APU Loop A",
        26_016 => "APU Loop B",
        26_017 => "Main Landing Gear Bay Loop A",
        26_018 => "Main Landing Gear Bay Loop B",
        29_000 => "Green reservoir leak",
        29_001 => "Yellow reservoir leak",
        29_002 => "Green reservoir air leak",
        29_003 => "Yellow reservoir air leak",
        29_004 => "Green reservoir return leak",
        29_005 => "Yellow reservoir return leak",
        29_006 => "Green A elec pump overheat",
        29_007 => "Green B elec pump overheat",
        29_008 => "Yellow A elec pump overheat",
        29_009 => "Yellow B elec pump overheat",
        _ => "",
    };
    if !fixed.is_empty() {
        return fixed.to_owned();
    }
    let k = |base: u64| id - base;
    match id {
        21_001..=21_004 => format!("Cabin Fan {}", k(21_000)),
        21_012..=21_015 => format!("FDAC {} Channel {}", k(21_012) / 2 + 1, k(21_012) % 2 + 1),
        21_016..=21_017 => format!("TADD Channel {}", k(21_015)),
        21_018..=21_019 => format!("Forward VCM Channel {}", k(21_017)),
        21_020..=21_021 => format!("Aft VCM Channel {}", k(21_019)),
        21_022..=21_025 => format!("Automatic Partition of OCSM {}", k(21_021)),
        21_026..=21_033 => format!("OCSM {} Channel {}", k(21_026) / 2 + 1, k(21_026) % 2 + 1),
        21_034..=21_037 => format!("AGS Application in CPIOM B{}", k(21_033)),
        21_038..=21_041 => format!("TCS Application in CPIOM B{}", k(21_037)),
        21_042..=21_045 => format!("VCS Application in CPIOM B{}", k(21_041)),
        21_046..=21_049 => format!("CPCS Application in CPIOM B{}", k(21_045)),
        24_000..=24_001 => format!("TR {}", k(23_999)),
        24_020..=24_023 => format!("Generator {}", k(24_019)),
        24_030..=24_031 => format!("APU Generator {}", k(24_029)),
        24_100..=24_103 => format!("AC {}", k(24_099)),
        24_108..=24_109 => format!("DC {}", k(24_107)),
        24_113..=24_114 => format!("DC HOT {}", k(24_112)),
        26_001..=26_004 => format!("Fire - Engine {}", k(26_000)),
        26_007..=26_014 => format!("Engine {} Loop {}", k(26_007) / 2 + 1, if k(26_007) % 2 == 0 { "A" } else { "B" }),
        29_010..=29_017 => {
            format!("Engine {} pump {} overheat", k(29_010) / 2 + 1, if k(29_010) % 2 == 0 { "A" } else { "B" })
        }
        32_000..=32_001 => format!("LGCIU {} Power supply", k(31_999)),
        32_002..=32_003 => format!("LGCIU {} Internal error", k(32_001)),
        34_000..=34_002 => format!("RA SYS {}", ["A", "B", "C"][k(34_000) as usize]),
        34_010..=34_012 => format!("RA SYS {} Interrupted", ["A", "B", "C"][k(34_010) as usize]),
        34_020..=34_022 => format!("RA SYS {} Direct Coupling", ["A", "B", "C"][k(34_020) as usize]),
        // The EFB lists only one proximity sensor and no jammed actuators;
        // the systems' registration names them.
        // Gear proximity sensors and actuators, named from FlyByWire's own
        // ids ("UplockGearNose1" -> "Nose gear uplock proximity sensor 1").
        32_004..=32_025 => gear_failure_name(id).unwrap_or_else(|| format!("failure {id}")),
        _ => format!("failure {id}"),
    }
}

// ---------------------------------------------------------------------------
// hyperrealism.md physics workstream 6 (failures, damage, MEL, persistence):
// the catalogue beyond FlyByWire's own 146 systems failures and 11 computer
// faults above. `a380_failures()`/`COMPUTER_FAILURES` mirror ids FlyByWire's
// own MSFS glue already assigns; nothing there was free to extend (every
// `FailureType` variant the A380 systems actually consume is already
// registered — `TrimAirOverheat`, `Acsc`, `BrakeHydraulicLeak` and the other
// unused variants in `systems::failures::FailureType` have no A380
// `SimulationElement` that reads them, so registering them would toggle a
// dataref with no physical effect, which the brief forbids). Everything
// below is new: either wired for real right now (this plugin's own model,
// or X-Plane's own native failure datarefs for effects no custom system
// claims), or a documented hook — a Var this plugin drives to 1.0/0.0 that
// the owning workstream's model should read, listed by owner in
// `docs/physics/failures.md` and the workstream 6 report. Ids are chosen so
// `id / 1000` is the failure's real ATA chapter, matching how the existing
// catalogue and the Study Failures page group by chapter.
pub mod extra {
    use crate::mel::MelCategory;

    /// Which model should end up consuming a hook variable. Purely
    /// documentation (the report and `docs/physics/failures.md` group by
    /// it); this module does not import the other workstreams' code.
    #[derive(Clone, Copy, PartialEq, Eq, Debug)]
    pub enum Owner {
        Engine,
        Electrical,
        Air,
        Fluids,
        Adirs,
    }

    impl Owner {
        pub fn label(self) -> &'static str {
            match self {
                Owner::Engine => "engine",
                Owner::Electrical => "electrical",
                Owner::Air => "air",
                Owner::Fluids => "fluids",
                Owner::Adirs => "adirs",
            }
        }
    }

    #[derive(Clone)]
    pub enum Effect {
        /// Not consumed by any model yet: this plugin drives `var` to
        /// 1.0/0.0 with the failure's active state; `owner`'s model should
        /// read it and apply the physical consequence.
        Hook { var: &'static str, owner: Owner },
        /// Wired now, through an X-Plane native failure dataref
        /// (`sim/operation/failures/rel_*`) for a structural/physical
        /// effect no custom system claims (tyre and brake force are X-Plane
        /// core flight-model physics regardless of the avionics driving the
        /// aircraft). `0` is working, `1` is failed, X-Plane's own
        /// convention for every `failure_enum` dataref.
        NativeXplane { dataref: &'static str },
        /// Wired now, entirely inside this workstream's own model
        /// (`physics/damage.rs`): a wear/exceedance state with no further
        /// physical model needed (e.g. driving the MEL/dispatch state).
        Local,
    }

    pub struct ExtraFailure {
        pub id: u64,
        pub ata: u16,
        pub name: String,
        pub description: String,
        pub effect: Effect,
        pub mel: Option<MelCategory>,
    }

    fn f(id: u64, ata: u16, name: impl Into<String>, description: impl Into<String>, effect: Effect, mel: Option<MelCategory>) -> ExtraFailure {
        ExtraFailure { id, ata, name: name.into(), description: description.into(), effect, mel }
    }

    /// The 18 electrical buses `a380_failures()` already lists (24_100..),
    /// in the same order, named the way `ElectricalBusType`'s `Display`
    /// impl does (a380_systems shared/mod.rs) — reused here only as plain
    /// labels for the new bus-short ids, not the enum itself.
    const BUS_LABELS: [&str; 18] = [
        "AC1", "AC2", "AC3", "AC4", "AC_ESS", "AC_ESS_SHED", "AC_247XP", "AC_GND_FLT_SVC", "DC1", "DC2", "DC_ESS",
        "DC_247PP", "DC_309PP", "DC_HOT1", "DC_HOT2", "DC_HOT3", "DC_HOT4", "DC_GND_FLT_SVC",
    ];

    /// ATA24: a short circuit on a bus, for the electrical workstream's
    /// breaker model to trip (Kirchhoff's-law overcurrent, not a plain
    /// open/dead bus like `ElectricalBus` above).
    fn electrical(v: &mut Vec<ExtraFailure>) {
        for (k, label) in BUS_LABELS.iter().enumerate() {
            v.push(f(
                24_200 + k as u64,
                24,
                format!("{label} bus short circuit"),
                format!("A short circuit on the {label} bus, drawing fault current until its breaker trips."),
                Effect::Hook { var: "FAIL_BUS_SHORT_HOOK", owner: Owner::Electrical },
                Some(MelCategory::C),
            ));
        }
    }

    /// ATA28: fuel system failures FlyByWire's own `FailureType` has no
    /// equivalent for (its fuel model runs through this plugin's
    /// `fuel_network.rs`/`fuel_transfer.rs`, not `systems::failures`).
    fn fuel(v: &mut Vec<ExtraFailure>) {
        let items: &[(&str, &str)] = &[
            ("Tank 1 feed pump A", "A main feed pump fails to prime its tank's feed line."),
            ("Tank 1 feed pump B", "A main feed pump fails to prime its tank's feed line."),
            ("Tank 2 feed pump A", "A main feed pump fails to prime its tank's feed line."),
            ("Tank 2 feed pump B", "A main feed pump fails to prime its tank's feed line."),
            ("Tank 3 feed pump A", "A main feed pump fails to prime its tank's feed line."),
            ("Tank 3 feed pump B", "A main feed pump fails to prime its tank's feed line."),
            ("Tank 4 feed pump A", "A main feed pump fails to prime its tank's feed line."),
            ("Tank 4 feed pump B", "A main feed pump fails to prime its tank's feed line."),
            ("Trim tank transfer pump", "The trim tank pump stops transferring fuel to the centre of gravity target."),
            ("Cross-feed valve 1-2", "The cross-feed valve between engines 1 and 2 sticks; it no longer opens or closes."),
            ("Cross-feed valve 3-4", "The cross-feed valve between engines 3 and 4 sticks; it no longer opens or closes."),
            ("Fuel jettison valve", "A wing jettison valve sticks open or closed."),
        ];
        for (k, (name, desc)) in items.iter().enumerate() {
            v.push(f(
                28_000 + k as u64,
                28,
                *name,
                *desc,
                Effect::Hook { var: "FAIL_FUEL_HOOK", owner: Owner::Fluids },
                Some(MelCategory::C),
            ));
        }
        for &(id, name, pump, _) in FUEL_ELEMENTS {
            let desc = if pump {
                format!("The {} fails: its head, and with it its flow, is lost.", name.to_ascii_lowercase())
            } else {
                format!("The {} seizes where it is; it no longer opens or closes.", name.to_ascii_lowercase())
            };
            v.push(f(id, 28, name, desc, Effect::Hook { var: "FAIL_FUEL_HOOK", owner: Owner::Fluids }, Some(MelCategory::C)));
        }
    }

    /// ATA28: every further pump and valve of the A380's own fuel system
    /// (flight_model.cfg's element names, the network `fuel_network.rs`
    /// solves): (id, name, is a pump, the elements it is). A failed pump
    /// loses its head and flow; a failed valve seizes where it is. Several
    /// elements for one id are the one physical valve's actuators.
    pub const FUEL_ELEMENTS: &[(u64, &str, bool, &[&str])] = &[
        (28_100, "Left outer tank transfer pump", true, &["LeftOuterTankPump"]),
        (28_101, "Right outer tank transfer pump", true, &["RightOuterTankPump"]),
        (28_102, "Left mid tank forward pump", true, &["LeftMidTankPumpFwd"]),
        (28_103, "Left mid tank aft pump", true, &["LeftMidTankPumpAft"]),
        (28_104, "Right mid tank forward pump", true, &["RightMidTankPumpFwd"]),
        (28_105, "Right mid tank aft pump", true, &["RightMidTankPumpAft"]),
        (28_106, "Left inner tank forward pump", true, &["LeftInnerTankPumpFwd"]),
        (28_107, "Left inner tank aft pump", true, &["LeftInnerTankPumpAft"]),
        (28_108, "Right inner tank forward pump", true, &["RightInnerTankPumpFwd"]),
        (28_109, "Right inner tank aft pump", true, &["RightInnerTankPumpAft"]),
        (28_110, "APU fuel pump", true, &["APUFeedPump"]),
        (28_120, "Left auxiliary refuel valve", false, &["GalleryAuxRefuelValveLeft"]),
        (28_121, "Right auxiliary refuel valve", false, &["GalleryAuxRefuelValveRight"]),
        (28_122, "Transfer/defuel valve", false, &["TransferDefuelValve"]),
        (28_123, "Feed tank 1 aft inlet valve", false, &["FeedTank1AftTransferValve1", "FeedTank1AftTransferValve2"]),
        (28_124, "Feed tank 2 aft inlet valve", false, &["FeedTank2AftTransferValve1", "FeedTank2AftTransferValve2"]),
        (28_125, "Feed tank 3 aft inlet valve", false, &["FeedTank3AftTransferValve1", "FeedTank3AftTransferValve2"]),
        (28_126, "Feed tank 4 aft inlet valve", false, &["FeedTank4AftTransferValve1", "FeedTank4AftTransferValve2"]),
        (28_127, "Feed tank 1 forward inlet valve", false, &["FeedTank1FwdTransferValve1", "FeedTank1FwdTransferValve2"]),
        (28_128, "Feed tank 2 forward inlet valve", false, &["FeedTank2FwdTransferValve1_1", "FeedTank2FwdTransferValve1_2", "FeedTank2FwdTransferValve2_1", "FeedTank2FwdTransferValve2_2"]),
        (28_129, "Feed tank 3 forward inlet valve", false, &["FeedTank3FwdTransferValve1_1", "FeedTank3FwdTransferValve1_2", "FeedTank3FwdTransferValve2_1", "FeedTank3FwdTransferValve2_2"]),
        (28_130, "Feed tank 4 forward inlet valve", false, &["FeedTank4FwdTransferValve1", "FeedTank4FwdTransferValve2"]),
        (28_131, "Left outer tank aft inlet valve", false, &["LeftOuterAftTransferValve1", "LeftOuterAftTransferValve2"]),
        (28_132, "Right outer tank aft inlet valve", false, &["RightOuterAftTransferValve1", "RightOuterAftTransferValve2"]),
        (28_133, "Left outer tank forward inlet valve", false, &["LeftOuterFwdTransferValve"]),
        (28_134, "Right outer tank forward inlet valve", false, &["RightOuterFwdTransferValve"]),
        (28_135, "Left mid tank aft inlet valve", false, &["LeftMidAftTransferValve1", "LeftMidAftTransferValve2"]),
        (28_136, "Right mid tank aft inlet valve", false, &["RightMidAftTransferValve1", "RightMidAftTransferValve2"]),
        (28_137, "Left mid tank forward inlet valve", false, &["LeftMidFwdTransferValve"]),
        (28_138, "Right mid tank forward inlet valve", false, &["RightMidFwdTransferValve"]),
        (28_139, "Left inner tank aft inlet valve", false, &["LeftInnerAftTransferValve1", "LeftInnerAftTransferValve2"]),
        (28_140, "Right inner tank aft inlet valve", false, &["RightInnerAftTransferValve1", "RightInnerAftTransferValve2"]),
        (28_141, "Left inner tank forward inlet valve", false, &["LeftInnerFwdTransferValve"]),
        (28_142, "Right inner tank forward inlet valve", false, &["RightInnerFwdTransferValve"]),
        (28_143, "Trim tank inlet valve", false, &["TrimTankInletValve1", "TrimTankInletValve2"]),
        (28_144, "Left outer tank emergency transfer valve", false, &["LeftOuterEmerTransferValve"]),
        (28_145, "Right outer tank emergency transfer valve", false, &["RightOuterEmerTransferValve"]),
    ];

    /// ATA29: hydraulic degradation beyond the binary reservoir/pump-overheat
    /// failures already registered.
    fn hydraulics(v: &mut Vec<ExtraFailure>) {
        let items: &[(&str, &str)] = &[
            ("Green circuit filter clogging", "The green circuit's return filter clogs, raising case drain back-pressure."),
            ("Yellow circuit filter clogging", "The yellow circuit's return filter clogs, raising case drain back-pressure."),
            ("Power transfer unit fault", "The PTU fails to transfer flow between the green and yellow circuits."),
            ("Green local electric pump degraded", "A green local electric pump's output falls below its rated flow."),
            ("Yellow local electric pump degraded", "A yellow local electric pump's output falls below its rated flow."),
            ("Hydraulic fluid contamination", "Particulate contamination accelerates wear in whichever pump is running."),
        ];
        for (k, (name, desc)) in items.iter().enumerate() {
            v.push(f(
                29_100 + k as u64,
                29,
                *name,
                *desc,
                Effect::Hook { var: "FAIL_HYDRAULIC_HOOK", owner: Owner::Fluids },
                Some(MelCategory::C),
            ));
        }
    }

    /// ATA32: tyre bursts and brake wear-out, wired now through X-Plane's
    /// own native failure datarefs (tyre/brake force are core flight-model
    /// physics, independent of FlyByWire's systems) — see `damage.rs` for
    /// the brake-energy model that arms these.
    fn gear(v: &mut Vec<ExtraFailure>) {
        let tyres: &[&str] = &["Nose gear", "Left wing gear", "Right wing gear", "Left body gear", "Right body gear"];
        for (k, leg) in tyres.iter().enumerate() {
            v.push(f(
                32_100 + k as u64,
                32,
                format!("{leg} tyre burst"),
                format!("A {} tyre bursts from fuse-plug overheat or overspeed/overweight touchdown.", leg.to_lowercase()),
                Effect::NativeXplane { dataref: ["sim/operation/failures/rel_tire1", "sim/operation/failures/rel_tire2", "sim/operation/failures/rel_tire3", "sim/operation/failures/rel_tire4", "sim/operation/failures/rel_tire5"][k] },
                None,
            ));
        }
        let brakes: &[&str] = &["Left wing gear", "Right wing gear", "Left body gear", "Right body gear"];
        for (k, leg) in brakes.iter().enumerate() {
            v.push(f(
                32_110 + k as u64,
                32,
                format!("{leg} brake wear-out"),
                format!("The {} brake stack wears past its life limit from accumulated brake energy.", leg.to_lowercase()),
                Effect::NativeXplane { dataref: if k < 2 { "sim/operation/failures/rel_lbrakes" } else { "sim/operation/failures/rel_rbrakes" } },
                Some(MelCategory::D),
            ));
        }
    }

    /// ATA30: probe heater open circuits, one per ADIRU. This is the real
    /// physical cause behind a pitot/static/AOA icing event in the air: the
    /// heating element itself opens (not the same fault as losing bus power,
    /// which the AUTO probe-heat logic in `physics/adirs.rs` already models
    /// separately from `powered`). `physics/adirs.rs`'s `update_adr` reads
    /// this id directly (`failures::active_ids()`, the same pattern
    /// `physics/damage.rs` uses) and forces that unit's `heat_on` false
    /// regardless of bus power or the airborne/engine-running AUTO logic, so
    /// the probe ices over in icing conditions exactly as an unheated probe
    /// does, and the ADR then blocks/freezes and feeds bad air data
    /// downstream -- no separate "failed" flag, the existing icing physics
    /// does the rest.
    fn ice_protection(v: &mut Vec<ExtraFailure>) {
        for adiru in 1..=3u64 {
            v.push(f(
                30_000 + (adiru - 1),
                30,
                format!("ADIRU {adiru} probe heater open circuit"),
                format!("ADIRU {adiru}'s pitot/static/AOA/TAT heating element opens; the probes no longer heat even when the AUTO probe-heat logic calls for it, so they ice over in icing conditions like an unpowered probe."),
                Effect::Hook { var: "FAIL_PROBE_HEATER_HOOK", owner: Owner::Adirs },
                Some(MelCategory::C),
            ));
        }
    }

    /// ATA34: air data/inertial sensor failures for the ADIRS workstream
    /// (three independent ADIRUs, each with its own probes).
    fn adirs(v: &mut Vec<ExtraFailure>) {
        let sensors: &[&str] = &["Pitot probe", "Static port", "AOA vane"];
        for (si, sensor) in sensors.iter().enumerate() {
            for adiru in 1..=3u64 {
                v.push(f(
                    34_100 + si as u64 * 3 + (adiru - 1),
                    34,
                    format!("ADIRU {adiru} {sensor} fault"),
                    format!("ADIRU {adiru}'s {} reads a frozen or biased value.", sensor.to_lowercase()),
                    Effect::Hook { var: "FAIL_ADIRU_SENSOR_HOOK", owner: Owner::Adirs },
                    Some(MelCategory::C),
                ));
            }
        }
        for adiru in 1..=3u64 {
            v.push(f(
                34_109 + adiru - 1,
                34,
                format!("ADIRU {adiru} internal fault"),
                format!("ADIRU {adiru}'s inertial platform fails; its outputs are no longer valid."),
                Effect::Hook { var: "FAIL_ADIRU_INTERNAL_HOOK", owner: Owner::Adirs },
                Some(MelCategory::C),
            ));
        }
    }

    /// ATA36: pneumatic/bleed duct failures for the air workstream.
    fn pneumatic(v: &mut Vec<ExtraFailure>) {
        for n in 1..=4u64 {
            v.push(f(
                36_000 + (n - 1),
                36,
                format!("Engine {n} bleed duct leak"),
                format!("A leak in engine {n}'s bleed duct bypasses the duct's own mass flow to ambient."),
                Effect::Hook { var: "FAIL_BLEED_DUCT_LEAK_HOOK", owner: Owner::Air },
                Some(MelCategory::C),
            ));
        }
        for n in 1..=4u64 {
            v.push(f(
                36_004 + (n - 1),
                36,
                format!("Engine {n} precooler fault"),
                format!("Engine {n}'s bleed precooler heat exchanger loses effectiveness."),
                Effect::Hook { var: "FAIL_PRECOOLER_HOOK", owner: Owner::Air },
                Some(MelCategory::C),
            ));
        }
        for n in 1..=4u64 {
            v.push(f(
                36_008 + (n - 1),
                36,
                format!("Engine {n} HP bleed valve stuck"),
                format!("Engine {n}'s HP bleed valve seizes where it is; it no longer opens or closes."),
                Effect::Local,
                Some(MelCategory::C),
            ));
        }
        for n in 1..=4u64 {
            v.push(f(
                36_012 + (n - 1),
                36,
                format!("Engine {n} bleed valve stuck"),
                format!("Engine {n}'s bleed pressure regulating (shut-off) valve seizes where it is; it no longer opens, closes or regulates."),
                Effect::Local,
                Some(MelCategory::C),
            ));
        }
        for (k, side) in ["Left", "Centre", "Right"].into_iter().enumerate() {
            v.push(f(
                36_016 + k as u64,
                36,
                format!("{side} cross bleed valve stuck"),
                format!("The {} cross bleed valve seizes where it is; it no longer opens or closes.", side.to_ascii_lowercase()),
                Effect::Local,
                Some(MelCategory::C),
            ));
        }
        for pack in 1..=2u64 {
            for valve in 1..=2u64 {
                v.push(f(
                    21_050 + (pack - 1) * 2 + (valve - 1),
                    21,
                    format!("Pack {pack} flow valve {valve} stuck"),
                    format!("Pack {pack}'s flow control valve {valve} seizes where it is; it no longer opens, closes or regulates the pack's flow."),
                    Effect::Local,
                    Some(MelCategory::C),
                ));
            }
        }
    }

    /// The pneumatic valves a failure seizes (FlyByWire's A380 pneumatic
    /// model, patched: `ValveSeizure`, reading `PNEU_VALVE_FAILED:n` every
    /// tick): (failure id, valve number).
    pub const PNEUMATIC_VALVES: &[(u64, usize)] = &[
        (36_008, 1),
        (36_009, 2),
        (36_010, 3),
        (36_011, 4),
        (36_012, 5),
        (36_013, 6),
        (36_014, 7),
        (36_015, 8),
        (36_016, 9),
        (36_017, 10),
        (36_018, 11),
        (49_003, 12),
        (21_050, 13),
        (21_051, 14),
        (21_052, 15),
        (21_053, 16),
    ];

    /// Every pneumatic valve's seizure, from its failure's magnitude, each
    /// tick before the systems run.
    pub fn write_pneumatic_valves<W: systems::simulation::VariableRegistry + systems::simulation::SimulatorReaderWriter>(vars: &mut W) {
        for &(id, n) in PNEUMATIC_VALVES {
            let ident = vars.get(format!("PNEU_VALVE_FAILED:{n}"));
            vars.write(&ident, super::magnitude(id));
        }
    }

    /// ATA49: APU failures. The EGT exceedance is wired now (it reads the
    /// APU's own live `APU_EGT`/`APU_EGT_WARNING` variables, already
    /// published by the ported systems — see `damage.rs`); the rest are
    /// hooks. `ApuGenerator(1|2)` faults already exist in `a380_failures()`.
    fn apu(v: &mut Vec<ExtraFailure>) {
        v.push(f(
            49_000,
            49,
            "APU EGT overtemperature damage",
            "The APU has run with EGT above its own warning threshold long enough to damage the turbine.",
            Effect::Local,
            None,
        ));
        v.push(f(
            49_001,
            49,
            "APU fuel control fault",
            "The APU's fuel control unit no longer meters fuel correctly.",
            Effect::Hook { var: "FAIL_APU_FUEL_CONTROL_HOOK", owner: Owner::Fluids },
            Some(MelCategory::C),
        ));
        v.push(f(
            49_002,
            49,
            "APU starter fault",
            "The APU starter/starter-generator fails to motor the APU during start.",
            Effect::Hook { var: "FAIL_APU_STARTER_HOOK", owner: Owner::Electrical },
            Some(MelCategory::C),
        ));
        v.push(f(
            49_003,
            49,
            "APU bleed valve stuck",
            "The APU bleed air valve seizes where it is; it no longer opens or closes.",
            Effect::Local,
            Some(MelCategory::C),
        ));
        v.push(f(
            49_004,
            49,
            "APU oil leak",
            "A seal or line leaks APU oil overboard; quantity falls until the APU is running starved of oil, the same physical route `physics/damage.rs` already arms the per-engine 72_000 bearing-wear failure from (see 79_004).",
            Effect::Local,
            Some(MelCategory::D),
        ));
    }

    /// ATA72-80: per-engine component failures. Every one is a hook for the
    /// engine workstream (`physics/engine.rs`); `damage.rs` is what arms the
    /// EGT/creep-triggered ones (bearing wear, compressor stall, turbine
    /// blade damage) from accumulated exceedance, the same way a real
    /// engine's on-condition maintenance would.
    fn engines(v: &mut Vec<ExtraFailure>) {
        struct Item {
            ata: u16,
            base: u64,
            name: &'static str,
            description: &'static str,
            mel: Option<MelCategory>,
        }
        let items = [
            Item { ata: 72, base: 72_000, name: "bearing wear", description: "A main shaft bearing wears, raising vibration and running clearances.", mel: None },
            Item { ata: 72, base: 72_004, name: "compressor stall", description: "The compressor stalls: a transient reversed-flow event with an EGT spike and thrust loss.", mel: None },
            Item { ata: 72, base: 72_008, name: "turbine blade damage", description: "Turbine blade damage from sustained overtemperature reduces turbine efficiency.", mel: None },
            // Exotic, as combinations of the engine model's own physical
            // quantities (engine_commands.rs `EXOTIC`), never scripted
            // outcomes: what follows (flame-out, run-down, overspeed,
            // oil heat) is whatever the gas path and spool physics do.
            Item { ata: 72, base: 72_012, name: "HP compressor destruction", description: "HP compressor blades are lost: the core can no longer compress, its flow capacity collapses and the imbalance loads the bearings. Compression lost, the flame goes out and the core runs down.", mel: None },
            Item { ata: 72, base: 72_016, name: "HP turbine blade release", description: "HP turbine blades are released: the turbine extracts far less work and the imbalance loads the bearings.", mel: None },
            Item { ata: 72, base: 72_020, name: "main bearing seizure", description: "A main shaft bearing seizes: friction beyond what the turbine can overcome.", mel: None },
            Item { ata: 73, base: 73_000, name: "FADEC channel fault", description: "One EEC channel fails; the engine reverts to its remaining channel.", mel: Some(MelCategory::C) },
            Item { ata: 73, base: 73_004, name: "fuel metering valve stuck", description: "The fuel metering valve sticks, decoupling commanded fuel flow from actual flow.", mel: Some(MelCategory::B) },
            Item { ata: 74, base: 74_000, name: "igniter fault", description: "An igniter no longer sparks reliably, lengthening light-up time and risking a hung start.", mel: Some(MelCategory::C) },
            Item { ata: 76, base: 76_000, name: "throttle resolver fault", description: "The thrust lever angle resolver reads a frozen or biased angle.", mel: Some(MelCategory::C) },
            Item { ata: 77, base: 77_000, name: "EGT probe fault", description: "The EGT thermocouple reads a frozen or biased temperature.", mel: Some(MelCategory::C) },
            Item { ata: 77, base: 77_004, name: "N1 tachometer fault", description: "The N1 tachometer reads a frozen or biased speed.", mel: Some(MelCategory::C) },
            Item { ata: 77, base: 77_008, name: "oil pressure sensor fault", description: "The oil pressure transducer reads a frozen or biased pressure.", mel: Some(MelCategory::C) },
            Item { ata: 77, base: 77_012, name: "fuel flow sensor fault", description: "The fuel flow transducer reads a frozen or biased flow.", mel: Some(MelCategory::C) },
            Item { ata: 78, base: 78_000, name: "thrust reverser lock fault", description: "The reverser fails to lock stowed, or fails to deploy on command.", mel: Some(MelCategory::C) },
            Item { ata: 79, base: 79_000, name: "oil pump fault", description: "The lubrication pump's output falls below the rate the bearings need.", mel: Some(MelCategory::B) },
            Item { ata: 79, base: 79_004, name: "oil leak", description: "A seal or line leaks oil overboard; quantity falls, then pressure, then the starved bearings wear and can seize.", mel: Some(MelCategory::B) },
            Item { ata: 80, base: 80_000, name: "starter valve stuck", description: "The pneumatic starter air valve sticks open or closed.", mel: Some(MelCategory::C) },
        ];
        for item in items {
            for n in 1..=4u64 {
                v.push(f(
                    item.base + (n - 1),
                    item.ata,
                    format!("Engine {n} {}", item.name),
                    format!("Engine {n}: {}", item.description),
                    Effect::Hook { var: "FAIL_ENGINE_COMPONENT_HOOK", owner: Owner::Engine },
                    item.mel,
                ));
            }
        }
    }

    /// Every new failure this workstream adds, beyond FlyByWire's own 146
    /// systems failures and 11 computer faults. Ids never collide with
    /// those (checked in the tests below) because every new range sits in
    /// an ATA chapter the original catalogue never used (24_2xx/28/29_1xx/
    /// 32_1xx/34_1xx/36/49/72-80), or is enumerated with `24_200.. >
    /// 24_117` past the original chapter's highest id.
    /// Flight-envelope/handling exceedances (`damage.rs` arms every one of
    /// these): a maintenance-log event with no further physical model
    /// needed here, since real operations already treat an overspeed, hard
    /// landing, tailstrike or overweight landing as a required inspection
    /// rather than an immediately different flight-model behaviour. None
    /// are MEL-eligible: all require an inspection before further flight.
    fn exceedances(v: &mut Vec<ExtraFailure>) {
        v.push(f(
            27_100,
            27,
            "Flap overspeed damage",
            "The flaps were extended above their placard speed for the selected CONF (FlyByWire flight_model.cfg's own per-CONF limit).",
            Effect::Local,
            None,
        ));
        v.push(f(
            32_120,
            32,
            "Gear overspeed damage",
            "The landing gear was extended, or left extended, above its placard speed (FlyByWire flight_model.cfg's own gear speed limit).",
            Effect::Local,
            None,
        ));
        v.push(f(
            34_120,
            34,
            "VMO/MMO overspeed damage",
            "The aircraft exceeded its maximum operating speed or Mach number (FlyByWire flight_model.cfg's own VMO/MMO redlines).",
            Effect::Local,
            None,
        ));
        v.push(f(
            32_121,
            32,
            "Hard landing",
            "The aircraft touched down at a sink rate or normal load factor beyond the generic transport-category hard-landing inspection threshold.",
            Effect::Local,
            None,
        ));
        v.push(f(
            32_122,
            32,
            "Tailstrike",
            "The aircraft's pitch at touchdown exceeded the aft-body ground-clearance angle derived from FlyByWire's own gear/fuselage geometry.",
            Effect::Local,
            None,
        ));
        v.push(f(
            32_123,
            32,
            "Overweight landing",
            "The aircraft landed above its maximum landing weight (Airbus's published A380 aircraft characteristics document).",
            Effect::Local,
            None,
        ));
    }

    pub fn extra_failures() -> Vec<ExtraFailure> {
        let mut v = Vec::new();
        electrical(&mut v);
        fuel(&mut v);
        hydraulics(&mut v);
        gear(&mut v);
        ice_protection(&mut v);
        adirs(&mut v);
        pneumatic(&mut v);
        apu(&mut v);
        engines(&mut v);
        exceedances(&mut v);
        v
    }

    /// The name shown in the Study Failures page and the log, matching
    /// `failure_name`'s style for the original catalogue.
    pub fn extra_failure_name(id: u64) -> Option<String> {
        extra_failures().into_iter().find(|x| x.id == id).map(|x| x.name)
    }

    /// Drive `id`'s effect (a hook variable, or an X-Plane native failure
    /// dataref) to match `active`. A no-op for ids `extra_failures()` does
    /// not know (the wired 157 apply through `Simulation::update_active_
    /// failures` instead) or for [`Effect::Local`] ones (`damage.rs` reads
    /// `failures::active_ids()` itself).
    pub fn drive<W: systems::simulation::VariableRegistry + systems::simulation::SimulatorReaderWriter>(
        vars: &mut W,
        xplm: Option<&crate::xp::Xplm>,
        id: u64,
        active: bool,
    ) {
        let Some(item) = extra_failures().into_iter().find(|x| x.id == id) else { return };
        match item.effect {
            Effect::Hook { var, .. } => {
                let ident = vars.get(var.to_owned());
                vars.write(&ident, active as i32 as f64);
            }
            Effect::NativeXplane { dataref } => {
                if let Some(xplm) = xplm {
                    if let Some(d) = xplm.find(dataref) {
                        xplm.set_i(d, active as i32);
                    }
                }
            }
            Effect::Local => {}
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;
        use std::collections::BTreeSet;

        #[test]
        fn every_extra_id_is_unique_and_outside_the_original_catalogue() {
            let extra = extra_failures();
            let ids: BTreeSet<u64> = extra.iter().map(|x| x.id).collect();
            assert_eq!(ids.len(), extra.len(), "no duplicate extra ids");
            let original: BTreeSet<u64> =
                super::super::a380_failures().iter().map(|x| x.0).chain(super::super::COMPUTER_FAILURES.iter().map(|x| x.0)).collect();
            assert!(ids.is_disjoint(&original), "extra ids never collide with the original catalogue");
        }

        #[test]
        fn the_catalogue_totals_within_the_requested_band() {
            let total = super::super::a380_failures().len() + super::super::COMPUTER_FAILURES.len() + extra_failures().len();
            // 250-300 was the first target; the catalogue is meant to keep
            // growing, so this guards the floor only.
            assert!(total >= 250, "total failure count {total} fell below 250");
        }

        #[test]
        fn every_extra_failure_has_a_non_empty_description() {
            for x in extra_failures() {
                assert!(!x.description.is_empty(), "{} has no description", x.id);
                assert!(x.ata > 0, "{} has no ATA chapter", x.id);
            }
        }

        #[test]
        fn tyre_and_brake_failures_map_to_distinct_native_datarefs() {
            let extra = extra_failures();
            let native: Vec<&str> = extra
                .iter()
                .filter_map(|x| match &x.effect {
                    Effect::NativeXplane { dataref } => Some(*dataref),
                    _ => None,
                })
                .collect();
            assert!(native.contains(&"sim/operation/failures/rel_tire1"));
            assert!(native.contains(&"sim/operation/failures/rel_lbrakes"));
        }
    }
}

/// The MEL category for an id from either catalogue, if it is deferrable.
/// The original 157 catalogue's classic single-unit-inoperative items
/// (Airbus MMEL structure: one of several redundant generators, TRs or
/// hydraulic pumps may be deferred) are marked generically here since this
/// plugin has no cited A380 MMEL item text for them; the new catalogue's
/// items carry their own category (`extra::ExtraFailure::mel`).
pub fn mel_item(id: u64) -> Option<crate::mel::MelCategory> {
    use crate::mel::MelCategory as C;
    if let Some(item) = extra::extra_failures().into_iter().find(|x| x.id == id) {
        return item.mel;
    }
    match id {
        // One generator inoperative: Airbus MMEL 24-2x structure (generic;
        // not a cited item number).
        24_020..=24_023 | 24_030..=24_031 => Some(C::C),
        // One transformer-rectifier or the static inverter inoperative.
        24_000..=24_004 => Some(C::C),
        // One engine-driven or electric hydraulic pump inoperative.
        29_010..=29_017 | 29_006..=29_009 => Some(C::C),
        _ => None,
    }
}

/// Every registered id's display name, from whichever catalogue has it.
/// Every catalogued failure id: FlyByWire's, the computers', and the extra
/// catalogue's.
pub fn all_ids() -> Vec<u64> {
    let mut ids: Vec<u64> = a380_failures().into_iter().map(|(id, _)| id).collect();
    ids.extend(COMPUTER_FAILURES.iter().map(|(id, _)| *id));
    ids.extend(extra::extra_failures().into_iter().map(|x| x.id));
    ids.sort_unstable();
    ids.dedup();
    ids
}

pub fn any_failure_name(id: u64) -> String {
    extra::extra_failure_name(id).unwrap_or_else(|| failure_name(id))
}

/// The physical cause behind `id`, for the Study/Failures tab (brief item
/// 5: "cause description"). The extra catalogue (`extra::extra_failures`)
/// already carries a real cause sentence per item; the original 157-id
/// catalogue's `FailureType` variants are themselves named after the real
/// LRU/component (`TransformerRectifier(1)`, `ReservoirLeak(Green)`, ...),
/// so their cause is the component named by `failure_name` failing/leaking/
/// sticking -- described generically rather than inventing per-id prose
/// this plugin cannot source.
pub fn cause_description(id: u64) -> String {
    if let Some(item) = extra::extra_failures().into_iter().find(|x| x.id == id) {
        return item.description;
    }
    if let Some((_, name)) = COMPUTER_FAILURES.iter().find(|(i, _)| *i == id) {
        return format!("Flight control computer {name} loses power or fails its internal self-test and drops off the bus.");
    }
    format!("{} fails or leaves its normal range; the owning system loses that unit's redundancy.", failure_name(id))
}

/// The component(s) `id` acts on physically, for the Study/Failures tab
/// (brief item 5: "affected components"). One entry for the original/
/// computer catalogues (the LRU the id itself names); the extra catalogue's
/// hook-driven items list both the failed part and the hook var the owning
/// system reads it from, so a reader can see which workstream's model
/// consumes it.
pub fn affected_components(id: u64) -> Vec<String> {
    if let Some(item) = extra::extra_failures().into_iter().find(|x| x.id == id) {
        let mut c = vec![item.name];
        if let extra::Effect::Hook { var, owner } = item.effect {
            c.push(format!("{var} (read by the {} model)", owner.label()));
        }
        return c;
    }
    vec![failure_name(id)]
}

/// How `id` can become active, for the Study/Failures tab (brief item 5:
/// "trigger condition"). Three sources exist today: a manual arm from the
/// Study/Failures panel (every id), the MTBF random-failure engine
/// (`random_failures.rs`'s curated component list), and `physics/damage.rs`'s
/// own wear/exceedance arming (touchdown, brake energy, EGT creep, VMO/MMO,
/// flap/gear overspeed) -- the same set `random_failures.rs`'s
/// `damage_armed` test excludes from the MTBF draw, kept in sync here.
pub fn trigger_condition(id: u64) -> &'static str {
    let damage_armed = (32_100..=32_123).contains(&id) || (72_000..=72_011).contains(&id) || id == 34_120 || id == 49_000;
    if damage_armed {
        return "manual arm, or wear/exceedance-armed by physics/damage.rs (touchdown, brake energy, EGT creep, overspeed)";
    }
    let mtbf_eligible = matches!(id, 24_020 | 24_021 | 29_010 | 29_012)
        || extra::extra_failures().iter().any(|x| x.id == id && matches!(x.ata, 24 | 28 | 29 | 34 | 36 | 49 | 73 | 74 | 76 | 78 | 79 | 80) && id != 34_120);
    if mtbf_eligible {
        "manual arm, or MTBF random failure when the Study panel's random-failures engine is enabled"
    } else {
        "manual arm only (Study/Failures panel)"
    }
}

/// The active set as X-Plane last left it, shared with the dataref and
/// command callbacks.
struct State {
    registered: Vec<u64>,
    active: BTreeSet<u64>,
    /// Continuous magnitude in `0.0..=1.0` for every id in `active`, e.g.
    /// "a valve stuck at 37% open" or "33% pump displacement loss" --
    /// docs/physics/failures.md's continuous-magnitude contract. An id can
    /// be in `active` with no entry here (the binary `set_active`/`toggle`/
    /// `replace` entry points, and any id `set_magnitude` has not touched
    /// since it last cleared): [`magnitude`] reads that as `1.0`, "fully
    /// failed", the same meaning binary activation always had, so every
    /// existing binary consumer (`is_active`/`active_ids`) keeps working
    /// unchanged. Never holds a `0.0` or out-of-range entry: `set_magnitude`
    /// removes the id from both maps instead of storing `0.0`.
    magnitudes: std::collections::BTreeMap<u64, f64>,
    /// Set whenever the active set may have changed.
    dirty: bool,
    /// Each catalogued failure's physical loss as the components system
    /// combines it (`components.rs`: the armed failure plus any direct
    /// setting on its component), when above zero. A failure acts at the
    /// larger of this and its armed magnitude, so a component degraded in
    /// the Components tab drives the same physics as the failure itself.
    component_levels: std::collections::BTreeMap<u64, f64>,
}

static STATE: Mutex<State> = Mutex::new(State {
    registered: Vec::new(),
    active: BTreeSet::new(),
    magnitudes: std::collections::BTreeMap::new(),
    dirty: true,
    component_levels: std::collections::BTreeMap::new(),
});

impl State {
    /// Armed failures plus every failure whose component is degraded.
    fn effective(&self) -> BTreeSet<u64> {
        let mut set = self.active.clone();
        set.extend(self.component_levels.keys().copied());
        set
    }
}

/// The components system's combined loss per failure (`components.rs`,
/// after every recompute). Marks the set changed when it differs.
pub fn set_component_levels(levels: std::collections::BTreeMap<u64, f64>) {
    with_state(|s| {
        if s.component_levels != levels {
            let keys_changed = !s.component_levels.keys().eq(levels.keys());
            s.component_levels = levels;
            s.dirty |= keys_changed;
        }
    });
}

/// A failure's armed magnitude only (0.0 when not armed): what the
/// components system combines, before its own direct settings.
pub fn armed_magnitude(id: u64) -> f64 {
    with_state(|s| if s.active.contains(&id) { s.magnitudes.get(&id).copied().unwrap_or(1.0) } else { 0.0 }).unwrap_or(0.0)
}

fn with_state<R>(f: impl FnOnce(&mut State) -> R) -> Option<R> {
    STATE.lock().ok().map(|mut s| f(&mut s))
}

/// Activate or clear one failure by id, at full magnitude (`1.0`) when
/// activating. Unknown ids are ignored, as FailureIdVisitor ignores them
/// (failures.rs:49-51). Re-affirming an id that is already active at a
/// fractional magnitude (`set_active(id, true)` on one `set_magnitude`
/// already set) leaves that magnitude alone rather than snapping it to
/// `1.0` -- only a fresh activation defaults to full.
pub fn set_active(id: u64, active: bool) {
    with_state(|s| {
        if !s.registered.contains(&id) {
            return;
        }
        let changed = if active {
            let inserted = s.active.insert(id);
            s.magnitudes.entry(id).or_insert(1.0);
            inserted
        } else {
            s.magnitudes.remove(&id);
            s.active.remove(&id)
        };
        s.dirty |= changed;
    });
}

pub fn toggle(id: u64) {
    let now = with_state(|s| s.active.contains(&id)).unwrap_or(false);
    set_active(id, !now);
}

/// Replace the whole set (FBW_FAILURE_UPDATE). An id that stays active
/// across the replace keeps whatever magnitude it had; a newly-added id
/// starts at full magnitude, matching legacy binary semantics for a source
/// (FBW's own EFB broadcast) that only ever sends whole ids, never
/// fractions.
pub fn replace(ids: impl IntoIterator<Item = u64>) {
    with_state(|s| {
        let new: BTreeSet<u64> = ids.into_iter().filter(|id| s.registered.contains(id)).collect();
        if new != s.active {
            s.magnitudes.retain(|id, _| new.contains(id));
            for &id in &new {
                s.magnitudes.entry(id).or_insert(1.0);
            }
            s.active = new;
            s.dirty = true;
        }
    });
}

pub fn active_ids() -> Vec<u64> {
    with_state(|s| s.effective().into_iter().collect()).unwrap_or_default()
}

/// Test-isolation helper (see `scenarios::reset_global_state`): clears every
/// active failure and its magnitude, leaving `registered` (populated once by
/// `Failures::new()`, and generally harmless to leave set between tests)
/// alone. Marks the state dirty so a subsequent `Failures::apply` sees the
/// now-empty set as a real change and logs the clear, matching how a fresh
/// `Failures::new()` behaves.
#[cfg(any(test, feature = "test-support"))]
pub fn reset_for_tests() {
    with_state(|s| {
        s.active.clear();
        s.magnitudes.clear();
        s.component_levels.clear();
        s.dirty = true;
    });
}

/// Whether one id (original, computer, or the `extra` catalogue) is
/// currently active. Used by the owning physics model to read a specific
/// `extra` failure's state directly by id, instead of the shared
/// `Effect::Hook` variable an item's whole group writes (which cannot tell
/// two ids in the same group apart) -- e.g. `fuel.rs`/`breakers.rs` gating a
/// real pressure/current effect on one particular pump's own failure id.
/// Active means `magnitude(id) > 0.0`; every existing consumer of this
/// binary reading keeps working unchanged under the continuous model.
pub fn is_active(id: u64) -> bool {
    with_state(|s| s.active.contains(&id) || s.component_levels.contains_key(&id)).unwrap_or(false)
}

/// Activate `id` at a continuous magnitude in `0.0..=1.0` -- a physical
/// perturbation fraction, e.g. "stuck at 37% open" or "33% displacement
/// loss" (docs/physics/failures.md), never a severity tier. `magnitude <=
/// 0.0` clears the failure exactly like `set_active(id, false)`; any other
/// value is clamped into `0.0..=1.0` and the failure becomes (or stays)
/// active at that fraction. Unknown ids are ignored, same as `set_active`.
pub fn set_magnitude(id: u64, magnitude: f64) {
    with_state(|s| {
        if !s.registered.contains(&id) {
            return;
        }
        if !(magnitude > 0.0) {
            let changed = s.active.remove(&id);
            s.magnitudes.remove(&id);
            s.dirty |= changed;
            return;
        }
        let clamped = magnitude.clamp(0.0, 1.0);
        let newly_active = s.active.insert(id);
        let prev = s.magnitudes.insert(id, clamped);
        s.dirty |= newly_active || prev != Some(clamped);
    });
}

/// `id`'s continuous magnitude in `0.0..=1.0`: `0.0` if not active or
/// unregistered, else the fraction `set_magnitude` last set, or `1.0` if it
/// was activated (or last re-affirmed) through the binary `set_active`/
/// `toggle`/`replace` path -- "fully failed", matching what binary
/// activation always meant. The owning physics model should treat this as
/// the physical perturbation fraction itself (e.g. `1.0 - magnitude` of a
/// pump's rated displacement remains), never as a severity number scaling a
/// scripted effect (docs/physics/failures.md).
pub fn magnitude(id: u64) -> f64 {
    with_state(|s| {
        let armed = if s.active.contains(&id) { s.magnitudes.get(&id).copied().unwrap_or(1.0) } else { 0.0 };
        armed.max(s.component_levels.get(&id).copied().unwrap_or(0.0))
    })
    .unwrap_or(0.0)
}

/// Every catalogued failure as a damageable component (`components.rs`):
/// one `loss` parameter, the physical fraction of that part lost, which is
/// what the failure's consumers already read as its magnitude. The engine
/// failures act on the engine's own components instead
/// (`engine_commands::is_engine_component_failure`). Idempotent.
pub fn register_component_catalogue() {
    static SPECS: std::sync::OnceLock<Vec<(u64, String, crate::components::ParamSpec)>> = std::sync::OnceLock::new();
    let specs = SPECS.get_or_init(|| {
        let slug = |s: &str| {
            let mut out = String::new();
            for c in s.chars() {
                if c.is_ascii_alphanumeric() {
                    out.push(c.to_ascii_lowercase());
                } else if !out.ends_with('_') && !out.is_empty() {
                    out.push('_');
                }
            }
            out.trim_end_matches('_').to_owned()
        };
        all_ids()
            .into_iter()
            .filter(|&id| !crate::engine_commands::is_engine_component_failure(id))
            .map(|id| {
                let ata = id / 1000;
                let chapter = crate::study::chapter_name(ata);
                let system = if chapter == "Other" {
                    format!("ata_{ata}")
                } else {
                    slug(chapter.trim_start_matches(|c: char| c.is_ascii_digit() || c == ' '))
                };
                let component = format!("{ata:02}_{system}.{}", slug(&any_failure_name(id)));
                let fbw_binary = !extra::extra_failures().iter().any(|x| x.id == id);
                let mut description = cause_description(id);
                if fbw_binary {
                    description.push_str(" (FlyByWire's own failure: any loss above zero fails it completely)");
                }
                let spec = crate::components::ParamSpec {
                    name: "loss",
                    unit: "fraction lost",
                    healthy: 0.0,
                    min: 0.0,
                    max: 1.0,
                    combine: crate::components::Combine::CompoundLoss,
                    description: Box::leak(description.into_boxed_str()),
                };
                (id, component, spec)
            })
            .collect()
    });
    crate::components::register_failure_mirrors(specs);
}

/// Every active id with its magnitude, for persistence
/// (`persistence::AirframeState::active_failure_magnitudes`) and the Study
/// panel's JSON. Ids with no fractional entry read as `1.0` (see
/// [`magnitude`]), so this always has one entry per `active_ids()` id.
pub fn active_magnitudes() -> std::collections::BTreeMap<u64, f64> {
    with_state(|s| s.active.iter().map(|&id| (id, s.magnitudes.get(&id).copied().unwrap_or(1.0))).collect()).unwrap_or_default()
}

/// Restore a previously-saved id->magnitude map (persistence load), setting
/// each id active at its saved magnitude. Ids the current catalogue no
/// longer registers are silently dropped, same as `replace`.
pub fn restore_magnitudes(saved: impl IntoIterator<Item = (u64, f64)>) {
    for (id, m) in saved {
        set_magnitude(id, m);
    }
}

/// Test-only global reset (this module's state is process-wide, as
/// X-Plane's own callbacks need it): clears every active id and magnitude
/// without re-registering the catalogue, so a test can call this at SETUP
/// instead of leaking activations into the next test. Does *not* touch
/// `registered` -- `Failures::new()` remains the only thing that changes the
/// known id set.
pub fn reset_all() {
    with_state(|s| {
        s.active.clear();
        s.magnitudes.clear();
        s.component_levels.clear();
        s.dirty = true;
    });
}

/// The failures the systems know, with what was last handed to them, plus
/// the C++ computers' ids (registered the same way, for `fbw/failure/<id>`,
/// but never handed to `Simulation::update_active_failures`: prim.rs and
/// extra_backend_fcdc.rs read them straight from [`active_ids`], as
/// FailuresConsumer::isActive does).
pub struct Failures {
    types: Vec<(u64, FailureType)>,
    computer_ids: Vec<u64>,
    applied: BTreeSet<u64>,
}

impl Default for Failures {
    fn default() -> Self {
        Self::new()
    }
}

impl Failures {
    pub fn new() -> Self {
        let types = a380_failures();
        let computer_ids: Vec<u64> = COMPUTER_FAILURES.iter().map(|(id, _)| *id).collect();
        let extra_ids: Vec<u64> = extra::extra_failures().iter().map(|x| x.id).collect();
        with_state(|s| {
            s.registered =
                types.iter().map(|(id, _)| *id).chain(computer_ids.iter().copied()).chain(extra_ids.iter().copied()).collect();
            s.active.clear();
            s.magnitudes.clear();
            // MSFS asks for the current set at start (FBW_FAILURE_REQUEST)
            // and the systems receive it before their first tick.
            s.dirty = true;
        });
        Self { types, computer_ids, applied: BTreeSet::new() }
    }

    pub fn ids(&self) -> impl Iterator<Item = u64> + '_ {
        self.types
            .iter()
            .map(|(id, _)| *id)
            .chain(self.computer_ids.iter().copied())
            .chain(extra::extra_failures().into_iter().map(|x| x.id))
    }

    /// Hand a changed set to the systems, before their tick, and drive the
    /// workstream-6 catalogue's own effects (hooks and X-Plane native
    /// failure datarefs) for whatever in it changed. Returns the log lines
    /// for what changed.
    pub fn apply<W: systems::simulation::VariableRegistry + systems::simulation::SimulatorReaderWriter>(
        &mut self,
        simulation: &mut impl crate::remote::FailureSink,
        vars: &mut W,
        xplm: Option<&crate::xp::Xplm>,
    ) -> Vec<String> {
        extra::write_pneumatic_valves(vars);
        let Some(active) = with_state(|s| std::mem::take(&mut s.dirty).then(|| s.effective())).flatten() else {
            return Vec::new();
        };
        let mut log = Vec::new();
        for id in active.difference(&self.applied) {
            log.push(format!("failure {id} ({}) activated", any_failure_name(*id)));
            extra::drive(vars, xplm, *id, true);
        }
        for id in self.applied.difference(&active) {
            log.push(format!("failure {id} ({}) cleared", any_failure_name(*id)));
            extra::drive(vars, xplm, *id, false);
        }
        let set = self
            .types
            .iter()
            .filter(|(id, _)| active.contains(id))
            .map(|(_, t)| *t)
            .collect();
        simulation.update_active_failures(set);
        self.applied = active;
        log
    }
}

// ---------------------------------------------------------------------------
// X-Plane: commands and datarefs.
// ---------------------------------------------------------------------------

extern "system" {
    fn LoadLibraryA(name: *const c_char) -> *mut c_void;
    fn GetProcAddress(module: *mut c_void, name: *const c_char) -> *mut c_void;
}

fn xplm_symbol(name: &str) -> Option<*mut c_void> {
    unsafe {
        let module_name = CString::new("XPLM_64.dll").ok()?;
        let module = LoadLibraryA(module_name.as_ptr());
        if module.is_null() {
            return None;
        }
        let name = CString::new(name).ok()?;
        let p = GetProcAddress(module, name.as_ptr());
        (!p.is_null()).then_some(p)
    }
}

type GetI = unsafe extern "C" fn(*mut c_void) -> c_int;
type SetI = unsafe extern "C" fn(*mut c_void, c_int);
type GetVi = unsafe extern "C" fn(*mut c_void, *mut c_int, c_int, c_int) -> c_int;
type SetVi = unsafe extern "C" fn(*mut c_void, *mut c_int, c_int, c_int);
type RegisterAccessor = unsafe extern "C" fn(
    *const c_char,
    c_int,
    c_int,
    Option<GetI>,
    Option<SetI>,
    *const c_void,
    *const c_void,
    *const c_void,
    *const c_void,
    Option<GetVi>,
    Option<SetVi>,
    *const c_void,
    *const c_void,
    *const c_void,
    *const c_void,
    *mut c_void,
    *mut c_void,
) -> DataRef;
type UnregisterAccessor = unsafe extern "C" fn(DataRef);

const TYPE_INT: c_int = 1;
const TYPE_INT_ARRAY: c_int = 16;

unsafe extern "C" fn get_one(refcon: *mut c_void) -> c_int {
    let id = refcon as u64;
    with_state(|s| s.active.contains(&id) as c_int).unwrap_or(0)
}

unsafe extern "C" fn set_one(refcon: *mut c_void, value: c_int) {
    set_active(refcon as u64, value != 0);
}

unsafe extern "C" fn get_count(_: *mut c_void) -> c_int {
    with_state(|s| s.active.len() as c_int).unwrap_or(0)
}

/// The active ids, zero padded to the registered count.
fn active_array() -> Vec<c_int> {
    with_state(|s| {
        let mut out: Vec<c_int> = s.active.iter().map(|&id| id as c_int).collect();
        out.resize(s.registered.len(), 0);
        out
    })
    .unwrap_or_default()
}

unsafe extern "C" fn get_array(_: *mut c_void, out: *mut c_int, offset: c_int, max: c_int) -> c_int {
    let values = active_array();
    if out.is_null() {
        return values.len() as c_int;
    }
    let offset = offset.max(0) as usize;
    let n = (max.max(0) as usize).min(values.len().saturating_sub(offset));
    std::ptr::copy_nonoverlapping(values[offset..].as_ptr(), out, n);
    n as c_int
}

unsafe extern "C" fn set_array(_: *mut c_void, values: *mut c_int, offset: c_int, count: c_int) {
    if values.is_null() || count <= 0 {
        return;
    }
    let mut array = active_array();
    let offset = offset.max(0) as usize;
    let written = std::slice::from_raw_parts(values, count as usize);
    for (i, v) in written.iter().enumerate() {
        if let Some(slot) = array.get_mut(offset + i) {
            *slot = *v;
        }
    }
    replace(array.into_iter().filter(|&v| v > 0).map(|v| v as u64));
}

unsafe extern "C" fn on_toggle(_: CommandRef, phase: c_int, refcon: *mut c_void) -> c_int {
    if phase == 0 {
        toggle(refcon as u64);
    }
    1
}

/// The commands and datarefs, unregistered when dropped.
pub struct XplaneFailures {
    xplm: &'static Xplm,
    commands: Vec<(CommandRef, u64)>,
    datarefs: Vec<DataRef>,
    _names: Vec<CString>,
}

impl XplaneFailures {
    pub fn register(xplm: &'static Xplm, ids: &[u64]) -> Self {
        let mut me = Self { xplm, commands: Vec::new(), datarefs: Vec::new(), _names: Vec::new() };
        for &id in ids {
            let name = format!("fbw/failure/{id}/toggle");
            if let Some(c) = xplm.create_command(&name, &format!("FlyByWire failure {id}: {}", failure_name(id))) {
                xplm.register_command_handler(c, on_toggle, id as *mut c_void);
                me.commands.push((c, id));
            }
        }
        let Some(register) = xplm_symbol("XPLMRegisterDataAccessor") else { return me };
        let register = unsafe { std::mem::transmute::<*mut c_void, RegisterAccessor>(register) };
        let null = std::ptr::null();
        let add = |me: &mut Self, name: String, types: c_int, gi: Option<GetI>, si: Option<SetI>, gv: Option<GetVi>, sv: Option<SetVi>, refcon: u64| {
            let Ok(c) = CString::new(name) else { return };
            let r = unsafe {
                register(
                    c.as_ptr(),
                    types,
                    1,
                    gi,
                    si,
                    null,
                    null,
                    null,
                    null,
                    gv,
                    sv,
                    null,
                    null,
                    null,
                    null,
                    refcon as *mut c_void,
                    refcon as *mut c_void,
                )
            };
            if !r.is_null() {
                me.datarefs.push(r);
            }
            me._names.push(c);
        };
        for &id in ids {
            add(&mut me, format!("fbw/failure/{id}"), TYPE_INT, Some(get_one), Some(set_one), None, None, id);
        }
        add(&mut me, "fbw/failures/active".into(), TYPE_INT_ARRAY, None, None, Some(get_array), Some(set_array), 0);
        add(&mut me, "fbw/failures/count".into(), TYPE_INT, Some(get_count), None, None, None, 0);
        me
    }
}

impl Drop for XplaneFailures {
    fn drop(&mut self) {
        for (c, id) in self.commands.drain(..) {
            self.xplm.unregister_command_handler(c, on_toggle, id as *mut c_void);
        }
        if let Some(f) = xplm_symbol("XPLMUnregisterDataAccessor") {
            let f = unsafe { std::mem::transmute::<*mut c_void, UnregisterAccessor>(f) };
            for d in self.datarefs.drain(..) {
                unsafe { f(d) };
            }
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::aspects::test_vars::TestVars;
    use std::time::Duration;
    use systems::simulation::StartState;

    /// The failure state is process-wide, as X-Plane's callbacks need it;
    /// tests touching it take turns.
    pub static SERIAL: Mutex<()> = Mutex::new(());

    #[test]
    fn every_registration_is_unique_and_counted() {
        let f = a380_failures();
        assert_eq!(f.len(), 146);
        let ids: BTreeSet<u64> = f.iter().map(|x| x.0).collect();
        assert_eq!(ids.len(), f.len());
        let types: std::collections::HashSet<FailureType> = f.iter().map(|x| x.1).collect();
        assert_eq!(types.len(), f.len());
        // Spot checks against a380_systems_wasm lib.rs.
        assert!(f.contains(&(24_106, FailureType::ElectricalBus(ElectricalBusType::AlternatingCurrentNamed("247XP")))));
        assert!(f.contains(&(26_018, FailureType::FireDetectionLoop(FireDetectionLoopID::B, FireDetectionZone::Mlg))));
        assert!(f.contains(&(29_017, FailureType::EnginePumpOverheat(AirbusEngineDrivenPumpId::Edp4b))));
        assert!(f.contains(&(32_015, FailureType::GearProxSensorDamage(ProximityDetectorId::DownlockDoorLeft1))));
        assert!(f.contains(&(32_025, FailureType::GearActuatorJammed(GearActuatorId::GearDoorRight))));
        assert!(f.contains(&(34_022, FailureType::RadioAntennaDirectCoupling(3))));
        assert_eq!(failure_name(26_010), "Engine 2 Loop B");
        assert_eq!(failure_name(24_109), "DC 2");
        assert_eq!(failure_name(21_029), "OCSM 2 Channel 2");
    }

    /// The C++ computers' FailuresConsumer ids (FailureList.h) are registered
    /// alongside the Rust systems' 146, toggle through the same X-Plane
    /// entry points, and never collide with the systems' own ids.
    #[test]
    fn the_computer_failure_ids_are_registered_and_toggle_like_the_others() {
        let _g = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        assert_eq!(COMPUTER_FAILURES.len(), 11);
        let computer_ids: BTreeSet<u64> = COMPUTER_FAILURES.iter().map(|(id, _)| *id).collect();
        assert_eq!(computer_ids.len(), 11, "no duplicate computer ids");
        let systems_ids: BTreeSet<u64> = a380_failures().iter().map(|(id, _)| *id).collect();
        assert!(computer_ids.is_disjoint(&systems_ids), "computer ids never alias a systems FailureType id");

        let f = Failures::new();
        let ids: BTreeSet<u64> = f.ids().collect();
        // 146 systems + 11 computer ids, plus workstream 6's own catalogue
        // extension (`extra::extra_failures`), which `Failures::ids` also
        // exposes so every registered id toggles through the same command/
        // dataref entry points.
        assert_eq!(ids.len(), systems_ids.len() + 11 + extra::extra_failures().len());
        for &id in &computer_ids {
            assert!(ids.contains(&id));
        }
        assert_eq!(failure_name(27_000), "PRIM 1");
        assert_eq!(failure_name(27_006), "FCDC 1");
        assert_eq!(failure_name(22_001), "ROLLOUT");

        // Toggle PRIM 2 (27001) as the command handler would.
        toggle(27_001);
        assert_eq!(active_ids(), vec![27_001]);
        assert_eq!(unsafe { get_one(27_001 as *mut c_void) }, 1);
        toggle(27_001);
        assert!(active_ids().is_empty());
        drop(f);
    }

    #[test]
    fn the_array_dataref_replaces_the_set_and_ignores_unknown_ids() {
        let _g = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        let _f = Failures::new();
        let mut values = [29_000, 12_345, 26_001];
        unsafe { set_array(std::ptr::null_mut(), values.as_mut_ptr(), 0, 3) };
        assert_eq!(active_ids(), vec![26_001, 29_000]);
        let mut out = [0; 4];
        let n = unsafe { get_array(std::ptr::null_mut(), out.as_mut_ptr(), 0, 4) };
        assert_eq!(n, 4);
        assert_eq!(out, [26_001, 29_000, 0, 0]);
        unsafe { set_one(26_001 as *mut c_void, 0) };
        toggle(34_000);
        assert_eq!(active_ids(), vec![29_000, 34_000]);
        assert_eq!(unsafe { get_one(34_000 as *mut c_void) }, 1);
        replace([]);
    }

    fn tick(sim: &mut Simulation<A380>, vars: &mut TestVars, n: usize) {
        for i in 0..n {
            sim.tick(Duration::from_millis(50), 100. + i as f64 * 0.05, vars);
        }
    }

    #[test]
    fn an_engine_fire_failure_reaches_the_systems() {
        let _g = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        let mut vars = TestVars::default();
        let mut sim = Simulation::new(StartState::Cruise, A380::new, &mut vars);
        let mut failures = Failures::new();
        assert!(failures.apply(&mut sim, &mut vars, None).is_empty(), "the start set is empty");
        tick(&mut sim, &mut vars, 3);
        assert_eq!(vars.value("A32NX_ENG_1_ON_FIRE"), 0.);

        set_active(26_001, true);
        let log = failures.apply(&mut sim, &mut vars, None);
        assert_eq!(log, vec!["failure 26001 (Fire - Engine 1) activated".to_owned()]);
        tick(&mut sim, &mut vars, 3);
        assert_eq!(vars.value("A32NX_ENG_1_ON_FIRE"), 1.);
        assert_eq!(vars.value("A32NX_ENG_2_ON_FIRE"), 0.);

        set_active(26_001, false);
        assert_eq!(failures.apply(&mut sim, &mut vars, None), vec!["failure 26001 (Fire - Engine 1) cleared".to_owned()]);
        tick(&mut sim, &mut vars, 3);
        assert_eq!(vars.value("A32NX_ENG_1_ON_FIRE"), 0.);
    }

    /// Failure, systems, the fire aspect and back into the systems: the
    /// SetOnFire module writes ENG_1_ON_FIRE, the aspect copies it to MSFS's
    /// ENG ON FIRE:1, and the fire detection unit reports it.
    #[test]
    fn an_engine_fire_is_detected_through_the_fire_aspect() {
        let _g = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        let mut vars = TestVars::default();
        let mut sim = Simulation::new(StartState::Apron, A380::new, &mut vars);
        let mut aspects = crate::aspects::a380(&mut vars);
        let mut failures = Failures::new();
        for bat in ["BAT_1", "BAT_2", "BAT_ESS", "BAT_APU"] {
            vars.set(&format!("A32NX_OVHD_ELEC_{bat}_PB_IS_AUTO"), 1.);
        }
        let mut run = |sim: &mut Simulation<A380>, vars: &mut TestVars, failures: &mut Failures, n: usize| {
            for i in 0..n {
                aspects.pre_tick(vars, 0.05);
                failures.apply(sim, vars, None);
                sim.tick(Duration::from_millis(50), 10. + i as f64 * 0.05, vars);
                aspects.post_tick(vars);
            }
        };
        run(&mut sim, &mut vars, &mut failures, 40);
        assert_eq!(vars.value("A32NX_FIRE_DETECTED_ENG1"), 0.);
        // Batteries alone leave loop B's DC 2 dead (fire_and_smoke_protection
        // .rs:158-170); with loop B failed, loop A alone confirms a fire.
        set_active(26_008, true);
        set_active(26_001, true);
        run(&mut sim, &mut vars, &mut failures, 40);
        assert_eq!(vars.value("ENG ON FIRE:1"), 1.);
        assert_eq!(
            vars.value("A32NX_FIRE_DETECTED_ENG1"),
            1.,
            "DC ESS {} DC 2 {}",
            vars.value("A32NX_ELEC_DC_ESS_BUS_IS_POWERED"),
            vars.value("A32NX_ELEC_DC_2_BUS_IS_POWERED")
        );
        assert_eq!(vars.value("A32NX_FIRE_DETECTED_ENG2"), 0.);
        replace([]);
    }

    /// Every FlyByWire bus the MSFS fuel circuits follow is one the A380's
    /// electrical system publishes.
    #[test]
    fn the_a380_publishes_every_bus_the_fuel_circuits_follow() {
        let mut vars = TestVars::default();
        let _sim = Simulation::new(StartState::Apron, A380::new, &mut vars);
        for (_, _, bus) in crate::circuits::MSFS_BUSES {
            let name = format!("A32NX_ELEC_{bus}_BUS_IS_POWERED");
            assert!(vars.index.contains_key(&name), "{name}");
        }
    }

    /// A cold apron start on ground power ends with every AC bus powered, and
    /// keeps running: ticking a powered A380 once panicked in the cabin air
    /// model (a cabin filled from zero ambient pressure has no air mass, so its
    /// temperature was NaN and the zone controllers' PID bounds with it),
    /// which threw away every frame's output.
    #[test]
    fn a_cold_apron_start_powers_the_ac_buses_from_external_power() {
        let _g = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        // Zero ambient air first, as a host can report before its first frame,
        // then ISA sea level.
        for ambient in [false, true] {
            let mut vars = TestVars::default();
            if ambient {
                vars.set("AMBIENT PRESSURE", 29.92);
                vars.set("AMBIENT TEMPERATURE", 15.);
                vars.set("AMBIENT DENSITY", 0.002377);
                vars.set("SEA LEVEL PRESSURE", 1013.25);
            }
            let mut sim = Simulation::new(StartState::Apron, A380::new, &mut vars);
            for n in 1..=4 {
                vars.set(&format!("A32NX_EXT_PWR_AVAIL:{n}"), 1.);
                vars.set(&format!("A32NX_OVHD_ELEC_EXT_PWR_{n}_PB_IS_ON"), 1.);
            }
            // Five minutes on ground power, every tick of the last second
            // powered (a source dropping in and out would pass a single look).
            tick(&mut sim, &mut vars, 6000);
            for _ in 0..20 {
                tick(&mut sim, &mut vars, 1);
                for n in 1..=4 {
                    assert_eq!(vars.value(&format!("A32NX_ELEC_AC_{n}_BUS_IS_POWERED")), 1., "AC {n} dropped (ambient {ambient})");
                }
            }
            for bus in ["AC_1", "AC_2", "AC_3", "AC_4"] {
                assert_eq!(vars.value(&format!("A32NX_ELEC_{bus}_BUS_IS_POWERED")), 1., "{bus} (ambient {ambient})");
            }
        }
    }

    #[test]
    fn a_green_reservoir_leak_drains_the_green_reservoir() {
        let _g = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        let run = |leak: bool| {
            let mut vars = TestVars::default();
            let mut sim = Simulation::new(StartState::Cruise, A380::new, &mut vars);
            let mut failures = Failures::new();
            set_active(29_000, leak);
            failures.apply(&mut sim, &mut vars, None);
            tick(&mut sim, &mut vars, 400);
            replace([]);
            (vars.value("A32NX_HYD_GREEN_RESERVOIR_LEVEL"), vars.value("A32NX_HYD_YELLOW_RESERVOIR_LEVEL"))
        };
        let (green_ok, yellow_ok) = run(false);
        let (green_leak, yellow_leak) = run(true);
        assert!(green_ok > 0., "green reservoir level is published: {green_ok}");
        assert!(green_leak < green_ok - 0.01, "leak {green_leak} vs normal {green_ok}");
        assert!((yellow_leak - yellow_ok).abs() < 1e-6);
    }

    /// The Study/Failures tab JSON's cause/components/trigger fields
    /// (`study/web.rs::failures_json`): every registered id across all
    /// three catalogues must have something non-empty to show, the same
    /// "every extra failure has a non-empty description" guarantee
    /// `extra::tests` already gives the new catalogue, extended to the
    /// original 157 and the 11 computer faults.
    #[test]
    fn every_registered_id_has_a_cause_components_and_trigger() {
        let mut ids: BTreeSet<u64> = a380_failures().iter().map(|(id, _)| *id).collect();
        ids.extend(COMPUTER_FAILURES.iter().map(|(id, _)| *id));
        ids.extend(extra::extra_failures().iter().map(|x| x.id));
        for id in ids {
            assert!(!cause_description(id).is_empty(), "{id} has no cause description");
            assert!(!affected_components(id).is_empty(), "{id} has no affected components");
            assert!(!affected_components(id).iter().any(|c| c.is_empty()), "{id} has an empty component name");
            assert!(!trigger_condition(id).is_empty(), "{id} has no trigger condition");
        }
    }

    /// `trigger_condition`'s damage-armed classification must agree with
    /// `random_failures.rs`'s own exclusion list (`damage_armed` in that
    /// module's tests): the two are hand-kept in sync, so this pins the
    /// wording to at least stay internally consistent -- a damage-armed id
    /// never claims plain MTBF eligibility.
    #[test]
    fn damage_armed_ids_are_never_described_as_mtbf_eligible() {
        let damage_armed = (32_100..=32_123).chain(72_000..=72_011).chain(std::iter::once(34_120)).chain(std::iter::once(49_000));
        for id in damage_armed {
            let t = trigger_condition(id);
            assert!(t.contains("wear/exceedance-armed") || !t.contains("MTBF"), "{id}: {t}");
        }
    }
}
