use systems::air_conditioning::{Channel, FdacId, OcsmId, VcmId};
use systems::failures::FailureType;
use systems::integrated_modular_avionics::core_processing_input_output_module::CpiomId;
use systems::shared::{
    AirbusElectricPumpId, AirbusEngineDrivenPumpId, ElectricalBusType, FireDetectionLoopID, FireDetectionZone, GearActuatorId,
    HydraulicColor, LgciuId, ProximityDetectorId,
};

pub fn fbw_failures() -> Vec<(u64, FailureType)> {
    vec![
        (21_000, FailureType::RapidDecompression),
        (21_001, FailureType::CabinFan(1)),
        (21_002, FailureType::CabinFan(2)),
        (21_003, FailureType::CabinFan(3)),
        (21_004, FailureType::CabinFan(4)),
        (21_005, FailureType::HotAir(1)),
        (21_006, FailureType::HotAir(2)),
        (21_007, FailureType::FwdIsolValve),
        (21_008, FailureType::FwdExtractFan),
        (21_009, FailureType::BulkIsolValve),
        (21_010, FailureType::BulkExtractFan),
        (21_011, FailureType::CargoHeater),
        (21_012, FailureType::Fdac(FdacId::One, Channel::ChannelOne)),
        (21_013, FailureType::Fdac(FdacId::One, Channel::ChannelTwo)),
        (21_014, FailureType::Fdac(FdacId::Two, Channel::ChannelOne)),
        (21_015, FailureType::Fdac(FdacId::Two, Channel::ChannelTwo)),
        (21_016, FailureType::Tadd(Channel::ChannelOne)),
        (21_017, FailureType::Tadd(Channel::ChannelTwo)),
        (21_018, FailureType::Vcm(VcmId::Fwd, Channel::ChannelOne)),
        (21_019, FailureType::Vcm(VcmId::Fwd, Channel::ChannelTwo)),
        (21_020, FailureType::Vcm(VcmId::Aft, Channel::ChannelOne)),
        (21_021, FailureType::Vcm(VcmId::Aft, Channel::ChannelTwo)),
        (21_022, FailureType::OcsmAutoPartition(OcsmId::One)),
        (21_023, FailureType::OcsmAutoPartition(OcsmId::Two)),
        (21_024, FailureType::OcsmAutoPartition(OcsmId::Three)),
        (21_025, FailureType::OcsmAutoPartition(OcsmId::Four)),
        (21_026, FailureType::Ocsm(OcsmId::One, Channel::ChannelOne)),
        (21_027, FailureType::Ocsm(OcsmId::One, Channel::ChannelTwo)),
        (21_028, FailureType::Ocsm(OcsmId::Two, Channel::ChannelOne)),
        (21_029, FailureType::Ocsm(OcsmId::Two, Channel::ChannelTwo)),
        (
            21_030,
            FailureType::Ocsm(OcsmId::Three, Channel::ChannelOne),
        ),
        (
            21_031,
            FailureType::Ocsm(OcsmId::Three, Channel::ChannelTwo),
        ),
        (21_032, FailureType::Ocsm(OcsmId::Four, Channel::ChannelOne)),
        (21_033, FailureType::Ocsm(OcsmId::Four, Channel::ChannelTwo)),
        (21_034, FailureType::AgsApp(CpiomId::B1)),
        (21_035, FailureType::AgsApp(CpiomId::B2)),
        (21_036, FailureType::AgsApp(CpiomId::B3)),
        (21_037, FailureType::AgsApp(CpiomId::B4)),
        (21_038, FailureType::TcsApp(CpiomId::B1)),
        (21_039, FailureType::TcsApp(CpiomId::B2)),
        (21_040, FailureType::TcsApp(CpiomId::B3)),
        (21_041, FailureType::TcsApp(CpiomId::B4)),
        (21_042, FailureType::VcsApp(CpiomId::B1)),
        (21_043, FailureType::VcsApp(CpiomId::B2)),
        (21_044, FailureType::VcsApp(CpiomId::B3)),
        (21_045, FailureType::VcsApp(CpiomId::B4)),
        (21_046, FailureType::CpcsApp(CpiomId::B1)),
        (21_047, FailureType::CpcsApp(CpiomId::B2)),
        (21_048, FailureType::CpcsApp(CpiomId::B3)),
        (21_049, FailureType::CpcsApp(CpiomId::B4)),
        (21_054, FailureType::HotAirPositionIndication(1)),
        (21_055, FailureType::HotAirPositionIndication(2)),
        (24_000, FailureType::TransformerRectifier(1)),
        (24_001, FailureType::TransformerRectifier(2)),
        (24_002, FailureType::TransformerRectifier(3)),
        (24_003, FailureType::TransformerRectifier(4)),
        (24_004, FailureType::StaticInverter),
        (24_020, FailureType::Generator(1)),
        (24_021, FailureType::Generator(2)),
        (24_022, FailureType::Generator(3)),
        (24_023, FailureType::Generator(4)),
        (24_030, FailureType::ApuGenerator(1)),
        (24_031, FailureType::ApuGenerator(2)),
        (
            24_100,
            FailureType::ElectricalBus(ElectricalBusType::AlternatingCurrent(1)),
        ),
        (
            24_101,
            FailureType::ElectricalBus(ElectricalBusType::AlternatingCurrent(2)),
        ),
        (
            24_102,
            FailureType::ElectricalBus(ElectricalBusType::AlternatingCurrent(3)),
        ),
        (
            24_103,
            FailureType::ElectricalBus(ElectricalBusType::AlternatingCurrent(4)),
        ),
        (
            24_104,
            FailureType::ElectricalBus(ElectricalBusType::AlternatingCurrentEssential),
        ),
        (
            24_105,
            FailureType::ElectricalBus(ElectricalBusType::AlternatingCurrentEssentialShed),
        ),
        (
            24_106,
            FailureType::ElectricalBus(ElectricalBusType::AlternatingCurrentNamed("247XP")),
        ),
        (
            24_107,
            FailureType::ElectricalBus(ElectricalBusType::AlternatingCurrentGndFltService),
        ),
        (
            24_108,
            FailureType::ElectricalBus(ElectricalBusType::DirectCurrent(1)),
        ),
        (
            24_109,
            FailureType::ElectricalBus(ElectricalBusType::DirectCurrent(2)),
        ),
        (
            24_110,
            FailureType::ElectricalBus(ElectricalBusType::DirectCurrentEssential),
        ),
        (
            24_111,
            FailureType::ElectricalBus(ElectricalBusType::DirectCurrentNamed("247PP")),
        ),
        (
            24_112,
            FailureType::ElectricalBus(ElectricalBusType::DirectCurrentNamed("309PP")),
        ),
        (
            24_113,
            FailureType::ElectricalBus(ElectricalBusType::DirectCurrentHot(1)),
        ),
        (
            24_114,
            FailureType::ElectricalBus(ElectricalBusType::DirectCurrentHot(2)),
        ),
        (
            24_115,
            FailureType::ElectricalBus(ElectricalBusType::DirectCurrentHot(3)),
        ),
        (
            24_116,
            FailureType::ElectricalBus(ElectricalBusType::DirectCurrentHot(4)),
        ),
        (
            24_117,
            FailureType::ElectricalBus(ElectricalBusType::DirectCurrentGndFltService),
        ),
        (26_001, FailureType::SetOnFire(FireDetectionZone::Engine(1))),
        (26_002, FailureType::SetOnFire(FireDetectionZone::Engine(2))),
        (26_003, FailureType::SetOnFire(FireDetectionZone::Engine(3))),
        (26_004, FailureType::SetOnFire(FireDetectionZone::Engine(4))),
        (26_005, FailureType::SetOnFire(FireDetectionZone::Apu)),
        (26_006, FailureType::SetOnFire(FireDetectionZone::Mlg)),
        (
            26_007,
            FailureType::FireDetectionLoop(FireDetectionLoopID::A, FireDetectionZone::Engine(1)),
        ),
        (
            26_008,
            FailureType::FireDetectionLoop(FireDetectionLoopID::B, FireDetectionZone::Engine(1)),
        ),
        (
            26_009,
            FailureType::FireDetectionLoop(FireDetectionLoopID::A, FireDetectionZone::Engine(2)),
        ),
        (
            26_010,
            FailureType::FireDetectionLoop(FireDetectionLoopID::B, FireDetectionZone::Engine(2)),
        ),
        (
            26_011,
            FailureType::FireDetectionLoop(FireDetectionLoopID::A, FireDetectionZone::Engine(3)),
        ),
        (
            26_012,
            FailureType::FireDetectionLoop(FireDetectionLoopID::B, FireDetectionZone::Engine(3)),
        ),
        (
            26_013,
            FailureType::FireDetectionLoop(FireDetectionLoopID::A, FireDetectionZone::Engine(4)),
        ),
        (
            26_014,
            FailureType::FireDetectionLoop(FireDetectionLoopID::B, FireDetectionZone::Engine(4)),
        ),
        (
            26_015,
            FailureType::FireDetectionLoop(FireDetectionLoopID::A, FireDetectionZone::Apu),
        ),
        (
            26_016,
            FailureType::FireDetectionLoop(FireDetectionLoopID::B, FireDetectionZone::Apu),
        ),
        (
            26_017,
            FailureType::FireDetectionLoop(FireDetectionLoopID::A, FireDetectionZone::Mlg),
        ),
        (
            26_018,
            FailureType::FireDetectionLoop(FireDetectionLoopID::B, FireDetectionZone::Mlg),
        ),
        (29_000, FailureType::ReservoirLeak(HydraulicColor::Green)),
        (29_001, FailureType::ReservoirLeak(HydraulicColor::Yellow)),
        (29_002, FailureType::ReservoirAirLeak(HydraulicColor::Green)),
        (
            29_003,
            FailureType::ReservoirAirLeak(HydraulicColor::Yellow),
        ),
        (
            29_004,
            FailureType::ReservoirReturnLeak(HydraulicColor::Green),
        ),
        (
            29_005,
            FailureType::ReservoirReturnLeak(HydraulicColor::Yellow),
        ),
        (
            29_006,
            FailureType::ElecPumpOverheat(AirbusElectricPumpId::GreenA),
        ),
        (
            29_007,
            FailureType::ElecPumpOverheat(AirbusElectricPumpId::GreenB),
        ),
        (
            29_008,
            FailureType::ElecPumpOverheat(AirbusElectricPumpId::YellowA),
        ),
        (
            29_009,
            FailureType::ElecPumpOverheat(AirbusElectricPumpId::YellowB),
        ),
        (
            29_010,
            FailureType::EnginePumpOverheat(AirbusEngineDrivenPumpId::Edp1a),
        ),
        (
            29_011,
            FailureType::EnginePumpOverheat(AirbusEngineDrivenPumpId::Edp1b),
        ),
        (
            29_012,
            FailureType::EnginePumpOverheat(AirbusEngineDrivenPumpId::Edp2a),
        ),
        (
            29_013,
            FailureType::EnginePumpOverheat(AirbusEngineDrivenPumpId::Edp2b),
        ),
        (
            29_014,
            FailureType::EnginePumpOverheat(AirbusEngineDrivenPumpId::Edp3a),
        ),
        (
            29_015,
            FailureType::EnginePumpOverheat(AirbusEngineDrivenPumpId::Edp3b),
        ),
        (
            29_016,
            FailureType::EnginePumpOverheat(AirbusEngineDrivenPumpId::Edp4a),
        ),
        (
            29_017,
            FailureType::EnginePumpOverheat(AirbusEngineDrivenPumpId::Edp4b),
        ),
        (32_000, FailureType::LgciuPowerSupply(LgciuId::Lgciu1)),
        (32_001, FailureType::LgciuPowerSupply(LgciuId::Lgciu2)),
        (32_002, FailureType::LgciuInternalError(LgciuId::Lgciu1)),
        (32_003, FailureType::LgciuInternalError(LgciuId::Lgciu2)),
        (
            32_004,
            FailureType::GearProxSensorDamage(ProximityDetectorId::UplockGearNose1),
        ),
        (
            32_005,
            FailureType::GearProxSensorDamage(ProximityDetectorId::DownlockGearNose2),
        ),
        (
            32_006,
            FailureType::GearProxSensorDamage(ProximityDetectorId::UplockGearRight1),
        ),
        (
            32_007,
            FailureType::GearProxSensorDamage(ProximityDetectorId::DownlockGearRight2),
        ),
        (
            32_008,
            FailureType::GearProxSensorDamage(ProximityDetectorId::UplockGearLeft2),
        ),
        (
            32_009,
            FailureType::GearProxSensorDamage(ProximityDetectorId::DownlockGearLeft1),
        ),
        (
            32_010,
            FailureType::GearProxSensorDamage(ProximityDetectorId::UplockDoorNose1),
        ),
        (
            32_011,
            FailureType::GearProxSensorDamage(ProximityDetectorId::DownlockDoorNose2),
        ),
        (
            32_012,
            FailureType::GearProxSensorDamage(ProximityDetectorId::UplockDoorRight2),
        ),
        (
            32_013,
            FailureType::GearProxSensorDamage(ProximityDetectorId::DownlockDoorRight1),
        ),
        (
            32_014,
            FailureType::GearProxSensorDamage(ProximityDetectorId::UplockDoorLeft2),
        ),
        (
            32_015,
            FailureType::GearProxSensorDamage(ProximityDetectorId::DownlockDoorLeft1),
        ),
        (
            32_020,
            FailureType::GearActuatorJammed(GearActuatorId::GearNose),
        ),
        (
            32_021,
            FailureType::GearActuatorJammed(GearActuatorId::GearLeft),
        ),
        (
            32_022,
            FailureType::GearActuatorJammed(GearActuatorId::GearRight),
        ),
        (
            32_023,
            FailureType::GearActuatorJammed(GearActuatorId::GearDoorNose),
        ),
        (
            32_024,
            FailureType::GearActuatorJammed(GearActuatorId::GearDoorLeft),
        ),
        (
            32_025,
            FailureType::GearActuatorJammed(GearActuatorId::GearDoorRight),
        ),
        (32_030, FailureType::BrakeAccumulatorGasLeak),
        (34_000, FailureType::RadioAltimeter(1)),
        (34_001, FailureType::RadioAltimeter(2)),
        (34_002, FailureType::RadioAltimeter(3)),
        (34_010, FailureType::RadioAntennaInterrupted(1)),
        (34_011, FailureType::RadioAntennaInterrupted(2)),
        (34_012, FailureType::RadioAntennaInterrupted(3)),
        (34_020, FailureType::RadioAntennaDirectCoupling(1)),
        (34_021, FailureType::RadioAntennaDirectCoupling(2)),
        (34_022, FailureType::RadioAntennaDirectCoupling(3)),
    ]
}
