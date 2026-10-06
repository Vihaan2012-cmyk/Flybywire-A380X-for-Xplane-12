use crate::air_conditioning::{
    acs_controller::AcscId, cabin_pressure_controller::CpcId, Channel, VcmId, ZoneType,
};
use crate::air_conditioning::{FdacId, OcsmId};
use crate::integrated_modular_avionics::core_processing_input_output_module::CpiomId;
use crate::shared::{
    AirbusElectricPumpId, AirbusEngineDrivenPumpId, ElectricalBusType, FireDetectionLoopID,
    FireDetectionZone, GearActuatorId, HydraulicColor, LgciuId, ProximityDetectorId,
};
use crate::simulation::SimulationElement;
use rustc_hash::FxHashSet;

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub enum FailureType {
    // ATA21
    Acsc(AcscId),
    CabinFan(usize),
    HotAir(usize),
    HotAirPositionIndication(usize),
    TrimAirOverheat(ZoneType),
    TrimAirFault(ZoneType),
    TrimAirHighPressure,
    GalleyFans,
    CpcFault(CpcId),
    OutflowValveFault,
    SafetyValveFault,
    RapidDecompression,
    Fdac(FdacId, Channel),
    Tadd(Channel),
    Vcm(VcmId, Channel),
    OcsmAutoPartition(OcsmId),
    Ocsm(OcsmId, Channel),
    AgsApp(CpiomId),
    TcsApp(CpiomId),
    VcsApp(CpiomId),
    CpcsApp(CpiomId),
    FwdIsolValve,
    FwdExtractFan,
    BulkIsolValve,
    BulkExtractFan,
    CargoHeater,
    // ATA24
    Generator(usize),
    ApuGenerator(usize),
    TransformerRectifier(usize),
    StaticInverter,
    ElectricalBus(ElectricalBusType),
    // ATA26
    SetOnFire(FireDetectionZone),
    FireDetectionLoop(FireDetectionLoopID, FireDetectionZone),
    // ATA27
    SlatWtb,
    FlapWtb,
    // ATA29
    ReservoirLeak(HydraulicColor),
    ReservoirAirLeak(HydraulicColor),
    ReservoirReturnLeak(HydraulicColor),
    EnginePumpOverheat(AirbusEngineDrivenPumpId),
    ElecPumpOverheat(AirbusElectricPumpId),
    // ATA32
    LgciuPowerSupply(LgciuId),
    LgciuInternalError(LgciuId),
    GearProxSensorDamage(ProximityDetectorId),
    GearActuatorJammed(GearActuatorId),
    BrakeHydraulicLeak(HydraulicColor),
    BrakeAccumulatorGasLeak,
    // ATA34
    RadioAltimeter(usize),
    RadioAntennaInterrupted(usize),
    RadioAntennaDirectCoupling(usize),
    EnhancedGroundProximityWarningSystemComputer,
}

pub struct Failure {
    failure_type: FailureType,
    is_active: bool,
}
impl Failure {
    pub fn new(failure_type: FailureType) -> Self {
        Self {
            failure_type,
            is_active: false,
        }
    }

    pub fn is_active(&self) -> bool {
        self.is_active
    }

    pub fn failure_type(&self) -> FailureType {
        self.failure_type
    }
}
impl SimulationElement for Failure {
    fn receive_failure(&mut self, active_failures: &FxHashSet<FailureType>) {
        self.is_active = active_failures.contains(&self.failure_type);
    }
}

#[derive(Default)]
pub struct ActiveFailureMerge {
    crew: FxHashSet<FailureType>,
    crew_changed: bool,
    derived: Vec<u64>,
}
impl ActiveFailureMerge {
    pub fn set_crew(&mut self, crew: FxHashSet<FailureType>) {
        self.crew = crew;
        self.crew_changed = true;
    }

    pub fn merged(
        &mut self,
        derived: &[u64],
        lookup: impl Fn(u64) -> Option<FailureType>,
    ) -> Option<FxHashSet<FailureType>> {
        let mut derived = derived.to_vec();
        derived.sort_unstable();
        derived.dedup();
        if !self.crew_changed && derived == self.derived {
            return None;
        }
        self.crew_changed = false;
        self.derived = derived;
        let mut set = self.crew.clone();
        set.extend(self.derived.iter().filter_map(|&id| lookup(id)));
        Some(set)
    }
}

#[cfg(test)]
mod merge_tests {
    use super::*;

    fn lookup(id: u64) -> Option<FailureType> {
        match id {
            24_000 => Some(FailureType::TransformerRectifier(1)),
            24_001 => Some(FailureType::TransformerRectifier(2)),
            _ => None,
        }
    }

    fn crew(ids: &[usize]) -> FxHashSet<FailureType> {
        ids.iter().map(|&n| FailureType::TransformerRectifier(n)).collect()
    }

    #[test]
    fn nothing_armed_and_nothing_derived_changes_nothing() {
        let mut m = ActiveFailureMerge::default();
        assert!(m.merged(&[], lookup).is_none());
    }

    #[test]
    fn a_derived_failure_joins_the_crew_set_and_leaves_it_intact_when_it_clears() {
        let mut m = ActiveFailureMerge::default();
        m.set_crew(crew(&[1]));
        assert!(m.merged(&[], lookup) == Some(crew(&[1])));
        assert!(m.merged(&[24_001], lookup) == Some(crew(&[1, 2])));
        assert!(m.merged(&[24_001], lookup).is_none(), "unchanged");
        assert!(m.merged(&[], lookup) == Some(crew(&[1])), "the crew's failure stays");
    }

    #[test]
    fn the_crew_clearing_theirs_leaves_a_derived_one_active() {
        let mut m = ActiveFailureMerge::default();
        m.set_crew(crew(&[2]));
        assert!(m.merged(&[24_001], lookup) == Some(crew(&[2])));
        m.set_crew(crew(&[]));
        assert!(m.merged(&[24_001], lookup) == Some(crew(&[2])), "still derived");
        assert!(m.merged(&[], lookup) == Some(crew(&[])));
    }

    #[test]
    fn unknown_ids_and_repeats_are_ignored() {
        let mut m = ActiveFailureMerge::default();
        assert!(m.merged(&[99_999, 24_000, 24_000], lookup) == Some(crew(&[1])));
        assert!(m.merged(&[24_000, 99_999], lookup).is_none());
    }
}

#[cfg(test)]
mod tests {
    use crate::simulation::test::{SimulationTestBed, TestBed};

    use super::*;

    #[test]
    fn starts_in_a_non_failed_state() {
        let failure = Failure::new(FailureType::TransformerRectifier(1));
        assert!(!failure.is_active());
    }

    #[test]
    fn becomes_failed_when_matching_failure_indicated() {
        let mut test_bed =
            SimulationTestBed::from(Failure::new(FailureType::TransformerRectifier(1)));
        test_bed.fail(FailureType::TransformerRectifier(1));
        test_bed.run();

        assert!(test_bed.query_element(|el| el.is_active()));
    }

    #[test]
    fn does_not_become_failed_when_non_matching_failure_indicated() {
        let mut test_bed =
            SimulationTestBed::from(Failure::new(FailureType::TransformerRectifier(1)));
        test_bed.fail(FailureType::TransformerRectifier(2));
        test_bed.run();

        assert!(test_bed.query_element(|el| !el.is_active()));
    }
}
