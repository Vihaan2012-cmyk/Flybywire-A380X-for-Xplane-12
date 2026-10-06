use uom::si::{electric_current::ampere, electric_potential::volt, f64::*, frequency::hertz, power::watt};

use crate::{
    shared::PowerConsumptionReport,
    simulation::{
        InitContext, Read, SimulationElement, SimulatorReader, SimulatorWriter, UpdateContext,
        VariableIdentifier, Write,
    },
};

use super::{
    ElectricalElement, ElectricalElementIdentifier, ElectricalElementIdentifierProvider,
    ElectricalStateWriter, ElectricitySource, Potential, PotentialOrigin, ProvideCurrent,
    ProvideFrequency, ProvidePotential,
};

pub struct ExternalPowerSource {
    external_power_available_id: VariableIdentifier,

    identifier: ElectricalElementIdentifier,
    writer: ElectricalStateWriter,
    is_connected: bool,
    output_frequency: Frequency,
    output_potential: ElectricPotential,
    output_current: ElectricCurrent,
    regulation_degradation_id: VariableIdentifier,
    regulation_degradation: f64,
}
impl ExternalPowerSource {
    const POWER_FACTOR: f64 = 0.8;
    const RATED_APPARENT_POWER_VA: f64 = 90_000.;
    const RATED_VOLTAGE_REGULATION: f64 = 0.02;
    const WEAK_CART_REACTANCE_MULTIPLIER: f64 = 5.0;

    pub fn new(context: &mut InitContext, id: u32) -> ExternalPowerSource {
        ExternalPowerSource {
            external_power_available_id: context.get_identifier(format!("EXT_PWR_AVAIL:{id}")),
            identifier: context.next_electrical_identifier(),
            writer: ElectricalStateWriter::new(context, "EXT_PWR"),
            is_connected: false,
            output_frequency: Frequency::new::<hertz>(0.),
            output_potential: ElectricPotential::new::<volt>(0.),
            output_current: ElectricCurrent::new::<ampere>(0.),
            regulation_degradation_id: context
                .get_identifier(format!("ELEC_EXT_PWR_{id}_REGULATION_DEGRADATION")),
            regulation_degradation: 0.,
        }
    }

    fn calculate_potential_under_load(&self, real_power: Power) -> (ElectricPotential, ElectricCurrent) {
        let target_voltage = 115. * (1. - Self::RATED_VOLTAGE_REGULATION);
        let base_xs =
            target_voltage * (115. - target_voltage) / Self::RATED_APPARENT_POWER_VA;
        let degradation = self.regulation_degradation;
        let xs = base_xs * (1. + degradation * (Self::WEAK_CART_REACTANCE_MULTIPLIER - 1.));
        let apparent_power = real_power.get::<watt>() / Self::POWER_FACTOR;
        let discriminant = 115. * 115. - 4. * apparent_power * xs;
        let voltage = if discriminant < 0. {
            115. / 2.
        } else {
            (115. + discriminant.sqrt()) / 2.
        };
        (
            ElectricPotential::new::<volt>(voltage),
            ElectricCurrent::new::<ampere>(apparent_power / voltage),
        )
    }

    pub fn update(&mut self, _: &UpdateContext) {}

    /// Indicates if the provided electricity's potential and frequency
    /// are within normal parameters. Use this to decide if the
    /// external power contactor should close.
    pub fn output_within_normal_parameters(&self) -> bool {
        self.should_provide_output() && self.potential_normal() && self.frequency_normal()
    }

    fn should_provide_output(&self) -> bool {
        self.is_connected
    }
}
impl ElectricalElement for ExternalPowerSource {
    fn input_identifier(&self) -> super::ElectricalElementIdentifier {
        self.identifier
    }

    fn output_identifier(&self) -> super::ElectricalElementIdentifier {
        self.identifier
    }

    fn is_conductive(&self) -> bool {
        true
    }
}
impl ElectricitySource for ExternalPowerSource {
    fn output_potential(&self) -> Potential {
        if self.should_provide_output() {
            Potential::new(PotentialOrigin::External, self.output_potential)
        } else {
            Potential::none()
        }
    }
}
provide_potential!(ExternalPowerSource, (110.0..=120.0));
provide_frequency!(ExternalPowerSource, (390.0..=410.0));
impl ProvideCurrent for ExternalPowerSource {
    fn current(&self) -> ElectricCurrent {
        self.output_current
    }

    fn current_normal(&self) -> bool {
        let rated_current = (Self::RATED_APPARENT_POWER_VA) / 115.;
        self.output_current.get::<ampere>().abs() <= rated_current
    }
}
impl SimulationElement for ExternalPowerSource {
    fn read(&mut self, reader: &mut SimulatorReader) {
        self.is_connected = reader.read(&self.external_power_available_id);
        self.regulation_degradation = reader.read(&self.regulation_degradation_id);
    }

    fn write(&self, writer: &mut SimulatorWriter) {
        self.writer.write_alternating_with_current(self, writer);
        writer.write(&self.regulation_degradation_id, self.regulation_degradation);
    }

    fn process_power_consumption_report<T: PowerConsumptionReport>(
        &mut self,
        _: &UpdateContext,
        report: &T,
    ) {
        self.output_frequency = if self.should_provide_output() {
            Frequency::new::<hertz>(400.)
        } else {
            Frequency::new::<hertz>(0.)
        };

        (self.output_potential, self.output_current) = if self.should_provide_output() {
            self.calculate_potential_under_load(
                report.total_consumption_of(PotentialOrigin::External),
            )
        } else {
            (ElectricPotential::new::<volt>(0.), ElectricCurrent::new::<ampere>(0.))
        };
    }
}

#[cfg(test)]
mod external_power_source_tests {
    use super::*;
    use crate::simulation::test::{ReadByName, WriteByName};
    use crate::simulation::InitContext;
    use crate::{
        electrical::Electricity,
        simulation::{
            test::{SimulationTestBed, TestBed},
            Aircraft, SimulationElementVisitor,
        },
    };

    struct ExternalPowerTestBed {
        test_bed: SimulationTestBed<TestAircraft>,
    }
    impl ExternalPowerTestBed {
        fn new() -> Self {
            Self {
                test_bed: SimulationTestBed::new(TestAircraft::new),
            }
        }

        fn with_disconnected_external_power(mut self) -> Self {
            self.disconnect_external_power();
            self
        }

        fn with_connected_external_power(mut self) -> Self {
            self.write_by_name("EXT_PWR_AVAIL:1", true);
            self
        }

        fn disconnect_external_power(&mut self) {
            self.write_by_name("EXT_PWR_AVAIL:1", false);
        }

        fn frequency_is_normal(&mut self) -> bool {
            self.read_by_name("ELEC_EXT_PWR_FREQUENCY_NORMAL")
        }

        fn potential_is_normal(&mut self) -> bool {
            self.read_by_name("ELEC_EXT_PWR_POTENTIAL_NORMAL")
        }

        fn ext_pwr_is_powered(&self) -> bool {
            self.query_elec(|a, elec| a.ext_pwr_is_powered(elec))
        }
    }
    impl TestBed for ExternalPowerTestBed {
        type Aircraft = TestAircraft;

        fn test_bed(&self) -> &SimulationTestBed<TestAircraft> {
            &self.test_bed
        }

        fn test_bed_mut(&mut self) -> &mut SimulationTestBed<TestAircraft> {
            &mut self.test_bed
        }
    }

    struct TestAircraft {
        ext_pwr: ExternalPowerSource,
        ext_pwr_output_within_normal_parameters_before_processing_power_consumption_report: bool,
    }
    impl TestAircraft {
        fn new(context: &mut InitContext) -> Self {
            Self {
                ext_pwr: ExternalPowerSource::new(context, 1),
                ext_pwr_output_within_normal_parameters_before_processing_power_consumption_report: false,
            }
        }

        fn ext_pwr_is_powered(&self, electricity: &Electricity) -> bool {
            electricity.is_powered(&self.ext_pwr)
        }

        fn ext_pwr_output_within_normal_parameters_after_processing_power_consumption_report(
            &self,
        ) -> bool {
            self.ext_pwr.output_within_normal_parameters()
        }

        fn ext_pwr_output_within_normal_parameters_before_processing_power_consumption_report(
            &self,
        ) -> bool {
            self.ext_pwr_output_within_normal_parameters_before_processing_power_consumption_report
        }
    }
    impl Aircraft for TestAircraft {
        fn update_before_power_distribution(
            &mut self,
            _context: &UpdateContext,
            electricity: &mut Electricity,
        ) {
            electricity.supplied_by(&self.ext_pwr);
            self.ext_pwr_output_within_normal_parameters_before_processing_power_consumption_report = self.ext_pwr.output_within_normal_parameters();
        }
    }
    impl SimulationElement for TestAircraft {
        fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
            self.ext_pwr.accept(visitor);
            visitor.visit(self);
        }
    }

    #[test]
    fn when_disconnected_provides_no_output() {
        let mut test_bed = ExternalPowerTestBed::new().with_disconnected_external_power();

        test_bed.run();

        assert!(!test_bed.ext_pwr_is_powered());
    }

    #[test]
    fn when_connected_provides_output() {
        let mut test_bed = ExternalPowerTestBed::new().with_connected_external_power();

        test_bed.run();

        assert!(test_bed.ext_pwr_is_powered());
    }

    #[test]
    fn when_disconnected_frequency_not_normal() {
        let mut test_bed = ExternalPowerTestBed::new().with_disconnected_external_power();

        test_bed.run();

        assert!(!test_bed.frequency_is_normal());
    }

    #[test]
    fn when_connected_frequency_normal() {
        let mut test_bed = ExternalPowerTestBed::new().with_connected_external_power();

        test_bed.run();

        assert!(test_bed.frequency_is_normal());
    }

    #[test]
    fn when_disconnected_potential_not_normal() {
        let mut test_bed = ExternalPowerTestBed::new().with_disconnected_external_power();

        test_bed.run();

        assert!(!test_bed.potential_is_normal());
    }

    #[test]
    fn when_connected_potential_normal() {
        let mut test_bed = ExternalPowerTestBed::new().with_connected_external_power();

        test_bed.run();

        assert!(test_bed.potential_is_normal());
    }

    #[test]
    fn output_not_within_normal_parameters_when_disconnected() {
        let mut test_bed = ExternalPowerTestBed::new().with_disconnected_external_power();

        test_bed.run();

        assert!(!test_bed.query(|a| a
            .ext_pwr_output_within_normal_parameters_after_processing_power_consumption_report()));
    }

    #[test]
    fn output_within_normal_parameters_when_connected() {
        let mut test_bed = ExternalPowerTestBed::new().with_connected_external_power();

        test_bed.run();

        assert!(test_bed.query(|a| a
            .ext_pwr_output_within_normal_parameters_after_processing_power_consumption_report()));
    }

    #[test]
    fn output_within_normal_parameters_adapts_to_no_longer_supplying_ext_pwr_instantaneously() {
        // The frequency and potential of the external power source are only known at the end of a tick,
        // due to them being directly related to the power consumption (large changes can cause
        // spikes and dips). However, the decision if EXT PWR source can supply power is made much
        // earlier in the tick. This is especially of great consequence when EXT PWR source no longer
        // supplies potential but the previous tick's frequency and potential are still normal.
        // With this test we ensure that an EXT PWR source which is no longer supplying power is
        // immediately noticed.
        let mut test_bed = ExternalPowerTestBed::new().with_connected_external_power();
        test_bed.run();

        test_bed.disconnect_external_power();
        test_bed.run();

        assert!(!test_bed.query(|a| a
            .ext_pwr_output_within_normal_parameters_before_processing_power_consumption_report()));
    }

    #[test]
    fn writes_its_state() {
        let mut test_bed = SimulationTestBed::new(TestAircraft::new);

        test_bed.run();

        assert!(test_bed.contains_variable_with_name("ELEC_EXT_PWR_POTENTIAL"));
        assert!(test_bed.contains_variable_with_name("ELEC_EXT_PWR_POTENTIAL_NORMAL"));
        assert!(test_bed.contains_variable_with_name("ELEC_EXT_PWR_FREQUENCY"));
        assert!(test_bed.contains_variable_with_name("ELEC_EXT_PWR_FREQUENCY_NORMAL"));
    }
}
