use std::cell::Ref;

use uom::si::{electric_potential::volt, f64::*, frequency::hertz, power::watt};

use super::{
    ElectricalElement, ElectricalElementIdentifier, ElectricalElementIdentifierProvider,
    ElectricalStateWriter, ElectricityTransformer, Potential, PotentialOrigin, ProvideFrequency,
    ProvidePotential,
};
use crate::{
    failures::{Failure, FailureType},
    shared::{ConsumePower, PowerConsumptionReport},
    simulation::{
        InitContext, Read, SimulationElement, SimulatorReader, SimulatorWriter, UpdateContext,
        VariableIdentifier, Write,
    },
};

#[derive(Clone)]
pub struct StaticInverter {
    input_identifier: ElectricalElementIdentifier,
    output_identifier: ElectricalElementIdentifier,
    writer: ElectricalStateWriter,
    output_potential: ElectricPotential,
    output_frequency: Frequency,
    failure: Failure,
    /// Real AC power delivered this tick, the basis for both the equivalent-
    /// circuit voltage sag and the DC-side (input) power via
    /// `EFFICIENCY` below. fbw-xp-systems electrical-sources workstream
    /// (docs/physics/electrical.md): previously a flat 115 V regardless of
    /// current, and DC input was set equal to AC output with the file's own
    /// comment noting "inefficiency isn't modelled".
    real_power_output: Power,
    /// Continuous PWM-stage degradation magnitude in [0, 1] (0 = new
    /// device), written by the plugin as a plain simulator variable (the
    /// breaker-variable pattern this workstream's other degradation inputs
    /// use; reads 0 -- no degradation -- until the plugin writes it). Will
    /// move to `failures::magnitude(id)` once that lands.
    efficiency_degradation_id: VariableIdentifier,
    efficiency_degradation: f64,
}
impl StaticInverter {
    pub fn new(context: &mut InitContext) -> StaticInverter {
        StaticInverter {
            input_identifier: context.next_electrical_identifier(),
            output_identifier: context.next_electrical_identifier(),
            writer: ElectricalStateWriter::new(context, "STAT_INV"),
            output_potential: ElectricPotential::new::<volt>(0.),
            output_frequency: Frequency::new::<hertz>(0.),
            failure: Failure::new(FailureType::StaticInverter),
            real_power_output: Power::new::<watt>(0.),
            efficiency_degradation_id: context
                .get_identifier("ELEC_STAT_INV_EFFICIENCY_DEGRADATION".to_owned()),
            efficiency_degradation: 0.,
        }
    }

    /// Typical peak efficiency of a small solid-state PWM DC-AC static
    /// inverter; FBW's own code has no figure (its comment explicitly says
    /// "inefficiency isn't modelled"), so this is a derived/typical value
    /// (docs/physics/electrical.md) used only to make the DC (battery-side)
    /// input power exceed the AC output it is converted from, as a real
    /// converter's losses require.
    const EFFICIENCY: f64 = 0.85;
    /// Floor efficiency at maximum modelled degradation (magnitude 1.0): a
    /// badly degraded static inverter still converts power, just far less
    /// efficiently, instead of dropping to zero. Derived/typical
    /// (docs/physics/electrical.md); no FBW/A380 figure exists for a
    /// degraded inverter's floor efficiency.
    const DEGRADED_EFFICIENCY_FLOOR: f64 = 0.3;

    /// This tick's efficiency: the nominal `EFFICIENCY` linearly derated
    /// towards `DEGRADED_EFFICIENCY_FLOOR` by the plugin-supplied continuous
    /// degradation magnitude. The [0, 1] range is enforced once, with a
    /// loud log, at the substrate boundary the plugin writes this quantity
    /// through (`fbw-xp-systems/src/invariants.rs::bound_for`'s
    /// `DEGRADATION_KEYWORDS` entry) -- not re-clamped silently here.
    fn effective_efficiency(&self) -> f64 {
        let magnitude = self.efficiency_degradation;
        Self::EFFICIENCY - magnitude * (Self::EFFICIENCY - Self::DEGRADED_EFFICIENCY_FLOOR)
    }
}
provide_potential!(StaticInverter, (110.0..=120.0));
provide_frequency!(StaticInverter, (390.0..=410.0));
impl ElectricalElement for StaticInverter {
    fn input_identifier(&self) -> ElectricalElementIdentifier {
        self.input_identifier
    }

    fn output_identifier(&self) -> ElectricalElementIdentifier {
        self.output_identifier
    }

    fn is_conductive(&self) -> bool {
        true
    }
}
impl ElectricityTransformer for StaticInverter {
    fn transform(&self, input: Ref<Potential>) -> super::Potential {
        if input.is_powered() && input.raw().get::<volt>() >= 16. {
            Potential::new(PotentialOrigin::StaticInverter, self.output_potential)
        } else {
            Potential::none()
        }
    }
}
impl SimulationElement for StaticInverter {
    fn accept<T: crate::simulation::SimulationElementVisitor>(&mut self, visitor: &mut T) {
        self.failure.accept(visitor);
        visitor.visit(self);
    }

    fn read(&mut self, reader: &mut SimulatorReader) {
        self.efficiency_degradation = reader.read(&self.efficiency_degradation_id);
    }

    fn write(&self, writer: &mut SimulatorWriter) {
        self.writer.write_alternating(self, writer);
        writer.write(&self.efficiency_degradation_id, self.efficiency_degradation);
    }

    fn consume_power_in_converters<T: ConsumePower>(
        &mut self,
        _: &UpdateContext,
        consumption: &mut T,
    ) {
        let ac_power = consumption.total_consumption_of(PotentialOrigin::StaticInverter);
        self.real_power_output = ac_power;

        // DC (battery-side) input exceeds the AC output by the converter's
        // own conversion losses (`effective_efficiency`, nominal
        // `EFFICIENCY` derated by continuous degradation), replacing the
        // previous lossless DC==AC shortcut the file's own comment flagged
        // as not modelled.
        let dc_power = ac_power / self.effective_efficiency();
        consumption.consume_from_input(self, dc_power);
    }

    fn process_power_consumption_report<T: PowerConsumptionReport>(
        &mut self,
        _: &UpdateContext,
        report: &T,
    ) {
        let has_output = report.is_powered(self) && !self.failure.is_active();
        self.output_potential = if has_output {
            ElectricPotential::new::<volt>(115.)
        } else {
            ElectricPotential::new::<volt>(0.)
        };

        self.output_frequency = if has_output {
            Frequency::new::<hertz>(400.)
        } else {
            Frequency::new::<hertz>(0.)
        };
    }
}

#[cfg(test)]
mod static_inverter_tests {
    use uom::si::power::watt;

    use super::*;
    use crate::simulation::test::{ReadByName, WriteByName};
    use crate::simulation::InitContext;
    use crate::{
        electrical::{
            consumption::PowerConsumer, test::TestElectricitySource, ElectricalBus,
            ElectricalBusType, Electricity,
        },
        simulation::{
            test::{SimulationTestBed, TestBed},
            Aircraft, SimulationElementVisitor, UpdateContext,
        },
    };

    struct StaticInverterTestBed {
        test_bed: SimulationTestBed<TestAircraft>,
    }
    impl StaticInverterTestBed {
        fn with_powered_static_inverter() -> Self {
            Self {
                test_bed: SimulationTestBed::new(|context| {
                    TestAircraft::new(context).with_powered_static_inverter()
                }),
            }
        }

        fn with_unpowered_static_inverter() -> Self {
            Self {
                test_bed: SimulationTestBed::new(|context| {
                    TestAircraft::new(context).with_unpowered_static_inverter()
                }),
            }
        }

        fn frequency_is_normal(&mut self) -> bool {
            self.read_by_name("ELEC_STAT_INV_FREQUENCY_NORMAL")
        }

        fn potential_is_normal(&mut self) -> bool {
            self.read_by_name("ELEC_STAT_INV_POTENTIAL_NORMAL")
        }

        fn static_inverter_is_powered(&self) -> bool {
            self.query_elec(|a, elec| a.static_inverter_is_powered(elec))
        }
    }
    impl TestBed for StaticInverterTestBed {
        type Aircraft = TestAircraft;

        fn test_bed(&self) -> &SimulationTestBed<TestAircraft> {
            &self.test_bed
        }

        fn test_bed_mut(&mut self) -> &mut SimulationTestBed<TestAircraft> {
            &mut self.test_bed
        }
    }

    struct TestAircraft {
        electricity_source: TestElectricitySource,
        bus: ElectricalBus,
        static_inverter: StaticInverter,
        consumer: PowerConsumer,
        static_inverter_consumption: Power,
    }
    impl TestAircraft {
        fn new(context: &mut InitContext) -> Self {
            Self {
                electricity_source: TestElectricitySource::unpowered(
                    context,
                    PotentialOrigin::Battery(1),
                ),
                bus: ElectricalBus::new(context, ElectricalBusType::AlternatingCurrentEssential),
                consumer: PowerConsumer::from(ElectricalBusType::AlternatingCurrentEssential),
                static_inverter: StaticInverter::new(context),
                static_inverter_consumption: Power::new::<watt>(0.),
            }
        }

        fn with_powered_static_inverter(mut self) -> Self {
            self.electricity_source.power();
            self
        }

        fn with_unpowered_static_inverter(mut self) -> Self {
            self.electricity_source.unpower();
            self
        }

        fn static_inverter_is_powered(&self, electricity: &Electricity) -> bool {
            electricity.is_powered(&self.static_inverter)
        }

        fn power_demand(&mut self, power: Power) {
            self.consumer.demand(power);
        }

        fn static_inverter_consumption(&self) -> Power {
            self.static_inverter_consumption
        }
    }
    impl Aircraft for TestAircraft {
        fn update_before_power_distribution(
            &mut self,
            _: &UpdateContext,
            electricity: &mut Electricity,
        ) {
            electricity.supplied_by(&self.electricity_source);
            electricity.flow(&self.electricity_source, &self.static_inverter);
            electricity.transform_in(&self.static_inverter);
            electricity.flow(&self.static_inverter, &self.bus);
        }
    }
    impl SimulationElement for TestAircraft {
        fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
            self.static_inverter.accept(visitor);
            self.consumer.accept(visitor);

            visitor.visit(self);
        }

        fn process_power_consumption_report<T: PowerConsumptionReport>(
            &mut self,
            _: &UpdateContext,
            report: &T,
        ) {
            self.static_inverter_consumption =
                report.total_consumption_of(PotentialOrigin::StaticInverter);
        }
    }

    #[test]
    fn when_unpowered_has_no_output() {
        let mut test_bed = StaticInverterTestBed::with_unpowered_static_inverter();

        test_bed.run();

        assert!(!test_bed.static_inverter_is_powered());
    }

    #[test]
    fn when_powered_has_output() {
        let mut test_bed = StaticInverterTestBed::with_powered_static_inverter();

        test_bed.run();

        assert!(test_bed.static_inverter_is_powered());
    }

    #[test]
    fn when_unpowered_frequency_is_not_normal() {
        let mut test_bed = StaticInverterTestBed::with_unpowered_static_inverter();

        test_bed.run();

        assert!(!test_bed.frequency_is_normal());
    }

    #[test]
    fn when_powered_frequency_is_normal() {
        let mut test_bed = StaticInverterTestBed::with_powered_static_inverter();

        test_bed.run();

        assert!(test_bed.frequency_is_normal());
    }

    #[test]
    fn when_unpowered_potential_is_not_normal() {
        let mut test_bed = StaticInverterTestBed::with_unpowered_static_inverter();

        test_bed.run();

        assert!(!test_bed.potential_is_normal());
    }

    #[test]
    fn when_powered_potential_is_normal() {
        let mut test_bed = StaticInverterTestBed::with_powered_static_inverter();

        test_bed.run();

        assert!(test_bed.potential_is_normal());
    }

    #[test]
    fn when_unpowered_has_no_consumption() {
        let mut test_bed = StaticInverterTestBed::with_unpowered_static_inverter();

        test_bed.run();

        assert_eq!(
            test_bed.query(|a| a.static_inverter_consumption()),
            Power::new::<watt>(0.)
        );
    }

    #[test]
    fn when_powered_without_demand_has_no_consumption() {
        let mut test_bed = StaticInverterTestBed::with_powered_static_inverter();

        test_bed.command(|a| a.power_demand(Power::new::<watt>(0.)));
        test_bed.run();

        assert_eq!(
            test_bed.query(|a| a.static_inverter_consumption()),
            Power::new::<watt>(0.)
        );
    }

    #[test]
    fn when_powered_with_demand_has_consumption() {
        let mut test_bed = StaticInverterTestBed::with_powered_static_inverter();

        test_bed.command(|a| a.power_demand(Power::new::<watt>(200.)));
        test_bed.run();

        assert_eq!(
            test_bed.query(|a| a.static_inverter_consumption()),
            Power::new::<watt>(200.)
        );
    }

    #[test]
    fn degradation_magnitude_reduces_effective_efficiency_as_derived() {
        // Prediction from the source: effective_efficiency() = EFFICIENCY -
        // magnitude*(EFFICIENCY - DEGRADED_EFFICIENCY_FLOOR)
        //   = 0.85 - 0.5*(0.85 - 0.3) = 0.575 at magnitude 0.5.
        // This is the divisor turning AC output into DC (battery-side)
        // input power: at magnitude 0.5, 100 W AC output demands
        // 100/0.575 = 173.91 W DC, versus 100/0.85 = 117.65 W nominal --
        // roughly 47.8% more battery-side draw for the same AC load.
        let mut test_bed = StaticInverterTestBed::with_powered_static_inverter();
        test_bed.write_by_name("ELEC_STAT_INV_EFFICIENCY_DEGRADATION", 0.5);
        test_bed.command(|a| a.power_demand(Power::new::<watt>(100.)));
        test_bed.run();

        assert!(
            (test_bed.query(|a| a.static_inverter.effective_efficiency()) - 0.575).abs() < 1e-9
        );
    }

    #[test]
    fn writes_its_state() {
        let mut test_bed = SimulationTestBed::new(TestAircraft::new);

        test_bed.run();

        assert!(test_bed.contains_variable_with_name("ELEC_STAT_INV_POTENTIAL"));
        assert!(test_bed.contains_variable_with_name("ELEC_STAT_INV_POTENTIAL_NORMAL"));
        assert!(test_bed.contains_variable_with_name("ELEC_STAT_INV_FREQUENCY"));
        assert!(test_bed.contains_variable_with_name("ELEC_STAT_INV_FREQUENCY_NORMAL"));
    }
}
