use std::cell::Ref;

use uom::si::{
    electric_current::ampere, electric_potential::volt, f64::*, power::watt,
    thermodynamic_temperature::degree_celsius,
};

use super::{
    ElectricalElement, ElectricalElementIdentifier, ElectricalElementIdentifierProvider,
    ElectricalStateWriter, ElectricityTransformer, Potential, PotentialOrigin, ProvideCurrent,
    ProvidePotential,
};
use crate::{
    failures::{Failure, FailureType},
    shared::{ConsumePower, PowerConsumptionReport},
    simulation::{
        InitContext, Read, SimulationElement, SimulationElementVisitor, SimulatorReader,
        SimulatorWriter, UpdateContext, VariableIdentifier, Write,
    },
};

#[derive(Clone)]
pub struct TransformerRectifier {
    writer: ElectricalStateWriter,
    number: usize,
    input_identifier: ElectricalElementIdentifier,
    output_identifier: ElectricalElementIdentifier,
    failure: Failure,
    output_potential: ElectricPotential,
    output_current: ElectricCurrent,
    /// Case temperature (fbw-xp-systems electrical-sources workstream,
    /// docs/physics/electrical.md): first-order I^2R-in/convective-cooling-
    /// out energy balance, the same technique `battery.rs`'s own thermal
    /// model uses, driven by `resistor_power` (the loss already computed
    /// every tick in `consume_power_in_converters` below but previously
    /// discarded after being folded into the AC-side input power -- this TR
    /// already modelled the *loss*, it just never turned that loss into a
    /// temperature).
    temperature: ThermodynamicTemperature,
    temperature_id: VariableIdentifier,
    /// Continuous diode/winding degradation magnitude in [0, 1] (0 = new
    /// unit), written by the plugin as a plain simulator variable (the
    /// breaker-variable pattern this workstream's other degradation inputs
    /// use; reads 0 until the plugin writes it). Scales
    /// `INTERNAL_RESISTANCE_OHM` up towards `DEGRADED_RESISTANCE_OHM` --
    /// aged rectifier diodes/windings have a measurably higher forward
    /// drop/copper resistance than a new unit. Will move to
    /// `failures::magnitude(id)` once that lands.
    resistance_degradation_id: VariableIdentifier,
    resistance_degradation: f64,
}
impl TransformerRectifier {
    // Value determined by output voltage at specific loads
    const INTERNAL_RESISTANCE_OHM: f64 = 0.0135;
    /// Internal resistance at maximum modelled degradation (magnitude 1.0):
    /// roughly 4x the nominal value, a defensible order-of-magnitude
    /// estimate for a badly aged TR's diode/winding resistance (no
    /// FBW/A380-specific aged-TR figure exists; derived/typical,
    /// docs/physics/electrical.md).
    const DEGRADED_RESISTANCE_OHM: f64 = 0.054;
    // Potential output without any load
    const IDLE_OUTPUT_VOLTAGE: f64 = 30.2;
    /// Thermal mass and natural-convection cooling coefficient for a TR's
    /// finned aluminium case; derived/typical estimates (no FBW/A380
    /// figure), the same order of magnitude reasoning as `battery.rs`'s own
    /// thermal constants (docs/physics/electrical.md).
    const THERMAL_MASS_J_PER_KELVIN: f64 = 1500.;
    const COOLING_W_PER_KELVIN: f64 = 3.5;

    pub fn new(context: &mut InitContext, number: usize) -> TransformerRectifier {
        TransformerRectifier {
            writer: ElectricalStateWriter::new(context, &format!("TR_{}", number)),
            number,
            input_identifier: context.next_electrical_identifier(),
            output_identifier: context.next_electrical_identifier(),
            failure: Failure::new(FailureType::TransformerRectifier(number)),
            output_potential: ElectricPotential::new::<volt>(0.),
            output_current: ElectricCurrent::new::<ampere>(0.),
            temperature: ThermodynamicTemperature::new::<degree_celsius>(20.),
            temperature_id: context.get_identifier(format!("ELEC_TR_{}_TEMPERATURE", number)),
            resistance_degradation_id: context
                .get_identifier(format!("ELEC_TR_{}_RESISTANCE_DEGRADATION", number)),
            resistance_degradation: 0.,
        }
    }

    /// This tick's series resistance: the nominal `INTERNAL_RESISTANCE_OHM`
    /// linearly derated towards `DEGRADED_RESISTANCE_OHM` by the
    /// plugin-supplied continuous degradation magnitude. The [0, 1] range is
    /// enforced once, with a loud log, at the substrate boundary the plugin
    /// writes this quantity through
    /// (`fbw-xp-systems/src/invariants.rs::bound_for`'s
    /// `DEGRADATION_KEYWORDS` entry) -- not re-clamped silently here.
    fn internal_resistance_ohm(&self) -> f64 {
        let magnitude = self.resistance_degradation;
        Self::INTERNAL_RESISTANCE_OHM
            + magnitude * (Self::DEGRADED_RESISTANCE_OHM - Self::INTERNAL_RESISTANCE_OHM)
    }

    /// First-order energy balance from this tick's real I^2R conversion
    /// loss (`resistor_power`, already computed in
    /// `consume_power_in_converters`) to ambient, exactly like
    /// `battery.rs::update_temperature`.
    fn update_temperature(&mut self, context: &UpdateContext, resistor_power_watt: f64) {
        let cooling = Self::COOLING_W_PER_KELVIN
            * (self.temperature.get::<degree_celsius>()
                - context.ambient_temperature().get::<degree_celsius>());
        let d_temp_dt = (resistor_power_watt - cooling) / Self::THERMAL_MASS_J_PER_KELVIN;
        self.temperature = ThermodynamicTemperature::new::<degree_celsius>(
            self.temperature.get::<degree_celsius>() + d_temp_dt * context.delta_as_secs_f64(),
        );
    }

    pub fn has_failed(&self) -> bool {
        self.failure.is_active()
    }

    fn calc_potential_for_power(&self, power: Power) -> [ElectricPotential; 2] {
        // Past the maximum power transfer point (P = V_idle^2 / 4R) the
        // quadratic has no real root: the source cannot deliver that power
        // and its output collapses to the half-idle-voltage point, rather
        // than turning NaN and poisoning the network solve.
        let discriminant = (Self::IDLE_OUTPUT_VOLTAGE * Self::IDLE_OUTPUT_VOLTAGE / 4.
            - self.internal_resistance_ohm() * power.get::<watt>())
        .max(0.)
        .sqrt();
        [discriminant, -discriminant]
            .map(|d| -ElectricPotential::new::<volt>(-Self::IDLE_OUTPUT_VOLTAGE / 2. + d))
    }
}
impl ProvideCurrent for TransformerRectifier {
    fn current(&self) -> ElectricCurrent {
        self.output_current
    }

    fn current_normal(&self) -> bool {
        self.output_current > ElectricCurrent::new::<ampere>(5.)
    }
}
provide_potential!(TransformerRectifier, (25.0..=31.0));
impl ElectricalElement for TransformerRectifier {
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
impl ElectricityTransformer for TransformerRectifier {
    fn transform(&self, input: Ref<Potential>) -> Potential {
        if !self.failure.is_active() && input.is_powered() {
            Potential::new(
                PotentialOrigin::TransformerRectifier(self.number),
                ElectricPotential::new::<volt>(28.),
            )
        } else {
            Potential::none()
        }
    }
}
impl SimulationElement for TransformerRectifier {
    fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
        self.failure.accept(visitor);

        visitor.visit(self);
    }

    fn read(&mut self, reader: &mut SimulatorReader) {
        self.resistance_degradation = reader.read(&self.resistance_degradation_id);
    }

    fn write(&self, writer: &mut SimulatorWriter) {
        self.writer.write_direct(self, writer);
        writer.write(&self.temperature_id, self.temperature);
        writer.write(&self.resistance_degradation_id, self.resistance_degradation);
    }

    fn consume_power_in_converters<T: ConsumePower>(
        &mut self,
        context: &UpdateContext,
        consumption: &mut T,
    ) {
        let dc_power =
            consumption.total_consumption_of(PotentialOrigin::TransformerRectifier(self.number));

        let [resistor_potential, dc_potential] = self.calc_potential_for_power(dc_power);
        let dc_current = dc_power / dc_potential;
        let resistor_power = resistor_potential * dc_current;
        let ac_power = dc_power + resistor_power;

        // Add the DC consumption to the TRs input (AC) consumption.
        consumption.consume_from_input(self, ac_power);

        // The same I^2R conversion loss just computed above (`resistor_power`)
        // is real waste heat, not just an AC-side accounting adjustment: feed
        // it into the case thermal balance.
        self.update_temperature(context, resistor_power.get::<watt>().abs());
    }

    fn process_power_consumption_report<T: PowerConsumptionReport>(
        &mut self,
        _: &UpdateContext,
        report: &T,
    ) {
        let consumption =
            report.total_consumption_of(PotentialOrigin::TransformerRectifier(self.number));
        self.output_potential = if report.is_powered(self) {
            self.calc_potential_for_power(consumption)[1]
        } else {
            ElectricPotential::new::<volt>(0.)
        };

        // Unpowered, no current flows (0 W over 0 V is 0 A, not NaN).
        self.output_current = if self.output_potential.get::<volt>() > 0. {
            consumption / self.output_potential
        } else {
            ElectricCurrent::new::<ampere>(0.)
        };
    }
}

#[cfg(test)]
mod transformer_rectifier_tests {
    use uom::si::power::watt;

    use super::*;
    use crate::simulation::test::{ReadByName, WriteByName};
    use crate::simulation::InitContext;
    use crate::{
        electrical::{
            consumption::PowerConsumer, test::TestElectricitySource, ElectricalBus,
            ElectricalBusType, Electricity, PotentialOrigin,
        },
        simulation::{
            test::{SimulationTestBed, TestBed},
            Aircraft, SimulationElementVisitor, UpdateContext,
        },
    };

    struct TransformerRectifierTestBed {
        test_bed: SimulationTestBed<TestAircraft>,
    }
    impl TransformerRectifierTestBed {
        fn with_unpowered_transformer_rectifier() -> Self {
            Self {
                test_bed: SimulationTestBed::new(|context| {
                    TestAircraft::new(context).with_unpowered_transformer_rectifier()
                }),
            }
        }

        fn with_powered_transformer_rectifier() -> Self {
            Self {
                test_bed: SimulationTestBed::new(|context| {
                    TestAircraft::new(context).with_powered_transformer_rectifier()
                }),
            }
        }

        fn current_is_normal(&mut self) -> bool {
            self.read_by_name("ELEC_TR_1_CURRENT_NORMAL")
        }

        fn potential_is_normal(&mut self) -> bool {
            self.read_by_name("ELEC_TR_1_POTENTIAL_NORMAL")
        }

        fn current(&mut self) -> ElectricCurrent {
            self.read_by_name("ELEC_TR_1_CURRENT")
        }

        fn potential(&mut self) -> ElectricPotential {
            self.read_by_name("ELEC_TR_1_POTENTIAL")
        }

        fn transformer_rectifier_is_powered(&self) -> bool {
            self.query_elec(|a, elec| a.transformer_rectifier_is_powered(elec))
        }
    }
    impl TestBed for TransformerRectifierTestBed {
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
        transformer_rectifier: TransformerRectifier,
        bus: ElectricalBus,
        consumer: PowerConsumer,
        transformer_rectifier_consumption: Power,
    }
    impl TestAircraft {
        fn new(context: &mut InitContext) -> Self {
            Self {
                electricity_source: TestElectricitySource::unpowered(
                    context,
                    PotentialOrigin::ApuGenerator(1),
                ),
                transformer_rectifier: TransformerRectifier::new(context, 1),
                bus: ElectricalBus::new(context, ElectricalBusType::DirectCurrent(1)),
                consumer: PowerConsumer::from(ElectricalBusType::DirectCurrent(1)),
                transformer_rectifier_consumption: Power::new::<watt>(0.),
            }
        }

        fn with_powered_transformer_rectifier(mut self) -> Self {
            self.electricity_source.power();
            self
        }

        fn with_unpowered_transformer_rectifier(mut self) -> Self {
            self.electricity_source.unpower();
            self
        }

        fn transformer_rectifier_is_powered(&self, electricity: &Electricity) -> bool {
            electricity.is_powered(&self.transformer_rectifier)
        }

        fn power_demand(&mut self, power: Power) {
            self.consumer.demand(power);
        }

        fn transformer_rectifier_consumption(&self) -> Power {
            self.transformer_rectifier_consumption
        }
    }
    impl Aircraft for TestAircraft {
        fn update_before_power_distribution(
            &mut self,
            _: &UpdateContext,
            electricity: &mut Electricity,
        ) {
            electricity.supplied_by(&self.electricity_source);
            electricity.flow(&self.electricity_source, &self.transformer_rectifier);
            electricity.transform_in(&self.transformer_rectifier);
            electricity.flow(&self.transformer_rectifier, &self.bus);
        }
    }
    impl SimulationElement for TestAircraft {
        fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
            self.transformer_rectifier.accept(visitor);
            self.consumer.accept(visitor);

            visitor.visit(self);
        }

        fn process_power_consumption_report<T: PowerConsumptionReport>(
            &mut self,
            _: &UpdateContext,
            report: &T,
        ) {
            self.transformer_rectifier_consumption =
                report.total_consumption_of(PotentialOrigin::TransformerRectifier(1));
        }
    }

    #[test]
    fn resistance_degradation_sags_output_voltage_as_derived() {
        // Prediction from the source's own Kirchhoff solve
        // (calc_potential_for_power): output V = IDLE/2 + sqrt(IDLE^2/4 -
        // R*P), IDLE = 30.2 V.
        // At nominal R = 0.0135 ohm and P = 200 W:
        //   sqrt(228.01 - 0.0135*200) = sqrt(225.31) = 15.01033...
        //   V = 15.1 + 15.01033 = 30.11033 V
        // At fully degraded R = 0.054 ohm (magnitude 1.0) and P = 200 W:
        //   sqrt(228.01 - 0.054*200) = sqrt(217.21) = 14.73771...
        //   V = 15.1 + 14.73771 = 29.83771 V
        // i.e. a ~0.273 V sag, still inside the TR's own 25-31 V normal band
        // (this magnitude of degradation alone should not itself trip
        // potential_normal).
        let mut nominal = TransformerRectifierTestBed::with_powered_transformer_rectifier();
        nominal.command(|a| a.power_demand(Power::new::<watt>(200.)));
        nominal.run();
        let nominal_potential = nominal.potential().get::<volt>();

        let mut degraded = TransformerRectifierTestBed::with_powered_transformer_rectifier();
        degraded.write_by_name("ELEC_TR_1_RESISTANCE_DEGRADATION", 1.0);
        degraded.command(|a| a.power_demand(Power::new::<watt>(200.)));
        degraded.run();
        let degraded_potential = degraded.potential().get::<volt>();

        assert!((nominal_potential - 30.11033).abs() < 1e-3);
        assert!((degraded_potential - 29.83771).abs() < 1e-3);
        assert!(degraded.potential_is_normal());
    }

    #[test]
    fn when_unpowered_has_no_output() {
        let mut test_bed = TransformerRectifierTestBed::with_unpowered_transformer_rectifier();

        test_bed.run();

        assert!(!test_bed.transformer_rectifier_is_powered());
    }

    #[test]
    fn when_powered_has_output() {
        let mut test_bed = TransformerRectifierTestBed::with_powered_transformer_rectifier();

        test_bed.run();

        assert!(test_bed.transformer_rectifier_is_powered());
    }

    #[test]
    fn when_powered_but_failed_has_no_output() {
        let mut test_bed = TransformerRectifierTestBed::with_powered_transformer_rectifier();
        test_bed.fail(FailureType::TransformerRectifier(1));

        test_bed.run();

        assert!(!test_bed.transformer_rectifier_is_powered());
    }

    #[test]
    fn when_unpowered_current_is_not_normal() {
        let mut test_bed = TransformerRectifierTestBed::with_unpowered_transformer_rectifier();

        test_bed.run();

        assert!(!test_bed.current_is_normal());
    }

    #[test]
    fn when_powered_with_too_little_demand_current_is_not_normal() {
        let mut test_bed = TransformerRectifierTestBed::with_powered_transformer_rectifier();

        test_bed.command(|a| a.power_demand(Power::new::<watt>(5. * 30.)));
        test_bed.run();

        assert!(!test_bed.current_is_normal());
    }

    #[test]
    fn when_powered_with_enough_demand_current_is_normal() {
        let mut test_bed = TransformerRectifierTestBed::with_powered_transformer_rectifier();

        test_bed.command(|a| a.power_demand(Power::new::<watt>((5. * 30.) + 1.)));
        test_bed.run();

        assert!(test_bed.current_is_normal());
    }

    #[test]
    fn when_unpowered_potential_is_not_normal() {
        let mut test_bed = TransformerRectifierTestBed::with_unpowered_transformer_rectifier();

        test_bed.run();

        assert!(!test_bed.potential_is_normal());
    }

    #[test]
    fn when_powered_potential_is_normal() {
        let mut test_bed = TransformerRectifierTestBed::with_powered_transformer_rectifier();

        test_bed.run();

        assert!(test_bed.potential_is_normal());
    }

    #[test]
    fn when_unpowered_has_no_consumption() {
        let mut test_bed = TransformerRectifierTestBed::with_unpowered_transformer_rectifier();

        test_bed.run();

        assert_eq!(
            test_bed.query(|a| a.transformer_rectifier_consumption()),
            Power::new::<watt>(0.)
        );
    }

    #[test]
    fn when_powered_without_demand_has_no_consumption() {
        let mut test_bed = TransformerRectifierTestBed::with_powered_transformer_rectifier();

        test_bed.command(|a| a.power_demand(Power::new::<watt>(0.)));
        test_bed.run();

        assert_eq!(
            test_bed.query(|a| a.transformer_rectifier_consumption()),
            Power::new::<watt>(0.)
        );
    }

    #[test]
    fn when_powered_with_demand_has_consumption() {
        let mut test_bed = TransformerRectifierTestBed::with_powered_transformer_rectifier();

        test_bed.command(|a| a.power_demand(Power::new::<watt>(200.)));
        test_bed.run();

        assert_eq!(
            test_bed.query(|a| a.transformer_rectifier_consumption()),
            Power::new::<watt>(200.)
        );
    }

    #[test]
    fn when_powered_with_demand_current_is_based_on_demand() {
        let mut test_bed = TransformerRectifierTestBed::with_powered_transformer_rectifier();

        let demand = Power::new::<watt>(200.);
        test_bed.command(|a| a.power_demand(demand));
        test_bed.run();

        assert_eq!(test_bed.current(), demand / test_bed.potential());
    }

    #[test]
    fn writes_its_state() {
        let mut test_bed = TransformerRectifierTestBed::with_powered_transformer_rectifier();

        test_bed.run();

        assert!(test_bed.contains_variable_with_name("ELEC_TR_1_CURRENT"));
        assert!(test_bed.contains_variable_with_name("ELEC_TR_1_CURRENT_NORMAL"));
        assert!(test_bed.contains_variable_with_name("ELEC_TR_1_POTENTIAL"));
        assert!(test_bed.contains_variable_with_name("ELEC_TR_1_POTENTIAL_NORMAL"));
    }
}
