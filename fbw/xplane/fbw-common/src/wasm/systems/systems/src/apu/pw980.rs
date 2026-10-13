use std::time::Duration;

use uom::{
    si::{
        electric_current::ampere, electric_potential::volt, f64::*, frequency::hertz,
        power::watt, pressure::psi, ratio::percent, thermodynamic_temperature::degree_celsius,
    },
    ConstZero,
};

use crate::{
    electrical::{
        ElectricalElement, ElectricalElementIdentifier, ElectricalElementIdentifierProvider,
        ElectricalStateWriter, ElectricitySource, Potential, ProvideCurrent, ProvideFrequency,
        ProvideLoad, ProvidePotential,
    },
    failures::{Failure, FailureType},
    shared::{
        calculate_towards_target_temperature, ConsumePower, ControllerSignal, ElectricalBusType,
        ElectricalBuses, InternationalStandardAtmosphere, PotentialOrigin, PowerConsumptionReport,
    },
    simulation::{InitContext, SimulationElement, SimulatorWriter, UpdateContext, VariableIdentifier, Write},
};

use super::pw980_physics::{ApuProtectiveTrip, Pw980Core};
use super::{ApuConstants, ApuGenerator, ApuStartMotor, Turbine, TurbineSignal, TurbineState};

#[derive(Clone)]
pub struct Pw980Constants;

impl ApuConstants for Pw980Constants {
    const RUNNING_WARNING_EGT: f64 = 900.; // Deg C
    const BLEED_AIR_COOLDOWN_DURATION: Duration = Duration::ZERO;
    const COOLDOWN_DURATION: Duration = Duration::from_secs(60);
    const AIR_INTAKE_FLAP_CLOSURE_PERCENT: f64 = 8.;
    const SHOULD_BE_AVAILABLE_DURING_SHUTDOWN: bool = false;
    const FUEL_LINE_ID: u8 = 141;
}

/// N2 (percent) at which the Starting state hands off to Running. Matches
/// `Pw980ApuGenerator::APU_GEN_POWERED_N` (77%) with headroom so the
/// generator is already providing valid output by the time the ECAM/ECB
/// reports the turbine as "Running" -- the same handoff point the old
/// curve-fit model used (it transitioned at `n() == 100%`, which the
/// physical model's quasi-static output mapping reaches at the same N2
/// this constant is set to; see `pw980_physics::Pw980Core::output_shaft_ratio`).
const STARTING_TO_RUNNING_N2_PERCENT: f64 = 83.0;

#[derive(Clone)]
pub struct ShutdownPw980Turbine {
    egt: ThermodynamicTemperature,
}
impl ShutdownPw980Turbine {
    pub fn new() -> Self {
        ShutdownPw980Turbine {
            egt: ThermodynamicTemperature::new::<degree_celsius>(0.),
        }
    }

    fn new_with_egt(egt: ThermodynamicTemperature) -> Self {
        ShutdownPw980Turbine { egt }
    }
}
impl Turbine for ShutdownPw980Turbine {
    fn update(
        mut self: Box<Self>,
        context: &UpdateContext,
        _: bool,
        _: bool,
        _: bool,
        _: Power,
        _: MassRate,
        controller: &dyn ControllerSignal<TurbineSignal>,
    ) -> Box<dyn Turbine> {
        self.egt = calculate_towards_ambient_egt(self.egt, context);

        match controller.signal() {
            Some(TurbineSignal::StartOrContinue) => Box::new(Spooling::new_starting(self.egt)),
            Some(TurbineSignal::Stop) | None => self,
        }
    }

    fn n(&self) -> Ratio {
        Ratio::default()
    }

    fn n2(&self) -> Ratio {
        Ratio::default()
    }

    fn egt(&self) -> ThermodynamicTemperature {
        self.egt
    }

    fn state(&self) -> TurbineState {
        TurbineState::Shutdown
    }

    fn bleed_air_pressure(&self) -> Pressure {
        Pressure::new::<psi>(14.7)
    }
}

/// Reported turbine phase for a spooling-or-running `Pw980Core`. Both
/// phases run the *same* physics (`Pw980Core::update`, always commanded to
/// run); only the `TurbineState` reported to the ECB, and the point at
/// which "Starting" becomes "Running", differ -- matching the fact that a
/// real gas generator does not physically change behaviour at that
/// boundary, only its annunciated status does.
#[derive(Clone, Copy, PartialEq, Eq)]
enum SpoolPhase {
    Starting,
    Running,
}

/// The gas-generator physically spooling and, once past
/// `STARTING_TO_RUNNING_N2_PERCENT`, running. See `pw980_physics.rs` for
/// the torque/energy balance itself; this type only holds the phase used
/// for reporting and state transitions.
#[derive(Clone)]
struct Spooling {
    core: Pw980Core,
    phase: SpoolPhase,
}
impl Spooling {
    fn new_starting(egt: ThermodynamicTemperature) -> Self {
        Self {
            core: Pw980Core::new(egt, Ratio::default()),
            phase: SpoolPhase::Starting,
        }
    }

    fn bleed_air_pressure_for(phase: SpoolPhase) -> Pressure {
        match phase {
            // No bleed pressure available until the core is actually
            // running and governed (matches the previous model and the
            // `starting_apu_has_no_bleed_air_pressure` reference test).
            SpoolPhase::Starting => Pressure::new::<psi>(14.7),
            // Value from refs, we add standard pressure at sea level as
            // state of unpressurized system.
            SpoolPhase::Running => {
                Pressure::new::<psi>(40.) + InternationalStandardAtmosphere::pressure_at_altitude(Length::ZERO)
            }
        }
    }
}
impl Turbine for Spooling {
    fn update(
        mut self: Box<Self>,
        context: &UpdateContext,
        apu_bleed_is_used: bool,
        apu_gen_is_used: bool,
        starter_powered: bool,
        elec_shaft_power: Power,
        bleed_extraction: MassRate,
        controller: &dyn ControllerSignal<TurbineSignal>,
    ) -> Box<dyn Turbine> {
        let bleed_pressure = Self::bleed_air_pressure_for(self.phase);
        self.core.update(
            context,
            true,
            starter_powered,
            if apu_gen_is_used { elec_shaft_power } else { Power::default() },
            if apu_bleed_is_used { bleed_extraction } else { MassRate::default() },
            bleed_pressure,
        );

        if self.phase == SpoolPhase::Starting
            && self.core.n2().get::<percent>() >= STARTING_TO_RUNNING_N2_PERCENT
        {
            self.phase = SpoolPhase::Running;
        }

        match controller.signal() {
            Some(TurbineSignal::StartOrContinue) if self.core.protective_trip().is_none() => self,
            _ => Box::new(Stopping::new(self.core)),
        }
    }

    fn n(&self) -> Ratio {
        self.core.output_shaft_ratio()
    }

    fn n2(&self) -> Ratio {
        self.core.n2()
    }

    fn egt(&self) -> ThermodynamicTemperature {
        self.core.egt()
    }

    fn state(&self) -> TurbineState {
        match self.phase {
            SpoolPhase::Starting => TurbineState::Starting,
            SpoolPhase::Running => TurbineState::Running,
        }
    }

    fn bleed_air_pressure(&self) -> Pressure {
        Self::bleed_air_pressure_for(self.phase)
    }

    fn fuel_flow(&self) -> MassRate {
        self.core.fuel_flow()
    }

    fn bleed_mass_flow_capacity(&self) -> MassRate {
        if self.phase == SpoolPhase::Running {
            self.core.bleed_flow_capacity()
        } else {
            MassRate::default()
        }
    }

    fn starter_current(&self) -> ElectricCurrent {
        self.core.starter_current()
    }

    fn oil_pressure(&self) -> Pressure {
        self.core.oil_pressure()
    }

    fn oil_temperature(&self) -> ThermodynamicTemperature {
        self.core.oil_temperature()
    }

    fn protective_trip(&self) -> Option<&'static str> {
        self.core.protective_trip().map(trip_name)
    }
}

fn trip_name(trip: ApuProtectiveTrip) -> &'static str {
    match trip {
        ApuProtectiveTrip::Overspeed => "APU N2 OVERSPEED",
        ApuProtectiveTrip::OverTemperature => "APU EGT OVER LIMIT",
        ApuProtectiveTrip::LowOilPressure => "APU LOW OIL PRESSURE",
    }
}

/// Fuel cut off, core decelerating under friction/windmilling alone (see
/// `Pw980Core::update`'s `running == false` path) until it reaches zero.
#[derive(Clone)]
struct Stopping {
    core: Pw980Core,
}
impl Stopping {
    fn new(core: Pw980Core) -> Self {
        Self { core }
    }
}
impl Turbine for Stopping {
    fn update(
        mut self: Box<Self>,
        context: &UpdateContext,
        _: bool,
        _: bool,
        _: bool,
        _: Power,
        _: MassRate,
        _: &dyn ControllerSignal<TurbineSignal>,
    ) -> Box<dyn Turbine> {
        let bleed_pressure = Pressure::new::<psi>(14.7);
        self.core.update(
            context,
            false,
            false,
            Power::default(),
            MassRate::default(),
            bleed_pressure,
        );

        if self.core.n2().get::<percent>() < 0.5 {
            Box::new(ShutdownPw980Turbine::new_with_egt(self.core.egt()))
        } else {
            self
        }
    }

    fn n(&self) -> Ratio {
        self.core.output_shaft_ratio()
    }

    fn n2(&self) -> Ratio {
        self.core.n2()
    }

    fn egt(&self) -> ThermodynamicTemperature {
        self.core.egt()
    }

    fn state(&self) -> TurbineState {
        TurbineState::Stopping
    }

    fn bleed_air_pressure(&self) -> Pressure {
        Pressure::new::<psi>(14.7)
    }

    fn fuel_flow(&self) -> MassRate {
        MassRate::default()
    }

    fn oil_pressure(&self) -> Pressure {
        self.core.oil_pressure()
    }

    fn oil_temperature(&self) -> ThermodynamicTemperature {
        self.core.oil_temperature()
    }
}

fn calculate_towards_ambient_egt(
    current_egt: ThermodynamicTemperature,
    context: &UpdateContext,
) -> ThermodynamicTemperature {
    const APU_AMBIENT_COEFFICIENT: f64 = 1.;
    calculate_towards_target_temperature(
        current_egt,
        context.ambient_temperature(),
        APU_AMBIENT_COEFFICIENT,
        context.delta(),
    )
}

/// PW980 APU Generator
#[derive(Clone)]
pub struct Pw980ApuGenerator {
    number: usize,
    identifier: ElectricalElementIdentifier,
    n: Ratio,
    writer: ElectricalStateWriter,
    /// The APU's half of the shared engine-load contract
    /// (`ENGINE_GEARBOX_ELEC_LOAD_W:0`, docs/briefs/hyperrealism.md), a
    /// simulator variable so the plugin can read it without a Rust API
    /// across the plugin/systems boundary.
    shaft_power_demand_id: VariableIdentifier,
    output_frequency: Frequency,
    output_potential: ElectricPotential,
    output_current: ElectricCurrent,
    /// Real electrical power delivered this tick; the APU's half of the
    /// shared engine-load contract (`ENGINE_GEARBOX_ELEC_LOAD_W:0`,
    /// docs/briefs/hyperrealism.md) divides this by `GENERATOR_EFFICIENCY`.
    real_power_output: Power,
    load: Ratio,
    is_emergency_shutdown: bool,
    failure: Failure,
}
impl Pw980ApuGenerator {
    pub(super) const APU_GEN_POWERED_N: f64 = 77.;
    /// Real A380 APU generator rating already used by this file's own load
    /// calculation below (120 kW at the same 0.8 power factor as the engine
    /// VFGs). No FBW figure exists for the generator's own regulation or
    /// synchronous reactance, so the same 3% derived estimate as
    /// engine_generator.rs's VFGs is used here (electrical.md), for
    /// consistency between the two generator families sharing one bus
    /// architecture.
    const RATED_VOLTAGE_REGULATION: f64 = 0.03;
    const GENERATOR_EFFICIENCY: f64 = 0.88;
    const MAXIMUM_LOAD_WATT: f64 = 120000.;
    const POWER_FACTOR: f64 = 0.8;

    pub fn new(context: &mut InitContext, number: usize) -> Pw980ApuGenerator {
        Pw980ApuGenerator {
            number,
            identifier: context.next_electrical_identifier(),
            n: Ratio::default(),
            writer: ElectricalStateWriter::new(context, &format!("APU_GEN_{}", number)),
            shaft_power_demand_id: context
                .get_identifier(format!("ELEC_APU_GEN_{}_SHAFT_POWER_DEMAND", number)),
            output_potential: ElectricPotential::default(),
            output_current: ElectricCurrent::default(),
            real_power_output: Power::default(),
            output_frequency: Frequency::default(),
            load: Ratio::default(),
            is_emergency_shutdown: false,
            failure: Failure::new(FailureType::ApuGenerator(number)),
        }
    }

    /// Real electrical power this generator delivered to the buses this
    /// tick.
    pub fn real_power_output(&self) -> Power {
        self.real_power_output
    }

    /// Mechanical shaft power the APU's own gearbox must supply to produce
    /// `real_power_output()` at `GENERATOR_EFFICIENCY`.
    pub fn shaft_power_demand(&self) -> Power {
        self.real_power_output / Self::GENERATOR_EFFICIENCY
    }

    /// Equivalent synchronous reactance, same construction as
    /// engine_generator.rs's VFGs: sized so the same quadratic used in
    /// `calculate_potential_under_load` sags the terminal voltage to
    /// `115*(1-RATED_VOLTAGE_REGULATION)` at rated apparent power.
    fn synchronous_reactance_ohm() -> f64 {
        let rated_apparent_power = Self::MAXIMUM_LOAD_WATT / Self::POWER_FACTOR;
        let target_voltage = 115. * (1. - Self::RATED_VOLTAGE_REGULATION);
        target_voltage * (115. - target_voltage) / rated_apparent_power
    }

    /// Kirchhoff's loop across the equivalent synchronous reactance:
    /// V = 115 - I*Xs with I = S/V (S the apparent power at the nameplate
    /// power factor), solved as `V^2 - 115*V + S*Xs = 0`, the same
    /// quadratic-in-V technique used throughout electrical/ for an
    /// output-impedance source. Replaces the previous flat 115 V regardless
    /// of current.
    fn calculate_potential_under_load(real_power: Power) -> (ElectricPotential, ElectricCurrent) {
        let apparent_power = real_power.get::<watt>() / Self::POWER_FACTOR;
        let xs = Self::synchronous_reactance_ohm();
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


    fn calculate_frequency(&self, n: Ratio) -> Frequency {
        let n = n.get::<percent>();

        // Refer to PW980.md for details on the values below and source data.
        if n < Pw980ApuGenerator::APU_GEN_POWERED_N {
            panic!("Should not be invoked for APU N below {}", n);
        } else if n < 100. {
            const APU_FREQ_CONST: f64 = -7946988.668472081;
            const APU_FREQ_X: f64 = 536340.196388485;
            const APU_FREQ_X2: f64 = -15054.60305885912;
            const APU_FREQ_X3: f64 = 224.9581339994097;
            const APU_FREQ_X4: f64 = -1.887344238031437;
            const APU_FREQ_X5: f64 = 0.008429263577308244;
            const APU_FREQ_X6: f64 = -0.00001565694620869725;

            Frequency::new::<hertz>(
                APU_FREQ_CONST
                    + (APU_FREQ_X * n)
                    + (APU_FREQ_X2 * n.powi(2))
                    + (APU_FREQ_X3 * n.powi(3))
                    + (APU_FREQ_X4 * n.powi(4))
                    + (APU_FREQ_X5 * n.powi(5))
                    + (APU_FREQ_X6 * n.powi(6)),
            )
        } else {
            Frequency::new::<hertz>(400.)
        }
    }

    fn should_provide_output(&self) -> bool {
        !self.failure.is_active()
            && !self.is_emergency_shutdown
            && self.n.get::<percent>() >= Pw980ApuGenerator::APU_GEN_POWERED_N
    }
}
impl ApuGenerator for Pw980ApuGenerator {
    fn update(&mut self, n: Ratio, is_emergency_shutdown: bool) {
        self.n = n;
        self.is_emergency_shutdown = is_emergency_shutdown;
    }

    /// Indicates if the provided electricity's potential and frequency
    /// are within normal parameters. Use this to decide if the
    /// generator contactor should close.
    /// Load shouldn't be taken into account, as overloading causes an
    /// overtemperature which over time will trigger a mechanical
    /// disconnect of the generator.
    fn output_within_normal_parameters(&self) -> bool {
        self.should_provide_output() && self.potential_normal() && self.frequency_normal()
    }

    fn shaft_power_demand(&self) -> Power {
        self.real_power_output / Self::GENERATOR_EFFICIENCY
    }
}
provide_potential!(Pw980ApuGenerator, (110.0..=120.0));
provide_frequency!(Pw980ApuGenerator, (390.0..=410.0));
provide_load!(Pw980ApuGenerator);
impl ProvideCurrent for Pw980ApuGenerator {
    fn current(&self) -> ElectricCurrent {
        self.output_current
    }

    fn current_normal(&self) -> bool {
        let rated_current = (Self::MAXIMUM_LOAD_WATT / Self::POWER_FACTOR) / 115.;
        self.output_current.get::<ampere>().abs() <= rated_current
    }
}
impl ElectricalElement for Pw980ApuGenerator {
    fn input_identifier(&self) -> ElectricalElementIdentifier {
        self.identifier
    }

    fn output_identifier(&self) -> ElectricalElementIdentifier {
        self.identifier
    }

    fn is_conductive(&self) -> bool {
        true
    }
}
impl ElectricitySource for Pw980ApuGenerator {
    fn output_potential(&self) -> Potential {
        if self.should_provide_output() {
            Potential::new(
                PotentialOrigin::ApuGenerator(self.number),
                self.output_potential,
            )
        } else {
            Potential::none()
        }
    }
}
impl SimulationElement for Pw980ApuGenerator {
    fn accept<T: crate::simulation::SimulationElementVisitor>(&mut self, visitor: &mut T) {
        self.failure.accept(visitor);
        visitor.visit(self);
    }

    fn write(&self, writer: &mut SimulatorWriter) {
        self.writer.write_alternating_with_load_and_current(self, writer);
        writer.write(&self.shaft_power_demand_id, self.shaft_power_demand().get::<watt>());
    }

    fn process_power_consumption_report<T: PowerConsumptionReport>(
        &mut self,
        _: &UpdateContext,
        report: &T,
    ) {
        self.real_power_output =
            report.total_consumption_of(PotentialOrigin::ApuGenerator(self.number));

        (self.output_potential, self.output_current) = if self.should_provide_output() {
            Self::calculate_potential_under_load(self.real_power_output)
        } else {
            (ElectricPotential::default(), ElectricCurrent::default())
        };

        self.output_frequency = if self.should_provide_output() {
            self.calculate_frequency(self.n)
        } else {
            Frequency::default()
        };

        let power_consumption = self.real_power_output.get::<watt>();
        let power_factor_correction = Self::POWER_FACTOR;
        let maximum_load = Self::MAXIMUM_LOAD_WATT;
        self.load = Ratio::new::<percent>(
            (power_consumption * power_factor_correction / maximum_load) * 100.,
        );
    }
}

#[derive(Clone)]
pub struct Pw980StartMotor {
    /// On the A380, the start motor is powered through the DC APU STARTING BUS.
    /// There are however additional contactors which open and close based on
    /// overhead panel push button positions. Therefore we cannot simply look
    /// at whether or not DC APU STARTING BUS is powered, but must instead handle
    /// potential coming in via those contactors.
    powered_by: ElectricalBusType,
    is_powered: bool,
    /// The real current `pw980_physics::Pw980Core` computed this tick from
    /// its DC motor circuit (bus voltage, armature resistance and the
    /// turbine's actual back-EMF at its actual speed) -- see
    /// `set_starter_current`. Replaces the previous 7th-order polynomial
    /// power-vs-elapsed-time curve, which had no causal link to whether
    /// the core was actually spinning.
    current: ElectricCurrent,
}
impl Pw980StartMotor {
    pub fn new(powered_by: ElectricalBusType) -> Self {
        Pw980StartMotor {
            powered_by,
            is_powered: false,
            current: ElectricCurrent::default(),
        }
    }
}
impl ApuStartMotor for Pw980StartMotor {
    fn is_powered(&self) -> bool {
        self.is_powered
    }

    fn set_starter_current(&mut self, current: ElectricCurrent) {
        self.current = current;
    }
}
impl SimulationElement for Pw980StartMotor {
    fn receive_power(&mut self, buses: &impl ElectricalBuses) {
        self.is_powered = buses.is_powered(self.powered_by);
    }

    fn consume_power<T: ConsumePower>(&mut self, _context: &UpdateContext, consumption: &mut T) {
        if self.is_powered && self.current.get::<ampere>() > 0. {
            // Same real DC circuit the torque balance used: P = V * I at
            // the starter bus voltage (`pw980_physics::STARTER_BUS_VOLTAGE_V`).
            let power = Power::new::<watt>(
                super::pw980_physics::STARTER_BUS_VOLTAGE_V * self.current.get::<ampere>(),
            );
            consumption.consume_from_bus(self.powered_by, power);
        }
    }
}

#[cfg(test)]
mod apu_generator_tests {
    use more_asserts::*;
    use ntest::assert_about_eq;
    use uom::si::frequency::hertz;

    use crate::simulation::InitContext;
    use crate::{
        apu::tests::test_bed_pw980 as test_bed_with,
        shared,
        simulation::test::{ElementCtorFn, SimulationTestBed, TestAircraft, TestBed},
    };

    use super::*;

    #[test]
    fn starts_without_output() {
        let test_bed = SimulationTestBed::from(ElementCtorFn(apu_generator));

        assert!(!test_bed
            .query_element_elec(|e, elec| { shared::PowerConsumptionReport::is_powered(elec, e) }));
    }

    #[test]
    fn when_apu_running_provides_output() {
        let mut test_bed = SimulationTestBed::from(ElementCtorFn(apu_generator));

        update_below_threshold(&mut test_bed);
        update_above_threshold(&mut test_bed);

        assert!(test_bed
            .query_element_elec(|e, elec| { shared::PowerConsumptionReport::is_powered(elec, e) }));
    }

    #[test]
    fn when_apu_shutdown_provides_no_output() {
        let mut test_bed = SimulationTestBed::from(ElementCtorFn(apu_generator));

        update_above_threshold(&mut test_bed);
        update_below_threshold(&mut test_bed);

        assert!(!test_bed
            .query_element_elec(|e, elec| { shared::PowerConsumptionReport::is_powered(elec, e) }));
    }

    #[test]
    fn from_n_84_provides_voltage() {
        let mut test_bed = test_bed_with().starting_apu();

        loop {
            test_bed = test_bed.run(Duration::from_millis(50));

            let n = test_bed.n().normal_value().unwrap().get::<percent>();
            if n > 84. {
                assert_gt!(test_bed.potential().get::<volt>(), 0.);
            }

            if (n - 100.).abs() < f64::EPSILON {
                break;
            }
        }
    }

    #[test]
    fn from_n_84_has_frequency() {
        let mut test_bed = test_bed_with().starting_apu();

        loop {
            test_bed = test_bed.run(Duration::from_millis(50));

            let n = test_bed.n().normal_value().unwrap().get::<percent>();
            if n > 84. {
                assert_gt!(test_bed.frequency().get::<hertz>(), 0.);
            }

            if (n - 100.).abs() < f64::EPSILON {
                break;
            }
        }
    }

    #[test]
    fn in_normal_conditions_when_n_100_voltage_114_or_115() {
        let mut test_bed = test_bed_with().running_apu();

        for _ in 0..100 {
            test_bed = test_bed.run(Duration::from_millis(50));

            let voltage = test_bed.potential().get::<volt>();
            assert!((114.0..=115.0).contains(&voltage))
        }
    }

    #[test]
    fn in_normal_conditions_when_n_100_frequency_400() {
        let mut test_bed = test_bed_with().running_apu();

        for _ in 0..100 {
            test_bed = test_bed.run(Duration::from_millis(50));

            let frequency = test_bed.frequency().get::<hertz>();
            assert_about_eq!(frequency, 400.);
        }
    }

    #[test]
    fn when_shutdown_frequency_not_normal() {
        let mut test_bed = test_bed_with().run(Duration::from_secs(1_000));

        assert!(!test_bed.frequency_within_normal_range());
    }

    #[test]
    fn when_running_frequency_normal() {
        let mut test_bed = test_bed_with()
            .running_apu()
            .run(Duration::from_secs(1_000));

        assert!(test_bed.frequency_within_normal_range());
    }

    #[test]
    fn when_shutdown_potential_not_normal() {
        let mut test_bed = test_bed_with().run(Duration::from_secs(1_000));

        assert!(!test_bed.potential_within_normal_range());
    }

    #[test]
    fn when_running_potential_normal() {
        let mut test_bed = test_bed_with()
            .running_apu()
            .run(Duration::from_secs(1_000));

        assert!(test_bed.potential_within_normal_range());
    }

    #[test]
    fn when_shutdown_has_no_load() {
        let mut test_bed = test_bed_with().run(Duration::from_secs(1_000));

        assert_eq!(test_bed.load(), Ratio::default());
    }

    #[test]
    fn when_running_but_potential_unused_has_no_load() {
        let mut test_bed = test_bed_with()
            .running_apu()
            .power_demand(Power::default())
            .run(Duration::from_secs(1_000));

        assert_eq!(test_bed.load(), Ratio::default());
    }

    #[test]
    fn when_running_and_potential_used_has_load() {
        let mut test_bed = test_bed_with()
            .running_apu()
            .power_demand(Power::new::<watt>(50000.))
            .run(Duration::from_secs(1_000));

        assert_gt!(test_bed.load(), Ratio::default());
    }

    #[test]
    fn when_load_below_maximum_it_is_normal() {
        let mut test_bed = test_bed_with()
            .running_apu()
            .power_demand(Power::new::<watt>(120000. / 0.8))
            .run(Duration::from_secs(1_000));

        assert!(test_bed.load_within_normal_range());
    }

    #[test]
    fn when_load_exceeds_maximum_not_normal() {
        let mut test_bed = test_bed_with()
            .running_apu()
            .power_demand(Power::new::<watt>((120000. / 0.8) + 1.))
            .run(Duration::from_secs(1_000));

        assert!(!test_bed.load_within_normal_range());
    }

    #[test]
    fn when_apu_emergency_shutdown_provides_no_output() {
        let test_bed = test_bed_with()
            .running_apu()
            .and()
            .released_apu_fire_pb()
            .run(Duration::from_secs(1));

        assert!(test_bed.generator_is_unpowered());
    }

    #[test]
    fn writes_its_state() {
        let mut test_bed = SimulationTestBed::from(ElementCtorFn(apu_generator));

        test_bed.run();

        assert!(test_bed.contains_variable_with_name("ELEC_APU_GEN_1_POTENTIAL"));
        assert!(test_bed.contains_variable_with_name("ELEC_APU_GEN_1_POTENTIAL_NORMAL"));
        assert!(test_bed.contains_variable_with_name("ELEC_APU_GEN_1_FREQUENCY"));
        assert!(test_bed.contains_variable_with_name("ELEC_APU_GEN_1_FREQUENCY_NORMAL"));
        assert!(test_bed.contains_variable_with_name("ELEC_APU_GEN_1_LOAD"));
        assert!(test_bed.contains_variable_with_name("ELEC_APU_GEN_1_LOAD_NORMAL"));
    }

    fn apu_generator(context: &mut InitContext) -> Pw980ApuGenerator {
        Pw980ApuGenerator::new(context, 1)
    }

    fn update_above_threshold(test_bed: &mut SimulationTestBed<TestAircraft<Pw980ApuGenerator>>) {
        test_bed.set_update_before_power_distribution(|generator, _, electricity| {
            generator.update(Ratio::new::<percent>(100.), false);
            electricity.supplied_by(generator);
        });
        test_bed.run();
    }

    fn update_below_threshold(test_bed: &mut SimulationTestBed<TestAircraft<Pw980ApuGenerator>>) {
        test_bed.set_update_before_power_distribution(|generator, _, electricity| {
            generator.update(Ratio::default(), false);
            electricity.supplied_by(generator);
        });
        test_bed.run();
    }
}
