use super::{
    ElectricalElement, ElectricalElementIdentifier, ElectricalElementIdentifierProvider,
    ElectricalStateWriter, ElectricitySource, EngineGeneratorPushButtons, Potential,
    PotentialOrigin, ProvideCurrent, ProvideFrequency, ProvideLoad, ProvidePotential,
};
use crate::{
    engine::Engine,
    failures::{Failure, FailureType},
    shared::{calculate_towards_target_temperature, EngineFirePushButtons, PowerConsumptionReport},
    simulation::{
        InitContext, Read, SimulationElement, SimulationElementVisitor, SimulatorReader,
        SimulatorWriter, UpdateContext, VariableIdentifier, Write,
    },
};
use std::{ops::RangeInclusive, time::Duration};
use uom::si::{
    angular_velocity::revolution_per_minute,
    electric_current::ampere,
    electric_potential::volt,
    f64::*,
    frequency::hertz,
    power::watt,
    ratio::{percent, ratio},
    thermodynamic_temperature::degree_celsius,
};

pub const INTEGRATED_DRIVE_GENERATOR_STABILIZATION_TIME: Duration = Duration::from_millis(500);

pub type IntegratedDriveGenerator = EngineGenerator<ConstantSpeedDrive>;
pub type VariableFrequencyGenerator = EngineGenerator<DirectDrive>;

pub trait EngineGeneratorDrive: SimulationElement {
    fn new_drive(context: &mut InitContext, number: usize) -> Self;
    fn update_drive(&mut self, context: &UpdateContext, engine: &impl Engine);
    fn output_speed(&self) -> AngularVelocity;
    fn disconnect(&mut self);
    fn is_connected(&self) -> bool;
}

#[derive(Clone)]
pub struct EngineGenerator<Drive: EngineGeneratorDrive> {
    writer: ElectricalStateWriter,
    /// The shared engine-load contract's `ENGINE_GEARBOX_ELEC_LOAD_W:n`
    /// input (docs/briefs/hyperrealism.md): written here so the plugin's
    /// engine model can read it as a plain simulator variable, the same way
    /// every other cross-workstream load is exchanged, without a Rust API
    /// across the plugin/systems boundary.
    shaft_power_demand_id: VariableIdentifier,
    number: usize,
    max_true_power: Power,
    identifier: ElectricalElementIdentifier,
    drive: Drive,
    activated: bool,
    output_frequency: Frequency,
    normal_frequency: RangeInclusive<f64>,
    output_potential: ElectricPotential,
    output_current: ElectricCurrent,
    /// Real electrical power delivered to the buses this tick (before the
    /// 0.8 power-factor correction `load` applies): the shared engine-load
    /// contract (`ENGINE_GEARBOX_ELEC_LOAD_W:n`) divides this by
    /// `GENERATOR_EFFICIENCY` to get the mechanical shaft power the VFG asks
    /// of the gearbox.
    real_power_output: Power,
    load: Ratio,
    time_above_threshold: Duration,
    failure: Failure,
    /// Real, time-integrated overload trip (fbw-xp-systems electrical-sources
    /// workstream, docs/physics/electrical.md): `output_within_normal_parameters`'s
    /// own doc comment said overload "over time will trigger a mechanical
    /// disconnect of the generator", but no such trip previously existed --
    /// only the `load_normal`/`current_normal` indication flags. `overload_heat`
    /// is the same normalised I^2t accumulator (trips at 1.0) and `K = 30`
    /// time constant the plugin's own breaker/SSPC curve already uses
    /// (`fbw-xp-systems/src/physics/electrical.rs::trip_step`,
    /// `THERMAL_TRIP_K`), reused here so both layers share one documented,
    /// sourced curve shape instead of inventing a second number. Once
    /// tripped the generator contactor stays open (`should_provide_output`
    /// below) until the GEN pushbutton is cycled off then on again (a real
    /// GCU overload trip is a manual-reset latch, not a momentary flag).
    overload_heat: f64,
    overload_tripped: bool,
    /// Continuous VFG/IDG degradation magnitudes, written by the plugin as
    /// plain simulator variables (the breaker-variable pattern this
    /// workstream's other degradation inputs use; both read 0 until the
    /// plugin writes them). `impedance_degradation` is in [0, 1] (0 = new
    /// winding) and scales `synchronous_reactance_ohm()` up towards
    /// `DEGRADED_REACTANCE_MULTIPLIER`x -- aged/damaged stator windings
    /// present a higher effective synchronous impedance. `regulator_drift`
    /// is in [-1, 1] (0 = perfectly trimmed GCU) and linearly shifts the
    /// GCU's regulated no-load target voltage by up to
    /// `MAX_REGULATOR_DRIFT_VOLT` either side of `RATED_VOLTAGE_VOLT` -- a
    /// drifted voltage regulator, a real and distinct GCU failure mode from
    /// winding impedance. Will move to `failures::magnitude(id)` once that
    /// lands.
    impedance_degradation_id: VariableIdentifier,
    impedance_degradation: f64,
    regulator_drift_id: VariableIdentifier,
    regulator_drift: f64,
}
impl<Drive: EngineGeneratorDrive> EngineGenerator<Drive> {
    /// Nominal (no-load) regulated line-neutral voltage FBW's own code
    /// already used as a flat constant (this file, pre-existing).
    const RATED_VOLTAGE_VOLT: f64 = 115.;
    const POWER_FACTOR: f64 = 0.8;
    /// A GCU-regulated brushless AC generator typically holds within a few
    /// percent of nominal up to rated load; FBW's code specifies no figure,
    /// so 3% at rated apparent power is a defensible derived estimate
    /// (electrical.md), chosen to stay inside FBW's own +-/normal band
    /// (`potential_normal`, 110-120 V for a 115 V nominal) at rated load,
    /// consistent with `output_within_normal_parameters`'s own doc comment
    /// that reaching 100% load should not by itself flip the generator
    /// abnormal (real overload trips on time-integrated overtemperature,
    /// not an instant voltage collapse). It sizes an equivalent synchronous
    /// reactance for Kirchhoff's-law terminal-voltage sag under load,
    /// replacing the previous flat 115 V regardless of current.
    const RATED_VOLTAGE_REGULATION: f64 = 0.03;
    /// Typical peak efficiency of a high-power brushless aircraft AC
    /// generator; FBW's code has no figure, so this is a derived/typical
    /// value (electrical.md) used only to turn electrical output power into
    /// the mechanical shaft power the engine model sees.
    const GENERATOR_EFFICIENCY: f64 = 0.88;
    /// Synchronous-reactance multiplier at maximum modelled
    /// `impedance_degradation` (magnitude 1.0): roughly 4x a healthy
    /// winding's reactance, a defensible order-of-magnitude estimate for
    /// significant stator winding damage/aging (no FBW/A380-specific
    /// degraded-generator figure exists; derived/typical,
    /// docs/physics/electrical.md).
    const DEGRADED_REACTANCE_MULTIPLIER: f64 = 4.0;
    /// Maximum GCU regulator-drift magnitude, applied at `regulator_drift`
    /// = +-1.0: a drifted-but-still-in-service regulator plausibly sits a
    /// few volts off its 115 V trim point before maintenance would catch it
    /// on the ground. Derived/typical (docs/physics/electrical.md); no
    /// FBW/A380-specific figure exists.
    const MAX_REGULATOR_DRIFT_VOLT: f64 = 8.0;

    pub fn new(
        context: &mut InitContext,
        number: usize,
        max_true_power: Power,
        normal_frequency: RangeInclusive<f64>,
    ) -> Self {
        EngineGenerator {
            writer: ElectricalStateWriter::new(context, &format!("ENG_GEN_{}", number)),
            shaft_power_demand_id: context
                .get_identifier(format!("ELEC_ENG_GEN_{}_SHAFT_POWER_DEMAND", number)),
            number,
            max_true_power,
            identifier: context.next_electrical_identifier(),
            drive: Drive::new_drive(context, number),
            activated: true,
            output_frequency: Frequency::new::<hertz>(0.),
            normal_frequency,
            output_potential: ElectricPotential::new::<volt>(0.),
            output_current: ElectricCurrent::new::<ampere>(0.),
            real_power_output: Power::new::<watt>(0.),
            load: Ratio::new::<percent>(0.),
            time_above_threshold: INTEGRATED_DRIVE_GENERATOR_STABILIZATION_TIME,
            failure: Failure::new(FailureType::Generator(number)),
            overload_heat: 0.,
            overload_tripped: false,
            impedance_degradation_id: context
                .get_identifier(format!("ELEC_ENG_GEN_{}_IMPEDANCE_DEGRADATION", number)),
            impedance_degradation: 0.,
            regulator_drift_id: context
                .get_identifier(format!("ELEC_ENG_GEN_{}_REGULATOR_DRIFT", number)),
            regulator_drift: 0.,
        }
    }

    /// This tick's GCU-regulated no-load target voltage: `RATED_VOLTAGE_VOLT`
    /// linearly shifted by the plugin-supplied `regulator_drift` magnitude
    /// times `MAX_REGULATOR_DRIFT_VOLT`. The [-1, 1] range is enforced once,
    /// with a loud log, at the substrate boundary the plugin writes this
    /// quantity through (`fbw-xp-systems/src/invariants.rs::bound_for`'s
    /// `REGULATOR_DRIFT` entry) -- not re-clamped silently here.
    fn regulated_voltage_volt(&self) -> f64 {
        Self::RATED_VOLTAGE_VOLT + self.regulator_drift * Self::MAX_REGULATOR_DRIFT_VOLT
    }

    /// Same I^2t curve as `fbw-xp-systems`'s own breaker/SSPC model
    /// (`trip_step`, `THERMAL_TRIP_K = 30`): at `ratio` = load/rated, trips
    /// once the accumulator reaches 1.0, which for a sustained 2x overload
    /// happens at `K/(2^2-1) = 10s` -- the same "overload ... over time"
    /// mechanical disconnect this file's own doc comment always described
    /// but never implemented. Cools at the same order of time constant while
    /// not overloaded, matching a real thermal element's own heat bleeding
    /// off.
    fn update_overload_trip(&mut self, context: &UpdateContext) {
        if !self.activated {
            // A GEN pushbutton cycle (off then on) is the real GCU's own
            // manual reset for an overload trip.
            self.overload_heat = 0.;
            self.overload_tripped = false;
            return;
        }
        const THERMAL_TRIP_K: f64 = 30.;
        const COOLDOWN_SECONDS: f64 = 20.;
        let load_ratio = self.load.get::<percent>() / 100.;
        let delta = context.delta_as_secs_f64();
        if load_ratio > 1. {
            self.overload_heat += delta * (load_ratio * load_ratio - 1.) / THERMAL_TRIP_K;
            if self.overload_heat >= 1. {
                self.overload_heat = 0.;
                self.overload_tripped = true;
            }
        } else {
            self.overload_heat = (self.overload_heat - delta / COOLDOWN_SECONDS).max(0.);
        }
    }

    /// Equivalent synchronous reactance sized so that the same quadratic
    /// (`V^2 - V_rated*V + S*Xs = 0`) used every tick below sags the
    /// terminal voltage to `V_rated*(1 - RATED_VOLTAGE_REGULATION)` at rated
    /// apparent power (`max_true_power`/`POWER_FACTOR`): Xs = V_target *
    /// (V_rated - V_target) / S_rated.
    fn synchronous_reactance_ohm(&self) -> f64 {
        let rated_apparent_power = self.max_true_power.get::<watt>() / Self::POWER_FACTOR;
        let target_voltage = Self::RATED_VOLTAGE_VOLT * (1. - Self::RATED_VOLTAGE_REGULATION);
        let healthy_reactance =
            target_voltage * (Self::RATED_VOLTAGE_VOLT - target_voltage) / rated_apparent_power;
        // Continuous winding degradation: linearly interpolate the reactance
        // multiplier from 1x (healthy) to `DEGRADED_REACTANCE_MULTIPLIER`x by
        // the plugin-supplied `impedance_degradation` magnitude. The [0, 1]
        // range is enforced once, with a loud log, at the substrate
        // boundary the plugin writes this quantity through
        // (`fbw-xp-systems/src/invariants.rs::bound_for`'s
        // `DEGRADATION_KEYWORDS` entry) -- not re-clamped silently here.
        let degradation_factor =
            1. + self.impedance_degradation * (Self::DEGRADED_REACTANCE_MULTIPLIER - 1.);
        healthy_reactance * degradation_factor
    }

    /// Real electrical power this generator delivered to the buses this
    /// tick: the basis for both `ENGINE_GEARBOX_ELEC_LOAD_W:n` (divided by
    /// efficiency for shaft power) and the equivalent-circuit current/sag
    /// calculation below.
    pub fn real_power_output(&self) -> Power {
        self.real_power_output
    }

    /// Mechanical shaft power the gearbox must supply to produce
    /// `real_power_output()` at `GENERATOR_EFFICIENCY`: the generator's half
    /// of the shared engine-load contract (docs/briefs/hyperrealism.md).
    pub fn shaft_power_demand(&self) -> Power {
        self.real_power_output / Self::GENERATOR_EFFICIENCY
    }

    pub fn update(
        &mut self,
        context: &UpdateContext,
        engine: &impl Engine,
        generator_buttons: &impl EngineGeneratorPushButtons,
        fire_buttons: &impl EngineFirePushButtons,
    ) {
        if generator_buttons.idg_push_button_is_released(self.number) {
            // The drive cannot be reconnected.
            self.drive.disconnect();
        }
        self.activated = generator_buttons.engine_gen_push_button_is_on(self.number)
            && !fire_buttons.is_released(self.number);
        self.drive.update_drive(context, engine);
        self.output_frequency = if self.activated {
            Frequency::new::<hertz>(
                self.drive.output_speed().get::<revolution_per_minute>() * 4. / 120.,
            )
        } else {
            Frequency::default()
        };
        self.update_stable_time(context);
    }

    // TODO: move to GCU when implemented
    fn update_stable_time(&mut self, context: &UpdateContext) {
        if !self.activated {
            self.time_above_threshold = Duration::ZERO;
            return;
        }

        let new_time = if self.frequency_normal() {
            self.time_above_threshold + context.delta()
        } else {
            Duration::ZERO
        };

        self.time_above_threshold = new_time.clamp(
            Duration::ZERO,
            INTEGRATED_DRIVE_GENERATOR_STABILIZATION_TIME,
        );
    }

    fn provides_stable_power_output(&self) -> bool {
        self.time_above_threshold == INTEGRATED_DRIVE_GENERATOR_STABILIZATION_TIME
    }

    /// Indicates if the provided electricity's potential and frequency
    /// are within normal parameters. Use this to decide if the
    /// generator contactor should close.
    /// Load shouldn't be taken into account, as overloading causes an
    /// overtemperature which over time will trigger a mechanical
    /// disconnect of the generator.
    pub fn output_within_normal_parameters(&self) -> bool {
        self.should_provide_output() && self.potential_normal()
    }

    fn should_provide_output(&self) -> bool {
        self.provides_stable_power_output()
            && self.activated
            && self.frequency_normal()
            && !self.failure.is_active()
            && !self.overload_tripped
    }

    pub fn is_drive_connected(&self) -> bool {
        self.drive.is_connected()
    }
}
impl<Drive: EngineGeneratorDrive> ElectricitySource for EngineGenerator<Drive> {
    fn output_potential(&self) -> Potential {
        if self.should_provide_output() {
            Potential::new(
                PotentialOrigin::EngineGenerator(self.number),
                self.output_potential,
            )
        } else {
            Potential::none()
        }
    }
}
// TODO: Move to GCU
impl<Drive: EngineGeneratorDrive> ProvidePotential for EngineGenerator<Drive> {
    fn potential(&self) -> ElectricPotential {
        self.output_potential
    }
    fn potential_normal(&self) -> bool {
        let volts = self.output_potential.get::<volt>();
        (110.0..=120.0).contains(&volts)
    }
}
// TODO: Move to GCU
impl<Drive: EngineGeneratorDrive> ProvideFrequency for EngineGenerator<Drive> {
    fn frequency(&self) -> Frequency {
        self.output_frequency
    }
    fn frequency_normal(&self) -> bool {
        let hz = self.output_frequency.get::<hertz>();
        self.normal_frequency.contains(&hz)
    }
}
impl<Drive: EngineGeneratorDrive> ProvideCurrent for EngineGenerator<Drive> {
    fn current(&self) -> ElectricCurrent {
        self.output_current
    }

    fn current_normal(&self) -> bool {
        // Rated current at the nameplate power factor; beyond this the GCU's
        // overload protection would trip the generator field, matching
        // `ProvideLoad::load_normal` below (100% of `max_true_power`).
        let rated_current =
            (self.max_true_power.get::<watt>() / Self::POWER_FACTOR) / Self::RATED_VOLTAGE_VOLT;
        self.output_current.get::<ampere>().abs() <= rated_current
    }
}
// TODO: Move to GCU
impl<Drive: EngineGeneratorDrive> ProvideLoad for EngineGenerator<Drive> {
    fn load(&self) -> Ratio {
        self.load
    }
    fn load_normal(&self) -> bool {
        self.load <= Ratio::new::<percent>(100.)
    }
}
impl<Drive: EngineGeneratorDrive> ElectricalElement for EngineGenerator<Drive> {
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
impl<Drive: EngineGeneratorDrive> SimulationElement for EngineGenerator<Drive> {
    fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
        self.drive.accept(visitor);
        self.failure.accept(visitor);

        visitor.visit(self);
    }

    fn read(&mut self, reader: &mut SimulatorReader) {
        self.impedance_degradation = reader.read(&self.impedance_degradation_id);
        self.regulator_drift = reader.read(&self.regulator_drift_id);
    }

    fn process_power_consumption_report<T: PowerConsumptionReport>(
        &mut self,
        context: &UpdateContext,
        report: &T,
    ) {
        self.real_power_output =
            report.total_consumption_of(PotentialOrigin::EngineGenerator(self.number));

        // Load (and the overload trip it drives) is computed from this
        // tick's real power *before* deciding whether to provide output, so
        // a trip this tick really does cut this same tick's output instead
        // of lagging a tick behind.
        let power_factor_correction = Ratio::new::<ratio>(Self::POWER_FACTOR);
        self.load = self.real_power_output * power_factor_correction / self.max_true_power;
        self.update_overload_trip(context);

        self.output_potential = if self.should_provide_output() {
            // Kirchhoff's loop across the equivalent synchronous reactance:
            // V = V_rated - I*Xs with I = S/V (S the apparent power at the
            // nameplate power factor), i.e. V^2 - V_rated*V + S*Xs = 0 (the
            // same quadratic-in-V technique transformer_rectifier.rs and
            // battery.rs use for their own output-impedance solves). This
            // replaces the previous flat 115 V regardless of current, and
            // self-limits at the reactance's own maximum power transfer
            // instead of allowing an unreal current at very low voltage.
            let apparent_power = self.real_power_output.get::<watt>() / Self::POWER_FACTOR;
            let xs = self.synchronous_reactance_ohm();
            let v_target = self.regulated_voltage_volt();
            let discriminant = v_target * v_target - 4. * apparent_power * xs;
            let sagged = if discriminant < 0. {
                v_target / 2.
            } else {
                (v_target + discriminant.sqrt()) / 2.
            };
            self.output_current = ElectricCurrent::new::<ampere>(apparent_power / sagged);
            ElectricPotential::new::<volt>(sagged)
        } else {
            self.output_current = ElectricCurrent::new::<ampere>(0.);
            ElectricPotential::new::<volt>(0.)
        };
    }

    fn write(&self, writer: &mut SimulatorWriter) {
        self.writer.write_alternating_with_load_and_current(self, writer);
        writer.write(&self.shaft_power_demand_id, self.shaft_power_demand().get::<watt>());
        writer.write(&self.impedance_degradation_id, self.impedance_degradation);
        writer.write(&self.regulator_drift_id, self.regulator_drift);
    }
}

#[derive(Clone)]
pub struct ConstantSpeedDrive {
    oil_outlet_temperature_id: VariableIdentifier,
    oil_outlet_temperature: ThermodynamicTemperature,
    is_connected_id: VariableIdentifier,
    connected: bool,
    output_speed: AngularVelocity,
}
impl ConstantSpeedDrive {
    // Threshold to reach target output speed = 58% of 16645 RPM
    pub const ENGINE_GEARBOX_POWER_UP_OUTPUT_THRESHOLD: f64 = 0.58 * 16645.;
    const OUTPUT_SPEED_RPM: f64 = 12000.;

    const M: f64 = Self::OUTPUT_SPEED_RPM / Self::ENGINE_GEARBOX_POWER_UP_OUTPUT_THRESHOLD;

    fn new(context: &mut InitContext, number: usize) -> ConstantSpeedDrive {
        ConstantSpeedDrive {
            oil_outlet_temperature_id: context.get_identifier(format!(
                "ELEC_ENG_GEN_{}_IDG_OIL_OUTLET_TEMPERATURE",
                number
            )),
            oil_outlet_temperature: ThermodynamicTemperature::new::<degree_celsius>(0.),
            is_connected_id: context
                .get_identifier(format!("ELEC_ENG_GEN_{}_IDG_IS_CONNECTED", number)),
            connected: true,
            output_speed: AngularVelocity::default(),
        }
    }

    pub fn update(&mut self, context: &UpdateContext, engine: &impl Engine) {
        self.output_speed = if self.connected {
            (Self::M * engine.gearbox_speed()).min(AngularVelocity::new::<revolution_per_minute>(
                Self::OUTPUT_SPEED_RPM,
            ))
        } else {
            AngularVelocity::default()
        };
        self.update_temperature(
            context,
            self.get_target_temperature(context, engine.corrected_n2()),
        );
    }

    fn update_temperature(&mut self, context: &UpdateContext, target: ThermodynamicTemperature) {
        const IDG_HEATING_COEFFICIENT: f64 = 1.4;
        const IDG_COOLING_COEFFICIENT: f64 = 0.4;

        self.oil_outlet_temperature = calculate_towards_target_temperature(
            self.oil_outlet_temperature,
            target,
            if self.oil_outlet_temperature < target {
                IDG_HEATING_COEFFICIENT
            } else {
                IDG_COOLING_COEFFICIENT
            },
            context.delta(),
        );
    }

    fn get_target_temperature(
        &self,
        context: &UpdateContext,
        corrected_n2: Ratio,
    ) -> ThermodynamicTemperature {
        const TEMPERATURE_TO_RPM_FACTOR: f64 = 1.8;

        if !self.connected {
            return context.ambient_temperature();
        }

        let ambient_temperature = context.ambient_temperature().get::<degree_celsius>();
        let target_idg =
            corrected_n2.get::<percent>() * TEMPERATURE_TO_RPM_FACTOR + ambient_temperature;

        // TODO improve this function with feedback @komp provides.

        ThermodynamicTemperature::new::<degree_celsius>(target_idg)
    }
}
impl EngineGeneratorDrive for ConstantSpeedDrive {
    fn new_drive(context: &mut InitContext, number: usize) -> Self {
        Self::new(context, number)
    }

    fn update_drive(&mut self, context: &UpdateContext, engine: &impl Engine) {
        self.update(context, engine);
    }

    fn output_speed(&self) -> AngularVelocity {
        self.output_speed
    }

    fn disconnect(&mut self) {
        self.connected = false;
    }

    fn is_connected(&self) -> bool {
        self.connected
    }
}
impl SimulationElement for ConstantSpeedDrive {
    fn write(&self, writer: &mut SimulatorWriter) {
        writer.write(&self.oil_outlet_temperature_id, self.oil_outlet_temperature);
        writer.write(&self.is_connected_id, self.connected);
    }
}

#[derive(Clone)]
pub struct DirectDrive {
    oil_outlet_temperature_id: VariableIdentifier,
    oil_outlet_temperature: ThermodynamicTemperature,
    is_connected_id: VariableIdentifier,
    connected: bool,
    output_speed: AngularVelocity,
}
impl DirectDrive {
    const TRANSMISSION_RATIO: f64 = 1.95;

    fn new(context: &mut InitContext, number: usize) -> Self {
        Self {
            oil_outlet_temperature_id: context.get_identifier(format!(
                "ELEC_ENG_GEN_{}_IDG_OIL_OUTLET_TEMPERATURE",
                number
            )),
            oil_outlet_temperature: ThermodynamicTemperature::new::<degree_celsius>(0.),
            is_connected_id: context
                .get_identifier(format!("ELEC_ENG_GEN_{}_IDG_IS_CONNECTED", number)),
            connected: true,
            output_speed: AngularVelocity::default(),
        }
    }

    fn update_temperature(&mut self, context: &UpdateContext, target: ThermodynamicTemperature) {
        const IDG_HEATING_COEFFICIENT: f64 = 1.4;
        const IDG_COOLING_COEFFICIENT: f64 = 0.4;

        self.oil_outlet_temperature = calculate_towards_target_temperature(
            self.oil_outlet_temperature,
            target,
            if self.oil_outlet_temperature < target {
                IDG_HEATING_COEFFICIENT
            } else {
                IDG_COOLING_COEFFICIENT
            },
            context.delta(),
        );
    }

    fn get_target_temperature(
        &self,
        context: &UpdateContext,
        corrected_n2: Ratio,
    ) -> ThermodynamicTemperature {
        const TEMPERATURE_TO_RPM_FACTOR: f64 = 1.8;

        if !self.connected {
            return context.ambient_temperature();
        }

        let ambient_temperature = context.ambient_temperature().get::<degree_celsius>();
        let target_idg =
            corrected_n2.get::<percent>() * TEMPERATURE_TO_RPM_FACTOR + ambient_temperature;

        // TODO improve this function with feedback @komp provides.

        ThermodynamicTemperature::new::<degree_celsius>(target_idg)
    }
}
impl EngineGeneratorDrive for DirectDrive {
    fn new_drive(context: &mut InitContext, number: usize) -> Self {
        Self::new(context, number)
    }

    fn update_drive(&mut self, context: &UpdateContext, engine: &impl Engine) {
        self.output_speed = if self.connected {
            engine.gearbox_speed() * Self::TRANSMISSION_RATIO
        } else {
            AngularVelocity::default()
        };

        self.update_temperature(
            context,
            self.get_target_temperature(context, engine.corrected_n2()),
        )
    }

    fn output_speed(&self) -> AngularVelocity {
        self.output_speed
    }

    fn disconnect(&mut self) {
        self.connected = false;
    }

    fn is_connected(&self) -> bool {
        self.connected
    }
}
impl SimulationElement for DirectDrive {
    fn write(&self, writer: &mut SimulatorWriter) {
        writer.write(&self.oil_outlet_temperature_id, self.oil_outlet_temperature);
        writer.write(&self.is_connected_id, self.connected);
    }
}

#[cfg(test)]
mod tests {
    use more_asserts::*;

    use super::*;
    use crate::shared::{EngineCorrectedN1, EngineCorrectedN2, EngineUncorrectedN2};

    struct TestEngine {
        corrected_n2: Ratio,
    }
    impl TestEngine {
        fn new(engine_corrected_n2: Ratio) -> Self {
            Self {
                corrected_n2: engine_corrected_n2,
            }
        }
    }
    impl EngineCorrectedN1 for TestEngine {
        fn corrected_n1(&self) -> Ratio {
            unimplemented!()
        }
    }
    impl EngineCorrectedN2 for TestEngine {
        fn corrected_n2(&self) -> Ratio {
            self.corrected_n2
        }
    }
    impl EngineUncorrectedN2 for TestEngine {
        fn uncorrected_n2(&self) -> Ratio {
            unimplemented!()
        }
    }
    impl Engine for TestEngine {
        fn hydraulic_pump_output_speed(&self) -> AngularVelocity {
            unimplemented!()
        }

        fn oil_pressure_is_low(&self) -> bool {
            unimplemented!()
        }

        fn is_above_minimum_idle(&self) -> bool {
            unimplemented!()
        }

        fn net_thrust(&self) -> Mass {
            unimplemented!()
        }

        fn gearbox_speed(&self) -> AngularVelocity {
            AngularVelocity::new::<revolution_per_minute>(self.corrected_n2.get::<ratio>() * 16000.)
        }
    }

    struct TestOverhead {
        engine_gen_push_button_is_on: bool,
        idg_push_button_is_released: bool,
    }
    impl TestOverhead {
        fn new(engine_gen_push_button_is_on: bool, idg_push_button_is_released: bool) -> Self {
            Self {
                engine_gen_push_button_is_on,
                idg_push_button_is_released,
            }
        }
    }
    impl EngineGeneratorPushButtons for TestOverhead {
        fn engine_gen_push_button_is_on(&self, _: usize) -> bool {
            self.engine_gen_push_button_is_on
        }

        fn idg_push_button_is_released(&self, _: usize) -> bool {
            self.idg_push_button_is_released
        }
    }

    struct TestFireOverhead {
        engine_fire_push_button_is_released: bool,
    }
    impl TestFireOverhead {
        fn new(engine_fire_push_button_is_released: bool) -> Self {
            Self {
                engine_fire_push_button_is_released,
            }
        }
    }
    impl EngineFirePushButtons for TestFireOverhead {
        fn is_released(&self, _: usize) -> bool {
            self.engine_fire_push_button_is_released
        }
    }

    #[cfg(test)]
    mod engine_generator_tests {
        use super::*;
        use crate::{
            electrical::{
                consumption::PowerConsumer, ElectricalBus, ElectricalBusType, Electricity,
            },
            simulation::{
                test::{ReadByName, SimulationTestBed, TestBed, WriteByName},
                Aircraft, InitContext,
            },
        };
        use uom::si::power::{kilowatt, watt};

        struct EngineGeneratorTestBed {
            test_bed: SimulationTestBed<TestAircraft>,
        }
        impl EngineGeneratorTestBed {
            fn with_running_engine() -> Self {
                Self {
                    test_bed: SimulationTestBed::new(TestAircraft::with_running_engine),
                }
            }

            fn with_shutdown_engine() -> Self {
                Self {
                    test_bed: SimulationTestBed::new(TestAircraft::with_shutdown_engine),
                }
            }

            fn frequency_is_normal(&mut self) -> bool {
                self.read_by_name("ELEC_ENG_GEN_1_FREQUENCY_NORMAL")
            }

            fn potential_is_normal(&mut self) -> bool {
                self.read_by_name("ELEC_ENG_GEN_1_POTENTIAL_NORMAL")
            }

            fn load_is_normal(&mut self) -> bool {
                self.read_by_name("ELEC_ENG_GEN_1_LOAD_NORMAL")
            }

            fn load(&mut self) -> Ratio {
                self.read_by_name("ELEC_ENG_GEN_1_LOAD")
            }

            fn potential(&mut self) -> ElectricPotential {
                self.read_by_name("ELEC_ENG_GEN_1_POTENTIAL")
            }

            fn current(&mut self) -> ElectricCurrent {
                self.read_by_name("ELEC_ENG_GEN_1_CURRENT")
            }

            fn shaft_power_demand(&self) -> Power {
                self.query(|a| a.shaft_power_demand())
            }

            fn real_power_output(&self) -> Power {
                self.query(|a| a.real_power_output())
            }

            fn generator_is_powered(&mut self) -> bool {
                self.query_elec(|a, elec| a.generator_is_powered(elec))
            }

            fn generator_provides_stable_power_output(&self) -> bool {
                self.query(|a| a.generator_output_within_normal_parameters())
            }
        }
        impl TestBed for EngineGeneratorTestBed {
            type Aircraft = TestAircraft;

            fn test_bed(&self) -> &SimulationTestBed<TestAircraft> {
                &self.test_bed
            }

            fn test_bed_mut(&mut self) -> &mut SimulationTestBed<TestAircraft> {
                &mut self.test_bed
            }
        }

        struct TestAircraft {
            engine_gen: IntegratedDriveGenerator,
            bus: ElectricalBus,
            running: bool,
            gen_push_button_on: bool,
            idg_push_button_released: bool,
            fire_push_button_released: bool,
            consumer: PowerConsumer,
            generator_output_within_normal_parameters_before_processing_power_consumption_report:
                bool,
        }
        impl TestAircraft {
            fn new(running: bool, context: &mut InitContext) -> Self {
                Self {
                    engine_gen: IntegratedDriveGenerator::new(context, 1, Power::new::<kilowatt>(90.), 390.0..=410.0),
                    bus: ElectricalBus::new(context, ElectricalBusType::AlternatingCurrent(1)),
                    running,
                    gen_push_button_on: true,
                    idg_push_button_released: false,
                    fire_push_button_released: false,
                    consumer: PowerConsumer::from(ElectricalBusType::AlternatingCurrent(1)),
                    generator_output_within_normal_parameters_before_processing_power_consumption_report: false
                }
            }

            fn with_shutdown_engine(context: &mut InitContext) -> Self {
                TestAircraft::new(false, context)
            }

            fn with_running_engine(context: &mut InitContext) -> Self {
                TestAircraft::new(true, context)
            }

            fn disconnect_idg(&mut self) {
                self.idg_push_button_released = true;
            }

            fn gen_push_button_off(&mut self) {
                self.gen_push_button_on = false;
            }

            fn release_fire_push_button(&mut self) {
                self.fire_push_button_released = true;
            }

            fn generator_is_powered(&self, electricity: &Electricity) -> bool {
                electricity.is_powered(&self.engine_gen)
            }

            fn power_demand(&mut self, power: Power) {
                self.consumer.demand(power);
            }

            fn generator_output_within_normal_parameters_after_processing_power_consumption_report(
                &self,
            ) -> bool {
                self.generator_output_within_normal_parameters()
            }

            fn shutdown_engine(&mut self) {
                self.running = false;
            }

            fn start_engine(&mut self) {
                self.running = true;
            }

            fn generator_output_within_normal_parameters_before_processing_power_consumption_report(
                &self,
            ) -> bool {
                self.generator_output_within_normal_parameters_before_processing_power_consumption_report
            }

            fn generator_output_within_normal_parameters(&self) -> bool {
                self.engine_gen.output_within_normal_parameters()
            }

            fn shaft_power_demand(&self) -> Power {
                self.engine_gen.shaft_power_demand()
            }

            fn real_power_output(&self) -> Power {
                self.engine_gen.real_power_output()
            }
        }
        impl Aircraft for TestAircraft {
            fn update_before_power_distribution(
                &mut self,
                context: &UpdateContext,
                electricity: &mut Electricity,
            ) {
                self.engine_gen.update(
                    context,
                    &TestEngine::new(Ratio::new::<percent>(if self.running { 80. } else { 0. })),
                    &TestOverhead::new(self.gen_push_button_on, self.idg_push_button_released),
                    &TestFireOverhead::new(self.fire_push_button_released),
                );
                electricity.supplied_by(&self.engine_gen);
                electricity.flow(&self.engine_gen, &self.bus);

                self.generator_output_within_normal_parameters_before_processing_power_consumption_report = self.engine_gen.output_within_normal_parameters();
            }
        }
        impl SimulationElement for TestAircraft {
            fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
                self.engine_gen.accept(visitor);
                self.consumer.accept(visitor);

                visitor.visit(self);
            }
        }

        #[test]
        fn starts_unstable_with_engines_off() {
            let mut test_bed = EngineGeneratorTestBed::with_shutdown_engine();
            test_bed.run_without_delta();

            assert!(!test_bed.generator_provides_stable_power_output());
        }

        #[test]
        fn starts_stable_with_engines_on() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();
            test_bed.run_without_delta();

            assert!(test_bed.generator_provides_stable_power_output());
        }

        #[test]
        fn becomes_stable_once_engine_above_threshold_for_500_milliseconds() {
            // First enforcing engine in off state
            let mut test_bed = EngineGeneratorTestBed::with_shutdown_engine();
            test_bed.run_without_delta();

            test_bed.command(|a| a.start_engine());
            test_bed.run_with_delta(Duration::from_millis(500));

            assert!(test_bed.generator_provides_stable_power_output());
        }

        #[test]
        fn does_not_become_stable_before_engine_above_threshold_for_500_milliseconds() {
            // First enforcing engine in off state
            let mut test_bed = EngineGeneratorTestBed::with_shutdown_engine();
            test_bed.run_without_delta();

            test_bed.command(|a| a.start_engine());
            test_bed.run_with_delta(Duration::from_millis(499));

            assert!(!test_bed.generator_provides_stable_power_output());
        }

        #[test]
        fn when_engine_running_provides_output() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();
            test_bed.run();

            assert!(test_bed.generator_is_powered());
        }

        #[test]
        fn when_engine_shutdown_provides_no_output() {
            let mut test_bed = EngineGeneratorTestBed::with_shutdown_engine();
            test_bed.run();

            assert!(!test_bed.generator_is_powered());
        }

        #[test]
        fn when_engine_running_but_idg_disconnected_provides_no_output() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();

            test_bed.command(|a| a.disconnect_idg());
            test_bed.run();

            assert!(!test_bed.generator_is_powered());
        }

        #[test]
        fn when_engine_running_but_generator_off_provides_no_output() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();

            test_bed.command(|a| a.gen_push_button_off());
            test_bed.run();

            assert!(!test_bed.generator_is_powered());
        }

        #[test]
        fn when_engine_running_but_fire_push_button_released_provides_no_output() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();

            test_bed.command(|a| a.release_fire_push_button());
            test_bed.run();

            assert!(!test_bed.generator_is_powered());
        }

        #[test]
        fn when_engine_shutdown_frequency_not_normal() {
            let mut test_bed = EngineGeneratorTestBed::with_shutdown_engine();
            test_bed.run();

            assert!(!test_bed.frequency_is_normal());
        }

        #[test]
        fn when_engine_running_but_idg_disconnected_frequency_not_normal() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();

            test_bed.command(|a| a.disconnect_idg());
            test_bed.run();

            assert!(!test_bed.frequency_is_normal());
        }

        #[test]
        fn when_engine_running_but_generator_off_frequency_not_normal() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();

            test_bed.command(|a| a.gen_push_button_off());
            test_bed.run();

            assert!(!test_bed.frequency_is_normal());
        }

        #[test]
        fn when_engine_running_but_fire_push_button_released_frequency_not_normal() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();

            test_bed.command(|a| a.release_fire_push_button());
            test_bed.run();

            assert!(!test_bed.frequency_is_normal());
        }

        #[test]
        fn when_engine_running_frequency_normal() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();
            test_bed.run();

            assert!(test_bed.frequency_is_normal());
        }

        #[test]
        fn when_engine_shutdown_potential_not_normal() {
            let mut test_bed = EngineGeneratorTestBed::with_shutdown_engine();
            test_bed.run();

            assert!(!test_bed.potential_is_normal());
        }

        #[test]
        fn when_engine_running_but_idg_disconnected_potential_not_normal() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();

            test_bed.command(|a| a.disconnect_idg());
            test_bed.run();

            assert!(!test_bed.potential_is_normal());
        }

        #[test]
        fn when_engine_running_but_generator_off_provides_potential_not_normal() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();

            test_bed.command(|a| a.gen_push_button_off());
            test_bed.run();

            assert!(!test_bed.potential_is_normal());
        }

        #[test]
        fn when_engine_running_but_fire_push_button_released_potential_not_normal() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();

            test_bed.command(|a| a.release_fire_push_button());
            test_bed.run();

            assert!(!test_bed.potential_is_normal());
        }

        #[test]
        fn when_engine_running_potential_normal() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();
            test_bed.run();

            assert!(test_bed.potential_is_normal());
        }

        #[test]
        fn when_engine_shutdown_has_no_load() {
            let mut test_bed = EngineGeneratorTestBed::with_shutdown_engine();
            test_bed.run();

            assert_eq!(test_bed.load(), Ratio::new::<percent>(0.));
        }

        #[test]
        fn when_engine_running_but_idg_disconnected_has_no_load() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();

            test_bed.command(|a| a.disconnect_idg());
            test_bed.run();

            assert_eq!(test_bed.load(), Ratio::new::<percent>(0.));
        }

        #[test]
        fn when_engine_running_but_generator_off_has_no_load() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();

            test_bed.command(|a| a.gen_push_button_off());
            test_bed.run();

            assert_eq!(test_bed.load(), Ratio::new::<percent>(0.));
        }

        #[test]
        fn when_engine_running_but_fire_push_button_released_has_no_load() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();

            test_bed.command(|a| a.release_fire_push_button());
            test_bed.run();

            assert_eq!(test_bed.load(), Ratio::new::<percent>(0.));
        }

        #[test]
        fn when_engine_running_but_potential_unused_has_no_load() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();
            test_bed.run();

            assert_eq!(test_bed.load(), Ratio::new::<percent>(0.));
        }

        #[test]
        fn when_engine_running_and_potential_used_has_load() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();

            test_bed.command(|a| a.power_demand(Power::new::<watt>(50000.)));
            test_bed.run();

            assert_gt!(test_bed.load(), Ratio::new::<percent>(0.));
        }

        #[test]
        fn when_load_below_maximum_it_is_normal() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();

            test_bed.command(|a| a.power_demand(Power::new::<watt>(90000. / 0.8)));
            test_bed.run();

            assert!(test_bed.load_is_normal());
        }

        #[test]
        fn when_load_exceeds_maximum_not_normal() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();

            test_bed.command(|a| a.power_demand(Power::new::<watt>((90000. / 0.8) + 1.)));
            test_bed.run();

            assert!(!test_bed.load_is_normal());
        }

        #[test]
        fn output_within_normal_parameters_when_load_exceeds_maximum() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();

            test_bed.command(|a| a.power_demand(Power::new::<watt>((90000. / 0.8) + 1.)));

            test_bed.run();

            assert!(test_bed.query(|a| a.generator_output_within_normal_parameters_after_processing_power_consumption_report()));
        }

        #[test]
        fn output_not_within_normal_parameters_when_engine_not_running() {
            let mut test_bed = EngineGeneratorTestBed::with_shutdown_engine();
            test_bed.run();

            assert!(!test_bed.query(|a| a.generator_output_within_normal_parameters_after_processing_power_consumption_report()));
        }

        #[test]
        fn output_within_normal_parameters_when_engine_running() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();
            test_bed.run();

            assert!(test_bed.query(|a| a.generator_output_within_normal_parameters_after_processing_power_consumption_report()));
        }

        #[test]
        fn output_within_normal_parameters_adapts_to_shutting_down_idg_instantaneously() {
            // The frequency and potential of the generator are only known at the end of a tick,
            // due to them being directly related to the power consumption (large changes can cause
            // spikes and dips). However, the decision if a generator can supply power is made much
            // earlier in the tick. This is especially of great consequence when the IDG no longer
            // supplies potential but the previous tick's frequency and potential are still normal.
            // With this test we ensure that an IDG which is no longer supplying power is
            // immediately noticed and doesn't require another tick.
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();
            test_bed.run();

            test_bed.command(|a| a.shutdown_engine());

            test_bed.run();

            assert!(!test_bed.query(|a| a.generator_output_within_normal_parameters_before_processing_power_consumption_report()));
        }

        #[test]
        fn writes_its_state() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();
            test_bed.run();

            assert!(test_bed.contains_variable_with_name("ELEC_ENG_GEN_1_POTENTIAL"));
            assert!(test_bed.contains_variable_with_name("ELEC_ENG_GEN_1_POTENTIAL_NORMAL"));
            assert!(test_bed.contains_variable_with_name("ELEC_ENG_GEN_1_FREQUENCY"));
            assert!(test_bed.contains_variable_with_name("ELEC_ENG_GEN_1_FREQUENCY_NORMAL"));
            assert!(test_bed.contains_variable_with_name("ELEC_ENG_GEN_1_LOAD"));
            assert!(test_bed.contains_variable_with_name("ELEC_ENG_GEN_1_LOAD_NORMAL"));
            assert!(test_bed.contains_variable_with_name("ELEC_ENG_GEN_1_CURRENT"));
            assert!(test_bed.contains_variable_with_name("ELEC_ENG_GEN_1_CURRENT_NORMAL"));
        }

        #[test]
        fn terminal_voltage_sags_more_under_a_heavier_load() {
            // Kirchhoff's loop across the equivalent synchronous reactance
            // (V = V_rated - I*Xs): a heavier load must sag the terminal
            // voltage further, unlike the old flat-115V-regardless-of-load
            // model.
            let mut light_load = EngineGeneratorTestBed::with_running_engine();
            light_load.command(|a| a.power_demand(Power::new::<watt>(10_000.)));
            light_load.run();
            let light_load_potential = light_load.potential();

            let mut heavy_load = EngineGeneratorTestBed::with_running_engine();
            heavy_load.command(|a| a.power_demand(Power::new::<watt>(90_000.)));
            heavy_load.run();
            let heavy_load_potential = heavy_load.potential();

            assert_lt!(heavy_load_potential, light_load_potential);
            assert!(light_load_potential.get::<volt>() <= 115.);
        }

        #[test]
        fn current_scales_with_real_power_delivered() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();
            test_bed.command(|a| a.power_demand(Power::new::<watt>(50_000.)));
            test_bed.run();

            assert_gt!(test_bed.current(), ElectricCurrent::new::<ampere>(0.));
        }

        #[test]
        fn shaft_power_demand_is_real_power_output_over_efficiency() {
            // The generator's half of the shared engine-load contract
            // (ENGINE_GEARBOX_ELEC_LOAD_W:n, docs/briefs/hyperrealism.md):
            // the mechanical shaft power the gearbox must supply always
            // exceeds the real electrical power delivered, by the
            // generator's own conversion losses.
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();
            test_bed.command(|a| a.power_demand(Power::new::<watt>(50_000.)));
            test_bed.run();

            let electrical = test_bed.real_power_output();
            let shaft = test_bed.shaft_power_demand();
            assert_gt!(electrical.get::<watt>(), 0.);
            assert_gt!(shaft, electrical);
            assert!((shaft.get::<watt>() / electrical.get::<watt>() - 1. / 0.88).abs() < 1e-9);
        }

        #[test]
        fn no_load_means_no_shaft_power_demand() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();
            test_bed.run();

            assert_eq!(test_bed.shaft_power_demand(), Power::new::<watt>(0.));
        }

        /// The intersection test (fbw-xp-systems electrical-sources
        /// workstream): two *independent* continuous degradation
        /// parameters on the same generator -- `impedance_degradation`
        /// (winding aging) and `regulator_drift` (GCU setpoint aging) --
        /// each individually held at a magnitude that stays inside the
        /// normal 110-120 V band, but whose combination, run through this
        /// file's own pre-existing Kirchhoff quadratic solve
        /// (`process_power_consumption_report`, unmodified by this test),
        /// sags the output below 110 V. Nothing here scripts an
        /// undervoltage outcome; both magnitudes are just perturbations to
        /// `synchronous_reactance_ohm()`/`regulated_voltage_volt()` that
        /// feed the same `V = (V_target + sqrt(V_target^2 - 4*S*Xs)) / 2`
        /// solve every other test in this file also exercises.
        ///
        /// Hand derivation (done before running the sim; see the
        /// accompanying Python check that produced these exact figures from
        /// the same formula, `docs/physics/electrical.md`):
        ///   max_true_power = 90 kW (this test rig's IDG, see `TestAircraft::new`)
        ///   rated_apparent_power = 90000 / 0.8 = 112500 VA
        ///   target_voltage = 115 * (1 - 0.03) = 111.55 V
        ///   Xs0 = 111.55 * (115 - 111.55) / 112500 = 0.0034208666... ohm
        ///   demand = 40000 W => apparent power S = 40000 / 0.8 = 50000 VA
        ///
        ///   V(imped_mag, drift) = (v_target + sqrt(v_target^2 - 4*S*Xs)) / 2
        ///     where Xs = Xs0 * (1 + 3*imped_mag), v_target = 115 + 8*drift
        ///
        ///   baseline        (0.0,  0.0): V = 113.493 V  (normal)
        ///   impedance only  (0.5,  0.0): V = 111.153 V  (normal)
        ///   drift only      (0.0, -0.3): V = 111.060 V  (normal)
        ///   combined        (0.5, -0.3): V = 108.665 V  (ABNORMAL, < 110 V)
        #[test]
        fn combined_impedance_degradation_and_regulator_drift_sag_voltage_below_normal_band() {
            let mut test_bed = EngineGeneratorTestBed::with_running_engine();
            test_bed.write_by_name("ELEC_ENG_GEN_1_IMPEDANCE_DEGRADATION", 0.5);
            test_bed.write_by_name("ELEC_ENG_GEN_1_REGULATOR_DRIFT", -0.3);
            test_bed.command(|a| a.power_demand(Power::new::<watt>(40_000.)));
            test_bed.run();

            let volts = test_bed.potential().get::<volt>();
            assert!(
                (volts - 108.665).abs() < 0.01,
                "expected ~108.665 V from the hand-derived quadratic, got {volts}"
            );
            assert!(!test_bed.potential_is_normal());
        }

        #[test]
        /// Decoupling proof for the intersection test above: with either
        /// one of the two magnitudes cut back to 0 (the coupling severed)
        /// while the other is held at the same value the combined test
        /// used, the undervoltage vanishes and the generator reports
        /// normal again -- confirming the abnormal voltage above is a
        /// genuine emergent consequence of *both* perturbations reaching
        /// the same real solve together, not an artifact of either one
        /// alone (or of the test itself).
        fn cutting_either_degradation_link_restores_normal_voltage() {
            let mut impedance_only = EngineGeneratorTestBed::with_running_engine();
            impedance_only.write_by_name("ELEC_ENG_GEN_1_IMPEDANCE_DEGRADATION", 0.5);
            impedance_only.write_by_name("ELEC_ENG_GEN_1_REGULATOR_DRIFT", 0.0);
            impedance_only.command(|a| a.power_demand(Power::new::<watt>(40_000.)));
            impedance_only.run();
            assert!(
                (impedance_only.potential().get::<volt>() - 111.153).abs() < 0.01,
                "impedance-only link: got {}",
                impedance_only.potential().get::<volt>()
            );
            assert!(impedance_only.potential_is_normal());

            let mut drift_only = EngineGeneratorTestBed::with_running_engine();
            drift_only.write_by_name("ELEC_ENG_GEN_1_IMPEDANCE_DEGRADATION", 0.0);
            drift_only.write_by_name("ELEC_ENG_GEN_1_REGULATOR_DRIFT", -0.3);
            drift_only.command(|a| a.power_demand(Power::new::<watt>(40_000.)));
            drift_only.run();
            assert!(
                (drift_only.potential().get::<volt>() - 111.060).abs() < 0.01,
                "drift-only link: got {}",
                drift_only.potential().get::<volt>()
            );
            assert!(drift_only.potential_is_normal());
        }
    }

    #[cfg(test)]
    mod engine_generator_drive_tests {
        use super::*;
        use crate::simulation::test::{ElementCtorFn, SimulationTestBed, TestBed};
        use rstest::rstest;
        use std::time::Duration;

        trait OilOutletTemperature {
            fn get_oil_outlet_temperature(&self) -> ThermodynamicTemperature;
        }
        impl OilOutletTemperature for ConstantSpeedDrive {
            fn get_oil_outlet_temperature(&self) -> ThermodynamicTemperature {
                self.oil_outlet_temperature
            }
        }
        impl OilOutletTemperature for DirectDrive {
            fn get_oil_outlet_temperature(&self) -> ThermodynamicTemperature {
                self.oil_outlet_temperature
            }
        }

        fn idg(context: &mut InitContext) -> ConstantSpeedDrive {
            ConstantSpeedDrive::new(context, 1)
        }

        fn vfg(context: &mut InitContext) -> DirectDrive {
            DirectDrive::new(context, 1)
        }

        #[rstest]
        #[case(idg)]
        #[case(vfg)]
        fn writes_its_state<T: SimulationElement>(#[case] drive: fn(&mut InitContext) -> T) {
            let mut test_bed = SimulationTestBed::from(ElementCtorFn(drive));
            test_bed.run();

            assert!(
                test_bed.contains_variable_with_name("ELEC_ENG_GEN_1_IDG_OIL_OUTLET_TEMPERATURE")
            );
            assert!(test_bed.contains_variable_with_name("ELEC_ENG_GEN_1_IDG_IS_CONNECTED"));
        }

        #[rstest]
        #[case(idg)]
        #[case(vfg)]
        fn running_engine_warms_up_drive<
            T: SimulationElement + EngineGeneratorDrive + OilOutletTemperature + 'static,
        >(
            #[case] drive: fn(&mut InitContext) -> T,
        ) {
            let mut test_bed = SimulationTestBed::from(ElementCtorFn(drive))
                .with_update_after_power_distribution(engine_running_above_threshold(false));

            let starting_temperature = test_bed.query_element(|e| e.get_oil_outlet_temperature());

            test_bed.run_with_delta(Duration::from_secs(10));

            assert!(
                test_bed.query_element(|e| e.get_oil_outlet_temperature()) > starting_temperature
            );
        }

        #[rstest]
        #[case(idg)]
        #[case(vfg)]
        fn running_engine_does_not_warm_up_drive_when_disconnected<
            T: SimulationElement + EngineGeneratorDrive + OilOutletTemperature + 'static,
        >(
            #[case] drive: fn(&mut InitContext) -> T,
        ) {
            let mut test_bed = SimulationTestBed::from(ElementCtorFn(drive))
                .with_update_after_power_distribution(engine_running_above_threshold(true));

            let starting_temperature = test_bed.query_element(|e| e.get_oil_outlet_temperature());

            test_bed.run_with_delta(Duration::from_secs(10));

            assert_eq!(
                test_bed.query_element(|e| e.get_oil_outlet_temperature()),
                starting_temperature
            );
        }

        #[rstest]
        #[case(idg)]
        #[case(vfg)]
        fn shutdown_engine_cools_down_drive<
            T: SimulationElement + EngineGeneratorDrive + OilOutletTemperature + 'static,
        >(
            #[case] drive: fn(&mut InitContext) -> T,
        ) {
            let mut test_bed = SimulationTestBed::from(ElementCtorFn(drive))
                .with_update_after_power_distribution(engine_running_above_threshold(false));
            test_bed.run_with_delta(Duration::from_secs(10));

            let starting_temperature = test_bed.query_element(|e| e.get_oil_outlet_temperature());

            test_bed.set_update_after_power_distribution(engine_not_running);
            test_bed.run_with_delta(Duration::from_secs(10));

            assert!(
                test_bed.query_element(|e| e.get_oil_outlet_temperature()) < starting_temperature
            );
        }

        #[rstest]
        #[case(idg)]
        #[case(vfg)]
        fn cannot_reconnect_once_disconnected<
            T: SimulationElement + EngineGeneratorDrive + 'static,
        >(
            #[case] drive: fn(&mut InitContext) -> T,
        ) {
            let mut test_bed = SimulationTestBed::from(ElementCtorFn(drive))
                .with_update_after_power_distribution(engine_running_above_threshold(true));
            test_bed.run_with_delta(Duration::from_millis(500));

            test_bed.set_update_after_power_distribution(engine_running_above_threshold(false));
            test_bed.run_with_delta(Duration::from_millis(500));

            assert!(!test_bed.query_element(|e| e.is_connected()));
        }

        fn engine_not_running(drive: &mut impl EngineGeneratorDrive, context: &UpdateContext) {
            drive.update_drive(context, &TestEngine::new(Ratio::new::<percent>(0.)))
        }

        fn engine_running_above_threshold<T: EngineGeneratorDrive>(
            idg_push_button_is_released: bool,
        ) -> impl Fn(&mut T, &UpdateContext) {
            move |drive: &mut T, context: &UpdateContext| {
                if idg_push_button_is_released {
                    drive.disconnect()
                }
                drive.update_drive(context, &TestEngine::new(Ratio::new::<percent>(80.)))
            }
        }
    }
}
