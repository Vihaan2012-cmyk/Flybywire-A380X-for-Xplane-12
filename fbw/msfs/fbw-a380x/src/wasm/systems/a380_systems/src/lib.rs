extern crate systems;

#[cfg(test)]
#[global_allocator]
static GLOBAL: mimalloc::MiMalloc = mimalloc::MiMalloc;

mod air_conditioning;
mod airframe;
mod autoflight;
mod apu;
mod avionics_data_communication_network;
mod control_display_system;
mod electrical;
mod failures;
mod fire_and_smoke_protection;
mod fuel;
pub mod hydraulic;
mod icing;
mod navigation;
mod oxygen;
mod payload;
mod pneumatic;
mod power_consumption;
mod deep_systems;
pub use failures::fbw_failures;
mod reverser;
mod structural_flex;

use self::{
    air_conditioning::{A380AirConditioning, A380PressurizationOverheadPanel},
    autoflight::DeepAutoflightAuthority,
    avionics_data_communication_network::A380AvionicsDataCommunicationNetwork,
    control_display_system::A380ControlDisplaySystem,
    fuel::A380Fuel,
    pneumatic::{A380Pneumatic, A380PneumaticOverheadPanel},
    structural_flex::A380StructuralFlex,
};
use airframe::A380Airframe;
use apu::DeepApuAuthority;
use avionics_data_communication_network::A380AvionicsDataCommunicationNetworkSimvarTranslator;
use electrical::{
    A380Electrical, A380ElectricalOverheadPanel, A380EmergencyElectricalOverheadPanel,
    APU_START_MOTOR_BUS_TYPE,
};
use fire_and_smoke_protection::A380FireAndSmokeProtection;
use hydraulic::{autobrakes::A380AutobrakePanel, A380Hydraulic, A380HydraulicOverheadPanel};
use deep_systems::DeepSystemsHost;
use icing::Icing;
use navigation::{A380AirDataInertialReferenceSystemBuilder, A380RadioAltimeters};
use oxygen::A380Oxygen;
use payload::A380Payload;
use power_consumption::A380PowerConsumption;
use reverser::{A380ReverserController, A380Reversers};
use uom::si::{f64::Length, length::nautical_mile};

use systems::{
    accept_iterable,
    apu::{
        AuxiliaryPowerUnit, AuxiliaryPowerUnitFactory, AuxiliaryPowerUnitFireOverheadPanel,
        AuxiliaryPowerUnitOverheadPanel, Pw980ApuGenerator, Pw980Constants, Pw980StartMotor,
    },
    electrical::{Electricity, ElectricitySource, ExternalPowerSource},
    engine::{reverser_thrust::ReverserForce, trent_engine::TrentEngine, EngineFireOverheadPanel},
    enhanced_gpwc::EnhancedGroundProximityWarningComputer,
    landing_gear::{LandingGear, LandingGearControlInterfaceUnitSet},
    navigation::adirs::{
        AirDataInertialReferenceSystem, AirDataInertialReferenceSystemOverheadPanel,
    },
    shared::ElectricalBusType,
    simulation::{
        Aircraft, InitContext, SimulationElement, SimulationElementVisitor, UpdateContext,
    },
};

pub struct A380 {
    adcn: A380AvionicsDataCommunicationNetwork,
    adcn_simvar_translation: A380AvionicsDataCommunicationNetworkSimvarTranslator,
    adirs: AirDataInertialReferenceSystem,
    adirs_overhead: AirDataInertialReferenceSystemOverheadPanel,
    air_conditioning: A380AirConditioning,
    apu: AuxiliaryPowerUnit<Pw980ApuGenerator, Pw980StartMotor, Pw980Constants, 2>,
    deep_apu_authority: DeepApuAuthority,
    apu_fire_overhead: AuxiliaryPowerUnitFireOverheadPanel,
    apu_overhead: AuxiliaryPowerUnitOverheadPanel,
    pneumatic_overhead: A380PneumaticOverheadPanel,
    pressurization_overhead: A380PressurizationOverheadPanel,
    electrical_overhead: A380ElectricalOverheadPanel,
    emergency_electrical_overhead: A380EmergencyElectricalOverheadPanel,
    payload: A380Payload,
    airframe: A380Airframe,
    fire_and_smoke_protection: A380FireAndSmokeProtection,
    fuel: A380Fuel,
    engine_1: TrentEngine,
    engine_2: TrentEngine,
    engine_3: TrentEngine,
    engine_4: TrentEngine,
    engine_fire_overhead: EngineFireOverheadPanel<4>,
    electrical: A380Electrical,
    power_consumption: A380PowerConsumption,
    ext_pwrs: [ExternalPowerSource; 4],
    lgcius: LandingGearControlInterfaceUnitSet,
    hydraulic: A380Hydraulic,
    hydraulic_overhead: A380HydraulicOverheadPanel,
    autobrake_panel: A380AutobrakePanel,
    landing_gear: LandingGear,
    pneumatic: A380Pneumatic,
    radio_altimeters: A380RadioAltimeters,
    cds: A380ControlDisplaySystem,
    egpwc: EnhancedGroundProximityWarningComputer,
    icing_simulation: Icing,
    structural_flex: A380StructuralFlex,
    deep_autoflight_authority: DeepAutoflightAuthority,

    engine_reverser_control: [A380ReverserController; 2],
    reversers_assembly: A380Reversers,
    reverse_thrust: ReverserForce,

    oxygen: A380Oxygen,

    deep_systems: DeepSystemsHost,
}
impl A380 {
    pub fn new(context: &mut InitContext) -> A380 {
        let mut adcn = A380AvionicsDataCommunicationNetwork::new(context);
        let adcn_simvar_translation =
            A380AvionicsDataCommunicationNetworkSimvarTranslator::new(context, &mut adcn);
        A380 {
            adcn,
            adcn_simvar_translation,
            adirs: A380AirDataInertialReferenceSystemBuilder::build(context),
            adirs_overhead: AirDataInertialReferenceSystemOverheadPanel::new(context),
            air_conditioning: A380AirConditioning::new(context),
            apu: AuxiliaryPowerUnitFactory::new_pw980(
                context,
                APU_START_MOTOR_BUS_TYPE,
                ElectricalBusType::DirectCurrentEssential,
                ElectricalBusType::DirectCurrentEssential,
            ),
            deep_apu_authority: DeepApuAuthority::new(context),
            apu_fire_overhead: AuxiliaryPowerUnitFireOverheadPanel::new(context),
            apu_overhead: AuxiliaryPowerUnitOverheadPanel::new(context),
            pneumatic_overhead: A380PneumaticOverheadPanel::new(context),
            pressurization_overhead: A380PressurizationOverheadPanel::new(context),
            electrical_overhead: A380ElectricalOverheadPanel::new(context),
            emergency_electrical_overhead: A380EmergencyElectricalOverheadPanel::new(context),
            payload: A380Payload::new(context),
            airframe: A380Airframe::new(context),
            fire_and_smoke_protection: A380FireAndSmokeProtection::new(context),
            fuel: A380Fuel::new(context),
            engine_1: TrentEngine::new(context, 1),
            engine_2: TrentEngine::new(context, 2),
            engine_3: TrentEngine::new(context, 3),
            engine_4: TrentEngine::new(context, 4),
            engine_fire_overhead: EngineFireOverheadPanel::new(context),
            electrical: A380Electrical::new(context),
            power_consumption: A380PowerConsumption::new(context),
            ext_pwrs: [1, 2, 3, 4].map(|i| ExternalPowerSource::new(context, i)),
            lgcius: LandingGearControlInterfaceUnitSet::new(
                context,
                ElectricalBusType::DirectCurrentEssential,
                ElectricalBusType::DirectCurrentGndFltService,
            ),
            hydraulic: A380Hydraulic::new(context),
            hydraulic_overhead: A380HydraulicOverheadPanel::new(context),
            autobrake_panel: A380AutobrakePanel::new(context),
            landing_gear: LandingGear::new(context, true),
            pneumatic: A380Pneumatic::new(context),
            radio_altimeters: A380RadioAltimeters::new(context),
            cds: A380ControlDisplaySystem::new(context),
            egpwc: EnhancedGroundProximityWarningComputer::new(
                context,
                ElectricalBusType::AlternatingCurrentEssential,
                vec![
                    Length::new::<nautical_mile>(0.0),
                    Length::new::<nautical_mile>(10.0),
                    Length::new::<nautical_mile>(20.0),
                    Length::new::<nautical_mile>(40.0),
                    Length::new::<nautical_mile>(80.0),
                    Length::new::<nautical_mile>(160.0),
                    Length::new::<nautical_mile>(320.0),
                    Length::new::<nautical_mile>(640.0),
                ],
                3,
            ),

            icing_simulation: Icing::new(context),
            structural_flex: A380StructuralFlex::new(context),
            deep_autoflight_authority: DeepAutoflightAuthority::new(context),
            engine_reverser_control: [
                A380ReverserController::new(context, 2),
                A380ReverserController::new(context, 3),
            ],
            reversers_assembly: A380Reversers::new(context),
            reverse_thrust: ReverserForce::new(context),

            oxygen: A380Oxygen::new(context),

            deep_systems: DeepSystemsHost::new(context),
        }
    }
}
impl Aircraft for A380 {
    fn update_before_power_distribution(
        &mut self,
        context: &UpdateContext,
        electricity: &mut Electricity,
    ) {
        self.deep_apu_authority.apply(&mut self.apu);
        self.apu.update_before_electrical(
            context,
            &self.apu_overhead,
            self.fire_and_smoke_protection.apu_fire_on_ground(),
            &self.apu_fire_overhead,
            self.pneumatic_overhead.apu_bleed_is_on(),
            // This will be replaced when integrating the whole electrical system.
            // For now we use the same logic as found in the JavaScript code; ignoring whether or not
            // the engine generators are supplying electricity.
            (self.electrical_overhead.apu_generator_is_on(1)
                || self.electrical_overhead.apu_generator_is_on(2))
                && !(self.electrical_overhead.external_power_is_on(1)
                    && self.electrical_overhead.external_power_is_available(1)),
            self.pneumatic.apu_bleed_air_valve(),
            self.fuel.apu_has_fuel(),
        );

        self.electrical.update(
            context,
            electricity,
            &self.ext_pwrs,
            &self.electrical_overhead,
            &self.emergency_electrical_overhead,
            &mut self.apu,
            &self.engine_fire_overhead,
            [
                &self.engine_1,
                &self.engine_2,
                &self.engine_3,
                &self.engine_4,
            ],
            self.lgcius.lgciu1(),
            &self.adirs,
        );

        self.electrical_overhead
            .update_after_electrical(&self.electrical, electricity);
        self.emergency_electrical_overhead
            .update_after_electrical(context, &self.electrical);
        self.payload.update(context);
        self.airframe
            .update(&self.fuel, &self.payload, &self.payload);
    }

    fn update_after_power_distribution(&mut self, context: &UpdateContext) {
        self.apu.update_after_power_distribution(
            &[
                &self.engine_1,
                &self.engine_2,
                &self.engine_3,
                &self.engine_4,
            ],
            [self.lgcius.lgciu1(), self.lgcius.lgciu2()],
        );
        self.apu_overhead.update_after_apu(&self.apu);

        self.adcn.update();
        self.adcn_simvar_translation.update(&self.adcn);
        self.lgcius.update(
            context,
            &self.landing_gear,
            self.hydraulic.gear_system(),
            self.ext_pwrs[0].output_potential().is_powered(),
        );

        self.fire_and_smoke_protection.update(
            context,
            &self.engine_fire_overhead,
            [self.lgcius.lgciu1(), self.lgcius.lgciu2()],
        );

        self.radio_altimeters.update(context);

        self.hydraulic.update(
            context,
            [
                &self.engine_1,
                &self.engine_2,
                &self.engine_3,
                &self.engine_4,
            ],
            &self.hydraulic_overhead,
            &self.autobrake_panel,
            &self.engine_fire_overhead,
            &self.lgcius,
            &self.pneumatic,
            &self.adirs,
        );

        self.pneumatic.update_hydraulic_reservoir_spatial_volumes(
            self.hydraulic.green_reservoir(),
            self.hydraulic.yellow_reservoir(),
        );

        self.hydraulic_overhead.update(&self.hydraulic);

        self.adirs.update(context, &self.adirs_overhead);
        self.adirs_overhead.update(context, &self.adirs);

        self.power_consumption.update(context);

        self.pneumatic.update(
            context,
            [
                &self.engine_1,
                &self.engine_2,
                &self.engine_3,
                &self.engine_4,
            ],
            &self.pneumatic_overhead,
            &self.engine_fire_overhead,
            &self.apu,
            &self.air_conditioning,
        );
        self.air_conditioning
            .mix_packs_air_update(self.pneumatic.packs());
        self.air_conditioning.update(
            context,
            &self.adirs,
            &self.hydraulic,
            &self.adcn,
            [
                &self.engine_1,
                &self.engine_2,
                &self.engine_3,
                &self.engine_4,
            ],
            &self.engine_fire_overhead,
            &self.payload,
            &self.pneumatic,
            &self.pneumatic_overhead,
            &self.pressurization_overhead,
            [self.lgcius.lgciu1(), self.lgcius.lgciu2()],
        );

        self.cds.update();

        self.egpwc.update(&self.adirs, self.lgcius.lgciu1());

        self.structural_flex.update(
            context,
            [
                self.hydraulic.left_elevator_aero_torques(),
                self.hydraulic.right_elevator_aero_torques(),
            ],
            self.hydraulic.up_down_rudder_aero_torques(),
            &self.hydraulic,
            &self.fuel,
        );
        self.cds.update();

        self.icing_simulation.update(context);

        self.egpwc.update(&self.adirs, self.lgcius.lgciu1());
        self.fuel
            .update(context, &self.adcn, A380Airframe::get_loadsheet());

        self.engine_reverser_control[0].update(
            &self.engine_2,
            self.lgcius.lgciu1(),
            self.reversers_assembly.reverser_feedback(0),
        );
        self.engine_reverser_control[1].update(
            &self.engine_3,
            self.lgcius.lgciu2(),
            self.reversers_assembly.reverser_feedback(1),
        );

        self.reversers_assembly
            .update(context, &self.engine_reverser_control);

        self.reverse_thrust.update(
            context,
            [&self.engine_2, &self.engine_3],
            self.reversers_assembly.reversers_position(),
        );

        self.oxygen.update(context);

        self.deep_systems.update(context);
    }

    fn derived_failure_ids(&self) -> Vec<u64> {
        self.deep_systems.derived_failure_ids()
    }
}
impl SimulationElement for A380 {
    fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
        self.adcn.accept(visitor);
        self.adcn_simvar_translation.accept(visitor);
        self.adirs.accept(visitor);
        self.adirs_overhead.accept(visitor);
        self.air_conditioning.accept(visitor);
        self.apu.accept(visitor);
        self.deep_apu_authority.accept(visitor);
        self.apu_fire_overhead.accept(visitor);
        self.apu_overhead.accept(visitor);
        self.electrical_overhead.accept(visitor);
        self.emergency_electrical_overhead.accept(visitor);
        self.fire_and_smoke_protection.accept(visitor);
        self.fuel.accept(visitor);
        self.payload.accept(visitor);
        self.airframe.accept(visitor);
        self.pneumatic_overhead.accept(visitor);
        self.pressurization_overhead.accept(visitor);
        self.engine_1.accept(visitor);
        self.engine_2.accept(visitor);
        self.engine_3.accept(visitor);
        self.engine_4.accept(visitor);
        self.engine_fire_overhead.accept(visitor);
        self.electrical.accept(visitor);
        self.power_consumption.accept(visitor);
        accept_iterable!(self.ext_pwrs, visitor);
        self.lgcius.accept(visitor);
        self.radio_altimeters.accept(visitor);
        self.autobrake_panel.accept(visitor);
        self.hydraulic.accept(visitor);
        self.hydraulic_overhead.accept(visitor);
        self.landing_gear.accept(visitor);
        self.pneumatic.accept(visitor);
        self.cds.accept(visitor);
        self.egpwc.accept(visitor);
        self.icing_simulation.accept(visitor);
        self.structural_flex.accept(visitor);

        accept_iterable!(self.engine_reverser_control, visitor);
        self.reversers_assembly.accept(visitor);
        self.reverse_thrust.accept(visitor);
        self.oxygen.accept(visitor);

        self.deep_systems.accept(visitor);

        self.deep_autoflight_authority.accept(visitor);

        visitor.visit(self);
    }
}

#[cfg(test)]
mod whole_aircraft_tests {
    use super::*;
    use std::time::Duration;
    use systems::{
        shared::{arinc429::Arinc429Word, InternationalStandardAtmosphere},
        simulation::{
            test::{ReadByName, SimulationTestBed, TestBed, WriteByName},
            Aircraft,
        },
    };
    use uom::si::{
        f64::*,
        length::foot,
        pressure::psi,
        velocity::knot,
    };

    const BUSES: [&str; 6] = ["AC_1", "AC_2", "AC_3", "AC_4", "DC_1", "DC_ESS"];

    fn tripped(test_bed: &SimulationTestBed<A380>) -> Vec<u64> {
        test_bed.query(|a| a.derived_failure_ids())
    }

    fn unpowered(test_bed: &mut SimulationTestBed<A380>) -> Vec<&'static str> {
        BUSES
            .into_iter()
            .filter(|bus| !ReadByName::<_, bool>::read_by_name(test_bed, &format!("ELEC_{bus}_BUS_IS_POWERED")))
            .collect()
    }

    fn not_finite(test_bed: &mut SimulationTestBed<A380>) -> Vec<&'static str> {
        [
            "COND_PACK_1_OUTLET_TEMPERATURE",
            "COND_PACK_2_OUTLET_TEMPERATURE",
            "COND_MAIN_DECK_1_TEMP",
        ]
        .into_iter()
        .filter(|name| !ReadByName::<_, f64>::read_by_name(test_bed, name).is_finite())
        .collect()
    }

    fn fly_at(test_bed: &mut SimulationTestBed<A380>, altitude: Length, tas_kt: f64) {
        test_bed.set_pressure_altitude(altitude);
        test_bed.set_ambient_pressure(InternationalStandardAtmosphere::pressure_at_altitude(altitude));
        test_bed.set_ambient_temperature(InternationalStandardAtmosphere::temperature_at_altitude(altitude));
        test_bed.set_true_airspeed(Velocity::new::<knot>(tas_kt));
        test_bed.set_indicated_airspeed(Velocity::new::<knot>(tas_kt.min(300.)));
    }

    fn cabin(test_bed: &mut SimulationTestBed<A380>) -> (f64, f64) {
        let altitude: Arinc429Word<Length> = test_bed.read_arinc429_by_name("PRESS_CABIN_ALTITUDE_B1");
        let delta: Arinc429Word<Pressure> = test_bed.read_arinc429_by_name("PRESS_CABIN_DELTA_PRESSURE_B1");
        (altitude.value().get::<foot>(), delta.value().get::<psi>())
    }

    fn engines(test_bed: &mut SimulationTestBed<A380>, n1: f64, n2: f64, n3: f64) {
        for n in 1..=4 {
            test_bed.write_by_name(&format!("ENGINE_STATE:{n}"), 1.);
            test_bed.write_by_name(&format!("TURB ENG CORRECTED N1:{n}"), n1);
            test_bed.write_by_name(&format!("TURB ENG CORRECTED N2:{n}"), n2);
            test_bed.write_by_name(&format!("ENGINE_N2:{n}"), n2);
            test_bed.write_by_name(&format!("ENGINE_N3:{n}"), n3);
        }
    }

    fn phase(test_bed: &mut SimulationTestBed<A380>, name: &str, seconds: u64, powered: bool) {
        test_bed.run_multiple_frames(Duration::from_secs(seconds));
        let (tripped, unpowered, not_finite) = (tripped(test_bed), unpowered(test_bed), not_finite(test_bed));
        let pack_c: f64 = test_bed.read_by_name("COND_PACK_1_OUTLET_TEMPERATURE");
        let cabin_c: f64 = test_bed.read_by_name("COND_MAIN_DECK_1_TEMP");
        let (cabin_ft, delta_psi) = cabin(test_bed);
        println!("{name}: tripped {tripped:?}, unpowered {unpowered:?}, pack 1 outlet {pack_c:.1} C, cabin {cabin_c:.1} C, cabin altitude {cabin_ft:.0} ft, delta p {delta_psi:.2} psi");
        assert!(tripped.is_empty(), "{name}: breakers tripped with nothing wrong: {tripped:?}");
        assert!(not_finite.is_empty(), "{name}: not a number: {not_finite:?}");
        if powered {
            assert!(unpowered.is_empty(), "{name}: buses unpowered: {unpowered:?}");
        }
    }

    #[test]
    fn a_whole_flight_trips_nothing_and_powers_every_main_bus() {
        let mut test_bed = SimulationTestBed::new(A380::new);
        test_bed.set_on_ground(true);
        test_bed.write_by_name("GEAR_HANDLE_POSITION", 1.0);
        for id in 1..=4 {
            test_bed.write_by_name(&format!("OVHD_ELEC_BAT_{id}_PB_IS_AUTO"), true);
        }
        phase(&mut test_bed, "cold and dark", 60, false);

        for i in 1..=4 {
            test_bed.write_by_name(&format!("EXT_PWR_AVAIL:{i}"), true);
            test_bed.write_by_name(&format!("OVHD_ELEC_EXT_PWR_{i}_PB_IS_ON"), true);
        }
        test_bed.write_by_name("CONFIG_ADIRS_IR_ALIGN_TIME", 1.);
        for n in 1..=3 {
            test_bed.write_by_name(&format!("OVHD_ADIRS_IR_{n}_MODE_SELECTOR_KNOB"), 1.);
        }
        phase(&mut test_bed, "external power", 600, true);

        engines(&mut test_bed, 20., 65., 70.);
        for i in 1..=4 {
            test_bed.write_by_name(&format!("OVHD_ELEC_EXT_PWR_{i}_PB_IS_ON"), false);
            test_bed.write_by_name(&format!("EXT_PWR_AVAIL:{i}"), false);
        }
        phase(&mut test_bed, "engines at idle", 600, true);

        test_bed.set_on_ground(false);
        engines(&mut test_bed, 85., 95., 97.);
        for minute in 1..=20 {
            fly_at(&mut test_bed, Length::new::<foot>(1750. * minute as f64), 250. + 10. * minute as f64);
            test_bed.run_multiple_frames(Duration::from_secs(60));
        }
        fly_at(&mut test_bed, Length::new::<foot>(35000.), 490.);
        phase(&mut test_bed, "cruise FL350", 1200, true);
        let (cabin_ft, delta_psi) = cabin(&mut test_bed);
        assert!(cabin_ft < 8000., "cruise cabin altitude {cabin_ft} ft");
        assert!((7. ..9.).contains(&delta_psi), "cruise differential pressure {delta_psi} psi");
    }
}
