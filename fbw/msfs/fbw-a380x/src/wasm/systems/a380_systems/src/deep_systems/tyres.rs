use std::collections::BTreeMap;

use deep_systems::physics::tyre::{
    leg_of_wheel, TyreWheel, BRAKED_WHEELS, LEG_FAILURE_IDS, NOSE_FAILURE_ID, UNBRAKED_WHEELS,
    WHEELS,
};
use systems::simulation::{
    InitContext, Read, SimulatorReader, SimulatorWriter, UpdateContext, VariableIdentifier, Write,
};
use uom::si::{thermodynamic_temperature::degree_celsius, velocity::meter_per_second};

pub(super) struct Tyres {
    wheels: [TyreWheel; WHEELS],
    brake_temperature_id: [VariableIdentifier; BRAKED_WHEELS],
    brake_temperature_c: [f64; BRAKED_WHEELS],
    pressure_id: [VariableIdentifier; WHEELS],
    temperature_id: [VariableIdentifier; WHEELS],
    leaked_id: [VariableIdentifier; WHEELS],
    tread_id: [VariableIdentifier; WHEELS],
    fuse_plug_id: [VariableIdentifier; WHEELS],
    burst_wheel: BTreeMap<u64, usize>,
}

impl Tyres {
    pub(super) fn new(context: &mut InitContext) -> Self {
        let per_wheel = |context: &mut InitContext, name: &str| -> [VariableIdentifier; WHEELS] {
            std::array::from_fn(|i| context.get_identifier(format!("{name}:{}", i + 1)))
        };
        Self {
            wheels: [TyreWheel::default(); WHEELS],
            brake_temperature_id: std::array::from_fn(|i| {
                context.get_identifier(format!("REPORTED_BRAKE_TEMPERATURE_{}", i + 1))
            }),
            brake_temperature_c: [0.; BRAKED_WHEELS],
            pressure_id: per_wheel(context, "TYRE_PRESSURE_PA"),
            temperature_id: per_wheel(context, "TYRE_TEMPERATURE_C"),
            leaked_id: per_wheel(context, "TYRE_LEAKED_FRACTION"),
            tread_id: per_wheel(context, "TYRE_TREAD_MM"),
            fuse_plug_id: per_wheel(context, "TYRE_FUSE_PLUG_MELTED"),
            burst_wheel: BTreeMap::new(),
        }
    }

    pub(super) fn failure_ids() -> impl Iterator<Item = u64> {
        std::iter::once(NOSE_FAILURE_ID).chain(LEG_FAILURE_IDS)
    }

    pub(super) fn read(&mut self, reader: &mut SimulatorReader) {
        for (c, id) in self.brake_temperature_c.iter_mut().zip(&self.brake_temperature_id) {
            *c = reader.read(id);
        }
    }

    pub(super) fn update(&mut self, context: &UpdateContext, armed: &mut BTreeMap<u64, f64>) {
        let dt = context.delta_as_secs_f64();
        let ambient_c = context.ambient_temperature().get::<degree_celsius>();
        let groundspeed_ms = if context.is_on_ground() { context.ground_speed().get::<meter_per_second>() } else { 0. };
        self.burst_wheel.retain(|id, _| armed.get(id).is_some_and(|&m| m >= 1.));
        for i in 0..WHEELS {
            let id = Self::failure_of(i);
            let magnitude = armed.get(&id).copied().unwrap_or(0.);
            let burst = magnitude >= 1.;
            if burst && i == *self.burst_wheel.entry(id).or_insert_with(|| Self::first_wheel_of(id)) {
                self.wheels[i].leaked_fraction = 1.;
            }
            let leak = if burst { 0. } else { magnitude };
            let melted_now = if i < BRAKED_WHEELS {
                self.wheels[i].step(self.brake_temperature_c[i], ambient_c, groundspeed_ms, leak, dt)
            } else {
                self.wheels[i].step_unbraked(ambient_c, groundspeed_ms, leak, dt)
            };
            if melted_now {
                self.burst_wheel.entry(id).or_insert(i);
                armed.insert(id, 1.);
            }
        }
    }

    fn failure_of(wheel: usize) -> u64 {
        if wheel < BRAKED_WHEELS {
            LEG_FAILURE_IDS[leg_of_wheel(wheel)]
        } else {
            UNBRAKED_WHEELS[wheel - BRAKED_WHEELS].1
        }
    }

    fn first_wheel_of(id: u64) -> usize {
        (0..WHEELS).find(|&w| Self::failure_of(w) == id).unwrap_or(0)
    }

    pub(super) fn pressures_pa(&self) -> [f64; WHEELS] {
        self.wheels.map(|w| w.pressure_pa())
    }

    pub(super) fn temperatures_c(&self) -> [f64; WHEELS] {
        self.wheels.map(|w| w.temp_c)
    }

    pub(super) fn write(&self, writer: &mut SimulatorWriter) {
        for (i, w) in self.wheels.iter().enumerate() {
            writer.write(&self.pressure_id[i], w.pressure_pa());
            writer.write(&self.temperature_id[i], w.temp_c);
            writer.write(&self.leaked_id[i], w.leaked_fraction);
            writer.write(&self.tread_id[i], w.tread_mm);
            writer.write(&self.fuse_plug_id[i], w.fuse_plug_melted);
        }
    }
}
