use systems::simulation::{
    InitContext, Read, SimulationElement, SimulationElementVisitor, SimulatorReader,
    SimulatorWriter, UpdateContext, VariableIdentifier, Writer,
};

const HEALTHY_CREW_BOTTLE_PSI: f64 = 1850.0;

pub(crate) struct A380Oxygen {
    deep_authority: DeepOxygenAuthority,

    crew_bottle_pressure_psi: f64,
    crew_bottle_pressure_psi_id: VariableIdentifier,
    crew_supply_available: bool,
    crew_supply_available_id: VariableIdentifier,
    pax_masks_deployed: bool,
    pax_masks_deployed_id: VariableIdentifier,
}
impl A380Oxygen {
    pub(crate) fn new(context: &mut InitContext) -> Self {
        Self {
            deep_authority: DeepOxygenAuthority::new(context),

            crew_bottle_pressure_psi: HEALTHY_CREW_BOTTLE_PSI,
            crew_bottle_pressure_psi_id: context
                .get_identifier("OXYGEN_CREW_PRESSURE_PSI".to_owned()),
            crew_supply_available: true,
            crew_supply_available_id: context
                .get_identifier("OXYGEN_CREW_SUPPLY_AVAILABLE".to_owned()),
            pax_masks_deployed: false,
            pax_masks_deployed_id: context
                .get_identifier("OXYGEN_PAX_MASKS_DEPLOYED".to_owned()),
        }
    }

    pub(crate) fn update(&mut self, _context: &UpdateContext) {
        self.deep_authority.apply(
            HEALTHY_CREW_BOTTLE_PSI,
            &mut self.crew_bottle_pressure_psi,
            &mut self.crew_supply_available,
            &mut self.pax_masks_deployed,
        );
    }
}
impl SimulationElement for A380Oxygen {
    fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
        self.deep_authority.accept(visitor);
        visitor.visit(self);
    }

    fn write(&self, writer: &mut SimulatorWriter) {
        writer.write_f64(&self.crew_bottle_pressure_psi_id, self.crew_bottle_pressure_psi);
        writer.write_f64(&self.crew_supply_available_id, if self.crew_supply_available { 1.0 } else { 0.0 });
        writer.write_f64(&self.pax_masks_deployed_id, if self.pax_masks_deployed { 1.0 } else { 0.0 });
    }
}

struct DeepOxygenAuthority {
    tick_us_id: VariableIdentifier,
    tick_us: f64,

    crew_bottle_pressure_psi_id: VariableIdentifier,
    crew_bottle_pressure_psi: f64,
    crew_supply_available_id: VariableIdentifier,
    crew_supply_available: f64,
    pax_masks_deployed_id: VariableIdentifier,
    pax_masks_deployed: f64,
}
impl DeepOxygenAuthority {
    fn new(context: &mut InitContext) -> Self {
        Self {
            tick_us_id: context.get_identifier("DEEP_SYSTEMS_TICK_US".to_owned()),
            tick_us: 0.0,
            crew_bottle_pressure_psi_id: context
                .get_identifier(deep_systems::lvar_key("DEEP_OXY_CREW_BOTTLE_GAUGE_PSI")),
            crew_bottle_pressure_psi: 0.0,
            crew_supply_available_id: context
                .get_identifier(deep_systems::lvar_key("DEEP_OXY_CREW_SUPPLY_AVAILABLE")),
            crew_supply_available: 0.0,
            pax_masks_deployed_id: context
                .get_identifier(deep_systems::lvar_key("DEEP_OXY_PAX_MASKS_DEPLOYED")),
            pax_masks_deployed: 0.0,
        }
    }

    fn apply(
        &self,
        healthy_crew_bottle_psi: f64,
        crew_bottle_pressure_psi: &mut f64,
        crew_supply_available: &mut bool,
        pax_masks_deployed: &mut bool,
    ) {
        if self.tick_us > 0.0 {
            *crew_bottle_pressure_psi = self.crew_bottle_pressure_psi;
            *crew_supply_available = self.crew_supply_available != 0.0;
            *pax_masks_deployed = self.pax_masks_deployed != 0.0;
        } else {
            *crew_bottle_pressure_psi = healthy_crew_bottle_psi;
            *crew_supply_available = true;
            *pax_masks_deployed = false;
        }
    }
}
impl SimulationElement for DeepOxygenAuthority {
    fn read(&mut self, reader: &mut SimulatorReader) {
        self.tick_us = reader.read(&self.tick_us_id);
        self.crew_bottle_pressure_psi = reader.read(&self.crew_bottle_pressure_psi_id);
        self.crew_supply_available = reader.read(&self.crew_supply_available_id);
        self.pax_masks_deployed = reader.read(&self.pax_masks_deployed_id);
    }
}
