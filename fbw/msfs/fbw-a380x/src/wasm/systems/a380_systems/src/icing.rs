use systems::{
    icing_state::{IcingState, PassiveIcingElement},
    simulation::{
        InitContext, Read, SimulationElement, SimulationElementVisitor, SimulatorReader,
        SimulatorWriter, UpdateContext, VariableIdentifier, Write,
    },
};

use std::time::Duration;

pub struct Icing {
    cockpit_icing_stick: IcingState,
    deep_ice_rain_authority: DeepIceRainAuthority,
}
impl Icing {
    pub fn new(context: &mut InitContext) -> Self {
        Self {
            cockpit_icing_stick: IcingState::new(
                context,
                "ICING_STICK_INDICATOR",
                Duration::from_secs(120),
                Duration::from_secs(200),
                None,
            ),
            deep_ice_rain_authority: DeepIceRainAuthority::new(context),
        }
    }

    pub fn update(&mut self, context: &UpdateContext) {
        self.cockpit_icing_stick
            .update(context, None::<&PassiveIcingElement>);
        self.deep_ice_rain_authority.update();
    }
}
impl SimulationElement for Icing {
    fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
        self.cockpit_icing_stick.accept(visitor);
        self.deep_ice_rain_authority.accept(visitor);
        visitor.visit(self);
    }

    fn read(&mut self, reader: &mut SimulatorReader) {
        self.deep_ice_rain_authority.read(reader);
    }

    fn write(&self, writer: &mut SimulatorWriter) {
        self.deep_ice_rain_authority.write(writer);
    }
}

struct DeepIceRainAuthority {
    wing_l_overheat_id: VariableIdentifier,
    wing_r_overheat_id: VariableIdentifier,
    probe_heat_pitot1_fault_id: VariableIdentifier,
    probe_heat_pitot2_fault_id: VariableIdentifier,
    window_heat_l_fault_id: VariableIdentifier,
    window_heat_r_fault_id: VariableIdentifier,
    wing_anti_ice_pb_has_fault_id: VariableIdentifier,
    probe_window_heat_pb_has_fault_id: VariableIdentifier,

    wing_l_overheat: f64,
    wing_r_overheat: f64,
    probe_heat_pitot1_fault: f64,
    probe_heat_pitot2_fault: f64,
    window_heat_l_fault: f64,
    window_heat_r_fault: f64,

    wing_anti_ice_pb_has_fault: f64,
    probe_window_heat_pb_has_fault: f64,
}

fn valid_magnitude(v: f64) -> f64 {
    if v.is_finite() && v >= 0.0 {
        v
    } else {
        0.0
    }
}

impl DeepIceRainAuthority {
    fn new(context: &mut InitContext) -> Self {
        Self {
            wing_l_overheat_id: context.get_identifier("ANTI_ICE_WING_L_OVERHEAT".to_owned()),
            wing_r_overheat_id: context.get_identifier("ANTI_ICE_WING_R_OVERHEAT".to_owned()),
            probe_heat_pitot1_fault_id: context
                .get_identifier("PROBE_HEAT_PITOT1_FAULT".to_owned()),
            probe_heat_pitot2_fault_id: context
                .get_identifier("PROBE_HEAT_PITOT2_FAULT".to_owned()),
            window_heat_l_fault_id: context.get_identifier("WINDOW_HEAT_L_FAULT".to_owned()),
            window_heat_r_fault_id: context.get_identifier("WINDOW_HEAT_R_FAULT".to_owned()),
            wing_anti_ice_pb_has_fault_id: context
                .get_identifier("OVHD_ANTI_ICE_WING_PB_HAS_FAULT".to_owned()),
            probe_window_heat_pb_has_fault_id: context
                .get_identifier("OVHD_ANTI_ICE_PROBE_WINDOW_HEAT_PB_HAS_FAULT".to_owned()),

            wing_l_overheat: 0.0,
            wing_r_overheat: 0.0,
            probe_heat_pitot1_fault: 0.0,
            probe_heat_pitot2_fault: 0.0,
            window_heat_l_fault: 0.0,
            window_heat_r_fault: 0.0,

            wing_anti_ice_pb_has_fault: 0.0,
            probe_window_heat_pb_has_fault: 0.0,
        }
    }

    fn read(&mut self, reader: &mut SimulatorReader) {
        self.wing_l_overheat = valid_magnitude(reader.read(&self.wing_l_overheat_id));
        self.wing_r_overheat = valid_magnitude(reader.read(&self.wing_r_overheat_id));
        self.probe_heat_pitot1_fault =
            valid_magnitude(reader.read(&self.probe_heat_pitot1_fault_id));
        self.probe_heat_pitot2_fault =
            valid_magnitude(reader.read(&self.probe_heat_pitot2_fault_id));
        self.window_heat_l_fault = valid_magnitude(reader.read(&self.window_heat_l_fault_id));
        self.window_heat_r_fault = valid_magnitude(reader.read(&self.window_heat_r_fault_id));
    }

    fn update(&mut self) {
        self.wing_anti_ice_pb_has_fault = self.wing_l_overheat.max(self.wing_r_overheat);
        self.probe_window_heat_pb_has_fault = self
            .probe_heat_pitot1_fault
            .max(self.probe_heat_pitot2_fault)
            .max(self.window_heat_l_fault)
            .max(self.window_heat_r_fault);
    }

    fn write(&self, writer: &mut SimulatorWriter) {
        writer.write(&self.wing_anti_ice_pb_has_fault_id, self.wing_anti_ice_pb_has_fault);
        writer.write(
            &self.probe_window_heat_pb_has_fault_id,
            self.probe_window_heat_pb_has_fault,
        );
    }
}
impl SimulationElement for DeepIceRainAuthority {}
