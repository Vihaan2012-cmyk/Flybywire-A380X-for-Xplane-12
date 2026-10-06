use super::nose_steering::Pushback;
use crate::simulation::{
    InitContext, Read, SimulationElement, SimulatorReader, SimulatorWriter, VariableIdentifier,
    Write,
};

pub struct BypassPin {
    nw_strg_disc_memo_id: VariableIdentifier,
    gsx_pin_state_id: VariableIdentifier,

    gsx_pin_inserted: bool,
    bypass_pin_inserted: bool,
    deep_disc_jammed_id: VariableIdentifier,
    deep_disc_jammed: bool,
    deep_disconnected_id: VariableIdentifier,
    deep_disconnected: bool,
}

impl BypassPin {
    pub fn new(context: &mut InitContext) -> Self {
        Self {
            gsx_pin_state_id: context.get_identifier("EXTERNAL_BYPASS_PIN_INSERTED".to_owned()),
            gsx_pin_inserted: false,
            bypass_pin_inserted: false,
            nw_strg_disc_memo_id: context.get_identifier("HYD_NW_STRG_DISC_ECAM_MEMO".to_owned()),
            deep_disc_jammed_id: context.get_identifier("NW_STEER_DISC_JAMMED".to_owned()),
            deep_disc_jammed: false,
            deep_disconnected_id: context.get_identifier("NW_STEER_DISCONNECTED".to_owned()),
            deep_disconnected: false,
        }
    }
    pub fn update(&mut self, fbw_tug: &impl Pushback) {
        self.bypass_pin_inserted = if self.deep_disc_jammed {
            self.deep_disconnected
        } else {
            fbw_tug.is_nose_wheel_steering_pin_inserted() || self.gsx_pin_inserted
        };
    }

    pub fn is_nose_wheel_steering_pin_inserted(&self) -> bool {
        self.bypass_pin_inserted
    }
}

impl SimulationElement for BypassPin {
    fn read(&mut self, reader: &mut SimulatorReader) {
        self.gsx_pin_inserted = reader.read(&self.gsx_pin_state_id);
        self.deep_disc_jammed = reader.read(&self.deep_disc_jammed_id);
        self.deep_disconnected = reader.read(&self.deep_disconnected_id);
    }

    fn write(&self, writer: &mut SimulatorWriter) {
        writer.write(&self.nw_strg_disc_memo_id, self.bypass_pin_inserted);
    }
}
