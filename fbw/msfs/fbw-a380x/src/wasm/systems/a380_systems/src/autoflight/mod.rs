use systems::simulation::{
    InitContext, Read, SimulationElement, SimulatorReader, SimulatorWriter, VariableIdentifier,
    Write,
};

pub(crate) struct DeepAutoflightAuthority {
    fcu_fault_id: VariableIdentifier,
    fcu_fault: f64,
    fcu_switched_off_id: VariableIdentifier,
    fcu_switched_off: f64,
    capt_bkup_fault_id: VariableIdentifier,
    capt_bkup_fault: f64,
    fo_bkup_fault_id: VariableIdentifier,
    fo_bkup_fault: f64,

    ap_active_id: [VariableIdentifier; 2],
    ap_active_raw: [f64; 2],
}

impl DeepAutoflightAuthority {
    pub fn new(context: &mut InitContext) -> Self {
        Self {
            fcu_fault_id: context.get_identifier("DEEP_AUTOFLT_FCU_FAULT".to_owned()),
            fcu_fault: 0.0,
            fcu_switched_off_id: context.get_identifier("DEEP_AUTOFLT_FCU_SWITCHED_OFF".to_owned()),
            fcu_switched_off: 0.0,
            capt_bkup_fault_id: context
                .get_identifier("DEEP_AUTOFLT_CAPT_FCU_BKUP_FAULT".to_owned()),
            capt_bkup_fault: 0.0,
            fo_bkup_fault_id: context.get_identifier("DEEP_AUTOFLT_FO_FCU_BKUP_FAULT".to_owned()),
            fo_bkup_fault: 0.0,
            ap_active_id: [1, 2].map(|n| {
                context.get_identifier(format!("AUTOPILOT_{n}_ACTIVE"))
            }),
            ap_active_raw: [0.0; 2],
        }
    }

    fn authority_margin(&self) -> f64 {
        1.0 - self
            .fcu_fault
            .max(self.fcu_switched_off)
            .max(self.capt_bkup_fault)
            .max(self.fo_bkup_fault)
            .clamp(0.0, 1.0)
    }

    fn apply(&self) -> [f64; 2] {
        let margin = self.authority_margin();
        self.ap_active_raw.map(|raw| raw * margin)
    }
}

impl SimulationElement for DeepAutoflightAuthority {
    fn read(&mut self, reader: &mut SimulatorReader) {
        self.fcu_fault = reader.read(&self.fcu_fault_id);
        self.fcu_switched_off = reader.read(&self.fcu_switched_off_id);
        self.capt_bkup_fault = reader.read(&self.capt_bkup_fault_id);
        self.fo_bkup_fault = reader.read(&self.fo_bkup_fault_id);
        self.ap_active_raw = self.ap_active_id.map(|id| reader.read(&id));
    }

    fn write(&self, writer: &mut SimulatorWriter) {
        let applied = self.apply();
        for (id, value) in self.ap_active_id.iter().zip(applied) {
            writer.write(id, value);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn authority_with(
        fcu_fault: f64,
        fcu_switched_off: f64,
        capt_bkup_fault: f64,
        fo_bkup_fault: f64,
        ap_raw: [f64; 2],
    ) -> DeepAutoflightAuthority {
        DeepAutoflightAuthority {
            fcu_fault_id: VariableIdentifier::default(),
            fcu_fault,
            fcu_switched_off_id: VariableIdentifier::default(),
            fcu_switched_off,
            capt_bkup_fault_id: VariableIdentifier::default(),
            capt_bkup_fault,
            fo_bkup_fault_id: VariableIdentifier::default(),
            fo_bkup_fault,
            ap_active_id: [VariableIdentifier::default(), VariableIdentifier::default()],
            ap_active_raw: ap_raw,
        }
    }

    #[test]
    fn a_healthy_fcu_leaves_flybywires_own_autopilots_byte_identical() {
        let authority = authority_with(0.0, 0.0, 0.0, 0.0, [1.0, 0.0]);
        assert_eq!(authority.authority_margin(), 1.0);
        assert_eq!(authority.apply(), [1.0, 0.0]);
    }

    #[test]
    fn a_partial_fcu_fault_scales_flybywires_own_autopilot_output_continuously_with_severity() {
        let mild = authority_with(0.3, 0.0, 0.0, 0.0, [1.0, 1.0]);
        let severe = authority_with(0.8, 0.0, 0.0, 0.0, [1.0, 1.0]);
        let full = authority_with(1.0, 0.0, 0.0, 0.0, [1.0, 1.0]);

        let mild_out = mild.apply();
        let severe_out = severe.apply();
        let full_out = full.apply();

        let close = |a: [f64; 2], b: [f64; 2]| (a[0] - b[0]).abs() < 1e-9 && (a[1] - b[1]).abs() < 1e-9;
        assert!(close(mild_out, [0.7, 0.7]), "{mild_out:?}");
        assert!(close(severe_out, [0.2, 0.2]), "{severe_out:?}");
        assert!(close(full_out, [0.0, 0.0]), "{full_out:?}");
        assert!(
            severe_out[0] < mild_out[0] && mild_out[0] < 1.0,
            "a worse FCU fault must sag FlyByWire's own autopilot output further than a milder one: mild {mild_out:?}, severe {severe_out:?}"
        );
    }

    #[test]
    fn fcu_switched_off_and_both_mfd_backups_combine_by_worst_case_not_sum() {
        let authority = authority_with(0.0, 0.4, 0.4, 0.0, [1.0, 1.0]);
        assert_eq!(authority.authority_margin(), 0.6);
    }
}
