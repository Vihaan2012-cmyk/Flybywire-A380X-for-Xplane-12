use deep_systems::deep::apu::params::OIL_PRESSURE_TRIP_PSI;
use systems::{
    apu::{ApuConstants, ApuGenerator, ApuStartMotor, AuxiliaryPowerUnit},
    simulation::{InitContext, Read, SimulationElement, SimulatorReader, VariableIdentifier},
};

pub(super) struct DeepApuAuthority {
    oil_pressure_psi_id: VariableIdentifier,
    oil_pressure_psi: f64,
}
impl DeepApuAuthority {
    pub fn new(context: &mut InitContext) -> Self {
        Self {
            oil_pressure_psi_id: context
                .get_identifier(deep_systems::lvar_key("DEEP_APU_OIL_PRESSURE_PSI")),
            oil_pressure_psi: 0.0,
        }
    }

    fn capability_loss(&self) -> f64 {
        ((OIL_PRESSURE_TRIP_PSI - self.oil_pressure_psi) / OIL_PRESSURE_TRIP_PSI).clamp(0.0, 1.0)
    }

    pub fn apply<T: ApuGenerator, U: ApuStartMotor, C: ApuConstants, const N: usize>(
        &self,
        apu: &mut AuxiliaryPowerUnit<T, U, C, N>,
    ) {
        apu.set_deep_capability_loss(self.capability_loss());
    }
}
impl SimulationElement for DeepApuAuthority {
    fn read(&mut self, reader: &mut SimulatorReader) {
        self.oil_pressure_psi = reader.read(&self.oil_pressure_psi_id);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn authority_with_oil_pressure(psi: f64) -> DeepApuAuthority {
        DeepApuAuthority {
            oil_pressure_psi_id: VariableIdentifier::default(),
            oil_pressure_psi: psi,
        }
    }

    #[test]
    fn a_healthy_aircraft_is_unchanged_by_the_apu_authority() {
        let authority = authority_with_oil_pressure(60.0);
        assert_eq!(
            authority.capability_loss(),
            0.0,
            "a healthy oil pressure reading must drive exactly zero capability loss"
        );

        let authority_default = authority_with_oil_pressure(0.0);
        assert_eq!(
            authority_default.capability_loss(),
            1.0,
            "0 psi alone reads as a full loss; the APU-off case is excluded by the caller's own gate, not this formula"
        );
    }

    #[test]
    fn a_deep_oil_fault_moves_the_derived_loss_continuously_with_its_severity() {
        let mild = authority_with_oil_pressure(12.0);
        let severe = authority_with_oil_pressure(3.0);

        assert!(
            mild.capability_loss() > 0.0,
            "pressure below the trip threshold must already read as a nonzero loss"
        );
        assert!(
            severe.capability_loss() > mild.capability_loss(),
            "a more severe armed fault (lower oil pressure) must drive a strictly larger \
             capability loss than a milder one -- continuous with severity, not a step"
        );
        assert!(
            severe.capability_loss() <= 1.0 && mild.capability_loss() <= 1.0,
            "the loss must stay within its own real 0..1 range regardless of how far below \
             trip the reading goes"
        );
    }
}
