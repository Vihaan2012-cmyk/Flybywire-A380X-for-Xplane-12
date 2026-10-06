use std::collections::BTreeMap;

use deep_systems::DerivedFailure;
use systems::simulation::{InitContext, VariableIdentifier, Writer};

const PNEUMATIC_VALVES: &[(u64, usize)] = &[
    (36_008, 1),
    (36_009, 2),
    (36_010, 3),
    (36_011, 4),
    (36_012, 5),
    (36_013, 6),
    (36_014, 7),
    (36_015, 8),
    (36_016, 9),
    (36_017, 10),
    (36_018, 11),
    (49_003, 12),
    (21_050, 13),
    (21_051, 14),
    (21_052, 15),
    (21_053, 16),
];

pub(super) struct PneumaticValves {
    ids: Vec<VariableIdentifier>,
    seizure: Vec<f64>,
}

impl PneumaticValves {
    pub(super) fn new(context: &mut InitContext) -> Self {
        Self {
            ids: PNEUMATIC_VALVES
                .iter()
                .map(|&(_, n)| context.get_identifier(format!("PNEU_VALVE_FAILED:{n}")))
                .collect(),
            seizure: vec![0.; PNEUMATIC_VALVES.len()],
        }
    }

    pub(super) fn update(&mut self, derived: &[DerivedFailure], armed: &BTreeMap<u64, f64>) {
        for (seizure, &(id, _)) in self.seizure.iter_mut().zip(PNEUMATIC_VALVES) {
            let deep = derived.iter().filter(|d| d.fbw_id == id).map(|d| d.magnitude).fold(0., f64::max);
            let crew = armed.get(&id).copied().unwrap_or(0.);
            *seizure = deep.max(crew).clamp(0., 1.);
        }
    }

    pub(super) fn write(&self, writer: &mut impl Writer) {
        for (id, &seizure) in self.ids.iter().zip(&self.seizure) {
            writer.write_f64(id, seizure);
        }
    }
}
