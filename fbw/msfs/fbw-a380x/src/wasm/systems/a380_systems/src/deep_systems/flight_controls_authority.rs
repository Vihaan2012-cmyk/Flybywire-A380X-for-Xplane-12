use std::collections::BTreeMap;

use systems::simulation::{InitContext, SimulatorWriter, VariableIdentifier, Write};

use deep_systems::Faults;

const JAM_THRESHOLD: f64 = 0.5;

fn pcu_jam_id(components: &BTreeMap<String, Vec<u64>>, component: &str) -> u64 {
    let ids = components
        .get(component)
        .unwrap_or_else(|| panic!("flight_controls registry has no component {component}"));
    assert_eq!(
        ids.len(),
        7,
        "component {component} registers {} failures, HighLiftIds::build in deep::flight_controls::live expects 7",
        ids.len()
    );
    ids[0]
}

const PRIMARY_SURFACE_COMPONENTS: [&str; 12] = [
    "27_fctl.ail_l1",
    "27_fctl.ail_l2",
    "27_fctl.ail_l3",
    "27_fctl.ail_r1",
    "27_fctl.ail_r2",
    "27_fctl.ail_r3",
    "27_fctl.elev_l_inbd",
    "27_fctl.elev_l_outbd",
    "27_fctl.elev_r_inbd",
    "27_fctl.elev_r_outbd",
    "27_fctl.rud_upper",
    "27_fctl.rud_lower",
];

const JAM_FIELD_INDEX: usize = 0;
const RUNAWAY_FIELD_INDEX: usize = 1;

fn primary_surface_ids(components: &BTreeMap<String, Vec<u64>>, component: &str) -> (u64, u64) {
    let ids = components
        .get(component)
        .unwrap_or_else(|| panic!("flight_controls registry has no component {component}"));
    assert!(
        ids.len() >= 2,
        "component {component} registers {} failures, surface_fields in deep::flight_controls::registry expects at least jam+runaway",
        ids.len()
    );
    (ids[JAM_FIELD_INDEX], ids[RUNAWAY_FIELD_INDEX])
}

pub struct FlightControlsAuthority {
    flaps_jammed_id: VariableIdentifier,
    slats_jammed_id: VariableIdentifier,

    flap_l_pcu_jam: u64,
    flap_r_pcu_jam: u64,
    slat_l_pcu_jam: u64,
    slat_r_pcu_jam: u64,

    flaps_jammed: bool,
    slats_jammed: bool,

    jam_ids: [u64; 12],
    runaway_ids: [u64; 12],
    jam: [f64; 12],
    runaway: [f64; 12],
    jam_var_ids: [VariableIdentifier; 12],
    runaway_var_ids: [VariableIdentifier; 12],
}

const PRIMARY_SURFACE_LVAR_NAMES: [&str; 12] = [
    "AIL_L1", "AIL_L2", "AIL_L3", "AIL_R1", "AIL_R2", "AIL_R3", "ELEV_L_INBD", "ELEV_L_OUTBD",
    "ELEV_R_INBD", "ELEV_R_OUTBD", "RUD_UPPER", "RUD_LOWER",
];

impl FlightControlsAuthority {
    pub fn new(context: &mut InitContext) -> Self {
        let registry = deep_systems::deep::registry();
        let components: BTreeMap<String, Vec<u64>> =
            registry.components.into_iter().map(|c| (c.id, c.failures)).collect();

        let mut jam_ids = [0u64; 12];
        let mut runaway_ids = [0u64; 12];
        for (i, name) in PRIMARY_SURFACE_COMPONENTS.iter().enumerate() {
            let (jam_id, runaway_id) = primary_surface_ids(&components, name);
            jam_ids[i] = jam_id;
            runaway_ids[i] = runaway_id;
        }

        Self {
            flaps_jammed_id: context.get_identifier("FLAPS_JAMMED".to_owned()),
            slats_jammed_id: context.get_identifier("SLATS_JAMMED".to_owned()),
            flap_l_pcu_jam: pcu_jam_id(&components, "27_fctl.flap_l"),
            flap_r_pcu_jam: pcu_jam_id(&components, "27_fctl.flap_r"),
            slat_l_pcu_jam: pcu_jam_id(&components, "27_fctl.slat_l"),
            slat_r_pcu_jam: pcu_jam_id(&components, "27_fctl.slat_r"),
            flaps_jammed: false,
            slats_jammed: false,
            jam_ids,
            runaway_ids,
            jam: [0.0; 12],
            runaway: [0.0; 12],
            jam_var_ids: std::array::from_fn(|i| {
                context.get_identifier(format!(
                    "DEEP_FCTL_{}_JAM",
                    PRIMARY_SURFACE_LVAR_NAMES[i]
                ))
            }),
            runaway_var_ids: std::array::from_fn(|i| {
                context.get_identifier(format!(
                    "DEEP_FCTL_{}_RUNAWAY",
                    PRIMARY_SURFACE_LVAR_NAMES[i]
                ))
            }),
        }
    }

    pub fn update(&mut self, faults: &Faults) {
        self.flaps_jammed =
            faults.get(self.flap_l_pcu_jam) >= JAM_THRESHOLD || faults.get(self.flap_r_pcu_jam) >= JAM_THRESHOLD;
        self.slats_jammed =
            faults.get(self.slat_l_pcu_jam) >= JAM_THRESHOLD || faults.get(self.slat_r_pcu_jam) >= JAM_THRESHOLD;

        for i in 0..12 {
            self.jam[i] = faults.get(self.jam_ids[i]);
            self.runaway[i] = faults.get(self.runaway_ids[i]);
        }
    }

    pub fn write(&self, writer: &mut SimulatorWriter) {
        writer.write(&self.flaps_jammed_id, self.flaps_jammed);
        writer.write(&self.slats_jammed_id, self.slats_jammed);
        for i in 0..12 {
            writer.write(&self.jam_var_ids[i], self.jam[i]);
            writer.write(&self.runaway_var_ids[i], self.runaway[i]);
        }
    }
}
