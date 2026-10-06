#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MsfsSpawn {
    ColdAndDark,
    GroundRunning,
    InFlight,
}

pub fn classify(on_ground: bool, engines_running: bool) -> MsfsSpawn {
    match (on_ground, engines_running) {
        (true, false) => MsfsSpawn::ColdAndDark,
        (true, true) => MsfsSpawn::GroundRunning,
        (false, _) => MsfsSpawn::InFlight,
    }
}

pub const WARM_OIL_K: f64 = 353.15;

pub const WARM_HOT_SECTION_METAL_K: f64 = 773.15;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InitialTemperatures {
    pub oil_tank_k: f64,
    pub hot_section_metal_k: f64,
    pub brake_stack_c: f64,
}

pub fn initial_temperatures(spawn: MsfsSpawn, ambient_c: f64) -> InitialTemperatures {
    let ambient_k = ambient_c + 273.15;
    let (oil_tank_k, hot_section_metal_k) = match spawn {
        MsfsSpawn::ColdAndDark => (ambient_k, ambient_k),
        MsfsSpawn::GroundRunning | MsfsSpawn::InFlight => {
            (WARM_OIL_K.max(ambient_k), WARM_HOT_SECTION_METAL_K.max(ambient_k))
        }
    };
    InitialTemperatures { oil_tank_k, hot_section_metal_k, brake_stack_c: ambient_c }
}

pub fn forbids_installed_ground_equipment(spawn: MsfsSpawn) -> bool {
    spawn == MsfsSpawn::InFlight
}
