use super::start_state::{
    classify, forbids_installed_ground_equipment, initial_temperatures, MsfsSpawn, WARM_HOT_SECTION_METAL_K,
    WARM_OIL_K,
};

#[test]
fn classify_reads_ground_and_engine_state_only() {
    assert_eq!(classify(true, false), MsfsSpawn::ColdAndDark);
    assert_eq!(classify(true, true), MsfsSpawn::GroundRunning);
    assert_eq!(classify(false, false), MsfsSpawn::InFlight);
    assert_eq!(classify(false, true), MsfsSpawn::InFlight);
}

#[test]
fn cold_and_dark_invents_no_heat_at_all() {
    let t = initial_temperatures(MsfsSpawn::ColdAndDark, -5.0);
    assert_eq!(t.oil_tank_k, 268.15);
    assert_eq!(t.hot_section_metal_k, 268.15);
    assert_eq!(t.brake_stack_c, -5.0);
}

#[test]
fn a_running_spawn_starts_warmed_not_ambient() {
    for spawn in [MsfsSpawn::GroundRunning, MsfsSpawn::InFlight] {
        let t = initial_temperatures(spawn, 15.0);
        assert!(t.oil_tank_k >= WARM_OIL_K, "{spawn:?}: oil must not start ambient-cold next to a running engine");
        assert!(
            t.hot_section_metal_k >= WARM_HOT_SECTION_METAL_K,
            "{spawn:?}: hot-section metal must not start ambient-cold next to a running engine"
        );
    }
}

#[test]
fn a_hot_day_is_never_cooled_down_to_the_warm_constant() {
    let t = initial_temperatures(MsfsSpawn::GroundRunning, 85.0);
    assert!((t.oil_tank_k - (85.0 + 273.15)).abs() < 1e-9, "oil must follow an ambient hotter than its own floor");
    assert_eq!(t.hot_section_metal_k, WARM_HOT_SECTION_METAL_K, "the hot-section floor still applies far below it");
}

#[test]
fn brake_stack_is_always_ambient() {
    for spawn in [MsfsSpawn::ColdAndDark, MsfsSpawn::GroundRunning, MsfsSpawn::InFlight] {
        assert_eq!(initial_temperatures(spawn, 12.0).brake_stack_c, 12.0);
    }
}

#[test]
fn only_in_flight_forbids_installed_ground_equipment() {
    assert!(!forbids_installed_ground_equipment(MsfsSpawn::ColdAndDark));
    assert!(!forbids_installed_ground_equipment(MsfsSpawn::GroundRunning));
    assert!(forbids_installed_ground_equipment(MsfsSpawn::InFlight));
}
