use std::collections::BTreeMap;

use crate::deep::api::{Area as RegArea, Cond, EcamAlert, FailureDef};
use crate::deep::live::{all_areas, CommandedSurfaces, Controls, Faults, Truth};

use super::weather_truth::EnvironmentTruth;

pub struct Profile {
    pub name: &'static str,
    pub truth: fn() -> Truth,
    pub frames: usize,
}

fn flying_surfaces() -> CommandedSurfaces {
    CommandedSurfaces {
        ailerons_deg: [[4.0, 3.0, 2.0], [-4.0, -3.0, -2.0]],
        elevators_deg: [[-3.0, -2.5], [-3.5, -3.0]],
        rudders_deg: [2.0, 1.5],
        spoilers_deg: [[5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0], [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]],
        ths_deg: -1.5,
    }
}

fn cold_dark() -> Truth {
    Truth { dt_s: 0.5, ..Truth::default() }
}

fn cruise() -> Truth {
    Truth {
        dt_s: 0.1,
        altitude_ft: 37_000.0,
        on_ground: false,
        flight_ready: true,
        fmgc_flight_phase: 4.0,
        engine_n1_frac: [0.88; 4],
        engine_tla_deg: [25.0; 4],
        to_flex_temp_set: false,
        engine_running: [true; 4],
        engine_oil_filter_bypassed: [false; 4],
        engine_bleed_pressure_pa: [207_000.0; 4],
        engine_bleed_temp_k: [523.0; 4],
        engine_hp_port_pressure_pa: [827_000.0; 4],
        engine_hp_port_temp_k: [703.0; 4],
        engine_ip_port_pressure_pa: [207_000.0; 4],
        engine_ip_port_temp_k: [523.0; 4],
        engine_n2_frac: [0.90; 4],
        engine_n3_frac: [0.93; 4],
        engine_n2_healthy_frac: [0.90; 4],
        engine_n3_healthy_frac: [0.93; 4],
        engine_n1_commanded_pct: [88.0; 4],
        aircraft_preset_quick_mode: false,
        engine_customer_bleed_kg_s: [0.0; 4],
        sim_engine_corrected_n1_pct: [0.0; 4],
        sim_engine_corrected_n2_pct: [0.0; 4],
        engine_fuel_flow_kg_s: [0.9; 4],
        fuel_tank_quantity_gal: None,
        tyre_pressure_pa: [1_550_000.0; crate::physics::tyre::WHEELS],
        tyre_temp_c: [15.0; crate::physics::tyre::WHEELS],
        engine_oil_pressure_pa: [3.1e5; 4],
        engine_oil_temp_c: [85.0; 4],
        engine_oil_quantity_fraction: [1.0; 4],
        engine_tgt_c: [700.0; 4],
        engine_t25_c: [140.0; 4],
        door_open_fraction: [0.0; crate::deep::live::DOOR_NAMES.len()],
        apu_running: false,
        apu_bleed_pressure_pa: 21_662.0,
        ac_bus_volts: [115.0; 4],
        dc_bus_volts: [28.0; 2],
        ac_bus_powered: [true; 4],
        dc_bus_powered: [true; 2],
        prim_healthy: [true; 3],
        sec_healthy: [true; 3],
        prim_left_sidestick_disabled: false,
        prim_right_sidestick_disabled: false,
        prim_left_sidestick_priority_locked: false,
        prim_right_sidestick_priority_locked: false,
        flap_lever_handle_index: 0.0,
        capt_sidestick_pitch_raw: 0.0,
        capt_sidestick_roll_raw: 0.0,
        rudder_pedal_raw: 0.0,
        body_rate_pitch_raw: 0.0,
        body_rate_yaw_raw: 0.0,
        body_rate_roll_raw: 0.0,
        hydraulic_pressure_pa: [34_474_000.0; 2],
        gpu_plugged_in: false,
        aircraft_mass_kg: 450_000.0,
        pitch_deg: 2.5,
        roll_deg: 0.0,
        heading_true_deg: 0.0,
        groundspeed_m_s: 250.0,
        angle_of_attack_deg: 2.5,
        radio_height_ft: 37_000.0,
        leg_on_ground: [false; 5],
        leg_touchdown_sink_speed_ms: [0.0; 5],
        cabin_pressure_pa: 75_000.0,
        cabin_temp_k: 297.0,
        sun_elevation_deg: 40.0,
        fdac_channel_failure: [[false; 2]; 2],
        ocsm_channel_failure: [[false; 2]; 4],
        vertical_speed_fpm: 0.0,
        landing_elevation_ft: 0.0,
        athr_status: 2.0,
        ap1_active: true,
        ap2_active: true,
        athr_eng_fault: [false; 4],
        pack_flow_insufficient_fwd_crg: false,
        ir: [crate::deep::live::IrOutputs { pitch_deg: Some(2.5), roll_deg: Some(0.0), true_heading_deg: Some(0.0), flight_path_angle_deg: Some(0.0) }; 3],
        att_hdg_switching_knob: 1.0,
        environment: EnvironmentTruth { sat_c: -56.5, leading_edge_c: -20.0, ambient_pressure_pa: 21_662.0, tas_ms: 250.0, precipitation_on_aircraft_ratio: 0.0, weather: None },
        commanded_surfaces: flying_surfaces(),
        controls: Controls { parking_brake_on: false, gear_lever_down: false, engine_master_on: [true; 4], ..Controls::default() },
        published: Default::default(),
        rudder_trim_cmd_deg: 0.0,
        flap_cmd_deg: 0.0,
        slat_cmd_deg: 0.0,
        droop_cmd_deg: 0.0,
    }
}

fn cruise_soak() -> Truth {
    Truth { dt_s: 1.0, ..cruise() }
}

fn ground_apu() -> Truth {
    Truth {
        dt_s: 0.2,
        altitude_ft: 0.0,
        on_ground: true,
        engine_running: [false; 4],
        apu_running: true,
        apu_bleed_pressure_pa: 310_000.0,
        ac_bus_volts: [115.0; 4],
        dc_bus_volts: [28.0; 2],
        hydraulic_pressure_pa: [0.0; 2],
        gpu_plugged_in: true,
        aircraft_mass_kg: 380_000.0,
        cabin_pressure_pa: 101_325.0,
        cabin_temp_k: 300.0,
        sun_elevation_deg: 55.0,
        leg_on_ground: [true; 5],
        controls: Controls { apu_master_sw_on: true, apu_start_pb_on: true, apu_bleed_pb_on: true, apu_gen_pb_on: [true; 2], cross_bleed_selector: 2.0, parking_brake_on: true, ..Controls::default() },
        ..Truth::default()
    }
}

fn engine_start() -> Truth {
    Truth {
        dt_s: 0.2,
        on_ground: true,
        engine_running: [false; 4],
        engine_n1_frac: [0.08; 4],
        engine_n2_frac: [0.25; 4],
        engine_n3_frac: [0.22; 4],
        engine_n2_healthy_frac: [0.25; 4],
        engine_n3_healthy_frac: [0.22; 4],
        engine_n1_commanded_pct: [8.0; 4],
        aircraft_preset_quick_mode: false,
        engine_customer_bleed_kg_s: [0.0; 4],
        sim_engine_corrected_n1_pct: [0.0; 4],
        sim_engine_corrected_n2_pct: [0.0; 4],
        engine_fuel_flow_kg_s: [0.05; 4],
        apu_running: true,
        apu_bleed_pressure_pa: 310_000.0,
        ac_bus_volts: [115.0; 4],
        dc_bus_volts: [28.0; 2],
        gpu_plugged_in: true,
        controls: Controls { apu_master_sw_on: true, apu_bleed_pb_on: true, starter_engaged: [true; 4], engine_master_on: [true; 4], cross_bleed_selector: 2.0, ..Controls::default() },
        ..Truth::default()
    }
}

fn takeoff_roll() -> Truth {
    Truth {
        dt_s: 0.1,
        on_ground: true,
        engine_n1_frac: [1.0; 4],
        engine_running: [true; 4],
        engine_bleed_pressure_pa: [345_000.0; 4],
        engine_bleed_temp_k: [573.0; 4],
        engine_hp_port_pressure_pa: [1_380_000.0; 4],
        engine_hp_port_temp_k: [773.0; 4],
        engine_n2_frac: [0.98; 4],
        engine_n3_frac: [1.0; 4],
        engine_n2_healthy_frac: [0.98; 4],
        engine_n3_healthy_frac: [1.0; 4],
        engine_n1_commanded_pct: [100.0; 4],
        aircraft_preset_quick_mode: false,
        engine_customer_bleed_kg_s: [0.0; 4],
        sim_engine_corrected_n1_pct: [0.0; 4],
        sim_engine_corrected_n2_pct: [0.0; 4],
        engine_fuel_flow_kg_s: [3.2; 4],
        ac_bus_volts: [115.0; 4],
        dc_bus_volts: [28.0; 2],
        hydraulic_pressure_pa: [34_474_000.0; 2],
        aircraft_mass_kg: 560_000.0,
        groundspeed_m_s: 51.0,
        pitch_deg: 0.0,
        radio_height_ft: 0.0,
        leg_on_ground: [true; 5],
        cabin_pressure_pa: 101_325.0,
        cabin_temp_k: 298.0,
        environment: EnvironmentTruth { sat_c: 30.0, leading_edge_c: 30.0, ambient_pressure_pa: 101_325.0, tas_ms: 51.0, precipitation_on_aircraft_ratio: 0.0, weather: None },
        commanded_surfaces: CommandedSurfaces { ths_deg: -3.0, ..flying_surfaces() },
        controls: Controls { engine_master_on: [true; 4], parking_brake_on: false, ground_spoiler_lever_armed: true, gear_lever_down: true, ..Controls::default() },
        ..Truth::default()
    }
}

fn touchdown() -> Truth {
    Truth {
        dt_s: 0.05,
        on_ground: true,
        engine_n1_frac: [0.25; 4],
        engine_running: [true; 4],
        engine_bleed_pressure_pa: [172_000.0; 4],
        engine_bleed_temp_k: [473.0; 4],
        engine_n2_frac: [0.65; 4],
        engine_n3_frac: [0.60; 4],
        engine_n2_healthy_frac: [0.65; 4],
        engine_n3_healthy_frac: [0.60; 4],
        engine_n1_commanded_pct: [25.0; 4],
        aircraft_preset_quick_mode: false,
        engine_customer_bleed_kg_s: [0.0; 4],
        sim_engine_corrected_n1_pct: [0.0; 4],
        sim_engine_corrected_n2_pct: [0.0; 4],
        engine_fuel_flow_kg_s: [0.3; 4],
        ac_bus_volts: [115.0; 4],
        dc_bus_volts: [28.0; 2],
        hydraulic_pressure_pa: [34_474_000.0; 2],
        aircraft_mass_kg: 390_000.0,
        groundspeed_m_s: 70.0,
        pitch_deg: 4.0,
        angle_of_attack_deg: 6.0,
        radio_height_ft: 0.0,
        leg_on_ground: [true; 5],
        leg_touchdown_sink_speed_ms: [2.0, 2.5, 2.5, 2.2, 2.2],
        cabin_pressure_pa: 101_325.0,
        cabin_temp_k: 297.0,
        environment: EnvironmentTruth { sat_c: 12.0, leading_edge_c: 12.0, ambient_pressure_pa: 101_325.0, tas_ms: 70.0, precipitation_on_aircraft_ratio: 0.3, weather: None },
        commanded_surfaces: CommandedSurfaces { spoilers_deg: [[50.0; 8], [50.0; 8]], ..flying_surfaces() },
        controls: Controls { engine_master_on: [true; 4], gear_lever_down: true, parking_brake_on: false, brake_pedal_pos: [1.0, 1.0], ground_spoiler_lever_armed: true, gear_door_commanded_open: [0.0; 3], ..Controls::default() },
        ..Truth::default()
    }
}

fn icing_climb() -> Truth {
    Truth {
        dt_s: 0.5,
        altitude_ft: 12_000.0,
        on_ground: false,
        engine_n1_frac: [0.92; 4],
        engine_running: [true; 4],
        engine_bleed_pressure_pa: [276_000.0; 4],
        engine_bleed_temp_k: [548.0; 4],
        engine_hp_port_pressure_pa: [1_034_000.0; 4],
        engine_hp_port_temp_k: [733.0; 4],
        engine_n2_frac: [0.94; 4],
        engine_n3_frac: [0.96; 4],
        engine_n2_healthy_frac: [0.94; 4],
        engine_n3_healthy_frac: [0.96; 4],
        engine_n1_commanded_pct: [92.0; 4],
        aircraft_preset_quick_mode: false,
        engine_customer_bleed_kg_s: [0.0; 4],
        sim_engine_corrected_n1_pct: [0.0; 4],
        sim_engine_corrected_n2_pct: [0.0; 4],
        engine_fuel_flow_kg_s: [2.0; 4],
        ac_bus_volts: [115.0; 4],
        dc_bus_volts: [28.0; 2],
        hydraulic_pressure_pa: [34_474_000.0; 2],
        aircraft_mass_kg: 520_000.0,
        pitch_deg: 8.0,
        groundspeed_m_s: 160.0,
        angle_of_attack_deg: 5.0,
        radio_height_ft: 12_000.0,
        leg_on_ground: [false; 5],
        cabin_pressure_pa: 95_000.0,
        cabin_temp_k: 295.0,
        sun_elevation_deg: -5.0,
        environment: EnvironmentTruth { sat_c: -8.0, leading_edge_c: -6.0, ambient_pressure_pa: 64_400.0, tas_ms: 170.0, precipitation_on_aircraft_ratio: 1.0, weather: None },
        commanded_surfaces: flying_surfaces(),
        controls: Controls { engine_master_on: [true; 4], wing_anti_ice_selected: true, nacelle_anti_ice_selected: [true; 4], rain_removal_selected: [true; 2], parking_brake_on: false, gear_lever_down: false, ..Controls::default() },
        ..Truth::default()
    }
}

fn all_commands_exercised() -> Truth {
    Truth {
        dt_s: 0.5,
        on_ground: false,
        altitude_ft: 20_000.0,
        engine_n1_frac: [0.60; 4],
        engine_running: [true; 4],
        engine_bleed_pressure_pa: [241_000.0; 4],
        engine_bleed_temp_k: [533.0; 4],
        engine_hp_port_pressure_pa: [896_000.0; 4],
        engine_hp_port_temp_k: [713.0; 4],
        engine_n2_frac: [0.85; 4],
        engine_n3_frac: [0.88; 4],
        engine_n2_healthy_frac: [0.85; 4],
        engine_n3_healthy_frac: [0.88; 4],
        engine_n1_commanded_pct: [60.0; 4],
        aircraft_preset_quick_mode: false,
        engine_customer_bleed_kg_s: [0.0; 4],
        sim_engine_corrected_n1_pct: [0.0; 4],
        sim_engine_corrected_n2_pct: [0.0; 4],
        engine_fuel_flow_kg_s: [1.2; 4],
        apu_running: true,
        apu_bleed_pressure_pa: 200_000.0,
        ac_bus_volts: [115.0; 4],
        dc_bus_volts: [28.0; 2],
        hydraulic_pressure_pa: [34_474_000.0; 2],
        gpu_plugged_in: true,
        aircraft_mass_kg: 480_000.0,
        groundspeed_m_s: 180.0,
        angle_of_attack_deg: 4.0,
        radio_height_ft: 20_000.0,
        leg_on_ground: [false; 5],
        cabin_pressure_pa: 85_000.0,
        cabin_temp_k: 296.0,
        environment: EnvironmentTruth { sat_c: -25.0, leading_edge_c: -15.0, ambient_pressure_pa: 46_600.0, tas_ms: 200.0, precipitation_on_aircraft_ratio: 0.5, weather: None },
        commanded_surfaces: flying_surfaces(),
        controls: Controls {
            fire_pb_released: [true; 4],
            fire_pb_apu_released: true,
            fire_agent_pb_pressed: [[true; 2]; 4],
            fire_agent_pb_apu_pressed: true,
            cargo_agent_pb_pressed: [true; 2],
            wing_anti_ice_selected: true,
            nacelle_anti_ice_selected: [true; 4],
            engine_bleed_pb_auto: [false; 4],
            apu_bleed_pb_on: true,
            cross_bleed_selector: 0.0,
            pack_pb_on: [false; 2],
            starter_engaged: [true; 4],
            rain_removal_selected: [true; 2],
            steering_command_deg: [20.0, 5.0, -5.0],
            jettison_armed: true,
            jettison_valve_selected: [true; 2],
            crossfeed_valve_selected: [true; 4],
            cargo_door_commanded_open: [1.0; 3],
            water_demand_l_s: [0.05, 0.02],
            gear_door_commanded_open: [1.0; 3],
            gear_lever_down: true,
            parking_brake_on: true,
            remote_cb_ctl_active: true,
            brake_pedal_pos: [0.5, 0.5],
            engine_master_on: [false; 4],
            eng_gen_pb_on: [false; 4],
            apu_gen_pb_on: [false; 2],
            bat_pb_auto: [false; 2],
            ground_spoiler_lever_armed: true,
            apu_master_sw_on: true,
            apu_start_pb_on: true,
            reverser_deploy_commanded: [true; 2],
            baro_mode: [1.0, 0.0],
            fcu_switch_off: false,
            gravity_extend_selected: false,
            nw_steer_disc_selected: false,
        },
        ..Truth::default()
    }
}

fn apu_start_soak() -> Truth {
    Truth {
        dt_s: 1.0,
        on_ground: true,
        ac_bus_volts: [115.0; 4],
        dc_bus_volts: [28.0; 2],
        gpu_plugged_in: true,
        apu_running: true,
        apu_bleed_pressure_pa: 310_000.0,
        cabin_pressure_pa: 101_325.0,
        cabin_temp_k: 300.0,
        controls: Controls { apu_master_sw_on: true, apu_start_pb_on: true, apu_gen_pb_on: [true; 2], apu_bleed_pb_on: true, ..Controls::default() },
        ..Truth::default()
    }
}

fn gear_cycle() -> Truth {
    Truth {
        dt_s: 0.5,
        altitude_ft: 3_000.0,
        radio_height_ft: 3_000.0,
        on_ground: false,
        leg_on_ground: [false; 5],
        engine_n1_frac: [0.85; 4],
        engine_running: [true; 4],
        engine_n2_frac: [0.9; 4],
        engine_n3_frac: [0.92; 4],
        engine_n2_healthy_frac: [0.9; 4],
        engine_n3_healthy_frac: [0.92; 4],
        engine_n1_commanded_pct: [85.0; 4],
        aircraft_preset_quick_mode: false,
        engine_customer_bleed_kg_s: [0.0; 4],
        sim_engine_corrected_n1_pct: [0.0; 4],
        sim_engine_corrected_n2_pct: [0.0; 4],
        engine_fuel_flow_kg_s: [1.5; 4],
        ac_bus_volts: [115.0; 4],
        dc_bus_volts: [28.0; 2],
        hydraulic_pressure_pa: [34_474_000.0; 2],
        aircraft_mass_kg: 500_000.0,
        groundspeed_m_s: 90.0,
        angle_of_attack_deg: 6.0,
        environment: EnvironmentTruth { sat_c: 10.0, leading_edge_c: 10.0, ambient_pressure_pa: 90_800.0, tas_ms: 95.0, precipitation_on_aircraft_ratio: 0.0, weather: None },
        commanded_surfaces: flying_surfaces(),
        controls: Controls { engine_master_on: [true; 4], gear_lever_down: false, gear_door_commanded_open: [1.0; 3], parking_brake_on: false, ..Controls::default() },
        ..Truth::default()
    }
}

pub fn profiles() -> Vec<Profile> {
    vec![
        Profile { name: "cruise", truth: cruise, frames: 8 },
        Profile { name: "all_commands_exercised", truth: all_commands_exercised, frames: 16 },
        Profile { name: "ground_apu", truth: ground_apu, frames: 16 },
        Profile { name: "touchdown", truth: touchdown, frames: 16 },
        Profile { name: "takeoff_roll", truth: takeoff_roll, frames: 12 },
        Profile { name: "engine_start", truth: engine_start, frames: 16 },
        Profile { name: "icing_climb", truth: icing_climb, frames: 24 },
        Profile { name: "cold_dark", truth: cold_dark, frames: 20 },
        Profile { name: "gear_cycle", truth: gear_cycle, frames: 40 },
        Profile { name: "cruise_soak", truth: cruise_soak, frames: 60 },
        Profile { name: "apu_start_soak", truth: apu_start_soak, frames: 120 },
    ]
}

pub const SENTINEL_ID: u64 = 999_999_999;

pub fn reference_faults() -> Faults {
    Faults::from_pairs([(SENTINEL_ID, 1.0)])
}

pub fn armed_with(id: u64, magnitude: f64) -> Faults {
    Faults::from_pairs([(SENTINEL_ID, 1.0), (id, magnitude)])
}

pub fn fresh_areas() -> crate::deep::live::Deep {
    crate::deep::electrical::live::board::clear();
    all_areas()
}

pub struct Baseline {
    pub names: Vec<String>,
    pub frames: Vec<Vec<f64>>,
}

pub fn baseline(truth: &Truth, faults: &Faults, frames: usize) -> Baseline {
    baseline_phased(&|_| truth.clone(), faults, frames)
}

pub fn baseline_phased(truth_at: &dyn Fn(usize) -> Truth, faults: &Faults, frames: usize) -> Baseline {
    let mut deep = fresh_areas();
    let mut names: Vec<String> = Vec::new();
    let mut out_frames = Vec::with_capacity(frames);
    for frame in 0..frames {
        let mut values = Vec::with_capacity(names.len());
        let mut seen: Vec<String> = Vec::new();
        let first = names.is_empty();
        deep.tick(truth_at(frame), faults, &mut |name, value| {
            values.push(value);
            if first {
                seen.push(name.to_owned());
            }
        });
        if first {
            names = seen;
        }
        out_frames.push(values);
    }
    Baseline { names, frames: out_frames }
}

#[derive(Clone, Debug, Default)]
pub struct Diff {
    pub changed: Vec<usize>,
    pub first_frame: Option<usize>,
    pub shape_changed: bool,
}

impl Diff {
    pub fn is_empty(&self) -> bool {
        self.changed.is_empty() && !self.shape_changed
    }
}

pub fn diff_against(base: &Baseline, truth: &Truth, faults: &Faults) -> Diff {
    diff_frames(base, &|_| truth.clone(), faults, false)
}

pub fn first_diff(base: &Baseline, truth: &Truth, faults: &Faults) -> Diff {
    diff_frames(base, &|_| truth.clone(), faults, true)
}

pub fn first_diff_phased(base: &Baseline, truth_at: &dyn Fn(usize) -> Truth, faults: &Faults) -> Diff {
    diff_frames(base, truth_at, faults, true)
}

fn diff_frames(base: &Baseline, truth_at: &dyn Fn(usize) -> Truth, faults: &Faults, stop_at_first: bool) -> Diff {
    let mut deep = fresh_areas();
    let mut diff = Diff::default();
    let mut changed = vec![false; base.names.len()];
    for (frame, expected) in base.frames.iter().enumerate() {
        let mut i = 0usize;
        let mut shape = false;
        let mut differed = false;
        deep.tick(truth_at(frame), faults, &mut |name, value| {
            if i >= base.names.len() || base.names[i] != name {
                shape = true;
            } else if !same(expected[i], value) {
                changed[i] = true;
                differed = true;
            }
            i += 1;
        });
        if i != base.names.len() {
            shape = true;
        }
        if shape {
            diff.shape_changed = true;
        }
        if (differed || shape) && diff.first_frame.is_none() {
            diff.first_frame = Some(frame);
            if stop_at_first {
                break;
            }
        }
    }
    diff.changed = changed.iter().enumerate().filter(|(_, &c)| c).map(|(i, _)| i).collect();
    diff
}

fn same(a: f64, b: f64) -> bool {
    a == b || (a.is_nan() && b.is_nan())
}

#[derive(Clone, Debug)]
pub struct Verdict {
    pub id: u64,
    pub area: RegArea,
    pub ata: u16,
    pub name: String,
    pub component: String,
    pub model_field: String,
    pub alive_in: Option<(&'static str, f64)>,
    pub moved: Vec<String>,
    pub moved_count: usize,
}

impl Verdict {
    pub fn is_live(&self) -> bool {
        self.alive_in.is_some()
    }
}

pub const MAGNITUDES: [f64; 2] = [1.0, 0.35];

fn verdict_for(f: &FailureDef, profiles: &[Profile], baselines: &[Baseline], truths: &[Truth]) -> Verdict {
    let mut v = Verdict { id: f.id, area: f.area, ata: f.ata, name: f.name.clone(), component: f.component.clone(), model_field: f.model_field.clone(), alive_in: None, moved: Vec::new(), moved_count: 0 };
    'search: for (p, profile) in profiles.iter().enumerate() {
        for m in MAGNITUDES {
            let d = diff_against(&baselines[p], &truths[p], &armed_with(f.id, m));
            if !d.is_empty() {
                v.alive_in = Some((profile.name, m));
                v.moved_count = d.changed.len();
                v.moved = d.changed.iter().take(6).map(|&i| baselines[p].names[i].clone()).collect();
                if d.shape_changed {
                    v.moved.push("<published name set changed>".into());
                }
                break 'search;
            }
        }
    }
    v
}

pub fn worker_threads() -> usize {
    std::thread::available_parallelism().map(|n| n.get().min(8)).unwrap_or(4).max(1)
}

pub fn sweep(failures: &[FailureDef], progress: &mut (dyn FnMut(usize, usize, usize) + Send)) -> Vec<Verdict> {
    sweep_over(&profiles(), failures, progress)
}

pub fn sweep_over(profiles: &[Profile], failures: &[FailureDef], progress: &mut (dyn FnMut(usize, usize, usize) + Send)) -> Vec<Verdict> {
    use std::sync::atomic::{AtomicUsize, Ordering};

    let threads = worker_threads();
    let done = AtomicUsize::new(0);
    let dead = AtomicUsize::new(0);
    let total = failures.len();
    let chunk = total.div_ceil(threads).max(1);

    let progress = std::sync::Mutex::new(progress);
    let mut out: Vec<Verdict> = std::thread::scope(|scope| {
        let handles: Vec<_> = failures
            .chunks(chunk)
            .map(|slice| {
                let done = &done;
                let dead = &dead;
                let progress = &progress;
                scope.spawn(move || {
                    let baselines: Vec<Baseline> = profiles.iter().map(|p| baseline(&(p.truth)(), &reference_faults(), p.frames)).collect();
                    let truths: Vec<Truth> = profiles.iter().map(|p| (p.truth)()).collect();
                    let mut mine = Vec::with_capacity(slice.len());
                    for f in slice {
                        let v = verdict_for(f, profiles, &baselines, &truths);
                        if !v.is_live() {
                            dead.fetch_add(1, Ordering::Relaxed);
                        }
                        let n = done.fetch_add(1, Ordering::Relaxed) + 1;
                        if n % 200 == 0 {
                            if let Ok(mut p) = progress.lock() {
                                p(n, total, dead.load(Ordering::Relaxed));
                            }
                        }
                        mine.push(v);
                    }
                    mine
                })
            })
            .collect();
        handles.into_iter().flat_map(|h| h.join().expect("a sweep worker panicked")).collect()
    });
    out.sort_by_key(|v| v.id);
    out
}

pub fn cond_vars(c: &Cond, into: &mut Vec<String>) {
    match c {
        Cond::Always => {}
        Cond::Var { name, .. } => into.push(name.clone()),
        Cond::VarVar { a, b, .. } => {
            into.push(a.clone());
            into.push(b.clone());
        }
        Cond::And(v) | Cond::Or(v) => v.iter().for_each(|x| cond_vars(x, into)),
        Cond::Not(x) => cond_vars(x, into),
    }
}

pub const NAME_PREFIX: &str = "A32NX_";

pub fn bare(name: &str) -> &str {
    name.strip_prefix(NAME_PREFIX).unwrap_or(name)
}

pub fn trigger_vars(a: &EcamAlert) -> Vec<String> {
    let mut v = Vec::new();
    cond_vars(&a.trigger, &mut v);
    let mut v: Vec<String> = v.iter().map(|n| bare(n).to_owned()).collect();
    v.sort();
    v.dedup();
    v
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tri {
    Never,
    Always,
    Reachable,
}

pub fn reachability(c: &Cond, known: &dyn Fn(&str) -> bool) -> Tri {
    fn fixed(cmp: crate::deep::api::Cmp, x: f64, y: f64) -> Tri {
        use crate::deep::api::Cmp::*;
        let t = match cmp {
            Lt => x < y,
            Le => x <= y,
            Gt => x > y,
            Ge => x >= y,
            Eq => (x - y).abs() < 1e-9,
            Ne => (x - y).abs() >= 1e-9,
        };
        if t {
            Tri::Always
        } else {
            Tri::Never
        }
    }
    match c {
        Cond::Always => Tri::Always,
        Cond::Var { name, cmp, value } => {
            if known(bare(name)) {
                Tri::Reachable
            } else {
                fixed(*cmp, 0.0, *value)
            }
        }
        Cond::VarVar { a, cmp, b } => {
            if known(bare(a)) || known(bare(b)) {
                Tri::Reachable
            } else {
                fixed(*cmp, 0.0, 0.0)
            }
        }
        Cond::And(v) => {
            let parts: Vec<Tri> = v.iter().map(|x| reachability(x, known)).collect();
            if parts.iter().any(|&t| t == Tri::Never) {
                Tri::Never
            } else if parts.iter().all(|&t| t == Tri::Always) {
                Tri::Always
            } else {
                Tri::Reachable
            }
        }
        Cond::Or(v) => {
            let parts: Vec<Tri> = v.iter().map(|x| reachability(x, known)).collect();
            if parts.iter().any(|&t| t == Tri::Always) {
                Tri::Always
            } else if parts.iter().all(|&t| t == Tri::Never) {
                Tri::Never
            } else {
                Tri::Reachable
            }
        }
        Cond::Not(x) => match reachability(x, known) {
            Tri::Never => Tri::Always,
            Tri::Always => Tri::Never,
            Tri::Reachable => Tri::Reachable,
        },
    }
}

#[derive(Clone, Debug)]
pub struct UnpublishedTrigger {
    pub key: String,
    pub title: String,
    pub ata: u16,
    pub missing: Vec<String>,
    pub present: Vec<String>,
}

pub fn alerts_reading_unpublished(alerts: &[EcamAlert], published: &std::collections::BTreeSet<String>) -> Vec<UnpublishedTrigger> {
    let mut out = Vec::new();
    for a in alerts {
        let vars = trigger_vars(a);
        let (present, missing): (Vec<String>, Vec<String>) = vars.into_iter().partition(|v| published.contains(v));
        if !missing.is_empty() {
            out.push(UnpublishedTrigger { key: a.key.clone(), title: a.title.clone(), ata: a.ata, missing, present });
        }
    }
    out
}

pub const PLUGIN_OWNED_TRIGGER_VARS: &[&str] = &[
    "A32NX_OVHD_APU_START_PB_IS_ON",
    "A32NX_OVHD_APU_MASTER_SW_PB_IS_ON",
    "A32NX_ENGINE_STATE:1",
    "A32NX_ENGINE_STATE:2",
    "A32NX_ENGINE_STATE:3",
    "A32NX_ENGINE_STATE:4",
];

pub fn trigger_verdicts(alerts: &[EcamAlert], published: &std::collections::BTreeSet<String>) -> Vec<(String, Tri, Vec<String>)> {
    let known = |n: &str| published.contains(n) || PLUGIN_OWNED_TRIGGER_VARS.contains(&n);
    alerts
        .iter()
        .map(|a| {
            let t = reachability(&a.trigger, &known);
            let missing: Vec<String> = trigger_vars(a).into_iter().filter(|v| !known(v)).collect();
            (a.key.clone(), t, missing)
        })
        .collect()
}

pub fn components_without_failures(r: &crate::deep::api::Registry) -> Vec<String> {
    let named: std::collections::BTreeSet<&str> = r.failures.iter().map(|f| f.component.as_str()).collect();
    r.components.iter().filter(|c| c.failures.is_empty() && !named.contains(c.id.as_str())).map(|c| c.id.clone()).collect()
}

fn source_outside_failures_rs() -> String {
    fn walk(dir: &std::path::Path, out: &mut String) {
        let Ok(entries) = std::fs::read_dir(dir) else { return };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                walk(&path, out);
            } else if path.extension().is_some_and(|e| e == "rs")
                && path.file_name().is_some_and(|n| n != "failures.rs" && n != "failure_audit.rs")
            {
                if let Ok(s) = std::fs::read_to_string(&path) {
                    out.push_str(&s);
                    out.push('\n');
                }
            }
        }
    }
    let mut out = String::new();
    walk(std::path::Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/src")), &mut out);
    out
}

pub struct LegacyHookFamily {
    pub var: String,
    pub owner: &'static str,
    pub ids: Vec<u64>,
    pub referenced_elsewhere: bool,
    pub dead_ids: Vec<u64>,
}

pub fn legacy_hook_families(source: &str) -> Vec<LegacyHookFamily> {
    use crate::failures::extra::{extra_failures, Effect};
    let mut by_var: BTreeMap<String, (&'static str, Vec<u64>)> = BTreeMap::new();
    for f in extra_failures() {
        if let Effect::Hook { var, owner } = f.effect {
            by_var.entry(var.to_owned()).or_insert_with(|| (owner.label(), Vec::new())).1.push(f.id);
        }
    }
    by_var
        .into_iter()
        .map(|(var, (owner, mut ids))| {
            ids.sort_unstable();
            let needle = format!("\"{var}\"");
            let referenced_elsewhere = source.contains(&needle);
            let dead_ids: Vec<u64> = ids.iter().copied().filter(|&id| !id_literal_referenced(source, id)).collect();
            LegacyHookFamily { var, owner, ids, referenced_elsewhere, dead_ids }
        })
        .collect()
}

pub fn id_literal_referenced(source: &str, id: u64) -> bool {
    let plain = id.to_string();
    let mut grouped = String::new();
    for (i, b) in plain.bytes().enumerate() {
        if i > 0 && (plain.len() - i) % 3 == 0 {
            grouped.push('_');
        }
        grouped.push(b as char);
    }
    [plain, grouped].iter().any(|needle| {
        source.match_indices(needle.as_str()).any(|(i, m)| {
            let before_digit = source[..i].chars().next_back().is_some_and(|c| c.is_ascii_digit());
            let after_digit = source[i + m.len()..].chars().next().is_some_and(|c| c.is_ascii_digit());
            !before_digit && !after_digit
        })
    })
}

fn family(v: &Verdict) -> String {
    let field = v.model_field.split_once('.').map_or(v.model_field.clone(), |(s, f)| format!("{s}.{f}"));
    format!("{:?}|{}|{}", v.area, v.ata, field)
}

pub fn report(verdicts: &[Verdict]) -> String {
    use std::fmt::Write;
    let mut s = String::new();
    let live = verdicts.iter().filter(|v| v.is_live()).count();
    let dead = verdicts.len() - live;
    let _ = writeln!(s, "= DEEP FAILURE AUDIT =");
    let _ = writeln!(s, "registered {} | move something published {} | move nothing {}", verdicts.len(), live, dead);

    let mut per_area: BTreeMap<String, (usize, usize)> = BTreeMap::new();
    for v in verdicts {
        let e = per_area.entry(format!("{:?}", v.area)).or_default();
        e.0 += 1;
        if !v.is_live() {
            e.1 += 1;
        }
    }
    let _ = writeln!(s, "\n-- per area: total / dead --");
    for (a, (t, d)) in &per_area {
        let _ = writeln!(s, "{a:<20} {t:>6} {d:>6}");
    }

    let mut by_profile: BTreeMap<&str, usize> = BTreeMap::new();
    for v in verdicts {
        if let Some((p, _)) = v.alive_in {
            *by_profile.entry(p).or_default() += 1;
        }
    }
    let _ = writeln!(s, "\n-- live failures by the first profile that showed them --");
    for (p, n) in &by_profile {
        let _ = writeln!(s, "{p:<28} {n:>6}");
    }

    let mut fams: BTreeMap<String, Vec<&Verdict>> = BTreeMap::new();
    for v in verdicts.iter().filter(|v| !v.is_live()) {
        fams.entry(family(v)).or_default().push(v);
    }
    let mut ordered: Vec<_> = fams.into_iter().collect();
    ordered.sort_by_key(|(_, v)| std::cmp::Reverse(v.len()));
    let _ = writeln!(s, "\n-- dead families (area | ata | model field), largest first: {} families --", ordered.len());
    for (k, vs) in &ordered {
        let _ = writeln!(s, "{:>5}  {}   e.g. {} [{}] id {}", vs.len(), k, vs[0].name, vs[0].component, vs[0].id);
    }

    let _ = writeln!(s, "\n-- every dead failure --");
    for v in verdicts.iter().filter(|v| !v.is_live()) {
        let _ = writeln!(s, "{} {:?} ata{} | {} | {} | {}", v.id, v.area, v.ata, v.name, v.component, v.model_field);
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;

    fn report_path(name: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(name)
    }

    #[test]
    fn a_healthy_run_is_deterministic_so_any_difference_is_the_failure() {
        let t = cruise();
        let base = baseline(&t, &reference_faults(), 5);
        let again = diff_against(&base, &t, &reference_faults());
        assert!(again.is_empty(), "{} variables differ between two identical healthy runs, e.g. {:?}", again.changed.len(), again.changed.iter().take(5).map(|&i| &base.names[i]).collect::<Vec<_>>());
        assert!(!base.names.is_empty());
    }

    #[test]
    fn an_id_no_area_owns_moves_nothing_which_is_what_a_dead_failure_looks_like() {
        let t = cruise();
        let base = baseline(&t, &reference_faults(), 5);
        let d = diff_against(&base, &t, &armed_with(999_999_998, 1.0));
        assert!(d.is_empty(), "an unregistered failure id moved {} variables, e.g. {:?}", d.changed.len(), d.changed.iter().take(6).map(|&i| &base.names[i]).collect::<Vec<_>>());

        let empty = baseline(&t, &Faults::default(), 5);
        let shortcut = diff_against(&empty, &t, &Faults::from_pairs([(999_999_998, 1.0)]));
        println!("AUDIT faults.any() branch moves {} published variables on its own", shortcut.changed.len());
    }

    #[test]
    fn a_sample_from_every_area_still_moves_something_published() {
        let r = crate::deep::registry();
        let mut seen: std::collections::BTreeSet<String> = Default::default();
        let mut sample: Vec<FailureDef> = Vec::new();
        for f in &r.failures {
            if seen.insert(format!("{:?}", f.area)) {
                sample.push(f.clone());
            }
        }
        let quick = vec![Profile { name: "cruise", truth: cruise, frames: 8 }, Profile { name: "all_commands_exercised", truth: all_commands_exercised, frames: 16 }];
        let t0 = Instant::now();
        let verdicts = sweep_over(&quick, &sample, &mut |_, _, _| {});
        let live = verdicts.iter().filter(|v| v.is_live()).count();
        let mut areas_all_dead: BTreeMap<String, (usize, usize)> = BTreeMap::new();
        for v in &verdicts {
            let e = areas_all_dead.entry(format!("{:?}", v.area)).or_default();
            e.0 += 1;
            if v.is_live() {
                e.1 += 1;
            }
        }
        println!("AUDIT sample {} areas of {} registered failures: live {}/{} in {:.1} s", sample.len(), r.failures.len(), live, verdicts.len(), t0.elapsed().as_secs_f64());
        for (a, (t, l)) in &areas_all_dead {
            println!("AUDIT   {a:<20} live {l:>3} / {t:>3}");
        }
        for v in verdicts.iter().filter(|v| !v.is_live()) {
            println!("AUDIT   dead sample {} {:?} ata{} {} | {} | {}", v.id, v.area, v.ata, v.name, v.component, v.model_field);
        }
        assert!(live * 2 >= verdicts.len(), "over half the one-per-area sample moves nothing published: {live} of {}", verdicts.len());
    }

    #[test]
    fn what_the_diagnostic_variables_read_in_each_profile() {
        let names = ["BREAKERS_TOTAL", "BREAKERS_OPEN_COUNT", "BREAKERS_PROTECTING_NO_MODELLED_LOAD", "BREAKERS_LOCKED_OUT_COUNT", "FUEL_TOTAL_TRUE_FOB_KG", "DEEP_APU_N", "APU_N", "FIRE_DETECTED_ENG:1"];
        for p in profiles() {
            let base = baseline(&(p.truth)(), &reference_faults(), p.frames);
            let last = base.frames.last().expect("every profile runs at least one frame");
            let mut shown = Vec::new();
            for n in names {
                if let Some(i) = base.names.iter().position(|x| bare(x) == n) {
                    shown.push(format!("{n}={}", last[i]));
                }
            }
            println!("AUDIT profile {:<24} {} published | {}", p.name, base.names.len(), shown.join(" "));
            let open: Vec<&str> = base
                .names
                .iter()
                .enumerate()
                .filter(|(i, n)| n.starts_with("BKR_") && n.ends_with("_OPEN") && last[*i] != 0.0)
                .map(|(_, n)| n.as_str())
                .collect();
            if !open.is_empty() {
                println!("AUDIT   healthy-but-open breakers {}: {}", open.len(), open.iter().take(12).cloned().collect::<Vec<_>>().join(" "));
            }
        }
    }

    #[test]
    #[ignore = "minutes: the whole catalogue against the whole profile set"]
    fn deep_failure_audit_full_sweep() {
        let r = crate::deep::registry();
        let t0 = Instant::now();
        let verdicts = sweep(&r.failures, &mut |n, total, dead| {
            println!("AUDIT {n}/{total} dead so far {dead} ({:.0} s)", t0.elapsed().as_secs_f64());
        });
        let text = report(&verdicts);
        let path = report_path("deep_failure_audit.txt");
        std::fs::write(&path, &text).expect("write the audit report");
        for l in text.lines().take_while(|l| !l.starts_with("-- every dead failure")) {
            println!("AUDIT {l}");
        }
        println!("AUDIT full report at {}", path.display());
        println!("AUDIT took {:.0} s", t0.elapsed().as_secs_f64());
    }

    #[test]
    fn ecam_triggers_that_read_a_variable_no_area_publishes() {
        let r = crate::deep::registry();
        let published: std::collections::BTreeSet<String> = all_areas().published_names().iter().map(|n| bare(n).to_owned()).collect();
        let verdicts = trigger_verdicts(&r.alerts, &published);
        let never: Vec<&(String, Tri, Vec<String>)> = verdicts.iter().filter(|v| v.1 == Tri::Never).collect();
        let always: Vec<&(String, Tri, Vec<String>)> = verdicts.iter().filter(|v| v.1 == Tri::Always).collect();
        let partial: Vec<&(String, Tri, Vec<String>)> = verdicts.iter().filter(|v| v.1 == Tri::Reachable && !v.2.is_empty()).collect();

        let mut every_missing: BTreeMap<String, usize> = BTreeMap::new();
        for v in &verdicts {
            for m in &v.2 {
                *every_missing.entry(m.clone()).or_default() += 1;
            }
        }
        println!("AUDIT alerts {} | can NEVER fire {} | ALWAYS on {} | partly blind (an OR arm is dead) {} | distinct unpublished names {}", r.alerts.len(), never.len(), always.len(), partial.len(), every_missing.len());
        let title = |k: &str| r.alerts.iter().find(|a| a.key == k).map_or(String::new(), |a| a.title.clone());
        for (k, _, missing) in &never {
            println!("AUDIT never-fires {k} \"{}\" needs {missing:?}", title(k));
        }
        for (k, _, missing) in &always {
            println!("AUDIT always-on  {k} \"{}\" because {missing:?} read as 0", title(k));
        }
        for (k, _, missing) in &partial {
            println!("AUDIT partly-dead {k} \"{}\" dead arm needs {missing:?}", title(k));
        }

        let mut text = String::new();
        text.push_str(&format!("alerts {} | never fires {} | always on {} | partly dead {}

", r.alerts.len(), never.len(), always.len(), partial.len()));
        text.push_str("-- variables a trigger reads that nothing publishes, and how many alerts want them --
");
        for (m, n) in &every_missing {
            text.push_str(&format!("{n:>4}  {m}
"));
        }
        for (label, set) in [("NEVER FIRES", &never), ("ALWAYS ON", &always), ("PARTLY DEAD", &partial)] {
            text.push_str(&format!("
-- {label} --
"));
            for (k, _, missing) in set.iter() {
                text.push_str(&format!("{k} \"{}\"  missing {missing:?}
", title(k)));
            }
        }
        let path = report_path("deep_ecam_unpublished.txt");
        std::fs::write(&path, text).expect("write the ECAM report");
        println!("AUDIT ECAM report at {}", path.display());
    }

    #[test]
    #[ignore = "minutes: every failure any alert names, through every profile"]
    fn alerts_whose_named_causes_cannot_move_their_trigger() {
        let r = crate::deep::registry();
        let mut wanted: BTreeMap<u64, std::collections::BTreeSet<String>> = BTreeMap::new();
        for a in &r.alerts {
            let vars = trigger_vars(a);
            for id in &a.failures {
                wanted.entry(*id).or_default().extend(vars.iter().cloned());
            }
        }
        let total = wanted.len();
        let work: Vec<(u64, std::collections::BTreeSet<String>)> = wanted.into_iter().collect();
        let threads = worker_threads();
        let chunk = work.len().div_ceil(threads).max(1);
        println!("AUDIT cause-check {total} failures named by an alert, {threads} workers");
        let mut inert: Vec<(u64, Vec<String>)> = std::thread::scope(|scope| {
            let handles: Vec<_> = work
                .chunks(chunk)
                .map(|slice| {
                    scope.spawn(move || {
                        let profiles = profiles();
                        let baselines: Vec<Baseline> = profiles.iter().map(|p| baseline(&(p.truth)(), &reference_faults(), p.frames)).collect();
                        let truths: Vec<Truth> = profiles.iter().map(|p| (p.truth)()).collect();
                        let mut mine: Vec<(u64, Vec<String>)> = Vec::new();
                        for (id, vars) in slice {
                            let mut reached = false;
                            'search: for p in 0..profiles.len() {
                                for m in MAGNITUDES {
                                    let d = diff_against(&baselines[p], &truths[p], &armed_with(*id, m));
                                    if d.changed.iter().any(|&i| vars.contains(bare(&baselines[p].names[i]))) {
                                        reached = true;
                                        break 'search;
                                    }
                                }
                            }
                            if !reached {
                                mine.push((*id, vars.iter().cloned().collect()));
                            }
                        }
                        mine
                    })
                })
                .collect();
            handles.into_iter().flat_map(|h| h.join().expect("a cause-check worker panicked")).collect()
        });
        inert.sort_by_key(|(id, _)| *id);
        println!("AUDIT failures named as a cause of some alert: {total} | that never move any of that alert's trigger variables: {}", inert.len());
        let mut text = String::new();
        for (id, vars) in &inert {
            let f = r.failures.iter().find(|f| f.id == *id);
            let line = match f {
                Some(f) => format!("{id} {:?} ata{} | {} | {} | wanted {:?}
", f.area, f.ata, f.name, f.model_field, vars),
                None => format!("{id} <not registered> wanted {vars:?}
"),
            };
            print!("AUDIT inert-cause {line}");
            text.push_str(&line);
        }
        let path = report_path("deep_alert_causes.txt");
        std::fs::write(&path, text).expect("write the alert-cause report");
        println!("AUDIT alert-cause report at {}", path.display());
    }

    #[test]
    fn components_nothing_can_break_and_alerts_nothing_raises() {
        let r = crate::deep::registry();
        let orphans = components_without_failures(&r);
        println!("AUDIT components {} | with no failure at all {}", r.components.len(), orphans.len());
        for c in orphans.iter().take(80) {
            println!("AUDIT   component with no failures: {c}");
        }
        let unraised: Vec<&str> = r.alerts.iter().filter(|a| a.failures.is_empty()).map(|a| a.key.as_str()).collect();
        println!("AUDIT alerts {} | with no raised_by failure {}", r.alerts.len(), unraised.len());
        for k in unraised.iter().take(80) {
            println!("AUDIT   alert with no raised_by: {k}");
        }
    }

    #[test]
    fn legacy_catalogue_hook_families_are_audited() {
        let source = source_outside_failures_rs();
        let families = legacy_hook_families(&source);
        assert!(
            families.is_empty(),
            "crate::failures::extra::extra_failures() (this crate's own lib.rs) is the only definition of that function anywhere in the repository (checked via `git grep` across the full HEAD tree, not just this sparse worktree) and it unconditionally returns an empty Vec -- there is no legacy Effect::Hook catalogue left to audit. FAIL_BUS_SHORT_HOOK / FAIL_ENGINE_COMPONENT_HOOK / FAIL_FUEL_HOOK, and the files this test used to cite (breakers.rs's `24_200 + k`, engine_commands.rs, fuel_network.rs) do not exist under those names anywhere in this repository either. If this now fails, someone populated extra_failures() with real Effect::Hook entries: restore the per-family dead-id assertions this test carried before (git blame this test) against that real data instead of this guard."
        );
    }
}
