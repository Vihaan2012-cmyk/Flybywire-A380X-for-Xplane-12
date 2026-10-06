use std::collections::BTreeMap;

pub use super::frame::{DerivedFailure, Faults, PublishedFrame};

use super::flight_controls::live::SurfaceAngles;
use super::integration::weather_truth::EnvironmentTruth;


pub const DOOR_NAMES: [&str; 13] = ["M1L", "M2L", "M2R", "M4L", "M5L", "U1L", "U1R", "U2L", "U2R", "U3L", "U3R", "CARGO_FWD", "CARGO_AFT"];

#[derive(Clone, Debug)]
pub struct Truth {
    pub dt_s: f64,
    pub environment: EnvironmentTruth,
    pub altitude_ft: f64,
    pub on_ground: bool,
    pub engine_n1_frac: [f64; 4],
    pub engine_running: [bool; 4],
    pub engine_bleed_pressure_pa: [f64; 4],
    pub engine_bleed_temp_k: [f64; 4],
    pub engine_oil_pressure_pa: [f64; 4],
    pub engine_oil_temp_c: [f64; 4],
    pub engine_oil_quantity_fraction: [f64; 4],
    pub engine_oil_filter_bypassed: [bool; 4],
    pub engine_tgt_c: [f64; 4],
    pub engine_t25_c: [f64; 4],
    pub tyre_pressure_pa: [f64; crate::physics::tyre::WHEELS],
    pub tyre_temp_c: [f64; crate::physics::tyre::WHEELS],
    pub door_open_fraction: [f64; DOOR_NAMES.len()],
    pub apu_running: bool,
    pub apu_bleed_pressure_pa: f64,
    pub ac_bus_volts: [f64; 4],
    pub dc_bus_volts: [f64; 2],
    pub ac_bus_powered: [bool; 4],
    pub dc_bus_powered: [bool; 2],
    pub prim_healthy: [bool; 3],
    pub sec_healthy: [bool; 3],
    pub prim_left_sidestick_disabled: bool,
    pub prim_right_sidestick_disabled: bool,
    pub prim_left_sidestick_priority_locked: bool,
    pub prim_right_sidestick_priority_locked: bool,
    pub flap_lever_handle_index: f64,
    pub capt_sidestick_pitch_raw: f64,
    pub capt_sidestick_roll_raw: f64,
    pub rudder_pedal_raw: f64,
    pub body_rate_pitch_raw: f64,
    pub body_rate_yaw_raw: f64,
    pub body_rate_roll_raw: f64,
    pub hydraulic_pressure_pa: [f64; 2],
    pub engine_n2_frac: [f64; 4],
    pub engine_n3_frac: [f64; 4],
    pub engine_n2_healthy_frac: [f64; 4],
    pub engine_n3_healthy_frac: [f64; 4],
    pub engine_n1_commanded_pct: [f64; 4],
    pub aircraft_preset_quick_mode: bool,
    pub engine_customer_bleed_kg_s: [f64; 4],
    pub sim_engine_corrected_n1_pct: [f64; 4],
    pub sim_engine_corrected_n2_pct: [f64; 4],
    pub engine_hp_port_pressure_pa: [f64; 4],
    pub engine_hp_port_temp_k: [f64; 4],
    pub engine_ip_port_pressure_pa: [f64; 4],
    pub engine_ip_port_temp_k: [f64; 4],
    pub engine_fuel_flow_kg_s: [f64; 4],
    pub engine_tla_deg: [f64; 4],
    pub to_flex_temp_set: bool,
    pub fuel_tank_quantity_gal: Option<[f64; 11]>,
    pub gpu_plugged_in: bool,
    pub controls: Controls,
    pub commanded_surfaces: CommandedSurfaces,
    pub rudder_trim_cmd_deg: f64,
    pub flap_cmd_deg: f64,
    pub slat_cmd_deg: f64,
    pub droop_cmd_deg: f64,
    pub aircraft_mass_kg: f64,
    pub pitch_deg: f64,
    pub roll_deg: f64,
    pub heading_true_deg: f64,
    pub groundspeed_m_s: f64,
    pub angle_of_attack_deg: f64,
    pub radio_height_ft: f64,
    pub leg_on_ground: [bool; 5],
    pub leg_touchdown_sink_speed_ms: [f64; 5],
    pub cabin_pressure_pa: f64,
    pub cabin_temp_k: f64,
    pub sun_elevation_deg: f64,
    pub fdac_channel_failure: [[bool; 2]; 2],
    pub ocsm_channel_failure: [[bool; 2]; 4],
    pub vertical_speed_fpm: f64,
    pub landing_elevation_ft: f64,
    pub athr_status: f64,
    pub ap1_active: bool,
    pub ap2_active: bool,
    pub fmgc_flight_phase: f64,
    pub flight_ready: bool,
    pub athr_eng_fault: [bool; 4],
    pub pack_flow_insufficient_fwd_crg: bool,
    pub ir: [IrOutputs; 3],
    pub att_hdg_switching_knob: f64,
    pub published: PublishedFrame,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Controls {
    pub fire_pb_released: [bool; 4],
    pub fire_pb_apu_released: bool,
    pub fire_agent_pb_pressed: [[bool; 2]; 4],
    pub fire_agent_pb_apu_pressed: bool,
    pub cargo_agent_pb_pressed: [bool; 2],
    pub wing_anti_ice_selected: bool,
    pub nacelle_anti_ice_selected: [bool; 4],
    pub engine_bleed_pb_auto: [bool; 4],
    pub apu_bleed_pb_on: bool,
    pub cross_bleed_selector: f64,
    pub pack_pb_on: [bool; 2],
    pub starter_engaged: [bool; 4],
    pub rain_removal_selected: [bool; 2],
    pub gear_door_commanded_open: [f64; 3],
    pub gear_lever_down: bool,
    pub parking_brake_on: bool,
    pub remote_cb_ctl_active: bool,
    pub steering_command_deg: [f64; 3],
    pub jettison_armed: bool,
    pub jettison_valve_selected: [bool; 2],
    pub crossfeed_valve_selected: [bool; 4],
    pub cargo_door_commanded_open: [f64; 3],
    pub water_demand_l_s: [f64; 2],
    pub brake_pedal_pos: [f64; 2],
    pub engine_master_on: [bool; 4],
    pub eng_gen_pb_on: [bool; 4],
    pub baro_mode: [f64; 2],
    pub apu_gen_pb_on: [bool; 2],
    pub bat_pb_auto: [bool; 2],
    pub ground_spoiler_lever_armed: bool,
    pub apu_master_sw_on: bool,
    pub apu_start_pb_on: bool,
    pub reverser_deploy_commanded: [bool; 2],
    pub fcu_switch_off: bool,
    pub gravity_extend_selected: bool,
    pub nw_steer_disc_selected: bool,
}

impl Default for Controls {
    fn default() -> Self {
        Self {
            fire_pb_released: [false; 4],
            fire_pb_apu_released: false,
            fire_agent_pb_pressed: [[false; 2]; 4],
            fire_agent_pb_apu_pressed: false,
            cargo_agent_pb_pressed: [false; 2],
            wing_anti_ice_selected: false,
            nacelle_anti_ice_selected: [false; 4],
            engine_bleed_pb_auto: [true; 4],
            apu_bleed_pb_on: false,
            cross_bleed_selector: 1.0,
            pack_pb_on: [true; 2],
            starter_engaged: [false; 4],
            rain_removal_selected: [false; 2],
            steering_command_deg: [0.0; 3],
            jettison_armed: false,
            jettison_valve_selected: [false; 2],
            crossfeed_valve_selected: [false; 4],
            cargo_door_commanded_open: [0.0; 3],
            water_demand_l_s: [0.0; 2],
            gear_door_commanded_open: [0.0; 3],
            gear_lever_down: true,
            parking_brake_on: true,
            remote_cb_ctl_active: false,
            brake_pedal_pos: [0.0; 2],
            engine_master_on: [false; 4],
            eng_gen_pb_on: [true; 4],
            baro_mode: [0.0; 2],
            apu_gen_pb_on: [true; 2],
            bat_pb_auto: [true; 2],
            ground_spoiler_lever_armed: false,
            apu_master_sw_on: false,
            apu_start_pb_on: false,
            reverser_deploy_commanded: [false; 2],
            fcu_switch_off: false,
            gravity_extend_selected: false,
            nw_steer_disc_selected: false,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CommandedSurfaces {
    pub ailerons_deg: [[f64; 3]; 2],
    pub elevators_deg: [[f64; 2]; 2],
    pub rudders_deg: [f64; 2],
    pub spoilers_deg: [[f64; 8]; 2],
    pub ths_deg: f64,
}

impl Default for Truth {
    fn default() -> Self {
        Self {
            dt_s: 1.0 / 30.0,
            environment: EnvironmentTruth {
                sat_c: 15.0,
                leading_edge_c: 15.0,
                ambient_pressure_pa: 101_325.0,
                tas_ms: 0.0,
                precipitation_on_aircraft_ratio: 0.0,
                weather: None,
            },
            altitude_ft: 0.0,
            on_ground: true,
            flight_ready: true,
            engine_n1_frac: [0.0; 4],
            engine_running: [false; 4],
            engine_bleed_pressure_pa: [101_325.0; 4],
            engine_bleed_temp_k: [288.15; 4],
            tyre_pressure_pa: [crate::physics::tyre::COLD_PRESSURE_PA; crate::physics::tyre::WHEELS],
            tyre_temp_c: [15.0; crate::physics::tyre::WHEELS],
            engine_oil_pressure_pa: [0.0; 4],
            engine_oil_temp_c: [15.0; 4],
            engine_oil_quantity_fraction: [1.0; 4],
            engine_oil_filter_bypassed: [false; 4],
            engine_tgt_c: [15.0; 4],
            engine_t25_c: [15.0; 4],
            door_open_fraction: [0.0; DOOR_NAMES.len()],
            apu_running: false,
            apu_bleed_pressure_pa: 101_325.0,
            ac_bus_volts: [0.0; 4],
            dc_bus_volts: [0.0; 2],
            ac_bus_powered: [false; 4],
            dc_bus_powered: [false; 2],
            prim_healthy: [false; 3],
            sec_healthy: [false; 3],
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
            hydraulic_pressure_pa: [0.0; 2],
            engine_n2_frac: [0.0; 4],
            engine_n3_frac: [0.0; 4],
            engine_n2_healthy_frac: [0.0; 4],
            engine_n3_healthy_frac: [0.0; 4],
            engine_n1_commanded_pct: [0.0; 4],
            aircraft_preset_quick_mode: false,
            engine_customer_bleed_kg_s: [0.0; 4],
            sim_engine_corrected_n1_pct: [0.0; 4],
            sim_engine_corrected_n2_pct: [0.0; 4],
            engine_hp_port_pressure_pa: [101_325.0; 4],
            engine_hp_port_temp_k: [288.15; 4],
            engine_ip_port_pressure_pa: [101_325.0; 4],
            engine_ip_port_temp_k: [288.15; 4],
            engine_fuel_flow_kg_s: [0.0; 4],
            engine_tla_deg: [0.0; 4],
            to_flex_temp_set: false,
            fuel_tank_quantity_gal: None,
            gpu_plugged_in: false,
            controls: Controls::default(),
            commanded_surfaces: CommandedSurfaces::default(),
            rudder_trim_cmd_deg: 0.0,
            flap_cmd_deg: 0.0,
            slat_cmd_deg: 0.0,
            droop_cmd_deg: 0.0,
            aircraft_mass_kg: 277_000.0,
            pitch_deg: 0.0,
            roll_deg: 0.0,
            heading_true_deg: 0.0,
            groundspeed_m_s: 0.0,
            angle_of_attack_deg: 0.0,
            radio_height_ft: 0.0,
            leg_on_ground: [true; 5],
            leg_touchdown_sink_speed_ms: [0.0; 5],
            cabin_pressure_pa: 101_325.0,
            cabin_temp_k: 288.15,
            sun_elevation_deg: 0.0,
            fdac_channel_failure: [[false; 2]; 2],
            ocsm_channel_failure: [[false; 2]; 4],
            vertical_speed_fpm: 0.0,
            landing_elevation_ft: 0.0,
            athr_status: 0.0,
            ap1_active: false,
            ap2_active: false,
            fmgc_flight_phase: 0.0,
            athr_eng_fault: [false; 4],
            pack_flow_insufficient_fwd_crg: false,
            ir: [IrOutputs::default(); 3],
            att_hdg_switching_knob: 1.0,
            published: PublishedFrame::default(),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct IrOutputs {
    pub pitch_deg: Option<f64>,
    pub roll_deg: Option<f64>,
    pub true_heading_deg: Option<f64>,
    pub flight_path_angle_deg: Option<f64>,
}

pub fn capt_fo_ir(att_hdg_switching_knob: f64) -> (usize, usize) {
    let knob = att_hdg_switching_knob.round();
    (if knob == 0.0 { 2 } else { 0 }, if knob == 2.0 { 2 } else { 1 })
}



pub trait Area {
    fn name(&self) -> &'static str;

    fn tick(&mut self, truth: &Truth, faults: &Faults);

    fn publish(&self, out: &mut dyn FnMut(&str, f64));

    fn derived_failures(&self, _out: &mut dyn FnMut(DerivedFailure)) {}

    fn flight_control_surface_angles(&self) -> Option<SurfaceAngles> {
        None
    }

    fn as_breakers(&self) -> Option<&crate::deep::breakers::live::BreakersLive> {
        None
    }

    fn as_breakers_mut(&mut self) -> Option<&mut crate::deep::breakers::live::BreakersLive> {
        None
    }
}

#[derive(Default)]
pub struct Deep {
    areas: Vec<Box<dyn Area>>,
    truth: Truth,
    last_published: PublishedFrame,
    last_counts: Vec<usize>,
    derived: Vec<DerivedFailure>,
}

impl Deep {
    pub fn new() -> Self {
        Self { areas: Vec::new(), truth: Truth::default(), last_published: PublishedFrame::default(), derived: Vec::new(), last_counts: Vec::new() }
    }

    pub fn with_area(mut self, area: Box<dyn Area>) -> Self {
        self.areas.push(area);
        self
    }

    pub fn truth(&self) -> &Truth {
        &self.truth
    }

    pub fn flight_control_surface_angles(&self) -> Option<SurfaceAngles> {
        self.areas.iter().find_map(|a| a.flight_control_surface_angles())
    }

    pub fn tick(&mut self, truth: Truth, faults: &Faults, out: &mut dyn FnMut(&str, f64)) {
        self.truth = truth;
        self.truth.published = std::mem::take(&mut self.last_published);
        for area in &mut self.areas {
            area.tick(&self.truth, faults);
        }
        let mut derived = std::mem::take(&mut self.derived);
        derived.clear();
        for area in &self.areas {
            area.derived_failures(&mut |d| {
                if d.magnitude > 0.0 {
                    derived.push(d);
                }
            });
        }
        self.derived = derived;
        let mut published = std::mem::take(&mut self.truth.published);
        published.begin();
        let mut counts = std::mem::take(&mut self.last_counts);
        counts.clear();
        for area in &self.areas {
            let mut n = 0usize;
            area.publish(&mut |name, value| {
                n += 1;
                published.set(name, value);
                out(name, value);
            });
            counts.push(n);
        }
        self.last_counts = counts;
        published.finish();
        self.last_published = published;
    }

    pub fn tick_timed(
        &mut self,
        truth: Truth,
        faults: &Faults,
        out: &mut dyn FnMut(&str, f64),
    ) -> Vec<(&'static str, std::time::Duration, std::time::Duration)> {
        use std::time::Instant;
        self.truth = truth;
        self.truth.published = std::mem::take(&mut self.last_published);
        let mut timings: Vec<(&'static str, std::time::Duration, std::time::Duration)> =
            Vec::with_capacity(self.areas.len());
        for area in &mut self.areas {
            let started = Instant::now();
            area.tick(&self.truth, faults);
            timings.push((area.name(), started.elapsed(), std::time::Duration::ZERO));
        }
        let mut derived = std::mem::take(&mut self.derived);
        derived.clear();
        for area in &self.areas {
            area.derived_failures(&mut |d| {
                if d.magnitude > 0.0 {
                    derived.push(d);
                }
            });
        }
        self.derived = derived;
        let mut published = std::mem::take(&mut self.truth.published);
        published.begin();
        for (i, area) in self.areas.iter().enumerate() {
            let started = Instant::now();
            area.publish(&mut |name, value| {
                published.set(name, value);
                out(name, value);
            });
            timings[i].2 = started.elapsed();
        }
        published.finish();
        self.last_published = published;
        timings
    }

    pub fn area_names(&self) -> Vec<&'static str> {
        self.areas.iter().map(|a| a.name()).collect()
    }

    pub fn published_counts_by_area(&self) -> Vec<(&'static str, usize)> {
        self.areas.iter().map(|a| a.name()).zip(self.last_counts.iter().copied()).collect()
    }

    pub fn last_published(&self) -> &PublishedFrame {
        &self.last_published
    }

    pub fn breakers(&self) -> Option<&crate::deep::breakers::live::BreakersLive> {
        self.areas.iter().find_map(|a| a.as_breakers())
    }

    pub fn breakers_mut(&mut self) -> Option<&mut crate::deep::breakers::live::BreakersLive> {
        self.areas.iter_mut().find_map(|a| a.as_breakers_mut())
    }

    pub fn derived_failures(&self) -> &[DerivedFailure] {
        &self.derived
    }

    pub fn derived_magnitudes(&self) -> BTreeMap<u64, f64> {
        let mut out: BTreeMap<u64, f64> = BTreeMap::new();
        for d in &self.derived {
            let slot = out.entry(d.fbw_id).or_insert(0.0);
            *slot = slot.max(d.magnitude.clamp(0.0, 1.0));
        }
        out
    }

    pub fn published_names(&self) -> Vec<String> {
        let mut names = Vec::new();
        for area in &self.areas {
            area.publish(&mut |name, _| names.push(name.to_owned()));
        }
        names
    }
}

pub fn all_areas() -> Deep {
    Deep::new()
        .with_area(crate::deep::apu::live::live_system())
        .with_area(crate::deep::autoflight::live::live_system())
        .with_area(crate::deep::avionics_network::live::live_system())
        .with_area(crate::deep::breakers::live::live_system())
        .with_area(crate::deep::cabin::live::live_system())
        .with_area(crate::deep::communications::live::live_system())
        .with_area(crate::deep::electrical::live::live_system())
        .with_area(crate::deep::engine_accessories::live::live_system())
        .with_area(crate::deep::environment::live::live_system())
        .with_area(crate::deep::fire_ice::live::live_system())
        .with_area(crate::deep::flight_controls::live::live_system())
        .with_area(crate::deep::fuel::live::live_system())
        .with_area(crate::deep::gear_structure::live::live_system())
        .with_area(crate::deep::hydraulics::live::live_system())
        .with_area(crate::deep::oxygen::live::live_system())
        .with_area(crate::deep::pneumatic_ducts::live::live_system())
        .with_area(crate::deep::sensors::live::live_system())
        .with_area(crate::deep::thermal_zones::live::live_system())
        .with_area(crate::deep::wiring::live::live_system())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Default)]
    struct Counter {
        ticks: usize,
        last_dt: f64,
        leak: f64,
    }

    impl Area for Counter {
        fn name(&self) -> &'static str {
            "counter"
        }
        fn tick(&mut self, truth: &Truth, faults: &Faults) {
            self.ticks += 1;
            self.last_dt = truth.dt_s;
            self.leak = faults.get(11_021_001);
        }
        fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
            out("TEST_COUNTER_TICKS", self.ticks as f64);
            out("TEST_COUNTER_LEAK", self.leak);
        }
    }

    #[test]
    fn a_default_truth_is_a_cold_aircraft_in_real_air_not_a_vacuum() {
        let t = Truth::default();
        assert!(t.environment.ambient_pressure_pa > 90_000.0);
        assert!(t.dt_s > 0.0);
        assert!(t.on_ground && !t.apu_running);
        assert_eq!(t.engine_running, [false; 4]);
    }

    #[test]
    fn an_area_is_ticked_with_the_truth_and_faults_and_then_publishes() {
        let mut deep = Deep::new().with_area(Box::new(Counter::default()));
        let faults = Faults::from_pairs([(11_021_001, 0.4)]);
        let mut published = BTreeMap::new();
        let truth = Truth { dt_s: 0.05, ..Truth::default() };
        deep.tick(truth, &faults, &mut |name, value| {
            published.insert(name.to_string(), value);
        });
        assert_eq!(published.get("TEST_COUNTER_TICKS"), Some(&1.0));
        assert_eq!(published.get("TEST_COUNTER_LEAK"), Some(&0.4));
        assert_eq!(deep.truth().dt_s, 0.05);
        assert_eq!(deep.area_names(), vec!["counter"]);
    }

    #[test]
    fn every_assembled_area_has_its_own_name_and_can_be_stepped_cold() {
        let mut deep = all_areas();
        let names = deep.area_names();
        let unique: std::collections::BTreeSet<_> = names.iter().collect();
        assert_eq!(unique.len(), names.len(), "an area is listed twice in all_areas(): {names:?}");
        let mut published = BTreeMap::new();
        deep.tick(Truth::default(), &Faults::default(), &mut |name, value| {
            assert!(value.is_finite(), "{name} published {value} on the first frame");
            published.insert(name.to_owned(), value);
        });
    }

    #[test]
    fn published_names_are_what_the_areas_actually_publish() {
        let deep = all_areas();
        let reported: std::collections::BTreeSet<String> = deep.published_names().into_iter().collect();
        let mut actual = std::collections::BTreeSet::new();
        for area in &deep.areas {
            area.publish(&mut |name, _| {
                actual.insert(name.to_owned());
            });
        }
        assert_eq!(reported, actual);
    }

    #[derive(Default)]
    struct Downstream {
        saw: Option<f64>,
    }

    impl Area for Downstream {
        fn name(&self) -> &'static str {
            "downstream"
        }
        fn tick(&mut self, truth: &Truth, _faults: &Faults) {
            self.saw = truth.published.get("TEST_COUNTER_TICKS");
        }
        fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
            out("TEST_DOWNSTREAM_SAW", self.saw.unwrap_or(-1.0));
        }
    }

    #[test]
    fn an_area_reads_what_another_published_on_the_previous_frame() {
        let mut deep = Deep::new().with_area(Box::new(Downstream::default())).with_area(Box::new(Counter::default()));
        let faults = Faults::default();
        let mut published = BTreeMap::new();
        let mut run = |deep: &mut Deep, published: &mut BTreeMap<String, f64>| {
            deep.tick(Truth::default(), &faults, &mut |name, value| {
                published.insert(name.to_string(), value);
            });
        };

        run(&mut deep, &mut published);
        assert_eq!(published.get("TEST_DOWNSTREAM_SAW"), Some(&-1.0), "nothing has been published yet, so the read must come back absent rather than zero");

        run(&mut deep, &mut published);
        assert_eq!(published.get("TEST_DOWNSTREAM_SAW"), Some(&1.0), "the second frame sees the first frame's value");

        run(&mut deep, &mut published);
        assert_eq!(published.get("TEST_DOWNSTREAM_SAW"), Some(&2.0), "and it keeps up, exactly one frame behind");
    }

    #[test]
    fn a_name_nobody_publishes_is_absent_not_zero() {
        let frame = PublishedFrame::default();
        assert_eq!(frame.get("NOBODY_PUBLISHES_THIS"), None);
        assert_eq!(frame.get_or("NOBODY_PUBLISHES_THIS", 288.15), 288.15);
        assert!(frame.is_empty());
    }

    #[derive(Default)]
    struct Coupled {
        verdict: f64,
    }

    impl Area for Coupled {
        fn name(&self) -> &'static str {
            "coupled"
        }
        fn tick(&mut self, _truth: &Truth, faults: &Faults) {
            self.verdict = faults.get(11_021_001);
        }
        fn publish(&self, _out: &mut dyn FnMut(&str, f64)) {}
        fn derived_failures(&self, out: &mut dyn FnMut(DerivedFailure)) {
            out(DerivedFailure { fbw_id: 24_020, magnitude: if self.verdict > 0.0 { 1.0 } else { 0.0 }, deep_component: "test.gen-1", reason: "test" });
            out(DerivedFailure { fbw_id: 24_021, magnitude: 0.0, deep_component: "test.gen-2", reason: "test" });
        }
    }

    #[test]
    fn a_healthy_aircraft_derives_no_flybywire_failures_at_all() {
        let mut deep = Deep::new().with_area(Box::new(Coupled::default())).with_area(Box::new(Counter::default()));
        deep.tick(Truth::default(), &Faults::default(), &mut |_, _| {});
        assert!(deep.derived_failures().is_empty(), "a zero-magnitude coupling must not reach FlyByWire: {:?}", deep.derived_failures());
        assert!(deep.derived_magnitudes().is_empty());
    }

    #[test]
    fn an_areas_verdict_on_a_flybywire_component_reaches_the_plugin_and_clears_again() {
        let mut deep = Deep::new().with_area(Box::new(Coupled::default()));
        deep.tick(Truth::default(), &Faults::from_pairs([(11_021_001, 1.0)]), &mut |_, _| {});
        assert_eq!(deep.derived_failures().len(), 1);
        let d = deep.derived_failures()[0];
        assert_eq!(d.fbw_id, 24_020);
        assert_eq!(d.magnitude, 1.0);
        assert_eq!(d.deep_component, "test.gen-1");
        assert_eq!(deep.derived_magnitudes().get(&24_020), Some(&1.0));

        deep.tick(Truth::default(), &Faults::default(), &mut |_, _| {});
        assert!(deep.derived_failures().is_empty());
    }

    #[test]
    fn an_area_with_nothing_flybywire_models_derives_nothing() {
        let mut deep = Deep::new().with_area(Box::new(Counter::default()));
        deep.tick(Truth::default(), &Faults::from_pairs([(11_021_001, 1.0)]), &mut |_, _| {});
        assert!(deep.derived_failures().is_empty());
    }

    #[test]
    fn an_unarmed_failure_reads_healthy_and_magnitudes_are_bounded() {
        let f = Faults::from_pairs([(1, 2.5), (2, -1.0)]);
        assert_eq!(f.get(999), 0.0, "a failure nobody armed must read healthy, not absent");
        assert_eq!(f.get(1), 1.0, "magnitudes are fractions and cannot exceed fully failed");
        assert_eq!(f.get(2), 0.0);
        assert!(f.any());
        assert!(!Faults::default().any());
    }
}


