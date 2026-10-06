use std::collections::HashMap;
use std::f64::consts::PI;

use deep_systems::{CommandedSurfaces, EnvironmentTruth, IrOutputs, Truth, DOOR_NAMES};
use deep_systems::deep::weather::{WeatherCloudLayer, WeatherSample};
use systems::{
    shared::{ElectricalBusType, ElectricalBuses},
    simulation::{InitContext, Reader, SimulatorReader, UpdateContext, VariableIdentifier},
};
use uom::si::{
    angle::degree, mass::kilogram, pressure::pascal,
    thermodynamic_temperature::degree_celsius, velocity::foot_per_minute,
    velocity::meter_per_second,
};

const PSI_TO_PA: f64 = 6894.757;

const DOOR_POINTS: [usize; DOOR_NAMES.len()] = [0, 2, 3, 6, 8, 10, 11, 12, 13, 14, 15, 16, 17];

fn solar_elevation_deg(lat_deg: f64, lon_deg: f64, day_of_year: f64, utc_seconds: f64) -> f64 {
    let hour_utc = utc_seconds / 3600.0;
    let gamma = 2.0 * PI / 365.0 * (day_of_year - 1.0 + (hour_utc - 12.0) / 24.0);

    let decl = 0.006918 - 0.399912 * gamma.cos() + 0.070257 * gamma.sin()
        - 0.006758 * (2.0 * gamma).cos()
        + 0.000907 * (2.0 * gamma).sin()
        - 0.002697 * (3.0 * gamma).cos()
        + 0.00148 * (3.0 * gamma).sin();

    let eqtime = 229.18
        * (0.000075 + 0.001868 * gamma.cos()
            - 0.032077 * gamma.sin()
            - 0.014615 * (2.0 * gamma).cos()
            - 0.040849 * (2.0 * gamma).sin());

    let true_solar_time_min = hour_utc * 60.0 + eqtime + 4.0 * lon_deg;

    let hour_angle_deg = ((true_solar_time_min / 4.0 - 180.0 + 180.0).rem_euclid(360.0)) - 180.0;
    let hour_angle = hour_angle_deg.to_radians();

    let lat = lat_deg.to_radians();
    let cos_zenith = lat.sin() * decl.sin() + lat.cos() * decl.cos() * hour_angle.cos();
    cos_zenith.clamp(-1.0, 1.0).asin().to_degrees()
}

#[cfg(test)]
mod solar_elevation_tests {
    use super::solar_elevation_deg;

    #[test]
    fn local_noon_at_equator_equinox_is_near_ninety() {
        let elev = solar_elevation_deg(0.0, 0.0, 80.0, 12.0 * 3600.0);
        assert!(elev > 85.0 && elev <= 90.0, "elev = {elev}");
    }

    #[test]
    fn local_midnight_at_equator_equinox_is_negative() {
        let elev = solar_elevation_deg(0.0, 0.0, 80.0, 0.0);
        assert!(elev < -80.0, "elev = {elev}");
    }
}

const MIN_DT_S: f64 = 0.001;
const MAX_DT_S: f64 = 0.2;

const ENGINE_STATE_ON: f64 = 1.0;
const ENGINE_STATE_STARTING: f64 = 2.0;
const ENGINE_STATE_RESTARTING: f64 = 3.0;
const STARTER_ENGAGE_TIMER_S: f64 = 1.7;

const REVERSER_OPENING_AUTHORISATION_TLA_DEG: f64 = -4.3;

const SSM_NORMAL_OPERATION: u32 = 3;

fn unpack_arinc(packed: f64) -> (f64, u32) {
    let bits = packed as u64;
    (f32::from_bits(bits as u32) as f64, ((bits >> 32) & 0b11) as u32)
}

fn leg_grounded(on_ground_now: bool, compressed: bool, was_grounded: bool) -> bool {
    on_ground_now && (compressed || was_grounded)
}

struct EngineIds {
    n1_pct: VariableIdentifier,
    n2_pct: VariableIdentifier,
    n3_pct: VariableIdentifier,
    n2_healthy_pct: VariableIdentifier,
    n3_healthy_pct: VariableIdentifier,
    n1_commanded_pct: VariableIdentifier,
    customer_bleed_kg_s: VariableIdentifier,
    sim_corrected_n1_pct: VariableIdentifier,
    sim_corrected_n2_pct: VariableIdentifier,
    state: VariableIdentifier,
    timer: VariableIdentifier,
    oil_temp_c: VariableIdentifier,
    tla_deg: VariableIdentifier,
    fire_pb_released: VariableIdentifier,
    fire_agent_pb_pressed: [VariableIdentifier; 2],
    nacelle_anti_ice_position: VariableIdentifier,
    bleed_pb_auto: VariableIdentifier,
    eng_gen_pb_on: VariableIdentifier,
    master: VariableIdentifier,
    igniter: VariableIdentifier,
    oil_pressure_psi: VariableIdentifier,
    oil_quantity: VariableIdentifier,
    oil_total: VariableIdentifier,
    tgt_measured_c: VariableIdentifier,
    fuel_flow_kg_h: VariableIdentifier,
    hp_pressure_psi: VariableIdentifier,
    hp_temperature_c: VariableIdentifier,
    ip_temperature_c: VariableIdentifier,
    hp_valve_open: VariableIdentifier,
    intermediate_transducer_pressure_psi: VariableIdentifier,
}

struct SurfaceIds {
    ailerons: [[VariableIdentifier; 3]; 2],
    elevators: [[VariableIdentifier; 2]; 2],
    rudders: [VariableIdentifier; 2],
    spoilers: [[VariableIdentifier; 8]; 2],
    ths: VariableIdentifier,
}

pub(super) struct TruthReader {
    engines: [EngineIds; 4],
    surfaces: SurfaceIds,
    apu_available: VariableIdentifier,
    apu_bleed_air_pressure: VariableIdentifier,
    hydraulic_pressure_psi: [VariableIdentifier; 2],
    flap_lever_handle_index: VariableIdentifier,
    fire_pb_apu_released: VariableIdentifier,
    fire_agent_pb_apu_pressed: VariableIdentifier,
    cargo_agent_pb_pressed: [VariableIdentifier; 2],
    wing_anti_ice_position: VariableIdentifier,
    apu_bleed_pb_on: VariableIdentifier,
    cross_bleed_selector: VariableIdentifier,
    pack_pb_on: [VariableIdentifier; 2],
    gear_door_position: [VariableIdentifier; 3],
    gear_handle_position: VariableIdentifier,
    nw_steer_disc_memo: VariableIdentifier,
    rain_removal_selected: [VariableIdentifier; 2],
    park_brake_lever_pos: VariableIdentifier,
    remote_cb_ctl_active: VariableIdentifier,
    apu_gen_pb_on: [VariableIdentifier; 2],
    bat_pb_auto: [VariableIdentifier; 2],
    apu_master_sw_on: VariableIdentifier,
    apu_start_pb_on: VariableIdentifier,
    ext_pwr_avail: [VariableIdentifier; 4],
    lgciu_gear_compressed: [VariableIdentifier; 3],
    cabin_delta_pressure: VariableIdentifier,
    cabin_temp_c: VariableIdentifier,
    cargo_door_position: [VariableIdentifier; 2],
    baro_mode: [VariableIdentifier; 2],
    to_flex_temp: VariableIdentifier,
    preset_quick_mode: VariableIdentifier,
    fdac_channel_failure: [[VariableIdentifier; 2]; 2],
    ocsm_channel_failure: [[VariableIdentifier; 2]; 4],
    landing_elevation_ft: VariableIdentifier,
    athr_status: VariableIdentifier,
    ap_active: [VariableIdentifier; 2],
    fmgc_flight_phase: VariableIdentifier,
    flight_ready: VariableIdentifier,
    ir: [[VariableIdentifier; 4]; 3],
    att_hdg_switching_knob: VariableIdentifier,
    prim_healthy: [VariableIdentifier; 3],
    sec_healthy: [VariableIdentifier; 3],
    total_air_temperature_c: VariableIdentifier,
    heading_true_deg: VariableIdentifier,
    alpha_deg: VariableIdentifier,
    altitude_msl_ft: VariableIdentifier,
    radio_altitude_1: VariableIdentifier,
    door_open_percent: [VariableIdentifier; DOOR_NAMES.len()],
    fuel_tank_quantity: [VariableIdentifier; 11],
    crossfeed_valve_open: [VariableIdentifier; 4],
    jettison_valve_open: [VariableIdentifier; 2],
    sidestick_x: VariableIdentifier,
    sidestick_y: VariableIdentifier,
    rudder_pedal_position: VariableIdentifier,
    prim_left_sidestick_disabled: VariableIdentifier,
    prim_right_sidestick_disabled: VariableIdentifier,
    prim_left_sidestick_priority_locked: VariableIdentifier,
    prim_right_sidestick_priority_locked: VariableIdentifier,
    sec1_rudder_trim_actual_deg: VariableIdentifier,
    left_flaps_angle_deg: VariableIdentifier,
    right_flaps_angle_deg: VariableIdentifier,
    left_slats_angle_deg: VariableIdentifier,
    right_slats_angle_deg: VariableIdentifier,
    left_brake_pedal_input: VariableIdentifier,
    right_brake_pedal_input: VariableIdentifier,
    spoilers_armed: VariableIdentifier,
    zulu_time_s: VariableIdentifier,
    zulu_day_of_year: VariableIdentifier,
    latitude_deg: VariableIdentifier,
    longitude_deg: VariableIdentifier,

    all: Vec<VariableIdentifier>,
    values: HashMap<VariableIdentifier, f64>,

    ac_bus_powered: [bool; 4],
    ac_bus_volts: [f64; 4],
    dc_bus_powered: [bool; 2],
    dc_bus_volts: [f64; 2],

    prev_leg_on_ground: [bool; 5],
    held_sink_speed_ms: [f64; 5],
}

fn reg(context: &mut InitContext, all: &mut Vec<VariableIdentifier>, name: String) -> VariableIdentifier {
    let id = context.get_identifier(name);
    all.push(id);
    id
}

impl TruthReader {
    pub(super) fn zulu_time_s(&self) -> f64 {
        self.v(&self.zulu_time_s)
    }

    pub(super) fn new(context: &mut InitContext) -> Self {
        let mut all = Vec::new();
        let a = &mut all;
        let engines = [1, 2, 3, 4].map(|n| EngineIds {
            n1_pct: reg(context, a, format!("ENGINE_N1:{n}")),
            n2_pct: reg(context, a, format!("ENGINE_N2:{n}")),
            n3_pct: reg(context, a, format!("ENGINE_N3:{n}")),
            n2_healthy_pct: reg(context, a, format!("ENGINE_N2_HEALTHY:{n}")),
            n3_healthy_pct: reg(context, a, format!("ENGINE_N3_HEALTHY:{n}")),
            n1_commanded_pct: reg(context, a, format!("AUTOTHRUST_N1_COMMANDED:{n}")),
            customer_bleed_kg_s: reg(context, a, format!("PNEU_ENG_{n}_BLEED_EXTRACTION_FLOW")),
            sim_corrected_n1_pct: reg(context, a, format!("TURB ENG CORRECTED N1:{n}")),
            sim_corrected_n2_pct: reg(context, a, format!("TURB ENG CORRECTED N2:{n}")),
            state: reg(context, a, format!("ENGINE_STATE:{n}")),
            timer: reg(context, a, format!("ENGINE_TIMER:{n}")),
            oil_temp_c: reg(context, a, format!("GENERAL ENG OIL TEMPERATURE:{n}")),
            tla_deg: reg(context, a, format!("AUTOTHRUST_TLA:{n}")),
            fire_pb_released: reg(context, a, format!("FIRE_BUTTON_ENG{n}")),
            fire_agent_pb_pressed: [1, 2].map(|b| reg(context, a, format!("OVHD_FIRE_AGENT_{b}_ENG_{n}_IS_PRESSED"))),
            nacelle_anti_ice_position: reg(context, a, format!("BUTTON_OVHD_ANTI_ICE_ENG_{n}_POSITION")),
            bleed_pb_auto: reg(context, a, format!("OVHD_PNEU_ENG_{n}_BLEED_PB_IS_AUTO")),
            eng_gen_pb_on: reg(context, a, format!("OVHD_ELEC_ENG_GEN_{n}_PB_IS_ON")),
            master: reg(context, a, format!("GENERAL ENG STARTER:{n}")),
            igniter: reg(context, a, format!("TURB ENG IGNITION SWITCH EX1:{n}")),
            oil_pressure_psi: reg(context, a, format!("GENERAL ENG OIL PRESSURE:{n}")),
            oil_quantity: reg(context, a, format!("ENGINE_OIL_QTY:{n}")),
            oil_total: reg(context, a, format!("ENGINE_OIL_TOTAL:{n}")),
            tgt_measured_c: reg(context, a, format!("ENGINE_EGT:{n}")),
            fuel_flow_kg_h: reg(context, a, format!("ENGINE_FF:{n}")),
            hp_pressure_psi: reg(context, a, format!("PNEU_ENG_{n}_HP_PRESSURE")),
            hp_temperature_c: reg(context, a, format!("PNEU_ENG_{n}_HP_TEMPERATURE")),
            ip_temperature_c: reg(context, a, format!("PNEU_ENG_{n}_IP_TEMPERATURE")),
            hp_valve_open: reg(context, a, format!("PNEU_ENG_{n}_HP_VALVE_OPEN")),
            intermediate_transducer_pressure_psi: reg(context, a, format!("PNEU_ENG_{n}_INTERMEDIATE_TRANSDUCER_PRESSURE")),
        });
        const SIDES: [&str; 2] = ["LEFT", "RIGHT"];
        let surfaces = SurfaceIds {
            ailerons: SIDES.map(|side| ["INWARD", "MIDDLE", "OUTWARD"].map(|part| reg(context, a, format!("HYD_AIL_{side}_{part}_DEFLECTION")))),
            elevators: SIDES.map(|side| ["INWARD", "OUTWARD"].map(|part| reg(context, a, format!("HYD_ELEV_{side}_{part}_DEFLECTION")))),
            rudders: ["UPPER", "LOWER"].map(|which| reg(context, a, format!("HYD_{which}_RUD_DEFLECTION"))),
            spoilers: SIDES.map(|side| std::array::from_fn(|i| reg(context, a, format!("HYD_SPOILER_{}_{side}_DEFLECTION", i + 1)))),
            ths: reg(context, a, "HYD_FINAL_THS_DEFLECTION".to_owned()),
        };
        Self {
            engines,
            surfaces,
            apu_available: reg(context, a, "OVHD_APU_START_PB_IS_AVAILABLE".to_owned()),
            apu_bleed_air_pressure: reg(context, a, "APU_BLEED_AIR_PRESSURE".to_owned()),
            hydraulic_pressure_psi: ["GREEN", "YELLOW"].map(|s| reg(context, a, format!("HYD_{s}_SYSTEM_1_SECTION_PRESSURE"))),
            flap_lever_handle_index: reg(context, a, "FLAPS_HANDLE_INDEX".to_owned()),
            fire_pb_apu_released: reg(context, a, "FIRE_BUTTON_APU".to_owned()),
            fire_agent_pb_apu_pressed: reg(context, a, "OVHD_FIRE_AGENT_1_APU_1_IS_PRESSED".to_owned()),
            cargo_agent_pb_pressed: ["FWD", "AFT"].map(|s| reg(context, a, format!("CARGOSMOKE_{s}_DISCHARGED"))),
            wing_anti_ice_position: reg(context, a, "BUTTON_OVHD_ANTI_ICE_WING_POSITION".to_owned()),
            apu_bleed_pb_on: reg(context, a, "OVHD_APU_BLEED_PB_IS_ON".to_owned()),
            cross_bleed_selector: reg(context, a, "KNOB_OVHD_AIRCOND_XBLEED_Position".to_owned()),
            pack_pb_on: [1, 2].map(|n| reg(context, a, format!("OVHD_COND_PACK_{n}_PB_IS_ON"))),
            gear_door_position: ["CENTER", "LEFT", "RIGHT"].map(|s| reg(context, a, format!("GEAR_DOOR_{s}_POSITION"))),
            gear_handle_position: reg(context, a, "GEAR_HANDLE_POSITION".to_owned()),
            nw_steer_disc_memo: reg(context, a, "HYD_NW_STRG_DISC_ECAM_MEMO".to_owned()),
            rain_removal_selected: [1, 2].map(|n| reg(context, a, format!("RAIN_REMOVAL_SELECTED_{n}"))),
            park_brake_lever_pos: reg(context, a, "PARK_BRAKE_LEVER_POS".to_owned()),
            remote_cb_ctl_active: {
                let id = context.get_unprefixed_identifier("A380X_REMOTE_CB_CTRL".to_owned());
                a.push(id);
                id
            },
            apu_gen_pb_on: [1, 2].map(|n| reg(context, a, format!("OVHD_ELEC_APU_GEN_{n}_PB_IS_ON"))),
            bat_pb_auto: [1, 2].map(|n| reg(context, a, format!("OVHD_ELEC_BAT_{n}_PB_IS_AUTO"))),
            apu_master_sw_on: reg(context, a, "OVHD_APU_MASTER_SW_PB_IS_ON".to_owned()),
            apu_start_pb_on: reg(context, a, "OVHD_APU_START_PB_IS_ON".to_owned()),
            ext_pwr_avail: [1, 2, 3, 4].map(|n| reg(context, a, format!("EXT_PWR_AVAIL:{n}"))),
            lgciu_gear_compressed: ["NOSE", "LEFT", "RIGHT"].map(|s| reg(context, a, format!("LGCIU_1_{s}_GEAR_COMPRESSED"))),
            cabin_delta_pressure: reg(context, a, "PRESS_CABIN_DELTA_PRESSURE_B1".to_owned()),
            cabin_temp_c: reg(context, a, "COND_MAIN_DECK_1_TEMP".to_owned()),
            cargo_door_position: ["FWD", "AFT"].map(|s| reg(context, a, format!("{s}_DOOR_CARGO_POSITION"))),
            baro_mode: ["L", "R"].map(|s| reg(context, a, format!("FCU_EFIS_{s}_DISPLAY_BARO_MODE"))),
            to_flex_temp: reg(context, a, "AIRLINER_TO_FLEX_TEMP".to_owned()),
            preset_quick_mode: reg(context, a, "AIRCRAFT_PRESET_QUICK_MODE".to_owned()),
            fdac_channel_failure: [1, 2].map(|f| [1, 2].map(|c| reg(context, a, format!("COND_FDAC_{f}_CHANNEL_{c}_FAILURE")))),
            ocsm_channel_failure: [1, 2, 3, 4].map(|o| [1, 2].map(|c| reg(context, a, format!("PRESS_OCSM_{o}_CHANNEL_{c}_FAILURE")))),
            landing_elevation_ft: reg(context, a, "FM1_LANDING_ELEVATION".to_owned()),
            athr_status: reg(context, a, "AUTOTHRUST_STATUS".to_owned()),
            ap_active: [1, 2].map(|n| reg(context, a, format!("AUTOPILOT_{n}_ACTIVE"))),
            fmgc_flight_phase: reg(context, a, "FMGC_FLIGHT_PHASE".to_owned()),
            flight_ready: reg(context, a, "IS_READY".to_owned()),
            ir: [1, 2, 3].map(|n| {
                ["PITCH", "ROLL", "TRUE_HEADING", "FLIGHT_PATH_ANGLE"].map(|w| reg(context, a, format!("ADIRS_IR_{n}_{w}")))
            }),
            att_hdg_switching_knob: reg(context, a, "ATT_HDG_SWITCHING_KNOB".to_owned()),
            prim_healthy: [1, 2, 3].map(|n| reg(context, a, format!("PRIM_{n}_HEALTHY"))),
            sec_healthy: [1, 2, 3].map(|n| reg(context, a, format!("SEC_{n}_HEALTHY"))),
            total_air_temperature_c: reg(context, a, "TOTAL AIR TEMPERATURE".to_owned()),
            heading_true_deg: reg(context, a, "PLANE HEADING DEGREES TRUE".to_owned()),
            alpha_deg: reg(context, a, "INCIDENCE ALPHA".to_owned()),
            altitude_msl_ft: reg(context, a, "PLANE ALTITUDE".to_owned()),
            radio_altitude_1: reg(context, a, "RA_1_RADIO_ALTITUDE".to_owned()),
            door_open_percent: DOOR_POINTS.map(|p| reg(context, a, format!("INTERACTIVE POINT OPEN:{p}"))),
            fuel_tank_quantity: std::array::from_fn(|i| reg(context, a, format!("FUELSYSTEM TANK QUANTITY:{}", i + 1))),
            crossfeed_valve_open: [46, 47, 48, 49].map(|n| reg(context, a, format!("FUELSYSTEM VALVE OPEN:{n}"))),
            jettison_valve_open: [57, 58].map(|n| reg(context, a, format!("FUELSYSTEM VALVE OPEN:{n}"))),
            sidestick_x: reg(context, a, "SIDESTICK_POSITION_X".to_owned()),
            sidestick_y: reg(context, a, "SIDESTICK_POSITION_Y".to_owned()),
            rudder_pedal_position: reg(context, a, "RUDDER_PEDAL_POSITION".to_owned()),
            prim_left_sidestick_disabled: reg(context, a, "PRIM_1_LEFT_SIDESTICK_DISABLED".to_owned()),
            prim_right_sidestick_disabled: reg(context, a, "PRIM_1_RIGHT_SIDESTICK_DISABLED".to_owned()),
            prim_left_sidestick_priority_locked: reg(context, a, "PRIM_1_LEFT_SIDESTICK_PRIORITY_LOCKED".to_owned()),
            prim_right_sidestick_priority_locked: reg(context, a, "PRIM_1_RIGHT_SIDESTICK_PRIORITY_LOCKED".to_owned()),
            sec1_rudder_trim_actual_deg: reg(context, a, "SEC_1_RUDDER_ACTUAL_POSITION".to_owned()),
            left_flaps_angle_deg: reg(context, a, "LEFT_FLAPS_ANGLE".to_owned()),
            right_flaps_angle_deg: reg(context, a, "RIGHT_FLAPS_ANGLE".to_owned()),
            left_slats_angle_deg: reg(context, a, "LEFT_SLATS_ANGLE".to_owned()),
            right_slats_angle_deg: reg(context, a, "RIGHT_SLATS_ANGLE".to_owned()),
            left_brake_pedal_input: reg(context, a, "LEFT_BRAKE_PEDAL_INPUT".to_owned()),
            right_brake_pedal_input: reg(context, a, "RIGHT_BRAKE_PEDAL_INPUT".to_owned()),
            spoilers_armed: reg(context, a, "SPOILERS_ARMED".to_owned()),
            zulu_time_s: reg(context, a, "ZULU TIME".to_owned()),
            zulu_day_of_year: reg(context, a, "ZULU DAY OF YEAR".to_owned()),
            latitude_deg: reg(context, a, "PLANE LATITUDE".to_owned()),
            longitude_deg: reg(context, a, "PLANE LONGITUDE".to_owned()),
            all,
            values: HashMap::new(),
            ac_bus_powered: [false; 4],
            ac_bus_volts: [0.; 4],
            dc_bus_powered: [false; 2],
            dc_bus_volts: [0.; 2],
            prev_leg_on_ground: [false; 5],
            held_sink_speed_ms: [0.; 5],
        }
    }

    pub(super) fn read(&mut self, reader: &mut SimulatorReader) {
        for id in &self.all {
            self.values.insert(*id, reader.read_f64(id));
        }
    }

    pub(super) fn receive_power(&mut self, buses: &impl ElectricalBuses) {
        for n in 0..4 {
            let bus = ElectricalBusType::AlternatingCurrent(n as u8 + 1);
            self.ac_bus_powered[n] = buses.is_powered(bus);
            self.ac_bus_volts[n] = buses.potential_of(bus).raw().get::<uom::si::electric_potential::volt>();
        }
        for n in 0..2 {
            let bus = ElectricalBusType::DirectCurrent(n as u8 + 1);
            self.dc_bus_powered[n] = buses.is_powered(bus);
            self.dc_bus_volts[n] = buses.potential_of(bus).raw().get::<uom::si::electric_potential::volt>();
        }
    }

    fn v(&self, id: &VariableIdentifier) -> f64 {
        self.values.get(id).copied().unwrap_or(0.)
    }

    pub(super) fn truth(&mut self, context: &UpdateContext) -> Truth {
        let default = Truth::default();
        let dt_s = context.delta_as_secs_f64().clamp(MIN_DT_S, MAX_DT_S);

        let ambient_pressure_pa = context.ambient_pressure().get::<pascal>();
        let weather = context.is_in_cloud().then(|| {
            let mut sample = WeatherSample::default();
            sample.clouds[0] = WeatherCloudLayer { cloud_type: 1.0, coverage: 1.0, alt_base_m: 0.0, alt_top_m: 0.0 };
            sample
        });
        let environment = EnvironmentTruth {
            sat_c: context.ambient_temperature().get::<degree_celsius>(),
            tas_ms: context.true_airspeed().get::<meter_per_second>(),
            leading_edge_c: self.v(&self.total_air_temperature_c),
            ambient_pressure_pa: if ambient_pressure_pa > 0. { ambient_pressure_pa } else { default.environment.ambient_pressure_pa },
            weather,
            ..default.environment
        };

        let mut engine_n1_frac = [0.0; 4];
        let mut engine_n2_frac = [0.0; 4];
        let mut engine_n3_frac = [0.0; 4];
        let mut engine_n2_healthy_frac = [0.0; 4];
        let mut engine_n3_healthy_frac = [0.0; 4];
        let mut engine_n1_commanded_pct = [0.0; 4];
        let mut engine_customer_bleed_kg_s = [0.0; 4];
        let mut sim_engine_corrected_n1_pct = [0.0; 4];
        let mut sim_engine_corrected_n2_pct = [0.0; 4];
        let mut engine_running = [false; 4];
        let mut engine_oil_temp_c = default.engine_oil_temp_c;
        let mut engine_oil_pressure_pa = default.engine_oil_pressure_pa;
        let mut engine_oil_quantity_fraction = default.engine_oil_quantity_fraction;
        let mut engine_tgt_c = default.engine_tgt_c;
        let mut engine_t25_c = default.engine_t25_c;
        let mut engine_bleed_pressure_pa = default.engine_bleed_pressure_pa;
        let mut engine_bleed_temp_k = default.engine_bleed_temp_k;
        let mut engine_hp_port_pressure_pa = default.engine_hp_port_pressure_pa;
        let mut engine_hp_port_temp_k = default.engine_hp_port_temp_k;
        let mut engine_ip_port_temp_k = default.engine_ip_port_temp_k;
        let mut engine_ip_port_pressure_pa = default.engine_ip_port_pressure_pa;
        let mut engine_fuel_flow_kg_s = default.engine_fuel_flow_kg_s;
        let mut engine_tla_deg = [0.0; 4];
        let mut controls = default.controls;
        for (i, e) in self.engines.iter().enumerate() {
            engine_n1_frac[i] = self.v(&e.n1_pct) / 100.0;
            engine_n2_frac[i] = self.v(&e.n2_pct) / 100.0;
            engine_n3_frac[i] = self.v(&e.n3_pct) / 100.0;
            engine_n2_healthy_frac[i] = self.v(&e.n2_healthy_pct) / 100.0;
            engine_n3_healthy_frac[i] = self.v(&e.n3_healthy_pct) / 100.0;
            engine_n1_commanded_pct[i] = self.v(&e.n1_commanded_pct);
            engine_customer_bleed_kg_s[i] = self.v(&e.customer_bleed_kg_s).max(0.0);
            sim_engine_corrected_n1_pct[i] = self.v(&e.sim_corrected_n1_pct);
            sim_engine_corrected_n2_pct[i] = self.v(&e.sim_corrected_n2_pct);
            let state = self.v(&e.state);
            engine_running[i] = state == ENGINE_STATE_ON;
            engine_oil_temp_c[i] = self.v(&e.oil_temp_c);
            engine_tla_deg[i] = self.v(&e.tla_deg);

            engine_oil_pressure_pa[i] = self.v(&e.oil_pressure_psi) * PSI_TO_PA;
            let oil_total = self.v(&e.oil_total);
            if oil_total > 0.0 {
                engine_oil_quantity_fraction[i] = (self.v(&e.oil_quantity) / oil_total).clamp(0.0, 1.0);
            }
            engine_tgt_c[i] = self.v(&e.tgt_measured_c);
            let ip_temp_c = self.v(&e.ip_temperature_c);
            engine_t25_c[i] = ip_temp_c;
            engine_ip_port_temp_k[i] = ip_temp_c + 273.15;
            let transducer_psi = self.v(&e.intermediate_transducer_pressure_psi);
            if transducer_psi > 0.0 {
                engine_ip_port_pressure_pa[i] = transducer_psi * PSI_TO_PA;
            }
            let hp_pressure_pa = self.v(&e.hp_pressure_psi) * PSI_TO_PA;
            let hp_temp_k = self.v(&e.hp_temperature_c) + 273.15;
            engine_hp_port_pressure_pa[i] = hp_pressure_pa;
            engine_hp_port_temp_k[i] = hp_temp_k;
            let hp_valve_open = self.v(&e.hp_valve_open) != 0.0;
            engine_bleed_temp_k[i] = if hp_valve_open { hp_temp_k } else { ip_temp_c + 273.15 };
            engine_bleed_pressure_pa[i] = if hp_valve_open { hp_pressure_pa } else { default.engine_bleed_pressure_pa[i] };
            engine_fuel_flow_kg_s[i] = self.v(&e.fuel_flow_kg_h) / 3600.0;

            controls.fire_pb_released[i] = self.v(&e.fire_pb_released) != 0.0;
            controls.fire_agent_pb_pressed[i] = e.fire_agent_pb_pressed.map(|id| self.v(&id) != 0.0);
            controls.nacelle_anti_ice_selected[i] = self.v(&e.nacelle_anti_ice_position) != 0.0;
            controls.engine_bleed_pb_auto[i] = self.v(&e.bleed_pb_auto) != 0.0;
            controls.eng_gen_pb_on[i] = self.v(&e.eng_gen_pb_on) != 0.0;
            if let Some(slot) = [1usize, 2].iter().position(|&e_index| e_index == i) {
                controls.reverser_deploy_commanded[slot] = engine_tla_deg[i] <= REVERSER_OPENING_AUTHORISATION_TLA_DEG;
            }
            let master = self.v(&e.master) != 0.0;
            controls.engine_master_on[i] = master;
            let igniter = self.v(&e.igniter).round();
            let timer = self.v(&e.timer);
            controls.starter_engaged[i] = master
                && igniter == 2.
                && (state == ENGINE_STATE_STARTING || state == ENGINE_STATE_RESTARTING)
                && timer >= STARTER_ENGAGE_TIMER_S;
        }

        let (apu_bleed_psi, apu_bleed_ssm) = unpack_arinc(self.v(&self.apu_bleed_air_pressure));
        let apu_bleed_pressure_pa = if apu_bleed_ssm == SSM_NORMAL_OPERATION && apu_bleed_psi > 0.0 {
            apu_bleed_psi * PSI_TO_PA
        } else {
            environment.ambient_pressure_pa
        };

        controls.fire_pb_apu_released = self.v(&self.fire_pb_apu_released) != 0.0;
        controls.fire_agent_pb_apu_pressed = self.v(&self.fire_agent_pb_apu_pressed) != 0.0;
        controls.cargo_agent_pb_pressed = self.cargo_agent_pb_pressed.map(|id| self.v(&id) != 0.0);
        controls.wing_anti_ice_selected = self.v(&self.wing_anti_ice_position) != 0.0;
        controls.apu_bleed_pb_on = self.v(&self.apu_bleed_pb_on) != 0.0;
        controls.cross_bleed_selector = self.v(&self.cross_bleed_selector);
        controls.pack_pb_on = self.pack_pb_on.map(|id| self.v(&id) != 0.0);
        controls.gear_door_commanded_open = self.gear_door_position.map(|id| self.v(&id));
        controls.gear_lever_down = self.v(&self.gear_handle_position) >= 0.5;
        controls.nw_steer_disc_selected = self.v(&self.nw_steer_disc_memo) >= 0.5;
        for (selected, id) in controls.rain_removal_selected.iter_mut().zip(&self.rain_removal_selected) {
            *selected = self.v(id) >= 0.5;
        }
        controls.parking_brake_on = self.v(&self.park_brake_lever_pos) >= 0.5;
        controls.remote_cb_ctl_active = self.v(&self.remote_cb_ctl_active) >= 0.5;
        controls.apu_gen_pb_on = self.apu_gen_pb_on.map(|id| self.v(&id) != 0.0);
        controls.bat_pb_auto = self.bat_pb_auto.map(|id| self.v(&id) != 0.0);
        controls.apu_master_sw_on = self.v(&self.apu_master_sw_on) != 0.0;
        controls.apu_start_pb_on = self.v(&self.apu_start_pb_on) != 0.0;
        controls.baro_mode = self.baro_mode.map(|id| self.v(&id));
        controls.cargo_door_commanded_open[0] = (self.v(&self.cargo_door_position[0]) / 100.0).clamp(0.0, 1.0);
        controls.cargo_door_commanded_open[1] = (self.v(&self.cargo_door_position[1]) / 100.0).clamp(0.0, 1.0);
        controls.ground_spoiler_lever_armed = self.v(&self.spoilers_armed) != 0.0;
        controls.brake_pedal_pos = [
            (self.v(&self.left_brake_pedal_input) / 100.0).clamp(0.0, 1.0),
            (self.v(&self.right_brake_pedal_input) / 100.0).clamp(0.0, 1.0),
        ];
        controls.jettison_valve_selected = self.jettison_valve_open.map(|id| self.v(&id) != 0.0);
        controls.jettison_armed = controls.jettison_valve_selected.iter().any(|&open| open);
        controls.crossfeed_valve_selected = self.crossfeed_valve_open.map(|id| self.v(&id) != 0.0);

        let gpu_plugged_in = self.ext_pwr_avail.iter().any(|id| self.v(id) != 0.0);
        let to_flex_temp_set = self.v(&self.to_flex_temp) != 0.0;

        let aileron_or_elevator_down_deg = |n: f64| 20. - 50. * n;
        let rudder_right_deg = |n: f64| 60. * n - 30.;
        let spoiler_up_deg = |n: f64| 50. * n;
        let s = &self.surfaces;
        let commanded_surfaces = CommandedSurfaces {
            ailerons_deg: s.ailerons.map(|side| side.map(|id| aileron_or_elevator_down_deg(self.v(&id)))),
            elevators_deg: s.elevators.map(|side| side.map(|id| aileron_or_elevator_down_deg(self.v(&id)))),
            rudders_deg: s.rudders.map(|id| -rudder_right_deg(self.v(&id))),
            spoilers_deg: s.spoilers.map(|side| side.map(|id| spoiler_up_deg(self.v(&id)))),
            ths_deg: self.v(&s.ths),
        };

        let on_ground_now = context.is_on_ground();
        let lc = &self.lgciu_gear_compressed;
        let (nose, left, right) = (self.v(&lc[0]) != 0.0, self.v(&lc[1]) != 0.0, self.v(&lc[2]) != 0.0);
        let compressed = [nose, left, right, left, right];
        let leg_on_ground = std::array::from_fn(|i| leg_grounded(on_ground_now, compressed[i], self.prev_leg_on_ground[i]));
        let descent_speed_m_s = (-context.vertical_speed().get::<meter_per_second>()).max(0.0);
        let mut leg_touchdown_sink_speed_ms = self.held_sink_speed_ms;
        for i in 0..5 {
            if leg_on_ground[i] && !self.prev_leg_on_ground[i] {
                leg_touchdown_sink_speed_ms[i] = descent_speed_m_s;
            } else if !leg_on_ground[i] {
                leg_touchdown_sink_speed_ms[i] = 0.0;
            }
        }
        self.prev_leg_on_ground = leg_on_ground;
        self.held_sink_speed_ms = leg_touchdown_sink_speed_ms;

        let (cabin_delta_psi, cabin_delta_ssm) = unpack_arinc(self.v(&self.cabin_delta_pressure));
        let cabin_pressure_pa = if cabin_delta_ssm == SSM_NORMAL_OPERATION {
            environment.ambient_pressure_pa + cabin_delta_psi * PSI_TO_PA
        } else {
            default.cabin_pressure_pa
        };
        let cabin_temp_c = self.v(&self.cabin_temp_c);
        let cabin_temp_k = if cabin_temp_c > -273.15 { cabin_temp_c + 273.15 } else { default.cabin_temp_k };

        let (landing_elevation_raw_ft, landing_elevation_ssm) = unpack_arinc(self.v(&self.landing_elevation_ft));
        let landing_elevation_ft = if landing_elevation_ssm == SSM_NORMAL_OPERATION { landing_elevation_raw_ft } else { 0.0 };

        let total_weight_kg = context.total_weight().get::<kilogram>();

        let (radio_alt_raw_ft, radio_alt_ssm) = unpack_arinc(self.v(&self.radio_altitude_1));
        let radio_height_ft = if radio_alt_ssm == SSM_NORMAL_OPERATION { radio_alt_raw_ft } else { default.radio_height_ft };

        let door_open_fraction = self.door_open_percent.map(|id| (self.v(&id) / 100.0).clamp(0.0, 1.0));

        let fuel_tank_quantity_gal = Some(self.fuel_tank_quantity.map(|id| self.v(&id)));

        let capt_sidestick_pitch_raw = self.v(&self.sidestick_y);
        let capt_sidestick_roll_raw = self.v(&self.sidestick_x);
        let rudder_pedal_raw = self.v(&self.rudder_pedal_position) / 100.0;

        let body_rate_rad_s = context.rotation_velocity_rad_s();
        let body_rate_pitch_raw = body_rate_rad_s.x;
        let body_rate_yaw_raw = body_rate_rad_s.y;
        let body_rate_roll_raw = body_rate_rad_s.z;

        let sun_elevation_deg = solar_elevation_deg(
            self.v(&self.latitude_deg),
            self.v(&self.longitude_deg),
            self.v(&self.zulu_day_of_year),
            self.v(&self.zulu_time_s),
        );

        Truth {
            dt_s,
            published: Default::default(),
            altitude_ft: self.v(&self.altitude_msl_ft),
            on_ground: on_ground_now,
            environment,
            engine_n1_frac,
            engine_running,
            engine_oil_temp_c,
            engine_oil_pressure_pa,
            engine_oil_quantity_fraction,
            engine_tgt_c,
            engine_t25_c,
            engine_bleed_pressure_pa,
            engine_bleed_temp_k,
            engine_hp_port_pressure_pa,
            engine_hp_port_temp_k,
            engine_ip_port_temp_k,
            engine_ip_port_pressure_pa,
            engine_fuel_flow_kg_s,
            door_open_fraction,
            radio_height_ft,
            fuel_tank_quantity_gal,
            capt_sidestick_pitch_raw,
            capt_sidestick_roll_raw,
            rudder_pedal_raw,
            body_rate_pitch_raw,
            body_rate_yaw_raw,
            body_rate_roll_raw,
            sun_elevation_deg,
            apu_running: self.v(&self.apu_available) != 0.0,
            apu_bleed_pressure_pa,
            ac_bus_volts: self.ac_bus_volts,
            dc_bus_volts: self.dc_bus_volts,
            ac_bus_powered: self.ac_bus_powered,
            dc_bus_powered: self.dc_bus_powered,
            flap_lever_handle_index: self.v(&self.flap_lever_handle_index),
            hydraulic_pressure_pa: self.hydraulic_pressure_psi.map(|id| self.v(&id) * PSI_TO_PA),
            engine_n2_frac,
            engine_n3_frac,
            engine_n2_healthy_frac,
            engine_n3_healthy_frac,
            engine_n1_commanded_pct,
            engine_customer_bleed_kg_s,
            sim_engine_corrected_n1_pct,
            sim_engine_corrected_n2_pct,
            engine_tla_deg,
            to_flex_temp_set,
            aircraft_preset_quick_mode: self.v(&self.preset_quick_mode) != 0.0,
            gpu_plugged_in,
            controls,
            commanded_surfaces,
            aircraft_mass_kg: if total_weight_kg > 0. { total_weight_kg } else { default.aircraft_mass_kg },
            pitch_deg: -context.pitch().get::<degree>(),
            roll_deg: -context.bank().get::<degree>(),
            heading_true_deg: self.v(&self.heading_true_deg),
            groundspeed_m_s: context.ground_speed().get::<meter_per_second>(),
            angle_of_attack_deg: self.v(&self.alpha_deg),
            leg_on_ground,
            leg_touchdown_sink_speed_ms,
            cabin_pressure_pa,
            cabin_temp_k,
            fdac_channel_failure: self.fdac_channel_failure.map(|f| f.map(|id| self.v(&id) != 0.0)),
            ocsm_channel_failure: self.ocsm_channel_failure.map(|o| o.map(|id| self.v(&id) != 0.0)),
            vertical_speed_fpm: context.vertical_speed().get::<foot_per_minute>(),
            landing_elevation_ft,
            athr_status: self.v(&self.athr_status),
            ap1_active: self.v(&self.ap_active[0]) != 0.0,
            ap2_active: self.v(&self.ap_active[1]) != 0.0,
            fmgc_flight_phase: self.v(&self.fmgc_flight_phase),
            flight_ready: self.v(&self.flight_ready) != 0.0,
            prim_left_sidestick_disabled: self.v(&self.prim_left_sidestick_disabled) != 0.0,
            prim_right_sidestick_disabled: self.v(&self.prim_right_sidestick_disabled) != 0.0,
            prim_left_sidestick_priority_locked: self.v(&self.prim_left_sidestick_priority_locked) != 0.0,
            prim_right_sidestick_priority_locked: self.v(&self.prim_right_sidestick_priority_locked) != 0.0,
            rudder_trim_cmd_deg: self.v(&self.sec1_rudder_trim_actual_deg),
            flap_cmd_deg: (self.v(&self.left_flaps_angle_deg) + self.v(&self.right_flaps_angle_deg)) / 2.0,
            slat_cmd_deg: (self.v(&self.left_slats_angle_deg) + self.v(&self.right_slats_angle_deg)) / 2.0,
            droop_cmd_deg: 0.0,
            ir: std::array::from_fn(|n| {
                let word = |p: usize| {
                    let (value, ssm) = unpack_arinc(self.v(&self.ir[n][p]));
                    (ssm == SSM_NORMAL_OPERATION).then_some(value)
                };
                IrOutputs { pitch_deg: word(0), roll_deg: word(1), true_heading_deg: word(2), flight_path_angle_deg: word(3) }
            }),
            att_hdg_switching_knob: self.v(&self.att_hdg_switching_knob),
            prim_healthy: self.prim_healthy.map(|id| self.v(&id) != 0.0),
            sec_healthy: self.sec_healthy.map(|id| self.v(&id) != 0.0),
            ..default
        }
    }
}
