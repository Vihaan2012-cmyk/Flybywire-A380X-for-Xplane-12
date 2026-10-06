use std::collections::BTreeMap;

use crate::deep::live::{Area as LiveArea, Faults, Truth};

use super::actuator::{ActuatorFaults, ActuatorMode, ActuatorGeometry, ElectricPumpFaults, PowerControlUnit, HYDRAULIC_SUPPLY_PA};
use super::allocation::{
    aileron_inboard, aileron_midboard, aileron_outboard, elevator_inboard, elevator_outboard, mode_for_surface, rudder_lower, rudder_upper, ths_motors, ActuatorAllocation,
    ComputerHealth, PowerAvailability, PowerDebounce,
};
use super::high_lift::{HighLiftFaults, HighLiftPair, HighLiftSystem};
use super::hinge_moment::HingeMomentCoefficients;
use super::sensors::{DualTransducer, DualTransducerOutput, TransducerFaults};
use super::spoiler::{GroundSpoilerInputs, GroundSpoilerLogic, GroundSpoilerLogicFaults};
use super::surface::{inertia_uniform_plate_kg_m2, AeroInputs, ControlSurface, SurfaceDamping, SurfaceFaults, SurfaceLimits, SurfaceOutput};
use super::ths::{RudderTrimActuator, ThsFaults, TrimmableHorizontalStabilizer};

const R_AIR_J_KGK: f64 = 287.052_87;

const AILERON_MIN_DEG: f64 = -20.0;
const AILERON_MAX_DEG: f64 = 30.0;
const RUDDER_LIMIT_DEG: f64 = 30.0;
const SPOILER_MAX_DEG: f64 = 50.0;
const SPOILER_1_2_MAX_DEG: f64 = 35.0;

fn spoiler_max_deg(index: usize) -> f64 {
    if index < 2 { SPOILER_1_2_MAX_DEG } else { SPOILER_MAX_DEG }
}

const TRANSDUCER_DISAGREE_RAD: f64 = 2.0 * std::f64::consts::PI / 180.0;
const TRANSDUCER_DISAGREE_TIMER_S: f64 = 1.0;

const HIGH_LIFT_ASYMMETRY_RAD: f64 = 5.0 * std::f64::consts::PI / 180.0;
const HIGH_LIFT_ASYMMETRY_TIMER_S: f64 = 1.0;

const AC_BUS_LIVE_V: f64 = 100.0;

const MS_TO_KT: f64 = 1.943_844_492_440_605;

fn registered_failures() -> BTreeMap<String, Vec<u64>> {
    let mut r = crate::deep::api::Registry::default();
    super::registry::register(&mut r);
    r.components.into_iter().map(|c| (c.id, c.failures)).collect()
}

fn take(map: &BTreeMap<String, Vec<u64>>, component: &str, count: usize) -> Vec<u64> {
    let ids = map.get(component).unwrap_or_else(|| panic!("flight_controls registry has no component {component}"));
    assert_eq!(ids.len(), count, "component {component} registers {} failures, live.rs binds {count}", ids.len());
    ids.clone()
}

#[derive(Clone, Copy, Debug)]
struct SurfaceIds {
    jam: u64,
    runaway: u64,
    supply_loss: u64,
    transducer_fault: u64,
    disconnect: u64,
    flutter_damper_loss: u64,
    valve_leakage: u64,
    piston_seal_wear: u64,
    transducer_drift: u64,
    transducer_open: u64,
    transducer_intermittent: u64,
}

impl SurfaceIds {
    fn build(map: &BTreeMap<String, Vec<u64>>, component: &str) -> Self {
        let i = take(map, component, 11);
        Self {
            jam: i[0],
            runaway: i[1],
            supply_loss: i[2],
            transducer_fault: i[3],
            disconnect: i[4],
            flutter_damper_loss: i[5],
            valve_leakage: i[6],
            piston_seal_wear: i[7],
            transducer_drift: i[8],
            transducer_open: i[9],
            transducer_intermittent: i[10],
        }
    }

    fn actuator_faults(&self, faults: &Faults, travel_span_rad: f64) -> ActuatorFaults {
        let transducer = faults.get(self.transducer_fault);
        ActuatorFaults {
            supply_loss: faults.get(self.supply_loss),
            jam: faults.get(self.jam),
            runaway: faults.get(self.runaway),
            runaway_sign: 1.0,
            transducer_frozen: transducer >= 1.0,
            transducer_bias_rad: if transducer >= 1.0 { 0.0 } else { transducer * travel_span_rad },
            valve_leakage: faults.get(self.valve_leakage),
            piston_seal_wear: faults.get(self.piston_seal_wear),
        }
    }

    fn surface_faults(&self, faults: &Faults) -> SurfaceFaults {
        SurfaceFaults {
            disconnected: faults.get(self.disconnect) >= 0.5,
            flutter_damper_loss: faults.get(self.flutter_damper_loss),
        }
    }

    fn transducer_faults(&self, faults: &Faults) -> TransducerFaults {
        TransducerFaults {
            drift: faults.get(self.transducer_drift),
            open_circuit: faults.get(self.transducer_open),
            intermittent: faults.get(self.transducer_intermittent),
        }
    }

    fn position_fault_active(&self, faults: &Faults) -> bool {
        faults.get(self.jam) > 0.0
            || faults.get(self.runaway) > 0.0
            || faults.get(self.supply_loss) > 0.0
            || faults.get(self.disconnect) > 0.0
            || faults.get(self.flutter_damper_loss) > 0.0
            || faults.get(self.valve_leakage) > 0.0
            || faults.get(self.piston_seal_wear) > 0.0
    }
}

#[derive(Clone, Copy, Debug)]
struct ThsIds {
    motor_green_supply_loss: u64,
    motor_yellow_supply_loss: u64,
    no_back_failure: u64,
    ballscrew_jam: u64,
    transducer_drift: u64,
    transducer_open: u64,
}

impl ThsIds {
    fn position_fault_active(&self, faults: &Faults) -> bool {
        faults.get(self.motor_green_supply_loss) > 0.0
            || faults.get(self.motor_yellow_supply_loss) > 0.0
            || faults.get(self.no_back_failure) > 0.0
            || faults.get(self.ballscrew_jam) > 0.0
    }
}

#[derive(Clone, Copy, Debug)]
struct RudderTrimIds {
    motor_failure: u64,
    jam: u64,
}

#[derive(Clone, Copy, Debug)]
struct HighLiftIds {
    pcu_jam: u64,
    pcu_runaway: u64,
    pcu_supply_loss: u64,
    limiter_bypass: u64,
    inboard_shaft_break: u64,
    outboard_shaft_break: u64,
    wingtip_brake_fail: u64,
}

impl HighLiftIds {
    fn build(map: &BTreeMap<String, Vec<u64>>, component: &str) -> Self {
        let i = take(map, component, 7);
        Self {
            pcu_jam: i[0],
            pcu_runaway: i[1],
            pcu_supply_loss: i[2],
            limiter_bypass: i[3],
            inboard_shaft_break: i[4],
            outboard_shaft_break: i[5],
            wingtip_brake_fail: i[6],
        }
    }

    fn faults(&self, faults: &Faults) -> HighLiftFaults {
        HighLiftFaults {
            pcu: ActuatorFaults {
                jam: faults.get(self.pcu_jam),
                runaway: faults.get(self.pcu_runaway),
                runaway_sign: 1.0,
                supply_loss: faults.get(self.pcu_supply_loss),
                ..ActuatorFaults::default()
            },
            limiter_bypass: faults.get(self.limiter_bypass),
            inboard_shaft_break: faults.get(self.inboard_shaft_break),
            outboard_shaft_break: faults.get(self.outboard_shaft_break),
            wingtip_brake_fail: faults.get(self.wingtip_brake_fail),
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct InputSensorIds {
    chan_a_open: u64,
    chan_a_drift: u64,
    chan_b_open: u64,
    chan_b_drift: u64,
}

impl InputSensorIds {
    fn build(map: &BTreeMap<String, Vec<u64>>, component: &str) -> Self {
        let i = take(map, component, 4);
        Self { chan_a_open: i[0], chan_a_drift: i[1], chan_b_open: i[2], chan_b_drift: i[3] }
    }

    fn chan_a(&self, faults: &Faults) -> TransducerFaults {
        TransducerFaults { open_circuit: faults.get(self.chan_a_open), drift: faults.get(self.chan_a_drift), intermittent: 0.0 }
    }

    fn chan_b(&self, faults: &Faults) -> TransducerFaults {
        TransducerFaults { open_circuit: faults.get(self.chan_b_open), drift: faults.get(self.chan_b_drift), intermittent: 0.0 }
    }
}

#[derive(Clone, Copy, Debug)]
struct PinProgIds {
    mismatch: u64,
}

impl PinProgIds {
    fn build(map: &BTreeMap<String, Vec<u64>>, component: &str) -> Self {
        let i = take(map, component, 1);
        Self { mismatch: i[0] }
    }

    fn disagrees(&self, faults: &Faults) -> bool {
        faults.get(self.mismatch) > 0.0
    }
}

#[derive(Clone, Copy, Debug)]
struct LoadAlleviationIds {
    left: [u64; 3],
    right: [u64; 3],
}

impl LoadAlleviationIds {
    fn build(map: &BTreeMap<String, Vec<u64>>, component: &str) -> Self {
        let i = take(map, component, 6);
        Self { left: [i[0], i[1], i[2]], right: [i[3], i[4], i[5]] }
    }

    fn fails(&self, faults: &Faults) -> bool {
        let failed = |ids: &[u64; 3]| ids.iter().filter(|&&id| faults.get(id) > 0.5).count();
        failed(&self.left) >= 2 || failed(&self.right) >= 2
    }
}

#[derive(Clone, Copy, Debug)]
struct FlapLeverCsuIds {
    chan_1_comm_lost: u64,
    chan_2_comm_lost: u64,
}

impl FlapLeverCsuIds {
    fn build(map: &BTreeMap<String, Vec<u64>>, component: &str) -> Self {
        let i = take(map, component, 2);
        Self { chan_1_comm_lost: i[0], chan_2_comm_lost: i[1] }
    }

    fn faults(&self, faults: &Faults) -> [bool; 2] {
        [faults.get(self.chan_1_comm_lost) > 0.0, faults.get(self.chan_2_comm_lost) > 0.0]
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct InputSensorOutputs {
    l_sidestick_pitch: DualTransducerOutput,
    l_sidestick_roll: DualTransducerOutput,
    rudder_pedal: DualTransducerOutput,
    l_sidestick_disabled_by_takeover: bool,
    r_sidestick_disabled_by_takeover: bool,
    prim_pin_prog_disagree: bool,
    sec_pin_prog_disagree: bool,
    rate_gyro_pitch: DualTransducerOutput,
    rate_gyro_roll: DualTransducerOutput,
    rate_gyro_yaw: DualTransducerOutput,
    r_sidestick_pitch: DualTransducerOutput,
    r_sidestick_roll: DualTransducerOutput,
    prim_elevator_channel_fault: [bool; 3],
    prim_rudder_channel_fault: [bool; 3],
    prim_sidestick_monitor_fault: [bool; 3],
    load_alleviation_fault: bool,
    flap_lever_sys_fault: [bool; 2],
}

const AILERON_COMPONENTS: [[&str; 3]; 2] = [
    ["27_fctl.ail_l3", "27_fctl.ail_l2", "27_fctl.ail_l1"],
    ["27_fctl.ail_r3", "27_fctl.ail_r2", "27_fctl.ail_r1"],
];
const ELEVATOR_COMPONENTS: [[&str; 2]; 2] = [
    ["27_fctl.elev_l_inbd", "27_fctl.elev_l_outbd"],
    ["27_fctl.elev_r_inbd", "27_fctl.elev_r_outbd"],
];
const RUDDER_COMPONENTS: [&str; 2] = ["27_fctl.rud_upper", "27_fctl.rud_lower"];
const HIGH_LIFT_COMPONENTS: [[&str; 2]; 3] =
    [["27_fctl.flap_l", "27_fctl.flap_r"], ["27_fctl.slat_l", "27_fctl.slat_r"], ["27_fctl.droop_l", "27_fctl.droop_r"]];

fn spoiler_component(side: usize, index: usize) -> String {
    format!("27_fctl.splr_{}{}", if side == 0 { 'l' } else { 'r' }, index + 1)
}

fn fault_var(component: &str) -> String {
    let bare = component.split('.').next_back().unwrap_or(component);
    format!("FCTL_{}_FAULT", bare.to_uppercase())
}

fn bare(component: &str) -> String {
    component.split('.').next_back().unwrap_or(component).to_uppercase()
}

fn limits(min_deg: f64, max_deg: f64) -> SurfaceLimits {
    SurfaceLimits { min_rad: min_deg.to_radians(), max_rad: max_deg.to_radians() }
}

fn aileron_surface(panel: usize) -> ControlSurface<2> {
    let (mass_kg, chord_m) = match panel {
        0 => (108.0, 1.6),
        1 => (118.0, 1.37),
        _ => (128.0, 1.4),
    };
    ControlSurface::new(
        std::array::from_fn(|_| PowerControlUnit::new(ActuatorGeometry::aileron())),
        HingeMomentCoefficients::aileron(),
        inertia_uniform_plate_kg_m2(mass_kg, chord_m),
        limits(AILERON_MIN_DEG, AILERON_MAX_DEG),
        SurfaceDamping::aileron(),
        0.0,
    )
}

fn elevator_surface(panel: usize) -> ControlSurface<2> {
    let (mass_kg, chord_m) = if panel == 0 { (189.0, 2.49) } else { (243.0, 2.23) };
    ControlSurface::new(
        std::array::from_fn(|_| PowerControlUnit::new(ActuatorGeometry::elevator())),
        HingeMomentCoefficients::elevator(),
        inertia_uniform_plate_kg_m2(mass_kg, chord_m),
        limits(AILERON_MIN_DEG, AILERON_MAX_DEG),
        SurfaceDamping::elevator(),
        0.0,
    )
}

fn rudder_surface(panel: usize) -> ControlSurface<2> {
    let (mass_kg, chord_m) = if panel == 0 { (357.0, 2.9) } else { (304.0, 3.41) };
    ControlSurface::new(
        std::array::from_fn(|_| PowerControlUnit::new(ActuatorGeometry::rudder())),
        HingeMomentCoefficients::rudder(),
        inertia_uniform_plate_kg_m2(mass_kg, chord_m),
        limits(-RUDDER_LIMIT_DEG, RUDDER_LIMIT_DEG),
        SurfaceDamping::rudder(),
        0.0,
    )
}

fn spoiler_surface(index: usize) -> ControlSurface<1> {
    ControlSurface::new(
        [PowerControlUnit::new(ActuatorGeometry::spoiler())],
        HingeMomentCoefficients::spoiler(),
        inertia_uniform_plate_kg_m2(42.0, 0.685),
        limits(0.0, spoiler_max_deg(index)),
        SurfaceDamping::spoiler(),
        0.0,
    )
}

fn spoiler_is_green(index: usize) -> bool {
    index % 2 == 1
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SurfaceCommands {
    pub rudder_trim_deg: f64,
    pub flap_deg: f64,
    pub slat_deg: f64,
    pub droop_deg: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SurfaceAngles {
    pub ailerons_deg: [[f64; 3]; 2],
    pub ailerons_override_active: [[bool; 3]; 2],
    pub elevators_deg: [[f64; 2]; 2],
    pub elevators_override_active: [[bool; 2]; 2],
    pub rudders_deg: [f64; 2],
    pub rudders_override_active: [bool; 2],
    pub spoilers_deg: [[f64; 8]; 2],
    pub spoilers_override_active: [[bool; 8]; 2],
    pub ths_deg: f64,
    pub ths_override_active: bool,
    pub rudder_trim_deg: f64,
    pub flap_deg: [[f64; 2]; 2],
    pub slat_deg: [[f64; 2]; 2],
    pub droop_deg: [[f64; 2]; 2],
}

pub struct FlightControlsLive {
    ailerons: [[ControlSurface<2>; 3]; 2],
    elevators: [[ControlSurface<2>; 2]; 2],
    rudders: [ControlSurface<2>; 2],
    spoilers: [[ControlSurface<1>; 8]; 2],
    ths: TrimmableHorizontalStabilizer,
    rudder_trim: RudderTrimActuator,
    high_lift: [HighLiftPair; 3],
    ground_spoiler: GroundSpoilerLogic,
    backup: super::backup::BackupControl,
    power_debounce: PowerDebounce,

    aileron_monitors: [[DualTransducer; 3]; 2],
    elevator_monitors: [[DualTransducer; 2]; 2],
    rudder_monitors: [DualTransducer; 2],
    spoiler_monitors: [[DualTransducer; 8]; 2],
    ths_monitor: DualTransducer,

    aileron_ids: [[SurfaceIds; 3]; 2],
    elevator_ids: [[SurfaceIds; 2]; 2],
    rudder_ids: [SurfaceIds; 2],
    spoiler_ids: [[SurfaceIds; 8]; 2],
    ths_ids: ThsIds,
    rudder_trim_ids: RudderTrimIds,
    high_lift_ids: [[HighLiftIds; 2]; 3],
    ground_spoiler_ids: [u64; 2],
    prim_ra_fault_ids: [u64; 3],
    prim_ra_fault: [f64; 3],
    l_sidestick_pitch_monitor: DualTransducer,
    l_sidestick_roll_monitor: DualTransducer,
    rudder_pedal_monitor: DualTransducer,
    l_sidestick_pitch_ids: InputSensorIds,
    l_sidestick_roll_ids: InputSensorIds,
    rudder_pedal_ids: InputSensorIds,
    prim_pin_prog_ids: PinProgIds,
    sec_pin_prog_ids: PinProgIds,
    rate_gyro_pitch_monitor: DualTransducer,
    rate_gyro_roll_monitor: DualTransducer,
    rate_gyro_yaw_monitor: DualTransducer,
    rate_gyro_pitch_ids: InputSensorIds,
    rate_gyro_roll_ids: InputSensorIds,
    rate_gyro_yaw_ids: InputSensorIds,
    r_sidestick_pitch_monitor: DualTransducer,
    r_sidestick_roll_monitor: DualTransducer,
    r_sidestick_pitch_ids: InputSensorIds,
    r_sidestick_roll_ids: InputSensorIds,
    prim_elevator_channel_ids: [u64; 3],
    prim_rudder_channel_ids: [u64; 3],
    prim_sidestick_monitor_ids: [u64; 3],
    load_alleviation_ids: LoadAlleviationIds,
    flap_lever_csu_ids: FlapLeverCsuIds,
    sec_direct_law_channel_ids: [u64; 3],
    fcdc_comm_fault_ids: [u64; 2],
    input_sensor_out: InputSensorOutputs,
    fault_vars: Vec<(String, Vec<u64>)>,
    armed: BTreeMap<u64, f64>,

    angles: SurfaceAngles,
    aileron_out: [[SurfaceOutput; 3]; 2],
    elevator_out: [[SurfaceOutput; 2]; 2],
    rudder_out: [SurfaceOutput; 2],
    spoiler_out: [[SurfaceOutput; 8]; 2],
    ths_out: super::ths::ThsOutput,
    monitor_fault: BTreeMap<String, bool>,
    ground_spoiler_deployed: bool,
    high_lift_brake: [bool; 3],
    high_lift_overspeed_damage: [f64; 3],
    green_demand_m3_s: f64,
    yellow_demand_m3_s: f64,

    commands: SurfaceCommands,
    go_around_selected: bool,
}

impl Default for FlightControlsLive {
    fn default() -> Self {
        Self::new()
    }
}

impl FlightControlsLive {
    pub fn new() -> Self {
        let map = registered_failures();

        let mut fault_vars: Vec<(String, Vec<u64>)> = Vec::new();
        for component in map.keys() {
            fault_vars.push((fault_var(component), map[component].clone()));
        }

        let ths = take(&map, "27_fctl.ths", 6);
        let trim = take(&map, "27_fctl.rudder_trim", 2);
        let gnd = take(&map, "27_fctl.gnd_splr_logic", 2);

        Self {
            ailerons: std::array::from_fn(|_| std::array::from_fn(aileron_surface)),
            elevators: std::array::from_fn(|_| std::array::from_fn(elevator_surface)),
            rudders: std::array::from_fn(rudder_surface),
            spoilers: std::array::from_fn(|_| std::array::from_fn(spoiler_surface)),
            ths: TrimmableHorizontalStabilizer::new_generic(),
            rudder_trim: RudderTrimActuator::new_generic(),
            high_lift: [
                HighLiftPair::new(HighLiftSystem::new_flap(), HighLiftSystem::new_flap(), HIGH_LIFT_ASYMMETRY_TIMER_S),
                HighLiftPair::new(HighLiftSystem::new_slat(), HighLiftSystem::new_slat(), HIGH_LIFT_ASYMMETRY_TIMER_S),
                HighLiftPair::new(HighLiftSystem::new_droop_nose(), HighLiftSystem::new_droop_nose(), HIGH_LIFT_ASYMMETRY_TIMER_S),
            ],
            ground_spoiler: GroundSpoilerLogic::new(),
            backup: super::backup::BackupControl::new(),
            power_debounce: PowerDebounce::default(),

            aileron_monitors: std::array::from_fn(|_| std::array::from_fn(|_| DualTransducer::new(TRANSDUCER_DISAGREE_TIMER_S))),
            elevator_monitors: std::array::from_fn(|_| std::array::from_fn(|_| DualTransducer::new(TRANSDUCER_DISAGREE_TIMER_S))),
            rudder_monitors: std::array::from_fn(|_| DualTransducer::new(TRANSDUCER_DISAGREE_TIMER_S)),
            spoiler_monitors: std::array::from_fn(|_| std::array::from_fn(|_| DualTransducer::new(TRANSDUCER_DISAGREE_TIMER_S))),
            ths_monitor: DualTransducer::new(TRANSDUCER_DISAGREE_TIMER_S),

            aileron_ids: std::array::from_fn(|side| std::array::from_fn(|p| SurfaceIds::build(&map, AILERON_COMPONENTS[side][p]))),
            elevator_ids: std::array::from_fn(|side| std::array::from_fn(|p| SurfaceIds::build(&map, ELEVATOR_COMPONENTS[side][p]))),
            rudder_ids: std::array::from_fn(|p| SurfaceIds::build(&map, RUDDER_COMPONENTS[p])),
            spoiler_ids: std::array::from_fn(|side| std::array::from_fn(|i| SurfaceIds::build(&map, &spoiler_component(side, i)))),
            ths_ids: ThsIds {
                motor_green_supply_loss: ths[0],
                motor_yellow_supply_loss: ths[1],
                no_back_failure: ths[2],
                ballscrew_jam: ths[3],
                transducer_drift: ths[4],
                transducer_open: ths[5],
            },
            rudder_trim_ids: RudderTrimIds { motor_failure: trim[0], jam: trim[1] },
            high_lift_ids: std::array::from_fn(|d| std::array::from_fn(|side| HighLiftIds::build(&map, HIGH_LIFT_COMPONENTS[d][side]))),
            ground_spoiler_ids: [gnd[0], gnd[1]],
            prim_ra_fault_ids: std::array::from_fn(|i| take(&map, &format!("27_fctl.prim_{}", i + 1), 1)[0]),
            prim_ra_fault: [0.0; 3],
            l_sidestick_pitch_monitor: DualTransducer::new(TRANSDUCER_DISAGREE_TIMER_S),
            l_sidestick_roll_monitor: DualTransducer::new(TRANSDUCER_DISAGREE_TIMER_S),
            rudder_pedal_monitor: DualTransducer::new(TRANSDUCER_DISAGREE_TIMER_S),
            l_sidestick_pitch_ids: InputSensorIds::build(&map, "27_fctl.l_sidestick_pitch"),
            l_sidestick_roll_ids: InputSensorIds::build(&map, "27_fctl.l_sidestick_roll"),
            rudder_pedal_ids: InputSensorIds::build(&map, "27_fctl.rudder_pedal"),
            prim_pin_prog_ids: PinProgIds::build(&map, "27_fctl.prim_pin_prog"),
            sec_pin_prog_ids: PinProgIds::build(&map, "27_fctl.sec_pin_prog"),
            rate_gyro_pitch_monitor: DualTransducer::new(TRANSDUCER_DISAGREE_TIMER_S),
            rate_gyro_roll_monitor: DualTransducer::new(TRANSDUCER_DISAGREE_TIMER_S),
            rate_gyro_yaw_monitor: DualTransducer::new(TRANSDUCER_DISAGREE_TIMER_S),
            rate_gyro_pitch_ids: InputSensorIds::build(&map, "27_fctl.rate_gyro_pitch"),
            rate_gyro_roll_ids: InputSensorIds::build(&map, "27_fctl.rate_gyro_roll"),
            rate_gyro_yaw_ids: InputSensorIds::build(&map, "27_fctl.rate_gyro_yaw"),
            r_sidestick_pitch_monitor: DualTransducer::new(TRANSDUCER_DISAGREE_TIMER_S),
            r_sidestick_roll_monitor: DualTransducer::new(TRANSDUCER_DISAGREE_TIMER_S),
            r_sidestick_pitch_ids: InputSensorIds::build(&map, "27_fctl.r_sidestick_pitch"),
            r_sidestick_roll_ids: InputSensorIds::build(&map, "27_fctl.r_sidestick_roll"),
            prim_elevator_channel_ids: std::array::from_fn(|i| take(&map, &format!("27_fctl.prim_{}_elevator_channel", i + 1), 1)[0]),
            prim_rudder_channel_ids: std::array::from_fn(|i| take(&map, &format!("27_fctl.prim_{}_rudder_channel", i + 1), 1)[0]),
            prim_sidestick_monitor_ids: std::array::from_fn(|i| take(&map, &format!("27_fctl.prim_{}_sidestick_monitor", i + 1), 1)[0]),
            load_alleviation_ids: LoadAlleviationIds::build(&map, "27_fctl.load_alleviation"),
            flap_lever_csu_ids: FlapLeverCsuIds::build(&map, "27_fctl.flap_lever_csu"),
            sec_direct_law_channel_ids: std::array::from_fn(|i| take(&map, &format!("27_fctl.sec_{}_direct_law_channel", i + 1), 1)[0]),
            fcdc_comm_fault_ids: std::array::from_fn(|i| take(&map, &format!("27_fctl.fcdc_{}", i + 1), 1)[0]),
            input_sensor_out: InputSensorOutputs::default(),
            fault_vars,
            armed: BTreeMap::new(),

            angles: SurfaceAngles::default(),
            aileron_out: Default::default(),
            elevator_out: Default::default(),
            rudder_out: Default::default(),
            spoiler_out: Default::default(),
            ths_out: Default::default(),
            monitor_fault: BTreeMap::new(),
            ground_spoiler_deployed: false,
            high_lift_brake: [false; 3],
            high_lift_overspeed_damage: [0.0; 3],
            green_demand_m3_s: 0.0,
            yellow_demand_m3_s: 0.0,

            commands: SurfaceCommands::default(),
            go_around_selected: false,
        }
    }

    pub fn set_commands(&mut self, commands: SurfaceCommands) {
        self.commands = commands;
    }

    pub fn set_go_around_selected(&mut self, go_around_selected: bool) {
        self.go_around_selected = go_around_selected;
    }

    pub fn surface_angles(&self) -> SurfaceAngles {
        self.angles
    }

    pub fn ground_spoilers_deployed(&self) -> bool {
        self.ground_spoiler_deployed
    }

    fn aero(&self, truth: &Truth) -> AeroInputs {
        let t_k = (truth.environment.sat_c + 273.15).max(1.0);
        let rho = truth.environment.ambient_pressure_pa.max(0.0) / (R_AIR_J_KGK * t_k);
        let tas = truth.environment.tas_ms.max(0.0);
        AeroInputs {
            dynamic_pressure_pa: 0.5 * rho * tas * tas,
            alpha_rad: truth.angle_of_attack_deg.to_radians(),
            mach: truth.environment.mach(),
            ..AeroInputs::default()
        }
    }

    fn power(truth: &Truth) -> PowerAvailability {
        let frac = |pa: f64| (pa / HYDRAULIC_SUPPLY_PA).clamp(0.0, 1.0);
        PowerAvailability {
            green: frac(truth.hydraulic_pressure_pa[0]),
            yellow: frac(truth.hydraulic_pressure_pa[1]),
            eha: if truth.ac_bus_volts.iter().any(|&v| v > AC_BUS_LIVE_V) { 1.0 } else { 0.0 },
        }
    }

    fn pressures<const N: usize>(allocations: &[ActuatorAllocation; N], power: &PowerAvailability) -> [f64; N] {
        std::array::from_fn(|i| {
            use super::allocation::PowerSource;
            match allocations[i].options.first().map(|&(_, source)| source) {
                Some(PowerSource::Green) => power.green,
                Some(PowerSource::Yellow) => power.yellow,
                Some(PowerSource::Eha) => power.eha,
                None => 0.0,
            }
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn step_surface<const N: usize>(
        surface: &mut ControlSurface<N>,
        monitor: &mut DualTransducer,
        ids: &SurfaceIds,
        allocations: &[ActuatorAllocation; N],
        health: &ComputerHealth,
        power: &PowerAvailability,
        mode_power: &PowerAvailability,
        faults: &Faults,
        commanded_deg: f64,
        travel_span_rad: f64,
        aero: &AeroInputs,
        dt: f64,
    ) -> (SurfaceOutput, bool, [ActuatorMode; N]) {
        let (modes, _) = mode_for_surface(allocations, health, mode_power);
        let pressures = Self::pressures(allocations, power);
        let actuator_faults = ids.actuator_faults(faults, travel_span_rad);
        let out = surface.step(
            modes,
            commanded_deg.to_radians(),
            pressures,
            [actuator_faults; N],
            &ids.surface_faults(faults),
            aero,
            dt,
        );
        let monitoring = monitor
            .step(out.angle_rad, &ids.transducer_faults(faults), &TransducerFaults::default(), TRANSDUCER_DISAGREE_RAD, dt)
            .monitoring_fault;
        (out, monitoring, modes)
    }

    fn accumulate_flow_demand<const N: usize>(
        allocations: &[ActuatorAllocation; N],
        modes: [ActuatorMode; N],
        rate_rad_s: f64,
        geometry: ActuatorGeometry,
        green_m3_s: &mut f64,
        yellow_m3_s: &mut f64,
    ) {
        use super::allocation::PowerSource;
        let flow = rate_rad_s.abs() * geometry.arm_m * geometry.bore_area_m2;
        for i in 0..N {
            if modes[i] == ActuatorMode::Active {
                match allocations[i].options.first().map(|&(_, source)| source) {
                    Some(PowerSource::Green) => *green_m3_s += flow,
                    Some(PowerSource::Yellow) => *yellow_m3_s += flow,
                    _ => {}
                }
            }
        }
    }
}

impl LiveArea for FlightControlsLive {
    fn name(&self) -> &'static str {
        "flight_controls"
    }

    fn flight_control_surface_angles(&self) -> Option<SurfaceAngles> {
        Some(self.surface_angles())
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        let dt = truth.dt_s.max(0.0);
        self.armed.clear();
        for (_, ids) in &self.fault_vars {
            for &id in ids {
                let magnitude = faults.get(id);
                if magnitude > 0.0 {
                    self.armed.insert(id, magnitude);
                }
            }
        }
        for i in 0..3 {
            self.prim_ra_fault[i] = faults.get(self.prim_ra_fault_ids[i]);
        }
        let aero = self.aero(truth);
        let power = Self::power(truth);
        let mode_power = self.power_debounce.step(&power);
        let cmd = &truth.commanded_surfaces;
        let mut green_demand_m3_s = 0.0_f64;
        let mut yellow_demand_m3_s = 0.0_f64;
        let ac_bus_powered = truth.ac_bus_volts.iter().any(|&v| v > AC_BUS_LIVE_V);
        let health = ComputerHealth { prim: truth.prim_healthy, sec: truth.sec_healthy };

        let _backup_commands = self.backup.step(
            &health,
            truth.hydraulic_pressure_pa,
            &super::backup::BackupInceptors::default(),
            dt,
        );

        let aileron_span = (AILERON_MAX_DEG - AILERON_MIN_DEG).to_radians();
        let rudder_span = (2.0 * RUDDER_LIMIT_DEG).to_radians();
        let spoiler_span = SPOILER_MAX_DEG.to_radians();

        let aileron_allocations = [aileron_inboard(), aileron_midboard(), aileron_outboard()];
        let elevator_allocations = [elevator_inboard(), elevator_outboard()];
        let rudder_allocations = [rudder_upper(), rudder_lower()];

        for side in 0..2 {
            for panel in 0..3 {
                let (out, monitoring, modes) = Self::step_surface(
                    &mut self.ailerons[side][panel],
                    &mut self.aileron_monitors[side][panel],
                    &self.aileron_ids[side][panel],
                    &aileron_allocations[panel],
                    &health,
                    &power,
                    &mode_power,
                    faults,
                    cmd.ailerons_deg[side][panel],
                    aileron_span,
                    &aero,
                    dt,
                );
                Self::accumulate_flow_demand(&aileron_allocations[panel], modes, out.rate_rad_s, ActuatorGeometry::aileron(), &mut green_demand_m3_s, &mut yellow_demand_m3_s);
                self.aileron_out[side][panel] = out;
                self.angles.ailerons_deg[side][panel] = out.angle_rad.to_degrees();
                self.angles.ailerons_override_active[side][panel] = self.aileron_ids[side][panel].position_fault_active(faults);
                self.monitor_fault.insert(bare(AILERON_COMPONENTS[side][panel]), monitoring);
            }
            for panel in 0..2 {
                let (out, monitoring, modes) = Self::step_surface(
                    &mut self.elevators[side][panel],
                    &mut self.elevator_monitors[side][panel],
                    &self.elevator_ids[side][panel],
                    &elevator_allocations[panel],
                    &health,
                    &power,
                    &mode_power,
                    faults,
                    cmd.elevators_deg[side][panel],
                    aileron_span,
                    &aero,
                    dt,
                );
                Self::accumulate_flow_demand(&elevator_allocations[panel], modes, out.rate_rad_s, ActuatorGeometry::elevator(), &mut green_demand_m3_s, &mut yellow_demand_m3_s);
                self.elevator_out[side][panel] = out;
                self.angles.elevators_deg[side][panel] = out.angle_rad.to_degrees();
                self.angles.elevators_override_active[side][panel] = self.elevator_ids[side][panel].position_fault_active(faults);
                self.monitor_fault.insert(bare(ELEVATOR_COMPONENTS[side][panel]), monitoring);
            }
        }

        for panel in 0..2 {
            let (out, monitoring, modes) = Self::step_surface(
                &mut self.rudders[panel],
                &mut self.rudder_monitors[panel],
                &self.rudder_ids[panel],
                &rudder_allocations[panel],
                &health,
                &power,
                &mode_power,
                faults,
                cmd.rudders_deg[panel],
                rudder_span,
                &aero,
                dt,
            );
            Self::accumulate_flow_demand(&rudder_allocations[panel], modes, out.rate_rad_s, ActuatorGeometry::rudder(), &mut green_demand_m3_s, &mut yellow_demand_m3_s);
            self.rudder_out[panel] = out;
            self.angles.rudders_deg[panel] = out.angle_rad.to_degrees();
            self.angles.rudders_override_active[panel] = self.rudder_ids[panel].position_fault_active(faults);
            self.monitor_fault.insert(bare(RUDDER_COMPONENTS[panel]), monitoring);
        }

        let on_ground = truth.on_ground;
        let ground_spoiler_faults = GroundSpoilerLogicFaults {
            fails_to_deploy: faults.get(self.ground_spoiler_ids[0]),
            fails_to_retract: faults.get(self.ground_spoiler_ids[1]),
        };
        let ground_spoiler_out = self.ground_spoiler.step(
            &GroundSpoilerInputs {
                lever_armed: truth.controls.ground_spoiler_lever_armed,
                main_gear_wow: [on_ground; 2],
                wheel_speed_kt: if on_ground { truth.environment.tas_ms * MS_TO_KT } else { 0.0 },
                radio_alt_ft: if on_ground { 0.0 } else { truth.altitude_ft },
                go_around_selected: self.go_around_selected,
            },
            &ground_spoiler_faults,
        );
        self.ground_spoiler_deployed = ground_spoiler_out.deployed;

        for side in 0..2 {
            for i in 0..8 {
                let commanded_deg = cmd.spoilers_deg[side][i].max(ground_spoiler_out.deploy_command * SPOILER_MAX_DEG);
                let source = if spoiler_is_green(i) { power.green } else { power.yellow };
                let supply = if i == 5 { source.max(power.eha) } else { source };
                let ids = &self.spoiler_ids[side][i];
                let actuator_faults = ids.actuator_faults(faults, spoiler_span);
                let active = supply > 0.5;
                let out = self.spoilers[side][i].step(
                    [if active { ActuatorMode::Active } else { ActuatorMode::Damping }],
                    commanded_deg.to_radians(),
                    [supply],
                    [actuator_faults],
                    &ids.surface_faults(faults),
                    &aero,
                    dt,
                );
                if active && source > 0.5 {
                    let geometry = ActuatorGeometry::spoiler();
                    let flow = out.rate_rad_s.abs() * geometry.arm_m * geometry.bore_area_m2;
                    if spoiler_is_green(i) {
                        green_demand_m3_s += flow;
                    } else {
                        yellow_demand_m3_s += flow;
                    }
                }
                let monitoring = self.spoiler_monitors[side][i]
                    .step(out.angle_rad, &ids.transducer_faults(faults), &TransducerFaults::default(), TRANSDUCER_DISAGREE_RAD, dt)
                    .monitoring_fault;
                self.spoiler_out[side][i] = out;
                self.angles.spoilers_deg[side][i] = out.angle_rad.to_degrees();
                self.angles.spoilers_override_active[side][i] = ids.position_fault_active(faults);
                self.monitor_fault.insert(bare(&spoiler_component(side, i)), monitoring);
            }
        }

        let ths_allocations = ths_motors();
        let (mut ths_modes, _) = mode_for_surface(&ths_allocations, &health, &power);
        let ths_command_rad = cmd.ths_deg.to_radians();
        if (ths_command_rad - self.ths.angle_rad()).abs() < 1.0e-4 {
            ths_modes = [ActuatorMode::Damping; 2];
        }
        let ths_faults = ThsFaults {
            motor_green: ActuatorFaults { supply_loss: faults.get(self.ths_ids.motor_green_supply_loss), ..ActuatorFaults::default() },
            motor_yellow: ActuatorFaults { supply_loss: faults.get(self.ths_ids.motor_yellow_supply_loss), ..ActuatorFaults::default() },
            no_back_failure: faults.get(self.ths_ids.no_back_failure),
            ballscrew_jam: faults.get(self.ths_ids.ballscrew_jam),
        };
        self.ths_out = self.ths.step(ths_modes, ths_command_rad, [power.green, power.yellow], &ths_faults, &aero, dt);
        self.angles.ths_deg = self.ths.angle_deg();
        self.angles.ths_override_active = self.ths_ids.position_fault_active(faults);
        let ths_monitoring = self
            .ths_monitor
            .step(
                self.ths.angle_rad(),
                &TransducerFaults {
                    drift: faults.get(self.ths_ids.transducer_drift),
                    open_circuit: faults.get(self.ths_ids.transducer_open),
                    intermittent: 0.0,
                },
                &TransducerFaults::default(),
                TRANSDUCER_DISAGREE_RAD,
                dt,
            )
            .monitoring_fault;
        self.monitor_fault.insert("THS".to_string(), ths_monitoring);

        let trim_power = if ac_bus_powered { 1.0 } else { 0.0 };
        self.angles.rudder_trim_deg = self
            .rudder_trim
            .step(
                truth.rudder_trim_cmd_deg.to_radians(),
                trim_power,
                &ElectricPumpFaults { motor_failure: faults.get(self.rudder_trim_ids.motor_failure) },
                &ActuatorFaults { jam: faults.get(self.rudder_trim_ids.jam), runaway_sign: 1.0, ..ActuatorFaults::default() },
                dt,
            )
            .to_degrees();

        let high_lift_supply = power.green.max(power.yellow);
        let high_lift_commands = [truth.flap_cmd_deg, truth.slat_cmd_deg, truth.droop_cmd_deg];
        for d in 0..3 {
            let device_faults = [self.high_lift_ids[d][0].faults(faults), self.high_lift_ids[d][1].faults(faults)];
            let (left, right, tripped) = self.high_lift[d].step(
                if high_lift_supply > 0.5 { ActuatorMode::Active } else { ActuatorMode::Damping },
                high_lift_commands[d].to_radians(),
                [high_lift_supply; 2],
                device_faults,
                aero.dynamic_pressure_pa,
                HIGH_LIFT_ASYMMETRY_RAD,
                dt,
            );
            self.high_lift_brake[d] = tripped;
            self.high_lift_overspeed_damage[d] = left.overspeed_damage.max(right.overspeed_damage);
            let target = match d {
                0 => &mut self.angles.flap_deg,
                1 => &mut self.angles.slat_deg,
                _ => &mut self.angles.droop_deg,
            };
            target[0] = [left.inboard_angle_rad.to_degrees(), left.outboard_angle_rad.to_degrees()];
            target[1] = [right.inboard_angle_rad.to_degrees(), right.outboard_angle_rad.to_degrees()];
        }

        self.green_demand_m3_s = green_demand_m3_s;
        self.yellow_demand_m3_s = yellow_demand_m3_s;

        self.input_sensor_out.l_sidestick_pitch = self.l_sidestick_pitch_monitor.step(
            truth.capt_sidestick_pitch_raw,
            &self.l_sidestick_pitch_ids.chan_a(faults),
            &self.l_sidestick_pitch_ids.chan_b(faults),
            TRANSDUCER_DISAGREE_RAD,
            dt,
        );
        self.input_sensor_out.l_sidestick_roll = self.l_sidestick_roll_monitor.step(
            truth.capt_sidestick_roll_raw,
            &self.l_sidestick_roll_ids.chan_a(faults),
            &self.l_sidestick_roll_ids.chan_b(faults),
            TRANSDUCER_DISAGREE_RAD,
            dt,
        );
        self.input_sensor_out.rudder_pedal = self.rudder_pedal_monitor.step(
            truth.rudder_pedal_raw,
            &self.rudder_pedal_ids.chan_a(faults),
            &self.rudder_pedal_ids.chan_b(faults),
            TRANSDUCER_DISAGREE_RAD,
            dt,
        );

        self.input_sensor_out.l_sidestick_disabled_by_takeover = truth.prim_left_sidestick_disabled;
        self.input_sensor_out.r_sidestick_disabled_by_takeover = truth.prim_right_sidestick_disabled;

        self.input_sensor_out.prim_pin_prog_disagree = self.prim_pin_prog_ids.disagrees(faults);
        self.input_sensor_out.sec_pin_prog_disagree = self.sec_pin_prog_ids.disagrees(faults);

        self.input_sensor_out.rate_gyro_pitch = self.rate_gyro_pitch_monitor.step(
            truth.body_rate_pitch_raw,
            &self.rate_gyro_pitch_ids.chan_a(faults),
            &self.rate_gyro_pitch_ids.chan_b(faults),
            TRANSDUCER_DISAGREE_RAD,
            dt,
        );
        self.input_sensor_out.rate_gyro_roll = self.rate_gyro_roll_monitor.step(
            truth.body_rate_roll_raw,
            &self.rate_gyro_roll_ids.chan_a(faults),
            &self.rate_gyro_roll_ids.chan_b(faults),
            TRANSDUCER_DISAGREE_RAD,
            dt,
        );
        self.input_sensor_out.rate_gyro_yaw = self.rate_gyro_yaw_monitor.step(
            truth.body_rate_yaw_raw,
            &self.rate_gyro_yaw_ids.chan_a(faults),
            &self.rate_gyro_yaw_ids.chan_b(faults),
            TRANSDUCER_DISAGREE_RAD,
            dt,
        );

        self.input_sensor_out.r_sidestick_pitch =
            self.r_sidestick_pitch_monitor.step(0.0, &self.r_sidestick_pitch_ids.chan_a(faults), &self.r_sidestick_pitch_ids.chan_b(faults), TRANSDUCER_DISAGREE_RAD, dt);
        self.input_sensor_out.r_sidestick_roll =
            self.r_sidestick_roll_monitor.step(0.0, &self.r_sidestick_roll_ids.chan_a(faults), &self.r_sidestick_roll_ids.chan_b(faults), TRANSDUCER_DISAGREE_RAD, dt);

        for i in 0..3 {
            let healthy = truth.prim_healthy[i];
            self.input_sensor_out.prim_elevator_channel_fault[i] = healthy && faults.get(self.prim_elevator_channel_ids[i]) > 0.0;
            self.input_sensor_out.prim_rudder_channel_fault[i] = healthy && faults.get(self.prim_rudder_channel_ids[i]) > 0.0;
            self.input_sensor_out.prim_sidestick_monitor_fault[i] = healthy && faults.get(self.prim_sidestick_monitor_ids[i]) > 0.0;
        }

        self.input_sensor_out.load_alleviation_fault = self.load_alleviation_ids.fails(faults);

        self.input_sensor_out.flap_lever_sys_fault = self.flap_lever_csu_ids.faults(faults);
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        let mut failed_spoilers = 0u32;
        for (name, ids) in &self.fault_vars {
            let worst = ids.iter().map(|&id| self.armed.get(&id).copied().unwrap_or(0.0)).fold(0.0_f64, f64::max);
            out(name, worst);
            if name.starts_with("FCTL_SPLR_") && worst > 0.5 {
                failed_spoilers += 1;
            }
        }
        out("FCTL_SPLR_FAILED_COUNT", f64::from(failed_spoilers));

        let ra_letters = ["A", "B", "C"];
        for prim in 0..3 {
            for (ra, letter) in ra_letters.iter().enumerate() {
                let using = !(prim == ra && self.prim_ra_fault[prim] > 0.0);
                out(&format!("DEEP_PRIM_{}_USING_RA_{}", prim + 1, letter), if using { 1.0 } else { 0.0 });
            }
        }

        for side in 0..2 {
            for panel in 0..3 {
                let n = bare(AILERON_COMPONENTS[side][panel]);
                out(&format!("FCTL_{n}_DEFLECTION_DEG"), self.angles.ailerons_deg[side][panel]);
                out(&format!("FCTL_{n}_BLOWN_BACK"), f64::from(u8::from(self.aileron_out[side][panel].blown_back)));
            }
            for panel in 0..2 {
                let n = bare(ELEVATOR_COMPONENTS[side][panel]);
                out(&format!("FCTL_{n}_DEFLECTION_DEG"), self.angles.elevators_deg[side][panel]);
                out(&format!("FCTL_{n}_BLOWN_BACK"), f64::from(u8::from(self.elevator_out[side][panel].blown_back)));
            }
            for i in 0..8 {
                let n = bare(&spoiler_component(side, i));
                out(&format!("FCTL_{n}_DEFLECTION_DEG"), self.angles.spoilers_deg[side][i]);
                out(&format!("FCTL_{n}_BLOWN_BACK"), f64::from(u8::from(self.spoiler_out[side][i].blown_back)));
            }
        }
        for panel in 0..2 {
            let n = bare(RUDDER_COMPONENTS[panel]);
            out(&format!("FCTL_{n}_DEFLECTION_DEG"), self.angles.rudders_deg[panel]);
            out(&format!("FCTL_{n}_BLOWN_BACK"), f64::from(u8::from(self.rudder_out[panel].blown_back)));
        }
        out("FCTL_THS_DEFLECTION_DEG", self.angles.ths_deg);
        out("FCTL_THS_NO_BACK_ENGAGED", f64::from(u8::from(self.ths_out.no_back_engaged)));
        out("FCTL_RUDDER_TRIM_DEFLECTION_DEG", self.angles.rudder_trim_deg);
        out("FCTL_BPS_1_AVAILABLE", f64::from(u8::from(self.backup.supplies[0].available())));
        out("FCTL_BPS_2_AVAILABLE", f64::from(u8::from(self.backup.supplies[1].available())));
        out("FCTL_BCM_ENGAGED", f64::from(u8::from(self.backup.module.engaged())));

        const HIGH_LIFT_NAMES: [&str; 3] = ["FLAP", "SLAT", "DROOP"];
        let high_lift_angles = [self.angles.flap_deg, self.angles.slat_deg, self.angles.droop_deg];
        for d in 0..3 {
            for (side, tag) in [(0usize, 'L'), (1usize, 'R')] {
                out(&format!("FCTL_{}_{tag}_INBOARD_DEG", HIGH_LIFT_NAMES[d]), high_lift_angles[d][side][0]);
                out(&format!("FCTL_{}_{tag}_OUTBOARD_DEG", HIGH_LIFT_NAMES[d]), high_lift_angles[d][side][1]);
            }
            out(&format!("FCTL_{}_WINGTIP_BRAKE_ON", HIGH_LIFT_NAMES[d]), f64::from(u8::from(self.high_lift_brake[d])));
            out(&format!("FCTL_{}_OVERSPEED_DAMAGE", HIGH_LIFT_NAMES[d]), self.high_lift_overspeed_damage[d]);
        }

        out("FCTL_GND_SPLR_DEPLOYED", f64::from(u8::from(self.ground_spoiler_deployed)));

        out("FCTL_GREEN_DEMAND_M3_S", self.green_demand_m3_s);
        out("FCTL_YELLOW_DEMAND_M3_S", self.yellow_demand_m3_s);

        for (component, fault) in &self.monitor_fault {
            out(&format!("FCTL_{component}_POSITION_MONITOR_FAULT"), f64::from(u8::from(*fault)));
        }

        let io = &self.input_sensor_out;
        out("FCTL_L_SIDESTICK_PITCH_FAULT", f64::from(u8::from(io.l_sidestick_pitch.consolidated_rad.is_none())));
        out("FCTL_L_SIDESTICK_ROLL_FAULT", f64::from(u8::from(io.l_sidestick_roll.consolidated_rad.is_none())));
        out("FCTL_L_SIDESTICK_PITCH_SENSOR_FAULT", f64::from(u8::from(io.l_sidestick_pitch.monitoring_fault)));
        out("FCTL_L_SIDESTICK_ROLL_SENSOR_FAULT", f64::from(u8::from(io.l_sidestick_roll.monitoring_fault)));
        out("FCTL_RUDDER_PEDAL_FAULT", f64::from(u8::from(io.rudder_pedal.consolidated_rad.is_none())));
        out("FCTL_RUDDER_PEDAL_SENSOR_FAULT", f64::from(u8::from(io.rudder_pedal.monitoring_fault)));
        out("FCTL_L_SIDESTICK_DISABLED_BY_TAKEOVER", f64::from(u8::from(io.l_sidestick_disabled_by_takeover)));
        out("FCTL_R_SIDESTICK_DISABLED_BY_TAKEOVER", f64::from(u8::from(io.r_sidestick_disabled_by_takeover)));
        out("FCTL_PRIM_VERSIONS_DISAGREE", f64::from(u8::from(io.prim_pin_prog_disagree)));
        out("FCTL_PRIM_PIN_PROG_DISAGREE", f64::from(u8::from(io.prim_pin_prog_disagree)));
        out("FCTL_SEC_VERSIONS_DISAGREE", f64::from(u8::from(io.sec_pin_prog_disagree)));

        out("FCTL_RATE_GYRO_PITCH_FAULT", f64::from(u8::from(io.rate_gyro_pitch.consolidated_rad.is_none())));
        out("FCTL_RATE_GYRO_ROLL_FAULT", f64::from(u8::from(io.rate_gyro_roll.consolidated_rad.is_none())));
        out("FCTL_RATE_GYRO_YAW_FAULT", f64::from(u8::from(io.rate_gyro_yaw.consolidated_rad.is_none())));
        out("FCTL_R_SIDESTICK_PITCH_FAULT", f64::from(u8::from(io.r_sidestick_pitch.consolidated_rad.is_none())));
        out("FCTL_R_SIDESTICK_ROLL_FAULT", f64::from(u8::from(io.r_sidestick_roll.consolidated_rad.is_none())));
        out("FCTL_R_SIDESTICK_PITCH_SENSOR_FAULT", f64::from(u8::from(io.r_sidestick_pitch.monitoring_fault)));
        out("FCTL_R_SIDESTICK_ROLL_SENSOR_FAULT", f64::from(u8::from(io.r_sidestick_roll.monitoring_fault)));
        for i in 0..3 {
            out(&format!("FCTL_PRIM_{}_ELEVATOR_CHANNEL_FAULT", i + 1), f64::from(u8::from(io.prim_elevator_channel_fault[i])));
            out(&format!("FCTL_PRIM_{}_RUDDER_CHANNEL_FAULT", i + 1), f64::from(u8::from(io.prim_rudder_channel_fault[i])));
            out(&format!("FCTL_PRIM_{}_SIDESTICK_MONITOR_FAULT", i + 1), f64::from(u8::from(io.prim_sidestick_monitor_fault[i])));
        }
        out("FCTL_LOAD_ALLEVIATION_FAULT", f64::from(u8::from(io.load_alleviation_fault)));
        out("FCTL_FLAPS_LEVER_SYS_1_FAULT", f64::from(u8::from(io.flap_lever_sys_fault[0])));
        out("FCTL_FLAPS_LEVER_SYS_2_FAULT", f64::from(u8::from(io.flap_lever_sys_fault[1])));
    }
}

pub fn live_system() -> Box<dyn LiveArea> {
    Box::new(FlightControlsLive::new())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deep::api::Registry;

    fn flying_truth() -> Truth {
        let mut t = Truth {
            dt_s: 0.02,
            on_ground: false,
            altitude_ft: 5000.0,
            ac_bus_volts: [115.0; 4],
            dc_bus_volts: [28.0; 2],
            prim_healthy: [true; 3],
            sec_healthy: [true; 3],
            hydraulic_pressure_pa: [HYDRAULIC_SUPPLY_PA; 2],
            ..Truth::default()
        };
        t.environment.tas_ms = 200.0;
        t
    }

    fn parked_truth() -> Truth {
        Truth {
            dt_s: 0.02,
            on_ground: true,
            ac_bus_volts: [115.0; 4],
            dc_bus_volts: [28.0; 2],
            prim_healthy: [true; 3],
            sec_healthy: [true; 3],
            hydraulic_pressure_pa: [HYDRAULIC_SUPPLY_PA; 2],
            ..Truth::default()
        }
    }

    #[test]
    fn the_backup_control_path_engages_only_with_every_computer_lost() {
        let mut area = FlightControlsLive::default();
        let healthy = run(&mut area, &flying_truth(), &Faults::default(), 5.0);
        assert_eq!(healthy["FCTL_BCM_ENGAGED"], 0.0, "a healthy aircraft must never engage backup control");
        assert_eq!(healthy["FCTL_BPS_1_AVAILABLE"], 0.0);
        assert_eq!(healthy["FCTL_BPS_2_AVAILABLE"], 0.0);

        let mut partial = flying_truth();
        partial.prim_healthy[1] = false;
        partial.sec_healthy[1] = false;
        let mut area = FlightControlsLive::default();
        let out = run(&mut area, &partial, &Faults::default(), 5.0);
        assert_eq!(out["FCTL_BPS_1_AVAILABLE"], 1.0, "PRIM 2 + SEC 2 lost should spin the supplies up");
        assert_eq!(out["FCTL_BPS_2_AVAILABLE"], 1.0);
        assert_eq!(out["FCTL_BCM_ENGAGED"], 0.0, "four computers still fly the aircraft");

        let mut dead = flying_truth();
        dead.prim_healthy = [false; 3];
        dead.sec_healthy = [false; 3];
        let mut area = FlightControlsLive::default();
        let out = run(&mut area, &dead, &Faults::default(), 5.0);
        assert_eq!(out["FCTL_BCM_ENGAGED"], 1.0, "with no computer left the BCM must take over");

        let mut no_hyd = dead.clone();
        no_hyd.hydraulic_pressure_pa = [0.0; 2];
        let mut area = FlightControlsLive::default();
        let out = run(&mut area, &no_hyd, &Faults::default(), 5.0);
        assert_eq!(out["FCTL_BCM_ENGAGED"], 0.0, "no hydraulics means no backup power supply");
    }

    fn run(area: &mut FlightControlsLive, truth: &Truth, faults: &Faults, seconds: f64) -> BTreeMap<String, f64> {
        let ticks = (seconds / truth.dt_s).round() as usize;
        for _ in 0..ticks {
            area.tick(truth, faults);
        }
        let mut published = BTreeMap::new();
        area.publish(&mut |name, value| {
            published.insert(name.to_string(), value);
        });
        published
    }

    fn ids() -> FlightControlsLive {
        FlightControlsLive::new()
    }

    fn push_surface(s: &SurfaceIds, bound: &mut Vec<u64>) {
        bound.extend([
            s.jam,
            s.runaway,
            s.supply_loss,
            s.transducer_fault,
            s.disconnect,
            s.flutter_damper_loss,
            s.valve_leakage,
            s.piston_seal_wear,
            s.transducer_drift,
            s.transducer_open,
            s.transducer_intermittent,
        ]);
    }

    #[test]
    fn every_registered_failure_id_reaches_a_model_field() {
        let live = ids();
        let mut bound: Vec<u64> = Vec::new();
        for side in 0..2 {
            for p in 0..3 {
                push_surface(&live.aileron_ids[side][p], &mut bound);
            }
            for p in 0..2 {
                push_surface(&live.elevator_ids[side][p], &mut bound);
            }
            for i in 0..8 {
                push_surface(&live.spoiler_ids[side][i], &mut bound);
            }
        }
        for p in 0..2 {
            push_surface(&live.rudder_ids[p], &mut bound);
        }
        let t = live.ths_ids;
        bound.extend([t.motor_green_supply_loss, t.motor_yellow_supply_loss, t.no_back_failure, t.ballscrew_jam, t.transducer_drift, t.transducer_open]);
        bound.extend([live.rudder_trim_ids.motor_failure, live.rudder_trim_ids.jam]);
        for d in 0..3 {
            for side in 0..2 {
                let h = live.high_lift_ids[d][side];
                bound.extend([h.pcu_jam, h.pcu_runaway, h.pcu_supply_loss, h.limiter_bypass, h.inboard_shaft_break, h.outboard_shaft_break, h.wingtip_brake_fail]);
            }
        }
        bound.extend(live.ground_spoiler_ids);
        bound.extend(live.prim_ra_fault_ids);
        for s in [live.l_sidestick_pitch_ids, live.l_sidestick_roll_ids, live.rudder_pedal_ids] {
            bound.extend([s.chan_a_open, s.chan_a_drift, s.chan_b_open, s.chan_b_drift]);
        }
        bound.push(live.prim_pin_prog_ids.mismatch);
        bound.push(live.sec_pin_prog_ids.mismatch);
        for s in [live.rate_gyro_pitch_ids, live.rate_gyro_roll_ids, live.rate_gyro_yaw_ids, live.r_sidestick_pitch_ids, live.r_sidestick_roll_ids] {
            bound.extend([s.chan_a_open, s.chan_a_drift, s.chan_b_open, s.chan_b_drift]);
        }
        bound.extend(live.prim_elevator_channel_ids);
        bound.extend(live.prim_rudder_channel_ids);
        bound.extend(live.prim_sidestick_monitor_ids);
        bound.extend(live.load_alleviation_ids.left);
        bound.extend(live.load_alleviation_ids.right);
        bound.extend([live.flap_lever_csu_ids.chan_1_comm_lost, live.flap_lever_csu_ids.chan_2_comm_lost]);
        bound.extend(live.sec_direct_law_channel_ids);
        bound.extend(live.fcdc_comm_fault_ids);
        bound.sort_unstable();

        let mut r = Registry::default();
        super::super::registry::register(&mut r);
        let mut registered: Vec<u64> = r.failures.iter().map(|f| f.id).collect();
        registered.sort_unstable();
        assert_eq!(bound, registered, "every registered flight-control failure must reach a model field");
    }

    #[test]
    fn all_fifty_eight_components_publish_the_fault_variable_their_ecam_trigger_reads() {
        let live = ids();
        assert_eq!(live.fault_vars.len(), 66);
        let published = run(&mut ids(), &parked_truth(), &Faults::default(), 0.1);
        for name in [
            "FCTL_AIL_L1_FAULT",
            "FCTL_ELEV_R_OUTBD_FAULT",
            "FCTL_RUD_UPPER_FAULT",
            "FCTL_SPLR_R8_FAULT",
            "FCTL_THS_FAULT",
            "FCTL_RUDDER_TRIM_FAULT",
            "FCTL_FLAP_L_FAULT",
            "FCTL_SLAT_R_FAULT",
            "FCTL_DROOP_R_FAULT",
            "FCTL_GND_SPLR_LOGIC_FAULT",
            "FCTL_L_SIDESTICK_PITCH_FAULT",
            "FCTL_L_SIDESTICK_ROLL_FAULT",
            "FCTL_RUDDER_PEDAL_FAULT",
            "FCTL_PRIM_PIN_PROG_FAULT",
            "FCTL_SEC_PIN_PROG_FAULT",
            "FCTL_RATE_GYRO_PITCH_FAULT",
            "FCTL_R_SIDESTICK_PITCH_FAULT",
            "FCTL_PRIM_1_ELEVATOR_CHANNEL_FAULT",
            "FCTL_PRIM_2_RUDDER_CHANNEL_FAULT",
            "FCTL_PRIM_3_SIDESTICK_MONITOR_FAULT",
            "FCTL_LOAD_ALLEVIATION_FAULT",
            "FCTL_FLAP_LEVER_CSU_FAULT",
            "FCTL_SEC_1_DIRECT_LAW_CHANNEL_FAULT",
            "FCTL_FCDC_1_FAULT",
        ] {
            assert!(published.contains_key(name), "missing {name}");
            assert_eq!(published[name], 0.0, "{name} must read healthy with nothing armed");
        }
    }

    #[test]
    fn a_jammed_aileron_pins_at_its_jam_angle_while_its_neighbours_follow_the_command() {
        let live = ids();
        let faults = Faults::from_pairs([(live.aileron_ids[0][2].jam, 1.0)]);
        let mut truth = parked_truth();
        for side in 0..2 {
            truth.commanded_surfaces.ailerons_deg[side] = [20.0; 3];
        }

        let published = run(&mut ids(), &truth, &faults, 3.0);

        assert!(
            published["FCTL_AIL_L1_DEFLECTION_DEG"].abs() < 1.0,
            "the jammed panel must stay where it seized, away from its non-neutral 20 deg command: {} deg",
            published["FCTL_AIL_L1_DEFLECTION_DEG"]
        );
        assert!(
            published["FCTL_AIL_L3_DEFLECTION_DEG"] > 18.0,
            "its unjammed neighbour must still follow the same non-neutral command: {} deg",
            published["FCTL_AIL_L3_DEFLECTION_DEG"]
        );
        assert_eq!(published["FCTL_AIL_L1_FAULT"], 1.0, "and the ECAM trigger variable must see it");
        assert_eq!(published["FCTL_AIL_L3_FAULT"], 0.0);
    }

    #[test]
    fn a_servo_hardover_drives_an_elevator_to_its_stop_with_nothing_commanded() {
        let live = ids();
        let faults = Faults::from_pairs([(live.elevator_ids[1][1].runaway, 1.0)]);
        let published = run(&mut ids(), &parked_truth(), &faults, 3.0);
        assert!(
            (published["FCTL_ELEV_R_OUTBD_DEFLECTION_DEG"] - AILERON_MAX_DEG).abs() < 0.5,
            "a full hardover must reach the stop: {} deg",
            published["FCTL_ELEV_R_OUTBD_DEFLECTION_DEG"]
        );
        assert!(
            published["FCTL_ELEV_L_OUTBD_DEFLECTION_DEG"].abs() < 0.5,
            "the other side must not move: {} deg",
            published["FCTL_ELEV_L_OUTBD_DEFLECTION_DEG"]
        );
    }

    #[test]
    fn losing_a_rudder_actuators_supply_lets_the_airload_carry_it_back_to_neutral() {
        let live = ids();
        let faults = Faults::from_pairs([(live.rudder_ids[0].supply_loss, 1.0)]);
        let mut truth = flying_truth();
        truth.commanded_surfaces.rudders_deg = [25.0, 25.0];

        let healthy_out = run(&mut ids(), &truth, &Faults::default(), 4.0);
        let starved_out = run(&mut ids(), &truth, &faults, 4.0);

        assert!(
            healthy_out["FCTL_RUD_UPPER_DEFLECTION_DEG"] > 15.0,
            "a healthy upper rudder holds most of its command at 24 kPa: {} deg",
            healthy_out["FCTL_RUD_UPPER_DEFLECTION_DEG"]
        );
        assert!(
            starved_out["FCTL_RUD_UPPER_DEFLECTION_DEG"] < 5.0,
            "with no supply the airload wins: {} deg",
            starved_out["FCTL_RUD_UPPER_DEFLECTION_DEG"]
        );
        assert!(
            starved_out["FCTL_RUD_LOWER_DEFLECTION_DEG"] > 15.0,
            "the lower rudder is a separate surface and must be unaffected: {} deg",
            starved_out["FCTL_RUD_LOWER_DEFLECTION_DEG"]
        );
    }

    #[test]
    fn a_disconnected_surface_free_floats_instead_of_holding_its_command() {
        let live = ids();
        let faults = Faults::from_pairs([(live.aileron_ids[1][0].disconnect, 1.0)]);
        let mut truth = flying_truth();
        truth.commanded_surfaces.ailerons_deg[1] = [20.0; 3];
        let published = run(&mut ids(), &truth, &faults, 4.0);
        assert!(
            published["FCTL_AIL_R3_DEFLECTION_DEG"].abs() < 3.0,
            "a sheared inward aileron weathervanes to neutral: {} deg",
            published["FCTL_AIL_R3_DEFLECTION_DEG"]
        );
        assert!(published["FCTL_AIL_R1_DEFLECTION_DEG"] > 10.0, "its neighbours still track");
    }

    #[test]
    fn a_free_floating_surface_settles_off_neutral_when_alpha_is_real() {
        let live = ids();
        let faults = Faults::from_pairs([(live.aileron_ids[1][0].disconnect, 1.0)]);

        let mut zero_alpha = flying_truth();
        zero_alpha.angle_of_attack_deg = 0.0;
        let without_alpha = run(&mut ids(), &zero_alpha, &faults, 6.0);
        assert!(
            without_alpha["FCTL_AIL_R3_DEFLECTION_DEG"].abs() < 1.0,
            "with zero alpha and nothing commanded, a free-floating surface settles near neutral: {} deg",
            without_alpha["FCTL_AIL_R3_DEFLECTION_DEG"]
        );

        let mut real_alpha = flying_truth();
        real_alpha.angle_of_attack_deg = 10.0;
        let with_alpha = run(&mut ids(), &real_alpha, &faults, 6.0);
        assert!(
            with_alpha["FCTL_AIL_R3_DEFLECTION_DEG"].abs() > 2.0,
            "a real 10 deg angle of attack must move the free-floating surface's equilibrium away from neutral through Ch_alpha: {} deg",
            with_alpha["FCTL_AIL_R3_DEFLECTION_DEG"]
        );
    }

    #[test]
    fn commanding_a_surface_publishes_a_nonzero_hydraulic_demand_that_settles_to_zero_at_rest() {
        let mut truth = parked_truth();
        let mut moving = ids();
        for _ in 0..5 {
            moving.tick(&truth, &Faults::default());
        }
        truth.commanded_surfaces.ailerons_deg[0] = [20.0; 3];
        moving.tick(&truth, &Faults::default());
        let mut published = BTreeMap::new();
        moving.publish(&mut |name, value| {
            published.insert(name.to_string(), value);
        });
        assert!(published["FCTL_GREEN_DEMAND_M3_S"] > 0.0, "a moving aileron must draw green flow: {}", published["FCTL_GREEN_DEMAND_M3_S"]);
        assert!(published["FCTL_YELLOW_DEMAND_M3_S"] > 0.0, "and its midboard panel draws yellow: {}", published["FCTL_YELLOW_DEMAND_M3_S"]);

        let rest = run(&mut ids(), &parked_truth(), &Faults::default(), 5.0);
        assert!(rest["FCTL_GREEN_DEMAND_M3_S"].abs() < 1e-9, "a settled surface with nothing commanded draws no flow: {}", rest["FCTL_GREEN_DEMAND_M3_S"]);
        assert!(rest["FCTL_YELLOW_DEMAND_M3_S"].abs() < 1e-9);
    }

    #[test]
    fn a_ths_ballscrew_jam_freezes_the_trim_where_it_seized() {
        let live = ids();
        let faults = Faults::from_pairs([(live.ths_ids.ballscrew_jam, 1.0)]);
        let mut truth = parked_truth();
        truth.commanded_surfaces.ths_deg = 8.0;

        let healthy_out = run(&mut ids(), &truth, &Faults::default(), 20.0);
        let jammed_out = run(&mut ids(), &truth, &faults, 20.0);

        assert!(healthy_out["FCTL_THS_DEFLECTION_DEG"] > 7.0, "a healthy THS reaches its trim: {} deg", healthy_out["FCTL_THS_DEFLECTION_DEG"]);
        assert!(
            jammed_out["FCTL_THS_DEFLECTION_DEG"] < 1.5,
            "a jammed ballscrew does not: {} deg",
            jammed_out["FCTL_THS_DEFLECTION_DEG"]
        );
        assert!(jammed_out["FCTL_THS_DEFLECTION_DEG"] < healthy_out["FCTL_THS_DEFLECTION_DEG"] * 0.25);
        assert_eq!(jammed_out["FCTL_THS_FAULT"], 1.0);
    }

    #[test]
    fn ground_spoiler_logic_that_fails_to_deploy_leaves_every_panel_down() {
        let live = ids();
        let faults = Faults::from_pairs([(live.ground_spoiler_ids[0], 1.0)]);
        let mut truth = parked_truth();
        truth.controls.ground_spoiler_lever_armed = true;

        let deployed = run(&mut ids(), &truth, &Faults::default(), 4.0);
        assert_eq!(deployed["FCTL_GND_SPLR_DEPLOYED"], 1.0);
        assert!(
            deployed["FCTL_SPLR_L4_DEFLECTION_DEG"] > 45.0,
            "armed and on the ground, an inboard panel comes right out: {} deg",
            deployed["FCTL_SPLR_L4_DEFLECTION_DEG"]
        );
        let outboard = deployed["FCTL_SPLR_L1_DEFLECTION_DEG"];
        assert!(
            outboard > SPOILER_1_2_MAX_DEG - 2.0 && outboard <= SPOILER_1_2_MAX_DEG + 1e-6,
            "panel 1 deploys, but stops at its own {SPOILER_1_2_MAX_DEG} deg travel: {outboard} deg"
        );

        let stowed = run(&mut ids(), &truth, &faults, 4.0);
        assert_eq!(stowed["FCTL_GND_SPLR_DEPLOYED"], 0.0);
        assert!(
            stowed["FCTL_SPLR_L1_DEFLECTION_DEG"] < 1.0,
            "with the logic failed they stay down: {} deg",
            stowed["FCTL_SPLR_L1_DEFLECTION_DEG"]
        );
        assert_eq!(stowed["FCTL_GND_SPLR_LOGIC_FAULT"], 1.0);
    }

    #[test]
    fn a_high_lift_shaft_break_costs_the_outboard_station_its_drive() {
        let live = ids();
        let faults = Faults::from_pairs([(live.high_lift_ids[0][0].outboard_shaft_break, 1.0)]);
        let mut truth = parked_truth();
        truth.flap_cmd_deg = 20.0;

        let mut broken = ids();
        let out = run(&mut broken, &truth, &faults, 12.0);
        assert!(out["FCTL_FLAP_L_INBOARD_DEG"] > 10.0, "the inboard station still drives: {} deg", out["FCTL_FLAP_L_INBOARD_DEG"]);
        assert!(
            out["FCTL_FLAP_L_OUTBOARD_DEG"] < out["FCTL_FLAP_L_INBOARD_DEG"] * 0.5,
            "the outboard station is left behind: {} vs {} deg",
            out["FCTL_FLAP_L_OUTBOARD_DEG"],
            out["FCTL_FLAP_L_INBOARD_DEG"]
        );
    }

    #[test]
    fn a_drifting_position_transducer_eventually_trips_the_dual_channel_monitor() {
        let live = ids();
        let faults = Faults::from_pairs([(live.rudder_ids[1].transducer_drift, 1.0)]);
        let truth = parked_truth();
        let early = run(&mut ids(), &truth, &faults, 1.0);
        assert_eq!(early["FCTL_RUD_LOWER_POSITION_MONITOR_FAULT"], 0.0, "a fresh drift has not diverged yet");
        let late = run(&mut ids(), &truth, &faults, 60.0);
        assert_eq!(late["FCTL_RUD_LOWER_POSITION_MONITOR_FAULT"], 1.0, "60 s of drift must trip the monitor");
        assert_eq!(late["FCTL_RUD_UPPER_POSITION_MONITOR_FAULT"], 0.0);
    }

    #[test]
    fn the_surface_angle_getter_matches_what_is_published() {
        let live = ids();
        let faults = Faults::from_pairs([(live.elevator_ids[0][0].runaway, 1.0)]);
        let mut area = ids();
        let published = run(&mut area, &parked_truth(), &faults, 2.0);
        let angles = area.surface_angles();
        assert_eq!(angles.elevators_deg[0][0], published["FCTL_ELEV_L_INBD_DEFLECTION_DEG"]);
        assert_eq!(angles.rudders_deg[0], published["FCTL_RUD_UPPER_DEFLECTION_DEG"]);
        assert_eq!(angles.spoilers_deg[1][7], published["FCTL_SPLR_R8_DEFLECTION_DEG"]);
        assert_eq!(angles.ths_deg, published["FCTL_THS_DEFLECTION_DEG"]);
        for side in 0..2 {
            for p in 0..3 {
                assert!((AILERON_MIN_DEG..=AILERON_MAX_DEG).contains(&angles.ailerons_deg[side][p]));
            }
            for i in 0..8 {
                assert!((0.0..=SPOILER_MAX_DEG).contains(&angles.spoilers_deg[side][i]));
            }
        }
        assert!(angles.rudders_deg.iter().all(|d| d.abs() <= RUDDER_LIMIT_DEG + 1e-9));
    }

    #[test]
    fn nothing_is_nan_on_the_very_first_frame_of_a_cold_aircraft() {
        let mut live = ids();
        live.tick(&Truth::default(), &Faults::default());
        let mut ok = true;
        live.publish(&mut |name, value| {
            if !value.is_finite() {
                println!("non-finite {name}");
                ok = false;
            }
        });
        assert!(ok);
    }

    #[test]
    fn the_area_plugs_into_deep_through_the_live_contract() {
        use crate::deep::live::Deep;
        let mut deep = Deep::new().with_area(super::live_system());
        assert_eq!(deep.area_names(), vec!["flight_controls"]);
        let mut published = BTreeMap::new();
        deep.tick(parked_truth(), &Faults::default(), &mut |name, value| {
            published.insert(name.to_string(), value);
        });
        assert!(published.contains_key("FCTL_AIL_L1_FAULT"));
        assert!(published.contains_key("FCTL_THS_DEFLECTION_DEG"));
    }

    #[test]
    fn prim1_and_sec1_down_moves_the_inboard_aileron_off_its_green_actuator() {
        let mut truth = parked_truth();
        for side in 0..2 {
            truth.commanded_surfaces.ailerons_deg[side][0] = 20.0;
        }

        let healthy = run(&mut ids(), &truth, &Faults::default(), 3.0);
        assert!(
            healthy["FCTL_AIL_L3_DEFLECTION_DEG"] > 18.0,
            "healthy inboard aileron should track its command: {} deg",
            healthy["FCTL_AIL_L3_DEFLECTION_DEG"]
        );
        assert!(healthy["FCTL_GREEN_DEMAND_M3_S"] > 0.0, "the Green actuator should be the one driving it while healthy");

        truth.prim_healthy[0] = false;
        truth.sec_healthy[0] = false;
        let failed = run(&mut ids(), &truth, &Faults::default(), 3.0);
        assert!(
            failed["FCTL_AIL_L3_DEFLECTION_DEG"] > 18.0,
            "the EHA actuator (PRIM2|SEC2) must still reach the command: {} deg",
            failed["FCTL_AIL_L3_DEFLECTION_DEG"]
        );
        assert_eq!(
            failed["FCTL_GREEN_DEMAND_M3_S"], 0.0,
            "with PRIM1 and SEC1 both down, the deep model must stop driving the Green actuator -- no counted Green flow can be left"
        );
    }

    #[test]
    fn losing_every_flight_control_computer_still_collapses_every_actuator_to_damping() {
        let mut truth = parked_truth();
        truth.prim_healthy = [false; 3];
        truth.sec_healthy = [false; 3];
        for side in 0..2 {
            truth.commanded_surfaces.ailerons_deg[side] = [20.0; 3];
        }
        truth.commanded_surfaces.rudders_deg = [25.0, 25.0];

        let out = run(&mut ids(), &truth, &Faults::default(), 3.0);
        for name in ["FCTL_AIL_L1_DEFLECTION_DEG", "FCTL_AIL_L2_DEFLECTION_DEG", "FCTL_AIL_L3_DEFLECTION_DEG", "FCTL_RUD_UPPER_DEFLECTION_DEG", "FCTL_RUD_LOWER_DEFLECTION_DEG"] {
            assert!(out[name].abs() < 1.0, "{name} should not move toward its command with no live computer at all: {} deg", out[name]);
        }
        assert_eq!(out["FCTL_GREEN_DEMAND_M3_S"], 0.0, "no computer left to drive any Green actuator");
        assert_eq!(out["FCTL_YELLOW_DEMAND_M3_S"], 0.0, "no computer left to drive any Yellow actuator");
    }

    #[test]
    fn a_healthy_captains_sidestick_reads_no_fault_and_both_channels_open_trips_it() {
        let mut truth = flying_truth();
        truth.capt_sidestick_pitch_raw = 0.3;
        let mut area = ids();
        let healthy = run(&mut area, &truth, &Faults::default(), 1.0);
        assert_eq!(healthy["FCTL_L_SIDESTICK_PITCH_FAULT"], 0.0, "a healthy captain's stick must not fault");
        assert_eq!(healthy["FCTL_L_SIDESTICK_PITCH_SENSOR_FAULT"], 0.0);

        let ids_ = area.l_sidestick_pitch_ids;
        let faults = Faults::from_pairs([(ids_.chan_a_open, 1.0), (ids_.chan_b_open, 1.0)]);
        let mut area = ids();
        let faulted = run(&mut area, &truth, &faults, 1.0);
        assert_eq!(faulted["FCTL_L_SIDESTICK_PITCH_FAULT"], 1.0, "both channels open must read as the stick lost outright");
    }

    #[test]
    fn one_drifting_sidestick_channel_trips_the_sensor_fault_not_the_plain_fault() {
        let mut truth = flying_truth();
        truth.capt_sidestick_roll_raw = 0.0;
        let mut area = ids();
        let ids_ = area.l_sidestick_roll_ids;
        let faults = Faults::from_pairs([(ids_.chan_a_drift, 1.0)]);
        let out = run(&mut area, &truth, &faults, 40.0);
        assert_eq!(out["FCTL_L_SIDESTICK_ROLL_FAULT"], 0.0, "one drifting channel is not the same as losing the stick");
        assert_eq!(out["FCTL_L_SIDESTICK_ROLL_SENSOR_FAULT"], 1.0, "a persistent channel disagreement must trip the sensor fault");
    }

    #[test]
    fn rudder_pedal_transducer_mirrors_the_sidestick_pattern() {
        let truth = flying_truth();
        let mut area = ids();
        let healthy = run(&mut area, &truth, &Faults::default(), 1.0);
        assert_eq!(healthy["FCTL_RUDDER_PEDAL_FAULT"], 0.0);
        assert_eq!(healthy["FCTL_RUDDER_PEDAL_SENSOR_FAULT"], 0.0);

        let ids_ = area.rudder_pedal_ids;
        let mut area = ids();
        let both_open = run(&mut area, &truth, &Faults::from_pairs([(ids_.chan_a_open, 1.0), (ids_.chan_b_open, 1.0)]), 1.0);
        assert_eq!(both_open["FCTL_RUDDER_PEDAL_FAULT"], 1.0);

        let mut area = ids();
        let disagreeing = run(&mut area, &truth, &Faults::from_pairs([(ids_.chan_a_drift, 1.0)]), 40.0);
        assert_eq!(disagreeing["FCTL_RUDDER_PEDAL_FAULT"], 0.0);
        assert_eq!(disagreeing["FCTL_RUDDER_PEDAL_SENSOR_FAULT"], 1.0);
    }

    #[test]
    fn sidestick_disabled_by_takeover_passes_truth_straight_through() {
        let healthy = run(&mut ids(), &flying_truth(), &Faults::default(), 0.1);
        assert_eq!(healthy["FCTL_L_SIDESTICK_DISABLED_BY_TAKEOVER"], 0.0);
        assert_eq!(healthy["FCTL_R_SIDESTICK_DISABLED_BY_TAKEOVER"], 0.0);

        let mut fo_pressed = flying_truth();
        fo_pressed.prim_left_sidestick_disabled = true;
        let out = run(&mut ids(), &fo_pressed, &Faults::default(), 0.1);
        assert_eq!(out["FCTL_L_SIDESTICK_DISABLED_BY_TAKEOVER"], 1.0, "271800001 CONFIG L SIDESTICK FAULT: the left stick reads disabled the same tick Truth says so");
        assert_eq!(out["FCTL_R_SIDESTICK_DISABLED_BY_TAKEOVER"], 0.0);

        let mut capt_pressed = flying_truth();
        capt_pressed.prim_right_sidestick_disabled = true;
        let out = run(&mut ids(), &capt_pressed, &Faults::default(), 0.1);
        assert_eq!(out["FCTL_R_SIDESTICK_DISABLED_BY_TAKEOVER"], 1.0, "271800002 CONFIG R SIDESTICK FAULT");
        assert_eq!(out["FCTL_L_SIDESTICK_DISABLED_BY_TAKEOVER"], 0.0);
    }

    #[test]
    fn prim_and_sec_pin_programming_disagree_reads_the_armed_fault_the_same_tick() {
        let truth = flying_truth();
        let healthy = run(&mut ids(), &truth, &Faults::default(), 0.1);
        assert_eq!(healthy["FCTL_PRIM_VERSIONS_DISAGREE"], 0.0);
        assert_eq!(healthy["FCTL_PRIM_PIN_PROG_DISAGREE"], 0.0);
        assert_eq!(healthy["FCTL_SEC_VERSIONS_DISAGREE"], 0.0);

        let prim_id = ids().prim_pin_prog_ids.mismatch;
        let out = run(&mut ids(), &truth, &Faults::from_pairs([(prim_id, 1.0)]), 0.02);
        assert_eq!(out["FCTL_PRIM_VERSIONS_DISAGREE"], 1.0, "271800045 F/CTL PRIM VERSIONS DISAGREE");
        assert_eq!(out["FCTL_PRIM_PIN_PROG_DISAGREE"], 1.0, "271800047 F/CTL PRIMs PIN PROG DISAGREE -- the same underlying check under FlyByWire's second title");
        assert_eq!(out["FCTL_SEC_VERSIONS_DISAGREE"], 0.0, "arming the PRIM mismatch must not also flag the SECs");

        let sec_id = ids().sec_pin_prog_ids.mismatch;
        let out = run(&mut ids(), &truth, &Faults::from_pairs([(sec_id, 1.0)]), 0.02);
        assert_eq!(out["FCTL_SEC_VERSIONS_DISAGREE"], 1.0, "271800046 F/CTL SEC VERSIONS DISAGREE");
        assert_eq!(out["FCTL_PRIM_VERSIONS_DISAGREE"], 0.0);
    }

    #[test]
    fn a_healthy_rate_gyro_reads_no_fault_and_both_channels_open_trips_it() {
        let mut truth = flying_truth();
        truth.body_rate_pitch_raw = 0.05;
        let mut area = ids();
        let healthy = run(&mut area, &truth, &Faults::default(), 1.0);
        assert_eq!(healthy["FCTL_RATE_GYRO_PITCH_FAULT"], 0.0, "271800018 must be quiet with a healthy gyro pair");

        let ids_ = area.rate_gyro_pitch_ids;
        let faulted = run(&mut ids(), &truth, &Faults::from_pairs([(ids_.chan_a_open, 1.0), (ids_.chan_b_open, 1.0)]), 1.0);
        assert_eq!(faulted["FCTL_RATE_GYRO_PITCH_FAULT"], 1.0, "both pitch-rate-gyro channels lost must read as 271800018's cause");
    }

    #[test]
    fn f_o_sidestick_fault_and_sensor_fault_mirror_the_captains_pattern_at_a_fixed_neutral() {
        let truth = flying_truth();
        let mut area = ids();
        let healthy = run(&mut area, &truth, &Faults::default(), 1.0);
        assert_eq!(healthy["FCTL_R_SIDESTICK_PITCH_FAULT"], 0.0, "271800026 must be quiet with a healthy F.O. stick");
        assert_eq!(healthy["FCTL_R_SIDESTICK_PITCH_SENSOR_FAULT"], 0.0, "271800028 must be quiet with a healthy F.O. stick");

        let ids_ = area.r_sidestick_pitch_ids;
        let both_open = run(&mut ids(), &truth, &Faults::from_pairs([(ids_.chan_a_open, 1.0), (ids_.chan_b_open, 1.0)]), 1.0);
        assert_eq!(both_open["FCTL_R_SIDESTICK_PITCH_FAULT"], 1.0, "both channels open must fault even though the stick never moves -- the FCOM's own trigger never requires deflection");

        let disagreeing = run(&mut ids(), &truth, &Faults::from_pairs([(ids_.chan_a_drift, 1.0)]), 40.0);
        assert_eq!(disagreeing["FCTL_R_SIDESTICK_PITCH_FAULT"], 0.0);
        assert_eq!(disagreeing["FCTL_R_SIDESTICK_PITCH_SENSOR_FAULT"], 1.0, "a channel drifting off a shared neutral still trips the disagreement monitor");
    }

    #[test]
    fn a_per_prim_channel_fault_only_reads_while_that_prim_is_itself_healthy() {
        let mut truth = flying_truth();
        truth.prim_healthy = [true, true, true];
        let elev1 = ids().prim_elevator_channel_ids[0];
        let rud2 = ids().prim_rudder_channel_ids[1];
        let stick3 = ids().prim_sidestick_monitor_ids[2];

        let healthy = run(&mut ids(), &truth, &Faults::default(), 0.1);
        assert_eq!(healthy["FCTL_PRIM_1_ELEVATOR_CHANNEL_FAULT"], 0.0);

        let armed = run(&mut ids(), &truth, &Faults::from_pairs([(elev1, 1.0), (rud2, 1.0), (stick3, 1.0)]), 0.1);
        assert_eq!(armed["FCTL_PRIM_1_ELEVATOR_CHANNEL_FAULT"], 1.0, "271800033 PRIM 1 ELEVATOR ACTUATOR FAULT");
        assert_eq!(armed["FCTL_PRIM_2_RUDDER_CHANNEL_FAULT"], 1.0, "271800040 PRIM 2 RUDDER ACTUATOR FAULT");
        assert_eq!(armed["FCTL_PRIM_3_SIDESTICK_MONITOR_FAULT"], 1.0, "271800044 PRIM 3 SIDESTICK SENSOR FAULT");
        assert_eq!(armed["FCTL_PRIM_2_ELEVATOR_CHANNEL_FAULT"], 0.0, "untouched PRIM 2's own elevator channel must stay quiet");

        truth.prim_healthy[0] = false;
        let dead_prim = run(&mut ids(), &truth, &Faults::from_pairs([(elev1, 1.0)]), 0.1);
        assert_eq!(dead_prim["FCTL_PRIM_1_ELEVATOR_CHANNEL_FAULT"], 0.0, "a channel fault on a wholly-dead PRIM must not double up with 271800036");
    }

    #[test]
    fn load_alleviation_needs_two_of_three_accelerometers_failed_in_one_wing() {
        let truth = flying_truth();
        let laf = ids().load_alleviation_ids;

        let healthy = run(&mut ids(), &truth, &Faults::default(), 0.1);
        assert_eq!(healthy["FCTL_LOAD_ALLEVIATION_FAULT"], 0.0);

        let one_failed = run(&mut ids(), &truth, &Faults::from_pairs([(laf.left[0], 1.0)]), 0.1);
        assert_eq!(one_failed["FCTL_LOAD_ALLEVIATION_FAULT"], 0.0, "one of three accelerometers failed is not yet a 2-of-3 vote");

        let two_failed_left = run(&mut ids(), &truth, &Faults::from_pairs([(laf.left[0], 1.0), (laf.left[1], 1.0)]), 0.1);
        assert_eq!(two_failed_left["FCTL_LOAD_ALLEVIATION_FAULT"], 1.0, "271800029: two of three left-wing LAF accelerometers failed");

        let two_failed_right = run(&mut ids(), &truth, &Faults::from_pairs([(laf.right[1], 1.0), (laf.right[2], 1.0)]), 0.1);
        assert_eq!(two_failed_right["FCTL_LOAD_ALLEVIATION_FAULT"], 1.0, "the vote is per wing -- the right wing's own 2-of-3 also faults it");
    }

    #[test]
    fn flap_lever_sys_fault_reads_per_sfcc_channel() {
        let truth = flying_truth();
        let csu = ids().flap_lever_csu_ids;

        let healthy = run(&mut ids(), &truth, &Faults::default(), 0.1);
        assert_eq!(healthy["FCTL_FLAPS_LEVER_SYS_1_FAULT"], 0.0);
        assert_eq!(healthy["FCTL_FLAPS_LEVER_SYS_2_FAULT"], 0.0);

        let sys1 = run(&mut ids(), &truth, &Faults::from_pairs([(csu.chan_1_comm_lost, 1.0)]), 0.1);
        assert_eq!(sys1["FCTL_FLAPS_LEVER_SYS_1_FAULT"], 1.0, "272800014 F/CTL FLAPS LEVER SYS 1 FAULT");
        assert_eq!(sys1["FCTL_FLAPS_LEVER_SYS_2_FAULT"], 0.0);

        let sys2 = run(&mut ids(), &truth, &Faults::from_pairs([(csu.chan_2_comm_lost, 1.0)]), 0.1);
        assert_eq!(sys2["FCTL_FLAPS_LEVER_SYS_2_FAULT"], 1.0, "272800015 F/CTL FLAPS LEVER SYS 2 FAULT");
        assert_eq!(sys2["FCTL_FLAPS_LEVER_SYS_1_FAULT"], 0.0);
    }
}
