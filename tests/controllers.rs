//! Runs FlyByWire's compiled A380 FADEC computer through the lever positions
//! FlyByWireInterface feeds it. Every expected value is worked out from
//! A380FadecComputer.cpp (step()) and the parameters in
//! A380FadecComputer_data.cpp; the line references are to those files.
//!
//! Lives outside the library so the module can be tested without declaring
//! it in lib.rs.

#[path = "../src/fbw_controllers.rs"]
mod fbw_controllers;
#[path = "../src/fbw_types.rs"]
mod fbw_types;
#[path = "../src/fbw_computers.rs"]
mod fbw_computers;

use fbw_controllers::*;

const IDLE: f64 = 19.0;
const CLB: f64 = 82.0;
const MCT: f64 = 88.0;
const FLEX: f64 = 85.0;
const TOGA: f64 = 90.0;
/// FlyByWireInterface.cpp:2923, with THRUST_LIMIT_REVERSE_PERCENTAGE_TOGA's
/// default of 0.813 (FlyByWireInterface.cpp:225).
const REV: f64 = TOGA * 0.813;
const DT: f64 = 0.05;

/// MSFS's lever position is clamped to this before it is written
/// (FlyByWireInterface.cpp:3010).
fn as_sent(position: f64) -> f64 {
    position.min(99.9999999999999)
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-9
}

/// A/THR off: the PRIM buses are all zero (SSM failure warning), so the
/// computer takes neither the A/THR N1 command nor the flex temperature.
fn inputs(tla: f64, on_ground: bool, ias: f64, engine_n1: f64, commanded_n1: f64) -> AthrIn {
    let mut i = AthrIn::default();
    i.time.dt = DT;
    i.data.V_ias_kn = ias;
    i.data.on_ground = on_ground as u8;
    i.data.is_engine_operative = 1;
    i.data.engine_N1_percent = engine_n1;
    i.data.commanded_engine_N1_percent = commanded_n1;
    i.data.TAT_degC = 15.;
    i.data.OAT_degC = 15.;
    i.input.TLA_deg = tla;
    i.input.thrust_limit_IDLE_percent = IDLE;
    i.input.thrust_limit_CLB_percent = CLB;
    i.input.thrust_limit_MCT_percent = MCT;
    i.input.thrust_limit_FLEX_percent = FLEX;
    i.input.thrust_limit_TOGA_percent = TOGA;
    i.input.thrust_limit_REV_percent = REV;
    i
}

fn run(model: &mut FadecModel, i: &mut AthrIn) -> AthrOut {
    let out = model.step(i);
    i.time.simulation_time += DT;
    out
}

#[test]
fn climb_detent_with_athr_off_commands_the_climb_limit() {
    let mut fadec = FadecModel::new();
    // Engine at the target, MSFS's commanded N1 two percent short of it.
    let mut i = inputs(25., false, 250., CLB, CLB - 2.);

    let out = run(&mut fadec, &mut i).output;
    // TLA 0..25 maps IDLE..CLB (cpp:1094-1098, 1127): 25 gives CLB.
    assert!(close(out.N1_TLA_percent, CLB));
    // Manual thrust: the command is the lever's N1 (cpp:1238).
    assert!(close(out.N1_c_percent, CLB));
    assert_eq!(out.sim_thrust_mode, 4.); // 25 <= TLA < 35 (cpp:1661)
    assert_eq!(ThrustLimitType::from_raw(out.thrust_limit_type), Some(ThrustLimitType::Clb)); // cpp:1151
    assert!(close(out.thrust_limit_percent, CLB));
    assert_eq!(out.is_in_reverse, 0);
    assert_eq!(out.athr_control_active, 0);
    // Forward lever integrator (cpp:1254-1281): gain 5 (Gain_Gain_d), the
    // N1 trim integrator stays 0 with the engine on target, so each step
    // adds 5 * (CLB - commanded) * dt = 5 * 2 * 0.05 = 0.5 percent.
    assert!(close(as_sent(out.sim_throttle_lever_pos), 0.5));
    let out = run(&mut fadec, &mut i).output;
    assert!(close(out.sim_throttle_lever_pos, 1.0));

    // Once MSFS's commanded N1 matches, the lever holds.
    i.data.commanded_engine_N1_percent = CLB;
    let out = run(&mut fadec, &mut i).output;
    assert!(close(out.sim_throttle_lever_pos, 1.0));
}

#[test]
fn idle_detent_gives_idle_and_never_a_negative_forward_lever() {
    let mut fadec = FadecModel::new();
    let mut i = inputs(0., false, 250., IDLE, IDLE + 3.);
    let out = run(&mut fadec, &mut i).output;
    assert!(close(out.N1_TLA_percent, IDLE));
    assert!(close(out.N1_c_percent, IDLE));
    assert_eq!(out.sim_thrust_mode, 2.); // TLA == 0 (cpp:1657)
    assert_eq!(out.is_in_reverse, 0);
    // 5 * (IDLE - (IDLE + 3)) * dt = -0.75, clamped at the lower limit 0
    // (DiscreteTimeIntegratorVariableTs_LowerLimit_d).
    assert_eq!(out.sim_throttle_lever_pos, 0.);
}

#[test]
fn toga_detent_gives_the_toga_limit_and_a_full_lever() {
    let mut fadec = FadecModel::new();
    let mut i = inputs(45., false, 250., TOGA, 0.);
    let mut out = run(&mut fadec, &mut i).output;
    // TLA 35..45 maps MCT (no flex temperature on the bus)..TOGA (cpp:1108-1117).
    assert!(close(out.N1_TLA_percent, TOGA));
    assert!(close(out.N1_c_percent, TOGA));
    assert_eq!(out.sim_thrust_mode, 6.); // TLA == 45 (cpp:1665)
    assert_eq!(ThrustLimitType::from_raw(out.thrust_limit_type), Some(ThrustLimitType::Toga));
    assert!(close(out.thrust_limit_percent, TOGA));
    // First step: 5 * 90 * 0.05 = 22.5.
    assert!(close(out.sim_throttle_lever_pos, 22.5));
    for _ in 0..4 {
        out = run(&mut fadec, &mut i).output;
    }
    // 112.5 clamped at the upper limit 100 (UpperLimit_l), then MSFS's cap.
    assert_eq!(out.sim_throttle_lever_pos, 100.);
    assert_eq!(as_sent(out.sim_throttle_lever_pos), 99.9999999999999);
}

#[test]
fn full_reverse_on_the_ground_commands_reverse_thrust() {
    // FlyByWire's throttle mapping gives engines 2 and 3 the reverse band;
    // the computer itself only sees the angle.
    let mut fadec = FadecModel::new();
    let mut i = inputs(-20., true, 100., REV, REV - 4.);
    let out = run(&mut fadec, &mut i).output;
    // TLA -6..-20 maps |IDLE + 1|..|REV| on |TLA| (cpp:1119-1125).
    assert!(close(out.N1_TLA_percent, REV));
    assert!(close(out.N1_c_percent, REV));
    assert_eq!(out.is_in_reverse, 1);
    assert_eq!(out.sim_thrust_mode, 1.); // TLA < 0 (cpp:1655)
    assert_eq!(ThrustLimitType::from_raw(out.thrust_limit_type), Some(ThrustLimitType::Reverse));
    assert!(close(out.thrust_limit_percent, REV));
    // Reverse lever integrator (cpp:1283-1295): gain -5 (Gain1_Gain), range
    // -100..0: -5 * (REV - (REV - 4)) * 0.05 = -1 percent per step.
    assert!(close(out.sim_throttle_lever_pos, -1.0));
    let out = run(&mut fadec, &mut i).output;
    assert!(close(out.sim_throttle_lever_pos, -2.0));

    // Reverse idle (-6) gives IDLE + 1.
    let mut fadec = FadecModel::new();
    let mut i = inputs(-6., true, 100., IDLE + 1., IDLE + 1.);
    let out = run(&mut fadec, &mut i).output;
    assert!(close(out.N1_TLA_percent, IDLE + 1.));
    assert_eq!(out.is_in_reverse, 1);
}

#[test]
fn reverse_lever_in_flight_is_idle_not_reverse() {
    // Airborne the angle is floored at 0 (cpp:1088-1090).
    let mut fadec = FadecModel::new();
    let mut i = inputs(-20., false, 250., IDLE, IDLE);
    let out = run(&mut fadec, &mut i).output;
    assert_eq!(out.is_in_reverse, 0);
    assert!(close(out.N1_TLA_percent, IDLE));
    assert!(out.sim_throttle_lever_pos >= 0.);
}

#[test]
fn low_speed_keeps_n1_out_of_the_62_5_to_73_5_band() {
    // cpp:1242-1252: below 60 kt a target inside (62.5, 73.5) snaps to an
    // edge; below 35 kt it is capped at 76.5. N1_c_percent is taken before.
    let mut fadec = FadecModel::new();
    // TLA 17.5 of 25 between IDLE 19 and CLB 82: 19 + 63 * 0.7 = 63.1.
    let n1 = IDLE + (CLB - IDLE) * 17.5 / 25.;
    let mut i = inputs(17.5, true, 20., 62.5, 62.5 - 1.);
    let out = run(&mut fadec, &mut i).output;
    assert!(close(out.N1_c_percent, n1));
    // Lever follows 62.5, not 63.1: 5 * (62.5 - 61.5) * 0.05 = 0.25.
    assert!(close(out.sim_throttle_lever_pos, 0.25));
}

#[test]
fn athr_engaged_on_the_prim_bus_takes_the_prim_n1_command() {
    let mut fadec = FadecModel::new();
    let mut i = inputs(25., false, 250., 70., 70.);
    // First step with the bus silent: the rising-edge detector starts
    // primed (MATLABFunction_f, cpp:82-96), so engagement needs an edge.
    let out = run(&mut fadec, &mut i).output;
    assert_eq!(out.athr_control_active, 0);

    // PRIM 1 and 2 not healthy, so the computer reads PRIM 3 (cpp:336-1068).
    let engaged_active = (1u32 << (ats_bit::ATHR_ENGAGED - 1)) | (1u32 << (ats_bit::ATHR_ACTIVE - 1));
    i.prim_3.fg.ats_discrete_word = BaseArinc429::normal(engaged_active as f32);
    i.prim_3.fg.n1_command_percent = BaseArinc429::normal(70.);
    let out = run(&mut fadec, &mut i).output;
    assert_eq!(out.athr_control_active, 1);
    // TLA in the active range, no alpha floor: the PRIM command limited to
    // IDLE..N1(TLA) = 19..82 (cpp:1219-1234).
    assert!(close(out.N1_c_percent, 70.));
    assert!(close(out.N1_TLA_percent, CLB));

    // A command above the lever is held to the lever.
    i.prim_3.fg.n1_command_percent = BaseArinc429::normal(86.);
    let out = run(&mut fadec, &mut i).output;
    assert!(close(out.N1_c_percent, CLB));

    // Instinctive disconnect drops A/THR back to the lever.
    i.input.ATHR_disconnect = 1;
    let out = run(&mut fadec, &mut i).output;
    assert_eq!(out.athr_control_active, 0);
}

#[test]
fn engines_keep_separate_state() {
    let mut a = FadecModel::new();
    let mut b = FadecModel::new();
    let mut ia = inputs(45., false, 250., TOGA, 0.);
    let mut ib = inputs(0., false, 250., IDLE, IDLE);
    for _ in 0..3 {
        run(&mut a, &mut ia);
        run(&mut b, &mut ib);
    }
    assert!(close(run(&mut a, &mut ia).output.sim_throttle_lever_pos, 90.));
    assert_eq!(run(&mut b, &mut ib).output.sim_throttle_lever_pos, 0.);
}

#[test]
fn smoke_prim_sec_fcu_step() {
    let mut p = fbw_computers::PrimComputer::new(0);
    let mut i = fbw_types::PrimInputs::default();
    i.time.dt = DT;
    p.set_inputs(&i);
    p.update(DT, 1.0, false, true);
    let _ = p.bus_outputs();
    let mut s = fbw_computers::SecComputer::new(0);
    s.update(DT, 1.0, false, true);
    let mut f = fbw_computers::FcuComputer::new();
    f.update(DT, 1.0, false, true);
}
