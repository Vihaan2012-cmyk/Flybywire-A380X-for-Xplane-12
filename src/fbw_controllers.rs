//! FlyByWire's A380 FADEC computer, compiled from their generated C++.
//!
//! Source: `fbw-a380x/src/wasm/fbw_a380/src/model/A380FadecComputer.cpp`,
//! `A380FadecComputer_data.cpp` and `A380FadecComputer_types.h`, built
//! unchanged by `build.rs` with the C ABI shim in `src/fbw_cpp/shim.cpp`.
//! Nothing of the control law is re-derived here.
//!
//! This is the whole path from thrust lever angle to the simulator's throttle
//! in FlyByWire's A380X: `FlyByWireInterface::updateFadec` feeds one computer
//! per engine and writes `out.output.sim_throttle_lever_pos` to MSFS's
//! `GENERAL ENG THROTTLE LEVER POSITION:n`. The A380X has no separate
//! autothrust model: A/THR engagement, modes and the N1 command are computed
//! by the PRIM's flight guidance (`A380PrimComputerFg`) and sent to this
//! computer on the PRIM output buses (`prim_1..3.fg.ats_discrete_word`,
//! `fg.n1_command_percent`, `fg.flx_to_temp_deg_c`). With those buses silent
//! (all zero, SSM failure warning) the computer runs the manual path: N1 from
//! lever angle and the thrust limits.
//!
//! The structs below mirror the model's buses field for field. `boolean_T` is
//! `unsigned char` (`u8` here) and the `athr_thrust_limit_type` enum is an
//! `int32_T` (`i32` here, see [`ThrustLimitType`]). Sizes and offsets are
//! asserted at compile time on both sides.

#![allow(dead_code, non_snake_case, clippy::upper_case_acronyms)]

use std::mem::{align_of, offset_of, size_of};

/// ARINC 429 sign/status matrix values (`SignStatusMatrix`).
pub mod ssm {
    pub const FAILURE_WARNING: u32 = 0;
    pub const NO_COMPUTED_DATA: u32 = 1;
    pub const FUNCTIONAL_TEST: u32 = 2;
    pub const NORMAL_OPERATION: u32 = 3;
}

/// Bits of `fg.ats_discrete_word`, numbered as FlyByWire reads them (bit n is
/// `(word >> (n - 1)) & 1`; A380PrimComputerFctl.cpp packs element i of the
/// word at bit i + 11).
pub mod ats_bit {
    pub const ATHR_ENGAGED: u32 = 11;
    pub const ATHR_ACTIVE: u32 = 12;
    pub const ATHR_INOP: u32 = 13;
    pub const ATHR_LIMITED: u32 = 14;
    pub const SPEED_MACH_MODE: u32 = 18;
    pub const RETARD_MODE: u32 = 19;
    pub const THRUST_MODE: u32 = 20;
    pub const ALPHA_FLOOR: u32 = 24;
}

/// `athr_thrust_limit_type`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ThrustLimitType {
    None,
    Clb,
    Mct,
    Flex,
    Toga,
    Reverse,
}

impl ThrustLimitType {
    pub fn from_raw(raw: i32) -> Option<Self> {
        Some(match raw {
            0 => Self::None,
            1 => Self::Clb,
            2 => Self::Mct,
            3 => Self::Flex,
            4 => Self::Toga,
            5 => Self::Reverse,
            _ => return None,
        })
    }
}

impl BaseArinc429 {
    /// A word in normal operation carrying `data`.
    pub fn normal(data: f32) -> Self {
        Self { SSM: ssm::NORMAL_OPERATION, Data: data }
    }

    /// Bit `n` as FlyByWire's `BitFromLabel` reads it.
    pub fn bit(&self, n: u32) -> bool {
        let word = self.Data.round().clamp(0., u32::MAX as f32) as u32;
        (word >> (n - 1)) & 1 != 0
    }
}

/// `base_arinc_429` in A380FadecComputer_types.h.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
#[allow(non_snake_case)]
pub struct BaseArinc429 {
    pub SSM: u32,
    pub Data: f32,
}

/// `base_prim_fctl_out_bus` in A380FadecComputer_types.h.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
#[allow(non_snake_case)]
pub struct BasePrimFctlOutBus {
    pub left_inboard_aileron_command_deg: BaseArinc429,
    pub right_inboard_aileron_command_deg: BaseArinc429,
    pub left_midboard_aileron_command_deg: BaseArinc429,
    pub right_midboard_aileron_command_deg: BaseArinc429,
    pub left_outboard_aileron_command_deg: BaseArinc429,
    pub right_outboard_aileron_command_deg: BaseArinc429,
    pub left_spoiler_1_command_deg: BaseArinc429,
    pub right_spoiler_1_command_deg: BaseArinc429,
    pub left_spoiler_2_command_deg: BaseArinc429,
    pub right_spoiler_2_command_deg: BaseArinc429,
    pub left_spoiler_3_command_deg: BaseArinc429,
    pub right_spoiler_3_command_deg: BaseArinc429,
    pub left_spoiler_4_command_deg: BaseArinc429,
    pub right_spoiler_4_command_deg: BaseArinc429,
    pub left_spoiler_5_command_deg: BaseArinc429,
    pub right_spoiler_5_command_deg: BaseArinc429,
    pub left_spoiler_6_command_deg: BaseArinc429,
    pub right_spoiler_6_command_deg: BaseArinc429,
    pub left_spoiler_7_command_deg: BaseArinc429,
    pub right_spoiler_7_command_deg: BaseArinc429,
    pub left_spoiler_8_command_deg: BaseArinc429,
    pub right_spoiler_8_command_deg: BaseArinc429,
    pub left_inboard_elevator_command_deg: BaseArinc429,
    pub right_inboard_elevator_command_deg: BaseArinc429,
    pub left_outboard_elevator_command_deg: BaseArinc429,
    pub right_outboard_elevator_command_deg: BaseArinc429,
    pub ths_command_deg: BaseArinc429,
    pub upper_rudder_command_deg: BaseArinc429,
    pub lower_rudder_command_deg: BaseArinc429,
    pub left_sidestick_pitch_command_deg: BaseArinc429,
    pub right_sidestick_pitch_command_deg: BaseArinc429,
    pub left_sidestick_roll_command_deg: BaseArinc429,
    pub right_sidestick_roll_command_deg: BaseArinc429,
    pub rudder_pedal_position_deg: BaseArinc429,
    pub aileron_status_word: BaseArinc429,
    pub left_aileron_1_position_deg: BaseArinc429,
    pub left_aileron_2_position_deg: BaseArinc429,
    pub right_aileron_1_position_deg: BaseArinc429,
    pub right_aileron_2_position_deg: BaseArinc429,
    pub spoiler_status_word: BaseArinc429,
    pub left_spoiler_position_deg: BaseArinc429,
    pub right_spoiler_position_deg: BaseArinc429,
    pub elevator_status_word: BaseArinc429,
    pub elevator_1_position_deg: BaseArinc429,
    pub elevator_2_position_deg: BaseArinc429,
    pub elevator_3_position_deg: BaseArinc429,
    pub ths_position_deg: BaseArinc429,
    pub rudder_status_word: BaseArinc429,
    pub rudder_1_position_deg: BaseArinc429,
    pub rudder_2_position_deg: BaseArinc429,
    pub radio_height_1_ft: BaseArinc429,
    pub radio_height_2_ft: BaseArinc429,
    pub fctl_law_status_word: BaseArinc429,
    pub discrete_status_word_1: BaseArinc429,
    pub v_alpha_lim_kn: BaseArinc429,
    pub v_alpha_prot_kn: BaseArinc429,
    pub v_alpha_stall_warn_kn: BaseArinc429,
}

/// `base_prim_fe_out_bus` in A380FadecComputer_types.h.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
#[allow(non_snake_case)]
pub struct BasePrimFeOutBus {
    pub gamma_a_deg: BaseArinc429,
    pub gamma_t_deg: BaseArinc429,
    pub sideslip_target_deg: BaseArinc429,
    pub v_ls_kn: BaseArinc429,
    pub v_stall_kn: BaseArinc429,
    pub speed_trend_kn: BaseArinc429,
    pub v_3_kn: BaseArinc429,
    pub v_4_kn: BaseArinc429,
    pub v_man_kn: BaseArinc429,
    pub v_max_kn: BaseArinc429,
    pub v_fe_next_kn: BaseArinc429,
    pub discrete_word_1: BaseArinc429,
}

/// `base_prim_fg_out_bus` in A380FadecComputer_types.h.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
#[allow(non_snake_case)]
pub struct BasePrimFgOutBus {
    pub pfd_spd_tgt_kts: BaseArinc429,
    pub pfd_short_term_mngd_spd_kts: BaseArinc429,
    pub selected_spd_kts: BaseArinc429,
    pub selected_mach_kts: BaseArinc429,
    pub selected_hdg_deg: BaseArinc429,
    pub selected_trk_deg: BaseArinc429,
    pub selected_alt_ft: BaseArinc429,
    pub selected_vs_ft_min: BaseArinc429,
    pub selected_fpa_deg: BaseArinc429,
    pub runway_hdg_memorized_deg: BaseArinc429,
    pub preset_mach_from_fms: BaseArinc429,
    pub preset_speed_from_fms_kts: BaseArinc429,
    pub roll_fd_command_1: BaseArinc429,
    pub pitch_fd_command_1: BaseArinc429,
    pub yaw_fd_command_1: BaseArinc429,
    pub roll_fd_command_2: BaseArinc429,
    pub pitch_fd_command_2: BaseArinc429,
    pub yaw_fd_command_2: BaseArinc429,
    pub discrete_word_1: BaseArinc429,
    pub fm_alt_constraint_ft: BaseArinc429,
    pub ats_discrete_word: BaseArinc429,
    pub ats_fma_discrete_word: BaseArinc429,
    pub discrete_word_2: BaseArinc429,
    pub discrete_word_3: BaseArinc429,
    pub discrete_word_4: BaseArinc429,
    pub discrete_word_5: BaseArinc429,
    pub discrete_word_6: BaseArinc429,
    pub low_target_speed_margin_kts: BaseArinc429,
    pub high_target_speed_margin_kts: BaseArinc429,
    pub nosewheel_cmd_deg: BaseArinc429,
    pub n1_command_percent: BaseArinc429,
    pub flx_to_temp_deg_c: BaseArinc429,
    pub discrete_word_7: BaseArinc429,
}

/// `athr_data_computed` in A380FadecComputer_types.h.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
#[allow(non_snake_case)]
pub struct AthrDataComputed {
    pub TLA_in_active_range: u8,  // boolean_T (unsigned char)
    pub is_FLX_active: u8,  // boolean_T (unsigned char)
    pub ATHR_disabled: u8,  // boolean_T (unsigned char)
    pub time_since_touchdown: f64,
}

/// `base_eec` in A380FadecComputer_types.h.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
#[allow(non_snake_case)]
pub struct BaseEec {
    pub selected_tla_deg: BaseArinc429,
    pub n1_ref_percent: BaseArinc429,
    pub selected_flex_temp_deg: BaseArinc429,
    pub ecu_status_word_1: BaseArinc429,
    pub ecu_status_word_2: BaseArinc429,
    pub ecu_status_word_3: BaseArinc429,
    pub ecu_status_word_4: BaseArinc429,
    pub n1_limit_percent: BaseArinc429,
    pub n1_maximum_percent: BaseArinc429,
    pub n1_command_percent: BaseArinc429,
    pub selected_n2_actual_percent: BaseArinc429,
    pub selected_n1_actual_percent: BaseArinc429,
    pub ecu_maintenance_word_6: BaseArinc429,
}

/// `athr_time` in A380FadecComputer_types.h.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
#[allow(non_snake_case)]
pub struct AthrTime {
    pub dt: f64,
    pub simulation_time: f64,
}

/// `athr_data` in A380FadecComputer_types.h.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
#[allow(non_snake_case)]
pub struct AthrData {
    pub V_ias_kn: f64,
    pub V_tas_kn: f64,
    pub V_mach: f64,
    pub V_gnd_kn: f64,
    pub alpha_deg: f64,
    pub H_ft: f64,
    pub H_ind_ft: f64,
    pub H_radio_ft: f64,
    pub H_dot_fpm: f64,
    pub on_ground: u8,  // boolean_T (unsigned char)
    pub flap_handle_index: f64,
    pub is_engine_operative: u8,  // boolean_T (unsigned char)
    pub commanded_engine_N1_percent: f64,
    pub engine_N1_percent: f64,
    pub engine_N2_percent: f64,
    pub TAT_degC: f64,
    pub OAT_degC: f64,
    pub ISA_degC: f64,
    pub ambient_density_kg_per_m3: f64,
}

/// `athr_input` in A380FadecComputer_types.h.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
#[allow(non_snake_case)]
pub struct AthrInput {
    pub ATHR_disconnect: u8,  // boolean_T (unsigned char)
    pub TLA_deg: f64,
    pub thrust_limit_REV_percent: f64,
    pub thrust_limit_IDLE_percent: f64,
    pub thrust_limit_CLB_percent: f64,
    pub thrust_limit_MCT_percent: f64,
    pub thrust_limit_FLEX_percent: f64,
    pub thrust_limit_TOGA_percent: f64,
    pub is_anti_ice_active: u8,  // boolean_T (unsigned char)
    pub is_air_conditioning_active: u8,  // boolean_T (unsigned char)
    pub ATHR_reset_disable: u8,  // boolean_T (unsigned char)
    pub tracking_mode_on_override: u8,  // boolean_T (unsigned char)
}

/// `base_prim_out_bus` in A380FadecComputer_types.h.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
#[allow(non_snake_case)]
pub struct BasePrimOutBus {
    pub fctl: BasePrimFctlOutBus,
    pub fe: BasePrimFeOutBus,
    pub fg: BasePrimFgOutBus,
}

/// `athr_in` in A380FadecComputer_types.h.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
#[allow(non_snake_case)]
pub struct AthrIn {
    pub time: AthrTime,
    pub data: AthrData,
    pub input: AthrInput,
    pub prim_1: BasePrimOutBus,
    pub prim_2: BasePrimOutBus,
    pub prim_3: BasePrimOutBus,
}

/// `athr_output` in A380FadecComputer_types.h.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
#[allow(non_snake_case)]
pub struct AthrOutput {
    pub sim_throttle_lever_pos: f64,
    pub sim_thrust_mode: f64,
    pub N1_TLA_percent: f64,
    pub is_in_reverse: u8,  // boolean_T (unsigned char)
    pub thrust_limit_type: i32,  // athr_thrust_limit_type (enum class : int32_T)
    pub thrust_limit_percent: f64,
    pub N1_c_percent: f64,
    pub athr_control_active: u8,  // boolean_T (unsigned char)
    pub memo_thrust_active: u8,  // boolean_T (unsigned char)
}

/// `athr_out` in A380FadecComputer_types.h.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
#[allow(non_snake_case)]
pub struct AthrOut {
    pub time: AthrTime,
    pub data: AthrData,
    pub data_computed: AthrDataComputed,
    pub input: AthrInput,
    pub prim_input: BasePrimOutBus,
    pub output: AthrOutput,
    pub fadec_bus_output: BaseEec,
}

// Layout contract with src/fbw_cpp/shim.cpp (same numbers asserted there).
const _: () = {
    assert!(size_of::<BaseArinc429>() == 8 && align_of::<BaseArinc429>() == 4);
    assert!(size_of::<BasePrimFctlOutBus>() == 456);
    assert!(size_of::<BasePrimFeOutBus>() == 96);
    assert!(size_of::<BasePrimFgOutBus>() == 264);
    assert!(size_of::<BasePrimOutBus>() == 816);
    assert!(offset_of!(BasePrimOutBus, fe) == 456 && offset_of!(BasePrimOutBus, fg) == 552);
    assert!(offset_of!(BasePrimFgOutBus, ats_discrete_word) == 160);
    assert!(offset_of!(BasePrimFgOutBus, n1_command_percent) == 240);
    assert!(offset_of!(BasePrimFgOutBus, flx_to_temp_deg_c) == 248);
    assert!(offset_of!(BasePrimFctlOutBus, fctl_law_status_word) == 416);
    assert!(size_of::<AthrTime>() == 16);
    assert!(size_of::<AthrData>() == 152 && align_of::<AthrData>() == 8);
    assert!(offset_of!(AthrData, on_ground) == 72);
    assert!(offset_of!(AthrData, flap_handle_index) == 80);
    assert!(offset_of!(AthrData, is_engine_operative) == 88);
    assert!(offset_of!(AthrData, commanded_engine_N1_percent) == 96);
    assert!(offset_of!(AthrData, ambient_density_kg_per_m3) == 144);
    assert!(size_of::<AthrInput>() == 72);
    assert!(offset_of!(AthrInput, TLA_deg) == 8);
    assert!(offset_of!(AthrInput, is_anti_ice_active) == 64);
    assert!(offset_of!(AthrInput, tracking_mode_on_override) == 67);
    assert!(size_of::<AthrIn>() == 2688 && align_of::<AthrIn>() == 8);
    assert!(offset_of!(AthrIn, data) == 16 && offset_of!(AthrIn, input) == 168);
    assert!(offset_of!(AthrIn, prim_1) == 240 && offset_of!(AthrIn, prim_2) == 1056 && offset_of!(AthrIn, prim_3) == 1872);
    assert!(size_of::<AthrDataComputed>() == 16);
    assert!(size_of::<BaseEec>() == 104);
    assert!(size_of::<AthrOutput>() == 56);
    assert!(offset_of!(AthrOutput, is_in_reverse) == 24 && offset_of!(AthrOutput, thrust_limit_type) == 28);
    assert!(offset_of!(AthrOutput, thrust_limit_percent) == 32 && offset_of!(AthrOutput, N1_c_percent) == 40);
    assert!(offset_of!(AthrOutput, athr_control_active) == 48 && offset_of!(AthrOutput, memo_thrust_active) == 49);
    assert!(size_of::<AthrOut>() == 1232 && align_of::<AthrOut>() == 8);
    assert!(offset_of!(AthrOut, data_computed) == 168 && offset_of!(AthrOut, input) == 184);
    assert!(offset_of!(AthrOut, prim_input) == 256 && offset_of!(AthrOut, output) == 1072);
    assert!(offset_of!(AthrOut, fadec_bus_output) == 1128);
};

#[repr(C)]
struct RawFadec {
    _private: [u8; 0],
}

#[link(name = "fbw_controllers", kind = "static")]
extern "C" {
    fn fbw_fadec_create() -> *mut RawFadec;
    fn fbw_fadec_destroy(fadec: *mut RawFadec);
    fn fbw_fadec_set_inputs(fadec: *mut RawFadec, inputs: *const AthrIn);
    fn fbw_fadec_step(fadec: *mut RawFadec);
    fn fbw_fadec_get_outputs(fadec: *const RawFadec, outputs: *mut AthrOut);
    fn fbw_fadec_athr_in_size() -> usize;
    fn fbw_fadec_athr_out_size() -> usize;
}

/// One A380 FADEC computer (`A380FadecComputer`); FlyByWire runs four, one
/// per engine, each with its own state.
pub struct FadecModel {
    raw: *mut RawFadec,
}

// The model is plain data with no thread affinity.
unsafe impl Send for FadecModel {}

impl FadecModel {
    pub fn new() -> Self {
        // A second check of the layout against what the C++ compiler did.
        unsafe {
            assert_eq!(fbw_fadec_athr_in_size(), size_of::<AthrIn>());
            assert_eq!(fbw_fadec_athr_out_size(), size_of::<AthrOut>());
        }
        let raw = unsafe { fbw_fadec_create() };
        assert!(!raw.is_null(), "could not allocate the FADEC model");
        Self { raw }
    }

    /// Sets the inputs, steps the model once and returns its output bus, as
    /// `setExternalInputs`, `step` and `getExternalOutputs().out` do in
    /// FlyByWireInterface::updateFadec.
    pub fn step(&mut self, inputs: &AthrIn) -> AthrOut {
        let mut out = AthrOut::default();
        unsafe {
            fbw_fadec_set_inputs(self.raw, inputs);
            fbw_fadec_step(self.raw);
            fbw_fadec_get_outputs(self.raw, &mut out);
        }
        out
    }
}

impl Default for FadecModel {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for FadecModel {
    fn drop(&mut self) {
        unsafe { fbw_fadec_destroy(self.raw) };
    }
}
