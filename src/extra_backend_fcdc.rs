//! FlyByWire's two FCDCs (flight control data concentrators) and the spoiler
//! lever LVars, as FlyByWireInterface runs them.
//!
//! Ported line for line from `fbw-a380x/src/wasm/fbw_a380/src`:
//! - `fcdc/Fcdc.cpp`, `Fcdc.h`, `FcdcIO.h`: the computer ([`Fcdc`]);
//! - `utils/TriggeredMonostableNode.cpp`: [`TriggeredMonostableNode`];
//! - `Arinc429Utils.cpp` / `Arinc429.cpp`: the helpers prim.rs does not
//!   already have (isFw, isNo, bitFromValue, setBit);
//! - `FlyByWireInterface::updateFcdc` (cpp:2219-2320): the inputs from the
//!   PRIM, SEC, RA, IR, ADR, SFCC and LGCIU buses prim.rs builds and from the
//!   LVars, and the `A32NX_FCDC_n_*` / `A32NX_BTV_LOST` LVars written back;
//! - `FlyByWireInterface::updateSpoilers` (cpp:3048-3063): `A32NX_SPOILERS_ARMED`
//!   and `A32NX_SPOILERS_HANDLE_POSITION`.
//!
//! In FlyByWireInterface::update the FCDCs run after the SECs and before the
//! FADECs (cpp:139-146), and the spoilers after the recording (cpp:151-155).
//! The plugin runs the FADECs inside engine_commands before
//! `Prims::update_after_fadecs`, so both run right after that; nothing in
//! between reads or writes what they use.
//!
//! The FCDC position and command LVars (`A32NX_FCDC_n_SPOILER_LEFT_1_POS`,
//! `_AILERON_LEFT_POS`, `_CAPT_ROLL_COMMAND`, `_PRIORITY_LIGHT_*`, ...) are
//! created at cpp:540-564 but nothing in FlyByWireInterface.cpp writes them,
//! so they are not written here either.
//!
//! The spoiler lever: FlyByWire's SpoilersHandler follows MSFS's spoiler key
//! events; here the lever is X-Plane's speedbrake handle, and the handler
//! state is [`SimReadings::spoilers_from_xplane`], the same readings the PRIMs
//! get (prim.rs update_prim, cpp:1585). The FCDCs take it too (cpp:2234, 2266).

use std::collections::HashMap;

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::fbw_types::*;
use crate::prim::{bit_or, from_simvar, to_simvar, value_or, Prims, SimReadings};
use crate::xp::{DataRef, Xplm};

/// Inputs this port has no source for, and what the FCDCs see instead
/// (documentation, as prim.rs's UNAVAILABLE).
#[allow(dead_code)]
pub const UNAVAILABLE: &[(&str, &str)] = &[(
    "SimData.simData (FcdcIO.h:73, cpp:2236)",
    "not ported: Fcdc.cpp never reads it",
)];

// Failures Fcdc1/Fcdc2 (27006/27007) and Rollout (22001) (FailuresConsumer,
// cpp:2227, 2249, 2299) are ported: failures.rs registers them
// (COMPUTER_FAILURES) and `update` (above) reads them from
// `failures::active_ids()`.

// ---------------------------------------------------------------------------
// Arinc429Utils (Arinc429Utils.cpp) and Arinc429DiscreteWord (Arinc429.cpp)
// beyond prim.rs's from_simvar, to_simvar, bit_or and value_or.
// ---------------------------------------------------------------------------

/// Arinc429SignStatus (Arinc429.h:5-10).
const SSM_FW: u32 = 0b00;
const SSM_NO: u32 = 0b11;

/// Arinc429Utils::isFw (Arinc429Utils.cpp:17-19).
fn is_fw(word: BaseArinc429) -> bool {
    word.SSM == SSM_FW
}

/// Arinc429Utils::isNo (Arinc429Utils.cpp:21-23).
fn is_no(word: BaseArinc429) -> bool {
    word.SSM == SSM_NO
}

/// Arinc429Utils::bitFromValue (Arinc429Utils.cpp:34-36).
fn bit_from_value(word: BaseArinc429, bit: u32) -> bool {
    (word.Data as u32 >> (bit - 1)) & 0x01 != 0
}

/// Arinc429DiscreteWord::setBit (Arinc429.cpp:99-101).
fn set_bit(word: &mut BaseArinc429, bit: u32, value: bool) {
    word.Data = ((word.Data as u32 & !(1 << (bit - 1))) | ((value as u32) << (bit - 1))) as f32;
}

/// Arinc429Word::setSsm (Arinc429.cpp:40-42).
fn set_ssm(word: &mut BaseArinc429, ssm: u32) {
    word.SSM = ssm;
}

// ---------------------------------------------------------------------------
// utils/TriggeredMonostableNode
// ---------------------------------------------------------------------------

/// TriggeredMonostableNode (TriggeredMonostableNode.h, .cpp).
#[derive(Clone, Copy, Debug)]
pub struct TriggeredMonostableNode {
    previous_value: bool,
    previous_output: bool,
    is_rising_edge: bool,
    retriggerable: bool,
    time_constant: f64,
    timer: f64,
}

impl TriggeredMonostableNode {
    /// TriggeredMonostableNode(timeDelay, isRisingEdge = true,
    /// retriggerable = false) (.h:5, .cpp:6-7).
    pub fn new(time_constant: f64) -> Self {
        Self::with(time_constant, true, false)
    }

    pub fn with(time_constant: f64, is_rising_edge: bool, retriggerable: bool) -> Self {
        Self {
            previous_value: !is_rising_edge,
            previous_output: false,
            is_rising_edge,
            retriggerable,
            time_constant,
            timer: 0.,
        }
    }

    /// .cpp:14-25
    pub fn write(&mut self, value: bool, delta_time: f64) -> bool {
        if self.timer > 0. {
            self.timer = (self.timer - delta_time).max(0.);
        }

        if (self.retriggerable || self.timer == 0.)
            && ((self.is_rising_edge && value && !self.previous_value)
                || (!self.is_rising_edge && !value && self.previous_value))
        {
            self.timer = self.time_constant;
        }

        self.previous_value = value;
        self.set_output(self.timer > 0.)
    }

    /// .cpp:27-29
    pub fn read(&self) -> bool {
        self.previous_output
    }

    /// .cpp:9-12
    fn set_output(&mut self, output: bool) -> bool {
        self.previous_output = output;
        output
    }
}

// ---------------------------------------------------------------------------
// fcdc/FcdcIO.h
// ---------------------------------------------------------------------------

/// FcdcBus (FcdcIO.h:6-27). `FcdcBus output = {}` leaves the words to
/// Arinc429DiscreteWord's empty constructor; they start at zero here.
#[derive(Clone, Copy, Debug, Default)]
pub struct FcdcBus {
    // F/CTL outputs
    /// Label 040
    pub efcs_status_1: BaseArinc429,
    /// Label 041
    pub efcs_status_2: BaseArinc429,
    /// Label 042
    pub efcs_status_3: BaseArinc429,
    /// Label 043
    pub efcs_status_4: BaseArinc429,
    /// Label 044
    pub efcs_status_5: BaseArinc429,

    // FG outputs
    pub fcdc_fg_discrete_word_1: BaseArinc429,
    pub fcdc_fg_discrete_word_2: BaseArinc429,
    pub fcdc_fg_discrete_word_3: BaseArinc429,

    pub landing_fct_discrete_word: BaseArinc429,
}

/// FcdcDiscreteInputs (FcdcIO.h:29-71). Some are set by updateFcdc but read
/// by nothing in Fcdc.cpp (primHealthy, nwsCommunicationAvailable,
/// abnProcImpactingLdgDistActive, autoBrakeMode); they are kept as FBW has
/// them.
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FcdcDiscreteInputs {
    pub spoilers_armed: bool,
    pub nose_gear_pressed: bool,
    pub prim_healthy: [bool; 3],
    pub other_fcdc_healthy: bool,
    pub btv_exit_missed: bool,
    // Some of these might be bus inputs, no refs though
    pub engine_operative: [bool; 4],
    pub apu_gen_connected: bool,
    pub every_dc_supplied_by_tr: bool,
    pub antiskid_available: bool,
    pub nws_communication_available: bool,
    pub yellow_hydraulic_available: bool,
    pub green_hydraulic_available: bool,
    pub abn_proc_impacting_ldg_perf_active: bool,
    pub abn_proc_impacting_ldg_dist_active: bool,
    pub oans_failed: bool,
    pub oans_ppos_lost: bool,
    pub dc_ess_failed: bool,
    pub dc2_failed: bool,
    pub ac2_failed: bool,
    pub auto_brake_active: bool,
    pub auto_brake_mode: i32,
    pub btv_state: i32,
}

/// FcdcAnalogInputs (FcdcIO.h:73-75).
#[derive(Clone, Copy, Debug, Default)]
pub struct FcdcAnalogInputs {
    pub spoilers_lever_pos: f64,
}

/// FcdcBusInputs (FcdcIO.h:77-86).
#[derive(Clone, Copy, Debug, Default)]
pub struct FcdcBusInputs {
    pub prims: [BasePrimOutBus; 3],
    pub secs: [BaseSecOutBus; 3],
    pub ra_bus_outputs: [BaseRaBus; 3],
    pub fws_discrete_word_126: [BaseArinc429; 2],
    pub ir_bus_outputs: [BaseIrBus; 3],
    pub adr_bus_outputs: [BaseAdrBus; 3],
    pub sfcc_bus_outputs: [BaseSfccBus; 2],
    pub lgciu_bus_outputs: [BaseLgciuBus; 2],
}

/// FcdcDiscreteOutputs (FcdcIO.h:88-102). The priority lights are always off
/// (Fcdc.cpp:204-207) and updateFcdc does not write them.
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FcdcDiscreteOutputs {
    pub capt_red_priority_light_on: bool,
    pub capt_green_priority_light_on: bool,
    pub fcdc_valid: bool,
    pub fo_red_priority_light_on: bool,
    pub fo_green_priority_light_on: bool,
    // This is architecturally not accurate, in the real thing this is done inside the PRIMs.
    // However, as BTV is implemented in Rust and the BTV INOP status is checked here, this would be good place to put it.
    pub btv_lost: bool,
}

// ---------------------------------------------------------------------------
// fcdc/Fcdc.h, Fcdc.cpp
// ---------------------------------------------------------------------------

/// Fcdc.h:7-11
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LateralLaw {
    NormalLaw,
    DirectLaw,
    None,
}

/// Fcdc.h:13-21
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PitchLaw {
    NormalLaw,
    AlternateLaw1A,
    AlternateLaw1B,
    AlternateLaw1C,
    AlternateLaw2,
    DirectLaw,
    None,
}

/// Fcdc (Fcdc.h:23-104). FlyByWire's FCDCs are members of a global
/// FlyByWireInterface (main.cpp:7), so the members its constructor leaves
/// alone start at zero.
#[derive(Clone, Debug)]
pub struct Fcdc {
    pub discrete_inputs: FcdcDiscreteInputs,
    pub analog_inputs: FcdcAnalogInputs,
    pub bus_inputs: FcdcBusInputs,

    // Computer monitoring and self-test vars
    monitoring_healthy: bool,
    power_supply_outage_time: f64,
    power_supply_fault: bool,
    self_test_timer: f64,
    self_test_complete: bool,
    /// Fcdc.h:68; not read by Fcdc.cpp.
    #[allow(dead_code)]
    is_unit_1: bool,

    master_prim_index: usize,
    all_prims_dead: bool,

    radio_alt: f64,

    last_btv_active: bool,
    last_btv_armed: bool,
    ldg_perf_affected_row_rop_lost: bool,
    ldg_perf_affected_btv_lost: bool,
    ldg_dist_affected_row_rop_lost: bool,
    ldg_dist_affected_btv_lost: bool,
    row_lost: bool,
    rop_lost: bool,
    btv_lost: bool,

    ldg_perf_affected_misc: bool,
    ldg_dist_affected_misc: bool,

    land2_capacity: bool,
    land3_fail_passive_capacity: bool,
    land3_fail_operational_capacity: bool,
    previous_land_capacity: i32,

    land2_inop: bool,
    land3_fail_passive_inop: bool,
    land3_fail_operational_inop: bool,

    /// Emit for 1s to make sure it reaches FWS (Fcdc.h:101)
    btv_triple_click_mtrig: TriggeredMonostableNode,
    capability_triple_click_mtrig: TriggeredMonostableNode,
    mode_reversion_triple_click_mtrig: TriggeredMonostableNode,
}

/// Fcdc.h:70
const MINIMUM_POWER_OUTAGE_TIME_FOR_FAILURE: f64 = 0.01;

impl Fcdc {
    /// Fcdc.cpp:7
    pub fn new(is_unit_1: bool) -> Self {
        Self {
            discrete_inputs: FcdcDiscreteInputs::default(),
            analog_inputs: FcdcAnalogInputs::default(),
            bus_inputs: FcdcBusInputs::default(),
            monitoring_healthy: false,
            power_supply_outage_time: 0.,
            power_supply_fault: false,
            self_test_timer: 0.,
            self_test_complete: false,
            is_unit_1,
            master_prim_index: 0,
            all_prims_dead: false,
            radio_alt: 0.,
            last_btv_active: false,
            last_btv_armed: false,
            ldg_perf_affected_row_rop_lost: false,
            ldg_perf_affected_btv_lost: false,
            ldg_dist_affected_row_rop_lost: false,
            ldg_dist_affected_btv_lost: false,
            row_lost: false,
            rop_lost: false,
            btv_lost: false,
            ldg_perf_affected_misc: false,
            ldg_dist_affected_misc: false,
            land2_capacity: false,
            land3_fail_passive_capacity: false,
            land3_fail_operational_capacity: false,
            previous_land_capacity: 0,
            land2_inop: false,
            land3_fail_passive_inop: false,
            land3_fail_operational_inop: false,
            btv_triple_click_mtrig: TriggeredMonostableNode::new(1.),
            capability_triple_click_mtrig: TriggeredMonostableNode::new(1.),
            mode_reversion_triple_click_mtrig: TriggeredMonostableNode::new(1.),
        }
    }

    // Perform the startup sequence, i.e.: Clear the memory, and initialize the self-test sequence.
    // If the power supply outage was lower than 3 seconds, or the aircraft is in the air or on ground an moving,
    // perform a short self-test.
    // Else, perform a long self-test.
    /// Fcdc.cpp:13-20
    fn startup(&mut self) {
        if self.power_supply_outage_time <= 3.0 || !self.discrete_inputs.nose_gear_pressed {
            self.self_test_timer = 0.5;
        } else {
            self.self_test_timer = 3.;
        }
        self.power_supply_outage_time = 0.0;
    }

    /// Fcdc.cpp:22-55
    pub fn update(&mut self, delta_time: f64, fault_active: bool, is_powered: bool) {
        self.monitor_power_supply(delta_time, is_powered);

        self.update_self_test(delta_time);
        self.monitor_self(fault_active);

        if self.monitoring_healthy {
            // Select master PRIM, use it for population of FCDC discrete words
            let prims = &self.bus_inputs.prims;
            self.all_prims_dead = false;
            if bit_or(prims[0].fctl.fctl_law_status_word, 21, false) {
                self.master_prim_index = 0;
            } else if bit_or(prims[1].fctl.fctl_law_status_word, 21, false) {
                self.master_prim_index = 1;
            } else if bit_or(prims[2].fctl.fctl_law_status_word, 21, false) {
                self.master_prim_index = 2;
            } else {
                self.all_prims_dead = true;
                self.master_prim_index = 0;
            }

            let ra = &self.bus_inputs.ra_bus_outputs;
            self.radio_alt = if is_no(ra[0].radio_height_ft) {
                ra[0].radio_height_ft.Data
            } else if is_no(ra[1].radio_height_ft) {
                ra[1].radio_height_ft.Data
            } else {
                ra[2].radio_height_ft.Data
            } as f64;

            self.update_approach_capability(delta_time);
            self.update_btv_row_rop(delta_time);

            let mode_reversion_request =
                bit_or(self.bus_inputs.prims[self.master_prim_index].fg.discrete_word_5, 28, false);

            self.mode_reversion_triple_click_mtrig.write(mode_reversion_request, delta_time);
        } else {
            self.previous_land_capacity = 0;
        }
    }

    /// Write the bus output data and return it (Fcdc.cpp:57-199).
    pub fn get_bus_outputs(&self) -> FcdcBus {
        let mut output = FcdcBus::default();

        if !self.monitoring_healthy {
            set_ssm(&mut output.efcs_status_1, SSM_FW);
            set_ssm(&mut output.efcs_status_2, SSM_FW);
            set_ssm(&mut output.efcs_status_3, SSM_FW);
            set_ssm(&mut output.efcs_status_4, SSM_FW);
            set_ssm(&mut output.efcs_status_5, SSM_FW);
            set_ssm(&mut output.fcdc_fg_discrete_word_1, SSM_FW);
            set_ssm(&mut output.fcdc_fg_discrete_word_2, SSM_FW);
            set_ssm(&mut output.fcdc_fg_discrete_word_3, SSM_FW);
            return output;
        }

        // Phase 1 of refactoring: Populate FCDC discrete words as per a32nx spec, disregarding the obvious differences.
        // Target: Should behave unsuspiciously in normal ops
        let ssm = SSM_NO;
        let fctl = &self.bus_inputs.prims[self.master_prim_index].fctl;

        let system_lateral_law = if self.all_prims_dead {
            LateralLaw::DirectLaw
        } else {
            Self::get_lateral_law_status_from_bits(
                bit_from_value(fctl.fctl_law_status_word, 19),
                bit_from_value(fctl.fctl_law_status_word, 20),
            )
        };

        let system_pitch_law = if self.all_prims_dead {
            PitchLaw::DirectLaw
        } else {
            Self::get_pitch_law_status_from_bits(
                bit_from_value(fctl.fctl_law_status_word, 16),
                bit_from_value(fctl.fctl_law_status_word, 17),
                bit_from_value(fctl.fctl_law_status_word, 18),
            )
        };

        // cpp:88-105
        let w = &mut output.efcs_status_1;
        set_ssm(w, ssm);
        set_bit(w, 11, system_pitch_law == PitchLaw::NormalLaw);
        set_bit(
            w,
            12,
            system_pitch_law == PitchLaw::AlternateLaw1A
                || system_pitch_law == PitchLaw::AlternateLaw1B
                || system_pitch_law == PitchLaw::AlternateLaw1C,
        );
        set_bit(w, 13, system_pitch_law == PitchLaw::AlternateLaw2);
        set_bit(w, 14, system_pitch_law == PitchLaw::AlternateLaw1A);
        set_bit(w, 15, system_pitch_law == PitchLaw::DirectLaw);
        set_bit(w, 16, system_lateral_law == LateralLaw::NormalLaw);
        set_bit(w, 17, system_lateral_law == LateralLaw::DirectLaw);
        for b in [19, 20, 21, 22, 23, 24, 25, 26, 29] {
            set_bit(&mut output.efcs_status_3, b, self.all_prims_dead);
        }

        let ail = |b: u32| bit_or(fctl.aileron_status_word, b, false);
        let elev = |b: u32| bit_or(fctl.elevator_status_word, b, false);

        // cpp:107-115
        let w = &mut output.efcs_status_2;
        set_ssm(w, ssm);
        set_bit(w, 11, !ail(11));
        set_bit(w, 12, !ail(11));
        set_bit(w, 13, !ail(14));
        set_bit(w, 14, !ail(14));
        set_bit(w, 15, !elev(11));
        set_bit(w, 16, !elev(11));
        set_bit(w, 17, !elev(14));
        set_bit(w, 18, !elev(14));

        // cpp:117-130
        let w = &mut output.efcs_status_3;
        set_ssm(w, ssm);
        set_bit(w, 11, ail(11));
        set_bit(w, 12, ail(11));
        set_bit(w, 13, ail(14));
        set_bit(w, 14, ail(14));
        set_bit(w, 15, elev(11));
        set_bit(w, 16, elev(11));
        set_bit(w, 17, elev(14));
        set_bit(w, 18, elev(14));
        for b in 21..=25 {
            set_bit(w, b, bit_or(fctl.spoiler_status_word, 11, false));
        }

        // FIXME inaccurate atm, improve
        // cpp:133-153
        let left_extended = value_or(fctl.left_spoiler_position_deg, 0.) < -2.5;
        let right_extended = value_or(fctl.right_spoiler_position_deg, 0.) < -2.5;
        let w = &mut output.efcs_status_4;
        set_ssm(w, ssm);
        for b in [11, 13, 15, 17, 19] {
            set_bit(w, b, left_extended);
        }
        for b in [12, 14, 16, 18, 20] {
            set_bit(w, b, right_extended);
        }
        let spoiler_valid = is_no(fctl.left_spoiler_position_deg) && is_no(fctl.right_spoiler_position_deg);
        for b in 21..=25 {
            set_bit(w, b, spoiler_valid);
        }
        set_bit(w, 26, value_or(fctl.left_spoiler_position_deg, 0.) < -5.);
        set_bit(w, 27, self.discrete_inputs.spoilers_armed);
        set_bit(w, 28, self.analog_inputs.spoilers_lever_pos > 0.9);

        // cpp:155-160
        let w = &mut output.efcs_status_5;
        w.Data = 0.;
        set_ssm(w, ssm);
        let spoilers_retracted = value_or(fctl.left_spoiler_position_deg, 0.0) >= -2.5
            && value_or(fctl.right_spoiler_position_deg, 0.0) >= -2.5;
        set_bit(w, 26, (self.analog_inputs.spoilers_lever_pos > 0.05) && spoilers_retracted);

        // cpp:162-165
        let w = &mut output.fcdc_fg_discrete_word_1;
        set_ssm(w, ssm);
        set_bit(w, 24, self.land2_capacity);
        set_bit(w, 25, self.land3_fail_passive_capacity);
        set_bit(w, 26, self.land3_fail_operational_capacity);

        // cpp:167-175
        let w = &mut output.fcdc_fg_discrete_word_2;
        set_ssm(w, ssm);
        for b in 11..=15 {
            set_bit(w, b, false);
        }
        set_bit(w, 24, self.land2_inop);
        set_bit(w, 25, self.land3_fail_passive_inop);
        set_bit(w, 26, self.land3_fail_operational_inop);

        // cpp:177-185
        let w = &mut output.fcdc_fg_discrete_word_3;
        set_ssm(w, ssm);
        for b in 11..=15 {
            set_bit(w, b, false);
        }
        set_bit(
            w,
            16,
            self.mode_reversion_triple_click_mtrig.read() || self.capability_triple_click_mtrig.read(),
        );
        set_bit(w, 17, self.btv_triple_click_mtrig.read());
        set_bit(w, 18, false);

        // cpp:187-196
        let w = &mut output.landing_fct_discrete_word;
        set_ssm(w, ssm);
        set_bit(w, 11, self.row_lost); // ROW LOST
        set_bit(w, 12, self.rop_lost); // ROP LOST
        set_bit(w, 13, self.btv_lost); // BTV LOST
        set_bit(w, 20, self.ldg_dist_affected_row_rop_lost); // LDG DIST AFFECTED LEADING TO ROW LOST
        set_bit(w, 21, self.ldg_perf_affected_row_rop_lost); // LDG PERF AFFECTED LEADING TO ROW LOST
        set_bit(w, 22, self.ldg_dist_affected_btv_lost); // LDG DIST AFFECTED LEADING TO BTV LOST
        set_bit(w, 23, self.ldg_perf_affected_btv_lost); // LDG PERF AFFECTED LEADING TO BTV LOST
        set_bit(w, 24, self.ldg_dist_affected_misc); // LDG DIST AFFECTED
        set_bit(w, 25, self.ldg_perf_affected_misc); // LDG PERF AFFECTED

        output
    }

    /// Fcdc.cpp:201-219
    pub fn get_discrete_outputs(&self) -> FcdcDiscreteOutputs {
        let mut output = FcdcDiscreteOutputs {
            capt_red_priority_light_on: false,
            capt_green_priority_light_on: false,
            fo_red_priority_light_on: false,
            fo_green_priority_light_on: false,
            fcdc_valid: self.monitoring_healthy,
            btv_lost: false,
        };

        if !self.monitoring_healthy {
            output.btv_lost = false;
            return output;
        }

        output.btv_lost = self.btv_lost;

        output
    }

    /// Fcdc.cpp:221-279
    fn update_approach_capability(&mut self, delta_time: f64) {
        // Calculate and set approach capacity
        // Each PRIM computes the approach capability it is able to provide. For LAND 3 Fail Op., PRIM 1 and 3 or 2 and 3 must be able to provide
        // LAND 3 Fail Op. The FCDC additionally checks for peripheral status that is not included in the PRIM computation, such as PFD, FWS,
        // FCDC Opp, etc, and the AP and A/THR engagement status.
        let prims = &self.bus_inputs.prims;
        let fg = &prims[self.master_prim_index].fg;
        let d = &self.discrete_inputs;

        let prim_land2_capability = bit_or(fg.discrete_word_1, 27, false);

        let prim_land3_fail_passive_capability = bit_or(fg.discrete_word_1, 28, false);

        let prim_land3_fail_operational_capability = (bit_or(prims[0].fg.discrete_word_1, 29, false)
            && bit_or(prims[1].fg.discrete_word_1, 29, false))
            || (bit_or(prims[0].fg.discrete_word_1, 29, false) && bit_or(prims[2].fg.discrete_word_1, 29, false));

        let one_ap_engaged = bit_or(fg.discrete_word_1, 11, false) || bit_or(fg.discrete_word_1, 12, false);
        let both_ap_engaged = bit_or(fg.discrete_word_1, 11, false) && bit_or(fg.discrete_word_1, 12, false);
        let athr_engaged = bit_or(fg.ats_discrete_word, 11, false);

        let land_mode_armed_or_engaged = bit_or(fg.discrete_word_1, 23, false) || bit_or(fg.discrete_word_2, 28, false);

        let fws_audio_function_available = bit_or(self.bus_inputs.fws_discrete_word_126[0], 16, false) as i32
            + bit_or(self.bus_inputs.fws_discrete_word_126[1], 16, false) as i32;
        let num_engines_operative = d.engine_operative.iter().filter(|&&e| e).count() as i32;

        let one_engine_on_each_side = (d.engine_operative[0] || d.engine_operative[1])
            && (d.engine_operative[2] || d.engine_operative[3]);
        let land3_fail_operational_engine_criteria =
            num_engines_operative == 4 || (num_engines_operative == 3 && d.apu_gen_connected);

        let land2_capability = prim_land2_capability && fws_audio_function_available > 0 && one_engine_on_each_side;
        let land3_fail_passive_capability =
            land2_capability && prim_land3_fail_passive_capability && num_engines_operative >= 3;
        let land3_fail_operational_capability = prim_land3_fail_operational_capability
            && fws_audio_function_available >= 2
            && d.other_fcdc_healthy
            && d.every_dc_supplied_by_tr
            && d.antiskid_available
            && land3_fail_operational_engine_criteria;

        let memorize_land3_capability = self.radio_alt < 200. && one_ap_engaged && land_mode_armed_or_engaged;
        let north_ref_true = bit_or(prims[0].fg.discrete_word_5, 13, false);

        self.land3_fail_operational_capacity = (self.land3_fail_operational_capacity && memorize_land3_capability)
            || (land3_fail_operational_capability
                && both_ap_engaged
                && athr_engaged
                && land_mode_armed_or_engaged
                && !north_ref_true);
        self.land3_fail_passive_capacity = (self.land3_fail_passive_capacity && memorize_land3_capability)
            || (land3_fail_passive_capability
                && one_ap_engaged
                && athr_engaged
                && land_mode_armed_or_engaged
                && !north_ref_true
                && !self.land3_fail_operational_capacity);
        self.land2_capacity = land2_capability
            && one_ap_engaged
            && land_mode_armed_or_engaged
            && !north_ref_true
            && !self.land3_fail_passive_capacity
            && !self.land3_fail_operational_capacity;

        self.land2_inop = !land2_capability;
        self.land3_fail_passive_inop = !land3_fail_passive_capability;
        self.land3_fail_operational_inop = !land3_fail_operational_capability;

        let new_land_capacity = if self.land3_fail_operational_capacity {
            5
        } else if self.land3_fail_passive_capacity {
            4
        } else if self.land2_capacity {
            3
        } else {
            0
        };
        self.capability_triple_click_mtrig
            .write(new_land_capacity < self.previous_land_capacity, delta_time);
        self.previous_land_capacity = new_land_capacity;
    }

    /// Fcdc.cpp:281-370
    fn update_btv_row_rop(&mut self, delta_time: f64) {
        // Populate BTV data
        self.btv_triple_click_mtrig.write(self.discrete_inputs.btv_exit_missed, delta_time);

        // BTV reversion triple click
        // On ground, if BTV is active and then deactivates --> triple click
        // In flight below 700ft RA, if BTV was armed and then was disarmed --> triple click
        let bus = self.bus_inputs;
        let d = self.discrete_inputs;
        let lgciu1_discrete_word_2 = bus.lgciu_bus_outputs[0].discrete_word_2;
        let lgciu2_discrete_word_2 = bus.lgciu_bus_outputs[1].discrete_word_2;
        let on_ground = bit_or(lgciu1_discrete_word_2, 11, false) || bit_or(lgciu2_discrete_word_2, 11, false);
        let btv_active = d.auto_brake_active && (d.btv_state == 2 || d.btv_state == 3 || d.btv_state == 4);
        let btv_armed = !d.auto_brake_active && d.btv_state == 1;
        if on_ground && !btv_active && self.last_btv_active {
            self.btv_triple_click_mtrig.write(true, delta_time);
        } else if !on_ground && self.radio_alt < 700. && !btv_armed && self.last_btv_armed {
            self.btv_triple_click_mtrig.write(true, delta_time);
        }
        self.last_btv_active = btv_active;
        self.last_btv_armed = btv_armed;

        // Check PRIM and SEC availability
        let mut prim_available = 0;
        let mut sec_available = 0;
        let mut ir_available = 0;
        let mut adr_available = 0;
        let mut ra_available = 0;
        let mut fws_audio_function_available = 0;

        for i in 0..3 {
            if is_no(bus.prims[i].fctl.fctl_law_status_word) {
                prim_available += 1;
            }
            if is_no(bus.secs[i].fctl_law_status_word) {
                sec_available += 1;
            }
            if is_no(bus.ir_bus_outputs[i].latitude_deg) {
                ir_available += 1;
            }
            if !is_fw(bus.adr_bus_outputs[i].aoa_corrected_deg) {
                adr_available += 1;
            }
            if !is_fw(bus.ra_bus_outputs[i].radio_height_ft) {
                ra_available += 1;
            }
        }

        for i in 0..2 {
            if bit_or(bus.fws_discrete_word_126[i], 16, false) {
                fws_audio_function_available += 1;
            }
        }

        // LDG PERF AFFECTED leading to ROP/ROW LOST
        self.ldg_perf_affected_row_rop_lost = d.abn_proc_impacting_ldg_perf_active;
        self.ldg_dist_affected_row_rop_lost = !d.yellow_hydraulic_available
            || !d.green_hydraulic_available
            || d.dc_ess_failed
            || d.dc2_failed
            || d.ac2_failed;

        // LDG PERF AFFECTED leading to BTV LOST
        let elev_status_word = bus.prims[self.master_prim_index].fctl.elevator_status_word;
        let double_elev_fault = (bit_or(elev_status_word, 11, false) as i32)
            + (bit_or(elev_status_word, 14, false) as i32)
            + (bit_or(elev_status_word, 17, false) as i32)
            < 2;
        // Fcdc.cpp:346 ("bool anyAileronFault = false; // FIXME add") never
        // wires this upstream either, but the PRIM's own aileron status word
        // is already read here for the FCDC bus writer (efcsStatus2/3,
        // cpp:108-121; write_bus_outputs's `ail(11)`/`ail(14)`, same bit
        // convention as elev_status_word above: 1 = that section available).
        // BTV landing-performance monitoring only has 2 monitored aileron
        // sections in this word (vs. the elevator's 3), so any one of them
        // being unavailable is already the fault this feeds.
        let aileron_status_word = bus.prims[self.master_prim_index].fctl.aileron_status_word;
        let any_aileron_fault =
            !bit_or(aileron_status_word, 11, false) || !bit_or(aileron_status_word, 14, false);
        let sfcc1_status_word = bus.sfcc_bus_outputs[0].slat_flap_system_status_word;
        let sfcc2_status_word = bus.sfcc_bus_outputs[1].slat_flap_system_status_word;
        let all_slats_fault = bit_or(sfcc1_status_word, 11, false) && bit_or(sfcc2_status_word, 11, false);
        let all_flaps_fault = bit_or(sfcc1_status_word, 12, false) && bit_or(sfcc2_status_word, 12, false);
        let slats_locked = bit_or(sfcc1_status_word, 15, false) || bit_or(sfcc2_status_word, 15, false);
        self.ldg_perf_affected_btv_lost = self.ldg_perf_affected_row_rop_lost
            || prim_available < 3
            || all_slats_fault
            || all_flaps_fault
            || slats_locked
            || double_elev_fault;
        self.ldg_dist_affected_btv_lost = sec_available < 3
            || self.ldg_dist_affected_row_rop_lost
            || !d.engine_operative[1]
            || !d.engine_operative[2]
            || any_aileron_fault;

        // common conditions for ROW/ROP and BTV lost
        let common_conditions =
            ir_available < 2 || adr_available < 2 || ra_available < 1 || fws_audio_function_available == 0;

        self.row_lost =
            common_conditions || self.ldg_perf_affected_row_rop_lost || self.ldg_dist_affected_row_rop_lost || d.oans_failed;
        self.rop_lost = common_conditions
            || self.ldg_perf_affected_row_rop_lost
            || self.ldg_dist_affected_row_rop_lost
            || d.oans_failed
            || d.oans_ppos_lost;
        self.btv_lost = common_conditions
            || self.ldg_perf_affected_btv_lost
            || self.ldg_dist_affected_btv_lost
            || d.oans_failed
            || d.oans_ppos_lost;

        // Misc. LDG DIST/LDG PERF effects
        self.ldg_dist_affected_misc = !d.antiskid_available;
    }

    /// Perform self monitoring (Fcdc.cpp:372-379)
    fn monitor_self(&mut self, fault_active: bool) {
        self.monitoring_healthy = !(fault_active || self.power_supply_fault || !self.self_test_complete);
    }

    // Monitor the power supply and record the outage time (used for self test and healthy logic).
    // If an outage lasts more than 10ms, stop the program execution.
    // If the power has been restored after an outage that lasted longer than 10ms, reset the RAM and
    // perform the startup sequence.
    /// Fcdc.cpp:381-396
    fn monitor_power_supply(&mut self, delta_time: f64, is_powered: bool) {
        if !is_powered {
            self.power_supply_outage_time += delta_time;
        }
        if self.power_supply_outage_time > MINIMUM_POWER_OUTAGE_TIME_FOR_FAILURE {
            self.power_supply_fault = true;
        }
        if is_powered && self.power_supply_fault {
            self.power_supply_fault = false;
            self.startup();
        }
    }

    /// Update the Self-test-Sequence (Fcdc.cpp:398-408)
    fn update_self_test(&mut self, delta_time: f64) {
        if self.self_test_timer > 0. {
            self.self_test_timer -= delta_time;
        }
        self.self_test_complete = self.self_test_timer <= 0.;
    }

    /// Fcdc.cpp:410-426
    fn get_pitch_law_status_from_bits(bit1: bool, bit2: bool, bit3: bool) -> PitchLaw {
        match (bit1, bit2, bit3) {
            (false, false, true) => PitchLaw::NormalLaw,
            (false, true, false) => PitchLaw::AlternateLaw1A,
            (false, true, true) => PitchLaw::AlternateLaw1B,
            (true, false, false) => PitchLaw::AlternateLaw1C,
            (true, false, true) => PitchLaw::AlternateLaw2,
            (true, true, false) => PitchLaw::DirectLaw,
            _ => PitchLaw::None,
        }
    }

    /// Fcdc.cpp:428-436
    fn get_lateral_law_status_from_bits(bit1: bool, bit2: bool) -> LateralLaw {
        if bit1 {
            LateralLaw::NormalLaw
        } else if bit2 {
            LateralLaw::DirectLaw
        } else {
            LateralLaw::None
        }
    }
}

// ---------------------------------------------------------------------------
// FlyByWireInterface::updateFcdc and updateSpoilers
// ---------------------------------------------------------------------------

/// Failure ids (FailureList.h:4, 13-14).
const FAILURE_ROLLOUT: u64 = 22001;
const FAILURE_FCDC: [u64; 2] = [27006, 27007];

/// Identifiers by full LVar name, as prim.rs's `Names`: `A32NX_` names go
/// through the prefixed lookup, the rest (simulator variables) unprefixed.
#[derive(Default)]
struct Names {
    ids: HashMap<String, VariableIdentifier>,
}

impl Names {
    fn id<V: VariableRegistry>(&mut self, vars: &mut V, name: &str) -> VariableIdentifier {
        if let Some(id) = self.ids.get(name) {
            return *id;
        }
        let id = match name.strip_prefix("A32NX_") {
            Some(bare) => vars.get(bare.to_owned()),
            None => vars.get_unprefixed(name.to_owned()),
        };
        self.ids.insert(name.to_owned(), id);
        id
    }

    fn get<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V, name: &str) -> f64 {
        let id = self.id(vars, name);
        vars.read(&id)
    }

    fn is<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V, name: &str) -> bool {
        self.get(vars, name) != 0.
    }

    fn set<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V, name: &str, value: f64) {
        let id = self.id(vars, name);
        vars.write(&id, value);
    }
}

/// What updateFcdc takes from MSFS's SimData rather than from variables.
#[derive(Clone, Copy, Debug, Default)]
pub struct FcdcSimData {
    /// ENG COMBUSTION:1..4 (SimConnectInterface.cpp:239-242, cpp:2238-2241),
    /// from X-Plane's `sim/flightmodel/engine/ENGN_running` as
    /// engine_commands.rs takes it for is_engine_operative.
    pub engine_combustion: [bool; 4],
}

/// The two FCDCs with FlyByWireInterface's state around them
/// (FlyByWireInterface.h:142-144).
pub struct ExtraBackendFcdc {
    names: Names,
    fcdcs: [Fcdc; 2],
    fcdcs_discrete_outputs: [FcdcDiscreteOutputs; 2],
    fcdcs_bus_outputs: [FcdcBus; 2],
    engines_running: Option<DataRef>,
}

impl ExtraBackendFcdc {
    pub fn new(xplm: &Xplm) -> Self {
        let mut me = Self::without_xplane();
        me.engines_running = xplm.find("sim/flightmodel/engine/ENGN_running");
        me
    }

    fn without_xplane() -> Self {
        Self {
            names: Names::default(),
            fcdcs: [Fcdc::new(true), Fcdc::new(false)],
            fcdcs_discrete_outputs: Default::default(),
            fcdcs_bus_outputs: Default::default(),
            engines_running: None,
        }
    }

    /// updateFcdc x2 (cpp:141-143), then updateSpoilers (cpp:154).
    pub fn update<V: VariableRegistry + SimulatorReaderWriter>(
        &mut self,
        vars: &mut V,
        xplm: &Xplm,
        prims: &Prims,
        readings: &SimReadings,
        dt: f64,
    ) {
        let mut running = [0i32; 4];
        if let Some(d) = self.engines_running {
            xplm.get_vi(d, &mut running);
        }
        let sim = FcdcSimData { engine_combustion: running.map(|r| r != 0) };
        self.update_with(vars, &sim, prims, readings, dt, &crate::failures::active_ids());
    }

    /// [`Self::update`] with the X-Plane readings and the active failure ids
    /// given.
    pub fn update_with<V: VariableRegistry + SimulatorReaderWriter>(
        &mut self,
        vars: &mut V,
        sim: &FcdcSimData,
        prims: &Prims,
        readings: &SimReadings,
        dt: f64,
        active_failures: &[u64],
    ) {
        // calculatedSampleTime (cpp:1035); the active pause check (cpp:2221)
        // is the plugin not ticking while paused.
        let dt = dt.max(0.002);
        for i in 0..2 {
            self.update_fcdc(vars, sim, prims, readings, dt, active_failures, i);
        }
        self.update_spoilers(vars, readings);
    }

    // `A32NX_CPIOM_C1_AVAIL`/`_C2_AVAIL`, `A32NX_AFDX_SWITCH_{3,4,13,14}_AVAIL`
    // and every `A32NX_AFDX_<node>_<dest>_REACHABLE` this port's FWS/FCDC
    // code reads below used to be guessed here from a handful of bus-power
    // LVars. That guess assumed `FlyByWireInterface.cpp`/the TS systems were
    // the only source of truth and, since neither writes these vars, that
    // nothing in this port ever would either.
    //
    // That premise was wrong: this plugin links `a380_systems` directly
    // (Cargo.toml) and runs its real `Simulation<A380>` every tick
    // (`self.simulation.tick(...)`, lib.rs, right after `self.fcdc.update()`)
    // -- in-process (`remote::Systems::Local`) or out-of-process through the
    // shared-memory wire protocol (`remote::Systems::Remote`,
    // src/remote/wire.rs), which mirrors every
    // registered variable (`remote.variable_count() > 1000` in
    // `remote::tests`) with no name filtering (checked wire.rs/server.rs/
    // client.rs). `A380`'s own `adcn: A380AvionicsDataCommunicationNetwork`
    // field (a380_systems/src/lib.rs) is `update()`d and `accept()`ed into
    // the simulation tree every tick, and each `AvionicsFullDuplexSwitch`/
    // `CoreProcessingInputOutputModule` (fbw-common's
    // integrated_modular_avionics) writes its own real `*_AVAIL` from its
    // `ElectricalBusType`'s actual powered state (patched into this port by
    // patches/fbw-rust/electrical.patch), and the ADCN's own breadth-first
    // routing table -- which also respects `AFDX_SWITCH_<id>_FAILURE`,
    // unlike the old guess -- writes every `AFDX_<a>_<b>_REACHABLE` pair
    // this file reads (verified against avionics_data_communication_
    // network.rs's routing-table construction for switches 1-9/11-19).
    //
    // Both `self.fcdc.update()` and `self.simulation.tick()` share the same
    // `self.vars` registry, so once the guess above is gone the real values
    // simply flow: `update_fcdc` below reads last tick's real ADCN output
    // (one tick of lag, like any other feedback path in this plugin), and
    // this tick's `self.simulation.tick()` -- called right after -- publishes
    // the real value FCDC will see next tick.
    #[allow(clippy::too_many_arguments)]
    fn update_fcdc<V: VariableRegistry + SimulatorReaderWriter>(
        &mut self,
        vars: &mut V,
        sim: &FcdcSimData,
        prims: &Prims,
        readings: &SimReadings,
        sample_time: f64,
        active_failures: &[u64],
        fcdc_index: usize,
    ) {
        let n = &mut self.names;
        let failure_active = active_failures.contains(&FAILURE_FCDC[fcdc_index]);

        // cpp:2229-2230
        let afdx_comm_available = if fcdc_index == 0 {
            n.get(vars, "A32NX_AFDX_SWITCH_3_AVAIL") == 1. || n.get(vars, "A32NX_AFDX_SWITCH_13_AVAIL") == 1.
        } else {
            n.get(vars, "A32NX_AFDX_SWITCH_4_AVAIL") == 1. || n.get(vars, "A32NX_AFDX_SWITCH_14_AVAIL") == 1.
        };

        let other_fcdc_valid = self.fcdcs_discrete_outputs[1 - fcdc_index].fcdc_valid;
        let fcdc = &mut self.fcdcs[fcdc_index];

        if afdx_comm_available {
            // cpp:2233-2266
            let d = &mut fcdc.discrete_inputs;
            // FlyByWire's own landing_gear/mod.rs:384 registers this
            // WITHOUT an A32NX_ prefix (`LGCIU_{n}_NOSE_GEAR_COMPRESSED`,
            // verified against `contains_variable_with_name` in that
            // module's own test at line 1789); the prefixed name this used
            // to read is never written by anything, so `nose_gear_pressed`
            // was permanently stuck false.
            d.nose_gear_pressed = n.is(vars, "LGCIU_1_NOSE_GEAR_COMPRESSED");
            d.spoilers_armed = readings.spoilers_armed;
            d.btv_exit_missed = n.is(vars, "A32NX_BTV_EXIT_MISSED");
            d.other_fcdc_healthy = other_fcdc_valid;
            d.engine_operative = sim.engine_combustion;
            d.apu_gen_connected = n.get(vars, "A32NX_ELEC_CONTACTOR_990XS1_IS_CLOSED") == 1.
                || n.get(vars, "A32NX_ELEC_CONTACTOR_990XS2_IS_CLOSED") == 1.;
            d.every_dc_supplied_by_tr = n.get(vars, "A32NX_ELEC_CONTACTOR_990PU1_IS_CLOSED") == 1.
                && n.get(vars, "A32NX_ELEC_CONTACTOR_990PU2_IS_CLOSED") == 1.
                && n.get(vars, "A32NX_ELEC_CONTACTOR_6PE_IS_CLOSED") == 1.
                && n.get(vars, "A32NX_ELEC_CONTACTOR_7PU_IS_CLOSED") == 1.;
            d.antiskid_available = n.is(vars, "ANTISKID BRAKES ACTIVE");
            d.yellow_hydraulic_available = n.is(vars, "A32NX_HYD_YELLOW_SYSTEM_1_SECTION_PRESSURE_SWITCH");
            let green_hydraulic_available = n.is(vars, "A32NX_HYD_GREEN_SYSTEM_1_SECTION_PRESSURE_SWITCH");
            d.green_hydraulic_available = green_hydraulic_available;
            // Nose wheel steering (handling.rs's steering system) is
            // actuated by the Green hydraulic system, same as the pressure
            // switch just read; a rollout-failure injection or the loss of
            // that hydraulic supply both take NWS communication down.
            d.nws_communication_available = !active_failures.contains(&FAILURE_ROLLOUT) && green_hydraulic_available;
            d.abn_proc_impacting_ldg_perf_active =
                n.is(vars, "A32NX_FWC_1_ABN_PROC_IMPACT_LDG_PERF") || n.is(vars, "A32NX_FWC_2_ABN_PROC_IMPACT_LDG_PERF");
            d.abn_proc_impacting_ldg_dist_active =
                n.is(vars, "A32NX_FWC_1_ABN_PROC_IMPACT_LDG_DIST") || n.is(vars, "A32NX_FWC_2_ABN_PROC_IMPACT_LDG_DIST");
            d.oans_failed = n.is(vars, "A32NX_OANS_FAILED");
            d.oans_ppos_lost = n.is(vars, "A32NX_ARPT_NAV_POS_LOST");
            d.dc_ess_failed = !n.is(vars, "A32NX_ELEC_108PH_BUS_IS_POWERED");
            d.dc2_failed = !n.is(vars, "A32NX_ELEC_DC_2_BUS_IS_POWERED");
            d.ac2_failed = !n.is(vars, "A32NX_ELEC_AC_2_BUS_IS_POWERED");
            d.auto_brake_active = n.get(vars, "A32NX_AUTOBRAKES_ACTIVE") == 1.;
            d.auto_brake_mode = n.get(vars, "A32NX_AUTOBRAKES_ARMED_MODE") as i32;
            d.btv_state = n.get(vars, "A32NX_BTV_STATE") as i32;

            // Fcdc.cpp's own speedBrakeLeverPos has no source on the PRIM
            // out bus; the real analog command comes straight from the
            // speedbrake lever instead (SpoilersHandler, cpp:1585 via
            // SimReadings::spoilers_handle_position), which is what feeds
            // efcs_status_4/5's speedbrake bits below.
            fcdc.analog_inputs.spoilers_lever_pos = readings.spoilers_handle_position;
        }

        // cpp:2269-2274
        let mut reachable = |a: &str, b: &str| n.get(vars, a) == 1. || n.get(vars, b) == 1.;
        let prim_sec_reachable = if fcdc_index == 0 {
            [
                reachable("A32NX_AFDX_1_3_REACHABLE", "A32NX_AFDX_11_13_REACHABLE"),
                reachable("A32NX_AFDX_2_3_REACHABLE", "A32NX_AFDX_12_13_REACHABLE"),
                reachable("A32NX_AFDX_9_3_REACHABLE", "A32NX_AFDX_19_13_REACHABLE"),
            ]
        } else {
            [
                reachable("A32NX_AFDX_1_4_REACHABLE", "A32NX_AFDX_11_14_REACHABLE"),
                reachable("A32NX_AFDX_2_4_REACHABLE", "A32NX_AFDX_12_14_REACHABLE"),
                reachable("A32NX_AFDX_9_4_REACHABLE", "A32NX_AFDX_19_14_REACHABLE"),
            ]
        };

        // cpp:2276-2297
        let (prim_discrete, prim_buses, sec_buses) =
            (prims.prim_discrete_outputs(), prims.prim_buses(), prims.sec_buses());
        let (ra, ir, adr) = (prims.ra_buses(), prims.ir_buses(), prims.adr_buses());
        let (sfcc, lgciu) = (prims.sfcc_buses(), prims.lgciu_buses());
        for i in 0..3 {
            fcdc.discrete_inputs.prim_healthy[i] = prim_sec_reachable[i] && prim_discrete[i].prim_healthy != 0;

            if prim_sec_reachable[i] {
                fcdc.bus_inputs.prims[i] = prim_buses[i];
                fcdc.bus_inputs.secs[i] = sec_buses[i];
            }

            if afdx_comm_available {
                fcdc.bus_inputs.ra_bus_outputs[i] = ra[i];
                fcdc.bus_inputs.ir_bus_outputs[i] = ir[i];
                fcdc.bus_inputs.adr_bus_outputs[i] = adr[i];
            }
        }

        for i in 0..2 {
            if afdx_comm_available {
                fcdc.bus_inputs.fws_discrete_word_126[i] =
                    from_simvar(n.get(vars, &format!("A32NX_FWC_{}_DISCRETE_WORD_126", i + 1)));
                fcdc.bus_inputs.sfcc_bus_outputs[i] = sfcc[i];
                fcdc.bus_inputs.lgciu_bus_outputs[i] = lgciu[i];
            }
        }

        // cpp:2299
        let cpiom_available = n.is(vars, &format!("A32NX_CPIOM_C{}_AVAIL", fcdc_index + 1));
        fcdc.update(sample_time, failure_active, cpiom_available);

        // cpp:2301-2302
        self.fcdcs_discrete_outputs[fcdc_index] = fcdc.get_discrete_outputs();
        self.fcdcs_bus_outputs[fcdc_index] = fcdc.get_bus_outputs();

        // cpp:2304-2317
        let bus = self.fcdcs_bus_outputs[fcdc_index];
        let p = format!("A32NX_FCDC_{}_", fcdc_index + 1);
        for (name, word) in [
            ("DISCRETE_WORD_1", bus.efcs_status_1),
            ("DISCRETE_WORD_2", bus.efcs_status_2),
            ("DISCRETE_WORD_3", bus.efcs_status_3),
            ("DISCRETE_WORD_4", bus.efcs_status_4),
            ("DISCRETE_WORD_5", bus.efcs_status_5),
            ("FG_DISCRETE_WORD_1", bus.fcdc_fg_discrete_word_1),
            ("FG_DISCRETE_WORD_2", bus.fcdc_fg_discrete_word_2),
            ("FG_DISCRETE_WORD_3", bus.fcdc_fg_discrete_word_3),
            ("LANDING_FCT_DISCRETE_WORD", bus.landing_fct_discrete_word),
        ] {
            n.set(vars, &format!("{p}{name}"), to_simvar(word));
        }

        n.set(vars, &format!("{p}HEALTHY"), if self.fcdcs_discrete_outputs[fcdc_index].fcdc_valid { 1. } else { 0. });
        // The capability-downgrade/mode-reversion/BTV triple-clicks (the
        // FCDC's own autoland-related aurals, Fcdc.cpp:183-184) are already
        // on FG_DISCRETE_WORD_3 bits 16-17 above, gated by the FWS audio
        // function (fws_audio_function_available, used in
        // update_landing_capability); a standalone "AUTOLAND" red warning is
        // a Flight Warning System alert with no FCDC bus word of its own in
        // Fcdc.cpp, so it belongs in FwsCore.ts, not here.
        let btv_lost = self.fcdcs_discrete_outputs[0].btv_lost || self.fcdcs_discrete_outputs[1].btv_lost;
        n.set(vars, "A32NX_BTV_LOST", if btv_lost { 1. } else { 0. });
    }

    /// updateSpoilers (cpp:3048-3063). The handler's initialisation from
    /// A32NX_SPOILERS_ARMED and SPOILERS HANDLE POSITION (cpp:3053-3055) has
    /// no counterpart: X-Plane's handle is the state from the first tick.
    fn update_spoilers<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V, readings: &SimReadings) {
        // set 3D handle position
        self.names.set(vars, "A32NX_SPOILERS_ARMED", if readings.spoilers_armed { 1. } else { 0. });
        self.names.set(vars, "A32NX_SPOILERS_HANDLE_POSITION", readings.spoilers_handle_position);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::prim::tests::{MapVars, Rig, DT};

    /// What the real IMA (AFDX switches, CPIOMs, routing table --
    /// `a380_systems`'s `A380AvionicsDataCommunicationNetwork`, linked into
    /// this plugin and ticked every frame by `self.simulation.tick()` in
    /// lib.rs) writes with everything available. This narrower `Rig`
    /// harness exercises `ExtraBackendFcdc::update_with` alone and does not
    /// run that simulation tick, so it sets the AVAIL/REACHABLE vars
    /// `update_fcdc` reads directly, standing in for the real ADCN's output.
    fn network_up(vars: &mut MapVars) {
        vars.set("A32NX_ELEC_DC_ESS_BUS_IS_POWERED", 1.);
        vars.set("A32NX_ELEC_AC_ESS_BUS_IS_POWERED", 1.);
        vars.set("A32NX_ELEC_DC_2_BUS_IS_POWERED", 1.);
        vars.set("A32NX_ELEC_AC_2_BUS_IS_POWERED", 1.);
        vars.set("ANTISKID BRAKES ACTIVE", 1.);
        vars.set("A32NX_AFDX_SWITCH_3_AVAIL", 1.);
        vars.set("A32NX_AFDX_SWITCH_13_AVAIL", 1.);
        vars.set("A32NX_AFDX_SWITCH_4_AVAIL", 1.);
        vars.set("A32NX_AFDX_SWITCH_14_AVAIL", 1.);
        for (a, b) in [(1, 3), (2, 3), (9, 3), (11, 13), (12, 13), (19, 13), (1, 4), (2, 4), (9, 4), (11, 14), (12, 14), (19, 14)] {
            vars.set(&format!("A32NX_AFDX_{a}_{b}_REACHABLE"), 1.);
        }
    }

    fn word(vars: &mut MapVars, name: &str) -> BaseArinc429 {
        from_simvar(vars.value(name))
    }

    fn step(rig: &mut Rig, fcdc: &mut ExtraBackendFcdc, sim: &FcdcSimData, readings: &SimReadings, ticks: usize) {
        for _ in 0..ticks {
            rig.tick(&[]);
            fcdc.update_with(&mut rig.vars, sim, &rig.prims, readings, DT, &[]);
        }
    }

    #[test]
    fn fcdcs_come_up_with_the_prims_and_carry_the_master_law() {
        let mut rig = Rig::new(true, 0., 0.);
        network_up(&mut rig.vars);
        let mut fcdc = ExtraBackendFcdc::without_xplane();
        let sim = FcdcSimData { engine_combustion: [true; 4] };
        step(&mut rig, &mut fcdc, &sim, &SimReadings::default(), 40);

        let law = rig.prims.prim_buses()[0].fctl.fctl_law_status_word;
        // PRIM 1 is master (bit 21) and in normal law on the ground: pitch
        // bits 16-18 = 0 0 1, lateral bit 19 (Fcdc.cpp:410-436).
        assert!(bit_or(law, 21, false));
        assert_eq!(
            (bit_from_value(law, 16), bit_from_value(law, 17), bit_from_value(law, 18), bit_from_value(law, 19)),
            (false, false, true, true)
        );
        for n in 1..=2 {
            assert_eq!(rig.vars.value(&format!("A32NX_FCDC_{n}_HEALTHY")), 1., "FCDC {n}");
            let w1 = word(&mut rig.vars, &format!("A32NX_FCDC_{n}_DISCRETE_WORD_1"));
            assert_eq!(w1.SSM, SSM_NO);
            // cpp:89-96: pitch normal (11) and lateral normal (16) only.
            assert_eq!(w1.Data as u32, 1 << 10 | 1 << 15, "FCDC {n} word 1 {:#x}", w1.Data as u32);
            for name in [
                "DISCRETE_WORD_2", "DISCRETE_WORD_3", "DISCRETE_WORD_4", "DISCRETE_WORD_5", "FG_DISCRETE_WORD_1",
                "FG_DISCRETE_WORD_2", "FG_DISCRETE_WORD_3", "LANDING_FCT_DISCRETE_WORD",
            ] {
                assert_eq!(word(&mut rig.vars, &format!("A32NX_FCDC_{n}_{name}")).SSM, SSM_NO, "{name}");
            }
            // All PRIMs alive: no "all PRIMs dead" bits in word 3 (cpp:97-105).
            let w3 = word(&mut rig.vars, &format!("A32NX_FCDC_{n}_DISCRETE_WORD_3"));
            assert!(!bit_from_value(w3, 19) && !bit_from_value(w3, 29));
            // Spoilers valid from the PRIM's positions (cpp:144-150).
            let w4 = word(&mut rig.vars, &format!("A32NX_FCDC_{n}_DISCRETE_WORD_4"));
            assert!(bit_from_value(w4, 21));
        }
        // The FWS writes no word 126 here, so BTV is lost (cpp:360-366).
        assert_eq!(rig.vars.value("A32NX_BTV_LOST"), 1.);
    }

    #[test]
    fn no_cpiom_no_healthy_fcdc() {
        let mut rig = Rig::new(true, 0., 0.);
        network_up(&mut rig.vars);
        // The real ADCN takes switch 3 (channel A) down with its DC ESS
        // supply; switch 13 (channel B) is wired from AC ESS instead
        // (avionics_data_communication_network.rs), so losing just this one
        // bus is how FCDC 1's `afdx_comm_available` check
        // (`A32NX_AFDX_SWITCH_3_AVAIL || A32NX_AFDX_SWITCH_13_AVAIL`) still
        // needs switch 13 killed too to go unhealthy.
        rig.vars.set("A32NX_AFDX_SWITCH_3_AVAIL", 0.);
        rig.vars.set("A32NX_AFDX_SWITCH_13_AVAIL", 0.);
        let mut fcdc = ExtraBackendFcdc::without_xplane();
        step(&mut rig, &mut fcdc, &FcdcSimData::default(), &SimReadings::default(), 40);
        assert_eq!(rig.vars.value("A32NX_FCDC_1_HEALTHY"), 0.);
        for name in ["DISCRETE_WORD_1", "DISCRETE_WORD_5", "FG_DISCRETE_WORD_3"] {
            let w = word(&mut rig.vars, &format!("A32NX_FCDC_1_{name}"));
            // cpp:61-70: failure warning, no data.
            assert_eq!((w.SSM, w.Data), (SSM_FW, 0.), "{name}");
        }
        assert_eq!(rig.vars.value("A32NX_FCDC_2_HEALTHY"), 1.);
    }

    /// The FCDC1/FCDC2 FailuresConsumer ids (27006/27007), toggled through
    /// X-Plane's `fbw/failure/<id>` as the plugin does, reach
    /// `failuresConsumer.isActive` at cpp:2299 through
    /// `crate::failures::active_ids()`, exactly as [`ExtraBackendFcdc::update`]
    /// (not `update_with`) reads them.
    #[test]
    fn the_fcdc_failure_ids_reach_the_fcdc_through_the_global_failures_state() {
        let _g = crate::failures::tests::SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        let _failures = crate::failures::Failures::new();
        let mut rig = Rig::new(true, 0., 0.);
        network_up(&mut rig.vars);
        let mut fcdc = ExtraBackendFcdc::without_xplane();
        let sim = FcdcSimData { engine_combustion: [true; 4] };

        crate::failures::set_active(27_006, true);
        for _ in 0..40 {
            rig.tick(&[]);
            fcdc.update_with(&mut rig.vars, &sim, &rig.prims, &SimReadings::default(), DT, &crate::failures::active_ids());
        }
        assert_eq!(rig.vars.value("A32NX_FCDC_1_HEALTHY"), 0., "FCDC 1 failed through the global failures state");
        assert_eq!(rig.vars.value("A32NX_FCDC_2_HEALTHY"), 1., "FCDC 2 untouched");

        crate::failures::replace([]);
    }

    #[test]
    fn self_test_after_a_power_outage() {
        let mut f = Fcdc::new(true);
        // Zero-initialised and powered from the start: no startup, healthy at once.
        f.update(0.05, false, true);
        assert!(f.get_discrete_outputs().fcdc_valid);

        // An outage over 3 s standing on the nose gear: the long self test.
        f.discrete_inputs.nose_gear_pressed = true;
        for _ in 0..70 {
            f.update(0.05, false, false);
        }
        assert!(!f.get_discrete_outputs().fcdc_valid);
        let mut t: f64 = 0.;
        while !{
            f.update(0.05, false, true);
            f.get_discrete_outputs().fcdc_valid
        } {
            t += 0.05;
            assert!(t < 10.);
        }
        assert!((t - 3.).abs() < 0.051, "{t}");

        // A short outage: 0.5 s (Fcdc.cpp:14-15).
        for _ in 0..10 {
            f.update(0.05, false, false);
        }
        let mut t: f64 = 0.;
        while !{
            f.update(0.05, false, true);
            f.get_discrete_outputs().fcdc_valid
        } {
            t += 0.05;
        }
        assert!((t - 0.5).abs() < 0.051, "{t}");

        // A failure makes it unhealthy while active (Fcdc.cpp:374).
        f.update(0.05, true, true);
        assert!(!f.get_discrete_outputs().fcdc_valid);
        assert_eq!(f.get_bus_outputs().efcs_status_1.SSM, SSM_FW);
    }

    #[test]
    fn all_prims_dead_is_direct_law() {
        let mut f = Fcdc::new(false);
        f.update(0.05, false, true);
        let out = f.get_bus_outputs();
        // No master PRIM bit anywhere (cpp:37-39) -> direct law (cpp:77-87).
        assert_eq!(out.efcs_status_1.SSM, SSM_NO);
        assert_eq!(out.efcs_status_1.Data as u32, 1 << 14 | 1 << 16);
        // Bits 21-25 are set again from the spoiler status word right after
        // (cpp:126-130), which is not normal operation here.
        for b in [19, 20, 26, 29] {
            assert!(bit_from_value(out.efcs_status_3, b), "bit {b}");
        }
        for b in 21..=25 {
            assert!(!bit_from_value(out.efcs_status_3, b), "bit {b}");
        }
    }

    #[test]
    fn spoiler_lvars_and_fcdc_bits_follow_the_speedbrake_handle() {
        let mut rig = Rig::new(true, 0., 0.);
        network_up(&mut rig.vars);
        let mut fcdc = ExtraBackendFcdc::without_xplane();
        let sim = FcdcSimData { engine_combustion: [true; 4] };
        for (ratio, armed, handle) in [(-0.5, 1., 0.), (0., 0., 0.), (0.5, 0., 0.5), (1., 0., 1.)] {
            let (spoilers_armed, spoilers_handle_position) = SimReadings::spoilers_from_xplane(ratio);
            let readings = SimReadings { spoilers_armed, spoilers_handle_position, ..Default::default() };
            step(&mut rig, &mut fcdc, &sim, &readings, 2);
            assert_eq!(rig.vars.value("A32NX_SPOILERS_ARMED"), armed, "ratio {ratio}");
            assert_eq!(rig.vars.value("A32NX_SPOILERS_HANDLE_POSITION"), handle, "ratio {ratio}");
            let w4 = word(&mut rig.vars, "A32NX_FCDC_1_DISCRETE_WORD_4");
            assert_eq!(bit_from_value(w4, 27), armed == 1., "armed bit, ratio {ratio}");
            assert_eq!(bit_from_value(w4, 28), handle > 0.9, "lever bit, ratio {ratio}");
        }
    }

    #[test]
    fn triggered_monostable_node_emits_once_per_rising_edge() {
        let mut node = TriggeredMonostableNode::new(1.);
        assert!(!node.write(false, 0.25));
        assert!(node.write(true, 0.25));
        for _ in 0..3 {
            assert!(node.write(true, 0.25));
        }
        assert!(!node.write(true, 0.25));
        assert!(!node.read());
        // Not again while held, again on the next rising edge.
        assert!(!node.write(true, 0.25));
        assert!(!node.write(false, 0.25));
        assert!(node.write(true, 0.25));
    }

    #[test]
    fn set_bit_and_bit_from_value_match_arinc429() {
        let mut w = BaseArinc429::default();
        set_bit(&mut w, 11, true);
        set_bit(&mut w, 29, true);
        assert_eq!(w.Data as u32, 1 << 10 | 1 << 28);
        set_bit(&mut w, 11, false);
        assert!(!bit_from_value(w, 11) && bit_from_value(w, 29));
        assert!(is_fw(w) && !is_no(w));
    }

    /// FCDC-001: any_aileron_fault (BTV's ldg_dist_affected_btv_lost) reads
    /// the PRIM's real aileron status word (bits 11/14, "section available"
    /// convention as efcs_status_2/3 already use for the same word), not the
    /// hard-coded `false` Fcdc.cpp itself never wires (cpp:346).
    #[test]
    fn any_aileron_fault_reads_the_prim_aileron_status_word() {
        let mut both_available = BaseArinc429::default();
        set_ssm(&mut both_available, SSM_NO);
        set_bit(&mut both_available, 11, true);
        set_bit(&mut both_available, 14, true);
        assert!(!(!bit_or(both_available, 11, false) || !bit_or(both_available, 14, false)));

        let mut one_faulted = both_available;
        set_bit(&mut one_faulted, 11, false);
        assert!(!bit_or(one_faulted, 11, false) || !bit_or(one_faulted, 14, false));
    }

    /// FCDC-003: nws_communication_available follows handling.rs's steering
    /// hydraulic supply (Green system) rather than only the rollout-failure
    /// injection.
    #[test]
    fn nws_communication_follows_green_hydraulic_supply() {
        let mut rig = Rig::new(true, 0., 0.);
        network_up(&mut rig.vars);
        rig.vars.set("A32NX_HYD_GREEN_SYSTEM_1_SECTION_PRESSURE_SWITCH", 1.);
        let mut fcdc = ExtraBackendFcdc::without_xplane();
        let sim = FcdcSimData { engine_combustion: [true; 4] };
        step(&mut rig, &mut fcdc, &sim, &SimReadings::default(), 5);
        assert!(fcdc.fcdcs[0].discrete_inputs.nws_communication_available);

        rig.vars.set("A32NX_HYD_GREEN_SYSTEM_1_SECTION_PRESSURE_SWITCH", 0.);
        step(&mut rig, &mut fcdc, &sim, &SimReadings::default(), 2);
        assert!(!fcdc.fcdcs[0].discrete_inputs.nws_communication_available);
    }
}
