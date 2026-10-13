//! The FCU, autopilot and autothrust events FlyByWire's fly-by-wire module
//! takes from MSFS, as X-Plane commands.
//!
//! In MSFS these are key events (`A32NX.FCU_ATHR_PUSH`, `AUTO_THROTTLE_ARM`,
//! ...). SimConnectInterface turns each into a one-frame input
//! (SimConnectInterface.cpp:2223-3055), cleared at the start of every update
//! (resetSimInputAutopilot / resetFcuFrontPanelInputs / resetSimInputThrottles,
//! SimConnectInterface.cpp:1562-1609, called from FlyByWireInterface.cpp:981-989).
//! Here each event is an X-Plane command `fbw/event/<MSFS name with dots as
//! underscores>`; a press between two ticks is the input for the next tick,
//! and the tick takes and clears them the same way.
//!
//! Events that carry a value (`A32NX.FCU_SPD_SET` and friends) have no command;
//! the port can queue them itself (FCU initialisation does), but nothing in
//! X-Plane sends them yet.

#![allow(dead_code)]

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;

use crate::fbw_types::{BaseFcuAfsPanelInputs, BaseFcuEfisPanelInputs};
use crate::xp::{CommandRef, DataRef, Xplm};

/// One event, named as in SimConnectInterface's `Events` enum.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Event {
    AutopilotOff,
    ApMaster,
    AutopilotDisengageToggle,
    FcuAp1Push,
    FcuAp2Push,
    FcuApDisconnectPush,
    FcuAthrPush,
    FcuAthrDisconnectPush,
    FcuFdPush,
    ToggleFlightDirector,
    FcuSpdInc,
    FcuSpdDec,
    FcuSpdSet(f64),
    FcuSpdPush,
    FcuSpdPull,
    FcuSpdMachTogglePush,
    FcuHdgInc,
    FcuHdgDec,
    FcuHdgSet(f64),
    FcuHdgPush,
    FcuHdgPull,
    FcuTrkFpaTogglePush,
    FcuTrueTogglePush,
    FcuAltInc,
    FcuAltDec,
    FcuAltSet(f64),
    FcuAltPush,
    FcuAltPull,
    FcuMetricAltTogglePush,
    FcuVsInc,
    FcuVsDec,
    FcuVsSet(f64),
    FcuVsPush,
    FcuVsPull,
    FcuLocPush,
    FcuApprPush,
    FcuAltButtonPush,
    /// A detent of one of the EFIS control panel's two knobs: which side
    /// (0 left, 1 right) and which way (+1 clockwise, -1 anticlockwise).
    /// The A380's ND mode and range knobs are infinite rotaries, and the FCU
    /// can count detents (`efis_mode_knob_turns`/`efis_range_knob_turns`)
    /// as well as take a position. This is what an MSFS key event carries.
    FcuEfisModeTurn(usize, i8),
    FcuEfisRangeTurn(usize, i8),
    FcuEfisBaroTurn(usize, i8),
    FcuEfisBaroPush(usize),
    FcuEfisBaroPull(usize),
    FcuEfisButton(usize, EfisButton),

    AutoThrottleArm,
    AutoThrottleDisconnect,
    AthrResetDisable,
    /// `K:A32NX.FMGC_DIR_TO_TRIGGER`: MfdFmsFplnDirectTo.tsx's DIR TO INSERT
    /// button (SimConnectInterface.cpp:2890 `simInputAutopilot.DIR_TO_trigger = 1`).
    FmgcDirToTrigger,
    /// `K:AP_MANAGED_SPEED_IN_MACH_ON`: FmcAircraftInterface.ts:1071 toggling the
    /// managed speed target to Mach. MSFS maps this stock event to its own
    /// `Events::A32NX_FMGC_MACH_MODE_ACTIVATE` (SimConnectInterface.cpp:759),
    /// not a same-named custom event - the mapping is intentional, not a typo.
    ApManagedSpeedInMachOn,
    /// `K:AP_MANAGED_SPEED_IN_MACH_OFF`: the same control's speed (non-Mach)
    /// side, mapped to `Events::A32NX_FMGC_SPD_MODE_ACTIVATE`
    /// (SimConnectInterface.cpp:760).
    ApManagedSpeedInMachOff,
    /// `K:A32NX.FMS_PRESET_SPD_ACTIVATE` (SimConnectInterface.cpp:2908
    /// `simInputAutopilot.preset_spd_activate = 1`).
    FmgcPresetSpdActivate,
}

/// A pushbutton on an EFIS control panel, as `A32NX.FCU_EFIS_<side>_<name>_PUSH`
/// names it (SimConnectInterface.cpp:2633-2725).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EfisButton {
    Vv,
    Ls,
    Taxi,
    Cstr,
    Wpt,
    Vord,
    Ndb,
    Arpt,
    Navaid1,
    Navaid2,
    Wx,
    Terr,
    Traf,
}

impl EfisButton {
    pub const ALL: [(EfisButton, &'static str); 13] = [
        (EfisButton::Vv, "VV"),
        (EfisButton::Ls, "LS"),
        (EfisButton::Taxi, "TAXI"),
        (EfisButton::Cstr, "CSTR"),
        (EfisButton::Wpt, "WPT"),
        (EfisButton::Vord, "VORD"),
        (EfisButton::Ndb, "NDB"),
        (EfisButton::Arpt, "ARPT"),
        (EfisButton::Navaid1, "NAVAID_1"),
        (EfisButton::Navaid2, "NAVAID_2"),
        (EfisButton::Wx, "WX"),
        (EfisButton::Terr, "TERR"),
        (EfisButton::Traf, "TRAF"),
    ];
}

/// `A32NX.FCU_EFIS_L_LS_PUSH` and the rest of both panels' pushbuttons.
pub fn efis_button_event(name: &str) -> Option<Event> {
    let rest = name.strip_prefix("A32NX.FCU_EFIS_")?.strip_suffix("_PUSH")?;
    let (side, button) = rest.split_once('_')?;
    let side = ["L", "R"].iter().position(|s| *s == side)?;
    EfisButton::ALL.iter().find(|(_, n)| *n == button).map(|&(b, _)| Event::FcuEfisButton(side, b))
}

fn efis_button_command(i: usize) -> Option<Event> {
    let j = i.checked_sub(COMMANDS.len())?;
    let per_side = EfisButton::ALL.len();
    (j < 2 * per_side).then(|| Event::FcuEfisButton(j / per_side, EfisButton::ALL[j % per_side].0))
}

/// The X-Plane commands: (command suffix, MSFS event name, event).
const COMMANDS: &[(&str, &str, Event)] = &[
    ("AUTOPILOT_OFF", "AUTOPILOT_OFF", Event::AutopilotOff),
    ("AP_MASTER", "AP_MASTER", Event::ApMaster),
    ("AUTOPILOT_DISENGAGE_TOGGLE", "AUTOPILOT_DISENGAGE_TOGGLE", Event::AutopilotDisengageToggle),
    ("A32NX_FCU_AP_1_PUSH", "A32NX.FCU_AP_1_PUSH", Event::FcuAp1Push),
    ("A32NX_FCU_AP_2_PUSH", "A32NX.FCU_AP_2_PUSH", Event::FcuAp2Push),
    ("A32NX_FCU_AP_DISCONNECT_PUSH", "A32NX.FCU_AP_DISCONNECT_PUSH", Event::FcuApDisconnectPush),
    ("A32NX_FCU_ATHR_PUSH", "A32NX.FCU_ATHR_PUSH", Event::FcuAthrPush),
    ("A32NX_FCU_ATHR_DISCONNECT_PUSH", "A32NX.FCU_ATHR_DISCONNECT_PUSH", Event::FcuAthrDisconnectPush),
    ("A32NX_FCU_FD_PUSH", "A32NX.FCU_FD_PUSH", Event::FcuFdPush),
    ("TOGGLE_FLIGHT_DIRECTOR", "TOGGLE_FLIGHT_DIRECTOR", Event::ToggleFlightDirector),
    ("A32NX_FCU_SPD_INC", "A32NX.FCU_SPD_INC", Event::FcuSpdInc),
    ("A32NX_FCU_SPD_DEC", "A32NX.FCU_SPD_DEC", Event::FcuSpdDec),
    ("A32NX_FCU_SPD_PUSH", "A32NX.FCU_SPD_PUSH", Event::FcuSpdPush),
    ("A32NX_FCU_SPD_PULL", "A32NX.FCU_SPD_PULL", Event::FcuSpdPull),
    ("A32NX_FCU_SPD_MACH_TOGGLE_PUSH", "A32NX.FCU_SPD_MACH_TOGGLE_PUSH", Event::FcuSpdMachTogglePush),
    ("A32NX_FCU_HDG_INC", "A32NX.FCU_HDG_INC", Event::FcuHdgInc),
    ("A32NX_FCU_HDG_DEC", "A32NX.FCU_HDG_DEC", Event::FcuHdgDec),
    ("A32NX_FCU_HDG_PUSH", "A32NX.FCU_HDG_PUSH", Event::FcuHdgPush),
    ("A32NX_FCU_HDG_PULL", "A32NX.FCU_HDG_PULL", Event::FcuHdgPull),
    ("A32NX_FCU_TRK_FPA_TOGGLE_PUSH", "A32NX.FCU_TRK_FPA_TOGGLE_PUSH", Event::FcuTrkFpaTogglePush),
    ("A32NX_FCU_TRUE_TOGGLE_PUSH", "A32NX.FCU_TRUE_TOGGLE_PUSH", Event::FcuTrueTogglePush),
    ("A32NX_FCU_ALT_INC", "A32NX.FCU_ALT_INC", Event::FcuAltInc),
    ("A32NX_FCU_ALT_DEC", "A32NX.FCU_ALT_DEC", Event::FcuAltDec),
    ("A32NX_FCU_ALT_PUSH", "A32NX.FCU_ALT_PUSH", Event::FcuAltPush),
    ("A32NX_FCU_ALT_PULL", "A32NX.FCU_ALT_PULL", Event::FcuAltPull),
    ("A32NX_FCU_METRIC_ALT_TOGGLE_PUSH", "A32NX.FCU_METRIC_ALT_TOGGLE_PUSH", Event::FcuMetricAltTogglePush),
    ("A32NX_FCU_VS_INC", "A32NX.FCU_VS_INC", Event::FcuVsInc),
    ("A32NX_FCU_VS_DEC", "A32NX.FCU_VS_DEC", Event::FcuVsDec),
    ("A32NX_FCU_VS_PUSH", "A32NX.FCU_VS_PUSH", Event::FcuVsPush),
    ("A32NX_FCU_VS_PULL", "A32NX.FCU_VS_PULL", Event::FcuVsPull),
    ("A32NX_FCU_LOC_PUSH", "A32NX.FCU_LOC_PUSH", Event::FcuLocPush),
    ("A32NX_FCU_APPR_PUSH", "A32NX.FCU_APPR_PUSH", Event::FcuApprPush),
    ("A32NX_FCU_ALT_BUTTON_PUSH", "A32NX.FCU_ALT_BUTTON_PUSH", Event::FcuAltButtonPush),
    // The EFIS control panel's ND mode and range knobs. efis-cp.xml's
    // `FBW_AIRLINER_Knob_ND_Template` fires these as key events
    // (`'A32NX.FCU_EFIS_#SIDE#_#TYPE#_INC' (>F:KeyEvent)`), the only
    // controls on that panel that do -- every button beside them writes its
    // L:var through RPN instead, which is why this family was once read as
    // entirely RPN-driven and left unserved.
    ("A32NX_FCU_EFIS_L_MODE_INC", "A32NX.FCU_EFIS_L_MODE_INC", Event::FcuEfisModeTurn(0, 1)),
    ("A32NX_FCU_EFIS_L_MODE_DEC", "A32NX.FCU_EFIS_L_MODE_DEC", Event::FcuEfisModeTurn(0, -1)),
    ("A32NX_FCU_EFIS_L_RANGE_INC", "A32NX.FCU_EFIS_L_RANGE_INC", Event::FcuEfisRangeTurn(0, 1)),
    ("A32NX_FCU_EFIS_L_RANGE_DEC", "A32NX.FCU_EFIS_L_RANGE_DEC", Event::FcuEfisRangeTurn(0, -1)),
    ("A32NX_FCU_EFIS_R_MODE_INC", "A32NX.FCU_EFIS_R_MODE_INC", Event::FcuEfisModeTurn(1, 1)),
    ("A32NX_FCU_EFIS_R_MODE_DEC", "A32NX.FCU_EFIS_R_MODE_DEC", Event::FcuEfisModeTurn(1, -1)),
    ("A32NX_FCU_EFIS_R_RANGE_INC", "A32NX.FCU_EFIS_R_RANGE_INC", Event::FcuEfisRangeTurn(1, 1)),
    ("A32NX_FCU_EFIS_R_RANGE_DEC", "A32NX.FCU_EFIS_R_RANGE_DEC", Event::FcuEfisRangeTurn(1, -1)),
    ("A32NX_FCU_EFIS_L_BARO_INC", "A32NX.FCU_EFIS_L_BARO_INC", Event::FcuEfisBaroTurn(0, 1)),
    ("A32NX_FCU_EFIS_L_BARO_DEC", "A32NX.FCU_EFIS_L_BARO_DEC", Event::FcuEfisBaroTurn(0, -1)),
    ("A32NX_FCU_EFIS_L_BARO_PUSH", "A32NX.FCU_EFIS_L_BARO_PUSH", Event::FcuEfisBaroPush(0)),
    ("A32NX_FCU_EFIS_L_BARO_PULL", "A32NX.FCU_EFIS_L_BARO_PULL", Event::FcuEfisBaroPull(0)),
    ("A32NX_FCU_EFIS_R_BARO_INC", "A32NX.FCU_EFIS_R_BARO_INC", Event::FcuEfisBaroTurn(1, 1)),
    ("A32NX_FCU_EFIS_R_BARO_DEC", "A32NX.FCU_EFIS_R_BARO_DEC", Event::FcuEfisBaroTurn(1, -1)),
    ("A32NX_FCU_EFIS_R_BARO_PUSH", "A32NX.FCU_EFIS_R_BARO_PUSH", Event::FcuEfisBaroPush(1)),
    ("A32NX_FCU_EFIS_R_BARO_PULL", "A32NX.FCU_EFIS_R_BARO_PULL", Event::FcuEfisBaroPull(1)),
    ("sim/instruments/barometer_up", "KOHLSMAN_INC 1", Event::FcuEfisBaroTurn(0, 1)),
    ("sim/instruments/barometer_down", "KOHLSMAN_DEC 1", Event::FcuEfisBaroTurn(0, -1)),
    ("sim/instruments/barometer_copilot_up", "KOHLSMAN_INC 2", Event::FcuEfisBaroTurn(1, 1)),
    ("sim/instruments/barometer_copilot_down", "KOHLSMAN_DEC 2", Event::FcuEfisBaroTurn(1, -1)),
    ("sim/instruments/barometer_std", "BAROMETRIC_STD_PRESSURE 1", Event::FcuEfisBaroPull(0)),
    ("sim/instruments/barometer_2992", "BAROMETRIC_STD_PRESSURE 1", Event::FcuEfisBaroPull(0)),
    ("sim/instruments/barometer_copilot_std", "BAROMETRIC_STD_PRESSURE 2", Event::FcuEfisBaroPull(1)),
    ("AUTO_THROTTLE_ARM", "AUTO_THROTTLE_ARM", Event::AutoThrottleArm),
    ("AUTO_THROTTLE_DISCONNECT", "AUTO_THROTTLE_DISCONNECT", Event::AutoThrottleDisconnect),
    ("A32NX_ATHR_RESET_DISABLE", "A32NX.ATHR_RESET_DISABLE", Event::AthrResetDisable),
];

/// `SimInputAutopilot` (SimConnectData.h:158), the fields FlyByWireInterface
/// reads, with resetSimInputAutopilot's values (SimConnectInterface.cpp:1562).
#[derive(Clone, Copy, Debug)]
pub struct SimInputAutopilot {
    pub ap_1_push: bool,
    pub ap_2_push: bool,
    pub ap_disconnect: bool,
    pub dir_to_trigger: bool,
    pub mach_mode_activate: bool,
    pub spd_mode_activate: bool,
    pub preset_spd_activate: bool,
    pub spd_mach_set: f64,
    pub hdg_trk_set: f64,
    pub alt_set: f64,
    pub vs_fpa_set: f64,
    pub baro_left_set: f64,
    pub baro_right_set: f64,
    pub efis_mode_left_set: f64,
    pub efis_range_left_set: f64,
    pub efis_navaid_mode_1_left_set: f64,
    pub efis_navaid_mode_2_left_set: f64,
    pub efis_mode_right_set: f64,
    pub efis_range_right_set: f64,
    pub efis_navaid_mode_1_right_set: f64,
    pub efis_navaid_mode_2_right_set: f64,
}

impl Default for SimInputAutopilot {
    fn default() -> Self {
        Self {
            ap_1_push: false,
            ap_2_push: false,
            ap_disconnect: false,
            dir_to_trigger: false,
            mach_mode_activate: false,
            spd_mode_activate: false,
            preset_spd_activate: false,
            spd_mach_set: -1.,
            hdg_trk_set: -1.,
            alt_set: -1.,
            vs_fpa_set: -1.,
            baro_left_set: -1.,
            baro_right_set: -1.,
            efis_mode_left_set: -1.,
            efis_range_left_set: -1.,
            efis_navaid_mode_1_left_set: -1.,
            efis_navaid_mode_2_left_set: -1.,
            efis_mode_right_set: -1.,
            efis_range_right_set: -1.,
            efis_navaid_mode_1_right_set: -1.,
            efis_navaid_mode_2_right_set: -1.,
        }
    }
}

/// `SimInputThrottles` (SimConnectData.h:183).
#[derive(Clone, Copy, Debug, Default)]
pub struct SimInputThrottles {
    pub athr_push: bool,
    pub athr_disconnect: bool,
    pub athr_reset_disable: bool,
}

/// Everything one tick's events set.
#[derive(Clone, Copy, Debug, Default)]
pub struct EventInputs {
    pub autopilot: SimInputAutopilot,
    pub throttles: SimInputThrottles,
    pub afs: BaseFcuAfsPanelInputs,
    pub efis: [BaseFcuEfisPanelInputs; 2],
}

impl EventInputs {
    /// Apply one event as SimConnectInterface::processEvent does.
    pub fn apply(&mut self, event: Event) {
        let ap = &mut self.autopilot;
        let afs = &mut self.afs;
        match event {
            Event::AutopilotOff => ap.ap_disconnect = true, // cpp:2223
            Event::ApMaster => ap.ap_1_push = true,         // cpp:2241
            Event::AutopilotDisengageToggle => ap.ap_1_push = true, // cpp:2258
            Event::FcuAp1Push => ap.ap_1_push = true,       // cpp:2264
            Event::FcuAp2Push => ap.ap_2_push = true,       // cpp:2270
            Event::FcuApDisconnectPush => ap.ap_disconnect = true, // cpp:2276
            Event::FcuAthrPush => self.throttles.athr_push = true, // cpp:2282
            Event::FcuAthrDisconnectPush => self.throttles.athr_disconnect = true, // cpp:2288
            Event::FcuFdPush | Event::ToggleFlightDirector => afs.fd_button_pressed = 1, // cpp:2294, 2235
            Event::FcuSpdInc => afs.spd_knob.turns = 1,     // cpp:2300
            Event::FcuSpdDec => afs.spd_knob.turns = -1,    // cpp:2306
            Event::FcuSpdSet(v) => ap.spd_mach_set = v as i64 as f64, // cpp:2312
            Event::FcuSpdPush => afs.spd_knob.pushed = 1,   // cpp:2318
            Event::FcuSpdPull => afs.spd_knob.pulled = 1,   // cpp:2325
            Event::FcuSpdMachTogglePush => afs.spd_mach_button_pressed = 1, // cpp:2332
            Event::FcuHdgInc => afs.hdg_trk_knob.turns = 1, // cpp:2339
            Event::FcuHdgDec => afs.hdg_trk_knob.turns = -1, // cpp:2345
            Event::FcuHdgSet(v) => ap.hdg_trk_set = v as i64 as f64, // cpp:2351
            Event::FcuHdgPush => afs.hdg_trk_knob.pushed = 1, // cpp:2357
            Event::FcuHdgPull => afs.hdg_trk_knob.pulled = 1, // cpp:2364
            Event::FcuTrkFpaTogglePush => afs.trk_fpa_button_pressed = 1, // cpp:2371
            Event::FcuTrueTogglePush => afs.true_mag_button_pressed = 1, // cpp:2378
            Event::FcuAltInc => afs.alt_knob.turns = 1,     // cpp:2384
            Event::FcuAltDec => afs.alt_knob.turns = -1,    // cpp:2390
            Event::FcuAltSet(v) => ap.alt_set = v as i64 as f64, // cpp:2396
            Event::FcuAltPush => afs.alt_knob.pushed = 1,   // cpp:2423
            Event::FcuAltPull => afs.alt_knob.pulled = 1,   // cpp:2430
            Event::FcuMetricAltTogglePush => afs.metric_alt_button_pressed = 1, // cpp:2437
            Event::FcuVsInc => afs.vs_fpa_knob.turns = 1,   // cpp:2443
            Event::FcuVsDec => afs.vs_fpa_knob.turns = -1,  // cpp:2449
            Event::FcuVsSet(v) => ap.vs_fpa_set = v as i64 as f64, // cpp:2455
            Event::FcuVsPush => afs.vs_fpa_knob.pushed = 1, // cpp:2461
            Event::FcuVsPull => afs.vs_fpa_knob.pulled = 1, // cpp:2468
            Event::FcuLocPush => afs.loc_button_pressed = 1, // cpp:2475
            Event::FcuApprPush => afs.appr_button_pressed = 1, // cpp:2482
            Event::FcuAltButtonPush => afs.alt_button_pressed = 1, // cpp:2489
            // Set, not accumulated, exactly as every other knob above: one
            // tick carries one detent, and `EventInputs` is rebuilt each
            // tick so the turn does not linger into the next one.
            Event::FcuEfisModeTurn(side, turns) => self.efis[side].efis_mode_knob_turns = turns,
            Event::FcuEfisRangeTurn(side, turns) => self.efis[side].efis_range_knob_turns = turns,
            Event::FcuEfisBaroTurn(side, turns) => self.efis[side].baro_knob.turns = turns,
            Event::FcuEfisBaroPush(side) => self.efis[side].baro_knob.pushed = 1,
            Event::FcuEfisBaroPull(side) => self.efis[side].baro_knob.pulled = 1,
            Event::FcuEfisButton(side, button) => {
                let e = &mut self.efis[side];
                *match button {
                    EfisButton::Vv => &mut e.vv_button_pushed,
                    EfisButton::Ls => &mut e.ls_button_pushed,
                    EfisButton::Taxi => &mut e.taxi_button_pushed,
                    EfisButton::Cstr => &mut e.cstr_button_pushed,
                    EfisButton::Wpt => &mut e.wpt_button_pushed,
                    EfisButton::Vord => &mut e.vord_button_pushed,
                    EfisButton::Ndb => &mut e.ndb_button_pushed,
                    EfisButton::Arpt => &mut e.arpt_button_pushed,
                    EfisButton::Navaid1 => &mut e.navaid_1_button_pushed,
                    EfisButton::Navaid2 => &mut e.navaid_2_button_pushed,
                    EfisButton::Wx => &mut e.wx_button_pushed,
                    EfisButton::Terr => &mut e.terr_button_pushed,
                    EfisButton::Traf => &mut e.traf_button_pushed,
                } = 1;
            }

            Event::AutoThrottleArm => self.throttles.athr_push = true, // cpp:3036
            Event::AutoThrottleDisconnect => self.throttles.athr_disconnect = true, // cpp:3042
            Event::AthrResetDisable => self.throttles.athr_reset_disable = true, // cpp:3051
            Event::FmgcDirToTrigger => ap.dir_to_trigger = true, // cpp:2891
            Event::ApManagedSpeedInMachOn => ap.mach_mode_activate = true, // cpp:2897
            Event::ApManagedSpeedInMachOff => ap.spd_mode_activate = true, // cpp:2903
            Event::FmgcPresetSpdActivate => ap.preset_spd_activate = true, // cpp:2909
        }
    }
}

static PENDING: Mutex<Vec<Event>> = Mutex::new(Vec::new());

/// Queue an event for the next tick, as a key event arriving between frames.
pub fn send(event: Event) {
    if let Ok(mut pending) = PENDING.lock() {
        pending.push(event);
    }
}

/// This tick's inputs: every event since the last call, applied to the
/// cleared inputs.
pub fn take() -> EventInputs {
    let events = PENDING.lock().map(|mut p| std::mem::take(&mut *p)).unwrap_or_default();
    let mut inputs = EventInputs::default();
    for event in events {
        inputs.apply(event);
    }
    inputs
}

// ---------------------------------------------------------------------------
// The EFIS control panel's ND mode and range knobs.
//
// These are the one part of that panel MSFS drives with key events:
// efis-cp.xml rigs each as an `ASOBO_GT_Knob_Infinite` whose turn code is
// `'A32NX.FCU_EFIS_#SIDE#_#TYPE#_INC' (>F:KeyEvent)`. The converted cockpit
// does not reproduce that, and -- this is the part that matters -- it does
// not reproduce it in a way that took two attempts to find.
//
// The aircraft exports each knob as an `ATTR_manip_command_knob` over a
// `fbw/cockpit/KNOB_EFIS_{CS,FO}_{ROSE,RANGE}_{up,down}` command pair, so
// the obvious reading is that the matching `fbw/cockpit/...` dataref is a
// counter to watch. It is not: it exists for the knob's *animation*, and the
// SASL script's command handler never touches it. What the handler actually
// does (`plugins/sasl/data/modules/main.lua`) is read
// `fbw/A32NX_EFIS_L_ND_MODE` and `fbw/A32NX_EFIS_L_ND_RANGE`, compute the
// next value with the panel's own clamping, and write it straight back.
//
// Those are this plugin's own published variables -- every FlyByWire
// variable becomes a writable `fbw/<name>` dataref -- and `prim.rs`
// republishes both of them every tick from the FCU's EFIS discrete word. So
// the crew's turn landed, and was overwritten before anything could read it.
// That is the whole "the knob moves and the picture never follows" symptom.
//
// The fix is to let the FCU see it. `FcuInputs::sim_input` carries
// `efis_mode`/`efis_range` as absolute values for exactly this purpose (-1
// meaning "no change"), fed from `SimInputAutopilot::efis_mode_left_set` and
// friends, which nothing in this port had ever assigned. Feeding them the
// dataref's current value each tick is a no-op while the FCU is the only
// writer -- it is handed back the number it just published -- and becomes
// the crew's command on the tick they turn the knob.
// ---------------------------------------------------------------------------

/// One knob's published variable and the detent event a turn of it sends.
struct KnobVar {
    path: &'static str,
    zoom_path: Option<&'static str>,
    event: fn(usize, i8) -> Event,
    side: usize,
    dataref: Option<DataRef>,
    zoom_dataref: Option<DataRef>,
    /// The value `prim.rs` last published, taken after the systems tick
    /// ([`EfisKnobs::latch`]). A read that differs from this is the crew's,
    /// because nothing else writes between the two points.
    baseline: Option<f64>,
}

/// A knob cannot be turned more than this in one tick. Anything larger is
/// the variable being *set* rather than turned -- a state load, a preset --
/// and must not be replayed into the FCU as a burst of detents.
const MAX_DETENTS_PER_TICK: f64 = 8.0;

impl KnobVar {
    const fn new(path: &'static str, side: usize, event: fn(usize, i8) -> Event) -> Self {
        Self { path, zoom_path: None, event, side, dataref: None, zoom_dataref: None, baseline: None }
    }

    const fn range(path: &'static str, zoom_path: &'static str, side: usize, event: fn(usize, i8) -> Event) -> Self {
        Self { path, zoom_path: Some(zoom_path), event, side, dataref: None, zoom_dataref: None, baseline: None }
    }

    fn read(&mut self, xplm: &Xplm) -> Option<f64> {
        if self.dataref.is_none() {
            self.dataref = xplm.find(self.path);
        }
        let value = xplm.get_f(self.dataref?) as f64;
        let Some(zoom_path) = self.zoom_path else { return Some(value) };
        if self.zoom_dataref.is_none() {
            self.zoom_dataref = xplm.find(zoom_path);
        }
        Some(range_selection(value, xplm.get_f(self.zoom_dataref?) as f64))
    }

    /// Turn the crew's write into detents.
    ///
    /// **Detents, not the value itself.** The published variable and the
    /// FCU's own input are on different scales: `prim.rs` publishes the ND's
    /// scale (0 = ZOOM, 1 = 10 NM, 2 = 20, ...) decoded from the EFIS
    /// discrete word, while `sim_input.efis_range` is the FCU's
    /// `a380_efis_range_selection` (0 = ZOOM 0.2, ... 5 = RANGE_10, ...).
    /// Feeding one into the other lands five steps low -- deep in the ZOOM
    /// band -- which is precisely when the ND draws the OANS airport map
    /// over ARC, NAV and PLAN, and shows NOT AVAIL there with no airport
    /// database loaded.
    ///
    /// A *difference* needs no shared origin. Both scales increase with
    /// range and with mode, so one step up on the published scale is one
    /// detent clockwise, whatever either scale's zero happens to mean.
    fn poll(&mut self, xplm: &Xplm) {
        let Some(value) = self.read(xplm) else { return };
        let Some(baseline) = self.baseline else {
            // First sight of it: record where the knob is, and send nothing.
            self.baseline = Some(value);
            return;
        };
        let moved = value - baseline;
        if moved != 0.0 && moved.abs() <= MAX_DETENTS_PER_TICK {
            send((self.event)(self.side, moved as i8));
        }
        // Either way this read is now the reference, so one turn is not
        // sent twice while the FCU catches up.
        self.baseline = Some(value);
    }

    /// Record what `prim.rs` has just published, so the next tick's read is
    /// compared against the FCU's own output rather than the crew's.
    fn latch(&mut self, xplm: &Xplm) {
        if let Some(value) = self.read(xplm) {
            self.baseline = Some(value);
        }
    }
}

/// The FCU's own range scale (0 = ZOOM 0.2 ... 4 = ZOOM 5, 5 = 10 NM ... 11 = 640) from
/// the ND range (0 = ZOOM) and the OANS zoom step the knob turns while at ZOOM, which
/// is 4 or above whenever the ND is not zoomed.
fn range_selection(nd_range: f64, oans_range: f64) -> f64 {
    if nd_range <= 0.0 {
        oans_range.clamp(0.0, 4.0)
    } else {
        nd_range + 4.0
    }
}

/// The four EFIS knob variables, captain and first officer.
pub struct EfisKnobs {
    vars: [KnobVar; 4],
}

impl EfisKnobs {
    pub const fn new() -> Self {
        Self {
            vars: [
                KnobVar::new("fbw/A32NX_EFIS_L_ND_MODE", 0, Event::FcuEfisModeTurn),
                KnobVar::range("fbw/A32NX_EFIS_L_ND_RANGE", "fbw/A32NX_EFIS_L_OANS_RANGE", 0, Event::FcuEfisRangeTurn),
                KnobVar::new("fbw/A32NX_EFIS_R_ND_MODE", 1, Event::FcuEfisModeTurn),
                KnobVar::range("fbw/A32NX_EFIS_R_ND_RANGE", "fbw/A32NX_EFIS_R_OANS_RANGE", 1, Event::FcuEfisRangeTurn),
            ],
        }
    }

    /// Call once a tick, before [`take`] collects the tick's events and
    /// before `prim.rs` republishes the variables.
    pub fn update(&mut self, xplm: &Xplm) {
        for v in &mut self.vars {
            v.poll(xplm);
        }
    }

    /// Call once a tick *after* the systems tick, when `prim.rs` has
    /// republished the EFIS variables. Without this the FCU's own output
    /// would read back next tick as though the crew had turned the knob,
    /// and every detent would repeat forever.
    pub fn latch(&mut self, xplm: &Xplm) {
        for v in &mut self.vars {
            v.latch(xplm);
        }
    }
}

impl Default for EfisKnobs {
    fn default() -> Self {
        Self::new()
    }
}

/// The converted cockpit comes from a FlyByWire package whose EFIS control
/// panel buttons write their own L:vars (`A380X_EFIS_L_LS_BUTTON_IS_ON`,
/// `A380X_EFIS_L_ACTIVE_FILTER`, ...) instead of sending
/// `A32NX.FCU_EFIS_L_LS_PUSH`. The FCU here only takes the push, and
/// `prim.rs` republishes those variables from it every tick, so a press
/// is read as a change from what the FCU last published, like the knobs.
#[derive(Clone, Copy)]
enum ButtonWatch {
    Toggle(EfisButton),
    Pressed(EfisButton),
    Filter,
    Overlay,
}

impl ButtonWatch {
    fn pushed(self, was: f64, now: f64) -> Option<EfisButton> {
        if now == was {
            return None;
        }
        let selected = if now != 0. { now } else { was };
        match self {
            ButtonWatch::Toggle(b) => Some(b),
            ButtonWatch::Pressed(b) => (was == 0.).then_some(b),
            ButtonWatch::Filter => [EfisButton::Wpt, EfisButton::Vord, EfisButton::Ndb].get((selected as usize).checked_sub(1)?).copied(),
            ButtonWatch::Overlay => [EfisButton::Wx, EfisButton::Terr].get((selected as usize).checked_sub(1)?).copied(),
        }
    }
}

struct ButtonVar {
    path: String,
    side: usize,
    watch: ButtonWatch,
    dataref: Option<DataRef>,
    baseline: Option<f64>,
}

impl ButtonVar {
    fn read(&mut self, xplm: &Xplm) -> Option<f64> {
        if self.dataref.is_none() {
            self.dataref = xplm.find(&self.path);
        }
        Some(xplm.get_f(self.dataref?) as f64)
    }

    fn poll(&mut self, xplm: &Xplm) {
        let Some(value) = self.read(xplm) else { return };
        if let Some(button) = self.baseline.and_then(|was| self.watch.pushed(was, value)) {
            crate::diag::event("efis", &format!("{} {:?} -> {value}: {button:?} pushed on side {}", self.path, self.baseline, self.side));
            send(Event::FcuEfisButton(self.side, button));
        }
        self.baseline = Some(value);
    }

    fn latch(&mut self, xplm: &Xplm) {
        if let Some(value) = self.read(xplm) {
            self.baseline = Some(value);
        }
    }
}

pub struct EfisButtons {
    vars: Vec<ButtonVar>,
}

impl EfisButtons {
    pub fn new() -> Self {
        let mut vars = Vec::new();
        for (side, (s, cockpit)) in [("L", "CS"), ("R", "FO")].into_iter().enumerate() {
            let mut add = |path: String, watch| vars.push(ButtonVar { path, side, watch, dataref: None, baseline: None });
            for (button, name) in [
                (EfisButton::Ls, "LS"),
                (EfisButton::Vv, "VV"),
                (EfisButton::Cstr, "CSTR"),
                (EfisButton::Arpt, "ARPT"),
                (EfisButton::Traf, "TRAF"),
            ] {
                add(format!("fbw/A380X_EFIS_{s}_{name}_BUTTON_IS_ON"), ButtonWatch::Toggle(button));
            }
            add(format!("fbw/A380X_EFIS_{s}_ACTIVE_FILTER"), ButtonWatch::Filter);
            add(format!("fbw/A380X_EFIS_{s}_ACTIVE_OVERLAY"), ButtonWatch::Overlay);
            add(format!("fbw/A32NX_EFIS_{s}_NAVAID_1_MODE"), ButtonWatch::Toggle(EfisButton::Navaid1));
            add(format!("fbw/A32NX_EFIS_{s}_NAVAID_2_MODE"), ButtonWatch::Toggle(EfisButton::Navaid2));
            add(format!("fbw/cockpit/PUSH_EFIS_{cockpit}_TAXI"), ButtonWatch::Pressed(EfisButton::Taxi));
        }
        Self { vars }
    }

    pub fn update(&mut self, xplm: &Xplm) {
        for v in &mut self.vars {
            v.poll(xplm);
        }
    }

    pub fn latch(&mut self, xplm: &Xplm) {
        for v in &mut self.vars {
            v.latch(xplm);
        }
    }
}

impl Default for EfisButtons {
    fn default() -> Self {
        Self::new()
    }
}

unsafe extern "C" fn on_command(_command: CommandRef, phase: std::ffi::c_int, refcon: *mut std::ffi::c_void) -> std::ffi::c_int {
    if let Some(event) = efis_button_command(refcon as usize) {
        if phase == 0 {
            crate::diag::event("command", &format!("{event:?}"));
            send(event);
        }
        return 1;
    }
    let Some(&(suffix, _, event)) = COMMANDS.get(refcon as usize) else { return 1 };
    if phase == 0 {
        crate::diag::event("command", suffix);
    }
    // A key event is one press: take the command's begin phase.
    if phase == 0 {
        send(event);
    }
    if suffix.starts_with("sim/") {
        0
    } else {
        1
    }
}

/// The commands X-Plane knows these events by.
pub struct Commands {
    registered: Vec<(CommandRef, usize)>,
}

unsafe impl Send for Commands {}

impl Commands {
    pub fn register(xplm: &Xplm) -> Self {
        let mut registered = Vec::new();
        for (i, (suffix, msfs, _)) in COMMANDS.iter().enumerate() {
            let name = if suffix.starts_with("sim/") { suffix.to_string() } else { format!("fbw/event/{suffix}") };
            if let Some(command) = xplm.create_command(&name, &format!("FlyByWire event {msfs}")) {
                xplm.register_command_handler(command, on_command, i as *mut std::ffi::c_void);
                registered.push((command, i));
            }
        }
        for (side, s) in ["L", "R"].into_iter().enumerate() {
            for (j, (_, button)) in EfisButton::ALL.iter().enumerate() {
                let i = COMMANDS.len() + side * EfisButton::ALL.len() + j;
                let name = format!("fbw/event/A32NX_FCU_EFIS_{s}_{button}_PUSH");
                if let Some(command) = xplm.create_command(&name, &format!("FlyByWire event A32NX.FCU_EFIS_{s}_{button}_PUSH")) {
                    xplm.register_command_handler(command, on_command, i as *mut std::ffi::c_void);
                    registered.push((command, i));
                }
            }
        }
        Self { registered }
    }

    pub fn release(&mut self, xplm: &Xplm) {
        for (command, i) in self.registered.drain(..) {
            xplm.unregister_command_handler(command, on_command, i as *mut std::ffi::c_void);
        }
    }
}

// ---------------------------------------------------------------------------
// Sidestick priority takeover: a held pushbutton, not a one-frame event.
//
// In MSFS this is a 3D cockpit click-spot writing L:A32NX_PRIORITY_TAKEOVER:1
// (captain) / :2 (first officer) to 1 while held, 0 on release
// (A32NX_Interior_Misc.xml:385-388), read by both PRIM and SEC discrete
// inputs (FlyByWireInterface.cpp:1562-1563, 2108-2109) as
// capt_priority_takeover_pressed / fo_priority_takeover_pressed, which the
// compiled control law uses to resolve a dual sidestick input by locking out
// the other side. `on_command` above only takes a command's begin phase (a
// momentary press); a held pushbutton also needs its end phase, so this is
// a separate pair of commands and handler.
// ---------------------------------------------------------------------------

static PRIORITY_CAPT_HELD: AtomicBool = AtomicBool::new(false);
static PRIORITY_FO_HELD: AtomicBool = AtomicBool::new(false);

/// XPLMCommandPhase: 0 = begin, 1 = continue, 2 = end.
unsafe extern "C" fn on_priority_command(_command: CommandRef, phase: std::ffi::c_int, refcon: *mut std::ffi::c_void) -> std::ffi::c_int {
    let held = match refcon as usize {
        0 => &PRIORITY_CAPT_HELD,
        _ => &PRIORITY_FO_HELD,
    };
    if phase == 0 {
        held.store(true, Ordering::Relaxed);
    } else if phase == 2 {
        held.store(false, Ordering::Relaxed);
    }
    1
}

/// This tick's priority takeover pushbutton state: (captain held, first
/// officer held).
pub fn priority_takeover_held() -> (bool, bool) {
    (PRIORITY_CAPT_HELD.load(Ordering::Relaxed), PRIORITY_FO_HELD.load(Ordering::Relaxed))
}

/// The two priority takeover commands, held (not pulsed) while pressed.
pub struct PriorityTakeoverCommands {
    registered: Vec<(CommandRef, usize)>,
}

unsafe impl Send for PriorityTakeoverCommands {}

impl PriorityTakeoverCommands {
    pub fn register(xplm: &Xplm) -> Self {
        let mut registered = Vec::new();
        for (i, suffix) in ["A32NX_PRIORITY_TAKEOVER_CAPT", "A32NX_PRIORITY_TAKEOVER_FO"].into_iter().enumerate() {
            let name = format!("fbw/event/{suffix}");
            if let Some(command) = xplm.create_command(&name, "FlyByWire sidestick priority takeover (hold)") {
                xplm.register_command_handler(command, on_priority_command, i as *mut std::ffi::c_void);
                registered.push((command, i));
            }
        }
        Self { registered }
    }

    pub fn release(&mut self, xplm: &Xplm) {
        for (command, i) in self.registered.drain(..) {
            xplm.unregister_command_handler(command, on_priority_command, i as *mut std::ffi::c_void);
        }
        // A reload should not leave the button latched held.
        PRIORITY_CAPT_HELD.store(false, Ordering::Relaxed);
        PRIORITY_FO_HELD.store(false, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod priority_takeover_tests {
    use super::*;

    #[test]
    fn begin_holds_and_end_releases() {
        // Simulate begin/continue/end without going through X-Plane's
        // command API: call the handler directly, as X-Plane would.
        unsafe {
            on_priority_command(std::ptr::null_mut(), 0, 0 as *mut std::ffi::c_void);
        }
        assert_eq!(priority_takeover_held(), (true, false));
        unsafe {
            on_priority_command(std::ptr::null_mut(), 1, 0 as *mut std::ffi::c_void);
        }
        // Continue must not clear it.
        assert_eq!(priority_takeover_held(), (true, false));
        unsafe {
            on_priority_command(std::ptr::null_mut(), 2, 0 as *mut std::ffi::c_void);
        }
        assert_eq!(priority_takeover_held(), (false, false));

        // The first officer's command (refcon 1) is independent.
        unsafe {
            on_priority_command(std::ptr::null_mut(), 0, 1 as *mut std::ffi::c_void);
        }
        assert_eq!(priority_takeover_held(), (false, true));
        unsafe {
            on_priority_command(std::ptr::null_mut(), 2, 1 as *mut std::ffi::c_void);
        }
        assert_eq!(priority_takeover_held(), (false, false));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The gap that left the ND frozen: every other FCU knob had an event
    /// wired to it, and the two EFIS ones did not, so
    /// `efis_mode_knob_turns`/`efis_range_knob_turns` stayed 0 for the whole
    /// flight and the FCU never moved the EFIS discrete word `prim.rs`
    /// publishes the ND mode and range from.
    #[test]
    fn turning_an_efis_knob_reaches_the_fcu_as_a_detent() {
        for (side, name) in [(0, "captain"), (1, "first officer")] {
            let mut inputs = EventInputs::default();
            assert_eq!(inputs.efis[side].efis_mode_knob_turns, 0, "{name}: starts still");
            assert_eq!(inputs.efis[side].efis_range_knob_turns, 0, "{name}: starts still");

            inputs.apply(Event::FcuEfisModeTurn(side, 1));
            inputs.apply(Event::FcuEfisRangeTurn(side, -1));
            assert_eq!(inputs.efis[side].efis_mode_knob_turns, 1, "{name}: mode knob turned clockwise");
            assert_eq!(inputs.efis[side].efis_range_knob_turns, -1, "{name}: range knob turned anticlockwise");

            // One side's knob must never move the other side's ND.
            let other = 1 - side;
            assert_eq!(inputs.efis[other].efis_mode_knob_turns, 0, "{name}: the other side stayed put");
            assert_eq!(inputs.efis[other].efis_range_knob_turns, 0, "{name}: the other side stayed put");
        }
    }

    /// The knobs must send *detents*, never the published value itself.
    ///
    /// `prim.rs` publishes the ND's scale (0 = ZOOM, 1 = 10 NM, 2 = 20) and
    /// the FCU's own input is `a380_efis_range_selection` (0 = ZOOM 0.2, ...
    /// 5 = RANGE_10). Feeding one into the other lands five steps low, deep
    /// in the ZOOM band -- which is exactly where the ND draws the OANS
    /// airport map over ARC, NAV and PLAN and shows NOT AVAIL with no
    /// airport database loaded. A difference needs no shared origin, so
    /// that is what is sent.
    #[test]
    fn turning_past_zoom_steps_through_the_airport_map_ranges() {
        let steps = [(1.0, 5.0), (0.0, 5.0), (0.0, 3.0), (0.0, 0.0), (0.0, 1.0), (0.0, 4.0), (1.0, 4.0), (7.0, 4.0)];
        let scale: Vec<f64> = steps.iter().map(|&(nd, oans)| range_selection(nd, oans)).collect();
        assert_eq!(scale, vec![5.0, 4.0, 3.0, 0.0, 1.0, 4.0, 5.0, 11.0]);
    }

    #[test]
    fn a_turn_becomes_a_detent_in_the_direction_it_moved() {
        let mut inputs = EventInputs::default();
        assert_eq!(inputs.efis[0].efis_range_knob_turns, 0);

        inputs.apply(Event::FcuEfisRangeTurn(0, 1));
        assert_eq!(inputs.efis[0].efis_range_knob_turns, 1, "one step up is one detent clockwise");
        inputs.apply(Event::FcuEfisRangeTurn(0, -1));
        assert_eq!(inputs.efis[0].efis_range_knob_turns, -1);

        inputs.apply(Event::FcuEfisModeTurn(1, 2));
        assert_eq!(inputs.efis[1].efis_mode_knob_turns, 2, "two steps is two detents");
        assert_eq!(inputs.efis[0].efis_mode_knob_turns, 0, "and the other side is untouched");
    }

    /// Every knob event must be reachable both ways it can arrive: as an
    /// X-Plane command the converted cockpit binds, and by the MSFS event
    /// name a script sends.
    #[test]
    fn the_baro_knob_turns_pushes_and_pulls_each_side_and_x_planes_own_commands_reach_it() {
        let mut inputs = EventInputs::default();
        inputs.apply(Event::FcuEfisBaroTurn(1, -1));
        inputs.apply(Event::FcuEfisBaroPull(0));
        assert_eq!(inputs.efis[1].baro_knob.turns, -1);
        assert_eq!(inputs.efis[0].baro_knob.turns, 0);
        assert_eq!(inputs.efis[0].baro_knob.pulled, 1);
        assert_eq!(inputs.efis[1].baro_knob.pushed, 0);
        for (command, event) in [
            ("sim/instruments/barometer_up", Event::FcuEfisBaroTurn(0, 1)),
            ("sim/instruments/barometer_copilot_down", Event::FcuEfisBaroTurn(1, -1)),
            ("sim/instruments/barometer_std", Event::FcuEfisBaroPull(0)),
        ] {
            assert!(COMMANDS.iter().any(|(s, _, e)| *s == command && *e == event), "{command}");
        }
        assert_eq!(crate::key_events::afs_event_for_test("A32NX.FCU_EFIS_R_BARO_PUSH"), Some(Event::FcuEfisBaroPush(1)));
    }

    #[test]
    fn both_efis_knobs_are_bound_on_both_sides() {
        for side in ["L", "R"] {
            for kind in ["MODE", "RANGE", "BARO"] {
                for way in ["INC", "DEC"] {
                    let msfs = format!("A32NX.FCU_EFIS_{side}_{kind}_{way}");
                    let suffix = format!("A32NX_FCU_EFIS_{side}_{kind}_{way}");
                    assert!(
                        COMMANDS.iter().any(|(s, m, _)| *s == suffix && *m == msfs),
                        "{msfs} has no fbw/event/ command"
                    );
                    assert!(
                        crate::key_events::afs_event_for_test(&msfs).is_some(),
                        "{msfs} cannot be sent by name from a script"
                    );
                }
            }
        }
    }

    #[test]
    fn every_efis_pushbutton_reaches_the_fcu_on_its_own_side_by_name_and_by_command() {
        let per_side = EfisButton::ALL.len();
        for (side, s) in ["L", "R"].into_iter().enumerate() {
            for (j, &(button, name)) in EfisButton::ALL.iter().enumerate() {
                let event = Event::FcuEfisButton(side, button);
                assert_eq!(crate::key_events::afs_event_for_test(&format!("A32NX.FCU_EFIS_{s}_{name}_PUSH")), Some(event));
                assert_eq!(efis_button_command(COMMANDS.len() + side * per_side + j), Some(event));
                let mut inputs = EventInputs::default();
                inputs.apply(event);
                let pushed = |e: &BaseFcuEfisPanelInputs| {
                    [
                        e.vv_button_pushed,
                        e.ls_button_pushed,
                        e.taxi_button_pushed,
                        e.cstr_button_pushed,
                        e.wpt_button_pushed,
                        e.vord_button_pushed,
                        e.ndb_button_pushed,
                        e.arpt_button_pushed,
                        e.navaid_1_button_pushed,
                        e.navaid_2_button_pushed,
                        e.wx_button_pushed,
                        e.terr_button_pushed,
                        e.traf_button_pushed,
                    ]
                };
                let mut expected = [0u8; 13];
                expected[j] = 1;
                assert_eq!(pushed(&inputs.efis[side]), expected, "{s} {name}");
                assert_eq!(pushed(&inputs.efis[1 - side]), [0u8; 13], "{s} {name} left the other side alone");
            }
        }
        assert_eq!(efis_button_command(COMMANDS.len() + 2 * per_side), None);
        assert_eq!(efis_button_command(0), None);
        assert_eq!(efis_button_event("A32NX.FCU_EFIS_L_BARO_PUSH"), None);
    }

    #[test]
    fn the_converted_cockpits_own_button_writes_are_read_as_pushes() {
        use EfisButton::*;
        let ls = ButtonWatch::Toggle(Ls);
        assert_eq!(ls.pushed(1., 0.), Some(Ls));
        assert_eq!(ls.pushed(0., 1.), Some(Ls));
        assert_eq!(ls.pushed(1., 1.), None, "the FCU's own value read back is no push");
        let filter = ButtonWatch::Filter;
        assert_eq!(filter.pushed(0., 2.), Some(Vord));
        assert_eq!(filter.pushed(1., 3.), Some(Ndb), "another filter chosen over WPT");
        assert_eq!(filter.pushed(1., 0.), Some(Wpt), "WPT pressed again to clear it");
        let overlay = ButtonWatch::Overlay;
        assert_eq!(overlay.pushed(0., 1.), Some(Wx));
        assert_eq!(overlay.pushed(2., 0.), Some(Terr));
        let taxi = ButtonWatch::Pressed(Taxi);
        assert_eq!(taxi.pushed(0., 1.), Some(Taxi));
        assert_eq!(taxi.pushed(1., 0.), None, "letting go is not a second push");
    }
}
