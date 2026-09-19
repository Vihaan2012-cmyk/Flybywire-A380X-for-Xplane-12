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
use crate::xp::{CommandRef, Xplm};

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

unsafe extern "C" fn on_command(_command: CommandRef, phase: std::ffi::c_int, refcon: *mut std::ffi::c_void) -> std::ffi::c_int {
    // A key event is one press: take the command's begin phase.
    if phase == 0 {
        if let Some(&(_, _, event)) = COMMANDS.get(refcon as usize) {
            send(event);
        }
    }
    1
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
            let name = format!("fbw/event/{suffix}");
            if let Some(command) = xplm.create_command(&name, &format!("FlyByWire event {msfs}")) {
                xplm.register_command_handler(command, on_command, i as *mut std::ffi::c_void);
                registered.push((command, i));
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
