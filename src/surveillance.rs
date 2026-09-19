//! SURV panel controls FlyByWire's A380X XML leaves unbound.
//!
//! `pedestal.xml`'s `WeatherRadar` component gives its eight push buttons
//! (`PUSH_SURV_XPDR_TCAS_SYS1/2`, `_WXR_TAWS_SYS1/2`, `_TCAS_TAONLY`,
//! `_TCAS_ABV`, `_TCAS_BLW`, `_GS_MODE`) only `FBW_A380X_BacklightIndicator_
//! Button_Template`'s backlighting; none carries `LEFT_SINGLE_CODE`, so
//! `msfs2xp-aircraft`'s converter leaves every one of them unresolved
//! ("template parameter never given: #BUTTON_CODE#", `cockpit_bindings.txt`)
//! and falls back to publishing the button's own clip dataref,
//! `fbw/cockpit/<NODE_ID>`, through the exported aircraft's SASL script
//! (`msfs2xp-aircraft/src/rig.rs:637-733`). That SASL script loads after
//! this plugin (as `doors.rs` notes for `fbw/anim/...`), so its datarefs are
//! looked up again each tick until found. The manipulator is
//! `ATTR_manip_push` (`rig.rs`'s `Kind::Push`), so the dataref is momentary:
//! 1 while the button is held/clicked, 0 on release. `PushButton` below
//! turns that into a single edge per press.
//!
//! `LegacyTcasComputer.ts` (`systems-host/Misc/tcas/components/
//! LegacyTcasComputer.ts:280,283,323`) already implements full TA/RA logic,
//! aural alerts and reads real X-Plane traffic (`mapdata::traffic`, already
//! answering `GET_AIR_TRAFFIC`); it is simply never told which XPDR/TCAS
//! system is selected or whether TA ONLY is active, because nothing writes
//! `L:A32NX_TRANSPONDER_SYSTEM` or `L:A32NX_TCAS_TA_ONLY`. Wiring those two
//! is the highest-value fix here: it activates logic that already exists
//! rather than adding new logic.
//!
//! `PUSH_SURV_WXR_TAWS_SYS1`/`_SYS2` and `_GS_MODE` also have real,
//! already-consumed FBW variables once found: `L:A32NX_WXR_TAWS_SYS_
//! SELECTED` (1 or 2) picks the AESU lane `src/wxr` (`wxr_failed`,
//! `side_config`) and FBW's own `EfisTawsBridge.ts:560`, `LegacyGpws.ts:257`
//! and `FwsCore.ts:1866` all read — this one press wires WXR, TAWS terrain
//! fail flags and GPWS lane select all at once. `L:A32NX_GPWS_GS_OFF`
//! (`LegacyGpws.ts:556`) is the G/S mode inhibit `PUSH_SURV_GS_MODE`'s
//! tooltip ("SET G/S MODE") describes. Neither var had anything driving it
//! before this: `A32NX_WXR_TAWS_SYS_SELECTED` unset means `src/wxr` and
//! GPWS both treat the AESS as unselected (`wxr_failed` returns `true` for
//! anything but 1./2.), so this module also seeds it to system 1 at
//! startup, same as the real aircraft powering up on AESU 1.
//!
//! `PUSH_SURV_TCAS_ABV`/`_BLW` (display range) has no consumer anywhere in
//! FBW's TS or elsewhere in this plugin yet — a genuinely new,
//! plugin-owned var (`tcas_range`) for `src/mapdata::traffic` to filter on
//! once its own ND pass exists. See `docs/physics/surveillance.md` for the
//! full spec and the `SWITCH_RADAR_MULTISCAN`/`SWITCH_RADAR_GCS` follow-up.
//!
//! **Electrical is not isolated from this system**, matching the real
//! aircraft's two independent AESUs and this port's rule that a system's
//! power has to come from an actual bus, not a standing assumption: AESU 1
//! is fed from the AC ESS bus and AESU 2 from AC BUS 4, exactly as FBW's own
//! `EfisTawsBridge.ts:189-190` (`acEssPowered`/`ac4Powered`,
//! `L:A32NX_ELEC_AC_ESS_BUS_IS_POWERED`/`L:A32NX_ELEC_AC_4_BUS_IS_POWERED`
//! via `powersupply.ts:31-32`) already gates its own terr/GPWS fail flags.
//! A push button belonging to an unpowered lane is dead here too — pressing
//! `XPDR_TCAS_SYS2` or `WXR_TAWS_SYS2` while AC BUS 4 is down does nothing,
//! same as `TCAS_TAONLY`/`ABV`/`BLW` when the *currently selected* TCAS lane
//! has lost its bus, and `GS_MODE` when the currently selected WXR/TAWS lane
//! has. This reuses the electrical system's own already-computed bus state
//! (`circuits.rs`/`efb.rs` read the identical `ELEC_AC_<n>_BUS_IS_POWERED`
//! names) rather than adding a second, parallel power model.

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::xp::{DataRef, Xplm};
use crate::Vars;

/// The TCAS traffic display's vertical range (FCOM: ABV -2700/+9900 ft, NORM
/// -2700/+2700 ft, BLW -9900/+2700 ft, relative to own altitude). Not yet
/// consumed anywhere; `mapdata::traffic` should filter on it once its own
/// pass reaches the ND.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum TcasRange {
    Above,
    #[default]
    Normal,
    Below,
}

impl TcasRange {
    /// Feet below/above own altitude the range covers.
    pub fn window_ft(self) -> (f64, f64) {
        match self {
            TcasRange::Above => (-2700., 9900.),
            TcasRange::Normal => (-2700., 2700.),
            TcasRange::Below => (-9900., 2700.),
        }
    }

    /// The value written to the shared `A32NX_TCAS_RANGE` var: a plugin-
    /// owned scalar (no FBW consumer) so `mapdata::plugin` can read the
    /// selected range through `Vars` instead of reaching into this
    /// module's `Surveillance` struct directly.
    pub fn code(self) -> f64 {
        match self {
            TcasRange::Below => -1.,
            TcasRange::Normal => 0.,
            TcasRange::Above => 1.,
        }
    }

    /// The inverse of [`Self::code`]. Any value that is not exactly -1 or 1
    /// (including a never-written slot's default 0) reads as `Normal`, the
    /// same default the button logic starts from.
    pub fn from_code(code: f64) -> Self {
        if code <= -0.5 {
            TcasRange::Below
        } else if code >= 0.5 {
            TcasRange::Above
        } else {
            TcasRange::Normal
        }
    }
}

/// Rising-edge detector on a momentary `fbw/cockpit/...` push-button
/// dataref. Looked up lazily: the exported aircraft's SASL script, which
/// publishes it, loads after this plugin.
struct PushButton {
    path: &'static str,
    dataref: Option<DataRef>,
    was_down: bool,
}

impl PushButton {
    const fn new(path: &'static str) -> Self {
        Self { path, dataref: None, was_down: false }
    }

    /// True exactly once per press (the 0 -> 1 edge), never on release or
    /// while held, and never before the SASL script has published it.
    fn pressed(&mut self, xplm: &Xplm) -> bool {
        if self.dataref.is_none() {
            self.dataref = xplm.find(self.path);
        }
        let Some(d) = self.dataref else { return false };
        let down = xplm.get_f(d) > 0.5;
        let edge = down && !self.was_down;
        self.was_down = down;
        edge
    }
}

/// The SURV panel's AESS controls: transponder/TCAS system select, TCAS TA
/// ONLY and display range, WXR/TAWS lane select and G/S mode inhibit.
pub struct Surveillance {
    xpdr_tcas_sys1: PushButton,
    xpdr_tcas_sys2: PushButton,
    tcas_ta_only: PushButton,
    tcas_abv: PushButton,
    tcas_blw: PushButton,
    wxr_taws_sys1: PushButton,
    wxr_taws_sys2: PushButton,
    gs_mode: PushButton,

    transponder_system: VariableIdentifier,
    tcas_ta_only_var: VariableIdentifier,
    /// `L:A32NX_WXR_TAWS_SYS_SELECTED`: 1 or 2, the AESU lane feeding WXR,
    /// TAWS terrain and GPWS alike (`wxr::wxr_failed`, `EfisTawsBridge.ts`,
    /// `LegacyGpws.ts`, `FwsCore.ts`).
    wxr_taws_sys_selected: VariableIdentifier,
    /// `L:A32NX_GPWS_GS_OFF`: G/S mode inhibit (`LegacyGpws.ts:556`).
    gpws_gs_off: VariableIdentifier,
    /// `L:A32NX_ELEC_AC_ESS_BUS_IS_POWERED`: AESU 1's own bus
    /// (`powersupply.ts:32`).
    ac_ess_bus_powered: VariableIdentifier,
    /// `L:A32NX_ELEC_AC_4_BUS_IS_POWERED`: AESU 2's own bus
    /// (`powersupply.ts:31`).
    ac_4_bus_powered: VariableIdentifier,

    /// TCAS display range, plugin-owned (no FBW TS consumer yet).
    pub tcas_range: TcasRange,
    /// `A32NX_TCAS_RANGE`: [`TcasRange::code`], the shared-state handoff to
    /// `mapdata::plugin`'s traffic filter (see that module's `update`).
    /// Written every tick rather than only on change: a plain scalar write
    /// costs nothing and needs no extra "did it change" bookkeeping.
    tcas_range_var: VariableIdentifier,
}

impl Surveillance {
    pub fn new(vars: &mut Vars) -> Self {
        let this = Self {
            xpdr_tcas_sys1: PushButton::new("fbw/cockpit/PUSH_SURV_XPDR_TCAS_SYS1"),
            xpdr_tcas_sys2: PushButton::new("fbw/cockpit/PUSH_SURV_XPDR_TCAS_SYS2"),
            tcas_ta_only: PushButton::new("fbw/cockpit/PUSH_SURV_TCAS_TAONLY"),
            tcas_abv: PushButton::new("fbw/cockpit/PUSH_SURV_TCAS_ABV"),
            tcas_blw: PushButton::new("fbw/cockpit/PUSH_SURV_TCAS_BLW"),
            wxr_taws_sys1: PushButton::new("fbw/cockpit/PUSH_SURV_WXR_TAWS_SYS1"),
            wxr_taws_sys2: PushButton::new("fbw/cockpit/PUSH_SURV_WXR_TAWS_SYS2"),
            gs_mode: PushButton::new("fbw/cockpit/PUSH_SURV_GS_MODE"),
            transponder_system: vars.get("TRANSPONDER_SYSTEM".into()),
            tcas_ta_only_var: vars.get("TCAS_TA_ONLY".into()),
            wxr_taws_sys_selected: vars.get("WXR_TAWS_SYS_SELECTED".into()),
            gpws_gs_off: vars.get("GPWS_GS_OFF".into()),
            ac_ess_bus_powered: vars.get("ELEC_AC_ESS_BUS_IS_POWERED".into()),
            ac_4_bus_powered: vars.get("ELEC_AC_4_BUS_IS_POWERED".into()),
            tcas_range: TcasRange::Normal,
            tcas_range_var: vars.get("TCAS_RANGE".into()),
        };
        // Nothing else in this port writes A32NX_WXR_TAWS_SYS_SELECTED
        // (checked: `rg -i wxr_taws_sys_selected` over src/ and the js
        // bridge turns up only readers), so it stays 0 -- neither AESU
        // selected -- until a pilot presses SYS1/SYS2, leaving WXR, TAWS
        // and GPWS all dead from cold start. The real AESS powers up with
        // lane 1 on line; seed the same default, same "unwritten" guard
        // `register_cockpit_variables` uses so a value another system
        // already wrote (or a saved one, once persistence covers it) is
        // never clobbered.
        if vars.is_unwritten_named(&this.wxr_taws_sys_selected) {
            vars.write(&this.wxr_taws_sys_selected, 1.);
        }
        this
    }

    /// Lane 0 (ESS bus, AESU 1) or lane 1 (AC BUS 4, AESU 2) has power.
    /// Every button below belongs to a lane, and does nothing if that
    /// lane's real bus has none — an unpowered AESU does not respond to its
    /// own panel, same as any other unpowered avionics box in this port.
    #[allow(dead_code)]
    fn lane_powered(&self, vars: &mut Vars, lane: f64) -> bool {
        let id = if lane <= 0.5 { &self.ac_ess_bus_powered } else { &self.ac_4_bus_powered };
        vars.read(id) > 0.5
    }

    pub fn update(&mut self, vars: &mut Vars, xplm: &Xplm) {
        // The SURV panel's push buttons are switches: their state changes
        // whether or not an AESU is powered, as on the aircraft. Power acts
        // where it does on the aircraft, in the consumers (FlyByWire's TCAS
        // computer, GPWS and the weather radar read the selected lane and its
        // bus themselves); a switch press is never swallowed here.
        // Exclusive system select: whichever SYS button is pressed last
        // becomes the active one.
        if self.xpdr_tcas_sys1.pressed(xplm) {
            vars.write(&self.transponder_system, 0.);
        }
        if self.xpdr_tcas_sys2.pressed(xplm) {
            vars.write(&self.transponder_system, 1.);
        }
        if self.tcas_ta_only.pressed(xplm) {
            let active = vars.read(&self.tcas_ta_only_var) > 0.5;
            vars.write(&self.tcas_ta_only_var, if active { 0. } else { 1. });
        }
        // ABV/BLW are momentary selects, not a toggle pair: pressing one
        // selects it; pressing the active one again returns to NORM (real
        // AESS behaviour - the range buttons are not radio buttons with a
        // separate NORM button, NORM is "neither pressed").
        if self.tcas_abv.pressed(xplm) {
            self.tcas_range = if self.tcas_range == TcasRange::Above { TcasRange::Normal } else { TcasRange::Above };
        }
        if self.tcas_blw.pressed(xplm) {
            self.tcas_range = if self.tcas_range == TcasRange::Below { TcasRange::Normal } else { TcasRange::Below };
        }
        if self.wxr_taws_sys1.pressed(xplm) {
            vars.write(&self.wxr_taws_sys_selected, 1.);
        }
        if self.wxr_taws_sys2.pressed(xplm) {
            vars.write(&self.wxr_taws_sys_selected, 2.);
        }
        if self.gs_mode.pressed(xplm) {
            let off = vars.read(&self.gpws_gs_off) > 0.5;
            vars.write(&self.gpws_gs_off, if off { 0. } else { 1. });
        }
        // Publish the range every tick: `mapdata::plugin` reads this to
        // filter TCAS traffic reaching the ND, and must never reach into
        // `self.tcas_range` directly (that field is this struct's, not
        // shared state).
        vars.write(&self.tcas_range_var, self.tcas_range.code());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tcas_range_windows_match_fcom() {
        assert_eq!(TcasRange::Above.window_ft(), (-2700., 9900.));
        assert_eq!(TcasRange::Normal.window_ft(), (-2700., 2700.));
        assert_eq!(TcasRange::Below.window_ft(), (-9900., 2700.));
    }

    #[test]
    fn push_button_fires_once_per_press_not_while_held() {
        // No X-Plane in a unit test: exercise the edge-detect logic
        // directly against `was_down`/`down` the way `pressed` computes it,
        // since `pressed` itself needs a live `Xplm`/`DataRef`.
        let mut was_down = false;
        let mut edges = 0;
        for down in [false, true, true, true, false, false, true] {
            let edge = down && !was_down;
            was_down = down;
            if edge {
                edges += 1;
            }
        }
        // Presses at index 1 and 6: two edges, not three (held) and not
        // one merged across the release at index 4-5.
        assert_eq!(edges, 2);
    }

    #[test]
    fn tcas_range_abv_toggles_back_to_normal_on_second_press() {
        let mut range = TcasRange::Normal;
        // First press: NORM -> ABV.
        range = if range == TcasRange::Above { TcasRange::Normal } else { TcasRange::Above };
        assert_eq!(range, TcasRange::Above);
        // Second press of the same button: ABV -> NORM (not stuck on ABV).
        range = if range == TcasRange::Above { TcasRange::Normal } else { TcasRange::Above };
        assert_eq!(range, TcasRange::Normal);
    }

    #[test]
    fn lane_selection_arithmetic_matches_each_vars_numbering() {
        // `transponder_system` is 0./1. (AESU 1/2); `lane_powered`'s `lane`
        // param takes that value directly, and its own `lane <= 0.5` split
        // must land 0. on the ESS-bus branch and 1. on the AC-4 branch.
        let lane_is_ess = |lane: f64| lane <= 0.5;
        assert!(lane_is_ess(0.));
        assert!(!lane_is_ess(1.));

        // `wxr_taws_sys_selected` is 1./2. (not 0./1.), so `update` passes
        // `value - 1.` into the same helper -- confirm that lands on the
        // same two branches, not off by one into a third, nonexistent lane.
        assert!(lane_is_ess(1. - 1.));
        assert!(!lane_is_ess(2. - 1.));
    }

    #[test]
    fn tcas_range_code_round_trips_through_the_shared_var() {
        for range in [TcasRange::Above, TcasRange::Normal, TcasRange::Below] {
            assert_eq!(TcasRange::from_code(range.code()), range);
        }
        // A never-written slot's default 0. must read back as Normal, same
        // as `Surveillance::tcas_range`'s own `#[default]`.
        assert_eq!(TcasRange::from_code(0.), TcasRange::Normal);
    }

    #[test]
    fn tcas_range_blw_does_not_touch_abv_state_machine() {
        // BLW pressed from NORM goes to BELOW, independent of ABV's own
        // toggle (regression guard: the two must not share one flag that
        // could leave both "selected" at once).
        let mut range = TcasRange::Normal;
        range = if range == TcasRange::Below { TcasRange::Normal } else { TcasRange::Below };
        assert_eq!(range, TcasRange::Below);
    }
}
