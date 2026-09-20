//! Electronic Checklist (ECL): the ECAM control panel's inputs, and the
//! normal-checklist sensing that was wrong.
//!
//! `docs/ecl.md` has the full investigation, the per-checklist comparison
//! against real A380 content and the sources for every change here. In short:
//!
//! FlyByWire's ECL machinery (`FwsNormalChecklists.ts`, `WdNormalChecklists.tsx`)
//! already runs and draws in this port -- SystemsHost and the EWD both boot
//! ("xphfbw view 15 loaded: A380X_SYSTEMSHOST", "view 1 loaded: A380X_EWD").
//! What did not work was the crew's side of it: the converted cockpit's ECAM
//! control panel buttons.
//!
//! FlyByWire's `ecam-cp.xml` gives every ECP pushbutton a `LEFT_SINGLE_CODE`
//! (`1 (>L:A32NX_BTN_<name>)`) *and* a `LEFT_LEAVE_CODE` (`0 (>L:...)`) --
//! a momentary button. The converted aircraft's SASL bindings
//! (`<aircraft>/plugins/sasl/data/modules/main.lua`, `do -- PUSH_ECAM_CL`)
//! carry only the press: every one of those `local function release()` bodies
//! is empty, so the variable latches at 1 and never returns to 0.
//! `FwsCore.update` feeds each button through an `NXLogicMemoryNode` and then
//! an `NXLogicPulseNode` (`clPulseNode` etc.), which fire on the *rising* edge
//! only -- so a latched-at-1 button produces exactly one action per session
//! and then nothing. Opening the ECL once and never being able to close it,
//! tick an item or move the cursor again is precisely that.
//!
//! The same conversion also lost the `<Condition NotEmpty="SIMVAR">` branch of
//! `FBW_ECAM_BUTTON_SubTemplate`, so three buttons write the fallback
//! `A32NX_BTN_#BASE_NAME#` name instead of the `#SIMVAR#` one FwsCore reads:
//!
//! | cockpit writes (main.lua) | FwsCore.ts reads | |
//! |---|---|---|
//! | `A32NX_BTN_TOCONF`   | `A32NX_BTN_TOCONFIG` | mismatch |
//! | `A32NX_BTN_CLR_LH`   | `A32NX_BTN_CLR`      | mismatch |
//! | `A32NX_BTN_CLR_RH`   | `A32NX_BTN_CLR2`     | mismatch |
//! | `A32NX_BTN_RCLLAST`  | `A32NX_BTN_RCL`      | mismatch (RCL LAST only) |
//! | `A32NX_BTN_CL`, `_CHECK_LH`, `_CHECK_RH`, `_UP`, `_DOWN`, `_ABNPROC`, `_RCL` | same | match |
//!
//! T.O CONFIG is an ECL matter too: the TAXI checklist's last sensed item is
//! `T.O CONFIG ... TEST/NORM`, fed by `toConfigNormal`, which only ever
//! becomes true after the T.O CONFIG TEST pushbutton is pressed. With the
//! name mismatch that item could never tick.
//!
//! Both are fixed in one patch over FwsCore's own ECP acquisition block
//! ([`ecl_ecp_buttons_are_momentary`]): read every name the cockpit might
//! write, and zero each one that read 1 -- exactly what `LEFT_LEAVE_CODE`
//! does in MSFS. Fixing it here rather than in the converter keeps
//! `D:\msfs2xp-aircraft` out of this workstream and works against the
//! already-installed aircraft.

use crate::js::msfs::SourcePatch;

const SYSTEMS_HOST: &str = "/Pages/VCockpit/Instruments/A380X/SystemsHost/SystemsHost.js";

pub(crate) fn source_patches() -> Vec<SourcePatch> {
    vec![ecl_ecp_buttons_are_momentary(), ecl_rudder_trim_neutral_is_signed()]
}

/// The ECAM control panel's pushbuttons, made momentary again, and read under
/// every name the converted cockpit writes.
///
/// Source for the intended behaviour: FlyByWire's own
/// `fbw-a380x/src/base/.../model/behaviour/ecam-cp.xml`,
/// `FBW_ECAM_BUTTON_SubTemplate` / `A32NX_ECAM_CLR_BUTTON_Template` /
/// `A32NX_ECAM_MORE_BUTTON_Template`, each of which pairs
/// `<LEFT_SINGLE_CODE>1 (&gt;L:...)</LEFT_SINGLE_CODE>` with
/// `<LEFT_LEAVE_CODE>0 (&gt;L:...)</LEFT_LEAVE_CODE>`; and FlyByWire's own
/// on-screen ECL soft keys (`EWD/elements/EclSoftKeys.tsx`), which do the same
/// thing explicitly -- `SetSimVarValue('L:A32NX_BTN_CHECK_LH', .., 1)` then
/// `setTimeout(() => SetSimVarValue(.., 0), 50)`.
///
/// Source for the alternative names: the converted aircraft's own
/// `plugins/sasl/data/modules/main.lua` (`wr("fbw/A32NX_BTN_TOCONF", ...)`,
/// `"fbw/A32NX_BTN_CLR_LH"`, `"fbw/A32NX_BTN_CLR_RH"`,
/// `"fbw/A32NX_BTN_RCLLAST"`), cross-checked against
/// `cockpit_bindings.txt` (`PUSH_ECAM_*: click command running the control's
/// MSFS code in SASL`).
///
/// Holding the button down still produces exactly one action, as it did
/// before and as it does in MSFS: the memory node feeding `clPulseNode` is
/// reset at the end of every FWS cycle and the pulse node only fires on a
/// rising edge.
fn ecl_ecp_buttons_are_momentary() -> SourcePatch {
    SourcePatch {
        path: SYSTEMS_HOST.to_string(),
        find: r#"      if (SimVar.GetSimVarValue("L:A32NX_BTN_TOCONFIG", "bool") && !this.fwsEcpFailed.get()) {
        this.toConfigInputBuffer.write(true, false);
      }
      const clearButtonLeft = SimVar.GetSimVarValue("L:A32NX_BTN_CLR", "bool");
      const clearButtonRight = SimVar.GetSimVarValue("L:A32NX_BTN_CLR2", "bool");
      if (clearButtonLeft || clearButtonRight) {
        this.clearButtonInputBuffer.write(true, false);
      }
      const recallButton = SimVar.GetSimVarValue("L:A32NX_BTN_RCL", "bool");
      if (recallButton && !this.fwsEcpFailed.get()) {
        this.recallButtonInputBuffer.write(true, false);
      }
      if (SimVar.GetSimVarValue("L:A32NX_BTN_CL", "bool")) {
        this.clInputBuffer.write(true, false);
      }
      if (SimVar.GetSimVarValue("L:A32NX_BTN_CHECK_LH", "bool") || SimVar.GetSimVarValue("L:A32NX_BTN_CHECK_RH", "bool")) {
        this.clCheckInputBuffer.write(true, false);
      }
      if (SimVar.GetSimVarValue("L:A32NX_BTN_UP", "bool")) {
        this.clUpInputBuffer.write(true, false);
      }
      if (SimVar.GetSimVarValue("L:A32NX_BTN_DOWN", "bool")) {
        this.clDownInputBuffer.write(true, false);
      }
      if (SimVar.GetSimVarValue("L:A32NX_BTN_ABNPROC", "bool")) {
        this.abnProcInputBuffer.write(true, false);
      }
"#
        .to_string(),
        replace: r#"      const ecpButtonPressed = (...names) => {
        let pressed = false;
        for (const btnName of names) {
          if (SimVar.GetSimVarValue("L:" + btnName, "bool")) {
            pressed = true;
            SimVar.SetSimVarValue("L:" + btnName, "bool", 0);
          }
        }
        return pressed;
      };
      if (ecpButtonPressed("A32NX_BTN_TOCONFIG", "A32NX_BTN_TOCONF") && !this.fwsEcpFailed.get()) {
        this.toConfigInputBuffer.write(true, false);
      }
      if (ecpButtonPressed("A32NX_BTN_CLR", "A32NX_BTN_CLR_LH", "A32NX_BTN_CLR2", "A32NX_BTN_CLR_RH")) {
        this.clearButtonInputBuffer.write(true, false);
      }
      if (ecpButtonPressed("A32NX_BTN_RCL", "A32NX_BTN_RCLLAST") && !this.fwsEcpFailed.get()) {
        this.recallButtonInputBuffer.write(true, false);
      }
      if (ecpButtonPressed("A32NX_BTN_CL")) {
        this.clInputBuffer.write(true, false);
      }
      if (ecpButtonPressed("A32NX_BTN_CHECK_LH", "A32NX_BTN_CHECK_RH")) {
        this.clCheckInputBuffer.write(true, false);
      }
      if (ecpButtonPressed("A32NX_BTN_UP")) {
        this.clUpInputBuffer.write(true, false);
      }
      if (ecpButtonPressed("A32NX_BTN_DOWN")) {
        this.clDownInputBuffer.write(true, false);
      }
      if (ecpButtonPressed("A32NX_BTN_ABNPROC")) {
        this.abnProcInputBuffer.write(true, false);
      }
"#
        .to_string(),
        reason: "the converted cockpit's ECAM control panel buttons only carry \
                 ecam-cp.xml's LEFT_SINGLE_CODE, never its LEFT_LEAVE_CODE, so each \
                 A32NX_BTN_* latched at 1 and FwsCore's rising-edge pulse nodes fired \
                 once per session (C/L, CHECK, UP, DOWN, ABN PROC dead after the first \
                 click); release them here, and read the three buttons the conversion \
                 named after BASE_NAME instead of SIMVAR (TOCONF, CLR_LH/CLR_RH, RCLLAST)"
            .to_string(),
    }
}

/// AFTER START, `RUDDER TRIM ... NEUTRAL` (a sensed item):
/// `FwsNormalChecklists.ts:545` tests `this.fws.rudderTrimPosition.get() < 0.35`.
/// `rudderTrimPosition` (`FwsCore.ts:4764`) is the SEC's *signed* rudder trim
/// position in degrees, straight out of
/// `Arinc429Word.fromSimVarValue('L:A32NX_SEC_1_RUDDER_ACTUAL_POSITION')`
/// (`FwsCore.ts:4755-4756`; written by
/// `fbw_a380/src/FlyByWireInterface.cpp:647,2139` as
/// `rudder_trim_actual_pos_deg`). Nine lines above the checklist's test,
/// FlyByWire's own rudder-trim-not-in-T.O-config warning takes the absolute
/// value of the very same word -- `Math.abs(sec1RudderTrimActualPos.valueOr(0))
/// > 3.6` (`FwsCore.ts:4761-4762`) -- and so does the ECL's own pre-2020
/// variant, kept commented out at `NormalProceduresBefore2020.ts:448-449`
/// (`Math.abs(... RUDDER_TRIM_1_COMMANDED_POSITION ...) < 0.35 && ...`).
///
/// Without the absolute value every *left* (negative) rudder trim setting
/// satisfies `< 0.35`, so the item ticks itself as NEUTRAL with the trim
/// wound fully left -- a sensed item reporting a state the aircraft is not in.
/// This changes only the comparison, not the 0.35 deg threshold and not the
/// item's text.
fn ecl_rudder_trim_neutral_is_signed() -> SourcePatch {
    SourcePatch {
        path: SYSTEMS_HOST.to_string(),
        find: "whichItemsChecked: () => [null, null, this.fws.rudderTrimPosition.get() < 0.35]".to_string(),
        replace: "whichItemsChecked: () => [null, null, Math.abs(this.fws.rudderTrimPosition.get()) < 0.35]"
            .to_string(),
        reason: "the AFTER START checklist's sensed RUDDER TRIM / NEUTRAL item compared \
                 the SEC's signed trim position against 0.35 deg without Math.abs, so any \
                 left (negative) trim ticked it as neutral; FwsCore.ts:4761-4762 takes the \
                 absolute value of the same word for the rudder-trim T.O warning"
            .to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::{Path, PathBuf};

    /// FlyByWire's built tree, the one `boots_fbw_cockpit_views` loads
    /// (docs/js-build.md).
    const HTML_UI: &str = r"D:\fbw-aircraft\fbw-a380x\out\flybywire-aircraft-a380-842\html_ui";

    /// Every `find` above must still match its built file exactly once, or the
    /// patch runs unchanged and the ECL silently goes back to being
    /// one-click-per-session. Skipped (not failed) where the build is absent,
    /// same as the `--ignored` tests that need it.
    #[test]
    fn every_ecl_patch_matches_its_built_file_exactly_once() {
        let root = Path::new(HTML_UI);
        if !root.is_dir() {
            println!("skipped: {HTML_UI} is not built (docs/js-build.md)");
            return;
        }
        for patch in source_patches() {
            let file = root.join(patch.path.trim_start_matches('/').replace('/', "\\"));
            let text = match std::fs::read_to_string(&file) {
                Ok(t) => t,
                Err(e) => panic!("{}: {e}", file.display()),
            };
            let found = text.matches(patch.find.as_str()).count();
            assert_eq!(found, 1, "{}: the patch ({}) matches {found} times, not once", patch.path, patch.reason);
        }
    }

    /// The bug this workstream was given: pressing C/L on the ECAM control
    /// panel opened the checklist once and then nothing on the panel ever
    /// worked again, because the converted cockpit's SASL binding never
    /// writes the button back to 0 and `FwsCore`'s pulse nodes are
    /// rising-edge only (module doc).
    ///
    /// Drives the real FlyByWire SystemsHost through two C/L "clicks" — each
    /// one setting `L:A32NX_BTN_CL` to 1 and, exactly like the converted
    /// cockpit, never clearing it — and checks the checklist toggles both
    /// times. Needs FlyByWire's built html_ui and the MSFS package's panel
    /// like `boots_fbw_cockpit_views`:
    /// `cargo test --release --features js -- --ignored ecl_opens_and_closes`.
    #[test]
    #[ignore]
    fn ecl_opens_and_closes_on_every_c_l_press() {
        use crate::js::msfs::{Cockpit, CockpitOptions};
        use crate::js::{Engine, Host, LogLevel};
        use std::collections::HashMap;

        struct Store {
            vars: HashMap<String, f64>,
            errors: Vec<String>,
            now_ms: f64,
        }
        impl Host for Store {
            fn get_var(&mut self, name: &str, _unit: &str) -> f64 {
                match name {
                    "E:ABSOLUTE TIME" => 62_135_596_800. + self.now_ms / 1000.,
                    "E:SIMULATION TIME" => self.now_ms / 1000.,
                    _ => *self.vars.get(name).unwrap_or(&0.),
                }
            }
            fn set_var(&mut self, name: &str, _unit: &str, value: f64) {
                self.vars.insert(name.to_string(), value);
            }
            fn log(&mut self, level: LogLevel, message: &str) {
                if level == LogLevel::Error {
                    self.errors.push(message.lines().next().unwrap_or("").to_string());
                }
            }
        }

        let html_ui = PathBuf::from(HTML_UI);
        let panel = PathBuf::from(
            r"D:\Microsoft Flight Simulator 2020\Microsoft Flight Simulator 2020 Packages\Community\flybywire-aircraft-a380-842\SimObjects\AirPlanes\FlyByWire_A380_842\panel",
        );
        let mut options = CockpitOptions::new(html_ui, std::fs::read_to_string(panel.join("panel.cfg")).unwrap());
        options.panel_xml = std::fs::read_to_string(panel.join("panel.xml")).unwrap_or_default();
        options.patches = source_patches();
        let mut cockpit = Cockpit::new(options, &|_engine: &Engine| Ok(())).unwrap();
        let mut host = Store { vars: HashMap::new(), errors: Vec::new(), now_ms: 0. };

        // Powered, CPIOM C available: what `fws_recovers_after_a_cold_and_
        // dark_power_up` establishes the FWS needs before it runs at all.
        // The ECP is only "reachable" while an AFDX route to it is
        // (FwsCore.ts's `ecpNotReachable`).
        for bus in [
            "AC_1", "AC_2", "AC_3", "AC_4", "AC_ESS", "DC_1", "DC_2", "DC_ESS", "DC_HOT_1", "DC_HOT_2",
            "AC_ESS_SHED", "DC_ESS_SHED", "247PP", "108PH",
        ] {
            host.vars.insert(format!("L:A32NX_ELEC_{bus}_BUS_IS_POWERED"), 1.);
        }
        host.vars.insert("L:A32NX_CPIOM_C1_AVAIL".to_string(), 1.);
        host.vars.insert("L:A32NX_CPIOM_C2_AVAIL".to_string(), 1.);
        for route in ["3_3", "13_13", "4_4", "14_14"] {
            host.vars.insert(format!("L:A32NX_AFDX_{route}_REACHABLE"), 1.);
        }

        let mut t = 0.;
        let run_for = |cockpit: &mut Cockpit, host: &mut Store, t: &mut f64, ms: f64| {
            let until = *t + ms;
            while *t < until {
                host.now_ms = *t;
                cockpit.tick(host, *t);
                *t += 50.;
            }
        };
        // Past FwsCore's CONFIG_SELF_TEST_TIME startup (as in
        // `fws_recovers_after_a_cold_and_dark_power_up`).
        run_for(&mut cockpit, &mut host, &mut t, 100_000.);
        assert!(cockpit.all_loaded(), "not every view loaded");

        const SHOWN: &str =
            "JSON.stringify((() => { const sh = document.querySelector('systems-host'); \
             return sh && sh.fwsCore ? sh.fwsCore.normalChecklists.checklistShown.get() : 'no fwsCore'; })())";
        // The EWD holds four `.ProceduresContainer`s, in render order: normal
        // checklists (`EWD.tsx:386`, `WdNormalChecklists`), abnormal sensed
        // (:387), abnormal non-sensed (:394) and the FWS-failed fallback. Only
        // the first is the ECL -- an abnormal procedure showing at the same
        // time must not read as the checklist being open.
        const EWD_ECL: &str = "JSON.stringify((() => { const els = document.querySelectorAll('.ProceduresContainer'); \
                               if (els.length !== 4) { return { display: 'expected 4 containers, got ' + els.length, text: '' }; } \
                               return { display: els[0].style.display, text: (els[0].textContent || '').slice(0, 600) }; })())";
        let shown = |cockpit: &Cockpit| cockpit.eval_in("VCockpit22", SHOWN).unwrap_or_default();
        assert_eq!(shown(&cockpit), "false", "the ECL was already open before any C/L press");

        // Press C/L the way the converted cockpit does: set the variable and
        // never clear it (plugins/sasl/data/modules/main.lua's empty
        // `release()` for `PUSH_ECAM_CL`).
        for press in 1..=2 {
            host.vars.insert("L:A32NX_BTN_CL".to_string(), 1.);
            run_for(&mut cockpit, &mut host, &mut t, 1_000.);
            assert_eq!(
                host.vars.get("L:A32NX_BTN_CL").copied().unwrap_or(0.),
                0.,
                "press {press}: A32NX_BTN_CL was never released, so the next press cannot produce a rising edge"
            );
            let want = if press % 2 == 1 { "true" } else { "false" };
            assert_eq!(shown(&cockpit), want, "press {press}: the ECL did not toggle");

            // ...and the EWD (VCockpit03, `$SCREEN_DU_EWD`) actually draws it:
            // `WdAbstractChecklistComponent.render` puts every checklist line
            // inside a `.ProceduresContainer` whose `display` follows the
            // `visible` prop (`EWD.tsx:386`, `normalChecklistsVisibleNotFailed`).
            let ecl = cockpit.eval_in("VCockpit03", EWD_ECL).unwrap_or_default();
            println!("press {press}: EWD .ProceduresContainer = {ecl}");
            if press % 2 == 1 {
                assert!(ecl.contains("\"display\":\"flex\""), "press {press}: the EWD never showed the ECL: {ecl}");
                assert!(
                    ecl.contains("COCKPIT PREPARATION"),
                    "press {press}: the EWD's ECL drew no checklist menu: {ecl}"
                );
            } else {
                assert!(!ecl.contains("\"display\":\"flex\""), "press {press}: the EWD still shows the ECL: {ecl}");
            }

            if press % 2 == 1 {
                // The rest of the crew's interaction, on the same latched
                // buttons: DOWN moves the cyan box off COCKPIT PREPARATION
                // onto BEFORE START, CHECK opens it. Before the patch neither
                // did anything at all after their own first press.
                for btn in ["L:A32NX_BTN_DOWN", "L:A32NX_BTN_CHECK_LH"] {
                    host.vars.insert(btn.to_string(), 1.);
                    run_for(&mut cockpit, &mut host, &mut t, 1_000.);
                    assert_eq!(host.vars.get(btn).copied().unwrap_or(0.), 0., "{btn} was never released");
                }
                let open = cockpit.eval_in("VCockpit03", EWD_ECL).unwrap_or_default();
                println!("after DOWN + CHECK: EWD .ProceduresContainer = {open}");
                assert!(
                    open.contains("PARKING BRAKE"),
                    "DOWN then CHECK did not open the BEFORE START checklist (its first item is PARKING BRAKE): {open}"
                );
            }
        }

        // Only this workstream's two views: the OIT is out of scope for this
        // port (team.md) and fails to import on its own, and the runaway-script
        // watchdog's "interrupted" is a timing artefact of the harness
        // (`js/mod.rs`'s `a_runaway_script_is_interrupted`).
        let ours: Vec<&String> = host
            .errors
            .iter()
            .filter(|e| !e.contains("interrupted") && (e.contains("SystemsHost") || e.contains("EWD")))
            .collect();
        assert!(ours.is_empty(), "script errors in SystemsHost/EWD: {ours:?}");
    }
}
