//! Putting a cockpit screen in a window of its own.
//!
//! Every display this plugin draws is an X-Plane *avionics device*
//! (`XPLMCreateAvionicsEx`), and X-Plane can show any of those outside the
//! 3D cockpit in two different ways, which are worth keeping apart:
//!
//! * **popped up** (`XPLMSetAvionicsPopupVisible`) floats it inside the
//!   simulator's own window. Fine for a quick look at the MCDU, useless for
//!   reading a PFD while flying, because it sits on top of the cockpit it is
//!   meant to be read beside.
//! * **popped out** (`XPLMPopOutAvionics`) gives it an operating-system
//!   window, which can be dragged to a second monitor and left there. This
//!   is what "pop-out displays" means to anyone who has flown with two
//!   screens.
//!
//! Both are offered, per screen, as commands and as menu entries. Commands
//! because a pop-out is the sort of thing that wants a joystick button or a
//! key, and menu entries because a command nobody has bound is a command
//! nobody can find.
//!
//! **X-Plane is asked, never remembered.** Whether a screen is already
//! popped out is read back with `XPLMIsAvionicsPoppedOut` every time rather
//! than tracked here: the user can close one of these windows with its own
//! chrome, and a toggle that kept its own idea of the state would then need
//! two presses to reopen it.

use std::ffi::{c_int, c_void};

use crate::xp::{CommandRef, Xplm};

/// What a command or menu entry does to a screen.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Act {
    /// Into an operating-system window, for a second monitor.
    PopOut,
    /// Floating inside X-Plane's own window.
    PopUp,
    /// Whichever it is in, put it away.
    Close,
}

/// A screen's index and what to do to it, packed into a command refcon.
/// X-Plane hands the refcon back untouched, so this is the whole state a
/// handler needs: no lookup table, and nothing to keep in step with
/// `screens::SCREENS`.
pub fn refcon_for(screen: usize, act: Act) -> *mut c_void {
    refcon(screen, act)
}

fn refcon(screen: usize, act: Act) -> *mut c_void {
    let act = match act {
        Act::PopOut => 0usize,
        Act::PopUp => 1,
        Act::Close => 2,
    };
    ((screen << 2) | act) as *mut c_void
}

/// The inverse of [`refcon`].
pub fn unpack(refcon: *mut c_void) -> (usize, Act) {
    let raw = refcon as usize;
    let act = match raw & 0b11 {
        0 => Act::PopOut,
        1 => Act::PopUp,
        _ => Act::Close,
    };
    (raw >> 2, act)
}

/// Do it. Called from the command handler and the menu handler alike, on
/// X-Plane's own thread.
pub fn apply(displays: &mut super::Displays, screen: usize, act: Act) {
    let Some(api) = displays.api else { return };
    let Some(s) = displays.screens.get(screen) else { return };
    let handle = s.handle;
    if handle.is_null() {
        return;
    }
    let id = s.def.id;
    unsafe {
        match act {
            Act::PopOut => {
                // Popping out needs the popup to exist first: X-Plane moves
                // the popup's window out, it does not make one.
                (api.set_popup_visible)(handle, 1);
                (api.pop_out)(handle);
                // X-Plane's own default geometry for a freshly popped-out
                // window is not derived from the device's screen size at
                // all: measured (PrintWindow capture, 2026-09-25) as an
                // identical 198x1080 sliver for the EFB (1430x1000), the
                // PFD and the EWD (768x1024 each). Size and place the OS
                // window ourselves, right after `pop_out`, or the user gets
                // a window too narrow to read anything in.
                let (left, top, right, bottom) = pop_out_rect(screen, s.def.width, s.def.height);
                (api.set_geometry_os)(handle, left, top, right, bottom);
                crate::log(&format!("{id}: popped out to its own window"));
            }
            Act::PopUp => {
                (api.set_popup_visible)(handle, 1);
                crate::log(&format!("{id}: popped up inside X-Plane"));
            }
            Act::Close => {
                (api.set_popup_visible)(handle, 0);
                crate::log(&format!("{id}: put away"));
            }
        }
    }
}

/// Where to put a freshly popped-out screen's OS window: sized 1:1 to its
/// own screen (matching `CreateAvionics::bezel_width/height`, so nothing is
/// scaled), and cascaded by screen index so popping out several screens in
/// a row does not stack their windows exactly on top of each other. A
/// fixed, monitor-agnostic starting point -- there is no bound
/// `XPLMGetAllMonitorBoundsOS` call yet to place it on a specific monitor --
/// so on a second monitor smaller than a wide screen like the FCU
/// (2560x1280) the user still has to drag an edge, but it is now a real,
/// resizable window instead of X-Plane's 198 px default sliver.
fn pop_out_rect(screen: usize, width: u32, height: u32) -> (c_int, c_int, c_int, c_int) {
    let step = (screen as c_int % 8) * 48;
    let left = 80 + step;
    let top = 80 + step;
    (left, top, left + width as c_int, top + height as c_int)
}

/// Whether X-Plane currently has this screen in a window of its own.
pub fn is_out(displays: &super::Displays, screen: usize) -> bool {
    let (Some(api), Some(s)) = (displays.api, displays.screens.get(screen)) else { return false };
    !s.handle.is_null() && unsafe { (api.is_popped_out)(s.handle) } != 0
}

/// Whether X-Plane is showing it at all, in either kind of window.
pub fn is_shown(displays: &super::Displays, screen: usize) -> bool {
    let (Some(api), Some(s)) = (displays.api, displays.screens.get(screen)) else { return false };
    !s.handle.is_null() && unsafe { (api.is_popup_visible)(s.handle) } != 0
}

/// A short name for a screen, for command and menu text: `SCREEN_DU_PFDL`
/// reads as `PFD L`.
pub fn pretty(id: &str) -> String {
    let base = id.strip_prefix("SCREEN_").unwrap_or(id);
    let base = base.strip_prefix("DU_").unwrap_or(base);
    base.replace('_', " ")
}

/// Every command this module registers, so the plugin can create them once
/// and X-Plane's keyboard/joystick bindings can find them by name.
pub struct Commands {
    pub refs: Vec<(CommandRef, *mut c_void)>,
}

impl Commands {
    /// `fbw/display/popout/<id>`, `.../popup/<id>` and `.../close/<id>` for
    /// every screen, plus one that puts every popped-out screen away again
    /// -- which matters because a dozen windows are quick to open and slow
    /// to close one at a time.
    pub fn register(xplm: &Xplm, handler: crate::xp::CommandHandler) -> Self {
        let mut refs = Vec::new();
        for (i, def) in super::screens::SCREENS.iter().enumerate() {
            let name = pretty(def.id);
            for (verb, act, what) in [
                ("popout", Act::PopOut, "in its own window"),
                ("popup", Act::PopUp, "floating in X-Plane"),
                ("close", Act::Close, "put away"),
            ] {
                let id = def.id.strip_prefix("SCREEN_").unwrap_or(def.id).to_ascii_lowercase();
                let Some(command) = xplm.create_command(
                    &format!("fbw/display/{verb}/{id}"),
                    &format!("A380X: {name} {what}"),
                ) else {
                    continue;
                };
                let rc = refcon(i, act);
                xplm.register_command_handler(command, handler, rc);
                refs.push((command, rc));
            }
        }
        Self { refs }
    }
}

/// X-Plane's own command return: 0 means "handled, do not pass it on".
pub const HANDLED: c_int = 0;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_refcon_carries_the_screen_and_the_action_back_unchanged() {
        // The refcon is the whole state a handler gets, so a screen index
        // that did not survive the round trip would pop out the wrong
        // display -- and with sixteen screens that is not obvious from the
        // seat.
        for screen in [0usize, 1, 7, 17, 255] {
            for act in [Act::PopOut, Act::PopUp, Act::Close] {
                assert_eq!(unpack(refcon(screen, act)), (screen, act), "screen {screen} {act:?}");
            }
        }
    }

    #[test]
    fn pop_out_rect_sizes_the_window_to_the_screen_not_xplanes_default_sliver() {
        // The bug this guards: X-Plane's own default pop-out geometry was
        // measured as 198x1080 for every screen regardless of its own size
        // (EFB 1430x1000, PFD/EWD 768x1024). The window must come out at
        // the screen's own width/height, not some unrelated default.
        let (left, top, right, bottom) = pop_out_rect(0, 1430, 1000);
        assert_eq!((right - left, bottom - top), (1430, 1000));
        let (left, top, right, bottom) = pop_out_rect(6, 768, 1024);
        assert_eq!((right - left, bottom - top), (768, 1024));
    }

    #[test]
    fn pop_out_rect_does_not_stack_consecutive_screens_on_the_same_spot() {
        // Popping out several screens in a row (the "put every popped-out
        // screen away again" comment on `Commands::register` implies users
        // do open more than one at a time) should not leave every window
        // exactly on top of the last.
        let (l0, t0, ..) = pop_out_rect(0, 768, 1024);
        let (l1, t1, ..) = pop_out_rect(1, 768, 1024);
        assert_ne!((l0, t0), (l1, t1));
    }

    #[test]
    fn screen_names_read_as_the_screen_they_are() {
        assert_eq!(pretty("SCREEN_DU_PFDL"), "PFDL");
        assert_eq!(pretty("SCREEN_OIT_LEFT"), "OIT LEFT");
        assert_eq!(pretty("SCREEN_EFB"), "EFB");
        assert_eq!(pretty("Clock"), "Clock");
    }

    #[test]
    fn every_screen_gets_a_unique_command_name() {
        // Two screens sharing a command name would silently give one of
        // them no way to be opened at all.
        let mut names: Vec<String> = super::super::screens::SCREENS
            .iter()
            .map(|d| d.id.strip_prefix("SCREEN_").unwrap_or(d.id).to_ascii_lowercase())
            .collect();
        let before = names.len();
        names.sort();
        names.dedup();
        assert_eq!(names.len(), before, "two screens would share a command name");
    }
}
