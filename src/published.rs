//! Datarefs and commands the plugin owns beyond the `fbw/<variable>` ones:
//! the radio, door and EFB interfaces.
//!
//! X-Plane calls accessors and command handlers on its main thread, between
//! flight loops, so a write or a press is held here and the flight loop takes
//! it on its next tick. A module keeps the handles `Published` gives it, reads
//! the writes and presses with [`take`] and [`presses`], and
//! shows its state with [`set`] and [`set_text`].
//!
//! Registration goes through xp.rs's free-standing bindings; outside
//! X-Plane (the tests) a handle is still made and works, it is only not
//! visible to anything.

use std::ffi::{c_int, c_void, CString};
use std::sync::Mutex;

use crate::xp::{self, CommandRef, DataRef};

/// A dataref this plugin owns.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Value(usize);

/// A command this plugin owns.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Command(usize);

enum Content {
    Number(f64),
    Text(Vec<u8>),
}

struct Slot {
    content: Content,
    writable: bool,
    /// Set by a write from X-Plane, taken by the module.
    written: bool,
}

struct State {
    slots: Vec<Slot>,
    presses: Vec<u32>,
}

static STATE: Mutex<State> = Mutex::new(State { slots: Vec::new(), presses: Vec::new() });

fn with_state<R>(f: impl FnOnce(&mut State) -> R) -> Option<R> {
    STATE.lock().ok().map(|mut s| f(&mut s))
}

/// Show a value.
pub fn set(value: Value, v: f64) {
    with_state(|s| {
        if let Some(Slot { content: Content::Number(n), .. }) = s.slots.get_mut(value.0) {
            *n = v;
        }
    });
}

pub fn get(value: Value) -> f64 {
    with_state(|s| match s.slots.get(value.0) {
        Some(Slot { content: Content::Number(n), .. }) => *n,
        _ => 0.,
    })
    .unwrap_or(0.)
}

/// What X-Plane wrote since the last call, if anything.
pub fn take(value: Value) -> Option<f64> {
    with_state(|s| match s.slots.get_mut(value.0) {
        Some(slot @ Slot { written: true, .. }) => {
            slot.written = false;
            match slot.content {
                Content::Number(n) => Some(n),
                Content::Text(_) => None,
            }
        }
        _ => None,
    })
    .flatten()
}

/// Show a string, cut to the dataref's length.
pub fn set_text(value: Value, text: &str) {
    with_state(|s| {
        if let Some(Slot { content: Content::Text(bytes), .. }) = s.slots.get_mut(value.0) {
            let n = bytes.len();
            bytes.iter_mut().for_each(|b| *b = 0);
            for (b, c) in bytes.iter_mut().zip(text.bytes().take(n)) {
                *b = c;
            }
        }
    });
}

#[cfg(test)]
pub fn get_text(value: Value) -> String {
    with_state(|s| match s.slots.get(value.0) {
        Some(Slot { content: Content::Text(bytes), .. }) => text_of(bytes),
        _ => String::new(),
    })
    .unwrap_or_default()
}

/// The string X-Plane wrote since the last call, if anything.
#[cfg(test)]
pub fn take_text(value: Value) -> Option<String> {
    with_state(|s| match s.slots.get_mut(value.0) {
        Some(slot @ Slot { written: true, .. }) => {
            slot.written = false;
            match &slot.content {
                Content::Text(bytes) => Some(text_of(bytes)),
                Content::Number(_) => None,
            }
        }
        _ => None,
    })
    .flatten()
}

#[cfg(test)]
fn text_of(bytes: &[u8]) -> String {
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).into_owned()
}

/// How many times a command was pressed since the last call.
pub fn presses(command: Command) -> u32 {
    with_state(|s| s.presses.get_mut(command.0).map(std::mem::take).unwrap_or(0)).unwrap_or(0)
}

/// Press a command from inside the plugin, as X-Plane would.
pub fn press(command: Command) {
    with_state(|s| {
        if let Some(p) = s.presses.get_mut(command.0) {
            *p += 1;
        }
    });
}

// ---------------------------------------------------------------------------
// X-Plane.
// ---------------------------------------------------------------------------

const TYPE_NUMBER: c_int = xp::DATA_TYPE_INT | xp::DATA_TYPE_FLOAT | xp::DATA_TYPE_DOUBLE;

fn write_number(refcon: *mut c_void, v: f64) {
    with_state(|s| {
        if let Some(slot) = s.slots.get_mut(refcon as usize) {
            if let (true, Content::Number(n)) = (slot.writable, &mut slot.content) {
                *n = v;
                slot.written = true;
            }
        }
    });
}

unsafe extern "C" fn get_i(refcon: *mut c_void) -> c_int {
    get(Value(refcon as usize)).round() as c_int
}
unsafe extern "C" fn get_f(refcon: *mut c_void) -> f32 {
    get(Value(refcon as usize)) as f32
}
unsafe extern "C" fn get_d(refcon: *mut c_void) -> f64 {
    get(Value(refcon as usize))
}
unsafe extern "C" fn set_i(refcon: *mut c_void, v: c_int) {
    write_number(refcon, v as f64)
}
unsafe extern "C" fn set_f(refcon: *mut c_void, v: f32) {
    write_number(refcon, v as f64)
}
unsafe extern "C" fn set_d(refcon: *mut c_void, v: f64) {
    write_number(refcon, v)
}

unsafe extern "C" fn get_b(refcon: *mut c_void, out: *mut c_void, offset: c_int, max: c_int) -> c_int {
    with_state(|s| match s.slots.get(refcon as usize) {
        Some(Slot { content: Content::Text(bytes), .. }) => {
            if out.is_null() {
                return bytes.len() as c_int;
            }
            let offset = (offset.max(0) as usize).min(bytes.len());
            let n = (max.max(0) as usize).min(bytes.len() - offset);
            std::ptr::copy_nonoverlapping(bytes[offset..].as_ptr(), out as *mut u8, n);
            n as c_int
        }
        _ => 0,
    })
    .unwrap_or(0)
}

unsafe extern "C" fn set_b(refcon: *mut c_void, values: *mut c_void, offset: c_int, count: c_int) {
    if values.is_null() || count <= 0 {
        return;
    }
    let written = std::slice::from_raw_parts(values as *const u8, count as usize);
    with_state(|s| {
        if let Some(slot) = s.slots.get_mut(refcon as usize) {
            if let (true, Content::Text(bytes)) = (slot.writable, &mut slot.content) {
                let offset = offset.max(0) as usize;
                // A write is a whole string: what follows it is cleared.
                bytes.iter_mut().skip(offset).for_each(|b| *b = 0);
                for (b, v) in bytes.iter_mut().skip(offset).zip(written) {
                    *b = *v;
                }
                slot.written = true;
            }
        }
    });
}

unsafe extern "C" fn on_command(_: CommandRef, phase: c_int, refcon: *mut c_void) -> c_int {
    if phase == 0 {
        press(Command(refcon as usize));
    }
    1
}

/// X-Plane's own command taken over: counted, and not passed on to X-Plane.
unsafe extern "C" fn on_intercepted(_: CommandRef, phase: c_int, refcon: *mut c_void) -> c_int {
    if phase == 0 {
        press(Command(refcon as usize));
    }
    0
}

/// The datarefs and commands one module registered, unregistered when
/// dropped.
#[derive(Default)]
pub struct Published {
    datarefs: Vec<DataRef>,
    commands: Vec<(CommandRef, usize)>,
    intercepted: Vec<(CommandRef, usize)>,
    names: Vec<CString>,
}

impl Published {
    fn slot(&mut self, name: &str, content: Content, writable: bool) -> usize {
        let types = match content {
            Content::Number(_) => TYPE_NUMBER,
            Content::Text(_) => xp::DATA_TYPE_DATA,
        };
        let index = with_state(|s| {
            s.slots.push(Slot { content, writable, written: false });
            s.slots.len() - 1
        })
        .unwrap_or(usize::MAX);
        let Ok(c) = CString::new(name) else { return index };
        let number = types == TYPE_NUMBER;
        let accessors = if number {
            xp::Accessors {
                get_i: Some(get_i),
                set_i: writable.then_some(set_i as xp::AccessorSetI),
                get_f: Some(get_f),
                set_f: writable.then_some(set_f as xp::AccessorSetF),
                get_d: Some(get_d),
                set_d: writable.then_some(set_d as xp::AccessorSetD),
                ..Default::default()
            }
        } else {
            xp::Accessors { get_b: Some(get_b), set_b: writable.then_some(set_b as xp::AccessorSetB), ..Default::default() }
        };
        if let Some(dataref) = xp::register_accessor(&c, types, writable, &accessors, index as *mut c_void) {
            self.datarefs.push(dataref);
        }
        self.names.push(c);
        index
    }

    /// An int, float and double dataref in one.
    pub fn number(&mut self, name: &str, start: f64, writable: bool) -> Value {
        Value(self.slot(name, Content::Number(start), writable))
    }

    /// A byte array dataref holding a string of at most `len` bytes.
    pub fn text(&mut self, name: &str, len: usize, writable: bool) -> Value {
        Value(self.slot(name, Content::Text(vec![0; len]), writable))
    }

    /// A command; its presses are counted for [`presses`].
    pub fn command(&mut self, name: &str, description: &str) -> Command {
        let index = with_state(|s| {
            s.presses.push(0);
            s.presses.len() - 1
        })
        .unwrap_or(usize::MAX);
        if let Some(command) = xp::command_by_name(name, description) {
            xp::add_command_handler(command, on_command, true, index as *mut c_void);
            self.commands.push((command, index));
        }
        Command(index)
    }

    /// Take one of X-Plane's commands over: its presses are counted here and
    /// X-Plane's own handling never sees them.
    pub fn intercept(&mut self, name: &str) -> Command {
        let index = with_state(|s| {
            s.presses.push(0);
            s.presses.len() - 1
        })
        .unwrap_or(usize::MAX);
        if let Some(command) = xp::command_by_name(name, name) {
            xp::add_command_handler(command, on_intercepted, true, index as *mut c_void);
            self.intercepted.push((command, index));
        }
        Command(index)
    }
}

impl Drop for Published {
    fn drop(&mut self) {
        for (c, index) in self.commands.drain(..) {
            xp::remove_command_handler(c, on_command, true, index as *mut c_void);
        }
        for (c, index) in self.intercepted.drain(..) {
            xp::remove_command_handler(c, on_intercepted, true, index as *mut c_void);
        }
        for d in self.datarefs.drain(..) {
            xp::unregister_accessor(d);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn writes_and_presses_are_held_for_the_flight_loop() {
        let mut p = Published::default();
        let v = p.number("fbw/test/published_number", 3., true);
        let r = p.number("fbw/test/published_read_only", 1., false);
        let t = p.text("fbw/test/published_text", 8, true);
        let c = p.command("fbw/test/published_command", "test");
        assert_eq!(get(v), 3.);
        assert_eq!(take(v), None);
        unsafe { set_f(v.0 as *mut c_void, 7.5) };
        unsafe { set_i(r.0 as *mut c_void, 9) };
        assert_eq!(take(v), Some(7.5));
        assert_eq!(take(v), None);
        assert_eq!(get(r), 1., "a read-only dataref ignores writes");
        let mut bytes = *b"IBOS";
        unsafe { set_b(t.0 as *mut c_void, bytes.as_mut_ptr() as *mut c_void, 0, 4) };
        assert_eq!(take_text(t).as_deref(), Some("IBOS"));
        set_text(t, "LONGER THAN EIGHT");
        assert_eq!(get_text(t), "LONGER T");
        unsafe {
            on_command(std::ptr::null_mut(), 0, c.0 as *mut c_void);
            on_command(std::ptr::null_mut(), 1, c.0 as *mut c_void);
            on_command(std::ptr::null_mut(), 0, c.0 as *mut c_void);
        }
        assert_eq!(presses(c), 2);
        assert_eq!(presses(c), 0);
    }
}
