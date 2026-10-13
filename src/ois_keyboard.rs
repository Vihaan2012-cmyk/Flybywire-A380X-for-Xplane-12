use crate::xp::{DataRef, Xplm};

const OIT_LEFT: &str = "SCREEN_OIT_LEFT";
const OIT_RIGHT: &str = "SCREEN_OIT_RIGHT";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Key {
    Char(char, char, u32),
    Special(u32),
    Shift,
    CapsLock,
}

fn key_of(name: &str) -> Option<Key> {
    let b = name.as_bytes();
    if b.len() == 1 && b[0].is_ascii_digit() {
        return Some(Key::Char(b[0] as char, b[0] as char, b[0] as u32));
    }
    if b.len() == 1 && b[0].is_ascii_uppercase() {
        return Some(Key::Char(b[0].to_ascii_lowercase() as char, b[0] as char, b[0] as u32));
    }
    if let Some(n) = name.strip_prefix('F').and_then(|n| n.parse::<u32>().ok()).filter(|n| (1..=12).contains(n)) {
        return Some(Key::Special(0x6F + n));
    }
    Some(match name {
        "SPACE_LH" | "SPACE_RH" => Key::Char(' ', ' ', 0x20),
        "COMMA" => Key::Char(',', ',', 0xBC),
        "FULLSTOP" => Key::Char('.', '.', 0xBE),
        "SLASH" => Key::Char('/', '/', 0xBF),
        "BACKSLASH" => Key::Char('\\', '\\', 0xDC),
        "SEMICOLON" => Key::Char(';', ':', 0xBA),
        "MINUS" => Key::Char('-', '_', 0xBD),
        "PLUS" => Key::Char('+', '+', 0xBB),
        "BRACE_OPEN" => Key::Char('(', '(', 0xDB),
        "BRACE_CLOSE" => Key::Char(')', ')', 0xDD),
        "SWUNG_DASH" => Key::Char('~', '~', 0xC0),
        "ELLIPSIS" => Key::Char('\u{2026}', '\u{2026}', 0),
        "ENTER" => Key::Special(crate::xp::VK_RETURN),
        "BACKSPACE" => Key::Special(crate::xp::VK_BACK),
        "TAB" => Key::Special(crate::xp::VK_TAB),
        "DEL" => Key::Special(crate::xp::VK_DELETE),
        "ESC" => Key::Special(crate::xp::VK_ESCAPE),
        "INS" => Key::Special(0x2D),
        "ARROW_LEFT" | "LEFT" => Key::Special(0x25),
        "ARROW_UP" | "UP" => Key::Special(0x26),
        "ARROW_RIGHT" | "RIGHT" => Key::Special(0x27),
        "ARROW_DOWN" | "DOWN" => Key::Special(0x28),
        "SKIP_UP" => Key::Special(0x21),
        "SKIP_DOWN" => Key::Special(0x22),
        "SHIFT_LH" | "SHIFT_RH" => Key::Shift,
        "CAPS" | "CAPSLOCK" => Key::CapsLock,
        _ => return None,
    })
}

const KEYBOARD: &[&str] = &[
    "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S",
    "T", "U", "V", "W", "X", "Y", "Z", "F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12", "SPACE_LH", "SPACE_RH", "COMMA",
    "FULLSTOP", "SLASH", "BACKSLASH", "SEMICOLON", "MINUS", "PLUS", "BRACE_OPEN", "BRACE_CLOSE", "SWUNG_DASH", "ELLIPSIS", "ENTER", "BACKSPACE",
    "TAB", "DEL", "ESC", "INS", "ARROW_LEFT", "ARROW_UP", "ARROW_RIGHT", "ARROW_DOWN", "SKIP_UP", "SKIP_DOWN", "SHIFT_LH", "SHIFT_RH", "CAPS",
    "CAPSLOCK",
];

const PAD: &[&str] = &["UP", "DOWN", "LEFT", "RIGHT", "ENTER", "ESC"];

struct Button {
    path: String,
    key: Key,
    screen: &'static str,
    dataref: Option<DataRef>,
    was_down: bool,
}

thread_local! {
    static STATE: std::cell::RefCell<Option<OisKeyboard>> = const { std::cell::RefCell::new(None) };
}

pub fn update(xplm: &Xplm) {
    STATE.with(|s| s.borrow_mut().get_or_insert_with(OisKeyboard::new).update(xplm));
}

pub struct OisKeyboard {
    buttons: Vec<Button>,
    shift: bool,
    caps: bool,
}

impl OisKeyboard {
    pub fn new() -> Self {
        let mut buttons = Vec::new();
        for name in KEYBOARD {
            if let Some(key) = key_of(name) {
                buttons.push(Button { path: format!("fbw/cockpit/PUSH_KBD_CPT_ANIM_{name}"), key, screen: OIT_LEFT, dataref: None, was_down: false });
            }
        }
        for (side, screen) in [("CPT", OIT_LEFT), ("FO", OIT_RIGHT)] {
            for name in PAD {
                if let Some(key) = key_of(name) {
                    buttons.push(Button { path: format!("fbw/cockpit/PUSH_ACD_{side}_{name}"), key, screen, dataref: None, was_down: false });
                }
            }
        }
        Self { buttons, shift: false, caps: false }
    }

    pub fn update(&mut self, xplm: &Xplm) {
        let mut typed = Vec::new();
        for b in &mut self.buttons {
            if b.dataref.is_none() {
                b.dataref = xplm.find(&b.path);
            }
            let Some(d) = b.dataref else { continue };
            let down = xplm.get_f(d) > 0.5;
            if down && !b.was_down {
                typed.push((b.key, b.screen));
            }
            b.was_down = down;
        }
        for (key, screen) in typed {
            if let Some((ch, vkey)) = self.press(key) {
                crate::display::type_on(screen, ch, vkey);
            }
        }
    }

    fn press(&mut self, key: Key) -> Option<(u32, u32)> {
        match key {
            Key::Shift => {
                self.shift = !self.shift;
                None
            }
            Key::CapsLock => {
                self.caps = !self.caps;
                None
            }
            Key::Special(vkey) => Some((0, vkey)),
            Key::Char(lower, upper, vkey) => {
                let up = self.shift != self.caps;
                self.shift = false;
                Some((if up { upper } else { lower } as u32, vkey))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_keyboard_and_pad_key_maps() {
        for name in KEYBOARD.iter().chain(PAD) {
            assert!(key_of(name).is_some(), "{name}");
        }
    }

    #[test]
    fn letters_are_lowercase_until_shift_or_caps() {
        let mut k = OisKeyboard::new();
        let a = key_of("A").unwrap();
        assert_eq!(k.press(a), Some(('a' as u32, 0x41)));
        k.press(Key::Shift);
        assert_eq!(k.press(a), Some(('A' as u32, 0x41)));
        assert_eq!(k.press(a), Some(('a' as u32, 0x41)), "shift lasts one key");
        k.press(Key::CapsLock);
        assert_eq!(k.press(a), Some(('A' as u32, 0x41)));
    }

    #[test]
    fn special_keys_send_no_character() {
        let mut k = OisKeyboard::new();
        assert_eq!(k.press(key_of("ENTER").unwrap()), Some((0, crate::xp::VK_RETURN)));
        assert_eq!(k.press(key_of("F5").unwrap()), Some((0, 0x74)));
        assert_eq!(k.press(key_of("ARROW_UP").unwrap()), Some((0, 0x26)));
    }

    #[test]
    fn the_pads_type_on_their_own_side() {
        let k = OisKeyboard::new();
        assert!(k.buttons.iter().any(|b| b.path == "fbw/cockpit/PUSH_ACD_FO_UP" && b.screen == OIT_RIGHT));
        assert!(k.buttons.iter().any(|b| b.path == "fbw/cockpit/PUSH_ACD_CPT_ENTER" && b.screen == OIT_LEFT));
        assert!(k.buttons.iter().any(|b| b.path == "fbw/cockpit/PUSH_KBD_CPT_ANIM_Q" && b.screen == OIT_LEFT));
    }
}
