//! `fbw/anim/eye_in_flight_deck`: 1 while the pilot's eye is inside the
//! flight deck, where the wings, engines, gear and tail cannot be seen, so the
//! exterior objects that are hidden by it are not drawn into the cockpit view
//! or its shadow passes.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::OnceLock;

use crate::xp::{DataRef, Xplm};

const AFT_OF_DEFAULT_EYE_M: f32 = 2.5;

static IN_FLIGHT_DECK: AtomicBool = AtomicBool::new(false);
static NAME: OnceLock<std::ffi::CString> = OnceLock::new();

unsafe extern "C" fn get(_refcon: *mut std::ffi::c_void) -> f32 {
    if IN_FLIGHT_DECK.load(Ordering::Relaxed) {
        1.0
    } else {
        0.0
    }
}

fn in_flight_deck(inside: bool, head_z: f32, default_eye_z: Option<f32>) -> bool {
    inside && default_eye_z.is_some_and(|eye| head_z - eye < AFT_OF_DEFAULT_EYE_M)
}

#[derive(Default)]
pub struct FlightDeckView {
    registered: bool,
    external: Option<Option<DataRef>>,
    head_z: Option<Option<DataRef>>,
    default_eye_z: Option<f32>,
}

impl FlightDeckView {
    pub fn update(&mut self, xplm: &Xplm) {
        if !self.registered {
            self.registered = true;
            let name = NAME.get_or_init(|| std::ffi::CString::new("fbw/anim/eye_in_flight_deck").unwrap_or_default());
            let accessors = crate::xp::Accessors { get_f: Some(get), ..Default::default() };
            let _ = crate::xp::register_accessor(name, crate::xp::DATA_TYPE_FLOAT, false, &accessors, std::ptr::null_mut());
        }
        let external = *self.external.get_or_insert_with(|| xplm.find("sim/graphics/view/view_is_external"));
        let head = *self.head_z.get_or_insert_with(|| xplm.find("sim/graphics/view/pilots_head_z"));
        let (Some(external), Some(head)) = (external, head) else { return };
        let inside = xplm.get_i(external) == 0;
        let z = xplm.get_f(head);
        if inside && self.default_eye_z.is_none() {
            self.default_eye_z = Some(z);
        }
        IN_FLIGHT_DECK.store(in_flight_deck(inside, z, self.default_eye_z), Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::in_flight_deck;

    #[test]
    fn the_eye_counts_as_in_the_flight_deck_only_inside_and_near_the_default_eye() {
        assert!(in_flight_deck(true, -29.6, Some(-29.63)));
        assert!(in_flight_deck(true, -31.0, Some(-29.63)), "leaning forward");
        assert!(!in_flight_deck(true, -26.0, Some(-29.63)), "walked back into the cabin");
        assert!(!in_flight_deck(false, -29.6, Some(-29.63)), "outside view");
        assert!(!in_flight_deck(true, -29.6, None), "no default eye yet");
    }
}
