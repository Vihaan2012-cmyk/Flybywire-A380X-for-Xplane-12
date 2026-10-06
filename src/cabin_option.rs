//! The EFB's "Passenger cabin" switch (Settings > Aircraft,
//! `xphfbw.cabinVisible` in xphfbw.json, read live by `app_settings`), put
//! on the dataref the converted cabin objects show by.
//!
//! msfs2xp-aircraft wraps every passenger-cabin object's geometry in
//! `ANIM_show 1 1 fbw/options/cabin_visible`, and its generated SASL module
//! creates that dataref as a float (writable, default 1). This only writes it: turning
//! the cabin off stops X-Plane drawing it, while its textures stay loaded
//! (a conversion with `--no-cabin` is what saves the video memory).
//!
//! The dataref belongs to SASL, which may not have created it yet on the
//! first ticks after an aircraft load, so the lookup is retried every
//! [`LOOKUP_EVERY_S`] until it succeeds. The value is re-asserted every
//! [`REASSERT_EVERY_S`] as well as on every change, so a SASL reload that
//! recreates the dataref at its default cannot silently turn a hidden cabin
//! back on.

use crate::xp::{DataRef, Xplm};

const DATAREF: &str = "fbw/options/cabin_visible";
const LOOKUP_EVERY_S: f64 = 5.0;
const REASSERT_EVERY_S: f64 = 2.0;

#[derive(Default)]
pub struct CabinOption {
    dataref: Option<DataRef>,
    looked_up: bool,
    since_lookup_s: f64,
    since_write_s: f64,
    last_written: Option<bool>,
}

impl CabinOption {
    pub fn update(&mut self, xplm: &Xplm, delta: f64) {
        self.since_lookup_s += delta;
        self.since_write_s += delta;
        if self.dataref.is_none() {
            // First tick, then every LOOKUP_EVERY_S until SASL has made it.
            if self.looked_up && self.since_lookup_s < LOOKUP_EVERY_S {
                return;
            }
            self.looked_up = true;
            self.since_lookup_s = 0.0;
            self.dataref = xplm.find(DATAREF);
        }
        let Some(d) = self.dataref else { return };
        let want = crate::app_settings::current().cabin_visible;
        if self.last_written != Some(want) || self.since_write_s >= REASSERT_EVERY_S {
            // SASL creates it as a float (`createGlobalPropertyf`), and
            // X-Plane silently drops an integer write to a float dataref --
            // which is why the switch did nothing on 2026-09-27.
            xplm.set_f(d, if want { 1.0 } else { 0.0 });
            self.last_written = Some(want);
            self.since_write_s = 0.0;
        }
    }
}
