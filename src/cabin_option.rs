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
const SWAP_FILES_AFTER_START_S: f64 = 60.0;

#[derive(Default)]
pub struct CabinOption {
    dataref: Option<DataRef>,
    looked_up: bool,
    since_lookup_s: f64,
    since_write_s: f64,
    last_written: Option<bool>,
    since_start_s: f64,
    files_for: Option<bool>,
}

pub fn swap_files(objects: &std::path::Path, want: bool) -> std::io::Result<usize> {
    let without = objects.join("cabin_off");
    let stored = objects.join("cabin_on");
    let Ok(entries) = std::fs::read_dir(&without) else { return Ok(0) };
    let mut changed = 0;
    for entry in entries.flatten() {
        let name = entry.file_name();
        let active = objects.join(&name);
        let full = stored.join(&name);
        if want {
            if full.exists() {
                std::fs::rename(&full, &active)?;
                changed += 1;
            }
        } else if !full.exists() && active.exists() {
            std::fs::create_dir_all(&stored)?;
            std::fs::rename(&active, &full)?;
            std::fs::copy(entry.path(), &active)?;
            changed += 1;
        }
    }
    Ok(changed)
}

impl CabinOption {
    pub fn update(&mut self, xplm: &Xplm, delta: f64) {
        self.since_lookup_s += delta;
        self.since_write_s += delta;
        self.since_start_s += delta;
        let wanted = crate::app_settings::current().cabin_visible;
        if self.since_start_s >= SWAP_FILES_AFTER_START_S && self.files_for != Some(wanted) {
            self.files_for = Some(wanted);
            if let Some(objects) = crate::sound_pack::plugin_dir().and_then(|d| Some(d.parent()?.parent()?.join("objects"))) {
                match swap_files(&objects, wanted) {
                    Ok(0) => {}
                    Ok(n) => crate::log(&format!("cabin: {n} cabin objects switched {} for the next aircraft load", if wanted { "on" } else { "off" })),
                    Err(e) => crate::log(&format!("cabin: could not switch the cabin objects: {e}")),
                }
            }
        }
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

#[cfg(test)]
mod tests {
    use super::swap_files;

    #[test]
    fn switching_the_cabin_off_and_on_swaps_the_object_files_and_back() {
        let dir = std::env::temp_dir().join(format!("cabin_swap_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(dir.join("cabin_off")).unwrap();
        std::fs::write(dir.join("seats.obj"), "full").unwrap();
        std::fs::write(dir.join("cabin_off").join("seats.obj"), "empty").unwrap();

        assert_eq!(swap_files(&dir, false).unwrap(), 1);
        assert_eq!(std::fs::read_to_string(dir.join("seats.obj")).unwrap(), "empty");
        assert_eq!(std::fs::read_to_string(dir.join("cabin_on").join("seats.obj")).unwrap(), "full");
        assert_eq!(swap_files(&dir, false).unwrap(), 0, "already off");

        assert_eq!(swap_files(&dir, true).unwrap(), 1);
        assert_eq!(std::fs::read_to_string(dir.join("seats.obj")).unwrap(), "full");
        assert!(!dir.join("cabin_on").join("seats.obj").exists());
        assert_eq!(swap_files(&dir, true).unwrap(), 0, "already on");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
