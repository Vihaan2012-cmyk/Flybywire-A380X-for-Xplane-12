use std::cell::RefCell;
use std::collections::HashMap;

use crate::xp::{DataRef, Xplm};

const REPORTED: &[&str] = &[
    "sim/private/controls/photometric/ev100",
    "sim/private/controls/photometric/ev100_bias",
    "sim/private/controls/photometric/ev100_min",
    "sim/private/controls/photometric/ev100_max",
    "sim/private/controls/photometric/ev100_mid",
    "sim/private/controls/photometric/ev100_mtr",
    "sim/private/controls/photometric/interior_lit_boost",
    "sim/private/controls/photometric/speed",
    "sim/private/controls/exposure/speed_up",
    "sim/private/controls/exposure/speed_down",
    "sim/private/controls/exposure_fusion/shadows",
    "sim/private/controls/autoexposure/gain_hi",
    "sim/private/controls/autoexposure/gain_lo",
    "sim/private/controls/autoexposure/trim_hi",
    "sim/private/controls/autoexposure/trim_lo",
    "sim/private/controls/autoexposure/null_hi",
    "sim/private/controls/autoexposure/null_lo",
    "sim/private/controls/tonemap/mode",
];

#[derive(Default)]
struct Tweaks {
    generation: Option<u64>,
    wanted: Vec<(String, f32)>,
    refs: HashMap<String, Option<DataRef>>,
    originals: HashMap<String, f32>,
    view_external: Option<Option<DataRef>>,
    reported: bool,
    last_report: Option<std::time::Instant>,
}

thread_local! {
    static TWEAKS: RefCell<Tweaks> = RefCell::new(Tweaks::default());
}

fn wanted_from_settings() -> Vec<(String, f32)> {
    let Some(root) = crate::xp::system_path() else { return Vec::new() };
    let path = crate::settings_files::app_settings_path(&root);
    std::fs::read_to_string(path)
        .ok()
        .and_then(|t| serde_json::from_str::<serde_json::Value>(&t).ok())
        .and_then(|v| {
            v.get("cockpitDatarefs")
                .and_then(|m| m.as_object())
                .map(|m| m.iter().filter_map(|(k, v)| v.as_f64().map(|x| (k.clone(), x as f32))).collect())
        })
        .unwrap_or_default()
}

fn is_int(d: DataRef) -> bool {
    let types = crate::xp::data_ref_types(d);
    types & crate::xp::DATA_TYPE_INT != 0 && types & crate::xp::DATA_TYPE_FLOAT == 0
}

fn read(xplm: &Xplm, d: DataRef) -> f32 {
    if is_int(d) {
        xplm.get_i(d) as f32
    } else {
        xplm.get_f(d)
    }
}

fn write(xplm: &Xplm, d: DataRef, value: f32) {
    if is_int(d) {
        xplm.set_i(d, value.round() as std::ffi::c_int);
    } else {
        xplm.set_f(d, value);
    }
}

impl Tweaks {
    fn dataref(&mut self, xplm: &Xplm, name: &str) -> Option<DataRef> {
        *self.refs.entry(name.to_owned()).or_insert_with(|| xplm.find(name))
    }

    fn report(&mut self, xplm: &Xplm, why: &str) {
        let values: Vec<String> = REPORTED
            .iter()
            .map(|name| {
                let short = name.trim_start_matches("sim/private/controls/");
                match self.dataref(xplm, name) {
                    Some(d) => format!("{short}={:.3}", xplm.get_f(d)),
                    None => format!("{short}=missing"),
                }
            })
            .collect();
        crate::log(&format!("cockpit tweaks ({why}): {}", values.join(" ")));
    }

    fn restore(&mut self, xplm: &Xplm) {
        let originals: Vec<(String, f32)> = self.originals.drain().collect();
        for (name, value) in originals {
            if let Some(d) = self.dataref(xplm, &name) {
                write(xplm, d, value);
            }
        }
    }

    fn tick(&mut self, xplm: &Xplm) {
        if !self.reported {
            self.reported = true;
            self.report(xplm, "X-Plane's own values");
        }
        if self.last_report.is_none_or(|t| t.elapsed().as_secs() >= 20) {
            self.last_report = Some(std::time::Instant::now());
            self.report(xplm, "now");
        }
        let generation = crate::app_settings::generation();
        if self.generation != Some(generation) {
            self.generation = Some(generation);
            self.restore(xplm);
            self.wanted = wanted_from_settings();
            if !self.wanted.is_empty() {
                crate::log(&format!("cockpit tweaks: applying {:?} in the cockpit view", self.wanted));
            }
        }
        let view = *self.view_external.get_or_insert_with(|| xplm.find("sim/graphics/view/view_is_external"));
        let internal = view.is_some_and(|d| xplm.get_i(d) == 0);
        if !internal {
            self.restore(xplm);
            return;
        }
        let wanted = self.wanted.clone();
        for (name, value) in wanted {
            let Some(d) = self.dataref(xplm, &name) else { continue };
            if !self.originals.contains_key(&name) {
                self.originals.insert(name.clone(), read(xplm, d));
            }
            write(xplm, d, value);
        }
    }
}

pub fn tick(xplm: &Xplm) {
    TWEAKS.with(|t| t.borrow_mut().tick(xplm));
}
