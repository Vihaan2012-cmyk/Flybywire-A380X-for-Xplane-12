//! FlyByWire's interior lighting presets: LightingPresets
//! (fbw-common/src/wasm/extra-backend/LightingPresets/LightingPresets.cpp)
//! with the A380X's lights (fbw-a380x/src/wasm/extra-backend-a380x/src/
//! LightingPresets/LightingPresets_A380X.cpp and .h).
//!
//! `A32NX_LIGHTING_PRESET_LOAD` or `_SAVE` set to a preset number loads or
//! saves it; a load moves every light towards the preset over
//! `A32NX_LIGHTING_PRESET_LOAD_TIME` seconds, as FlyByWire's does.
//!
//! Differences from MSFS:
//! - The presets file: MSFS keeps `\work\InteriorLightingPresets.ini` in the
//!   package's work folder (LightingPresets.h:18). Here it is
//!   `Output/preferences/fbw_a380x_lighting_presets.ini`, beside the fuel
//!   levels fuel.rs keeps, in the same ini format.
//! - The potentiometers are the plugin's `LIGHT POTENTIOMETER:n` variables,
//!   kept as MSFS keeps them, percent over 100 (key_events.rs), and worked
//!   in percent as FlyByWire reads and writes them (LightingPresets.cpp:
//!   131-133), instead of through the `LIGHT_POTENTIOMETER_SET` event.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use super::named;

/// LightingPresets.h:21-25.
const MIN_STEP_SIZE: f64 = 1.05;
const MAX_STEP_SIZE: f64 = 10.;
const TOTAL_LOADING_TIME: f64 = 2.;
const UPDATE_DELAY_TIME: f64 = 0.15;

/// The A380X's lights: ini key and LIGHT POTENTIOMETER index, in
/// LightingValues_A380X's order (LightingPresets_A380X.h:16-54,
/// LightingPresets_A380X.cpp:32-79, 155-196). `None` is the EFB brightness,
/// the named variable EFB_BRIGHTNESS, whose default is 80 (cpp:164).
const LIGHTS: [(&str, Option<u32>); 26] = [
    ("efb_brightness", None),
    ("reading_cpt_lt", Some(96)),
    ("reading_fo_lt", Some(97)),
    ("glareshield_int_lt", Some(84)),
    ("glareshield_lcd_lt", Some(87)),
    ("table_cpt_lt", Some(10)),
    ("table_fo_lt", Some(11)),
    ("pfd_cpt_lvl", Some(88)),
    ("nd_cpt_lvl", Some(89)),
    ("wx_cpt_lvl", Some(94)),
    ("mfd_cpt_lvl", Some(98)),
    ("console_cpt_lt", Some(8)),
    ("pfd_fo_lvl", Some(90)),
    ("nd_fo_lvl", Some(91)),
    ("wx_fo_lvl", Some(95)),
    ("mfd_fo_lvl", Some(99)),
    ("console_fo_lt", Some(9)),
    ("rmp_cpt_lt", Some(80)),
    ("rmp_fo_lt", Some(81)),
    ("rmp_ovhd_lt", Some(82)),
    ("ecam_upper_lvl", Some(92)),
    ("ecam_lower_lvl", Some(93)),
    ("flood_ped_lvl", Some(76)),
    ("flood_pnl_lt", Some(83)),
    ("pedestal_int_lt", Some(85)),
    ("cabin_light", Some(7)),
];

/// Percent per stored unit: MSFS keeps a potentiometer in percent over 100
/// (what `LIGHT_POTENTIOMETER_SET`'s percent becomes, key_events.rs); the
/// EFB brightness is a plain number.
fn scale(index: Option<u32>) -> f64 {
    if index.is_some() {
        100.
    } else {
        1.
    }
}

pub fn ini_path() -> PathBuf {
    PathBuf::from("Output").join("preferences").join("fbw_a380x_lighting_presets.ini")
}

type Ini = BTreeMap<String, BTreeMap<String, String>>;

/// mINI's reading: `[section]`, `key = value`, `;` and `#` comments, keys
/// lower-cased.
fn parse_ini(text: &str) -> Ini {
    let mut ini = Ini::new();
    let mut section = String::new();
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with(';') || line.starts_with('#') {
            continue;
        }
        if let Some(name) = line.strip_prefix('[').and_then(|l| l.strip_suffix(']')) {
            section = name.trim().to_ascii_lowercase();
            ini.entry(section.clone()).or_default();
        } else if let Some((k, v)) = line.split_once('=') {
            ini.entry(section.clone()).or_default().insert(k.trim().to_ascii_lowercase(), v.trim().to_string());
        }
    }
    ini
}

/// mINI's pretty writing.
fn write_ini(ini: &Ini) -> String {
    let mut out = String::new();
    for (section, keys) in ini {
        out.push_str(&format!("[{section}]\n"));
        for (k, v) in keys {
            out.push_str(&format!("{k} = {v}\n"));
        }
        out.push('\n');
    }
    out
}

/// LightingPresets::convergeValue (cpp:151-161).
pub fn converge_value(momentary: f64, target: f64, step: f64) -> f64 {
    if (momentary - target).abs() <= step {
        target
    } else if momentary < target {
        (momentary + step).min(target)
    } else {
        (momentary - step).max(target)
    }
}

pub struct LightingPresets {
    path: PathBuf,
    ac1_powered: VariableIdentifier,
    is_ready: VariableIdentifier,
    load_request: VariableIdentifier,
    save_request: VariableIdentifier,
    load_time: VariableIdentifier,
    lights: Vec<VariableIdentifier>,
    ini: Ini,
    read_ini_file: bool,
    last_update: f64,
    /// loadedLightValues, kept between loads as FlyByWire's member is.
    loaded: [f64; 26],
}

impl LightingPresets {
    /// LightingPresets::initialize and initialize_aircraft (cpp:10-28,
    /// A380X cpp:32-79).
    pub fn new<V: VariableRegistry + SimulatorReaderWriter>(vars: &mut V, path: PathBuf) -> Self {
        let load_request = named(vars, "LIGHTING_PRESET_LOAD");
        let save_request = named(vars, "LIGHTING_PRESET_SAVE");
        let load_time = named(vars, "LIGHTING_PRESET_LOAD_TIME");
        vars.write(&load_request, 0.);
        vars.write(&save_request, 0.);
        vars.write(&load_time, TOTAL_LOADING_TIME);
        let lights = LIGHTS
            .iter()
            .map(|(_, index)| match index {
                Some(n) => vars.get(format!("LIGHT POTENTIOMETER:{n}")),
                None => named(vars, "EFB_BRIGHTNESS"),
            })
            .collect();
        Self {
            path,
            ac1_powered: named(vars, "ELEC_AC_1_BUS_IS_POWERED"),
            is_ready: named(vars, "IS_READY"),
            load_request,
            save_request,
            load_time,
            lights,
            ini: Ini::new(),
            read_ini_file: true,
            last_update: 0.,
            loaded: [0.; 26],
        }
    }

    /// LightingPresets::update (cpp:30-65). `time` is the simulation time.
    pub fn update<V: SimulatorReaderWriter>(&mut self, vars: &mut V, time: f64) {
        if vars.read(&self.is_ready) == 0. || vars.read(&self.ac1_powered) == 0. {
            return;
        }
        let load = vars.read(&self.load_request) as i64;
        if load != 0 {
            if self.load_lighting_preset(vars, load, time) {
                self.read_ini_file = true;
                vars.write(&self.load_request, 0.);
                crate::log(&format!("extra backend: Lighting Preset: {load} successfully loaded."));
            }
        } else if vars.read(&self.save_request) != 0. {
            let save = vars.read(&self.save_request) as i64;
            self.save_lighting_preset(vars, save);
            vars.write(&self.save_request, 0.);
        }
    }

    fn read_from_aircraft<V: SimulatorReaderWriter>(&self, vars: &mut V) -> Vec<f64> {
        self.lights.iter().zip(LIGHTS.iter()).map(|(id, (_, index))| vars.read(id) * scale(*index)).collect()
    }

    /// LightingPresets::loadLightingPreset (cpp:73-99): true when finished.
    fn load_lighting_preset<V: SimulatorReaderWriter>(&mut self, vars: &mut V, preset: i64, time: f64) -> bool {
        let delta = time - self.last_update;
        if delta < UPDATE_DELAY_TIME {
            return false;
        }
        let partial_load = vars.read(&self.load_time) / delta;
        let step = (100. / partial_load).clamp(MIN_STEP_SIZE, MAX_STEP_SIZE);
        self.last_update = time;

        let current = self.read_from_aircraft(vars);
        if !self.read_from_store() {
            crate::log(&format!("extra backend: Loading Lighting Preset: {preset} failed."));
            return true;
        }
        // LightingPresets_A380X::loadFromIni (cpp:155-196) with
        // iniGetOrDefault (LightingPresets.cpp:135-149). For a preset the
        // file lacks, FlyByWire sets its intermediate values to 50 % and
        // returns without touching the loaded values, but
        // calculateIntermediateValues then recomputes the intermediate values
        // from the loaded ones (LightingPresets.cpp:92-95), so the lights move
        // towards whatever was loaded last (zero at first). Kept as it is.
        if let Some(keys) = self.ini.get(&format!("preset {preset}")) {
            for (loaded, (key, index)) in self.loaded.iter_mut().zip(LIGHTS.iter()) {
                let default = if index.is_none() { 80. } else { 50. };
                *loaded = keys.get(*key).and_then(|v| v.parse::<f64>().ok()).unwrap_or(default);
            }
        }
        let loaded = self.loaded;
        // calculateIntermediateValues and applyToAircraft (A380X cpp:270-306,
        // 120-153); finished when every light is within 0.1 (h:144-177).
        let mut finished = true;
        for (((id, now), target), (_, index)) in self.lights.iter().zip(&current).zip(&loaded).zip(LIGHTS.iter()) {
            let next = converge_value(*now, *target, step);
            finished &= (next - target).abs() <= 0.1;
            vars.write(id, next / scale(*index));
        }
        finished
    }

    /// LightingPresets::readFromStore (cpp:111-122): the file is read once
    /// per load, so it can be edited between loads.
    fn read_from_store(&mut self) -> bool {
        if self.read_ini_file {
            match std::fs::read_to_string(&self.path) {
                Ok(text) => self.ini = parse_ini(&text),
                // mINI reads a missing file as empty; a load then gives the
                // 50 % default.
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => self.ini = Ini::new(),
                Err(e) => {
                    crate::log(&format!("extra backend: could not read {}: {e}", self.path.display()));
                    return false;
                }
            }
            self.read_ini_file = false;
        }
        true
    }

    /// LightingPresets::saveLightingPreset and saveToStore (cpp:101-129).
    fn save_lighting_preset<V: SimulatorReaderWriter>(&mut self, vars: &mut V, preset: i64) {
        let current = self.read_from_aircraft(vars);
        if let Ok(text) = std::fs::read_to_string(&self.path) {
            self.ini = parse_ini(&text);
        }
        let section = self.ini.entry(format!("preset {preset}")).or_default();
        for ((key, _), value) in LIGHTS.iter().zip(current) {
            // std::to_string of a double.
            section.insert(key.to_string(), format!("{value:.6}"));
        }
        match write_file(&self.path, &write_ini(&self.ini)) {
            Ok(()) => crate::log(&format!("extra backend: Lighting Preset: {preset} successfully saved.")),
            Err(e) => crate::log(&format!("extra backend: Saving Lighting Preset: {preset} failed: {e}")),
        }
    }
}

fn write_file(path: &Path, text: &str) -> std::io::Result<()> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    std::fs::write(path, text)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aspects::test_vars::TestVars;

    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join("fbw_extra_backend_tests");
        let _ = std::fs::create_dir_all(&dir);
        let path = dir.join(name);
        let _ = std::fs::remove_file(&path);
        path
    }

    #[test]
    fn saved_preset_loads_back_gradually() {
        let path = scratch("lighting_presets.ini");
        let mut vars = TestVars::default();
        let mut p = LightingPresets::new(&mut vars, path.clone());
        vars.set("A32NX_IS_READY", 1.);
        vars.set("A32NX_ELEC_AC_1_BUS_IS_POWERED", 1.);
        vars.set("LIGHT POTENTIOMETER:88", 1.);
        vars.set("A32NX_EFB_BRIGHTNESS", 30.);
        vars.set("A32NX_LIGHTING_PRESET_SAVE", 3.);
        p.update(&mut vars, 1.);
        assert_eq!(vars.value("A32NX_LIGHTING_PRESET_SAVE"), 0.);
        let text = std::fs::read_to_string(&path).unwrap();
        assert!(text.contains("[preset 3]") && text.contains("pfd_cpt_lvl = 100.000000") && text.contains("efb_brightness = 30.000000"), "{text}");

        vars.set("LIGHT POTENTIOMETER:88", 0.);
        vars.set("A32NX_EFB_BRIGHTNESS", 80.);
        vars.set("A32NX_LIGHTING_PRESET_LOAD", 3.);
        let mut t = 2.;
        let mut frames = 0;
        let mut seen_between = false;
        while vars.value("A32NX_LIGHTING_PRESET_LOAD") != 0. && frames < 1000 {
            t += 1. / 30.;
            frames += 1;
            p.update(&mut vars, t);
            let v = vars.value("LIGHT POTENTIOMETER:88");
            seen_between |= v > 0. && v < 1.;
        }
        assert!(seen_between, "the light moves in steps");
        assert!((vars.value("LIGHT POTENTIOMETER:88") - 1.).abs() < 1e-12);
        assert_eq!(vars.value("A32NX_EFB_BRIGHTNESS"), 30.);
        // A 2 s load at 30 fps: 0.15 s steps of at most 10 %.
        assert!(frames > 30 && frames < 120, "{frames} frames");
    }

    #[test]
    fn nothing_happens_unpowered() {
        let mut vars = TestVars::default();
        let mut p = LightingPresets::new(&mut vars, scratch("unpowered.ini"));
        vars.set("A32NX_IS_READY", 1.);
        vars.set("A32NX_LIGHTING_PRESET_LOAD", 1.);
        p.update(&mut vars, 5.);
        assert_eq!(vars.value("A32NX_LIGHTING_PRESET_LOAD"), 1.);
        assert_eq!(vars.value("LIGHT POTENTIOMETER:88"), 0.);
    }

    #[test]
    fn convergence_matches_flybywire() {
        assert_eq!(converge_value(0., 100., 10.), 10.);
        assert_eq!(converge_value(95., 100., 10.), 100.);
        assert_eq!(converge_value(50., 20., 10.), 40.);
    }
}
