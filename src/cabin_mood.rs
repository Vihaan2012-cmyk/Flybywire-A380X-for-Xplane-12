//! Cabin mood lighting: scenes by flight phase and local time (or picked by hand), coloured per livery by `liveries/<name>/cabin.json`.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::OnceLock;

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::xp::{DataRef, Xplm};
use crate::Vars;

pub const PALETTE: [(&str, [f64; 3]); 8] = [
    ("white", [1.0, 0.95, 0.88]),
    ("warm", [1.0, 0.72, 0.42]),
    ("amber", [1.0, 0.5, 0.15]),
    ("pink", [1.0, 0.42, 0.62]),
    ("purple", [0.62, 0.36, 1.0]),
    ("blue", [0.32, 0.5, 1.0]),
    ("teal", [0.25, 0.85, 0.8]),
    ("night", [0.12, 0.16, 0.45]),
];

const NIGHT: usize = 7;
const BLUE: usize = 5;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Scene {
    Boarding,
    Taxi,
    Takeoff,
    Climb,
    Cruise,
    Meal,
    Night,
    Sunrise,
    Descent,
    Landing,
    Arrival,
}

pub const SCENES: [Scene; 11] = [
    Scene::Boarding,
    Scene::Taxi,
    Scene::Takeoff,
    Scene::Climb,
    Scene::Cruise,
    Scene::Meal,
    Scene::Night,
    Scene::Sunrise,
    Scene::Descent,
    Scene::Landing,
    Scene::Arrival,
];

impl Scene {
    pub fn key(self) -> &'static str {
        match self {
            Scene::Boarding => "boarding",
            Scene::Taxi => "taxi",
            Scene::Takeoff => "takeoff",
            Scene::Climb => "climb",
            Scene::Cruise => "cruise",
            Scene::Meal => "meal",
            Scene::Night => "night",
            Scene::Sunrise => "sunrise",
            Scene::Descent => "descent",
            Scene::Landing => "landing",
            Scene::Arrival => "arrival",
        }
    }

    fn default_look(self) -> Look {
        let (palette, level) = match self {
            Scene::Boarding => (1, 0.9),
            Scene::Taxi => (5, 0.25),
            Scene::Takeoff => (5, 0.15),
            Scene::Climb => (1, 0.5),
            Scene::Cruise => (0, 0.7),
            Scene::Meal => (1, 0.9),
            Scene::Night => (NIGHT, 0.8),
            Scene::Sunrise => (2, 0.5),
            Scene::Descent => (1, 0.5),
            Scene::Landing => (5, 0.15),
            Scene::Arrival => (0, 0.9),
        };
        Look { rgb: PALETTE[palette].1, level }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Look {
    pub rgb: [f64; 3],
    pub level: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CabinConfig {
    pub looks: [Look; 11],
    pub stars: bool,
}

impl Default for CabinConfig {
    fn default() -> Self {
        Self { looks: SCENES.map(Scene::default_look), stars: true }
    }
}

pub fn parse_colour(s: &str) -> Option<[f64; 3]> {
    let s = s.trim();
    if let Some((_, rgb)) = PALETTE.iter().find(|(n, _)| n.eq_ignore_ascii_case(s)) {
        return Some(*rgb);
    }
    let hex = s.strip_prefix('#')?;
    if hex.len() != 6 {
        return None;
    }
    let v = u32::from_str_radix(hex, 16).ok()?;
    Some([((v >> 16) & 255) as f64 / 255.0, ((v >> 8) & 255) as f64 / 255.0, (v & 255) as f64 / 255.0])
}

fn chroma(rgb: [f64; 3]) -> [f64; 3] {
    let m = rgb.iter().cloned().fold(0.0, f64::max).max(1e-6);
    rgb.map(|c| c / m)
}

pub fn nearest_palette(rgb: [f64; 3]) -> usize {
    let c = chroma(rgb);
    (0..PALETTE.len())
        .min_by(|&a, &b| {
            let d = |i: usize| chroma(PALETTE[i].1).iter().zip(c).map(|(p, q)| (p - q).powi(2)).sum::<f64>();
            d(a).total_cmp(&d(b))
        })
        .unwrap_or(0)
}

pub fn parse_config(text: &str) -> CabinConfig {
    let mut cfg = CabinConfig::default();
    let Ok(v) = serde_json::from_str::<serde_json::Value>(text) else { return cfg };
    if let Some(stars) = v.get("stars").and_then(|s| s.as_bool()) {
        cfg.stars = stars;
    }
    if let Some(scenes) = v.get("scenes").and_then(|s| s.as_object()) {
        for (i, scene) in SCENES.iter().enumerate() {
            let Some(entry) = scenes.get(scene.key()) else { continue };
            let (colour, level) = match entry {
                serde_json::Value::String(s) => (Some(s.as_str()), None),
                serde_json::Value::Object(o) => (o.get("color").or(o.get("colour")).and_then(|c| c.as_str()), o.get("level").and_then(|l| l.as_f64())),
                _ => (None, None),
            };
            if let Some(rgb) = colour.and_then(parse_colour) {
                cfg.looks[i].rgb = rgb;
            }
            if let Some(level) = level {
                cfg.looks[i].level = level.clamp(0.0, 1.0);
            }
        }
    }
    cfg
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Inputs {
    pub on_ground: bool,
    pub ground_speed_kt: f64,
    pub engines_running: bool,
    pub agl_ft: f64,
    pub vs_fpm: f64,
    pub local_hour: f64,
}

const MEAL_FROM_S: f64 = 20.0 * 60.0;
const MEAL_UNTIL_S: f64 = 75.0 * 60.0;
const TAKEOFF_HOLD_S: f64 = 120.0;

#[derive(Debug)]
pub struct Phase {
    airborne_s: f64,
    cruise_s: f64,
    landed: bool,
    descending: bool,
}

impl Default for Phase {
    fn default() -> Self {
        Self { airborne_s: 0.0, cruise_s: 0.0, landed: false, descending: false }
    }
}

fn night_hour(h: f64) -> bool {
    !(6.0..21.0).contains(&h)
}

impl Phase {
    pub fn step(&mut self, i: &Inputs, dt: f64) -> Scene {
        if i.on_ground {
            if self.airborne_s > 0.0 {
                self.landed = true;
            }
            self.airborne_s = 0.0;
            self.cruise_s = 0.0;
            self.descending = false;
            return if self.landed {
                if i.ground_speed_kt > 30.0 {
                    Scene::Landing
                } else {
                    Scene::Arrival
                }
            } else if i.engines_running && i.ground_speed_kt > 40.0 {
                Scene::Takeoff
            } else if i.engines_running && i.ground_speed_kt > 3.0 {
                Scene::Taxi
            } else {
                Scene::Boarding
            };
        }
        self.landed = false;
        self.airborne_s += dt;
        let level = i.vs_fpm.abs() < 500.0 && i.agl_ft > 15_000.0;
        if level {
            self.cruise_s += dt;
            self.descending = false;
        } else if i.vs_fpm < -500.0 && self.cruise_s > 0.0 {
            self.descending = true;
        }
        if self.airborne_s < TAKEOFF_HOLD_S && i.vs_fpm > 0.0 {
            return Scene::Takeoff;
        }
        if i.vs_fpm < -300.0 && i.agl_ft < 2_500.0 {
            return Scene::Landing;
        }
        if self.descending {
            return Scene::Descent;
        }
        if !level {
            return if i.vs_fpm > 300.0 { Scene::Climb } else { Scene::Cruise };
        }
        if (MEAL_FROM_S..MEAL_UNTIL_S).contains(&self.cruise_s) {
            return Scene::Meal;
        }
        if (5.0..6.5).contains(&i.local_hour) && self.cruise_s >= MEAL_UNTIL_S {
            return Scene::Sunrise;
        }
        if night_hour(i.local_hour) && self.cruise_s >= MEAL_UNTIL_S {
            return Scene::Night;
        }
        Scene::Cruise
    }
}

const FADE_PER_S: f64 = 1.25;
const BLEND_PER_S: f64 = 0.6;

#[derive(Debug)]
pub struct Output {
    pub variant: usize,
    pub glow: f64,
    pub spill: [f64; 3],
}

impl Default for Output {
    fn default() -> Self {
        Self { variant: 0, glow: 0.0, spill: [0.0; 3] }
    }
}

impl Output {
    pub fn step(&mut self, variant: usize, level: f64, rgb: [f64; 3], dt: f64) {
        let toward = |from: f64, to: f64, rate: f64| if from < to { (from + rate).min(to) } else { (from - rate).max(to) };
        if variant != self.variant {
            self.glow = toward(self.glow, 0.0, FADE_PER_S * dt);
            if self.glow <= 0.0 {
                self.variant = variant;
            }
        } else {
            self.glow = toward(self.glow, level, FADE_PER_S * dt);
        }
        for c in 0..3 {
            self.spill[c] = toward(self.spill[c], rgb[c] * level, BLEND_PER_S * dt);
        }
    }
}

pub fn variant_for(look: &Look, scene: Scene, stars: bool) -> usize {
    let v = nearest_palette(look.rgb);
    match (scene, v) {
        (Scene::Night, NIGHT) if !stars => BLUE,
        (_, NIGHT) if scene != Scene::Night => BLUE,
        _ => v,
    }
}

pub fn livery_name(path: &str) -> Option<String> {
    let trimmed = path.trim_end_matches(['/', '\\', '\0']);
    let mut parts = trimmed.rsplit(['/', '\\']);
    let name = parts.next()?;
    (parts.next()?.eq_ignore_ascii_case("liveries") && !name.is_empty()).then(|| name.to_owned())
}

const DATAREFS: [&str; 6] = ["fbw/cabin/variant", "fbw/cabin/glow_level", "fbw/cabin/spill_r", "fbw/cabin/spill_g", "fbw/cabin/spill_b", "fbw/cabin/scene"];
static VALUES: [AtomicU32; 6] = [const { AtomicU32::new(0) }; 6];
static NAMES: OnceLock<Vec<std::ffi::CString>> = OnceLock::new();

unsafe extern "C" fn get(refcon: *mut std::ffi::c_void) -> f32 {
    VALUES.get(refcon as usize).map_or(0.0, |v| f32::from_bits(v.load(Ordering::Relaxed)))
}

fn publish(i: usize, v: f64) {
    VALUES[i].store((v as f32).to_bits(), Ordering::Relaxed);
}

struct Refs {
    livery: Option<DataRef>,
    on_ground: Option<DataRef>,
    ground_speed: Option<DataRef>,
    burning: Option<DataRef>,
    agl: Option<DataRef>,
    vs: Option<DataRef>,
    local_time: Option<DataRef>,
    paused: Option<DataRef>,
}

struct Ids {
    ac: [VariableIdentifier; 4],
    theme: VariableIdentifier,
}

#[derive(Default)]
pub struct CabinMood {
    refs: Option<Refs>,
    ids: Option<Ids>,
    livery: Option<Option<String>>,
    since_livery_check_s: f64,
    config: CabinConfig,
    phase: Phase,
    out: Output,
    theme_rev: f64,
}

const LIVERY_CHECK_S: f64 = 5.0;

impl CabinMood {
    pub fn update(&mut self, vars: &mut Vars, xplm: &Xplm, delta: f64) {
        if self.refs.is_none() {
            let names = NAMES.get_or_init(|| DATAREFS.iter().map(|n| std::ffi::CString::new(*n).unwrap_or_default()).collect());
            let accessors = crate::xp::Accessors { get_f: Some(get), ..Default::default() };
            for (i, name) in names.iter().enumerate() {
                let _ = crate::xp::register_accessor(name, crate::xp::DATA_TYPE_FLOAT, false, &accessors, i as *mut std::ffi::c_void);
            }
        }
        let r = self.refs.get_or_insert_with(|| Refs {
            livery: xplm.find("sim/aircraft/view/acf_livery_path"),
            on_ground: xplm.find("sim/flightmodel/failures/onground_any"),
            ground_speed: xplm.find("sim/flightmodel/position/groundspeed"),
            burning: xplm.find("sim/flightmodel2/engines/engine_is_burning_fuel"),
            agl: xplm.find("sim/flightmodel/position/y_agl"),
            vs: xplm.find("sim/flightmodel/position/vh_ind_fpm"),
            local_time: xplm.find("sim/time/local_time_sec"),
            paused: xplm.find("sim/time/paused"),
        });
        let ids = self.ids.get_or_insert_with(|| Ids {
            ac: [1, 2, 3, 4].map(|n| vars.get(format!("A32NX_ELEC_AC_{n}_BUS_IS_POWERED"))),
            theme: vars.get("A380X_IFE_THEME".to_owned()),
        });
        if r.paused.is_some_and(|d| xplm.get_i(d) != 0) || delta <= 0.0 {
            return;
        }

        self.since_livery_check_s += delta;
        if self.livery.is_none() || self.since_livery_check_s >= LIVERY_CHECK_S {
            self.since_livery_check_s = 0.0;
            let mut raw = [0u8; 1024];
            let n = r.livery.map_or(0, |d| xplm.get_vb(d, &mut raw));
            let path = String::from_utf8_lossy(&raw[..n.min(raw.len())]).to_string();
            let name = livery_name(&path);
            if self.livery.as_ref() != Some(&name) {
                self.config = load_config(name.as_deref());
                self.theme_rev += 1.0;
                publish_ife_theme(name.as_deref());
                crate::log(&format!("cabin: livery {}, {}", name.as_deref().unwrap_or("(default)"), if cabin_json(name.as_deref()).is_some_and(|p| p.exists()) { "cabin.json found" } else { "default cabin lighting" }));
                self.livery = Some(name);
            }
            vars.write(&ids.theme, self.theme_rev);
        }

        let f = |d: Option<DataRef>| d.map_or(0.0, |d| xplm.get_f(d) as f64);
        let mut burning = [0i32; 4];
        if let Some(d) = r.burning {
            xplm.get_vi(d, &mut burning);
        }
        let inputs = Inputs {
            on_ground: r.on_ground.is_some_and(|d| xplm.get_i(d) != 0),
            ground_speed_kt: f(r.ground_speed) * 1.943_844,
            engines_running: burning.iter().any(|&b| b != 0),
            agl_ft: f(r.agl) * 3.280_84,
            vs_fpm: f(r.vs),
            local_hour: f(r.local_time) / 3600.0,
        };
        let auto = self.phase.step(&inputs, delta);
        let settings = crate::app_settings::current();
        let scene = match settings.cabin_scene {
            0 => auto,
            n => SCENES.get(n as usize - 1).copied().unwrap_or(auto),
        };
        let index = SCENES.iter().position(|s| *s == scene).unwrap_or(0);
        let look = self.config.looks[index];
        let powered = ids.ac.iter().any(|id| vars.read(id) != 0.0);
        let level = if powered { look.level * settings.cabin_brightness } else { 0.0 };
        let variant = variant_for(&look, scene, self.config.stars);
        self.out.step(variant, level, look.rgb, delta);

        publish(0, self.out.variant as f64);
        publish(1, self.out.glow);
        publish(2, self.out.spill[0]);
        publish(3, self.out.spill[1]);
        publish(4, self.out.spill[2]);
        publish(5, index as f64);
    }
}

fn aircraft_dir() -> Option<PathBuf> {
    crate::sound_pack::plugin_dir().and_then(|d| Some(d.parent()?.parent()?.to_path_buf()))
}

fn cabin_json(livery: Option<&str>) -> Option<PathBuf> {
    Some(aircraft_dir()?.join("liveries").join(livery?).join("cabin.json"))
}

fn load_config(livery: Option<&str>) -> CabinConfig {
    cabin_json(livery).and_then(|p| std::fs::read_to_string(p).ok()).map(|t| parse_config(&t)).unwrap_or_default()
}

const IFE_FILES: [&str; 2] = ["ife_logo.png", "ife_background.png"];

fn publish_ife_theme(livery: Option<&str>) {
    let Some(aircraft) = aircraft_dir() else { return };
    let theme = aircraft.join("html_ui/Pages/VCockpit/Instruments/A380X/IFE/theme");
    if std::fs::create_dir_all(&theme).is_err() {
        return;
    }
    let source = livery.map(|l| aircraft.join("liveries").join(l));
    let ife = source
        .as_deref()
        .and_then(|s| std::fs::read_to_string(s.join("cabin.json")).ok())
        .and_then(|t| serde_json::from_str::<serde_json::Value>(&t).ok())
        .and_then(|v| v.get("ife").cloned())
        .unwrap_or(serde_json::Value::Object(Default::default()));
    let _ = write_theme(&theme, &ife, source.as_deref());
}

fn write_theme(theme: &Path, ife: &serde_json::Value, source: Option<&Path>) -> std::io::Result<()> {
    let mut v = ife.clone();
    if !v.is_object() {
        v = serde_json::Value::Object(Default::default());
    }
    for file in IFE_FILES {
        let dst = theme.join(file);
        match source.map(|s| s.join(file)).filter(|p| p.exists()) {
            Some(src) => {
                std::fs::copy(src, &dst)?;
                v[file.trim_end_matches(".png")] = serde_json::Value::String(file.to_owned());
            }
            None => {
                let _ = std::fs::remove_file(&dst);
            }
        }
    }
    let json = serde_json::to_string_pretty(&v).unwrap_or_default();
    std::fs::write(theme.join("theme.js"), format!("window.IFE_THEME = {json};\n"))?;
    std::fs::write(theme.join("theme.json"), json)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(on_ground: bool, gs: f64, engines: bool, agl: f64, vs: f64, hour: f64) -> Inputs {
        Inputs { on_ground, ground_speed_kt: gs, engines_running: engines, agl_ft: agl, vs_fpm: vs, local_hour: hour }
    }

    #[test]
    fn a_whole_flight_walks_through_the_scenes() {
        let mut p = Phase::default();
        assert_eq!(p.step(&at(true, 0.0, false, 0.0, 0.0, 22.0), 1.0), Scene::Boarding);
        assert_eq!(p.step(&at(true, 12.0, true, 0.0, 0.0, 22.0), 1.0), Scene::Taxi);
        assert_eq!(p.step(&at(true, 120.0, true, 0.0, 0.0, 22.0), 1.0), Scene::Takeoff);
        assert_eq!(p.step(&at(false, 170.0, true, 500.0, 2500.0, 22.0), 1.0), Scene::Takeoff);
        assert_eq!(p.step(&at(false, 300.0, true, 9000.0, 2000.0, 22.0), 200.0), Scene::Climb);
        assert_eq!(p.step(&at(false, 480.0, true, 37000.0, 0.0, 22.0), 60.0), Scene::Cruise);
        assert_eq!(p.step(&at(false, 480.0, true, 37000.0, 0.0, 22.0), 30.0 * 60.0), Scene::Meal);
        assert_eq!(p.step(&at(false, 480.0, true, 37000.0, 0.0, 23.5), 60.0 * 60.0), Scene::Night);
        assert_eq!(p.step(&at(false, 480.0, true, 37000.0, 0.0, 5.5), 60.0), Scene::Sunrise);
        assert_eq!(p.step(&at(false, 400.0, true, 20000.0, -2000.0, 6.0), 60.0), Scene::Descent);
        assert_eq!(p.step(&at(false, 150.0, true, 1500.0, -700.0, 6.2), 60.0), Scene::Landing);
        assert_eq!(p.step(&at(true, 120.0, true, 0.0, 0.0, 6.3), 1.0), Scene::Landing);
        assert_eq!(p.step(&at(true, 10.0, true, 0.0, 0.0, 6.3), 1.0), Scene::Arrival);
        assert_eq!(p.step(&at(true, 0.0, false, 0.0, 0.0, 6.4), 1.0), Scene::Arrival);
    }

    #[test]
    fn a_daytime_long_cruise_stays_in_cruise_after_the_meal() {
        let mut p = Phase::default();
        p.step(&at(true, 0.0, false, 0.0, 0.0, 12.0), 1.0);
        p.step(&at(false, 480.0, true, 37000.0, 0.0, 12.0), 2.0 * 3600.0);
        assert_eq!(p.step(&at(false, 480.0, true, 37000.0, 0.0, 14.0), 60.0), Scene::Cruise);
    }

    #[test]
    fn colours_come_from_hex_or_palette_names_and_snap_to_the_strip_palette() {
        assert_eq!(parse_colour("#FF0000"), Some([1.0, 0.0, 0.0]));
        assert_eq!(parse_colour("purple"), Some(PALETTE[4].1));
        assert_eq!(parse_colour("#12"), None);
        assert_eq!(nearest_palette([0.6, 0.3, 0.95]), 4);
        assert_eq!(nearest_palette([1.0, 1.0, 0.9]), 0);
        assert_eq!(nearest_palette([0.2, 0.9, 0.85]), 6);
    }

    #[test]
    fn cabin_json_overrides_only_what_it_names() {
        let cfg = parse_config(r##"{"stars": false, "scenes": {"boarding": "#C8102E", "night": {"color": "purple", "level": 0.1}}}"##);
        assert!(!cfg.stars);
        assert_eq!(cfg.looks[0].rgb, [200.0 / 255.0, 16.0 / 255.0, 46.0 / 255.0]);
        assert_eq!(cfg.looks[0].level, 0.9);
        assert_eq!(cfg.looks[6].level, 0.1);
        assert_eq!(cfg.looks[4], Scene::Cruise.default_look());
        assert_eq!(parse_config("not json"), CabinConfig::default());
    }

    #[test]
    fn the_starry_night_strip_only_shows_at_night_and_when_the_livery_wants_it() {
        let night = Scene::Night.default_look();
        assert_eq!(variant_for(&night, Scene::Night, true), NIGHT);
        assert_eq!(variant_for(&night, Scene::Night, false), BLUE);
        assert_eq!(variant_for(&night, Scene::Cruise, true), BLUE);
    }

    #[test]
    fn a_colour_change_fades_out_switches_and_fades_back_in() {
        let mut o = Output::default();
        for _ in 0..100 {
            o.step(1, 0.8, [1.0, 0.7, 0.4], 0.05);
        }
        assert_eq!(o.variant, 1);
        assert!((o.glow - 0.8).abs() < 1e-9);
        o.step(4, 0.8, [0.6, 0.3, 1.0], 0.05);
        assert_eq!(o.variant, 1, "fades out first");
        assert!(o.glow < 0.8);
        for _ in 0..100 {
            o.step(4, 0.8, [0.6, 0.3, 1.0], 0.05);
        }
        assert_eq!(o.variant, 4);
        assert!((o.glow - 0.8).abs() < 1e-9);
        assert!((o.spill[2] - 0.8).abs() < 1e-9);
    }

    #[test]
    fn the_livery_name_is_the_folder_under_liveries() {
        assert_eq!(livery_name("Aircraft/A380 for X-Plane 12/liveries/Emirates/"), Some("Emirates".into()));
        assert_eq!(livery_name("C:\\X-Plane 12\\Aircraft\\A380\\liveries\\Qantas\\"), Some("Qantas".into()));
        assert_eq!(livery_name("Aircraft/A380 for X-Plane 12/"), None);
        assert_eq!(livery_name(""), None);
    }

    #[test]
    fn the_ife_theme_takes_the_liverys_settings_and_images() {
        let dir = std::env::temp_dir().join("fbw-ife-theme-test");
        let theme = dir.join("theme");
        let livery = dir.join("livery");
        std::fs::create_dir_all(&theme).unwrap();
        std::fs::create_dir_all(&livery).unwrap();
        std::fs::write(livery.join("ife_logo.png"), b"png").unwrap();
        std::fs::write(theme.join("ife_background.png"), b"old").unwrap();
        write_theme(&theme, &serde_json::json!({"accent": "#C8102E"}), Some(&livery)).unwrap();
        let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(theme.join("theme.json")).unwrap()).unwrap();
        assert_eq!(v["accent"], "#C8102E");
        assert_eq!(v["ife_logo"], "ife_logo.png");
        assert!(theme.join("ife_logo.png").exists());
        assert!(std::fs::read_to_string(theme.join("theme.js")).unwrap().starts_with("window.IFE_THEME = {"));
        assert!(!theme.join("ife_background.png").exists());
    }
}
