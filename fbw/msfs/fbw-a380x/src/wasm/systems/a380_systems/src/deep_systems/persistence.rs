use std::collections::BTreeMap;
use std::io::Write as _;
use std::path::{Path, PathBuf};

use deep_systems::random_failures::{Config as RandomConfig, RandomFailures};
use deep_systems::scripted_failures::{ArmCondition, Phase, Scripted, ScriptedTrigger};
use deep_systems::wear::{Wear, WearStore};

const CURRENT_VERSION: u32 = 1;

const SAVE_INTERVAL_S: f64 = 120.0;

pub(super) const FILE_NAME: &str = "a380x_deep_airframe.toml";

#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct WearEntry {
    pub hot_hours: f64,
    pub cycles: u32,
    pub thermal_stress_integral: f64,
    pub degradation_fraction: f64,
}

fn wear_to_entry(w: Wear) -> WearEntry {
    WearEntry { hot_hours: w.hot_hours, cycles: w.cycles, thermal_stress_integral: w.thermal_stress_integral, degradation_fraction: w.degradation_fraction }
}

fn entry_to_wear(e: WearEntry) -> Wear {
    Wear { hot_hours: e.hot_hours, cycles: e.cycles, thermal_stress_integral: e.thermal_stress_integral, degradation_fraction: e.degradation_fraction }
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum ArmConditionEntry {
    ElapsedHours(f64),
    AboveAltitudeFt(f64),
    BelowAltitudeFt(f64),
    AboveSpeedKt(f64),
    OnFlightPhase(u8),
    AboveIasKt(f64),
    AboveRadioHeightFt(f64),
    BelowRadioHeightFt(f64, bool),
    SecondsAfterLiftoff(f64),
}

pub(super) fn phase_to_u8(p: Phase) -> u8 {
    match p {
        Phase::Preflight => 0,
        Phase::Taxi => 1,
        Phase::Takeoff => 2,
        Phase::Climb => 3,
        Phase::Cruise => 4,
        Phase::Descent => 5,
        Phase::Approach => 6,
        Phase::GoAround => 7,
    }
}

fn condition_to_entry(c: ArmCondition) -> ArmConditionEntry {
    match c {
        ArmCondition::ElapsedHours(h) => ArmConditionEntry::ElapsedHours(h),
        ArmCondition::AboveAltitudeFt(ft) => ArmConditionEntry::AboveAltitudeFt(ft),
        ArmCondition::BelowAltitudeFt(ft) => ArmConditionEntry::BelowAltitudeFt(ft),
        ArmCondition::AboveSpeedKt(kt) => ArmConditionEntry::AboveSpeedKt(kt),
        ArmCondition::OnFlightPhase(p) => ArmConditionEntry::OnFlightPhase(phase_to_u8(p)),
        ArmCondition::AboveIasKt(kt) => ArmConditionEntry::AboveIasKt(kt),
        ArmCondition::AboveRadioHeightFt(ft) => ArmConditionEntry::AboveRadioHeightFt(ft),
        ArmCondition::BelowRadioHeightFt { ft, seen_above } => ArmConditionEntry::BelowRadioHeightFt(ft, seen_above),
        ArmCondition::SecondsAfterLiftoff(t) => ArmConditionEntry::SecondsAfterLiftoff(t),
    }
}

fn entry_to_condition(e: ArmConditionEntry) -> Option<ArmCondition> {
    Some(match e {
        ArmConditionEntry::ElapsedHours(h) => ArmCondition::ElapsedHours(h),
        ArmConditionEntry::AboveAltitudeFt(ft) => ArmCondition::AboveAltitudeFt(ft),
        ArmConditionEntry::BelowAltitudeFt(ft) => ArmCondition::BelowAltitudeFt(ft),
        ArmConditionEntry::AboveSpeedKt(kt) => ArmCondition::AboveSpeedKt(kt),
        ArmConditionEntry::OnFlightPhase(n) => ArmCondition::OnFlightPhase(Phase::from_fmgc(n as f64)?),
        ArmConditionEntry::AboveIasKt(kt) => ArmCondition::AboveIasKt(kt),
        ArmConditionEntry::AboveRadioHeightFt(ft) => ArmCondition::AboveRadioHeightFt(ft),
        ArmConditionEntry::BelowRadioHeightFt(ft, seen_above) => ArmCondition::BelowRadioHeightFt { ft, seen_above },
        ArmConditionEntry::SecondsAfterLiftoff(t) => ArmCondition::SecondsAfterLiftoff(t),
    })
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ScriptedEntry {
    pub id: u64,
    pub condition: ArmConditionEntry,
    #[serde(default = "one")]
    pub magnitude: f64,
    #[serde(default)]
    pub external: bool,
}

fn one() -> f64 {
    1.0
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct AirframeState {
    pub version: u32,
    pub airframe_hours: f64,
    #[serde(default)]
    pub armed_failures: BTreeMap<u64, f64>,
    #[serde(default)]
    pub wear: BTreeMap<String, WearEntry>,
    #[serde(default)]
    pub random_enabled: bool,
    #[serde(default = "one")]
    pub random_rate_multiplier: f64,
    #[serde(default)]
    pub random_rng_seed: u64,
    #[serde(default)]
    pub scripted_pending: Vec<ScriptedEntry>,
    #[serde(default)]
    pub mel_deferred_items: std::collections::BTreeSet<usize>,
}

impl Default for AirframeState {
    fn default() -> Self {
        Self {
            version: CURRENT_VERSION,
            airframe_hours: 0.0,
            armed_failures: BTreeMap::new(),
            wear: BTreeMap::new(),
            random_enabled: false,
            random_rate_multiplier: 1.0,
            random_rng_seed: 0x1234_5678_9abc_def0,
            scripted_pending: Vec::new(),
            mel_deferred_items: std::collections::BTreeSet::new(),
        }
    }
}

pub struct Persistence {
    path: Option<PathBuf>,
    since_save_s: f64,
    pub state: AirframeState,
}

impl Persistence {
    pub fn new(base_dir: &Path) -> Self {
        let path = base_dir.join(FILE_NAME);
        let state = Self::load(&path).unwrap_or_default();
        Self { path: Some(path), since_save_s: 0.0, state }
    }

    pub fn for_host() -> Self {
        #[cfg(target_arch = "wasm32")]
        {
            Self::new(&Self::work_dir())
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            Self { path: None, since_save_s: 0.0, state: AirframeState::default() }
        }
    }

    pub fn work_dir() -> PathBuf {
        #[cfg(target_arch = "wasm32")]
        {
            PathBuf::from("\\work")
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            static COUNTER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
            let n = COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            std::env::temp_dir().join(format!("a380x-deep-test-{}-{n}", std::process::id()))
        }
    }

    fn load(path: &Path) -> Option<AirframeState> {
        Self::load_one(path).or_else(|| Self::load_one(&Self::backup_path(path)))
    }

    fn backup_path(path: &Path) -> PathBuf {
        path.with_extension("toml.bak")
    }

    fn load_one(path: &Path) -> Option<AirframeState> {
        let text = read_plain(path).ok()?;
        match toml::from_str::<AirframeState>(&text) {
            Ok(state) if state.version == CURRENT_VERSION => Some(state),
            Ok(_) => {
                deep_systems::log("a380x_deep_airframe.toml is from a different version; starting a new airframe");
                None
            }
            Err(e) => {
                deep_systems::log(&format!("a380x_deep_airframe.toml is corrupt ({e}); starting a new airframe"));
                None
            }
        }
    }

    pub fn save(&self) -> std::io::Result<()> {
        let Some(path) = &self.path else {
            return Ok(());
        };
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir)?;
        }
        let text = toml::to_string(&self.state).map_err(std::io::Error::other)?;
        write_plain(&Self::backup_path(path), &text)?;
        write_plain(path, &text)
    }

    pub fn tick(&mut self, delta_hours: f64, delta_s: f64) {
        self.state.airframe_hours += delta_hours;
        self.since_save_s += delta_s;
        if self.since_save_s >= SAVE_INTERVAL_S {
            self.since_save_s = 0.0;
            if let Err(e) = self.save() {
                deep_systems::log(&format!("could not save a380x_deep_airframe.toml: {e}"));
            }
        }
    }

    pub fn reset(&mut self) {
        self.state = AirframeState::default();
        let _ = self.save();
    }

    pub fn apply_to(&self, random: &mut RandomFailures, scripted: &mut Scripted, wear: &mut WearStore) -> BTreeMap<u64, f64> {
        random.restore(RandomConfig { enabled: self.state.random_enabled, rate_multiplier: self.state.random_rate_multiplier }, self.state.random_rng_seed);
        let pending: Vec<ScriptedTrigger> = self
            .state
            .scripted_pending
            .iter()
            .filter_map(|e| entry_to_condition(e.condition).map(|condition| ScriptedTrigger { id: e.id, condition, magnitude: e.magnitude, external: e.external }))
            .collect();
        scripted.restore(pending);
        for (id, entry) in &self.state.wear {
            wear.set(id, entry_to_wear(*entry));
        }
        self.state.armed_failures.clone()
    }

    pub fn deferred_mel_items(&self) -> std::collections::BTreeSet<usize> {
        self.state.mel_deferred_items.clone()
    }

    pub fn capture_from(
        &mut self,
        armed_failures: &BTreeMap<u64, f64>,
        random: &RandomFailures,
        scripted: &Scripted,
        wear: &WearStore,
        mel_deferred_items: &std::collections::BTreeSet<usize>,
    ) {
        self.state.armed_failures = armed_failures.clone();
        self.state.mel_deferred_items = mel_deferred_items.clone();
        let (config, seed) = random.snapshot();
        self.state.random_enabled = config.enabled;
        self.state.random_rate_multiplier = config.rate_multiplier;
        self.state.random_rng_seed = seed;
        self.state.scripted_pending = scripted
            .snapshot()
            .into_iter()
            .map(|t| ScriptedEntry { id: t.id, condition: condition_to_entry(t.condition), magnitude: t.magnitude, external: t.external })
            .collect();
        self.state.wear = wear.ids().map(|id| (id.to_owned(), wear_to_entry(wear.get(id)))).collect();
    }
}

fn write_plain(path: &Path, text: &str) -> std::io::Result<()> {
    let mut f = std::fs::File::create(path)?;
    f.write_all(text.as_bytes())
}

fn read_plain(path: &Path) -> std::io::Result<String> {
    use std::io::Read as _;
    let mut f = std::fs::File::open(path)?;
    let mut bytes = Vec::new();
    let mut chunk = [0u8; 4096];
    loop {
        let n = f.read(&mut chunk)?;
        if n == 0 {
            break;
        }
        bytes.extend_from_slice(&chunk[..n]);
    }
    String::from_utf8(bytes).map_err(std::io::Error::other)
}
