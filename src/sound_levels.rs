use std::ffi::c_void;
use std::sync::atomic::{AtomicU32, Ordering};

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::Vars;

static FBW_GAIN: AtomicU32 = AtomicU32::new(0x3f80_0000);

pub fn fbw_gain() -> f32 {
    f32::from_bits(FBW_GAIN.load(Ordering::Relaxed))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Group {
    Engines,
    Airframe,
    Weather,
    Cockpit,
    Atc,
    Cabin,
}

const GROUPS: [Group; 6] = [Group::Engines, Group::Airframe, Group::Weather, Group::Cockpit, Group::Atc, Group::Cabin];

pub const SETTINGS: [&str; 8] = [
    "SOUND_LEVEL_MASTER",
    "SOUND_LEVEL_ENGINES",
    "SOUND_LEVEL_AIRFRAME",
    "SOUND_LEVEL_WEATHER",
    "SOUND_LEVEL_COCKPIT",
    "SOUND_LEVEL_ATC",
    "SOUND_LEVEL_CABIN",
    "SOUND_LEVEL_FBW",
];

pub fn group_of(path: &str) -> Option<Group> {
    let p = path.strip_prefix("event:/")?;
    Some(if p.starts_with("Engines/") {
        Group::Engines
    } else if p.starts_with("Dynamics/") {
        Group::Airframe
    } else if p.starts_with("Weather/") {
        Group::Weather
    } else if p.starts_with("Cockpit/ATC") {
        Group::Atc
    } else if p.starts_with("Cockpit/") || p.starts_with("Warning/") {
        Group::Cockpit
    } else if p.starts_with("PA/") {
        Group::Cabin
    } else {
        return None;
    })
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Guid {
    d1: u32,
    d2: u16,
    d3: u16,
    d4: [u8; 8],
}

pub fn parse_guid(s: &str) -> Option<Guid> {
    let h: Vec<&str> = s.trim_matches(|c| c == '{' || c == '}').split('-').collect();
    if h.len() != 5 || h[3].len() != 4 || h[4].len() != 12 {
        return None;
    }
    let tail = format!("{}{}", h[3], h[4]);
    let mut d4 = [0u8; 8];
    for (i, b) in d4.iter_mut().enumerate() {
        *b = u8::from_str_radix(tail.get(i * 2..i * 2 + 2)?, 16).ok()?;
    }
    Some(Guid { d1: u32::from_str_radix(h[0], 16).ok()?, d2: u16::from_str_radix(h[1], 16).ok()?, d3: u16::from_str_radix(h[2], 16).ok()?, d4 })
}

pub fn events(guids_txt: &str) -> Vec<(Guid, Group)> {
    guids_txt
        .lines()
        .filter_map(|line| {
            let (guid, path) = line.trim().split_once(' ')?;
            Some((parse_guid(guid)?, group_of(path.trim())?))
        })
        .collect()
}

type GetStudio = unsafe extern "C" fn() -> *mut c_void;
type GetEventById = unsafe extern "C" fn(*mut c_void, *const Guid, *mut *mut c_void) -> i32;
type GetInstanceList = unsafe extern "C" fn(*mut c_void, *mut *mut c_void, i32, *mut i32) -> i32;
type SetVolume = unsafe extern "C" fn(*mut c_void, f32) -> i32;

#[cfg(windows)]
extern "system" {
    fn GetModuleHandleW(name: *const u16) -> *mut c_void;
    fn GetProcAddress(module: *mut c_void, name: *const u8) -> *mut c_void;
}

struct Fmod {
    studio: GetStudio,
    by_id: GetEventById,
    instances: GetInstanceList,
    set_volume: SetVolume,
}

impl Fmod {
    #[cfg(windows)]
    fn load() -> Option<Fmod> {
        let studio = crate::xp::Xplm::xplm_symbol("XPLMGetFMODStudio")?;
        let name: Vec<u16> = "fmodstudio.dll\0".encode_utf16().collect();
        let module = unsafe { GetModuleHandleW(name.as_ptr()) };
        if module.is_null() {
            return None;
        }
        let f = |n: &str| {
            let p = unsafe { GetProcAddress(module, format!("{n}\0").as_ptr()) };
            (!p.is_null()).then_some(p)
        };
        unsafe {
            Some(Fmod {
                studio: std::mem::transmute::<*mut c_void, GetStudio>(studio),
                by_id: std::mem::transmute::<*mut c_void, GetEventById>(f("FMOD_Studio_System_GetEventByID")?),
                instances: std::mem::transmute::<*mut c_void, GetInstanceList>(f("FMOD_Studio_EventDescription_GetInstanceList")?),
                set_volume: std::mem::transmute::<*mut c_void, SetVolume>(f("FMOD_Studio_EventInstance_SetVolume")?),
            })
        }
    }

    #[cfg(not(windows))]
    fn load() -> Option<Fmod> {
        None
    }
}

pub struct SoundLevels {
    ids: [VariableIdentifier; 8],
    events: Vec<(Guid, Group)>,
    descriptions: Vec<(*mut c_void, Group)>,
    fmod: Option<Fmod>,
    applied: [f32; 6],
    resolved: bool,
}

impl SoundLevels {
    pub fn new(vars: &mut Vars) -> Self {
        let events = crate::sound_pack::plugin_dir()
            .and_then(|d| Some(d.parent()?.parent()?.join("fmod").join("GUIDs.txt")))
            .and_then(|p| std::fs::read_to_string(p).ok())
            .map(|t| events(&t))
            .unwrap_or_default();
        SoundLevels {
            ids: SETTINGS.map(|s| vars.get(s.to_owned())),
            fmod: if events.is_empty() { None } else { Fmod::load() },
            events,
            descriptions: Vec::new(),
            applied: [1.0; 6],
            resolved: false,
        }
    }

    pub fn update(&mut self, vars: &mut Vars) {
        let level = |vars: &mut Vars, i: usize| (vars.read(&self.ids[i]) / 100.).clamp(0., 1.) as f32;
        let master = level(vars, 0);
        FBW_GAIN.store((master * level(vars, 7)).to_bits(), Ordering::Relaxed);
        let wanted: [f32; 6] = std::array::from_fn(|g| master * level(vars, g + 1));
        let Some(fmod) = &self.fmod else { return };
        if !self.resolved {
            let system = unsafe { (fmod.studio)() };
            if system.is_null() {
                return;
            }
            self.descriptions.clear();
            for (guid, group) in &self.events {
                let mut d = std::ptr::null_mut();
                if unsafe { (fmod.by_id)(system, guid, &mut d) } == 0 && !d.is_null() {
                    self.descriptions.push((d, *group));
                }
            }
            if self.descriptions.is_empty() {
                return;
            }
            crate::log(&format!("sound levels: {} of the pack's {} events found", self.descriptions.len(), self.events.len()));
            self.resolved = true;
        }
        if wanted.iter().all(|&w| w == 1.0) && self.applied.iter().all(|&a| a == 1.0) {
            return;
        }
        let mut list = [std::ptr::null_mut(); 32];
        for (d, group) in &self.descriptions {
            let volume = wanted[GROUPS.iter().position(|g| g == group).unwrap_or(0)];
            let mut n = 0;
            if unsafe { (fmod.instances)(*d, list.as_mut_ptr(), list.len() as i32, &mut n) } != 0 {
                continue;
            }
            for &instance in &list[..n.clamp(0, list.len() as i32) as usize] {
                unsafe { (fmod.set_volume)(instance, volume) };
            }
        }
        self.applied = wanted;
    }

    pub fn release(&mut self) {
        FBW_GAIN.store(1.0f32.to_bits(), Ordering::Relaxed);
        self.descriptions.clear();
        self.resolved = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_guid_reads_the_way_fmod_lays_it_out() {
        let g = parse_guid("{4c2ed8b8-04c4-42b7-b0cb-ab422a5a2197}").unwrap();
        assert_eq!(g, Guid { d1: 0x4c2ed8b8, d2: 0x04c4, d3: 0x42b7, d4: [0xb0, 0xcb, 0xab, 0x42, 0x2a, 0x5a, 0x21, 0x97] });
        assert_eq!(parse_guid("{not-a-guid}"), None);
    }

    #[test]
    fn each_pack_event_falls_in_the_group_its_slider_names() {
        assert_eq!(group_of("event:/Engines/RR/EngineA"), Some(Group::Engines));
        assert_eq!(group_of("event:/Dynamics/GearDpL"), Some(Group::Airframe));
        assert_eq!(group_of("event:/Weather/WindA"), Some(Group::Weather));
        assert_eq!(group_of("event:/Cockpit/ATCGRD"), Some(Group::Atc));
        assert_eq!(group_of("event:/Cockpit/PacksL"), Some(Group::Cockpit));
        assert_eq!(group_of("event:/PA/Boarding"), Some(Group::Cabin));
        assert_eq!(group_of("bus:/Master/Interior"), None, "buses are not ours to set");
        assert_eq!(group_of("snapshot:/No Ext"), None);
    }

    #[test]
    fn the_guid_list_gives_every_event_and_nothing_else() {
        let txt = "{4c2ed8b8-04c4-42b7-b0cb-ab422a5a2197} event:/Cockpit/ATCAPP\n{9f08114d-36ec-44cc-b8bf-b2babda09f9e} bus:/\n{d75bdf04-b203-4d0c-a463-ddb8141bf4d7} snapshot:/No Ext\n{83710756-8c4b-4c1c-9358-d927cd70c12b} event:/Engines/RR/EngineA\n";
        let e = events(txt);
        assert_eq!(e.len(), 2);
        assert_eq!(e[0].1, Group::Atc);
        assert_eq!(e[1].1, Group::Engines);
    }
}
