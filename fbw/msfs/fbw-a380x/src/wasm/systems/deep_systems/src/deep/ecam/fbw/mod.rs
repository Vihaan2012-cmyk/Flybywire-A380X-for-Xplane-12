use crate::deep::api::{Cond, Level};

pub mod ata21_22_23;
pub mod ata24;
pub mod ata26;
pub mod ata27;
pub mod ata46_49_52_56;
pub mod ata28;
pub mod ata29;
pub mod ata31_33;
pub mod ata32;
pub mod ata34;
pub mod ata70;
pub mod ata_cheap_wins;
pub mod generated;
pub mod wave5_w_ata27_1;
pub mod wave5_e03_compressor;
pub mod wave3_titles3;

pub mod sd_page {
    pub const ENG: i32 = 0;
    pub const APU: i32 = 1;
    pub const BLEED: i32 = 2;
    pub const COND: i32 = 3;
    pub const PRESS: i32 = 4;
    pub const DOOR: i32 = 5;
    pub const ELEC_AC: i32 = 6;
    pub const ELEC_DC: i32 = 7;
    pub const FUEL: i32 = 8;
    pub const WHEEL: i32 = 9;
    pub const HYD: i32 = 10;
    pub const FCTL: i32 = 11;
    pub const CB: i32 = 12;
    pub const CRZ: i32 = 13;
    pub const STATUS: i32 = 14;
}

pub mod phase {
    pub const ELEC_PWR: u32 = 1;
    pub const FIRST_ENG_STARTED: u32 = 2;
    pub const SECOND_ENG_TO_POWER: u32 = 3;
    pub const AT_OR_ABOVE_80_KT: u32 = 4;
    pub const AT_OR_ABOVE_V1: u32 = 5;
    pub const LIFT_OFF: u32 = 6;
    pub const AT_OR_ABOVE_400_FT: u32 = 7;
    pub const AT_OR_ABOVE_1500_FT: u32 = 8;
    pub const AT_OR_BELOW_800_FT: u32 = 9;
    pub const TOUCH_DOWN: u32 = 10;
    pub const AT_OR_BELOW_80_KT: u32 = 11;
    pub const ENGINES_SHUTDOWN: u32 = 12;

    pub const TAKEOFF_AND_LANDING: &[u32] = &[3, 4, 5, 6, 7, 9, 10];

    pub const TAKEOFF_AND_LANDING_ROLL: &[u32] = &[4, 5, 6, 7, 9, 10];

    pub const NONE: &[u32] = &[];

    pub const ENG_56: &[u32] = &[5, 6];
}

#[derive(Clone, Debug)]
pub struct FbwItem {
    pub index: usize,
    pub show: Option<Cond>,
    pub checked: Option<Cond>,
}

pub fn item(index: usize) -> FbwItem {
    FbwItem { index, show: None, checked: None }
}

impl FbwItem {
    pub fn checked(mut self, c: Cond) -> Self {
        self.checked = Some(c);
        self
    }
    pub fn shown_if(mut self, c: Cond) -> Self {
        self.show = Some(c);
        self
    }
}

#[derive(Clone, Debug)]
pub struct FbwProc {
    pub id: u64,
    pub title: &'static str,
    pub level: Level,
    pub sys_page: i32,
    pub inhibit: &'static [u32],
    pub confirm_s: f64,
    pub trigger: Cond,
    pub suppressed_by: &'static [u64],
    pub items: Vec<FbwItem>,
    pub item_count: usize,
    pub note: &'static str,
}

pub fn proc(id: u64, title: &'static str, level: Level, sys_page: i32, trigger: Cond, note: &'static str) -> FbwProc {
    FbwProc {
        id,
        title,
        level,
        sys_page,
        inhibit: phase::TAKEOFF_AND_LANDING,
        confirm_s: 0.0,
        trigger,
        suppressed_by: &[],
        items: Vec::new(),
        item_count: 0,
        note,
    }
}

impl FbwProc {
    pub fn confirm(mut self, s: f64) -> Self {
        self.confirm_s = s;
        self
    }
    pub fn inhibit(mut self, phases: &'static [u32]) -> Self {
        self.inhibit = phases;
        self
    }
    pub fn suppressed_by(mut self, ids: &'static [u64]) -> Self {
        self.suppressed_by = ids;
        self
    }
    pub fn items(mut self, count: usize, items: Vec<FbwItem>) -> Self {
        self.item_count = count;
        self.items = items;
        self
    }
}

pub fn wirings() -> Vec<FbwProc> {
    let mut v = Vec::new();
    ata21_22_23::wire(&mut v);
    ata24::wire(&mut v);
    ata26::wire(&mut v);
    ata27::wire(&mut v);
    ata46_49_52_56::wire(&mut v);
    ata28::wire(&mut v);
    ata29::wire(&mut v);
    ata31_33::wire(&mut v);
    ata32::wire(&mut v);
    ata34::wire(&mut v);
    ata70::wire(&mut v);
    ata_cheap_wins::wire(&mut v);
    for g in generated::procs().into_iter().chain(wave5_e03_compressor::procs()).chain(wave5_w_ata27_1::procs()).chain(wave3_titles3::procs()) {
        match v.iter_mut().find(|p| p.id == g.id) {
            Some(p) => p.trigger = Cond::Or(vec![p.trigger.clone(), g.trigger]),
            None => v.push(g),
        }
    }
    v.sort_by_key(|p| p.id);
    v
}
