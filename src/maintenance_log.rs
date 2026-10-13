use std::collections::{BTreeMap, BTreeSet};

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::xp::{DataRef, Xplm};
use crate::Vars;

const LOG_CAPACITY: usize = 64;

const KIND_FAILURE_ARMED: f64 = 1.;
const KIND_FAILURE_CLEARED: f64 = 2.;
const KIND_DERIVED_ACTIVE: f64 = 3.;
const KIND_DERIVED_CLEARED: f64 = 4.;
const KIND_BREAKER_TRIPPED: f64 = 5.;
const KIND_BREAKER_RESET: f64 = 6.;
const KIND_MEL_DEFERRED: f64 = 7.;
const KIND_MEL_RELEASED: f64 = 8.;

const CMD_OPEN: i64 = 1;
const CMD_CLOSE: i64 = 2;

#[derive(Clone, Copy, Default, Debug, PartialEq)]
struct Event {
    kind: f64,
    id: f64,
    time_s: f64,
    phase: f64,
    ata: f64,
}

struct Slot {
    kind: VariableIdentifier,
    id: VariableIdentifier,
    time: VariableIdentifier,
    phase: VariableIdentifier,
    ata: VariableIdentifier,
}

struct Breaker {
    id: &'static str,
    ata: u16,
    status: VariableIdentifier,
    cmd: VariableIdentifier,
}

struct MelItem {
    failures: &'static [u64],
    deferred: VariableIdentifier,
    remaining_hours: VariableIdentifier,
}

#[derive(Default)]
struct Log {
    events: Vec<Event>,
    seq: u64,
    prev_armed: BTreeSet<u64>,
    prev_derived: BTreeSet<u64>,
    prev_open: Vec<bool>,
    prev_mel: BTreeSet<usize>,
}

impl Log {
    fn push(&mut self, kind: f64, id: f64, time_s: f64, phase: f64, ata: u16) {
        self.events.insert(0, Event { kind, id, time_s, phase, ata: ata as f64 });
        self.events.truncate(LOG_CAPACITY);
        self.seq += 1;
    }

    #[allow(clippy::too_many_arguments)]
    fn update(
        &mut self,
        time_s: f64,
        phase: f64,
        armed: &BTreeSet<u64>,
        derived: &BTreeSet<u64>,
        breakers: &[(bool, bool, u16)],
        mel: &BTreeSet<usize>,
        ata_of: impl Fn(u64) -> u16,
    ) {
        if self.prev_open.len() != breakers.len() {
            self.prev_open = breakers.iter().map(|b| b.0).collect();
        }
        for &id in armed.difference(&self.prev_armed.clone()) {
            self.push(KIND_FAILURE_ARMED, id as f64, time_s, phase, ata_of(id));
        }
        for &id in self.prev_armed.clone().difference(armed) {
            self.push(KIND_FAILURE_CLEARED, id as f64, time_s, phase, ata_of(id));
        }
        self.prev_armed = armed.clone();
        for &id in derived.difference(&self.prev_derived.clone()) {
            self.push(KIND_DERIVED_ACTIVE, id as f64, time_s, phase, ata_of(id));
        }
        for &id in self.prev_derived.clone().difference(derived) {
            self.push(KIND_DERIVED_CLEARED, id as f64, time_s, phase, ata_of(id));
        }
        self.prev_derived = derived.clone();
        for (i, &(open, commanded, ata)) in breakers.iter().enumerate() {
            let was_open = self.prev_open[i];
            if open && !was_open && !commanded {
                self.push(KIND_BREAKER_TRIPPED, i as f64, time_s, phase, ata);
            } else if !open && was_open {
                self.push(KIND_BREAKER_RESET, i as f64, time_s, phase, ata);
            }
            self.prev_open[i] = open;
        }
        for &item in mel.difference(&self.prev_mel.clone()) {
            self.push(KIND_MEL_DEFERRED, item as f64, time_s, phase, 0);
        }
        for &item in self.prev_mel.clone().difference(mel) {
            self.push(KIND_MEL_RELEASED, item as f64, time_s, phase, 0);
        }
        self.prev_mel = mel.clone();
    }

    fn active_by_ata(&self, ata_of: impl Fn(u64) -> u16) -> BTreeMap<u16, u32> {
        let mut counts = BTreeMap::new();
        for &id in self.prev_armed.iter().chain(&self.prev_derived) {
            let ata = ata_of(id);
            if ata != 0 {
                *counts.entry(ata).or_insert(0) += 1;
            }
        }
        counts
    }
}

pub struct MaintenanceLog {
    log: Log,
    slots: Vec<Slot>,
    count: VariableIdentifier,
    seq: VariableIdentifier,
    breakers: Vec<Breaker>,
    mel: Vec<MelItem>,
    ata_active: BTreeMap<u16, VariableIdentifier>,
    phase: VariableIdentifier,
    zulu: Option<DataRef>,
}

fn ata_of(id: u64) -> u16 {
    crate::study::failures::ata_of(id) as u16
}

impl MaintenanceLog {
    pub fn new(vars: &mut Vars, xplm: &Xplm) -> Self {
        let slots = (0..LOG_CAPACITY)
            .map(|k| Slot {
                kind: vars.get(format!("A32NX_DEEP_MAINT_LOG_{k}_KIND")),
                id: vars.get(format!("A32NX_DEEP_MAINT_LOG_{k}_ID")),
                time: vars.get(format!("A32NX_DEEP_MAINT_LOG_{k}_TIME")),
                phase: vars.get(format!("A32NX_DEEP_MAINT_LOG_{k}_PHASE")),
                ata: vars.get(format!("A32NX_DEEP_MAINT_LOG_{k}_ATA")),
            })
            .collect();
        let mut defs: Vec<_> = crate::deep::breakers::catalog::all().iter().collect();
        defs.sort_by_key(|d| d.id);
        let breakers = defs
            .into_iter()
            .map(|d| Breaker {
                id: d.id,
                ata: d.ata,
                status: vars.get(format!("A32NX_BKR_{}_STATUS", d.id)),
                cmd: vars.get(format!("A32NX_BKR_{}_CMD", d.id)),
            })
            .collect::<Vec<_>>();
        let mut items: Vec<(&str, &'static [u64])> = crate::mel_catalog::MEL_FAILURES.to_vec();
        items.sort_by_key(|(r, _)| *r);
        let mel = items
            .into_iter()
            .enumerate()
            .map(|(k, (_, failures))| MelItem {
                failures,
                deferred: vars.get(format!("A32NX_DEEP_MEL_ITEM_{k}_DEFERRED")),
                remaining_hours: vars.get(format!("A32NX_DEEP_MEL_ITEM_{k}_REMAINING_HOURS")),
            })
            .collect();
        let mut chapters: BTreeSet<u16> = crate::failures::all_ids().into_iter().map(ata_of).collect();
        chapters.extend(breakers.iter().map(|b| b.ata));
        let ata_active = chapters
            .into_iter()
            .filter(|&a| a != 0)
            .map(|a| (a, vars.get(format!("A32NX_DEEP_MAINT_ATA_{a:02}_ACTIVE"))))
            .collect();
        Self {
            log: Log::default(),
            slots,
            count: vars.get("A32NX_DEEP_MAINT_LOG_COUNT".to_string()),
            seq: vars.get("A32NX_DEEP_MAINT_LOG_SEQ".to_string()),
            breakers,
            mel,
            ata_active,
            phase: vars.get("A32NX_FMGC_FLIGHT_PHASE".to_string()),
            zulu: xplm.find("sim/time/zulu_time_sec"),
        }
    }

    pub fn update(&mut self, vars: &mut Vars, xplm: &Xplm) {
        for b in &self.breakers {
            match vars.read(&b.cmd).round() as i64 {
                CMD_OPEN => crate::deep::breakers::live::request_pull(b.id),
                CMD_CLOSE => crate::deep::breakers::live::request_reset(b.id),
                0 => continue,
                _ => {}
            }
            vars.write(&b.cmd, 0.);
        }

        let armed: BTreeSet<u64> = crate::failures::armed_ids().into_iter().collect();
        let derived: BTreeSet<u64> = crate::failures::active_ids().into_iter().filter(|id| !armed.contains(id)).collect();
        let breakers: Vec<(bool, bool, u16)> = self
            .breakers
            .iter()
            .map(|b| {
                let status = vars.read(&b.status).round() as i64;
                (status != 0, status == 1, b.ata)
            })
            .collect();
        let mut mel_deferred = BTreeSet::new();
        for (k, item) in self.mel.iter().enumerate() {
            let remaining = item.failures.iter().filter_map(|&id| crate::mel::deferred_state(id)).reduce(f64::min);
            if remaining.is_some() {
                mel_deferred.insert(k);
            }
            vars.write(&item.deferred, if remaining.is_some() { 1. } else { 0. });
            vars.write(&item.remaining_hours, remaining.unwrap_or(0.));
        }
        let time_s = self.zulu.map_or(0., |d| xplm.get_f(d) as f64);
        let phase = vars.read(&self.phase);
        self.log.update(time_s, phase, &armed, &derived, &breakers, &mel_deferred, ata_of);

        vars.write(&self.count, self.log.events.len() as f64);
        vars.write(&self.seq, self.log.seq as f64);
        for (k, slot) in self.slots.iter().enumerate() {
            let e = self.log.events.get(k).copied().unwrap_or_default();
            vars.write(&slot.kind, e.kind);
            vars.write(&slot.id, e.id);
            vars.write(&slot.time, e.time_s);
            vars.write(&slot.phase, e.phase);
            vars.write(&slot.ata, e.ata);
        }
        let counts = self.log.active_by_ata(ata_of);
        for (ata, id) in &self.ata_active {
            vars.write(id, counts.get(ata).copied().unwrap_or(0) as f64);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ata(id: u64) -> u16 {
        (id / 1000) as u16
    }

    #[test]
    fn the_log_records_failures_breakers_and_mel_newest_first() {
        let mut log = Log::default();
        let none = BTreeSet::new();
        log.update(10., 2., &none, &none, &[(false, false, 24)], &BTreeSet::new(), ata);
        assert!(log.events.is_empty(), "a quiet aircraft logs nothing");
        log.update(20., 2., &BTreeSet::from([21001]), &none, &[(true, false, 24)], &BTreeSet::new(), ata);
        log.update(30., 3., &none, &BTreeSet::from([29000]), &[(false, false, 24)], &BTreeSet::from([5]), ata);
        let kinds: Vec<f64> = log.events.iter().map(|e| e.kind).collect();
        assert_eq!(kinds, vec![KIND_MEL_DEFERRED, KIND_BREAKER_RESET, KIND_DERIVED_ACTIVE, KIND_FAILURE_CLEARED, KIND_BREAKER_TRIPPED, KIND_FAILURE_ARMED]);
        assert_eq!(log.seq, 6);
        assert_eq!(log.events[3].ata, 21.);
        assert_eq!(log.active_by_ata(ata).get(&29), Some(&1));
    }

    #[test]
    fn a_crew_opened_breaker_is_not_logged_as_a_trip() {
        let mut log = Log::default();
        let none = BTreeSet::new();
        log.update(1., 0., &none, &none, &[(false, false, 24)], &BTreeSet::new(), ata);
        log.update(2., 0., &none, &none, &[(true, true, 24)], &BTreeSet::new(), ata);
        assert!(log.events.is_empty());
    }

    #[test]
    fn the_log_keeps_only_the_latest_sixty_four_events() {
        let mut log = Log::default();
        let none = BTreeSet::new();
        for i in 0..100u64 {
            let armed = if i % 2 == 0 { BTreeSet::from([21001]) } else { BTreeSet::new() };
            log.update(i as f64, 0., &armed, &none, &[], &BTreeSet::new(), ata);
        }
        assert_eq!(log.events.len(), LOG_CAPACITY);
        assert_eq!(log.seq, 100);
        assert_eq!(log.events[0].time_s, 99.);
    }
}
