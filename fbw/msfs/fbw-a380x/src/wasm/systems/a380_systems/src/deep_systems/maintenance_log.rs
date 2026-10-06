use std::collections::{BTreeMap, BTreeSet};

use deep_systems::deep::breakers::catalog as breaker_catalog;
use deep_systems::deep::frame::DerivedFailure;
use systems::simulation::{InitContext, SimulatorWriter, VariableIdentifier, Writer};

pub const LOG_CAPACITY: usize = 64;

pub const KIND_FAILURE_ARMED: f64 = 1.;
pub const KIND_FAILURE_CLEARED: f64 = 2.;
pub const KIND_DERIVED_ACTIVE: f64 = 3.;
pub const KIND_DERIVED_CLEARED: f64 = 4.;
pub const KIND_BREAKER_TRIPPED: f64 = 5.;
pub const KIND_BREAKER_RESET: f64 = 6.;
pub const KIND_MEL_DEFERRED: f64 = 7.;
pub const KIND_MEL_RELEASED: f64 = 8.;

const ATA_UNKNOWN: u16 = 0;

#[derive(Clone, Copy, Debug, Default)]
struct Event {
    kind: f64,
    id: f64,
    time_s: f64,
    phase: f64,
    ata: f64,
}

struct SlotIds {
    kind: VariableIdentifier,
    id: VariableIdentifier,
    time: VariableIdentifier,
    phase: VariableIdentifier,
    ata: VariableIdentifier,
}

pub struct MaintenanceLog {
    events: [Event; LOG_CAPACITY],
    next: usize,
    len: usize,
    seq: u64,

    slots: Vec<SlotIds>,
    count_id: VariableIdentifier,
    seq_id: VariableIdentifier,

    failure_ata: BTreeMap<u64, u16>,
    unit_ata: Vec<u16>,

    ata_active_ids: BTreeMap<u16, VariableIdentifier>,

    prev_armed: BTreeSet<u64>,
    prev_derived: BTreeSet<u64>,
    prev_unit_open: Vec<bool>,
    prev_mel_deferred: BTreeSet<usize>,

    unit_catalogue_index: Vec<usize>,
}

impl MaintenanceLog {
    pub fn new(context: &mut InitContext, unit_ids: &[&str], unit_count: usize) -> Self {
        let slots = (0..LOG_CAPACITY)
            .map(|k| SlotIds {
                kind: context.get_identifier(format!("DEEP_MAINT_LOG_{k}_KIND")),
                id: context.get_identifier(format!("DEEP_MAINT_LOG_{k}_ID")),
                time: context.get_identifier(format!("DEEP_MAINT_LOG_{k}_TIME")),
                phase: context.get_identifier(format!("DEEP_MAINT_LOG_{k}_PHASE")),
                ata: context.get_identifier(format!("DEEP_MAINT_LOG_{k}_ATA")),
            })
            .collect();

        let registry = deep_systems::deep::registry();
        let failure_ata: BTreeMap<u64, u16> = registry.failures.iter().map(|f| (f.id, f.ata)).collect();

        let mut sorted: Vec<&str> = unit_ids.to_vec();
        sorted.sort_unstable();
        let unit_catalogue_index: Vec<usize> = unit_ids.iter().map(|id| sorted.binary_search(id).unwrap_or(0)).collect();

        let breaker_defs = breaker_catalog::all();
        let unit_ata: Vec<u16> = unit_ids
            .iter()
            .map(|id| breaker_defs.iter().find(|b| b.id == *id).map(|b| b.ata).unwrap_or(ATA_UNKNOWN))
            .collect();

        let mut chapters: BTreeSet<u16> = failure_ata.values().copied().collect();
        chapters.extend(unit_ata.iter().copied());
        let ata_active_ids = chapters
            .into_iter()
            .filter(|&ata| ata != ATA_UNKNOWN)
            .map(|ata| (ata, context.get_identifier(format!("DEEP_MAINT_ATA_{ata:02}_ACTIVE"))))
            .collect();

        Self {
            events: [Event::default(); LOG_CAPACITY],
            next: 0,
            len: 0,
            seq: 0,
            slots,
            count_id: context.get_identifier("DEEP_MAINT_LOG_COUNT".to_owned()),
            seq_id: context.get_identifier("DEEP_MAINT_LOG_SEQ".to_owned()),
            failure_ata,
            unit_ata,
            ata_active_ids,
            prev_armed: BTreeSet::new(),
            prev_derived: BTreeSet::new(),
            prev_unit_open: vec![false; unit_count],
            prev_mel_deferred: BTreeSet::new(),
            unit_catalogue_index,
        }
    }

    fn push(&mut self, kind: f64, id: f64, time_s: f64, phase: f64, ata: u16) {
        self.events[self.next] = Event { kind, id, time_s, phase, ata: ata as f64 };
        self.next = (self.next + 1) % LOG_CAPACITY;
        self.len = (self.len + 1).min(LOG_CAPACITY);
        self.seq += 1;
    }

    pub fn update(
        &mut self,
        zulu_s: f64,
        phase: f64,
        armed: &BTreeMap<u64, f64>,
        derived: &[DerivedFailure],
        unit_open: &[(bool, bool)],
        mel_deferred: &BTreeSet<usize>,
    ) {
        if zulu_s <= 0. {
            return;
        }
        let time_s = zulu_s;
        let now_armed: BTreeSet<u64> = armed.keys().copied().collect();
        let newly_armed: Vec<u64> = now_armed.difference(&self.prev_armed).copied().collect();
        let newly_cleared: Vec<u64> = self.prev_armed.difference(&now_armed).copied().collect();
        for id in newly_armed {
            let ata = self.failure_ata.get(&id).copied().unwrap_or(ATA_UNKNOWN);
            self.push(KIND_FAILURE_ARMED, id as f64, time_s, phase, ata);
        }
        for id in newly_cleared {
            let ata = self.failure_ata.get(&id).copied().unwrap_or(ATA_UNKNOWN);
            self.push(KIND_FAILURE_CLEARED, id as f64, time_s, phase, ata);
        }
        self.prev_armed = now_armed;

        let now_derived: BTreeSet<u64> = derived.iter().filter(|d| d.magnitude > 0.).map(|d| d.fbw_id).collect();
        let newly_active: Vec<u64> = now_derived.difference(&self.prev_derived).copied().collect();
        let newly_inactive: Vec<u64> = self.prev_derived.difference(&now_derived).copied().collect();
        for id in newly_active {
            let ata = self.failure_ata.get(&id).copied().unwrap_or(ATA_UNKNOWN);
            self.push(KIND_DERIVED_ACTIVE, id as f64, time_s, phase, ata);
        }
        for id in newly_inactive {
            let ata = self.failure_ata.get(&id).copied().unwrap_or(ATA_UNKNOWN);
            self.push(KIND_DERIVED_CLEARED, id as f64, time_s, phase, ata);
        }
        self.prev_derived = now_derived;

        for (i, &(open, commanded)) in unit_open.iter().enumerate() {
            let was_open = self.prev_unit_open.get(i).copied().unwrap_or(false);
            let ata = self.unit_ata.get(i).copied().unwrap_or(ATA_UNKNOWN);
            if open && !was_open && !commanded {
                let id = self.unit_catalogue_index.get(i).copied().unwrap_or(i) as f64;
                self.push(KIND_BREAKER_TRIPPED, id, time_s, phase, ata);
            } else if !open && was_open {
                let id = self.unit_catalogue_index.get(i).copied().unwrap_or(i) as f64;
                self.push(KIND_BREAKER_RESET, id, time_s, phase, ata);
            }
            if let Some(slot) = self.prev_unit_open.get_mut(i) {
                *slot = open;
            }
        }

        let newly_deferred: Vec<usize> = mel_deferred.difference(&self.prev_mel_deferred).copied().collect();
        let newly_released: Vec<usize> = self.prev_mel_deferred.difference(mel_deferred).copied().collect();
        for item in newly_deferred {
            self.push(KIND_MEL_DEFERRED, item as f64, time_s, phase, ATA_UNKNOWN);
        }
        for item in newly_released {
            self.push(KIND_MEL_RELEASED, item as f64, time_s, phase, ATA_UNKNOWN);
        }
        self.prev_mel_deferred = mel_deferred.clone();
    }

    fn active_counts(&self) -> BTreeMap<u16, u32> {
        let mut counts: BTreeMap<u16, u32> = BTreeMap::new();
        for &id in &self.prev_armed {
            let ata = self.failure_ata.get(&id).copied().unwrap_or(ATA_UNKNOWN);
            if ata != ATA_UNKNOWN {
                *counts.entry(ata).or_insert(0) += 1;
            }
        }
        for &id in &self.prev_derived {
            let ata = self.failure_ata.get(&id).copied().unwrap_or(ATA_UNKNOWN);
            if ata != ATA_UNKNOWN {
                *counts.entry(ata).or_insert(0) += 1;
            }
        }
        counts
    }

    pub fn write(&self, writer: &mut SimulatorWriter) {
        writer.write_f64(&self.count_id, self.len as f64);
        writer.write_f64(&self.seq_id, self.seq as f64);
        for k in 0..LOG_CAPACITY {
            let slot = &self.slots[k];
            if k < self.len {
                let idx = (self.next + LOG_CAPACITY - 1 - k) % LOG_CAPACITY;
                let e = &self.events[idx];
                writer.write_f64(&slot.kind, e.kind);
                writer.write_f64(&slot.id, e.id);
                writer.write_f64(&slot.time, e.time_s);
                writer.write_f64(&slot.phase, e.phase);
                writer.write_f64(&slot.ata, e.ata);
            } else {
                writer.write_f64(&slot.kind, 0.);
                writer.write_f64(&slot.id, 0.);
                writer.write_f64(&slot.time, 0.);
                writer.write_f64(&slot.phase, 0.);
                writer.write_f64(&slot.ata, 0.);
            }
        }
        for (&ata, id) in &self.ata_active_ids {
            let count = self.active_counts().get(&ata).copied().unwrap_or(0);
            writer.write_f64(id, count as f64);
        }
    }
}
