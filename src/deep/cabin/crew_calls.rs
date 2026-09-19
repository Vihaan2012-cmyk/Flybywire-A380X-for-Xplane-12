//! Cabin crew calls: the discrete interphone/call-system events a flight
//! crew receives from the cabin — attendant, purser, emergency and cockpit
//! calls, plus automatic calls this directory's other models raise (a
//! galley or IFE smoke detection, a waste tank reaching full, a potable
//! water system fault) — with the real A380 interphone's priority ordering
//! (emergency over purser over a normal attendant call) and one active
//! "highest priority unacknowledged call" state for a simple annunciator.
//!
//! This is deliberately a different layer from `deep::api`'s `EcamAlert`/
//! `Ecam` (registered per module in `registry.rs`): ECAM shows synoptic/
//! procedural alerts on the displays, while the cabin interphone's call
//! system is a separate, real piece of aircraft equipment (call lights and
//! a distinct chime at the purser's station and on the flight deck) that
//! this module models directly, taking every other module's *own* output
//! only as plain data (`CabinSnapshot`) so this directory's modules stay
//! mutually decoupled, per the deep systems brief's self-containment rule.
//!
//! **Sourcing:** cabin interphone/call systems with a flight-deck "master
//! call" annunciator and a real priority order (emergency highest, purser
//! next, a normal attendant-to-attendant or attendant-to-cockpit call
//! lowest) are standard, widely documented transport-aircraft equipment
//! (ATA 23/44 cabin communications descriptions in publicly available
//! type-training material); no A380-specific priority table or timing is
//! public, so the exact ordering and the "held until acknowledged" latch
//! behaviour here are `GENERIC`, chosen to match that commonly described
//! architecture.

use super::Zone;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum CallPriority {
    Normal,
    Purser,
    Emergency,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SmokeSource {
    Galley(Zone),
    Ife(Zone),
}

#[derive(Clone, Debug, PartialEq)]
pub enum CabinEvent {
    AttendantCall(Zone),
    PurserCall,
    EmergencyCall,
    /// The cockpit calling the cabin (the one call direction this system
    /// also carries the other way).
    CockpitCall,
    Smoke(SmokeSource),
    WasteTankFull(Zone),
    WaterSystemFault,
    DoorNotLatchedDisagree { door_index: usize },
    SlideLowPressure { door_index: usize },
}

impl CabinEvent {
    pub fn priority(&self) -> CallPriority {
        match self {
            CabinEvent::EmergencyCall | CabinEvent::Smoke(_) => CallPriority::Emergency,
            CabinEvent::PurserCall | CabinEvent::WasteTankFull(_) | CabinEvent::DoorNotLatchedDisagree { .. } | CabinEvent::SlideLowPressure { .. } => {
                CallPriority::Purser
            }
            CabinEvent::AttendantCall(_) | CabinEvent::CockpitCall | CabinEvent::WaterSystemFault => CallPriority::Normal,
        }
    }
}

/// Momentary call-button presses this tick (edge-triggered internally: a
/// button held down for several ticks raises one event, not one per tick).
#[derive(Clone, Debug, Default)]
pub struct CrewCallInputs {
    pub attendant_call_pressed: [bool; Zone::COUNT],
    pub purser_call_pressed: bool,
    pub emergency_call_pressed: bool,
    pub cockpit_call_pressed: bool,
}

/// Plain-data readout of the rest of this directory's models, filled by
/// whoever wires them together — this module takes no dependency on
/// `water`/`waste`/`ife`/`galley`/`doors_slides`'s own types.
#[derive(Clone, Debug, Default)]
pub struct CabinSnapshot {
    pub oven_smoke: [bool; Zone::COUNT],
    pub ife_zone_smoke: [bool; Zone::COUNT],
    pub waste_tank_full: [bool; Zone::COUNT],
    pub water_system_fault: bool,
    /// Per door index (whatever numbering the door model uses).
    pub door_not_latched_disagree: Vec<bool>,
    pub slide_low_pressure: Vec<bool>,
}

pub struct CrewCallSystem {
    prev_attendant: [bool; Zone::COUNT],
    prev_purser: bool,
    prev_emergency: bool,
    prev_cockpit: bool,
    prev_oven_smoke: [bool; Zone::COUNT],
    prev_ife_smoke: [bool; Zone::COUNT],
    prev_waste_full: [bool; Zone::COUNT],
    prev_water_fault: bool,
    prev_door_disagree: Vec<bool>,
    prev_slide_low: Vec<bool>,
    /// Unacknowledged calls, most recent last; `active_call` reports the
    /// highest-priority one.
    active: Vec<CabinEvent>,
}

impl CrewCallSystem {
    pub fn new() -> Self {
        Self {
            prev_attendant: [false; Zone::COUNT],
            prev_purser: false,
            prev_emergency: false,
            prev_cockpit: false,
            prev_oven_smoke: [false; Zone::COUNT],
            prev_ife_smoke: [false; Zone::COUNT],
            prev_waste_full: [false; Zone::COUNT],
            prev_water_fault: false,
            prev_door_disagree: Vec::new(),
            prev_slide_low: Vec::new(),
            active: Vec::new(),
        }
    }

    fn rising(prev: &mut bool, now: bool) -> bool {
        let edge = now && !*prev;
        *prev = now;
        edge
    }

    /// The single highest-priority unacknowledged call, matching a real
    /// interphone's one "MASTER CALL" annunciator lit from whichever call is
    /// most urgent.
    pub fn active_call(&self) -> Option<&CabinEvent> {
        self.active.iter().max_by_key(|e| e.priority())
    }

    /// Clears one specific call once the crew has answered it. No effect if
    /// the underlying condition (e.g. smoke) is still latched active by the
    /// next `step` — a real acknowledgement silences the chime, not the
    /// underlying fault.
    pub fn acknowledge(&mut self, event: &CabinEvent) {
        self.active.retain(|e| e != event);
    }

    pub fn acknowledge_all(&mut self) {
        self.active.clear();
    }

    pub fn step(&mut self, inputs: &CrewCallInputs, snapshot: &CabinSnapshot, _dt: f64) -> Vec<CabinEvent> {
        let mut new_events = Vec::new();

        for i in 0..Zone::COUNT {
            if Self::rising(&mut self.prev_attendant[i], inputs.attendant_call_pressed[i]) {
                new_events.push(CabinEvent::AttendantCall(Zone::ALL[i]));
            }
        }
        if Self::rising(&mut self.prev_purser, inputs.purser_call_pressed) {
            new_events.push(CabinEvent::PurserCall);
        }
        if Self::rising(&mut self.prev_emergency, inputs.emergency_call_pressed) {
            new_events.push(CabinEvent::EmergencyCall);
        }
        if Self::rising(&mut self.prev_cockpit, inputs.cockpit_call_pressed) {
            new_events.push(CabinEvent::CockpitCall);
        }

        for i in 0..Zone::COUNT {
            if Self::rising(&mut self.prev_oven_smoke[i], snapshot.oven_smoke[i]) {
                new_events.push(CabinEvent::Smoke(SmokeSource::Galley(Zone::ALL[i])));
            }
            if Self::rising(&mut self.prev_ife_smoke[i], snapshot.ife_zone_smoke[i]) {
                new_events.push(CabinEvent::Smoke(SmokeSource::Ife(Zone::ALL[i])));
            }
            if Self::rising(&mut self.prev_waste_full[i], snapshot.waste_tank_full[i]) {
                new_events.push(CabinEvent::WasteTankFull(Zone::ALL[i]));
            }
        }
        if Self::rising(&mut self.prev_water_fault, snapshot.water_system_fault) {
            new_events.push(CabinEvent::WaterSystemFault);
        }

        if self.prev_door_disagree.len() != snapshot.door_not_latched_disagree.len() {
            self.prev_door_disagree = vec![false; snapshot.door_not_latched_disagree.len()];
        }
        for i in 0..snapshot.door_not_latched_disagree.len() {
            if Self::rising(&mut self.prev_door_disagree[i], snapshot.door_not_latched_disagree[i]) {
                new_events.push(CabinEvent::DoorNotLatchedDisagree { door_index: i });
            }
        }
        if self.prev_slide_low.len() != snapshot.slide_low_pressure.len() {
            self.prev_slide_low = vec![false; snapshot.slide_low_pressure.len()];
        }
        for i in 0..snapshot.slide_low_pressure.len() {
            if Self::rising(&mut self.prev_slide_low[i], snapshot.slide_low_pressure[i]) {
                new_events.push(CabinEvent::SlideLowPressure { door_index: i });
            }
        }

        self.active.extend(new_events.iter().cloned());
        new_events
    }
}

impl Default for CrewCallSystem {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_held_button_raises_exactly_one_event_not_one_per_tick() {
        let mut c = CrewCallSystem::new();
        let mut inputs = CrewCallInputs::default();
        inputs.purser_call_pressed = true;
        let e1 = c.step(&inputs, &CabinSnapshot::default(), 1.0);
        let e2 = c.step(&inputs, &CabinSnapshot::default(), 1.0);
        assert_eq!(e1, vec![CabinEvent::PurserCall]);
        assert!(e2.is_empty(), "holding the button should not repeat the event");
    }

    #[test]
    fn releasing_and_pressing_again_raises_a_new_event() {
        let mut c = CrewCallSystem::new();
        let mut inputs = CrewCallInputs::default();
        inputs.attendant_call_pressed[Zone::Mid.index()] = true;
        c.step(&inputs, &CabinSnapshot::default(), 1.0);
        inputs.attendant_call_pressed[Zone::Mid.index()] = false;
        c.step(&inputs, &CabinSnapshot::default(), 1.0);
        inputs.attendant_call_pressed[Zone::Mid.index()] = true;
        let e = c.step(&inputs, &CabinSnapshot::default(), 1.0);
        assert_eq!(e, vec![CabinEvent::AttendantCall(Zone::Mid)]);
    }

    #[test]
    fn emergency_outranks_purser_which_outranks_a_normal_attendant_call() {
        let mut c = CrewCallSystem::new();
        let mut inputs = CrewCallInputs::default();
        inputs.attendant_call_pressed[Zone::Fwd.index()] = true;
        c.step(&inputs, &CabinSnapshot::default(), 1.0);
        assert_eq!(c.active_call(), Some(&CabinEvent::AttendantCall(Zone::Fwd)));

        inputs.purser_call_pressed = true;
        c.step(&inputs, &CabinSnapshot::default(), 1.0);
        assert_eq!(c.active_call(), Some(&CabinEvent::PurserCall));

        inputs.emergency_call_pressed = true;
        c.step(&inputs, &CabinSnapshot::default(), 1.0);
        assert_eq!(c.active_call(), Some(&CabinEvent::EmergencyCall));
    }

    #[test]
    fn a_galley_smoke_condition_raises_an_emergency_priority_call() {
        let mut c = CrewCallSystem::new();
        let mut snapshot = CabinSnapshot::default();
        snapshot.oven_smoke[Zone::Aft.index()] = true;
        let e = c.step(&CrewCallInputs::default(), &snapshot, 1.0);
        assert_eq!(e, vec![CabinEvent::Smoke(SmokeSource::Galley(Zone::Aft))]);
        assert_eq!(c.active_call().unwrap().priority(), CallPriority::Emergency);
    }

    #[test]
    fn acknowledging_clears_the_active_call_but_a_persisting_condition_can_re_raise_it() {
        let mut c = CrewCallSystem::new();
        let mut snapshot = CabinSnapshot::default();
        snapshot.waste_tank_full[Zone::Fwd.index()] = true;
        c.step(&CrewCallInputs::default(), &snapshot, 1.0);
        let event = CabinEvent::WasteTankFull(Zone::Fwd);
        assert_eq!(c.active_call(), Some(&event));
        c.acknowledge(&event);
        assert_eq!(c.active_call(), None);

        // The condition clearing and re-triggering later raises a fresh event.
        snapshot.waste_tank_full[Zone::Fwd.index()] = false;
        c.step(&CrewCallInputs::default(), &snapshot, 1.0);
        snapshot.waste_tank_full[Zone::Fwd.index()] = true;
        let e = c.step(&CrewCallInputs::default(), &snapshot, 1.0);
        assert_eq!(e, vec![event]);
    }

    #[test]
    fn door_and_slide_event_vectors_track_a_variable_number_of_doors() {
        let mut c = CrewCallSystem::new();
        let mut snapshot = CabinSnapshot::default();
        snapshot.door_not_latched_disagree = vec![false, false, true];
        let e = c.step(&CrewCallInputs::default(), &snapshot, 1.0);
        assert_eq!(e, vec![CabinEvent::DoorNotLatchedDisagree { door_index: 2 }]);

        snapshot.slide_low_pressure = vec![true];
        let e = c.step(&CrewCallInputs::default(), &snapshot, 1.0);
        assert_eq!(e, vec![CabinEvent::SlideLowPressure { door_index: 0 }]);
    }

    #[test]
    fn acknowledge_all_clears_every_active_call() {
        let mut c = CrewCallSystem::new();
        let mut inputs = CrewCallInputs::default();
        inputs.purser_call_pressed = true;
        inputs.attendant_call_pressed[Zone::Fwd.index()] = true;
        c.step(&inputs, &CabinSnapshot::default(), 1.0);
        assert!(c.active_call().is_some());
        c.acknowledge_all();
        assert!(c.active_call().is_none());
    }

    #[test]
    fn no_nan_or_panic_at_rest_or_dt_zero() {
        let mut c = CrewCallSystem::new();
        let e = c.step(&CrewCallInputs::default(), &CabinSnapshot::default(), 0.0);
        assert!(e.is_empty());
        assert!(c.active_call().is_none());
    }
}
