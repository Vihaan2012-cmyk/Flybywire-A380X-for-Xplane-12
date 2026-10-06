use super::Zone;

const SEAT_POWER_W_ECONOMY: f64 = 25.0;
const SEAT_POWER_W_PREMIUM: f64 = 100.0;

const SEAT_COUNT: [u32; Zone::COUNT] = [56, 260, 190];
const IS_PREMIUM: [bool; Zone::COUNT] = [true, false, false];

fn seat_power_w(zone: Zone) -> f64 {
    let per_seat = if IS_PREMIUM[zone.index()] { SEAT_POWER_W_PREMIUM } else { SEAT_POWER_W_ECONOMY };
    SEAT_COUNT[zone.index()] as f64 * per_seat
}

const SERVER_RATED_W: f64 = 1500.0;
pub const N_SERVERS: usize = 2;

const FAULT_HEAT_W: f64 = 1150.0;
const ZONE_CAPACITY_J_K: f64 = 6000.0;
const ZONE_LOSS_W_K: f64 = 6.0;
const ZONE_AMBIENT_C: f64 = 24.0;
pub const OVERHEAT_TEMP_C: f64 = 70.0;
pub const SMOKE_TEMP_C: f64 = 110.0;

fn seat_label(zone: Zone, seed: u64) -> String {
    let (row_lo, row_hi, letters): (u32, u32, &[char]) = match zone {
        Zone::Fwd => (1, 14, &['A', 'D', 'G', 'K']),
        Zone::Mid => (15, 40, &['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'J', 'K']),
        Zone::Aft => (41, 59, &['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'J', 'K']),
    };
    let span = (row_hi - row_lo + 1) as u64;
    let row = row_lo as u64 + (seed % span);
    let letter = letters[((seed / span) as usize) % letters.len()];
    let ports = ["USB-C", "USB-A", "AC outlet", "handset"];
    let port = ports[((seed / (span * letters.len() as u64)) as usize) % ports.len()];
    format!("seat {row}{letter} {port}")
}

fn next_seed(seed: &mut u64) -> u64 {
    *seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
    *seed
}

#[derive(Clone, Copy, Debug, Default)]
pub struct IfeFaults {
    pub seat_fault: [f64; Zone::COUNT],
    pub server_fault: [f64; N_SERVERS],
    pub smoke_detector_fault: [f64; Zone::COUNT],
}

#[derive(Clone, Copy, Debug)]
pub struct IfeInputs {
    pub commercial_power_available: bool,
    pub seat_power_on: bool,
}

impl Default for IfeInputs {
    fn default() -> Self {
        Self { commercial_power_available: true, seat_power_on: true }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SeatFaultKind {
    Short,
    Overheating,
    Smoke,
}

#[derive(Clone, Debug, PartialEq)]
pub enum IfeEvent {
    SeatFault { zone: Zone, label: String, kind: SeatFaultKind },
    ServerFailure { server: usize },
    ServerRestored { server: usize },
}

#[derive(Clone, Copy, Debug, Default)]
pub struct IfeOutputs {
    pub zone_power_w: [f64; Zone::COUNT],
    pub zone_temp_c: [f64; Zone::COUNT],
    pub zone_tripped: [bool; Zone::COUNT],
    pub zone_smoke_detector_fault: [bool; Zone::COUNT],
    pub server_power_w: [f64; N_SERVERS],
    pub server_failed: [bool; N_SERVERS],
    pub content_available: bool,
    pub total_power_w: f64,
}

#[derive(Clone, Debug)]
struct ZoneState {
    temp_c: f64,
    tripped: bool,
    was_faulted: bool,
    was_overheating: bool,
    was_smoking: bool,
}

pub struct IfeSystem {
    zones: [ZoneState; Zone::COUNT],
    zone_labels: [Option<String>; Zone::COUNT],
    server_was_failed: [bool; N_SERVERS],
    seed: u64,
}

impl IfeSystem {
    pub fn new() -> Self {
        Self {
            zones: std::array::from_fn(|_| ZoneState { temp_c: ZONE_AMBIENT_C, tripped: false, was_faulted: false, was_overheating: false, was_smoking: false }),
            zone_labels: Default::default(),
            server_was_failed: [false; N_SERVERS],
            seed: 1,
        }
    }

    pub fn reset_zone(&mut self, zone: Zone, faults: &IfeFaults) {
        if faults.seat_fault[zone.index()] <= 0.0 {
            self.zones[zone.index()].tripped = false;
            self.zones[zone.index()].temp_c = ZONE_AMBIENT_C;
        }
    }

    pub fn step(&mut self, inputs: &IfeInputs, faults: &IfeFaults, dt: f64) -> (IfeOutputs, Vec<IfeEvent>) {
        let dt = dt.max(0.0);
        let mut events = Vec::new();
        let mut out = IfeOutputs::default();

        let bus_live = inputs.commercial_power_available && inputs.seat_power_on;

        for i in 0..Zone::COUNT {
            let zone = Zone::ALL[i];
            let fault = faults.seat_fault[i].clamp(0.0, 1.0);
            let state = &mut self.zones[i];

            if fault > 0.0 && !state.was_faulted {
                let label = seat_label(zone, next_seed(&mut self.seed));
                events.push(IfeEvent::SeatFault { zone, label: label.clone(), kind: SeatFaultKind::Short });
                self.zone_labels[i] = Some(label);
            }
            state.was_faulted = fault > 0.0;

            let fault_heat_w = fault * FAULT_HEAT_W;
            let target_c = ZONE_AMBIENT_C + fault_heat_w / ZONE_LOSS_W_K;
            let tau_s = (ZONE_CAPACITY_J_K / ZONE_LOSS_W_K).max(1e-6);
            state.temp_c += (target_c - state.temp_c) * (1.0 - (-dt / tau_s).exp());

            let overheating = state.temp_c >= OVERHEAT_TEMP_C;
            if overheating && !state.was_overheating {
                let label = self.zone_labels[i].clone().unwrap_or_else(|| seat_label(zone, next_seed(&mut self.seed)));
                events.push(IfeEvent::SeatFault { zone, label, kind: SeatFaultKind::Overheating });
            }
            state.was_overheating = overheating;

            let smoking = state.temp_c >= SMOKE_TEMP_C;
            if smoking && !state.was_smoking {
                let label = self.zone_labels[i].clone().unwrap_or_else(|| seat_label(zone, next_seed(&mut self.seed)));
                events.push(IfeEvent::SeatFault { zone, label, kind: SeatFaultKind::Smoke });
                state.tripped = true;
            }
            state.was_smoking = smoking;

            let commanded_w = if bus_live && !state.tripped { seat_power_w(zone) } else { 0.0 };
            out.zone_power_w[i] = commanded_w;
            out.zone_temp_c[i] = state.temp_c;
            out.zone_tripped[i] = state.tripped;
            out.zone_smoke_detector_fault[i] = faults.smoke_detector_fault[i] > 0.0;
        }

        let mut any_server_healthy = false;
        for i in 0..N_SERVERS {
            let health = 1.0 - faults.server_fault[i].clamp(0.0, 1.0);
            let failed = health <= 0.05;
            if failed && !self.server_was_failed[i] {
                events.push(IfeEvent::ServerFailure { server: i });
            } else if !failed && self.server_was_failed[i] {
                events.push(IfeEvent::ServerRestored { server: i });
            }
            self.server_was_failed[i] = failed;
            out.server_failed[i] = failed;
            out.server_power_w[i] = if bus_live && !failed { SERVER_RATED_W } else { 0.0 };
            any_server_healthy |= bus_live && !failed;
        }

        out.content_available = any_server_healthy;
        out.total_power_w = out.zone_power_w.iter().sum::<f64>() + out.server_power_w.iter().sum::<f64>();
        (out, events)
    }
}

impl Default for IfeSystem {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_healthy_zone_draws_its_full_rated_power_when_the_bus_is_live() {
        let mut s = IfeSystem::new();
        let (out, events) = s.step(&IfeInputs::default(), &IfeFaults::default(), 1.0);
        assert!(events.is_empty());
        assert_eq!(out.zone_power_w[Zone::Fwd.index()], seat_power_w(Zone::Fwd));
        assert!(out.content_available);
    }

    #[test]
    fn shedding_the_commercial_bus_zeroes_seat_and_server_power() {
        let mut s = IfeSystem::new();
        let inputs = IfeInputs { commercial_power_available: false, seat_power_on: true };
        let (out, _) = s.step(&inputs, &IfeFaults::default(), 1.0);
        assert!(out.zone_power_w.iter().all(|&p| p == 0.0));
        assert!(out.server_power_w.iter().all(|&p| p == 0.0));
        assert!(!out.content_available);
    }

    #[test]
    fn a_developing_short_raises_temperature_and_eventually_smokes_and_trips() {
        let mut s = IfeSystem::new();
        let mut faults = IfeFaults::default();
        faults.seat_fault[Zone::Mid.index()] = 1.0;
        let mut events = Vec::new();
        let mut smoke_at_s: Option<f64> = None;
        for step in 0..36000 {
            let (_, e) = s.step(&IfeInputs::default(), &faults, 1.0);
            if smoke_at_s.is_none() && e.iter().any(|e| matches!(e, IfeEvent::SeatFault { kind: SeatFaultKind::Smoke, .. })) {
                smoke_at_s = Some((step + 1) as f64);
            }
            events.extend(e);
        }
        assert!((smoke_at_s.unwrap_or(f64::NAN) - 595.0).abs() < 5.0, "{smoke_at_s:?}");
        assert!(matches!(events.iter().find(|e| matches!(e, IfeEvent::SeatFault { kind: SeatFaultKind::Short, .. })), Some(_)));
        assert!(matches!(events.iter().find(|e| matches!(e, IfeEvent::SeatFault { kind: SeatFaultKind::Overheating, .. })), Some(_)));
        assert!(matches!(events.iter().find(|e| matches!(e, IfeEvent::SeatFault { kind: SeatFaultKind::Smoke, .. })), Some(_)));
        assert!(s.zones[Zone::Mid.index()].tripped, "a smoking short should trip its own zone");
    }

    #[test]
    fn a_tripped_zone_loses_power_even_though_the_bus_is_still_live() {
        let mut s = IfeSystem::new();
        s.zones[Zone::Aft.index()].tripped = true;
        let (out, _) = s.step(&IfeInputs::default(), &IfeFaults::default(), 1.0);
        assert_eq!(out.zone_power_w[Zone::Aft.index()], 0.0);
        assert!(out.zone_power_w[Zone::Fwd.index()] > 0.0, "other zones are unaffected");
    }

    #[test]
    fn a_smoke_detector_circuit_fault_is_independent_of_the_seat_wiring_thermal_path() {
        let mut s = IfeSystem::new();
        let healthy = s.step(&IfeInputs::default(), &IfeFaults::default(), 1.0).0;
        assert!(healthy.zone_smoke_detector_fault.iter().all(|&f| !f));

        let mut s2 = IfeSystem::new();
        let mut faults = IfeFaults::default();
        faults.smoke_detector_fault[Zone::Mid.index()] = 1.0;
        let (out, _) = s2.step(&IfeInputs::default(), &faults, 1.0);
        assert!(out.zone_smoke_detector_fault[Zone::Mid.index()]);
        assert!(!out.zone_smoke_detector_fault[Zone::Fwd.index()], "only the faulted zone's own detector is affected");
        assert!((out.zone_temp_c[Zone::Mid.index()] - ZONE_AMBIENT_C).abs() < 1e-6);
        assert!(!out.zone_tripped[Zone::Mid.index()]);
    }

    #[test]
    fn fault_events_carry_a_seat_label_that_varies_between_occurrences() {
        let mut s = IfeSystem::new();
        let mut faults = IfeFaults::default();
        faults.seat_fault[Zone::Fwd.index()] = 1.0;
        let (_, e1) = s.step(&IfeInputs::default(), &faults, 1.0);
        let IfeEvent::SeatFault { label: label1, .. } = &e1[0] else { panic!("expected a seat fault event") };
        assert!(label1.starts_with("seat "));

        faults.seat_fault[Zone::Fwd.index()] = 0.0;
        s.step(&IfeInputs::default(), &faults, 1.0);
        faults.seat_fault[Zone::Fwd.index()] = 1.0;
        let (_, e2) = s.step(&IfeInputs::default(), &faults, 1.0);
        let IfeEvent::SeatFault { label: label2, .. } = &e2[0] else { panic!("expected a seat fault event") };
        assert_ne!(label1, label2, "successive faults should not always pick the same flavour seat");
    }

    #[test]
    fn a_failed_server_still_leaves_content_available_via_the_redundant_one() {
        let mut s = IfeSystem::new();
        let mut faults = IfeFaults::default();
        faults.server_fault[0] = 1.0;
        let (out, events) = s.step(&IfeInputs::default(), &faults, 1.0);
        assert!(out.server_failed[0]);
        assert!(!out.server_failed[1]);
        assert!(out.content_available, "the healthy redundant server should still serve content");
        assert!(events.iter().any(|e| matches!(e, IfeEvent::ServerFailure { server: 0 })));
    }

    #[test]
    fn both_servers_failing_loses_content_entirely() {
        let mut s = IfeSystem::new();
        let faults = IfeFaults { server_fault: [1.0; N_SERVERS], ..Default::default() };
        let (out, _) = s.step(&IfeInputs::default(), &faults, 1.0);
        assert!(!out.content_available);
    }

    #[test]
    fn reset_zone_only_clears_a_trip_once_the_fault_is_gone() {
        let mut s = IfeSystem::new();
        let mut faults = IfeFaults::default();
        faults.seat_fault[Zone::Fwd.index()] = 1.0;
        s.zones[Zone::Fwd.index()].tripped = true;
        s.reset_zone(Zone::Fwd, &faults);
        assert!(s.zones[Zone::Fwd.index()].tripped, "fault still active: reset should not clear it");
        faults.seat_fault[Zone::Fwd.index()] = 0.0;
        s.reset_zone(Zone::Fwd, &faults);
        assert!(!s.zones[Zone::Fwd.index()].tripped);
    }

    #[test]
    fn no_nan_at_rest_or_dt_zero() {
        let mut s = IfeSystem::new();
        let (out, _) = s.step(&IfeInputs::default(), &IfeFaults::default(), 0.0);
        assert!(out.zone_temp_c.iter().all(|t| !t.is_nan()));
        assert!(!out.total_power_w.is_nan());
    }
}
