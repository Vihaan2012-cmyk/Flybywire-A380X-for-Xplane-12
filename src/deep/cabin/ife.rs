//! ATA 44/25: in-flight entertainment and cabin power. Per-seat-zone seat
//! electronics boxes as electrical loads, redundant IFE head-end servers,
//! commercial/in-seat power shedding, and two fault families: a seat
//! box/port short or overheat (a chosen seat/port label is generated only
//! to flavour the crew-call message — the fault itself is modelled per
//! zone, exactly as the backlog specifies) and IFE server failure.
//!
//! No FlyByWire source exists to port (see `deep/cabin/mod.rs`'s doc): the
//! only IFE/seat-power-adjacent code anywhere in FlyByWire's own trees is
//! the galley electrical shed *flag* (`a380_systems/src/electrical/
//! galley.rs`), no seat/IFE model at all. Native addition.
//!
//! **Sourcing:**
//! - Seatback IFE plus USB/AC in-seat power is commonly documented (avionics
//!   and cabin-systems trade press, e.g. aircraft interiors publications) as
//!   drawing on the order of 15-30 W per economy seat position and 60-150 W
//!   per premium seat position with its own AC outlet; `GENERIC` figures of
//!   25 W (economy) and 100 W (premium) are used here, at the low/typical
//!   end of those ranges, not any specific airline's or OEM's rating.
//! - IFE head-end server racks are commonly reported (same trade press) as
//!   drawing on the order of hundreds of watts to a few kilowatts per unit
//!   depending on generation; 1500 W is `GENERIC`, a round mid-range figure.
//! - The three-zone seat counts (56/260/190, module doc below) are a
//!   `GENERIC` illustrative A380 three-class layout (roughly matching the
//!   type's commonly cited 500-ish three-class capacity), not any specific
//!   airline's real seat map — the per-seat/port labels this module
//!   generates for fault messages are flavour text over that illustrative
//!   map, never a claim about a real configuration.
//! - The short/overheat thermal model (a zone's wiring dissipating extra
//!   heat under fault, an accumulator rising to an "overheat" then "smoke"
//!   threshold before its own protection trips the zone) is the same
//!   general I^2R self-heating physics `physics::electrical`'s breaker
//!   model uses; the thresholds and heat/loss coefficients here are
//!   `GENERIC` (no public A380 seat-power SSPC data), sized only for the
//!   right qualitative order: minutes, not seconds, from a developing fault
//!   to smoke.

use super::Zone;

/// GENERIC per-seat power draw, W (module doc): economy-class seatback
/// IFE + USB/AC.
const SEAT_POWER_W_ECONOMY: f64 = 25.0;
/// GENERIC per-seat power draw, W: premium (business/first) seat with its
/// own AC power outlet and larger display.
const SEAT_POWER_W_PREMIUM: f64 = 100.0;

/// GENERIC illustrative three-class seat counts per zone (module doc): the
/// forward zone is premium, mid/aft are economy.
const SEAT_COUNT: [u32; Zone::COUNT] = [56, 260, 190];
const IS_PREMIUM: [bool; Zone::COUNT] = [true, false, false];

fn seat_power_w(zone: Zone) -> f64 {
    let per_seat = if IS_PREMIUM[zone.index()] { SEAT_POWER_W_PREMIUM } else { SEAT_POWER_W_ECONOMY };
    SEAT_COUNT[zone.index()] as f64 * per_seat
}

/// GENERIC IFE head-end server rating, W (module doc).
const SERVER_RATED_W: f64 = 1500.0;
/// Two redundant head-end servers, either able to serve content to the
/// whole cabin (a typical redundant IFE architecture). GENERIC count.
pub const N_SERVERS: usize = 2;

/// Extra heat a fully faulted zone's short dissipates into its wiring
/// bundle, W (module doc: the same I^2R self-heating idea
/// `physics::electrical`'s breaker curve uses, not that curve itself).
///
/// Bounded by the branch that feeds the seat boxes rather than picked: a
/// cabin seat-power branch is 115 V AC single phase behind a 10 A SSPC, and
/// a fault drawing more than that opens its protection at once, so the
/// worst *sustained* heating a developing (high-resistance / arc-tracking)
/// fault can put into the bundle is 115 V * 10 A = 1150 W. That is the
/// fault = 1.0 end of the scale; anything larger is the instant-trip case,
/// not the slow-cook one this thermal model is for.
const FAULT_HEAT_W: f64 = 1150.0;
/// GENERIC zone wiring bundle: thermal capacity (J/K) and loss to the
/// surrounding structure (W/K), sized so a full-fault heats from cabin
/// ambient to the smoke threshold over a few minutes, not instantly. Hand
/// check with the figures below: the bundle's equilibrium rise under a full
/// fault is 1150/6 = 191.7 K (to 215.7 C, so it does reach both thresholds
/// rather than levelling off below them), with a time constant of
/// 6000/6 = 1000 s, giving `-tau*ln(1 - dT/191.7)` = 274 s to the 70 C
/// overheat call and 595 s to 110 C and smoke.
const ZONE_CAPACITY_J_K: f64 = 6000.0;
const ZONE_LOSS_W_K: f64 = 6.0;
const ZONE_AMBIENT_C: f64 = 24.0;
/// GENERIC thresholds: a developing fault is reported "overheating" once
/// the bundle passes this, and "smoke" (with the zone's own protection
/// tripping it dead) at the higher one.
const OVERHEAT_TEMP_C: f64 = 70.0;
const SMOKE_TEMP_C: f64 = 110.0;

/// Row range and seat-letter set used only to flavour a fault message
/// (module doc: illustrative, not a real seat map).
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

/// A deterministic (not random-quality, just varied) sequence for picking a
/// flavour label each time a new fault activates, so repeated faults do not
/// always name the same seat. Not `rand`/any crate: a plain linear
/// congruential step (Numerical Recipes' constants), std only.
fn next_seed(seed: &mut u64) -> u64 {
    *seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
    *seed
}

#[derive(Clone, Copy, Debug, Default)]
pub struct IfeFaults {
    /// Per zone, the fraction (0..1) of that zone's seat wiring presently
    /// shorted/faulted, driving the thermal model above.
    pub seat_fault: [f64; Zone::COUNT],
    /// Per server, 0..1: at 1.0 the server has failed outright.
    pub server_fault: [f64; N_SERVERS],
}

#[derive(Clone, Copy, Debug)]
pub struct IfeInputs {
    /// The commercial/non-essential bus feeding seat power and the IFE
    /// servers is powered (false when shed: single-generator ops, external
    /// power limits, or the overhead COMMERCIAL pushbutton off, matching
    /// the shedding conditions FlyByWire's own `MainGalley`/`SecondaryGalley`
    /// check for the galley bus).
    pub commercial_power_available: bool,
    /// The cabin crew's own seat-power/IFE master switch.
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

    /// Resets a zone's tripped protection (e.g. maintenance reset on the
    /// ground) — has no effect while the fault causing it is still active.
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
                // The zone's own protection interrupts a smoking short,
                // exactly as `physics::electrical`'s breaker trips a
                // circuit whose I^2t accumulator reaches its threshold —
                // modelled locally here to stay self-contained.
                state.tripped = true;
            }
            state.was_smoking = smoking;

            let commanded_w = if bus_live && !state.tripped { seat_power_w(zone) } else { 0.0 };
            out.zone_power_w[i] = commanded_w;
            out.zone_temp_c[i] = state.temp_c;
            out.zone_tripped[i] = state.tripped;
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
        // From the heat balance in FAULT_HEAT_W/ZONE_CAPACITY_J_K's docs:
        // -1000 * ln(1 - (110 - 24)/191.7) = 595 s to reach the smoke
        // threshold. Minutes, as the module doc requires, not seconds.
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
