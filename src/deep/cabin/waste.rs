//! ATA 38: waste. Vacuum toilets (using the cabin/outside-atmosphere
//! differential pressure at altitude, with an electric vacuum generator for
//! the ground/low-altitude case where that differential is too small), zoned
//! waste tanks with level sensors, "tank full" making a zone's lavatories
//! inoperative, sensor faults, flush-valve faults, and the bowl rinse this
//! draws from the potable system (`water.rs`; not imported — the volume is
//! returned to whatever wires the two together, per the deep systems
//! brief's self-containment rule).
//!
//! No FlyByWire source exists to port (see `deep/cabin/mod.rs`'s doc), so
//! this is a native addition.
//!
//! **Sourcing:**
//! - Vacuum toilets are widely documented (aviation-engineering and toilet-
//!   technology references) as using the pressure differential between the
//!   pressurised cabin and the low ambient pressure outside at altitude to
//!   pull waste into a central tank through a small-bore line, using far
//!   less water than a gravity system; because that differential barely
//!   exists on the ground or at low altitude, an electric vacuum generator
//!   (a blower/venturi) provides equivalent suction until enough altitude
//!   differential exists, when it shuts down automatically. This module
//!   models exactly that switchover; no A380-specific AMM differential
//!   threshold, tank size or generator rating is public, so those are
//!   `GENERIC`.
//! - The natural-vacuum threshold (3.0 psi cabin/ambient differential) is
//!   `GENERIC`: chosen so the switchover happens early in the climb (a
//!   modest differential, well under the roughly 9-11 psi typical
//!   large-transport cruise maximum), matching how these systems are
//!   commonly described as generator-assisted "low down, natural once
//!   climbing".
//! - Vacuum toilets' low water use per flush (0.3 L here) is `GENERIC`,
//!   consistent with the commonly cited "under half a litre" figure for
//!   aircraft vacuum toilets versus several litres for a gravity/flush
//!   toilet.
//! - Tank capacity (200 L per zone) and the "full" safety margin (95% of
//!   capacity, leaving headroom before physical overflow) are `GENERIC`
//!   engineering-practice figures; no public A380 AMM tank size exists.

use super::Zone;

/// Pascals per psi (exact).
const PSI_TO_PA: f64 = 6894.757;

/// GENERIC natural-vacuum threshold: at or above this cabin/ambient
/// differential, suction is sufficient without the generator (module doc).
pub const NATURAL_VACUUM_THRESHOLD_PA: f64 = 3.0 * PSI_TO_PA;
/// GENERIC vacuum generator electrical rating (a small blower motor).
pub const GENERATOR_RATED_W: f64 = 400.0;
/// GENERIC waste tank capacity, litres, per zone.
pub const TANK_CAPACITY_L: f64 = 200.0;
/// GENERIC "full" safety margin (module doc).
const FULL_FRACTION: f64 = 0.95;
/// GENERIC rinse/waste volume added per full-effectiveness flush, litres.
pub const FLUSH_VOLUME_L: f64 = 0.3;
/// A valve stuck fully open keeps drawing a small continuous trickle
/// instead of one discrete flush per command: modelled as this fraction of
/// a full flush's volume per second while stuck. GENERIC.
const STUCK_OPEN_LEAK_L_S: f64 = FLUSH_VOLUME_L / 10.0;

/// Faults this system carries, each 0.0 (healthy) .. 1.0 (fully failed).
#[derive(Clone, Copy, Debug, Default)]
pub struct WasteFaults {
    /// The electric vacuum generator (shared system).
    pub generator_fault: f64,
    /// Each zone's tank level sensor: above 0.5 it freezes its last good
    /// reading (a stuck float/capacitance probe).
    pub tank_level_sensor_fault: [f64; Zone::COUNT],
    /// Each zone's flush valve stuck open: continuous small leak/suction
    /// loss instead of discrete flushes.
    pub valve_stuck_open: [f64; Zone::COUNT],
    /// Each zone's flush valve stuck closed: a flush command has
    /// proportionally less effect.
    pub valve_stuck_closed: [f64; Zone::COUNT],
}

#[derive(Clone, Copy, Debug)]
pub struct WasteInputs {
    /// Cabin pressure minus outside ambient pressure, Pa (the same figure
    /// `doors.rs` reads as `PRESS_MAN_CABIN_DELTA_PRESSURE`, converted to
    /// Pa at the integration boundary).
    pub cabin_diff_pressure_pa: f64,
    /// A flush command per zone this tick (edge-triggered by the caller;
    /// held true for more than one tick just re-triggers, matching a real
    /// toilet's own momentary switch debounce being the caller's concern).
    pub flush_commanded: [bool; Zone::COUNT],
}

impl Default for WasteInputs {
    fn default() -> Self {
        Self { cabin_diff_pressure_pa: 0.0, flush_commanded: [false; Zone::COUNT] }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct WasteOutputs {
    pub level_percent: [f64; Zone::COUNT],
    pub tank_full: [bool; Zone::COUNT],
    /// Backlog item 2: lavatories in a zone are inoperative once its tank
    /// reads full.
    pub lav_inoperative: [bool; Zone::COUNT],
    /// Whether this tick's flush command (if any) had effective suction.
    pub flush_effective: [bool; Zone::COUNT],
    /// Water drawn from the potable system's rinse this tick, per zone,
    /// litres — the hook `water.rs`'s consumer wires up.
    pub rinse_used_l: [f64; Zone::COUNT],
    pub generator_running: bool,
    pub generator_power_w: f64,
}

struct TankZone {
    level_l: f64,
    displayed_percent: f64,
}

pub struct WasteSystem {
    zones: [TankZone; Zone::COUNT],
}

impl WasteSystem {
    pub fn new() -> Self {
        Self { zones: std::array::from_fn(|_| TankZone { level_l: 0.0, displayed_percent: 0.0 }) }
    }

    /// Ground service: empties every tank.
    pub fn service(&mut self) {
        for z in &mut self.zones {
            z.level_l = 0.0;
        }
    }

    pub fn step(&mut self, inputs: &WasteInputs, faults: &WasteFaults, dt: f64) -> WasteOutputs {
        let dt = dt.max(0.0);

        // The natural differential alone gives a smooth 0..1 suction
        // fraction approaching the threshold (an orifice/venturi's suction
        // grows with the driving pressure, not a hard step), and the
        // generator (when running and at least partly healthy) makes up
        // the rest.
        let natural_fraction = (inputs.cabin_diff_pressure_pa / NATURAL_VACUUM_THRESHOLD_PA).clamp(0.0, 1.0);
        let generator_needed = natural_fraction < 1.0;
        let generator_running = generator_needed && faults.generator_fault < 1.0;
        let generator_health = if generator_running { 1.0 - faults.generator_fault } else { 0.0 };
        let suction_fraction = (natural_fraction + (1.0 - natural_fraction) * generator_health).clamp(0.0, 1.0);
        let generator_power_w = if generator_running { GENERATOR_RATED_W * generator_health } else { 0.0 };

        let mut level_percent = [0.0; Zone::COUNT];
        let mut tank_full = [false; Zone::COUNT];
        let mut lav_inoperative = [false; Zone::COUNT];
        let mut flush_effective = [false; Zone::COUNT];
        let mut rinse_used_l = [0.0; Zone::COUNT];

        for (i, zone_state) in self.zones.iter_mut().enumerate() {
            let full_now = zone_state.level_l >= TANK_CAPACITY_L * FULL_FRACTION;

            let stuck_open = faults.valve_stuck_open[i] > 0.5;
            let stuck_closed_frac = faults.valve_stuck_closed[i].clamp(0.0, 1.0);

            let mut added_l = 0.0;
            if stuck_open {
                added_l += STUCK_OPEN_LEAK_L_S * dt;
                flush_effective[i] = false;
            } else if inputs.flush_commanded[i] && !full_now {
                let effectiveness = suction_fraction * (1.0 - stuck_closed_frac);
                added_l += FLUSH_VOLUME_L * effectiveness;
                flush_effective[i] = effectiveness > 0.5;
            }
            rinse_used_l[i] = added_l;

            if !full_now {
                zone_state.level_l = (zone_state.level_l + added_l).min(TANK_CAPACITY_L);
            }

            let real_percent = 100.0 * zone_state.level_l / TANK_CAPACITY_L;
            if faults.tank_level_sensor_fault[i] < 0.5 {
                zone_state.displayed_percent = real_percent;
            }
            level_percent[i] = zone_state.displayed_percent;
            tank_full[i] = zone_state.displayed_percent >= 100.0 * FULL_FRACTION;
            lav_inoperative[i] = tank_full[i];
        }

        WasteOutputs { level_percent, tank_full, lav_inoperative, flush_effective, rinse_used_l, generator_running, generator_power_w }
    }
}

impl Default for WasteSystem {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_flush_at_cruise_needs_no_generator_and_adds_the_full_volume() {
        let mut w = WasteSystem::new();
        let mut inputs = WasteInputs { cabin_diff_pressure_pa: 8.0 * PSI_TO_PA, ..Default::default() };
        inputs.flush_commanded[Zone::Fwd.index()] = true;
        let out = w.step(&inputs, &WasteFaults::default(), 1.0);
        assert!(!out.generator_running, "ample natural differential, generator should not be needed");
        assert!((w.zones[Zone::Fwd.index()].level_l - FLUSH_VOLUME_L).abs() < 1e-9);
        assert!(out.flush_effective[Zone::Fwd.index()]);
    }

    #[test]
    fn on_the_ground_the_generator_is_needed_and_a_failed_one_weakens_the_flush() {
        let mut healthy = WasteSystem::new();
        let mut failed = WasteSystem::new();
        let mut inputs = WasteInputs { cabin_diff_pressure_pa: 0.0, ..Default::default() };
        inputs.flush_commanded[Zone::Mid.index()] = true;

        let out_healthy = healthy.step(&inputs, &WasteFaults::default(), 1.0);
        let out_failed = failed.step(&inputs, &WasteFaults { generator_fault: 1.0, ..Default::default() }, 1.0);

        assert!(out_healthy.generator_running);
        assert!(!out_failed.generator_running, "a fully failed generator cannot run");
        assert!(healthy.zones[Zone::Mid.index()].level_l > failed.zones[Zone::Mid.index()].level_l, "no suction, weaker flush");
        assert_eq!(failed.zones[Zone::Mid.index()].level_l, 0.0);
    }

    #[test]
    fn tank_full_makes_the_zone_inoperative_and_stops_accepting_more_waste() {
        let mut w = WasteSystem::new();
        w.zones[Zone::Aft.index()].level_l = TANK_CAPACITY_L * FULL_FRACTION;
        let mut inputs = WasteInputs { cabin_diff_pressure_pa: 8.0 * PSI_TO_PA, ..Default::default() };
        inputs.flush_commanded[Zone::Aft.index()] = true;
        let level_before = w.zones[Zone::Aft.index()].level_l;
        let out = w.step(&inputs, &WasteFaults::default(), 1.0);
        assert!(out.tank_full[Zone::Aft.index()]);
        assert!(out.lav_inoperative[Zone::Aft.index()]);
        assert_eq!(w.zones[Zone::Aft.index()].level_l, level_before, "a full tank must not accept more waste");
        // Other zones are unaffected.
        assert!(!out.lav_inoperative[Zone::Fwd.index()]);
    }

    #[test]
    fn a_valve_stuck_open_leaks_continuously_without_any_flush_command() {
        let mut w = WasteSystem::new();
        let inputs = WasteInputs { cabin_diff_pressure_pa: 8.0 * PSI_TO_PA, ..Default::default() };
        let mut faults = WasteFaults::default();
        faults.valve_stuck_open[Zone::Fwd.index()] = 1.0;
        for _ in 0..10 {
            w.step(&inputs, &faults, 1.0);
        }
        assert!(w.zones[Zone::Fwd.index()].level_l > 0.0, "a stuck-open valve should fill the tank with no flush command");
    }

    #[test]
    fn a_valve_stuck_closed_makes_a_commanded_flush_ineffective() {
        let mut w = WasteSystem::new();
        let mut inputs = WasteInputs { cabin_diff_pressure_pa: 8.0 * PSI_TO_PA, ..Default::default() };
        inputs.flush_commanded[Zone::Mid.index()] = true;
        let mut faults = WasteFaults::default();
        faults.valve_stuck_closed[Zone::Mid.index()] = 1.0;
        let out = w.step(&inputs, &faults, 1.0);
        assert_eq!(w.zones[Zone::Mid.index()].level_l, 0.0);
        assert!(!out.flush_effective[Zone::Mid.index()]);
    }

    #[test]
    fn a_stuck_level_sensor_freezes_its_reading_while_the_real_level_keeps_rising() {
        let mut w = WasteSystem::new();
        let mut inputs = WasteInputs { cabin_diff_pressure_pa: 8.0 * PSI_TO_PA, ..Default::default() };
        let mut faults = WasteFaults::default();
        faults.tank_level_sensor_fault[Zone::Fwd.index()] = 1.0;
        inputs.flush_commanded[Zone::Fwd.index()] = true;
        let out1 = w.step(&inputs, &faults, 1.0);
        let stuck_reading = out1.level_percent[Zone::Fwd.index()];
        for _ in 0..20 {
            w.step(&inputs, &faults, 1.0);
        }
        let out2 = w.step(&inputs, &faults, 1.0);
        assert_eq!(out2.level_percent[Zone::Fwd.index()], stuck_reading);
        assert!(w.zones[Zone::Fwd.index()].level_l / TANK_CAPACITY_L * 100.0 > stuck_reading);
    }

    #[test]
    fn service_empties_every_tank() {
        let mut w = WasteSystem::new();
        for z in &mut w.zones {
            z.level_l = 50.0;
        }
        w.service();
        assert!(w.zones.iter().all(|z| z.level_l == 0.0));
    }

    #[test]
    fn no_nan_at_rest_or_dt_zero() {
        let mut w = WasteSystem::new();
        let out = w.step(&WasteInputs::default(), &WasteFaults::default(), 0.0);
        assert!(out.level_percent.iter().all(|p| !p.is_nan()));
        assert!(!out.generator_power_w.is_nan());
    }
}
