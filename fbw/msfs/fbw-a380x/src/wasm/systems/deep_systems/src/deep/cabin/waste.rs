use super::Zone;

const PSI_TO_PA: f64 = 6894.757;

pub const NATURAL_VACUUM_THRESHOLD_PA: f64 = 3.0 * PSI_TO_PA;
pub const GENERATOR_RATED_W: f64 = 400.0;
pub const TANK_CAPACITY_L: f64 = 200.0;
const FULL_FRACTION: f64 = 0.95;
pub const FLUSH_VOLUME_L: f64 = 0.3;
const STUCK_OPEN_LEAK_L_S: f64 = FLUSH_VOLUME_L / 10.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct WasteFaults {
    pub generator_fault: f64,
    pub tank_level_sensor_fault: [f64; Zone::COUNT],
    pub valve_stuck_open: [f64; Zone::COUNT],
    pub valve_stuck_closed: [f64; Zone::COUNT],
}

#[derive(Clone, Copy, Debug)]
pub struct WasteInputs {
    pub cabin_diff_pressure_pa: f64,
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
    pub lav_inoperative: [bool; Zone::COUNT],
    pub flush_effective: [bool; Zone::COUNT],
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

    pub fn service(&mut self) {
        for z in &mut self.zones {
            z.level_l = 0.0;
        }
    }

    pub fn step(&mut self, inputs: &WasteInputs, faults: &WasteFaults, dt: f64) -> WasteOutputs {
        let dt = dt.max(0.0);

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
