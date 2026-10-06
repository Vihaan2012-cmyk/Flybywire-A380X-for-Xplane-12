use super::allocation::ComputerHealth;

const BPS_MIN_SUPPLY_FRACTION: f64 = 0.25;
const NOMINAL_SYSTEM_PA: f64 = 5000.0 * 6894.757;
const BPS_SPINUP_S: f64 = 0.5;
const BCM_ENGAGE_S: f64 = 0.3;

const BCM_TRIM_RATE_DEG_S: f64 = 0.3;

#[derive(Clone, Copy, Debug, Default)]
pub struct BackupPowerSupply {
    output: f64,
}

impl BackupPowerSupply {
    pub fn new() -> Self {
        Self { output: 0.0 }
    }

    pub fn step(&mut self, commanded: bool, supply_pressure_pa: f64, dt_s: f64) -> bool {
        let driven = supply_pressure_pa >= BPS_MIN_SUPPLY_FRACTION * NOMINAL_SYSTEM_PA;
        if !(commanded && driven) {
            self.output = 0.0;
        } else {
            let dt = dt_s.max(0.0);
            self.output = (self.output + dt / BPS_SPINUP_S.max(1e-9)).min(1.0);
        }
        self.available()
    }

    pub fn available(&self) -> bool {
        self.output >= 1.0
    }

    pub fn output_fraction(&self) -> f64 {
        self.output
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BackupInceptors {
    pub sidestick_pitch: f64,
    pub sidestick_roll: f64,
    pub rudder_pedal: f64,
    pub pitch_trim_switch: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BackupCommands {
    pub inner_aileron_deg: [f64; 2],
    pub outer_elevator_deg: [f64; 2],
    pub rudder_deg: f64,
    pub ths_rate_deg_s: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BackupControlModule {
    engaged: f64,
}

const AILERON_UP_DEG: f64 = 30.0;
const AILERON_DOWN_DEG: f64 = -20.0;
const ELEVATOR_UP_DEG: f64 = 30.0;
const ELEVATOR_DOWN_DEG: f64 = -20.0;
const RUDDER_DEG: f64 = 30.0;

impl BackupControlModule {
    pub fn new() -> Self {
        Self { engaged: 0.0 }
    }

    pub fn power_supplies_commanded(computers: &ComputerHealth) -> bool {
        !computers.prim[1] && !computers.sec[1]
    }

    pub fn all_computers_lost(computers: &ComputerHealth) -> bool {
        computers.prim.iter().chain(computers.sec.iter()).all(|healthy| !healthy)
    }

    pub fn step(
        &mut self,
        computers: &ComputerHealth,
        bps_available: [bool; 2],
        inceptors: &BackupInceptors,
        dt_s: f64,
    ) -> Option<BackupCommands> {
        let powered = bps_available.iter().any(|a| *a);
        let wanted = powered && Self::all_computers_lost(computers);
        let dt = dt_s.max(0.0);
        if wanted {
            self.engaged = (self.engaged + dt / BCM_ENGAGE_S.max(1e-9)).min(1.0);
        } else {
            self.engaged = 0.0;
            return None;
        }
        if self.engaged < 1.0 {
            return None;
        }

        let clamp = |n: f64| n.clamp(-1.0, 1.0);
        let scale = |n: f64, up: f64, down: f64| if n >= 0.0 { n * up } else { -n * down };

        let roll = clamp(inceptors.sidestick_roll);
        let pitch = clamp(inceptors.sidestick_pitch);
        let yaw = clamp(inceptors.rudder_pedal);

        let right_up = scale(roll, AILERON_UP_DEG, AILERON_DOWN_DEG);
        Some(BackupCommands {
            inner_aileron_deg: [-right_up, right_up],
            outer_elevator_deg: [scale(pitch, ELEVATOR_UP_DEG, ELEVATOR_DOWN_DEG); 2],
            rudder_deg: yaw * RUDDER_DEG,
            ths_rate_deg_s: clamp(inceptors.pitch_trim_switch) * BCM_TRIM_RATE_DEG_S,
        })
    }

    pub fn engaged(&self) -> bool {
        self.engaged >= 1.0
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BackupControl {
    pub supplies: [BackupPowerSupply; 2],
    pub module: BackupControlModule,
}

impl BackupControl {
    pub fn new() -> Self {
        Self { supplies: [BackupPowerSupply::new(); 2], module: BackupControlModule::new() }
    }

    pub fn step(
        &mut self,
        computers: &ComputerHealth,
        system_pressure_pa: [f64; 2],
        inceptors: &BackupInceptors,
        dt_s: f64,
    ) -> Option<BackupCommands> {
        let commanded = BackupControlModule::power_supplies_commanded(computers);
        let available = [
            self.supplies[0].step(commanded, system_pressure_pa[0], dt_s),
            self.supplies[1].step(commanded, system_pressure_pa[1], dt_s),
        ];
        self.module.step(computers, available, inceptors, dt_s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FULL: f64 = NOMINAL_SYSTEM_PA;
    const DT: f64 = 0.02;

    fn healthy() -> ComputerHealth {
        ComputerHealth::all_healthy()
    }

    fn all_dead() -> ComputerHealth {
        ComputerHealth { prim: [false; 3], sec: [false; 3] }
    }

    fn settle(b: &mut BackupControl, c: &ComputerHealth, p: [f64; 2], i: &BackupInceptors) -> Option<BackupCommands> {
        let mut out = None;
        for _ in 0..200 {
            out = b.step(c, p, i, DT);
        }
        out
    }

    #[test]
    fn the_supplies_start_only_once_prim_2_and_sec_2_are_both_gone() {
        let mut c = healthy();
        assert!(!BackupControlModule::power_supplies_commanded(&c));
        c.prim[1] = false;
        assert!(!BackupControlModule::power_supplies_commanded(&c), "PRIM 2 alone is not the condition");
        c.sec[1] = false;
        assert!(BackupControlModule::power_supplies_commanded(&c));
        let other = ComputerHealth { prim: [false, true, false], sec: [false, true, false] };
        assert!(!BackupControlModule::power_supplies_commanded(&other));
    }

    #[test]
    fn nothing_engages_while_any_computer_still_flies() {
        let mut b = BackupControl::new();
        assert_eq!(settle(&mut b, &healthy(), [FULL; 2], &BackupInceptors::default()), None);
        let one_sec = ComputerHealth { prim: [false; 3], sec: [false, false, true] };
        let mut b = BackupControl::new();
        assert_eq!(settle(&mut b, &one_sec, [FULL; 2], &BackupInceptors::default()), None);
        assert!(!b.module.engaged());
    }

    #[test]
    fn with_every_computer_lost_the_bcm_takes_over_and_moves_its_four_surfaces() {
        let mut b = BackupControl::new();
        let stick = BackupInceptors { sidestick_pitch: 1.0, sidestick_roll: 1.0, rudder_pedal: 1.0, pitch_trim_switch: 1.0 };
        let out = settle(&mut b, &all_dead(), [FULL; 2], &stick).expect("the BCM must fly the aircraft");
        assert!(b.module.engaged());
        assert!(out.inner_aileron_deg[1] > 0.0 && out.inner_aileron_deg[0] < 0.0);
        assert!((out.inner_aileron_deg[0] + out.inner_aileron_deg[1]).abs() < 1e-9);
        assert!((out.outer_elevator_deg[0] - ELEVATOR_UP_DEG).abs() < 1e-9);
        assert_eq!(out.outer_elevator_deg[0], out.outer_elevator_deg[1]);
        assert!((out.rudder_deg - RUDDER_DEG).abs() < 1e-9);
        assert!(out.ths_rate_deg_s > 0.0);
    }

    #[test]
    fn either_supply_alone_powers_the_module() {
        for (green, yellow) in [(FULL, 0.0), (0.0, FULL)] {
            let mut b = BackupControl::new();
            let out = settle(&mut b, &all_dead(), [green, yellow], &BackupInceptors::default());
            assert!(out.is_some(), "one hydraulic system should still power the BCM ({green}, {yellow})");
        }
        let mut b = BackupControl::new();
        assert_eq!(settle(&mut b, &all_dead(), [0.0, 0.0], &BackupInceptors::default()), None);
    }

    #[test]
    fn a_supply_drops_at_once_and_returns_on_its_spin_up_time() {
        let mut bps = BackupPowerSupply::new();
        for _ in 0..200 {
            bps.step(true, FULL, DT);
        }
        assert!(bps.available());
        assert!(!bps.step(true, 0.0, DT), "losing the hydraulics stops the generator at once");
        assert_eq!(bps.output_fraction(), 0.0);
        assert!(!bps.step(true, FULL, DT), "must not be back within one tick");
        for _ in 0..200 {
            bps.step(true, FULL, DT);
        }
        assert!(bps.available());
    }

    #[test]
    fn a_recovered_computer_drops_the_bcm_immediately() {
        let mut b = BackupControl::new();
        assert!(settle(&mut b, &all_dead(), [FULL; 2], &BackupInceptors::default()).is_some());
        let mut recovered = all_dead();
        recovered.prim[0] = true;
        assert_eq!(b.step(&recovered, [FULL; 2], &BackupInceptors::default(), DT), None);
        assert!(!b.module.engaged());
    }

    #[test]
    fn an_over_range_inceptor_cannot_exceed_the_surfaces_travel() {
        let mut b = BackupControl::new();
        let over = BackupInceptors { sidestick_pitch: -9.0, sidestick_roll: 9.0, rudder_pedal: -9.0, pitch_trim_switch: 9.0 };
        let out = settle(&mut b, &all_dead(), [FULL; 2], &over).unwrap();
        assert!((out.outer_elevator_deg[0] - ELEVATOR_DOWN_DEG).abs() < 1e-9);
        assert!((out.rudder_deg + RUDDER_DEG).abs() < 1e-9);
        assert!(out.inner_aileron_deg[1] <= AILERON_UP_DEG + 1e-9);
        assert!((out.ths_rate_deg_s - BCM_TRIM_RATE_DEG_S).abs() < 1e-9);
    }
}
