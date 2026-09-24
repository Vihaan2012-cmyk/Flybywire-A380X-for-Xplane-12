//! Backup control: the two Backup Power Supplies (BPS) and the Backup
//! Control Module (BCM) that fly the aircraft when every flight control
//! computer is gone.
//!
//! **Neither this port nor FlyByWire modelled this.** FlyByWire's A380
//! sources contain no BCM or BPS at all, so with all six computers failed
//! their aircraft simply has no control path left. The real A380 does: this
//! is the layer below direct law, and it is the reason losing PRIMs 1-3 and
//! SECs 1-3 is survivable rather than terminal.
//!
//! Everything about the architecture below is from the FCOM (DSC-27-10-40
//! "Backup Control", p.1899), which is the only source this project has for
//! it:
//!
//! * there are **two BPS**, each an electrical generator driven by one
//!   hydraulic system, and they **start to operate when PRIM 2 and SEC 2 are
//!   failed or no longer supplied** -- note that this is deliberately
//!   *earlier* than the BCM itself runs, so the supply is already spun up
//!   and settled before it is needed;
//! * there is **one BCM**, **powered by either BPS**, which **starts to
//!   operate when all flight control computers are lost**;
//! * it controls the **inner ailerons**, the **upper and lower rudders**,
//!   the **outer elevators** and the **trimmable horizontal stabiliser** --
//!   one surface per axis per side, the minimum set that still gives three
//!   axes plus trim;
//! * the crew fly it through the **sidesticks**, the **rudder pedals** and
//!   the **pitch trim switch**.
//!
//! What the FCOM does *not* give, and is therefore GENERIC here, is marked
//! at each constant: the BPS's spin-up time and the hydraulic pressure it
//! needs, and the stick-to-surface gains. In particular the FCOM says only
//! that backup control "provides control and stability of the aircraft" and
//! states no gains, no damping terms and no law structure. This module
//! therefore implements a **direct proportional** path -- inceptor position
//! straight to surface deflection, scaled to the surfaces' own travel -- and
//! does not pretend to reproduce Airbus's actual backup law, which is not
//! published. Anything more would be invention dressed as fidelity.

use super::allocation::ComputerHealth;

/// GENERIC: hydraulic pressure below which a BPS cannot turn its generator
/// fast enough to produce useful power. A fraction of the 5000 psi nominal
/// system pressure rather than an absolute figure, so it tracks the system
/// this port already models; the FCOM states only that each BPS is
/// "supplied by one hydraulic system", not the pressure it needs.
const BPS_MIN_SUPPLY_FRACTION: f64 = 0.25;
/// The A380's nominal hydraulic system pressure, Pa (5000 psi, FCOM
/// DSC-29-10). Used only to turn [`BPS_MIN_SUPPLY_FRACTION`] into a
/// pressure.
const NOMINAL_SYSTEM_PA: f64 = 5000.0 * 6894.757;
/// GENERIC: how long a BPS takes to come up to usable output once its
/// running condition is met. A hydraulically-driven generator spinning up
/// and its regulator settling is a fraction of a second; no A380 figure is
/// published.
const BPS_SPINUP_S: f64 = 0.5;
/// GENERIC: how long the BCM takes to take over once powered and commanded.
/// Kept short deliberately -- this is the last control path there is, and a
/// long transfer would be a worse invention than a short one.
const BCM_ENGAGE_S: f64 = 0.3;

/// GENERIC: pitch trim rate the BCM drives the THS at from the pitch trim
/// switch, deg/s. The FCOM names the switch as the crew's trim input in
/// backup control but gives no rate; this is the same order as the normal
/// manual trim rate this port already uses elsewhere.
const BCM_TRIM_RATE_DEG_S: f64 = 0.3;

/// One Backup Power Supply: a generator driven by one hydraulic system.
#[derive(Clone, Copy, Debug, Default)]
pub struct BackupPowerSupply {
    /// 0 at rest, 1 fully spun up and regulating.
    output: f64,
}

impl BackupPowerSupply {
    pub fn new() -> Self {
        Self { output: 0.0 }
    }

    /// Advance one tick. `commanded` is the FCOM's running condition (PRIM 2
    /// and SEC 2 failed or unsupplied); `supply_pressure_pa` is this BPS's
    /// own hydraulic system.
    ///
    /// The supply falls away immediately when the hydraulics do -- a
    /// generator with no drive stops making power at once -- but comes up on
    /// a spin-up constant, which is the asymmetry a real machine has.
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

/// The crew's inceptors, as the BCM sees them. Each is a normalised
/// position in `-1..=1` (the convention the rest of this directory uses for
/// a commanded fraction of travel), plus the pitch trim switch as a
/// three-position up/off/down.
#[derive(Clone, Copy, Debug, Default)]
pub struct BackupInceptors {
    /// Positive nose up.
    pub sidestick_pitch: f64,
    /// Positive right wing down.
    pub sidestick_roll: f64,
    /// Positive nose right.
    pub rudder_pedal: f64,
    /// -1 nose down, 0 off, +1 nose up.
    pub pitch_trim_switch: f64,
}

/// What the BCM commands while it is flying the aircraft. Angles are
/// degrees, in the same sign conventions `live::SurfaceAngles` uses.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BackupCommands {
    /// `[left, right]`, the **inner** aileron of each wing, positive TE up.
    pub inner_aileron_deg: [f64; 2],
    /// `[left, right]`, the **outer** elevator of each side, positive TE up.
    pub outer_elevator_deg: [f64; 2],
    /// Both the upper and the lower rudder, which the BCM drives together.
    pub rudder_deg: f64,
    /// Commanded THS rate this tick, deg/s, positive nose up.
    pub ths_rate_deg_s: f64,
}

/// The Backup Control Module.
#[derive(Clone, Copy, Debug, Default)]
pub struct BackupControlModule {
    engaged: f64,
}

/// Travel the BCM commands over, degrees. These are the surfaces' own
/// limits from `live.rs`, restated here so this module stays self-contained
/// (the directory's own rule); a mismatch would only ever clip.
const AILERON_UP_DEG: f64 = 30.0;
const AILERON_DOWN_DEG: f64 = -20.0;
const ELEVATOR_UP_DEG: f64 = 30.0;
const ELEVATOR_DOWN_DEG: f64 = -20.0;
const RUDDER_DEG: f64 = 30.0;

impl BackupControlModule {
    pub fn new() -> Self {
        Self { engaged: 0.0 }
    }

    /// Whether the BPS running condition of the FCOM is met: **PRIM 2 and
    /// SEC 2 failed or no longer supplied**. Indexed as `allocation`'s
    /// `ComputerHealth` is, so index 1 is computer number 2.
    pub fn power_supplies_commanded(computers: &ComputerHealth) -> bool {
        !computers.prim[1] && !computers.sec[1]
    }

    /// Whether the BCM's own condition is met: **all** flight control
    /// computers lost.
    pub fn all_computers_lost(computers: &ComputerHealth) -> bool {
        computers.prim.iter().chain(computers.sec.iter()).all(|healthy| !healthy)
    }

    /// Advance one tick and, once engaged, return what the BCM is
    /// commanding. `None` while any computer still flies the aircraft, or
    /// while neither BPS is up -- a BCM with no power commands nothing, and
    /// the surfaces are left to whatever the rest of the system does with
    /// them (in practice, damping).
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
            // Losing power or regaining a computer drops the BCM at once:
            // it is a last-resort path, not something that hands over
            // gradually.
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

        // Roll right: left inner aileron trailing edge down, right up.
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

/// The whole backup control path: two supplies and the module they feed.
#[derive(Clone, Copy, Debug, Default)]
pub struct BackupControl {
    /// `[green-driven, yellow-driven]`.
    pub supplies: [BackupPowerSupply; 2],
    pub module: BackupControlModule,
}

impl BackupControl {
    pub fn new() -> Self {
        Self { supplies: [BackupPowerSupply::new(); 2], module: BackupControlModule::new() }
    }

    /// One tick of the whole path. `system_pressure_pa` is `[green,
    /// yellow]`, each BPS being driven by one of them.
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

    /// The FCOM's BPS condition is PRIM 2 **and** SEC 2, not any computer.
    #[test]
    fn the_supplies_start_only_once_prim_2_and_sec_2_are_both_gone() {
        let mut c = healthy();
        assert!(!BackupControlModule::power_supplies_commanded(&c));
        c.prim[1] = false;
        assert!(!BackupControlModule::power_supplies_commanded(&c), "PRIM 2 alone is not the condition");
        c.sec[1] = false;
        assert!(BackupControlModule::power_supplies_commanded(&c));
        // Another pair does not do it.
        let other = ComputerHealth { prim: [false, true, false], sec: [false, true, false] };
        assert!(!BackupControlModule::power_supplies_commanded(&other));
    }

    /// A healthy aircraft must never see the backup path engage.
    #[test]
    fn nothing_engages_while_any_computer_still_flies() {
        let mut b = BackupControl::new();
        assert_eq!(settle(&mut b, &healthy(), [FULL; 2], &BackupInceptors::default()), None);
        // One surviving SEC is still direct law, not backup control.
        let one_sec = ComputerHealth { prim: [false; 3], sec: [false, false, true] };
        let mut b = BackupControl::new();
        assert_eq!(settle(&mut b, &one_sec, [FULL; 2], &BackupInceptors::default()), None);
        assert!(!b.module.engaged());
    }

    /// All six computers lost, hydraulics up: the BCM takes over and flies.
    #[test]
    fn with_every_computer_lost_the_bcm_takes_over_and_moves_its_four_surfaces() {
        let mut b = BackupControl::new();
        let stick = BackupInceptors { sidestick_pitch: 1.0, sidestick_roll: 1.0, rudder_pedal: 1.0, pitch_trim_switch: 1.0 };
        let out = settle(&mut b, &all_dead(), [FULL; 2], &stick).expect("the BCM must fly the aircraft");
        assert!(b.module.engaged());
        // Roll right: right inner aileron up, left down, and they oppose.
        assert!(out.inner_aileron_deg[1] > 0.0 && out.inner_aileron_deg[0] < 0.0);
        assert!((out.inner_aileron_deg[0] + out.inner_aileron_deg[1]).abs() < 1e-9);
        // Full nose-up pitch on both outer elevators, together.
        assert!((out.outer_elevator_deg[0] - ELEVATOR_UP_DEG).abs() < 1e-9);
        assert_eq!(out.outer_elevator_deg[0], out.outer_elevator_deg[1]);
        assert!((out.rudder_deg - RUDDER_DEG).abs() < 1e-9);
        assert!(out.ths_rate_deg_s > 0.0);
    }

    /// Either supply alone is enough -- the FCOM says the BCM is powered by
    /// either BPS.
    #[test]
    fn either_supply_alone_powers_the_module() {
        for (green, yellow) in [(FULL, 0.0), (0.0, FULL)] {
            let mut b = BackupControl::new();
            let out = settle(&mut b, &all_dead(), [green, yellow], &BackupInceptors::default());
            assert!(out.is_some(), "one hydraulic system should still power the BCM ({green}, {yellow})");
        }
        // Both hydraulic systems down: no supply, so no backup control.
        let mut b = BackupControl::new();
        assert_eq!(settle(&mut b, &all_dead(), [0.0, 0.0], &BackupInceptors::default()), None);
    }

    /// A supply stops making power the instant its drive goes, but needs its
    /// spin-up time to come back.
    #[test]
    fn a_supply_drops_at_once_and_returns_on_its_spin_up_time() {
        let mut bps = BackupPowerSupply::new();
        for _ in 0..200 {
            bps.step(true, FULL, DT);
        }
        assert!(bps.available());
        assert!(!bps.step(true, 0.0, DT), "losing the hydraulics stops the generator at once");
        assert_eq!(bps.output_fraction(), 0.0);
        // Back up, but not instantly.
        assert!(!bps.step(true, FULL, DT), "must not be back within one tick");
        for _ in 0..200 {
            bps.step(true, FULL, DT);
        }
        assert!(bps.available());
    }

    /// Regaining a computer hands control straight back.
    #[test]
    fn a_recovered_computer_drops_the_bcm_immediately() {
        let mut b = BackupControl::new();
        assert!(settle(&mut b, &all_dead(), [FULL; 2], &BackupInceptors::default()).is_some());
        let mut recovered = all_dead();
        recovered.prim[0] = true;
        assert_eq!(b.step(&recovered, [FULL; 2], &BackupInceptors::default(), DT), None);
        assert!(!b.module.engaged());
    }

    /// Inceptors are clamped: an over-range command cannot drive a surface
    /// past its travel.
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
