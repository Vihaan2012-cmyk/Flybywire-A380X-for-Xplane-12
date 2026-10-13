//! Air turbine starter (ATS): the pneumatic turbine that bolts to the
//! accessory gearbox and, through a sprag (one-way) clutch, drives the HP
//! spool up during a start. A real air turbine, like any turbine, has a
//! single torque-speed line from its stalled (zero-speed) torque down to
//! true zero torque at its own no-load free-running speed -- the "starter
//! cutoff" a real start sequence uses (matching
//! `physics::engine::starter`'s own `CUTOFF_N3_FRAC`) is a *control*
//! decision to shut the air valve there, not a kink in the turbine's own
//! physics, so this module uses one straight line all the way to a higher
//! free-running speed and lets the air valve's commanded closure be what
//! normally stops the turbine well short of it.
//!
//! The sprag clutch is what makes a starter safe to leave coupled at all:
//! once the HP spool (driven by combustion) overtakes the turbine's own
//! speed, the sprag freewheels and the turbine coasts down under its own
//! friction rather than being dragged along. A clutch that fails to
//! *disengage* defeats that: the spool is then forced to keep turning the
//! starter's rotor at (a fraction of) its own speed for as long as the
//! engine runs, and if the HP spool later climbs toward its own overspeed
//! protection limit (`physics::engine::params::MAX_N3_PROTECTION_PCT`,
//! 116.5%, a public FADEC setpoint restated here for this self-contained
//! module) with the clutch still coupled, the starter rotor -- built for a
//! ~50-75%-N3-equivalent duty, never for continuous full-speed running --
//! is forced well past its structural limit and disintegrates. That is the
//! causal chain this module models, not a scripted "clutch fault ->
//! disintegration" shortcut: disintegration only happens if the clutch
//! fault *and* a high enough sustained spool speed both occur together.
//!
//! No Trent-900 ATS figures are public. `PEAK_POWER_W`/`CUTOFF_N3_FRAC` are
//! restated at the same order of magnitude and value `physics::engine::
//! starter` derives and cites (this module cannot import that sibling
//! model, per this directory's isolation rule, so the same public
//! derivation -- a large turbofan's compressor drag at light-off needing
//! several hundred kW with margin -- is repeated here rather than copied).
//! `FREE_SPEED_FRAC` and `DISINTEGRATION_FRAC` are **GENERIC**: a real ATS
//! free-spins somewhat above its designed cutoff (the reason an overspeed
//! cutout switch exists at all), and its rotor is proof-margined above that
//! free speed but far short of full N3, typical of accessory-gearbox-driven
//! rotating equipment not designed for core-speed duty.

use std::f64::consts::PI;

/// Trent 972B-84 HP spool design speed, RPM (restated from `physics::engine::
/// params::N3_DESIGN_RPM`, a public/derived figure this module cannot import).
pub const N3_DESIGN_RPM: f64 = 12200.0;
/// Peak shaft power (**derived, see module docs**, restating `physics::
/// engine::starter::PEAK_POWER_W`'s own public derivation).
pub const PEAK_POWER_W: f64 = 600_000.0;
/// Normal EEC-commanded handoff speed, fraction of `N3_DESIGN_RPM` (restates
/// `physics::engine::starter::CUTOFF_N3_FRAC`'s public derivation).
pub const CUTOFF_N3_FRAC: f64 = 0.50;
/// The turbine's own true no-load free-running speed, fraction of
/// `N3_DESIGN_RPM` (**GENERIC**: comfortably above the handoff point, the
/// margin an overspeed cutout switch exists to catch).
pub const FREE_SPEED_FRAC: f64 = 0.75;
/// Structural burst margin, fraction of `N3_DESIGN_RPM` (**GENERIC**).
pub const DISINTEGRATION_FRAC: f64 = 0.95;
/// Windage/friction drag coefficient for a clutch that fails to disengage,
/// N·m per (rad/s)^2 (**GENERIC**, sized so full coupling at 100% N3 drags
/// a few percent of the compressor's own design torque -- a real parasitic
/// accessory load, not a dominant one).
const DRAG_COEFF_NM_S2: f64 = 0.02;

fn omega_at_frac(frac: f64) -> f64 {
    (frac * N3_DESIGN_RPM).max(0.0) * PI / 30.0
}

fn stall_torque_n_m() -> f64 {
    4.0 * PEAK_POWER_W / omega_at_frac(FREE_SPEED_FRAC)
}

/// Faults the starter/clutch assembly can carry, 0 (healthy) .. 1 (fully
/// failed).
#[derive(Clone, Copy, Debug, Default)]
pub struct AtsFaults {
    /// Clutch fails to *engage*: reduces the torque actually transmitted to
    /// the spool during cranking (0 full transmission .. 1 none -- a hung
    /// start with a perfectly good turbine spinning uselessly).
    pub clutch_fails_to_engage: f64,
    /// Clutch fails to *disengage* once the spool overtakes the turbine:
    /// couples that fraction of the spool's own speed back into the
    /// starter rotor instead of letting it freewheel, and (combined with a
    /// high enough sustained spool speed) is the path to disintegration.
    pub clutch_fails_to_disengage: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct AtsState {
    /// Torque the turbine applies to the HP spool, N·m (positive = driving,
    /// negative = the drag a partially-stuck clutch imposes once the spool
    /// has overtaken the turbine).
    pub torque_n_m: f64,
    /// The starter rotor's own speed, RPM (equal to the HP spool's, N3
    /// frame, while the sprag is engaged or forced; its own free-spin speed
    /// once it has overtaken/decoupled).
    pub rotor_rpm: f64,
    pub disintegrated: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct AirTurbineStarter {
    disintegrated: bool,
}

impl AirTurbineStarter {
    pub fn new() -> Self {
        Self { disintegrated: false }
    }

    pub fn disintegrated(&self) -> bool {
        self.disintegrated
    }

    /// One step. `supply_fraction` is the air valve's open fraction times
    /// the supply pressure ratio available (0..~1); `n3_rpm` the HP spool's
    /// actual current speed.
    pub fn step(&mut self, supply_fraction: f64, n3_rpm: f64, faults: &AtsFaults) -> AtsState {
        if self.disintegrated {
            return AtsState { torque_n_m: 0.0, rotor_rpm: 0.0, disintegrated: true };
        }
        let n3_frac = (n3_rpm.max(0.0) / N3_DESIGN_RPM).max(0.0);
        let supply = supply_fraction.clamp(0.0, 1.5);
        let engage = 1.0 - faults.clutch_fails_to_engage.clamp(0.0, 1.0);
        let disengage_fault = faults.clutch_fails_to_disengage.clamp(0.0, 1.0);

        // Below the turbine's own free-running speed, the air (if the valve
        // is open) drives it along one straight torque-speed line.
        let omega = omega_at_frac(n3_frac);
        let free_omega = omega_at_frac(FREE_SPEED_FRAC);
        let driving_torque = if supply > 0.0 {
            (stall_torque_n_m() * supply * (1.0 - omega / free_omega).max(0.0) * engage).max(0.0)
        } else {
            0.0
        };

        // A sprag that fails to disengage forces the rotor toward the
        // spool's own speed once the spool has overtaken the turbine's free
        // speed; a healthy sprag (disengage_fault ~ 0) just freewheels and
        // applies no drag at all.
        let rotor_frac = if n3_frac > FREE_SPEED_FRAC {
            FREE_SPEED_FRAC + (n3_frac - FREE_SPEED_FRAC) * disengage_fault
        } else {
            n3_frac.max(FREE_SPEED_FRAC * (omega / free_omega).min(1.0))
        };
        let drag_torque = if n3_frac > FREE_SPEED_FRAC {
            -DRAG_COEFF_NM_S2 * disengage_fault * omega * omega
        } else {
            0.0
        };

        if rotor_frac >= DISINTEGRATION_FRAC {
            self.disintegrated = true;
            return AtsState { torque_n_m: 0.0, rotor_rpm: rotor_frac * N3_DESIGN_RPM, disintegrated: true };
        }

        AtsState { torque_n_m: driving_torque + drag_torque, rotor_rpm: rotor_frac * N3_DESIGN_RPM, disintegrated: false }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_supply_gives_no_torque_and_no_nan() {
        let mut ats = AirTurbineStarter::new();
        let s = ats.step(0.0, 0.0, &AtsFaults::default());
        assert_eq!(s.torque_n_m, 0.0);
        assert!(!s.torque_n_m.is_nan());
    }

    #[test]
    fn full_supply_at_rest_gives_the_highest_torque() {
        let mut ats = AirTurbineStarter::new();
        let stall = ats.step(1.0, 0.0, &AtsFaults::default());
        let mut ats2 = AirTurbineStarter::new();
        let partway = ats2.step(1.0, N3_DESIGN_RPM * CUTOFF_N3_FRAC, &AtsFaults::default());
        assert!(stall.torque_n_m > partway.torque_n_m);
        assert!(partway.torque_n_m > 0.0);
    }

    #[test]
    fn torque_reaches_zero_at_the_turbines_own_free_speed() {
        let mut ats = AirTurbineStarter::new();
        let s = ats.step(1.0, N3_DESIGN_RPM * FREE_SPEED_FRAC, &AtsFaults::default());
        assert!(s.torque_n_m.abs() < 1.0, "{}", s.torque_n_m);
    }

    #[test]
    fn a_clutch_that_wont_engage_gives_a_hung_start_with_less_torque() {
        let mut healthy = AirTurbineStarter::new();
        let mut hung = AirTurbineStarter::new();
        let h = healthy.step(1.0, 0.0, &AtsFaults::default());
        let g = hung.step(1.0, 0.0, &AtsFaults { clutch_fails_to_engage: 0.9, ..Default::default() });
        assert!(g.torque_n_m < 0.2 * h.torque_n_m);
    }

    #[test]
    fn a_healthy_sprag_freewheels_above_free_speed_with_no_drag() {
        let mut ats = AirTurbineStarter::new();
        let s = ats.step(0.0, N3_DESIGN_RPM * 0.9, &AtsFaults::default());
        assert_eq!(s.torque_n_m, 0.0);
        assert!(!s.disintegrated);
    }

    #[test]
    fn a_clutch_that_wont_disengage_at_high_spool_speed_disintegrates_the_starter() {
        let mut ats = AirTurbineStarter::new();
        let mut s = AtsState::default();
        for _ in 0..5 {
            // A spool speed at the FADEC's own overspeed protection setpoint
            // (116.5% N3, physics::engine::params::MAX_N3_PROTECTION_PCT)
            // with the clutch fully failed to disengage.
            s = ats.step(0.0, N3_DESIGN_RPM * 1.165, &AtsFaults { clutch_fails_to_disengage: 1.0, ..Default::default() });
        }
        assert!(s.disintegrated);
        assert!(ats.disintegrated());
    }

    #[test]
    fn once_disintegrated_the_starter_never_produces_torque_again() {
        let mut ats = AirTurbineStarter::new();
        ats.step(0.0, N3_DESIGN_RPM * 1.165, &AtsFaults { clutch_fails_to_disengage: 1.0, ..Default::default() });
        let s = ats.step(1.0, 0.0, &AtsFaults::default());
        assert_eq!(s.torque_n_m, 0.0);
        assert!(s.disintegrated);
    }

    #[test]
    fn a_partially_stuck_clutch_drags_the_spool_without_full_disintegration() {
        let mut ats = AirTurbineStarter::new();
        let s = ats.step(0.0, N3_DESIGN_RPM * 0.8, &AtsFaults { clutch_fails_to_disengage: 0.3, ..Default::default() });
        assert!(s.torque_n_m < 0.0, "a stuck clutch above free speed drags, it does not drive");
        assert!(!s.disintegrated);
    }
}
