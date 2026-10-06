use std::f64::consts::PI;

pub const N3_DESIGN_RPM: f64 = 12200.0;
pub const PEAK_POWER_W: f64 = 600_000.0;
pub const CUTOFF_N3_FRAC: f64 = 0.50;
pub const FREE_SPEED_FRAC: f64 = 0.75;
pub const DISINTEGRATION_FRAC: f64 = 0.95;
const DRAG_COEFF_NM_S2: f64 = 0.02;

fn omega_at_frac(frac: f64) -> f64 {
    (frac * N3_DESIGN_RPM).max(0.0) * PI / 30.0
}

fn stall_torque_n_m() -> f64 {
    4.0 * PEAK_POWER_W / omega_at_frac(FREE_SPEED_FRAC)
}

#[derive(Clone, Copy, Debug, Default)]
pub struct AtsFaults {
    pub clutch_fails_to_engage: f64,
    pub clutch_fails_to_disengage: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct AtsState {
    pub torque_n_m: f64,
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

    pub fn step(&mut self, supply_fraction: f64, n3_rpm: f64, faults: &AtsFaults) -> AtsState {
        if self.disintegrated {
            return AtsState { torque_n_m: 0.0, rotor_rpm: 0.0, disintegrated: true };
        }
        let n3_frac = (n3_rpm.max(0.0) / N3_DESIGN_RPM).max(0.0);
        let supply = supply_fraction.clamp(0.0, 1.5);
        let engage = 1.0 - faults.clutch_fails_to_engage.clamp(0.0, 1.0);
        let disengage_fault = faults.clutch_fails_to_disengage.clamp(0.0, 1.0);

        let omega = omega_at_frac(n3_frac);
        let free_omega = omega_at_frac(FREE_SPEED_FRAC);
        let driving_torque = if supply > 0.0 {
            (stall_torque_n_m() * supply * (1.0 - omega / free_omega).max(0.0) * engage).max(0.0)
        } else {
            0.0
        };

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
