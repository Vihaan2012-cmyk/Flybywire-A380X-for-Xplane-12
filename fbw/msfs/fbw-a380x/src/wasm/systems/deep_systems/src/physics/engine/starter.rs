use super::params::N3_DESIGN_RPM;

pub const PEAK_POWER_W: f64 = 600_000.0;

pub const CUTOFF_N3_FRAC: f64 = 0.50;

fn cutoff_omega_rad_s() -> f64 {
    (CUTOFF_N3_FRAC * N3_DESIGN_RPM) * std::f64::consts::PI / 30.0
}

fn stall_torque_n_m() -> f64 {
    4.0 * PEAK_POWER_W / cutoff_omega_rad_s()
}

pub fn torque_n_m(n3_rpm: f64, supply_fraction: f64) -> f64 {
    let omega = n3_rpm.max(0.0) * std::f64::consts::PI / 30.0;
    let cutoff = cutoff_omega_rad_s();
    if omega >= cutoff {
        return 0.0;
    }
    stall_torque_n_m() * supply_fraction.clamp(0.0, 1.0) * (1.0 - omega / cutoff)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stall_torque_is_highest_at_zero_speed() {
        let stall = torque_n_m(0.0, 1.0);
        let partway = torque_n_m(N3_DESIGN_RPM * CUTOFF_N3_FRAC * 0.5, 1.0);
        assert!(stall > partway);
        assert!(partway > 0.0);
    }

    #[test]
    fn torque_reaches_zero_at_the_cutoff_speed() {
        let at_cutoff = torque_n_m(N3_DESIGN_RPM * CUTOFF_N3_FRAC, 1.0);
        assert!(at_cutoff.abs() < 1e-6);
        let past_cutoff = torque_n_m(N3_DESIGN_RPM, 1.0);
        assert!(past_cutoff.abs() < 1e-6);
    }

    #[test]
    fn a_weak_supply_gives_proportionally_less_torque() {
        let full = torque_n_m(0.0, 1.0);
        let half = torque_n_m(0.0, 0.5);
        assert!((half - full / 2.0).abs() < 1e-6);
    }
}
