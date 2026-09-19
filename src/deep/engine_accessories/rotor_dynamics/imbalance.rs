//! Rotor imbalance -> synchronous (1x shaft speed) vibration: a fan blade
//! loss, asymmetric ice accretion, or a bird-strike dent all do the same
//! physical thing -- they put an eccentric mass somewhere on the rotor --
//! and a spinning eccentric mass is a rotating force, `F = m * r * omega^2`,
//! that excites the rotor-support structure exactly like any forced
//! single-degree-of-freedom mass-spring-damper system: `x(omega) = F/k /
//! sqrt((1-(omega/omega_n)^2)^2 + (2*zeta*omega/omega_n)^2)`. Vibration
//! rising with the *square* of speed (not linearly) is why a damaged fan
//! blade shows a much bigger vibration jump at climb power than at idle --
//! that behaviour is not scripted here, it falls straight out of the
//! `omega^2` forcing term.
//!
//! A real engine vibration monitoring system extracts the 1x (synchronous)
//! component from a broadband accelerometer signal with a "tracking
//! filter" locked to shaft speed; since this model has no broadband noise
//! to filter out of, the tracking filter is instead modelled as what it
//! actually is physically -- a filter with a settling time -- by lagging
//! the instantaneous steady-state amplitude through a first-order response,
//! so a step change in imbalance or a fast spool-up shows the same
//! reporting lag a real tracking filter has, rather than an instantaneous
//! reading.
//!
//! No Trent-900 rotor-support stiffness/damping or fan blade mass is
//! public. `SpoolSpec`'s natural frequency, stiffness and reference
//! (single-blade-loss-equivalent) unbalance are **GENERIC**, chosen so a
//! full-magnitude fault gives a vibration reading that would plausibly
//! trip a real engine's high-vibration alert, without a published number to
//! calibrate the exact trip level against.

/// Per-spool rotor-support parameters.
#[derive(Clone, Copy, Debug)]
pub struct SpoolSpec {
    /// Support natural frequency, rad/s.
    pub omega_n_rad_s: f64,
    /// Effective support (bearing + casing) stiffness, N/m.
    pub stiffness_n_m: f64,
    /// Damping ratio (dimensionless).
    pub zeta: f64,
    /// Unbalance (mass x radius), kg*m, that `unbalance_frac == 1.0`
    /// represents for each fault channel below.
    pub reference_unbalance_kg_m: f64,
    /// Displacement, mm, that reads as vibration index 1.0 on the EICAS-
    /// style 0..10 scale this module reports.
    pub mm_per_index_unit: f64,
    /// Tracking filter settling time constant, s.
    pub filter_tau_s: f64,
}

/// Fan/LP spool: the largest rotor, softest support, most exposed to blade
/// loss/ice/bird strike (**GENERIC**, see module docs).
pub const FAN_SPEC: SpoolSpec = SpoolSpec { omega_n_rad_s: 45.0, stiffness_n_m: 6.0e7, zeta: 0.12, reference_unbalance_kg_m: 12.0, mm_per_index_unit: 0.08, filter_tau_s: 0.6 };
/// IP spool (**GENERIC**).
pub const IP_SPEC: SpoolSpec = SpoolSpec { omega_n_rad_s: 120.0, stiffness_n_m: 1.2e8, zeta: 0.1, reference_unbalance_kg_m: 3.0, mm_per_index_unit: 0.05, filter_tau_s: 0.5 };
/// HP spool: smallest, stiffest support (**GENERIC**).
pub const HP_SPEC: SpoolSpec = SpoolSpec { omega_n_rad_s: 220.0, stiffness_n_m: 2.5e8, zeta: 0.08, reference_unbalance_kg_m: 1.0, mm_per_index_unit: 0.03, filter_tau_s: 0.4 };

/// Faults contributing eccentric mass to one spool's rotor, 0 (none) .. 1
/// (a full single-blade-loss-equivalent unbalance). Summed directly rather
/// than vector-added by angular position (a **documented simplification**:
/// worst case is coincident phase, which this treats as the default rather
/// than modelling rotor phase angle).
#[derive(Clone, Copy, Debug, Default)]
pub struct ImbalanceFaults {
    pub blade_loss_frac: f64,
    pub ice_frac: f64,
    pub bird_strike_frac: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct SpoolVibration {
    spec: SpoolSpec,
    filtered_index: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct VibrationState {
    /// The tracking filter's current output, EICAS-style index (typically
    /// 0..10, healthy well under 2).
    pub index: f64,
    /// The instantaneous steady-state value the filter is chasing (before
    /// its settling lag), same units -- useful for tests/diagnostics.
    pub instantaneous_index: f64,
}

impl SpoolVibration {
    pub fn new(spec: SpoolSpec) -> Self {
        Self { spec, filtered_index: 0.0 }
    }

    /// One step. `omega_rad_s` is this spool's actual rotational speed.
    pub fn step(&mut self, omega_rad_s: f64, faults: &ImbalanceFaults, dt_s: f64) -> VibrationState {
        let omega = omega_rad_s.max(0.0);
        let unbalance_kg_m = self.spec.reference_unbalance_kg_m
            * (faults.blade_loss_frac.clamp(0.0, 1.0) + faults.ice_frac.clamp(0.0, 1.0) + faults.bird_strike_frac.clamp(0.0, 1.0));
        let force_n = unbalance_kg_m * omega * omega;
        let r = omega / self.spec.omega_n_rad_s;
        let denom = ((1.0 - r * r).powi(2) + (2.0 * self.spec.zeta * r).powi(2)).sqrt().max(1e-9);
        let displacement_m = force_n / self.spec.stiffness_n_m / denom;
        let instantaneous_index = (displacement_m * 1000.0) / self.spec.mm_per_index_unit;

        let dt = dt_s.max(0.0);
        let k = 1.0 - (-dt / self.spec.filter_tau_s).exp();
        self.filtered_index += (instantaneous_index - self.filtered_index) * k;

        VibrationState { index: self.filtered_index, instantaneous_index }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn settle(spec: SpoolSpec, omega: f64, faults: &ImbalanceFaults, seconds: f64) -> VibrationState {
        let mut v = SpoolVibration::new(spec);
        let dt = 0.02;
        let mut out = VibrationState::default();
        for _ in 0..(seconds / dt) as usize {
            out = v.step(omega, faults, dt);
        }
        out
    }

    #[test]
    fn a_balanced_rotor_reads_zero_vibration_at_any_speed_no_nan() {
        let s = settle(FAN_SPEC, 300.0, &ImbalanceFaults::default(), 5.0);
        assert_eq!(s.index, 0.0);
        assert!(!s.index.is_nan());
    }

    #[test]
    fn zero_speed_gives_zero_vibration_even_with_a_full_fault() {
        let s = settle(FAN_SPEC, 0.0, &ImbalanceFaults { blade_loss_frac: 1.0, ..Default::default() }, 5.0);
        assert_eq!(s.index, 0.0);
    }

    #[test]
    fn vibration_rises_with_the_square_of_speed_away_from_resonance() {
        // Well below the support's natural frequency, response ~ omega^2
        // (the mass-dominated/force-driven regime).
        let low = settle(FAN_SPEC, 5.0, &ImbalanceFaults { blade_loss_frac: 1.0, ..Default::default() }, 5.0);
        let high = settle(FAN_SPEC, 10.0, &ImbalanceFaults { blade_loss_frac: 1.0, ..Default::default() }, 5.0);
        assert!((high.index / low.index - 4.0).abs() < 0.5, "ratio {}", high.index / low.index);
    }

    #[test]
    fn a_blade_loss_gives_far_more_vibration_than_healthy() {
        let healthy = settle(FAN_SPEC, 300.0, &ImbalanceFaults::default(), 5.0);
        let damaged = settle(FAN_SPEC, 300.0, &ImbalanceFaults { blade_loss_frac: 1.0, ..Default::default() }, 5.0);
        assert!(damaged.index > healthy.index + 1.0);
    }

    #[test]
    fn multiple_simultaneous_causes_add_up() {
        let one = settle(FAN_SPEC, 300.0, &ImbalanceFaults { ice_frac: 0.5, ..Default::default() }, 5.0);
        let two = settle(FAN_SPEC, 300.0, &ImbalanceFaults { ice_frac: 0.5, bird_strike_frac: 0.5, ..Default::default() }, 5.0);
        assert!(two.index > one.index);
    }

    #[test]
    fn the_tracking_filter_lags_a_sudden_imbalance_rather_than_jumping_instantly() {
        let mut v = SpoolVibration::new(FAN_SPEC);
        // Run healthy first so the filter starts at zero, then apply a
        // sudden full blade-loss fault for one short step.
        v.step(300.0, &ImbalanceFaults::default(), 1.0);
        let s = v.step(300.0, &ImbalanceFaults { blade_loss_frac: 1.0, ..Default::default() }, 0.05);
        assert!(s.index > 0.0 && s.index < s.instantaneous_index, "the filtered reading should lag the instantaneous one");
    }
}
