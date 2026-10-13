//! Starter duty-cycle heating: the air turbine starter's gearbox and
//! bearings run hot under load (turbine and gear friction, none of it ever
//! ideal), and a starter has no dedicated cooling of its own -- it relies on
//! natural convection to the nacelle between starts. That is the entire
//! physical reason real starter duty cycles exist ("one start, then wait N
//! minutes, up to M consecutive attempts, then a longer cooldown"): not an
//! arbitrary rule, a single thermal mass that heats under load and cools
//! slowly at rest, exactly like `physics::engine::hot_section`'s casing
//! model but sized for a much smaller accessory, with a much shorter time
//! constant.
//!
//! No Trent-900 ATS thermal data is public. Every figure below is
//! **GENERIC**: the thermal mass and heat-loss conductance are sized so a
//! single ~60 s crank raises the housing a physically modest amount and a
//! healthy engine-off soak brings it back down within several minutes,
//! matching the qualitative shape of real starter duty-cycle limits (a
//! handful of consecutive starts before a mandatory extended cooldown)
//! without any published Trent 900/A380 number to calibrate the exact
//! limit against.

use super::turbine::PEAK_POWER_W;

/// Housing thermal mass (steel/aluminium gearbox casing + oil), J/K, and the
/// natural-convection loss to the nacelle, W/K (**GENERIC**, chosen
/// together so a single ~60 s crank at typical cranking power stays well
/// under the overheat threshold below while several back-to-back cranks
/// with no cooldown between them cross it -- the ~8 minute thermal time
/// constant this pair implies is the right order of magnitude for the
/// "wait N minutes between attempts" shape of a real starter duty cycle,
/// not a measured Trent 900/A380 figure).
const THERMAL_MASS_J_K: f64 = 45_000.0;
const LOSS_W_K: f64 = 90.0;
/// Fraction of the turbine's mechanical power converted to heat in the
/// gearbox/bearings rather than delivered to the spool (**GENERIC**,
/// typical of a geared accessory under heavy transient load).
const HEAT_FRACTION: f64 = 0.15;
/// Housing temperature rise above ambient at which the starter is
/// considered overheated and further cranking should not be commanded
/// (**GENERIC**).
const OVERHEAT_RISE_K: f64 = 120.0;

#[derive(Clone, Copy, Debug)]
pub struct DutyCycleHeat {
    rise_k: f64,
}

impl DutyCycleHeat {
    pub fn new() -> Self {
        Self { rise_k: 0.0 }
    }

    /// Temperature rise above ambient, K.
    pub fn rise_k(&self) -> f64 {
        self.rise_k
    }

    pub fn overheated(&self) -> bool {
        self.rise_k >= OVERHEAT_RISE_K
    }

    /// One step. `cranking_power_w` is the mechanical power the turbine is
    /// currently producing (`turbine::AtsState::torque_n_m` times the
    /// rotor's own angular speed, or zero when not cranking); `ambient_k`
    /// only sets the reference the rise is measured from, this state itself
    /// tracks the *rise*, which is exact-exponential and rest-safe at
    /// `dt = 0` and zero power.
    pub fn step(&mut self, cranking_power_w: f64, dt_s: f64) {
        let dt = dt_s.max(0.0);
        let heat_in_w = cranking_power_w.max(0.0) * HEAT_FRACTION;
        let target = heat_in_w / LOSS_W_K;
        let k = LOSS_W_K / THERMAL_MASS_J_K;
        self.rise_k = target + (self.rise_k - target) * (-k * dt).exp();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn starting_from_cold_gives_zero_rise_and_no_nan() {
        let h = DutyCycleHeat::new();
        assert_eq!(h.rise_k(), 0.0);
        assert!(!h.overheated());
    }

    #[test]
    fn a_single_normal_length_crank_does_not_overheat_it() {
        let mut h = DutyCycleHeat::new();
        for _ in 0..(60.0 / 0.1) as usize {
            h.step(PEAK_POWER_W * 0.6, 0.1);
        }
        assert!(!h.overheated(), "{} K rise after one start", h.rise_k());
        assert!(h.rise_k() > 0.0);
    }

    #[test]
    fn repeated_back_to_back_cranks_without_cooldown_eventually_overheat_it() {
        let mut h = DutyCycleHeat::new();
        for _ in 0..(600.0 / 0.1) as usize {
            h.step(PEAK_POWER_W * 0.6, 0.1);
        }
        assert!(h.overheated(), "{} K rise after ten minutes continuous cranking", h.rise_k());
    }

    #[test]
    fn it_cools_back_down_once_cranking_stops() {
        let mut h = DutyCycleHeat::new();
        for _ in 0..(60.0 / 0.1) as usize {
            h.step(PEAK_POWER_W * 0.6, 0.1);
        }
        let after_crank = h.rise_k();
        for _ in 0..(600.0 / 0.5) as usize {
            h.step(0.0, 0.5);
        }
        assert!(h.rise_k() < after_crank * 0.5, "should have cooled substantially in ten minutes at rest");
    }

    #[test]
    fn zero_dt_never_produces_nan() {
        let mut h = DutyCycleHeat::new();
        h.step(PEAK_POWER_W, 0.0);
        assert!(!h.rise_k().is_nan());
    }
}
