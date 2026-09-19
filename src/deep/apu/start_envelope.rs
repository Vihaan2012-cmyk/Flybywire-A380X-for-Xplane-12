//! Start envelope by altitude, temperature and airspeed -- windmilling,
//! relight/light-off limits at altitude, and the ram conditions the APU's
//! inlet scoop sees at airspeed. Cold-soaked oil's effect on cranking drag
//! lives in `oil.rs` (`OilSystem::cold_drag_torque_nm`); cold-soaked
//! battery effects belong to whatever supplies this directory's
//! `interfaces::BatteryInput` (`deep::electrical`'s own model, see
//! `interfaces.rs`) -- this file only reacts to whatever voltage/resistance
//! that interface hands the starter, it does not model the battery itself.
//!
//! The APU's inlet is a tailcone scoop, not a forward-facing podded-engine
//! inlet, so it only partially couples to freestream ram effects
//! (`params::SCOOP_RAM_COUPLING_FRAC`) -- windmilling and ram heating/
//! pressurisation are real but modest for this class of installation,
//! unlike a main engine's own inlet.

use super::gas;
use super::params;

#[derive(Clone, Copy, Debug)]
pub struct FlightCondition {
    pub ambient_pressure_pa: f64,
    pub ambient_temperature_k: f64,
    pub true_airspeed_mps: f64,
}

impl FlightCondition {
    pub fn ground(ambient_pressure_pa: f64, ambient_temperature_k: f64) -> Self {
        Self { ambient_pressure_pa, ambient_temperature_k, true_airspeed_mps: 0.0 }
    }

    /// Freestream Mach number from true airspeed and the local speed of
    /// sound, `a = sqrt(gamma*R*T)`.
    pub fn mach(&self) -> f64 {
        let t = self.ambient_temperature_k.max(1.0);
        let speed_of_sound = (gas::GAMMA_AIR * gas::R_AIR_J_KG_K * t).sqrt();
        (self.true_airspeed_mps.max(0.0) / speed_of_sound).max(0.0)
    }

    /// The effective Mach number the scoop inlet actually sees, after its
    /// partial ram coupling.
    pub fn scoop_effective_mach(&self) -> f64 {
        self.mach() * params::SCOOP_RAM_COUPLING_FRAC
    }

    /// The total (ram-recovered) temperature and pressure at the compressor
    /// face, fed to `power_section.rs` in place of plain static ambient
    /// conditions.
    pub fn inlet_total_conditions(&self) -> (f64, f64) {
        let mach = self.scoop_effective_mach();
        let t = gas::total_temperature_k(self.ambient_temperature_k, mach, gas::GAMMA_AIR);
        let p = gas::total_pressure_pa(self.ambient_pressure_pa, mach, gas::GAMMA_AIR);
        (t, p)
    }

    /// Ambient (static, altitude-driven) density ratio to ISA sea level --
    /// what actually governs whether combustion can be sustained at
    /// altitude, not airspeed.
    pub fn density_ratio(&self) -> f64 {
        gas::density_ratio(self.ambient_pressure_pa, self.ambient_temperature_k)
    }

    /// True (static) freestream dynamic pressure, `0.5*rho*V^2`, from the
    /// ideal gas law (`rho = P/(R*T)`).
    pub fn dynamic_pressure_pa(&self) -> f64 {
        let t = self.ambient_temperature_k.max(1.0);
        let rho = self.ambient_pressure_pa.max(0.0) / (gas::R_AIR_J_KG_K * t);
        0.5 * rho * self.true_airspeed_mps.max(0.0).powi(2)
    }
}

/// Relight/light-off is only certified above a minimum ambient density
/// (`params::MIN_RELIGHT_DENSITY_RATIO`, a generic ~20,000 ft-class ceiling)
/// -- below it, `starter.rs` refuses ignition regardless of cranking speed,
/// exactly like a real APU relight envelope limit.
pub fn relight_permitted(fc: &FlightCondition) -> bool {
    fc.density_ratio() >= params::MIN_RELIGHT_DENSITY_RATIO
}

/// The torque ram air passing through the (open) inlet scoop imparts to the
/// spool with no starter or combustion running -- a real but modest
/// windmilling effect for this class of scoop inlet (see module docs),
/// decaying to zero as the spool itself approaches the speed the ram flow
/// alone can drive it to (modelled simply as decaying with `1 - n_frac`,
/// so it never by itself drives the spool past a plausible windmill speed;
/// the actual equilibrium point still falls out of the torque balance
/// against the compressor's own drag in `power_section.rs`, not asserted
/// here).
pub fn windmill_torque_nm(fc: &FlightCondition, n_frac: f64) -> f64 {
    let q = fc.dynamic_pressure_pa() * params::SCOOP_RAM_COUPLING_FRAC;
    params::WINDMILL_TORQUE_COEFF_NM_PER_PA * q * (1.0 - n_frac).clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ground_static_has_zero_mach_and_zero_dynamic_pressure() {
        let fc = FlightCondition::ground(101_325.0, 288.15);
        assert_eq!(fc.mach(), 0.0);
        assert_eq!(fc.dynamic_pressure_pa(), 0.0);
        let (t, p) = fc.inlet_total_conditions();
        assert!((t - 288.15).abs() < 1e-6);
        assert!((p - 101_325.0).abs() < 1e-3);
    }

    #[test]
    fn dynamic_pressure_and_ram_conditions_rise_with_airspeed() {
        let slow = FlightCondition { ambient_pressure_pa: 101_325.0, ambient_temperature_k: 288.15, true_airspeed_mps: 50.0 };
        let fast = FlightCondition { ambient_pressure_pa: 101_325.0, ambient_temperature_k: 288.15, true_airspeed_mps: 250.0 };
        assert!(fast.dynamic_pressure_pa() > slow.dynamic_pressure_pa());
        let (t_slow, p_slow) = slow.inlet_total_conditions();
        let (t_fast, p_fast) = fast.inlet_total_conditions();
        assert!(t_fast > t_slow);
        assert!(p_fast > p_slow);
    }

    #[test]
    fn relight_is_permitted_at_sea_level_and_refused_high_enough() {
        let sea_level = FlightCondition::ground(101_325.0, 288.15);
        assert!(relight_permitted(&sea_level));
        // A representative ~35,000 ft ISA point: ~23.8 kPa, ~218.8 K --
        // density ratio well below the generic 20,000 ft-class ceiling.
        let high = FlightCondition::ground(23_800.0, 218.8);
        assert!(!relight_permitted(&high));
    }

    #[test]
    fn windmill_torque_grows_with_airspeed_and_decays_as_speed_approaches_full() {
        let fc = FlightCondition { ambient_pressure_pa: 101_325.0, ambient_temperature_k: 288.15, true_airspeed_mps: 230.0 };
        let low_speed = windmill_torque_nm(&fc, 0.0);
        let near_full = windmill_torque_nm(&fc, 0.95);
        assert!(low_speed > 0.0);
        assert!(near_full < low_speed);
        assert_eq!(windmill_torque_nm(&fc, 1.0), 0.0);

        let stationary = FlightCondition::ground(101_325.0, 288.15);
        assert_eq!(windmill_torque_nm(&stationary, 0.0), 0.0);
    }

    #[test]
    fn nothing_is_nan_at_pathological_inputs() {
        let fc = FlightCondition { ambient_pressure_pa: 0.0, ambient_temperature_k: 0.0, true_airspeed_mps: -10.0 };
        assert!(fc.mach().is_finite());
        assert!(fc.dynamic_pressure_pa().is_finite());
        assert!(fc.density_ratio().is_finite());
        assert!(windmill_torque_nm(&fc, 0.5).is_finite());
    }
}
