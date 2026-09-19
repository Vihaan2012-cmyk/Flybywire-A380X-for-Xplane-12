//! Angle-of-attack vane: a small weathervane on the fuselage that aligns
//! itself with the local airflow, geared to a resolver (or synchro) that
//! reports its angle. Models the vane's own aerodynamic response lag, its
//! anti-ice heater, icing that can jam it at a fixed angle, mechanical
//! sticking, resolver drift, and physical damage (a bent vane).
//!
//! ## Local flow amplification
//! A vane mounted ahead of the fuselage sits in the aircraft's own upwash
//! field, so it senses a larger angle than the free-stream AoA. 1.10 (10%)
//! is a commonly used generic transport-aircraft planning figure for this
//! upwash amplification (not A380-specific) -- GENERIC, the same figure
//! `src/physics/adirs.rs` uses for the same reason (re-derived
//! independently here per this brief's no-cross-module-dependency rule).
//!
//! ## Vane dynamics
//! The vane is a small aerodynamic surface with its own moment of inertia
//! and restoring aerodynamic hinge moment; disturbed, it settles onto the
//! local flow angle with a short time constant rather than instantaneously.
//! Modelled as a first-order lag, which is the standard reduced-order model
//! for a weathervane-type sensor (a full second-order spring-mass-damper
//! hinge model is a refinement beyond what any moment-of-inertia/hinge-
//! stiffness figure published for this component would justify). Time
//! constant GENERIC (representative of a small, low-inertia vane: response
//! measured in tenths of a second).
//!
//! ## Icing / jamming
//! Icing on the vane's hinge/counterweight can physically stop it from
//! rotating. This is modelled the same way as the pitot/TAT heat balance
//! (Messinger-style: ice accretes when the heater cannot supply the heat a
//! (much smaller) surface needs, melts when it can) -- ATA 34's own accident
//! history (e.g. published NTSB/BEA findings on AoA-vane icing/jamming
//! contributing to unreliable-airspeed and stall-warning events) is the
//! reason this is treated as a first-class fault rather than only "AoA
//! wrong", not just "sensor noisy": a jammed vane holds its *last free*
//! angle indefinitely, independent of subsequent manoeuvring.
//!
//! ## Resolver drift and damage
//! A resolver's excitation/output windings can develop a slowly growing
//! zero-offset error with wear (electromechanical degradation) -- modelled
//! as a bias that random-walks at a rate scaled by the fault fraction
//! (GENERIC: no published resolver drift-rate spec for this component).
//! Physical damage (bird strike, ground handling) is modelled as a fixed
//! bias proportional to the fault fraction, up to a GENERIC maximum bend
//! angle.

use super::rng::Rng;

const UPWASH_FACTOR: f64 = 1.10;
/// Vane aerodynamic response time constant, seconds. GENERIC.
const VANE_TAU_S: f64 = 0.15;
/// Heater rating, W. GENERIC: an AoA vane is much smaller than a pitot tube,
/// so its heater is rated lower.
const RATED_HEATER_W: f64 = 60.0;
/// Vane frontal/wetted area exposed to icing, m^2. GENERIC (a small vane).
const VANE_AREA_M2: f64 = 0.01;
/// Ice mass that jams the vane's hinge, kg. GENERIC.
const ICE_JAM_MASS_KG: f64 = 0.0003;
const WATER_LF_J_KG: f64 = 334_000.0;
const WATER_CP_J_KGK: f64 = 4186.0;
/// Simplified convective coefficient scaling with TAS (a flat-plate-like
/// correlation rather than the pitot's cylinder correlation, since a vane's
/// cross-section is aerofoil-like, not cylindrical) -- GENERIC coefficient
/// chosen so the heat duty is of the same order as the rated heater power in
/// representative icing/TAS conditions.
const CONVECTIVE_COEFF: f64 = 6.0;
/// Resolver drift-rate scale at fault magnitude 1.0, degrees per hour.
/// GENERIC.
const RESOLVER_DRIFT_DEG_PER_HR: f64 = 2.0;
/// Maximum fixed bias a fully bent/damaged vane reads, degrees. GENERIC.
const MAX_DAMAGE_BIAS_DEG: f64 = 8.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct AoaVaneFaults {
    pub heater_failure: f64,
    /// Commanded/mechanically seized at its current position (bearing
    /// corrosion, foreign object): `1.0` fully seized.
    pub mechanically_stuck: f64,
    /// Resolver electromechanical wear driving a slow bias drift.
    pub resolver_wear: f64,
    /// Physical damage (bent vane): a fixed bias, scaled by this fraction.
    pub damage: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct AoaVaneOutput {
    /// The angle the resolver reports, degrees.
    pub sensed_aoa_deg: f64,
    pub jammed_by_ice: bool,
    pub heater_power_w: f64,
    pub ice_kg: f64,
}

#[derive(Clone, Debug)]
pub struct AoaVane {
    vane_angle_deg: f64,
    ice_kg: f64,
    resolver_bias_deg: f64,
    rng: Rng,
}

impl AoaVane {
    pub fn new(seed: u64, initial_aoa_deg: f64) -> Self {
        Self { vane_angle_deg: initial_aoa_deg * UPWASH_FACTOR, ice_kg: 0.0, resolver_bias_deg: 0.0, rng: Rng::new(seed) }
    }

    /// `true_aoa_deg`: free-stream angle of attack. `tas_ms`, `sat_c`,
    /// `lwc_gm3`: for the icing heat balance. `powered`: heater bus live.
    pub fn step(
        &mut self,
        true_aoa_deg: f64,
        tas_ms: f64,
        sat_c: f64,
        lwc_gm3: f64,
        powered: bool,
        faults: &AoaVaneFaults,
        dt_s: f64,
    ) -> AoaVaneOutput {
        let dt = dt_s.max(0.0);
        let local_aoa_deg = true_aoa_deg * UPWASH_FACTOR;

        // ---- Icing heat balance (Messinger-style, see module docs).
        let rated_w = if powered { RATED_HEATER_W * (1.0 - faults.heater_failure.clamp(0.0, 1.0)) } else { 0.0 };
        let delta_t = (0.0 - sat_c).max(0.0);
        let required_w = CONVECTIVE_COEFF * VANE_AREA_M2 * delta_t * (1.0 + tas_ms.max(0.0) / 100.0);
        let lwc_kg_m3 = lwc_gm3.max(0.0) * 1e-3;
        let catch_kg_s = lwc_kg_m3 * tas_ms.max(0.0) * VANE_AREA_M2 * 0.1;
        let q_water = catch_kg_s * (WATER_CP_J_KGK * delta_t + WATER_LF_J_KG);
        let total_required_w = required_w + q_water;
        let deficit_w = (total_required_w - rated_w).max(0.0);
        let surplus_w = (rated_w - total_required_w).max(0.0);
        let accretion_kg_s = if total_required_w > 0.0 { catch_kg_s * (deficit_w / total_required_w).min(1.0) } else { 0.0 };
        let melt_kg_s = surplus_w / WATER_LF_J_KG;
        self.ice_kg = (self.ice_kg + (accretion_kg_s - melt_kg_s) * dt).max(0.0);
        let jammed_by_ice = self.ice_kg >= ICE_JAM_MASS_KG;

        // ---- Mechanical position: free-following unless jammed (by ice or
        // by the mechanical-stuck fault), in which case the vane simply
        // keeps whatever angle it last had.
        let stuck = jammed_by_ice || faults.mechanically_stuck.clamp(0.0, 1.0) >= 0.98;
        if !stuck {
            let tau = VANE_TAU_S.max(1e-6);
            let k = (-dt / tau).exp();
            self.vane_angle_deg = local_aoa_deg + (self.vane_angle_deg - local_aoa_deg) * k;
        }

        // ---- Resolver drift: a slow random walk scaled by wear fraction.
        let wear = faults.resolver_wear.clamp(0.0, 1.0);
        if wear > 0.0 && dt > 0.0 {
            let sigma_deg = RESOLVER_DRIFT_DEG_PER_HR * wear / 3600.0 * dt.sqrt();
            self.resolver_bias_deg += self.rng.gaussian() * sigma_deg;
        }

        // ---- Damage: fixed bias, deterministic function of the fault
        // fraction (a bent vane doesn't wander further once bent).
        let damage_bias_deg = MAX_DAMAGE_BIAS_DEG * faults.damage.clamp(0.0, 1.0);

        AoaVaneOutput {
            sensed_aoa_deg: self.vane_angle_deg + self.resolver_bias_deg + damage_bias_deg,
            jammed_by_ice,
            heater_power_w: rated_w,
            ice_kg: self.ice_kg,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn healthy_vane_tracks_true_aoa_with_upwash_after_settling() {
        let mut vane = AoaVane::new(1, 0.0);
        let mut out = AoaVaneOutput::default();
        for _ in 0..200 {
            out = vane.step(5.0, 150.0, 15.0, 0.0, true, &AoaVaneFaults::default(), 0.05);
        }
        assert!((out.sensed_aoa_deg - 5.0 * UPWASH_FACTOR).abs() < 0.05, "{}", out.sensed_aoa_deg);
        assert!(!out.jammed_by_ice);
    }

    #[test]
    fn vane_lags_a_step_change_rather_than_jumping() {
        let mut vane = AoaVane::new(1, 0.0);
        let out = vane.step(10.0, 150.0, 15.0, 0.0, true, &AoaVaneFaults::default(), 0.05);
        assert!(out.sensed_aoa_deg < 10.0 * UPWASH_FACTOR);
        assert!(out.sensed_aoa_deg > 0.0);
    }

    #[test]
    fn unheated_icing_jams_the_vane_and_it_holds_its_last_free_angle() {
        let mut vane = AoaVane::new(2, 3.0);
        let faults = AoaVaneFaults::default();
        // Settle at 3 degrees first, healthy and warm (no icing yet).
        for _ in 0..100 {
            vane.step(3.0, 150.0, 15.0, 0.0, true, &faults, 0.05);
        }
        // Now cold, wet, unpowered: ices up and jams.
        let mut out = AoaVaneOutput::default();
        for _ in 0..2000 {
            out = vane.step(3.0, 150.0, -25.0, 0.8, false, &faults, 0.05);
        }
        assert!(out.jammed_by_ice);
        let jammed_reading = out.sensed_aoa_deg;
        // The true AoA now changes a lot (a manoeuvre); the jammed vane
        // must not follow.
        for _ in 0..200 {
            out = vane.step(15.0, 150.0, -25.0, 0.8, false, &faults, 0.05);
        }
        assert!((out.sensed_aoa_deg - jammed_reading).abs() < 0.01);
    }

    #[test]
    fn heater_prevents_jamming_in_the_same_conditions() {
        let mut vane = AoaVane::new(3, 3.0);
        let faults = AoaVaneFaults::default();
        let mut out = AoaVaneOutput::default();
        for _ in 0..2000 {
            out = vane.step(3.0, 150.0, -25.0, 0.8, true, &faults, 0.05);
        }
        assert!(!out.jammed_by_ice);
    }

    #[test]
    fn mechanically_stuck_fault_freezes_the_vane_even_without_ice() {
        let mut vane = AoaVane::new(4, 0.0);
        let mut faults = AoaVaneFaults::default();
        vane.step(2.0, 150.0, 15.0, 0.0, true, &faults, 1.0);
        let before = vane.step(2.0, 150.0, 15.0, 0.0, true, &faults, 1.0).sensed_aoa_deg;
        faults.mechanically_stuck = 1.0;
        let after = vane.step(20.0, 150.0, 15.0, 0.0, true, &faults, 1.0).sensed_aoa_deg;
        assert!((after - before).abs() < 1e-6);
    }

    #[test]
    fn resolver_drift_accumulates_over_time_when_worn() {
        let mut worn = AoaVane::new(5, 0.0);
        let mut healthy = AoaVane::new(5, 0.0);
        let worn_faults = AoaVaneFaults { resolver_wear: 1.0, ..Default::default() };
        let mut worn_out = AoaVaneOutput::default();
        let mut healthy_out = AoaVaneOutput::default();
        for _ in 0..7200 {
            worn_out = worn.step(2.0, 150.0, 15.0, 0.0, true, &worn_faults, 1.0);
            healthy_out = healthy.step(2.0, 150.0, 15.0, 0.0, true, &AoaVaneFaults::default(), 1.0);
        }
        assert!((worn_out.sensed_aoa_deg - healthy_out.sensed_aoa_deg).abs() > 0.01);
    }

    #[test]
    fn damage_introduces_a_fixed_bias_proportional_to_the_fault() {
        let mut vane = AoaVane::new(6, 5.0);
        let half_damage = AoaVaneFaults { damage: 0.5, ..Default::default() };
        let mut out = AoaVaneOutput::default();
        for _ in 0..100 {
            out = vane.step(5.0, 150.0, 15.0, 0.0, true, &half_damage, 0.05);
        }
        let expected = 5.0 * UPWASH_FACTOR + 0.5 * MAX_DAMAGE_BIAS_DEG;
        assert!((out.sensed_aoa_deg - expected).abs() < 0.05, "{}", out.sensed_aoa_deg);
    }

    #[test]
    fn no_nan_at_zero_dt_or_rest() {
        let mut vane = AoaVane::new(7, 0.0);
        let out = vane.step(0.0, 0.0, 15.0, 0.0, false, &AoaVaneFaults::default(), 0.0);
        assert!(out.sensed_aoa_deg.is_finite());
    }
}
