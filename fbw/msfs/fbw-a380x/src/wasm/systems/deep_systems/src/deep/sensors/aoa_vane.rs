use super::rng::Rng;

pub(crate) const UPWASH_FACTOR: f64 = 1.10;
const VANE_TAU_S: f64 = 0.15;
pub const RATED_HEATER_W: f64 = 60.0;
const VANE_AREA_M2: f64 = 0.01;
const ICE_JAM_MASS_KG: f64 = 0.0003;
const WATER_LF_J_KG: f64 = 334_000.0;
const WATER_CP_J_KGK: f64 = 4186.0;
const CONVECTIVE_COEFF: f64 = 6.0;
const RESOLVER_DRIFT_DEG_PER_HR: f64 = 2.0;
const RESOLVER_JITTER_DEG_PER_SQRT_HR: f64 = 0.2;
const MAX_DAMAGE_BIAS_DEG: f64 = 8.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct AoaVaneFaults {
    pub heater_failure: f64,
    pub mechanically_stuck: f64,
    pub resolver_wear: f64,
    pub damage: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct AoaVaneOutput {
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
    resolver_drift_sign: f64,
    rng: Rng,
}

impl AoaVane {
    pub fn new(seed: u64, initial_aoa_deg: f64) -> Self {
        let mut rng = Rng::new(seed);
        let resolver_drift_sign = if rng.next_f64() < 0.5 { -1.0 } else { 1.0 };
        Self {
            vane_angle_deg: initial_aoa_deg * UPWASH_FACTOR,
            ice_kg: 0.0,
            resolver_bias_deg: 0.0,
            resolver_drift_sign,
            rng,
        }
    }

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

        let stuck = jammed_by_ice || faults.mechanically_stuck.clamp(0.0, 1.0) >= 0.98;
        if !stuck {
            let tau = VANE_TAU_S.max(1e-6);
            let k = (-dt / tau).exp();
            self.vane_angle_deg = local_aoa_deg + (self.vane_angle_deg - local_aoa_deg) * k;
        }

        let wear = faults.resolver_wear.clamp(0.0, 1.0);
        if wear > 0.0 && dt > 0.0 {
            let dt_hr = dt / 3600.0;
            self.resolver_bias_deg += self.resolver_drift_sign * RESOLVER_DRIFT_DEG_PER_HR * wear * dt_hr;
            let sigma_deg = RESOLVER_JITTER_DEG_PER_SQRT_HR * wear * dt_hr.sqrt();
            self.resolver_bias_deg += self.rng.gaussian() * sigma_deg;
        }

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
        for _ in 0..100 {
            vane.step(3.0, 150.0, 15.0, 0.0, true, &faults, 0.05);
        }
        let mut out = AoaVaneOutput::default();
        for _ in 0..2000 {
            out = vane.step(3.0, 150.0, -25.0, 0.8, false, &faults, 0.05);
        }
        assert!(out.jammed_by_ice);
        let jammed_reading = out.sensed_aoa_deg;
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
        for _ in 0..3600 {
            worn_out = worn.step(2.0, 150.0, 15.0, 0.0, true, &worn_faults, 1.0);
            healthy_out = healthy.step(2.0, 150.0, 15.0, 0.0, true, &AoaVaneFaults::default(), 1.0);
        }
        let after_1h = (worn_out.sensed_aoa_deg - healthy_out.sensed_aoa_deg).abs();
        for _ in 0..3600 {
            worn_out = worn.step(2.0, 150.0, 15.0, 0.0, true, &worn_faults, 1.0);
            healthy_out = healthy.step(2.0, 150.0, 15.0, 0.0, true, &AoaVaneFaults::default(), 1.0);
        }
        let after_2h = (worn_out.sensed_aoa_deg - healthy_out.sensed_aoa_deg).abs();
        assert!((after_1h - 2.0).abs() < 0.6, "1 h drift {after_1h}");
        assert!((after_2h - 4.0).abs() < 0.85, "2 h drift {after_2h}");
        assert!(after_2h > after_1h * 1.5, "{after_1h} -> {after_2h}");
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
