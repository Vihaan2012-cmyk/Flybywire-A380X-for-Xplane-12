pub const GAMMA_AIR: f64 = 1.4;
pub const R_AIR: f64 = 8.314462618 / 0.0289647;
pub const CP_AIR: f64 = 1005.0;

pub const LATENT_HEAT_FUSION_WATER_J_KG: f64 = 334_000.0;
pub const LATENT_HEAT_VAPORIZATION_WATER_J_KG: f64 = 2_501_000.0;
pub const CP_WATER_J_KG_K: f64 = 4186.0;
pub const CP_ICE_J_KG_K: f64 = 2100.0;
pub const DENSITY_ICE_KG_M3: f64 = 917.0;
pub const DENSITY_WATER_KG_M3: f64 = 1000.0;

pub const STEFAN_BOLTZMANN: f64 = 5.670_374e-8;
pub const PSI_TO_PA: f64 = 6894.757;
pub const KT_TO_M_S: f64 = 0.514444;

pub fn orifice_mass_flow_kg_s(cd: f64, area_m2: f64, p_up_pa: f64, t_up_k: f64, p_down_pa: f64) -> f64 {
    if p_up_pa <= 0.0 || t_up_k <= 0.0 || area_m2 <= 0.0 {
        return 0.0;
    }
    let critical_ratio = (2.0 / (GAMMA_AIR + 1.0)).powf(GAMMA_AIR / (GAMMA_AIR - 1.0));
    let pr = (p_down_pa / p_up_pa).clamp(0.0, 1.0);
    if pr <= critical_ratio {
        cd * area_m2
            * p_up_pa
            * (GAMMA_AIR / (R_AIR * t_up_k)).sqrt()
            * (2.0 / (GAMMA_AIR + 1.0)).powf((GAMMA_AIR + 1.0) / (2.0 * (GAMMA_AIR - 1.0)))
    } else {
        let term = (pr.powf(2.0 / GAMMA_AIR) - pr.powf((GAMMA_AIR + 1.0) / GAMMA_AIR)).max(0.0);
        cd * area_m2 * p_up_pa * ((2.0 * GAMMA_AIR) / (R_AIR * t_up_k * (GAMMA_AIR - 1.0)) * term).sqrt()
    }
}

pub fn saturation_vapor_pressure_pa(temp_c: f64) -> f64 {
    610.78 * (17.27 * temp_c / (temp_c + 237.3)).exp()
}

pub fn clamp01(x: f64) -> f64 {
    if x.is_nan() {
        0.0
    } else {
        x.clamp(0.0, 1.0)
    }
}

pub fn recovery_temperature_c(static_air_c: f64, tas_m_s: f64, recovery_factor: f64) -> f64 {
    let t_static_k = (static_air_c + 273.15).max(1.0);
    let sonic_m_s = (GAMMA_AIR * R_AIR * t_static_k).sqrt();
    let mach = (tas_m_s.max(0.0) / sonic_m_s).min(3.0);
    let t_recovery_k = t_static_k * (1.0 + recovery_factor * (GAMMA_AIR - 1.0) / 2.0 * mach * mach);
    t_recovery_k - 273.15
}

pub fn surface_net_loss_w_m2(
    h_w_m2k: f64,
    static_air_c: f64,
    recovery_c: f64,
    lwc_kg_m3: f64,
    beta0: f64,
    tas_m_s: f64,
    ambient_pressure_pa: f64,
    surface_c: f64,
) -> (f64, f64) {
    let impingement = (clamp01(beta0) * lwc_kg_m3.max(0.0) * tas_m_s.max(0.0)).max(0.0);

    let q_conv = h_w_m2k.max(0.0) * (surface_c - recovery_c);

    let q_evap = if impingement > 0.0 {
        let h_m = h_w_m2k.max(0.0) / CP_AIR;
        let e_surf = saturation_vapor_pressure_pa(surface_c);
        let e_air = saturation_vapor_pressure_pa(static_air_c);
        let p = ambient_pressure_pa.max(1000.0);
        h_m * LATENT_HEAT_VAPORIZATION_WATER_J_KG * 0.622 / p * (e_surf - e_air)
    } else {
        0.0
    };

    let q_sensible_water = impingement * CP_WATER_J_KG_K * (surface_c - static_air_c);

    let q_kinetic = impingement * 0.5 * tas_m_s.max(0.0).powi(2);

    (q_conv + q_evap + q_sensible_water - q_kinetic, impingement)
}

#[derive(Clone, Copy, Debug, Default)]
pub struct MessingerResult {
    pub impingement_kg_m2_s: f64,
    pub freezing_fraction: f64,
    pub anti_ice_demand_w_m2: f64,
}

pub fn messinger_freezing_fraction(
    h_w_m2k: f64,
    static_air_c: f64,
    recovery_c: f64,
    lwc_kg_m3: f64,
    beta0: f64,
    tas_m_s: f64,
    ambient_pressure_pa: f64,
) -> MessingerResult {
    let (net_loss, impingement) =
        surface_net_loss_w_m2(h_w_m2k, static_air_c, recovery_c, lwc_kg_m3, beta0, tas_m_s, ambient_pressure_pa, 0.0);
    let freezing_fraction = if impingement > 1e-9 {
        clamp01(net_loss / (impingement * LATENT_HEAT_FUSION_WATER_J_KG))
    } else {
        0.0
    };
    MessingerResult { impingement_kg_m2_s: impingement, freezing_fraction, anti_ice_demand_w_m2: net_loss.max(0.0) }
}

pub fn equilibrium_surface_c_with_heater(
    h_w_m2k: f64,
    static_air_c: f64,
    recovery_c: f64,
    lwc_kg_m3: f64,
    beta0: f64,
    tas_m_s: f64,
    ambient_pressure_pa: f64,
    heater_w_m2: f64,
) -> f64 {
    let (mut lo, mut hi) = (-80.0_f64, 500.0_f64);
    for _ in 0..48 {
        let mid = 0.5 * (lo + hi);
        let (loss, _) = surface_net_loss_w_m2(h_w_m2k, static_air_c, recovery_c, lwc_kg_m3, beta0, tas_m_s, ambient_pressure_pa, mid);
        if loss < heater_w_m2 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

#[allow(clippy::too_many_arguments)]
pub fn equilibrium_surface_c_with_bleed(
    h_w_m2k: f64,
    static_air_c: f64,
    recovery_c: f64,
    lwc_kg_m3: f64,
    beta0: f64,
    tas_m_s: f64,
    ambient_pressure_pa: f64,
    bleed_slope_w_m2k: f64,
    bleed_supply_c: f64,
) -> f64 {
    let (mut lo, mut hi) = (-80.0_f64, bleed_supply_c.max(-79.0));
    for _ in 0..48 {
        let mid = 0.5 * (lo + hi);
        let (loss, _) = surface_net_loss_w_m2(h_w_m2k, static_air_c, recovery_c, lwc_kg_m3, beta0, tas_m_s, ambient_pressure_pa, mid);
        let heater = bleed_slope_w_m2k.max(0.0) * (bleed_supply_c - mid).max(0.0);
        if loss < heater {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

pub fn relax_toward_equilibrium_c(current_c: f64, equilibrium_c: f64, tau_s: f64, dt_s: f64) -> f64 {
    if tau_s <= 0.0 {
        return equilibrium_c;
    }
    equilibrium_c + (current_c - equilibrium_c) * (-dt_s.max(0.0) / tau_s).exp()
}

pub fn droplet_inertia_parameter(
    droplet_diameter_m: f64,
    tas_m_s: f64,
    characteristic_length_m: f64,
    dynamic_viscosity_air_pa_s: f64,
) -> f64 {
    (DENSITY_WATER_KG_M3 * droplet_diameter_m.powi(2) * tas_m_s.max(0.0))
        / (18.0 * dynamic_viscosity_air_pa_s.max(1e-9) * characteristic_length_m.max(1e-6))
}

pub fn collection_efficiency_beta0(inertia_k: f64) -> f64 {
    clamp01(1.40 * (inertia_k - 0.125))
}

pub fn air_dynamic_viscosity_pa_s(temp_c: f64) -> f64 {
    const MU0: f64 = 1.716e-5;
    const T0: f64 = 273.15;
    const S: f64 = 110.4;
    let t = (temp_c + 273.15).max(1.0);
    MU0 * (t / T0).powf(1.5) * (T0 + S) / (t + S)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orifice_flow_is_zero_with_no_pressure_or_area() {
        assert_eq!(orifice_mass_flow_kg_s(0.65, 0.0, 5e5, 300.0, 1e5), 0.0);
        assert_eq!(orifice_mass_flow_kg_s(0.65, 1e-4, 0.0, 300.0, 1e5), 0.0);
    }

    #[test]
    fn orifice_flow_increases_with_upstream_pressure() {
        let low = orifice_mass_flow_kg_s(0.65, 1e-4, 2e5, 300.0, 1e5);
        let high = orifice_mass_flow_kg_s(0.65, 1e-4, 4e5, 300.0, 1e5);
        assert!(high > low);
    }

    #[test]
    fn saturation_pressure_matches_known_boiling_point_check() {
        assert!((saturation_vapor_pressure_pa(0.0) - 611.0).abs() < 5.0);
    }

    #[test]
    fn recovery_temperature_rises_above_static_with_speed() {
        let static_c = -20.0;
        let slow = recovery_temperature_c(static_c, 50.0, 0.9);
        let fast = recovery_temperature_c(static_c, 250.0, 0.9);
        assert!(fast > slow);
        assert!(slow > static_c);
    }

    #[test]
    fn high_speed_recovery_heating_can_exceed_freezing_even_in_cold_static_air() {
        let recovery = recovery_temperature_c(-10.0, 250.0, 0.9);
        assert!(recovery > 0.0, "recovery {recovery}");
    }

    #[test]
    fn messinger_gives_full_freezing_in_cold_still_air_and_no_freezing_when_hot() {
        let cold = messinger_freezing_fraction(80.0, -20.0, -20.0, 5e-4, 0.5, 25.0, 80_000.0);
        assert_eq!(cold.freezing_fraction, 1.0, "{cold:?}");
        assert!(cold.impingement_kg_m2_s > 0.0);
        assert!(cold.anti_ice_demand_w_m2 > 0.0);

        let glaze = messinger_freezing_fraction(80.0, -20.0, -20.0, 5e-4, 0.5, 100.0, 80_000.0);
        assert!(
            glaze.freezing_fraction > 0.0 && glaze.freezing_fraction < 1.0,
            "{glaze:?}"
        );
        assert!(glaze.freezing_fraction < cold.freezing_fraction);

        let hot = messinger_freezing_fraction(80.0, 20.0, 20.0, 5e-4, 0.5, 100.0, 101_325.0);
        assert_eq!(hot.freezing_fraction, 0.0);
        assert_eq!(hot.anti_ice_demand_w_m2, 0.0);
    }

    #[test]
    fn heater_can_hold_surface_above_freezing_and_needs_more_power_when_colder() {
        let mild = equilibrium_surface_c_with_heater(80.0, -10.0, -10.0, 5e-4, 0.5, 100.0, 90_000.0, 3000.0);
        let cold = equilibrium_surface_c_with_heater(80.0, -30.0, -30.0, 5e-4, 0.5, 100.0, 90_000.0, 3000.0);
        assert!(mild > cold, "same heater power must hold a milder environment warmer: mild {mild} cold {cold}");
    }

    #[test]
    fn collection_efficiency_is_zero_below_the_inertia_threshold_and_rises_with_droplet_size() {
        let mu = air_dynamic_viscosity_pa_s(-20.0);
        let small_k = droplet_inertia_parameter(5e-6, 100.0, 2.0, mu);
        let big_k = droplet_inertia_parameter(40e-6, 100.0, 2.0, mu);
        assert_eq!(collection_efficiency_beta0(small_k), 0.0, "K={small_k}");
        assert!(collection_efficiency_beta0(big_k) > 0.0, "K={big_k}");
    }

    #[test]
    fn small_bodies_collect_more_efficiently_than_large_bodies_at_the_same_conditions() {
        let mu = air_dynamic_viscosity_pa_s(-20.0);
        let probe_k = droplet_inertia_parameter(20e-6, 100.0, 0.005, mu);
        let wing_k = droplet_inertia_parameter(20e-6, 100.0, 1.0, mu);
        assert!(collection_efficiency_beta0(probe_k) > collection_efficiency_beta0(wing_k));
    }

    #[test]
    fn viscosity_rises_with_temperature() {
        assert!(air_dynamic_viscosity_pa_s(20.0) > air_dynamic_viscosity_pa_s(-40.0));
    }

    #[test]
    fn bleed_equilibrium_never_exceeds_the_supply_temperature() {
        let t = equilibrium_surface_c_with_bleed(1.0, -10.0, -10.0, 0.0, 0.0, 50.0, 90_000.0, 1_000_000.0, 200.0);
        assert!(t <= 200.0 + 1e-6, "{t}");
    }

    #[test]
    fn bleed_equilibrium_rises_with_a_bigger_bleed_slope() {
        let weak = equilibrium_surface_c_with_bleed(120.0, -10.0, -10.0, 0.0, 0.0, 100.0, 90_000.0, 20.0, 200.0);
        let strong = equilibrium_surface_c_with_bleed(120.0, -10.0, -10.0, 0.0, 0.0, 100.0, 90_000.0, 200.0, 200.0);
        assert!(strong > weak, "weak {weak} strong {strong}");
    }

    #[test]
    fn relax_toward_equilibrium_holds_at_zero_dt_and_converges_over_many_time_constants() {
        assert_eq!(relax_toward_equilibrium_c(10.0, 50.0, 20.0, 0.0), 10.0);
        let nearly_there = relax_toward_equilibrium_c(10.0, 50.0, 20.0, 200.0);
        assert!((nearly_there - 50.0).abs() < 0.01, "{nearly_there}");
    }

    #[test]
    fn relax_toward_equilibrium_is_halfway_after_one_half_life() {
        let half_life = 20.0 * 2f64.ln();
        let t = relax_toward_equilibrium_c(0.0, 100.0, 20.0, half_life);
        assert!((t - 50.0).abs() < 1e-6, "{t}");
    }
}
