//! Shared physical constants and small formulas reused by every model in
//! this directory (fire detection loops, zone combustion, extinguishing,
//! ice accretion and anti-ice). Kept private to `fire_ice` -- per
//! `docs/deep/BRIEF.md` rule 2 this directory must stay self-contained and
//! cannot import `crate::physics` (other agents' modules, e.g.
//! `physics::bays.rs`, independently re-derive the same public relations
//! for the same reason; nothing here is copied from them, only the same
//! textbook physics restated).
//!
//! Sources for the constants below: CODATA 2018 (universal gas constant),
//! ISA (air molar mass), standard engineering-thermodynamics references
//! (air/water/ice properties) -- the same figures `physics::bays.rs`,
//! `physics::engine::oil.rs` and `sensors::pitot.rs` already cite for
//! identical quantities.

/// Ratio of specific heats for air (diatomic ideal gas), standard.
pub const GAMMA_AIR: f64 = 1.4;
/// Specific gas constant for dry air, J/(kg K): R_universal (8.314462618
/// J/(mol K), CODATA 2018) / molar mass of dry air (0.0289647 kg/mol, ISA).
pub const R_AIR: f64 = 8.314462618 / 0.0289647;
/// Dry air specific heat at constant pressure, J/(kg K) (standard,
/// sea-level moderate-temperature range).
pub const CP_AIR: f64 = 1005.0;

/// Water/ice properties (standard thermodynamic tables).
pub const LATENT_HEAT_FUSION_WATER_J_KG: f64 = 334_000.0;
pub const LATENT_HEAT_VAPORIZATION_WATER_J_KG: f64 = 2_501_000.0;
pub const CP_WATER_J_KG_K: f64 = 4186.0;
pub const CP_ICE_J_KG_K: f64 = 2100.0;
pub const DENSITY_ICE_KG_M3: f64 = 917.0;
pub const DENSITY_WATER_KG_M3: f64 = 1000.0;

pub const STEFAN_BOLTZMANN: f64 = 5.670_374e-8;
pub const PSI_TO_PA: f64 = 6894.757;
pub const KT_TO_M_S: f64 = 0.514444;

/// Standard isentropic-ideal-gas compressible orifice/nozzle mass flow rate
/// (subsonic or choked). The same general relation `physics::bays.rs`'s
/// bleed-duct-leak model and `physics::air.md`'s cabin outflow valve use;
/// re-derived independently here (this directory cannot depend on
/// `crate::physics`, BRIEF rule 2).
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

/// Saturation vapor pressure of water over a liquid surface, Pa (Tetens'
/// formula, a standard public meteorological approximation valid roughly
/// -40..50 C to within about 1%: Tetens, O. (1930), Z. Geophys. 6, 297-309;
/// reproduced in any atmospheric-science reference, e.g. Murray, F.W.
/// (1967), J. Appl. Meteorol. 6, 203-204).
pub fn saturation_vapor_pressure_pa(temp_c: f64) -> f64 {
    610.78 * (17.27 * temp_c / (temp_c + 237.3)).exp()
}

/// Never-NaN clamp to `0.0..=1.0` for a fault/severity fraction.
pub fn clamp01(x: f64) -> f64 {
    if x.is_nan() {
        0.0
    } else {
        x.clamp(0.0, 1.0)
    }
}

/// Adiabatic recovery (stagnation-adjacent) temperature a surface in an
/// airstream tends toward from aerodynamic/compressive heating, deg C.
/// Standard compressible-flow relation: `Tr = Ts*(1 + r*(gamma-1)/2*M^2)`
/// (Anderson, J.D., *Fundamentals of Aerodynamics*; recovery factor `r`
/// per the standard rule of thumb `sqrt(Pr)` for laminar / `Pr^(1/3)` for
/// turbulent boundary layers, both close to 0.9 for air -- the same figure
/// this project's icing-relevant modules use). This is the real mechanism
/// behind the published "high-speed/high-Mach flight does not ice" idea
/// (FAA AC 25-1419, EASA CS-25 Appendix C icing envelope notes): kinetic/
/// compressive heating raises a surface above 0 C once `M^2 * T_static_K`
/// is large enough, which happens at quite moderate Mach numbers in the
/// relatively mild end of the icing envelope (e.g. -10 C, where Mach
/// ~0.7-0.8 is already enough) but needs a much higher Mach number at the
/// cold end (e.g. -50 C static, typical of high-altitude cruise) --
/// consistent with real high-altitude cruise still requiring anti-ice
/// systems to be available, icing there being avoided mainly by the
/// near-zero liquid water content at those altitudes/temperatures, not by
/// this heating effect alone.
pub fn recovery_temperature_c(static_air_c: f64, tas_m_s: f64, recovery_factor: f64) -> f64 {
    let t_static_k = (static_air_c + 273.15).max(1.0);
    let sonic_m_s = (GAMMA_AIR * R_AIR * t_static_k).sqrt();
    let mach = (tas_m_s.max(0.0) / sonic_m_s).min(3.0);
    let t_recovery_k = t_static_k * (1.0 + recovery_factor * (GAMMA_AIR - 1.0) / 2.0 * mach * mach);
    t_recovery_k - 273.15
}

/// Net heat, W/m^2, a surface held at `surface_c` must itself supply
/// (positive) or would shed (negative) to stay at that temperature under
/// convective and evaporative exchange with the recovery-temperature
/// boundary layer plus the sensible/kinetic effect of impinging
/// supercooled water -- the Messinger (1953) surface energy balance
/// (Messinger, "Equilibrium Temperature of an Unheated Icing Surface as a
/// Function of Air Speed", J. Aeronautical Sciences 20(1), 29-42), the
/// same published method `sensors::pitot.rs` cites for its own probe heat
/// balance, re-derived independently here (BRIEF rule 2, no cross-module
/// dependency). Terms, per unit area:
/// - convective exchange with the recovery temperature (kinetic/aero
///   heating already folded into `recovery_c` by the caller via
///   [`recovery_temperature_c`]);
/// - evaporative loss via the Lewis-relation mass-transfer analogy
///   (`h_m = h/cp_air`, Lewis number ~= 1 for water vapour in air, the
///   standard psychrometric approximation), against ambient assumed
///   saturated (icing occurs in visible cloud moisture, i.e. ~100% RH by
///   definition of the icing condition);
/// - sensible heat needed to warm impinging water from the static air
///   temperature up to the surface temperature before it can freeze or
///   run off;
/// - kinetic heating the droplets deliver to the surface on impact
///   (subtracted: it is a gain, not a loss).
///
/// Returns `(net_loss_w_m2, impingement_kg_m2_s)`.
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

    // Evaporation can only cool the surface if there is actually a liquid
    // water film on it to evaporate from -- with no impingement (dry air,
    // e.g. anti-ice running with no icing/rain present), there is nothing
    // to evaporate regardless of how warm or dry the surface is, so this
    // term must be gated on impingement being present, not applied
    // unconditionally from ambient humidity alone.
    let q_evap = if impingement > 0.0 {
        let h_m = h_w_m2k.max(0.0) / CP_AIR;
        let e_surf = saturation_vapor_pressure_pa(surface_c);
        let e_air = saturation_vapor_pressure_pa(static_air_c); // cloud assumed saturated
        let p = ambient_pressure_pa.max(1000.0);
        h_m * LATENT_HEAT_VAPORIZATION_WATER_J_KG * 0.622 / p * (e_surf - e_air)
    } else {
        0.0
    };

    let q_sensible_water = impingement * CP_WATER_J_KG_K * (surface_c - static_air_c);

    let q_kinetic = impingement * 0.5 * tas_m_s.max(0.0).powi(2);

    (q_conv + q_evap + q_sensible_water - q_kinetic, impingement)
}

/// The classical Messinger freezing-fraction result: how much of the
/// impinging water freezes at the surface (evaluated at the standard 0 C
/// wet-ice assumption) versus running off as liquid, and the heat an
/// anti-ice system would need to supply to hold the surface exactly at the
/// freezing point (0 freezing fraction, "running wet" anti-ice).
#[derive(Clone, Copy, Debug, Default)]
pub struct MessingerResult {
    /// Impingement (water catch) mass flux reaching the surface, kg/(m^2 s).
    pub impingement_kg_m2_s: f64,
    /// Fraction of the impinging water that freezes on contact: 0 (all
    /// liquid, glaze/runback) .. 1 (all freezes, dry rime regime).
    pub freezing_fraction: f64,
    /// Heat an anti-ice system would need to add, W/m^2, to hold the
    /// surface at 0 C and stop any freezing (0 when the surface would not
    /// ice at all).
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

/// The equilibrium surface temperature, deg C, a heater supplying
/// `heater_w_m2` (net of losses already accounted by
/// [`surface_net_loss_w_m2`]) settles to under the given environment.
/// `surface_net_loss_w_m2` is monotonically increasing in `surface_c` (a
/// hotter surface always convects, evaporates and warms incoming water
/// more, and kinetic heating does not depend on surface temperature), so
/// bisection is well posed.
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

/// The self-consistent equilibrium surface temperature, deg C, when a
/// bleed-air (or any similarly source-temperature-limited) heater
/// delivers heat proportional to the *current* temperature gap to its
/// supply temperature (`bleed_slope_w_m2k * (supply_c - T).max(0)`, the
/// fixed `effectiveness*mdot*cp/area` slope of a simple heat exchanger)
/// rather than a temperature-independent fixed power. Both sides of the
/// balance are monotonic in `T` (surface losses rise with `T`; heat
/// input falls as `T` approaches the supply temperature and cannot delivr
/// heat once `T` reaches it), so their difference is monotonically
/// increasing and the intersection is unique and found directly here by
/// bisection over the *combined* balance, rather than by iterating the
/// heater estimate tick to tick against [`equilibrium_surface_c_with_
/// heater`] (which, for a large enough `bleed_slope_w_m2k` relative to
/// `h_w_m2k`, would not converge). The result can never exceed
/// `bleed_supply_c` (heat input is exactly zero there).
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

/// Relax a surface's own displayed temperature toward an equilibrium
/// (computed fresh this tick from the current heater/environment state)
/// with the surface's thermal time constant `tau_s` -- the exact
/// exponential first-order-lag step this crate's conventions call for
/// (`docs/deep/BRIEF.md` conventions), so a bang-bang (on/off) heater
/// controller cannot swing the modelled surface instantly between two
/// extreme equilibria every tick the way a zero-thermal-mass surface
/// would; any real object's own mass smooths that out.
pub fn relax_toward_equilibrium_c(current_c: f64, equilibrium_c: f64, tau_s: f64, dt_s: f64) -> f64 {
    if tau_s <= 0.0 {
        return equilibrium_c;
    }
    equilibrium_c + (current_c - equilibrium_c) * (-dt_s.max(0.0) / tau_s).exp()
}

/// Water-droplet inertia (non-dimensional Stokes) parameter `K` and the
/// simplified engineering approximation of local collection efficiency
/// `beta0` it maps to. **GENERIC/derived**: the exact curve depends on body
/// shape and is normally produced by a numerical trajectory code (e.g.
/// NASA LEWICE); no such code or proprietary curve is used here. This uses
/// the widely reproduced simplified closed-form relation from the
/// Langmuir & Blodgett (1946) inertia-parameter analysis (US Army Air
/// Forces Tech. Report 5418; summarised in Gent, R.W., Dart, N.P. &
/// Cansdale, J.T., "Aircraft Icing", Phil. Trans. R. Soc. Lond. A 358
/// (2000), 2873-2911, eq. for `beta0` vs `K`), which is public and
/// standard in icing-engineering teaching material. It captures the
/// correct, well-documented qualitative behaviour this model needs: small
/// bodies (probes) collect efficiently even in low-K conditions; large
/// bodies (wings) need larger droplets/higher speed to collect well; and
/// there is a inertia threshold below which a body collects essentially no
/// water at all.
pub fn droplet_inertia_parameter(
    droplet_diameter_m: f64,
    tas_m_s: f64,
    characteristic_length_m: f64,
    dynamic_viscosity_air_pa_s: f64,
) -> f64 {
    (DENSITY_WATER_KG_M3 * droplet_diameter_m.powi(2) * tas_m_s.max(0.0))
        / (18.0 * dynamic_viscosity_air_pa_s.max(1e-9) * characteristic_length_m.max(1e-6))
}

/// Local collection efficiency from the inertia parameter, 0..1.
pub fn collection_efficiency_beta0(inertia_k: f64) -> f64 {
    clamp01(1.40 * (inertia_k - 0.125))
}

/// Sutherland's law for dry air dynamic viscosity, Pa s (standard;
/// Sutherland, W. (1893), Phil. Mag. 5, 36, 507-531; reference viscosity
/// 1.716e-5 Pa s at 273.15 K, Sutherland constant 110.4 K -- the
/// textbook-standard coefficients reproduced in any gas-dynamics
/// reference, e.g. White, F.M., *Viscous Fluid Flow*).
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
        // At 0 C, saturation vapor pressure of water is about 611 Pa
        // (standard reference value).
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
        // -10 C static air at a high subsonic TAS (Mach ~0.77): kinetic/
        // compressive heating alone recovers to well above 0 C, matching
        // the published "high Mach/high-speed flight does not ice" idea --
        // note this needs a much higher Mach number at colder static
        // temperatures (e.g. typical cruise near the tropopause, ISA
        // -56.5 C) to cross freezing at all; it is not a universal
        // boundary independent of temperature.
        let recovery = recovery_temperature_c(-10.0, 250.0, 0.9);
        assert!(recovery > 0.0, "recovery {recovery}");
    }

    #[test]
    fn messinger_gives_full_freezing_in_cold_still_air_and_no_freezing_when_hot() {
        let cold = messinger_freezing_fraction(80.0, -20.0, -20.0, 5e-4, 0.5, 100.0, 80_000.0);
        assert!(cold.freezing_fraction > 0.9, "{:?}", cold);
        assert!(cold.impingement_kg_m2_s > 0.0);

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
        let small_k = droplet_inertia_parameter(5e-6, 100.0, 2.0, mu); // small droplet, big body (wing)
        let big_k = droplet_inertia_parameter(40e-6, 100.0, 2.0, mu); // large droplet, same body
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
