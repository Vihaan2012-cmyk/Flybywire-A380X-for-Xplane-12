//! Ice accretion: water catch efficiency and Messinger freezing fraction on
//! the wing leading edge, nacelle inlets, probes and the windshield, as a
//! function of liquid water content (LWC), droplet size (MVD), static air
//! temperature and airspeed -- ice mass/thickness, and an aerodynamic
//! penalty for lifting surfaces.
//!
//! The energy balance itself ([`super::util::messinger_freezing_fraction`],
//! [`super::util::recovery_temperature_c`]) and the droplet collection
//! efficiency ([`super::util::droplet_inertia_parameter`],
//! [`super::util::collection_efficiency_beta0`]) are shared with
//! `anti_ice.rs`, which needs the identical physics to compute how much
//! heat a de-ice system must supply to stop exactly this same accretion.
//!
//! ## Why small bodies ice worse than the wing
//! A probe or antenna's leading-edge radius (millimetres) is far smaller
//! than the wing's (tens of centimetres): the droplet inertia parameter
//! `K` scales inversely with that characteristic length, so small bodies
//! collect efficiently at droplet sizes/speeds where a wing collects
//! almost nothing -- the real, well documented reason unheated probes ice
//! first and standard icing certification envelopes (EASA CS-25 Appendix
//! C) are defined by droplet size distributions, not by a single number.
//!
//! ## Sources
//! Cited per-function in `util.rs`. Aerodynamic-penalty coefficients below
//! are **GENERIC**: no public per-aircraft lift/drag-vs-ice-thickness curve
//! exists for the A380; the qualitative figures used (a few tenths of a
//! millimetre of leading-edge roughness measurably degrading Cl_max, and a
//! few millimetres causing a large penalty) are the commonly cited public
//! order-of-magnitude figures from FAA/NASA icing safety education
//! material (e.g. FAA *Aircraft Icing* safety pamphlets; NASA Glenn Icing
//! Branch public outreach material), not a specific certified curve.

use super::util::{
    air_dynamic_viscosity_pa_s, clamp01, collection_efficiency_beta0, droplet_inertia_parameter, messinger_freezing_fraction, recovery_temperature_c,
    DENSITY_ICE_KG_M3,
};

/// Atmospheric icing environment this tick (shared by every surface).
#[derive(Clone, Copy, Debug)]
pub struct IcingEnvironment {
    /// Liquid water content, kg/m^3 (typical continuous-maximum icing
    /// envelopes: ~0.0002-0.0008 kg/m^3, EASA CS-25 Appendix C -- passed in
    /// directly rather than hard-coded so callers can drive it from
    /// whatever weather/cloud model the plugin uses).
    pub lwc_kg_m3: f64,
    /// Median volumetric droplet diameter, m (typical continuous-maximum
    /// envelope: 15-40 micron).
    pub droplet_diameter_m: f64,
    pub static_air_c: f64,
    pub tas_m_s: f64,
    pub ambient_pressure_pa: f64,
}

/// A surface's fixed geometry for collection efficiency and aerodynamic
/// penalty purposes.
#[derive(Clone, Copy, Debug)]
pub struct SurfaceGeometry {
    /// Characteristic (leading-edge) length for the inertia parameter, m.
    pub characteristic_length_m: f64,
    pub area_m2: f64,
    /// Convective heat transfer coefficient, W/(m^2 K) (a fixed
    /// representative value per surface here; a fuller model would derive
    /// this from local Reynolds number, kept simple per this module's
    /// scope).
    pub h_w_m2k: f64,
    /// Chord (wing) or reference length (nacelle) used only for the
    /// aerodynamic-penalty correlation, m. `None` for probes/windshield,
    /// which have no lift-penalty interpretation.
    pub aero_reference_length_m: Option<f64>,
}

/// GENERIC representative surfaces (module doc for the size sourcing
/// philosophy): wing leading-edge radius, nacelle lip radius and typical
/// pitot-probe radius are public order-of-magnitude figures for
/// large-transport hardware, not A380-specific measurements.
pub const WING_LEADING_EDGE: SurfaceGeometry =
    SurfaceGeometry { characteristic_length_m: 0.15, area_m2: 2.0, h_w_m2k: 120.0, aero_reference_length_m: Some(6.0) };
pub const NACELLE_INLET: SurfaceGeometry =
    SurfaceGeometry { characteristic_length_m: 0.05, area_m2: 1.2, h_w_m2k: 150.0, aero_reference_length_m: Some(1.2) };
pub const PROBE: SurfaceGeometry = SurfaceGeometry { characteristic_length_m: 0.005, area_m2: 0.001, h_w_m2k: 200.0, aero_reference_length_m: None };
pub const WINDSHIELD: SurfaceGeometry = SurfaceGeometry { characteristic_length_m: 0.3, area_m2: 1.5, h_w_m2k: 80.0, aero_reference_length_m: None };

/// Recovery factor for a turbulent boundary layer (module doc in
/// `util.rs`, `Pr^(1/3)` for air is close to 0.9).
const RECOVERY_FACTOR: f64 = 0.9;

#[derive(Clone, Copy, Debug, Default)]
pub struct IcingOutputs {
    pub impingement_kg_m2_s: f64,
    pub freezing_fraction: f64,
    pub ice_mass_kg: f64,
    pub ice_thickness_m: f64,
    /// Fraction of maximum lift coefficient lost to accreted ice, 0..1
    /// (only meaningful when the surface has an `aero_reference_length_m`).
    pub cl_max_loss_fraction: f64,
    /// Fractional increase in profile drag from accreted ice (only
    /// meaningful with `aero_reference_length_m`).
    pub cd_increase_fraction: f64,
}

pub struct IcingSurface {
    geometry: SurfaceGeometry,
    ice_mass_kg: f64,
}

impl IcingSurface {
    pub fn new(geometry: SurfaceGeometry) -> Self {
        Self { geometry, ice_mass_kg: 0.0 }
    }

    pub fn ice_mass_kg(&self) -> f64 {
        self.ice_mass_kg
    }

    /// One tick of natural (unheated) icing. `deicing_removal_kg_m2_s` is
    /// any active removal (anti-ice runback, mechanical) already computed
    /// elsewhere; pass 0 for a fully passive surface.
    pub fn step(&mut self, env: &IcingEnvironment, deicing_removal_kg_m2_s: f64, dt_s: f64) -> IcingOutputs {
        let dt = dt_s.max(0.0);
        let mu = air_dynamic_viscosity_pa_s(env.static_air_c);
        let k = droplet_inertia_parameter(env.droplet_diameter_m, env.tas_m_s, self.geometry.characteristic_length_m, mu);
        let beta0 = collection_efficiency_beta0(k);
        let recovery_c = recovery_temperature_c(env.static_air_c, env.tas_m_s, RECOVERY_FACTOR);

        let result = messinger_freezing_fraction(self.geometry.h_w_m2k, env.static_air_c, recovery_c, env.lwc_kg_m3, beta0, env.tas_m_s, env.ambient_pressure_pa);

        let accretion_kg_m2_s = (result.impingement_kg_m2_s * result.freezing_fraction - deicing_removal_kg_m2_s.max(0.0)).max(-self.ice_mass_kg / self.geometry.area_m2.max(1e-9) / dt.max(1e-9));
        self.ice_mass_kg = (self.ice_mass_kg + accretion_kg_m2_s * self.geometry.area_m2 * dt).max(0.0);

        let thickness_m = self.ice_mass_kg / (DENSITY_ICE_KG_M3 * self.geometry.area_m2.max(1e-9));

        let (cl_loss, cd_gain) = match self.geometry.aero_reference_length_m {
            Some(chord_m) => aerodynamic_penalty(thickness_m, chord_m),
            None => (0.0, 0.0),
        };

        IcingOutputs {
            impingement_kg_m2_s: result.impingement_kg_m2_s,
            freezing_fraction: result.freezing_fraction,
            ice_mass_kg: self.ice_mass_kg,
            ice_thickness_m: thickness_m,
            cl_max_loss_fraction: cl_loss,
            cd_increase_fraction: cd_gain,
        }
    }
}

/// **GENERIC** (module doc): ice roughness on a leading edge degrades
/// Cl_max and raises Cd roughly in proportion to ice thickness normalized
/// by chord, with Cl_max the more sensitive of the two (public FAA/NASA
/// icing-safety material commonly cites a several-tenths-of-a-mm
/// roughness already producing a double-digit-percent Cl_max loss, with
/// the effect flattening out well before "all lift gone" -- captured here
/// as a saturating relation, not literal certification data).
fn aerodynamic_penalty(thickness_m: f64, chord_m: f64) -> (f64, f64) {
    let ratio = (thickness_m / chord_m.max(1e-6)).max(0.0);
    let cl_loss = clamp01(1.0 - (-400.0 * ratio).exp());
    let cd_gain = (150.0 * ratio).min(3.0);
    (cl_loss, cd_gain)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn continuous_max_icing() -> IcingEnvironment {
        // A representative EASA CS-25 Appendix C continuous-maximum icing
        // point: -10 C, moderate LWC/MVD, holding pattern speed.
        IcingEnvironment { lwc_kg_m3: 5e-4, droplet_diameter_m: 20e-6, static_air_c: -10.0, tas_m_s: 100.0, ambient_pressure_pa: 80_000.0 }
    }

    #[test]
    fn a_wing_leading_edge_accretes_ice_in_icing_conditions() {
        let mut surface = IcingSurface::new(WING_LEADING_EDGE);
        let mut out = IcingOutputs::default();
        for _ in 0..1800 {
            out = surface.step(&continuous_max_icing(), 0.0, 1.0);
        }
        assert!(out.ice_mass_kg > 0.0, "should accrete measurable ice over 30 minutes in CS-25 Appendix C continuous-max conditions");
        assert!(out.ice_thickness_m > 0.0);
    }

    #[test]
    fn no_ice_accretes_above_freezing() {
        let mut surface = IcingSurface::new(WING_LEADING_EDGE);
        let warm = IcingEnvironment { static_air_c: 15.0, ..continuous_max_icing() };
        let out = surface.step(&warm, 0.0, 1.0);
        assert_eq!(out.ice_mass_kg, 0.0);
        assert_eq!(out.freezing_fraction, 0.0);
    }

    #[test]
    fn probes_collect_water_and_ice_faster_per_unit_area_than_the_wing() {
        let mut probe = IcingSurface::new(PROBE);
        let mut wing = IcingSurface::new(WING_LEADING_EDGE);
        let env = IcingEnvironment { droplet_diameter_m: 15e-6, ..continuous_max_icing() }; // smaller MVD: wing barely collects
        let probe_out = probe.step(&env, 0.0, 1.0);
        let wing_out = wing.step(&env, 0.0, 1.0);
        assert!(probe_out.impingement_kg_m2_s > wing_out.impingement_kg_m2_s, "probe {} vs wing {}", probe_out.impingement_kg_m2_s, wing_out.impingement_kg_m2_s);
    }

    #[test]
    fn high_speed_flight_can_prevent_icing_even_in_a_supercooled_cloud() {
        // -10 C at a high subsonic TAS (Mach ~0.77): recovery/kinetic
        // heating alone (see `util.rs`'s `recovery_temperature_c` doc)
        // pushes the surface energy balance to fully liquid, unlike the
        // same LWC/MVD at a low-speed condition (`continuous_max_icing`,
        // TAS 100 m/s), which does freeze (first test in this module).
        let mut surface = IcingSurface::new(WING_LEADING_EDGE);
        let high_speed = IcingEnvironment { tas_m_s: 250.0, static_air_c: -10.0, ..continuous_max_icing() };
        let out = surface.step(&high_speed, 0.0, 1.0);
        assert_eq!(out.freezing_fraction, 0.0, "kinetic/recovery heating at high TAS should prevent freezing even in a supercooled cloud");
    }

    #[test]
    fn thicker_ice_costs_more_lift_and_drag() {
        let (thin_cl, thin_cd) = aerodynamic_penalty(0.001, 6.0);
        let (thick_cl, thick_cd) = aerodynamic_penalty(0.02, 6.0);
        assert!(thick_cl > thin_cl);
        assert!(thick_cd > thin_cd);
    }

    #[test]
    fn probes_and_windshields_report_zero_aerodynamic_penalty() {
        let mut probe = IcingSurface::new(PROBE);
        let out = probe.step(&continuous_max_icing(), 0.0, 1.0);
        assert_eq!(out.cl_max_loss_fraction, 0.0);
        assert_eq!(out.cd_increase_fraction, 0.0);
    }

    #[test]
    fn active_removal_can_hold_ice_mass_at_zero() {
        let mut surface = IcingSurface::new(WING_LEADING_EDGE);
        let env = continuous_max_icing();
        // Remove at least as fast as it could ever accrete per unit area.
        let out = surface.step(&env, 1.0, 1.0);
        assert_eq!(out.ice_mass_kg, 0.0);
    }

    #[test]
    fn no_nan_at_rest_or_zero_dt() {
        let mut surface = IcingSurface::new(WINDSHIELD);
        let out = surface.step(&continuous_max_icing(), 0.0, 0.0);
        assert!(!out.ice_mass_kg.is_nan());
        assert!(!out.ice_thickness_m.is_nan());
        let out2 = surface.step(&IcingEnvironment { lwc_kg_m3: 0.0, droplet_diameter_m: 0.0, static_air_c: 20.0, tas_m_s: 0.0, ambient_pressure_pa: 101_325.0 }, 0.0, 1.0);
        assert!(!out2.ice_mass_kg.is_nan());
    }
}
