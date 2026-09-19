//! Ice crystal icing: core ice accretion on warm compressor-front
//! surfaces, periodic shedding, and the resulting engine roll-back/
//! flameout risk.
//!
//! ## Sources
//! - Mechanism: ice crystal icing is a distinct hazard from classic
//!   supercooled-liquid airframe icing. It occurs high in deep convective
//!   clouds where the air is too warm for supercooled liquid water to
//!   exist, so the ice is already fully glaciated (crystals, not
//!   droplets) and does *not* stick to a cold surface on impact. Mason,
//!   Strapp & Chow, *"The Ice Particle Threat to Engines in Flight"*
//!   (AIAA 2006-206), a widely cited foundational paper, identified that
//!   crystals only accrete where a surface sits near 0 C: warm enough for
//!   the impacting crystal to partially melt and adhere as a wet
//!   ice/water mix (subsequent crystals then stick to that mix), but not
//!   so warm that it simply washes off as liquid. This is why the hazard
//!   appears deep in the engine core (fan-exit guide vanes, IP compressor
//!   front stages, splitters) rather than on the cold airframe, and why
//!   it causes compressor-stage blockage leading to roll-back or flameout
//!   rather than the classic wing/tail lift-and-control-effectiveness
//!   loss of airframe icing. FAA/EASA rulemaking added a dedicated mixed-
//!   phase/ice-crystal icing envelope (14 CFR/CS-25 Appendix P; Part 33
//!   Appendix D) after this and other engine-power-loss events were
//!   traced to it -- both public.
//! - The `+-4 C` sticking-efficiency window around 0 C, the capture
//!   fraction, accreted-ice density, full-blockage reference mass and the
//!   shedding threshold are all `GENERIC`: they encode Mason's qualitative
//!   finding (a narrow near-0 C adherence window) without claiming a
//!   measured number, since no public Trent-900-specific ice-crystal test
//!   report exists.

/// Inputs the weather/engine-thermal side provides each step.
#[derive(Clone, Copy, Debug)]
pub struct IceCrystalInputs {
    /// Ice water content of the core airflow, g/m^3 (a weather-model
    /// input; deep convective cores can exceed several g/m^3).
    pub ice_water_content_g_m3: f64,
    pub core_mass_flow_kg_s: f64,
    /// Temperature of the compressor-front surfaces ice can stick to, C
    /// (an engine-thermal-model input: warmed by compression from the
    /// ambient total air temperature, this is what actually decides
    /// adherence, not ambient temperature directly).
    pub warm_surface_temp_c: f64,
    pub dt_s: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct IceCrystalOutputs {
    pub accreted_mass_kg: f64,
    /// 0 clear .. 1 stage fully blocked.
    pub flow_capacity_loss_frac: f64,
    /// Reduced corrected flow pushes the compressor operating point
    /// toward stall/surge, which the FADEC (or the compressor itself)
    /// answers by rolling the core back.
    pub rollback_risk_frac: f64,
    pub flameout_risk_frac: f64,
    /// True the step a chunk of accreted ice broke off under its own
    /// aerodynamic/centrifugal load (documented as a sharp bang/transient
    /// vibration in real ice-crystal-icing events, sometimes followed by
    /// a brief further core disturbance as it passes through).
    pub shedding_event: bool,
}

/// GENERIC: accreted ice density, kg/m^3 (rime/mixed ice with entrained
/// air is well below solid ice's 917 kg/m^3; a mid-range order-of-
/// magnitude figure from icing literature).
const ICE_DENSITY_KG_M3: f64 = 700.0;
/// GENERIC: fraction of the core ice water content that impinges on and
/// is captured by the relevant stator/vane geometry.
const CAPTURE_FRACTION: f64 = 0.02;
/// GENERIC: half-width, C, of the near-0 C adherence window (Mason et al.,
/// see module doc).
const STICKING_WINDOW_C: f64 = 4.0;
/// GENERIC: accreted mass, kg, at which the local blockage reaches 100%
/// flow-capacity loss.
const FULL_BLOCKAGE_MASS_KG: f64 = 0.5;
/// GENERIC: flow-capacity loss fraction at which the accreted ice sheds.
const SHED_THRESHOLD_FRAC: f64 = 0.6;
/// GENERIC: fraction of accreted mass left behind after a shedding event
/// (it rarely sheds perfectly clean).
const SHED_RETAINED_FRACTION: f64 = 0.1;

/// GENERIC parabolic window peaking at 1.0 at 0 C and reaching 0 at
/// +-`STICKING_WINDOW_C`: too cold and the crystal bounces off dry, too
/// warm and it melts fully and washes away; only the narrow near-freezing
/// band lets it adhere as a wet mix (Mason et al., see module doc).
fn sticking_efficiency(surface_temp_c: f64) -> f64 {
    let x = surface_temp_c / STICKING_WINDOW_C;
    (1.0 - x * x).max(0.0)
}

#[derive(Clone, Copy, Debug, Default)]
pub struct IceCrystalIcingState {
    accreted_mass_kg: f64,
}

impl IceCrystalIcingState {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn step(&mut self, i: &IceCrystalInputs) -> IceCrystalOutputs {
        let dt = i.dt_s.max(0.0);
        let iwc_kg_m3 = i.ice_water_content_g_m3.max(0.0) * 1e-3;
        let ice_mass_flow_kg_s = iwc_kg_m3 * i.core_mass_flow_kg_s.max(0.0);
        let efficiency = sticking_efficiency(i.warm_surface_temp_c);
        let accrete_rate_kg_s = ice_mass_flow_kg_s * CAPTURE_FRACTION * efficiency;
        self.accreted_mass_kg += accrete_rate_kg_s * dt;

        let mut flow_capacity_loss_frac = (self.accreted_mass_kg / FULL_BLOCKAGE_MASS_KG).min(1.0);
        let mut shedding_event = false;
        if flow_capacity_loss_frac >= SHED_THRESHOLD_FRAC {
            self.accreted_mass_kg *= SHED_RETAINED_FRACTION;
            flow_capacity_loss_frac = (self.accreted_mass_kg / FULL_BLOCKAGE_MASS_KG).min(1.0);
            shedding_event = true;
        }

        // GENERIC: roll-back risk grows faster than linear as the stall
        // margin the blockage eats into shrinks (a compressor's surge
        // line gets closer, not further, as more of it is choked).
        let rollback_risk_frac = flow_capacity_loss_frac.powi(2);
        // A shedding event throws a slug of ice into the combustor, a
        // documented momentary flame-stability threat on top of whatever
        // the blockage itself was already doing.
        let flameout_risk_frac = (rollback_risk_frac + if shedding_event { 0.4 } else { 0.0 }).min(1.0);

        IceCrystalOutputs { accreted_mass_kg: self.accreted_mass_kg, flow_capacity_loss_frac, rollback_risk_frac, flameout_risk_frac, shedding_event }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn inputs(iwc_g_m3: f64, surface_temp_c: f64, dt_s: f64) -> IceCrystalInputs {
        IceCrystalInputs { ice_water_content_g_m3: iwc_g_m3, core_mass_flow_kg_s: 15.0, warm_surface_temp_c: surface_temp_c, dt_s }
    }

    #[test]
    fn no_ice_water_or_zero_dt_causes_no_accretion_and_no_nan() {
        let mut s = IceCrystalIcingState::new();
        let out = s.step(&inputs(0.0, 0.0, 10.0));
        assert_eq!(out.accreted_mass_kg, 0.0);
        assert_eq!(out.flow_capacity_loss_frac, 0.0);
        assert!(!out.rollback_risk_frac.is_nan());
        let out2 = s.step(&inputs(5.0, 0.0, 0.0));
        assert_eq!(out2.accreted_mass_kg, 0.0);
    }

    #[test]
    fn sticking_efficiency_peaks_at_zero_and_vanishes_outside_the_window() {
        assert!((sticking_efficiency(0.0) - 1.0).abs() < 1e-9);
        assert_eq!(sticking_efficiency(-20.0), 0.0);
        assert_eq!(sticking_efficiency(20.0), 0.0);
        assert!(sticking_efficiency(0.0) > sticking_efficiency(2.0));
    }

    #[test]
    fn cold_dry_or_hot_wet_surfaces_accrete_nothing_despite_high_ice_water_content() {
        let mut cold = IceCrystalIcingState::new();
        let cold_out = cold.step(&inputs(6.0, -30.0, 120.0));
        assert_eq!(cold_out.accreted_mass_kg, 0.0);

        let mut hot = IceCrystalIcingState::new();
        let hot_out = hot.step(&inputs(6.0, 40.0, 120.0));
        assert_eq!(hot_out.accreted_mass_kg, 0.0);
    }

    #[test]
    fn a_near_freezing_surface_accretes_ice_and_raises_rollback_risk() {
        let mut s = IceCrystalIcingState::new();
        let a = s.step(&inputs(6.0, 0.0, 60.0));
        let b = s.step(&inputs(6.0, 0.0, 60.0));
        assert!(a.accreted_mass_kg > 0.0);
        assert!(b.accreted_mass_kg > a.accreted_mass_kg || b.shedding_event);
        assert!(b.rollback_risk_frac > 0.0);
    }

    #[test]
    fn accretion_sheds_once_the_threshold_is_crossed_and_the_mass_drops() {
        let mut s = IceCrystalIcingState::new();
        let mut shed_seen = false;
        let mut last_mass = 0.0;
        for _ in 0..200 {
            let out = s.step(&inputs(8.0, 0.0, 30.0));
            if out.shedding_event {
                shed_seen = true;
                assert!(out.accreted_mass_kg < last_mass, "mass should drop on shedding");
                assert!(out.flameout_risk_frac >= 0.4);
                break;
            }
            last_mass = out.accreted_mass_kg;
        }
        assert!(shed_seen, "expected at least one shedding event over a sustained encounter");
    }

    #[test]
    fn flow_capacity_loss_never_exceeds_one() {
        let mut s = IceCrystalIcingState::new();
        let mut out = s.step(&inputs(8.0, 0.0, 1.0));
        for _ in 0..1000 {
            out = s.step(&inputs(8.0, 0.0, 30.0));
            assert!(out.flow_capacity_loss_frac <= 1.0);
        }
    }
}
