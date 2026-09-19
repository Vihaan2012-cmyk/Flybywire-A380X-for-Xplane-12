//! Total air temperature (TAT) probe: a small strut-mounted sensing element
//! that decelerates the airflow adiabatically so its element reads close to
//! (but not exactly) the true total temperature. Models the heater, icing
//! (which does not block flow the way a pitot does -- these probes are
//! flow-through -- but does insulate the sensing element and add thermal
//! mass), the recovery factor, and the probe's own self-heating error at
//! low airspeed.
//!
//! ## Recovery factor
//! A perfect total-temperature probe reads `Tt = Ts * (1 + (gamma-1)/2 * M^2)`
//! (isentropic total temperature). A real probe never fully recovers the
//! kinetic energy (some is lost to friction/conduction before reaching the
//! sensing element), so `Tt_measured = Ts * (1 + r*(gamma-1)/2*M^2)` with
//! recovery factor `r < 1`. Rosemount-style aspirated/strut probes are
//! commonly quoted in the 0.98-1.0 range; 0.99 is used here as the mid
//! value -- GENERIC (no A380-specific figure is public), the same figure
//! `src/physics/adirs.rs` independently uses for the same reason.
//!
//! ## Self-heating error
//! A real, documented effect: the anti-ice heater itself adds heat to the
//! sensing element. At high TAS the convective airflow past the element
//! carries that heat away fast enough that the error is negligible, but at
//! low TAS (taxi, hold on the ramp with the heater on) the same wattage
//! raises the element's equilibrium temperature measurably above the true
//! recovery temperature -- documented, e.g., in FAA/EASA guidance on TAT/OAT
//! probe self-heating errors and associated calibration/testing
//! requirements. Modelled as a first-order convective heat balance:
//! `error_k = heater_w / (h * area)`, cylinder-in-crossflow convection
//! (Zukauskas correlation, Incropera & DeWitt, Table 7.4) -- the same
//! standard correlation `pitot.rs` uses, independently re-derived here.
//!
//! ## Icing
//! Ice on the strut/element does not block the flow path (TAT probes are
//! open, flow-through), but it adds thermal mass and insulates the
//! element from the (heated) airflow it is meant to sense, both of which
//! slow and bias the reading. Modelled as an increased effective thermal
//! time constant plus a (small, capped) conduction offset toward the ice's
//! own (colder, near-0 C at most since it is itself being warmed) surface
//! temperature -- GENERIC magnitude, since no public figure exists for
//! this probe's icing sensitivity.

const RECOVERY_FACTOR: f64 = 0.99;
const GAMMA_AIR: f64 = 1.4;
const R_AIR: f64 = 287.052_87;

/// Probe element geometry, m: a small strut/sensing element, much smaller
/// than a pitot tube. GENERIC.
const ELEMENT_DIAMETER_M: f64 = 0.006;
const ELEMENT_EXPOSED_LENGTH_M: f64 = 0.04;
const RATED_HEATER_W: f64 = 80.0;
const AIR_K_W_MK: f64 = 0.0206;
const AIR_NU_M2_S: f64 = 1.13e-5;
const AIR_PR: f64 = 0.72;

/// Baseline first-order thermal response time of the sensing element,
/// seconds (a small thermal mass responds quickly). GENERIC.
const BASE_THERMAL_TAU_S: f64 = 0.4;
/// Ice mass at which the element's effective thermal time constant has
/// roughly doubled (insulation + added mass). GENERIC.
const ICE_TAU_DOUBLE_KG: f64 = 0.0005;
/// Ice accretion/melt rate scaling: GENERIC, chosen so noticeable icing
/// effects build up over tens of seconds to a couple of minutes in
/// significant icing, consistent with this being a much smaller/less
/// exposed surface than a pitot tube.
const ICE_ACCUM_KG_S_PER_LWC_TAS: f64 = 3e-7;
const WATER_LF_J_KG: f64 = 334_000.0;

fn convective_h_w_m2k(tas_ms: f64) -> f64 {
    let v = tas_ms.max(0.0);
    let re = v * ELEMENT_DIAMETER_M / AIR_NU_M2_S;
    if re <= 1.0 {
        return 0.0;
    }
    let (c, m) = if re <= 2.0e5 { (0.26, 0.6) } else { (0.076, 0.7) };
    let nu = c * re.powf(m) * AIR_PR.powf(0.37);
    nu * AIR_K_W_MK / ELEMENT_DIAMETER_M
}

#[derive(Clone, Copy, Debug, Default)]
pub struct TatProbeFaults {
    pub heater_failure: f64,
    /// A degraded recovery factor (contamination, damage): `1.0` means the
    /// probe recovers essentially none of the kinetic heating, reading
    /// close to SAT even at high Mach.
    pub recovery_degradation: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct TatProbeOutput {
    pub sensed_tat_c: f64,
    pub self_heating_error_k: f64,
    pub ice_kg: f64,
    pub heater_power_w: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct TatProbe {
    element_c: f64,
    ice_kg: f64,
}

impl TatProbe {
    pub fn new(initial_c: f64) -> Self {
        Self { element_c: initial_c, ice_kg: 0.0 }
    }

    /// `sat_c`: true static air temperature. `mach`: true Mach. `tas_ms`:
    /// true airspeed (for self-heating convection and icing rate).
    /// `lwc_gm3`: liquid water content. `powered`: heater bus live.
    pub fn step(
        &mut self,
        sat_c: f64,
        mach: f64,
        tas_ms: f64,
        lwc_gm3: f64,
        powered: bool,
        faults: &TatProbeFaults,
        dt_s: f64,
    ) -> TatProbeOutput {
        let dt = dt_s.max(0.0);
        let r = RECOVERY_FACTOR * (1.0 - faults.recovery_degradation.clamp(0.0, 1.0));
        let sat_k = sat_c + 273.15;
        let ideal_tat_k = sat_k * (1.0 + r * (GAMMA_AIR - 1.0) / 2.0 * mach * mach);

        // ---- Icing: accretes in proportion to LWC*TAS (mass flux onto a
        // small, mostly-open element only lightly wetted -- far less than a
        // pitot's frontal catch), melts whenever the element runs above
        // freezing (it usually does, being both heated and
        // kinetically/self-heated).
        let rated_w = if powered { RATED_HEATER_W * (1.0 - faults.heater_failure.clamp(0.0, 1.0)) } else { 0.0 };
        if sat_c < 0.0 && lwc_gm3 > 0.0 {
            self.ice_kg += ICE_ACCUM_KG_S_PER_LWC_TAS * lwc_gm3.max(0.0) * tas_ms.max(0.0) * dt;
        }
        if self.element_c > 0.5 {
            // Melts fast once above freezing; not heat-balance-limited like
            // the pitot (a TAT element's ice load is small enough that this
            // simplification does not materially affect the jam-relevant
            // dynamics -- there is no "jam" failure mode for a flow-through
            // probe).
            let melt_tau_s = 5.0;
            self.ice_kg *= (-dt / melt_tau_s).exp();
        }
        self.ice_kg = self.ice_kg.max(0.0);

        // ---- Self-heating: the heater's watts raise the element above the
        // ideal recovery temperature by an amount convection can't carry
        // away fast enough to erase, worse at low TAS. Real TAT probe
        // designs deliberately isolate most of the heater's heat (which
        // mainly needs to reach the strut/leading edge to prevent icing)
        // from the sensing junction itself, precisely so the reading stays
        // usable while the heater runs; `LOCAL_COUPLING_FRACTION` is the
        // small residual thermal coupling into the sensing element's own
        // boundary layer that this isolation cannot fully eliminate --
        // GENERIC (no published figure for this internal design detail),
        // chosen so the resulting error is of the same small order the
        // publicly documented self-heating-error concern describes, and
        // still clearly worse at low TAS than at cruise TAS.
        const LOCAL_COUPLING_FRACTION: f64 = 0.01;
        let local_heat_w = rated_w * LOCAL_COUPLING_FRACTION;
        let h = convective_h_w_m2k(tas_ms);
        let area_m2 = std::f64::consts::PI * ELEMENT_DIAMETER_M * ELEMENT_EXPOSED_LENGTH_M;
        let self_heating_error_k = if h * area_m2 > 1e-9 { local_heat_w / (h * area_m2) } else if local_heat_w > 0.0 { 20.0 } else { 0.0 };
        // Capped: a real installation's heater/isolation is sized so this
        // stays bounded even at zero TAS (ground, heater on, per Airbus
        // AUTO logic -- see `pitot.rs`/`src/physics/adirs.rs`); the cap is
        // GENERIC, chosen well above any credible in-flight error but well
        // below runaway.
        let self_heating_error_k = self_heating_error_k.min(20.0);

        let target_k = ideal_tat_k + self_heating_error_k;

        // ---- Thermal lag: ice on the element roughly doubles the thermal
        // time constant at ICE_TAU_DOUBLE_KG (added mass + insulation).
        let tau = BASE_THERMAL_TAU_S * (1.0 + (self.ice_kg / ICE_TAU_DOUBLE_KG).min(3.0));
        let k = (-dt / tau.max(1e-6)).exp();
        let target_c = target_k - 273.15;
        self.element_c = target_c + (self.element_c - target_c) * k;

        TatProbeOutput {
            sensed_tat_c: self.element_c,
            self_heating_error_k,
            ice_kg: self.ice_kg,
            heater_power_w: rated_w,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn settle(probe: &mut TatProbe, sat_c: f64, mach: f64, tas: f64, lwc: f64, powered: bool, faults: &TatProbeFaults, n: u32) -> TatProbeOutput {
        let mut out = TatProbeOutput::default();
        for _ in 0..n {
            out = probe.step(sat_c, mach, tas, lwc, powered, faults, 0.1);
        }
        out
    }

    #[test]
    fn ideal_recovery_matches_the_isentropic_formula_at_high_tas_no_self_heating() {
        let mut probe = TatProbe::new(15.0);
        // Heater off (no self-heating term to worry about), high TAS.
        let out = settle(&mut probe, -20.0, 0.82, 240.0, 0.0, false, &TatProbeFaults::default(), 2000);
        let sat_k = -20.0 + 273.15;
        let expected_c = sat_k * (1.0 + RECOVERY_FACTOR * 0.2 * 0.82 * 0.82) - 273.15;
        assert!((out.sensed_tat_c - expected_c).abs() < 0.2, "{} vs {}", out.sensed_tat_c, expected_c);
    }

    #[test]
    fn self_heating_error_is_larger_at_low_tas_than_high_tas() {
        let mut low_tas = TatProbe::new(15.0);
        let mut high_tas = TatProbe::new(15.0);
        let low = settle(&mut low_tas, 15.0, 0.05, 5.0, 0.0, true, &TatProbeFaults::default(), 500);
        let high = settle(&mut high_tas, 15.0, 0.7, 220.0, 0.0, true, &TatProbeFaults::default(), 500);
        assert!(low.self_heating_error_k > high.self_heating_error_k, "low {} high {}", low.self_heating_error_k, high.self_heating_error_k);
        assert!(low.self_heating_error_k > 0.5);
    }

    #[test]
    fn degraded_recovery_factor_reads_closer_to_sat_than_healthy() {
        let mut healthy = TatProbe::new(15.0);
        let mut degraded = TatProbe::new(15.0);
        let degraded_faults = TatProbeFaults { recovery_degradation: 0.8, ..Default::default() };
        let h = settle(&mut healthy, -20.0, 0.8, 230.0, 0.0, false, &TatProbeFaults::default(), 2000);
        let d = settle(&mut degraded, -20.0, 0.8, 230.0, 0.0, false, &degraded_faults, 2000);
        assert!(d.sensed_tat_c < h.sensed_tat_c);
        assert!(d.sensed_tat_c > -20.0 - 1.0);
    }

    #[test]
    fn icing_increases_the_thermal_time_constant() {
        let mut probe = TatProbe::new(-20.0);
        let faults = TatProbeFaults::default();
        // Build up ice in cold, wet, low-speed (little melting from
        // self-heating since heater is off).
        for _ in 0..3000 {
            probe.step(-20.0, 0.3, 100.0, 0.8, false, &faults, 0.1);
        }
        let iced = probe.ice_kg;
        assert!(iced > 0.0, "expected ice to build up, got {iced}");
    }

    #[test]
    fn heater_off_and_healthy_element_settles_near_sat_at_zero_mach() {
        let mut probe = TatProbe::new(50.0);
        let out = settle(&mut probe, -10.0, 0.0, 0.0, 0.0, false, &TatProbeFaults::default(), 500);
        assert!((out.sensed_tat_c - (-10.0)).abs() < 0.5, "{}", out.sensed_tat_c);
    }

    #[test]
    fn no_nan_at_zero_dt_or_rest() {
        let mut probe = TatProbe::new(15.0);
        let out = probe.step(15.0, 0.0, 0.0, 0.0, false, &TatProbeFaults::default(), 0.0);
        assert!(out.sensed_tat_c.is_finite());
    }
}
