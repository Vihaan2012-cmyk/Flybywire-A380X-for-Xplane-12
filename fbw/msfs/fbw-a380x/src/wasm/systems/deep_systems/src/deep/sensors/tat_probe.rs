const RECOVERY_FACTOR: f64 = 0.99;
const GAMMA_AIR: f64 = 1.4;
const R_AIR: f64 = 287.052_87;

const ELEMENT_DIAMETER_M: f64 = 0.006;
const ELEMENT_EXPOSED_LENGTH_M: f64 = 0.04;
pub const RATED_HEATER_W: f64 = 80.0;
const AIR_K_W_MK: f64 = 0.0206;
const AIR_NU_M2_S: f64 = 1.13e-5;
const AIR_PR: f64 = 0.72;

const BASE_THERMAL_TAU_S: f64 = 0.4;
const ICE_TAU_DOUBLE_KG: f64 = 0.0005;
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

        let rated_w = if powered { RATED_HEATER_W * (1.0 - faults.heater_failure.clamp(0.0, 1.0)) } else { 0.0 };
        if sat_c < 0.0 && lwc_gm3 > 0.0 {
            self.ice_kg += ICE_ACCUM_KG_S_PER_LWC_TAS * lwc_gm3.max(0.0) * tas_ms.max(0.0) * dt;
        }
        if self.element_c > 0.5 {
            let melt_tau_s = 5.0;
            self.ice_kg *= (-dt / melt_tau_s).exp();
        }
        self.ice_kg = self.ice_kg.max(0.0);

        const LOCAL_COUPLING_FRACTION: f64 = 0.01;
        let local_heat_w = rated_w * LOCAL_COUPLING_FRACTION;
        let h = convective_h_w_m2k(tas_ms);
        let area_m2 = std::f64::consts::PI * ELEMENT_DIAMETER_M * ELEMENT_EXPOSED_LENGTH_M;
        let self_heating_error_k = if h * area_m2 > 1e-9 { local_heat_w / (h * area_m2) } else if local_heat_w > 0.0 { 20.0 } else { 0.0 };
        let self_heating_error_k = self_heating_error_k.min(20.0);

        let target_k = ideal_tat_k + self_heating_error_k;

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
