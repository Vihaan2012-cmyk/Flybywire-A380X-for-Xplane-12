use std::f64::consts::PI;

const PROBE_DIAMETER_M: f64 = 0.013;
const PROBE_EXPOSED_LENGTH_M: f64 = 0.15;
pub const RATED_HEATER_W: f64 = 350.0;
const TARGET_SURFACE_C: f64 = 0.0;

const AIR_K_W_MK: f64 = 0.0206;
const AIR_NU_M2_S: f64 = 1.13e-5;
const AIR_PR: f64 = 0.72;
const WATER_CP_J_KGK: f64 = 4186.0;
const WATER_LF_J_KG: f64 = 334_000.0;
const REFERENCE_LWC_GM3: f64 = 0.6;

const ICE_BLOCK_MASS_KG: f64 = 0.0012;

const PNEUMATIC_TAU_S: f64 = 0.25;
const DRAIN_LEAK_TAU_S: f64 = 4.0;
const BLOCKED_OPEN_FRACTION: f64 = 0.03;

fn convective_h_w_m2k(tas_ms: f64) -> f64 {
    let v = tas_ms.max(0.0);
    let re = v * PROBE_DIAMETER_M / AIR_NU_M2_S;
    if re <= 1.0 {
        return 0.0;
    }
    let (c, m) = if re <= 2.0e5 { (0.26, 0.6) } else { (0.076, 0.7) };
    let nu = c * re.powf(m) * AIR_PR.powf(0.37);
    nu * AIR_K_W_MK / PROBE_DIAMETER_M
}

fn heat_required_w(sat_c: f64, tas_ms: f64, lwc_gm3: f64) -> (f64, f64) {
    let delta_t = (TARGET_SURFACE_C - sat_c).max(0.0);
    let h = convective_h_w_m2k(tas_ms);
    let conv_area_m2 = PI * PROBE_DIAMETER_M * PROBE_EXPOSED_LENGTH_M;
    let q_conv = h * conv_area_m2 * delta_t;

    let frontal_area_m2 = PROBE_DIAMETER_M * PROBE_EXPOSED_LENGTH_M;
    let lwc_kg_m3 = lwc_gm3.max(0.0) * 1e-3;
    let catch_kg_s = lwc_kg_m3 * tas_ms.max(0.0) * frontal_area_m2;
    let q_water = catch_kg_s * (WATER_CP_J_KGK * delta_t + WATER_LF_J_KG);

    (q_conv + q_water, catch_kg_s)
}

#[derive(Clone, Copy, Debug, Default)]
pub struct PitotFaults {
    pub heater_failure: f64,
    pub insect_or_tape_blockage: f64,
    pub mechanical_damage: f64,
    pub drain_blocked: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct PitotOutput {
    pub sensed_total_pressure_pa: f64,
    pub blocked: bool,
    pub ice_kg: f64,
    pub heater_power_w: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct PitotProbe {
    ice_kg: f64,
    sensed_pa: f64,
}

impl PitotProbe {
    pub fn new(initial_total_pa: f64) -> Self {
        Self { ice_kg: 0.0, sensed_pa: initial_total_pa }
    }

    pub fn step(
        &mut self,
        true_total_pa: f64,
        true_static_pa: f64,
        tas_ms: f64,
        sat_c: f64,
        lwc_gm3: f64,
        powered: bool,
        faults: &PitotFaults,
        dt_s: f64,
    ) -> PitotOutput {
        let dt = dt_s.max(0.0);

        let rated_w = if powered { RATED_HEATER_W * (1.0 - faults.heater_failure.clamp(0.0, 1.0)) } else { 0.0 };
        let (required_w, catch_kg_s) = heat_required_w(sat_c, tas_ms, lwc_gm3);
        let deficit_w = (required_w - rated_w).max(0.0);
        let surplus_w = (rated_w - required_w).max(0.0);
        let accretion_kg_s = if required_w > 0.0 { catch_kg_s * (deficit_w / required_w).min(1.0) } else { 0.0 };
        let melt_kg_s = surplus_w / WATER_LF_J_KG;
        self.ice_kg = (self.ice_kg + (accretion_kg_s - melt_kg_s) * dt).max(0.0);

        let ice_open = (1.0 - (self.ice_kg / ICE_BLOCK_MASS_KG).min(1.0)).max(0.0);
        let mech_open = (1.0 - faults.insect_or_tape_blockage.clamp(0.0, 1.0))
            * (1.0 - faults.mechanical_damage.clamp(0.0, 1.0));
        let open_fraction = (ice_open * mech_open).clamp(0.0, 1.0);
        let blocked = open_fraction < BLOCKED_OPEN_FRACTION;

        if blocked {
            let drain_clear = 1.0 - faults.drain_blocked.clamp(0.0, 1.0);
            if drain_clear > 0.02 {
                let tau = (DRAIN_LEAK_TAU_S / drain_clear.max(0.02)).max(0.1);
                let k = (-dt / tau).exp();
                self.sensed_pa = true_static_pa + (self.sensed_pa - true_static_pa) * k;
            }
        } else {
            let tau = PNEUMATIC_TAU_S / (open_fraction * open_fraction).max(1e-3);
            let k = (-dt / tau.max(1e-6)).exp();
            self.sensed_pa = true_total_pa + (self.sensed_pa - true_total_pa) * k;
        }

        PitotOutput {
            sensed_total_pressure_pa: self.sensed_pa,
            blocked,
            ice_kg: self.ice_kg,
            heater_power_w: rated_w,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn step_n(probe: &mut PitotProbe, total: f64, static_p: f64, tas: f64, sat_c: f64, lwc: f64, powered: bool, faults: &PitotFaults, dt: f64, n: u32) -> PitotOutput {
        let mut out = PitotOutput::default();
        for _ in 0..n {
            out = probe.step(total, static_p, tas, sat_c, lwc, powered, faults, dt);
        }
        out
    }

    #[test]
    fn healthy_probe_tracks_a_pressure_step_within_a_few_time_constants() {
        let mut probe = PitotProbe::new(101_325.0);
        let out = step_n(&mut probe, 108_000.0, 95_000.0, 200.0, 15.0, 0.0, true, &PitotFaults::default(), 0.05, 200);
        assert!((out.sensed_total_pressure_pa - 108_000.0).abs() < 10.0, "{}", out.sensed_total_pressure_pa);
        assert!(!out.blocked);
    }

    #[test]
    fn no_nan_at_zero_dt_or_rest() {
        let mut probe = PitotProbe::new(0.0);
        let out = probe.step(0.0, 0.0, 0.0, 15.0, 0.0, false, &PitotFaults::default(), 0.0);
        assert!(out.sensed_total_pressure_pa.is_finite());
        assert!(!out.sensed_total_pressure_pa.is_nan());
    }

    #[test]
    fn unheated_icing_eventually_blocks_the_tube() {
        let mut probe = PitotProbe::new(101_325.0);
        let faults = PitotFaults::default();
        let out = step_n(&mut probe, 108_000.0, 95_000.0, 220.0, -20.0, 0.6, false, &faults, 0.5, 400);
        assert!(out.blocked, "ice_kg={}", out.ice_kg);
        assert!(out.ice_kg > 0.0);
    }

    #[test]
    fn heater_prevents_blockage_in_the_same_icing_conditions() {
        let mut probe = PitotProbe::new(101_325.0);
        let faults = PitotFaults::default();
        let out = step_n(&mut probe, 108_000.0, 95_000.0, 220.0, -20.0, 0.6, true, &faults, 0.5, 400);
        assert!(!out.blocked);
        assert_eq!(out.ice_kg, 0.0);
    }

    #[test]
    fn blocked_tube_with_clear_drain_decays_toward_static_pressure() {
        let mut probe = PitotProbe::new(101_325.0);
        let faults = PitotFaults { mechanical_damage: 1.0, ..Default::default() };
        let true_total = 108_000.0;
        let true_static = 95_000.0;
        let out = step_n(&mut probe, true_total, true_static, 200.0, 15.0, 0.0, true, &faults, 0.5, 60);
        assert!(out.blocked);
        assert!((out.sensed_total_pressure_pa - true_static).abs() < 500.0, "{}", out.sensed_total_pressure_pa);
    }

    #[test]
    fn blocked_tube_with_blocked_drain_freezes_regardless_of_true_pressure_changes() {
        let mut probe = PitotProbe::new(101_325.0);
        let faults = PitotFaults { mechanical_damage: 1.0, drain_blocked: 1.0, ..Default::default() };
        let frozen = probe.step(108_000.0, 95_000.0, 200.0, 15.0, 0.0, true, &faults, 0.5).sensed_total_pressure_pa;
        let out = step_n(&mut probe, 40_000.0, 30_000.0, 250.0, -40.0, 0.0, true, &faults, 0.5, 200);
        assert!((out.sensed_total_pressure_pa - frozen).abs() < 1.0, "frozen {} now {}", frozen, out.sensed_total_pressure_pa);
    }

    #[test]
    fn partial_insect_blockage_slows_the_pneumatic_response_without_declaring_blocked() {
        let mut clean = PitotProbe::new(101_325.0);
        let mut restricted = PitotProbe::new(101_325.0);
        let restricted_faults = PitotFaults { insect_or_tape_blockage: 0.5, ..Default::default() };
        let clean_out = clean.step(108_000.0, 95_000.0, 200.0, 15.0, 0.0, true, &PitotFaults::default(), 0.2);
        let restricted_out = restricted.step(108_000.0, 95_000.0, 200.0, 15.0, 0.0, true, &restricted_faults, 0.2);
        assert!(!restricted_out.blocked);
        let clean_progress = (clean_out.sensed_total_pressure_pa - 101_325.0).abs();
        let restricted_progress = (restricted_out.sensed_total_pressure_pa - 101_325.0).abs();
        assert!(restricted_progress < clean_progress, "clean {clean_progress}, restricted {restricted_progress}");
    }

    #[test]
    fn severe_insect_blockage_is_treated_as_blocked() {
        let mut probe = PitotProbe::new(101_325.0);
        let faults = PitotFaults { insect_or_tape_blockage: 0.99, ..Default::default() };
        let out = probe.step(108_000.0, 95_000.0, 200.0, 15.0, 0.0, true, &faults, 0.1);
        assert!(out.blocked);
    }

    #[test]
    fn heater_power_reports_zero_when_unpowered_even_if_the_element_is_healthy() {
        let mut probe = PitotProbe::new(101_325.0);
        let out = probe.step(108_000.0, 95_000.0, 200.0, -20.0, 0.6, false, &PitotFaults::default(), 0.1);
        assert_eq!(out.heater_power_w, 0.0);
    }
}
