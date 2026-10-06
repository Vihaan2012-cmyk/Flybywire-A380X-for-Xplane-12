use super::rng::Rng;

const G: f64 = 9.806_65;

#[derive(Clone, Copy, Debug)]
pub struct WindShearInputs {
    pub headwind_rate_ms2: f64,
    pub downdraft_ms: f64,
    pub tas_ms: f64,
}

pub fn f_factor(i: &WindShearInputs) -> f64 {
    let v = i.tas_ms.max(1.0);
    -i.headwind_rate_ms2 / G + i.downdraft_ms / v
}

pub const F_FACTOR_HAZARD_THRESHOLD: f64 = 0.105;

pub fn is_hazardous(f: f64) -> bool {
    f > F_FACTOR_HAZARD_THRESHOLD
}

#[derive(Clone, Copy, Debug)]
pub struct MicroburstEvent {
    pub peak_headwind_swing_ms: f64,
    pub peak_downdraft_ms: f64,
    pub length_m: f64,
}

impl MicroburstEvent {
    fn sample(&self, distance_m: f64, ground_speed_ms: f64) -> WindShearInputs {
        let l = self.length_m.max(1.0);
        if distance_m < 0.0 || distance_m > l {
            return WindShearInputs { headwind_rate_ms2: 0.0, downdraft_ms: 0.0, tas_ms: ground_speed_ms.max(1.0) };
        }
        let s = distance_m / l;
        let dheadwind_dx = -self.peak_headwind_swing_ms * (std::f64::consts::PI / l) * (std::f64::consts::PI * s).sin();
        let headwind_rate_ms2 = dheadwind_dx * ground_speed_ms.max(0.0);
        let downdraft_ms = self.peak_downdraft_ms * (std::f64::consts::PI * s).sin();
        WindShearInputs { headwind_rate_ms2, downdraft_ms, tas_ms: ground_speed_ms.max(1.0) }
    }
}

const BASE_RATE_PER_S_AT_FULL_INTENSITY: f64 = 1.0 / (600.0 * 60.0);
const SEVERE_HEADWIND_SWING_MS: f64 = 20.0;
const SEVERE_DOWNDRAFT_MS: f64 = 8.0;
const SEVERE_LENGTH_M: f64 = 3000.0;

#[derive(Default)]
pub struct WindShearModel {
    active: Option<(MicroburstEvent, f64)>,
    pub random_mode: bool,
}

impl WindShearModel {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn trigger(&mut self, event: MicroburstEvent) {
        self.active = Some((event, 0.0));
    }

    pub fn step(&mut self, convective_intensity: f64, ground_speed_ms: f64, dt_s: f64, rng: &mut Rng) -> Option<WindShearInputs> {
        if self.active.is_none() && self.random_mode {
            let p = BASE_RATE_PER_S_AT_FULL_INTENSITY * convective_intensity.clamp(0.0, 1.0) * dt_s.max(0.0);
            if rng.chance(p) {
                self.active = Some((MicroburstEvent { peak_headwind_swing_ms: SEVERE_HEADWIND_SWING_MS, peak_downdraft_ms: SEVERE_DOWNDRAFT_MS, length_m: SEVERE_LENGTH_M }, 0.0));
            }
        }
        let (event, distance) = self.active.as_mut()?;
        *distance += ground_speed_ms.max(0.0) * dt_s.max(0.0);
        if *distance > event.length_m {
            self.active = None;
            return None;
        }
        Some(event.sample(*distance, ground_speed_ms))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TurbulenceIntensity {
    Light,
    Moderate,
    Severe,
}

impl TurbulenceIntensity {
    fn sigma_w_ms(self) -> f64 {
        match self {
            TurbulenceIntensity::Light => 1.0,
            TurbulenceIntensity::Moderate => 3.0,
            TurbulenceIntensity::Severe => 6.0,
        }
    }
}

fn scale_length_w_m(agl_m: f64) -> f64 {
    let h_ft = (agl_m / 0.3048).max(0.0);
    let l_ft = if h_ft <= 1000.0 {
        h_ft.max(10.0)
    } else if h_ft >= 2000.0 {
        1750.0
    } else {
        let t = (h_ft - 1000.0) / 1000.0;
        1000.0 + (1750.0 - 1000.0) * t
    };
    l_ft * 0.3048
}

fn scale_length_uv_m(agl_m: f64) -> f64 {
    let h_ft = (agl_m / 0.3048).max(0.0);
    if h_ft <= 1000.0 {
        let lw_ft = h_ft.max(10.0);
        (lw_ft / (0.177 + 0.000823 * h_ft).powf(1.2)) * 0.3048
    } else {
        scale_length_w_m(agl_m)
    }
}

fn sigma_uv_ms(agl_m: f64, sigma_w: f64) -> f64 {
    let h_ft = (agl_m / 0.3048).max(0.0);
    if h_ft <= 1000.0 {
        sigma_w / (0.177 + 0.000823 * h_ft).powf(0.4)
    } else {
        sigma_w
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TurbulenceGusts {
    pub u_ms: f64,
    pub v_ms: f64,
    pub w_ms: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct TurbulenceModel {
    u: f64,
    v: f64,
    w: f64,
}

fn ou_step(x: f64, scale_length_m: f64, sigma: f64, tas_ms: f64, dt_s: f64, rng: &mut Rng) -> f64 {
    let tau = (scale_length_m.max(1e-3) / tas_ms.max(1.0)).max(1e-6);
    let decay = (-dt_s.max(0.0) / tau).exp();
    x * decay + sigma * (1.0 - decay * decay).max(0.0).sqrt() * rng.normal()
}

impl TurbulenceModel {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn step(&mut self, altitude_agl_m: f64, tas_ms: f64, intensity: TurbulenceIntensity, dt_s: f64, rng: &mut Rng) -> TurbulenceGusts {
        let sigma_w = intensity.sigma_w_ms();
        let sigma_uv = sigma_uv_ms(altitude_agl_m, sigma_w);
        let lw = scale_length_w_m(altitude_agl_m);
        let luv = scale_length_uv_m(altitude_agl_m);
        self.u = ou_step(self.u, luv, sigma_uv, tas_ms, dt_s, rng);
        self.v = ou_step(self.v, luv, sigma_uv, tas_ms, dt_s, rng);
        self.w = ou_step(self.w, lw, sigma_w, tas_ms, dt_s, rng);
        TurbulenceGusts { u_ms: self.u, v_ms: self.v, w_ms: self.w }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn calm_air_gives_zero_f_factor_and_no_hazard() {
        let f = f_factor(&WindShearInputs { headwind_rate_ms2: 0.0, downdraft_ms: 0.0, tas_ms: 70.0 });
        assert_eq!(f, 0.0);
        assert!(!is_hazardous(f));
    }

    #[test]
    fn losing_headwind_and_a_downdraft_together_are_hazardous() {
        let f = f_factor(&WindShearInputs { headwind_rate_ms2: -3.0, downdraft_ms: 5.0, tas_ms: 70.0 });
        assert!(f > 0.0);
        assert!(is_hazardous(f));
    }

    #[test]
    fn gaining_headwind_is_never_hazardous() {
        let f = f_factor(&WindShearInputs { headwind_rate_ms2: 3.0, downdraft_ms: 0.0, tas_ms: 70.0 });
        assert!(f < 0.0);
        assert!(!is_hazardous(f));
    }

    #[test]
    fn a_severe_microburst_profile_produces_a_hazardous_f_factor_somewhere_in_the_core() {
        let event = MicroburstEvent { peak_headwind_swing_ms: SEVERE_HEADWIND_SWING_MS, peak_downdraft_ms: SEVERE_DOWNDRAFT_MS, length_m: SEVERE_LENGTH_M };
        let ground_speed = 70.0;
        let mut max_f = f64::MIN;
        let mut steps = 0;
        for i in 0..1000 {
            let d = event.length_m * i as f64 / 999.0;
            let inputs = event.sample(d, ground_speed);
            max_f = max_f.max(f_factor(&inputs));
            steps += 1;
        }
        assert!(steps > 0);
        assert!(max_f > F_FACTOR_HAZARD_THRESHOLD, "max F {max_f}");
    }

    #[test]
    fn outside_the_event_the_wind_field_is_calm() {
        let event = MicroburstEvent { peak_headwind_swing_ms: 20.0, peak_downdraft_ms: 8.0, length_m: 1000.0 };
        let before = event.sample(-10.0, 70.0);
        let after = event.sample(1500.0, 70.0);
        assert_eq!(before.headwind_rate_ms2, 0.0);
        assert_eq!(before.downdraft_ms, 0.0);
        assert_eq!(after.downdraft_ms, 0.0);
    }

    #[test]
    fn manual_trigger_runs_the_aircraft_through_the_event_and_then_clears() {
        let mut model = WindShearModel::new();
        model.trigger(MicroburstEvent { peak_headwind_swing_ms: 15.0, peak_downdraft_ms: 6.0, length_m: 500.0 });
        let mut rng = Rng::new(1);
        let mut saw_any = false;
        let mut cleared = false;
        for _ in 0..200 {
            match model.step(0.0, 70.0, 1.0, &mut rng) {
                Some(_) => saw_any = true,
                None => {
                    cleared = true;
                    break;
                }
            }
        }
        assert!(saw_any);
        assert!(cleared);
    }

    #[test]
    fn random_mode_needs_convective_weather() {
        let mut model = WindShearModel::new();
        model.random_mode = true;
        let mut rng = Rng::new(2);
        for _ in 0..500_000 {
            assert!(model.step(0.0, 70.0, 1.0, &mut rng).is_none());
        }
    }

    #[test]
    fn scale_lengths_match_the_mil_hdbk_1797_reference_points() {
        assert!((scale_length_w_m(1000.0 * 0.3048) / 0.3048 - 1000.0).abs() < 1.0);
        assert!((scale_length_w_m(2000.0 * 0.3048) / 0.3048 - 1750.0).abs() < 1.0);
        assert!((scale_length_w_m(10_000.0 * 0.3048) / 0.3048 - 1750.0).abs() < 1.0);
    }

    #[test]
    fn turbulence_is_numerically_safe_at_rest_and_zero_dt() {
        let mut t = TurbulenceModel::new();
        let mut rng = Rng::new(3);
        let g = t.step(0.0, 0.0, TurbulenceIntensity::Severe, 0.0, &mut rng);
        assert!(!g.u_ms.is_nan() && !g.v_ms.is_nan() && !g.w_ms.is_nan());
    }

    #[test]
    fn severe_turbulence_has_a_larger_rms_gust_than_light_turbulence() {
        let mut light = TurbulenceModel::new();
        let mut severe = TurbulenceModel::new();
        let mut rng_l = Rng::new(10);
        let mut rng_s = Rng::new(10);
        let (mut sum_l, mut sum_s, n) = (0.0, 0.0, 20_000);
        for _ in 0..n {
            let gl = light.step(3000.0, 120.0, TurbulenceIntensity::Light, 0.1, &mut rng_l);
            let gs = severe.step(3000.0, 120.0, TurbulenceIntensity::Severe, 0.1, &mut rng_s);
            sum_l += gl.w_ms * gl.w_ms;
            sum_s += gs.w_ms * gs.w_ms;
        }
        let rms_l = (sum_l / n as f64).sqrt();
        let rms_s = (sum_s / n as f64).sqrt();
        assert!(rms_s > rms_l * 2.0, "light {rms_l}, severe {rms_s}");
    }
}
