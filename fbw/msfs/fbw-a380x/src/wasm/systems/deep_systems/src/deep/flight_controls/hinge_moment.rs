use std::f64::consts::PI;

const DEG_RAD: f64 = PI / 180.0;

#[derive(Clone, Copy, Debug)]
pub struct HingeMomentCoefficients {
    pub ch_delta_per_rad: f64,
    pub ch_alpha_per_rad: f64,
    pub ch_max: f64,
    pub area_m2: f64,
    pub chord_m: f64,
}

impl HingeMomentCoefficients {
    pub fn aileron() -> Self {
        Self { ch_delta_per_rad: -0.34, ch_alpha_per_rad: -0.20, ch_max: 0.30, area_m2: 1.6, chord_m: 1.4 }
    }

    pub fn elevator() -> Self {
        Self { ch_delta_per_rad: -0.32, ch_alpha_per_rad: -0.22, ch_max: 0.30, area_m2: 4.0, chord_m: 2.3 }
    }

    pub fn rudder() -> Self {
        Self { ch_delta_per_rad: -0.33, ch_alpha_per_rad: -0.18, ch_max: 0.30, area_m2: 3.5, chord_m: 2.2 }
    }

    pub fn spoiler() -> Self {
        Self { ch_delta_per_rad: -0.55, ch_alpha_per_rad: -0.02, ch_max: 0.45, area_m2: 1.2, chord_m: 0.685 }
    }
}

pub fn compressibility(mach: f64, mach_crit: f64) -> f64 {
    let m = mach.max(0.0);
    let pg = |x: f64| 1.0 / (1.0 - x * x).max(0.01).sqrt();
    if m < mach_crit {
        pg(m.min(0.95))
    } else {
        let at_crit = pg(mach_crit.min(0.95));
        let falloff = (1.0 - 0.7 * ((m - mach_crit) / 0.35).clamp(0.0, 1.0)).max(0.15);
        at_crit * falloff
    }
}

pub fn hinge_moment_nm(
    c: &HingeMomentCoefficients,
    delta_rad: f64,
    alpha_rad: f64,
    dynamic_pressure_pa: f64,
    mach: f64,
    mach_crit: f64,
) -> f64 {
    let q = dynamic_pressure_pa.max(0.0);
    let ch_lin = c.ch_delta_per_rad * delta_rad + c.ch_alpha_per_rad * alpha_rad;
    let ch = ch_lin.clamp(-c.ch_max, c.ch_max);
    q * c.area_m2 * c.chord_m * ch * compressibility(mach, mach_crit)
}

pub fn hinge_moment_nm_deg(
    c: &HingeMomentCoefficients,
    delta_deg: f64,
    alpha_deg: f64,
    dynamic_pressure_pa: f64,
    mach: f64,
    mach_crit: f64,
) -> f64 {
    hinge_moment_nm(c, delta_deg * DEG_RAD, alpha_deg * DEG_RAD, dynamic_pressure_pa, mach, mach_crit)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_dynamic_pressure_gives_zero_moment_no_nan() {
        let c = HingeMomentCoefficients::aileron();
        let m = hinge_moment_nm(&c, 0.3, 0.1, 0.0, 0.0, 0.75);
        assert_eq!(m, 0.0);
        assert!(m.is_finite());
    }

    #[test]
    fn deflection_produces_a_restoring_moment() {
        let c = HingeMomentCoefficients::elevator();
        let m = hinge_moment_nm(&c, 0.2, 0.0, 5000.0, 0.3, 0.7);
        assert!(m < 0.0);
        let m2 = hinge_moment_nm(&c, -0.2, 0.0, 5000.0, 0.3, 0.7);
        assert!(m2 > 0.0);
        assert!((m + m2).abs() < 1e-6, "should be antisymmetric in delta");
    }

    #[test]
    fn moment_scales_with_dynamic_pressure() {
        let c = HingeMomentCoefficients::rudder();
        let low = hinge_moment_nm(&c, 0.2, 0.0, 2000.0, 0.2, 0.7).abs();
        let high = hinge_moment_nm(&c, 0.2, 0.0, 8000.0, 0.2, 0.7).abs();
        assert!((high - 4.0 * low).abs() < 1.0);
    }

    #[test]
    fn coefficient_saturates_rather_than_growing_without_bound() {
        let c = HingeMomentCoefficients::spoiler();
        let m = hinge_moment_nm(&c, 10.0, 0.0, 5000.0, 0.2, 0.7).abs();
        let max_possible = 5000.0 * c.area_m2 * c.chord_m * c.ch_max * compressibility(0.2, 0.7);
        assert!(m <= max_possible + 1e-6);
    }

    #[test]
    fn compressibility_grows_toward_mach_crit_then_falls_off() {
        let sub = compressibility(0.2, 0.75);
        let near_crit = compressibility(0.7, 0.75);
        let above = compressibility(0.95, 0.75);
        assert!(near_crit > sub);
        assert!(above < near_crit, "transonic fall-off should reduce effectiveness");
        assert!(above.is_finite() && above > 0.0);
    }
}
