//! Aerodynamic hinge moment on a control surface: the torque about the
//! hinge line that the actuator(s) in `actuator.rs` must react in order to
//! hold or move the surface, and which drives it (blow-back) whenever they
//! can't.
//!
//! Hinge moments are non-dimensionalised the standard way in aerodynamics
//! texts (e.g. Roskam, "Airplane Flight Dynamics and Automatic Flight
//! Controls" Part I, ch. 2; Etkin & Reid, "Dynamics of Flight", ch. 2 on
//! control-surface hinge moments):
//!
//!   `M_hinge = q * S * c_bar * Ch(delta, alpha)`
//!   `Ch = Ch_delta * delta + Ch_alpha * alpha` (linear region, small angles)
//!   `q = 0.5 * rho * V_true^2`
//!
//! with a Prandtl-Glauert compressibility correction `1/sqrt(1 - M^2)` below
//! the surface's critical Mach, and a GENERIC transonic fall-off above it
//! (shock-induced separation reduces control effectiveness rather than
//! letting Ch diverge at M -> 1, which the plain PG factor would do).
//!
//! No A380-specific hinge-moment data is public. `Ch_delta`/`Ch_alpha`
//! magnitudes below are GENERIC: representative slopes for large-transport
//! plain control surfaces, the same order as NACA TR-868's measured
//! `Ch_delta` for a 0.3c plain flap on a NACA 0009 section
//! (about -0.006/deg = -0.34/rad at low Mach), scaled to each A380 surface's
//! own area and mean chord (GENERIC areas/chords: no public planform data
//! for the individual panels; sized to match each panel's span from
//! `flight_controls.rs` at a chord fraction typical of that surface type).

use std::f64::consts::PI;

const DEG_RAD: f64 = PI / 180.0;

/// One surface's hinge-moment sizing: coefficient slopes plus the area and
/// mean chord that turn them into a physical moment.
#[derive(Clone, Copy, Debug)]
pub struct HingeMomentCoefficients {
    /// Per radian of surface deflection (restoring: negative).
    pub ch_delta_per_rad: f64,
    /// Per radian of local wing/tail angle of attack.
    pub ch_alpha_per_rad: f64,
    /// The coefficient's own saturation magnitude: real Ch flattens out
    /// once flow starts separating off the deflected surface well before
    /// the linear extrapolation would suggest (GENERIC).
    pub ch_max: f64,
    pub area_m2: f64,
    pub chord_m: f64,
}

impl HingeMomentCoefficients {
    /// Aileron panel: GENERIC 1.6 m^2 x 1.4 m mean chord (roughly the
    /// aileron body sizes in a380_systems/src/hydraulic/mod.rs:449-452,
    /// which give panel depths 1.37-1.6 m), Ch_delta/Ch_alpha the NACA
    /// TR-868 order above.
    pub fn aileron() -> Self {
        Self { ch_delta_per_rad: -0.34, ch_alpha_per_rad: -0.20, ch_max: 0.30, area_m2: 1.6, chord_m: 1.4 }
    }

    /// Elevator panel: GENERIC 4.0 m^2 x 2.3 m mean chord (elevator body
    /// depths 2.23-2.49 m, mod.rs:784-788), same coefficient order (a plain
    /// hinged surface, not a slotted/tabbed one).
    pub fn elevator() -> Self {
        Self { ch_delta_per_rad: -0.32, ch_alpha_per_rad: -0.22, ch_max: 0.30, area_m2: 4.0, chord_m: 2.3 }
    }

    /// Rudder panel: GENERIC 3.5 m^2 x 2.2 m mean chord (rudder body sizes
    /// referenced from mod.rs:960 onward), a fin-mounted plain surface so
    /// `alpha` there means local sideslip/fin angle, not wing AoA.
    pub fn rudder() -> Self {
        Self { ch_delta_per_rad: -0.33, ch_alpha_per_rad: -0.18, ch_max: 0.30, area_m2: 3.5, chord_m: 2.2 }
    }

    /// Spoiler: a flat plate standing up into the flow rather than a
    /// trailing-edge flap, so its hinge moment grows almost linearly with
    /// deflection over its whole (small) travel and does not meaningfully
    /// depend on wing alpha the way a flap's does. GENERIC 1.2 m^2 x 0.685 m
    /// chord (the spoiler body's own chordwise size, mod.rs:624).
    pub fn spoiler() -> Self {
        Self { ch_delta_per_rad: -0.55, ch_alpha_per_rad: -0.02, ch_max: 0.45, area_m2: 1.2, chord_m: 0.685 }
    }
}

/// Compressibility factor: Prandtl-Glauert below `mach_crit`, a GENERIC
/// linear fall-off toward mostly-blanked-off effectiveness above it (real
/// transonic hinge moments are dominated by shock position, which needs
/// unsteady/CFD data this crate does not have).
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

/// The hinge moment, N*m, positive in the sense that increases `delta_rad`.
/// `dynamic_pressure_pa` and `mach` are always >= 0 by construction here (no
/// NaN at V = 0: q = 0 gives M_hinge = 0 regardless of angles).
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

/// Convenience: hinge moment for degrees rather than radians.
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
        // Positive deflection should give a negative (restoring) moment
        // since Ch_delta is negative.
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
        let m = hinge_moment_nm(&c, 10.0, 0.0, 5000.0, 0.2, 0.7).abs(); // absurd delta
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
