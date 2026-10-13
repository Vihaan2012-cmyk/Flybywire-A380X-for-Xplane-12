//! ISA atmosphere plus wind, shear and gusts: what the aerodynamics and
//! landing-gear friction models need to turn ground/air-relative speed into
//! forces, and what the emulator's own environment setters
//! (`set_oat_c`/`set_qnh_pa`/`set_wind_ms`, `lib.rs`) are the coupling
//! points for -- see `PROGRESS.md` for exactly which `Emulator` method feeds
//! which field here.
//!
//! ISA constants and the troposphere lapse-rate formula are the standard
//! ICAO Doc 7488 (International Standard Atmosphere) relations, the same
//! ones every flight-dynamics text (e.g. Anderson, "Introduction to Flight")
//! reproduces; nothing about the A380 specifically. QNH/OAT *offsets* from
//! ISA are runtime inputs (what the emulator's `set_oat_c`/`set_qnh_pa`
//! already carry), not constants.

use super::math::Vec3;

pub const ISA_SEA_LEVEL_TEMP_K: f64 = 288.15;
pub const ISA_SEA_LEVEL_PRESSURE_PA: f64 = 101_325.0;
/// Troposphere lapse rate, K/m, valid to 11 km.
pub const ISA_LAPSE_K_M: f64 = 0.0065;
/// Specific gas constant for dry air, J/(kg K).
pub const R_AIR: f64 = 287.05;
/// Ratio of specific heats for air (speed of sound `a = sqrt(gamma*R*T)`).
pub const GAMMA_AIR: f64 = 1.4;
pub const G0: f64 = 9.806_65;
const TROPOPAUSE_M: f64 = 11_000.0;
const TROPOPAUSE_TEMP_K: f64 = ISA_SEA_LEVEL_TEMP_K - ISA_LAPSE_K_M * TROPOPAUSE_M;

/// The atmosphere's state at one point: static temperature/pressure/
/// density and the speed of sound, all SI.
#[derive(Clone, Copy, Debug)]
pub struct AirState {
    pub temp_k: f64,
    pub pressure_pa: f64,
    pub density_kg_m3: f64,
    pub sound_speed_m_s: f64,
}

/// ISA plus a sea-level temperature offset (`oat_offset_k`, e.g. an ISA+15
/// day) and a sea-level pressure offset (`qnh_offset_pa`, non-standard
/// QNH), at geometric altitude `alt_m` (flat-earth, no need for the
/// geopotential correction at these altitudes).
pub fn isa(alt_m: f64, oat_offset_k: f64, qnh_offset_pa: f64) -> AirState {
    let sea_level_temp = ISA_SEA_LEVEL_TEMP_K + oat_offset_k;
    let sea_level_pressure = ISA_SEA_LEVEL_PRESSURE_PA + qnh_offset_pa;
    let alt = alt_m.max(0.0);
    let (temp_k, pressure_pa) = if alt <= TROPOPAUSE_M {
        let t = sea_level_temp - ISA_LAPSE_K_M * alt;
        // Barometric formula for a linear lapse rate (ICAO 7488 eq. 5).
        let p = sea_level_pressure * (t / sea_level_temp).powf(G0 / (ISA_LAPSE_K_M * R_AIR));
        (t, p)
    } else {
        let t_tropopause = sea_level_temp - ISA_LAPSE_K_M * TROPOPAUSE_M;
        let p_tropopause =
            sea_level_pressure * (t_tropopause / sea_level_temp).powf(G0 / (ISA_LAPSE_K_M * R_AIR));
        // Isothermal stratosphere (ICAO 7488 eq. 3).
        let p = p_tropopause * (-G0 * (alt - TROPOPAUSE_M) / (R_AIR * t_tropopause.max(1.0))).exp();
        (t_tropopause, p)
    };
    let temp_k = temp_k.max(1.0);
    AirState {
        temp_k,
        pressure_pa: pressure_pa.max(0.0),
        density_kg_m3: pressure_pa.max(0.0) / (R_AIR * temp_k),
        sound_speed_m_s: (GAMMA_AIR * R_AIR * temp_k).sqrt(),
    }
}

/// Wind at one point: a steady component (world/NED frame, m/s) plus a
/// shear gradient with height and a gust/turbulence component.
#[derive(Clone, Copy, Debug, Default)]
pub struct Wind {
    /// Steady wind at 10 m AGL (the standard meteorological reference
    /// height, ICAO Annex 3), world frame (north, east, down), m/s.
    pub steady_10m: Vec3,
    /// Low-level wind-shear exponent for the power-law profile
    /// `v(h) = v_10m * (h/10)^shear_exponent` (ESDU 82026 / the classic
    /// 1/7-power-law atmospheric boundary layer approximation; GENERIC
    /// default 0.14 below, a mid-range open-terrain value, overridable per
    /// scenario for a low-level wind-shear case).
    pub shear_exponent: f64,
    /// A single discrete gust: peak speed added along the steady wind's own
    /// direction, world frame, m/s, and a period, s, over which it ramps up
    /// and back down as one half sine cycle (a simple, boundary-free
    /// stand-in for a full Dryden/von Karman turbulence spectrum -- GENERIC,
    /// enough to exercise gust-load and gust-upset test cases without a
    /// random-process model this crate has no `rand` dependency for).
    pub gust_peak_m_s: f64,
    pub gust_period_s: f64,
    /// Elapsed time into the current gust cycle, s; the caller
    /// (`FlightModel::step`) advances this every tick.
    pub gust_phase_s: f64,
}

impl Wind {
    /// World-frame wind at `height_agl_m`, including shear and the gust's
    /// current phase. `height_agl_m <= 0` (below the reference) does not
    /// divide by zero or go negative: the power law is clamped to a 1 m
    /// floor, matching the usual boundary-layer modelling convention.
    pub fn at_height(&self, height_agl_m: f64) -> Vec3 {
        let h = height_agl_m.max(1.0);
        let shear = (h / 10.0).powf(self.shear_exponent);
        let steady = self.steady_10m.scale(shear);
        let dir = self.steady_10m.normalized_or(Vec3::new(1.0, 0.0, 0.0));
        let phase = if self.gust_period_s > 1e-6 { (self.gust_phase_s / self.gust_period_s).clamp(0.0, 1.0) } else { 0.0 };
        let gust = dir.scale(self.gust_peak_m_s * (std::f64::consts::PI * phase).sin());
        steady.add(gust)
    }

    /// Advances the gust clock, wrapping at `gust_period_s` (a repeating
    /// gust train) so a scenario can just hold `gust_peak_m_s` constant.
    pub fn advance(&mut self, dt_s: f64) {
        self.gust_phase_s += dt_s.max(0.0);
        if self.gust_period_s > 1e-6 && self.gust_phase_s > self.gust_period_s {
            self.gust_phase_s %= self.gust_period_s;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sea_level_isa_matches_the_standard_reference_values() {
        let a = isa(0.0, 0.0, 0.0);
        assert!((a.temp_k - 288.15).abs() < 1e-9);
        assert!((a.pressure_pa - 101_325.0).abs() < 1e-6);
        assert!((a.density_kg_m3 - 1.225).abs() < 0.001);
        assert!((a.sound_speed_m_s - 340.3).abs() < 0.5);
    }

    #[test]
    fn cruise_altitude_is_colder_and_thinner() {
        let cruise = isa(11_000.0, 0.0, 0.0);
        // ISA at the tropopause: -56.5 C, ~226.3 hPa.
        assert!((cruise.temp_k - 216.65).abs() < 0.1);
        assert!(cruise.pressure_pa < 25_000.0 && cruise.pressure_pa > 21_000.0);
        assert!(cruise.density_kg_m3 < 0.5);
    }

    #[test]
    fn no_altitude_or_offset_produces_a_nan_or_negative_density() {
        for alt in [-500.0, 0.0, 5000.0, 11_000.0, 15_000.0, 20_000.0] {
            for offset in [-40.0, 0.0, 40.0] {
                let a = isa(alt, offset, -5000.0);
                assert!(a.density_kg_m3.is_finite() && a.density_kg_m3 >= 0.0, "alt {alt} offset {offset}");
                assert!(a.sound_speed_m_s.is_finite() && a.sound_speed_m_s > 0.0);
            }
        }
    }

    #[test]
    fn shear_grows_wind_with_height_and_gust_peaks_at_half_period() {
        let mut w = Wind { steady_10m: Vec3::new(10.0, 0.0, 0.0), shear_exponent: 0.14, gust_peak_m_s: 5.0, gust_period_s: 10.0, gust_phase_s: 0.0 };
        let low = w.at_height(2.0).norm();
        let high = w.at_height(200.0).norm();
        assert!(high > low, "wind should build with height under positive shear");
        w.advance(5.0); // half the gust period
        let gusting = w.at_height(10.0).norm();
        assert!(gusting > 14.0, "gust should have added close to its full peak at mid-cycle: {gusting}");
    }

    #[test]
    fn zero_wind_never_produces_nan_direction() {
        let w = Wind::default();
        let v = w.at_height(50.0);
        assert!(v.x.is_finite() && v.y.is_finite() && v.z.is_finite());
    }
}
