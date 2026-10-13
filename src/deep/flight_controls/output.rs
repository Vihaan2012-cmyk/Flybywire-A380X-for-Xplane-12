//! What a flight model needs back from this whole area: every surface's
//! position (matching the normalised/degree conventions the crate's own
//! `flight_controls.rs` already documents and consumes) plus the hinge
//! force/torque a structural or force-feedback model would want, gathered
//! in one place rather than each caller reaching into every sub-model.
//!
//! This module holds only plain data plus unit conversions; it takes no
//! dependency on `surface`/`high_lift`/`ths` beyond the field types, so any
//! of those can build one of these each tick without this module needing to
//! know how.

use std::f64::consts::PI;

const RAD_DEG: f64 = 180.0 / PI;

/// One surface's usable output: angle in both units (radians is what the
/// physics uses internally; degrees is what `flight_controls.rs`'s own
/// conversions, and X-Plane/FlyByWire's datarefs, are expressed in), rate
/// for rate-of-deflection-sensitive consumers (e.g. buffet/noise, or a
/// structural model), and the net hinge torque a loads model can turn into
/// stick force or structural load.
#[derive(Clone, Copy, Debug, Default)]
pub struct SurfaceState {
    pub angle_deg: f64,
    pub rate_deg_s: f64,
    pub hinge_torque_nm: f64,
    /// True while any actuator on this surface is saturated and the surface
    /// is meaningfully off its commanded position (see
    /// `surface::SurfaceOutput::blown_back`).
    pub blown_back: bool,
}

impl SurfaceState {
    pub fn from_surface_output(o: &super::surface::SurfaceOutput) -> Self {
        Self {
            angle_deg: o.angle_rad * RAD_DEG,
            rate_deg_s: o.rate_rad_s * RAD_DEG,
            hinge_torque_nm: o.hinge_moment_nm,
            blown_back: o.blown_back,
        }
    }

    pub fn from_ths_output(o: &super::ths::ThsOutput) -> Self {
        // `ThsOutput` carries no saturated/drifted flag of its own (unlike
        // `surface::SurfaceOutput::blown_back`, which is
        // `saturated && drifted off command`); `at_stop` -- the screw
        // against one of its travel limits -- is the nearest real signal
        // this module has, and is a strict improvement on the hardcoded
        // `false` that could never be true no matter what the THS did. It
        // is not a perfect match: a THS legitimately trimmed to full
        // authority also reads `at_stop` with nothing wrong at all.
        Self { angle_deg: o.angle_rad * RAD_DEG, rate_deg_s: o.rate_rad_s * RAD_DEG, hinge_torque_nm: o.hinge_moment_nm, blown_back: o.at_stop }
    }
}

/// Every A380 flight-control surface this area models, for one wing's worth
/// of ailerons/spoilers plus the shared tail surfaces. Indexing matches
/// `flight_controls.rs`'s own `[side][...]` convention (0 = left, 1 =
/// right) so a future wiring pass can fill `flight_controls::Actuators`
/// straight from this.
#[derive(Clone, Copy, Debug, Default)]
pub struct FlightControlOutputs {
    /// [side][inward, middle, outward].
    pub ailerons: [[SurfaceState; 3]; 2],
    /// [side][inward, outward].
    pub elevators: [[SurfaceState; 2]; 2],
    /// [upper, lower].
    pub rudders: [SurfaceState; 2],
    /// [side][spoiler 1..8].
    pub spoilers: [[SurfaceState; 8]; 2],
    pub ths: SurfaceState,
    /// Rudder trim bias, degrees (added to the rudder's own commanded
    /// deflection upstream of this crate's actuator model, or reported
    /// separately for a display).
    pub rudder_trim_deg: f64,
    /// [side][inboard, outboard]: flap high-lift stations.
    pub flaps: [[f64; 2]; 2],
    /// [side][inboard, outboard]: slat high-lift stations.
    pub slats: [[f64; 2]; 2],
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::surface::SurfaceOutput;
    use super::super::ths::ThsOutput;

    #[test]
    fn conversions_are_exact_and_finite_at_zero() {
        let s = SurfaceState::from_surface_output(&SurfaceOutput::default());
        assert_eq!(s.angle_deg, 0.0);
        assert!(s.angle_deg.is_finite() && s.rate_deg_s.is_finite());

        let o = SurfaceOutput { angle_rad: PI / 2.0, rate_rad_s: PI, hinge_moment_nm: 123.0, ..Default::default() };
        let s = SurfaceState::from_surface_output(&o);
        assert!((s.angle_deg - 90.0).abs() < 1e-9);
        assert!((s.rate_deg_s - 180.0).abs() < 1e-9);
        assert_eq!(s.hinge_torque_nm, 123.0);

        let t = ThsOutput { angle_rad: -PI / 18.0, ..Default::default() };
        let ts = SurfaceState::from_ths_output(&t);
        assert!((ts.angle_deg + 10.0).abs() < 1e-6);
    }

    #[test]
    fn ths_blown_back_tracks_at_stop_instead_of_being_hardcoded() {
        let running = ThsOutput { at_stop: false, ..Default::default() };
        assert!(!SurfaceState::from_ths_output(&running).blown_back);

        let stopped = ThsOutput { at_stop: true, ..Default::default() };
        assert!(SurfaceState::from_ths_output(&stopped).blown_back, "at_stop must reach blown_back, not a hardcoded false");
    }

    #[test]
    fn default_outputs_are_all_finite() {
        let out = FlightControlOutputs::default();
        for side in out.ailerons {
            for a in side {
                assert!(a.angle_deg.is_finite());
            }
        }
        assert!(out.ths.angle_deg.is_finite());
    }
}
