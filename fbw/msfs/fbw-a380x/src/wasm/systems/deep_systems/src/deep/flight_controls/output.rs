use std::f64::consts::PI;

const RAD_DEG: f64 = 180.0 / PI;

#[derive(Clone, Copy, Debug, Default)]
pub struct SurfaceState {
    pub angle_deg: f64,
    pub rate_deg_s: f64,
    pub hinge_torque_nm: f64,
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
        Self { angle_deg: o.angle_rad * RAD_DEG, rate_deg_s: o.rate_rad_s * RAD_DEG, hinge_torque_nm: o.hinge_moment_nm, blown_back: o.at_stop }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct FlightControlOutputs {
    pub ailerons: [[SurfaceState; 3]; 2],
    pub elevators: [[SurfaceState; 2]; 2],
    pub rudders: [SurfaceState; 2],
    pub spoilers: [[SurfaceState; 8]; 2],
    pub ths: SurfaceState,
    pub rudder_trim_deg: f64,
    pub flaps: [[f64; 2]; 2],
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
