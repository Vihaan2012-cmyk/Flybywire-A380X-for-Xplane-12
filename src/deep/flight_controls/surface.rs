//! One control surface: its rigid body about the hinge, driven by one or
//! more `actuator::PowerControlUnit`s and loaded by `hinge_moment`'s
//! aerodynamics, integrated exactly the way FlyByWire's own
//! `LinearActuatedRigidBodyOnHingeAxis` + `HydraulicLinearActuatorAssembly`
//! pairing does (fbw-common .../hydraulic/linear_actuator.rs) but reduced to
//! a single rotational degree of freedom (hinge angle), which is all a
//! flight model needs back.
//!
//! Blow-back is not a separate rule: it falls out of the torque balance.
//! Each actuator returns a torque capped at its own force limit; if the sum
//! of those caps is smaller than the aerodynamic hinge moment, the net
//! torque on the surface is dominated by the aerodynamics and the surface
//! moves away from its commanded angle no matter how hard the servo loop
//! tries -- exactly a real PCU stalling under airload.
//!
//! A surface disconnect (linkage between the PCUs and the surface sheared)
//! removes 100% of actuator torque but not the hinge aerodynamics or the
//! surface's own structural damping, so a disconnected surface free-floats
//! and weathervanes rather than freezing. A flutter damper is a small
//! dedicated damper independent of the PCUs (present on ailerons, rudders
//! and spoilers on real transports specifically because a PCU in `Damping`
//! or `Standby` mode, or a disconnected one, no longer provides enough rate
//! feedback on its own); losing it is modelled as a loss of damping that can
//! turn net damping negative at high dynamic pressure, the same
//! energy-balance sign flip that causes real classical flutter (a reduced
//! order proxy for the destabilising aerodynamic term real flutter analysis
//! gets from unsteady/CFD data this crate does not have -- GENERIC).

use super::actuator::{servo_rate_step, ActuatorFaults, ActuatorMode, PowerControlUnit, ServoLoad};
use super::hinge_moment::{hinge_moment_nm, HingeMomentCoefficients};

/// Default critical Mach number for an A380 lifting surface, derived from
/// the aircraft's own published MMO rather than picked.
///
/// MMO = M 0.89 (FlyByWire's own `fbw-a380x/src/systems/shared/src/
/// PerformanceConstants.ts:2`, `export const Mmo = 0.89`, matching the
/// figure in EASA TCDS A.110's operating limitations).
///
/// An aircraft is not certified to a maximum operating Mach that sits
/// inside its own transonic drag rise, so the wing's drag-divergence Mach
/// `M_dd` must be at least MMO. W. H. Mason, *Transonic Aerodynamics of
/// Airfoils and Wings* (Configuration Aerodynamics notes, Virginia Tech,
/// section 7), equates Lock's empirical drag rise `C_D = 20 (M - M_crit)^4`
/// with the standard `dC_D/dM = 0.1` definition of drag divergence and gets
///
///   M_crit = M_dd - (0.1/80)^(1/3)
///
/// with `(0.1/80)^(1/3) = 0.00125^(1/3) = 0.107722`. Taking the binding
/// case `M_dd = MMO`:
///
///   M_crit >= 0.89 - 0.107722 = 0.782278
///
/// so 0.7823 is a **lower bound** on the wing's critical Mach implied by
/// the A380's own MMO, not a measured section figure -- Airbus publishes
/// neither the A380's t/c distribution nor its `M_crit`, so the Korn
/// equation cannot be closed for this wing. It replaces an earlier
/// unsourced 0.75, which would have put `M_dd` at 0.858, i.e. *below* MMO
/// and below the published M 0.85 long-range cruise Mach -- self-evidently
/// wrong for this aircraft. Using the bound is the conservative direction:
/// compressibility keeps building to a slightly higher Mach than a lower
/// value would allow, and no surface sees an invented early fall-off.
pub const DEFAULT_MACH_CRIT: f64 = 0.7823;

/// This tick's aerodynamic environment at the surface.
#[derive(Clone, Copy, Debug)]
pub struct AeroInputs {
    pub dynamic_pressure_pa: f64,
    pub alpha_rad: f64,
    pub mach: f64,
    pub mach_crit: f64,
}

impl Default for AeroInputs {
    fn default() -> Self {
        Self { dynamic_pressure_pa: 0.0, alpha_rad: 0.0, mach: 0.0, mach_crit: DEFAULT_MACH_CRIT }
    }
}

/// Surface-level (as opposed to per-actuator) faults.
#[derive(Clone, Copy, Debug, Default)]
pub struct SurfaceFaults {
    /// The mechanical link between every PCU and the surface has sheared:
    /// no actuator torque reaches it at all, and it free-floats under
    /// aerodynamics and structural damping alone.
    pub disconnected: bool,
    /// 0 healthy .. 1 the dedicated flutter/gust damper has failed.
    pub flutter_damper_loss: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct SurfaceLimits {
    pub min_rad: f64,
    pub max_rad: f64,
}

/// GENERIC structural and flutter-damping figures for one surface type,
/// chosen so a healthy surface stays positively damped out to a dynamic
/// pressure well past normal cruise, while losing the dedicated damper pulls
/// the flutter onset dynamic pressure down into the ordinary flight
/// envelope -- the entire reason those dampers are flight-critical parts.
#[derive(Clone, Copy, Debug)]
pub struct SurfaceDamping {
    pub structural_nm_s_per_rad_s: f64,
    pub flutter_damper_nm_s_per_rad_s: f64,
    /// The destabilising aerodynamic term's strength, N*m per (rad/s) per Pa
    /// of dynamic pressure.
    pub flutter_drive_nm_s_per_rad_s_per_pa: f64,
}

impl SurfaceDamping {
    pub fn aileron() -> Self {
        Self { structural_nm_s_per_rad_s: 200.0, flutter_damper_nm_s_per_rad_s: 5000.0, flutter_drive_nm_s_per_rad_s_per_pa: 0.02 }
    }
    pub fn elevator() -> Self {
        Self { structural_nm_s_per_rad_s: 400.0, flutter_damper_nm_s_per_rad_s: 9000.0, flutter_drive_nm_s_per_rad_s_per_pa: 0.035 }
    }
    pub fn rudder() -> Self {
        Self { structural_nm_s_per_rad_s: 350.0, flutter_damper_nm_s_per_rad_s: 8000.0, flutter_drive_nm_s_per_rad_s_per_pa: 0.03 }
    }
    pub fn spoiler() -> Self {
        Self { structural_nm_s_per_rad_s: 60.0, flutter_damper_nm_s_per_rad_s: 1200.0, flutter_drive_nm_s_per_rad_s_per_pa: 0.01 }
    }
}

/// Standard uniform-rod-about-one-end moment of inertia, `I = m*c^2/3`, the
/// textbook approximation for a thin plate rotating about its own leading
/// edge (e.g. Hibbeler, "Engineering Mechanics: Dynamics", table of common
/// shapes) -- used because no A380 control-surface inertia data is public.
pub fn inertia_uniform_plate_kg_m2(mass_kg: f64, chord_m: f64) -> f64 {
    mass_kg.max(0.0) * chord_m.max(0.0).powi(2) / 3.0
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SurfaceOutput {
    pub angle_rad: f64,
    pub rate_rad_s: f64,
    pub hinge_moment_nm: f64,
    pub actuator_torque_nm: f64,
    pub actuator_capacity_nm: f64,
    pub at_stop: bool,
    /// True when the actuators' combined torque ceiling is below what the
    /// aerodynamics demand and the surface has drifted meaningfully off its
    /// commanded angle as a result.
    pub blown_back: bool,
}

/// One surface driven by `N` power control units.
pub struct ControlSurface<const N: usize> {
    pcus: [PowerControlUnit; N],
    hinge: HingeMomentCoefficients,
    inertia_kg_m2: f64,
    limits: SurfaceLimits,
    damping: SurfaceDamping,
    angle_rad: f64,
    rate_rad_s: f64,
}

impl<const N: usize> ControlSurface<N> {
    pub fn new(
        pcus: [PowerControlUnit; N],
        hinge: HingeMomentCoefficients,
        inertia_kg_m2: f64,
        limits: SurfaceLimits,
        damping: SurfaceDamping,
        initial_angle_rad: f64,
    ) -> Self {
        Self {
            pcus,
            hinge,
            inertia_kg_m2: inertia_kg_m2.max(1e-3),
            limits,
            damping,
            angle_rad: initial_angle_rad.clamp(limits.min_rad, limits.max_rad),
            rate_rad_s: 0.0,
        }
    }

    pub fn angle_rad(&self) -> f64 {
        self.angle_rad
    }

    pub fn rate_rad_s(&self) -> f64 {
        self.rate_rad_s
    }

    /// The standby-mode and jam springs in `actuator::PowerControlUnit` are
    /// stiff enough (chosen to be a very rigid trapped-fluid/seizure model)
    /// that integrating them at a typical simulation tick (10-20 ms) with
    /// this struct's explicit (semi-implicit Euler) integrator would not be
    /// numerically stable. Semi-implicit Euler on an undamped harmonic
    /// oscillator is stable exactly while `omega*dt <= 2`; this sub-step is
    /// chosen well inside that bound for the stiffest constant this crate
    /// uses (aileron standby spring vs. its own inertia, `omega` ~ 210
    /// rad/s), per the crate convention "sub-stepping where stiff".
    ///
    /// The actuators' *damping* terms are far stiffer still (the inner rate
    /// loop alone is `c/I` ~ 3000 s^-1, which no affordable sub-step can
    /// resolve explicitly) and are therefore not sub-stepped at all but
    /// solved exactly by `actuator::servo_rate_step`; this bound only has
    /// to cover the springs.
    const MAX_SUBSTEP_S: f64 = 0.0005;

    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        modes: [ActuatorMode; N],
        commanded_angle_rad: f64,
        pressure_fractions: [f64; N],
        actuator_faults: [ActuatorFaults; N],
        surface_faults: &SurfaceFaults,
        aero: &AeroInputs,
        dt_s: f64,
    ) -> SurfaceOutput {
        let dt_total = dt_s.max(0.0);
        let n = ((dt_total / Self::MAX_SUBSTEP_S).ceil() as usize).max(1);
        let sub_dt = dt_total / n as f64;
        let mut out = SurfaceOutput::default();
        for _ in 0..n {
            out = self.step_once(modes, commanded_angle_rad, pressure_fractions, actuator_faults, surface_faults, aero, sub_dt);
        }
        out
    }

    #[allow(clippy::too_many_arguments)]
    fn step_once(
        &mut self,
        modes: [ActuatorMode; N],
        commanded_angle_rad: f64,
        pressure_fractions: [f64; N],
        actuator_faults: [ActuatorFaults; N],
        surface_faults: &SurfaceFaults,
        aero: &AeroInputs,
        dt: f64,
    ) -> SurfaceOutput {
        let mut actuator_torque = 0.0;
        let mut actuator_capacity = 0.0;
        let mut servo = ServoLoad::NONE;
        let mut jam_damping = 0.0;
        let mut any_saturated = false;
        for i in 0..N {
            let out = self.pcus[i].step(
                modes[i],
                commanded_angle_rad,
                self.angle_rad,
                self.rate_rad_s,
                pressure_fractions[i],
                &actuator_faults[i],
            );
            actuator_capacity += out.max_torque_nm;
            any_saturated |= out.saturated;
            if !surface_faults.disconnected {
                actuator_torque += out.torque_nm;
                // A sheared linkage transmits no torque *law* either, so a
                // disconnected surface's integrator must not see one.
                servo.add(&out.servo);
                jam_damping += out.jam_damping_nm_s_per_rad;
            }
        }

        let hinge_m =
            hinge_moment_nm(&self.hinge, self.angle_rad, aero.alpha_rad, aero.dynamic_pressure_pa, aero.mach, aero.mach_crit);

        let flutter_damper =
            self.damping.flutter_damper_nm_s_per_rad_s * (1.0 - surface_faults.flutter_damper_loss.clamp(0.0, 1.0));
        let destabilizing = self.damping.flutter_drive_nm_s_per_rad_s_per_pa * aero.dynamic_pressure_pa.max(0.0);
        let net_damping = self.damping.structural_nm_s_per_rad_s + flutter_damper - destabilizing;
        let damping_torque = -net_damping * self.rate_rad_s;

        // Everything except the actuators' own clamped servo law is
        // ordinary linear load on the body; the servo law goes in whole, so
        // its clamp is solved rather than stepped across (see
        // `actuator::servo_rate_step`).
        let other_torque = actuator_torque - servo.torque_at(self.rate_rad_s) + hinge_m + damping_torque;
        self.rate_rad_s = servo_rate_step(
            self.rate_rad_s,
            &servo,
            other_torque,
            jam_damping + net_damping,
            self.inertia_kg_m2,
            dt,
        );
        self.angle_rad += self.rate_rad_s * dt;

        let mut at_stop = false;
        if self.angle_rad <= self.limits.min_rad {
            self.angle_rad = self.limits.min_rad;
            if self.rate_rad_s < 0.0 {
                self.rate_rad_s = 0.0;
            }
            at_stop = true;
        } else if self.angle_rad >= self.limits.max_rad {
            self.angle_rad = self.limits.max_rad;
            if self.rate_rad_s > 0.0 {
                self.rate_rad_s = 0.0;
            }
            at_stop = true;
        }

        let blown_back =
            !surface_faults.disconnected && any_saturated && (self.angle_rad - commanded_angle_rad).abs() > 0.02;

        SurfaceOutput {
            angle_rad: self.angle_rad,
            rate_rad_s: self.rate_rad_s,
            hinge_moment_nm: hinge_m,
            actuator_torque_nm: actuator_torque,
            actuator_capacity_nm: actuator_capacity,
            at_stop,
            blown_back,
        }
    }
}

/// A debounced position-mismatch detector, the same causal role as the
/// A380's SFCC asymmetry monitors: it only trips once two surfaces (e.g.
/// left/right flaps) have disagreed by more than `threshold_rad` for at
/// least `timer_s`, so a brief transient during normal motion does not
/// falsely trigger a wingtip brake.
#[derive(Clone, Copy, Debug)]
pub struct AsymmetryMonitor {
    timer_s: f64,
    elapsed_s: f64,
    tripped: bool,
}

impl AsymmetryMonitor {
    pub fn new(timer_s: f64) -> Self {
        Self { timer_s: timer_s.max(0.0), elapsed_s: 0.0, tripped: false }
    }

    pub fn step(&mut self, a_rad: f64, b_rad: f64, threshold_rad: f64, dt_s: f64) -> bool {
        if (a_rad - b_rad).abs() > threshold_rad.max(0.0) {
            self.elapsed_s += dt_s.max(0.0);
        } else {
            self.elapsed_s = 0.0;
            self.tripped = false;
        }
        if self.elapsed_s >= self.timer_s {
            self.tripped = true;
        }
        self.tripped
    }

    pub fn tripped(&self) -> bool {
        self.tripped
    }
}

#[cfg(test)]
mod tests {
    use super::super::actuator::ActuatorGeometry;
    use super::*;

    fn aileron_surface() -> ControlSurface<2> {
        ControlSurface::new(
            [PowerControlUnit::new(ActuatorGeometry::aileron()), PowerControlUnit::new(ActuatorGeometry::aileron())],
            HingeMomentCoefficients::aileron(),
            inertia_uniform_plate_kg_m2(118.0, 1.4), // FBW's middle-panel mass, mod.rs:456
            SurfaceLimits { min_rad: -20.0_f64.to_radians(), max_rad: 30.0_f64.to_radians() },
            SurfaceDamping::aileron(),
            0.0,
        )
    }

    #[test]
    fn no_nan_at_rest_or_zero_dt() {
        let mut s = aileron_surface();
        let out = s.step(
            [ActuatorMode::Active; 2],
            0.0,
            [1.0; 2],
            [ActuatorFaults::default(); 2],
            &SurfaceFaults::default(),
            &AeroInputs::default(),
            0.0,
        );
        assert!(out.angle_rad.is_finite() && out.rate_rad_s.is_finite());
    }

    #[test]
    fn a_healthy_surface_tracks_its_command_at_low_dynamic_pressure() {
        let mut s = aileron_surface();
        let target = 0.2_f64;
        let mut out = SurfaceOutput::default();
        for _ in 0..20_000 {
            out = s.step(
                [ActuatorMode::Active; 2],
                target,
                [1.0; 2],
                [ActuatorFaults::default(); 2],
                &SurfaceFaults::default(),
                &AeroInputs { dynamic_pressure_pa: 500.0, ..Default::default() },
                0.01,
            );
        }
        assert!((out.angle_rad - target).abs() < 0.01, "settled at {}", out.angle_rad);
        assert!(!out.blown_back);
    }

    #[test]
    fn overwhelming_hinge_moment_blows_the_surface_back_off_command() {
        let mut s = aileron_surface();
        // Both actuators starved of hydraulic power: near-zero torque
        // ceiling. A big commanded deflection at high dynamic pressure
        // should fail to track.
        let faults = ActuatorFaults { supply_loss: 0.98, ..Default::default() };
        let mut out = SurfaceOutput::default();
        for _ in 0..5000 {
            out = s.step(
                [ActuatorMode::Active; 2],
                0.4,
                [1.0; 2],
                [faults; 2],
                &SurfaceFaults::default(),
                &AeroInputs { dynamic_pressure_pa: 20000.0, alpha_rad: 0.05, mach: 0.6, mach_crit: DEFAULT_MACH_CRIT },
                0.01,
            );
        }
        assert!(out.blown_back);
        assert!((out.angle_rad - 0.4).abs() > 0.05);
    }

    #[test]
    fn a_disconnected_surface_free_floats_under_aerodynamics_alone() {
        let mut s = aileron_surface();
        let faults = SurfaceFaults { disconnected: true, ..Default::default() };
        let mut out = SurfaceOutput::default();
        for _ in 0..5000 {
            out = s.step(
                [ActuatorMode::Active; 2],
                0.3,
                [1.0; 2],
                [ActuatorFaults::default(); 2],
                &faults,
                &AeroInputs { dynamic_pressure_pa: 5000.0, alpha_rad: 0.02, mach: 0.4, mach_crit: DEFAULT_MACH_CRIT },
                0.01,
            );
        }
        assert_eq!(out.actuator_torque_nm, 0.0);
        // A restoring (Ch_delta < 0) surface with no actuator torque
        // settles near zero deflection, not at the ignored command.
        assert!(out.angle_rad.abs() < 0.05, "free-floated to {}", out.angle_rad);
    }

    #[test]
    fn mechanical_stops_halt_the_surface_and_zero_its_rate_into_the_stop() {
        let mut s = aileron_surface();
        let mut out = SurfaceOutput::default();
        for _ in 0..20_000 {
            out = s.step(
                [ActuatorMode::Active; 2],
                10.0, // way past the +30 deg travel limit
                [1.0; 2],
                [ActuatorFaults::default(); 2],
                &SurfaceFaults::default(),
                &AeroInputs::default(),
                0.01,
            );
        }
        assert!(out.at_stop);
        assert!((out.angle_rad - 30.0_f64.to_radians()).abs() < 1e-6);
        assert_eq!(out.rate_rad_s, 0.0);
    }

    #[test]
    fn losing_the_flutter_damper_at_high_dynamic_pressure_grows_oscillation() {
        // Healthy: perturb the surface and expect the oscillation to decay.
        let mut healthy = aileron_surface();
        let mut faulty = aileron_surface();
        let q = 15000.0; // within this test's tuned instability threshold
        let aero = AeroInputs { dynamic_pressure_pa: q, alpha_rad: 0.0, mach: 0.5, mach_crit: DEFAULT_MACH_CRIT };
        let damping_only = [ActuatorMode::Damping; 2];
        let no_faults = [ActuatorFaults::default(); 2];
        let damper_lost = SurfaceFaults { flutter_damper_loss: 1.0, ..Default::default() };

        // Kick both the same way, then let them run with the PCUs' linkage
        // sheared, which is the condition this module's doc comment names as
        // the one the dedicated flutter damper exists for. It has to be that
        // condition and not merely `Damping`-mode PCUs: a PCU in `Damping`
        // is still a 24 kN*m*s/rad damper (`k_damping = max_torque /
        // rated_rate` = 16575/0.682 for an aileron, times two units), five
        // times the flutter damper's own 5 kN*m*s/rad, so with the PCUs
        // attached the net damping stays hugely positive whether the flutter
        // damper is there or not and nothing is being tested. Disconnected,
        // the damping budget is exactly what the model claims it is:
        //   healthy: 200 (structural) + 5000 (damper) - 0.02*15000 (the
        //            destabilising aero term) = +4800 N*m*s/rad -> decays
        //   faulty:  200 + 0 - 300 = -100 N*m*s/rad -> grows, at
        //            exp(100/(2*77.1) * t) = exp(0.65 t) over 15 s
        // i.e. the sign flip the module doc describes, from the damper alone.
        let disconnected = SurfaceFaults { disconnected: true, ..Default::default() };
        let disconnected_damper_lost = SurfaceFaults { disconnected: true, ..damper_lost };
        healthy.angle_rad = 0.05;
        faulty.angle_rad = 0.05;
        let mut healthy_peak = 0.0_f64;
        let mut faulty_peak = 0.0_f64;
        for _ in 0..3000 {
            let ho = healthy.step(damping_only, 0.0, [1.0; 2], no_faults, &disconnected, &aero, 0.005);
            let fo = faulty.step(damping_only, 0.0, [1.0; 2], no_faults, &disconnected_damper_lost, &aero, 0.005);
            healthy_peak = healthy_peak.max(ho.angle_rad.abs());
            faulty_peak = faulty_peak.max(fo.angle_rad.abs());
        }
        assert!(healthy.angle_rad.abs() < 0.05, "healthy should have decayed: {}", healthy.angle_rad);
        assert!(faulty_peak > healthy_peak * 2.0, "faulty should grow relative to healthy: {faulty_peak} vs {healthy_peak}");
    }

    #[test]
    fn asymmetry_monitor_needs_both_a_gap_and_time_before_it_trips() {
        let mut m = AsymmetryMonitor::new(1.0);
        // A brief mismatch under the timer should not trip it.
        for _ in 0..50 {
            assert!(!m.step(0.3, 0.0, 0.05, 0.01)); // 0.5 s of mismatch
        }
        let mut m2 = AsymmetryMonitor::new(0.2);
        let mut tripped = false;
        for _ in 0..40 {
            tripped = m2.step(0.3, 0.0, 0.05, 0.01); // 0.4 s of mismatch
        }
        assert!(tripped);
        // Once it agrees again, it resets.
        assert!(!m2.step(0.1, 0.1, 0.05, 0.01));
    }
}
