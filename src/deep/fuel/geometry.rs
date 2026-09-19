//! Per-tank fuel geometry and attitude effects (backlog item 1): free-surface
//! tilt under pitch/roll/acceleration, usable-vs-unusable fuel, pump/probe
//! inlet unporting and a sloshing settle time constant, modelled per tank
//! instead of with one plugin-wide shape.
//!
//! The existing plugin-wide model already does real attitude physics for a
//! *single* generic tank shape: `src/physics/fluids.rs::unporting_factor`
//! and `tank_tilt_fraction` take one `aspect` ratio for whichever tank calls
//! them, and `src/fuel.rs` in turn calls them with one constant,
//! `TANK_ASPECT_RATIO = 6.0` (`fuel.rs:255`), for all eleven tanks alike. The
//! real A380 tanks are not the same shape: the trim tank is a shallow, wide
//! box spanning the tailplane box far aft of the wing tanks; the outer wing
//! tanks are long, thin and highly tapered near the tip; the feed tanks sit
//! low and central under the fuselage. This module gives each of the
//! eleven tanks its own `TankShape` (aspect ratio, and bank/pitch
//! sensitivity multipliers relative to the wing-box default) so the same
//! pitch/bank/acceleration produces a different free-surface tilt, unporting
//! margin and usable fraction per tank -- a CL650-style per-tank attitude
//! map -- and adds a sloshing time constant the existing model has no
//! equivalent of at all.
//!
//! Capacities are FlyByWire's own published values
//! (`fbw-a380x/.../flight_model.cfg` lines 142-152, `[FUEL_SYSTEM]`
//! `Tank.1`..`Tank.11`, `Capacity` in US gallons); their sum, 85 471.7 US gal
//! = 323 511 L, matches the A380-800's publicly quoted ~323 546 L maximum
//! fuel capacity (Airbus A380 aircraft characteristics / airport planning
//! document) to within rounding, confirming these are real tank sizes, not
//! placeholders. Every other shape parameter below (aspect ratio,
//! bank/pitch sensitivity, inlet margin) is `GENERIC`: no public source
//! gives the A380's internal tank rib/baffle geometry, so these are
//! engineering-judgement box approximations from each tank's known role and
//! position, in the same spirit as `physics::fluids`'s own `GENERIC`
//! `TANK_ASPECT_RATIO`/`PUMP_SUBMERSION_MARGIN_FRACTION`.

use std::f64::consts::PI;

/// Standard gravity, m/s^2.
pub const G: f64 = 9.80665;
/// US gallons to cubic metres (1 US gal = 231 in^3 exactly), matching
/// `crate::fuel_network::GAL_TO_M3` and `crate::fuel`'s own constant of the
/// same value (kept local per this directory's self-containment rule).
pub const GAL_TO_M3: f64 = 0.003785411784;

/// The eleven real fuel tanks (FlyByWire `Tank.1`..`Tank.11`), by their cfg
/// index. Numbered exactly as `flight_model.cfg`'s `Tank.N` so a failure or
/// component id naming a tank number matches the network's own.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Tank {
    LeftOuter = 1,
    Feed1 = 2,
    LeftMid = 3,
    LeftInner = 4,
    Feed2 = 5,
    Feed3 = 6,
    RightInner = 7,
    RightMid = 8,
    Feed4 = 9,
    RightOuter = 10,
    Trim = 11,
}
pub const ALL_TANKS: [Tank; 11] =
    [Tank::LeftOuter, Tank::Feed1, Tank::LeftMid, Tank::LeftInner, Tank::Feed2, Tank::Feed3, Tank::RightInner, Tank::RightMid, Tank::Feed4, Tank::RightOuter, Tank::Trim];

impl Tank {
    pub fn name(self) -> &'static str {
        match self {
            Tank::LeftOuter => "LeftOuter",
            Tank::Feed1 => "Feed1",
            Tank::LeftMid => "LeftMid",
            Tank::LeftInner => "LeftInner",
            Tank::Feed2 => "Feed2",
            Tank::Feed3 => "Feed3",
            Tank::RightInner => "RightInner",
            Tank::RightMid => "RightMid",
            Tank::Feed4 => "Feed4",
            Tank::RightOuter => "RightOuter",
            Tank::Trim => "Trim",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TankKind {
    Outer,
    Mid,
    Inner,
    Feed,
    Trim,
}

/// One tank's box-approximation shape and attitude sensitivity. See the
/// module doc for what is sourced (capacity) and what is `GENERIC`.
#[derive(Clone, Copy, Debug)]
pub struct TankShape {
    pub tank: Tank,
    pub kind: TankKind,
    /// `flight_model.cfg` `Tank.N` `Capacity`, US gallons.
    pub capacity_gal: f64,
    /// `GENERIC`: length-to-depth ratio of the box approximation (a long,
    /// shallow outer tank has a high ratio; a squarer feed tank a low one).
    pub aspect_ratio: f64,
    /// `GENERIC`: multiplier on bank-induced surface tilt relative to a
    /// central wing-box tank (1.0) -- above 1 for tanks whose long axis lies
    /// more nearly along the span (outer wing, trim-in-THS), below 1 for
    /// tanks oriented more fore-aft.
    pub bank_sensitivity: f64,
    /// `GENERIC`: multiplier on pitch-induced surface tilt, similarly
    /// relative to 1.0 -- the trim tank sits on the longest fuselage moment
    /// arm from the CG of any tank (aft in the tailplane box) so the same
    /// body pitch rate/attitude sweeps its surface through more of its own
    /// depth than a wing tank near the CG.
    pub pitch_sensitivity: f64,
    /// `GENERIC`: pump/probe inlet standpipe height as a fraction of box
    /// height (`physics::fluids::PUMP_SUBMERSION_MARGIN_FRACTION` uses one
    /// shared 0.15 for every tank; this lets a shallower tank have a
    /// tighter margin and a deeper one a looser margin, since the same
    /// absolute standpipe height is a larger fraction of a shallow tank).
    pub inlet_margin_fraction: f64,
    /// `GENERIC`: base unusable (trapped sump) fraction at wings-level,
    /// static attitude, independent of tilt. FlyByWire's own cfg sets every
    /// tank's `UnusableCapacity` to 0 (`flight_model.cfg:142-152`, an MSFS
    /// simplification); real wing tanks always trap a little fuel around
    /// ribs/baffles and the boost-pump sump, so this is a small non-zero
    /// figure -- a genuine deepening of a value FlyByWire's own model
    /// idealises to zero, not a duplicate of it.
    pub base_unusable_fraction: f64,
    /// `GENERIC`: slosh damping ratio (baffled/ribbed wing tanks settle much
    /// faster than a single unbaffled bay); used only by
    /// [`sloshing_settle_time_s`].
    pub slosh_damping_ratio: f64,
}

fn shape(tank: Tank) -> TankShape {
    use Tank::*;
    use TankKind::*;
    match tank {
        LeftOuter | RightOuter => TankShape {
            tank,
            kind: Outer,
            capacity_gal: 2731.5,
            aspect_ratio: 10.0,
            bank_sensitivity: 1.2,
            pitch_sensitivity: 0.6,
            inlet_margin_fraction: 0.20,
            base_unusable_fraction: 0.003,
            slosh_damping_ratio: 0.25,
        },
        LeftMid | RightMid => TankShape {
            tank,
            kind: Mid,
            capacity_gal: 9632.0,
            aspect_ratio: 7.0,
            bank_sensitivity: 1.0,
            pitch_sensitivity: 0.8,
            inlet_margin_fraction: 0.15,
            base_unusable_fraction: 0.002,
            slosh_damping_ratio: 0.22,
        },
        LeftInner | RightInner => TankShape {
            tank,
            kind: Inner,
            capacity_gal: 12189.4,
            aspect_ratio: 6.0,
            bank_sensitivity: 0.9,
            pitch_sensitivity: 0.9,
            inlet_margin_fraction: 0.15,
            base_unusable_fraction: 0.002,
            slosh_damping_ratio: 0.20,
        },
        Feed1 | Feed4 => TankShape {
            tank,
            kind: Feed,
            capacity_gal: 7299.6,
            aspect_ratio: 4.0,
            bank_sensitivity: 0.7,
            pitch_sensitivity: 1.0,
            inlet_margin_fraction: 0.10,
            base_unusable_fraction: 0.0015,
            slosh_damping_ratio: 0.18,
        },
        Feed2 | Feed3 => TankShape {
            tank,
            kind: Feed,
            capacity_gal: 7753.2,
            aspect_ratio: 4.0,
            bank_sensitivity: 0.7,
            pitch_sensitivity: 1.0,
            inlet_margin_fraction: 0.10,
            base_unusable_fraction: 0.0015,
            slosh_damping_ratio: 0.18,
        },
        Tank::Trim => TankShape {
            tank,
            kind: TankKind::Trim,
            capacity_gal: 6260.3,
            aspect_ratio: 14.0,
            bank_sensitivity: 1.3,
            pitch_sensitivity: 1.4,
            inlet_margin_fraction: 0.25,
            base_unusable_fraction: 0.004,
            slosh_damping_ratio: 0.10,
        },
    }
}

impl TankShape {
    pub fn of(tank: Tank) -> Self {
        shape(tank)
    }
    pub fn capacity_m3(&self) -> f64 {
        self.capacity_gal * GAL_TO_M3
    }
    /// Box height from `capacity = aspect * height^3` (a box whose
    /// cross-section is `height` square and whose length is
    /// `aspect * height` -- the same method
    /// `physics::fluids::tank_box_height_m` uses for its one shared shape,
    /// applied here per tank).
    pub fn box_height_m(&self) -> f64 {
        (self.capacity_m3() / self.aspect_ratio.max(1e-6)).cbrt()
    }
    pub fn box_length_m(&self) -> f64 {
        self.box_height_m() * self.aspect_ratio.max(1.0)
    }
}

/// The free surface's tilt as a fraction of box height, from body
/// pitch/bank *and* sustained lateral/longitudinal acceleration. An
/// accelerating tank sees an effective gravity vector tilted by
/// `atan(a/g)` off the body axis it acts along -- the same "equivalent
/// gravity" construction used for any free liquid surface in an
/// accelerating frame (elementary rigid-body mechanics; see e.g. Abramson,
/// NASA SP-106, "The Dynamic Behavior of Liquids in Moving Containers",
/// 1966, section II, for the fuel-slosh literature's own use of it) -- the
/// existing plugin-wide `tank_tilt_fraction` only takes attitude, not
/// acceleration, so a steady turn or a sustained shove today produces no
/// extra tilt there at all.
pub fn tilt_fraction(shape: &TankShape, pitch_deg: f64, bank_deg: f64, lateral_accel_g: f64, longitudinal_accel_g: f64) -> f64 {
    let h = shape.box_height_m();
    if h <= 0.0 {
        return 0.0;
    }
    let l = shape.box_length_m();
    let eff_bank = bank_deg.to_radians() + lateral_accel_g.atan();
    let eff_pitch = pitch_deg.to_radians() + longitudinal_accel_g.atan();
    let tilt_m = (l / 2.0) * eff_bank.tan().abs() * shape.bank_sensitivity + (h / 2.0) * eff_pitch.tan().abs() * shape.pitch_sensitivity;
    tilt_m / h
}

/// Pump/probe inlet unporting, 0 (dry at the inlet) .. 1 (fully submerged),
/// from this tank's own inlet margin instead of the shared 0.15 every tank
/// uses today (`physics::fluids::PUMP_SUBMERSION_MARGIN_FRACTION`).
pub fn unporting_factor(shape: &TankShape, fill_fraction: f64, pitch_deg: f64, bank_deg: f64, lateral_accel_g: f64, longitudinal_accel_g: f64) -> f64 {
    let fill = fill_fraction.clamp(0.0, 1.0);
    let tilt = tilt_fraction(shape, pitch_deg, bank_deg, lateral_accel_g, longitudinal_accel_g);
    let depth_at_inlet = fill - tilt;
    let margin = shape.inlet_margin_fraction.max(1e-6);
    (depth_at_inlet / margin).clamp(0.0, 1.0)
}

/// The fraction of nominal capacity that is *not* usable right now: the
/// tank's own static sump (`base_unusable_fraction`) plus fuel temporarily
/// stranded away from the inlet by the current tilt (the same geometry
/// [`unporting_factor`] ramps over, but expressed as a lost-fraction rather
/// than a pump-derate so a caller can show "unusable fuel" on a synoptic
/// independent of whether a pump is actually running).
pub fn unusable_fraction_now(shape: &TankShape, fill_fraction: f64, pitch_deg: f64, bank_deg: f64, lateral_accel_g: f64, longitudinal_accel_g: f64) -> f64 {
    let unporting = unporting_factor(shape, fill_fraction, pitch_deg, bank_deg, lateral_accel_g, longitudinal_accel_g);
    let attitude_loss = shape.inlet_margin_fraction.max(0.0) * (1.0 - unporting);
    (shape.base_unusable_fraction + attitude_loss).min(1.0)
}

/// The lowest-mode sloshing natural period, s, from shallow-water wave
/// theory for a rectangular tank of length `l` and liquid depth `d`:
/// `omega^2 = (pi*g/l) * tanh(pi*d/l)` (Abramson, NASA SP-106, eq. for the
/// fundamental antisymmetric sloshing mode of a rectangular tank -- the
/// standard first-order result cited throughout the aerospace slosh
/// literature). Returns `f64::INFINITY` for an empty tank (no free surface
/// to slosh).
pub fn sloshing_period_s(shape: &TankShape, fill_fraction: f64) -> f64 {
    let h = shape.box_height_m();
    let l = shape.box_length_m();
    let depth = (fill_fraction.clamp(0.0, 1.0) * h).max(0.0);
    if depth <= 1e-9 || l <= 0.0 {
        return f64::INFINITY;
    }
    let omega2 = (PI * G / l) * (PI * depth / l).tanh();
    if omega2 <= 0.0 {
        return f64::INFINITY;
    }
    2.0 * PI / omega2.sqrt()
}

/// First-order settle time constant for the surface returning to rest after
/// a disturbance, `tau = 1 / (zeta * omega)` for a lightly damped
/// oscillator (standard result: the envelope of a damped sinusoid decays as
/// `exp(-zeta*omega*t)`). Ribbed/baffled real wing tanks damp much faster
/// than an open tank of the same size (aviation-fuel-tank design texts
/// commonly cite baffles for exactly this reason); `slosh_damping_ratio` is
/// `GENERIC` per tank (see the struct doc).
pub fn sloshing_settle_time_s(shape: &TankShape, fill_fraction: f64) -> f64 {
    let period = sloshing_period_s(shape, fill_fraction);
    if !period.is_finite() {
        return 0.0;
    }
    let omega = 2.0 * PI / period;
    let zeta = shape.slosh_damping_ratio.max(1e-3);
    1.0 / (zeta * omega)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_tank_has_a_positive_finite_shape() {
        for &t in &ALL_TANKS {
            let s = TankShape::of(t);
            assert!(s.box_height_m() > 0.0, "{}", t.name());
            assert!(s.box_length_m() > s.box_height_m(), "{}", t.name());
            assert!(s.capacity_m3() > 0.0);
        }
    }

    #[test]
    fn trim_tank_tilts_more_than_a_feed_tank_for_the_same_pitch() {
        let trim = TankShape::of(Tank::Trim);
        let feed = TankShape::of(Tank::Feed1);
        let t_trim = tilt_fraction(&trim, 5.0, 0.0, 0.0, 0.0);
        let t_feed = tilt_fraction(&feed, 5.0, 0.0, 0.0, 0.0);
        assert!(t_trim > t_feed, "trim {t_trim} feed {t_feed}");
    }

    #[test]
    fn outer_tank_tilts_more_than_a_feed_tank_for_the_same_bank() {
        let outer = TankShape::of(Tank::LeftOuter);
        let feed = TankShape::of(Tank::Feed1);
        let t_outer = tilt_fraction(&outer, 0.0, 20.0, 0.0, 0.0);
        let t_feed = tilt_fraction(&feed, 0.0, 20.0, 0.0, 0.0);
        assert!(t_outer > t_feed, "outer {t_outer} feed {t_feed}");
    }

    #[test]
    fn lateral_acceleration_tilts_the_surface_like_an_equivalent_bank_angle() {
        let s = TankShape::of(Tank::LeftMid);
        let level = tilt_fraction(&s, 0.0, 0.0, 0.0, 0.0);
        let accelerating = tilt_fraction(&s, 0.0, 0.0, 0.3, 0.0);
        assert_eq!(level, 0.0);
        assert!(accelerating > 0.0);
        // 0.3 g lateral ~ atan(0.3) ~ 16.7 deg of equivalent bank.
        let equivalent_bank = tilt_fraction(&s, 0.0, 0.3f64.atan().to_degrees(), 0.0, 0.0);
        assert!((accelerating - equivalent_bank).abs() < 1e-9);
    }

    #[test]
    fn a_full_level_tank_is_fully_ported_and_a_dry_one_is_not() {
        let s = TankShape::of(Tank::Feed2);
        assert_eq!(unporting_factor(&s, 1.0, 0.0, 0.0, 0.0, 0.0), 1.0);
        assert_eq!(unporting_factor(&s, 0.0, 0.0, 0.0, 0.0, 0.0), 0.0);
    }

    #[test]
    fn steep_bank_at_low_fill_unports_the_inlet_progressively_not_as_a_cliff() {
        let s = TankShape::of(Tank::LeftOuter);
        let mut last = 1.0;
        for bank in [0.0, 5.0, 10.0, 15.0, 20.0, 30.0] {
            let f = unporting_factor(&s, 0.15, 0.0, bank, 0.0, 0.0);
            assert!(f <= last + 1e-9, "unporting should not increase with more bank");
            assert!((0.0..=1.0).contains(&f));
            last = f;
        }
        assert!(last < 1.0, "steep bank at 15% fill should unport somewhat");
    }

    #[test]
    fn unusable_fraction_grows_as_the_tank_unports_and_never_exceeds_one() {
        let s = TankShape::of(Tank::Trim);
        let level = unusable_fraction_now(&s, 0.5, 0.0, 0.0, 0.0, 0.0);
        let tilted = unusable_fraction_now(&s, 0.02, 0.0, 45.0, 0.0, 0.0);
        assert!(tilted >= level);
        assert!(tilted <= 1.0);
        assert!(level >= s.base_unusable_fraction - 1e-12);
    }

    #[test]
    fn an_empty_tank_has_no_finite_sloshing_period() {
        let s = TankShape::of(Tank::Feed1);
        assert!(sloshing_period_s(&s, 0.0).is_infinite());
        assert_eq!(sloshing_settle_time_s(&s, 0.0), 0.0);
    }

    #[test]
    fn a_part_full_tank_has_a_positive_finite_sloshing_period_and_settle_time() {
        let s = TankShape::of(Tank::LeftMid);
        let period = sloshing_period_s(&s, 0.4);
        let settle = sloshing_settle_time_s(&s, 0.4);
        assert!(period > 0.0 && period.is_finite());
        assert!(settle > 0.0 && settle.is_finite());
    }

    #[test]
    fn a_more_heavily_damped_tank_settles_faster() {
        let mut baffled = TankShape::of(Tank::LeftInner);
        baffled.slosh_damping_ratio = 0.5;
        let mut sloshy = TankShape::of(Tank::LeftInner);
        sloshy.slosh_damping_ratio = 0.05;
        assert!(sloshing_settle_time_s(&baffled, 0.5) < sloshing_settle_time_s(&sloshy, 0.5));
    }
}
