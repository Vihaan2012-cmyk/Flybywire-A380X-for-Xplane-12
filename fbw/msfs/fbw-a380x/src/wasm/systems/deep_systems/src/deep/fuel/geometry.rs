use std::f64::consts::PI;

pub const G: f64 = 9.80665;
pub const GAL_TO_M3: f64 = 0.003785411784;

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

#[derive(Clone, Copy, Debug)]
pub struct TankShape {
    pub tank: Tank,
    pub kind: TankKind,
    pub capacity_gal: f64,
    pub aspect_ratio: f64,
    pub bank_sensitivity: f64,
    pub pitch_sensitivity: f64,
    pub inlet_margin_fraction: f64,
    pub base_unusable_fraction: f64,
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
    pub fn box_height_m(&self) -> f64 {
        (self.capacity_m3() / self.aspect_ratio.max(1e-6)).cbrt()
    }
    pub fn box_length_m(&self) -> f64 {
        self.box_height_m() * self.aspect_ratio.max(1.0)
    }
}

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

pub fn unporting_factor(shape: &TankShape, fill_fraction: f64, pitch_deg: f64, bank_deg: f64, lateral_accel_g: f64, longitudinal_accel_g: f64) -> f64 {
    let fill = fill_fraction.clamp(0.0, 1.0);
    let tilt = tilt_fraction(shape, pitch_deg, bank_deg, lateral_accel_g, longitudinal_accel_g);
    let depth_at_inlet = fill - tilt;
    let margin = shape.inlet_margin_fraction.max(1e-6);
    (depth_at_inlet / margin).clamp(0.0, 1.0)
}

pub fn unusable_fraction_now(shape: &TankShape, fill_fraction: f64, pitch_deg: f64, bank_deg: f64, lateral_accel_g: f64, longitudinal_accel_g: f64) -> f64 {
    let unporting = unporting_factor(shape, fill_fraction, pitch_deg, bank_deg, lateral_accel_g, longitudinal_accel_g);
    let attitude_loss = shape.inlet_margin_fraction.max(0.0) * (1.0 - unporting);
    (shape.base_unusable_fraction + attitude_loss).min(1.0)
}

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
