#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Contaminant {
    Dry,
    Frost,
    Water { depth_mm: f64 },
    Slush { depth_mm: f64 },
    DrySnow { depth_mm: f64 },
    WetSnow { depth_mm: f64 },
    CompactedSnow { oat_c: f64 },
    Ice,
    WaterOverIceOrCompactedSnow,
}

const DEPTH_THRESHOLD_MM: f64 = 3.0;
const COMPACTED_SNOW_COLD_THRESHOLD_C: f64 = -15.0;

pub fn runway_condition_code(c: Contaminant) -> u8 {
    match c {
        Contaminant::Dry => 6,
        Contaminant::Frost => 5,
        Contaminant::Water { depth_mm } | Contaminant::Slush { depth_mm } | Contaminant::DrySnow { depth_mm } | Contaminant::WetSnow { depth_mm } => {
            if depth_mm <= DEPTH_THRESHOLD_MM {
                5
            } else {
                3
            }
        }
        Contaminant::CompactedSnow { oat_c } => {
            if oat_c <= COMPACTED_SNOW_COLD_THRESHOLD_C {
                4
            } else {
                3
            }
        }
        Contaminant::Ice => 1,
        Contaminant::WaterOverIceOrCompactedSnow => 0,
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BrakingAction {
    Good,
    GoodToMedium,
    Medium,
    MediumToPoor,
    Poor,
    Nil,
}

pub fn braking_action(rwy_cc: u8) -> BrakingAction {
    match rwy_cc {
        6 | 5 => BrakingAction::Good,
        4 => BrakingAction::GoodToMedium,
        3 => BrakingAction::Medium,
        2 => BrakingAction::MediumToPoor,
        1 => BrakingAction::Poor,
        _ => BrakingAction::Nil,
    }
}

pub const MU_COMPACTED_SNOW_COLD: f64 = 0.20;
pub const MU_SNOW_OR_SLIPPERY_WET: f64 = 0.16;
pub const MU_ICE: f64 = 0.08;

fn generic_mu_pending_25_109c(rwy_cc: u8) -> f64 {
    match rwy_cc {
        6 => 0.40,
        5 => 0.38,
        4 => 0.33,
        3 => 0.28,
        2 => 0.22,
        1 => 0.16,
        _ => 0.05,
    }
}

pub fn wheel_braking_coefficient(c: Contaminant) -> f64 {
    match c {
        Contaminant::Dry => generic_mu_pending_25_109c(6),
        Contaminant::Frost => generic_mu_pending_25_109c(runway_condition_code(c)),
        Contaminant::Water { depth_mm } | Contaminant::Slush { depth_mm } => {
            if depth_mm <= DEPTH_THRESHOLD_MM {
                generic_mu_pending_25_109c(runway_condition_code(c))
            } else {
                (0.5 * generic_mu_pending_25_109c(runway_condition_code(c))).min(MU_SNOW_OR_SLIPPERY_WET)
            }
        }
        Contaminant::DrySnow { depth_mm } | Contaminant::WetSnow { depth_mm } => {
            if depth_mm <= DEPTH_THRESHOLD_MM {
                generic_mu_pending_25_109c(runway_condition_code(c))
            } else {
                MU_SNOW_OR_SLIPPERY_WET
            }
        }
        Contaminant::CompactedSnow { oat_c } => {
            if oat_c <= COMPACTED_SNOW_COLD_THRESHOLD_C {
                MU_COMPACTED_SNOW_COLD
            } else {
                MU_SNOW_OR_SLIPPERY_WET
            }
        }
        Contaminant::Ice => MU_ICE,
        Contaminant::WaterOverIceOrCompactedSnow => MU_HYDROPLANE_RESIDUAL,
    }
}

fn has_fluid_film(c: Contaminant) -> bool {
    match c {
        Contaminant::Water { depth_mm } | Contaminant::Slush { depth_mm } => depth_mm > DEPTH_THRESHOLD_MM,
        _ => false,
    }
}

pub fn hydroplane_speed_kt(tire_pressure_psi: f64) -> f64 {
    9.0 * tire_pressure_psi.max(0.0).sqrt()
}

pub const MU_HYDROPLANE_RESIDUAL: f64 = 0.05;
const HYDROPLANE_FULL_FRACTION: f64 = 0.85;
const HYDROPLANE_ONSET_FRACTION: f64 = 0.70;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RunwayFrictionOutput {
    pub contaminant: Contaminant,
    pub rwy_cc: u8,
    pub braking_action: BrakingAction,
    pub mu_effective: f64,
    pub hydroplaning: bool,
    pub hydroplane_speed_kt: f64,
}

pub fn friction(contaminant: Contaminant, tire_pressure_psi: f64, groundspeed_kt: f64) -> RunwayFrictionOutput {
    let rwy_cc = runway_condition_code(contaminant);
    let mu_low_speed = wheel_braking_coefficient(contaminant);
    let v_p = hydroplane_speed_kt(tire_pressure_psi);
    let (mu_effective, hydroplaning) = if has_fluid_film(contaminant) && v_p > 0.0 {
        let ratio = (groundspeed_kt.max(0.0) / v_p).clamp(0.0, 2.0);
        let span = HYDROPLANE_FULL_FRACTION - HYDROPLANE_ONSET_FRACTION;
        let onset = ((ratio - HYDROPLANE_ONSET_FRACTION) / span).clamp(0.0, 1.0);
        (
            mu_low_speed + (MU_HYDROPLANE_RESIDUAL - mu_low_speed) * onset,
            ratio >= HYDROPLANE_FULL_FRACTION,
        )
    } else {
        (mu_low_speed, false)
    };
    RunwayFrictionOutput { contaminant, rwy_cc, braking_action: braking_action(rwy_cc), mu_effective, hydroplaning, hydroplane_speed_kt: v_p }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dry_and_ice_map_to_the_published_codes_and_bands() {
        let dry = friction(Contaminant::Dry, 200.0, 140.0);
        assert_eq!(dry.rwy_cc, 6);
        assert_eq!(dry.braking_action, BrakingAction::Good);
        let ice = friction(Contaminant::Ice, 200.0, 30.0);
        assert_eq!(ice.rwy_cc, 1);
        assert_eq!(ice.braking_action, BrakingAction::Poor);
        let nil = friction(Contaminant::WaterOverIceOrCompactedSnow, 200.0, 30.0);
        assert_eq!(nil.rwy_cc, 0);
        assert_eq!(nil.braking_action, BrakingAction::Nil);
    }

    #[test]
    fn deeper_contaminant_of_the_same_type_gives_a_lower_friction_coefficient() {
        let shallow = friction(Contaminant::Water { depth_mm: 2.0 }, 200.0, 20.0);
        let deep = friction(Contaminant::Water { depth_mm: 6.0 }, 200.0, 20.0);
        assert!(deep.mu_effective < shallow.mu_effective);
    }

    #[test]
    fn colder_compacted_snow_brakes_better_than_warmer_compacted_snow() {
        let cold = friction(Contaminant::CompactedSnow { oat_c: -20.0 }, 200.0, 20.0);
        let warm = friction(Contaminant::CompactedSnow { oat_c: -5.0 }, 200.0, 20.0);
        assert!(cold.mu_effective > warm.mu_effective);
    }

    #[test]
    fn hydroplane_speed_matches_hornes_worked_example() {
        assert!((hydroplane_speed_kt(50.0) - 63.6).abs() < 0.5);
    }

    #[test]
    fn exceeding_hydroplane_speed_on_a_wet_runway_collapses_friction() {
        let slow = friction(Contaminant::Water { depth_mm: 10.0 }, 180.0, 50.0);
        let fast = friction(Contaminant::Water { depth_mm: 10.0 }, 180.0, 160.0);
        let v_p = hydroplane_speed_kt(180.0);
        assert!(160.0 > v_p, "test setup: {v_p}");
        assert!(!slow.hydroplaning);
        assert!(fast.hydroplaning);
        assert!(fast.mu_effective < slow.mu_effective);
        assert!((fast.mu_effective - MU_HYDROPLANE_RESIDUAL).abs() < 1e-9);
    }

    #[test]
    fn a_dry_runway_never_hydroplanes_regardless_of_speed() {
        let out = friction(Contaminant::Dry, 200.0, 999.0);
        assert!(!out.hydroplaning);
        assert_eq!(out.mu_effective, wheel_braking_coefficient(Contaminant::Dry));
    }

    #[test]
    fn zero_tyre_pressure_is_numerically_safe() {
        let out = friction(Contaminant::Water { depth_mm: 10.0 }, 0.0, 100.0);
        assert!(!out.mu_effective.is_nan());
        assert!(!out.hydroplaning);
        assert_eq!(out.hydroplane_speed_kt, 0.0);
    }

    #[test]
    fn the_sourced_coefficients_are_exactly_ac_25_31_table_2() {
        assert_eq!(wheel_braking_coefficient(Contaminant::CompactedSnow { oat_c: -15.0 }), 0.20);
        assert_eq!(wheel_braking_coefficient(Contaminant::CompactedSnow { oat_c: -40.0 }), 0.20);
        assert_eq!(wheel_braking_coefficient(Contaminant::CompactedSnow { oat_c: -14.9 }), 0.16);
        assert_eq!(wheel_braking_coefficient(Contaminant::DrySnow { depth_mm: 3.1 }), 0.16);
        assert_eq!(wheel_braking_coefficient(Contaminant::WetSnow { depth_mm: 50.0 }), 0.16);
        assert_eq!(wheel_braking_coefficient(Contaminant::Ice), 0.08);
        assert!((wheel_braking_coefficient(Contaminant::Water { depth_mm: 10.0 }) - 0.14).abs() < 1e-12);
        assert!(wheel_braking_coefficient(Contaminant::Water { depth_mm: 10.0 }) <= 0.16);
    }

    #[test]
    fn the_residual_is_reached_at_eighty_five_percent_of_the_hydroplaning_speed() {
        let p_psi = 200.0;
        let v_p = hydroplane_speed_kt(p_psi);
        let at_85 = friction(Contaminant::Water { depth_mm: 10.0 }, p_psi, 0.85 * v_p);
        assert!((at_85.mu_effective - MU_HYDROPLANE_RESIDUAL).abs() < 1e-9, "{}", at_85.mu_effective);
        assert!(at_85.hydroplaning);
        let at_69 = friction(Contaminant::Water { depth_mm: 10.0 }, p_psi, 0.69 * v_p);
        assert!((at_69.mu_effective - wheel_braking_coefficient(Contaminant::Water { depth_mm: 10.0 })).abs() < 1e-9);
        assert!(!at_69.hydroplaning);
        let mut last = f64::INFINITY;
        for i in 0..=20 {
            let v = v_p * (0.70 + 0.15 * (i as f64) / 20.0);
            let mu = friction(Contaminant::Water { depth_mm: 10.0 }, p_psi, v).mu_effective;
            assert!(mu <= last + 1e-12, "friction must not rise with speed: {mu} after {last}");
            last = mu;
        }
    }

    #[test]
    fn snow_does_not_hydroplane_but_deep_water_and_slush_do() {
        let fast_snow = friction(Contaminant::WetSnow { depth_mm: 20.0 }, 200.0, 200.0);
        assert!(!fast_snow.hydroplaning);
        assert_eq!(fast_snow.mu_effective, MU_SNOW_OR_SLIPPERY_WET);

        for c in [Contaminant::Water { depth_mm: 10.0 }, Contaminant::Slush { depth_mm: 10.0 }] {
            assert!(friction(c, 200.0, 200.0).hydroplaning, "{c:?} must hydroplane");
        }
        assert!(!friction(Contaminant::Water { depth_mm: 3.0 }, 200.0, 200.0).hydroplaning);
    }

    #[test]
    fn the_coefficient_ordering_holds_across_every_contaminant() {
        let nil = wheel_braking_coefficient(Contaminant::WaterOverIceOrCompactedSnow);
        let dry = wheel_braking_coefficient(Contaminant::Dry);
        for c in [
            Contaminant::Dry,
            Contaminant::Frost,
            Contaminant::Water { depth_mm: 1.0 },
            Contaminant::Water { depth_mm: 20.0 },
            Contaminant::Slush { depth_mm: 1.0 },
            Contaminant::Slush { depth_mm: 20.0 },
            Contaminant::DrySnow { depth_mm: 1.0 },
            Contaminant::DrySnow { depth_mm: 20.0 },
            Contaminant::WetSnow { depth_mm: 1.0 },
            Contaminant::WetSnow { depth_mm: 20.0 },
            Contaminant::CompactedSnow { oat_c: -30.0 },
            Contaminant::CompactedSnow { oat_c: 0.0 },
            Contaminant::Ice,
            Contaminant::WaterOverIceOrCompactedSnow,
        ] {
            let mu = wheel_braking_coefficient(c);
            assert!(mu >= nil - 1e-12 && mu <= dry + 1e-12, "{c:?} -> {mu}");
            assert!(mu > 0.0);
        }
    }
}
