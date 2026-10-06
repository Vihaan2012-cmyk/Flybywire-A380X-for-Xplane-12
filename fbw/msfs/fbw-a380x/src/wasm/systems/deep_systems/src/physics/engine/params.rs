pub const T_REF_K: f64 = 288.15;
pub const P_REF_PA: f64 = 101_325.0;

pub const N1_DESIGN_RPM: f64 = 2900.0;
pub const N2_DESIGN_RPM: f64 = 8300.0;
pub const N3_DESIGN_RPM: f64 = 12200.0;

pub const STATIC_THRUST_N: f64 = 88800.0 * 4.448_221_615_3;

pub const IDLE_N1_PCT: f64 = 15.0;
pub const IDLE_N3_PCT: f64 = 60.0;

pub const MIN_N1_FOR_COMBUSTION_PCT: f64 = 10.0;
pub const MIN_N3_FOR_COMBUSTION_PCT: f64 = 20.0;

pub const MAX_N1_PROTECTION_PCT: f64 = 101.0;
pub const MAX_N3_PROTECTION_PCT: f64 = 116.5;

pub const STARTER_ALONE_MAX_N1_PCT: f64 = 12.0;

pub const MAX_COMBUSTOR_FUEL_AIR_RATIO: f64 = 0.08;

pub const BYPASS_RATIO: f64 = 8.5;

pub const OPR_DESIGN: f64 = 42.0;

pub const FAN_DIAMETER_M: f64 = 2.95;

pub const DRY_WEIGHT_KG: f64 = 6246.0;

pub const PR_FAN_DESIGN: f64 = 1.68;
pub const PR_IPC_DESIGN: f64 = 4.0;
pub const PR_HPC_DESIGN: f64 = 6.6;

pub const MDOT_TOTAL_DESIGN_KG_S: f64 = 1420.0;

pub const ETA_FAN_DESIGN: f64 = 0.86;
pub const ETA_IPC_DESIGN: f64 = 0.86;
pub const ETA_HPC_DESIGN: f64 = 0.83;
pub const ETA_HPT_DESIGN: f64 = 0.86;
pub const ETA_IPT_DESIGN: f64 = 0.89;
pub const ETA_LPT_DESIGN: f64 = 0.91;

pub const COMBUSTOR_EFFICIENCY: f64 = 0.999;
pub const COMBUSTOR_PRESSURE_LOSS_FRAC: f64 = 0.05;

pub const RAM_RECOVERY: f64 = 0.99;

pub const MECH_EFFICIENCY: f64 = 0.99;

pub const BYPASS_DUCT_LOSS_FRAC: f64 = 0.02;

pub const LHV_JET_A1_J_KG: f64 = 43.1e6;

pub const FAR_STOICHIOMETRIC: f64 = 1.0 / 14.7;

pub mod inertia {
    use super::{DRY_WEIGHT_KG, FAN_DIAMETER_M};

    const FAN_RADIUS_M: f64 = FAN_DIAMETER_M / 2.0;

    pub const LP_MASS_FRACTION: f64 = 0.18;
    pub const IP_MASS_FRACTION: f64 = 0.08;
    pub const HP_MASS_FRACTION: f64 = 0.10;

    pub const LP_RADIUS_FRAC: f64 = 0.50;
    pub const IP_RADIUS_FRAC: f64 = 0.20;
    pub const HP_RADIUS_FRAC: f64 = 0.15;

    pub fn i_lp() -> f64 {
        (DRY_WEIGHT_KG * LP_MASS_FRACTION) * (FAN_RADIUS_M * LP_RADIUS_FRAC).powi(2)
    }
    pub fn i_ip() -> f64 {
        (DRY_WEIGHT_KG * IP_MASS_FRACTION) * (FAN_RADIUS_M * IP_RADIUS_FRAC).powi(2)
    }
    pub fn i_hp() -> f64 {
        (DRY_WEIGHT_KG * HP_MASS_FRACTION) * (FAN_RADIUS_M * HP_RADIUS_FRAC).powi(2)
    }
}
