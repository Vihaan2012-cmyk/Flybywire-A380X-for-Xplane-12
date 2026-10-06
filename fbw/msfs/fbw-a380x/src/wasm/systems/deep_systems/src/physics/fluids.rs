pub fn orifice_flow_m3_s(discharge_coefficient: f64, area_m2: f64, delta_pressure_pa: f64, density_kg_m3: f64) -> f64 {
    if area_m2 <= 0. || density_kg_m3 <= 0. || delta_pressure_pa <= 0. {
        return 0.;
    }
    discharge_coefficient * area_m2 * (2. * delta_pressure_pa / density_kg_m3).sqrt()
}

pub fn effective_cda_m2(reference_flow_m3_s: f64, reference_delta_pressure_pa: f64, density_kg_m3: f64) -> f64 {
    if reference_delta_pressure_pa <= 0. || density_kg_m3 <= 0. {
        return 0.;
    }
    reference_flow_m3_s / (2. * reference_delta_pressure_pa / density_kg_m3).sqrt()
}

pub fn recovery_temperature_k(static_temp_k: f64, mach: f64, recovery_factor: f64) -> f64 {
    const GAMMA: f64 = 1.4;
    static_temp_k * (1. + recovery_factor * (GAMMA - 1.) / 2. * mach * mach)
}

pub fn convective_heat_w(h_w_m2k: f64, area_m2: f64, delta_t_k: f64) -> f64 {
    h_w_m2k * area_m2 * delta_t_k
}

pub fn tank_wetted_area_m2(volume_m3: f64, aspect: f64) -> f64 {
    if volume_m3 <= 0. {
        return 0.;
    }
    let a = aspect.max(1.);
    let w = tank_box_height_m(volume_m3, aspect);
    2. * w * w * (2. * a + a * a)
}

pub fn tank_box_height_m(volume_m3: f64, aspect: f64) -> f64 {
    if volume_m3 <= 0. {
        return 0.;
    }
    let a = aspect.max(1.);
    (volume_m3 / (a * a)).cbrt()
}

pub const JET_A_DENSITY_KG_M3_AT_15C: f64 = 802.5;

pub const JET_A_THERMAL_EXPANSION_PER_K: f64 = 9.0e-4;

pub fn jet_a_density_kg_m3(temp_c: f64) -> f64 {
    JET_A_DENSITY_KG_M3_AT_15C / (1. + JET_A_THERMAL_EXPANSION_PER_K * (temp_c - 15.))
}

const VISC_REF1_TEMP_K: f64 = 293.15;
const VISC_REF1_CST: f64 = 1.5;
const VISC_REF2_TEMP_K: f64 = 253.15;
const VISC_REF2_CST: f64 = 8.0;

pub fn jet_a_viscosity_cst(temp_c: f64) -> f64 {
    let temp_k = temp_c + 273.15;
    let z = |v: f64| (v + 0.7).log10().log10();
    let (z1, z2) = (z(VISC_REF1_CST), z(VISC_REF2_CST));
    let (t1, t2) = (VISC_REF1_TEMP_K.log10(), VISC_REF2_TEMP_K.log10());
    let b = (z1 - z2) / (t2 - t1);
    let a = z1 + b * t1;
    let zt = a - b * temp_k.log10();
    (10f64.powf(10f64.powf(zt))) - 0.7
}

pub const FUEL_FREEZE_POINT_C: f64 = -40.;

const REFERENCE_VISCOSITY_CST: f64 = 1.5;
pub fn viscosity_flow_derate(viscosity_cst: f64) -> f64 {
    (REFERENCE_VISCOSITY_CST / viscosity_cst.max(REFERENCE_VISCOSITY_CST)).clamp(0.05, 1.0)
}

pub fn hydrostatic_pressure_pa(height_m: f64, density_kg_m3: f64) -> f64 {
    const G: f64 = 9.80665;
    height_m.max(0.) * density_kg_m3 * G
}

pub fn unporting_factor(
    fill_fraction: f64,
    pitch_deg: f64,
    bank_deg: f64,
    box_height_m: f64,
    aspect: f64,
    margin_fraction: f64,
) -> f64 {
    if box_height_m <= 0. {
        return 1.;
    }
    let fill_fraction = fill_fraction.clamp(0., 1.);
    let box_length_m = aspect.max(1.) * box_height_m;
    let tilt_m =
        (box_length_m / 2.) * bank_deg.to_radians().tan().abs() + (box_height_m / 2.) * pitch_deg.to_radians().tan().abs();
    let depth_at_inlet_m = fill_fraction * box_height_m - tilt_m;
    let margin_m = margin_fraction.max(1e-6) * box_height_m;
    (depth_at_inlet_m / margin_m).clamp(0., 1.)
}

pub fn tank_tilt_fraction(pitch_deg: f64, bank_deg: f64, box_height_m: f64, aspect: f64) -> f64 {
    if box_height_m <= 0. {
        return 0.;
    }
    let box_length_m = aspect.max(1.) * box_height_m;
    let tilt_m =
        (box_length_m / 2.) * bank_deg.to_radians().tan().abs() + (box_height_m / 2.) * pitch_deg.to_radians().tan().abs();
    tilt_m / box_height_m
}

pub fn probe_indicated_fill_fraction(fill_fraction: f64, tilt_fraction: f64, probe_count: u32) -> f64 {
    let n = probe_count.max(1);
    let fill_fraction = fill_fraction.clamp(0., 1.);
    let mut sum = 0.0;
    for i in 0..n {
        let s = (i as f64 + 0.5) / n as f64 - 0.5;
        sum += (fill_fraction + tilt_fraction * s).clamp(0., 1.);
    }
    sum / n as f64
}

pub struct PumpCurve {
    pub shutoff_pressure_pa: f64,
    pub rated_flow_m3_s: f64,
}
impl PumpCurve {
    pub fn flow_m3_s(&self, back_pressure_pa: f64) -> f64 {
        if self.shutoff_pressure_pa <= 0. {
            return 0.;
        }
        let ratio = 1. - (back_pressure_pa / self.shutoff_pressure_pa).clamp(0., 1.);
        self.rated_flow_m3_s * ratio
    }
}

pub fn hydraulic_power_w(pressure_pa: f64, flow_m3_s: f64) -> f64 {
    pressure_pa * flow_m3_s
}

pub fn pump_current_a(hydraulic_power_w: f64, voltage_v: f64, motor_efficiency: f64) -> f64 {
    if voltage_v <= 0. || motor_efficiency <= 0. {
        return 0.;
    }
    hydraulic_power_w / (motor_efficiency * voltage_v)
}

pub const HHX_EFFECTIVENESS: f64 = 0.6;

pub fn heat_exchanger_transfer_w(effectiveness: f64, c_min_w_per_k: f64, hot_in_k: f64, cold_in_k: f64) -> f64 {
    effectiveness * c_min_w_per_k * (hot_in_k - cold_in_k)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orifice_flow_scales_with_sqrt_of_pressure() {
        let f1 = orifice_flow_m3_s(0.7, 0.001, 100_000., 800.);
        let f4 = orifice_flow_m3_s(0.7, 0.001, 400_000., 800.);
        assert!((f4 / f1 - 2.0).abs() < 1e-9);
    }

    #[test]
    fn orifice_flow_is_zero_with_no_pressure_or_closed_area() {
        assert_eq!(orifice_flow_m3_s(0.7, 0., 100_000., 800.), 0.);
        assert_eq!(orifice_flow_m3_s(0.7, 0.001, 0., 800.), 0.);
        assert_eq!(orifice_flow_m3_s(0.7, 0.001, -1., 800.), 0.);
    }

    #[test]
    fn effective_cda_reproduces_the_reference_flow() {
        let cda = effective_cda_m2(0.01, 50_000., 800.);
        let flow = orifice_flow_m3_s(1.0, cda, 50_000., 800.);
        assert!((flow - 0.01).abs() < 1e-9);
    }

    #[test]
    fn recovery_temperature_exceeds_static_in_flight() {
        let static_k = 273.15 - 50.;
        let recovered = recovery_temperature_k(static_k, 0.85, 0.9);
        assert!(recovered > static_k);
        assert!(recovered - static_k < 40.);
    }

    #[test]
    fn recovery_temperature_at_zero_mach_is_static() {
        assert_eq!(recovery_temperature_k(280., 0., 0.9), 280.);
    }

    #[test]
    fn wetted_area_grows_with_volume() {
        let small = tank_wetted_area_m2(1.0, 4.0);
        let big = tank_wetted_area_m2(8.0, 4.0);
        assert!(big > small);
        assert!((big / small - 4.0).abs() < 1e-6);
    }

    #[test]
    fn jet_a_density_falls_with_temperature() {
        let cold = jet_a_density_kg_m3(-20.);
        let warm = jet_a_density_kg_m3(40.);
        assert!(cold > warm, "fuel should be denser when cold");
        assert!((jet_a_density_kg_m3(15.) - JET_A_DENSITY_KG_M3_AT_15C).abs() < 1e-9);
    }

    #[test]
    fn jet_a_viscosity_matches_its_two_reference_points() {
        assert!((jet_a_viscosity_cst(20.) - VISC_REF1_CST).abs() < 1e-6);
        assert!((jet_a_viscosity_cst(-20.) - VISC_REF2_CST).abs() < 1e-6);
    }

    #[test]
    fn jet_a_viscosity_rises_as_fuel_cools() {
        assert!(jet_a_viscosity_cst(-40.) > jet_a_viscosity_cst(-20.));
        assert!(jet_a_viscosity_cst(-20.) > jet_a_viscosity_cst(20.));
    }

    #[test]
    fn viscosity_derate_is_full_at_reference_and_falls_when_thick() {
        assert_eq!(viscosity_flow_derate(REFERENCE_VISCOSITY_CST), 1.0);
        assert_eq!(viscosity_flow_derate(0.1), 1.0);
        assert!(viscosity_flow_derate(30.) < 1.0);
        assert!(viscosity_flow_derate(1000.) >= 0.05);
    }

    #[test]
    fn pump_curve_is_linear_between_shutoff_and_rated_flow() {
        let curve = PumpCurve { shutoff_pressure_pa: 400_000., rated_flow_m3_s: 0.002 };
        assert_eq!(curve.flow_m3_s(0.), 0.002);
        assert_eq!(curve.flow_m3_s(400_000.), 0.);
        assert!((curve.flow_m3_s(200_000.) - 0.001).abs() < 1e-9);
        assert_eq!(curve.flow_m3_s(1_000_000.), 0.);
    }

    #[test]
    fn pump_current_scales_inversely_with_voltage_and_efficiency() {
        let i = pump_current_a(1000., 115., 0.9);
        assert!((i - 1000. / (0.9 * 115.)).abs() < 1e-9);
        assert_eq!(pump_current_a(1000., 0., 0.9), 0.);
    }

    #[test]
    fn tank_box_height_grows_with_volume() {
        let h1 = tank_box_height_m(1.0, 6.0);
        let h8 = tank_box_height_m(8.0, 6.0);
        assert!((h8 / h1 - 2.0).abs() < 1e-6);
    }

    #[test]
    fn hydrostatic_pressure_scales_with_height_and_density() {
        assert_eq!(hydrostatic_pressure_pa(0., 800.), 0.);
        let p1 = hydrostatic_pressure_pa(1., 800.);
        let p2 = hydrostatic_pressure_pa(2., 800.);
        assert!((p2 / p1 - 2.0).abs() < 1e-9);
        assert!(p1 > 0.);
    }

    #[test]
    fn heat_exchanger_transfer_is_zero_at_equal_temperatures() {
        assert_eq!(heat_exchanger_transfer_w(0.6, 500., 300., 300.), 0.);
        assert!(heat_exchanger_transfer_w(0.6, 500., 350., 300.) > 0.);
    }

    #[test]
    fn unporting_full_tank_level_attitude_is_full_pressure() {
        assert_eq!(unporting_factor(1.0, 0., 0., 1.0, 6.0, 0.15), 1.0);
    }

    #[test]
    fn unporting_empty_tank_is_zero_regardless_of_attitude() {
        assert_eq!(unporting_factor(0.0, 0., 0., 1.0, 6.0, 0.15), 0.0);
        assert_eq!(unporting_factor(0.0, 20., 30., 1.0, 6.0, 0.15), 0.0);
    }

    #[test]
    fn unporting_zero_box_height_is_a_safe_default() {
        assert_eq!(unporting_factor(0.5, 10., 10., 0.0, 6.0, 0.15), 1.0);
    }

    #[test]
    fn unporting_increases_with_bank_at_low_fill() {
        let level = unporting_factor(0.2, 0., 0., 1.0, 6.0, 0.15);
        let banked = unporting_factor(0.2, 0., 20., 1.0, 6.0, 0.15);
        assert!(banked < level, "banked {banked} should derate more than level {level}");
    }

    #[test]
    fn unporting_bank_derates_more_than_equal_pitch_for_a_span_elongated_tank() {
        let pitched = unporting_factor(0.3, 20., 0., 1.0, 6.0, 0.15);
        let banked = unporting_factor(0.3, 0., 20., 1.0, 6.0, 0.15);
        assert!(banked <= pitched);
    }

    #[test]
    fn unporting_never_negative_or_above_one() {
        for fill in [0.0, 0.05, 0.3, 0.7, 1.0] {
            for angle in [-45., -20., 0., 20., 45.] {
                let f = unporting_factor(fill, angle, angle, 1.2, 6.0, 0.15);
                assert!((0.0..=1.0).contains(&f), "fill={fill} angle={angle} -> {f}");
            }
        }
    }

    #[test]
    fn probe_indication_is_exact_at_level_attitude() {
        for fill in [0.0, 0.1, 0.5, 0.9, 1.0] {
            for n in [1, 4, 8, 14] {
                let ind = probe_indicated_fill_fraction(fill, 0.0, n);
                assert!((ind - fill).abs() < 1e-9, "fill={fill} n={n} -> {ind}");
            }
        }
    }

    #[test]
    fn probe_indication_diverges_from_true_only_once_tilt_clips_the_array() {
        let mild = probe_indicated_fill_fraction(0.6, 0.2, 8);
        assert!((mild - 0.6).abs() < 1e-9);
        let steep = probe_indicated_fill_fraction(0.1, 1.0, 8);
        assert!(steep > 0.1, "steep tilt at low fill should read higher than true: {steep}");
    }

    #[test]
    fn tilt_fraction_is_zero_when_level() {
        assert_eq!(tank_tilt_fraction(0., 0., 1.0, 6.0), 0.);
        assert!(tank_tilt_fraction(20., 20., 1.0, 6.0) > 0.);
    }

    #[test]
    fn unporting_is_monotonic_in_fill_fraction_at_fixed_attitude() {
        let low = unporting_factor(0.1, 5., 10., 1.0, 6.0, 0.15);
        let high = unporting_factor(0.4, 5., 10., 1.0, 6.0, 0.15);
        assert!(high >= low);
    }
}
