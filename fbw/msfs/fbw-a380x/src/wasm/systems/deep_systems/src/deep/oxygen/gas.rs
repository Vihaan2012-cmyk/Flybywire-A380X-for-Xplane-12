pub const R_UNIVERSAL_J_PER_MOL_K: f64 = 8.314_462_618;

pub const M_O2_KG_PER_MOL: f64 = 0.031_998_8;

pub const VDW_A_PA_M6_PER_MOL2: f64 = 0.138_2;

pub const VDW_B_M3_PER_MOL: f64 = 3.186e-5;

pub const CP_O2_J_PER_KG_K: f64 = 918.0;

pub const PSI_TO_PA: f64 = 6894.757_293_168_361;

pub const AIR_O2_MOLE_FRACTION: f64 = 0.209_5;

pub const ORIFICE_DISCHARGE_COEFFICIENT: f64 = 0.6;

pub fn r_specific_j_per_kg_k() -> f64 {
    R_UNIVERSAL_J_PER_MOL_K / M_O2_KG_PER_MOL
}

pub fn cv_o2_j_per_kg_k() -> f64 {
    CP_O2_J_PER_KG_K - r_specific_j_per_kg_k()
}

pub fn gamma_o2() -> f64 {
    CP_O2_J_PER_KG_K / cv_o2_j_per_kg_k()
}

pub fn pressure_pa(mass_kg: f64, volume_m3: f64, temp_k: f64) -> f64 {
    if !(volume_m3 > 0.0) || !(temp_k > 0.0) || !(mass_kg > 0.0) {
        return 0.0;
    }
    let n = mass_kg / M_O2_KG_PER_MOL;
    let free = (volume_m3 - n * VDW_B_M3_PER_MOL).max(volume_m3 * 1e-6);
    let p = n * R_UNIVERSAL_J_PER_MOL_K * temp_k / free - VDW_A_PA_M6_PER_MOL2 * n * n / (volume_m3 * volume_m3);
    if p.is_finite() {
        p.max(0.0)
    } else {
        0.0
    }
}

pub fn mass_for_pressure_kg(pressure_pa_target: f64, volume_m3: f64, temp_k: f64) -> f64 {
    if !(pressure_pa_target > 0.0) || !(volume_m3 > 0.0) || !(temp_k > 0.0) {
        return 0.0;
    }
    let mut hi = 0.9 * volume_m3 / VDW_B_M3_PER_MOL * M_O2_KG_PER_MOL;
    if pressure_pa(hi, volume_m3, temp_k) < pressure_pa_target {
        return hi;
    }
    let mut lo = 0.0;
    for _ in 0..80 {
        let mid = 0.5 * (lo + hi);
        if pressure_pa(mid, volume_m3, temp_k) < pressure_pa_target {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

pub fn volume_for_charge_m3(mass_kg: f64, pressure_pa_target: f64, temp_k: f64) -> f64 {
    if !(mass_kg > 0.0) || !(pressure_pa_target > 0.0) || !(temp_k > 0.0) {
        return 0.0;
    }
    let n = mass_kg / M_O2_KG_PER_MOL;
    let mut lo = n * VDW_B_M3_PER_MOL * 1.001;
    let mut hi = lo.max(1e-6);
    for _ in 0..200 {
        if pressure_pa(mass_kg, hi, temp_k) <= pressure_pa_target {
            break;
        }
        hi *= 2.0;
    }
    for _ in 0..80 {
        let mid = 0.5 * (lo + hi);
        if pressure_pa(mass_kg, mid, temp_k) > pressure_pa_target {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

pub fn mass_from_free_air_kg(free_air_liters: f64, reference_temp_k: f64) -> f64 {
    const ONE_ATMOSPHERE_PA: f64 = 101_325.0;
    mass_for_pressure_kg(ONE_ATMOSPHERE_PA, free_air_liters * 1e-3, reference_temp_k)
}

pub fn delivered_density_kg_m3(pressure_pa: f64, temp_k: f64) -> f64 {
    if !(pressure_pa > 0.0) || !(temp_k > 0.0) {
        return 0.0;
    }
    pressure_pa / (r_specific_j_per_kg_k() * temp_k)
}

pub fn orifice_mass_flow_kg_s(area_m2: f64, upstream_pa: f64, upstream_temp_k: f64, downstream_pa: f64) -> f64 {
    if !(area_m2 > 0.0) || !(upstream_temp_k > 0.0) || upstream_pa <= downstream_pa.max(0.0) {
        return 0.0;
    }
    let g = gamma_o2();
    let r_s = r_specific_j_per_kg_k();
    let critical_ratio = (2.0 / (g + 1.0)).powf(g / (g - 1.0));
    let ratio = (downstream_pa.max(0.0) / upstream_pa).clamp(0.0, 1.0);
    let flow_function = if ratio <= critical_ratio {
        g.sqrt() * (2.0 / (g + 1.0)).powf((g + 1.0) / (2.0 * (g - 1.0)))
    } else {
        let term = ratio.powf(2.0 / g) - ratio.powf((g + 1.0) / g);
        (2.0 * g / (g - 1.0) * term.max(0.0)).sqrt()
    };
    let m = ORIFICE_DISCHARGE_COEFFICIENT * area_m2 * upstream_pa * flow_function / (r_s * upstream_temp_k).sqrt();
    if m.is_finite() {
        m.max(0.0)
    } else {
        0.0
    }
}

pub fn blowdown_cooling_factor(mass_kg: f64, mass_flow_out_kg_s: f64, dt_s: f64) -> f64 {
    if !(mass_kg > 0.0) || !(mass_flow_out_kg_s > 0.0) || !(dt_s > 0.0) {
        return 1.0;
    }
    let exponent = -(mass_flow_out_kg_s / mass_kg) * (r_specific_j_per_kg_k() / cv_o2_j_per_kg_k()) * dt_s;
    exponent.exp()
}

pub fn relax(x: f64, target: f64, tau_s: f64, dt_s: f64) -> f64 {
    if !(tau_s > 0.0) || !(dt_s > 0.0) {
        return x;
    }
    let k = (-dt_s / tau_s).exp();
    target + (x - target) * k
}

pub fn pressure_altitude_ft(pressure_pa: f64) -> f64 {
    const SEA_LEVEL_PA: f64 = 101_325.0;
    const TROPOPAUSE_PA: f64 = 22_632.1;
    const TROPOPAUSE_ALT_FT: f64 = 36_089.24;
    const M_PER_FT: f64 = 0.3048;
    if !(pressure_pa > 0.0) {
        return 100_000.0;
    }
    if pressure_pa >= TROPOPAUSE_PA {
        const TROPOSPHERE_SCALE_M: f64 = 44_330.77;
        TROPOSPHERE_SCALE_M * (1.0 - (pressure_pa / SEA_LEVEL_PA).powf(0.190_263_1)) / M_PER_FT
    } else {
        const SCALE_HEIGHT_M: f64 = 6341.62;
        TROPOPAUSE_ALT_FT + SCALE_HEIGHT_M * (TROPOPAUSE_PA / pressure_pa).ln() / M_PER_FT
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_derived_gas_properties_agree_with_the_published_ones() {
        assert!((r_specific_j_per_kg_k() - 259.84).abs() < 0.01, "{}", r_specific_j_per_kg_k());
        assert!((cv_o2_j_per_kg_k() - 658.2).abs() < 0.5, "{}", cv_o2_j_per_kg_k());
        assert!((gamma_o2() - 1.395).abs() < 0.005, "{}", gamma_o2());
    }

    #[test]
    fn a_charged_cylinder_is_meaningfully_non_ideal() {
        let volume = 0.046;
        let temp = 294.15;
        let mass = mass_for_pressure_kg(1850.0 * PSI_TO_PA + 101_325.0, volume, temp);
        let real = pressure_pa(mass, volume, temp);
        let ideal = mass * r_specific_j_per_kg_k() * temp / volume;
        let deviation = (ideal - real).abs() / real;
        assert!(deviation > 0.05 && deviation < 0.20, "ideal {ideal} vs real {real}, deviation {deviation}");
    }

    #[test]
    fn pressure_and_mass_invert_each_other_exactly() {
        let volume = 0.046;
        for temp in [233.15, 273.15, 294.15, 333.15] {
            for psi in [100.0, 900.0, 1850.0, 2775.0] {
                let target = psi * PSI_TO_PA + 101_325.0;
                let mass = mass_for_pressure_kg(target, volume, temp);
                let back = pressure_pa(mass, volume, temp);
                assert!((back - target).abs() / target < 1e-9, "{psi} psi at {temp} K: {back} vs {target}");
            }
        }
    }

    #[test]
    fn volume_for_charge_reproduces_the_pressure_it_was_sized_for() {
        let mass = mass_from_free_air_kg(3260.0, 294.15);
        let charge = 1850.0 * PSI_TO_PA + 101_325.0;
        let volume = volume_for_charge_m3(mass, charge, 294.15);
        assert!((pressure_pa(mass, volume, 294.15) - charge).abs() / charge < 1e-9);
        let boyle = 3.260 * 101_325.0 / charge;
        assert!(volume < boyle * 0.95, "real {volume} vs Boyle {boyle}");
        assert!(volume > 0.015 && volume < 0.035, "{volume} m^3");
    }

    #[test]
    fn a_colder_bottle_reads_lower_with_exactly_the_same_oxygen_in_it() {
        let volume = 0.046;
        let mass = mass_for_pressure_kg(1850.0 * PSI_TO_PA + 101_325.0, volume, 294.15);
        let cold = pressure_pa(mass, volume, 253.15);
        let warm = pressure_pa(mass, volume, 294.15);
        let hot = pressure_pa(mass, volume, 323.15);
        assert!(cold < warm && warm < hot);
        let fall_psi = (warm - cold) / PSI_TO_PA;
        assert!(fall_psi > 150.0 && fall_psi < 400.0, "{fall_psi} psi");
    }

    #[test]
    fn a_leak_is_choked_out_of_a_full_bottle_and_tapers_as_it_empties() {
        let area = 1e-6;
        let full = orifice_mass_flow_kg_s(area, 1850.0 * PSI_TO_PA + 101_325.0, 294.15, 75_000.0);
        let low = orifice_mass_flow_kg_s(area, 300_000.0, 294.15, 75_000.0);
        let nearly_equal = orifice_mass_flow_kg_s(area, 80_000.0, 294.15, 75_000.0);
        assert!(full > low && low > nearly_equal);
        assert_eq!(orifice_mass_flow_kg_s(area, 70_000.0, 294.15, 75_000.0), 0.0, "no flow backwards");
        let a = orifice_mass_flow_kg_s(area, 10e6, 294.15, 75_000.0);
        let b = orifice_mass_flow_kg_s(area, 5e6, 294.15, 75_000.0);
        assert!((a / b - 2.0).abs() < 1e-9, "{a} {b}");
    }

    #[test]
    fn blowdown_cools_the_gas_left_behind_and_never_drives_it_negative() {
        let f = blowdown_cooling_factor(8.0, 0.01, 1.0);
        assert!(f > 0.0 && f < 1.0, "{f}");
        assert_eq!(blowdown_cooling_factor(8.0, 0.0, 1.0), 1.0);
        assert_eq!(blowdown_cooling_factor(0.0, 0.01, 1.0), 1.0);

        for (mass, dt) in [(8.0, 1.0), (8.0, 0.001), (1e-6, 1.0), (1e-6, 60.0)] {
            let most = mass / dt;
            let f = blowdown_cooling_factor(mass, most, dt);
            assert!(f > 0.0 && f <= 1.0, "mass {mass} dt {dt} gave {f}");
        }

        let absurd = blowdown_cooling_factor(0.001, 10.0, 1000.0);
        assert!(absurd.is_finite() && (0.0..=1.0).contains(&absurd), "{absurd}");
    }

    #[test]
    fn pressure_altitude_matches_the_standard_atmosphere() {
        assert!((pressure_altitude_ft(101_325.0)).abs() < 1.0);
        assert!((pressure_altitude_ft(75_262.0) - 8000.0).abs() < 20.0, "{}", pressure_altitude_ft(75_262.0));
        assert!((pressure_altitude_ft(22_632.1) - 36_089.0).abs() < 20.0, "{}", pressure_altitude_ft(22_632.1));
        assert!(pressure_altitude_ft(10_000.0) > 36_089.0);
        assert!(pressure_altitude_ft(0.0).is_finite());
    }

    #[test]
    fn nothing_here_produces_a_nan_at_rest() {
        for f in [
            pressure_pa(0.0, 0.0, 0.0),
            mass_for_pressure_kg(0.0, 0.0, 0.0),
            volume_for_charge_m3(0.0, 0.0, 0.0),
            delivered_density_kg_m3(0.0, 0.0),
            orifice_mass_flow_kg_s(0.0, 0.0, 0.0, 0.0),
            blowdown_cooling_factor(0.0, 0.0, 0.0),
            relax(1.0, 2.0, 0.0, 0.0),
            pressure_altitude_ft(101_325.0),
        ] {
            assert!(f.is_finite(), "{f}");
        }
    }
}
