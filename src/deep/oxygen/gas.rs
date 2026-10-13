//! Oxygen as a real gas, and the arithmetic every bottle in this area
//! shares.
//!
//! An oxygen cylinder charged to 1850 psig sits at about 128 bar. That is
//! nowhere near the ideal-gas regime -- at that density real O2 departs
//! from `PV = nRT` by roughly ten percent -- so every pressure this area
//! publishes comes out of the van der Waals equation of state rather than
//! the ideal one. The crate's own `src/physics/gas.rs` already makes this
//! point and already carries a test pinning the size of the gap; this
//! module re-derives it because `docs/deep/BRIEF.md` hard rule 2 forbids a
//! deep area from depending on crate internals. Nothing here is copied
//! from that file: the constants are the published ones and the code is
//! this area's own (the cylinder inversions, the choked-orifice leak and
//! the blowdown temperature relation below have no equivalent there).
//!
//! ## Why the temperature matters
//!
//! A bottle gauge does not read a level. It reads a pressure, and pressure
//! is a function of *both* how much gas is left and how warm it is. A full
//! crew cylinder cold-soaked at -20 C reads several hundred psi below the
//! same cylinder at +21 C with exactly the same mass of oxygen in it,
//! which is why every real oxygen dispatch table is a pressure-versus-
//! temperature chart and not a single number. [`pressure_pa`] is what
//! makes that fall out rather than being scripted, and
//! [`crate::deep::oxygen::cylinder`] adds the two-node thermal model that
//! makes the bottle's own temperature a state rather than an input.
//!
//! ## Sources
//!
//! * Molar gas constant: 8.314 462 618 J/(mol K), exact by the 2019 SI
//!   redefinition (CODATA).
//! * Molar mass of O2: 0.031 998 8 kg/mol, from the IUPAC standard atomic
//!   weight of oxygen (15.999).
//! * van der Waals constants for O2: a = 1.382 bar L^2/mol^2, b = 0.031 86
//!   L/mol (CRC Handbook of Chemistry and Physics, "van der Waals
//!   constants for gases"), converted to SI below.
//! * c_p of O2: 0.918 kJ/(kg K) at 300 K, 1 bar (NIST Chemistry WebBook).
//!   c_v follows from c_p - R_specific, and gamma from their ratio --
//!   derived, not separately asserted.
//! * 1 psi = 6894.757 293 168 361 Pa, exact (the pound-force and the inch
//!   are both exactly defined).

/// Molar gas constant, J/(mol K). Exact (CODATA, 2019 SI).
pub const R_UNIVERSAL_J_PER_MOL_K: f64 = 8.314_462_618;

/// Molar mass of molecular oxygen, kg/mol (IUPAC standard atomic weight).
pub const M_O2_KG_PER_MOL: f64 = 0.031_998_8;

/// van der Waals `a` for O2, Pa m^6/mol^2 (CRC: 1.382 bar L^2/mol^2;
/// 1 bar L^2/mol^2 = 1e5 Pa * 1e-6 m^6 = 0.1 Pa m^6/mol^2).
pub const VDW_A_PA_M6_PER_MOL2: f64 = 0.138_2;

/// van der Waals `b` for O2, m^3/mol (CRC: 0.031 86 L/mol).
pub const VDW_B_M3_PER_MOL: f64 = 3.186e-5;

/// Specific heat at constant pressure for O2, J/(kg K), NIST at 300 K.
pub const CP_O2_J_PER_KG_K: f64 = 918.0;

/// Pascals per psi. Exact.
pub const PSI_TO_PA: f64 = 6894.757_293_168_361;

/// Mole fraction of oxygen in dry air, 0.209 5 (the standard atmospheric
/// composition figure; ICAO/US Standard Atmosphere 1976 lists 20.9476 %).
/// A diluter-demand regulator adds nothing at sea level precisely because
/// cabin air already contains this much.
pub const AIR_O2_MOLE_FRACTION: f64 = 0.209_5;

/// Discharge coefficient of a sharp-edged orifice, the textbook value for
/// a thin plate with a vena contracta (fluid-mechanics standard, ~0.6 --
/// e.g. White, *Fluid Mechanics*). Used for every modelled leak and for
/// the burst disc's bore, because a leak is an orifice whatever caused it.
pub const ORIFICE_DISCHARGE_COEFFICIENT: f64 = 0.6;

/// Specific gas constant of oxygen, J/(kg K): R / M = 259.84.
pub fn r_specific_j_per_kg_k() -> f64 {
    R_UNIVERSAL_J_PER_MOL_K / M_O2_KG_PER_MOL
}

/// Specific heat at constant volume, J/(kg K), from Mayer's relation.
pub fn cv_o2_j_per_kg_k() -> f64 {
    CP_O2_J_PER_KG_K - r_specific_j_per_kg_k()
}

/// Ratio of specific heats for O2, derived (about 1.395).
pub fn gamma_o2() -> f64 {
    CP_O2_J_PER_KG_K / cv_o2_j_per_kg_k()
}

/// Absolute pressure of `mass_kg` of oxygen in `volume_m3` at `temp_k`,
/// Pa, from the van der Waals equation of state
/// `P = nRT/(V - nb) - a n^2 / V^2`.
///
/// Returns zero for an empty, zero-volume or zero-temperature bottle
/// rather than a NaN or an infinity: `docs/deep/BRIEF.md`'s numerical-
/// safety rule, and the state every one of these cylinders is in on the
/// frame the plugin loads.
pub fn pressure_pa(mass_kg: f64, volume_m3: f64, temp_k: f64) -> f64 {
    if !(volume_m3 > 0.0) || !(temp_k > 0.0) || !(mass_kg > 0.0) {
        return 0.0;
    }
    let n = mass_kg / M_O2_KG_PER_MOL;
    // The co-volume term goes singular as the gas approaches close
    // packing. Real cylinders never get near it (a full crew bottle uses
    // about one part in sixty of its own free volume), but the floor keeps
    // the function total rather than relying on that.
    let free = (volume_m3 - n * VDW_B_M3_PER_MOL).max(volume_m3 * 1e-6);
    let p = n * R_UNIVERSAL_J_PER_MOL_K * temp_k / free - VDW_A_PA_M6_PER_MOL2 * n * n / (volume_m3 * volume_m3);
    if p.is_finite() {
        p.max(0.0)
    } else {
        0.0
    }
}

/// The mass of oxygen that puts `volume_m3` at `pressure_pa` and `temp_k`,
/// kg -- [`pressure_pa`] inverted.
///
/// Bisection rather than an analytic root because the van der Waals cubic
/// has three of them and only one is physical. Above the critical
/// temperature of oxygen (154.6 K, far below anything an aircraft bottle
/// sees) pressure rises monotonically with density at fixed volume, so a
/// bisection on mass is exact to machine precision in a fixed 80 steps and
/// cannot pick the wrong root.
pub fn mass_for_pressure_kg(pressure_pa_target: f64, volume_m3: f64, temp_k: f64) -> f64 {
    if !(pressure_pa_target > 0.0) || !(volume_m3 > 0.0) || !(temp_k > 0.0) {
        return 0.0;
    }
    // Upper bracket: most of the way to the co-volume packing limit.
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

/// The physical volume that holds `mass_kg` at `pressure_pa` and `temp_k`,
/// m^3.
///
/// This is how every cylinder in this area gets its size: a real cylinder
/// is specified by its *free-air capacity* (the volume its charge would
/// occupy at one atmosphere) and its charge pressure, not by its internal
/// volume, and turning the first pair into the second honestly needs the
/// real equation of state -- Boyle's law overstates a 1850 psi oxygen
/// cylinder's internal volume by about a tenth.
pub fn volume_for_charge_m3(mass_kg: f64, pressure_pa_target: f64, temp_k: f64) -> f64 {
    if !(mass_kg > 0.0) || !(pressure_pa_target > 0.0) || !(temp_k > 0.0) {
        return 0.0;
    }
    let n = mass_kg / M_O2_KG_PER_MOL;
    // Pressure falls monotonically with volume at fixed mass and
    // temperature, so bisect between just above the co-volume (where
    // pressure is enormous) and a volume large enough to be below any
    // charge pressure of interest.
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

/// The mass of oxygen a cylinder rated at `free_air_liters` holds, kg.
///
/// "Free air capacity" is the volume the charge occupies once let down to
/// one standard atmosphere at the reference temperature, which is exactly
/// a statement about mass. At one atmosphere oxygen is ideal to within a
/// tenth of a percent, but this still goes through the real equation of
/// state so that there is one definition of mass in this area and not two.
pub fn mass_from_free_air_kg(free_air_liters: f64, reference_temp_k: f64) -> f64 {
    const ONE_ATMOSPHERE_PA: f64 = 101_325.0;
    mass_for_pressure_kg(ONE_ATMOSPHERE_PA, free_air_liters * 1e-3, reference_temp_k)
}

/// Density of oxygen at a delivery pressure, kg/m^3.
///
/// Ideal gas, deliberately: this is used for gas that has already been let
/// down to cabin or regulator pressure (a few bar at most), where the van
/// der Waals correction is below a part in a thousand and the extra
/// arithmetic would be noise in a per-frame loop.
pub fn delivered_density_kg_m3(pressure_pa: f64, temp_k: f64) -> f64 {
    if !(pressure_pa > 0.0) || !(temp_k > 0.0) {
        return 0.0;
    }
    pressure_pa / (r_specific_j_per_kg_k() * temp_k)
}

/// Mass flow of oxygen through an orifice of `area_m2` from a reservoir at
/// `upstream_pa`/`upstream_temp_k` into `downstream_pa`, kg/s.
///
/// Compressible orifice flow, the standard isentropic relations: choked
/// (sonic at the throat) while the pressure ratio is below the critical
/// one, sub-critical above it. A high-pressure oxygen bottle leaking to a
/// cabin is always choked -- the critical ratio for gamma = 1.395 is 0.528
/// and the bottle is a hundred times cabin pressure -- but the
/// sub-critical branch is what makes the leak taper off properly as the
/// bottle empties rather than stopping abruptly at some threshold.
pub fn orifice_mass_flow_kg_s(area_m2: f64, upstream_pa: f64, upstream_temp_k: f64, downstream_pa: f64) -> f64 {
    if !(area_m2 > 0.0) || !(upstream_temp_k > 0.0) || upstream_pa <= downstream_pa.max(0.0) {
        return 0.0;
    }
    let g = gamma_o2();
    let r_s = r_specific_j_per_kg_k();
    let critical_ratio = (2.0 / (g + 1.0)).powf(g / (g - 1.0));
    let ratio = (downstream_pa.max(0.0) / upstream_pa).clamp(0.0, 1.0);
    let flow_function = if ratio <= critical_ratio {
        // Choked: sqrt(gamma) * (2/(gamma+1))^((gamma+1)/(2(gamma-1)))
        g.sqrt() * (2.0 / (g + 1.0)).powf((g + 1.0) / (2.0 * (g - 1.0)))
    } else {
        // Sub-critical: sqrt( 2 gamma/(gamma-1) * (r^(2/g) - r^((g+1)/g)) )
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

/// How much the gas left in a rigid vessel cools when some of it leaves,
/// as a multiplicative factor on absolute temperature over `dt_s`.
///
/// The energy balance for a rigid control volume losing mass is
/// `d(mu)/dt = -m_dot h`, and with `u = c_v T`, `h = c_p T` that reduces
/// to `dT/dt = -(m_dot / m) R_specific T`: the gas that stays behind does
/// the work of pushing the gas that leaves out, and cools doing it. This
/// is why a bottle being discharged reads lower than its own contents
/// justify, and why the reading creeps back up afterwards as the cylinder
/// wall warms the gas again. Returned as an exact exponential factor so
/// the step is stable at any `dt_s` and cannot drive the temperature
/// negative.
pub fn blowdown_cooling_factor(mass_kg: f64, mass_flow_out_kg_s: f64, dt_s: f64) -> f64 {
    if !(mass_kg > 0.0) || !(mass_flow_out_kg_s > 0.0) || !(dt_s > 0.0) {
        return 1.0;
    }
    let exponent = -(mass_flow_out_kg_s / mass_kg) * (r_specific_j_per_kg_k() / cv_o2_j_per_kg_k()) * dt_s;
    exponent.exp()
}

/// One exact first-order lag step: `x` relaxed toward `target` with time
/// constant `tau_s`. Exact rather than Euler so the step is stable at any
/// frame length -- `docs/deep/BRIEF.md`'s numerical convention.
pub fn relax(x: f64, target: f64, tau_s: f64, dt_s: f64) -> f64 {
    if !(tau_s > 0.0) || !(dt_s > 0.0) {
        return x;
    }
    let k = (-dt_s / tau_s).exp();
    target + (x - target) * k
}

/// Pressure altitude from a static pressure, feet, by the ICAO Standard
/// Atmosphere troposphere relation (sea-level 1013.25 hPa, lapse 6.5 K/km,
/// the exponent `g M / (R L)` = 5.255 88 giving the familiar 1/5.255 88 =
/// 0.190 263 form).
///
/// Used for cabin altitude, which is what the passenger masks' deployment
/// threshold and the crew regulator's dilution schedule are both written
/// against. `Truth` carries cabin *pressure*, which is the physical
/// quantity; this is the conversion, in one place.
pub fn pressure_altitude_ft(pressure_pa: f64) -> f64 {
    const SEA_LEVEL_PA: f64 = 101_325.0;
    const TROPOPAUSE_PA: f64 = 22_632.1;
    const TROPOPAUSE_ALT_FT: f64 = 36_089.24;
    const M_PER_FT: f64 = 0.3048;
    if !(pressure_pa > 0.0) {
        // Vacuum: report the top of the modelled range rather than an
        // infinity. Nothing in this area should ever see it.
        return 100_000.0;
    }
    if pressure_pa >= TROPOPAUSE_PA {
        // Troposphere: h = (T0/L) (1 - (P/P0)^(R L / g M)), with T0/L =
        // 288.15/0.0065 = 44 330.77 m and the exponent 1/5.255 88.
        const TROPOSPHERE_SCALE_M: f64 = 44_330.77;
        TROPOSPHERE_SCALE_M * (1.0 - (pressure_pa / SEA_LEVEL_PA).powf(0.190_263_1)) / M_PER_FT
    } else {
        // Isothermal stratosphere, 216.65 K: h = 11 km + (R T / g) ln(P11/P).
        const SCALE_HEIGHT_M: f64 = 6341.62;
        TROPOPAUSE_ALT_FT + SCALE_HEIGHT_M * (TROPOPAUSE_PA / pressure_pa).ln() / M_PER_FT
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_derived_gas_properties_agree_with_the_published_ones() {
        // R_specific, c_v and gamma are all derived from two published
        // numbers rather than asserted separately, so this is the check
        // that the derivation lands where the literature does.
        assert!((r_specific_j_per_kg_k() - 259.84).abs() < 0.01, "{}", r_specific_j_per_kg_k());
        assert!((cv_o2_j_per_kg_k() - 658.2).abs() < 0.5, "{}", cv_o2_j_per_kg_k());
        assert!((gamma_o2() - 1.395).abs() < 0.005, "{}", gamma_o2());
    }

    #[test]
    fn a_charged_cylinder_is_meaningfully_non_ideal() {
        // The whole reason this module exists: at 1850 psig the ideal gas
        // law is wrong by about ten percent, so a bottle sized with Boyle's
        // law and read back with van der Waals would not be self-consistent.
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
        // And it must be materially smaller than Boyle's law would claim,
        // which is the point of doing it this way.
        let boyle = 3.260 * 101_325.0 / charge;
        assert!(volume < boyle * 0.95, "real {volume} vs Boyle {boyle}");
        // A 3260 L (115 cubic foot) cylinder is a real object about the
        // size of a large scuba tank; this pins the order of magnitude.
        assert!(volume > 0.015 && volume < 0.035, "{volume} m^3");
    }

    #[test]
    fn a_colder_bottle_reads_lower_with_exactly_the_same_oxygen_in_it() {
        // The gauge reads a pressure, not a level. This is the whole
        // reason the cylinder model carries a temperature.
        let volume = 0.046;
        let mass = mass_for_pressure_kg(1850.0 * PSI_TO_PA + 101_325.0, volume, 294.15);
        let cold = pressure_pa(mass, volume, 253.15);
        let warm = pressure_pa(mass, volume, 294.15);
        let hot = pressure_pa(mass, volume, 323.15);
        assert!(cold < warm && warm < hot);
        // Roughly proportional to absolute temperature: -41 K off 294 K is
        // about a 14 % fall, several hundred psi on a full bottle.
        let fall_psi = (warm - cold) / PSI_TO_PA;
        assert!(fall_psi > 150.0 && fall_psi < 400.0, "{fall_psi} psi");
    }

    #[test]
    fn a_leak_is_choked_out_of_a_full_bottle_and_tapers_as_it_empties() {
        let area = 1e-6; // 1 mm^2
        let full = orifice_mass_flow_kg_s(area, 1850.0 * PSI_TO_PA + 101_325.0, 294.15, 75_000.0);
        let low = orifice_mass_flow_kg_s(area, 300_000.0, 294.15, 75_000.0);
        let nearly_equal = orifice_mass_flow_kg_s(area, 80_000.0, 294.15, 75_000.0);
        assert!(full > low && low > nearly_equal);
        assert_eq!(orifice_mass_flow_kg_s(area, 70_000.0, 294.15, 75_000.0), 0.0, "no flow backwards");
        // Choked flow is linear in upstream pressure; halving it must halve
        // the flow exactly while both ends stay choked.
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

        // Across the whole physically reachable domain -- a frame cannot
        // expel more than the vessel holds, which `cylinder::HpCylinder`
        // enforces before it calls this -- the factor is strictly
        // positive, so the temperature can never reach zero however hard
        // the vessel is blown down.
        for (mass, dt) in [(8.0, 1.0), (8.0, 0.001), (1e-6, 1.0), (1e-6, 60.0)] {
            let most = mass / dt; // the largest flow that frame can pass
            let f = blowdown_cooling_factor(mass, most, dt);
            assert!(f > 0.0 && f <= 1.0, "mass {mass} dt {dt} gave {f}");
        }

        // Outside it -- an input no caller can produce -- the exponential
        // underflows to exactly zero rather than going negative or to a
        // NaN, which is the property the cylinder's own `.max(1.0)` on gas
        // temperature relies on.
        let absurd = blowdown_cooling_factor(0.001, 10.0, 1000.0);
        assert!(absurd.is_finite() && (0.0..=1.0).contains(&absurd), "{absurd}");
    }

    #[test]
    fn pressure_altitude_matches_the_standard_atmosphere() {
        assert!((pressure_altitude_ft(101_325.0)).abs() < 1.0);
        // 8000 ft cabin is 75 262 Pa in the ISA.
        assert!((pressure_altitude_ft(75_262.0) - 8000.0).abs() < 20.0, "{}", pressure_altitude_ft(75_262.0));
        assert!((pressure_altitude_ft(22_632.1) - 36_089.0).abs() < 20.0, "{}", pressure_altitude_ft(22_632.1));
        // Above the tropopause the isothermal branch takes over.
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
