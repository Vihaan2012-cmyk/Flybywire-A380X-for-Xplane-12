//! Inlet: freestream to fan-face total conditions, with ram recovery.

use super::gas::{self, GAMMA_AIR};
use super::params::RAM_RECOVERY;

/// Total temperature and pressure at the fan face (station 2), from
/// ambient static conditions and Mach.
#[derive(Clone, Copy, Debug, Default)]
pub struct Station2 {
    pub tt_k: f64,
    pub pt_pa: f64,
}

/// The recovery applies to the *ram rise*, `pt2 = p + eta_r (pt0 - p)`, and
/// not to the ambient static pressure. An inlet's total-pressure loss is a
/// duct loss — a fraction of the dynamic head the air arrives with — so it
/// has to vanish with the dynamic head. `pt0 * RAM_RECOVERY` instead takes
/// 1% off the *static* pressure, leaving a standing engine's fan face 1013
/// Pa below the air around it with nothing flowing to lose it across. In
/// the gas path that was a pressure every plenum behind the fan sat above,
/// so from rest the compressors' flow states were driven negative before
/// anything had turned. At Mach 0.85 the two forms differ by 0.6% of `pt2`
/// (1.594 p against 1.584 p), well inside what `RAM_RECOVERY` is itself
/// known to.
pub fn station2(ambient_pressure_pa: f64, ambient_temp_k: f64, mach: f64) -> Station2 {
    let tt = gas::total_temperature(ambient_temp_k, mach, GAMMA_AIR);
    let pt_ideal = gas::total_pressure(ambient_pressure_pa, mach, GAMMA_AIR);
    Station2 { tt_k: tt, pt_pa: ambient_pressure_pa + RAM_RECOVERY * (pt_ideal - ambient_pressure_pa) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn standing_still_there_is_no_ram_and_so_nothing_to_recover() {
        // With no dynamic head arriving there is no duct loss to take: the
        // fan face sits at ambient, not 1% below it (see `station2`).
        let s = station2(101325.0, 288.15, 0.0);
        assert!((s.tt_k - 288.15).abs() < 1e-6);
        assert!((s.pt_pa - 101325.0).abs() < 1e-6, "{}", s.pt_pa);
    }

    #[test]
    fn in_flight_the_recovery_takes_its_fraction_of_the_ram_rise() {
        let (p, m) = (22632.0, 0.85);
        let ideal = gas::total_pressure(p, m, GAMMA_AIR);
        let s = station2(p, 216.65, m);
        assert!((s.pt_pa - (p + RAM_RECOVERY * (ideal - p))).abs() < 1e-6);
        assert!(s.pt_pa < ideal && s.pt_pa > p);
    }

    #[test]
    fn total_pressure_rises_with_mach() {
        let low = station2(22632.0, 216.65, 0.3).pt_pa;
        let high = station2(22632.0, 216.65, 0.85).pt_pa;
        assert!(high > low);
    }
}
