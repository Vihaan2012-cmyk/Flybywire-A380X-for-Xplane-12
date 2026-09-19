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

pub fn station2(ambient_pressure_pa: f64, ambient_temp_k: f64, mach: f64) -> Station2 {
    let tt = gas::total_temperature(ambient_temp_k, mach, GAMMA_AIR);
    let pt_ideal = gas::total_pressure(ambient_pressure_pa, mach, GAMMA_AIR);
    Station2 { tt_k: tt, pt_pa: pt_ideal * RAM_RECOVERY }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn static_conditions_at_zero_mach_lose_only_ram_recovery() {
        let s = station2(101325.0, 288.15, 0.0);
        assert!((s.tt_k - 288.15).abs() < 1e-6);
        assert!((s.pt_pa - 101325.0 * RAM_RECOVERY).abs() < 1.0);
    }

    #[test]
    fn total_pressure_rises_with_mach() {
        let low = station2(22632.0, 216.65, 0.3).pt_pa;
        let high = station2(22632.0, 216.65, 0.85).pt_pa;
        assert!(high > low);
    }
}
