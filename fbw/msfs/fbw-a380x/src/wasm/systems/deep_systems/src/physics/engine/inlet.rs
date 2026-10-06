use super::gas::{self, GAMMA_AIR};
use super::params::RAM_RECOVERY;

#[derive(Clone, Copy, Debug, Default)]
pub struct Station2 {
    pub tt_k: f64,
    pub pt_pa: f64,
}

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
