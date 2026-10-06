#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Awg {
    Size(i32),
    Aught(u32),
}

fn ohms_per_kft_at_20c(awg: Awg) -> f64 {
    match awg {
        Awg::Size(22) => 16.14,
        Awg::Size(20) => 10.15,
        Awg::Size(18) => 6.385,
        Awg::Size(16) => 4.016,
        Awg::Size(14) => 2.525,
        Awg::Size(12) => 1.588,
        Awg::Size(10) => 0.9989,
        Awg::Size(8) => 0.6282,
        Awg::Size(6) => 0.3951,
        Awg::Size(4) => 0.2485,
        Awg::Size(2) => 0.1563,
        Awg::Aught(1) => 0.09827,
        Awg::Aught(2) => 0.07793,
        Awg::Aught(4) => 0.04901,
        Awg::Size(n) => ohms_per_kft_at_20c(Awg::Size(nearest_tabulated(n))),
        Awg::Aught(_) => ohms_per_kft_at_20c(Awg::Aught(4)),
    }
}

fn nearest_tabulated(n: i32) -> i32 {
    const SIZES: [i32; 11] = [22, 20, 18, 16, 14, 12, 10, 8, 6, 4, 2];
    *SIZES.iter().min_by_key(|&&s| (s - n).abs()).unwrap_or(&22)
}

const FT1000_TO_M: f64 = 304.8;

const COPPER_ALPHA_PER_K_AT_20C: f64 = 0.00393;

impl Awg {
    pub fn resistance_per_m_at_20c(self) -> f64 {
        ohms_per_kft_at_20c(self) / FT1000_TO_M
    }

    pub fn resistance_per_m(self, temp_c: f64) -> f64 {
        let r20 = self.resistance_per_m_at_20c();
        (r20 * (1.0 + COPPER_ALPHA_PER_K_AT_20C * (temp_c - 20.0))).max(r20 * 0.05)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Insulation {
    Etfe150,
    Ptfe200,
    Ptfe260,
}
impl Insulation {
    pub fn max_temp_c(self) -> f64 {
        match self {
            Insulation::Etfe150 => 150.0,
            Insulation::Ptfe200 => 200.0,
            Insulation::Ptfe260 => 260.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WireSpec {
    pub awg: Awg,
    pub insulation: Insulation,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resistance_falls_with_thicker_gauge() {
        let thin = Awg::Size(22).resistance_per_m_at_20c();
        let thick = Awg::Size(10).resistance_per_m_at_20c();
        let heavy = Awg::Aught(1).resistance_per_m_at_20c();
        assert!(thin > thick, "22 AWG must resist more per metre than 10 AWG");
        assert!(thick > heavy, "10 AWG must resist more per metre than 1/0");
    }

    #[test]
    fn resistance_rises_with_temperature() {
        let cold = Awg::Size(20).resistance_per_m(-40.0);
        let hot = Awg::Size(20).resistance_per_m(150.0);
        let ref20 = Awg::Size(20).resistance_per_m_at_20c();
        assert!(cold < ref20 && ref20 < hot, "cold {cold} < ref {ref20} < hot {hot}");
    }

    #[test]
    fn resistance_never_reaches_zero_or_goes_negative_at_extreme_cold() {
        let r = Awg::Size(18).resistance_per_m(-273.0);
        assert!(r.is_finite() && r > 0.0);
    }

    #[test]
    fn unlisted_awg_falls_back_to_nearest_tabulated_value_not_a_panic() {
        let r = Awg::Size(19).resistance_per_m_at_20c();
        let r18 = Awg::Size(18).resistance_per_m_at_20c();
        let r20 = Awg::Size(20).resistance_per_m_at_20c();
        assert!(r == r18 || r == r20);
    }

    #[test]
    fn insulation_classes_order_by_rating() {
        assert!(Insulation::Etfe150.max_temp_c() < Insulation::Ptfe200.max_temp_c());
        assert!(Insulation::Ptfe200.max_temp_c() < Insulation::Ptfe260.max_temp_c());
    }
}
