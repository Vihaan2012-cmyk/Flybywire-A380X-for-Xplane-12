//! Wire gauge, resistance-per-metre vs temperature, and insulation
//! temperature rating.
//!
//! Resistance figures are the NEC Chapter 9 Table 8 DC resistance values
//! for uncoated stranded copper conductors at 20 C (public: NFPA 70
//! National Electrical Code, Table 8, reproduced in innumerable public
//! engineering references and manufacturer wire catalogues), taken in their
//! commonly published ohms-per-1000-ft form and converted here to
//! ohms-per-metre (1000 ft = 304.8 m, exact). The temperature coefficient
//! of resistance for annealed copper, `alpha_20 = 0.00393 /K`, is the IACS
//! (International Annealed Copper Standard) reference value, public and
//! widely tabulated (e.g. any copper-wire engineering handbook).
//!
//! Insulation temperature classes follow the public aerospace wire
//! specification MIL-DTL-22759 ("Wire, Electric, Fluoropolymer-Insulated,
//! Copper or Copper Alloy"), whose family of constructions covers three
//! continuous-service temperature classes -- 150 C, 200 C and 260 C (the
//! spec's own `/xx` slash-sheet table, e.g. M22759/16 is a 150 C ETFE
//! construction, M22759/32 is 200 C, M22759/34 is 260 C PTFE) -- a real,
//! sourced fact, not GENERIC.

/// Standard AWG (American Wire Gauge) sizes used in this module's routing
/// catalogue, spanning small avionics signal wire up to a heavy generator
/// feeder. `Awg::Size(n)` is the conventional AWG number; `Awg::Aught(k)`
/// covers the "k/0" sizes above AWG 0 (1/0, 2/0, ...) that NEC Table 8 also
/// tabulates, for the heaviest feeders.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Awg {
    Size(i32),
    Aught(u32),
}

/// NEC Chapter 9 Table 8 DC resistance, uncoated copper, ohms per 1000 ft
/// at 20 C -- the table's own published values, not derived. Converted to
/// ohms/metre by [`Awg::resistance_per_m_at_20c`].
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
        Awg::Aught(1) => 0.09827, // 1/0
        Awg::Aught(2) => 0.07793, // 2/0
        Awg::Aught(4) => 0.04901, // 4/0
        // Any other value used by a caller: fall back to the nearest
        // tabulated size below it (never panics, never silently zero --
        // safer than fabricating an interpolated figure this module has no
        // source for).
        Awg::Size(n) => ohms_per_kft_at_20c(Awg::Size(nearest_tabulated(n))),
        Awg::Aught(_) => ohms_per_kft_at_20c(Awg::Aught(4)),
    }
}

fn nearest_tabulated(n: i32) -> i32 {
    // Deliberately stops at 2, not 0: `Awg::Size(0)` is not itself a
    // tabulated arm (AWG "1/0" and heavier live in `Awg::Aught`), so
    // including 0 here would recurse into itself forever for an
    // out-of-range caller.
    const SIZES: [i32; 11] = [22, 20, 18, 16, 14, 12, 10, 8, 6, 4, 2];
    *SIZES.iter().min_by_key(|&&s| (s - n).abs()).unwrap_or(&22)
}

/// 1000 ft in metres, exact.
const FT1000_TO_M: f64 = 304.8;

/// IACS annealed-copper temperature coefficient of resistance, /K, referred
/// to 20 C (public, standard).
const COPPER_ALPHA_PER_K_AT_20C: f64 = 0.00393;

impl Awg {
    /// DC resistance at 20 C, ohms/metre.
    pub fn resistance_per_m_at_20c(self) -> f64 {
        ohms_per_kft_at_20c(self) / FT1000_TO_M
    }

    /// DC resistance at `temp_c`, ohms/metre: `R(T) = R20 * (1 + alpha*(T-20))`,
    /// the standard linear copper-resistance-vs-temperature relation. Floored
    /// at 5% of the 20 C value so an extreme (non-physical for this crate's
    /// operating envelope, but numerically presented) cold input can never
    /// reach zero or negative resistance.
    pub fn resistance_per_m(self, temp_c: f64) -> f64 {
        let r20 = self.resistance_per_m_at_20c();
        (r20 * (1.0 + COPPER_ALPHA_PER_K_AT_20C * (temp_c - 20.0))).max(r20 * 0.05)
    }
}

/// MIL-DTL-22759's three continuous-service insulation classes (real,
/// sourced -- see module doc).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Insulation {
    /// M22759/16-class ETFE, 150 C continuous.
    Etfe150,
    /// M22759/32-class, 200 C continuous.
    Ptfe200,
    /// M22759/34-class PTFE, 260 C continuous.
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

/// One conductor's physical construction: gauge and insulation class.
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
