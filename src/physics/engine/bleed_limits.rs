//! The Trent 900's maximum permissible customer bleed, EASA TCDS E.012
//! Issue 12 section 10: a fraction of the core flow at the port the air
//! comes off (HP6: %W26, the HP compressor's inlet flow; IP8: %W24, the IP
//! compressor's), scheduled against turbine entry temperature T41 and
//! varying linearly between the listed points. A port the data sheet marks
//! n/a at a temperature is not a permitted source there at all.
//!
//! The HP6 tables' first row runs from "low idle", which has no stated
//! temperature, to the first listed one (normal: 12.5% falling to 12%).
//! Without the low-idle T41 the slope is unknown, so below the first listed
//! temperature the row's lower end holds: never more than the data sheet
//! permits anywhere on that row.

/// Which customer bleed port the air comes off.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Port {
    Ip8,
    Hp6,
}

/// Normal operation: four bleeds and two packs. Abnormal: two bleeds and
/// one pack.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Configuration {
    Normal,
    Abnormal,
}

// (T41 K, % of the port's core flow): each row's two ends. A port's table
// covers only the temperatures its rows list.
const HP6_NORMAL: &[(f64, f64)] = &[(1251.0, 12.0), (1545.0, 5.5)];
const IP8_NORMAL: &[(f64, f64)] = &[(1545.0, 5.11), (1672.0, 4.1), (1787.0, 1.95)];
const HP6_ABNORMAL: &[(f64, f64)] = &[(1272.0, 14.35), (1406.0, 13.25), (1662.0, 6.6)];
const IP8_ABNORMAL: &[(f64, f64)] = &[(1662.0, 6.35), (1803.0, 2.6)];

/// The most customer bleed, kg/s, the data sheet permits from `port` at
/// turbine entry temperature `tet_k`, given that port's core flow
/// (`core_flow_kg_s`: W24 for IP8, W26 for HP6). `None` where the data sheet
/// does not permit bleeding from that port at that temperature.
pub fn max_customer_bleed_kg_s(port: Port, configuration: Configuration, tet_k: f64, core_flow_kg_s: f64) -> Option<f64> {
    let pct = match (port, configuration) {
        (Port::Hp6, Configuration::Normal) => hp6(tet_k, HP6_NORMAL),
        (Port::Hp6, Configuration::Abnormal) => hp6(tet_k, HP6_ABNORMAL),
        (Port::Ip8, Configuration::Normal) => ip8(tet_k, IP8_NORMAL),
        (Port::Ip8, Configuration::Abnormal) => ip8(tet_k, IP8_ABNORMAL),
    }?;
    Some(pct / 100.0 * core_flow_kg_s.max(0.0))
}

/// The limit for `port` when the data sheet lists that port at `tet_k`, and
/// otherwise its table held at the nearest listed end, with whether it was
/// listed. The aircraft picks the port by pressure (section 10's own rule),
/// so a model whose port pressures and T41 do not line up the way the real
/// engine's do can draw from a port outside the temperatures its table
/// covers: that is a calibration gap to report, not a bleed exceedance.
pub fn customer_bleed_limit_kg_s(port: Port, configuration: Configuration, tet_k: f64, core_flow_kg_s: f64) -> (f64, bool) {
    match max_customer_bleed_kg_s(port, configuration, tet_k, core_flow_kg_s) {
        Some(limit) => (limit, true),
        None => {
            let table = match (port, configuration) {
                (Port::Hp6, Configuration::Normal) => HP6_NORMAL,
                (Port::Hp6, Configuration::Abnormal) => HP6_ABNORMAL,
                (Port::Ip8, Configuration::Normal) => IP8_NORMAL,
                (Port::Ip8, Configuration::Abnormal) => IP8_ABNORMAL,
            };
            let (first_t, first_pct) = table[0];
            let (_, last_pct) = table[table.len() - 1];
            let pct = if tet_k < first_t { first_pct } else { last_pct };
            (pct / 100.0 * core_flow_kg_s.max(0.0), false)
        }
    }
}

/// HP6 runs from low idle up to its last listed temperature.
fn hp6(tet_k: f64, table: &[(f64, f64)]) -> Option<f64> {
    let (first_t, first_pct) = table[0];
    if tet_k <= first_t {
        return Some(first_pct);
    }
    within(tet_k, table)
}

/// IP8 runs from its first listed temperature up to maximum take-off, its
/// last value held above the last point.
fn ip8(tet_k: f64, table: &[(f64, f64)]) -> Option<f64> {
    let (first_t, _) = table[0];
    let (last_t, last_pct) = table[table.len() - 1];
    if tet_k < first_t {
        None
    } else if tet_k >= last_t {
        Some(last_pct)
    } else {
        within(tet_k, table)
    }
}

fn within(tet_k: f64, table: &[(f64, f64)]) -> Option<f64> {
    table.windows(2).find(|w| tet_k >= w[0].0 && tet_k <= w[1].0).map(|w| {
        let (t0, p0) = w[0];
        let (t1, p1) = w[1];
        p0 + (p1 - p0) * (tet_k - t0) / (t1 - t0)
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pct(port: Port, configuration: Configuration, tet_k: f64) -> Option<f64> {
        max_customer_bleed_kg_s(port, configuration, tet_k, 100.0)
    }

    #[test]
    fn the_listed_points_come_back_exactly() {
        use Configuration::*;
        use Port::*;
        assert_eq!(pct(Hp6, Normal, 1000.0), Some(12.0));
        assert!((pct(Hp6, Normal, 1251.0).unwrap() - 12.0).abs() < 1e-9);
        assert!((pct(Hp6, Normal, 1545.0).unwrap() - 5.5).abs() < 1e-9);
        assert!((pct(Ip8, Normal, 1545.0).unwrap() - 5.11).abs() < 1e-9);
        assert!((pct(Ip8, Normal, 1672.0).unwrap() - 4.1).abs() < 1e-9);
        assert!((pct(Ip8, Normal, 1787.0).unwrap() - 1.95).abs() < 1e-9);
        assert_eq!(pct(Ip8, Normal, 1900.0), Some(1.95));
        assert_eq!(pct(Hp6, Abnormal, 1100.0), Some(14.35));
        assert!((pct(Hp6, Abnormal, 1406.0).unwrap() - 13.25).abs() < 1e-9);
        assert!((pct(Ip8, Abnormal, 1662.0).unwrap() - 6.35).abs() < 1e-9);
        assert_eq!(pct(Ip8, Abnormal, 1850.0), Some(2.6));
    }

    #[test]
    fn a_port_marked_not_applicable_permits_nothing() {
        assert_eq!(pct(Port::Hp6, Configuration::Normal, 1600.0), None);
        assert_eq!(pct(Port::Ip8, Configuration::Normal, 1400.0), None);
        assert_eq!(pct(Port::Hp6, Configuration::Abnormal, 1700.0), None);
        assert_eq!(pct(Port::Ip8, Configuration::Abnormal, 1600.0), None);
    }

    #[test]
    fn outside_its_listed_temperatures_a_port_holds_its_nearest_end_and_says_so() {
        let (limit, listed) = customer_bleed_limit_kg_s(Port::Ip8, Configuration::Normal, 1400.0, 100.0);
        assert!(!listed && (limit - 5.11).abs() < 1e-9);
        let (limit, listed) = customer_bleed_limit_kg_s(Port::Hp6, Configuration::Normal, 1700.0, 100.0);
        assert!(!listed && (limit - 5.5).abs() < 1e-9);
        assert_eq!(customer_bleed_limit_kg_s(Port::Hp6, Configuration::Normal, 1300.0, 100.0).1, true);
    }

    #[test]
    fn it_varies_linearly_between_points() {
        let mid = pct(Port::Hp6, Configuration::Normal, (1251.0 + 1545.0) / 2.0).unwrap();
        assert!((mid - (12.0 + 5.5) / 2.0).abs() < 1e-9);
    }
}
