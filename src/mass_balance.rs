//! The A380's stations and tanks from FlyByWire's own `flight_model.cfg`,
//! and the moment arithmetic over them: what `weight_balance` writes to
//! X-Plane and what the deep fuel area reads, with nothing host-specific,
//! so the MSFS systems module compiles the same file.

pub const FLIGHT_MODEL_CFG: &str = include_str!(
    "../../fbw-aircraft/fbw-a380x/src/base/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380X/common/config/flight_model.cfg"
);

pub const LB_TO_KG: f64 = 0.453_592_37;
const FT_TO_M: f64 = 0.3048;

/// A mass at a position: pounds, and feet z (back to front), x, y from the
/// reference datum.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Mass {
    pub pounds: f64,
    pub position: [f64; 3],
}

/// The cfg's balance: the empty aircraft, the stations with their default
/// weights, and the tank positions.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Balance {
    pub datum: [f64; 3],
    pub empty: Mass,
    pub stations: Vec<Mass>,
    /// Each station's type (1-2 pilots, 3 passengers, 6 cargo).
    pub station_kinds: Vec<u32>,
    pub tanks: Vec<[f64; 3]>,
}

fn numbers(s: &str) -> Vec<f64> {
    s.split(',').filter_map(|v| v.trim().parse().ok()).collect()
}

fn position(v: &[f64]) -> [f64; 3] {
    [v.first().copied().unwrap_or(0.), v.get(1).copied().unwrap_or(0.), v.get(2).copied().unwrap_or(0.)]
}

/// flight_model.cfg's `[WEIGHT_AND_BALANCE]` and `[FUEL_SYSTEM]` tanks.
pub fn parse(cfg: &str) -> Balance {
    let mut balance = Balance::default();
    let mut stations = Vec::new();
    let mut tanks = Vec::new();
    for line in cfg.lines() {
        let line = line.split(';').next().unwrap_or("").trim();
        let Some((key, value)) = line.split_once('=') else { continue };
        let key = key.trim().to_ascii_lowercase();
        match key.as_str() {
            "reference_datum_position" => balance.datum = position(&numbers(value)),
            "empty_weight" => balance.empty.pounds = numbers(value).first().copied().unwrap_or(0.),
            "empty_weight_cg_position" => balance.empty.position = position(&numbers(value)),
            _ => {
                if let Some(n) = key.strip_prefix("station_load.").and_then(|n| n.parse::<usize>().ok()) {
                    // weight, z, x, y, name, type
                    let v = numbers(value);
                    // (the name is not a number, so the type follows y)
                    let kind = v.get(4).copied().unwrap_or(0.) as u32;
                    stations.push((n, Mass { pounds: v.first().copied().unwrap_or(0.), position: position(&v[1.min(v.len())..]) }, kind));
                } else if let Some(n) = key.strip_prefix("tank.").and_then(|n| n.parse::<usize>().ok()) {
                    let at = value.split('#').find_map(|f| f.trim().strip_prefix("Position:")).map(numbers).unwrap_or_default();
                    tanks.push((n, position(&at)));
                }
            }
        }
    }
    stations.sort_by_key(|s| s.0);
    tanks.sort_by_key(|t| t.0);
    balance.station_kinds = stations.iter().map(|s| s.2).collect();
    balance.stations = stations.into_iter().map(|s| s.1).collect();
    balance.tanks = tanks.into_iter().map(|t| t.1).collect();
    balance
}

/// X-Plane's station count (`sim/flightmodel/weight/m_stations` is float[9]).
pub const XPLANE_STATIONS: usize = 9;

/// The payload, in pounds, above which FlyByWire is taken to have actually
/// published one. Two crew at ~170 lb each clear this; the 2 lb an
/// unpopulated loadsheet reports does not.
pub const MIN_REPORTED_PAYLOAD_LB: f64 = 100.;

fn kind_class(kind: u32) -> u32 {
    // Captain and first officer (1, 2) are one kind.
    if kind == 2 { 1 } else { kind }
}

/// The cfg stations grouped into X-Plane's nine, as the converter groups them
/// for the .acf's stations (msfs2xp-aircraft stations.rs): stations at the
/// same z and y of one kind share one, then the two closest of one kind merge
/// (arms weighted by the cfg's weights) until nine are left.
pub fn station_groups(b: &Balance) -> Vec<Vec<usize>> {
    let kind = |i: usize| kind_class(b.station_kinds.get(i).copied().unwrap_or(0));
    let arm = |g: &[usize]| {
        let total: f64 = g.iter().map(|&i| b.stations[i].pounds.max(0.)).sum();
        let mut p = [0.; 3];
        for &i in g {
            let w = if total > 0. { b.stations[i].pounds.max(0.) / total } else { 1. / g.len() as f64 };
            for (k, v) in p.iter_mut().enumerate() {
                *v += b.stations[i].position[k] * w;
            }
        }
        p
    };
    let mut groups: Vec<Vec<usize>> = Vec::new();
    for (i, s) in b.stations.iter().enumerate() {
        match groups.iter_mut().find(|g| {
            let t = &b.stations[g[0]];
            t.position[0] == s.position[0] && t.position[2] == s.position[2] && kind(g[0]) == kind(i)
        }) {
            Some(g) => g.push(i),
            None => groups.push(vec![i]),
        }
    }
    while groups.len() > XPLANE_STATIONS {
        let closest = |same_kind: bool| {
            let mut best: Option<(f64, usize, usize)> = None;
            for a in 0..groups.len() {
                for c in a + 1..groups.len() {
                    if same_kind && kind(groups[a][0]) != kind(groups[c][0]) {
                        continue;
                    }
                    let (pa, pc) = (arm(&groups[a]), arm(&groups[c]));
                    let d = (pa[0] - pc[0]).abs() + (pa[2] - pc[2]).abs();
                    if best.is_none_or(|(bd, _, _)| d < bd) {
                        best = Some((d, a, c));
                    }
                }
            }
            best
        };
        let Some((_, a, c)) = closest(true).or_else(|| closest(false)) else { break };
        let moved = groups.remove(c);
        groups[a].extend(moved);
    }
    groups
}

/// Total pounds and the centre of gravity, feet from the datum.
pub fn centre_of_gravity(masses: impl IntoIterator<Item = Mass>) -> (f64, [f64; 3]) {
    let mut total = 0.;
    let mut moment = [0.; 3];
    for m in masses {
        total += m.pounds;
        for (k, p) in m.position.iter().enumerate() {
            moment[k] += m.pounds * p;
        }
    }
    if total <= 0. {
        return (0., [0.; 3]);
    }
    (total, moment.map(|m| m / total))
}

/// X-Plane's `cg_offset_z` for an MSFS CG: metres aft of the .acf's reference
/// point, which is `reference_ft` along X-Plane's z (aft).
pub fn xplane_cg_offset_z(msfs_cg_z_ft: f64, datum_z_ft: f64, reference_ft: f64) -> f64 {
    (-(msfs_cg_z_ft + datum_z_ft) - reference_ft) * FT_TO_M
}

/// The empty aircraft's mass, at the position the .acf's own declared
/// reference point (`acf_cgZ_original`) actually puts it, instead of the
/// cfg's raw `empty_weight_cg_position`.
///
/// `xplane_cg_offset_z` writes an offset *from* the reference point, and
/// X-Plane adds the two back together (`cg_offset_z`'s own DataRefs.txt
/// wording: "offset ... from the reference point"), which algebraically
/// cancels `reference_ft` out of the final applied CG --
/// `reference_ft + xplane_cg_offset_z(cg, datum, reference_ft) ==
/// -(cg + datum)` for any `reference_ft`. So if the centre-of-gravity sum
/// here still anchored on the cfg's raw empty position while the reference
/// point had been moved (msfs2xp-aircraft's `--cg-z`, used on the installed
/// A380 to replace the cfg's -16 ft with the certificated -8 ft: main.rs's
/// own doc comment on that flag), every tick silently discarded that
/// correction and put the aircraft back at the cfg's distrusted figure,
/// reproducing the exact "16% of the weight on the nose leg, would not
/// rotate" fault `--cg-z` exists to prevent. Re-deriving the empty position
/// from whatever the .acf actually declares keeps the two in step
/// automatically, with or without an override, without duplicating the
/// certificated figure in this crate.
///
/// `reference_ft` is `None` only when X-Plane has no
/// `sim/aircraft/weight/acf_cgZ_original` (dataref not found); the cfg's own
/// figure is the least-wrong fallback then, as before this fix.
pub fn corrected_empty(empty: Mass, datum_z_ft: f64, reference_ft: Option<f64>) -> Mass {
    match reference_ft {
        Some(r) => Mass { pounds: empty.pounds, position: [-r - datum_z_ft, empty.position[1], empty.position[2]] },
        None => empty,
    }
}
