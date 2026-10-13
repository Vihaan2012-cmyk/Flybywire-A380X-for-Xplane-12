//! Payload stations: MSFS's `station_load.N` grouped into X-Plane's nine.
//!
//! flight_model.cfg lists a station as `weight, longitudinal, lateral,
//! vertical, name, type` (feet from the datum; type 1-2 pilots, 3 passengers,
//! 6 cargo). X-Plane has at most nine (`sim/flightmodel/weight/m_stations`
//! is float[9]), so stations at the same longitudinal and vertical arm share
//! one, then the two closest of the same kind merge until nine are left. The
//! systems plugin groups the same way (fbw-xp-systems weight_balance.rs) and
//! writes each group's weight; it also sets the whole aircraft's centre of
//! gravity itself, so a group's arm only has to be where its load sits.

/// A cfg station.
#[derive(Debug, Clone, PartialEq)]
pub struct Station {
    pub max_lb: f64,
    /// Longitudinal, lateral, vertical (feet from the datum).
    pub arm: [f64; 3],
    pub name: String,
    pub kind: u32,
}

/// X-Plane's station count.
pub const XPLANE_STATIONS: usize = 9;

/// `station_load.N` values, by N.
pub fn parse(values: &[(usize, &str)]) -> Vec<Station> {
    let mut sorted: Vec<_> = values.to_vec();
    sorted.sort_by_key(|(n, _)| *n);
    sorted
        .into_iter()
        .filter_map(|(_, v)| {
            let f: Vec<&str> = v.split(',').map(str::trim).collect();
            let n = |i: usize| f.get(i).and_then(|x| x.parse::<f64>().ok());
            Some(Station {
                max_lb: n(0)?,
                arm: [n(1)?, n(2)?, n(3)?],
                name: f.get(4).copied().unwrap_or("").to_string(),
                kind: f.get(5).and_then(|x| x.parse().ok()).unwrap_or(0),
            })
        })
        .collect()
}

fn kind_class(kind: u32) -> u32 {
    // Captain and first officer (1, 2) are one kind.
    if kind == 2 { 1 } else { kind }
}

/// Groups of cfg station indices, at most `XPLANE_STATIONS`, in cfg order.
pub fn group(stations: &[Station]) -> Vec<Vec<usize>> {
    let mut groups: Vec<Vec<usize>> = Vec::new();
    for (i, s) in stations.iter().enumerate() {
        match groups.iter_mut().find(|g| {
            let t = &stations[g[0]];
            t.arm[0] == s.arm[0] && t.arm[2] == s.arm[2] && kind_class(t.kind) == kind_class(s.kind)
        }) {
            Some(g) => g.push(i),
            None => groups.push(vec![i]),
        }
    }
    while groups.len() > XPLANE_STATIONS {
        let mut best: Option<(f64, usize, usize)> = None;
        for a in 0..groups.len() {
            for b in a + 1..groups.len() {
                if kind_class(stations[groups[a][0]].kind) != kind_class(stations[groups[b][0]].kind) {
                    continue;
                }
                let (pa, pb) = (arm(stations, &groups[a]), arm(stations, &groups[b]));
                let d = (pa[0] - pb[0]).abs() + (pa[2] - pb[2]).abs();
                if best.is_none_or(|(bd, _, _)| d < bd) {
                    best = Some((d, a, b));
                }
            }
        }
        // Nothing of one kind left to merge: the closest of any kinds.
        let (_, a, b) = best.unwrap_or_else(|| {
            let mut any = (f64::INFINITY, 0, 1);
            for a in 0..groups.len() {
                for b in a + 1..groups.len() {
                    let (pa, pb) = (arm(stations, &groups[a]), arm(stations, &groups[b]));
                    let d = (pa[0] - pb[0]).abs() + (pa[2] - pb[2]).abs();
                    if d < any.0 {
                        any = (d, a, b);
                    }
                }
            }
            any
        });
        let moved = groups.remove(b);
        groups[a].extend(moved);
    }
    groups
}

/// A group's arm: its stations' arms weighted by their maximum loads.
pub fn arm(stations: &[Station], group: &[usize]) -> [f64; 3] {
    let total: f64 = group.iter().map(|&i| stations[i].max_lb.max(0.0)).sum();
    let mut p = [0.0; 3];
    for &i in group {
        let w = if total > 0.0 { stations[i].max_lb.max(0.0) / total } else { 1.0 / group.len() as f64 };
        for k in 0..3 {
            p[k] += stations[i].arm[k] * w;
        }
    }
    p
}

#[cfg(test)]
mod tests {
    use super::*;

    // FlyByWire's A380X flight_model.cfg:32-57.
    const A380: [&str; 19] = [
        "5185.3, 75.7, 0, 7.1, MAIN DECK FWD A, 3",
        "5185.3, 75.7, 0, 7.1, MAIN DECK FWD B, 3",
        "7222.4, 30.6, 0, 7.1, MAIN DECK MID 1A, 3",
        "9259.4, 30.6, 0, 7.1, MAIN DECK MID 1B, 3",
        "7963.1, 30.6, 0, 7.1, MAIN DECK MID 1C, 3",
        "8889.0, -11.1, 0, 7.1, MAIN DECK MID 2A, 3",
        "7407.5, -11.1, 0, 7.1, MAIN DECK MID 2B, 3",
        "6666.8, -11.1, 0, 7.1, MAIN DECK MID 2C, 3",
        "7777.9, -46.9, 0, 7.1, MAIN DECK AFT A, 3",
        "7407.5, -46.9, 0, 7.1, MAIN DECK AFT B, 3",
        "2592.6, 60.8, 0, 15.6, UPPER DECK FWD, 3",
        "5370.5, 11.7, 0, 15.6, UPPER DECK MID A, 3",
        "5370.5, 11.7, 0, 15.6, UPPER DECK MID B, 3",
        "3334.1, -29.3, 0, 15.6, UPPER DECK AFT, 3",
        "63000, 67.4, 0, -0.95, FWD CARGO HOLD, 6",
        "44775, -18.5, 0, -0.95, AFT CARGO HOLD, 6",
        "5540, -52.9, 0, -0.71, BULK CARGO, 6",
        "1, 105.3, -1.95, 7.1, CAPTAIN, 1",
        "1, 105.3, 1.95, 7.1, FIRST OFFICER, 2",
    ];

    #[test]
    fn the_a380s_nineteen_stations_fit_in_nine_of_their_own_kind() {
        let values: Vec<(usize, &str)> = A380.iter().copied().enumerate().collect();
        let s = parse(&values);
        let g = group(&s);
        assert_eq!(
            g,
            vec![vec![0, 1, 10], vec![2, 3, 4], vec![5, 6, 7], vec![8, 9, 13], vec![11, 12], vec![14], vec![15], vec![16], vec![17, 18]]
        );
        // No group mixes passengers, cargo and crew.
        for grp in &g {
            assert!(grp.iter().all(|&i| kind_class(s[i].kind) == kind_class(s[grp[0]].kind)));
        }
        let crew = arm(&s, &g[8]);
        assert_eq!(crew, [105.3, 0.0, 7.1]);
    }
}
