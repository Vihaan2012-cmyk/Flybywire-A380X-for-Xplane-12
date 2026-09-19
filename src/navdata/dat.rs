//! Line parsers for X-Plane's global navigation data, following Laminar's
//! specifications: XP-NAV1200 (earth_nav.dat), XP-FIX1200 (earth_fix.dat),
//! XP-AWY1101 (earth_awy.dat) and XP-HOLD1140 (earth_hold.dat).
//!
//! Every parser takes one line and returns `None` for headers, the `99`
//! terminator, and anything malformed.

use super::icao::{region, Code};

/// Splits a line into its first `n` whitespace separated fields and the
/// remainder (for names, which may contain spaces).
fn fields(line: &str, n: usize) -> Option<(Vec<&str>, &str)> {
    let mut out = Vec::with_capacity(n);
    let mut rest = line.trim_start();
    while out.len() < n {
        if rest.is_empty() {
            return None;
        }
        let end = rest.find(|c: char| c.is_ascii_whitespace()).unwrap_or(rest.len());
        out.push(&rest[..end]);
        rest = rest[end..].trim_start();
    }
    Some((out, rest.trim_end()))
}

/// `ENRT` means no airport.
fn airport(s: &str) -> Code {
    if s == "ENRT" {
        Code::EMPTY
    } else {
        Code::lossy(s)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum NavRecord {
    /// Row 2.
    Ndb { lat: f64, lon: f64, elev_ft: f32, freq_khz: u32, class: u16, bfo: bool, ident: Code, airport: Code, region: [u8; 2], name: String },
    /// Row 3: VOR, VOR/DME, VORTAC or TACAN (by the name's suffix).
    Vor { lat: f64, lon: f64, elev_ft: f32, freq: u32, class: u16, slaved_var: f32, ident: Code, region: [u8; 2], name: String },
    /// Rows 4 (ILS) and 5 (LOC, LDA, SDF).
    Loc { row: u8, lat: f64, lon: f64, elev_ft: f32, freq: u32, true_bearing: f64, mag_course: Option<f64>, ident: Code, airport: Code, region: [u8; 2], runway: Code, name: String },
    /// Row 6.
    Gs { lat: f64, lon: f64, elev_ft: f32, angle: f64, ident: Code, airport: Code },
    /// Rows 12 and 13.
    Dme { row: u8, lat: f64, lon: f64, elev_ft: f32, freq: u32, class: u16, bias_nm: f32, ident: Code, airport: Code, region: [u8; 2], name: String },
}

pub fn nav(line: &str) -> Option<NavRecord> {
    let row: u8 = line.split_ascii_whitespace().next()?.parse().ok()?;
    let num = |s: &str| s.parse::<f64>().ok();
    match row {
        2 | 3 | 12 | 13 => {
            let (f, name) = fields(line, 10)?;
            let (lat, lon, elev) = (num(f[1])?, num(f[2])?, num(f[3])? as f32);
            let (freq, class, extra) = (num(f[4])? as u32, num(f[5])? as u16, num(f[6])?);
            let (ident, apt, reg) = (Code::lossy(f[7]), airport(f[8]), region(f[9]));
            let name = name.to_string();
            Some(match row {
                2 => NavRecord::Ndb { lat, lon, elev_ft: elev, freq_khz: freq, class, bfo: extra >= 1., ident, airport: apt, region: reg, name },
                3 => NavRecord::Vor { lat, lon, elev_ft: elev, freq, class, slaved_var: extra as f32, ident, region: reg, name },
                _ => NavRecord::Dme { row, lat, lon, elev_ft: elev, freq, class, bias_nm: extra as f32, ident, airport: apt, region: reg, name },
            })
        }
        4 | 5 => {
            let (f, name) = fields(line, 11)?;
            let encoded = num(f[6])?;
            // True bearing, plus the integer magnetic front course times 360.
            let true_bearing = encoded.rem_euclid(360.);
            let mag_course = (encoded >= 360.).then(|| (encoded / 360.).floor());
            Some(NavRecord::Loc {
                row,
                lat: num(f[1])?,
                lon: num(f[2])?,
                elev_ft: num(f[3])? as f32,
                freq: num(f[4])? as u32,
                true_bearing,
                mag_course,
                ident: Code::lossy(f[7]),
                airport: airport(f[8]),
                region: region(f[9]),
                runway: Code::lossy(f[10]),
                name: name.to_string(),
            })
        }
        6 => {
            let (f, _) = fields(line, 11)?;
            let encoded = num(f[6])?;
            Some(NavRecord::Gs {
                lat: num(f[1])?,
                lon: num(f[2])?,
                elev_ft: num(f[3])? as f32,
                // Angle in hundredths of a degree times 1000 (so, degrees
                // times 100 000), plus the true bearing.
                angle: (encoded / 1000.).floor() / 100.,
                ident: Code::lossy(f[7]),
                airport: airport(f[8]),
            })
        }
        _ => None,
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct FixRecord {
    pub lat: f64,
    pub lon: f64,
    pub ident: Code,
    pub airport: Code,
    pub region: [u8; 2],
    /// ARINC 424 field 5.42, three columns.
    pub wtype: [u8; 3],
}

pub fn fix(line: &str) -> Option<FixRecord> {
    let (f, _) = fields(line, 5)?;
    let lat = f[0].parse().ok()?;
    let lon = f[1].parse().ok()?;
    // The waypoint type is the three ARINC bytes as a little endian integer.
    let mut wtype = [b' '; 3];
    if let Some(v) = line.split_ascii_whitespace().nth(5).and_then(|s| s.parse::<u32>().ok()) {
        let b = v.to_le_bytes();
        for i in 0..3 {
            wtype[i] = if b[i] == 0 { b' ' } else { b[i] };
        }
    }
    Some(FixRecord { lat, lon, ident: Code::lossy(f[2]), airport: airport(f[3]), region: region(f[4]), wtype })
}

#[derive(Clone, Debug, PartialEq)]
pub struct AwyRecord {
    pub from: (Code, [u8; 2], u8),
    pub to: (Code, [u8; 2], u8),
    /// `N`, `F` or `B`.
    pub direction: u8,
    /// 1 low, 2 high.
    pub level: u8,
    pub base_ft: i32,
    pub top_ft: i32,
    pub names: Vec<String>,
}

pub fn awy(line: &str) -> Option<AwyRecord> {
    let (f, _) = fields(line, 11)?;
    let end = |i: usize| -> Option<(Code, [u8; 2], u8)> { Some((Code::lossy(f[i]), region(f[i + 1]), f[i + 2].parse().ok()?)) };
    Some(AwyRecord {
        from: end(0)?,
        to: end(3)?,
        direction: *f[6].as_bytes().first()?,
        level: f[7].parse().ok()?,
        base_ft: f[8].parse::<i32>().ok()? * 100,
        top_ft: f[9].parse::<i32>().ok()? * 100,
        names: f[10].split('-').filter(|s| !s.is_empty()).map(str::to_string).collect(),
    })
}

#[derive(Clone, Debug, PartialEq)]
pub struct HoldRecord {
    pub ident: Code,
    pub region: [u8; 2],
    pub airport: Code,
    /// 11 fix, 2 NDB, 3 VHF navaid.
    pub fix_type: u8,
    pub inbound_course: f32,
    pub leg_time_min: f32,
    pub leg_length_nm: f32,
    pub turn_right: bool,
    pub min_alt_ft: i32,
    pub max_alt_ft: i32,
    pub speed_kt: i32,
}

pub fn hold(line: &str) -> Option<HoldRecord> {
    let (f, _) = fields(line, 11)?;
    Some(HoldRecord {
        ident: Code::lossy(f[0]),
        region: region(f[1]),
        airport: airport(f[2]),
        fix_type: f[3].parse().ok()?,
        inbound_course: f[4].parse().ok()?,
        leg_time_min: f[5].parse().ok()?,
        leg_length_nm: f[6].parse().ok()?,
        turn_right: f[7] == "R",
        min_alt_ft: f[8].parse().ok()?,
        max_alt_ft: f[9].parse().ok()?,
        speed_kt: f[10].parse().ok()?,
    })
}

/// A row of earth_aptmeta.dat. Laminar has not published this file's
/// specification, so only its self-evident columns are read: ident, region,
/// reference point, and the transition altitude and level (feet, `FLnnn`,
/// or -1 for none).
#[derive(Clone, Debug, PartialEq)]
pub struct AptMetaRecord {
    pub ident: Code,
    pub region: [u8; 2],
    pub lat: f64,
    pub lon: f64,
    pub ta_ft: i32,
    pub tl_ft: i32,
}

pub fn apt_meta(line: &str) -> Option<AptMetaRecord> {
    let (f, _) = fields(line, 10)?;
    let alt = |s: &str| -> Option<i32> {
        match s.strip_prefix("FL") {
            Some(fl) => fl.parse::<i32>().ok().map(|v| v * 100),
            None => s.parse::<i32>().ok().map(|v| v.max(0)),
        }
    };
    Some(AptMetaRecord {
        ident: Code::new(f[0])?,
        region: region(f[1]),
        lat: f[2].parse().ok()?,
        lon: f[3].parse().ok()?,
        ta_ft: alt(f[8])?,
        tl_ft: alt(f[9])?,
    })
}

/// The AIRAC cycle named on a file's version line ("... data cycle 2406, ...").
pub fn cycle(header: &str) -> Option<u32> {
    let at = header.find("data cycle")?;
    header[at + 10..].trim_start().get(..4)?.parse().ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    // Lines copied from X-Plane 12's installed data (cycle 2406).
    #[test]
    fn earth_nav_rows() {
        let vor = nav(" 3   9.037805556    7.285111111     1191    11630   130     -0.000  ABC ENRT DN ABUJA VOR/DME").unwrap();
        let NavRecord::Vor { freq, class, ident, region, name, .. } = vor else { panic!() };
        assert_eq!((freq, class, ident.as_str(), &region, name.as_str()), (11630, 130, "ABC", b"DN", "ABUJA VOR/DME"));

        let loc = nav(" 4  51.464761111   -0.491166667       77    10950    18  97109.717  ILL EGLL EG 27L ILS-cat-III").unwrap();
        let NavRecord::Loc { true_bearing, mag_course, ident, airport, runway, name, .. } = loc else { panic!() };
        assert!((true_bearing - 269.717).abs() < 1e-6);
        assert_eq!(mag_course, Some(269.));
        assert_eq!((ident.as_str(), airport.as_str(), runway.as_str(), name.as_str()), ("ILL", "EGLL", "27L", "ILS-cat-III"));

        let gs = nav(" 6  51.463741667   -0.438888889       77    10950    18 300269.717  ILL EGLL EG 27L GS").unwrap();
        let NavRecord::Gs { angle, .. } = gs else { panic!() };
        assert!((angle - 3.0).abs() < 1e-9);

        let dme = nav("12  51.463816667   -0.458583333       93    10950    25      0.900  ILL EGLL EG HEATHROW DME-ILS").unwrap();
        let NavRecord::Dme { bias_nm, airport, .. } = dme else { panic!() };
        assert_eq!((bias_nm, airport.as_str()), (0.9, "EGLL"));

        let ndb = nav(" 2 -10.366666667   56.600000000        0      429    75      0.000  AGG ENRT FI AGALEGA NDB").unwrap();
        let NavRecord::Ndb { freq_khz, class, airport, .. } = ndb else { panic!() };
        assert_eq!((freq_khz, class, airport), (429, 75, Code::EMPTY));

        assert_eq!(nav("99"), None);
        assert_eq!(nav("1200 Version - data cycle 2406, build 20251002, metadata NavXP1200."), None);
    }

    #[test]
    fn earth_fix_awy_hold_and_meta_rows() {
        let f = fix(" 51.477902778   -0.353233333  30LOC EGLL EG 4608073 IRR089005").unwrap();
        assert_eq!((f.ident.as_str(), f.airport.as_str(), &f.region), ("30LOC", "EGLL", b"EG"));
        // 4608073 = 0x465049: 'I', 'P', 'F'.
        assert_eq!(&f.wtype, b"IPF");
        let e = fix(" 33.492513889    9.217400000  07EBA ENRT DT 2118994 EBA357107").unwrap();
        assert_eq!((e.airport, &e.wtype), (Code::EMPTY, b"RU "));

        let a = awy("07EBA DT 11 GILEX DT 11 N 1  95 245 G869").unwrap();
        assert_eq!((a.from.0.as_str(), a.to.0.as_str(), a.to.2, a.direction, a.level, a.base_ft, a.top_ft), ("07EBA", "GILEX", 11, b'N', 1, 9500, 24500));
        assert_eq!(a.names, vec!["G869"]);

        let h = hold("DUFFY K6 KJFK 11    242.0      1.0      0.0 L     3000        0        0").unwrap();
        assert_eq!((h.airport.as_str(), h.inbound_course, h.leg_time_min, h.turn_right, h.min_alt_ft), ("KJFK", 242., 1., false, 3000));

        let m = apt_meta("KJFK K6  40.639927778  -73.778691667    13 C 14500 I 18000 FL180").unwrap();
        assert_eq!((m.ident.as_str(), m.ta_ft, m.tl_ft), ("KJFK", 18000, 18000));
        let m = apt_meta("EGLL EG  51.477500000   -0.461388889    83 C 12700 I  6000    -1").unwrap();
        assert_eq!((m.ta_ft, m.tl_ft), (6000, 0));

        assert_eq!(cycle("1200 Version - data cycle 2406, build 20251002, metadata NavXP1200."), Some(2406));
    }
}
