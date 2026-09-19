//! Terminal procedures from CIFP/<ICAO>.dat, following Laminar's XP-CIFP
//! specification: each SID, STAR or APPCH row is one ARINC 424 procedure
//! leg (fields as ARINC 424.18 4.1.9.1), RWY rows are runway records
//! (4.1.10.1) and PRDAT rows are procedure data continuation records.
//!
//! Column numbers below are the specification's, 1-based after the row code.

use super::icao::{region, Code};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FixRef {
    pub ident: Code,
    pub region: [u8; 2],
    /// ARINC section and subsection codes (`D`/` ` VHF navaid, `D`/`B` NDB,
    /// `E`/`A` waypoint, `P`/`C` terminal waypoint, `P`/`N` terminal NDB,
    /// `P`/`G` runway, `P`/`A` airport, `P`/`I` localizer).
    pub section: u8,
    pub subsection: u8,
}

impl FixRef {
    pub fn is_empty(&self) -> bool {
        self.ident.is_empty()
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Leg {
    pub seq: u16,
    /// ARINC 5.7 route type.
    pub route_type: u8,
    pub transition: String,
    pub fix: FixRef,
    /// ARINC 5.17 waypoint description code, four columns.
    pub desc: [u8; 4],
    /// `L`, `R`, `E` or blank.
    pub turn: u8,
    pub rnp_nm: Option<f64>,
    /// ARINC 5.21 path and termination.
    pub path: [u8; 2],
    pub turn_valid: bool,
    pub navaid: FixRef,
    pub arc_radius_nm: Option<f64>,
    pub theta: Option<f64>,
    pub rho_nm: Option<f64>,
    pub course: Option<f64>,
    pub course_true: bool,
    /// Distance in NM, or time in minutes when `minutes`.
    pub distance: Option<f64>,
    pub minutes: bool,
    /// ARINC 5.29 altitude description.
    pub alt_desc: u8,
    pub alt1_ft: Option<i32>,
    pub alt2_ft: Option<i32>,
    pub transition_alt_ft: Option<i32>,
    /// ARINC 5.261 speed limit description.
    pub speed_desc: u8,
    pub speed_kt: Option<i32>,
    pub vertical_angle: Option<f64>,
    pub center: FixRef,
    pub qualifier1: u8,
    pub qualifier2: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Sid,
    Star,
    Approach,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Procedure {
    pub kind: Kind,
    pub ident: String,
    pub legs: Vec<Leg>,
    /// Levels of service from PRDAT rows: (authorised, name).
    pub service: Vec<(bool, String)>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct RunwayRecord {
    /// "RW09L".
    pub id: Code,
    pub threshold_elev_ft: Option<i32>,
    pub ls_ident: Code,
    /// ARINC 5.80.
    pub ls_category: u8,
    pub threshold: Option<(f64, f64)>,
    pub displaced_ft: Option<i32>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Cifp {
    pub procedures: Vec<Procedure>,
    pub runways: Vec<RunwayRecord>,
}

fn opt_num<T: std::str::FromStr>(s: &str) -> Option<T> {
    let s = s.trim();
    if s.is_empty() {
        None
    } else {
        s.parse().ok()
    }
}

fn byte(s: &str) -> u8 {
    s.as_bytes().first().copied().unwrap_or(b' ')
}

/// ARINC 5.30 altitude: feet, or `FLnnn`.
fn altitude(s: &str) -> Option<i32> {
    let s = s.trim();
    match s.strip_prefix("FL") {
        Some(fl) => fl.parse::<i32>().ok().map(|v| v * 100),
        None => opt_num(s),
    }
}

/// ARINC 5.36/5.37 coordinates: `N51275382`, `W000260276` (degrees,
/// minutes, seconds and hundredths).
fn coordinate(s: &str) -> Option<f64> {
    let s = s.trim();
    let hemi = *s.as_bytes().first()?;
    let digits = &s[1..];
    let deg_len = if matches!(hemi, b'N' | b'S') { 2 } else { 3 };
    if digits.len() < deg_len + 6 {
        return None;
    }
    let d: f64 = digits[..deg_len].parse().ok()?;
    let m: f64 = digits[deg_len..deg_len + 2].parse().ok()?;
    let sec: f64 = digits[deg_len + 2..deg_len + 6].parse::<f64>().ok()? / 100.;
    let v = d + m / 60. + sec / 3600.;
    Some(if matches!(hemi, b'S' | b'W') { -v } else { v })
}

fn fix_ref(ident: &str, reg: &str, section: &str, subsection: &str) -> FixRef {
    FixRef { ident: Code::lossy(ident), region: region(reg), section: byte(section), subsection: byte(subsection) }
}

/// Parses one procedure row's fields (after `SID:` and so on).
pub fn leg(payload: &str) -> Option<Leg> {
    let f: Vec<&str> = payload.split(';').next()?.split(',').collect();
    if f.len() < 34 {
        return None;
    }
    let c = |i: usize| f.get(i - 1).copied().unwrap_or("");
    let desc = {
        let mut d = [b' '; 4];
        for (i, b) in c(9).bytes().take(4).enumerate() {
            d[i] = b;
        }
        d
    };
    let path = {
        let b = c(12).trim().as_bytes();
        if b.len() != 2 {
            return None;
        }
        [b[0], b[1]]
    };
    // 5.26: tenths of a degree magnetic, or whole degrees with a T for true.
    let course_text = c(21).trim();
    let (course, course_true) = match course_text.strip_suffix('T') {
        Some(t) => (opt_num::<f64>(t), true),
        None => (opt_num::<f64>(course_text).map(|v| v / 10.), false),
    };
    // 5.27: tenths of a NM, or T and tenths of a minute.
    let dist_text = c(22).trim();
    let (distance, minutes) = match dist_text.strip_prefix('T') {
        Some(t) => (opt_num::<f64>(t).map(|v| v / 10.), true),
        None => (opt_num::<f64>(dist_text).map(|v| v / 10.), false),
    };
    // 5.211: two digits and a negative power of ten.
    let rnp_nm = {
        let t = c(11).trim();
        (t.len() == 3).then(|| -> Option<f64> { Some(t[..2].parse::<f64>().ok()? * 10f64.powi(-t[2..].parse::<i32>().ok()?)) }).flatten()
    };
    Some(Leg {
        seq: opt_num(c(1))?,
        route_type: byte(c(2)),
        transition: c(4).trim().to_string(),
        fix: fix_ref(c(5), c(6), c(7), c(8)),
        desc,
        turn: byte(c(10)),
        rnp_nm,
        path,
        turn_valid: c(13).trim() == "Y",
        navaid: fix_ref(c(14), c(15), c(16), c(17)),
        arc_radius_nm: opt_num::<f64>(c(18)).map(|v| v / 1000.),
        theta: opt_num::<f64>(c(19)).map(|v| v / 10.),
        rho_nm: opt_num::<f64>(c(20)).map(|v| v / 10.),
        course,
        course_true,
        distance,
        minutes,
        alt_desc: byte(c(23)),
        alt1_ft: altitude(c(24)),
        alt2_ft: altitude(c(25)),
        transition_alt_ft: altitude(c(26)),
        speed_desc: byte(c(27)),
        speed_kt: opt_num(c(28)),
        // 5.70: hundredths of a degree.
        vertical_angle: opt_num::<f64>(c(29)).map(|v| v / 100.),
        center: fix_ref(c(31), c(32), c(33), c(34)),
        qualifier1: byte(c(37)),
        qualifier2: byte(c(38)),
    })
}

pub fn runway(payload: &str) -> Option<RunwayRecord> {
    let mut groups = payload.split(';');
    let f: Vec<&str> = groups.next()?.split(',').collect();
    let g: Vec<&str> = groups.next().unwrap_or("").split(',').collect();
    let c = |i: usize| f.get(i - 1).copied().unwrap_or("");
    let threshold = match (g.first().and_then(|s| coordinate(s)), g.get(1).and_then(|s| coordinate(s))) {
        (Some(lat), Some(lon)) => Some((lat, lon)),
        _ => None,
    };
    Some(RunwayRecord {
        id: Code::new(c(1))?,
        threshold_elev_ft: opt_num(c(4)),
        ls_ident: Code::lossy(c(6)),
        ls_category: byte(c(7)),
        threshold,
        displaced_ft: g.get(2).and_then(|s| opt_num(s)),
    })
}

/// Levels of service from a PRDAT row: authorised flag and name pairs in
/// columns 1-6.
pub fn service(payload: &str) -> Vec<(bool, String)> {
    let f: Vec<&str> = payload.split(';').next().unwrap_or("").split(',').collect();
    (0..3)
        .filter_map(|i| {
            let name = f.get(i * 2 + 1)?.trim();
            let authorised = f.get(i * 2)?.trim() == "A";
            (!name.is_empty()).then(|| (authorised, name.to_string()))
        })
        .collect()
}

/// Parses a whole CIFP file.
pub fn parse(text: &str) -> Cifp {
    let mut out = Cifp::default();
    for line in text.lines() {
        let Some((row, payload)) = line.split_once(':') else { continue };
        let kind = match row {
            "SID" => Kind::Sid,
            "STAR" => Kind::Star,
            "APPCH" => Kind::Approach,
            "RWY" => {
                if let Some(r) = runway(payload) {
                    out.runways.push(r);
                }
                continue;
            }
            "PRDAT" => {
                if let Some(p) = out.procedures.last_mut() {
                    for s in service(payload) {
                        if !p.service.contains(&s) {
                            p.service.push(s);
                        }
                    }
                }
                continue;
            }
            // Helicopter procedures (HSID, HSTAR, HAPPCH, HPRDAT) have no
            // MSFS counterpart.
            _ => continue,
        };
        let Some(leg) = leg(payload) else { continue };
        let ident = payload.split(',').nth(2).unwrap_or("").trim();
        match out.procedures.last_mut() {
            Some(p) if p.kind == kind && p.ident == ident => p.legs.push(leg),
            _ => {
                // Records are grouped by procedure, but look back in case a
                // file interleaves them.
                if let Some(p) = out.procedures.iter_mut().find(|p| p.kind == kind && p.ident == ident) {
                    p.legs.push(leg);
                } else {
                    out.procedures.push(Procedure { kind, ident: ident.to_string(), legs: vec![leg], service: Vec::new() });
                }
            }
        }
    }
    out
}

/// MSFS `LegType` for an ARINC path terminator.
pub fn leg_type(path: [u8; 2]) -> u8 {
    match &path {
        b"AF" => 1,
        b"CA" => 2,
        b"CD" => 3,
        b"CF" => 4,
        b"CI" => 5,
        b"CR" => 6,
        b"DF" => 7,
        b"FA" => 8,
        b"FC" => 9,
        b"FD" => 10,
        b"FM" => 11,
        b"HA" => 12,
        b"HF" => 13,
        b"HM" => 14,
        b"IF" => 15,
        b"PI" => 16,
        b"RF" => 17,
        b"TF" => 18,
        b"VA" => 19,
        b"VD" => 20,
        b"VI" => 21,
        b"VM" => 22,
        b"VR" => 23,
        _ => 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn procedure_rows() {
        // X-Plane 12 CIFP/EGLL.dat.
        let l = leg("020,I,I27L, ,FF27L,EG,P,C,E  F, ,   ,CF, ,ILL,EG,P,I,      ,0897,0096,2690,0025,G,02500,02500,     , ,   ,-300,   ,LON,EG,D, , ,0,D,S;").unwrap();
        assert_eq!((l.seq, l.route_type, l.fix.ident.as_str(), &l.fix.region, l.fix.section, l.fix.subsection), (20, b'I', "FF27L", b"EG", b'P', b'C'));
        assert_eq!((&l.desc, &l.path, l.navaid.ident.as_str(), l.navaid.subsection), (b"E  F", b"CF", "ILL", b'I'));
        assert_eq!((l.theta, l.rho_nm, l.course, l.course_true, l.distance, l.minutes), (Some(89.7), Some(9.6), Some(269.), false, Some(2.5), false));
        assert_eq!((l.alt_desc, l.alt1_ft, l.alt2_ft, l.vertical_angle), (b'G', Some(2500), Some(2500), Some(-3.)));
        assert_eq!((l.qualifier1, l.qualifier2), (b'D', b'S'));

        let s = leg("010,2,BPK5K,RW09L,D110B,EG,P,C,EY  , ,   ,CF, ,LON,EG,D, ,      ,1104,0015,0890,0020,+,00590,     ,06000, ,   ,    ,   , , , , , , , , ;").unwrap();
        assert_eq!((s.transition.as_str(), s.transition_alt_ft, s.alt_desc, s.alt1_ft, &s.desc), ("RW09L", Some(6000), b'+', Some(590), b"EY  "));

        let h = leg("010,A,I27L,BNN,BNN,EG,D, ,V  C, ,   ,FD, ,BNN,EG,D, ,      ,0000,0000,1260,0110, ,07000,     ,     ,-,220,    ,   , , , , , ,0,D,S;").unwrap();
        assert_eq!((h.speed_desc, h.speed_kt, h.alt1_ft), (b'-', Some(220), Some(7000)));

        // CIFP/KJFK.dat: an RF leg with its centre fix and RNP.
        let rf = leg("027,R,R13L, ,JEVNI,K6,P,C,E   ,R,302,RF, , , , , ,002100,    ,    ,    ,0033,+,00313,     ,     , ,   ,-300,   ,CFBMG,K6,P,C, ,A,P,S;").unwrap();
        assert_eq!((rf.turn, rf.arc_radius_nm, rf.center.ident.as_str(), rf.qualifier1), (b'R', Some(2.1), "CFBMG", b'P'));
        assert!((rf.rnp_nm.unwrap() - 0.3).abs() < 1e-12);
        assert_eq!(leg_type(rf.path), 17);
    }

    #[test]
    fn runway_and_procedure_data_rows() {
        let r = runway("RW27L,     ,      ,00077, ,ILL ,3,   ;N51275382,W000260276,0000;").unwrap();
        assert_eq!((r.id.as_str(), r.threshold_elev_ft, r.ls_ident.as_str(), r.ls_category, r.displaced_ft), ("RW27L", Some(77), "ILL", b'3', Some(0)));
        let (lat, lon) = r.threshold.unwrap();
        assert!((lat - (51. + 27. / 60. + 53.82 / 3600.)).abs() < 1e-9);
        assert!((lon + (26. / 60. + 2.76 / 3600.)).abs() < 1e-9);
        assert_eq!(service("A,       LPV,A, LNAV/VNAV,A,      LNAV, ,   , ,   , ,   , ,   ,J,S;"), vec![(true, "LPV".into()), (true, "LNAV/VNAV".into()), (true, "LNAV".into())]);
        assert_eq!(service(" ,          , ,          ,A,      LNAV, ,   , ,   , ,   , ,   ,J,S;"), vec![(true, "LNAV".into())]);

        let c = parse(
            "APPCH:010,R,R04LY, ,REPRE,K6,P,C,E  I, ,   ,IF, , , , , ,      ,    ,    ,    ,    ,+,02000,     ,18000, ,   ,    ,   , , , , , ,A,J,S;\n\
             APPCH:020,R,R04LY, ,KRSTL,K6,P,C,E  F, ,010,TF, , , , , ,      ,    ,    ,0440,0065,+,01500,     ,     , ,   ,    ,   ,RW04L,K6,P,G, ,A,J,S;\n\
             PRDAT:A,       LPV,A, LNAV/VNAV,A,      LNAV, ,   , ,   , ,   , ,   ,J,S;\n\
             RWY:RW04L,     ,      ,00012, ,IHIQ,1,   ;N40372318,W073470505,0460;\n",
        );
        assert_eq!(c.procedures.len(), 1);
        assert_eq!((c.procedures[0].ident.as_str(), c.procedures[0].legs.len(), c.procedures[0].service.len()), ("R04LY", 2, 3));
        assert_eq!(c.runways[0].displaced_ft, Some(460));
    }
}
