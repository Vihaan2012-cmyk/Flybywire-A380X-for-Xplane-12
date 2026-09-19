//! Airports from apt.dat, following Laminar's apt.dat 1200 specification.
//!
//! The global apt.dat is close to 400 MB, almost all of it taxiway and
//! ground network detail. The scan keeps only what facility records need:
//! the header (1, 16, 17), runways (100), sealanes (101), helipads (102),
//! metadata (1302) and frequencies (50-56, 1050-1056), with each airport's
//! byte offset so its gates (1300) can be read later on demand.

use std::fs::File;
use std::io::{BufRead, BufReader, Seek, SeekFrom};
use std::path::Path;

use super::icao::{region, Code};

#[derive(Clone, Debug, Default)]
pub struct RunwayEnd {
    /// "09L", "27", "08W".
    pub id: Code,
    pub lat: f64,
    pub lon: f64,
    pub displaced_m: f32,
    pub overrun_m: f32,
}

#[derive(Clone, Debug, Default)]
pub struct Runway {
    pub width_m: f32,
    /// The apt.dat surface code; 13 for sealanes.
    pub surface: u16,
    pub edge_lights: u8,
    pub water: bool,
    pub ends: [RunwayEnd; 2],
}

#[derive(Clone, Debug)]
pub struct Frequency {
    /// 0 recorded (ATIS/AWOS/ASOS), 1 unicom/CTAF, 2 clearance, 3 ground,
    /// 4 tower, 5 approach, 6 departure.
    pub kind: u8,
    pub khz: u32,
    pub name: String,
    /// From the old 50-56 rows, which 1050-1056 supersede.
    pub legacy: bool,
}

#[derive(Clone, Debug, Default)]
pub struct Airport {
    /// The header's airport identifier.
    pub header_id: Code,
    /// The ICAO code from metadata, or the header identifier.
    pub ident: Code,
    pub name: String,
    pub city: String,
    pub iata: Code,
    pub region: [u8; 2],
    /// Reference point, if the metadata gives one.
    pub datum: Option<(f64, f64)>,
    pub lat: f64,
    pub lon: f64,
    pub elev_ft: f32,
    /// 1 land, 16 seaplane base, 17 heliport.
    pub kind: u8,
    pub closed: bool,
    pub runways: Vec<Runway>,
    pub helipads: u16,
    pub freqs: Vec<Frequency>,
    pub ta_ft: Option<i32>,
    pub tl_ft: Option<i32>,
    /// Which scanned file, and where in it the header row starts.
    pub file: u16,
    pub offset: u64,
}

impl Airport {
    pub fn towered(&self) -> bool {
        self.freqs.iter().any(|f| f.kind == 4)
    }

    /// Lat/lon from the runways and helipads when there is no datum.
    fn settle_position(&mut self, pads: &[(f64, f64)]) {
        // Rows 1050-1056 supersede 50-56.
        if self.freqs.iter().any(|f| !f.legacy) {
            self.freqs.retain(|f| !f.legacy);
        }
        if let Some((lat, lon)) = self.datum {
            self.lat = lat;
            self.lon = lon;
            return;
        }
        let points: Vec<(f64, f64)> = self
            .runways
            .iter()
            .flat_map(|r| r.ends.iter().map(|e| (e.lat, e.lon)))
            .chain(pads.iter().copied())
            .collect();
        if points.is_empty() {
            return;
        }
        let (mut lat0, mut lat1, mut lon0, mut lon1) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
        for (lat, lon) in points {
            lat0 = lat0.min(lat);
            lat1 = lat1.max(lat);
            lon0 = lon0.min(lon);
            lon1 = lon1.max(lon);
        }
        self.lat = (lat0 + lat1) / 2.;
        self.lon = (lon0 + lon1) / 2.;
    }
}

fn num<T: std::str::FromStr>(s: Option<&str>) -> Option<T> {
    s?.parse().ok()
}

/// Scans one apt.dat, calling `found` for every airport.
pub fn scan(path: &Path, file: u16, mut found: impl FnMut(Airport)) -> std::io::Result<()> {
    let mut reader = BufReader::with_capacity(1 << 20, File::open(path)?);
    let mut line = Vec::with_capacity(256);
    let mut offset = 0u64;
    let mut current: Option<Airport> = None;
    let mut pads: Vec<(f64, f64)> = Vec::new();
    let mut finish = |a: Option<Airport>, pads: &mut Vec<(f64, f64)>| {
        if let Some(mut a) = a {
            a.settle_position(pads);
            found(a);
        }
        pads.clear();
    };
    loop {
        line.clear();
        let n = reader.read_until(b'\n', &mut line)?;
        if n == 0 {
            break;
        }
        let start = offset;
        offset += n as u64;
        // Cheap row code test before any decoding: nearly every row is
        // ground network or pavement.
        let code_end = line.iter().position(|b| !b.is_ascii_digit()).unwrap_or(line.len());
        let code = &line[..code_end];
        let wanted = matches!(code, b"1" | b"16" | b"17" | b"99" | b"100" | b"101" | b"102" | b"1302")
            || (code.len() == 2 && code[0] == b'5')
            || (code.len() == 4 && code.starts_with(b"105"));
        if !wanted || code_end == line.len() || !line[code_end].is_ascii_whitespace() {
            continue;
        }
        let text = String::from_utf8_lossy(&line);
        let mut f = text.split_ascii_whitespace();
        let row = f.next().unwrap_or("");
        match row {
            "1" | "16" | "17" => {
                finish(current.take(), &mut pads);
                let elev: f32 = num(f.next()).unwrap_or(0.);
                let _ = (f.next(), f.next());
                let Some(id) = f.next().and_then(Code::new) else { continue };
                let name = f.collect::<Vec<_>>().join(" ");
                current = Some(Airport {
                    header_id: id,
                    ident: id,
                    closed: name.starts_with("[X]"),
                    name,
                    elev_ft: elev,
                    kind: row.parse().unwrap_or(1),
                    file,
                    offset: start,
                    ..Default::default()
                });
            }
            "99" => finish(current.take(), &mut pads),
            _ => {
                let Some(a) = current.as_mut() else { continue };
                parse_row(a, row, &mut f, &mut pads);
            }
        }
    }
    finish(current.take(), &mut pads);
    Ok(())
}

fn parse_row<'a>(a: &mut Airport, row: &str, f: &mut impl Iterator<Item = &'a str>, pads: &mut Vec<(f64, f64)>) {
    match row {
        "100" => {
            let width = num(f.next()).unwrap_or(0.);
            let surface = num(f.next()).unwrap_or(0);
            let _shoulder = f.next();
            let _smoothness = f.next();
            let _centre = f.next();
            let edge = num(f.next()).unwrap_or(0);
            let _signs = f.next();
            let mut end = || -> Option<RunwayEnd> {
                let id = Code::lossy(f.next()?);
                let lat = num(f.next())?;
                let lon = num(f.next())?;
                let displaced_m = num(f.next()).unwrap_or(0.);
                let overrun_m = num(f.next()).unwrap_or(0.);
                for _ in 0..4 {
                    f.next();
                }
                Some(RunwayEnd { id, lat, lon, displaced_m, overrun_m })
            };
            if let (Some(e1), Some(e2)) = (end(), end()) {
                a.runways.push(Runway { width_m: width, surface, edge_lights: edge, water: false, ends: [e1, e2] });
            }
        }
        "101" => {
            let width = num(f.next()).unwrap_or(0.);
            let _buoys = f.next();
            let mut end = || -> Option<RunwayEnd> {
                Some(RunwayEnd { id: Code::lossy(f.next()?), lat: num(f.next())?, lon: num(f.next())?, ..Default::default() })
            };
            if let (Some(e1), Some(e2)) = (end(), end()) {
                a.runways.push(Runway { width_m: width, surface: 13, edge_lights: 0, water: true, ends: [e1, e2] });
            }
        }
        "102" => {
            let _id = f.next();
            if let (Some(lat), Some(lon)) = (num(f.next()), num(f.next())) {
                pads.push((lat, lon));
                a.helipads += 1;
            }
        }
        "1302" => {
            let key = f.next().unwrap_or("");
            let value = f.collect::<Vec<_>>().join(" ");
            match key {
                "icao_code" => {
                    if let Some(c) = Code::new(&value) {
                        a.ident = c;
                    }
                }
                "iata_code" => a.iata = Code::lossy(&value),
                "region_code" => a.region = region(&value),
                "city" => a.city = value,
                "datum_lat" => {
                    if let Ok(lat) = value.parse() {
                        a.datum = Some((lat, a.datum.map_or(0., |d| d.1)));
                    }
                }
                "datum_lon" => {
                    if let Ok(lon) = value.parse() {
                        a.datum = Some((a.datum.map_or(0., |d| d.0), lon));
                    }
                }
                "transition_alt" => a.ta_ft = value.parse().ok().filter(|v| *v > 0),
                "transition_level" => {
                    a.tl_ft = match value.strip_prefix("FL") {
                        Some(fl) => fl.parse::<i32>().ok().map(|v| v * 100),
                        None => value.parse().ok(),
                    }
                    .filter(|v| *v > 0)
                }
                _ => {}
            }
        }
        _ => {
            // 50-56 (frequency in tens of kHz) and 1050-1056 (kHz).
            let Ok(code) = row.parse::<u32>() else { return };
            let (kind, scale) = match code {
                50..=56 => (code - 50, 10),
                1050..=1056 => (code - 1050, 1),
                _ => return,
            };
            let Some(freq) = num::<u32>(f.next()) else { return };
            let name = f.collect::<Vec<_>>().join(" ");
            a.freqs.push(Frequency { kind: kind as u8, khz: freq * scale, name, legacy: scale == 10 });
        }
    }
}

/// A gate or stand (row 1300): position and name.
#[derive(Clone, Debug)]
pub struct Gate {
    pub lat: f64,
    pub lon: f64,
    pub kind: String,
    pub name: String,
}

/// Reads the startup locations of the airport whose header starts at `offset`.
pub fn gates(path: &Path, offset: u64) -> std::io::Result<Vec<Gate>> {
    let mut file = File::open(path)?;
    file.seek(SeekFrom::Start(offset))?;
    let mut reader = BufReader::with_capacity(1 << 16, file);
    let mut line = String::new();
    let mut out = Vec::new();
    let mut first = true;
    loop {
        line.clear();
        let mut bytes = Vec::new();
        if reader.read_until(b'\n', &mut bytes)? == 0 {
            break;
        }
        line.push_str(&String::from_utf8_lossy(&bytes));
        let mut f = line.split_ascii_whitespace();
        match f.next() {
            Some("1" | "16" | "17" | "99") if !first => break,
            Some("1300") => {
                let (Some(lat), Some(lon)) = (num(f.next()), num(f.next())) else { continue };
                let _heading = f.next();
                let kind = f.next().unwrap_or("").to_string();
                let _types = f.next();
                out.push(Gate { lat, lon, kind, name: f.collect::<Vec<_>>().join(" ") });
            }
            _ => {}
        }
        first = false;
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scans_airport_rows() {
        // Rows as they appear in X-Plane 12's Global Airports apt.dat.
        let dir = std::env::temp_dir().join(format!("navdata-apt-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("apt.dat");
        std::fs::write(
            &path,
            "A\n1200 Generated by WorldEditor\n\n\
             1     83 0 0 EGLL London Heathrow\n\
             1302 city London\n1302 datum_lat 51.4775\n1302 datum_lon -0.461388889\n\
             1302 iata_code LHR\n1302 icao_code EGLL\n1302 region_code EG\n\
             1302 transition_alt 6000\n1302 transition_level 7000\n\
             100 49.99 25 2030 0.00 1 3 0 09L 51.4774961 -0.4894104 306 0 3 4 1 1 27R 51.4776934 -0.4332280 0 55 3 4 1 1\n\
             111  51.46 -0.45\n\
             1300 51.4700 -0.4500 90.0 gate jets|heavy 505\n\
             1054 118500 HEATHROW TWR\n\
             54 11850 OLD TWR\n\
             1 28 0 0 OMDB Dubai Intl\n\
             102 H1 25.25 55.36 0 10 10 1 0 0 0.25 0\n\
             99\n",
        )
        .unwrap();
        let mut found = Vec::new();
        scan(&path, 0, |a| found.push(a)).unwrap();
        assert_eq!(found.len(), 2);
        let egll = &found[0];
        assert_eq!((egll.ident.as_str(), egll.iata.as_str(), &egll.region, egll.city.as_str()), ("EGLL", "LHR", b"EG", "London"));
        assert_eq!((egll.lat, egll.lon, egll.elev_ft), (51.4775, -0.461388889, 83.));
        assert_eq!((egll.ta_ft, egll.tl_ft), (Some(6000), Some(7000)));
        assert_eq!(egll.runways.len(), 1);
        let r = &egll.runways[0];
        assert_eq!((r.surface, r.edge_lights, r.ends[0].id.as_str(), r.ends[0].displaced_m, r.ends[1].overrun_m), (25, 3, "09L", 306., 55.));
        assert!(egll.towered());
        assert_eq!(egll.freqs.len(), 1, "1054 supersedes 54: {:?}", egll.freqs);
        let omdb = &found[1];
        assert_eq!((omdb.helipads, omdb.lat, omdb.lon), (1, 25.25, 55.36));
        let g = gates(&path, egll.offset).unwrap();
        assert_eq!(g.len(), 1);
        assert_eq!((g[0].kind.as_str(), g[0].name.as_str()), ("gate", "505"));
    }
}
