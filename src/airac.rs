//! AIRAC cycle and validity dates from X-Plane's navigation data header, as
//! MSFS's `FLIGHT NAVDATA DATE RANGE` gives them.

use std::path::Path;

/// Days since 1970-01-01 for a civil date.
fn days_from_civil(y: i64, m: i64, d: i64) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let doy = (153 * (m + if m > 2 { -3 } else { 9 }) + 2) / 5 + d - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}

fn civil_from_days(z: i64) -> (i64, i64, i64) {
    let z = z + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    (yoe + era * 400 + if m <= 2 { 1 } else { 0 }, m, d)
}

/// The AIRAC cycle named on a file's version line ("... data cycle 2406, ...").
pub fn cycle(header: &str) -> Option<u32> {
    let at = header.find("data cycle")?;
    header[at + 10..].trim_start().get(..4)?.parse().ok()
}

/// The AIRAC cycle from a nav data file's header, reading only the first few
/// lines rather than the whole file.
pub fn header_cycle(path: &Path) -> Option<u32> {
    use std::io::BufRead;
    let file = std::fs::File::open(path).ok()?;
    let mut reader = std::io::BufReader::new(file);
    let mut line = String::new();
    for _ in 0..3 {
        line.clear();
        if reader.read_line(&mut line).ok()? == 0 {
            return None;
        }
        if let Some(cycle) = cycle(&line) {
            return Some(cycle);
        }
    }
    None
}

/// "MMMddMMMdd/yy" for an AIRAC cycle "YYNN": cycles are 28 days, and
/// 2001 took effect on 2 January 2020.
pub fn date_range(cycle: u32) -> Option<String> {
    let (yy, nn) = ((cycle / 100) as i64, (cycle % 100) as i64);
    if nn == 0 || nn > 14 {
        return None;
    }
    let epoch = days_from_civil(2020, 1, 2);
    let year = 2000 + yy;
    let jan1 = days_from_civil(year, 1, 1);
    let first = epoch + (jan1 - epoch).div_euclid(28) * 28 + if (jan1 - epoch).rem_euclid(28) == 0 { 0 } else { 28 };
    let effective = first + (nn - 1) * 28;
    let expires = effective + 27;
    const MONTHS: [&str; 12] = ["JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"];
    let (_, m1, d1) = civil_from_days(effective);
    let (y2, m2, d2) = civil_from_days(expires);
    Some(format!("{}{:02}{}{:02}/{:02}", MONTHS[(m1 - 1) as usize], d1, MONTHS[(m2 - 1) as usize], d2, y2 % 100))
}

/// The date range of the earth_nav.dat X-Plane itself uses: `Custom Data`'s
/// when there is one, else `Resources/default data`'s.
pub fn date_range_for_xplane(xplane_root: &Path) -> Option<String> {
    let custom = xplane_root.join("Custom Data").join("earth_nav.dat");
    let nav = if custom.is_file() { custom } else { xplane_root.join("Resources").join("default data").join("earth_nav.dat") };
    date_range(header_cycle(&nav)?)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_date_range_comes_straight_from_xplanes_own_navdata_header() {
        let root = std::env::temp_dir().join(format!("fbw-airac-range-test-{}", std::process::id()));
        let default_data = root.join("Resources").join("default data");
        std::fs::create_dir_all(&default_data).unwrap();
        std::fs::write(default_data.join("earth_nav.dat"), "I\n1200 Version - data cycle 2406, build 20251002, metadata NavXP1200.\n\n").unwrap();
        assert_eq!(date_range_for_xplane(&root).as_deref(), Some("JUN13JUL10/24"));
        let custom = root.join("Custom Data");
        std::fs::create_dir_all(&custom).unwrap();
        std::fs::write(custom.join("earth_nav.dat"), "I\n1200 Version - data cycle 2510, build 20251002.\n").unwrap();
        assert_eq!(date_range_for_xplane(&root), date_range(2510), "Custom Data replaces the default data");
        std::fs::remove_dir_all(&root).ok();
    }
}
