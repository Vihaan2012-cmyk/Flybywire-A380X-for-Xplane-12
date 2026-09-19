//! A real World Magnetic Model: NOAA's WMM2025 (epoch 2025.0, spherical
//! harmonic degree/order 12) — `window.__xphfbw.magVar` (agent E,
//! docs/briefs/xphfbw-js-bridge.md rule list item 9 / the renderer contract).
//!
//! Ported from NOAA's public-domain reference implementation
//! (`GeomagnetismLibrary.c`: `MAG_GeodeticToSpherical`,
//! `MAG_AssociatedLegendreFunction`/`MAG_PcupLow`,
//! `MAG_ComputeSphericalHarmonicVariables`, `MAG_Summation`,
//! `MAG_RotateMagneticVector`, `MAG_CalculateGeoMagneticElements` — WMM
//! Technical Report equations 7-19). `magVar(lat, lon)` takes no date or
//! altitude, so this evaluates at the model's own epoch (2025.0) and sea
//! level (WGS84 height 0), which is exactly the case NOAA's own published
//! WMM2025 test values cover (this module's unit tests check against them).
//! Secular variation (the `gdot`/`hdot` columns of WMM.COF) is therefore
//! not needed and not used.

const N_MAX: usize = 12;
/// `(N_MAX + 1) * (N_MAX + 2) / 2` — one slot per `(n, m)` pair, `n` in
/// `0..=N_MAX`, `m` in `0..=n`.
const TERMS: usize = 91;

/// WGS84 ellipsoid semi-major/semi-minor axes (km).
const A: f64 = 6378.137;
const B: f64 = 6356.7523142;
/// The WMM's own geomagnetic reference radius (km) — not the WGS84 mean
/// radius, this is the constant NOAA's spherical harmonic series is defined
/// against (WMM Technical Report eq. 10-12).
const RE: f64 = 6371.2;

/// NOAA WMM2025 (epoch 2025.0) Gauss coefficients `g(n, m)`, `h(n, m)` in
/// nT, transcribed from the published `WMM.COF`
/// (https://www.ncei.noaa.gov/products/world-magnetic-model, WMM2025COF.zip,
/// "2025.0 WMM-2025 11/13/2024"). The secular-variation columns of that file
/// are omitted; see the module doc comment for why.
const COEFFS: &[(usize, usize, f64, f64)] = &[
    (1, 0, -29351.8, 0.0),
    (1, 1, -1410.8, 4545.4),
    (2, 0, -2556.6, 0.0),
    (2, 1, 2951.1, -3133.6),
    (2, 2, 1649.3, -815.1),
    (3, 0, 1361.0, 0.0),
    (3, 1, -2404.1, -56.6),
    (3, 2, 1243.8, 237.5),
    (3, 3, 453.6, -549.5),
    (4, 0, 895.0, 0.0),
    (4, 1, 799.5, 278.6),
    (4, 2, 55.7, -133.9),
    (4, 3, -281.1, 212.0),
    (4, 4, 12.1, -375.6),
    (5, 0, -233.2, 0.0),
    (5, 1, 368.9, 45.4),
    (5, 2, 187.2, 220.2),
    (5, 3, -138.7, -122.9),
    (5, 4, -142.0, 43.0),
    (5, 5, 20.9, 106.1),
    (6, 0, 64.4, 0.0),
    (6, 1, 63.8, -18.4),
    (6, 2, 76.9, 16.8),
    (6, 3, -115.7, 48.8),
    (6, 4, -40.9, -59.8),
    (6, 5, 14.9, 10.9),
    (6, 6, -60.7, 72.7),
    (7, 0, 79.5, 0.0),
    (7, 1, -77.0, -48.9),
    (7, 2, -8.8, -14.4),
    (7, 3, 59.3, -1.0),
    (7, 4, 15.8, 23.4),
    (7, 5, 2.5, -7.4),
    (7, 6, -11.1, -25.1),
    (7, 7, 14.2, -2.3),
    (8, 0, 23.2, 0.0),
    (8, 1, 10.8, 7.1),
    (8, 2, -17.5, -12.6),
    (8, 3, 2.0, 11.4),
    (8, 4, -21.7, -9.7),
    (8, 5, 16.9, 12.7),
    (8, 6, 15.0, 0.7),
    (8, 7, -16.8, -5.2),
    (8, 8, 0.9, 3.9),
    (9, 0, 4.6, 0.0),
    (9, 1, 7.8, -24.8),
    (9, 2, 3.0, 12.2),
    (9, 3, -0.2, 8.3),
    (9, 4, -2.5, -3.3),
    (9, 5, -13.1, -5.2),
    (9, 6, 2.4, 7.2),
    (9, 7, 8.6, -0.6),
    (9, 8, -8.7, 0.8),
    (9, 9, -12.9, 10.0),
    (10, 0, -1.3, 0.0),
    (10, 1, -6.4, 3.3),
    (10, 2, 0.2, 0.0),
    (10, 3, 2.0, 2.4),
    (10, 4, -1.0, 5.3),
    (10, 5, -0.6, -9.1),
    (10, 6, -0.9, 0.4),
    (10, 7, 1.5, -4.2),
    (10, 8, 0.9, -3.8),
    (10, 9, -2.7, 0.9),
    (10, 10, -3.9, -9.1),
    (11, 0, 2.9, 0.0),
    (11, 1, -1.5, 0.0),
    (11, 2, -2.5, 2.9),
    (11, 3, 2.4, -0.6),
    (11, 4, -0.6, 0.2),
    (11, 5, -0.1, 0.5),
    (11, 6, -0.6, -0.3),
    (11, 7, -0.1, -1.2),
    (11, 8, 1.1, -1.7),
    (11, 9, -1.0, -2.9),
    (11, 10, -0.2, -1.8),
    (11, 11, 2.6, -2.3),
    (12, 0, -2.0, 0.0),
    (12, 1, -0.2, -1.3),
    (12, 2, 0.3, 0.7),
    (12, 3, 1.2, 1.0),
    (12, 4, -1.3, -1.4),
    (12, 5, 0.6, -0.0),
    (12, 6, 0.6, 0.6),
    (12, 7, 0.5, -0.1),
    (12, 8, -0.1, 0.8),
    (12, 9, -0.4, 0.1),
    (12, 10, -0.2, -1.0),
    (12, 11, -1.3, 0.1),
    (12, 12, -0.7, 0.2),
];

/// The Gauss/Schmidt term index NOAA's code uses: one triangular table
/// indexed by `(n, m)`.
const fn index(n: usize, m: usize) -> usize {
    n * (n + 1) / 2 + m
}

/// `(g, h)` coefficient arrays indexed like `index(n, m)`.
fn coeff_arrays() -> ([f64; TERMS], [f64; TERMS]) {
    let mut g = [0.0; TERMS];
    let mut h = [0.0; TERMS];
    for &(n, m, gnm, hnm) in COEFFS {
        let i = index(n, m);
        g[i] = gnm;
        h[i] = hnm;
    }
    (g, h)
}

/// WGS84 geodetic latitude/height (km) to the geocentric spherical
/// coordinates the model is evaluated in: distance from Earth's centre `r`
/// (km) and geocentric latitude `phig` (degrees). Longitude is unchanged
/// (`MAG_GeodeticToSpherical`).
fn geodetic_to_spherical(lat_deg: f64, height_km: f64) -> (f64, f64) {
    let epssq = 1.0 - (B * B) / (A * A);
    let (sin_lat, cos_lat) = lat_deg.to_radians().sin_cos();
    let rc = A / (1.0 - epssq * sin_lat * sin_lat).sqrt();
    let xp = (rc + height_km) * cos_lat;
    let zp = (rc * (1.0 - epssq) + height_km) * sin_lat;
    let r = (xp * xp + zp * zp).sqrt();
    let phig = (zp / r).asin().to_degrees();
    (r, phig)
}

/// Schmidt semi-normalized associated Legendre functions and their
/// derivative with respect to latitude, evaluated at `sin_phi = sin(geocentric
/// latitude)`, up to degree `N_MAX` (`MAG_PcupLow` — the low/medium-degree
/// path NOAA's own code uses for `nMax <= 16`).
fn legendre(sin_phi: f64) -> ([f64; TERMS], [f64; TERMS]) {
    let mut p = [0.0; TERMS];
    let mut dp = [0.0; TERMS];
    p[0] = 1.0;
    dp[0] = 0.0;
    let x = sin_phi;
    let z = ((1.0 - x) * (1.0 + x)).max(0.0).sqrt();

    for n in 1..=N_MAX {
        for m in 0..=n {
            let idx = index(n, m);
            if n == m {
                let idx1 = index(n - 1, m - 1);
                p[idx] = z * p[idx1];
                dp[idx] = z * dp[idx1] + x * p[idx1];
            } else if n == 1 && m == 0 {
                let idx1 = index(0, 0);
                p[idx] = x * p[idx1];
                dp[idx] = x * dp[idx1] - z * p[idx1];
            } else {
                // n > 1 && n != m here: n == 1's only other case (m == 0) is
                // handled above, and n == m is handled above too.
                let idx2 = index(n - 1, m);
                if m > n - 2 {
                    p[idx] = x * p[idx2];
                    dp[idx] = x * dp[idx2] - z * p[idx2];
                } else {
                    let idx1 = index(n - 2, m);
                    let k = ((n - 1) * (n - 1)) as f64 - (m * m) as f64;
                    let k = k / ((2 * n - 1) * (2 * n - 3)) as f64;
                    p[idx] = x * p[idx2] - k * p[idx1];
                    dp[idx] = x * dp[idx2] - z * p[idx2] - k * dp[idx1];
                }
            }
        }
    }

    // Gauss-normalized -> Schmidt quasi-normalized associated Legendre
    // functions.
    let mut schmidt = [0.0; TERMS];
    schmidt[0] = 1.0;
    for n in 1..=N_MAX {
        let idx = index(n, 0);
        let idx1 = index(n - 1, 0);
        schmidt[idx] = schmidt[idx1] * (2 * n - 1) as f64 / n as f64;
        for m in 1..=n {
            let idx = index(n, m);
            let idxm1 = index(n, m - 1);
            let factor = if m == 1 { 2.0 } else { 1.0 };
            schmidt[idx] = schmidt[idxm1] * (((n - m + 1) as f64 * factor) / (n + m) as f64).sqrt();
        }
    }
    for n in 1..=N_MAX {
        for m in 0..=n {
            let idx = index(n, m);
            p[idx] *= schmidt[idx];
            // The sign is flipped: NOAA's routine differentiates with
            // respect to latitude, not colatitude.
            dp[idx] = -dp[idx] * schmidt[idx];
        }
    }
    (p, dp)
}

/// Magnetic declination (degrees, positive east of true north) at a WGS84
/// geodetic position, sea level, WMM2025 epoch — `window.__xphfbw.magVar`.
pub fn declination(lat_deg: f64, lon_deg: f64) -> f64 {
    let (g, h) = coeff_arrays();
    let (r, phig) = geodetic_to_spherical(lat_deg, 0.0);
    let sin_phi = phig.to_radians().sin();
    let (p, dp) = legendre(sin_phi);

    let (sin_lambda, cos_lambda) = lon_deg.to_radians().sin_cos();
    let mut cosm = [1.0; N_MAX + 1];
    let mut sinm = [0.0; N_MAX + 1];
    cosm[1] = cos_lambda;
    sinm[1] = sin_lambda;
    for m in 2..=N_MAX {
        cosm[m] = cosm[m - 1] * cos_lambda - sinm[m - 1] * sin_lambda;
        sinm[m] = cosm[m - 1] * sin_lambda + sinm[m - 1] * cos_lambda;
    }

    let mut relpow = [0.0; N_MAX + 1];
    relpow[0] = (RE / r) * (RE / r);
    for n in 1..=N_MAX {
        relpow[n] = relpow[n - 1] * (RE / r);
    }

    // Bx (north), By (east), Bz (down), in the geocentric spherical frame
    // (WMM Technical Report eq. 10-12).
    let (mut bx, mut by, mut bz) = (0.0f64, 0.0f64, 0.0f64);
    for n in 1..=N_MAX {
        for m in 0..=n {
            let idx = index(n, m);
            let (gnm, hnm) = (g[idx], h[idx]);
            bz -= relpow[n] * (gnm * cosm[m] + hnm * sinm[m]) * (n + 1) as f64 * p[idx];
            by += relpow[n] * (gnm * sinm[m] - hnm * cosm[m]) * m as f64 * p[idx];
            bx -= relpow[n] * (gnm * cosm[m] + hnm * sinm[m]) * dp[idx];
        }
    }
    let cos_phi = phig.to_radians().cos();
    if cos_phi.abs() > 1.0e-10 {
        by /= cos_phi;
    }

    // Rotate spherical (Bx, Bz) to the geodetic frame (MAG_RotateMagneticVector,
    // WMM Technical Report eq. 16); By is unchanged.
    let psi = (phig - lat_deg).to_radians();
    let bx_geo = bx * psi.cos() - bz * psi.sin();
    let by_geo = by;

    by_geo.atan2(bx_geo).to_degrees()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// NOAA's own published WMM2025 test values (WMM2025COF.zip /
    /// WMM2025_TEST_VALUES.txt), date 2025.0, height above the WGS84
    /// ellipsoid 0.0 km — exactly the case this module evaluates
    /// (`magVar` has no date or altitude). Field 11 is Declination.
    #[test]
    fn matches_published_wmm2025_test_values_at_sea_level() {
        let cases = [
            // (lat, lon, expected declination, deg)
            (80.0, 0.0, 1.28),
            (0.0, 120.0, -0.16),
            (-80.0, 240.0, 68.78),
        ];
        for (lat, lon, expected) in cases {
            let got = declination(lat, lon);
            assert!(
                (got - expected).abs() < 0.02,
                "declination({lat}, {lon}) = {got}, published WMM2025 test value is {expected}"
            );
        }
    }

    #[test]
    fn declination_is_continuous_across_the_antimeridian() {
        let a = declination(51.5, 179.9);
        let b = declination(51.5, -179.9);
        assert!((a - b).abs() < 0.5, "a={a} b={b}");
    }

    #[test]
    fn declination_is_finite_at_the_poles() {
        assert!(declination(90.0, 0.0).is_finite());
        assert!(declination(-90.0, 0.0).is_finite());
    }
}
