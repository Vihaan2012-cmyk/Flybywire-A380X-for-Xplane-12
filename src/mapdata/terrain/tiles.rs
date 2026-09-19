//! Terrain tiles in FlyByWire's terrain map format, made from X-Plane's DSF
//! scenery.
//!
//! SimBridge reads `terrain.map` (simbridge `apps/server/src/terrain/
//! fileformat/tile.ts`): one-degree tiles, each an N x N grid of
//! little-endian `i16` metres, gzip compressed, row 0 at the north edge,
//! `-1` marking water; a tile that is not in the file is all water. N is
//! 278 to 281 depending on latitude (0.215 nm per pixel). On loading, land
//! elevations become feet (`Math.round(m * 3.28084)`), water stays `-1`.
//!
//! The same grids are made here from the DSF X-Plane uses for the tile:
//! elevation from its `elevation` raster (or, for a mesh without one, the
//! mesh triangles), water from its `terrain_Water` patches. Tiles with no
//! DSF are water, as tiles absent from terrain.map are. Converted tiles are
//! kept on disk in terrain.map's own tile encoding so a tile is converted
//! once.

use std::io::{Read, Write};
use std::path::{Path, PathBuf};

use crate::mapdata::dsf::{Dsf, Triangle};
use crate::mapdata::scenery::{tile_name, Scenery};

/// The tile size FlyByWire's terrain map uses per latitude band, indexed by
/// the band's southern latitude + 90. Read from the tile headers of
/// terrain.map (0.215 nm per pixel on the WGS84 meridian, rounded as their
/// preprocessing did); bands terrain.map has no tiles for use the nearest.
const TILE_SIZE_BY_LATITUDE: [u16; 180] = [
    281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280,
    280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279,
    279, 279, 279, 279, 279, 279, 279, 279, 279, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278,
    278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278,
    278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 279, 279, 279, 279, 279,
    279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 280, 280, 280, 280, 280, 280, 280, 280, 280,
    280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 281, 281, 281, 281, 281, 281,
    281, 281, 281, 281, 281,
];

/// The water marker, in the file and after loading.
pub const WATER: i16 = -1;

pub fn tile_size(lat: i32) -> usize {
    TILE_SIZE_BY_LATITUDE[(lat + 90).clamp(0, 179) as usize] as usize
}

/// A loaded tile: `size` x `size` feet, row 0 north, `-1` water
/// (SimBridge's `ElevationGrid` after `Tile.loadElevationGrid`).
pub struct ElevationGrid {
    pub rows: usize,
    pub columns: usize,
    pub map: Vec<i16>,
}

/// JavaScript's `Math.round`: halves go up.
pub fn js_round(v: f64) -> f64 {
    (v + 0.5).floor()
}

impl ElevationGrid {
    /// From the file's metres, as `Tile.loadElevationGrid` converts them.
    pub fn from_metres(rows: usize, columns: usize, metres: &[i16]) -> Self {
        let map = metres.iter().map(|&m| if m == WATER { WATER } else { js_round(m as f64 * 3.28084) as i16 }).collect();
        Self { rows, columns, map }
    }
}

/// How the X-Plane raster is taken down to terrain map resolution.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Resample {
    /// Bilinear at the pixel's centre.
    Centre,
    /// The mean of the raster posts inside the pixel.
    Mean,
    /// The highest raster post inside the pixel.
    Max,
}

/// The resampling used. Chosen by comparing converted X-Plane tiles with the
/// same tiles of FlyByWire's terrain.map (see `tests::resampling_matches_*`):
/// both are SRTM-derived, and the area mean matched best.
pub const RESAMPLE: Resample = Resample::Mean;

/// Make a tile (metres, row 0 north, `-1` water) from a DSF with a mesh.
pub fn convert(dsf: &Dsf, lat: i32, lon: i32, resample: Resample) -> Result<Vec<i16>, String> {
    convert_with(dsf, lat, lon, resample, QUANTISE)
}

pub fn convert_with(dsf: &Dsf, lat: i32, lon: i32, resample: Resample, how: Quantise) -> Result<Vec<i16>, String> {
    let n = tile_size(lat);
    let mut metres = vec![i16::MIN; n * n];
    let water_index = dsf.terrain_names.iter().position(|t| t == "terrain_Water");
    let mut water = vec![false; n * n];
    let raster = dsf.raster("elevation");
    let mut mesh_elevation = raster.is_none().then(|| vec![f32::NAN; n * n]);
    dsf.triangles(|definition, overlay, t| {
        if overlay {
            return;
        }
        if Some(definition) == water_index {
            rasterise(t, lat, lon, n, |i, _| water[i] = true);
        }
        if let Some(mesh) = mesh_elevation.as_mut() {
            rasterise(t, lat, lon, n, |i, e| mesh[i] = e as f32);
        }
    })?;
    for row in 0..n {
        for col in 0..n {
            let i = row * n + col;
            let value = match (raster, &mesh_elevation) {
                (Some(r), _) => raster_value(r, row, col, n, resample),
                (None, Some(mesh)) => mesh[i],
                _ => f32::NAN,
            };
            metres[i] = if water[i] {
                WATER
            } else if value.is_nan() {
                0
            } else {
                // Land exactly at -1 m would read as water; FlyByWire's data
                // cannot hold it either.
                let m = js_round(value as f64).clamp(-32000., 32000.) as i16;
                if m == WATER { -2 } else { m }
            };
        }
    }
    quantise(&mut metres, how);
    Ok(metres)
}

/// FlyByWire's terrain map holds land elevations in 50 m steps counted from
/// each tile's lowest land elevation (every value in terrain.map is the tile
/// minimum plus a multiple of 50).
pub const HEIGHT_STEP: i32 = 50;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Quantise {
    None,
    Floor,
    Round,
}

/// Measured against terrain.map (`tests::resampling_matches_*`).
pub const QUANTISE: Quantise = Quantise::Floor;

pub fn quantise(metres: &mut [i16], how: Quantise) {
    let Some(min) = metres.iter().filter(|&&m| m != WATER).min().map(|&m| m as i32) else { return };
    for m in metres.iter_mut().filter(|m| **m != WATER) {
        let steps = (*m as i32 - min) as f64 / HEIGHT_STEP as f64;
        let steps = match how {
            Quantise::None => continue,
            Quantise::Floor => steps.floor(),
            Quantise::Round => js_round(steps),
        };
        let v = min + steps as i32 * HEIGHT_STEP;
        *m = if v == WATER as i32 { -2 } else { v as i16 };
    }
}

fn raster_value(r: &crate::mapdata::dsf::Raster, row: usize, col: usize, n: usize, resample: Resample) -> f32 {
    // Pixel edges in 0..1 of the tile, from the west and the south.
    let (x0, x1) = (col as f64 / n as f64, (col + 1) as f64 / n as f64);
    let (y1, y0) = (1. - row as f64 / n as f64, 1. - (row + 1) as f64 / n as f64);
    match resample {
        Resample::Centre => r.sample((x0 + x1) / 2., (y0 + y1) / 2.),
        Resample::Mean | Resample::Max => {
            let (w, h) = ((r.width - 1) as f64, (r.height - 1) as f64);
            let (cx0, cx1) = ((x0 * w).ceil() as usize, ((x1 * w).floor() as usize).min(r.width - 1));
            let (cy0, cy1) = ((y0 * h).ceil() as usize, ((y1 * h).floor() as usize).min(r.height - 1));
            if cx0 > cx1 || cy0 > cy1 {
                return r.sample((x0 + x1) / 2., (y0 + y1) / 2.);
            }
            let (mut sum, mut count, mut max) = (0f64, 0usize, f32::MIN);
            for y in cy0..=cy1 {
                for x in cx0..=cx1 {
                    let v = r.values[y * r.width + x];
                    sum += v as f64;
                    count += 1;
                    max = max.max(v);
                }
            }
            if resample == Resample::Max { max } else { (sum / count as f64) as f32 }
        }
    }
}

/// Every tile pixel whose centre a triangle covers, with the triangle's
/// elevation there.
fn rasterise(t: &Triangle, lat: i32, lon: i32, n: usize, mut f: impl FnMut(usize, f64)) {
    // Pixel coordinates: x east from the west edge, y south from the north
    // edge, centres at .5.
    let p: Vec<(f64, f64, f64)> = t.iter().map(|c| ((c[0] - lon as f64) * n as f64, (lat as f64 + 1. - c[1]) * n as f64, c[2])).collect();
    let (min_x, max_x) = (p.iter().map(|v| v.0).fold(f64::MAX, f64::min), p.iter().map(|v| v.0).fold(f64::MIN, f64::max));
    let (min_y, max_y) = (p.iter().map(|v| v.1).fold(f64::MAX, f64::min), p.iter().map(|v| v.1).fold(f64::MIN, f64::max));
    let area = (p[1].0 - p[0].0) * (p[2].1 - p[0].1) - (p[2].0 - p[0].0) * (p[1].1 - p[0].1);
    if area.abs() < 1e-12 {
        return;
    }
    let first_x = (min_x - 0.5).ceil().max(0.) as usize;
    let last_x = ((max_x - 0.5).floor()).min(n as f64 - 1.);
    let first_y = (min_y - 0.5).ceil().max(0.) as usize;
    let last_y = ((max_y - 0.5).floor()).min(n as f64 - 1.);
    if last_x < 0. || last_y < 0. {
        return;
    }
    for y in first_y..=last_y as usize {
        let py = y as f64 + 0.5;
        for x in first_x..=last_x as usize {
            let px = x as f64 + 0.5;
            let w0 = ((p[1].0 - px) * (p[2].1 - py) - (p[2].0 - px) * (p[1].1 - py)) / area;
            let w1 = ((p[2].0 - px) * (p[0].1 - py) - (p[0].0 - px) * (p[2].1 - py)) / area;
            let w2 = 1. - w0 - w1;
            const EDGE: f64 = -1e-9;
            if w0 >= EDGE && w1 >= EDGE && w2 >= EDGE {
                f(y * n + x, w0 * p[0].2 + w1 * p[1].2 + w2 * p[2].2);
            }
        }
    }
}

/// A tile as terrain.map stores it: the 11-byte tile header and the gzip
/// elevation data.
pub fn encode(lat: i32, lon: i32, metres: &[i16]) -> Vec<u8> {
    let n = tile_size(lat);
    let mut raw = Vec::with_capacity(metres.len() * 2);
    for m in metres {
        raw.extend(m.to_le_bytes());
    }
    let mut gz = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::fast());
    let _ = gz.write_all(&raw);
    let compressed = gz.finish().unwrap_or_default();
    let mut out = Vec::with_capacity(11 + compressed.len());
    out.extend((n as u16).to_le_bytes());
    out.extend((n as u16).to_le_bytes());
    out.push(lat as i8 as u8);
    out.extend((lon as i16).to_le_bytes());
    out.extend((compressed.len() as u32).to_le_bytes());
    out.extend(compressed);
    out
}

/// A terrain.map tile back to metres: (rows, columns, south-west, data).
pub fn decode(bytes: &[u8]) -> Result<(usize, usize, (i32, i32), Vec<i16>), String> {
    if bytes.len() < 11 {
        return Err("a short tile".into());
    }
    let rows = u16::from_le_bytes([bytes[0], bytes[1]]) as usize;
    let columns = u16::from_le_bytes([bytes[2], bytes[3]]) as usize;
    let lat = bytes[4] as i8 as i32;
    let lon = i16::from_le_bytes([bytes[5], bytes[6]]) as i32;
    let size = u32::from_le_bytes(bytes[7..11].try_into().unwrap()) as usize;
    let data = bytes.get(11..11 + size).ok_or("a tile shorter than its header says")?;
    let mut raw = Vec::with_capacity(rows * columns * 2);
    flate2::read::GzDecoder::new(data).read_to_end(&mut raw).map_err(|e| format!("tile data: {e}"))?;
    if raw.len() < rows * columns * 2 {
        return Err("tile data shorter than its grid".into());
    }
    let metres = raw.chunks_exact(2).take(rows * columns).map(|c| i16::from_le_bytes([c[0], c[1]])).collect();
    Ok((rows, columns, (lat, lon), metres))
}

/// What a tile's sources gave.
pub enum Loaded {
    /// A tile from a DSF mesh.
    Grid(ElevationGrid),
    /// No DSF has a mesh for this tile: water.
    Water,
}

/// Loads tiles for the terrain map: from the cache when the DSF behind it
/// has not changed, otherwise converted from the DSF and cached.
pub struct TileSource {
    scenery: Scenery,
    cache: Option<PathBuf>,
}

impl TileSource {
    pub fn new(scenery: Scenery, cache: Option<PathBuf>) -> Self {
        if let Some(dir) = &cache {
            let _ = std::fs::create_dir_all(dir);
        }
        Self { scenery, cache }
    }

    /// Whether any scenery has a DSF for this tile. Cheap: file checks.
    pub fn exists(&self, lat: i32, lon: i32) -> bool {
        !self.scenery.candidates(lat, lon).is_empty()
    }

    pub fn load(&self, lat: i32, lon: i32) -> Result<Loaded, String> {
        for path in self.scenery.candidates(lat, lon) {
            let stamp = stamp(&path);
            let cached = self.cache.as_ref().map(|dir| dir.join(format!("{}-{:016x}.tile", tile_name(lat, lon), fnv(path.to_string_lossy().as_bytes()))));
            if let Some(file) = &cached {
                if let Ok(bytes) = std::fs::read(file) {
                    if let Some(result) = read_cached(&bytes, stamp) {
                        match result? {
                            Some(grid) => return Ok(Loaded::Grid(grid)),
                            None => continue,
                        }
                    }
                }
            }
            let dsf = Dsf::read(&path)?;
            let overlay = dsf.is_overlay();
            let tile = if overlay { None } else { Some(convert(&dsf, lat, lon, RESAMPLE)?) };
            if let Some(file) = &cached {
                let mut out = Vec::new();
                out.extend(b"FBWXPT1\0");
                out.extend(stamp.0.to_le_bytes());
                out.extend(stamp.1.to_le_bytes());
                match &tile {
                    Some(metres) => {
                        out.push(1);
                        out.extend(encode(lat, lon, metres));
                    }
                    None => out.push(0),
                }
                let temp = file.with_extension("part");
                if std::fs::write(&temp, &out).is_ok() {
                    let _ = std::fs::rename(&temp, file);
                }
            }
            if let Some(metres) = tile {
                let n = tile_size(lat);
                return Ok(Loaded::Grid(ElevationGrid::from_metres(n, n, &metres)));
            }
        }
        Ok(Loaded::Water)
    }
}

/// A cached conversion, if it was made from this version of the DSF:
/// `Some(Ok(Some(grid)))` a tile, `Some(Ok(None))` an overlay.
fn read_cached(bytes: &[u8], stamp: (u64, u64)) -> Option<Result<Option<ElevationGrid>, String>> {
    if bytes.len() < 25 || &bytes[0..8] != b"FBWXPT1\0" {
        return None;
    }
    let len = u64::from_le_bytes(bytes[8..16].try_into().ok()?);
    let modified = u64::from_le_bytes(bytes[16..24].try_into().ok()?);
    if (len, modified) != stamp {
        return None;
    }
    if bytes[24] == 0 {
        return Some(Ok(None));
    }
    Some(decode(&bytes[25..]).map(|(rows, columns, _, metres)| Some(ElevationGrid::from_metres(rows, columns, &metres))))
}

fn stamp(path: &Path) -> (u64, u64) {
    let meta = std::fs::metadata(path).ok();
    let len = meta.as_ref().map_or(0, |m| m.len());
    let modified = meta
        .and_then(|m| m.modified().ok())
        .and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok())
        .map_or(0, |d| d.as_secs());
    (len, modified)
}

fn fnv(bytes: &[u8]) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for b in bytes {
        h ^= *b as u64;
        h = h.wrapping_mul(0x0100_0000_01b3);
    }
    h
}

#[cfg(test)]
mod tests {
    use super::*;

    const XPLANE: &str = "D:/Steam Games/steamapps/common/X-Plane 12";
    const REFERENCE: &str = "D:/fbw-build/terrain-reference.map";

    #[test]
    fn tiles_round_trip_through_terrain_map_encoding() {
        let n = tile_size(47);
        assert_eq!(n, 279);
        let metres: Vec<i16> = (0..n * n).map(|i| if i % 7 == 0 { WATER } else { (i % 3000) as i16 }).collect();
        let bytes = encode(47, 11, &metres);
        let (rows, columns, sw, back) = decode(&bytes).unwrap();
        assert_eq!((rows, columns, sw), (n, n, (47, 11)));
        assert_eq!(back, metres);
        let grid = ElevationGrid::from_metres(rows, columns, &back);
        assert_eq!(grid.map[0], WATER);
        // 1000 m is 3281 ft, as Math.round(1000 * 3.28084).
        assert_eq!(grid.map[1000], 3281);
    }

    #[test]
    fn tile_sizes_follow_latitude() {
        assert_eq!(tile_size(0), 278);
        assert_eq!(tile_size(-83), 281);
        assert_eq!(tile_size(55), 280);
        assert_eq!(tile_size(89), 281);
    }

    #[test]
    fn a_water_triangle_marks_the_pixels_it_covers() {
        use crate::mapdata::dsf::tests_support::sample_dsf;
        let dsf = Dsf::parse(&sample_dsf()).unwrap();
        let metres = convert_with(&dsf, 47, 11, Resample::Centre, Quantise::None).unwrap();
        let n = tile_size(47);
        // The water strip covers the south-west half: (11,47), (12,47), (11,48).
        let south_west = metres[(n - 1) * n];
        let north_east = metres[n - 1];
        assert_eq!(south_west, WATER);
        assert_ne!(north_east, WATER);
        // The raster rises from 100 m in the south-west to 400 m in the north-east.
        assert!((395..=400).contains(&north_east), "{north_east}");
        // Quantised, land is the tile's lowest land plus whole 50 m steps.
        let quantised = convert(&dsf, 47, 11, Resample::Centre).unwrap();
        let min = quantised.iter().filter(|&&m| m != WATER).min().copied().unwrap();
        assert!(quantised.iter().filter(|&&m| m != WATER).all(|&m| (m - min) % 50 == 0));
    }

    /// Measures how X-Plane's converted tiles compare with FlyByWire's for
    /// the same tiles, for each resampling. Needs X-Plane and a copy of
    /// terrain.map (research only; it is never shipped or read at run time).
    #[test]
    #[ignore]
    fn resampling_matches_flybywire_terrain_map() {
        let reference = std::fs::read(REFERENCE).expect("terrain.map for comparison");
        let mut tiles = std::collections::HashMap::new();
        let mut off = 14;
        while off + 11 <= reference.len() {
            let size = u32::from_le_bytes(reference[off + 7..off + 11].try_into().unwrap()) as usize;
            let lat = reference[off + 4] as i8 as i32;
            let lon = i16::from_le_bytes([reference[off + 5], reference[off + 6]]) as i32;
            tiles.insert((lat, lon), off);
            off += 11 + size;
        }
        let scenery = Scenery::of_installation(Path::new(XPLANE));
        for (lat, lon) in [(47, 11), (46, 7), (40, -74), (40, -122), (27, 86), (-34, 18)] {
            let Some(path) = scenery.candidates(lat, lon).into_iter().last() else { continue };
            let t = std::time::Instant::now();
            let dsf = Dsf::read(&path).unwrap();
            let read = t.elapsed();
            let (_, _, _, fbw) = decode(&reference[tiles[&(lat, lon)]..]).unwrap();
            for (resample, how) in [(Resample::Centre, Quantise::None), (Resample::Mean, Quantise::None), (Resample::Centre, Quantise::Floor), (Resample::Mean, Quantise::Floor), (Resample::Mean, Quantise::Round), (Resample::Max, Quantise::Floor)] {
                let t = std::time::Instant::now();
                let mut xp = convert(&dsf, lat, lon, resample).unwrap();
                // convert() applies the chosen quantisation; undo nothing, re-run from raw for others.
                if how != QUANTISE {
                    xp = convert_with(&dsf, lat, lon, resample, how).unwrap();
                }
                let took = t.elapsed();
                let (mut land, mut err, mut abs_sum, mut water_agree) = (0usize, 0f64, 0f64, 0usize);
                for (a, b) in xp.iter().zip(&fbw) {
                    if (*a == WATER) == (*b == WATER) {
                        water_agree += 1;
                    }
                    if *a != WATER && *b != WATER {
                        land += 1;
                        let d = (*a as f64) - (*b as f64);
                        err += d;
                        abs_sum += d.abs();
                    }
                }
                println!(
                    "{} {resample:?} {how:?}: read {read:?} convert {took:?}; land pixels {land}, mean error {:.1} m, mean abs error {:.1} m; water agreement {:.1}%",
                    tile_name(lat, lon),
                    err / land.max(1) as f64,
                    abs_sum / land.max(1) as f64,
                    100. * water_agree as f64 / xp.len() as f64
                );
            }
        }
    }
}
