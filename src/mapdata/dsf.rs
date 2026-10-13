//! X-Plane's DSF scenery files: the parts the terrain map needs.
//!
//! A DSF tile covers one degree by one degree. X-Plane 12 ships them 7z
//! compressed. Inside (developer.x-plane.com, "DSF File Format
//! Specification"): the `XPLNEDSF` cookie and version, then atoms (a 4-byte
//! id stored little-endian, so `HEAD` reads "DAEH" on disk, and a 32-bit
//! length that includes the 8-byte header), then a 16-byte MD5 footer.
//!
//! Read here:
//! - `HEAD/PROP`: the properties, to tell an overlay (`sim/overlay 1`) from a
//!   base mesh;
//! - `DEFN/TERT`: terrain definition names, to find `terrain_Water`;
//! - `DEFN/DEMN` + `DEMS` (`DEMI`, `DEMD`): raster layers; X-Plane 12's
//!   `elevation` layer is 1201 x 1201 signed 16-bit metres, post-centric,
//!   row 0 at the south edge;
//! - `GEOD` (`POOL` + `SCAL`) and `CMDS`: the terrain mesh, as triangles with
//!   the terrain definition each patch uses.


/// A tile's contents, as far as the terrain map needs them.
#[derive(Default)]
pub struct Dsf {
    pub properties: Vec<(String, String)>,
    pub terrain_names: Vec<String>,
    pub rasters: Vec<Raster>,
    /// Coordinate pools: each is `planes` values per point.
    pools: Vec<Pool>,
    commands: Vec<u8>,
}

/// One raster layer.
pub struct Raster {
    pub name: String,
    pub width: usize,
    pub height: usize,
    /// Post-centric: edge pixels lie exactly on the tile's edges.
    pub post_centric: bool,
    /// Final values (raw * scale + offset), row 0 south, west to east.
    pub values: Vec<f32>,
}

struct Pool {
    planes: usize,
    /// `planes` doubles per point, already scaled.
    data: Vec<f64>,
}

/// A mesh triangle: three (longitude, latitude, elevation) corners.
pub type Triangle = [[f64; 3]; 3];

impl Dsf {
    /// Read a DSF file, 7z compressed or not.
    pub fn read(path: &std::path::Path) -> Result<Self, String> {
        let bytes = std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
        Self::parse(&decompress(bytes)?)
    }

    pub fn parse(data: &[u8]) -> Result<Self, String> {
        if data.len() < 12 + 16 || &data[0..8] != b"XPLNEDSF" {
            return Err("not a DSF file".into());
        }
        let version = u32::from_le_bytes(data[8..12].try_into().unwrap());
        if version != 1 {
            return Err(format!("DSF version {version} is not supported"));
        }
        let mut dsf = Dsf::default();
        let body = &data[12..data.len() - 16];
        let mut raster_names = Vec::new();
        let mut raster_infos = Vec::new();
        let mut scales: Vec<Vec<(f32, f32)>> = Vec::new();
        let mut raw_pools: Vec<(usize, Vec<Vec<u16>>)> = Vec::new();
        for (id, atom) in atoms(body)? {
            match &id {
                b"HEAD" => {
                    for (id, sub) in atoms(atom)? {
                        if &id == b"PROP" {
                            let strings = strings(sub);
                            dsf.properties = strings.chunks(2).filter(|c| c.len() == 2).map(|c| (c[0].clone(), c[1].clone())).collect();
                        }
                    }
                }
                b"DEFN" => {
                    for (id, sub) in atoms(atom)? {
                        match &id {
                            b"TERT" => dsf.terrain_names = strings(sub),
                            b"DEMN" => raster_names = strings(sub),
                            _ => {}
                        }
                    }
                }
                b"GEOD" => {
                    for (id, sub) in atoms(atom)? {
                        match &id {
                            b"POOL" => raw_pools.push(read_pool(sub)?),
                            b"SCAL" => scales.push(
                                sub.chunks_exact(8)
                                    .map(|c| (f32::from_le_bytes(c[0..4].try_into().unwrap()), f32::from_le_bytes(c[4..8].try_into().unwrap())))
                                    .collect(),
                            ),
                            _ => {}
                        }
                    }
                }
                b"DEMS" => {
                    for (id, sub) in atoms(atom)? {
                        match &id {
                            b"DEMI" => raster_infos.push(read_dem_info(sub)?),
                            b"DEMD" => {
                                let index = dsf.rasters.len();
                                let info = raster_infos.get(index).ok_or("raster data before its information")?;
                                let name = raster_names.get(index).cloned().unwrap_or_default();
                                dsf.rasters.push(read_dem_data(name, info, sub)?);
                            }
                            _ => {}
                        }
                    }
                }
                b"CMDS" => dsf.commands = atom.to_vec(),
                _ => {}
            }
        }
        for (i, (count, planes)) in raw_pools.into_iter().enumerate() {
            let scale = scales.get(i).ok_or("a coordinate pool without its scale")?;
            if scale.len() < planes.len() {
                return Err("a coordinate pool scale with too few planes".into());
            }
            let mut data = vec![0.; count * planes.len()];
            for (p, plane) in planes.iter().enumerate() {
                let (multiplier, offset) = (scale[p].0 as f64, scale[p].1 as f64);
                for (n, &raw) in plane.iter().enumerate() {
                    // A 16-bit pool spans its plane's range: raw / 65535 of
                    // the multiplier, from the offset.
                    data[n * planes.len() + p] = if multiplier == 0. { offset } else { raw as f64 / 65535. * multiplier + offset };
                }
            }
            dsf.pools.push(Pool { planes: planes.len(), data });
        }
        Ok(dsf)
    }

    pub fn property(&self, name: &str) -> Option<&str> {
        self.properties.iter().find(|(k, _)| k == name).map(|(_, v)| v.as_str())
    }

    /// Whether this DSF only overlays another tile's mesh.
    pub fn is_overlay(&self) -> bool {
        self.property("sim/overlay").is_some_and(|v| v.trim() == "1")
    }

    pub fn raster(&self, name: &str) -> Option<&Raster> {
        self.rasters.iter().find(|r| r.name == name)
    }

    /// Every base-mesh triangle (not overlay patches), with its terrain
    /// definition index.
    pub fn triangles(&self, mut each: impl FnMut(usize, bool, &Triangle)) -> Result<(), String> {
        let c = &self.commands;
        let mut at = 0usize;
        let mut pool = 0usize;
        let mut definition = 0usize;
        let mut flags = 1u8;
        let take = |at: &mut usize, n: usize| -> Result<&[u8], String> {
            let s = c.get(*at..*at + n).ok_or("the command stream ends early")?;
            *at += n;
            Ok(s)
        };
        let u8_ = |at: &mut usize| -> Result<u8, String> { Ok(take(at, 1)?[0]) };
        let u16_ = |at: &mut usize| -> Result<u16, String> { Ok(u16::from_le_bytes(take(at, 2)?.try_into().unwrap())) };
        let u32_ = |at: &mut usize| -> Result<u32, String> { Ok(u32::from_le_bytes(take(at, 4)?.try_into().unwrap())) };
        let point = |pool: usize, index: usize| -> Result<[f64; 3], String> {
            let p = self.pools.get(pool).ok_or("a triangle in a missing pool")?;
            if p.planes < 3 {
                return Err("a terrain pool with fewer than three planes".into());
            }
            let base = index * p.planes;
            let v = p.data.get(base..base + 3).ok_or("a triangle vertex outside its pool")?;
            Ok([v[0], v[1], v[2]])
        };
        let mut emit = |corners: [(usize, usize); 3], definition: usize, flags: u8| -> Result<(), String> {
            let t = [point(corners[0].0, corners[0].1)?, point(corners[1].0, corners[1].1)?, point(corners[2].0, corners[2].1)?];
            // Overlay patches (flag 2) are drawn over another patch.
            each(definition, flags & 2 != 0, &t);
            Ok(())
        };
        while at < c.len() {
            let op = u8_(&mut at)?;
            match op {
                1 => pool = u16_(&mut at)? as usize,
                2 => {
                    u32_(&mut at)?;
                }
                3 => definition = u8_(&mut at)? as usize,
                4 => definition = u16_(&mut at)? as usize,
                5 => definition = u32_(&mut at)? as usize,
                6 => {
                    u8_(&mut at)?;
                }
                7 => {
                    u16_(&mut at)?;
                }
                8 | 10 => {
                    take(&mut at, 4)?;
                }
                9 => {
                    let n = u8_(&mut at)? as usize;
                    take(&mut at, n * 2)?;
                }
                11 => {
                    let n = u8_(&mut at)? as usize;
                    take(&mut at, n * 4)?;
                }
                12 => {
                    u16_(&mut at)?;
                    let n = u8_(&mut at)? as usize;
                    take(&mut at, n * 2)?;
                }
                13 => {
                    take(&mut at, 6)?;
                }
                14 => {
                    u16_(&mut at)?;
                    let windings = u8_(&mut at)?;
                    for _ in 0..windings {
                        let n = u8_(&mut at)? as usize;
                        take(&mut at, n * 2)?;
                    }
                }
                15 => {
                    // Nested polygon range: the count is the number of
                    // windings, followed by one more index than that (each
                    // winding's start, then the end).
                    u16_(&mut at)?;
                    let n = u8_(&mut at)? as usize;
                    take(&mut at, (n + 1) * 2)?;
                }
                16 => {}
                17 => flags = u8_(&mut at)?,
                18 => {
                    flags = u8_(&mut at)?;
                    take(&mut at, 8)?;
                }
                23 | 26 | 29 => {
                    let n = u8_(&mut at)? as usize;
                    let mut v = Vec::with_capacity(n);
                    for _ in 0..n {
                        v.push((pool, u16_(&mut at)? as usize));
                    }
                    primitive(op, &v, |t| emit(t, definition, flags))?;
                }
                24 | 27 | 30 => {
                    let n = u8_(&mut at)? as usize;
                    let mut v = Vec::with_capacity(n);
                    for _ in 0..n {
                        let p = u16_(&mut at)? as usize;
                        v.push((p, u16_(&mut at)? as usize));
                    }
                    primitive(op - 1, &v, |t| emit(t, definition, flags))?;
                }
                25 | 28 | 31 => {
                    let first = u16_(&mut at)? as usize;
                    let end = u16_(&mut at)? as usize;
                    let v: Vec<(usize, usize)> = (first..end.max(first)).map(|i| (pool, i)).collect();
                    primitive(op - 2, &v, |t| emit(t, definition, flags))?;
                }
                32 => {
                    let n = u8_(&mut at)? as usize;
                    take(&mut at, n)?;
                }
                33 => {
                    let n = u16_(&mut at)? as usize;
                    take(&mut at, n)?;
                }
                34 => {
                    let n = u32_(&mut at)? as usize;
                    take(&mut at, n)?;
                }
                other => return Err(format!("unknown DSF command {other} at {} of {}: {:?}", at - 1, c.len(), &c[at.saturating_sub(40)..(at + 8).min(c.len())])),
            }
        }
        Ok(())
    }
}

/// Triangles from a list, strip (23+3) or fan (23+6) of vertices.
fn primitive<T: Copy>(op: u8, v: &[T], mut f: impl FnMut([T; 3]) -> Result<(), String>) -> Result<(), String> {
    match op {
        23 => {
            for t in v.chunks_exact(3) {
                f([t[0], t[1], t[2]])?;
            }
        }
        26 => {
            for i in 2..v.len() {
                if i % 2 == 0 {
                    f([v[i - 2], v[i - 1], v[i]])?;
                } else {
                    f([v[i - 1], v[i - 2], v[i]])?;
                }
            }
        }
        29 => {
            for i in 2..v.len() {
                f([v[0], v[i - 1], v[i]])?;
            }
        }
        _ => {}
    }
    Ok(())
}

/// The DSF inside a 7z archive, or the bytes as they are.
pub fn decompress(bytes: Vec<u8>) -> Result<Vec<u8>, String> {
    if !bytes.starts_with(b"7z\xBC\xAF\x27\x1C") {
        return Ok(bytes);
    }
    let mut reader = sevenz_rust2::ArchiveReader::new(std::io::Cursor::new(bytes), sevenz_rust2::Password::empty())
        .map_err(|e| format!("7z: {e}"))?;
    let mut out = Vec::new();
    reader
        .for_each_entries(|_, entry| {
            if out.is_empty() {
                entry.read_to_end(&mut out)?;
            }
            Ok(true)
        })
        .map_err(|e| format!("7z: {e}"))?;
    Ok(out)
}

fn atoms(mut data: &[u8]) -> Result<Vec<([u8; 4], &[u8])>, String> {
    let mut out = Vec::new();
    while data.len() >= 8 {
        let mut id: [u8; 4] = data[0..4].try_into().unwrap();
        id.reverse();
        let len = u32::from_le_bytes(data[4..8].try_into().unwrap()) as usize;
        if len < 8 || len > data.len() {
            return Err(format!("a DSF atom {} with a bad length", String::from_utf8_lossy(&id)));
        }
        out.push((id, &data[8..len]));
        data = &data[len..];
    }
    Ok(out)
}

fn strings(data: &[u8]) -> Vec<String> {
    let mut v: Vec<String> = data.split(|&b| b == 0).map(|s| String::from_utf8_lossy(s).into_owned()).collect();
    // The table ends with a terminator, which leaves one empty string.
    if v.last().is_some_and(String::is_empty) {
        v.pop();
    }
    v
}

/// A 16-bit planar numeric pool: item count, plane count, then per plane an
/// encoding (0 raw, 1 differenced, 2 run-length, 3 run-length differenced)
/// and its values.
fn read_pool(data: &[u8]) -> Result<(usize, Vec<Vec<u16>>), String> {
    if data.len() < 5 {
        return Err("a short coordinate pool".into());
    }
    let count = u32::from_le_bytes(data[0..4].try_into().unwrap()) as usize;
    let plane_count = data[4] as usize;
    let mut at = 5;
    let mut planes = Vec::with_capacity(plane_count);
    let short = || "a coordinate pool ends early".to_string();
    for _ in 0..plane_count {
        let encoding = *data.get(at).ok_or_else(short)?;
        at += 1;
        let mut values = Vec::with_capacity(count);
        let value = |at: usize| -> Result<u16, String> {
            Ok(u16::from_le_bytes(data.get(at..at + 2).ok_or_else(short)?.try_into().unwrap()))
        };
        if encoding & 2 == 0 {
            for _ in 0..count {
                values.push(value(at)?);
                at += 2;
            }
        } else {
            while values.len() < count {
                let code = *data.get(at).ok_or_else(short)?;
                at += 1;
                let n = (code & 0x7F) as usize;
                if code & 0x80 != 0 {
                    let v = value(at)?;
                    at += 2;
                    values.extend(std::iter::repeat_n(v, n));
                } else {
                    for _ in 0..n {
                        values.push(value(at)?);
                        at += 2;
                    }
                }
            }
            values.truncate(count);
        }
        if encoding & 1 != 0 {
            let mut sum = 0u16;
            for v in &mut values {
                sum = sum.wrapping_add(*v);
                *v = sum;
            }
        }
        planes.push(values);
    }
    Ok((count, planes))
}

struct DemInfo {
    bytes_per_pixel: usize,
    flags: u16,
    width: usize,
    height: usize,
    scale: f32,
    offset: f32,
}

fn read_dem_info(d: &[u8]) -> Result<DemInfo, String> {
    if d.len() < 20 {
        return Err("a short raster information atom".into());
    }
    Ok(DemInfo {
        bytes_per_pixel: d[1] as usize,
        flags: u16::from_le_bytes([d[2], d[3]]),
        width: u32::from_le_bytes(d[4..8].try_into().unwrap()) as usize,
        height: u32::from_le_bytes(d[8..12].try_into().unwrap()) as usize,
        scale: f32::from_le_bytes(d[12..16].try_into().unwrap()),
        offset: f32::from_le_bytes(d[16..20].try_into().unwrap()),
    })
}

fn read_dem_data(name: String, info: &DemInfo, d: &[u8]) -> Result<Raster, String> {
    let (bpp, n) = (info.bytes_per_pixel, info.width * info.height);
    if d.len() < n * bpp {
        return Err(format!("raster {name} is shorter than its size"));
    }
    let kind = info.flags & 3;
    let raw = |i: usize| -> f32 {
        let b = &d[i * bpp..(i + 1) * bpp];
        match (kind, bpp) {
            (0, 4) => f32::from_le_bytes(b.try_into().unwrap()),
            (1, 1) => b[0] as i8 as f32,
            (1, 2) => i16::from_le_bytes(b.try_into().unwrap()) as f32,
            (1, 4) => i32::from_le_bytes(b.try_into().unwrap()) as f32,
            (_, 1) => b[0] as f32,
            (_, 2) => u16::from_le_bytes(b.try_into().unwrap()) as f32,
            (_, _) => u32::from_le_bytes(b[0..4].try_into().unwrap()) as f32,
        }
    };
    let values = (0..n).map(|i| raw(i) * info.scale + info.offset).collect();
    Ok(Raster { name, width: info.width, height: info.height, post_centric: info.flags & 4 != 0, values })
}

impl Raster {
    /// The value at a position inside the tile, bilinearly interpolated.
    /// `x` and `y` are 0..1 from the west and south edges.
    pub fn sample(&self, x: f64, y: f64) -> f32 {
        let (w, h) = (self.width as f64, self.height as f64);
        let (px, py) = if self.post_centric { (x * (w - 1.), y * (h - 1.)) } else { (x * w - 0.5, y * h - 0.5) };
        let px = px.clamp(0., w - 1.);
        let py = py.clamp(0., h - 1.);
        let (x0, y0) = (px.floor() as usize, py.floor() as usize);
        let (x1, y1) = ((x0 + 1).min(self.width - 1), (y0 + 1).min(self.height - 1));
        let (fx, fy) = ((px - x0 as f64) as f32, (py - y0 as f64) as f32);
        let at = |x: usize, y: usize| self.values[y * self.width + x];
        let south = at(x0, y0) * (1. - fx) + at(x1, y0) * fx;
        let north = at(x0, y1) * (1. - fx) + at(x1, y1) * fx;
        south * (1. - fy) + north * fy
    }
}

#[cfg(test)]
pub(crate) mod tests_support {
    fn atom(id: &[u8; 4], body: &[u8]) -> Vec<u8> {
        let mut v = id.to_vec();
        v.reverse();
        v.extend(((body.len() + 8) as u32).to_le_bytes());
        v.extend(body);
        v
    }

    /// A tiny DSF: two terrain types, one pool of four corners, a 2x2
    /// raster, and a water patch as a strip plus a land triangle.
    pub fn sample_dsf() -> Vec<u8> {
        let mut pool = 4u32.to_le_bytes().to_vec();
        pool.push(3);
        // lon plane: raw (encoding 0)
        pool.push(0);
        for v in [0u16, 65535, 0, 65535] {
            pool.extend(v.to_le_bytes());
        }
        // lat plane: RLE differenced (3): differences 0, 0, 65535 as a
        // literal run, then a repeated 0
        pool.push(3);
        pool.push(3);
        for v in [0u16, 0, 65535] {
            pool.extend(v.to_le_bytes());
        }
        pool.push(0x81);
        pool.extend(0u16.to_le_bytes());
        // elevation plane: RLE (2), a run of four
        pool.push(2);
        pool.push(0x84);
        pool.extend(32768u16.to_le_bytes());
        let mut scal = Vec::new();
        for (m, o) in [(1f32, 11f32), (1., 47.), (1000., 0.)] {
            scal.extend(m.to_le_bytes());
            scal.extend(o.to_le_bytes());
        }
        let mut demi = vec![1u8, 2];
        demi.extend(5u16.to_le_bytes());
        demi.extend(2u32.to_le_bytes());
        demi.extend(2u32.to_le_bytes());
        demi.extend(1f32.to_le_bytes());
        demi.extend(0f32.to_le_bytes());
        let mut demd = Vec::new();
        for v in [100i16, 200, 300, 400] {
            demd.extend(v.to_le_bytes());
        }
        let mut cmds = vec![1u8, 0, 0, 3, 1, 17, 1, 26, 3];
        for i in [0u16, 1, 2] {
            cmds.extend(i.to_le_bytes());
        }
        cmds.extend([32u8, 2, b'h', b'i', 3, 0, 18, 3]);
        cmds.extend([0u8; 8]);
        cmds.extend([25u8]);
        cmds.extend(1u16.to_le_bytes());
        cmds.extend(4u16.to_le_bytes());
        let mut body = b"XPLNEDSF".to_vec();
        body.extend(1u32.to_le_bytes());
        body.extend(atom(b"HEAD", &atom(b"PROP", b"sim/west\011\0sim/overlay\00\0")));
        let mut defn = atom(b"TERT", b"lib/land.ter\0terrain_Water\0");
        defn.extend(atom(b"DEMN", b"elevation\0"));
        body.extend(atom(b"DEFN", &defn));
        let mut geod = atom(b"POOL", &pool);
        geod.extend(atom(b"SCAL", &scal));
        body.extend(atom(b"GEOD", &geod));
        let mut dems = atom(b"DEMI", &demi);
        dems.extend(atom(b"DEMD", &demd));
        body.extend(atom(b"DEMS", &dems));
        body.extend(atom(b"CMDS", &cmds));
        body.extend([0u8; 16]);
        body
    }

}

#[cfg(test)]
mod tests {
    use super::*;
    use super::tests_support::sample_dsf;

    #[test]
    fn properties_definitions_and_rasters_are_read() {
        let dsf = Dsf::parse(&sample_dsf()).unwrap();
        assert_eq!(dsf.property("sim/west"), Some("11"));
        assert!(!dsf.is_overlay());
        assert_eq!(dsf.terrain_names, vec!["lib/land.ter", "terrain_Water"]);
        let r = dsf.raster("elevation").unwrap();
        assert!(r.post_centric);
        assert_eq!((r.width, r.height), (2, 2));
        assert_eq!(r.sample(0., 0.), 100.);
        assert_eq!(r.sample(1., 1.), 400.);
        assert_eq!(r.sample(0.5, 0.5), 250.);
    }

    #[test]
    fn mesh_triangles_come_from_strips_and_ranges() {
        let dsf = Dsf::parse(&sample_dsf()).unwrap();
        let mut seen = Vec::new();
        dsf.triangles(|def, overlay, t| seen.push((def, overlay, *t))).unwrap();
        assert_eq!(seen.len(), 2);
        // The strip: water, physical.
        assert_eq!(seen[0].0, 1);
        assert!(!seen[0].1);
        assert_eq!(seen[0].2[0], [11., 47., 1000. * 32768. / 65535.]);
        assert_eq!(seen[0].2[1], [12., 47., 1000. * 32768. / 65535.]);
        assert_eq!(seen[0].2[2][1], 48.);
        // The range 1..4 as a list: one triangle from points 1, 2, 3, in an overlay patch.
        assert_eq!(seen[1].0, 0);
        assert!(seen[1].1);
        assert_eq!(seen[1].2[2][0..2], [12., 48.]);
    }

    #[test]
    fn a_truncated_file_is_an_error_not_a_panic() {
        let data = sample_dsf();
        assert!(Dsf::parse(&data[..40]).is_err());
    }
}
