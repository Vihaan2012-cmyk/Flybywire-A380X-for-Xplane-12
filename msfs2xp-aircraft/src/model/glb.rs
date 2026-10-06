//! Reading the GLB models inside MSFS model libraries.
//!
//! They are glTF 2.0 binaries with Asobo's packing on top, which is why the
//! `gltf` crate's accessor readers cannot be used as they stand:
//!
//! - the JSON chunk is padded with NUL bytes, which strict JSON parsers reject;
//! - normals and tangents are signed bytes, and UVs and colours 16-bit values,
//!   all flagged `normalized: false` against the spec. Measured on 39 real
//!   models, the UVs and colours are IEEE half floats (they land in 0..1) and
//!   the normals are signed-normalised bytes (they come out unit length);
//! - image URIs are absolute paths on the author's build machine.
//!
//! So the JSON is read with serde and accessors are decoded here, by semantic.

use std::collections::HashMap;

use serde_json::Value;

#[derive(thiserror::Error, Debug)]
pub enum ModelError {
    #[error("not a GLB file")]
    NotGlb,
    #[error("GLB truncated: {0}")]
    Truncated(String),
    #[error("GLB JSON is invalid: {0}")]
    Json(String),
    #[error("GLB is malformed: {0}")]
    Malformed(String),
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vertex {
    pub pos: [f32; 3],
    pub normal: [f32; 3],
    pub uv: [f32; 2],
}

#[derive(Debug, Clone, Default)]
pub struct Mesh {
    pub name: String,
    /// Index into [`Model::materials`].
    pub material: usize,
    pub vertices: Vec<Vertex>,
    /// Triangle list, counter-clockwise front faces.
    pub indices: Vec<u32>,
    /// The node this geometry hangs from ([`Model::nodes`]): the mesh's own
    /// node, or for skinned meshes the joint weighing most on it. Animating
    /// that node or any ancestor moves the geometry.
    pub node: Option<usize>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AlphaMode {
    #[default]
    Opaque,
    Mask,
    Blend,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Material {
    pub name: String,
    /// Texture file names only, without the author's directory.
    pub base_color: Option<String>,
    pub normal: Option<String>,
    /// glTF metallic-roughness texture (MSFS "COMP": occlusion, roughness,
    /// metalness in red, green, blue).
    pub metal_rough: Option<String>,
    pub emissive: Option<String>,
    /// glTF `emissiveFactor`: black (all zero) when the material does not
    /// glow. Multiplies the emissive texture per channel, so a factor below
    /// white dims it and an uneven one tints it - FlyByWire's A380 pushbutton
    /// legends carry [0.78, 0.85, 1.00] (a faint blue) and its windshield
    /// frame glow [0.7, 0.7, 0.7] (dimmed), not the plain on/off gate this
    /// used to be collapsed to.
    pub emissive_factor: [f32; 3],
    pub base_color_factor: [f32; 4],
    pub alpha: AlphaMode,
    pub alpha_cutoff: f32,
    pub double_sided: bool,
    /// Asobo geometry decal: drawn slightly above the surface it sits on.
    pub decal: bool,
    /// MSFS's glass shader ("kitty glass"), whose see-through look comes
    /// from the shader, not from the material's opacity.
    pub glass: bool,
    /// MSFS UV tiling and offset (ASOBO_material_UV_options): the texture
    /// repeats `uv_tiling` times over the mesh's UV range.
    pub uv_tiling: [f32; 2],
    pub uv_offset: [f32; 2],
}

impl Default for Material {
    fn default() -> Self {
        Material {
            name: String::new(),
            base_color: None,
            normal: None,
            metal_rough: None,
            emissive: None,
            emissive_factor: [0.0; 3],
            base_color_factor: [1.0; 4],
            alpha: AlphaMode::Opaque,
            alpha_cutoff: 0.5,
            double_sided: false,
            decal: false,
            glass: false,
            uv_tiling: [1.0, 1.0],
            uv_offset: [0.0, 0.0],
        }
    }
}

/// A light MSFS attaches to a model node (`ASOBO_street_light`), in model
/// space: the beam runs along the node's +Z axis.
#[derive(Debug, Clone, PartialEq)]
pub struct LightPoint {
    pub pos: [f32; 3],
    /// Unit beam direction.
    pub dir: [f32; 3],
    pub color: [f32; 3],
    /// Candela.
    pub intensity: f32,
    /// Full cone angle in degrees.
    pub cone_deg: f32,
    /// Lights the ground (false for glare-only "flare" lights).
    pub spill: bool,
    /// Beacons and strobes; drawn as glare only.
    pub flashing: bool,
}

/// A scene node in its rest pose.
#[derive(Debug, Clone)]
pub struct Node {
    pub name: String,
    pub parent: Option<usize>,
    pub translation: [f64; 3],
    /// Unit quaternion, x y z w.
    pub rotation: [f64; 4],
    pub scale: [f64; 3],
    /// Rest world matrix, column-major.
    pub world: M4,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AnimPath {
    Translation,
    Rotation,
    Scale,
}

/// One animated property of one node.
#[derive(Debug, Clone)]
pub struct Channel {
    pub node: usize,
    pub path: AnimPath,
    /// Key times in seconds.
    pub times: Vec<f32>,
    /// One value per key: x y z (w unused) or a quaternion x y z w.
    pub values: Vec<[f32; 4]>,
    /// Hold each key until the next (glTF STEP) instead of interpolating.
    pub step: bool,
}

/// A glTF animation: MSFS drives each by one variable over its whole length.
#[derive(Debug, Clone)]
pub struct Clip {
    pub name: String,
    pub channels: Vec<Channel>,
}

#[derive(Debug, Clone, Default)]
pub struct Model {
    pub meshes: Vec<Mesh>,
    pub materials: Vec<Material>,
    pub warnings: Vec<String>,
    pub lights: Vec<LightPoint>,
    /// Every node of the file, for animation.
    pub nodes: Vec<Node>,
    pub clips: Vec<Clip>,
    /// Geometry MSFS never draws but clicks on (`ASOBO_material_invisible`,
    /// the cockpit's click spots): kept apart from `meshes`, for the click
    /// object only.
    pub click_spots: Vec<Mesh>,
}

impl Model {
    pub fn vertex_count(&self) -> usize {
        self.meshes.iter().map(|m| m.vertices.len()).sum()
    }

    pub fn triangle_count(&self) -> usize {
        self.meshes.iter().map(|m| m.indices.len() / 3).sum()
    }

    /// Axis-aligned bounds of every vertex, if there are any.
    pub fn bounds(&self) -> Option<([f32; 3], [f32; 3])> {
        let mut it = self.meshes.iter().flat_map(|m| m.vertices.iter());
        let first = it.next()?.pos;
        let (mut lo, mut hi) = (first, first);
        for v in it {
            for k in 0..3 {
                lo[k] = lo[k].min(v.pos[k]);
                hi[k] = hi[k].max(v.pos[k]);
            }
        }
        Some((lo, hi))
    }
}

fn le32(b: &[u8], at: usize) -> Option<u32> {
    b.get(at..at.checked_add(4)?)
        .map(|s| u32::from_le_bytes([s[0], s[1], s[2], s[3]]))
}

/// Split a GLB into its JSON and binary chunks.
fn glb_chunks(bytes: &[u8]) -> Result<(&[u8], &[u8]), ModelError> {
    if bytes.len() < 20 || &bytes[0..4] != b"glTF" {
        return Err(ModelError::NotGlb);
    }
    if le32(bytes, 4) != Some(2) {
        return Err(ModelError::Malformed("only glTF 2 is supported".into()));
    }
    let total = (le32(bytes, 8).unwrap_or(0) as usize).min(bytes.len());
    let (mut json, mut bin): (Option<&[u8]>, &[u8]) = (None, &[]);
    let mut pos = 12usize;
    while pos + 8 <= total {
        let len = le32(bytes, pos).unwrap_or(0) as usize;
        let kind = &bytes[pos + 4..pos + 8];
        let start = pos + 8;
        let end = start
            .checked_add(len)
            .filter(|&e| e <= total)
            .ok_or_else(|| ModelError::Truncated(format!("chunk at {pos} runs past the file")))?;
        match kind {
            b"JSON" => json = Some(&bytes[start..end]),
            b"BIN\0" => bin = &bytes[start..end],
            _ => {}
        }
        pos = (end + 3) & !3;
    }
    let json = json.ok_or_else(|| ModelError::Malformed("no JSON chunk".into()))?;
    Ok((json, bin))
}

/// IEEE 754 half precision to single precision.
pub fn f16_to_f32(h: u16) -> f32 {
    let sign = ((h >> 15) & 1) as u32;
    let exp = ((h >> 10) & 0x1F) as i32;
    let frac = (h & 0x3FF) as u32;
    let bits = match exp {
        0 if frac == 0 => sign << 31,
        0 => {
            // Subnormal: normalise the fraction.
            let mut e = -14i32;
            let mut f = frac;
            while f & 0x400 == 0 {
                f <<= 1;
                e -= 1;
            }
            (sign << 31) | (((e + 127) as u32) << 23) | ((f & 0x3FF) << 13)
        }
        0x1F => (sign << 31) | 0x7F80_0000 | (frac << 13),
        _ => (sign << 31) | (((exp - 15 + 127) as u32) << 23) | (frac << 13),
    };
    f32::from_bits(bits)
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Semantic {
    Position,
    Normal,
    TexCoord,
    Index,
    /// Anything else: floats as stored, normalised integers scaled.
    Other,
}

struct Accessor<'a> {
    bytes: &'a [u8],
    stride: usize,
    count: usize,
    comps: usize,
    ctype: u64,
    normalized: bool,
}

fn comp_size(ctype: u64) -> Option<usize> {
    match ctype {
        5120 | 5121 => Some(1),
        5122 | 5123 => Some(2),
        5125 | 5126 => Some(4),
        _ => None,
    }
}

fn comp_count(kind: &str) -> Option<usize> {
    match kind {
        "SCALAR" => Some(1),
        "VEC2" => Some(2),
        "VEC3" => Some(3),
        "VEC4" => Some(4),
        "MAT4" => Some(16),
        _ => None,
    }
}

fn accessor<'a>(json: &Value, bin: &'a [u8], idx: usize) -> Result<Accessor<'a>, ModelError> {
    let acc = &json["accessors"][idx];
    if acc.is_null() {
        return Err(ModelError::Malformed(format!("accessor {idx} missing")));
    }
    if !acc["sparse"].is_null() {
        return Err(ModelError::Malformed(format!("accessor {idx} is sparse")));
    }
    let ctype = acc["componentType"].as_u64().unwrap_or(0);
    let csize = comp_size(ctype).ok_or_else(|| ModelError::Malformed(format!("component type {ctype}")))?;
    let comps = comp_count(acc["type"].as_str().unwrap_or(""))
        .ok_or_else(|| ModelError::Malformed(format!("accessor {idx} type")))?;
    let count = acc["count"].as_u64().unwrap_or(0) as usize;
    let bv_idx = acc["bufferView"]
        .as_u64()
        .ok_or_else(|| ModelError::Malformed(format!("accessor {idx} has no buffer view")))? as usize;
    let bv = &json["bufferViews"][bv_idx];
    if bv["buffer"].as_u64().unwrap_or(0) != 0 {
        return Err(ModelError::Malformed("external buffers are not supported".into()));
    }
    let elem = csize * comps;
    let stride = bv["byteStride"]
        .as_u64()
        .map(|s| s as usize)
        .filter(|&s| s >= elem)
        .unwrap_or(elem);
    let view_start = bv["byteOffset"].as_u64().unwrap_or(0) as usize;
    let view_len = bv["byteLength"].as_u64().unwrap_or(0) as usize;
    let start = view_start + acc["byteOffset"].as_u64().unwrap_or(0) as usize;
    let end = if count == 0 {
        start
    } else {
        start + (count - 1) * stride + elem
    };
    if end > bin.len() || end > view_start + view_len {
        return Err(ModelError::Truncated(format!("accessor {idx} runs past its buffer")));
    }
    Ok(Accessor {
        bytes: &bin[start..end.max(start)],
        stride,
        count,
        comps,
        ctype,
        normalized: acc["normalized"].as_bool().unwrap_or(false),
    })
}

fn component(a: &Accessor, raw: &[u8], sem: Semantic) -> f32 {
    let half_like = !a.normalized && sem == Semantic::TexCoord;
    match a.ctype {
        5126 => f32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]),
        5120 => {
            let v = raw[0] as i8 as f32;
            if a.normalized || sem == Semantic::Normal {
                (v / 127.0).max(-1.0)
            } else {
                v
            }
        }
        5121 => {
            let v = raw[0] as f32;
            if a.normalized || sem == Semantic::TexCoord {
                v / 255.0
            } else {
                v
            }
        }
        5122 => {
            let bits = u16::from_le_bytes([raw[0], raw[1]]);
            if half_like {
                f16_to_f32(bits)
            } else if a.normalized || sem == Semantic::Normal {
                (bits as i16 as f32 / 32767.0).max(-1.0)
            } else {
                bits as i16 as f32
            }
        }
        5123 => {
            let bits = u16::from_le_bytes([raw[0], raw[1]]);
            if half_like {
                f16_to_f32(bits)
            } else if a.normalized {
                bits as f32 / 65535.0
            } else {
                bits as f32
            }
        }
        5125 => u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]) as f32,
        _ => 0.0,
    }
}

/// Decode an accessor to `count` rows of `N` floats (extra components dropped,
/// missing ones zero).
fn read_rows<const N: usize>(a: &Accessor, sem: Semantic) -> Vec<[f32; N]> {
    let csize = comp_size(a.ctype).unwrap_or(4);
    (0..a.count)
        .map(|i| {
            let mut row = [0.0f32; N];
            for (k, slot) in row.iter_mut().enumerate().take(a.comps.min(N)) {
                let at = i * a.stride + k * csize;
                if let Some(raw) = a.bytes.get(at..at + csize) {
                    *slot = component(a, raw, sem);
                }
            }
            row
        })
        .collect()
}

fn read_indices(a: &Accessor) -> Vec<u32> {
    let csize = comp_size(a.ctype).unwrap_or(4);
    (0..a.count)
        .filter_map(|i| {
            let raw = a.bytes.get(i * a.stride..i * a.stride + csize)?;
            Some(match a.ctype {
                5121 => raw[0] as u32,
                5123 => u16::from_le_bytes([raw[0], raw[1]]) as u32,
                5125 => u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]),
                _ => component(a, raw, Semantic::Index) as u32,
            })
        })
        .collect()
}

/// Column-major 4x4 matrix, as glTF stores it.
pub type M4 = [f64; 16];

pub const IDENTITY: M4 = [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.];

pub fn mul(a: &M4, b: &M4) -> M4 {
    let mut out = [0.0; 16];
    for c in 0..4 {
        for r in 0..4 {
            out[c * 4 + r] = (0..4).map(|k| a[k * 4 + r] * b[c * 4 + k]).sum();
        }
    }
    out
}

fn floats(v: &Value, n: usize) -> Option<Vec<f64>> {
    let arr = v.as_array()?;
    (arr.len() == n).then(|| arr.iter().map(|x| x.as_f64().unwrap_or(0.0)).collect())
}

fn node_matrix(node: &Value) -> M4 {
    if let Some(m) = floats(&node["matrix"], 16) {
        let mut out = [0.0; 16];
        out.copy_from_slice(&m);
        return out;
    }
    let t = floats(&node["translation"], 3).unwrap_or(vec![0.0; 3]);
    let q = floats(&node["rotation"], 4).unwrap_or(vec![0.0, 0.0, 0.0, 1.0]);
    let s = floats(&node["scale"], 3).unwrap_or(vec![1.0; 3]);
    trs_matrix([t[0], t[1], t[2]], [q[0], q[1], q[2], q[3]], [s[0], s[1], s[2]])
}

/// Translation * rotation (unit quaternion x y z w) * scale, column-major.
pub fn trs_matrix(t: [f64; 3], q: [f64; 4], s: [f64; 3]) -> M4 {
    let (x, y, z, w) = (q[0], q[1], q[2], q[3]);
    // Rotation matrix from a unit quaternion, scaled per axis.
    let r = [
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y + z * w),
        2.0 * (x * z - y * w),
        2.0 * (x * y - z * w),
        1.0 - 2.0 * (x * x + z * z),
        2.0 * (y * z + x * w),
        2.0 * (x * z + y * w),
        2.0 * (y * z - x * w),
        1.0 - 2.0 * (x * x + y * y),
    ];
    [
        r[0] * s[0],
        r[1] * s[0],
        r[2] * s[0],
        0.0,
        r[3] * s[1],
        r[4] * s[1],
        r[5] * s[1],
        0.0,
        r[6] * s[2],
        r[7] * s[2],
        r[8] * s[2],
        0.0,
        t[0],
        t[1],
        t[2],
        1.0,
    ]
}

fn transform_point(m: &M4, p: [f32; 3]) -> [f32; 3] {
    let (x, y, z) = (p[0] as f64, p[1] as f64, p[2] as f64);
    [
        (m[0] * x + m[4] * y + m[8] * z + m[12]) as f32,
        (m[1] * x + m[5] * y + m[9] * z + m[13]) as f32,
        (m[2] * x + m[6] * y + m[10] * z + m[14]) as f32,
    ]
}

/// The matrix for normals: the inverse transpose of the upper 3x3, so that
/// non-uniform scale does not skew them. Also returns the determinant, whose
/// sign says whether the transform mirrors geometry.
fn normal_matrix(m: &M4) -> ([f64; 9], f64) {
    let a = [m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]]; // columns
    let (a00, a10, a20, a01, a11, a21, a02, a12, a22) = (a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8]);
    let det = a00 * (a11 * a22 - a12 * a21) - a01 * (a10 * a22 - a12 * a20) + a02 * (a10 * a21 - a11 * a20);
    if det.abs() < 1e-12 {
        return ([a00, a10, a20, a01, a11, a21, a02, a12, a22], det);
    }
    // Cofactor matrix divided by the determinant is the inverse transpose.
    let inv_t = [
        (a11 * a22 - a12 * a21) / det,
        -(a01 * a22 - a02 * a21) / det,
        (a01 * a12 - a02 * a11) / det,
        -(a10 * a22 - a12 * a20) / det,
        (a00 * a22 - a02 * a20) / det,
        -(a00 * a12 - a02 * a10) / det,
        (a10 * a21 - a11 * a20) / det,
        -(a00 * a21 - a01 * a20) / det,
        (a00 * a11 - a01 * a10) / det,
    ];
    // Stored row-major: row r = (inv_t[r*3], inv_t[r*3+1], inv_t[r*3+2]) applied to (x,y,z).
    (
        [
            inv_t[0], inv_t[3], inv_t[6], inv_t[1], inv_t[4], inv_t[7], inv_t[2], inv_t[5], inv_t[8],
        ],
        det,
    )
}

fn transform_normal(nm: &[f64; 9], n: [f32; 3]) -> [f32; 3] {
    let (x, y, z) = (n[0] as f64, n[1] as f64, n[2] as f64);
    let v = [
        nm[0] * x + nm[1] * y + nm[2] * z,
        nm[3] * x + nm[4] * y + nm[5] * z,
        nm[6] * x + nm[7] * y + nm[8] * z,
    ];
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len < 1e-12 {
        [0.0, 1.0, 0.0]
    } else {
        [(v[0] / len) as f32, (v[1] / len) as f32, (v[2] / len) as f32]
    }
}

fn basename(uri: &str) -> String {
    uri.rsplit(['/', '\\']).next().unwrap_or(uri).to_string()
}

fn texture_file(json: &Value, texture_index: Option<u64>) -> Option<String> {
    let tex = &json["textures"][texture_index? as usize];
    let source = tex["extensions"]["MSFT_texture_dds"]["source"]
        .as_u64()
        .or_else(|| tex["source"].as_u64())?;
    json["images"][source as usize]["uri"].as_str().map(basename)
}

/// Whether another material in the same document shares this base colour
/// texture and is unambiguously a decal (glTF `BLEND` with Asobo's own decal
/// extension). MSFS aircraft pack lettering and placards for a whole panel
/// into one "decal atlas" texture that several materials share; normally
/// each of them carries `alphaMode: BLEND` plus `ASOBO_material_geometry_decal`
/// or `ASOBO_material_blend_gbuffer`, which the match below already turns
/// into [`AlphaMode::Blend`]. FlyByWire's A380 ships one material on that
/// atlas without either marker (`A380_COCKPIT_MISC02_LED_GREEN`, a doubleSided
/// OPAQUE material with no alphaMode and no ASOBO extension at all, used for
/// a lit RMP panel indicator) - so it fell through to OPAQUE and drew the
/// dilated, transparent-background colour the decal treatment cuts away for
/// its siblings as a solid box. A sibling material on the very same texture
/// that IS already marked as a decal is strong, narrow evidence this one
/// needs the same treatment: it only fires when there is concrete proof in
/// this document, not a guess from the texture's file name.
///
/// Compared by the resolved file name, not the glTF `baseColorTexture`
/// index: MSFS's exporter mints a fresh `textures[]` entry per material even
/// when several of them reuse one image (FlyByWire's own decal atlas is
/// image 32, reached through five different texture indices), so comparing
/// indices never matched at all.
fn shares_decal_texture(json: &Value, base_color_file: Option<&str>) -> bool {
    let Some(file) = base_color_file else { return false };
    json["materials"].as_array().into_iter().flatten().any(|sib| {
        let sib_ext = &sib["extensions"];
        sib["alphaMode"].as_str() == Some("BLEND")
            && (!sib_ext["ASOBO_material_geometry_decal"].is_null() || !sib_ext["ASOBO_material_blend_gbuffer"].is_null())
            && texture_file(json, sib["pbrMetallicRoughness"]["baseColorTexture"]["index"].as_u64()).as_deref() == Some(file)
    })
}

fn material(json: &Value, m: &Value) -> (Material, bool) {
    let pbr = &m["pbrMetallicRoughness"];
    let mut factor = floats(&pbr["baseColorFactor"], 4)
        .map(|v| [v[0] as f32, v[1] as f32, v[2] as f32, v[3] as f32])
        .unwrap_or([1.0; 4]);
    let ext = &m["extensions"];
    let name = m["name"].as_str().unwrap_or_default().to_string();
    let decal_ext = !ext["ASOBO_material_geometry_decal"].is_null() || !ext["ASOBO_material_blend_gbuffer"].is_null();
    let base_color = texture_file(json, pbr["baseColorTexture"]["index"].as_u64());
    // Only promotes a material that has neither an explicit MASK/BLEND
    // alphaMode nor the decal extension of its own: a masked cutout (its own
    // cutoff, tested in the ordinary opaque pass) or a real blend (glass) on
    // the same shared texture is not evidence it needs the decal treatment.
    let shared_decal = !decal_ext
        && !matches!(m["alphaMode"].as_str(), Some("MASK") | Some("BLEND"))
        && shares_decal_texture(json, base_color.as_deref());
    // Hidden in MSFS: invisible materials, and geometry decals that blend no
    // colour (they only change the surface's normals or roughness, so their
    // texture would show here as stray patches).
    // Decals blending only part of their colour into the surface below
    // (rivet and grime ribbons at 13%) fade by that part; at none they only
    // change its shading, so they show nothing here.
    // Absent (as the exporter leaves every factor it was never set away
    // from its own default) means full strength, 1.0: the extension's own
    // schema example gives `"baseColorBlendFactor": 1`
    // (docs.flightsimulator.com, "glTF Schemas", ASOBO_material_blend_gbuffer),
    // and this converter's own decal atlas has plenty of `{}` materials
    // (every factor default) that must still draw at full colour.
    let blend = ext["ASOBO_material_blend_gbuffer"]["baseColorBlendFactor"]
        .as_f64()
        .or(ext["ASOBO_material_geometry_decal"]["baseColorBlendFactor"].as_f64());
    let colourless = blend == Some(0.0);
    // Carried into the factor's own alpha channel: `obj8::split_by_texture_lit`
    // reads it back out of `base_color_factor[3]` as `TextureKey::blend_alpha`
    // for a blended (non-glass) decal, and `TextureKey::alpha` for glass.
    factor[3] *= blend.unwrap_or(1.0).clamp(0.0, 1.0) as f32;
    // Glass with no texture and next to no opacity (MSFS rain layers) shows
    // nothing without MSFS's glass shader; drawn here it would be a white sheet.
    let clear = m["alphaMode"].as_str() == Some("BLEND") && pbr["baseColorTexture"].is_null() && factor[3] <= 0.05;
    // Environment occluders are shells MSFS only uses to keep rain and sky
    // light out of the cockpit; drawn, they wall the windscreen off in grey.
    let occluder = !ext["ASOBO_material_environment_occluder"].is_null();
    // A validator's own placeholder for a material reference broken before
    // the file ever reached this converter: FlyByWire's A380 has exactly
    // one such material in each of the cockpit and exterior files, named
    // "glTFValidator_Added_Default_Material" and otherwise completely
    // empty (no pbrMetallicRoughness, no extensions, no alphaMode - just a
    // name), used by a handful of passenger-door sub-parts whose real
    // material a glTF validator could not resolve and patched over rather
    // than leave the file invalid. There is no colour, texture or factor
    // here to translate honestly: the placeholder carries none of the
    // original material's information, only glTF's bare spec defaults
    // (an implicit baseColorFactor of white), so drawing it as a flat white
    // slab is not a faithful rendering of anything - it is a hole in the
    // source data rendered literally. Detected structurally (every key but
    // `name` absent), not by name, so a different validator's own
    // placeholder convention is still caught.
    let broken_reference = m.as_object().is_some_and(|o| o.keys().all(|k| k == "name"));
    let invisible = !ext["ASOBO_material_invisible"].is_null() || colourless || clear || occluder || broken_reference;
    let uv = &ext["ASOBO_material_UV_options"];
    let uv_num = |k: &str, default: f64| uv[k].as_f64().unwrap_or(default) as f32;
    (
        Material {
            base_color,
            normal: texture_file(json, m["normalTexture"]["index"].as_u64()),
            metal_rough: texture_file(json, pbr["metallicRoughnessTexture"]["index"].as_u64()),
            emissive: texture_file(json, m["emissiveTexture"]["index"].as_u64()),
            // The full factor, not just its strongest channel: glTF defines
            // it as a per-channel multiplier on the emissive texture, and
            // MSFS uses it for colour (a blue-tinted pushbutton legend) as
            // well as intensity, not only as an on/off gate.
            emissive_factor: floats(&m["emissiveFactor"], 3)
                .map(|v| [v[0] as f32, v[1] as f32, v[2] as f32])
                .unwrap_or([0.0; 3]),
            base_color_factor: factor,
            // MSFS decals blend by their texture's alpha whatever glTF's
            // alphaMode says: panel lettering exported OPAQUE drew its
            // transparent surround as a solid box. `shared_decal` catches
            // the same failure one step removed, where the decal markers
            // were left off a material that still shares the decal atlas
            // with a properly marked sibling (see `shares_decal_texture`).
            alpha: match m["alphaMode"].as_str() {
                Some("MASK") => AlphaMode::Mask,
                Some("BLEND") => AlphaMode::Blend,
                _ if decal_ext && !pbr["baseColorTexture"].is_null() => AlphaMode::Blend,
                _ if shared_decal => AlphaMode::Blend,
                _ => AlphaMode::Opaque,
            },
            alpha_cutoff: m["alphaCutoff"].as_f64().unwrap_or(0.5) as f32,
            double_sided: m["doubleSided"].as_bool().unwrap_or(false),
            // Blend-gbuffer decals (panel lettering, rivet ribbons) sit a
            // hair above their surface too; without an offset they z-fight.
            // A material promoted by `shared_decal` gets lifted the same
            // way: once it is alpha-tested it no longer fully covers the
            // surface underneath it either.
            decal: decal_ext || shared_decal,
            // MSFS's glass shader, or blended light covers and bulbs
            // ("..._LIGHT_GLASS", "..._ACCESSORIES_BLEND"), which without
            // that shader read as dark sheets over the wings. Fan and RAT
            // blur discs keep their own opacity.
            glass: !ext["ASOBO_material_kitty_glass"].is_null() || {
                let n = name.to_ascii_uppercase();
                (n.contains("GLASS") || n.ends_with("_BLEND")) && !n.contains("BLUR")
            },
            uv_tiling: [uv_num("UVTilingU", 1.0), uv_num("UVTilingV", 1.0)],
            uv_offset: [uv_num("UVOffsetU", 0.0), uv_num("UVOffsetV", 0.0)],
            name,
        },
        invisible,
    )
}

/// Flat per-vertex normals for primitives that ship without any.
fn generate_normals(verts: &mut [Vertex], indices: &[u32]) {
    let mut acc = vec![[0.0f32; 3]; verts.len()];
    for t in indices.chunks_exact(3) {
        let (Some(a), Some(b), Some(c)) = (
            verts.get(t[0] as usize),
            verts.get(t[1] as usize),
            verts.get(t[2] as usize),
        ) else {
            continue;
        };
        let u = [b.pos[0] - a.pos[0], b.pos[1] - a.pos[1], b.pos[2] - a.pos[2]];
        let v = [c.pos[0] - a.pos[0], c.pos[1] - a.pos[1], c.pos[2] - a.pos[2]];
        let n = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        for &i in t {
            for k in 0..3 {
                acc[i as usize][k] += n[k];
            }
        }
    }
    for (v, n) in verts.iter_mut().zip(acc) {
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        v.normal = if len > 1e-12 {
            [n[0] / len, n[1] / len, n[2] / len]
        } else {
            [0.0, 1.0, 0.0]
        };
    }
}

fn normalize3(v: [f32; 3]) -> [f32; 3] {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len > 1e-12 {
        [v[0] / len, v[1] / len, v[2] / len]
    } else {
        [0.0, 1.0, 0.0]
    }
}

/// Every node with its rest transform, parent and world matrix.
fn parse_nodes(json: &Value) -> Vec<Node> {
    let arr = json["nodes"].as_array().map(Vec::as_slice).unwrap_or(&[]);
    let n = arr.len();
    let mut parent = vec![None; n];
    for (i, node) in arr.iter().enumerate() {
        for c in node["children"].as_array().into_iter().flatten().filter_map(Value::as_u64) {
            if (c as usize) < n && c as usize != i {
                parent[c as usize] = Some(i);
            }
        }
    }
    let mut out: Vec<Node> = arr
        .iter()
        .enumerate()
        .map(|(i, node)| {
            let (t, r, s) = match floats(&node["matrix"], 16) {
                Some(m) => {
                    let mut mm = [0.0; 16];
                    mm.copy_from_slice(&m);
                    let len = |c: usize| (mm[c * 4].powi(2) + mm[c * 4 + 1].powi(2) + mm[c * 4 + 2].powi(2)).sqrt();
                    ([mm[12], mm[13], mm[14]], super::anim::quat_of(&mm), [len(0), len(1), len(2)])
                }
                None => {
                    let t = floats(&node["translation"], 3).unwrap_or(vec![0.0; 3]);
                    let q = floats(&node["rotation"], 4).unwrap_or(vec![0.0, 0.0, 0.0, 1.0]);
                    let s = floats(&node["scale"], 3).unwrap_or(vec![1.0; 3]);
                    ([t[0], t[1], t[2]], [q[0], q[1], q[2], q[3]], [s[0], s[1], s[2]])
                }
            };
            Node {
                name: node["name"].as_str().unwrap_or_default().to_string(),
                parent: parent[i],
                translation: t,
                rotation: r,
                scale: s,
                world: node_matrix(node),
            }
        })
        .collect();
    // World matrices: each node's local matrix under its parent's world.
    let mut done = vec![false; n];
    for i in 0..n {
        let mut chain = vec![i];
        while let Some(p) = out[*chain.last().unwrap()].parent {
            if done[p] || chain.len() > 256 || chain.contains(&p) {
                break;
            }
            chain.push(p);
        }
        for &k in chain.iter().rev() {
            if done[k] {
                continue;
            }
            if let Some(p) = out[k].parent.filter(|&p| done[p]) {
                out[k].world = mul(&out[p].world, &out[k].world);
            }
            done[k] = true;
        }
    }
    out
}

/// Every animation's translation, rotation and scale channels.
fn parse_clips(json: &Value, bin: &[u8], warnings: &mut Vec<String>) -> Vec<Clip> {
    let mut out = Vec::new();
    for (ai, a) in json["animations"].as_array().into_iter().flatten().enumerate() {
        let name = a["name"].as_str().map(str::to_string).unwrap_or_else(|| format!("animation_{ai}"));
        let mut channels = Vec::new();
        for ch in a["channels"].as_array().into_iter().flatten() {
            let (Some(node), Some(path)) = (ch["target"]["node"].as_u64(), ch["target"]["path"].as_str()) else {
                continue;
            };
            let path = match path {
                "translation" => AnimPath::Translation,
                "rotation" => AnimPath::Rotation,
                "scale" => AnimPath::Scale,
                _ => continue,
            };
            let Some(sampler) = ch["sampler"].as_u64().and_then(|s| a["samplers"].get(s as usize)) else {
                continue;
            };
            let (Some(i), Some(o)) = (sampler["input"].as_u64(), sampler["output"].as_u64()) else {
                continue;
            };
            let (Ok(ia), Ok(oa)) = (accessor(json, bin, i as usize), accessor(json, bin, o as usize)) else {
                warnings.push(format!("{name}: unreadable animation channel skipped"));
                continue;
            };
            let times: Vec<f32> = read_rows::<1>(&ia, Semantic::Other).into_iter().map(|r| r[0]).collect();
            let mut values: Vec<[f32; 4]> = read_rows::<4>(&oa, Semantic::Other);
            let interp = sampler["interpolation"].as_str().unwrap_or("LINEAR");
            if interp == "CUBICSPLINE" && values.len() == times.len() * 3 {
                values = values.chunks_exact(3).map(|c| c[1]).collect();
            }
            if times.is_empty() || values.len() != times.len() {
                continue;
            }
            channels.push(Channel {
                node: node as usize,
                path,
                times,
                values,
                step: interp == "STEP",
            });
        }
        out.push(Clip { name, channels });
    }
    out
}

/// One skin joint in the rest pose: its skinning matrix (joint world times
/// inverse bind), that matrix for normals, its determinant and its node.
struct JointMat {
    m: M4,
    nm: [f64; 9],
    det: f64,
    node: usize,
}

/// Load a GLB from memory, flattening its node hierarchy into world space.
pub fn load_glb(bytes: &[u8]) -> Result<Model, ModelError> {
    let (json_chunk, bin) = glb_chunks(bytes)?;
    let end = json_chunk
        .iter()
        .rposition(|&c| c != 0 && c != b' ')
        .map_or(0, |p| p + 1);
    let json: Value = serde_json::from_slice(&json_chunk[..end]).map_err(|e| ModelError::Json(e.to_string()))?;

    let mut model = Model::default();
    let mut invisible = Vec::new();
    let mut click_only = Vec::new();
    for m in json["materials"].as_array().map(Vec::as_slice).unwrap_or(&[]) {
        let (mat, hidden) = material(&json, m);
        model.materials.push(mat);
        let click = !m["extensions"]["ASOBO_material_invisible"].is_null();
        invisible.push(hidden && !click);
        click_only.push(click);
    }
    // Primitives without a material get a default one at the end.
    let default_material = model.materials.len();
    model.materials.push(Material {
        name: "default".into(),
        ..Default::default()
    });
    invisible.push(false);
    click_only.push(false);

    model.nodes = parse_nodes(&json);
    model.clips = parse_clips(&json, bin, &mut model.warnings);
    let skins: Vec<Vec<JointMat>> = json["skins"]
        .as_array()
        .map(|arr| {
            arr.iter()
                .map(|s| {
                    let ibm: Vec<[f32; 16]> = s["inverseBindMatrices"]
                        .as_u64()
                        .and_then(|i| accessor(&json, bin, i as usize).ok())
                        .map(|a| read_rows::<16>(&a, Semantic::Other))
                        .unwrap_or_default();
                    s["joints"]
                        .as_array()
                        .into_iter()
                        .flatten()
                        .filter_map(Value::as_u64)
                        .enumerate()
                        .map(|(k, jn)| {
                            let jn = jn as usize;
                            let jw = model.nodes.get(jn).map_or(IDENTITY, |n| n.world);
                            let ib: M4 = ibm.get(k).map_or(IDENTITY, |m| m.map(|x| x as f64));
                            let m = mul(&jw, &ib);
                            let (nm, det) = normal_matrix(&m);
                            JointMat { m, nm, det, node: jn }
                        })
                        .collect()
                })
                .collect()
        })
        .unwrap_or_default();

    let scene = json["scene"].as_u64().unwrap_or(0) as usize;
    let roots: Vec<usize> = match json["scenes"][scene]["nodes"].as_array() {
        Some(nodes) => nodes.iter().filter_map(|n| n.as_u64().map(|v| v as usize)).collect(),
        None => (0..json["nodes"].as_array().map_or(0, Vec::len)).collect(),
    };

    let mut stack: Vec<(usize, M4, usize)> = roots.into_iter().rev().map(|n| (n, IDENTITY, 0)).collect();
    let mut hidden_meshes = 0usize;
    while let Some((idx, parent, depth)) = stack.pop() {
        if depth > 64 {
            model
                .warnings
                .push("node hierarchy deeper than 64 levels; truncated".into());
            continue;
        }
        let node = &json["nodes"][idx];
        if node.is_null() {
            continue;
        }
        let world = mul(&parent, &node_matrix(node));
        if let Some(children) = node["children"].as_array() {
            for c in children.iter().rev().filter_map(Value::as_u64) {
                stack.push((c as usize, world, depth + 1));
            }
        }
        if let Some(sl) = node["extensions"]["ASOBO_street_light"].as_object() {
            let num = |k: &str, d: f64| sl.get(k).and_then(Value::as_f64).unwrap_or(d);
            let pos = transform_point(&world, [0.0, 0.0, 0.0]);
            let tip = transform_point(&world, [0.0, 0.0, 1.0]);
            let mut dir = [tip[0] - pos[0], tip[1] - pos[1], tip[2] - pos[2]];
            let len = (dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]).sqrt();
            if len > 1e-6 {
                for c in &mut dir {
                    *c /= len;
                }
            }
            let color = sl
                .get("color")
                .and_then(Value::as_array)
                .map(|c| {
                    let g = |i: usize| c.get(i).and_then(Value::as_f64).unwrap_or(1.0) as f32;
                    [g(0), g(1), g(2)]
                })
                .unwrap_or([1.0; 3]);
            model.lights.push(LightPoint {
                pos,
                dir,
                color,
                intensity: num("intensity", 1000.0) as f32,
                cone_deg: num("cone_angle", 120.0) as f32,
                spill: !sl.get("flare_only").and_then(Value::as_bool).unwrap_or(false),
                flashing: num("flash_frequency", 0.0) > 0.0,
            });
        }
        let Some(mesh_idx) = node["mesh"].as_u64() else {
            continue;
        };
        let mesh = &json["meshes"][mesh_idx as usize];
        let (nm, det) = normal_matrix(&world);
        let mesh_name = mesh["name"].as_str().or(node["name"].as_str()).unwrap_or_default();
        for prim in mesh["primitives"].as_array().map(Vec::as_slice).unwrap_or(&[]) {
            if prim["mode"].as_u64().unwrap_or(4) != 4 {
                model
                    .warnings
                    .push(format!("{mesh_name}: non-triangle primitive skipped"));
                continue;
            }
            let material = prim["material"]
                .as_u64()
                .map(|m| m as usize)
                .filter(|&m| m < default_material);
            let material = material.unwrap_or(default_material);
            if invisible[material] {
                hidden_meshes += 1;
                continue;
            }
            let attrs = &prim["attributes"];
            let Some(pos_idx) = attrs["POSITION"].as_u64() else {
                model
                    .warnings
                    .push(format!("{mesh_name}: primitive without positions skipped"));
                continue;
            };
            let positions: Vec<[f32; 3]> = match accessor(&json, bin, pos_idx as usize) {
                Ok(a) => read_rows(&a, Semantic::Position),
                Err(e) => {
                    model.warnings.push(format!("{mesh_name}: {e}"));
                    continue;
                }
            };
            let normals: Option<Vec<[f32; 3]>> = attrs["NORMAL"]
                .as_u64()
                .and_then(|i| accessor(&json, bin, i as usize).ok())
                .map(|a| read_rows(&a, Semantic::Normal));
            let uvs: Option<Vec<[f32; 2]>> = attrs["TEXCOORD_0"]
                .as_u64()
                .and_then(|i| accessor(&json, bin, i as usize).ok())
                .map(|a| read_rows(&a, Semantic::TexCoord));
            let mut indices: Vec<u32> = match prim["indices"].as_u64() {
                Some(i) => match accessor(&json, bin, i as usize) {
                    Ok(a) => read_indices(&a),
                    Err(e) => {
                        model.warnings.push(format!("{mesh_name}: {e}"));
                        continue;
                    }
                },
                None => (0..positions.len() as u32).collect(),
            };
            // MSFS packs many primitives into one vertex pool and one index
            // list; each primitive's share is given only in
            // extras.ASOBO_primitive. Without it every primitive would read the
            // whole list and join vertices of unrelated parts.
            if let Some(ap) = prim["extras"]["ASOBO_primitive"].as_object() {
                let start = ap.get("StartIndex").and_then(Value::as_u64).unwrap_or(0) as usize;
                let base = ap.get("BaseVertexIndex").and_then(Value::as_u64).unwrap_or(0) as u32;
                let end = match ap.get("PrimitiveCount").and_then(Value::as_u64) {
                    Some(n) => start.saturating_add(n as usize * 3),
                    None => indices.len(),
                };
                if start > end || end > indices.len() {
                    model
                        .warnings
                        .push(format!("{mesh_name}: primitive range outside its index list; skipped"));
                    continue;
                }
                indices = indices[start..end].iter().map(|&i| i.saturating_add(base)).collect();
            }
            indices.truncate(indices.len() / 3 * 3);
            if indices.iter().any(|&i| i as usize >= positions.len()) {
                model
                    .warnings
                    .push(format!("{mesh_name}: index out of range; primitive skipped"));
                continue;
            }
            // Keep only the vertices this primitive uses, renumbered in order
            // of first use.
            let mut remap = vec![u32::MAX; positions.len()];
            let mut used: Vec<usize> = Vec::new();
            for i in indices.iter_mut() {
                let old = *i as usize;
                if remap[old] == u32::MAX {
                    remap[old] = used.len() as u32;
                    used.push(old);
                }
                *i = remap[old];
            }
            let (tile, shift) = (model.materials[material].uv_tiling, model.materials[material].uv_offset);
            let uv_at = |i: usize| {
                uvs.as_ref()
                    .and_then(|u| u.get(i))
                    .map(|u| [u[0] * tile[0] + shift[0], u[1] * tile[1] + shift[1]])
                    .unwrap_or([0.0, 0.0])
            };
            let rigid = |i: usize| Vertex {
                pos: transform_point(&world, positions[i]),
                normal: normals
                    .as_ref()
                    .and_then(|n| n.get(i))
                    .map(|n| transform_normal(&nm, *n))
                    .unwrap_or([0.0, 1.0, 0.0]),
                uv: uv_at(i),
            };
            // Skinned meshes are posed by their joints (glTF ignores the mesh
            // node's own transform). Each triangle then hangs from the joint
            // weighing most on it, so bone animation moves it rigidly.
            let skin = node["skin"].as_u64().and_then(|s| skins.get(s as usize)).filter(|s| !s.is_empty());
            let read4 = |k: &str| -> Option<Vec<[f32; 4]>> {
                attrs[k]
                    .as_u64()
                    .and_then(|i| accessor(&json, bin, i as usize).ok())
                    .map(|a| read_rows(&a, Semantic::Other))
            };
            let (joints, weights) = match skin {
                Some(_) => (read4("JOINTS_0"), read4("WEIGHTS_0")),
                None => (None, None),
            };
            let mut vertices: Vec<Vertex> = Vec::with_capacity(used.len());
            // Per vertex, the joint slots and weights posing it (skinned only).
            let mut influences: Vec<Vec<(usize, f32)>> = Vec::new();
            if let (Some(sk), Some(js), Some(ws)) = (skin, &joints, &weights) {
                for &i in &used {
                    let (j, w) = (js.get(i).copied().unwrap_or_default(), ws.get(i).copied().unwrap_or_default());
                    let infl: Vec<(usize, f32)> = (0..4)
                        .filter(|&k| w[k] > 0.0 && (j[k] as usize) < sk.len())
                        .map(|k| (j[k] as usize, w[k]))
                        .collect();
                    let sum: f32 = infl.iter().map(|x| x.1).sum();
                    if sum <= 1e-6 {
                        vertices.push(rigid(i));
                        influences.push(Vec::new());
                        continue;
                    }
                    let (p, n) = (positions[i], normals.as_ref().and_then(|n| n.get(i)).copied());
                    let (mut pos, mut nor) = ([0.0f32; 3], [0.0f32; 3]);
                    for &(s, wt) in &infl {
                        let wt = wt / sum;
                        let q = transform_point(&sk[s].m, p);
                        let t = n.map(|n| transform_normal(&sk[s].nm, n)).unwrap_or([0.0; 3]);
                        for c in 0..3 {
                            pos[c] += q[c] * wt;
                            nor[c] += t[c] * wt;
                        }
                    }
                    vertices.push(Vertex {
                        pos,
                        normal: normalize3(nor),
                        uv: uv_at(i),
                    });
                    influences.push(infl);
                }
            } else {
                vertices.extend(used.iter().map(|&i| rigid(i)));
            }
            // Each triangle's node, and whether its transform mirrors it (a
            // mirror flips which side faces front).
            let tri_info: Vec<(usize, bool)> = indices
                .chunks_exact(3)
                .map(|t| {
                    let Some(sk) = skin.filter(|_| !influences.is_empty()) else {
                        return (idx, det < 0.0);
                    };
                    let mut acc: Vec<(usize, f32)> = Vec::new();
                    for &v in t {
                        for &(s, w) in &influences[v as usize] {
                            match acc.iter_mut().find(|a| a.0 == s) {
                                Some(a) => a.1 += w,
                                None => acc.push((s, w)),
                            }
                        }
                    }
                    match acc.iter().max_by(|a, b| a.1.total_cmp(&b.1)) {
                        Some(&(s, _)) => (sk[s].node, sk[s].det < 0.0),
                        None => (idx, det < 0.0),
                    }
                })
                .collect();
            let mut groups: Vec<(usize, Vec<u32>)> = Vec::new();
            for (t, &(own, flip)) in indices.chunks_exact(3).zip(&tri_info) {
                let tri = if flip { [t[0], t[2], t[1]] } else { [t[0], t[1], t[2]] };
                match groups.iter_mut().find(|g| g.0 == own) {
                    Some(g) => g.1.extend_from_slice(&tri),
                    None => groups.push((own, tri.to_vec())),
                }
            }
            if groups.len() == 1 {
                let (own, idxs) = groups.pop().unwrap_or_default();
                if normals.is_none() {
                    generate_normals(&mut vertices, &idxs);
                }
                model.meshes.push(Mesh {
                    name: mesh_name.to_string(),
                    material,
                    vertices,
                    indices: idxs,
                    node: Some(own),
                });
            } else {
                for (own, idxs) in groups {
                    let mut map: HashMap<u32, u32> = HashMap::new();
                    let mut verts: Vec<Vertex> = Vec::new();
                    let local: Vec<u32> = idxs
                        .iter()
                        .map(|&i| {
                            *map.entry(i).or_insert_with(|| {
                                verts.push(vertices[i as usize]);
                                verts.len() as u32 - 1
                            })
                        })
                        .collect();
                    if normals.is_none() {
                        generate_normals(&mut verts, &local);
                    }
                    model.meshes.push(Mesh {
                        name: mesh_name.to_string(),
                        material,
                        vertices: verts,
                        indices: local,
                        node: Some(own),
                    });
                }
            }
        }
    }
    if hidden_meshes > 0 {
        model
            .warnings
            .push(format!("{hidden_meshes} invisible primitive(s) skipped"));
    }
    let (spots, meshes): (Vec<Mesh>, Vec<Mesh>) = std::mem::take(&mut model.meshes).into_iter().partition(|m| click_only.get(m.material).copied().unwrap_or(false));
    model.meshes = meshes;
    model.click_spots = spots;
    Ok(model)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    fn f32_to_f16(x: f32) -> u16 {
        // Enough for test values: normal numbers only.
        let b = x.to_bits();
        let sign = ((b >> 16) & 0x8000) as u16;
        let exp = ((b >> 23) & 0xFF) as i32 - 127 + 15;
        let frac = ((b >> 13) & 0x3FF) as u16;
        if x == 0.0 {
            sign
        } else {
            sign | ((exp as u16) << 10) | frac
        }
    }

    /// A GLB with one triangle in Asobo's packing: f32 positions, i8 normals,
    /// half-float UVs, u16 indices, inside a node translated by (10, 0, 0).
    pub(crate) fn asobo_triangle(extra_json: &str) -> Vec<u8> {
        let mut bin = Vec::new();
        let tri = [[0.0f32, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]];
        for v in tri {
            for c in v {
                bin.extend_from_slice(&c.to_le_bytes());
            }
            bin.extend_from_slice(&[0u8, 127, 0, 127]); // normal +Y
            let uv = [f32_to_f16(0.25), f32_to_f16(0.75)];
            bin.extend_from_slice(&uv[0].to_le_bytes());
            bin.extend_from_slice(&uv[1].to_le_bytes());
        }
        let idx_off = bin.len();
        for i in [0u16, 1, 2, 0] {
            bin.extend_from_slice(&i.to_le_bytes()); // 4th is padding
        }
        let json = format!(
            r#"{{"asset":{{"version":"2.0"}},"scene":0,"scenes":[{{"nodes":[0]}}],
            "nodes":[{{"mesh":0,"translation":[10,0,0]}}],
            "meshes":[{{"name":"tri","primitives":[{{"attributes":{{"POSITION":0,"NORMAL":1,"TEXCOORD_0":2}},"indices":3,"material":0}}]}}],
            "materials":[{{"name":"m","alphaMode":"MASK","doubleSided":true,"pbrMetallicRoughness":{{"baseColorTexture":{{"index":0}}}}{extra_json}}}],
            "textures":[{{"extensions":{{"MSFT_texture_dds":{{"source":0}}}}}}],
            "images":[{{"uri":"F:\\PLASTIC\\OMDB\\TEXTURE\\GATE_ALBD.PNG.KTX2"}}],
            "buffers":[{{"byteLength":{bl}}}],
            "bufferViews":[{{"buffer":0,"byteOffset":0,"byteLength":{vl},"byteStride":20}},{{"buffer":0,"byteOffset":{io},"byteLength":8}}],
            "accessors":[
              {{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"}},
              {{"bufferView":0,"byteOffset":12,"componentType":5120,"count":3,"type":"VEC4"}},
              {{"bufferView":0,"byteOffset":16,"componentType":5122,"count":3,"type":"VEC2"}},
              {{"bufferView":1,"componentType":5123,"count":3,"type":"SCALAR"}}]}}"#,
            bl = bin.len(),
            vl = idx_off,
            io = idx_off
        );
        let mut jbytes = json.into_bytes();
        while jbytes.len() % 4 != 0 {
            jbytes.push(0); // NUL padding, as Asobo writes it
        }
        jbytes.extend_from_slice(&[0, 0, 0, 0]);
        let mut out = b"glTF".to_vec();
        out.extend_from_slice(&2u32.to_le_bytes());
        let total = 12 + 8 + jbytes.len() + 8 + bin.len();
        out.extend_from_slice(&(total as u32).to_le_bytes());
        out.extend_from_slice(&(jbytes.len() as u32).to_le_bytes());
        out.extend_from_slice(b"JSON");
        out.extend_from_slice(&jbytes);
        out.extend_from_slice(&(bin.len() as u32).to_le_bytes());
        out.extend_from_slice(b"BIN\0");
        out.extend_from_slice(&bin);
        out
    }

    /// Wrap a JSON document and binary chunk into a GLB file.
    fn pack_glb(json: String, bin: Vec<u8>) -> Vec<u8> {
        let mut jbytes = json.into_bytes();
        while jbytes.len() % 4 != 0 {
            jbytes.push(b' ');
        }
        let mut out = b"glTF".to_vec();
        out.extend_from_slice(&2u32.to_le_bytes());
        let total = 12 + 8 + jbytes.len() + 8 + bin.len();
        out.extend_from_slice(&(total as u32).to_le_bytes());
        out.extend_from_slice(&(jbytes.len() as u32).to_le_bytes());
        out.extend_from_slice(b"JSON");
        out.extend_from_slice(&jbytes);
        out.extend_from_slice(&(bin.len() as u32).to_le_bytes());
        out.extend_from_slice(b"BIN\0");
        out.extend_from_slice(&bin);
        out
    }

    #[test]
    fn asobo_primitives_take_their_own_slice_of_shared_buffers() {
        // Two triangles in one vertex pool and one index list, as MSFS packs
        // them: the second primitive starts at index 3 with base vertex 3.
        let mut bin = Vec::new();
        let verts = [
            [0.0f32, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
            [100.0, 0.0, 0.0],
            [101.0, 0.0, 0.0],
            [100.0, 0.0, -1.0],
        ];
        for v in verts {
            for c in v {
                bin.extend_from_slice(&c.to_le_bytes());
            }
        }
        let io = bin.len();
        for i in [0u16, 1, 2, 0, 1, 2] {
            bin.extend_from_slice(&i.to_le_bytes());
        }
        let json = format!(
            r#"{{"asset":{{"version":"2.0"}},"scene":0,"scenes":[{{"nodes":[0]}}],"nodes":[{{"mesh":0}}],
            "meshes":[{{"primitives":[
              {{"attributes":{{"POSITION":0}},"indices":1,"extras":{{"ASOBO_primitive":{{"PrimitiveCount":1,"VertexCount":3}}}}}},
              {{"attributes":{{"POSITION":0}},"indices":1,"extras":{{"ASOBO_primitive":{{"BaseVertexIndex":3,"StartIndex":3,"PrimitiveCount":1,"VertexCount":3}}}}}}]}}],
            "buffers":[{{"byteLength":{bl}}}],
            "bufferViews":[{{"buffer":0,"byteOffset":0,"byteLength":{io}}},{{"buffer":0,"byteOffset":{io},"byteLength":12}}],
            "accessors":[{{"bufferView":0,"componentType":5126,"count":6,"type":"VEC3"}},{{"bufferView":1,"componentType":5123,"count":6,"type":"SCALAR"}}]}}"#,
            bl = bin.len(),
            io = io
        );
        let m = load_glb(&pack_glb(json, bin)).unwrap();
        assert_eq!(m.meshes.len(), 2);
        assert_eq!(m.triangle_count(), 2, "each primitive reads only its own triangle");
        assert_eq!(m.meshes[0].vertices.len(), 3, "only the vertices a primitive uses are kept");
        assert_eq!(m.meshes[1].indices, vec![0, 1, 2]);
        assert_eq!(m.meshes[1].vertices[0].pos, [100.0, 0.0, 0.0], "base vertex is applied");
    }

    #[test]
    fn half_floats_decode() {
        assert_eq!(f16_to_f32(0x3C00), 1.0);
        assert_eq!(f16_to_f32(0x3800), 0.5);
        assert_eq!(f16_to_f32(0xC000), -2.0);
        assert_eq!(f16_to_f32(0x0000), 0.0);
        assert!((f16_to_f32(0x0001) - 5.96e-8).abs() < 1e-9, "smallest subnormal");
        assert!(f16_to_f32(0x7C00).is_infinite());
    }

    #[test]
    fn loads_asobo_packing_and_bakes_transforms() {
        let m = load_glb(&asobo_triangle("")).unwrap();
        assert_eq!(m.meshes.len(), 1);
        let v = &m.meshes[0].vertices;
        assert_eq!(v[1].pos, [11.0, 0.0, 0.0], "node translation is baked in");
        assert!((v[0].normal[1] - 1.0).abs() < 1e-6, "i8 normals are signed-normalised");
        assert!(
            (v[0].uv[0] - 0.25).abs() < 1e-3 && (v[0].uv[1] - 0.75).abs() < 1e-3,
            "UVs are half floats"
        );
        assert_eq!(m.meshes[0].indices, vec![0, 1, 2]);
        let mat = &m.materials[0];
        assert_eq!(
            mat.base_color.as_deref(),
            Some("GATE_ALBD.PNG.KTX2"),
            "author path stripped"
        );
        assert_eq!(mat.alpha, AlphaMode::Mask);
        assert!(mat.double_sided);
        assert_eq!(m.triangle_count(), 1);
    }

    #[test]
    fn invisible_materials_are_not_drawn_but_kept_as_click_spots() {
        let m = load_glb(&asobo_triangle(r#","extensions":{"ASOBO_material_invisible":{}}"#)).unwrap();
        assert!(m.meshes.is_empty());
        // MSFS's click spots: invisible, but where the cockpit is clicked.
        assert_eq!(m.click_spots.len(), 1);
    }

    #[test]
    fn a_validators_bare_placeholder_material_is_treated_as_invisible() {
        // FlyByWire's A380 ships exactly this material, unchanged, in both
        // its cockpit and exterior files: nothing but a name, substituted
        // by a glTF validator for a material reference broken before the
        // file ever reached this converter.
        let doc = serde_json::json!({"materials": [{"name": "glTFValidator_Added_Default_Material"}]});
        let (mat, hidden) = material(&doc, &doc["materials"][0]);
        assert!(hidden, "no data to draw honestly, so it is not drawn");
        assert_eq!(mat.base_color, None);

        // Not by name: any material reduced to nothing but its name is
        // caught the same way, since the same validator convention could
        // give it a different one.
        let doc = serde_json::json!({"materials": [{"name": "whatever_a_different_tool_calls_it"}]});
        let (_, hidden) = material(&doc, &doc["materials"][0]);
        assert!(hidden);
    }

    #[test]
    fn an_untextured_material_with_a_real_authored_colour_still_draws() {
        // SUNBLOCKER, as FlyByWire's A380 ships it: no texture, but an
        // explicit, deliberate black factor - not glTF's bare default, and
        // not reduced to just a name. Must not be swept up by the
        // placeholder check above.
        let doc = serde_json::json!({
            "materials": [{
                "name": "SUNBLOCKER",
                "doubleSided": true,
                "pbrMetallicRoughness": {"baseColorFactor": [0.0, 0.0, 0.0, 1.0]},
            }],
        });
        let (mat, hidden) = material(&doc, &doc["materials"][0]);
        assert!(!hidden, "an authored colour is real content, not a broken reference");
        assert_eq!(mat.base_color_factor, [0.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn a_blend_gbuffer_factor_scales_the_alpha_it_carries() {
        // FlyByWire's A380_DETAILS_RIVETS material, texture
        // A380_DETAIL_RIBBON01_ALBEDO: `ASOBO_material_blend_gbuffer`'s
        // `baseColorBlendFactor: 0.13` -- MSFS blends the decal's colour into
        // the fuselage below at 13%; drawn at the texture's own alpha
        // (effectively 100%) it showed as a dark rivet grid.
        let doc = serde_json::json!({
            "materials": [{
                "name": "A380_DETAILS_RIVETS",
                "alphaMode": "BLEND",
                "extensions": {"ASOBO_material_blend_gbuffer": {"baseColorBlendFactor": 0.13}},
                "pbrMetallicRoughness": {"baseColorTexture": {"index": 0}},
            }],
            "textures": [{"source": 0}],
            "images": [{"uri": "A380_DETAIL_RIBBON01_ALBEDO.PNG.DDS"}],
        });
        let (mat, hidden) = material(&doc, &doc["materials"][0]);
        assert!(!hidden, "13% still paints colour, unlike a blend factor of 0");
        assert_eq!(mat.base_color_factor[3], 0.13, "alpha x 0.13");

        // `{}` -- every ASOBO_material_blend_gbuffer factor left at its own
        // default. The exporter omits a factor left at its default, and
        // docs.flightsimulator.com's glTF Schemas page gives the extension's
        // own example as `"baseColorBlendFactor": 1`, so an absent factor
        // leaves the alpha this reads untouched, the same as a material
        // with no blend extension at all.
        let doc = serde_json::json!({
            "materials": [{
                "name": "NEW_DECAL_NOEMIS",
                "alphaMode": "BLEND",
                "extensions": {"ASOBO_material_blend_gbuffer": {}},
                "pbrMetallicRoughness": {"baseColorTexture": {"index": 0}},
            }],
            "textures": [{"source": 0}],
            "images": [{"uri": "ATLAS.DDS"}],
        });
        let (mat, _) = material(&doc, &doc["materials"][0]);
        assert_eq!(mat.base_color_factor[3], 1.0, "an unmarked factor leaves the alpha unchanged");
    }

    #[test]
    fn mirroring_transforms_flip_winding() {
        let mut glb = asobo_triangle("");
        // Rewrite the node to mirror X, byte for byte so the binary chunk and all
        // lengths stay intact.
        let from = br#""translation":[10,0,0]"#;
        let to = br#""scale":[-1,1,1]      "#;
        let at = glb.windows(from.len()).position(|w| w == from).unwrap();
        glb[at..at + from.len()].copy_from_slice(to);
        let m = load_glb(&glb).unwrap();
        assert_eq!(m.meshes[0].indices, vec![0, 2, 1]);
    }

    #[test]
    fn garbage_is_rejected_cleanly() {
        assert!(matches!(
            load_glb(b"not a glb at all, just text"),
            Err(ModelError::NotGlb)
        ));
        let mut glb = asobo_triangle("");
        glb.truncate(glb.len() - 30);
        assert!(load_glb(&glb).is_err());
    }

    /// Load every GLB in `MSFS2XP_GLB_DIR` (real models extracted from a library).
    #[test]
    #[ignore]
    fn real_models_load() {
        let Ok(dir) = std::env::var("MSFS2XP_GLB_DIR") else {
            return;
        };
        let (mut ok, mut tris) = (0, 0);
        for entry in std::fs::read_dir(&dir).unwrap().filter_map(Result::ok) {
            let bytes = std::fs::read(entry.path()).unwrap();
            let m = load_glb(&bytes).unwrap_or_else(|e| panic!("{}: {e}", entry.path().display()));
            assert!(m
                .meshes
                .iter()
                .all(|x| x.vertices.iter().all(|v| v.pos.iter().all(|c| c.is_finite()))));
            tris += m.triangle_count();
            ok += 1;
        }
        eprintln!("{ok} models, {tris} triangles");
    }

    #[test]
    fn msfs_uv_tiling_repeats_the_texture() {
        let glb = asobo_triangle(r#","extensions":{"ASOBO_material_UV_options":{"UVTilingU":10.0,"UVTilingV":4.0}}"#);
        let m = load_glb(&glb).unwrap();
        let uv = m.meshes[0].vertices[0].uv;
        assert!((uv[0] - 2.5).abs() < 1e-2 && (uv[1] - 3.0).abs() < 1e-2, "{uv:?}");
    }

    #[test]
    fn decals_that_paint_no_colour_are_left_out() {
        let glb = asobo_triangle(r#","extensions":{"ASOBO_material_geometry_decal":{"baseColorBlendFactor":0.0}}"#);
        assert!(load_glb(&glb).unwrap().meshes.is_empty());
        let glb = asobo_triangle(r#","extensions":{"ASOBO_material_geometry_decal":{}}"#);
        assert_eq!(load_glb(&glb).unwrap().meshes.len(), 1, "ordinary decals stay");
    }

    /// A document with two materials, two `textures[]` entries (indices 0
    /// and 1) that both resolve to the same image (index 0, "ATLAS.DDS"),
    /// and a third texture (index 2) on a different image ("OTHER.DDS"):
    /// a properly marked decal on texture 0, and a second material (whose
    /// own alphaMode, extensions and texture vary per test) filled in by
    /// the caller. Two distinct `textures[]` entries per image mirrors how
    /// MSFS's exporter actually packs a shared decal atlas: it mints a
    /// fresh texture index per material even when they share one image, so
    /// a fix that compared `baseColorTexture.index` directly (rather than
    /// resolving it to the image it names) never matched real models.
    fn shared_texture_doc(second: serde_json::Value) -> Value {
        serde_json::json!({
            "materials": [
                {
                    "name": "NEW_DECAL_NOEMIS",
                    "alphaMode": "BLEND",
                    "extensions": {"ASOBO_material_blend_gbuffer": {}},
                    "pbrMetallicRoughness": {"baseColorTexture": {"index": 0}},
                },
                second,
            ],
            "textures": [{"source": 0}, {"source": 0}, {"source": 1}],
            "images": [{"uri": "ATLAS.DDS"}, {"uri": "OTHER.DDS"}],
        })
    }

    #[test]
    fn an_opaque_material_sharing_a_decal_atlas_is_promoted() {
        // FlyByWire's A380_COCKPIT_MISC02_LED_GREEN: no alphaMode (glTF
        // default OPAQUE), no ASOBO decal extension, a *different* texture
        // index (1, not the decal sibling's 0) that resolves to the same
        // image (0) as the sibling's.
        let doc = shared_texture_doc(serde_json::json!({
            "name": "A380_COCKPIT_MISC02_LED_GREEN",
            "doubleSided": true,
            "pbrMetallicRoughness": {"baseColorTexture": {"index": 1}},
        }));
        let (mat, _) = material(&doc, &doc["materials"][1]);
        assert_eq!(mat.base_color.as_deref(), Some("ATLAS.DDS"));
        assert_eq!(mat.alpha, AlphaMode::Blend, "promoted from OPAQUE by its marked sibling");
        assert!(mat.decal, "also lifted off its surface like an ordinary decal");
    }

    #[test]
    fn promotion_needs_a_marked_sibling_on_the_same_image() {
        // Texture index 2 resolves to a different image (1): no evidence
        // the atlas is shared, stays OPAQUE.
        let doc = shared_texture_doc(serde_json::json!({
            "name": "unrelated",
            "pbrMetallicRoughness": {"baseColorTexture": {"index": 2}},
        }));
        let (mat, _) = material(&doc, &doc["materials"][1]);
        assert_eq!(mat.alpha, AlphaMode::Opaque);

        // Same image, but the sibling is glass (BLEND without the decal
        // extension): not evidence of a shared decal atlas.
        let doc = serde_json::json!({
            "materials": [
                {
                    "name": "COCKPIT_GLASS",
                    "alphaMode": "BLEND",
                    "pbrMetallicRoughness": {"baseColorTexture": {"index": 0}},
                },
                {
                    "name": "unrelated_opaque",
                    "pbrMetallicRoughness": {"baseColorTexture": {"index": 1}},
                },
            ],
            "textures": [{"source": 0}, {"source": 0}],
            "images": [{"uri": "ATLAS.DDS"}],
        });
        let (mat, _) = material(&doc, &doc["materials"][1]);
        assert_eq!(mat.alpha, AlphaMode::Opaque);
    }

    #[test]
    fn promotion_never_overrides_an_explicit_alpha_mode() {
        // A MASK material on the same atlas keeps its own alphaMode and
        // cutoff; sharing the texture is not reason to treat it as a decal.
        let doc = shared_texture_doc(serde_json::json!({
            "name": "A380_COCKPIT_STICKER_8k",
            "alphaMode": "MASK",
            "alphaCutoff": 0.3,
            "pbrMetallicRoughness": {"baseColorTexture": {"index": 1}},
        }));
        let (mat, _) = material(&doc, &doc["materials"][1]);
        assert_eq!(mat.alpha, AlphaMode::Mask);
        assert_eq!(mat.alpha_cutoff, 0.3);
        assert!(!mat.decal, "masked cutouts are not lifted like blended decals");
    }
}
