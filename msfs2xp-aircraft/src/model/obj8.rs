//! Writing X-Plane OBJ8 files.
//!
//! OBJ8 allows one base texture per object, while an Asobo model usually has
//! several materials with a texture each. A model is therefore split into one
//! object per base texture; all of them are placed at the same point.
//!
//! Axes are turned 180 degrees about the vertical. In MSFS model files a model
//! faces +Z with +X on its left (glTF convention), so for a placement heading of
//! zero +Z points north and +X west. X-Plane object space has +X east and +Z
//! south, so X and Z are both negated. This is a rotation, not a mirror, so
//! triangle winding is unchanged. (O'Hare's Terminal 3 confirms it: its model
//! lies at +X and -Z of a container placed with heading 180, and the terminal
//! is east and north of that point.) Texture rows also need care: glTF
//! puts V = 0 at the top of the image and X-Plane puts T = 0 at the bottom, so
//! T = 1 - V. The texture converter keeps images in X-Plane's orientation so the
//! same rule holds for every texture format.

use std::collections::{BTreeMap, HashMap};
use std::fmt::Write as _;

use super::glb::{AlphaMode, Model, Vertex};

/// How to write one object.
#[derive(Debug, Clone, Default)]
pub struct ObjOptions {
    /// Path of the base texture relative to the object file.
    pub texture: Option<String>,
    /// Uniform scale baked into the vertices (DSF placements cannot scale).
    pub scale: f32,
    /// Height in metres added to every vertex, for objects MSFS places above
    /// the ground (DSF placements always sit on the terrain).
    pub offset_y: f32,
    /// Night texture: X-Plane shows it after dark, like MSFS's day/night
    /// emissive materials.
    pub texture_lit: Option<String>,
    /// Write the model's lights into this object (only one object per model).
    pub lights: bool,
    /// Normal map (`TEXTURE_MAP normal`).
    pub texture_normal: Option<String>,
    /// Metal/gloss map (`TEXTURE_MAP material_gloss`) going with the normal map.
    pub texture_material: Option<String>,
    /// Emit `ATTR_draw_disable`, so the object's triangles are never drawn
    /// but still answer the mouse. For the click object, whose geometry is
    /// an invisible copy of every clickable part: without it X-Plane
    /// rasterizes and blends 53,088 fully transparent triangles over 1,042
    /// draw calls every frame, in each shadow pass too, for something that
    /// by construction cannot be seen. Laminar's own aircraft put the same
    /// attribute in front of their click geometry.
    pub draw_disable: bool,
    /// A flat `GLOBAL_specular <level>` for glass that has a `texture_material`
    /// but no `texture_normal` to pair it with (kept well under 1.0: unlike
    /// an opaque part, glass with a screen or a dial behind it must stay
    /// readable through the reflection). `None` leaves the object exactly as
    /// before -- no material_gloss, no specular -- for every object that is
    /// not this case.
    pub glass_specular: Option<f32>,
    /// Draw this object only within `0..lod_far` metres of the viewer
    /// (`ATTR_LOD 0 lod_far`, in `write_obj8_animated`). `None` draws at
    /// every distance, as every object did before this field existed. A
    /// single animated object always starts its one LOD at 0: unlike
    /// `write_obj8_lods`'s multi-part chain there is no lower-detail
    /// stand-in to hand off to past `lod_far`, so the geometry simply stops
    /// being drawn there -- meant for triangle-heavy detail (tire tread,
    /// say) that is sub-pixel at any distance still worth a draw call.
    pub lod_far: Option<f32>,
}

/// The textures one object carries: base, night and, when normal maps are
/// converted, the normal map with its metal/roughness companion.
#[derive(Debug, Clone, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TextureKey {
    pub base: Option<String>,
    pub lit: Option<String>,
    pub normal: Option<String>,
    pub metal_rough: Option<String>,
    /// Blended glass whose material fades it (MSFS keeps glass opacity in
    /// the material, not the texture): opacity in percent, to bake into a
    /// copy of the texture.
    pub alpha: Option<u8>,
    /// A blended decal (lettering, placards, rivet ribbons) whose material's
    /// `ASOBO_material_blend_gbuffer.baseColorBlendFactor` fades it below
    /// MSFS's own default of full strength: that factor, in percent, to bake
    /// into a copy of the *decal's* dilated texture (`Job::Decal`), the same
    /// way `alpha` bakes glass opacity into `Job::Faded`'s. See its
    /// construction in `split_by_texture_lit` for why a decal's own alpha
    /// test does not stop this the way it would a `mask_cutoff` cutout's.
    pub blend_alpha: Option<u8>,
    /// Glass (MSFS's glass shader, light covers): X-Plane draws such objects
    /// with glass lighting, last, and lets light in through them. Blended
    /// decals stay in ordinary objects; as glass they are lit as windows and
    /// cost a second pass over large meshes.
    pub glass: bool,
    /// Blended but not glass: lettering decals, placards, legend
    /// backgrounds. X-Plane 12 draws an object's blended geometry with its
    /// alpha only when the .acf marks the object as translucent, so these
    /// get their own objects; in an ordinary object every decal quad shows
    /// as a black rectangle.
    pub blend: bool,
    /// Masked (glTF `MASK`) geometry's own alpha-test cutoff, 0..255 on the
    /// same scale as [`DECAL_ALPHA_CUTOFF`], when every mesh grouped under
    /// this key shares one. Declared as `GLOBAL_no_blend` too (see
    /// `DECAL_GLOBAL_ALPHA_TEST`'s doc): an attached aircraft object's own
    /// per-mesh `ATTR_no_blend` is not reliably enough on its own. Unlike
    /// `blend`, masked cutouts are not routed through the decal texture's
    /// dilation (their cutoff is whatever the material says, not the
    /// dilated-atlas convention's fixed 0.90) and do not move the object
    /// into the translucent pass: mask cutouts (window frames, cockpit
    /// stickers) are not reliably small or mostly-transparent the way
    /// lettering decals are, and putting a large mostly-opaque surface in
    /// the translucent pass risks depth-sorting against the ordinary opaque
    /// geometry drawn around it.
    pub mask_cutoff: Option<u8>,
    /// For a material with no texture: its colour (sRGB, with alpha), for a
    /// texture of that one colour. Untextured, X-Plane draws it light grey.
    pub solid: Option<[u8; 4]>,
    /// For a material that has *both* a base colour texture and a
    /// `baseColorFactor`: that factor's RGB, sRGB-encoded, to multiply into
    /// a copy of the texture.
    ///
    /// glTF defines the factor as a multiplier on the texture; [`Self::solid`]
    /// reads it only when there is no texture to multiply, which silently
    /// dropped it everywhere else. MSFS uses it constantly: on the A380's
    /// cockpit, 34 of the 35 materials carrying both are tinted by it, and
    /// `DECAL_BLACK_NOEMIS` is [0, 0, 0, 1] -- black placards drawn from the
    /// same white lettering stencil the white ones use. Without the factor
    /// every one of them renders as the raw white stencil.
    ///
    /// It has to be part of this key, not applied to the texture in place:
    /// four of the A380's base colour images are drawn by materials with
    /// different factors, the cockpit decal atlas by five at once (white,
    /// 0.8 grey and two different blacks), so each tint needs a texture of
    /// its own the way [`Self::alpha`] already does for opacity.
    pub tint: Option<[u8; 3]>,
    /// For a material with an emissive texture and an `emissiveFactor` that
    /// is not plain white: that factor's RGB, sRGB-encoded, to multiply into
    /// a copy of the night texture the same way [`Self::tint`] does for the
    /// day one. glTF defines `emissiveFactor` as a per-channel multiplier on
    /// the emissive texture; collapsing it to a single "does it glow" gate
    /// (its strongest channel, the old `emissive_strength`) drew every
    /// glowing texture at full, untinted brightness - wrong for the A380's
    /// windshield-frame glow (factor 0.7, should be dimmed) and its
    /// pushbutton legends (0.78/0.85/1.00, a faint blue cast).
    pub lit_tint: Option<[u8; 3]>,
}

/// Linear 0..1 to sRGB 0..255, the transfer `msfs2xp::texture::tint_rgba`
/// inverts when it applies the tint. glTF factors are linear.
fn linear_to_srgb_u8(l: f32) -> u8 {
    let c = if l <= 0.003_130_8 { l * 12.92 } else { 1.055 * l.powf(1.0 / 2.4) - 0.055 };
    (c.clamp(0.0, 1.0) * 255.0).round() as u8
}

/// Group a model's meshes by the textures they need, in a stable order. An
/// object can carry one of each, so glowing materials (and, with `normals`,
/// materials with their own normal map) get their own group.
pub fn split_by_texture(model: &Model, normals: bool) -> Vec<(TextureKey, Vec<usize>)> {
    split_by_texture_lit(model, normals, &|_| None)
}

/// [`split_by_texture`], where `driven(mesh)` says whether an emissive code
/// lights the mesh (`Some(true)`: its emissive texture is its night texture
/// even with a zero emissive factor, as MSFS's code overrides the factor;
/// `Some(false)`: never lit) or leaves it to its material (`None`).
pub fn split_by_texture_lit(model: &Model, normals: bool, driven: &dyn Fn(usize) -> Option<bool>) -> Vec<(TextureKey, Vec<usize>)> {
    let mut groups: BTreeMap<TextureKey, Vec<usize>> = BTreeMap::new();
    for (i, mesh) in model.meshes.iter().enumerate() {
        if mesh.indices.is_empty() {
            continue;
        }
        let m = model.materials.get(mesh.material);
        // Decals (lettering) carry no normal map of their own: their COMP
        // roughness is unrelated to the paint they sit on, and with it the
        // lettering mirrored the sun as a white glare.
        let normal = m.filter(|m| normals && !m.decal).and_then(|m| m.normal.clone());
        let key = TextureKey {
            base: m.and_then(|m| m.base_color.clone()),
            lit: m.filter(|m| driven(i).unwrap_or(m.emissive_factor.iter().any(|&c| c > 0.0))).and_then(|m| m.emissive.clone()),
            // Kept for glass even without a normal map: glass has no relief
            // to bump-map, but its own COMP source still says how strongly
            // it should reflect (FBW's glass shader carries
            // `glassReflectionMaskFactor`), and dropping it left every
            // BLEND_GLASS object with no normal map -- the DU/EFB display
            // covers, cabin windows -- with no shine at all (see
            // `ObjOptions::glass_specular`, main.rs).
            metal_rough: m
                .filter(|m| normal.is_some() || (m.alpha == AlphaMode::Blend && m.glass))
                .and_then(|m| m.metal_rough.clone()),
            normal,
            glass: m.is_some_and(|m| m.alpha == AlphaMode::Blend && m.glass),
            blend: m.is_some_and(|m| m.alpha == AlphaMode::Blend && !m.glass),
            mask_cutoff: m
                .filter(|m| m.alpha == AlphaMode::Mask)
                .map(|m| (m.alpha_cutoff.clamp(0.0, 1.0) * 255.0).round() as u8),
            // Glass only: a decal's own fade is `blend_alpha` below, baked
            // through `Job::Decal` instead of `Job::Faded` so it still gets
            // the dilate-then-remip treatment a decal's transparent texels
            // need (see `blend_alpha`'s own doc).
            alpha: m.filter(|m| m.alpha == AlphaMode::Blend && m.glass).map(|m| {
                // MSFS's glass shader shows the world through the glass;
                // without it the texture alone reads as dark tinted paint
                // (a black windscreen), so glass keeps at most a quarter.
                let a = m.base_color_factor[3].min(0.25);
                (a.clamp(0.0, 1.0) * 100.0).round() as u8
            }),
            // A blended decal's own MSFS opacity: `ASOBO_material_blend_gbuffer`'s
            // `baseColorBlendFactor`, already multiplied into `base_color_factor[3]`
            // by `glb::material()`. Decals draw with real translucency, not an
            // alpha test (`DECAL_ALPHA_TEST` is `ATTR_blend`, the same directive
            // plain glass gets -- see `material_attrs` and the
            // `a_decal_object_blends_rather_than_declaring_an_alpha_test` test),
            // so scaling every texel's alpha by this factor reproduces the same
            // fade a real blend would, unlike a glTF `MASK` cutout's alpha test
            // (mask_cutoff above), which can only keep a texel whole or drop it.
            // FlyByWire's exterior rivet ribbons (`A380_DETAILS_RIVETS`, texture
            // `A380_DETAIL_RIBBON01_ALBEDO`) blend their colour at 13%; drawn at
            // the texture's own alpha (effectively 100%, the extension's own
            // default when the exporter omits it -- docs.flightsimulator.com's
            // glTF Schemas page for `ASOBO_material_blend_gbuffer`,
            // `"baseColorBlendFactor": 1`) the fuselage showed a dark rivet grid.
            // `< 1.0` (not `!= 1.0`): the factor only ever fades a decal, never
            // amplifies it past its own texture, so a value at or above 1
            // (the unmarked default, or a future exporter writing it explicitly)
            // needs no second copy of the texture.
            blend_alpha: m.filter(|m| m.alpha == AlphaMode::Blend && !m.glass && m.base_color_factor[3] < 1.0).map(|m| {
                (m.base_color_factor[3].clamp(0.0, 1.0) * 100.0).round() as u8
            }),
            // MSFS colours these in the material (black sun blockers), or its
            // gauges draw on them (`$SCREEN_...`), which are dark when off.
            solid: m.filter(|m| m.base_color.is_none()).map(|m| {
                if m.name.starts_with('$') {
                    return [0, 0, 0, 255];
                }
                let f = m.base_color_factor;
                let srgb = |c: f32| (c.clamp(0.0, 1.0).powf(1.0 / 2.2) * 255.0).round() as u8;
                [srgb(f[0]), srgb(f[1]), srgb(f[2]), (f[3].clamp(0.0, 1.0) * 255.0).round() as u8]
            }),
            // Only where there is a texture for it to multiply; without one
            // the factor *is* the colour, which `solid` already handles.
            tint: m.filter(|m| m.base_color.is_some()).and_then(|m| {
                let f = m.base_color_factor;
                // A factor within half a code value of white changes nothing
                // a texel can represent, so it is not worth a second copy of
                // the texture.
                let t = [linear_to_srgb_u8(f[0]), linear_to_srgb_u8(f[1]), linear_to_srgb_u8(f[2])];
                (t != [255, 255, 255]).then_some(t)
            }),
            // Same reasoning as `tint`, for the night texture: only where
            // there is one for `emissive_factor` to multiply, and only when
            // it actually changes a texel. Values above white (MSFS drives
            // some materials over 1, e.g. 5.0, with no
            // `KHR_materials_emissive_strength` extension to say how much
            // brighter than the texture that means) clamp to no change in
            // `linear_to_srgb_u8`, the same as an over-1 `baseColorFactor`
            // already does for `tint` - conservative: it never darkens a
            // texture that should be brighter than it already draws.
            lit_tint: m.filter(|m| m.emissive.is_some()).and_then(|m| {
                let f = m.emissive_factor;
                let t = [linear_to_srgb_u8(f[0]), linear_to_srgb_u8(f[1]), linear_to_srgb_u8(f[2])];
                (t != [255, 255, 255]).then_some(t)
            }),
        };
        groups.entry(key).or_default().push(i);
    }
    groups.into_iter().collect()
}

/// One level of detail: some meshes of a model, drawn when the viewer is
/// between `near` and `far` metres away.
#[derive(Debug, Clone)]
pub struct LodPart<'a> {
    pub model: &'a Model,
    pub meshes: Vec<usize>,
    pub near: f32,
    /// Infinite means no distance limit.
    pub far: f32,
}

/// Render the given meshes of a model as one OBJ8 file.
pub fn write_obj8(model: &Model, meshes: &[usize], opts: &ObjOptions) -> String {
    write_obj8_lods(
        &[LodPart {
            model,
            meshes: meshes.to_vec(),
            near: 0.0,
            far: f32::INFINITY,
        }],
        opts,
    )
}

/// Render levels of detail as one OBJ8 file. X-Plane draws only the part whose
/// range holds the viewer's distance, and nothing beyond the last part. The
/// lights (the first part's model's) are repeated in every part so they show
/// whichever part is drawn.
pub fn write_obj8_lods(parts: &[LodPart], opts: &ObjOptions) -> String {
    let scale = if opts.scale.is_finite() && opts.scale > 0.0 {
        opts.scale
    } else {
        1.0
    };
    let ranged = parts.iter().any(|p| p.far.is_finite());
    let mut vt = String::new();
    let mut indices: Vec<u32> = Vec::new();
    // Per part, (first index, count, material) per mesh, for the command section.
    let mut part_spans: Vec<Vec<(usize, usize, usize)>> = Vec::new();
    let mut base = 0u32;
    for part in parts {
        let mut spans = Vec::new();
        for &mi in &part.meshes {
            let Some(mesh) = part.model.meshes.get(mi) else { continue };
            if mesh.indices.is_empty() {
                continue;
            }
            for v in &mesh.vertices {
                let _ = writeln!(
                    vt,
                    "VT {:.4} {:.4} {:.4} {:.4} {:.4} {:.4} {:.5} {:.5}",
                    // Adding 0.0 turns -0.0 into 0.0 so output stays tidy.
                    -v.pos[0] * scale + 0.0,
                    v.pos[1] * scale + opts.offset_y,
                    -v.pos[2] * scale + 0.0,
                    -v.normal[0] + 0.0,
                    v.normal[1],
                    -v.normal[2] + 0.0,
                    v.uv[0],
                    1.0 - v.uv[1]
                );
            }
            spans.push((indices.len(), mesh.indices.len(), mesh.material));
            indices.extend(mesh.indices.iter().map(|&i| i + base));
            base += mesh.vertices.len() as u32;
        }
        part_spans.push(spans);
    }

    let mut out = String::with_capacity(vt.len() + indices.len() * 8 + 256);
    out.push_str("I\n800\nOBJ\n\n");
    if let Some(tex) = &opts.texture {
        let _ = writeln!(out, "TEXTURE {tex}");
    }
    if let Some(lit) = &opts.texture_lit {
        let _ = writeln!(out, "TEXTURE_LIT {lit}");
    }
    if let Some(normal) = &opts.texture_normal {
        // X-Plane 12's layout, as Laminar's objects have it: the normal map
        // and a separate metal/gloss map, each at its own size. Without
        // GLOBAL_specular the object has no shine at all, whatever the maps say.
        //
        // No NORMAL_METALNESS. It was declared here (W96) on the strength
        // of a survey of Laminar's own shipped aircraft: every OBJ8 under
        // Aircraft/Laminar Research that carries `TEXTURE_MAP
        // material_gloss` (45 files) also declares it, with no exceptions.
        // But NORMAL_METALNESS does not select a workflow for
        // material_gloss at all -- the spec ties it to the *normal* map's
        // own blue and alpha channels, the older single-texture layout
        // where metalness and gloss are packed into the normal map itself
        // (`msfs2xp::texture::normal_metal_png`'s blue = metalness, alpha =
        // gloss), a layout this converter does not write. The normal map it
        // actually writes (`msfs2xp::texture::normal_png`) is a plain RGB
        // PNG with no alpha channel and an always-zero blue channel; the
        // metalness signal lives in the *separate* material_gloss map's own
        // red channel (`msfs2xp::texture::material_png`: red `metal()`,
        // green `gloss()`, blue always zero) regardless of this flag.
        // Laminar's own aircraft presumably have real data in those
        // channels of their *normal* maps; this converter's do not, so
        // declaring the flag told X-Plane to read a metalness/gloss signal
        // out of channels that carry none. Removed 2026-09-27 against the
        // installed, user-approved cockpit, which has it stripped for
        // exactly this reason.
        let _ = writeln!(out, "TEXTURE_MAP normal {normal}");
        if let Some(material) = &opts.texture_material {
            let _ = writeln!(out, "TEXTURE_MAP material_gloss {material}");
        }
        out.push_str("GLOBAL_specular 1.0\n");
    }
    let _ = writeln!(out, "POINT_COUNTS {} 0 0 {}\n", base, indices.len());
    out.push_str(&vt);
    out.push('\n');
    let mut chunks = indices.chunks_exact(10);
    for c in chunks.by_ref() {
        let _ = writeln!(
            out,
            "IDX10 {} {} {} {} {} {} {} {} {} {}",
            c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7], c[8], c[9]
        );
    }
    for i in chunks.remainder() {
        let _ = writeln!(out, "IDX {i}");
    }
    out.push('\n');

    for (part, spans) in parts.iter().zip(&part_spans) {
        if ranged {
            let far = if part.far.is_finite() { part.far } else { 100_000.0 };
            let _ = writeln!(out, "ATTR_LOD {:.0} {:.0}", part.near, far);
        }
        // Emit state changes only when they differ from the previous span;
        // each level of detail starts afresh.
        let (mut cull, mut blend, mut offset): (Option<bool>, Option<String>, Option<u8>) = (None, None, None);
        let mut shadow: Option<bool> = None;
        for &(first, count, mat) in spans {
            let m = part.model.materials.get(mat).cloned().unwrap_or_default();
            let want_cull = !m.double_sided;
            if cull != Some(want_cull) {
                out.push_str(if want_cull { "ATTR_cull\n" } else { "ATTR_no_cull\n" });
                cull = Some(want_cull);
            }
            let want_blend = match m.alpha {
                AlphaMode::Blend if !m.glass => DECAL_ALPHA_TEST.to_string(),
                AlphaMode::Blend => "ATTR_blend".to_string(),
                AlphaMode::Mask => format!("ATTR_no_blend {:.2}", m.alpha_cutoff.clamp(0.0, 1.0)),
                // glTF OPAQUE ignores alpha. MSFS opaque albedo textures often hold
                // unrelated data in alpha, and a 0.5 cutoff would punch holes in walls.
                AlphaMode::Opaque => "ATTR_no_blend 0.00".to_string(),
            };
            if blend.as_deref() != Some(want_blend.as_str()) {
                let _ = writeln!(out, "{want_blend}");
                blend = Some(want_blend);
            }
            let want_shadow = casts_shadow(&m, false);
            if shadow.unwrap_or(true) != want_shadow {
                out.push_str(if want_shadow { "ATTR_shadow\n" } else { "ATTR_no_shadow\n" });
                shadow = Some(want_shadow);
            }
            // No polygon offset for decals: X-Plane treats offset geometry as
            // draped, and lettering vanished with it (see `lift_decals`).
            let want_offset = 0;
            if offset != Some(want_offset) {
                let _ = writeln!(out, "ATTR_poly_os {want_offset}");
                offset = Some(want_offset);
            }
            let _ = writeln!(out, "TRIS {first} {count}");
        }
        if opts.lights {
            if let Some(first) = parts.first() {
                write_lights(&mut out, first.model, scale, opts.offset_y);
            }
        }
    }
    out
}

/// What surrounds a mesh's triangles in an animated object.
#[derive(Debug, Clone, Default, PartialEq, Eq, Hash)]
pub struct MeshAnim {
    /// Commands between ANIM_begin and ANIM_end; empty for a static mesh.
    pub commands: String,
    /// Manipulator line(s) (ATTR_manip_*, ATTR_manip_wheel) for clickable parts.
    pub manip: Option<String>,
    /// Clicked but not drawn (ATTR_draw_disable), for click spots.
    pub hidden: bool,
    /// A click spot: drawn with a polygon offset so it wins the depth test
    /// against the part it covers.
    pub click: bool,
    /// `ATTR_light_level` arguments ("v1 v2 dataref"): how brightly the
    /// night texture shows.
    pub light: Option<String>,
    /// `ATTR_cockpit_device` arguments ("id bus channel auto_adjust"): a
    /// screen the systems plugin draws, mapped by the mesh's UVs.
    pub device: Option<String>,
}

impl MeshAnim {
    fn is_plain(&self) -> bool {
        self.commands.is_empty() && self.manip.is_none() && !self.hidden && !self.click && self.light.is_none() && self.device.is_none()
    }
}

/// Material state last written, so attributes are only written on change.
#[derive(Default)]
struct AttrState {
    cull: Option<bool>,
    blend: Option<String>,
    offset: Option<u8>,
    shadow: Option<bool>,
}

/// Whether a mesh with this material casts a shadow. Decals and click spots
/// do not: X-Plane casts a shadow from the whole quad, see-through parts
/// included, and a decal lifted off its panel left a black rectangle under
/// every label.
fn casts_shadow(m: &super::glb::Material, click: bool) -> bool {
    !click && !(m.alpha == AlphaMode::Blend && !m.glass)
}

/// A draw batch smaller than this across casts no shadow. X-Plane draws
/// every shadow-casting batch again in each shadow pass; a knob, pushbutton,
/// screw or key throws a shadow a few millimetres across that nobody can
/// see, yet on the A380 those parts were 1,471 of the 1,649 batches in every
/// interior shadow pass (2026-09-28, CPU-bound at 58 ms a frame). They still
/// receive shadows, so a panel in the glareshield's shade stays in it; only
/// the structure -- panels, glareshield, walls, seats -- casts.
const SMALL_PART_SHADOW_M: f32 = 0.15;

/// A batch's bounding-box diagonal, in metres.
fn span_extent_m(positions: &[[f32; 3]], indices: &[u32]) -> f32 {
    let (mut lo, mut hi) = ([f32::INFINITY; 3], [f32::NEG_INFINITY; 3]);
    for &i in indices {
        let Some(p) = positions.get(i as usize) else { continue };
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    if lo[0] > hi[0] {
        return 0.0;
    }
    ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt()
}

/// Lettering, placards and legend decals blend, as MSFS draws them.
///
/// They used to be alpha-tested instead, because X-Plane 12 drew their
/// transparent parts as black rectangles around every letter. That was the
/// .acf not marking the object translucent, which it now does
/// (`OBJ_FLAG_TRANSLUCENT`, `acf::translucent_flags`), and the night
/// texture's alpha being opaque, which `Job::LitMasked` now fixes.
///
/// A hard test cannot survive a mip. Measured on the A380's decal atlas,
/// the share of texels above the 0.90 cutoff falls 20.4% at full size to
/// 11.4% at 512, 3.8% at 256 and 1.9% at 128: at the level a cockpit
/// legend is really sampled from at head distance, four fifths of the
/// lettering has averaged away and the placard reads blank until you lean
/// into it. Blending draws a stroke thinner than a texel as the partial
/// coverage it is, which is the whole point of it.
const DECAL_ALPHA_TEST: &str = "ATTR_blend";

/// The ratio of full alpha both of those test against, so the test below can
/// hold them and [`DECAL_ALPHA_CUTOFF`] to one number.
#[cfg(test)]
const DECAL_ALPHA_RATIO: f32 = 0.90;

/// [`DECAL_ALPHA_TEST`]'s cutoff as a texture alpha byte (0.90 of 255,
/// rounded up: X-Plane keeps a texel whose alpha is at least the ratio), for
/// the texture converter to keep a decal's mips at the same coverage against
/// as X-Plane tests against here (see
/// `msfs2xp::texture::preserve_alpha_coverage`).
pub const DECAL_ALPHA_CUTOFF: u8 = 230;

fn material_attrs(out: &mut String, m: &super::glb::Material, st: &mut AttrState, click: bool, small: bool) {
    let want_cull = !m.double_sided;
    if st.cull != Some(want_cull) {
        out.push_str(if want_cull { "ATTR_cull\n" } else { "ATTR_no_cull\n" });
        st.cull = Some(want_cull);
    }
    let want_blend = match m.alpha {
        AlphaMode::Blend if !m.glass => DECAL_ALPHA_TEST.to_string(),
        AlphaMode::Blend => "ATTR_blend".to_string(),
        AlphaMode::Mask => format!("ATTR_no_blend {:.2}", m.alpha_cutoff.clamp(0.0, 1.0)),
        AlphaMode::Opaque => "ATTR_no_blend 0.00".to_string(),
    };
    if st.blend.as_deref() != Some(want_blend.as_str()) {
        let _ = writeln!(out, "{want_blend}");
        st.blend = Some(want_blend);
    }
    // Click spots are drawn with a larger offset so they win the depth test
    // against the part they cover, and cast no shadow.
    // Decals get no offset: X-Plane treats offset geometry as draped, and
    // lettering vanished with it; decal meshes are lifted off their surface
    // instead (see `lift_decals`).
    let want_offset = if click { 3 } else { 0 };
    if st.offset != Some(want_offset) {
        let _ = writeln!(out, "ATTR_poly_os {want_offset}");
        st.offset = Some(want_offset);
    }
    let want_shadow = casts_shadow(m, click) && !small;
    // Objects cast shadows unless told otherwise.
    if st.shadow.unwrap_or(true) != want_shadow {
        out.push_str(if want_shadow { "ATTR_shadow\n" } else { "ATTR_no_shadow\n" });
        st.shadow = Some(want_shadow);
    }
}

/// Render meshes as one OBJ8 where each mesh may be animated, clickable or
/// hidden. Meshes sharing the same animation go in one ANIM block; static
/// meshes come first. `extra` (lights) is written at the end.
/// `header` goes into the object's header (e.g. `GLOBAL_cockpit_lit`).
pub fn write_obj8_animated(
    model: &Model,
    meshes: &[usize],
    opts: &ObjOptions,
    anim: &dyn Fn(usize) -> MeshAnim,
    extra: &str,
    header: &str,
) -> String {
    let scale = if opts.scale.is_finite() && opts.scale > 0.0 { opts.scale } else { 1.0 };
    // Group meshes by their animation, static first, then by material.
    let mut ids: HashMap<MeshAnim, usize> = HashMap::new();
    let mut keys: Vec<MeshAnim> = Vec::new();
    let mut order: Vec<(bool, usize, usize, usize)> = Vec::new();
    for &mi in meshes {
        let Some(mesh) = model.meshes.get(mi) else { continue };
        if mesh.indices.is_empty() {
            continue;
        }
        let key = anim(mi);
        let plain = key.is_plain();
        let gid = *ids.entry(key.clone()).or_insert_with(|| {
            keys.push(key);
            keys.len() - 1
        });
        order.push((!plain, gid, mesh.material, mi));
    }
    order.sort_unstable();

    let mut vt = String::new();
    let mut indices: Vec<u32> = Vec::new();
    // Each written vertex's position, for `span_extent_m`.
    let mut positions: Vec<[f32; 3]> = Vec::new();
    // (group, first index, count, material), merged when contiguous.
    let mut spans: Vec<(usize, usize, usize, usize)> = Vec::new();
    let mut base = 0u32;
    for &(_, gid, mat, mi) in &order {
        let mesh = &model.meshes[mi];
        positions.extend(mesh.vertices.iter().map(|v| [v.pos[0] * scale, v.pos[1] * scale, v.pos[2] * scale]));
        for v in &mesh.vertices {
            let _ = writeln!(
                vt,
                "VT {:.4} {:.4} {:.4} {:.4} {:.4} {:.4} {:.5} {:.5}",
                -v.pos[0] * scale + 0.0,
                v.pos[1] * scale + opts.offset_y,
                -v.pos[2] * scale + 0.0,
                -v.normal[0] + 0.0,
                v.normal[1],
                -v.normal[2] + 0.0,
                v.uv[0],
                1.0 - v.uv[1]
            );
        }
        let first = indices.len();
        indices.extend(mesh.indices.iter().map(|&i| i + base));
        base += mesh.vertices.len() as u32;
        match spans.last_mut() {
            Some(s) if s.0 == gid && s.3 == mat && s.1 + s.2 == first => s.2 += mesh.indices.len(),
            _ => spans.push((gid, first, mesh.indices.len(), mat)),
        }
    }

    let mut out = String::with_capacity(vt.len() + indices.len() * 8 + extra.len() + 256);
    out.push_str("I\n800\nOBJ\n\n");
    out.push_str(header);
    if let Some(tex) = &opts.texture {
        let _ = writeln!(out, "TEXTURE {tex}");
    }
    if let Some(lit) = &opts.texture_lit {
        let _ = writeln!(out, "TEXTURE_LIT {lit}");
    }
    if let Some(normal) = &opts.texture_normal {
        // X-Plane 12's layout, as Laminar's objects have it: the normal map
        // and a separate metal/gloss map, each at its own size. Without
        // GLOBAL_specular the object has no shine at all, whatever the maps say.
        // No NORMAL_METALNESS: see `write_obj8`'s identical block above for
        // why (it does not select a workflow for material_gloss at all).
        let _ = writeln!(out, "TEXTURE_MAP normal {normal}");
        if let Some(material) = &opts.texture_material {
            let _ = writeln!(out, "TEXTURE_MAP material_gloss {material}");
        }
        out.push_str("GLOBAL_specular 1.0\n");
    } else if let (Some(material), Some(level)) = (&opts.texture_material, opts.glass_specular) {
        // Glass has nothing to bump-map, so there is no normal map here --
        // but the spec ties neither TEXTURE_MAP material_gloss nor
        // GLOBAL_specular to one; GLOBAL_specular "is similar to
        // ATTR_shiny_rat" (a plain shininess level), and each TEXTURE_MAP
        // usage stands on its own. `level` is kept well under 1.0 so the
        // reflection this adds does not wash out whatever is behind the
        // pane -- a screen, most of the time.
        let _ = writeln!(out, "TEXTURE_MAP material_gloss {material}");
        let _ = writeln!(out, "GLOBAL_specular {level:.2}");
    }
    let _ = writeln!(out, "POINT_COUNTS {} 0 0 {}\n", base, indices.len());
    out.push_str(&vt);
    out.push('\n');
    let mut chunks = indices.chunks_exact(10);
    for c in chunks.by_ref() {
        let _ = writeln!(out, "IDX10 {} {} {} {} {} {} {} {} {} {}", c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7], c[8], c[9]);
    }
    for i in chunks.remainder() {
        let _ = writeln!(out, "IDX {i}");
    }
    out.push('\n');

    // Applies to every TRIS this object writes. The OBJ8 spec: an ATTR_LOD
    // "should be used to represent the object when the viewer is between
    // near (inclusive) and far (exclusive)"; with only one range declared
    // and nothing after `far`, X-Plane has nothing left to draw beyond it.
    if let Some(far) = opts.lod_far.filter(|f| f.is_finite() && *f > 0.0) {
        let _ = writeln!(out, "ATTR_LOD 0 {far:.0}");
    }

    if opts.draw_disable {
        out.push_str("ATTR_draw_disable\n");
    }

    let mut st = AttrState::default();
    let mut i = 0;
    while i < spans.len() {
        let key = &keys[spans[i].0];
        if !key.commands.is_empty() {
            out.push_str("ANIM_begin\n");
            out.push_str(&key.commands);
        }
        if key.hidden {
            out.push_str("ATTR_draw_disable\n");
        }
        if let Some(m) = &key.manip {
            out.push_str(m);
            out.push('\n');
        }
        // A screen's brightness is the systems plugin's, not the model's.
        // MSFS gives a screen mesh an emissive code like any other lit part,
        // and the converter turns that into an `ATTR_light_level` -- a second
        // multiplier over a device whose brightness the plugin already sets
        // (`display/mod.rs`'s brightness callback, and its own per-region
        // dimming), driven by a dataref that reads zero on a cold aircraft.
        // Laminar write none: sixteen of their eighteen objects carrying a
        // cockpit device have no `ATTR_light_level` in them at all.
        if let Some(l) = key.light.as_ref().filter(|_| key.device.is_none()) {
            let _ = writeln!(out, "ATTR_light_level {l}");
        }
        if let Some(d) = &key.device {
            let _ = writeln!(out, "ATTR_cockpit_device {d}");
            // A device's own triangles are NOT touch-sensitive by
            // themselves -- the OBJ8 spec is explicit that
            // `ATTR_manip_device` is what "creates a touch screen
            // manipulator for a cockpit device", and that the manipulator
            // "must also be tagged with ATTR_cockpit_device" for X-Plane to
            // recognise it (developer.x-plane.com/article/
            // obj8-file-format-specification). Laminar's `FMS.obj` (three
            // `ATTR_cockpit_device` MCDUs, no manipulator) is not a
            // counter-example: those MCDUs are operated through their own
            // bezel buttons, not tapped on the glass, so they simply never
            // needed one.
            //
            // `key.manip` (set above) is a screen's manipulator when it has
            // one: `main.rs` puts it only on the screens the real aircraft
            // is actually touched on (`screens::TOUCH_SCREENS`). Proven
            // pre-merge (`Log-keep-113715.txt`: "input callback: touch on
            // SCREEN_EFB", "touch on SCREEN_DU_MFD"), that manipulator keeps
            // working wherever its device is drawn, even in an object
            // X-Plane's single-cockpit-object arbitration does not pick --
            // so it stays on the screen's own triangles, in the object that
            // draws them, rather than moving to the click object.
        }
        let gid = spans[i].0;
        while i < spans.len() && spans[i].0 == gid {
            let (_, first, count, mat) = spans[i];
            let m = model.materials.get(mat).cloned().unwrap_or_default();
            let small = span_extent_m(&positions, &indices[first..first + count]) < SMALL_PART_SHADOW_M;
            material_attrs(&mut out, &m, &mut st, key.click, small);
            let _ = writeln!(out, "TRIS {first} {count}");
            i += 1;
        }
        if key.manip.is_some() {
            out.push_str("ATTR_manip_none\n");
        }
        if key.light.is_some() && key.device.is_none() {
            out.push_str("ATTR_light_level_reset\n");
        }
        if key.device.is_some() {
            out.push_str("ATTR_no_cockpit\n");
        }
        if key.hidden {
            out.push_str("ATTR_draw_enable\n");
        }
        if !key.commands.is_empty() {
            out.push_str("ANIM_end\n");
        }
    }
    out.push_str(extra);
    out
}

/// One mesh's geometry at a particular level of detail: `mesh` is the index
/// into [`Model::meshes`] used to look up its material and the `anim`
/// closure passed to [`write_obj8_animated_lods`], so the same
/// animation/manipulator/light/device information applies at every level a
/// mesh has geometry at. `vertices`/`indices` are this level's own copy --
/// full detail at L0, a [`simplify_mesh`] result at L1/L2 -- not a slice
/// into the original mesh, so a simplified level carries no dead vertices.
/// An empty `indices` means this mesh draws nothing at this level; its
/// animation wrapper is then skipped there rather than written empty.
#[derive(Debug, Clone)]
pub struct LodMesh<'a> {
    pub mesh: usize,
    pub vertices: &'a [Vertex],
    pub indices: &'a [u32],
}

/// One level of detail across a whole animated object: every mesh's own
/// geometry between `near` and `far` metres from the viewer, for
/// [`write_obj8_animated_lods`].
#[derive(Debug, Clone)]
pub struct AnimLodLevel<'a> {
    pub near: f32,
    /// Infinite means no distance limit (matches [`LodPart::far`]).
    pub far: f32,
    pub meshes: Vec<LodMesh<'a>>,
}

/// [`write_obj8_animated`], but drawing different geometry per level of
/// detail (`levels`, nearest first) instead of one level at every distance --
/// the exterior LOD chain: full detail close in, a [`simplify_mesh`] result
/// further out. Every level repeats the animation hierarchy of the meshes it
/// has geometry for, as the OBJ8 spec requires of `ATTR_LOD` sections (each
/// one is self-contained; a control surface, gear leg or fan blade keeps
/// moving in its simplified levels too, because its `ANIM_begin`/`ANIM_end`
/// wrapper is written fresh around whichever level's triangles it has).
///
/// A single level with an infinite `far` writes no `ATTR_LOD` at all and
/// reproduces [`write_obj8_animated`]'s output exactly for the same meshes
/// (see `--no-exterior-lods` in `main.rs`, and
/// `a_single_level_matches_write_obj8_animated` below): the grouping,
/// sorting and attribute-state logic are the same code, just generalised
/// from one implicit level to a list of them.
pub fn write_obj8_animated_lods(
    model: &Model,
    levels: &[AnimLodLevel],
    opts: &ObjOptions,
    anim: &dyn Fn(usize) -> MeshAnim,
    extra: &str,
    header: &str,
) -> String {
    let scale = if opts.scale.is_finite() && opts.scale > 0.0 { opts.scale } else { 1.0 };
    let ranged = levels.iter().any(|l| l.far.is_finite());
    // Group id is by `MeshAnim` alone, shared across every level: the same
    // mesh's animation/manip/light/device is identical whichever level draws
    // it, so one map keeps `keys` (and therefore the written commands) the
    // same object whether it is first seen at L0 or, for a mesh with no L0
    // entry, at a later level.
    let mut ids: HashMap<MeshAnim, usize> = HashMap::new();
    let mut keys: Vec<MeshAnim> = Vec::new();

    let mut vt = String::new();
    let mut indices: Vec<u32> = Vec::new();
    let mut base = 0u32;
    // Per level, (group, first index, count, material) spans, merged when contiguous.
    let mut level_spans: Vec<Vec<(usize, usize, usize, usize)>> = Vec::new();

    for level in levels {
        let mut order: Vec<(bool, usize, usize, usize)> = Vec::new(); // (!plain, gid, mat, li)
        for (li, lm) in level.meshes.iter().enumerate() {
            if lm.indices.is_empty() {
                continue;
            }
            let key = anim(lm.mesh);
            let plain = key.is_plain();
            let gid = *ids.entry(key.clone()).or_insert_with(|| {
                keys.push(key);
                keys.len() - 1
            });
            let mat = model.meshes.get(lm.mesh).map_or(0, |m| m.material);
            order.push((!plain, gid, mat, li));
        }
        order.sort_unstable();

        let mut spans: Vec<(usize, usize, usize, usize)> = Vec::new();
        for &(_, gid, mat, li) in &order {
            let lm = &level.meshes[li];
            for v in lm.vertices {
                let _ = writeln!(
                    vt,
                    "VT {:.4} {:.4} {:.4} {:.4} {:.4} {:.4} {:.5} {:.5}",
                    -v.pos[0] * scale + 0.0,
                    v.pos[1] * scale + opts.offset_y,
                    -v.pos[2] * scale + 0.0,
                    -v.normal[0] + 0.0,
                    v.normal[1],
                    -v.normal[2] + 0.0,
                    v.uv[0],
                    1.0 - v.uv[1]
                );
            }
            let first = indices.len();
            indices.extend(lm.indices.iter().map(|&i| i + base));
            base += lm.vertices.len() as u32;
            match spans.last_mut() {
                Some(s) if s.0 == gid && s.3 == mat && s.1 + s.2 == first => s.2 += lm.indices.len(),
                _ => spans.push((gid, first, lm.indices.len(), mat)),
            }
        }
        level_spans.push(spans);
    }

    let mut out = String::with_capacity(vt.len() + indices.len() * 8 + extra.len() + 256);
    out.push_str("I\n800\nOBJ\n\n");
    out.push_str(header);
    if let Some(tex) = &opts.texture {
        let _ = writeln!(out, "TEXTURE {tex}");
    }
    if let Some(lit) = &opts.texture_lit {
        let _ = writeln!(out, "TEXTURE_LIT {lit}");
    }
    if let Some(normal) = &opts.texture_normal {
        // No NORMAL_METALNESS: see `write_obj8`'s identical block for why.
        let _ = writeln!(out, "TEXTURE_MAP normal {normal}");
        if let Some(material) = &opts.texture_material {
            let _ = writeln!(out, "TEXTURE_MAP material_gloss {material}");
        }
        out.push_str("GLOBAL_specular 1.0\n");
    } else if let (Some(material), Some(level)) = (&opts.texture_material, opts.glass_specular) {
        let _ = writeln!(out, "TEXTURE_MAP material_gloss {material}");
        let _ = writeln!(out, "GLOBAL_specular {level:.2}");
    }
    let _ = writeln!(out, "POINT_COUNTS {} 0 0 {}\n", base, indices.len());
    out.push_str(&vt);
    out.push('\n');
    let mut chunks = indices.chunks_exact(10);
    for c in chunks.by_ref() {
        let _ = writeln!(
            out,
            "IDX10 {} {} {} {} {} {} {} {} {} {}",
            c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7], c[8], c[9]
        );
    }
    for i in chunks.remainder() {
        let _ = writeln!(out, "IDX {i}");
    }
    out.push('\n');

    if opts.draw_disable {
        out.push_str("ATTR_draw_disable\n");
    }

    for (level, spans) in levels.iter().zip(&level_spans) {
        if ranged {
            let far = if level.far.is_finite() { level.far } else { 100_000.0 };
            let _ = writeln!(out, "ATTR_LOD {:.0} {:.0}", level.near, far);
        }
        // Fresh per level: each ATTR_LOD section is self-contained, so the
        // first TRIS in it cannot rely on a state change written in a
        // previous section.
        let mut st = AttrState::default();
        let mut i = 0;
        while i < spans.len() {
            let key = &keys[spans[i].0];
            if !key.commands.is_empty() {
                out.push_str("ANIM_begin\n");
                out.push_str(&key.commands);
            }
            if key.hidden {
                out.push_str("ATTR_draw_disable\n");
            }
            if let Some(m) = &key.manip {
                out.push_str(m);
                out.push('\n');
            }
            if let Some(l) = key.light.as_ref().filter(|_| key.device.is_none()) {
                let _ = writeln!(out, "ATTR_light_level {l}");
            }
            if let Some(d) = &key.device {
                let _ = writeln!(out, "ATTR_cockpit_device {d}");
            }
            let gid = spans[i].0;
            while i < spans.len() && spans[i].0 == gid {
                let (_, first, count, mat) = spans[i];
                let m = model.materials.get(mat).cloned().unwrap_or_default();
                material_attrs(&mut out, &m, &mut st, key.click, false);
                let _ = writeln!(out, "TRIS {first} {count}");
                i += 1;
            }
            if key.manip.is_some() {
                out.push_str("ATTR_manip_none\n");
            }
            if key.light.is_some() && key.device.is_none() {
                out.push_str("ATTR_light_level_reset\n");
            }
            if key.device.is_some() {
                out.push_str("ATTR_no_cockpit\n");
            }
            if key.hidden {
                out.push_str("ATTR_draw_enable\n");
            }
            if !key.commands.is_empty() {
                out.push_str("ANIM_end\n");
            }
        }
    }
    out.push_str(extra);
    out
}

/// Simplify one mesh with meshoptimizer, locking its topological border in
/// place (`SimplifyOptions::LockBorder`) so it does not move -- glTF already
/// splits vertices along UV seams and hard-normal edges, and a vertex that
/// is only ever reached from one side of the index buffer already reads as
/// a border to the algorithm, so seams stay put with no extra bookkeeping
/// here.
///
/// The returned index buffer references a *new*, compacted vertex array
/// (unlike meshopt's own `simplify`, which still points at the original
/// one): every survivor keeps its original position, normal and UV exactly
/// -- the algorithm only removes vertices by collapsing edges onto an
/// existing endpoint, it never computes a new blended one, so there is
/// nothing here to recompute. Compacting away the vertices the simplified
/// index buffer no longer touches is this function's own job (meshopt's
/// docs recommend `optimize_vertex_fetch` for the same reason): an LOD
/// exists to shrink what gets drawn, and shipping every original vertex
/// alongside a shorter index list would undo most of that in the file
/// itself.
///
/// Returns the simplified vertices, the simplified (compacted) indices, and
/// the achieved ratio of triangles kept (`1.0` for an empty or unsimplifiable
/// mesh) -- the caller compares that against the ratio it asked for to know
/// whether this mesh actually reached it.
pub fn simplify_mesh(vertices: &[Vertex], indices: &[u32], ratio: f32) -> (Vec<Vertex>, Vec<u32>, f32) {
    let Some((adapter_bytes, target_count)) = simplify_setup(vertices, indices, ratio) else {
        return (Vec::new(), Vec::new(), 1.0);
    };
    let Ok(adapter) = meshopt::VertexDataAdapter::new(&adapter_bytes, 3 * std::mem::size_of::<f32>(), 0) else {
        return (vertices.to_vec(), indices.to_vec(), 1.0);
    };
    let mut result_error = 0.0f32;
    let simplified = meshopt::simplify(
        indices,
        &adapter,
        target_count,
        // Relative to mesh extent, and meshopt clamps sensible values to
        // 0..1: passing 1.0 leaves the target triangle count, not an error
        // budget, in charge of how far this goes.
        1.0,
        meshopt::SimplifyOptions::LockBorder,
        Some(&mut result_error),
    );
    compact(vertices, indices, simplified)
}

/// [`simplify_mesh`], but ignoring topology altogether
/// (`meshopt::simplify_sloppy`, a voxel/cluster reduction with no edge-collapse
/// or border lock at all) rather than respecting the mesh's own borders and
/// UV seams. Meant only as a fallback past the point a real viewer would
/// ever resolve the difference -- X-Plane's own LOD ranges, not this
/// function, are what keep it from running at any distance a seam crack
/// could actually be seen at -- for the handful of meshes
/// [`simplify_mesh`]'s border lock leaves far short of an aggressive target
/// ratio (typically hard-surface greeble built with no shared vertices
/// between adjacent faces at all, so every vertex already reads as a
/// border and there is nothing left for an edge collapse to do).
pub fn simplify_mesh_sloppy(vertices: &[Vertex], indices: &[u32], ratio: f32) -> (Vec<Vertex>, Vec<u32>, f32) {
    let Some((adapter_bytes, target_count)) = simplify_setup(vertices, indices, ratio) else {
        return (Vec::new(), Vec::new(), 1.0);
    };
    let Ok(adapter) = meshopt::VertexDataAdapter::new(&adapter_bytes, 3 * std::mem::size_of::<f32>(), 0) else {
        return (vertices.to_vec(), indices.to_vec(), 1.0);
    };
    let mut result_error = 0.0f32;
    let simplified = meshopt::simplify_sloppy(indices, &adapter, target_count, 1.0, Some(&mut result_error));
    compact(vertices, indices, simplified)
}

/// The position-only vertex buffer bytes and the target index count both
/// [`simplify_mesh`] and [`simplify_mesh_sloppy`] hand to meshopt. `None`
/// for an empty mesh, which the caller returns as-is (ratio `1.0`, nothing
/// to simplify).
fn simplify_setup(vertices: &[Vertex], indices: &[u32], ratio: f32) -> Option<(Vec<u8>, usize)> {
    if vertices.is_empty() || indices.is_empty() {
        return None;
    }
    let ratio = ratio.clamp(0.0, 1.0);
    let positions: Vec<f32> = vertices.iter().flat_map(|v| v.pos).collect();
    let target_count = (((indices.len() as f64) * ratio as f64 / 3.0).round() as usize) * 3;
    Some((meshopt::typed_to_bytes(&positions).to_vec(), target_count))
}

/// Keep only the vertices `simplified` still references, in first-referenced
/// order, remapped so the result carries no dead vertices, plus the achieved
/// ratio of triangles kept against `original_indices`. Every survivor keeps
/// its original position, normal and UV exactly: meshopt's simplifiers only
/// remove vertices by collapsing an edge onto an existing endpoint, never by
/// computing a new blended one, so there is nothing here to recompute
/// (meshopt's own docs recommend `optimize_vertex_fetch` for the same
/// compaction, for the same reason).
fn compact(vertices: &[Vertex], original_indices: &[u32], simplified: Vec<u32>) -> (Vec<Vertex>, Vec<u32>, f32) {
    let achieved = simplified.len() as f32 / original_indices.len() as f32;
    let mut remap: HashMap<u32, u32> = HashMap::new();
    let mut new_vertices = Vec::new();
    let new_indices: Vec<u32> = simplified
        .iter()
        .map(|&old| {
            *remap.entry(old).or_insert_with(|| {
                new_vertices.push(vertices[old as usize]);
                (new_vertices.len() - 1) as u32
            })
        })
        .collect();
    (new_vertices, new_indices, achieved)
}

fn write_lights(out: &mut String, model: &Model, scale: f32, offset_y: f32) {
    for l in &model.lights {
        // Same axis turn as the vertices; the height offset lifts lights too.
        let (x, y, z) = (
            -l.pos[0] * scale + 0.0,
            l.pos[1] * scale + offset_y,
            -l.pos[2] * scale + 0.0,
        );
        let (dx, dy, dz) = (-l.dir[0] + 0.0, l.dir[1] + 0.0, -l.dir[2] + 0.0);
        // X-Plane's WIDTH is the cosine of half the cone (Laminar's 120 degree
        // ramp floods use 0.5).
        let width = ((l.cone_deg.clamp(0.0, 360.0) as f64) / 2.0).to_radians().cos().max(0.0);
        let [r, g, b] = l.color;
        if l.spill && !l.flashing {
            let _ = writeln!(
                out,
                "LIGHT_PARAM spot_params_sp_pm {x:.3} {y:.3} {z:.3} {r:.3} {g:.3} {b:.3} 1 {:.0}cd {dx:.4} {dy:.4} {dz:.4} {width:.4}",
                l.intensity.max(1.0)
            );
        }
        let _ = writeln!(
            out,
            "LIGHT_PARAM spot_params_bb_pm {x:.3} {y:.3} {z:.3} {r:.3} {g:.3} {b:.3} {:.0}cd {dx:.4} {dy:.4} {dz:.4} {width:.4}",
            (l.intensity / 6.0).max(100.0)
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::glb::{LightPoint, Material, Mesh, Vertex};

    fn model(n_tris: usize, material: Material) -> Model {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();
        for t in 0..n_tris {
            let base = vertices.len() as u32;
            for k in 0..3 {
                vertices.push(Vertex {
                    pos: [t as f32, k as f32, 0.0],
                    normal: [0.0, 0.0, 1.0],
                    uv: [0.0, 0.25],
                });
            }
            indices.extend([base, base + 1, base + 2]);
        }
        Model {
            meshes: vec![Mesh {
                name: "m".into(),
                material: 0,
                vertices,
                indices,
                node: None,
            }],
            materials: vec![material],
            warnings: vec![],
            lights: vec![],
            ..Default::default()
        }
    }

    #[test]
    fn models_are_turned_to_xplane_axes_and_lifted() {
        // Two triangles: the second sits at x = 1 in model space.
        let m = model(
            2,
            Material {
                base_color: Some("a.dds".into()),
                ..Default::default()
            },
        );
        let s = write_obj8(
            &m,
            &[0],
            &ObjOptions {
                texture: None,
                scale: 1.0,
                offset_y: 5.0,
                texture_lit: None,
                lights: false,
                texture_normal: None,
                texture_material: None,
                draw_disable: false,
                glass_specular: None,
                lod_far: None,
            },
        );
        // Model +X becomes object -X, and the height offset lifts every vertex.
        assert!(s.contains("VT -1.0000 6.0000 0.0000 0.0000 0.0000 -1.0000"), "{s}");
        assert!(!s.contains("VT -0.0000"), "no negative zeros: {s}");
    }

    #[test]
    fn writes_night_texture_and_lights() {
        let mut m = model(
            1,
            Material {
                base_color: Some("a.dds".into()),
                ..Default::default()
            },
        );
        m.lights.push(LightPoint {
            pos: [1.0, 10.0, 2.0],
            dir: [0.0, -1.0, 1.0],
            color: [1.0, 1.0, 1.0],
            intensity: 7500.0,
            cone_deg: 120.0,
            spill: true,
            flashing: false,
        });
        let s = write_obj8(
            &m,
            &[0],
            &ObjOptions {
                texture: Some("t/a.dds".into()),
                scale: 1.0,
                offset_y: 0.0,
                texture_lit: Some("t/a_lit.dds".into()),
                lights: true,
                texture_normal: Some("t/a_nm.png".into()),
                texture_material: Some("t/a_mat.png".into()),
                draw_disable: false,
                glass_specular: None,
                lod_far: None,
            },
        );
        assert!(
            s.contains("TEXTURE t/a.dds\nTEXTURE_LIT t/a_lit.dds\nTEXTURE_MAP normal t/a_nm.png\nTEXTURE_MAP material_gloss t/a_mat.png\nGLOBAL_specular 1.0\nPOINT_COUNTS"),
            "{s}"
        );
        assert!(!s.contains("NORMAL_METALNESS"), "not this converter's data (its normal map has no alpha and a zero blue channel): {s}");
        assert!(
            s.contains("LIGHT_PARAM spot_params_sp_pm -1.000 10.000 -2.000 1.000 1.000 1.000 1 7500cd 0.0000 -1.0000 -1.0000 0.5000"),
            "{s}"
        );
        assert!(s.contains("LIGHT_PARAM spot_params_bb_pm -1.000 10.000 -2.000"), "{s}");
    }

    #[test]
    fn normal_map_without_a_material_map_writes_no_normal_metalness_either() {
        let m = model(
            1,
            Material {
                base_color: Some("a.dds".into()),
                ..Default::default()
            },
        );
        let s = write_obj8(
            &m,
            &[0],
            &ObjOptions {
                texture: Some("t/a.dds".into()),
                scale: 1.0,
                offset_y: 0.0,
                texture_lit: None,
                lights: false,
                texture_normal: Some("t/a_nm.png".into()),
                texture_material: None,
                draw_disable: false,
                glass_specular: None,
                lod_far: None,
            },
        );
        assert!(s.contains("TEXTURE_MAP normal t/a_nm.png\nGLOBAL_specular 1.0\n"), "{s}");
        assert!(!s.contains("NORMAL_METALNESS"), "{s}");
    }

    /// Display glass (and cabin windows) have no relief to bump-map, so
    /// there is no normal map -- but they still get a faint, flat
    /// `GLOBAL_specular` off their own material map, per `write_obj8_animated`.
    #[test]
    fn glass_without_a_normal_map_gets_a_flat_specular() {
        let m = model(
            1,
            Material {
                base_color: Some("a.dds".into()),
                ..Default::default()
            },
        );
        let s = write_obj8_animated(
            &m,
            &[0],
            &ObjOptions {
                texture: Some("t/glass.dds".into()),
                texture_material: Some("t/glass_mat.png".into()),
                glass_specular: Some(0.5),
                ..Default::default()
            },
            &|_| MeshAnim::default(),
            "",
            "GLOBAL_cockpit_lit\nBLEND_GLASS\n",
        );
        assert!(
            s.contains("TEXTURE t/glass.dds\nTEXTURE_MAP material_gloss t/glass_mat.png\nGLOBAL_specular 0.50\nPOINT_COUNTS"),
            "{s}"
        );
        assert!(!s.contains("TEXTURE_MAP normal"), "no normal map to pair it with:\n{s}");
    }

    /// Without `glass_specular` set, a bare `texture_material` (should not
    /// happen from `main.rs`, but the writer must not invent a specular
    /// level on its own) draws with no shine at all, same as before this fix.
    #[test]
    fn a_material_map_with_no_glass_specular_writes_nothing() {
        let m = model(
            1,
            Material {
                base_color: Some("a.dds".into()),
                ..Default::default()
            },
        );
        let s = write_obj8_animated(
            &m,
            &[0],
            &ObjOptions {
                texture: Some("t/a.dds".into()),
                texture_material: Some("t/a_mat.png".into()),
                ..Default::default()
            },
            &|_| MeshAnim::default(),
            "",
            "",
        );
        assert!(!s.contains("GLOBAL_specular"), "{s}");
        assert!(!s.contains("material_gloss"), "{s}");
    }

    #[test]
    fn glowing_materials_get_their_own_object() {
        let mut m = model(
            1,
            Material {
                base_color: Some("a.dds".into()),
                ..Default::default()
            },
        );
        m.materials.push(Material {
            base_color: Some("a.dds".into()),
            emissive: Some("a.dds".into()),
            emissive_factor: [1.0, 1.0, 1.0],
            ..Default::default()
        });
        let mut second = m.meshes[0].clone();
        second.material = 1;
        m.meshes.push(second);
        let groups = split_by_texture(&m, false);
        assert_eq!(groups.len(), 2, "same base texture, different night texture");
        assert!(groups.iter().any(|g| g.0.lit.as_deref() == Some("a.dds") && g.1 == vec![1]));
        assert!(groups.iter().any(|g| g.0.lit.is_none() && g.1 == vec![0]));
    }

    #[test]
    fn glow_factor_tints_and_dims_the_night_texture_like_the_day_ones_base_colour() {
        // FlyByWire's A380: the windshield-frame glow carries emissiveFactor
        // [0.7, 0.7, 0.7] (should dim, not draw at full brightness), and the
        // pushbutton legends carry [0.78, 0.85, 1.00] (a faint blue cast).
        let dimmed = Material {
            emissive: Some("frame_emis.dds".into()),
            emissive_factor: [0.7, 0.7, 0.7],
            ..Default::default()
        };
        let groups = split_by_texture(&model(1, dimmed), false);
        assert_eq!(groups[0].0.lit.as_deref(), Some("frame_emis.dds"), "still glows: every channel is above zero");
        assert_eq!(groups[0].0.lit_tint, Some([218, 218, 218]), "sRGB-encoded 0.7, same transfer as base colour's tint");

        let tinted = Material {
            emissive: Some("legend_emis.dds".into()),
            emissive_factor: [0.7792515, 0.8499762, 1.0],
            ..Default::default()
        };
        let groups = split_by_texture(&model(1, tinted), false);
        assert_eq!(groups[0].0.lit_tint, Some([228, 237, 255]), "uneven channels are a colour cast, not just dimming");

        let white = Material {
            emissive: Some("plain_emis.dds".into()),
            emissive_factor: [1.0, 1.0, 1.0],
            ..Default::default()
        };
        let groups = split_by_texture(&model(1, white), false);
        assert_eq!(groups[0].0.lit_tint, None, "plain white changes no texel, so it is not worth a second copy");
    }

    #[test]
    fn levels_of_detail_become_distance_ranges() {
        let mut near = model(2, Material::default());
        near.lights.push(LightPoint {
            pos: [0.0, 10.0, 0.0],
            dir: [0.0, -1.0, 0.0],
            color: [1.0, 1.0, 1.0],
            intensity: 600.0,
            cone_deg: 120.0,
            spill: true,
            flashing: false,
        });
        let far = model(1, Material::default());
        let s = write_obj8_lods(
            &[
                LodPart {
                    model: &near,
                    meshes: vec![0],
                    near: 0.0,
                    far: 40.0,
                },
                LodPart {
                    model: &far,
                    meshes: vec![0],
                    near: 40.0,
                    far: 400.0,
                },
            ],
            &ObjOptions {
                lights: true,
                ..Default::default()
            },
        );
        assert!(s.contains("POINT_COUNTS 9 0 0 9"), "{s}");
        let near_at = s.find("ATTR_LOD 0 40\n").expect("near range");
        let far_at = s.find("ATTR_LOD 40 400\n").expect("far range");
        let near_tris = s.find("TRIS 0 6\n").expect("near triangles");
        let far_tris = s.find("TRIS 6 3\n").expect("far triangles");
        assert!(near_at < near_tris && near_tris < far_at && far_at < far_tris, "{s}");
        assert_eq!(s.matches("spot_params_bb_pm").count(), 2, "lights repeat per level");
        assert!(!write_obj8(&near, &[0], &ObjOptions::default()).contains("ATTR_LOD"));
    }

    /// A knob-sized batch casts no shadow; a panel-sized one still does.
    #[test]
    fn a_part_smaller_than_the_shadow_threshold_casts_no_shadow() {
        let m = model(2, Material::default()); // two triangles, about 2 m across
        let panel = write_obj8_animated(&m, &[0], &ObjOptions::default(), &|_| MeshAnim::default(), "", "");
        assert!(!panel.contains("ATTR_no_shadow"), "a panel-sized part must still cast: {panel}");
        let knob = ObjOptions { scale: 0.02, ..Default::default() }; // the same, about 4 cm across
        let small = write_obj8_animated(&m, &[0], &knob, &|_| MeshAnim::default(), "", "");
        let no_shadow = small.find("ATTR_no_shadow\n").expect("a knob-sized part casts no shadow");
        assert!(no_shadow < small.find("TRIS 0 6\n").expect("triangles"), "{small}");
    }

    /// `write_obj8_animated` draws exactly one level of detail (there is no
    /// lower-poly stand-in for the tire mesh to hand off to), so `lod_far`
    /// writes a single `ATTR_LOD 0 <far>` ahead of its triangles rather than
    /// `write_obj8_lods`'s multi-part chain, and geometry past `far` is
    /// simply never drawn.
    #[test]
    fn lod_far_stops_the_object_being_drawn_past_it() {
        let m = model(2, Material::default());
        let opts = ObjOptions {
            lod_far: Some(500.0),
            ..Default::default()
        };
        let obj = write_obj8_animated(&m, &[0], &opts, &|_| MeshAnim::default(), "", "");
        let lod_at = obj.find("ATTR_LOD 0 500\n").expect("lod range");
        let tris_at = obj.find("TRIS 0 6\n").expect("triangles");
        assert!(lod_at < tris_at, "{obj}");
        // No `lod_far`: every object still draws at every distance, as
        // before this field existed.
        let plain = write_obj8_animated(&m, &[0], &ObjOptions::default(), &|_| MeshAnim::default(), "", "");
        assert!(!plain.contains("ATTR_LOD"), "{plain}");
    }

    #[test]
    fn writes_the_obj8_structure() {
        let m = model(
            1,
            Material {
                base_color: Some("a.dds".into()),
                ..Default::default()
            },
        );
        let s = write_obj8(
            &m,
            &[0],
            &ObjOptions {
                texture: Some("textures/a.dds".into()),
                scale: 2.0,
                offset_y: 0.0,
                texture_lit: None,
                lights: false,
                texture_normal: None,
                texture_material: None,
                draw_disable: false,
                glass_specular: None,
                lod_far: None,
            },
        );
        assert!(s.starts_with("I\n800\nOBJ\n\nTEXTURE textures/a.dds\nPOINT_COUNTS 3 0 0 3\n"));
        assert!(s.contains("VT 0.0000 2.0000 0.0000"), "scale is baked in:\n{s}");
        assert!(s.contains(" 0.00000 0.75000\n"), "V is flipped to T");
        assert!(s.contains("ATTR_cull\nATTR_no_blend 0.00\nATTR_poly_os 0\nTRIS 0 3\n"));
    }

    #[test]
    fn a_faded_decal_keeps_the_alpha_its_own_test_needs() {
        let decal = Material {
            base_color: Some("decal.dds".into()),
            alpha: AlphaMode::Blend,
            base_color_factor: [1.0, 1.0, 1.0, 0.9],
            ..Default::default()
        };
        let groups = split_by_texture(&model(1, decal), false);
        assert!(groups[0].0.blend, "a blended non-glass material is a decal");
        assert_eq!(groups[0].0.alpha, None, "its material's fade is not baked into the alpha the test reads");

        let glass = Material {
            base_color: Some("glass.dds".into()),
            alpha: AlphaMode::Blend,
            glass: true,
            base_color_factor: [1.0, 1.0, 1.0, 0.9],
            ..Default::default()
        };
        let groups = split_by_texture(&model(1, glass), false);
        assert!(groups[0].0.alpha.is_some(), "glass, which really is blended, still fades in its texture");
    }

    #[test]
    fn a_blended_decals_msfs_opacity_bakes_into_blend_alpha_not_alpha() {
        // FlyByWire's A380_DETAILS_RIVETS material (texture
        // A380_DETAIL_RIBBON01_ALBEDO): `ASOBO_material_blend_gbuffer`'s
        // `baseColorBlendFactor: 0.13`, already folded into
        // `base_color_factor[3]` by `glb::material()` before this struct
        // ever sees it.
        let rivets = Material {
            base_color: Some("rivets.dds".into()),
            alpha: AlphaMode::Blend,
            base_color_factor: [1.0, 1.0, 1.0, 0.13],
            ..Default::default()
        };
        let groups = split_by_texture(&model(1, rivets), false);
        assert!(groups[0].0.blend, "a blended non-glass material is a decal");
        assert_eq!(groups[0].0.blend_alpha, Some(13), "MSFS blends this decal's colour at 13%");
        assert_eq!(groups[0].0.alpha, None, "a decal's fade is `blend_alpha`, not glass's `alpha`");

        // `{}` -- every ASOBO_material_blend_gbuffer factor at its own
        // default (baseColorBlendFactor 1.0) -- leaves the decal's alpha
        // unbaked, same as a material carrying no blend extension at all.
        let unblended = Material {
            base_color: Some("decal.dds".into()),
            alpha: AlphaMode::Blend,
            base_color_factor: [1.0, 1.0, 1.0, 1.0],
            ..Default::default()
        };
        assert_eq!(split_by_texture(&model(1, unblended), false)[0].0.blend_alpha, None, "an unmarked factor of 1.0 needs no second copy of the texture");

        // Glass keeps fading through `alpha` (its own translucent pass);
        // `blend_alpha` stays empty for it even at the same 0.13 factor.
        let glass = Material {
            base_color: Some("glass.dds".into()),
            alpha: AlphaMode::Blend,
            glass: true,
            base_color_factor: [1.0, 1.0, 1.0, 0.13],
            ..Default::default()
        };
        assert_eq!(split_by_texture(&model(1, glass), false)[0].0.blend_alpha, None, "glass fades through `alpha`, not `blend_alpha`");
    }

    #[test]
    fn masked_materials_get_their_own_cutoff_not_the_decal_pass() {
        // A380_COCKPIT_STICKER_8k-style material: glTF MASK, its own cutoff,
        // no ASOBO decal markers at all.
        let masked = Material {
            base_color: Some("sticker.dds".into()),
            alpha: AlphaMode::Mask,
            alpha_cutoff: 0.3,
            ..Default::default()
        };
        let groups = split_by_texture(&model(1, masked), false);
        assert!(!groups[0].0.blend, "masked cutouts are not routed through the decal/dilation path");
        assert_eq!(
            groups[0].0.mask_cutoff,
            Some(77),
            "0.3 of 255, rounded, not the decal convention's fixed 0.90 (230)"
        );

        // The default cutoff (glTF's 0.5) rounds to 128, distinct from the
        // decal path's DECAL_ALPHA_CUTOFF (230).
        let default_cutoff = Material {
            base_color: Some("sticker.dds".into()),
            alpha: AlphaMode::Mask,
            ..Default::default()
        };
        let groups = split_by_texture(&model(1, default_cutoff), false);
        assert_eq!(groups[0].0.mask_cutoff, Some(128));
        assert_ne!(groups[0].0.mask_cutoff, Some(DECAL_ALPHA_CUTOFF));

        // Opaque and blended-decal materials carry no mask cutoff at all.
        let opaque = Material {
            base_color: Some("panel.dds".into()),
            ..Default::default()
        };
        assert_eq!(split_by_texture(&model(1, opaque), false)[0].0.mask_cutoff, None);
        let decal = Material {
            base_color: Some("decal.dds".into()),
            alpha: AlphaMode::Blend,
            ..Default::default()
        };
        assert_eq!(split_by_texture(&model(1, decal), false)[0].0.mask_cutoff, None);
    }

    #[test]
    fn a_decal_blends_rather_than_being_alpha_tested() {
        // A hard test cannot survive a mip: on the A380's decal atlas the
        // share of texels above 0.90 falls from 20.4% at full size to 3.8%
        // at 256, so a placard reads blank at head distance and only comes
        // back when you lean into it. Blending draws a stroke thinner than
        // a texel as the partial coverage it is.
        assert_eq!(DECAL_ALPHA_TEST, "ATTR_blend");
        let m = Material { base_color: Some("decal.dds".into()), alpha: AlphaMode::Blend, ..Default::default() };
        let s = write_obj8(&model(1, m), &[0], &ObjOptions::default());
        assert!(s.contains("
ATTR_blend
"), "{s}");
        assert!(!s.contains("ATTR_no_blend 0.90"), "{s}");
        // Glass still blends as glass, in its own pass.
        let g = Material { base_color: Some("g.dds".into()), alpha: AlphaMode::Blend, glass: true, ..Default::default() };
        assert!(write_obj8(&model(1, g), &[0], &ObjOptions::default()).contains("
ATTR_blend
"));
        // The cutoff byte still governs what the texture converter keeps a
        // decal's own alpha above when it floods the background colour in.
        assert_eq!(DECAL_ALPHA_CUTOFF, (DECAL_ALPHA_RATIO * 255.0).ceil() as u8);
    }

    /// A touch screen's manipulator must sit on its own triangles, tagged
    /// with `ATTR_cockpit_device`, both before the `TRIS` they cover
    /// (developer.x-plane.com/article/obj8-file-format-specification: "the
    /// manipulator must also be tagged with ATTR_cockpit_device"), and both
    /// closed off afterwards so they do not leak onto unrelated meshes.
    #[test]
    fn a_touch_screen_carries_its_manipulator_and_device_together() {
        let m = model(1, Material { base_color: Some("a.dds".into()), ..Default::default() });
        let anim = |_: usize| MeshAnim {
            manip: Some("ATTR_manip_device hand SCREEN_EFB SCREEN_EFB".into()),
            device: Some("SCREEN_EFB 0 0 0".into()),
            ..Default::default()
        };
        let s = write_obj8_animated(&m, &[0], &ObjOptions::default(), &anim, "", "GLOBAL_cockpit_lit\n");
        let manip_at = s.find("ATTR_manip_device hand SCREEN_EFB SCREEN_EFB\n").expect("manipulator: {s}");
        let device_at = s.find("ATTR_cockpit_device SCREEN_EFB 0 0 0\n").expect("device: {s}");
        let tris_at = s.find("TRIS 0 3\n").expect("tris: {s}");
        assert!(manip_at < tris_at && device_at < tris_at, "both attrs precede the triangles they cover:\n{s}");
        assert!(s.contains("ATTR_manip_none\n"), "manipulator is closed: {s}");
        assert!(s.contains("ATTR_no_cockpit\n"), "device is closed: {s}");
    }

    /// A screen with no manipulator (the read-only majority: PFD, ND, EWD,
    /// SD, ISIS, clock, RMPs, FCU) still gets its device, just no manip
    /// lines at all -- there is nothing here for the OBJ8 spec's pairing
    /// requirement to apply to.
    #[test]
    fn a_read_only_screen_gets_no_manipulator() {
        let m = model(1, Material { base_color: Some("a.dds".into()), ..Default::default() });
        let anim = |_: usize| MeshAnim { device: Some("SCREEN_DU_PFDL 0 0 0".into()), ..Default::default() };
        let s = write_obj8_animated(&m, &[0], &ObjOptions::default(), &anim, "", "GLOBAL_cockpit_lit\n");
        assert!(s.contains("ATTR_cockpit_device SCREEN_DU_PFDL 0 0 0\n"), "{s}");
        assert!(!s.contains("ATTR_manip_"), "{s}");
    }

    #[test]
    fn indices_come_in_tens_then_singles() {
        let m = model(9, Material::default()); // 27 indices
        let s = write_obj8(&m, &[0], &ObjOptions::default());
        assert_eq!(s.lines().filter(|l| l.starts_with("IDX10 ")).count(), 2);
        assert_eq!(s.lines().filter(|l| l.starts_with("IDX ")).count(), 7);
    }

    #[test]
    fn material_state_maps_to_attributes() {
        let m = model(
            1,
            Material {
                double_sided: true,
                alpha: AlphaMode::Mask,
                alpha_cutoff: 0.3,
                decal: true,
                ..Default::default()
            },
        );
        let s = write_obj8(&m, &[0], &ObjOptions::default());
        // Decals get no polygon offset (X-Plane would drape them); they are
        // lifted off their surface instead.
        assert!(s.contains("ATTR_no_cull\nATTR_no_blend 0.30\nATTR_poly_os 0\n"), "{s}");
    }

    #[test]
    fn untextured_materials_keep_their_colour() {
        let mut m = model(1, Material::default());
        m.materials = vec![
            Material {
                base_color_factor: [0.0, 0.0, 0.0, 1.0],
                ..Default::default()
            },
            Material {
                name: "$SCREEN_DU_PFDL".into(),
                ..Default::default()
            },
            Material {
                base_color: Some("a.dds".into()),
                ..Default::default()
            },
        ];
        for i in 1..3 {
            let mut other = m.meshes[0].clone();
            other.material = i;
            m.meshes.push(other);
        }
        let groups = split_by_texture(&m, false);
        let solid = |mesh: usize| groups.iter().find(|g| g.1.contains(&mesh)).unwrap().0.solid;
        assert_eq!(solid(0), Some([0, 0, 0, 255]), "a black material stays black");
        assert_eq!(solid(1), Some([0, 0, 0, 255]), "a gauge screen is dark, not white");
        assert_eq!(solid(2), None, "a textured material uses its texture");
    }

    #[test]
    fn meshes_split_by_texture() {
        let mut m = model(1, Material::default());
        m.materials = vec![
            Material {
                base_color: Some("b.dds".into()),
                ..Default::default()
            },
            Material {
                base_color: Some("a.dds".into()),
                ..Default::default()
            },
        ];
        let mut second = m.meshes[0].clone();
        second.material = 1;
        m.meshes.push(second);
        let groups = split_by_texture(&m, false);
        assert_eq!(groups.len(), 2);
        assert_eq!(groups[0].0.base.as_deref(), Some("a.dds"));
        assert_eq!(groups[0].1, vec![1]);
        // With normal maps on, a different normal map splits the same base texture.
        m.materials[1].normal = Some("n.dds".into());
        m.materials[0].base_color = Some("a.dds".into());
        let groups = split_by_texture(&m, true);
        assert_eq!(groups.len(), 2);
        assert!(groups.iter().any(|g| g.0.normal.as_deref() == Some("n.dds")));
        assert_eq!(split_by_texture(&m, false).len(), 1, "without normal maps they share an object");
    }

    #[test]
    fn a_real_glb_round_trips_to_obj8() {
        let glb = crate::model::glb::tests::asobo_triangle("");
        let model = crate::model::load_glb(&glb).unwrap();
        let groups = split_by_texture(&model, false);
        let s = write_obj8(&model, &groups[0].1, &ObjOptions::default());
        assert!(s.contains("POINT_COUNTS 3 0 0 3"));
        assert!(s.contains("ATTR_no_cull"));
    }

    // -- Exterior LOD (`simplify_mesh`, `write_obj8_animated_lods`) --------

    /// A flat `n`x`n` grid in the XZ plane: every interior vertex is
    /// perfectly coplanar with its neighbours, so meshoptimizer's quadric
    /// error for collapsing them is exactly zero and it is free to chase the
    /// requested triangle count almost exactly, without `LockBorder`
    /// fighting it anywhere but the outer edge. That makes it a fair mesh to
    /// measure the *ratio* against: a real fuselage panel would fall short
    /// of the target by more, not less, than this does.
    fn grid_mesh(n: usize) -> (Vec<Vertex>, Vec<u32>) {
        let mut vertices = Vec::new();
        for z in 0..n {
            for x in 0..n {
                vertices.push(Vertex {
                    pos: [x as f32, 0.0, z as f32],
                    normal: [0.0, 1.0, 0.0],
                    uv: [x as f32 / (n - 1) as f32, z as f32 / (n - 1) as f32],
                });
            }
        }
        let mut indices = Vec::new();
        for z in 0..n - 1 {
            for x in 0..n - 1 {
                let a = (z * n + x) as u32;
                let b = a + 1;
                let c = a + n as u32;
                let d = c + 1;
                indices.extend([a, c, b, b, c, d]);
            }
        }
        (vertices, indices)
    }

    #[test]
    fn simplify_ratio_lands_within_tolerance() {
        let (vertices, indices) = grid_mesh(24); // 23*23*2 = 1058 triangles
        let want = 0.25;
        let (new_vertices, new_indices, ratio) = simplify_mesh(&vertices, &indices, want);
        assert!(!new_indices.is_empty(), "a mesh this size must simplify to something");
        assert_eq!(new_indices.len() % 3, 0, "a whole number of triangles");
        assert!((ratio - want).abs() < 0.08, "achieved ratio {ratio}, wanted {want}");
        assert_eq!(ratio, new_indices.len() as f32 / indices.len() as f32, "ratio matches the returned buffers");
        // Recomputes nothing: every surviving vertex is byte-identical to
        // one the original mesh had, not a new blended position.
        for v in &new_vertices {
            assert!(vertices.contains(v), "{v:?} is not one of the original vertices");
        }
        // No dead vertices: the compacted array is not just the original one.
        assert!(new_vertices.len() < vertices.len());
        for &i in &new_indices {
            assert!((i as usize) < new_vertices.len());
        }
    }

    #[test]
    fn simplify_mesh_handles_the_empty_case() {
        let (vertices, _) = grid_mesh(4);
        let (v, i, ratio) = simplify_mesh(&vertices, &[], 0.25);
        assert!(v.is_empty() && i.is_empty());
        assert_eq!(ratio, 1.0);
        let (v, i, ratio) = simplify_mesh(&[], &[0, 1, 2], 0.25);
        assert!(v.is_empty() && i.is_empty());
        assert_eq!(ratio, 1.0);
    }

    /// Parse every `ATTR_LOD near far` line out of a written object, in the
    /// order it appears.
    fn attr_lod_ranges(s: &str) -> Vec<(f32, f32)> {
        s.lines()
            .filter_map(|l| l.strip_prefix("ATTR_LOD "))
            .filter_map(|rest| {
                let mut it = rest.split_whitespace();
                Some((it.next()?.parse().ok()?, it.next()?.parse().ok()?))
            })
            .collect()
    }

    #[test]
    fn lod_chain_ranges_are_consecutive() {
        let m = model(2, Material::default());
        let mid = model(1, Material::default());
        let far = model(1, Material::default());
        let levels = vec![
            AnimLodLevel { near: 0.0, far: 400.0, meshes: vec![LodMesh { mesh: 0, vertices: &m.meshes[0].vertices, indices: &m.meshes[0].indices }] },
            AnimLodLevel { near: 400.0, far: 1500.0, meshes: vec![LodMesh { mesh: 0, vertices: &mid.meshes[0].vertices, indices: &mid.meshes[0].indices }] },
            AnimLodLevel {
                near: 1500.0,
                far: f32::INFINITY,
                meshes: vec![LodMesh { mesh: 0, vertices: &far.meshes[0].vertices, indices: &far.meshes[0].indices }],
            },
        ];
        let s = write_obj8_animated_lods(&m, &levels, &ObjOptions::default(), &|_| MeshAnim::default(), "", "");
        let ranges = attr_lod_ranges(&s);
        assert_eq!(ranges, vec![(0.0, 400.0), (400.0, 1500.0), (1500.0, 100_000.0)], "{s}");
        for w in ranges.windows(2) {
            assert_eq!(w[0].1, w[1].0, "each level's far is the next one's near: {s}");
        }
    }

    /// A single level with an infinite far range needs no `ATTR_LOD` line at
    /// all -- `--no-exterior-lods` must be indistinguishable from a build
    /// that never had this feature.
    #[test]
    fn a_single_infinite_level_writes_no_attr_lod() {
        let m = model(2, Material::default());
        let levels = vec![AnimLodLevel { near: 0.0, far: f32::INFINITY, meshes: vec![LodMesh { mesh: 0, vertices: &m.meshes[0].vertices, indices: &m.meshes[0].indices }] }];
        let s = write_obj8_animated_lods(&m, &levels, &ObjOptions::default(), &|_| MeshAnim::default(), "", "");
        assert!(!s.contains("ATTR_LOD"), "{s}");
    }

    /// [`write_obj8_animated_lods`] with one infinite level must be
    /// byte-for-byte what [`write_obj8_animated`] writes for the same
    /// meshes: `--no-exterior-lods` (and every excluded object: glass,
    /// decals, tyres, anything with a manipulator) routes through the plain
    /// function, but the grouping/sorting/attribute-state logic living in
    /// both must never quietly drift apart.
    #[test]
    fn a_single_level_matches_write_obj8_animated() {
        let mut m = model(2, Material { base_color: Some("a.dds".into()), ..Default::default() });
        m.materials.push(Material { base_color: Some("b.dds".into()), alpha: AlphaMode::Blend, ..Default::default() });
        let mut second = Mesh { name: "b".into(), material: 1, ..m.meshes[0].clone() };
        second.vertices.truncate(3);
        second.indices.truncate(3);
        m.meshes.push(second);
        let anim = |mi: usize| {
            if mi == 1 {
                MeshAnim { commands: "ANIM_trans 0 0 0 1 0 0 0 1 sim/none\n".into(), ..Default::default() }
            } else {
                MeshAnim::default()
            }
        };
        let header = "GLOBAL_cockpit_lit\n";
        let plain = write_obj8_animated(&m, &[0, 1], &ObjOptions::default(), &anim, "extra\n", header);
        let level0: Vec<LodMesh> = [0usize, 1]
            .iter()
            .map(|&mi| LodMesh { mesh: mi, vertices: &m.meshes[mi].vertices, indices: &m.meshes[mi].indices })
            .collect();
        let levels = vec![AnimLodLevel { near: 0.0, far: f32::INFINITY, meshes: level0 }];
        let lods = write_obj8_animated_lods(&m, &levels, &ObjOptions::default(), &anim, "extra\n", header);
        assert_eq!(plain, lods);
    }

    /// Every `ANIM_begin` in a written object must have a matching
    /// `ANIM_end`, in every level, whether or not any mesh was simplified
    /// away to nothing there.
    #[test]
    fn anim_begin_end_counts_balance_in_every_level() {
        let mut m = model(4, Material::default());
        let flap = Mesh { name: "flap".into(), material: 0, ..m.meshes[0].clone() };
        m.meshes.push(flap);
        let anim = |mi: usize| {
            if mi == 1 {
                MeshAnim { commands: "ANIM_rotate 0 1 0 0 20 flap_pos\n".into(), ..Default::default() }
            } else {
                MeshAnim::default()
            }
        };
        // L0: both meshes full detail. L1: the fuselage (mesh 0) simplified,
        // the flap (mesh 1) simplified away to nothing -- its animation
        // wrapper must simply not appear at L1, balanced or not.
        let (fuse_v, fuse_i, _) = simplify_mesh(&m.meshes[0].vertices, &m.meshes[0].indices, 0.5);
        let empty_v: Vec<Vertex> = Vec::new();
        let empty_i: Vec<u32> = Vec::new();
        let levels = vec![
            AnimLodLevel {
                near: 0.0,
                far: 400.0,
                meshes: vec![
                    LodMesh { mesh: 0, vertices: &m.meshes[0].vertices, indices: &m.meshes[0].indices },
                    LodMesh { mesh: 1, vertices: &m.meshes[1].vertices, indices: &m.meshes[1].indices },
                ],
            },
            AnimLodLevel {
                near: 400.0,
                far: f32::INFINITY,
                meshes: vec![LodMesh { mesh: 0, vertices: &fuse_v, indices: &fuse_i }, LodMesh { mesh: 1, vertices: &empty_v, indices: &empty_i }],
            },
        ];
        let s = write_obj8_animated_lods(&m, &levels, &ObjOptions::default(), &anim, "", "");
        assert_eq!(s.matches("ANIM_begin").count(), s.matches("ANIM_end").count(), "{s}");
        // L0 still has the flap's animation; L1 (past the mesh's own LOD
        // range) has none left to balance against triangles that no longer
        // exist there.
        let l0 = &s[..s.find("ATTR_LOD 400").unwrap()];
        let l1 = &s[s.find("ATTR_LOD 400").unwrap()..];
        assert_eq!(l0.matches("ANIM_begin").count(), 1, "{s}");
        assert_eq!(l1.matches("ANIM_begin").count(), 0, "the flap has no geometry left at L1: {s}");
    }

    /// An animated part that still has *some* triangles at a farther level
    /// keeps its animation there too -- a control surface, gear leg or fan
    /// blade must keep moving at distance, not freeze because its own
    /// simplified geometry landed in a later `ATTR_LOD` section.
    #[test]
    fn an_animated_part_keeps_its_animation_wherever_it_has_triangles() {
        let (grid_v, grid_i) = grid_mesh(10);
        let m = Model {
            meshes: vec![Mesh { name: "surface".into(), material: 0, vertices: grid_v, indices: grid_i, node: None }],
            materials: vec![Material::default()],
            ..Default::default()
        };
        let anim = |_: usize| MeshAnim { commands: "ANIM_rotate 0 1 0 0 25 aileron_pos\n".into(), ..Default::default() };
        let (l1_v, l1_i, ratio) = simplify_mesh(&m.meshes[0].vertices, &m.meshes[0].indices, 0.25);
        assert!(!l1_i.is_empty(), "a 10x10 grid must still have triangles at 25%: ratio {ratio}");
        let levels = vec![
            AnimLodLevel { near: 0.0, far: 400.0, meshes: vec![LodMesh { mesh: 0, vertices: &m.meshes[0].vertices, indices: &m.meshes[0].indices }] },
            AnimLodLevel { near: 400.0, far: f32::INFINITY, meshes: vec![LodMesh { mesh: 0, vertices: &l1_v, indices: &l1_i }] },
        ];
        let s = write_obj8_animated_lods(&m, &levels, &ObjOptions::default(), &anim, "", "");
        let l0 = &s[..s.find("ATTR_LOD 400").unwrap()];
        let l1 = &s[s.find("ATTR_LOD 400").unwrap()..];
        assert!(l0.contains("ANIM_rotate 0 1 0 0 25 aileron_pos"), "{s}");
        assert!(l1.contains("ANIM_rotate 0 1 0 0 25 aileron_pos"), "the simplified level must still animate: {s}");
        assert_eq!(l0.matches("ANIM_begin").count(), l0.matches("ANIM_end").count());
        assert_eq!(l1.matches("ANIM_begin").count(), l1.matches("ANIM_end").count());
    }
}
