//! msfs2xp-aircraft: an MSFS aircraft package as an X-Plane 12 aircraft.
//!
//! Writes an aircraft folder that X-Plane loads:
//!
//! - `objects/`: every model (LOD 0) as OBJ8 files, one per texture set, with
//!   the MSFS animations as X-Plane keyframe animations (see `rig`), and
//!   every texture of the package in X-Plane's form: colour and night
//!   textures as DXT1/DXT5 DDS in X-Plane's row order (MSFS's BC7 is
//!   re-encoded), normal maps and their MSFS "COMP" texture as X-Plane 12's
//!   separate normal and metal/gloss maps, laid out as Laminar's
//!   (`*_NML.png`, `*_MAT.png`).
//! - `objects/<cockpit>_click.obj`: invisible copies of every clickable
//!   cockpit part, carrying the manipulators; `objects/lights.obj`: the
//!   exterior lights at the MSFS light nodes.
//! - `plugins/sasl/`: SASL 3 (copied from the user's download) running the
//!   generated module that creates the datarefs and drives the exterior.
//! - `liveries/<name>/objects/`: each livery package's textures.
//! - `<name>.acf`: built on a Plane Maker template (see `acf`).
//!
//! Not converted: systems, displays and sounds.

mod acf;
mod behaviour;
mod lights;
// A copy of the converter's model code; not every helper is used here.
#[allow(dead_code)]
mod model;
mod rig;
mod screens;
mod stations;
mod sasl;

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::path::{Path, PathBuf};

use anyhow::{anyhow, bail, Context};
use clap::Parser;
use rayon::prelude::*;

use acf::ObjKind;
use model::{load_glb, simplify_mesh, simplify_mesh_sloppy, write_obj8_animated, write_obj8_animated_lods, AnimLodLevel, Animator, ClipIndex, LodMesh, MeshAnim, Model, ObjOptions};
use msfs2xp::texture::{
    convert_for_xplane_capped, convert_for_xplane_capped_with_ao, decode_within, detect, dilate_transparent, material_png, normal_png, output_extension, FLAT_FLOOR,
    preserve_alpha_coverage_under, tint_rgba, CoverageFootprint, SourceFormat,
};

#[derive(Parser, Debug)]
#[command(version, about = "Convert an MSFS aircraft package into an X-Plane 12 aircraft: models, animations, cockpit clicks, textures, .acf")]
struct Args {
    /// MSFS aircraft package (the folder with manifest.json and SimObjects).
    package: PathBuf,
    /// Folder to write the aircraft into.
    #[arg(short, long)]
    out: PathBuf,
    /// Livery packages to convert into liveries/.
    #[arg(long)]
    livery: Vec<PathBuf>,
    /// Aircraft folder name (default: the package title).
    #[arg(long)]
    name: Option<String>,
    /// Longitudinal centre of gravity, feet in X-Plane's own frame
    /// (negative forward of the datum), replacing the one derived from the
    /// cfg's `empty_weight_cg_position`.
    ///
    /// That derivation is right in principle -- X-Plane adds the payload and
    /// fuel moments to it, so the empty centre of gravity is what belongs
    /// there -- and wrong in practice whenever the converted stations do not
    /// move the balance the way MSFS's own do. On the A380 the cfg gave
    /// -16 ft, which put 16 % of the weight on the nose leg: the aircraft
    /// rode its nose wheel down the runway, hammered the strut hard enough
    /// to throw +/-127 deg/s^2 of pitch acceleration through the airframe,
    /// and would not rotate.
    ///
    /// The certificated figure settles what it should be. Airbus's Aircraft
    /// Characteristics (07-03-00, weight variant WV003) gives the nose gear
    /// a static load of 39 760 kg at the most forward centre of gravity,
    /// 35.81 % MAC, against a 512 000 kg maximum ramp weight -- 7.77 % on
    /// the nose. Over the A380's 99.15 ft wheelbase that places the centre
    /// of gravity 7.70 ft ahead of the main gear, so `--cg-z -8` is the
    /// A380's value and measures 7.8 % in the simulator.
    #[arg(long)]
    cg_z: Option<f64>,
    /// Largest colour texture side.
    #[arg(long, default_value_t = 4096)]
    max_texture: u32,
    /// Largest normal map side. Panel lettering, screws and edges are
    /// embossed in them; smaller, the cockpit shades flat.
    #[arg(long, default_value_t = 4096)]
    normal_max: u32,
    /// Include the passenger cabin (the LOD01 graft, 180k triangles behind
    /// the cockpit door; on by default). Accepted as a no-op for old command
    /// lines: the cabin was opt-in before it became the default. Never read
    /// -- `--no-cabin` is the only flag that changes anything now.
    #[arg(long)]
    #[allow(dead_code)]
    cabin: bool,
    /// Leave the passenger cabin out, as every build did before it became
    /// the default (5 million LOD0 triangles behind the cockpit door,
    /// avoided either way -- this skips the 180k-triangle LOD01 graft too).
    #[arg(long)]
    no_cabin: bool,
    /// Largest side of textures only the cockpit and cabin use, normal maps
    /// included. Panel lettering needs the full 4096.
    #[arg(long, default_value_t = 4096)]
    interior_max: u32,
    /// Largest side of a colour or emissive texture only the passenger
    /// cabin uses (seats, suites, galley, sidewalls, ceiling, stairs -- a
    /// texture referenced only by materials under the LOD01 `a380_cabin`
    /// graft, never by the cockpit): about 242 MB, mostly 4K colour maps,
    /// for a cabin the user sees through doors from the cockpit, never up
    /// close. Normal and material maps follow `--interior-max` and the
    /// usual shrink-to-detail policy instead (see `LETTERED_NORMAL_MAPS`);
    /// this caps only the ones that policy does not already shrink on its
    /// own.
    #[arg(long, default_value_t = 2048)]
    cabin_max: u32,
    /// Largest side of textures the exterior uses. Paint, panel lines and
    /// placards on a 73 m fuselage do not repay 4096 from any distance the
    /// aircraft is actually seen at, and the eye spends its time in the
    /// cockpit: on the A380, capping the exterior at 2048 gave back 565 MB
    /// of video memory, with the interior left at `--interior-max`.
    #[arg(long, default_value_t = 2048)]
    exterior_max: u32,
    /// Largest side of an interior texture of a plain surface
    /// ([`PLAIN_SURFACE_TEXTURES`]: leather, textiles, carpet, cables, plain
    /// wall and ceiling panels, glass, the cabin's seats), colour and normal
    /// maps alike. None of them carries lettering, and at 4096 they held
    /// about 280 MB of video memory on the A380 for grain nobody sees closer
    /// than arm's length.
    #[arg(long, default_value_t = 2048)]
    surface_max: u32,
    /// Largest side of a livery's fuselage-paint and registration textures
    /// (an output name carrying the whole token `FUSE`/`FUSE<n>`, `TAIL`,
    /// `TITLE`/`TITLES` or `REGISTRATION` -- see `is_fuselage_paint`), left
    /// at `--exterior-max` unless set. Unlike the rest of the exterior,
    /// these carry a livery's titles and registration lettering close
    /// enough to the eye (a nose-in gate view, a spotter shot) that source
    /// art is routinely authored past `--exterior-max`: on the A380 the
    /// Emirates livery ships 4096 `FUSE1`-`FUSE5` albedos and the Pride
    /// livery ships 8192 `FUSE3`/`FUSE4`, both crushed to 2048 by the plain
    /// exterior cap (2x and 4x linear). Raising just this costs video
    /// memory only on the textures that carry lettering, not on every gear
    /// bay and engine pylon texture along with them; still bounded above by
    /// `--max-texture`.
    #[arg(long)]
    fuselage_max: Option<u32>,
    /// Also write every texture of the package that no object references
    /// (0.62 GB on the A380), rather than only what the models ask for
    #[arg(long)]
    all_textures: bool,
    /// Only convert textures, not models.
    #[arg(long)]
    no_models: bool,
    /// Skip textures (models, animations and .acf only; for quick rebuilds
    /// over an earlier full conversion).
    #[arg(long)]
    no_textures: bool,
    /// Build the aircraft's .acf on this Plane Maker .acf, a real X-Plane
    /// airliner (e.g. the default Airbus A330-300's A330.acf); its airfoils
    /// folder is copied along.
    #[arg(long)]
    acf_template: Option<PathBuf>,
    /// Folder with flight_model.cfg, engines.cfg and aircraft.cfg (default:
    /// the package's aircraft folder).
    #[arg(long)]
    cfg_dir: Option<PathBuf>,
    /// Maximum operating speed in knots, from the aircraft's documentation.
    /// Default when omitted: FlyByWire's A380X published Vmo, 340 kt
    /// (fbw-a380x/src/systems/shared/src/PerformanceConstants.ts:1), so a
    /// build never silently keeps the `--acf-template`'s own Vmo (Laminar's
    /// A330: 330 kt).
    #[arg(long)]
    vmo: Option<f64>,
    /// Maximum operating Mach number, from the aircraft's documentation.
    /// Default when omitted: FlyByWire's A380X published Mmo, 0.89
    /// (fbw-a380x/src/systems/shared/src/PerformanceConstants.ts:2), so a
    /// build never silently keeps the `--acf-template`'s own Mmo (Laminar's
    /// A330: 0.86).
    #[arg(long)]
    mmo: Option<f64>,
    /// Leading edge of the mean aerodynamic chord, feet along the MSFS
    /// longitudinal axis from the datum, from the aircraft's loadsheet.
    #[arg(long, allow_hyphen_values = true)]
    lemac: Option<f64>,
    /// Mean aerodynamic chord length in feet, from the aircraft's loadsheet.
    #[arg(long)]
    mac: Option<f64>,
    /// SASL 3 (the zip from 1-sim.com, or its unpacked folder), copied into
    /// plugins/sasl to run the animations and cockpit datarefs.
    #[arg(long)]
    sasl: Option<PathBuf>,
    /// The cockpit model XML whose behaviours say what each control does
    /// (default: the package's interior model XML, from model.cfg). FlyByWire's
    /// source tree works too (`A380_Cockpit_Behavior.xml`), if its node names
    /// match the model's.
    #[arg(long)]
    behaviour: Option<PathBuf>,
    /// MSFS's own model behaviour definitions (`fs-base-aircraft-common\
    /// ModelBehaviorDefs`), which say what Asobo's templates do with a
    /// control's emissive codes (legends, backlights). Default: found next to
    /// the package's Community folder (Official\OneStore or Official\Steam).
    #[arg(long)]
    asobo_behaviours: Option<PathBuf>,
    /// Trailing-edge flap angle per handle detent (degrees, comma-separated),
    /// when the systems' differ from flight_model.cfg's (FlyByWire's A380X:
    /// 0,0,8,17,26,33 from a380_systems hydraulic/mod.rs:1735-1739).
    #[arg(long, value_delimiter = ',')]
    flap_degrees: Option<Vec<f64>>,
    /// Slat angle per handle detent (degrees, comma-separated), when the
    /// systems' differ (FlyByWire's A380X: 0,20,20,20,23,23 from
    /// a380_systems hydraulic/mod.rs:1741-1745 and sfcc/slats_channel.rs).
    #[arg(long, value_delimiter = ',')]
    slat_degrees: Option<Vec<f64>>,
    /// Nose wheel steering limit (degrees), when the systems' differs from the
    /// contact point's (FlyByWire's A380X: 75, hydraulic/mod.rs:1792).
    #[arg(long)]
    nose_steering: Option<f64>,
    /// Steered main (body) gear limit (degrees), when the systems' differs
    /// (FlyByWire's A380X: 15, hydraulic/mod.rs:1803-1814).
    #[arg(long)]
    body_steering: Option<f64>,
    /// Instrument folders whose screens stay plain (comma-separated; panel.cfg
    /// `htmlgauge` paths like `A380X/EFB/efb.html`). The rest of panel.cfg's
    /// screens become avionics devices the systems plugin draws.
    ///
    /// The EFB is no longer skipped by default: XPHFBW draws its own
    /// settings/study UI on the `SCREEN_EFB` mesh in place of FlyByWire's own
    /// (unrendered) EFB gauge (fbw-xp-systems's display/screens.rs
    /// `SCREENS`, app/src/views.rs `spawn_efb_view`), so the mesh needs
    /// `ATTR_cockpit_device`/`ATTR_manip_device` like any other screen.
    ///
    /// The OITs are no longer skipped either: the plugin now draws
    /// FlyByWire's `A380X/OIT/oit.html` on `SCREEN_OIT_LEFT` and
    /// `SCREEN_OIT_RIGHT` (fbw-xp-systems's display/screens.rs `SCREENS`,
    /// docs/oit.md). `OITlegacy` stays listed, but on its own it skips
    /// nothing: a screen is only left plain when *every* gauge on it is
    /// skipped, and both OIT sections list `A380X/OIT/oit.html` first.
    #[arg(long, value_delimiter = ',', default_value = "OITlegacy")]
    skip_screens: Vec<String>,
    /// Distance (metres) where the exterior's L0 (full detail) hands off to
    /// L1 (`--ext-lod1-ratio` of the triangles). Default: 400 m, well past
    /// the pilot's seat -- the aircraft's own origin, where every exterior
    /// LOD distance is measured from, sits about 33 m from the cockpit view,
    /// so the cockpit always sees L0.
    #[arg(long, default_value_t = 400.0)]
    ext_lod1_far: f32,
    /// Distance (metres) where L1 hands off to L2 (`--ext-lod2-ratio` of the
    /// triangles).
    #[arg(long, default_value_t = 1500.0)]
    ext_lod2_far: f32,
    /// Distance (metres) past which the exterior stops being drawn at all.
    #[arg(long, default_value_t = 20000.0)]
    ext_lod3_far: f32,
    /// L1's share of L0's triangle count, per mesh (meshoptimizer's target,
    /// not a guarantee -- see the conversion's own report for what a build
    /// actually reached).
    #[arg(long, default_value_t = 0.25)]
    ext_lod1_ratio: f32,
    /// L2's share of L0's triangle count, per mesh.
    #[arg(long, default_value_t = 0.06)]
    ext_lod2_ratio: f32,
    /// Write the exterior at one level of detail everywhere, as before this
    /// feature existed (decals, glass, tyres and anything with a
    /// manipulator already always are, LOD or not).
    #[arg(long)]
    no_exterior_lods: bool,
    /// Distance (metres) past which the passenger cabin (the LOD01 graft)
    /// stops being drawn, from outside the aircraft as well as in --
    /// unlike the cockpit, the cabin is not marked internal-only, so an
    /// open door does not show a hollow fuselage from a nose-in gate view.
    /// From the pilot's seat the aircraft's own origin (where this is
    /// measured from) is about 33 m away, well under the default, so the
    /// cabin still draws from inside.
    #[arg(long, default_value_t = 150.0)]
    cabin_lod_far: f32,
}

/// Whether base colour textures get MSFS's ambient occlusion baked in
/// (`convert_for_xplane_capped_with_ao`, `msfs2xp::texture`).
///
/// On. It was off, on the reasoning that a straight per-material multiply
/// over-darkens: `A380_COCKPIT_MISC02` has a mean occlusion of 0.305 over
/// its actually-sampled texels, and baking it drops that panel's mean
/// luminance by 57%, which was judged to trade a few white fixture meshes
/// for a whole panel reading grimy.
///
/// The mean was the wrong statistic. The distribution on that same texture
/// is 37.7% of texels below 0.25 occlusion, 21.0% below 0.5, 21.7% below
/// 0.75, and only 10.3% effectively unoccluded. MSFS is not using this
/// channel to tint an ambient term at the margins; the shading of these
/// panels lives in it. The albedo underneath is painted near-white on
/// purpose -- `whitesweep` puts 30.0% of `A380_COCKPIT_MISC02_ALBEDO` above
/// 200 per channel -- because the COMP is expected to supply the dark. Drop
/// it and switch legends and fixture plates render as flat white blocks,
/// which is the reported defect.
///
/// The known cost is real and is the reason this is worth revisiting: X-Plane
/// has no separate ambient term for the baked value to attenuate, so it
/// multiplies direct light too, and a surface MSFS would keep bright under a
/// strong lamp comes out darker here. That is the usual approximation when
/// baking one engine's term into a texture the other reads as flat diffuse.
/// Scoping the bake per-mesh by UV footprint (as `dilate_outside_uv` already
/// scopes the material/gloss map's fix) would let the strongly-occluded
/// areas darken fully while leaving broad lit panel faces alone; that is
/// still not built, and it is the right next step if this reads too dark.
const BAKE_OCCLUSION: bool = false;

/// One texture to write.
#[derive(Debug, Clone)]
enum Job {
    /// Colour, night or other texture. `tint`: the material's
    /// `baseColorFactor` RGB to multiply in (see `TextureKey::tint`).
    Plain { src: PathBuf, tint: Option<[u8; 3]> },
    /// A base colour texture with its material's MSFS ambient occlusion
    /// (COMP red channel) baked in, since X-Plane 12's interior lighting has
    /// no ambient term of its own to carry it (see `bake_occlusion`).
    Albedo { src: PathBuf, comp: PathBuf, tint: Option<[u8; 3]> },
    /// A glass texture with its material's opacity (percent) baked into the
    /// alpha, since OBJ8 has no per-material opacity.
    Faded { src: PathBuf, alpha: u8, tint: Option<[u8; 3]> },
    /// Normal map (`TEXTURE_MAP normal`). `used`: the same UV-coverage
    /// accumulation as `Material`'s, passed to `normal_png` so an atlas's
    /// unused padding is dilated in from its real content before the
    /// flatness heuristic measures it. `floor`: usually
    /// `msfs2xp::texture::FLAT_FLOOR`, raised for `LETTERED_NORMAL_MAPS`.
    Normal { normal: PathBuf, used: Vec<[[f32; 2]; 3]>, floor: u32 },
    /// Metal/gloss map (`TEXTURE_MAP material_gloss`) from a COMP texture;
    /// without one, a plain non-metal. `used`: the UV-space triangles
    /// (`[[u, v]; 3]`, 0..1) of every mesh whose material points at this
    /// COMP texture, gathered from every group that shares it (see
    /// `Plan::material`). MSFS packs several small parts' COMP data into one
    /// shared atlas with the rest of the canvas left at whatever default
    /// the exporter fills unused space with; no mesh ever samples that
    /// padding directly, but X-Plane's own mip chain still averages it into
    /// the real content at any mip small enough for a texel to span both -
    /// exactly the small, distant flat surfaces a cockpit pushbutton or
    /// knob cap is (see `msfs2xp::texture::material_png`'s doc for the fix).
    /// `floor`: usually `msfs2xp::texture::FLAT_FLOOR`, raised for
    /// `LETTERED_MATERIAL_MAPS`.
    Material { comp: Option<PathBuf>, used: Vec<[[f32; 2]; 3]>, floor: u32 },
    /// A lettering/placard decal's base colour texture: colour-dilated at
    /// its transparent texels and re-encoded with freshly built mips (rather
    /// than MSFS's own, or none at all for a PNG), so neither a bled-in dark
    /// fringe nor an alpha-crushed vanishing stroke survives mipmapping
    /// (see `dilate_transparent`, `preserve_alpha_coverage`). `blend`: MSFS's
    /// own `baseColorBlendFactor` on this decal (`TextureKey::blend_alpha`),
    /// in percent, scaled into the alpha this same dilation pass builds --
    /// applied after dilation, not before, so `dilate_transparent`'s fixed
    /// 128 threshold still separates real lettering from the flooded
    /// background at MSFS's own full-strength alpha, not this decal's own
    /// faded copy of it.
    Decal { src: PathBuf, tint: Option<[u8; 3]>, blend: Option<u8> },
    /// A masked cutout's (glTF `MASK`) base colour texture: colour-dilated
    /// at its below-cutoff texels and re-encoded with freshly built mips
    /// that preserve the material's own alpha-test coverage -- the same
    /// treatment [`Job::Decal`] already gives a blended decal's texture,
    /// but at the cutoff this material itself declares
    /// (`TextureKey::mask_cutoff`, written as `ATTR_no_blend`/
    /// `GLOBAL_no_blend`) rather than the decal atlas's fixed 0.90.
    ///
    /// Without this, a masked cutout's base texture went through `Plain`
    /// or `Albedo` like any other texture: mips built by plain averaging,
    /// with nothing to stop a fine cutout's coverage collapsing the same
    /// way an undilated, unrestored decal's used to -- only silently,
    /// because no whole-object test ever complained about a blank
    /// rectangle the way the decal case did. Measured on the A380's own
    /// `A380X_WINDOW_FRAME` material (cutoff 0.5): full-size coverage is
    /// 1.66%, and a plain box-filter mip chain collapses that to 0% by the
    /// 16x16 level and stays there down to 4x4 -- the cutout vanishes
    /// completely at any distance sampling those mips.
    Masked { src: PathBuf, cutoff: u8, tint: Option<[u8; 3]> },
    /// A night texture for geometry that is alpha-tested, carrying the
    /// albedo's alpha instead of its own.
    ///
    /// X-Plane applies an alpha-tested object's cutoff to the night
    /// texture's alpha when the object has one, not to the albedo's. MSFS
    /// emissive atlases are fully opaque -- alpha means nothing in them --
    /// so copying one across leaves an object whose test can never discard
    /// anything, and every lettering quad draws as a filled rectangle. That
    /// is the white boxes over the A380's cockpit placards: measured, the
    /// decal albedo passes the 0.90 cutoff on 20.4% of its texels and the
    /// emissive on 100% of them.
    LitMasked { lit: PathBuf, albedo: PathBuf },
}

/// Cockpit normal maps whose real, load-bearing relief -- panel lettering,
/// a speaker grille's perforations, knob knurling -- no statistic run over
/// the map can reliably tell apart from an ordinary flat map's own noise
/// (compression artifacts, a seat's stitching, a wall's paint texture).
/// Measured over all 43 of FlyByWire's A380 cockpit normal maps that the
/// flatness heuristic (`msfs2xp::texture::normal_png`) was shrinking to
/// 256px against four candidate metrics -- whole-canvas RMS, tile-max RMS
/// at three tile sizes, a 99.9th-percentile tile error, and RMS restricted
/// to `used` UV texels -- none separated this list from the merely-flat
/// majority with one threshold: either this list stayed at 256 too, or the
/// merely-flat maps stopped shrinking as well. So these are named outright
/// and exempted from the heuristic (see `Plan::normal`'s `floor`), capped
/// at 1024 rather than restored to their full 2048-4096 source: 1024 keeps
/// FCU's numerals legible (`E:/fbw-debug/T01-compare-fcu.png`) for about
/// 78 MB more video memory across the list than leaving them at 256, well
/// under the source-resolution cost of ~3 GB that restoring every shrunk
/// normal and material map on the aircraft would have taken (see T10).
const LETTERED_NORMAL_MAPS: &[&str] = &[
    "A380_COCKPIT_FCU_4K_NORMAL",
    "A380_COCKPIT_GLARESHIELD_4K_NORMAL",
    "A380_COCKPIT_KEYBOARD_NORMAL",
    "A380_COCKPIT_MIP03_4K_NORMAL",
    "A380_COCKPIT_MIP04_4K_NORMAL",
    "A380_COCKPIT_MIP_KNOB01_4K_NORMAL",
    "A380_COCKPIT_MIP_KNOBS_02_4K_NORMAL",
    // Found by a full sweep of every other cockpit normal map against this
    // same worst_tile_error/RG-std test (E:/fbw-debug/fixes/W202.md, a lead
    // from W200): both independently lose over a third of their true RG
    // (embossed-relief) standard deviation to this heuristic's floor, in the
    // same severity range as every name in this list.
    "A380_COCKPIT_OVHD_SCREWS_4K_NORMAL",
    "A380_COCKPIT_OVHD01_4K_NORMAL",
    "A380_COCKPIT_OVHD02_4K_NORMAL",
    "A380_COCKPIT_OVHD_KNOBS_4K_NORMAL",
    "A380_COCKPIT_PEDESTAL03_4K_NORMAL",
    "A380_COCKPIT_PEDESTAL04_4K_NORMAL",
    "A380_COCKPIT_PEDESTAL_KNOBS02_4K_NORMAL",
    "A380_COCKPIT_PUSHBUTTONS01_4K_NORMAL",
    "A380_COCKPIT_RUDDER_4K_NORMAL",
    "A380_COCKPIT_STICKER_8K_NORMAL",
];

/// Interior textures of plain surfaces, capped at `--surface-max` instead of
/// `--interior-max`: leather, textiles, carpet, cables, plain sidewall,
/// ceiling and back-wall panels, glass, and the cabin's business-class seats
/// and main-deck sidewalls. Each was checked on a contact sheet of the
/// converted 4096 texture (29 Sep 2026) for lettering, placards or control
/// legends, and none has any; the ones that did (the side-window frame, the
/// seats' control panels, the window structure, ceiling 01, the passenger
/// doors, the main-deck assets) are deliberately not here and keep 4096.
/// Matched as a prefix of the output file stem, so a tinted or faded variant
/// (`_KE7E7E7`, `_MASK80`, `_A25`) and a map's normal (`_NORMAL`, `_NORM`)
/// follow their surface.
const PLAIN_SURFACE_TEXTURES: &[&str] = &[
    "A380_COCKPIT_LEATHER01_4K_",
    "A380_COCKPIT_LEATHER02_4K_",
    "A380_COCKPIT_LEATHER03_4K_",
    "A380_COCKPIT_SEAT_TEXTILE_4K_",
    "A380_COCKPIT_SEATSBELT_TEXTILE_4K_",
    "A380_COCKPIT_FLOOR_4K_",
    "RUDDER_CABLES_4K_",
    "MIKES_CABLES_4K_",
    "A380_COCKPIT_SIDEWALL01_4K_",
    "A380_COCKPIT_SIDEWALL02_4K_",
    "A380_COCKPIT_CEILING02_4K_",
    "A380_COCKPIT_BACKWALL_4K_",
    "A380_WINDSHIELD_ALBEDO",
    "A380X_LIGHT_GLASS_ALBEDO",
    "A380X_BC_SEATS_",
    "A380X_MAIN_DECK_PEC_SIDEWALLS_",
];

/// Whether an output texture file is one of [`PLAIN_SURFACE_TEXTURES`].
fn is_plain_surface(file_name: &str) -> bool {
    let upper = file_name.to_ascii_uppercase();
    PLAIN_SURFACE_TEXTURES.iter().any(|p| upper.starts_with(p))
}

/// The minimum size `normal_png` may shrink a normal map named `stem` (its
/// output file name, minus `_NML.png`) down to: 1024 for
/// [`LETTERED_NORMAL_MAPS`], otherwise the usual flatness-heuristic floor.
fn normal_floor(stem: &str) -> u32 {
    if LETTERED_NORMAL_MAPS.iter().any(|s| s.eq_ignore_ascii_case(stem)) {
        1024
    } else {
        FLAT_FLOOR
    }
}

/// COMP-derived material (metal/gloss) maps whose gloss channel carries
/// real, load-bearing contrast the flatness heuristic
/// (`msfs2xp::texture::material_png`) crushes anyway: measured directly
/// against FlyByWire's own source COMP textures (texpng-decoded, compared
/// to the real installed output with the same `worst_tile_error` metric
/// T01 built for `LETTERED_NORMAL_MAPS`: E:/fbw-debug/fixes/W105.md), these
/// four lose 60-80% of their gloss standard deviation once shrunk to the
/// heuristic's floor -- a switch's metal bezel against its matte face, a
/// keycap's edge -- while the other 11 maps `LETTERED_NORMAL_MAPS` protects
/// for their *normal* map lose under ~20% on their *material* map and are
/// left alone here. Capped at 1024 rather than restored further: 1024 is
/// already this aircraft's hard ceiling for every material map (see
/// `write_texture`'s `Job::Material` arm), so raising `floor` to 1024 just
/// stops the heuristic from shrinking these four past a decode cap every
/// material map on the aircraft already has, for about 4.75 MB total.
const LETTERED_MATERIAL_MAPS: &[&str] = &[
    "A380_COCKPIT_GLARESHIELD_4K_COMP",
    "A380_COCKPIT_KEYBOARD_COMP",
    // The next 6 were found by a full sweep of every other cockpit COMP map
    // against the same worst_tile_error/gloss-std test (E:/fbw-debug/fixes/W202.md):
    // each independently loses 36-79% of its true gloss-channel contrast to
    // this heuristic's floor, in the same range as the four names above, even
    // though none of these panels' own *normal* maps needed T01's protection
    // (which is why W105's original sweep, scoped to T01's 15-panel list,
    // never reached them).
    "A380_COCKPIT_MIP01_8K_COMP",
    "A380_COCKPIT_MIP02_4K_COMP",
    "A380_COCKPIT_MIP03_4K_COMP",
    "A380_COCKPIT_MIP_KNOBS_02_4K_COMP",
    "A380_COCKPIT_MISC01_4K_COMP",
    "A380_COCKPIT_MISC03_COMP",
    "A380_COCKPIT_PEDESTAL01_4K_COMP",
    "A380_COCKPIT_PEDESTAL_KNOBS01_4K_COMP",
    "A380_COCKPIT_RUDDER_4K_COMP",
];

/// The minimum size `material_png` may shrink a COMP-derived material map
/// named `stem` (its output file name, minus `_MAT.png`) down to: 1024 for
/// [`LETTERED_MATERIAL_MAPS`], otherwise the usual flatness-heuristic floor.
fn material_floor(stem: &str) -> u32 {
    if LETTERED_MATERIAL_MAPS.iter().any(|s| s.eq_ignore_ascii_case(stem)) {
        1024
    } else {
        FLAT_FLOOR
    }
}

/// The next mip level: each 2x2 block averaged in linear light (a box filter,
/// the usual mip filter). Lanczos rang around panel lettering, and under
/// FXAA and TAA the halos shimmered; averaging sRGB values instead would
/// grey thin white lettering out at a distance.
fn half(img: &image::RgbaImage) -> image::RgbaImage {
    let lin: Vec<f32> = (0..=255u32)
        .map(|v| {
            let c = v as f32 / 255.0;
            if c <= 0.04045 { c / 12.92 } else { ((c + 0.055) / 1.055).powf(2.4) }
        })
        .collect();
    let srgb = |l: f32| {
        let c = if l <= 0.003_130_8 { l * 12.92 } else { 1.055 * l.powf(1.0 / 2.4) - 0.055 };
        (c.clamp(0.0, 1.0) * 255.0).round() as u8
    };
    let (w, h) = (img.width(), img.height());
    image::RgbaImage::from_fn((w / 2).max(1), (h / 2).max(1), |x, y| {
        let (mut rgb, mut a) = ([0f32; 3], 0u32);
        for (dx, dy) in [(0, 0), (1, 0), (0, 1), (1, 1)] {
            let p = img.get_pixel((2 * x + dx).min(w - 1), (2 * y + dy).min(h - 1));
            for k in 0..3 {
                rgb[k] += lin[p[k] as usize];
            }
            a += p[3] as u32;
        }
        image::Rgba([srgb(rgb[0] / 4.0), srgb(rgb[1] / 4.0), srgb(rgb[2] / 4.0), ((a + 2) / 4) as u8])
    })
}

/// How much unsharp `sharpen_mip` should apply to mip level `level` (1 = the
/// first `half()` output, counting up from there): 0 at mip 1, where the box
/// output is already close to (sometimes past) a direct Lanczos downsample
/// of the source and a flat correction overshoots it; ramping up to 0.18 by
/// mip 3, where repeated box filtering has compounded into a real 4-17%
/// softness gap against Lanczos (measured on cockpit panel crops,
/// `E:/fbw-debug/T3-albedo-mips_proto_results_v2.json`,
/// `E:/fbw-debug/T03-proto_results.json`); capped at 0.18 for anything
/// deeper, since that is as far as it was measured and validated.
fn mip_sharpen_amount(level: u32) -> f32 {
    (0.09 * (level as f32 - 1.0)).clamp(0.0, 0.18)
}

/// A small unsharp mask in linear light, undoing part of the blur that
/// compounds through repeated `half()` calls: each mip level is filtered
/// from the previous level, not from the original, so by mip 3 the
/// equivalent blur is much wider than a single box pass. `half()` itself
/// stays a plain box filter (Lanczos rang around panel lettering and
/// shimmered under TAA, see its doc comment) -- this only nudges the
/// *already-boxed* result back toward a Lanczos-sharp target, and only at
/// `amount` > 0 (see `mip_sharpen_amount`: zero at mip 1, so the level
/// closest to the eye is untouched). Alpha is left exactly as `half()`
/// produced it; only RGB is sharpened.
fn sharpen_mip(img: &image::RgbaImage, amount: f32) -> image::RgbaImage {
    if amount <= 0.0 {
        return img.clone();
    }
    let lin: Vec<f32> = (0..=255u32)
        .map(|v| {
            let c = v as f32 / 255.0;
            if c <= 0.04045 { c / 12.92 } else { ((c + 0.055) / 1.055).powf(2.4) }
        })
        .collect();
    let srgb = |l: f32| {
        let c = if l <= 0.003_130_8 { l * 12.92 } else { 1.055 * l.powf(1.0 / 2.4) - 0.055 };
        (c.clamp(0.0, 1.0) * 255.0).round() as u8
    };
    let (w, h) = (img.width() as usize, img.height() as usize);
    let mut centre = vec![0f32; w * h * 3];
    for (i, p) in img.pixels().enumerate() {
        for k in 0..3 {
            centre[i * 3 + k] = lin[p[k] as usize];
        }
    }
    // The unsharp mask's low-pass: two passes of a separable 1-2-1 blur
    // (edges clamped/replicated, like `half()`'s own sampling), wide enough
    // to reach past the compounding box blur without spreading past a
    // glyph stroke onto the placard around it.
    let blur_pass = |buf: &[f32]| -> Vec<f32> {
        let mut tmp = vec![0f32; w * h * 3];
        for y in 0..h {
            for x in 0..w {
                let xm = x.saturating_sub(1);
                let xp = (x + 1).min(w - 1);
                for k in 0..3 {
                    tmp[(y * w + x) * 3 + k] =
                        (buf[(y * w + xm) * 3 + k] + 2.0 * buf[(y * w + x) * 3 + k] + buf[(y * w + xp) * 3 + k]) / 4.0;
                }
            }
        }
        let mut out = vec![0f32; w * h * 3];
        for y in 0..h {
            let ym = y.saturating_sub(1);
            let yp = (y + 1).min(h - 1);
            for x in 0..w {
                for k in 0..3 {
                    out[(y * w + x) * 3 + k] =
                        (tmp[(ym * w + x) * 3 + k] + 2.0 * tmp[(y * w + x) * 3 + k] + tmp[(yp * w + x) * 3 + k]) / 4.0;
                }
            }
        }
        out
    };
    let blurred = blur_pass(&blur_pass(&centre));
    image::RgbaImage::from_fn(img.width(), img.height(), |x, y| {
        let i = (y as usize * w + x as usize) * 3;
        let src = img.get_pixel(x, y);
        let mut out = [0u8; 4];
        for k in 0..3 {
            out[k] = srgb(centre[i + k] + amount * (centre[i + k] - blurred[i + k]));
        }
        out[3] = src[3];
        image::Rgba(out)
    })
}

#[cfg(test)]
mod click_tests {
    use super::*;

    fn mesh(points: &[[f32; 3]]) -> model::glb::Mesh {
        model::glb::Mesh {
            vertices: points
                .iter()
                .map(|&pos| model::glb::Vertex {
                    pos,
                    normal: [0.0, 1.0, 0.0],
                    uv: [0.0, 0.0],
                })
                .collect(),
            indices: vec![0, 1, 2],
            ..Default::default()
        }
    }

    fn size(m: &model::glb::Mesh) -> [f32; 3] {
        let (mut lo, mut hi) = ([f32::MAX; 3], [f32::MIN; 3]);
        for v in &m.vertices {
            for k in 0..3 {
                lo[k] = lo[k].min(v.pos[k]);
                hi[k] = hi[k].max(v.pos[k]);
            }
        }
        [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]]
    }

    #[test]
    fn a_part_posed_away_from_its_node_still_covers_only_itself() {
        // SUNSHADE_RH as the loader hands it over: four vertices posed
        // through a joint the file does not list, so the part sits twelve
        // metres below the cockpit while its node is up on the windscreen.
        let strays = mesh(&[
            [0.81, 3.07, -32.5],
            [0.83, 3.10, -32.4],
            [4.39, -9.21, -2.61],
            [4.40, -9.33, -2.08],
        ]);
        let loose = size(&click_shape(&strays, None));
        assert!(loose.iter().any(|&s| s > 3.0), "unchecked, it spans the cockpit: {loose:?}");
        let held = size(&click_shape(&strays, Some([0.82, 3.08, -32.45])));
        assert!(held.iter().all(|&s| s < 0.2), "held to its node, it covers the sunshade: {held:?}");
    }

    #[test]
    fn a_switch_keeps_its_own_size() {
        let button = mesh(&[[0.10, 2.40, -32.10], [0.11, 2.41, -32.09], [0.105, 2.405, -32.095]]);
        let box_ = size(&click_shape(&button, Some([0.105, 2.405, -32.095])));
        assert!(box_.iter().all(|&s| s < 0.03), "a button stays a button: {box_:?}");
    }
}

#[cfg(test)]
mod mip_tests {
    #[test]
    fn mips_average_in_linear_light() {
        let img = image::RgbaImage::from_fn(2, 2, |x, y| {
            image::Rgba(if (x + y) % 2 == 0 { [255, 255, 255, 255] } else { [0, 0, 0, 255] })
        });
        let m = super::half(&img);
        assert_eq!((m.width(), m.height()), (1, 1));
        // Half white and half black is 0.5 linear, which is 188 in sRGB (not 128).
        assert_eq!(m.get_pixel(0, 0).0, [188, 188, 188, 255]);
    }

    #[test]
    fn mip_sharpen_amount_is_zero_at_mip_one_and_caps_at_mip_three() {
        assert_eq!(super::mip_sharpen_amount(1), 0.0);
        assert!((super::mip_sharpen_amount(2) - 0.09).abs() < 1e-6);
        assert!((super::mip_sharpen_amount(3) - 0.18).abs() < 1e-6);
        // Deeper levels stay at the same cap, not keep climbing.
        assert_eq!(super::mip_sharpen_amount(4), 0.18);
        assert_eq!(super::mip_sharpen_amount(7), 0.18);
    }

    #[test]
    fn sharpen_mip_at_zero_amount_is_a_no_op() {
        let img = image::RgbaImage::from_fn(4, 4, |x, y| image::Rgba([((x * 60 + y * 10) % 255) as u8, 128, 0, 200]));
        let out = super::sharpen_mip(&img, 0.0);
        for y in 0..4 {
            for x in 0..4 {
                assert_eq!(out.get_pixel(x, y).0, img.get_pixel(x, y).0);
            }
        }
    }

    #[test]
    fn sharpen_mip_preserves_alpha_and_sharpens_an_edge() {
        // A flat dark half next to a flat bright half: unsharp should push
        // the dark side darker and the bright side brighter right at the
        // step (more contrast there), without ever touching alpha.
        let img = image::RgbaImage::from_fn(6, 6, |x, _y| {
            let v = if x < 3 { 40u8 } else { 220u8 };
            image::Rgba([v, v, v, 137])
        });
        let out = super::sharpen_mip(&img, 0.18);
        for y in 0..6 {
            for x in 0..6 {
                assert_eq!(out.get_pixel(x, y)[3], 137);
            }
        }
        let dark_interior = out.get_pixel(0, 3)[0];
        let dark_at_edge = out.get_pixel(2, 3)[0];
        let light_at_edge = out.get_pixel(3, 3)[0];
        let light_interior = out.get_pixel(5, 3)[0];
        assert!(dark_at_edge <= dark_interior, "the dark side should not get lighter right at the edge");
        assert!(light_at_edge >= light_interior, "the light side should not get darker right at the edge");
    }

    #[test]
    fn encode_dds_mip_chain_reaches_1x1() {
        // 16x16 opaque -> DXT1, 5 levels: 16,8,4,2,1. The old `lw <= 4`
        // cutoff stopped after the 4x4 level and never produced 2x2/1x1.
        let rgba = vec![255u8; 16 * 16 * 4];
        let dds = super::encode_dds(rgba, 16, 16, false, None).unwrap();
        let mip_count = u32::from_le_bytes(dds[28..32].try_into().unwrap());
        assert_eq!(mip_count, 5, "chain should run 16,8,4,2,1, not stop at 4x4");
    }

    #[test]
    fn bc7_alpha_noise_goes_to_dxt1_inside_but_the_livery_keeps_its_format() {
        // Alpha 252 everywhere: FlyByWire's BC7 noise on a solid surface.
        let noisy = [120u8, 130, 140, 252].repeat(16 * 16);
        let interior = super::encode_dds(noisy.clone(), 16, 16, false, None).unwrap();
        assert_eq!(&interior[84..88], b"DXT1", "an interior texture drops an alpha nothing reads");
        let exterior = super::encode_dds(noisy, 16, 16, true, None).unwrap();
        assert_eq!(&exterior[84..88], b"DXT5", "the house livery comes out exactly as before");
    }
}

#[cfg(test)]
mod decal_tests {
    use super::*;

    /// Where the glyph sits, on whole 4x4 blocks so no compressed block
    /// mixes it with the background.
    const GLYPH: std::ops::Range<u32> = 12..20;

    /// A decal as MSFS ships one: an opaque glyph on a background that is
    /// transparent and carries no colour of its own.
    fn lettering(w: u32, h: u32) -> Vec<u8> {
        let mut rgba = Vec::with_capacity((w * h * 4) as usize);
        for y in 0..h {
            for x in 0..w {
                let glyph = GLYPH.contains(&x) && GLYPH.contains(&y);
                rgba.extend_from_slice(if glyph { &[255, 255, 255, 255] } else { &[0, 0, 0, 0] });
            }
        }
        rgba
    }

    /// The whole decal texture path: dilate, encode, and the background is
    /// still transparent in the file X-Plane reads. The dilation fills the
    /// background with the lettering's own colour on purpose (a dark fringe
    /// must not bleed into the letters when the mips are built), so if the
    /// alpha goes the flood is what shows: a white box around every letter.
    #[test]
    fn a_decals_background_stays_transparent_through_the_decal_path() {
        let (w, h) = (32u32, 32u32);
        let mut rgba = lettering(w, h);
        dilate_transparent(&mut rgba, w, h, 128, 32);
        assert_eq!(&rgba[0..3], &[255, 255, 255], "the dilation floods the lettering's colour outwards");
        assert_eq!(rgba[3], 0, "and leaves the alpha that hides it alone");

        let dds = encode_dds(rgba, w, h, false, Some(model::DECAL_ALPHA_CUTOFF)).unwrap();
        assert_eq!(&dds[84..88], b"DXT5", "a decal needs its alpha channel, and DXT1 has none to give");

        let format = texpresso::Format::Bc3;
        let (lw, lh) = (w as usize, h as usize);
        let mut level0 = vec![0u8; lw * lh * 4];
        format.decompress(&dds[128..128 + format.compressed_size(lw, lh)], lw, lh, &mut level0);
        for y in 0..h {
            for x in 0..w {
                let px = ((y * w + x) * 4) as usize;
                let a = level0[px + 3];
                if GLYPH.contains(&x) && GLYPH.contains(&y) {
                    assert!(a >= model::DECAL_ALPHA_CUTOFF, "the lettering draws: {x},{y} alpha {a}");
                } else {
                    assert!(a < model::DECAL_ALPHA_CUTOFF, "the flooded background stays hidden: {x},{y} alpha {a}");
                    assert_eq!(&level0[px..px + 3], &[255, 255, 255], "and it is the flood that is hidden: {x},{y}");
                }
            }
        }
    }

    /// MSFS's own `baseColorBlendFactor` on a blended decal
    /// (`ASOBO_material_blend_gbuffer`, `Job::Decal`'s `blend`), reproduced
    /// by hand the way `write_texture`'s own `Job::Decal` arm does: dilate
    /// at full strength first (so the flood is still found and hidden by
    /// the *unscaled* 128 threshold, not this decal's own faded copy of
    /// it), then scale every texel's alpha by the same percent MSFS blends
    /// its colour by, then encode against the proportionally scaled
    /// coverage cutoff. FlyByWire's rivet ribbons (`A380_DETAILS_RIVETS`)
    /// blend at 13%.
    #[test]
    fn a_decals_blend_factor_scales_its_baked_alpha_by_the_same_percent() {
        let (w, h) = (32u32, 32u32);
        let mut rgba = lettering(w, h);
        dilate_transparent(&mut rgba, w, h, 128, 32);
        let blend = 13u8;
        for px in rgba.chunks_exact_mut(4) {
            px[3] = (px[3] as u32 * blend as u32 / 100) as u8;
        }
        let cutoff = (model::DECAL_ALPHA_CUTOFF as u32 * blend as u32 / 100) as u8;
        assert_eq!(cutoff, 29, "230 x 0.13, rounded down");

        let dds = encode_dds(rgba, w, h, false, Some(cutoff)).unwrap();
        let format = texpresso::Format::Bc3;
        let (lw, lh) = (w as usize, h as usize);
        let mut level0 = vec![0u8; lw * lh * 4];
        format.decompress(&dds[128..128 + format.compressed_size(lw, lh)], lw, lh, &mut level0);
        for y in 0..h {
            for x in 0..w {
                let px = ((y * w + x) * 4) as usize;
                let a = level0[px + 3];
                if GLYPH.contains(&x) && GLYPH.contains(&y) {
                    // MSFS's own full-strength lettering alpha (255) scaled
                    // by the same 13% its material blends the colour by,
                    // within DXT5's own block-compression error.
                    assert!((28..=38).contains(&a), "the decal still draws, faded to 13%: {x},{y} alpha {a}");
                } else {
                    assert!(a < cutoff, "the flooded background stays hidden under the scaled cutoff too: {x},{y} alpha {a}");
                }
            }
        }
    }

    /// The dilated copy is only safe where the object alpha-tests it, so a
    /// texture drawn both ways gets a file each (see `Plan::decal`).
    #[test]
    fn the_dilated_copy_is_not_the_file_an_untested_draw_shows() {
        let dir = std::env::temp_dir().join("msfs2xp_decal_plan_test");
        std::fs::create_dir_all(&dir).unwrap();
        let src = dir.join("LETTERING.PNG.DDS");
        std::fs::write(&src, encode_dds(vec![255; 4 * 4 * 4], 4, 4, false, None).unwrap()).unwrap();
        let index = BTreeMap::from([("lettering.png.dds".to_string(), src.clone())]);

        let mut plan = Plan::default();
        let decal = plan.decal(&index, "LETTERING.PNG.DDS", None, None).unwrap();
        let plain = plan.plain(&index, "LETTERING.PNG.DDS", None).unwrap();
        assert_ne!(decal, plain, "one file cannot be both dilated and drawn with no alpha test");
        assert!(matches!(plan.jobs.get(&decal), Some(Job::Decal { .. })), "{:?}", plan.jobs);
        assert!(matches!(plan.jobs.get(&plain), Some(Job::Plain { .. })), "{:?}", plan.jobs);
    }

    /// A decal blended at MSFS's own `baseColorBlendFactor` needs a file of
    /// its own too: two materials can share one decal atlas at two different
    /// factors (or one blended, one not), and each needs its own alpha
    /// scaled by its own percent, not whichever material was planned first.
    #[test]
    fn a_decals_own_blend_factor_is_its_own_file_with_an_a_suffix() {
        let dir = std::env::temp_dir().join("msfs2xp_decal_blend_plan_test");
        std::fs::create_dir_all(&dir).unwrap();
        let src = dir.join("RIVETS.PNG.DDS");
        std::fs::write(&src, encode_dds(vec![255; 4 * 4 * 4], 4, 4, false, None).unwrap()).unwrap();
        let index = BTreeMap::from([("rivets.png.dds".to_string(), src.clone())]);

        let mut plan = Plan::default();
        let full = plan.decal(&index, "RIVETS.PNG.DDS", None, None).unwrap();
        let faded = plan.decal(&index, "RIVETS.PNG.DDS", None, Some(13)).unwrap();
        assert_ne!(full, faded, "a faded copy cannot share the full-strength file");
        assert!(faded.ends_with("_A13.dds"), "{faded}");
        assert!(matches!(plan.jobs.get(&faded), Some(Job::Decal { blend: Some(13), .. })), "{:?}", plan.jobs);
        assert!(matches!(plan.jobs.get(&full), Some(Job::Decal { blend: None, .. })), "{:?}", plan.jobs);
    }

    /// One image drawn by materials with different `baseColorFactor`s needs
    /// a file each, and the tint has to reach the texels. FlyByWire's
    /// cockpit decal atlas is drawn by five materials at once -- white, a
    /// 0.8 grey and two different blacks -- so sharing one file would let
    /// whichever material was planned first decide the colour for all of
    /// them, which is how black placards came out as the white stencil.
    #[test]
    fn one_image_drawn_with_two_tints_gets_a_file_each() {
        let dir = std::env::temp_dir().join("msfs2xp_tint_plan_test");
        std::fs::create_dir_all(&dir).unwrap();
        let src = dir.join("DECALS.PNG.DDS");
        // Opaque white, the stencil a tint is meant to colour.
        std::fs::write(&src, encode_dds(vec![255; 4 * 4 * 4], 4, 4, false, None).unwrap()).unwrap();
        let index = BTreeMap::from([("decals.png.dds".to_string(), src.clone())]);

        let mut plan = Plan::default();
        let untinted = plan.decal(&index, "DECALS.PNG.DDS", None, None).unwrap();
        let black = plan.decal(&index, "DECALS.PNG.DDS", Some([0, 0, 0]), None).unwrap();
        let grey = plan.decal(&index, "DECALS.PNG.DDS", Some([231, 231, 231]), None).unwrap();
        assert_ne!(untinted, black, "a tinted copy cannot share the untinted file");
        assert_ne!(black, grey, "and two different tints cannot share one either");
        assert_eq!(plan.jobs.len(), 3, "{:?}", plan.jobs.keys().collect::<Vec<_>>());

        // The tint reaches the texels: the black copy is black where the
        // source was white, and alpha is left alone so the alpha test still
        // sees the coverage the art had.
        let args = Args::parse_from(["msfs2xp-aircraft", "--out", "x", "pkg"]);
        let out = dir.join("out.dds");
        write_texture(plan.jobs.get(&black).unwrap(), &out, &args, false, false).unwrap();
        let (_, _, px) = decode_within(&std::fs::read(&out).unwrap(), 4096).unwrap();
        assert!(px.chunks_exact(4).all(|p| p[0] < 8 && p[1] < 8 && p[2] < 8), "black tint reaches the texels: {:?}", &px[..8]);
        assert!(px.chunks_exact(4).all(|p| p[3] == 255), "and leaves alpha alone");
    }

    /// A colour texture only the passenger cabin uses (`cabin` true) is
    /// capped by `--cabin-max`, not `--interior-max` -- the same source,
    /// written once as a cabin-only texture and once as an ordinary
    /// cockpit one, must come out two different sizes.
    #[test]
    fn plain_surfaces_are_capped_and_lettered_textures_are_not() {
        for plain in [
            "A380_COCKPIT_LEATHER01_4K_ALBEDO.dds",
            "A380_COCKPIT_LEATHER02_4K_NORMAL_NML.png",
            "A380_COCKPIT_FLOOR_4K_ALBEDO_KE7E7E7.dds",
            "A380X_LIGHT_GLASS_ALBEDO_MASK80.dds",
            "A380X_BC_SEATS_NORM_NML.png",
            "mikes_cables_4k_emmis.dds",
        ] {
            assert!(is_plain_surface(plain), "{plain}");
        }
        for lettered in [
            "A380_COCKPIT_MIP03_4K_ALBEDO.dds",
            "A380_COCKPIT_DECAL_ALBEDO_DECAL.dds",
            "PUSH_TEXT_EMIS.dds",
            "A380_COCKPIT_SIDEWALLS_FRONTSHIELD_4K_ALBEDO.dds",
            "A380_COCKPIT_SEATS01_4K_ALBEDO.dds",
            "A380_COCKPIT_CEILING01_4K_ALBEDO.dds",
            "A380X_PAX_DOORS_ALBEDO_KE7E7E7.dds",
            "A380_COCKPIT_PEDESTAL01_4K_NORMAL_NML.png",
        ] {
            assert!(!is_plain_surface(lettered), "{lettered}");
        }
    }

    #[test]
    fn cabin_only_colour_textures_are_capped_by_cabin_max() {
        let dir = std::env::temp_dir().join("msfs2xp_cabin_max_test");
        std::fs::create_dir_all(&dir).unwrap();
        let src = dir.join("A380X_SUITES_ALBEDO.png");
        image::RgbaImage::from_pixel(256, 256, image::Rgba([200, 100, 50, 255])).save(&src).unwrap();
        let job = Job::Plain { src, tint: None };
        let args = Args::parse_from(["msfs2xp-aircraft", "--out", "x", "pkg", "--cabin-max", "64", "--interior-max", "256"]);

        let cabin_out = dir.join("cabin.dds");
        write_texture(&job, &cabin_out, &args, false, true).unwrap();
        let (cw, ch, _) = decode_within(&std::fs::read(&cabin_out).unwrap(), 4096).unwrap();
        assert_eq!((cw, ch), (64, 64), "a cabin-only colour texture is capped by --cabin-max");

        let cockpit_out = dir.join("cockpit.dds");
        write_texture(&job, &cockpit_out, &args, false, false).unwrap();
        let (kw, kh, _) = decode_within(&std::fs::read(&cockpit_out).unwrap(), 4096).unwrap();
        assert_eq!((kw, kh), (256, 256), "the same texture, not cabin-only, still uses --interior-max");
    }

    /// The whole masked-cutout texture path: dilate at the material's own
    /// cutoff, encode, and the flooded background stays hidden by that
    /// same cutoff. Mirrors
    /// `a_decals_background_stays_transparent_through_the_decal_path` for
    /// `Job::Masked`, at a cutoff the material declares (0.5, i.e. 128)
    /// rather than the decal atlas's fixed 0.90 -- the fix this closes:
    /// before it, a masked cutout's texture went through `Job::Plain`/
    /// `Job::Albedo` with no dilation and no coverage preservation at all
    /// (see `Job::Masked`'s doc for the measured collapse on the A380's
    /// own `A380X_WINDOW_FRAME`).
    #[test]
    fn a_masked_cutouts_background_stays_hidden_at_its_own_cutoff() {
        let (w, h) = (32u32, 32u32);
        let cutoff = 128u8; // this material's own alphaCutoff (0.5 of 255)
        let mut rgba = lettering(w, h);
        dilate_transparent(&mut rgba, w, h, cutoff, 32);
        assert_eq!(&rgba[0..3], &[255, 255, 255], "the dilation floods the cutout's colour outwards");
        assert_eq!(rgba[3], 0, "and leaves the alpha that hides it alone");

        let dds = encode_dds(rgba, w, h, false, Some(cutoff)).unwrap();
        assert_eq!(&dds[84..88], b"DXT5", "a partial cutout needs its alpha channel, and DXT1 has none to give");

        let format = texpresso::Format::Bc3;
        let (lw, lh) = (w as usize, h as usize);
        let mut level0 = vec![0u8; lw * lh * 4];
        format.decompress(&dds[128..128 + format.compressed_size(lw, lh)], lw, lh, &mut level0);
        for y in 0..h {
            for x in 0..w {
                let px = ((y * w + x) * 4) as usize;
                let a = level0[px + 3];
                if GLYPH.contains(&x) && GLYPH.contains(&y) {
                    assert!(a >= cutoff, "the cutout draws: {x},{y} alpha {a}");
                } else {
                    assert!(a < cutoff, "the flooded background stays hidden: {x},{y} alpha {a}");
                }
            }
        }
    }

    /// `Plan::masked` gives a masked cutout its own treated file, distinct
    /// from the untreated `plain`/`albedo` path (a texture with no alpha
    /// test at all still needs that path -- a lens or blur disc sharing
    /// the same image, per `Plan::decal`'s own doc), and distinct per
    /// cutoff, so two materials sharing one source image with different
    /// `mask_cutoff`s each get coverage preserved against their own test.
    #[test]
    fn a_masked_copy_is_its_own_file_per_cutoff() {
        let dir = std::env::temp_dir().join("msfs2xp_masked_plan_test");
        std::fs::create_dir_all(&dir).unwrap();
        let src = dir.join("CUTOUT.PNG.DDS");
        std::fs::write(&src, encode_dds(vec![255; 4 * 4 * 4], 4, 4, false, None).unwrap()).unwrap();
        let index = BTreeMap::from([("cutout.png.dds".to_string(), src.clone())]);

        let mut plan = Plan::default();
        let masked_half = plan.masked(&index, "CUTOUT.PNG.DDS", 128, None).unwrap();
        let masked_strict = plan.masked(&index, "CUTOUT.PNG.DDS", 230, None).unwrap();
        let plain = plan.plain(&index, "CUTOUT.PNG.DDS", None).unwrap();
        assert_ne!(masked_half, plain, "coverage-preserved and untreated cannot share a file");
        assert_ne!(masked_half, masked_strict, "two cutoffs on one image cannot share a file either");
        assert!(matches!(plan.jobs.get(&masked_half), Some(Job::Masked { cutoff: 128, .. })), "{:?}", plan.jobs);
        assert!(matches!(plan.jobs.get(&masked_strict), Some(Job::Masked { cutoff: 230, .. })), "{:?}", plan.jobs);
        assert!(matches!(plan.jobs.get(&plain), Some(Job::Plain { .. })), "{:?}", plan.jobs);
    }

    /// A lettering/placard object blends, as MSFS draws it: it must not
    /// declare a whole-object alpha test. It used to, because a blended
    /// decal drew as a solid rectangle -- which was its night texture's
    /// alpha, opaque as MSFS leaves it, defeating the test rather than the
    /// blend (see `Job::LitMasked`).
    #[test]
    fn a_decal_object_blends_rather_than_declaring_an_alpha_test() {
        let header = "GLOBAL_cockpit_lit\n".to_string();
        let obj = model::write_obj8_animated(
            &Model::default(),
            &[],
            &ObjOptions {
                texture: Some("A380_COCKPIT_DECAL_ALBEDO_DECAL.dds".into()),
                ..Default::default()
            },
            &|_| MeshAnim::default(),
            "",
            &header,
        );
        let head = obj.split("POINT_COUNTS").next().unwrap_or_default();
        assert!(!head.contains("GLOBAL_no_blend"), "a decal must not alpha-test the whole object:\n{obj}");
        assert!(head.contains("GLOBAL_cockpit_lit"), "{obj}");
    }

    /// The click object is an invisible copy of every clickable part. Its
    /// triangles must answer the mouse without ever being rasterized.
    #[test]
    fn the_click_object_is_never_drawn() {
        let obj = model::write_obj8_animated(
            &Model::default(),
            &[],
            &ObjOptions {
                texture: Some("solid_00000000.png".into()),
                draw_disable: true,
                ..Default::default()
            },
            &|_| MeshAnim::default(),
            "",
            "GLOBAL_cockpit_lit\n",
        );
        assert!(obj.contains("\nATTR_draw_disable\n"), "{obj}");
        // After the vertex and index tables, where attributes belong.
        let (head, body) = obj.split_once("POINT_COUNTS").unwrap();
        assert!(!head.contains("ATTR_draw_disable"), "not in the header:\n{obj}");
        assert!(body.contains("ATTR_draw_disable"), "{obj}");
    }

    /// A touch screen's own `ATTR_manip_device` must not make its object
    /// compete with the click object for X-Plane's one click-tested slot:
    /// proven pre-merge, it kept taking taps in an object the arbitration
    /// did not pick (see `has_classic_manipulator`'s doc comment).
    #[test]
    fn a_screens_own_manipulator_does_not_make_its_object_clickable() {
        assert!(!has_classic_manipulator("ATTR_manip_device hand SCREEN_EFB SCREEN_EFB\nTRIS 0 3\nATTR_manip_none\n"));
    }

    /// An ordinary manipulator (a breaker, a knob, a switch) still has to:
    /// it is the kind X-Plane really does test only in the one object it
    /// picks, which is why the breakers live in the click object.
    #[test]
    fn an_ordinary_manipulator_makes_its_object_clickable() {
        assert!(has_classic_manipulator("ATTR_manip_toggle hand 1 0 fbw/cockpit/x_click tip\nTRIS 0 3\nATTR_manip_none\n"));
        assert!(has_classic_manipulator("ATTR_manip_push hand 0 1 fbw/cockpit/y tip\n"));
    }

    /// An object with no manipulator at all (the common case: ordinary
    /// panels, read-only screens) is not clickable either.
    #[test]
    fn an_object_with_no_manipulator_is_not_clickable() {
        assert!(!has_classic_manipulator("ATTR_cockpit_device SCREEN_DU_PFDL 0 0 0\nTRIS 0 3\nATTR_no_cockpit\n"));
    }

    /// Several `TextureKey` groups can share one COMP texture (different
    /// objects, even different models), and the job for it is created once,
    /// by whichever call reaches it first - so its UV coverage has to
    /// accumulate across every call, not just win from the first one, or a
    /// later group's own real content would stay undilated padding as far
    /// as `material_png` is concerned.
    #[test]
    fn plan_material_unions_coverage_across_every_group_that_shares_a_comp_texture() {
        let dir = std::env::temp_dir().join("msfs2xp_material_coverage_test");
        std::fs::create_dir_all(&dir).unwrap();
        let src = dir.join("PUSH_BASE_METAL.PNG.DDS");
        std::fs::write(&src, encode_dds(vec![200; 4 * 4 * 4], 4, 4, false, None).unwrap()).unwrap();
        let index = BTreeMap::from([("push_base_metal.png.dds".to_string(), src)]);

        let mut plan = Plan::default();
        let a = plan.material(&index, Some("PUSH_BASE_METAL.PNG.DDS"), &[[[0.0, 0.0], [0.2, 0.0], [0.2, 0.2]]]);
        let b = plan.material(&index, Some("PUSH_BASE_METAL.PNG.DDS"), &[[[0.5, 0.5], [0.6, 0.5], [0.6, 0.6]]]);
        assert_eq!(a, b, "the same source file is one output job");
        match plan.jobs.get(&a) {
            Some(Job::Material { used, .. }) => {
                assert_eq!(used.len(), 2, "both groups' own triangles are kept, not just the first: {used:?}");
                assert!(used.contains(&[[0.0, 0.0], [0.2, 0.0], [0.2, 0.2]]));
                assert!(used.contains(&[[0.5, 0.5], [0.6, 0.5], [0.6, 0.6]]));
            }
            other => panic!("expected a Job::Material, got {other:?}"),
        }
    }

    /// `LETTERED_NORMAL_MAPS` protects exactly the maps named in it, by
    /// their output stem, and leaves every other normal map on the usual
    /// flatness floor: a plain wall texture must still be free to shrink.
    #[test]
    fn plan_normal_raises_the_floor_only_for_lettered_maps() {
        let dir = std::env::temp_dir().join("msfs2xp_normal_floor_test");
        std::fs::create_dir_all(&dir).unwrap();
        let fcu = dir.join("A380_COCKPIT_FCU_4K_NORMAL.PNG.DDS");
        let wall = dir.join("A380_COCKPIT_BACKWALL_4K_NORMAL.PNG.DDS");
        std::fs::write(&fcu, encode_dds(vec![128; 4 * 4 * 4], 4, 4, false, None).unwrap()).unwrap();
        std::fs::write(&wall, encode_dds(vec![128; 4 * 4 * 4], 4, 4, false, None).unwrap()).unwrap();
        let index = BTreeMap::from([
            ("a380_cockpit_fcu_4k_normal.png.dds".to_string(), fcu),
            ("a380_cockpit_backwall_4k_normal.png.dds".to_string(), wall),
        ]);

        let mut plan = Plan::default();
        let fcu_name = plan.normal(&index, "A380_COCKPIT_FCU_4K_NORMAL.PNG.DDS", &[]).unwrap();
        let wall_name = plan.normal(&index, "A380_COCKPIT_BACKWALL_4K_NORMAL.PNG.DDS", &[]).unwrap();

        match plan.jobs.get(&fcu_name) {
            Some(Job::Normal { floor, .. }) => assert_eq!(*floor, 1024, "FCU is on the lettered list"),
            other => panic!("expected a Job::Normal, got {other:?}"),
        }
        match plan.jobs.get(&wall_name) {
            Some(Job::Normal { floor, .. }) => assert_eq!(*floor, FLAT_FLOOR, "an ordinary map keeps the default floor and stays free to shrink"),
            other => panic!("expected a Job::Normal, got {other:?}"),
        }
    }

    /// The 2-panel extension from the full cockpit-normal-map sweep
    /// (E:/fbw-debug/fixes/W202.md, following a W200 lead): OVHD_SCREWS
    /// measured a 38% RG-std loss and is on the list; WINDSHIELD_FRAME,
    /// measured in the same sweep (8% loss, clearly fine), is not, and must
    /// keep the default floor.
    #[test]
    fn plan_normal_raises_the_floor_for_the_w202_sweep_offenders_but_not_their_fine_neighbours() {
        let dir = std::env::temp_dir().join("msfs2xp_normal_floor_w202_test");
        std::fs::create_dir_all(&dir).unwrap();
        let ovhd_screws = dir.join("A380_COCKPIT_OVHD_SCREWS_4K_NORMAL.PNG.DDS");
        let windshield = dir.join("A380_COCKPIT_WINDSHIELD_FRAME_4K_NORMAL.PNG.DDS");
        std::fs::write(&ovhd_screws, encode_dds(vec![128; 4 * 4 * 4], 4, 4, false, None).unwrap()).unwrap();
        std::fs::write(&windshield, encode_dds(vec![128; 4 * 4 * 4], 4, 4, false, None).unwrap()).unwrap();
        let index = BTreeMap::from([
            ("a380_cockpit_ovhd_screws_4k_normal.png.dds".to_string(), ovhd_screws),
            ("a380_cockpit_windshield_frame_4k_normal.png.dds".to_string(), windshield),
        ]);

        let mut plan = Plan::default();
        let os_name = plan.normal(&index, "A380_COCKPIT_OVHD_SCREWS_4K_NORMAL.PNG.DDS", &[]).unwrap();
        let wf_name = plan.normal(&index, "A380_COCKPIT_WINDSHIELD_FRAME_4K_NORMAL.PNG.DDS", &[]).unwrap();

        match plan.jobs.get(&os_name) {
            Some(Job::Normal { floor, .. }) => assert_eq!(*floor, 1024, "OVHD_SCREWS measured a 38% RG-std loss (fixes/W202.md) and is on the lettered normal list"),
            other => panic!("expected a Job::Normal, got {other:?}"),
        }
        match plan.jobs.get(&wf_name) {
            Some(Job::Normal { floor, .. }) => assert_eq!(*floor, FLAT_FLOOR, "WINDSHIELD_FRAME measured fine (8% loss, fixes/W202.md) and must not be on the list"),
            other => panic!("expected a Job::Normal, got {other:?}"),
        }
    }

    /// Same reasoning as
    /// `plan_material_unions_coverage_across_every_group_that_shares_a_comp_texture`:
    /// several groups can point the same normal map, and the job for it is
    /// created once, by whichever reaches it first.
    #[test]
    fn plan_normal_unions_coverage_across_every_group_that_shares_a_normal_texture() {
        let dir = std::env::temp_dir().join("msfs2xp_normal_coverage_test");
        std::fs::create_dir_all(&dir).unwrap();
        let src = dir.join("A380_COCKPIT_BACKWALL_4K_NORMAL.PNG.DDS");
        std::fs::write(&src, encode_dds(vec![128; 4 * 4 * 4], 4, 4, false, None).unwrap()).unwrap();
        let index = BTreeMap::from([("a380_cockpit_backwall_4k_normal.png.dds".to_string(), src)]);

        let mut plan = Plan::default();
        let a = plan.normal(&index, "A380_COCKPIT_BACKWALL_4K_NORMAL.PNG.DDS", &[[[0.0, 0.0], [0.2, 0.0], [0.2, 0.2]]]).unwrap();
        let b = plan.normal(&index, "A380_COCKPIT_BACKWALL_4K_NORMAL.PNG.DDS", &[[[0.5, 0.5], [0.6, 0.5], [0.6, 0.6]]]).unwrap();
        assert_eq!(a, b, "the same source file is one output job");
        match plan.jobs.get(&a) {
            Some(Job::Normal { used, .. }) => {
                assert_eq!(used.len(), 2, "both groups' own triangles are kept, not just the first: {used:?}");
            }
            other => panic!("expected a Job::Normal, got {other:?}"),
        }
    }

    /// `LETTERED_MATERIAL_MAPS` protects exactly the maps named in it, by
    /// their output stem, and leaves every other COMP-derived material map
    /// on the usual flatness floor: a plain wall's material map must still
    /// be free to shrink.
    #[test]
    fn plan_material_raises_the_floor_only_for_lettered_maps() {
        let dir = std::env::temp_dir().join("msfs2xp_material_floor_test");
        std::fs::create_dir_all(&dir).unwrap();
        let glareshield = dir.join("A380_COCKPIT_GLARESHIELD_4K_COMP.PNG.DDS");
        let backwall = dir.join("A380_COCKPIT_BACKWALL_4K_COMP.PNG.DDS");
        std::fs::write(&glareshield, encode_dds(vec![128; 4 * 4 * 4], 4, 4, false, None).unwrap()).unwrap();
        std::fs::write(&backwall, encode_dds(vec![128; 4 * 4 * 4], 4, 4, false, None).unwrap()).unwrap();
        let index = BTreeMap::from([
            ("a380_cockpit_glareshield_4k_comp.png.dds".to_string(), glareshield),
            ("a380_cockpit_backwall_4k_comp.png.dds".to_string(), backwall),
        ]);

        let mut plan = Plan::default();
        let g_name = plan.material(&index, Some("A380_COCKPIT_GLARESHIELD_4K_COMP.PNG.DDS"), &[]);
        let w_name = plan.material(&index, Some("A380_COCKPIT_BACKWALL_4K_COMP.PNG.DDS"), &[]);

        match plan.jobs.get(&g_name) {
            Some(Job::Material { floor, .. }) => assert_eq!(*floor, 1024, "GLARESHIELD is on the lettered material list"),
            other => panic!("expected a Job::Material, got {other:?}"),
        }
        match plan.jobs.get(&w_name) {
            Some(Job::Material { floor, .. }) => assert_eq!(*floor, FLAT_FLOOR, "an ordinary COMP map keeps the default floor and stays free to shrink"),
            other => panic!("expected a Job::Material, got {other:?}"),
        }
    }

    /// The 6-panel extension from the full cockpit-COMP sweep
    /// (E:/fbw-debug/fixes/W202.md): PEDESTAL01 measured a 68% gloss-std
    /// loss and is on the list; PEDESTAL02, measured alongside it (16%
    /// loss, in the same range as an already-correctly-excluded panel), is
    /// not, and must keep the default floor.
    #[test]
    fn plan_material_raises_the_floor_for_the_w202_sweep_offenders_but_not_their_fine_neighbours() {
        let dir = std::env::temp_dir().join("msfs2xp_material_floor_w202_test");
        std::fs::create_dir_all(&dir).unwrap();
        let pedestal01 = dir.join("A380_COCKPIT_PEDESTAL01_4K_COMP.PNG.DDS");
        let pedestal02 = dir.join("A380_COCKPIT_PEDESTAL02_4K_COMP.PNG.DDS");
        std::fs::write(&pedestal01, encode_dds(vec![128; 4 * 4 * 4], 4, 4, false, None).unwrap()).unwrap();
        std::fs::write(&pedestal02, encode_dds(vec![128; 4 * 4 * 4], 4, 4, false, None).unwrap()).unwrap();
        let index = BTreeMap::from([
            ("a380_cockpit_pedestal01_4k_comp.png.dds".to_string(), pedestal01),
            ("a380_cockpit_pedestal02_4k_comp.png.dds".to_string(), pedestal02),
        ]);

        let mut plan = Plan::default();
        let p01_name = plan.material(&index, Some("A380_COCKPIT_PEDESTAL01_4K_COMP.PNG.DDS"), &[]);
        let p02_name = plan.material(&index, Some("A380_COCKPIT_PEDESTAL02_4K_COMP.PNG.DDS"), &[]);

        match plan.jobs.get(&p01_name) {
            Some(Job::Material { floor, .. }) => assert_eq!(*floor, 1024, "PEDESTAL01 measured a 68% gloss-std loss (fixes/W202.md) and is on the lettered material list"),
            other => panic!("expected a Job::Material, got {other:?}"),
        }
        match plan.jobs.get(&p02_name) {
            Some(Job::Material { floor, .. }) => assert_eq!(*floor, FLAT_FLOOR, "PEDESTAL02 measured fine (16% loss, fixes/W202.md) and must not be on the list"),
            other => panic!("expected a Job::Material, got {other:?}"),
        }
    }

    /// A triangle mesh with `uvs.len() / 3` triangles, each one its own
    /// [0,1,2), [3,4,5), ... index group.
    fn tri_mesh(uvs: &[[f32; 2]]) -> model::glb::Mesh {
        model::glb::Mesh {
            vertices: uvs.iter().map(|&uv| model::glb::Vertex { pos: [0.0; 3], normal: [0.0, 1.0, 0.0], uv }).collect(),
            indices: (0..uvs.len() as u32).collect(),
            ..Default::default()
        }
    }

    #[test]
    fn uv_triangles_is_each_triangles_own_shape_not_a_bounding_box() {
        let model = Model {
            meshes: vec![
                tri_mesh(&[[0.1, 0.2], [0.3, 0.2], [0.1, 0.4]]),
                tri_mesh(&[[0.6, 0.1], [0.7, 0.1], [0.7, 0.05]]),
                tri_mesh(&[[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]), // not in `meshes` below: must not appear at all
            ],
            ..Default::default()
        };
        let tris = uv_triangles(&model, &[0, 1]);
        assert_eq!(tris, vec![[[0.1, 0.2], [0.3, 0.2], [0.1, 0.4]], [[0.6, 0.1], [0.7, 0.1], [0.7, 0.05]]]);
        assert_eq!(uv_triangles(&model, &[]), Vec::<[[f32; 2]; 3]>::new(), "no meshes, no triangles");
    }
}

/// A texture's name without the MSFS double extension (`FOO.PNG.DDS` -> `FOO`).
fn stem(file: &str) -> String {
    let mut s = file.to_string();
    loop {
        let lower = s.to_ascii_lowercase();
        match [".dds", ".ktx2", ".png", ".tif", ".tga", ".jpg", ".bmp"].iter().find(|e| lower.ends_with(*e)) {
            Some(e) => s.truncate(s.len() - e.len()),
            None => return s,
        }
    }
}

/// An output texture name: the stem with spaces made underscores, since an
/// OBJ's TEXTURE line ends at the first space ("FWD CARGO DECALS").
fn out_stem(file: &str) -> String {
    stem(file).replace(' ', "_")
}

/// Folder names X-Plane and Windows both accept.
fn safe(name: &str) -> String {
    let s: String = name.chars().map(|c| if "<>:\"/\\|?*".contains(c) { '-' } else { c }).collect();
    s.trim().trim_end_matches('.').to_string()
}

fn manifest_title(package: &Path) -> Option<String> {
    let j: serde_json::Value = serde_json::from_slice(&std::fs::read(package.join("manifest.json")).ok()?).ok()?;
    j["title"].as_str().map(str::to_string)
}

/// The first `title =` of an aircraft.cfg.
fn cfg_title(path: &Path) -> Option<String> {
    std::fs::read_to_string(path)
        .ok()?
        .lines()
        .find_map(|l| {
            let (k, v) = l.split(';').next()?.trim().split_once('=')?;
            k.trim().eq_ignore_ascii_case("title").then(|| v.trim().trim_matches('"').trim().to_string())
        })
        .filter(|t| !t.is_empty())
}

/// An MSFS thumbnail in these texture folders (`thumbnail.jpg`).
fn find_thumbnail(dirs: &[PathBuf]) -> Option<PathBuf> {
    dirs.iter()
        .flat_map(|d| std::fs::read_dir(d).into_iter().flatten().filter_map(Result::ok))
        .map(|e| e.path())
        .find(|p| p.file_name().is_some_and(|n| n.to_string_lossy().eq_ignore_ascii_case("thumbnail.jpg")))
}

/// X-Plane's picker pictures for an aircraft or livery folder, cropped to
/// fill Laminar's sizes: `<acf>_icon11.png` (800x450) and
/// `<acf>_icon11_thumb.png` (174x107).
fn write_icons(jpg: &Path, dir: &Path, acf_stem: &str) -> anyhow::Result<()> {
    let img = image::open(jpg).with_context(|| format!("reading {}", jpg.display()))?;
    std::fs::create_dir_all(dir)?;
    img.resize_to_fill(800, 450, image::imageops::FilterType::Lanczos3)
        .save(dir.join(format!("{acf_stem}_icon11.png")))?;
    img.resize_to_fill(174, 107, image::imageops::FilterType::Lanczos3)
        .save(dir.join(format!("{acf_stem}_icon11_thumb.png")))?;
    Ok(())
}

/// Built-in livery variants of a package: its other aircraft folders (like
/// `_FlyByWire_A380_842-PRIDE` with `texture = "FBWPRIDE"`), by title.
fn variants(package: &Path, plane: &Path) -> Vec<(String, PathBuf)> {
    let planes = package.join("SimObjects").join("AirPlanes");
    let mut out = Vec::new();
    for e in std::fs::read_dir(&planes).into_iter().flatten().filter_map(Result::ok) {
        let dir = e.path();
        if dir == plane || !dir.is_dir() {
            continue;
        }
        if !dir.join("aircraft.cfg").is_file() {
            continue;
        }
        let title = cfg_title(&dir.join("aircraft.cfg")).unwrap_or_else(|| e.file_name().to_string_lossy().to_string());
        out.push((title, dir));
    }
    out.sort();
    out
}

/// Every texture folder under `root` (`texture`, `texture.XYZ`).
fn texture_dirs(root: &Path) -> Vec<PathBuf> {
    let mut dirs: Vec<PathBuf> = walkdir::WalkDir::new(root)
        .into_iter()
        .filter_map(Result::ok)
        .filter(|e| e.file_type().is_dir())
        .filter(|e| e.file_name().to_string_lossy().to_ascii_lowercase().starts_with("texture"))
        .map(|e| e.into_path())
        .collect();
    dirs.sort();
    dirs
}

/// Texture files of the given folders, by lower-case file name.
fn texture_index(dirs: &[PathBuf]) -> BTreeMap<String, PathBuf> {
    let mut out = BTreeMap::new();
    for d in dirs {
        for e in std::fs::read_dir(d).into_iter().flatten().filter_map(Result::ok) {
            let lower = e.file_name().to_string_lossy().to_ascii_lowercase();
            if [".dds", ".ktx2", ".png"].iter().any(|x| lower.ends_with(x)) {
                out.entry(lower).or_insert(e.path());
            }
        }
    }
    out
}

/// The aircraft's own folder under SimObjects/AirPlanes: the one with models.
fn aircraft_dir(package: &Path) -> anyhow::Result<PathBuf> {
    let planes = package.join("SimObjects").join("AirPlanes");
    let mut found: Vec<PathBuf> = std::fs::read_dir(&planes)
        .with_context(|| format!("no SimObjects/AirPlanes in {}", package.display()))?
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.join("model").is_dir())
        .collect();
    // Variants like "_FlyByWire_A380_842-PRIDE" reuse the main model.
    found.sort_by_key(|p| p.file_name().is_some_and(|n| n.to_string_lossy().starts_with('_')));
    found.into_iter().next().context("no aircraft folder with a model/ folder")
}

/// A .gltf with its one .bin buffer, packed as a GLB in memory.
fn gltf_as_glb(gltf: &Path) -> anyhow::Result<Vec<u8>> {
    let mut json = std::fs::read(gltf)?;
    let j: serde_json::Value = serde_json::from_slice(&json)?;
    let buffers = j["buffers"].as_array().map_or(0, Vec::len);
    if buffers != 1 {
        bail!("{buffers} buffers; only single-buffer models are supported");
    }
    let uri = j["buffers"][0]["uri"].as_str().context("the buffer names no file")?;
    let mut bin = std::fs::read(gltf.with_file_name(uri)).with_context(|| format!("reading {uri}"))?;
    while json.len() % 4 != 0 {
        json.push(b' ');
    }
    while bin.len() % 4 != 0 {
        bin.push(0);
    }
    let total = 12 + 8 + json.len() + 8 + bin.len();
    let mut out = Vec::with_capacity(total);
    out.extend_from_slice(b"glTF");
    out.extend_from_slice(&2u32.to_le_bytes());
    out.extend_from_slice(&u32::try_from(total)?.to_le_bytes());
    out.extend_from_slice(&(json.len() as u32).to_le_bytes());
    out.extend_from_slice(b"JSON");
    out.extend_from_slice(&json);
    out.extend_from_slice(&(bin.len() as u32).to_le_bytes());
    out.extend_from_slice(b"BIN\0");
    out.extend_from_slice(&bin);
    Ok(out)
}

/// RGBA8 (top row first) as a DXT1 DDS when fully opaque, DXT5 otherwise,
/// with mips down to 4x4, top row first: X-Plane reads DDS like any other
/// image (drawing the fuselage from these files confirmed it; bottom-first
/// DDS put the livery upside down and scrambled the window rows).
/// `best`: iterative cluster fit (the encoder's highest quality, several
/// times slower) for textures seen up close from outside; cluster fit else.
/// `alpha_coverage_cutoff`: for a decal, the alpha-test cutoff (0..255) its
/// mips must keep their coverage against (see `preserve_alpha_coverage`), so
/// a thin lettering stroke does not fade below the cutoff and vanish a mip
/// or two before the texture itself would need to.
fn encode_dds(rgba: Vec<u8>, w: u32, h: u32, best: bool, alpha_coverage_cutoff: Option<u8>) -> anyhow::Result<Vec<u8>> {
    // Interior textures shrug off FlyByWire's BC7 alpha noise (251-254 on
    // solid surfaces) and go to DXT1 at half the video memory
    // (`msfs2xp::texture::effectively_opaque`). Exterior ones (`best`)
    // keep the exact-255 rule: the house livery's own files must come out
    // of a re-conversion exactly as they do today.
    let opaque = if best || alpha_coverage_cutoff.is_some() {
        rgba.chunks_exact(4).all(|p| p[3] == 255)
    } else {
        msfs2xp::texture::effectively_opaque(&rgba)
    };
    let (format, fourcc) = if opaque { (texpresso::Format::Bc1, b"DXT1") } else { (texpresso::Format::Bc3, b"DXT5") };
    let base_coverage = alpha_coverage_cutoff.map(|cutoff| {
        let n = rgba.chunks_exact(4).filter(|p| p[3] >= cutoff).count();
        n as f32 / (w as usize * h as usize).max(1) as f32
    });
    // How much of the full-resolution art actually passed the alpha test
    // under each texel, carried down beside the image -- see
    // `CoverageFootprint`. Without it, restoring a sparse decal atlas's
    // whole-sheet coverage promotes the empty space between glyphs over the
    // alpha test along with the glyphs, and that empty space is the flat
    // white rectangles on the placards.
    let mut footprint = alpha_coverage_cutoff.map(|cutoff| {
        let alpha: Vec<u8> = rgba.chunks_exact(4).map(|p| p[3]).collect();
        CoverageFootprint::new(&alpha, w as usize, h as usize, cutoff)
    });
    let mut img = image::RgbaImage::from_raw(w, h, rgba).context("texture size does not match its pixels")?;
    let mut levels: Vec<Vec<u8>> = Vec::new();
    let mut level = 0u32;
    loop {
        let (lw, lh) = (img.width() as usize, img.height() as usize);
        // Correct a copy, and keep filtering the uncorrected chain. Feeding
        // each level's restored alpha back into the box filter ratchets the
        // correction up mip by mip: the next level is averaged from already
        // boosted texels, is measured short again, and is boosted again,
        // which is how coverage stays pinned at the full-resolution figure
        // all the way down instead of decaying. Every level is corrected
        // against the base independently, which is what the algorithm means.
        let mut encoded = img.clone();
        if let (Some(cutoff), Some(target)) = (alpha_coverage_cutoff, base_coverage) {
            let mut alpha: Vec<u8> = img.as_raw().chunks_exact(4).map(|p| p[3]).collect();
            preserve_alpha_coverage_under(&mut alpha, cutoff, target, footprint.as_ref().map(CoverageFootprint::as_slice));
            for (px, a) in encoded.chunks_exact_mut(4).zip(alpha) {
                px[3] = a;
            }
        }
        let mut out = vec![0u8; format.compressed_size(lw, lh)];
        let params = texpresso::Params {
            algorithm: if best { texpresso::Algorithm::IterativeClusterFit } else { texpresso::Algorithm::ClusterFit },
            ..texpresso::Params::default()
        };
        format.compress(encoded.as_raw(), lw, lh, params, &mut out);
        levels.push(out);
        // Keep halving all the way to 1x1. texpresso's `compress` already
        // rounds each level up to a whole 4x4 block internally
        // (`num_blocks`, with bounds-checked reads past the edge), so
        // stopping at 4x4 bought nothing but a DDS two mip levels short of
        // the chain the MSFS-source pass-through DDS files (`writes_dds`)
        // already carry -- which is the convention a mipmap-complete
        // OpenGL texture needs unless the loader also clamps
        // GL_TEXTURE_MAX_LEVEL to the DDS header's own mip count.
        if lw <= 1 && lh <= 1 {
            break;
        }
        img = half(&img);
        level += 1;
        img = sharpen_mip(&img, mip_sharpen_amount(level));
        footprint = footprint.map(|f| f.half());
    }
    let mut out = Vec::with_capacity(128 + levels.iter().map(Vec::len).sum::<usize>());
    let u32s = |v: &mut Vec<u8>, xs: &[u32]| xs.iter().for_each(|x| v.extend_from_slice(&x.to_le_bytes()));
    out.extend_from_slice(b"DDS ");
    // size, flags (caps|height|width|pixelformat|mipmapcount|linearsize), height, width, linear size, depth, mips
    u32s(&mut out, &[124, 0x1 | 0x2 | 0x4 | 0x1000 | 0x20000 | 0x80000, h, w, levels[0].len() as u32, 0, levels.len() as u32]);
    u32s(&mut out, &[0; 11]);
    u32s(&mut out, &[32, 0x4]);
    out.extend_from_slice(fourcc);
    u32s(&mut out, &[0; 5]);
    // caps: texture | complex | mipmap
    u32s(&mut out, &[0x1000 | 0x8 | 0x40_0000, 0, 0, 0, 0]);
    for l in levels {
        out.extend_from_slice(&l);
    }
    Ok(out)
}

/// The file-name suffix a tinted copy of a texture gets, so materials that
/// draw one image with different `baseColorFactor`s do not collide. Four of
/// the A380's base colour images are shared that way, the cockpit decal
/// atlas by five materials at once.
fn tint_suffix(tint: Option<[u8; 3]>) -> String {
    tint.map_or(String::new(), |t| format!("_K{:02X}{:02X}{:02X}", t[0], t[1], t[2]))
}

/// Plans texture outputs by name, so each is written once.
#[derive(Default)]
struct Plan {
    jobs: BTreeMap<String, Job>,
    used: HashSet<String>,
    missing: BTreeSet<String>,
}

impl Plan {
    /// A colour or night texture: its output file name (relative to objects/).
    fn plain(&mut self, index: &BTreeMap<String, PathBuf>, file: &str, tint: Option<[u8; 3]>) -> Option<String> {
        let lower = file.to_ascii_lowercase();
        let Some(src) = index.get(&lower) else {
            self.missing.insert(file.to_string());
            return None;
        };
        let head = read_head(src)?;
        // A tinted copy is re-encoded from its texels, so it is a DDS even
        // where the source was a PNG that would otherwise pass straight
        // through.
        let ext = if tint.is_none() && detect(&head) == SourceFormat::Png { "png" } else { "dds" };
        let name = format!("{}{}.{ext}", out_stem(&src.file_name()?.to_string_lossy()), tint_suffix(tint));
        self.used.insert(lower);
        self.jobs.entry(name.clone()).or_insert(Job::Plain { src: src.clone(), tint });
        Some(name)
    }

    /// A base colour texture: its output file name, with the material's MSFS
    /// ambient occlusion (COMP red channel) baked in when it has one.
    /// Without a COMP texture (not found, or none named), this is exactly
    /// `plain`, unbaked and byte-for-byte what it wrote before.
    fn albedo(&mut self, index: &BTreeMap<String, PathBuf>, file: &str, comp: Option<&str>, tint: Option<[u8; 3]>) -> Option<String> {
        // Baking is off until it writes the aircraft's usual compressed DDS:
        // an uncompressed PNG per albedo costs gigabytes of disk and video
        // memory, and a regeneration without textures left every object
        // pointing at files that were never written.
        if !BAKE_OCCLUSION {
            return self.plain(index, file, tint);
        }
        let Some(csrc) = comp.and_then(|c| index.get(&c.to_ascii_lowercase())).cloned() else {
            return self.plain(index, file, tint);
        };
        let lower = file.to_ascii_lowercase();
        let Some(src) = index.get(&lower) else {
            self.missing.insert(file.to_string());
            return None;
        };
        let name = format!("{}{}_AO.dds", out_stem(&src.file_name()?.to_string_lossy()), tint_suffix(tint));
        self.used.insert(lower);
        self.used.insert(csrc.file_name().unwrap_or_default().to_string_lossy().to_ascii_lowercase());
        self.jobs.entry(name.clone()).or_insert(Job::Albedo { src: src.clone(), comp: csrc, tint });
        Some(name)
    }

    /// A lettering/placard decal's base colour texture: its output file
    /// name, colour-dilated and re-mipped (see `Job::Decal`).
    ///
    /// The dilated copy gets a file of its own. Its transparent texels carry
    /// the lettering's colour rather than their own, which only stays
    /// invisible where the object alpha-tests the texture; MSFS materials
    /// that draw the same texture with no alpha test at all (the lens over a
    /// cockpit LED, a fan blur disc) would show that flood as a solid box,
    /// and they keep the undilated texture instead. Sharing one file left
    /// whichever material was planned first to decide for both.
    ///
    /// `blend`: this decal's own `baseColorBlendFactor` (percent,
    /// `TextureKey::blend_alpha`), when MSFS fades it below full strength --
    /// its own suffix, `_A<nn>` as `Plan::faded` already uses for glass, so a
    /// decal shared at two different blend factors (or one blended and one
    /// not) still gets a file each.
    fn decal(&mut self, index: &BTreeMap<String, PathBuf>, file: &str, tint: Option<[u8; 3]>, blend: Option<u8>) -> Option<String> {
        let lower = file.to_ascii_lowercase();
        let Some(src) = index.get(&lower) else {
            self.missing.insert(file.to_string());
            return None;
        };
        let blend_suffix = blend.map_or(String::new(), |b| format!("_A{b:02}"));
        let name = format!("{}{}_DECAL{blend_suffix}.dds", out_stem(&src.file_name()?.to_string_lossy()), tint_suffix(tint));
        self.used.insert(lower);
        self.jobs.entry(name.clone()).or_insert(Job::Decal { src: src.clone(), tint, blend });
        Some(name)
    }

    /// A masked cutout's base colour texture: its output file name,
    /// colour-dilated and re-mipped at its own alpha-test cutoff rather
    /// than left to lose coverage like any other texture (see
    /// `Job::Masked`).
    ///
    /// Its own file, named with the cutoff: two `TextureKey`s can share one
    /// source image with different `mask_cutoff`s (a cutout used once at
    /// its glTF default and once with a stricter override), and each needs
    /// its coverage preserved against its own test, not the other one's.
    fn masked(&mut self, index: &BTreeMap<String, PathBuf>, file: &str, cutoff: u8, tint: Option<[u8; 3]>) -> Option<String> {
        let lower = file.to_ascii_lowercase();
        let Some(src) = index.get(&lower) else {
            self.missing.insert(file.to_string());
            return None;
        };
        let name = format!("{}{}_MASK{cutoff:02X}.dds", out_stem(&src.file_name()?.to_string_lossy()), tint_suffix(tint));
        self.used.insert(lower);
        self.jobs.entry(name.clone()).or_insert(Job::Masked { src: src.clone(), cutoff, tint });
        Some(name)
    }

    /// A glass texture faded to `alpha` percent: its output file name.
    fn faded(&mut self, index: &BTreeMap<String, PathBuf>, file: &str, alpha: u8, tint: Option<[u8; 3]>) -> Option<String> {
        let lower = file.to_ascii_lowercase();
        let Some(src) = index.get(&lower) else {
            self.missing.insert(file.to_string());
            return None;
        };
        let name = format!("{}{}_A{alpha:02}.dds", out_stem(&src.file_name()?.to_string_lossy()), tint_suffix(tint));
        self.used.insert(lower);
        self.jobs.entry(name.clone()).or_insert(Job::Faded { src: src.clone(), alpha, tint });
        Some(name)
    }

    /// A night texture for an alpha-tested object, rebuilt with the
    /// albedo's alpha (see `Job::LitMasked`). Its own file, since the same
    /// emissive can also be drawn by objects that are not alpha-tested and
    /// must keep its original alpha there.
    fn lit_masked(&mut self, index: &BTreeMap<String, PathBuf>, lit: &str, albedo: &str) -> Option<String> {
        let ll = lit.to_ascii_lowercase();
        let al = albedo.to_ascii_lowercase();
        let (Some(lsrc), Some(asrc)) = (index.get(&ll), index.get(&al)) else {
            if !index.contains_key(&ll) {
                self.missing.insert(lit.to_string());
            }
            return self.plain(index, lit, None);
        };
        let name = format!("{}_LITA.dds", out_stem(&lsrc.file_name()?.to_string_lossy()));
        self.used.insert(ll);
        self.jobs.entry(name.clone()).or_insert(Job::LitMasked { lit: lsrc.clone(), albedo: asrc.clone() });
        Some(name)
    }

    /// A normal map: its output file name.
    /// `used`: this call's own contribution to the map's UV coverage, on
    /// the same reasoning as `Plan::material`'s (see `Job::Normal`),
    /// unioned into whatever the job already has.
    fn normal(&mut self, index: &BTreeMap<String, PathBuf>, normal: &str, used: &[[[f32; 2]; 3]]) -> Option<String> {
        let nl = normal.to_ascii_lowercase();
        let Some(nsrc) = index.get(&nl) else {
            self.missing.insert(normal.to_string());
            return None;
        };
        let stem = out_stem(&nsrc.file_name()?.to_string_lossy());
        let name = format!("{stem}_NML.png");
        self.used.insert(nl);
        match self.jobs.entry(name.clone()) {
            std::collections::btree_map::Entry::Occupied(mut e) => {
                if let Job::Normal { used: job_used, .. } = e.get_mut() {
                    job_used.extend_from_slice(used);
                }
            }
            std::collections::btree_map::Entry::Vacant(e) => {
                e.insert(Job::Normal { normal: nsrc.clone(), used: used.to_vec(), floor: normal_floor(&stem) });
            }
        }
        Some(name)
    }

    /// The metal/gloss map for a COMP texture, or a plain one without: its
    /// output file name.
    /// `used`: this call's own contribution to the COMP texture's UV
    /// coverage (see `Job::Material`), unioned into whatever the job already
    /// has: several different `TextureKey` groups (different objects, even
    /// different models) can share one COMP file, and the job for it is
    /// created once, by whichever call reaches it first. Coverage has to
    /// accumulate across every call, not just the first, since textures are
    /// only written after every group has been planned (`main`'s `plan.jobs`
    /// is not drained until then). The job's `floor` is resolved from
    /// `material_floor` once, the same time its output name is chosen.
    fn material(&mut self, index: &BTreeMap<String, PathBuf>, comp: Option<&str>, used: &[[[f32; 2]; 3]]) -> String {
        let csrc = comp.and_then(|c| {
            let found = index.get(&c.to_ascii_lowercase());
            if found.is_none() {
                self.missing.insert(c.to_string());
            }
            found
        });
        let name = match csrc {
            Some(c) => format!("{}_MAT.png", out_stem(&c.file_name().unwrap_or_default().to_string_lossy())),
            None => "default_MAT.png".to_string(),
        };
        if let Some(c) = csrc {
            self.used.insert(c.file_name().unwrap_or_default().to_string_lossy().to_ascii_lowercase());
        }
        match self.jobs.entry(name.clone()) {
            std::collections::btree_map::Entry::Occupied(mut e) => {
                if let Job::Material { used: job_used, .. } = e.get_mut() {
                    job_used.extend_from_slice(used);
                }
            }
            std::collections::btree_map::Entry::Vacant(e) => {
                let floor = material_floor(name.trim_end_matches("_MAT.png"));
                e.insert(Job::Material { comp: csrc.cloned(), used: used.to_vec(), floor });
            }
        }
        name
    }
}

/// The first bytes of a file, enough to tell its format.
fn read_head(path: &Path) -> Option<Vec<u8>> {
    use std::io::Read;
    let mut buf = vec![0u8; 16];
    let n = std::fs::File::open(path).ok()?.read(&mut buf).ok()?;
    buf.truncate(n);
    Some(buf)
}

/// Is this a normal map, by the name's last word?
fn is_normal(file: &str) -> bool {
    let s = stem(file).to_ascii_uppercase();
    let last = s.rsplit(['_', ' ', '-']).next().unwrap_or("");
    last.starts_with("NORM") || last == "NML"
}

/// Is this a livery's fuselage-paint or registration texture, by an
/// underscore/space/hyphen-separated token in its output name? Matching
/// whole tokens, not a substring search, keeps `DETAIL` from tripping the
/// `TAIL` check: `DETAIL` splits to its own token and `TAIL` never appears
/// as one inside it. Covers this aircraft's `FUSE1`-`FUSE5` sections and
/// `REGISTRATION` decal, plus `TAIL`/`TITLE` for aircraft that split those
/// out as their own texture instead of baking them into the fuselage skin.
fn is_fuselage_paint(name: &str) -> bool {
    stem(name).to_ascii_uppercase().split(['_', ' ', '-']).any(|t| {
        t == "FUSE" || t == "TAIL" || t == "TITLE" || t == "TITLES" || t == "REGISTRATION"
            || (t.len() > 4 && t.starts_with("FUSE") && t[4..].bytes().all(|b| b.is_ascii_digit()))
    })
}

#[cfg(test)]
mod fuselage_paint_tests {
    use super::is_fuselage_paint;

    #[test]
    fn fuse_sections_and_registration_match() {
        // Real installed/source names (Emirates, Pride, base house install).
        assert!(is_fuselage_paint("A380X_FUSE1_ALBEDO_KE7E7E7.dds"));
        assert!(is_fuselage_paint("A380X_FUSE5_ALBEDO.dds"));
        assert!(is_fuselage_paint("A380X_REGISTRATION_ALBEDO_KE7E7E7_DECAL.dds"));
        assert!(is_fuselage_paint("737_TAIL_ALBEDO.dds"), "other aircraft may name it TAIL, not FUSE<n>");
    }

    #[test]
    fn detail_does_not_false_positive_on_tail() {
        // "DETAIL" contains the letters T-A-I-L but is its own token; a
        // substring search on "TAIL" would wrongly match this real name.
        assert!(!is_fuselage_paint("A380X_DETAIL_CARPET_SEWING_ALBEDO_KE7E7E7.dds"));
    }

    #[test]
    fn unrelated_exterior_textures_do_not_match() {
        assert!(!is_fuselage_paint("A380_EXTERIOR_WING1_ALBEDO.dds"));
        assert!(!is_fuselage_paint("ENGINE_BLUR1_ALBEDO_KE7E7E7_DECAL.dds"));
    }
}

/// The COMP texture next to a normal map, by name (`X_NORMAL` -> `X_COMP`).
fn comp_for(index: &BTreeMap<String, PathBuf>, normal: &str) -> Option<String> {
    let s = stem(normal);
    let cut = s.rfind(['_', ' ', '-'])?;
    let want = format!("{}_comp", s[..cut].to_ascii_lowercase());
    index.keys().find(|k| stem(k) == want).cloned()
}

/// Write one texture. Textures only the cockpit and cabin use (`exterior`
/// false) are capped at `--interior-max`, normal maps included. A livery's
/// fuselage-paint and registration textures (`is_fuselage_paint`) are
/// capped at `--fuselage-max` instead of the plain exterior cap, when it is
/// set; left unset, this is exactly the old exterior cap for every texture.
///
/// `cabin`: a colour or emissive texture (`Job::Plain`/`Job::Albedo`/
/// `Job::Faded`/`Job::LitMasked`/decal and masked variants) referenced only
/// by the passenger cabin is capped at `--cabin-max` instead of
/// `--interior-max`. Normal and material maps ignore it -- they already
/// have their own policy (the shrink-to-detail heuristic and
/// `LETTERED_NORMAL_MAPS`/`LETTERED_MATERIAL_MAPS`'s floors), which does
/// not need a second, cabin-specific cap layered on top.
fn write_texture(job: &Job, out: &Path, args: &Args, exterior: bool, cabin: bool) -> anyhow::Result<usize> {
    let ext_cap = match args.fuselage_max {
        Some(m) if exterior && is_fuselage_paint(&out.file_name().unwrap_or_default().to_string_lossy()) => m,
        _ => args.exterior_max,
    };
    let surface = !exterior && is_plain_surface(&out.file_name().unwrap_or_default().to_string_lossy());
    let interior_cap = if cabin { args.cabin_max } else { args.interior_max };
    let interior_cap = if surface { interior_cap.min(args.surface_max) } else { interior_cap };
    let max = args.max_texture.min(if exterior { ext_cap } else { interior_cap });
    let normal_interior = if surface { args.interior_max.min(args.surface_max) } else { args.interior_max };
    let nmax = args.normal_max.min(if exterior { ext_cap } else { normal_interior });
    let bytes = match job {
        Job::Plain { src, tint } => {
            let data = std::fs::read(src)?;
            // PNGs and DXT DDS convert as they are (top row first, as X-Plane
            // reads a DDS); BC7 and the rest are re-encoded, since X-Plane is
            // only sure to read DXT.
            // A tint has to touch the texels, so it always takes the
            // decode-and-re-encode path rather than passing the file through.
            let passthrough = tint.is_none()
                && (detect(&data) == SourceFormat::Png || output_extension(&data).map_err(|e| anyhow!("{e}"))? == "dds");
            if passthrough {
                convert_for_xplane_capped(&data, max).map_err(|e| anyhow!("{e}"))?.bytes
            } else {
                let (w, h, mut rgba) = decode_within(&data, max).map_err(|e| anyhow!("{e}"))?;
                if let Some(t) = tint {
                    tint_rgba(&mut rgba, *t);
                }
                encode_dds(rgba, w, h, exterior, None)?
            }
        }
        Job::Albedo { src, comp, tint } => {
            let data = std::fs::read(src)?;
            let c = std::fs::read(comp)?;
            convert_for_xplane_capped_with_ao(&data, Some(&c), max, *tint).map_err(|e| anyhow!("{e}"))?.bytes
        }
        Job::Faded { src, alpha, tint } => {
            let (w, h, mut rgba) = decode_within(&std::fs::read(src)?, max).map_err(|e| anyhow!("{e}"))?;
            for px in rgba.chunks_exact_mut(4) {
                px[3] = (px[3] as u32 * *alpha as u32 / 100) as u8;
            }
            if let Some(t) = tint {
                tint_rgba(&mut rgba, *t);
            }
            encode_dds(rgba, w, h, false, None)?
        }
        Job::Normal { normal, used, floor } => {
            // Uncompressed PNG: block compression turns normals into 4x4
            // facets that show as squares on smooth paint, and X-Plane warns
            // about compressed material maps.
            normal_png(&std::fs::read(normal)?, nmax, used, *floor).map_err(|e| anyhow!("{e}"))?.bytes
        }
        Job::Material { comp, used, floor } => {
            let c = comp.as_ref().map(std::fs::read).transpose()?;
            // Metal and gloss barely change across a surface, so these maps
            // carry no fine detail: 1024 keeps the whole aircraft under the
            // video memory budget, where FBW's 4096 alone cost 5 GB. A few
            // panels' gloss channel is the exception (see
            // `LETTERED_MATERIAL_MAPS`), so `floor` can hold the flatness
            // heuristic back from shrinking past this same 1024 cap for
            // them -- never above it, since there is nothing
            // higher-resolution left once the decode step has already
            // thrown it away.
            let cap = nmax.min(1024);
            material_png(c.as_deref(), cap, used, (*floor).min(cap)).map_err(|e| anyhow!("{e}"))?.bytes
        }
        Job::LitMasked { lit, albedo } => {
            let (lw, lh, mut px) = decode_within(&std::fs::read(lit)?, max).map_err(|e| anyhow!("{e}"))?;
            let (aw, ah, alb) = decode_within(&std::fs::read(albedo)?, max).map_err(|e| anyhow!("{e}"))?;
            // Sampled by proportion, so a night texture and its albedo at
            // different sizes still line up through the UV set they share.
            for y in 0..lh as usize {
                for x in 0..lw as usize {
                    let ax = x * aw as usize / (lw as usize).max(1);
                    let ay = y * ah as usize / (lh as usize).max(1);
                    px[(y * lw as usize + x) * 4 + 3] = alb[(ay * aw as usize + ax) * 4 + 3];
                }
            }
            encode_dds(px, lw, lh, exterior, None)?
        }
        Job::Decal { src, tint, blend } => {
            let (w, h, mut rgba) = decode_within(&std::fs::read(src)?, max).map_err(|e| anyhow!("{e}"))?;
            // Below this a texel is "the transparent background", not
            // lettering: FlyByWire's decal texels that show are 240 or more
            // (see `DECAL_ALPHA_TEST`'s own comment), so anything under a
            // half is unambiguously background whose colour never shows.
            // Run at MSFS's own full-strength alpha, before `blend` fades it
            // below -- a decal faded to 13% (the rivet ribbons) would
            // otherwise put every real texel under this fixed 128 too.
            dilate_transparent(&mut rgba, w, h, 128, 32);
            // After the dilation, so the flood carries the tinted colour of
            // the lettering it stands in for rather than the untinted one.
            if let Some(t) = tint {
                tint_rgba(&mut rgba, *t);
            }
            // MSFS's own `baseColorBlendFactor` (`TextureKey::blend_alpha`):
            // a decal draws with real translucency (`ATTR_blend`, not an
            // alpha test), so scaling every texel's alpha by the same
            // factor MSFS blends its colour by reproduces the same fade.
            // The coverage cutoff `encode_dds` preserves mip coverage
            // against is scaled the same proportion, so it still means "the
            // same fraction of the texture MSFS's own alpha would call
            // solid" once every texel's alpha has been scaled down to
            // match -- not a runtime test (decals declare none), only the
            // authoring-time heuristic `preserve_alpha_coverage_under` uses
            // to stop a faded decal's pattern collapsing at small mips.
            let cutoff = match blend {
                Some(b) => {
                    for px in rgba.chunks_exact_mut(4) {
                        px[3] = (px[3] as u32 * *b as u32 / 100) as u8;
                    }
                    (model::DECAL_ALPHA_CUTOFF as u32 * *b as u32 / 100) as u8
                }
                None => model::DECAL_ALPHA_CUTOFF,
            };
            encode_dds(rgba, w, h, exterior, Some(cutoff))?
        }
        Job::Masked { src, cutoff, tint } => {
            let (w, h, mut rgba) = decode_within(&std::fs::read(src)?, max).map_err(|e| anyhow!("{e}"))?;
            // A texel already below this material's own cutoff never shows
            // before coverage restoration runs, whatever colour it
            // carries -- the same reasoning `Job::Decal` applies at its own
            // fixed 128, here at the cutoff this material actually tests
            // against, so restoration cannot promote a texel that still
            // holds whatever the transparent background happened to
            // contain.
            dilate_transparent(&mut rgba, w, h, *cutoff, 32);
            if let Some(t) = tint {
                tint_rgba(&mut rgba, *t);
            }
            encode_dds(rgba, w, h, exterior, Some(*cutoff))?
        }
    };
    std::fs::write(out, &bytes)?;
    Ok(bytes.len())
}

/// Lift decal meshes (lettering, rivet ribbons) along their normals. MSFS
/// draws them with a depth bias on top of the surface they sit on, some a
/// fraction of a millimetre above it; X-Plane's polygon offset is no use (it
/// drapes such geometry), so they are moved off the surface instead.
///
/// Exterior decals (fuselage stencils, rivet lines) keep the original 1.5 mm:
/// on parts metres across, and seen from metres away, it clears the surface
/// with room to spare and is not noticed. The same 1.5 mm on a cockpit
/// pushbutton lens or legend, a part perhaps 15 mm across seen from arm's
/// length, is 10% of the part's own size: it read as a lens glued a visible
/// gap above its button rather than sitting in it, exactly the "lifted"
/// button caps this was written to explain (`docs/briefs/debug.md`, symptom
/// 6). 0.3 mm keeps clear of the surface (still 20x MSFS's own bias) while
/// staying below what shows as a gap at cockpit viewing distance.
fn lift_decals(model: &mut Model, interior: bool) {
    const EXTERIOR_LIFT: f32 = 0.0015;
    const INTERIOR_LIFT: f32 = 0.0003;
    let lift = if interior { INTERIOR_LIFT } else { EXTERIOR_LIFT };
    let mut lifted = 0;
    for mesh in &mut model.meshes {
        if !model.materials.get(mesh.material).is_some_and(|m| m.decal) {
            continue;
        }
        for v in &mut mesh.vertices {
            for k in 0..3 {
                v.pos[k] += v.normal[k] * lift;
            }
        }
        lifted += 1;
    }
    if lifted > 0 {
        println!("  {lifted} decal meshes lifted {:.1} mm off their surfaces", lift * 1000.0);
    }
}

/// Every node's index that is `root` itself or descends from it, found by
/// walking each node's own parent chain rather than recursing down `root`'s
/// children: one pass over every node, and it does not care whether
/// `children` arrays are complete (this crate rebuilds `Node::parent` from
/// them once, at load time; nothing downstream re-reads `children`).
fn node_subtree(nodes: &[model::glb::Node], root: usize) -> Vec<bool> {
    (0..nodes.len())
        .map(|i| {
            let mut n = Some(i);
            for _ in 0..128 {
                let Some(k) = n else { break };
                if k == root {
                    return true;
                }
                n = nodes[k].parent;
            }
            false
        })
        .collect()
}

/// `mat_map[old]`, filling it in from `lod01_materials[old]` on first use:
/// an existing entry in `model_materials` that is equal field-for-field is
/// reused (true for every material LOD0 and LOD01 share, per `graft_cabin`'s
/// own doc comment) so the graft does not duplicate a texture the cockpit
/// model already loads.
fn resolve_material(model_materials: &mut Vec<model::glb::Material>, mat_map: &mut HashMap<usize, usize>, lod01_materials: &[model::glb::Material], old: usize) -> usize {
    if let Some(&m) = mat_map.get(&old) {
        return m;
    }
    let mat = lod01_materials[old].clone();
    let idx = model_materials.iter().position(|x| *x == mat).unwrap_or_else(|| {
        model_materials.push(mat);
        model_materials.len() - 1
    });
    mat_map.insert(old, idx);
    idx
}

/// The `a380_cabin` subtree of `lod01`, grafted onto `model` in place of the
/// LOD0 cabin the caller already stripped out of it. Returns the number of
/// triangles added.
///
/// LOD0's own passenger cabin is 5.65M triangles; MSFS's own LOD01 model
/// carries the same `a380_cabin` subtree at 180k (measured on FlyByWire's
/// A380X package: `a380_cockpit.gltf` vs `a380_cockpit_LOD01.gltf`), and
/// every textured material it uses there but one points at the same
/// `*_ALBEDO/_NORM/_COMP/_EMIS.PNG.DDS` files LOD0 uses (`A380X_BC_SEATS`,
/// `A380X_SUITES`, `A380X_MAIN_DECK_ASSETS`, and so on, checked by name) --
/// only a few surfaces the pilot is never close to (cabin floor, pax window
/// glass) are simplified from a texture to a flat colour at LOD01, which is
/// what an LOD1 asset is supposed to do.
///
/// This is not lossless: LOD01's `a380_cabin` has no passenger-door
/// sub-parts at all -- 420 nodes and 144 animation channels in LOD0 (door
/// leaves, hinge arms, guide rods, armed indicators), none in LOD01 -- so
/// the spliced cabin's doors are plain fuselage wall, no door detail. Nor
/// does this carry any animation over: only nodes, meshes and materials are
/// grafted, not `lod01.clips`, so the grafted cabin has no manipulators or
/// `fbw/anim/*` datarefs of its own. Nothing in `fbw-xp-systems` drives the
/// LOD0 door clips today (no writer for a `fbw/anim/PAX_DOOR_*`-style
/// dataref), so this does not regress anything that currently animates; a
/// future passenger-door or cabin-shade feature would need to keep the LOD0
/// parts (and their clips) instead of calling this.
fn graft_cabin(model: &mut Model, lod01: &Model) -> anyhow::Result<usize> {
    let cab = lod01
        .nodes
        .iter()
        .position(|n| n.name.trim().eq_ignore_ascii_case("a380_cabin"))
        .context("no a380_cabin node")?;
    let inside = node_subtree(&lod01.nodes, cab);

    // Append the subtree's own nodes; `node_subtree` only returns true by
    // reaching `cab`, so every ancestor of an "inside" node is "inside"
    // too, and the parent remap below only ever falls back to `None` at
    // `cab` itself (a root node with no parent in both files).
    let mut node_map: HashMap<usize, usize> = HashMap::new();
    for (i, node) in lod01.nodes.iter().enumerate() {
        if inside[i] {
            node_map.insert(i, model.nodes.len());
            model.nodes.push(node.clone());
        }
    }
    for (&old, &new) in &node_map {
        model.nodes[new].parent = lod01.nodes[old].parent.and_then(|p| node_map.get(&p).copied());
    }

    let mut mat_map: HashMap<usize, usize> = HashMap::new();
    let mut added = 0usize;
    for m in lod01.meshes.iter().filter(|m| m.node.is_some_and(|n| inside[n])) {
        let material = resolve_material(&mut model.materials, &mut mat_map, &lod01.materials, m.material);
        added += m.indices.len() / 3;
        model.meshes.push(model::glb::Mesh {
            name: m.name.clone(),
            material,
            vertices: m.vertices.clone(),
            indices: m.indices.clone(),
            node: m.node.and_then(|n| node_map.get(&n).copied()),
        });
    }
    for m in lod01.click_spots.iter().filter(|m| m.node.is_some_and(|n| inside[n])) {
        let material = resolve_material(&mut model.materials, &mut mat_map, &lod01.materials, m.material);
        model.click_spots.push(model::glb::Mesh {
            name: m.name.clone(),
            material,
            vertices: m.vertices.clone(),
            indices: m.indices.clone(),
            node: m.node.and_then(|n| node_map.get(&n).copied()),
        });
    }
    Ok(added)
}

/// Loads `<gltf's name>_LOD01.gltf` from the same folder and grafts its
/// `a380_cabin` subtree in (see `graft_cabin`). Returns an error rather than
/// panicking when that sibling file is missing, unreadable, or carries no
/// `a380_cabin` node, so the caller can fall back to leaving the cabin out
/// instead of failing the whole conversion over it.
fn splice_lod01_cabin(model: &mut Model, gltf: &Path) -> anyhow::Result<usize> {
    let lod01_path = gltf.with_file_name(format!("{}_LOD01.gltf", gltf.file_stem().unwrap_or_default().to_string_lossy()));
    let lod01: Model = gltf_as_glb(&lod01_path)
        .with_context(|| format!("reading {}", lod01_path.display()))
        .and_then(|b| load_glb(&b).map_err(|e| anyhow!("{e}")))?;
    graft_cabin(model, &lod01)
}

#[cfg(test)]
mod cabin_lod_tests {
    use super::*;
    use model::glb::{Material, Mesh, Node, Vertex, IDENTITY};

    fn node(name: &str, parent: Option<usize>) -> Node {
        Node {
            name: name.into(),
            parent,
            translation: [0.0; 3],
            rotation: [0.0, 0.0, 0.0, 1.0],
            scale: [1.0; 3],
            world: IDENTITY,
        }
    }

    fn tri(node: usize, material: usize) -> Mesh {
        Mesh {
            name: "m".into(),
            material,
            vertices: vec![Vertex::default(); 3],
            indices: vec![0, 1, 2],
            node: Some(node),
        }
    }

    #[test]
    fn only_the_cabin_subtree_is_grafted_in() {
        let mut model = Model::default();
        let lod01 = Model {
            nodes: vec![
                node("A380_CABIN", None), // 0: subtree root
                node("SEAT", Some(0)),    // 1: inside
                node("OTHER", None),      // 2: outside, unrelated root
            ],
            materials: vec![Material {
                name: "A380X_BC_SEATS".into(),
                ..Default::default()
            }],
            meshes: vec![tri(1, 0), tri(2, 0)],
            ..Default::default()
        };
        let added = graft_cabin(&mut model, &lod01).unwrap();
        assert_eq!(added, 1, "only the mesh under A380_CABIN is grafted");
        assert_eq!(model.meshes.len(), 1);
        let grafted_node = model.meshes[0].node.unwrap();
        assert_eq!(model.nodes[grafted_node].name, "SEAT");
        let seat_parent = model.nodes[grafted_node].parent.unwrap();
        assert_eq!(
            model.nodes[seat_parent].name, "A380_CABIN",
            "SEAT's parent is remapped to the grafted cabin root, not left pointing at LOD01's own node 0"
        );
    }

    #[test]
    fn a_material_identical_to_one_already_loaded_is_reused_not_duplicated() {
        let mut model = Model {
            materials: vec![Material {
                name: "A380X_BC_SEATS".into(),
                ..Default::default()
            }],
            ..Default::default()
        };
        let lod01 = Model {
            nodes: vec![node("A380_CABIN", None)],
            materials: vec![Material {
                name: "A380X_BC_SEATS".into(),
                ..Default::default()
            }],
            meshes: vec![tri(0, 0)],
            ..Default::default()
        };
        graft_cabin(&mut model, &lod01).unwrap();
        assert_eq!(model.materials.len(), 1, "LOD0 already had this exact material; the graft must not push a second copy");
        assert_eq!(model.meshes[0].material, 0);
    }

    #[test]
    fn missing_a380_cabin_node_is_an_error_not_a_panic() {
        let mut model = Model::default();
        let lod01 = Model {
            nodes: vec![node("SOMETHING_ELSE", None)],
            ..Default::default()
        };
        assert!(graft_cabin(&mut model, &lod01).is_err());
    }
}

/// A 4x4 texture of one colour (RGBA) in `dir`, for materials that have no
/// texture; its file name.
fn solid_texture(dir: &Path, c: [u8; 4]) -> anyhow::Result<String> {
    let name = format!("solid_{:02x}{:02x}{:02x}{:02x}.png", c[0], c[1], c[2], c[3]);
    image::RgbaImage::from_pixel(4, 4, image::Rgba(c)).save(dir.join(&name))?;
    Ok(name)
}

/// The UV-space triangles (`[[u, v]; 3]`, 0..1) of the given meshes: their
/// actual shape, not a bounding box. A shared COMP atlas's real content is
/// often scattered across it, not one contiguous block: FlyByWire's A380
/// packs one small crop per pushbutton or knob cap, placed all over the
/// canvas (see `Job::Material`). A box over every mesh in a `TextureKey`
/// group stretched corner to corner and swallowed the unused padding
/// between crops right along with them - the first thing tried here, and it
/// left the fix a no-op. A box per triangle was still not enough: MSFS's own
/// UV unwrap puts a handful of triangles right on a seam, their three
/// corners legitimately far apart (one edge of a wrapped cylindrical bezel,
/// say), and a handful of seam triangles' bounding boxes were still enough
/// to swallow most of the canvas as "used" between them - only their exact
/// shape (a thin sliver along the real seam, not the box around it) is
/// tight. `dilate_outside_uv` in `msfs2xp::texture` rasterizes these
/// properly rather than their bounding boxes.
fn uv_triangles(model: &Model, meshes: &[usize]) -> Vec<[[f32; 2]; 3]> {
    meshes
        .iter()
        .filter_map(|&mi| model.meshes.get(mi))
        .flat_map(|mesh| {
            mesh.indices.chunks_exact(3).filter_map(move |tri| {
                let a = mesh.vertices.get(tri[0] as usize)?.uv;
                let b = mesh.vertices.get(tri[1] as usize)?.uv;
                let c = mesh.vertices.get(tri[2] as usize)?.uv;
                Some([a, b, c])
            })
        })
        .collect()
}

/// A click shape for a cockpit part: a box 2 mm larger than the part, faced
/// both ways, or the part's own mesh when it is big (a seat, a table, a
/// shade), where a box would cover its neighbours.
fn click_shape(m: &model::glb::Mesh, at: Option<[f32; 3]>) -> model::glb::Mesh {
    /// Parts up to this across get a box.
    const BOX_UP_TO: f32 = 0.6;
    /// How far from its own node a control's click shape may reach. MSFS
    /// poses a few cockpit parts (sunshades, sliding windows) through joints
    /// the file does not list, so they come out of the loader sitting
    /// metres away; boxed as they are, one such part blankets the cockpit in
    /// invisible click geometry and takes the clicks meant for the switches
    /// inside it. Only the vertices around the part's own node are boxed,
    /// which is the part itself: the biggest cockpit control (a sidestick,
    /// a thrust lever) is well under half a metre from its pivot.
    const FROM_ITS_NODE: f32 = 0.5;
    /// Past this a mesh is no longer one control's shape: it is a skinned
    /// part whose rest pose is spread over the whole model. Left as it is,
    /// such a shape blankets the cockpit in invisible click geometry and
    /// takes every click meant for the switches inside it.
    const SPREAD_OVER: f32 = 1.5;
    /// How far from its middle such a part's click box reaches.
    const AROUND_MIDDLE: f32 = 0.75;

    if m.vertices.is_empty() {
        return m.clone();
    }
    let bounds = |points: &[[f32; 3]]| {
        let (mut lo, mut hi) = ([f32::MAX; 3], [f32::MIN; 3]);
        for p in points {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        (lo, hi)
    };
    let points: Vec<[f32; 3]> = m.vertices.iter().map(|v| v.pos).collect();
    let (mut lo, mut hi) = bounds(&points);
    let diag = ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt();
    if (BOX_UP_TO..=SPREAD_OVER).contains(&diag) {
        return m.clone();
    }
    if diag > SPREAD_OVER {
        // Box only what sits around the middle of the part, which is where
        // the control itself is; the strays are the skinning's doing.
        let mut middle = [0.0; 3];
        for k in 0..3 {
            let mut axis: Vec<f32> = points.iter().map(|p| p[k]).collect();
            axis.sort_by(f32::total_cmp);
            middle[k] = axis[axis.len() / 2];
        }
        let near: Vec<[f32; 3]> = points
            .iter()
            .copied()
            .filter(|p| (0..3).all(|k| (p[k] - middle[k]).abs() <= AROUND_MIDDLE))
            .collect();
        (lo, hi) = bounds(if near.is_empty() { &points } else { &near });
    }
    // Whatever the vertices say, a control's click shape covers the part
    // around its own node and nothing else: box the vertices near the node,
    // and where none are (the part is posed away wholesale), a small box on
    // the node itself, which is where the control really sits.
    if let Some(at) = at {
        let near: Vec<[f32; 3]> = points
            .iter()
            .copied()
            .filter(|p| (0..3).all(|k| (p[k] - at[k]).abs() <= FROM_ITS_NODE))
            .collect();
        if near.is_empty() {
            (lo, hi) = ([at[0] - 0.02, at[1] - 0.02, at[2] - 0.02], [at[0] + 0.02, at[1] + 0.02, at[2] + 0.02]);
        } else {
            (lo, hi) = bounds(&near);
        }
    }
    for k in 0..3 {
        lo[k] -= 0.002;
        hi[k] += 0.002;
    }
    let corner = |i: usize| {
        [
            if i & 1 == 0 { lo[0] } else { hi[0] },
            if i & 2 == 0 { lo[1] } else { hi[1] },
            if i & 4 == 0 { lo[2] } else { hi[2] },
        ]
    };
    let vertices = (0..8)
        .map(|i| model::glb::Vertex {
            pos: corner(i),
            normal: [0.0, 1.0, 0.0],
            uv: [0.0, 0.0],
        })
        .collect();
    let mut indices = Vec::with_capacity(72);
    for f in [[0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1], [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3]] {
        for t in [[f[0], f[1], f[2]], [f[0], f[2], f[3]]] {
            indices.extend(t.map(|x: usize| x as u32));
            indices.extend([t[0], t[2], t[1]].map(|x: usize| x as u32));
        }
    }
    model::glb::Mesh {
        name: m.name.clone(),
        material: m.material,
        vertices,
        indices,
        node: m.node,
    }
}

/// The interior model XML named in the aircraft's model.cfg (`interior=`).
fn interior_xml(plane: &Path) -> Option<PathBuf> {
    let dir = plane.join("model");
    let cfg = std::fs::read_dir(&dir).ok()?.filter_map(Result::ok).map(|e| e.path()).find(|p| {
        p.file_name().is_some_and(|n| n.to_string_lossy().eq_ignore_ascii_case("model.cfg"))
    })?;
    let text = std::fs::read_to_string(cfg).ok()?;
    let name = text.lines().find_map(|l| {
        let (k, v) = l.split(';').next()?.split_once('=')?;
        k.trim().eq_ignore_ascii_case("interior").then(|| v.trim().to_string())
    })?;
    let path = dir.join(name.replace('\\', "/"));
    path.is_file().then_some(path)
}

/// MSFS's own model behaviour definitions, next to the Community folder the
/// package is in: `Official\OneStore` (Microsoft Store) or `Official\Steam`.
fn asobo_behaviours(package: &Path) -> Option<PathBuf> {
    let packages = package.parent()?.parent()?;
    ["OneStore", "Steam"]
        .iter()
        .map(|s| packages.join("Official").join(s).join("fs-base-aircraft-common").join("ModelBehaviorDefs"))
        .find(|p| p.is_dir())
}

/// A mesh's light: the `ATTR_light_level` of the innermost node above it
/// (or itself) with an emissive code, and whether that code lights it.
fn light_of(animator: &Animator, rig: &rig::Rig, node: Option<usize>) -> (Option<String>, Option<bool>) {
    let Some(node) = node else { return (None, None) };
    for n in animator.chain(node).into_iter().rev() {
        if let Some(&lit) = rig.light_driven.get(&n) {
            return (rig.light_levels.get(&n).cloned(), Some(lit));
        }
    }
    (None, None)
}

/// Anchors `ATTR_light_level`'s optional fourth argument -- a maximum
/// brightness in nits, which switches on X-Plane 12's photometric renderer
/// for that surface -- to a material's glTF `emissiveFactor`, for the one
/// material on the whole aircraft MSFS drives past white:
/// `A380X_FC_DECK_SIDWEALLS` at `[5.0, 5.0, 5.0]`, with no
/// `KHR_materials_emissive_strength` extension to say how many real nits
/// that means. `tint`/`lit_tint` (`glb.rs`, `obj8.rs`) can only dim a baked
/// LDR night texture, never brighten it past what its own texels already
/// hold, so today a factor above 1.0 is silently thrown away and the
/// surface draws no brighter than an ordinary one -- this is the only lever
/// OBJ8 gives back for it.
///
/// `BASELINE_NITS` is what an *ordinary* (factor 1.0) backlit cockpit
/// surface is worth: not a measurement of this aircraft, but the same order
/// of magnitude Laminar's own aircraft already use for plain instrument
/// backlighting (`instrument_brightness_ratio[4] 1000`, Vans
/// RV-10_interior1.obj) -- a factor above white asks for that many times
/// brighter, not an arbitrary absolute number.
fn hdr_light_level(base: Option<String>, emissive_factor: [f32; 3]) -> Option<String> {
    let peak = emissive_factor.iter().fold(0.0f32, |a, &x| a.max(x));
    if peak <= 1.0 {
        return base;
    }
    let nits = (BASELINE_NITS * peak).round() as i64;
    Some(match base {
        // Already gated by a real dataref (a switch, a dimmer): keep the
        // gate, just add the ceiling.
        Some(l) => format!("{l} {nits}"),
        // MSFS drives this surface with no controlling code at all (a
        // decorative glow, not a switch) -- `light_of` above has no dataref
        // to attach the nits to, but OBJ8 requires one even for a level
        // that never changes. `sim/version/xplane_internal_version` is
        // X-Plane's own build number: always far above 1 in any real
        // session, so `0 1` clamps to the v2 end permanently -- an
        // always-on level with no new dataref this converter would have to
        // publish itself.
        None => format!("0 1 {ALWAYS_ON_DATAREF} {nits}"),
    })
}

/// See `hdr_light_level`.
const BASELINE_NITS: f32 = 1000.0;
/// See `hdr_light_level`.
const ALWAYS_ON_DATAREF: &str = "sim/version/xplane_internal_version";

#[cfg(test)]
mod hdr_light_tests {
    use super::*;

    #[test]
    fn an_ordinary_or_dimmed_factor_leaves_the_light_line_untouched() {
        let base = Some("0 1 fbw/cockpit/lt/windshield_frame".to_string());
        assert_eq!(hdr_light_level(base.clone(), [1.0, 1.0, 1.0]), base);
        assert_eq!(hdr_light_level(base.clone(), [0.7, 0.7, 0.7]), base, "T12's dimming stays a tint, not a nits ceiling");
        assert_eq!(hdr_light_level(None, [1.0, 1.0, 1.0]), None, "no code, ordinary factor: still no line, as today");
    }

    #[test]
    fn an_over_driven_factor_appends_nits_to_an_existing_gated_light() {
        let base = Some("0 1 fbw/cockpit/lt/deck_wash".to_string());
        assert_eq!(hdr_light_level(base, [5.0, 5.0, 5.0]), Some("0 1 fbw/cockpit/lt/deck_wash 5000".to_string()));
    }

    #[test]
    fn an_over_driven_factor_with_no_code_gets_a_synthetic_always_on_line() {
        // FlyByWire's A380X_FC_DECK_SIDWEALLS: no switch/dimmer code at all.
        assert_eq!(hdr_light_level(None, [5.0, 5.0, 5.0]), Some(format!("0 1 {ALWAYS_ON_DATAREF} 5000")));
    }
}

/// XML controls with a click spot: (controls whose node or clip is in the
/// model, of those with a click spot on it).
fn click_coverage(model: &Model, animator: &Animator, res: &behaviour::Resolution, spots: &[Option<usize>]) -> (usize, usize, Vec<String>) {
    let by_name: HashMap<String, usize> = model.nodes.iter().enumerate().map(|(i, n)| (n.name.trim().to_ascii_lowercase(), i)).collect();
    let clip_nodes = |anim: &str| -> Vec<usize> {
        let a = anim.trim();
        match model.clips.iter().position(|c| c.name.trim().eq_ignore_ascii_case(a)) {
            Some(ci) => (0..model.nodes.len()).filter(|&n| animator.clips_at(n).contains(&ci)).collect(),
            None => Vec::new(),
        }
    };
    let spot_chains: Vec<HashSet<usize>> = spots.iter().flatten().map(|&n| animator.chain(n).into_iter().collect()).collect();
    let controls = res
        .bindings
        .iter()
        .map(|b| (b.node.clone(), b.anim.clone()))
        .chain(res.unresolved.iter().map(|u| (u.node.clone(), u.anim.clone())))
        .chain(res.levers.iter().map(|l| (l.clone(), l.clone())));
    let (mut present, mut covered, mut missing) = (0, 0, Vec::new());
    for (node, anim) in controls {
        let mut want: Vec<usize> = clip_nodes(&anim);
        want.extend(by_name.get(&node.trim().to_ascii_lowercase()));
        if want.is_empty() {
            continue;
        }
        present += 1;
        if spot_chains.iter().any(|c| want.iter().any(|w| c.contains(w))) {
            covered += 1;
        } else {
            missing.push(if node.eq_ignore_ascii_case(&anim) { anim } else { format!("{anim} (node {node})") });
        }
    }
    (present, covered, missing)
}

/// Every real breaker on the A380X cockpit model's own overhead/avionics-bay
/// circuit-breaker panel (`a380_cockpit.gltf`, node names `CB_*`), in the
/// order its synthetic circuit number (`PANEL_ONLY_BASE + index`) is built
/// from. FlyByWire's behaviour XML never makes these clickable and MSFS's
/// simplified systems.cfg electrical model has no `CIRCUIT_*` for any of
/// them (ATC, FMC, CIDS, GCU, ... are real-A380 avionics-bay systems it
/// never represents as their own circuit); see fbw-xp-systems's
/// `circuits.rs` for that check. The 8 `CB_EMPTYn` nodes are blank filler
/// plates on the real panel and are left out (and left plain).
///
/// Keep this exact order and spelling in sync with fbw-xp-systems's own
/// copy (`src/circuits.rs` `PANEL_CB_NODES`) by hand: the two crates share
/// no code, and the synthetic number this list and that one agree on only
/// matches because the index does.
const PANEL_CB_NODES: &[&str] = &[
    "CB_AESU1",
    "CB_AESU2",
    "CB_AICU1",
    "CB_AICU2",
    "CB_ARPT_NAV",
    "CB_ATC",
    "CB_AVS1",
    "CB_AVS2",
    "CB_BSCS1",
    "CB_BSCS2",
    "CB_CIDS1",
    "CB_CIDS2",
    "CB_CIDS3",
    "CB_CPCS1",
    "CB_CPCS2",
    "CB_DSMS",
    "CB_DTLNKROUTER",
    "CB_ENG1_EIPM2",
    "CB_ENG2_EIPM1",
    "CB_ENG3_EIPM2",
    "CB_ENG4_EIPM1",
    "CB_ESS_TR",
    "CB_FLAPS1",
    "CB_FLAPS2",
    "CB_FMC_A",
    "CB_FMC_B",
    "CB_FMC_C",
    "CB_FQMS1",
    "CB_FQMS2",
    "CB_FWS1",
    "CB_FWS2",
    "CB_GCU",
    "CB_LGCIS1",
    "CB_LGCIS2",
    "CB_NSS_AVNCS",
    "CB_NSS_FLT_OPS",
    "CB_PACK1_CTL",
    "CB_PACK2_CTL",
    "CB_PAX_BBAND",
    "CB_SCS1",
    "CB_SCS2",
    "CB_SDF1",
    "CB_SDF2",
    "CB_SDF3",
    "CB_SLAT2",
    "CB_SLATS1",
    "CB_TCS1",
    "CB_TCS2",
    "CB_TR1",
    "CB_TR_2A",
    "CB_VCS1",
    "CB_VCS2",
];

/// Matches `circuits::PANEL_ONLY_BASE` in fbw-xp-systems: well above any real
/// `circuit.N` in the embedded systems.cfg, so a panel-only breaker's number
/// can't collide with a real circuit's.
const PANEL_ONLY_BASE: usize = 10_000;

/// `PANEL_CB_NODES`'s synthetic circuit number for a node name, or `None` for
/// a name not in the list (including the `CB_EMPTYn` spares).
fn panel_cb_number(node_name: &str) -> Option<usize> {
    PANEL_CB_NODES.iter().position(|&n| n == node_name).map(|i| PANEL_ONLY_BASE + i)
}

/// A circuit breaker's pull-out translation: the average of its own vertex
/// normals (a breaker cap is a small convex button, so this is roughly the
/// direction its face points, i.e. out of the panel), scaled to a realistic
/// throw and flipped into the same right-handed, X/Z-negated space
/// `write_obj8_animated` bakes vertex positions into. `[0.0; 3]` for a mesh
/// with no vertices (never clickable then either — see `circuit_breakers`).
const CB_PULL_M: f32 = 0.010; // ~10 mm: a typical A320/A380 CB throw.

fn cb_pull_offset(mesh: &model::glb::Mesh) -> [f32; 3] {
    let mut n = [0.0f32; 3];
    for v in &mesh.vertices {
        n[0] += v.normal[0];
        n[1] += v.normal[1];
        n[2] += v.normal[2];
    }
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if len < 1e-6 {
        return [0.0; 3];
    }
    [-n[0] / len * CB_PULL_M, n[1] / len * CB_PULL_M, -n[2] / len * CB_PULL_M]
}

/// Every mesh on the cockpit's own CB panel geometry that [`PANEL_CB_NODES`]
/// names, with the extra `ANIM_trans` command that pulls it and the
/// `ATTR_manip_toggle` that lets a click drive it — both bound to the
/// systems plugin's own writable breaker dataref (fbw-xp-systems
/// `circuits.rs`, published as `fbw/CIRCUIT_BREAKER_CLOSED_<n>`; pulling one
/// currently gates nothing further downstream, same as any circuit number
/// outside the embedded systems.cfg — see `PANEL_CB_NODES`'s own doc
/// comment). Keyed by mesh index, for the cockpit model's own `visible`
/// closure to fold in.
fn circuit_breakers(model: &Model) -> HashMap<usize, (String, String)> {
    let mut out = HashMap::new();
    for (mi, mesh) in model.meshes.iter().enumerate() {
        let Some(node) = mesh.node else { continue };
        let Some(name) = model.nodes.get(node).map(|n| n.name.as_str()) else { continue };
        let Some(number) = panel_cb_number(name) else { continue };
        let dataref = behaviour::bind::dataref(&format!("CIRCUIT BREAKER CLOSED:{number}"));
        let [dx, dy, dz] = cb_pull_offset(mesh);
        let commands = format!("ANIM_trans {dx:.4} {dy:.4} {dz:.4} 0 0 0 0 1 {dataref}\n");
        let label = name.trim_start_matches("CB_").replace('_', " ");
        let manip = format!("ATTR_manip_toggle hand 1 0 {dataref} {label} circuit breaker");
        out.insert(mi, (commands, manip));
    }
    out
}

/// Whether an object's own text carries a manipulator of the kind X-Plane's
/// single-cockpit-object arbitration cares about: a classic
/// `ATTR_manip_*` (toggle, push, drag, command, axis knob, ...), which only
/// the one object it picks per aircraft is ever tested for.
///
/// `ATTR_manip_device` is excluded on purpose. Pre-merge (`Log-keep-
/// 113715.txt`), object 3 (the EFB and MFD among its seventeen screens)
/// logged "0 will be used and 3 will be skipped" -- X-Plane's arbitration
/// picked object 0 instead -- and object 3's screens still took real taps
/// ("input callback: touch on SCREEN_EFB", "touch on SCREEN_DU_MFD"). A
/// device's own manipulator plainly is not gated by that pick, so counting
/// it here would only cost a screen's object the classification it needs
/// for nothing: it would make that object compete for the one click-tested
/// slot the actual click object (2137 ordinary manipulators) has to win,
/// exactly the merge this codebase already backed out of once (see
/// `555500a`, `5d9a222`).
///
/// `ATTR_manip_none` (the reset every manipulator, device or not, closes
/// with) is excluded too -- it says nothing about which kind opened it.
fn has_classic_manipulator(text: &str) -> bool {
    text.lines().any(|l| l.starts_with("ATTR_manip_") && !l.starts_with("ATTR_manip_device") && l != "ATTR_manip_none")
}

/// The clickable manipulator for a mesh: the one of the innermost node on
/// its chain that the behaviour XML makes clickable or a clip moves.
fn manip_for(animator: &Animator, rig: &rig::Rig, node: Option<usize>) -> Option<String> {
    let node = node?;
    for n in animator.chain(node).into_iter().rev() {
        if let Some((_, m)) = rig.node_manips.get(&n) {
            return Some(m.clone());
        }
        let clips = animator.clips_at(n);
        if clips.is_empty() {
            continue;
        }
        return clips.iter().filter_map(|c| rig.manips.get(c)).max_by_key(|m| m.0).map(|m| m.1.clone());
    }
    None
}

/// Vmo (kt) and Mmo: `--vmo`/`--mmo` when given, otherwise FlyByWire's A380X
/// published figures (`fbw-a380x/src/systems/shared/src/
/// PerformanceConstants.ts:1-2`, `Vmo = 340`/`Mmo = 0.89`) -- the normal
/// operating limit ("barber pole") speeds, not flight_model.cfg's own
/// `max_indicated_speed`/`max_mach` (MSFS's overspeed-DAMAGE thresholds,
/// 390 kt/0.97 for this airframe -- see the comment on `acf::Inputs::vmo`).
/// Defaulting here means a build with neither flag never silently keeps the
/// `--acf-template`'s own Vmo/Mmo (Laminar's A330: 330 kt/0.86), as the
/// current install did.
fn vmo_mmo_or_default(vmo: Option<f64>, mmo: Option<f64>) -> (f64, f64) {
    const DEFAULT_VMO_KTS: f64 = 340.0;
    const DEFAULT_MMO: f64 = 0.89;
    (vmo.unwrap_or(DEFAULT_VMO_KTS), mmo.unwrap_or(DEFAULT_MMO))
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();
    let title = manifest_title(&args.package).unwrap_or_else(|| "Converted aircraft".into());
    let name = safe(args.name.as_deref().unwrap_or(&title));
    let root = args.out.join(&name);
    let objects = root.join("objects");
    std::fs::create_dir_all(&objects)?;
    let plane = aircraft_dir(&args.package)?;
    println!("{title}: {}", plane.display());

    // The model's own textures; built-in variants become liveries.
    let variants = variants(&args.package, &plane);
    let base_dirs: Vec<PathBuf> = texture_dirs(&args.package.join("SimObjects"))
        .into_iter()
        .filter(|d| !variants.iter().any(|(_, v)| d.starts_with(v)))
        .collect();
    let index = texture_index(&base_dirs);
    println!("  {} texture files", index.len());
    let mut plan = Plan::default();
    // Which textures the exterior uses, and which the cockpit and cabin do.
    //
    // A texture is only capped at `--exterior-max` when the *interior* does
    // not use it as well. Being referenced by the exterior model is not
    // enough on its own: that model carries cockpit materials it never
    // draws -- its nose skin has the cockpit windows painted on, and the
    // overhead and avionics-bay panels ride along in it -- so anything they
    // touch landed in this set and was capped at exterior resolution.
    //
    // On the A380 that was 43 cockpit albedos halved from 4096 to 2048, and
    // every one of them was used *only* by cockpit objects: not one was
    // shared with the exterior, so nothing was being traded off. It was the
    // panel lettering, at half the resolution it was authored at, which is
    // exactly what `--interior-max`'s own doc comment says must not happen.
    let mut ext_textures: HashSet<String> = HashSet::new();
    let mut int_textures: HashSet<String> = HashSet::new();
    // Colour/emissive textures referenced only by the passenger cabin, and
    // only by the cockpit, respectively -- so a texture used by both (rare:
    // `graft_cabin` dedupes a cabin material identical to one LOD0 already
    // has) falls back to the cockpit's own, uncapped-by-`--cabin-max`
    // treatment rather than risking the cockpit's copy being crushed.
    let mut cabin_textures: HashSet<String> = HashSet::new();
    let mut cockpit_int_textures: HashSet<String> = HashSet::new();
    let mut object_files: Vec<(String, ObjKind)> = Vec::new();
    // Each written object's extent (see where it is filled), by file name.
    let mut object_bounds: HashMap<String, ([f64; 3], [f64; 3])> = HashMap::new();
    let mut exterior: Option<Model> = None;
    let mut rigs: Vec<rig::Rig> = Vec::new();
    let mut notes = Vec::new();
    let systems_cfg = std::fs::read_to_string(plane.join("systems.cfg")).unwrap_or_default();
    // The simulator's state before anything is clicked: the flight the
    // aircraft starts with and its electrical definition.
    let apron = std::fs::read_dir(&plane)
        .ok()
        .and_then(|d| d.filter_map(Result::ok).map(|e| e.path()).find(|p| p.file_name().is_some_and(|n| n.to_string_lossy().eq_ignore_ascii_case("apron.flt"))))
        .and_then(|p| std::fs::read_to_string(p).ok())
        .unwrap_or_default();
    let sim = behaviour::sim::sim_state(&apron, &systems_cfg);
    // The glass cockpit's screens, which the systems plugin draws.
    let panel_cfg = std::fs::read_to_string(plane.join("panel").join("panel.cfg")).unwrap_or_default();
    let screen_list = screens::parse_panel(&panel_cfg, &args.skip_screens);

    // What each cockpit control does, from the behaviour XML.
    let behaviour_xml = args.behaviour.clone().or_else(|| interior_xml(&plane));
    let asobo = args.asobo_behaviours.clone().or_else(|| asobo_behaviours(&args.package));
    let resolution = match &behaviour_xml {
        Some(x) => match behaviour::resolve(x, asobo.as_deref(), &sim) {
            Ok((r, warn)) => {
                println!("  behaviour: {} ({} controls)", x.display(), r.bindings.len() + r.unresolved.len() + r.levers.len());
                for w in warn {
                    println!("  behaviour: {w}");
                    notes.push(format!("behaviour: {w}"));
                }
                Some(r)
            }
            Err(e) => {
                eprintln!("  behaviour: {}: {e:#}; cockpit controls keep their own datarefs", x.display());
                None
            }
        },
        None => {
            println!("  behaviour: no interior model XML found; cockpit controls keep their own datarefs");
            None
        }
    };
    let mut bindings_report: Vec<String> = Vec::new();

    // Models: LOD 0 of each, one object per texture set, animated.
    let mut models: Vec<PathBuf> = std::fs::read_dir(plane.join("model"))?
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x.eq_ignore_ascii_case("gltf")))
        .filter(|p| !stem(&p.file_name().unwrap_or_default().to_string_lossy()).to_ascii_lowercase().contains("_lod"))
        .collect();
    models.sort();
    if args.no_models {
        models.clear();
    }
    for gltf in &models {
        let model_name = gltf.file_stem().unwrap_or_default().to_string_lossy().to_string();
        let mut model: Model = match gltf_as_glb(gltf).and_then(|b| load_glb(&b).map_err(|e| anyhow!("{e}"))) {
            Ok(m) => m,
            Err(e) => {
                eprintln!("  {model_name}: skipped ({e:#})");
                continue;
            }
        };
        let is_ext = model_name.to_ascii_lowercase().contains("exterior");
        lift_decals(&mut model, !is_ext);
        // The passenger cabin: most of the cockpit model, all behind the
        // cockpit door; left out unless asked for. With --cabin, LOD0's own
        // cabin (5.65M triangles) is swapped for the same `a380_cabin`
        // subtree in the package's LOD01 model, 180k triangles built from
        // the same textures (see `splice_lod01_cabin`).
        if !is_ext {
            if let Some(cab) = model.nodes.iter().position(|n| n.name.trim().eq_ignore_ascii_case("a380_cabin")) {
                let inside = node_subtree(&model.nodes, cab);
                let before: usize = model.triangle_count();
                model.meshes.retain(|m| !m.node.is_some_and(|n| inside[n]));
                model.click_spots.retain(|m| !m.node.is_some_and(|n| inside[n]));
                let removed = before - model.triangle_count();
                if !args.no_cabin {
                    match splice_lod01_cabin(&mut model, gltf) {
                        Ok(added) => println!(
                            "  {model_name}: passenger cabin swapped for the LOD01 model ({removed} LOD0 triangles out, {added} LOD01 triangles in)"
                        ),
                        Err(e) => {
                            eprintln!("  {model_name}: could not splice the LOD01 cabin ({e:#}); passenger cabin left out (--no-cabin also does this, on purpose)");
                            notes.push(format!("{model_name}: could not splice the LOD01 cabin ({e:#})"));
                        }
                    }
                } else {
                    println!("  {model_name}: passenger cabin left out ({removed} triangles; --no-cabin was given)");
                }
            }
        }
        // Debug: MSFS2XP_REGION="x0,x1,y0,y1,z0,z1" (X-Plane metres) lists
        // the meshes reaching into that box.
        if let Ok(r) = std::env::var("MSFS2XP_REGION") {
            let b: Vec<f32> = r.split(',').filter_map(|v| v.trim().parse().ok()).collect();
            if b.len() == 6 {
                for m in &model.meshes {
                    let n = m
                        .vertices
                        .iter()
                        .filter(|v| {
                            let (x, y, z) = (-v.pos[0], v.pos[1], -v.pos[2]);
                            x > b[0] && x < b[1] && y > b[2] && y < b[3] && z > b[4] && z < b[5]
                        })
                        .count();
                    if n > 0 {
                        let node = m.node.map_or("-", |i| model.nodes[i].name.as_str());
                        let mat = model.materials.get(m.material).map_or("-", |x| x.name.as_str());
                        println!("  region: {n:6} verts  mesh {} node {node} material {mat}", m.name);
                    }
                }
            }
        }
        // MSFS light-node meshes: only the ones with no real authored
        // material are left out; the rest is real airframe surface and
        // stays. Inspecting a380_exterior.gltf directly (its materials
        // array) shows two very different kinds of geometry hanging off
        // LIGHT_ASOBO_*/LIGHT_AMBIENT_* nodes and systems.cfg's own EmMesh
        // targets:
        //   - A380X_ACCESSORIES (the light housings/bezels, e.g.
        //     LIGHT_ASOBO_TURNOFF_LH, LIGHT_ASOBO_LAND_1_LH,
        //     LIGHT_ASOBO_TAXI_WING_LH, LIGHT_ASOBO_OBSTRUCTION_LH),
        //     A380X_LIGHT_GLASS (the BLEND glass dome over a landing-light
        //     bulb), NAVIGATION_LIGHT_RED/GREEN and STROBE_LIGHT (near-black
        //     baseColorFactor + a baked emissiveFactor -- a real, always-
        //     opaque dark lens that MSFS itself keeps drawing when the
        //     light is off; systems.cfg's own doc comment confirms EmMesh
        //     only toggles *emission*, "this particular emissive will stop
        //     emitting light" -- not the mesh's visibility). Dropping any
        //     of these outright punches a hole in the real airframe surface
        //     (a socket with nothing in it).
        //   - glTFValidator_Added_Default_Material: a filler the export
        //     pipeline stamps on a primitive that was never given a real
        //     material at all (seen only on the LOGO_AFT/FWD_LH/RH and
        //     TAXI_CAM_FWD_LH/RH nodes) -- not a modelled surface, so
        //     leaving these out costs nothing.
        // X-Plane's own LIGHT_PARAM lights (lights.rs's `lights_obj`) still
        // add the switch-driven glow/beam on top of this now-kept geometry,
        // at the same node position -- the same "always-visible base +
        // glow when powered" split MSFS itself uses, just with X-Plane's
        // native light glow standing in for MSFS's emissive-material pass.
        if is_ext && !systems_cfg.is_empty() {
            let glow = lights::glow_meshes(&systems_cfg);
            let hidden: Vec<bool> = (0..model.nodes.len())
                .map(|i| {
                    let mut n = Some(i);
                    for _ in 0..128 {
                        let Some(k) = n else { break };
                        // Blender copies carry ".004"-style suffixes; lamp
                        // meshes on MSFS's LIGHT_ nodes are light-node too.
                        let name = model.nodes[k].name.trim().to_ascii_lowercase();
                        let base = name.split('.').next().unwrap_or(&name).to_string();
                        if glow.contains(&name) || glow.contains(&base) || base.starts_with("light_asobo_") || base.starts_with("light_ambient_") {
                            return true;
                        }
                        n = model.nodes[k].parent;
                    }
                    false
                })
                .collect();
            let before = model.meshes.len();
            model.meshes.retain(|m| {
                let on_a_light_node = m.node.is_some_and(|n| hidden[n]);
                let no_real_material = model.materials.get(m.material).is_none_or(|mat| mat.name.starts_with("glTFValidator") || mat.name.is_empty());
                !(on_a_light_node && no_real_material)
            });
            println!("  {model_name}: {} placeholder-material light-node meshes left out", before - model.meshes.len());
        }
        // Which nodes are the (freshly grafted, if present) passenger
        // cabin: a fresh lookup, not the `inside` used to strip LOD0's own
        // cabin above. `model.nodes.retain`/`.meshes.retain` above only ever
        // drop *meshes*, never nodes, so LOD0's own (now mesh-less)
        // `a380_cabin` node is still sitting in `model.nodes` at its
        // original index; `splice_lod01_cabin` then appends a brand new
        // node of the very same name (and its own subtree) after it. A
        // plain `position` (first match) would find LOD0's stale one and
        // report every mesh as non-cabin -- `rposition` (last match) is
        // what actually finds the grafted subtree.
        let cabin_subtree: Vec<bool> = match model.nodes.iter().rposition(|n| n.name.trim().eq_ignore_ascii_case("a380_cabin")) {
            Some(root) => node_subtree(&model.nodes, root),
            None => vec![false; model.nodes.len()],
        };
        let is_cabin_mesh = |mi: usize| model.meshes.get(mi).is_some_and(|m| m.node.is_some_and(|n| cabin_subtree[n]));
        let clips = ClipIndex::new(&model);
        let rig = if is_ext { rig::exterior(&model, &clips, resolution.as_ref()) } else { rig::cockpit(&model, &clips, resolution.as_ref()) };
        bindings_report.extend(rig.bindings.iter().cloned());
        for r in &rig.report {
            println!("  {model_name}: {r}");
            notes.push(format!("{model_name}: {r}"));
        }
        {
            let animator = Animator::new(&model, &clips, rig.drefs.clone(), rig.vis.clone());
            // Overhead/avionics-bay CB panel geometry: not exterior, and not
            // in rig's own clip/behaviour-driven manips (no clip touches
            // these nodes), so folded into `visible` separately below.
            let cb: HashMap<usize, (String, String)> = if is_ext { HashMap::new() } else { circuit_breakers(&model) };
            if !cb.is_empty() {
                println!("  {model_name}: {} circuit breakers made clickable ({} panel labels have no matching systems.cfg circuit)", cb.len(), cb.len());
            }
            // The exterior's nose skin has the cockpit windows painted on:
            // solid over the windscreen. MSFS does not draw it from the
            // cockpit; drawn here it shades the whole cockpit (dark panels, a
            // dark view out). Hidden while the view is inside.
            let nose_skin: HashSet<usize> = if is_ext {
                (0..model.meshes.len())
                    .filter(|&mi| {
                        let m = &model.meshes[mi];
                        let mat = model.materials.get(m.material).map_or(String::new(), |x| x.name.to_ascii_uppercase());
                        if !(mat.contains("FUSE") || mat.contains("RADOME")) || m.vertices.is_empty() {
                            return false;
                        }
                        let inside = m
                            .vertices
                            .iter()
                            .filter(|v| {
                                let (x, y, z) = (-v.pos[0], v.pos[1], -v.pos[2]);
                                x.abs() < 4.0 && y > -1.0 && y < 6.0 && z > -38.0 && z < -26.0
                            })
                            .count();
                        inside * 100 / m.vertices.len() >= 15
                    })
                    .collect()
            } else {
                HashSet::new()
            };
            if !nose_skin.is_empty() {
                println!("  {model_name}: {} nose skin meshes hidden from the cockpit view", nose_skin.len());
            }
            // The screen this mesh draws, if any (`None` in the exterior
            // pass, whose meshes carry no screen material).
            let screen_of = |mi: usize| {
                if is_ext {
                    None
                } else {
                    model.materials.get(model.meshes[mi].material).and_then(|m| screens::screen_of(&screen_list, &m.name))
                }
            };
            let visible = |mi: usize| {
                let sc = screen_of(mi);
                MeshAnim {
                    commands: {
                        let c = animator.commands(model.meshes[mi].node);
                        let c = if nose_skin.contains(&mi) {
                            format!("ANIM_hide -0.5 0.5 sim/graphics/view/view_is_external\n{c}")
                        } else if let Some((cb_cmd, _)) = cb.get(&mi) {
                            format!("{c}{cb_cmd}")
                        } else {
                            c
                        };
                        // A runtime, no-reconversion way to hide the cabin
                        // for frame rate (`fbw/options/cabin_visible`,
                        // created in `main_lua`, default 1/shown): unlike
                        // `--no-cabin`, this only saves draw time -- the
                        // textures still load -- but needs no reconversion
                        // and a later settings UI can flip it in flight.
                        if is_cabin_mesh(mi) {
                            format!("ANIM_show 1 1 fbw/options/cabin_visible\n{c}")
                        } else {
                            c
                        }
                    },
                    // The breakers' manipulators live in the click object
                    // (see there); leaving a copy here would make this
                    // object claim the interior-cockpit flag too, and
                    // X-Plane's single-cockpit-object arbitration would cost
                    // it the flag it actually needs to be click-tested.
                    //
                    // A touch screen's own `ATTR_manip_device` is different:
                    // proven pre-merge (`Log-keep-113715.txt`: "input
                    // callback: touch on SCREEN_EFB", "touch on
                    // SCREEN_DU_MFD"), it kept taking real taps in the
                    // object that draws it even though that object was not
                    // the one X-Plane's arbitration picked ("This aircraft
                    // has two interior cockpit objects; 0 will be used and 3
                    // will be skipped" -- object 3 held the EFB and MFD).
                    // So it stays here, on the screen's own triangles, which
                    // is also where the spec requires it to be (OBJ8 spec:
                    // "the manipulator must be the same shape, and
                    // UV-mapped the same way as the screen of the cockpit
                    // device"). Only the screens the real aircraft is
                    // actually touched on get one (`screens::TOUCH_SCREENS`,
                    // `has_classic_manipulator` below keeps this out of the
                    // click-object competition either way).
                    manip: sc
                        .filter(|s| screens::TOUCH_SCREENS.contains(&s.id.as_str()))
                        .map(|s| format!("ATTR_manip_device hand {} {}", s.id, s.id)),
                    hidden: false,
                    click: false,
                    light: hdr_light_level(
                        light_of(&animator, &rig, model.meshes[mi].node).0,
                        model.materials.get(model.meshes[mi].material).map_or([0.0; 3], |m| m.emissive_factor),
                    ),
                    device: sc.map(|sc| format!("{} {}", sc.id, screens::DEVICE_ARGS)),
                }
            };
            // Meshes under a node with an emissive code glow as the code says,
            // whatever their material's own emissive factor.
            let driven = |mi: usize| light_of(&animator, &rig, model.meshes[mi].node).1;
            if !rig.light_driven.is_empty() {
                let (mut lit, mut dark, mut plain) = (0, 0, 0);
                for (mi, m) in model.meshes.iter().enumerate() {
                    if driven(mi) == Some(true) {
                        match model.materials.get(m.material).and_then(|x| x.emissive.as_ref()) {
                            Some(_) => lit += 1,
                            None => plain += 1,
                        }
                    } else if driven(mi) == Some(false) {
                        dark += 1;
                    }
                }
                let line = format!(
                    "cockpit lights: {lit} meshes glow by their emissive code, {plain} under an emissive code have no emissive texture to glow with, {dark} never glow (code always 0)"
                );
                println!("  {model_name}: {line}");
                notes.push(format!("{model_name}: {line}"));
            }
            let groups = model::obj8::split_by_texture_lit(&model, true, &driven);
            let mut tris = 0;
            // Exterior LOD chain (`--no-exterior-lods` to disable): triangle
            // totals per level for the report, and objects meshoptimizer
            // could not bring within reach of the ratio asked for (a whole
            // mesh made only of triangles it cannot collapse without moving
            // a locked border vertex -- see `simplify_mesh`).
            let mut ext_lod_tris = [0usize; 3];
            let mut ext_lod_objects = 0usize;
            let mut ext_lod_excluded_objects = 0usize;
            let mut ext_lod_failures: Vec<String> = Vec::new();
            // Meshes where L2's border-locked simplify missed its ratio by
            // more than 1.5x, so a topology-ignoring sloppy pass ran too.
            let mut ext_lod_l2_sloppy = 0usize;
            // MSFS ships no lower-detail exterior model to swap in at
            // distance (see the tire-object investigation: the tire/wheel
            // assembly alone, material `A380_EXTERIOR_TYRES`, is 868,432
            // triangles, 30% of the whole exterior, drawn unconditionally).
            // A full tire is 1.4 m; at 1500 m and 60 degrees FOV on a
            // 2560-wide (1440p) screen that projects to under 2.3 px,
            // already smaller than anti-aliasing resolves, so an exterior
            // group made entirely of tire material stops being drawn past
            // there -- no draw call anyone could tell apart from one that
            // still ran.
            const TIRE_LOD_TRIS: usize = 10_000;
            const TIRE_LOD_FAR_M: f32 = 1500.0;
            // Every cockpit object gets an explicit range. Without one the
            // OBJ8 spec has X-Plane calculate it "automatically", and it
            // measures from the object's origin -- the aircraft reference
            // point, about 33 m from the pilot -- so a small object is
            // culled while the pilot is looking right at it. The likely
            // cause of the A380's 18 cm speed brake plate (its own object,
            // for its LIT texture) showing as the black SUNBLOCKER under it,
            // with only the lettering decal on top, when its texture,
            // normals and material all read blue-grey (2026-09-28). 1000 m
            // covers every view that can see into the cockpit.
            const COCKPIT_LOD_FAR_M: f32 = 1000.0;
            for (i, (key, meshes)) in groups.iter().enumerate() {
                let group_tris = meshes.iter().map(|&m| model.meshes[m].indices.len() / 3).sum::<usize>();
                let is_tire = is_ext
                    && group_tris > TIRE_LOD_TRIS
                    && meshes.iter().all(|&m| {
                        model
                            .materials
                            .get(model.meshes[m].material)
                            .is_some_and(|mat| mat.name.to_ascii_uppercase().contains("TYRE") || mat.name.to_ascii_uppercase().contains("TIRE"))
                    });
                // Every mesh in this group is part of the grafted passenger
                // cabin (a mixed group -- rare, see `graft_cabin`'s material
                // dedup -- falls back to the cockpit's own treatment: still
                // drawn correctly, just not distance-culled or shown from
                // outside).
                let is_cabin_group = !is_ext && !meshes.is_empty() && meshes.iter().all(|&mi| is_cabin_mesh(mi));
                // This group's own share of its normal and COMP textures'
                // UV coverage (see `Job::Normal`, `Job::Material`): every
                // mesh in it points at the same `key.normal`/`key.metal_rough`.
                let group_used = uv_triangles(&model, meshes);
                // Interior glass with its own metal/roughness source but no
                // normal map (the DU/EFB display covers, cabin windows: glass
                // has no relief to bump-map) currently drew with no shine at
                // all, unlike the windshield. FBW's own glass shader marks
                // these panes `glassReflectionMaskFactor: 0.5`, half the
                // windshield's implicit full strength, so a screen or a dial
                // sitting right behind the pane stays readable through it.
                let glass_reflect = !is_ext && key.glass && key.normal.is_none() && key.metal_rough.is_some();
                let opts = ObjOptions {
                    scale: 1.0,
                    texture: match (key.base.as_deref(), key.solid) {
                        (Some(t), _) => match key.alpha {
                            Some(a) => plan.faded(&index, t, a, key.tint),
                            // Lettering, placard and legend decals: their
                            // transparent texels' colour is dilated in from
                            // the opaque lettering next to it before mips
                            // are built, so a dark fringe cannot bleed in
                            // (see `Job::Decal`). `key.blend_alpha`: MSFS's
                            // own `baseColorBlendFactor` on this decal, baked
                            // into its alpha the same pass (see
                            // `TextureKey::blend_alpha`).
                            None if key.blend => plan.decal(&index, t, key.tint, key.blend_alpha),
                            // A masked cutout (glTF MASK): the same
                            // dilate-then-mip treatment as a decal, but at
                            // the cutoff this material itself declares
                            // (see `Job::Masked`), not the decal atlas's
                            // own dilation/coverage target.
                            None => match key.mask_cutoff {
                                Some(cutoff) => plan.masked(&index, t, cutoff, key.tint),
                                None => plan.albedo(&index, t, key.metal_rough.as_deref(), key.tint),
                            },
                        },
                        (None, Some(c)) => Some(solid_texture(&objects, c)?),
                        (None, None) => None,
                    },
                    // An alpha-tested object's night texture has to carry
                    // the albedo's alpha, or the test never discards
                    // anything (see `Job::LitMasked`).
                    texture_lit: key.lit.as_deref().and_then(|t| match key.base.as_deref() {
                        Some(base) if key.blend || key.mask_cutoff.is_some() => plan.lit_masked(&index, t, base),
                        _ => plan.plain(&index, t, key.lit_tint),
                    }),
                    texture_normal: key.normal.as_deref().and_then(|n| plan.normal(&index, n, &group_used)),
                    texture_material: (key.normal.is_some() || glass_reflect).then(|| plan.material(&index, key.metal_rough.as_deref(), &group_used)),
                    glass_specular: glass_reflect.then_some(0.5),
                    // The cabin's own LOD cutoff (`--cabin-lod-far`): unlike
                    // the tyres, which simply vanish, the point here is
                    // that this object is *not* marked internal-only (see
                    // `acf.rs`'s `internal`), so an exterior view now draws
                    // it too -- past this distance it stops again, so an
                    // open passenger door does not show a hollow fuselage
                    // right up close but a nose-in gate view still does not
                    // pay for 180k triangles it cannot tell from nothing.
                    lod_far: is_tire
                        .then_some(TIRE_LOD_FAR_M)
                        .or(is_cabin_group.then_some(args.cabin_lod_far))
                        .or((!is_ext).then_some(COCKPIT_LOD_FAR_M)),
                    ..Default::default()
                };
                let referenced: Vec<String> = [&opts.texture, &opts.texture_lit, &opts.texture_normal, &opts.texture_material]
                    .into_iter()
                    .flatten()
                    .cloned()
                    .collect();
                if is_ext {
                    ext_textures.extend(referenced.iter().cloned());
                } else {
                    int_textures.extend(referenced.iter().cloned());
                    if is_cabin_group {
                        cabin_textures.extend(referenced.iter().cloned());
                    } else {
                        cockpit_int_textures.extend(referenced.iter().cloned());
                    }
                }
                let file = format!("{model_name}_{i:03}.obj");
                // Cockpit and cabin objects take X-Plane's interior lighting,
                // as Laminar's do; lit as exterior they sit in the fuselage's
                // shadow and come out black. Glass blends as glass
                // (BLEND_GLASS), as Laminar's and FlightFactor's glass does.
                let lighting = match (is_ext, key.glass) {
                    (true, false) => "",
                    (true, true) => "BLEND_GLASS\n",
                    (false, false) => "GLOBAL_cockpit_lit\n",
                    (false, true) => "GLOBAL_cockpit_lit\nBLEND_GLASS\n",
                };
                // A lettering/placard object is alpha-tested as a whole, in
                // its header: inside the object the same test is a state
                // change X-Plane applies or not by the pass the .acf entry
                // puts the object in, and an entry that read as ordinary
                // opaque geometry left the colour dilated behind the
                // lettering drawn as a white box around every letter (see
                // `model::DECAL_GLOBAL_ALPHA_TEST`).
                //
                // Masked (glTF MASK) cutouts get the same whole-object
                // declaration, at their own cutoff rather than the decal
                // atlas's fixed 0.90 (see `TextureKey::mask_cutoff`): they
                // are not routed into the decal texture's dilation or the
                // translucent pass, only reinforced with the same
                // load-time-read alpha test their meshes already carry.
                // A lettering/placard object blends, as MSFS draws it
                // (`alphaMode: BLEND`). It used to be alpha-tested instead,
                // because a blended decal drew as a solid rectangle -- but
                // that was its night texture's alpha, fully opaque as MSFS
                // leaves it, defeating the test rather than the blend (see
                // `Job::LitMasked`). With that fixed, blending gives the
                // anti-aliased strokes a hard cutoff cannot.
                let header = if key.blend {
                    lighting.to_string()
                } else if let Some(cutoff) = key.mask_cutoff {
                    format!("{lighting}GLOBAL_no_blend {:.2}\n", cutoff as f32 / 255.0)
                } else {
                    lighting.to_string()
                };
                // Eligible for the exterior LOD chain: exterior, not glass
                // or a lettering/placard decal (both kept at L0 -- see
                // `key.glass`/`key.blend`'s own docs), not the tyre group
                // (already has its own `lod_far` cutoff above), and no mesh
                // in it carries a manipulator (none of the exterior's own
                // meshes do today -- `screen_of` always answers `None` here
                // -- but a future manipulator on an exterior part must not
                // be simplified out from under it).
                let lod_excluded = key.glass || key.blend || is_tire || meshes.iter().any(|&mi| visible(mi).manip.is_some());
                let use_lods = is_ext && !args.no_exterior_lods && !lod_excluded;
                let text = if use_lods {
                    // Owned simplified geometry for L1/L2, one entry per
                    // mesh, kept alive across the `write_obj8_animated_lods`
                    // call below (which only borrows it).
                    let mut l1_store: Vec<(usize, Vec<model::glb::Vertex>, Vec<u32>)> = Vec::new();
                    let mut l2_store: Vec<(usize, Vec<model::glb::Vertex>, Vec<u32>)> = Vec::new();
                    for &mi in meshes {
                        let mesh = &model.meshes[mi];
                        let (v1, i1, r1) = simplify_mesh(&mesh.vertices, &mesh.indices, args.ext_lod1_ratio);
                        let (mut v2, mut i2, mut r2) = simplify_mesh(&mesh.vertices, &mesh.indices, args.ext_lod2_ratio);
                        let group_mesh_tris = mesh.indices.len() / 3;
                        // L2 only: past 1500 m the aircraft is a few dozen
                        // pixels, so a border-crossing seam crack the
                        // border-locked pass above was protecting against
                        // is not something a viewer could tell apart from
                        // one that ran cleanly. Falling back to
                        // `simplify_mesh_sloppy` (no border lock at all) for
                        // just the meshes the locked pass left far short of
                        // its target -- typically hard-surface greeble with
                        // no shared vertices between adjacent faces, so
                        // every vertex already reads as a border and the
                        // locked pass has nothing left to collapse -- gets
                        // most of the rest of the way there without paying
                        // for it on every mesh, including the ones the
                        // locked pass already handles well. L1 stays
                        // border-locked unconditionally: at 400-1500 m a
                        // crack is still close enough to matter.
                        let l2_sloppy = r2 > args.ext_lod2_ratio * 1.5;
                        if l2_sloppy {
                            let (sv2, si2, sr2) = simplify_mesh_sloppy(&mesh.vertices, &mesh.indices, args.ext_lod2_ratio);
                            if sr2 < r2 {
                                (v2, i2, r2) = (sv2, si2, sr2);
                            }
                            ext_lod_l2_sloppy += 1;
                        }
                        // Only worth flagging past a handful of triangles:
                        // a bolt-sized mesh legitimately cannot reach a
                        // fine ratio (there is nothing left to collapse
                        // once it is down to a few triangles), and that is
                        // not a meshoptimizer failure worth a report line.
                        if group_mesh_tris > 200 && r1 > (args.ext_lod1_ratio * 1.5).max(args.ext_lod1_ratio + 0.15) {
                            ext_lod_failures.push(format!(
                                "{model_name}_{i:03}.obj mesh {mi} ({group_mesh_tris} tris): L1 kept {:.0}% (wanted {:.0}%)",
                                r1 * 100.0,
                                args.ext_lod1_ratio * 100.0
                            ));
                        }
                        if group_mesh_tris > 200 && r2 > (args.ext_lod2_ratio * 1.5).max(args.ext_lod2_ratio + 0.15) {
                            ext_lod_failures.push(format!(
                                "{model_name}_{i:03}.obj mesh {mi} ({group_mesh_tris} tris): L2 kept {:.0}% (wanted {:.0}%){}",
                                r2 * 100.0,
                                args.ext_lod2_ratio * 100.0,
                                if l2_sloppy { ", even with the sloppy fallback" } else { "" }
                            ));
                        }
                        ext_lod_tris[0] += group_mesh_tris;
                        ext_lod_tris[1] += i1.len() / 3;
                        ext_lod_tris[2] += i2.len() / 3;
                        l1_store.push((mi, v1, i1));
                        l2_store.push((mi, v2, i2));
                    }
                    let l0: Vec<LodMesh> = meshes.iter().map(|&mi| LodMesh { mesh: mi, vertices: &model.meshes[mi].vertices, indices: &model.meshes[mi].indices }).collect();
                    let l1: Vec<LodMesh> = l1_store.iter().map(|(mi, v, ix)| LodMesh { mesh: *mi, vertices: v, indices: ix }).collect();
                    let l2: Vec<LodMesh> = l2_store.iter().map(|(mi, v, ix)| LodMesh { mesh: *mi, vertices: v, indices: ix }).collect();
                    let levels = [
                        AnimLodLevel { near: 0.0, far: args.ext_lod1_far, meshes: l0 },
                        AnimLodLevel { near: args.ext_lod1_far, far: args.ext_lod2_far, meshes: l1 },
                        AnimLodLevel { near: args.ext_lod2_far, far: args.ext_lod3_far, meshes: l2 },
                    ];
                    ext_lod_objects += 1;
                    write_obj8_animated_lods(&model, &levels, &opts, &visible, "", &header)
                } else {
                    if is_ext {
                        ext_lod_excluded_objects += 1;
                        ext_lod_tris[0] += group_tris;
                    }
                    write_obj8_animated(&model, meshes, &opts, &visible, "", &header)
                };
                // X-Plane only tests one object per aircraft for classic
                // panel clicks (the template cockpit object's flags); a
                // screen's own `ATTR_manip_device` is excluded from this
                // check on purpose (see `has_classic_manipulator`), so a
                // touch screen never competes with the click object for
                // that one slot.
                let clickable = has_classic_manipulator(&text);
                std::fs::write(objects.join(&file), text)?;
                // Its extent in the OBJ's own frame (x right, y up, z aft,
                // metres from the reference point it is attached at), for
                // the .acf to tell whether it can shade the flight deck.
                let scale = if opts.scale.is_finite() && opts.scale > 0.0 { opts.scale } else { 1.0 };
                let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
                for &mi in meshes.iter() {
                    for v in &model.meshes[mi].vertices {
                        let p = [-(v.pos[0] * scale) as f64, (v.pos[1] * scale + opts.offset_y) as f64, -(v.pos[2] * scale) as f64];
                        for k in 0..3 {
                            lo[k] = lo[k].min(p[k]);
                            hi[k] = hi[k].max(p[k]);
                        }
                    }
                }
                if lo[0] <= hi[0] {
                    object_bounds.insert(file.clone(), (lo, hi));
                }
                tris += group_tris;
                object_files.push((
                    file,
                    match (is_ext, key.glass, key.blend) {
                        (true, true, _) => ObjKind::ExteriorGlass,
                        (true, false, true) => ObjKind::ExteriorBlend,
                        (true, false, false) => ObjKind::Exterior,
                        (false, true, _) => ObjKind::CabinGlass,
                        (false, false, true) => ObjKind::CabinBlend,
                        (false, false, false) if clickable => ObjKind::Cockpit,
                        // The passenger cabin proper (not glass or a decal,
                        // which stay `CabinGlass`/`CabinBlend`, marked
                        // internal-only as before -- see `ObjKind::PaxCabin`'s
                        // own doc for why only the plain case changes here).
                        (false, false, false) if is_cabin_group => ObjKind::PaxCabin,
                        (false, false, false) => ObjKind::Cabin,
                    },
                ));
            }
            let (lo, hi) = model.bounds().unwrap_or(([0.0; 3], [0.0; 3]));
            println!(
                "  {model_name}: {} objects, {tris} triangles, {:.1} m wide, {:.1} m tall, {:.1} m long",
                groups.len(),
                hi[0] - lo[0],
                hi[1] - lo[1],
                hi[2] - lo[2]
            );
            if is_ext && !args.no_exterior_lods {
                let pct = |n: usize| if ext_lod_tris[0] > 0 { n as f64 * 100.0 / ext_lod_tris[0] as f64 } else { 0.0 };
                println!(
                    "  {model_name}: exterior LOD: {ext_lod_objects} objects simplified ({ext_lod_l2_sloppy} meshes fell back to a sloppy L2 pass), {ext_lod_excluded_objects} left at L0 only (glass/decal/tyre/manipulator); triangles L0 {} (0-{:.0} m), L1 {} ({:.0}%, {:.0}-{:.0} m), L2 {} ({:.0}%, {:.0}-{:.0} m)",
                    ext_lod_tris[0],
                    args.ext_lod1_far,
                    ext_lod_tris[1],
                    pct(ext_lod_tris[1]),
                    args.ext_lod1_far,
                    args.ext_lod2_far,
                    ext_lod_tris[2],
                    pct(ext_lod_tris[2]),
                    args.ext_lod2_far,
                    args.ext_lod3_far
                );
                for f in &ext_lod_failures {
                    println!("  {model_name}: exterior LOD: {f}");
                    notes.push(format!("{model_name}: exterior LOD: {f}"));
                }
            }

            // Cockpit click spots: invisible copies of every part a click
            // should move, riding the same animations.
            if !is_ext {
                // Every drawn part, then MSFS's own invisible click spots
                // (oxygen masks, tables, doors are clicked through those).
                let sources: Vec<&model::glb::Mesh> = model.meshes.iter().chain(model.click_spots.iter()).collect();
                let mut spots: HashMap<usize, String> = (0..sources.len())
                    .filter_map(|mi| manip_for(&animator, &rig, sources[mi].node).map(|m| (mi, m)))
                    .collect();
                // The circuit breakers' own manipulators, which reach this
                // object rather than staying on the panel they are drawn on.
                //
                // X-Plane tests exactly one interior object for clicks, and
                // it is this one. A breaker manipulator written into the
                // drawn panel instead made that panel claim the same flag,
                // and X-Plane answers two claims by *skipping* the loser
                // outright -- geometry and all. Fifty-two breakers were
                // being drawn on an object X-Plane never drew, in exchange
                // for clicks it never tested.
                //
                // Nothing is lost by moving them: unlike a screen, whose
                // touches only reach its device through a manipulator on the
                // screen's own shape and UVs (`obj8.rs`), a breaker only
                // needs a shape in the right place, which is what a click
                // spot is.
                for (&mi, (_, manip)) in &cb {
                    spots.entry(mi).or_insert_with(|| manip.clone());
                }
                // Touch screens (EFB, OIT, MFD): X-Plane click-tests only
                // this one interior object (acf `_obj_flags` 0x805), so a
                // screen's `ATTR_manip_device` has to exist here too, not
                // only on the drawn cockpit object -- confirmed against the
                // installed, hand-fixed aircraft, which carries all six
                // targets (SCREEN_EFB x2, SCREEN_DU_MFD x2, SCREEN_OIT_LEFT,
                // SCREEN_OIT_RIGHT) in both places. The drawn object keeps
                // its own copy exactly as it was (`visible`, above); this is
                // additive. Unlike an ordinary click spot's padded box, a
                // screen's copy has to be "the same shape, and UV-mapped
                // the same way as the screen of the cockpit device" (the
                // OBJ8 spec's own pairing requirement), so it clones the
                // drawn mesh verbatim rather than going through
                // `click_shape`, and rides the same animation hierarchy the
                // drawn mesh has (the EFB's own hide/slide clip) so a
                // retracted screen cannot be tapped where it is not drawn.
                let screen_taps: Vec<(usize, String, String)> = model
                    .meshes
                    .iter()
                    .enumerate()
                    .filter_map(|(mi, m)| {
                        if m.indices.is_empty() {
                            return None;
                        }
                        let screen = model.materials.get(m.material).and_then(|mat| screens::screen_of(&screen_list, &mat.name))?;
                        screens::TOUCH_SCREENS.contains(&screen.id.as_str()).then(|| {
                            (
                                mi,
                                format!("ATTR_manip_device hand {} {}", screen.id, screen.id),
                                format!("{} {}", screen.id, screens::DEVICE_ARGS),
                            )
                        })
                    })
                    .collect();
                if !spots.is_empty() || !screen_taps.is_empty() {
                    let mut ids: Vec<usize> = spots.keys().copied().collect();
                    ids.sort_unstable();
                    // Click shapes: drawn fully transparent (draw-disabled
                    // geometry is not clickable, and untextured geometry shows
                    // as grey boxes). Each small part gets a padded box rather
                    // than a copy of its mesh: a fraction of the triangles, and
                    // in front of the part, so it wins the click. Big parts
                    // (seats, tables, shades) keep their mesh.
                    let mut click_model = Model {
                        materials: vec![model::glb::Material {
                            alpha: model::glb::AlphaMode::Blend,
                            double_sided: true,
                            ..Default::default()
                        }],
                        nodes: model.nodes.clone(),
                        ..Default::default()
                    };
                    let mut click_manips = Vec::new();
                    // `None` for an ordinary click spot; `Some(device args)`
                    // for a screen tap, which needs its own
                    // `ATTR_cockpit_device` (the OBJ8 spec: "the manipulator
                    // must also be tagged with ATTR_cockpit_device").
                    let mut click_devices: Vec<Option<String>> = Vec::new();
                    for &mi in &ids {
                        let at = sources[mi].node.map(|n| {
                            let w = model.nodes[n].world;
                            [w[12] as f32, w[13] as f32, w[14] as f32]
                        });
                        let mut shape = click_shape(sources[mi], at);
                        shape.material = 0;
                        click_model.meshes.push(shape);
                        click_manips.push(spots[&mi].clone());
                        click_devices.push(None);
                    }
                    for &(mi, ref manip, ref dev) in &screen_taps {
                        let mut shape = model.meshes[mi].clone();
                        shape.material = 0;
                        click_model.meshes.push(shape);
                        click_manips.push(manip.clone());
                        click_devices.push(Some(dev.clone()));
                    }
                    let clear = solid_texture(&objects, [0, 0, 0, 0])?;
                    let cids: Vec<usize> = (0..click_model.meshes.len()).collect();
                    let click = |ci: usize| MeshAnim {
                        commands: animator.commands(click_model.meshes[ci].node),
                        manip: Some(click_manips[ci].clone()),
                        hidden: false,
                        click: click_devices[ci].is_none(),
                        light: None,
                        device: click_devices[ci].clone(),
                    };
                    let file = format!("{model_name}_click.obj");
                    let opts = ObjOptions { scale: 1.0, texture: Some(clear), draw_disable: true, ..Default::default() };
                    std::fs::write(
                        objects.join(&file),
                        write_obj8_animated(&click_model, &cids, &opts, &click, "", "GLOBAL_cockpit_lit\n"),
                    )?;
                    if !screen_taps.is_empty() {
                        let line = format!("{} screen taps copied into the click object (EFB, OIT, MFD)", screen_taps.len());
                        println!("  {model_name}: {line}");
                        notes.push(format!("{model_name}: {line}"));
                    }
                    // A part can carry one manipulator only, so where several
                    // controls animate the same part (a knob that also
                    // pushes), the highest-priority one takes it and the rest
                    // keep their dataref but cannot be clicked.
                    let controls: HashSet<&String> = spots.values().collect();
                    let sharing = rig.manips.values().filter(|(_, m)| !controls.contains(m)).count();
                    if let Some(res) = &resolution {
                        // Before: clips' own manipulators only, on the parts
                        // those clips move.
                        let old_rig = rig::cockpit(&model, &clips, None);
                        let old_anim = Animator::new(&model, &clips, old_rig.drefs.clone(), old_rig.vis.clone());
                        // (and drawn parts only: the loader used to drop the
                        // invisible click spots).
                        let old_nodes: Vec<Option<usize>> = model.meshes.iter().filter(|m| manip_for(&old_anim, &old_rig, m.node).is_some()).map(|m| m.node).collect();
                        let new_nodes: Vec<Option<usize>> = spots.keys().map(|&mi| sources[mi].node).collect();
                        let (present, before, _) = click_coverage(&model, &old_anim, res, &old_nodes);
                        let (_, after, missing) = click_coverage(&model, &animator, res, &new_nodes);
                        for m in missing {
                            bindings_report.push(format!("{m}: no click spot (no geometry under its node or clip)"));
                        }
                        let bound_spots = spots.values().filter(|m| m.contains(" fbw/cockpit/") && (m.contains("_click ") || m.contains("_up "))
                            || !m.contains("fbw/cockpit/")).count();
                        let line = format!(
                            "click spots: {present} XML controls in the model; {before} had a click spot from their clips alone, {after} have one now; {bound_spots} of {} spots act on the systems",
                            ids.len()
                        );
                        println!("  {model_name}: {line}");
                        notes.push(format!("{model_name}: {line}"));
                    }
                    println!(
                        "  {model_name}: {} clickable controls on {} parts ({sharing} share a part with another control)",
                        controls.len(),
                        ids.len()
                    );
                    object_files.push((file, ObjKind::Cockpit));
                }
            } else {
                // Exterior click spots: controls whose node lives on this
                // model's own geometry (the 16 passenger-door handles are
                // the motivating case: `rig::exterior` now finds them a
                // manipulator via `Resolution`, but until this object
                // exists nothing in `objects/` carries it). No circuit
                // breakers and no click-coverage report here -- neither
                // concept applies to the exterior model.
                let sources: Vec<&model::glb::Mesh> = model.meshes.iter().chain(model.click_spots.iter()).collect();
                let spots: HashMap<usize, String> = (0..sources.len())
                    .filter_map(|mi| manip_for(&animator, &rig, sources[mi].node).map(|m| (mi, m)))
                    .collect();
                if !spots.is_empty() {
                    let mut ids: Vec<usize> = spots.keys().copied().collect();
                    ids.sort_unstable();
                    let mut click_model = Model {
                        materials: vec![model::glb::Material {
                            alpha: model::glb::AlphaMode::Blend,
                            double_sided: true,
                            ..Default::default()
                        }],
                        nodes: model.nodes.clone(),
                        ..Default::default()
                    };
                    let mut click_manips = Vec::new();
                    for &mi in &ids {
                        let at = sources[mi].node.map(|n| {
                            let w = model.nodes[n].world;
                            [w[12] as f32, w[13] as f32, w[14] as f32]
                        });
                        let mut shape = click_shape(sources[mi], at);
                        shape.material = 0;
                        click_model.meshes.push(shape);
                        click_manips.push(spots[&mi].clone());
                    }
                    let clear = solid_texture(&objects, [0, 0, 0, 0])?;
                    let cids: Vec<usize> = (0..click_model.meshes.len()).collect();
                    let click = |ci: usize| MeshAnim {
                        commands: animator.commands(click_model.meshes[ci].node),
                        manip: Some(click_manips[ci].clone()),
                        hidden: false,
                        click: true,
                        light: None,
                        device: None,
                    };
                    let file = format!("{model_name}_click.obj");
                    let opts = ObjOptions { scale: 1.0, texture: Some(clear), draw_disable: true, ..Default::default() };
                    std::fs::write(objects.join(&file), write_obj8_animated(&click_model, &cids, &opts, &click, "", ""))?;
                    let line = format!("click spots: {} behaviour-XML controls found a manipulator on this model's own geometry", ids.len());
                    println!("  {model_name}: {line}");
                    notes.push(format!("{model_name}: {line}"));
                    object_files.push((file, ObjKind::Exterior));
                }
            }

            // Exterior lights at the systems.cfg light nodes.
            if is_ext && !systems_cfg.is_empty() {
                let (text, n) = lights::lights_obj(&model, &animator, &systems_cfg);
                if n > 0 {
                    std::fs::write(objects.join("lights.obj"), text)?;
                    object_files.push(("lights.obj".into(), ObjKind::Lights));
                    println!("  lights: {n} at the systems.cfg light nodes");
                    notes.push(format!("lights: {n} exterior lights at the MSFS light nodes"));
                }
            }
        }
        rigs.push(rig);
        if is_ext {
            exterior = Some(model);
        }
    }

    // SASL and its module: the datarefs the animations and clicks use.
    if !bindings_report.is_empty() {
        bindings_report.sort();
        let mut text = String::from("Cockpit controls, from FlyByWire's behaviour XML: what each click does in X-Plane.\r\n\r\n");
        for l in &bindings_report {
            text.push_str(l);
            text.push_str("\r\n");
        }
        std::fs::write(root.join("cockpit_bindings.txt"), text)?;
        // The variables the cockpit binds, for the systems plugin to publish
        // at start (a dataref the scripts look for before it exists only
        // logs a warning and the control does nothing).
        let names = behaviour::bind::variables();
        let dir = root.join("plugins").join("fbw_a380_systems");
        std::fs::create_dir_all(&dir)?;
        // name, tab, start value where the aircraft's start state gives one.
        let lines: Vec<String> = names
            .iter()
            .map(|n| match sim.defaults.get(&behaviour::bind::dataref(n)) {
                Some(v) => format!("{n}\t{v}"),
                None => n.clone(),
            })
            .collect();
        std::fs::write(dir.join("cockpit_variables.txt"), lines.join("\n"))?;
        notes.push(format!("plugins/fbw_a380_systems/cockpit_variables.txt: {} variables the cockpit binds", names.len()));
        notes.push("cockpit_bindings.txt lists every XML control: its binding, or why it keeps its own dataref".into());
        // Click/switch sounds: <dataref> <Direction> <NormalizedTime> <WwiseEvent>,
        // one line per <EventTrigger> MSFS's own model behaviour templates
        // pair with this dataref's animation. Only present when
        // --asobo-behaviours resolved sound triggers (see behaviour::resolve).
        if let Some(triggers) = resolution.as_ref().map(|r| &r.sound_triggers).filter(|t| !t.is_empty()) {
            // 5 columns as of W221: NormalizedTime and Count are mutually
            // exclusive per line (whichever is absent is written as an
            // empty field, not a sentinel) -- see bind::SoundTrigger's doc.
            // A pre-W221 sound_triggers.txt has only 4 columns; the
            // plugin's parser treats that as "no triggers" rather than
            // erroring, so both sides of this pipeline must be rebuilt
            // together for click sounds to keep working.
            let lines: Vec<String> = triggers
                .iter()
                .map(|t| {
                    format!(
                        "{}\t{}\t{}\t{}\t{}",
                        t.dataref,
                        t.direction,
                        t.normalized_time.map(|n| n.to_string()).unwrap_or_default(),
                        t.wwise_event,
                        t.count.map(|c| c.to_string()).unwrap_or_default(),
                    )
                })
                .collect();
            std::fs::write(dir.join("sound_triggers.txt"), lines.join("\n"))?;
            notes.push(format!("plugins/fbw_a380_systems/sound_triggers.txt: {} click/switch sound triggers", triggers.len()));
        }
    }
    if !rigs.is_empty() {
        let lua = rig::main_lua(&title, &rigs.iter().collect::<Vec<_>>(), Some(&sim));
        let line = sasl::install(&root, args.sasl.as_deref(), &lua, &name)?;
        println!("  {line}");
        notes.push(line);
    }

    // The screens' fonts and images, which the systems plugin's own
    // tessellating renderer loads from <aircraft>/html_ui (FlyByWire's
    // GPL-3.0 package) to draw its op-stream screens (PFD, ND, ...), leaving
    // out only the EFB's fonts: XPHFBW draws the EFB from its own browser
    // bitmap instead (display/screens.rs's SCREEN_EFB, app/src/views.rs's
    // spawn_efb_view), never through this text/image renderer. The OITs'
    // images (`Images/fbw-a380x/oit/`) are copied: the OIT pages are drawn
    // now (display/screens.rs's SCREEN_OIT_LEFT/RIGHT, fbw-xp-systems
    // docs/oit.md).
    if !screen_list.is_empty() {
        let mut copied = 0;
        for (sub, skip) in [("Fonts", Some("EFB")), ("Images", None)] {
            let from = args.package.join("html_ui").join(sub);
            for entry in walkdir::WalkDir::new(&from).into_iter().filter_map(Result::ok).filter(|e| e.file_type().is_file()).map(|e| e.into_path()) {
                let rel = entry.strip_prefix(&from).unwrap_or(&entry);
                if skip.is_some_and(|skip| rel.components().any(|c| c.as_os_str().to_string_lossy().eq_ignore_ascii_case(skip))) {
                    continue;
                }
                let to = root.join("html_ui").join(sub).join(rel);
                if let Some(dir) = to.parent() {
                    std::fs::create_dir_all(dir)?;
                }
                std::fs::copy(&entry, &to)?;
                copied += 1;
            }
        }
        let line = format!("screens: {} devices; {copied} font and image files in html_ui", screen_list.len());
        println!("  {line}");
        notes.push(line);
        // html_ui/Pages and html_ui/JS -- FlyByWire's own built gauge
        // instruments (PFD, ND, EWD, SD, MFD, ...) -- are a SEPARATE build
        // this converter never touches (docs/js-build.md): it has no path to
        // FlyByWire's build output, and the MSFS package's own html_ui/Pages
        // is not a safe substitute (it lags FlyByWire's real build by days
        // and is not what SourcePatch/js_bridge match text against). They
        // still have to be merged in by tools/install.sh (D:/A380/fbw-xp-systems)
        // or P10-build-install.ps1 before X-Plane can show a single screen.
        // A 2026-09-25 install that skipped that merge left every cockpit
        // screen black with nothing anywhere saying why -- say so here too,
        // since this converter is sometimes run and inspected on its own.
        let reminder = format!(
            "REMINDER: {} still needs FlyByWire's built html_ui/Pages + html_ui/JS merged in \
             (tools/install.sh or P10-build-install.ps1) -- this converter only wrote Fonts/Images \
             above. Without that merge every cockpit screen is BLACK.",
            root.join("html_ui").display()
        );
        println!("  {reminder}");
        notes.push(reminder);
    }

    // Every other texture of the package, so the set is complete: normal maps
    // with their COMP, the rest as they are. Off by default: on the A380
    // this wrote 76 files, 0.62 GB, that no emitted object ever references,
    // and a converted aircraft is not a copy of the package.
    let rest: Vec<String> = if args.all_textures {
        index.keys().filter(|k| !plan.used.contains(*k)).cloned().collect()
    } else {
        Vec::new()
    };
    for file in rest.iter().filter(|f| is_normal(f)) {
        let comp = comp_for(&index, file);
        // No mesh/UV grouping reaches this catch-all pass, so there is no
        // coverage to give it: it keeps the old, undilated behaviour.
        plan.normal(&index, file, &[]);
        plan.material(&index, comp.as_deref(), &[]);
    }
    for file in &rest {
        if !plan.used.contains(file) {
            plan.plain(&index, file, None);
        }
    }

    // Liveries: the package's built-in variants, then livery packages.
    let mut sources: Vec<(String, Vec<PathBuf>)> = variants.iter().map(|(t, d)| (safe(t), texture_dirs(d))).collect();
    for pkg in &args.livery {
        let fallback = pkg.file_name().unwrap_or_default().to_string_lossy().to_string();
        sources.push((safe(&manifest_title(pkg).unwrap_or(fallback)), texture_dirs(&pkg.join("SimObjects"))));
    }
    let mut liveries = Vec::new();
    if !args.no_textures {
        let jobs: Vec<(String, Job)> = plan.jobs.clone().into_iter().collect();
        let results: Vec<Result<usize, String>> = jobs
            .par_iter()
            .map(|(n, j)| {
                write_texture(j, &objects.join(n), &args, ext_textures.contains(n) && !int_textures.contains(n), cabin_textures.contains(n) && !cockpit_int_textures.contains(n))
                    .map_err(|e| format!("{n}: {e:#}"))
            })
            .collect();
        let bytes: usize = results.iter().filter_map(|r| r.as_ref().ok()).sum();
        let failed: Vec<&String> = results.iter().filter_map(|r| r.as_ref().err()).collect();
        println!("  textures: {} written ({} MB), {} failed", jobs.len() - failed.len(), bytes / 1_000_000, failed.len());
        for f in failed.iter().take(10) {
            eprintln!("    {f}");
        }
        if !plan.missing.is_empty() {
            println!("  {} textures the models name are not in the package:", plan.missing.len());
            for m in plan.missing.iter().take(10) {
                println!("    {m}");
            }
        }

        // Liveries: the same output names, from each livery's textures.
        for (lname, dirs) in &sources {
            let lname = lname.clone();
            let lindex = texture_index(dirs);
            let dir = root.join("liveries").join(&lname).join("objects");
            std::fs::create_dir_all(&dir)?;
            let lookup = |p: &PathBuf| lindex.get(&p.file_name().unwrap_or_default().to_string_lossy().to_ascii_lowercase()).cloned();
            let ljobs: Vec<(String, Job)> = plan
                .jobs
                .iter()
                .filter_map(|(out_name, job)| {
                    let j = match job {
                        Job::Plain { src, tint } => lookup(src).map(|s| Job::Plain { src: s, tint: *tint }),
                        // A livery without the base package's COMP texture
                        // still gets its own albedo, just unbaked. Either way
                        // it keeps the material's own tint.
                        Job::Albedo { src, comp, tint } => lookup(src).map(|s| match lookup(comp) {
                            Some(c) => Job::Albedo { src: s, comp: c, tint: *tint },
                            None => Job::Plain { src: s, tint: *tint },
                        }),
                        Job::Faded { src, alpha, tint } => lookup(src).map(|s| Job::Faded { src: s, alpha: *alpha, tint: *tint }),
                        Job::Normal { normal, used, floor } => lookup(normal).map(|n| Job::Normal { normal: n, used: used.clone(), floor: *floor }),
                        Job::Material { comp, used, floor } => comp.as_ref().and_then(lookup).map(|c| Job::Material { comp: Some(c), used: used.clone(), floor: *floor }),
                        Job::Decal { src, tint, blend } => lookup(src).map(|s| Job::Decal { src: s, tint: *tint, blend: *blend }),
                        Job::Masked { src, cutoff, tint } => lookup(src).map(|s| Job::Masked { src: s, cutoff: *cutoff, tint: *tint }),
                        Job::LitMasked { lit, albedo } => lookup(lit)
                            .and_then(|l| lookup(albedo).map(|a| Job::LitMasked { lit: l, albedo: a })),
                    }?;
                    Some((out_name.clone(), j))
                })
                .collect();
            let ok = ljobs
                .par_iter()
                .filter(|(n, j)| match write_texture(
                    j,
                    &dir.join(n),
                    &args,
                    ext_textures.contains(n) && !int_textures.contains(n),
                    cabin_textures.contains(n) && !cockpit_int_textures.contains(n),
                ) {
                    Ok(_) => true,
                    Err(e) => {
                        eprintln!("    livery {lname}: {n}: {e:#}");
                        false
                    }
                })
                .count();
            println!("  livery {lname}: {ok} of {} textures ({} files in the livery)", ljobs.len(), lindex.len());
            liveries.push(lname);
        }
    }

    // Pictures for X-Plane's aircraft and livery pickers, from MSFS's.
    let mut icons = 0;
    let targets = std::iter::once((root.clone(), base_dirs.clone()))
        .chain(sources.iter().map(|(l, d)| (root.join("liveries").join(l), d.clone())));
    for (dir, dirs) in targets {
        if let Some(jpg) = find_thumbnail(&dirs) {
            match write_icons(&jpg, &dir, &name) {
                Ok(()) => icons += 1,
                Err(e) => eprintln!("  icon for {}: {e:#}", dir.display()),
            }
        }
    }
    println!("  icons: {icons} (aircraft and liveries)");

    // X-Plane allows exactly one interior cockpit object.
    //
    // The flag is bit 0x800 of _obj_flags, and it is what makes X-Plane
    // test an object for clicks. ObjKind::Cockpit carries it, and it was
    // being given to every interior object that held an ATTR_manip_ --
    // three of them here. X-Plane keeps the first and SKIPS THE REST
    // ENTIRELY:
    //
    //   E/ACF: This aircraft has two interior cockpit objects;
    //          0 will be used and 50 will be skipped.
    //
    // So two whole cockpit objects never drew at all -- geometry, panels
    // and displays with them. A missing click spot is a nuisance; a
    // missing object is half a cockpit.
    //
    // The one that keeps the flag is the dedicated click-spot object if
    // there is one, since holding click spots is the whole of its job;
    // otherwise the first, which is what X-Plane would have chosen. The
    // rest become ordinary interior objects: they draw, and what they
    // lose is the clicks, which is the lesser half of the trade and is
    // reported rather than left to be discovered.
    {
        let cockpits: Vec<usize> = object_files
            .iter()
            .enumerate()
            .filter(|(_, (_, k))| matches!(k, ObjKind::Cockpit))
            .map(|(i, _)| i)
            .collect();
        if cockpits.len() > 1 {
            let keep = cockpits
                .iter()
                .copied()
                .find(|&i| object_files[i].0.contains("_click"))
                .unwrap_or(cockpits[0]);
            let demoted: Vec<String> = cockpits
                .iter()
                .filter(|&&i| i != keep)
                .map(|&i| {
                    object_files[i].1 = ObjKind::Cabin;
                    object_files[i].0.clone()
                })
                .collect();
            let line = format!(
                "objects: {} is the interior cockpit object (X-Plane allows one); {} kept their geometry but lost their click spots: {}",
                object_files[keep].0,
                demoted.len(),
                demoted.join(", ")
            );
            println!("  {line}");
            notes.push(line);
        }
    }

    // The .acf, measured from the exterior model.
    let mut acf_report = Vec::new();
    if let Some(template) = &args.acf_template {
        match &exterior {
            None => eprintln!("  acf: no exterior model to measure; skipped"),
            Some(ext) => {
                let cfg_dir = args.cfg_dir.clone().unwrap_or_else(|| plane.clone());
                let (vmo, mmo) = vmo_mmo_or_default(args.vmo, args.mmo);
                let (text, report) = acf::build(&acf::Inputs {
                    template,
                    cfg_dir: &cfg_dir,
                    exterior: ext,
                    objects: &object_files,
                    object_bounds: &object_bounds,
                    name: &name,
                    livery: &cfg_title(&plane.join("aircraft.cfg")).unwrap_or_else(|| title.clone()),
                    vmo: Some(vmo),
                    mmo: Some(mmo),
                    lemac: args.lemac,
                    cg_z: args.cg_z,
                    mac: args.mac,
                    flap_degrees: args.flap_degrees.clone(),
                    slat_degrees: args.slat_degrees.clone(),
                    nose_steering: args.nose_steering,
                    body_steering: args.body_steering,
                })?;
                std::fs::write(root.join(format!("{name}.acf")), text)?;
                if let Some(src) = template.parent().map(|d| d.join("airfoils")).filter(|d| d.is_dir()) {
                    let dst = root.join("airfoils");
                    std::fs::create_dir_all(&dst)?;
                    for e in std::fs::read_dir(&src)?.filter_map(Result::ok).filter(|e| e.path().is_file()) {
                        std::fs::copy(e.path(), dst.join(e.file_name()))?;
                    }
                }
                for r in &report {
                    println!("  acf: {r}");
                }
                acf_report = report;
            }
        }
    } else {
        // An .acf lists its objects by number, and the numbering follows the
        // texture groups: change what the groups are and an older .acf's
        // per-object flags land on the wrong objects, which X-Plane reads as
        // "this decal object is ordinary opaque geometry" and "this glass
        // object is not glass" (it says the latter in Log.txt).
        println!("  acf: none written (no --acf-template): an .acf kept from an earlier run lists these objects by number, and its per-object flags no longer match them");
    }

    let mut readme = format!(
        "{title}, converted for X-Plane 12 by msfs2xp-aircraft {}.\r\n\r\n\
         Derived from FlyByWire Simulations' work, licensed GPL-3.0\r\n\
         (https://github.com/flybywiresim/aircraft). Anything built from these\r\n\
         files must stay GPL-3.0 and credit FlyByWire.\r\n\r\n\
         objects/          models as OBJ8 (one file per texture set) and textures:\r\n\
         \x20                 *.dds       colour and night textures, DXT1/DXT5\r\n\
         \x20                 *_NML.png   normal maps with metalness and gloss\r\n\
         \x20                 *_click.obj invisible click spots for every cockpit control\r\n\
         \x20                 lights.obj  exterior lights (X-Plane's light switches work them)\r\n\
         plugins/sasl/     SASL 3 running data/modules/main.lua, which creates the\r\n\
         \x20                 datarefs: fbw/anim/* (exterior animations, 0..1 over each\r\n\
         \x20                 MSFS clip) and fbw/cockpit/* (every cockpit control).\r\n\
         liveries/         livery textures under the same names; X-Plane swaps them in.\r\n\r\n\
         Animated: gear, gear doors, bogie tilt, steering, wheels, flaps, slats,\r\n\
         spoilers, ailerons, elevators, stabiliser trim, rudders, fans (with blur\r\n\
         discs by N1) and reversers follow X-Plane. Passenger and cargo doors, the\r\n\
         RAT and outflow valves have datarefs at rest, for the systems to drive.\r\n\
         Cockpit: pushbuttons press and spring back, knobs turn by click or wheel,\r\n\
         switches step, guards and breakers toggle, levers, seats, tables and\r\n\
         shades drag; controls FlyByWire's XML resolves act on the systems (see\r\n\
         cockpit_bindings.txt), and legends, backlights and annunciators glow as\r\n\
         their XML emissive codes say.\r\n\
         SASL is 1-sim's (https://1-sim.com), free for free projects; it may not be\r\n\
         redistributed, so it is copied from your own download.\r\n\
         Not converted: systems, displays, sounds.\r\n",
        env!("CARGO_PKG_VERSION")
    );
    for n in &notes {
        readme.push_str(&format!("\r\nNote: {n}"));
    }
    if !liveries.is_empty() {
        readme.push_str(&format!("\r\n\r\nLiveries: {}\r\n", liveries.join(", ")));
    }
    if !acf_report.is_empty() {
        readme.push_str(&format!(
            "\r\n\r\n{name}.acf, built on {} with:\r\n",
            args.acf_template.as_ref().map_or(String::new(), |t| t.display().to_string())
        ));
        for r in &acf_report {
            readme.push_str(&format!("  {r}\r\n"));
        }
    }
    readme.push_str("\r\n\r\nObjects:\r\n");
    for (f, _) in &object_files {
        readme.push_str(&format!("  objects/{f}\r\n"));
    }
    std::fs::write(root.join("README.txt"), readme)?;
    println!("wrote {}", root.display());
    Ok(())
}

#[cfg(test)]
mod vmo_mmo_tests {
    use super::vmo_mmo_or_default;

    #[test]
    fn defaults_to_flybywires_a380x_figures_when_both_are_omitted() {
        // fbw-a380x/src/systems/shared/src/PerformanceConstants.ts:1-2.
        assert_eq!(vmo_mmo_or_default(None, None), (340.0, 0.89));
    }

    #[test]
    fn an_explicit_flag_overrides_only_its_own_default() {
        assert_eq!(vmo_mmo_or_default(Some(300.0), None), (300.0, 0.89));
        assert_eq!(vmo_mmo_or_default(None, Some(0.82)), (340.0, 0.82));
        assert_eq!(vmo_mmo_or_default(Some(300.0), Some(0.82)), (300.0, 0.82));
    }
}
