//! The A380X's cockpit screens: FlyByWire's instruments draw a command
//! stream per screen (docs/display-stream.md) and this module shows them on
//! X-Plane cockpit devices (docs/screens.md).
//!
//! A new stream is read and tessellated once, when it arrives, on the
//! thread the instruments run on ([`Tess`]), and the mesh handed to the draw
//! side under a brief lock. X-Plane calls each device's draw callback every
//! frame, which uploads the vertices when the stream is new and replays the
//! cached batches into the device's texture, dimmed to its knobs. Mouse clicks,
//! drags and the wheel over a device come back as screen events for the
//! scripts, in the screen's CSS pixels.

pub mod image;
pub mod path;
pub mod plan;
pub mod screens;
pub mod stream;
pub mod tessellate;
pub mod text;
mod gl;
#[cfg(test)]
mod soft;
#[cfg(test)]
mod tests;
mod xphfbw;

use std::collections::{HashSet, VecDeque};
use std::ffi::{c_char, c_int, c_void, CString};
use std::path::{Path, PathBuf};
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex};
use std::time::Instant;

use crate::xp::{self, AvionicsId, Xplm};
use crate::xphfbw_bridge::{Input, InputKind, ScreenBlock, ScreenHeader, Session, MAX_DIRTY_RECTS};
use screens::{ScreenDef, SCREENS};
use stream::Op;
use tessellate::{Images, Mesh, Resources, Tessellator};
use text::{Atlas, Fonts};

/// The instruments' root under the aircraft folder, laid out as the MSFS
/// package's `html_ui`: image URLs (`/Images/fbw-a380x/...`) resolve from
/// it, and the fonts are in [`FONTS_DIR`] under it.
pub const HTML_UI_DIR: &str = "html_ui";
pub const FONTS_DIR: &str = "Fonts/fbw-a380x";

/// A mouse event for the scripts' `__screenEvent`.
#[derive(Clone, Debug, PartialEq)]
pub struct ScreenEvent {
    pub screen: &'static str,
    pub kind: &'static str,
    pub x: f64,
    pub y: f64,
    pub button: i32,
    pub delta: f64,
}

/// What the last tessellation of a screen cost and made.
#[derive(Clone, Copy, Debug, Default)]
pub struct Stats {
    pub tessellate_ms: f64,
    pub vertices: usize,
    pub batches: usize,
    pub draw_calls: usize,
}

struct Screen {
    def: &'static ScreenDef,
    /// Counts meshes made, so the GPU knows when its copy is old.
    stream: u64,
    /// Device pixels per CSS pixel the device is drawn at.
    scale: (f64, f64),
    mesh: Mesh,
    steps: Vec<plan::Step>,
    gpu: gl::ScreenGpu,
    stats: Stats,
    /// Each dimming region's brightness, 0..1.
    brightness: Vec<f32>,
    handle: AvionicsId,
    pressed: bool,
    hover: Option<(c_int, c_int)>,
}

/// One screen's link to XPHFBW: the `ScreenBlock` this module creates for it
/// (docs/briefs/xphfbw-js-bridge.md, "Screens"), its GPU texture, and rule
/// 6's upload bookkeeping. `None` when this screen's `ScreenBlock` could not
/// be made (index out of [`xphfbw_bridge::MAX_VIEWS`]-sized limits does not
/// apply here, but the shared-memory object itself could fail to open).
struct BridgeScreen {
    block: ScreenBlock,
    gpu: gl::BridgeGpu,
    /// The last header `frame` this screen was fully uploaded through.
    last_uploaded: u64,
    /// Set when an upload raced a new publish (rule 6); forces the next
    /// upload to be a full one instead of trusting the latest dirty rects.
    force_full: bool,
    /// The ND's terrain or weather radar picture drawn under Chromium's ND
    /// page, and the layer generation last uploaded (0: none yet).
    underlay: gl::BridgeGpu,
    underlay_generation: u64,
    /// When this screen's texture was last brought up to date.
    last_upload_at: Option<std::time::Instant>,
}

/// Screen pixels copied into X-Plane's textures in one X-Plane frame, at
/// most: each copy stalls X-Plane's GPU, and 15 screens refreshed in full
/// every frame (58 MB) took X-Plane down to 2-3 fps. Screens take turns.
const UPLOAD_BUDGET_BYTES: usize = 12 << 20;
/// A screen is refreshed at most this often (the instruments' own 30 Hz).
const UPLOAD_MIN_INTERVAL: std::time::Duration = std::time::Duration::from_millis(33);
/// A screen that has waited this long is refreshed even over the budget, so
/// a big one (the FCU's 2560x1280) never starves.
const UPLOAD_MAX_WAIT: std::time::Duration = std::time::Duration::from_millis(150);

/// XPHFBW's session for this plugin run (agent C/A hands this in once the
/// plugin's `Session` opens; see [`set_bridge`]).
struct Bridge {
    session: &'static Session,
    /// Indexed like [`SCREENS`] / [`Displays::screens`]; also the `screen`
    /// index in the `Input` records this module pushes ([`Session::input`]
    /// order matches `ScreenBlock` order, both `SCREENS` order).
    screens: Vec<Option<BridgeScreen>>,
}

/// What X-Plane's main thread draws from: the meshes tessellation made, a
/// copy of the glyph atlas and the images they use, and the devices. The
/// instruments' thread holds it only briefly, to hand over a new mesh.
pub struct Displays {
    xplm: Option<&'static Xplm>,
    api: Option<xp::AvionicsApi>,
    screens: Vec<Screen>,
    /// The atlas and images as the meshes need them; no fonts.
    res: Resources,
    renderer: Option<Result<gl::Renderer, String>>,
    fbo: Option<xp::DataRef>,
    events: VecDeque<ScreenEvent>,
    /// XPHFBW's screens, once attached; `None` keeps every screen on the
    /// QuickJS drawing path (rule 7: `displays_active` starts and stays 0
    /// with no session to read it from).
    bridge: Option<Bridge>,
    /// What is left of this X-Plane frame's [`UPLOAD_BUDGET_BYTES`].
    upload_budget: usize,
}

// X-Plane's handles are only touched on its main thread.
unsafe impl Send for Displays {}

/// Events kept for the scripts at most; older ones are dropped.
const EVENT_LIMIT: usize = 256;

static DISPLAYS: Mutex<Option<Displays>> = Mutex::new(None);

/// Tessellation: the fonts, the glyph atlas it fills and the images, with
/// each screen's last stream. It runs where the streams arrive (the
/// instruments' thread), so X-Plane's frame never waits on it.
static TESSELLATION: Mutex<Option<Tess>> = Mutex::new(None);

/// Device scales the draw callbacks found changed, for tessellation to make
/// those screens' meshes again.
static RESCALED: Mutex<Vec<(usize, (f64, f64))>> = Mutex::new(Vec::new());

fn log(message: &str) {
    crate::log(&format!("display: {message}"));
}

/// A new mesh for a screen.
struct Made {
    index: usize,
    mesh: Mesh,
    steps: Vec<plan::Step>,
    stats: Stats,
}

/// Changes to the glyph atlas since the draw side's copy.
enum AtlasUpdate {
    Whole { size: u32, generation: u64, pixels: Vec<u8> },
    Rows { first: u32, pixels: Vec<u8> },
}

struct TessScreen {
    ops: Vec<Op>,
    raw: Vec<f64>,
    strings: Vec<String>,
    scale: (f64, f64),
    have: bool,
}

/// Tessellation's state (see [`TESSELLATION`]).
pub struct Tess {
    tessellator: Tessellator,
    res: Resources,
    screens: Vec<TessScreen>,
    /// The atlas generation and size the draw side has, and how many images.
    sent_atlas: (u64, u32),
    sent_pictures: usize,
    logged: HashSet<String>,
}

impl Tess {
    pub fn new(html_ui: PathBuf) -> Self {
        let fonts_dir = html_ui.join(FONTS_DIR);
        let (fonts, problems) = Fonts::load(&fonts_dir);
        for p in problems {
            log(&p);
        }
        if fonts.is_empty() {
            log(&format!("no fonts in {}; text will not be drawn", fonts_dir.display()));
        }
        Self {
            tessellator: Tessellator::default(),
            res: Resources { fonts, atlas: Atlas::default(), images: Images::new(html_ui) },
            screens: SCREENS
                .iter()
                .map(|_| TessScreen { ops: Vec::new(), raw: Vec::new(), strings: Vec::new(), scale: (1., 1.), have: false })
                .collect(),
            sent_atlas: (0, 0),
            sent_pictures: 0,
            logged: HashSet::new(),
        }
    }

    fn log_once(&mut self, message: String) {
        if self.logged.len() < 1000 && self.logged.insert(message.clone()) {
            log(&message);
        }
    }

    /// A new stream for a screen, read and tessellated; nothing for a stream
    /// identical to the last.
    fn submit(&mut self, screen: &str, ops: &[f64], strings: Vec<String>) -> Result<Vec<Made>, String> {
        let index = screens::find(screen).ok_or_else(|| format!("there is no screen {screen}"))?;
        let s = &mut self.screens[index];
        if s.have && s.raw == ops && s.strings == strings {
            return Ok(Vec::new());
        }
        let parsed = stream::parse(ops, strings.len()).map_err(|e| format!("{screen}: {e}"))?;
        s.ops = parsed;
        s.raw.clear();
        s.raw.extend_from_slice(ops);
        s.strings = strings;
        s.have = true;
        Ok(self.make(index))
    }

    /// Tessellate a screen again at a new device scale.
    fn rescale(&mut self, index: usize, scale: (f64, f64)) -> Vec<Made> {
        let s = &mut self.screens[index];
        if s.scale == scale {
            return Vec::new();
        }
        s.scale = scale;
        if s.have {
            self.make(index)
        } else {
            Vec::new()
        }
    }

    /// The screen's mesh, and every other screen's too should the atlas
    /// have started again (their glyph coordinates are then stale).
    fn make(&mut self, index: usize) -> Vec<Made> {
        let generation = self.res.atlas.generation;
        let mut out: Vec<Made> = self.tessellate(index).into_iter().collect();
        if self.res.atlas.generation != generation {
            let others: Vec<usize> = (0..self.screens.len()).filter(|&i| i != index && self.screens[i].have).collect();
            for i in others {
                out.extend(self.tessellate(i));
            }
        }
        out
    }

    fn tessellate(&mut self, index: usize) -> Option<Made> {
        let started = Instant::now();
        let def = &SCREENS[index];
        let s = &self.screens[index];
        let size = ((def.width as f64 * s.scale.0).round() as u32, (def.height as f64 * s.scale.1).round() as u32);
        let mut mesh = None;
        // The atlas may fill and start again (a new generation) part way;
        // then everything is tessellated against the new one.
        for _ in 0..4 {
            if let Ok(m) = self.tessellator.run(&s.ops, &s.strings, def.id, size, s.scale, &mut self.res) {
                mesh = Some(m);
                break;
            }
        }
        let Some(mut mesh) = mesh else {
            self.log_once(format!("{}: the glyph atlas cannot hold this screen's text", def.id));
            return None;
        };
        let (sx, sy) = s.scale;
        let regions: Vec<[f32; 4]> = def
            .dimming
            .iter()
            .map(|d| {
                let [x, y, w, h] = d.region.map(|v| v as f64);
                [x * sx, y * sy, (x + w) * sx, (y + h) * sy].map(|v| v.round() as f32)
            })
            .collect();
        tessellate::add_dimming(&mut mesh, &regions);
        let steps = plan::plan(&mesh);
        let stats = Stats {
            tessellate_ms: started.elapsed().as_secs_f64() * 1000.,
            vertices: mesh.vertices.len(),
            batches: mesh.batches.len(),
            draw_calls: plan::draw_calls(&steps),
        };
        for p in std::mem::take(&mut mesh.problems) {
            self.log_once(format!("{}: {p}", def.id));
        }
        for family in std::mem::take(&mut self.res.fonts.missing) {
            self.log_once(format!("no font for the family {family}"));
        }
        Some(Made { index, mesh, steps, stats })
    }

    /// What the draw side's atlas copy lacks.
    fn atlas_update(&mut self) -> Option<AtlasUpdate> {
        let atlas = &mut self.res.atlas;
        if self.sent_atlas != (atlas.generation, atlas.size) {
            self.sent_atlas = (atlas.generation, atlas.size);
            atlas.dirty = None;
            return Some(AtlasUpdate::Whole { size: atlas.size, generation: atlas.generation, pixels: atlas.pixels.clone() });
        }
        let (first, last) = atlas.dirty.take()?;
        let row = atlas.size as usize;
        Some(AtlasUpdate::Rows { first, pixels: atlas.pixels[first as usize * row..last as usize * row].to_vec() })
    }

    pub fn measure_text(&mut self, screen: Option<&str>, family: &str, size: f64, text: &str) -> f64 {
        match self.res.fonts.select(screen.unwrap_or(""), family, 400, false) {
            Some(face) if size.is_finite() && size > 0. => self.res.fonts.layout(face, size, text).width,
            _ => 0.,
        }
    }

    pub fn font_metrics(&mut self, screen: Option<&str>, family: &str, size: f64) -> (f64, f64) {
        match self.res.fonts.select(screen.unwrap_or(""), family, 400, false) {
            Some(face) if size.is_finite() && size > 0. => self.res.fonts.metrics(face, size),
            _ => (0., 0.),
        }
    }
}

impl Displays {
    pub fn new(xplm: Option<&'static Xplm>, html_ui: PathBuf) -> Self {
        let screens = SCREENS
            .iter()
            .map(|def| Screen {
                def,
                stream: 0,
                scale: (1., 1.),
                mesh: Mesh { width: def.width, height: def.height, clips: vec![Default::default()], ..Default::default() },
                steps: Vec::new(),
                gpu: gl::ScreenGpu::default(),
                stats: Stats::default(),
                brightness: vec![0.; def.dimming.len()],
                handle: std::ptr::null_mut(),
                pressed: false,
                hover: None,
            })
            .collect();
        Self {
            xplm,
            api: xplm.and_then(|x| x.avionics()),
            screens,
            res: Resources { fonts: Fonts::default(), atlas: Atlas::default(), images: Images::new(html_ui) },
            renderer: None,
            fbo: xplm.and_then(|x| x.find("sim/graphics/view/current_gl_fbo")),
            events: VecDeque::new(),
            bridge: None,
            upload_budget: UPLOAD_BUDGET_BYTES,
        }
    }

    /// Take tessellation's new meshes, atlas changes and images.
    fn install(&mut self, made: Vec<Made>, atlas: Option<AtlasUpdate>, pictures: Vec<Arc<image::Picture>>) {
        match atlas {
            Some(AtlasUpdate::Whole { size, generation, pixels }) => {
                let a = &mut self.res.atlas;
                a.size = size;
                a.generation = generation;
                a.pixels = pixels;
                a.dirty = Some((0, size));
            }
            Some(AtlasUpdate::Rows { first, pixels }) => {
                let a = &mut self.res.atlas;
                let row = a.size as usize;
                let start = first as usize * row;
                if row > 0 && start + pixels.len() <= a.pixels.len() {
                    a.pixels[start..start + pixels.len()].copy_from_slice(&pixels);
                    let last = first + (pixels.len() / row) as u32;
                    a.dirty = Some(match a.dirty {
                        Some((f, l)) => (f.min(first), l.max(last)),
                        None => (first, last),
                    });
                }
            }
            None => {}
        }
        self.res.images.pictures.extend(pictures);
        for m in made {
            let s = &mut self.screens[m.index];
            s.mesh = m.mesh;
            s.steps = m.steps;
            s.stats = m.stats;
            s.stream += 1;
            if s.stream == 1 {
                let st = s.stats;
                log(&format!(
                    "{}: first stream, {} vertices, {} batches, {} draw calls, {:.2} ms",
                    s.def.id, st.vertices, st.batches, st.draw_calls, st.tessellate_ms
                ));
            }
        }
    }

    /// Draw a screen, from its device's X-Plane draw callback.
    fn draw_screen(&mut self, index: usize) {
        let (Some(xplm), Some(api)) = (self.xplm, self.api) else { return };
        // Nothing to draw for when another aircraft is loaded.
        let handle = self.screens[index].handle;
        if handle.is_null() || unsafe { (api.is_bound)(handle) } == 0 {
            return;
        }
        if self.renderer.is_none() {
            let r = gl::Renderer::new(api);
            match &r {
                Ok(r) => log(&format!(
                    "OpenGL ready, {}",
                    if r.anti_aliased() { format!("{}x multisampling", gl::SAMPLES) } else { "without multisampling".into() }
                )),
                Err(e) => log(&format!("cannot draw: {e}")),
            }
            self.renderer = Some(r);
        }
        let Some(Ok(renderer)) = self.renderer.as_mut() else { return };
        let framebuffer = self.fbo.map_or(0, |d| xplm.get_i(d)) as u32;
        let target = gl::Target { framebuffer, viewport: renderer.viewport() };
        let [_, _, vw, vh] = target.viewport;
        let def = self.screens[index].def;
        // X-Plane draws devices at the size they were made with; a mesh is
        // made again should the viewport ever differ.
        let scale = (vw as f64 / def.width as f64, vh as f64 / def.height as f64);
        if self.screens[index].scale != scale && scale.0 > 0. && scale.1 > 0. {
            // Tessellation makes the mesh again at this scale. Only needed
            // for the QuickJS path (below); harmless to keep current while
            // the bridge is drawing, since a screen it takes back over must
            // start from an up to date scale.
            self.screens[index].scale = scale;
            if let Ok(mut r) = RESCALED.lock() {
                r.push((index, scale));
            }
        }
        if self.draw_bridge_screen(index, target) {
            return;
        }
        let Some(Ok(renderer)) = self.renderer.as_mut() else { return };
        let s = &mut self.screens[index];
        let region = [0, 0, s.mesh.width as i32, s.mesh.height as i32];
        // [wxr] WXR_L/WXR_R fall back here when mapdata has nothing for an
        // id (terrain's own ids); crate::wxr::native_image, docs/wxr.md.
        let natives: Vec<_> =
            s.mesh.natives.iter().map(|id| crate::mapdata::plugin::native_image(id).or_else(|| crate::wxr::native_image(id))).collect();
        renderer.draw(&mut s.gpu, s.stream, &s.mesh, &s.steps, region, target, &s.brightness, &natives, &mut self.res);
    }

    /// The bridge path for one screen's draw callback (rules 6 and 7):
    /// upload the rectangles [`xphfbw::plan_upload`] plans into this
    /// screen's texture and draw it, its dimming regions applied on top
    /// just as the QuickJS path dims its mesh. `true` if this screen was
    /// drawn this way, so the caller must not also run the QuickJS path;
    /// `false` (no session attached, `displays_active` is 0, or this screen
    /// has no `ScreenBlock`) leaves it to the caller's QuickJS path.
    fn draw_bridge_screen(&mut self, index: usize, target: gl::Target) -> bool {
        if !self.bridge_active() {
            return false;
        }
        let def = self.screens[index].def;
        let brightness = self.screens[index].brightness.clone();
        let dimming: Vec<[f32; 4]> = def
            .dimming
            .iter()
            .map(|d| {
                let [x, y, w, h] = d.region;
                [x as f32, y as f32, (x + w) as f32, (y + h) as f32]
            })
            .collect();

        let Some(bridge) = self.bridge.as_mut() else { return false };
        let Some(Some(bs)) = bridge.screens.get_mut(index) else { return false };
        let header = bs.block.header();
        let width = header.width.load(Ordering::Relaxed);
        let height = header.height.load(Ordering::Relaxed);
        let now = std::time::Instant::now();
        let waited = bs.last_upload_at.map_or(UPLOAD_MAX_WAIT, |t| now - t);
        let bytes = width as usize * height as usize * 4;
        // A screen with no dimming regions (SCREEN_ISIS_1, Clock, RTPI, BAT,
        // SCREEN_EFB) dims in its own drawing rather than through this
        // mechanism (`ScreenDef::dimming`'s doc comment) and is never
        // considered dark here: `brightness.iter().all(..)` is vacuously
        // true on an empty slice, which would otherwise skip every upload
        // after the first (the "already dark, no point re-uploading" branch
        // below) and freeze such a screen on its first frame forever.
        let dark = !brightness.is_empty() && brightness.iter().all(|b| *b <= 0.);
        let due = header.frame.load(Ordering::Relaxed) != bs.last_uploaded
            && waited >= UPLOAD_MIN_INTERVAL
            && (bytes <= self.upload_budget || waited >= UPLOAD_MAX_WAIT)
            && !(dark && bs.last_upload_at.is_some());
        if due && header.writing.load(Ordering::Relaxed) == 0 {
            bs.last_upload_at = Some(now);
            let frame = header.frame.load(Ordering::Relaxed);
            let dirty = read_dirty_rects(header);
            let rects = xphfbw::plan_upload(frame, bs.last_uploaded, bs.force_full, &dirty, width, height);
            if !rects.is_empty() {
                let area: usize = rects.iter().map(|r| r[2] as usize * r[3] as usize * 4).sum();
                self.upload_budget = self.upload_budget.saturating_sub(area);
                let pixels = bs.block.pixels();
                if let Some(Ok(renderer)) = self.renderer.as_mut() {
                    renderer.bridge_upload(&mut bs.gpu, width, height, pixels, &rects);
                }
            }
            // Rule 6: re-read `frame`; if it moved during the upload above,
            // the rectangles just copied may already be stale, so the next
            // upload is forced to be a full one instead of trusting the
            // next dirty rects alone.
            let frame_after = bs.block.header().frame.load(Ordering::Relaxed);
            if xphfbw::torn_by_a_new_publish(frame, frame_after) {
                bs.force_full = true;
            } else {
                bs.force_full = false;
                bs.last_uploaded = frame;
            }
        }
        // The NDs: FlyByWire's terrain gauge (or the weather radar) under
        // nd.html, whose background is transparent (mapdata::plugin::
        // terrain_layer's compositing contract).
        let side = match def.id {
            "SCREEN_DU_NDL" => Some(crate::mapdata::terrain::types::Side::Left),
            "SCREEN_DU_NDR" => Some(crate::mapdata::terrain::types::Side::Right),
            _ => None,
        };
        let layer = side.and_then(|s| crate::mapdata::plugin::terrain_layer(s).or_else(|| crate::wxr::layer(s)));
        let mut has_underlay = false;
        if let Some((lw, lh, generation, rgba)) = layer {
            if generation != bs.underlay_generation {
                if let Some(Ok(renderer)) = self.renderer.as_mut() {
                    renderer.bridge_underlay_upload(&mut bs.underlay, lw, lh, &rgba);
                    bs.underlay_generation = generation;
                }
            }
            has_underlay = true;
        }
        // `writing` = 1: draw whatever was uploaded last tick and try the
        // upload again next frame, rather than skip the draw outright.
        if let Some(Ok(renderer)) = self.renderer.as_mut() {
            let underlay = has_underlay.then_some(&bs.underlay);
            renderer.draw_bridge(&mut bs.gpu, underlay, width, height, target, &brightness, &dimming);
        }
        true
    }

    /// A device's texel (origin bottom-left) as a point on its screen.
    fn to_screen(&self, screen: usize, x: c_int, y: c_int) -> (f64, f64) {
        let s = &self.screens[screen];
        let (sx, sy) = s.scale;
        let height = s.def.height as f64 * sy;
        xphfbw::device_to_css(x as f64, y as f64, height, (sx, sy))
    }

    /// Whether XPHFBW should be drawing (rule 7): `false` with no session
    /// attached, so every screen keeps the QuickJS drawing path and every
    /// pointer event keeps going to the scripts' `ScreenEvent`s.
    fn bridge_active(&self) -> bool {
        self.bridge.as_ref().is_some_and(|b| b.session.slots.header().displays_active.load(Ordering::Relaxed) != 0)
    }

    fn push_event(&mut self, screen: usize, kind: &'static str, x: c_int, y: c_int, button: i32, delta: f64) {
        let (cx, cy) = self.to_screen(screen, x, y);
        if self.events.len() == EVENT_LIMIT {
            self.events.pop_front();
        }
        let id = self.screens[screen].def.id;
        self.events.push_back(ScreenEvent { screen: id, kind, x: cx, y: cy, button, delta });
    }

    /// A pointer event on a screen: XPHFBW's `Input` ring while its displays
    /// are active (CSS pixel coordinates, same mapping as `push_event`'s
    /// `ScreenEvent`s; the ring has no room for a right-click distinction,
    /// so every button maps to the same `InputKind`), the scripts'
    /// `ScreenEvent` queue otherwise.
    fn dispatch_pointer(&mut self, screen: usize, screen_event_kind: &'static str, input_kind: InputKind, x: c_int, y: c_int, button: i32, delta: f64) {
        if self.bridge_active() {
            let (cx, cy) = self.to_screen(screen, x, y);
            if input_kind != InputKind::Move {
                static LOGGED: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
                if LOGGED.fetch_add(1, std::sync::atomic::Ordering::Relaxed) < 20 {
                    log(&format!("input: {} {screen_event_kind} at device ({x},{y}) -> page ({cx:.0},{cy:.0}), to XPHFBW", self.screens[screen].def.id));
                }
            }
            if let Some(bridge) = &self.bridge {
                let record = Input { screen: screen as u32, kind: input_kind, x: cx as f32, y: cy as f32, delta: delta as f32 };
                bridge.session.input.push(&record.encode());
            }
        } else {
            self.push_event(screen, screen_event_kind, x, y, button, delta);
        }
    }

    /// One keystroke from a device's `XPLMAvionicsKeyboard_f` callback (the
    /// KCCU typing into the MFD, [`InputKind::Key`]'s doc comment): only
    /// meaningful while XPHFBW is drawing (rule 7), since the QuickJS path
    /// has no keyboard route of its own. `false` (dropped silently) with no
    /// bridge attached or `displays_active` still 0.
    fn dispatch_key(&mut self, screen: usize, ch: u32, vkey: u32, flags: u32) -> bool {
        if !self.bridge_active() {
            return false;
        }
        let Some(bridge) = &self.bridge else { return false };
        let record = Input { screen: screen as u32, kind: InputKind::Key, x: ch as f32, y: vkey as f32, delta: flags as f32 };
        bridge.session.input.push(&record.encode());
        true
    }

    /// The KCCU gesture: right-click the MFD to pop it out and give it
    /// keyboard focus (so typing reaches it, [`mfd_keyboard_callback`]'s doc
    /// comment), right-click again to give the focus back and hide the
    /// popup. `has_keyboard_focus` (not local bookkeeping) decides which way
    /// to go, so this stays correct even if the popup was closed some other
    /// way (X-Plane's own popup chrome, Alt+F4, focus taken by another
    /// device) since the last toggle.
    fn toggle_mfd_keyboard(&mut self, index: usize) {
        let Some(api) = self.api else { return };
        let handle = self.screens[index].handle;
        if handle.is_null() {
            return;
        }
        unsafe {
            if (api.has_keyboard_focus)(handle) != 0 {
                api.release_keyboard_focus(handle);
                log("MFD: keyboard focus released, popup hidden");
            } else {
                (api.set_popup_visible)(handle, 1);
                (api.take_keyboard_focus)(handle);
                log("MFD: popped up and given keyboard focus (KCCU)");
            }
        }
    }

    /// The popup lost keyboard focus some other way (the user clicked
    /// outside it, or X-Plane's own window manager moved focus elsewhere):
    /// hide it too, so a stray popup does not linger with no way to type
    /// into it. Safe to call whether or not the popup is even visible.
    fn hide_mfd_popup(&mut self, index: usize) {
        let Some(api) = self.api else { return };
        let handle = self.screens[index].handle;
        if handle.is_null() {
            return;
        }
        unsafe { api.release_keyboard_focus(handle) };
    }

    fn mouse(&mut self, screen: usize, x: c_int, y: c_int, status: c_int, button: i32) {
        let (kind, input_kind) = match status {
            xp::MOUSE_DOWN => {
                self.screens[screen].pressed = true;
                ("down", InputKind::Down)
            }
            xp::MOUSE_DRAG => ("move", InputKind::Move),
            xp::MOUSE_UP => {
                self.screens[screen].pressed = false;
                ("up", InputKind::Up)
            }
            _ => return,
        };
        self.screens[screen].hover = Some((x, y));
        self.dispatch_pointer(screen, kind, input_kind, x, y, button, 0.);
    }

    fn cursor(&mut self, screen: usize, x: c_int, y: c_int) {
        let s = &mut self.screens[screen];
        if !s.pressed && s.hover != Some((x, y)) {
            s.hover = Some((x, y));
            self.dispatch_pointer(screen, "move", InputKind::Move, x, y, 0, 0.);
        }
    }

    /// Set each dimming region's brightness from the variables `read` gives.
    pub fn update_brightness(&mut self, mut read: impl FnMut(&str) -> Option<f64>) {
        // Once per X-Plane frame, from the plugin's tick: a fresh upload budget.
        self.upload_budget = UPLOAD_BUDGET_BYTES;
        for s in &mut self.screens {
            for (value, d) in s.brightness.iter_mut().zip(s.def.dimming) {
                let powered = d.buses.iter().any(|bus| read(bus).is_some_and(|v| v != 0.));
                *value = screens::brightness(read(&format!("LIGHT POTENTIOMETER:{}", d.potentiometer)), powered);
            }
        }
    }

    /// See [`set_bridge`].
    fn set_bridge(&mut self, tag: Option<&'static str>, session: Option<&'static Session>) {
        let Some((tag, session)) = tag.zip(session) else {
            if self.bridge.take().is_some() {
                log("XPHFBW bridge detached");
            }
            return;
        };
        let screens: Vec<Option<BridgeScreen>> = SCREENS
            .iter()
            .map(|def| {
                let block = ScreenBlock::create(tag, def.id, def.width, def.height);
                if block.is_none() {
                    log(&format!("could not create the XPHFBW ScreenBlock for {}", def.id));
                }
                block.map(|block| BridgeScreen { block, gpu: gl::BridgeGpu::default(), last_uploaded: 0, force_full: true, underlay: gl::BridgeGpu::default(), underlay_generation: 0, last_upload_at: None })
            })
            .collect();
        log(&format!("XPHFBW bridge attached ({tag}), {}/{} screens", screens.iter().filter(|s| s.is_some()).count(), SCREENS.len()));
        self.bridge = Some(Bridge { session, screens });
    }
}

/// The instruments' root: `html_ui` in the aircraft folder, two above the
/// plugin's (`<aircraft>/plugins/<plugin>`).
fn html_ui() -> Option<PathBuf> {
    let plugin = crate::js_bridge::plugin_dir()?;
    Some(plugin.parent()?.parent().unwrap_or(Path::new(".")).join(HTML_UI_DIR))
}

fn with<R>(f: impl FnOnce(&mut Displays) -> R) -> Option<R> {
    let mut guard = DISPLAYS.lock().ok()?;
    if guard.is_none() {
        *guard = Some(Displays::new(None, html_ui()?));
    }
    guard.as_mut().map(f)
}

fn with_tess<R>(f: impl FnOnce(&mut Tess) -> R) -> Option<R> {
    let mut guard = TESSELLATION.lock().ok()?;
    if guard.is_none() {
        *guard = Some(Tess::new(html_ui()?));
    }
    guard.as_mut().map(f)
}

/// Hand new meshes to the draw side, with the atlas and images they use.
fn hand_over(tess: &mut Tess, made: Vec<Made>) {
    if made.is_empty() {
        return;
    }
    let atlas = tess.atlas_update();
    let pictures: Vec<_> = tess.res.images.pictures[tess.sent_pictures..].to_vec();
    tess.sent_pictures = tess.res.images.pictures.len();
    let _ = with(|d| d.install(made, atlas, pictures));
}

/// For X-Plane's callbacks, which can arrive while the displays are in use
/// on the same thread (a device made while starting up): skipped then.
fn with_from_callback<R>(f: impl FnOnce(&mut Displays) -> R) -> Option<R> {
    let mut guard = DISPLAYS.try_lock().ok()?;
    guard.as_mut().map(f)
}

// The host functions the scripts call (docs/display-stream.md).

/// A screen's new stream, tessellated here and handed to the draw side.
pub fn submit(screen: &str, ops: &[f64], strings: Vec<String>) -> Result<(), String> {
    with_tess(|t| {
        let made = t.submit(screen, ops, strings)?;
        hand_over(t, made);
        Ok(())
    })
    .unwrap_or_else(|| Err("the displays are not running".into()))
}

/// Make again the meshes of screens whose devices changed scale. Called
/// where the streams are submitted, once a tick.
pub fn service() {
    let requests = RESCALED.lock().map(|mut r| std::mem::take(&mut *r)).unwrap_or_default();
    if requests.is_empty() {
        return;
    }
    let _ = with_tess(|t| {
        for (index, scale) in requests {
            let made = t.rescale(index, scale);
            hand_over(t, made);
        }
    });
}

pub fn measure_text(screen: Option<&str>, family: &str, size: f64, text: &str) -> f64 {
    with_tess(|t| t.measure_text(screen, family, size, text)).unwrap_or(0.)
}

pub fn font_metrics(screen: Option<&str>, family: &str, size: f64) -> (f64, f64) {
    with_tess(|t| t.font_metrics(screen, family, size)).unwrap_or((0., 0.))
}

pub fn screen_size(screen: &str) -> Option<(u32, u32)> {
    screens::find(screen).map(|i| (SCREENS[i].width, SCREENS[i].height))
}

/// Mouse events since the last call, oldest first.
pub fn take_events() -> Vec<ScreenEvent> {
    with(|d| d.events.drain(..).collect()).unwrap_or_default()
}

/// Read each screen's potentiometers and buses. `read` gives a variable's
/// value, or `None` if nothing has made the variable.
pub fn update_brightness(read: impl FnMut(&str) -> Option<f64>) {
    let _ = DISPLAYS.try_lock().map(|mut guard| guard.as_mut().map(|d| d.update_brightness(read)));
}

/// A screen's dirty rectangles as the app last set them (rule 6); at most
/// [`MAX_DIRTY_RECTS`].
fn read_dirty_rects(header: &ScreenHeader) -> Vec<[u32; 4]> {
    let count = (header.dirty_count.load(Ordering::Relaxed) as usize).min(MAX_DIRTY_RECTS);
    (0..count)
        .map(|i| {
            let at = i * 4;
            [
                header.dirty[at].load(Ordering::Relaxed),
                header.dirty[at + 1].load(Ordering::Relaxed),
                header.dirty[at + 2].load(Ordering::Relaxed),
                header.dirty[at + 3].load(Ordering::Relaxed),
            ]
        })
        .collect()
}

/// Attach (or detach) XPHFBW's session for this plugin run: agent C/A calls
/// this once the plugin's `Session` opens (`xphfbw_host.rs`, not merged at
/// the time this module was written — until it is, nothing calls this and
/// every screen simply keeps the QuickJS drawing path, rule 7), and again
/// with `None` when it closes (XPHFBW's process exits, or the session
/// restarts under a new tag, rule 9 — the caller must pass the new tag and
/// session, not reuse the old ones).
///
/// Creates this session's `ScreenBlock`s, one per [`SCREENS`] entry, sized
/// from panel.cfg: agent C's `xphfbw_bridge_views::view_list` once merged
/// (its sizes come from the same panel.cfg parse `SCREENS` is written
/// against, so no behaviour change is expected either way), `SCREENS`
/// itself until then.
pub fn set_bridge(tag: Option<&'static str>, session: Option<&'static Session>) {
    let _ = with(|d| d.set_bridge(tag, session));
}

/// Create the cockpit devices, one per screen. Called once the plugin is
/// enabled.
pub fn start(xplm: &'static Xplm) {
    let Some(api) = xplm.avionics() else {
        log("this X-Plane has no cockpit device API (12.1 or later needed); the screens are not shown");
        return;
    };
    let Some(root) = html_ui() else { return };
    {
        let Ok(mut guard) = DISPLAYS.lock() else { return };
        let mut displays = guard.take().unwrap_or_else(|| Displays::new(Some(xplm), root));
        displays.xplm = Some(xplm);
        displays.api = Some(api);
        displays.fbo = xplm.find("sim/graphics/view/current_gl_fbo");
        *guard = Some(displays);
    }
    // Made without holding the lock: X-Plane may call back straight away.
    let mut handles = Vec::new();
    for (index, def) in SCREENS.iter().enumerate() {
        let id = CString::new(def.id).expect("no nul");
        let title = CString::new(def.title).expect("no nul");
        let (w, h) = (def.width as c_int, def.height as c_int);
        let mut spec = xp::CreateAvionics {
            struct_size: std::mem::size_of::<xp::CreateAvionics>() as c_int,
            screen_width: w,
            screen_height: h,
            bezel_width: w,
            bezel_height: h,
            screen_offset_x: 0,
            screen_offset_y: 0,
            // Drawn every frame: replaying the cached batches is cheap, and
            // brightness changes need no bookkeeping.
            draw_on_demand: 0,
            bezel_draw: None,
            draw: Some(draw_callback),
            bezel_click: None,
            bezel_right_click: None,
            bezel_scroll: None,
            bezel_cursor: None,
            screen_touch: Some(touch_callback),
            screen_right_touch: Some(right_touch_callback),
            screen_scroll: Some(scroll_callback),
            screen_cursor: Some(cursor_callback),
            // The KCCU: only the MFD takes keyboard input (mfd_keyboard_callback's doc comment).
            keyboard: (def.id == "SCREEN_DU_MFD").then_some(mfd_keyboard_callback as xp::AvionicsKey),
            brightness: Some(brightness_callback),
            device_id: id.as_ptr(),
            device_name: title.as_ptr(),
            refcon: index as *mut c_void,
        };
        let handle = unsafe { (api.create)(&mut spec) };
        if handle.is_null() {
            log(&format!("X-Plane refused the device {}", def.id));
        }
        handles.push(handle);
    }
    let _ = with(|d| {
        for (screen, handle) in d.screens.iter_mut().zip(handles) {
            screen.handle = handle;
        }
        log(&format!("{} cockpit devices made", d.screens.iter().filter(|s| !s.handle.is_null()).count()));
    });
}

/// Destroy the devices and free what was made on the GPU.
pub fn stop(xplm: &'static Xplm) {
    let taken = DISPLAYS.lock().ok().and_then(|mut g| g.take());
    let Some(mut d) = taken else { return };
    if let Some(api) = xplm.avionics() {
        for s in &d.screens {
            if !s.handle.is_null() {
                unsafe { (api.destroy)(s.handle) };
            }
        }
    }
    if let Some(Ok(renderer)) = d.renderer.as_mut() {
        let mut gpus: Vec<&mut gl::ScreenGpu> = d.screens.iter_mut().map(|s| &mut s.gpu).collect();
        let mut bridges: Vec<&mut gl::BridgeGpu> =
            d.bridge.iter_mut().flat_map(|b| b.screens.iter_mut()).filter_map(|s| s.as_mut()).map(|bs| &mut bs.gpu).collect();
        renderer.release(&mut gpus, &mut bridges);
    }
}

fn screen_of(refcon: *mut c_void) -> usize {
    refcon as usize
}

unsafe extern "C" fn draw_callback(refcon: *mut c_void) {
    let _ = std::panic::catch_unwind(|| crate::perf::time("screens (draw callbacks)", || with_from_callback(|d| d.draw_screen(screen_of(refcon)))));
}

/// The first few pointer callbacks X-Plane makes, whatever happens to them
/// next: whether 3D-cockpit clicks reach the plugin at all.
fn log_callback(what: &str, refcon: *mut c_void, x: c_int, y: c_int, status: c_int, handled: bool) {
    static LOGGED: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
    if LOGGED.fetch_add(1, Ordering::Relaxed) < 40 {
        let id = SCREENS.get(screen_of(refcon)).map_or("?", |d| d.id);
        log(&format!("input callback: {what} on {id} at ({x},{y}) status {status}, displays lock taken: {handled}"));
    }
}

unsafe extern "C" fn touch_callback(x: c_int, y: c_int, status: c_int, refcon: *mut c_void) -> c_int {
    let handled = std::panic::catch_unwind(|| with_from_callback(|d| d.mouse(screen_of(refcon), x, y, status, 0)).is_some()).unwrap_or(false);
    log_callback("touch", refcon, x, y, status, handled);
    1
}

unsafe extern "C" fn right_touch_callback(x: c_int, y: c_int, status: c_int, refcon: *mut c_void) -> c_int {
    let index = screen_of(refcon);
    // The KCCU gesture (toggle_mfd_keyboard's doc comment): only the MFD,
    // only on the initial press, and it does not also forward as a click.
    if status == xp::MOUSE_DOWN && SCREENS.get(index).is_some_and(|d| d.id == "SCREEN_DU_MFD") {
        let _ = std::panic::catch_unwind(|| with_from_callback(|d| d.toggle_mfd_keyboard(index)));
        return 1;
    }
    let _ = std::panic::catch_unwind(|| with_from_callback(|d| d.mouse(index, x, y, status, 2)));
    1
}

unsafe extern "C" fn scroll_callback(x: c_int, y: c_int, wheel: c_int, clicks: c_int, refcon: *mut c_void) -> c_int {
    // Vertical only: the contract's wheel event has one delta. A DOM wheel
    // event's deltaY is positive scrolling down, towards the user, about
    // 100 CSS pixels a notch; X-Plane counts clicks positive away from the
    // user.
    if wheel != 0 {
        return 0;
    }
    let _ = std::panic::catch_unwind(|| {
        with_from_callback(|d| d.dispatch_pointer(screen_of(refcon), "wheel", InputKind::Wheel, x, y, 0, -100. * clicks as f64))
    });
    1
}

unsafe extern "C" fn cursor_callback(x: c_int, y: c_int, refcon: *mut c_void) -> c_int {
    let handled = std::panic::catch_unwind(|| with_from_callback(|d| d.cursor(screen_of(refcon), x, y)).is_some()).unwrap_or(false);
    static FIRST: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
    if FIRST.fetch_add(1, Ordering::Relaxed) < 5 {
        log_callback("cursor", refcon, x, y, 0, handled);
    }
    xp::CURSOR_DEFAULT
}

/// Full brightness: the dimming FlyByWire's model does (potentiometer and
/// bus power) is drawn into the texture per region, which X-Plane's single
/// per-device brightness could not do for the two MFDs on one texture.
unsafe extern "C" fn brightness_callback(_rheo: f32, _ambient: f32, _bus: f32, _refcon: *mut c_void) -> f32 {
    1.
}

/// The KCCU types into the MFD through this: `XPLMAvionicsKeyboard_f`
/// (XPLMDisplay.h) fires "when your device is popped up and you've
/// requested to capture the keyboard", i.e. only once
/// `XPLMTakeAvionicsKeyboardFocus` has been called for this device's popup
/// window — XPLM410 has no way to capture the keyboard for a device
/// embedded in the 3D cockpit alone. [`right_touch_callback`] gives the MFD
/// that popup and focus (right-click it; right-click again, or click away,
/// gives the focus back — [`Displays::toggle_mfd_keyboard`]).
///
/// Only registered on the MFD's device ([`start`]): the KCCU is the MFD's
/// keyboard, not a general cockpit keyboard.
unsafe extern "C" fn mfd_keyboard_callback(key: c_char, flags: c_int, vkey: c_char, refcon: *mut c_void, losing_focus: c_int) -> c_int {
    if losing_focus != 0 {
        // hide_mfd_popup's doc comment: close the popup along with the
        // focus, however it was lost, so it never lingers unfocused.
        let _ = std::panic::catch_unwind(|| with_from_callback(|d| d.hide_mfd_popup(screen_of(refcon))));
        return 0;
    }
    let ch = key as u8 as u32;
    let handled = std::panic::catch_unwind(|| {
        with_from_callback(|d| d.dispatch_key(screen_of(refcon), ch, vkey as u8 as u32, flags as u32)).unwrap_or(false)
    })
    .unwrap_or(false);
    static LOGGED: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
    if LOGGED.fetch_add(1, Ordering::Relaxed) < 40 {
        log(&format!(
            "input callback: key {:?} (vkey {}) flags {:#x} on the MFD, handled: {handled}",
            char::from_u32(ch).filter(|_| ch != 0),
            vkey as u8,
            flags
        ));
    }
    handled as c_int
}
