//! Drawing a screen's mesh with OpenGL, inside X-Plane's avionics callback.
//!
//! X-Plane 12 draws with Vulkan and gives plugins an OpenGL context of their
//! own, bridged to its textures. The rules this code keeps
//! (developer.x-plane.com, "Plugin Guidance for OpenGL Drawing"):
//!
//! - Blending, texturing, alpha test, depth test and depth writes go through
//!   `XPLMSetGraphicsState`, and 2-D textures bind through
//!   `XPLMBindTexture2d`, never glEnable or glBindTexture directly.
//! - Any other state is put back as found before returning: shaders, VAOs,
//!   VBOs and non-2-D textures unbound, only the fixed-function vertex array
//!   left enabled.
//! - After drawing into an FBO of our own, the FBO X-Plane had bound is
//!   bound again, read from `sim/graphics/view/current_gl_fbo` rather than
//!   glGet; framebuffer completeness is checked once, when it is made.
//! - No glGetError in the per-draw hot path (it stalls the pipeline once a
//!   frame is enough to notice, every triangle is not). It is checked after
//!   the rare, state-establishing uploads in [`Renderer::upload`] --
//!   texture and vertex buffer creation, which only run when an atlas
//!   generation, image or mesh actually changed -- since a silently
//!   rejected texture format or buffer upload there (GL_INVALID_ENUM,
//!   GL_INVALID_VALUE, GL_OUT_OF_MEMORY) is exactly the kind of failure
//!   that would leave geometry undrawn while everything sharing a
//!   different, unaffected call still works.
//!
//! Anti-aliasing is 4x multisampling. The mesh is drawn into a multisampled
//! framebuffer with a stencil buffer (for clip paths and translucent
//! strokes), in tiles of at most `TILE` pixels so one framebuffer serves
//! every screen, and each tile is resolved into the device's texture with
//! glBlitFramebuffer. Without framebuffer objects it draws straight into
//! X-Plane's target, without anti-aliasing and without clip paths.

use std::ffi::{c_char, c_int, c_uint, c_void, CString};

use std::collections::HashMap;
use std::sync::Arc;

use super::plan::{Step, StencilTest, LEVELS, MARK};
use super::tessellate::{Mesh, Paint, Resources, Vertex, RAMP};
use crate::mapdata::terrain::terronnd::NativeImage;
use crate::xp::AvionicsApi;

/// The multisampled framebuffer's size: screens larger than this draw in
/// tiles. 1024 covers every CDS screen in one or two tiles.
pub const TILE: i32 = 1024;
/// Samples per pixel. See docs/screens.md for the measurements behind four.
pub const SAMPLES: i32 = 4;

const GL_TRIANGLES: c_uint = 0x0004;
const GL_SCISSOR_TEST: c_uint = 0x0C11;
const GL_STENCIL_TEST: c_uint = 0x0B90;
const GL_VIEWPORT: c_uint = 0x0BA2;
const GL_COLOR_BUFFER_BIT: c_uint = 0x4000;
const GL_STENCIL_BUFFER_BIT: c_uint = 0x0400;
const GL_PROJECTION: c_uint = 0x1701;
const GL_MODELVIEW: c_uint = 0x1700;
const GL_TEXTURE: c_uint = 0x1702;
const GL_VERTEX_ARRAY: c_uint = 0x8074;
const GL_COLOR_ARRAY: c_uint = 0x8076;
const GL_TEXTURE_COORD_ARRAY: c_uint = 0x8078;
const GL_FLOAT: c_uint = 0x1406;
const GL_UNSIGNED_BYTE: c_uint = 0x1401;
const GL_ARRAY_BUFFER: c_uint = 0x8892;
const GL_DYNAMIC_DRAW: c_uint = 0x88E8;
const GL_TEXTURE_2D: c_uint = 0x0DE1;
const GL_ALPHA: c_uint = 0x1906;
const GL_RGBA: c_uint = 0x1908;
const GL_ALPHA8: c_int = 0x803C;
const GL_RGBA8: c_int = 0x8058;
const GL_TEXTURE_MIN_FILTER: c_uint = 0x2801;
const GL_TEXTURE_MAG_FILTER: c_uint = 0x2800;
const GL_LINEAR: c_int = 0x2601;
const GL_NEAREST: c_uint = 0x2600;
const GL_LINEAR_MIPMAP_LINEAR: c_int = 0x2703;
const GL_TEXTURE_WRAP_S: c_uint = 0x2802;
const GL_TEXTURE_WRAP_T: c_uint = 0x2803;
const GL_CLAMP_TO_EDGE: c_int = 0x812F;
const GL_TEXTURE_MAX_LEVEL: c_uint = 0x813D;
const GL_UNPACK_ALIGNMENT: c_uint = 0x0CF5;
const GL_TEXTURE_ENV: c_uint = 0x2300;
const GL_TEXTURE_ENV_MODE: c_uint = 0x2200;
const GL_MODULATE: c_int = 0x2100;
const GL_FRAMEBUFFER: c_uint = 0x8D40;
const GL_READ_FRAMEBUFFER: c_uint = 0x8CA8;
const GL_DRAW_FRAMEBUFFER: c_uint = 0x8CA9;
const GL_RENDERBUFFER: c_uint = 0x8D41;
const GL_COLOR_ATTACHMENT0: c_uint = 0x8CE0;
const GL_DEPTH_STENCIL_ATTACHMENT: c_uint = 0x821A;
const GL_DEPTH24_STENCIL8: c_uint = 0x88F0;
const GL_FRAMEBUFFER_COMPLETE: c_uint = 0x8CD5;
const GL_MAX_SAMPLES: c_uint = 0x8D57;
const GL_STENCIL_BITS: c_uint = 0x0D57;
const GL_SAMPLES: c_uint = 0x80A9;
const GL_KEEP: c_uint = 0x1E00;
const GL_INCR: c_uint = 0x1E02;
const GL_INVERT: c_uint = 0x150A;
const GL_EQUAL: c_uint = 0x0202;
const GL_SRC_ALPHA: c_uint = 0x0302;
const GL_ONE_MINUS_SRC_ALPHA: c_uint = 0x0303;
const GL_ONE: c_uint = 1;
const GL_BGRA: c_uint = 0x80E1;
const GL_UNPACK_ROW_LENGTH: c_uint = 0x0CF2;
const GL_NO_ERROR: c_uint = 0;
const GL_INVALID_ENUM: c_uint = 0x0500;
const GL_INVALID_VALUE: c_uint = 0x0501;
const GL_INVALID_OPERATION: c_uint = 0x0502;
const GL_STACK_OVERFLOW: c_uint = 0x0503;
const GL_STACK_UNDERFLOW: c_uint = 0x0504;
const GL_OUT_OF_MEMORY: c_uint = 0x0505;
const GL_INVALID_FRAMEBUFFER_OPERATION: c_uint = 0x0506;

/// A human name for a `glGetError` code, for the log; the numeric value too,
/// since a code this list does not recognise is still worth reporting.
fn gl_error_name(code: c_uint) -> String {
    match code {
        GL_INVALID_ENUM => "GL_INVALID_ENUM".into(),
        GL_INVALID_VALUE => "GL_INVALID_VALUE".into(),
        GL_INVALID_OPERATION => "GL_INVALID_OPERATION".into(),
        GL_STACK_OVERFLOW => "GL_STACK_OVERFLOW".into(),
        GL_STACK_UNDERFLOW => "GL_STACK_UNDERFLOW".into(),
        GL_OUT_OF_MEMORY => "GL_OUT_OF_MEMORY".into(),
        GL_INVALID_FRAMEBUFFER_OPERATION => "GL_INVALID_FRAMEBUFFER_OPERATION".into(),
        other => format!("0x{other:04X}"),
    }
}

/// Drain and log every pending `glGetError` (there can be more than one),
/// once per distinct `site` (`warned` is [`Renderer::gl_error_warned`]).
/// Only called after the rare, state-establishing uploads in
/// [`Renderer::upload`] -- never in the per-triangle draw loop.
fn check_upload_errors(gl: &Fns, warned: &mut std::collections::HashSet<&'static str>, site: &'static str) {
    loop {
        let code = unsafe { (gl.get_error)() };
        if code == GL_NO_ERROR {
            return;
        }
        if warned.insert(site) {
            crate::log(&format!("display: GL error {} after {site}", gl_error_name(code)));
        }
    }
}

#[link(name = "kernel32")]
extern "system" {
    fn LoadLibraryA(name: *const c_char) -> *mut c_void;
    fn GetProcAddress(module: *mut c_void, name: *const c_char) -> *mut c_void;
}

macro_rules! gl_functions {
    ($($field:ident: $name:literal => fn($($arg:ty),*) $(-> $ret:ty)?;)*) => {
        #[allow(non_snake_case)]
        struct Fns {
            $($field: unsafe extern "system" fn($($arg),*) $(-> $ret)?,)*
        }

        impl Fns {
            /// Every function, from opengl32.dll or, for those it does not
            /// export (anything past OpenGL 1.1), from the current context.
            unsafe fn load() -> Result<Self, String> {
                let dll = CString::new("opengl32.dll").expect("no nul");
                let module = LoadLibraryA(dll.as_ptr());
                if module.is_null() {
                    return Err("opengl32.dll is not loaded".into());
                }
                let wgl = GetProcAddress(module, c"wglGetProcAddress".as_ptr());
                let wgl: Option<unsafe extern "system" fn(*const c_char) -> *mut c_void> =
                    (!wgl.is_null()).then(|| std::mem::transmute(wgl));
                let find = |name: &str| -> Result<*mut c_void, String> {
                    let c = CString::new(name).expect("no nul");
                    let mut p = GetProcAddress(module, c.as_ptr());
                    if p.is_null() {
                        if let Some(wgl) = wgl {
                            p = wgl(c.as_ptr());
                        }
                    }
                    // wglGetProcAddress may answer 1, 2, 3 or -1 for "no".
                    if (p as isize) >= -1 && (p as isize) <= 3 {
                        return Err(format!("OpenGL has no {name}"));
                    }
                    Ok(p)
                };
                Ok(Fns { $($field: std::mem::transmute::<*mut c_void, unsafe extern "system" fn($($arg),*) $(-> $ret)?>(find($name)?),)* })
            }
        }
    };
}

gl_functions! {
    enable: "glEnable" => fn(c_uint);
    disable: "glDisable" => fn(c_uint);
    is_enabled: "glIsEnabled" => fn(c_uint) -> u8;
    get_integer: "glGetIntegerv" => fn(c_uint, *mut c_int);
    viewport: "glViewport" => fn(c_int, c_int, c_int, c_int);
    scissor: "glScissor" => fn(c_int, c_int, c_int, c_int);
    clear: "glClear" => fn(c_uint);
    clear_color: "glClearColor" => fn(f32, f32, f32, f32);
    clear_stencil: "glClearStencil" => fn(c_int);
    color_mask: "glColorMask" => fn(u8, u8, u8, u8);
    stencil_func: "glStencilFunc" => fn(c_uint, c_int, c_uint);
    stencil_op: "glStencilOp" => fn(c_uint, c_uint, c_uint);
    stencil_mask: "glStencilMask" => fn(c_uint);
    blend_func: "glBlendFunc" => fn(c_uint, c_uint);
    blend_func_separate: "glBlendFuncSeparate" => fn(c_uint, c_uint, c_uint, c_uint);
    matrix_mode: "glMatrixMode" => fn(c_uint);
    push_matrix: "glPushMatrix" => fn();
    pop_matrix: "glPopMatrix" => fn();
    load_identity: "glLoadIdentity" => fn();
    ortho: "glOrtho" => fn(f64, f64, f64, f64, f64, f64);
    scale: "glScaled" => fn(f64, f64, f64);
    color4f: "glColor4f" => fn(f32, f32, f32, f32);
    enable_client_state: "glEnableClientState" => fn(c_uint);
    disable_client_state: "glDisableClientState" => fn(c_uint);
    vertex_pointer: "glVertexPointer" => fn(c_int, c_uint, c_int, *const c_void);
    tex_coord_pointer: "glTexCoordPointer" => fn(c_int, c_uint, c_int, *const c_void);
    color_pointer: "glColorPointer" => fn(c_int, c_uint, c_int, *const c_void);
    draw_arrays: "glDrawArrays" => fn(c_uint, c_int, c_int);
    tex_image: "glTexImage2D" => fn(c_uint, c_int, c_int, c_int, c_int, c_int, c_uint, c_uint, *const c_void);
    tex_sub_image: "glTexSubImage2D" => fn(c_uint, c_int, c_int, c_int, c_int, c_int, c_uint, c_uint, *const c_void);
    tex_parameter: "glTexParameteri" => fn(c_uint, c_uint, c_int);
    tex_env: "glTexEnvi" => fn(c_uint, c_uint, c_int);
    pixel_store: "glPixelStorei" => fn(c_uint, c_int);
    delete_textures: "glDeleteTextures" => fn(c_int, *const c_uint);
    use_program: "glUseProgram" => fn(c_uint);
    bind_vertex_array: "glBindVertexArray" => fn(c_uint);
    gen_buffers: "glGenBuffers" => fn(c_int, *mut c_uint);
    bind_buffer: "glBindBuffer" => fn(c_uint, c_uint);
    buffer_data: "glBufferData" => fn(c_uint, isize, *const c_void, c_uint);
    delete_buffers: "glDeleteBuffers" => fn(c_int, *const c_uint);
    gen_framebuffers: "glGenFramebuffers" => fn(c_int, *mut c_uint);
    bind_framebuffer: "glBindFramebuffer" => fn(c_uint, c_uint);
    delete_framebuffers: "glDeleteFramebuffers" => fn(c_int, *const c_uint);
    gen_renderbuffers: "glGenRenderbuffers" => fn(c_int, *mut c_uint);
    bind_renderbuffer: "glBindRenderbuffer" => fn(c_uint, c_uint);
    delete_renderbuffers: "glDeleteRenderbuffers" => fn(c_int, *const c_uint);
    renderbuffer_storage_multisample: "glRenderbufferStorageMultisample" => fn(c_uint, c_int, c_uint, c_int, c_int);
    framebuffer_renderbuffer: "glFramebufferRenderbuffer" => fn(c_uint, c_uint, c_uint, c_uint);
    check_framebuffer_status: "glCheckFramebufferStatus" => fn(c_uint) -> c_uint;
    blit_framebuffer: "glBlitFramebuffer" => fn(c_int, c_int, c_int, c_int, c_int, c_int, c_int, c_int, c_uint, c_uint);
    get_error: "glGetError" => fn() -> c_uint;
}

/// The multisampled framebuffer every screen draws through.
struct Multisample {
    framebuffer: c_uint,
    colour: c_uint,
    depth_stencil: c_uint,
}

/// A screen's GPU copy of its mesh.
#[derive(Default)]
pub struct ScreenGpu {
    buffer: c_uint,
    /// The mesh (by its stream number) the buffer holds.
    uploaded: u64,
    gradients: c_int,
    gradient_rows: usize,
}

/// A screen's GPU copy of XPHFBW's pixels (docs/briefs/xphfbw-js-bridge.md
/// rule 6), drawn as a textured quad in place of the tessellated mesh while
/// `displays_active`.
#[derive(Default)]
pub struct BridgeGpu {
    texture: c_int,
    /// The quad's vertex buffer (rebuilt each draw: the full screen, then
    /// one quad per dimmed region).
    buffer: c_uint,
    /// The texture's current size, so a size change is caught and uploaded
    /// whole even if the caller thinks it only has a dirty rect.
    size: (u32, u32),
}

/// A device's picture of where it is drawing, from X-Plane's state on entry.
#[derive(Clone, Copy)]
pub struct Target {
    pub framebuffer: c_uint,
    pub viewport: [c_int; 4],
}

pub struct Renderer {
    gl: Fns,
    xp: AvionicsApi,
    multisample: Option<Multisample>,
    atlas_texture: c_int,
    atlas_size: u32,
    atlas_generation: u64,
    /// X-Plane texture ids for images, by image index.
    images: Vec<c_int>,
    /// Native images by id: texture, and the generation and size it holds.
    natives: HashMap<String, (c_int, u64, u32, u32)>,
    /// This frame's texture for each of the mesh's native images, 0 for
    /// one with nothing to draw.
    native_textures: Vec<c_int>,
    /// Draw calls in the last frame, for the statistics.
    pub draw_calls: usize,
    stencil_warned: bool,
    /// Call sites `check_upload_errors` has already logged a GL error for,
    /// so a state that keeps failing (e.g. every frame's mesh re-upload on
    /// a driver that rejects it) is reported once, not forever.
    gl_error_warned: std::collections::HashSet<&'static str>,
    /// Whether X-Plane's device target is itself multisampled, asked once:
    /// a multisampled framebuffer cannot be resolved into one, so the mesh
    /// is then drawn straight into it with its own samples.
    target_multisampled: Option<bool>,
}

/// Where in the target a region of the screen lands.
#[derive(Clone, Copy)]
struct Frame {
    /// The region, in device pixels with y down: x0, y0, x1, y1.
    region: [i32; 4],
    /// The target pixel (bottom-left origin) the region's bottom-left maps to.
    origin: [i32; 2],
    /// Scissoring has to stay on, because the target is shared.
    always_scissor: bool,
    /// Target pixels per region pixel.
    scale: (f64, f64),
}

impl Frame {
    fn scissor_box(&self, rect: Option<[i32; 4]>) -> [c_int; 4] {
        let [rx0, ry0, rx1, ry1] = self.region;
        let [x0, y0, x1, y1] = rect.unwrap_or(self.region);
        let (x0, y0, x1, y1) = (x0.max(rx0), y0.max(ry0), x1.min(rx1), y1.min(ry1));
        if x0 >= x1 || y0 >= y1 {
            return [self.origin[0], self.origin[1], 0, 0];
        }
        let (sx, sy) = self.scale;
        let left = (self.origin[0] as f64 + (x0 - rx0) as f64 * sx).round() as c_int;
        let right = (self.origin[0] as f64 + (x1 - rx0) as f64 * sx).round() as c_int;
        let bottom = (self.origin[1] as f64 + (ry1 - y1) as f64 * sy).round() as c_int;
        let top = (self.origin[1] as f64 + (ry1 - y0) as f64 * sy).round() as c_int;
        [left, bottom, right - left, top - bottom]
    }
}

impl Renderer {
    /// Bind OpenGL. Must run with X-Plane's plugin context current, which is
    /// the case inside drawing callbacks.
    ///
    /// Before this, `xphfbw.displaySafeMode` (the app's "Use display safe
    /// mode" checkbox, app/ui/index.html) was read by nobody on the plugin
    /// side: a pilot who ticked it because their screens stayed black or
    /// flickered saw no change at all, since nothing in this module ever
    /// looked at the setting. The multisampled framebuffer this constructor
    /// makes is the one hardware-dependent piece of the display path (an
    /// FBO with a 4x `GL_MAX_SAMPLES` renderbuffer, [`make_multisample`]);
    /// safe mode now skips it, falling back to the plain, always-available
    /// path already in [`Renderer::draw`] (the `_ =>` arm, straight into
    /// X-Plane's target with no anti-aliasing and no clip paths) -- exactly
    /// what the checkbox's own tooltip promises.
    pub fn new(xp: AvionicsApi) -> Result<Self, String> {
        let gl = unsafe { Fns::load()? };
        let safe_mode = display_safe_mode();
        let mut r = Renderer {
            gl,
            xp,
            multisample: None,
            atlas_texture: 0,
            atlas_size: 0,
            atlas_generation: 0,
            images: Vec::new(),
            natives: HashMap::new(),
            native_textures: Vec::new(),
            draw_calls: 0,
            stencil_warned: false,
            gl_error_warned: std::collections::HashSet::new(),
            target_multisampled: None,
        };
        if safe_mode {
            crate::log("display: xphfbw.displaySafeMode is on; skipping the multisampled framebuffer (no anti-aliasing, no clip paths)");
        } else {
            r.multisample = unsafe { r.make_multisample() };
        }
        Ok(r)
    }

    /// The viewport X-Plane set for the device being drawn.
    pub fn viewport(&self) -> [c_int; 4] {
        let mut v = [0; 4];
        unsafe { (self.gl.get_integer)(GL_VIEWPORT, v.as_mut_ptr()) };
        v
    }

    pub fn anti_aliased(&self) -> bool {
        self.multisample.is_some()
    }

    unsafe fn make_multisample(&self) -> Option<Multisample> {
        let gl = &self.gl;
        let mut max = 0;
        (gl.get_integer)(GL_MAX_SAMPLES, &mut max);
        if max < SAMPLES {
            return None;
        }
        let (mut framebuffer, mut colour, mut depth_stencil) = (0, 0, 0);
        (gl.gen_framebuffers)(1, &mut framebuffer);
        (gl.gen_renderbuffers)(1, &mut colour);
        (gl.gen_renderbuffers)(1, &mut depth_stencil);
        (gl.bind_renderbuffer)(GL_RENDERBUFFER, colour);
        (gl.renderbuffer_storage_multisample)(GL_RENDERBUFFER, SAMPLES, GL_RGBA8 as c_uint, TILE, TILE);
        (gl.bind_renderbuffer)(GL_RENDERBUFFER, depth_stencil);
        (gl.renderbuffer_storage_multisample)(GL_RENDERBUFFER, SAMPLES, GL_DEPTH24_STENCIL8, TILE, TILE);
        (gl.bind_renderbuffer)(GL_RENDERBUFFER, 0);
        (gl.bind_framebuffer)(GL_FRAMEBUFFER, framebuffer);
        (gl.framebuffer_renderbuffer)(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colour);
        (gl.framebuffer_renderbuffer)(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, depth_stencil);
        let complete = (gl.check_framebuffer_status)(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
        let ms = Multisample { framebuffer, colour, depth_stencil };
        if complete {
            Some(ms)
        } else {
            Self::delete_multisample(gl, &ms);
            None
        }
    }

    fn delete_multisample(gl: &Fns, ms: &Multisample) {
        unsafe {
            (gl.delete_framebuffers)(1, &ms.framebuffer);
            (gl.delete_renderbuffers)(1, &ms.colour);
            (gl.delete_renderbuffers)(1, &ms.depth_stencil);
        }
    }

    /// Free everything this renderer made.
    pub fn release(&mut self, screens: &mut [&mut ScreenGpu], bridges: &mut [&mut BridgeGpu]) {
        let gl = &self.gl;
        unsafe {
            if let Some(ms) = self.multisample.take() {
                Self::delete_multisample(gl, &ms);
            }
            let mut textures: Vec<c_uint> = self.images.drain(..).map(|t| t as c_uint).collect();
            textures.extend(self.natives.drain().map(|(_, (t, ..))| t as c_uint));
            if self.atlas_texture != 0 {
                textures.push(self.atlas_texture as c_uint);
            }
            for s in screens.iter_mut() {
                if s.buffer != 0 {
                    (gl.delete_buffers)(1, &s.buffer);
                }
                if s.gradients != 0 {
                    textures.push(s.gradients as c_uint);
                }
                **s = ScreenGpu::default();
            }
            for b in bridges.iter_mut() {
                if b.buffer != 0 {
                    (gl.delete_buffers)(1, &b.buffer);
                }
                if b.texture != 0 {
                    textures.push(b.texture as c_uint);
                }
                **b = BridgeGpu::default();
            }
            if !textures.is_empty() {
                (gl.delete_textures)(textures.len() as c_int, textures.as_ptr());
            }
        }
        self.atlas_texture = 0;
    }

    fn new_texture(&self) -> c_int {
        let mut id = 0;
        unsafe { (self.xp.generate_textures)(&mut id, 1) };
        id
    }

    unsafe fn texture_parameters(&self, filter: c_int, levels: c_int) {
        let gl = &self.gl;
        (gl.tex_parameter)(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
        (gl.tex_parameter)(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        (gl.tex_parameter)(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        (gl.tex_parameter)(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        (gl.tex_parameter)(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, levels - 1);
    }

    /// Bring the atlas, the images the mesh uses and the mesh itself up to
    /// date on the GPU.
    unsafe fn upload(&mut self, gpu: &mut ScreenGpu, stream: u64, mesh: &Mesh, res: &mut Resources, natives: &[Option<Arc<NativeImage>>]) {
        let gl = &self.gl;
        (gl.pixel_store)(GL_UNPACK_ALIGNMENT, 1);
        let atlas = &mut res.atlas;
        if self.atlas_texture == 0 || self.atlas_size != atlas.size || self.atlas_generation != atlas.generation {
            if self.atlas_texture == 0 {
                self.atlas_texture = self.new_texture();
            }
            (self.xp.bind_texture)(self.atlas_texture, 0);
            (gl.tex_image)(GL_TEXTURE_2D, 0, GL_ALPHA8, atlas.size as c_int, atlas.size as c_int, 0, GL_ALPHA, GL_UNSIGNED_BYTE, atlas.pixels.as_ptr().cast());
            self.texture_parameters(GL_LINEAR, 1);
            self.atlas_size = atlas.size;
            self.atlas_generation = atlas.generation;
            atlas.dirty = None;
            // The call most worth checking in this file: GL_ALPHA / GL_ALPHA8
            // is a single-channel legacy format removed from core-profile
            // OpenGL. If X-Plane ever hands a plugin a core context instead
            // of the compatibility one this whole renderer assumes, this is
            // where it would first show -- as a rejected atlas upload, while
            // the images and native textures below (GL_RGBA8, never legacy)
            // keep working. That alone would not explain text surviving
            // (glyphs share this same atlas texture), but a texture that
            // fails here can be left holding whatever the driver had bound
            // before it, so partial/stale sampling is not ruled out either;
            // this makes that visible instead of silent.
            check_upload_errors(gl, &mut self.gl_error_warned, "atlas texture upload (GL_ALPHA8)");
        } else if let Some((first, last)) = atlas.dirty.take() {
            (self.xp.bind_texture)(self.atlas_texture, 0);
            let row = (first * atlas.size) as usize;
            (gl.tex_sub_image)(
                GL_TEXTURE_2D,
                0,
                0,
                first as c_int,
                atlas.size as c_int,
                (last - first) as c_int,
                GL_ALPHA,
                GL_UNSIGNED_BYTE,
                atlas.pixels[row..].as_ptr().cast(),
            );
            check_upload_errors(gl, &mut self.gl_error_warned, "atlas texture sub-upload (GL_ALPHA)");
        }
        for batch in &mesh.batches {
            if let Paint::Image(i) = batch.paint {
                if self.images.len() <= i {
                    self.images.resize(i + 1, 0);
                }
                if self.images[i] == 0 {
                    let picture = &res.images.pictures[i];
                    self.images[i] = self.new_texture();
                    (self.xp.bind_texture)(self.images[i], 0);
                    for (level, (w, h, pixels)) in picture.levels.iter().enumerate() {
                        (gl.tex_image)(GL_TEXTURE_2D, level as c_int, GL_RGBA8, *w as c_int, *h as c_int, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels.as_ptr().cast());
                    }
                    self.texture_parameters(GL_LINEAR_MIPMAP_LINEAR, picture.levels.len() as c_int);
                    check_upload_errors(gl, &mut self.gl_error_warned, "picture texture upload");
                }
            }
        }
        self.native_textures.clear();
        for (id, image) in mesh.natives.iter().zip(natives) {
            let Some(image) = image.as_ref().filter(|i| i.width > 0 && i.height > 0 && i.rgba.len() >= (i.width * i.height * 4) as usize) else {
                self.native_textures.push(0);
                continue;
            };
            let (texture, generation, w, h) = match self.natives.get(id) {
                Some(e) => *e,
                None => (self.new_texture(), u64::MAX, 0, 0),
            };
            if generation != image.generation || w != image.width || h != image.height {
                (self.xp.bind_texture)(texture, 0);
                let (iw, ih) = (image.width as c_int, image.height as c_int);
                if w == image.width && h == image.height {
                    (gl.tex_sub_image)(GL_TEXTURE_2D, 0, 0, 0, iw, ih, GL_RGBA, GL_UNSIGNED_BYTE, image.rgba.as_ptr().cast());
                } else {
                    (gl.tex_image)(GL_TEXTURE_2D, 0, GL_RGBA8, iw, ih, 0, GL_RGBA, GL_UNSIGNED_BYTE, image.rgba.as_ptr().cast());
                    self.texture_parameters(GL_LINEAR, 1);
                }
                self.natives.insert(id.clone(), (texture, image.generation, image.width, image.height));
                check_upload_errors(gl, &mut self.gl_error_warned, "native image texture upload");
            }
            self.native_textures.push(texture);
        }
        if gpu.uploaded != stream {
            if gpu.buffer == 0 {
                (gl.gen_buffers)(1, &mut gpu.buffer);
            }
            (gl.bind_buffer)(GL_ARRAY_BUFFER, gpu.buffer);
            (gl.buffer_data)(
                GL_ARRAY_BUFFER,
                std::mem::size_of_val(&mesh.vertices[..]) as isize,
                mesh.vertices.as_ptr().cast(),
                GL_DYNAMIC_DRAW,
            );
            // Every vertex the mesh has -- text glyph quads and flat-colour
            // fill/stroke triangles alike -- lands in this one buffer.
            // GL_OUT_OF_MEMORY or a rejected size here would drop both
            // together, not one selectively; still worth knowing.
            check_upload_errors(gl, &mut self.gl_error_warned, "vertex buffer upload");
            (gl.bind_buffer)(GL_ARRAY_BUFFER, 0);
            gpu.gradient_rows = mesh.gradients.len();
            if !mesh.gradients.is_empty() {
                if gpu.gradients == 0 {
                    gpu.gradients = self.new_texture();
                }
                (self.xp.bind_texture)(gpu.gradients, 0);
                let rows = mesh.gradients.concat();
                (gl.tex_image)(GL_TEXTURE_2D, 0, GL_RGBA8, RAMP as c_int, mesh.gradients.len() as c_int, 0, GL_RGBA, GL_UNSIGNED_BYTE, rows.as_ptr().cast());
                self.texture_parameters(GL_LINEAR, 1);
                check_upload_errors(gl, &mut self.gl_error_warned, "gradient ramp texture upload");
            }
            gpu.uploaded = stream;
        }
        (gl.pixel_store)(GL_UNPACK_ALIGNMENT, 4);
    }

    /// Draw `region` of a screen's mesh (device pixels, y down) into the
    /// viewport X-Plane gave the device, each dimming region at its
    /// `brightness`.
    #[allow(clippy::too_many_arguments)]
    pub fn draw(
        &mut self,
        gpu: &mut ScreenGpu,
        stream: u64,
        mesh: &Mesh,
        steps: &[Step],
        region: [i32; 4],
        target: Target,
        brightness: &[f32],
        natives: &[Option<Arc<NativeImage>>],
        res: &mut Resources,
    ) {
        self.draw_calls = 0;
        unsafe {
            let gl = &self.gl;
            let scissor_was_on = (gl.is_enabled)(GL_SCISSOR_TEST) != 0;
            (gl.use_program)(0);
            (gl.bind_vertex_array)(0);
            self.upload(gpu, stream, mesh, res, natives);
            let gl = &self.gl;
            (self.xp.set_graphics_state)(0, 0, 0, 0, 1, 0, 0);
            (gl.blend_func_separate)(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
            (gl.tex_env)(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
            for mode in [GL_PROJECTION, GL_MODELVIEW, GL_TEXTURE] {
                (gl.matrix_mode)(mode);
                (gl.push_matrix)();
                (gl.load_identity)();
            }
            (gl.bind_buffer)(GL_ARRAY_BUFFER, gpu.buffer);
            let stride = std::mem::size_of::<Vertex>() as c_int;
            (gl.enable_client_state)(GL_VERTEX_ARRAY);
            (gl.enable_client_state)(GL_COLOR_ARRAY);
            (gl.enable_client_state)(GL_TEXTURE_COORD_ARRAY);
            (gl.vertex_pointer)(2, GL_FLOAT, stride, std::ptr::null());
            (gl.tex_coord_pointer)(2, GL_FLOAT, stride, 8 as *const c_void);
            (gl.color_pointer)(4, GL_UNSIGNED_BYTE, stride, 16 as *const c_void);

            let [vx, vy, vw, vh] = target.viewport;
            let target_multisampled = *self.target_multisampled.get_or_insert_with(|| {
                let mut samples = 0;
                (gl.get_integer)(GL_SAMPLES, &mut samples);
                samples > 1
            });
            let [rx0, ry0, rx1, ry1] = region;
            let (sx, sy) = (vw as f64 / (rx1 - rx0).max(1) as f64, vh as f64 / (ry1 - ry0).max(1) as f64);
            match &self.multisample {
                Some(ms) if !target_multisampled && (sx - 1.).abs() < 1e-9 && (sy - 1.).abs() < 1e-9 => {
                    let framebuffer = ms.framebuffer;
                    (gl.bind_framebuffer)(GL_FRAMEBUFFER, framebuffer);
                    let mut ty = ry0;
                    while ty < ry1 {
                        let th = TILE.min(ry1 - ty);
                        let mut tx = rx0;
                        while tx < rx1 {
                            let tw = TILE.min(rx1 - tx);
                            let frame = Frame { region: [tx, ty, tx + tw, ty + th], origin: [0, 0], always_scissor: false, scale: (1., 1.) };
                            let gl = &self.gl;
                            (gl.bind_framebuffer)(GL_FRAMEBUFFER, framebuffer);
                            (gl.viewport)(0, 0, tw, th);
                            (gl.disable)(GL_SCISSOR_TEST);
                            (gl.clear_color)(0., 0., 0., 1.);
                            (gl.clear_stencil)(0);
                            (gl.stencil_mask)(0xff);
                            (gl.clear)(GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
                            self.run(gpu, steps, frame, true, brightness);
                            let gl = &self.gl;
                            (gl.bind_framebuffer)(GL_READ_FRAMEBUFFER, framebuffer);
                            (gl.bind_framebuffer)(GL_DRAW_FRAMEBUFFER, target.framebuffer);
                            (gl.disable)(GL_SCISSOR_TEST);
                            let (dx, dy) = (vx + (tx - rx0), vy + (ry1 - (ty + th)));
                            (gl.blit_framebuffer)(0, 0, tw, th, dx, dy, dx + tw, dy + th, GL_COLOR_BUFFER_BIT, GL_NEAREST);
                            tx += tw;
                        }
                        ty += th;
                    }
                    (self.gl.bind_framebuffer)(GL_FRAMEBUFFER, target.framebuffer);
                }
                _ => {
                    // Straight into X-Plane's target, scaled to its viewport.
                    let frame = Frame { region, origin: [vx, vy], always_scissor: true, scale: (sx, sy) };
                    (gl.viewport)(vx, vy, vw, vh);
                    (gl.enable)(GL_SCISSOR_TEST);
                    (gl.scissor)(vx, vy, vw, vh);
                    (gl.clear_color)(0., 0., 0., 1.);
                    (gl.clear)(GL_COLOR_BUFFER_BIT);
                    let mut bits = 0;
                    (gl.get_integer)(GL_STENCIL_BITS, &mut bits);
                    let stencil = bits >= 8;
                    if !stencil && !self.stencil_warned {
                        self.stencil_warned = true;
                        crate::log("display: the screen target has no stencil buffer; clip paths are not applied");
                    }
                    // The projection maps the region onto the whole viewport,
                    // so a texture of another size scales the drawing with it.
                    self.run(gpu, steps, frame, stencil, brightness);
                }
            }

            let gl = &self.gl;
            (gl.viewport)(vx, vy, vw, vh);
            if scissor_was_on {
                (gl.enable)(GL_SCISSOR_TEST);
            } else {
                (gl.disable)(GL_SCISSOR_TEST);
            }
            (gl.disable)(GL_STENCIL_TEST);
            (gl.stencil_mask)(0xff);
            (gl.color_mask)(1, 1, 1, 1);
            (gl.blend_func)(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            for mode in [GL_TEXTURE, GL_MODELVIEW, GL_PROJECTION] {
                (gl.matrix_mode)(mode);
                (gl.pop_matrix)();
            }
            (gl.matrix_mode)(GL_MODELVIEW);
            (gl.disable_client_state)(GL_COLOR_ARRAY);
            (gl.disable_client_state)(GL_TEXTURE_COORD_ARRAY);
            (gl.enable_client_state)(GL_VERTEX_ARRAY);
            (gl.bind_buffer)(GL_ARRAY_BUFFER, 0);
        }
    }

    /// Bring a bridge screen's texture up to date with XPHFBW's pixels
    /// (docs/briefs/xphfbw-js-bridge.md rule 6): `rects` are the regions to
    /// copy, already planned and clamped by [`super::xphfbw::plan_upload`].
    /// `pixels` is the screen's full BGRA buffer, top row first
    /// ([`crate::xphfbw_bridge::ScreenBlock::pixels`]); a size change (first
    /// use, or a resized screen) always uploads the whole thing regardless
    /// of `rects`.
    pub fn bridge_upload(&mut self, gpu: &mut BridgeGpu, width: u32, height: u32, pixels: &[u8], rects: &[[u32; 4]]) {
        if width == 0 || height == 0 || pixels.len() < (width as usize * height as usize * 4) {
            return;
        }
        let gl = &self.gl;
        unsafe {
            if gpu.texture == 0 {
                gpu.texture = self.new_texture();
            }
            (self.xp.bind_texture)(gpu.texture, 0);
            (gl.pixel_store)(GL_UNPACK_ALIGNMENT, 4);
            if gpu.size != (width, height) {
                crate::perf::UPLOADED_BYTES.fetch_add(width as u64 * height as u64 * 4, std::sync::atomic::Ordering::Relaxed);
                (gl.tex_image)(GL_TEXTURE_2D, 0, GL_RGBA8, width as c_int, height as c_int, 0, GL_BGRA, GL_UNSIGNED_BYTE, pixels.as_ptr().cast());
                self.texture_parameters(GL_LINEAR, 1);
                gpu.size = (width, height);
                return;
            }
            (gl.pixel_store)(GL_UNPACK_ROW_LENGTH, width as c_int);
            for &[x, y, w, h] in rects {
                if w == 0 || h == 0 || x + w > width || y + h > height {
                    continue;
                }
                let offset = (y as usize * width as usize + x as usize) * 4;
                crate::perf::UPLOADED_BYTES.fetch_add(w as u64 * h as u64 * 4, std::sync::atomic::Ordering::Relaxed);
                (gl.tex_sub_image)(GL_TEXTURE_2D, 0, x as c_int, y as c_int, w as c_int, h as c_int, GL_BGRA, GL_UNSIGNED_BYTE, pixels[offset..].as_ptr().cast());
            }
            (gl.pixel_store)(GL_UNPACK_ROW_LENGTH, 0);
        }
    }

    /// Upload a whole straight-alpha RGBA layer (the ND's terrain or weather
    /// radar picture) into `gpu` for [`Renderer::draw_bridge`]'s underlay.
    pub fn bridge_underlay_upload(&mut self, gpu: &mut BridgeGpu, width: u32, height: u32, rgba: &[u8]) {
        if width == 0 || height == 0 || rgba.len() < width as usize * height as usize * 4 {
            return;
        }
        unsafe {
            let fresh = gpu.texture == 0;
            if fresh {
                gpu.texture = self.new_texture();
            }
            (self.xp.bind_texture)(gpu.texture, 0);
            (self.gl.pixel_store)(GL_UNPACK_ALIGNMENT, 4);
            crate::perf::UPLOADED_BYTES.fetch_add(width as u64 * height as u64 * 4, std::sync::atomic::Ordering::Relaxed);
            if !fresh && gpu.size == (width, height) {
                // Same size as last time, which is the normal case: the ND's
                // terrain and weather pictures are a fixed 768x1024 canvas
                // and only their contents change. `glTexImage2D` would
                // reallocate the texture's storage on every update -- twice
                // a picture with both NDs drawn -- and drivers orphan the old
                // allocation rather than reuse it, which churns and fragments
                // video memory for a picture whose shape never changed.
                (self.gl.tex_sub_image)(
                    GL_TEXTURE_2D,
                    0,
                    0,
                    0,
                    width as c_int,
                    height as c_int,
                    GL_RGBA,
                    GL_UNSIGNED_BYTE,
                    rgba.as_ptr().cast(),
                );
            } else {
                (self.gl.tex_image)(GL_TEXTURE_2D, 0, GL_RGBA8, width as c_int, height as c_int, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba.as_ptr().cast());
                self.texture_parameters(GL_LINEAR, 1);
                gpu.size = (width, height);
            }
        }
    }

    /// Draw a bridge screen's texture as a full-screen quad into the
    /// viewport X-Plane gave the device, with the same per-region brightness
    /// dimming [`Renderer::draw`] applies to a tessellated mesh (screen
    /// pixel space, not device pixels: GL's viewport transform does the
    /// scaling).
    ///
    /// Layers, bottom to top, as MSFS stacks an ND's two gauges on one
    /// texture (panel.cfg: the terrain gauge, then nd.html): opaque black,
    /// `underlay` (straight alpha) when given, then Chromium's pixels, which
    /// CEF paints with premultiplied alpha.
    pub fn draw_bridge(&mut self, gpu: &mut BridgeGpu, underlay: Option<&BridgeGpu>, width: u32, height: u32, target: Target, brightness: &[f32], dimming: &[[f32; 4]]) {
        if gpu.texture == 0 || width == 0 || height == 0 {
            return;
        }
        unsafe {
            let gl = &self.gl;
            let scissor_was_on = (gl.is_enabled)(GL_SCISSOR_TEST) != 0;
            (gl.use_program)(0);
            (gl.bind_vertex_array)(0);
            (gl.bind_framebuffer)(GL_FRAMEBUFFER, target.framebuffer);
            let [vx, vy, vw, vh] = target.viewport;
            (gl.viewport)(vx, vy, vw, vh);
            (gl.disable)(GL_SCISSOR_TEST);
            (gl.blend_func)(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            (gl.tex_env)(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
            for mode in [GL_PROJECTION, GL_MODELVIEW, GL_TEXTURE] {
                (gl.matrix_mode)(mode);
                (gl.push_matrix)();
                (gl.load_identity)();
            }
            (gl.matrix_mode)(GL_PROJECTION);
            (gl.ortho)(0., width as f64, height as f64, 0., -1., 1.);
            (gl.matrix_mode)(GL_MODELVIEW);

            if gpu.buffer == 0 {
                (gl.gen_buffers)(1, &mut gpu.buffer);
            }
            let stride = std::mem::size_of::<Vertex>() as c_int;
            (gl.bind_buffer)(GL_ARRAY_BUFFER, gpu.buffer);
            (gl.enable_client_state)(GL_VERTEX_ARRAY);
            (gl.enable_client_state)(GL_COLOR_ARRAY);
            (gl.enable_client_state)(GL_TEXTURE_COORD_ARRAY);
            (gl.vertex_pointer)(2, GL_FLOAT, stride, std::ptr::null());
            (gl.tex_coord_pointer)(2, GL_FLOAT, stride, 8 as *const c_void);
            (gl.color_pointer)(4, GL_UNSIGNED_BYTE, stride, 16 as *const c_void);

            // Opaque black under everything: the screen's glass is black
            // wherever the pages paint nothing.
            (gl.disable_client_state)(GL_COLOR_ARRAY);
            let base = plain_quad(0., 0., width as f32, height as f32);
            (gl.buffer_data)(GL_ARRAY_BUFFER, std::mem::size_of_val(&base) as isize, base.as_ptr().cast(), GL_DYNAMIC_DRAW);
            (self.xp.set_graphics_state)(0, 0, 0, 0, 0, 0, 0);
            (gl.color4f)(0., 0., 0., 1.);
            (gl.draw_arrays)(GL_TRIANGLES, 0, 6);
            (gl.color4f)(1., 1., 1., 1.);
            (gl.enable_client_state)(GL_COLOR_ARRAY);

            let quad = textured_quad(width as f32, height as f32);
            (gl.buffer_data)(GL_ARRAY_BUFFER, std::mem::size_of_val(&quad) as isize, quad.as_ptr().cast(), GL_DYNAMIC_DRAW);
            self.draw_calls = 1;
            if let Some(under) = underlay.filter(|u| u.texture != 0) {
                (self.xp.set_graphics_state)(0, 1, 0, 0, 1, 0, 0);
                (gl.blend_func)(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                (self.xp.bind_texture)(under.texture, 0);
                (gl.draw_arrays)(GL_TRIANGLES, 0, 6);
                self.draw_calls += 1;
            }
            // CEF's off-screen pixels are premultiplied.
            (self.xp.set_graphics_state)(0, 1, 0, 0, 1, 0, 0);
            (gl.blend_func)(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
            (self.xp.bind_texture)(gpu.texture, 0);
            (gl.draw_arrays)(GL_TRIANGLES, 0, 6);
            (gl.blend_func)(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            self.draw_calls += 1;

            // The dimming regions on top, same darkening as Step::Dim.
            (gl.disable_client_state)(GL_COLOR_ARRAY);
            for (i, region) in dimming.iter().enumerate() {
                let b = brightness.get(i).copied().unwrap_or(1.).clamp(0., 1.);
                if b >= 1. {
                    continue;
                }
                let [x0, y0, x1, y1] = *region;
                let quad = plain_quad(x0, y0, x1, y1);
                (gl.buffer_data)(GL_ARRAY_BUFFER, std::mem::size_of_val(&quad) as isize, quad.as_ptr().cast(), GL_DYNAMIC_DRAW);
                (self.xp.set_graphics_state)(0, 0, 0, 0, 1, 0, 0);
                (gl.color4f)(0., 0., 0., 1. - screens_dim_srgb(b));
                (gl.draw_arrays)(GL_TRIANGLES, 0, 6);
                self.draw_calls += 1;
            }
            (gl.color4f)(1., 1., 1., 1.);
            (gl.enable_client_state)(GL_COLOR_ARRAY);

            (gl.viewport)(vx, vy, vw, vh);
            if scissor_was_on {
                (gl.enable)(GL_SCISSOR_TEST);
            } else {
                (gl.disable)(GL_SCISSOR_TEST);
            }
            (gl.bind_buffer)(GL_ARRAY_BUFFER, 0);
            for mode in [GL_TEXTURE, GL_MODELVIEW, GL_PROJECTION] {
                (gl.matrix_mode)(mode);
                (gl.pop_matrix)();
            }
            (gl.matrix_mode)(GL_MODELVIEW);
            (gl.disable_client_state)(GL_TEXTURE_COORD_ARRAY);
        }
    }

    /// Carry out the steps for one frame: a tile of the multisampled
    /// framebuffer, or the whole region straight in X-Plane's target.
    unsafe fn run(&mut self, gpu: &ScreenGpu, steps: &[Step], frame: Frame, stencil: bool, brightness: &[f32]) {
        let gl = &self.gl;
        let [rx0, ry0, rx1, ry1] = frame.region;
        (gl.matrix_mode)(GL_PROJECTION);
        (gl.load_identity)();
        (gl.ortho)(rx0 as f64, rx1 as f64, ry1 as f64, ry0 as f64, -1., 1.);
        (gl.matrix_mode)(GL_MODELVIEW);
        let mut current: Option<[i32; 4]> = None;
        let apply_scissor = |gl: &Fns, rect: Option<[i32; 4]>| {
            if rect.is_none() && !frame.always_scissor {
                (gl.disable)(GL_SCISSOR_TEST);
            } else {
                let [x, y, w, h] = frame.scissor_box(rect);
                (gl.enable)(GL_SCISSOR_TEST);
                (gl.scissor)(x, y, w, h);
            }
        };
        let mut textured: Option<Paint> = None;
        for step in steps {
            match *step {
                Step::Scissor(rect) => {
                    current = rect;
                    apply_scissor(gl, rect);
                }
                Step::ClearStencil if stencil => {
                    apply_scissor(gl, None);
                    (gl.stencil_mask)(0xff);
                    (gl.clear)(GL_STENCIL_BUFFER_BIT);
                    apply_scissor(gl, current);
                }
                Step::ClearMark if stencil => {
                    (gl.stencil_mask)(MARK as c_uint);
                    (gl.clear)(GL_STENCIL_BUFFER_BIT);
                }
                Step::ClipPath { level, first, count } if stencil => {
                    (self.xp.set_graphics_state)(0, 0, 0, 0, 1, 0, 0);
                    textured = None;
                    (gl.color_mask)(0, 0, 0, 0);
                    (gl.enable)(GL_STENCIL_TEST);
                    (gl.stencil_mask)(LEVELS as c_uint);
                    (gl.stencil_func)(GL_EQUAL, level as c_int, LEVELS as c_uint);
                    (gl.stencil_op)(GL_KEEP, GL_KEEP, GL_INCR);
                    (gl.draw_arrays)(GL_TRIANGLES, first as c_int, count as c_int);
                    (gl.color_mask)(1, 1, 1, 1);
                    self.draw_calls += 1;
                }
                Step::ClearStencil | Step::ClearMark | Step::ClipPath { .. } => {}
                Step::Dim { region, first, count } => {
                    let b = brightness.get(region).copied().unwrap_or(1.).clamp(0., 1.);
                    if b >= 1. {
                        continue;
                    }
                    (gl.disable)(GL_STENCIL_TEST);
                    (self.xp.set_graphics_state)(0, 0, 0, 0, 1, 0, 0);
                    textured = None;
                    (gl.disable_client_state)(GL_COLOR_ARRAY);
                    // MSFS scales the screen's emitted light by the knob;
                    // the texture holds sRGB, so the same fraction of light
                    // darkens its values by b^(1/2.2), not b.
                    (gl.color4f)(0., 0., 0., 1. - screens_dim_srgb(b));
                    (gl.draw_arrays)(GL_TRIANGLES, first as c_int, count as c_int);
                    (gl.color4f)(1., 1., 1., 1.);
                    (gl.enable_client_state)(GL_COLOR_ARRAY);
                    self.draw_calls += 1;
                }
                Step::Draw { paint, first, count, stencil: test } => {
                    match test {
                        StencilTest::Equal { reference, mask, mark } if stencil => {
                            (gl.enable)(GL_STENCIL_TEST);
                            (gl.stencil_func)(GL_EQUAL, reference as c_int, mask as c_uint);
                            if mark {
                                (gl.stencil_mask)(MARK as c_uint);
                                (gl.stencil_op)(GL_KEEP, GL_KEEP, GL_INVERT);
                            } else {
                                (gl.stencil_mask)(0);
                                (gl.stencil_op)(GL_KEEP, GL_KEEP, GL_KEEP);
                            }
                        }
                        _ => (gl.disable)(GL_STENCIL_TEST),
                    }
                    let native = match paint {
                        Paint::Native(i) => self.native_textures.get(i).copied().unwrap_or(0),
                        _ => 0,
                    };
                    if matches!(paint, Paint::Native(_)) && native == 0 {
                        continue;
                    }
                    if textured != Some(paint) {
                        match paint {
                            Paint::Atlas => {
                                (self.xp.set_graphics_state)(0, 1, 0, 0, 1, 0, 0);
                                (self.xp.bind_texture)(self.atlas_texture, 0);
                            }
                            Paint::Image(i) => {
                                (self.xp.set_graphics_state)(0, 1, 0, 0, 1, 0, 0);
                                (self.xp.bind_texture)(self.images.get(i).copied().unwrap_or(0), 0);
                            }
                            Paint::Gradients => {
                                (self.xp.set_graphics_state)(0, 1, 0, 0, 1, 0, 0);
                                (self.xp.bind_texture)(gpu.gradients, 0);
                            }
                            Paint::Native(_) => {
                                (self.xp.set_graphics_state)(0, 1, 0, 0, 1, 0, 0);
                                (self.xp.bind_texture)(native, 0);
                            }
                        }
                        (gl.matrix_mode)(GL_TEXTURE);
                        (gl.load_identity)();
                        if paint == Paint::Gradients {
                            (gl.scale)(1., 1. / gpu.gradient_rows.max(1) as f64, 1.);
                        }
                        (gl.matrix_mode)(GL_MODELVIEW);
                        textured = Some(paint);
                    }
                    (gl.draw_arrays)(GL_TRIANGLES, first as c_int, count as c_int);
                    self.draw_calls += 1;
                }
            }
        }
        (gl.disable)(GL_STENCIL_TEST);
    }
}

/// `xphfbw.displaySafeMode` out of a parsed xphfbw.json map: `true`/`"true"`
/// only (matching `app_settings.rs`'s `as_bool`, which this intentionally
/// duplicates rather than importing -- `app_settings.rs` is outside this
/// module's ownership, and `AppSettings` does not carry this key today).
/// Anything else (missing, malformed, a stray number) is "off", the
/// checkbox's own default (`app/ui/index.html`'s `DEFAULTS`).
fn parse_display_safe_mode(map: &serde_json::Map<String, serde_json::Value>) -> bool {
    match map.get("displaySafeMode") {
        Some(serde_json::Value::Bool(b)) => *b,
        Some(serde_json::Value::String(s)) => s == "true",
        _ => false,
    }
}

/// Reads `xphfbw.json` fresh (this runs once, in [`Renderer::new`], not per
/// frame): `false` with no X-Plane folder known yet (unit tests, or called
/// before `xp::system_path` can resolve) or no readable/parsable file, same
/// fallback `app_settings.rs::load_from_path` uses for a missing install.
fn display_safe_mode() -> bool {
    let Some(root) = crate::xp::system_path() else { return false };
    let path = crate::settings_files::app_settings_path(&root);
    let map = std::fs::read_to_string(&path)
        .ok()
        .and_then(|t| serde_json::from_str::<serde_json::Value>(&t).ok())
        .and_then(|v| v.as_object().cloned())
        .unwrap_or_default();
    parse_display_safe_mode(&map)
}

#[cfg(test)]
mod safe_mode_tests {
    use super::parse_display_safe_mode;
    use serde_json::{json, Value};

    #[test]
    fn off_by_default_when_the_key_is_missing() {
        let map = json!({}).as_object().cloned().unwrap();
        assert!(!parse_display_safe_mode(&map));
    }

    #[test]
    fn on_when_the_app_wrote_a_json_bool() {
        let map = json!({"displaySafeMode": true}).as_object().cloned().unwrap();
        assert!(parse_display_safe_mode(&map));
    }

    #[test]
    fn on_when_the_app_wrote_a_string_true_like_its_other_booleans() {
        let map = json!({"displaySafeMode": "true"}).as_object().cloned().unwrap();
        assert!(parse_display_safe_mode(&map));
    }

    #[test]
    fn off_for_anything_else() {
        for v in [Value::Bool(false), Value::String("false".into()), Value::String("nonsense".into()), Value::Null] {
            let map = json!({"displaySafeMode": v}).as_object().cloned().unwrap();
            assert!(!parse_display_safe_mode(&map));
        }
    }
}

/// The factor on sRGB-encoded values that scales their light by `b`.
pub fn screens_dim_srgb(b: f32) -> f32 {
    b.clamp(0., 1.).powf(1. / 2.2)
}

/// A full-screen quad (device pixels, y down, matching [`Vertex`]'s
/// convention), textured over its whole extent.
fn textured_quad(width: f32, height: f32) -> [Vertex; 6] {
    let rgba = [255, 255, 255, 255];
    let v = |x: f32, y: f32, u: f32, v: f32| Vertex { x, y, u, v, rgba };
    [v(0., 0., 0., 0.), v(width, 0., 1., 0.), v(width, height, 1., 1.), v(0., 0., 0., 0.), v(width, height, 1., 1.), v(0., height, 0., 1.)]
}

/// An untextured black quad over one region, for the dimming pass
/// ([`Renderer::draw_bridge`]); its alpha is set with `glColor4f` instead of
/// a per-vertex colour, as [`Step::Dim`] does for the tessellated path.
fn plain_quad(x0: f32, y0: f32, x1: f32, y1: f32) -> [Vertex; 6] {
    let rgba = [0, 0, 0, 255];
    let v = |x: f32, y: f32| Vertex { x, y, u: 0., v: 0., rgba };
    [v(x0, y0), v(x1, y0), v(x1, y1), v(x0, y0), v(x1, y1), v(x0, y1)]
}
