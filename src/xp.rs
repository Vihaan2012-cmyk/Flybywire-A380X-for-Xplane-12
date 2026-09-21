//! The part of X-Plane's plugin API this needs, looked up at run time.
//!
//! The SDK is not installed here, and on Windows a plugin does not need it:
//! `XPLM_64.dll` is already in the process by the time a plugin starts, so
//! its functions can be taken from it by name. That also keeps this plugin
//! free of the SDK's headers and import libraries.

use std::ffi::{c_char, c_int, c_uint, c_void, CString};

use crate::deep::integration::xp_consequences::{ForceSink, PlugForceAxis};
use crate::deep::weather::{WeatherCloudLayer, WeatherSample, WeatherSource};

/// An X-Plane dataref handle.
pub type DataRef = *mut c_void;

/// An X-Plane window and menu handle.
pub type WindowId = *mut c_void;
pub type MenuId = *mut c_void;

/// The window drawing callback: X-Plane calls it every frame while shown.
pub type DrawWindow = unsafe extern "C" fn(WindowId, *mut c_void);
pub type HandleMouse = unsafe extern "C" fn(WindowId, c_int, c_int, c_int, *mut c_void) -> c_int;
pub type HandleKey = unsafe extern "C" fn(WindowId, c_char, c_int, c_char, *mut c_void, c_int);
pub type HandleCursor = unsafe extern "C" fn(WindowId, c_int, c_int, *mut c_void) -> c_int;
pub type HandleWheel = unsafe extern "C" fn(WindowId, c_int, c_int, c_int, c_int, *mut c_void) -> c_int;
pub type MenuHandler = unsafe extern "C" fn(*mut c_void, *mut c_void);

/// X-Plane's own window chrome: the rounded dark frame with a close dot and a
/// pop-out button, as every study window in the sim wears. Three is the
/// self-decorated style, which draws nothing at all.
pub const DECORATION_ROUND_RECT: c_int = 1;
/// Free-floating inside the sim, and able to pop out to its own OS window.
pub const POSITION_FREE: c_int = 0;
/// Proportional font, the one X-Plane's own dialogs use.
pub const FONT_PROPORTIONAL: c_int = 18;
/// Fixed-width font: values line up in columns.
pub const FONT_BASIC: c_int = 0;
/// The first of X-Plane's mouse states: the button has just gone down.
pub const MOUSE_DOWN: c_int = 1;
/// The ordinary arrow, so a window full of clickable rows looks clickable.
pub const CURSOR_ARROW: c_int = 2;

#[repr(C)]
pub struct CreateWindow {
    pub struct_size: c_int,
    pub left: c_int,
    pub top: c_int,
    pub right: c_int,
    pub bottom: c_int,
    pub visible: c_int,
    pub draw: Option<DrawWindow>,
    pub mouse_click: Option<HandleMouse>,
    pub key: Option<HandleKey>,
    pub cursor: Option<HandleCursor>,
    pub wheel: Option<HandleWheel>,
    pub refcon: *mut c_void,
    pub decoration: c_int,
    pub layer: c_int,
    pub mouse_right: Option<HandleMouse>,
}

/// A callback X-Plane runs every frame.
pub type FlightLoop = unsafe extern "C" fn(f32, f32, c_int, *mut c_void) -> f32;

type GetI = Option<unsafe extern "C" fn(*mut c_void) -> c_int>;
type SetI = Option<unsafe extern "C" fn(*mut c_void, c_int)>;
type GetF = Option<unsafe extern "C" fn(*mut c_void) -> f32>;
type SetF = Option<unsafe extern "C" fn(*mut c_void, f32)>;
type GetD = Option<unsafe extern "C" fn(*mut c_void) -> f64>;
type SetD = Option<unsafe extern "C" fn(*mut c_void, f64)>;
type GetVi = Option<unsafe extern "C" fn(*mut c_void, *mut c_int, c_int, c_int) -> c_int>;
type SetVi = Option<unsafe extern "C" fn(*mut c_void, *mut c_int, c_int, c_int)>;
type GetVf = Option<unsafe extern "C" fn(*mut c_void, *mut f32, c_int, c_int) -> c_int>;
type SetVf = Option<unsafe extern "C" fn(*mut c_void, *mut f32, c_int, c_int)>;
type GetB = Option<unsafe extern "C" fn(*mut c_void, *mut c_void, c_int, c_int) -> c_int>;
type SetB = Option<unsafe extern "C" fn(*mut c_void, *mut c_void, c_int, c_int)>;

type CreateWindowExFn = unsafe extern "C" fn(*mut CreateWindow) -> WindowId;
type DestroyWindowFn = unsafe extern "C" fn(WindowId);
type SetWindowTitleFn = unsafe extern "C" fn(WindowId, *const c_char);
type SetWindowPositioningModeFn = unsafe extern "C" fn(WindowId, c_int, c_int);
type SetWindowResizingLimitsFn = unsafe extern "C" fn(WindowId, c_int, c_int, c_int, c_int);
type GetWindowGeometryFn = unsafe extern "C" fn(WindowId, *mut c_int, *mut c_int, *mut c_int, *mut c_int);
type SetWindowIsVisibleFn = unsafe extern "C" fn(WindowId, c_int);
type GetWindowIsVisibleFn = unsafe extern "C" fn(WindowId) -> c_int;
type BringWindowToFrontFn = unsafe extern "C" fn(WindowId);
type GetScreenBoundsGlobalFn = unsafe extern "C" fn(*mut c_int, *mut c_int, *mut c_int, *mut c_int);
type DrawStringFn = unsafe extern "C" fn(*mut f32, c_int, c_int, *const c_char, *mut c_int, c_int);
type MeasureStringFn = unsafe extern "C" fn(c_int, *const c_char, c_int) -> f32;
type GetFontDimensionsFn = unsafe extern "C" fn(c_int, *mut c_int, *mut c_int, *mut c_int);
type FindAircraftMenuFn = unsafe extern "C" fn() -> MenuId;
type FindPluginsMenuFn = unsafe extern "C" fn() -> MenuId;
type CreateMenuFn = unsafe extern "C" fn(*const c_char, MenuId, c_int, Option<MenuHandler>, *mut c_void) -> MenuId;
type AppendMenuItemFn = unsafe extern "C" fn(MenuId, *const c_char, *mut c_void, c_int) -> c_int;
type AppendMenuSeparatorFn = unsafe extern "C" fn(MenuId);
type DestroyMenuFn = unsafe extern "C" fn(MenuId);
type SetGraphicsStateFn = unsafe extern "C" fn(c_int, c_int, c_int, c_int, c_int, c_int, c_int);

// The drawing X-Plane itself does not offer: filled and outlined boxes, drawn
// straight through OpenGL the way every panel in the sim is drawn.
type GlBeginFn = unsafe extern "C" fn(c_uint);
type GlEndFn = unsafe extern "C" fn();
type GlColorFn = unsafe extern "C" fn(f32, f32, f32, f32);
type GlVertexFn = unsafe extern "C" fn(f32, f32);
type GlLineWidthFn = unsafe extern "C" fn(f32);

const GL_LINES: c_uint = 1;
const GL_LINE_LOOP: c_uint = 2;
const GL_LINE_STRIP: c_uint = 3;
const GL_TRIANGLE_FAN: c_uint = 6;
const GL_QUADS: c_uint = 7;

/// OpenGL's immediate mode, enough to paint a panel.
struct Gl {
    begin: GlBeginFn,
    end: GlEndFn,
    colour: GlColorFn,
    vertex: GlVertexFn,
    line_width: GlLineWidthFn,
}
type FindDataRefFn = unsafe extern "C" fn(*const c_char) -> DataRef;
type GetDatafFn = unsafe extern "C" fn(DataRef) -> f32;
type GetDatadFn = unsafe extern "C" fn(DataRef) -> f64;
type GetDataiFn = unsafe extern "C" fn(DataRef) -> c_int;
type GetDatavfFn = unsafe extern "C" fn(DataRef, *mut f32, c_int, c_int) -> c_int;
type GetDataviFn = unsafe extern "C" fn(DataRef, *mut c_int, c_int, c_int) -> c_int;
type SetDatafFn = unsafe extern "C" fn(DataRef, f32);
type SetDataiFn = unsafe extern "C" fn(DataRef, c_int);
type SetDatavfFn = unsafe extern "C" fn(DataRef, *mut f32, c_int, c_int);
type SetDataviFn = unsafe extern "C" fn(DataRef, *mut c_int, c_int, c_int);
type DebugStringFn = unsafe extern "C" fn(*const c_char);
type RegisterFlightLoopFn = unsafe extern "C" fn(FlightLoop, f32, *mut c_void);
type UnregisterFlightLoopFn = unsafe extern "C" fn(FlightLoop, *mut c_void);
#[allow(clippy::type_complexity)]
type RegisterDataAccessorFn = unsafe extern "C" fn(
    *const c_char,
    c_int,
    c_int,
    GetI,
    SetI,
    GetF,
    SetF,
    GetD,
    SetD,
    GetVi,
    SetVi,
    GetVf,
    SetVf,
    GetB,
    SetB,
    *mut c_void,
    *mut c_void,
) -> DataRef;

/// An int, float and double dataref in one (X-Plane's type flags 1, 2 and 4),
/// so Lua, SASL and other plugins can read a variable however they like.
const NUMBER_TYPES: c_int = 1 | 2 | 4;

/// Run every frame, after the flight model.
const EVERY_FRAME: f32 = -1.;

#[cfg(windows)]
#[link(name = "kernel32")]
extern "system" {
    fn LoadLibraryA(name: *const c_char) -> *mut c_void;
    fn GetProcAddress(module: *mut c_void, name: *const c_char) -> *mut c_void;
}

/// Not Windows -- the MSFS wasm build compiles this file. There is no
/// XPLM_64.dll (nor opengl32.dll) in that process to load, and these answer exactly as the real
/// calls do for a DLL that is not there: a null module, so every loader
/// above returns `None`. Nothing is stood in for.
#[cfg(not(windows))]
#[allow(non_snake_case)]
unsafe fn LoadLibraryA(_name: *const c_char) -> *mut c_void {
    std::ptr::null_mut()
}
#[cfg(not(windows))]
#[allow(non_snake_case)]
unsafe fn GetProcAddress(_module: *mut c_void, _name: *const c_char) -> *mut c_void {
    std::ptr::null_mut()
}

/// X-Plane's functions, bound to the copy of XPLM already in the process.
/// The whole of the API this binds is kept, including the few calls the
/// panels do not happen to need yet.
#[allow(dead_code)]
pub struct Xplm {
    find: FindDataRefFn,
    get_f: GetDatafFn,
    get_d: GetDatadFn,
    get_i: GetDataiFn,
    get_vf: GetDatavfFn,
    get_vi: GetDataviFn,
    set_f: SetDatafFn,
    set_i: SetDataiFn,
    set_vf: SetDatavfFn,
    set_vi: SetDataviFn,
    debug: DebugStringFn,
    register_accessor: RegisterDataAccessorFn,
    register_loop: RegisterFlightLoopFn,
    unregister_loop: UnregisterFlightLoopFn,
    create_window: CreateWindowExFn,
    destroy_window: DestroyWindowFn,
    set_window_title: SetWindowTitleFn,
    set_positioning: SetWindowPositioningModeFn,
    set_resizing_limits: SetWindowResizingLimitsFn,
    get_geometry: GetWindowGeometryFn,
    set_visible: SetWindowIsVisibleFn,
    get_visible: GetWindowIsVisibleFn,
    bring_to_front: BringWindowToFrontFn,
    screen_bounds: GetScreenBoundsGlobalFn,
    draw_string: DrawStringFn,
    measure_string: MeasureStringFn,
    font_dimensions: GetFontDimensionsFn,
    aircraft_menu: FindAircraftMenuFn,
    plugins_menu: FindPluginsMenuFn,
    create_menu: CreateMenuFn,
    append_item: AppendMenuItemFn,
    append_separator: AppendMenuSeparatorFn,
    destroy_menu: DestroyMenuFn,
    graphics_state: SetGraphicsStateFn,
    /// Missing only if this build of X-Plane has no OpenGL for plugins, in
    /// which case the panels still draw their text.
    gl: Option<Gl>,
}

/// OpenGL, already loaded in the process alongside X-Plane. A panel needs
/// nothing more than boxes and lines from it.
fn load_gl() -> Option<Gl> {
    unsafe {
        let name = CString::new("opengl32.dll").ok()?;
        let module = LoadLibraryA(name.as_ptr());
        if module.is_null() {
            return None;
        }
        let symbol = |s: &str| -> Option<*mut c_void> {
            let s = CString::new(s).ok()?;
            let p = GetProcAddress(module, s.as_ptr());
            (!p.is_null()).then_some(p)
        };
        Some(Gl {
            begin: std::mem::transmute::<*mut c_void, GlBeginFn>(symbol("glBegin")?),
            end: std::mem::transmute::<*mut c_void, GlEndFn>(symbol("glEnd")?),
            colour: std::mem::transmute::<*mut c_void, GlColorFn>(symbol("glColor4f")?),
            vertex: std::mem::transmute::<*mut c_void, GlVertexFn>(symbol("glVertex2f")?),
            line_width: std::mem::transmute::<*mut c_void, GlLineWidthFn>(symbol("glLineWidth")?),
        })
    }
}

impl Xplm {
    /// Bind to XPLM, or nothing if this is not running inside X-Plane.
    pub fn load() -> Option<Self> {
        unsafe {
            let name = CString::new("XPLM_64.dll").ok()?;
            let module = LoadLibraryA(name.as_ptr());
            if module.is_null() {
                return None;
            }
            let symbol = |s: &str| -> Option<*mut c_void> {
                let s = CString::new(s).ok()?;
                let p = GetProcAddress(module, s.as_ptr());
                (!p.is_null()).then_some(p)
            };
            Some(Self {
                find: std::mem::transmute::<*mut c_void, FindDataRefFn>(symbol("XPLMFindDataRef")?),
                get_f: std::mem::transmute::<*mut c_void, GetDatafFn>(symbol("XPLMGetDataf")?),
                get_d: std::mem::transmute::<*mut c_void, GetDatadFn>(symbol("XPLMGetDatad")?),
                get_i: std::mem::transmute::<*mut c_void, GetDataiFn>(symbol("XPLMGetDatai")?),
                get_vf: std::mem::transmute::<*mut c_void, GetDatavfFn>(symbol("XPLMGetDatavf")?),
                get_vi: std::mem::transmute::<*mut c_void, GetDataviFn>(symbol("XPLMGetDatavi")?),
                set_f: std::mem::transmute::<*mut c_void, SetDatafFn>(symbol("XPLMSetDataf")?),
                set_i: std::mem::transmute::<*mut c_void, SetDataiFn>(symbol("XPLMSetDatai")?),
                set_vf: std::mem::transmute::<*mut c_void, SetDatavfFn>(symbol("XPLMSetDatavf")?),
                set_vi: std::mem::transmute::<*mut c_void, SetDataviFn>(symbol("XPLMSetDatavi")?),
                debug: std::mem::transmute::<*mut c_void, DebugStringFn>(symbol("XPLMDebugString")?),
                register_accessor: std::mem::transmute::<*mut c_void, RegisterDataAccessorFn>(
                    symbol("XPLMRegisterDataAccessor")?,
                ),
                register_loop: std::mem::transmute::<*mut c_void, RegisterFlightLoopFn>(
                    symbol("XPLMRegisterFlightLoopCallback")?,
                ),
                unregister_loop: std::mem::transmute::<*mut c_void, UnregisterFlightLoopFn>(
                    symbol("XPLMUnregisterFlightLoopCallback")?,
                ),
                create_window: std::mem::transmute::<*mut c_void, CreateWindowExFn>(symbol("XPLMCreateWindowEx")?),
                destroy_window: std::mem::transmute::<*mut c_void, DestroyWindowFn>(symbol("XPLMDestroyWindow")?),
                set_window_title: std::mem::transmute::<*mut c_void, SetWindowTitleFn>(symbol("XPLMSetWindowTitle")?),
                set_positioning: std::mem::transmute::<*mut c_void, SetWindowPositioningModeFn>(
                    symbol("XPLMSetWindowPositioningMode")?,
                ),
                set_resizing_limits: std::mem::transmute::<*mut c_void, SetWindowResizingLimitsFn>(
                    symbol("XPLMSetWindowResizingLimits")?,
                ),
                get_geometry: std::mem::transmute::<*mut c_void, GetWindowGeometryFn>(symbol("XPLMGetWindowGeometry")?),
                set_visible: std::mem::transmute::<*mut c_void, SetWindowIsVisibleFn>(symbol("XPLMSetWindowIsVisible")?),
                get_visible: std::mem::transmute::<*mut c_void, GetWindowIsVisibleFn>(symbol("XPLMGetWindowIsVisible")?),
                bring_to_front: std::mem::transmute::<*mut c_void, BringWindowToFrontFn>(symbol("XPLMBringWindowToFront")?),
                screen_bounds: std::mem::transmute::<*mut c_void, GetScreenBoundsGlobalFn>(
                    symbol("XPLMGetScreenBoundsGlobal")?,
                ),
                draw_string: std::mem::transmute::<*mut c_void, DrawStringFn>(symbol("XPLMDrawString")?),
                measure_string: std::mem::transmute::<*mut c_void, MeasureStringFn>(symbol("XPLMMeasureString")?),
                font_dimensions: std::mem::transmute::<*mut c_void, GetFontDimensionsFn>(symbol("XPLMGetFontDimensions")?),
                aircraft_menu: std::mem::transmute::<*mut c_void, FindAircraftMenuFn>(symbol("XPLMFindAircraftMenu")?),
                plugins_menu: std::mem::transmute::<*mut c_void, FindPluginsMenuFn>(symbol("XPLMFindPluginsMenu")?),
                create_menu: std::mem::transmute::<*mut c_void, CreateMenuFn>(symbol("XPLMCreateMenu")?),
                append_item: std::mem::transmute::<*mut c_void, AppendMenuItemFn>(symbol("XPLMAppendMenuItem")?),
                append_separator: std::mem::transmute::<*mut c_void, AppendMenuSeparatorFn>(
                    symbol("XPLMAppendMenuSeparator")?,
                ),
                destroy_menu: std::mem::transmute::<*mut c_void, DestroyMenuFn>(symbol("XPLMDestroyMenu")?),
                graphics_state: std::mem::transmute::<*mut c_void, SetGraphicsStateFn>(
                    symbol("XPLMSetGraphicsState")?,
                ),
                gl: load_gl(),
            })
        }
    }

    /// X-Plane's own dataref of this name, if it has one.
    pub fn find(&self, name: &str) -> Option<DataRef> {
        let name = CString::new(name).ok()?;
        let dataref = unsafe { (self.find)(name.as_ptr()) };
        (!dataref.is_null()).then_some(dataref)
    }

    pub fn get_f(&self, dataref: DataRef) -> f32 {
        unsafe { (self.get_f)(dataref) }
    }

    pub fn get_d(&self, dataref: DataRef) -> f64 {
        unsafe { (self.get_d)(dataref) }
    }

    pub fn get_i(&self, dataref: DataRef) -> c_int {
        unsafe { (self.get_i)(dataref) }
    }

    /// Fill `out` from the start of a float array dataref. Returns how many
    /// values X-Plane wrote.
    pub fn get_vf(&self, dataref: DataRef, out: &mut [f32]) -> usize {
        let n = unsafe { (self.get_vf)(dataref, out.as_mut_ptr(), 0, out.len() as c_int) };
        n.max(0) as usize
    }

    pub fn set_f(&self, dataref: DataRef, value: f32) {
        unsafe { (self.set_f)(dataref, value) }
    }

    pub fn set_i(&self, dataref: DataRef, value: c_int) {
        unsafe { (self.set_i)(dataref, value) }
    }

    /// Write one element of a float array dataref.
    pub fn set_vf_at(&self, dataref: DataRef, index: usize, value: f32) {
        let mut v = value;
        unsafe { (self.set_vf)(dataref, &mut v, index as c_int, 1) }
    }

    /// Write one element of an int array dataref.
    pub fn set_vi_at(&self, dataref: DataRef, index: usize, value: c_int) {
        let mut v = value;
        unsafe { (self.set_vi)(dataref, &mut v, index as c_int, 1) }
    }

    /// Write the start of a float array dataref.
    pub fn set_vf(&self, dataref: DataRef, values: &[f32]) {
        let mut copy = values.to_vec();
        unsafe { (self.set_vf)(dataref, copy.as_mut_ptr(), 0, copy.len() as c_int) }
    }

    /// Fill `out` from the start of an int array dataref.
    pub fn get_vi(&self, dataref: DataRef, out: &mut [c_int]) -> usize {
        let n = unsafe { (self.get_vi)(dataref, out.as_mut_ptr(), 0, out.len() as c_int) };
        n.max(0) as usize
    }

    /// Publish a writable dataref reading and writing the slot in `refcon`.
    /// X-Plane keeps the name, so the caller must keep the string alive.
    pub fn publish(&self, name: &CString, refcon: *mut c_void) -> DataRef {
        unsafe {
            (self.register_accessor)(
                name.as_ptr(),
                NUMBER_TYPES,
                1,
                Some(crate::get_datai),
                Some(crate::set_datai),
                Some(crate::get_dataf),
                Some(crate::set_dataf),
                Some(crate::get_datad),
                Some(crate::set_datad),
                None,
                None,
                None,
                None,
                None,
                None,
                refcon,
                refcon,
            )
        }
    }

    pub fn register_loop(&self, callback: FlightLoop) {
        unsafe { (self.register_loop)(callback, EVERY_FRAME, std::ptr::null_mut()) }
    }

    pub fn unregister_loop(&self, callback: FlightLoop) {
        unsafe { (self.unregister_loop)(callback, std::ptr::null_mut()) }
    }

    /// A window wearing X-Plane's own chrome, placed near the middle of the
    /// screen, free to be dragged or popped out to its own OS window.
    #[allow(clippy::too_many_arguments)]
    pub fn window(
        &self,
        width: c_int,
        height: c_int,
        draw: DrawWindow,
        mouse: HandleMouse,
        wheel: HandleWheel,
        cursor: HandleCursor,
        refcon: *mut c_void,
    ) -> WindowId {
        let (mut l, mut t, mut r, mut b) = (0, 0, 0, 0);
        unsafe { (self.screen_bounds)(&mut l, &mut t, &mut r, &mut b) };
        // Centred, and never taller than the screen it opens on, so a long
        // page does not start below the bottom edge.
        let height = height.min((t - b) - 80).max(200);
        let width = width.min((r - l) - 80).max(380);
        let left = l + ((r - l) - width).max(0) / 2;
        let top = b + (t - b + height) / 2;
        let mut spec = CreateWindow {
            struct_size: std::mem::size_of::<CreateWindow>() as c_int,
            left,
            top,
            right: left + width,
            bottom: top - height,
            visible: 1,
            draw: Some(draw),
            mouse_click: Some(mouse),
            key: None,
            cursor: Some(cursor),
            wheel: Some(wheel),
            refcon,
            decoration: DECORATION_ROUND_RECT,
            layer: 0,
            mouse_right: None,
        };
        let id = unsafe { (self.create_window)(&mut spec) };
        unsafe {
            (self.set_positioning)(id, POSITION_FREE, -1);
            (self.set_resizing_limits)(id, 920, 640, 3200, 2000);
        }
        id
    }

    pub fn set_title(&self, window: WindowId, title: &str) {
        if let Ok(t) = CString::new(title) {
            unsafe { (self.set_window_title)(window, t.as_ptr()) }
        }
    }

    /// `(left, top, right, bottom)` in global screen coordinates.
    pub fn geometry(&self, window: WindowId) -> (c_int, c_int, c_int, c_int) {
        let (mut l, mut t, mut r, mut b) = (0, 0, 0, 0);
        unsafe { (self.get_geometry)(window, &mut l, &mut t, &mut r, &mut b) };
        (l, t, r, b)
    }

    #[allow(dead_code)]
    pub fn is_visible(&self, window: WindowId) -> bool {
        unsafe { (self.get_visible)(window) != 0 }
    }

    pub fn show(&self, window: WindowId, visible: bool) {
        unsafe { (self.set_visible)(window, visible as c_int) };
        if visible {
            unsafe { (self.bring_to_front)(window) };
        }
    }

    pub fn destroy_window(&self, window: WindowId) {
        unsafe { (self.destroy_window)(window) }
    }

    /// Draw one line of text. `colour` is red, green, blue in 0..1.
    pub fn text(&self, x: c_int, y: c_int, colour: [f32; 3], font: c_int, s: &str) {
        let Ok(c) = CString::new(s) else { return };
        let mut rgb = colour;
        unsafe { (self.draw_string)(rgb.as_mut_ptr(), x, y, c.as_ptr(), std::ptr::null_mut(), font) }
    }

    #[allow(dead_code)]
    pub fn text_width(&self, font: c_int, s: &str) -> c_int {
        let Ok(c) = CString::new(s) else { return 0 };
        unsafe { (self.measure_string)(font, c.as_ptr(), s.len() as c_int) as c_int }
    }

    /// `(character width, line height)` for a font.
    pub fn font_size(&self, font: c_int) -> (c_int, c_int) {
        let (mut w, mut h) = (0, 0);
        unsafe { (self.font_dimensions)(font, &mut w, &mut h, std::ptr::null_mut()) };
        (w.max(1), h.max(10))
    }

    /// The aircraft's own menu, where X-Plane puts an aircraft's plugins.
    pub fn menu(&self, title: &str, handler: MenuHandler) -> MenuId {
        let Ok(t) = CString::new(title) else { return std::ptr::null_mut() };
        unsafe {
            let parent = (self.aircraft_menu)();
            let parent = if parent.is_null() { (self.plugins_menu)() } else { parent };
            let slot = (self.append_item)(parent, t.as_ptr(), std::ptr::null_mut(), 0);
            (self.create_menu)(t.as_ptr(), parent, slot, Some(handler), std::ptr::null_mut())
        }
    }

    pub fn submenu(&self, parent: MenuId, title: &str, handler: MenuHandler) -> MenuId {
        let Ok(t) = CString::new(title) else { return std::ptr::null_mut() };
        unsafe {
            let slot = (self.append_item)(parent, t.as_ptr(), std::ptr::null_mut(), 0);
            (self.create_menu)(t.as_ptr(), parent, slot, Some(handler), std::ptr::null_mut())
        }
    }

    pub fn menu_item(&self, menu: MenuId, title: &str, refcon: *mut c_void) {
        if let Ok(t) = CString::new(title) {
            unsafe { (self.append_item)(menu, t.as_ptr(), refcon, 0) };
        }
    }

    pub fn menu_separator(&self, menu: MenuId) {
        unsafe { (self.append_separator)(menu) }
    }

    pub fn destroy_menu(&self, menu: MenuId) {
        if !menu.is_null() {
            unsafe { (self.destroy_menu)(menu) }
        }
    }

    /// A filled box, in the window coordinates the draw callback works in.
    pub fn fill(&self, left: c_int, top: c_int, right: c_int, bottom: c_int, colour: [f32; 4]) {
        let Some(gl) = &self.gl else { return };
        unsafe {
            // No texture, no lighting, no depth: blending only, as X-Plane
            // asks of anything drawing flat over its window.
            (self.graphics_state)(0, 0, 0, 0, 1, 0, 0);
            (gl.colour)(colour[0], colour[1], colour[2], colour[3]);
            (gl.begin)(GL_QUADS);
            (gl.vertex)(left as f32, bottom as f32);
            (gl.vertex)(left as f32, top as f32);
            (gl.vertex)(right as f32, top as f32);
            (gl.vertex)(right as f32, bottom as f32);
            (gl.end)();
        }
    }

    /// A box's outline, half a pixel in so the line lands on the edge itself.
    pub fn frame(&self, left: c_int, top: c_int, right: c_int, bottom: c_int, colour: [f32; 4], width: f32) {
        let Some(gl) = &self.gl else { return };
        unsafe {
            (self.graphics_state)(0, 0, 0, 0, 1, 0, 0);
            (gl.line_width)(width);
            (gl.colour)(colour[0], colour[1], colour[2], colour[3]);
            (gl.begin)(GL_LINE_LOOP);
            (gl.vertex)(left as f32 + 0.5, bottom as f32 + 0.5);
            (gl.vertex)(left as f32 + 0.5, top as f32 - 0.5);
            (gl.vertex)(right as f32 - 0.5, top as f32 - 0.5);
            (gl.vertex)(right as f32 - 0.5, bottom as f32 + 0.5);
            (gl.end)();
            (gl.line_width)(1.);
        }
    }

    /// A straight line, for the connections a synoptic page draws between
    /// what feeds what.
    pub fn line(&self, x1: c_int, y1: c_int, x2: c_int, y2: c_int, colour: [f32; 4], width: f32) {
        let Some(gl) = &self.gl else { return };
        unsafe {
            (self.graphics_state)(0, 0, 0, 0, 1, 0, 0);
            (gl.line_width)(width);
            (gl.colour)(colour[0], colour[1], colour[2], colour[3]);
            (gl.begin)(GL_LINES);
            (gl.vertex)(x1 as f32 + 0.5, y1 as f32 + 0.5);
            (gl.vertex)(x2 as f32 + 0.5, y2 as f32 + 0.5);
            (gl.end)();
            (gl.line_width)(1.);
        }
    }

    /// A filled convex shape: a nacelle, a spinner, an arrow head.
    pub fn poly(&self, points: &[(f32, f32)], colour: [f32; 4]) {
        let Some(gl) = &self.gl else { return };
        if points.len() < 3 {
            return;
        }
        unsafe {
            (self.graphics_state)(0, 0, 0, 0, 1, 0, 0);
            (gl.colour)(colour[0], colour[1], colour[2], colour[3]);
            (gl.begin)(GL_TRIANGLE_FAN);
            for &(x, y) in points {
                (gl.vertex)(x, y);
            }
            (gl.end)();
        }
    }

    /// Connected line segments, open or closed: an outline, a plot.
    pub fn strip(&self, points: &[(f32, f32)], colour: [f32; 4], width: f32, closed: bool) {
        let Some(gl) = &self.gl else { return };
        if points.len() < 2 {
            return;
        }
        unsafe {
            (self.graphics_state)(0, 0, 0, 0, 1, 0, 0);
            (gl.line_width)(width);
            (gl.colour)(colour[0], colour[1], colour[2], colour[3]);
            (gl.begin)(if closed { GL_LINE_LOOP } else { GL_LINE_STRIP });
            for &(x, y) in points {
                (gl.vertex)(x + 0.5, y + 0.5);
            }
            (gl.end)();
            (gl.line_width)(1.);
        }
    }

    /// Write a line to X-Plane's Log.txt.
    pub fn log(&self, message: &str) {
        if let Ok(message) = CString::new(message) {
            unsafe { (self.debug)(message.as_ptr()) }
        }
    }

    // ------------------------------------------------------------------
    // Commands (added for the FCU / autothrust events, src/afs_events.rs).
    // Looked up on use rather than in `load`, so nothing above changes.
    // ------------------------------------------------------------------

    fn xplm_symbol(name: &str) -> Option<*mut c_void> {
        unsafe {
            let module_name = CString::new("XPLM_64.dll").ok()?;
            let module = LoadLibraryA(module_name.as_ptr());
            if module.is_null() {
                return None;
            }
            let name = CString::new(name).ok()?;
            let p = GetProcAddress(module, name.as_ptr());
            (!p.is_null()).then_some(p)
        }
    }

    /// X-Plane's command of this name, created if it does not exist yet.
    pub fn create_command(&self, name: &str, description: &str) -> Option<CommandRef> {
        if name.starts_with("sim/") {
            return find_command(name);
        }
        let create = Self::xplm_symbol("XPLMCreateCommand")?;
        let create = unsafe { std::mem::transmute::<*mut c_void, CreateCommandFn>(create) };
        let name = CString::new(name).ok()?;
        let description = CString::new(description).ok()?;
        let command = unsafe { create(name.as_ptr(), description.as_ptr()) };
        (!command.is_null()).then_some(command)
    }

    pub fn register_command_handler(&self, command: CommandRef, handler: CommandHandler, refcon: *mut c_void) {
        if let Some(f) = Self::xplm_symbol("XPLMRegisterCommandHandler") {
            let f = unsafe { std::mem::transmute::<*mut c_void, CommandHandlerFn>(f) };
            unsafe { f(command, handler, 1, refcon) }
        }
    }

    pub fn unregister_command_handler(&self, command: CommandRef, handler: CommandHandler, refcon: *mut c_void) {
        if let Some(f) = Self::xplm_symbol("XPLMUnregisterCommandHandler") {
            let f = unsafe { std::mem::transmute::<*mut c_void, CommandHandlerFn>(f) };
            unsafe { f(command, handler, 1, refcon) }
        }
    }
}

/// An X-Plane command handle.
pub type CommandRef = *mut c_void;
/// A command handler: command, phase (0 begin, 1 continue, 2 end), refcon.
/// Returns 1 to let X-Plane and other handlers see the command too.
pub type CommandHandler = unsafe extern "C" fn(CommandRef, c_int, *mut c_void) -> c_int;
type CreateCommandFn = unsafe extern "C" fn(*const c_char, *const c_char) -> CommandRef;
type CommandHandlerFn = unsafe extern "C" fn(CommandRef, CommandHandler, c_int, *mut c_void);

// ------------------------------------------------------------------
// Owned datarefs, byte arrays and commands without an `Xplm` at hand
// (added for published.rs). Looked up on use; outside X-Plane they do
// nothing and return nothing.
// ------------------------------------------------------------------

pub type AccessorGetI = unsafe extern "C" fn(*mut c_void) -> c_int;
pub type AccessorSetI = unsafe extern "C" fn(*mut c_void, c_int);
pub type AccessorGetF = unsafe extern "C" fn(*mut c_void) -> f32;
pub type AccessorSetF = unsafe extern "C" fn(*mut c_void, f32);
pub type AccessorGetD = unsafe extern "C" fn(*mut c_void) -> f64;
pub type AccessorSetD = unsafe extern "C" fn(*mut c_void, f64);
pub type AccessorGetB = unsafe extern "C" fn(*mut c_void, *mut c_void, c_int, c_int) -> c_int;
pub type AccessorSetB = unsafe extern "C" fn(*mut c_void, *mut c_void, c_int, c_int);

/// The callbacks of one accessor; `None` for the types it does not serve.
#[derive(Default)]
pub struct Accessors {
    pub get_i: Option<AccessorGetI>,
    pub set_i: Option<AccessorSetI>,
    pub get_f: Option<AccessorGetF>,
    pub set_f: Option<AccessorSetF>,
    pub get_d: Option<AccessorGetD>,
    pub set_d: Option<AccessorSetD>,
    pub get_b: Option<AccessorGetB>,
    pub set_b: Option<AccessorSetB>,
}

/// XPLMDataTypeID's int, float, double and data (byte array) bits.
pub const DATA_TYPE_INT: c_int = 1;
pub const DATA_TYPE_FLOAT: c_int = 2;
pub const DATA_TYPE_DOUBLE: c_int = 4;
pub const DATA_TYPE_DATA: c_int = 32;

#[allow(clippy::type_complexity)]
type RegisterAnyAccessorFn = unsafe extern "C" fn(
    *const c_char,
    c_int,
    c_int,
    Option<AccessorGetI>,
    Option<AccessorSetI>,
    Option<AccessorGetF>,
    Option<AccessorSetF>,
    Option<AccessorGetD>,
    Option<AccessorSetD>,
    GetVi,
    SetVi,
    GetVf,
    SetVf,
    Option<AccessorGetB>,
    Option<AccessorSetB>,
    *mut c_void,
    *mut c_void,
) -> DataRef;
type UnregisterAccessorFn = unsafe extern "C" fn(DataRef);
type GetDatabFn = unsafe extern "C" fn(DataRef, *mut c_void, c_int, c_int) -> c_int;

/// Register a dataref of the given types. X-Plane keeps the name, so the
/// caller keeps the string alive.
pub fn register_accessor(name: &CString, types: c_int, writable: bool, a: &Accessors, refcon: *mut c_void) -> Option<DataRef> {
    let f = Xplm::xplm_symbol("XPLMRegisterDataAccessor")?;
    let f = unsafe { std::mem::transmute::<*mut c_void, RegisterAnyAccessorFn>(f) };
    let d = unsafe {
        f(
            name.as_ptr(),
            types,
            writable as c_int,
            a.get_i,
            a.set_i,
            a.get_f,
            a.set_f,
            a.get_d,
            a.set_d,
            None,
            None,
            None,
            None,
            a.get_b,
            a.set_b,
            refcon,
            refcon,
        )
    };
    (!d.is_null()).then_some(d)
}

pub fn unregister_accessor(dataref: DataRef) {
    if let Some(f) = Xplm::xplm_symbol("XPLMUnregisterDataAccessor") {
        let f = unsafe { std::mem::transmute::<*mut c_void, UnregisterAccessorFn>(f) };
        unsafe { f(dataref) }
    }
}

/// X-Plane's command of this name, created if it does not exist yet.
pub fn command_by_name(name: &str, description: &str) -> Option<CommandRef> {
    if name.starts_with("sim/") {
        return find_command(name);
    }
    let create = Xplm::xplm_symbol("XPLMCreateCommand")?;
    let create = unsafe { std::mem::transmute::<*mut c_void, CreateCommandFn>(create) };
    let name = CString::new(name).ok()?;
    let description = CString::new(description).ok()?;
    let command = unsafe { create(name.as_ptr(), description.as_ptr()) };
    (!command.is_null()).then_some(command)
}

/// Handle a command before X-Plane does (`before`) or after it.
pub fn add_command_handler(command: CommandRef, handler: CommandHandler, before: bool, refcon: *mut c_void) {
    if let Some(f) = Xplm::xplm_symbol("XPLMRegisterCommandHandler") {
        let f = unsafe { std::mem::transmute::<*mut c_void, CommandHandlerFn>(f) };
        unsafe { f(command, handler, before as c_int, refcon) }
    }
}

pub fn remove_command_handler(command: CommandRef, handler: CommandHandler, before: bool, refcon: *mut c_void) {
    if let Some(f) = Xplm::xplm_symbol("XPLMUnregisterCommandHandler") {
        let f = unsafe { std::mem::transmute::<*mut c_void, CommandHandlerFn>(f) };
        unsafe { f(command, handler, before as c_int, refcon) }
    }
}

impl Xplm {
    /// Fill `out` from the start of a byte array dataref (a string). Returns
    /// how many bytes X-Plane wrote.
    pub fn get_vb(&self, dataref: DataRef, out: &mut [u8]) -> usize {
        let Some(f) = Self::xplm_symbol("XPLMGetDatab") else { return 0 };
        let f = unsafe { std::mem::transmute::<*mut c_void, GetDatabFn>(f) };
        let n = unsafe { f(dataref, out.as_mut_ptr() as *mut c_void, 0, out.len() as c_int) };
        n.max(0) as usize
    }

    /// A byte array dataref read as text, up to its first zero.
    pub fn get_text(&self, dataref: DataRef, max: usize) -> String {
        let mut bytes = vec![0u8; max];
        let n = self.get_vb(dataref, &mut bytes).min(max);
        let end = bytes[..n].iter().position(|&b| b == 0).unwrap_or(n);
        String::from_utf8_lossy(&bytes[..end]).trim().to_owned()
    }
}

// ----------------------------------------------------------------------
// Cockpit devices and texture handling (added for the screens, src/display).
// Layouts from the X-Plane SDK 4.3.0 headers (XPLMDisplay.h, XPLMGraphics.h);
// the avionics device API needs XPLM410, X-Plane 12.1 or later.
// ----------------------------------------------------------------------

/// An X-Plane cockpit device handle.
pub type AvionicsId = *mut c_void;
pub type AvionicsScreen = unsafe extern "C" fn(*mut c_void);
pub type AvionicsBezel = unsafe extern "C" fn(f32, f32, f32, *mut c_void);
pub type AvionicsMouse = unsafe extern "C" fn(c_int, c_int, c_int, *mut c_void) -> c_int;
pub type AvionicsWheel = unsafe extern "C" fn(c_int, c_int, c_int, c_int, *mut c_void) -> c_int;
pub type AvionicsCursor = unsafe extern "C" fn(c_int, c_int, *mut c_void) -> c_int;
pub type AvionicsKey = unsafe extern "C" fn(c_char, c_int, c_char, *mut c_void, c_int) -> c_int;
pub type AvionicsBrightness = unsafe extern "C" fn(f32, f32, f32, *mut c_void) -> f32;

/// XPLMCreateAvionics_t (XPLMDisplay.h, XPLM410).
#[repr(C)]
pub struct CreateAvionics {
    pub struct_size: c_int,
    pub screen_width: c_int,
    pub screen_height: c_int,
    pub bezel_width: c_int,
    pub bezel_height: c_int,
    pub screen_offset_x: c_int,
    pub screen_offset_y: c_int,
    pub draw_on_demand: c_int,
    pub bezel_draw: Option<AvionicsBezel>,
    pub draw: Option<AvionicsScreen>,
    pub bezel_click: Option<AvionicsMouse>,
    pub bezel_right_click: Option<AvionicsMouse>,
    pub bezel_scroll: Option<AvionicsWheel>,
    pub bezel_cursor: Option<AvionicsCursor>,
    pub screen_touch: Option<AvionicsMouse>,
    pub screen_right_touch: Option<AvionicsMouse>,
    pub screen_scroll: Option<AvionicsWheel>,
    pub screen_cursor: Option<AvionicsCursor>,
    pub keyboard: Option<AvionicsKey>,
    pub brightness: Option<AvionicsBrightness>,
    pub device_id: *const c_char,
    pub device_name: *const c_char,
    pub refcon: *mut c_void,
}

/// The mouse states avionics callbacks get (XPLMDefs.h).
pub const MOUSE_DRAG: c_int = 2;
pub const MOUSE_UP: c_int = 3;
/// Let X-Plane choose the cursor (XPLMDefs.h).
pub const CURSOR_DEFAULT: c_int = 0;

/// The cockpit device calls and the texture calls drawing with them needs.
#[derive(Clone, Copy)]
pub struct AvionicsApi {
    pub create: unsafe extern "C" fn(*mut CreateAvionics) -> AvionicsId,
    pub destroy: unsafe extern "C" fn(AvionicsId),
    pub is_bound: unsafe extern "C" fn(AvionicsId) -> c_int,
    pub set_graphics_state: SetGraphicsStateFn,
    pub bind_texture: unsafe extern "C" fn(c_int, c_int),
    pub generate_textures: unsafe extern "C" fn(*mut c_int, c_int),
    /// XPLMSetAvionicsPopupVisible: shows or hides a device's floating 2D
    /// popup window. The only way (XPLM410) to give a device keyboard focus
    /// is through this popup — see `take_keyboard_focus`'s doc comment.
    pub set_popup_visible: unsafe extern "C" fn(AvionicsId, c_int),
    /// XPLMTakeAvionicsKeyboardFocus: gives the device's popup window
    /// keyboard focus, so its `CreateAvionics::keyboard` callback starts
    /// receiving keystrokes. XPLMDisplay.h is explicit that
    /// `XPLMAvionicsKeyboard_f` only fires once this has been called on a
    /// device whose popup is visible (`set_popup_visible` first) — there is
    /// no way to capture the keyboard for a device embedded in the 3D
    /// cockpit alone; the popup is required.
    pub take_keyboard_focus: unsafe extern "C" fn(AvionicsId),
    /// XPLMHasAvionicsKeyboardFocus: whether the device's popup currently
    /// holds keyboard focus.
    pub has_keyboard_focus: unsafe extern "C" fn(AvionicsId) -> c_int,
}

impl AvionicsApi {
    /// Gives a device's keyboard focus back to X-Plane. XPLMDisplay.h has no
    /// `XPLMReleaseAvionicsKeyboardFocus`/opposite of `take_keyboard_focus`;
    /// hiding the popup is what does it, since
    /// `XPLMAvionicsKeyboard_f` only fires for a visible, focused popup in
    /// the first place (see `take_keyboard_focus`'s doc comment) — so this
    /// is `set_popup_visible(handle, 0)`, named for what callers want rather
    /// than the one SDK call it happens to be.
    pub unsafe fn release_keyboard_focus(&self, handle: AvionicsId) {
        (self.set_popup_visible)(handle, 0);
    }
}

impl Xplm {
    /// The cockpit device API, when this X-Plane has it.
    pub fn avionics(&self) -> Option<AvionicsApi> {
        unsafe {
            Some(AvionicsApi {
                create: std::mem::transmute::<*mut c_void, unsafe extern "C" fn(*mut CreateAvionics) -> AvionicsId>(
                    Self::xplm_symbol("XPLMCreateAvionicsEx")?,
                ),
                destroy: std::mem::transmute::<*mut c_void, unsafe extern "C" fn(AvionicsId)>(Self::xplm_symbol("XPLMDestroyAvionics")?),
                is_bound: std::mem::transmute::<*mut c_void, unsafe extern "C" fn(AvionicsId) -> c_int>(
                    Self::xplm_symbol("XPLMIsAvionicsBound")?,
                ),
                set_graphics_state: self.graphics_state,
                bind_texture: std::mem::transmute::<*mut c_void, unsafe extern "C" fn(c_int, c_int)>(Self::xplm_symbol("XPLMBindTexture2d")?),
                generate_textures: std::mem::transmute::<*mut c_void, unsafe extern "C" fn(*mut c_int, c_int)>(
                    Self::xplm_symbol("XPLMGenerateTextureNumbers")?,
                ),
                set_popup_visible: std::mem::transmute::<*mut c_void, unsafe extern "C" fn(AvionicsId, c_int)>(
                    Self::xplm_symbol("XPLMSetAvionicsPopupVisible")?,
                ),
                take_keyboard_focus: std::mem::transmute::<*mut c_void, unsafe extern "C" fn(AvionicsId)>(
                    Self::xplm_symbol("XPLMTakeAvionicsKeyboardFocus")?,
                ),
                has_keyboard_focus: std::mem::transmute::<*mut c_void, unsafe extern "C" fn(AvionicsId) -> c_int>(
                    Self::xplm_symbol("XPLMHasAvionicsKeyboardFocus")?,
                ),
            })
        }
    }
}

type FindCommandFn = unsafe extern "C" fn(*const c_char) -> CommandRef;

/// One of X-Plane's existing commands, by name.
pub fn find_command(name: &str) -> Option<CommandRef> {
    let find = Xplm::xplm_symbol("XPLMFindCommand")?;
    let name = CString::new(name).ok()?;
    let command = unsafe { std::mem::transmute::<*mut c_void, FindCommandFn>(find)(name.as_ptr()) };
    (!command.is_null()).then_some(command)
}
type CommandOnceFn = unsafe extern "C" fn(CommandRef);

/// Run one of X-Plane's commands once, as a key press would. Returns whether
/// X-Plane knows the command.
pub fn command_once(name: &str) -> bool {
    if !on_main_thread() {
        QUEUED_COMMANDS.lock().unwrap_or_else(std::sync::PoisonError::into_inner).push(name.to_string());
        return true;
    }
    let (Some(find), Some(once)) = (Xplm::xplm_symbol("XPLMFindCommand"), Xplm::xplm_symbol("XPLMCommandOnce")) else {
        return false;
    };
    let Ok(name) = CString::new(name) else { return false };
    unsafe {
        let find = std::mem::transmute::<*mut c_void, FindCommandFn>(find);
        let once = std::mem::transmute::<*mut c_void, CommandOnceFn>(once);
        let command = find(name.as_ptr());
        if command.is_null() {
            return false;
        }
        once(command);
    }
    true
}

// ------------------------------------------------------------------
// Dataref types and sound (added for src/extra_backend and src/sound).
// ------------------------------------------------------------------

type GetDataRefTypesFn = unsafe extern "C" fn(DataRef) -> c_int;

/// XPLMGetDataRefTypes: the XPLMDataTypeID bits of a dataref, 0 if unknown.
pub fn data_ref_types(dataref: DataRef) -> c_int {
    let Some(f) = Xplm::xplm_symbol("XPLMGetDataRefTypes") else { return 0 };
    unsafe { std::mem::transmute::<*mut c_void, GetDataRefTypesFn>(f)(dataref) }
}

/// An FMOD channel X-Plane plays a sound on (XPLMSound.h, XPLM400).
pub type FmodChannel = *mut c_void;
/// XPLMPCMComplete_f: called when a sound finishes, with an FMOD_RESULT.
pub type PcmComplete = unsafe extern "C" fn(*mut c_void, c_int);

/// XPLMAudioBus values (XPLMSound.h).
pub const AUDIO_EXTERIOR_AIRCRAFT: c_int = 4;
pub const AUDIO_INTERIOR: c_int = 7;
/// FMOD_SOUND_FORMAT_PCM16 (XPLMSound.h).
const FMOD_SOUND_FORMAT_PCM16: c_int = 2;

type PlayPcmOnBusFn =
    unsafe extern "C" fn(*mut c_void, u32, c_int, c_int, c_int, c_int, c_int, Option<PcmComplete>, *mut c_void) -> FmodChannel;
type StopAudioFn = unsafe extern "C" fn(FmodChannel) -> c_int;
type SetAudioVolumeFn = unsafe extern "C" fn(FmodChannel, f32) -> c_int;

/// XPLMPlayPCMOnBus with 16-bit interleaved samples, which X-Plane copies
/// before this returns. `None` before X-Plane 12.04 or when FMOD refuses.
pub fn play_pcm16_on_bus(
    samples: &[i16],
    rate: u32,
    channels: u16,
    looped: bool,
    bus: c_int,
    done: Option<PcmComplete>,
    refcon: *mut c_void,
) -> Option<FmodChannel> {
    let f = Xplm::xplm_symbol("XPLMPlayPCMOnBus")?;
    let f = unsafe { std::mem::transmute::<*mut c_void, PlayPcmOnBusFn>(f) };
    let bytes = std::mem::size_of_val(samples) as u32;
    let channel = unsafe {
        f(
            samples.as_ptr() as *mut c_void,
            bytes,
            FMOD_SOUND_FORMAT_PCM16,
            rate as c_int,
            channels as c_int,
            looped as c_int,
            bus,
            done,
            refcon,
        )
    };
    (!channel.is_null()).then_some(channel)
}

/// Whether this X-Plane has XPLMPlayPCMOnBus.
pub fn has_pcm_audio() -> bool {
    Xplm::xplm_symbol("XPLMPlayPCMOnBus").is_some()
}

/// XPLMStopAudio.
pub fn stop_audio(channel: FmodChannel) {
    if let Some(f) = Xplm::xplm_symbol("XPLMStopAudio") {
        unsafe { std::mem::transmute::<*mut c_void, StopAudioFn>(f)(channel) };
    }
}

/// XPLMSetAudioVolume: 1 is the sound's own level.
pub fn set_audio_volume(channel: FmodChannel, volume: f32) {
    if let Some(f) = Xplm::xplm_symbol("XPLMSetAudioVolume") {
        unsafe { std::mem::transmute::<*mut c_void, SetAudioVolumeFn>(f)(channel, volume) };
    }
}

// ----------------------------------------------------------------------
// [navdata] X-Plane's folder and magnetic variation, for the facility
// database (src/navdata). Looked up on use, like the commands above.
// ----------------------------------------------------------------------

type GetSystemPathFn = unsafe extern "C" fn(*mut c_char);
type GetMagneticVariationFn = unsafe extern "C" fn(f64, f64) -> f32;

// ----------------------------------------------------------------------
// The main thread. X-Plane's SDK may only be called from the thread X-Plane
// calls the plugin on ("has bypassed XPLM when calling SDK functions"),
// yet the panel server, the settings watcher and the logging of worker
// threads reach for it: those calls are cached or queued for the next tick.

static MAIN_THREAD: std::sync::OnceLock<std::thread::ThreadId> = std::sync::OnceLock::new();

/// Called from XPluginStart, on X-Plane's thread.
pub fn remember_main_thread() {
    let _ = MAIN_THREAD.set(std::thread::current().id());
    let _ = system_path();
}

/// Whether this is X-Plane's thread (true before the plugin started, as in tests).
pub fn on_main_thread() -> bool {
    MAIN_THREAD.get().is_none_or(|id| *id == std::thread::current().id())
}

static QUEUED_COMMANDS: std::sync::Mutex<Vec<String>> = std::sync::Mutex::new(Vec::new());

/// Runs the commands other threads asked for; the plugin's tick calls this.
pub fn run_queued_commands() {
    let names = std::mem::take(&mut *QUEUED_COMMANDS.lock().unwrap_or_else(std::sync::PoisonError::into_inner));
    for name in names {
        command_once(&name);
    }
}

/// X-Plane's own folder (XPLMGetSystemPath), or `None` outside X-Plane.
/// Asked once, on X-Plane's thread; every thread reads that answer.
pub fn system_path() -> Option<std::path::PathBuf> {
    static CACHED: std::sync::OnceLock<Option<std::path::PathBuf>> = std::sync::OnceLock::new();
    if let Some(path) = CACHED.get() {
        return path.clone();
    }
    if !on_main_thread() {
        return None;
    }
    let path = system_path_uncached();
    let _ = CACHED.set(path.clone());
    path
}

fn system_path_uncached() -> Option<std::path::PathBuf> {
    let f = Xplm::xplm_symbol("XPLMGetSystemPath")?;
    // The SDK asks for at least 512 characters.
    let mut buffer = vec![0u8; 2048];
    let path = unsafe {
        std::mem::transmute::<*mut c_void, GetSystemPathFn>(f)(buffer.as_mut_ptr() as *mut c_char);
        std::ffi::CStr::from_ptr(buffer.as_ptr() as *const c_char)
    };
    let path = path.to_string_lossy().into_owned();
    (!path.is_empty()).then(|| std::path::PathBuf::from(path))
}

/// X-Plane's simulated magnetic variation at a point, degrees east
/// (XPLMGetMagneticVariation, XPLMScenery.h). Main thread only.
pub fn magnetic_variation(lat: f64, lon: f64) -> Option<f32> {
    if !on_main_thread() {
        return None;
    }
    static SYMBOL: std::sync::OnceLock<Option<usize>> = std::sync::OnceLock::new();
    let f = (*SYMBOL.get_or_init(|| Xplm::xplm_symbol("XPLMGetMagneticVariation").map(|p| p as usize)))?;
    Some(unsafe { std::mem::transmute::<usize, GetMagneticVariationFn>(f)(lat, lon) })
}

// ----------------------------------------------------------------------
// [wxr] X-Plane's weather API (XPLMWeather.h, XPLM400/420), for src/wxr.
// Looked up on use, like the commands above. XPLMGetWeatherAtLocation's
// header says "not intended to be used per-frame ... called only during
// the pre-flight loop callback"; src/wxr budgets its calls (a handful per
// tick, spread over many ticks) rather than calling it for every pixel,
// and only from the main thread's own tick (docs/wxr.md).
// ----------------------------------------------------------------------

const WXR_WIND_LAYERS: usize = 13;
const WXR_CLOUD_LAYERS: usize = 3;
/// XPLM420's per-layer temperature/dewpoint arrays; not read here, but part
/// of the struct's real layout, needed so `struct_size` and field offsets
/// match X-Plane's copy exactly.
const WXR_TEMP_LAYERS: usize = 13;

/// `XPLMWeatherInfoWinds_t` (XPLMWeather.h). Only `turbulence` is read.
#[repr(C)]
#[derive(Clone, Copy, Default)]
struct WeatherWinds {
    alt_msl: f32,
    speed: f32,
    direction: f32,
    gust_speed: f32,
    shear: f32,
    turbulence: f32,
}

/// `XPLMWeatherInfoClouds_t` (XPLMWeather.h).
#[repr(C)]
#[derive(Clone, Copy, Default)]
struct WeatherClouds {
    cloud_type: f32,
    coverage: f32,
    alt_top: f32,
    alt_base: f32,
}

/// `XPLMWeatherInfo_t` (XPLMWeather.h), field for field including the
/// XPLM420 tail, so `structSize` and every offset match X-Plane's copy.
/// X-Plane 12.4.4 with SDK 4.3.0 headers has all of it.
#[repr(C)]
struct WeatherInfoRaw {
    struct_size: c_int,
    temperature_alt: f32,
    dewpoint_alt: f32,
    pressure_alt: f32,
    precip_rate_alt: f32,
    wind_dir_alt: f32,
    wind_spd_alt: f32,
    turbulence_alt: f32,
    wave_height: f32,
    wave_length: f32,
    wave_dir: c_int,
    wave_speed: f32,
    visibility: f32,
    precip_rate: f32,
    thermal_climb: f32,
    pressure_sl: f32,
    wind_layers: [WeatherWinds; WXR_WIND_LAYERS],
    cloud_layers: [WeatherClouds; WXR_CLOUD_LAYERS],
    temp_layers: [f32; WXR_TEMP_LAYERS],
    dewp_layers: [f32; WXR_TEMP_LAYERS],
    troposphere_alt: f32,
    troposphere_temp: f32,
    age: f32,
    radius_nm: f32,
    max_altitude_msl_ft: f32,
}

type GetWeatherAtLocationFn = unsafe extern "C" fn(f64, f64, f64, *mut WeatherInfoRaw) -> c_int;

// `WeatherSample`/`WeatherCloudLayer` are `deep::weather`'s own types now
// (imported above) -- `deep` owns the shape, this file only fills it from
// `XPLMWeatherInfo_t`. `WXR_CLOUD_LAYERS` (3) matches
// `deep::weather::CLOUD_LAYERS` exactly; both come from the same real SDK
// struct.

/// `XPLMGetWeatherAtLocation`: X-Plane's weather at a point (main thread
/// only; see the module comment above). `None` if this X-Plane has no
/// weather API (pre-12 SDKs) or the call is unavailable outside X-Plane.
pub fn weather_at_location(lat: f64, lon: f64, alt_m: f64) -> Option<WeatherSample> {
    let f = Xplm::xplm_symbol("XPLMGetWeatherAtLocation")?;
    let f = unsafe { std::mem::transmute::<*mut c_void, GetWeatherAtLocationFn>(f) };
    let mut raw: WeatherInfoRaw = unsafe { std::mem::zeroed() };
    raw.struct_size = std::mem::size_of::<WeatherInfoRaw>() as c_int;
    let detailed = unsafe { f(lat, lon, alt_m, &mut raw) } != 0;
    Some(WeatherSample {
        precip_rate_alt: raw.precip_rate_alt,
        precip_rate: raw.precip_rate,
        turbulence_alt: raw.turbulence_alt,
        clouds: raw.cloud_layers.map(|c| WeatherCloudLayer {
            cloud_type: c.cloud_type,
            coverage: c.coverage,
            alt_base_m: c.alt_base,
            alt_top_m: c.alt_top,
        }),
        detailed,
    })
}

/// Whether this X-Plane has `XPLMGetWeatherAtLocation` (the weather API is
/// `XPLM400`, X-Plane 12; absent in a build against an older SDK target).
pub fn has_weather_api() -> bool {
    Xplm::xplm_symbol("XPLMGetWeatherAtLocation").is_some()
}

/// `deep::weather::WeatherSource`, live: forwards to [`weather_at_location`]/
/// [`has_weather_api`] above, unchanged. `Xplm`'s own methods don't need
/// `self` for these two (both look up their symbol independently), so this
/// impl exists purely so `deep::integration::weather_truth` can hold an
/// `Option<&dyn WeatherSource>` the same way it already holds
/// `Option<&Xplm>` for everything else it reads.
impl WeatherSource for Xplm {
    fn has_weather_api(&self) -> bool {
        has_weather_api()
    }

    fn weather_at_location(&self, lat: f64, lon: f64, alt_m: f64) -> Option<WeatherSample> {
        weather_at_location(lat, lon, alt_m)
    }
}

// ----------------------------------------------------------------------
// [deep/integration/xp_consequences] The plugin-force/gear-deploy-ratio
// host seam (`ForceSink`): read-add-write on the six `*_plug_acf`
// datarefs (X-Plane zeroes them every frame -- see that module's own
// doc), plus `sim/flightmodel2/gear/deploy_ratio`. Handles resolved once
// in `XpForceSink::new`, like every other seam in this file.
// ----------------------------------------------------------------------

/// The live [`ForceSink`]: the same six plug-force datarefs and the same
/// `deploy_ratio` element `deep::integration::xp_consequences` always
/// wrote, just resolved and cached here instead of inline in that file.
pub struct XpForceSink<'a> {
    xplm: &'a Xplm,
    fside: Option<DataRef>,
    fnrml: Option<DataRef>,
    faxil: Option<DataRef>,
    roll: Option<DataRef>,
    pitch: Option<DataRef>,
    yaw: Option<DataRef>,
    gear_deploy_ratio: Option<DataRef>,
}

impl<'a> XpForceSink<'a> {
    pub fn new(xplm: &'a Xplm) -> Self {
        Self {
            xplm,
            fside: xplm.find("sim/flightmodel/forces/fside_plug_acf"),
            fnrml: xplm.find("sim/flightmodel/forces/fnrml_plug_acf"),
            faxil: xplm.find("sim/flightmodel/forces/faxil_plug_acf"),
            roll: xplm.find("sim/flightmodel/forces/L_plug_acf"),
            pitch: xplm.find("sim/flightmodel/forces/M_plug_acf"),
            yaw: xplm.find("sim/flightmodel/forces/N_plug_acf"),
            gear_deploy_ratio: xplm.find("sim/flightmodel2/gear/deploy_ratio"),
        }
    }

    fn axis_ref(&self, axis: PlugForceAxis) -> Option<DataRef> {
        match axis {
            PlugForceAxis::Fside => self.fside,
            PlugForceAxis::Fnrml => self.fnrml,
            PlugForceAxis::Faxil => self.faxil,
            PlugForceAxis::Roll => self.roll,
            PlugForceAxis::Pitch => self.pitch,
            PlugForceAxis::Yaw => self.yaw,
        }
    }
}

impl ForceSink for XpForceSink<'_> {
    fn add_plug_force(&self, axis: PlugForceAxis, delta: f64) {
        if let Some(d) = self.axis_ref(axis) {
            self.xplm.set_f(d, (self.xplm.get_f(d) as f64 + delta) as f32);
        }
    }

    fn set_gear_deploy_ratio(&self, index: usize, ratio: f32) {
        if let Some(d) = self.gear_deploy_ratio {
            self.xplm.set_vf_at(d, index, ratio);
        }
    }
}

// ----------------------------------------------------------------------
// [physics/adirs] Terrain probing (XPLMScenery.h, XPLM200), for the radio
// altimeter's boresight-range physics (src/physics/adirs.rs,
// docs/physics/adirs.md). Looked up on use, like the weather API above; the
// probe object itself is created once and reused, as the header recommends,
// rather than per call. src/physics/adirs.rs budgets calls to this (a
// throttled sample every few ticks), consistent with src/wxr's own note
// that XPLM's per-frame terrain/weather APIs are not meant to be hammered.
// ----------------------------------------------------------------------

/// `XPLMProbeInfo_t` (XPLMScenery.h); only the hit location is read.
#[repr(C)]
struct ProbeInfoRaw {
    struct_size: c_int,
    location_x: f32,
    location_y: f32,
    location_z: f32,
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
    velocity_x: f32,
    velocity_y: f32,
    velocity_z: f32,
    is_wet: c_int,
}

type CreateProbeFn = unsafe extern "C" fn(c_int) -> *mut c_void;
type ProbeTerrainXyzFn =
    unsafe extern "C" fn(*mut c_void, f32, f32, f32, *mut ProbeInfoRaw) -> c_int;

/// Probes X-Plane's terrain at a local OpenGL-coordinate point (the same
/// frame `sim/flightmodel/position/local_x/y/z` are in), returning the
/// terrain's Y (up) coordinate, metres, if the probe hit terrain.
/// `XPLMProbeTerrainXYZ` (XPLMScenery.h, `XPLM200`); `xplm_ProbeY` (probe
/// type, value 0) and `xplm_ProbeHitTerrain` (result, value 0) are the
/// header's own constants. Main thread only.
pub fn probe_terrain_y(x: f64, y: f64, z: f64) -> Option<f64> {
    static PROBE: std::sync::OnceLock<Option<usize>> = std::sync::OnceLock::new();
    let probe = (*PROBE.get_or_init(|| {
        let create = Xplm::xplm_symbol("XPLMCreateProbe")?;
        let create = unsafe { std::mem::transmute::<*mut c_void, CreateProbeFn>(create) };
        let probe = unsafe { create(0) }; // xplm_ProbeY
        (!probe.is_null()).then_some(probe as usize)
    }))?;
    let f = Xplm::xplm_symbol("XPLMProbeTerrainXYZ")?;
    let f = unsafe { std::mem::transmute::<*mut c_void, ProbeTerrainXyzFn>(f) };
    let mut info: ProbeInfoRaw = unsafe { std::mem::zeroed() };
    info.struct_size = std::mem::size_of::<ProbeInfoRaw>() as c_int;
    let result = unsafe { f(probe as *mut c_void, x as f32, y as f32, z as f32, &mut info) };
    (result == 0).then_some(info.location_y as f64) // xplm_ProbeHitTerrain
}

#[cfg(any(test, feature = "test-support"))]
thread_local! {
    /// [`Xplm::memory`]'s datarefs: name -> index, and each one's values.
    /// Per thread, so an emulator on one thread (a battery case, or one
    /// abandoned after a timeout) never sees another's.
    static MEMORY_DATAREFS: std::cell::RefCell<(std::collections::HashMap<String, usize>, Vec<Vec<f64>>)> = Default::default();
}

#[cfg(any(test, feature = "test-support"))]
impl Xplm {
    /// Clear this thread's [`Xplm::memory`] datarefs (a fresh aircraft).
    pub fn reset_memory() {
        MEMORY_DATAREFS.with(|m| *m.borrow_mut() = Default::default());
    }

    /// Write element `index` of a dataref in this thread's
    /// [`Xplm::memory`] store by name: what X-Plane itself would supply
    /// (a thrust lever position, an airspeed), for the offline emulator.
    pub fn memory_set(name: &str, index: usize, value: f64) {
        MEMORY_DATAREFS.with(|m| {
            let mut m = m.borrow_mut();
            let next = m.1.len();
            let slot = *m.0.entry(name.to_owned()).or_insert(next);
            if slot == next {
                m.1.push(Vec::new());
            }
            let v = &mut m.1[slot];
            if v.len() <= index {
                v.resize(index + 1, 0.0);
            }
            v[index] = value;
        });
    }

    /// A binding with no X-Plane behind it whose datarefs hold what was last
    /// written to them, as X-Plane's own do, and read 0 (or an empty array)
    /// until then. The plugin's modules talk to each other through some
    /// datarefs (the engine physics writes N1/N2/fuel flow that the FADEC
    /// reads back); [`Xplm::dummy`] drops those writes, this keeps them.
    /// Nothing X-Plane itself computes (the flight model, weather) appears:
    /// only values a module wrote.
    pub fn memory() -> Self {
        fn slot(d: DataRef) -> Option<usize> {
            (d as usize).checked_sub(1)
        }
        fn read(d: DataRef, i: usize) -> f64 {
            slot(d).map_or(0., |s| MEMORY_DATAREFS.with(|m| m.borrow().1.get(s).and_then(|v| v.get(i)).copied().unwrap_or(0.)))
        }
        fn write(d: DataRef, i: usize, value: f64) {
            if let Some(s) = slot(d) {
                MEMORY_DATAREFS.with(|m| {
                    let mut m = m.borrow_mut();
                    if let Some(v) = m.1.get_mut(s) {
                        if v.len() <= i {
                            v.resize(i + 1, 0.);
                        }
                        v[i] = value;
                    }
                });
            }
        }
        fn len(d: DataRef) -> usize {
            slot(d).map_or(0, |s| MEMORY_DATAREFS.with(|m| m.borrow().1.get(s).map_or(0, Vec::len)))
        }
        unsafe extern "C" fn find(name: *const c_char) -> DataRef {
            let name = unsafe { std::ffi::CStr::from_ptr(name) }.to_string_lossy().into_owned();
            MEMORY_DATAREFS.with(|m| {
                let mut m = m.borrow_mut();
                let next = m.1.len();
                let index = *m.0.entry(name).or_insert(next);
                if index == next {
                    m.1.push(Vec::new());
                }
                (index + 1) as DataRef
            })
        }
        unsafe extern "C" fn get_f(d: DataRef) -> f32 {
            read(d, 0) as f32
        }
        unsafe extern "C" fn get_d(d: DataRef) -> f64 {
            read(d, 0)
        }
        unsafe extern "C" fn get_i(d: DataRef) -> c_int {
            read(d, 0) as c_int
        }
        // X-Plane's array read: with no buffer, the array's size; otherwise
        // copy up to `max` values from `offset` and return how many.
        unsafe extern "C" fn get_vf(d: DataRef, out: *mut f32, offset: c_int, max: c_int) -> c_int {
            let n = len(d);
            if out.is_null() {
                return n as c_int;
            }
            let count = n.saturating_sub(offset.max(0) as usize).min(max.max(0) as usize);
            for k in 0..count {
                unsafe { *out.add(k) = read(d, offset as usize + k) as f32 };
            }
            count as c_int
        }
        unsafe extern "C" fn get_vi(d: DataRef, out: *mut c_int, offset: c_int, max: c_int) -> c_int {
            let n = len(d);
            if out.is_null() {
                return n as c_int;
            }
            let count = n.saturating_sub(offset.max(0) as usize).min(max.max(0) as usize);
            for k in 0..count {
                unsafe { *out.add(k) = read(d, offset as usize + k) as c_int };
            }
            count as c_int
        }
        unsafe extern "C" fn set_f(d: DataRef, value: f32) {
            write(d, 0, value as f64);
        }
        unsafe extern "C" fn set_i(d: DataRef, value: c_int) {
            write(d, 0, value as f64);
        }
        unsafe extern "C" fn set_vf(d: DataRef, values: *mut f32, offset: c_int, count: c_int) {
            for k in 0..count.max(0) as usize {
                write(d, offset.max(0) as usize + k, unsafe { *values.add(k) } as f64);
            }
        }
        unsafe extern "C" fn set_vi(d: DataRef, values: *mut c_int, offset: c_int, count: c_int) {
            for k in 0..count.max(0) as usize {
                write(d, offset.max(0) as usize + k, unsafe { *values.add(k) } as f64);
            }
        }
        Self { find, get_f, get_d, get_i, get_vf, get_vi, set_f, set_i, set_vf, set_vi, ..Self::dummy() }
    }

    /// A do-nothing binding for tests, which do not run inside X-Plane and
    /// so cannot `LoadLibraryA("XPLM_64.dll")`. `find` always answers "no
    /// such dataref" (`None`); that is enough for `Vars::add`/`read`/`write`
    /// (lib.rs), which never call X-Plane except through `find`, and only
    /// for a simulator variable that happens to have an X-Plane input
    /// mapping — an `L:` (named/aircraft) variable never does. The rest of
    /// the bindings are never exercised by a test that stays off real
    /// datarefs, windows and menus.
    ///
    /// `pub` under `feature = "test-support"` only, so the out-of-tree
    /// `emulator` crate can build a `Vars` the same way `offline_harness.rs`
    /// does, with no live X-Plane process. See that feature's doc comment
    /// in Cargo.toml.
    pub fn dummy() -> Self {
        unsafe extern "C" fn find(_: *const c_char) -> DataRef {
            std::ptr::null_mut()
        }
        unsafe extern "C" fn get_f(_: DataRef) -> f32 {
            0.
        }
        unsafe extern "C" fn get_d(_: DataRef) -> f64 {
            0.
        }
        unsafe extern "C" fn get_i(_: DataRef) -> c_int {
            0
        }
        unsafe extern "C" fn get_vf(_: DataRef, _: *mut f32, _: c_int, _: c_int) -> c_int {
            0
        }
        unsafe extern "C" fn get_vi(_: DataRef, _: *mut c_int, _: c_int, _: c_int) -> c_int {
            0
        }
        unsafe extern "C" fn set_f(_: DataRef, _: f32) {}
        unsafe extern "C" fn set_i(_: DataRef, _: c_int) {}
        unsafe extern "C" fn set_vf(_: DataRef, _: *mut f32, _: c_int, _: c_int) {}
        unsafe extern "C" fn set_vi(_: DataRef, _: *mut c_int, _: c_int, _: c_int) {}
        unsafe extern "C" fn debug(_: *const c_char) {}
        unsafe extern "C" fn register_accessor(
            _: *const c_char,
            _: c_int,
            _: c_int,
            _: GetI,
            _: SetI,
            _: GetF,
            _: SetF,
            _: GetD,
            _: SetD,
            _: GetVi,
            _: SetVi,
            _: GetVf,
            _: SetVf,
            _: GetB,
            _: SetB,
            _: *mut c_void,
            _: *mut c_void,
        ) -> DataRef {
            std::ptr::null_mut()
        }
        unsafe extern "C" fn register_loop(_: FlightLoop, _: f32, _: *mut c_void) {}
        unsafe extern "C" fn unregister_loop(_: FlightLoop, _: *mut c_void) {}
        unsafe extern "C" fn create_window(_: *mut CreateWindow) -> WindowId {
            std::ptr::null_mut()
        }
        unsafe extern "C" fn destroy_window(_: WindowId) {}
        unsafe extern "C" fn set_window_title(_: WindowId, _: *const c_char) {}
        unsafe extern "C" fn set_positioning(_: WindowId, _: c_int, _: c_int) {}
        unsafe extern "C" fn set_resizing_limits(_: WindowId, _: c_int, _: c_int, _: c_int, _: c_int) {}
        unsafe extern "C" fn get_geometry(_: WindowId, _: *mut c_int, _: *mut c_int, _: *mut c_int, _: *mut c_int) {}
        unsafe extern "C" fn set_visible(_: WindowId, _: c_int) {}
        unsafe extern "C" fn get_visible(_: WindowId) -> c_int {
            0
        }
        unsafe extern "C" fn bring_to_front(_: WindowId) {}
        unsafe extern "C" fn screen_bounds(_: *mut c_int, _: *mut c_int, _: *mut c_int, _: *mut c_int) {}
        unsafe extern "C" fn draw_string(_: *mut f32, _: c_int, _: c_int, _: *const c_char, _: *mut c_int, _: c_int) {}
        unsafe extern "C" fn measure_string(_: c_int, _: *const c_char, _: c_int) -> f32 {
            0.
        }
        unsafe extern "C" fn font_dimensions(_: c_int, _: *mut c_int, _: *mut c_int, _: *mut c_int) {}
        unsafe extern "C" fn aircraft_menu() -> MenuId {
            std::ptr::null_mut()
        }
        unsafe extern "C" fn plugins_menu() -> MenuId {
            std::ptr::null_mut()
        }
        unsafe extern "C" fn create_menu(_: *const c_char, _: MenuId, _: c_int, _: Option<MenuHandler>, _: *mut c_void) -> MenuId {
            std::ptr::null_mut()
        }
        unsafe extern "C" fn append_item(_: MenuId, _: *const c_char, _: *mut c_void, _: c_int) -> c_int {
            0
        }
        unsafe extern "C" fn append_separator(_: MenuId) {}
        unsafe extern "C" fn destroy_menu(_: MenuId) {}
        unsafe extern "C" fn graphics_state(_: c_int, _: c_int, _: c_int, _: c_int, _: c_int, _: c_int, _: c_int) {}
        Self {
            find,
            get_f,
            get_d,
            get_i,
            get_vf,
            get_vi,
            set_f,
            set_i,
            set_vf,
            set_vi,
            debug,
            register_accessor,
            register_loop,
            unregister_loop,
            create_window,
            destroy_window,
            set_window_title,
            set_positioning,
            set_resizing_limits,
            get_geometry,
            set_visible,
            get_visible,
            bring_to_front,
            screen_bounds,
            draw_string,
            measure_string,
            font_dimensions,
            aircraft_menu,
            plugins_menu,
            create_menu,
            append_item,
            append_separator,
            destroy_menu,
            graphics_state,
            gl: None,
        }
    }
}
