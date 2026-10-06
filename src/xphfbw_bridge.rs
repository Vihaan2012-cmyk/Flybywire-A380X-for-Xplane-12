//! The protocol between the X-Plane plugin and XPHFBW's instruments
//! (docs/briefs/xphfbw-js-bridge.md). One module, used by both sides: the
//! plugin (which creates every object) and XPHFBW's browser and renderer
//! processes (which open them by the session tag).
//!
//! It carries the same exchange the plugin's QuickJS worker uses
//! (js_worker.rs `Frame`/`Outbox`):
//!
//! - **Slots** (`SlotTable`): the first time a view reads a variable in a
//!   unit it registers `(name, unit)` and gets a slot; every frame the plugin
//!   writes every slot's converted value into `values`. Renderer processes
//!   map the block and read values directly, as MSFS's SimVar reads are.
//! - **Uplink** (`Ring`, views to plugin): slot writes, events, Coherent
//!   calls for the plugin's providers, GAME: string requests, logs.
//! - **Downlink** (`Ring` per view, plugin to a view): call replies, H:
//!   events, provider events, GAME: string values.
//! - **Screens** (`ScreenBlock` per cockpit screen, app to plugin): BGRA
//!   pixels with dirty rectangles.
//! - **Input** (`Ring`, plugin to app): mouse and wheel on a screen.
//!
//! Change a layout only with [`VERSION`] bumped: both sides check it.

use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

use crate::remote::win::{self, Shared};

pub const MAGIC: u64 = 0x5850_4846_4257_4A53; // "XPHFBWJS"
/// Bumped for `SlotHeader::view_loaded` (app/src/views.rs's `watch_loaded`
/// watchdog needs it; a session created by an old plugin binary simply fails
/// `SlotTable::open`'s version check and the app waits for a fresh one).
pub const VERSION: u32 = 2;

/// Views (panel.cfg VCockpit sections, including the screenless hosts).
pub const MAX_VIEWS: usize = 32;
pub const SLOT_CAPACITY: usize = 65_536;
pub const SLOT_NAMES_BYTES: usize = 8 << 20;
pub const UPLINK_BYTES: usize = 8 << 20;
pub const DOWNLINK_BYTES: usize = 2 << 20;
pub const INPUT_BYTES: usize = 256 << 10;
pub const MAX_DIRTY_RECTS: usize = 32;

/// Object names for a session (`tag` as the plugin passes XPHFBW).
pub fn object_name(tag: &str, what: &str) -> String {
    format!("Local\\XPHFBW_{tag}_{what}")
}

// ---------------------------------------------------------------------------
// A named mutex, for many writers across processes.
// ---------------------------------------------------------------------------

#[cfg(windows)]
#[link(name = "kernel32")]
extern "system" {
    fn CreateMutexW(attributes: *mut std::ffi::c_void, initial_owner: i32, name: *const u16) -> win::Handle;
    fn OpenMutexW(access: u32, inherit: i32, name: *const u16) -> win::Handle;
    fn ReleaseMutex(mutex: win::Handle) -> i32;
    fn WaitForSingleObject(object: win::Handle, milliseconds: u32) -> u32;
    fn CloseHandle(object: win::Handle) -> i32;
}

/// Not Windows -- the MSFS wasm build compiles this file, though no second
/// process exists there to share a mutex with. Answers as the real calls do
/// when the object cannot be had, so `NamedMutex::create`/`open` return
/// `None` through their existing null checks.
#[cfg(not(windows))]
#[allow(non_snake_case)]
mod stand_ins {
    use crate::remote::win;
    pub unsafe fn CreateMutexW(_: *mut std::ffi::c_void, _: i32, _: *const u16) -> win::Handle { std::ptr::null_mut() }
    pub unsafe fn OpenMutexW(_: u32, _: i32, _: *const u16) -> win::Handle { std::ptr::null_mut() }
    pub unsafe fn ReleaseMutex(_: win::Handle) -> i32 { 0 }
    pub unsafe fn WaitForSingleObject(_: win::Handle, _: u32) -> u32 { win::WAIT_TIMEOUT }
    pub unsafe fn CloseHandle(_: win::Handle) -> i32 { 0 }
}
#[cfg(not(windows))]
use stand_ins::*;

fn wide(s: &str) -> Vec<u16> {
    s.encode_utf16().chain(std::iter::once(0)).collect()
}

pub struct NamedMutex(win::Handle);

unsafe impl Send for NamedMutex {}
unsafe impl Sync for NamedMutex {}

impl NamedMutex {
    pub fn create(name: &str) -> Option<Self> {
        let n = wide(name);
        let h = unsafe { CreateMutexW(std::ptr::null_mut(), 0, n.as_ptr()) };
        (!h.is_null()).then_some(Self(h))
    }

    pub fn open(name: &str) -> Option<Self> {
        const SYNCHRONIZE: u32 = 0x0010_0000;
        const MUTEX_MODIFY_STATE: u32 = 0x0001;
        let n = wide(name);
        let h = unsafe { OpenMutexW(SYNCHRONIZE | MUTEX_MODIFY_STATE, 0, n.as_ptr()) };
        (!h.is_null()).then_some(Self(h))
    }

    /// Run `f` holding the mutex. A holder that died (WAIT_ABANDONED) still
    /// hands it over; the data it guards is length-prefixed and checked.
    pub fn with<R>(&self, f: impl FnOnce() -> R) -> R {
        const WAIT_ABANDONED: u32 = 0x80;
        let r = unsafe { WaitForSingleObject(self.0, win::INFINITE) };
        let locked = r == win::WAIT_OBJECT_0 || r == WAIT_ABANDONED;
        let out = f();
        if locked {
            unsafe { ReleaseMutex(self.0) };
        }
        out
    }
}

impl Drop for NamedMutex {
    fn drop(&mut self) {
        unsafe { CloseHandle(self.0) };
    }
}

// ---------------------------------------------------------------------------
// Ring: a byte queue of length-prefixed records in shared memory.
// ---------------------------------------------------------------------------

#[repr(C)]
struct RingHeader {
    magic: AtomicU64,
    capacity: AtomicU32,
    /// Bytes written and read so far, modulo 2^32 (head - tail = in use).
    head: AtomicU32,
    tail: AtomicU32,
    /// Records dropped because the ring was full.
    dropped: AtomicU32,
}

const RING_HEADER: usize = std::mem::size_of::<RingHeader>();

/// Many writers, one reader; every operation holds the ring's mutex.
pub struct Ring {
    shared: Shared,
    mutex: NamedMutex,
}

impl Ring {
    pub fn create(name: &str, capacity: usize) -> Option<Self> {
        let shared = Shared::create(name, RING_HEADER + capacity)?;
        let mutex = NamedMutex::create(&format!("{name}_lock"))?;
        let ring = Self { shared, mutex };
        let h = ring.header();
        h.magic.store(MAGIC, Ordering::Relaxed);
        h.capacity.store(capacity as u32, Ordering::Relaxed);
        Some(ring)
    }

    pub fn open(name: &str, capacity: usize) -> Option<Self> {
        let shared = Shared::open(name, RING_HEADER + capacity)?;
        let mutex = NamedMutex::open(&format!("{name}_lock"))?;
        let ring = Self { shared, mutex };
        (ring.header().magic.load(Ordering::Relaxed) == MAGIC).then_some(ring)
    }

    fn header(&self) -> &RingHeader {
        unsafe { &*(self.shared.ptr() as *const RingHeader) }
    }

    fn data(&self) -> *mut u8 {
        unsafe { self.shared.ptr().add(RING_HEADER) }
    }

    /// Append one record; false (and counted) when it does not fit.
    pub fn push(&self, record: &[u8]) -> bool {
        self.mutex.with(|| {
            let h = self.header();
            let cap = h.capacity.load(Ordering::Relaxed) as usize;
            let head = h.head.load(Ordering::Relaxed);
            let used = head.wrapping_sub(h.tail.load(Ordering::Relaxed)) as usize;
            let need = 4 + record.len();
            if used + need > cap {
                h.dropped.fetch_add(1, Ordering::Relaxed);
                return false;
            }
            let mut at = head as usize % cap;
            for byte in (record.len() as u32).to_le_bytes().iter().chain(record) {
                unsafe { *self.data().add(at) = *byte };
                at = (at + 1) % cap;
            }
            h.head.store(head.wrapping_add(need as u32), Ordering::Relaxed);
            true
        })
    }

    /// Take every record queued so far.
    pub fn drain(&self) -> Vec<Vec<u8>> {
        self.mutex.with(|| {
            let h = self.header();
            let cap = h.capacity.load(Ordering::Relaxed) as usize;
            let head = h.head.load(Ordering::Relaxed);
            let mut tail = h.tail.load(Ordering::Relaxed);
            let mut out = Vec::new();
            let byte = |at: usize| unsafe { *self.data().add(at % cap) };
            while tail != head {
                let at = tail as usize;
                let len = u32::from_le_bytes([byte(at), byte(at + 1), byte(at + 2), byte(at + 3)]) as usize;
                if 4 + len > head.wrapping_sub(tail) as usize {
                    // A torn record (a writer died mid-way): drop the rest.
                    tail = head;
                    break;
                }
                out.push((0..len).map(|i| byte(at + 4 + i)).collect());
                tail = tail.wrapping_add(4 + len as u32);
            }
            h.tail.store(tail, Ordering::Relaxed);
            out
        })
    }

    pub fn dropped(&self) -> u32 {
        self.header().dropped.load(Ordering::Relaxed)
    }
}

// ---------------------------------------------------------------------------
// Records: a kind byte, then fields; strings as u32 length + UTF-8.
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, PartialEq)]
pub enum Uplink {
    /// A view wrote a slot (value in the slot's unit).
    Write { slot: u32, value: f64 },
    /// `K:`/`H:` writes and single-value events (`name` with its prefix).
    Event { name: String, values: Vec<f64> },
    /// A string variable write (`SimVar.SetSimVarValue` with a string).
    SetString { name: String, value: String },
    /// A Coherent call for the plugin's providers.
    Call { view: u32, id: u64, name: String, args_json: String },
    /// A `GAME:` string a view wants.
    GameString { view: u32, name: String },
    Log { view: u32, level: u8, text: String },
    /// A view finished loading (or failed: `ok` false and why).
    Loaded { view: u32, ok: bool, text: String },
}

#[derive(Debug, Clone, PartialEq)]
pub enum Downlink {
    /// `ok` true: resolve with JSON (empty for undefined); false: reject with the message.
    Reply { id: u64, ok: bool, text: String },
    HEvent { name: String },
    /// A provider's event for Coherent.on / view listeners.
    ProviderEvent { name: String, json: String },
    GameString { name: String, value: String },
    /// A string variable's value.
    StringValue { name: String, value: String },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InputKind {
    Down,
    Up,
    Move,
    Wheel,
    /// One keystroke from a cockpit device's `XPLMAvionicsKeyboard_f`
    /// callback (fires only once that device's popup window has keyboard
    /// focus, XPLMDisplay.h): `x` carries the typed character's code point
    /// (0 if the key has none, e.g. a bare modifier), `y` the SDK's virtual
    /// key code, `delta` its modifier flags. The KCCU types into the MFD
    /// this way.
    Key,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Input {
    /// Index into the screens the app announced (ScreenBlock order).
    pub screen: u32,
    pub kind: InputKind,
    /// CSS pixels from the screen's top left ([`InputKind::Key`]: see its
    /// doc comment instead, these are not pixels then).
    pub x: f32,
    pub y: f32,
    /// Wheel clicks, up positive.
    pub delta: f32,
}

struct Writer(Vec<u8>);

impl Writer {
    fn new(kind: u8) -> Self {
        Self(vec![kind])
    }
    fn u32(mut self, v: u32) -> Self {
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }
    fn u64(mut self, v: u64) -> Self {
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }
    fn f64(mut self, v: f64) -> Self {
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }
    fn f32(mut self, v: f32) -> Self {
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }
    fn u8(mut self, v: u8) -> Self {
        self.0.push(v);
        self
    }
    fn str(self, s: &str) -> Self {
        let mut w = self.u32(s.len() as u32);
        w.0.extend_from_slice(s.as_bytes());
        w
    }
}

struct Reader<'a>(&'a [u8]);

impl<'a> Reader<'a> {
    fn take(&mut self, n: usize) -> Option<&'a [u8]> {
        if self.0.len() < n {
            return None;
        }
        let (a, b) = self.0.split_at(n);
        self.0 = b;
        Some(a)
    }
    fn u8(&mut self) -> Option<u8> {
        self.take(1).map(|b| b[0])
    }
    fn u32(&mut self) -> Option<u32> {
        self.take(4).map(|b| u32::from_le_bytes(b.try_into().unwrap()))
    }
    fn u64(&mut self) -> Option<u64> {
        self.take(8).map(|b| u64::from_le_bytes(b.try_into().unwrap()))
    }
    fn f64(&mut self) -> Option<f64> {
        self.take(8).map(|b| f64::from_le_bytes(b.try_into().unwrap()))
    }
    fn f32(&mut self) -> Option<f32> {
        self.take(4).map(|b| f32::from_le_bytes(b.try_into().unwrap()))
    }
    fn str(&mut self) -> Option<String> {
        let n = self.u32()? as usize;
        self.take(n).map(|b| String::from_utf8_lossy(b).into_owned())
    }
}

impl Uplink {
    pub fn encode(&self) -> Vec<u8> {
        match self {
            Uplink::Write { slot, value } => Writer::new(1).u32(*slot).f64(*value),
            Uplink::Event { name, values } => {
                let mut w = Writer::new(2).str(name).u32(values.len() as u32);
                for v in values {
                    w = w.f64(*v);
                }
                w
            }
            Uplink::SetString { name, value } => Writer::new(3).str(name).str(value),
            Uplink::Call { view, id, name, args_json } => Writer::new(4).u32(*view).u64(*id).str(name).str(args_json),
            Uplink::GameString { view, name } => Writer::new(5).u32(*view).str(name),
            Uplink::Log { view, level, text } => Writer::new(6).u32(*view).u8(*level).str(text),
            Uplink::Loaded { view, ok, text } => Writer::new(7).u32(*view).u8(*ok as u8).str(text),
        }
        .0
    }

    pub fn decode(bytes: &[u8]) -> Option<Self> {
        let mut r = Reader(bytes);
        Some(match r.u8()? {
            1 => Uplink::Write { slot: r.u32()?, value: r.f64()? },
            2 => {
                let name = r.str()?;
                let n = r.u32()? as usize;
                let values = (0..n.min(16)).map(|_| r.f64()).collect::<Option<Vec<_>>>()?;
                Uplink::Event { name, values }
            }
            3 => Uplink::SetString { name: r.str()?, value: r.str()? },
            4 => Uplink::Call { view: r.u32()?, id: r.u64()?, name: r.str()?, args_json: r.str()? },
            5 => Uplink::GameString { view: r.u32()?, name: r.str()? },
            6 => Uplink::Log { view: r.u32()?, level: r.u8()?, text: r.str()? },
            7 => Uplink::Loaded { view: r.u32()?, ok: r.u8()? != 0, text: r.str()? },
            _ => return None,
        })
    }
}

impl Downlink {
    pub fn encode(&self) -> Vec<u8> {
        match self {
            Downlink::Reply { id, ok, text } => Writer::new(1).u64(*id).u8(*ok as u8).str(text),
            Downlink::HEvent { name } => Writer::new(2).str(name),
            Downlink::ProviderEvent { name, json } => Writer::new(3).str(name).str(json),
            Downlink::GameString { name, value } => Writer::new(4).str(name).str(value),
            Downlink::StringValue { name, value } => Writer::new(5).str(name).str(value),
        }
        .0
    }

    pub fn decode(bytes: &[u8]) -> Option<Self> {
        let mut r = Reader(bytes);
        Some(match r.u8()? {
            1 => Downlink::Reply { id: r.u64()?, ok: r.u8()? != 0, text: r.str()? },
            2 => Downlink::HEvent { name: r.str()? },
            3 => Downlink::ProviderEvent { name: r.str()?, json: r.str()? },
            4 => Downlink::GameString { name: r.str()?, value: r.str()? },
            5 => Downlink::StringValue { name: r.str()?, value: r.str()? },
            _ => return None,
        })
    }
}

impl Input {
    pub fn encode(&self) -> Vec<u8> {
        let kind = match self.kind {
            InputKind::Down => 0,
            InputKind::Up => 1,
            InputKind::Move => 2,
            InputKind::Wheel => 3,
            InputKind::Key => 4,
        };
        Writer::new(1).u32(self.screen).u8(kind).f32(self.x).f32(self.y).f32(self.delta).0
    }

    pub fn decode(bytes: &[u8]) -> Option<Self> {
        let mut r = Reader(bytes);
        if r.u8()? != 1 {
            return None;
        }
        let screen = r.u32()?;
        let kind = match r.u8()? {
            0 => InputKind::Down,
            1 => InputKind::Up,
            2 => InputKind::Move,
            4 => InputKind::Key,
            _ => InputKind::Wheel,
        };
        Some(Input { screen, kind, x: r.f32()?, y: r.f32()?, delta: r.f32()? })
    }
}

// ---------------------------------------------------------------------------
// Slots: the variables views read, and every slot's value each frame.
// ---------------------------------------------------------------------------

#[repr(C)]
pub struct SlotHeader {
    pub magic: AtomicU64,
    pub version: AtomicU32,
    /// Slots registered (entries valid in `names`).
    pub count: AtomicU32,
    /// Bytes of the names area in use.
    pub names_used: AtomicU32,
    /// 1 once XPHFBW should draw the screens (plugin's QuickJS engine off).
    pub displays_active: AtomicU32,
    /// Frames the plugin has published (values current as of this frame).
    pub frame: AtomicU64,
    /// Simulation time the frame was published at, milliseconds (f64 bits).
    pub time_ms: AtomicU64,
    /// Slots the plugin has resolved so far (values valid below this).
    pub resolved: AtomicU32,
    pub _pad: AtomicU32,
    /// Each view's last `Uplink::Loaded` report, indexed like
    /// `Uplink::Loaded.view`/`Session.downlinks`: 0 not yet reported, 1
    /// reported `ok: true`, 2 reported `ok: false`. Written by the plugin
    /// (`xphfbw_host.rs`'s `apply_deferred`, mirroring the same report it
    /// folds into `displays_active`) and read by the app's own UI thread
    /// (`app/src/views.rs`'s `watch_loaded`) so it can tell a screen that
    /// painted but whose page never finished FlyByWire's own bootstrap from
    /// one that is still legitimately starting up, without draining the
    /// `uplink` ring itself (only the plugin does that; a second drainer
    /// would race it for records the plugin still needs).
    pub view_loaded: [AtomicU32; MAX_VIEWS],
}

const SLOT_HEADER: usize = std::mem::size_of::<SlotHeader>();
/// Per slot: offset of "name\0unit" in the names area (u32) and its length (u32).
const SLOT_INDEX: usize = SLOT_CAPACITY * 8;
const VALUES_AT: usize = SLOT_HEADER + SLOT_INDEX + SLOT_NAMES_BYTES;
pub const SLOT_TABLE_BYTES: usize = VALUES_AT + SLOT_CAPACITY * 8;

pub struct SlotTable {
    shared: Shared,
    mutex: NamedMutex,
}

impl SlotTable {
    pub fn create(tag: &str) -> Option<Self> {
        let name = object_name(tag, "slots");
        let shared = Shared::create(&name, SLOT_TABLE_BYTES)?;
        let mutex = NamedMutex::create(&format!("{name}_lock"))?;
        let t = Self { shared, mutex };
        t.header().magic.store(MAGIC, Ordering::Relaxed);
        t.header().version.store(VERSION, Ordering::Relaxed);
        Some(t)
    }

    pub fn open(tag: &str) -> Option<Self> {
        let name = object_name(tag, "slots");
        let shared = Shared::open(&name, SLOT_TABLE_BYTES)?;
        let mutex = NamedMutex::open(&format!("{name}_lock"))?;
        let t = Self { shared, mutex };
        let h = t.header();
        (h.magic.load(Ordering::Relaxed) == MAGIC && h.version.load(Ordering::Relaxed) == VERSION).then_some(t)
    }

    pub fn header(&self) -> &SlotHeader {
        unsafe { &*(self.shared.ptr() as *const SlotHeader) }
    }

    fn index_entry(&self, slot: usize) -> (u32, u32) {
        unsafe {
            let p = self.shared.ptr().add(SLOT_HEADER + slot * 8) as *const u32;
            (p.read_unaligned(), p.add(1).read_unaligned())
        }
    }

    /// The slot for `(name, unit)`, registering it if new. `None` when full.
    /// Registration is cross-process (views in several renderer processes).
    pub fn register(&self, name: &str, unit: &str) -> Option<u32> {
        self.mutex.with(|| {
            let h = self.header();
            let count = h.count.load(Ordering::Relaxed) as usize;
            for slot in 0..count {
                let (n, u) = self.entry(slot);
                if n == name && u == unit {
                    return Some(slot as u32);
                }
            }
            let key = format!("{name}\0{unit}");
            let used = h.names_used.load(Ordering::Relaxed) as usize;
            if count >= SLOT_CAPACITY || used + key.len() > SLOT_NAMES_BYTES {
                return None;
            }
            unsafe {
                std::ptr::copy_nonoverlapping(key.as_ptr(), self.shared.ptr().add(SLOT_HEADER + SLOT_INDEX + used), key.len());
                let p = self.shared.ptr().add(SLOT_HEADER + count * 8) as *mut u32;
                p.write_unaligned(used as u32);
                p.add(1).write_unaligned(key.len() as u32);
            }
            h.names_used.store((used + key.len()) as u32, Ordering::Relaxed);
            h.count.store(count as u32 + 1, Ordering::Relaxed);
            Some(count as u32)
        })
    }

    /// A slot's name and unit.
    pub fn entry(&self, slot: usize) -> (String, String) {
        let (off, len) = self.index_entry(slot);
        let bytes = unsafe {
            std::slice::from_raw_parts(self.shared.ptr().add(SLOT_HEADER + SLOT_INDEX + off as usize), len as usize)
        };
        let text = String::from_utf8_lossy(bytes);
        match text.split_once('\0') {
            Some((n, u)) => (n.to_string(), u.to_string()),
            None => (text.into_owned(), String::new()),
        }
    }

    pub fn values(&self) -> &mut [f64] {
        unsafe { std::slice::from_raw_parts_mut(self.shared.ptr().add(VALUES_AT) as *mut f64, SLOT_CAPACITY) }
    }

    pub fn read(&self, slot: u32) -> f64 {
        self.values().get(slot as usize).copied().unwrap_or(0.)
    }
}

// ---------------------------------------------------------------------------
// Screens: pixels from XPHFBW for one cockpit screen.
// ---------------------------------------------------------------------------

#[repr(C)]
pub struct ScreenHeader {
    pub magic: AtomicU64,
    /// Width and height in pixels (the gauge's panel.cfg size).
    pub width: AtomicU32,
    pub height: AtomicU32,
    /// Bumped by the app after each paint; the plugin uploads when it changes.
    pub frame: AtomicU64,
    /// Dirty rectangles of the last paint: x, y, w, h each.
    pub dirty_count: AtomicU32,
    pub dirty: [AtomicU32; MAX_DIRTY_RECTS * 4],
    /// 1 while the app is writing pixels (the plugin skips that frame).
    pub writing: AtomicU32,
}

const SCREEN_HEADER: usize = std::mem::size_of::<ScreenHeader>();

pub struct ScreenBlock {
    shared: Shared,
}

impl ScreenBlock {
    pub fn bytes(width: u32, height: u32) -> usize {
        SCREEN_HEADER + width as usize * height as usize * 4
    }

    /// Created by the plugin for each screen id (display::screens), sized
    /// from panel.cfg.
    pub fn create(tag: &str, screen_id: &str, width: u32, height: u32) -> Option<Self> {
        let shared = Shared::create(&object_name(tag, &format!("screen_{screen_id}")), Self::bytes(width, height))?;
        let b = Self { shared };
        let h = b.header();
        h.magic.store(MAGIC, Ordering::Relaxed);
        h.width.store(width, Ordering::Relaxed);
        h.height.store(height, Ordering::Relaxed);
        Some(b)
    }

    pub fn open(tag: &str, screen_id: &str, width: u32, height: u32) -> Option<Self> {
        let shared = Shared::open(&object_name(tag, &format!("screen_{screen_id}")), Self::bytes(width, height))?;
        let b = Self { shared };
        (b.header().magic.load(Ordering::Relaxed) == MAGIC).then_some(b)
    }

    pub fn header(&self) -> &ScreenHeader {
        unsafe { &*(self.shared.ptr() as *const ScreenHeader) }
    }

    /// BGRA, top row first, `width * height * 4` bytes.
    pub fn pixels(&self) -> &mut [u8] {
        let h = self.header();
        let n = h.width.load(Ordering::Relaxed) as usize * h.height.load(Ordering::Relaxed) as usize * 4;
        unsafe { std::slice::from_raw_parts_mut(self.shared.ptr().add(SCREEN_HEADER), n) }
    }
}

/// The session's fixed objects, as the plugin creates them.
pub struct Session {
    pub slots: SlotTable,
    pub uplink: Ring,
    pub downlinks: Vec<Ring>,
    pub input: Ring,
}

impl Session {
    pub fn create(tag: &str) -> Option<Self> {
        Some(Self {
            slots: SlotTable::create(tag)?,
            uplink: Ring::create(&object_name(tag, "uplink"), UPLINK_BYTES)?,
            downlinks: (0..MAX_VIEWS).map(|v| Ring::create(&object_name(tag, &format!("downlink_{v}")), DOWNLINK_BYTES)).collect::<Option<_>>()?,
            input: Ring::create(&object_name(tag, "input"), INPUT_BYTES)?,
        })
    }

    pub fn open(tag: &str) -> Option<Self> {
        Some(Self {
            slots: SlotTable::open(tag)?,
            uplink: Ring::open(&object_name(tag, "uplink"), UPLINK_BYTES)?,
            downlinks: (0..MAX_VIEWS).map(|v| Ring::open(&object_name(tag, &format!("downlink_{v}")), DOWNLINK_BYTES)).collect::<Option<_>>()?,
            input: Ring::open(&object_name(tag, "input"), INPUT_BYTES)?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tag() -> String {
        format!("test_{}_{}", std::process::id(), std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().subsec_nanos())
    }

    #[test]
    fn records_round_trip() {
        let ups = [
            Uplink::Write { slot: 7, value: -1.5 },
            Uplink::Event { name: "K:A32NX.FCU_AP_1_PUSH".into(), values: vec![1., 2.] },
            Uplink::SetString { name: "L:X".into(), value: "héllo".into() },
            Uplink::Call { view: 3, id: 99, name: "LOAD_AIRPORT".into(), args_json: "[\"A      EGLL\"]".into() },
            Uplink::GameString { view: 1, name: "AIRCRAFT ORIENTATION AXIS".into() },
            Uplink::Log { view: 0, level: 2, text: "boom".into() },
            Uplink::Loaded { view: 5, ok: false, text: "404".into() },
        ];
        for u in ups {
            assert_eq!(Uplink::decode(&u.encode()), Some(u));
        }
        let downs = [
            Downlink::Reply { id: 4, ok: true, text: "{}".into() },
            Downlink::HEvent { name: "A320_Neo_MFD_BTN_CSTR_1".into() },
            Downlink::ProviderEvent { name: "FACILITY_LOADED".into(), json: "{}".into() },
            Downlink::GameString { name: "X".into(), value: "Y".into() },
            Downlink::StringValue { name: "L:S".into(), value: "".into() },
        ];
        for d in downs {
            assert_eq!(Downlink::decode(&d.encode()), Some(d));
        }
        let i = Input { screen: 2, kind: InputKind::Wheel, x: 10.5, y: 3., delta: -1. };
        assert_eq!(Input::decode(&i.encode()), Some(i));
        let k = Input { screen: 6, kind: InputKind::Key, x: 'a' as u32 as f32, y: 65., delta: 0. };
        assert_eq!(Input::decode(&k.encode()), Some(k));
    }

    #[test]
    fn a_ring_carries_records_in_order_and_wraps() {
        let t = tag();
        let writer = Ring::create(&object_name(&t, "ring"), 64).unwrap();
        let reader = Ring::open(&object_name(&t, "ring"), 64).unwrap();
        for round in 0..20u8 {
            assert!(writer.push(&[round; 10]));
            assert!(writer.push(&[round + 1; 20]));
            assert_eq!(reader.drain(), vec![vec![round; 10], vec![round + 1; 20]]);
        }
        assert!(!writer.push(&[0; 61]), "too big for the ring");
        assert_eq!(writer.dropped(), 1);
    }

    #[test]
    fn slots_are_shared_between_openers_and_deduplicated() {
        let t = tag();
        let plugin = SlotTable::create(&t).unwrap();
        let view = SlotTable::open(&t).unwrap();
        let a = view.register("L:A32NX_ELEC_AC_1_BUS_IS_POWERED", "Bool").unwrap();
        let b = view.register("AIRSPEED INDICATED", "knots").unwrap();
        assert_eq!(view.register("L:A32NX_ELEC_AC_1_BUS_IS_POWERED", "Bool"), Some(a));
        assert_ne!(view.register("AIRSPEED INDICATED", "meters per second"), Some(b));
        assert_eq!(plugin.header().count.load(Ordering::Relaxed), 3);
        assert_eq!(plugin.entry(b as usize), ("AIRSPEED INDICATED".to_string(), "knots".to_string()));
        plugin.values()[b as usize] = 250.;
        assert_eq!(view.read(b), 250.);
    }

    #[test]
    fn a_screen_block_is_shared() {
        let t = tag();
        let plugin = ScreenBlock::create(&t, "SCREEN_DU_PFDL", 4, 2).unwrap();
        let app = ScreenBlock::open(&t, "SCREEN_DU_PFDL", 4, 2).unwrap();
        app.pixels()[..4].copy_from_slice(&[1, 2, 3, 4]);
        app.header().frame.fetch_add(1, Ordering::Relaxed);
        assert_eq!(&plugin.pixels()[..4], &[1, 2, 3, 4]);
        assert_eq!(plugin.header().frame.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn view_loaded_is_shared_between_the_plugin_and_the_app() {
        let t = tag();
        let plugin = Session::create(&t).unwrap();
        let app = Session::open(&t).unwrap();
        assert_eq!(app.slots.header().view_loaded[3].load(Ordering::Relaxed), 0, "not yet reported");
        plugin.slots.header().view_loaded[3].store(1, Ordering::Relaxed);
        assert_eq!(app.slots.header().view_loaded[3].load(Ordering::Relaxed), 1);
        // Every other view stays untouched.
        assert_eq!(app.slots.header().view_loaded[4].load(Ordering::Relaxed), 0);
    }

    #[test]
    fn a_session_opens_what_the_plugin_created() {
        let t = tag();
        let plugin = Session::create(&t).unwrap();
        let app = Session::open(&t).unwrap();
        app.uplink.push(&Uplink::Log { view: 1, level: 0, text: "hi".into() }.encode());
        let got: Vec<_> = plugin.uplink.drain().iter().filter_map(|r| Uplink::decode(r)).collect();
        assert_eq!(got, vec![Uplink::Log { view: 1, level: 0, text: "hi".into() }]);
        plugin.downlinks[4].push(&Downlink::HEvent { name: "H".into() }.encode());
        assert_eq!(app.downlinks[4].drain().len(), 1);
    }

    /// The regression test the EFB click investigation asked for
    /// (`docs/deep/debug_screen_clicks.md`): a synthetic tap (down, then up)
    /// on the EFB tablet, pushed onto the shared `input` ring exactly as
    /// `Displays::dispatch_pointer` does (`src/display/mod.rs`), then
    /// drained and decoded exactly as `app/src/views.rs::drain_input` does.
    /// Proves the wire format survives a real `Session::create`/`open` pair
    /// (not just `Input::encode`/`decode` in isolation, `records_round_trip`
    /// above) and that the screen index a click on `SCREEN_EFB` carries is
    /// the same index `xphfbw_bridge_views::SCREEN_ORDER` gives it — the
    /// index `app/src/views.rs::screen_slot(EFB_SCREEN)` resolves the EFB's
    /// own hand-made browser from (asserted `Some(15)` in that crate's own
    /// test) — so a future reordering of either side's screen list alone
    /// would be caught here too, not just by each crate's own ordering test.
    #[test]
    fn a_synthetic_click_on_the_efb_screen_round_trips_through_the_input_ring() {
        use crate::xphfbw_bridge_views::{EFB_HEIGHT, EFB_SCREEN, EFB_WIDTH, SCREEN_ORDER};

        let t = tag();
        let plugin = Session::create(&t).unwrap();
        let app = Session::open(&t).unwrap();

        let efb = SCREEN_ORDER.iter().position(|s| *s == EFB_SCREEN).expect("SCREEN_EFB is in SCREEN_ORDER") as u32;

        // The same sequence `Displays::mouse` produces for one tap: Down at
        // the touch point, Up at (about) the same point (a real tap is
        // rarely pixel-exact between the two).
        let down = Input { screen: efb, kind: InputKind::Down, x: 120.5, y: 40.0, delta: 0. };
        let up = Input { screen: efb, kind: InputKind::Up, x: 121.0, y: 41.0, delta: 0. };
        assert!(plugin.input.push(&down.encode()));
        assert!(plugin.input.push(&up.encode()));

        let got: Vec<Input> = app.input.drain().iter().filter_map(|r| Input::decode(r)).collect();
        assert_eq!(got, vec![down, up], "the down/up pair must survive the ring in order and unchanged");

        for input in &got {
            // Every decoded record names the EFB's own slot, not a
            // panel.cfg view's index (those are a different numbering
            // space entirely, `Uplink::Call`/`Loaded`'s `view`).
            assert_eq!(input.screen, efb);
            assert_eq!(SCREEN_ORDER[input.screen as usize], EFB_SCREEN);
            // Inside the EFB's own CSS pixel bounds (`display/screens.rs`'s
            // `SCREEN_EFB` entry, matched by `EFB_WIDTH`/`EFB_HEIGHT` here):
            // a coordinate this test pushes that fell outside them would
            // mean the two sides disagree about the tablet's size.
            assert!(input.x >= 0. && input.x < EFB_WIDTH as f32);
            assert!(input.y >= 0. && input.y < EFB_HEIGHT as f32);
        }
    }
}
