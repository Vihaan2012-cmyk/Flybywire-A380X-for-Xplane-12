//! The few Windows primitives the systems bridge needs: a named shared
//! memory block, named auto-reset events, and waiting on another process.

use std::ffi::c_void;

pub type Handle = *mut c_void;

const INVALID_HANDLE_VALUE: Handle = -1isize as Handle;
const PAGE_READWRITE: u32 = 0x04;
const FILE_MAP_ALL_ACCESS: u32 = 0x000F_001F;
const EVENT_MODIFY_STATE: u32 = 0x0002;
const SYNCHRONIZE: u32 = 0x0010_0000;
pub const WAIT_OBJECT_0: u32 = 0;
pub const WAIT_TIMEOUT: u32 = 0x102;
pub const INFINITE: u32 = 0xFFFF_FFFF;

#[link(name = "kernel32")]
extern "system" {
    fn CreateFileMappingW(file: Handle, attributes: *mut c_void, protect: u32, size_high: u32, size_low: u32, name: *const u16) -> Handle;
    fn OpenFileMappingW(access: u32, inherit: i32, name: *const u16) -> Handle;
    fn MapViewOfFile(mapping: Handle, access: u32, offset_high: u32, offset_low: u32, bytes: usize) -> *mut c_void;
    fn UnmapViewOfFile(address: *const c_void) -> i32;
    fn CreateEventW(attributes: *mut c_void, manual_reset: i32, initial: i32, name: *const u16) -> Handle;
    fn OpenEventW(access: u32, inherit: i32, name: *const u16) -> Handle;
    fn SetEvent(event: Handle) -> i32;
    fn WaitForSingleObject(object: Handle, milliseconds: u32) -> u32;
    fn WaitForMultipleObjects(count: u32, objects: *const Handle, wait_all: i32, milliseconds: u32) -> u32;
    fn OpenProcess(access: u32, inherit: i32, pid: u32) -> Handle;
    fn CloseHandle(object: Handle) -> i32;
}

fn wide(s: &str) -> Vec<u16> {
    s.encode_utf16().chain(std::iter::once(0)).collect()
}

/// A named block of memory both processes map.
pub struct Shared {
    mapping: Handle,
    view: *mut u8,
    pub len: usize,
}

unsafe impl Send for Shared {}

impl Shared {
    pub fn create(name: &str, len: usize) -> Option<Self> {
        let n = wide(name);
        let mapping = unsafe {
            CreateFileMappingW(INVALID_HANDLE_VALUE, std::ptr::null_mut(), PAGE_READWRITE, (len as u64 >> 32) as u32, len as u32, n.as_ptr())
        };
        Self::map(mapping, len)
    }

    pub fn open(name: &str, len: usize) -> Option<Self> {
        let n = wide(name);
        let mapping = unsafe { OpenFileMappingW(FILE_MAP_ALL_ACCESS, 0, n.as_ptr()) };
        Self::map(mapping, len)
    }

    fn map(mapping: Handle, len: usize) -> Option<Self> {
        if mapping.is_null() {
            return None;
        }
        let view = unsafe { MapViewOfFile(mapping, FILE_MAP_ALL_ACCESS, 0, 0, len) } as *mut u8;
        if view.is_null() {
            unsafe { CloseHandle(mapping) };
            return None;
        }
        Some(Self { mapping, view, len })
    }

    pub fn ptr(&self) -> *mut u8 {
        self.view
    }
}

impl Drop for Shared {
    fn drop(&mut self) {
        unsafe {
            UnmapViewOfFile(self.view as *const c_void);
            CloseHandle(self.mapping);
        }
    }
}

/// A named auto-reset event.
pub struct Event(Handle);

unsafe impl Send for Event {}

impl Event {
    pub fn create(name: &str) -> Option<Self> {
        let n = wide(name);
        let h = unsafe { CreateEventW(std::ptr::null_mut(), 0, 0, n.as_ptr()) };
        (!h.is_null()).then_some(Self(h))
    }

    pub fn open(name: &str) -> Option<Self> {
        let n = wide(name);
        let h = unsafe { OpenEventW(EVENT_MODIFY_STATE | SYNCHRONIZE, 0, n.as_ptr()) };
        (!h.is_null()).then_some(Self(h))
    }

    pub fn set(&self) {
        unsafe { SetEvent(self.0) };
    }

    /// `WAIT_OBJECT_0` when signalled, `WAIT_TIMEOUT` otherwise.
    pub fn wait(&self, milliseconds: u32) -> u32 {
        unsafe { WaitForSingleObject(self.0, milliseconds) }
    }

    pub fn handle(&self) -> Handle {
        self.0
    }
}

impl Drop for Event {
    fn drop(&mut self) {
        unsafe { CloseHandle(self.0) };
    }
}

/// Another process, to notice when it has gone.
pub struct Process(Handle);

unsafe impl Send for Process {}

impl Process {
    pub fn open(pid: u32) -> Option<Self> {
        let h = unsafe { OpenProcess(SYNCHRONIZE, 0, pid) };
        (!h.is_null()).then_some(Self(h))
    }

    pub fn handle(&self) -> Handle {
        self.0
    }
}

impl Drop for Process {
    fn drop(&mut self) {
        unsafe { CloseHandle(self.0) };
    }
}

/// Wait for the first of `objects`; its index, or `None` on timeout/failure.
pub fn wait_any(objects: &[Handle], milliseconds: u32) -> Option<usize> {
    let r = unsafe { WaitForMultipleObjects(objects.len() as u32, objects.as_ptr(), 0, milliseconds) };
    ((r as usize) < objects.len()).then_some(r as usize)
}
