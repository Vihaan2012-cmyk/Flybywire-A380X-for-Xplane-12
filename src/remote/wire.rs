//! The shared memory layout between the plugin and the systems process.
//!
//! ```text
//! [Header][names: NAMES_BYTES][values: f64 x CAPACITY][written: u64 x CAPACITY/64]
//! ```
//!
//! Only one side touches the block at a time: the plugin between a reply and
//! its next request, the systems process between a request and its reply.

use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

pub const MAGIC: u64 = 0x4642_5733_3830_5359; // "FBW380SY"
pub const VERSION: u32 = 1;
pub const CAPACITY: usize = 32_768;
pub const NAMES_BYTES: usize = 4 << 20;
pub const MAX_FAILURES: usize = 1024;

pub const STATE_STARTING: u32 = 0;
pub const STATE_READY: u32 = 1;
pub const STATE_FAILED: u32 = 2;
pub const STATE_QUIT: u32 = 3;

#[repr(C)]
pub struct Header {
    pub magic: AtomicU64,
    pub version: AtomicU32,
    pub state: AtomicU32,
    /// FlyByWire's START_STATE number (1 hangar ... 8 final).
    pub start_state: AtomicU32,
    /// Variables the systems registered, in registration order.
    pub count: AtomicU32,
    /// Bytes of the names block in use.
    pub names_len: AtomicU32,
    pub _pad: AtomicU32,
    pub request: AtomicU64,
    pub reply: AtomicU64,
    /// f64 bits.
    pub delta: AtomicU64,
    pub time: AtomicU64,
    /// Microseconds the last tick took inside the systems process.
    pub tick_micros: AtomicU64,
    /// 1 when `failures` holds a new active set.
    pub failures_dirty: AtomicU32,
    pub failures_len: AtomicU32,
    /// Indices into the failure catalogue (`failures::catalogue_types`).
    pub failures: [AtomicU32; MAX_FAILURES],
    pub message_len: AtomicU32,
    pub message: [u8; 1024],
}

pub const HEADER_BYTES: usize = std::mem::size_of::<Header>();
pub const VALUES_OFFSET: usize = HEADER_BYTES + NAMES_BYTES;
pub const WRITTEN_OFFSET: usize = VALUES_OFFSET + CAPACITY * 8;
pub const TOTAL_BYTES: usize = WRITTEN_OFFSET + CAPACITY.div_ceil(64) * 8;

/// A view of the block, laid out as above.
pub struct Block {
    base: *mut u8,
}

unsafe impl Send for Block {}

impl Block {
    /// # Safety
    /// `base` points at a mapping of at least [`TOTAL_BYTES`] bytes that
    /// lives as long as the returned view.
    pub unsafe fn new(base: *mut u8) -> Self {
        Self { base }
    }

    pub fn header(&self) -> &Header {
        unsafe { &*(self.base as *const Header) }
    }

    pub fn names_mut(&mut self) -> &mut [u8] {
        unsafe { std::slice::from_raw_parts_mut(self.base.add(HEADER_BYTES), NAMES_BYTES) }
    }

    pub fn names(&self) -> &[u8] {
        let len = (self.header().names_len.load(Ordering::Relaxed) as usize).min(NAMES_BYTES);
        unsafe { std::slice::from_raw_parts(self.base.add(HEADER_BYTES), len) }
    }

    pub fn values(&self) -> &mut [f64] {
        unsafe { std::slice::from_raw_parts_mut(self.base.add(VALUES_OFFSET) as *mut f64, CAPACITY) }
    }

    pub fn written(&self) -> &mut [u64] {
        unsafe { std::slice::from_raw_parts_mut(self.base.add(WRITTEN_OFFSET) as *mut u64, CAPACITY.div_ceil(64)) }
    }

    pub fn set_message(&mut self, text: &str) {
        let bytes = text.as_bytes();
        let n = bytes.len().min(1024);
        let h = unsafe { &mut *(self.base as *mut Header) };
        h.message[..n].copy_from_slice(&bytes[..n]);
        h.message_len.store(n as u32, Ordering::Relaxed);
    }

    pub fn message(&self) -> String {
        let h = self.header();
        let n = (h.message_len.load(Ordering::Relaxed) as usize).min(1024);
        String::from_utf8_lossy(&h.message[..n]).into_owned()
    }
}

pub fn f64_to_bits(v: f64) -> u64 {
    v.to_bits()
}

pub fn bits_to_f64(v: u64) -> f64 {
    f64::from_bits(v)
}

/// The object names for one connection.
pub fn names(tag: &str) -> (String, String, String) {
    (format!("Local\\FbwA380Systems_{tag}_shm"), format!("Local\\FbwA380Systems_{tag}_req"), format!("Local\\FbwA380Systems_{tag}_rep"))
}

/// A registration record: `g` for a prefixed (`get`) name, `u` for
/// `get_unprefixed`, then the name, one per line.
pub fn encode_names(names: &[(bool, String)]) -> Vec<u8> {
    let mut out = Vec::new();
    for (unprefixed, name) in names {
        out.push(if *unprefixed { b'u' } else { b'g' });
        out.extend_from_slice(name.as_bytes());
        out.push(b'\n');
    }
    out
}

pub fn decode_names(bytes: &[u8]) -> Vec<(bool, String)> {
    String::from_utf8_lossy(bytes)
        .lines()
        .filter(|l| !l.is_empty())
        .map(|l| (l.starts_with('u'), l[1..].to_string()))
        .collect()
}
