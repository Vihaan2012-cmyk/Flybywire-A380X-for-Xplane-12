//! The one place this plugin talks to the internet.
//!
//! FlyByWire's own code already knows how to fetch a SimBrief OFP and fly it:
//! `simbriefParser.ts`'s `getSimBriefOfp` reads the user id this plugin
//! already stores (`CONFIG_OVERRIDE_SIMBRIEF_USERID`, the app's Settings
//! tab writes it), and the A380's MFD turns it into a flight plan by itself
//! -- `FlightManagementComputer::cpnyFplnRequest` downloads the OFP and hands
//! it to `SimBriefUplinkAdapter::uplinkFlightPlanFromSimbrief`, which is what
//! the FMS INIT page's CPNY F-PLN REQUEST button calls. None of that needed
//! writing here.
//!
//! What stopped it was this side: `js_bridge`'s `fetch` answers SimBridge's
//! terrain API and Navigraph's AMDB out of local data and returns 404 for
//! every other URL, so the request never left the machine. This module is
//! the missing half -- a real HTTPS GET/POST, so a script asking for a URL
//! nothing local owns gets the actual answer.
//!
//! **WinHTTP, not an HTTP crate.** A TLS stack in Rust would be a large new
//! dependency tree for one request an hour; Windows already has one, and
//! this plugin already reaches for system DLLs by name
//! (`Xplm::xplm_symbol`). WinHTTP also brings the user's proxy settings and
//! redirect handling with it, which matters because SimBrief's API answers
//! over http with a redirect to https.
//!
//! **Never on the simulator's thread.** Every request runs on its own
//! thread; the caller polls [`poll`], which answers `None` until the reply
//! is in. That suits the runtime's `CallReply::Pending` exactly: pending
//! means "ask again next tick", so the script's promise stays unresolved and
//! the frame is never blocked on a network round trip.

use std::collections::HashMap;
use std::ffi::{c_void, OsStr};
use std::os::windows::ffi::OsStrExt;
use std::sync::Mutex;

/// A finished request: the HTTP status and the body as text.
pub type Response = (u16, String);

enum State {
    InFlight,
    Done(Response),
}

static REQUESTS: Mutex<Option<HashMap<String, State>>> = Mutex::new(None);

/// How many bodies to keep. A request is removed the moment it is read, so
/// this only bounds what an abandoned promise can leave behind.
const MAX_TRACKED: usize = 64;

/// Ask for `url`, and keep asking. The first call starts the request and
/// answers `None`; later calls answer `None` until it finishes, then the
/// response once, after which the request is forgotten and a further call
/// starts a fresh one.
///
/// `key` identifies the request across those repeated calls. The runtime
/// re-issues a pending call with the same method, url and body, so that
/// triple is what the caller passes.
pub fn poll(key: &str, method: &str, url: &str, body: &str) -> Option<Response> {
    let mut guard = REQUESTS.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
    let map = guard.get_or_insert_with(HashMap::new);
    match map.get(key) {
        Some(State::Done(_)) => match map.remove(key) {
            Some(State::Done(r)) => Some(r),
            _ => None,
        },
        Some(State::InFlight) => None,
        None => {
            if map.len() >= MAX_TRACKED {
                // Nothing is reading these any more; a page was closed or a
                // promise dropped. Keeping them would leak.
                map.retain(|_, s| matches!(s, State::InFlight));
            }
            map.insert(key.to_string(), State::InFlight);
            let (method, url, body) = (method.to_string(), url.to_string(), body.to_string());
            let done_key = key.to_string();
            let spawned = std::thread::Builder::new().name("fbw-net".into()).spawn(move || {
                let result = request(&method, &url, &body).unwrap_or((0, String::new()));
                let mut guard = REQUESTS.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
                if let Some(map) = guard.as_mut() {
                    map.insert(done_key, State::Done(result));
                }
            });
            if spawned.is_err() {
                map.remove(key);
            }
            None
        }
    }
}

// ---------------------------------------------------------------------------
// WinHTTP
// ---------------------------------------------------------------------------

type Handle = *mut c_void;

const ACCESS_TYPE_AUTOMATIC_PROXY: u32 = 4;
const FLAG_SECURE: u32 = 0x0080_0000;
const HTTPS_PORT: u16 = 443;
const HTTP_PORT: u16 = 80;
const QUERY_STATUS_CODE: u32 = 19;
const QUERY_FLAG_NUMBER: u32 = 0x2000_0000;
const HEADER_INDEX_NONE: u32 = 0;

#[link(name = "kernel32")]
extern "system" {
    fn LoadLibraryA(name: *const i8) -> *mut c_void;
    fn GetProcAddress(module: *mut c_void, name: *const i8) -> *mut c_void;
}

/// One WinHTTP entry point, looked up by name the same way the XPLM ones
/// are. Missing WinHTTP is not an error worth panicking over -- the request
/// simply cannot be made.
fn symbol(name: &str) -> Option<*mut c_void> {
    unsafe {
        let dll = std::ffi::CString::new("winhttp.dll").ok()?;
        let module = LoadLibraryA(dll.as_ptr());
        if module.is_null() {
            return None;
        }
        let name = std::ffi::CString::new(name).ok()?;
        let p = GetProcAddress(module, name.as_ptr());
        (!p.is_null()).then_some(p)
    }
}

macro_rules! winhttp {
    ($name:literal, $t:ty) => {
        match symbol($name) {
            Some(p) => unsafe { std::mem::transmute::<*mut c_void, $t>(p) },
            None => return None,
        }
    };
}

fn wide(s: &str) -> Vec<u16> {
    OsStr::new(s).encode_wide().chain(std::iter::once(0)).collect()
}

/// `https://host/path?query` split into (secure, host, path-with-query).
/// Anything without a host is not a request this can make.
fn split_url(url: &str) -> Option<(bool, String, String)> {
    let (scheme, rest) = url.split_once("://")?;
    let secure = match scheme.to_ascii_lowercase().as_str() {
        "https" => true,
        "http" => false,
        _ => return None,
    };
    let (host, path) = match rest.find('/') {
        Some(i) => (&rest[..i], &rest[i..]),
        None => (rest, "/"),
    };
    (!host.is_empty()).then(|| (secure, host.to_string(), path.to_string()))
}

/// A closing handle guard: WinHTTP handles must be closed on every path out
/// of [`request`], including the early returns its error handling takes.
struct Owned(Handle, unsafe extern "system" fn(Handle) -> i32);

impl Drop for Owned {
    fn drop(&mut self) {
        if !self.0.is_null() {
            unsafe { (self.1)(self.0) };
        }
    }
}

/// One blocking request. Runs on its own thread ([`poll`]), never the
/// simulator's.
fn request(method: &str, url: &str, body: &str) -> Option<Response> {
    type Open = unsafe extern "system" fn(*const u16, u32, *const u16, *const u16, u32) -> Handle;
    type Connect = unsafe extern "system" fn(Handle, *const u16, u16, u32) -> Handle;
    type OpenRequest = unsafe extern "system" fn(Handle, *const u16, *const u16, *const u16, *const u16, *const *const u16, u32) -> Handle;
    type SendRequest = unsafe extern "system" fn(Handle, *const u16, u32, *const c_void, u32, u32, usize) -> i32;
    type ReceiveResponse = unsafe extern "system" fn(Handle, *mut c_void) -> i32;
    type QueryHeaders = unsafe extern "system" fn(Handle, u32, *const u16, *mut c_void, *mut u32, *mut u32) -> i32;
    type ReadData = unsafe extern "system" fn(Handle, *mut c_void, u32, *mut u32) -> i32;
    type CloseHandle = unsafe extern "system" fn(Handle) -> i32;

    let open = winhttp!("WinHttpOpen", Open);
    let connect = winhttp!("WinHttpConnect", Connect);
    let open_request = winhttp!("WinHttpOpenRequest", OpenRequest);
    let send = winhttp!("WinHttpSendRequest", SendRequest);
    let receive = winhttp!("WinHttpReceiveResponse", ReceiveResponse);
    let query = winhttp!("WinHttpQueryHeaders", QueryHeaders);
    let read = winhttp!("WinHttpReadData", ReadData);
    let close = winhttp!("WinHttpCloseHandle", CloseHandle);

    let (secure, host, path) = split_url(url)?;

    unsafe {
        let agent = wide("FlyByWire A380X (X-Plane)");
        let session = Owned(open(agent.as_ptr(), ACCESS_TYPE_AUTOMATIC_PROXY, std::ptr::null(), std::ptr::null(), 0), close);
        if session.0.is_null() {
            return None;
        }
        let host_w = wide(&host);
        let port = if secure { HTTPS_PORT } else { HTTP_PORT };
        let conn = Owned(connect(session.0, host_w.as_ptr(), port, 0), close);
        if conn.0.is_null() {
            return None;
        }
        let verb = wide(method);
        let path_w = wide(&path);
        let flags = if secure { FLAG_SECURE } else { 0 };
        let req = Owned(
            open_request(conn.0, verb.as_ptr(), path_w.as_ptr(), std::ptr::null(), std::ptr::null(), std::ptr::null(), flags),
            close,
        );
        if req.0.is_null() {
            return None;
        }

        // A body only goes with the verbs that carry one; SimBrief's is a
        // plain GET.
        let (data, len) = if body.is_empty() { (std::ptr::null(), 0) } else { (body.as_ptr().cast::<c_void>(), body.len() as u32) };
        let headers = wide("Content-Type: application/json\r\n");
        let (headers_ptr, headers_len) = if body.is_empty() { (std::ptr::null(), 0) } else { (headers.as_ptr(), u32::MAX) };
        if send(req.0, headers_ptr, headers_len, data, len, len, 0) == 0 {
            return None;
        }
        if receive(req.0, std::ptr::null_mut()) == 0 {
            return None;
        }

        let mut status: u32 = 0;
        let mut size = std::mem::size_of::<u32>() as u32;
        let mut index = HEADER_INDEX_NONE;
        let got_status = query(
            req.0,
            QUERY_STATUS_CODE | QUERY_FLAG_NUMBER,
            std::ptr::null(),
            (&mut status as *mut u32).cast(),
            &mut size,
            &mut index,
        ) != 0;
        if !got_status {
            return None;
        }

        let mut out: Vec<u8> = Vec::new();
        let mut buffer = [0u8; 16 << 10];
        loop {
            let mut read_now: u32 = 0;
            if read(req.0, buffer.as_mut_ptr().cast(), buffer.len() as u32, &mut read_now) == 0 {
                return None;
            }
            if read_now == 0 {
                break;
            }
            out.extend_from_slice(&buffer[..read_now as usize]);
            // A runaway body must not eat the machine's memory: an OFP is a
            // few hundred kilobytes, and nothing this plugin asks for is
            // anywhere near this.
            if out.len() > 32 << 20 {
                break;
            }
        }
        Some((status as u16, String::from_utf8_lossy(&out).into_owned()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_url_splits_into_what_winhttp_needs() {
        assert_eq!(
            split_url("https://www.simbrief.com/api/xml.fetcher.php?userid=1&json=v2"),
            Some((true, "www.simbrief.com".into(), "/api/xml.fetcher.php?userid=1&json=v2".into()))
        );
        // No path at all still asks for the root.
        assert_eq!(split_url("http://example.com"), Some((false, "example.com".into(), "/".into())));
        // Not something this can fetch.
        assert_eq!(split_url("file:///C:/secrets.txt"), None);
        assert_eq!(split_url("/api/terrain"), None);
        assert_eq!(split_url("https://"), None);
    }

    /// The first ask starts the request and never blocks; it must not return
    /// a body it cannot have yet.
    #[test]
    fn the_first_poll_answers_nothing_and_does_not_block() {
        let key = "test-key-that-no-request-uses";
        // An address nothing answers, so the worker fails rather than
        // reaching the network from a unit test.
        assert_eq!(poll(key, "GET", "https://127.0.0.1:9/nothing", ""), None);
        assert_eq!(poll(key, "GET", "https://127.0.0.1:9/nothing", ""), None, "still in flight, still nothing");
        let mut guard = REQUESTS.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
        if let Some(map) = guard.as_mut() {
            map.remove(key);
        }
    }
}
