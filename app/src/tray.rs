//! A Windows notification-area (tray) icon for XPHFBW, with a menu: Show
//! XPHFBW, Open logs, Quit XPHFBW (docs/briefs/xphfbw-app.md, agent L scope
//! 1). Pure Win32 (`windows-sys`), on a message loop of its own thread —
//! not CEF, so it keeps working even before the settings page's browser
//! exists and doesn't compete with CEF's own message pump.
//!
//! Left click, or "Show XPHFBW": sets `Shared::show_requested`, exactly the
//! flag `window.rs`'s `poll_show_requests` already watches to open or raise
//! the settings window.
//!
//! "Quit XPHFBW" ends the app only when the systems thread is not currently
//! serving X-Plane (`Status::systems_running`); otherwise it brings the
//! settings window up instead (same `show_requested` flag), where the
//! "X-Plane link"/systems status the page already shows explains why — we
//! don't quit out from under a live X-Plane session.

use std::sync::atomic::Ordering;
use std::sync::{Arc, OnceLock};

use windows_sys::Win32::Foundation::{HWND, LPARAM, LRESULT, POINT, WPARAM};
use windows_sys::Win32::System::LibraryLoader::GetModuleHandleW;
use windows_sys::Win32::UI::Shell::{Shell_NotifyIconW, NIF_ICON, NIF_MESSAGE, NIF_TIP, NIM_ADD, NIM_DELETE, NOTIFYICONDATAW};
use windows_sys::Win32::UI::WindowsAndMessaging::{
    AppendMenuW, CreatePopupMenu, CreateWindowExW, DefWindowProcW, DestroyMenu, GetCursorPos, GetMessageW, LoadIconW, PostQuitMessage,
    RegisterClassExW, SetForegroundWindow, TrackPopupMenu, TranslateMessage, DispatchMessageW, HWND_MESSAGE, MF_STRING, MSG,
    TPM_BOTTOMALIGN, TPM_LEFTALIGN, TPM_RIGHTBUTTON, WM_APP, WM_COMMAND, WM_DESTROY, WM_LBUTTONUP, WM_RBUTTONUP, WNDCLASSEXW,
};

use crate::Shared;

// ---------------------------------------------------------------------------
// The window/app icon CEF itself can show (agent L scope: "set the window
// icon if CEF views allow it"). It does: `ImplWindow::set_window_icon` /
// `set_window_app_icon`, fed a `cef::Image` built from PNG bytes via
// `image_create()` + `ImplImage::add_png`. `window.rs` (owned by another
// agent) is where `on_window_created` builds the window, so wiring this in
// is one line for whoever owns that file — see `app_icon_image`'s doc
// comment for the exact call.
// ---------------------------------------------------------------------------

/// The same navy-roundel/white-"X" app icon `build.rs` embeds as the exe's
/// resource, built again here as PNG bytes for CEF's `Image` (a build
/// script can't share code with the crate it builds, so this duplicates
/// `build.rs`'s small pixel generator rather than depending on it).
///
/// Not wired up: call this from `window.rs`'s `on_window_created`, after
/// `window.set_title(...)`:
/// ```ignore
/// if let Some(mut icon) = tray::app_icon_image() {
///     window.set_window_icon(Some(&mut icon));
///     window.set_window_app_icon(Some(&mut icon));
/// }
/// ```
///
/// `#[allow(dead_code)]`: unused until `window.rs` wires the call above in;
/// remove once it does.
#[allow(dead_code)]
pub fn app_icon_image() -> Option<cef::Image> {
    use cef::ImplImage;
    let png = encode_png(32, &icon_rgba(32));
    let image = cef::image_create()?;
    if image.add_png(1.0, Some(&png)) == 0 {
        return None;
    }
    Some(image)
}

/// 32x32 RGBA pixels: a white "X" stroke inside a navy roundel, transparent
/// outside it. Matches `build.rs`'s `build_icon` (BGRA there, for the ICO
/// container; RGBA here, for PNG).
fn icon_rgba(size: usize) -> Vec<u8> {
    const NAVY: [u8; 3] = [0x0b, 0x2b, 0x66]; // R, G, B
    let mut out = vec![0u8; size * size * 4];
    for y in 0..size {
        for x in 0..size {
            let center = (size as f32 - 1.0) / 2.0;
            let (dx, dy) = (x as f32 - center, y as f32 - center);
            let in_roundel = (dx * dx + dy * dy).sqrt() <= size as f32 / 2.0 - 0.5;
            let on_stroke = {
                let d1 = (x as i32 - y as i32).abs();
                let d2 = (x as i32 - (size as i32 - 1 - y as i32)).abs();
                d1 <= 2 || d2 <= 2
            };
            let i = (y * size + x) * 4;
            let rgba = if !in_roundel {
                [0, 0, 0, 0]
            } else if on_stroke {
                [255, 255, 255, 255]
            } else {
                [NAVY[0], NAVY[1], NAVY[2], 255]
            };
            out[i..i + 4].copy_from_slice(&rgba);
        }
    }
    out
}

/// A minimal PNG encoder (uncompressed "stored" deflate blocks) so the icon
/// needs no image/zlib crate: just the IHDR/IDAT/IEND chunks a decoder
/// requires, each framed with the length/CRC-32 PNG demands, wrapping a
/// zlib stream with its own Adler-32 trailer.
fn encode_png(size: u32, rgba: &[u8]) -> Vec<u8> {
    let mut png = Vec::new();
    png.extend_from_slice(&[137, 80, 78, 71, 13, 10, 26, 10]);

    let mut ihdr = Vec::new();
    ihdr.extend_from_slice(&size.to_be_bytes());
    ihdr.extend_from_slice(&size.to_be_bytes());
    ihdr.extend_from_slice(&[8, 6, 0, 0, 0]); // 8-bit depth, color type 6 = RGBA
    png_chunk(&mut png, b"IHDR", &ihdr);

    let stride = size as usize * 4;
    let mut raw = Vec::with_capacity((1 + stride) * size as usize);
    for row in 0..size as usize {
        raw.push(0); // filter type 0: none
        raw.extend_from_slice(&rgba[row * stride..(row + 1) * stride]);
    }

    let mut zlib = Vec::with_capacity(raw.len() + 16);
    zlib.extend_from_slice(&[0x78, 0x01]); // zlib header (32K window, valid checksum)
    let mut offset = 0;
    loop {
        let end = (offset + 65535).min(raw.len());
        let block = &raw[offset..end];
        let is_final = end == raw.len();
        zlib.push(is_final as u8); // BFINAL in bit 0, BTYPE=00 (stored) in bits 1-2
        zlib.extend_from_slice(&(block.len() as u16).to_le_bytes());
        zlib.extend_from_slice(&(!(block.len() as u16)).to_le_bytes());
        zlib.extend_from_slice(block);
        offset = end;
        if is_final {
            break;
        }
    }
    zlib.extend_from_slice(&adler32(&raw).to_be_bytes());
    png_chunk(&mut png, b"IDAT", &zlib);
    png_chunk(&mut png, b"IEND", &[]);
    png
}

fn png_chunk(out: &mut Vec<u8>, kind: &[u8; 4], data: &[u8]) {
    out.extend_from_slice(&(data.len() as u32).to_be_bytes());
    let mut crc_input = Vec::with_capacity(4 + data.len());
    crc_input.extend_from_slice(kind);
    crc_input.extend_from_slice(data);
    out.extend_from_slice(&crc_input);
    out.extend_from_slice(&crc32(&crc_input).to_be_bytes());
}

fn crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

fn adler32(data: &[u8]) -> u32 {
    const MODULO: u32 = 65521;
    let (mut a, mut b) = (1u32, 0u32);
    for &byte in data {
        a = (a + byte as u32) % MODULO;
        b = (b + a) % MODULO;
    }
    (b << 16) | a
}

/// The mouse/keyboard message `Shell_NotifyIconW` calls our window with
/// (legacy behaviour: `lParam` is the raw mouse message, e.g. `WM_RBUTTONUP`).
const WM_TRAYICON: u32 = WM_APP + 1;
const ID_SHOW: usize = 1;
const ID_OPEN_LOGS: usize = 2;
const ID_QUIT: usize = 3;
/// Resource id 1: winresource's default application icon id (`build.rs`
/// embeds the generated .ico there), which `LoadIconW` reads back here.
const APP_ICON_ID: *const u16 = 1usize as *const u16;

static SHARED: OnceLock<Arc<Shared>> = OnceLock::new();

/// Starts the tray icon on a thread of its own. Call once, after `Shared`
/// is built.
pub fn start(shared: Arc<Shared>) {
    let _ = SHARED.set(shared);
    let _ = std::thread::Builder::new().name("tray".into()).spawn(run);
}

fn wide(s: &str) -> Vec<u16> {
    s.encode_utf16().chain(std::iter::once(0)).collect()
}

fn run() {
    unsafe {
        let hinstance = GetModuleHandleW(std::ptr::null());
        let class_name = wide("XphfbwTrayWindow");
        let wc = WNDCLASSEXW {
            cbSize: std::mem::size_of::<WNDCLASSEXW>() as u32,
            style: 0,
            lpfnWndProc: Some(wndproc),
            cbClsExtra: 0,
            cbWndExtra: 0,
            hInstance: hinstance,
            hIcon: std::ptr::null_mut(),
            hCursor: std::ptr::null_mut(),
            hbrBackground: std::ptr::null_mut(),
            lpszMenuName: std::ptr::null(),
            lpszClassName: class_name.as_ptr(),
            hIconSm: std::ptr::null_mut(),
        };
        if RegisterClassExW(&wc) == 0 {
            crate::logging::log("tray: RegisterClassExW failed, no tray icon");
            return;
        }

        // A message-only window: it never appears, it only receives the
        // notification-icon callback and menu commands.
        let hwnd = CreateWindowExW(
            0,
            class_name.as_ptr(),
            wide("XPHFBW tray").as_ptr(),
            0,
            0,
            0,
            0,
            0,
            HWND_MESSAGE,
            std::ptr::null_mut(),
            hinstance,
            std::ptr::null(),
        );
        if hwnd.is_null() {
            crate::logging::log("tray: CreateWindowExW failed, no tray icon");
            return;
        }

        // Falls back to no icon (Shell_NotifyIconW still adds a blank
        // slot) if build.rs's resource didn't get embedded, e.g. a dev
        // build without the Windows SDK's rc.exe.
        let icon = LoadIconW(hinstance, APP_ICON_ID);

        let mut nid: NOTIFYICONDATAW = std::mem::zeroed();
        nid.cbSize = std::mem::size_of::<NOTIFYICONDATAW>() as u32;
        nid.hWnd = hwnd;
        nid.uID = 1;
        nid.uFlags = NIF_ICON | NIF_MESSAGE | NIF_TIP;
        nid.uCallbackMessage = WM_TRAYICON;
        nid.hIcon = icon;
        let tip = wide("XPHFBW");
        let n = tip.len().min(nid.szTip.len());
        nid.szTip[..n].copy_from_slice(&tip[..n]);
        if Shell_NotifyIconW(NIM_ADD, &nid) == 0 {
            crate::logging::log("tray: Shell_NotifyIconW(NIM_ADD) failed, no tray icon");
            return;
        }
        crate::logging::log("tray: icon added");

        let mut msg: MSG = std::mem::zeroed();
        while GetMessageW(&mut msg, std::ptr::null_mut(), 0, 0) > 0 {
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
        Shell_NotifyIconW(NIM_DELETE, &nid);
    }
}

unsafe extern "system" fn wndproc(hwnd: HWND, msg: u32, wparam: WPARAM, lparam: LPARAM) -> LRESULT {
    match msg {
        WM_TRAYICON => {
            match lparam as u32 {
                WM_LBUTTONUP => handle_command(ID_SHOW),
                WM_RBUTTONUP => show_menu(hwnd),
                _ => {}
            }
            0
        }
        WM_COMMAND => {
            handle_command(wparam & 0xFFFF);
            0
        }
        WM_DESTROY => {
            PostQuitMessage(0);
            0
        }
        _ => DefWindowProcW(hwnd, msg, wparam, lparam),
    }
}

unsafe fn show_menu(hwnd: HWND) {
    let hmenu = CreatePopupMenu();
    if hmenu.is_null() {
        return;
    }
    AppendMenuW(hmenu, MF_STRING, ID_SHOW, wide("Show XPHFBW").as_ptr());
    AppendMenuW(hmenu, MF_STRING, ID_OPEN_LOGS, wide("Open logs").as_ptr());
    AppendMenuW(hmenu, MF_STRING, ID_QUIT, wide("Quit XPHFBW").as_ptr());

    let mut pt: POINT = std::mem::zeroed();
    GetCursorPos(&mut pt);
    // Required so the popup menu dismisses itself on an outside click
    // instead of staying stuck open (the standard tray-icon dance).
    SetForegroundWindow(hwnd);
    TrackPopupMenu(hmenu, TPM_RIGHTBUTTON | TPM_BOTTOMALIGN | TPM_LEFTALIGN, pt.x, pt.y, 0, hwnd, std::ptr::null());
    DestroyMenu(hmenu);
}

fn handle_command(id: usize) {
    let Some(shared) = SHARED.get() else { return };
    match id {
        ID_SHOW => {
            shared.show_requested.store(true, Ordering::Relaxed);
            crate::logging::log("tray: Show XPHFBW requested");
        }
        ID_OPEN_LOGS => {
            let _ = std::process::Command::new("explorer").arg(crate::logging::dir()).spawn();
            crate::logging::log("tray: Open logs requested");
        }
        ID_QUIT => {
            let serving = shared.status.lock().map(|s| s.systems_running).unwrap_or(false);
            if serving {
                crate::logging::log("tray: Quit requested while serving X-Plane; showing the window instead of quitting");
                shared.show_requested.store(true, Ordering::Relaxed);
            } else {
                crate::logging::log("tray: Quit XPHFBW");
                crate::window::quit_from_any_thread();
            }
        }
        _ => {}
    }
}
