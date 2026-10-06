//! Crash reports: a Rust panic hook and a Win32 unhandled-exception filter.
//! When `xphfbw.autoCrashReport` is on (read from `xphfbw.json`, agent B's
//! app-settings key, via `settings::load`), each writes a report folder
//! `%LOCALAPPDATA%\XPHFBW\crash\<timestamp>\` with the app log tail,
//! X-Plane's `Log.txt`, the newest state dump, and a text summary
//! (docs/briefs/xphfbw-app.md, agent L scope 3). No zip: a plain folder.

use std::path::{Path, PathBuf};
use std::sync::{Arc, OnceLock};

use windows_sys::Win32::Foundation::HMODULE;
use windows_sys::Win32::System::Diagnostics::Debug::{
    SetUnhandledExceptionFilter, EXCEPTION_CONTINUE_SEARCH, EXCEPTION_EXECUTE_HANDLER, EXCEPTION_POINTERS,
};
use windows_sys::Win32::System::LibraryLoader::{
    GetModuleFileNameW, GetModuleHandleExW, GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS, GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
};

use crate::Shared;

static SHARED: OnceLock<Arc<Shared>> = OnceLock::new();

/// Installs the panic hook and the unhandled-exception filter. Call once,
/// after `Shared` is built.
pub fn install(shared: Arc<Shared>) {
    let _ = SHARED.set(shared);

    let default_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        let message = panic_message(info);
        crate::logging::log(&format!("panic: {message}"));
        write_report("Rust panic", &message);
        default_hook(info);
    }));

    unsafe {
        SetUnhandledExceptionFilter(Some(exception_filter));
    }
}

fn panic_message(info: &std::panic::PanicHookInfo) -> String {
    let payload = info.payload();
    let text = payload
        .downcast_ref::<&str>()
        .map(|s| s.to_string())
        .or_else(|| payload.downcast_ref::<String>().cloned())
        .unwrap_or_else(|| "(panic payload was not a string)".to_string());
    match info.location() {
        Some(loc) => format!("{text} at {}:{}:{}", loc.file(), loc.line(), loc.column()),
        None => text,
    }
}

/// `SetUnhandledExceptionFilter`'s callback: runs on the crashing thread,
/// already past the point where Windows would otherwise show its own
/// "stopped working" dialog. We write the report and let the process end
/// (`EXCEPTION_EXECUTE_HANDLER`); returning `EXCEPTION_CONTINUE_SEARCH`
/// would hand it back to Windows' default handling instead, which for a
/// windowed-subsystem background app is just a silent terminate anyway, so
/// we take the report-then-exit path deliberately.
unsafe extern "system" fn exception_filter(info: *const EXCEPTION_POINTERS) -> i32 {
    let (code, address) = match info.as_ref().and_then(|i| i.ExceptionRecord.as_ref()) {
        Some(record) => (record.ExceptionCode as u32, record.ExceptionAddress as usize),
        None => (0, 0),
    };
    let module = module_containing(address);
    let detail = format!("exception code=0x{code:08X} address=0x{address:016X} module={module}");
    crate::logging::log(&format!("crash: unhandled exception: {detail}"));
    write_report("Unhandled exception", &detail);
    let _ = EXCEPTION_CONTINUE_SEARCH; // kept for documentation; see the comment above
    EXCEPTION_EXECUTE_HANDLER
}

/// The file name of the module that contains `address`, or `"?"`.
fn module_containing(address: usize) -> String {
    if address == 0 {
        return "?".to_string();
    }
    unsafe {
        let mut module: HMODULE = std::ptr::null_mut();
        let flags = GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT;
        if GetModuleHandleExW(flags, address as *const u16, &mut module) == 0 {
            return "?".to_string();
        }
        let mut buf = [0u16; 260];
        let len = GetModuleFileNameW(module, buf.as_mut_ptr(), buf.len() as u32);
        if len == 0 {
            return "?".to_string();
        }
        String::from_utf16_lossy(&buf[..len as usize])
    }
}

/// The newest dump file in the newest session folder under
/// `D:\A380\fbw-build\state-dumps` (the same root `src/state_dump.rs` writes to;
/// not exported from there, so kept in sync with that literal here).
fn newest_state_dump() -> Option<(PathBuf, Vec<u8>)> {
    let root = Path::new(r"D:\A380\fbw-build\state-dumps");
    let newest_dir = std::fs::read_dir(root)
        .ok()?
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.is_dir())
        .max_by_key(|p| std::fs::metadata(p).and_then(|m| m.modified()).ok())?;
    let newest_file = std::fs::read_dir(&newest_dir)
        .ok()?
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|e| e == "tsv"))
        .max_by_key(|p| std::fs::metadata(p).and_then(|m| m.modified()).ok())?;
    let bytes = std::fs::read(&newest_file).ok()?;
    Some((newest_file, bytes))
}

/// Writes `%LOCALAPPDATA%\XPHFBW\crash\<timestamp>\` when
/// `xphfbw.autoCrashReport` is on; otherwise just logs that a crash
/// happened and a report was skipped.
fn write_report(kind: &str, detail: &str) {
    let Some(shared) = SHARED.get() else { return };

    // `settings::load` is read-only here: agent B owns the writer side
    // (app_settings.rs / app/src/settings.rs save()).
    let auto = crate::settings::load(&shared.xplane_root)
        .get("xphfbw.autoCrashReport")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    if !auto {
        crate::logging::log(&format!("crash: {kind} ({detail}) - autoCrashReport is off, no report written"));
        return;
    }

    let base = std::env::var("LOCALAPPDATA").map(PathBuf::from).unwrap_or_else(|_| std::env::temp_dir());
    let dir = base.join("XPHFBW").join("crash").join(crate::logging::filename_timestamp());
    if std::fs::create_dir_all(&dir).is_err() {
        crate::logging::log(&format!("crash: could not create report folder {}", dir.display()));
        return;
    }

    let _ = std::fs::write(dir.join("app-log-tail.txt"), crate::logging::tail(500));

    if let Ok(bytes) = std::fs::read(shared.xplane_root.join("Log.txt")) {
        let _ = std::fs::write(dir.join("Log.txt"), bytes);
    }

    if let Some((path, bytes)) = newest_state_dump() {
        let name = path.file_name().map(PathBuf::from).unwrap_or_else(|| PathBuf::from("state-dump.tsv"));
        let _ = std::fs::write(dir.join(name), bytes);
    }

    let summary = format!(
        "XPHFBW crash report\n\
         kind: {kind}\n\
         detail: {detail}\n\
         xplane root: {}\n\
         \n\
         Included, when found: app-log-tail.txt (this app's own log, last 500 lines),\n\
         Log.txt (X-Plane's, copied from --xp-root), and the newest state dump\n\
         (dump-*.tsv, from the newest session under D:\\A380\\fbw-build\\state-dumps).\n",
        shared.xplane_root.display(),
    );
    let _ = std::fs::write(dir.join("summary.txt"), summary);

    crate::logging::log(&format!("crash: report written to {}", dir.display()));
}
