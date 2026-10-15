//! The systems panel: the simulation's state, served to anything that asks.
//!
//! X-Plane's own windows are a poor place to read a hundred system states, and
//! a panel outside the sim can live on a second screen or a tablet. So the
//! plugin answers on a local socket: `/vars` hands over every variable as
//! JSON, `/` serves the page that reads it.
//!
//! The same socket also serves the XPHFBW app's Study tab
//! (docs/briefs/xphfbw-app.md): `/study/pages`, `/study/failures` and
//! `/study/breakers` are read-only JSON built by `study::web` from the exact
//! Rust the XPLM Study windows draw from, and `POST /study/action` applies a
//! click the same way a Study window's click does, onto the plugin's own
//! request queues (`study::web::apply_action`'s own doc comment lists them).
//! Those queues are drained once per flight loop tick, so a write from this
//! thread can never race the systems tick that reads them.
//!
//! Nothing here touches X-Plane's API directly; it only reads the snapshot
//! the flight loop leaves behind and pushes onto queues already built for
//! cross-thread use, so it is safe on its own thread.

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

static RUNNING: AtomicBool = AtomicBool::new(false);

/// The page, built into the plugin so there is nothing to install.
const PAGE: &str = include_str!("panel.html");

/// Serve the panel on `127.0.0.1:<port>` until [`stop`] is called.
pub fn start(port: u16) -> std::io::Result<u16> {
    let listener = TcpListener::bind(("127.0.0.1", port))?;
    listener.set_nonblocking(true)?;
    let port = listener.local_addr()?.port();
    RUNNING.store(true, Ordering::Relaxed);
    std::thread::Builder::new()
        .name("fbw-panel".into())
        .spawn(move || {
            while RUNNING.load(Ordering::Relaxed) {
                match listener.accept() {
                    Ok((stream, _)) => {
                        let _ = std::thread::Builder::new().name("fbw-panel-request".into()).spawn(move || {
                            let _ = serve(stream);
                        });
                    }
                    // Nothing waiting: idle rather than spin.
                    Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                        std::thread::sleep(Duration::from_millis(40));
                    }
                    Err(_) => break,
                }
            }
        })?;
    Ok(port)
}

pub fn stop() {
    RUNNING.store(false, Ordering::Relaxed);
}

fn serve(mut stream: TcpStream) -> std::io::Result<()> {
    stream.set_read_timeout(Some(Duration::from_millis(250)))?;
    stream.set_nodelay(true)?;
    let mut buf = vec![0u8; 4096];
    let mut total = 0;
    // Read until the header block is in, growing the buffer for a body
    // longer than the first read (an action's JSON is always small, but a
    // slow client can still split it across reads).
    let header_end = loop {
        let read = stream.read(&mut buf[total..]).unwrap_or(0);
        if read == 0 {
            break None;
        }
        total += read;
        if let Some(at) = find(&buf[..total], b"\r\n\r\n") {
            break Some(at);
        }
        if total == buf.len() {
            buf.resize(buf.len() * 2, 0);
        }
    };
    let Some(header_end) = header_end else {
        return reply(&mut stream, 200, "text/html; charset=utf-8", PAGE.as_bytes());
    };
    let head = String::from_utf8_lossy(&buf[..header_end]).into_owned();
    let mut lines = head.lines();
    let request_line = lines.next().unwrap_or("");
    let mut parts = request_line.split_whitespace();
    let method = parts.next().unwrap_or("GET");
    let target = parts.next().unwrap_or("/");
    let (path, query) = target.split_once('?').unwrap_or((target, ""));
    let content_length: usize = lines
        .find_map(|l| l.to_ascii_lowercase().starts_with("content-length:").then(|| l["content-length:".len()..].trim().parse().ok()).flatten())
        .unwrap_or(0);

    if method.eq_ignore_ascii_case("OPTIONS") {
        return reply_headers(&mut stream, 204, "text/plain", b"");
    }

    let body_start = header_end + 4;
    let mut body = buf[body_start..total].to_vec();
    while body.len() < content_length {
        let mut chunk = [0u8; 4096];
        match stream.read(&mut chunk) {
            Ok(0) | Err(_) => break,
            Ok(n) => body.extend_from_slice(&chunk[..n]),
        }
    }
    let body = String::from_utf8_lossy(&body);

    #[cfg(feature = "js")]
    if path == "/keyboard/efb" {
        crate::display::set_efb_text_focus(body.trim() == "1");
        return reply(&mut stream, 200, "application/json", b"{}");
    }

    if path.starts_with("/diag/") {
        let (status, text) = diag_route(path, &body);
        return reply(&mut stream, status, "application/json", text.as_bytes());
    }

    if method.eq_ignore_ascii_case("POST") && path == "/study/action" {
        crate::diag::event("efb-action", &body);
        return match crate::study::web::apply_action(&body) {
            Ok(()) => reply(&mut stream, 200, "application/json", b"{\"ok\":true}"),
            Err(message) => reply(&mut stream, 400, "application/json", format!("{{\"ok\":false,\"message\":{}}}", json_string(&message)).as_bytes()),
        };
    }

    #[cfg(feature = "js")]
    if let Some((status, kind, text)) = network_route(method, path, query, &body) {
        return reply(&mut stream, status, kind, text.as_bytes());
    }

    match path {
        "/vars" => reply(&mut stream, 200, "application/json", variables_as_json(names_filter(query)).as_bytes()),
        "/efb/state" => reply(&mut stream, 200, "application/json", efb_state_json().as_bytes()),
        "/throttle/state" => reply(&mut stream, 200, "application/json", crate::throttle::state_json().as_bytes()),
        "/study/pages" => reply(&mut stream, 200, "application/json", crate::study::web::pages_json().as_bytes()),
        "/study/failures" => reply(&mut stream, 200, "application/json", crate::study::web::failures_json().as_bytes()),
        "/study/breakers" => reply(&mut stream, 200, "application/json", crate::study::web::breakers_json().as_bytes()),
        "/study/components" => reply(&mut stream, 200, "application/json", crate::study::web::components_json().as_bytes()),
        "/study/maintenance" => reply(&mut stream, 200, "application/json", crate::study::web::maintenance_json().as_bytes()),
        "/study/wear" => reply(&mut stream, 200, "application/json", crate::study::web::wear_json().as_bytes()),
        "/study/mel" => reply(&mut stream, 200, "application/json", crate::study::web::mel_search_json(query).as_bytes()),
        #[cfg(feature = "js")]
        "/study/charts" => {
            let icao = query.split('&').find_map(|kv| kv.strip_prefix("icao=")).unwrap_or("");
            let (status, body) = crate::oans::plugin::charts_index(icao);
            reply(&mut stream, status, "application/json", body.as_bytes())
        }
        _ => reply(&mut stream, 200, "text/html; charset=utf-8", PAGE.as_bytes()),
    }
}

/// What XPHFBW's screens would otherwise ask the network for directly
/// (`app/js/msfs-runtime.js` routes it here): Navigraph's AMDB, SimBridge,
/// and the weather FlyByWire's API no longer serves.
fn diag_route(path: &str, body: &str) -> (u16, String) {
    match path {
        "/diag/status" => {}
        "/diag/health" => return (200, serde_json::json!({ "report": crate::diag::health_text() }).to_string()),
        "/diag/mark" => {
            crate::diag::mark(body);
        }
        "/diag/step" => crate::diag::set_step(body),
        "/diag/photo-tour" => crate::diag::request_photo_tour(),
        "/diag/screens" => crate::diag::request_screens(),
        "/diag/save" => crate::diag::request_save(),
        "/diag/enable" => {
            if let Err(e) = crate::diag::set_enabled_for_next_launch(body.trim() != "0") {
                return (500, serde_json::json!({ "error": e.to_string() }).to_string());
            }
        }
        _ => return (404, "{}".to_string()),
    }
    (200, crate::diag::status_json())
}

#[cfg(feature = "js")]
fn network_route(method: &str, path: &str, query: &str, body: &str) -> Option<(u16, &'static str, String)> {
    let started = std::time::Instant::now();
    let answer = network_route_inner(method, path, query, body)?;
    crate::diag::event(
        "net",
        &format!("{method} {path}{}{query} -> {} ({} bytes, {} ms)", if query.is_empty() { "" } else { "?" }, answer.0, answer.2.len(), started.elapsed().as_millis()),
    );
    Some(answer)
}

#[cfg(feature = "js")]
fn network_route_inner(method: &str, path: &str, query: &str, body: &str) -> Option<(u16, &'static str, String)> {
    const JSON: &str = "application/json";
    let with_query = |p: &str| if query.is_empty() { p.to_string() } else { format!("{p}?{query}") };
    if let Some(rest) = path.strip_prefix("/amdb/") {
        let url = format!("https://amdb.api.navigraph.com/{}", with_query(rest));
        let deadline = std::time::Instant::now() + Duration::from_secs(120);
        loop {
            match crate::oans::plugin::amdb_fetch("GET", &url, &url) {
                Some(Some((status, text))) => return Some((status, JSON, text)),
                None => return Some((404, JSON, String::new())),
                Some(None) if std::time::Instant::now() < deadline => std::thread::sleep(Duration::from_millis(50)),
                Some(None) => return Some((504, JSON, String::new())),
            }
        }
    }
    if let Some(rest) = path.strip_prefix("/simbridge") {
        if rest == "/health" {
            return Some((200, JSON, r#"{"status":"ok","info":{"api":{"status":"up"}},"error":{},"details":{"api":{"status":"up"}}}"#.to_string()));
        }
        let (status, text) = crate::mapdata::plugin::simbridge_request(method, rest, body).unwrap_or((404, String::new()));
        return Some((status, JSON, text));
    }
    let rest = path.strip_prefix("/wx/")?;
    let (kind, icao) = rest.split_once('/')?;
    let source = query.split('&').find_map(|kv| kv.strip_prefix("source=")).unwrap_or("");
    let (status, text) = crate::weather_net::answer(kind, icao, source)?;
    Some((status, JSON, text))
}

/// The byte offset of `needle`'s first occurrence in `haystack`, if any.
fn find(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    haystack.windows(needle.len()).position(|w| w == needle)
}

/// `?names=A,B,C` on `/vars`: only those variables, for a Study tab page
/// that only wants the handful it is showing, polled at a few Hz. `None`
/// (no `names` parameter) keeps the full snapshot, as before.
fn names_filter(query: &str) -> Option<Vec<String>> {
    query.split('&').find_map(|kv| {
        let (k, v) = kv.split_once('=')?;
        (k == "names").then(|| v.split(',').filter(|s| !s.is_empty()).map(|s| urldecode(s)).collect())
    })
}

/// The minimal `application/x-www-form-urlencoded` decoding `/vars?names=`
/// needs: `%XX` escapes (a variable name is never percent-encoded in
/// practice, since names hold no reserved characters, but a caller may still
/// encode them) and `+` for space.
fn urldecode(s: &str) -> String {
    let bytes = s.as_bytes();
    let mut out = String::with_capacity(bytes.len());
    let mut i = 0;
    while i < bytes.len() {
        match bytes[i] {
            b'+' => {
                out.push(' ');
                i += 1;
            }
            b'%' if i + 2 < bytes.len() => {
                if let Ok(byte) = u8::from_str_radix(&s[i + 1..i + 3], 16) {
                    out.push(byte as char);
                    i += 3;
                } else {
                    out.push('%');
                    i += 1;
                }
            }
            b => {
                out.push(b as char);
                i += 1;
            }
        }
    }
    out
}

/// A JSON string literal, for the one place this module builds JSON by
/// hand (an error message that may hold arbitrary text).
fn json_string(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

fn reply(stream: &mut TcpStream, status: u16, kind: &str, body: &[u8]) -> std::io::Result<()> {
    reply_headers(stream, status, kind, body)
}

fn reply_headers(stream: &mut TcpStream, status: u16, kind: &str, body: &[u8]) -> std::io::Result<()> {
    let status_text = match status {
        200 => "OK",
        201 => "Created",
        204 => "No Content",
        400 => "Bad Request",
        404 => "Not Found",
        503 => "Service Unavailable",
        504 => "Gateway Timeout",
        _ => "OK",
    };
    write!(
        stream,
        "HTTP/1.1 {status} {status_text}\r\nContent-Type: {kind}\r\nContent-Length: {}\r\nCache-Control: no-store\r\n\
         Access-Control-Allow-Origin: *\r\nAccess-Control-Allow-Methods: GET, POST, OPTIONS\r\nAccess-Control-Allow-Headers: Content-Type\r\n\
         Connection: close\r\n\r\n",
        body.len()
    )?;
    stream.write_all(body)?;
    stream.flush()
}

/// The snapshot as JSON: the simulation's clock, and every variable by name
/// (or, with `only`, just the names asked for — a Study tab page's own
/// fields, polled at a few Hz instead of the whole ~5,000-variable set).
fn efb_state_json() -> String {
    let values: serde_json::Map<String, serde_json::Value> = crate::published::numbers_named("fbw/efb/")
        .into_iter()
        .map(|(name, v, _)| (name.trim_start_matches("fbw/efb/").to_string(), serde_json::json!(if v.is_finite() { v } else { 0. })))
        .collect();
    serde_json::json!({ "values": values }).to_string()
}

fn variables_as_json(only: Option<Vec<String>>) -> String {
    // Copy under the lock and format after it, so a browser asking for the
    // variables never holds up the flight loop.
    let Ok((time, ticks, zulu, local, names, values)) = crate::snapshot()
        .lock()
        .map(|s| (s.time, s.ticks, s.zulu_s, s.local_s, s.names.clone(), s.values.clone()))
    else {
        return "{\"running\":false}".into();
    };
    struct Copy {
        time: f64,
        ticks: u64,
        names: Vec<String>,
        values: Vec<f64>,
    }
    let snapshot = Copy { time, ticks, names, values };
    let mut out = String::with_capacity(snapshot.values.len() * 48 + 64);
    out.push_str("{\"running\":true,\"time\":");
    out.push_str(&format!("{:.1}", snapshot.time));
    out.push_str(",\"ticks\":");
    out.push_str(&snapshot.ticks.to_string());
    out.push_str(&format!(",\"zulu\":{zulu:.1},\"local\":{local:.1}"));
    out.push_str(",\"vars\":{");
    let index: std::collections::HashMap<&str, usize> = if only.is_some() {
        snapshot.names.iter().enumerate().map(|(i, n)| (n.as_str(), i)).collect()
    } else {
        std::collections::HashMap::new()
    };
    let write_one = |out: &mut String, first: &mut bool, name: &str, value: f64| {
        if !*first {
            out.push(',');
        }
        *first = false;
        out.push('"');
        out.push_str(name);
        out.push_str("\":");
        if value.is_finite() {
            let _ = std::fmt::Write::write_fmt(out, format_args!("{value:.4}"));
        } else {
            out.push('0');
        }
    };
    let mut first = true;
    match only {
        Some(wanted) => {
            for name in &wanted {
                if let Some(&i) = index.get(name.as_str()) {
                    write_one(&mut out, &mut first, name, snapshot.values[i]);
                }
            }
        }
        None => {
            for (name, &value) in snapshot.names.iter().zip(&snapshot.values) {
                write_one(&mut out, &mut first, name, value);
            }
        }
    }
    out.push_str("}}");
    out
}
