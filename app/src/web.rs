//! The local web server the settings page talks to: the page itself, and
//! JSON for settings, actions and status. Bound to 127.0.0.1 on a free port.

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Duration;

use serde_json::{json, Map, Value};

use crate::Shared;

pub fn start(shared: Arc<Shared>) {
    let Ok(listener) = TcpListener::bind(("127.0.0.1", 0)) else { return };
    if let Ok(addr) = listener.local_addr() {
        shared.port.store(addr.port(), Ordering::Relaxed);
    }
    let _ = std::thread::Builder::new().name("settings server".into()).spawn(move || {
        for stream in listener.incoming().flatten() {
            let s = shared.clone();
            let _ = std::thread::Builder::new().spawn(move || {
                let _ = serve(stream, &s);
            });
        }
    });
}

fn serve(mut stream: TcpStream, shared: &Shared) -> std::io::Result<()> {
    stream.set_read_timeout(Some(Duration::from_secs(5)))?;
    let mut buf = Vec::new();
    let mut chunk = [0u8; 8192];
    // Headers, then as much body as Content-Length says.
    let (head_end, content_length) = loop {
        let n = stream.read(&mut chunk)?;
        if n == 0 {
            return Ok(());
        }
        buf.extend_from_slice(&chunk[..n]);
        if let Some(i) = buf.windows(4).position(|w| w == b"\r\n\r\n") {
            let head = String::from_utf8_lossy(&buf[..i]).to_ascii_lowercase();
            let len = head
                .lines()
                .find_map(|l| l.strip_prefix("content-length:").and_then(|v| v.trim().parse::<usize>().ok()))
                .unwrap_or(0);
            break (i + 4, len);
        }
        if buf.len() > 1 << 20 {
            return Ok(());
        }
    };
    while buf.len() < head_end + content_length {
        let n = stream.read(&mut chunk)?;
        if n == 0 {
            break;
        }
        buf.extend_from_slice(&chunk[..n]);
    }
    let head = String::from_utf8_lossy(&buf[..head_end]).to_string();
    let body = &buf[head_end..(head_end + content_length).min(buf.len())];
    let mut first = head.lines().next().unwrap_or("").split_whitespace();
    let (method, target) = (first.next().unwrap_or(""), first.next().unwrap_or("/"));
    let (path, query) = target.split_once('?').unwrap_or((target, ""));

    match (method, path) {
        ("GET", "/") | ("GET", "/index.html") => match std::fs::read(shared.ui_dir.join("index.html")) {
            Ok(page) => reply(&mut stream, "200 OK", "text/html; charset=utf-8", &page),
            Err(e) => reply(&mut stream, "500 Internal Server Error", "text/plain", e.to_string().as_bytes()),
        },
        // X-Plane 12's own logo, read from the X-Plane install (not bundled).
        ("GET", "/xplane12-logo.png") => {
            let logo = shared.xplane_root.join("Resources").join("bitmaps").join("interface11").join("logo@2x.png");
            match std::fs::read(logo) {
                Ok(png) => reply(&mut stream, "200 OK", "image/png", &png),
                Err(_) => reply(&mut stream, "404 Not Found", "text/plain", b"no logo"),
            }
        }
        ("GET", "/api/settings") => json_reply(&mut stream, &Value::Object(crate::settings::load(&shared.xplane_root))),
        ("POST", "/api/settings") => {
            let values = serde_json::from_slice::<Value>(body).ok().and_then(|v| v.as_object().cloned()).unwrap_or_default();
            match crate::settings::save(&shared.xplane_root, &values) {
                Ok(()) => json_reply(&mut stream, &json!({ "ok": true })),
                Err(e) => json_reply(&mut stream, &json!({ "ok": false, "message": e })),
            }
        }
        ("GET", "/api/status") => json_reply(&mut stream, &status(shared)),
        // The Study pages' layouts, from the same Rust the plugin serves them
        // from, for when X-Plane (the plugin's port) is not running.
        ("GET", "/study/pages") => reply(&mut stream, "200 OK", "application/json", fbw_a380_systems::study_json::pages().as_bytes()),
        ("GET", "/study/failures") => reply(&mut stream, "200 OK", "application/json", fbw_a380_systems::study_json::failures().as_bytes()),
        ("GET", "/study/breakers") => reply(&mut stream, "200 OK", "application/json", fbw_a380_systems::study_json::breakers().as_bytes()),
        ("GET", "/study/components") => reply(&mut stream, "200 OK", "application/json", fbw_a380_systems::study_json::components().as_bytes()),
        ("GET", "/study/maintenance") => reply(&mut stream, "200 OK", "application/json", fbw_a380_systems::study_json::maintenance().as_bytes()),
        ("GET", "/study/mel") => reply(&mut stream, "200 OK", "application/json", fbw_a380_systems::study_json::mel(query).as_bytes()),
        ("POST", p) if p.starts_with("/api/action/") => json_reply(&mut stream, &action(shared, &p["/api/action/".len()..])),
        _ => reply(&mut stream, "404 Not Found", "text/plain", b"not found"),
    }
}

fn reply(stream: &mut TcpStream, status: &str, kind: &str, body: &[u8]) -> std::io::Result<()> {
    write!(
        stream,
        "HTTP/1.1 {status}\r\nContent-Type: {kind}\r\nContent-Length: {}\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n",
        body.len()
    )?;
    stream.write_all(body)
}

fn json_reply(stream: &mut TcpStream, value: &Value) -> std::io::Result<()> {
    reply(stream, "200 OK", "application/json", value.to_string().as_bytes())
}

fn status(shared: &Shared) -> Value {
    let st = shared.status.lock().unwrap();
    let mut out = Map::new();
    out.insert("xplane".into(), Value::Bool(st.xplane_connected));
    out.insert(
        "systems".into(),
        if st.systems_running {
            // `serve_systems` (main.rs) runs FlyByWire's systems on a thread
            // of this very process; these are its own live counters
            // (src/remote/server.rs), not a round trip to another process.
            use fbw_a380_systems::remote::server::{LAST_TICK_MICROS, TICKS_SERVED, VARIABLE_COUNT};
            json!({
                "ticks": TICKS_SERVED.load(Ordering::Relaxed),
                "lastTickMicros": LAST_TICK_MICROS.load(Ordering::Relaxed),
                "variables": VARIABLE_COUNT.load(Ordering::Relaxed),
            })
        } else {
            Value::Null
        },
    );
    out.insert("displays".into(), Value::Null);
    if let Some(e) = &st.systems_error {
        out.insert("systemsError".into(), Value::String(e.clone()));
    }
    Value::Object(out)
}

fn open(target: &str) {
    let _ = std::process::Command::new("explorer").arg(target).spawn();
}

fn action(shared: &Shared, name: &str) -> Value {
    let root = &shared.xplane_root;
    match name {
        "open-logs" => {
            let _ = std::process::Command::new("explorer").arg(format!("/select,{}", root.join("Log.txt").display())).spawn();
            json!({ "ok": true, "message": "Opened X-Plane's Log.txt" })
        }
        "open-dumps" => {
            open(r"D:\A380\fbw-build\state-dumps");
            json!({ "ok": true, "message": "Opened the state dumps" })
        }
        "report" => {
            open("https://github.com/flybywiresim/aircraft/issues");
            json!({ "ok": true })
        }
        "discord" => {
            open("https://discord.gg/2aUM4E7GQw");
            json!({ "ok": true })
        }
        "restart-displays" => {
            crate::views::restart_displays();
            json!({ "ok": true, "message": "Restarting the instrument displays" })
        }
        "navigraph" => {
            json!({ "ok": false, "message": "Navigraph account linking isn't available yet; set a SimBrief user ID above instead to use INIT > CPNY F-PLN REQUEST" })
        }
        "restart-all" | "restart-systems" | "reset-airframe" | "licenses" => {
            json!({ "ok": false, "message": "Not available yet in this XPHFBW build" })
        }
        other => json!({ "ok": false, "message": format!("Unknown action {other}") }),
    }
}
