use std::io::{Read, Write};
use std::net::TcpStream;
use std::os::raw::c_int;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cef::*;

pub const ROUTE: &str = "/__plugin/";
const PLUGIN: &str = "127.0.0.1:8390";
const PLUGIN_WAIT: Duration = Duration::from_secs(150);

#[derive(Debug, PartialEq)]
pub struct Reply {
    pub status: i32,
    pub mime: String,
    pub body: Vec<u8>,
}

impl Reply {
    fn failed(status: i32) -> Self {
        Self { status, mime: "text/plain".into(), body: Vec::new() }
    }
}

pub fn target_of(url: &str) -> Option<&str> {
    let rest = url.strip_prefix("coui://")?;
    let at = rest.find(ROUTE)?;
    Some(&rest[at + ROUTE.len()..])
}

pub fn forward(address: &str, method: &str, target: &str, body: &[u8]) -> Reply {
    let Ok(mut stream) = TcpStream::connect(address) else { return Reply::failed(502) };
    let _ = stream.set_read_timeout(Some(PLUGIN_WAIT));
    let method = if method.is_empty() { "GET" } else { method };
    let head = format!(
        "{method} /{target} HTTP/1.1\r\nHost: {address}\r\nConnection: close\r\nContent-Type: application/json\r\nContent-Length: {}\r\n\r\n",
        body.len()
    );
    if stream.write_all(head.as_bytes()).and_then(|_| stream.write_all(body)).is_err() {
        return Reply::failed(502);
    }
    let mut raw = Vec::new();
    if stream.read_to_end(&mut raw).is_err() && raw.is_empty() {
        return Reply::failed(504);
    }
    parse(&raw).unwrap_or_else(|| Reply::failed(502))
}

pub fn parse(raw: &[u8]) -> Option<Reply> {
    let end = raw.windows(4).position(|w| w == b"\r\n\r\n")?;
    let head = std::str::from_utf8(&raw[..end]).ok()?;
    let mut lines = head.split("\r\n");
    let status = lines.next()?.split_whitespace().nth(1)?.parse().ok()?;
    let mut mime = "application/octet-stream".to_string();
    let mut length = None;
    for line in lines {
        let Some((name, value)) = line.split_once(':') else { continue };
        match name.trim().to_ascii_lowercase().as_str() {
            "content-type" => mime = value.split(';').next().unwrap_or("").trim().to_string(),
            "content-length" => length = value.trim().parse::<usize>().ok(),
            _ => {}
        }
    }
    let mut body = raw[end + 4..].to_vec();
    if let Some(n) = length {
        body.truncate(n);
    }
    Some(Reply { status, mime, body })
}

fn post_body(request: &mut Request) -> Vec<u8> {
    let Some(data) = request.post_data() else { return Vec::new() };
    let mut elements: Vec<Option<PostDataElement>> = (0..data.element_count()).map(|_| None).collect();
    data.elements(Some(&mut elements));
    let mut out = Vec::new();
    for element in elements.into_iter().flatten() {
        let mut bytes = vec![0u8; element.bytes_count()];
        let got = element.bytes(bytes.len(), bytes.as_mut_ptr());
        bytes.truncate(got);
        out.extend(bytes);
    }
    out
}

wrap_resource_handler! {
    struct PluginRoute {
        target: String,
        reply: Arc<Mutex<Option<(Reply, usize)>>>,
    }

    impl ResourceHandler {
        fn open(&self, request: Option<&mut Request>, handle_request: Option<&mut c_int>, _callback: Option<&mut Callback>) -> c_int {
            let (method, body) = match request {
                Some(r) => (CefString::from(&r.method()).to_string(), post_body(r)),
                None => (String::new(), Vec::new()),
            };
            let reply = forward(PLUGIN, &method, &self.target, &body);
            if let Ok(mut slot) = self.reply.lock() {
                *slot = Some((reply, 0));
            }
            if let Some(handle) = handle_request {
                *handle = 1;
            }
            1
        }

        fn response_headers(&self, response: Option<&mut Response>, response_length: Option<&mut i64>, _redirect_url: Option<&mut CefString>) {
            let Ok(slot) = self.reply.lock() else { return };
            let Some((reply, _)) = slot.as_ref() else { return };
            if let Some(response) = response {
                response.set_status(reply.status);
                response.set_status_text(Some(&CefString::from(if (200..300).contains(&reply.status) { "OK" } else { "Error" })));
                response.set_mime_type(Some(&CefString::from(reply.mime.as_str())));
            }
            if let Some(length) = response_length {
                *length = reply.body.len() as i64;
            }
        }

        fn read(&self, data_out: *mut u8, bytes_to_read: c_int, bytes_read: Option<&mut c_int>, _callback: Option<&mut ResourceReadCallback>) -> c_int {
            let Ok(mut slot) = self.reply.lock() else { return 0 };
            let Some((reply, offset)) = slot.as_mut() else { return 0 };
            let n = (reply.body.len() - *offset).min(bytes_to_read.max(0) as usize);
            if let Some(read) = bytes_read {
                *read = n as c_int;
            }
            if n == 0 {
                return 0;
            }
            unsafe { std::ptr::copy_nonoverlapping(reply.body[*offset..].as_ptr(), data_out, n) };
            *offset += n;
            1
        }
    }
}

pub fn handler(target: &str) -> ResourceHandler {
    PluginRoute::new(target.to_string(), Arc::new(Mutex::new(None)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpListener;

    #[test]
    fn only_the_plugin_route_is_forwarded() {
        assert_eq!(target_of("coui://html_ui/__plugin/amdb/v1/search?q="), Some("amdb/v1/search?q="));
        assert_eq!(target_of("coui://html_ui/Pages/VCockpit/Instruments/A380X/ND/nd.html"), None);
        assert_eq!(target_of("http://127.0.0.1:8390/__plugin/amdb/v1/x"), None);
    }

    #[test]
    fn a_reply_keeps_its_status_type_and_exact_body() {
        let raw = b"HTTP/1.1 200 OK\r\nContent-Type: application/json; charset=utf-8\r\nContent-Length: 4\r\n\r\n[1,2]extra";
        assert_eq!(parse(raw), Some(Reply { status: 200, mime: "application/json".into(), body: b"[1,2".to_vec() }));
        assert_eq!(parse(b"garbage"), None);
    }

    #[test]
    fn a_request_reaches_the_plugin_with_its_method_and_body() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let address = listener.local_addr().unwrap().to_string();
        let server = std::thread::spawn(move || {
            let (mut s, _) = listener.accept().unwrap();
            let mut buf = vec![0u8; 4096];
            let n = s.read(&mut buf).unwrap();
            let request = String::from_utf8_lossy(&buf[..n]).to_string();
            s.write_all(b"HTTP/1.1 201 Created\r\nContent-Type: application/json\r\nContent-Length: 2\r\nConnection: close\r\n\r\n{}").unwrap();
            request
        });
        let reply = forward(&address, "POST", "simbridge/api/v1/terrain/x?y=1", b"{\"a\":1}");
        let request = server.join().unwrap();
        assert!(request.starts_with("POST /simbridge/api/v1/terrain/x?y=1 HTTP/1.1\r\n"), "{request}");
        assert!(request.ends_with("{\"a\":1}"), "{request}");
        assert_eq!(reply, Reply { status: 201, mime: "application/json".into(), body: b"{}".to_vec() });
    }

    #[test]
    fn a_plugin_that_is_not_running_gives_a_gateway_error() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let address = listener.local_addr().unwrap().to_string();
        drop(listener);
        assert_eq!(forward(&address, "GET", "amdb/v1/search?q=", b"").status, 502);
    }
}
