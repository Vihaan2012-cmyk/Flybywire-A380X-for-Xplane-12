//! Newest-copy installs across a chain of releases: every file comes from the
//! newest release that carries it, and only the zip members still needed are
//! fetched, by HTTP range when most of a part is superseded.

use std::collections::HashSet;
use std::io::Read;

pub const STEPS: &str = "install.json";
const EOCD: u32 = 0x0605_4b50;
const CENTRAL: u32 = 0x0201_4b50;
const LOCAL: u32 = 0x0403_4b50;
const TAIL: u64 = 65_557;

pub struct EntryContents {
    pub members: Vec<String>,
    pub deleted: Vec<String>,
}

pub struct Merge {
    pub keep: Vec<HashSet<String>>,
    pub delete: Vec<String>,
}

pub fn merge(entries: &[EntryContents]) -> Merge {
    let mut keep = vec![HashSet::new(); entries.len()];
    let mut claimed: HashSet<String> = HashSet::new();
    let mut deleted_later: HashSet<String> = HashSet::new();
    let mut delete = Vec::new();
    let mut seen = HashSet::new();
    for i in (0..entries.len()).rev() {
        for name in &entries[i].members {
            if name == STEPS {
                keep[i].insert(name.clone());
                continue;
            }
            let gone = name.strip_prefix("aircraft/").is_some_and(|rel| deleted_later.contains(rel));
            if !gone && claimed.insert(name.clone()) {
                keep[i].insert(name.clone());
            }
        }
        for rel in &entries[i].deleted {
            if !claimed.contains(&format!("aircraft/{rel}")) && seen.insert(rel.clone()) {
                delete.push(rel.clone());
            }
            deleted_later.insert(rel.clone());
        }
    }
    Merge { keep, delete }
}

#[derive(Clone, Debug)]
pub struct Member {
    pub name: String,
    pub offset: u64,
    pub end: u64,
    pub compressed: u64,
    pub method: u16,
    pub crc: u32,
}

fn u16_at(b: &[u8], i: usize) -> Option<u16> {
    Some(u16::from_le_bytes(b.get(i..i + 2)?.try_into().ok()?))
}

fn u32_at(b: &[u8], i: usize) -> Option<u32> {
    Some(u32::from_le_bytes(b.get(i..i + 4)?.try_into().ok()?))
}

pub fn members(size: u64, fetch: &dyn Fn(u64, u64) -> Result<Vec<u8>, String>) -> Result<Vec<Member>, String> {
    let tail_start = size.saturating_sub(TAIL);
    let tail = fetch(tail_start, size)?;
    let at = (0..tail.len().saturating_sub(21)).rev().find(|&i| u32_at(&tail, i) == Some(EOCD)).ok_or("no end of central directory")?;
    let dir_size = u32_at(&tail, at + 12).ok_or("short end record")? as u64;
    let dir_start = u32_at(&tail, at + 16).ok_or("short end record")? as u64;
    let dir = if dir_start >= tail_start { tail[(dir_start - tail_start) as usize..(dir_start - tail_start + dir_size) as usize].to_vec() } else { fetch(dir_start, dir_start + dir_size)? };
    let mut out = Vec::new();
    let mut i = 0;
    while u32_at(&dir, i) == Some(CENTRAL) {
        let method = u16_at(&dir, i + 10).ok_or("short directory")?;
        let crc = u32_at(&dir, i + 16).ok_or("short directory")?;
        let compressed = u32_at(&dir, i + 20).ok_or("short directory")? as u64;
        let (n, e, c) = (u16_at(&dir, i + 28).ok_or("short directory")? as usize, u16_at(&dir, i + 30).ok_or("short directory")? as usize, u16_at(&dir, i + 32).ok_or("short directory")? as usize);
        let offset = u32_at(&dir, i + 42).ok_or("short directory")? as u64;
        let name = String::from_utf8_lossy(dir.get(i + 46..i + 46 + n).ok_or("short directory")?).replace('\\', "/");
        out.push(Member { name, offset, end: 0, compressed, method, crc });
        i += 46 + n + e + c;
    }
    out.sort_by_key(|m| m.offset);
    for k in 0..out.len() {
        out[k].end = out.get(k + 1).map_or(dir_start, |next| next.offset);
    }
    Ok(out)
}

pub fn extract(span: &[u8], m: &Member) -> Result<Vec<u8>, String> {
    if u32_at(span, 0) != Some(LOCAL) {
        return Err(format!("{}: no local header", m.name));
    }
    let n = u16_at(span, 26).ok_or("short local header")? as usize;
    let e = u16_at(span, 28).ok_or("short local header")? as usize;
    let data = span.get(30 + n + e..30 + n + e + m.compressed as usize).ok_or_else(|| format!("{}: short data", m.name))?;
    let out = match m.method {
        0 => data.to_vec(),
        8 => {
            let mut v = Vec::new();
            flate2::read::DeflateDecoder::new(data).read_to_end(&mut v).map_err(|e| format!("{}: {e}", m.name))?;
            v
        }
        other => return Err(format!("{}: compression method {other}", m.name)),
    };
    if crc32fast::hash(&out) != m.crc {
        return Err(format!("{}: CRC mismatch", m.name));
    }
    Ok(out)
}

pub fn http_range(url: &str, start: u64, end: u64) -> Result<Vec<u8>, String> {
    let response = ureq::get(url)
        .set("User-Agent", "fbw-a380x-installer")
        .set("Range", &format!("bytes={start}-{}", end.saturating_sub(1)))
        .call()
        .map_err(|e| format!("range {start}-{end}: {e}"))?;
    if response.status() != 206 {
        return Err(format!("the server ignored a range request ({})", response.status()));
    }
    let mut v = Vec::with_capacity((end - start) as usize);
    response.into_reader().read_to_end(&mut v).map_err(|e| e.to_string())?;
    if v.len() as u64 != end - start {
        return Err(format!("range {start}-{end} came back with {} bytes", v.len()));
    }
    Ok(v)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    fn entry(members: &[&str], deleted: &[&str]) -> EntryContents {
        EntryContents { members: members.iter().map(|s| s.to_string()).collect(), deleted: deleted.iter().map(|s| s.to_string()).collect() }
    }

    #[test]
    fn every_file_comes_from_the_newest_release_and_only_live_deletions_remain() {
        let m = merge(&[
            entry(&["install.json", "aircraft/a.obj", "aircraft/b.obj", "aircraft/c.obj"], &[]),
            entry(&["install.json", "aircraft/a.obj"], &["c.obj"]),
            entry(&["install.json", "aircraft/a.obj", "aircraft/c.obj"], &["b.obj"]),
        ]);
        assert_eq!(m.keep[0], ["install.json"].iter().map(|s| s.to_string()).collect());
        assert_eq!(m.keep[1], ["install.json"].iter().map(|s| s.to_string()).collect());
        assert_eq!(m.keep[2], ["install.json", "aircraft/a.obj", "aircraft/c.obj"].iter().map(|s| s.to_string()).collect());
        assert_eq!(m.delete, vec!["b.obj".to_string()], "c.obj came back in the newest release");
    }

    #[test]
    fn a_member_is_read_back_from_its_byte_range_alone() {
        let mut buf = std::io::Cursor::new(Vec::new());
        {
            let mut z = zip::ZipWriter::new(&mut buf);
            let deflate = zip::write::SimpleFileOptions::default().compression_method(zip::CompressionMethod::Deflated);
            for (name, body) in [("install.json", "{}".repeat(50)), ("aircraft/x.obj", "VT 1 2 3\n".repeat(2000)), ("aircraft/y.dds", "y".repeat(10))] {
                z.start_file(name, deflate).unwrap();
                z.write_all(body.as_bytes()).unwrap();
            }
            z.finish().unwrap();
        }
        let bytes = buf.into_inner();
        let fetched = std::cell::Cell::new(0u64);
        let fetch = |s: u64, e: u64| -> Result<Vec<u8>, String> {
            fetched.set(fetched.get() + e - s);
            Ok(bytes[s as usize..e as usize].to_vec())
        };
        let list = members(bytes.len() as u64, &fetch).unwrap();
        assert_eq!(list.iter().map(|m| m.name.as_str()).collect::<Vec<_>>(), ["install.json", "aircraft/x.obj", "aircraft/y.dds"]);
        let x = &list[1];
        let body = extract(&fetch(x.offset, x.end).unwrap(), x).unwrap();
        assert_eq!(body, "VT 1 2 3\n".repeat(2000).as_bytes());
    }
}
