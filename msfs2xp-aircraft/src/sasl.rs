//! SASL 3 in the aircraft's plugins folder, running the generated module.
//!
//! SASL is 1-sim's proprietary plugin, free for free projects; its licence
//! forbids redistributing it. So it is copied from the user's own download
//! (the zip or its unpacked folder) into this local build only, and the
//! README says where it came from. Without it the aircraft still loads; the
//! animations then sit at rest and the click spots do nothing.

use std::io::Read;
use std::path::{Path, PathBuf};

use anyhow::{bail, Context};

/// Copy SASL from `src` (a zip or a folder) into `<root>/plugins/sasl`, then
/// write the module. Returns a line for the report.
pub fn install(root: &Path, src: Option<&Path>, main_lua: &str, name: &str) -> anyhow::Result<String> {
    let dst = root.join("plugins").join("sasl");
    let mut report = String::from("SASL not given (--sasl): copy SASL 3 into plugins/sasl to run the animations");
    if let Some(src) = src {
        let files = if src.is_file() { from_zip(src, &dst)? } else { from_dir(src, &dst)? };
        report = format!("SASL copied from {} ({files} files)", src.display());
    }
    let modules = dst.join("data").join("modules");
    std::fs::create_dir_all(modules.join("configuration"))?;
    write_if_changed(&modules.join("main.lua"), main_lua.as_bytes())?;
    // SASL refuses to start without its project configuration.
    write_if_changed(&modules.join("configuration").join("configuration.ini"), config_ini(name).as_bytes())?;
    Ok(report)
}

/// Write `bytes` to `path` unless it already holds exactly them. X-Plane
/// keeps a loaded plugin's files open, so rewriting an unchanged SASL while
/// the aircraft is loaded failed with "being used by another process".
fn write_if_changed(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    if std::fs::read(path).is_ok_and(|old| old == bytes) {
        return Ok(());
    }
    std::fs::write(path, bytes)
}

/// SASL's project configuration: an aircraft project, started enabled,
/// without the developer widget.
fn config_ini(name: &str) -> String {
    let name: String = name.chars().filter(|c| !c.is_control() && *c != '=' && *c != '#').collect();
    format!(
        "#################### SASL PROJECT CONFIGURATION ####################\r\n\
         # Written by msfs2xp-aircraft.\r\n\r\n\
         [project]\r\n    id=0\r\n    name={name}\r\n    type=0\r\n    startDisabled=0\r\n    widget=0\r\n"
    )
}

/// Is this path (relative to SASL's root) one to copy? Everything but the
/// sample module folder, which the generated module replaces.
fn wanted(rel: &str) -> bool {
    let r = rel.replace('\\', "/");
    !r.is_empty() && !r.split('/').any(|p| p == "..") && !r.starts_with("data/modules/Custom Module")
}

fn from_zip(zip: &Path, dst: &Path) -> anyhow::Result<usize> {
    let mut archive = zip::ZipArchive::new(std::fs::File::open(zip)?).with_context(|| format!("{} is not a zip", zip.display()))?;
    let names: Vec<String> = (0..archive.len())
        .filter_map(|i| archive.by_index(i).ok().map(|f| f.name().replace('\\', "/")))
        .collect();
    let Some(prefix) = names
        .iter()
        .find_map(|n| n.strip_suffix("64/win.xpl").map(str::to_string))
    else {
        bail!("{} has no 64/win.xpl; is it SASL 3?", zip.display());
    };
    let mut files = 0;
    for i in 0..archive.len() {
        let mut f = archive.by_index(i)?;
        let name = f.name().replace('\\', "/");
        let Some(rel) = name.strip_prefix(&prefix) else { continue };
        if !wanted(rel) || f.is_dir() {
            continue;
        }
        let out = dst.join(rel);
        if let Some(p) = out.parent() {
            std::fs::create_dir_all(p)?;
        }
        let mut bytes = Vec::new();
        f.read_to_end(&mut bytes)?;
        write_if_changed(&out, &bytes)?;
        files += 1;
    }
    Ok(files)
}

fn from_dir(dir: &Path, dst: &Path) -> anyhow::Result<usize> {
    let root: PathBuf = walkdir::WalkDir::new(dir)
        .into_iter()
        .filter_map(Result::ok)
        .find(|e| e.file_name().eq_ignore_ascii_case("win.xpl") && e.path().parent().is_some_and(|p| p.ends_with("64")))
        .and_then(|e| e.path().parent()?.parent().map(Path::to_path_buf))
        .with_context(|| format!("no 64/win.xpl under {}; is it SASL 3?", dir.display()))?;
    let mut files = 0;
    for e in walkdir::WalkDir::new(&root).into_iter().filter_map(Result::ok).filter(|e| e.file_type().is_file()) {
        let rel = e.path().strip_prefix(&root)?.to_string_lossy().to_string();
        if !wanted(&rel) {
            continue;
        }
        let out = dst.join(&rel);
        if let Some(p) = out.parent() {
            std::fs::create_dir_all(p)?;
        }
        write_if_changed(&out, &std::fs::read(e.path())?)?;
        files += 1;
    }
    Ok(files)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_sample_module_and_escapes_are_skipped() {
        assert!(wanted("64/win.xpl"));
        assert!(wanted("data/init/initMain.lua"));
        assert!(!wanted("data/modules/Custom Module/readme.txt"));
        assert!(!wanted("../evil.txt"));
    }

    #[test]
    fn unchanged_files_are_left_alone() {
        let dir = std::env::temp_dir().join(format!("msfs2xp-sasl-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let f = dir.join("win.xpl");
        write_if_changed(&f, b"plugin").unwrap();
        assert_eq!(std::fs::read(&f).unwrap(), b"plugin", "a missing file is written");
        let before = std::fs::metadata(&f).unwrap().modified().unwrap();
        std::thread::sleep(std::time::Duration::from_millis(20));
        write_if_changed(&f, b"plugin").unwrap();
        assert_eq!(std::fs::metadata(&f).unwrap().modified().unwrap(), before, "identical bytes are not rewritten");
        write_if_changed(&f, b"newer").unwrap();
        assert_eq!(std::fs::read(&f).unwrap(), b"newer", "changed bytes are written");
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
