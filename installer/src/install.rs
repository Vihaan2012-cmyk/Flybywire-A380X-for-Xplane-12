//! The installer's work, off the UI thread: manifest, download, verify,
//! install steps, version record.

use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::sync::mpsc::Sender;

use native_windows_gui as nwg;
use serde::Deserialize;
use sha2::{Digest, Sha256};

pub struct Answers {
    pub xplane: PathBuf,
    /// Only a release whose steps convert from the user's own MSFS package
    /// needs these.
    pub msfs_package: Option<PathBuf>,
    pub livery: Option<PathBuf>,
    pub sasl: PathBuf,
    pub server: String,
}

pub enum Msg {
    Status(String),
    Log(String),
    Progress(f64),
    Done(Result<String, String>),
}

/// The release server's `/manifest`.
#[derive(Deserialize)]
struct Manifest {
    version: String,
    #[serde(default)]
    notes: String,
    files: Vec<ReleaseFile>,
}

#[derive(Deserialize)]
struct ReleaseFile {
    name: String,
    size: u64,
    sha256: String,
}

/// `install.json` inside the release archive: what to do, in order.
#[derive(Deserialize)]
struct Steps {
    /// The aircraft folder, relative to the X-Plane folder.
    aircraft: String,
    steps: Vec<Step>,
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum Step {
    /// Copy a folder from the archive into the aircraft folder.
    Copy { from: String, to: String },
    /// Run a program from the archive; `{xplane}`, `{aircraft}`,
    /// `{msfs_package}`, `{livery}`, `{sasl}` and `{release}` are filled in.
    /// Arguments containing `{livery}` are dropped when no livery was chosen.
    Run { program: String, args: Vec<String> },
    /// Copy files from the user's own MSFS package into the aircraft folder.
    CopyFromMsfs { from: String, files: Vec<String>, to: String },
    /// Install SASL from the user's own archive (SASL is proprietary and
    /// never in a release): everything under the folder holding
    /// `64/win.xpl`, except SASL's sample module.
    Sasl { to: String },
}

const VERSION_FILE: &str = "fbw_xp_installed_version.txt";

struct Ctx {
    tx: Sender<Msg>,
    notice: nwg::NoticeSender,
}

impl Ctx {
    fn send(&self, m: Msg) {
        let _ = self.tx.send(m);
        self.notice.notice();
    }
    fn status(&self, s: impl Into<String>) {
        self.send(Msg::Status(s.into()));
    }
    fn log(&self, s: impl Into<String>) {
        self.send(Msg::Log(s.into()));
    }
}

pub fn run(a: Answers, tx: Sender<Msg>, notice: nwg::NoticeSender) {
    let ctx = Ctx { tx, notice };
    let result = work(&a, &ctx);
    ctx.send(Msg::Done(result));
}

fn work(a: &Answers, ctx: &Ctx) -> Result<String, String> {
    ctx.status(format!("Checking {} for the latest release...", a.server));
    let manifest: Manifest = ureq::get(&format!("{}/manifest", a.server.trim_end_matches('/')))
        .call()
        .map_err(|e| format!("could not reach the release server: {e}"))?
        .into_string()
        .map_err(|e| format!("bad manifest: {e}"))
        .and_then(|t| serde_json::from_str(&t).map_err(|e| format!("bad manifest: {e}")))?;
    ctx.log(format!("Latest release: {}", manifest.version));
    if !manifest.notes.is_empty() {
        ctx.log(manifest.notes.clone());
    }

    let work_dir = std::env::temp_dir().join(format!("fbw-a380x-xp-{}", manifest.version));
    std::fs::create_dir_all(&work_dir).map_err(|e| e.to_string())?;
    let total: u64 = manifest.files.iter().map(|f| f.size).sum::<u64>().max(1);
    let mut done = 0u64;
    let mut archives = Vec::new();
    for f in &manifest.files {
        let url = format!("{}/download/{}/{}", a.server.trim_end_matches('/'), manifest.version, f.name);
        let path = work_dir.join(&f.name);
        ctx.status(format!("Downloading {}...", f.name));
        download(&url, &path, f, ctx, &mut done, total)?;
        archives.push(path);
    }

    ctx.status("Unpacking...");
    ctx.send(Msg::Progress(0.0));
    let release = work_dir.join("release");
    let _ = std::fs::remove_dir_all(&release);
    for z in &archives {
        unzip(z, &release)?;
    }

    let steps: Steps = serde_json::from_str(&std::fs::read_to_string(release.join("install.json")).map_err(|e| format!("install.json: {e}"))?)
        .map_err(|e| format!("install.json: {e}"))?;
    let aircraft = a.xplane.join(&steps.aircraft);
    let installed = std::fs::read_to_string(aircraft.join(VERSION_FILE)).ok();
    if let Some(v) = &installed {
        ctx.log(format!("Installed: {}", v.trim()));
    }

    let n = steps.steps.len().max(1);
    for (i, step) in steps.steps.iter().enumerate() {
        ctx.send(Msg::Progress(i as f64 / n as f64));
        match step {
            Step::Copy { from, to } => {
                ctx.status(format!("Copying {from}..."));
                copy_tree(&release.join(from), &aircraft.join(to))?;
            }
            Step::Run { program, args } => {
                let fill = |s: &str| -> Option<String> {
                    if s.contains("{livery}") && a.livery.is_none() {
                        return None;
                    }
                    Some(
                        s.replace("{xplane}", &a.xplane.to_string_lossy())
                            .replace("{aircraft}", &aircraft.to_string_lossy())
                            .replace("{msfs_package}", &a.msfs_package.as_ref().map(|p| p.to_string_lossy().into_owned()).unwrap_or_default())
                            .replace("{livery}", &a.livery.as_ref().map(|l| l.to_string_lossy().into_owned()).unwrap_or_default())
                            .replace("{sasl}", &a.sasl.to_string_lossy())
                            .replace("{release}", &release.to_string_lossy()),
                    )
                };
                // An option whose value names the livery is dropped along with
                // it: "--livery" "{livery}".
                let mut filled = Vec::new();
                let mut i = 0;
                while i < args.len() {
                    if args[i].starts_with("--") && args.get(i + 1).is_some_and(|v| v.contains("{livery}")) && a.livery.is_none() {
                        i += 2;
                        continue;
                    }
                    if let Some(v) = fill(&args[i]) {
                        filled.push(v);
                    }
                    i += 1;
                }
                ctx.status(format!("Running {program} (this converts from your own MSFS package; it can take several minutes)..."));
                run_program(&release.join(program), &filled, ctx)?;
            }
            Step::CopyFromMsfs { from, files, to } => {
                ctx.status(format!("Copying {} from your MSFS package...", files.join(", ")));
                let dst = aircraft.join(to);
                std::fs::create_dir_all(&dst).map_err(|e| e.to_string())?;
                let package = a.msfs_package.as_ref().ok_or("this release needs your MSFS package")?;
                for f in files {
                    let src = package.join(from).join(f);
                    std::fs::copy(&src, dst.join(f)).map_err(|e| format!("{}: {e}", src.display()))?;
                }
            }
            Step::Sasl { to } => {
                ctx.status("Installing SASL from your archive...");
                let n = install_sasl(&a.sasl, &aircraft.join(to))?;
                ctx.log(format!("SASL: {n} files"));
            }
        }
    }
    std::fs::write(aircraft.join(VERSION_FILE), &manifest.version).map_err(|e| e.to_string())?;
    ctx.send(Msg::Progress(1.0));
    Ok(match installed {
        Some(v) if v.trim() == manifest.version => format!("Reinstalled {} into {}", manifest.version, aircraft.display()),
        Some(v) => format!("Updated {} -> {} in {}", v.trim(), manifest.version, aircraft.display()),
        None => format!("Installed {} into {}", manifest.version, aircraft.display()),
    })
}

fn download(url: &str, path: &Path, f: &ReleaseFile, ctx: &Ctx, done: &mut u64, total: u64) -> Result<(), String> {
    // Already downloaded and intact: keep it.
    if path.exists() && sha256_file(path).is_ok_and(|h| h.eq_ignore_ascii_case(&f.sha256)) {
        *done += f.size;
        ctx.log(format!("{} already downloaded and verified", f.name));
        return Ok(());
    }
    let response = ureq::get(url).call().map_err(|e| format!("download {}: {e}", f.name))?;
    let mut reader = response.into_reader();
    let mut out = std::fs::File::create(path).map_err(|e| e.to_string())?;
    let mut hasher = Sha256::new();
    let mut buf = vec![0u8; 1 << 20];
    loop {
        let n = reader.read(&mut buf).map_err(|e| format!("download {}: {e}", f.name))?;
        if n == 0 {
            break;
        }
        hasher.update(&buf[..n]);
        out.write_all(&buf[..n]).map_err(|e| e.to_string())?;
        *done += n as u64;
        ctx.send(Msg::Progress(*done as f64 / total as f64));
    }
    let got = hex(&hasher.finalize());
    if !got.eq_ignore_ascii_case(&f.sha256) {
        let _ = std::fs::remove_file(path);
        return Err(format!("{} failed verification (SHA-256 {got}, expected {})", f.name, f.sha256));
    }
    ctx.log(format!("{} verified", f.name));
    Ok(())
}

/// SASL 3 from its distribution zip: the files under the folder that holds
/// `64/win.xpl`, without its sample module (`data/modules/Custom Module`),
/// which the aircraft's own module replaces. The same selection the
/// aircraft converter makes (msfs2xp-aircraft `sasl.rs`).
fn install_sasl(zip_path: &Path, dst: &Path) -> Result<usize, String> {
    let file = std::fs::File::open(zip_path).map_err(|e| format!("{}: {e}", zip_path.display()))?;
    let mut zip = zip::ZipArchive::new(file).map_err(|e| format!("{} is not a zip: {e}", zip_path.display()))?;
    let names: Vec<String> = (0..zip.len()).filter_map(|i| zip.by_index(i).ok().map(|f| f.name().replace('\\', "/"))).collect();
    let prefix = names
        .iter()
        .find_map(|n| n.strip_suffix("64/win.xpl").map(str::to_owned))
        .ok_or_else(|| format!("{} has no 64/win.xpl; is it SASL 3?", zip_path.display()))?;
    let mut n = 0;
    for i in 0..zip.len() {
        let mut f = zip.by_index(i).map_err(|e| e.to_string())?;
        let name = f.name().replace('\\', "/");
        let Some(rel) = name.strip_prefix(&prefix) else { continue };
        if rel.is_empty() || f.is_dir() || rel.split('/').any(|p| p == "..") || rel.starts_with("data/modules/Custom Module") {
            continue;
        }
        let out = dst.join(rel);
        if let Some(p) = out.parent() {
            std::fs::create_dir_all(p).map_err(|e| e.to_string())?;
        }
        let mut w = std::fs::File::create(&out).map_err(|e| format!("{}: {e}", out.display()))?;
        std::io::copy(&mut f, &mut w).map_err(|e| e.to_string())?;
        n += 1;
    }
    Ok(n)
}

fn sha256_file(path: &Path) -> Result<String, String> {
    let mut file = std::fs::File::open(path).map_err(|e| e.to_string())?;
    let mut hasher = Sha256::new();
    let mut buf = vec![0u8; 1 << 20];
    loop {
        let n = file.read(&mut buf).map_err(|e| e.to_string())?;
        if n == 0 {
            break;
        }
        hasher.update(&buf[..n]);
    }
    Ok(hex(&hasher.finalize()))
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

fn unzip(archive: &Path, into: &Path) -> Result<(), String> {
    let file = std::fs::File::open(archive).map_err(|e| e.to_string())?;
    let mut zip = zip::ZipArchive::new(file).map_err(|e| format!("{}: {e}", archive.display()))?;
    for i in 0..zip.len() {
        let mut entry = zip.by_index(i).map_err(|e| e.to_string())?;
        // Only paths that stay inside the target folder.
        let Some(rel) = entry.enclosed_name() else { continue };
        let out = into.join(rel);
        if entry.is_dir() {
            std::fs::create_dir_all(&out).map_err(|e| e.to_string())?;
            continue;
        }
        if let Some(parent) = out.parent() {
            std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
        }
        let mut w = std::fs::File::create(&out).map_err(|e| e.to_string())?;
        std::io::copy(&mut entry, &mut w).map_err(|e| e.to_string())?;
    }
    Ok(())
}

fn copy_tree(from: &Path, to: &Path) -> Result<(), String> {
    std::fs::create_dir_all(to).map_err(|e| e.to_string())?;
    for entry in std::fs::read_dir(from).map_err(|e| format!("{}: {e}", from.display()))? {
        let entry = entry.map_err(|e| e.to_string())?;
        let dst = to.join(entry.file_name());
        if entry.file_type().map_err(|e| e.to_string())?.is_dir() {
            copy_tree(&entry.path(), &dst)?;
        } else {
            std::fs::copy(entry.path(), &dst).map_err(|e| format!("{}: {e}", dst.display()))?;
        }
    }
    Ok(())
}

fn run_program(program: &Path, args: &[String], ctx: &Ctx) -> Result<(), String> {
    use std::io::BufRead;
    use std::os::windows::process::CommandExt;
    const CREATE_NO_WINDOW: u32 = 0x0800_0000;
    let mut child = std::process::Command::new(program)
        .args(args)
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .creation_flags(CREATE_NO_WINDOW)
        .spawn()
        .map_err(|e| format!("{}: {e}", program.display()))?;
    if let Some(err) = child.stderr.take() {
        let tx = ctx.tx.clone();
        let notice = ctx.notice;
        std::thread::spawn(move || {
            for line in std::io::BufReader::new(err).lines().map_while(Result::ok) {
                let _ = tx.send(Msg::Log(line));
                notice.notice();
            }
        });
    }
    if let Some(out) = child.stdout.take() {
        for line in std::io::BufReader::new(out).lines().map_while(Result::ok) {
            ctx.log(line);
        }
    }
    let status = child.wait().map_err(|e| e.to_string())?;
    if status.success() {
        Ok(())
    } else {
        Err(format!("{} exited with {status}", program.display()))
    }
}

// ---- detection ------------------------------------------------------------

/// X-Plane 12's own record of its installs (`%LOCALAPPDATA%\x-plane_install_12.txt`,
/// one folder per line); the first that exists.
pub fn detect_xplane() -> Option<PathBuf> {
    let list = std::fs::read_to_string(PathBuf::from(std::env::var("LOCALAPPDATA").ok()?).join("x-plane_install_12.txt")).ok()?;
    list.lines().map(|l| PathBuf::from(l.trim())).find(|p| !p.as_os_str().is_empty() && p.join("Aircraft").is_dir())
}

/// The FlyByWire A380X package in MSFS's Community folder, from MSFS's own
/// `UserCfg.opt` (`InstalledPackagesPath`), for the Steam and Store installs.
pub fn detect_fbw_package() -> Option<PathBuf> {
    let appdata = PathBuf::from(std::env::var("APPDATA").ok()?);
    let local = PathBuf::from(std::env::var("LOCALAPPDATA").ok()?);
    let candidates = [
        appdata.join("Microsoft Flight Simulator").join("UserCfg.opt"),
        local.join("Packages").join("Microsoft.FlightSimulator_8wekyb3d8bbwe").join("LocalCache").join("UserCfg.opt"),
    ];
    for cfg in candidates {
        let Ok(text) = std::fs::read_to_string(&cfg) else { continue };
        for line in text.lines() {
            if let Some(rest) = line.trim().strip_prefix("InstalledPackagesPath") {
                let root = PathBuf::from(rest.trim().trim_matches('"'));
                let pkg = root.join("Community").join("flybywire-aircraft-a380-842");
                if pkg.is_dir() {
                    return Some(pkg);
                }
            }
        }
    }
    None
}
