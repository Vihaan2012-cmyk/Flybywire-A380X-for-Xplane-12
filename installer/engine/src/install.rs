use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use serde::Deserialize;
use sha2::{Digest, Sha256};

pub struct Answers {
    pub xplane: PathBuf,
    pub msfs_package: Option<PathBuf>,
    pub livery: Option<PathBuf>,
    pub sasl: Option<PathBuf>,
    pub sound_pack: Option<PathBuf>,
    pub converter: Option<PathBuf>,
    pub repo: String,
    pub cockpit_cap: Option<u32>,
    pub textures: Option<u32>,
    pub clean: bool,
}

pub enum Msg {
    Status(String),
    Log(String),
    Progress(f64),
    Done(Result<String, String>),
}

#[derive(Deserialize)]
struct Manifest {
    version: String,
    #[serde(default)]
    notes: String,
    files: Vec<ReleaseFile>,
    #[serde(default)]
    chain: Vec<ChainEntry>,
}

#[derive(Clone, Debug, Deserialize)]
pub struct ChainEntry {
    pub version: String,
    #[serde(default = "full")]
    pub kind: String,
    pub files: Vec<ReleaseFile>,
    #[serde(default)]
    pub deleted: Vec<String>,
}

fn full() -> String {
    "full".to_owned()
}

#[derive(Clone, Debug, Deserialize)]
pub struct ReleaseFile {
    pub name: String,
    pub size: u64,
    pub sha256: String,
    #[serde(default)]
    pub component: String,
    #[serde(skip)]
    pub url: String,
}

#[derive(Debug)]
pub struct Release {
    pub version: String,
    pub notes: String,
    pub files: Vec<ReleaseFile>,
    pub chain: Vec<ChainEntry>,
}

#[derive(Deserialize)]
struct GhRelease {
    #[serde(default)]
    draft: bool,
    #[serde(default)]
    assets: Vec<GhAsset>,
}

#[derive(Deserialize)]
struct GhAsset {
    name: String,
    browser_download_url: String,
}

pub fn github_get(url: &str) -> Result<ureq::Response, String> {
    ureq::get(url)
        .set("User-Agent", "fbw-a380x-installer")
        .set("Accept", "application/vnd.github+json")
        .call()
        .map_err(|e| match e {
            ureq::Error::Status(404, _) => format!("{url}: not found (the GitHub repository is private or was renamed)"),
            ureq::Error::Status(403, _) => format!("{url}: GitHub refused the request (rate limit: try again in an hour)"),
            e => format!("{url}: {e}"),
        })
}

pub fn pick_release(releases_json: &str) -> Result<(String, Vec<(String, String)>), String> {
    let releases: Vec<GhRelease> = serde_json::from_str(releases_json).map_err(|e| format!("GitHub's release list: {e}"))?;
    let release = releases
        .into_iter()
        .filter(|r| !r.draft)
        .find(|r| r.assets.iter().any(|a| a.name == "manifest.json"))
        .ok_or("no release on GitHub has a manifest.json yet")?;
    let manifest_url = release.assets.iter().find(|a| a.name == "manifest.json").map(|a| a.browser_download_url.clone()).unwrap_or_default();
    Ok((manifest_url, release.assets.into_iter().map(|a| (a.name, a.browser_download_url)).collect()))
}

pub fn latest_release(repo: &str) -> Result<Release, String> {
    let list = github_get(&format!("https://api.github.com/repos/{repo}/releases?per_page=30"))?
        .into_string()
        .map_err(|e| format!("GitHub's release list: {e}"))?;
    let (manifest_url, assets) = pick_release(&list)?;
    let text = github_get(&manifest_url)?.into_string().map_err(|e| format!("manifest.json: {e}"))?;
    release_from(&text, &assets, repo)
}

pub fn release_from(manifest_json: &str, assets: &[(String, String)], repo: &str) -> Result<Release, String> {
    let manifest: Manifest = serde_json::from_str(manifest_json).map_err(|e| format!("manifest.json: {e}"))?;
    let mut files = Vec::new();
    for mut f in manifest.files {
        f.url = assets
            .iter()
            .find(|(n, _)| *n == f.name)
            .map(|(_, u)| u.clone())
            .ok_or_else(|| format!("the release lists {} but GitHub has no such file", f.name))?;
        files.push(f);
    }
    let mut chain = manifest.chain;
    if chain.is_empty() {
        let aircraft = files.iter().filter(|f| wanted(f)).cloned().collect();
        chain.push(ChainEntry { version: manifest.version.clone(), kind: full(), files: aircraft, deleted: Vec::new() });
    } else {
        for entry in &mut chain {
            for f in &mut entry.files {
                f.url = format!("https://github.com/{repo}/releases/download/v{}/{}", entry.version, f.name);
            }
        }
    }
    Ok(Release { version: manifest.version, notes: manifest.notes, files, chain })
}

/// Which releases to apply: `(wipe, first)`. An install whose version is in
/// the chain takes only the patches after it; anything else (no install, one
/// older than the chain's base, or a clean reinstall asked for) is removed
/// with every trace and rebuilt from the base.
pub fn plan(chain: &[ChainEntry], installed: Option<&str>, clean: bool) -> (bool, usize) {
    match installed.filter(|_| !clean).and_then(|v| chain.iter().position(|e| e.version == v)) {
        Some(i) => (false, i + 1),
        None => (true, 0),
    }
}

/// A path from a patch's deleted list, inside the aircraft folder only.
fn inside(aircraft: &Path, rel: &str) -> Option<PathBuf> {
    let rel = Path::new(rel);
    if rel.as_os_str().is_empty() || rel.is_absolute() || rel.components().any(|c| !matches!(c, std::path::Component::Normal(_))) {
        return None;
    }
    Some(aircraft.join(rel))
}

/// Removes the aircraft and everything it and XPHFBW leave elsewhere:
/// X-Plane's `Output/preferences/fbw_a380x_*` and `xphfbw.json*`,
/// `Output/fbw-a380x`, `%LOCALAPPDATA%/XPHFBW`, and earlier downloads in
/// `temp` (all but `keep`, this run's own). The updater may be the program
/// running, which Windows will not delete, so it is moved to `OLD_UPDATER`
/// first. Returns what it removed.
pub fn wipe_traces(xplane: &Path, local_appdata: Option<&Path>, temp: &Path, keep: &Path) -> Result<Vec<PathBuf>, String> {
    let mut removed = Vec::new();
    let mut gone = |p: PathBuf| -> Result<(), String> {
        let r = if p.is_dir() { std::fs::remove_dir_all(&p) } else { std::fs::remove_file(&p) };
        match r {
            Ok(()) => {
                removed.push(p);
                Ok(())
            }
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(()),
            Err(e) => Err(format!("could not remove {}: {e}", p.display())),
        }
    };
    let updater = xplane.join(AIRCRAFT_DIR).join(UPDATER_NAME);
    if updater.exists() {
        let old = xplane.join(OLD_UPDATER);
        let _ = std::fs::create_dir_all(xplane.join("Output"));
        let _ = std::fs::remove_file(&old);
        std::fs::rename(&updater, &old).map_err(|e| format!("could not move {} out of the way: {e}", updater.display()))?;
    }
    gone(xplane.join(AIRCRAFT_DIR))?;
    let prefs = xplane.join("Output").join("preferences");
    if let Ok(entries) = std::fs::read_dir(&prefs) {
        for e in entries.flatten() {
            let name = e.file_name().to_string_lossy().to_ascii_lowercase();
            if name.starts_with("fbw_a380x_") || name.starts_with("xphfbw.json") {
                gone(e.path())?;
            }
        }
    }
    gone(xplane.join("Output").join("fbw-a380x"))?;
    if let Some(local) = local_appdata {
        gone(local.join("XPHFBW"))?;
    }
    if let Ok(entries) = std::fs::read_dir(temp) {
        for e in entries.flatten() {
            if e.file_name().to_string_lossy().starts_with("fbw-a380x-xp-") && e.path() != keep {
                gone(e.path())?;
            }
        }
    }
    Ok(removed)
}

#[derive(Default, serde::Serialize, Deserialize)]
pub struct Remembered {
    pub xplane: Option<PathBuf>,
    pub converter: Option<PathBuf>,
    pub cockpit_cap: Option<u32>,
}

fn remembered_path() -> Option<PathBuf> {
    Some(PathBuf::from(std::env::var("LOCALAPPDATA").ok()?).join("FlyByWire A380X XP").join("installer.json"))
}

impl Remembered {
    pub fn load() -> Self {
        remembered_path().and_then(|p| std::fs::read_to_string(p).ok()).and_then(|t| serde_json::from_str(&t).ok()).unwrap_or_default()
    }

    fn save(&self) {
        if let Some(p) = remembered_path() {
            if let Some(d) = p.parent() {
                let _ = std::fs::create_dir_all(d);
            }
            let _ = std::fs::write(p, serde_json::to_string_pretty(self).unwrap_or_default());
        }
    }
}

const AIRCRAFT_DIR: &str = "Aircraft/FlyByWire A380X";
const UPDATER_NAME: &str = "FlyByWire A380X Updater.exe";
const OLD_UPDATER: &str = "Output/fbw-a380x-updater.old.exe";
const TAKEOVER: &str = "--takeover";

pub fn installed_version(xplane: &Path) -> Option<String> {
    std::fs::read_to_string(xplane.join(AIRCRAFT_DIR).join(VERSION_FILE)).ok().map(|v| v.trim().to_owned()).filter(|v| !v.is_empty())
}

fn xplane_running() -> bool {
    use std::os::windows::process::CommandExt;
    std::process::Command::new("tasklist")
        .args(["/FI", "IMAGENAME eq X-Plane.exe", "/NH"])
        .creation_flags(0x0800_0000)
        .output()
        .map(|o| String::from_utf8_lossy(&o.stdout).to_ascii_lowercase().contains("x-plane.exe"))
        .unwrap_or(false)
}

/// `install.json` inside the release archive: what to do, in order.
#[derive(Deserialize)]
struct Steps {
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
    /// Install SASL, downloaded from 1-sim unless the user gave an archive
    /// (SASL is proprietary and never in a release): everything under the
    /// folder holding `64/win.xpl`, except SASL's sample module.
    Sasl {
        to: String,
        #[serde(default)]
        url: Option<String>,
        #[serde(default)]
        sha256: Option<String>,
    },
    /// Copy files from the user's own X-Plane into the aircraft folder.
    FromXplane { from: String, files: Vec<String>, to: String },
    /// Build an .acf from the user's own X-Plane aircraft file and the patch of this aircraft's own values.
    BuildAcf { template: String, patch: String, to: String },
    /// Download a file the release does not carry, checked against its SHA-256.
    Download { url: String, sha256: String, to: String },
    /// Install X-Plane Sounds' A380 pack, from the user's Downloads or its own download link: its banks with our dataref names, its GUIDs, and our .snd built on its own.
    SoundPack {
        #[serde(default)]
        url: Option<String>,
        #[serde(default)]
        zip_sha256: Option<String>,
        to: String,
        sha256: std::collections::BTreeMap<String, String>,
        banks: Vec<String>,
        copy: Vec<String>,
        renames: Vec<(String, String)>,
        snd_template: String,
        snd_patch: String,
        snd_to: String,
    },
}

const TEXT_PATCH_MAGIC: &str = "XP-TEXT-PATCH 1";

const ACF_PATCH_MAGIC: &str = "XP-ACF-PATCH 1";

const SASL_URL: &str = "https://1-sim.com/files/SASL-v3.23.3.zip";
const SASL_SHA256: &str = "f48d71adde3244e091c561ac6f9892b1dead3d98508d779ffb538224a50cceec";
const SOUND_PACK_URL: &str = "https://mega.nz/file/a4IUHByK#0a5HymbXbkhJYQfphs9SYyTBduyRgXJI36C3UiRtlJQ";
const SOUND_PACK_SHA256: &str = "e142a5c18629ac9dacb700e5e8ffdde11ae5bbef65b8e559c88de367cdfff0bb";

const VERSION_FILE: &str = "fbw_xp_installed_version.txt";
const TEXTURES_FILE: &str = "fbw_xp_textures.txt";
pub const FULL_TEXTURES: u32 = 4096;
pub const PACK_4K: &str = "textures-4k";
pub const PACK_8K: &str = "textures-8k";

pub fn packs_for(textures: u32) -> &'static [&'static str] {
    match textures {
        t if t > FULL_TEXTURES => &[PACK_4K, PACK_8K],
        FULL_TEXTURES => &[PACK_4K],
        _ => &[],
    }
}

pub fn installed_textures(xplane: &Path) -> Option<u32> {
    installed_version(xplane)?;
    Some(std::fs::read_to_string(xplane.join(AIRCRAFT_DIR).join(TEXTURES_FILE)).ok().and_then(|t| t.trim().parse().ok()).unwrap_or(FULL_TEXTURES))
}

fn wanted(f: &ReleaseFile) -> bool {
    !matches!(f.component.as_str(), "converter" | PACK_4K | PACK_8K)
}

fn apply_textures(aircraft: &Path, textures: u32, pack: &[ReleaseFile], dir: &Path, ctx: &Ctx) -> Result<(), String> {
    if !pack.is_empty() {
        let total = pack.iter().map(|f| f.size).sum::<u64>().max(1);
        let mut done = 0;
        std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
        for f in pack {
            ctx.status(format!("Downloading the {textures} px textures ({})...", f.name));
            download(&f.url, &dir.join(&f.name), f, ctx, &mut done, total)?;
        }
        for f in pack {
            ctx.status(format!("Installing the {textures} px textures..."));
            unzip(&dir.join(&f.name), aircraft)?;
        }
    }
    if textures <= FULL_TEXTURES {
        ctx.status(format!("Fitting the aircraft's textures to {textures} px..."));
        let s = crate::textures::cap_all(aircraft, textures, |i, n| ctx.send(Msg::Progress(i as f64 / n.max(1) as f64)))?;
        if s.resized > 0 {
            ctx.log(format!("Textures: {} resized to {textures} px", s.resized));
        }
    }
    std::fs::write(aircraft.join(TEXTURES_FILE), textures.to_string()).map_err(|e| e.to_string())
}

pub fn uninstall(xplane: &Path, ctx: &Ctx) -> Result<String, String> {
    if installed_version(xplane).is_none() && !xplane.join(AIRCRAFT_DIR).exists() {
        return Ok("The A380X is not installed".to_owned());
    }
    ctx.status("Removing the A380X and every trace of it...");
    let local = std::env::var("LOCALAPPDATA").ok().map(PathBuf::from);
    let temp = std::env::temp_dir();
    let removed = wipe_traces(xplane, local.as_deref(), &temp, &temp.join("fbw-a380x-xp-keep-nothing"))?;
    for p in &removed {
        ctx.log(format!("Removed {}", p.display()));
    }
    Ok(format!("Uninstalled the A380X from {}", xplane.join(AIRCRAFT_DIR).display()))
}

pub struct Ctx {
    pub json: bool,
}

impl Ctx {
    pub fn send(&self, m: Msg) {
        use std::io::Write as _;
        if self.json {
            let line = match m {
                Msg::Status(s) => serde_json::json!({ "kind": "status", "text": s }),
                Msg::Log(s) => serde_json::json!({ "kind": "log", "text": s }),
                Msg::Progress(p) => serde_json::json!({ "kind": "progress", "value": p.clamp(0.0, 1.0) }),
                Msg::Done(Ok(s)) => serde_json::json!({ "kind": "done", "ok": true, "text": s }),
                Msg::Done(Err(e)) => serde_json::json!({ "kind": "done", "ok": false, "text": e }),
            };
            let mut out = std::io::stdout().lock();
            let _ = writeln!(out, "{line}");
            let _ = out.flush();
            return;
        }
        match m {
            Msg::Status(s) | Msg::Log(s) => println!("{s}"),
            Msg::Done(Err(e)) => eprintln!("{e}"),
            _ => {}
        }
    }
    fn status(&self, s: impl Into<String>) {
        self.send(Msg::Status(s.into()));
    }
    fn log(&self, s: impl Into<String>) {
        self.send(Msg::Log(s.into()));
    }
}

pub fn run(a: Answers, ctx: &Ctx) -> bool {
    let result = work(&a, ctx);
    let ok = result.is_ok();
    ctx.send(Msg::Done(result));
    ok
}

/// Run by an updater from before the chain (0.2.x), as the first step of a
/// release's install.json: installs the latest release the way this
/// installer does (the old install and its traces wiped, then the base and
/// every update), so the old updater's own copy that follows changes
/// nothing, and leaves this program as the aircraft's updater.
pub fn takeover(xplane: PathBuf) -> Result<String, String> {
    let remembered = Remembered::load();
    let a = Answers {
        xplane,
        msfs_package: None,
        livery: None,
        sasl: None,
        sound_pack: None,
        converter: remembered.converter.clone(),
        repo: std::env::var("FBW_XP_REPO").unwrap_or_else(|_| crate::DEFAULT_REPO.to_owned()),
        cockpit_cap: remembered.cockpit_cap,
        textures: None,
        clean: false,
    };
    work(&a, &Ctx { json: false })
}

fn work(a: &Answers, ctx: &Ctx) -> Result<String, String> {
    let _ = std::fs::remove_file(a.xplane.join(OLD_UPDATER));
    ctx.status(format!("Checking GitHub ({}) for the latest release...", a.repo));
    let latest = latest_release(&a.repo)?;
    ctx.log(format!("Latest release: {}", latest.version));
    if !latest.notes.is_empty() {
        ctx.log(latest.notes.clone());
    }
    let before = installed_version(&a.xplane);
    let have_textures = installed_textures(&a.xplane);
    let textures = a.textures.unwrap_or(have_textures.unwrap_or(FULL_TEXTURES));
    let (mut wipe, mut first) = plan(&latest.chain, before.as_deref(), a.clean);
    let aircraft = a.xplane.join(AIRCRAFT_DIR);
    let work_root = std::env::temp_dir().join(format!("fbw-a380x-xp-{}", latest.version));
    let pack: Vec<ReleaseFile> = packs_for(textures)
        .iter()
        .flat_map(|component| latest.files.iter().filter(move |f| f.component == *component))
        .cloned()
        .collect();
    if textures > FULL_TEXTURES && !pack.iter().any(|f| f.component == PACK_8K) {
        return Err("This release has no 8K texture pack yet. Choose the 4K or 2K version.".into());
    }
    if !wipe && have_textures.is_some_and(|have| have < FULL_TEXTURES && textures > have) {
        ctx.log(format!("{textures} px textures need the aircraft downloaded again (it has {} px ones)", have_textures.unwrap_or_default()));
        wipe = true;
        first = 0;
    }
    if !wipe && first >= latest.chain.len() {
        if have_textures != Some(textures) {
            apply_textures(&aircraft, textures, &pack, &work_root.join("textures"), ctx)?;
            return Ok(format!("Textures set to {textures} px"));
        }
        return Ok(format!("Already up to date ({})", latest.version));
    }
    let todo = &latest.chain[first..];
    match (&before, wipe) {
        (Some(v), true) => ctx.log(format!("Installed: {v}. It and every trace of it are removed, then {} is installed from its base.", latest.version)),
        (Some(v), false) => ctx.log(format!("Installed: {v}. Applying {} update(s).", todo.len())),
        (None, _) => ctx.log(format!("Installing {} from its base and {} update(s).", latest.version, todo.len().saturating_sub(1))),
    }

    std::fs::create_dir_all(&work_root).map_err(|e| e.to_string())?;
    let converter_files: Vec<&ReleaseFile> = latest.files.iter().filter(|f| f.component == "converter" && a.converter.is_some()).collect();
    let total: u64 = (todo.iter().flat_map(|e| e.files.iter()).filter(|f| wanted(f)).map(|f| f.size).sum::<u64>() + converter_files.iter().map(|f| f.size).sum::<u64>()).max(1);
    let mut done = 0u64;
    for entry in todo {
        let dir = work_root.join(&entry.version);
        std::fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
        for f in entry.files.iter().filter(|f| wanted(f)) {
            let legacy = work_root.join(&f.name);
            if legacy.is_file() && !dir.join(&f.name).exists() {
                let _ = std::fs::rename(&legacy, dir.join(&f.name));
            }
            ctx.status(format!("Downloading {} ({})...", f.name, entry.version));
            download(&f.url, &dir.join(&f.name), f, ctx, &mut done, total)?;
        }
    }
    for f in &converter_files {
        ctx.status(format!("Downloading {}...", f.name));
        download(&f.url, &work_root.join(&f.name), f, ctx, &mut done, total)?;
    }

    if xplane_running() {
        ctx.status("Downloaded. Close X-Plane to finish installing...");
        while xplane_running() {
            std::thread::sleep(std::time::Duration::from_secs(2));
        }
    }

    if wipe {
        ctx.status("Removing the old install and every trace of it...");
        let local = std::env::var("LOCALAPPDATA").ok().map(PathBuf::from);
        for p in wipe_traces(&a.xplane, local.as_deref(), &std::env::temp_dir(), &work_root)? {
            ctx.log(format!("Removed {}", p.display()));
        }
    }

    for (k, entry) in todo.iter().enumerate() {
        ctx.status(format!("Installing {} {} ({} of {})...", if entry.kind == "patch" { "update" } else { "base" }, entry.version, k + 1, todo.len()));
        ctx.send(Msg::Progress(k as f64 / todo.len() as f64));
        let dir = work_root.join(&entry.version);
        let release = dir.join("release");
        let _ = std::fs::remove_dir_all(&release);
        for f in entry.files.iter().filter(|f| wanted(f)) {
            unzip(&dir.join(&f.name), &release)?;
        }
        if entry.kind == "patch" {
            for rel in &entry.deleted {
                match inside(&aircraft, rel) {
                    Some(p) => match std::fs::remove_file(&p) {
                        Ok(()) => ctx.log(format!("Removed {rel}")),
                        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
                        Err(e) => return Err(format!("could not remove {}: {e}", p.display())),
                    },
                    None => ctx.log(format!("Skipped a deletion outside the aircraft: {rel}")),
                }
            }
        }
        let steps: Steps = serde_json::from_str(&std::fs::read_to_string(release.join("install.json")).map_err(|e| format!("install.json: {e}"))?)
            .map_err(|e| format!("install.json: {e}"))?;
        run_steps(a, ctx, &steps, &release, &aircraft, &work_root)?;
        std::fs::write(aircraft.join(VERSION_FILE), &entry.version).map_err(|e| e.to_string())?;
        ctx.log(format!("{} {} installed", if entry.kind == "patch" { "Update" } else { "Base" }, entry.version));
    }

    if let Some(dir) = &a.converter {
        for f in &converter_files {
            ctx.status(format!("Installing the aircraft converter into {}...", dir.display()));
            unzip(&work_root.join(&f.name), dir)?;
        }
    }
    apply_textures(&aircraft, textures, &pack, &work_root.join("textures"), ctx)?;
    if let Some(cap) = a.cockpit_cap {
        ctx.status(format!("Resizing the cockpit textures to {cap} px..."));
        let summary = crate::textures::cap_cockpit(&aircraft, cap, |i, n| ctx.send(Msg::Progress(i as f64 / n.max(1) as f64)))?;
        ctx.log(format!(
            "Cockpit textures: {} resized to {cap} px ({:.0} MB on disk to {:.0} MB), {} already small enough",
            summary.resized,
            summary.bytes_before as f64 / 1_048_576.0,
            summary.bytes_after as f64 / 1_048_576.0,
            summary.kept
        ));
    }
    Remembered { xplane: Some(a.xplane.clone()), converter: a.converter.clone(), cockpit_cap: a.cockpit_cap }.save();
    ctx.send(Msg::Progress(1.0));
    Ok(match (before, wipe) {
        (Some(v), false) => format!("Updated {v} -> {} in {}", latest.version, aircraft.display()),
        (Some(v), true) => format!("Removed {v} and installed {} into {}", latest.version, aircraft.display()),
        (None, _) => format!("Installed {} into {}", latest.version, aircraft.display()),
    })
}

fn run_steps(a: &Answers, ctx: &Ctx, steps: &Steps, release: &Path, aircraft: &Path, work_dir: &Path) -> Result<(), String> {
    let mut sasl_zip: Option<PathBuf> = a.sasl.clone();
    for step in &steps.steps {
        match step {
            Step::Copy { from, to } => {
                ctx.status(format!("Copying {from}..."));
                copy_tree(&release.join(from), &aircraft.join(to))?;
            }
            Step::Run { args, .. } if args.first().is_some_and(|a| a == TAKEOVER) => {}
            Step::Run { program, args } => {
                if args.iter().any(|a| a.contains("{sasl}")) && sasl_zip.is_none() {
                    sasl_zip = Some(fetch_cached(SASL_URL, SASL_SHA256, work_dir, ctx)?);
                }
                let sasl = sasl_zip.clone().unwrap_or_default();
                let fill = |s: &str| -> Option<String> {
                    if s.contains("{livery}") && a.livery.is_none() {
                        return None;
                    }
                    Some(
                        s.replace("{xplane}", &a.xplane.to_string_lossy())
                            .replace("{aircraft}", &aircraft.to_string_lossy())
                            .replace("{msfs_package}", &a.msfs_package.as_ref().map(|p| p.to_string_lossy().into_owned()).unwrap_or_default())
                            .replace("{livery}", &a.livery.as_ref().map(|l| l.to_string_lossy().into_owned()).unwrap_or_default())
                            .replace("{sasl}", &sasl.to_string_lossy())
                            .replace("{release}", &release.to_string_lossy()),
                    )
                };
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
            Step::Sasl { to, url, sha256 } => {
                let zip = match &a.sasl {
                    Some(own) => own.clone(),
                    None => {
                        ctx.status("Downloading SASL from 1-sim...");
                        fetch_cached(url.as_deref().unwrap_or(SASL_URL), sha256.as_deref().unwrap_or(SASL_SHA256), work_dir, ctx)?
                    }
                };
                ctx.status("Installing SASL...");
                let n = install_sasl(&zip, &aircraft.join(to))?;
                ctx.log(format!("SASL: {n} files"));
                sasl_zip = Some(zip);
            }
            Step::FromXplane { from, files, to } => {
                ctx.status(format!("Copying {} from your X-Plane...", files.join(", ")));
                let dst = aircraft.join(to);
                std::fs::create_dir_all(&dst).map_err(|e| e.to_string())?;
                for f in files {
                    let src = a.xplane.join(from).join(f);
                    std::fs::copy(&src, dst.join(f))
                        .map_err(|e| format!("{}: {e} (this aircraft takes it from your own X-Plane 12)", src.display()))?;
                }
            }
            Step::BuildAcf { template, patch, to } => {
                ctx.status(format!("Building {to} from your X-Plane's {template}..."));
                let differing = build_acf(&a.xplane.join(template), &aircraft.join(patch), &aircraft.join(to))?;
                if differing > 0 {
                    ctx.log(format!("{to}: {differing} values in your {template} differ from the X-Plane version this release was made with"));
                }
            }
            Step::Download { url, sha256, to } => {
                ctx.status(format!("Downloading {to}..."));
                fetch_verified(url, sha256, &aircraft.join(to))?;
                ctx.log(format!("{to} downloaded and verified"));
            }
            Step::SoundPack { url, zip_sha256, to, sha256, banks, copy, renames, snd_template, snd_patch, snd_to } => {
                let zip_sha256 = zip_sha256.as_deref().unwrap_or(SOUND_PACK_SHA256);
                let local = a.sound_pack.clone().filter(|p| sha256_file(p).is_ok_and(|h| h.eq_ignore_ascii_case(zip_sha256)));
                let zip = match local {
                    Some(p) => {
                        ctx.log(format!("A380 Sound Pack: using {}", p.display()));
                        p
                    }
                    None => {
                        ctx.status("Downloading X-Plane Sounds' A380 Sound Pack...");
                        fetch_cached(url.as_deref().unwrap_or(SOUND_PACK_URL), zip_sha256, work_dir, ctx)?
                    }
                };
                ctx.status("Installing the A380 Sound Pack...");
                let files = read_sound_pack(&zip, sha256)?;
                let dst = aircraft.join(to);
                std::fs::create_dir_all(&dst).map_err(|e| e.to_string())?;
                for bank in banks {
                    let (patched, n) = rename_in_bank(&files[bank], renames)?;
                    std::fs::write(dst.join(bank), patched).map_err(|e| e.to_string())?;
                    ctx.log(format!("{bank}: {n} parameters renamed"));
                }
                for name in copy {
                    std::fs::write(dst.join(name), &files[name]).map_err(|e| e.to_string())?;
                }
                let patch = std::fs::read(aircraft.join(snd_patch)).map_err(|e| format!("{snd_patch}: {e}"))?;
                let snd = apply_text_patch(&latin1(&patch), &files[snd_template])?;
                std::fs::write(aircraft.join(snd_to), snd).map_err(|e| e.to_string())?;
            }

        }
    }
    Ok(())
}

fn download(url: &str, path: &Path, f: &ReleaseFile, ctx: &Ctx, done: &mut u64, total: u64) -> Result<(), String> {
    // Already downloaded and intact: keep it.
    if path.exists() && sha256_file(path).is_ok_and(|h| h.eq_ignore_ascii_case(&f.sha256)) {
        *done += f.size;
        ctx.log(format!("{} already downloaded and verified", f.name));
        return Ok(());
    }
    let response = ureq::get(url).set("User-Agent", "fbw-a380x-installer").call().map_err(|e| format!("download {}: {e}", f.name))?;
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

fn latin1(bytes: &[u8]) -> String {
    bytes.iter().map(|&b| b as char).collect()
}

fn build_acf(template: &Path, patch: &Path, out: &Path) -> Result<usize, String> {
    let base_text = latin1(&std::fs::read(template).map_err(|e| {
        format!("{}: {e} (this aircraft builds its .acf from your own X-Plane 12's Laminar Research Airbus A330-300)", template.display())
    })?);
    let mut base = std::collections::HashMap::new();
    for line in base_text.split('\n') {
        if let Some(rest) = line.strip_prefix("P ") {
            let (key, value) = rest.split_once(' ').unwrap_or((rest, ""));
            base.insert(key, value);
        }
    }
    let patch_text = latin1(&std::fs::read(patch).map_err(|e| format!("{}: {e}", patch.display()))?);
    let mut lines = patch_text.split('\n');
    if lines.next() != Some(ACF_PATCH_MAGIC) {
        return Err(format!("{} is not an .acf patch", patch.display()));
    }
    lines.next();
    let mut text = String::with_capacity(base_text.len() / 4);
    let mut differing = 0;
    for (i, line) in lines.enumerate() {
        if i > 0 {
            text.push('\n');
        }
        match line.strip_prefix('=') {
            Some(rest) => {
                let (key, expected) = rest.split_once(' ').unwrap_or((rest, ""));
                let value = base.get(key).ok_or_else(|| format!("{} has no {key}", template.display()))?;
                let raw: Vec<u8> = value.chars().map(|c| c as u32 as u8).collect();
                if !hex(&Sha256::digest(&raw))[..8].eq_ignore_ascii_case(expected) {
                    differing += 1;
                }
                text.push_str("P ");
                text.push_str(key);
                text.push(' ');
                text.push_str(value);
            }
            None => text.push_str(line),
        }
    }
    let bytes: Vec<u8> = text.chars().map(|c| c as u32 as u8).collect();
    std::fs::write(out, bytes).map_err(|e| format!("{}: {e}", out.display()))?;
    Ok(differing)
}

fn read_sound_pack(zip_path: &Path, expected: &std::collections::BTreeMap<String, String>) -> Result<std::collections::HashMap<String, Vec<u8>>, String> {
    let file = std::fs::File::open(zip_path).map_err(|e| format!("{}: {e}", zip_path.display()))?;
    let mut zip = zip::ZipArchive::new(file).map_err(|e| format!("{} is not a zip: {e}", zip_path.display()))?;
    let mut out = std::collections::HashMap::new();
    for i in 0..zip.len() {
        let mut entry = zip.by_index(i).map_err(|e| e.to_string())?;
        let name = entry.name().replace('\\', "/");
        let Some(wanted) = expected.keys().find(|k| name.ends_with(&format!("fmod/{k}"))) else { continue };
        let mut data = Vec::new();
        entry.read_to_end(&mut data).map_err(|e| e.to_string())?;
        out.insert(wanted.clone(), data);
    }
    for (name, hash) in expected {
        let data = out.get(name).ok_or_else(|| format!("{} has no fmod/{name}; is it X-Plane Sounds' A380_Sp(RR)(P)(C).zip?", zip_path.display()))?;
        if !hex(&Sha256::digest(data)).eq_ignore_ascii_case(hash) {
            return Err(format!("fmod/{name} in {} is not the version this release was made for", zip_path.display()));
        }
    }
    Ok(out)
}

fn rename_in_bank(data: &[u8], renames: &[(String, String)]) -> Result<(Vec<u8>, usize), String> {
    let mut out = data.to_vec();
    let mut n = 0;
    for (original, renamed) in renames {
        let (a, b) = (original.as_bytes(), renamed.as_bytes());
        if a.len() != b.len() || a.is_empty() {
            return Err(format!("bank rename {original} -> {renamed} changes the length"));
        }
        let mut i = 0;
        while i + a.len() <= out.len() {
            if &out[i..i + a.len()] == a {
                out[i..i + a.len()].copy_from_slice(b);
                n += 1;
                i += a.len();
            } else {
                i += 1;
            }
        }
    }
    Ok((out, n))
}

fn apply_text_patch(patch: &str, template: &[u8]) -> Result<Vec<u8>, String> {
    let mut lines = patch.split('\n');
    if lines.next() != Some(TEXT_PATCH_MAGIC) {
        return Err("not a text patch".into());
    }
    let expected = format!("TEMPLATE {}", hex(&Sha256::digest(template)));
    if lines.next() != Some(expected.as_str()) {
        return Err("the sound pack's .snd is not the one this release was made for".into());
    }
    let eol = if lines.next() == Some("EOL CRLF") { "\r\n" } else { "\n" };
    let base_text = latin1(template);
    let base: Vec<&str> = if base_text.contains("\r\n") { base_text.split("\r\n").collect() } else { base_text.split('\n').collect() };
    let mut out: Vec<&str> = Vec::new();
    for line in lines {
        if let Some(rest) = line.strip_prefix('=') {
            let (start, count) = rest.split_once(' ').ok_or_else(|| format!("bad patch line {line:?}"))?;
            let start: usize = start.parse().map_err(|_| format!("bad patch line {line:?}"))?;
            let count: usize = count.parse().map_err(|_| format!("bad patch line {line:?}"))?;
            out.extend(base.get(start..start + count).ok_or_else(|| format!("bad patch line {line:?}"))?);
        } else if let Some(rest) = line.strip_prefix('+') {
            out.push(rest);
        } else {
            return Err(format!("bad patch line {line:?}"));
        }
    }
    Ok(out.join(eol).chars().map(|c| c as u32 as u8).collect())
}

pub fn detect_sound_pack() -> Option<PathBuf> {
    let downloads = PathBuf::from(std::env::var("USERPROFILE").ok()?).join("Downloads");
    let mut found: Vec<PathBuf> = std::fs::read_dir(&downloads)
        .ok()?
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.file_name().is_some_and(|n| n.to_string_lossy().starts_with("A380_Sp(RR)(P)(C)") && n.to_string_lossy().ends_with(".zip")))
        .collect();
    found.sort();
    found.into_iter().next()
}

fn fetch_cached(url: &str, sha256: &str, dir: &Path, ctx: &Ctx) -> Result<PathBuf, String> {
    let path = dir.join(format!("{}.zip", &sha256[..16.min(sha256.len())]));
    if path.exists() && sha256_file(&path).is_ok_and(|h| h.eq_ignore_ascii_case(sha256)) {
        return Ok(path);
    }
    let got = if url.starts_with("https://mega.nz/") {
        mega_download(url, &path)?
    } else {
        stream_to(ureq::get(url).call().map_err(|e| format!("download {url}: {e}"))?.into_reader(), &path, |b| b.to_vec())?
    };
    if !got.eq_ignore_ascii_case(sha256) {
        let _ = std::fs::remove_file(&path);
        return Err(format!("{url} failed verification (SHA-256 {got}, expected {sha256})"));
    }
    ctx.log(format!("{url} downloaded and verified"));
    Ok(path)
}

fn stream_to(mut reader: impl Read, path: &Path, mut transform: impl FnMut(&[u8]) -> Vec<u8>) -> Result<String, String> {
    let mut out = std::fs::File::create(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let mut hasher = Sha256::new();
    let mut buf = vec![0u8; 1 << 20];
    loop {
        let n = reader.read(&mut buf).map_err(|e| format!("download: {e}"))?;
        if n == 0 {
            break;
        }
        let data = transform(&buf[..n]);
        hasher.update(&data);
        out.write_all(&data).map_err(|e| e.to_string())?;
    }
    Ok(hex(&hasher.finalize()))
}

fn base64url(s: &str) -> Result<Vec<u8>, String> {
    let mut out = Vec::with_capacity(s.len() * 3 / 4);
    let (mut acc, mut bits) = (0u32, 0);
    for c in s.bytes() {
        let v = match c {
            b'A'..=b'Z' => c - b'A',
            b'a'..=b'z' => c - b'a' + 26,
            b'0'..=b'9' => c - b'0' + 52,
            b'-' | b'+' => 62,
            b'_' | b'/' => 63,
            b'=' => break,
            _ => return Err(format!("bad base64 {s:?}")),
        };
        acc = (acc << 6) | v as u32;
        bits += 6;
        if bits >= 8 {
            bits -= 8;
            out.push((acc >> bits) as u8);
        }
    }
    Ok(out)
}

fn mega_link(link: &str) -> Result<(String, [u8; 16], u128), String> {
    let (path, key) = link.split_once('#').ok_or_else(|| format!("{link} has no key"))?;
    let handle = path.trim_end_matches('/').rsplit('/').next().unwrap_or_default().to_owned();
    let raw = base64url(key)?;
    if raw.len() != 32 || handle.is_empty() {
        return Err(format!("{link} is not a MEGA file link"));
    }
    let word = |i: usize| u32::from_be_bytes([raw[4 * i], raw[4 * i + 1], raw[4 * i + 2], raw[4 * i + 3]]);
    let mut file_key = [0u8; 16];
    for i in 0..4 {
        file_key[4 * i..4 * i + 4].copy_from_slice(&(word(i) ^ word(i + 4)).to_be_bytes());
    }
    let nonce = ((word(4) as u128) << 96) | ((word(5) as u128) << 64);
    Ok((handle, file_key, nonce))
}

struct AesCtr {
    cipher: aes::Aes128,
    counter: u128,
    stream: [u8; 16],
    used: usize,
}

impl AesCtr {
    fn new(key: &[u8; 16], counter: u128) -> Self {
        use aes::cipher::KeyInit;
        Self { cipher: aes::Aes128::new(key.into()), counter, stream: [0; 16], used: 16 }
    }

    fn apply(&mut self, data: &[u8]) -> Vec<u8> {
        use aes::cipher::BlockEncrypt;
        data.iter()
            .map(|b| {
                if self.used == 16 {
                    let mut block = self.counter.to_be_bytes().into();
                    self.cipher.encrypt_block(&mut block);
                    self.stream = block.into();
                    self.counter = self.counter.wrapping_add(1);
                    self.used = 0;
                }
                self.used += 1;
                b ^ self.stream[self.used - 1]
            })
            .collect()
    }
}

fn mega_download(link: &str, to: &Path) -> Result<String, String> {
    let (handle, key, nonce) = mega_link(link)?;
    let request = serde_json::json!([{ "a": "g", "g": 1, "ssl": 2, "p": handle }]).to_string();
    let reply: serde_json::Value = ureq::post("https://g.api.mega.co.nz/cs?id=1")
        .set("Content-Type", "application/json")
        .send_string(&request)
        .map_err(|e| format!("MEGA: {e}"))?
        .into_string()
        .map_err(|e| format!("MEGA: {e}"))
        .and_then(|t| serde_json::from_str(&t).map_err(|e| format!("MEGA: {e}")))?;
    let url = reply[0]["g"].as_str().ok_or_else(|| format!("MEGA refused {link}: {reply}"))?;
    let mut ctr = AesCtr::new(&key, nonce);
    stream_to(ureq::get(url).call().map_err(|e| format!("MEGA download: {e}"))?.into_reader(), to, |b| ctr.apply(b))
}

fn fetch_verified(url: &str, sha256: &str, to: &Path) -> Result<(), String> {
    let mut bytes = Vec::new();
    ureq::get(url)
        .call()
        .map_err(|e| format!("download {url}: {e}"))?
        .into_reader()
        .read_to_end(&mut bytes)
        .map_err(|e| format!("download {url}: {e}"))?;
    let got = hex(&Sha256::digest(&bytes));
    if !got.eq_ignore_ascii_case(sha256) {
        return Err(format!("{url} failed verification (SHA-256 {got}, expected {sha256})"));
    }
    if let Some(parent) = to.parent() {
        std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    std::fs::write(to, bytes).map_err(|e| format!("{}: {e}", to.display()))
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
        let json = ctx.json;
        std::thread::spawn(move || {
            let ctx = Ctx { json };
            for line in std::io::BufReader::new(err).lines().map_while(Result::ok) {
                ctx.log(line);
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

#[cfg(test)]
mod tests {
    use super::*;

    const RELEASES: &str = r#"[
        {"tag_name": "v0.3.0", "draft": true, "assets": [{"name": "manifest.json", "browser_download_url": "https://x/draft/manifest.json"}]},
        {"tag_name": "assets-2026-10-06", "draft": false, "prerelease": true, "assets": [{"name": "xplane.part01.zip", "browser_download_url": "https://x/a/xplane.part01.zip"}]},
        {"tag_name": "v0.2.0", "draft": false, "assets": [
            {"name": "aircraft.part01.zip", "browser_download_url": "https://x/v2/aircraft.part01.zip"},
            {"name": "converter.zip", "browser_download_url": "https://x/v2/converter.zip"},
            {"name": "manifest.json", "browser_download_url": "https://x/v2/manifest.json"}
        ]}
    ]"#;

    #[test]
    fn the_newest_published_release_with_a_manifest_is_chosen() {
        let (manifest, assets) = pick_release(RELEASES).unwrap();
        assert_eq!(manifest, "https://x/v2/manifest.json");
        assert_eq!(assets.len(), 3);
        assert!(pick_release(r#"[{"draft": false, "assets": []}]"#).is_err());
    }

    #[test]
    fn manifest_files_get_their_github_download_urls() {
        let (_, assets) = pick_release(RELEASES).unwrap();
        let manifest = r#"{"version": "0.2.0", "notes": "n", "files": [
            {"name": "aircraft.part01.zip", "size": 10, "sha256": "ab"},
            {"name": "converter.zip", "size": 5, "sha256": "cd", "component": "converter"}
        ]}"#;
        let r = release_from(manifest, &assets, "o/r").unwrap();
        assert_eq!(r.version, "0.2.0");
        assert_eq!(r.files[0].url, "https://x/v2/aircraft.part01.zip");
        assert_eq!(r.files[1].component, "converter");
        assert_eq!(r.chain.len(), 1, "a release from before the chain is its own base");
        assert_eq!(r.chain[0].files.len(), 1, "the converter is not part of the aircraft");
        let missing = r#"{"version": "0.2.0", "files": [{"name": "gone.zip", "size": 1, "sha256": "00"}]}"#;
        assert!(release_from(missing, &assets, "o/r").unwrap_err().contains("gone.zip"));
    }

    fn entry(version: &str, kind: &str) -> ChainEntry {
        ChainEntry { version: version.into(), kind: kind.into(), files: Vec::new(), deleted: Vec::new() }
    }

    #[test]
    fn an_install_on_the_chain_takes_only_the_updates_after_it() {
        let chain = [entry("0.3.0", "full"), entry("0.3.1", "patch"), entry("0.3.2", "patch")];
        assert_eq!(plan(&chain, Some("0.3.1"), false), (false, 2), "only the newest update");
        assert_eq!(plan(&chain, Some("0.3.0"), false), (false, 1));
        assert_eq!(plan(&chain, Some("0.3.2"), false), (false, 3), "up to date");
        assert_eq!(plan(&chain, None, false), (true, 0), "a new install is the base and every update");
        assert_eq!(plan(&chain, Some("0.2.7"), false), (true, 0), "an install older than the base is replaced");
        assert_eq!(plan(&chain, Some("0.3.1"), true), (true, 0), "a clean reinstall when asked for");
    }

    #[test]
    fn chain_files_download_from_their_own_release() {
        let manifest = r#"{"version": "0.3.1", "files": [{"name": "p.zip", "size": 1, "sha256": "aa"}], "chain": [
            {"version": "0.3.0", "kind": "full", "files": [{"name": "b1.zip", "size": 1, "sha256": "bb"}]},
            {"version": "0.3.1", "kind": "patch", "files": [{"name": "p.zip", "size": 1, "sha256": "aa"}], "deleted": ["objects/old.obj"]}
        ]}"#;
        let r = release_from(manifest, &[("p.zip".into(), "https://x/p.zip".into())], "o/r").unwrap();
        assert_eq!(r.chain[0].files[0].url, "https://github.com/o/r/releases/download/v0.3.0/b1.zip");
        assert_eq!(r.chain[1].files[0].url, "https://github.com/o/r/releases/download/v0.3.1/p.zip");
        assert_eq!(r.chain[1].deleted, ["objects/old.obj"]);
    }

    #[test]
    fn each_texture_size_takes_its_own_pack_and_the_aircraft_never_takes_a_pack_as_its_base() {
        assert!(packs_for(2048).is_empty());
        assert_eq!(packs_for(4096), [PACK_4K]);
        assert_eq!(packs_for(8192), [PACK_4K, PACK_8K], "the 8K pack goes over the 4K one");
        let file = |component: &str| ReleaseFile { name: "x.zip".into(), size: 1, sha256: String::new(), component: component.into(), url: String::new() };
        assert!(wanted(&file("")) && wanted(&file("aircraft")));
        assert!(!wanted(&file(PACK_4K)) && !wanted(&file(PACK_8K)) && !wanted(&file("converter")));
    }

    #[test]
    fn a_patch_can_only_delete_inside_the_aircraft() {
        let aircraft = Path::new("C:/X/Aircraft/A");
        assert_eq!(inside(aircraft, "objects/a.obj"), Some(aircraft.join("objects/a.obj")));
        assert_eq!(inside(aircraft, "../b.txt"), None);
        assert_eq!(inside(aircraft, "C:/Windows/x"), None);
        assert_eq!(inside(aircraft, ""), None);
    }

    #[test]
    fn a_clean_install_removes_every_trace_and_nothing_else() {
        let root = std::env::temp_dir().join(format!("fbw-wipe-test-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        let xplane = root.join("X-Plane 12");
        let local = root.join("Local");
        let temp = root.join("Temp");
        for d in [xplane.join(AIRCRAFT_DIR).join("objects"), xplane.join("Output/preferences"), xplane.join("Output/fbw-a380x/diagnostics"), local.join("XPHFBW/cache"), temp.join("fbw-a380x-xp-0.2.7"), temp.join("fbw-a380x-xp-0.3.0"), temp.join("other")] {
            std::fs::create_dir_all(d).unwrap();
        }
        for f in ["Output/preferences/fbw_a380x_airframe.json", "Output/preferences/xphfbw.json", "Output/preferences/xphfbw.json.bak", "Output/preferences/X-Plane.prf", &format!("{AIRCRAFT_DIR}/{UPDATER_NAME}")] {
            std::fs::write(xplane.join(f), "x").unwrap();
        }
        let keep = temp.join("fbw-a380x-xp-0.3.0");
        let removed = wipe_traces(&xplane, Some(&local), &temp, &keep).unwrap();
        assert!(!xplane.join(AIRCRAFT_DIR).exists());
        assert!(!xplane.join("Output/preferences/fbw_a380x_airframe.json").exists());
        assert!(!xplane.join("Output/preferences/xphfbw.json.bak").exists());
        assert!(!xplane.join("Output/fbw-a380x").exists());
        assert!(!local.join("XPHFBW").exists());
        assert!(!temp.join("fbw-a380x-xp-0.2.7").exists());
        assert!(xplane.join("Output/preferences/X-Plane.prf").exists(), "X-Plane's own settings stay");
        assert!(xplane.join(OLD_UPDATER).exists(), "the updater, maybe the program running, is moved aside, not deleted");
        assert!(keep.exists(), "this run's downloads stay");
        assert!(temp.join("other").exists());
        assert_eq!(removed.len(), 7);
        let _ = std::fs::remove_dir_all(&root);
    }

    #[test]
    fn an_old_updater_hands_the_install_to_this_one_and_this_one_skips_that_step() {
        let steps: Steps = serde_json::from_str(r#"{"aircraft": "Aircraft/FlyByWire A380X", "steps": [
            {"kind": "run", "program": "FlyByWire-A380X-XP-Installer.exe", "args": ["--takeover", "{xplane}"]},
            {"kind": "sasl", "to": "plugins/sasl"},
            {"kind": "copy", "from": "aircraft", "to": ""}
        ]}"#)
        .unwrap();
        assert!(matches!(&steps.steps[0], Step::Run { args, .. } if args[0] == TAKEOVER));
        assert_eq!(steps.steps.len(), 3);
    }

    #[test]
    #[ignore]
    fn the_live_github_release_resolves() {
        let r = latest_release(crate::DEFAULT_REPO).unwrap();
        assert!(!r.files.is_empty());
        assert!(r.files.iter().all(|f| f.url.starts_with("https://github.com/") && !f.sha256.is_empty()));
        eprintln!("{} with {} files", r.version, r.files.len());
    }

    fn short(v: &str) -> String {
        hex(&Sha256::digest(v.as_bytes()))[..8].to_string()
    }

    #[test]
    fn builds_an_acf_from_a_template_and_a_patch() {
        let dir = std::env::temp_dir().join("fbw-acf-patch-test");
        std::fs::create_dir_all(&dir).unwrap();
        let template = dir.join("template.acf");
        let patch = dir.join("aircraft.acfpatch");
        let out = dir.join("aircraft.acf");
        std::fs::write(&template, "I\n1200 Version\nP a 1\nP b 2\nP c 3\n").unwrap();
        let patch_text = format!("{ACF_PATCH_MAGIC}\nTEMPLATE x\nI\n1200 Version\n={} {}\nP b 5\n=c {}\nP d 9\n", "a", short("1"), short("3"));
        std::fs::write(&patch, patch_text).unwrap();
        assert_eq!(build_acf(&template, &patch, &out).unwrap(), 0);
        assert_eq!(std::fs::read_to_string(&out).unwrap(), "I\n1200 Version\nP a 1\nP b 5\nP c 3\nP d 9\n");
        std::fs::write(&template, "I\n1200 Version\nP a 1\nP b 2\nP c 4\n").unwrap();
        assert_eq!(build_acf(&template, &patch, &out).unwrap(), 1);
    }

    #[test]
    fn aes_ctr_matches_the_nist_example() {
        let key: [u8; 16] = [0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c];
        let counter = u128::from_be_bytes([0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff]);
        let plain = [
            0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a, 0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c,
            0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51,
        ];
        let mut ctr = AesCtr::new(&key, counter);
        let mut out = ctr.apply(&plain[..5]);
        out.extend(ctr.apply(&plain[5..]));
        assert_eq!(hex(&out), "874d6191b620e3261bef6864990db6ce9806f66b7970fdff8617187bb9fffdff");
    }

    #[test]
    fn reads_a_mega_file_link() {
        let (handle, key, nonce) = mega_link(SOUND_PACK_URL).unwrap();
        assert_eq!(handle, "a4IUHByK");
        assert_eq!(nonce as u64, 0);
        assert_ne!(key, [0; 16]);
        assert!(mega_link("https://mega.nz/file/a4IUHByK").is_err());
    }

    #[test]
    #[ignore]
    fn downloads_the_sound_pack_and_sasl() {
        let dir = std::env::temp_dir().join("fbw-installer-download-test");
        std::fs::create_dir_all(&dir).unwrap();
        assert_eq!(mega_download(SOUND_PACK_URL, &dir.join("pack.zip")).unwrap(), SOUND_PACK_SHA256);
        let sasl = stream_to(ureq::get(SASL_URL).call().unwrap().into_reader(), &dir.join("sasl.zip"), |b| b.to_vec()).unwrap();
        assert_eq!(sasl, SASL_SHA256);
    }
}
