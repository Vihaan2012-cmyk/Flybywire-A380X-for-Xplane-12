mod amdb;
mod chain;
mod install;
mod textures;

use std::path::PathBuf;

use install::{Answers, Ctx, Remembered};

pub const DEFAULT_REPO: &str = "Vihaan2012-cmyk/Flybywire-A380X-for-Xplane-12";

fn arg(args: &[String], name: &str) -> Option<String> {
    args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned()
}

fn repo(args: &[String]) -> String {
    arg(args, "--repo").or_else(|| std::env::var("FBW_XP_REPO").ok()).unwrap_or_else(|| DEFAULT_REPO.to_owned())
}

fn xplane(args: &[String]) -> Option<PathBuf> {
    arg(args, "--xplane")
        .map(PathBuf::from)
        .or_else(|| Remembered::load().xplane.filter(|p| p.join("Aircraft").is_dir()))
        .or_else(install::detect_xplane)
}

fn usage() -> ! {
    eprintln!(
        "usage:\n  engine detect\n  engine status [--xplane DIR] [--repo OWNER/NAME]\n  engine install --xplane DIR [--clean] [--cockpit-cap PX] [--textures 2048|4096|8192] [--sasl ZIP] [--sound-pack ZIP] [--converter DIR] [--repo OWNER/NAME]\n  engine uninstall --xplane DIR\n  engine amdb-status\n  engine --takeover DIR\n  engine --cockpit-textures PX DIR"
    );
    std::process::exit(2);
}

fn finish(result: Result<String, String>) {
    let ok = result.is_ok();
    Ctx { json: true }.send(install::Msg::Done(result));
    if !ok {
        std::process::exit(1);
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if let Some(i) = args.iter().position(|a| a == "--cockpit-textures") {
        let cap = args.get(i + 1).and_then(|c| c.parse().ok()).unwrap_or(textures::COCKPIT_REDUCED);
        let Some(aircraft) = args.get(i + 2).map(PathBuf::from) else { usage() };
        match textures::cap_cockpit(&aircraft, cap, |_, _| {}) {
            Ok(s) => println!("{} resized to {cap} px ({} -> {} bytes), {} kept", s.resized, s.bytes_before, s.bytes_after, s.kept),
            Err(e) => {
                eprintln!("{e}");
                std::process::exit(1);
            }
        }
        return;
    }
    if let Some(i) = args.iter().position(|a| a == "--takeover") {
        let Some(xplane) = args.get(i + 1).map(PathBuf::from) else { usage() };
        match install::takeover(xplane) {
            Ok(s) => println!("{s}"),
            Err(e) => {
                eprintln!("{e}");
                std::process::exit(1);
            }
        }
        return;
    }
    match args.get(1).map(String::as_str) {
        Some("detect") => {
            let found = xplane(&args);
            println!("{}", serde_json::json!({ "xplane": found, "soundPack": install::detect_sound_pack() }));
        }
        Some("status") => {
            let found = xplane(&args);
            let installed = found.as_deref().and_then(install::installed_version);
            let textures = found.as_deref().and_then(install::installed_textures);
            let (latest, notes, error) = match install::latest_release(&repo(&args)) {
                Ok(r) => (Some(r.version), r.notes, None),
                Err(e) => (None, String::new(), Some(e)),
            };
            println!(
                "{}",
                serde_json::json!({ "xplane": found, "installed": installed, "textures": textures, "latest": latest, "notes": notes, "error": error })
            );
        }
        Some("install") => {
            let Some(xplane) = arg(&args, "--xplane").map(PathBuf::from) else { usage() };
            let answers = Answers {
                xplane,
                msfs_package: install::detect_fbw_package(),
                livery: None,
                sasl: arg(&args, "--sasl").map(PathBuf::from),
                sound_pack: arg(&args, "--sound-pack").map(PathBuf::from).or_else(install::detect_sound_pack),
                converter: arg(&args, "--converter").map(PathBuf::from),
                repo: repo(&args),
                cockpit_cap: arg(&args, "--cockpit-cap").and_then(|c| c.parse().ok()),
                textures: arg(&args, "--textures").and_then(|c| c.parse().ok()),
                clean: args.iter().any(|a| a == "--clean"),
            };
            if !install::run(answers, &Ctx { json: true }) {
                std::process::exit(1);
            }
        }
        Some("uninstall") => {
            let Some(xplane) = arg(&args, "--xplane").map(PathBuf::from) else { usage() };
            finish(install::uninstall(&xplane, &Ctx { json: true }));
        }
        Some("amdb-status") => {
            let (latest, page, notes, error) = match amdb::latest() {
                Ok(l) => (Some(l.version), Some(l.page), l.notes, None),
                Err(e) => (None, None, String::new(), Some(e)),
            };
            println!(
                "{}",
                serde_json::json!({ "installed": amdb::installed(), "running": amdb::running(), "latest": latest, "page": page, "notes": notes, "error": error })
            );
        }
        _ => usage(),
    }
}
