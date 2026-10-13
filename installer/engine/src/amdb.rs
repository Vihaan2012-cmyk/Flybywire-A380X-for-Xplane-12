use serde::Deserialize;

pub const REPO: &str = "Vihaan2012-cmyk/Free-Airport-Mapping-DB";
const UNINSTALL_KEY: &str = r"Software\Microsoft\Windows\CurrentVersion\Uninstall\{6B2E8F3A-4C1D-4E7B-9A55-3F0C2D8E1A47}_is1";
const HEALTH_URL: &str = "http://127.0.0.1:8770/v1";
const SETUP_PREFIX: &str = "AMDB-Bridge-Setup-";

#[derive(Deserialize)]
struct GhRelease {
    #[serde(default)]
    draft: bool,
    #[serde(default)]
    prerelease: bool,
    #[serde(default)]
    body: String,
    #[serde(default)]
    html_url: String,
    #[serde(default)]
    assets: Vec<GhAsset>,
}

#[derive(Deserialize)]
struct GhAsset {
    name: String,
}

pub struct Latest {
    pub version: String,
    pub notes: String,
    pub page: String,
}

fn reg_value(name: &str) -> Option<String> {
    use std::os::windows::process::CommandExt;
    for hive in ["HKCU", "HKLM"] {
        let out = std::process::Command::new("reg")
            .args(["query", &format!("{hive}\\{UNINSTALL_KEY}"), "/v", name])
            .creation_flags(0x0800_0000)
            .output()
            .ok()?;
        let text = String::from_utf8_lossy(&out.stdout);
        if let Some(value) = text.lines().find_map(|l| l.trim().strip_prefix(name)).and_then(|rest| rest.trim().split_once("REG_SZ")) {
            return Some(value.1.trim().to_owned());
        }
    }
    None
}

pub fn installed() -> Option<String> {
    reg_value("DisplayVersion").filter(|v| !v.is_empty())
}

pub fn running() -> Option<String> {
    let agent = ureq::AgentBuilder::new().timeout(std::time::Duration::from_millis(800)).build();
    let reply: serde_json::Value = serde_json::from_str(&agent.get(HEALTH_URL).call().ok()?.into_string().ok()?).ok()?;
    (reply["service"] == "amdb-bridge").then(|| reply["version"].as_str().unwrap_or_default().to_owned())
}

pub fn version_of_setup(name: &str) -> Option<String> {
    name.strip_prefix(SETUP_PREFIX)?.strip_suffix(".exe").map(str::to_owned)
}

pub fn pick_latest(releases_json: &str) -> Result<Latest, String> {
    let releases: Vec<GhRelease> = serde_json::from_str(releases_json).map_err(|e| format!("GitHub's release list: {e}"))?;
    releases
        .into_iter()
        .filter(|r| !r.draft && !r.prerelease)
        .find_map(|r| {
            let asset = r.assets.into_iter().find(|a| version_of_setup(&a.name).is_some())?;
            Some(Latest { version: version_of_setup(&asset.name)?, notes: r.body, page: r.html_url })
        })
        .ok_or_else(|| "no AMDB Bridge release on GitHub has a setup program".to_owned())
}

pub fn latest() -> Result<Latest, String> {
    let list = crate::install::github_get(&format!("https://api.github.com/repos/{REPO}/releases?per_page=20"))?
        .into_string()
        .map_err(|e| format!("GitHub's release list: {e}"))?;
    pick_latest(&list)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_newest_release_with_a_setup_program_is_the_latest() {
        let json = r#"[
            {"draft": true, "assets": [{"name": "AMDB-Bridge-Setup-9.9.9.exe"}]},
            {"prerelease": false, "body": "notes", "html_url": "p", "assets": [{"name": "AMDB-Navdata-1.2.10.zip"}, {"name": "AMDB-Bridge-Setup-1.2.10.exe"}]},
            {"assets": [{"name": "AMDB-Bridge-Setup-1.2.9.exe"}]}
        ]"#;
        let l = pick_latest(json).unwrap();
        assert_eq!((l.version.as_str(), l.page.as_str(), l.notes.as_str()), ("1.2.10", "p", "notes"));
        assert_eq!(version_of_setup("AMDB-Bridge-Setup-1.2.10.zip"), None);
    }
}
