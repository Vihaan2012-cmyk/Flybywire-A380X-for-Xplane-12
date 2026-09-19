//! The settings the page edits, kept where their readers read them:
//! FlyByWire's keys in its stored data (NXDataStore, `A380X_<KEY>`) and, for
//! the flyPad settings the plugin mirrors into variables, its ini too; the
//! app's own (`xphfbw.*`) in `xphfbw.json`.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use fbw_a380_systems::settings_files as files;
use serde_json::{Map, Value};

/// Keys FlyByWire stores JSON-encoded (NXDataStore.set rather than setLegacy).
const JSON_ENCODED: &[&str] = &["ACARS_PROVIDER", "CONFIG_ATIS_SRC"];

/// X-Plane 12's folder, from X-Plane's own install record.
pub fn find_xplane() -> Option<PathBuf> {
    let local = std::env::var("LOCALAPPDATA").ok()?;
    let text = std::fs::read_to_string(Path::new(&local).join("x-plane_install_12.txt")).ok()?;
    text.lines().map(str::trim).find(|l| !l.is_empty()).map(PathBuf::from)
}

fn read_json(path: &Path) -> Map<String, Value> {
    std::fs::read_to_string(path)
        .ok()
        .and_then(|t| serde_json::from_str::<Value>(&t).ok())
        .and_then(|v| v.as_object().cloned())
        .unwrap_or_default()
}

fn write_json(path: &Path, map: &Map<String, Value>) -> Result<(), String> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    }
    let text = serde_json::to_string_pretty(&Value::Object(map.clone())).map_err(|e| e.to_string())?;
    let partial = path.with_extension("json.part");
    std::fs::write(&partial, text).map_err(|e| e.to_string())?;
    std::fs::rename(&partial, path).map_err(|e| e.to_string())
}

/// Every stored setting, by the key the page uses.
pub fn load(root: &Path) -> Map<String, Value> {
    let mut out = Map::new();
    let prefix = files::stored_key("");
    for (k, v) in read_json(&files::datastore_path(root)) {
        let Some(key) = k.strip_prefix(&prefix) else { continue };
        let text = v.as_str().unwrap_or_default();
        // A JSON-encoded string comes back as its content.
        let value = serde_json::from_str::<String>(text).unwrap_or_else(|_| text.to_string());
        out.insert(key.to_string(), Value::String(value));
    }
    if let Ok(text) = std::fs::read_to_string(files::flypad_ini_path(root)) {
        for (k, v) in files::parse_ini(&text) {
            if let Some(key) = k.strip_prefix(&prefix) {
                out.insert(key.to_string(), Value::String(v));
            }
        }
    }
    for (k, v) in read_json(&files::app_settings_path(root)) {
        out.insert(format!("xphfbw.{k}"), v);
    }
    out
}

/// Store what the page applied. Race/desync rule 8
/// (`docs/briefs/xphfbw-js-bridge.md`): the datastore JSON, the flyPad ini
/// and xphfbw.json are all written by more than one process (this app, and
/// the plugin's own flyPad-settings writer, `js_bridge.rs`'s
/// `FlyPadSettings::set`), so the whole read-modify-write below holds the
/// same named mutex both sides use (`app_settings::with_settings_lock`).
pub fn save(root: &Path, values: &Map<String, Value>) -> Result<(), String> {
    fbw_a380_systems::app_settings::with_settings_lock(|| save_locked(root, values))
}

fn save_locked(root: &Path, values: &Map<String, Value>) -> Result<(), String> {
    let mut datastore = read_json(&files::datastore_path(root));
    let ini_path = files::flypad_ini_path(root);
    let mut ini: BTreeMap<String, String> = std::fs::read_to_string(&ini_path).map(|t| files::parse_ini(&t)).unwrap_or_default();
    let mut app = read_json(&files::app_settings_path(root));
    let (mut datastore_changed, mut ini_changed, mut app_changed) = (false, false, false);

    for (key, value) in values {
        if let Some(own) = key.strip_prefix("xphfbw.") {
            app.insert(own.to_string(), value.clone());
            app_changed = true;
            continue;
        }
        let text = match value {
            Value::String(s) => s.clone(),
            other => other.to_string(),
        };
        let stored = files::stored_key(key);
        let encoded = if JSON_ENCODED.contains(&key.as_str()) { Value::String(text.clone()).to_string() } else { text.clone() };
        datastore.insert(stored.clone(), Value::String(encoded));
        datastore_changed = true;
        if files::flypad_owns(key) {
            ini.insert(stored, text);
            ini_changed = true;
        }
    }
    if datastore_changed {
        write_json(&files::datastore_path(root), &datastore)?;
    }
    if ini_changed {
        if let Some(dir) = ini_path.parent() {
            std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
        }
        std::fs::write(&ini_path, files::write_ini(&ini)).map_err(|e| e.to_string())?;
    }
    if app_changed {
        write_json(&files::app_settings_path(root), &app)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn settings_round_trip_to_where_their_readers_look() {
        let root = std::env::temp_dir().join(format!("xphfbw-settings-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        let mut values = Map::new();
        values.insert("ACARS_PROVIDER".into(), Value::String("HOPPIE".into()));
        values.insert("CONFIG_SELF_TEST_TIME".into(), Value::String("12".into()));
        values.insert("xphfbw.stateDumps".into(), Value::Bool(true));
        save(&root, &values).unwrap();

        let datastore = read_json(&files::datastore_path(&root));
        assert_eq!(datastore["A380X_ACARS_PROVIDER"], Value::String("\"HOPPIE\"".into()), "NXDataStore.get decodes JSON");
        assert_eq!(datastore["A380X_CONFIG_SELF_TEST_TIME"], Value::String("12".into()), "getLegacy reads it raw");
        let loaded = load(&root);
        assert_eq!(loaded["ACARS_PROVIDER"], Value::String("HOPPIE".into()));
        assert_eq!(loaded["CONFIG_SELF_TEST_TIME"], Value::String("12".into()));
        assert_eq!(loaded["xphfbw.stateDumps"], Value::Bool(true));
        let _ = std::fs::remove_dir_all(&root);
    }
}
