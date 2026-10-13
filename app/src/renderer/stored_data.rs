//! `storedData` (`window.__xphfbw.storedData`, agent E): FlyByWire's
//! stored-data JSON (NXDataStore's backing file), read/written the way rule
//! 8 requires (docs/briefs/xphfbw-js-bridge.md): every writer holds the
//! named mutex `Local\XPHFBW_settings_files` around a read-modify-write,
//! and writes go to a temp file that is then renamed into place. This is a
//! separate critical section from `app/src/settings.rs`'s own save path,
//! but the same mutex name makes the two (and the plugin's writer)
//! mutually exclusive.

use std::path::Path;

use serde_json::{Map, Value};

use fbw_a380_systems::settings_files;
use fbw_a380_systems::xphfbw_bridge::NamedMutex;

const MUTEX_NAME: &str = "Local\\XPHFBW_settings_files";

fn read_json(path: &Path) -> Map<String, Value> {
    std::fs::read_to_string(path)
        .ok()
        .and_then(|t| serde_json::from_str::<Value>(&t).ok())
        .and_then(|v| v.as_object().cloned())
        .unwrap_or_default()
}

fn write_json(path: &Path, map: &Map<String, Value>) -> std::io::Result<()> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let text = serde_json::to_string_pretty(&Value::Object(map.clone())).unwrap_or_else(|_| "{}".to_string());
    let partial = path.with_extension("json.part");
    std::fs::write(&partial, text)?;
    std::fs::rename(&partial, path)
}

/// `op` is `"get"`, `"set"`, `"delete"` or `"search"` (`key` a prefix,
/// returning a JSON array of `{"key":..,"data":..}`, matching MSFS's
/// `SearchStoredData`/`DataStorageSearchData`); `value` is used by `"set"`
/// only. The whole read-modify-write runs under the named mutex.
pub fn run(xp_root: &Path, op: &str, key: &str, value: &str) -> String {
    let Some(mutex) = NamedMutex::create(MUTEX_NAME) else {
        // No cross-process guarantee without the mutex; refuse rather than
        // race the plugin or another view.
        return String::new();
    };
    mutex.with(|| {
        let path = settings_files::datastore_path(xp_root);
        match op {
            "get" => {
                let map = read_json(&path);
                map.get(key).and_then(|v| v.as_str()).map(str::to_string).unwrap_or_default()
            }
            "set" => {
                let mut map = read_json(&path);
                map.insert(key.to_string(), Value::String(value.to_string()));
                let _ = write_json(&path, &map);
                String::new()
            }
            "delete" => {
                let mut map = read_json(&path);
                map.remove(key);
                let _ = write_json(&path, &map);
                String::new()
            }
            "search" => {
                let map = read_json(&path);
                let matches: Vec<Value> =
                    map.iter().filter(|(k, _)| k.starts_with(key)).map(|(k, v)| serde_json::json!({ "key": k, "data": v })).collect();
                serde_json::to_string(&matches).unwrap_or_else(|_| "[]".to_string())
            }
            _ => String::new(),
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn get_set_delete_and_search_round_trip_through_the_datastore_json() {
        let root = std::env::temp_dir().join(format!("xphfbw-storeddata-{}-{}", std::process::id(), line!()));
        let _ = std::fs::remove_dir_all(&root);

        assert_eq!(run(&root, "get", "A380X_FOO", ""), "", "missing key reads as empty");

        run(&root, "set", "A380X_FOO", "bar");
        assert_eq!(run(&root, "get", "A380X_FOO", ""), "bar");

        run(&root, "set", "A380X_FOO2", "baz");
        let found = run(&root, "search", "A380X_FOO", "");
        let parsed: Value = serde_json::from_str(&found).unwrap();
        let keys: Vec<&str> = parsed.as_array().unwrap().iter().map(|e| e["key"].as_str().unwrap()).collect();
        assert!(keys.contains(&"A380X_FOO"));
        assert!(keys.contains(&"A380X_FOO2"));

        run(&root, "delete", "A380X_FOO", "");
        assert_eq!(run(&root, "get", "A380X_FOO", ""), "");
        assert_eq!(run(&root, "get", "A380X_FOO2", ""), "baz", "unrelated key untouched");

        let _ = std::fs::remove_dir_all(&root);
    }
}
