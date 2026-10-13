use std::sync::Mutex;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    AircraftState,
    Airframe,
}

static REQUEST: Mutex<Option<Kind>> = Mutex::new(None);

pub fn request(kind: Kind) {
    if let Ok(mut r) = REQUEST.lock() {
        *r = Some(if *r == Some(Kind::Airframe) { Kind::Airframe } else { kind });
    }
}

pub fn take() -> Option<Kind> {
    REQUEST.lock().ok().and_then(|mut r| r.take())
}

pub fn backup(path: &std::path::Path) -> Option<std::path::PathBuf> {
    if !path.is_file() {
        return None;
    }
    let stamp = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
    let name = format!("{}.bak-{stamp}", path.file_name()?.to_string_lossy());
    let dst = path.with_file_name(name);
    std::fs::copy(path, &dst).ok().map(|_| dst)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_new_airframe_request_is_not_downgraded_by_a_later_state_reset() {
        let _ = take();
        request(Kind::Airframe);
        request(Kind::AircraftState);
        assert_eq!(take(), Some(Kind::Airframe));
        assert_eq!(take(), None);
        request(Kind::AircraftState);
        assert_eq!(take(), Some(Kind::AircraftState));
    }

    #[test]
    fn a_backup_copies_the_file_beside_itself() {
        let dir = std::env::temp_dir().join(format!("fbw_reset_backup_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("fbw_a380x_airframe.json");
        assert!(backup(&path).is_none());
        std::fs::write(&path, b"{}").unwrap();
        let copy = backup(&path).unwrap();
        assert_eq!(std::fs::read(&copy).unwrap(), b"{}");
        assert!(copy.file_name().unwrap().to_string_lossy().starts_with("fbw_a380x_airframe.json.bak-"));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
