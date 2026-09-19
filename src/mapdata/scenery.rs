//! Which DSF files X-Plane would use for a tile's terrain mesh.
//!
//! X-Plane takes a tile's base mesh from the first scenery pack in
//! `Custom Scenery/scenery_packs.ini` that has a DSF for it and is not an
//! overlay, and falls back to the global scenery. Overlays (airports,
//! objects) do not change the mesh, so they are skipped once read.

use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::sync::Mutex;

/// The folders a tile's DSF may be in, highest priority first.
#[derive(Debug)]
pub struct Scenery {
    packs: Vec<PathBuf>,
    /// Each pack's ten-degree folder listings, read once each: the terrain
    /// map asks about a thousand tiles at a time.
    listings: Mutex<HashMap<(usize, String), HashSet<String>>>,
}

/// `+47+011`: the tile's name and folder name use its south-west corner.
pub fn tile_name(lat: i32, lon: i32) -> String {
    format!("{lat:+03}{lon:+04}")
}

/// The ten-degree folder a tile sits in (`+40+010`).
pub fn folder_name(lat: i32, lon: i32) -> String {
    tile_name(lat.div_euclid(10) * 10, lon.div_euclid(10) * 10)
}

impl Scenery {
    /// The packs of an X-Plane installation, in the order X-Plane uses them.
    pub fn of_installation(root: &Path) -> Self {
        let mut packs = Vec::new();
        let custom = root.join("Custom Scenery");
        if let Ok(text) = std::fs::read_to_string(custom.join("scenery_packs.ini")) {
            for line in text.lines() {
                let line = line.trim();
                // SCENERY_PACK_DISABLED lines are left out; *GLOBAL_AIRPORTS*
                // has no mesh.
                let Some(path) = line.strip_prefix("SCENERY_PACK ") else { continue };
                let path = path.trim();
                if path.starts_with('*') {
                    continue;
                }
                let path = PathBuf::from(path.replace('\\', "/"));
                packs.push(if path.is_absolute() { path } else { root.join(path) });
            }
        }
        let global = root.join("Global Scenery");
        packs.push(global.join("X-Plane 12 Global Scenery"));
        packs.push(global.join("X-Plane 12 Demo Areas"));
        Self::from_packs(packs)
    }

    pub fn from_packs(packs: Vec<PathBuf>) -> Self {
        Self { packs, listings: Mutex::new(HashMap::new()) }
    }

    /// Every DSF for this tile, highest priority first. The caller skips the
    /// overlays among them.
    pub fn candidates(&self, lat: i32, lon: i32) -> Vec<PathBuf> {
        let (folder, tile) = (folder_name(lat, lon), format!("{}.dsf", tile_name(lat, lon)));
        let mut listings = self.listings.lock().unwrap_or_else(|e| e.into_inner());
        let mut found = Vec::new();
        for (i, pack) in self.packs.iter().enumerate() {
            let dir = pack.join("Earth nav data").join(&folder);
            let names = listings.entry((i, folder.clone())).or_insert_with(|| {
                // Names as X-Plane's scenery uses them; the file system does
                // not care about case on Windows.
                std::fs::read_dir(&dir)
                    .map(|entries| entries.flatten().map(|e| e.file_name().to_string_lossy().to_ascii_lowercase()).collect())
                    .unwrap_or_default()
            });
            if names.contains(&tile.to_ascii_lowercase()) {
                found.push(dir.join(&tile));
            }
        }
        found
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tiles_are_named_by_their_south_west_corner() {
        assert_eq!(tile_name(47, 11), "+47+011");
        assert_eq!(tile_name(-34, -58), "-34-058");
        assert_eq!(folder_name(47, 11), "+40+010");
        assert_eq!(folder_name(-34, -58), "-40-060");
        assert_eq!(folder_name(0, -1), "+00-010");
    }

    #[test]
    fn packs_follow_scenery_packs_ini_then_global_scenery() {
        let root = std::env::temp_dir().join(format!("fbw-mapdata-scenery-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        let mesh = |pack: &str| {
            let dir = root.join(pack).join("Earth nav data").join("+40+010");
            std::fs::create_dir_all(&dir).unwrap();
            std::fs::write(dir.join("+47+011.dsf"), b"x").unwrap();
        };
        mesh("Custom Scenery/zOrtho");
        mesh("Custom Scenery/Disabled");
        mesh("Global Scenery/X-Plane 12 Global Scenery");
        std::fs::write(
            root.join("Custom Scenery/scenery_packs.ini"),
            "I\n1000 Version\nSCENERY\n\nSCENERY_PACK *GLOBAL_AIRPORTS*\nSCENERY_PACK_DISABLED Custom Scenery/Disabled/\nSCENERY_PACK Custom Scenery/zOrtho/\n",
        )
        .unwrap();
        let found = Scenery::of_installation(&root).candidates(47, 11);
        assert_eq!(found.len(), 2);
        assert!(found[0].to_string_lossy().contains("zOrtho"));
        assert!(found[1].to_string_lossy().contains("Global Scenery"));
        assert!(Scenery::of_installation(&root).candidates(48, 11).is_empty());
    }
}
