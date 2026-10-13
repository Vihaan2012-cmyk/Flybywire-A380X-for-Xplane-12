//! SimBridge's world map: the stitched elevation map around the aircraft
//! and the two ways it is sampled, a local map for the ND and a profile for
//! the vertical display (simbridge `apps/server/src/terrain/mapdata/
//! worldmap.ts`, `tilemanager.ts`, `processing/maphandler.ts`,
//! `processing/gpu/elevationmap.ts`, `processing/gpu/elevationprofile.ts`).
//!
//! What differs, and why:
//! - Tiles come from X-Plane's DSF scenery (`tiles.rs`) instead of
//!   terrain.map. A tile "is in the map" when X-Plane has a DSF for it;
//!   one that turns out to have no mesh is water, as tiles absent from
//!   terrain.map are.
//! - SimBridge decompresses terrain.map tiles synchronously. A DSF takes
//!   about 0.2 to 0.4 s to read, so tiles load on background threads,
//!   nearest first, and a tile not loaded yet is `UnknownElevation`, which
//!   is what SimBridge's map holds for a tile it has not loaded.
//! - When tiles arrive and the lookup grid is unchanged, only their cells of
//!   the stitched map are rewritten; SimBridge rewrites all of it. The
//!   result is the same.
//! - The stitched map holds `i16` (every value it can hold fits) instead of
//!   a `Float32Array`, halving its memory.

use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::{Arc, Condvar, Mutex};

use super::geo::*;
use super::tiles::{tile_size, ElevationGrid, Loaded};
use super::types::{AircraftStatus, EfisData};

/// The world map is at most this many pixels on a side (`GpuMaxPixelSize`).
const GPU_MAX_PIXEL_SIZE: usize = 16384;
/// Radius around the aircraft whose tiles are kept, nm (`Worldmap.VisibilityRange`).
const VISIBILITY_RANGE: f64 = 800.;
/// terrain.map's tiles are one degree on a side (its file header).
const LATITUDE_STEP: f64 = 1.;
const LONGITUDE_STEP: f64 = 1.;
const GRID_ROWS: usize = 180;
const GRID_COLUMNS: usize = 360;

/// Where tiles come from.
pub trait TileProvider: Send + Sync + 'static {
    /// Whether the map has a tile here (terrain.map's `tileIndex !== -1`).
    fn exists(&self, lat: i32, lon: i32) -> bool;
    fn load(&self, lat: i32, lon: i32) -> Result<Loaded, String>;
}

impl TileProvider for super::tiles::TileSource {
    fn exists(&self, lat: i32, lon: i32) -> bool {
        super::tiles::TileSource::exists(self, lat, lon)
    }
    fn load(&self, lat: i32, lon: i32) -> Result<Loaded, String> {
        super::tiles::TileSource::load(self, lat, lon)
    }
}

type Cell = (usize, usize);

fn cell_south_west(cell: Cell) -> (i32, i32) {
    (cell.0 as i32 - 90, cell.1 as i32 - 180)
}

/// Background tile loading, nearest tile first.
struct Loader {
    queue: Mutex<LoaderQueue>,
    wake: Condvar,
}

#[derive(Default)]
struct LoaderQueue {
    wanted: VecDeque<Cell>,
    busy: HashSet<Cell>,
    done: Vec<(Cell, Result<Loaded, String>)>,
    stop: bool,
}

impl Loader {
    fn start(provider: Arc<dyn TileProvider>, threads: usize) -> Arc<Self> {
        let loader = Arc::new(Self { queue: Mutex::new(LoaderQueue::default()), wake: Condvar::new() });
        for n in 0..threads {
            let (loader, provider) = (loader.clone(), provider.clone());
            let _ = std::thread::Builder::new().name(format!("fbw-terrain-tiles-{n}")).spawn(move || loop {
                let cell = {
                    let mut q = loader.queue.lock().unwrap_or_else(|e| e.into_inner());
                    loop {
                        if q.stop {
                            return;
                        }
                        if let Some(cell) = q.wanted.pop_front() {
                            q.busy.insert(cell);
                            break cell;
                        }
                        q = loader.wake.wait(q).unwrap_or_else(|e| e.into_inner());
                    }
                };
                let (lat, lon) = cell_south_west(cell);
                let result = provider.load(lat, lon);
                let mut q = loader.queue.lock().unwrap_or_else(|e| e.into_inner());
                q.busy.remove(&cell);
                q.done.push((cell, result));
            });
        }
        loader
    }

    /// Replace the queue with these cells, in order, leaving out any being loaded.
    fn want(&self, cells: Vec<Cell>) {
        let mut q = self.queue.lock().unwrap_or_else(|e| e.into_inner());
        let busy = q.busy.clone();
        q.wanted = cells.into_iter().filter(|c| !busy.contains(c)).collect();
        drop(q);
        self.wake.notify_all();
    }

    fn take_done(&self) -> Vec<(Cell, Result<Loaded, String>)> {
        std::mem::take(&mut self.queue.lock().unwrap_or_else(|e| e.into_inner()).done)
    }

    fn stop(&self) {
        self.queue.lock().unwrap_or_else(|e| e.into_inner()).stop = true;
        self.wake.notify_all();
    }
}

/// `GridLookupData`.
#[derive(Clone, Debug, PartialEq)]
struct GridLookup {
    southwest: (f64, f64),
    northeast: (f64, f64),
    grid: Vec<Vec<Cell>>,
    min_width_per_tile: usize,
    min_height_per_tile: usize,
}

#[derive(Clone, Copy, Debug)]
struct WorldMapMetadata {
    southwest: (f64, f64),
    northeast: (f64, f64),
    grid_x: f64,
    grid_y: f64,
    min_width_per_tile: usize,
    min_height_per_tile: usize,
    width: usize,
    height: usize,
}

impl Default for WorldMapMetadata {
    fn default() -> Self {
        Self {
            southwest: (-100., -190.),
            northeast: (-100., -190.),
            grid_x: 0.,
            grid_y: 0.,
            min_width_per_tile: 0,
            min_height_per_tile: 0,
            width: 0,
            height: 0,
        }
    }
}

/// `ElevationProfile`.
#[derive(Clone, Debug, PartialEq)]
pub struct ElevationProfile {
    pub path_width: f64,
    pub waypoints_latitudes: Vec<f64>,
    pub waypoints_longitudes: Vec<f64>,
    pub range: f64,
    pub track_changes_significantly_at_distance: f64,
    pub fms_path_used: bool,
}

/// `MapHandler` with `Worldmap` and `TileManager`.
pub struct MapHandler {
    provider: Arc<dyn TileProvider>,
    loader: Arc<Loader>,
    /// `tileIndex !== -1`, per cell, asked once.
    exists: HashMap<Cell, bool>,
    /// `TileManager.grid[..][..].elevationmap`.
    grids: HashMap<Cell, Arc<ElevationGrid>>,
    ground_truth: Option<(f64, f64)>,
    aircraft_status: Option<AircraftStatus>,
    cached: Vec<i16>,
    cached_tiles: usize,
    /// The lookup the stitched map was last built for.
    built_for: Option<GridLookup>,
    meta: WorldMapMetadata,
    initialized: bool,
    pub errors: Vec<String>,
}

impl Drop for MapHandler {
    fn drop(&mut self) {
        self.loader.stop();
    }
}

impl MapHandler {
    pub fn new(provider: Arc<dyn TileProvider>, loader_threads: usize) -> Self {
        let loader = Loader::start(provider.clone(), loader_threads.max(1));
        Self {
            provider,
            loader,
            exists: HashMap::new(),
            grids: HashMap::new(),
            ground_truth: None,
            aircraft_status: None,
            cached: Vec::new(),
            cached_tiles: 0,
            built_for: None,
            meta: WorldMapMetadata::default(),
            initialized: false,
            errors: Vec::new(),
        }
    }

    /// `initialize`: SimBridge runs one update at Innsbruck to compile its
    /// kernels; there is nothing to compile here.
    pub fn initialize(&mut self) {
        self.initialized = true;
    }

    /// `cleanupMemory`.
    pub fn reset(&mut self) {
        self.grids.clear();
        self.cached = Vec::new();
        self.cached_tiles = 0;
        self.built_for = None;
        self.meta = WorldMapMetadata::default();
        self.ground_truth = None;
        self.aircraft_status = None;
        self.loader.want(Vec::new());
        let _ = self.loader.take_done();
    }

    pub fn aircraft_status_update(&mut self, status: &AircraftStatus) {
        self.aircraft_status = Some(status.clone());
    }

    pub fn has_aircraft_status(&self) -> bool {
        self.aircraft_status.is_some()
    }

    /// `positionUpdate`.
    pub fn position_update(&mut self, latitude: f64, longitude: f64) {
        if self.initialized {
            self.update_ground_truth_position_and_cached_tiles((latitude, longitude));
        }
    }

    /// Whether tiles are still being loaded.
    pub fn loading(&self) -> bool {
        let q = self.loader.queue.lock().unwrap_or_else(|e| e.into_inner());
        !q.wanted.is_empty() || !q.busy.is_empty() || !q.done.is_empty()
    }

    /// Take tiles that finished loading. SimBridge loads tiles while it
    /// handles a position update, so they are in the map at once; loaded in
    /// the background, they would otherwise wait for the gauge's next status,
    /// which a parked aircraft does not send.
    pub fn poll(&mut self) {
        let arrived = !self.loader.queue.lock().unwrap_or_else(|e| e.into_inner()).done.is_empty();
        if let (true, true, Some(position)) = (self.initialized, arrived, self.ground_truth) {
            self.update_ground_truth_position_and_cached_tiles(position);
        }
    }

    fn tile_exists(&mut self, cell: Cell) -> bool {
        let provider = &self.provider;
        *self.exists.entry(cell).or_insert_with(|| {
            let (lat, lon) = cell_south_west(cell);
            provider.exists(lat, lon)
        })
    }

    /// `worldMapIndices`.
    fn world_map_indices(latitude: f64, longitude: f64) -> Option<Cell> {
        let row = ((latitude + 90.) / LATITUDE_STEP).floor();
        let column = ((longitude + 180.) / LONGITUDE_STEP).floor();
        if row < 0. || row >= GRID_ROWS as f64 || column < 0. || column >= GRID_COLUMNS as f64 {
            return None;
        }
        Some((row as usize, column as usize))
    }

    /// `createGridLookupTable`.
    fn create_grid_lookup_table(&mut self, position: (f64, f64), max_width: usize, max_height: usize, default_tile_size: usize) -> Option<GridLookup> {
        let r = VISIBILITY_RANGE * 1852.;
        let south = project_wgs84(position.0, position.1, 180., r).0;
        let southwest = project_wgs84(position.0, position.1, 225., r);
        let west = project_wgs84(position.0, position.1, 270., r).1;
        let north = project_wgs84(position.0, position.1, 0., r).0;
        let east = project_wgs84(position.0, position.1, 90., r).1;
        let northeast = project_wgs84(position.0, position.1, 45., r);

        let mut southwest_lat = south.min(southwest.0);
        let mut northeast_lat = north.max(northeast.0);
        let mut southwest_long = west.min(southwest.1);
        let mut northeast_long = east.min(northeast.1);
        // handle the 180 degree wrap around for the western coordinate
        if west * southwest.1 < 0. {
            southwest_long = west.max(southwest.1);
        }
        // handle the 180 degree wrap around for the eastern coordinate
        if east * northeast.1 < 0. {
            northeast_long = east.max(northeast.1);
        }

        // SimBridge reads `.row` of an undefined index here and stops; a
        // lookup outside the grid (only at a pole) is skipped instead.
        let southwest_grid = Self::world_map_indices(southwest_lat, southwest_long)?;
        let northeast_grid = Self::world_map_indices(northeast_lat, northeast_long)?;

        let (sw_row, ne_row) = (southwest_grid.0 as i64, northeast_grid.0 as i64);
        let mut row_count = ne_row - sw_row;
        let mut row_direction = 1;
        if southwest_lat >= position.0 {
            // we are at the south pole
            row_count = sw_row + ne_row;
            row_direction = -1;
        } else if northeast_lat <= position.0 {
            // we are at the north pole
            row_count = GRID_ROWS as i64 - sw_row + GRID_ROWS as i64 - ne_row;
        }
        row_count += 1;

        let mut column_count = northeast_grid.1 as i64 - southwest_grid.1 as i64;
        if northeast_long < southwest_long {
            // wrap around at 180°
            column_count = GRID_COLUMNS as i64 - southwest_grid.1 as i64 + northeast_grid.1 as i64;
        }
        column_count += 1;
        if row_count <= 0 || column_count <= 0 {
            return None;
        }

        // create the look up table and sort from north->south and west->east
        let mut grid = vec![Vec::new(); row_count as usize];
        for y in 0..row_count {
            let mut row = sw_row + row_direction * y;
            if row < 0 {
                row = row.abs();
            }
            if row >= GRID_ROWS as i64 {
                row -= GRID_ROWS as i64;
            }
            grid[(row_count - 1 - y) as usize] = (0..column_count)
                .map(|x| (row as usize, ((southwest_grid.1 as i64 + x) % GRID_COLUMNS as i64) as usize))
                .collect();
        }

        // find the minimum dimensions per tile
        let (mut min_width_per_tile, mut min_height_per_tile) = (5000, 5000);
        for cell in grid.iter().flatten().copied().collect::<Vec<_>>() {
            if self.tile_exists(cell) {
                let size = tile_size(cell_south_west(cell).0);
                min_width_per_tile = min_width_per_tile.min(size);
                min_height_per_tile = min_height_per_tile.min(size);
            }
        }
        if min_width_per_tile == 5000 {
            min_width_per_tile = default_tile_size;
        }
        if min_height_per_tile == 5000 {
            min_height_per_tile = default_tile_size;
        }

        let map_height = min_height_per_tile * grid.len();
        let map_width = min_width_per_tile * grid[0].len();

        // delete rows if necessary (shared clipping between top and bottom)
        if map_height > max_height {
            let clipping_tile_count = (map_height - max_height).div_ceil(min_height_per_tile);
            let top = clipping_tile_count.div_ceil(2);
            let bottom = clipping_tile_count / 2;
            grid.drain(0..top.min(grid.len()));
            let keep = grid.len().saturating_sub(bottom);
            grid.truncate(keep);
            northeast_lat -= LATITUDE_STEP * top as f64;
            southwest_lat += LATITUDE_STEP * bottom as f64;
        }

        // delete columns as necessary (shared clipping between left and right)
        if map_width > max_width {
            let clipping_tile_count = (map_width - max_width).div_ceil(min_width_per_tile);
            let start = clipping_tile_count.div_ceil(2);
            let end = clipping_tile_count / 2;
            for row in &mut grid {
                row.drain(0..start.min(row.len()));
                let keep = row.len().saturating_sub(end);
                row.truncate(keep);
            }
            southwest_long += LONGITUDE_STEP * start as f64;
            northeast_long -= LONGITUDE_STEP * end as f64;
            // ensure correct updates at -180.0, 180.0 degree wrap around
            if southwest_long >= 180. {
                southwest_long -= 360.;
            }
            if northeast_long < -180. {
                northeast_long += 360.;
            }
        }
        if grid.is_empty() || grid[0].is_empty() {
            return None;
        }

        Some(GridLookup {
            southwest: (southwest_lat, southwest_long),
            northeast: (northeast_lat, northeast_long),
            grid,
            min_width_per_tile,
            min_height_per_tile,
        })
    }

    /// `Worldmap.updatePosition`: asks for the missing tiles of the grid,
    /// nearest first, and takes the ones that have arrived. Returns the
    /// cells whose content changed.
    fn update_position(&mut self, lookup: &GridLookup, position: (f64, f64)) -> Vec<Cell> {
        let relevant: HashSet<Cell> = lookup.grid.iter().flatten().copied().collect();
        let mut changed = Vec::new();
        for (cell, result) in self.loader.take_done() {
            match result {
                Ok(Loaded::Grid(grid)) => {
                    if relevant.contains(&cell) {
                        self.grids.insert(cell, Arc::new(grid));
                        changed.push(cell);
                    }
                }
                Ok(Loaded::Water) => {
                    self.exists.insert(cell, false);
                    changed.push(cell);
                }
                Err(e) => {
                    // Not in the map as far as the ND can tell: water.
                    let (lat, lon) = cell_south_west(cell);
                    self.errors.push(format!("tile {lat:+03}{lon:+04}: {e}"));
                    self.exists.insert(cell, false);
                    changed.push(cell);
                }
            }
        }
        let mut wanted: Vec<Cell> = relevant.iter().copied().filter(|c| self.exists.get(c) == Some(&true) && !self.grids.contains_key(c)).collect();
        let distance = |c: &Cell| {
            let (lat, lon) = cell_south_west(*c);
            distance_wgs84(position.0, position.1, lat as f64 + 0.5, lon as f64 + 0.5)
        };
        wanted.sort_by(|a, b| distance(a).total_cmp(&distance(b)));
        self.loader.want(wanted);
        changed
    }

    /// Copy a cell's tile into the stitched map, as the rebuild loop in
    /// `updateGroundTruthPositionAndCachedTiles` does.
    fn write_cell(&mut self, lookup: &GridLookup, row_index: usize, column_index: usize) {
        let meta = self.meta;
        let cell = lookup.grid[row_index][column_index];
        let width = meta.min_width_per_tile * lookup.grid[0].len();
        let map = if self.exists.get(&cell) == Some(&true) { Some(self.grids.get(&cell).cloned()) } else { None };
        for y in 0..meta.min_height_per_tile {
            let start = (row_index * meta.min_height_per_tile + y) * width + column_index * meta.min_width_per_tile;
            let target = &mut self.cached[start..start + meta.min_width_per_tile];
            match &map {
                None => target.fill(WATER_ELEVATION),
                Some(None) => target.fill(UNKNOWN_ELEVATION),
                Some(Some(map)) => {
                    // share subsampling error between all sides of the tile
                    let offset_y = if map.rows > meta.min_height_per_tile { (map.rows - meta.min_height_per_tile).div_ceil(2) } else { 0 };
                    let offset_x = if map.columns > meta.min_width_per_tile { (map.columns - meta.min_width_per_tile).div_ceil(2) } else { 0 };
                    let from = (y + offset_y) * map.columns + offset_x;
                    match map.map.get(from..from + meta.min_width_per_tile) {
                        Some(source) => target.copy_from_slice(source),
                        None => target.fill(UNKNOWN_ELEVATION),
                    }
                }
            }
        }
    }

    /// `updateGroundTruthPositionAndCachedTiles`.
    fn update_ground_truth_position_and_cached_tiles(&mut self, position: (f64, f64)) {
        self.ground_truth = Some(position);
        let Some(lookup) = self.create_grid_lookup_table(position, GPU_MAX_PIXEL_SIZE, GPU_MAX_PIXEL_SIZE, DEFAULT_TILE_SIZE) else {
            return;
        };
        let changed = self.update_position(&lookup, position);
        let relevant_tile_count = lookup.grid.len() * lookup.grid[0].len();

        if !changed.is_empty() || self.cached_tiles != relevant_tile_count {
            let southwest_grid = Self::world_map_indices(lookup.southwest.0, lookup.southwest.1);
            let northeast_grid = Self::world_map_indices(lookup.northeast.0, lookup.northeast.1);
            let world_width = lookup.min_width_per_tile * lookup.grid[0].len();
            let world_height = lookup.min_height_per_tile * lookup.grid.len();
            self.meta.min_width_per_tile = lookup.min_width_per_tile;
            self.meta.min_height_per_tile = lookup.min_height_per_tile;

            let same_layout = self.built_for.as_ref() == Some(&lookup) && self.cached.len() == world_width * world_height;
            if same_layout {
                let changed: HashSet<Cell> = changed.into_iter().collect();
                for r in 0..lookup.grid.len() {
                    for c in 0..lookup.grid[0].len() {
                        if changed.contains(&lookup.grid[r][c]) {
                            self.write_cell(&lookup, r, c);
                        }
                    }
                }
            } else {
                self.cached = vec![0; world_width * world_height];
                for r in 0..lookup.grid.len() {
                    for c in 0..lookup.grid[0].len() {
                        self.write_cell(&lookup, r, c);
                    }
                }
            }

            // update the world map metadata for the rendering
            if let (Some(sw), Some(ne)) = (southwest_grid, northeast_grid) {
                let (sw_lat, sw_lon) = cell_south_west(sw);
                let (ne_lat, ne_lon) = cell_south_west(ne);
                self.meta.southwest = (sw_lat as f64, sw_lon as f64);
                self.meta.northeast = (ne_lat as f64 + LATITUDE_STEP, ne_lon as f64 + LONGITUDE_STEP);
            }
            self.meta.width = world_width;
            self.meta.height = world_height;

            // cleanupElevationCache
            let relevant: HashSet<Cell> = lookup.grid.iter().flatten().copied().collect();
            self.grids.retain(|c, _| relevant.contains(c));
            self.cached_tiles = relevant_tile_count;
            self.built_for = Some(lookup.clone());
        }

        // calculate the correct pixel coordinate in every step
        match Self::world_map_indices(position.0, position.1) {
            Some(ego) => {
                let lat_step = LATITUDE_STEP / self.meta.min_height_per_tile as f64;
                let long_step = LONGITUDE_STEP / self.meta.min_width_per_tile as f64;
                let (sw_lat, sw_lon) = cell_south_west(ego);
                let lat_delta = position.0 - sw_lat as f64;
                let long_delta = position.1 - sw_lon as f64;
                let (mut x_offset, mut y_offset) = (0, 0);
                for (row_index, row) in lookup.grid.iter().enumerate() {
                    if row[0].0 == ego.0 {
                        for (column_index, cell) in row.iter().enumerate() {
                            if cell.1 == ego.1 {
                                y_offset = row_index * self.meta.min_height_per_tile;
                                x_offset = column_index * self.meta.min_width_per_tile;
                            }
                        }
                    }
                }
                self.meta.grid_x = x_offset as f64 + long_delta / long_step;
                self.meta.grid_y = y_offset as f64 + self.meta.min_height_per_tile as f64 - lat_delta / lat_step;
            }
            None => {
                self.meta.grid_x = self.meta.width as f64 / 2.;
                self.meta.grid_y = self.meta.height as f64 / 2.;
            }
        }
    }

    fn frame(&self) -> Option<WorldMapFrame> {
        let (lat, lon) = self.ground_truth?;
        Some(WorldMapFrame {
            ground_truth_latitude: lat,
            ground_truth_longitude: lon,
            southwest_lat: self.meta.southwest.0,
            southwest_long: self.meta.southwest.1,
            northeast_lat: self.meta.northeast.0,
            northeast_long: self.meta.northeast.1,
            width: self.meta.width as f64,
            height: self.meta.height as f64,
            grid_x: self.meta.grid_x,
            grid_y: self.meta.grid_y,
        })
    }

    fn world(&self, x: f64, y: f64) -> i16 {
        // The kernels only index inside the map.
        self.cached[y as usize * self.meta.width + x as usize]
    }

    /// `extractElevation`.
    pub fn extract_elevation(&self, latitude: f64, longitude: f64) -> i16 {
        let (Some(status), Some(truth)) = (&self.aircraft_status, self.ground_truth) else { return INVALID_ELEVATION };
        if self.cached.is_empty() {
            return INVALID_ELEVATION;
        }
        let m = &self.meta;
        let step = degrees_per_pixel(m.southwest.0, m.southwest.1, m.northeast.0, m.northeast.1, status.latitude, m.width as f64, m.height as f64);
        let lat_pixel_delta = (truth.0 - latitude) / step.0;
        let long_pixel_delta = (longitude - truth.1) / step.1;
        let index = ((m.grid_y + lat_pixel_delta) * m.width as f64 + m.grid_x + long_pixel_delta).floor();
        if index >= self.cached.len() as f64 || index < 0. || index.is_nan() {
            return UNKNOWN_ELEVATION;
        }
        self.cached[index as usize]
    }

    /// `createLocalElevationMap` with the `createLocalElevationMap` kernel:
    /// `map_width` x `map_height` elevations, row 0 at the top.
    pub fn create_local_elevation_map(&self, config: &EfisData) -> Option<Vec<i16>> {
        let status = self.aircraft_status.as_ref()?;
        let frame = self.frame()?;
        if self.cached.is_empty() {
            return None;
        }
        let (nd_width, nd_height) = (config.map_width, config.map_height);
        let mut metres_per_pixel = js_round((config.nd_range * NAUTICAL_MILES_TO_METRES) / (nd_height as f64 - config.center_offset_y));
        if config.arc_mode {
            metres_per_pixel *= 2.;
        }
        let center_x = nd_width as f64 / 2.;
        let mut out = vec![0i16; nd_width * nd_height];
        for y in 0..nd_height {
            for x in 0..nd_width {
                let delta = (x as f64 - center_x, nd_height as f64 - y as f64 - config.center_offset_y);
                let distance_pixels = (delta.0 * delta.0 + delta.1 * delta.1).sqrt();
                // Cut off ARC shape when in A32NX and arc mode
                if config.center_offset_y == 0. && config.arc_mode && distance_pixels > nd_height as f64 {
                    out[y * nd_width + x] = INVALID_ELEVATION;
                    continue;
                }
                let distance = distance_pixels * (metres_per_pixel / 2.);
                // At the centre acos(0/0) is NaN; any bearing gives the aircraft's position.
                let angle = rad2deg((delta.1 / distance_pixels).acos());
                let angle = if angle.is_nan() { 0. } else { angle };
                let bearing = if x as f64 > center_x { angle } else { 360. - angle };
                let bearing = normalize_heading(bearing + status.heading);
                let projected = project_wgs84(status.latitude, status.longitude, bearing, distance);
                let pixel = wgs84_to_pixel_coordinate(status.latitude, projected.0, projected.1, &frame);
                out[y * nd_width + x] = if !(pixel.1 >= 0. && pixel.1 < frame.height && pixel.0 >= 0. && pixel.0 < frame.width) {
                    UNKNOWN_ELEVATION
                } else {
                    self.world(pixel.0, pixel.1)
                };
            }
        }
        Some(out)
    }

    /// `createElevationProfile` with the `createElevationProfile` kernel.
    pub fn create_elevation_profile(&self, profile: &ElevationProfile, profile_width: usize) -> Option<Vec<i32>> {
        let status = self.aircraft_status.as_ref()?;
        let frame = self.frame()?;
        if self.cached.is_empty() {
            return None;
        }
        let (lats, lons) = (&profile.waypoints_latitudes, &profile.waypoints_longitudes);
        if lats.is_empty() || lats.len() != lons.len() {
            return None;
        }
        let distance_per_pixel = profile.range / profile_width as f64;
        let invalid = INVALID_ELEVATION as i32;
        let unknown = UNKNOWN_ELEVATION as i32;
        let mut out = Vec::with_capacity(profile_width);
        for thread_x in 0..profile_width {
            let distance_for_pixel = distance_per_pixel * thread_x as f64;
            let mut route_segment_index = lats.len();
            let mut route_start_point_distance = 0.;
            let (mut start_latitude, mut start_longitude) = (status.latitude, status.longitude);
            // find the correct starting point
            for i in 0..lats.len() {
                let current = distance_wgs84(start_latitude, start_longitude, lats[i], lons[i]);
                if route_start_point_distance + current >= distance_for_pixel {
                    route_segment_index = i;
                    break;
                }
                route_start_point_distance += current;
                start_latitude = lats[i];
                start_longitude = lons[i];
            }
            // check if we exceeded the points
            if route_segment_index >= lats.len() {
                out.push(invalid);
                continue;
            }
            let remaining_distance = (distance_for_pixel - route_start_point_distance) * 1852.;
            let bearing = bearing_wgs84(start_latitude, start_longitude, lats[route_segment_index], lons[route_segment_index]);
            let center = project_wgs84(start_latitude, start_longitude, bearing, remaining_distance);

            let mut bearing_start = bearing - 90.;
            if bearing_start < 0. {
                bearing_start += 360.;
            }
            let mut bearing_end = bearing + 90.;
            if bearing_end >= 360. {
                bearing_end -= 360.;
            }
            // pathOffset is passed as full width of corridor
            let offset_metres = (profile.path_width * 1852.) / 2.;
            let start_projected = project_wgs84(center.0, center.1, bearing_start, offset_metres);
            let start_pixel = wgs84_to_pixel_coordinate(status.latitude, start_projected.0, start_projected.1, &frame);
            let end_projected = project_wgs84(center.0, center.1, bearing_end, offset_metres);
            let end_pixel = wgs84_to_pixel_coordinate(status.latitude, end_projected.0, end_projected.1, &frame);

            // Use a modified Bresenham line algorithm to sample the line along the world map pixels
            let delta_x = (end_pixel.0 - start_pixel.0).abs();
            let step_x = if start_pixel.0 < end_pixel.0 { 1. } else { -1. };
            let delta_y = -(end_pixel.1 - start_pixel.1).abs();
            let step_y = if start_pixel.1 < end_pixel.1 { 1. } else { -1. };
            let mut error = delta_x + delta_y;
            let mut max_elevation = -1000;
            let (mut x, mut y) = start_pixel;
            if !(x.is_finite() && y.is_finite() && end_pixel.0.is_finite() && end_pixel.1.is_finite()) {
                out.push(max_elevation);
                continue;
            }
            loop {
                if y >= 0. && y < frame.height && x >= 0. && x < frame.width {
                    let elevation = self.world(x, y) as i32;
                    if elevation != invalid && elevation != unknown && elevation > max_elevation {
                        max_elevation = elevation;
                    }
                }
                if x == end_pixel.0 && y == end_pixel.1 {
                    break;
                }
                let error_double = 2. * error;
                if error_double >= delta_y {
                    if x == end_pixel.0 {
                        break;
                    }
                    error += delta_y;
                    x += step_x;
                }
                if error_double <= delta_x {
                    if y == end_pixel.1 {
                        break;
                    }
                    error += delta_x;
                    y += step_y;
                }
            }
            out.push(max_elevation);
        }
        Some(out)
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// Land everywhere at a height that rises one foot per pixel eastward
    /// from 1000 ft, except tiles south of 40 degrees, which are absent.
    pub struct Slope;

    impl TileProvider for Slope {
        fn exists(&self, lat: i32, _lon: i32) -> bool {
            lat >= 40
        }
        fn load(&self, lat: i32, _lon: i32) -> Result<Loaded, String> {
            let n = tile_size(lat);
            let metres: Vec<i16> = (0..n * n).map(|i| ((1000 + (i % n) as i32) as f64 / 3.28084).round() as i16).collect();
            Ok(Loaded::Grid(ElevationGrid::from_metres(n, n, &metres)))
        }
    }

    pub fn loaded_map(lat: f64, lon: f64) -> MapHandler {
        let mut map = MapHandler::new(Arc::new(Slope), 4);
        map.initialize();
        let t = std::time::Instant::now();
        loop {
            map.position_update(lat, lon);
            if !map.loading() && map.cached.iter().all(|&v| v != UNKNOWN_ELEVATION) {
                break;
            }
            assert!(t.elapsed().as_secs() < 60, "tiles did not load");
            std::thread::sleep(std::time::Duration::from_millis(20));
        }
        map
    }

    #[test]
    fn the_world_map_stitches_tiles_around_the_aircraft() {
        let map = loaded_map(47.26, 11.35);
        let m = map.meta;
        // 800 nm around 47 N: 27 rows of tiles, and the grid is 16384 px at most.
        assert_eq!(m.min_width_per_tile, 279);
        assert!(m.width <= GPU_MAX_PIXEL_SIZE && m.height <= GPU_MAX_PIXEL_SIZE);
        assert_eq!(m.width % 279, 0);
        // The aircraft sits in its tile's pixel.
        let (tile_x, tile_y) = (m.grid_x as usize % 279, m.grid_y as usize % 279);
        assert_eq!(tile_x, (0.35 * 279.) as usize);
        assert_eq!(tile_y, 279 - (0.26 * 279.) as usize - 1);
        // Tiles south of 40 N are absent: water.
        assert_eq!(map.cached[map.cached.len() - 1], WATER_ELEVATION);
        assert!(map.cached[0] >= 1000);
    }
}
