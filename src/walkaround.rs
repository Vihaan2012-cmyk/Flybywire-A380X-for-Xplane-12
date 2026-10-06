//! The exterior preflight walkaround: covers, plugs, gear pins and chocks a
//! ground crew fits between flights and removes before departure, modelled
//! the CL650 way -- a set of installed/removed items with real physical
//! consequences if the crew forgets one, not a cosmetic checklist.
//!
//! Interface (shared with the 3D-object side; do not change): 25 items,
//! grouped probes/engines/gear pins/chocks (`ITEMS`, fixed order). Each has
//! a plugin-owned float dataref `fbw/walkaround/<id>`, 1 installed / 0
//! removed, writable (so a 3D click-spot can drive it directly), plus a
//! read-only `fbw/walkaround/any_installed`. Each also has a
//! `fbw/walkaround/<id>_toggle` command; `fbw/walkaround/remove_all` and
//! `fbw/walkaround/install_all` act on every item at once.
//!
//! Start state (item 1 of the brief): cold and dark (on the ground, no
//! engine running, `start_state::read_situation`) restores last session's
//! saved state if the airframe save file has one, else every item starts
//! installed (a ground crew fits them between flights). Starting with any
//! engine running or airborne starts every item removed -- nobody flies
//! with a pitot cover on. Persisted in `persistence.rs`'s
//! `AirframeState::walkaround_installed`.
//!
//! Consequences (item 2), each through an existing model or failure, never
//! a scripted message:
//! - `pitot_cover_N`/`aoa_cover_N` block ADIRU N's own pitot/AoA source;
//!   `static_covers` blocks every ADIRU's static source at once (the
//!   interface has one item for the whole aircraft's static ports, not one
//!   per ADIRU). Routed through `failures.rs`'s existing extra-catalogue
//!   ATA34 ids 34_100-34_108 ("ADIRU n pitot/static/AoA fault"), which
//!   `physics/adirs.rs`'s `update_adr` now also reads directly (in addition
//!   to their pre-existing hook) to force that source blocked/frozen -- the
//!   most physical route available, since it is the same real pitot-static
//!   model an icing-blocked probe already uses (see `physics/adirs.rs`'s
//!   own doc comments at the read sites). Carried in a brand new failure
//!   channel, `failures::set_walkaround_levels`, modelled exactly on
//!   `failures::set_breaker_levels`: effective while the cover is on, never
//!   saved as crew-armed, cleared the instant the cover comes off.
//! - `eng_inlet_cover_N` installed while engine N is rotating past the same
//!   "the starter has it lit" threshold `fadec.rs`'s own state machine and
//!   running-detection heuristic already use (`ENGINE_ROTATING_N2_PERCENT`,
//!   20% N2 -- see its doc comment for the two citations) is ingested: the
//!   item is destroyed (removed) and the cover goes through the fan and
//!   down the core. `physics/damage.rs`'s `Damage::arm_fod` arms 72_024+n,
//!   which `engine_commands.rs`'s `EXOTIC` table gives the same HP
//!   compressor destruction as 72_012, so the engine model itself loses
//!   its flame and runs down. Persisted on the engine's wear record until
//!   the Study panel's repair.
//! - `eng_exhaust_cover_N` installed under the same rotating engine is
//!   blown out: the item is removed, log line only (there is no sourced
//!   physical model for what a blown exhaust plug would do to the jet pipe
//!   worth arming a failure over; the brief calls for a log line here).
//! - `gear_pin_*` installed jams that side's retraction actuator in
//!   FlyByWire's own gear model: GearActuatorJammed 32_020 (nose), 32_021
//!   (left), 32_022 (right), which the A380's `HydraulicGearSystem` reads
//!   (fbw-common hydraulic/landing_gear.rs, `jammed_actuator_failure`), so
//!   the gear, the LGCIUs and the ECAM all see a leg that will not come up.
//! - `chocks_*` installed while the aircraft rolls past a walking pace
//!   (`CHOCK_OVERRUN_SPEED_MS`, generic/derived -- see its doc comment) is
//!   overrun: the item is removed, log line only.
//!
//! Study panel (item 3): `study::walkaround::draw`, listed as its own
//! "Walkaround" page (`study/mod.rs`'s `PageKind::Walkaround`).

use std::collections::BTreeMap;

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::published::{self, Command, Published, Value};
use crate::xp::{DataRef, Xplm};
use crate::Vars;

/// How many items the interface defines.
pub const N: usize = 25;

/// Every item's id, in the interface's fixed order -- also the dataref/
/// command name suffix and the persisted map's key. Do not reorder or
/// rename: the 3D-object side and any existing save file both depend on
/// these exact strings.
pub const ITEMS: [&str; N] = [
    "pitot_cover_1",
    "pitot_cover_2",
    "pitot_cover_3",
    "static_covers",
    "aoa_cover_1",
    "aoa_cover_2",
    "aoa_cover_3",
    "eng_inlet_cover_1",
    "eng_inlet_cover_2",
    "eng_inlet_cover_3",
    "eng_inlet_cover_4",
    "eng_exhaust_cover_1",
    "eng_exhaust_cover_2",
    "eng_exhaust_cover_3",
    "eng_exhaust_cover_4",
    "gear_pin_nose",
    "gear_pin_lwing",
    "gear_pin_rwing",
    "gear_pin_lbody",
    "gear_pin_rbody",
    "chocks_nose",
    "chocks_lwing",
    "chocks_rwing",
    "chocks_lbody",
    "chocks_rbody",
];

const PITOT: [usize; 3] = [0, 1, 2];
const STATIC_COVERS: usize = 3;
const AOA: [usize; 3] = [4, 5, 6];
const INLET: [usize; 4] = [7, 8, 9, 10];
const EXHAUST: [usize; 4] = [11, 12, 13, 14];
const GEAR_PIN_NOSE: usize = 15;
const GEAR_PIN_LWING: usize = 16;
const GEAR_PIN_RWING: usize = 17;
const GEAR_PIN_LBODY: usize = 18;
const GEAR_PIN_RBODY: usize = 19;
const CHOCKS: [usize; 5] = [20, 21, 22, 23, 24];

/// The Study page's four groups, by index into `ITEMS`.
pub const GROUP_PROBES: [usize; 7] = [0, 1, 2, 3, 4, 5, 6];
pub const GROUP_ENGINES: [usize; 8] = [7, 8, 9, 10, 11, 12, 13, 14];
pub const GROUP_GEAR_PINS: [usize; 5] = [15, 16, 17, 18, 19];
pub const GROUP_CHOCKS: [usize; 5] = [20, 21, 22, 23, 24];

/// The failure ids a covered probe activates (`failures::extra::adirs`,
/// ATA34): pitot 34_100+n, static 34_103+n, AoA 34_106+n, `n` the ADIRU
/// index 0..2.
fn pitot_fail_id(n: usize) -> u64 {
    34_100 + n as u64
}
fn static_fail_ids() -> [u64; 3] {
    [34_103, 34_104, 34_105]
}
fn aoa_fail_id(n: usize) -> u64 {
    34_106 + n as u64
}

/// The engine state machine's own "the starter has it rotating enough to be
/// considered lit" threshold (`fadec.rs`'s `next_state` Off->On gate,
/// `sim_n3 > 20.`, and the running-detection heuristic a few hundred lines
/// later, `xp.n2[i] > 20.`) -- reused here, not a new figure, as the point
/// past which a rotating engine would draw a loose inlet cover in and blow
/// an exhaust plug out.
const ENGINE_ROTATING_N2_PERCENT: f64 = 20.0;

/// A wheel chock is a static wedge with no positive lock to the tyre;
/// generic, not a cited AMM figure (this crate's convention for an uncited
/// threshold, matching e.g. `physics/damage.rs`'s `FUSE_PLUG_MELT_C`) -- any
/// sustained rolling groundspeed above a walking pace means the tyre has
/// already climbed over it. Kept small so a chocked, parked aircraft
/// (groundspeed reads ~0) never trips this from sensor noise, matching the
/// forgiving-failure rule: nothing here triggers on normal operation.
const CHOCK_OVERRUN_SPEED_MS: f64 = 0.5; // ~1 kt

/// The start-state rule (brief item 1), as a pure function: cold and dark
/// restores the saved map if it has anything in it (a missing key inside a
/// non-empty map still defaults to installed -- a save from before an item
/// existed), otherwise every item starts installed; any other start starts
/// every item removed.
pub fn initial_state(cold_and_dark: bool, saved: &BTreeMap<String, bool>) -> [bool; N] {
    if !cold_and_dark {
        return [false; N];
    }
    if saved.is_empty() {
        [true; N]
    } else {
        std::array::from_fn(|i| *saved.get(ITEMS[i]).unwrap_or(&true))
    }
}

/// Engine-inlet ingestion and exhaust-plug blow-out, as a pure function over
/// `installed`: for each engine rotating past [`ENGINE_ROTATING_N2_PERCENT`]
/// with its inlet cover or exhaust plug still on, removes that item and
/// returns a log line; an ingested inlet additionally returns that engine's
/// 0-based index for the caller to arm FOD damage through (`Damage::arm_fod`
/// lives on `physics::damage::Damage`, which this module does not depend
/// on, to keep this function pure and testable without X-Plane).
fn ingest_and_blow_out(installed: &mut [bool; N], n2_percent: [f64; 4]) -> (Vec<usize>, Vec<String>) {
    let mut fod = Vec::new();
    let mut lines = Vec::new();
    for e in 0..4usize {
        if n2_percent[e] <= ENGINE_ROTATING_N2_PERCENT {
            continue;
        }
        let inlet = INLET[e];
        if installed[inlet] {
            installed[inlet] = false;
            lines.push(format!(
                "walkaround: engine {} inlet cover ingested at N2 {:.0}% -- FOD damage",
                e + 1,
                n2_percent[e]
            ));
            fod.push(e);
        }
        let exhaust = EXHAUST[e];
        if installed[exhaust] {
            installed[exhaust] = false;
            lines.push(format!("walkaround: engine {} exhaust plug blown out at N2 {:.0}%", e + 1, n2_percent[e]));
        }
    }
    (fod, lines)
}

/// Chock overrun, as a pure function: any installed chock is removed the
/// moment groundspeed exceeds [`CHOCK_OVERRUN_SPEED_MS`], each producing a
/// log line.
fn chock_overrun(installed: &mut [bool; N], groundspeed_ms: f64) -> Vec<String> {
    let mut lines = Vec::new();
    if groundspeed_ms.abs() <= CHOCK_OVERRUN_SPEED_MS {
        return lines;
    }
    for &idx in &CHOCKS {
        if installed[idx] {
            installed[idx] = false;
            lines.push(format!(
                "walkaround: {} overrun at {:.1} kt groundspeed",
                ITEMS[idx],
                groundspeed_ms * 1.943_844
            ));
        }
    }
    lines
}

/// Which of the nose/left/right gear groups a pin currently holds down:
/// left/right each cover both the wing and body gear on that side, since
/// FlyByWire's own model retracts them as one group per side (one
/// `GearActuatorId` each: GearNose, GearLeft, GearRight).
fn gear_block_from(installed: &[bool; N]) -> (bool, bool, bool) {
    (
        installed[GEAR_PIN_NOSE],
        installed[GEAR_PIN_LWING] || installed[GEAR_PIN_LBODY],
        installed[GEAR_PIN_RWING] || installed[GEAR_PIN_RBODY],
    )
}

/// The systems variable each item's state is mirrored into, as the web
/// Study panel's `/vars` poll names it. That poll reads the variable
/// snapshot, which holds no X-Plane datarefs, so without the mirror the web
/// tab could not show whether an item is on.
pub fn mirror_name(id: &str) -> String {
    format!("{}WALKAROUND_{}", crate::NAME_PREFIX, id.to_ascii_uppercase())
}

/// The ADIRS-blocking failure levels the currently-installed probe covers
/// cause, for `failures::set_walkaround_levels`.
fn failure_levels_from(installed: &[bool; N]) -> BTreeMap<u64, f64> {
    let mut levels = BTreeMap::new();
    for n in 0..3usize {
        if installed[PITOT[n]] {
            levels.insert(pitot_fail_id(n), 1.0);
        }
        if installed[AOA[n]] {
            levels.insert(aoa_fail_id(n), 1.0);
        }
    }
    if installed[STATIC_COVERS] {
        for id in static_fail_ids() {
            levels.insert(id, 1.0);
        }
    }
    // A downlock pin stops its leg's retraction actuator from moving: that
    // is FlyByWire's own GearActuatorJammed (32_020 GearNose, 32_021
    // GearLeft, 32_022 GearRight), which the A380's HydraulicGearSystem
    // reads (fbw-common hydraulic/landing_gear.rs), so its gear model, the
    // LGCIUs and the ECAM all see a leg that will not come up.
    let (nose, left, right) = gear_block_from(installed);
    for (pinned, id) in [(nose, 32_020), (left, 32_021), (right, 32_022)] {
        if pinned {
            levels.insert(id, 1.0);
        }
    }
    levels
}

/// The exterior walkaround in X-Plane: owned datarefs/commands plus the
/// live state driving them.
pub struct Walkaround {
    installed: [bool; N],
    _published: Published,
    values: [Value; N],
    toggles: [Command; N],
    any_installed: Value,
    remove_all: Command,
    install_all: Command,
    n2: [VariableIdentifier; 4],
    /// Each item's state as a systems variable ([`mirror_name`]).
    mirror: [VariableIdentifier; N],
    groundspeed_ms: Option<DataRef>,
    /// Log lines from this tick's automatic changes (ingestion, blow-out,
    /// chock overrun), drained by the caller.
    pub events: Vec<String>,
    /// 0-based engine indices this tick's ingestion armed FOD damage on,
    /// drained by [`Walkaround::take_fod_events`] so the caller can call
    /// into `physics::damage::Damage::arm_fod` (this module does not depend
    /// on `Damage`, to keep its own logic pure and testable).
    fod_events: Vec<usize>,
}

impl Walkaround {
    /// Everything starts removed: which items are really on is decided by
    /// [`Walkaround::confirm_start`] once X-Plane has placed the aircraft.
    /// Deciding here, at plugin load, read X-Plane before the aircraft was
    /// placed: a runway start with engines running looked cold and dark,
    /// every item went on, and all four engines ingested their covers the
    /// moment the start completed (2026-09-29).
    pub fn new(vars: &mut Vars, xplm: &Xplm, saved: &BTreeMap<String, bool>) -> Self {
        let _ = saved;
        let installed = [false; N];

        let mut p = Published::default();
        let values: [Value; N] = std::array::from_fn(|i| p.number(&format!("fbw/walkaround/{}", ITEMS[i]), installed[i] as i32 as f64, true));
        let toggles: [Command; N] =
            std::array::from_fn(|i| p.command(&format!("fbw/walkaround/{}_toggle", ITEMS[i]), &format!("Walkaround: toggle {}", ITEMS[i])));
        let any_installed = p.number("fbw/walkaround/any_installed", installed.iter().any(|&b| b) as i32 as f64, false);
        let remove_all = p.command("fbw/walkaround/remove_all", "Walkaround: remove every item");
        let install_all = p.command("fbw/walkaround/install_all", "Walkaround: install every item");

        Self {
            installed,
            _published: p,
            values,
            toggles,
            any_installed,
            remove_all,
            install_all,
            n2: std::array::from_fn(|i| vars.get(format!("ENGINE_N2:{}", i + 1))),
            mirror: std::array::from_fn(|i| vars.get(mirror_name(ITEMS[i]))),
            groundspeed_ms: xplm.find("sim/flightmodel/position/groundspeed"),
            events: Vec::new(),
            fod_events: Vec::new(),
        }
    }

    /// The start state as confirmed once X-Plane has placed the aircraft
    /// (`Plugin::confirm_start_state`, before the first tick that runs
    /// anything): items go on or off silently -- nobody fitted or removed
    /// anything, this is how the aircraft was found.
    pub fn confirm_start(&mut self, cold_and_dark: bool, saved: &BTreeMap<String, bool>) {
        self.installed = initial_state(cold_and_dark, saved);
        for i in 0..N {
            published::set(self.values[i], self.installed[i] as i32 as f64);
        }
        published::set(self.any_installed, self.installed.iter().any(|&b| b) as i32 as f64);
        crate::log(&format!(
            "walkaround: start ({}): {}/{} items installed",
            if cold_and_dark { "cold and dark" } else { "engine(s) running or airborne" },
            self.installed.iter().filter(|&&b| b).count(),
            N
        ));
    }

    fn set(&mut self, i: usize, want: bool) {
        if self.installed[i] != want {
            self.installed[i] = want;
            self.events.push(format!("walkaround: {} {}", ITEMS[i], if want { "installed" } else { "removed" }));
        }
    }

    /// Before the systems (X-Plane's own commands/writes take effect right
    /// away) and after them is both fine for this module: nothing here
    /// reads a systems output but this tick's N2/groundspeed, which are
    /// X-Plane/engine values, not FlyByWire's. Called once per tick.
    pub fn update(&mut self, vars: &mut Vars, xplm: &Xplm) {
        self.events.clear();
        // A direct write to the dataref (e.g. a 3D click-spot bound straight
        // to it) wins over a stale toggle-command press the same tick --
        // the same "a value written from outside is where it is now" rule
        // `doors.rs` uses for its own owned position dataref.
        for i in 0..N {
            if let Some(v) = published::take(self.values[i]) {
                self.set(i, v > 0.5);
            }
        }
        for i in 0..N {
            for _ in 0..published::presses(self.toggles[i]) {
                self.set(i, !self.installed[i]);
            }
        }
        for _ in 0..published::presses(self.remove_all) {
            for i in 0..N {
                self.set(i, false);
            }
        }
        for _ in 0..published::presses(self.install_all) {
            for i in 0..N {
                self.set(i, true);
            }
        }

        let n2_percent: [f64; 4] = std::array::from_fn(|i| vars.read(&self.n2[i]));
        let (fod, lines) = ingest_and_blow_out(&mut self.installed, n2_percent);
        self.fod_events.extend(fod);
        self.events.extend(lines);

        let groundspeed_ms = self.groundspeed_ms.map_or(0.0, |d| xplm.get_f(d) as f64);
        self.events.extend(chock_overrun(&mut self.installed, groundspeed_ms));

        for i in 0..N {
            published::set(self.values[i], self.installed[i] as i32 as f64);
            vars.write(&self.mirror[i], self.installed[i] as i32 as f64);
        }
        published::set(self.any_installed, self.installed.iter().any(|&b| b) as i32 as f64);

        crate::failures::set_walkaround_levels(failure_levels_from(&self.installed));
    }

    /// This tick's automatically-armed FOD events (engine-inlet ingestion),
    /// as 0-based engine indices, taking them out of the queue.
    pub fn take_fod_events(&mut self) -> Vec<usize> {
        std::mem::take(&mut self.fod_events)
    }


    /// The current installed/removed state, for `persistence.rs`.
    pub fn snapshot(&self) -> BTreeMap<String, bool> {
        ITEMS.iter().enumerate().map(|(i, &name)| (name.to_owned(), self.installed[i])).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cold_and_dark_with_no_save_installs_everything() {
        let saved = BTreeMap::new();
        assert_eq!(initial_state(true, &saved), [true; N]);
    }

    #[test]
    fn cold_and_dark_with_a_save_restores_it() {
        let mut saved = BTreeMap::new();
        for name in ITEMS {
            saved.insert(name.to_owned(), false);
        }
        saved.insert("gear_pin_nose".to_owned(), true);
        let state = initial_state(true, &saved);
        assert!(state[GEAR_PIN_NOSE]);
        assert!(!state[0], "everything else in the save was removed");
    }

    #[test]
    fn a_save_missing_a_newer_item_defaults_that_one_to_installed() {
        // A save from before some item existed: present but incomplete.
        let mut saved = BTreeMap::new();
        saved.insert("chocks_nose".to_owned(), false);
        let state = initial_state(true, &saved);
        assert!(!state[CHOCKS[0]]);
        assert!(state[0], "an item the save never mentions defaults to installed");
    }

    #[test]
    fn engines_running_or_airborne_removes_everything_regardless_of_any_save() {
        let mut saved = BTreeMap::new();
        for name in ITEMS {
            saved.insert(name.to_owned(), true);
        }
        assert_eq!(initial_state(false, &saved), [false; N]);
    }

    #[test]
    fn a_rotating_engine_ingests_its_inlet_cover_and_arms_fod_only_for_that_engine() {
        let mut installed = [true; N];
        let n2 = [ENGINE_ROTATING_N2_PERCENT + 5.0, 0.0, 0.0, 0.0];
        let (fod, lines) = ingest_and_blow_out(&mut installed, n2);
        assert_eq!(fod, vec![0]);
        assert!(!installed[INLET[0]], "the ingested cover is destroyed");
        assert!(!installed[EXHAUST[0]], "the exhaust plug blows out the same way");
        assert!(installed[INLET[1]], "an idle engine's cover is untouched");
        assert!(installed[EXHAUST[1]]);
        assert_eq!(lines.len(), 2);
    }

    #[test]
    fn an_idle_engine_never_ingests_its_cover() {
        let mut installed = [true; N];
        let (fod, lines) = ingest_and_blow_out(&mut installed, [0.0; 4]);
        assert!(fod.is_empty());
        assert!(lines.is_empty());
        assert!(installed[INLET[0]]);
    }

    #[test]
    fn removed_inlet_covers_are_never_re_ingested() {
        let mut installed = [false; N];
        let (fod, lines) = ingest_and_blow_out(&mut installed, [50.0; 4]);
        assert!(fod.is_empty(), "nothing to ingest once already removed");
        assert!(lines.is_empty());
    }

    #[test]
    fn chocks_overrun_above_walking_pace_and_never_below_it() {
        let mut installed = [true; N];
        let lines = chock_overrun(&mut installed, 0.1);
        assert!(lines.is_empty(), "a stationary, chocked aircraft must never trip this");
        assert!(installed[CHOCKS[0]]);
        let lines = chock_overrun(&mut installed, 2.0);
        assert_eq!(lines.len(), CHOCKS.len());
        for &idx in &CHOCKS {
            assert!(!installed[idx]);
        }
    }

    #[test]
    fn a_gear_pin_jams_its_sides_actuator_in_flybywires_gear_model() {
        let mut installed = [false; N];
        let jams = |i: &[bool; N]| failure_levels_from(i).keys().copied().filter(|id| (32_020..=32_025).contains(id)).collect::<Vec<_>>();
        assert!(jams(&installed).is_empty());
        installed[GEAR_PIN_NOSE] = true;
        assert_eq!(jams(&installed), vec![32_020], "nose pin: GearActuatorJammed(GearNose)");
        installed = [false; N];
        installed[GEAR_PIN_LWING] = true;
        assert_eq!(jams(&installed), vec![32_021], "left wing pin: the left retraction group");
        installed = [false; N];
        installed[GEAR_PIN_RBODY] = true;
        assert_eq!(jams(&installed), vec![32_022], "right body pin: the right retraction group");
    }

    #[test]
    fn gear_pins_combine_wing_and_body_per_side() {
        let mut installed = [false; N];
        assert_eq!(gear_block_from(&installed), (false, false, false));
        installed[GEAR_PIN_LBODY] = true;
        assert_eq!(gear_block_from(&installed), (false, true, false), "a body pin alone still blocks its side's shared retraction group");
        installed = [false; N];
        installed[GEAR_PIN_NOSE] = true;
        assert_eq!(gear_block_from(&installed), (true, false, false));
    }

    #[test]
    fn a_pitot_cover_maps_to_its_own_adiru_only() {
        let mut installed = [false; N];
        installed[PITOT[1]] = true; // pitot_cover_2
        let levels = failure_levels_from(&installed);
        assert_eq!(levels.get(&34_101), Some(&1.0));
        assert_eq!(levels.len(), 1, "no other ADIRU's pitot, and no static/AoA id, is touched");
    }

    #[test]
    fn the_single_static_covers_item_blocks_all_three_adirus_at_once() {
        let mut installed = [false; N];
        installed[STATIC_COVERS] = true;
        let levels = failure_levels_from(&installed);
        assert_eq!(levels.keys().copied().collect::<Vec<_>>(), vec![34_103, 34_104, 34_105]);
    }

    #[test]
    fn an_aoa_cover_maps_to_its_own_adiru_only() {
        let mut installed = [false; N];
        installed[AOA[2]] = true; // aoa_cover_3
        let levels = failure_levels_from(&installed);
        assert_eq!(levels.get(&34_108), Some(&1.0));
        assert_eq!(levels.len(), 1);
    }

    #[test]
    fn no_covers_means_no_failure_levels() {
        assert!(failure_levels_from(&[false; N]).is_empty());
    }

    #[test]
    fn items_and_groups_partition_every_index_exactly_once() {
        let mut seen = [0u32; N];
        for &i in GROUP_PROBES.iter().chain(&GROUP_ENGINES).chain(&GROUP_GEAR_PINS).chain(&GROUP_CHOCKS) {
            seen[i] += 1;
        }
        assert!(seen.iter().all(|&c| c == 1), "every item must be in exactly one Study-page group: {seen:?}");
    }
}
