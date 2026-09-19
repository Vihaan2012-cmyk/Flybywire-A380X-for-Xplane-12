//! FlyByWire's cockpit sounds, played from the user's own FlyByWire MSFS
//! package through X-Plane's sound API.
//!
//! - [`wwise`]: the package's Wwise soundbanks (`sound/*.PCK`): media and the
//!   events that play them.
//! - [`vorbis`]: Wwise's Vorbis media decoded to PCM.
//! - [`triggers`]: `sound.xml`, as MSFS reads it.
//!
//! ## Threads
//!
//! The instruments' scripts (`Coherent.call('PLAY_INSTRUMENT_SOUND', name)`,
//! `LegacySoundManager`, `FwsSoundManager`) will run on a worker thread once
//! the lead moves them off the main one. [`play_instrument_sound`] is called
//! from wherever that ends up, so it only ever pushes the name onto
//! [`INSTRUMENT_QUEUE`], a `Mutex`, never a `thread_local!` (a worker thread
//! would have its own copy of one of those, and this plugin's XPLM audio
//! calls would never see it): the main thread's [`Sound::update`] drains the
//! queue and does the actual work.
//!
//! Three other things run off the main thread, none of them ever touching
//! XPLM:
//! - a one-shot loader thread that reads and indexes the package's banks and
//!   `sound.xml` ([`spawn_loader`]);
//! - a decode worker thread that turns a requested media id into PCM
//!   ([`decode_worker`]) and drops it into a shared, `Mutex`-guarded
//!   [`PcmCache`] bounded to [`CACHE_BUDGET_BYTES`];
//! - nothing else: [`Sound::update`], the only thing that calls
//!   `play_pcm16_on_bus`/`stop_audio`/`set_audio_volume`, runs on the main
//!   thread's flight loop only.
//!
//! ## What plays
//!
//! Each tick, `sound.xml`'s `SimVarSounds` triggers are stepped from their
//! `L:`/simulator variable ([`triggers::TriggerState::step`]); a continuous
//! one starts a loop while its condition holds and stops it when the
//! condition leaves, a one-shot plays once on entry. `AvionicSounds` have no
//! trigger of their own: the instruments ask for them by name through
//! [`play_instrument_sound`], mapped to a bank event exactly as MSFS does
//! ([`wwise::msfs_event_id`]) since FlyByWire's own sound managers already
//! pass sound.xml's `WwiseEvent` names straight through
//! (`LegacySoundManager.ts` `soundList`, `FwsSoundManager.ts`'s
//! `wwiseEventName`s).
//!
//! A resolved event's Play actions are flattened to the leaf sounds they
//! reach ([`flatten`]): a Layer container plays every child at once, matching
//! Wwise; a Random container picks one child by its weight, but with no
//! avoid-repeat or shuffle state kept between picks; a Sequence container
//! always plays its first child, not the full step/continuous playlist state
//! Wwise keeps. These are the simplifications this reader makes to stay
//! within scope; the common case in these banks is a single `Sound` or a
//! Random container of variants, both handled faithfully. A leaf sound's
//! volume is its own Volume and Make-Up Gain properties plus its containers'
//! and its Wwise-hierarchy parents' Volume, summed in dB and converted to
//! linear gain for `XPLMSetAudioVolume`; loop is the sound.xml Continuous
//! state for a `SimVarSounds` trigger (`true` starts a loop, `false` forces
//! non-looping even if the media's own Wwise Loop property says otherwise —
//! a `Continuous="false"` entry is a one-shot by the sound.xml contract, so
//! nothing may leave it looping forever with no trigger index to ever stop
//! it by), else, for an `AvionicSounds` name with no sound.xml Continuous of
//! its own, the Wwise Loop property (0 = forever, else once — a specific
//! finite repeat count is not reproduced). Everything
//! plays on the interior bus, flat (X-Plane's `XPLMPlayPCMOnBus` has no 3D
//! position parameter to place a positional sound at, so [`wwise`]'s
//! `positional` flag is read but not used here).
//!
//! `<WwiseRTPC>`-driven entries are left out by [`triggers::parse`] itself
//! (see its doc comment); an event's own Stop actions (as opposed to
//! sound.xml's Continuous/StopLoop semantics, which this does honour) are not
//! applied, since they are rare in these banks and resolving their scope
//! correctly needs more of Wwise's runtime state than this reader keeps.

pub mod triggers;
pub mod vorbis;
pub mod wwise;

use std::collections::{HashMap, HashSet, VecDeque};
use std::path::{Path, PathBuf};
use std::sync::mpsc::{self, Receiver, Sender, TryRecvError};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use systems::simulation::SimulatorReaderWriter;

use crate::xp;
use crate::Vars;
use triggers::{Action, SoundXml, TriggerState};
use vorbis::Pcm;
use wwise::{ActionKind, ContainerKind, Package, PlayNode};

// ---------------------------------------------------------------------------
// Instrument sounds: `Coherent.call('PLAY_INSTRUMENT_SOUND', name)`.

/// Names queued by [`play_instrument_sound`] for the next main-thread tick.
static INSTRUMENT_QUEUE: Mutex<Vec<String>> = Mutex::new(Vec::new());

/// `Coherent.call('PLAY_INSTRUMENT_SOUND', name)` and FlyByWire's
/// `LegacySoundManager`/`FwsSoundManager` aurals, whose `name` is already the
/// bank event's `sound.xml` `WwiseEvent` name. Thread-safe and never touches
/// XPLM: only queues the request for [`Sound::update`] to resolve and play on
/// the main thread. Safe to call from the instruments' worker thread once the
/// lead moves them off the main one.
pub fn play_instrument_sound(name: &str) {
    if let Ok(mut queue) = INSTRUMENT_QUEUE.lock() {
        queue.push(name.to_owned());
    }
}

// ---------------------------------------------------------------------------
// Loading

/// What the loader thread hands back.
struct Loaded {
    package: Arc<Package>,
    sound_xml: SoundXml,
    /// `sound.xml`'s `<MainPackage Name="...">`, for [`wwise::msfs_event_id`].
    main_package: String,
}

fn load(sound_dir: &Path) -> Result<Loaded, String> {
    let package = Package::load_dir(sound_dir)?;
    let xml_path = sound_dir.join("sound.xml");
    let xml_text = std::fs::read_to_string(&xml_path).map_err(|e| format!("{}: {e}", xml_path.display()))?;
    let sound_xml = triggers::parse(&xml_text)?;
    let main_package = wwise::sound_xml_main_package(&xml_text).unwrap_or("Asobo_A320_NEO").to_owned();
    Ok(Loaded { package: Arc::new(package), sound_xml, main_package })
}

/// Reads and indexes the package's banks and `sound.xml` off the main
/// thread; large packages take real time to parse (the three FlyByWire
/// packages, ~1500 media items) and must not stall the flight loop.
fn spawn_loader(sound_dir: PathBuf) -> Receiver<Result<Loaded, String>> {
    let (tx, rx) = mpsc::channel();
    std::thread::spawn(move || {
        let _ = tx.send(load(&sound_dir));
    });
    rx
}

/// MSFS's `UserCfg.opt`'s `InstalledPackagesPath "..."`, from the standard
/// Microsoft Store and Steam/boxed locations for it.
fn installed_packages_path() -> Option<PathBuf> {
    let mut candidates = Vec::new();
    if let Some(local) = std::env::var_os("LOCALAPPDATA") {
        candidates.push(
            PathBuf::from(local)
                .join("Packages")
                .join("Microsoft.FlightSimulator_8wekyb3d8bbwe")
                .join("LocalCache")
                .join("UserCfg.opt"),
        );
    }
    if let Some(roaming) = std::env::var_os("APPDATA") {
        candidates.push(PathBuf::from(roaming).join("Microsoft Flight Simulator").join("UserCfg.opt"));
    }
    candidates.iter().find_map(|p| std::fs::read_to_string(p).ok().and_then(|t| parse_installed_packages_path(&t)))
}

/// `InstalledPackagesPath "D:\..."`, one line among UserCfg.opt's others.
fn parse_installed_packages_path(text: &str) -> Option<PathBuf> {
    let value = text.lines().find_map(|line| line.trim().strip_prefix("InstalledPackagesPath"))?.trim().trim_matches('"');
    (!value.is_empty()).then(|| PathBuf::from(value))
}

/// This plugin's own fallback, for anyone whose `UserCfg.opt` is somewhere
/// else: a `package=<InstalledPackagesPath>` line. Not written by the plugin
/// itself; a manual override.
fn sound_ini_path() -> PathBuf {
    PathBuf::from("Output").join("preferences").join("fbw_a380x_sound.ini")
}

fn ini_override_path() -> Option<PathBuf> {
    let text = std::fs::read_to_string(sound_ini_path()).ok()?;
    let value = text.lines().find_map(|line| line.trim().strip_prefix("package="))?.trim();
    (!value.is_empty()).then(|| PathBuf::from(value))
}

/// The FlyByWire A380X's `sound` folder inside the user's installed MSFS
/// packages, or `None` when it cannot be found or is not there.
fn find_sound_dir() -> Option<PathBuf> {
    let installed = installed_packages_path().or_else(ini_override_path)?;
    let dir = installed
        .join("Community")
        .join("flybywire-aircraft-a380-842")
        .join("SimObjects")
        .join("AirPlanes")
        .join("FlyByWire_A380_842")
        .join("sound");
    dir.is_dir().then_some(dir)
}

// ---------------------------------------------------------------------------
// Decoding and the PCM cache

/// Decoded PCM, most recently touched last, bounded to
/// [`CACHE_BUDGET_BYTES`]. Shared between the decode worker (which inserts)
/// and the main thread (which reads and evicts), behind a `Mutex`.
#[derive(Default)]
struct PcmCache {
    entries: HashMap<u32, Arc<Pcm>>,
    order: VecDeque<u32>,
    bytes: usize,
}

/// How much decoded PCM this plugin keeps cached at once: 16-bit samples, so
/// roughly 4.5 minutes of stereo audio at 44.1 kHz — enough to hold every
/// callout and chime this aircraft is likely to play in a short span, without
/// growing without bound over a long flight.
const CACHE_BUDGET_BYTES: usize = 48 * 1024 * 1024;

impl PcmCache {
    fn get(&mut self, id: u32) -> Option<Arc<Pcm>> {
        let pcm = self.entries.get(&id)?.clone();
        self.touch(id);
        Some(pcm)
    }

    fn touch(&mut self, id: u32) {
        if let Some(pos) = self.order.iter().position(|&x| x == id) {
            self.order.remove(pos);
        }
        self.order.push_back(id);
    }

    fn insert(&mut self, id: u32, pcm: Arc<Pcm>) {
        let bytes = pcm.samples.len() * 2;
        if self.entries.insert(id, pcm).is_none() {
            self.bytes += bytes;
        }
        self.touch(id);
        while self.bytes > CACHE_BUDGET_BYTES {
            let Some(oldest) = self.order.pop_front() else { break };
            if oldest == id {
                // Nothing smaller left to evict; keep the one just inserted.
                self.order.push_front(oldest);
                break;
            }
            if let Some(removed) = self.entries.remove(&oldest) {
                self.bytes = self.bytes.saturating_sub(removed.samples.len() * 2);
            }
        }
    }
}

/// Decodes whatever media id arrives on `rx` and drops it into `cache`.
/// Never touches XPLM: this is pure CPU work (lewton's Vorbis decode, or a
/// PCM copy), safe to run off the main thread.
fn decode_worker(package: Arc<Package>, cache: Arc<Mutex<PcmCache>>, pending: Arc<Mutex<HashSet<u32>>>, errors: Arc<Mutex<HashMap<u32, String>>>, rx: Receiver<u32>) {
    for media_id in rx {
        let already_cached = cache.lock().map(|c| c.entries.contains_key(&media_id)).unwrap_or(true);
        if !already_cached {
            let decoded = package.media(media_id).ok_or_else(|| format!("media {media_id}: in no bank")).and_then(vorbis::decode_wem);
            match decoded {
                Ok(pcm) => {
                    if let Ok(mut cache) = cache.lock() {
                        cache.insert(media_id, Arc::new(pcm));
                    }
                }
                Err(e) => {
                    if let Ok(mut errors) = errors.lock() {
                        errors.insert(media_id, e);
                    }
                }
            }
        }
        if let Ok(mut pending) = pending.lock() {
            pending.remove(&media_id);
        }
    }
}

// ---------------------------------------------------------------------------
// Flattening a Play target to the leaf sounds it reaches

/// One sound to actually play: its media, combined volume in dB, and whether
/// it loops.
struct FlatSound {
    media_id: u32,
    looped: bool,
    volume_db: f32,
}

/// xorshift64: good enough to pick among a Random container's children
/// without another dependency; not used for anything else.
fn xorshift(state: &mut u64) -> u64 {
    let mut x = *state;
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    *state = x;
    x
}

/// A Random container's weighted pick (Wwise's default weight is 50000; any
/// non-positive or mismatched weight list falls back to the first child).
/// Keeps no avoid-repeat or shuffle state between calls, unlike Wwise.
fn weighted_pick<'a>(children: &'a [PlayNode], weights: &[i32], rng: &mut u64) -> Option<&'a PlayNode> {
    if children.is_empty() {
        return None;
    }
    let total: i64 = weights.iter().map(|&w| w.max(0) as i64).sum();
    if weights.len() != children.len() || total <= 0 {
        return children.first();
    }
    let mut roll = (xorshift(rng) % total as u64) as i64;
    for (child, &w) in children.iter().zip(weights) {
        roll -= w.max(0) as i64;
        if roll < 0 {
            return Some(child);
        }
    }
    children.last()
}

/// Flattens a resolved Play target to the leaf sounds it reaches, with each
/// leaf's combined dB (this node's Volume plus its ancestors') and its loop
/// request: `force_loop` overrides Wwise's own Loop property, for a
/// sound.xml continuous trigger, whose loop is the trigger's condition, not
/// the bank's.
fn flatten(node: &PlayNode, gain_db: f32, rng: &mut u64, force_loop: Option<bool>, out: &mut Vec<FlatSound>) {
    match node {
        PlayNode::Sound(s) => out.push(FlatSound {
            media_id: s.media_id,
            looped: force_loop.unwrap_or(s.loop_count == 0),
            volume_db: gain_db + s.volume_db + s.make_up_gain_db,
        }),
        PlayNode::Container(c) => {
            let gain_db = gain_db + c.volume_db;
            match c.kind {
                ContainerKind::Layer => {
                    for child in &c.children {
                        flatten(child, gain_db, rng, force_loop, out);
                    }
                }
                ContainerKind::Random => {
                    if let Some(child) = weighted_pick(&c.children, &c.weights, rng) {
                        flatten(child, gain_db, rng, force_loop, out);
                    }
                }
                ContainerKind::Sequence => {
                    if let Some(child) = c.children.first() {
                        flatten(child, gain_db, rng, force_loop, out);
                    }
                }
            }
        }
        PlayNode::Missing(_) | PlayNode::Unsupported { .. } => {}
    }
}

// ---------------------------------------------------------------------------
// Playback

/// A leaf sound waiting for its media to finish decoding.
struct PendingLeaf {
    /// The sound.xml trigger this belongs to, for [`Sound::stop_trigger`];
    /// `None` for a one-shot (an avionic sound or a non-continuous trigger),
    /// which nothing ever needs to stop early.
    trigger: Option<usize>,
    media_id: u32,
    looped: bool,
    volume_db: f32,
    since: Instant,
}

/// How long a leaf sound waits for its decode before this plugin gives up on
/// it (a stuck or failed decode should not grow [`Sound::pending`] forever).
const PENDING_TIMEOUT: Duration = Duration::from_secs(10);

/// What's loaded: the package, `sound.xml`, and the machinery to decode and
/// play from them.
struct State {
    package: Arc<Package>,
    main_package: String,
    sound_xml: SoundXml,
    /// One per `sound_xml.triggers`, stepped every tick.
    trigger_states: Vec<TriggerState>,
    cache: Arc<Mutex<PcmCache>>,
    decode_tx: Sender<u32>,
    /// Media ids already requested from the decode worker, so a sound played
    /// every tick does not queue the same request every tick.
    decode_pending: Arc<Mutex<HashSet<u32>>>,
    decode_errors: Arc<Mutex<HashMap<u32, String>>>,
    /// FMOD channels a continuous trigger started, by its index into
    /// `sound_xml.triggers`, stopped when its condition leaves.
    active: HashMap<usize, Vec<xp::FmodChannel>>,
}

/// FlyByWire's cockpit sounds: loads the user's own MSFS package off the
/// main thread, decodes media lazily on a worker, and plays through XPLM's
/// PCM bus from the main thread's tick. See the module doc comment for the
/// thread model and what is and isn't reproduced.
pub struct Sound {
    load_rx: Option<Receiver<Result<Loaded, String>>>,
    state: Option<State>,
    pending: Vec<PendingLeaf>,
    rng_state: u64,
    logged_no_audio: bool,
}

impl Sound {
    /// Starts loading the user's FlyByWire A380X package in the background.
    /// Nothing here touches XPLM or the systems' variables, so it needs
    /// neither `Vars` nor `Xplm`.
    pub fn new() -> Self {
        let load_rx = find_sound_dir().map(spawn_loader);
        if load_rx.is_none() {
            crate::log("sound: no FlyByWire A380X package found (checked UserCfg.opt and Output/preferences/fbw_a380x_sound.ini); cockpit sounds are off");
        }
        let seed = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_nanos() as u64).unwrap_or(0x9E37_79B9);
        Self { load_rx, state: None, pending: Vec::new(), rng_state: seed | 1, logged_no_audio: false }
    }

    /// One tick: pick up the loaded package once it's ready, drain
    /// instrument sound requests, step `sound.xml`'s triggers, and retry any
    /// sound still waiting on its decode. Main thread only: this is the only
    /// place that calls XPLM's audio functions.
    pub fn update(&mut self, vars: &mut Vars) {
        self.poll_load();
        self.drain_instrument_queue();
        self.evaluate_triggers(vars);
        self.process_pending();
    }

    /// Stops what's still looping. Called once, when the plugin is disabled:
    /// a one-shot is short enough to just let finish.
    pub fn release(&mut self) {
        if let Some(state) = self.state.as_mut() {
            for channels in state.active.values() {
                for &channel in channels {
                    xp::stop_audio(channel);
                }
            }
            state.active.clear();
        }
        self.pending.clear();
    }

    fn poll_load(&mut self) {
        if self.state.is_some() {
            return;
        }
        let Some(rx) = &self.load_rx else { return };
        match rx.try_recv() {
            Ok(Ok(loaded)) => {
                self.load_rx = None;
                self.install(loaded);
            }
            Ok(Err(e)) => {
                self.load_rx = None;
                crate::log(&format!("sound: {e}; cockpit sounds are off"));
            }
            Err(TryRecvError::Empty) => {}
            Err(TryRecvError::Disconnected) => self.load_rx = None,
        }
    }

    /// Builds playback state from what the loader (or, in a test, a direct
    /// read of the real package) found, and starts the decode worker.
    fn install(&mut self, loaded: Loaded) {
        let cache = Arc::new(Mutex::new(PcmCache::default()));
        let decode_pending = Arc::new(Mutex::new(HashSet::new()));
        let decode_errors = Arc::new(Mutex::new(HashMap::new()));
        let (decode_tx, decode_rx) = mpsc::channel();
        {
            let package = loaded.package.clone();
            let cache = cache.clone();
            let decode_pending = decode_pending.clone();
            let decode_errors = decode_errors.clone();
            std::thread::spawn(move || decode_worker(package, cache, decode_pending, decode_errors, decode_rx));
        }
        crate::log(&format!(
            "sound: {} banks, {} sound.xml triggers ({} left out for their WwiseRTPC), {} avionic sounds, main package {}",
            loaded.package.banks.len(),
            loaded.sound_xml.triggers.len(),
            loaded.sound_xml.skipped_rtpc.len(),
            loaded.sound_xml.avionic_events.len(),
            loaded.main_package,
        ));
        let trigger_states = vec![TriggerState::default(); loaded.sound_xml.triggers.len()];
        self.state = Some(State {
            package: loaded.package,
            main_package: loaded.main_package,
            sound_xml: loaded.sound_xml,
            trigger_states,
            cache,
            decode_tx,
            decode_pending,
            decode_errors,
            active: HashMap::new(),
        });
    }

    fn drain_instrument_queue(&mut self) {
        let names = match INSTRUMENT_QUEUE.lock() {
            Ok(mut queue) => std::mem::take(&mut *queue),
            Err(_) => return,
        };
        for name in names {
            self.fire_event(&name, None, None);
        }
    }

    fn evaluate_triggers(&mut self, vars: &mut Vars) {
        let Some(state) = &mut self.state else { return };
        let mut actions = Vec::new();
        for i in 0..state.sound_xml.triggers.len() {
            let value = read_named(vars, &state.sound_xml.triggers[i].variable);
            // Every <Requires> gate on this entry (sound.xml, 30 entries in
            // the package use one) must also hold for MSFS to consider it
            // inside; with none, this is vacuously true.
            let requires_hold = state.sound_xml.triggers[i].requires.iter().all(|r| r.holds(read_named(vars, &r.variable)));
            let action = state.trigger_states[i].step(&state.sound_xml.triggers[i], value, requires_hold);
            if action != Action::None {
                actions.push((i, action, state.sound_xml.triggers[i].event.clone()));
            }
        }
        for (i, action, event) in actions {
            match action {
                // Continuous="false": MSFS plays this to completion exactly
                // once per entry, never looping, regardless of the Wwise
                // media's own Loop property. Real bug (X-Plane 12 session):
                // deferring to the bank's Loop property here (as
                // AvionicSounds legitimately do below, having no sound.xml
                // Continuous of their own) let a one-shot cabin PA authored
                // with an infinite Loop property play nonstop, and since
                // PlayOnce sounds pass no trigger index, `stop_trigger` could
                // never reach it either — forcing non-looping is the only
                // fix, not just tracking it to stop later.
                Action::PlayOnce => self.fire_event(&event, Some(false), None),
                Action::StartLoop => self.fire_event(&event, Some(true), Some(i)),
                Action::StopLoop => self.stop_trigger(i),
                Action::None => {}
            }
        }
    }

    /// Resolves a `sound.xml` `WwiseEvent` name to the bank event MSFS would
    /// post for it, and plays (or queues) every leaf sound its Play actions
    /// reach.
    fn fire_event(&mut self, event_name: &str, force_loop: Option<bool>, trigger: Option<usize>) {
        let Some(state) = &self.state else { return };
        let event_id = wwise::msfs_event_id(&state.main_package, event_name);
        let Some(playback) = state.package.resolve_event(event_id) else { return };
        let mut rng = self.rng_state;
        let mut leaves = Vec::new();
        for action in &playback.actions {
            if let ActionKind::Play { node, parent_volume_db } = &action.kind {
                flatten(node, *parent_volume_db, &mut rng, force_loop, &mut leaves);
            }
        }
        self.rng_state = rng;
        for leaf in leaves {
            self.play_or_queue(leaf.media_id, leaf.looped, leaf.volume_db, trigger);
        }
    }

    fn stop_trigger(&mut self, trigger: usize) {
        self.pending.retain(|p| p.trigger != Some(trigger));
        if let Some(state) = self.state.as_mut() {
            if let Some(channels) = state.active.remove(&trigger) {
                for channel in channels {
                    xp::stop_audio(channel);
                }
            }
        }
    }

    fn play_or_queue(&mut self, media_id: u32, looped: bool, volume_db: f32, trigger: Option<usize>) {
        if let Some(pcm) = self.cache_get(media_id) {
            self.play_pcm(&pcm, looped, volume_db, trigger);
            return;
        }
        self.request_decode(media_id);
        self.pending.push(PendingLeaf { trigger, media_id, looped, volume_db, since: Instant::now() });
    }

    fn cache_get(&self, media_id: u32) -> Option<Arc<Pcm>> {
        self.state.as_ref()?.cache.lock().ok()?.get(media_id)
    }

    fn request_decode(&self, media_id: u32) {
        let Some(state) = self.state.as_ref() else { return };
        let Ok(mut pending) = state.decode_pending.lock() else { return };
        if pending.insert(media_id) {
            let _ = state.decode_tx.send(media_id);
        }
    }

    /// Retries every leaf sound still waiting on its decode: plays it once
    /// its media is in the cache, drops it once its decode has failed or it
    /// has waited longer than [`PENDING_TIMEOUT`].
    fn process_pending(&mut self) {
        let mut i = 0;
        while i < self.pending.len() {
            let media_id = self.pending[i].media_id;
            if let Some(pcm) = self.cache_get(media_id) {
                let leaf = self.pending.remove(i);
                self.play_pcm(&pcm, leaf.looped, leaf.volume_db, leaf.trigger);
                continue;
            }
            let errored = self.state.as_ref().is_some_and(|s| s.decode_errors.lock().is_ok_and(|e| e.contains_key(&media_id)));
            if errored || self.pending[i].since.elapsed() > PENDING_TIMEOUT {
                self.pending.remove(i);
                continue;
            }
            i += 1;
        }
    }

    /// The only place that calls `XPLMPlayPCMOnBus`: always the interior
    /// bus, main thread only.
    fn play_pcm(&mut self, pcm: &Pcm, looped: bool, volume_db: f32, trigger: Option<usize>) {
        if !xp::has_pcm_audio() {
            if !self.logged_no_audio {
                self.logged_no_audio = true;
                crate::log("sound: no XPLMPlayPCMOnBus (needs X-Plane 12.04 or later); cockpit sounds are off");
            }
            return;
        }
        let Some(channel) = xp::play_pcm16_on_bus(&pcm.samples, pcm.sample_rate, pcm.channels, looped, xp::AUDIO_INTERIOR, None, std::ptr::null_mut()) else {
            return;
        };
        let gain = 10f32.powf(volume_db / 20.0);
        xp::set_audio_volume(channel, gain);
        if let Some(trigger) = trigger {
            if let Some(state) = self.state.as_mut() {
                state.active.entry(trigger).or_default().push(channel);
            }
        }
    }
}

impl Default for Sound {
    fn default() -> Self {
        Self::new()
    }
}

/// A named variable's current value, read straight from `Vars` without
/// registering it: a `sound.xml` trigger for a variable nothing else in this
/// plugin tracks (an unmapped MSFS simulator variable, or an `L:` var no
/// system writes) stays at 0, its real (absent) value, rather than a
/// fabricated one.
fn read_named(vars: &mut Vars, name: &str) -> f64 {
    match vars.ids.get(name).copied() {
        Some(id) => vars.read(&id),
        None => 0.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn installed_packages_path_reads_usercfg_syntax() {
        let text = "DisplayResolution 3840 2160 default\r\nInstalledPackagesPath \"D:\\Microsoft Flight Simulator 2020\\Microsoft Flight Simulator 2020 Packages\"\r\nLanguage \"en-US\"\r\n";
        let path = parse_installed_packages_path(text).unwrap();
        assert_eq!(path, PathBuf::from(r"D:\Microsoft Flight Simulator 2020\Microsoft Flight Simulator 2020 Packages"));
        assert!(parse_installed_packages_path("Language \"en-US\"").is_none());
    }

    #[test]
    fn cache_evicts_the_oldest_once_over_budget() {
        let mut cache = PcmCache::default();
        let make = |n: usize| Arc::new(Pcm { sample_rate: 44100, channels: 1, samples: vec![0i16; n], loop_frames: None });
        let half = CACHE_BUDGET_BYTES / 2 / 2; // half the budget, in samples (2 bytes each)
        cache.insert(1, make(half));
        cache.insert(2, make(half));
        assert!(cache.entries.contains_key(&1) && cache.entries.contains_key(&2));
        cache.insert(3, make(half));
        assert!(!cache.entries.contains_key(&1), "the least recently touched entry should have been evicted");
        assert!(cache.entries.contains_key(&2) && cache.entries.contains_key(&3));
        assert!(cache.bytes <= CACHE_BUDGET_BYTES + half * 2, "cache grew unbounded");
    }

    #[test]
    fn weighted_pick_never_leaves_the_list() {
        let children = [PlayNode::Missing(1), PlayNode::Missing(2), PlayNode::Missing(3)];
        let weights = [1, 1, 1];
        let mut rng = 42u64;
        for _ in 0..50 {
            let picked = weighted_pick(&children, &weights, &mut rng).unwrap();
            assert!(matches!(picked, PlayNode::Missing(1..=3)));
        }
    }

    /// A `sound.xml` real package builds `Sound` state without going through
    /// the loader thread or the directory search, for the tests below.
    fn test_sound() -> Option<Sound> {
        let dir = Path::new(wwise::PACKAGE_SOUND_DIR);
        if !dir.is_dir() {
            eprintln!("skipped: MSFS package not found at {}", wwise::PACKAGE_SOUND_DIR);
            return None;
        }
        let loaded = load(dir).expect("package loads");
        let mut sound = Sound { load_rx: None, state: None, pending: Vec::new(), rng_state: 0xDEAD_BEEF_1234_5678, logged_no_audio: false };
        sound.install(loaded);
        Some(sound)
    }

    /// Real bug (X-Plane 12 session, 2026): the cabin "prepare for landing"
    /// PA played nonstop, looping, on the ground at spawn. Root cause part
    /// 2: `Action::PlayOnce` (a sound.xml `Continuous="false"` entry) called
    /// `fire_event` with `force_loop: None`, deferring to the Wwise media's
    /// own Loop property (`SoundPlay::loop_count == 0` means "loop
    /// forever"). A one-shot announcement authored that way — or any bank
    /// content relying on a Stop action this reader does not apply (see the
    /// module doc) — would then loop forever, and since `PlayOnce` passes no
    /// trigger index, `stop_trigger` has nothing to reach: nothing could
    /// ever silence it, not even `Sound::release`. `flatten` must force
    /// `looped = false` whenever the caller passes `Some(false)`, regardless
    /// of the leaf's own `loop_count`.
    #[test]
    fn a_forced_non_loop_overrides_the_banks_own_infinite_loop_property() {
        let node = PlayNode::Sound(wwise::SoundPlay {
            id: 1,
            media_id: 42,
            media_bank_id: None,
            stream_type: 0,
            codec_plugin_id: 0,
            loop_count: 0, // authored as "loop forever"
            volume_db: 0.0,
            make_up_gain_db: 0.0,
            positional: false,
        });
        let mut rng = 1u64;
        let mut leaves = Vec::new();
        flatten(&node, 0.0, &mut rng, Some(false), &mut leaves);
        assert_eq!(leaves.len(), 1);
        assert!(!leaves[0].looped, "PlayOnce must force non-looping playback even when the bank's own Loop property says to loop forever");

        // Unforced (the AvionicSounds/instrument-sound path, which has no
        // sound.xml Continuous of its own): the bank's own property still
        // decides, as intended.
        let mut leaves2 = Vec::new();
        flatten(&node, 0.0, &mut rng, None, &mut leaves2);
        assert!(leaves2[0].looped, "with no sound.xml trigger, the bank's own Loop property should still apply");
    }

    #[test]
    fn flatten_reaches_real_leaf_media() {
        let Some(pkg) = wwise::test_package() else {
            eprintln!("skipped: MSFS package not found");
            return;
        };
        for name in ["new_retard", "cavcharge", "mastercaution"] {
            let pb = pkg.resolve_event(wwise::msfs_event_id("Asobo_A320_NEO", name)).unwrap_or_else(|| panic!("{name}"));
            let mut rng = 7u64;
            let mut leaves = Vec::new();
            for action in &pb.actions {
                if let ActionKind::Play { node, parent_volume_db } = &action.kind {
                    flatten(node, *parent_volume_db, &mut rng, Some(true), &mut leaves);
                }
            }
            assert!(!leaves.is_empty(), "{name}: no leaf sounds");
            for leaf in &leaves {
                assert!(leaf.looped, "{name}: force_loop should carry through the container");
                assert!(pkg.media(leaf.media_id).is_some(), "{name}: media {} in no bank", leaf.media_id);
            }
        }
    }

    /// The full path a `Coherent.call('PLAY_INSTRUMENT_SOUND', ...)` takes:
    /// [`play_instrument_sound`] queues the name from any thread,
    /// [`Sound::drain_instrument_queue`] resolves and flattens it, and the
    /// decode worker fills the cache the pending leaf is waiting on.
    #[test]
    fn play_instrument_sound_resolves_decodes_and_caches() {
        let Some(mut sound) = test_sound() else { return };
        play_instrument_sound("new_retard");
        sound.drain_instrument_queue();
        assert!(!sound.pending.is_empty(), "new_retard should have queued a decode");
        let deadline = Instant::now() + Duration::from_secs(20);
        loop {
            sound.process_pending();
            if sound.pending.is_empty() {
                break;
            }
            assert!(Instant::now() < deadline, "decode did not finish in time");
            std::thread::sleep(Duration::from_millis(10));
        }
        let state = sound.state.as_ref().unwrap();
        assert!(!state.cache.lock().unwrap().entries.is_empty(), "no media decoded for new_retard");
    }
}
