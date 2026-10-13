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

pub mod anim_triggers;
pub mod rtpc;
pub mod triggers;
pub mod voice_pack;
pub mod voices;
pub mod vorbis;
pub mod wwise;

use std::collections::{HashMap, HashSet, VecDeque};
use std::path::{Path, PathBuf};
use std::sync::mpsc::{self, Receiver, Sender, TryRecvError};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::xp;
use crate::Vars;
use anim_triggers::AnimTrigger;
use triggers::{Action, SoundXml, TriggerState};
use vorbis::Pcm;
use wwise::{ActionKind, ContainerKind, Package, PlayNode, RtpcCurve};

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
    if crate::sound_pack::installed() {
        crate::sound_pack::request(name);
        if crate::sound_pack::replaces(name) {
            return;
        }
    }
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
    let xml_path = sound_dir.join("sound.xml");
    let xml_text = std::fs::read_to_string(&xml_path).map_err(|e| format!("{}: {e}", xml_path.display()))?;
    let sound_xml = triggers::parse(&xml_text)?;
    let main_package = wwise::sound_xml_main_package(&xml_text).unwrap_or("Asobo_A320_NEO").to_owned();
    let package = Package::load_dir(sound_dir)?;
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

fn find_sound_dir() -> Option<PathBuf> {
    #[cfg(feature = "js")]
    if let Some(dir) = crate::js_bridge::plugin_dir().map(|d| d.join("sound")).filter(|d| d.join("sound.xml").is_file() && has_pck(d)) {
        return Some(dir);
    }
    None
}

fn has_pck(dir: &Path) -> bool {
    std::fs::read_dir(dir)
        .map(|entries| entries.flatten().any(|e| e.path().extension().is_some_and(|x| x.eq_ignore_ascii_case("pck"))))
        .unwrap_or(false)
}

/// `sound_triggers.txt`, next to `cockpit_variables.txt` in the plugin's own
/// directory (not the MSFS package): the converter's pairing of each
/// `fbw/cockpit/<anim>` click-position dataref with the Wwise event(s) it
/// should fire (see the `anim_triggers` module doc). Missing or unreadable
/// is not an error: an older converter output, or `--asobo-behaviours` not
/// given at convert time, simply means no click sounds — the same
/// degrade-gracefully rule `Sound::new`'s missing-package case already
/// follows for every other sound.
fn click_fallback(name: &str) -> Option<&'static str> {
    let n = name.trim().to_ascii_lowercase();
    if n.ends_with("button_on") {
        Some("pb1on")
    } else if n.ends_with("button_off") {
        Some("pb1off")
    } else if n.ends_with("knob") {
        Some("turnknob")
    } else if n.contains("switch") {
        Some("lswitch")
    } else if n.contains("lever") || n.ends_with("lock_on") || n.ends_with("lock_off") {
        Some("pulloutknob")
    } else {
        None
    }
}

fn parse_sound_nodes(text: &str) -> HashMap<String, ([f32; 3], bool)> {
    text.lines()
        .filter_map(|l| {
            let f: Vec<&str> = l.split('\t').collect();
            if f.len() < 5 {
                return None;
            }
            let n = |i: usize| f[i].trim().parse::<f32>().ok();
            Some((f[0].trim().to_ascii_uppercase(), ([n(1)?, n(2)?, n(3)?], f[4].trim() == "1")))
        })
        .collect()
}

fn load_sound_nodes() -> HashMap<String, ([f32; 3], bool)> {
    #[cfg(feature = "js")]
    let path = crate::js_bridge::plugin_dir().map(|d| d.join("sound_nodes.txt"));
    #[cfg(not(feature = "js"))]
    let path: Option<PathBuf> = None;
    path.and_then(|p| std::fs::read_to_string(p).ok()).map(|t| parse_sound_nodes(&t)).unwrap_or_default()
}

fn load_anim_triggers() -> Vec<AnimTrigger> {
    #[cfg(feature = "js")]
    let path = crate::js_bridge::plugin_dir().map(|d| d.join("sound_triggers.txt"));
    #[cfg(not(feature = "js"))]
    let path: Option<PathBuf> = None;
    path.and_then(|p| std::fs::read_to_string(p).ok()).map(|t| anim_triggers::parse(&t)).unwrap_or_default()
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
const CACHE_BUDGET_BYTES: usize = 256 * 1024 * 1024;

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
pub(crate) struct FlatSound {
    pub media_id: u32,
    pub looped: bool,
    pub volume_db: f32,
    pub curves: Vec<RtpcCurve>,
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
    flatten_with(node, gain_db, &[], rng, force_loop, out);
}

pub(crate) fn flatten_with(node: &PlayNode, gain_db: f32, curves: &[RtpcCurve], rng: &mut u64, force_loop: Option<bool>, out: &mut Vec<FlatSound>) {
    match node {
        PlayNode::Sound(s) => out.push(FlatSound {
            media_id: s.media_id,
            looped: force_loop.unwrap_or(s.loop_count == 0),
            volume_db: gain_db + s.volume_db + s.make_up_gain_db,
            curves: curves.iter().chain(&s.rtpcs).cloned().collect(),
        }),
        PlayNode::Container(c) => {
            let gain_db = gain_db + c.volume_db;
            let mine: Vec<RtpcCurve> = curves.iter().chain(&c.rtpcs).cloned().collect();
            match c.kind {
                ContainerKind::Layer => {
                    for child in &c.children {
                        flatten_with(child, gain_db, &mine, rng, force_loop, out);
                    }
                }
                ContainerKind::Random => {
                    if let Some(child) = weighted_pick(&c.children, &c.weights, rng) {
                        flatten_with(child, gain_db, &mine, rng, force_loop, out);
                    }
                }
                ContainerKind::Sequence => {
                    if let Some(child) = c.children.first() {
                        flatten_with(child, gain_db, &mine, rng, force_loop, out);
                    }
                }
            }
        }
        PlayNode::Missing(_) | PlayNode::Unsupported { .. } => {}
    }
}

fn curve_params(node: &PlayNode, out: &mut std::collections::BTreeSet<u32>) {
    match node {
        PlayNode::Sound(s) => out.extend(s.rtpcs.iter().map(|c| c.game_param)),
        PlayNode::Container(c) => {
            out.extend(c.rtpcs.iter().map(|r| r.game_param));
            for ch in &c.children {
                curve_params(ch, out);
            }
        }
        _ => {}
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
    outside_only: bool,
    since: Instant,
    voice: Option<voices::Spec>,
}

/// Which side of the cabin wall the camera is on, and every channel playing,
/// so a change of view can silence or restore them.
///
/// X-Plane's interior bus does not follow the camera outside, so every
/// flight-deck sound played on at full level from the ramp (the V1 and
/// radio-altitude callouts heard from the outside view, 30 Sep). MSFS places
/// them in the flight deck (sound.xml `NodeName`), where the fuselage cuts
/// them off; with no 3D position to give `XPLMPlayPCMOnBus`, a flight-deck
/// sound is silent outside here, and one sound.xml marks
/// `ViewPoint="Outside"` (the APU's exhaust) is silent inside, as MSFS plays
/// those.
#[derive(Default)]
struct ViewMix {
    /// `CAMERA STATE`: lib.rs feeds it from `sim/graphics/view/
    /// view_is_external`, 3 outside and 2 in the flight deck.
    camera: Option<VariableIdentifier>,
    outside: bool,
    live: Vec<Live>,
}

/// A channel this plugin started.
struct Live {
    channel: xp::FmodChannel,
    /// Its own level, before the view silences it.
    gain: f32,
    outside_only: bool,
    /// When a one-shot has played out; `None` for a loop.
    ends: Option<Instant>,
}

/// A sound's level for where the camera is: its own, or silent.
fn audible(outside_only: bool, outside: bool) -> f32 {
    if outside_only == outside {
        1.0
    } else {
        0.0
    }
}

const HANDOFF_MARGIN_S: f64 = 0.5;

enum LoopPlan {
    Whole,
    Body(Pcm),
    IntroThenBody { intro: Pcm, body: Pcm, handoff_s: f64 },
}

fn loop_plan(pcm: &Pcm) -> LoopPlan {
    let ch = pcm.channels.max(1) as usize;
    let Some((s, e)) = pcm.loop_frames else { return LoopPlan::Whole };
    let (s, e) = (s as usize, (e as usize).min(pcm.frames()));
    if e <= s + 1 {
        return LoopPlan::Whole;
    }
    let len = e - s;
    let sr = pcm.sample_rate.max(1) as usize;
    let make = |samples: Vec<i16>| Pcm { sample_rate: pcm.sample_rate, channels: pcm.channels, samples, loop_frames: None };
    let body_frame = |k: usize| {
        let f = s + k % len;
        &pcm.samples[f * ch..(f + 1) * ch]
    };
    if s * 100 < sr {
        return LoopPlan::Body(make(pcm.samples[s * ch..e * ch].to_vec()));
    }
    let margin = (HANDOFF_MARGIN_S * sr as f64) as usize;
    let mut intro = Vec::with_capacity((s + margin) * ch);
    intro.extend_from_slice(&pcm.samples[..s * ch]);
    for k in 0..margin {
        intro.extend_from_slice(body_frame(k));
    }
    let offset = (len - s % len) % len;
    let mut body = Vec::with_capacity(len * ch);
    for k in 0..len {
        body.extend_from_slice(body_frame(k + offset));
    }
    LoopPlan::IntroThenBody { intro: make(intro), body: make(body), handoff_s: s as f64 / sr as f64 }
}

struct Handoff {
    at: Instant,
    intro: xp::FmodChannel,
    body: xp::FmodChannel,
    gain: f32,
    outside_only: bool,
    trigger: Option<usize>,
}

impl ViewMix {
    /// Reads the camera, forgets channels that have played out, and
    /// re-levels the rest when the camera has crossed the cabin wall.
    fn follow(&mut self, vars: &mut Vars) {
        let id = *self.camera.get_or_insert_with(|| vars.get("CAMERA STATE".to_owned()));
        let outside = vars.read(&id) > 2.5;
        let now = Instant::now();
        self.live.retain(|l| l.ends.map_or(true, |e| e > now));
        if outside != self.outside {
            self.outside = outside;
            for l in &self.live {
                xp::set_audio_volume(l.channel, l.gain * audible(l.outside_only, outside));
            }
        }
    }
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
    /// Cockpit switch/button click sounds: `sound_triggers.txt` (the
    /// converter's pairing of a `fbw/cockpit/<anim>` dataref with the Wwise
    /// event it should fire — see the `anim_triggers` module doc). Read once
    /// at startup from the plugin's own directory, not the MSFS package, so
    /// it is available even before (or without) the sound package loading.
    anim_triggers: Vec<AnimTrigger>,
    anim_trigger_states: anim_triggers::States,
    /// Every event fired since `fired_since`, with what fired it, logged in
    /// one line every 10 s: a sound that keeps repeating (a trigger variable
    /// chattering across its bound) names itself with a count.
    fired: std::collections::BTreeMap<String, u32>,
    fired_since: Option<std::time::Instant>,
    view: ViewMix,
    handoffs: Vec<Handoff>,
    voices: Vec<voices::Voice>,
    inputs: Option<rtpc::Inputs>,
    pending_stops: HashMap<usize, Instant>,
    last_voice_tick: Option<Instant>,
    door_ids: Vec<VariableIdentifier>,
    xp_muted: Vec<(xp::DataRef, f32)>,
    xplm: Option<&'static xp::Xplm>,
    sound_nodes: HashMap<String, ([f32; 3], bool)>,
    pose_refs: Option<[Option<xp::DataRef>; 9]>,
    plans: HashMap<u32, Arc<LoopPlan>>,
    stats_since: Option<Instant>,
    stats_frames: u32,
    voice_pack: voice_pack::VoicePack,
}

fn volume_to_restore(was: f32) -> f32 {
    if was > 0. {
        was
    } else {
        1.
    }
}

impl Sound {
    /// Starts loading the user's FlyByWire A380X package in the background.
    /// Nothing here touches XPLM or the systems' variables, so it needs
    /// neither `Vars` nor `Xplm`.
    pub fn new() -> Self {
        let load_rx = find_sound_dir().map(spawn_loader);
        if load_rx.is_none() {
            crate::log("sound: no sound packages in the plugin's sound folder; cockpit sounds are off");
        }
        let seed = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_nanos() as u64).unwrap_or(0x9E37_79B9);
        let mut anim_triggers = load_anim_triggers();
        if crate::sound_pack::installed() {
            anim_triggers.retain(|t| !crate::sound_pack::replaces(&t.event));
        }
        crate::log(&format!("sound: {} cockpit click/switch sound triggers (sound_triggers.txt)", anim_triggers.len()));
        let anim_trigger_states = anim_triggers::fresh_states(&anim_triggers);
        let sound_nodes = load_sound_nodes();
        let mut sound = Self { load_rx, state: None, pending: Vec::new(), rng_state: seed | 1, logged_no_audio: false, anim_triggers, anim_trigger_states, fired: std::collections::BTreeMap::new(), fired_since: None, view: ViewMix::default(), handoffs: Vec::new(), voices: Vec::new(), inputs: None, pending_stops: HashMap::new(), last_voice_tick: None, door_ids: Vec::new(), xp_muted: Vec::new(), xplm: None, sound_nodes: HashMap::new(), pose_refs: None, plans: HashMap::new(), stats_since: None, stats_frames: 0, voice_pack: voice_pack::VoicePack::default() };
        sound.sound_nodes = sound_nodes;
        if let Some(dir) = crate::sound_pack::plugin_dir().map(|d| d.join("sound").join("voices")) {
            sound.voice_pack = voice_pack::VoicePack::load(&dir);
            if sound.voice_pack.len() > 0 {
                crate::log(&format!("sound: voice pack: {} of FlyByWire's callouts and alerts play recordings from {}", sound.voice_pack.len(), dir.display()));
            }
        }
        sound
    }

    /// One tick: pick up the loaded package once it's ready, drain
    /// instrument sound requests, step `sound.xml`'s triggers, and retry any
    /// sound still waiting on its decode. Main thread only: this is the only
    /// place that calls XPLM's audio functions.
    pub fn update(&mut self, vars: &mut Vars) {
        self.view.follow(vars);
        self.poll_load();
        self.drain_instrument_queue();
        self.evaluate_triggers(vars);
        crate::perf::lap("tick-after-systems: sound clicks");
        self.evaluate_animation_triggers(vars);
        crate::perf::lap("tick-after-systems: sound starts");
        self.process_pending();
        self.process_handoffs();
        crate::perf::lap("tick-after-systems: sound voices");
        self.update_voices(vars);
        self.log_stats();
    }

    fn log_stats(&mut self) {
        use std::sync::atomic::Ordering;
        self.stats_frames += 1;
        let now = Instant::now();
        let since = *self.stats_since.get_or_insert(now);
        if now.duration_since(since) < Duration::from_secs(10) {
            return;
        }
        let frames = self.stats_frames.max(1) as f64;
        let calls = xp::AUDIO_CALLS.swap(0, Ordering::Relaxed) as f64 / frames;
        let restarts = voices::RESTARTS.swap(0, Ordering::Relaxed);
        let failures = voices::RESTART_FAILURES.swap(0, Ordering::Relaxed);
        let audible = self.voices.iter().filter(|v| v.audible()).count();
        let mut loudest: Vec<(f32, String)> = self.voices.iter().filter_map(|v| v.loudness()).collect();
        loudest.sort_by(|a, b| b.0.total_cmp(&a.0));
        let loudest: Vec<String> = loudest.into_iter().take(6).map(|(_, d)| d).collect();
        crate::log(&format!(
            "sound: {} voices ({audible} audible), {calls:.0} X-Plane audio calls a frame, {restarts} restarts ({failures} failed) and {} pending in the last 10 s; loudest: {}",
            self.voices.len(),
            self.pending.len(),
            loudest.join(", ")
        ));
        self.stats_since = Some(now);
        self.stats_frames = 0;
    }

    fn plan_for(&mut self, media_id: u32, pcm: &Pcm, looped: bool) -> Arc<LoopPlan> {
        if !looped {
            return Arc::new(LoopPlan::Whole);
        }
        self.plans.entry(media_id).or_insert_with(|| Arc::new(loop_plan(pcm))).clone()
    }

    fn ensure_inputs(&mut self, vars: &mut Vars) {
        if self.inputs.is_some() {
            return;
        }
        let Some(state) = &self.state else { return };
        let known = rtpc::known_inputs(&state.sound_xml.triggers);
        let mut lists = Vec::with_capacity(state.sound_xml.triggers.len());
        let mut engine_sound = false;
        for t in &state.sound_xml.triggers {
            let mut used = std::collections::BTreeSet::new();
            if let Some(pb) = state.package.resolve_event(wwise::msfs_event_id(&state.main_package, &t.event)) {
                for a in &pb.actions {
                    used.extend(state.package.ancestor_rtpcs(a.target).iter().map(|c| c.game_param));
                    if let ActionKind::Play { node, .. } = &a.kind {
                        curve_params(node, &mut used);
                    }
                }
                engine_sound |= t.event.starts_with("Engine_");
            }
            let mut list = t.rtpcs.clone();
            list.extend(rtpc::inferred(t, &used, &known));
            lists.push(list);
        }
        let inputs = rtpc::Inputs::new(vars, lists);
        if !inputs.unfed.is_empty() {
            crate::log(&format!("sound: RTPC inputs with no source here: {}", inputs.unfed.join(", ")));
        }
        self.inputs = Some(inputs);
        self.door_ids = (0..18).map(|i| vars.get(format!("INTERACTIVE POINT OPEN:{i}"))).collect();
        let fans = !crate::sound_pack::installed() && state.package.resolve_event(wwise::msfs_event_id(&state.main_package, "AEVC380")).is_some();
        if engine_sound {
            self.mute_xplane("sim/operation/sound/engine_volume_ratio", vars);
        }
        if fans {
            self.mute_xplane("sim/operation/sound/fan_volume_ratio", vars);
        }
    }

    fn mute_xplane(&mut self, path: &str, vars: &mut Vars) {
        let Some(d) = vars.xplm.find(path) else { return };
        if self.xp_muted.iter().any(|(m, _)| *m == d) {
            return;
        }
        self.xplm = Some(vars.xplm);
        let was = vars.xplm.get_f(d);
        vars.xplm.set_f(d, 0.);
        let restore = volume_to_restore(was);
        self.xp_muted.push((d, restore));
        crate::log(&format!("sound: {path} {was:.2} -> 0 while FlyByWire's own sounds play ({restore:.2} when the aircraft unloads)"));
    }

    fn update_voices(&mut self, vars: &mut Vars) {
        if self.state.is_none() {
            return;
        }
        self.ensure_inputs(vars);
        let now = Instant::now();
        let dt = self.last_voice_tick.map_or(0., |t| now.duration_since(t).as_secs_f64());
        self.last_voice_tick = Some(now);
        let due: Vec<usize> = self.pending_stops.iter().filter(|(_, &at)| now >= at).map(|(&i, _)| i).collect();
        for i in due {
            self.pending_stops.remove(&i);
            self.stop_trigger(i);
        }
        self.voices.retain(|v| !v.done);
        if self.voices.is_empty() {
            return;
        }
        let door_open = self.door_ids.iter().any(|id| vars.read(id) > 0.1);
        let mix = voices::Mix { outside: self.view.outside, door_open };
        let mut params: HashMap<usize, HashMap<u32, f32>> = HashMap::new();
        if let Some(inputs) = self.inputs.as_mut() {
            let mut triggers: Vec<usize> = self.voices.iter().map(|v| v.spec.trigger).collect();
            triggers.sort_unstable();
            triggers.dedup();
            for t in triggers {
                params.insert(t, inputs.values(t, vars, dt));
            }
        }
        let pose = self.pose(vars);
        let empty = HashMap::new();
        for v in &mut self.voices {
            v.update(params.get(&v.spec.trigger).unwrap_or(&empty), &mix, pose.as_ref(), now);
        }
    }

    fn pose(&mut self, vars: &mut Vars) -> Option<voices::Pose> {
        let x = vars.xplm;
        let refs = self.pose_refs.get_or_insert_with(|| {
            [
                "sim/flightmodel/position/local_x",
                "sim/flightmodel/position/local_y",
                "sim/flightmodel/position/local_z",
                "sim/flightmodel/position/local_vx",
                "sim/flightmodel/position/local_vy",
                "sim/flightmodel/position/local_vz",
                "sim/flightmodel/position/psi",
                "sim/flightmodel/position/theta",
                "sim/flightmodel/position/phi",
            ]
            .map(|n| x.find(n))
        });
        let r = *refs;
        let d = |i: usize| r[i].map(|d| x.get_d(d));
        let f = |i: usize| r[i].map(|d| x.get_f(d));
        Some(voices::Pose {
            origin: [d(0)?, d(1)?, d(2)?],
            velocity: [f(3)?, f(4)?, f(5)?],
            psi: f(6)? as f64,
            theta: f(7)? as f64,
            phi: f(8)? as f64,
        })
    }

    fn process_handoffs(&mut self) {
        let now = Instant::now();
        let outside = self.view.outside;
        let mut i = 0;
        while i < self.handoffs.len() {
            if self.handoffs[i].at > now {
                i += 1;
                continue;
            }
            let h = self.handoffs.remove(i);
            xp::set_audio_volume(h.body, h.gain * audible(h.outside_only, outside));
            xp::stop_audio(h.intro);
            self.view.live.retain(|l| l.channel != h.intro);
            self.view.live.push(Live { channel: h.body, gain: h.gain, outside_only: h.outside_only, ends: None });
            if let (Some(t), Some(state)) = (h.trigger, self.state.as_mut()) {
                if let Some(channels) = state.active.get_mut(&t) {
                    channels.retain(|c| *c != h.intro);
                }
            }
        }
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
        for v in &mut self.voices {
            v.release();
        }
        self.voices.clear();
        self.pending_stops.clear();
        if let Some(x) = self.xplm {
            for (d, was) in self.xp_muted.drain(..) {
                x.set_f(d, was);
            }
        }
        for h in self.handoffs.drain(..) {
            xp::stop_audio(h.intro);
            xp::stop_audio(h.body);
        }
        self.pending.clear();
        self.view.live.clear();
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
    fn install(&mut self, mut loaded: Loaded) {
        if crate::sound_pack::installed() {
            let before = loaded.sound_xml.triggers.len();
            loaded.sound_xml.triggers.retain(|t| !crate::sound_pack::replaces(&t.event));
            crate::log(&format!("sound: {} sound.xml triggers left to the FMOD sound pack", before - loaded.sound_xml.triggers.len()));
        }
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
            "sound: {} banks, {} sound.xml triggers ({} with RTPCs), {} avionic sounds, {} animation sounds ({} wired from sound_triggers.txt), main package {}",
            loaded.package.banks.len(),
            loaded.sound_xml.triggers.len(),
            loaded.sound_xml.triggers.iter().filter(|t| !t.rtpcs.is_empty()).count(),
            loaded.sound_xml.avionic_events.len(),
            loaded.sound_xml.animation_events.len(),
            self.anim_triggers.len(),
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
            self.fire_event(&name, None, None, false);
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
                actions.push((i, action, state.sound_xml.triggers[i].event.clone(), state.sound_xml.triggers[i].outside));
            }
        }
        for (i, action, event, outside) in actions {
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
                Action::PlayOnce => {
                    *self.fired.entry(format!("{event} (sound.xml)")).or_insert(0) += 1;
                    self.fire_event(&event, Some(false), Some(i), outside)
                }
                Action::StartLoop => {
                    if self.pending_stops.remove(&i).is_some() {
                        continue;
                    }
                    *self.fired.entry(format!("{event} (sound.xml loop start)")).or_insert(0) += 1;
                    self.fire_event(&event, None, Some(i), outside)
                }
                Action::StopLoop => {
                    let delay = self.state.as_ref().map_or(0., |s| s.sound_xml.triggers[i].stop_delay_s);
                    if delay > 0. {
                        self.pending_stops.insert(i, Instant::now() + Duration::from_secs_f64(delay));
                    } else {
                        self.stop_trigger(i);
                    }
                }
                Action::None => {}
            }
        }
    }

    /// Cockpit switch/button clicks: each `fbw/cockpit/<anim>` dataref
    /// crossing the point in its travel `sound_triggers.txt` recorded fires
    /// that click's Wwise event once, exactly like MSFS's own compiled
    /// `<AnimationTriggers>` (see the `anim_triggers` module doc). Always
    /// one-shot, never looping — MSFS's `<EventTrigger>` has no
    /// continuous/stop half the way a `sound.xml` `SimVarSounds`
    /// `Continuous="true"` entry does.
    ///
    /// Real bug (W177's review of this): steps nothing until `self.state`
    /// exists, the same rule [`Self::evaluate_triggers`] already follows
    /// for `sound.xml`'s own triggers just below (`let Some(state) =
    /// &mut self.state else { return };`). `fire_event` itself silently
    /// no-ops with no package loaded (`let Some(state) = &self.state else
    /// { return };`), so without this guard a crossing evaluated during
    /// the async load window would still advance
    /// [`anim_triggers::AnimTriggerState`]'s own `last` reading —
    /// permanently marking that specific click "already handled" — while
    /// its sound never actually played, with no way to fire it once
    /// loading finishes. Holding off means the first evaluation after load
    /// sees the dataref's then-current value as its very first reading —
    /// the same "must not fire on the first reading" rule `step` already
    /// documents for cold spawn — instead of a falsely-consumed crossing.
    fn evaluate_animation_triggers(&mut self, vars: &mut Vars) {
        let pack = crate::sound_pack::installed();
        if self.state.is_none() && !pack {
            return;
        }
        let mut fired = Vec::new();
        for i in 0..self.anim_triggers.len() {
            let value = read_named(vars, &self.anim_triggers[i].dataref);
            if self.anim_trigger_states[i].step(&self.anim_triggers[i], value) {
                fired.push((self.anim_triggers[i].event.clone(), self.anim_triggers[i].dataref.clone()));
            }
        }
        for (event, dataref) in fired {
            *self.fired.entry(format!("{event} <- {dataref}")).or_insert(0) += 1;
            if pack && crate::sound_pack::click_sound(click_fallback(&event).unwrap_or(&event)).is_some() {
                crate::sound_pack::request(click_fallback(&event).unwrap_or(&event));
                continue;
            }
            self.fire_event(&event, Some(false), None, false);
        }
        self.log_fired();
    }

    fn log_fired(&mut self) {
        let now = std::time::Instant::now();
        let since = *self.fired_since.get_or_insert(now);
        if now.duration_since(since).as_secs_f64() < 10. {
            return;
        }
        if !self.fired.is_empty() {
            let mut list: Vec<(&String, &u32)> = self.fired.iter().collect();
            list.sort_by(|a, b| b.1.cmp(a.1));
            let text: Vec<String> = list.iter().take(12).map(|(k, n)| format!("{k} x{n}")).collect();
            crate::log(&format!("sound: fired in the last 10 s: {}", text.join(", ")));
        }
        self.fired.clear();
        self.fired_since = Some(now);
    }

    /// Resolves a `sound.xml` `WwiseEvent` name to the bank event MSFS would
    /// post for it, and plays (or queues) every leaf sound its Play actions
    /// reach.
    fn fire_event(&mut self, event_name: &str, force_loop: Option<bool>, trigger: Option<usize>, outside_only: bool) {
        let Some(state) = &self.state else { return };
        let mut leaves = Vec::new();
        if let Some(media_id) = self.voice_pack.media_for(event_name) {
            leaves.push(FlatSound { media_id, looped: force_loop.unwrap_or(false), volume_db: 0., curves: Vec::new() });
        } else {
            let event_id = wwise::msfs_event_id(&state.main_package, event_name);
            let resolved = state.package.resolve_event(event_id).or_else(|| {
                click_fallback(event_name).and_then(|f| state.package.resolve_event(wwise::msfs_event_id(&state.main_package, f)))
            });
            let Some(playback) = resolved else { return };
            let mut rng = self.rng_state;
            for action in &playback.actions {
                if let ActionKind::Play { node, parent_volume_db } = &action.kind {
                    let inherited = if trigger.is_some() { state.package.ancestor_rtpcs(action.target) } else { Vec::new() };
                    flatten_with(node, *parent_volume_db, &inherited, &mut rng, force_loop, &mut leaves);
                }
            }
            self.rng_state = rng;
        }
        let placement = trigger.and_then(|i| state.sound_xml.triggers.get(i)).map(|t| {
            let view = if t.outside {
                voices::View::OutsideOnly
            } else if t.inside_only {
                voices::View::InsideOnly
            } else {
                voices::View::Both
            };
            let node = self.sound_nodes.get(&t.node.trim().to_ascii_uppercase());
            let exterior = node.map_or(t.exterior(), |n| n.1 || t.outside);
            let position = node.map(|n| n.0).or(exterior.then_some([0., 0., 0.]));
            let cone_aft = t.cone_heading.is_some_and(|h| (h - 180.).abs() < 45.);
            (exterior, view, position, cone_aft)
        });
        for leaf in leaves {
            match (trigger, placement) {
                (Some(i), Some((exterior, view, position, cone_aft))) => {
                    let spec = voices::Spec { trigger: i, media_id: leaf.media_id, looped: leaf.looped, base_db: leaf.volume_db, curves: leaf.curves, exterior, view, position, cone_aft };
                    self.start_voice(spec);
                }
                _ => self.play_or_queue(leaf.media_id, leaf.looped, leaf.volume_db, trigger, outside_only),
            }
        }
    }

    fn start_voice(&mut self, spec: voices::Spec) {
        if let Some(pcm) = self.cache_get(spec.media_id) {
            let plan = self.plan_for(spec.media_id, &pcm, spec.looped);
            if let Some(v) = voices::Voice::start(spec, pcm, plan) {
                self.voices.push(v);
            }
            return;
        }
        self.request_decode(spec.media_id);
        self.pending.push(PendingLeaf { trigger: Some(spec.trigger), media_id: spec.media_id, looped: spec.looped, volume_db: spec.base_db, outside_only: false, since: Instant::now(), voice: Some(spec) });
    }

    fn stop_trigger(&mut self, trigger: usize) {
        self.pending.retain(|p| p.trigger != Some(trigger));
        self.handoffs.retain(|h| h.trigger != Some(trigger));
        let fade = self.state.as_ref().and_then(|s| s.sound_xml.triggers.get(trigger)).map_or(0., |t| t.fade_out_s);
        for v in self.voices.iter_mut().filter(|v| v.spec.trigger == trigger) {
            v.stop(fade);
        }
        if let Some(state) = self.state.as_mut() {
            if let Some(channels) = state.active.remove(&trigger) {
                for &channel in &channels {
                    xp::stop_audio(channel);
                }
                self.view.live.retain(|l| !channels.contains(&l.channel));
            }
        }
    }

    fn play_or_queue(&mut self, media_id: u32, looped: bool, volume_db: f32, trigger: Option<usize>, outside_only: bool) {
        if let Some(pcm) = self.cache_get(media_id) {
            self.play_pcm(&pcm, looped, volume_db, trigger, outside_only);
            return;
        }
        self.request_decode(media_id);
        self.pending.push(PendingLeaf { trigger, media_id, looped, volume_db, outside_only, since: Instant::now(), voice: None });
    }

    fn cache_get(&self, media_id: u32) -> Option<Arc<Pcm>> {
        if let Some(pcm) = self.voice_pack.pcm(media_id) {
            return Some(pcm);
        }
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
                match leaf.voice {
                    Some(spec) => {
                        let plan = self.plan_for(spec.media_id, &pcm, spec.looped);
                        if let Some(v) = voices::Voice::start(spec, pcm, plan) {
                            self.voices.push(v);
                        }
                    }
                    None => self.play_pcm(&pcm, leaf.looped, leaf.volume_db, leaf.trigger, leaf.outside_only),
                }
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
    fn play_pcm(&mut self, pcm: &Pcm, looped: bool, volume_db: f32, trigger: Option<usize>, outside_only: bool) {
        if !xp::has_pcm_audio() {
            if !self.logged_no_audio {
                self.logged_no_audio = true;
                crate::log("sound: no XPLMPlayPCMOnBus (needs X-Plane 12.04 or later); cockpit sounds are off");
            }
            return;
        }
        if !looped {
            self.start_channel(pcm, false, volume_db, trigger, outside_only, true);
            return;
        }
        match loop_plan(pcm) {
            LoopPlan::Whole => {
                self.start_channel(pcm, true, volume_db, trigger, outside_only, true);
            }
            LoopPlan::Body(body) => {
                self.start_channel(&body, true, volume_db, trigger, outside_only, true);
            }
            LoopPlan::IntroThenBody { intro, body, handoff_s } => {
                let Some(intro_channel) = self.start_channel(&intro, false, volume_db, trigger, outside_only, true) else { return };
                let Some(body_channel) = self.start_channel(&body, true, volume_db, trigger, outside_only, false) else { return };
                self.handoffs.push(Handoff {
                    at: Instant::now() + Duration::from_secs_f64(handoff_s + 0.05),
                    intro: intro_channel,
                    body: body_channel,
                    gain: 10f32.powf(volume_db / 20.0),
                    outside_only,
                    trigger,
                });
            }
        }
    }

    fn start_channel(&mut self, pcm: &Pcm, looped: bool, volume_db: f32, trigger: Option<usize>, outside_only: bool, audible_now: bool) -> Option<xp::FmodChannel> {
        let channel = xp::play_pcm16_on_bus(&pcm.samples, pcm.sample_rate, pcm.channels, looped, xp::AUDIO_INTERIOR, None, std::ptr::null_mut())?;
        let gain = 10f32.powf(volume_db / 20.0);
        if audible_now {
            xp::set_audio_volume(channel, gain * audible(outside_only, self.view.outside));
            let frames = pcm.samples.len() as f64 / f64::from(pcm.channels.max(1));
            let ends = (!looped).then(|| Instant::now() + Duration::from_secs_f64(frames / f64::from(pcm.sample_rate.max(1)) + 0.5));
            self.view.live.push(Live { channel, gain, outside_only, ends });
        } else {
            xp::set_audio_volume(channel, 0.0);
        }
        if let Some(trigger) = trigger {
            if let Some(state) = self.state.as_mut() {
                state.active.entry(trigger).or_default().push(channel);
            }
        }
        Some(channel)
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
    fn clicks_missing_from_the_banks_fall_back_to_flybywires_own() {
        assert_eq!(click_fallback("_push_button_on"), Some("pb1on"));
        assert_eq!(click_fallback("_push_button_off"), Some("pb1off"));
        assert_eq!(click_fallback("_knob"), Some("turnknob"));
        assert_eq!(click_fallback("button_switch_generic"), Some("lswitch"));
        assert_eq!(click_fallback("radio_switch_on"), Some("lswitch"));
        assert_eq!(click_fallback("new_retard"), None);
        let Some(pkg) = wwise::test_package() else { return };
        for f in ["pb1on", "pb1off", "turnknob", "lswitch", "pulloutknob"] {
            assert!(pkg.resolve_event(wwise::msfs_event_id("Asobo_A320_NEO", f)).is_some(), "{f} is in FlyByWire's own banks");
        }
    }

    #[test]
    fn sound_node_positions_are_read_from_the_converters_file() {
        let nodes = parse_sound_nodes("SOUND_ENGINE_1\t-25.695\t-0.609\t-4.500\t1\nspeaker_ohvd_lp\t-0.119\t3.673\t-31.808\t0\nbroken\t1\t2\n");
        assert_eq!(nodes.len(), 2);
        assert_eq!(nodes["SOUND_ENGINE_1"], ([-25.695, -0.609, -4.5], true));
        assert_eq!(nodes["SPEAKER_OHVD_LP"].1, false);
    }

    #[test]
    fn a_loop_plays_its_intro_once_then_only_its_loop_region_in_phase() {
        let frame = |i: usize| [(i * 7 % 1000) as i16, -((i * 3 % 1000) as i16)];
        let (s, e, total) = (30_000usize, 50_000usize, 52_000usize);
        let pcm = Pcm { sample_rate: 44100, channels: 2, samples: (0..total).flat_map(frame).collect(), loop_frames: Some((s as u32, e as u32)) };
        let LoopPlan::IntroThenBody { intro, body, handoff_s } = loop_plan(&pcm) else { panic!("an intro") };
        assert!((handoff_s - s as f64 / 44100.0).abs() < 1e-9);
        assert_eq!(body.frames(), e - s, "the tail after the loop end is not looped");
        let at = |p: &Pcm, i: usize| [p.samples[i * 2], p.samples[i * 2 + 1]];
        for i in 0..s {
            assert_eq!(at(&intro, i), frame(i));
        }
        let len = e - s;
        for t in s..intro.frames() {
            assert_eq!(at(&intro, t), at(&body, t % len), "frame {t}: intro and body must carry the same audio through the handoff");
            assert_eq!(at(&body, t % len), frame(s + (t - s) % len));
        }
        let looped_once = Pcm { loop_frames: Some((10, e as u32)), ..pcm.clone() };
        assert!(matches!(loop_plan(&looped_once), LoopPlan::Body(b) if b.frames() == e - 10));
        let whole = Pcm { loop_frames: None, ..pcm };
        assert!(matches!(loop_plan(&whole), LoopPlan::Whole));
    }

    #[test]
    fn a_continuous_trigger_keeps_each_sounds_own_loop_setting() {
        let Some(pkg) = wwise::test_package() else { return };
        let event_id = wwise::msfs_event_id("Asobo_A320_NEO", "bat1");
        let playback = pkg.resolve_event(event_id).expect("bat1");
        let mut rng = 1u64;
        let mut leaves = Vec::new();
        for action in &playback.actions {
            if let ActionKind::Play { node, parent_volume_db } = &action.kind {
                flatten(node, *parent_volume_db, &mut rng, None, &mut leaves);
            }
        }
        let looped = leaves.iter().filter(|l| l.looped).count();
        assert_eq!((leaves.len(), looped), (4, 2), "the two contactor clicks play once, the two hums loop");
    }

    /// Real bug (X-Plane 12 session, 30 Sep): the callouts carried on at
    /// full level from the outside view, since X-Plane's interior bus does
    /// not follow the camera out of the aircraft.
    #[test]
    fn flight_deck_sounds_are_silent_outside_and_the_apus_exhaust_inside() {
        assert_eq!(audible(false, false), 1.0, "a callout in the flight deck");
        assert_eq!(audible(false, true), 0.0, "a callout from outside");
        assert_eq!(audible(true, true), 1.0, "the APU's exhaust from outside");
        assert_eq!(audible(true, false), 0.0, "the APU's exhaust in the flight deck");
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
    fn a_volume_left_at_zero_by_an_earlier_session_comes_back_full() {
        assert_eq!(volume_to_restore(0.), 1.);
        assert_eq!(volume_to_restore(0.6), 0.6);
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
        let mut sound = Sound { load_rx: None, state: None, pending: Vec::new(), rng_state: 0xDEAD_BEEF_1234_5678, logged_no_audio: false, anim_triggers: Vec::new(), anim_trigger_states: Vec::new(), fired: std::collections::BTreeMap::new(), fired_since: None, view: ViewMix::default(), handoffs: Vec::new(), voices: Vec::new(), inputs: None, pending_stops: HashMap::new(), last_voice_tick: None, door_ids: Vec::new(), xp_muted: Vec::new(), xplm: None, sound_nodes: HashMap::new(), pose_refs: None, plans: HashMap::new(), stats_since: None, stats_frames: 0, voice_pack: voice_pack::VoicePack::default() };
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
            rtpcs: Vec::new(),
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

    /// The path W101 adds: a click position dataref crossing its trigger's
    /// `NormalizedTime` must reach `fire_event` and decode media, exactly
    /// like [`play_instrument_sound_resolves_decodes_and_caches`] already
    /// proves for the `Coherent.call` path. `evaluate_animation_triggers`
    /// itself needs a real `Vars` (unavailable in a unit test without XPLM);
    /// this exercises the same state-machine-then-fire_event seam directly.
    #[test]
    fn an_animation_trigger_crossing_its_threshold_resolves_decodes_and_caches() {
        let Some(mut sound) = test_sound() else { return };
        let trig = anim_triggers::AnimTrigger {
            dataref: "fbw/cockpit/TEST_SWITCH".into(),
            direction: anim_triggers::Direction::Forward,
            normalized_time: 0.1,
            event: "new_retard".into(),
        };
        sound.anim_triggers = vec![trig];
        sound.anim_trigger_states = anim_triggers::fresh_states(&sound.anim_triggers);

        assert!(!sound.anim_trigger_states[0].step(&sound.anim_triggers[0], 0.0), "spawn state must not fire");
        assert!(sound.anim_trigger_states[0].step(&sound.anim_triggers[0], 1.0), "0 -> 1 must cross 0.1 forward");
        sound.fire_event("new_retard", Some(false), None, false);
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
        assert!(!state.cache.lock().unwrap().entries.is_empty(), "no media decoded for the animation-triggered new_retard");
    }

    /// Real bug (W177's review of fixes/W101.md), fixed by the guard on
    /// `evaluate_animation_triggers` (W221): without the
    /// `self.state.is_none()` guard, a crossing evaluated while no package
    /// is loaded yet still advances the trigger's own state, permanently
    /// losing that specific click's sound (`fire_event` has nothing to
    /// resolve against either way, so nothing plays, but unlike before the
    /// fix, nothing *records* that it happened either). Proven without
    /// needing a loaded package at all: two ticks that cross the trigger's
    /// threshold while `state` is `None` must leave the state machine
    /// exactly as untouched as a fresh one -- checked by calling
    /// `AnimTriggerState::step` directly afterward and confirming it still
    /// reacts to the same 0 -> 1 move as if for the very first time.
    #[test]
    fn evaluate_animation_triggers_does_not_step_state_before_the_package_loads() {
        let xplm: &'static xp::Xplm = Box::leak(Box::new(xp::Xplm::dummy()));
        let mut vars = crate::Vars::new(xplm);
        let id = vars.register_named("fbw/cockpit/TEST_SWITCH");

        let trigger = anim_triggers::AnimTrigger {
            dataref: "fbw/cockpit/TEST_SWITCH".into(),
            direction: anim_triggers::Direction::Forward,
            normalized_time: 0.1,
            event: "new_retard".into(),
        };
        let states = anim_triggers::fresh_states(&[trigger.clone()]);
        let mut sound = Sound { load_rx: None, state: None, pending: Vec::new(), rng_state: 0xDEAD_BEEF_1234_5678, logged_no_audio: false, anim_triggers: vec![trigger], anim_trigger_states: states, fired: std::collections::BTreeMap::new(), fired_since: None, view: ViewMix::default(), handoffs: Vec::new(), voices: Vec::new(), inputs: None, pending_stops: HashMap::new(), last_voice_tick: None, door_ids: Vec::new(), xp_muted: Vec::new(), xplm: None, sound_nodes: HashMap::new(), pose_refs: None, plans: HashMap::new(), stats_since: None, stats_frames: 0, voice_pack: voice_pack::VoicePack::default() };

        // No package loaded (`state` stays `None` throughout). A real
        // click happens: the dataref crosses 0.1 moving up.
        vars.write(&id, 0.0);
        sound.evaluate_animation_triggers(&mut vars);
        vars.write(&id, 1.0);
        sound.evaluate_animation_triggers(&mut vars);
        assert!(sound.pending.is_empty(), "no package loaded: nothing could have been queued");

        // Without the guard, the second call above would have called
        // `step`, recording `last = Some(1.0)` -- this exact crossing
        // "used up" with no sound ever played for it. With the guard,
        // `last` is still `None` (never touched while `state` was `None`),
        // so this behaves exactly like a brand new trigger: a baseline
        // reading, then a real fire on the next crossing.
        assert!(!sound.anim_trigger_states[0].step(&sound.anim_triggers[0], 0.0), "baseline reading must not fire");
        assert!(sound.anim_trigger_states[0].step(&sound.anim_triggers[0], 1.0), "the state machine must not have already consumed this crossing while state was None");
    }
}
