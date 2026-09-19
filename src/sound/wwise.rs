//! FlyByWire's Wwise sound packages (`sound/*.PCK`) read: the soundbanks
//! inside, their embedded media, and the events that play them.
//!
//! Layouts are Wwise's for bank generator version 135 (Wwise 2019.2, what
//! MSFS 2020 packages carry) as wwiser documents them
//! (github.com/bnnm/wwiser, wwiser/parser/wparser.py; the function names
//! below are its `CAk...` names).
//!
//! - Package (Wwise's file package, `AkFilePackage`; the layout was read off
//!   the three FlyByWire files, whose header is 72 bytes): "AKPK", header
//!   size, version 1, then the sizes of the language map, the soundbank
//!   table, the streamed-file table and (when the sizes add up to it) the
//!   external-file table; each table a u32 count and 20-byte entries (id,
//!   block size, file size, start block, language).
//! - Bank: tagged sections. BKHD (version, bank id), DIDX (12-byte media
//!   index into DATA), DATA, HIRC (u32 count, then objects: u8 type, u32
//!   size, u32 id, body).
//!
//! HIRC objects honoured:
//! - 2 Sound (`CAkSound__SetInitialValues`): the media source (plugin id,
//!   stream type, source id) and node base parameters.
//! - 3 Action (`CAkAction__SetInitialValues`): target, properties, and for
//!   Play (0x04xx) the bank id, for Stop (0x01xx) the exception list.
//! - 4 Event: its action list.
//! - 5 Random/Sequence container (`CAkRanSeqCntr__SetInitialValues`), 9
//!   Layer container (`CAkLayerCntr__SetInitialValues`), 7 Actor-Mixer:
//!   node base parameters, children, and the playlist and play settings.
//!
//! Every other type (State 1, Bus 8, Attenuation 14, FxCustom 17, ...) is
//! kept only as its type. Node base parameters
//! (`CAkParameterNodeBase__SetNodeBaseParams`) are parsed in full to reach
//! what follows them; only the parent, output bus, property bundles and
//! RTPC targets are kept.

use std::collections::{BTreeMap, HashMap};
use std::path::Path;
use std::sync::Arc;

/// Wwise's ShortID for a name: 32-bit FNV-1 over the lower-cased name
/// (wwiser/parser/wdefs.py and Wwise's `AK::SoundEngine::GetIDFromString`).
pub fn short_id(name: &str) -> u32 {
    let mut h: u32 = 2_166_136_261;
    for b in name.bytes() {
        h = h.wrapping_mul(16_777_619);
        h ^= b.to_ascii_lowercase() as u32;
    }
    h
}

/// The Wwise event MSFS posts for a sound.xml `<Sound WwiseEvent="name">`:
/// the ShortID of `play_<MainPackage>_<name>`, MainPackage being sound.xml's
/// `<WwisePackages><MainPackage Name="...">`. No public document states
/// this; it is what FlyByWire's banks hold. Their bank ids are the ShortIDs
/// of the `AdditionalPackage` names (FBW_A320_NEO_1..3 =
/// 814486318..814486316), no event id is the ShortID of a bare sound.xml
/// name, and this form with `Asobo_A320_NEO` (sound.xml:9) gives an event id
/// for 206 of sound.xml's 365 names. The other 159 are MSFS's own cockpit
/// sounds (switches, knobs, rattles, the stock `aural_*ft` callouts), which
/// live in the sim's own Asobo_A320_NEO package, not in these files.
pub fn msfs_event_id(main_package: &str, wwise_event: &str) -> u32 {
    short_id(&format!("play_{main_package}_{wwise_event}"))
}

/// sound.xml's `<MainPackage Name="...">`.
pub fn sound_xml_main_package(sound_xml: &str) -> Option<&str> {
    let rest = &sound_xml[sound_xml.find("<MainPackage")?..];
    let rest = &rest[rest.find("Name=\"")? + 6..];
    rest.split('"').next()
}

/// HIRC object types for bank version 135 (wparser.py `get_hirc_dispatch`).
pub mod hirc {
    pub const STATE: u8 = 0x01;
    pub const SOUND: u8 = 0x02;
    pub const ACTION: u8 = 0x03;
    pub const EVENT: u8 = 0x04;
    pub const RAN_SEQ_CNTR: u8 = 0x05;
    pub const SWITCH_CNTR: u8 = 0x06;
    pub const ACTOR_MIXER: u8 = 0x07;
    pub const BUS: u8 = 0x08;
    pub const LAYER_CNTR: u8 = 0x09;
}

/// Property ids for bank versions 128-145 (wdefs.py `AkPropID_128`).
pub mod prop {
    pub const VOLUME: u8 = 0x00;
    pub const MAKE_UP_GAIN: u8 = 0x06;
    pub const DELAY_TIME: u8 = 0x0F;
    pub const TRANSITION_TIME: u8 = 0x10;
    pub const PROBABILITY: u8 = 0x11;
    pub const LOOP: u8 = 0x3A;
    pub const INITIAL_DELAY: u8 = 0x3B;
}

// ---------------------------------------------------------------------------
// Reader

struct Rd<'a> {
    b: &'a [u8],
    p: usize,
}

impl<'a> Rd<'a> {
    fn new(b: &'a [u8]) -> Self {
        Self { b, p: 0 }
    }

    fn take(&mut self, n: usize) -> Result<&'a [u8], String> {
        let s = self.b.get(self.p..self.p + n).ok_or_else(|| format!("truncated at {:#x} reading {n} bytes", self.p))?;
        self.p += n;
        Ok(s)
    }

    fn skip(&mut self, n: usize) -> Result<(), String> {
        self.take(n).map(|_| ())
    }

    fn u8(&mut self) -> Result<u8, String> {
        Ok(self.take(1)?[0])
    }

    fn u16(&mut self) -> Result<u16, String> {
        let s = self.take(2)?;
        Ok(u16::from_le_bytes([s[0], s[1]]))
    }

    fn u32(&mut self) -> Result<u32, String> {
        let s = self.take(4)?;
        Ok(u32::from_le_bytes([s[0], s[1], s[2], s[3]]))
    }

    fn f32(&mut self) -> Result<f32, String> {
        self.u32().map(f32::from_bits)
    }

    /// Wwise's variable-size integer: 7 bits per byte, most significant
    /// first, high bit set on all but the last (wmodel.py `TYPE_VAR`).
    fn var(&mut self) -> Result<u32, String> {
        let mut cur = self.u8()?;
        let mut v = (cur & 0x7F) as u32;
        let mut n = 0;
        while cur & 0x80 != 0 {
            if n >= 4 {
                return Err("variable-size integer too long".into());
            }
            cur = self.u8()?;
            v = (v << 7) | (cur & 0x7F) as u32;
            n += 1;
        }
        Ok(v)
    }

    fn rest(&self) -> usize {
        self.b.len() - self.p
    }
}

// ---------------------------------------------------------------------------
// HIRC objects

/// What `CAkParameterNodeBase__SetNodeBaseParams` carries that is kept.
#[derive(Clone, Debug, Default)]
pub struct NodeBase {
    pub override_bus_id: u32,
    pub parent_id: u32,
    /// `AkPropBundle<AkPropValue,unsigned char>`: property id and the raw
    /// 32-bit value (float or integer depending on the property).
    pub props: Vec<(u8, u32)>,
    /// `AkPropBundle<RANGED_MODIFIERS<AkPropValue>>`: id, min, max.
    pub ranged_props: Vec<(u8, u32, u32)>,
    /// RTPC curves: (game parameter id, parameter id).
    pub rtpcs: Vec<(u32, u32)>,
    /// `SetPositioningParams`' "3D position" bit, when this node overrides
    /// its parent's positioning at all (`has_positioning`): `Some(true)`
    /// places the sound in space (an engine, a switch on a panel) rather
    /// than flat as MSFS's 2D sounds play; `None` inherits the parent's,
    /// resolved by [`Package::positional`].
    pub positional_override: Option<bool>,
}

impl NodeBase {
    pub fn prop(&self, id: u8) -> Option<u32> {
        self.props.iter().find(|p| p.0 == id).map(|p| p.1)
    }

    /// The Volume property in dB, 0 when absent.
    pub fn volume_db(&self) -> f32 {
        self.prop(prop::VOLUME).map(f32::from_bits).unwrap_or(0.0)
    }
}

#[derive(Clone, Debug)]
pub struct SoundObject {
    pub base: NodeBase,
    /// Codec plugin, e.g. 0x00040001 Vorbis (wdefs.py `AkPluginType_id`).
    pub plugin_id: u32,
    /// 0 embedded in a bank, 1 prefetch streaming, 2 streaming
    /// (wdefs.py `AkBank__AKBKSourceType_112`).
    pub stream_type: u8,
    pub source_id: u32,
    pub media_size: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ContainerKind {
    Random,
    Sequence,
    Layer,
}

#[derive(Clone, Debug)]
pub struct ContainerObject {
    pub kind: ContainerKind,
    pub base: NodeBase,
    pub children: Vec<u32>,
    /// Random/sequence playlist: (child id, weight).
    pub playlist: Vec<(u32, i32)>,
    /// `sLoopCount`: 1 plays once, 0 loops forever.
    pub loop_count: u16,
    pub transition_time_s: f32,
    /// `AkTransitionMode`: 0 disabled, 1 cross-fade amp, 2 cross-fade power,
    /// 3 delay, 4 sample accurate, 5 trigger rate.
    pub transition_mode: u8,
    /// `AkRandomMode`: 0 normal, 1 shuffle.
    pub random_mode: u8,
    pub avoid_repeat_count: u16,
    /// Continuous plays the whole list; step plays one item per play.
    pub continuous: bool,
    pub reset_playlist_each_play: bool,
}

#[derive(Clone, Debug)]
pub struct ActionObject {
    /// `AkActionType_062`: high byte the action, low byte the scope.
    pub action_type: u16,
    pub target: u32,
    pub is_bus: bool,
    pub props: Vec<(u8, u32)>,
    /// Play: the bank holding the target.
    pub bank_id: Option<u32>,
    /// Stop: objects excepted.
    pub exceptions: Vec<u32>,
}

impl ActionObject {
    pub fn prop(&self, id: u8) -> Option<u32> {
        self.props.iter().find(|p| p.0 == id).map(|p| p.1)
    }
}

#[derive(Clone, Debug)]
pub enum HircObject {
    Sound(SoundObject),
    Action(ActionObject),
    Event(Vec<u32>),
    Container(ContainerObject),
    ActorMixer { base: NodeBase, children: Vec<u32> },
    Other { hirc_type: u8 },
}

impl HircObject {
    pub fn base(&self) -> Option<&NodeBase> {
        match self {
            HircObject::Sound(s) => Some(&s.base),
            HircObject::Container(c) => Some(&c.base),
            HircObject::ActorMixer { base, .. } => Some(base),
            _ => None,
        }
    }
}

fn prop_bundle(r: &mut Rd) -> Result<Vec<(u8, u32)>, String> {
    let n = r.u8()? as usize;
    let ids = r.take(n)?.to_vec();
    ids.into_iter().map(|id| Ok((id, r.u32()?))).collect()
}

fn ranged_prop_bundle(r: &mut Rd) -> Result<Vec<(u8, u32, u32)>, String> {
    let n = r.u8()? as usize;
    let ids = r.take(n)?.to_vec();
    ids.into_iter().map(|id| Ok((id, r.u32()?, r.u32()?))).collect()
}

/// `SetInitialRTPC_CAkParameterNodeBase_` for 90 <= version <= 141.
fn initial_rtpc(r: &mut Rd) -> Result<Vec<(u32, u32)>, String> {
    let n = r.u16()?;
    let mut out = Vec::with_capacity(n as usize);
    for _ in 0..n {
        let rtpc_id = r.u32()?;
        let _rtpc_type = r.u8()?;
        let _accum = r.u8()?;
        let param_id = r.var()?;
        let _curve_id = r.u32()?;
        let _scaling = r.u8()?;
        let points = r.u16()? as usize;
        r.skip(points * 12)?; // AkRTPCGraphPoint: from f32, to f32, interp u32
        out.push((rtpc_id, param_id));
    }
    Ok(out)
}

/// `CAkParameterNodeBase__SetNodeBaseParams` for version 135.
fn node_base(r: &mut Rd) -> Result<NodeBase, String> {
    // CAkParameterNode__SetInitialFxParams
    let _override_fx = r.u8()?;
    let num_fx = r.u8()? as usize;
    if num_fx > 0 {
        let _bypass = r.u8()?;
        r.skip(num_fx * 7)?; // index u8, fx id u32, is share set u8, is rendered u8
    }
    // (SetInitialMetadataParams only from version 137.)
    let _override_attachment = r.u8()?;
    let override_bus_id = r.u32()?;
    let parent_id = r.u32()?;
    let _priority_bits = r.u8()?;
    // CAkParameterNode__SetInitialParams
    let props = prop_bundle(r)?;
    let ranged_props = ranged_prop_bundle(r)?;

    // CAkParameterNodeBase__SetPositioningParams, version > 129
    let positioning = r.u8()?;
    let has_positioning = positioning & 1 != 0;
    let has_3d = (positioning >> 1) & 1 != 0;
    let positional_override = has_positioning.then_some(has_3d);
    if has_positioning && has_3d {
        let _bits_3d = r.u8()?;
        let position_type = (positioning >> 5) & 3;
        if position_type != 0 {
            let _path_mode = r.u8()?;
            let _transition_time = r.u32()?;
            let vertices = r.u32()? as usize;
            r.skip(vertices * 16)?;
            let items = r.u32()? as usize;
            r.skip(items * 8)?;
            r.skip(items * 12)?; // Ak3DAutomationParams: x, y, z ranges
        }
    }

    // CAkParameterNodeBase__SetAuxParams, version 135 (not the custom one)
    let aux = r.u8()?;
    if (aux >> 3) & 1 != 0 {
        r.skip(16)?;
    }
    let _reflections_aux_bus = r.u32()?;

    // CAkParameterNode__SetAdvSettingsParams, version > 89
    r.skip(6)?;

    // CAkStateAware__ReadStateChunk, version <= 145
    let state_props = r.var()?;
    for _ in 0..state_props {
        let _property = r.var()?;
        let _accum = r.u8()?;
        let _in_db = r.u8()?;
    }
    let state_groups = r.var()?;
    for _ in 0..state_groups {
        let _group = r.u32()?;
        let _sync = r.u8()?;
        let states = r.var()? as usize;
        r.skip(states * 8)?;
    }

    let rtpcs = initial_rtpc(r)?;
    Ok(NodeBase { override_bus_id, parent_id, props, ranged_props, rtpcs, positional_override })
}

/// `CAkParentNode_CAkParameterNode___SetChildren`
fn children(r: &mut Rd) -> Result<Vec<u32>, String> {
    let n = r.u32()?;
    (0..n).map(|_| r.u32()).collect()
}

fn parse_sound(r: &mut Rd) -> Result<SoundObject, String> {
    // CAkBankMgr__LoadSource, 112 < version <= 150
    let plugin_id = r.u32()?;
    let stream_type = r.u8()?;
    let source_id = r.u32()?;
    let media_size = r.u32()?;
    let _source_bits = r.u8()?;
    if plugin_id & 0x0F == 2 {
        let size = r.u32()? as usize;
        r.skip(size)?;
    }
    let base = node_base(r)?;
    Ok(SoundObject { base, plugin_id, stream_type, source_id, media_size })
}

fn parse_ran_seq(r: &mut Rd) -> Result<ContainerObject, String> {
    let base = node_base(r)?;
    let loop_count = r.u16()?;
    let _loop_mod_min = r.u16()?;
    let _loop_mod_max = r.u16()?;
    let transition_time_s = r.f32()?;
    let _transition_mod_min = r.f32()?;
    let _transition_mod_max = r.f32()?;
    let avoid_repeat_count = r.u16()?;
    let transition_mode = r.u8()?;
    let random_mode = r.u8()?;
    let mode = r.u8()?;
    let bits = r.u8()?;
    let children = children(r)?;
    let n = r.u16()?;
    let playlist = (0..n).map(|_| Ok((r.u32()?, r.u32()? as i32))).collect::<Result<Vec<_>, String>>()?;
    Ok(ContainerObject {
        kind: if mode == 1 { ContainerKind::Sequence } else { ContainerKind::Random },
        base,
        children,
        playlist,
        loop_count,
        transition_time_s,
        transition_mode,
        random_mode,
        avoid_repeat_count,
        continuous: (bits >> 3) & 1 != 0,
        reset_playlist_each_play: (bits >> 1) & 1 != 0,
    })
}

fn parse_layer(r: &mut Rd) -> Result<ContainerObject, String> {
    let base = node_base(r)?;
    let children = children(r)?;
    let layers = r.u32()?;
    for _ in 0..layers {
        let _layer_id = r.u32()?;
        // CAkLayer__SetInitialValues
        initial_rtpc(r)?;
        let _rtpc_id = r.u32()?;
        let _rtpc_type = r.u8()?;
        let assocs = r.u32()?;
        for _ in 0..assocs {
            let _child = r.u32()?;
            let points = r.u32()? as usize;
            r.skip(points * 12)?;
        }
    }
    let _continuous_validation = r.u8()?;
    Ok(ContainerObject {
        kind: ContainerKind::Layer,
        base,
        children,
        playlist: Vec::new(),
        loop_count: 1,
        transition_time_s: 0.0,
        transition_mode: 0,
        random_mode: 0,
        avoid_repeat_count: 0,
        continuous: true,
        reset_playlist_each_play: false,
    })
}

/// Returns the action and whether its layout was read to the end.
fn parse_action(r: &mut Rd) -> Result<(ActionObject, bool), String> {
    let action_type = r.u16()?;
    let target = r.u32()?;
    let is_bus = r.u8()? & 1 != 0;
    let props = prop_bundle(r)?;
    let _ranged = ranged_prop_bundle(r)?;
    let mut action = ActionObject { action_type, target, is_bus, props, bank_id: None, exceptions: Vec::new() };
    let complete = match action_type >> 8 {
        // CAkActionPlay__SetActionParams
        0x04 => {
            let _fade_curve = r.u8()?;
            action.bank_id = Some(r.u32()?);
            true
        }
        // CAkActionActive__SetActionParams with the Stop specific params and
        // CAkActionExcept__SetExceptParams
        0x01 => {
            let _fade_curve = r.u8()?;
            let _stop_bits = r.u8()?;
            let n = r.var()?;
            for _ in 0..n {
                action.exceptions.push(r.u32()?);
                let _is_bus = r.u8()?;
            }
            true
        }
        _ => false,
    };
    Ok((action, complete))
}

fn parse_object(hirc_type: u8, r: &mut Rd) -> Result<(HircObject, bool), String> {
    Ok(match hirc_type {
        hirc::SOUND => (HircObject::Sound(parse_sound(r)?), true),
        hirc::ACTION => {
            let (a, complete) = parse_action(r)?;
            (HircObject::Action(a), complete)
        }
        hirc::EVENT => {
            let n = r.var()?;
            (HircObject::Event((0..n).map(|_| r.u32()).collect::<Result<_, _>>()?), true)
        }
        hirc::RAN_SEQ_CNTR => (HircObject::Container(parse_ran_seq(r)?), true),
        hirc::LAYER_CNTR => (HircObject::Container(parse_layer(r)?), true),
        hirc::ACTOR_MIXER => {
            let base = node_base(r)?;
            (HircObject::ActorMixer { base, children: children(r)? }, true)
        }
        _ => (HircObject::Other { hirc_type }, false),
    })
}

// ---------------------------------------------------------------------------
// Banks and packages

/// One soundbank.
pub struct Bank {
    pub id: u32,
    pub version: u32,
    pub language_id: u32,
    bytes: Arc<Vec<u8>>,
    /// DATA payload: start and length within `bytes`.
    data: (usize, usize),
    /// DIDX: media id to (offset in DATA, size), in index order.
    media: HashMap<u32, (usize, usize)>,
    media_order: Vec<u32>,
    pub objects: HashMap<u32, HircObject>,
    pub hirc_counts: BTreeMap<u8, usize>,
    /// Objects of an honoured type that failed to parse or whose size did
    /// not match what was read: (id, type, error).
    pub parse_errors: Vec<(u32, u8, String)>,
    /// Objects whose id was already taken in this bank (the first is kept).
    pub id_collisions: Vec<(u32, u8)>,
}

impl Bank {
    fn parse(bytes: Arc<Vec<u8>>, start: usize, len: usize, language_id: u32) -> Result<Bank, String> {
        let buf = bytes.get(start..start + len).ok_or("bank: outside the package")?;
        let mut bank = Bank {
            id: 0,
            version: 0,
            language_id,
            bytes: bytes.clone(),
            data: (0, 0),
            media: HashMap::new(),
            media_order: Vec::new(),
            objects: HashMap::new(),
            hirc_counts: BTreeMap::new(),
            parse_errors: Vec::new(),
            id_collisions: Vec::new(),
        };
        let mut didx: &[u8] = &[];
        let mut hirc_section: &[u8] = &[];
        let mut off = 0;
        while off + 8 <= buf.len() {
            let tag = &buf[off..off + 4];
            let size = u32::from_le_bytes(buf[off + 4..off + 8].try_into().unwrap()) as usize;
            let body = buf.get(off + 8..off + 8 + size).ok_or("bank: section truncated")?;
            match tag {
                b"BKHD" => {
                    let mut r = Rd::new(body);
                    bank.version = r.u32()?;
                    bank.id = r.u32()?;
                }
                b"DIDX" => didx = body,
                b"DATA" => bank.data = (start + off + 8, size),
                b"HIRC" => hirc_section = body,
                _ => {}
            }
            off += 8 + size;
        }
        if bank.version != 135 {
            return Err(format!("bank {}: version {} (only 135 is read)", bank.id, bank.version));
        }
        for e in didx.chunks_exact(12) {
            let id = u32::from_le_bytes(e[0..4].try_into().unwrap());
            let o = u32::from_le_bytes(e[4..8].try_into().unwrap()) as usize;
            let s = u32::from_le_bytes(e[8..12].try_into().unwrap()) as usize;
            if o + s > bank.data.1 {
                return Err(format!("bank {}: media {id} outside DATA", bank.id));
            }
            if bank.media.insert(id, (o, s)).is_none() {
                bank.media_order.push(id);
            }
        }

        let mut r = Rd::new(hirc_section);
        let count = if hirc_section.is_empty() { 0 } else { r.u32()? };
        for _ in 0..count {
            let hirc_type = r.u8()?;
            let size = r.u32()? as usize;
            let body = r.take(size)?;
            *bank.hirc_counts.entry(hirc_type).or_insert(0) += 1;
            let mut br = Rd::new(body);
            let id = br.u32()?;
            let object = match parse_object(hirc_type, &mut br) {
                Ok((object, complete)) => {
                    if complete && br.rest() != 0 {
                        bank.parse_errors.push((id, hirc_type, format!("{} bytes left unread", br.rest())));
                    }
                    object
                }
                Err(e) => {
                    bank.parse_errors.push((id, hirc_type, e));
                    HircObject::Other { hirc_type }
                }
            };
            if bank.objects.contains_key(&id) {
                bank.id_collisions.push((id, hirc_type));
            } else {
                bank.objects.insert(id, object);
            }
        }
        Ok(bank)
    }

    /// A media item (a WEM file) embedded in this bank.
    pub fn media(&self, id: u32) -> Option<&[u8]> {
        let &(o, s) = self.media.get(&id)?;
        self.bytes.get(self.data.0 + o..self.data.0 + o + s)
    }

    /// Media ids in DIDX order.
    pub fn media_ids(&self) -> impl Iterator<Item = u32> + '_ {
        self.media_order.iter().copied()
    }
}

/// Reads an AKPK file package and the soundbanks in it.
pub fn parse_pck(bytes: Vec<u8>) -> Result<Vec<Bank>, String> {
    let bytes = Arc::new(bytes);
    let mut r = Rd::new(&bytes);
    if r.take(4)? != b"AKPK" {
        return Err("pck: not an AKPK file package".into());
    }
    let header_size = r.u32()? as usize;
    let version = r.u32()?;
    if version != 1 {
        return Err(format!("pck: version {version}"));
    }
    let language_map = r.u32()? as usize;
    let banks_lut = r.u32()? as usize;
    let streams_lut = r.u32()? as usize;
    let tables = language_map + banks_lut + streams_lut;
    // With an external-file table the fixed fields are 28 bytes, else 24.
    if 24 + tables != header_size + 8 {
        let externals_lut = r.u32()? as usize;
        if 28 + tables + externals_lut != header_size + 8 {
            return Err("pck: table sizes do not add up to the header size".into());
        }
    }
    r.skip(language_map)?;

    let mut lr = Rd::new(r.take(banks_lut)?);
    let n = lr.u32()?;
    let mut banks = Vec::with_capacity(n as usize);
    for _ in 0..n {
        let _file_id = lr.u32()?;
        let block_size = lr.u32()? as usize;
        let file_size = lr.u32()? as usize;
        let start_block = lr.u32()? as usize;
        let language_id = lr.u32()?;
        banks.push(Bank::parse(bytes.clone(), start_block * block_size, file_size, language_id)?);
    }
    Ok(banks)
}

// ---------------------------------------------------------------------------
// Event resolution

/// A Stop action's scope, from the action type's low byte.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StopScope {
    /// 0x02/0x03: the target (and what plays under it).
    Target,
    /// 0x04/0x05: everything.
    All,
    /// 0x08/0x09: everything but the exceptions.
    AllExcept,
}

#[derive(Clone, Debug)]
pub struct SoundPlay {
    pub id: u32,
    pub media_id: u32,
    /// The bank whose DIDX holds the media, if any does.
    pub media_bank_id: Option<u32>,
    pub stream_type: u8,
    pub codec_plugin_id: u32,
    /// The Loop property: 1 plays once (the default when absent), 0 loops
    /// forever, n plays n times.
    pub loop_count: u32,
    /// The Volume property, dB.
    pub volume_db: f32,
    /// The Make-Up Gain property, dB.
    pub make_up_gain_db: f32,
    /// Whether the bank places this sound in 3D space rather than flat,
    /// resolved up the actor-mixer hierarchy ([`Package::positional`]).
    pub positional: bool,
}

#[derive(Clone, Debug)]
pub struct ContainerPlay {
    pub id: u32,
    pub kind: ContainerKind,
    /// Random/sequence: the playlist in order; layer: all children.
    pub children: Vec<PlayNode>,
    /// Random: each playlist item's weight (Wwise's default is 50000).
    pub weights: Vec<i32>,
    /// 1 plays once, 0 loops forever; applies to continuous containers.
    pub loop_count: u16,
    pub continuous: bool,
    pub shuffle: bool,
    pub avoid_repeat_count: u16,
    pub transition_mode: u8,
    pub transition_time_s: f32,
    pub volume_db: f32,
    /// Whether the bank places this container in 3D space rather than flat,
    /// resolved up the actor-mixer hierarchy ([`Package::positional`]).
    pub positional: bool,
}

#[derive(Clone, Debug)]
pub enum PlayNode {
    Sound(SoundPlay),
    Container(ContainerPlay),
    /// No bank holds the object.
    Missing(u32),
    /// A type this reader doesn't play (switch containers, music).
    Unsupported { id: u32, hirc_type: u8 },
}

impl PlayNode {
    pub fn id(&self) -> u32 {
        match self {
            PlayNode::Sound(s) => s.id,
            PlayNode::Container(c) => c.id,
            PlayNode::Missing(id) => *id,
            PlayNode::Unsupported { id, .. } => *id,
        }
    }

    fn collect_media(&self, out: &mut Vec<u32>) {
        match self {
            PlayNode::Sound(s) => out.push(s.media_id),
            PlayNode::Container(c) => c.children.iter().for_each(|n| n.collect_media(out)),
            _ => {}
        }
    }
}

#[derive(Clone, Debug)]
pub enum ActionKind {
    Play {
        node: PlayNode,
        /// Sum of the Volume properties of the target's parents (containers
        /// and actor-mixers; buses are not included), dB.
        parent_volume_db: f32,
    },
    Stop {
        scope: StopScope,
        exceptions: Vec<u32>,
    },
    /// Any other action (pause, set state, ...), left to the caller.
    Other,
}

#[derive(Clone, Debug)]
pub struct EventAction {
    pub action_id: u32,
    pub action_type: u16,
    pub target: u32,
    /// The bank the target object was found in.
    pub target_bank_id: Option<u32>,
    /// The action's DelayTime property, raw (see `prop::DELAY_TIME`).
    pub delay: Option<u32>,
    /// The action's TransitionTime property (fade), raw.
    pub transition: Option<u32>,
    pub kind: ActionKind,
}

#[derive(Clone, Debug)]
pub struct Playback {
    pub event_id: u32,
    pub event_bank_id: u32,
    pub actions: Vec<EventAction>,
}

impl Playback {
    /// Every media id any Play action can reach.
    pub fn media_ids(&self) -> Vec<u32> {
        let mut out = Vec::new();
        for a in &self.actions {
            if let ActionKind::Play { node, .. } = &a.kind {
                node.collect_media(&mut out);
            }
        }
        out
    }

    /// Whether an action's target, or a media item a sound plays, sits in a
    /// different bank than the event.
    pub fn crosses_banks(&self) -> bool {
        fn media_elsewhere(node: &PlayNode, bank: u32) -> bool {
            match node {
                PlayNode::Sound(s) => s.media_bank_id.is_some_and(|b| b != bank),
                PlayNode::Container(c) => c.children.iter().any(|n| media_elsewhere(n, bank)),
                _ => false,
            }
        }
        self.actions.iter().any(|a| {
            a.target_bank_id.is_some_and(|b| b != self.event_bank_id)
                || matches!(&a.kind, ActionKind::Play { node, .. } if media_elsewhere(node, self.event_bank_id))
        })
    }
}

/// All banks of all of an aircraft's packages.
pub struct Package {
    pub banks: Vec<Bank>,
}

impl Package {
    pub fn from_pcks(pcks: Vec<Vec<u8>>) -> Result<Package, String> {
        let mut banks = Vec::new();
        for p in pcks {
            banks.extend(parse_pck(p)?);
        }
        Ok(Package { banks })
    }

    /// Reads every `.pck` file (any case) in a directory, in name order.
    pub fn load_dir(dir: &Path) -> Result<Package, String> {
        let mut paths: Vec<_> = std::fs::read_dir(dir)
            .map_err(|e| format!("{}: {e}", dir.display()))?
            .filter_map(|e| e.ok().map(|e| e.path()))
            .filter(|p| p.extension().is_some_and(|x| x.eq_ignore_ascii_case("pck")))
            .collect();
        paths.sort();
        let pcks = paths
            .iter()
            .map(|p| std::fs::read(p).map_err(|e| format!("{}: {e}", p.display())))
            .collect::<Result<Vec<_>, _>>()?;
        Package::from_pcks(pcks)
    }

    /// A media item from whichever bank embeds it.
    pub fn media(&self, id: u32) -> Option<&[u8]> {
        self.banks.iter().find_map(|b| b.media(id))
    }

    /// An object, preferring bank `prefer`, and the bank it came from.
    pub fn object(&self, id: u32, prefer: Option<usize>) -> Option<(usize, &HircObject)> {
        if let Some(i) = prefer {
            if let Some(o) = self.banks[i].objects.get(&id) {
                return Some((i, o));
            }
        }
        self.banks.iter().enumerate().find_map(|(i, b)| b.objects.get(&id).map(|o| (i, o)))
    }

    /// The node's parents up the actor-mixer hierarchy, nearest first.
    pub fn ancestors(&self, id: u32) -> Vec<u32> {
        let mut out = Vec::new();
        let mut cur = id;
        while let Some(parent) = self.object(cur, None).and_then(|(_, o)| o.base()).map(|b| b.parent_id) {
            if parent == 0 || out.contains(&parent) || out.len() > 64 {
                break;
            }
            out.push(parent);
            cur = parent;
        }
        out
    }

    /// Whether a node plays in 3D space: its own `positional_override` if it
    /// has one, else its nearest ancestor's, else flat (2D, MSFS's default).
    pub fn positional(&self, id: u32) -> bool {
        if let Some(p) = self.object(id, None).and_then(|(_, o)| o.base()).and_then(|b| b.positional_override) {
            return p;
        }
        self.ancestors(id)
            .iter()
            .find_map(|&a| self.object(a, None).and_then(|(_, o)| o.base()).and_then(|b| b.positional_override))
            .unwrap_or(false)
    }

    pub fn resolve_event_name(&self, name: &str) -> Option<Playback> {
        self.resolve_event(short_id(name))
    }

    /// What an event does: its actions, with Play targets resolved to the
    /// sounds and containers beneath them.
    pub fn resolve_event(&self, event_id: u32) -> Option<Playback> {
        let (bank_index, actions) = self.banks.iter().enumerate().find_map(|(i, b)| match b.objects.get(&event_id) {
            Some(HircObject::Event(actions)) => Some((i, actions)),
            _ => None,
        })?;
        let mut out = Playback { event_id, event_bank_id: self.banks[bank_index].id, actions: Vec::new() };
        for &action_id in actions {
            let Some((_, HircObject::Action(a))) = self.object(action_id, Some(bank_index)) else {
                continue;
            };
            let target_bank = self.object(a.target, Some(bank_index)).map(|(i, _)| self.banks[i].id);
            let kind = match a.action_type >> 8 {
                0x04 => ActionKind::Play {
                    node: self.resolve_node(a.target, Some(bank_index), 0),
                    parent_volume_db: self
                        .ancestors(a.target)
                        .iter()
                        .filter_map(|&p| self.object(p, None).and_then(|(_, o)| o.base()))
                        .map(NodeBase::volume_db)
                        .sum(),
                },
                0x01 => ActionKind::Stop {
                    scope: match a.action_type & 0xFF {
                        0x04 | 0x05 => StopScope::All,
                        0x08 | 0x09 => StopScope::AllExcept,
                        _ => StopScope::Target,
                    },
                    exceptions: a.exceptions.clone(),
                },
                _ => ActionKind::Other,
            };
            out.actions.push(EventAction {
                action_id,
                action_type: a.action_type,
                target: a.target,
                target_bank_id: target_bank,
                delay: a.prop(prop::DELAY_TIME),
                transition: a.prop(prop::TRANSITION_TIME),
                kind,
            });
        }
        Some(out)
    }

    fn resolve_node(&self, id: u32, prefer: Option<usize>, depth: usize) -> PlayNode {
        let Some((bank, object)) = self.object(id, prefer) else {
            return PlayNode::Missing(id);
        };
        if depth > 32 {
            return PlayNode::Unsupported { id, hirc_type: 0 };
        }
        match object {
            HircObject::Sound(s) => PlayNode::Sound(SoundPlay {
                id,
                media_id: s.source_id,
                media_bank_id: self.banks.iter().find(|b| b.media.contains_key(&s.source_id)).map(|b| b.id),
                stream_type: s.stream_type,
                codec_plugin_id: s.plugin_id,
                loop_count: s.base.prop(prop::LOOP).unwrap_or(1),
                volume_db: s.base.volume_db(),
                make_up_gain_db: s.base.prop(prop::MAKE_UP_GAIN).map(f32::from_bits).unwrap_or(0.0),
                positional: self.positional(id),
            }),
            HircObject::Container(c) => {
                let (ids, weights): (Vec<u32>, Vec<i32>) = match c.kind {
                    ContainerKind::Layer => (c.children.clone(), Vec::new()),
                    _ => c.playlist.iter().copied().unzip(),
                };
                PlayNode::Container(ContainerPlay {
                    id,
                    kind: c.kind,
                    children: ids.iter().map(|&child| self.resolve_node(child, Some(bank), depth + 1)).collect(),
                    weights: if c.kind == ContainerKind::Random { weights } else { Vec::new() },
                    loop_count: c.loop_count,
                    continuous: c.continuous,
                    shuffle: c.random_mode == 1,
                    avoid_repeat_count: c.avoid_repeat_count,
                    transition_mode: c.transition_mode,
                    transition_time_s: c.transition_time_s,
                    volume_db: c.base.volume_db(),
                    positional: self.positional(id),
                })
            }
            HircObject::Action(_) => PlayNode::Unsupported { id, hirc_type: hirc::ACTION },
            HircObject::Event(_) => PlayNode::Unsupported { id, hirc_type: hirc::EVENT },
            HircObject::ActorMixer { .. } => PlayNode::Unsupported { id, hirc_type: hirc::ACTOR_MIXER },
            HircObject::Other { hirc_type } => PlayNode::Unsupported { id, hirc_type: *hirc_type },
        }
    }
}

/// The FlyByWire A380X MSFS package's sound folder on this machine, for the
/// tests.
#[cfg(test)]
pub(crate) const PACKAGE_SOUND_DIR: &str = "D:/Microsoft Flight Simulator 2020/Microsoft Flight Simulator 2020 Packages/Community/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380_842/sound";

/// The package loaded once for all tests, or None when it isn't installed.
#[cfg(test)]
pub(crate) fn test_package() -> Option<&'static Package> {
    static PACKAGE: std::sync::OnceLock<Option<Package>> = std::sync::OnceLock::new();
    PACKAGE
        .get_or_init(|| {
            let dir = Path::new(PACKAGE_SOUND_DIR);
            dir.is_dir().then(|| Package::load_dir(dir).expect("package loads"))
        })
        .as_ref()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sound::vorbis::decode_wem;
    use std::collections::BTreeSet;
    use std::time::Instant;

    const WAV_DIR: &str = "C:/Users/bansa/AppData/Local/Temp/claude/d--Converter/2ab03b5e-47e8-485c-9ab0-6b1a7f42eaed/scratchpad/wav";

    /// sound.xml's main package and `WwiseEvent="..."` names.
    fn sound_xml_events() -> (String, Vec<String>) {
        let xml = std::fs::read_to_string(Path::new(PACKAGE_SOUND_DIR).join("sound.xml")).unwrap();
        let mut names = BTreeSet::new();
        for part in xml.split("WwiseEvent=\"").skip(1) {
            names.insert(part.split('"').next().unwrap().to_string());
        }
        (sound_xml_main_package(&xml).unwrap().to_string(), names.into_iter().collect())
    }

    /// A sound.xml name's event.
    fn resolve(pkg: &Package, name: &str) -> Option<Playback> {
        pkg.resolve_event(msfs_event_id("Asobo_A320_NEO", name))
    }

    #[test]
    fn short_id_is_fnv1_of_lowercase() {
        // FNV-1 32-bit of "" is the offset basis; of "a": (basis * prime) ^ 0x61.
        assert_eq!(short_id(""), 0x811C_9DC5);
        assert_eq!(short_id("a"), 0x050C_5D7E);
        assert_eq!(short_id("New_Retard"), short_id("new_retard"));
    }

    #[test]
    fn parse_all_packages() {
        let Some(pkg) = test_package() else {
            eprintln!("skipped: MSFS package not found at {PACKAGE_SOUND_DIR}");
            return;
        };
        assert_eq!(pkg.banks.len(), 3);
        for b in &pkg.banks {
            eprintln!(
                "bank {} v{} lang {}: {} media, HIRC {:?}, {} id collisions {:?}",
                b.id,
                b.version,
                b.language_id,
                b.media_order.len(),
                b.hirc_counts,
                b.id_collisions.len(),
                &b.id_collisions[..b.id_collisions.len().min(5)]
            );
            assert!(b.parse_errors.is_empty(), "bank {}: {:?}", b.id, &b.parse_errors[..b.parse_errors.len().min(10)]);
            for id in b.media_ids() {
                assert_eq!(b.media(id).unwrap().get(0..4), Some(&b"RIFF"[..]), "media {id}");
            }
        }
        let mut ids: Vec<u32> = pkg.banks.iter().map(|b| b.id).collect();
        ids.sort();
        assert_eq!(ids, [814486316, 814486317, 814486318]);

        // HIRC types this reader keeps only as their type (module doc: "kept
        // only as its type"): how many of each are actually in these banks.
        let (mut states, mut switches, mut buses) = (0, 0, 0);
        for b in &pkg.banks {
            states += b.hirc_counts.get(&hirc::STATE).copied().unwrap_or(0);
            switches += b.hirc_counts.get(&hirc::SWITCH_CNTR).copied().unwrap_or(0);
            buses += b.hirc_counts.get(&hirc::BUS).copied().unwrap_or(0);
        }
        eprintln!("HIRC types not played: {states} State, {switches} Switch container, {buses} Bus");

        // NodeBase data parsed to stay aligned but not applied by playback
        // (bus routing, ranged modifiers, RTPC curves are all out of
        // scope): how many nodes actually carry each, and a media_size
        // sanity check against the DIDX entry it names.
        let (mut with_bus, mut with_ranged, mut with_rtpc, mut actor_mixer_children) = (0, 0, 0, 0);
        for b in &pkg.banks {
            for o in b.objects.values() {
                if let Some(base) = o.base() {
                    with_bus += (base.override_bus_id != 0) as usize;
                    with_ranged += !base.ranged_props.is_empty() as usize;
                    with_rtpc += !base.rtpcs.is_empty() as usize;
                }
                match o {
                    HircObject::Sound(s) => {
                        if let Some(media) = pkg.media(s.source_id) {
                            assert_eq!(s.media_size as usize, media.len(), "sound {}: media_size disagrees with DIDX", s.source_id);
                        }
                    }
                    HircObject::Container(c) => {
                        if c.reset_playlist_each_play {
                            eprintln!("{:?} container resets its playlist each play (not reproduced)", c.kind);
                        }
                    }
                    HircObject::Action(a) => {
                        if a.is_bus {
                            eprintln!("action targets a bus (type {:#06x}, target {}, not applied)", a.action_type, a.target);
                        }
                    }
                    HircObject::ActorMixer { children, .. } => actor_mixer_children += children.len(),
                    _ => {}
                }
            }
        }
        eprintln!(
            "node base: {with_bus} override their bus, {with_ranged} have ranged modifiers, {with_rtpc} have RTPC curves (none applied); actor-mixers reach {actor_mixer_children} children total"
        );

        // `resolve_event_name` (short_id of the raw name) against
        // `resolve_event` (an already-computed id): both must find the same
        // event for the full `play_<MainPackage>_<name>` form msfs_event_id
        // builds.
        for name in ["new_retard", "cavcharge", "mastercaution"] {
            let raw = format!("play_Asobo_A320_NEO_{name}");
            assert_eq!(
                pkg.resolve_event_name(&raw).map(|pb| pb.event_id),
                pkg.resolve_event(msfs_event_id("Asobo_A320_NEO", name)).map(|pb| pb.event_id),
                "{name}: resolve_event_name should agree with resolve_event"
            );
        }

        // What the property values look like, to know float from integer.
        let mut seen: BTreeMap<(u8, &str), BTreeSet<u32>> = BTreeMap::new();
        for b in &pkg.banks {
            for o in b.objects.values() {
                match o {
                    HircObject::Action(a) => {
                        for &(p, v) in &a.props {
                            seen.entry((p, "action")).or_default().insert(v);
                        }
                    }
                    HircObject::Sound(s) => {
                        for &(p, v) in &s.base.props {
                            seen.entry((p, "sound")).or_default().insert(v);
                        }
                        if s.stream_type != 0 {
                            eprintln!("sound {} streams (type {})", s.source_id, s.stream_type);
                        }
                    }
                    _ => {}
                }
            }
        }
        for ((p, kind), vals) in &seen {
            let sample: Vec<String> =
                vals.iter().take(6).map(|&v| format!("{v:#x}={}/{}", v as i32, f32::from_bits(v))).collect();
            eprintln!("{kind} prop {p:#04x}: {} distinct, e.g. {}", vals.len(), sample.join(", "));
        }
        // Two properties this reader has ids for but never reads (Random
        // containers' picks come from the playlist's own weights, not a
        // per-child Probability; a Play action's own start delay is not
        // applied): how many actions actually carry them.
        let with_probability = seen.get(&(prop::PROBABILITY, "action")).map_or(0, BTreeSet::len);
        let with_initial_delay = seen.get(&(prop::INITIAL_DELAY, "action")).map_or(0, BTreeSet::len);
        eprintln!("action prop values seen: {with_probability} distinct Probability, {with_initial_delay} distinct InitialDelay (neither applied)");
    }

    #[test]
    fn sound_xml_events_resolve() {
        let Some(pkg) = test_package() else {
            eprintln!("skipped: MSFS package not found at {PACKAGE_SOUND_DIR}");
            return;
        };
        let (main_package, names) = sound_xml_events();
        assert_eq!(main_package, "Asobo_A320_NEO");
        // The package names are the bank ids; the bare sound.xml names are
        // not event ids.
        for (i, bank_id) in [(1, 814486318u32), (2, 814486317), (3, 814486316)] {
            assert_eq!(short_id(&format!("FBW_A320_NEO_{i}")), bank_id);
        }
        let events: BTreeSet<u32> = pkg
            .banks
            .iter()
            .flat_map(|b| b.objects.iter().filter(|(_, o)| matches!(o, HircObject::Event(_))).map(|(&id, _)| id))
            .collect();
        assert!(names.iter().all(|n| !events.contains(&short_id(n))));

        let mut found = 0;
        let mut missing = Vec::new();
        let mut crossing = Vec::new();
        let mut media_refs = 0;
        for name in &names {
            let Some(pb) = pkg.resolve_event(msfs_event_id(&main_package, name)) else {
                missing.push(name.as_str());
                continue;
            };
            found += 1;
            if pb.crosses_banks() {
                crossing.push(name.as_str());
            }
            for a in &pb.actions {
                if let ActionKind::Play { node, .. } = &a.kind {
                    assert!(!matches!(node, PlayNode::Missing(_)), "{name}: play target {} missing", a.target);
                }
            }
            for m in pb.media_ids() {
                assert!(pkg.media(m).is_some(), "{name}: media {m} in no bank");
                media_refs += 1;
            }
        }
        eprintln!(
            "sound.xml: {} event names, {found} resolve, {media_refs} media references all present; crossing banks {crossing:?}; not in these packages {missing:?}",
            names.len()
        );
        assert_eq!(found, 206);
        for name in ["new_retard", "new_100", "aural_minimumnew", "cavcharge", "mastercaution"] {
            assert!(resolve(pkg, name).is_some(), "{name}");
        }

        // Every event in every bank, not just sound.xml's.
        let (mut events, mut refs, mut cross) = (0, 0, 0);
        let mut stops = Vec::new();
        let mut kinds = BTreeMap::new();
        let sound_xml_ids: BTreeMap<u32, &str> =
            names.iter().map(|n| (msfs_event_id(&main_package, n), n.as_str())).collect();
        for b in &pkg.banks {
            for (&id, o) in &b.objects {
                if let HircObject::Event(_) = o {
                    let pb = pkg.resolve_event(id).unwrap();
                    events += 1;
                    cross += pb.crosses_banks() as usize;
                    for a in &pb.actions {
                        *kinds.entry(a.action_type).or_insert(0) += 1;
                        if let ActionKind::Stop { scope, exceptions } = &a.kind {
                            let target = pkg.object(a.target, None).map(|(_, o)| format!("{o:?}").chars().take(60).collect::<String>());
                            stops.push((sound_xml_ids.get(&id).copied(), id, a.target, *scope, exceptions.len(), target));
                        }
                    }
                    for m in pb.media_ids() {
                        assert!(pkg.media(m).is_some(), "event {id}: media {m} in no bank");
                        refs += 1;
                    }
                }
            }
        }
        eprintln!(
            "all banks: {events} events, {refs} media references present, {cross} events crossing banks, action types {kinds:x?}, stops (sound.xml name, event, target, scope, target object) {stops:?}"
        );
    }

    fn describe(pkg: &Package, node: &PlayNode, indent: usize, out: &mut String) {
        let pad = " ".repeat(indent);
        match node {
            PlayNode::Sound(s) => {
                let info = pkg.media(s.media_id).and_then(|w| crate::sound::vorbis::wem_info(w).ok());
                let fmt = info
                    .map(|i| format!("tag {:#06x} {} ch {} Hz {} frames", i.format_tag, i.channels, i.sample_rate, i.frames))
                    .unwrap_or_default();
                out.push_str(&format!(
                    "{pad}sound {} media {} (bank {:?}) stream {} codec {:#010x} positional {} loop {} vol {:+.1} dB makeup {:+.1} dB [{fmt}]\n",
                    s.id, s.media_id, s.media_bank_id, s.stream_type, s.codec_plugin_id, s.positional, s.loop_count, s.volume_db, s.make_up_gain_db
                ));
            }
            PlayNode::Container(c) => {
                out.push_str(&format!(
                    "{pad}{:?} container {} loop {} continuous {} shuffle {} avoid {} transition {} {:.2}s vol {:+.1} dB positional {} weights {:?}\n",
                    c.kind, c.id, c.loop_count, c.continuous, c.shuffle, c.avoid_repeat_count, c.transition_mode, c.transition_time_s, c.volume_db, c.positional, c.weights
                ));
                for ch in &c.children {
                    describe(pkg, ch, indent + 2, out);
                }
            }
            PlayNode::Missing(id) => out.push_str(&format!("{pad}missing {id} (id() {})\n", node.id())),
            PlayNode::Unsupported { id, hirc_type } => {
                out.push_str(&format!("{pad}unsupported {id} hirc_type {hirc_type:#04x} (id() {})\n", node.id()))
            }
        }
    }

    fn write_wav(path: &Path, pcm: &crate::sound::vorbis::Pcm) {
        let data_len = pcm.samples.len() as u32 * 2;
        let mut w = Vec::with_capacity(44 + data_len as usize);
        w.extend_from_slice(b"RIFF");
        w.extend_from_slice(&(36 + data_len).to_le_bytes());
        w.extend_from_slice(b"WAVEfmt ");
        w.extend_from_slice(&16u32.to_le_bytes());
        w.extend_from_slice(&1u16.to_le_bytes());
        w.extend_from_slice(&pcm.channels.to_le_bytes());
        w.extend_from_slice(&pcm.sample_rate.to_le_bytes());
        w.extend_from_slice(&(pcm.sample_rate * pcm.channels as u32 * 2).to_le_bytes());
        w.extend_from_slice(&(pcm.channels * 2).to_le_bytes());
        w.extend_from_slice(&16u16.to_le_bytes());
        w.extend_from_slice(b"data");
        w.extend_from_slice(&data_len.to_le_bytes());
        for s in &pcm.samples {
            w.extend_from_slice(&s.to_le_bytes());
        }
        std::fs::write(path, w).unwrap();
    }

    /// The callouts and chimes: their resolution, a decode timing, and WAVs
    /// to listen to.
    #[test]
    fn callouts_to_wav() {
        let Some(pkg) = test_package() else {
            eprintln!("skipped: MSFS package not found at {PACKAGE_SOUND_DIR}");
            return;
        };
        let _ = std::fs::create_dir_all(WAV_DIR);
        let names = [
            "new_retard",
            "new_100",
            "new_2500",
            "aural_minimumnew",
            "aural_100above",
            "cavcharge",
            "mastercaution",
            "3click",
            "aural_stall_new",
        ];
        let mut report = String::new();
        for name in names {
            let Some(pb) = resolve(pkg, name) else {
                report.push_str(&format!("{name}: no event\n"));
                continue;
            };
            report.push_str(&format!("{name} = event {} in bank {}\n", pb.event_id, pb.event_bank_id));
            for a in &pb.actions {
                report.push_str(&format!(
                    "  action {} type {:#06x} target {} (bank {:?}) delay {:?} transition {:?}\n",
                    a.action_id, a.action_type, a.target, a.target_bank_id, a.delay, a.transition
                ));
                match &a.kind {
                    ActionKind::Play { node, parent_volume_db } => {
                        report.push_str(&format!("    parents vol {parent_volume_db:+.1} dB\n"));
                        describe(pkg, node, 4, &mut report);
                    }
                    k => report.push_str(&format!("    {k:?}\n")),
                }
            }
            for (i, m) in pb.media_ids().into_iter().enumerate() {
                let wem = pkg.media(m).unwrap();
                let t0 = Instant::now();
                let pcm = decode_wem(wem).unwrap();
                let ms = t0.elapsed().as_secs_f64() * 1e3;
                let path = Path::new(WAV_DIR).join(format!("{name}_{i}_{m}.wav"));
                write_wav(&path, &pcm);
                report.push_str(&format!(
                    "  media {m}: {} frames, {:.2} s, loop {:?}, decoded in {ms:.2} ms -> {}\n",
                    pcm.frames(),
                    pcm.frames() as f64 / pcm.sample_rate as f64,
                    pcm.loop_frames,
                    path.display()
                ));
            }
        }
        eprintln!("{report}");
    }
}
