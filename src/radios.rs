//! The radios: MSFS's NAV, ADF and COM receivers as FlyByWire's A380X reads
//! and tunes them, on X-Plane's radios.
//!
//! FlyByWire tunes through key events and reads simulator variables. The FMS
//! navaid tuner (fbw-a32nx fmgc NavaidTuner.ts, shared by the A380) sends
//! `NAVn_RADIO_SET_HZ` for VOR 1/2 (NAV1/2, :489) and the MMRs (NAV3/4, :526),
//! `VORn_SET` for their courses (:504, :550) and `ADF_COMPLETE_SET` (:569);
//! it lets the pilot's own tuning events through while the RMP tunes (its
//! intercept list, :63-174). The VHF radios send `COMn_RADIO_SET_HZ`
//! (systems-host Communications/VhfRadio.ts:298, 331) and the audio manager
//! the volume, ident, receive and transmitter events (SimAudioManager.ts:55-153).
//!
//! What each event does is MSFS's, from its SDK documentation (Event IDs,
//! Aircraft Radio Navigation Events) and, for the BCD encodings, from MSFS's
//! own `Avionics.Utils` (fs-base-ui html_ui/JS/Avionics.js:4-13) and FlyByWire's
//! RadioUtils (fbw-common shared/src/RadioUtils.ts:177-223). The receivers hold
//! the frequency exactly as MSFS does, in hertz, and X-Plane's radios follow at
//! their own resolution; a change made in X-Plane (its cockpit, another
//! plugin) is taken in the next tick.
//!
//! | MSFS | X-Plane (DataRefs.txt) | notes |
//! |---|---|---|
//! | NAV1-4 active / standby | cockpit2/radios/actuators/nav_frequency_hz[0-3], nav_standby_frequency_hz | 10 kHz |
//! | VOR1-4 OBS | nav_obs_deg_mag_pilot[0-3], nav_obs_deg_mag_copilot | |
//! | ADF1-2 active / standby | adf1/2_frequency_hz, adf1/2_standby_frequency_hz | kHz (the C172's own script clamps them to 200-1799, c172_custom_datarefs.lua:63-66) |
//! | COM1-2 active / standby | com1/2_frequency_hz_833, com1/2_standby_frequency_hz_833 | kHz; X-Plane has no COM3, which lives here only |
//! | NAV/ADF/COM volume, ident, receive | audio_volume_*, audio_selection_* | |
//! | transmitter | audio_com_selection (6 COM1, 7 COM2) | |
//! | marker sound | audio_marker_enabled | |
//!
//! The simulator variables FlyByWire reads, and their units:
//!
//! | variable | read in | from |
//! |---|---|---|
//! | NAV ACTIVE FREQUENCY:1-4, NAV FREQUENCY:3 | MHz (VorBusPublisher.ts:72-109, PFDSimvarPublisher.tsx:248, Navigation.ts:442) | the receiver |
//! | NAV OBS:1-4 | degrees (VorBusPublisher.ts:76-113) | nav_obs_deg_mag_pilot |
//! | NAV HAS NAV:1-4 | bool (:80-116) | nav_type not 0 (a navaid received) |
//! | NAV HAS DME, NAV DME:1,2,4 | bool, nm | nav_has_dme, nav_dme_distance_nm |
//! | NAV RELATIVE BEARING TO STATION:1-4 | degrees | nav_relative_bearing_deg |
//! | NAV LOCALIZER:1-4 | degrees (FmcAircraftInterface.ts:1668) | nav_course_deg_mag_pilot while a localizer is received |
//! | NAV RADIAL ERROR:1,2,4 | degrees | localizer: hdef dots at FBW's scale, as sensors.rs; VOR: radial minus OBS |
//! | NAV MAGVAR:1,2,4 | degrees | as sensors.rs |
//! | ADF ACTIVE FREQUENCY:1-2 | kHz (VorBusPublisher.ts:125, 130) | the receiver |
//! | ADF RADIAL:1-2 | degrees, used as the relative bearing (ND RadioNeedle.tsx:291-293) | adf1/2_relative_bearing_deg |
//! | COM ACTIVE / STANDBY FREQUENCY:1-3 | Frequency BCD32 (VhfRadio.ts:384, RMP VhfComManager.ts:165, 172) | the receiver |
//! | MARKER SOUND | bool (SimAudioManager.ts:89) | audio_marker_enabled |
//! | NAV VOLUME:1-4, NAV SOUND:1-4 | percent, bool | the receiver |
//! | NAV TOFROM:1-4 | enum 0 off, 1 to, 2 from | nav_flag_from_to_pilot (same enum) |
//! | ADF VOLUME:1-2, ADF SOUND:1-2 | percent over 100, bool | the receiver |
//! | COM VOLUME:1-3, COM RECEIVE:1-3, COM TRANSMIT:1-3, COM RECEIVE ALL | percent, bool | the receiver |
//! | MARKER BEACON STATE | enum 0 none, 1 outer, 2 middle, 3 inner (PFDSimvarPublisher.tsx:253) | over_*_marker |
//!
//! NAV :3's localizer, glide slope, DME and magnetic variation are sensors.rs's
//! (the PRIMs' ILS) and are left to it. Not served: the string variables
//! (NAV IDENT, ADF IDENT), which the plugin's numeric variables cannot hold
//! (published instead as `fbw/radio/<receiver>/ident`); NAV VOR LATLONALT and
//! NAV GS LATLONALT, which are structures; ADF SIGNAL, NAV SIGNAL and NAV
//! CDI, for which X-Plane has no signal strength and no needle on MSFS's
//! +/-127 scale; and the TACAN events, as X-Plane's TACANs are not these
//! receivers. Units are the SDK's (Aircraft Radio Navigation Variables).
//!
//! The MFD's manual LS tuning (POSITION/NAVAIDS, MfdFmsPositionNavaids.tsx:
//! 278-311, 664-676) is `fbw/radio/ls/frequency_mhz` and `course_deg`: see
//! [`LsTuning`].

use std::sync::Mutex;

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::published::{self, Command, Published, Value};
use crate::sensors::{is_localizer_frequency, magnetic_minus_true, normalise_180, normalise_360, LOC_DEG_PER_DOT};
use crate::xp::{DataRef, Xplm};
use crate::Vars;

// ---------------------------------------------------------------------------
// Frequency encodings.
// ---------------------------------------------------------------------------

/// FlyByWire's RadioUtils.unpackBcd32 (RadioUtils.ts:214-223): MSFS's BCD32,
/// kHz digits from bit 4 up, to hertz.
pub fn bcd32_to_hz(bcd32: u32) -> f64 {
    (0..6).map(|k| ((bcd32 >> (4 + 4 * k)) & 0xf) as f64 * 1_000. * 10f64.powi(k as i32)).sum()
}

/// RadioUtils.packBcd32 (RadioUtils.ts:177-187), to 1 kHz.
pub fn hz_to_bcd32(hz: f64) -> u32 {
    let khz = (hz / 1_000.).round() as u64;
    (0..6).map(|k| (((khz / 10u64.pow(k)) % 10) as u32) << (4 + 4 * k)).sum()
}

/// RadioUtils.bcd16ToBcd32 (RadioUtils.ts:204-206), then to hertz: the four
/// digits below the hundreds of MHz, as MSFS's make_bcd16 packs a NAV or COM
/// frequency (Avionics.js:4-8).
pub fn bcd16_to_hz(bcd16: u32) -> f64 {
    bcd32_to_hz(0x100_0000 | (bcd16 << 8))
}

/// MSFS's ADF BCD32 (make_adf_bcd32, Avionics.js:9-13): tenths of kHz in five
/// BCD digits, shifted 12 bits up, to hertz.
pub fn adf_bcd32_to_hz(bcd32: u32) -> f64 {
    let digits = bcd32 >> 12;
    let tenths: f64 = (0..5).map(|k| ((digits >> (4 * k)) & 0xf) as f64 * 10f64.powi(k as i32)).sum();
    tenths * 100.
}

#[cfg(test)]
pub fn hz_to_adf_bcd32(hz: f64) -> u32 {
    let tenths = (hz / 100.).round() as u64;
    (0..5).map(|k| (((tenths / 10u64.pow(k)) % 10) as u32) << (4 * k)).sum::<u32>() << 12
}

// ---------------------------------------------------------------------------
// Tuning steps, as the MSFS SDK describes each event.
// ---------------------------------------------------------------------------

const MHZ: f64 = 1_000_000.;
const KHZ: f64 = 1_000.;

/// The VOR/ILS band, 108.00 to 117.95 MHz (ICAO Annex 10), inside which
/// MSFS's NAV whole-MHz and carrying steps wrap.
const NAV_BAND_MHZ: (f64, f64) = (108., 118.);

/// "Values are from 118 to 137, and this will wrap" (COM_RADIO_WHOLE_INC).
const COM_BAND_MHZ: (f64, f64) = (118., 137.);

fn wrap(v: f64, low: f64, high: f64) -> f64 {
    low + (v - low).rem_euclid(high - low)
}

/// A whole-MHz step, the kHz kept.
fn whole_step(hz: f64, sign: f64, band: (f64, f64)) -> f64 {
    let mhz = (hz / MHZ).floor();
    let khz = hz - mhz * MHZ;
    wrap(mhz + sign, band.0, band.1) * MHZ + khz
}

/// A kHz step that wraps within the MHz ("no carry when digit wraps").
fn fract_step(hz: f64, step_khz: f64, band: (f64, f64)) -> f64 {
    let mhz = (hz / MHZ).floor().clamp(band.0, band.1 - 1.);
    let khz = (hz - mhz * MHZ + step_khz * KHZ).rem_euclid(MHZ);
    mhz * MHZ + khz
}

/// A kHz step that carries into the MHz, wrapping around the band.
fn carry_step(hz: f64, step_khz: f64, band: (f64, f64)) -> f64 {
    wrap(hz + step_khz * KHZ, band.0 * MHZ, band.1 * MHZ)
}

/// An ADF step on one BCD digit of the tenths of kHz (0 tenths, 1 ones, 2
/// tens, 3 hundreds), the digit wrapping on its own.
fn adf_digit_step(hz: f64, digit: u32, sign: i64) -> f64 {
    let tenths = (hz / 100.).round() as i64;
    let place = 10i64.pow(digit);
    let d = (tenths / place) % 10;
    let new = (d + sign).rem_euclid(10);
    (tenths + (new - d) * place) as f64 * 100.
}

/// An ADF step that carries, over the five digits the BCD32 holds.
fn adf_carry_step(hz: f64, tenths: i64) -> f64 {
    (((hz / 100.).round() as i64 + tenths).rem_euclid(100_000)) as f64 * 100.
}

// ---------------------------------------------------------------------------
// The receivers.
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Nav {
    pub active_hz: f64,
    pub standby_hz: f64,
    pub obs_deg: f64,
    /// 0-100 (NAVn_VOLUME_SET_EX1).
    pub volume: f64,
    pub ident_on: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Adf {
    pub active_hz: f64,
    pub standby_hz: f64,
    /// 0-100 (ADF_VOLUME_SET).
    pub volume: f64,
    pub ident_on: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Com {
    pub active_hz: f64,
    pub standby_hz: f64,
    /// 0-1 (COMn_VOLUME_SET).
    pub volume: f64,
    pub receive: bool,
}

/// MSFS's radios, as the events leave them.
#[derive(Clone, Debug, PartialEq)]
pub struct Receivers {
    pub nav: [Nav; 4],
    pub adf: [Adf; 2],
    pub com: [Com; 3],
    /// PILOT_TRANSMITTER_SET: 0 COM1, 1 COM2, 2 COM3, 4 none.
    pub transmitter: u32,
    pub marker_sound: bool,
}

impl Default for Receivers {
    fn default() -> Self {
        Self {
            nav: Default::default(),
            adf: Default::default(),
            com: Default::default(),
            transmitter: 4,
            marker_sound: false,
        }
    }
}

/// `NAME` with an optional receiver number after `prefix`: ("NAV3_RADIO_SET",
/// "NAV") is (3, "_RADIO_SET"), ("COM_RADIO_SET", "COM") is (1, "_RADIO_SET").
fn indexed<'a>(name: &'a str, prefix: &str) -> Option<(usize, &'a str)> {
    let rest = name.strip_prefix(prefix)?;
    match rest.chars().next()? {
        c @ '1'..='4' => Some((c as usize - '0' as usize, &rest[1..])),
        '_' => Some((1, rest)),
        _ => None,
    }
}

impl Receivers {
    /// Apply one MSFS key event. Returns whether it is a radio event.
    pub fn apply(&mut self, name: &str, value: f64) -> bool {
        let name = name.trim().trim_start_matches("K:");
        let name = name.strip_prefix("RADIO_").unwrap_or(name);
        let data = value as i64 as u32;
        if let Some((n, rest)) = indexed(name, "NAV").filter(|(n, _)| (1..=4).contains(n)) {
            return self.apply_nav(n - 1, rest, value, data);
        }
        if let Some((n, rest)) = indexed(name, "VOR").filter(|(n, _)| (1..=4).contains(n)) {
            return self.apply_vor(n - 1, rest, value);
        }
        if let Some((n, rest)) = indexed(name, "ADF").filter(|(n, _)| (1..=2).contains(n)) {
            return self.apply_adf(n - 1, rest, value, data);
        }
        if let Some((n, rest)) = indexed(name, "COM").filter(|(n, _)| (1..=3).contains(n)) {
            return self.apply_com(n - 1, rest, value, data);
        }
        match name {
            "COM_RECEIVE_ALL_SET" => self.com.iter_mut().for_each(|c| c.receive = data != 0),
            "COM_RECEIVE_ALL_TOGGLE" => self.com.iter_mut().for_each(|c| c.receive = !c.receive),
            // Both seats' transmitter select the same radios here: X-Plane has
            // one selector per aircraft.
            "PILOT_TRANSMITTER_SET" | "COPILOT_TRANSMITTER_SET" => self.transmitter = data,
            "MARKER_SOUND_TOGGLE" => self.marker_sound = !self.marker_sound,
            _ => return false,
        }
        true
    }

    fn apply_nav(&mut self, i: usize, event: &str, value: f64, data: u32) -> bool {
        let nav = &mut self.nav[i];
        match event {
            "_RADIO_SET" => nav.active_hz = bcd16_to_hz(data),
            "_RADIO_SET_HZ" => nav.active_hz = value.round(),
            "_STBY_SET" => nav.standby_hz = bcd16_to_hz(data),
            "_STBY_SET_HZ" => nav.standby_hz = value.round(),
            "_RADIO_SWAP" => std::mem::swap(&mut nav.active_hz, &mut nav.standby_hz),
            "_RADIO_WHOLE_INC" => nav.active_hz = whole_step(nav.active_hz, 1., NAV_BAND_MHZ),
            "_RADIO_WHOLE_DEC" => nav.active_hz = whole_step(nav.active_hz, -1., NAV_BAND_MHZ),
            // "by 25 KHz" without carry, "by 50 KHz, and will carry".
            "_RADIO_FRACT_INC" => nav.active_hz = fract_step(nav.active_hz, 25., NAV_BAND_MHZ),
            "_RADIO_FRACT_DEC" => nav.active_hz = fract_step(nav.active_hz, -25., NAV_BAND_MHZ),
            "_RADIO_FRACT_INC_CARRY" => nav.active_hz = carry_step(nav.active_hz, 50., NAV_BAND_MHZ),
            "_RADIO_FRACT_DEC_CARRY" => nav.active_hz = carry_step(nav.active_hz, -50., NAV_BAND_MHZ),
            "_VOLUME_SET_EX1" => nav.volume = value.clamp(0., 100.),
            _ => return false,
        }
        true
    }

    fn apply_vor(&mut self, i: usize, event: &str, value: f64) -> bool {
        let nav = &mut self.nav[i];
        match event {
            "_SET" => nav.obs_deg = value.clamp(0., 360.),
            "_OBI_INC" => nav.obs_deg = normalise_360(nav.obs_deg.round() + 1.),
            "_OBI_DEC" => nav.obs_deg = normalise_360(nav.obs_deg.round() - 1.),
            // "The value will stop on 360 and not wrap", "stop on 0".
            "_OBI_FAST_INC" => nav.obs_deg = (nav.obs_deg + 10.).min(360.),
            "_OBI_FAST_DEC" => nav.obs_deg = (nav.obs_deg - 10.).max(0.),
            "_IDENT_SET" => nav.ident_on = value != 0.,
            "_IDENT_TOGGLE" => nav.ident_on = !nav.ident_on,
            _ => return false,
        }
        true
    }

    fn apply_adf(&mut self, i: usize, event: &str, value: f64, data: u32) -> bool {
        let adf = &mut self.adf[i];
        match event {
            "_SET" | "_COMPLETE_SET" | "_ACTIVE_SET" | "_EXTENDED_SET" | "_HIGHRANGE_SET" | "_LOWRANGE_SET" => {
                adf.active_hz = adf_bcd32_to_hz(data)
            }
            "_STBY_SET" => adf.standby_hz = adf_bcd32_to_hz(data),
            "_RADIO_SWAP" => std::mem::swap(&mut adf.active_hz, &mut adf.standby_hz),
            "_1_INC" => adf.active_hz = adf_digit_step(adf.active_hz, 1, 1),
            "_1_DEC" => adf.active_hz = adf_digit_step(adf.active_hz, 1, -1),
            "_10_INC" => adf.active_hz = adf_digit_step(adf.active_hz, 2, 1),
            "_10_DEC" => adf.active_hz = adf_digit_step(adf.active_hz, 2, -1),
            "_100_INC" => adf.active_hz = adf_digit_step(adf.active_hz, 3, 1),
            "_100_DEC" => adf.active_hz = adf_digit_step(adf.active_hz, 3, -1),
            "_RADIO_TENTHS_INC" => adf.active_hz = adf_digit_step(adf.active_hz, 0, 1),
            "_RADIO_TENTHS_DEC" => adf.active_hz = adf_digit_step(adf.active_hz, 0, -1),
            "_FRACT_INC_CARRY" => adf.active_hz = adf_carry_step(adf.active_hz, 1),
            "_FRACT_DEC_CARRY" => adf.active_hz = adf_carry_step(adf.active_hz, -1),
            "_WHOLE_INC" => adf.active_hz = adf_carry_step(adf.active_hz, 10),
            "_WHOLE_DEC" => adf.active_hz = adf_carry_step(adf.active_hz, -10),
            "_VOLUME_SET" => adf.volume = value.clamp(0., 100.),
            "_IDENT_SET" => adf.ident_on = value != 0.,
            "_IDENT_TOGGLE" => adf.ident_on = !adf.ident_on,
            _ => return false,
        }
        true
    }

    fn apply_com(&mut self, i: usize, event: &str, value: f64, data: u32) -> bool {
        let com = &mut self.com[i];
        match event {
            "_RADIO_SET" => com.active_hz = bcd16_to_hz(data),
            "_RADIO_SET_HZ" => com.active_hz = value.round(),
            "_STBY_RADIO_SET" => com.standby_hz = bcd16_to_hz(data),
            "_STBY_RADIO_SET_HZ" => com.standby_hz = value.round(),
            // COM_STBY_RADIO_SWAP "Swaps COM 1 frequency with standby".
            "_RADIO_SWAP" | "_STBY_RADIO_SWAP" => std::mem::swap(&mut com.active_hz, &mut com.standby_hz),
            "_RADIO_WHOLE_INC" => com.active_hz = whole_step(com.active_hz, 1., COM_BAND_MHZ),
            "_RADIO_WHOLE_DEC" => com.active_hz = whole_step(com.active_hz, -1., COM_BAND_MHZ),
            "_RADIO_FRACT_INC" => com.active_hz = fract_step(com.active_hz, 25., COM_BAND_MHZ),
            "_RADIO_FRACT_DEC" => com.active_hz = fract_step(com.active_hz, -25., COM_BAND_MHZ),
            "_RADIO_FRACT_INC_CARRY" => com.active_hz = carry_step(com.active_hz, 25., COM_BAND_MHZ),
            "_RADIO_FRACT_DEC_CARRY" => com.active_hz = carry_step(com.active_hz, -25., COM_BAND_MHZ),
            "_VOLUME_SET" => com.volume = value.clamp(0., 1.),
            // "by 0.02, clamped between 0 and 1".
            "_VOLUME_INC" => com.volume = (com.volume + 0.02).clamp(0., 1.),
            "_VOLUME_DEC" => com.volume = (com.volume - 0.02).clamp(0., 1.),
            "_RECEIVE_SELECT" => com.receive = data != 0,
            "_TRANSMIT_SELECT" => self.transmitter = i as u32,
            _ => return false,
        }
        true
    }
}

// ---------------------------------------------------------------------------
// The MFD's manual LS tuning.
// ---------------------------------------------------------------------------

/// The ILS frequencies the MFD accepts (FrequencyILSFormat,
/// DataEntryFormats.tsx:1634-1636).
pub const LS_FREQUENCY_MHZ: (f64, f64) = (108., 111.95);

/// FmgcFlightPhase.Approach (fbw-a380x shared flightphase.ts:4-13).
const PHASE_APPROACH: f64 = 5.;

/// NavaidTuner.isMmrTuningLocked (:610-615): no MMR tuning in the approach
/// phase below 700 ft radio height.
pub fn mmr_tuning_locked(flight_phase: f64, radio_height_ft: f64) -> bool {
    flight_phase == PHASE_APPROACH && radio_height_ft < 700.
}

/// NavRadioUtils.vhfFrequenciesAreEqual (NavRadioUtils.ts:5-7), with null as 0.
fn frequencies_equal(a: Option<f64>, b: Option<f64>) -> bool {
    (a.unwrap_or(0.) - b.unwrap_or(0.)).abs() < 0.01
}

/// JavaScript's Math.round of a course, null being 0.
fn js_round(v: Option<f64>) -> f64 {
    (v.unwrap_or(0.) + 0.5).floor()
}

/// The manual LS selection, as NavaidTuner keeps it for both MMRs, and what
/// it last sent (lastMmrFrequencies / lastMmrCourses).
///
/// - A frequency is `setManualIls(freq)` (:686-723): both MMRs manual on that
///   frequency, their course cleared. Out of the MFD's range it is refused, as
///   the entry format refuses it. Zero clears it (`setManualIls(null)`).
/// - A course is `setIlsCourse(course)` (:730-736): `course % 360`, manual; a
///   negative course clears it back to the database course, which a
///   frequency-only selection does not have. The MFD takes no back course yet
///   (DataEntryFormats.tsx:1776-1777).
/// - Each tick, as `updateNavaidSelection` and the tune calls do (:436-460,
///   515-554): unless MMR tuning is locked, a frequency not already sent goes
///   out as `NAV3_RADIO_SET_HZ` and `NAV4_RADIO_SET_HZ`, a course whose
///   rounded value changed as `VOR3_SET` and `VOR4_SET`; the back beam
///   outputs are cleared.
/// - `A32NX_FM_LS_COURSE`, which the PRIMs' ILS bus takes the runway heading
///   from (FlyByWireInterface.cpp:1350), follows `updateIlsCourse`
///   (FmcAircraftInterface.ts:1663-1672): the course, else NAV LOCALIZER:3
///   while a frequency is tuned and the receiver's localizer is valid, else
///   -1. The FMC writes it only with an active flight plan; with no FMC
///   running it is written while a manual selection exists, and once more
///   when it is cleared.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LsTuning {
    pub frequency_mhz: Option<f64>,
    pub course_deg: Option<f64>,
    last_frequency: Option<f64>,
    last_course: Option<f64>,
    /// A selection was cleared and the course output not yet reset.
    cleared: bool,
}

impl LsTuning {
    pub fn is_manual(&self) -> bool {
        self.frequency_mhz.is_some() || self.course_deg.is_some()
    }

    /// Returns whether the frequency was taken.
    pub fn set_frequency(&mut self, mhz: f64) -> bool {
        if mhz <= 0. {
            self.clear();
            return true;
        }
        if !(LS_FREQUENCY_MHZ.0..=LS_FREQUENCY_MHZ.1 + 1e-9).contains(&mhz) {
            return false;
        }
        self.frequency_mhz = Some((mhz * 100.).round() / 100.);
        self.course_deg = None;
        true
    }

    pub fn set_course(&mut self, course: f64) {
        if course < 0. {
            self.course_deg = None;
            if !self.is_manual() {
                self.cleared = true;
            }
        } else {
            self.course_deg = Some(course.min(360.) % 360.);
        }
    }

    pub fn clear(&mut self) {
        if self.is_manual() {
            self.cleared = true;
        }
        self.frequency_mhz = None;
        self.course_deg = None;
    }

    /// The events this tick sends, in NavaidTuner's order.
    pub fn tuning_events(&mut self, locked: bool) -> Vec<(&'static str, f64)> {
        let mut events = Vec::new();
        if locked || !(self.is_manual() || self.last_frequency.is_some() || self.last_course.is_some()) {
            return events;
        }
        if !frequencies_equal(self.last_frequency, self.frequency_mhz) {
            self.last_frequency = self.frequency_mhz;
            let hz = self.frequency_mhz.unwrap_or(0.) * MHZ;
            events.push(("NAV3_RADIO_SET_HZ", hz));
            events.push(("NAV4_RADIO_SET_HZ", hz));
        }
        if js_round(self.last_course) != js_round(self.course_deg) {
            self.last_course = self.course_deg;
            let course = self.course_deg.unwrap_or(0.);
            events.push(("VOR3_SET", course));
            events.push(("VOR4_SET", course));
        }
        events
    }

    /// `A32NX_FM_LS_COURSE`, when this tick writes it.
    pub fn ls_course_output(&mut self, loc_valid: bool, nav_localizer_deg: f64) -> Option<f64> {
        if self.is_manual() {
            Some(match (self.course_deg, self.frequency_mhz) {
                (Some(course), _) => course,
                (None, Some(_)) if loc_valid => nav_localizer_deg,
                _ => -1.,
            })
        } else if std::mem::take(&mut self.cleared) {
            Some(-1.)
        } else {
            None
        }
    }
}

// ---------------------------------------------------------------------------
// X-Plane.
// ---------------------------------------------------------------------------

/// What the study page shows beside the variables: the idents, which are
/// strings.
#[derive(Clone, Debug, Default)]
pub struct Display {
    pub nav_ident: [String; 4],
    pub adf_ident: [String; 2],
    pub ls: LsTuning,
}

static DISPLAY: Mutex<Option<Display>> = Mutex::new(None);

pub fn display() -> Display {
    DISPLAY.lock().ok().and_then(|d| d.clone()).unwrap_or_default()
}

/// The LS tuning's datarefs and commands, for the study page's controls.
#[derive(Clone, Copy, Debug)]
pub struct LsControls {
    pub frequency: Value,
    pub course: Value,
    pub frequency_up: Command,
    pub frequency_down: Command,
    pub frequency_up_coarse: Command,
    pub frequency_down_coarse: Command,
    pub course_up: Command,
    pub course_down: Command,
    pub course_up_coarse: Command,
    pub course_down_coarse: Command,
    pub clear: Command,
}

static LS_CONTROLS: Mutex<Option<LsControls>> = Mutex::new(None);

pub fn ls_controls() -> Option<LsControls> {
    LS_CONTROLS.lock().ok().and_then(|c| *c)
}

/// One X-Plane int or float array element or scalar, written only when the
/// value it should hold changed; a change X-Plane made itself since is read
/// back first.
#[derive(Clone, Copy)]
struct Follow {
    written: Option<f64>,
}

impl Follow {
    const NEW: Self = Self { written: None };

    /// Returns the value X-Plane holds now if someone else changed it.
    fn changed_elsewhere(&self, xp_value: f64, tolerance: f64) -> Option<f64> {
        self.written.filter(|w| (w - xp_value).abs() > tolerance).map(|_| xp_value)
    }
}

struct Refs {
    nav_frequency: Option<DataRef>,
    nav_standby: Option<DataRef>,
    obs_pilot: Option<DataRef>,
    obs_copilot: Option<DataRef>,
    nav_course: Option<DataRef>,
    has_dme: Option<DataRef>,
    dme: Option<DataRef>,
    relative_bearing: Option<DataRef>,
    bearing: Option<DataRef>,
    hdef: Option<DataRef>,
    horizontal: Option<DataRef>,
    nav_type: Option<DataRef>,
    from_to: Option<DataRef>,
    nav_id: [Option<DataRef>; 4],
    nav_volume: [Option<DataRef>; 4],
    nav_audio: [Option<DataRef>; 4],
    adf_frequency: [Option<DataRef>; 2],
    adf_standby: [Option<DataRef>; 2],
    adf_bearing: [Option<DataRef>; 2],
    adf_id: [Option<DataRef>; 2],
    adf_volume: [Option<DataRef>; 2],
    adf_audio: [Option<DataRef>; 2],
    com_frequency: [Option<DataRef>; 2],
    com_standby: [Option<DataRef>; 2],
    com_volume: [Option<DataRef>; 2],
    com_audio: [Option<DataRef>; 2],
    com_selection: Option<DataRef>,
    marker_audio: Option<DataRef>,
    markers: [Option<DataRef>; 3],
    psi: Option<DataRef>,
    mag_psi: Option<DataRef>,
    radio_height: Option<DataRef>,
    latitude: Option<DataRef>,
    longitude: Option<DataRef>,
}

impl Refs {
    fn new(xplm: &Xplm) -> Self {
        let f = |n: &str| xplm.find(n);
        let a = |n: &str| xplm.find(&format!("sim/cockpit2/radios/actuators/{n}"));
        let i = |n: &str| xplm.find(&format!("sim/cockpit2/radios/indicators/{n}"));
        Self {
            nav_frequency: a("nav_frequency_hz"),
            nav_standby: a("nav_standby_frequency_hz"),
            obs_pilot: a("nav_obs_deg_mag_pilot"),
            obs_copilot: a("nav_obs_deg_mag_copilot"),
            nav_course: a("nav_course_deg_mag_pilot"),
            has_dme: i("nav_has_dme"),
            dme: i("nav_dme_distance_nm"),
            relative_bearing: i("nav_relative_bearing_deg"),
            bearing: i("nav_bearing_deg_mag"),
            hdef: i("nav_hdef_dots_pilot"),
            horizontal: i("nav_display_horizontal"),
            nav_type: i("nav_type"),
            from_to: i("nav_flag_from_to_pilot"),
            nav_id: [1, 2, 3, 4].map(|n| i(&format!("nav{n}_nav_id"))),
            nav_volume: [1, 2, 3, 4].map(|n| a(&format!("audio_volume_nav{n}"))),
            nav_audio: [1, 2, 3, 4].map(|n| a(&format!("audio_selection_nav{n}"))),
            adf_frequency: [1, 2].map(|n| a(&format!("adf{n}_frequency_hz"))),
            adf_standby: [1, 2].map(|n| a(&format!("adf{n}_standby_frequency_hz"))),
            adf_bearing: [1, 2].map(|n| i(&format!("adf{n}_relative_bearing_deg"))),
            adf_id: [1, 2].map(|n| i(&format!("adf{n}_nav_id"))),
            adf_volume: [1, 2].map(|n| a(&format!("audio_volume_adf{n}"))),
            adf_audio: [1, 2].map(|n| a(&format!("audio_selection_adf{n}"))),
            com_frequency: [1, 2].map(|n| a(&format!("com{n}_frequency_hz_833"))),
            com_standby: [1, 2].map(|n| a(&format!("com{n}_standby_frequency_hz_833"))),
            com_volume: [1, 2].map(|n| a(&format!("audio_volume_com{n}"))),
            com_audio: [1, 2].map(|n| a(&format!("audio_selection_com{n}"))),
            com_selection: a("audio_com_selection"),
            marker_audio: a("audio_marker_enabled"),
            markers: ["over_outer_marker", "over_middle_marker", "over_inner_marker"].map(i),
            psi: f("sim/flightmodel/position/psi"),
            mag_psi: f("sim/flightmodel/position/mag_psi"),
            radio_height: f("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot"),
            latitude: f("sim/flightmodel/position/latitude"),
            longitude: f("sim/flightmodel/position/longitude"),
        }
    }
}

/// The earth's mean radius in nautical miles, for the great-circle
/// destination-point formula below.
const EARTH_RADIUS_NM: f64 = 3_440.065;

/// A VOR/DME station's lat/lon, triangulated from the aircraft's own
/// position and what its receiver already measures: the true bearing to the
/// station (`nav_bearing_deg_mag` minus magnetic variation) and its slant
/// range (`nav_dme_distance_nm`). X-Plane has no per-receiver station
/// position dataref, so this reconstructs it the way a real receiver's own
/// signals would place the station — the standard spherical
/// destination-point formula (Ed Williams' Aviation Formulary, "Lat/lon
/// given radial and distance"). This is what FlyByWire's `NAV VOR LATLONALT`
/// read wants (VorBusPublisher.ts's `nav1Location`..`nav4Location`, type
/// `LLA`) and what the ND's raw-nav-data provider inside `@microsoft/msfs-sdk`
/// also reads (`nav_lla`, msfssdk.js:10573).
fn destination_point(lat1_deg: f64, lon1_deg: f64, true_bearing_deg: f64, distance_nm: f64) -> (f64, f64) {
    let (lat1, lon1, brng) = (lat1_deg.to_radians(), lon1_deg.to_radians(), true_bearing_deg.to_radians());
    let d = distance_nm / EARTH_RADIUS_NM;
    let lat2 = (lat1.sin() * d.cos() + lat1.cos() * d.sin() * brng.cos()).asin();
    let lon2 = lon1 + (brng.sin() * d.sin() * lat1.cos()).atan2(d.cos() - lat1.sin() * lat2.sin());
    (lat2.to_degrees(), normalise_180(lon2.to_degrees()))
}

/// The simulator variables this module feeds.
struct Ids {
    nav_active: [VariableIdentifier; 4],
    nav_frequency_3: VariableIdentifier,
    nav_standby: [VariableIdentifier; 4],
    nav_obs: [VariableIdentifier; 4],
    has_nav: [VariableIdentifier; 4],
    relative_bearing: [VariableIdentifier; 4],
    localizer: [VariableIdentifier; 4],
    /// Receivers 1, 2 and 4: 3 is sensors.rs's.
    has_dme: [VariableIdentifier; 3],
    dme: [VariableIdentifier; 3],
    radial_error: [VariableIdentifier; 3],
    magvar: [VariableIdentifier; 3],
    adf_active: [VariableIdentifier; 2],
    adf_radial: [VariableIdentifier; 2],
    com_active: [VariableIdentifier; 3],
    com_standby: [VariableIdentifier; 3],
    marker_sound: VariableIdentifier,
    marker_state: VariableIdentifier,
    nav_volume: [VariableIdentifier; 4],
    nav_sound: [VariableIdentifier; 4],
    nav_to_from: [VariableIdentifier; 4],
    adf_volume: [VariableIdentifier; 2],
    adf_sound: [VariableIdentifier; 2],
    com_volume: [VariableIdentifier; 3],
    com_receive: [VariableIdentifier; 3],
    com_transmit: [VariableIdentifier; 3],
    com_receive_all: VariableIdentifier,
    // FlyByWire's own.
    flight_phase: VariableIdentifier,
    loc_valid: VariableIdentifier,
    ls_course: VariableIdentifier,
    backbeam: [VariableIdentifier; 2],
    /// A32NX_RA_1/2/3_RADIO_ALTITUDE, the ARINC word the PRIMs read
    /// (prim.rs:631) that nothing in production ever wrote before this.
    radio_altitude: [VariableIdentifier; 3],
}

/// The receivers other than 3 whose DME, radial error and variation this
/// module feeds, zero-based.
const OWN_RECEIVERS: [usize; 3] = [0, 1, 3];

impl Ids {
    fn new(vars: &mut Vars) -> Self {
        let mut get = |n: String| vars.get(n);
        let nav = |get: &mut dyn FnMut(String) -> VariableIdentifier, what: &str| [1, 2, 3, 4].map(|n| get(format!("{what}:{n}")));
        let own = |get: &mut dyn FnMut(String) -> VariableIdentifier, what: &str| OWN_RECEIVERS.map(|i| get(format!("{what}:{}", i + 1)));
        Self {
            nav_active: nav(&mut get, "NAV ACTIVE FREQUENCY"),
            nav_frequency_3: get("NAV FREQUENCY:3".into()),
            nav_standby: nav(&mut get, "NAV STANDBY FREQUENCY"),
            nav_obs: nav(&mut get, "NAV OBS"),
            has_nav: nav(&mut get, "NAV HAS NAV"),
            relative_bearing: nav(&mut get, "NAV RELATIVE BEARING TO STATION"),
            localizer: nav(&mut get, "NAV LOCALIZER"),
            has_dme: own(&mut get, "NAV HAS DME"),
            dme: own(&mut get, "NAV DME"),
            radial_error: own(&mut get, "NAV RADIAL ERROR"),
            magvar: own(&mut get, "NAV MAGVAR"),
            adf_active: [1, 2].map(|n| get(format!("ADF ACTIVE FREQUENCY:{n}"))),
            adf_radial: [1, 2].map(|n| get(format!("ADF RADIAL:{n}"))),
            com_active: [1, 2, 3].map(|n| get(format!("COM ACTIVE FREQUENCY:{n}"))),
            com_standby: [1, 2, 3].map(|n| get(format!("COM STANDBY FREQUENCY:{n}"))),
            marker_sound: get("MARKER SOUND".into()),
            marker_state: get("MARKER BEACON STATE".into()),
            nav_volume: nav(&mut get, "NAV VOLUME"),
            nav_sound: nav(&mut get, "NAV SOUND"),
            nav_to_from: nav(&mut get, "NAV TOFROM"),
            adf_volume: [1, 2].map(|n| get(format!("ADF VOLUME:{n}"))),
            adf_sound: [1, 2].map(|n| get(format!("ADF SOUND:{n}"))),
            com_volume: [1, 2, 3].map(|n| get(format!("COM VOLUME:{n}"))),
            com_receive: [1, 2, 3].map(|n| get(format!("COM RECEIVE:{n}"))),
            com_transmit: [1, 2, 3].map(|n| get(format!("COM TRANSMIT:{n}"))),
            com_receive_all: get("COM RECEIVE ALL".into()),
            flight_phase: get("FMGC_FLIGHT_PHASE".into()),
            loc_valid: get("RADIO_RECEIVER_LOC_IS_VALID".into()),
            ls_course: get("FM_LS_COURSE".into()),
            backbeam: [1, 2].map(|n| get(format!("FM{n}_BACKBEAM_SELECTED"))),
            radio_altitude: [1, 2, 3].map(|n| get(format!("A32NX_RA_{n}_RADIO_ALTITUDE"))),
        }
    }
}

// ---------------------------------------------------------------------------
// Radio altimeters 1-3.
// ---------------------------------------------------------------------------

/// ARINC 429 SSM as FlyByWire packs it (prim.rs's own `SSM_NCD`/`SSM_NO`/
/// `SSM_FT`, Arinc429Utils.cpp): 0 failure warning, 1 no computed data, 2
/// functional test, 3 normal operation.
const RA_SSM_FAILURE_WARNING: u32 = 0;
const RA_SSM_NCD: u32 = 1;
const RA_SSM_NORMAL: u32 = 3;

/// The radio altimeter's own range: above it (or with no return) the receiver
/// has nothing to report, same as a real one going NCD out of range (A380
/// RA range is 2500 ft radio height, matching autoland_warning_condition's
/// and the flare law's own operating range in this plugin).
pub const RA_RANGE_FT: f64 = 2500.;

/// One radio altimeter's word, from X-Plane's own probe (there is one
/// terrain-return height per aircraft here, `radio_altimeter_height_ft_pilot`,
/// fanned out to all three RAs since X-Plane exposes no separate antenna per
/// RA) and that RA's own power state. Unpowered is SSM failure-warning, not
/// merely NCD, since an unpowered ARINC transmitter drives its output low
/// rather than tagging it no-computed-data (prim.rs's `SSM_NCD`/`SSM_NO`
/// comment on the ILS bus draws the same distinction for a missing signal
/// versus a missing receiver).
pub fn radio_altimeter_word(height_ft: Option<f64>, powered: bool) -> crate::fbw_types::BaseArinc429 {
    if !powered {
        return crate::fbw_types::BaseArinc429 { SSM: RA_SSM_FAILURE_WARNING, Data: 0. };
    }
    match height_ft {
        Some(h) if (0. ..=RA_RANGE_FT).contains(&h) => {
            crate::fbw_types::BaseArinc429 { SSM: RA_SSM_NORMAL, Data: h as f32 }
        }
        _ => crate::fbw_types::BaseArinc429 { SSM: RA_SSM_NCD, Data: 0. },
    }
}

/// The radios.
pub struct Radios {
    receivers: Receivers,
    ls: LsTuning,
    refs: Refs,
    ids: Ids,
    nav_frequency: [Follow; 4],
    nav_standby: [Follow; 4],
    obs: [Follow; 4],
    adf_frequency: [Follow; 2],
    adf_standby: [Follow; 2],
    com_frequency: [Follow; 2],
    com_standby: [Follow; 2],
    marker_audio: Follow,
    /// Unregistered with the plugin, when it is dropped.
    _published: Published,
    ls_controls: LsControls,
    nav_ident: [Value; 4],
    adf_ident: [Value; 2],
    nav_lat: [Value; 4],
    nav_lon: [Value; 4],
    ls_manual: Value,
    ls_ident: Value,
}

impl Radios {
    pub fn new(vars: &mut Vars, xplm: &Xplm) -> Self {
        let mut p = Published::default();
        let ls_controls = LsControls {
            frequency: p.number("fbw/radio/ls/frequency_mhz", 0., true),
            course: p.number("fbw/radio/ls/course_deg", -1., true),
            frequency_up: p.command("fbw/radio/ls/frequency_up", "LS frequency up 0.05 MHz"),
            frequency_down: p.command("fbw/radio/ls/frequency_down", "LS frequency down 0.05 MHz"),
            frequency_up_coarse: p.command("fbw/radio/ls/frequency_up_coarse", "LS frequency up 1 MHz"),
            frequency_down_coarse: p.command("fbw/radio/ls/frequency_down_coarse", "LS frequency down 1 MHz"),
            course_up: p.command("fbw/radio/ls/course_up", "LS course up 1 degree"),
            course_down: p.command("fbw/radio/ls/course_down", "LS course down 1 degree"),
            course_up_coarse: p.command("fbw/radio/ls/course_up_coarse", "LS course up 10 degrees"),
            course_down_coarse: p.command("fbw/radio/ls/course_down_coarse", "LS course down 10 degrees"),
            clear: p.command("fbw/radio/ls/clear", "Clear the manual LS selection"),
        };
        if let Ok(mut c) = LS_CONTROLS.lock() {
            *c = Some(ls_controls);
        }
        let nav_ident = [1, 2, 3, 4].map(|n| p.text(&format!("fbw/radio/nav{n}/ident"), 8, false));
        let adf_ident = [1, 2].map(|n| p.text(&format!("fbw/radio/adf{n}/ident"), 8, false));
        // NAV VOR LATLONALT:n, served through app/js and src/js's
        // msfs/simvar.js `struct()` (see destination_point's doc comment).
        let nav_lat = [1, 2, 3, 4].map(|n| p.number(&format!("fbw/radio/nav{n}/lat"), 0., false));
        let nav_lon = [1, 2, 3, 4].map(|n| p.number(&format!("fbw/radio/nav{n}/lon"), 0., false));
        let ls_manual = p.number("fbw/radio/ls/manual", 0., false);
        let ls_ident = p.text("fbw/radio/ls/ident", 8, false);
        let mut radios = Self {
            receivers: Receivers::default(),
            ls: LsTuning::default(),
            refs: Refs::new(xplm),
            ids: Ids::new(vars),
            nav_frequency: [Follow::NEW; 4],
            nav_standby: [Follow::NEW; 4],
            obs: [Follow::NEW; 4],
            adf_frequency: [Follow::NEW; 2],
            adf_standby: [Follow::NEW; 2],
            com_frequency: [Follow::NEW; 2],
            com_standby: [Follow::NEW; 2],
            marker_audio: Follow::NEW,
            _published: p,
            ls_controls,
            nav_ident,
            adf_ident,
            nav_lat,
            nav_lon,
            ls_manual,
            ls_ident,
        };
        // The radios start where X-Plane has them.
        radios.take_from_xplane(xplm, true);
        radios
    }

    /// An MSFS key event from a script or a cockpit control. Returns whether
    /// it was a radio event.
    #[cfg_attr(not(feature = "js"), allow(dead_code))]
    pub fn handle_event(&mut self, name: &str, value: f64, xplm: &Xplm) -> bool {
        if !self.receivers.apply(name, value) {
            return false;
        }
        self.write_to_xplane(xplm);
        true
    }

    fn ints(xplm: &Xplm, d: Option<DataRef>) -> [i32; 12] {
        let mut v = [0; 12];
        if let Some(d) = d {
            xplm.get_vi(d, &mut v);
        }
        v
    }

    fn floats(xplm: &Xplm, d: Option<DataRef>) -> [f32; 12] {
        let mut v = [0.; 12];
        if let Some(d) = d {
            xplm.get_vf(d, &mut v);
        }
        v
    }

    /// X-Plane's frequencies and courses, where they differ from what this
    /// module last wrote (or all of them at start).
    fn take_from_xplane(&mut self, xplm: &Xplm, all: bool) {
        let take = |follow: &Follow, xp: f64, tolerance: f64, into: &mut f64, scale: f64| {
            let now = if all { Some(xp) } else { follow.changed_elsewhere(xp, tolerance) };
            if let Some(v) = now {
                *into = v * scale;
            }
        };
        let r = &self.refs;
        let (active, standby, obs) = (Self::ints(xplm, r.nav_frequency), Self::ints(xplm, r.nav_standby), Self::floats(xplm, r.obs_pilot));
        for i in 0..4 {
            let nav = &mut self.receivers.nav[i];
            if r.nav_frequency.is_some() {
                take(&self.nav_frequency[i], active[i] as f64, 0.5, &mut nav.active_hz, 10_000.);
            }
            if r.nav_standby.is_some() {
                take(&self.nav_standby[i], standby[i] as f64, 0.5, &mut nav.standby_hz, 10_000.);
            }
            if r.obs_pilot.is_some() {
                take(&self.obs[i], obs[i] as f64, 0.01, &mut nav.obs_deg, 1.);
            }
        }
        for i in 0..2 {
            let adf = &mut self.receivers.adf[i];
            if let Some(d) = r.adf_frequency[i] {
                take(&self.adf_frequency[i], xplm.get_i(d) as f64, 0.5, &mut adf.active_hz, KHZ);
            }
            if let Some(d) = r.adf_standby[i] {
                take(&self.adf_standby[i], xplm.get_i(d) as f64, 0.5, &mut adf.standby_hz, KHZ);
            }
            let com = &mut self.receivers.com[i];
            if let Some(d) = r.com_frequency[i] {
                take(&self.com_frequency[i], xplm.get_i(d) as f64, 0.5, &mut com.active_hz, KHZ);
            }
            if let Some(d) = r.com_standby[i] {
                take(&self.com_standby[i], xplm.get_i(d) as f64, 0.5, &mut com.standby_hz, KHZ);
            }
        }
        if let Some(d) = r.marker_audio {
            let mut sound = self.receivers.marker_sound as i32 as f64;
            take(&self.marker_audio, xplm.get_i(d) as f64, 0.5, &mut sound, 1.);
            self.receivers.marker_sound = sound != 0.;
        }
    }

    fn write_to_xplane(&mut self, xplm: &Xplm) {
        let r = &self.refs;
        let rx = &self.receivers;
        let int_at = |d: Option<DataRef>, follow: &mut Follow, i: usize, v: f64| {
            if let (Some(d), false) = (d, follow.written == Some(v)) {
                xplm.set_vi_at(d, i, v as i32);
                follow.written = Some(v);
            }
        };
        let int = |d: Option<DataRef>, follow: &mut Follow, v: f64| {
            if let (Some(d), false) = (d, follow.written == Some(v)) {
                xplm.set_i(d, v as i32);
                follow.written = Some(v);
            }
        };
        for i in 0..4 {
            let nav = &rx.nav[i];
            // X-Plane's NAV radios tune in 10 kHz.
            int_at(r.nav_frequency, &mut self.nav_frequency[i], i, (nav.active_hz / 10_000.).round());
            int_at(r.nav_standby, &mut self.nav_standby[i], i, (nav.standby_hz / 10_000.).round());
            if self.obs[i].written != Some(nav.obs_deg) {
                for d in [r.obs_pilot, r.obs_copilot].into_iter().flatten() {
                    xplm.set_vf_at(d, i, nav.obs_deg as f32);
                }
                self.obs[i].written = r.obs_pilot.map(|_| nav.obs_deg);
            }
            if let Some(d) = r.nav_volume[i] {
                xplm.set_f(d, (nav.volume / 100.) as f32);
            }
            if let Some(d) = r.nav_audio[i] {
                xplm.set_i(d, nav.ident_on as i32);
            }
        }
        for i in 0..2 {
            let adf = &rx.adf[i];
            int(r.adf_frequency[i], &mut self.adf_frequency[i], (adf.active_hz / KHZ).floor());
            int(r.adf_standby[i], &mut self.adf_standby[i], (adf.standby_hz / KHZ).floor());
            if let Some(d) = r.adf_volume[i] {
                xplm.set_f(d, (adf.volume / 100.) as f32);
            }
            if let Some(d) = r.adf_audio[i] {
                xplm.set_i(d, adf.ident_on as i32);
            }
            let com = &rx.com[i];
            int(r.com_frequency[i], &mut self.com_frequency[i], (com.active_hz / KHZ).round());
            int(r.com_standby[i], &mut self.com_standby[i], (com.standby_hz / KHZ).round());
            if let Some(d) = r.com_volume[i] {
                xplm.set_f(d, com.volume as f32);
            }
            if let Some(d) = r.com_audio[i] {
                xplm.set_i(d, com.receive as i32);
            }
        }
        // X-Plane's selector knows COM1 (6) and COM2 (7) only.
        if let (Some(d), Some(v)) = (r.com_selection, [6, 7].get(rx.transmitter as usize)) {
            xplm.set_i(d, *v);
        }
        int(r.marker_audio, &mut self.marker_audio, rx.marker_sound as i32 as f64);
    }

    /// Before the systems: X-Plane's changes in, the LS tuning out, the
    /// simulator variables filled.
    pub fn update(&mut self, vars: &mut Vars, xplm: &Xplm) {
        self.take_from_xplane(xplm, false);
        self.update_ls(vars, xplm);
        self.write_to_xplane(xplm);
        self.feed_variables(vars, xplm);
    }

    fn update_ls(&mut self, vars: &mut Vars, xplm: &Xplm) {
        let c = self.ls_controls;
        if let Some(mhz) = published::take(c.frequency) {
            if !self.ls.set_frequency(mhz) {
                crate::log(&format!("LS frequency {mhz:.2} refused: the MFD takes 108.00 to 111.95 MHz"));
            }
        }
        if let Some(course) = published::take(c.course) {
            self.ls.set_course(course);
        }
        let nav3_mhz = self.receivers.nav[2].active_hz / MHZ;
        for (command, step) in [
            (c.frequency_up, 0.05),
            (c.frequency_down, -0.05),
            (c.frequency_up_coarse, 1.),
            (c.frequency_down_coarse, -1.),
        ] {
            let n = published::presses(command);
            if n > 0 {
                let from = self.ls.frequency_mhz.unwrap_or(nav3_mhz);
                let target = from + n as f64 * step;
                self.ls.set_frequency(target.clamp(LS_FREQUENCY_MHZ.0, LS_FREQUENCY_MHZ.1));
            }
        }
        for (command, step) in [(c.course_up, 1.), (c.course_down, -1.), (c.course_up_coarse, 10.), (c.course_down_coarse, -10.)] {
            let n = published::presses(command);
            if n > 0 {
                let from = self.ls.course_deg.unwrap_or(self.receivers.nav[2].obs_deg);
                self.ls.set_course(normalise_360(from.round() + n as f64 * step));
            }
        }
        if published::presses(c.clear) > 0 {
            self.ls.clear();
        }

        let height = self.refs.radio_height.map_or(f64::INFINITY, |d| xplm.get_f(d) as f64);
        let locked = mmr_tuning_locked(vars.read(&self.ids.flight_phase), height);
        let manual = self.ls.is_manual();
        for (event, value) in self.ls.tuning_events(locked) {
            self.receivers.apply(event, value);
        }
        if manual {
            for id in self.ids.backbeam {
                vars.write(&id, 0.);
            }
        }
        let loc_valid = vars.read(&self.ids.loc_valid) == 1.;
        let localizer = vars.read(&self.ids.localizer[2]);
        if let Some(course) = self.ls.ls_course_output(loc_valid, localizer) {
            vars.write(&self.ids.ls_course, course);
        }
    }

    fn feed_variables(&mut self, vars: &mut Vars, xplm: &Xplm) {
        let r = &self.refs;
        let rx = &self.receivers;
        let ids = &self.ids;
        let (has_dme, dme, relative, bearing) = (
            Self::ints(xplm, r.has_dme),
            Self::floats(xplm, r.dme),
            Self::floats(xplm, r.relative_bearing),
            Self::floats(xplm, r.bearing),
        );
        let from_to = Self::ints(xplm, r.from_to);
        let (hdef, horizontal, nav_type, course) = (
            Self::floats(xplm, r.hdef),
            Self::ints(xplm, r.horizontal),
            Self::ints(xplm, r.nav_type),
            Self::floats(xplm, r.nav_course),
        );
        let magvar = match (r.psi, r.mag_psi) {
            (Some(p), Some(m)) => magnetic_minus_true(xplm.get_f(p) as f64, xplm.get_f(m) as f64),
            _ => 0.,
        };
        for i in 0..4 {
            let nav = &rx.nav[i];
            let localizer = is_localizer_frequency((nav.active_hz / 10_000.).round() as i32) && horizontal[i] != 0;
            vars.write_from_xplane(&ids.nav_active[i], nav.active_hz / MHZ);
            vars.write_from_xplane(&ids.nav_standby[i], nav.standby_hz / MHZ);
            vars.write_from_xplane(&ids.nav_obs[i], nav.obs_deg);
            vars.write_from_xplane(&ids.has_nav[i], (nav_type[i] != 0) as i32 as f64);
            vars.write_from_xplane(&ids.relative_bearing[i], relative[i] as f64);
            vars.write_from_xplane(&ids.localizer[i], if localizer { course[i] as f64 } else { 0. });
            vars.write_from_xplane(&ids.nav_volume[i], nav.volume);
            vars.write_from_xplane(&ids.nav_sound[i], nav.ident_on as i32 as f64);
            vars.write_from_xplane(&ids.nav_to_from[i], from_to[i] as f64);
            if let Some(k) = OWN_RECEIVERS.iter().position(|&o| o == i) {
                vars.write_from_xplane(&ids.has_dme[k], (has_dme[i] != 0) as i32 as f64);
                vars.write_from_xplane(&ids.dme[k], dme[i] as f64);
                // A localizer's error at FBW's PFD scale, as sensors.rs's :3;
                // a VOR's radial (from the station) minus the OBS.
                let error = if localizer {
                    hdef[i] as f64 * LOC_DEG_PER_DOT
                } else {
                    normalise_180(bearing[i] as f64 + 180. - nav.obs_deg)
                };
                vars.write_from_xplane(&ids.radial_error[k], error);
                vars.write_from_xplane(&ids.magvar[k], magvar);
            }
        }
        // NAV VOR LATLONALT:n, for the ND: only where a station is actually
        // received and its DME (needed for the range leg) is in.
        if let (Some(la), Some(lo)) = (r.latitude, r.longitude) {
            let (lat, lon) = (xplm.get_d(la), xplm.get_d(lo));
            for i in 0..4 {
                if nav_type[i] != 0 && has_dme[i] != 0 {
                    let true_bearing = normalise_360(bearing[i] as f64 - magvar);
                    let (slat, slon) = destination_point(lat, lon, true_bearing, dme[i] as f64);
                    published::set(self.nav_lat[i], slat);
                    published::set(self.nav_lon[i], slon);
                }
            }
        }
        // Radio altimeters: NOT written here. Correction from this session's
        // earlier (wrong) finding: `A32NX_RA_n_RADIO_ALTITUDE` already has a
        // real production writer — it is not this plugin's job at all, but
        // the linked `a380_systems` crate's own `Ala52BRadioAltimeter`
        // (fbw-common/src/wasm/systems/systems/src/navigation/ala52b.rs),
        // wired into `A380RadioAltimeters` (fbw-a380x .../navigation.rs)
        // with a real `powered_by: ElectricalBusType` and antenna/transceiver
        // model, run every `Simulation::tick()` as part of the A380's own
        // SimulationElement tree. Writing to the same variable from here
        // would fight that real simulation rather than complete it. What
        // *may* still be missing (not confirmed this session, ran out of
        // time): whether the `TransceiverPair` each `Ala52BRadioAltimeter`
        // reads its terrain response from is actually fed a real
        // height-above-terrain in this X-Plane port, or returns `None`
        // always — check `ala52b.rs`'s `TransceiverPair` consumer and how/if
        // `a380_systems_wasm` or this plugin supplies one on X-Plane, not
        // whether the ARINC word itself gets written.
        vars.write_from_xplane(&ids.nav_frequency_3, rx.nav[2].active_hz / MHZ);
        for i in 0..2 {
            vars.write_from_xplane(&ids.adf_active[i], rx.adf[i].active_hz / KHZ);
            let radial = r.adf_bearing[i].map_or(0., |d| xplm.get_f(d) as f64);
            vars.write_from_xplane(&ids.adf_radial[i], radial);
            vars.write_from_xplane(&ids.adf_volume[i], rx.adf[i].volume / 100.);
            vars.write_from_xplane(&ids.adf_sound[i], rx.adf[i].ident_on as i32 as f64);
        }
        for i in 0..3 {
            vars.write_from_xplane(&ids.com_active[i], hz_to_bcd32(rx.com[i].active_hz) as f64);
            vars.write_from_xplane(&ids.com_standby[i], hz_to_bcd32(rx.com[i].standby_hz) as f64);
            vars.write_from_xplane(&ids.com_volume[i], rx.com[i].volume * 100.);
            vars.write_from_xplane(&ids.com_receive[i], rx.com[i].receive as i32 as f64);
            vars.write_from_xplane(&ids.com_transmit[i], (rx.transmitter == i as u32) as i32 as f64);
        }
        vars.write_from_xplane(&ids.com_receive_all, rx.com.iter().all(|c| c.receive) as i32 as f64);
        vars.write_from_xplane(&ids.marker_sound, rx.marker_sound as i32 as f64);
        let over = r.markers.map(|d| d.is_some_and(|d| xplm.get_i(d) != 0));
        let state = if over[2] {
            3.
        } else if over[1] {
            2.
        } else if over[0] {
            1.
        } else {
            0.
        };
        vars.write_from_xplane(&ids.marker_state, state);

        // Idents and the LS selection, for anything in X-Plane and the page.
        let nav_ident = r.nav_id.map(|d| d.map(|d| xplm.get_text(d, 150)).unwrap_or_default());
        let adf_ident = r.adf_id.map(|d| d.map(|d| xplm.get_text(d, 150)).unwrap_or_default());
        for (v, s) in self.nav_ident.iter().zip(&nav_ident) {
            published::set_text(*v, s);
        }
        for (v, s) in self.adf_ident.iter().zip(&adf_ident) {
            published::set_text(*v, s);
        }
        published::set_text(self.ls_ident, &nav_ident[2]);
        let c = self.ls_controls;
        published::set(c.frequency, self.ls.frequency_mhz.unwrap_or(rx.nav[2].active_hz / MHZ));
        published::set(c.course, self.ls.course_deg.unwrap_or(rx.nav[2].obs_deg));
        published::set(self.ls_manual, self.ls.is_manual() as i32 as f64);
        if let Ok(mut d) = DISPLAY.lock() {
            *d = Some(Display { nav_ident, adf_ident, ls: self.ls.clone() });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn radio_altimeter_word_is_normal_in_range_ncd_beyond_it_and_fw_unpowered() {
        let normal = radio_altimeter_word(Some(500.), true);
        assert_eq!((normal.SSM, normal.Data), (RA_SSM_NORMAL, 500.));
        let out_of_range = radio_altimeter_word(Some(RA_RANGE_FT + 1.), true);
        assert_eq!(out_of_range.SSM, RA_SSM_NCD);
        let no_source = radio_altimeter_word(None, true);
        assert_eq!(no_source.SSM, RA_SSM_NCD);
        let unpowered = radio_altimeter_word(Some(500.), false);
        assert_eq!((unpowered.SSM, unpowered.Data), (RA_SSM_FAILURE_WARNING, 0.));
        // The word round-trips through prim.rs's own encoding.
        let bits = crate::prim::to_simvar(normal);
        assert_eq!(crate::prim::from_simvar(bits).SSM, RA_SSM_NORMAL);
    }

    #[test]
    fn vor_station_position_triangulates_from_bearing_and_dme() {
        // Due north, 60 nm (about 1 degree of latitude) from the equator.
        let (lat, lon) = destination_point(0., 0., 0., 60.);
        assert!((lat - 1.).abs() < 0.01, "{lat}");
        assert!(lon.abs() < 0.01, "{lon}");
        // Due east, 60 nm from 0,0: about 1 degree of longitude there too.
        let (lat2, lon2) = destination_point(0., 0., 90., 60.);
        assert!(lat2.abs() < 0.01, "{lat2}");
        assert!((lon2 - 1.).abs() < 0.01, "{lon2}");
        // No distance: the station is the aircraft's own position.
        let (lat3, lon3) = destination_point(51.5, -0.1, 45., 0.);
        assert!((lat3 - 51.5).abs() < 1e-9 && (lon3 + 0.1).abs() < 1e-9);
    }

    #[test]
    fn bcd_encodings_round_trip_the_way_msfs_packs_them() {
        // 121.500 MHz is 0x1215000 in BCD32 (RadioUtils.packBcd32).
        assert_eq!(hz_to_bcd32(121_500_000.), 0x121_5000);
        assert_eq!(bcd32_to_hz(0x121_5000), 121_500_000.);
        // 8.33 kHz channels keep their kHz: 118.005.
        assert_eq!(bcd32_to_hz(hz_to_bcd32(118_005_000.)), 118_005_000.);
        // make_bcd16(110.30 MHz) = 0x1030.
        assert_eq!(bcd16_to_hz(0x1030), 110_300_000.);
        assert_eq!(bcd16_to_hz(0x2150), 121_500_000.);
        // make_adf_bcd32(350.5 kHz): tenths 3505, digits 0x3505, << 12.
        assert_eq!(hz_to_adf_bcd32(350_500.), 0x3505 << 12);
        assert_eq!(adf_bcd32_to_hz(0x3505 << 12), 350_500.);
        assert_eq!(adf_bcd32_to_hz(hz_to_adf_bcd32(1_750_000.)), 1_750_000.);
    }

    #[test]
    fn event_names_find_their_receiver() {
        assert_eq!(indexed("NAV3_RADIO_SET_HZ", "NAV"), Some((3, "_RADIO_SET_HZ")));
        assert_eq!(indexed("COM_RADIO_SET_HZ", "COM"), Some((1, "_RADIO_SET_HZ")));
        assert_eq!(indexed("ADF_1_INC", "ADF"), Some((1, "_1_INC")));
        assert_eq!(indexed("ADF2_100_DEC", "ADF"), Some((2, "_100_DEC")));
        assert_eq!(indexed("ADF1_RADIO_SWAP", "ADF"), Some((1, "_RADIO_SWAP")));
        assert_eq!(indexed("NAVIGATION", "NAV"), None);
    }

    #[test]
    fn the_fms_tuning_events_set_the_receivers() {
        let mut r = Receivers::default();
        // NavaidTuner.tuneMmrIlsFrequency: NAV3_RADIO_SET_HZ, frequency * 1e6.
        assert!(r.apply("K:NAV3_RADIO_SET_HZ", 110.3 * 1e6));
        assert_eq!(r.nav[2].active_hz, 110_300_000.);
        assert!(r.apply("VOR3_SET", 273.));
        assert_eq!(r.nav[2].obs_deg, 273.);
        // tuneAdf: ADF_COMPLETE_SET with make_adf_bcd32(kHz * 1000).
        assert!(r.apply("ADF2_COMPLETE_SET", hz_to_adf_bcd32(415_000.) as f64));
        assert_eq!(r.adf[1].active_hz, 415_000.);
        // VhfRadio: COM3_RADIO_SET_HZ in Hz.
        assert!(r.apply("COM3_RADIO_SET_HZ", 131_725_000.));
        assert_eq!(r.com[2].active_hz, 131_725_000.);
        assert!(r.apply("NAV1_RADIO_SET", 0x1390 as f64));
        assert_eq!(r.nav[0].active_hz, 113_900_000.);
        assert!(r.apply("RADIO_VOR4_IDENT_SET", 1.));
        assert!(r.nav[3].ident_on);
        assert!(r.apply("PILOT_TRANSMITTER_SET", 1.));
        assert_eq!(r.transmitter, 1);
        assert!(!r.apply("TOGGLE_AIRCRAFT_EXIT", 1.));
        assert!(!r.apply("TACAN1_SET", 1.));
    }

    #[test]
    fn steps_wrap_and_carry_as_the_sdk_describes() {
        let mut r = Receivers::default();
        r.nav[0].active_hz = 117_950_000.;
        r.apply("NAV1_RADIO_FRACT_INC_CARRY", 0.);
        assert_eq!(r.nav[0].active_hz, 108_000_000., "carry wraps around the band");
        r.nav[0].active_hz = 110_975_000.;
        r.apply("NAV1_RADIO_FRACT_INC", 0.);
        assert_eq!(r.nav[0].active_hz, 110_000_000., "no carry: the MHz stays");
        r.apply("NAV1_RADIO_WHOLE_DEC", 0.);
        r.apply("NAV1_RADIO_WHOLE_DEC", 0.);
        r.apply("NAV1_RADIO_WHOLE_DEC", 0.);
        assert_eq!(r.nav[0].active_hz, 117_000_000.);
        r.nav[0].standby_hz = 109_500_000.;
        r.apply("NAV1_RADIO_SWAP", 0.);
        assert_eq!((r.nav[0].active_hz, r.nav[0].standby_hz), (109_500_000., 117_000_000.));

        r.com[1].active_hz = 136_975_000.;
        r.apply("COM2_RADIO_FRACT_INC_CARRY", 0.);
        assert_eq!(r.com[1].active_hz, 118_000_000.);
        r.apply("COM2_RADIO_WHOLE_DEC", 0.);
        assert_eq!(r.com[1].active_hz, 136_000_000.);

        r.nav[1].obs_deg = 355.;
        r.apply("VOR2_OBI_FAST_INC", 0.);
        assert_eq!(r.nav[1].obs_deg, 360.);
        r.apply("VOR2_OBI_INC", 0.);
        assert_eq!(r.nav[1].obs_deg, 1.);

        r.adf[0].active_hz = 399_900.;
        r.apply("ADF_1_INC", 0.);
        assert_eq!(r.adf[0].active_hz, 390_900., "the ones digit wraps alone");
        r.apply("ADF_FRACT_INC_CARRY", 0.);
        assert_eq!(r.adf[0].active_hz, 391_000.);
        r.apply("ADF1_WHOLE_DEC", 0.);
        assert_eq!(r.adf[0].active_hz, 390_000.);
        r.apply("ADF_100_DEC", 0.);
        assert_eq!(r.adf[0].active_hz, 290_000.);

        r.com[0].volume = 0.99;
        r.apply("COM1_VOLUME_INC", 0.);
        assert_eq!(r.com[0].volume, 1.);
    }

    #[test]
    fn manual_ls_tuning_sends_what_the_navaid_tuner_sends() {
        let mut ls = LsTuning::default();
        assert!(ls.tuning_events(false).is_empty(), "nothing selected, nothing sent");
        assert!(!ls.set_frequency(113.1), "the MFD refuses a VOR frequency");
        assert!(ls.set_frequency(109.9));
        let events = ls.tuning_events(false);
        assert_eq!(events.len(), 2, "course null rounds to the 0 already sent: {events:?}");
        assert_eq!(events[0].0, "NAV3_RADIO_SET_HZ");
        assert!((events[1].1 - 109_900_000.).abs() < 1.);
        assert!(ls.tuning_events(false).is_empty(), "sent once");
        // No course yet: the localizer's course once it is received.
        assert_eq!(ls.ls_course_output(false, 0.), Some(-1.));
        assert_eq!(ls.ls_course_output(true, 91.), Some(91.));
        ls.set_course(452.);
        assert_eq!(ls.course_deg, Some(0.), "course % 360, from at most 360");
        ls.set_course(92.);
        assert_eq!(ls.tuning_events(true), vec![], "locked on final approach");
        assert!(mmr_tuning_locked(5., 650.));
        assert!(!mmr_tuning_locked(5., 750.));
        assert!(!mmr_tuning_locked(4., 100.));
        assert_eq!(ls.tuning_events(false), vec![("VOR3_SET", 92.), ("VOR4_SET", 92.)]);
        assert_eq!(ls.ls_course_output(true, 91.), Some(92.));
        ls.clear();
        let events = ls.tuning_events(false);
        assert_eq!(events, vec![("NAV3_RADIO_SET_HZ", 0.), ("NAV4_RADIO_SET_HZ", 0.), ("VOR3_SET", 0.), ("VOR4_SET", 0.)]);
        assert_eq!(ls.ls_course_output(true, 91.), Some(-1.), "reset once when cleared");
        assert_eq!(ls.ls_course_output(true, 91.), None);
    }
}
