//! FlyByWire's FCUs, PRIMs and SECs, fed as FlyByWireInterface feeds them.
//!
//! The computers themselves are FlyByWire's compiled C++ (see
//! `fbw_computers.rs`). This module is the port of the parts of
//! `fbw_a380/src/FlyByWireInterface.cpp` around them, with the line each
//! assignment comes from:
//!
//! - bus assembly from the Rust systems' variables: updateRa, updateLgciu,
//!   updateSfcc, updateIls, updateAdirs, updateFqms, updateTcas, updateAesu
//!   (cpp:1291-1449);
//! - updateFcu x2, updateFcuAfsLvars, updateFcuShim (cpp:2322-2729);
//! - updatePrim x3, updatePrimFgShim (cpp:1451-2007);
//! - updateSec x3 (cpp:2009-2217);
//! - after the FADECs: updateServoSolenoidStatus (cpp:2731-2884).
//!
//! in FlyByWireInterface::update's order (cpp:95-149). The MSFS-only calls in
//! those functions (sendEvent, sendData, execute_calculator_code, client data)
//! have no counterpart and are left out; each is noted where it is skipped.
//!
//! Inputs without a source in this port are left at what FlyByWire would see
//! with that source absent, and listed in [`UNAVAILABLE`].

#![allow(dead_code)]

use std::collections::HashMap;

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::afs_events::{Event, EventInputs};
use crate::fbw_computers::{FcuComputer, PrimComputer, SecComputer};
use crate::fbw_types::*;

/// Inputs this port has no source for, and what the computers see instead.
pub const UNAVAILABLE: &[(&str, &str)] = &[
    (
        "A32NX_RADIO_RECEIVER_USAGE_ENABLED's calculated receiver (CalculatedRadioReceiver.cpp, cpp:1138-1154)",
        "not ported: the receiver 3 sim data is used as with the option off",
    ),
    ("localizer distance without DME (localizer.distance, cpp:1158)", "0"),
    ("pitch trim switch discretes (SimInputPitchTrim, cpp:1576-1577, 2113-2114)", "wired: key_events.rs ELEV_TRIM_UP/DN, see prim.rs update_prim/update_sec"),
    ("rudder trim switch discretes (SimInputRudderTrim, cpp:2110-2112)", "wired: key_events.rs RUDDER_TRIM_LEFT/RIGHT/RESET, see prim.rs update_sec"),
    ("FMS LVars (A32NX_FMGC_FLIGHT_PHASE, AIRLINER_V2_SPEED, A32NX_SPEEDS_*, A32NX_FG_*, A32NX_FM1_*, ...)", "0 until an FMS writes them"),
    (
        "EFIS panel value events (A32NX.FCU_*_BARO/EFIS mode/range/navaid SET)",
        "-1 / no input, except FCU initialisation (SPD/HDG/ALT/VS knob turns and ALT knob absolute-set now reach the FCU/PRIMs; see msfs2xp-aircraft events.rs h_event/k_event)",
    ),
    ("vertical/lateral accelerometers, ISIS, rate gyros (cpp:1605-1610, 1622-1629)", "0 / zeroed bus, as FBW hard-codes"),
    ("idFm1BackbeamSelected (cpp:1658, never created in setupLocalVariables)", "false"),
];

/// FailuresConsumer ids for the FCUs (FailureList.h: Fcu1 = 22002, Fcu2 =
/// 22003), read at cpp:2374.
const FAILURE_FCU: [u64; 2] = [22_002, 22_003];
/// FailuresConsumer ids for the PRIMs (FailureList.h: Prim1..3 =
/// 27000..27002), read at cpp:1711.
const FAILURE_PRIM: [u64; 3] = [27_000, 27_001, 27_002];
/// FailuresConsumer ids for the SECs (FailureList.h: Sec1..3 = 27003..27005),
/// read at cpp:2200.
const FAILURE_SEC: [u64; 3] = [27_003, 27_004, 27_005];

/// What FlyByWire takes from MSFS's own sim data rather than from variables,
/// read from X-Plane by the plugin (or set directly in tests).
#[derive(Clone, Copy, Debug, Default)]
pub struct SimReadings {
    /// SimInput.inputs[0..3]: the elevator, aileron and rudder axes as MSFS
    /// hands them (AXIS_*_SET / 16384; negative is pull, right roll... see
    /// [`SimReadings::from_xplane_axes`]).
    pub inputs: [f64; 3],
    /// PLANE HEADING DEGREES MAGNETIC (FCU initialisation, cpp:930).
    pub psi_magnetic_deg: f64,
    /// INDICATED ALTITUDE (cpp:929).
    pub h_ind_ft: f64,
    /// RADIO HEIGHT (autoland warning and H-dot filter, cpp:1943-1964).
    pub h_radio_ft: f64,
    /// The spoilers handler state (SpoilersHandler, cpp:1585).
    pub spoilers_armed: bool,
    pub spoilers_handle_position: f64,
    /// G FORCE / SimData.nz_g (updateBaseData, cpp:1214), from X-Plane's
    /// g_nrml; see [`crate::sensors::g_force`].
    pub nz_g: f64,
    /// STRUCT BODY ROTATION VELOCITY (cpp:1178-1180), MSFS's x=pitch/y=yaw/z=roll
    /// radians per second; see [`crate::sensors::body_rotation_velocity_rad_s`].
    pub body_rotation_velocity_rad_s: (f64, f64, f64),
    /// STRUCT BODY ROTATION ACCELERATION (cpp:1186-1188), same axes as
    /// `body_rotation_velocity_rad_s` in radians per second squared; see
    /// [`crate::sensors::body_rotation_acceleration_rad_s2`].
    pub body_rotation_acceleration_rad_s2: (f64, f64, f64),
    /// ACCELERATION BODY Z / SimData.bz_m_s2 (cpp:1220), metres per second
    /// squared; see [`crate::sensors::accel_body_z_m_s2`].
    pub accel_body_z_m_s2: f64,
    /// AUTOPILOT MASTER / SimData.autopilot_master_on (cpp:165): X-Plane's
    /// own native autopilot engaged flag; see [`crate::sensors::autopilot_master_on`].
    pub autopilot_master_on: bool,
    /// KOHLSMAN SETTING STD:4 / SimData.kohlsmanSettingStd_4 (cpp:3070); see
    /// [`crate::sensors::kohlsman_setting_std_4`].
    pub kohlsman_setting_std_4: bool,
}

impl SimReadings {
    /// MSFS's axes from X-Plane's joystick ratios. FlyByWire's sidestick
    /// position is the negated axis (cpp:2892-2896) and is documented as
    /// +1 full right / full back and pedals +100 full right
    /// (fbw-a32nx/docs/a320-simvars.md:3850-3873); X-Plane's
    /// yoke_roll/pitch/heading ratios are +1 right / pull / right.
    pub fn from_xplane_axes(pitch_ratio: f64, roll_ratio: f64, heading_ratio: f64) -> [f64; 3] {
        [-pitch_ratio, -roll_ratio, -heading_ratio]
    }

    /// The handler's state from X-Plane's speedbrake handle, where -0.5 is
    /// armed and 0..1 the handle travel.
    pub fn spoilers_from_xplane(ratio: f64) -> (bool, f64) {
        if ratio < -0.25 {
            (true, 0.)
        } else {
            (false, ratio.clamp(0., 1.))
        }
    }
}

/// This tick's manual pitch/rudder trim switch presses (SimInputPitchTrim /
/// SimInputRudderTrim, SimConnectData.h:147-155): one-frame pulses, cleared
/// after every tick the same way FlyByWireInterface.cpp:985-987 clears them
/// (resetSimInputPitchTrim/resetSimInputRudderTrim). Read once per tick in
/// [`Prims::update_with`] (msfs2xp-aircraft's converter routes cockpit
/// ELEV_TRIM_UP/DN and RUDDER_TRIM_LEFT/RIGHT/RESET here; key_events.rs's
/// same-named X-Plane trim commands still move X-Plane's own trim wheel
/// dataref, which nothing downstream reads — the THS surface itself comes
/// from A32NX_HYD_FINAL_THS_DEFLECTION).
#[derive(Clone, Copy, Debug, Default)]
struct TrimPulses {
    pitch_up: bool,
    pitch_down: bool,
    rudder_left: bool,
    rudder_right: bool,
    rudder_reset: bool,
}

impl TrimPulses {
    const NAMES: [&'static str; 5] =
        ["XP_PITCH_TRIM_UP_PULSE", "XP_PITCH_TRIM_DOWN_PULSE", "XP_RUDDER_TRIM_LEFT_PULSE", "XP_RUDDER_TRIM_RIGHT_PULSE", "XP_RUDDER_TRIM_RESET_PULSE"];

    /// Read this tick's pulses and clear them (a command firing between two
    /// ticks is the input for the next one, like [`crate::afs_events`]).
    fn take<V: VariableRegistry + SimulatorReaderWriter>(names: &mut Names, vars: &mut V) -> Self {
        let get = |names: &mut Names, vars: &mut V, n: &str| names.get(vars, n) != 0.;
        let pulses = Self {
            pitch_up: get(names, vars, Self::NAMES[0]),
            pitch_down: get(names, vars, Self::NAMES[1]),
            rudder_left: get(names, vars, Self::NAMES[2]),
            rudder_right: get(names, vars, Self::NAMES[3]),
            rudder_reset: get(names, vars, Self::NAMES[4]),
        };
        for n in Self::NAMES {
            names.set(vars, n, 0.);
        }
        pulses
    }
}

/// SimData's ILS fields (SimConnectData.h:77-82, 119): MSFS's NAV simvars for
/// receiver 3, the A380X's multi-mode receiver (SimConnectInterface.cpp:214-219,
/// 255). They are simulator variables under their MSFS names, which the
/// sensors module fills from X-Plane's nav receiver 3.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NavSimData {
    /// NAV HAS DME:3
    pub dme_valid: bool,
    /// NAV DME:3, nautical miles.
    pub dme_nmi: f64,
    /// NAV HAS LOCALIZER:3
    pub loc_valid: bool,
    /// NAV RADIAL ERROR:3, degrees.
    pub loc_error_deg: f64,
    /// NAV HAS GLIDE SLOPE:3
    pub gs_valid: bool,
    /// NAV GLIDE SLOPE ERROR:3, degrees.
    pub gs_error_deg: f64,
    /// NAV RAW GLIDE SLOPE:3, degrees.
    pub gs_deg: f64,
    /// NAV MAGVAR:3, degrees.
    pub loc_magvar_deg: f64,
}

impl NavSimData {
    pub const DME_VALID: &'static str = "NAV HAS DME:3";
    pub const DME: &'static str = "NAV DME:3";
    pub const LOC_VALID: &'static str = "NAV HAS LOCALIZER:3";
    pub const LOC_ERROR: &'static str = "NAV RADIAL ERROR:3";
    pub const GS_VALID: &'static str = "NAV HAS GLIDE SLOPE:3";
    pub const GS_ERROR: &'static str = "NAV GLIDE SLOPE ERROR:3";
    pub const GS_ANGLE: &'static str = "NAV RAW GLIDE SLOPE:3";
    pub const MAGVAR: &'static str = "NAV MAGVAR:3";

    fn read<V: VariableRegistry + SimulatorReaderWriter>(names: &mut Names, vars: &mut V) -> Self {
        Self {
            dme_valid: names.get(vars, Self::DME_VALID) != 0.,
            dme_nmi: names.get(vars, Self::DME),
            loc_valid: names.get(vars, Self::LOC_VALID) != 0.,
            loc_error_deg: names.get(vars, Self::LOC_ERROR),
            gs_valid: names.get(vars, Self::GS_VALID) != 0.,
            gs_error_deg: names.get(vars, Self::GS_ERROR),
            gs_deg: names.get(vars, Self::GS_ANGLE),
            loc_magvar_deg: names.get(vars, Self::MAGVAR),
        }
    }
}

/// MathUtils::normalise180 (fbw-common/src/wasm/utils/MathUtils.h:39-47):
/// [-180, 180).
fn normalise_180(angle: f64) -> f64 {
    let a = ((angle % 360.) + 360.) % 360.;
    if a >= 180. {
        a - 360.
    } else {
        a
    }
}

/// MathUtils::correctMsfsLocaliserError (MathUtils.h:80-89).
pub fn correct_msfs_localiser_error(radial_error: f64) -> f64 {
    let e = normalise_180(radial_error);
    if e < -90. {
        -180. - e
    } else if e > 90. {
        180. - e
    } else {
        e
    }
}

/// The autoland warning trigger (cpp:1951-1954).
pub fn autoland_warning_condition(any_ap_engaged: bool, h_radio_ft: f64, nav: &NavSimData) -> bool {
    !any_ap_engaged
        || (h_radio_ft > 15. && (nav.loc_error_deg.abs() > 0.2 || !nav.loc_valid))
        || (h_radio_ft > 100. && (nav.gs_error_deg.abs() > 0.4 || !nav.gs_valid))
}

/// One ILS bus as updateIls builds it (cpp:1349-1358), from the receiver 3 sim
/// data and A32NX_FM_LS_COURSE.
pub fn ils_bus(nav: &NavSimData, ls_course_deg: f64) -> BaseIlsBus {
    let ssm = |valid: bool| if valid { SSM_NO } else { SSM_NCD };
    let mut ils = BaseIlsBus::default();
    ils.runway_heading_deg = BaseArinc429 {
        SSM: ssm(nav.loc_valid),
        Data: (((ls_course_deg - nav.loc_magvar_deg) % 360. + 360.) % 360.) as f32,
    };
    ils.ils_frequency_mhz = BaseArinc429 { SSM: SSM_NO, Data: 0. };
    ils.localizer_deviation_deg = BaseArinc429 {
        SSM: ssm(nav.loc_valid),
        Data: correct_msfs_localiser_error(nav.loc_error_deg) as f32,
    };
    ils.glideslope_deviation_deg = BaseArinc429 { SSM: ssm(nav.gs_valid), Data: nav.gs_error_deg as f32 };
    ils
}

// ---------------------------------------------------------------------------
// ARINC 429 words as FlyByWire packs them into variables (Arinc429Utils.cpp).
// ---------------------------------------------------------------------------

pub const SSM_NCD: u32 = 1;
pub const SSM_NO: u32 = 3;
pub const SSM_FT: u32 = 2;

/// Arinc429Utils::fromSimVar (Arinc429Utils.cpp:3-10).
pub fn from_simvar(value: f64) -> BaseArinc429 {
    let bits = value as u64;
    BaseArinc429 { SSM: (bits >> 32) as u32, Data: f32::from_bits(bits as u32) }
}

/// Arinc429Utils::toSimVar (Arinc429Utils.cpp:12-15).
pub fn to_simvar(word: BaseArinc429) -> f64 {
    (word.Data.to_bits() as u64 | (word.SSM as u64) << 32) as f64
}

/// Arinc429Utils::bitFromValueOr (Arinc429Utils.cpp:38-45).
pub fn bit_or(word: BaseArinc429, bit: u32, default: bool) -> bool {
    if word.SSM == SSM_NO || word.SSM == SSM_FT {
        (word.Data as u32 >> (bit - 1)) & 1 != 0
    } else {
        default
    }
}

/// Arinc429Utils::valueOr (Arinc429Utils.cpp:25-32).
pub fn value_or(word: BaseArinc429, default: f32) -> f32 {
    if word.SSM == SSM_NO || word.SSM == SSM_FT {
        word.Data
    } else {
        default
    }
}

fn b(v: f64) -> u8 {
    (v != 0.) as u8
}

fn f(v: bool) -> f64 {
    v as i32 as f64
}

// ---------------------------------------------------------------------------
// Variable names.
// ---------------------------------------------------------------------------

/// Identifiers by full LVar name, registered on first use. Names with
/// FlyByWire's `A32NX_` prefix go through the registry's prefixed lookup so
/// they meet the Rust systems' variables of the same name; the few LVars
/// without it (`AIRLINER_V2_SPEED`, `A380X_...`, `XMLVAR_...`) are taken
/// unprefixed.
#[derive(Default)]
struct Names {
    ids: HashMap<String, VariableIdentifier>,
}

impl Names {
    fn id<V: VariableRegistry>(&mut self, vars: &mut V, name: &str) -> VariableIdentifier {
        if let Some(id) = self.ids.get(name) {
            return *id;
        }
        let id = match name.strip_prefix("A32NX_") {
            Some(bare) => vars.get(bare.to_owned()),
            None => vars.get_unprefixed(name.to_owned()),
        };
        self.ids.insert(name.to_owned(), id);
        id
    }

    fn get<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V, name: &str) -> f64 {
        let id = self.id(vars, name);
        vars.read(&id)
    }

    fn set<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V, name: &str, value: f64) {
        let id = self.id(vars, name);
        vars.write(&id, value);
    }

    fn word<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V, name: &str) -> BaseArinc429 {
        from_simvar(self.get(vars, name))
    }
}

/// The ailerons', elevators' and rudders' positions in MSFS's format, as
/// FlyByWire's MSFS glue derives them from the hydraulics after each tick
/// (a380_systems_wasm/src/ailerons.rs:14-87 and 151-162, elevators.rs:14-60
/// and 105, rudder.rs:18-30), and as FlyByWireInterface reads them back
/// (cpp:740-757).
fn hyd_deflection_to_msfs_deflection(hyd: f64, min_angle: f64, max_angle: f64) -> f64 {
    (hyd * (max_angle + min_angle) - min_angle) / max_angle
}

/// One FCU channel, PRIM or SEC input and output, as FlyByWireInterface keeps
/// them between functions and ticks (FlyByWireInterface.h:110-147).
pub struct Prims {
    names: Names,
    prims: [PrimComputer; 3],
    secs: [SecComputer; 3],
    fcus: [FcuComputer; 2],

    prim_discrete: [BasePrimDiscreteOutputs; 3],
    prim_analog: [BasePrimAnalogOutputs; 3],
    prim_buses: [BasePrimOutBus; 3],
    sec_discrete: [BaseSecDiscreteOutputs; 3],
    sec_analog: [BaseSecAnalogOutputs; 3],
    sec_buses: [BaseSecOutBus; 3],
    fcu_buses: [BaseFcuBus; 2],
    fadec_buses: [BaseEec; 4],

    ra: [BaseRaBus; 3],
    lgciu: [BaseLgciuBus; 2],
    sfcc: [BaseSfccBus; 2],
    ils: [BaseIlsBus; 2],
    /// This tick's receiver 3 sim data (SimData nav_*).
    nav: NavSimData,
    adr: [BaseAdrBus; 3],
    ir: [BaseIrBus; 3],
    fqms: BaseFqms,
    tcas: BaseTcas,
    aesu: BaseAesuBus,

    monotonic_time: f64,
    start_state: f64,
    fcu_initialized: bool,
    time_ready: Option<f64>,
    autoland_warning_latch: bool,
    autoland_warning_triggered: bool,
    h_dot_filter_prev_u: f64,
    h_dot_filter_prev_y: f64,
    /// This tick's manual trim switch pulses, set once in
    /// [`Self::update_with`] and read by every PRIM/SEC in the same tick.
    trim: TrimPulses,
}

impl Prims {
    /// `start_state` is A32NX_START_STATE's number (StartState in
    /// systems/src/simulation/mod.rs:66-104), used by FCU initialisation.
    pub fn new<V: VariableRegistry + SimulatorReaderWriter>(vars: &mut V, start_state: f64) -> Self {
        let mut names = Names::default();
        // Every FlyByWire flight file starts with the PRIM and SEC overhead
        // pushbuttons in and the switching knobs at NORM (apron.FLT:242,246,
        // 386-397; the same in hangar, taxi, runway, climb, cruise, approach
        // and final). Nothing else in this port loads the flight files.
        for n in 1..=3 {
            names.set(vars, &format!("A32NX_PRIM_{n}_PUSHBUTTON_PRESSED"), 1.);
            names.set(vars, &format!("A32NX_SEC_{n}_PUSHBUTTON_PRESSED"), 1.);
        }
        names.set(vars, "A32NX_AIR_DATA_SWITCHING_KNOB", 1.);
        names.set(vars, "A32NX_ATT_HDG_SWITCHING_KNOB", 1.);
        // A fresh X-Plane dataref defaults to 0, which is a real altitude;
        // -1 (TrimPulses' sentinel is a bool so 0 already means "no press",
        // but this one carries a value) must be set explicitly so the first
        // tick does not read "the ALT knob wants 0 ft".
        names.set(vars, "XP_FCU_ALT_SET_PENDING", -1.);

        Self {
            names,
            prims: [PrimComputer::new(0), PrimComputer::new(1), PrimComputer::new(2)],
            secs: [SecComputer::new(0), SecComputer::new(1), SecComputer::new(2)],
            fcus: [FcuComputer::new(), FcuComputer::new()],
            prim_discrete: Default::default(),
            prim_analog: Default::default(),
            prim_buses: Default::default(),
            sec_discrete: Default::default(),
            sec_analog: Default::default(),
            sec_buses: Default::default(),
            fcu_buses: Default::default(),
            fadec_buses: Default::default(),
            ra: Default::default(),
            lgciu: Default::default(),
            sfcc: Default::default(),
            ils: Default::default(),
            nav: NavSimData::default(),
            adr: Default::default(),
            ir: Default::default(),
            fqms: Default::default(),
            tcas: Default::default(),
            aesu: Default::default(),
            monotonic_time: 0.,
            start_state,
            fcu_initialized: false,
            time_ready: None,
            autoland_warning_latch: false,
            autoland_warning_triggered: false,
            h_dot_filter_prev_u: 0.,
            h_dot_filter_prev_y: 0.,
            trim: TrimPulses::default(),
        }
    }

    pub fn prim_buses(&self) -> [BasePrimOutBus; 3] {
        self.prim_buses
    }

    pub fn prim_discrete_outputs(&self) -> [BasePrimDiscreteOutputs; 3] {
        self.prim_discrete
    }

    pub fn prim_analog_outputs(&self) -> [BasePrimAnalogOutputs; 3] {
        self.prim_analog
    }

    pub fn sec_analog_outputs(&self) -> [BaseSecAnalogOutputs; 3] {
        self.sec_analog
    }

    pub fn fcu_buses(&self) -> [BaseFcuBus; 2] {
        self.fcu_buses
    }

    pub fn sec_buses(&self) -> [BaseSecOutBus; 3] {
        self.sec_buses
    }

    pub fn ra_buses(&self) -> [BaseRaBus; 3] {
        self.ra
    }

    pub fn ir_buses(&self) -> [BaseIrBus; 3] {
        self.ir
    }

    pub fn adr_buses(&self) -> [BaseAdrBus; 3] {
        self.adr
    }

    pub fn sfcc_buses(&self) -> [BaseSfccBus; 2] {
        self.sfcc
    }

    pub fn lgciu_buses(&self) -> [BaseLgciuBus; 2] {
        self.lgciu
    }

    /// handleFcuInitialization (cpp:909-974): the FCU events FlyByWire sends
    /// once after spawning, returned for the caller to queue (they arrive as
    /// key events, so on the next tick).
    pub fn fcu_initialization(&mut self, readings: &SimReadings, simulation_time: f64) -> Vec<Event> {
        if self.fcu_initialized {
            return Vec::new();
        }
        let ready = *self.time_ready.get_or_insert(simulation_time);
        let since_ready = simulation_time - ready;
        let mut events = Vec::new();
        if self.start_state >= 5. && since_ready > 6. {
            let target_altitude = (readings.h_ind_ft / 1000.).round() * 1000.;
            let target_heading = ((readings.psi_magnetic_deg / 10.).round() * 10.) % 360.;
            events.extend([
                Event::FcuSpdPush,
                Event::FcuHdgSet(target_heading),
                Event::FcuHdgPull,
                Event::FcuAltSet(target_altitude),
                Event::FcuVsSet(if readings.h_ind_ft < target_altitude { 1000. } else { -1000. }),
                Event::FcuVsPull,
                Event::FcuFdPush,
                Event::FcuAthrPush,
                Event::FcuAp1Push,
            ]);
            // EFIS mode/range/TRAF/NAVAID events (cpp:940-949) have no EFIS
            // panel event input here.
            self.fcu_initialized = true;
        } else if self.start_state == 4. && since_ready > 1. {
            events.extend([Event::FcuAltSet(15000.), Event::FcuFdPush]);
            self.fcu_initialized = true;
        } else if self.start_state < 4. && since_ready > 1. {
            self.fcu_initialized = true;
        }
        events
    }

    /// FlyByWireInterface::update from updateRa to updateSec (cpp:95-139),
    /// returning the PRIM output buses the FADECs take. Reads the active
    /// failure ids from [`crate::failures`] (see [`Self::update_with`]).
    pub fn update<V: VariableRegistry + SimulatorReaderWriter>(
        &mut self,
        vars: &mut V,
        readings: &SimReadings,
        events: &EventInputs,
        dt: f64,
        simulation_time: f64,
    ) -> [BasePrimOutBus; 3] {
        let active_failures = crate::failures::active_ids();
        self.update_with(vars, readings, events, dt, simulation_time, &active_failures)
    }

    /// [`Self::update`] with the active FailuresConsumer ids given explicitly
    /// (for tests, as extra_backend_fcdc.rs's `update_with` does).
    #[allow(clippy::too_many_arguments)]
    pub fn update_with<V: VariableRegistry + SimulatorReaderWriter>(
        &mut self,
        vars: &mut V,
        readings: &SimReadings,
        events: &EventInputs,
        dt: f64,
        simulation_time: f64,
        active_failures: &[u64],
    ) -> [BasePrimOutBus; 3] {
        // readDataAndLocalVariables (cpp:1035-1037).
        let dt = dt.max(0.002);
        self.monotonic_time += dt;
        // XP_FCU_ALT_SET_PENDING: the port's own one-shot input for the
        // stock Asobo altitude knob's turn (converter events.rs k_event
        // "AP_ALT_VAR_SET_ENGLISH"), since an X-Plane command carries no
        // argument the way SimConnect's AddInputEvent does. Applied exactly
        // like FlyByWireInterface.cpp:1554 (`sim_input.alt =
        // simInputAutopilot.ALT_set`) for this tick's PRIMs, then consumed.
        let mut ev = *events;
        let pending_alt = self.names.get(vars, "XP_FCU_ALT_SET_PENDING");
        if pending_alt >= 0. {
            ev.autopilot.alt_set = pending_alt;
            self.names.set(vars, "XP_FCU_ALT_SET_PENDING", -1.);
        }
        let events = &ev;
        self.trim = TrimPulses::take(&mut self.names, vars);
        // updateRadioReceiver (cpp:76) runs before the laws; the receiver 3
        // sim data it and updateIls, updatePrim and updatePrimFgShim read.
        self.nav = NavSimData::read(&mut self.names, vars);
        self.update_radio_receiver(vars);

        for i in 0..3 {
            self.update_ra(vars, i);
        }
        for i in 0..2 {
            self.update_lgciu(vars, i);
            self.update_sfcc(vars, i);
            self.update_ils(vars, i);
        }
        for i in 0..3 {
            self.update_adirs(vars, i);
        }
        self.update_fqms(vars);
        self.update_tcas(vars);
        self.update_aesu(vars);
        self.write_surface_positions(vars);

        for i in 0..2 {
            let fault_active = active_failures.contains(&FAILURE_FCU[i]);
            self.update_fcu(vars, events, dt, simulation_time, i, fault_active);
        }
        // updateEfisSync (cpp:2421-2479) only sends baro sync events.
        self.update_fcu_afs_lvars(vars);
        self.update_fcu_shim(vars);
        for i in 0..3 {
            let fault_active = active_failures.contains(&FAILURE_PRIM[i]);
            self.update_prim(vars, readings, events, dt, simulation_time, i, fault_active);
        }
        self.update_prim_fg_shim(vars, readings, dt);
        for i in 0..3 {
            let fault_active = active_failures.contains(&FAILURE_SEC[i]);
            self.update_sec(vars, readings, dt, simulation_time, i, fault_active);
        }
        // updateFcdc (cpp:141-143) is not ported (extra_backend_fcdc.rs).
        self.prim_buses
    }

    /// After the FADECs (cpp:145-149): keep their buses for the next tick's
    /// PRIM and SEC inputs (cpp:1692-1695, 2178-2181), then
    /// updateServoSolenoidStatus.
    pub fn update_after_fadecs<V: VariableRegistry + SimulatorReaderWriter>(
        &mut self,
        vars: &mut V,
        fadec_buses: [BaseEec; 4],
    ) {
        self.fadec_buses = fadec_buses;
        self.update_servo_solenoid_status(vars);
    }

    // -- buses from the Rust systems -------------------------------------------

    fn update_ra<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V, i: usize) {
        // cpp:1292
        self.ra[i].radio_height_ft = self.names.word(vars, &format!("A32NX_RA_{}_RADIO_ALTITUDE", i + 1));
    }

    fn update_lgciu<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V, i: usize) {
        // cpp:1302-1305
        let n = i + 1;
        let l = &mut self.lgciu[i];
        l.discrete_word_1 = self.names.word(vars, &format!("A32NX_LGCIU_{n}_DISCRETE_WORD_1"));
        l.discrete_word_2 = self.names.word(vars, &format!("A32NX_LGCIU_{n}_DISCRETE_WORD_2"));
        l.discrete_word_3 = self.names.word(vars, &format!("A32NX_LGCIU_{n}_DISCRETE_WORD_3"));
        l.discrete_word_4 = self.names.word(vars, &format!("A32NX_LGCIU_{n}_DISCRETE_WORD_4"));
    }

    fn update_sfcc<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V, i: usize) {
        // cpp:1315-1320
        let n = i + 1;
        let s = &mut self.sfcc[i];
        s.slat_flap_component_status_word = self.names.word(vars, &format!("A32NX_SFCC_{n}_SLAT_FLAP_COMPONENT_STATUS_WORD"));
        s.slat_flap_system_status_word = self.names.word(vars, &format!("A32NX_SFCC_{n}_SLAT_FLAP_SYSTEM_STATUS_WORD"));
        s.slat_flap_actual_position_word = self.names.word(vars, &format!("A32NX_SFCC_{n}_SLAT_FLAP_ACTUAL_POSITION_WORD"));
        s.slat_actual_position_deg = self.names.word(vars, &format!("A32NX_SFCC_{n}_SLAT_ACTUAL_POSITION_WORD"));
        s.flap_actual_position_deg = self.names.word(vars, &format!("A32NX_SFCC_{n}_FLAP_ACTUAL_POSITION_WORD"));
    }

    fn update_ils<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V, i: usize) {
        // cpp:1329-1365. With A32NX_RADIO_RECEIVER_USAGE_ENABLED set FlyByWire
        // reads the receiver LVars (cpp:1337-1341); update_radio_receiver
        // writes those from the same sim data here, so both paths agree.
        let course = self.names.get(vars, "A32NX_FM_LS_COURSE");
        self.ils[i] = ils_bus(&self.nav, course);
    }

    /// updateRadioReceiver (cpp:1134-1165) with the option off (cpp:1155-1161):
    /// the receiver 3 sim data into the LVars the displays read.
    fn update_radio_receiver<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V) {
        let nav = self.nav;
        let n = &mut self.names;
        n.set(vars, "A32NX_RADIO_RECEIVER_LOC_IS_VALID", f(nav.loc_valid));
        n.set(vars, "A32NX_RADIO_RECEIVER_LOC_DEVIATION", nav.loc_error_deg);
        n.set(vars, "A32NX_RADIO_RECEIVER_LOC_DISTANCE", if nav.dme_valid { nav.dme_nmi } else { 0. });
        n.set(vars, "A32NX_RADIO_RECEIVER_GS_IS_VALID", f(nav.gs_valid));
        n.set(vars, "A32NX_RADIO_RECEIVER_GS_DEVIATION", nav.gs_error_deg);
    }

    fn update_adirs<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V, i: usize) {
        let n = i + 1;
        let adr = |names: &mut Names, vars: &mut V, s: &str| names.word(vars, &format!("A32NX_ADIRS_ADR_{n}_{s}"));
        // cpp:1368-1377
        let a = BaseAdrBus {
            altitude_standard_ft: adr(&mut self.names, vars, "ALTITUDE"),
            altitude_corrected_1_ft: adr(&mut self.names, vars, "BARO_CORRECTED_ALTITUDE_1"),
            altitude_corrected_2_ft: adr(&mut self.names, vars, "BARO_CORRECTED_ALTITUDE_2"),
            mach: adr(&mut self.names, vars, "MACH"),
            airspeed_computed_kn: adr(&mut self.names, vars, "COMPUTED_AIRSPEED"),
            airspeed_true_kn: adr(&mut self.names, vars, "TRUE_AIRSPEED"),
            vertical_speed_ft_min: adr(&mut self.names, vars, "BAROMETRIC_VERTICAL_SPEED"),
            aoa_corrected_deg: adr(&mut self.names, vars, "ANGLE_OF_ATTACK"),
            corrected_average_static_pressure: adr(&mut self.names, vars, "CORRECTED_AVERAGE_STATIC_PRESSURE"),
        };
        self.adr[i] = a;
        let ir = |names: &mut Names, vars: &mut V, s: &str| names.word(vars, &format!("A32NX_ADIRS_IR_{n}_{s}"));
        // cpp:1379-1402; the fields FBW does not set stay as they were (zero).
        let r = &mut self.ir[i];
        r.discrete_word_1 = ir(&mut self.names, vars, "MAINT_WORD");
        r.latitude_deg = ir(&mut self.names, vars, "LATITUDE");
        r.longitude_deg = ir(&mut self.names, vars, "LONGITUDE");
        r.ground_speed_kn = ir(&mut self.names, vars, "GROUND_SPEED");
        r.track_angle_true_deg = ir(&mut self.names, vars, "TRUE_TRACK");
        r.heading_true_deg = ir(&mut self.names, vars, "TRUE_HEADING");
        r.wind_speed_kn = ir(&mut self.names, vars, "WIND_SPEED");
        r.wind_direction_true_deg = ir(&mut self.names, vars, "WIND_DIRECTION");
        r.track_angle_magnetic_deg = ir(&mut self.names, vars, "TRACK");
        r.heading_magnetic_deg = ir(&mut self.names, vars, "HEADING");
        r.drift_angle_deg = ir(&mut self.names, vars, "DRIFT_ANGLE");
        r.flight_path_angle_deg = ir(&mut self.names, vars, "FLIGHT_PATH_ANGLE");
        r.pitch_angle_deg = ir(&mut self.names, vars, "PITCH");
        r.roll_angle_deg = ir(&mut self.names, vars, "ROLL");
        r.body_pitch_rate_deg_s = ir(&mut self.names, vars, "BODY_PITCH_RATE");
        r.body_roll_rate_deg_s = ir(&mut self.names, vars, "BODY_ROLL_RATE");
        r.body_yaw_rate_deg_s = ir(&mut self.names, vars, "BODY_YAW_RATE");
        r.body_long_accel_g = ir(&mut self.names, vars, "BODY_LONGITUDINAL_ACC");
        r.body_lat_accel_g = ir(&mut self.names, vars, "BODY_LATERAL_ACC");
        r.body_normal_accel_g = ir(&mut self.names, vars, "BODY_NORMAL_ACC");
        r.track_angle_rate_deg_s = ir(&mut self.names, vars, "HEADING_RATE");
        r.pitch_att_rate_deg_s = ir(&mut self.names, vars, "PITCH_ATT_RATE");
        r.roll_att_rate_deg_s = ir(&mut self.names, vars, "ROLL_ATT_RATE");
        r.inertial_vertical_speed_ft_s = ir(&mut self.names, vars, "VERTICAL_SPEED");
    }

    fn update_fqms<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V) {
        // cpp:1413-1414
        self.fqms.gross_weight_kg = self.names.word(vars, "A32NX_FQMS_GROSS_WEIGHT");
        self.fqms.gross_weight_cg_pct = self.names.word(vars, "A32NX_FQMS_CENTER_OF_GRAVITY_MAC");
    }

    fn update_tcas<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V) {
        // cpp:1424-1429
        let state = self.names.get(vars, "A32NX_TCAS_STATE");
        self.tcas.tcas_valid = (self.names.get(vars, "A32NX_TCAS_FAULT") == 0.) as u8;
        self.tcas.ta_ra_mode = (self.names.get(vars, "A32NX_TCAS_MODE") >= 2.) as u8;
        self.tcas.ta_active = (state == 1.) as u8;
        self.tcas.ra_active = (state >= 2.) as u8;
        self.tcas.ra_rate_to_maintain = (self.names.get(vars, "A32NX_TCAS_RA_RATE_TO_MAINTAIN") / 100.).round();
        self.tcas.ra_corrective = b(self.names.get(vars, "A32NX_TCAS_RA_CORRECTIVE"));
    }

    fn update_aesu<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V) {
        // cpp:1439-1442
        let ta_or_ra = self.names.get(vars, "A32NX_TCAS_STATE") >= 1.;
        self.aesu.aesu_status_word = BaseArinc429 { SSM: SSM_NO, Data: ((ta_or_ra as u32) << 10) as f32 };
    }

    /// The MSFS-format surface positions FlyByWire's aspects write
    /// (see [`hyd_deflection_to_msfs_deflection`]).
    fn write_surface_positions<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V) {
        for side in ["LEFT", "RIGHT"] {
            for part in ["OUTWARD", "MIDDLE", "INWARD"] {
                let hyd = self.names.get(vars, &format!("A32NX_HYD_AIL_{side}_{part}_DEFLECTION"));
                let msfs = hyd_deflection_to_msfs_deflection(hyd, 20., 30.);
                let msfs = if side == "LEFT" { -msfs } else { msfs };
                self.names.set(vars, &format!("A32NX_HYD_AILERON_{side}_{part}_DEFLECTION"), msfs);
            }
            for part in ["OUTWARD", "INWARD"] {
                let hyd = self.names.get(vars, &format!("A32NX_HYD_ELEV_{side}_{part}_DEFLECTION"));
                self.names.set(
                    vars,
                    &format!("A32NX_HYD_ELEVATOR_{side}_{part}_DEFLECTION"),
                    hyd_deflection_to_msfs_deflection(hyd, 20., 30.),
                );
            }
        }
        for which in ["UPPER", "LOWER"] {
            let hyd = self.names.get(vars, &format!("A32NX_HYD_{which}_RUD_DEFLECTION"));
            self.names.set(vars, &format!("A32NX_HYD_{which}_RUDDER_DEFLECTION"), hyd * 2. - 1.);
        }
    }

    // -- FCU -------------------------------------------------------------------

    #[allow(clippy::too_many_arguments)]
    fn update_fcu<V: VariableRegistry + SimulatorReaderWriter>(
        &mut self,
        vars: &mut V,
        events: &EventInputs,
        dt: f64,
        simulation_time: f64,
        i: usize,
        fault_active: bool,
    ) {
        let side = if i == 0 { "L" } else { "R" };
        let ap = &events.autopilot;
        let mut input = FcuInputs::default();
        // cpp:2326-2333
        input.time = BaseTime { dt, simulation_time, monotonic_time: self.monotonic_time };
        input.sim_data.tracking_mode_on_override = (self.names.get(vars, "A32NX_EXTERNAL_OVERRIDE") == 1.) as u8;
        // cpp:2335-2344
        let pick = |l: f64, r: f64| if i == 0 { l } else { r };
        input.sim_input.baro_setting_hpa = pick(ap.baro_left_set, ap.baro_right_set) as f32;
        input.sim_input.efis_mode = pick(ap.efis_mode_left_set, ap.efis_mode_right_set) as i8;
        input.sim_input.efis_range = pick(ap.efis_range_left_set, ap.efis_range_right_set) as i8;
        input.sim_input.navaid_1_mode = pick(ap.efis_navaid_mode_1_left_set, ap.efis_navaid_mode_1_right_set) as i8;
        input.sim_input.navaid_2_mode = pick(ap.efis_navaid_mode_2_left_set, ap.efis_navaid_mode_2_right_set) as i8;
        // cpp:2346-2360
        let d = &mut input.discrete_inputs;
        d.fcu_switched_off = b(self.names.get(vars, "A32NX_FCU_SWITCHED_OFF"));
        d.efis_backup_activated = b(self.names.get(vars, &format!("A32NX_FCU_EFIS_{side}_BACKUP_ACTIVE")));
        let select = |p: &BasePrimDiscreteOutputs| if i == 0 { p.fcu_1_select } else { p.fcu_2_select };
        d.selected_by_prim_1 = select(&self.prim_discrete[0]);
        d.selected_by_prim_2 = select(&self.prim_discrete[1]);
        d.selected_by_prim_3 = select(&self.prim_discrete[2]);
        d.lights_test = b(self.names.get(vars, "A32NX_OVHD_INTLT_ANN"));
        d.pin_prog_qfe_avail = 0;
        d.efis_inputs = events.efis[i];
        d.efis_inputs.baro_is_inhg = b(self.names.get(vars, &format!("A32NX_FCU_EFIS_{side}_BARO_IS_INHG")));
        d.afs_inputs = events.afs;
        d.afs_inputs.alt_increment_1000 = b(self.names.get(vars, "A32NX_FCU_ALT_INCREMENT_1000"));
        // cpp:2362-2365
        input.bus_inputs.prim_1_bus = self.prim_buses[0];
        input.bus_inputs.prim_2_bus = self.prim_buses[1];
        input.bus_inputs.prim_3_bus = self.prim_buses[2];
        input.bus_inputs.aesu_bus = self.aesu;

        self.fcus[i].set_inputs(&input);
        // cpp:2367: taken before the update, and not refreshed after it.
        let discrete = self.fcus[i].discrete_outputs();
        // cpp:2374-2376
        let power = if i == 0 { "A32NX_ELEC_108PH_BUS_IS_POWERED" } else { "A32NX_ELEC_DC_2_BUS_IS_POWERED" };
        let powered = self.names.get(vars, power) != 0.;
        // cpp:2374: failuresConsumer.isActive(Fcu1/Fcu2).
        self.fcus[i].update(dt, simulation_time, fault_active, powered);
        self.fcu_buses[i] = self.fcus[i].bus_outputs();

        // cpp:2385-2416
        let bus = self.fcu_buses[i];
        let n = &mut self.names;
        n.set(vars, &format!("A32NX_FCU_EFIS_{side}_DISCRETE_WORD_1"), to_simvar(bus.efis_discrete_word_1));
        n.set(vars, &format!("A32NX_FCU_EFIS_{side}_DISCRETE_WORD_2"), to_simvar(bus.efis_discrete_word_2));
        n.set(vars, &format!("A32NX_FCU_EFIS_{side}_BARO"), to_simvar(bus.baro_setting_inhg));
        n.set(vars, &format!("A32NX_FCU_EFIS_{side}_BARO_HPA"), to_simvar(bus.baro_setting_hpa));
        n.set(vars, &format!("A32NX_FCU_AFS_{side}_DISCRETE_WORD_1"), to_simvar(bus.afs_discrete_word_1));
        n.set(vars, &format!("A32NX_FCU_AFS_{side}_DISCRETE_WORD_2"), to_simvar(bus.afs_discrete_word_2));
        let e = discrete.efis_outputs;
        for (name, v) in [
            ("VV_LIGHT_ON", e.vv_light_on),
            ("LS_LIGHT_ON", e.ls_light_on),
            ("TAXI_LIGHT_ON", e.taxi_light_on),
            ("CSTR_LIGHT_ON", e.cstr_light_on),
            ("WPT_LIGHT_ON", e.wpt_light_on),
            ("VORD_LIGHT_ON", e.vord_light_on),
            ("NDB_LIGHT_ON", e.ndb_light_on),
            ("ARPT_LIGHT_ON", e.arpt_light_on),
            ("TRAF_LIGHT_ON", e.traf_light_on),
            ("WX_LIGHT_ON", e.wxr_light_on),
            ("TERR_LIGHT_ON", e.terr_light_on),
            ("DISPLAY_BARO_IS_INHG", e.baro_is_inhg),
            ("DISPLAY_BARO_IS_STD", e.baro_is_std),
            ("DISPLAY_BARO_PRESET_VISIBLE", e.baro_preset_visible),
            ("CP_ACTIVE", e.efis_cp_active),
        ] {
            n.set(vars, &format!("A32NX_FCU_EFIS_{side}_{name}"), v as f64);
        }
        n.set(vars, &format!("A32NX_FCU_EFIS_{side}_NAVAID_1_MODE"), e.navaid_1_mode as f64);
        n.set(vars, &format!("A32NX_FCU_EFIS_{side}_NAVAID_2_MODE"), e.navaid_2_mode as f64);
        n.set(vars, &format!("A32NX_FCU_EFIS_{side}_EFIS_RANGE"), e.efis_range as f64);
        n.set(vars, &format!("A32NX_FCU_EFIS_{side}_EFIS_MODE"), e.efis_mode as f64);
        n.set(vars, &format!("A32NX_FCU_EFIS_{side}_DISPLAY_BARO_VALUE"), e.baro_value as f64);
        n.set(vars, &format!("A32NX_FCU_EFIS_{side}_DISPLAY_BARO_MODE"), e.baro_mode as f64);
    }

    fn update_fcu_afs_lvars<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V) {
        // cpp:2481-2511
        let a1 = self.fcus[0].discrete_outputs().afs_outputs;
        let a2 = self.fcus[1].discrete_outputs().afs_outputs;
        // XP_FCU_ALT_INCREMENT_FT: the real increment (100 or 1000 ft) as a
        // usable number, for the converter's bind.rs to feed the stock
        // altitude knob's own INCREMENT read (see home()'s
        // XMLVAR_Autopilot_Altitude_Increment override) instead of an
        // unwritten local var that would otherwise divide/modulo by zero.
        let increment_ft = if self.names.get(vars, "A32NX_FCU_ALT_INCREMENT_1000") != 0. { 1000. } else { 100. };
        self.names.set(vars, "XP_FCU_ALT_INCREMENT_FT", increment_ft);
        let n = &mut self.names;
        let or = |x: u8, y: u8| f(x != 0 || y != 0);
        n.set(vars, "A32NX_FCU_AFS_CP_ACTIVE", or(a1.afs_cp_active, a2.afs_cp_active));
        n.set(vars, "A32NX_FCU_AP_1_LIGHT_ON", or(a1.ap_1_light_on, a2.ap_1_light_on));
        n.set(vars, "A32NX_FCU_AP_2_LIGHT_ON", or(a1.ap_2_light_on, a2.ap_2_light_on));
        n.set(vars, "A32NX_FCU_FD_LIGHT_ON", or(a1.fd_light_on, a2.fd_light_on));
        n.set(vars, "A32NX_FCU_ATHR_LIGHT_ON", or(a1.athr_light_on, a2.athr_light_on));
        n.set(vars, "A32NX_FCU_LOC_LIGHT_ON", or(a1.loc_light_on, a2.loc_light_on));
        n.set(vars, "A32NX_FCU_ALT_LIGHT_ON", or(a1.alt_light_on, a2.alt_light_on));
        n.set(vars, "A32NX_FCU_APPR_LIGHT_ON", or(a1.appr_light_on, a2.appr_light_on));
        let s = if a1.afs_cp_active != 0 { a1 } else { a2 };
        n.set(vars, "A32NX_FCU_AFS_DISPLAY_TRK_FPA_MODE", s.trk_fpa_mode as f64);
        n.set(vars, "A32NX_PUSH_TRUE_REF", s.true_mode as f64);
        n.set(vars, "A32NX_FCU_AFS_DISPLAY_MACH_MODE", s.mach_mode as f64);
        n.set(vars, "A32NX_FCU_AFS_DISPLAY_TRUE_MODE", s.true_mode as f64);
        n.set(vars, "A32NX_FCU_AFS_DISPLAY_SPD_MACH_VALUE", s.spd_mach_value);
        n.set(vars, "A32NX_FCU_AFS_DISPLAY_SPD_MACH_DASHES", s.spd_mach_dashes as f64);
        n.set(vars, "A32NX_FCU_AFS_DISPLAY_HDG_TRK_VALUE", s.hdg_trk_value);
        n.set(vars, "A32NX_FCU_AFS_DISPLAY_HDG_TRK_DASHES", s.hdg_trk_dashes as f64);
        n.set(vars, "A32NX_FCU_AFS_DISPLAY_ALT_VALUE", s.alt_value);
        n.set(vars, "A32NX_FCU_AFS_DISPLAY_VS_FPA_VALUE", s.vs_fpa_value);
        n.set(vars, "A32NX_FCU_AFS_DISPLAY_VS_FPA_DASHES", s.vs_fpa_dashes as f64);
    }

    fn update_fcu_shim<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V) {
        // cpp:2514-2729, the LVars (KOHLSMAN_SET, SimOutputAltimeter,
        // HEADING_BUG_SET, AP_ALT_VAR_SET and the H-events are MSFS-only).
        let navaid = |adf: bool, vor: bool| if adf { 1. } else if vor { 2. } else { 0. };
        let nd_mode = |b1: bool, b2: bool, b3: bool, b4: bool, b5: bool| {
            if b5 {
                0.
            } else if b4 {
                1.
            } else if b3 {
                2.
            } else if b2 {
                3.
            } else if b1 {
                4.
            } else {
                0.
            }
        };
        let nd_range = |bits: [bool; 6], zoom: bool| {
            bits.iter().position(|&x| x).map_or(if !zoom { 7. } else { 0. }, |p| (p + 1) as f64)
        };
        let oans_range = |bits: [bool; 5]| bits.iter().position(|&x| x).map_or(5., |p| p as f64);
        let nd_filter = |b1: bool, b2: bool, b3: bool, b4: bool, b5: bool| {
            (b1 as i32 | (b2 as i32) << 2 | (b3 as i32) << 1 | (b4 as i32) << 3 | (b5 as i32) << 4) as f64
        };
        let overlay = |b1: bool, b2: bool| if b1 { 1. } else if b2 { 2. } else { 0. };
        let baro_mode = |b1: bool, b2: bool| if b1 { 3. } else if b2 { 1. } else { 0. };

        for (i, side) in ["L", "R"].into_iter().enumerate() {
            let w1 = self.fcu_buses[i].efis_discrete_word_1;
            let w2 = self.fcu_buses[i].efis_discrete_word_2;
            let oans = oans_range([19, 20, 21, 22, 23].map(|bit| bit_or(w1, bit, false)));
            let n = &mut self.names;
            n.set(vars, &format!("A32NX_EFIS_{side}_NAVAID_1_MODE"), navaid(bit_or(w2, 26, false), bit_or(w2, 28, true)));
            n.set(vars, &format!("A32NX_EFIS_{side}_NAVAID_2_MODE"), navaid(bit_or(w2, 27, true), bit_or(w2, 29, false)));
            n.set(
                vars,
                &format!("A32NX_EFIS_{side}_ND_MODE"),
                nd_mode(bit_or(w1, 11, false), bit_or(w1, 12, false), bit_or(w1, 13, true), bit_or(w1, 14, false), bit_or(w1, 15, false)),
            );
            n.set(
                vars,
                &format!("A32NX_EFIS_{side}_ND_RANGE"),
                nd_range(
                    [bit_or(w1, 24, false), bit_or(w1, 25, false), bit_or(w1, 26, false), bit_or(w1, 27, true), bit_or(w1, 28, false), bit_or(w1, 29, false)],
                    oans != 5.,
                ),
            );
            n.set(vars, &format!("A32NX_EFIS_{side}_OANS_RANGE"), oans);
            n.set(
                vars,
                &format!("A32NX_EFIS_{side}_OPTION"),
                nd_filter(bit_or(w2, 17, false), bit_or(w2, 18, false), bit_or(w2, 19, false), bit_or(w2, 20, false), bit_or(w2, 21, false)),
            );
            n.set(vars, &format!("A380X_EFIS_{side}_ACTIVE_OVERLAY"), overlay(bit_or(w2, 23, false), bit_or(w2, 24, false)));
            n.set(vars, &format!("A32NX_EFIS_TERR_{side}_ACTIVE"), f(bit_or(w2, 24, false)));
            n.set(vars, &format!("A380X_EFIS_{side}_TRAF_BUTTON_IS_ON"), f(bit_or(w2, 25, true)));
            n.set(vars, &format!("A380X_EFIS_{side}_LS_BUTTON_IS_ON"), f(bit_or(w2, 14, true)));
            n.set(vars, &format!("XMLVAR_Baro{}_Mode", i + 1), baro_mode(bit_or(w2, 11, true), bit_or(w2, 12, false)));
        }
        // cpp:2684-2685
        let (l, r) = (self.fcu_buses[0].baro_setting_hpa, self.fcu_buses[1].baro_setting_hpa);
        self.names.set(vars, "A32NX_FCU_LEFT_EIS_BARO_HPA", to_simvar(l));
        self.names.set(vars, "A32NX_FCU_RIGHT_EIS_BARO_HPA", to_simvar(r));

        // cpp:2688-2714
        let a1 = self.fcus[0].discrete_outputs().afs_outputs;
        let a2 = self.fcus[1].discrete_outputs().afs_outputs;
        let s = if a1.afs_cp_active != 0 { a1 } else { a2 };
        let n = &mut self.names;
        n.set(vars, "A32NX_FCU_SPD_MANAGED_DASHES", s.spd_mach_dashes as f64);
        n.set(vars, "A32NX_AUTOPILOT_SPEED_SELECTED", if s.spd_mach_dashes != 0 { -1. } else { s.spd_mach_value });
        n.set(vars, "A32NX_TRK_FPA_MODE_ACTIVE", s.trk_fpa_mode as f64);
        let hdg = if s.hdg_trk_dashes != 0 { -1. } else { s.hdg_trk_value };
        n.set(vars, "A32NX_FCU_HEADING_SELECTED", hdg);
        n.set(vars, "A32NX_AUTOPILOT_HEADING_SELECTED", hdg);
        n.set(vars, "A320_FCU_SHOW_SELECTED_HEADING", f(s.hdg_trk_dashes == 0));
        n.set(vars, "A32NX_FCU_HDG_MANAGED_DASHES", s.hdg_trk_dashes as f64);
        n.set(vars, "A32NX_AUTOPILOT_VS_SELECTED", if s.trk_fpa_mode != 0 { 0. } else { s.vs_fpa_value });
        n.set(vars, "A32NX_AUTOPILOT_FPA_SELECTED", if s.trk_fpa_mode == 0 { 0. } else { s.vs_fpa_value });
        n.set(vars, "A32NX_FCU_VS_MANAGED", s.vs_fpa_dashes as f64);
    }

    // -- PRIM ------------------------------------------------------------------

    #[allow(clippy::too_many_arguments)]
    fn update_prim<V: VariableRegistry + SimulatorReaderWriter>(
        &mut self,
        vars: &mut V,
        readings: &SimReadings,
        events: &EventInputs,
        dt: f64,
        simulation_time: f64,
        i: usize,
        fault_active: bool,
    ) {
        let n = &mut self.names;
        let hyd = |n: &mut Names, vars: &mut V, s: &str| n.get(vars, &format!("A32NX_HYD_{s}_DEFLECTION"));
        // cpp:1476-1536
        let (la1, ra1, la2, ra2, spoiler, e1, e2, e3, ths, r1, r2, ra_buses) = match i {
            0 => (
                hyd(n, vars, "AILERON_LEFT_INWARD"),
                hyd(n, vars, "AILERON_RIGHT_INWARD"),
                hyd(n, vars, "AILERON_LEFT_MIDDLE"),
                hyd(n, vars, "AILERON_RIGHT_MIDDLE"),
                6,
                hyd(n, vars, "ELEVATOR_LEFT_OUTWARD"),
                hyd(n, vars, "ELEVATOR_LEFT_INWARD"),
                hyd(n, vars, "ELEVATOR_RIGHT_OUTWARD"),
                -n.get(vars, "A32NX_HYD_FINAL_THS_DEFLECTION"),
                hyd(n, vars, "UPPER_RUDDER"),
                hyd(n, vars, "LOWER_RUDDER"),
                (0, 2),
            ),
            1 => (
                hyd(n, vars, "AILERON_LEFT_OUTWARD"),
                hyd(n, vars, "AILERON_RIGHT_OUTWARD"),
                hyd(n, vars, "AILERON_LEFT_INWARD"),
                hyd(n, vars, "AILERON_RIGHT_INWARD"),
                5,
                hyd(n, vars, "ELEVATOR_RIGHT_OUTWARD"),
                hyd(n, vars, "ELEVATOR_LEFT_OUTWARD"),
                hyd(n, vars, "ELEVATOR_RIGHT_INWARD"),
                0.,
                hyd(n, vars, "UPPER_RUDDER"),
                0.,
                (1, 2),
            ),
            _ => (
                hyd(n, vars, "AILERON_LEFT_MIDDLE"),
                hyd(n, vars, "AILERON_RIGHT_MIDDLE"),
                hyd(n, vars, "AILERON_LEFT_OUTWARD"),
                hyd(n, vars, "AILERON_RIGHT_OUTWARD"),
                4,
                hyd(n, vars, "ELEVATOR_LEFT_INWARD"),
                hyd(n, vars, "ELEVATOR_RIGHT_INWARD"),
                0.,
                -n.get(vars, "A32NX_HYD_FINAL_THS_DEFLECTION"),
                hyd(n, vars, "LOWER_RUDDER"),
                0.,
                (0, 1),
            ),
        };
        let n = &mut self.names;
        let left_spoiler = n.get(vars, &format!("A32NX_HYD_SPOILER_{spoiler}_LEFT_DEFLECTION"));
        let right_spoiler = n.get(vars, &format!("A32NX_HYD_SPOILER_{spoiler}_RIGHT_DEFLECTION"));

        // cpp:1538-1539
        let athr_instinctive_disc =
            events.throttles.athr_disconnect || n.get(vars, "A32NX_AUTOTHRUST_DISCONNECT") == 1.;
        let ap_instinctive_disc = events.autopilot.ap_disconnect;

        let mut input = PrimInputs::default();
        // cpp:1543-1550; slew and pause never reach an update here, as in
        // FBW (cpp:82-87), and TAILSTRIKE_PROTECTION_ENABLED defaults off.
        input.time = BaseTime { dt, simulation_time, monotonic_time: self.monotonic_time };
        input.sim_data.tracking_mode_on_override = (n.get(vars, "A32NX_EXTERNAL_OVERRIDE") == 1.) as u8;
        // cpp:1552-1555
        input.sim_input.spd_mach = events.autopilot.spd_mach_set as f32;
        input.sim_input.hdg_trk = events.autopilot.hdg_trk_set as f32;
        input.sim_input.alt = events.autopilot.alt_set as f32;
        input.sim_input.vs_fpa = events.autopilot.vs_fpa_set as f32;

        // cpp:1557-1579
        let att_hdg = n.get(vars, "A32NX_ATT_HDG_SWITCHING_KNOB");
        let air_data = n.get(vars, "A32NX_AIR_DATA_SWITCHING_KNOB");
        let d = &mut input.discrete_inputs;
        d.prim_overhead_button_pressed = b(n.get(vars, &format!("A32NX_PRIM_{}_PUSHBUTTON_PRESSED", i + 1)));
        d.is_unit_1 = (i == 0) as u8;
        d.is_unit_2 = (i == 1) as u8;
        d.is_unit_3 = (i == 2) as u8;
        d.capt_priority_takeover_pressed = (n.get(vars, "A32NX_PRIORITY_TAKEOVER:1") != 0. || ap_instinctive_disc) as u8;
        d.fo_priority_takeover_pressed = b(n.get(vars, "A32NX_PRIORITY_TAKEOVER:2"));
        d.ap_1_pushbutton_pressed = events.autopilot.ap_1_push as u8;
        d.ap_2_pushbutton_pressed = events.autopilot.ap_2_push as u8;
        d.fcu_1_healthy = 1;
        d.fcu_2_healthy = 1;
        d.athr_pushbutton = events.throttles.athr_push as u8;
        d.ir_3_on_capt = (att_hdg == 0.) as u8;
        d.ir_3_on_fo = (att_hdg == 2.) as u8;
        d.adr_3_on_capt = (air_data == 0.) as u8;
        d.adr_3_on_fo = (air_data == 2.) as u8;
        d.rat_deployed = (i == 0 && n.get(vars, "A32NX_RAT_STOW_POSITION") > 0.9) as u8;
        d.rat_contactor_closed = if i == 0 { b(n.get(vars, "A32NX_ELEC_CONTACTOR_5XE_IS_CLOSED")) } else { 0 };
        d.athr_instinctive_disc = athr_instinctive_disc as u8;
        // SimInputPitchTrim (cpp:1576-1577): the manual pitch trim switch,
        // read once per tick by update_with into self.trim.
        d.pitch_trim_up_pressed = self.trim.pitch_up as u8;
        d.pitch_trim_down_pressed = self.trim.pitch_down as u8;
        d.green_low_pressure = (n.get(vars, "A32NX_HYD_GREEN_SYSTEM_1_SECTION_PRESSURE_SWITCH") == 0.) as u8;
        d.yellow_low_pressure = (n.get(vars, "A32NX_HYD_YELLOW_SYSTEM_1_SECTION_PRESSURE_SWITCH") == 0.) as u8;

        // cpp:1581-1614
        let rudder_trim = n.get(vars, "A32NX_RUDDER_TRIM_ACTUAL_POSITION");
        let a = &mut input.analog_inputs;
        a.capt_pitch_stick_pos = -readings.inputs[0];
        a.capt_roll_stick_pos = -readings.inputs[1];
        a.speed_brake_lever_pos = if readings.spoilers_armed { -0.05 } else { readings.spoilers_handle_position };
        a.thr_lever_1_pos = n.get(vars, "A32NX_AUTOTHRUST_TLA:1");
        a.thr_lever_2_pos = n.get(vars, "A32NX_AUTOTHRUST_TLA:2");
        a.thr_lever_3_pos = n.get(vars, "A32NX_AUTOTHRUST_TLA:3");
        a.thr_lever_4_pos = n.get(vars, "A32NX_AUTOTHRUST_TLA:4");
        a.elevator_1_pos_deg = -30. * e1;
        a.elevator_2_pos_deg = -30. * e2;
        a.elevator_3_pos_deg = -30. * e3;
        a.ths_pos_deg = ths;
        a.left_aileron_1_pos_deg = 30. * la1;
        a.left_aileron_2_pos_deg = 30. * la2;
        a.right_aileron_1_pos_deg = -30. * ra1;
        a.right_aileron_2_pos_deg = -30. * ra2;
        a.left_spoiler_pos_deg = -50. * left_spoiler;
        a.right_spoiler_pos_deg = -50. * right_spoiler;
        a.rudder_1_pos_deg = -30. * r1;
        a.rudder_2_pos_deg = -30. * r2;
        a.rudder_pedal_pos = -(readings.inputs[2] + rudder_trim / 30.);
        a.yellow_hyd_pressure_psi = n.get(vars, "A32NX_HYD_YELLOW_SYSTEM_1_SECTION_PRESSURE");
        a.green_hyd_pressure_psi = n.get(vars, "A32NX_HYD_GREEN_SYSTEM_1_SECTION_PRESSURE");
        a.left_body_wheel_speed = n.get(vars, "A32NX_WHEEL_RPM_1") * 0.146189;
        a.left_wing_wheel_speed = n.get(vars, "A32NX_WHEEL_RPM_3") * 0.146189;
        a.right_body_wheel_speed = n.get(vars, "A32NX_WHEEL_RPM_2") * 0.146189;
        a.right_wing_wheel_speed = n.get(vars, "A32NX_WHEEL_RPM_4") * 0.146189;

        // cpp:1616-1653
        let bus = &mut input.bus_inputs;
        bus.adr_1_bus = self.adr[0];
        bus.adr_2_bus = self.adr[1];
        bus.adr_3_bus = self.adr[2];
        bus.ir_1_bus = self.ir[0];
        bus.ir_2_bus = self.ir[1];
        bus.ir_3_bus = self.ir[2];
        bus.ra_1_bus = self.ra[ra_buses.0];
        bus.ra_2_bus = self.ra[ra_buses.1];
        bus.ils_1_bus = self.ils[0];
        bus.ils_2_bus = self.ils[1];
        bus.sfcc_1_bus = self.sfcc[0];
        bus.sfcc_2_bus = self.sfcc[1];
        bus.lgciu_1_bus = self.lgciu[0];
        bus.lgciu_2_bus = self.lgciu[1];
        bus.fcu_1_bus = self.fcu_buses[0];
        bus.fcu_2_bus = self.fcu_buses[1];
        let (x, y) = match i {
            0 => (1, 2),
            1 => (0, 2),
            _ => (0, 1),
        };
        // The other PRIMs as they are now: already this tick's for lower
        // numbers, still last tick's for higher.
        bus.prim_x_bus = self.prim_buses[x];
        bus.prim_y_bus = self.prim_buses[y];
        bus.sec_1_bus = self.sec_buses[0];
        bus.sec_2_bus = self.sec_buses[1];
        bus.sec_3_bus = self.sec_buses[2];

        // cpp:1655-1696
        let fm = &mut input.adcn_inputs.fms;
        fm.fm_valid = 1;
        fm.active_fms_flight_phase = n.get(vars, "A32NX_FMGC_FLIGHT_PHASE") as i32;
        fm.selected_approach_type = if n.get(vars, "A32NX_FG_RNAV_APP_SELECTED") != 0. { 2 } else { 1 };
        fm.backbeam_selected = 0;
        // localizer.distance (CalculatedRadioReceiver.cpp:1158): the PRIMs'
        // own ILS receiver's localizer-without-DME range, from receiver 3's
        // DME the same way A32NX_RADIO_RECEIVER_LOC_DISTANCE above does.
        fm.fms_loc_distance = if self.nav.dme_valid { self.nav.dme_nmi } else { 0. };
        fm.fms_unrealistic_gs_angle_deg = if self.nav.gs_valid { -self.nav.gs_deg } else { 0. };
        fm.lateral_flight_plan_valid = b(n.get(vars, "A32NX_FM_LATERAL_FLIGHTPLAN_AVAIL"));
        fm.nav_capture_condition = b(n.get(vars, "A32NX_FM1_NAV_CAPTURE_CONDITION"));
        fm.phi_c_deg = n.get(vars, "A32NX_FG_PHI_COMMAND");
        fm.xtk_nmi = n.get(vars, "A32NX_FG_CROSS_TRACK_ERROR");
        fm.tke_deg = n.get(vars, "A32NX_FG_TRACK_ANGLE_ERROR");
        fm.phi_limit_deg = n.get(vars, "A32NX_FG_PHI_LIMIT");
        fm.direct_to_nav_engage = events.autopilot.dir_to_trigger as u8;
        fm.vertical_flight_plan_valid = b(n.get(vars, "A32NX_FM_VERTICAL_PROFILE_AVAIL"));
        fm.final_app_can_engage = b(n.get(vars, "A32NX_FG_FINAL_CAN_ENGAGE"));
        fm.next_alt_cstr_ft = n.get(vars, "A32NX_FG_ALTITUDE_CONSTRAINT");
        fm.requested_des_submode = n.get(vars, "A32NX_FG_REQUESTED_VERTICAL_MODE") as i32;
        fm.alt_profile_tgt_ft = n.get(vars, "A32NX_FG_TARGET_ALTITUDE");
        fm.vs_target_ft_min = n.get(vars, "A32NX_FG_TARGET_VERTICAL_SPEED");
        fm.v_2_kts = n.get(vars, "AIRLINER_V2_SPEED");
        fm.v_app_kts = n.get(vars, "A32NX_SPEEDS_VAPP");
        fm.v_managed_kts = n.get(vars, "A32NX_SPEEDS_MANAGED_PFD");
        fm.v_upper_margin_kts = n.get(vars, "A32NX_PFD_UPPER_SPEED_MARGIN");
        fm.v_lower_margin_kts = n.get(vars, "A32NX_PFD_LOWER_SPEED_MARGIN");
        fm.show_speed_margins = b(n.get(vars, "A32NX_PFD_SHOW_SPEED_MARGINS"));
        fm.preset_spd_kts = n.get(vars, "A32NX_SpeedPreselVal");
        fm.preset_mach = n.get(vars, "A32NX_MachPreselVal");
        fm.preset_spd_mach_activate = events.autopilot.preset_spd_activate as u8;
        fm.fms_spd_mode_activate = events.autopilot.spd_mode_activate as u8;
        fm.fms_mach_mode_activate = events.autopilot.mach_mode_activate as u8;
        fm.flex_temp_deg_c = n.get(vars, "A32NX_AIRLINER_TO_FLEX_TEMP");
        // Arinc429NumericWord::valueOr(0) (cpp:1014-1015, 1686-1687).
        fm.acceleration_alt_ft = value_or(n.word(vars, "A32NX_FM1_ACC_ALT"), 0.) as f64;
        fm.thrust_reduction_alt_ft = value_or(n.word(vars, "A32NX_FM1_THR_RED_ALT"), 0.) as f64;
        fm.cruise_alt_ft = n.get(vars, "A32NX_AIRLINER_CRUISE_ALTITUDE");
        fm.tower_headwind_kn = n.word(vars, "A380X_FM_APPROACH_HEADWIND_COMPONENT");
        fm.flap_3_approach_selected = b(n.get(vars, "A380X_FM_LANDING_CONF3"));
        input.adcn_inputs.fqms = self.fqms;
        input.adcn_inputs.eec_1 = self.fadec_buses[0];
        input.adcn_inputs.eec_2 = self.fadec_buses[1];
        input.adcn_inputs.eec_3 = self.fadec_buses[2];
        input.adcn_inputs.eec_4 = self.fadec_buses[3];
        input.adcn_inputs.tcas = self.tcas;

        // cpp:1702-1715
        let power = ["A32NX_ELEC_108PH_BUS_IS_POWERED", "A32NX_ELEC_247PP_BUS_IS_POWERED", "A32NX_ELEC_DC_1_BUS_IS_POWERED"][i];
        let powered = n.get(vars, power) != 0.;
        self.prims[i].set_inputs(&input);
        // cpp:1711: failuresConsumer.isActive(Prim1/Prim2/Prim3).
        self.prims[i].update(dt, simulation_time, fault_active, powered);

        // cpp:1717-1719
        self.prim_discrete[i] = self.prims[i].discrete_outputs();
        self.prim_analog[i] = self.prims[i].analog_outputs();
        self.prim_buses[i] = self.prims[i].bus_outputs();

        // [diagnostic] One-time per unit: PRIM_3/SEC_3 have been observed
        // unhealthy while `powered` (A32NX_ELEC_DC_1_BUS_IS_POWERED = 1),
        // causing spurious PRIM 3/SEC 3, BTV and ROW/ROP ECAM faults. The
        // health bit itself is computed inside the compiled Simulink model
        // (fbw_a380/src/prim/Prim.cpp via fbw_prim_discrete_outputs), so
        // this dumps `PrimDiagnostics` (triple ADR/IR/RA loss, all-SFCC-
        // lost) the moment it happens, to find which input is actually
        // gating health besides `powered`. Remove once root-caused.
        {
            static LOGGED: [std::sync::atomic::AtomicBool; 3] =
                [const { std::sync::atomic::AtomicBool::new(false) }; 3];
            if powered && self.prim_discrete[i].prim_healthy == 0 && !LOGGED[i].swap(true, std::sync::atomic::Ordering::Relaxed) {
                let d = self.prims[i].diagnostics();
                crate::log(&format!(
                    "js: PRIM {} unhealthy while powered ({power}=1): triple_adr_failure={} triple_ir_failure={} all_sfcc_lost={} all_ra_failure={} speed_scale_lost={}",
                    i + 1,
                    d.triple_adr_failure,
                    d.triple_ir_failure,
                    d.all_sfcc_lost,
                    d.all_ra_failure,
                    d.speed_scale_lost,
                ));
            }
        }

        // cpp:1721-1769
        let p = i + 1;
        let o = self.prim_buses[i];
        let n = &mut self.names;
        n.set(vars, &format!("A32NX_PRIM_{p}_HEALTHY"), self.prim_discrete[i].prim_healthy as f64);
        n.set(vars, &format!("A32NX_PRIM_{p}_AP_ENGAGED"), self.prim_discrete[i].ap_engaged as f64);
        for (name, word) in [
            ("FCTL_LAW_STATUS_WORD", o.fctl.fctl_law_status_word),
            ("GAMMA_A", o.fe.gamma_a_deg),
            ("GAMMA_T", o.fe.gamma_t_deg),
            ("SIDESLIP_TARGET", o.fe.sideslip_target_deg),
            ("V_ALPHA_LIM", o.fctl.v_alpha_lim_kn),
            ("V_LS", o.fe.v_ls_kn),
            ("V_STALL_1G", o.fe.v_stall_kn),
            ("V_ALPHA_PROT", o.fctl.v_alpha_prot_kn),
            ("V_STALL_WARN", o.fctl.v_alpha_stall_warn_kn),
            ("SPEED_TREND", o.fe.speed_trend_kn),
            ("V_3", o.fe.v_3_kn),
            ("V_4", o.fe.v_4_kn),
            ("V_MAN", o.fe.v_man_kn),
            ("V_MAX", o.fe.v_max_kn),
            ("V_FE_NEXT", o.fe.v_fe_next_kn),
            ("PFD_SELECTED_SPEED", o.fg.pfd_spd_tgt_kts),
            ("PFD_SHORT_TERM_MANAGED_SPEED", o.fg.pfd_short_term_mngd_spd_kts),
            ("SELECTED_AIRSPEED", o.fg.selected_spd_kts),
            ("SELECTED_MACH", o.fg.selected_mach_kts),
            ("SELECTED_HEADING", o.fg.selected_hdg_deg),
            ("SELECTED_TRACK", o.fg.selected_trk_deg),
            ("SELECTED_ALTITUDE", o.fg.selected_alt_ft),
            ("SELECTED_VERTICAL_SPEED", o.fg.selected_vs_ft_min),
            ("SELECTED_FPA", o.fg.selected_fpa_deg),
            ("PRESEL_MACH", o.fg.preset_mach_from_fms),
            ("PRESEL_SPEED", o.fg.preset_speed_from_fms_kts),
            ("RWY_HDG_MEMO", o.fg.runway_hdg_memorized_deg),
            ("ROLL_FD_COMMAND_1", o.fg.roll_fd_command_1),
            ("PITCH_FD_COMMAND_1", o.fg.pitch_fd_command_1),
            ("YAW_FD_COMMAND_1", o.fg.yaw_fd_command_1),
            ("ROLL_FD_COMMAND_2", o.fg.roll_fd_command_2),
            ("PITCH_FD_COMMAND_2", o.fg.pitch_fd_command_2),
            ("YAW_FD_COMMAND_2", o.fg.yaw_fd_command_2),
            ("FM_ALTITUDE_CONSTRAINT", o.fg.fm_alt_constraint_ft),
            ("FG_ATS_DISCRETE_WORD", o.fg.ats_discrete_word),
            ("FG_ATS_FMA_DISCRETE_WORD", o.fg.ats_fma_discrete_word),
            ("FG_DISCRETE_WORD_1", o.fg.discrete_word_1),
            ("FG_DISCRETE_WORD_2", o.fg.discrete_word_2),
            ("FG_DISCRETE_WORD_3", o.fg.discrete_word_3),
            ("FG_DISCRETE_WORD_4", o.fg.discrete_word_4),
            ("FG_DISCRETE_WORD_5", o.fg.discrete_word_5),
            ("FG_DISCRETE_WORD_6", o.fg.discrete_word_6),
            ("SPEED_MARGIN_HIGH", o.fg.high_target_speed_margin_kts),
            ("SPEED_MARGIN_LOW", o.fg.low_target_speed_margin_kts),
        ] {
            n.set(vars, &format!("A32NX_PRIM_{p}_{name}"), to_simvar(word));
        }
    }

    fn update_prim_fg_shim<V: VariableRegistry + SimulatorReaderWriter>(
        &mut self,
        vars: &mut V,
        readings: &SimReadings,
        dt: f64,
    ) {
        // cpp:1776-1786
        let master = if bit_or(self.prim_buses[0].fctl.fctl_law_status_word, 21, false) {
            0
        } else if bit_or(self.prim_buses[1].fctl.fctl_law_status_word, 21, false) {
            1
        } else {
            2
        };
        let fg = self.prim_buses[master].fg;
        let w1 = |bit| bit_or(fg.discrete_word_1, bit, false);
        let w2 = |bit| bit_or(fg.discrete_word_2, bit, false);
        let w3 = |bit| bit_or(fg.discrete_word_3, bit, false);
        let w4 = |bit| bit_or(fg.discrete_word_4, bit, false);
        let ats = |bit| bit_or(fg.ats_discrete_word, bit, false);
        let fma = |bit| bit_or(fg.ats_fma_discrete_word, bit, false);

        let ap1 = w1(11);
        let ap2 = w1(12);
        // cpp:1791-1819
        let lateral_mode = if w4(16) {
            10
        } else if w4(17) {
            11
        } else if w4(12) {
            20
        } else if w4(13) {
            30
        } else if w4(14) {
            31
        } else if w1(23) && !w3(24) && !w4(26) {
            32
        } else if w3(24) && !w4(26) {
            33
        } else if w4(26) {
            34
        } else if w4(11) && w4(18) {
            40
        } else if w4(11) && w4(19) {
            41
        } else if w4(15) {
            50
        } else {
            0
        };
        let lateral_armed = w2(22) as i32 | (w2(23) as i32) << 1;
        // cpp:1825-1884. FBW's chain assigns lateralMode, not verticalMode,
        // in its three LAND/FLARE/ROLL OUT branches (cpp:1869-1877); a
        // lateral mode already chosen above is then overwritten there, and
        // verticalMode stays 0. Kept as written.
        let alt_constraint_valid = fg.fm_alt_constraint_ft.SSM == SSM_NO;
        let (alt_hold, alt_acq, dash) = (w3(20), w3(19), w3(26));
        let mut lateral_mode = lateral_mode;
        let vertical_mode = if alt_hold && !dash && !alt_constraint_valid {
            10
        } else if alt_acq && !dash && !alt_constraint_valid {
            11
        } else if w3(13) {
            12
        } else if w3(14) {
            13
        } else if w3(17) {
            14
        } else if w3(18) {
            15
        } else if alt_hold && !dash && alt_constraint_valid {
            20
        } else if alt_acq && !dash && alt_constraint_valid {
            21
        } else if w3(11) {
            22
        } else if w3(12) {
            23
        } else if w3(23) {
            24
        } else if w3(21) {
            30
        } else if w3(22) {
            31
        } else if w1(23) && !w3(24) && !w4(26) {
            lateral_mode = 32;
            0
        } else if w3(24) && !w4(26) {
            lateral_mode = 33;
            0
        } else if w4(26) {
            lateral_mode = 34;
            0
        } else if w3(15) {
            40
        } else if w3(16) {
            41
        } else if w3(25) {
            50
        } else {
            0
        };
        // cpp:1886-1892
        let vertical_armed = w2(11) as i32
            | (w2(15) as i32) << 2
            | (w2(16) as i32) << 3
            | (w2(13) as i32) << 4
            | (w2(14) as i32) << 5
            | (w2(18) as i32) << 6;
        // cpp:1894-1937
        let (at_engaged, at_active) = (ats(11), ats(12));
        let athr_status = if at_engaged && !at_active {
            1
        } else if at_engaged && at_active {
            2
        } else {
            0
        };
        let athr_mode = if fma(11) {
            1
        } else if fma(13) {
            3
        } else if fma(12) && at_engaged && !at_active {
            5
        } else if fma(15) && at_engaged && !at_active {
            6
        } else if fma(19) {
            7
        } else if fma(20) {
            8
        } else if fma(12) && at_engaged && at_active {
            9
        } else if fma(14) {
            10
        } else if fma(15) && at_engaged && at_active {
            11
        } else if fma(16) {
            12
        } else if fma(17) {
            13
        } else if fma(18) {
            14
        } else {
            0
        };
        let athr_message = if fma(26) {
            3
        } else if fma(27) {
            4
        } else if fma(25) {
            5
        } else {
            0
        };

        // cpp:1942-1958, with the raw receiver 3 errors (not the corrected
        // localiser error the ILS bus carries).
        let nav = self.nav;
        let n = &mut self.names;
        let h_radio = readings.h_radio_ft;
        if h_radio < 200. && self.prim_discrete[master].ap_engaged != 0 && (vertical_mode == 32 || vertical_mode == 33) {
            self.autoland_warning_latch = true;
        } else if h_radio >= 200. || (vertical_mode != 32 && vertical_mode != 33) {
            self.autoland_warning_latch = false;
            self.autoland_warning_triggered = false;
            n.set(vars, "A32NX_AUTOPILOT_AUTOLAND_WARNING", 0.);
        }
        if self.autoland_warning_latch
            && !self.autoland_warning_triggered
            && autoland_warning_condition(ap1 || ap2, h_radio, &nav)
        {
            self.autoland_warning_triggered = true;
            n.set(vars, "A32NX_AUTOPILOT_AUTOLAND_WARNING", 1.);
        }

        // cpp:1961-1964
        let k = 1. / 15.;
        let y = 1. / (dt + k) * (h_radio - self.h_dot_filter_prev_u + k * self.h_dot_filter_prev_y);
        self.h_dot_filter_prev_u = h_radio;
        self.h_dot_filter_prev_y = y;

        // cpp:1966-1977
        n.set(vars, "A32NX_AUTOPILOT_NOSEWHEEL_DEMAND", value_or(fg.nosewheel_cmd_deg, 0.) as f64);
        n.set(vars, "A32NX_FMA_LATERAL_MODE", lateral_mode as f64);
        n.set(vars, "A32NX_FMA_LATERAL_ARMED", lateral_armed as f64);
        n.set(vars, "A32NX_FMA_VERTICAL_MODE", vertical_mode as f64);
        n.set(vars, "A32NX_FMA_VERTICAL_ARMED", vertical_armed as f64);
        n.set(vars, "A32NX_AUTOPILOT_ACTIVE", f(ap1 || ap2));
        n.set(vars, "A32NX_AUTOPILOT_1_ACTIVE", f(ap1));
        n.set(vars, "A32NX_AUTOPILOT_2_ACTIVE", f(ap2));
        n.set(vars, "A32NX_AUTOPILOT_H_DOT_RADIO", y * 60.);
        n.set(vars, "A32NX_AUTOTHRUST_STATUS", athr_status as f64);
        n.set(vars, "A32NX_AUTOTHRUST_MODE", athr_mode as f64);
        n.set(vars, "A32NX_AUTOTHRUST_MODE_MESSAGE", athr_message as f64);

        // cpp:1980-1986, from PRIM 1 whichever is master.
        let flare = self.prims[0].flare_law();
        let n = &mut self.names;
        n.set(vars, "A32NX_DEV_FLARE_H_DOT", flare.H_dot_radio_fpm);
        n.set(vars, "A32NX_DEV_FLARE_H_DOT_C", flare.H_dot_c_fpm);
        n.set(vars, "A32NX_DEV_FLARE_CONDITION", flare.condition_Flare as f64);
        n.set(vars, "A32NX_DEV_FLARE_DELTA_THETA_H_DOT", flare.delta_Theta_H_dot_deg);
        n.set(vars, "A32NX_DEV_FLARE_DELTA_THETA_BZ", flare.delta_Theta_bz_deg);
        n.set(vars, "A32NX_DEV_FLARE_DELTA_THETA_BX", flare.delta_Theta_bx_deg);
        n.set(vars, "A32NX_DEV_FLARE_DELTA_THETA_BETA_C", flare.delta_Theta_beta_c_deg);

        // cpp:1989-2004; the AP_*_SLOT_INDEX_SET and AP_SPD_VAR_SET events are
        // MSFS autopilot slots.
        n.set(vars, "A32NX_FCU_SPD_MANAGED_DOT", f(bit_or(fg.discrete_word_5, 17, false)));
        n.set(vars, "A32NX_FCU_HDG_MANAGED_DOT", 0.);
        n.set(vars, "A32NX_FCU_ALT_MANAGED", 0.);
    }

    // -- SEC -------------------------------------------------------------------

    #[allow(clippy::too_many_arguments)]
    fn update_sec<V: VariableRegistry + SimulatorReaderWriter>(
        &mut self,
        vars: &mut V,
        readings: &SimReadings,
        dt: f64,
        simulation_time: f64,
        i: usize,
        fault_active: bool,
    ) {
        let n = &mut self.names;
        let hyd = |n: &mut Names, vars: &mut V, s: &str| n.get(vars, &format!("A32NX_HYD_{s}_DEFLECTION"));
        let spoiler = |n: &mut Names, vars: &mut V, k: usize, side: &str| {
            n.get(vars, &format!("A32NX_HYD_SPOILER_{k}_{side}_DEFLECTION"))
        };
        let ths_deg = -n.get(vars, "A32NX_HYD_FINAL_THS_DEFLECTION");
        // cpp:2036-2093
        let (la1, ra1, la2, ra2, ls1, rs1, ls2, rs2, e1, e2, e3, ths, r1, r2) = match i {
            0 => (
                hyd(n, vars, "AILERON_LEFT_INWARD"),
                hyd(n, vars, "AILERON_RIGHT_INWARD"),
                hyd(n, vars, "AILERON_LEFT_MIDDLE"),
                hyd(n, vars, "AILERON_RIGHT_MIDDLE"),
                spoiler(n, vars, 3, "LEFT"),
                spoiler(n, vars, 3, "RIGHT"),
                0.,
                0.,
                hyd(n, vars, "ELEVATOR_LEFT_OUTWARD"),
                hyd(n, vars, "ELEVATOR_LEFT_INWARD"),
                hyd(n, vars, "ELEVATOR_RIGHT_OUTWARD"),
                ths_deg,
                hyd(n, vars, "LOWER_RUDDER"),
                hyd(n, vars, "UPPER_RUDDER"),
            ),
            1 => (
                hyd(n, vars, "AILERON_LEFT_OUTWARD"),
                hyd(n, vars, "AILERON_RIGHT_OUTWARD"),
                hyd(n, vars, "AILERON_LEFT_INWARD"),
                hyd(n, vars, "AILERON_RIGHT_INWARD"),
                spoiler(n, vars, 2, "LEFT"),
                spoiler(n, vars, 2, "RIGHT"),
                spoiler(n, vars, 7, "LEFT"),
                spoiler(n, vars, 7, "RIGHT"),
                hyd(n, vars, "ELEVATOR_RIGHT_OUTWARD"),
                hyd(n, vars, "ELEVATOR_LEFT_OUTWARD"),
                hyd(n, vars, "ELEVATOR_RIGHT_INWARD"),
                0.,
                hyd(n, vars, "UPPER_RUDDER"),
                0.,
            ),
            _ => (
                hyd(n, vars, "AILERON_LEFT_MIDDLE"),
                hyd(n, vars, "AILERON_RIGHT_MIDDLE"),
                hyd(n, vars, "AILERON_LEFT_OUTWARD"),
                hyd(n, vars, "AILERON_RIGHT_OUTWARD"),
                spoiler(n, vars, 1, "LEFT"),
                spoiler(n, vars, 1, "RIGHT"),
                spoiler(n, vars, 8, "LEFT"),
                spoiler(n, vars, 8, "RIGHT"),
                hyd(n, vars, "ELEVATOR_LEFT_INWARD"),
                hyd(n, vars, "ELEVATOR_RIGHT_INWARD"),
                0.,
                ths_deg,
                hyd(n, vars, "LOWER_RUDDER"),
                0.,
            ),
        };

        let mut input = SecInputs::default();
        // cpp:2095-2102
        input.time = BaseTime { dt, simulation_time, monotonic_time: self.monotonic_time };
        input.sim_data.tracking_mode_on_override = (n.get(vars, "A32NX_EXTERNAL_OVERRIDE") == 1.) as u8;
        // cpp:2104-2118
        let d = &mut input.discrete_inputs;
        d.sec_overhead_button_pressed = b(n.get(vars, &format!("A32NX_SEC_{}_PUSHBUTTON_PRESSED", i + 1)));
        d.is_unit_1 = (i == 0) as u8;
        d.is_unit_2 = (i == 1) as u8;
        d.is_unit_3 = (i == 2) as u8;
        d.capt_priority_takeover_pressed = b(n.get(vars, "A32NX_PRIORITY_TAKEOVER:1"));
        d.fo_priority_takeover_pressed = b(n.get(vars, "A32NX_PRIORITY_TAKEOVER:2"));
        d.rat_deployed = (i == 0 && n.get(vars, "A32NX_RAT_STOW_POSITION") > 0.9) as u8;
        d.rat_contactor_closed = if i == 0 { b(n.get(vars, "A32NX_ELEC_CONTACTOR_5XE_IS_CLOSED")) } else { 0 };
        d.green_low_pressure = (n.get(vars, "A32NX_HYD_GREEN_SYSTEM_1_SECTION_PRESSURE_SWITCH") == 0.) as u8;
        d.yellow_low_pressure = (n.get(vars, "A32NX_HYD_YELLOW_SYSTEM_1_SECTION_PRESSURE_SWITCH") == 0.) as u8;
        // SimInputPitchTrim/SimInputRudderTrim (cpp:2110-2114): the manual
        // trim switches, read once per tick by update_with into self.trim.
        d.pitch_trim_up_pressed = self.trim.pitch_up as u8;
        d.pitch_trim_down_pressed = self.trim.pitch_down as u8;
        d.rudder_trim_left_pressed = self.trim.rudder_left as u8;
        d.rudder_trim_right_pressed = self.trim.rudder_right as u8;
        d.rudder_trim_reset_pressed = self.trim.rudder_reset as u8;
        // cpp:2120-2139
        let rudder_trim = n.get(vars, "A32NX_RUDDER_TRIM_ACTUAL_POSITION");
        let a = &mut input.analog_inputs;
        a.capt_pitch_stick_pos = -readings.inputs[0];
        a.capt_roll_stick_pos = -readings.inputs[1];
        a.elevator_1_pos_deg = -30. * e1;
        a.elevator_2_pos_deg = -30. * e2;
        a.elevator_3_pos_deg = -30. * e3;
        a.ths_pos_deg = ths;
        a.left_aileron_1_pos_deg = 30. * la1;
        a.left_aileron_2_pos_deg = 30. * la2;
        a.right_aileron_1_pos_deg = -30. * ra1;
        a.right_aileron_2_pos_deg = -30. * ra2;
        a.left_spoiler_1_pos_deg = -50. * ls1;
        a.right_spoiler_1_pos_deg = -50. * rs1;
        a.left_spoiler_2_pos_deg = -50. * ls2;
        a.right_spoiler_2_pos_deg = -50. * rs2;
        a.rudder_1_pos_deg = -30. * r1;
        a.rudder_2_pos_deg = -30. * r2;
        a.rudder_pedal_pos_deg = -(readings.inputs[2] + rudder_trim / 30.);
        a.rudder_trim_actual_pos_deg = rudder_trim;
        // cpp:2141-2181
        let (adr1, adr2) = [(0, 1), (1, 2), (0, 2)][i];
        let bus = &mut input.bus_inputs;
        bus.adr_1_bus = self.adr[adr1];
        bus.adr_2_bus = self.adr[adr2];
        bus.ir_1_bus = self.ir[adr1];
        bus.ir_2_bus = self.ir[adr2];
        bus.sfcc_1_bus = self.sfcc[0];
        bus.sfcc_2_bus = self.sfcc[1];
        bus.lgciu_1_bus = self.lgciu[0];
        bus.lgciu_2_bus = self.lgciu[1];
        bus.prim_1_bus = self.prim_buses[0];
        bus.prim_2_bus = self.prim_buses[1];
        bus.prim_3_bus = self.prim_buses[2];
        let (x, y) = [(1, 2), (0, 2), (0, 1)][i];
        bus.sec_x_bus = self.sec_buses[x];
        bus.sec_y_bus = self.sec_buses[y];
        input.adcn_inputs.eec_1 = self.fadec_buses[0];
        input.adcn_inputs.eec_2 = self.fadec_buses[1];
        input.adcn_inputs.eec_3 = self.fadec_buses[2];
        input.adcn_inputs.eec_4 = self.fadec_buses[3];

        // cpp:2191-2205
        let power = ["A32NX_ELEC_108PH_BUS_IS_POWERED", "A32NX_ELEC_247PP_BUS_IS_POWERED", "A32NX_ELEC_DC_1_BUS_IS_POWERED"][i];
        let powered = n.get(vars, power) != 0.;
        self.secs[i].set_inputs(&input);
        // cpp:2200: failuresConsumer.isActive(Sec1/Sec2/Sec3).
        self.secs[i].update(dt, simulation_time, fault_active, powered);
        self.sec_discrete[i] = self.secs[i].discrete_outputs();
        self.sec_analog[i] = self.secs[i].analog_outputs();
        self.sec_buses[i] = self.secs[i].bus_outputs();

        // cpp:2212-2214
        let s = i + 1;
        let n = &mut self.names;
        n.set(vars, &format!("A32NX_SEC_{s}_HEALTHY"), self.sec_discrete[i].sec_healthy as f64);
        n.set(vars, &format!("A32NX_SEC_{s}_RUDDER_STATUS_WORD"), to_simvar(self.sec_buses[i].rudder_status_word));
        n.set(vars, &format!("A32NX_SEC_{s}_RUDDER_ACTUAL_POSITION"), to_simvar(self.sec_buses[i].rudder_trim_actual_pos_deg));
    }

    // -- surfaces --------------------------------------------------------------

    fn update_servo_solenoid_status<V: VariableRegistry + SimulatorReaderWriter>(&mut self, vars: &mut V) {
        let (pd, pa, sd, sa) = (self.prim_discrete, self.prim_analog, self.sec_discrete, self.sec_analog);
        let n = &mut self.names;
        let mut set = |name: &str, v: f64| n.set(vars, &format!("A32NX_{name}"), v);
        let or = |x: u8, y: u8| f(x != 0 || y != 0);
        // cpp:2732-2773
        set("LEFT_INBOARD_AIL_GREEN_SERVO_SOLENOID_ENERGIZED", or(pd[0].left_aileron_1_active_mode, sd[0].left_aileron_1_active_mode));
        set("LEFT_INBOARD_AIL_GREEN_COMMANDED_POSITION", pa[0].left_aileron_1_pos_order_deg + sa[0].left_aileron_1_pos_order_deg);
        set("LEFT_INBOARD_AIL_EHA_SERVO_SOLENOID_ENERGIZED", or(pd[1].left_aileron_2_active_mode, sd[1].left_aileron_2_active_mode));
        set("LEFT_INBOARD_AIL_EHA_COMMANDED_POSITION", pa[1].left_aileron_2_pos_order_deg + sa[1].left_aileron_2_pos_order_deg);
        set("RIGHT_INBOARD_AIL_GREEN_SERVO_SOLENOID_ENERGIZED", or(pd[0].right_aileron_1_active_mode, sd[0].right_aileron_1_active_mode));
        set("RIGHT_INBOARD_AIL_GREEN_COMMANDED_POSITION", pa[0].right_aileron_1_pos_order_deg + sa[0].right_aileron_1_pos_order_deg);
        set("RIGHT_INBOARD_AIL_EHA_SERVO_SOLENOID_ENERGIZED", or(pd[1].right_aileron_2_active_mode, sd[1].right_aileron_2_active_mode));
        set("RIGHT_INBOARD_AIL_EHA_COMMANDED_POSITION", pa[1].right_aileron_2_pos_order_deg + sa[1].right_aileron_2_pos_order_deg);
        set("LEFT_MIDBOARD_AIL_YELLOW_SERVO_SOLENOID_ENERGIZED", or(pd[2].left_aileron_1_active_mode, sd[2].left_aileron_1_active_mode));
        set("LEFT_MIDBOARD_AIL_YELLOW_COMMANDED_POSITION", pa[2].left_aileron_1_pos_order_deg + sa[2].left_aileron_1_pos_order_deg);
        set("LEFT_MIDBOARD_AIL_EHA_SERVO_SOLENOID_ENERGIZED", or(pd[0].left_aileron_2_active_mode, sd[0].left_aileron_2_active_mode));
        set("LEFT_MIDBOARD_AIL_EHA_COMMANDED_POSITION", pa[0].left_aileron_2_pos_order_deg + sa[0].left_aileron_2_pos_order_deg);
        set("RIGHT_MIDBOARD_AIL_YELLOW_SERVO_SOLENOID_ENERGIZED", or(pd[2].right_aileron_1_active_mode, sd[2].right_aileron_1_active_mode));
        set("RIGHT_MIDBOARD_AIL_YELLOW_COMMANDED_POSITION", pa[2].right_aileron_1_pos_order_deg + sa[2].right_aileron_1_pos_order_deg);
        set("RIGHT_MIDBOARD_AIL_EHA_SERVO_SOLENOID_ENERGIZED", or(pd[0].right_aileron_2_active_mode, sd[0].right_aileron_2_active_mode));
        set("RIGHT_MIDBOARD_AIL_EHA_COMMANDED_POSITION", pa[0].right_aileron_2_pos_order_deg + sa[0].right_aileron_2_pos_order_deg);
        set("LEFT_OUTBOARD_AIL_GREEN_SERVO_SOLENOID_ENERGIZED", pd[1].left_aileron_1_active_mode as f64);
        set("LEFT_OUTBOARD_AIL_GREEN_COMMANDED_POSITION", pa[1].left_aileron_1_pos_order_deg);
        set("LEFT_OUTBOARD_AIL_YELLOW_SERVO_SOLENOID_ENERGIZED", pd[2].left_aileron_2_active_mode as f64);
        set("LEFT_OUTBOARD_AIL_YELLOW_COMMANDED_POSITION", pa[2].left_aileron_2_pos_order_deg);
        set("RIGHT_OUTBOARD_AIL_GREEN_SERVO_SOLENOID_ENERGIZED", pd[1].right_aileron_1_active_mode as f64);
        set("RIGHT_OUTBOARD_AIL_GREEN_COMMANDED_POSITION", pa[1].right_aileron_1_pos_order_deg);
        set("RIGHT_OUTBOARD_AIL_YELLOW_SERVO_SOLENOID_ENERGIZED", pd[2].right_aileron_2_active_mode as f64);
        set("RIGHT_OUTBOARD_AIL_YELLOW_COMMANDED_POSITION", pa[2].right_aileron_2_pos_order_deg);
        // cpp:2775-2792
        let spoilers: [(f64, f64); 8] = [
            (sa[2].left_spoiler_1_pos_order_deg, sa[2].right_spoiler_1_pos_order_deg),
            (sa[1].left_spoiler_1_pos_order_deg, sa[1].right_spoiler_1_pos_order_deg),
            (sa[0].left_spoiler_1_pos_order_deg, sa[0].right_spoiler_1_pos_order_deg),
            (pa[2].left_spoiler_pos_order_deg, pa[2].right_spoiler_pos_order_deg),
            (pa[1].left_spoiler_pos_order_deg, pa[1].right_spoiler_pos_order_deg),
            (pa[0].left_spoiler_pos_order_deg, pa[0].right_spoiler_pos_order_deg),
            (sa[1].left_spoiler_2_pos_order_deg, sa[1].right_spoiler_2_pos_order_deg),
            (sa[2].left_spoiler_2_pos_order_deg, sa[2].right_spoiler_2_pos_order_deg),
        ];
        for (k, (l, r)) in spoilers.iter().enumerate() {
            set(&format!("LEFT_SPOILER_{}_COMMANDED_POSITION", k + 1), -l);
            set(&format!("RIGHT_SPOILER_{}_COMMANDED_POSITION", k + 1), -r);
        }
        set("LEFT_SPOILER_6_EBHA_ELECTRONIC_ENABLE", pd[0].left_spoiler_electronic_module_enable as f64);
        set("RIGHT_SPOILER_6_EBHA_ELECTRONIC_ENABLE", pd[0].right_spoiler_electronic_module_enable as f64);
        // cpp:2794-2826
        set("LEFT_INBOARD_ELEV_GREEN_SERVO_SOLENOID_ENERGIZED", or(pd[2].elevator_1_active_mode, sd[2].elevator_1_active_mode));
        set("LEFT_INBOARD_ELEV_GREEN_COMMANDED_POSITION", pa[2].elevator_1_pos_order_deg + sa[2].elevator_1_pos_order_deg);
        set("LEFT_INBOARD_ELEV_EHA_SERVO_SOLENOID_ENERGIZED", or(pd[0].elevator_2_active_mode, sd[0].elevator_2_active_mode));
        set("LEFT_INBOARD_ELEV_EHA_COMMANDED_POSITION", pa[0].elevator_2_pos_order_deg + sa[0].elevator_2_pos_order_deg);
        set("RIGHT_INBOARD_ELEV_YELLOW_SERVO_SOLENOID_ENERGIZED", or(pd[2].elevator_2_active_mode, sd[2].elevator_2_active_mode));
        set("RIGHT_INBOARD_ELEV_YELLOW_COMMANDED_POSITION", pa[2].elevator_2_pos_order_deg + sa[2].elevator_2_pos_order_deg);
        set("RIGHT_INBOARD_ELEV_EHA_SERVO_SOLENOID_ENERGIZED", or(pd[1].elevator_3_active_mode, sd[1].elevator_3_active_mode));
        set("RIGHT_INBOARD_ELEV_EHA_COMMANDED_POSITION", pa[1].elevator_3_pos_order_deg + sa[1].elevator_3_pos_order_deg);
        set("LEFT_OUTBOARD_ELEV_GREEN_SERVO_SOLENOID_ENERGIZED", or(pd[0].elevator_1_active_mode, sd[0].elevator_1_active_mode));
        set("LEFT_OUTBOARD_ELEV_GREEN_COMMANDED_POSITION", pa[0].elevator_1_pos_order_deg + sa[0].elevator_1_pos_order_deg);
        set("LEFT_OUTBOARD_ELEV_EHA_SERVO_SOLENOID_ENERGIZED", or(pd[1].elevator_2_active_mode, sd[1].elevator_2_active_mode));
        set("LEFT_OUTBOARD_ELEV_EHA_COMMANDED_POSITION", pa[1].elevator_2_pos_order_deg + sa[1].elevator_2_pos_order_deg);
        set("RIGHT_OUTBOARD_ELEV_YELLOW_SERVO_SOLENOID_ENERGIZED", or(pd[1].elevator_1_active_mode, sd[1].elevator_1_active_mode));
        set("RIGHT_OUTBOARD_ELEV_YELLOW_COMMANDED_POSITION", pa[1].elevator_1_pos_order_deg + sa[1].elevator_1_pos_order_deg);
        set("RIGHT_OUTBOARD_ELEV_EHA_SERVO_SOLENOID_ENERGIZED", or(pd[0].elevator_3_active_mode, sd[0].elevator_3_active_mode));
        set("RIGHT_OUTBOARD_ELEV_EHA_COMMANDED_POSITION", pa[0].elevator_3_pos_order_deg + sa[0].elevator_3_pos_order_deg);
        // cpp:2828-2831
        set("THS_GREEN_SERVO_SOLENOID_ENERGIZED", or(pd[2].ths_active_mode, sd[2].ths_active_mode));
        set("THS_GREEN_COMMANDED_POSITION", pa[2].ths_pos_order_deg + sa[2].ths_pos_order_deg);
        set("THS_YELLOW_SERVO_SOLENOID_ENERGIZED", or(pd[0].ths_active_mode, sd[0].ths_active_mode));
        set("THS_YELLOW_COMMANDED_POSITION", pa[0].ths_pos_order_deg + sa[0].ths_pos_order_deg);
        // cpp:2833-2853
        set("UPPER_RUDDER_YELLOW_EBHA_HYDRAULIC_MODE_SOLENOID_ENERGIZED", or(pd[0].rudder_1_hydraulic_active_mode, sd[0].rudder_1_hydraulic_active_mode));
        set("UPPER_RUDDER_YELLOW_EBHA_ELECTRIC_MODE_SOLENOID_ENERGIZED", or(pd[0].rudder_1_electric_active_mode, sd[0].rudder_1_electric_active_mode));
        set("UPPER_RUDDER_YELLOW_EBHA_COMMANDED_POSITION", pa[0].rudder_1_pos_order_deg + sa[0].rudder_1_pos_order_deg);
        set("UPPER_RUDDER_GREEN_EBHA_HYDRAULIC_MODE_SOLENOID_ENERGIZED", or(pd[1].rudder_1_hydraulic_active_mode, sd[1].rudder_1_hydraulic_active_mode));
        set("UPPER_RUDDER_GREEN_EBHA_ELECTRIC_MODE_SOLENOID_ENERGIZED", or(pd[1].rudder_1_electric_active_mode, sd[1].rudder_1_electric_active_mode));
        set("UPPER_RUDDER_GREEN_EBHA_COMMANDED_POSITION", pa[1].rudder_1_pos_order_deg + sa[1].rudder_1_pos_order_deg);
        set("LOWER_RUDDER_GREEN_EBHA_HYDRAULIC_MODE_SOLENOID_ENERGIZED", or(pd[0].rudder_2_hydraulic_active_mode, sd[0].rudder_2_hydraulic_active_mode));
        set("LOWER_RUDDER_GREEN_EBHA_ELECTRIC_MODE_SOLENOID_ENERGIZED", or(pd[0].rudder_2_electric_active_mode, sd[0].rudder_2_electric_active_mode));
        set("LOWER_RUDDER_GREEN_EBHA_COMMANDED_POSITION", pa[0].rudder_2_pos_order_deg + sa[0].rudder_2_pos_order_deg);
        set("LOWER_RUDDER_YELLOW_EBHA_HYDRAULIC_MODE_SOLENOID_ENERGIZED", or(pd[2].rudder_1_hydraulic_active_mode, sd[2].rudder_1_hydraulic_active_mode));
        set("LOWER_RUDDER_YELLOW_EBHA_ELECTRIC_MODE_SOLENOID_ENERGIZED", or(pd[2].rudder_1_electric_active_mode, sd[2].rudder_1_electric_active_mode));
        set("LOWER_RUDDER_YELLOW_EBHA_COMMANDED_POSITION", pa[2].rudder_1_pos_order_deg + sa[2].rudder_1_pos_order_deg);
        // cpp:2855-2863
        set("RUDDER_TRIM_1_ACTIVE_MODE_COMMANDED", sd[0].rudder_trim_active_mode as f64);
        set("RUDDER_TRIM_1_COMMANDED_POSITION", sa[0].rudder_trim_command_deg);
        set("RUDDER_TRIM_2_ACTIVE_MODE_COMMANDED", sd[2].rudder_trim_active_mode as f64);
        set("RUDDER_TRIM_2_COMMANDED_POSITION", sa[2].rudder_trim_command_deg);
        if sd[0].rudder_trim_active_mode != 0 || sd[2].rudder_trim_active_mode != 0 {
            set("RUDDER_TRIM_ACTUAL_POSITION", sa[0].rudder_trim_command_deg + sa[2].rudder_trim_command_deg);
        }
        // cpp:2865-2878 (SimOutputSpoilers) is the MSFS spoiler feedback.
        // cpp:2881
        set(
            "STICK_LOCK_ACTIVE",
            f(pd[0].ap_engaged != 0 || pd[1].ap_engaged != 0 || pd[2].ap_engaged != 0),
        );
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::fbw_controllers::{AthrIn, AthrOut, FadecModel};

    /// The variables, as a map (the plugin's `Vars` needs X-Plane).
    #[derive(Default)]
    pub struct MapVars {
        ids: HashMap<String, VariableIdentifier>,
        values: Vec<f64>,
    }

    impl MapVars {
        fn add(&mut self, name: String) -> VariableIdentifier {
            if let Some(id) = self.ids.get(&name) {
                return *id;
            }
            let mut id = VariableIdentifier::new(0usize);
            for _ in 0..self.values.len() {
                id = id.next();
            }
            self.values.push(0.);
            self.ids.insert(name, id);
            id
        }

        pub fn set(&mut self, name: &str, v: f64) {
            let id = self.add(name.to_owned());
            self.values[id.identifier_index()] = v;
        }

        pub fn value(&mut self, name: &str) -> f64 {
            let id = self.add(name.to_owned());
            self.values[id.identifier_index()]
        }

        pub fn word(&mut self, name: &str, data: f32) {
            self.set(name, to_simvar(BaseArinc429 { SSM: SSM_NO, Data: data }));
        }
    }

    impl VariableRegistry for MapVars {
        fn get(&mut self, name: String) -> VariableIdentifier {
            self.add(format!("A32NX_{name}"))
        }

        fn get_unprefixed(&mut self, name: String) -> VariableIdentifier {
            self.add(name)
        }
    }

    impl SimulatorReaderWriter for MapVars {
        fn read(&mut self, id: &VariableIdentifier) -> f64 {
            self.values.get(id.identifier_index()).copied().unwrap_or(0.)
        }

        fn write(&mut self, id: &VariableIdentifier, v: f64) {
            if let Some(slot) = self.values.get_mut(id.identifier_index()) {
                *slot = v;
            }
        }
    }

    /// A powered aircraft with every ADIRU, RA, LGCIU and SFCC word in normal
    /// operation, level and steady (or standing still on the ground).
    pub fn world(vars: &mut MapVars, on_ground: bool, ias: f32, altitude: f32, radio_height: f32) {
        for bus in ["108PH", "247PP", "DC_1", "DC_2", "AC_2"] {
            vars.set(&format!("A32NX_ELEC_{bus}_BUS_IS_POWERED"), 1.);
        }
        for sys in ["GREEN", "YELLOW"] {
            vars.set(&format!("A32NX_HYD_{sys}_SYSTEM_1_SECTION_PRESSURE_SWITCH"), 1.);
            vars.set(&format!("A32NX_HYD_{sys}_SYSTEM_1_SECTION_PRESSURE"), 5000.);
        }
        let mach = ias / 600.;
        for n in 1..=3 {
            let adr = |s: &str| format!("A32NX_ADIRS_ADR_{n}_{s}");
            vars.word(&adr("ALTITUDE"), altitude);
            vars.word(&adr("BARO_CORRECTED_ALTITUDE_1"), altitude);
            vars.word(&adr("BARO_CORRECTED_ALTITUDE_2"), altitude);
            vars.word(&adr("MACH"), mach);
            vars.word(&adr("COMPUTED_AIRSPEED"), ias);
            vars.word(&adr("TRUE_AIRSPEED"), ias * 1.2);
            vars.word(&adr("BAROMETRIC_VERTICAL_SPEED"), 0.);
            vars.word(&adr("ANGLE_OF_ATTACK"), if on_ground { 0. } else { 3. });
            vars.word(
                &adr("CORRECTED_AVERAGE_STATIC_PRESSURE"),
                1013.25 * (1. - altitude / 145_366.45).powf(5.2559),
            );
            let ir = |s: &str| format!("A32NX_ADIRS_IR_{n}_{s}");
            for s in [
                "MAINT_WORD", "LATITUDE", "LONGITUDE", "TRUE_TRACK", "TRUE_HEADING", "WIND_SPEED",
                "WIND_DIRECTION", "TRACK", "HEADING", "DRIFT_ANGLE", "FLIGHT_PATH_ANGLE", "ROLL",
                "BODY_PITCH_RATE", "BODY_ROLL_RATE", "BODY_YAW_RATE", "BODY_LONGITUDINAL_ACC",
                "BODY_LATERAL_ACC", "HEADING_RATE", "PITCH_ATT_RATE", "ROLL_ATT_RATE", "VERTICAL_SPEED",
            ] {
                vars.word(&ir(s), 0.);
            }
            vars.word(&ir("GROUND_SPEED"), ias);
            vars.word(&ir("PITCH"), if on_ground { 0. } else { 3. });
            vars.word(&ir("BODY_NORMAL_ACC"), 1.);
            vars.word(&format!("A32NX_RA_{n}_RADIO_ALTITUDE"), radio_height);
        }
        for n in 1..=2 {
            // Discrete word 2 bits 11, 13, 14: main gears compressed
            // (systems/src/landing_gear/mod.rs:1195-1198).
            let compressed = if on_ground { (1u32 << 10 | 1 << 12 | 1 << 13) as f32 } else { 0. };
            vars.word(&format!("A32NX_LGCIU_{n}_DISCRETE_WORD_1"), 0.);
            vars.word(&format!("A32NX_LGCIU_{n}_DISCRETE_WORD_2"), compressed);
            vars.word(&format!("A32NX_LGCIU_{n}_DISCRETE_WORD_3"), 0.);
            vars.word(&format!("A32NX_LGCIU_{n}_DISCRETE_WORD_4"), 0.);
            for s in [
                "SLAT_FLAP_COMPONENT_STATUS_WORD", "SLAT_FLAP_SYSTEM_STATUS_WORD", "SLAT_FLAP_ACTUAL_POSITION_WORD",
                "SLAT_ACTUAL_POSITION_WORD", "FLAP_ACTUAL_POSITION_WORD",
            ] {
                vars.word(&format!("A32NX_SFCC_{n}_{s}"), 0.);
            }
        }
        vars.word("A32NX_FQMS_GROSS_WEIGHT", 400_000.);
        vars.word("A32NX_FQMS_CENTER_OF_GRAVITY_MAC", 30.);
        // Hydraulic surfaces at neutral: ratio 0.4 is 0 degrees for ailerons
        // and elevators, 0.5 for the rudders.
        for side in ["LEFT", "RIGHT"] {
            for part in ["OUTWARD", "MIDDLE", "INWARD"] {
                vars.set(&format!("A32NX_HYD_AIL_{side}_{part}_DEFLECTION"), 0.4);
            }
            for part in ["OUTWARD", "INWARD"] {
                vars.set(&format!("A32NX_HYD_ELEV_{side}_{part}_DEFLECTION"), 0.4);
            }
        }
        vars.set("A32NX_HYD_UPPER_RUD_DEFLECTION", 0.5);
        vars.set("A32NX_HYD_LOWER_RUD_DEFLECTION", 0.5);
        for n in 1..=4 {
            vars.set(&format!("A32NX_AUTOTHRUST_TLA:{n}"), 25.);
        }
    }

    /// The four FADECs as engine_commands.rs feeds them, the engines holding
    /// what they are commanded.
    pub fn step_fadecs(
        fadecs: &mut [FadecModel; 4],
        prims: &[BasePrimOutBus; 3],
        on_ground: bool,
        ias: f64,
        dt: f64,
        time: f64,
        n1: &mut [f64; 4],
    ) -> ([BaseEec; 4], [AthrOut; 4]) {
        let mut eec = [BaseEec::default(); 4];
        let mut outs = [AthrOut::default(); 4];
        for i in 0..4 {
            let mut input = AthrIn::default();
            input.time.dt = dt;
            input.time.simulation_time = time;
            input.data.V_ias_kn = ias;
            input.data.on_ground = on_ground as u8;
            input.data.is_engine_operative = 1;
            input.data.engine_N1_percent = n1[i];
            input.data.commanded_engine_N1_percent = n1[i];
            input.data.OAT_degC = 15.;
            input.data.TAT_degC = 15.;
            input.input.TLA_deg = 25.;
            input.input.thrust_limit_IDLE_percent = 19.;
            input.input.thrust_limit_CLB_percent = 82.;
            input.input.thrust_limit_MCT_percent = 88.;
            input.input.thrust_limit_FLEX_percent = 85.;
            input.input.thrust_limit_TOGA_percent = 90.;
            input.input.thrust_limit_REV_percent = 90. * 0.813;
            input.prim_1 = prims[0];
            input.prim_2 = prims[1];
            input.prim_3 = prims[2];
            let out = fadecs[i].step(&input);
            eec[i] = out.fadec_bus_output;
            outs[i] = out;
            n1[i] = out.output.N1_c_percent;
        }
        (eec, outs)
    }

    /// PRIMs, SECs, FCUs and four FADECs, ticked in FlyByWireInterface's order.
    pub struct Rig {
        pub vars: MapVars,
        pub prims: Prims,
        pub fadecs: [FadecModel; 4],
        pub n1: [f64; 4],
        pub time: f64,
        pub on_ground: bool,
        pub ias: f64,
        pub buses: [BasePrimOutBus; 3],
        pub fadec_out: [AthrOut; 4],
        /// What the plugin hands the computers each frame outside the
        /// variable registry: the sidestick, the load factor and the body
        /// rates. Zero by default, as `SimReadings::default()` is -- which
        /// includes a load factor of zero, so a test that wants the pitch
        /// law to do anything has to set at least that.
        pub readings: SimReadings,
    }

    pub const DT: f64 = 0.05;

    impl Rig {
        pub fn new(on_ground: bool, ias: f32, radio_height: f32) -> Self {
            let mut vars = MapVars::default();
            world(&mut vars, on_ground, ias, if on_ground { 0. } else { 10_000. }, radio_height);
            let prims = Prims::new(&mut vars, if on_ground { 2. } else { 6. });
            Self {
                vars,
                prims,
                fadecs: [FadecModel::new(), FadecModel::new(), FadecModel::new(), FadecModel::new()],
                n1: [if on_ground { 20. } else { 70. }; 4],
                time: 10.,
                on_ground,
                ias: ias as f64,
                buses: Default::default(),
                fadec_out: Default::default(),
                readings: SimReadings { nz_g: 1.0, ..Default::default() },
            }
        }

        pub fn tick(&mut self, events: &[Event]) {
            self.tick_with_failures(events, &[]);
        }

        /// [`Self::tick`] with the FailuresConsumer ids given explicitly
        /// active, as `update_with` takes them (cpp:1711, 2200, 2374).
        pub fn tick_with_failures(&mut self, events: &[Event], active_failures: &[u64]) {
            let mut inputs = EventInputs::default();
            for e in events {
                inputs.apply(*e);
            }
            self.buses =
                self.prims.update_with(&mut self.vars, &self.readings, &inputs, DT, self.time, active_failures);
            let (eec, outs) =
                step_fadecs(&mut self.fadecs, &self.buses, self.on_ground, self.ias, DT, self.time, &mut self.n1);
            self.fadec_out = outs;
            self.prims.update_after_fadecs(&mut self.vars, eec);
            self.time += DT;
        }

        pub fn run(&mut self, ticks: usize) {
            for _ in 0..ticks {
                self.tick(&[]);
            }
        }
    }

    fn athr_engaged(bus: &BasePrimOutBus) -> bool {
        bit_or(bus.fg.ats_discrete_word, 11, false)
    }

    #[test]
    fn powered_computers_come_up_healthy_on_the_ground() {
        let mut rig = Rig::new(true, 0., 0.);
        rig.run(40);
        for (i, p) in rig.prims.prim_discrete.iter().enumerate() {
            // Prim.cpp:314 with the self test done (timer zero at start,
            // Prim.cpp:272-279) and the pushbutton in (Prim.cpp:234-241).
            assert_eq!(p.prim_healthy, 1, "PRIM {}", i + 1);
            assert_eq!(rig.vars.value(&format!("A32NX_PRIM_{}_HEALTHY", i + 1)), 1.);
        }
        for (i, s) in rig.prims.sec_discrete.iter().enumerate() {
            assert_eq!(s.sec_healthy, 1, "SEC {}", i + 1);
        }
        let d = rig.prims.prims[0].diagnostics();
        // LGCIU word 2 main gear bits (GeneralLogic.cpp:1011-1031).
        assert!(d.on_ground);
        // The FADECs' ECU status word 4 (GeneralLogic.cpp:1062-1074).
        assert!(d.engine_running);
        assert!(!d.triple_adr_failure && !d.triple_ir_failure && !d.speed_scale_lost);
        // Exactly one master PRIM, PRIM 1 (law status word bit 21, cpp:1776).
        assert!(bit_or(rig.buses[0].fctl.fctl_law_status_word, 21, false));
        // Healthy PRIMs send normal-operation words; nothing engaged.
        assert_eq!(rig.buses[0].fg.ats_discrete_word.SSM, SSM_NO);
        assert!(!athr_engaged(&rig.buses[0]));
        assert_eq!(rig.vars.value("A32NX_AUTOTHRUST_STATUS"), 0.);
        assert_eq!(rig.fadec_out[0].output.athr_control_active, 0);
        // Stick and pedals neutral, surfaces at neutral: the commanded
        // positions stay small, and the powered actuators are engaged.
        for name in [
            "A32NX_LEFT_INBOARD_AIL_GREEN_COMMANDED_POSITION",
            "A32NX_LEFT_OUTBOARD_ELEV_GREEN_COMMANDED_POSITION",
            "A32NX_UPPER_RUDDER_YELLOW_EBHA_COMMANDED_POSITION",
        ] {
            let v = rig.vars.value(name);
            assert!(v.is_finite() && v.abs() < 5., "{name} = {v}");
        }
        let energized = [
            "A32NX_LEFT_INBOARD_AIL_GREEN_SERVO_SOLENOID_ENERGIZED",
            "A32NX_LEFT_OUTBOARD_ELEV_GREEN_SERVO_SOLENOID_ENERGIZED",
            "A32NX_THS_YELLOW_SERVO_SOLENOID_ENERGIZED",
        ]
        .iter()
        .filter(|n| rig.vars.value(n) == 1.)
        .count();
        assert!(energized > 0);
    }

    #[test]
    fn unpowered_prims_are_silent() {
        let mut rig = Rig::new(true, 0., 0.);
        for bus in ["108PH", "247PP", "DC_1"] {
            rig.vars.set(&format!("A32NX_ELEC_{bus}_BUS_IS_POWERED"), 0.);
        }
        rig.run(5);
        // An outage over 0.02 s is a power supply fault (Prim.cpp:257-269);
        // the wrapper then sends an all-zero bus (Prim.cpp:296-301).
        for bus in rig.buses {
            assert_eq!(bus.fg.ats_discrete_word.SSM, 0);
            assert_eq!(bus.fctl.fctl_law_status_word.SSM, 0);
        }
        assert!(rig.prims.prim_discrete.iter().all(|p| p.prim_healthy == 0));
        assert_eq!(rig.fadec_out[0].output.athr_control_active, 0);
    }

    /// FailuresConsumer ids reach the computers exactly where
    /// FlyByWireInterface reads them (cpp:1711 PRIM, cpp:2374 FCU): a failed
    /// PRIM 1 hands mastership to PRIM 2, and a failed FCU 1 alone stays
    /// unhealthy while FCU 2 (not failed) is unaffected.
    #[test]
    fn a_failuresconsumer_id_faults_only_its_own_computer() {
        let mut rig = Rig::new(true, 0., 0.);
        rig.run(40);
        assert!(rig.prims.prim_discrete[0].prim_healthy == 1);
        assert!(bit_or(rig.buses[0].fctl.fctl_law_status_word, 21, false), "PRIM 1 master before the failure");

        for _ in 0..40 {
            rig.tick_with_failures(&[], &[FAILURE_PRIM[0], FAILURE_FCU[0]]);
        }
        assert_eq!(rig.prims.prim_discrete[0].prim_healthy, 0, "PRIM 1 faulted");
        assert_eq!(rig.prims.prim_discrete[1].prim_healthy, 1, "PRIM 2 unaffected");
        assert!(bit_or(rig.buses[1].fctl.fctl_law_status_word, 21, false), "PRIM 2 takes over as master");
        assert_eq!(rig.prims.fcus[0].discrete_outputs().fcu_healthy, 0, "FCU 1 faulted");
        assert_eq!(rig.prims.fcus[1].discrete_outputs().fcu_healthy, 1, "FCU 2 unaffected");

        // Clearing the failures lets PRIM 1 and FCU 1 recover.
        for _ in 0..40 {
            rig.tick_with_failures(&[], &[]);
        }
        assert_eq!(rig.prims.prim_discrete[0].prim_healthy, 1, "PRIM 1 recovered");
        assert_eq!(rig.prims.fcus[0].discrete_outputs().fcu_healthy, 1, "FCU 1 recovered");
    }

    #[test]
    fn athr_pushbutton_in_flight_engages_and_the_fadecs_fly_the_prim_n1() {
        let mut rig = Rig::new(false, 250., 2_500.);
        rig.run(40);
        assert!(!athr_engaged(&rig.buses[0]));
        assert_eq!(rig.fadec_out[0].output.athr_control_active, 0);

        rig.tick(&[Event::FcuAthrPush]);
        // Fg.cpp:1149-1151 (rising edge of athr_pushbutton), 1184-1197 (set
        // with speed control active and RA above 100 ft).
        // The master PRIM (1) engages on the push; PRIMs 2 and 3 are in mode
        // sync (Fg.cpp:839, 1188-1197) and follow from the delayed bus of
        // the master (Fg.cpp:968-988) within a second.
        assert!(athr_engaged(&rig.buses[0]));
        assert!(bit_or(rig.buses[0].fg.ats_discrete_word, 12, false), "active");
        assert!(!athr_engaged(&rig.buses[1]) && !athr_engaged(&rig.buses[2]));
        rig.run(20);
        for bus in &rig.buses {
            assert!(athr_engaged(bus));
        }
        assert_eq!(rig.vars.value("A32NX_AUTOTHRUST_STATUS"), 2.); // cpp:1894-1901
        assert_eq!(rig.vars.value("A32NX_FCU_ATHR_LIGHT_ON"), 1.);
        let first = rig.buses[0].fg.n1_command_percent;
        assert_eq!(first.SSM, SSM_NO);
        for out in &rig.fadec_out {
            // A380FadecComputer.cpp:1219-1234: engaged and active, the PRIM
            // command limited to IDLE..N1(TLA 25) = 19..82.
            assert_eq!(out.output.athr_control_active, 1);
            assert!((out.output.N1_c_percent - (first.Data as f64).clamp(19., 82.)).abs() < 1e-4);
        }
        // Flying 250 kt against the FCU's 100 kt selected speed: the command
        // comes down towards idle.
        rig.run(100);
        let later = rig.buses[0].fg.n1_command_percent.Data;
        assert!(later < first.Data - 5., "{} -> {}", first.Data, later);
        assert!((rig.fadec_out[2].output.N1_c_percent - (later as f64).clamp(19., 82.)).abs() < 1e-4);

        // The instinctive disconnect LVar (cpp:1538, Fg.cpp:1195).
        rig.vars.set("A32NX_AUTOTHRUST_DISCONNECT", 1.);
        rig.tick(&[]);
        rig.vars.set("A32NX_AUTOTHRUST_DISCONNECT", 0.);
        rig.run(2);
        assert!(!athr_engaged(&rig.buses[0]));
        assert_eq!(rig.fadec_out[0].output.athr_control_active, 0);
        assert_eq!(rig.vars.value("A32NX_AUTOTHRUST_STATUS"), 0.);
    }

    #[test]
    fn athr_pushbutton_below_100_ft_does_not_engage() {
        // Fg.cpp:1185-1187: the pushbutton engages only above 100 ft RA.
        let mut rig = Rig::new(true, 0., 0.);
        rig.run(40);
        rig.tick(&[Event::FcuAthrPush]);
        rig.run(2);
        assert!(!athr_engaged(&rig.buses[0]));
        assert_eq!(rig.fadec_out[0].output.athr_control_active, 0);
    }

    /// CTRL-002/CPU-006: the stock Asobo altitude knob's absolute-value
    /// write (converter events.rs `AP_ALT_VAR_SET_ENGLISH`) has no X-Plane
    /// command to carry its argument, so it goes through this one-shot
    /// dataref instead; a fresh aircraft must not read the unset default (0
    /// ft) as a real request, and a value, once applied, must not repeat.
    #[test]
    fn fcu_alt_set_pending_is_a_one_shot_input_not_the_zero_default() {
        let mut rig = Rig::new(true, 0., 0.);
        // Never touched: stays the "no input" sentinel, not 0 ft.
        rig.run(3);
        assert_eq!(rig.vars.value("XP_FCU_ALT_SET_PENDING"), -1.);

        rig.vars.set("XP_FCU_ALT_SET_PENDING", 9000.);
        rig.tick(&[]);
        // Consumed and reset to the sentinel, not left at 9000.
        assert_eq!(rig.vars.value("XP_FCU_ALT_SET_PENDING"), -1.);
        rig.run(2);
        assert_eq!(rig.vars.value("XP_FCU_ALT_SET_PENDING"), -1.);
    }

    /// CTRL-002: the altitude-increment selector's real dataref
    /// (A32NX_FCU_ALT_INCREMENT_1000, toggled by the converter's direct
    /// binding for ASOBO_AUTOPILOT_Switch_Altitude_Increment_Template) is
    /// republished in feet for the stock knob's own INCREMENT read
    /// (XMLVAR_Autopilot_Altitude_Increment, routed there by bind.rs's
    /// home() override) so it is never left at the unwritten-XMLVAR default
    /// of 0, which would divide/modulo by zero.
    #[test]
    fn fcu_alt_increment_republishes_as_a_real_feet_value() {
        let mut rig = Rig::new(true, 0., 0.);
        rig.tick(&[]);
        assert_eq!(rig.vars.value("XP_FCU_ALT_INCREMENT_FT"), 100.);
        rig.vars.set("A32NX_FCU_ALT_INCREMENT_1000", 1.);
        rig.tick(&[]);
        assert_eq!(rig.vars.value("XP_FCU_ALT_INCREMENT_FT"), 1000.);
    }

    /// CPU-003/CPU-004/FCTL-003: the converter's pulse datarefs for the
    /// manual pitch/rudder trim switches (events.rs ELEV_TRIM_UP/DN,
    /// RUDDER_TRIM_LEFT/RIGHT/RESET) are one-frame inputs, cleared the same
    /// way FlyByWireInterface.cpp:985-987 clears SimInputPitchTrim/
    /// SimInputRudderTrim every tick.
    #[test]
    fn manual_trim_switch_pulses_are_one_shot() {
        let mut rig = Rig::new(true, 0., 0.);
        for n in TrimPulses::NAMES {
            rig.vars.set(n, 1.);
        }
        rig.tick(&[]);
        for n in TrimPulses::NAMES {
            assert_eq!(rig.vars.value(n), 0., "{n}");
        }
    }

    #[test]
    fn arinc_words_round_trip_as_the_rust_systems_pack_them() {
        let w = BaseArinc429 { SSM: SSM_NO, Data: 250.5 };
        let v = to_simvar(w);
        assert_eq!((v as u64) >> 32, 3);
        let back = from_simvar(v);
        assert_eq!(back.SSM, 3);
        assert_eq!(back.Data, 250.5);
        let bits = BaseArinc429 { SSM: SSM_NO, Data: (1u32 << 10) as f32 };
        assert!(bit_or(bits, 11, false));
        assert!(!bit_or(bits, 12, false));
        assert!(bit_or(BaseArinc429::default(), 11, true));
    }

    #[test]
    fn surface_deflections_follow_the_msfs_glue() {
        // Neutral hydraulic ratio 0.4 of the 50 degree travel is 0 degrees.
        assert!(hyd_deflection_to_msfs_deflection(0.4, 20., 30.).abs() < 1e-12);
        assert!((hyd_deflection_to_msfs_deflection(1., 20., 30.) - 1.).abs() < 1e-12);
    }

    /// FCTL-002/CPU-002: the PRIMs' own ILS receiver (fms_loc_distance,
    /// cpp:1655-1696) and the diagnostic LVar it shares its source with both
    /// read the localizer-without-DME range from receiver 3's real DME
    /// (NavSimData, sensors.rs), not a hard-coded 0.
    #[test]
    fn localizer_distance_comes_from_the_receiver_3_dme() {
        let mut rig = Rig::new(true, 0., 0.);
        rig.run(2);
        assert_eq!(rig.vars.value("A32NX_RADIO_RECEIVER_LOC_DISTANCE"), 0.);

        rig.vars.set("NAV HAS DME:3", 1.);
        rig.vars.set("NAV DME:3", 12.4);
        rig.tick(&[]);
        assert_eq!(rig.vars.value("A32NX_RADIO_RECEIVER_LOC_DISTANCE"), 12.4);

        // Without a valid DME the range is 0 again, not the stale reading.
        rig.vars.set("NAV HAS DME:3", 0.);
        rig.tick(&[]);
        assert_eq!(rig.vars.value("A32NX_RADIO_RECEIVER_LOC_DISTANCE"), 0.);
    }

    fn prim_1_word(rig: &mut Rig, name: &str) -> f32 {
        from_simvar(rig.vars.value(&format!("A32NX_PRIM_1_{name}"))).Data
    }

    /// A whole-aircraft dynamic pass (debug.md's brief): several minutes of
    /// hands-off cruise, then the AP dialled onto a new heading and a lower
    /// altitude and A/THR onto a slower speed together, checking every value
    /// this port feeds the FCU/PRIM computers and writes back from them.
    /// Heading/altitude/IAS never move in this rig (there is no 6-DOF
    /// aircraft here, only the computers), so once a target is selected the
    /// error the FCU/PRIM see never closes; that is deliberate; it drives
    /// the roll/pitch/thrust commands to their steady commanded limit and
    /// keeps them there, which is exactly what should show a wrong unit
    /// (e.g. a heading fed in radians would demand a much larger, saturated
    /// or NaN roll command instantly) or a wrong sign (thrust moving up
    /// instead of down for a slower target).
    #[test]
    fn a_cruise_flight_engages_ap_hdg_alt_and_athr_together() {
        let mut rig = Rig::new(false, 280., 2_500.);
        rig.run(40); // the computers come up healthy and settle

        let fctl_words = |rig: &Rig| {
            let b = rig.buses[0].fctl;
            [
                b.left_inboard_aileron_command_deg.Data,
                b.right_inboard_aileron_command_deg.Data,
                b.left_outboard_elevator_command_deg.Data,
                b.right_outboard_elevator_command_deg.Data,
                b.ths_command_deg.Data,
                b.upper_rudder_command_deg.Data,
            ]
        };

        // Fly hands-off for three minutes (3600 ticks at 50 ms): nothing
        // should drift into a non-finite or wildly saturated state from
        // per-tick accumulation alone (a per-frame-delta bug in our feed
        // would show up as a slow runaway here even with no pilot input).
        rig.run(3_600);
        for v in fctl_words(&rig) {
            assert!(v.is_finite() && v.abs() < 40., "level cruise command {v} out of range");
        }
        assert_eq!(rig.vars.value("A32NX_AUTOPILOT_ACTIVE"), 0.);
        assert_eq!(rig.vars.value("A32NX_AUTOTHRUST_STATUS"), 0.);

        // Dial in a heading 90 degrees off the current one (world() holds
        // IR HEADING at 0), a lower altitude (world() holds ADR ALTITUDE at
        // 10,000 ft) and a slower speed target, pulling each knob to select
        // it (the same pattern fcu_initialization uses), then engage AP 1
        // and A/THR.
        rig.tick(&[
            Event::FcuHdgSet(90.),
            Event::FcuHdgPull,
            Event::FcuAltSet(5_000.),
            Event::FcuAltPull,
            Event::FcuSpdSet(220.),
            Event::FcuSpdPull,
            Event::FcuAp1Push,
            Event::FcuAthrPush,
        ]);

        // The selected values reach the FCU/PRIM bus already this tick, in
        // their real units (degrees, feet, knots) and un-clipped: this is
        // the direct check that our feed indexes the right var and does not
        // scale it (e.g. by DEG_TO_RAD or FT_TO_M) before handing it to the
        // FCU computer.
        assert!((prim_1_word(&mut rig, "SELECTED_HEADING") - 90.).abs() < 1., "heading not received in degrees");
        assert!((prim_1_word(&mut rig, "SELECTED_ALTITUDE") - 5_000.).abs() < 1., "altitude not received in feet");
        assert!((prim_1_word(&mut rig, "SELECTED_AIRSPEED") - 220.).abs() < 1., "speed target not received in knots");

        // Let the modes engage and the laws settle onto their commanded
        // limit.
        rig.run(200); // 10 s
        assert_eq!(rig.vars.value("A32NX_AUTOPILOT_ACTIVE"), 1., "AP did not engage");
        let lateral_mode = rig.vars.value("A32NX_FMA_LATERAL_MODE");
        let vertical_mode = rig.vars.value("A32NX_FMA_VERTICAL_MODE");
        assert_ne!(lateral_mode, 0., "lateral FMA mode stuck at 0 after a heading was selected");
        assert_ne!(vertical_mode, 0., "vertical FMA mode stuck at 0 after an altitude was selected");
        assert_eq!(rig.vars.value("A32NX_AUTOTHRUST_STATUS"), 2., "A/THR did not reach engaged+active");

        // A heading error that never closes drives a steady, one-sided roll
        // command (not a symmetric, near-zero one): confirms the heading
        // error actually reaches the roll law with the right sign and scale,
        // not zeroed out or overflowing.
        let after_engage = fctl_words(&rig);
        for v in after_engage {
            assert!(v.is_finite() && v.abs() < 40., "commanded surface {v} out of physical range after engage");
        }
        let roll_asymmetry = (after_engage[0] - after_engage[1]).abs();
        assert!(roll_asymmetry > 0.5, "no roll command asymmetry {roll_asymmetry} despite a 90 degree heading error");

        let n1_at_engage = rig.buses[0].fg.n1_command_percent.Data;

        // Run a further two and a half minutes: the still-open heading and
        // altitude errors should hold a steady commanded bank/pitch, not
        // grow without bound (a divergence would mean our per-tick dt feed
        // is wrong) or start oscillating (sampled every second below).
        let mut samples = Vec::new();
        for _ in 0..150 {
            rig.run(20); // 1 s
            samples.push(fctl_words(&rig));
        }
        for s in &samples {
            for v in s {
                assert!(v.is_finite() && v.abs() < 40., "surface command {v} left its physical range over the flight");
            }
        }
        // Oscillation check: the roll command's sign should not flip on
        // every one-second sample once the law has settled onto one side of
        // a heading error that never closes.
        let late = &samples[100..];
        let signs: Vec<bool> = late.iter().map(|s| s[0] - s[1] > 0.).collect();
        let flips = signs.windows(2).filter(|w| w[0] != w[1]).count();
        assert!(flips <= 1, "roll command asymmetry kept flipping sign in steady state: {signs:?}");

        // A/THR: the new target (220 kt) is well under the trimmed cruise
        // speed (280 kt), so the commanded N1 should have moved down, not up
        // or stayed put (the sign/scale of the speed error we feed in).
        let n1_later = rig.buses[0].fg.n1_command_percent.Data;
        assert!(n1_later < n1_at_engage - 1., "N1 command did not come down for a slower speed target: {n1_at_engage} -> {n1_later}");
        assert!(n1_later.is_finite() && (0. ..=105.).contains(&n1_later), "N1 command {n1_later} out of range");

        // The FADECs (fed this bus by the same step_fadecs helper
        // engine_commands.rs uses) still see a sane, engaged, in-range N1
        // command and a finite lever position throughout.
        let (_eec, outs) = step_fadecs(&mut rig.fadecs, &rig.buses, false, rig.ias, DT, rig.time, &mut rig.n1);
        for out in &outs {
            assert_eq!(out.output.athr_control_active, 1);
            assert!(out.output.sim_throttle_lever_pos.is_finite() && out.output.sim_throttle_lever_pos <= 100.);
            assert!(out.output.N1_c_percent.is_finite() && (0. ..=105.).contains(&out.output.N1_c_percent));
        }
    }
}
