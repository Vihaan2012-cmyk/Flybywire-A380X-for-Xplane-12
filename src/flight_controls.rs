//! X-Plane's control surfaces, moved by FlyByWire's actuators.
//!
//! **Two dataref families, and both are needed.** `sim/flightmodel2/wing/
//! *_deg` are the *drawn* surfaces -- X-Plane's "flightmodel2" namespace is
//! its graphics model -- while `sim/flightmodel/controls/*_def` are the
//! aerodynamic deflections the flight model integrates. X-Plane's own
//! DataRefs.txt says so where it documents the override this module sets:
//! "override_control_surfaces: Overrides individual control surfaces, e.g.
//! sim/flightmodel/controls/lail1def".
//!
//! Writing only the first is a trap worth naming, because it fails in the
//! most convincing way possible: the override switches X-Plane's own
//! joystick-to-surface path off, nothing then writes the aerodynamic
//! deflections, and they sit at zero -- while the cockpit's sidestick moves,
//! the 3D surfaces track it exactly, and the aircraft does not respond at
//! all. Everything looks right except the flying.
//!
//! FlyByWire's PRIMs and SECs (prim.rs) turn the sidestick and pedals into
//! commanded positions; FlyByWire's hydraulic actuators (a380_systems) move
//! each surface and write its position, normalised 0..1. In MSFS their glue
//! (a380_systems_wasm) turns those into MSFS's AILERON/ELEVATOR/RUDDER
//! POSITION and ELEVATOR TRIM POSITION. Here the same outputs become the
//! deflection of each X-Plane wing's surfaces, with X-Plane's own
//! joystick-to-surface path switched off by `override_control_surfaces`. The
//! joystick ratios themselves are not overridden: the PRIMs read them.
//!
//! Unit and sign of each FlyByWire output (a380_systems/src/hydraulic/mod.rs):
//!
//! - Ailerons and elevators: the body travels -20..+30 degrees, positive
//!   trailing edge up (a380_aileron_body, mod.rs:439-481; a380_elevator_body,
//!   mod.rs:773-814), and 0 is down, 1 is up (mod.rs:5067, 5947). The
//!   controllers invert the same line: request = 20/50 - order/50
//!   (mod.rs:5514-5516 with the negation at 5522; 5865-5867). So trailing
//!   edge down is `20 - 50 n`. FlyByWire's glue writes the same angle over
//!   30 to MSFS (ailerons.rs:151-162, elevators.rs:267-278, limits 30 in
//!   flight_model.cfg:613-616), negated on the left wing only because MSFS
//!   takes one roll value; X-Plane takes each wing, so no side is negated.
//! - Rudders: -30..+30 (a380_rudder_body, mod.rs:952-994), request
//!   0.5 - order/60 (mod.rs:6299-6301); FlyByWire's glue gives `2n - 1` of
//!   the 30 degree limit (rudder.rs:296-308, flight_model.cfg:617), which
//!   their F/CTL page draws as positive to the right (SDv2 Rudder.tsx:94,
//!   HorizontalDeflectionIndicator.tsx:28-30; the Airbus-signed rudder trim
//!   is negated to draw it, RudderTrim.tsx:96). Trailing edge right is
//!   `60 n - 30`.
//! - Spoilers: request = surface degrees / 50 (mod.rs:6950-6952), so `50 n`
//!   degrees up.
//! - Stabiliser: HYD_FINAL_THS_DEFLECTION is already degrees, positive nose
//!   up, -2..+10 (trimmable_horizontal_stabilizer.rs:806-809, mod.rs:2111-2114,
//!   flight_model.cfg:619-620); MSFS gets it unchanged as ELEVATOR TRIM
//!   POSITION (a380_systems_wasm trimmable_horizontal_stabilizer.rs:406-421).
//!
//! X-Plane has fewer, differently cut surfaces than the A380 (the converted
//! .acf): two aileron sets, one elevator per side, one rudder, and five
//! spoiler groups per side. Each X-Plane surface takes the span-weighted
//! mean of the FlyByWire panels lying along the same stretch of span, with
//! FlyByWire's panel spans from their bodies and X-Plane's from the .acf.
//! When the panels agree (the normal case) that is exactly their deflection.

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::xp::{DataRef, Xplm};

const LEFT: usize = 0;
const RIGHT: usize = 1;
const SIDES: [&str; 2] = ["LEFT", "RIGHT"];

// ---------------------------------------------------------------------------
// FlyByWire's panels, inboard to outboard, with their spans in metres.
// ---------------------------------------------------------------------------

/// Aileron panels inward, middle, outward: the bodies' x size, the hinge
/// axis (mod.rs:442-446).
const AILERON_SPANS_M: [f64; 3] = [2.26, 2.9, 4.06];
/// Elevator panels inward, outward (mod.rs:777-781).
const ELEVATOR_SPANS_M: [f64; 2] = [5., 9.];
/// Rudder panels upper, lower: the bodies' y size, their hinge axis
/// (mod.rs:956-960).
const RUDDER_SPANS_M: [f64; 2] = [9.63, 4.72];
/// Spoilers 1 to 8, all one body (mod.rs:617-618). Spoiler 1 is the
/// inboard one: FlyByWire's F/CTL page draws 1 next to the fuselage and 8
/// at the wing tip on both sides (SDv2 FctlPage.tsx:31-51).
const SPOILER_SPANS_M: [f64; 8] = [1.785; 8];

// ---------------------------------------------------------------------------
// The converted .acf ("FlyByWire A380X.acf"), read-only reference.
// Wing indices are the .acf's `_wing/N`; even N is the left wing
// (`_is_right_mult -1`). Element widths assume Plane-Maker's equal split of
// `_semilen_SEG` over `_els`.
// ---------------------------------------------------------------------------

const WING1: [usize; 2] = [0, 1]; // _semilen_SEG 46.851937399 ft, 10 elements
const WING2: [usize; 2] = [2, 3]; // 29.877354231 ft, 6 elements
const WING3: [usize; 2] = [4, 5]; // 52.891496399 ft, 10 elements
const WING4: [usize; 2] = [6, 7]; // 27.945241533 ft, 6 elements
const HSTAB: [usize; 2] = [8, 9]; // _elev1 on elements 0-8
const VSTAB: usize = 10; // _rudd1 on elements 0-9

const WING1_ELEMENT_FT: f64 = 46.851937399 / 10.;
const WING2_ELEMENT_FT: f64 = 29.877354231 / 6.;
const WING3_ELEMENT_FT: f64 = 52.891496399 / 10.;
const WING4_ELEMENT_FT: f64 = 27.945241533 / 6.;

/// X-Plane's aileron sets, inboard to outboard: `_ailn1` on wing 3 elements
/// 6-9, `_ailn2` on wing 4 elements 0-4.
const XP_AILERON_SPANS_FT: [f64; 2] = [4. * WING3_ELEMENT_FT, 5. * WING4_ELEMENT_FT];

/// Which X-Plane surface set a spoiler group is.
#[derive(Clone, Copy, Debug, PartialEq)]
enum SpoilerSet {
    Speedbrake1,
    Spoiler1,
    Spoiler2,
}

/// X-Plane's spoiler groups, inboard to outboard: wing 1 `_sbrk1` elements
/// 6-8, wing 2 `_spoi1` 1-3 and `_spoi2` 4-5, wing 3 `_spoi1` 0-3 and
/// `_spoi2` 4-5.
const XP_SPOILER_GROUPS: [(SpoilerSet, [usize; 2]); 5] = [
    (SpoilerSet::Speedbrake1, WING1),
    (SpoilerSet::Spoiler1, WING2),
    (SpoilerSet::Spoiler2, WING2),
    (SpoilerSet::Spoiler1, WING3),
    (SpoilerSet::Spoiler2, WING3),
];
const XP_SPOILER_SPANS_FT: [f64; 5] = [
    3. * WING1_ELEMENT_FT,
    3. * WING2_ELEMENT_FT,
    2. * WING2_ELEMENT_FT,
    4. * WING3_ELEMENT_FT,
    2. * WING3_ELEMENT_FT,
];

/// The .acf's `_stab_trim_up`/`_stab_trim_dn`, used if X-Plane does not
/// give them.
const ACF_STAB_TRIM_DEG: f64 = 8.;

// ---------------------------------------------------------------------------
// Conversions.
// ---------------------------------------------------------------------------

/// Aileron or elevator trailing edge down, in degrees, from the actuator's
/// normalised position.
pub fn aileron_or_elevator_down_deg(n: f64) -> f64 {
    20. - 50. * n
}

/// Rudder trailing edge right, in degrees.
pub fn rudder_right_deg(n: f64) -> f64 {
    60. * n - 30.
}

/// Spoiler up, in degrees.
pub fn spoiler_up_deg(n: f64) -> f64 {
    50. * n
}

/// The vertical load on each leg, as a share of the aeroplane's weight.
///
/// The one measurement that says outright whether the nose gear is carrying
/// what it should. An A380 puts six to eight per cent on the nose; a nose
/// share well past that is an aeroplane riding its nose wheel, which is what
/// a nose-down ground attitude, a hammered nose strut and a refusal to
/// rotate all look like from the seat, and what no amount of reasoning about
/// where the centre of gravity *ought* to be can settle.
///
/// X-Plane's own order: 0 nose, then the mains.
fn gear_load(xplm: &Xplm, tire_force: Option<DataRef>) -> String {
    let Some(d) = tire_force else { return "-".to_owned() };
    let mut n = [0f32; 10];
    xplm.get_vf(d, &mut n);
    let total: f32 = n.iter().sum();
    if total <= 1.0 {
        return "airborne".to_owned();
    }
    let share = |i: usize| 100.0 * f64::from(n[i]) / f64::from(total);
    format!(
        "nose {:.0}% mains {:.0}/{:.0}/{:.0}/{:.0}% of {:.0} t",
        share(0),
        share(1),
        share(2),
        share(3),
        share(4),
        f64::from(total) / 9.81 / 1000.0
    )
}

/// Whether the aeroplane pitched the way the elevator asked it to.
///
/// The one sign this port cannot read off anything: X-Plane documents
/// `elv_trim` as "-1 = max nose down, 1 = max nose up" and
/// `flightmodel2/wing/elevator1_deg` as "positive is trailing-edge down",
/// but `sim/flightmodel/controls/elv1_def` -- the aerodynamic elevator this
/// module writes -- carries no sign in `DataRefs.txt` at all, and the
/// override's own entry names the scalar `lail1def` group rather than the
/// `[WING]` arrays. Guessing it wrong inverts the pitch axis, which looks
/// exactly like what was flown: a rotation that turns into a nose-down
/// divergence, pulling back making it worse rather than better, and the
/// elevators *visually* in the right place the whole time, because the
/// drawn surfaces come off a dataref whose sign X-Plane does document.
///
/// So it is measured instead. A trailing-edge-up elevator (negative here)
/// pitches a conventional aeroplane nose up, which is a positive pitch
/// acceleration in X-Plane's `Q_dot`. Agreement is `ok`; a clear
/// disagreement at a deflection big enough to dominate thrust and trim is
/// `INVERTED?`. Small deflections say nothing and are reported as `-`.
fn pitch_verdict(elevator_te_down_deg: f64, q_dot_rad_s2: f64) -> &'static str {
    // Below these the aeroplane's own trim, thrust line and gravity are as
    // large as the elevator's contribution, so the comparison means nothing.
    const DEFLECTION_DEG: f64 = 5.0;
    const ACCEL_RAD_S2: f64 = 0.01;
    if !q_dot_rad_s2.is_finite() || elevator_te_down_deg.abs() < DEFLECTION_DEG || q_dot_rad_s2.abs() < ACCEL_RAD_S2 {
        return "-";
    }
    // Trailing edge down pitches nose down: the two should have opposite
    // signs.
    if elevator_te_down_deg * q_dot_rad_s2 < 0.0 {
        "ok"
    } else {
        "INVERTED?"
    }
}

/// X-Plane's trim ratio (+1 full nose up) for a stabiliser angle, positive
/// nose up, given the aircraft's trim travel each way.
///
/// **The two travels are crossed on purpose.** `acf_hstb_trim_up` and
/// `acf_hstb_trim_dn` name the *stabiliser's* deflection, not the trim's
/// sense, and they come back the other way round from the aircraft file's
/// own fields: this airframe's `.acf` says `_stab_trim_up 10.0` and
/// `_stab_trim_dn 2.0`, while the datarefs read up 2.000 and dn 10.000.
/// Taking them at face value divided a +5.8 degree nose-up stabiliser by a
/// travel of 2, saturating X-Plane's trim at +1.00 and leaving the aircraft
/// permanently at full nose-up trim whatever FlyByWire commanded. Crossed,
/// the same 5.8 degrees is 0.58 of a 10 degree travel, and the pair line up
/// with FlyByWire's own THS range (-2 to +12 degrees,
/// `a380_systems/hydraulic/mod.rs`) at both ends.
pub fn trim_ratio(ths_deg: f64, travel_up_deg: f64, travel_down_deg: f64) -> f64 {
    let travel = if ths_deg >= 0. { travel_down_deg } else { travel_up_deg };
    if travel <= 0. {
        return 0.;
    }
    (ths_deg / travel).clamp(-1., 1.)
}

/// Each destination segment's span-weighted mean of the source segments over
/// the same fraction of the span. Both lists run the same way (inboard to
/// outboard, or top to bottom) and are scaled to the same total length.
fn spread<const N: usize, const M: usize>(from: &[f64; N], values: &[f64; N], to: &[f64; M]) -> [f64; M] {
    let from_total: f64 = from.iter().sum();
    let to_total: f64 = to.iter().sum();
    let mut out = [0.; M];
    let mut to_start = 0.;
    for (j, width) in to.iter().enumerate() {
        let to_end = to_start + width / to_total;
        let mut from_start = 0.;
        let mut sum = 0.;
        for (i, from_width) in from.iter().enumerate() {
            let from_end = from_start + from_width / from_total;
            let overlap = (to_end.min(from_end) - to_start.max(from_start)).max(0.);
            sum += overlap * values[i];
            from_start = from_end;
        }
        out[j] = sum / (to_end - to_start);
        to_start = to_end;
    }
    out
}

/// FlyByWire's actuator outputs for one tick.
#[derive(Clone, Copy, Debug, Default)]
pub struct Actuators {
    /// [side][inward, middle, outward], normalised.
    pub ailerons: [[f64; 3]; 2],
    /// [side][inward, outward], normalised.
    pub elevators: [[f64; 2]; 2],
    /// [upper, lower], normalised.
    pub rudders: [f64; 2],
    /// [side][spoiler 1..8], normalised.
    pub spoilers: [[f64; 8]; 2],
    /// Degrees, positive nose up.
    pub ths_deg: f64,
}

/// What X-Plane is given, per side where there are sides.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Surfaces {
    /// Trailing edge down, degrees: [side][aileron1, aileron2].
    pub ailerons_deg: [[f64; 2]; 2],
    /// Trailing edge down, degrees: [side].
    pub elevators_deg: [f64; 2],
    /// Trailing edge right, degrees.
    pub rudder_deg: f64,
    /// Up, degrees: [side][group in XP_SPOILER_GROUPS order].
    pub spoilers_deg: [[f64; 5]; 2],
    /// Stabiliser, degrees, positive nose up.
    pub ths_deg: f64,
}

impl Surfaces {
    pub fn from_fbw(a: &Actuators) -> Self {
        let mut s = Surfaces { ths_deg: a.ths_deg, ..Default::default() };
        for side in [LEFT, RIGHT] {
            let ail = a.ailerons[side].map(aileron_or_elevator_down_deg);
            s.ailerons_deg[side] = spread(&AILERON_SPANS_M, &ail, &XP_AILERON_SPANS_FT);
            let elev = a.elevators[side].map(aileron_or_elevator_down_deg);
            s.elevators_deg[side] = spread(&ELEVATOR_SPANS_M, &elev, &[1.])[0];
            let spoilers = a.spoilers[side].map(spoiler_up_deg);
            s.spoilers_deg[side] = spread(&SPOILER_SPANS_M, &spoilers, &XP_SPOILER_SPANS_FT);
        }
        s.rudder_deg = spread(&RUDDER_SPANS_M, &a.rudders.map(rudder_right_deg), &[1.])[0];
        s
    }
}

// ---------------------------------------------------------------------------
// The plugin side.
// ---------------------------------------------------------------------------

struct Ids {
    ailerons: [[VariableIdentifier; 3]; 2],
    elevators: [[VariableIdentifier; 2]; 2],
    rudders: [VariableIdentifier; 2],
    spoilers: [[VariableIdentifier; 8]; 2],
    ths: VariableIdentifier,
    tracking_mode: VariableIdentifier,
}

struct Refs {
    override_surfaces: Option<DataRef>,
    aileron1: Option<DataRef>,
    aileron2: Option<DataRef>,
    elevator1: Option<DataRef>,
    rudder1: Option<DataRef>,
    spoiler1: Option<DataRef>,
    spoiler2: Option<DataRef>,
    speedbrake1: Option<DataRef>,
    aero_aileron1: Option<DataRef>,
    aero_aileron2: Option<DataRef>,
    aero_elevator1: Option<DataRef>,
    aero_rudder1: Option<DataRef>,
    aero_spoiler1: Option<DataRef>,
    aero_spoiler2: Option<DataRef>,
    trim_actual: Option<DataRef>,
    trim_requested: Option<DataRef>,
    trim_travel_up: Option<DataRef>,
    trim_travel_down: Option<DataRef>,
    /// `FBW_FCTL_STATS` only: what the aeroplane did about it.
    pitch_rate: Option<DataRef>,
    pitch_accel: Option<DataRef>,
    theta: Option<DataRef>,
    /// Vertical load on each leg, and the total pitching moment: where the
    /// aeroplane's weight actually sits, and what is moving it.
    tire_force: Option<DataRef>,
    pitch_moment: Option<DataRef>,
}

pub struct FlightControls {
    ids: Ids,
    refs: Refs,
    /// `FBW_FCTL_STATS` only.
    stats_at: Option<std::time::Instant>,
    hyd_green: VariableIdentifier,
    hyd_yellow: VariableIdentifier,
}

impl FlightControls {
    pub fn new<V: VariableRegistry>(vars: &mut V, xplm: &Xplm) -> Self {
        let mut get = |name: String| vars.get(name);
        // Names as a380_systems registers them (mod.rs:6417-6436, 6534-6545,
        // 6645-6647, 6736-6739; trimmable_horizontal_stabilizer.rs:709).
        let ailerons = SIDES.map(|side| {
            ["INWARD", "MIDDLE", "OUTWARD"].map(|part| get(format!("HYD_AIL_{side}_{part}_DEFLECTION")))
        });
        let elevators =
            SIDES.map(|side| ["INWARD", "OUTWARD"].map(|part| get(format!("HYD_ELEV_{side}_{part}_DEFLECTION"))));
        let rudders = ["UPPER", "LOWER"].map(|which| get(format!("HYD_{which}_RUD_DEFLECTION")));
        let spoilers = SIDES.map(|side| {
            [1, 2, 3, 4, 5, 6, 7, 8].map(|k| get(format!("HYD_SPOILER_{k}_{side}_DEFLECTION")))
        });
        let ths = get("HYD_FINAL_THS_DEFLECTION".to_owned());
        // FlyByWire's glue stops writing the surfaces while this is set
        // (ailerons.rs:141-146, elevators.rs:257-262, rudder.rs:340-345,
        // trimmable_horizontal_stabilizer.rs:420-425).
        let tracking_mode = get("FLIGHT_CONTROLS_TRACKING_MODE".to_owned());
        let refs = Refs {
            override_surfaces: xplm.find("sim/operation/override/override_control_surfaces"),
            aileron1: xplm.find("sim/flightmodel2/wing/aileron1_deg"),
            aileron2: xplm.find("sim/flightmodel2/wing/aileron2_deg"),
            elevator1: xplm.find("sim/flightmodel2/wing/elevator1_deg"),
            rudder1: xplm.find("sim/flightmodel2/wing/rudder1_deg"),
            spoiler1: xplm.find("sim/flightmodel2/wing/spoiler1_deg"),
            spoiler2: xplm.find("sim/flightmodel2/wing/spoiler2_deg"),
            speedbrake1: xplm.find("sim/flightmodel2/wing/speedbrake1_deg"),
            // The aerodynamic half. See `Refs`'s own doc comment: the
            // `flightmodel2` names above are the *drawn* surfaces, and on
            // their own they move the model and nothing else.
            aero_aileron1: xplm.find("sim/flightmodel/controls/ail1_def"),
            aero_aileron2: xplm.find("sim/flightmodel/controls/ail2_def"),
            aero_elevator1: xplm.find("sim/flightmodel/controls/elv1_def"),
            aero_rudder1: xplm.find("sim/flightmodel/controls/rudd_def"),
            aero_spoiler1: xplm.find("sim/flightmodel/controls/splr_def"),
            aero_spoiler2: xplm.find("sim/flightmodel/controls/splr2_def"),
            trim_actual: xplm.find("sim/flightmodel/controls/elv_trim"),
            trim_requested: xplm.find("sim/cockpit2/controls/elevator_trim"),
            trim_travel_up: xplm.find("sim/aircraft/controls/acf_hstb_trim_up"),
            trim_travel_down: xplm.find("sim/aircraft/controls/acf_hstb_trim_dn"),
            tire_force: xplm.find("sim/flightmodel2/gear/tire_vertical_force_n_mtr"),
            pitch_moment: xplm.find("sim/flightmodel/forces/M_total"),
            pitch_rate: xplm.find("sim/flightmodel/position/Q"),
            pitch_accel: xplm.find("sim/flightmodel/position/Q_dot"),
            theta: xplm.find("sim/flightmodel/position/theta"),
        };
        if let Some(d) = refs.override_surfaces {
            xplm.set_i(d, 1);
        }
        Self {
            ids: Ids { ailerons, elevators, rudders, spoilers, ths, tracking_mode },
            refs,
            stats_at: None,
            hyd_green: get("HYD_GREEN_SYSTEM_1_SECTION_PRESSURE".to_owned()),
            hyd_yellow: get("HYD_YELLOW_SYSTEM_1_SECTION_PRESSURE".to_owned()),
        }
    }

    fn read<V: SimulatorReaderWriter>(&self, vars: &mut V) -> Actuators {
        let ids = &self.ids;
        let mut a = Actuators { ths_deg: vars.read(&ids.ths), ..Default::default() };
        for side in [LEFT, RIGHT] {
            for (i, id) in ids.ailerons[side].iter().enumerate() {
                a.ailerons[side][i] = vars.read(id);
            }
            for (i, id) in ids.elevators[side].iter().enumerate() {
                a.elevators[side][i] = vars.read(id);
            }
            for (i, id) in ids.spoilers[side].iter().enumerate() {
                a.spoilers[side][i] = vars.read(id);
            }
        }
        for (i, id) in ids.rudders.iter().enumerate() {
            a.rudders[i] = vars.read(id);
        }
        a
    }

    /// After FlyByWire's systems have moved the actuators this tick.
    /// `FBW_FCTL_STATS=1`: every 2 s, the deflections this module is handing
    /// X-Plane, in the sign X-Plane reads them (positive trailing edge
    /// down, so a positive elevator is nose *down*), beside the hydraulic
    /// pressure the actuators are running on.
    ///
    /// It answers the question that is otherwise guesswork from the seat:
    /// when the aeroplane pitches somewhere nobody asked it to, is a
    /// surface actually there, or is something else moving the aircraft?
    /// The sidestick and the 3D model both look right either way.
    fn log_stats<V: SimulatorReaderWriter>(&mut self, vars: &mut V, xplm: &Xplm, s: &Surfaces, trim_ratio: f64) {
        use std::sync::OnceLock;
        static ON: OnceLock<bool> = OnceLock::new();
        if !*ON.get_or_init(|| std::env::var("FBW_FCTL_STATS").is_ok_and(|v| v.trim() != "0" && !v.trim().is_empty())) {
            return;
        }
        let now = std::time::Instant::now();
        if self.stats_at.is_some_and(|t| now - t < std::time::Duration::from_secs(2)) {
            return;
        }
        self.stats_at = Some(now);
        crate::log(&format!(
            "fctl (TE-down +): elev L{:+.1} R{:+.1}, ail L{:+.1} R{:+.1}, rud {:+.1}, THS {:+.1} deg -> trim {:+.2} (X-Plane trim travel up {:.3} dn {:.3}, as read); spoilers L{:?}; hyd {:.0}/{:.0} psi; pitch {:+.1} deg, rate {:+.2}, accel {:+.3} deg/s2 [{}]; gear {}; pitch moment {:+.0} kN.m",
            s.elevators_deg[LEFT],
            s.elevators_deg[RIGHT],
            s.ailerons_deg[LEFT][0],
            s.ailerons_deg[RIGHT][0],
            s.rudder_deg,
            s.ths_deg,
            trim_ratio,
            self.refs.trim_travel_up.map_or(f64::NAN, |d| xplm.get_f(d) as f64),
            self.refs.trim_travel_down.map_or(f64::NAN, |d| xplm.get_f(d) as f64),
            s.spoilers_deg[LEFT].iter().map(|v| v.round() as i32).collect::<Vec<_>>(),
            vars.read(&self.hyd_green),
            vars.read(&self.hyd_yellow),
            self.refs.theta.map_or(f64::NAN, |d| xplm.get_f(d) as f64),
            self.refs.pitch_rate.map_or(f64::NAN, |d| xplm.get_f(d) as f64),
            self.refs.pitch_accel.map_or(f64::NAN, |d| xplm.get_f(d) as f64 * 57.295_78),
            pitch_verdict(s.elevators_deg[LEFT], self.refs.pitch_accel.map_or(f64::NAN, |d| xplm.get_f(d) as f64)),
            gear_load(xplm, self.refs.tire_force),
            self.refs.pitch_moment.map_or(f64::NAN, |d| xplm.get_f(d) as f64 / 1000.),
        ));
    }

    pub fn update<V: SimulatorReaderWriter>(&mut self, vars: &mut V, xplm: &Xplm) {
        if vars.read(&self.ids.tracking_mode) != 0. {
            return;
        }
        let r = &self.refs;
        // Another plugin or a reloaded aircraft can clear the override.
        if let Some(d) = r.override_surfaces {
            if xplm.get_i(d) == 0 {
                xplm.set_i(d, 1);
            }
        }
        let s = Surfaces::from_fbw(&self.read(vars));
        let set = |d: Option<DataRef>, wing: usize, v: f64| {
            if let Some(d) = d {
                xplm.set_vf_at(d, wing, v as f32);
            }
        };
        // Every surface goes to both families: the drawn one so the model
        // moves, and the aerodynamic one so the aircraft does.
        // The drawn surface and the aerodynamic one do *not* take the same
        // sign, and X-Plane says so if you read the two groups together.
        // `sim/flightmodel2/wing/elevator1_deg` is documented "positive is
        // trailing-edge down" -- geometry. Everything documented in
        // `sim/flightmodel/controls/` is the opposite idea: `elv_trim` is
        // "-1 = max nose down, 1 = max nose up", `ail_trim` "max left ..
        // max right", `rud_trim` "max left .. max right" -- the *effect* the
        // control has, not where the metal is. The `[WING]` deflection
        // arrays in that same group carry no sign in `DataRefs.txt` at all,
        // and were being written with the geometric sign the drawn surfaces
        // take.
        //
        // For the elevator those two are exactly opposed: trailing edge up
        // is nose up. So a rotation command reached the aeroplane as a
        // nose-down command, which is what was flown -- the aircraft
        // pitching over onto its nose as the tail came up, pulling back
        // making it worse rather than better, and the elevators looking
        // correct throughout, because the drawn surfaces come off the one
        // dataref whose sign X-Plane does document. It also explains the
        // refusal to rotate at 200 kt earlier: the harder the rotation was
        // commanded, the harder the aeroplane was held down.
        //
        // Only pitch is inverted here. Roll and yaw are left alone: for
        // those the sign depends on which wing an element belongs to, so
        // "positive = roll right" and "positive = trailing edge down" are
        // not simply opposite the way they are on a symmetric tail, and
        // nothing observed says they are wrong.
        let set_both = |drawn: Option<DataRef>, aero: Option<DataRef>, wing: usize, v: f64| {
            set(drawn, wing, v);
            set(aero, wing, v);
        };
        let set_both_pitch = |drawn: Option<DataRef>, aero: Option<DataRef>, wing: usize, v: f64| {
            set(drawn, wing, v);
            set(aero, wing, -v);
        };
        for side in [LEFT, RIGHT] {
            set_both(r.aileron1, r.aero_aileron1, WING3[side], s.ailerons_deg[side][0]);
            set_both(r.aileron2, r.aero_aileron2, WING4[side], s.ailerons_deg[side][1]);
            set_both_pitch(r.elevator1, r.aero_elevator1, HSTAB[side], s.elevators_deg[side]);
            for (k, (set_kind, wings)) in XP_SPOILER_GROUPS.iter().enumerate() {
                let (drawn, aero) = match set_kind {
                    SpoilerSet::Speedbrake1 => (r.speedbrake1, r.aero_spoiler1),
                    SpoilerSet::Spoiler1 => (r.spoiler1, r.aero_spoiler1),
                    SpoilerSet::Spoiler2 => (r.spoiler2, r.aero_spoiler2),
                };
                set_both(drawn, aero, wings[side], s.spoilers_deg[side][k]);
            }
        }
        set_both(r.rudder1, r.aero_rudder1, VSTAB, s.rudder_deg);

        let travel = |d: Option<DataRef>| match d.map(|d| xplm.get_f(d) as f64) {
            Some(t) if t > 0. => t,
            _ => ACF_STAB_TRIM_DEG,
        };
        let ratio = trim_ratio(s.ths_deg, travel(r.trim_travel_up), travel(r.trim_travel_down));
        // Both the requested and the actual trim, so X-Plane's own trim
        // motor has nothing to run towards.
        for d in [r.trim_requested, r.trim_actual].into_iter().flatten() {
            xplm.set_f(d, ratio as f32);
        }
        // After the  borrow above has ended.
        self.log_stats(vars, xplm, &s, ratio);
    }

    /// Hand the surfaces back to X-Plane.
    pub fn release(&self, xplm: &Xplm) {
        if let Some(d) = self.refs.override_surfaces {
            xplm.set_i(d, 0);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    /// Every panel at the same normalised position.
    fn uniform(n_roll: f64, n_elev: f64, n_rud: f64, n_spoiler: f64) -> Actuators {
        Actuators {
            ailerons: [[n_roll; 3]; 2],
            elevators: [[n_elev; 2]; 2],
            rudders: [n_rud; 2],
            spoilers: [[n_spoiler; 8]; 2],
            ths_deg: 0.,
        }
    }

    #[test]
    fn neutral_actuators_give_neutral_surfaces() {
        // 0.4 is 0 degrees for ailerons and elevators (mod.rs:5514-5516),
        // 0.5 for rudders (mod.rs:6299-6301), 0 for spoilers.
        let s = Surfaces::from_fbw(&uniform(0.4, 0.4, 0.5, 0.));
        assert_eq!(s, Surfaces::default());
    }

    #[test]
    fn travel_ends_match_flybywire_bodies() {
        // Ailerons and elevators: 0 is down, 1 is up, -20..+30 body travel.
        let s = Surfaces::from_fbw(&uniform(0., 0., 0., 0.));
        assert!(s.ailerons_deg.iter().flatten().all(|&d| close(d, 20.)));
        assert!(s.elevators_deg.iter().all(|&d| close(d, 20.)));
        assert!(close(s.rudder_deg, -30.));
        let s = Surfaces::from_fbw(&uniform(1., 1., 1., 1.));
        assert!(s.ailerons_deg.iter().flatten().all(|&d| close(d, -30.)));
        assert!(s.elevators_deg.iter().all(|&d| close(d, -30.)));
        assert!(close(s.rudder_deg, 30.));
        assert!(s.spoilers_deg.iter().flatten().all(|&d| close(d, 50.)));
    }

    #[test]
    fn conversions_invert_the_controllers_requests() {
        // FlyByWire's request for an order in degrees, trailing edge down for
        // ailerons/elevators (mod.rs:5514-5516, 5865-5867) and Airbus-signed
        // (positive left) for the rudder (mod.rs:6299-6301).
        for order in [-30., -12.5, 0., 7., 20.] {
            let n = order / -50. + 20. / 50.;
            assert!(close(aileron_or_elevator_down_deg(n), order));
        }
        for order in [-30., -4., 0., 11., 30.] {
            let n = -order / 60. + 0.5;
            assert!(close(rudder_right_deg(n), -order));
        }
        for order in [0., 17., 50.] {
            assert!(close(spoiler_up_deg(order / 50.), order));
        }
    }

    #[test]
    fn right_roll_raises_the_right_ailerons_and_lowers_the_left() {
        let mut a = uniform(0.4, 0.4, 0.5, 0.);
        // Right wing 15 degrees up, left 10 degrees down.
        a.ailerons[RIGHT] = [(15. + 20.) / 50.; 3];
        a.ailerons[LEFT] = [(-10. + 20.) / 50.; 3];
        let s = Surfaces::from_fbw(&a);
        assert!(s.ailerons_deg[RIGHT].iter().all(|&d| close(d, -15.)));
        assert!(s.ailerons_deg[LEFT].iter().all(|&d| close(d, 10.)));
    }

    #[test]
    fn three_ailerons_blend_by_span_into_two() {
        let mut a = uniform(0.4, 0.4, 0.5, 0.);
        // Inward 10 down, middle 0, outward 20 up.
        a.ailerons[LEFT] = [(-10. + 20.) / 50., 0.4, (20. + 20.) / 50.];
        let s = Surfaces::from_fbw(&a);
        let fbw_total = 2.26 + 2.9 + 4.06;
        let xp_in = 4. * 52.891496399 / 10.;
        let xp_out = 5. * 27.945241533 / 6.;
        let split = xp_in / (xp_in + xp_out);
        let (inward_end, middle_end) = (2.26 / fbw_total, (2.26 + 2.9) / fbw_total);
        // Aileron 1 covers the inward panel and part of the middle one;
        // aileron 2 the rest of the middle and the outward panel.
        assert!(split > inward_end && split < middle_end);
        let expected_1 = (inward_end * 10. + (split - inward_end) * 0.) / split;
        let expected_2 = ((middle_end - split) * 0. + (1. - middle_end) * -20.) / (1. - split);
        assert!(close(s.ailerons_deg[LEFT][0], expected_1));
        assert!(close(s.ailerons_deg[LEFT][1], expected_2));
    }

    #[test]
    fn elevators_and_rudders_average_by_span() {
        let mut a = uniform(0.4, 0.4, 0.5, 0.);
        a.elevators[RIGHT] = [(-5. + 20.) / 50., (-19. + 20.) / 50.]; // inward 5 down, outward 19 down
        a.rudders = [(10. + 30.) / 60., (-20. + 30.) / 60.]; // upper 10 right, lower 20 left
        let s = Surfaces::from_fbw(&a);
        assert!(close(s.elevators_deg[RIGHT], (5. * 5. + 9. * 19.) / 14.));
        assert!(close(s.elevators_deg[LEFT], 0.));
        assert!(close(s.rudder_deg, (9.63 * 10. - 4.72 * 20.) / (9.63 + 4.72)));
    }

    #[test]
    fn inboard_spoilers_reach_only_the_inboard_groups() {
        let mut a = uniform(0.4, 0.4, 0.5, 0.);
        // Spoiler 1 (inboard) fully out on the left, spoiler 8 (outboard)
        // half out on the right.
        a.spoilers[LEFT][0] = 1.;
        a.spoilers[RIGHT][7] = 0.5;
        let s = Surfaces::from_fbw(&a);
        let xp_total: f64 = XP_SPOILER_SPANS_FT.iter().sum();
        let first = XP_SPOILER_SPANS_FT[0] / xp_total;
        // Spoiler 1 is the first eighth of the span, inside the wing 1 group.
        assert!(first > 1. / 8.);
        assert!(close(s.spoilers_deg[LEFT][0], 50. * (1. / 8.) / first));
        assert!(s.spoilers_deg[LEFT][1..].iter().all(|&d| close(d, 0.)));
        // Spoiler 8 is the last eighth, inside the outboard wing 3 group.
        let last = XP_SPOILER_SPANS_FT[4] / xp_total;
        assert!(last > 1. / 8.);
        assert!(close(s.spoilers_deg[RIGHT][4], 25. * (1. / 8.) / last));
        assert!(s.spoilers_deg[RIGHT][..4].iter().all(|&d| close(d, 0.)));
    }

    #[test]
    fn spreading_keeps_the_span_weighted_total() {
        let values = [3., -7., 11., 0., 40., 2., 9., 25.];
        let out = spread(&SPOILER_SPANS_M, &values, &XP_SPOILER_SPANS_FT);
        let xp_total: f64 = XP_SPOILER_SPANS_FT.iter().sum();
        let area_in: f64 = values.iter().sum::<f64>() / 8.;
        let area_out: f64 = out.iter().zip(XP_SPOILER_SPANS_FT).map(|(v, w)| v * w / xp_total).sum();
        assert!(close(area_in, area_out));
    }

    #[test]
    fn stabiliser_maps_onto_x_planes_trim_travel() {
        // Symmetric travel: nothing to cross, so either reading agrees.
        assert!(close(trim_ratio(4., 8., 8.), 0.5));
        assert!(close(trim_ratio(-2., 8., 8.), -0.25));
        assert!(close(trim_ratio(10., 8., 8.), 1.));

        // This airframe, as the datarefs actually read: up 2.000, dn 10.000
        // (the aircraft file's own fields are the other way round -- see
        // `trim_ratio`). The stabiliser angle seen in the cockpit, +5.8
        // degrees nose up, used to saturate at +1.00 here; against the 10
        // degree travel it is a little over half.
        assert!(close(trim_ratio(5.8, 2., 10.), 0.58));
        // Both ends of FlyByWire's own THS range (-2 to +12 degrees) land
        // where they should against this pairing, which is the check that
        // the crossing is the right way round rather than merely different:
        // its full nose-down travel is exactly the 2.000 the "up" dataref
        // reports, so it reaches -1.00 and no further.
        assert!(close(trim_ratio(-2., 2., 10.), -1.));
        assert!(close(trim_ratio(12., 2., 10.), 1.));
        // ... and taken uncrossed, that same nose-down limit would read a
        // fifth of its travel, and the nose-up side would saturate at a
        // third of the angle the aircraft actually trims to.
        assert!(close(trim_ratio(4., 2., 10.), 0.4));
    }

    #[test]
    fn sidestick_axes_follow_flybywires_key_events() {
        // X-Plane: +1 is pull, right roll, right pedal (DataRefs.txt:4092-4094).
        // FlyByWire: ELEV_UP, AILERONS_RIGHT and RUDDER_RIGHT all decrease
        // the axis (SimConnectInterface.cpp:2172, 2121, 1972).
        let axes = crate::prim::SimReadings::from_xplane_axes(1., 1., 1.);
        assert!(axes.iter().all(|&v| v < 0.));
    }
}
