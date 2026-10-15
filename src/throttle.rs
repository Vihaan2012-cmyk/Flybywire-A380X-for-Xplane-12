//! The thrust levers: FlyByWire's throttle axis mapping, ported from their
//! C++ (`fbw-common/src/wasm/fbw_common/src/ThrottleAxisMapping.cpp` and
//! `InterpolatingLookupTable.cpp`), and the A380 set-up of it in
//! `fbw-a380x/src/wasm/fbw_a380/src/FlyByWireInterface.cpp`.
//!
//! In MSFS a throttle axis sends a value from -1 to 1; FlyByWire maps it
//! through detent bands to a thrust lever angle (reverse -20, reverse idle
//! -6, idle 0, CLB 25, FLX/MCT 35, TOGA 45 degrees), writes it to
//! `A32NX_AUTOTHRUST_TLA:n`, and drives the 3D lever from the angle through
//! a second table. Engines 1 and 4 have no reverser.
//!
//! Here the axis comes from X-Plane's throttle, which the pilot's hardware
//! moves: X-Plane's -1..0 reverse range is laid over FlyByWire's reverse
//! band and its 0..1 forward range over the band from idle to TOGA, so every
//! detent sits where FlyByWire's default configuration puts it. A lever
//! dragged in the cockpit sets the angle directly, as FlyByWire's drag code
//! does, and X-Plane's throttle follows it.

use crate::xp::{DataRef, Xplm};

/// FlyByWire's default detent configuration (ThrottleAxisMapping.h).
pub const REV_LO: f64 = -1.00;
pub const REV_HI: f64 = -0.95;
pub const REV_IDLE_LO: f64 = -0.85;
pub const REV_IDLE_HI: f64 = -0.75;
pub const IDLE_LO: f64 = -0.55;
pub const IDLE_HI: f64 = -0.45;
pub const CLB_LO: f64 = -0.05;
pub const CLB_HI: f64 = 0.05;
pub const FLX_LO: f64 = 0.45;
pub const FLX_HI: f64 = 0.55;
pub const TOGA_LO: f64 = 0.95;
pub const TOGA_HI: f64 = 1.00;

pub const TLA_REVERSE: f64 = -20.0;
pub const TLA_REVERSE_IDLE: f64 = -6.0;
pub const TLA_IDLE: f64 = 0.0;
pub const TLA_CLIMB: f64 = 25.0;
pub const TLA_FLEX_MCT: f64 = 35.0;
pub const TLA_TOGA: f64 = 45.0;

/// FlyByWire's interpolating lookup table, quirks included: a value off the
/// table reads 0, and two points sharing an x return that x.
pub struct LookupTable {
    table: Vec<(f64, f64)>,
    minimum: f64,
    maximum: f64,
}

impl LookupTable {
    pub fn new(table: Vec<(f64, f64)>, minimum: f64, maximum: f64) -> Self {
        Self { table, minimum, maximum }
    }

    pub fn get(&self, value: f64) -> f64 {
        if self.table.is_empty() {
            return 0.;
        }
        for pair in self.table.windows(2) {
            let ((x0, y0), (x1, y1)) = (pair[0], pair[1]);
            if x0 <= value && x1 >= value {
                let diff_x = value - x0;
                let diff_n = x1 - x0;
                let mut result = x0;
                if diff_n != 0. {
                    result = y0 + (y1 - y0) * diff_x / diff_n;
                }
                return result.clamp(self.minimum, self.maximum);
            }
        }
        0.
    }
}

/// The axis to thrust lever angle table with reverse and TOGA on the axis,
/// FlyByWire's default.
pub fn thrust_lever_angle_table() -> LookupTable {
    LookupTable::new(
        vec![
            (REV_LO, TLA_REVERSE),
            (REV_HI, TLA_REVERSE),
            (REV_IDLE_LO, TLA_REVERSE_IDLE),
            (REV_IDLE_HI, TLA_REVERSE_IDLE),
            (IDLE_LO, TLA_IDLE),
            (IDLE_HI, TLA_IDLE),
            (CLB_LO, TLA_CLIMB),
            (CLB_HI, TLA_CLIMB),
            (FLX_LO, TLA_FLEX_MCT),
            (FLX_HI, TLA_FLEX_MCT),
            (TOGA_LO, TLA_TOGA),
            (TOGA_HI, TLA_TOGA),
        ],
        TLA_REVERSE,
        TLA_TOGA,
    )
}

/// The thrust lever angle to 3D lever position table (FlyByWireInterface).
pub fn lever_3d_table() -> LookupTable {
    LookupTable::new(vec![(-20.0, 0.0), (0.0, 0.0), (25.0, 55.0), (35.0, 78.0), (45.0, 100.0)], 0., 100.)
}

/// One lever's mapping (ThrottleAxisMapping::setCurrentValue with reverse
/// and TOGA on the axis).
pub fn angle_for_axis(table: &LookupTable, value: f64, has_reverser: bool) -> f64 {
    let tla = table.get(value);
    if has_reverser {
        tla
    } else {
        tla.max(TLA_IDLE)
    }
}

/// X-Plane's throttle, -1 full reverse to 1 full forward, as FlyByWire's axis.
pub fn axis_for_xplane(ratio: f64) -> f64 {
    let r = ratio.clamp(-1., 1.);
    if r >= 0. {
        IDLE_HI + r * (TOGA_HI - IDLE_HI)
    } else {
        IDLE_LO + (-r) * (REV_LO - IDLE_LO)
    }
}

/// The inverse: FlyByWire's axis as X-Plane's throttle.
pub fn xplane_for_axis(axis: f64) -> f64 {
    if axis >= IDLE_HI {
        ((axis - IDLE_HI) / (TOGA_HI - IDLE_HI)).clamp(0., 1.)
    } else if axis <= IDLE_LO {
        -((axis - IDLE_LO) / (REV_LO - IDLE_LO)).clamp(0., 1.)
    } else {
        0.
    }
}

/// The axis value that gives this angle, for a lever set by angle (a cockpit
/// drag). Detents are bands; the middle of the band is used.
pub fn axis_for_angle(angle: f64) -> f64 {
    let bands = [
        (REV_LO, REV_HI, TLA_REVERSE),
        (REV_IDLE_LO, REV_IDLE_HI, TLA_REVERSE_IDLE),
        (IDLE_LO, IDLE_HI, TLA_IDLE),
        (CLB_LO, CLB_HI, TLA_CLIMB),
        (FLX_LO, FLX_HI, TLA_FLEX_MCT),
        (TOGA_LO, TOGA_HI, TLA_TOGA),
    ];
    let angle = angle.clamp(TLA_REVERSE, TLA_TOGA);
    for (lo, hi, at) in bands {
        if (angle - at).abs() < 1e-9 {
            return (lo + hi) / 2.;
        }
    }
    // Between two detents: invert the linear segment joining them.
    for pair in bands.windows(2) {
        let (_, hi_a, tla_a) = pair[0];
        let (lo_b, _, tla_b) = pair[1];
        if angle > tla_a && angle < tla_b {
            return hi_a + (angle - tla_a) / (tla_b - tla_a) * (lo_b - hi_a);
        }
    }
    (IDLE_LO + IDLE_HI) / 2.
}

/// The angle a 3D lever position (0..100) stands for: FlyByWire's lever
/// table inverted. Position 0 is idle; reverse is on the separate lever.
pub fn angle_for_lever_3d(position: f64) -> f64 {
    let points = [(0.0, 0.0), (25.0, 55.0), (35.0, 78.0), (45.0, 100.0)];
    let p = position.clamp(0., 100.);
    for pair in points.windows(2) {
        let ((a0, p0), (a1, p1)) = (pair[0], pair[1]);
        if p >= p0 && p <= p1 {
            return a0 + (p - p0) / (p1 - p0) * (a1 - a0);
        }
    }
    0.
}

pub const DETENT_NAMES: [&str; 6] = ["Reverse", "Reverse idle", "Idle", "CLB", "FLX/MCT", "TOGA"];
const DETENT_TLA: [f64; 6] = [TLA_REVERSE, TLA_REVERSE_IDLE, TLA_IDLE, TLA_CLIMB, TLA_FLEX_MCT, TLA_TOGA];

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Detent {
    pub center: f64,
    pub band: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Calibration {
    pub enabled: bool,
    pub reverse_on_axis: bool,
    pub detents: [Detent; 6],
}

const CALIBRATION_DEFAULT: Calibration = Calibration {
    enabled: false,
    reverse_on_axis: true,
    detents: [
        Detent { center: (REV_LO + REV_HI) / 2., band: (REV_HI - REV_LO) / 2. },
        Detent { center: (REV_IDLE_LO + REV_IDLE_HI) / 2., band: (REV_IDLE_HI - REV_IDLE_LO) / 2. },
        Detent { center: (IDLE_LO + IDLE_HI) / 2., band: (IDLE_HI - IDLE_LO) / 2. },
        Detent { center: (CLB_LO + CLB_HI) / 2., band: (CLB_HI - CLB_LO) / 2. },
        Detent { center: (FLX_LO + FLX_HI) / 2., band: (FLX_HI - FLX_LO) / 2. },
        Detent { center: (TOGA_LO + TOGA_HI) / 2., band: (TOGA_HI - TOGA_LO) / 2. },
    ],
};

impl Default for Calibration {
    fn default() -> Self {
        CALIBRATION_DEFAULT
    }
}

impl Calibration {
    fn inverted(&self) -> bool {
        self.detents[5].center < self.detents[2].center
    }

    pub fn axis(&self, raw: f64) -> f64 {
        let a = raw * 2. - 1.;
        if self.inverted() {
            -a
        } else {
            a
        }
    }

    pub fn table(&self) -> Result<LookupTable, String> {
        let sign = if self.inverted() { -1. } else { 1. };
        let first = if self.reverse_on_axis { 0 } else { 2 };
        let mut points: Vec<(f64, f64)> = Vec::new();
        for i in first..6 {
            let d = self.detents[i];
            if !(d.center.is_finite() && d.band.is_finite()) || d.band < 0. || d.band > 0.5 {
                return Err(format!("{}: band must be between 0 and 0.5", DETENT_NAMES[i]));
            }
            let c = d.center * sign;
            let (mut lo, hi) = (c - d.band, c + d.band);
            if i == first {
                lo = -1.;
            }
            if let Some(&(prev, _)) = points.last() {
                if lo <= prev {
                    return Err(format!("{} overlaps {}: set the detents in lever order", DETENT_NAMES[i], DETENT_NAMES[i - 1]));
                }
            }
            points.push((lo, DETENT_TLA[i]));
            points.push((hi.max(lo), DETENT_TLA[i]));
        }
        if let Some(last) = points.last_mut() {
            last.0 = last.0.max(1.);
        }
        Ok(LookupTable::new(points, DETENT_TLA[first], TLA_TOGA))
    }

    fn to_json(self) -> serde_json::Value {
        serde_json::json!({
            "enabled": self.enabled,
            "reverseOnAxis": self.reverse_on_axis,
            "detents": self.detents.iter().map(|d| serde_json::json!({ "center": d.center, "band": d.band })).collect::<Vec<_>>(),
        })
    }

    pub fn from_json(v: &serde_json::Value) -> Result<Self, String> {
        let mut c = Self::default();
        if let Some(b) = v.get("enabled").and_then(serde_json::Value::as_bool) {
            c.enabled = b;
        }
        if let Some(b) = v.get("reverseOnAxis").and_then(serde_json::Value::as_bool) {
            c.reverse_on_axis = b;
        }
        if let Some(list) = v.get("detents").and_then(serde_json::Value::as_array) {
            if list.len() != 6 {
                return Err("six detents expected".into());
            }
            for (d, j) in c.detents.iter_mut().zip(list) {
                let f = |k: &str| j.get(k).and_then(serde_json::Value::as_f64).ok_or(format!("detent {k} missing"));
                *d = Detent { center: f("center")?, band: f("band")? };
            }
        }
        c.table()?;
        Ok(c)
    }
}

struct Shared {
    calibration: Calibration,
    generation: u64,
    raw: [Option<f64>; 4],
    angle: [f64; 4],
}

static SHARED: std::sync::Mutex<Shared> = std::sync::Mutex::new(Shared { calibration: CALIBRATION_DEFAULT, generation: 0, raw: [None; 4], angle: [0.; 4] });

fn shared<R>(f: impl FnOnce(&mut Shared) -> R) -> R {
    f(&mut SHARED.lock().unwrap_or_else(std::sync::PoisonError::into_inner))
}

fn calibration_path() -> Option<std::path::PathBuf> {
    crate::xp::system_path().map(|root| root.join("Output").join("preferences").join("fbw_a380x_throttle.json"))
}

pub fn load_calibration() {
    let Some(path) = calibration_path() else { return };
    let Ok(text) = std::fs::read_to_string(&path) else { return };
    match serde_json::from_str::<serde_json::Value>(&text).map_err(|e| e.to_string()).and_then(|v| Calibration::from_json(&v)) {
        Ok(c) => shared(|s| {
            s.calibration = c;
            s.generation += 1;
        }),
        Err(e) => crate::log(&format!("throttle: {} ignored: {e}", path.display())),
    }
}

pub fn save_calibration(v: &serde_json::Value) -> Result<(), String> {
    let c = Calibration::from_json(v)?;
    if let Some(path) = calibration_path() {
        let tmp = path.with_extension("json.tmp");
        std::fs::write(&tmp, serde_json::to_string_pretty(&c.to_json()).unwrap_or_default()).map_err(|e| e.to_string())?;
        std::fs::rename(&tmp, &path).map_err(|e| e.to_string())?;
    }
    shared(|s| {
        s.calibration = c;
        s.generation += 1;
    });
    Ok(())
}

pub fn state_json() -> String {
    shared(|s| {
        let c = s.calibration;
        serde_json::json!({
            "calibration": c.to_json(),
            "defaults": Calibration::default().to_json(),
            "names": DETENT_NAMES,
            "raw": s.raw,
            "axis": s.raw.map(|r| r.map(|r| c.axis(r))),
            "angle": s.angle,
        })
        .to_string()
    })
}

/// A cockpit dataref another plugin owns, looked for until it exists.
pub(crate) struct Cockpit {
    name: &'static str,
    pub(crate) dataref: Option<DataRef>,
    /// What this plugin last wrote, to tell a pilot's drag from our own echo.
    last_written: Option<f32>,
}

impl Cockpit {
    pub(crate) const fn new(name: &'static str) -> Self {
        Self { name, dataref: None, last_written: None }
    }

    fn find(&mut self, xplm: &Xplm) -> Option<DataRef> {
        if self.dataref.is_none() {
            self.dataref = xplm.find(self.name);
        }
        self.dataref
    }

    /// A value the pilot set, if it differs from what was last written.
    pub(crate) fn moved(&mut self, xplm: &Xplm) -> Option<f32> {
        let d = self.find(xplm)?;
        let value = xplm.get_f(d);
        match self.last_written {
            Some(w) if (w - value).abs() < 1e-4 => None,
            None => {
                self.last_written = Some(value);
                None
            }
            _ => Some(value),
        }
    }

    pub(crate) fn write(&mut self, xplm: &Xplm, value: f32) {
        if let Some(d) = self.find(xplm) {
            if self.last_written.is_none_or(|w| (w - value).abs() >= 1e-4) {
                xplm.set_f(d, value);
                self.last_written = Some(value);
            }
        }
    }
}

/// What the levers read this tick, per engine.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Levers {
    pub axis: [f64; 4],
    pub angle: [f64; 4],
    pub lever_3d: [f64; 4],
}

/// The four thrust levers.
pub struct Throttles {
    table: LookupTable,
    lever_table: LookupTable,
    xplane_throttle: Option<DataRef>,
    joy_avail: Option<DataRef>,
    joy_value: Option<DataRef>,
    calibrated: Option<LookupTable>,
    calibration: Calibration,
    generation: u64,
    hardware_levers: [bool; 4],
    levers: [Cockpit; 4],
    reversers: [Cockpit; 2],
    /// Seconds until the cockpit datarefs are looked for again.
    retry: f64,
}

/// Engines 1 and 4 have no reverser.
pub const HAS_REVERSER: [bool; 4] = [false, true, true, false];

impl Throttles {
    pub fn new(xplm: &Xplm) -> Self {
        load_calibration();
        Self {
            table: thrust_lever_angle_table(),
            lever_table: lever_3d_table(),
            xplane_throttle: xplm.find("sim/cockpit2/engine/actuators/throttle_jet_rev_ratio"),
            joy_avail: xplm.find("sim/joystick/joy_mapped_axis_avail"),
            joy_value: xplm.find("sim/joystick/joy_mapped_axis_value"),
            calibrated: None,
            calibration: Calibration::default(),
            generation: u64::MAX,
            hardware_levers: [false; 4],
            levers: [
                Cockpit::new("fbw/cockpit/throttle_lever_1"),
                Cockpit::new("fbw/cockpit/throttle_lever_2"),
                Cockpit::new("fbw/cockpit/throttle_lever_3"),
                Cockpit::new("fbw/cockpit/throttle_lever_4"),
            ],
            reversers: [
                Cockpit::new("fbw/cockpit/LEVER_THROTTLE_2_REVERSE"),
                Cockpit::new("fbw/cockpit/LEVER_THROTTLE_3_REVERSE"),
            ],
            retry: 0.,
        }
    }

    /// Read the levers: a cockpit drag wins over the hardware for that tick
    /// and moves X-Plane's throttle to match.
    pub fn update(&mut self, xplm: &Xplm, delta: f64) -> Levers {
        let mut ratio = [0f32; 4];
        if let Some(d) = self.xplane_throttle {
            xplm.get_vf(d, &mut ratio);
        }
        let (generation, calibration) = shared(|s| (s.generation, s.calibration));
        if generation != self.generation {
            self.generation = generation;
            self.calibration = calibration;
            self.calibrated = calibration.table().ok();
        }
        let mut raw_axes = [0f32; AXIS_THROTTLE_1 + 4];
        if let Some(d) = self.joy_value {
            xplm.get_vf(d, &mut raw_axes);
        }
        if let Some(d) = self.joy_avail {
            let mut avail = [0 as std::os::raw::c_int; AXIS_THROTTLE_1 + 4];
            xplm.get_vi(d, &mut avail);
            crate::latch_axes(&mut self.hardware_levers, &avail[AXIS_THROTTLE_1..]);
            ratio = spread_two_levers(ratio, self.hardware_levers);
        }
        let raw = raw_levers(&raw_axes[AXIS_THROTTLE_1..], self.hardware_levers);

        // The cockpit's datarefs belong to the cockpit plugin, which may load
        // after this one; look for missing ones about once a second.
        self.retry -= delta;
        let look = self.retry <= 0.;
        if look {
            self.retry = 1.;
        }

        let mut out = Levers::default();
        for i in 0..4 {
            let mut angle: Option<f64> = None;
            if look || self.levers[i].dataref.is_some() {
                if let Some(v) = self.levers[i].moved(xplm) {
                    angle = Some(angle_for_lever_3d(v as f64 * 100.));
                }
            }
            if let Some(r) = reverser_slot(i) {
                if look || self.reversers[r].dataref.is_some() {
                    if let Some(v) = self.reversers[r].moved(xplm) {
                        if v > 0.001 {
                            angle = Some(TLA_REVERSE * (v as f64).clamp(0., 1.));
                        } else if angle.is_none() {
                            angle = Some(TLA_IDLE);
                        }
                    }
                }
            }

            let (axis, tla) = match angle {
                Some(a) => {
                    let axis = axis_for_angle(a);
                    let tla = angle_for_axis(&self.table, axis, HAS_REVERSER[i]);
                    if let Some(d) = self.xplane_throttle {
                        let mut r = xplane_for_axis(axis) as f32;
                        if !HAS_REVERSER[i] {
                            r = r.max(0.);
                        }
                        xplm.set_vf_at(d, i, r);
                    }
                    (axis, tla)
                }
                None => match (&self.calibrated, raw[i]) {
                    (Some(table), Some(r)) if self.calibration.enabled && (self.calibration.reverse_on_axis || ratio[i] >= -0.01 || !HAS_REVERSER[i]) => {
                        let axis = self.calibration.axis(r);
                        (axis, angle_for_axis(table, axis, HAS_REVERSER[i]))
                    }
                    _ => {
                        let r = if HAS_REVERSER[i] { ratio[i] as f64 } else { (ratio[i] as f64).max(0.) };
                        let axis = axis_for_xplane(r);
                        (axis, angle_for_axis(&self.table, axis, HAS_REVERSER[i]))
                    }
                },
            };
            let position = self.lever_table.get(tla);
            out.axis[i] = axis;
            out.angle[i] = tla;
            out.lever_3d[i] = position;

            // The cockpit levers follow the angle, as FlyByWire's animation does.
            self.levers[i].write(xplm, (position / 100.) as f32);
            if let Some(r) = reverser_slot(i) {
                let reverse = if tla < 0. { (tla / TLA_REVERSE).clamp(0., 1.) } else { 0. };
                self.reversers[r].write(xplm, reverse as f32);
            }
        }
        shared(|s| {
            s.raw = raw;
            s.angle = out.angle;
        });
        out
    }
}

const AXIS_THROTTLE_1: usize = 20;

pub fn spread_two_levers(ratio: [f32; 4], hardware_levers: [bool; 4]) -> [f32; 4] {
    match hardware_levers {
        [true, true, false, false] => [ratio[0], ratio[0], ratio[1], ratio[1]],
        _ => ratio,
    }
}

pub fn raw_levers(values: &[f32], hardware: [bool; 4]) -> [Option<f64>; 4] {
    let source: [Option<usize>; 4] = match hardware {
        [true, true, false, false] => [Some(0), Some(0), Some(1), Some(1)],
        _ => {
            let first = hardware.iter().position(|&h| h);
            std::array::from_fn(|i| if hardware[i] { Some(i) } else { first })
        }
    };
    source.map(|s| s.and_then(|j| values.get(j)).map(|&v| v as f64))
}

fn reverser_slot(engine: usize) -> Option<usize> {
    match engine {
        1 => Some(0),
        2 => Some(1),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    #[test]
    fn detents_read_as_flybywire_sets_them() {
        let t = thrust_lever_angle_table();
        assert!(close(t.get(-1.0), TLA_REVERSE));
        assert!(close(t.get(-0.8), TLA_REVERSE_IDLE));
        assert!(close(t.get(-0.5), TLA_IDLE));
        assert!(close(t.get(0.0), TLA_CLIMB));
        assert!(close(t.get(0.5), TLA_FLEX_MCT));
        assert!(close(t.get(1.0), TLA_TOGA));
        // Between idle and climb it is linear.
        assert!(close(t.get(-0.25), 12.5));
    }

    #[test]
    fn the_lookup_keeps_flybywires_quirks() {
        let t = LookupTable::new(vec![(0., 10.), (1., 20.)], 0., 100.);
        assert_eq!(t.get(2.), 0.);
        let flat = LookupTable::new(vec![(0.5, 10.), (0.5, 20.)], 0., 100.);
        assert_eq!(flat.get(0.5), 0.5);
    }

    #[test]
    fn engines_without_a_reverser_stop_at_idle() {
        let t = thrust_lever_angle_table();
        assert!(close(angle_for_axis(&t, -1.0, false), TLA_IDLE));
        assert!(close(angle_for_axis(&t, -1.0, true), TLA_REVERSE));
    }

    #[test]
    fn two_hardware_levers_fly_all_four_engines_in_pairs() {
        let ratio = [0.3, 0.8, 0.98, 0.98];
        assert_eq!(spread_two_levers(ratio, [true, true, false, false]), [0.3, 0.3, 0.8, 0.8]);
        assert_eq!(spread_two_levers(ratio, [true, true, true, true]), ratio);
        assert_eq!(spread_two_levers(ratio, [false; 4]), ratio);
    }

    #[test]
    fn two_levers_held_at_toga_keep_all_four_engines_at_toga() {
        let mut seen = [false; 4];
        crate::latch_axes(&mut seen, &[1, 1, 0, 0]);
        crate::latch_axes(&mut seen, &[0, 0, 0, 0]);
        let ratio = spread_two_levers([1., 1., 0., 0.], seen);
        let t = thrust_lever_angle_table();
        for (i, r) in ratio.iter().enumerate() {
            assert!(close(angle_for_axis(&t, axis_for_xplane(*r as f64), HAS_REVERSER[i]), TLA_TOGA), "engine {}", i + 1);
        }
    }

    #[test]
    fn xplane_throttle_covers_idle_to_toga_and_reverse() {
        let t = thrust_lever_angle_table();
        assert!(close(angle_for_axis(&t, axis_for_xplane(0.), true), TLA_IDLE));
        assert!(close(angle_for_axis(&t, axis_for_xplane(1.), true), TLA_TOGA));
        assert!(close(angle_for_axis(&t, axis_for_xplane(-1.), true), TLA_REVERSE));
        for r in [-1.0, -0.4, 0.0, 0.3, 0.8, 1.0] {
            assert!(close(xplane_for_axis(axis_for_xplane(r)), r), "{r}");
        }
    }

    #[test]
    fn an_angle_set_in_the_cockpit_reads_back_as_that_angle() {
        let t = thrust_lever_angle_table();
        for angle in [-20.0, -6.0, 0.0, 10.0, 25.0, 30.0, 35.0, 40.0, 45.0] {
            let back = angle_for_axis(&t, axis_for_angle(angle), true);
            assert!((back - angle).abs() < 1e-6, "{angle} -> {back}");
        }
    }

    #[test]
    fn the_3d_lever_follows_flybywires_table_both_ways() {
        let l = lever_3d_table();
        assert_eq!(l.get(25.), 55.);
        assert_eq!(l.get(-20.), 0.);
        for angle in [0.0, 12.0, 25.0, 30.0, 35.0, 45.0] {
            assert!((angle_for_lever_3d(l.get(angle)) - angle).abs() < 1e-9);
        }
    }

    #[test]
    fn the_default_calibration_is_flybywires_table() {
        let c = Calibration { enabled: true, ..Calibration::default() };
        let t = c.table().unwrap();
        for raw in [0.0, 0.1, 0.25, 0.4, 0.5, 0.7, 0.75, 0.9, 1.0] {
            let a = c.axis(raw);
            assert!(close(t.get(a), thrust_lever_angle_table().get(a)), "raw {raw}");
        }
    }

    #[test]
    fn a_calibrated_axis_puts_each_detent_where_the_pilot_set_it() {
        let mut c = Calibration { enabled: true, reverse_on_axis: false, ..Calibration::default() };
        for (i, center) in [(2, 0.1), (3, 0.5), (4, 0.7), (5, 0.95)] {
            c.detents[i] = Detent { center: center * 2. - 1., band: 0.02 };
        }
        let t = c.table().unwrap();
        assert!(close(t.get(c.axis(0.0)), TLA_IDLE), "fully back is idle without reverse on the axis");
        assert!(close(t.get(c.axis(0.5)), TLA_CLIMB));
        assert!(close(t.get(c.axis(0.7)), TLA_FLEX_MCT));
        assert!(close(t.get(c.axis(1.0)), TLA_TOGA));
    }

    #[test]
    fn an_axis_that_runs_backwards_is_calibrated_too() {
        let mut c = Calibration { enabled: true, reverse_on_axis: false, ..Calibration::default() };
        for (i, raw) in [(2, 0.9), (3, 0.5), (4, 0.3), (5, 0.02)] {
            c.detents[i] = Detent { center: raw * 2. - 1., band: 0.02 };
        }
        let t = c.table().unwrap();
        assert!(close(t.get(c.axis(0.5)), TLA_CLIMB));
        assert!(close(t.get(c.axis(0.0)), TLA_TOGA));
    }

    #[test]
    fn overlapping_detents_are_refused() {
        let mut c = Calibration::default();
        c.detents[3] = Detent { center: -0.5, band: 0.05 };
        assert!(c.table().is_err());
        let json = serde_json::json!({ "enabled": true, "reverseOnAxis": true, "detents": c.detents.iter().map(|d| serde_json::json!({"center": d.center, "band": d.band})).collect::<Vec<_>>() });
        assert!(Calibration::from_json(&json).is_err());
    }

    #[test]
    fn levers_without_an_axis_follow_the_first_one() {
        let v = [0.2, 0.8, 0.0, 0.0];
        let (a, b) = (Some(0.2f32 as f64), Some(0.8f32 as f64));
        assert_eq!(raw_levers(&v, [true, false, false, false]), [a; 4]);
        assert_eq!(raw_levers(&v, [true, true, false, false]), [a, a, b, b]);
        assert_eq!(raw_levers(&v, [false; 4]), [None; 4]);
    }
}
