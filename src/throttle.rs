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

/// A cockpit dataref another plugin owns, looked for until it exists.
struct Cockpit {
    name: &'static str,
    dataref: Option<DataRef>,
    /// What this plugin last wrote, to tell a pilot's drag from our own echo.
    last_written: Option<f32>,
}

impl Cockpit {
    const fn new(name: &'static str) -> Self {
        Self { name, dataref: None, last_written: None }
    }

    fn find(&mut self, xplm: &Xplm) -> Option<DataRef> {
        if self.dataref.is_none() {
            self.dataref = xplm.find(self.name);
        }
        self.dataref
    }

    /// A value the pilot set, if it differs from what was last written.
    fn moved(&mut self, xplm: &Xplm) -> Option<f32> {
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

    fn write(&mut self, xplm: &Xplm, value: f32) {
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
    levers: [Cockpit; 4],
    reversers: [Cockpit; 2],
    /// Seconds until the cockpit datarefs are looked for again.
    retry: f64,
}

/// Engines 1 and 4 have no reverser.
pub const HAS_REVERSER: [bool; 4] = [false, true, true, false];

impl Throttles {
    pub fn new(xplm: &Xplm) -> Self {
        Self {
            table: thrust_lever_angle_table(),
            lever_table: lever_3d_table(),
            xplane_throttle: xplm.find("sim/cockpit2/engine/actuators/throttle_jet_rev_ratio"),
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
                None => {
                    let r = if HAS_REVERSER[i] { ratio[i] as f64 } else { (ratio[i] as f64).max(0.) };
                    let axis = axis_for_xplane(r);
                    (axis, angle_for_axis(&self.table, axis, HAS_REVERSER[i]))
                }
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
        out
    }
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
}
