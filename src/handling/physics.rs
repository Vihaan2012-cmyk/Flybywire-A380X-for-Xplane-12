//! What X-Plane's flight model is given from FlyByWire's gear, brakes, flaps,
//! slats and steering: the X-Plane side of FlyByWire's `VariablesToObject`
//! writes and `variable_to_event` brake and steering events.

/// Gear deployment for X-Plane's five legs, from FlyByWire's three gear
/// positions (percent), with the fake door drag of `GearPosition::write`
/// (gear.rs:91-111).
///
/// MSFS drives its contact points by retract group, the fifth column of each
/// `point.N` in flight_model.cfg:100-104: point 0 (nose) with GEAR CENTER
/// POSITION, points 1 and 3 with GEAR LEFT POSITION, points 2 and 4 with GEAR
/// RIGHT POSITION. The converted .acf keeps MSFS's order (`_gear/0` nose at
/// x 0; `_gear/1`, `_gear/3` at x -11.5 and -23; `_gear/2`, `_gear/4` at x 11.5
/// and 23).
pub fn gear_deploy(center: f64, left: f64, right: f64, door_center: f64, door_left: f64, door_right: f64) -> ([f64; 5], bool) {
    const GEAR_POSITION_FOR_FAKE_DOOR_DRAG: f64 = 0.10;
    let gear_deployed = center > 5. || left > 5. || right > 5.;
    let door_opened = door_center > 10. || door_left > 10. || door_right > 10.;
    let (nose, l, r) = if door_opened && !gear_deployed {
        (
            (door_center / 100.).min(GEAR_POSITION_FOR_FAKE_DOOR_DRAG),
            (door_left / 100.).min(GEAR_POSITION_FOR_FAKE_DOOR_DRAG),
            (door_right / 100.).min(GEAR_POSITION_FOR_FAKE_DOOR_DRAG),
        )
    } else {
        (center / 100., left / 100., right / 100.)
    };
    ([nose, l, r, l, r], gear_deployed)
}

/// The converted .acf's flap detents, degrees of flap 1 and 2 at each of its
/// `acf/_flap_detents 5` stops (`acf/_flap1_dn/0..5` = `acf/_flap2_dn/0..5`),
/// which the converter took from flight_model.cfg's [FLAPS.0]
/// `flaps-position.0..5` (lines 874-879).
pub const ACF_FLAP_DETENT_DEG: [f64; 6] = [0., 0.01, 8., 17., 26., 32.];
/// `acf/_slat1_dn_max_deg` = `acf/_slat2_dn_max_deg`.
pub const ACF_SLAT_MAX_DEG: f64 = 23.;

/// X-Plane's flap ratio for a flap angle: the inverse of X-Plane's detent
/// table, taking deflection as linear in the ratio between detents. Angles
/// past the last detent (FlyByWire's FULL is 33 degrees, the .acf's 32) give 1.
pub fn flap_ratio_for_angle(deg: f64, detents: &[f64]) -> f64 {
    let steps = (detents.len() - 1) as f64;
    if deg <= detents[0] {
        return 0.;
    }
    for i in 0..detents.len() - 1 {
        let (lo, hi) = (detents[i], detents[i + 1]);
        if deg <= hi && hi > lo {
            return (i as f64 + (deg - lo) / (hi - lo)) / steps;
        }
    }
    1.
}

/// X-Plane's slat ratio for a slat angle.
pub fn slat_ratio_for_angle(deg: f64) -> f64 {
    (deg / ACF_SLAT_MAX_DEG).clamp(0., 1.)
}

/// FlyByWire's steering actuators' travel (a380_systems hydraulic/mod.rs:1792,
/// 1803, 1814): the `_POSITION_RATIO` they write is angle over this
/// (nose_steering.rs:276-280).
pub const NOSE_STEERING_MAX_DEG: f64 = 75.;
pub const BODY_STEERING_MAX_DEG: f64 = 15.;

/// Tyre steering for X-Plane's five legs, degrees positive right: nose from
/// the nose actuator, the body gears (`_gear/1`, `_gear/2`) from theirs, the
/// wing gears fixed. FlyByWire's ratios are positive right (the tiller is
/// "-1 is left", nose_wheel_steering.rs:105; body gear demand opposes the nose,
/// hydraulic/mod.rs:4148-4174, which these already carry).
pub fn tyre_steer_deg(nose_ratio: f64, left_body_ratio: f64, right_body_ratio: f64) -> [f64; 5] {
    [
        nose_ratio * NOSE_STEERING_MAX_DEG,
        left_body_ratio * BODY_STEERING_MAX_DEG,
        right_body_ratio * BODY_STEERING_MAX_DEG,
        0.,
        0.,
    ]
}

/// What a cockpit lever the converter animates (a `fbw/cockpit/...` dataref
/// SASL owns) did, and what it should show.
#[derive(Debug, PartialEq)]
pub enum LeverAction {
    /// The pilot moved it: the value it was moved to.
    Moved(f64),
    /// The systems changed what it stands for: move it to this.
    Show(f64),
    None,
}

/// Follows one cockpit lever both ways without fighting a drag: the pilot's
/// movement is taken when the lever changes, and the lever is only moved to
/// FlyByWire's value when that value changes.
#[derive(Default)]
pub struct LeverBridge {
    lever: Option<f64>,
    fbw: Option<f64>,
}

impl LeverBridge {
    pub fn update(&mut self, lever: f64, fbw_as_lever: f64) -> LeverAction {
        const EPS: f64 = 1e-4;
        let action = match (self.lever, self.fbw) {
            (Some(last), _) if (lever - last).abs() > EPS => LeverAction::Moved(lever),
            (_, Some(last)) if (fbw_as_lever - last).abs() > EPS && (lever - fbw_as_lever).abs() > EPS => {
                LeverAction::Show(fbw_as_lever)
            }
            (None, _) if (lever - fbw_as_lever).abs() > EPS => LeverAction::Show(fbw_as_lever),
            _ => LeverAction::None,
        };
        self.lever = Some(match action {
            LeverAction::Show(v) => v,
            _ => lever,
        });
        self.fbw = Some(fbw_as_lever);
        action
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gear_positions_go_to_the_legs_by_retract_group() {
        let (deploy, handle) = gear_deploy(100., 50., 20., 100., 100., 100.);
        assert_eq!(deploy, [1., 0.5, 0.2, 0.5, 0.2]);
        assert!(handle);
        let (up, handle) = gear_deploy(0., 0., 0., 0., 0., 0.);
        assert_eq!(up, [0.; 5]);
        assert!(!handle);
    }

    #[test]
    fn open_doors_with_gear_up_show_a_little_gear_for_drag() {
        // gear.rs:97-101: doors open, gear still in.
        let (deploy, handle) = gear_deploy(0., 4., 0., 100., 5., 60.);
        assert_eq!(deploy, [0.1, 0.05, 0.1, 0.05, 0.1]);
        assert!(!handle);
    }

    #[test]
    fn flap_angles_map_onto_the_acf_detents() {
        let d = &ACF_FLAP_DETENT_DEG;
        assert_eq!(flap_ratio_for_angle(0., d), 0.);
        assert!((flap_ratio_for_angle(8., d) - 0.4).abs() < 1e-12);
        assert!((flap_ratio_for_angle(17., d) - 0.6).abs() < 1e-12);
        assert!((flap_ratio_for_angle(21.5, d) - 0.7).abs() < 1e-12);
        assert!((flap_ratio_for_angle(26., d) - 0.8).abs() < 1e-12);
        assert_eq!(flap_ratio_for_angle(32., d), 1.);
        // FlyByWire's FULL (hydraulic/mod.rs:1738-1739) is past the .acf's.
        assert_eq!(flap_ratio_for_angle(33., d), 1.);
    }

    #[test]
    fn slat_angles_are_a_share_of_the_acf_maximum() {
        // hydraulic/mod.rs:1744-1745: slats 20 then 23 degrees.
        assert!((slat_ratio_for_angle(20.) - 20. / 23.).abs() < 1e-12);
        assert_eq!(slat_ratio_for_angle(23.), 1.);
        assert_eq!(slat_ratio_for_angle(-1.), 0.);
    }

    #[test]
    fn steering_ratios_become_tyre_angles() {
        assert_eq!(tyre_steer_deg(1., -1., 0.5), [75., -15., 7.5, 0., 0.]);
    }

    #[test]
    fn a_lever_follows_the_pilot_and_the_systems_without_fighting() {
        let mut b = LeverBridge::default();
        // First sight: the lever is put where the systems are.
        assert_eq!(b.update(0., 0.5), LeverAction::Show(0.5));
        // Nothing changed.
        assert_eq!(b.update(0.5, 0.5), LeverAction::None);
        // Pilot drags part way: taken, and not pushed back while the systems
        // have not changed.
        assert_eq!(b.update(0.6, 0.5), LeverAction::Moved(0.6));
        assert_eq!(b.update(0.6, 0.5), LeverAction::None);
        // The systems change (a key press): the lever is moved.
        assert_eq!(b.update(0.6, 1.), LeverAction::Show(1.));
        assert_eq!(b.update(1., 1.), LeverAction::None);
    }
}
