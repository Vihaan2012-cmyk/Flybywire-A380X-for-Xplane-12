//! The sidesticks and the speedbrake lever, drawn where FlyByWire's systems
//! put them (A32NX_Interior_Sidestick.xml, A32NX_Interior_Handling.xml
//! FBW_HANDLING_Lever_Spoilers_Template).

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::throttle::Cockpit;
use crate::xp::{DataRef, Xplm};
use crate::Vars;

const STICK_X: &str = "fbw/cockpit/yoke_lever_stick1_l_r";
const STICK_Y: &str = "fbw/cockpit/yoke_lever_stick_fore1_aft";
const SPEEDBRAKE: &str = "fbw/cockpit/lever_speed_brake";

pub fn sidestick_fraction(position: f64) -> f64 {
    ((position + 1.) / 2.).clamp(0., 1.)
}

pub fn speedbrake_fraction(armed: bool, handle: f64) -> f64 {
    if armed {
        0.
    } else {
        ((1. + 2. * handle) / 3.).clamp(0., 1.)
    }
}

pub fn xplane_speedbrake_for_lever(fraction: f64) -> f64 {
    if fraction < 1. / 6. {
        -0.5
    } else {
        ((3. * fraction - 1.) / 2.).clamp(0., 1.)
    }
}

pub struct CockpitLevers {
    stick_x: VariableIdentifier,
    stick_y: VariableIdentifier,
    handle: VariableIdentifier,
    armed: VariableIdentifier,
    sticks: [Cockpit; 2],
    speedbrake: Cockpit,
    xplane_speedbrake: Option<DataRef>,
    ticks: u32,
}

impl CockpitLevers {
    pub fn new(vars: &mut Vars, xplm: &Xplm) -> Self {
        Self {
            stick_x: vars.get("SIDESTICK_POSITION_X".into()),
            stick_y: vars.get("SIDESTICK_POSITION_Y".into()),
            handle: vars.get("SPOILERS_HANDLE_POSITION".into()),
            armed: vars.get("SPOILERS_ARMED".into()),
            sticks: [Cockpit::new(STICK_X), Cockpit::new(STICK_Y)],
            speedbrake: Cockpit::new(SPEEDBRAKE),
            xplane_speedbrake: xplm.find("sim/cockpit2/controls/speedbrake_ratio"),
            ticks: 0,
        }
    }

    pub fn update(&mut self, vars: &mut Vars, xplm: &Xplm) {
        self.ticks = self.ticks.wrapping_add(1);
        let look = self.ticks % 60 == 1;
        let stick = [vars.read(&self.stick_x), vars.read(&self.stick_y)];
        for (lever, position) in self.sticks.iter_mut().zip(stick) {
            if look || lever.dataref.is_some() {
                lever.write(xplm, sidestick_fraction(position) as f32);
            }
        }
        if !(look || self.speedbrake.dataref.is_some()) {
            return;
        }
        if let Some(dragged) = self.speedbrake.moved(xplm) {
            if let Some(d) = self.xplane_speedbrake {
                xplm.set_f(d, xplane_speedbrake_for_lever(dragged as f64) as f32);
            }
            return;
        }
        let armed = vars.read(&self.armed) != 0.;
        self.speedbrake.write(xplm, speedbrake_fraction(armed, vars.read(&self.handle)) as f32);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_sidestick_sits_mid_clip_when_centred() {
        assert_eq!(sidestick_fraction(0.), 0.5);
        assert_eq!(sidestick_fraction(-1.), 0.);
        assert_eq!(sidestick_fraction(1.), 1.);
        assert_eq!(sidestick_fraction(3.), 1.);
    }

    #[test]
    fn the_speedbrake_lever_has_armed_retracted_and_full_positions() {
        assert_eq!(speedbrake_fraction(true, 0.7), 0.);
        assert!((speedbrake_fraction(false, 0.) - 1. / 3.).abs() < 1e-12);
        assert_eq!(speedbrake_fraction(false, 1.), 1.);
    }

    #[test]
    fn dragging_the_speedbrake_lever_reaches_x_planes_handle_and_back() {
        assert_eq!(xplane_speedbrake_for_lever(0.), -0.5);
        for handle in [0., 0.25, 0.5, 1.] {
            let back = xplane_speedbrake_for_lever(speedbrake_fraction(false, handle));
            assert!((back - handle).abs() < 1e-12, "{handle} -> {back}");
        }
    }
}
