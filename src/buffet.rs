//! Stall, Mach, speedbrake and gear buffet: eye shake and a small vertical force; `fbw/anim/buffet` is the intensity.

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::OnceLock;

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::xp::{DataRef, Xplm};
use crate::Vars;

const WING_AREA_M2: f64 = 845.0;
const ALPHA_MAX_DEG: [f64; 6] = [11.0, 15.0, 15.0, 14.0, 13.4, 13.4];
const STALL_ONSET_BELOW_DEG: f64 = 1.5;
const STALL_FULL_ABOVE_DEG: f64 = 3.0;
const MACH_BUFFET_FROM: f64 = 0.70;
const MACH_BUFFET_CL: [(f64, f64); 7] = [(0.70, 0.95), (0.75, 0.88), (0.80, 0.82), (0.85, 0.73), (0.89, 0.60), (0.92, 0.42), (0.95, 0.20)];
const MACH_BUFFET_SPAN_CL: f64 = 0.15;
const Q_REF_PA: f64 = 15_000.0;
const SPEEDBRAKE_SHARE: f64 = 0.35;
const GEAR_SHARE: f64 = 0.3;
const HEAD_PITCH_DEG: f64 = 0.25;
const FORCE_G: f64 = 0.03;
const ATTACK_PER_S: f64 = 6.0;
const DECAY_PER_S: f64 = 3.0;

static INTENSITY: AtomicU32 = AtomicU32::new(0);
static NAME: OnceLock<std::ffi::CString> = OnceLock::new();

unsafe extern "C" fn get(_refcon: *mut std::ffi::c_void) -> f32 {
    f32::from_bits(INTENSITY.load(Ordering::Relaxed))
}

fn ramp(x: f64, from: f64, to: f64) -> f64 {
    ((x - from) / (to - from)).clamp(0.0, 1.0)
}

pub fn stall(alpha_deg: f64, flap_index: f64) -> f64 {
    let i = (flap_index.round().max(0.0) as usize).min(ALPHA_MAX_DEG.len() - 1);
    let max = ALPHA_MAX_DEG[i];
    ramp(alpha_deg, max - STALL_ONSET_BELOW_DEG, max + STALL_FULL_ABOVE_DEG)
}

pub fn buffet_onset_cl(mach: f64) -> f64 {
    let t = &MACH_BUFFET_CL;
    if mach <= t[0].0 {
        return t[0].1;
    }
    for w in t.windows(2) {
        let ((m0, c0), (m1, c1)) = (w[0], w[1]);
        if mach <= m1 {
            return c0 + (c1 - c0) * (mach - m0) / (m1 - m0);
        }
    }
    t[t.len() - 1].1
}

pub fn high_speed(mach: f64, cl: f64) -> f64 {
    if mach < MACH_BUFFET_FROM {
        return 0.0;
    }
    let onset = buffet_onset_cl(mach);
    ramp(cl, onset, onset + MACH_BUFFET_SPAN_CL)
}

pub fn speedbrake(extension: f64, q_pa: f64) -> f64 {
    SPEEDBRAKE_SHARE * extension.clamp(0.0, 1.0) * (q_pa / Q_REF_PA).clamp(0.0, 1.5)
}

pub fn gear(deploy: f64, q_pa: f64) -> f64 {
    let in_transit = if deploy > 0.02 && deploy < 0.98 { 1.0 } else { 0.0 };
    GEAR_SHARE * (0.6 * deploy.clamp(0.0, 1.0) + 0.4 * in_transit) * (q_pa / Q_REF_PA).clamp(0.0, 1.5)
}

pub fn combine(parts: &[f64]) -> f64 {
    parts.iter().map(|p| p * p).sum::<f64>().sqrt().min(1.0)
}

fn noise(t: f64, seed: f64) -> f64 {
    let a = (t * 2.0 * std::f64::consts::PI * 5.3 + seed).sin();
    let b = (t * 2.0 * std::f64::consts::PI * 7.9 + 1.7 * seed).sin();
    let c = (t * 2.0 * std::f64::consts::PI * 11.2 + 2.3 * seed).sin();
    let envelope = 0.75 + 0.25 * (t * 2.0 * std::f64::consts::PI * 1.3 + seed).sin();
    envelope * (0.5 * a + 0.3 * b + 0.2 * c)
}

#[derive(Default)]
struct Refs {
    alpha: Option<DataRef>,
    mach: Option<DataRef>,
    tas: Option<DataRef>,
    rho: Option<DataRef>,
    mass: Option<DataRef>,
    aero_lift: Option<DataRef>,
    gear: Option<DataRef>,
    paused: Option<DataRef>,
    external: Option<DataRef>,
    head_the: Option<DataRef>,
    fnrml: Option<DataRef>,
}

struct Ids {
    flap_index: VariableIdentifier,
    speedbrake: VariableIdentifier,
}

#[derive(Default)]
pub struct Buffet {
    refs: Option<Refs>,
    ids: Option<Ids>,
    t: f64,
    intensity: f64,
    applied: [f64; 1],
}

impl Buffet {
    pub fn update(&mut self, vars: &mut Vars, xplm: &Xplm, delta: f64, enabled: bool) {
        let r = self.refs.get_or_insert_with(|| {
            let name = NAME.get_or_init(|| std::ffi::CString::new("fbw/anim/buffet").unwrap_or_default());
            let accessors = crate::xp::Accessors { get_f: Some(get), ..Default::default() };
            let _ = crate::xp::register_owned_accessor(name, crate::xp::DATA_TYPE_FLOAT, false, &accessors, std::ptr::null_mut());
            Refs {
                alpha: xplm.find("sim/flightmodel2/misc/AoA_angle_degrees"),
                mach: xplm.find("sim/flightmodel/misc/machno"),
                tas: xplm.find("sim/flightmodel/position/true_airspeed"),
                rho: xplm.find("sim/weather/rho"),
                mass: xplm.find("sim/flightmodel/weight/m_total"),
                aero_lift: xplm.find("sim/flightmodel/forces/fnrml_aero"),
                gear: xplm.find("sim/flightmodel2/gear/deploy_ratio"),
                paused: xplm.find("sim/time/paused"),
                external: xplm.find("sim/graphics/view/view_is_external"),
                head_the: xplm.find("sim/graphics/view/pilots_head_the"),
                fnrml: xplm.find("sim/flightmodel/forces/fnrml_plug_acf"),
            }
        });
        let ids = self.ids.get_or_insert_with(|| Ids {
            flap_index: vars.get("FLAPS_HANDLE_INDEX".to_owned()),
            speedbrake: vars.get("A32NX_SPOILERS_HANDLE_POSITION".to_owned()),
        });
        let f = |d: Option<DataRef>| d.map_or(0.0, |d| xplm.get_f(d) as f64);
        if r.paused.is_some_and(|d| xplm.get_i(d) != 0) || delta <= 0.0 {
            return;
        }
        let tas = f(r.tas);
        let q = 0.5 * f(r.rho) * tas * tas;
        let cl = if q > 500.0 { f(r.aero_lift) / (q * WING_AREA_M2) } else { 0.0 };
        let mut gear_deploy = [0f32; 5];
        if let Some(d) = r.gear {
            xplm.get_vf(d, &mut gear_deploy);
        }
        let deploy = gear_deploy.iter().map(|&x| x as f64).sum::<f64>() / gear_deploy.len() as f64;
        let extension = vars.read(&ids.speedbrake);
        let airborne_stall = if tas > 30.0 { stall(f(r.alpha), vars.read(&ids.flap_index)) } else { 0.0 };
        let target = if enabled {
            combine(&[airborne_stall, high_speed(f(r.mach), cl), speedbrake(extension, q), gear(deploy, q)])
        } else {
            0.0
        };
        let rate = if target > self.intensity { ATTACK_PER_S } else { DECAY_PER_S };
        self.intensity += (target - self.intensity) * (rate * delta).min(1.0);
        if self.intensity < 1e-3 {
            self.intensity = 0.0;
        }
        INTENSITY.store((self.intensity as f32).to_bits(), Ordering::Relaxed);
        self.t += delta;

        let inside = r.external.is_some_and(|d| xplm.get_i(d) == 0);
        let heads = [(r.head_the, HEAD_PITCH_DEG, 5.9)];
        for (k, (d, amplitude, seed)) in heads.into_iter().enumerate() {
            let Some(d) = d else { continue };
            let want = if inside { amplitude * self.intensity * noise(self.t, seed) } else { 0.0 };
            let now = xplm.get_f(d) as f64;
            let base = if inside { now - self.applied[k] } else { now };
            if want != 0.0 || self.applied[k] != 0.0 {
                xplm.set_f(d, (base + want) as f32);
            }
            self.applied[k] = want;
        }
        if self.intensity > 0.0 && !crate::xp_writes_skip("forces") {
            if let Some(d) = r.fnrml {
                let force = FORCE_G * self.intensity * f(r.mass) * 9.80665 * noise(self.t, 11.7);
                xplm.set_f(d, (xplm.get_f(d) as f64 + force) as f32);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stall_buffet_starts_below_alpha_max_for_the_flap_setting() {
        assert_eq!(stall(9.0, 0.0), 0.0);
        assert!(stall(10.0, 0.0) > 0.0);
        assert_eq!(stall(14.0, 0.0), 1.0);
        assert_eq!(stall(12.0, 1.0), 0.0, "CONF 1 allows 15 deg");
        assert!(stall(14.0, 5.0) > stall(14.0, 1.0));
    }

    #[test]
    fn a_heavy_cruise_keeps_a_buffet_margin_and_a_hard_pull_or_overspeed_buffets() {
        let q = 0.7 * 20_646.0 * 0.85 * 0.85;
        let cl = 500_000.0 * 9.80665 / (q * WING_AREA_M2);
        assert_eq!(high_speed(0.85, cl), 0.0);
        assert!(high_speed(0.85, cl * 1.3) < 0.3);
        assert!(high_speed(0.85, cl * 1.6) > 0.9);
        assert!(high_speed(0.95, cl) > 0.9);
    }

    #[test]
    fn low_speed_flight_and_the_ground_roll_never_mach_buffet() {
        assert_eq!(high_speed(0.18, 6.0), 0.0, "take-off roll");
        assert_eq!(high_speed(0.21, 1.4), 0.0, "heavy approach, flaps out");
        assert_eq!(high_speed(0.69, 1.2), 0.0);
        assert!(high_speed(0.72, 1.2) > 0.9);
    }

    #[test]
    fn speedbrake_and_gear_buffet_grow_with_dynamic_pressure() {
        assert_eq!(speedbrake(0.0, 15_000.0), 0.0);
        assert!(speedbrake(1.0, 15_000.0) > speedbrake(1.0, 5_000.0));
        assert!(gear(0.5, 8_000.0) > gear(1.0, 8_000.0), "doors open in transit");
        assert_eq!(gear(0.0, 8_000.0), 0.0);
        assert!(combine(&[1.0, 1.0]) <= 1.0);
    }

    #[test]
    fn the_shake_has_no_steady_offset() {
        let n = 20_000;
        let mean: f64 = (0..n).map(|i| noise(i as f64 / 1000.0, 3.1)).sum::<f64>() / n as f64;
        assert!(mean.abs() < 0.02, "{mean}");
    }
}
