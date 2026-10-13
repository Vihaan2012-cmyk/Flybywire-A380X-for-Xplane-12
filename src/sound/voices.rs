use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use super::vorbis::Pcm;
use super::wwise::{rtpc_param, RtpcCurve};
use super::LoopPlan;
use crate::xp;

pub const INSULATION_CLOSED_DB: f32 = -22.;
pub const INSULATION_OPEN_DB: f32 = -12.;
const VIRTUAL_BELOW_DB: f32 = -75.;
const REAL_ABOVE_DB: f32 = -70.;
const VIRTUAL_AFTER: Duration = Duration::from_secs(1);
const RETRY_AFTER: Duration = Duration::from_secs(1);

pub static RESTARTS: AtomicU64 = AtomicU64::new(0);
pub static RESTART_FAILURES: AtomicU64 = AtomicU64::new(0);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum View {
    Both,
    InsideOnly,
    OutsideOnly,
}

pub struct Spec {
    pub trigger: usize,
    pub media_id: u32,
    pub looped: bool,
    pub base_db: f32,
    pub curves: Vec<RtpcCurve>,
    pub exterior: bool,
    pub view: View,
    pub position: Option<[f32; 3]>,
    pub cone_aft: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Pose {
    pub origin: [f64; 3],
    pub velocity: [f32; 3],
    pub psi: f64,
    pub theta: f64,
    pub phi: f64,
}

impl Pose {
    pub fn rotate(&self, b: [f32; 3]) -> [f64; 3] {
        let (x, y, z) = (b[0] as f64, b[1] as f64, b[2] as f64);
        let c = -self.phi.to_radians();
        let (x, y) = (x * c.cos() - y * c.sin(), x * c.sin() + y * c.cos());
        let t = self.theta.to_radians();
        let (y, z) = (y * t.cos() - z * t.sin(), y * t.sin() + z * t.cos());
        let a = -self.psi.to_radians();
        let (x, z) = (x * a.cos() + z * a.sin(), -x * a.sin() + z * a.cos());
        [x, y, z]
    }

    pub fn to_world(&self, b: [f32; 3]) -> [f32; 3] {
        let r = self.rotate(b);
        [(self.origin[0] + r[0]) as f32, (self.origin[1] + r[1]) as f32, (self.origin[2] + r[2]) as f32]
    }
}

#[derive(Clone, Copy)]
struct Sent {
    gain: f32,
    hz: f32,
    at: [f32; 3],
    velocity: [f32; 3],
    aft: [f32; 3],
}

const UNSENT: Sent = Sent { gain: f32::NAN, hz: f32::NAN, at: [f32::NAN; 3], velocity: [f32::NAN; 3], aft: [f32::NAN; 3] };

fn changed(was: f32, now: f32, relative: f32) -> bool {
    was.is_nan() || (was - now).abs() > relative * was.abs().max(now.abs())
}

fn moved(was: [f32; 3], now: [f32; 3], by: f32) -> bool {
    (0..3).any(|k| was[k].is_nan() || (was[k] - now[k]).abs() > by)
}

pub struct Voice {
    pub spec: Spec,
    pcm: Arc<Pcm>,
    plan: Arc<LoopPlan>,
    channels: Vec<xp::FmodChannel>,
    sent: Vec<Sent>,
    retry_at: Option<Instant>,
    live: usize,
    handoff_at: Option<Instant>,
    ends: Option<Instant>,
    fade: Option<(Instant, f64)>,
    quiet_since: Option<Instant>,
    is_virtual: bool,
    pub done: bool,
}

pub struct Mix {
    pub outside: bool,
    pub door_open: bool,
}

pub fn level(curves: &[RtpcCurve], params: &HashMap<u32, f32>) -> (f32, f32) {
    let (mut db, mut cents) = (0f32, 0f32);
    for c in curves {
        let x = params.get(&c.game_param).copied().unwrap_or(0.);
        let y = c.eval(x);
        match c.param {
            rtpc_param::VOLUME => db += if c.scaling == 2 { 20. * (y + 1.).max(1e-5).log10() } else { y },
            rtpc_param::PITCH => cents += y,
            _ => {}
        }
    }
    (db, cents)
}

pub fn placement_db(exterior: bool, view: View, mix: &Mix) -> Option<f32> {
    match (view, mix.outside) {
        (View::InsideOnly, true) | (View::OutsideOnly, false) => return None,
        _ => {}
    }
    match (exterior, mix.outside) {
        (true, false) => Some(if mix.door_open { INSULATION_OPEN_DB } else { INSULATION_CLOSED_DB }),
        (false, true) => None,
        _ => Some(0.),
    }
}

fn bus(exterior: bool) -> std::ffi::c_int {
    if exterior {
        xp::AUDIO_EXTERIOR_AIRCRAFT
    } else {
        xp::AUDIO_INTERIOR
    }
}

fn start(pcm: &Pcm, looped: bool, exterior: bool) -> Option<xp::FmodChannel> {
    let ch = xp::play_pcm16_on_bus(&pcm.samples, pcm.sample_rate, pcm.channels, looped, bus(exterior), None, std::ptr::null_mut())?;
    xp::set_audio_volume(ch, 0.);
    if exterior {
        xp::set_audio_fade_distance(ch, 40., 6000.);
    } else {
        xp::set_audio_fade_distance(ch, 2., 60.);
    }
    Some(ch)
}

impl Voice {
    pub fn start(spec: Spec, pcm: Arc<Pcm>, plan: Arc<LoopPlan>) -> Option<Voice> {
        let mut v = Voice { spec, pcm, plan, channels: Vec::new(), sent: Vec::new(), retry_at: None, live: 0, handoff_at: None, ends: None, fade: None, quiet_since: None, is_virtual: false, done: false };
        v.begin(true)?;
        Some(v)
    }

    fn begin(&mut self, with_intro: bool) -> Option<()> {
        let exterior = self.spec.exterior;
        if !self.spec.looped {
            self.channels = vec![start(&self.pcm, false, exterior)?];
            self.sent = vec![UNSENT];
            let secs = self.pcm.frames() as f64 / f64::from(self.pcm.sample_rate.max(1));
            self.ends = Some(Instant::now() + Duration::from_secs_f64(secs + 0.25));
            return Some(());
        }
        let plan = self.plan.clone();
        match &*plan {
            LoopPlan::Whole => self.channels = vec![start(&self.pcm, true, exterior)?],
            LoopPlan::Body(body) => self.channels = vec![start(body, true, exterior)?],
            LoopPlan::IntroThenBody { intro, body, handoff_s } if with_intro => {
                let i = start(intro, false, exterior)?;
                let Some(b) = start(body, true, exterior) else {
                    xp::stop_audio(i);
                    return None;
                };
                self.channels = vec![i, b];
                self.handoff_at = Some(Instant::now() + Duration::from_secs_f64(handoff_s + 0.05));
            }
            LoopPlan::IntroThenBody { body, .. } => self.channels = vec![start(body, true, exterior)?],
        }
        self.sent = vec![UNSENT; self.channels.len()];
        self.live = 0;
        Some(())
    }

    fn stop_channels(&mut self) {
        for &c in &self.channels {
            xp::stop_audio(c);
        }
        self.channels.clear();
        self.sent.clear();
        self.handoff_at = None;
    }

    pub fn stop(&mut self, fade_s: f64) {
        if fade_s > 0.01 && !self.is_virtual && !self.channels.is_empty() {
            self.fade = Some((Instant::now(), fade_s));
        } else {
            self.stop_channels();
            self.done = true;
        }
    }

    pub fn release(&mut self) {
        self.stop_channels();
        self.done = true;
    }

    pub fn loudness(&self) -> Option<(f32, String)> {
        let sent = self.sent.get(self.live)?;
        if self.done || self.is_virtual || sent.gain.is_nan() || sent.gain <= 0. {
            return None;
        }
        let db = 20. * sent.gain.log10();
        let ratio = sent.hz / self.pcm.sample_rate.max(1) as f32;
        Some((db, format!("media {} trigger {} {db:.0} dB pitch x{ratio:.2}", self.spec.media_id, self.spec.trigger)))
    }

    pub fn audible(&self) -> bool {
        !self.done && !self.is_virtual && !self.channels.is_empty()
    }

    pub fn update(&mut self, params: &HashMap<u32, f32>, mix: &Mix, pose: Option<&Pose>, now: Instant) {
        if self.done {
            return;
        }
        if self.ends.is_some_and(|e| now >= e) {
            self.release();
            return;
        }
        if self.handoff_at.is_some_and(|t| now >= t) && self.channels.len() == 2 {
            xp::stop_audio(self.channels[0]);
            self.channels.remove(0);
            self.sent.remove(0);
            self.handoff_at = None;
            self.live = 0;
        }
        let fade = match self.fade {
            Some((t0, len)) => {
                let k = 1. - (now.duration_since(t0).as_secs_f64() / len);
                if k <= 0. {
                    self.release();
                    return;
                }
                k as f32
            }
            None => 1.,
        };
        let (curve_db, cents) = level(&self.spec.curves, params);
        let place = placement_db(self.spec.exterior, self.spec.view, mix);
        let db = self.spec.base_db + curve_db + place.unwrap_or(0.);
        let silent = place.is_none() || db < VIRTUAL_BELOW_DB;
        if self.spec.looped && self.fade.is_none() {
            if silent {
                let since = *self.quiet_since.get_or_insert(now);
                if !self.is_virtual && now.duration_since(since) >= VIRTUAL_AFTER {
                    self.stop_channels();
                    self.is_virtual = true;
                }
            } else {
                self.quiet_since = None;
                if self.is_virtual && db > REAL_ABOVE_DB && self.retry_at.map_or(true, |t| now >= t) {
                    RESTARTS.fetch_add(1, Ordering::Relaxed);
                    if self.begin(false).is_some() {
                        self.is_virtual = false;
                        self.retry_at = None;
                    } else {
                        RESTART_FAILURES.fetch_add(1, Ordering::Relaxed);
                        self.retry_at = Some(now + RETRY_AFTER);
                    }
                }
            }
        }
        if self.is_virtual || self.channels.is_empty() {
            return;
        }
        let gain = if place.is_none() { 0. } else { 10f32.powf(db / 20.) * fade };
        let hz = self.pcm.sample_rate as f32 * 2f32.powf(cents / 1200.);
        let placed = pose.zip(self.spec.position).map(|(pose, p)| (pose.to_world(p), pose.velocity, pose.rotate([0., 0., 1.])));
        for (i, &c) in self.channels.iter().enumerate() {
            let sent = &mut self.sent[i];
            let g = if i == self.live { gain } else { 0. };
            if changed(sent.gain, g, 0.005) {
                xp::set_audio_volume(c, g);
                sent.gain = g;
            }
            if g <= 0. {
                continue;
            }
            if changed(sent.hz, hz, 0.0005) {
                xp::set_audio_pitch(c, hz);
                sent.hz = hz;
            }
            if let Some((at, vel, aft)) = placed {
                if moved(sent.at, at, 0.02) || moved(sent.velocity, vel, 0.05) {
                    xp::set_audio_position(c, at, vel);
                    sent.at = at;
                    sent.velocity = vel;
                }
                let aft = [aft[0] as f32, aft[1] as f32, aft[2] as f32];
                if self.spec.cone_aft && moved(sent.aft, aft, 0.005) {
                    xp::set_audio_cone(c, 120., 300., 0.45, aft);
                    sent.aft = aft;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn curve(game_param: u32, param: u32, scaling: u8, points: &[(f32, f32)]) -> RtpcCurve {
        RtpcCurve { game_param, param, scaling, points: points.iter().map(|&(x, y)| (x, y, 4)).collect() }
    }

    #[test]
    fn a_db_scaled_volume_curve_and_a_pitch_curve_follow_their_game_parameter() {
        let curves = vec![
            curve(1, rtpc_param::VOLUME, 2, &[(0., -1.), (50., 0.)]),
            curve(1, rtpc_param::PITCH, 0, &[(0., -1200.), (100., 1200.)]),
            curve(2, rtpc_param::VOLUME, 2, &[(0., -0.437_658_67), (10., -0.437_658_67)]),
        ];
        let at = |n1: f32| level(&curves, &[(1u32, n1), (2u32, 0.)].into_iter().collect());
        let (db, cents) = at(50.);
        assert!((db - -5.).abs() < 0.01, "{db}");
        assert!(cents.abs() < 1e-3);
        let (db0, cents0) = at(0.);
        assert!(db0 < -90., "silent at 0 N1: {db0}");
        assert!((cents0 - -1200.).abs() < 1e-3);
    }

    #[test]
    fn a_body_offset_turns_with_heading_pitch_and_roll() {
        let close = |a: [f64; 3], b: [f64; 3]| (0..3).all(|k| (a[k] - b[k]).abs() < 1e-9);
        let east = Pose { psi: 90., ..Default::default() };
        assert!(close(east.rotate([0., 0., -1.]), [1., 0., 0.]), "nose points east: {:?}", east.rotate([0., 0., -1.]));
        let climb = Pose { theta: 30., ..Default::default() };
        assert!(climb.rotate([0., 0., -1.])[1] > 0.4, "nose up");
        let bank = Pose { phi: 30., ..Default::default() };
        assert!(bank.rotate([1., 0., 0.])[1] < -0.4, "right wing down in a right bank");
        let p = Pose { origin: [100., 5., -50.], ..Default::default() };
        assert_eq!(p.to_world([-25.7, -0.6, -4.5]), [74.3, 4.4, -54.5]);
    }

    #[test]
    fn exterior_sounds_are_insulated_inside_and_flight_deck_sounds_stay_in_the_flight_deck() {
        let inside = Mix { outside: false, door_open: false };
        let open = Mix { outside: false, door_open: true };
        let outside = Mix { outside: true, door_open: false };
        assert_eq!(placement_db(true, View::Both, &inside), Some(INSULATION_CLOSED_DB));
        assert_eq!(placement_db(true, View::Both, &open), Some(INSULATION_OPEN_DB));
        assert_eq!(placement_db(true, View::Both, &outside), Some(0.));
        assert_eq!(placement_db(false, View::Both, &inside), Some(0.));
        assert_eq!(placement_db(false, View::Both, &outside), None);
        assert_eq!(placement_db(true, View::InsideOnly, &outside), None);
        assert_eq!(placement_db(true, View::OutsideOnly, &inside), None);
    }
}

#[cfg(test)]
mod package_tests {
    use super::*;
    use crate::sound::wwise::{self, short_id, ActionKind};

    #[test]
    #[ignore]
    fn engine_layers_crossfade_with_n1() {
        let Some(pkg) = wwise::test_package() else { return };
        let pb = pkg.resolve_event(wwise::msfs_event_id("Asobo_A320_NEO", "Engine_1_Front")).unwrap();
        let mut leaves = Vec::new();
        let mut rng = 1u64;
        for a in &pb.actions {
            if let ActionKind::Play { node, parent_volume_db } = &a.kind {
                crate::sound::flatten_with(node, *parent_volume_db, &pkg.ancestor_rtpcs(a.target), &mut rng, None, &mut leaves);
            }
        }
        for n1 in [0f32, 20., 50., 80., 100.] {
            let params: HashMap<u32, f32> = [
                (short_id("SIMVAR_TURB_ENG_N1"), n1),
                (short_id("SIMVAR_TURB_ENG_N2"), if n1 == 0. { 0. } else { 40. + n1 * 0.6 }),
                (short_id("SIMVAR_AIRSPEED_TRUE"), 0.),
                (short_id("SIMVAR_PLANE_ALT_ABOVE_GROUND"), 0.),
                (short_id("LOCALVAR_A32NX_INTERIOR_ENG_VOLUME"), 0.),
            ]
            .into_iter()
            .collect();
            let line: Vec<String> = leaves
                .iter()
                .map(|l| {
                    let (db, cents) = level(&l.curves, &params);
                    format!("{:+.0}dB/{:+.0}c", l.volume_db + db, cents)
                })
                .collect();
            eprintln!("N1 {n1:>3}: {}", line.join(" "));
        }
    }
}
