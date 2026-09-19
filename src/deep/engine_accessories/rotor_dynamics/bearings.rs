//! Per-bearing wear and oil-debris (chip detector) model, wrapping
//! `bearing::signature` with the Trent's real distributed bearing
//! arrangement instead of one representative set standing in for all of
//! it: the same three oil-fed bearing chambers `physics::engine::oil`
//! already models (`oil::Chamber::{Front,HpIp,Tail}` and its own
//! `FLOW_SHARE`/`HEAT_SHARE`) house, per that module's own doc comments,
//! "fan and IP compressor front bearings" (Front, two bearings), "HP and
//! IP turbine bearings ... next to the combustor" (HpIp, two bearings),
//! and a single "LP turbine rear bearing" (Tail, one bearing) -- five main
//! bearings per engine in total, each on its own spool at its own speed,
//! each carrying its own independent defect signature
//! (`bearing::signature`) and its own wear/debris state, rather than one
//! generic bearing for the whole engine.
//!
//! Beyond the vibration signature, a real bearing failure sheds metal: the
//! oil system's magnetic chip detector (in the scavenge line from each
//! chamber) accumulates ferrous debris and, once enough has bridged its
//! gap, gives a discrete "chip" indication -- distinct from, and normally
//! *preceding*, a vibration alert, since a bearing can shed detectable
//! material well before its spall grows large enough to ring strongly at
//! its own defect frequency. Debris generation is modelled as proportional
//! to the worst active defect's severity and to shaft speed (more material
//! removed per revolution at a given severity -- the same physical
//! reasoning `bearing.rs`'s own amplitude-vs-speed scaling uses),
//! integrated over time rather than read instantaneously, and only cleared
//! by an explicit maintenance action (`clear_chip_detector`), matching a
//! real magnetic plug that stays tripped until physically inspected.
//!
//! No Trent-900 chip-detector sensitivity or debris generation rate is
//! public; both are **GENERIC**, sized so a severe, sustained single-defect
//! spall trips the detector within a plausible span of running time while
//! a minor one does not trip it before it would show up on vibration first.

use super::bearing::{signature, BearingFaults, BearingSignature};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Spool {
    Lp,
    Ip,
    Hp,
}

/// Mirrors `physics::engine::oil::Chamber` exactly; restated here since
/// this module cannot import that sibling model (this directory's
/// isolation rule) but must still key off the same three-chamber split.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OilChamber {
    Front,
    HpIp,
    Tail,
}

/// The five main bearings, matching `physics::engine::oil`'s own chamber
/// doc comments exactly (see module docs above for the citation).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BearingId {
    /// Front chamber: fan/LP shaft front bearing.
    FanFront,
    /// Front chamber: IP compressor front bearing.
    IpFront,
    /// HP/IP chamber: HP turbine bearing.
    HpTurbine,
    /// HP/IP chamber: IP turbine bearing.
    IpTurbine,
    /// Tail chamber: LP turbine rear bearing, in the exhaust.
    LpTurbineRear,
}

pub const BEARINGS: [BearingId; 5] = [BearingId::FanFront, BearingId::IpFront, BearingId::HpTurbine, BearingId::IpTurbine, BearingId::LpTurbineRear];

impl BearingId {
    pub fn spool(self) -> Spool {
        match self {
            BearingId::FanFront | BearingId::LpTurbineRear => Spool::Lp,
            BearingId::IpFront | BearingId::IpTurbine => Spool::Ip,
            BearingId::HpTurbine => Spool::Hp,
        }
    }

    pub fn chamber(self) -> OilChamber {
        match self {
            BearingId::FanFront | BearingId::IpFront => OilChamber::Front,
            BearingId::HpTurbine | BearingId::IpTurbine => OilChamber::HpIp,
            BearingId::LpTurbineRear => OilChamber::Tail,
        }
    }
}

/// Debris generation rate at full (1.0) severity and design speed, g/s
/// (**GENERIC**).
const DEBRIS_RATE_G_S: f64 = 6.0e-4;
/// Accumulated debris that bridges the chip detector's gap, g
/// (**GENERIC**).
const CHIP_DETECTOR_THRESHOLD_G: f64 = 0.5;

#[derive(Clone, Copy, Debug, Default)]
pub struct BearingWear {
    debris_g: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BearingState {
    pub signature: BearingSignature,
    /// The single worst active fault fraction: a simple aggregate wear
    /// indicator for a components page, alongside the four individual
    /// fault fractions the underlying `BearingFaults` already carries.
    pub overall_wear: f64,
    pub debris_g: f64,
    pub chip_detected: bool,
}

impl BearingWear {
    pub fn new() -> Self {
        Self { debris_g: 0.0 }
    }

    /// Simulates removing and inspecting the magnetic plug: the only way
    /// debris accumulation ever goes down.
    pub fn clear_chip_detector(&mut self) {
        self.debris_g = 0.0;
    }

    pub fn debris_g(&self) -> f64 {
        self.debris_g
    }

    /// One step. `omega_rad_s`/`design_omega_rad_s` are this bearing's own
    /// spool's actual and design speed.
    pub fn step(&mut self, omega_rad_s: f64, design_omega_rad_s: f64, faults: &BearingFaults, dt_s: f64) -> BearingState {
        let dt = dt_s.max(0.0);
        let sig = signature(omega_rad_s, design_omega_rad_s, faults);
        let overall_wear = faults.outer_race_spall.max(faults.inner_race_spall).max(faults.rolling_element_spall).max(faults.cage_wear).clamp(0.0, 1.0);
        let speed_frac = (omega_rad_s.max(0.0) / design_omega_rad_s.max(1e-6)).min(1.5);
        self.debris_g += DEBRIS_RATE_G_S * overall_wear * speed_frac * dt;
        BearingState { signature: sig, overall_wear, debris_g: self.debris_g, chip_detected: self.debris_g >= CHIP_DETECTOR_THRESHOLD_G }
    }
}

/// One engine's five bearings' faults together, indexed like `BEARINGS`.
#[derive(Clone, Copy, Debug, Default)]
pub struct EngineBearingFaults {
    pub faults: [BearingFaults; 5],
}

/// One engine's five bearings together.
#[derive(Clone, Copy, Debug)]
pub struct EngineBearings {
    wear: [BearingWear; 5],
}

impl EngineBearings {
    pub fn new() -> Self {
        Self { wear: [BearingWear::new(); 5] }
    }

    pub fn bearing(&self, id: BearingId) -> &BearingWear {
        &self.wear[BEARINGS.iter().position(|b| *b == id).unwrap()]
    }

    /// One step. `omega_lp`/`omega_ip`/`omega_hp` are the three spools'
    /// current speeds, rad/s; `design_lp`/`design_ip`/`design_hp` their
    /// design speeds. Returns each bearing's state, indexed like `BEARINGS`.
    pub fn step(&mut self, omega_lp: f64, omega_ip: f64, omega_hp: f64, design_lp: f64, design_ip: f64, design_hp: f64, faults: &EngineBearingFaults, dt_s: f64) -> [BearingState; 5] {
        let mut out = [BearingState::default(); 5];
        for i in 0..5 {
            let (omega, design) = match BEARINGS[i].spool() {
                Spool::Lp => (omega_lp, design_lp),
                Spool::Ip => (omega_ip, design_ip),
                Spool::Hp => (omega_hp, design_hp),
            };
            out[i] = self.wear[i].step(omega, design, &faults.faults[i], dt_s);
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_five_bearings_map_to_the_oil_systems_three_chambers_as_documented() {
        assert_eq!(BearingId::FanFront.chamber(), OilChamber::Front);
        assert_eq!(BearingId::IpFront.chamber(), OilChamber::Front);
        assert_eq!(BearingId::HpTurbine.chamber(), OilChamber::HpIp);
        assert_eq!(BearingId::IpTurbine.chamber(), OilChamber::HpIp);
        assert_eq!(BearingId::LpTurbineRear.chamber(), OilChamber::Tail);
        // Front = 2 bearings, HpIp = 2, Tail = 1, matching oil.rs's own
        // per-chamber bearing description exactly.
        let front = BEARINGS.iter().filter(|b| b.chamber() == OilChamber::Front).count();
        let hpip = BEARINGS.iter().filter(|b| b.chamber() == OilChamber::HpIp).count();
        let tail = BEARINGS.iter().filter(|b| b.chamber() == OilChamber::Tail).count();
        assert_eq!((front, hpip, tail), (2, 2, 1));
    }

    #[test]
    fn each_bearing_is_driven_by_its_own_spools_speed() {
        assert_eq!(BearingId::FanFront.spool(), Spool::Lp);
        assert_eq!(BearingId::LpTurbineRear.spool(), Spool::Lp);
        assert_eq!(BearingId::IpFront.spool(), Spool::Ip);
        assert_eq!(BearingId::IpTurbine.spool(), Spool::Ip);
        assert_eq!(BearingId::HpTurbine.spool(), Spool::Hp);
    }

    #[test]
    fn a_healthy_bearing_never_accumulates_debris_no_nan() {
        let mut w = BearingWear::new();
        let mut s = BearingState::default();
        for _ in 0..1000 {
            s = w.step(300.0, 300.0, &BearingFaults::default(), 1.0);
        }
        assert_eq!(s.debris_g, 0.0);
        assert!(!s.chip_detected);
        assert!(!s.debris_g.is_nan());
    }

    #[test]
    fn a_severe_sustained_spall_eventually_trips_the_chip_detector() {
        let mut w = BearingWear::new();
        let faults = BearingFaults { inner_race_spall: 1.0, ..Default::default() };
        let mut s = BearingState::default();
        for _ in 0..2000 {
            s = w.step(1000.0, 1000.0, &faults, 1.0);
        }
        assert!(s.chip_detected, "{} g accumulated", s.debris_g);
        assert!(s.overall_wear == 1.0);
    }

    #[test]
    fn a_minor_brief_spall_does_not_trip_the_detector() {
        let mut w = BearingWear::new();
        let faults = BearingFaults { cage_wear: 0.1, ..Default::default() };
        let s = w.step(1000.0, 1000.0, &faults, 5.0);
        assert!(!s.chip_detected);
    }

    #[test]
    fn clearing_the_chip_detector_resets_accumulated_debris() {
        let mut w = BearingWear::new();
        let faults = BearingFaults { inner_race_spall: 1.0, ..Default::default() };
        w.step(1000.0, 1000.0, &faults, 2000.0);
        assert!(w.debris_g() > 0.0);
        w.clear_chip_detector();
        assert_eq!(w.debris_g(), 0.0);
    }

    #[test]
    fn engine_bearings_routes_each_bearing_to_its_own_spool_speed() {
        let mut eb = EngineBearings::new();
        let mut faults = EngineBearingFaults::default();
        // Fault only the HP turbine bearing; drive HP much faster than LP/IP.
        let hp_index = BEARINGS.iter().position(|b| *b == BearingId::HpTurbine).unwrap();
        faults.faults[hp_index] = BearingFaults { outer_race_spall: 1.0, ..Default::default() };
        let out = eb.step(50.0, 100.0, 1000.0, 300.0, 800.0, 1200.0, &faults, 1.0);
        let fan_index = BEARINGS.iter().position(|b| *b == BearingId::FanFront).unwrap();
        assert!(out[hp_index].signature.outer_race.1 > 0.0);
        assert!(out[fan_index].signature.outer_race.1 == 0.0, "the fan bearing has no fault and should be silent");
        // The HP bearing's defect frequency should reflect the HP spool's
        // much higher speed, not the LP spool's.
        assert!(out[hp_index].signature.outer_race.0 > out[fan_index].signature.cage.0 * 10.0);
    }
}
