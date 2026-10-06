use super::bearing::{signature, BearingFaults, BearingSignature};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Spool {
    Lp,
    Ip,
    Hp,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OilChamber {
    Front,
    HpIp,
    Tail,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BearingId {
    FanFront,
    IpFront,
    HpTurbine,
    IpTurbine,
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

const DEBRIS_RATE_G_S: f64 = 6.0e-4;
const CHIP_DETECTOR_THRESHOLD_G: f64 = 0.5;

#[derive(Clone, Copy, Debug, Default)]
pub struct BearingWear {
    debris_g: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BearingState {
    pub signature: BearingSignature,
    pub overall_wear: f64,
    pub debris_g: f64,
    pub chip_detected: bool,
}

impl BearingWear {
    pub fn new() -> Self {
        Self { debris_g: 0.0 }
    }

    pub fn clear_chip_detector(&mut self) {
        self.debris_g = 0.0;
    }

    pub fn debris_g(&self) -> f64 {
        self.debris_g
    }

    pub fn step(&mut self, omega_rad_s: f64, design_omega_rad_s: f64, faults: &BearingFaults, dt_s: f64) -> BearingState {
        let dt = dt_s.max(0.0);
        let sig = signature(omega_rad_s, design_omega_rad_s, faults);
        let overall_wear = faults.outer_race_spall.max(faults.inner_race_spall).max(faults.rolling_element_spall).max(faults.cage_wear).clamp(0.0, 1.0);
        let speed_frac = (omega_rad_s.max(0.0) / design_omega_rad_s.max(1e-6)).min(1.5);
        self.debris_g += DEBRIS_RATE_G_S * overall_wear * speed_frac * dt;
        BearingState { signature: sig, overall_wear, debris_g: self.debris_g, chip_detected: self.debris_g >= CHIP_DETECTOR_THRESHOLD_G }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct EngineBearingFaults {
    pub faults: [BearingFaults; 5],
}

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
        let hp_index = BEARINGS.iter().position(|b| *b == BearingId::HpTurbine).unwrap();
        faults.faults[hp_index] = BearingFaults { outer_race_spall: 1.0, ..Default::default() };
        let out = eb.step(50.0, 100.0, 1000.0, 300.0, 800.0, 1200.0, &faults, 1.0);
        let fan_index = BEARINGS.iter().position(|b| *b == BearingId::FanFront).unwrap();
        assert!(out[hp_index].signature.outer_race.1 > 0.0);
        assert!(out[fan_index].signature.outer_race.1 == 0.0, "the fan bearing has no fault and should be silent");
        assert!(out[hp_index].signature.outer_race.0 > out[fan_index].signature.cage.0 * 10.0);
    }
}
