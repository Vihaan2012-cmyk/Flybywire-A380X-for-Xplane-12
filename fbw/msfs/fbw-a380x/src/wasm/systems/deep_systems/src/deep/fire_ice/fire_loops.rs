use super::util::clamp01;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Zone {
    Engine(u8),
    Apu,
    MainGearBay,
    CargoFwd,
    CargoAft,
    Avionics,
}

pub const ZONES: [Zone; 9] = [
    Zone::Engine(1),
    Zone::Engine(2),
    Zone::Engine(3),
    Zone::Engine(4),
    Zone::Apu,
    Zone::MainGearBay,
    Zone::CargoFwd,
    Zone::CargoAft,
    Zone::Avionics,
];

const THERMISTOR_BETA_K: f64 = 3435.0;
const THERMISTOR_T0_K: f64 = 298.15;
const THERMISTOR_R0_OHM: f64 = 10_000.0;

pub const FIRE_TRIP_C: f64 = 200.0;
const COLD_SOAK_C: f64 = -65.0;

fn thermistor_resistance_ohm(temp_c: f64) -> f64 {
    let t_k = (temp_c + 273.15).max(1.0);
    THERMISTOR_R0_OHM * (THERMISTOR_BETA_K * (1.0 / t_k - 1.0 / THERMISTOR_T0_K)).exp()
}

fn thermistor_trip_resistance_ohm() -> f64 {
    thermistor_resistance_ohm(FIRE_TRIP_C)
}

fn thermistor_fault_resistance_ohm() -> f64 {
    thermistor_resistance_ohm(COLD_SOAK_C) * 5.0
}

const PNEUMATIC_P0_PA: f64 = 150_000.0;
const PNEUMATIC_T0_K: f64 = 298.15;
const PNEUMATIC_DISCRETE_BONUS_PA: f64 = 200_000.0;
const PNEUMATIC_DISCRETE_RELEASE_C: f64 = 150.0;
const PNEUMATIC_DISCRETE_SPAN_C: f64 = 20.0;

pub const PNEUMATIC_TRIP_PA: f64 = PNEUMATIC_P0_PA * (FIRE_TRIP_C + 273.15) / PNEUMATIC_T0_K;

fn pneumatic_pressure_pa(avg_c: f64, hot_spot_c: f64) -> f64 {
    let avg_k = (avg_c + 273.15).max(1.0);
    let average_response = PNEUMATIC_P0_PA * avg_k / PNEUMATIC_T0_K;
    let discrete_weight = clamp01((hot_spot_c - PNEUMATIC_DISCRETE_RELEASE_C) / PNEUMATIC_DISCRETE_SPAN_C);
    average_response + PNEUMATIC_DISCRETE_BONUS_PA * discrete_weight
}

fn pneumatic_fault_pressure_pa() -> f64 {
    PNEUMATIC_P0_PA * (COLD_SOAK_C + 273.15) / PNEUMATIC_T0_K * 0.3
}

#[derive(Clone, Copy, Debug, Default)]
pub struct LoopFaults {
    pub open_circuit: f64,
    pub short_circuit: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Technology {
    Thermistor,
    Pneumatic,
}

#[derive(Clone, Copy, Debug)]
pub struct DetectorLoop {
    pub technology: Technology,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LoopReading {
    pub raw: f64,
    pub fire_signal: bool,
    pub loop_fault: bool,
}

impl DetectorLoop {
    pub fn new(technology: Technology) -> Self {
        Self { technology }
    }

    pub fn sense(&self, average_zone_c: f64, hot_spot_c: f64, faults: LoopFaults) -> LoopReading {
        let sensed_c = average_zone_c.max(hot_spot_c);
        let open = clamp01(faults.open_circuit);
        let short = clamp01(faults.short_circuit);

        match self.technology {
            Technology::Thermistor => {
                let healthy = thermistor_resistance_ohm(sensed_c);
                let with_short = healthy * (1.0 - short);
                let raw = with_short + (thermistor_fault_resistance_ohm() - with_short) * open;
                let fire_signal = raw < thermistor_trip_resistance_ohm();
                let loop_fault = raw > thermistor_resistance_ohm(COLD_SOAK_C) * 1.5;
                LoopReading { raw, fire_signal, loop_fault }
            }
            Technology::Pneumatic => {
                let healthy = pneumatic_pressure_pa(average_zone_c, hot_spot_c);
                let with_short = healthy + (PNEUMATIC_P0_PA * 3.0 - healthy) * short;
                let raw = with_short + (pneumatic_fault_pressure_pa() - with_short) * open;
                let fire_signal = raw > PNEUMATIC_TRIP_PA;
                let loop_fault = raw < PNEUMATIC_P0_PA * (COLD_SOAK_C + 273.15) / PNEUMATIC_T0_K * 0.6;
                LoopReading { raw, fire_signal, loop_fault }
            }
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoopLogic {
    And,
    Or,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ZoneFireStatus {
    pub fire: bool,
    pub loop_a_fault: bool,
    pub loop_b_fault: bool,
    pub loop_a_signal: bool,
    pub loop_b_signal: bool,
}

impl ZoneFireStatus {
    pub fn loop_disagree(&self) -> bool {
        self.loop_a_signal != self.loop_b_signal
    }
}

pub struct ZoneDetector {
    pub loop_a: DetectorLoop,
    pub loop_b: DetectorLoop,
    pub logic: LoopLogic,
}

impl ZoneDetector {
    pub fn new(logic: LoopLogic) -> Self {
        Self { loop_a: DetectorLoop::new(Technology::Thermistor), loop_b: DetectorLoop::new(Technology::Pneumatic), logic }
    }

    pub fn evaluate(&self, average_zone_c: f64, hot_spot_c: f64, faults_a: LoopFaults, faults_b: LoopFaults) -> ZoneFireStatus {
        let a = self.loop_a.sense(average_zone_c, hot_spot_c, faults_a);
        let b = self.loop_b.sense(average_zone_c, hot_spot_c, faults_b);

        let fire = if a.loop_fault != b.loop_fault {
            (a.fire_signal && !a.loop_fault) || (b.fire_signal && !b.loop_fault)
        } else if a.loop_fault && b.loop_fault {
            true
        } else {
            match self.logic {
                LoopLogic::And => a.fire_signal && b.fire_signal,
                LoopLogic::Or => a.fire_signal || b.fire_signal,
            }
        };

        ZoneFireStatus { fire, loop_a_fault: a.loop_fault, loop_b_fault: b.loop_fault, loop_a_signal: a.fire_signal, loop_b_signal: b.fire_signal }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn thermistor_resistance_falls_as_temperature_rises() {
        assert!(thermistor_resistance_ohm(20.0) > thermistor_resistance_ohm(200.0));
    }

    #[test]
    fn healthy_thermistor_loop_trips_at_fire_temperature_and_not_below() {
        let l = DetectorLoop::new(Technology::Thermistor);
        let cold = l.sense(20.0, 20.0, LoopFaults::default());
        assert!(!cold.fire_signal && !cold.loop_fault);
        let hot = l.sense(FIRE_TRIP_C + 50.0, FIRE_TRIP_C + 50.0, LoopFaults::default());
        assert!(hot.fire_signal && !hot.loop_fault);
    }

    #[test]
    fn thermistor_short_circuit_produces_a_false_fire_not_a_fault() {
        let l = DetectorLoop::new(Technology::Thermistor);
        let shorted = l.sense(20.0, 20.0, LoopFaults { short_circuit: 1.0, ..Default::default() });
        assert!(shorted.fire_signal, "a dead short must read as fire");
        assert!(!shorted.loop_fault, "a short is not distinguishable from real heat, so must not read as a fault");
    }

    #[test]
    fn thermistor_open_circuit_produces_a_fault_not_a_fire() {
        let l = DetectorLoop::new(Technology::Thermistor);
        let open = l.sense(20.0, 20.0, LoopFaults { open_circuit: 1.0, ..Default::default() });
        assert!(open.loop_fault);
        assert!(!open.fire_signal);
    }

    #[test]
    fn pneumatic_loop_average_response_rises_with_temperature() {
        assert!(pneumatic_pressure_pa(20.0, 20.0) < pneumatic_pressure_pa(FIRE_TRIP_C, FIRE_TRIP_C));
    }

    #[test]
    fn pneumatic_discrete_response_trips_on_a_local_hot_spot_below_the_average_fire_threshold() {
        let l = DetectorLoop::new(Technology::Pneumatic);
        let localized = l.sense(30.0, 160.0, LoopFaults::default());
        assert!(localized.fire_signal, "{:?}", localized);
    }

    #[test]
    fn pneumatic_short_is_false_fire_and_open_is_fault() {
        let l = DetectorLoop::new(Technology::Pneumatic);
        let shorted = l.sense(20.0, 20.0, LoopFaults { short_circuit: 1.0, ..Default::default() });
        assert!(shorted.fire_signal && !shorted.loop_fault);
        let open = l.sense(20.0, 20.0, LoopFaults { open_circuit: 1.0, ..Default::default() });
        assert!(open.loop_fault && !open.fire_signal);
    }

    #[test]
    fn and_logic_needs_both_loops_while_or_logic_needs_only_one() {
        let and_zone = ZoneDetector::new(LoopLogic::And);
        let or_zone = ZoneDetector::new(LoopLogic::Or);
        let faults_a = LoopFaults { short_circuit: 1.0, ..Default::default() };
        let and_status = and_zone.evaluate(20.0, 20.0, faults_a, LoopFaults::default());
        let or_status = or_zone.evaluate(20.0, 20.0, faults_a, LoopFaults::default());
        assert!(!and_status.fire, "AND logic must not trip on a single disagreeing loop");
        assert!(or_status.fire, "OR logic must trip on either loop alone");
    }

    #[test]
    fn a_faulted_loop_falls_back_to_trusting_the_other_loop_even_under_and_logic() {
        let zone = ZoneDetector::new(LoopLogic::And);
        let faults_a = LoopFaults { open_circuit: 1.0, ..Default::default() };
        let status = zone.evaluate(FIRE_TRIP_C + 50.0, FIRE_TRIP_C + 50.0, faults_a, LoopFaults::default());
        assert!(status.fire, "with A faulted, a real fire on healthy loop B alone must still be declared");
        assert!(status.loop_a_fault && !status.loop_b_fault);
    }

    #[test]
    fn both_loops_faulted_simultaneously_fails_toward_declaring_fire() {
        let zone = ZoneDetector::new(LoopLogic::And);
        let faults_a = LoopFaults { open_circuit: 1.0, ..Default::default() };
        let faults_b = LoopFaults { open_circuit: 1.0, ..Default::default() };
        let status = zone.evaluate(20.0, 20.0, faults_a, faults_b);
        assert!(status.loop_a_fault && status.loop_b_fault);
        assert!(status.fire, "total simultaneous loss of both loops must fail safe toward presumed fire");
    }

    #[test]
    fn a_lone_shorted_loop_annunciates_disagreement_while_the_zone_withholds_fire() {
        let zone = ZoneDetector::new(LoopLogic::And);
        let shorted_a = LoopFaults { short_circuit: 1.0, ..Default::default() };
        let status = zone.evaluate(20.0, 20.0, shorted_a, LoopFaults::default());
        assert!(!status.fire, "AND logic must still withhold the zone-level fire warning");
        assert!(!status.loop_a_fault && !status.loop_b_fault, "a short is not a loop fault");
        assert!(status.loop_a_signal, "the shorted loop's own raw reading must say fire");
        assert!(!status.loop_b_signal, "the healthy loop's own raw reading must say no fire");
        assert!(status.loop_disagree(), "the two loops disagreeing must be its own visible discrete");
    }

    #[test]
    fn two_healthy_loops_agreeing_never_disagree() {
        let zone = ZoneDetector::new(LoopLogic::And);
        let cold = zone.evaluate(20.0, 20.0, LoopFaults::default(), LoopFaults::default());
        assert!(!cold.loop_disagree());
        let hot = zone.evaluate(FIRE_TRIP_C + 50.0, FIRE_TRIP_C + 50.0, LoopFaults::default(), LoopFaults::default());
        assert!(!hot.loop_disagree());
        assert!(hot.fire);
    }

    #[test]
    fn zones_constant_lists_four_engines_and_five_other_zones() {
        let engines = ZONES.iter().filter(|z| matches!(z, Zone::Engine(_))).count();
        assert_eq!(engines, 4);
        assert_eq!(ZONES.len(), 9);
    }
}
