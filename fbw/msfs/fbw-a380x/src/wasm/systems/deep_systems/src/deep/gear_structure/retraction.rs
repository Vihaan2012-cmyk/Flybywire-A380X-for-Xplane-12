use super::LegKind;

const GEAR_NOMINAL_TRAVEL_S: f64 = 8.0;
const DOOR_NOMINAL_TRAVEL_S: f64 = 4.0;
const GRAVITY_EXTEND_TRAVEL_S: f64 = 66.0;
const GRAVITY_EXTEND_TOTAL_S: f64 = 70.0;

const DOOR_OPEN_THRESHOLD: f64 = 0.98;
const DOOR_CLOSED_THRESHOLD: f64 = 0.02;
const GEAR_UP_THRESHOLD: f64 = 0.02;
const GEAR_DOWN_THRESHOLD: f64 = 0.98;

const MIN_RELEASE_PRESSURE_FRACTION: f64 = 0.3;
const UPLOCK_JAM_DEFEATS_HYDRAULIC: f64 = 0.5;
const UPLOCK_JAM_DEFEATS_EMERGENCY: f64 = 0.95;
pub(crate) const DOWNLOCK_ENGAGE_FAULT_THRESHOLD: f64 = 0.5;
const DOOR_JAM_FREEZE_THRESHOLD: f64 = 0.5;
const SENSOR_LIE_THRESHOLD: f64 = 0.5;

fn kind_rate_scale(kind: LegKind) -> f64 {
    match kind {
        LegKind::Nose => 1.3,
        LegKind::Wing | LegKind::Body => 1.0,
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct RetractionFaults {
    pub actuator_leak: f64,
    pub uplock_jam: f64,
    pub downlock_fail: f64,
    pub door_jam: f64,
    pub sensor_lies: f64,
    pub bogie_trim_fail: f64,
}

const BOGIE_TRIM_FAIL_THRESHOLD: f64 = 0.5;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    Locked,
    DoorsOpening,
    Traveling,
    DoorsClosing,
}

pub fn ground_interlock(gear_lever_down: bool, on_ground: bool) -> bool {
    gear_lever_down || on_ground
}

#[derive(Clone, Copy, Debug)]
pub struct RetractionInputs {
    pub gear_lever_down: bool,
    pub gravity_extend_commanded: bool,
    pub hydraulic_pressure_fraction: f64,
    pub dt_s: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct RetractionOutputs {
    pub gear_position: f64,
    pub door_position: f64,
    pub uplocked: bool,
    pub downlocked: bool,
    pub sensed_uplocked: bool,
    pub sensed_downlocked: bool,
    pub phase: Phase,
    pub stuck_locked: bool,
    pub bogie_trimmed: bool,
}

pub struct Retraction {
    kind: LegKind,
    phase: Phase,
    position_down: bool,
    target_down: bool,
    gravity_mode: bool,
    gear_position: f64,
    door_position: f64,
    uplocked: bool,
    downlocked: bool,
    stuck_locked: bool,
}

impl Retraction {
    pub fn new(kind: LegKind) -> Self {
        Self {
            kind,
            phase: Phase::Locked,
            position_down: true,
            target_down: true,
            gravity_mode: false,
            gear_position: 1.0,
            door_position: 0.0,
            uplocked: false,
            downlocked: true,
            stuck_locked: false,
        }
    }

    pub fn step(&mut self, inputs: &RetractionInputs, faults: &RetractionFaults) -> RetractionOutputs {
        let dt = inputs.dt_s.max(0.0);
        let commanded_down = inputs.gear_lever_down || inputs.gravity_extend_commanded;
        let scale = kind_rate_scale(self.kind);

        if self.phase == Phase::Locked {
            self.stuck_locked = false;
            if self.position_down != commanded_down {
                if !self.position_down && commanded_down {
                    let can_release = if inputs.gravity_extend_commanded {
                        faults.uplock_jam < UPLOCK_JAM_DEFEATS_EMERGENCY
                    } else {
                        inputs.hydraulic_pressure_fraction >= MIN_RELEASE_PRESSURE_FRACTION && faults.uplock_jam < UPLOCK_JAM_DEFEATS_HYDRAULIC
                    };
                    if can_release {
                        self.uplocked = false;
                        self.target_down = true;
                        self.gravity_mode = inputs.gravity_extend_commanded;
                        self.phase = Phase::DoorsOpening;
                    } else {
                        self.stuck_locked = true;
                    }
                } else {
                    self.downlocked = false;
                    self.target_down = false;
                    self.gravity_mode = false;
                    self.phase = Phase::DoorsOpening;
                }
            }
        }

        let jam = faults.door_jam.clamp(0.0, 1.0);
        let door_rate = if jam >= DOOR_JAM_FREEZE_THRESHOLD { 0.0 } else { scale * (1.0 - jam / DOOR_JAM_FREEZE_THRESHOLD) / DOOR_NOMINAL_TRAVEL_S };

        match self.phase {
            Phase::Locked => {}
            Phase::DoorsOpening => {
                self.door_position = (self.door_position + door_rate * dt).min(1.0);
                let bogie_ok = self.kind != LegKind::Body || self.target_down || faults.bogie_trim_fail < BOGIE_TRIM_FAIL_THRESHOLD;
                if self.door_position >= DOOR_OPEN_THRESHOLD && bogie_ok {
                    self.phase = Phase::Traveling;
                }
            }
            Phase::Traveling => {
                let leak_factor = (1.0 - faults.actuator_leak.clamp(0.0, 1.0)).max(0.0);
                let gear_rate = if self.gravity_mode {
                    scale / GRAVITY_EXTEND_TRAVEL_S
                } else {
                    scale * inputs.hydraulic_pressure_fraction.clamp(0.0, 2.0) * leak_factor / GEAR_NOMINAL_TRAVEL_S
                };
                if self.target_down {
                    self.gear_position = (self.gear_position + gear_rate * dt).min(1.0);
                    if self.gear_position >= GEAR_DOWN_THRESHOLD {
                        self.downlocked = faults.downlock_fail < DOWNLOCK_ENGAGE_FAULT_THRESHOLD;
                        self.position_down = true;
                        self.phase = Phase::DoorsClosing;
                    }
                } else {
                    self.gear_position = (self.gear_position - gear_rate * dt).max(0.0);
                    if self.gear_position <= GEAR_UP_THRESHOLD {
                        self.uplocked = true;
                        self.position_down = false;
                        self.phase = Phase::DoorsClosing;
                    }
                }
            }
            Phase::DoorsClosing => {
                self.door_position = (self.door_position - door_rate * dt).max(0.0);
                if self.door_position <= DOOR_CLOSED_THRESHOLD {
                    self.phase = Phase::Locked;
                }
            }
        }

        let sensed_invert = faults.sensor_lies >= SENSOR_LIE_THRESHOLD;
        RetractionOutputs {
            gear_position: self.gear_position,
            door_position: self.door_position,
            uplocked: self.uplocked,
            downlocked: self.downlocked,
            sensed_uplocked: self.uplocked != sensed_invert,
            sensed_downlocked: self.downlocked != sensed_invert,
            phase: self.phase,
            stuck_locked: self.stuck_locked,
            bogie_trimmed: faults.bogie_trim_fail < BOGIE_TRIM_FAIL_THRESHOLD,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn healthy() -> RetractionFaults {
        RetractionFaults::default()
    }

    fn run_to_locked(r: &mut Retraction, inputs: &RetractionInputs, faults: &RetractionFaults, max_ticks: u32) -> RetractionOutputs {
        let mut out = r.step(inputs, faults);
        for _ in 0..max_ticks {
            if out.phase == Phase::Locked {
                break;
            }
            out = r.step(inputs, faults);
        }
        out
    }

    #[test]
    fn starts_down_and_locked() {
        let mut r = Retraction::new(LegKind::Wing);
        let inputs = RetractionInputs { gear_lever_down: true, gravity_extend_commanded: false, hydraulic_pressure_fraction: 1.0, dt_s: 0.1 };
        let out = r.step(&inputs, &healthy());
        assert_eq!(out.phase, Phase::Locked);
        assert!(out.downlocked);
        assert!((out.gear_position - 1.0).abs() < 1e-9);
    }

    #[test]
    fn gravity_extension_takes_the_seventy_seconds_the_fcom_gives() {
        let mut r = Retraction::new(LegKind::Wing);
        let faults = healthy();
        let up = RetractionInputs { gear_lever_down: false, gravity_extend_commanded: false, hydraulic_pressure_fraction: 1.0, dt_s: 0.1 };
        let out = run_to_locked(&mut r, &up, &faults, 4_000);
        assert!(out.uplocked);

        let dt = 0.1;
        let drop = RetractionInputs { gear_lever_down: false, gravity_extend_commanded: true, hydraulic_pressure_fraction: 0.0, dt_s: dt };
        let mut seconds = 0.0;
        for _ in 0..4_000 {
            let out = r.step(&drop, &faults);
            seconds += dt;
            if out.phase == Phase::Locked && out.gear_position > GEAR_DOWN_THRESHOLD {
                break;
            }
        }
        assert!(
            (seconds - GRAVITY_EXTEND_TOTAL_S).abs() < 2.0,
            "gravity extension should take about {GRAVITY_EXTEND_TOTAL_S} s, took {seconds}"
        );
        assert!(seconds > 4.0 * (GEAR_NOMINAL_TRAVEL_S + DOOR_NOMINAL_TRAVEL_S));
    }

    #[test]
    fn a_full_retract_then_extend_cycle_locks_at_both_ends() {
        let mut r = Retraction::new(LegKind::Wing);
        let faults = healthy();
        let up_inputs = RetractionInputs { gear_lever_down: false, gravity_extend_commanded: false, hydraulic_pressure_fraction: 1.0, dt_s: 0.1 };
        let out = run_to_locked(&mut r, &up_inputs, &faults, 2_000);
        assert_eq!(out.phase, Phase::Locked);
        assert!(out.uplocked);
        assert!(out.gear_position < 0.05);
        assert!(out.door_position < 0.05, "doors must close again once up-locked");

        let down_inputs = RetractionInputs { gear_lever_down: true, gravity_extend_commanded: false, hydraulic_pressure_fraction: 1.0, dt_s: 0.1 };
        let out = run_to_locked(&mut r, &down_inputs, &faults, 2_000);
        assert_eq!(out.phase, Phase::Locked);
        assert!(out.downlocked);
        assert!(out.gear_position > 0.95);
    }

    #[test]
    fn a_jammed_uplock_prevents_ever_leaving_the_uplocked_state_hydraulically() {
        let mut r = Retraction::new(LegKind::Wing);
        let faults = RetractionFaults { uplock_jam: 0.8, ..Default::default() };
        let up_inputs = RetractionInputs { gear_lever_down: false, gravity_extend_commanded: false, hydraulic_pressure_fraction: 1.0, dt_s: 0.1 };
        let out = run_to_locked(&mut r, &up_inputs, &RetractionFaults::default(), 2_000);
        assert!(out.uplocked);

        let down_inputs = RetractionInputs { gear_lever_down: true, gravity_extend_commanded: false, hydraulic_pressure_fraction: 1.0, dt_s: 0.1 };
        let mut out = r.step(&down_inputs, &faults);
        for _ in 0..49 {
            out = r.step(&down_inputs, &faults);
        }
        assert!(out.stuck_locked, "a jammed uplock must prevent the normal release, not just slow it");
        assert_eq!(out.phase, Phase::Locked);
        assert!(out.gear_position < 0.05, "the gear must not have moved at all");

        let gravity_inputs = RetractionInputs { gear_lever_down: true, gravity_extend_commanded: true, hydraulic_pressure_fraction: 0.0, dt_s: 0.1 };
        let out = run_to_locked(&mut r, &gravity_inputs, &faults, 5_000);
        assert!(out.downlocked, "gravity extension must still succeed through a moderate uplock jam");
        assert!(out.gear_position > 0.95);
    }

    #[test]
    fn a_severe_uplock_jam_defeats_even_gravity_extension() {
        let mut r = Retraction::new(LegKind::Nose);
        let faults = RetractionFaults { uplock_jam: 0.99, ..Default::default() };
        let up_inputs = RetractionInputs { gear_lever_down: false, gravity_extend_commanded: false, hydraulic_pressure_fraction: 1.0, dt_s: 0.1 };
        let out = run_to_locked(&mut r, &up_inputs, &RetractionFaults::default(), 2_000);
        assert!(out.uplocked);

        let gravity_inputs = RetractionInputs { gear_lever_down: true, gravity_extend_commanded: true, hydraulic_pressure_fraction: 0.0, dt_s: 0.1 };
        let mut out = r.step(&gravity_inputs, &faults);
        for _ in 0..50 {
            out = r.step(&gravity_inputs, &faults);
        }
        assert!(out.stuck_locked, "a severe enough jam must defeat gravity extension too");
    }

    #[test]
    fn a_failed_downlock_reaches_the_down_position_but_never_reports_locked() {
        let mut r = Retraction::new(LegKind::Body);
        let faults = RetractionFaults { downlock_fail: 0.9, ..Default::default() };
        let inputs = RetractionInputs { gear_lever_down: true, gravity_extend_commanded: false, hydraulic_pressure_fraction: 1.0, dt_s: 0.1 };
        let up_inputs = RetractionInputs { gear_lever_down: false, ..inputs };
        run_to_locked(&mut r, &up_inputs, &RetractionFaults::default(), 2_000);
        let out = run_to_locked(&mut r, &inputs, &faults, 2_000);
        assert!(out.gear_position > 0.95, "the leg still reaches the down position geometrically");
        assert!(!out.downlocked, "but the downlock never truly engages");
    }

    #[test]
    fn a_lying_sensor_inverts_only_the_indication_not_the_truth() {
        let mut r = Retraction::new(LegKind::Wing);
        let faults = RetractionFaults { sensor_lies: 1.0, ..Default::default() };
        let inputs = RetractionInputs { gear_lever_down: true, gravity_extend_commanded: false, hydraulic_pressure_fraction: 1.0, dt_s: 0.1 };
        let out = r.step(&inputs, &faults);
        assert!(out.downlocked, "truth: still genuinely down-locked at start");
        assert!(!out.sensed_downlocked, "indication: lying sensor shows the opposite");
    }

    #[test]
    fn a_frozen_door_jam_prevents_the_gear_from_ever_starting_to_move() {
        let mut r = Retraction::new(LegKind::Wing);
        let faults = RetractionFaults { door_jam: 0.9, ..Default::default() };
        let up_inputs = RetractionInputs { gear_lever_down: false, gravity_extend_commanded: false, hydraulic_pressure_fraction: 1.0, dt_s: 0.1 };
        let mut out = r.step(&up_inputs, &faults);
        for _ in 0..500 {
            out = r.step(&up_inputs, &faults);
        }
        assert_eq!(out.phase, Phase::DoorsOpening, "a fully seized door must block the sequence indefinitely");
        assert!(out.gear_position > 0.95, "the gear itself must not have moved while the doors are stuck");
    }

    #[test]
    fn a_bogie_trim_failure_only_stalls_a_body_legs_retraction() {
        let mut r = Retraction::new(LegKind::Body);
        let faults = RetractionFaults { bogie_trim_fail: 1.0, ..Default::default() };
        let up_inputs = RetractionInputs { gear_lever_down: false, gravity_extend_commanded: false, hydraulic_pressure_fraction: 1.0, dt_s: 0.1 };
        let mut out = r.step(&up_inputs, &faults);
        for _ in 0..2_000 {
            out = r.step(&up_inputs, &faults);
        }
        assert!(!out.bogie_trimmed, "the BITE flag must report the failure");
        assert_eq!(out.phase, Phase::DoorsOpening, "a body leg must stall in DoorsOpening once its bogie fails to trim");
        assert!(out.door_position >= DOOR_OPEN_THRESHOLD, "the door itself must have finished opening -- it is the bogie gate that is stuck");
        assert!(out.gear_position > 0.95, "the leg itself must never have started travelling away from the down position, got {}", out.gear_position);

        let mut healthy_extend = Retraction::new(LegKind::Body);
        let down_inputs = RetractionInputs { gear_lever_down: true, ..up_inputs };
        let extend_out = run_to_locked(&mut healthy_extend, &down_inputs, &faults, 2_000);
        assert_eq!(extend_out.phase, Phase::Locked, "extending must not be blocked by a bogie-trim failure");
        assert!(extend_out.downlocked);

        let mut nose = Retraction::new(LegKind::Nose);
        let nose_out = run_to_locked(&mut nose, &up_inputs, &faults, 2_000);
        assert_eq!(nose_out.phase, Phase::Locked, "a nose leg must be unaffected by bogie_trim_fail, which does not apply to it");
        assert!(nose_out.uplocked);
    }

    #[test]
    fn numerically_safe_at_rest_and_dt_zero() {
        let mut r = Retraction::new(LegKind::Nose);
        let inputs = RetractionInputs { gear_lever_down: true, gravity_extend_commanded: false, hydraulic_pressure_fraction: 1.0, dt_s: 0.0 };
        let out = r.step(&inputs, &healthy());
        assert!(out.gear_position.is_finite() && out.door_position.is_finite());
    }
}
