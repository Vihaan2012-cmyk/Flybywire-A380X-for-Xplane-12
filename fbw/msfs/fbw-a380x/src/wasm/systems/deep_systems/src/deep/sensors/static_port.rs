pub const REFERENCE_AOA_DEG: f64 = 2.5;
const POSITION_ERROR_COEFF: f64 = 0.0025;
const MACH_SENSITIVITY: f64 = 0.4;

const STATIC_PORT_TAU_S: f64 = 0.2;

#[derive(Clone, Copy, Debug, Default)]
pub struct StaticPortFaults {
    pub blocked: f64,
    pub leak_to_cabin: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct StaticPortOutput {
    pub sensed_static_pressure_pa: f64,
    pub blocked: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct StaticPort {
    sensed_pa: f64,
}

impl StaticPort {
    pub fn new(initial_static_pa: f64) -> Self {
        Self { sensed_pa: initial_static_pa }
    }

    pub fn step(
        &mut self,
        true_static_pa: f64,
        cabin_pa: f64,
        alpha_deg: f64,
        mach: f64,
        faults: &StaticPortFaults,
        dt_s: f64,
    ) -> StaticPortOutput {
        let dt = dt_s.max(0.0);
        let blocked = faults.blocked.clamp(0.0, 1.0) >= 0.98;
        if blocked {
            return StaticPortOutput { sensed_static_pressure_pa: self.sensed_pa, blocked: true };
        }

        let position_error_pa = true_static_pa
            * POSITION_ERROR_COEFF
            * (alpha_deg - REFERENCE_AOA_DEG)
            * (1.0 + MACH_SENSITIVITY * mach.max(0.0));
        let true_reading_pa = true_static_pa + position_error_pa;

        let own_conductance = (1.0 - faults.blocked.clamp(0.0, 1.0)).max(0.0);
        let leak_conductance = faults.leak_to_cabin.max(0.0);
        let total_conductance = own_conductance + leak_conductance;
        let ambient_and_leak_pa = if total_conductance > 1e-9 {
            (own_conductance * true_reading_pa + leak_conductance * cabin_pa) / total_conductance
        } else {
            self.sensed_pa
        };
        let conductance = own_conductance.max(0.02);
        let tau = STATIC_PORT_TAU_S * (1.0 / conductance - 1.0);
        let k = if tau <= 1e-9 { 0.0 } else { (-dt / tau).exp() };
        self.sensed_pa = ambient_and_leak_pa + (self.sensed_pa - ambient_and_leak_pa) * k;

        StaticPortOutput { sensed_static_pressure_pa: self.sensed_pa, blocked: false }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct StaticAveragingLineFaults {
    pub line_blocked: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct StaticPortPairOutput {
    pub averaged_pa: f64,
    pub degraded: bool,
}

pub fn average_pair(left: StaticPortOutput, right: StaticPortOutput, faults: &StaticAveragingLineFaults) -> StaticPortPairOutput {
    if faults.line_blocked.clamp(0.0, 1.0) >= 0.98 {
        return StaticPortPairOutput { averaged_pa: left.sensed_static_pressure_pa, degraded: true };
    }
    let (left_weight, right_weight) = match (left.blocked, right.blocked) {
        (true, false) => (0.0, 1.0),
        (false, true) => (1.0, 0.0),
        _ => (0.5, 0.5),
    };
    let averaged_pa = left_weight * left.sensed_static_pressure_pa + right_weight * right.sensed_static_pressure_pa;
    StaticPortPairOutput { averaged_pa, degraded: left.blocked || right.blocked }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn healthy_port_matches_true_static_pressure_at_reference_aoa() {
        let mut port = StaticPort::new(101_325.0);
        let out = port.step(95_000.0, 79_000.0, REFERENCE_AOA_DEG, 0.3, &StaticPortFaults::default(), 0.1);
        assert!((out.sensed_static_pressure_pa - 95_000.0).abs() < 1.0);
        assert!(!out.blocked);
    }

    #[test]
    fn position_error_flips_sign_either_side_of_reference_aoa() {
        let mut low = StaticPort::new(95_000.0);
        let mut high = StaticPort::new(95_000.0);
        let lo = low.step(95_000.0, 79_000.0, REFERENCE_AOA_DEG - 5.0, 0.3, &StaticPortFaults::default(), 0.1);
        let hi = high.step(95_000.0, 79_000.0, REFERENCE_AOA_DEG + 5.0, 0.3, &StaticPortFaults::default(), 0.1);
        assert!(lo.sensed_static_pressure_pa < 95_000.0);
        assert!(hi.sensed_static_pressure_pa > 95_000.0);
    }

    #[test]
    fn blocked_port_freezes_the_last_reading() {
        let mut port = StaticPort::new(95_000.0);
        let before = port.step(95_000.0, 79_000.0, REFERENCE_AOA_DEG, 0.3, &StaticPortFaults::default(), 0.1);
        let faults = StaticPortFaults { blocked: 1.0, ..Default::default() };
        let mut out = before;
        for _ in 0..50 {
            out = port.step(40_000.0, 79_000.0, REFERENCE_AOA_DEG, 0.8, &faults, 0.1);
        }
        assert!(out.blocked);
        assert_eq!(out.sensed_static_pressure_pa, before.sensed_static_pressure_pa);
    }

    #[test]
    fn a_large_leak_pulls_the_reading_toward_cabin_pressure() {
        let mut port = StaticPort::new(30_000.0);
        let faults = StaticPortFaults { leak_to_cabin: 5.0, ..Default::default() };
        let mut out = StaticPortOutput::default();
        for _ in 0..500 {
            out = port.step(30_000.0, 79_000.0, REFERENCE_AOA_DEG, 0.8, &faults, 0.1);
        }
        assert!(out.sensed_static_pressure_pa > 65_000.0, "{}", out.sensed_static_pressure_pa);
    }

    #[test]
    fn no_leak_is_unaffected_by_cabin_pressure() {
        let mut port = StaticPort::new(95_000.0);
        let out = port.step(95_000.0, 40_000.0, REFERENCE_AOA_DEG, 0.3, &StaticPortFaults::default(), 0.1);
        assert!((out.sensed_static_pressure_pa - 95_000.0).abs() < 1.0);
    }

    #[test]
    fn partial_restriction_response_time_constant_is_independent_of_step_size() {
        let faults = StaticPortFaults { blocked: 0.5, ..Default::default() };
        let mut coarse = StaticPort::new(101_325.0);
        let mut fine = StaticPort::new(101_325.0);
        let total_s = 0.2;
        let coarse_steps = 4;
        let fine_steps = 200;
        let mut coarse_out = StaticPortOutput::default();
        for _ in 0..coarse_steps {
            coarse_out = coarse.step(95_000.0, 79_000.0, REFERENCE_AOA_DEG, 0.3, &faults, total_s / coarse_steps as f64);
        }
        let mut fine_out = StaticPortOutput::default();
        for _ in 0..fine_steps {
            fine_out = fine.step(95_000.0, 79_000.0, REFERENCE_AOA_DEG, 0.3, &faults, total_s / fine_steps as f64);
        }
        assert!(
            (coarse_out.sensed_static_pressure_pa - fine_out.sensed_static_pressure_pa).abs() < 1.0,
            "coarse {} fine {}",
            coarse_out.sensed_static_pressure_pa,
            fine_out.sensed_static_pressure_pa
        );
        assert!((coarse_out.sensed_static_pressure_pa - 101_325.0).abs() > 1.0);
        assert!((coarse_out.sensed_static_pressure_pa - 95_000.0).abs() > 1.0);
    }

    #[test]
    fn no_nan_at_zero_dt_or_full_blockage() {
        let mut port = StaticPort::new(0.0);
        let faults = StaticPortFaults { blocked: 1.0, leak_to_cabin: 1.0 };
        let out = port.step(0.0, 0.0, 0.0, 0.0, &faults, 0.0);
        assert!(out.sensed_static_pressure_pa.is_finite());
    }

    #[test]
    fn both_ports_healthy_average_fifty_fifty() {
        let left = StaticPortOutput { sensed_static_pressure_pa: 95_000.0, blocked: false };
        let right = StaticPortOutput { sensed_static_pressure_pa: 95_200.0, blocked: false };
        let out = average_pair(left, right, &StaticAveragingLineFaults::default());
        assert!((out.averaged_pa - 95_100.0).abs() < 1e-6, "{}", out.averaged_pa);
        assert!(!out.degraded);
    }

    #[test]
    fn one_side_blocked_falls_back_fully_to_the_healthy_side() {
        let left_blocked = StaticPortOutput { sensed_static_pressure_pa: 80_000.0, blocked: true };
        let right_healthy = StaticPortOutput { sensed_static_pressure_pa: 95_000.0, blocked: false };
        let out = average_pair(left_blocked, right_healthy, &StaticAveragingLineFaults::default());
        assert!((out.averaged_pa - 95_000.0).abs() < 1e-6, "{}", out.averaged_pa);
        assert!(out.degraded);
    }

    #[test]
    fn a_blocked_averaging_line_isolates_the_sides_without_invalidating_either_reading() {
        let left = StaticPortOutput { sensed_static_pressure_pa: 95_000.0, blocked: false };
        let right = StaticPortOutput { sensed_static_pressure_pa: 96_000.0, blocked: false };
        let faults = StaticAveragingLineFaults { line_blocked: 1.0 };
        let out = average_pair(left, right, &faults);
        assert!(out.degraded);
        assert!((out.averaged_pa - 95_000.0).abs() < 1e-6, "{}", out.averaged_pa);
    }

    #[test]
    fn no_nan_in_average_pair_defaults() {
        let out = average_pair(StaticPortOutput::default(), StaticPortOutput::default(), &StaticAveragingLineFaults::default());
        assert!(out.averaged_pa.is_finite());
    }
}
