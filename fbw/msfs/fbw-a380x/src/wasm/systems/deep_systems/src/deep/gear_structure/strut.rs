use super::{G_MS2, LegKind, MLW_KG, MTOW_KG};

const N_POLY: f64 = 1.25;
const Y_STATIC: f64 = 0.30;
const ALPHA_DAMPING: f64 = 1.8;

const SINK_SPEED_LIMIT_MLW_MS: f64 = 3.05;
const SINK_SPEED_LIMIT_MTOW_MS: f64 = 1.83;
const SINK_SPEED_RESERVE_ENERGY_MS: f64 = 12.0 * 0.3048;
const ULTIMATE_FACTOR: f64 = 1.5;
const SIDE_LOAD_FACTOR: f64 = 0.8;

const SUB_STEP_S: f64 = 0.001;

const CYCLE_EPS_M: f64 = 1e-9;

const BASE_LEAK_RATE_PER_S: f64 = 1.0 / (24.0 * 3600.0);
const SEAL_DAMAGE_PER_OVERLOAD_UNIT: f64 = 0.05;
const SEAL_DAMAGE_LEAK_COEFF: f64 = BASE_LEAK_RATE_PER_S * 2.0;

const FATIGUE_EXPONENT: f64 = 8.0;
const FATIGUE_REFERENCE_CYCLES: f64 = 20_000.0;

const UNLOCKED_COLLAPSE_FRACTION: f64 = 0.05;

const PRE_EXISTING_FRACTURE_MIN_STRENGTH_FRACTION: f64 = 0.03;

pub(super) fn nominal_compression_frac() -> f64 {
    Y_STATIC
}

impl LegKind {
    pub(super) fn stroke_m(self) -> f64 {
        match self {
            LegKind::Nose => 0.45,
            LegKind::Wing => 0.65,
            LegKind::Body => 0.70,
        }
    }

    pub(super) fn unsprung_kg(self) -> f64 {
        match self {
            LegKind::Nose => 300.0,
            LegKind::Wing => 1_200.0,
            LegKind::Body => 1_600.0,
        }
    }

    pub fn static_fraction(self) -> f64 {
        match self {
            LegKind::Nose => 2.0 / 22.0,
            LegKind::Wing => 4.0 / 22.0,
            LegKind::Body => 6.0 / 22.0,
        }
    }

    fn nominal_static_load_n(self) -> f64 {
        self.static_fraction() * MLW_KG * G_MS2
    }
}

fn gas_force_n(x_m: f64, f_ref_n: f64, gas_charge_fraction: f64, stroke_m: f64) -> f64 {
    let y = (x_m / stroke_m.max(1e-6)).clamp(0.0, 0.999_999);
    let base = ((1.0 - Y_STATIC) / (1.0 - y)).powf(N_POLY);
    gas_charge_fraction.clamp(0.0, 1.0) * f_ref_n.max(0.0) * base
}

fn equilibrium_y(f_ref_n: f64, w_load_n: f64, gas_charge_fraction: f64) -> f64 {
    let gcf = gas_charge_fraction.clamp(0.0, 1.0);
    let w = w_load_n.max(0.0);
    if gcf <= 0.0 || f_ref_n <= 0.0 {
        return 0.999_999;
    }
    if w <= 0.0 {
        return 0.0;
    }
    let ratio = (gcf * f_ref_n / w).powf(1.0 / N_POLY);
    (1.0 - (1.0 - Y_STATIC) * ratio).clamp(0.0, 0.999_999)
}

fn peak_force_for_drop(f_ref_n: f64, w_load_n: f64, sink_speed_ms: f64, k_damp: f64, stroke_m: f64, unsprung_kg: f64) -> f64 {
    let m_eff = (w_load_n.max(0.0) / G_MS2).max(unsprung_kg);
    let mut x = equilibrium_y(f_ref_n, w_load_n, 1.0) * stroke_m;
    let mut v = sink_speed_ms.max(0.0);
    let mut peak = 0.0_f64;
    for _ in 0..3_000 {
        let force = gas_force_n(x, f_ref_n, 1.0, stroke_m) + k_damp * v * v.abs();
        if force > peak {
            peak = force;
        }
        let a = G_MS2 - force / m_eff;
        v += a * SUB_STEP_S;
        x += v * SUB_STEP_S;
        if x < 0.0 {
            x = 0.0;
            if v < 0.0 {
                v = 0.0;
            }
        }
        let max_x = stroke_m * 0.999_999;
        if x > max_x {
            x = max_x;
        }
    }
    peak
}

#[derive(Clone, Copy, Debug, Default)]
pub struct StrutFaults {
    pub gas_leak: f64,
    pub oil_leak: f64,
    pub gas_charge_sensor_fail: f64,
    pub wow_sensing_fail: f64,
    pub pre_existing_fracture: f64,
}

const BITE_THRESHOLD: f64 = 0.5;
const SENSOR_LIE_THRESHOLD: f64 = 0.5;

#[derive(Clone, Copy, Debug)]
pub struct StrutInputs {
    pub on_ground: bool,
    pub sink_speed_ms: f64,
    pub load_n: f64,
    pub side_load_n: f64,
    pub locked_down: bool,
    pub dt_s: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct StrutOutputs {
    pub force_n: f64,
    pub compression_frac: f64,
    pub gas_charge_fraction: f64,
    pub oil_level_fraction: f64,
    pub collapsed: bool,
    pub overload_event: bool,
    pub cycle_completed: bool,
    pub peak_force_last_cycle_n: f64,
    pub life_fraction_consumed: f64,
    pub gas_charge_sensor_fault: bool,
    pub sensed_on_ground: bool,
}

pub struct Strut {
    kind: LegKind,
    f_ref_n: f64,
    stroke_m: f64,
    unsprung_kg: f64,
    k_damp: f64,
    pub limit_load_n: f64,
    pub ultimate_load_n: f64,
    lateral_limit_n: f64,

    x_m: f64,
    v_ms: f64,
    was_on_ground: bool,
    pub gas_charge_fraction: f64,
    pub oil_level_fraction: f64,
    seal_damage: f64,
    peak_force_this_cycle: f64,
    pub life_fraction_consumed: f64,
    pub collapsed: bool,
}

impl Strut {
    pub fn new(kind: LegKind) -> Self {
        let f_ref_n = kind.nominal_static_load_n();
        let stroke_m = kind.stroke_m();
        let unsprung_kg = kind.unsprung_kg();
        let k_damp = ALPHA_DAMPING * f_ref_n / (SINK_SPEED_LIMIT_MLW_MS * SINK_SPEED_LIMIT_MLW_MS);

        let peak_mlw = peak_force_for_drop(f_ref_n, f_ref_n, SINK_SPEED_LIMIT_MLW_MS, k_damp, stroke_m, unsprung_kg);
        let w_mtow = kind.static_fraction() * MTOW_KG * G_MS2;
        let peak_mtow = peak_force_for_drop(f_ref_n, w_mtow, SINK_SPEED_LIMIT_MTOW_MS, k_damp, stroke_m, unsprung_kg);
        let limit_load_n = peak_mlw.max(peak_mtow);

        let peak_reserve_energy = peak_force_for_drop(f_ref_n, f_ref_n, SINK_SPEED_RESERVE_ENERGY_MS, k_damp, stroke_m, unsprung_kg);
        let ultimate_load_n = (limit_load_n * ULTIMATE_FACTOR).max(peak_reserve_energy);
        let lateral_limit_n = limit_load_n * SIDE_LOAD_FACTOR;

        let x0 = equilibrium_y(f_ref_n, f_ref_n, 1.0) * stroke_m;

        Self {
            kind,
            f_ref_n,
            stroke_m,
            unsprung_kg,
            k_damp,
            limit_load_n,
            ultimate_load_n,
            lateral_limit_n,
            x_m: x0,
            v_ms: 0.0,
            was_on_ground: true,
            gas_charge_fraction: 1.0,
            oil_level_fraction: 1.0,
            seal_damage: 0.0,
            peak_force_this_cycle: 0.0,
            life_fraction_consumed: 0.0,
            collapsed: false,
        }
    }

    pub fn kind(&self) -> LegKind {
        self.kind
    }

    pub fn compression_frac_now(&self) -> f64 {
        self.x_m / self.stroke_m.max(1e-6)
    }

    pub fn current_force_n(&self) -> f64 {
        gas_force_n(self.x_m, self.f_ref_n, self.gas_charge_fraction, self.stroke_m) + self.oil_level_fraction.clamp(0.0, 1.0) * self.k_damp * self.v_ms * self.v_ms.abs()
    }

    fn close_cycle(&mut self) -> (bool, f64) {
        if self.peak_force_this_cycle > 0.0 {
            let ratio = self.peak_force_this_cycle / self.limit_load_n.max(1.0);
            self.life_fraction_consumed += ratio.powf(FATIGUE_EXPONENT) / FATIGUE_REFERENCE_CYCLES;
            let peak = self.peak_force_this_cycle;
            self.peak_force_this_cycle = 0.0;
            (true, peak)
        } else {
            (false, 0.0)
        }
    }

    pub fn step(&mut self, inputs: &StrutInputs, faults: &StrutFaults) -> StrutOutputs {
        let dt = inputs.dt_s.max(0.0);

        let gas_charge_sensor_fault = faults.gas_charge_sensor_fail >= BITE_THRESHOLD;
        let wow_lie = faults.wow_sensing_fail >= SENSOR_LIE_THRESHOLD;
        let sensed_on_ground = !self.collapsed && (inputs.on_ground != wow_lie);

        let gas_leak_rate = BASE_LEAK_RATE_PER_S * faults.gas_leak.clamp(0.0, 1.0) + SEAL_DAMAGE_LEAK_COEFF * self.seal_damage;
        self.gas_charge_fraction = (self.gas_charge_fraction - gas_leak_rate * dt).clamp(0.0, 1.0);
        let oil_leak_rate = BASE_LEAK_RATE_PER_S * faults.oil_leak.clamp(0.0, 1.0);
        self.oil_level_fraction = (self.oil_level_fraction - oil_leak_rate * dt).clamp(0.0, 1.0);

        if !inputs.on_ground {
            let was_grounded = self.was_on_ground;
            self.x_m = 0.0;
            self.v_ms = 0.0;
            self.was_on_ground = false;
            let (cycle_completed, peak_force_last_cycle_n) = if was_grounded { self.close_cycle() } else { (false, 0.0) };
            return StrutOutputs {
                force_n: 0.0,
                compression_frac: 0.0,
                gas_charge_fraction: self.gas_charge_fraction,
                oil_level_fraction: self.oil_level_fraction,
                collapsed: self.collapsed,
                overload_event: false,
                cycle_completed,
                peak_force_last_cycle_n,
                life_fraction_consumed: self.life_fraction_consumed,
                gas_charge_sensor_fault,
                sensed_on_ground,
            };
        }

        if !self.was_on_ground {
            self.x_m = equilibrium_y(self.f_ref_n, inputs.load_n.max(0.0), self.gas_charge_fraction) * self.stroke_m;
            self.v_ms = inputs.sink_speed_ms.max(0.0);
            self.peak_force_this_cycle = 0.0;
        }
        self.was_on_ground = true;

        let w = inputs.load_n.max(0.0);
        let m_eff = (w / G_MS2).max(self.unsprung_kg);

        let mut remaining = dt;
        let mut tick_peak_force =
            gas_force_n(self.x_m, self.f_ref_n, self.gas_charge_fraction, self.stroke_m) + self.oil_level_fraction.clamp(0.0, 1.0) * self.k_damp * self.v_ms * self.v_ms.abs();
        while remaining > 1e-9 {
            let h = SUB_STEP_S.min(remaining);
            let force = gas_force_n(self.x_m, self.f_ref_n, self.gas_charge_fraction, self.stroke_m) + self.oil_level_fraction.clamp(0.0, 1.0) * self.k_damp * self.v_ms * self.v_ms.abs();
            if force > tick_peak_force {
                tick_peak_force = force;
            }
            let a = G_MS2 - force / m_eff;
            self.v_ms += a * h;
            self.x_m += self.v_ms * h;
            if self.x_m < 0.0 {
                self.x_m = 0.0;
                if self.v_ms < 0.0 {
                    self.v_ms = 0.0;
                }
            }
            let max_x = self.stroke_m * 0.999_999;
            if self.x_m > max_x {
                self.x_m = max_x;
            }
            remaining -= h;
        }
        let force_n =
            gas_force_n(self.x_m, self.f_ref_n, self.gas_charge_fraction, self.stroke_m) + self.oil_level_fraction.clamp(0.0, 1.0) * self.k_damp * self.v_ms * self.v_ms.abs();
        self.peak_force_this_cycle = self.peak_force_this_cycle.max(tick_peak_force);

        let fracture_strength_fraction = 1.0 - faults.pre_existing_fracture.clamp(0.0, 1.0) * (1.0 - PRE_EXISTING_FRACTURE_MIN_STRENGTH_FRACTION);
        let effective_limit_load_n = self.limit_load_n * fracture_strength_fraction;
        let effective_ultimate_load_n = self.ultimate_load_n * fracture_strength_fraction;
        let effective_lateral_limit_n = self.lateral_limit_n * fracture_strength_fraction;

        let vertical_ratio = tick_peak_force / effective_limit_load_n.max(1.0);
        let side_ratio = inputs.side_load_n.abs() / effective_lateral_limit_n.max(1.0);
        let utilization = vertical_ratio.max(side_ratio);
        let vertical_ultimate_ratio = tick_peak_force / effective_ultimate_load_n.max(1.0);
        let side_ultimate_ratio = inputs.side_load_n.abs() / (effective_lateral_limit_n * ULTIMATE_FACTOR).max(1.0);
        let ultimate_ratio = vertical_ultimate_ratio.max(side_ultimate_ratio);

        let mut overload_event = false;
        if utilization >= 1.0 && ultimate_ratio < 1.0 {
            overload_event = true;
            self.seal_damage += (utilization - 1.0) * SEAL_DAMAGE_PER_OVERLOAD_UNIT;
        }
        if ultimate_ratio >= 1.0 {
            self.collapsed = true;
        }
        if !inputs.locked_down && tick_peak_force > UNLOCKED_COLLAPSE_FRACTION * self.limit_load_n {
            if !self.collapsed {
                crate::log(&format!(
                    "gear: leg folding - not downlocked while carrying {:.0} N ({:.0}% of the {:.0} N limit, threshold {:.0}%); on_ground={} sink={:.2} m/s",
                    tick_peak_force,
                    100.0 * tick_peak_force / self.limit_load_n.max(1.0),
                    self.limit_load_n,
                    100.0 * UNLOCKED_COLLAPSE_FRACTION,
                    inputs.on_ground,
                    inputs.sink_speed_ms
                ));
            }
            self.collapsed = true;
        }
        if ultimate_ratio >= 1.0 && !self.collapsed {
            crate::log(&format!("gear: leg folding - ultimate load exceeded, ratio {ultimate_ratio:.2}"));
        }

        let (cycle_completed, peak_force_last_cycle_n) = if self.x_m <= CYCLE_EPS_M { self.close_cycle() } else { (false, 0.0) };

        StrutOutputs {
            force_n,
            compression_frac: self.x_m / self.stroke_m.max(1e-6),
            gas_charge_fraction: self.gas_charge_fraction,
            oil_level_fraction: self.oil_level_fraction,
            collapsed: self.collapsed,
            overload_event,
            cycle_completed,
            peak_force_last_cycle_n,
            life_fraction_consumed: self.life_fraction_consumed,
            gas_charge_sensor_fault,
            sensed_on_ground,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn healthy() -> StrutFaults {
        StrutFaults::default()
    }

    fn land(s: &mut Strut, sink_speed_ms: f64, ticks: usize) -> (StrutOutputs, bool) {
        let faults = healthy();
        s.step(&StrutInputs { on_ground: false, sink_speed_ms: 0.0, load_n: 0.0, side_load_n: 0.0, locked_down: true, dt_s: 0.05 }, &faults);
        let mut out = StrutOutputs::default();
        let mut saw_overload = false;
        for tick in 0..ticks {
            let inputs = StrutInputs {
                on_ground: true,
                sink_speed_ms: if tick == 0 { sink_speed_ms } else { 0.0 },
                load_n: s.f_ref_n,
                side_load_n: 0.0,
                locked_down: true,
                dt_s: 0.001,
            };
            out = s.step(&inputs, &faults);
            saw_overload |= out.overload_event;
        }
        (out, saw_overload)
    }

    #[test]
    fn ultimate_is_the_larger_of_the_factored_limit_and_the_reserve_energy_case() {
        for kind in [LegKind::Nose, LegKind::Wing, LegKind::Body] {
            let s = Strut::new(kind);
            assert!(
                s.ultimate_load_n >= s.limit_load_n * ULTIMATE_FACTOR - 1e-6,
                "{kind:?}: ultimate must never fall below CS 25.303's 1.5 x limit"
            );
            let reserve = peak_force_for_drop(s.f_ref_n, s.f_ref_n, SINK_SPEED_RESERVE_ENERGY_MS, s.k_damp, s.stroke_m, s.unsprung_kg);
            assert!(
                s.ultimate_load_n >= reserve - 1e-6,
                "{kind:?}: ultimate must never fall below the CS 25.723(b) reserve-energy peak"
            );
            assert!((s.ultimate_load_n - (s.limit_load_n * ULTIMATE_FACTOR).max(reserve)).abs() < 1e-6);

            let reserve_over_limit = reserve / s.limit_load_n;
            assert!(
                reserve_over_limit < ULTIMATE_FACTOR,
                "{kind:?}: reserve/limit {reserve_over_limit} -- if this ever reaches 1.5 the \
                 reserve-energy case governs ultimate and the comment above needs redoing"
            );
            assert!(
                reserve_over_limit > 1.0,
                "{kind:?}: a 12 fps drop must be worse than the 10 fps limit drop"
            );
            assert!((s.ultimate_load_n / s.limit_load_n - ULTIMATE_FACTOR).abs() < 1e-9);
        }
    }

    #[test]
    fn the_reserve_energy_drop_does_not_fail_the_leg() {
        for kind in [LegKind::Nose, LegKind::Wing, LegKind::Body] {
            let mut s = Strut::new(kind);
            let (out, saw_overload) = land(&mut s, SINK_SPEED_RESERVE_ENERGY_MS, 3_000);
            assert!(!out.collapsed, "{kind:?}: the CS 25.723(b) reserve-energy drop must not fail the leg");
            assert!(saw_overload, "{kind:?}: a 12 fps drop is past limit load and must register as an overload");
        }
    }

    #[test]
    fn limit_load_scales_with_leg_wheel_share_and_is_a_few_times_static() {
        let wing = Strut::new(LegKind::Wing);
        let ratio = wing.limit_load_n / wing.f_ref_n;
        assert!(ratio > 1.3 && ratio < 6.0, "wing limit/static ratio {ratio}");

        let body = Strut::new(LegKind::Body);
        assert!(body.f_ref_n > wing.f_ref_n);
        assert!(body.limit_load_n > wing.limit_load_n);
    }

    #[test]
    fn resting_compression_matches_the_closed_form_equilibrium() {
        let s = Strut::new(LegKind::Nose);
        let expected_y = equilibrium_y(s.f_ref_n, s.f_ref_n, 1.0);
        assert!((s.x_m / s.stroke_m - expected_y).abs() < 1e-6);
        assert!((expected_y - Y_STATIC).abs() < 1e-9, "at the design static load this must be exactly Y_STATIC");
    }

    #[test]
    fn a_touchdown_at_exactly_the_limit_condition_does_not_collapse() {
        let mut s = Strut::new(LegKind::Wing);
        let (out, _) = land(&mut s, SINK_SPEED_LIMIT_MLW_MS, 3_000);
        assert!(!out.collapsed, "the certification limit condition must not itself collapse the leg");
        assert!(out.life_fraction_consumed >= 0.0);
    }

    #[test]
    fn a_touchdown_well_past_the_limit_sink_speed_collapses_the_leg() {
        let mut s = Strut::new(LegKind::Wing);
        let (out, _) = land(&mut s, SINK_SPEED_LIMIT_MLW_MS * 2.2, 3_000);
        assert!(out.collapsed, "a sink speed well past the limit condition must exceed ultimate and collapse the leg");
    }

    #[test]
    fn an_unlocked_leg_collapses_under_load_even_well_below_limit() {
        let mut s = Strut::new(LegKind::Nose);
        let faults = healthy();
        let inputs = StrutInputs { on_ground: true, sink_speed_ms: 0.0, load_n: s.f_ref_n, side_load_n: 0.0, locked_down: false, dt_s: 0.05 };
        let mut out = StrutOutputs::default();
        for _ in 0..40 {
            out = s.step(&inputs, &faults);
        }
        assert!(out.collapsed, "a real static load reacted through an unlocked leg must fold it");
    }

    #[test]
    fn a_gas_leak_sags_the_leg_to_a_higher_static_compression() {
        let mut s = Strut::new(LegKind::Body);
        let faults = StrutFaults { gas_leak: 1.0, ..StrutFaults::default() };
        let inputs = StrutInputs { on_ground: true, sink_speed_ms: 0.0, load_n: s.f_ref_n, side_load_n: 0.0, locked_down: true, dt_s: 3600.0 };
        let before = s.x_m / s.stroke_m;
        let out = s.step(&inputs, &faults);
        assert!(out.gas_charge_fraction < 1.0, "an hour at full leak magnitude must deplete some charge");
        let mut out2 = out;
        for _ in 0..2_000 {
            out2 = s.step(&StrutInputs { dt_s: 0.05, ..inputs }, &faults);
        }
        assert!(out2.compression_frac > before, "a depleted gas charge must sag the leg further for the same static load");
    }

    #[test]
    fn an_overload_event_leaves_lasting_seal_damage_that_accelerates_the_leak() {
        let mut s = Strut::new(LegKind::Wing);
        let (_, saw_overload) = land(&mut s, SINK_SPEED_LIMIT_MLW_MS * 1.35, 3_000);
        assert!(saw_overload, "a sink speed above the limit condition (but well under 2.2x) should overload without collapsing");
        assert!(!s.collapsed);
        assert!(s.seal_damage > 0.0, "an overload event must leave lasting seal damage");
    }

    #[test]
    fn fatigue_accumulates_more_from_a_harder_landing_than_a_gentle_one() {
        let mut gentle = Strut::new(LegKind::Wing);
        let mut hard = Strut::new(LegKind::Wing);
        let faults = healthy();
        land(&mut gentle, SINK_SPEED_LIMIT_MLW_MS * 0.3, 3_000);
        land(&mut hard, SINK_SPEED_LIMIT_MLW_MS * 0.95, 3_000);
        let air = StrutInputs { on_ground: false, sink_speed_ms: 0.0, load_n: 0.0, side_load_n: 0.0, locked_down: true, dt_s: 0.05 };
        gentle.step(&air, &faults);
        hard.step(&air, &faults);
        assert!(hard.life_fraction_consumed > gentle.life_fraction_consumed, "a harder landing must consume more fatigue life than a gentle one");
    }

    #[test]
    fn numerically_safe_at_rest_and_at_dt_zero() {
        let mut s = Strut::new(LegKind::Nose);
        let faults = healthy();
        let inputs = StrutInputs { on_ground: true, sink_speed_ms: 0.0, load_n: 0.0, side_load_n: 0.0, locked_down: true, dt_s: 0.0 };
        let out = s.step(&inputs, &faults);
        assert!(out.force_n.is_finite());
        assert!(out.compression_frac.is_finite());
        assert!(!out.force_n.is_nan());
    }

    #[test]
    fn gas_charge_sensor_and_wow_sensing_faults_are_independent_bite_flags() {
        let mut s = Strut::new(LegKind::Wing);
        let grounded = StrutInputs { on_ground: true, sink_speed_ms: 0.0, load_n: s.f_ref_n, side_load_n: 0.0, locked_down: true, dt_s: 1.0 };

        let healthy_out = s.step(&grounded, &healthy());
        assert!(!healthy_out.gas_charge_sensor_fault);
        assert!(healthy_out.sensed_on_ground, "on the ground, healthy sensing must report on_ground");

        let mut sensor_faulted = Strut::new(LegKind::Wing);
        let faults = StrutFaults { gas_charge_sensor_fail: 1.0, ..StrutFaults::default() };
        let out = sensor_faulted.step(&grounded, &faults);
        assert!(out.gas_charge_sensor_fault, "the armed pressure-monitoring BITE must report failed");
        assert!(out.sensed_on_ground, "and must not affect weight-on-wheels sensing, which is a separate channel");

        let mut wow_faulted = Strut::new(LegKind::Wing);
        let wow_faults = StrutFaults { wow_sensing_fail: 1.0, ..StrutFaults::default() };
        let wow_out = wow_faulted.step(&grounded, &wow_faults);
        assert!(!wow_out.gas_charge_sensor_fault, "and must not affect the pressure-monitoring channel");
        assert!(!wow_out.sensed_on_ground, "an armed WOW-sensing fault must invert the sensed ground-contact state");

        let airborne = StrutInputs { on_ground: false, ..grounded };
        let airborne_out = wow_faulted.step(&airborne, &wow_faults);
        assert!(airborne_out.sensed_on_ground, "airborne with the same fault armed, the sensed state must invert the other way");
    }
}

