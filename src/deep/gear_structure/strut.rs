//! The oleo-pneumatic shock strut: a nitrogen gas spring (polytropic
//! compression) in series with hydraulic orifice damping, one instance per
//! landing gear leg (nose, 2 x wing, 2 x body).
//!
//! # Gas spring
//! An oleo strut's air chamber compresses as the piston strokes; treating
//! the gas volume as shrinking linearly with stroke and applying the
//! standard polytropic relation `P*V^n = const` (the same family as
//! `physics::gas`'s ideal-gas law, generalised from the isothermal/adiabatic
//! `n=1`/`n=1.4` cases to the intermediate exponent real gas springs show
//! under a landing impact's timescale) gives, with `y = x/stroke` the
//! fractional compression:
//!
//!   F_air(y) = gas_charge_fraction * F_ref * ((1 - y_static)/(1 - y))^n
//!
//! `F_ref` is defined as this leg's own design static reaction (its share of
//! MLW), so the law is self-normalising: at `gas_charge_fraction = 1` and
//! `y = y_static` it reproduces `F_ref` exactly by construction, and it
//! diverges as `y -> 1` (bottoming = metal-to-metal contact), which is the
//! real, physical hard stop -- no separate contact-stiffness special case is
//! needed. `n` (`N_POLY`) and `y_static` (`Y_STATIC`) are GENERIC, textbook
//! oleo-strut design figures (e.g. Currey, *Aircraft Landing Gear Design*,
//! AIAA): polytropic exponent in the 1.1-1.35 range for the gas-spring
//! compression timescale of a landing impact, static compression typically
//! around a third of total stroke for good ride/stroke-utilisation balance.
//! No A380-specific strut data (areas, precharge pressure, stroke) is
//! public, so those are GENERIC per-leg figures, order-of-magnitude sized
//! from the aircraft's own published weights.
//!
//! # Damping
//! Hydraulic fluid forced through a fixed orifice gives a drag quadratic in
//! velocity (`dP = 0.5*rho*(Q/(Cd*A))^2` standard orifice flow, `F = dP*A`),
//! the same functional form used throughout this crate's other quadratic
//! flow-damping paths. The coefficient `k_damp` is GENERIC, sized (not
//! calibrated by search/bisection, just algebra) so the damping force at the
//! A380 special-condition limit sink speed is a fixed, documented multiple
//! (`ALPHA_DAMPING`) of the static reaction -- see `Strut::new`.
//!
//! # Limit and ultimate loads
//! The descent velocities here are **the A380's own**, not the generic
//! CS-25 minima: the FAA's special conditions for this aircraft's five-leg
//! gear state them explicitly. "Special Conditions: Airbus Model A380-800
//! Airplane, Loading Conditions for Multi-Leg Landing Gear", Docket No.
//! NM341, Federal Register 28 March 2006 (document 06-2973), special
//! condition A.2 "Symmetric landing load conditions", requires the gear and
//! airframe to be designed
//!   - "with a limit descent velocity of 3.05 m/sec (10 fps) at the design
//!     landing weight (the maximum weight for landing conditions at maximum
//!     descent velocity)", and
//!   - "with a limit descent velocity of 1.83 m/sec (6 fps) at the design
//!     takeoff weight (the maximum weight for landing conditions at a
//!     reduced descent velocity)".
//!
//! Both of those are *limit* load conditions. An earlier revision of this
//! file labelled the 6 fps / MTOW case the "reserve energy" case; that was
//! a mis-citation. The reserve-energy condition is a **third**, separate
//! requirement that governs the gear alone: 14 CFR / CS 25.723(b), "the
//! landing gear may not fail in a test, demonstrating its reserve energy
//! absorption capacity, simulating a descent velocity of 12 f.p.s. at
//! design landing weight, assuming airplane lift not greater than airplane
//! weight acting during the landing impact". 12 fps is 3.66 m/s, and that
//! no-lift assumption is exactly this module's own simplification (below),
//! so the reserve-energy drop is reproduced here literally. Because it is a
//! *must-not-fail* requirement, the leg's ultimate capability is the larger
//! of the 1.5-factored limit load and the reserve-energy peak -- see
//! `Strut::new`, which takes that maximum rather than assuming which case
//! wins.
//!
//! This module reproduces the certification method directly: it runs this
//! same strut model (healthy, at rest, `x=0`) from each required touchdown
//! condition and reads off the peak reaction force, rather than asserting a
//! load factor from elsewhere -- the limit load is whichever of the two
//! limit cases is worse, matching how the real drop test envelope is built.
//! The 1.5 factor is CS 25.303's factor of safety ("unless otherwise
//! specified... the factor of safety is 1.5").
//! Side loads are checked against `0.8` times the vertical limit (CS
//! 25.485's limit side load factor).
//!
//! The per-leg **load share** stays GENERIC (wheel-count proportional, see
//! `LegKind::static_fraction`): the same special conditions say only that
//! "load sharing between landing gear must be determined in a rational
//! manner considering the flexibility of the airplane" -- they mandate a
//! method, not a number, and Airbus's own five-leg distribution is not
//! public.
//!
//! Aircraft weights: MLW 386,000 kg / MTOW 510,000 kg, Airbus "A380 Aircraft
//! Characteristics - Airport and Maintenance Planning" (WV000 variant),
//! matching `physics/damage.rs`'s own `MLW_KG` citation.
//!
//! Simplification (documented, not hidden): the drop-test analogy here
//! assumes the full aircraft weight share is "falling" onto the leg (CS
//! 25.473(b) allows crediting some concurrent wing lift, which this
//! self-contained module has no speed/AoA history to compute) -- so the
//! loads this module derives run mildly conservative (higher) relative to
//! the certified figures, which is the safe direction for a damage model.
//!
//! # Servicing, overload, fatigue, collapse
//! `gas_charge_fraction`/`oil_level_fraction` (nitrogen precharge / hydraulic
//! oil level, `1.0` = correctly serviced) deplete under `StrutFaults`' leak
//! fractions; a low gas charge sags the leg onto a higher static compression
//! for the same load (see `equilibrium_y`), eating into the stroke margin
//! before the next landing. Any tick whose peak force exceeds the limit load
//! (but not ultimate) is an overload event: it does not break the leg
//! outright, but it damages the seals, which feeds back into a faster gas
//! leak from then on (`seal_damage`) -- a genuine emergent consequence, not
//! a scripted symptom. Fatigue is accumulated by Miner's rule once per
//! ground-contact cycle (touchdown to the next full extension), from that
//! cycle's own peak load ratio to the limit load, with a GENERIC Basquin-type
//! exponent (`FATIGUE_EXPONENT`, textbook mid-range for a metal S-N curve)
//! and a GENERIC reference cycle count. The leg collapses (permanently) if
//! ultimate is exceeded, or if a real load is reacted while the downlock is
//! not engaged (`locked_down = false`): without the lock's geometry taking
//! the axial reaction, the leg has essentially no load path and folds under
//! any real fraction of its rated load.

/// Standard gravity, m/s^2 (exact SI definition), shared with `super`.
use super::{G_MS2, LegKind, MLW_KG, MTOW_KG};

/// Polytropic exponent for the nitrogen gas spring (GENERIC, textbook
/// oleo-strut range 1.1-1.35; module doc has the citation).
const N_POLY: f64 = 1.25;
/// Static compression as a fraction of total stroke at the leg's own design
/// static load (GENERIC, typical oleo-strut design practice).
const Y_STATIC: f64 = 0.30;
/// Damping force at the A380 special-condition limit sink speed (3.05 m/s
/// at MLW), as a multiple of the
/// leg's static reaction (GENERIC dimensionless sizing factor; the
/// resulting dynamic/static peak ratio is checked by this file's own test
/// against the widely cited 2-3x "reaction load factor" order of magnitude
/// for transport-category main gear).
const ALPHA_DAMPING: f64 = 1.8;

/// A380-800 special conditions (Docket NM341, FR 28 Mar 2006, cond. A.2):
/// "a limit descent velocity of 3.05 m/sec (10 fps) at the design landing
/// weight". The A380's own figure, quoted in metres per second by the
/// special condition itself -- not the generic CS-25.473(a)(1) minimum it
/// happens to coincide with.
const SINK_SPEED_LIMIT_MLW_MS: f64 = 3.05;
/// Same special condition A.2: "a limit descent velocity of 1.83 m/sec
/// (6 fps) at the design takeoff weight". A *limit* condition, not the
/// reserve-energy one (see module doc).
const SINK_SPEED_LIMIT_MTOW_MS: f64 = 1.83;
/// 14 CFR / CS 25.723(b) reserve energy absorption: "a descent velocity of
/// 12 f.p.s. at design landing weight, assuming airplane lift not greater
/// than airplane weight acting during the landing impact". 12 ft/s x
/// 0.3048 m/ft = 3.6576 m/s. The gear may not *fail* here, so this case
/// sets a floor under the leg's ultimate capability.
const SINK_SPEED_RESERVE_ENERGY_MS: f64 = 12.0 * 0.3048;
/// CS 25.303: factor of safety of 1.5 on limit loads to obtain ultimate,
/// unless otherwise specified.
const ULTIMATE_FACTOR: f64 = 1.5;
/// CS 25.485: limit side load factor, as a fraction of the vertical limit
/// load, acting at the gear.
const SIDE_LOAD_FACTOR: f64 = 0.8;

/// Integration sub-step, s: the gas spring stiffens without bound near
/// bottoming and the damping is quadratic in velocity, both stiff close to
/// touchdown, so `step` always sub-steps at this resolution regardless of
/// the caller's own tick (brief: "sub-stepping where stiff").
const SUB_STEP_S: f64 = 0.001;

/// Below this compression the leg is considered fully extended again (a
/// genuine liftoff, not just numerical residue) for Miner's-rule cycle
/// counting.
const CYCLE_EPS_M: f64 = 1e-9;

/// Servicing leak rate at fault magnitude 1.0: full loss over 24 h (GENERIC
/// -- a "severe" seal leak scenario, not a slow multi-week weep, sized to
/// matter within an operationally meaningful window for the Study/MEL use
/// of this model).
const BASE_LEAK_RATE_PER_S: f64 = 1.0 / (24.0 * 3600.0);
/// Wear-units added to `seal_damage` per unit of overload (utilisation - 1.0)
/// on a single event (GENERIC).
const SEAL_DAMAGE_PER_OVERLOAD_UNIT: f64 = 0.05;
/// Extra gas leak rate per unit of accumulated seal damage (GENERIC: one
/// full "wear unit" of accumulated overload damage doubles the base leak
/// rate).
const SEAL_DAMAGE_LEAK_COEFF: f64 = BASE_LEAK_RATE_PER_S * 2.0;

/// GENERIC Basquin-type S-N exponent for a landing-gear-grade high-strength
/// steel forging (textbook mid-range, e.g. Shigley's *Mechanical
/// Engineering Design* typical exponents of 1/b ~ 7-12 for such alloys).
const FATIGUE_EXPONENT: f64 = 8.0;
/// GENERIC reference cycle count: the number of touchdowns *at exactly the
/// limit load* that would consume the design fatigue budget once over.
/// Loosely informed by publicly discussed large-transport design-service
/// landing counts (tens of thousands); not cited to Airbus.
const FATIGUE_REFERENCE_CYCLES: f64 = 20_000.0;

/// Fraction of the limit load that reacting *any* real force through an
/// unlocked leg is assumed able to survive before it folds (GENERIC: an
/// unlocked leg has essentially no intended load path).
const UNLOCKED_COLLAPSE_FRACTION: f64 = 0.05;

/// The design static compression fraction (`Y_STATIC`), exposed to
/// `structure.rs` so it can turn a body leg's *current* compression into an
/// "extra sag beyond nominal" for the tailstrike geometry coupling, without
/// making the constant itself part of this module's public API.
pub(super) fn nominal_compression_frac() -> f64 {
    Y_STATIC
}

impl LegKind {
    /// Full compression travel, m (GENERIC, order-of-magnitude widebody
    /// strut stroke).
    pub(super) fn stroke_m(self) -> f64 {
        match self {
            LegKind::Nose => 0.45,
            LegKind::Wing => 0.65,
            LegKind::Body => 0.70,
        }
    }

    /// Unsprung unsprung mass floor for the drop-test effective mass, kg
    /// (GENERIC, order-of-magnitude leg/axle/bogie mass; also the safety
    /// floor that keeps `m_eff` from vanishing if a caller ever passes a
    /// near-zero instantaneous load).
    pub(super) fn unsprung_kg(self) -> f64 {
        match self {
            LegKind::Nose => 300.0,
            LegKind::Wing => 1_200.0,
            LegKind::Body => 1_600.0,
        }
    }

    /// This leg's share of total aircraft weight (GENERIC: proportional to
    /// its own wheel count out of the 22 total -- landing gear wheel count
    /// is chosen by designers to keep per-wheel/pavement loading sane for
    /// the load that leg carries, so wheel count is a reasonable, documented
    /// proxy for load share in the absence of a published figure).
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

/// Polytropic gas-spring reaction at compression `x_m` (see module doc for
/// the derivation). `f_ref_n` is the force this law gives at
/// `x = y_static*stroke_m` when `gas_charge_fraction == 1.0`.
fn gas_force_n(x_m: f64, f_ref_n: f64, gas_charge_fraction: f64, stroke_m: f64) -> f64 {
    let y = (x_m / stroke_m.max(1e-6)).clamp(0.0, 0.999_999);
    let base = ((1.0 - Y_STATIC) / (1.0 - y)).powf(N_POLY);
    gas_charge_fraction.clamp(0.0, 1.0) * f_ref_n.max(0.0) * base
}

/// Inverse of `gas_force_n`: the compression fraction at which the gas
/// spring alone reacts exactly `w_load_n`, used both for a leg's resting
/// position and by this file's own equilibrium test.
fn equilibrium_y(f_ref_n: f64, w_load_n: f64, gas_charge_fraction: f64) -> f64 {
    let gcf = gas_charge_fraction.clamp(0.0, 1.0);
    let w = w_load_n.max(0.0);
    if gcf <= 0.0 || f_ref_n <= 0.0 {
        return 0.999_999; // no gas spring left to support anything: bottomed.
    }
    if w <= 0.0 {
        return 0.0; // nothing to support: fully extended.
    }
    let ratio = (gcf * f_ref_n / w).powf(1.0 / N_POLY);
    (1.0 - (1.0 - Y_STATIC) * ratio).clamp(0.0, 0.999_999)
}

/// Runs this same strut law from a standing start (`x=0`) at `sink_speed_ms`
/// under a healthy, fully serviced strut, and returns the peak reaction
/// force reached -- the certification drop test, reproduced directly rather
/// than assumed from an external load-factor figure.
fn peak_force_for_drop(f_ref_n: f64, w_load_n: f64, sink_speed_ms: f64, k_damp: f64, stroke_m: f64, unsprung_kg: f64) -> f64 {
    let m_eff = (w_load_n.max(0.0) / G_MS2).max(unsprung_kg);
    let mut x = 0.0_f64;
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

/// Faults this strut carries, each a fraction 0 (healthy) .. 1 (fully
/// failed).
#[derive(Clone, Copy, Debug, Default)]
pub struct StrutFaults {
    /// Nitrogen precharge leak rate (seal or schrader valve).
    pub gas_leak: f64,
    /// Hydraulic oil leak rate (damping fluid).
    pub oil_leak: f64,
    /// `E-IND-DESIGN.md` 320800043 L/G OLEO PRESS MONITORING FAULT: the
    /// strut's own pressure-*sensing* function has failed, independent of
    /// the real `gas_charge_fraction` above -- a BITE-reported boolean
    /// (>= `BITE_THRESHOLD`), mirroring `retraction::RetractionFaults`'s
    /// own boolean-above-threshold convention.
    pub gas_charge_sensor_fail: f64,
    /// `E-IND-DESIGN.md` 320800046 L/G WEIGHT ON WHEELS FAULT: this leg's
    /// weight-on-wheels sensing disagrees with its true ground-contact
    /// state above `SENSOR_LIE_THRESHOLD`, the identical pattern
    /// `retraction::RetractionFaults::sensor_lies` already uses for
    /// lock-position sensing, applied to ground-contact sensing instead.
    pub wow_sensing_fail: f64,
}

/// A BITE-reported boolean fault (a self-test either passes or fails) is
/// considered failed at or above this fraction -- this crate's own
/// established convention (`retraction::SENSOR_LIE_THRESHOLD`,
/// `gear_structure::live::ANTISKID_BITE_THRESHOLD`), reused rather than a
/// second GENERIC number invented for the same idea.
const BITE_THRESHOLD: f64 = 0.5;
/// As `retraction::SENSOR_LIE_THRESHOLD`: above this severity the *sensed*
/// state reads the opposite of the true one.
const SENSOR_LIE_THRESHOLD: f64 = 0.5;

/// One tick's inputs.
#[derive(Clone, Copy, Debug)]
pub struct StrutInputs {
    /// Whether this leg's wheels are touching the ground this tick.
    pub on_ground: bool,
    /// Vertical closing speed at the instant of touchdown, m/s (only read on
    /// the tick `on_ground` first becomes true; ignored otherwise).
    pub sink_speed_ms: f64,
    /// This leg's current share of aircraft weight, N (varies with CG/pitch/
    /// braking weight transfer; only meaningful while `on_ground`).
    pub load_n: f64,
    /// Lateral load applied at the axle this tick, N (crosswind/drift
    /// landing, turning loads).
    pub side_load_n: f64,
    /// Whether the downlock is engaged (from `retraction::Retraction`).
    pub locked_down: bool,
    pub dt_s: f64,
}

/// One tick's outputs.
#[derive(Clone, Copy, Debug, Default)]
pub struct StrutOutputs {
    /// Instantaneous vertical reaction force at the end of this tick, N.
    pub force_n: f64,
    pub compression_frac: f64,
    pub gas_charge_fraction: f64,
    pub oil_level_fraction: f64,
    pub collapsed: bool,
    /// A limit-load exceedance happened this tick (not ultimate: the leg
    /// survives, but its seals just took overload damage).
    pub overload_event: bool,
    /// A full ground-contact cycle (touchdown to liftoff) just completed and
    /// contributed its Miner's-rule increment.
    pub cycle_completed: bool,
    pub peak_force_last_cycle_n: f64,
    pub life_fraction_consumed: f64,
    /// `E-IND-DESIGN.md` 320800043: the strut's own pressure-monitoring BITE
    /// has failed (independent of the real `gas_charge_fraction`).
    pub gas_charge_sensor_fault: bool,
    /// `E-IND-DESIGN.md` 320800046: what weight-on-wheels sensing *reports*
    /// for this leg, which `wow_sensing_fail` can detach from the true
    /// ground-contact state (`inputs.on_ground`) above `SENSOR_LIE_
    /// THRESHOLD` -- the same true/sensed split `retraction::
    /// RetractionOutputs::sensed_downlocked` already carries for lock state.
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

        // The two A380 special-condition A.2 limit drop cases; the worse one
        // governs (matches how a real certification envelope is built).
        let peak_mlw = peak_force_for_drop(f_ref_n, f_ref_n, SINK_SPEED_LIMIT_MLW_MS, k_damp, stroke_m, unsprung_kg);
        let w_mtow = kind.static_fraction() * MTOW_KG * G_MS2;
        let peak_mtow = peak_force_for_drop(f_ref_n, w_mtow, SINK_SPEED_LIMIT_MTOW_MS, k_damp, stroke_m, unsprung_kg);
        let limit_load_n = peak_mlw.max(peak_mtow);

        // CS 25.723(b) reserve energy: 12 fps at design landing weight, no
        // lift credit -- which is this model's own standing assumption, so
        // the same drop routine reproduces the required test directly. The
        // gear may not fail here, so ultimate is the larger of the
        // 1.5-factored limit load and this peak. Whether the factor or the
        // reserve-energy case governs is left to the arithmetic rather than
        // assumed: at ALPHA_DAMPING = 1.8 the drop is damping-dominated at
        // touchdown, so the peak goes roughly as v^2 and (12/10)^2 = 1.44
        // would beat 1.5 only if the (v-independent) gas-spring share of the
        // peak were small enough -- it is not, so 1.5x limit currently wins.
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

    /// This leg's compression fraction right now (not just the last tick's
    /// `StrutOutputs`), for callers building a report between ticks.
    pub fn compression_frac_now(&self) -> f64 {
        self.x_m / self.stroke_m.max(1e-6)
    }

    /// This leg's instantaneous reaction force right now, recomputed from
    /// its current state (see `compression_frac_now`).
    pub fn current_force_n(&self) -> f64 {
        gas_force_n(self.x_m, self.f_ref_n, self.gas_charge_fraction, self.stroke_m) + self.oil_level_fraction.clamp(0.0, 1.0) * self.k_damp * self.v_ms * self.v_ms.abs()
    }

    /// Closes out the current ground-contact cycle (if one is in progress)
    /// and applies its Miner's-rule fatigue increment from that cycle's own
    /// peak load ratio. Called both when the leg bounces back to full
    /// extension while still nominally "on ground", and when it genuinely
    /// lifts off -- either way, the cycle that started at the last touchdown
    /// is now over and its peak is already known.
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

        // Two independent BITE-style side channels, computed once up front
        // since both apply identically whether or not the leg is on the
        // ground this tick (module doc on each field).
        let gas_charge_sensor_fault = faults.gas_charge_sensor_fail >= BITE_THRESHOLD;
        let wow_lie = faults.wow_sensing_fail >= SENSOR_LIE_THRESHOLD;
        let sensed_on_ground = inputs.on_ground != wow_lie;

        // Servicing: gas/oil depletion from active leak faults, plus this
        // strut's own accumulated seal damage from past overload events --
        // an emergent, not scripted, escalation.
        let gas_leak_rate = BASE_LEAK_RATE_PER_S * faults.gas_leak.clamp(0.0, 1.0) + SEAL_DAMAGE_LEAK_COEFF * self.seal_damage;
        self.gas_charge_fraction = (self.gas_charge_fraction - gas_leak_rate * dt).clamp(0.0, 1.0);
        let oil_leak_rate = BASE_LEAK_RATE_PER_S * faults.oil_leak.clamp(0.0, 1.0);
        self.oil_level_fraction = (self.oil_level_fraction - oil_leak_rate * dt).clamp(0.0, 1.0);

        if !inputs.on_ground {
            // Airborne: nothing holds the strut anywhere but fully extended
            // against its own mechanical stop. If a ground-contact cycle was
            // in progress, liftoff is what ends it -- a leg that settles at
            // its taxi equilibrium and never mechanically returns to x=0
            // must still get fatigue credit for the touchdown that put it
            // there once it actually leaves the ground.
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
            // Touchdown edge: initial closing velocity is the sink rate.
            self.x_m = 0.0;
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

        // Combined vertical + side (CS-25.485) utilisation: whichever load
        // path is closer to its own limit governs overload/collapse.
        let vertical_ratio = tick_peak_force / self.limit_load_n.max(1.0);
        let side_ratio = inputs.side_load_n.abs() / self.lateral_limit_n.max(1.0);
        let utilization = vertical_ratio.max(side_ratio);
        let vertical_ultimate_ratio = tick_peak_force / self.ultimate_load_n.max(1.0);
        let side_ultimate_ratio = inputs.side_load_n.abs() / (self.lateral_limit_n * ULTIMATE_FACTOR).max(1.0);
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

        // Miner's rule fatigue: a bounce back to full extension while still
        // notionally on the ground also closes out the cycle (see
        // `close_cycle`'s doc comment).
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

    /// Flies a freshly built strut, then lands it at `sink_speed_ms` and holds
    /// it on the ground for `ticks` ticks of 1 ms. Returns the last tick's
    /// outputs and whether any tick of the touchdown raised an overload event.
    ///
    /// `Strut::new` builds the leg *parked*: `x_m` sits at the static
    /// equilibrium and `was_on_ground` is already true, which is the correct
    /// state for an aircraft standing on its gear at the gate.
    /// `sink_speed_ms` is only read on the tick `on_ground` goes false ->
    /// true (see `StrutInputs`), so a test that wants a genuine touchdown
    /// transient must put the leg in the air first -- otherwise the drop is
    /// silently ignored and the leg simply sits at its static reaction, well
    /// under any limit.
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
        // `Strut::new` takes `max(1.5 * limit, reserve-energy peak)`. Both
        // branches are real requirements, so the test checks the max
        // relation itself and then which branch governs, rather than
        // hard-coding one of them.
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

            // Which branch wins, and why. At touchdown the drop is
            // damping-dominated (v = v_sink, x = 0), so the peak is
            //   F = F_gas(0) + k*v^2,   k = ALPHA * F_ref / v_limit^2.
            // With F_gas(0) = F_ref*(1-Y_STATIC)^N_POLY = F_ref*0.7^1.25
            // = 0.6403*F_ref and ALPHA_DAMPING = 1.8:
            //   F(10 fps) = (0.6403 + 1.8) * F_ref             = 2.440 * F_ref
            //   F(12 fps) = (0.6403 + 1.8*(3.6576/3.05)^2)*F_ref
            //             = (0.6403 + 1.8*1.4378) * F_ref      = 3.228 * F_ref
            // so reserve/limit = 3.228/2.440 = 1.323 < 1.5. The gas spring's
            // v-independent share is what keeps the ratio below (12/10)^2 =
            // 1.44 and hence below the 1.5 factor: the factored limit load
            // governs, by a margin of about 13%.
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
        // CS 25.723(b): the landing gear "may not fail" in the 12 fps
        // reserve-energy test at design landing weight. This is the
        // requirement, run as a real touchdown rather than as arithmetic on
        // `ultimate_load_n`.
        for kind in [LegKind::Nose, LegKind::Wing, LegKind::Body] {
            let mut s = Strut::new(kind);
            let (out, saw_overload) = land(&mut s, SINK_SPEED_RESERVE_ENERGY_MS, 3_000);
            assert!(!out.collapsed, "{kind:?}: the CS 25.723(b) reserve-energy drop must not fail the leg");
            // It is past limit load, though: 12 fps is a reserve-energy
            // condition, not a limit one, so the seals take overload damage.
            assert!(saw_overload, "{kind:?}: a 12 fps drop is past limit load and must register as an overload");
        }
    }

    #[test]
    fn limit_load_scales_with_leg_wheel_share_and_is_a_few_times_static() {
        // A widely cited order of magnitude for transport main-gear drop
        // tests is a peak reaction of roughly 2-4x the static load; this is
        // a sanity check on the derivation, not a tight calibration target.
        let wing = Strut::new(LegKind::Wing);
        let ratio = wing.limit_load_n / wing.f_ref_n;
        assert!(ratio > 1.3 && ratio < 6.0, "wing limit/static ratio {ratio}");

        let body = Strut::new(LegKind::Body);
        // Body legs carry a larger fraction of MLW, so a larger absolute
        // static and limit load than the wing legs.
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
        // A drop's absorbed energy goes as v^2, so the sink speed that reaches
        // ultimate (1.5x limit) is roughly sqrt(1.5) ~ 1.22x the limit sink
        // speed for a constant-force absorber, and somewhat more once the gas
        // spring's rising polytropic force curve is included; 2.2x the limit
        // sink speed is ~4.8x the energy and is unambiguously past ultimate.
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
        // Give it time to re-settle at the new (lower) equilibrium.
        let mut out2 = out;
        for _ in 0..2_000 {
            out2 = s.step(&StrutInputs { dt_s: 0.05, ..inputs }, &faults);
        }
        assert!(out2.compression_frac > before, "a depleted gas charge must sag the leg further for the same static load");
    }

    #[test]
    fn an_overload_event_leaves_lasting_seal_damage_that_accelerates_the_leak() {
        let mut s = Strut::new(LegKind::Wing);
        // A sink speed comfortably between the limit (no overload) and 2.2x
        // limit (collapse) cases above: an overload without a collapse. 1.35x
        // the limit sink speed is ~1.8x the kinetic energy of the limit drop,
        // enough to pass limit but short of the 1.5x factor of safety.
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
        // Liftoff: return both to airborne so their in-progress cycle closes
        // out and contributes its Miner's-rule increment.
        let air = StrutInputs { on_ground: false, sink_speed_ms: 0.0, load_n: 0.0, side_load_n: 0.0, locked_down: true, dt_s: 0.05 };
        gentle.step(&air, &faults);
        hard.step(&air, &faults);
        // One more ground contact to force the cycle-completion edge from
        // the *next* touchdown's x==0 check would also work, but re-entering
        // flight already zeroes compression; step once more airborne is
        // enough since the cycle closes the instant x reaches 0 while still
        // "on_ground" in the loops above -- assert the airborne state itself
        // still reflects the accumulated difference either way.
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

    /// `E-IND-DESIGN.md` 320800043 L/G OLEO PRESS MONITORING FAULT and
    /// 320800046 L/G WEIGHT ON WHEELS FAULT: both are independent BITE-
    /// style side channels, unrelated to each other and to the strut's own
    /// real `gas_leak`/`oil_leak` faults.
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

