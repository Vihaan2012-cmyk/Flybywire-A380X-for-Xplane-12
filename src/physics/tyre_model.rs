//! The per-wheel tyre physics of `tyre`, with nothing host-specific: the
//! X-Plane plugin drives it from its own bindings (`tyre::Tyres`) and the
//! MSFS systems module from FlyByWire's brake temperatures.

use crate::invariants;
use crate::physics::damage;
use crate::physics::gas;

// ---------------------------------------------------------------------------
// Cited constants.
// ---------------------------------------------------------------------------

/// Nitrogen's molar mass, kg/mol (N2, IUPAC standard atomic weight
/// 14.007 g/mol x 2).
pub const N2_MOLAR_MASS_KG_MOL: f64 = 0.0280134;
/// Nitrogen's specific gas constant, J/(kg*K): `R / M` (~296.8 J/(kg K)),
/// the same `R_universal` `gas.rs` already cites for oxygen.
pub const N2_SPECIFIC_GAS_CONSTANT: f64 = gas::R_UNIVERSAL / N2_MOLAR_MASS_KG_MOL;

/// Representative A380 main-gear cold (unheated, on-ground) tyre inflation
/// pressure. No page-specific Airbus figure was pinned down in this pass
/// (the "Aircraft Characteristics - Airport and Maintenance Planning"
/// document `damage.rs` already cites for MLW also tabulates tyre
/// pressures); this is the widely-published order-of-magnitude figure for
/// A380 main gear tyres (~15.5 bar / 225 psi) and is flagged generic rather
/// than cited to a specific page, matching this file's own convention for
/// uncited thresholds (see `damage.rs`'s `FUSE_PLUG_MELT_C`).
pub const COLD_PRESSURE_PA: f64 = 1_550_000.0;
/// ISA reference temperature the cold pressure above is quoted at (15 C),
/// the same reference `gas.rs`'s own tests use.
pub const COLD_TEMP_K: f64 = 288.15;

/// Heat-soak time constant, brake stack -> tyre bead/gas: slower than the
/// brake's own cooling (`damage.rs::BRAKE_COOLING_TAU_S` = 600 s) because
/// the path is conduction through the wheel hub into a comparatively large
/// gas+carcass thermal mass. Generic order-of-magnitude figure (no
/// published A380 value), flagged as such.
pub const SOAK_TAU_S: f64 = 900.0;
/// Ambient cooling time constant for the tyre carcass+gas (slower than the
/// brake's own `BRAKE_COOLING_TAU_S` for the same thermal-mass reason
/// above). Generic, flagged.
pub const COOL_TAU_S: f64 = 1200.0;
pub const SOAK_RATE_PER_S: f64 = 1.0 / SOAK_TAU_S;
pub const COOL_RATE_PER_S: f64 = 1.0 / COOL_TAU_S;

/// Rolling/flex heating coefficient (deg C per (m/s) of groundspeed per
/// second, at the tyre's rated cold pressure): an underinflated tyre
/// flexes more per revolution and dissipates more heat into its own
/// carcass, a standard tyre-engineering effect with no published A380
/// coefficient, so this is a generic derived figure sized only to be a
/// real (non-zero, few-degree-per-taxi) contributor, flagged as such
/// (docs/physics/failures.md).
pub const ROLL_HEAT_COEFF_C_PER_MS_PER_S: f64 = 0.02;

/// Continuous slow-leak rate, as a fraction of the wheel's own nitrogen
/// mass lost per second at `failures::magnitude() == 1.0` (the most severe
/// puncture this model represents without an instantaneous full burst).
/// Generic derived figure (docs/physics/failures.md): sized so a
/// magnitude-1.0 leak empties a wheel in a few thousand seconds (order of
/// an hour), consistent with a "slow leak" rather than an instant
/// deflation (which the fuse-plug/burst path already covers).
pub const LEAK_RATE_FRACTION_PER_S_AT_FULL_MAGNITUDE: f64 = 0.0005;

/// New-tyre tread depth to the wear-pin (wear indicator), a generic
/// transport-category figure (FAA AC 20-97B discusses tread-wear
/// indicators; no A380-specific depth was pinned down in this pass, so
/// this is explicitly generic, matching this file's convention).
pub const NEW_TREAD_DEPTH_MM: f64 = 6.0;
/// Abnormal tread-wear rate (mm/s) at `failures::magnitude() == 1.0`
/// (e.g. a misalignment/underinflation fault chewing tread far faster than
/// ordinary rolling wear). Generic derived figure, flagged.
pub const WEAR_RATE_MM_PER_S_AT_FULL_MAGNITUDE: f64 = 0.002;

/// Per-leg failure ids this module reads `failures::magnitude()` on and,
/// at fuse-plug melt, re-arms at full magnitude -- the same four ids and
/// ordering `damage.rs::LEG_WHEEL_INDICES` already uses (left wing, right
/// wing, left body, right body; the nose leg, `32_100`, has no brakes and
/// so no wheel entries here).
pub const LEG_FAILURE_IDS: [u64; 4] = [32_101, 32_102, 32_103, 32_104];

/// Wheel/brake position groupings duplicated from `damage.rs` (kept
/// separate rather than made `pub` there, since the two modules' update
/// order relative to each other is otherwise unconstrained -- this module
/// only reads `BRAKE_TEMPERATURE_n`, never `damage.rs`'s own state).
pub const LEG_WHEEL_INDICES: [[usize; 4]; 4] = [
    [0, 1, 4, 5],
    [2, 3, 6, 7],
    [8, 9, 12, 13],
    [10, 11, 14, 15],
];

/// How many wheels carry a braked tyre: indices `0..BRAKED_WHEELS` of
/// [`Tyres::wheels`], one per `BRAKE_TEMPERATURE_n`/`TYRE_PRESSURE_PA:n`.
pub const BRAKED_WHEELS: usize = 16;

/// Every tyre on the aircraft. An A380 stands on 22: two on the nose leg,
/// four on each wing leg, six on each body leg. Sixteen of those are
/// braked (all four wing-leg wheels a side, and the forward two axles of
/// each body leg); the nose pair and each body leg's steerable rear axle
/// are not.
pub const WHEELS: usize = 22;

/// Each wheel's position, in [`Tyres::wheels`]'/`TYRE_PRESSURE_PA:n`'s
/// own index order (`n` is the index + 1). Indices `0..16` are unchanged
/// from when this model carried only the braked wheels, so every existing
/// wheel map still points at the wheel it always did.
pub const WHEEL_NAMES: [&str; WHEELS] = [
    "L wing 1", "L wing 2", "R wing 1", "R wing 2", "L wing 3", "L wing 4", "R wing 3", "R wing 4", "L body 1", "L body 2",
    "R body 1", "R body 2", "L body 3", "L body 4", "R body 3", "R body 4", "Nose 1", "Nose 2", "L body 5", "L body 6",
    "R body 5", "R body 6",
];

/// The unbraked wheels, by the leg whose failure id they share: the nose
/// pair, then each body leg's rear axle. The nose leg has its own id
/// (`32_100`, matching X-Plane's own `rel_tire1`), which the braked
/// wheels' [`LEG_FAILURE_IDS`] does not carry.
pub const NOSE_FAILURE_ID: u64 = 32_100;
/// `(wheel index, failure id)` for every wheel past [`BRAKED_WHEELS`]: the
/// nose pair on the nose leg's own id, and each body leg's rear axle on
/// the same leg id its four braked wheels already use.
pub const UNBRAKED_WHEELS: [(usize, u64); WHEELS - BRAKED_WHEELS] = [
    (16, NOSE_FAILURE_ID),
    (17, NOSE_FAILURE_ID),
    (18, LEG_FAILURE_IDS[2]),
    (19, LEG_FAILURE_IDS[2]),
    (20, LEG_FAILURE_IDS[3]),
    (21, LEG_FAILURE_IDS[3]),
];

pub fn leg_of_wheel(wheel: usize) -> usize {
    LEG_WHEEL_INDICES.iter().position(|indices| indices.contains(&wheel)).expect("every wheel 0..16 is in exactly one leg")
}

/// One main-gear wheel's continuous tyre state.
#[derive(Clone, Copy, Debug)]
pub struct TyreWheel {
    /// Nitrogen temperature (C); starts at a plausible ambient/ramp value.
    pub temp_c: f64,
    /// Fraction of the wheel's original nitrogen mass lost to a leak or a
    /// fuse-plug melt, `0.0..=1.0` (1.0 = fully flat).
    pub leaked_fraction: f64,
    /// Remaining tread depth to the wear-pin (mm), `0.0..=NEW_TREAD_DEPTH_MM`.
    pub tread_mm: f64,
    /// Latched once this wheel's fuse plug has melted this session.
    pub fuse_plug_melted: bool,
}

impl Default for TyreWheel {
    fn default() -> Self {
        Self { temp_c: 15.0, leaked_fraction: 0.0, tread_mm: NEW_TREAD_DEPTH_MM, fuse_plug_melted: false }
    }
}

impl TyreWheel {
    /// This wheel's nitrogen pressure (Pa): the sealed-volume (Gay-Lussac)
    /// form of the ideal gas law, `P = P_cold * (T / T_cold)`, scaled by
    /// the nitrogen mass fraction remaining. Equivalent to
    /// `gas::ideal_gas_pressure_pa` at fixed volume with `m` replaced by
    /// `m_cold * (1 - leaked_fraction)`: both `V` and `m_cold` cancel out
    /// of the ratio, leaving only the temperature ratio and the leaked
    /// fraction, which is why no tyre-cavity volume figure is needed here.
    pub fn pressure_pa(&self) -> f64 {
        let temp_k = self.temp_c + 273.15;
        COLD_PRESSURE_PA * (temp_k / COLD_TEMP_K) * (1.0 - self.leaked_fraction)
    }

    /// Advance this wheel one tick. `brake_temp_c` is FlyByWire's own real
    /// `BRAKE_TEMPERATURE_n` for this wheel; `ambient_c` and
    /// `groundspeed_ms` are raw X-Plane state; `magnitude` is
    /// `failures::magnitude()` on this wheel's leg id (`0.0` = no active
    /// leak/wear fault, already clamped to `0.0..=1.0` by
    /// `failures::magnitude`'s own contract -- not re-clamped here);
    /// `delta` is real (unpaused) seconds this tick. Returns `true` the
    /// tick this wheel's fuse plug melts (edge, not level), so the caller
    /// can arm the leg's burst failure exactly once.
    ///
    /// No quantity here is clamped locally: every physically-bounded value
    /// this function produces is run through `invariants::check` (the same
    /// shared, logged accessor `Vars::write` uses in `lib.rs`), so a
    /// coupling bug that drives one of these out of range is caught and
    /// logged at the point it happens rather than silently absorbed.
    pub fn step(&mut self, brake_temp_c: f64, ambient_c: f64, groundspeed_ms: f64, magnitude: f64, delta: f64) -> bool {
        use crate::invariants::{self, Bound};

        // Leak: continuous nitrogen mass loss, magnitude-scaled. Left
        // unclamped through the addition itself -- a leak that runs past
        // "empty" is caught by the `Range(0.0, 1.0)` bound below (and, in
        // production, again when `TYRE_LEAKED_FRACTION:n` is published,
        // since that name also matches the `FRACTION` keyword).
        let leak_rate = LEAK_RATE_FRACTION_PER_S_AT_FULL_MAGNITUDE * magnitude;
        let leaked_raw = self.leaked_fraction + leak_rate * delta;
        self.leaked_fraction = invariants::check("TYRE_LEAKED_FRACTION", leaked_raw, Bound::Range(0.0, 1.0), "TyreWheel::step (leak)");

        // Temperature: heat soak from the brake, flex heating that grows
        // as inflation falls, cooling to ambient. All three terms are
        // linear in `temp_c`/constant per tick, so this integrates the
        // same closed-form linear ODE this file's own tests solve by hand.
        // `pressure_ratio` can be exactly 0.0 (fully flat, from the bound
        // above); `flex_heat`'s `1/pressure_ratio` then diverges to
        // infinity for that one tick, which `invariants::check` catches
        // (non-finite is always caught, regardless of bound) rather than a
        // hand-picked local floor pretending a flat tyre still holds some
        // residual pressure.
        let pressure_ratio = 1.0 - self.leaked_fraction;
        let flex_heat_raw = ROLL_HEAT_COEFF_C_PER_MS_PER_S * (1.0 / pressure_ratio) * groundspeed_ms.max(0.0);
        let flex_heat = invariants::check("TYRE_FLEX_HEAT_RATE_C_S", flex_heat_raw, Bound::NonNegative, "TyreWheel::step (flex heat)");
        let d_temp = SOAK_RATE_PER_S * (brake_temp_c - self.temp_c) + flex_heat - COOL_RATE_PER_S * (self.temp_c - ambient_c);
        let temp_raw = self.temp_c + d_temp * delta;
        self.temp_c = invariants::check("TYRE_TEMPERATURE_C", temp_raw, Bound::TemperatureFloor(-273.15), "TyreWheel::step (temp)");

        // Tread wear: magnitude-scaled abnormal wear.
        let wear_rate = WEAR_RATE_MM_PER_S_AT_FULL_MAGNITUDE * magnitude;
        let tread_raw = self.tread_mm - wear_rate * delta;
        self.tread_mm = invariants::check("TYRE_TREAD_MM", tread_raw, Bound::NonNegative, "TyreWheel::step (tread)");

        // Fuse plug: a real physical cause for full, immediate deflation,
        // independent of anything the leak model was already doing -- not
        // a scripted/bucketed effect, a genuine consequence of this same
        // continuous temperature crossing a physical melt point.
        if !self.fuse_plug_melted && self.temp_c > damage::FUSE_PLUG_MELT_C {
            self.fuse_plug_melted = true;
            self.leaked_fraction = 1.0;
            return true;
        }
        false
    }

    /// One tick for a wheel with no brake on it: the nose pair and each
    /// body leg's steerable rear axle.
    ///
    /// Identical physics, minus the one term that has nothing to couple
    /// to: there is no brake stack bolted inside these wheels, so there is
    /// no conduction path from one, and `k_soak * (T_brake - T)` is
    /// severed by handing the wheel its own temperature rather than by a
    /// flag. Everything else is real and still runs -- the nitrogen still
    /// obeys Gay-Lussac, a leak still deflates it, rolling flex still
    /// heats it (and heats it *more* as it goes soft), and it still cools
    /// to ambient.
    pub fn step_unbraked(&mut self, ambient_c: f64, groundspeed_ms: f64, magnitude: f64, delta: f64) -> bool {
        let own_temp = self.temp_c;
        self.step(own_temp, ambient_c, groundspeed_ms, magnitude, delta)
    }
}
