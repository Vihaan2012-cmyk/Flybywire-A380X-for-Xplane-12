//! Per-wheel main-gear tyre model (hyperrealism physics workstream 6:
//! failures, damage, MEL and persistence; continuation of `damage.rs`'s
//! tyre-burst work). `docs/physics/failures.md` has the wider threshold
//! table; sources for this file's own constants are cited alongside each
//! one below.
//!
//! Three continuous quantities per wheel, all driven by real inputs this
//! plugin already owns or FlyByWire already publishes:
//! - nitrogen pressure via the ideal gas law (constant-volume/Gay-Lussac
//!   form: a sealed tyre's own volume barely changes, so `P/T` at a fixed
//!   nitrogen mass is a constant -- this is the same law aviation tyre
//!   placards use for the standard "2% pressure rise per 5 C" rule of
//!   thumb), scaled down by whatever fraction of the original nitrogen
//!   mass a leak has removed;
//! - tyre temperature, a first-order heat-soak from FlyByWire's own real
//!   `BRAKE_TEMPERATURE_n` (`a380_systems/src/hydraulic/mod.rs:2074-2101`,
//!   the same array `damage.rs` reads) plus rolling/flex heating that grows
//!   as inflation falls (an underinflated tyre flexes more per revolution
//!   and runs hotter -- standard tyre-engineering fact, no A380-specific
//!   coefficient published so the *coefficient* is a generic derived
//!   figure, not the *mechanism*), less cooling to ambient;
//! - a fuse-plug melt at [`damage::FUSE_PLUG_MELT_C`], now driven by this
//!   module's own soaked tyre temperature rather than reading brake
//!   temperature directly (`damage.rs`'s existing
//!   `update_brake_temperature_tyre_burst` stays as the coarse brake-only
//!   backstop; this is the richer, continuous path).
//!
//! The leak rate and tread wear-pin consumption both come from
//! `failures::magnitude(id)` on the same per-leg id `damage.rs` already
//! arms at melt (`32_101`..`32_104`, left wing/right wing/left body/right
//! body -- there is no per-wheel id, matching X-Plane's own five
//! `rel_tireN` failure datarefs, one per leg incl. the nose). A fractional
//! magnitude below 1.0 is read as "this leg has an active slow leak/
//! puncture of this severity", continuous and non-binary
//! (`failures.rs`'s own doc comment on `set_magnitude`); reaching 1.0 (a
//! full burst) is this module's own doing once the physics crosses a real
//! threshold, not scripted.

use crate::failures;
use crate::physics::damage;
use crate::physics::gas;
use crate::xp::{DataRef, Xplm};
use systems::simulation::VariableIdentifier;

// ---------------------------------------------------------------------------
// Cited constants.
// ---------------------------------------------------------------------------

/// Nitrogen's molar mass, kg/mol (N2, IUPAC standard atomic weight
/// 14.007 g/mol x 2).
const N2_MOLAR_MASS_KG_MOL: f64 = 0.0280134;
/// Nitrogen's specific gas constant, J/(kg*K): `R / M` (~296.8 J/(kg K)),
/// the same `R_universal` `gas.rs` already cites for oxygen.
const N2_SPECIFIC_GAS_CONSTANT: f64 = gas::R_UNIVERSAL / N2_MOLAR_MASS_KG_MOL;

/// Representative A380 main-gear cold (unheated, on-ground) tyre inflation
/// pressure. No page-specific Airbus figure was pinned down in this pass
/// (the "Aircraft Characteristics - Airport and Maintenance Planning"
/// document `damage.rs` already cites for MLW also tabulates tyre
/// pressures); this is the widely-published order-of-magnitude figure for
/// A380 main gear tyres (~15.5 bar / 225 psi) and is flagged generic rather
/// than cited to a specific page, matching this file's own convention for
/// uncited thresholds (see `damage.rs`'s `FUSE_PLUG_MELT_C`).
const COLD_PRESSURE_PA: f64 = 1_550_000.0;
/// ISA reference temperature the cold pressure above is quoted at (15 C),
/// the same reference `gas.rs`'s own tests use.
const COLD_TEMP_K: f64 = 288.15;

/// Heat-soak time constant, brake stack -> tyre bead/gas: slower than the
/// brake's own cooling (`damage.rs::BRAKE_COOLING_TAU_S` = 600 s) because
/// the path is conduction through the wheel hub into a comparatively large
/// gas+carcass thermal mass. Generic order-of-magnitude figure (no
/// published A380 value), flagged as such.
const SOAK_TAU_S: f64 = 900.0;
/// Ambient cooling time constant for the tyre carcass+gas (slower than the
/// brake's own `BRAKE_COOLING_TAU_S` for the same thermal-mass reason
/// above). Generic, flagged.
const COOL_TAU_S: f64 = 1200.0;
const SOAK_RATE_PER_S: f64 = 1.0 / SOAK_TAU_S;
const COOL_RATE_PER_S: f64 = 1.0 / COOL_TAU_S;

/// Rolling/flex heating coefficient (deg C per (m/s) of groundspeed per
/// second, at the tyre's rated cold pressure): an underinflated tyre
/// flexes more per revolution and dissipates more heat into its own
/// carcass, a standard tyre-engineering effect with no published A380
/// coefficient, so this is a generic derived figure sized only to be a
/// real (non-zero, few-degree-per-taxi) contributor, flagged as such
/// (docs/physics/failures.md).
const ROLL_HEAT_COEFF_C_PER_MS_PER_S: f64 = 0.02;

/// Continuous slow-leak rate, as a fraction of the wheel's own nitrogen
/// mass lost per second at `failures::magnitude() == 1.0` (the most severe
/// puncture this model represents without an instantaneous full burst).
/// Generic derived figure (docs/physics/failures.md): sized so a
/// magnitude-1.0 leak empties a wheel in a few thousand seconds (order of
/// an hour), consistent with a "slow leak" rather than an instant
/// deflation (which the fuse-plug/burst path already covers).
const LEAK_RATE_FRACTION_PER_S_AT_FULL_MAGNITUDE: f64 = 0.0005;

/// New-tyre tread depth to the wear-pin (wear indicator), a generic
/// transport-category figure (FAA AC 20-97B discusses tread-wear
/// indicators; no A380-specific depth was pinned down in this pass, so
/// this is explicitly generic, matching this file's convention).
const NEW_TREAD_DEPTH_MM: f64 = 6.0;
/// Abnormal tread-wear rate (mm/s) at `failures::magnitude() == 1.0`
/// (e.g. a misalignment/underinflation fault chewing tread far faster than
/// ordinary rolling wear). Generic derived figure, flagged.
const WEAR_RATE_MM_PER_S_AT_FULL_MAGNITUDE: f64 = 0.002;

/// Per-leg failure ids this module reads `failures::magnitude()` on and,
/// at fuse-plug melt, re-arms at full magnitude -- the same four ids and
/// ordering `damage.rs::LEG_WHEEL_INDICES` already uses (left wing, right
/// wing, left body, right body; the nose leg, `32_100`, has no brakes and
/// so no wheel entries here).
const LEG_FAILURE_IDS: [u64; 4] = [32_101, 32_102, 32_103, 32_104];

/// Wheel/brake position groupings duplicated from `damage.rs` (kept
/// separate rather than made `pub` there, since the two modules' update
/// order relative to each other is otherwise unconstrained -- this module
/// only reads `BRAKE_TEMPERATURE_n`, never `damage.rs`'s own state).
const LEG_WHEEL_INDICES: [[usize; 4]; 4] = [
    [0, 1, 4, 5],
    [2, 3, 6, 7],
    [8, 9, 12, 13],
    [10, 11, 14, 15],
];

fn leg_of_wheel(wheel: usize) -> usize {
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
}

/// All 16 main-gear wheels (`LEG_WHEEL_INDICES`' own ordering, matching
/// `BRAKE_TEMPERATURE_1..16`), plus the X-Plane/FlyByWire bindings needed
/// to drive them and publish their state.
pub struct Tyres {
    pub wheels: [TyreWheel; 16],
    brake_temperature: [VariableIdentifier; 16],
    pressure_out: [VariableIdentifier; 16],
    temp_out: [VariableIdentifier; 16],
    leak_out: [VariableIdentifier; 16],
    tread_out: [VariableIdentifier; 16],
    fuse_plug_out: [VariableIdentifier; 16],
    ambient_c: Option<DataRef>,
    groundspeed_ms: Option<DataRef>,
    pub events: Vec<String>,
}

impl Tyres {
    pub fn new<W: systems::simulation::VariableRegistry>(vars: &mut W, xplm: Option<&Xplm>) -> Self {
        Self {
            wheels: [TyreWheel::default(); 16],
            brake_temperature: std::array::from_fn(|i| vars.get(format!("BRAKE_TEMPERATURE_{}", i + 1))),
            pressure_out: std::array::from_fn(|i| vars.get(format!("TYRE_PRESSURE_PA:{}", i + 1))),
            temp_out: std::array::from_fn(|i| vars.get(format!("TYRE_TEMPERATURE_C:{}", i + 1))),
            leak_out: std::array::from_fn(|i| vars.get(format!("TYRE_LEAKED_FRACTION:{}", i + 1))),
            tread_out: std::array::from_fn(|i| vars.get(format!("TYRE_TREAD_MM:{}", i + 1))),
            fuse_plug_out: std::array::from_fn(|i| vars.get(format!("TYRE_FUSE_PLUG_MELTED:{}", i + 1))),
            ambient_c: xplm.and_then(|x| x.find("sim/weather/aircraft/temperature_ambient_deg_c")),
            groundspeed_ms: xplm.and_then(|x| x.find("sim/flightmodel/position/groundspeed")),
            events: Vec::new(),
        }
    }

    fn get_f(xplm: Option<&Xplm>, d: Option<DataRef>) -> f64 {
        match (xplm, d) {
            (Some(x), Some(d)) => x.get_f(d) as f64,
            _ => 0.0,
        }
    }

    pub fn update<W: systems::simulation::SimulatorReaderWriter>(&mut self, vars: &mut W, xplm: Option<&Xplm>, delta: f64) {
        let ambient_c = Self::get_f(xplm, self.ambient_c);
        let groundspeed_ms = Self::get_f(xplm, self.groundspeed_ms);
        for i in 0..16 {
            let brake_temp_c = vars.read(&self.brake_temperature[i]);
            let leg = leg_of_wheel(i);
            let magnitude = failures::magnitude(LEG_FAILURE_IDS[leg]);
            let melted_now = self.wheels[i].step(brake_temp_c, ambient_c, groundspeed_ms, magnitude, delta);
            if melted_now {
                let names = ["left wing", "right wing", "left body", "right body"];
                self.events.push(format!(
                    "{} gear wheel {} fuse plug melted ({:.0} C tyre) -- tyre burst",
                    names[leg],
                    i + 1,
                    self.wheels[i].temp_c,
                ));
                failures::set_magnitude(LEG_FAILURE_IDS[leg], 1.0);
            }
            vars.write(&self.pressure_out[i], self.wheels[i].pressure_pa());
            vars.write(&self.temp_out[i], self.wheels[i].temp_c);
            vars.write(&self.leak_out[i], self.wheels[i].leaked_fraction);
            vars.write(&self.tread_out[i], self.wheels[i].tread_mm);
            vars.write(&self.fuse_plug_out[i], if self.wheels[i].fuse_plug_melted { 1.0 } else { 0.0 });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Gas law sanity: doubling absolute temperature at zero leak doubles
    /// pressure (Gay-Lussac, sealed constant volume) -- the textbook
    /// relation, checked against this module's own formula independently
    /// of any simulated stepping.
    #[test]
    fn pressure_scales_with_absolute_temperature_at_full_mass() {
        let mut w = TyreWheel { temp_c: COLD_TEMP_K - 273.15, leaked_fraction: 0.0, ..Default::default() };
        let p_cold = w.pressure_pa();
        assert!((p_cold - COLD_PRESSURE_PA).abs() < 1.0, "{p_cold}");
        w.temp_c = 2.0 * COLD_TEMP_K - 273.15; // doubled absolute temperature
        let p_hot = w.pressure_pa();
        assert!((p_hot / p_cold - 2.0).abs() < 1e-9, "{}", p_hot / p_cold);
    }

    /// A leak alone, cold and stationary, does not raise temperature (no
    /// heat source): only pressure falls, linearly with leaked fraction.
    #[test]
    fn leak_alone_lowers_pressure_without_heating_the_tyre() {
        let mut w = TyreWheel::default();
        w.temp_c = 15.0; // matches COLD_TEMP_K's 15 C reference
        for _ in 0..2000 {
            w.step(/* brake */ 15.0, /* ambient */ 15.0, /* groundspeed */ 0.0, /* magnitude */ 0.6, /* delta */ 1.0);
        }
        let expected_leaked = (LEAK_RATE_FRACTION_PER_S_AT_FULL_MAGNITUDE * 0.6 * 2000.0_f64).min(1.0);
        assert!((w.leaked_fraction - expected_leaked).abs() < 1e-9, "{} vs {}", w.leaked_fraction, expected_leaked);
        assert!((w.temp_c - 15.0).abs() < 0.05, "temp drifted: {}", w.temp_c);
        let expected_pressure = COLD_PRESSURE_PA * (1.0 - expected_leaked);
        assert!((w.pressure_pa() - expected_pressure).abs() / expected_pressure < 1e-6);
    }

    /// THE INTERSECTION TEST (external prediction, not the sim's own
    /// output): a single normal-energy landing, fully cooled before the
    /// next brake application, never reaches the fuse-plug melt point --
    /// but a slow leak (lowering inflation, and so raising flex heating)
    /// PLUS a short turnaround (residual brake heat not fully soaked away
    /// before the next taxi-out) together push the same wheel over
    /// `FUSE_PLUG_MELT_C`, which neither factor produces alone.
    ///
    /// Hand derivation (all closed-form, solved independently of the
    /// `step()` Euler loop under test):
    ///
    /// Phase A -- brake soak, stationary (no leak, no flex-heat term,
    /// `groundspeed = 0`): `dT/dt = k_soak*(T_brake - T) - k_cool*(T -
    /// T_amb)` is linear, `dT/dt = -(k_soak+k_cool)*(T - T_eq)` with
    /// `T_eq = (k_soak*T_brake + k_cool*T_amb) / (k_soak+k_cool)`.
    /// With `T_brake = 280 C` (a firm but unremarkable landing -- well
    /// under any brake-overtemp threshold), `T_amb = 20 C`,
    /// `k_soak = 1/900`, `k_cool = 1/1200`: `k_total = 7/3600 /s`,
    /// `T_eq = (280/900 + 20/1200) / (7/3600) = 0.327778/0.0019444
    /// = 168.6 C`. That equilibrium itself is under the 177 C melt point,
    /// so this single landing cannot melt the plug *however long* it
    /// soaks, stationary and leak-free -- the first half of "neither alone
    /// does it". Over `t = 900 s` (15 min, parked): `T(900) = 168.6 +
    /// (15-168.6)*exp(-900*7/3600/900)`... `exp(-7*900/3600/900)` --
    /// simplified, `exp(-k_total*t) = exp(-0.0019444*900) = exp(-1.75)
    /// = 0.17377`, so `T(900) = 168.6 - 153.6*0.17377 = 141.90 C`.
    /// Meanwhile the leak (magnitude 0.8, `LEAK_RATE = 0.0005/s`) removes
    /// `0.0005*0.8*900 = 0.36` of the nitrogen (`pressure_ratio = 0.64`).
    ///
    /// Phase B -- short turnaround taxi-out, `t = 250 s`: brake has
    /// dropped to taxi-idle `T_brake = 90 C`; groundspeed `15 m/s`. The
    /// leak (`magnitude = 0.8`) is still running through this phase too
    /// (same as Phase A), so `pressure_ratio` keeps falling and
    /// `flex_heat = ROLL_HEAT_COEFF*(1/pressure_ratio)*groundspeed` keeps
    /// rising through the 250 s window -- no longer the constant-coefficient
    /// linear ODE Phase A is (there is no elementary closed form for
    /// `1/(p0 - r*t)` convolved against `exp(-k*(t-s))`). The numbers below
    /// freeze `pressure_ratio` at its Phase-A-end value (`0.64`) as a
    /// *lower-bound* estimate -- real flex heat only grows from there -- which
    /// is enough for the ">"/"<" melt-point sanity checks the code below
    /// runs before trusting the sim, but not tight enough to assert an exact
    /// final temperature against (see the decoupled wheel below, where the
    /// test does exactly that and so integrates the real, non-frozen
    /// pressure ratio instead).
    /// Leaking wheel's flex heat at Phase B's start: `0.02*(1/0.64)*15 =
    /// 0.46875 C/s`; `T_eq_B = (90/900 + 0.46875 + 20/1200)/(7/3600)
    /// = 0.585417/0.0019444 = 301.07 C`. `T(250) = 301.07 +
    /// (141.90-301.07)*exp(-0.0019444*250) = 301.07 - 159.17*0.61510
    /// = 203.2 C` (lower bound) -- past the 177 C melt point already, so the
    /// real (hotter) trajectory crosses it too, and sooner.
    /// Control wheel (no leak, `pressure_ratio = 1.0` throughout, so this one
    /// *is* exact): flex heat `0.02*1*15 = 0.3 C/s`; `T_eq_B =
    /// (0.1+0.3+0.016667)/0.0019444 = 214.29 C`. `T(250) = 214.29 -
    /// 72.39*0.61510 = 169.8 C` -- stays under 177 C on the identical
    /// brake/taxi profile: the leak is necessary.
    ///
    /// Decoupling proof (the brake-heat-soak link cut, per the brief):
    /// the *same* leaking wheel, but every `step()` call is fed its own
    /// current temperature as `brake_temp_c`, so `k_soak*(T_brake-T) = 0`
    /// identically -- no scripted flag, the coupling term itself is
    /// simply never given anything to couple to. Phase A (no flex, no
    /// soak): `dT/dt = -k_cool*(T-20)`, `T(900) = 20 + (15-20)*
    /// exp(-900/1200) = 20 - 5*0.47237 = 17.64 C`. Phase B (soak still
    /// severed) is where the still-running leak's effect on flex heat
    /// actually matters for an exact number: with `pressure_ratio` frozen
    /// at Phase A's `0.64` the naive target is `20 + 0.46875*1200 =
    /// 582.5 C`, `T(250) = 582.5 + (17.64-582.5)*exp(-250/1200) = 582.5 -
    /// 564.86*0.81213 = 123.7 C`, but the leak keeps draining
    /// `pressure_ratio` by another `0.0005*0.8*250 = 0.1` over the same
    /// window (`0.64` -> `0.54`), so `flex_heat` keeps climbing (`0.46875`
    /// -> `0.3/0.54 = 0.5556 C/s`) instead of holding flat; folded through
    /// the same `exp(-k*(t-s))` weighting the closed form uses, that raises
    /// the true `T(250)` by roughly ten degrees over the frozen-ratio
    /// estimate above -- large next to the `1 C` integration tolerance this
    /// test holds `TyreWheel::step` to, so `predicted_decoupled_final`
    /// below is instead integrated one second at a time with the same
    /// falling `pressure_ratio`, independently of `TyreWheel::step` (same
    /// equations, no call into the module under test), to get a number
    /// accurate enough to assert against. It still stays under 177 C: the
    /// brake-soak coupling is necessary too, not just the leak.
    #[test]
    fn slow_leak_plus_short_turnaround_together_melt_a_fuse_plug_that_neither_does_alone() {
        const T_BRAKE_LANDING: f64 = 280.0;
        const T_AMB: f64 = 20.0;
        const T_BRAKE_TAXI: f64 = 90.0;
        const GROUNDSPEED_MS: f64 = 15.0;
        const MAGNITUDE: f64 = 0.8;
        const PHASE_A_S: f64 = 900.0;
        const PHASE_B_S: f64 = 250.0;
        const K_SOAK: f64 = SOAK_RATE_PER_S;
        const K_COOL: f64 = COOL_RATE_PER_S;
        const K_TOTAL: f64 = K_SOAK + K_COOL;

        fn relax(t0: f64, t_eq: f64, rate: f64, t: f64) -> f64 {
            t_eq + (t0 - t_eq) * (-rate * t).exp()
        }

        // Phase B, `1 s` Euler-integrated with the leak still running (see
        // the doc comment above): independent of `TyreWheel::step` (same
        // governing equations, no call into the module under test), but
        // tracking the same falling `pressure_ratio` that function does
        // rather than freezing it at its Phase-A-end value. `soak_coupled
        // = false` reproduces the decoupled wheel's severed brake-soak term
        // (fed its own temperature every tick, so `k_soak*(T_brake-T) = 0`
        // identically) without needing a `brake_temp` argument at all.
        fn integrate_phase_b(t0: f64, brake_temp: f64, initial_pressure_ratio: f64, soak_coupled: bool) -> f64 {
            let mut temp = t0;
            let mut pressure_ratio = initial_pressure_ratio;
            for _ in 0..(PHASE_B_S as u64) {
                // Leak first, same order as `TyreWheel::step`: this tick's
                // flex heat uses the fraction *after* this tick's leak.
                pressure_ratio -= LEAK_RATE_FRACTION_PER_S_AT_FULL_MAGNITUDE * MAGNITUDE; // dt = 1 s
                let flex = ROLL_HEAT_COEFF_C_PER_MS_PER_S * (1.0 / pressure_ratio) * GROUNDSPEED_MS;
                let soak = if soak_coupled { K_SOAK * (brake_temp - temp) } else { 0.0 };
                let d_temp = soak + flex - K_COOL * (temp - T_AMB);
                temp += d_temp; // dt = 1 s
            }
            temp
        }

        // --- Phase A: parked, brake soaking, leak accumulating. ---
        let t_eq_a = (K_SOAK * T_BRAKE_LANDING + K_COOL * T_AMB) / K_TOTAL;
        assert!(t_eq_a < damage::FUSE_PLUG_MELT_C, "derivation error: a single landing's own equilibrium must stay under melt: {t_eq_a}");
        let predicted_after_a = relax(15.0, t_eq_a, K_TOTAL, PHASE_A_S);

        let mut leaking = TyreWheel { temp_c: 15.0, ..Default::default() };
        for _ in 0..(PHASE_A_S as u64) {
            leaking.step(T_BRAKE_LANDING, T_AMB, 0.0, MAGNITUDE, 1.0);
        }
        assert!((leaking.temp_c - predicted_after_a).abs() < 0.5, "{} vs {}", leaking.temp_c, predicted_after_a);
        assert!(leaking.temp_c < damage::FUSE_PLUG_MELT_C, "phase A alone must not melt the plug: {}", leaking.temp_c);
        let leaked_after_a = LEAK_RATE_FRACTION_PER_S_AT_FULL_MAGNITUDE * MAGNITUDE * PHASE_A_S;
        assert!((leaking.leaked_fraction - leaked_after_a).abs() < 1e-9);

        let mut control = TyreWheel { temp_c: 15.0, ..Default::default() };
        for _ in 0..(PHASE_A_S as u64) {
            control.step(T_BRAKE_LANDING, T_AMB, 0.0, 0.0, 1.0);
        }
        assert!((control.temp_c - predicted_after_a).abs() < 0.5);
        assert_eq!(control.leaked_fraction, 0.0);

        // Decoupled wheel: same leak as `leaking`, but the brake-soak link
        // is severed for the whole scenario (see derivation above).
        let mut decoupled = TyreWheel { temp_c: 15.0, ..Default::default() };
        for _ in 0..(PHASE_A_S as u64) {
            let own_temp = decoupled.temp_c;
            decoupled.step(own_temp, T_AMB, 0.0, MAGNITUDE, 1.0);
        }
        let predicted_decoupled_after_a = relax(15.0, T_AMB, K_COOL, PHASE_A_S);
        assert!((decoupled.temp_c - predicted_decoupled_after_a).abs() < 0.5, "{} vs {}", decoupled.temp_c, predicted_decoupled_after_a);

        // --- Phase B: short turnaround taxi-out. ---
        // Frozen-ratio lower bound, only used for the ">" sanity check
        // below (real leaking+coupled runs hotter than this, see doc
        // comment above); not asserted against the sim's exact value.
        let leaking_pressure_ratio = 1.0 - leaked_after_a;
        let leaking_flex = ROLL_HEAT_COEFF_C_PER_MS_PER_S * (1.0 / leaking_pressure_ratio) * GROUNDSPEED_MS;
        let t_eq_b_leaking = (K_SOAK * T_BRAKE_TAXI + leaking_flex + K_COOL * T_AMB) / K_TOTAL;
        let predicted_leaking_final = relax(predicted_after_a, t_eq_b_leaking, K_TOTAL, PHASE_B_S);

        let control_flex = ROLL_HEAT_COEFF_C_PER_MS_PER_S * 1.0 * GROUNDSPEED_MS;
        let t_eq_b_control = (K_SOAK * T_BRAKE_TAXI + control_flex + K_COOL * T_AMB) / K_TOTAL;
        let predicted_control_final = relax(predicted_after_a, t_eq_b_control, K_TOTAL, PHASE_B_S);

        // Exact (not frozen-ratio) prediction: this one IS asserted against
        // the sim's final temperature below, so it has to track the leak's
        // continued effect on pressure_ratio through Phase B.
        let predicted_decoupled_final =
            integrate_phase_b(predicted_decoupled_after_a, 0.0, leaking_pressure_ratio, false);

        // Sanity on the hand derivation itself before trusting the sim.
        assert!(predicted_leaking_final > damage::FUSE_PLUG_MELT_C, "derivation error: leaking+coupled must cross melt: {predicted_leaking_final}");
        assert!(predicted_control_final < damage::FUSE_PLUG_MELT_C, "derivation error: no-leak control must stay under melt: {predicted_control_final}");
        assert!(predicted_decoupled_final < damage::FUSE_PLUG_MELT_C, "derivation error: leak-without-soak-coupling must stay under melt: {predicted_decoupled_final}");

        let mut leaking_melted = false;
        for _ in 0..(PHASE_B_S as u64) {
            if leaking.step(T_BRAKE_TAXI, T_AMB, GROUNDSPEED_MS, MAGNITUDE, 1.0) {
                leaking_melted = true;
            }
        }
        let mut control_melted = false;
        for _ in 0..(PHASE_B_S as u64) {
            if control.step(T_BRAKE_TAXI, T_AMB, GROUNDSPEED_MS, 0.0, 1.0) {
                control_melted = true;
            }
        }
        let mut decoupled_melted = false;
        for _ in 0..(PHASE_B_S as u64) {
            let own_temp = decoupled.temp_c;
            if decoupled.step(own_temp, T_AMB, GROUNDSPEED_MS, MAGNITUDE, 1.0) {
                decoupled_melted = true;
            }
        }

        // The external prediction: leak + short turnaround, WITH the
        // brake-heat-soak coupling intact, together melt the plug (and
        // fully deflate/burst that wheel). Cut either ingredient --  the
        // leak (control) or the soak coupling itself (decoupled) -- on the
        // identical brake/taxi profile, and it does not.
        assert!(leaking_melted, "leaking+coupled wheel should have melted its fuse plug and burst");
        assert!(leaking.fuse_plug_melted);
        assert_eq!(leaking.leaked_fraction, 1.0, "a melted fuse plug fully deflates the wheel");
        assert!(!control_melted, "no-leak control must not melt under the same brake/taxi profile");
        assert!(!control.fuse_plug_melted);
        assert!(!decoupled_melted, "leak without the brake-soak coupling must not melt: the coupling itself is a necessary ingredient, not just the leak");
        assert!(!decoupled.fuse_plug_melted);

        // Sim trajectories match the closed-form predictions within
        // integration tolerance (1 s Euler steps against an analytic
        // solution of the same linear ODE).
        assert!((control.temp_c - predicted_control_final).abs() < 1.0, "{} vs {}", control.temp_c, predicted_control_final);
        assert!((decoupled.temp_c - predicted_decoupled_final).abs() < 1.0, "{} vs {}", decoupled.temp_c, predicted_decoupled_final);
    }

    #[test]
    fn tread_wears_only_when_a_wear_fault_is_active() {
        let mut w = TyreWheel::default();
        w.step(15.0, 15.0, 0.0, 0.0, 1000.0);
        assert_eq!(w.tread_mm, NEW_TREAD_DEPTH_MM, "no magnitude -> no abnormal wear");
        w.step(15.0, 15.0, 0.0, 1.0, 500.0);
        let expected = NEW_TREAD_DEPTH_MM - WEAR_RATE_MM_PER_S_AT_FULL_MAGNITUDE * 500.0;
        assert!((w.tread_mm - expected).abs() < 1e-9);
    }

    #[test]
    fn leg_wheel_mapping_covers_all_sixteen_wheels_exactly_once() {
        let mut seen = [false; 16];
        for leg in 0..4 {
            for &w in &LEG_WHEEL_INDICES[leg] {
                assert!(!seen[w], "wheel {w} claimed by two legs");
                seen[w] = true;
                assert_eq!(leg_of_wheel(w), leg);
            }
        }
        assert!(seen.iter().all(|&s| s));
    }
}
