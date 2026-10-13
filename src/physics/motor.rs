//! Shared DC/AC motor-current model (hyperrealism physics, breaker-coupling
//! workstream): the one place every "extra load/fault raises motor current"
//! coupling computes its multiplier from, so `bearing_overcurrent_multiplier`
//! (src/breakers.rs), the fuel feed-pump cavitation coupling
//! (src/fuel_network.rs / src/fuel.rs), and any future coupling (fan
//! obstruction, actuator binding, winding insulation breakdown) share one
//! physically derived relation instead of each inventing its own fudge
//! factor -- the user's principle: "you don't implement trips, you implement
//! draws," with the breaker's own I^2t/magnetic curve
//! (`physics::electrical::trip_step`) doing the tripping from a current this
//! module computed from a physical cause.
//!
//! ## Mechanical-load coupling (bearing wear, cavitation, fan/actuator
//! binding, ...): back-EMF derivation
//!
//! A DC (or field-oriented AC) motor at steady state: `I = (V - Ke*omega) /
//! R` (Ohm's law across the armature, less the back-EMF the spinning rotor
//! generates -- e.g. Sen, *Principles of Electric Machines and Power
//! Electronics*, ch. 5; torque `tau = Kt*I` is the same relation's other
//! half and is not needed here because this module works directly in
//! current). Extra mechanical load torque (a dragging bearing, an uneven
//! cavitating pump load, ice-jammed fan blades, a binding actuator) does not
//! change `V` or `R`; a torque-limited motor instead settles at a lower
//! steady-state speed `omega`, which *lowers* the back-EMF term and so
//! *raises* `I` for the same `V`/`R` -- current rises continuously as the
//! extra load fraction rises, with no authored bucket.
//!
//! Model `omega` dropping linearly with the load fraction `w` (0 = no extra
//! load, 1 = the load fully stalls the rotor): `omega(w) = omega_rated *
//! (1 - w)`. At `w = 1`, `omega = 0`: back-EMF collapses to zero and `I`
//! reaches the classic **locked-rotor current**, `I_LR = V / R` -- 5-7x
//! rated running current for a typical aerospace/industrial induction or
//! PMSM pump motor (NEMA MG-1 design B/E locked-rotor current tables;
//! general motor-protection literature, e.g. IEEE Std 242 ch. 9 motor
//! protection). Solving the back-EMF relation for `I(w)` given only the two
//! datapoints every consumer already has -- rated current and the locked-
//! rotor multiple -- collapses to a single linear expression in `w`
//! ([`mechanical_current_multiplier`]); see that function's own derivation
//! comment for the algebra.
//!
//! ## Winding insulation breakdown: partial inter-turn short
//!
//! A winding's insulation does not fail as a single dead short; enamel/
//! varnish breakdown between adjacent turns first creates a partial
//! inter-turn short, which shorts out a *fraction* of the winding's turns
//! and so lowers the winding's own effective impedance -- the source
//! voltage is unchanged, so (Ohm's law again) current rises as impedance
//! falls. [`insulation_current_multiplier`] models effective impedance as
//! falling linearly from the healthy value at `health = 1.0` to a small but
//! non-zero residual at `health = 0.0` (a real winding fault is never a
//! literal zero-ohm short -- unshorted turns and contact resistance remain
//! in the current path; [`INSULATION_SHORT_RESIDUAL_IMPEDANCE_FRACTION`] is
//! the documented residual, typical/derived, no A380 TR/motor-specific
//! winding-fault datasheet is public).
//!
//! ## Decouple check (every consumer's own test)
//! Each coupling test asserts the current predicted by this module against
//! an independent hand computation from the cited relation, *and* a
//! decouple check: set the physical cause back to its healthy value and
//! confirm the current (and any trip it caused) reverts -- proving the rise
//! is a real function of the physical input, not an authored constant that
//! happens to look right at one operating point.

use std::collections::HashMap;
use std::sync::Mutex;

/// Typical locked-rotor current as a multiple of rated running current for
/// an aerospace/industrial induction or PMSM pump motor: 5-7x is the
/// standard cited range (NEMA MG-1 design B/E characteristics; motor-
/// protection literature). 6.0 is the range's midpoint -- a defensible
/// single figure absent a published A380-specific pump-motor curve, same
/// "derived/typical" convention this crate already uses elsewhere (see
/// `physics::electrical`'s own `MAGNETIC_TRIP_MULTIPLE`/`THERMAL_TRIP_K`
/// doc comments) for a quantity no public exact source gives.
pub const TYPICAL_LOCKED_ROTOR_MULTIPLE: f64 = 6.0;

/// Current multiplier (>= 1.0) from an extra mechanical load fraction `w`
/// (0.0 = none, 1.0 = fully stalls the rotor), from the back-EMF derivation
/// in this module's own doc comment.
///
/// Derivation: let `m` = [`TYPICAL_LOCKED_ROTOR_MULTIPLE`] (`I_LR /
/// I_rated`), `V` the supply, `R` the armature resistance, `Ke` the back-EMF
/// constant, `omega_rated` the rated speed. At locked rotor (`omega = 0`):
/// `I_LR = V/R`, so `R = V/I_LR`. At rated load (`w = 0`, `omega =
/// omega_rated`): `I_rated = (V - Ke*omega_rated)/R`, so `Ke*omega_rated =
/// V*(1 - I_rated/I_LR) = V*(1 - 1/m)`. With the linear speed-droop model
/// `omega(w) = omega_rated*(1-w)`:
/// ```text
/// I(w) = (V - Ke*omega_rated*(1-w)) / R
///      = I_LR * (1 - (1 - 1/m)*(1-w))
///      = 1 + (m - 1)*w     [as a multiple of I_rated; algebra in the module doc]
/// ```
/// Check: `w=0` gives `1.0` (rated current, no derate); `w=1` gives `m`
/// (exactly the locked-rotor multiple). Monotonic and continuous in `w` --
/// no bucket.
pub fn mechanical_current_multiplier(load_fraction: f64) -> f64 {
    mechanical_current_multiplier_with_multiple(load_fraction, TYPICAL_LOCKED_ROTOR_MULTIPLE)
}

/// [`mechanical_current_multiplier`] with an explicit locked-rotor multiple,
/// for a consumer that cites its own motor-specific figure instead of the
/// generic [`TYPICAL_LOCKED_ROTOR_MULTIPLE`].
pub fn mechanical_current_multiplier_with_multiple(load_fraction: f64, locked_rotor_multiple: f64) -> f64 {
    let w = load_fraction.clamp(0.0, 1.0);
    1.0 + (locked_rotor_multiple - 1.0) * w
}

/// A partial inter-turn winding short's residual effective impedance
/// fraction at `health = 0.0` (fully broken down): a real fault never
/// reaches a literal zero-ohm short (unshorted turns and contact resistance
/// remain), so this floors the effective impedance instead of letting
/// current diverge to infinity. Typical/derived (no A380 TR/motor winding-
/// fault datasheet is public) -- chosen so the resulting current lands in
/// the thermal-trip region (a few times rated) rather than an instant
/// magnetic trip, matching real inter-turn-short behaviour (it usually
/// escalates over seconds, not instantly, until it cascades to a full
/// short).
pub const INSULATION_SHORT_RESIDUAL_IMPEDANCE_FRACTION: f64 = 0.15;

/// Current multiplier (>= 1.0) from winding insulation health (1.0 =
/// healthy, 0.0 = fully broken down), from the partial-inter-turn-short
/// derivation in this module's own doc comment: effective impedance `Z(h) =
/// Z_rated * (h + (1-h)*residual)`, current (fixed `V`) is `I = V/Z`, so the
/// multiplier relative to the healthy current is `1 / (h + (1-h)*residual)`.
/// Check: `h=1` gives `1.0` (no derate); `h=0` gives `1/residual` (~6.7x at
/// the documented residual).
pub fn insulation_current_multiplier(health: f64) -> f64 {
    let h = health.clamp(0.0, 1.0);
    let z = h + (1.0 - h) * INSULATION_SHORT_RESIDUAL_IMPEDANCE_FRACTION;
    1.0 / z
}

/// ## Cavitating centrifugal pump: hydraulic-power coupling (not a
/// mechanical-load coupling)
///
/// A cavitating centrifugal boost pump does **not** behave like a dragging
/// bearing or a binding actuator. Vapor forming at the impeller eye starves
/// the impeller of liquid, so the pump moves *less* fluid at *lower* head --
/// its hydraulic output power collapses, it does not rise. Modelling that as
/// an "extra load fraction" fed into [`mechanical_current_multiplier`] (as
/// an earlier pass of this coupling did) is backwards: it made a cavitating
/// pump's current climb toward [`TYPICAL_LOCKED_ROTOR_MULTIPLE`], the
/// opposite of the real physics.
///
/// The correct coupling is the standard centrifugal-pump shaft-power
/// relation `P_shaft = rho * g * Q * H / eta` (equivalently `P_shaft =
/// delta_p * Q / eta` in pressure terms -- Karassik, Messina, Cooper &
/// Heald, *Pump Handbook*, 4th ed., ch. 2, "Centrifugal Pump Theory"): less
/// flow `Q` (and, once cavitation is severe enough to collapse the pump
/// curve, less head `H`/`delta_p`) means less hydraulic power delivered,
/// which a motor driven at roughly constant speed answers with *less*
/// current, not more -- the same current-tracks-mechanical-power
/// relationship [`mechanical_current_multiplier`]'s own back-EMF derivation
/// uses, just with the load moving the other direction. Current cannot fall
/// to zero, though: an unloaded (or fully vapor-locked) induction/PMSM motor
/// still draws its own magnetizing/windage no-load current.
///
/// `hydraulic_power_current_multiplier` is that relation; [`fuel.rs`]'s
/// cavitation coupling publishes the pump's real delivered-power fraction
/// through [`publish_hydraulic_power_fraction`]/[`hydraulic_power_fraction`]
/// (a registry separate from [`LOAD_FRACTIONS`] -- the two couplings have
/// opposite sign and must never be added together under one key).
/// Typical induction-motor no-load (magnetizing + windage/friction) current
/// as a fraction of full-load rated current: a motor spinning with no
/// mechanical load still draws real current for core magnetization and
/// bearing/windage friction. 20-40% of full-load current is the standard
/// textbook range for a general-purpose induction motor at no load (e.g.
/// Fitzgerald, Kingsley & Umans, *Electric Machinery*, 6th ed., ch. 6, the
/// no-load test); 0.3 is the range's midpoint -- derived/typical, the same
/// convention [`TYPICAL_LOCKED_ROTOR_MULTIPLE`] already documents, since no
/// A380-specific fuel-boost-pump-motor figure is public.
pub const TYPICAL_NO_LOAD_CURRENT_FRACTION: f64 = 0.3;

/// Current multiplier (as a fraction of rated current) from the hydraulic
/// power a centrifugal pump is *actually delivering* right now, as a
/// fraction of its rated hydraulic power (`hydraulic_power_fraction`, `P /
/// P_rated`, both `P = delta_p * Q / eta` -- see this module's own "cavitating
/// centrifugal pump" doc section for the citation). Linear between the
/// no-load floor at zero delivered power and rated current at rated power,
/// matching a motor's roughly linear current-vs-shaft-power characteristic
/// over its normal operating range (the same "current tracks mechanical
/// power, not added torque toward stall" physics as
/// [`mechanical_current_multiplier`], applied to *reduced* rather than
/// *increased* load). `hydraulic_power_fraction` above 1.0 (a pump briefly
/// loaded past its rated point) is clamped to 1.0 x rated current here; the
/// breaker's own I^2t curve, not this function, is what should answer a
/// genuine overload.
///
/// Check: `hydraulic_power_fraction = 1.0` (healthy, rated delivery) gives
/// exactly `1.0` (rated current, no derate); `hydraulic_power_fraction =
/// 0.0` (fully vapor-locked, no fluid moved at all) gives
/// [`TYPICAL_NO_LOAD_CURRENT_FRACTION`] (no-load current only) --
/// monotonic and continuous in between, no bucket.
pub fn hydraulic_power_current_multiplier(hydraulic_power_fraction: f64) -> f64 {
    let f = hydraulic_power_fraction.max(0.0).min(1.0);
    TYPICAL_NO_LOAD_CURRENT_FRACTION + (1.0 - TYPICAL_NO_LOAD_CURRENT_FRACTION) * f
}

/// Process-global "latest published *delivered hydraulic power fraction*"
/// registry for [`hydraulic_power_current_multiplier`] -- kept separate from
/// [`LOAD_FRACTIONS`] (an *added mechanical load*, opposite sign) so the two
/// couplings can never be summed under the same breaker id by accident. An
/// id with no published value reads as `1.0` (rated delivery, no derate) --
/// the neutral value for this registry, unlike `LOAD_FRACTIONS`'s neutral
/// `0.0`, since a breaker nothing has ever coupled this way must draw
/// exactly its normal rated-current estimate, not the no-load floor.
static HYDRAULIC_POWER_FRACTIONS: Mutex<Option<HashMap<String, f64>>> = Mutex::new(None);

/// Publish `id`'s current delivered-hydraulic-power fraction (0.0..=1.0,
/// clamped; values above 1.0 clamp to 1.0, matching
/// [`hydraulic_power_current_multiplier`]'s own clamp), overwriting any
/// previous value. Call once per tick per affected breaker id.
pub fn publish_hydraulic_power_fraction(id: &str, fraction: f64) {
    let mut guard = HYDRAULIC_POWER_FRACTIONS.lock().unwrap_or_else(|e| e.into_inner());
    let map = guard.get_or_insert_with(HashMap::new);
    map.insert(id.to_owned(), fraction.clamp(0.0, 1.0));
}

/// `id`'s last published delivered-hydraulic-power fraction, or `1.0`
/// (rated, no derate) if nothing has ever published one for it.
pub fn hydraulic_power_fraction(id: &str) -> f64 {
    let guard = HYDRAULIC_POWER_FRACTIONS.lock().unwrap_or_else(|e| e.into_inner());
    guard.as_ref().and_then(|m| m.get(id)).copied().unwrap_or(1.0)
}

/// Process-global "latest published load fraction" registry, the same
/// pattern `wear.rs`'s own module doc describes ("a process-global 'latest'
/// published each tick"): a physics module that computes a continuous
/// mechanical load fraction (e.g. `fuel.rs`'s cavitation coupling) publishes
/// it here by the *breaker catalogue id* it affects, and `breakers.rs`
/// reads it back at `post_systems` current-computation time without needing
/// a direct dependency on the publishing module (`breakers.rs` already
/// cannot see `FuelNetwork`/`Fuel` instances at that point). Ids with no
/// published value read as `0.0` (no extra load) -- a coupling that has
/// never run yet must never silently inflate a breaker's current.
static LOAD_FRACTIONS: Mutex<Option<HashMap<String, f64>>> = Mutex::new(None);

/// Publish `id`'s current mechanical-load fraction (0.0..=1.0, clamped),
/// overwriting any previous value for the same id. Call once per tick per
/// affected breaker id.
pub fn publish_load_fraction(id: &str, load_fraction: f64) {
    let mut guard = LOAD_FRACTIONS.lock().unwrap_or_else(|e| e.into_inner());
    let map = guard.get_or_insert_with(HashMap::new);
    map.insert(id.to_owned(), load_fraction.clamp(0.0, 1.0));
}

/// `id`'s last published mechanical-load fraction, or `0.0` if nothing has
/// ever published one for it.
pub fn load_fraction(id: &str) -> f64 {
    let guard = LOAD_FRACTIONS.lock().unwrap_or_else(|e| e.into_inner());
    guard.as_ref().and_then(|m| m.get(id)).copied().unwrap_or(0.0)
}

/// Test-isolation helper (see `scenarios::reset_global_state`): clears every
/// published load fraction so one test's coupling cannot leak into the
/// next's.
#[cfg(any(test, feature = "test-support"))]
pub fn reset_for_tests() {
    let mut guard = LOAD_FRACTIONS.lock().unwrap_or_else(|e| e.into_inner());
    *guard = None;
    let mut guard = HYDRAULIC_POWER_FRACTIONS.lock().unwrap_or_else(|e| e.into_inner());
    *guard = None;
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Independent prediction from the back-EMF algebra in the module doc,
    /// hand-computed (not re-derived from the function under test): at
    /// `w=0.5`, `m=6.0`, `I = 1 + (6-1)*0.5 = 3.5`x rated.
    #[test]
    fn mechanical_multiplier_matches_hand_computed_back_emf_prediction() {
        let got = mechanical_current_multiplier(0.5);
        assert!((got - 3.5).abs() < 1e-9, "got {got}, hand-computed prediction is 3.5x rated at w=0.5, m=6");
    }

    #[test]
    fn mechanical_multiplier_is_1x_at_no_load_and_locked_rotor_multiple_at_full_seizure() {
        assert!((mechanical_current_multiplier(0.0) - 1.0).abs() < 1e-9);
        assert!((mechanical_current_multiplier(1.0) - TYPICAL_LOCKED_ROTOR_MULTIPLE).abs() < 1e-9);
    }

    /// Decouple check: removing the load fraction (the physical cause)
    /// removes the current rise entirely, proving the relation is a real
    /// function of the input, not a constant.
    #[test]
    fn mechanical_multiplier_decouple_check_removing_load_removes_the_rise() {
        let loaded = mechanical_current_multiplier(0.8);
        let healthy = mechanical_current_multiplier(0.0);
        assert!(loaded > healthy, "a real mechanical load must raise current above the healthy baseline");
        assert!((healthy - 1.0).abs() < 1e-9, "with the cause removed, current must return exactly to rated (1x)");
    }

    /// Independent prediction from the partial-inter-turn-short algebra:
    /// at `health=0.4`, `Z = 0.4 + 0.6*0.15 = 0.49`, `I multiplier =
    /// 1/0.49 = 2.0408...`.
    #[test]
    fn insulation_multiplier_matches_hand_computed_impedance_prediction() {
        let got = insulation_current_multiplier(0.4);
        let want = 1.0 / 0.49;
        assert!((got - want).abs() < 1e-9, "got {got}, hand-computed prediction is {want}");
    }

    #[test]
    fn insulation_multiplier_decouple_check_healthy_winding_draws_rated_current() {
        let broken_down = insulation_current_multiplier(0.1);
        let healthy = insulation_current_multiplier(1.0);
        assert!(broken_down > healthy, "a real partial short must raise current above the healthy baseline");
        assert!((healthy - 1.0).abs() < 1e-9, "with the insulation fault removed, current must return exactly to rated (1x)");
    }

    /// Independent hand computation at `f=0.4`: `0.3 + 0.7*0.4 = 0.58`.
    #[test]
    fn hydraulic_power_multiplier_matches_hand_computed_linear_prediction() {
        let got = hydraulic_power_current_multiplier(0.4);
        assert!((got - 0.58).abs() < 1e-9, "got {got}, hand-computed prediction is 0.58");
    }

    #[test]
    fn hydraulic_power_multiplier_is_rated_at_full_delivery_and_no_load_floor_at_zero() {
        assert!((hydraulic_power_current_multiplier(1.0) - 1.0).abs() < 1e-9);
        assert!((hydraulic_power_current_multiplier(0.0) - TYPICAL_NO_LOAD_CURRENT_FRACTION).abs() < 1e-9);
    }

    /// The physical direction check this coupling exists for: a cavitating
    /// pump delivering less hydraulic power must draw *less* current than a
    /// healthy one, never more (the bug this module's doc section
    /// describes: an earlier pass fed cavitation into the *mechanical-load*
    /// coupling and made current rise toward locked rotor instead).
    #[test]
    fn hydraulic_power_multiplier_falls_as_delivered_power_falls_never_rising_toward_locked_rotor() {
        let healthy = hydraulic_power_current_multiplier(1.0);
        let cavitating = hydraulic_power_current_multiplier(0.2);
        let vapor_locked = hydraulic_power_current_multiplier(0.0);
        assert!(cavitating < healthy, "reduced hydraulic delivery must draw less current, not more");
        assert!(vapor_locked < cavitating);
        assert!(vapor_locked < TYPICAL_LOCKED_ROTOR_MULTIPLE, "cavitation current must never approach a locked-rotor multiple");
    }

    #[test]
    fn hydraulic_power_fraction_registry_publishes_reads_back_defaults_to_rated_and_resets() {
        reset_for_tests();
        // Neutral default is 1.0 (rated, no derate) -- opposite of
        // LOAD_FRACTIONS's 0.0 default, since an unpublished id must draw
        // its normal rated-current estimate, not the no-load floor.
        assert_eq!(hydraulic_power_fraction("never-published"), 1.0);
        publish_hydraulic_power_fraction("pump-id", 0.42);
        assert!((hydraulic_power_fraction("pump-id") - 0.42).abs() < 1e-9);
        reset_for_tests();
        assert_eq!(hydraulic_power_fraction("pump-id"), 1.0, "reset must restore the neutral default");
    }

    #[test]
    fn load_fraction_registry_publishes_reads_back_and_resets() {
        reset_for_tests();
        assert_eq!(load_fraction("never-published"), 0.0);
        publish_load_fraction("test-id", 0.42);
        assert!((load_fraction("test-id") - 0.42).abs() < 1e-9);
        reset_for_tests();
        assert_eq!(load_fraction("test-id"), 0.0, "reset must clear previously published values");
    }
}
