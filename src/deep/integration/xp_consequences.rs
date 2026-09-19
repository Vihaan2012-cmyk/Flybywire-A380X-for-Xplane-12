//! Aerodynamic/structural consequences other areas' models already compute
//! (ice mass/shape drag penalty, bird-strike/hail leading-edge damage, a
//! collapsed gear leg) turned into what X-Plane can actually express.
//!
//! ## What X-Plane exposes, and which consequence uses which
//! - **Force/moment injection**: `sim/flightmodel/forces/{fside,fnrml,
//!   faxil}_plug_acf` (N) and `{L,M,N}_plug_acf` (N*m), the SDK's own
//!   plugin-force mechanism (`developer.x-plane.com/article/
//!   movingtheplane`, X-Plane 10.30+): body axes, origin at the aircraft's
//!   CG, positive `faxil` aft (so drag is positive, thrust negative --
//!   confirmed by that article's own worked example), positive `L` right
//!   roll, positive `M` nose up, positive `N` clockwise from above (yaw
//!   right). **X-Plane resets every one of these to zero every frame**;
//!   a plugin must read-add-write every tick, which is why every function
//!   below returns a **delta**, applied through [`add_plug_force`] rather
//!   than ever calling `xplm.set_f` with an absolute value. This is a
//!   different, more physically direct mechanism than
//!   `extra_backend_fbw.rs::apply_reverser_thrust`'s velocity/yaw-rate
//!   nudge (that file predates this documentation search and does not use
//!   it -- not this module's file to change); ice drag and a dragging
//!   collapsed gear leg are genuinely *forces*, not a one-off displacement,
//!   so the plug-force path is the correct one for them.
//! - **Per-surface visible damage** (bird-strike windshield/radome cracks,
//!   nose-gear damage): no X-Plane dataref expresses "this specific panel
//!   is cracked" -- these stay avionics/systems-level facts (weather radar
//!   loss, visibility loss) with no airframe-force consequence of their
//!   own; only their *drag* (`StrikeOutcome.leading_edge_dent_drag_delta_cd`)
//!   is aerodynamic and handled the same way ice drag is, below.
//! - **A collapsed gear leg**: no X-Plane dataref represents "this strut is
//!   structurally broken" either. [`gear_deploy_override`] retracts that
//!   one gear's `sim/flightmodel2/gear/deploy_ratio` element instead, the
//!   closest real consequence available -- X-Plane then computes *zero*
//!   ground reaction there on its own (the same way a genuinely retracted
//!   gear gets none), so the corner of the airframe that leg was holding
//!   up has nothing supporting it and the aircraft settles/tips under its
//!   own weight exactly as a real collapse would, from X-Plane's own
//!   physics rather than a scripted animation. The honest caveat: this
//!   *looks* like a retracted gear (the wheel disappears) rather than a
//!   visibly bent/dragging strut, since the SDK has nothing better; the
//!   still-dragging-strut drag/yaw force while on the ground is added
//!   separately via the plug-force path so the *physics* stay right even
//!   though the *visual* is imperfect.

use crate::deep::gear_structure::LegOutput;
use crate::xp::{DataRef, Xplm};

// ---------------------------------------------------------------------------
// The plug-force mechanism: read-add-write every tick (X-Plane zeroes these
// every frame -- module doc).
// ---------------------------------------------------------------------------

/// Adds `delta` to a plugin-force dataref (one of the six named in the
/// module doc) and writes the result back, the mandatory pattern for a
/// dataref X-Plane resets to zero every frame. Safe to call from more than
/// one consequence source per tick on the same `dataref` (each call reads
/// what the previous one just wrote), and a no-op if `dataref` is `None`
/// (older SDK target, or the offline test harness).
pub fn add_plug_force(xplm: &Xplm, dataref: Option<DataRef>, delta: f64) {
    if let Some(d) = dataref {
        xplm.set_f(d, (xplm.get_f(d) as f64 + delta) as f32);
    }
}

/// Handles to all six plugin-force datarefs, looked up once.
pub struct PlugForceRefs {
    pub fside: Option<DataRef>,
    pub fnrml: Option<DataRef>,
    pub faxil: Option<DataRef>,
    pub roll: Option<DataRef>,
    pub pitch: Option<DataRef>,
    pub yaw: Option<DataRef>,
}

impl PlugForceRefs {
    pub fn new(xplm: &Xplm) -> Self {
        Self {
            fside: xplm.find("sim/flightmodel/forces/fside_plug_acf"),
            fnrml: xplm.find("sim/flightmodel/forces/fnrml_plug_acf"),
            faxil: xplm.find("sim/flightmodel/forces/faxil_plug_acf"),
            roll: xplm.find("sim/flightmodel/forces/L_plug_acf"),
            pitch: xplm.find("sim/flightmodel/forces/M_plug_acf"),
            yaw: xplm.find("sim/flightmodel/forces/N_plug_acf"),
        }
    }
}

// ---------------------------------------------------------------------------
// Ice / dent drag: the one aerodynamic consequence common to
// `fire_ice::icing::IcingOutputs.cd_increase_fraction` and
// `environment::bird_strike::StrikeOutcome.leading_edge_dent_drag_delta_cd`
// -- both are, physically, an extra profile-drag coefficient on some
// reference area, so one function serves both.
// ---------------------------------------------------------------------------

/// Standard incompressible dynamic pressure, Pa, from ambient density
/// (ideal-gas law: `rho = P/(R*T)`, `R` dry air's specific gas constant)
/// and true airspeed -- computed here rather than read from a `Var`
/// because `"AMBIENT DENSITY"` (`src/lib.rs:644`) is already converted to
/// MSFS's own slug/ft^3 simvar convention for FlyByWire's benefit, and
/// re-converting back would be a needless round trip through the wrong
/// native unit for an SI-internal crate.
pub fn dynamic_pressure_pa(ambient_pressure_pa: f64, sat_c: f64, tas_ms: f64) -> f64 {
    const R_AIR_J_KGK: f64 = 287.052_87;
    let t_k = (sat_c + 273.15).max(1.0);
    let rho = ambient_pressure_pa.max(0.0) / (R_AIR_J_KGK * t_k);
    0.5 * rho * tas_ms.max(0.0).powi(2)
}

/// A380 public reference wing area, m^2 (Airbus's own published "Aircraft
/// Characteristics -- Airport and Maintenance Planning" figure, widely
/// repeated: 845 m^2).
pub const A380_WING_REFERENCE_AREA_M2: f64 = 845.0;

/// Extra drag force, N, from a profile-drag-coefficient increment on the
/// reference wing area (`D = q * S * dCd`, the standard drag equation).
pub fn extra_drag_force_n(delta_cd: f64, dynamic_pressure_pa: f64, reference_area_m2: f64) -> f64 {
    delta_cd.max(0.0) * dynamic_pressure_pa.max(0.0) * reference_area_m2.max(0.0)
}

/// Yaw moment, N*m, from left/right wings carrying different drag
/// increments (asymmetric icing, or a dent on one side only): each side's
/// extra drag acts at its own spanwise centre of pressure, `moment_arm_m`
/// out from the centreline (GENERIC: half of `A380_WING_REFERENCE_AREA_M2`'s
/// implied span is not the right figure for a drag centroid, so this takes
/// the arm as a parameter rather than assuming one -- callers with a real
/// aerodynamic model should pass the actual spanwise drag centroid).
/// Positive output yaws right (matches `N_plug_acf`'s own sign, module
/// doc), which is correct when the *left* side carries the extra drag
/// (drag aft on the left wing yaws the nose right).
pub fn asymmetric_drag_yaw_moment_nm(left_delta_cd: f64, right_delta_cd: f64, dynamic_pressure_pa: f64, reference_area_m2: f64, moment_arm_m: f64) -> f64 {
    let left_n = extra_drag_force_n(left_delta_cd, dynamic_pressure_pa, reference_area_m2 * 0.5);
    let right_n = extra_drag_force_n(right_delta_cd, dynamic_pressure_pa, reference_area_m2 * 0.5);
    (left_n - right_n) * moment_arm_m.max(0.0)
}

// ---------------------------------------------------------------------------
// A collapsed gear leg (module doc for the reasoning).
// ---------------------------------------------------------------------------

/// GENERIC: sliding friction of a bare strut/wheel-well structure on
/// pavement, well above a rolling tyre's typical ~0.02 rolling-resistance
/// coefficient (a broken strut is dragging metal, not rolling on a tyre) but
/// below a locked/skidding rubber tyre's ~0.5-0.8 (no rubber contact patch
/// left at all once the leg has collapsed) -- a mid-range representative
/// figure, not a measured one.
pub const COLLAPSED_LEG_SLIDING_FRICTION: f64 = 0.35;

/// Extra drag force, N, from a collapsed leg still in ground contact and
/// dragging: friction coefficient times the load that leg *would* be
/// carrying (`Strut::static_fraction()` of the aircraft's weight, already
/// modelled by `gear_structure` -- passed in here rather than recomputed,
/// since a collapsed leg's own `force_n` output has already dropped to
/// whatever the buckled structure can still react, not the load it used to
/// carry, which is what actually determines drag against the ground: the
/// aircraft's weight is still there, now reacted partly by scraping
/// structure). Zero whenever the leg is not both collapsed and on the
/// ground (airborne, or an intact leg with its own normal rolling/braking
/// drag already handled elsewhere).
pub fn collapsed_leg_drag_force_n(leg: &LegOutput, static_load_share_n: f64, on_ground: bool) -> f64 {
    if !leg.collapsed || !on_ground {
        return 0.0;
    }
    COLLAPSED_LEG_SLIDING_FRICTION * static_load_share_n.max(0.0)
}

/// Overrides one gear's `sim/flightmodel2/gear/deploy_ratio` element to 0
/// once its `LegOutput.collapsed` is true (module doc: X-Plane then
/// computes zero ground reaction there on its own). `index` is that
/// dataref's own gear ordering (0 nose, then the four main legs in the
/// `.acf`'s own order -- left wing/right wing/left body/right body for the
/// converted A380X, matching `flight_controls.rs`'s own `WING1..4`
/// left/right convention).
pub fn gear_deploy_override(xplm: &Xplm, deploy_ratio: Option<DataRef>, index: usize, leg: &LegOutput) {
    if leg.collapsed {
        if let Some(d) = deploy_ratio {
            xplm.set_vf_at(d, index, 0.0);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dynamic_pressure_is_zero_at_rest_and_grows_with_the_square_of_tas() {
        assert_eq!(dynamic_pressure_pa(101_325.0, 15.0, 0.0), 0.0);
        let low = dynamic_pressure_pa(101_325.0, 15.0, 100.0);
        let high = dynamic_pressure_pa(101_325.0, 15.0, 200.0);
        assert!((high - 4.0 * low).abs() / high < 1e-9);
    }

    #[test]
    fn dynamic_pressure_matches_isa_sea_level_order_of_magnitude() {
        // ISA sea level rho ~ 1.225 kg/m^3; q at 100 m/s should be close to
        // 0.5 * 1.225 * 100^2 = 6125 Pa.
        let q = dynamic_pressure_pa(101_325.0, 15.0, 100.0);
        assert!((q - 6125.0).abs() < 50.0, "{q}");
    }

    #[test]
    fn extra_drag_force_scales_linearly_with_cd_and_is_never_negative() {
        let q = 5000.0;
        let d = extra_drag_force_n(0.01, q, A380_WING_REFERENCE_AREA_M2);
        assert!((d - 0.01 * q * A380_WING_REFERENCE_AREA_M2).abs() < 1e-6);
        assert_eq!(extra_drag_force_n(-0.5, q, A380_WING_REFERENCE_AREA_M2), 0.0, "a negative dCd must never produce thrust");
    }

    #[test]
    fn symmetric_icing_produces_no_yaw_moment() {
        let m = asymmetric_drag_yaw_moment_nm(0.02, 0.02, 5000.0, A380_WING_REFERENCE_AREA_M2, 15.0);
        assert!((m).abs() < 1e-9);
    }

    #[test]
    fn heavier_ice_on_the_left_wing_yaws_right() {
        let m = asymmetric_drag_yaw_moment_nm(0.05, 0.0, 5000.0, A380_WING_REFERENCE_AREA_M2, 15.0);
        assert!(m > 0.0, "extra left-wing drag should yaw the nose right (positive N_plug_acf)");
    }

    fn leg(collapsed: bool) -> LegOutput {
        LegOutput { collapsed, ..Default::default() }
    }

    #[test]
    fn an_intact_leg_or_an_airborne_collapsed_leg_adds_no_drag() {
        assert_eq!(collapsed_leg_drag_force_n(&leg(false), 200_000.0, true), 0.0);
        assert_eq!(collapsed_leg_drag_force_n(&leg(true), 200_000.0, false), 0.0);
    }

    #[test]
    fn a_collapsed_leg_on_the_ground_drags_proportionally_to_its_static_load_share() {
        let d = collapsed_leg_drag_force_n(&leg(true), 200_000.0, true);
        assert!((d - COLLAPSED_LEG_SLIDING_FRICTION * 200_000.0).abs() < 1e-6);
        assert!(d > 0.0);
    }

    #[test]
    fn add_plug_force_is_a_no_op_without_a_live_dataref() {
        // Exercises the `None` branch (offline/test harness, or an SDK
        // target predating X-Plane 10.30) without needing a live `Xplm`.
        let xplm = Xplm::dummy();
        add_plug_force(&xplm, None, 1234.0); // must not panic
    }
}
